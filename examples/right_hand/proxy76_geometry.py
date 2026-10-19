"""Static surface metrics for the actual union of convex collision parts.

All input triangles must already share one link frame, in metres. Each part is
convexified independently, as mesh collision does; this is analysis, not convex
decomposition. No MuJoCo state or dynamics is used. The proxy boundary is made
by subtracting other convex solids from every convex face, not by measuring
against hidden overlap faces. Distances are continuous point-to-triangle queries;
only the distribution of query points is finitely sampled.
"""
import importlib.util
from pathlib import Path

import numpy as np
from scipy.spatial import ConvexHull, cKDTree


CLIP_TOL_M = 1e-10
QUERY_TOL_M = 1e-12


def _geometry():
    path = Path(__file__).resolve().parent / 'evidence/d3_geometry/triangle/surface_geometry.py'
    spec = importlib.util.spec_from_file_location('proxy76_source_geometry', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _triangles(values):
    result = np.asarray(values, dtype=np.float64)
    if result.ndim != 3 or result.shape[1:] != (3, 3) or not len(result):
        raise ValueError('Expected nonempty Nx3x3 triangles in metres')
    if not np.all(np.isfinite(result)):
        raise ValueError('Nonfinite triangle coordinates')
    return result


def _areas(triangles):
    return np.linalg.norm(np.cross(triangles[:, 1]-triangles[:, 0],
                                   triangles[:, 2]-triangles[:, 0]), axis=1)/2


def _point_triangle_pairs(points, triangles):
    """Nearest points on equal-length independent continuous triangle pairs."""
    a, b, c = triangles[:, 0], triangles[:, 1], triangles[:, 2]
    ab, ac = b-a, c-a
    normal = np.cross(ab, ac)
    norm2 = np.einsum('ij,ij->i', normal, normal)
    signed = np.einsum('ij,ij->i', points-a, normal)
    projected = points-normal*np.divide(signed, norm2, out=np.zeros_like(signed),
                                        where=norm2 > 0)[:, None]
    v = projected-a
    u = np.divide(np.einsum('ij,ij->i', np.cross(v, ac), normal), norm2,
                  out=np.full_like(norm2, np.nan), where=norm2 > 0)
    w = np.divide(np.einsum('ij,ij->i', np.cross(ab, v), normal), norm2,
                  out=np.full_like(norm2, np.nan), where=norm2 > 0)
    inside = (u >= 0) & (w >= 0) & (u+w <= 1) & (norm2 > 0)
    best = np.where(inside, np.sum((points-projected)**2, axis=1), np.inf)
    nearest = projected.copy()
    # Edges include all vertices and handle zero-area/zero-length input faces.
    for start, end in ((a, b), (b, c), (c, a)):
        edge = end-start
        length2 = np.einsum('ij,ij->i', edge, edge)
        parameter = np.clip(np.divide(np.einsum('ij,ij->i', points-start, edge),
                                     length2, out=np.zeros_like(length2),
                                     where=length2 > 0), 0, 1)
        closest = start+parameter[:, None]*edge
        d2 = np.sum((points-closest)**2, axis=1)
        take = d2 < best
        best[take], nearest[take] = d2[take], closest[take]
    return best, nearest


class TriangleSurface:
    """Reuse the existing source-surface BVH for batched exact nearest queries."""

    def __init__(self, triangles):
        self.triangles = _triangles(triangles)
        self.bvh = _geometry().BVH(self.triangles)
        self.vertices = self.triangles.reshape(-1, 3)
        self.tree = cKDTree(self.vertices)

    def nearest(self, points):
        points = np.asarray(points, dtype=np.float64).reshape(-1, 3)
        if not np.all(np.isfinite(points)):
            raise ValueError('Nonfinite query points')
        if not len(points):
            return np.empty(0), np.empty((0, 3)), np.empty(0, dtype=int)
        initial, vertex = self.tree.query(points)
        best = initial**2
        nearest = self.vertices[vertex].copy()
        face = vertex//3
        pending = [(0, np.arange(len(points)))]
        while pending:
            node_id, point_ids = pending.pop()
            lo, hi, left, right, triangle_ids = self.bvh.nodes[node_id]
            delta = np.maximum(0, np.maximum(lo-points[point_ids], points[point_ids]-hi))
            lower = np.einsum('ij,ij->i', delta, delta)
            limit = best[point_ids]+2*np.sqrt(best[point_ids])*QUERY_TOL_M+QUERY_TOL_M**2
            point_ids = point_ids[lower <= limit]
            if not len(point_ids):
                continue
            if triangle_ids is None:
                pending.extend(((right, point_ids), (left, point_ids)))
                continue
            chunk_size = max(1, 32768//len(triangle_ids))
            for start in range(0, len(point_ids), chunk_size):
                ids = point_ids[start:start+chunk_size]
                pairs = np.repeat(points[ids], len(triangle_ids), axis=0)
                tris = np.tile(self.triangles[triangle_ids], (len(ids), 1, 1))
                d2, closest = _point_triangle_pairs(pairs, tris)
                d2 = d2.reshape(len(ids), len(triangle_ids))
                chosen = np.argmin(d2, axis=1)
                rows = np.arange(len(ids))
                take = d2[rows, chosen] < best[ids]
                targets = ids[take]
                best[targets] = d2[rows[take], chosen[take]]
                nearest[targets] = closest.reshape(len(ids), len(triangle_ids), 3)[rows[take], chosen[take]]
                face[targets] = triangle_ids[chosen[take]]
        return np.sqrt(np.maximum(best, 0)), nearest, face


def _clean_polygon(polygon, tolerance):
    if len(polygon) < 3:
        return None
    result = [polygon[0]]
    for vertex in polygon[1:]:
        if np.linalg.norm(vertex-result[-1]) > tolerance:
            result.append(vertex)
    if len(result) > 1 and np.linalg.norm(result[0]-result[-1]) <= tolerance:
        result.pop()
    return np.asarray(result) if len(result) >= 3 else None


def _split_polygon(polygon, normal, offset, tolerance):
    """Split at a convex halfspace; near-plane vertices snap symbolically to 0."""
    distances = polygon@normal+offset
    distances[np.abs(distances) <= tolerance] = 0
    if np.all(distances <= 0):
        return polygon, None
    if np.all(distances >= 0):
        return None, polygon
    inside, outside = [], []
    for k, vertex in enumerate(polygon):
        following = polygon[(k+1) % len(polygon)]
        a, b = distances[k], distances[(k+1) % len(polygon)]
        if a <= 0:
            inside.append(vertex)
        if a >= 0:
            outside.append(vertex)
        if a*b < 0:
            cross = vertex+(a/(a-b))*(following-vertex)
            inside.append(cross)
            outside.append(cross)
    return (_clean_polygon(inside, tolerance), _clean_polygon(outside, tolerance))


def _subtract_convex(polygon, equations, tolerance):
    # Each emitted piece fails one halfspace while satisfying all preceding
    # ones. Interiors of emitted pieces are disjoint; the final inside is removed.
    values = polygon@equations[:, :3].T+equations[:, 3]
    if np.any(np.all(values > tolerance, axis=0)):
        return [polygon]
    remaining, result = polygon, []
    for equation in equations:
        remaining, outside = _split_polygon(remaining, equation[:3], equation[3], tolerance)
        if outside is not None:
            result.append(outside)
        if remaining is None:
            break
    return result


class ConvexUnion:
    def __init__(self, triangles, face_parts, hulls, diagnostics, tolerance):
        self.triangles = triangles
        self.face_parts = np.asarray(face_parts, dtype=int)
        self.hulls = hulls
        self.diagnostics = diagnostics
        self.tolerance = tolerance
        self.surface = TriangleSurface(triangles)

    def membership_margin(self, points):
        """min_part(max_plane(n.p+b)); sign is membership, not true distance."""
        points = np.asarray(points, dtype=np.float64).reshape(-1, 3)
        margins = np.full(len(points), np.inf)
        for hull in self.hulls:
            equations = hull['equations']
            for start in range(0, len(points), 4096):
                values = points[start:start+4096]@equations[:, :3].T+equations[:, 3]
                margins[start:start+4096] = np.minimum(margins[start:start+4096], values.max(axis=1))
        return margins

    def contains(self, points):
        return self.membership_margin(points) <= self.tolerance


def convex_union_surface(proxy_parts, *, tolerance_m=CLIP_TOL_M):
    """Float64 exposed union boundary, after independent part convexification.

    Convex face polygons are sequentially subtracted by every overlapping other
    convex solid. Coincident same-outward planes retain the earlier part's face
    where they overlap. Opposite-outward touching interfaces are removed from
    both parts. The tolerance is numerical, not an estimated modelling error.
    """
    if not proxy_parts:
        raise ValueError('At least one collision part is required')
    hulls = []
    for i, part in enumerate(proxy_parts):
        vertices = np.unique(_triangles(part['triangles']).reshape(-1, 3), axis=0)
        hull = ConvexHull(vertices)
        triangles = vertices[hull.simplices].copy()
        wrong = np.einsum('ij,ij->i', np.cross(triangles[:, 1]-triangles[:, 0],
                                             triangles[:, 2]-triangles[:, 0]),
                          hull.equations[:, :3]) < 0
        triangles[wrong] = triangles[wrong][:, [0, 2, 1]]
        hulls.append({'name': str(part.get('name', i)), 'triangles': triangles,
                      'equations': hull.equations, 'lo': vertices.min(axis=0),
                      'hi': vertices.max(axis=0), 'input_vertices': len(vertices),
                      'volume_m3': float(hull.volume)})
    exposed, face_parts = [], []
    discarded_area = 0.
    face_count = sum(len(h['triangles']) for h in hulls)
    for part_id, hull in enumerate(hulls):
        for face_id, triangle in enumerate(hull['triangles']):
            pieces = [triangle]
            normal, offset = hull['equations'][face_id, :3], hull['equations'][face_id, 3]
            face_lo, face_hi = triangle.min(axis=0), triangle.max(axis=0)
            for other_id, other in enumerate(hulls):
                if part_id == other_id or np.any(face_hi < other['lo']-tolerance_m) or np.any(face_lo > other['hi']+tolerance_m):
                    continue
                equations = other['equations']
                same_outward = ((np.max(np.abs(equations[:, :3]-normal), axis=1) <= 1e-10)
                                 & (np.abs(equations[:, 3]-offset) <= tolerance_m))
                if part_id < other_id and np.any(same_outward):
                    continue
                pieces = [piece for polygon in pieces
                          for piece in _subtract_convex(polygon, equations, tolerance_m)]
                if not pieces:
                    break
                if len(pieces) > 100000:
                    raise RuntimeError('Union face clipping fragment limit exceeded; no approximate fallback')
            for polygon in pieces:
                for k in range(1, len(polygon)-1):
                    tri = np.array([polygon[0], polygon[k], polygon[k+1]])
                    area = _areas(tri[None])[0]
                    if area > tolerance_m**2:
                        exposed.append(tri)
                        face_parts.append(part_id)
                    else:
                        discarded_area += area
    if not exposed:
        raise ValueError('No exposed boundary remained after convex union clipping')
    triangles = np.asarray(exposed)
    diagnostics = {'part_count': len(hulls), 'independent_convex_hull_faces': face_count,
                   'exposed_boundary_triangles': len(triangles),
                   'exposed_boundary_area_m2': float(_areas(triangles).sum()),
                   'discarded_degenerate_fragment_area_m2': float(discarded_area),
                   'clipping_tolerance_m': tolerance_m,
                   'coplanar_unit_normal_component_tolerance': 1e-10,
                   'hidden_overlap_faces_excluded': True,
                   'boundary_method': 'convex-face polygon subtraction by all other convex halfspaces; float64',
                   'parts': [{'name': h['name'], 'input_unique_vertices': h['input_vertices'],
                              'convex_hull_triangles': len(h['triangles']), 'volume_m3': h['volume_m3']}
                             for h in hulls]}
    return ConvexUnion(triangles, face_parts, hulls, diagnostics, tolerance_m)


def _sample_surface(triangles, count, seed):
    areas = _areas(triangles)
    total = areas.sum()
    if total <= 0 or count < 1:
        raise ValueError('Positive source surface area and sample count are required')
    rng = np.random.default_rng(seed)
    cumulative = np.cumsum(areas)
    positions = (np.arange(count)+rng.random(count))*total/count
    face_ids = np.searchsorted(cumulative, positions, side='right')
    values = rng.random((count, 2))
    root = np.sqrt(values[:, 0])
    bary = np.column_stack((1-root, root*(1-values[:, 1]), root*values[:, 1]))
    return np.einsum('ni,nij->nj', bary, triangles[face_ids]), face_ids


def _summarize(points, distances, nearest, faces, margins=None, face_parts=None, hulls=None,
               target_triangles=None):
    if not len(points):
        return {'sample_count': 0, 'p95_m': None, 'p99_m': None, 'max_m': None,
                'max_witness': None, 'status': 'UNSAMPLED'}
    k = int(np.argmax(distances))
    witness = {'query_point_m': points[k].tolist(), 'nearest_surface_point_m': nearest[k].tolist(),
               'nearest_triangle_id': int(faces[k]), 'distance_m': float(distances[k])}
    if target_triangles is not None:
        witness['nearest_triangle_vertices_m'] = target_triangles[faces[k]].tolist()
    if face_parts is not None:
        witness['nearest_proxy_part'] = hulls[face_parts[faces[k]]]['name']
    result = {'sample_count': len(points), 'p95_m': float(np.quantile(distances, .95)),
              'p99_m': float(np.quantile(distances, .99)), 'max_m': float(distances[k]),
              'max_witness': witness, 'status': 'FINITE_SAMPLE_ONLY'}
    if margins is not None:
        outside = margins > CLIP_TOL_M
        # Touching parts can make an internal interface a boundary of every
        # individual hull. Union interior therefore uses distance to the exposed
        # union boundary, not a requirement to be strictly inside one part.
        strict_inside = ~outside & (distances > CLIP_TOL_M)
        boundary = ~outside & ~strict_inside
        signed = np.where(strict_inside, -distances, np.where(outside, distances, 0.))
        witness['proxy_membership'] = ('strictly_inside' if strict_inside[k] else
                                        'outside' if outside[k] else 'boundary_tolerance_band')
        witness['signed_distance_to_proxy_union_m'] = float(signed[k])
        result.update({'source_points_strictly_inside_proxy_count': int(strict_inside.sum()),
                       'source_points_outside_proxy_count': int(outside.sum()),
                       'source_points_boundary_band_count': int(boundary.sum()),
                       'source_points_inside_or_boundary_fraction': float((~outside).mean()),
                       'source_points_outside_fraction': float(outside.mean()),
                       'signed_distance_min_m': float(signed.min()),
                       'signed_distance_p05_m': float(np.quantile(signed, .05)),
                       'signed_distance_p50_m': float(np.quantile(signed, .5)),
                       'signed_distance_p95_m': float(np.quantile(signed, .95)),
                       'signed_distance_max_m': float(signed.max()),
                       'signed_distance_convention': 'negative inside proxy union, positive outside; zero within exposed-boundary tolerance band'})
        for label, mask in [('buried_source', strict_inside), ('uncovered_source', outside)]:
            ids = np.flatnonzero(mask)
            if len(ids):
                j = int(ids[np.argmax(distances[ids])])
                result[label+'_max_witness'] = {'query_point_m': points[j].tolist(),
                                                'nearest_surface_point_m': nearest[j].tolist(),
                                                'nearest_triangle_id': int(faces[j]),
                                                'distance_m': float(distances[j]),
                                                'signed_distance_m': float(signed[j])}
                if target_triangles is not None:
                    result[label+'_max_witness']['nearest_triangle_vertices_m'] = target_triangles[faces[j]].tolist()
            else:
                result[label+'_max_witness'] = None
    return result


def _region_mask(points, region):
    if 'axis' in region:
        values = points[:, int(region['axis'])]
        return (values >= region.get('min', -np.inf)) & (values <= region.get('max', np.inf))
    return (np.all(points >= np.asarray(region.get('min', [-np.inf]*3)), axis=1)
            & np.all(points <= np.asarray(region.get('max', [np.inf]*3)), axis=1))


def analyze_link(source_triangles_m, proxy_parts, *, samples_per_direction=4096,
                 seed=760, regions=None, include_vertex_extrema=True):
    """Return serializable bidirectional metrics in a common link-local frame.

    Quantiles use deterministic stratified triangle-area samples. Independent
    all-unique-vertex checks can find additional extrema; their quantiles are not
    surface-area statistics. Neither check certifies a global Hausdorff maximum.
    Source STL is treated as a surface: reverse distances remain unsigned, with
    no unverified watertight/solid-interior assumption. A buried groove surface
    and a source surface missed by the proxy are reported separately by union
    membership, rather than conflated as unsigned deviation.
    """
    source_triangles_m = _triangles(source_triangles_m)
    if int(samples_per_direction) != samples_per_direction or samples_per_direction < 1:
        raise ValueError('samples_per_direction must be a positive integer')
    count = int(samples_per_direction)
    union = convex_union_surface(proxy_parts)
    source = TriangleSurface(source_triangles_m)
    source_points, _ = _sample_surface(source_triangles_m, count, seed)
    proxy_points, _ = _sample_surface(union.triangles, count, seed+1)
    directions = {}

    def query(label, points, destination, with_membership):
        distances, nearest, faces = destination.nearest(points)
        margins = union.membership_margin(points) if with_membership else None
        directions[label] = (points, distances, nearest, faces, margins, destination.triangles)
        return _summarize(points, distances, nearest, faces, margins,
                          union.face_parts if with_membership else None,
                          union.hulls if with_membership else None, destination.triangles)

    result = {'units': 'm', 'samples_per_direction': count, 'seed': int(seed),
              'sampling': 'stratified by triangle area, uniform barycentric draw; deterministic PCG64 seed',
              'distance_query': 'continuous triangles; BVH exhausts every node within nearest-distance bound',
              'source_triangle_count': len(source_triangles_m),
              'source_area_m2': float(_areas(source_triangles_m).sum()),
              'proxy_union': union.diagnostics,
              'source_to_proxy_union': query('source_to_proxy_union', source_points, union.surface, True),
              'proxy_union_to_source': query('proxy_union_to_source', proxy_points, source, False),
              'source_interior_classification': 'UNKNOWN; no source watertightness/embedded-solid assumption',
              'global_hausdorff_distance': 'UNKNOWN; maxima are observed finite-query lower bounds',
              'interpretation': 'Negative source-to-proxy distance marks source surface buried by proxy (including possible groove/gap capping); positive marks uncovered source. Reverse unsigned deviation alone does not classify outside-source volume.'}
    if include_vertex_extrema:
        result['source_vertices_to_proxy_union'] = query(
            'source_vertices_to_proxy_union', np.unique(source_triangles_m.reshape(-1, 3), axis=0), union.surface, True)
        result['proxy_union_vertices_to_source'] = query(
            'proxy_union_vertices_to_source', np.unique(union.triangles.reshape(-1, 3), axis=0), source, False)
        result['vertex_extrema_note'] = 'All unique source and exposed-union vertices checked separately; vertex quantiles are not surface-area quantiles.'
    result['regions'] = {}
    for name, region in (regions or {}).items():
        values = {'definition_link_frame_m': region}
        for label, (points, distances, nearest, faces, margins, target_triangles) in directions.items():
            mask = _region_mask(points, region)
            values[label] = _summarize(points[mask], distances[mask], nearest[mask], faces[mask],
                                       margins[mask] if margins is not None else None,
                                       union.face_parts if margins is not None else None,
                                       union.hulls if margins is not None else None, target_triangles)
        result['regions'][name] = values
    return result


def focused_checks():
    """Small analytic checks; no environment, dynamics, or repository mutation."""
    from scipy.spatial import HalfspaceIntersection
    from scipy.spatial.transform import Rotation

    def box(lo, hi):
        vertices = np.array([[x, y, z] for x in (lo[0], hi[0])
                             for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])
        return vertices[ConvexHull(vertices).simplices]

    triangle = np.array([[[0., 0, 0], [1, 0, 0], [0, 1, 0]]])
    distance, nearest, _ = TriangleSurface(triangle).nearest(
        np.array([[.2, .2, 1], [2, 0, 0], [.8, .8, 0]]))
    if not (np.allclose(distance, [1, 1, np.sqrt(.18)])
            and np.allclose(nearest[0], [.2, .2, 0])):
        raise AssertionError('Continuous triangle interior/edge projection failed')
    cube = box([0., 0, 0], [1, 1, 1])
    results = {'continuous_triangle_interior_and_edges': 'PASS'}
    cases = {'overlap_hidden_interface': ([cube, box([.5, 0, 0], [1.5, 1, 1])], 8., [1, .5, .5], .5),
             'coincident_surface_tie': ([cube, cube], 6., [.5, .5, .5], .5),
             'touching_internal_interface': ([cube, box([1., 0, 0], [2, 1, 1])], 10., [1, .5, .5], .5),
             'contained_part': ([cube, box([.2, .2, .2], [.8, .8, .8])], 6., [.2, .5, .5], .2)}
    for name, (parts, area, point, expected_distance) in cases.items():
        union = convex_union_surface([{'name': str(i), 'triangles': part}
                                      for i, part in enumerate(parts)])
        query_points = np.array([point])
        measured, nearest, faces = union.surface.nearest(query_points)
        if not (np.isclose(_areas(union.triangles).sum(), area, atol=1e-9)
                and np.isclose(measured[0], expected_distance, atol=1e-9)):
            raise AssertionError(name)
        signed = _summarize(query_points, measured, nearest, faces,
                            union.membership_margin(query_points))
        if signed['source_points_strictly_inside_proxy_count'] != 1:
            raise AssertionError(name+' exposed-union signed interior')
        results[name] = 'PASS'
    for name, rotation, shift in [('oblique_overlap', [.2, .1, .35], [.3, 0, 0]),
                                   ('nearly_parallel_distinct_planes', [5e-6, 0, 0], [0, 0, 0])]:
        second = (cube-.5)@Rotation.from_euler('xyz', rotation).as_matrix().T+.5+shift
        union = convex_union_surface([{'triangles': cube}, {'triangles': second}])
        centers = union.triangles.mean(axis=1)
        normals = np.cross(union.triangles[:, 1]-union.triangles[:, 0],
                           union.triangles[:, 2]-union.triangles[:, 0])
        normals /= np.linalg.norm(normals, axis=1)[:, None]
        if (np.any(union.contains(centers+1e-7*normals))
                or not np.all(union.contains(centers-1e-7*normals))):
            raise AssertionError(name+' outward/inward boundary classification')
        equations = np.concatenate([hull['equations'] for hull in union.hulls])
        vertices = HalfspaceIntersection(equations, np.array([.5, .5, .5])+np.asarray(shift)/2).intersections
        expected = 2-ConvexHull(vertices).volume
        measured = np.einsum('ij,ij->', union.triangles[:, 0],
                            np.cross(union.triangles[:, 1], union.triangles[:, 2]))/6
        if not np.isclose(measured, expected, atol=1e-10, rtol=0):
            raise AssertionError(name+' inclusion-exclusion volume')
        results[name] = 'PASS'
    return results
