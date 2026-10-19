"""Offline experimental contact provenance; no MuJoCo execution API.

Convex candidates come from complete polytope/box intersections, never an MPR
midpoint radius. Exact dyadic hull predicates and rational clipping preserve
all face/edge/vertex candidates. UNKNOWN is the default semantic assignment.
"""
from collections import Counter
from fractions import Fraction as Q
from functools import reduce
from math import gcd, isqrt
import numpy as np
from scipy.optimize import linprog
from scipy.spatial import ConvexHull


def rational(x):
    return x if isinstance(x,Q) else Q(x) if isinstance(x,str) else Q.from_float(float(x))


def dot(a,b):return sum(x*y for x,y in zip(a,b))
def cross(a,b):return (a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0])


def affine_vertices(points,rotation,translation):
    r=[tuple(map(rational,row))for row in rotation];t=tuple(map(rational,translation))
    return [tuple(dot(row,tuple(map(rational,p)))+offset for row,offset in zip(r,t))for p in points]


class CertifiedHull:
    """Qhull proposes topology; integer predicates independently certify it."""
    def __init__(self,vertices,exact_points=None,faces=None):
        self.points=[tuple(map(rational,p))for p in (vertices if exact_points is None else exact_points)]
        self.vertices=np.array(self.points,float)
        self.faces=ConvexHull(self.vertices).simplices if faces is None else np.asarray(faces,int)
        ex=max(x.denominator.bit_length()-1 for p in self.points for x in p)
        scale=1<<ex
        if any(scale%x.denominator for p in self.points for x in p):raise ValueError('Expected frozen dyadic coordinates')
        ints=[[x.numerator*(scale//x.denominator)for x in p]for p in self.points]
        center=[sum(p[k]for p in ints)for k in range(3)]
        self.planes=[];normals=[];equations=[]
        for face in self.faces:
            a,b,c=[ints[i]for i in face]
            n=cross([b[i]-a[i]for i in range(3)],[c[i]-a[i]for i in range(3)])
            if dot(n,[len(ints)*a[k]-center[k]for k in range(3)])<0:n=tuple(-x for x in n)
            d=-dot(n,a)
            if not any(n) or any(dot(n,p)+d>0 for p in ints):
                raise ValueError('UNKNOWN: proposed facet is not an exact convex support')
            row=tuple(x*scale for x in n)+(d,)
            divisor=reduce(gcd,map(abs,row));row=tuple(x//divisor for x in row)
            self.planes.append(row)
            m=max(abs(x)for x in row[:3]);nf=np.array([float(Q(x,m))for x in row[:3]])
            length=np.linalg.norm(nf);normals.append(nf/length)
            equations.append(np.r_[nf/length,float(Q(row[3],m))/length])
        edges=Counter(tuple(sorted((int(a),int(b))))for f in self.faces for a,b in zip(f,np.roll(f,-1)))
        used=set(map(int,self.faces.ravel()))
        if set(edges.values())!={2} or len(used)-len(edges)+len(self.faces)!=2:
            raise ValueError('UNKNOWN: hull closure topology not certified')
        reached={next(iter(used))}
        while True:
            expanded=reached|{v for a,b in edges if a in reached or b in reached for v in (a,b)}
            if expanded==reached:break
            reached=expanded
        if reached!=used:raise ValueError('UNKNOWN: disconnected hull')
        self.normals=np.array(normals);self.equations=np.array(equations)
        self.certificate={'all_vertices_inside_every_exact_support':True,'closed_edges':len(edges),
                          'euler_characteristic':2,'connected':True,'faces':len(self.faces),
                          'vertices':len(self.vertices),'dyadic_scale_exponent':ex}


def clip(poly,row):
    """Closed halfspace intersection, including zero-area edge/point results."""
    if not poly:return []
    ds=[dot(row[:-1],p)+row[-1]for p in poly];out=[]
    for i,a in enumerate(poly):
        j=(i+1)%len(poly);b=poly[j];da,db=ds[i],ds[j]
        if da<=0:out.append(a)
        if (da<=0)!=(db<=0):out.append(tuple(x+(y-x)*da/(da-db)for x,y in zip(a,b)))
    return list(dict.fromkeys(out))


def contact_faces(a,b,guard_m):
    """All boundary patches of a potentially in b, with geometric roundoff guard."""
    guard=rational(guard_m)
    lower=[min(p[k]for p in b.points)-guard for k in range(3)]
    upper=[max(p[k]for p in b.points)+guard for k in range(3)]
    rows=[tuple(row[:-1])+(row[-1]-guard*sum(map(abs,row[:-1])),)for row in b.planes]
    result=[]
    for i,face in enumerate(a.faces):
        poly=[a.points[j]for j in face]
        if any(max(p[k]for p in poly)<lower[k]or min(p[k]for p in poly)>upper[k]for k in range(3)):continue
        for row in rows:
            poly=clip(poly,row)
            if not poly:break
        if poly:
            dimension=0 if len(poly)==1 else 1
            if len(poly)>=3 and any(any(cross([v[k]-poly[0][k]for k in range(3)],
                                             [w[k]-poly[0][k]for k in range(3)]))for v,w in zip(poly[1:],poly[2:])):dimension=2
            result.append({'face':i,'dimension':dimension,'normal':a.normals[i].tolist(),
                           'polygon_m':[[float(x)for x in p]for p in poly],
                           'polygon_exact_m':[[str(x)for x in p]for p in poly]})
    return result


def intersection_certificate(a,b,guard_m):
    """Exact separation support, or LP proposal followed by exact common-ball proof."""
    guard=rational(guard_m)
    for side,first,second in [('a',a,b),('b',b,a)]:
        for i,row in enumerate(first.planes):
            gap=min(dot(row[:3],p)+row[3]for p in second.points)
            reserve=guard*sum(map(abs,row[:3]))
            if gap>reserve:
                # Numeric distance is informative; exact sign is the certificate.
                m=max(abs(x)for x in row[:3]);den=np.linalg.norm([float(Q(x,m))for x in row[:3]])
                return {'status':'SEPARATED','axis_owner':side,'face':i,
                        'gap_lower_bound_m':float((gap-reserve)/m)/den,
                        'exact_positive_residual':str(gap-reserve),'plane_integer':list(map(str,row))}
    eq=np.concatenate([a.equations,b.equations])
    proposal=linprog([0,0,0,-1],A_ub=np.c_[eq[:,:3],np.ones(len(eq))],b_ub=-eq[:,3]*1000,
                     bounds=[(None,None)]*3+[(0,None)],method='highs')
    if proposal.success and proposal.x[3]>0:
        center=tuple(map(rational,proposal.x[:3]/1000));radius=rational(proposal.x[3]/2000)
        margins=[]
        for row in a.planes+b.planes:
            length2=sum(x*x for x in row[:3]);bound=isqrt(length2)
            if bound*bound<length2:bound+=1
            margin=-(dot(row[:3],center)+row[3]+radius*bound+guard*sum(map(abs,row[:3])))
            margins.append(margin)
        if min(margins)>0:
            return {'status':'STRICT_OVERLAP','common_ball_radius_m':float(radius),
                    'center_m':list(map(float,center)),'radius_exact_m':str(radius),
                    'center_exact_m':list(map(str,center)),'all_exact_eroded_halfspace_inequalities_strict':True,
                    'minimum_integer_scaled_margin_exact':str(min(margins))}
    return {'status':'UNKNOWN_TOUCH_OR_NO_STRICT_CERTIFICATE'}


def hull2(points):
    points=sorted(set(points))
    if len(points)<3:return points
    def turn(a,b,c):return (b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0])
    lower=[];upper=[]
    for target,seq in [(lower,points),(upper,list(reversed(points)))]:
        for p in seq:
            while len(target)>=2 and turn(target[-2],target[-1],p)<=0:target.pop()
            target.append(p)
    return lower[:-1]+upper[:-1]


def triangle_rows2(triangle):
    area=sum(a[0]*b[1]-a[1]*b[0]for a,b in zip(triangle,triangle[1:]+triangle[:1]))
    sign=1 if area>0 else -1
    return [(sign*(b[1]-a[1]),sign*(a[0]-b[0]),
             sign*((b[0]-a[0])*a[1]-(b[1]-a[1])*a[0]))for a,b in zip(triangle,triangle[1:]+triangle[:1])]


def semantic_patch(polygon,part,source_functional,cuts,guard_m,cavity_planes=()):
    """Complete conservative partition: proven source patch, risk, else UNKNOWN.

    The functional registry is ONLY the two previously identified source faces.
    Adjacent source triangles are never automatically promoted to functional.
    """
    if part==0:return {'status':'ROOT_BORE_RISK_ENVELOPE','coverage':'entire root piece; no material-inside claim'}
    p=[tuple(map(rational,v))for v in polygon];g=rational(guard_m)
    if len(p)<3:return {'status':'UNKNOWN_EDGE_OR_POINT'}
    if any(min(v[2]for v in p)<=rational(z)+g and max(v[2]for v in p)>=rational(z)-g for z in cuts):
        return {'status':'CUT_SEAM_RISK_ENVELOPE'}
    for row in cavity_planes:
        if all(abs(dot(tuple(map(rational,row[:3])),v)+rational(row[3]))<=g for v in p):
            return {'status':'CAVITY_WITNESS_CLOSURE_RISK_ENVELOPE'}
    projected=hull2([(v[1]+dy*g,v[2]+dz*g)for v in p for dy in (-1,1)for dz in (-1,1)])
    if len(projected)<3:return {'status':'UNKNOWN_DEGENERATE_PROJECTION'}
    remaining=[projected];gap_values=[]
    n=cross([p[1][k]-p[0][k]for k in range(3)],[p[2][k]-p[0][k]for k in range(3)])
    if not n[0]:return {'status':'UNKNOWN_NONFRONT_PROJECTION'}
    d=-dot(n,p[0])
    for raw in source_functional:
        t=[tuple(map(rational,v))for v in raw];rows=triangle_rows2([(v[1],v[2])for v in t])
        source_n=cross([t[1][k]-t[0][k]for k in range(3)],[t[2][k]-t[0][k]for k in range(3)])
        if not source_n[0]:continue
        source_d=-dot(source_n,t[0]);overlap=projected
        for row in rows:overlap=clip(overlap,row)
        for y,z in overlap:
            px=-(n[1]*y+n[2]*z+d)/n[0]
            sx=-(source_n[1]*y+source_n[2]*z+source_d)/source_n[0]
            gap_values.append(px-sx)
        new=[]
        for q in remaining:
            for row in rows:
                if any(dot(row[:2],v)+row[2]>0 for v in q):
                    outside=clip(q,tuple(-x for x in row))
                    if outside:new.append(outside)
                q=clip(q,row)
                if not q:break
        remaining=new
    area=sum(abs(sum(a[0]*b[1]-a[1]*b[0]for a,b in zip(q,q[1:]+q[:1])))/2 for q in remaining)
    if remaining:status='UNKNOWN_CAVITY_OR_UNREGISTERED_SURFACE'
    elif not gap_values or min(gap_values)<-g:status='UNKNOWN_INWARD_OR_WRONG_SIDE'
    else:status='FUNCTIONAL_SOURCE_PATCH_APPROXIMATION'
    return {'status':status,'uncovered_projected_area_m2':float(area),'uncovered_area_exact_m2':str(area),
            'all_registered_source_projection_covered':not remaining,
            'source_ray_offset_range_m':None if not gap_values else [float(min(gap_values)),float(max(gap_values))],
            'limits':'Functional provenance only; no new fidelity budget, material certificate or grasp/release PASS.'}


def decision(statuses):
    if not statuses:return 'STOP_UNKNOWN_NO_GEOMETRIC_CANDIDATE'
    if any('RISK_ENVELOPE'in s for s in statuses):return 'STOP_RISK_ENVELOPE'
    if any(s.startswith('UNKNOWN')for s in statuses):return 'STOP_UNKNOWN_SEMANTICS'
    return 'FUNCTIONAL_PROVENANCE_ONLY_NOT_RELEASE_PASS'


def source_front_certificate(polygon,source,face_id,guard_m):
    """Continuous full-source +X envelope proof, beyond a normal-only test.

    Establishes directional exterior geometry only. A functional-neighbor
    adjacency certificate and explicit registration are separate requirements.
    """
    p=[tuple(map(rational,v))for v in polygon];g=rational(guard_m)
    t=[tuple(map(rational,v))for v in source[face_id]]
    n=cross([t[1][k]-t[0][k]for k in range(3)],[t[2][k]-t[0][k]for k in range(3)])
    if n[0]<=0:return {'certified':False,'reason':'Not a +X-oriented source face'}
    d=-dot(n,t[0]);domain=hull2([(v[1]+dy*g,v[2]+dz*g)for v in p for dy in (-1,1)for dz in (-1,1)])
    if len(domain)<3:return {'certified':False,'reason':'Degenerate projected domain'}
    if any(dot(row[:2],v)+row[2]>0 for row in triangle_rows2([(v[1],v[2])for v in t])for v in domain):
        return {'certified':False,'reason':'Source triangle does not cover entire guarded projection'}
    rows=triangle_rows2(domain)
    lo=np.nextafter(np.array(domain,float).min(0),-np.inf)
    hi=np.nextafter(np.array(domain,float).max(0),np.inf)
    maximum=None;owner=None;intersecting=[]
    for i,tri in enumerate(source):
        if np.any(tri[:,1:].max(0)<lo)or np.any(tri[:,1:].min(0)>hi):continue
        q=[tuple(map(rational,v))for v in tri]
        for a,b,c in rows:
            q=clip(q,(0,a,b,c))
            if not q:break
        if not q:continue
        upper=max(v[0]+(n[1]*v[1]+n[2]*v[2]+d)/n[0]for v in q)
        intersecting.append(i)
        if maximum is None or upper>maximum:maximum,owner=upper,i
    return {'certified':maximum is not None and maximum<=0,'source_face':face_id,
            'complete_source_triangles_considered':len(source),'projecting_triangle_ids':intersecting,
            'source_forward_excess_upper_exact_m':None if maximum is None else str(maximum),
            'source_forward_excess_upper_m':None if maximum is None else float(maximum),
            'maximum_owner':owner,'all_projected_source_points_at_or_behind_reference_plane':maximum is not None and maximum<=0,
            'limits':'Directional exterior surface provenance, not a watertight material, tolerance waiver or release certificate.'}
