"""Offline full original-STL surface geometry. No dynamics or mesh convexification.
Original binary-STL face order is retained (zero-based). Distances are between
continuous closed triangles, not sampled vertices. NumPy/SciPy only.
"""
from pathlib import Path
import heapq
import struct
import numpy as np
from scipy.spatial import cKDTree


def read_stl(path):
    buf = Path(path).read_bytes()
    n, = struct.unpack_from('<I', buf, 80)
    if len(buf) != 84 + 50*n:
        raise ValueError('Expected binary STL of exact declared size')
    dtype = np.dtype([('normal','<f4',(3,)), ('vertices','<f4',(3,3)), ('attr','<u2')])
    return np.frombuffer(buf, dtype=dtype, offset=84, count=n)['vertices'].astype(np.float64)


def mesh_topology(tris):
    vertices, inverse = np.unique(tris.reshape(-1,3), axis=0, return_inverse=True)
    faces = inverse.reshape(-1,3)
    edges = faces[:,[[0,1],[1,2],[2,0]]].reshape(-1,2)
    sorted_edges = np.sort(edges,axis=1)
    unique_edges, edge_inverse, counts = np.unique(sorted_edges,axis=0,return_counts=True,return_inverse=True)
    directions=np.where(edges[:,0] < edges[:,1],1,-1)
    balance=np.bincount(edge_inverse,weights=directions,minlength=len(unique_edges))
    area2=np.linalg.norm(np.cross(tris[:,1]-tris[:,0],tris[:,2]-tris[:,0]),axis=1)
    from scipy.sparse import coo_matrix
    from scipy.sparse.csgraph import connected_components
    adjacency=coo_matrix((np.ones(len(edges)*2),(np.r_[edges[:,0],edges[:,1]],np.r_[edges[:,1],edges[:,0]])),shape=(len(vertices),len(vertices))).tocsr()
    ncomp,labels=connected_components(adjacency,directed=False)
    _,facecounts=np.unique(np.sort(faces,axis=1),axis=0,return_counts=True)
    return {'faces':len(tris),'exact_weld_vertices':len(vertices),'undirected_edges':len(counts),
            'boundary_edges':int(np.sum(counts==1)), 'nonmanifold_edges':int(np.sum(counts>2)),
            'two_face_orientation_conflicts':int(np.sum((counts==2)&(balance!=0))),
            'zero_area_faces':int(np.sum(area2==0)), 'min_double_area_m2':float(area2.min()),
            'duplicate_faces_extra':int(np.sum(facecounts-1)), 'vertex_connected_components':int(ncomp),
            'component_vertex_counts':np.bincount(labels).tolist(),
            'edge_watertight_exact_float32_weld':bool(np.all(counts==2)),
            'consistently_oriented_edges':bool(np.all(balance==0)),
            'note':'Topology uses exact original float32 coordinates; edge-water-tightness does not itself establish absence of self-intersections.'}


class BVH:
    def __init__(self,triangles,leaf_size=8):
        self.tris=np.asarray(triangles,dtype=np.float64)
        self.nodes=[]
        self.centers=self.tris.mean(axis=1)
        self._build(np.arange(len(self.tris)),leaf_size)
        del self.centers
    def _build(self,ids,leaf_size):
        i=len(self.nodes);self.nodes.append(None)
        lo=self.tris[ids].min(axis=(0,1));hi=self.tris[ids].max(axis=(0,1))
        if len(ids)<=leaf_size:
            self.nodes[i]=(lo,hi,-1,-1,ids)
        else:
            extent=np.ptp(self.centers[ids],axis=0);axis=int(np.argmax(extent));mid=len(ids)//2
            order=np.argpartition(self.centers[ids,axis],mid)
            left=self._build(ids[order[:mid]],leaf_size);right=self._build(ids[order[mid:]],leaf_size)
            self.nodes[i]=(lo,hi,left,right,None)
        return i
    def refit(self,triangles):
        result=object.__new__(BVH);result.tris=np.asarray(triangles,dtype=np.float64);result.nodes=[None]*len(self.nodes)
        for i in range(len(self.nodes)-1,-1,-1):
            _,_,left,right,ids=self.nodes[i]
            if ids is not None:lo=result.tris[ids].min(axis=(0,1));hi=result.tris[ids].max(axis=(0,1))
            else:lo=np.minimum(result.nodes[left][0],result.nodes[right][0]);hi=np.maximum(result.nodes[left][1],result.nodes[right][1])
            result.nodes[i]=(lo,hi,left,right,ids)
        return result


def aabb_d2(a,b):
    delta=np.maximum(0,np.maximum(a[0]-b[1],b[0]-a[1]))
    return float(delta@delta)


def _point_triangle(p,t):
    """All P independent pairs; projection inside face else nearest edge."""
    a,b,c=t[:,0],t[:,1],t[:,2];ab=b-a;ac=c-a;n=np.cross(ab,ac)
    n2=np.einsum('ij,ij->i',n,n);signed=np.einsum('ij,ij->i',p-a,n)
    proj=p-n*np.divide(signed,n2,out=np.zeros_like(signed),where=n2>0)[:,None]
    v=proj-a;d00=np.einsum('ij,ij->i',ab,ab);d01=np.einsum('ij,ij->i',ab,ac);d11=np.einsum('ij,ij->i',ac,ac)
    d20=np.einsum('ij,ij->i',v,ab);d21=np.einsum('ij,ij->i',v,ac);den=n2
    u=np.divide(np.einsum('ij,ij->i',np.cross(v,ac),n),den,out=np.full_like(den,np.nan),where=den>0)
    w=np.divide(np.einsum('ij,ij->i',np.cross(ab,v),n),den,out=np.full_like(den,np.nan),where=den>0)
    inside=(u>=0)&(w>=0)&(u+w<=1)&(n2>0)
    d2=np.where(inside,np.einsum('ij,ij->i',p-proj,p-proj),np.inf);q=proj.copy()
    return d2,q


def _segment_segment(p,q,r,s):
    # Closest finite segments incl parallel and zero length (Ericson clamping).
    u=q-p;v=s-r;w=p-r
    a=np.einsum('ij,ij->i',u,u);b=np.einsum('ij,ij->i',u,v);c=np.einsum('ij,ij->i',v,v)
    d=np.einsum('ij,ij->i',u,w);e=np.einsum('ij,ij->i',v,w);den=np.einsum('ij,ij->i',np.cross(u,v),np.cross(u,v))
    ss=np.clip(np.divide(b*e-c*d,den,out=np.zeros_like(den),where=den>0),0,1)
    tt=np.divide(b*ss+e,c,out=np.zeros_like(c),where=c>0)
    lower=tt<0;upper=tt>1;tt=np.clip(tt,0,1)
    ss[lower]=np.clip(np.divide(-d[lower],a[lower],out=np.zeros(np.sum(lower)),where=a[lower]>0),0,1)
    ss[upper]=np.clip(np.divide(b[upper]-d[upper],a[upper],out=np.zeros(np.sum(upper)),where=a[upper]>0),0,1)
    second_point=(c==0)
    ss[second_point]=np.clip(np.divide(-d[second_point],a[second_point],out=np.zeros(np.sum(second_point)),where=a[second_point]>0),0,1)
    pa=p+ss[:,None]*u;pb=r+tt[:,None]*v
    return np.einsum('ij,ij->i',pa-pb,pa-pb),pa,pb


def _segment_triangle(p,q,t,plane_tol=1e-12,bary_tol=1e-10):
    a,b,c=t[:,0],t[:,1],t[:,2];ab=b-a;ac=c-a;n=np.cross(ab,ac);nn=np.linalg.norm(n,axis=1)
    nh=np.divide(n,nn[:,None],out=np.zeros_like(n),where=nn[:,None]>0)
    dp=np.einsum('ij,ij->i',p-a,nh);dq=np.einsum('ij,ij->i',q-a,nh)
    delta=dp-dq;f=np.divide(dp,delta,out=np.full_like(dp,np.nan),where=delta!=0)
    hit=p+f[:,None]*(q-p);v=hit-a
    d00=np.einsum('ij,ij->i',ab,ab);d01=np.einsum('ij,ij->i',ab,ac);d11=np.einsum('ij,ij->i',ac,ac)
    d20=np.einsum('ij,ij->i',v,ab);d21=np.einsum('ij,ij->i',v,ac);den=nn*nn
    u=np.divide(np.einsum('ij,ij->i',np.cross(v,ac),n),den,out=np.full_like(den,np.nan),where=den>0)
    w=np.divide(np.einsum('ij,ij->i',np.cross(ab,v),n),den,out=np.full_like(den,np.nan),where=den>0)
    bary=np.stack([1-u-w,u,w],axis=1)
    hitmask=(f>=0)&(f<=1)&np.all(bary>=-bary_tol,axis=1)&np.all(bary<=1+bary_tol,axis=1)&(nn>0)
    strict=hitmask&(((dp>plane_tol)&(dq<-plane_tol))|((dp<-plane_tol)&(dq>plane_tol)))&np.all(bary>bary_tol,axis=1)
    return hitmask,strict,hit,f,bary,dp,dq


def triangle_pairs(a,b):
    """Return continuous min on equal-length triangle pair arrays, plus strict crossing."""
    count=len(a);best=np.full(count,np.inf);pa=np.zeros((count,3));pb=pa.copy();witness=None
    def update(d,x,y):
        take=d<best;best[take]=d[take];pa[take]=x[take];pb[take]=y[take]
    for direction,src,dst in [(0,a,b),(1,b,a)]:
        for j in range(3):
            mask,strict,hit,f,bary,dp,dq=_segment_triangle(src[:,j],src[:,(j+1)%3],dst)
            if np.any(strict):
                k=int(np.flatnonzero(strict)[0]);witness={'local_pair_index':k,'edge_from_triangle':direction,'edge_vertices':[j,(j+1)%3],
                  'edge_parameter':float(f[k]),'other_triangle_barycentric':bary[k].tolist(),'edge_endpoint_signed_plane_distances_m':[float(dp[k]),float(dq[k])],
                  'point':hit[k].tolist(),'strict_transverse_crossing':True}
                return np.zeros(count),pa,pb,witness
            update(np.where(mask,0,np.inf),hit,hit)
            d,closest=_point_triangle(src[:,j],dst)
            update(d,src[:,j] if direction==0 else closest,closest if direction==0 else src[:,j])
    for j in range(3):
        for k in range(3):
            d,x,y=_segment_segment(a[:,j],a[:,(j+1)%3],b[:,k],b[:,(k+1)%3]);update(d,x,y)
    return best,pa,pb,witness


def surface_query(triA,triB,bvhA=None,bvhB=None,absolute_tol_m=1e-12):
    """All triangles included. Stops early only for proven strict crossing (min=0).
    For positive distance, all BVH pairs with lower bound < current best exhaust.
    Zero from boundary/coplanar numerical contact is distinguished from a strict
    transverse witness; this is floating-point geometry, not exact predicates.
    """
    a=BVH(triA) if bvhA is None else bvhA;b=BVH(triB) if bvhB is None else bvhB
    av=a.tris.reshape(-1,3);bv=b.tris.reshape(-1,3)
    # Any actual vertex pair gives a valid upper bound; sampling is for this
    # initialization only, never a substitute for the full triangle search.
    sample=np.linspace(0,len(av)-1,min(1024,len(av)),dtype=int)
    tree=cKDTree(bv);dd,ind=tree.query(av[sample],k=1)
    k=int(np.argmin(dd));best=float(dd[k]**2);wa=av[sample[k]];wb=bv[ind[k]];faces=(int(sample[k])//3,int(ind[k])//3)
    heap=[(aabb_d2(a.nodes[0],b.nodes[0]),0,0)];visited=leaves=tests=0
    while heap:
        batch_a=[];batch_b=[];batch_size=0
        # Batch independent leaves to keep NumPy work vectorized; best remains a
        # valid upper bound while collecting, and every retained pair is tested.
        while heap and batch_size<4096:
            bound,ia,ib=heapq.heappop(heap)
            if bound>best+2*np.sqrt(max(best,0))*absolute_tol_m+absolute_tol_m**2:continue
            visited+=1;na=a.nodes[ia];nb=b.nodes[ib];ida=na[4];idb=nb[4]
            if ida is not None and idb is not None:
                leaves+=1;batch_size+=len(ida)*len(idb)
                batch_a.append(np.repeat(ida,len(idb)));batch_b.append(np.tile(idb,len(ida)))
            else:
                if idb is not None or (ida is None and np.prod(na[1]-na[0])>=np.prod(nb[1]-nb[0])):
                    children=[(na[2],ib),(na[3],ib)]
                else:children=[(ia,nb[2]),(ia,nb[3])]
                for ca,cb in children:
                    d=aabb_d2(a.nodes[ca],b.nodes[cb])
                    if d<=best+2*np.sqrt(max(best,0))*absolute_tol_m+absolute_tol_m**2:heapq.heappush(heap,(d,ca,cb))
        if not batch_a:continue
        aa=np.concatenate(batch_a);bb=np.concatenate(batch_b);tests+=len(aa)
        ds,ps,qs,hit=triangle_pairs(a.tris[aa],b.tris[bb])
        if hit is not None:
            j=hit.pop('local_pair_index');hit['face_ids_zero_based']=[int(aa[j]),int(bb[j])]
            hit['triangle_a']=a.tris[aa[j]].tolist();hit['triangle_b']=b.tris[bb[j]].tolist()
            return {'surface_min_distance_m':0.0,'surface_intersection':True,'strict_transverse_witness':hit,'nearest_face_ids_zero_based':hit['face_ids_zero_based'],
                    'closest_points':[hit['point'],hit['point']],'visited_node_pairs':visited,'leaf_pairs':leaves,'triangle_pairs_evaluated':tests,'all_surface_search_complete':False,'global_zero_certified_by_intersection':True}
        j=int(np.argmin(ds))
        if ds[j]<best:best=float(ds[j]);wa=ps[j];wb=qs[j];faces=(int(aa[j]),int(bb[j]))
    return {'surface_min_distance_m':float(np.sqrt(max(best,0))),'surface_intersection':bool(best==0),'strict_transverse_witness':None,
            'nearest_face_ids_zero_based':list(faces),'closest_points':[wa.tolist(),wb.tolist()],
            'visited_node_pairs':visited,'leaf_pairs':leaves,'triangle_pairs_evaluated':tests,'all_surface_search_complete':True,'global_zero_certified_by_intersection':False,
            'numerical_absolute_bound_tolerance_m':absolute_tol_m}


def winding_number(points,triangles,chunk_size=20000):
    """Oriented solid-angle winding; report continuous values, not a binary claim.
    Well-defined as solid interior only for closed, consistently oriented embedded
    surfaces. Kept for diagnosing containment when topology permits it.
    """
    result=[]
    for point in np.asarray(points):
        total=0.
        for start in range(0,len(triangles),chunk_size):
            a,b,c=np.moveaxis(triangles[start:start+chunk_size]-point,1,0)
            la=np.linalg.norm(a,axis=1);lb=np.linalg.norm(b,axis=1);lc=np.linalg.norm(c,axis=1)
            num=np.einsum('ij,ij->i',a,np.cross(b,c))
            den=la*lb*lc+np.einsum('ij,ij->i',a,b)*lc+np.einsum('ij,ij->i',b,c)*la+np.einsum('ij,ij->i',c,a)*lb
            total+=np.sum(2*np.arctan2(num,den))
        result.append(float(total/(4*np.pi)))
    return result
