import os
"""Focused offline verification; no repository checks/gates or simulation."""
import json
from pathlib import Path
import numpy as np
import surface_geometry as g
OUT=Path(os.environ['NERO_D3_OUTPUT'])/'triangle'
A=np.array([[[0.,0,0],[1,0,0],[0,1,0]]])
cases={
 'parallel_face_interior':(A,A+[0,0,2],2.),
 'strict_transverse_crossing':(A,np.array([[[.2,.2,-1],[.2,.2,1],[.7,.2,0]]]),0.),
 'coplanar_overlap':(A,A+[.2,.2,0],0.),
 'edge_edge_interior':(A,np.array([[[1.,1,1],[1.,1,2],[2.,2,1]]]),np.sqrt(1.5)),
 'second_triangle_point_above_face':(A,np.array([[[.2,.2,3]]*3]),3.),
 'second_triangle_point_nearest_edge':(A,np.array([[[.5,-.5,0]]*3]),.5),
 'first_triangle_point_nearest_edge':(np.array([[[.5,-.5,0]]*3]),A,.5),
 'both_triangles_points':(np.zeros((1,3,3)),np.ones((1,3,3)),np.sqrt(3)),
}
results={}
for name,(a,b,expected) in cases.items():
 answer=g.surface_query(a,b)
 error=abs(answer['surface_min_distance_m']-expected)
 assert error<1e-12,(name,answer,expected)
 results[name]={'distance_m':answer['surface_min_distance_m'],'expected_m':expected,'absolute_error_m':error}
assert g.surface_query(*cases['strict_transverse_crossing'][:2])['strict_transverse_witness'] is not None
rng=np.random.default_rng(130618);a=rng.normal(size=(29,3,3));b=rng.normal(size=(31,3,3))+[0,0,5]
query=g.surface_query(a,b);ds,_,_,hit=g.triangle_pairs(np.repeat(a,len(b),axis=0),np.tile(b,(len(a),1,1)))
assert hit is None
assert abs(query['surface_min_distance_m']**2-ds.min())<1e-12
reverse=g.surface_query(b,a)
assert abs(query['surface_min_distance_m']-reverse['surface_min_distance_m'])<1e-12
rs=json.loads((OUT/'surface_results.json').read_text())
assert len(rs)==24
assert sum('certificate_reused_from_mode' not in r for r in rs)==16
assert sum(r['strict_transverse_witness'] is not None for r in rs)==20
assert all(r['strict_transverse_witness']['decimal_recheck']['strict_crossing_reconfirmed'] for r in rs if r['strict_transverse_witness'])
assert all(r['strict_transverse_witness']['within_15mm_of_original_contact'] for r in rs if r['strict_transverse_witness'])
assert all(r['closest_pair_independent_QP']['scipy_SLSQP_success'] for r in rs if r['surface_min_distance_m']>0)
assert all(r['all_surface_search_complete'] for r in rs if r['surface_min_distance_m']>0)
result={'analytic_cases':results,'BVH_vs_exhaustive_triangle_pairs':29*31,'BVH_distance_m':query['surface_min_distance_m'],'reverse_distance_m':reverse['surface_min_distance_m'],
 'results_rows':len(rs),'actual_surface_queries':16,'reused_A_certificates':8,'positive_distance_cases':4,
 'all_positive_distance_nearest_pair_QP_succeeded':True,'all_crossing_witnesses_decimal_rechecked':True,'all_crossing_witnesses_within_recorded_contact_15mm':True,
 'note':'Finite-precision continuous triangle geometry. Decimal checks strengthen recorded transformed-triangle witnesses; they do not establish an exact solver-stage state. Three source STLs are non-watertight/nonmanifold, so disjoint-solid inference is limited.'}
(OUT/'focused_verification.json').write_text(json.dumps(result,indent=2))
print(json.dumps(result,indent=2))
