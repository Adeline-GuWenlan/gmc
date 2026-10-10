import gzip,json,numpy as np,bl_harness as H
r=[json.loads(x) for x in gzip.open('results/baselines/pilot/foci/WWEST/cylinder/task_00.jsonl.gz','rt') if '"F4X-00002"' in x][0]
J=H.Judge('WWEST','cylinder')
from gmc.gs3d.trajectory import replay_plan
_,res=J.export(np.asarray(r['route_polyline']),r['goal_uv'])
rep=replay_plan(res,J._oracle_cls(J.C.prepared)); g=rep['geometry']; last=g['reports'][-1]
print(g['reason'], last)
ids=set(last['ids'] if 'ids' in last else last.get('primitive_ids',()))
P=np.asarray(res['trajectory']['poses']); e=len(g['reports'])-1; print('edge', P[e], P[e+1], 'uv', J.C.frame.to_plan(P[e][:3]))
prep=J.C.prepared; idx=[i for i,x in enumerate(prep.ids) if int(x) in ids]
mp=J.C.frame.to_plan(prep.means[idx]); sd=2*np.sqrt(np.einsum('nii->ni',prep.covs[idx]))
for a,b in zip(mp,sd): print('gauss plan', a.round(3), '2sigma ext', b.round(3))
