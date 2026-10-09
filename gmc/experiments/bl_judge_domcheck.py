"""bl J (a) follow-up: the one HEAD-only test failure (test_aerial3d_pairs::test_route_prism_domain_matches_the_baseline_oracle_in_an_empty_scene)
asserts GMC planning domain == judge free set. Same scene as that test, 20000 poses: count both directions.
Run from gmc/ with PYTHONPATH=src:experiments (HEAD) or the old tree."""
import json
import sys, numpy as np
sys.path.insert(0, "tests/unit"); sys.path.insert(0, "tests")
import test_aerial3d_pairs as T
from gmc.aerial3d.pairs import domain_from_scene
from gmc.gs3d.oracle import GaussianBodyOracle
from gmc.gs3d.integration import RouteBoxKnownSpace
from gmc.gs3d.contracts import Pose3
t = 1.13
Rw = np.array([[np.cos(t), np.sin(t), 0], [np.sin(t), -np.cos(t), 0], [0, 0, 1]])
origin = np.array([9.8, 26.9, -1.2])
lo_r, hi_r = np.array((-3., -.35, 0.)), np.array((3.7, 2.75, 2.43))
known = RouteBoxKnownSpace(tuple(origin), tuple(map(tuple, Rw)), tuple(lo_r), tuple(hi_r))
lo, hi = known.world_bounds()
scene = T.make_scene(lower=lo, upper=hi, known=known)
frame, domain = domain_from_scene(scene, T.UAV, margin_m=T.M)
oracle = GaussianBodyOracle(scene)
rng = np.random.default_rng(2)
c = {"both": 0, "neither": 0, "gmc_only": 0, "judge_only": 0}
band_ok = True; worst = 0
r = T.UAV.radius_m; e = r * (abs(Rw[0, 0]) + abs(Rw[0, 1]) - 1)
for qp in rng.uniform((-3.2, -.5, -.1), (3.9, 2.9, 2.6), size=(20000, 3)):
    s = domain.row_slack(qp)
    if abs(s) < 1e-6: continue
    free = oracle.pose(Pose3(tuple(frame.to_world(qp))), T.UAV, margin_m=T.M).occupancy == "free"
    g = s > 0
    k = "both" if g and free else "neither" if not g and not free else "gmc_only" if g else "judge_only"
    c[k] += 1
    if k == "judge_only":
        worst = min(worst, s); band_ok &= s >= -e - 1e-9
out = {"counts": c, "band_m": e, "worst_judge_only_slack_m": worst, "judge_only_within_band": bool(band_ok), "oracle_file": __import__("gmc.gs3d.oracle", fromlist=["x"]).__file__}
print(json.dumps(out))
print(c, "band e=r(|c|+|s|-1)=", round(e, 5), "worst slack", round(worst, 5), "all judge_only within band:", band_ok)
