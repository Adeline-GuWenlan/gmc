"""bl B1: the harness's own methods (gmc-venv only), run through ``bl_worker`` exactly like a baseline.

  astar_replay  the pair's own A* route (the route the pair was confirmed with). It is checked the way stage J's
                gate (b) did: the stored 0.1 mm-rounded route as unicycle poses through ``verify_path`` on the
                compile's oracle; only if rounding makes it fail is the A* re-run from the exact endpoints
                (``aerial3dg_fail5_trace.Ctx.rerun_astar``: gs3d LatticePlanner, 0.1 m, margin 0.001) and its exact
                poses returned. Sanity check of the harness: it must be SUCCESS on every pair.
  gmc_requery   GMC itself (``gmc.aerial3d.api.query``, G2/F4 QCONFIG) on the persisted compile; returns the exact
                plan-frame polyline and, in ``info``, GMC's own gs3d export, so a test can compare it byte for byte
                with the harness's export.
  straight      the segment start -> goal (claimed): a negative control (collides where an obstacle is in the way).
  sleep         sleeps ``config["sleep_s"]`` then returns the straight segment (TIMEOUT test).
  raise         raises RuntimeError (ERROR test).
  crash         ``os._exit(3)`` mid-query: the worker dies (ERROR test, worker restart).

The state persisted by ``build_setup`` is only the compile's path + SHA-256; ``instantiate`` loads and checks it.
"""
from __future__ import annotations

import os
import time

import numpy as np

MULTI = True


def _sha256_file(p):
    import hashlib
    h = hashlib.sha256()
    with open(p, "rb") as f:
        for b in iter(lambda: f.read(1 << 22), b""):
            h.update(b)
    return h.hexdigest()


class Adapter:
    def __init__(self, method):
        self.method = method
        self.setup_info, self.instance_info = {}, {}

    def build_setup(self, scene, body, config):
        meta = scene["meta"]
        return {"method": self.method, "a3c": meta["a3c"], "a3c_sha256": meta["a3c_sha256"],
                "region": meta["region"], "robot": meta["robot"]}

    def instantiate(self, state, config):
        live = {"state": state, "config": config}
        if self.method in ("astar_replay", "gmc_requery"):
            from gmc.aerial3d.api import load_compiled
            if _sha256_file(state["a3c"]) != state["a3c_sha256"]:
                raise ValueError("compile SHA-256 changed since setup")
            live["C"] = load_compiled(state["a3c"])
            self.instance_info = {"compile_id": live["C"].compile_id}
        return live

    # ------------------------------------------------------------------------------------------------ plan
    def plan(self, live, start_uv, goal_uv, query):
        m = self.method
        if m == "straight":
            return {"claimed": True, "claimed_reason": "straight_segment", "path_uv": [start_uv, goal_uv]}
        if m == "sleep":
            time.sleep(float(live["config"].get("sleep_s", 1e6)))
            return {"claimed": True, "claimed_reason": "slept", "path_uv": [start_uv, goal_uv]}
        if m == "raise":
            raise RuntimeError("test method raises")
        if m == "crash":
            os._exit(3)
        if m == "gmc_requery":
            return self._gmc(live, start_uv, goal_uv)
        if m == "astar_replay":
            return self._astar(live, start_uv, goal_uv, query["pair"])
        raise ValueError(m)

    @staticmethod
    def _world(C, uv):
        b = C.body
        return C.frame.to_world([uv[0], uv[1], b.ground_clearance_m + b.half_height_m])

    def _gmc(self, live, s_uv, g_uv):
        from gmc.aerial3d.api import query
        from aerial3dg_run import QCONFIG
        C = live["C"]
        r = query(C, self._world(C, s_uv), self._world(C, g_uv), config=QCONFIG, call_id="bl_gmc_requery")
        stages = {t["stage"]: t["seconds"] for t in r["timings"]["records"]}
        if r["status"] != "REACHABLE":
            return {"claimed": False, "claimed_reason": f"{r['status']}:{r['reason']}", "path_uv": None,
                    "stages": stages}
        pts = np.asarray(r["polyline_plan"], float)
        return {"claimed": True, "claimed_reason": r["reason"], "path_uv": pts[:, :2].tolist(), "stages": stages,
                "info": {"gmc_trajectory": r["gs3d_result"]["trajectory"],
                         "gmc_original_goal": r["gs3d_result"]["original_goal"],
                         "gmc_polyline_world": r["polyline_world"]}}

    def _astar(self, live, s_uv, g_uv, pair):
        from gmc.gs3d.contracts import GoalRegion, Pose3
        from gmc.gs3d.oracle import GaussianBodyOracle
        from gmc.gs3d.validation import angle_delta, verify_path
        import math
        C = live["C"]
        body = C.body
        if "oracle" not in live:
            live["oracle"] = GaussianBodyOracle(C.prepared)
        route = [list(map(float, p)) for p in pair["astar_route_uv"]]
        W = [self._world(C, p) for p in route]
        poses = [Pose3(tuple(map(float, W[0])), 0.)]
        for q in W[1:]:                               # aerial3dg_fail5_trace.Ctx.route_poses
            prev = poses[-1]
            d = np.asarray(q) - prev.xyz
            if float(np.linalg.norm(d[:2])) <= 1e-12:
                continue
            yaw = prev.yaw + angle_delta(math.atan2(d[1], d[0]), prev.yaw)
            if yaw != prev.yaw:
                poses.append(Pose3(prev.xyz, yaw))
            poses.append(Pose3(tuple(map(float, q)), yaw))
        last = poses[-1]
        turn = angle_delta(0., last.yaw)
        if turn != 0.:
            poses.append(Pose3(last.xyz, last.yaw + turn))
        goal = GoalRegion(Pose3(tuple(map(float, self._world(C, g_uv))), 0.), 0., .05)
        v = verify_path(live["oracle"], poses, body, margin_m=C.config.margin_m, goal=goal)
        if v["passed"]:
            return {"claimed": True, "claimed_reason": "stored_route_verified", "path_uv": route,
                    "info": {"source": "stored_route_uv_0.1mm"}}
        from gmc.gs3d.contracts import PlannerConfig, SearchBudget
        from gmc.gs3d.planner import LatticePlanner
        from aerial3dg_fail_oracle import BUDGET
        if "planner" not in live:
            live["planner"] = LatticePlanner(C.prepared)
        conf = PlannerConfig(resolution_m=.1, margin_m=C.config.margin_m, seed=0,
                             budget=SearchBudget(max_wall_s=120., **BUDGET))
        start = Pose3(tuple(map(float, self._world(C, s_uv))), 0.)
        res = live["planner"].plan(C.prepared.scene, body, start, goal, conf)
        if res["status"] != "success":
            return {"claimed": False, "claimed_reason": f"astar_rerun_{res['status']}", "path_uv": None,
                    "info": {"source": "rerun", "stored_route_reason": v["reason"]}}
        P = np.asarray([p[:3] for p in res["trajectory"]["poses"]], float)
        uv = C.frame.to_plan(P)[:, :2]
        keep = [0] + [i for i in range(1, len(uv)) if np.linalg.norm(uv[i] - uv[i - 1]) > 1e-12]
        return {"claimed": True, "claimed_reason": "astar_rerun_exact", "path_uv": uv[keep].tolist(),
                "info": {"source": "rerun_exact_poses", "stored_route_reason": v["reason"]}}
