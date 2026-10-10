"""bl B2: freeze a map-based method's config from its tuning table (pick by the pre-registered rule).

``configs/baselines/<method>.json`` in B1's format (full effective parameters per robot = adapter DEFAULTS + the
picked candidate, adapter SHA-256, tuned-on counts) plus the SHA-256 of every shared raster the frozen config reads.
Usage (gmc-venv, from gmc/): python experiments/bl_b2_freeze.py --method pno|cust_fields
"""
import argparse
import hashlib
import importlib
import json
import sys
import time
from pathlib import Path

sys.dont_write_bytecode = True
import bl_raster as R  # noqa: E402

MOD = {"pno": "bl_pno", "cust_fields": "bl_custfields"}


def sha(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--method", required=True, choices=sorted(MOD))
    a = ap.parse_args()
    tt = json.loads(Path(f"results/baselines/tuning/{a.method}/tuning_table.json").read_text())
    mod = importlib.import_module(MOD[a.method])
    doc = {"method": a.method, "frozen_utc": time.strftime("%Y-%m-%dT%H:%MZ", time.gmtime()), "stage": "bl B2",
           "selected_by": "configs/baselines/tuning/RULE.md (pre-registered, B2 addendum), applied by bl_harness tunetable",
           "tuning_table": f"results/baselines/tuning/{a.method}/tuning_table.json",
           "tuning_set": "plan §3.5: first 25 WWEST + 15 GAPW1 + 10 S of F3 pilot_pairs.json (disjoint from the F4 5000)",
           "adapter": f"experiments/{MOD[a.method]}.py", "adapter_sha256": sha(f"experiments/{MOD[a.method]}.py"),
           "bl_raster_sha256": sha("experiments/bl_raster.py"), "tuned_on": {}, "common": {}, "robots": {},
           "candidate": {}, "rasters": {}}
    for robot, pick in tt["pick"].items():
        if pick is None:
            raise SystemExit(f"no eligible pick for {robot}")
        cand = json.loads(Path(f"configs/baselines/tuning/{a.method}/{pick}.json").read_text())
        cfg = dict(mod.DEFAULTS, **cand.get("common", {}), **cand.get("robots", {}).get(robot, {}))
        doc["robots"][robot] = cfg
        doc["candidate"][robot] = pick
        t = tt["table"][robot][pick]
        doc["tuned_on"][robot] = {k: t[k] for k in ("n", "SUCCESS", "CLAIMED_COLLIDES", "CLAIMED_UNPROVEN", "FAIL",
                                                     "TIMEOUT", "ERROR", "median_alg_s")}
        for region in ("WWEST", "GAPW1", "S"):
            p = R.raster_path(region, robot, float(cfg["raster_res_m"]))
            side = json.loads(Path(f"{p}.json").read_text())
            if sha(p) != side["sha256"]:
                raise SystemExit(f"{p} differs from its sidecar")
            doc["rasters"][f"{region}_{robot}"] = {"path": str(p), "sha256": side["sha256"],
                                                   "build_wall_s": side["build_wall_s"]}
    out = Path(f"configs/baselines/{a.method}.json")
    out.write_text(json.dumps(doc, indent=1) + "\n")
    print(json.dumps({k: doc[k] for k in ("candidate", "tuned_on")}, indent=1))


if __name__ == "__main__":
    main()
