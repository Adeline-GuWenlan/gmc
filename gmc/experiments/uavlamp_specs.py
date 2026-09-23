"""Write the Task-3 single-query specs for the booth derivative (one planner call each).

M1 main (low start) · M5 high start · N2 necessity (test-only plug under the lamp)
· P3* leak-probe goals at would-be bypasses · C4/C4h counterfactual (lamp only)
· B6 box open-air face pushed out by 1 m.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

BIG = {"max_wall_s": 7200., "max_expansions": 2_000_000, "max_oracle_calls": 50_000_000,
       "max_narrowphase_pairs": 1_000_000_000}
NECESSITY = {**BIG, "max_wall_s": 20700.}


def main(argv=None):
    p = argparse.ArgumentParser()
    p.add_argument("--config", type=Path, default=Path("configs/uavlamp_scene.json"))
    p.add_argument("--scene", type=Path, default=Path("outputs/uavlamp/scene"))
    p.add_argument("--out", type=Path, default=Path("outputs/uavlamp/specs"))
    p.add_argument("--version", default="v1")
    a = p.parse_args(argv)
    cfg = json.loads(a.config.read_text())
    q = cfg["query"]
    lamp = q["lamp"]
    ul = (lamp["footprint_route_uv"][0][0] + lamp["footprint_route_uv"][1][0]) / 2
    zl = lamp["underside_z"]
    vmid = (lamp["footprint_route_uv"][0][1] + lamp["footprint_route_uv"][1][1]) / 2
    vlen = lamp["footprint_route_uv"][1][1] - lamp["footprint_route_uv"][0][1]
    base = {"archive": str(a.scene / "uavlamp_scene.npz"), "manifest": str(a.scene / "manifest.json"),
            "frame": cfg["frame"], "box_route": q["box_route"], "start_route": q["start_route"],
            "goal_route": q["goal_route"], "resolution_m": q["resolution_m"], "margin_m": q["margin_m"],
            "budget": BIG, "lamp": lamp, "table": q["table"], "top_band_z": [0., 2.45],
            "side_slab_v": [q["box_route"]["lower"][1], q["box_route"]["upper"][1]]}
    plug = {"edit_id": "plug_under_lamp", "role": "test_only_plug", "kind": "panel",
            "center_route": [ul, vmid, zl / 2], "plane": "vz", "size_m": [vlen, zl],
            "spacing_m": .08, "half_thickness_m": .02, "color_rgb": [1., 0., 0.], "opacity": .95,
            "justification": "TEST ONLY: fills the under-lamp opening; never shipped",
            "contact": "floor to lamp underside, wall A to wall B, in the lamp's centre plane"}
    pushed = {"lower": [q["box_route"]["lower"][0] - 1.0, *q["box_route"]["lower"][1:]],
              "upper": q["box_route"]["upper"]}
    specs = {
        "M1_main_low_start": {},
        "M5_high_start": {"start_route": q["high_start_route"]},
        "N2_necessity_plug": {"extra_builders": [plug], "budget": NECESSITY},
        "P3a_goal_above_lamp_in_bulkhead": {"goal_route": [ul, 1.2, 1.80]},
        "P3b_goal_lamp_top_bulkhead_seam": {"goal_route": [ul, 1.2, zl + .17]},
        "P3c_goal_beside_lamp_wallA": {"goal_route": [ul, .15, zl + .07]},
        "P3d_goal_beside_lamp_wallB": {"goal_route": [ul, 2.20, zl + .07]},
        "C4_counterfactual_lamp_only_low": {"drop_roles": ["drop_ceiling", "bulkhead", "back_panel"]},
        "C4h_counterfactual_lamp_only_high": {"drop_roles": ["drop_ceiling", "bulkhead", "back_panel"],
                                              "start_route": q["high_start_route"]},
        "B6_box_umin_pushed_1m": {"box_route": pushed},
        "B6h_box_umin_pushed_1m_high": {"box_route": pushed, "start_route": q["high_start_route"]},
    }
    a.out.mkdir(parents=True, exist_ok=True)
    for name, over in specs.items():
        s = {**base, **over, "name": f"{a.version}_{name}",
             "output": f"outputs/uavlamp/t3_{a.version}/{name}"}
        (a.out / f"{a.version}_{name}.json").write_text(json.dumps(s, indent=1))
        print(a.out / f"{a.version}_{name}.json")


if __name__ == "__main__":
    main()
