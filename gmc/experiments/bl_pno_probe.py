"""bl B2 probe (pno env, GPU): PNO's value-function unit and its SDF approximator's unit, zero-shot.

1. Empty S x S maps (S = 64, 256, 1024, 2048), goal at the centre: V at cells d away along a row -> V per cell. The
   adapter converts V to cells by S / 64 (V per cell = 64 / S); this checks that on the published weights.
2. City 256 test map 0 (the notebook's data): FNOSDF(mask) vs the exact EDT of the mask in cells -> SDF unit.
Writes ``results/baselines/pno/probe_units.json``.
"""
import json
import os
import sys
from pathlib import Path

import numpy as np
import torch
from scipy.ndimage import distance_transform_edt

sys.dont_write_bytecode = True
sys.path.insert(0, str(Path(__file__).resolve().parent))
import bl_pno as P  # noqa: E402


def main():
    dev = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    torch.set_default_device(dev)
    if dev.type == "cpu":
        _r = torch.fft.rfft2
        torch.fft.rfft2 = lambda x, *a, **k: _r(x.contiguous(), *a, **k)
    sys.path.insert(0, os.path.join(P.REPO, "examples"))
    from models.deepnormMultiGoal import DEEPNORM2dMultiGoal
    from models.fno import FNO2d
    sdf_m = FNO2d(4, 1, 8, 8, 16)
    sdf_m.load_state_dict(torch.load(P.MODELS / "FNOSDF/best_model.pt", map_location=dev, weights_only=True))
    sdf_m = sdf_m.to(dev).eval()
    out = {"device": str(dev), "empty": {}, "city0": {}}
    for name in ("PNO", "PNOwPINN"):
        m = DEEPNORM2dMultiGoal(4, 8, 8, 16)
        m.load_state_dict(torch.load(P.MODELS / name / "best_model.pt", map_location=dev, weights_only=True))
        m = m.to(dev).eval()
        for S in (64, 256, 1024, 2048):
            with torch.no_grad():
                mask = torch.ones(1, S, S, 1)
                # border cells occupied (a closed room), as the training maps have
                mask[:, 0, :] = mask[:, -1, :] = mask[:, :, 0] = mask[:, :, -1] = 0
                chi = torch.mul(torch.tanh(sdf_m(mask) * 5.), (mask - .5)) + .5
                c = S // 2
                V = m(chi, torch.tensor([[c, c]], dtype=torch.long)).cpu().numpy().reshape(S, S)
            ds = [d for d in (S // 16, S // 8, S // 4, 3 * S // 8)]
            per_cell = [float(V[c, c + d] / d) for d in ds] + [float(V[c + d, c] / d) for d in ds]
            out["empty"][f"{name}_{S}"] = {"V_per_cell_along_row_and_col": per_cell, "expected_64_over_S": 64 / S,
                                           "V_goal": float(V[c, c])}
            print(name, S, per_cell, 64 / S, flush=True)
    d = P.GMC / "outputs/baselines/pno/dataset/cityData/256x256"
    mk = np.load(d / "mask.npy")[0].astype(np.float32)
    with torch.no_grad():
        s = sdf_m(torch.tensor(mk).reshape(1, 256, 256, 1)).cpu().numpy().reshape(256, 256)
    edt = distance_transform_edt(mk)
    free = mk > 0
    ratio = s[free & (edt > 2)] / edt[free & (edt > 2)]
    out["city0"] = {"fnosdf_over_edt_cells_p10_p50_p90": [float(x) for x in np.percentile(ratio, [10, 50, 90])],
                    "expected_if_units_are_map_width_64": 64 / 256}
    print(out["city0"], flush=True)
    o = P.GMC / "results/baselines/pno/probe_units.json"
    o.parent.mkdir(parents=True, exist_ok=True)
    o.write_text(json.dumps(out, indent=1) + "\n")


if __name__ == "__main__":
    main()
