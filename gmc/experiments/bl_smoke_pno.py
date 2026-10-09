"""bl B0 smoke test: PNO 2D inference with the published pretrained weights, zero-shot (no training).

Replicates the repo's own evaluation, `examples/2DexampleNotebook.ipynb` §3 `test_model` (relative L2 of the
predicted value function vs the FMM ground truth, model output masked by the free-space mask, SDF approximated by
the pretrained FNOSDF and smoothed with `smooth_chi(·, 5)`), on the notebook's "Real-world City 256x256" test set
(all 90 maps: the notebook loads it with trainDataCount=0, testDataCount=1, batch_size=2). Headless: no LaTeX, no
notebook; the notebook's `torch.set_default_device('cuda')` becomes `--device`.

The figure adds one thing the notebook does not do: an 8-neighbour steepest-descent path on the predicted value
function from a free cell to the goal, labelled as a B0 illustration (the paper's 2D planning use is A* with the
PNO value as heuristic, `2D_Neural_Heuristics/heuristics.py`; B2 decides the extraction).

Usage (pno env): python -B bl_smoke_pno.py --repo .../PNO --models DIR --data DIR --out DIR [--device cpu|cuda]
"""
import argparse
import hashlib
import json
import os
import sys
import time

sys.dont_write_bytecode = True

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import torch  # noqa: E402


def sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def smooth_chi(mask, dist, smooth_coef):  # verbatim from the notebook
    return torch.mul(torch.tanh(dist * smooth_coef), (mask - 0.5)) + 0.5


def descend(V, free, start, goal, max_steps=4000):
    """8-neighbour steepest descent on V over free cells (B0 illustration only)."""
    path = [start]
    r, c = start
    for _ in range(max_steps):
        if (c, r) == tuple(goal):
            return path, True
        best, best_v = None, V[r, c]
        for dr in (-1, 0, 1):
            for dc in (-1, 0, 1):
                rr, cc = r + dr, c + dc
                if (dr or dc) and 0 <= rr < V.shape[0] and 0 <= cc < V.shape[1] and free[rr, cc] and V[rr, cc] < best_v:
                    best, best_v = (rr, cc), V[rr, cc]
        if best is None:
            return path, False
        r, c = best
        path.append(best)
    return path, False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--repo", required=True)
    ap.add_argument("--models", required=True)
    ap.add_argument("--data", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--device", default="cuda" if torch.cuda.is_available() else "cpu")
    ap.add_argument("--batch", type=int, default=2)
    ap.add_argument("--time_inst", type=int, default=20)
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    dev = torch.device(a.device)
    torch.set_default_device(dev)  # the notebook does this with 'cuda'; model internals rely on it
    if dev.type == "cpu":
        # DEEPNORM2dMultiGoal.forward feeds `chi.expand(...)` (a stride-0 view) to torch.fft.rfft2; oneMKL rejects
        # that layout on CPU ("DFTI ERROR: Inconsistent configuration parameters", job 19506845) while cuFFT, which
        # the notebook used, accepts it. Same values, contiguous copy first; repo code untouched.
        _rfft2 = torch.fft.rfft2
        torch.fft.rfft2 = lambda x, *args, **kw: _rfft2(x.contiguous(), *args, **kw)
    sys.path.insert(0, os.path.join(a.repo, "examples"))
    from models.deepnormMultiGoal import DEEPNORM2dMultiGoal
    from models.fno import FNO2d
    from utilities.losses import LpLoss

    weights = {}
    def load(model, name):
        p = os.path.join(a.models, name, "best_model.pt")
        weights[name] = {"path": p, "sha256": sha256(p)}
        expect = open(p + ".sha256").read().strip()
        assert weights[name]["sha256"] == expect, f"{p}: SHA-256 mismatch"
        model.load_state_dict(torch.load(p, map_location=dev, weights_only=True))
        return model.to(dev).eval()

    t0 = time.perf_counter()
    # Constructors exactly as in the notebook (cell 11).
    model_sdf = load(FNO2d(4, 1, 8, 8, 16), "FNOSDF")
    models = {"PNO": load(DEEPNORM2dMultiGoal(4, 8, 8, 16), "PNO"),
              "PNOwPINN": load(DEEPNORM2dMultiGoal(4, 8, 8, 16), "PNOwPINN")}
    load_s = time.perf_counter() - t0

    mask_np = np.load(os.path.join(a.data, "mask.npy"))
    goals_np = np.load(os.path.join(a.data, "goal.npy"))
    y_np = np.load(os.path.join(a.data, "output.npy"))
    n, s = mask_np.shape[0], mask_np.shape[1]
    mask = torch.tensor(mask_np, dtype=torch.float, device=dev).reshape(n, s, s, 1)
    goals = torch.tensor(goals_np, dtype=torch.int, device=dev).reshape(n, 2).long()
    y = torch.tensor(y_np, dtype=torch.float, device=dev).reshape(n, s, s, 1)
    print(f"data: {n} maps {s}x{s}, mask values {np.unique(mask_np)[:4]}, goal[0]={goals_np[0]}")

    loss_func = LpLoss(d=2, p=2)
    res = {}
    preds = {}
    with torch.no_grad():
        for name, model in models.items():
            test_loss, samples = 0.0, 0
            for i in range(0, n, a.batch):
                m, g, yy = mask[i:i + a.batch], goals[i:i + a.batch], y[i:i + a.batch]
                chi = smooth_chi(m, model_sdf(m), 5)
                out = model(chi, g) * m  # mask_func = lambda a: a for PNO / PNO w PINN
                if i == 0:
                    preds[name] = out[0, :, :, 0].cpu().numpy()
                test_loss += loss_func(out.view(yy.shape[0], s, s), yy.view(yy.shape[0], s, s)).item()
                samples += yy.shape[0]
            # single-map latency, as the notebook's timing loop (SDF via FNOSDF + value function)
            tsdf = tvf = 0.0
            for _ in range(a.time_inst):
                if dev.type == "cuda":
                    torch.cuda.synchronize()
                t1 = time.perf_counter()
                chi = smooth_chi(mask[:1], model_sdf(mask[:1]), 5)
                if dev.type == "cuda":
                    torch.cuda.synchronize()
                t2 = time.perf_counter()
                out = model(chi, goals[:1]) * mask[:1]
                if dev.type == "cuda":
                    torch.cuda.synchronize()
                t3 = time.perf_counter()
                tsdf += t2 - t1
                tvf += t3 - t2
            res[name] = {"rel_l2_test_loss": test_loss / samples, "maps": samples,
                         "ms_sdf": 1000 * tsdf / a.time_inst, "ms_vf": 1000 * tvf / a.time_inst}
            print(name, res[name])

    # Figure: map 0 -- mask, FMM ground truth, PNO w PINN prediction (+ illustrative descent path), |error|.
    free = mask_np[0] > 0.5
    gx, gy = int(goals_np[0][0]), int(goals_np[0][1])  # model indexes x[b, goal[1], goal[0]] -> (col, row)
    V = preds["PNOwPINN"].copy()
    V[~free] = np.inf
    rng = np.random.default_rng(0)
    cand = np.argwhere(free)
    far = cand[np.hypot(cand[:, 0] - gy, cand[:, 1] - gx) > s / 3]
    start = tuple(far[rng.integers(len(far))]) if len(far) else tuple(cand[0])
    path, reached = descend(V, free, start, (gx, gy))
    path = np.array(path)
    fig, ax = plt.subplots(1, 4, figsize=(16, 4.3))
    ax[0].imshow(mask_np[0], origin="lower", cmap="gray"); ax[0].set_title("city 256 map 0 (white = free)")
    vmax = float(np.nanmax(y_np[0]))
    ax[1].imshow(y_np[0], origin="lower", vmin=0, vmax=vmax); ax[1].set_title("FMM value (ground truth)")
    ax[2].imshow(preds["PNOwPINN"], origin="lower", vmin=0, vmax=vmax)
    ax[2].contour(preds["PNOwPINN"], levels=30, colors="k", linewidths=0.3)
    ax[2].plot(path[:, 1], path[:, 0], "-", color="crimson", lw=1.5,
               label=f"descent on V̂ ({'reached' if reached else 'stuck'})")
    ax[2].legend(loc="lower right", fontsize=7)
    ax[2].set_title(f"PNO w PINN (pretrained, zero-shot)\nrel L2 over 90 maps = {res['PNOwPINN']['rel_l2_test_loss']:.3f}")
    im = ax[3].imshow(np.abs(preds["PNOwPINN"] - y_np[0]), origin="lower", vmin=0, vmax=5); ax[3].set_title("|PNO w PINN − FMM|")
    fig.colorbar(im, ax=ax[3], fraction=0.046)
    for a_ in ax:
        a_.plot(gx, gy, "ro", ms=5)
        a_.set_xticks([]); a_.set_yticks([])
    fig.tight_layout()
    fig_path = os.path.join(a.out, "pno_city256_map0.png")
    fig.savefig(fig_path, dpi=110)

    summary = {"demo": "PNO examples/2DexampleNotebook.ipynb §3 test_model, Real-world City 256x256 test set",
               "device": str(dev), "torch": torch.__version__, "cuda_available": torch.cuda.is_available(),
               "model_load_s": load_s, "weights": weights, "results": res,
               "illustration_descent": {"start_rc": [int(start[0]), int(start[1])], "goal_xy": [gx, gy],
                                        "reached": bool(reached), "cells": int(len(path))},
               "figure": fig_path}
    with open(os.path.join(a.out, "summary.json"), "w") as f:
        json.dump(summary, f, indent=1)
    print(json.dumps(summary, indent=1))


if __name__ == "__main__":
    main()
