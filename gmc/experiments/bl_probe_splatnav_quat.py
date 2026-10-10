"""bl B0 probe: for which quaternions does SplatNav's quaternion_to_rotation_matrix return NaN? (splatnav env, CPU)"""
import sys

sys.dont_write_bytecode = True
import numpy as np
import torch

sys.path.insert(0, sys.argv[1] if len(sys.argv) > 1 else "/scratch/wg2381/ext_repos/splatnav")
from ellipsoids.covariance_utils import quaternion_to_rotation_matrix

for dt in (torch.float32, torch.float64):
    for axis in "zx":
        for a in [0.0, 1e-12, 1e-8, 1e-6, 1e-4, 1e-2, np.pi / 2, np.pi, 2 * np.pi]:
            v = [0.0, 0.0, 0.0]
            v["xyz".index(axis)] = np.sin(a / 2)
            R = quaternion_to_rotation_matrix(torch.tensor([[np.cos(a / 2), *v]], dtype=dt))[0]
            fin = bool(torch.isfinite(R).all())
            err = float((R @ R.T - torch.eye(3, dtype=dt)).abs().max()) if fin else float("nan")
            print(f"{str(dt):14s} axis={axis} angle={a:<20.12g} {'finite' if fin else 'NaN':6s} max|RR^T-I|={err:.2e}")
    for q in ([1.0, 0, 0, 0], [-1.0, 0, 0, 0], [2.0, 0, 0, 0]):
        R = quaternion_to_rotation_matrix(torch.tensor([q], dtype=dt))
        print(f"{str(dt):14s} q={q} finite={bool(torch.isfinite(R).all())}")
