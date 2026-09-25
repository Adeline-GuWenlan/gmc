# External repo envs — install status
_Generated 2026-09-24 16:17 by `/scratch/wg2381/ext_repos/env_setup/check_envs.py`._
Repos: `/scratch/wg2381/ext_repos/<name>`; envs: `conda activate /scratch/wg2381/.conda/envs/<name>`; install logs: `/scratch/wg2381/ext_repos/env_setup/logs/`.

## Summary

| env | status |
|---|---|
| cust_fields | env missing |
| pno | env missing |
| foci | MISSING: open3d, MA27 |
| splatnav | env missing |

## cust_fields
**ENV NOT CREATED** — see install log.

## pno
**ENV NOT CREATED** — see install log.

## foci
| module | required | status | version / error |
|---|---|---|---|
| `foci` | yes | ✅ |  |
| `numpy` | yes | ✅ | 1.26.4 |
| `open3d` | yes | ❌ | ImportError: libEGL.so.1: cannot open shared object file: No such file or directory |
| `matplotlib` | yes | ✅ | 3.10.9 |
| `viser` | yes | ✅ | 1.1.1 |
| `plyfile` | yes | ✅ |  |
| `scipy` | yes | ✅ | 1.15.3 |
| `astar` | yes | ✅ |  |
| `casadi` | yes | ✅ | 3.8.1 |
| `warp` | yes | ✅ | 1.17.0 |
| `sklearn` | yes | ✅ | 1.7.2 |
| `pandas` | yes | ✅ | 2.3.3 |
| `pxr` | optional | ✅ |  |

Extra checks:

```
MA27 FAIL Invalid_Option
LFS OK 34431123 bytes
```

## splatnav
**ENV NOT CREATED** — see install log.

## Known manual items (cannot be automated)

- **foci / HSL MA27**: foci hard-codes `ipopt.linear_solver: ma27` (`foci/optim/solvers.py:196`,
  `foci/optim/initial_guess.py:135`, `foci/convolution/gaussian_robot_warp.py:398`). MA27 needs a licensed
  coin-hsl download (https://licences.stfc.ac.uk/product/coin-hsl, free academic). Either get it and build
  `libcoinhsl.so` onto `LD_LIBRARY_PATH`, or change those three lines to `"mumps"` (bundled with pip casadi).
- **splatnav / bpy, mathutils**: only for `blender/` scripts; needs Blender's own Python — optional.
- **splatnav / trained splats + data**: scripts expect `outputs/<scene>/splatfacto/.../config.yml`; download per its README.
- **PNO / datasets + models**: from Hugging Face per its README.
