# A2 — airborne showcase scene (compute checkpoint)

The committed generator prepares a deterministic derivative of the existing, manually
floor-edited 7M-Gaussian showcase. It appends exactly three labelled manual opaque
Gaussians: the hanging shade, suspension, and ceiling attachment. It retains the
source table-A Gaussian arrangement, labels its complete 3-D support selection, and
does not modify either source asset.

The tiny synthetic companion was generated and opened at
`gmc/results/gs3d_scene/tiny_companion_scale.png`; it confirms the edit scale and the
low-then-high witness geometry. Unit tests passed: `2 passed` for
`tests/unit/test_gs3d_scene.py`.

Full source build and actual EWA Gaussian renders are running as compute job `18232908`
through `compute/A2_scene_build.sh`. On completion it will write the ignored,
worktree-local archive, manifest, render manifest and three images under
`gmc/results/gs3d_scene/`. The script loads that same derivative back and fails if the
archive/manifest rows or SHA-256 disagree. The candidate clearance computation is
explicitly an A0-style conservative enclosing-box feasibility witness, not a planner
success claim; A1/A5 retain authority for the 3-D body oracle and planning validation.

Reproduce after the job has completed:

```bash
python3 "$GS3D_ROOT/submit_compute.py" --stage A2 \
  --script "$GS3D_ROOT/compute/A2_scene_build.sh" --cpus 2 --mem-gb 8 --hours 1
```
