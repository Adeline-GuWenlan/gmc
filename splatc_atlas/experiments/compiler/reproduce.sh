#!/bin/bash
# Reproduce every canonical artifact in the round-4 final package from the
# packaged source snapshot.  Deterministic (no RNG); pose-query counts and
# margins reproduce exactly; wall-clock varies by host.
#
# Environment: python 3.11, numpy 1.26.4, scipy 1.10.*, matplotlib
# (see environment.txt).  Run from the package's src_snapshot/ root:
#   bash experiments/compiler/reproduce.sh
set -eu
export PYTHONPATH=src

python -m pytest tests/ -q                                   # full suite (see round-8 report for count)
python experiments/compiler/ridge_continuation.py            # canonical sweep
python experiments/compiler/tagfree_validation.py            # P1a bitwise check
python experiments/compiler/p2_generalization.py             # 42-case within-family
python experiments/compiler/gamma_delta.py                   # Gamma(delta) 3 arms
python experiments/compiler/p3_matched_budget.py             # Gate B + controls
python experiments/compiler/p36_fairness.py                  # 2x2 + pair-aware + cuts
python experiments/compiler/p37_matched.py                   # P3.7 matched arms
python experiments/compiler/p38_connector_bench.py           # P3.8 chart-factored connectors
python experiments/compiler/p4a_semantic.py                  # P4a.2 reachability consistency
python experiments/compiler/p4a3_validation.py               # P4a.3 sealed validation set
python experiments/compiler/ridge_continuation_scalar.py     # scalar ablation
python experiments/compiler/witness_upgrade.py               # oracle amendments
python experiments/compiler/floor_probe.py                   # oracle-tube bracket
python experiments/compiler/ridge_diag_plots.py              # figures (+sidecars)
python experiments/compiler/gamma_delta_plots.py
python experiments/compiler/p3_plots.py
python experiments/compiler/p36_plots.py                     # fairness suppl.
python experiments/compiler/p37_plots.py                     # matched arms fig.

echo "reproduction complete; now run BOTH gate layers:"
echo "  python <package_dir>/validate_package.py <package_dir>"
echo "  python <package_dir>/compare_reproduction.py <package_dir> <this src_snapshot dir>"
