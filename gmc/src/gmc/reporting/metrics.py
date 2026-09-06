"""Run metrics counters (Guide §15.3 Efficiency dimension)."""
import json
from pathlib import Path


def collect_metrics(mc, decomposition, wall_s: float, *, pair_stats=None,
                    ledger=None) -> dict:
    total_pairs = len(mc.scene.supports) * len(mc.robot.supports)
    metrics = {
        "support_calls_total": int(sum(o.calls for o in mc.oracles)),
        "support_value_evals": int(sum(getattr(o, "value_calls", 0)
                                           for o in mc.oracles)),
        "support_point_evals": int(sum(getattr(o, "point_calls", 0)
                                           for o in mc.oracles)),
        "n_pairs_total": total_pairs,
        "n_pairs_retained": len(mc.oracles),
        "n_pairs_pruned": total_pairs - len(mc.oracles),
        "n_slices": decomposition.n_slices,
        "n_slabs": len(decomposition.slabs),
        "n_slabs_regular": sum(s.kind == "regular"
                               for s in decomposition.slabs),
        "n_slabs_uncertain": sum(s.kind == "uncertain"
                                 for s in decomposition.slabs),
        "safe_nodes": mc.M_safe.number_of_nodes(),
        "safe_edges": mc.M_safe.number_of_edges(),
        "possible_nodes": mc.M_possible.number_of_nodes(),
        "possible_edges": mc.M_possible.number_of_edges(),
        "compile_wall_seconds": wall_s,
    }
    if pair_stats is not None:
        metrics["bvh"] = {
            "tree_nodes": pair_stats.tree_nodes,
            "tree_depth": pair_stats.tree_depth,
            "nodes_visited": pair_stats.nodes_visited,
            "nodes_pruned": pair_stats.nodes_pruned,
            "pair_tests": pair_stats.pair_tests,
            "pruned_pairs": pair_stats.pruned_pairs,
        }
    if ledger is not None:
        metrics["work_ledger"] = ledger.snapshot()
    return metrics


def write_metrics(run_dir: Path, metrics: dict) -> None:
    (Path(run_dir) / "metrics.json").write_text(json.dumps(metrics, indent=2))
