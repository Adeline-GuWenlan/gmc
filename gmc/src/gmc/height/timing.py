"""Per-stage wall-clock timing for the height pipeline (Amendment 3 §P1e).

Task 4 of Amendment 3 asks where the wall clock actually goes: denoise/scene-prep → projection →
compile → query → certification/connectivity, and what that says about the algorithm's efficiency.
P4 answers it by fitting time against input size, so a stage that is not measured *with its input
size* is a stage P4 cannot say anything about. Hence :meth:`StageTimer.stage` takes the sizes with
the measurement, and :meth:`StageTimer.to_dict` names the stages that never ran rather than
silently omitting them.

The stage list is closed on purpose. A typo like ``compile_paris`` would otherwise become a new
stage that P4 aggregates separately and nobody notices, so an unknown name raises.

Threading, and what it is not allowed to disturb: :func:`gmc.height.run.compile_and_query` and
``experiments/showcase_run.py`` accept an optional timer and are unchanged without one, and
``compile_seconds`` / ``query_seconds`` keep the exact meaning they already had -- the sub-stage
records are additional, not a redefinition. No read-only module is touched; the compile sub-stages
are timed around the calls that ``run.py`` already makes.
"""
import time
from contextlib import contextmanager

#: The stages Amendment 3 §P1e names, in pipeline order.
STAGES = ("scene_load", "floor_replace", "project", "compile_pairs", "compile_slabs",
          "compile_mobility", "query", "verify_curve", "replay3d")


class StageTimer:
    """Collects one record per stage run: name, optional label, seconds, and input sizes.

    ``label`` distinguishes repeats of the same stage -- ``project`` runs once per robot, and P4
    needs the three apart -- while ``by_stage`` totals still aggregate over them.
    """

    def __init__(self, clock=time.perf_counter):
        self._clock = clock
        self.records = []

    def record(self, stage, seconds, *, label=None, failed=False, **sizes):
        """Add a measurement taken elsewhere (a probe, or a stage timed by existing code)."""
        if stage not in STAGES:
            raise ValueError(f"unknown stage {stage!r}; expected one of {', '.join(STAGES)}")
        rec = {"stage": stage, "label": label, "seconds": float(seconds),
               "failed": bool(failed), "sizes": dict(sizes)}
        self.records.append(rec)
        return rec

    @contextmanager
    def stage(self, stage, *, label=None, **sizes):
        """Time a block. Yields the record so sizes only known afterwards can be added to it.

        The record is kept even if the block raises -- a stage that blew its budget is exactly the
        one P4 wants the time for -- and the exception propagates unchanged.
        """
        if stage not in STAGES:
            raise ValueError(f"unknown stage {stage!r}; expected one of {', '.join(STAGES)}")
        rec = {"stage": stage, "label": label, "seconds": 0.0, "failed": False,
               "sizes": dict(sizes)}
        self.records.append(rec)
        t0 = self._clock()
        try:
            yield rec
        except BaseException:
            rec["failed"] = True
            raise
        finally:
            rec["seconds"] = float(self._clock() - t0)

    def by_stage(self):
        out = {}
        for rec in self.records:
            out[rec["stage"]] = out.get(rec["stage"], 0.0) + rec["seconds"]
        return out

    def to_dict(self):
        """The JSON block every downstream run writes into its result."""
        by = self.by_stage()
        return {"stages": list(self.records), "by_stage": by,
                "total_seconds": float(sum(by.values())),
                "missing_stages": [s for s in STAGES if s not in by],
                "stage_order": list(STAGES)}
