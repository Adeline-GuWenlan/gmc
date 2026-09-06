"""M5 predicates and event brackets (Guide §8.2–8.3).

v0 predicate ladder:
  P0 candidate pair set change  — v0 pruning is theta-independent: constant.
  P1 union component counts     — left/mid/right slices, safe and possible.
  P2 component lineage          — bijective overlap matching on both sides.
  P3 event bracket              — discrete-verdict bisection to theta_min;
                                  a found root is an isolated event bracket,
                                  an absent sign change is NOT proof of no
                                  event (§8.3).
"""
from dataclasses import dataclass

from ..spatial.arrangement import match_components
from .intervals import Interval


@dataclass(frozen=True)
class PredicateReport:
    pair_set_constant: bool
    counts_match: bool
    lineage_bijective_safe: bool
    lineage_bijective_possible: bool
    details: dict
    # The v0 predicates are evaluated at only three orientations.  Passing
    # them is useful evidence for building candidate SAFE routes, but it is
    # not an interval proof that no event occurs between the samples.
    interval_certified: bool = False
    uncertainty_sources: tuple = (
        "left_mid_right_sampling_has_no_interval_event_bound",
    )

    @property
    def all_pass(self) -> bool:
        return (self.pair_set_constant and self.counts_match
                and self.lineage_bijective_safe
                and self.lineage_bijective_possible)

    @property
    def regular_candidate(self) -> bool:
        """Whether the finite-sample predicates support a candidate slab."""
        return self.all_pass

    @property
    def certifies_no_event(self) -> bool:
        """Whether this report proves the entire interval event-free."""
        return self.all_pass and self.interval_certified


def collect_predicates(left_slice, mid_slice, right_slice, *,
                       ledger=None) -> PredicateReport:
    counts = {
        "safe": (len(left_slice.D_safe), len(mid_slice.D_safe),
                 len(right_slice.D_safe)),
        "possible": (len(left_slice.D_possible), len(mid_slice.D_possible),
                     len(right_slice.D_possible)),
    }
    counts_match = (len(set(counts["safe"])) == 1
                    and len(set(counts["possible"])) == 1)
    _, _, _, bij_s1 = match_components(left_slice.D_safe, mid_slice.D_safe,
                                       ledger=ledger)
    _, _, _, bij_s2 = match_components(mid_slice.D_safe, right_slice.D_safe,
                                       ledger=ledger)
    _, _, _, bij_p1 = match_components(left_slice.D_possible,
                                       mid_slice.D_possible, ledger=ledger)
    _, _, _, bij_p2 = match_components(mid_slice.D_possible,
                                       right_slice.D_possible, ledger=ledger)
    return PredicateReport(
        pair_set_constant=True,
        counts_match=counts_match,
        lineage_bijective_safe=bij_s1 and bij_s2,
        lineage_bijective_possible=bij_p1 and bij_p2,
        details={
            "counts": counts,
            "sample_thetas": (
                float(left_slice.theta),
                float(mid_slice.theta),
                float(right_slice.theta),
            ),
            "evidence_scope": "finite_left_mid_right_samples_only",
        })


def bracket_event(build_slice_at, interval: Interval, differs,
                  theta_min: float) -> Interval:
    """Bisect a discrete verdict change to width <= theta_min.

    `differs(slice_a, slice_b) -> bool` compares endpoint slices; the caller
    guarantees differs(left, right) is True on entry."""
    lo, hi = interval.lo, interval.hi
    s_lo = build_slice_at(lo)
    while hi - lo > theta_min:
        mid = 0.5 * (lo + hi)
        s_mid = build_slice_at(mid)
        if differs(s_lo, s_mid):
            hi = mid
        else:
            lo, s_lo = mid, s_mid
    return Interval(lo, hi)
