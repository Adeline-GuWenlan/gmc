"""P4a structural validation tests (round-6): Atlas.add duplicate guard +
atlas.validate() violation classes, each triggered by a minimal broken
atlas; the reference mini-atlas passes clean."""
import sys
import os

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from splatc.compiler.atlas_types import (
    Atlas, OpenChart, ContactSignature, SectionComponent, GateBranch,
    CertifiedTransition, AtlasEdge)
from test_atlas_api import small_atlas


def test_valid_atlas_has_no_violations():
    assert small_atlas().validate() == []


def test_duplicate_id_raises():
    a = small_atlas()
    with pytest.raises(ValueError, match="duplicate"):
        a.add(OpenChart("chartL", {"bbox": [0, 1, 0, 1]}, certified=True))


def test_dangling_signature_detected():
    a = small_atlas()
    a.components["c0"]["contact_signature_id"] = "sigX"
    assert any("dangling signature" in v for v in a.validate())


def test_ambiguous_with_certificate_detected():
    a = small_atlas()
    a.components["c0"]["status"] = "AMBIGUOUS"
    a.components["c0"]["certified_min_margin"] = 0.003
    assert any("sampled/certified separation" in v for v in a.validate())


def test_same_chart_pseudo_gate_detected():
    a = small_atlas()
    a.edges["e0"]["target_chart"] = "chartL"
    assert any("same-chart pseudo-gate" in v for v in a.validate())


def test_uncertified_chart_on_edge_detected():
    a = small_atlas()
    a.charts["chartR"]["certified"] = False
    assert any("not certified" in v for v in a.validate())


def test_edge_without_witness_detected():
    a = small_atlas()
    a.edges["e0"]["certified_witness_id"] = None
    assert any("no certified witness" in v for v in a.validate())


def test_dangling_witness_detected():
    a = small_atlas()
    a.edges["e0"]["certified_witness_id"] = "tX"
    assert any("dangling witness" in v for v in a.validate())


def test_nonpositive_transition_certificate_detected():
    a = small_atlas()
    a.transitions["t0"]["certificate"]["min_margin_m"] = 0.0
    assert any("non-positive" in v for v in a.validate())


def test_goal_conditioned_provenance_detected():
    a = small_atlas()
    a.compile_provenance = {"note": "ranked candidates by goal distance"}
    assert any("goal-conditioned" in v for v in a.validate())


def test_bad_opposing_pair_detected():
    a = small_atlas()
    a.signatures["sig0"]["opposing_pair"] = [0, 5]
    assert any("opposing_pair" in v for v in a.validate())
    a.signatures["sig0"]["opposing_pair"] = [1, 1]
    assert any("opposing_pair" in v for v in a.validate())


def test_self_transition_detected():
    a = small_atlas()
    a.transitions["t0"]["target_component"] = "c0"
    assert any("self-transition" in v for v in a.validate())


# ---- round-8 witness-chain checks ----------------------------------------

def test_bbox_certified_chart_detected():
    a = small_atlas()
    a.charts["chartL"]["region"] = {"bbox": [-3, -0.7, -2, 2]}
    assert any("no serialized certified cells" in v for v in a.validate())


def test_missing_attachment_detected():
    a = small_atlas()
    a.edges["e0"]["source_attachment_id"] = None
    assert any("missing source_attachment_id" in v for v in a.validate())


def test_attachment_wrong_chart_detected():
    a = small_atlas()
    a.attachments["aL"]["chart_id"] = "chartR"
    assert any("anchors chart chartR" in v for v in a.validate())


def test_attachment_component_off_branch_detected():
    a = small_atlas()
    a.add(SectionComponent("c2", "s2", [[1, 0], [0, 1]], [-0.01, 0.01],
                           [2.9, 3.3], "FREE", [0.5, 0.0, 3.1], 0.05,
                           "sig0"))
    a.attachments["aL"]["component_id"] = "c2"
    assert any("outside branch" in v for v in a.validate())


def test_witness_off_branch_detected():
    a = small_atlas()
    a.branches["b0"]["component_ids"] = ["c1"]
    assert any("not a component of branch" in v for v in a.validate())


def test_branch_chain_gap_detected():
    a = small_atlas()
    del a.transitions["t0"]
    a.edges["e0"]["certified_witness_id"] = None
    assert any("not joined by any certified transition" in v
               for v in a.validate())


def test_adjacent_branches_mirror_detected():
    a = small_atlas()
    a.charts["chartL"]["adjacent_gate_branches"] = []
    assert any("does not list gate_branch" in v for v in a.validate())
    b = small_atlas()
    del b.edges["e0"]
    assert any("no edge joins them" in v for v in b.validate())


def test_pred_succ_bidirectional_detected():
    a = small_atlas()
    a.components["c0"]["successor_ids"] = ["c1"]   # c1 not updated
    assert any("does not list c0" in v for v in a.validate())


def test_transition_endpoint_mismatch_detected():
    a = small_atlas()
    a.transitions["t0"]["connector_wps"][0] = [0.4, 0.0, 3.1]
    assert any("endpoint does not match" in v for v in a.validate())


def test_portal_violations_detected():
    from splatc.compiler.atlas_types import OpenPortal
    a = small_atlas()
    a.add(OpenPortal("p0", "chartL", "chartL",
                     [[-1.0, 0, 0], [1.0, 0, 0]],
                     {"min_margin_m": 0.0, "checks": 2, "checker": "x"}))
    v = a.validate()
    assert any("same-chart portal" in s for s in v)
    assert any("portal p0: missing/non-positive" in s for s in v)


# ---- round-10 adversarial mutations: all three passed the round-9
# validator silently (review §5); each must now be CAUGHT ------------------

def test_r10_geometric_garbage_attachment_caught():
    a = small_atlas()
    a.attachments["aL"]["waypoints"] = [[99, 99, 0], [88, 88, 0]]
    v = a.validate()
    assert any("not inside chart" in s for s in v)
    assert any("does not match component anchor" in s for s in v)


def test_r10_forged_portal_caught():
    from splatc.compiler.atlas_types import OpenPortal
    a = small_atlas()
    a.add(OpenPortal("pX", "chartL", "chartR",
                     [[99, 99, 0], [88, 88, 0]],
                     {"min_margin_m": 0.05, "checks": 3,
                      "checker": "bubble/checker2",
                      "certified_tube_radius_m": 0.02}))
    v = a.validate()
    assert any("portal pX: endpoint" in s and "not" in s for s in v)


def test_r10_corrupt_region_caught():
    a = small_atlas()
    a.charts["chartL"]["region"]["cell_cert_slack_m"] = [-1.0]
    a.charts["chartL"]["region"]["adjacency"] = [[0, 99]]
    v = a.validate()
    assert any("non-positive obstacle cert slack" in s for s in v)
    assert any("adjacency index" in s for s in v)


def test_r10_fake_adjacency_edge_caught():
    a = small_atlas()
    reg = a.charts["chartL"]["region"]
    # two far-apart level-1 cells that do NOT share a face
    reg["cells"] = [[1, 0, 0, 0], [1, 1, 1, 1]]
    reg["cell_cert_slack_m"] = [0.05, 0.05]
    reg["adjacency"] = [[0, 1]]
    assert any("do not share a face" in s for s in a.validate())


def test_r10_unbound_certificates_caught():
    a = small_atlas()
    a.checker_id = ""
    a.scene_hash = ""
    assert any("unbound" in s for s in a.validate())
    b = small_atlas()
    b.portals = {}
    b.attachments["aL"]["certificate"]["checker"] = "someone-else"
    assert any("does not match atlas checker_id" in s
               for s in b.validate())


def test_r10_broken_edge_chain_caught():
    a = small_atlas()
    a.add(SectionComponent("c2", "s2", [[1, 0], [0, 1]], [-0.01, 0.01],
                           [2.9, 3.3], "FREE", [0.01, 0.0, 3.1], 0.05,
                           "sig0"))
    a.branches["b0"]["component_ids"] = ["c0", "c2", "c1"]
    # witness still joins c0-c1 (endpoints) so chain check passes; now
    # re-anchor an attachment to the MIDDLE component
    a.attachments["aL"]["component_id"] = "c2"
    a.attachments["aL"]["waypoints"][-1] = [0.01, 0.0, 3.1]
    v = a.validate()
    assert any("not a branch ENDPOINT" in s for s in v)
