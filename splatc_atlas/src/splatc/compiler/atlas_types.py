"""Atlas data structures (P4a; schema RE-OPENED by round-8 review — the
round-5 "frozen" label is withdrawn until the chart layer passes its
semantic-invariance acceptance).

Round-8 additions:
  - OpenPortal: broad-clearance transition between two OpenCharts,
    certified by a generic local connector (splatc.compiler.portals).
    A refinement-artifact split between charts is NOT a contact gate;
    portals keep it semantically connected without inventing one.
  - ChartAttachment: certified polyline from a chart's interior to a
    gate-side component — the missing links that make an AtlasEdge a
    verified chain (chart -> branch endpoint -> ... -> chart) instead of
    a typed claim.
  - validate() now checks the whole edge witness chain (see checklist in
    the method), the bidirectional predecessor/successor consistency,
    and that certified=True charts actually carry a serialized certified
    cell region (a bare bbox is a boolean claim, not a region).

Object kinds; contracts they encode:
  - sampled_* and certified_* fields are SEPARATE: a sampled observation
    never silently becomes a certificate.
  - Section components carry tri-state status; only AMBIGUOUS-free claims
    that passed continuous certification may fill certified_min_margin.
  - Component adjacency enters the graph ONLY through a
    CertifiedTransition (continuous connector certificate).
  - A GateBranch becomes a semantic gate ONLY when an AtlasEdge links two
    DIFFERENT OpenCharts through it (val_016's jamb branch stays a real
    local contact structure, never upgraded).
  - Multiple bilateral hypotheses coexist (no global two-sides
    assumption); compile stores ALL branches — no goal-specific pruning.
  - Serialization is canonical (sorted keys) and atlas_hash() is the
    goal-independence witness: querying different goals MUST leave it
    unchanged (enforced by tests/test_atlas_api.py).
"""
from __future__ import annotations

import hashlib
import json
from dataclasses import dataclass, field, asdict
from typing import Optional


@dataclass
class ContactSignature:
    signature_id: str
    # each cluster: {"normal_cone": [cx, cy, half_angle_rad],
    #               "strength_range": [h_lo, h_hi],
    #               "support": "spatial locality note / primitive count"}
    clusters: list
    opposing_pair: list                 # two cluster indices forming a gap
    predecessor_id: Optional[str] = None
    successor_id: Optional[str] = None
    decomposition_provenance: str = ""  # primitive organization this saw


@dataclass
class SectionComponent:
    component_id: str
    section_id: str
    local_frame: list                   # [[tx, ty], [nx, ny]] at anchor
    position_interval: list             # [n_lo, n_hi] along normal (m)
    orientation_interval: list          # [th_lo, th_hi] (rad)
    status: str                         # FREE | COLLISION | AMBIGUOUS
    sampled_best_pose: list             # [x, y, th]
    sampled_min_margin: float           # PW, sampled — NOT a certificate
    contact_signature_id: str
    certified_min_margin: Optional[float] = None   # only via certification
    predecessor_ids: list = field(default_factory=list)
    successor_ids: list = field(default_factory=list)


@dataclass
class GateBranch:
    branch_id: str
    component_ids: list                 # ordered along the branch
    events: list                        # {"kind": birth|split|merge|death,
    #                                      "section_id": ..., ...}
    validity_interval: list             # [s_lo, s_hi] arc range observed
    min_certified_margin: Optional[float] = None


@dataclass
class CertifiedTransition:
    transition_id: str
    source_component: str
    target_component: str
    connector_wps: list                 # polished pose polyline
    certificate: dict                   # {"min_margin_m":, "checks":,
    #                                     "checker": "..."}
    cost: float = 0.0


@dataclass
class OpenChart:
    chart_id: str
    region: dict                        # coarse certified-free description
    certified: bool = False
    frontier_ids: list = field(default_factory=list)
    representative_poses: list = field(default_factory=list)
    adjacent_gate_branches: list = field(default_factory=list)


@dataclass
class OpenPortal:
    portal_id: str
    chart_a: str                        # canonical: chart_a < chart_b
    chart_b: str
    waypoints: list                     # certified polyline, pose in
    #                                     chart_a -> pose in chart_b
    certificate: dict                   # {"min_margin_m":, "checks":,
    #   "checker":, "certified_tube_radius_m": ...}
    # round-10: a portal is a PROOF SOURCE (generic certified connector
    # with a certified tube), not a topology class — its absence proves
    # nothing and its presence does not claim "broad clearance"
    kind: str = "generic_certified_connector"


@dataclass
class ChartAttachment:
    attachment_id: str
    chart_id: str
    component_id: str                   # gate-side anchor component
    waypoints: list                     # certified polyline, chart
    #                                     interior -> component pose
    certificate: dict


@dataclass
class AtlasEdge:
    edge_id: str
    source_chart: str
    gate_branch: str
    target_chart: str
    cost: float
    certified_witness_id: Optional[str] = None
    source_attachment_id: Optional[str] = None
    target_attachment_id: Optional[str] = None


@dataclass
class Atlas:
    charts: dict = field(default_factory=dict)
    signatures: dict = field(default_factory=dict)
    components: dict = field(default_factory=dict)
    branches: dict = field(default_factory=dict)
    transitions: dict = field(default_factory=dict)
    portals: dict = field(default_factory=dict)
    attachments: dict = field(default_factory=dict)
    edges: dict = field(default_factory=dict)
    compile_provenance: dict = field(default_factory=dict)
    compile_query_count: int = 0        # frozen after compile; queries at
    #                                     query time MUST NOT increase it
    # round-10 identity binding: certificates are meaningless without
    # the scene/robot/checker they were computed against
    scene_hash: str = ""
    robot_hash: str = ""
    domain_hash: str = ""               # "" = identity/none
    checker_id: str = ""

    def add(self, obj):
        table = {ContactSignature: (self.signatures, "signature_id"),
                 SectionComponent: (self.components, "component_id"),
                 GateBranch: (self.branches, "branch_id"),
                 CertifiedTransition: (self.transitions, "transition_id"),
                 OpenPortal: (self.portals, "portal_id"),
                 ChartAttachment: (self.attachments, "attachment_id"),
                 OpenChart: (self.charts, "chart_id"),
                 AtlasEdge: (self.edges, "edge_id")}[type(obj)]
        d, key = table
        oid = getattr(obj, key)
        if oid in d:
            # round-6 P4a: silent overwrite let a later object shadow an
            # earlier one with no trace — now a hard error
            raise ValueError(f"duplicate {key}: {oid}")
        d[oid] = asdict(obj)

    # round-6 P4a: structural integrity checker.  Returns a list of
    # violation strings (empty = valid).  Mirrors the review's checklist:
    # dangling references, uncertified edges, same-chart pseudo-gates,
    # AMBIGUOUS components carrying certificates, goal-conditioned
    # provenance.  Gate semantics are chart-attachment based: an AtlasEdge
    # is the ONLY thing that makes a GateBranch a semantic gate, and it
    # must join two DIFFERENT certified OpenCharts through a certified
    # witness.  (No x-span or any frame-conditioned geometric test here.)
    _GOALLIKE = ("start", "goal")

    # ---- round-10 semantic validators (adversarial-mutation review:
    # a wrong-geometry attachment, a forged portal and a corrupt region
    # all passed the old reference-existence checks silently) ----------

    def _region_contains(self, chart, pose):
        from .chart_builder import chart_contains
        try:
            return chart_contains(chart["region"], pose)
        except Exception:
            return None

    def _validate_regions(self, v):
        from .chart_builder import cell_bounds
        for chid, ch in self.charts.items():
            if not ch.get("certified"):
                continue
            reg = ch.get("region") or {}
            cells = reg.get("cells") or []
            grid = reg.get("grid") or {}
            slack = reg.get("cell_cert_slack_m")
            if slack is None or len(slack) != len(cells):
                v.append(f"chart {chid}: cell_cert_slack_m missing or "
                         f"length != cells")
            elif any(s is None or s <= 0 for s in slack):
                v.append(f"chart {chid}: non-positive obstacle cert "
                         f"slack in region")
            dsl = reg.get("cell_domain_slack_m")
            if reg.get("domain") is not None:
                if dsl is None or len(dsl) != len(cells):
                    v.append(f"chart {chid}: domain present but "
                             f"cell_domain_slack_m missing/short")
                elif any(s is None or s <= 0 for s in dsl):
                    v.append(f"chart {chid}: non-positive domain "
                             f"support slack in region")
            n = len(cells)
            for e in reg.get("adjacency") or []:
                if len(e) != 2 or not all(
                        isinstance(i, int) and 0 <= i < n for i in e):
                    v.append(f"chart {chid}: adjacency index {e} out of "
                             f"range")
                    continue
                if grid and not self._cells_face_adjacent(
                        grid, cells[e[0]], cells[e[1]]):
                    v.append(f"chart {chid}: adjacency edge {e} joins "
                             f"cells that do not share a face")

    @staticmethod
    def _cells_face_adjacent(grid, ca, cb):
        import numpy as np
        root = np.asarray(grid["root"])
        origin = np.asarray(grid["origin"])
        span = np.asarray(grid["span"])

        def bounds(c):
            l, ix, iy, ik = c
            nn = root * (2 ** l)
            lo = origin + np.array([ix, iy, ik]) / nn * span
            hi = origin + (np.array([ix, iy, ik]) + 1.0) / nn * span
            return lo, hi

        loA, hiA = bounds(ca)
        loB, hiB = bounds(cb)
        tol = 1e-9
        top = origin[2] + span[2]
        for d in range(3):
            touch = (abs(hiA[d] - loB[d]) < tol
                     or abs(hiB[d] - loA[d]) < tol)
            if d == 2 and not touch:
                touch = ((abs(hiA[2] - top) < tol
                          and abs(loB[2] - origin[2]) < tol)
                         or (abs(hiB[2] - top) < tol
                             and abs(loA[2] - origin[2]) < tol))
            if not touch:
                continue
            others_overlap = all(
                min(hiA[o], hiB[o]) - max(loA[o], loB[o]) > tol
                for o in range(3) if o != d)
            if others_overlap:
                return True
        return False

    def _validate_portals(self, v):
        for pid, p in self.portals.items():
            wps = p.get("waypoints") or []
            if len(wps) < 2:
                continue      # length violation reported elsewhere
            for fld, wp in (("chart_a", wps[0]), ("chart_b", wps[-1])):
                ch = self.charts.get(p[fld])
                if ch is not None and \
                        self._region_contains(ch, wp) is None:
                    v.append(f"portal {pid}: endpoint {wp} is not "
                             f"inside {fld} {p[fld]}'s certified region")
            cert = p.get("certificate") or {}
            if p.get("kind") == "generic_certified_connector":
                r = cert.get("certified_tube_radius_m")
                if r is None or r <= 0:
                    v.append(f"portal {pid}: missing/non-positive "
                             f"certified_tube_radius_m")
            if self.checker_id and \
                    self.checker_id not in (cert.get("checker") or ""):
                v.append(f"portal {pid}: certificate checker does not "
                         f"match atlas checker_id")

    def _validate_attachments(self, v):
        for aid, a in self.attachments.items():
            wps = a.get("waypoints") or []
            if len(wps) < 2:
                continue
            ch = self.charts.get(a["chart_id"])
            if ch is not None and \
                    self._region_contains(ch, wps[0]) is None:
                v.append(f"attachment {aid}: chart-side endpoint "
                         f"{wps[0]} is not inside chart "
                         f"{a['chart_id']}'s certified region")
            comp = self.components.get(a["component_id"])
            if comp is not None:
                anchor = comp.get("sampled_best_pose") or []
                if len(anchor) == 3 and max(
                        abs(x - y) for x, y in zip(anchor, wps[-1])) \
                        > 1e-6:
                    v.append(f"attachment {aid}: component-side "
                             f"endpoint does not match component "
                             f"anchor pose")
            cert = a.get("certificate") or {}
            if self.checker_id and \
                    self.checker_id not in (cert.get("checker") or ""):
                v.append(f"attachment {aid}: certificate checker does "
                         f"not match atlas checker_id")

    def _validate_edge_chains(self, v):
        for eid, e in self.edges.items():
            br = self.branches.get(e["gate_branch"])
            if br is None or not br["component_ids"]:
                continue
            ends = {br["component_ids"][0], br["component_ids"][-1]}
            att_comps = set()
            for fld in ("source_attachment_id", "target_attachment_id"):
                a = self.attachments.get(e.get(fld) or "")
                if a is not None:
                    att_comps.add(a["component_id"])
                    if a["component_id"] not in ends:
                        v.append(f"edge {eid}: attachment {e.get(fld)} "
                                 f"anchors component "
                                 f"{a['component_id']}, not a branch "
                                 f"ENDPOINT of {e['gate_branch']}")
            w = self.transitions.get(e.get("certified_witness_id") or "")
            if w is not None and len(att_comps) == 2:
                wset = {w["source_component"], w["target_component"]}
                if wset != att_comps:
                    v.append(f"edge {eid}: witness joins {sorted(wset)} "
                             f"but attachments anchor "
                             f"{sorted(att_comps)} — chain is not "
                             f"attachment-branch-attachment")

    def _validate_identity(self, v):
        has_cert = (any(c.get("certified") for c in self.charts.values())
                    or self.portals or self.transitions
                    or self.attachments)
        if has_cert:
            for fld in ("scene_hash", "robot_hash", "checker_id"):
                if not getattr(self, fld):
                    v.append(f"atlas carries certificates but {fld} is "
                             f"empty — certificates are unbound")
            for tid, t in self.transitions.items():
                cert = t.get("certificate") or {}
                if self.checker_id and \
                        self.checker_id not in (cert.get("checker")
                                                or ""):
                    v.append(f"transition {tid}: certificate checker "
                             f"does not match atlas checker_id")

    def validate(self):
        v = []
        self._validate_regions(v)
        self._validate_portals(v)
        self._validate_attachments(v)
        self._validate_edge_chains(v)
        self._validate_identity(v)
        for cid, c in self.components.items():
            if c["contact_signature_id"] and \
                    c["contact_signature_id"] not in self.signatures:
                v.append(f"component {cid}: dangling signature "
                         f"{c['contact_signature_id']}")
            if c["status"] != "FREE" and \
                    c.get("certified_min_margin") is not None:
                v.append(f"component {cid}: status {c['status']} carries "
                         f"certified_min_margin (sampled/certified "
                         f"separation violated)")
            for fld, inv in (("predecessor_ids", "successor_ids"),
                             ("successor_ids", "predecessor_ids")):
                for other in c.get(fld, []):
                    if other not in self.components:
                        v.append(f"component {cid}: dangling {fld} {other}")
                    elif cid not in self.components[other].get(inv, []):
                        # round-8 §8 item 8: predecessor/successor links
                        # must be bidirectionally consistent
                        v.append(f"component {cid}: {fld} {other} does "
                                 f"not list {cid} in its {inv}")
        for chid, ch in self.charts.items():
            # round-8 §8 item 6: certified=True must be backed by a
            # serialized certified region (grid spec + member cells) —
            # a bbox with a boolean is a claim, not a region
            if ch.get("certified"):
                reg = ch.get("region") or {}
                if not reg.get("grid") or not reg.get("cells"):
                    v.append(f"chart {chid}: certified=True but region "
                             f"carries no serialized certified cells")
            for bid in ch.get("adjacent_gate_branches", []):
                # round-8 §8 item 7: adjacent_gate_branches must mirror
                # the actual edge set
                if bid not in self.branches:
                    v.append(f"chart {chid}: adjacent_gate_branches "
                             f"dangling {bid}")
                elif not any(e["gate_branch"] == bid and chid in
                             (e["source_chart"], e["target_chart"])
                             for e in self.edges.values()):
                    v.append(f"chart {chid}: adjacent_gate_branches "
                             f"lists {bid} but no edge joins them")
        for eid, e in self.edges.items():
            gb = e["gate_branch"]
            for chid in (e["source_chart"], e["target_chart"]):
                ch = self.charts.get(chid)
                if ch is not None and gb in self.branches and \
                        gb not in ch.get("adjacent_gate_branches", []):
                    v.append(f"edge {eid}: chart {chid} does not list "
                             f"gate_branch {gb} in adjacent_gate_branches")
        for sid, s in self.signatures.items():
            op = s.get("opposing_pair") or []
            nc = len(s.get("clusters") or [])
            if len(op) != 2 or len(set(op)) != 2 \
                    or any(not (0 <= i < nc) for i in op):
                v.append(f"signature {sid}: opposing_pair {op} does not "
                         f"index two distinct clusters (have {nc})")
        for bid, b in self.branches.items():
            for cid in b["component_ids"]:
                if cid not in self.components:
                    v.append(f"branch {bid}: dangling component {cid}")
            # round-8 §8 item 3: consecutive branch components must be
            # joined by a CertifiedTransition (either direction) — a
            # branch is a certified chain, not an ordered name list
            ids = b["component_ids"]
            for c1, c2 in zip(ids[:-1], ids[1:]):
                if not any({t["source_component"], t["target_component"]}
                           == {c1, c2} for t in self.transitions.values()):
                    v.append(f"branch {bid}: components {c1} -> {c2} "
                             f"not joined by any certified transition")
        for tid, t in self.transitions.items():
            for fld in ("source_component", "target_component"):
                if t[fld] not in self.components:
                    v.append(f"transition {tid}: dangling {fld} {t[fld]}")
            if t["source_component"] == t["target_component"]:
                v.append(f"transition {tid}: source == target "
                         f"({t['source_component']}) — self-transition")
            cert = t.get("certificate") or {}
            if not cert.get("checks") or cert.get("min_margin_m") is None \
                    or cert.get("min_margin_m") <= 0:
                v.append(f"transition {tid}: missing/non-positive "
                         f"certificate")
            # round-8 §8 item 9: the connector polyline must exist and
            # its endpoints must be the anchor poses of the components
            # it claims to join
            wps = t.get("connector_wps") or []
            if len(wps) < 2:
                v.append(f"transition {tid}: connector_wps missing/too "
                         f"short")
            else:
                for fld, wp in (("source_component", wps[0]),
                                ("target_component", wps[-1])):
                    comp = self.components.get(t[fld])
                    if comp is None:
                        continue
                    anchor = comp.get("sampled_best_pose") or []
                    if len(anchor) == 3 and max(
                            abs(a - b) for a, b in zip(anchor, wp)) > 1e-6:
                        v.append(f"transition {tid}: endpoint does not "
                                 f"match {fld} anchor pose")
        for pid, p in self.portals.items():
            if p["chart_a"] == p["chart_b"]:
                v.append(f"portal {pid}: same-chart portal "
                         f"({p['chart_a']})")
            for fld in ("chart_a", "chart_b"):
                ch = self.charts.get(p[fld])
                if ch is None:
                    v.append(f"portal {pid}: dangling {fld} {p[fld]}")
                elif not ch.get("certified"):
                    v.append(f"portal {pid}: {fld} {p[fld]} not certified")
            cert = p.get("certificate") or {}
            if not cert.get("checks") or cert.get("min_margin_m") is None \
                    or cert.get("min_margin_m") <= 0:
                v.append(f"portal {pid}: missing/non-positive certificate")
            if len(p.get("waypoints") or []) < 2:
                v.append(f"portal {pid}: waypoints missing/too short")
        for aid, a in self.attachments.items():
            ch = self.charts.get(a["chart_id"])
            if ch is None:
                v.append(f"attachment {aid}: dangling chart "
                         f"{a['chart_id']}")
            elif not ch.get("certified"):
                v.append(f"attachment {aid}: chart {a['chart_id']} "
                         f"not certified")
            if a["component_id"] not in self.components:
                v.append(f"attachment {aid}: dangling component "
                         f"{a['component_id']}")
            cert = a.get("certificate") or {}
            if not cert.get("checks") or cert.get("min_margin_m") is None \
                    or cert.get("min_margin_m") <= 0:
                v.append(f"attachment {aid}: missing/non-positive "
                         f"certificate")
            if len(a.get("waypoints") or []) < 2:
                v.append(f"attachment {aid}: waypoints missing/too short")
        for eid, e in self.edges.items():
            if e["source_chart"] == e["target_chart"]:
                v.append(f"edge {eid}: same-chart pseudo-gate "
                         f"({e['source_chart']})")
            for fld in ("source_chart", "target_chart"):
                ch = self.charts.get(e[fld])
                if ch is None:
                    v.append(f"edge {eid}: dangling {fld} {e[fld]}")
                elif not ch.get("certified"):
                    v.append(f"edge {eid}: {fld} {e[fld]} not certified")
            if e["gate_branch"] not in self.branches:
                v.append(f"edge {eid}: dangling gate_branch "
                         f"{e['gate_branch']}")
            w = e.get("certified_witness_id")
            if not w:
                v.append(f"edge {eid}: no certified witness")
            elif w not in self.transitions:
                v.append(f"edge {eid}: dangling witness {w}")
            else:
                # round-8 §8 items 1-2: the witness transition must
                # actually run along THIS edge's branch — both endpoint
                # components belong to the branch it names
                br = self.branches.get(e["gate_branch"])
                if br is not None:
                    ids = set(br["component_ids"])
                    t = self.transitions[w]
                    for fld in ("source_component", "target_component"):
                        if t[fld] not in ids:
                            v.append(f"edge {eid}: witness {w} endpoint "
                                     f"{t[fld]} is not a component of "
                                     f"branch {e['gate_branch']}")
            # round-8 §8 items 4-5: an edge is only a verified chain if
            # BOTH chart-to-branch attachments exist and are certified
            for fld, chart_fld in (("source_attachment_id",
                                    "source_chart"),
                                   ("target_attachment_id",
                                    "target_chart")):
                aid = e.get(fld)
                if not aid:
                    v.append(f"edge {eid}: missing {fld} (chart-to-"
                             f"branch attachment certificate)")
                    continue
                a = self.attachments.get(aid)
                if a is None:
                    v.append(f"edge {eid}: dangling {fld} {aid}")
                    continue
                if a["chart_id"] != e[chart_fld]:
                    v.append(f"edge {eid}: attachment {aid} anchors "
                             f"chart {a['chart_id']}, not {e[chart_fld]}")
                br = self.branches.get(e["gate_branch"])
                if br is not None and \
                        a["component_id"] not in br["component_ids"]:
                    v.append(f"edge {eid}: attachment {aid} anchors "
                             f"component {a['component_id']} outside "
                             f"branch {e['gate_branch']}")
        def walk_keys(obj):
            if isinstance(obj, dict):
                for k, val in obj.items():
                    yield str(k)
                    yield from walk_keys(val)
            elif isinstance(obj, list):
                for x in obj:
                    yield from walk_keys(x)
        for k in walk_keys(self.compile_provenance):
            lk = k.lower()
            if "goal" in lk or lk in ("start", "start_pose"):
                v.append(f"compile_provenance key '{k}' — "
                         f"goal-conditioned compile suspected")
        text = json.dumps(self.compile_provenance).lower()
        for benign in ("goal-independent", "goal-independence",
                       "goal-free", "no goal", "without goal"):
            text = text.replace(benign, "")
        if "goal" in text:
            v.append("compile_provenance mentions 'goal' — "
                     "goal-conditioned compile suspected")
        return v

    def serialize(self):
        """Canonical JSON: sorted keys, fixed float repr."""
        return json.dumps(
            {"charts": self.charts, "signatures": self.signatures,
             "components": self.components, "branches": self.branches,
             "transitions": self.transitions, "portals": self.portals,
             "attachments": self.attachments, "edges": self.edges,
             "compile_provenance": self.compile_provenance,
             "compile_query_count": self.compile_query_count,
             "scene_hash": self.scene_hash,
             "robot_hash": self.robot_hash,
             "domain_hash": self.domain_hash,
             "checker_id": self.checker_id},
            sort_keys=True, separators=(",", ":"))

    def atlas_hash(self):
        return hashlib.sha256(self.serialize().encode()).hexdigest()[:16]
