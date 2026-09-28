"""G2 batch machinery: seeded pair sampler, array slicing, per-pair checkpoint, compile-once task."""
import json

import numpy as np
import pytest

from gmc.aerial3d.api import CompileConfig, compile_complex, save_compiled
import aerial3dg_batch as ab
from aerial3dg_fixtures import CYLINDER, SWEEPER, table_corridor

BOX = (0., 0., 10., 4.)


def _free(name, uv):
    """Synthetic clearance: a wall strip u in [4, 5] blocks both robots; v > 3 only the sweeper fits."""
    if 4. <= uv[0] <= 5.:
        return False, None
    if name == "cylinder" and uv[1] > 3.:
        return False, None
    return True, 0.1


def test_sampler_is_seeded_and_every_pair_meets_the_rules():
    a, ca = ab.sample_pairs(200, seed=7, box_uv=BOX, is_free=_free, robots=("sweeper", "cylinder"))
    b, cb = ab.sample_pairs(200, seed=7, box_uv=BOX, is_free=_free, robots=("sweeper", "cylinder"))
    assert a == b and ca == cb
    c, _ = ab.sample_pairs(200, seed=8, box_uv=BOX, is_free=_free, robots=("sweeper", "cylinder"))
    assert a != c
    assert [p["index"] for p in a] == list(range(200))
    for p in a:
        s, g = np.array(p["start_uv"]), np.array(p["goal_uv"])
        assert np.linalg.norm(g - s) >= 3.0 and p["dist_m"] == pytest.approx(np.linalg.norm(g - s))
        for uv in (s, g):
            assert BOX[0] <= uv[0] <= BOX[2] and BOX[1] <= uv[1] <= BOX[3]
            assert _free("sweeper", uv)[0] and _free("cylinder", uv)[0]
        assert set(p["clearance_m"]) == {"sweeper", "cylinder"}


def test_sampler_counts_every_rejection():
    pairs, c = ab.sample_pairs(100, seed=1, box_uv=BOX, is_free=_free, robots=("sweeper", "cylinder"))
    assert c["accepted"] == len(pairs) == 100
    rejected = c["dist_below_min"] + sum(v for k, v in c.items() if k.startswith("endpoint_not_free_"))
    assert c["pair_draws"] == c["accepted"] + rejected
    assert c["endpoint_not_free_cylinder"] > 0          # the v > 3 strip is sweeper-only


def test_sampler_prefix_is_the_smaller_run():
    """Pairs are i.i.d. draws in order: the first k of a 200-run equal a k-run (any prefix is unbiased)."""
    big, _ = ab.sample_pairs(200, seed=3, box_uv=BOX, is_free=_free, robots=("sweeper",))
    small, _ = ab.sample_pairs(50, seed=3, box_uv=BOX, is_free=_free, robots=("sweeper",))
    assert big[:50] == small


def test_task_slices_partition_the_list():
    seen = []
    for t in range(7):
        seen += list(ab.task_slice(5000, 7, t))
    assert seen == list(range(5000))
    assert len(ab.task_slice(5000, 10, 3)) == 500


def test_checkpoint_drops_a_torn_tail(tmp_path):
    f = tmp_path / "t.jsonl"
    f.write_text(json.dumps({"index": 0, "status": "REACHABLE"}) + "\n"
                 + json.dumps({"index": 1, "status": "UNKNOWN"}) + "\n" + '{"index": 2, "sta')
    done = ab.read_checkpoint(f)
    assert sorted(done) == [0, 1]
    assert f.read_text().endswith("\n") and len(f.read_text().splitlines()) == 2   # file repaired


@pytest.fixture(scope="module")
def persisted(tmp_path_factory):
    scene, _ = table_corridor()
    d = tmp_path_factory.mktemp("a3c")
    compiled = compile_complex(scene, SWEEPER, config=CompileConfig(margin_m=.001))
    save_compiled(compiled, d / "sweeper.a3c")
    return d / "sweeper.a3c", scene


def _corridor_pairs(scene, n=3):
    """The fixture's 'under' query (identity plan frame): (-1.8, .5) -> (1.8, .5)."""
    return [{"index": k, "pair_id": f"T{k}", "start_uv": [-1.8, .5], "goal_uv": [1.8, .5], "dist_m": 3.6}
            for k in range(n)]


def test_run_task_answers_from_one_load_and_resumes(tmp_path, persisted):
    path, scene = persisted
    pairs = _corridor_pairs(scene)
    out = tmp_path / "task.jsonl"
    s1 = ab.run_task(path, pairs[:2], out, timeout_s=60)
    assert s1["answered_this_run"] == 2 and s1["resumed_from_checkpoint"] == 0
    s2 = ab.run_task(path, pairs, out, timeout_s=60)
    assert s2["answered_this_run"] == 1 and s2["resumed_from_checkpoint"] == 2
    rows = [json.loads(x) for x in out.read_text().splitlines()]
    assert [r["index"] for r in rows] == [0, 1, 2]
    assert len({r["compile_id"] for r in rows}) == 1
    assert all(r["compile_stages_in_call"] == [] for r in rows)
    assert len({(r["status"], r["polyline_sha256"]) for r in rows}) == 1   # same pair -> same answer
    proof = s2["compile_once_proof"]
    assert proof["compiles_in_this_process"] == 0 and proof["loads_in_this_process"] == 1
    assert proof["compile_records_changed_by_queries"] is False
