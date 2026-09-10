"""fast_solve 模块单元测试 (无 ROS 依赖)。"""
import math
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import fast_solve
import pytest


def test_save_load_roundtrip(tmp_path):
    recs = [(0.1, -0.2, 0.3, 10.0, 5.0), (0.4, 0.0, 0.2, -5.0, 0.0)]
    p = tmp_path / "points_test.offsets.txt"
    fast_solve.save_offsets(p, recs)
    assert fast_solve.load_offsets(p) == recs


def test_load_ignores_comments_and_blank(tmp_path):
    p = tmp_path / "x.offsets.txt"
    p.write_text("# header comment\n\n0.1 0.2 0.3 1.0 2.0\n")
    assert fast_solve.load_offsets(p) == [(0.1, 0.2, 0.3, 1.0, 2.0)]


def test_weighted_offset_inverse_distance_weighting():
    # 距离 0.02 与 0.04 的两点, 权重比 2:1 → 偏移 = (2*10 + 1*(-5))/3
    recs = [(0.02, 0.0, 0.0, 10.0, 0.0),
            (0.04, 0.0, 0.0, -5.0, 0.0)]
    da, db = fast_solve.weighted_offset(recs, 0.0, 0.0, 0.0)
    assert da == pytest.approx((2 * 10.0 + 1 * -5.0) / 3)
    assert db == pytest.approx(0.0)


def test_weighted_offset_radius_filter():
    # 唯一记录在门限外 → None (无合格邻居, 跳过 fast_solve)
    far = fast_solve.NEIGHBOR_RADIUS + 0.01
    recs = [(far, 0.0, 0.0, 10.0, 10.0)]
    assert fast_solve.weighted_offset(recs, 0.0, 0.0, 0.0) is None


def test_weighted_offset_filters_far_keeps_near():
    # k=2 内一个超门限一个在门限内 → 只用门限内的
    far = fast_solve.NEIGHBOR_RADIUS + 0.01
    recs = [(far, 0.0, 0.0, 30.0, 30.0),
            (0.02, 0.0, 0.0, 10.0, -4.0)]
    da, db = fast_solve.weighted_offset(recs, 0.0, 0.0, 0.0)
    assert da == pytest.approx(10.0)
    assert db == pytest.approx(-4.0)


def test_weighted_offset_empty_records():
    assert fast_solve.weighted_offset([], 0.1, 0.1, 0.1) is None


# ------------------------------------------------------------------
# _priority_candidates 重锚 (controller 集成)
# ------------------------------------------------------------------

def _make_ctrl(records):
    from piper_arm_controller import PiperArmController
    c = object.__new__(PiperArmController)
    c.search_radius, c.search_step = 30.0, 10.0
    c.eye_position = (1.1, 0.0, -0.05)
    c._offset_records = records
    return c


def test_priority_candidates_reanchored_first():
    from piper_arm_controller import PiperArmController

    # 目标点恰在记录点上 → 加权偏移 = 记录偏移 (dist=0 直返)
    c = _make_ctrl([(0.40, 0.0, 0.30, 15.0, -10.0)])
    cands = PiperArmController._priority_candidates(c, 0.40, 0.0, 0.30)

    spiral = PiperArmController.alpha_beta_candidates(c, 0.40, 0.0, 0.30)
    ideal_a = spiral[0][0] - spiral[0][2]
    ideal_b = spiral[0][1] - spiral[0][3]

    # 重锚圆盘以 ideal+(15,-10) 为中心, 绕中心的环序 → 首候选即中心本身
    # (速度优先: 直接从经验最优姿态开始试)
    first = cands[0]
    assert first[4] == 'fast'                       # 首候选来自重锚圆盘
    assert first[0] == pytest.approx(ideal_a + 15.0)  # 锚点即首候选
    assert first[1] == pytest.approx(ideal_b - 10.0)
    assert first[2] == pytest.approx(15.0)          # d 值 = 相对几何理想
    # 后续 'fast' 候选按绕中心的偏离环序 (并列环内次序由决胜键定, 不逐一断言)
    fast_devs = [math.hypot(c_[0] - (ideal_a + 15.0), c_[1] - (ideal_b - 10.0))
                 for c_ in cands if c_[4] == 'fast']
    assert fast_devs[0] == pytest.approx(0.0)
    for prev, cur in zip(fast_devs, fast_devs[1:]):
        assert cur >= prev - 1e-6
    assert all(c_[4] is None or c_[4] == 'fast' for c_ in cands)


def test_priority_candidates_dedup_zero_offset():
    """偏移 (0,0) 时修正锚圆盘与理想锚圆盘重合 → 去重后仍 29 个。"""
    from piper_arm_controller import PiperArmController

    c = _make_ctrl([(0.40, 0.0, 0.30, 0.0, 0.0)])
    cands = PiperArmController._priority_candidates(c, 0.40, 0.0, 0.30)
    assert len(cands) == 29
    keys = {(round(c_[0], 3), round(c_[1], 3)) for c_ in cands}
    assert len(keys) == 29                          # 无重复


def test_priority_candidates_far_neighbor_ignored():
    """唯一邻居在门限外 → 无重锚，纯理想螺旋 (回退)。"""
    from piper_arm_controller import PiperArmController
    import fast_solve

    far = fast_solve.NEIGHBOR_RADIUS + 0.01
    c = _make_ctrl([(0.40 + far, 0.0, 0.30, 15.0, -10.0)])
    cands = PiperArmController._priority_candidates(c, 0.40, 0.0, 0.30)
    assert len(cands) == 29
    assert all(c_[4] is None for c_ in cands)


def test_priority_candidates_empty_records_pure_spiral():
    from piper_arm_controller import PiperArmController

    c = _make_ctrl([])
    cands = PiperArmController._priority_candidates(c, 0.40, 0.0, 0.30)
    assert len(cands) == 29
    assert all(c_[4] is None for c_ in cands)  # 纯螺旋，行为精确退化为现状 (直连/分析模式路径)
