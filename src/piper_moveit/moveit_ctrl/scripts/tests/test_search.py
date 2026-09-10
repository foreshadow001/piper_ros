"""统一搜索循环与候选生成的单元测试 (无硬件依赖, stub 后端)。

圆盘搜索域: √(Δα²+Δβ²) ≤ search_radius, 网格 search_step, 全局理想锚直接偏移。
"""
import sys
import threading
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import pytest

from piper_arm_controller import PiperArmController


def make_ctrl(radius=30.0, step=10.0):
    """绕过 __init__ (需要 rospy/moveit)，只挂搜索所需属性。"""
    c = object.__new__(PiperArmController)
    c.search_radius = radius
    c.search_step = step
    c.eye_position = (1.1, 0.0, -0.05)
    c._offset_records = []          # 无偏移数据 → 纯理想锚圆盘
    c._lock = threading.RLock()     # search_orientation 的 @_synchronized 需要
    return c


def test_candidate_count_default():
    # R=30, S=10: 层 {0,±10,±20,±30} → 7 + 5×2 + 5×2 + 1×2 = 29
    assert len(PiperArmController.alpha_beta_candidates(make_ctrl(), 0.4, 0.0, 0.3)) == 29


def test_candidate_count_dense_grid():
    # R=20, S=5: 9 + 7×2 + 7×2 + 5×2 + 1×2 = 49
    assert len(PiperArmController.alpha_beta_candidates(make_ctrl(20, 5), 0.4, 0.0, 0.3)) == 49


def test_first_candidate_is_ideal():
    cands = PiperArmController.alpha_beta_candidates(make_ctrl(), 0.4, 0.0, 0.3)
    a, b, da, db = cands[0]
    assert da == 0.0 and db == 0.0
    assert 0.0 <= a <= 180.0 and 0.0 <= b <= 180.0


def test_all_candidates_within_disk():
    """每个候选的 (Δα, Δβ) 都满足 √(Δα²+Δβ²) ≤ radius + 数值容差。"""
    c = make_ctrl(radius=30.0, step=10.0)
    cands = PiperArmController.alpha_beta_candidates(c, 0.4, 0.0, 0.3)
    for _, _, da, db in cands:
        # 容差: 裁剪 [0,180] 只会缩小偏移; 1e-6 为浮点余量
        assert math_hypot(da, db) <= c.search_radius + 1e-6


def math_hypot(a, b):
    import math
    return math.hypot(a, b)


def test_ring_ordering_total_deviation_ascending():
    """环序: 总偏离升序; 同环 |Δα| 小者优先 (先调 β 后动 α)。"""
    import math
    cands = PiperArmController.alpha_beta_candidates(make_ctrl(), 0.4, 0.0, 0.3)
    keys = [(math.hypot(da, db), abs(da)) for _, _, da, db in cands]
    # 分环比较: 环内 |Δα| 升序; 环间总偏离升序
    rings = []
    for dev, absda in keys:
        if not rings or abs(rings[-1][0][0] - dev) > 1e-6:
            rings.append([(dev, absda)])
        else:
            rings[-1].append((dev, absda))
    devs = [r[0][0] for r in rings]
    assert devs == sorted(devs)                     # 环间: 总偏离升序
    for ring in rings:
        absdas = [x[1] for x in ring]
        assert absdas == sorted(absdas)             # 环内: |Δα| 升序


def test_search_returns_first_success_with_offsets():
    calls = []

    def try_pose(alpha, beta):
        calls.append((alpha, beta))
        return len(calls) == 3          # 第 3 个候选成功

    res = PiperArmController.search_orientation(make_ctrl(), 0.4, 0.0, 0.3, try_pose)
    assert res is not None
    assert len(res) == 5
    assert (res[0], res[1]) == calls[-1]     # 返回的正是成功的那个候选
    assert len(calls) == 3                   # 成功即停，不多试


def test_search_all_fail_returns_none():
    res = PiperArmController.search_orientation(make_ctrl(), 0.4, 0.0, 0.3,
                                                lambda a, b: False)
    assert res is None


def test_search_abort_check_stops_loop():
    n = {"i": 0}

    def try_pose(alpha, beta):
        n["i"] += 1
        return False

    res = PiperArmController.search_orientation(
        make_ctrl(), 0.4, 0.0, 0.3, try_pose,
        abort_check=lambda: n["i"] >= 2)
    assert res is None
    assert n["i"] == 2                      # 第 3 次候选前被 abort_check 拦截


def test_search_exception_propagates():
    """后端异常必须穿透循环 (ServiceCallTimeout 中止语义依赖此行为)。"""
    def try_pose(alpha, beta):
        raise RuntimeError("backend died")

    try:
        PiperArmController.search_orientation(make_ctrl(), 0.4, 0.0, 0.3, try_pose)
        assert False, "should have raised"
    except RuntimeError:
        pass
