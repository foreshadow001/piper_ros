"""统一搜索循环与候选生成的单元测试 (无硬件依赖, stub 后端)。"""
import sys
import threading
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import pytest

from piper_arm_controller import PiperArmController


def make_ctrl():
    """绕过 __init__ (需要 rospy/moveit)，只挂搜索所需属性。"""
    c = object.__new__(PiperArmController)
    c.yaw_step, c.yaw_levels = 10.0, 3
    c.pitch_step, c.pitch_range = 5.0, 20.0
    c.eye_position = (1.1, 0.0, -0.05)
    c._offset_records = []          # Task 4 前为空
    c._lock = threading.RLock()     # search_orientation 的 @_synchronized 需要
    return c


def test_candidate_count_default():
    # 理想层 9 + 非理想 4 层 × 5 = 29
    assert len(PiperArmController.alpha_beta_candidates(make_ctrl(), 0.4, 0.0, 0.3)) == 29


def test_first_candidate_near_ideal():
    cands = PiperArmController.alpha_beta_candidates(make_ctrl(), 0.4, 0.0, 0.3)
    a, b, da, db = cands[0]
    assert da == 0.0 and db == 0.0
    assert 0.0 <= a <= 180.0 and 0.0 <= b <= 180.0


def test_candidates_ordered_best_first():
    """分层螺旋: α 层按 |Δα| 升序; 层内围绕该层的局部理想 β 交替扩展。

    层内模式为 [local, local+s, local-s, local+2s, local-2s, ...] —
    理想层 local == ideal_beta (严格 best-first); 非理想层 local 偏离
    ideal, 对 |β-ideal_beta| 不单调, 原实现 (重构前) 即如此。
    """
    cands = PiperArmController.alpha_beta_candidates(make_ctrl(), 0.4, 0.0, 0.3)

    layers = []                                   # [(alpha, [betas])]
    for a, b, _, _ in cands:
        if not layers or layers[-1][0] != a:
            layers.append((a, [b]))
        else:
            layers[-1][1].append(b)

    layer_offsets = [abs(a - layers[0][0]) for a, _ in layers]
    assert layer_offsets == [0, 10, 10, 20, 20]    # 层序: 理想, ±10, ±20

    for _, betas in layers:
        local = betas[0]
        expect = [local]
        for i in range(1, len(betas) // 2 + 1):
            expect.extend([local + i * 5.0, local - i * 5.0])
        assert betas == pytest.approx(expect)      # 层内交替 ±5° 扩展


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
