# fast_solve 实施计划（含方案评估）

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** controller 增加 `fast_solve` 开关：离线记录"可达点求解成功时的 (Δα, Δβ) 偏移"，运行时对目标点做最近邻查询，把偏移修正后的姿态作为优先候选，压缩典型求解延迟。

**Architecture:** 四层。① 把 `move_to` 的搜索循环抽成 controller 上的**唯一**方法 `search_orientation(x, y, z, try_pose, abort_check)`，三种消费者（服务模式 / 直连执行 / analyzer 仅规划）各自只提供 `try_pose` 后端——**搜索顺序、裁剪、中止语义在仿真与实测间逐位一致**（用户 2026-08-23 指示：analyzer 必须走与实测完全相同的搜索）；② 新增纯 Python 模块 `fast_solve.py`（偏移文件 save/load/kNN，无 ROS 依赖，可单测）；③ analyzer 换用共享循环并记录成功偏移；④ 运行时在螺旋候选前插入 k 个偏移修正候选（插入而非替换，保证回退）。

**Tech Stack:** ROS Noetic / rospy / moveit_commander（均现有）；测试用 pytest（仓库当前无测试基建，本计划为搜索循环与 `fast_solve.py` 建立最小 `scripts/tests/`）。

**Spec:** 用户口头需求（2026-08-22 会话）：
1. controller 加 `fast_solve` 开关；
2. 关闭 fast_solve 跑 workspace_analyzer，保存可达点 + 成功时的 pitch/yaw 偏移；
3. 开启后，执行指令时查最近可达点（或多个）取偏移，作为理想目标，后续流程不变。

追加约束（2026-08-23）：**analyzer 必须调用与实测 `move_to` 完全相同的搜索函数**，保证仿真与实测一致。同日决定：**fast_solve 不设配置开关，由模式派生**——服务模式自动开启（文件缺失则回退），直连/分析模式恒关。

再次修订（2026-08-23，偏移使用方式）：k 近邻偏移做**反距离加权平均**（越近权重越高；邻居距离门限 `resolution·√3`，过滤后至少 1 个，全超门限则跳过 fast_solve），平均偏移作为**螺旋搜索的新锚点**（基础螺旋整体平移），修正锚螺旋失败再回退理想锚螺旋。替代早先"k 个离散偏移候选插入队首"的设计。

---

## Part A: 方案评估

### 前提问题：两处搜索目前不一致（必须先统一）

`move_to` 的搜索：几何理想 (α₀, β₀) → 螺旋扩展，理想层 β 范围 ±`pitch_range`(20°)、步长 5°，非理想 α 层范围减半，共 29 个候选（默认参数）。且服务模式与直连模式是**两份平行实现**（`_service_move_to` / `_inprocess_move_to`）。

`workspace_analyzer.check_reachability` 的搜索：α = atan2(x, −y)（与理想 α 同式），β 只有固定三个候选 **[90°, 45°, 0°]**（[workspace_analyzer.py:102](../workspace_analyzer.py#L102)），无偏移记录。

**统一到什么程度：** 仅共享"候选生成"不够——循环体（遍历顺序、越界裁剪、失败清理、中止检查、日志）仍是 analyzer 自己写的一份，`move_to` 一改 analyzer 就漂移，记录的偏移随之失真。因此统一单位是**整个搜索循环**：`search_orientation` 只负责"按序取候选 → 调 `try_pose` → 首个成功即返回（含偏移）"，姿态如何被执行/验证全部下沉到 `try_pose` 后端。三种后端：

| 消费者 | try_pose 后端 | 行为 |
|--------|--------------|------|
| 服务模式（实测遥操作） | 调 `joint_moveit_ctrl_endpose` + `/end_pose` 距离验证 | 规划+执行+验证 |
| 直连模式（实测批处理） | `_plan_with_timeout` + `_execute` | 规划+执行 |
| analyzer（离线仿真） | `move_group.plan()` 仅规划 | 规划即可达 |

Ctrl+C 语义差异通过 `abort_check` 参数桥接（analyzer 的 `keep_running` 标志会拦截 SIGINT 使 `rospy.is_shutdown()` 保持 False，故循环必须支持外部中止回调，不能只查 rospy）。

### 架构约束：统一循环 ≠ 合并双模式

两种模式面向两套运行环境，合并任一方向都会破坏对方，`try_pose` 后端不是 DRY 糖，而是同时满足两个约束的接缝：

- **实机执行必须走服务（单编排者约束）**。MoveIt 用 fake controller 执行（轨迹 → `/joint_states` 话题 → CAN 驱动），`joint_moveit_ctrl_server` 是这条链路的唯一编排者，每次运动执行 block/unblock 仪式（[joint_moveit_ctrl_server.py:78-95](../joint_moveit_ctrl_server.py#L78-L95)），防止 fake controller 残留目标位姿自行驱动真臂（commit `29ae46c` 修复的 "No Move" 错误）。若 controller 直连 `go()`，两个 MoveGroupCommander 争同一个 fake controller——block/unblock 时序互踩，残留位姿直达真臂。
- **离线分析必须直连（零硬件约束）**。~1000 点 × ≤29 候选 ≈ 3 万次规划的批量负载：进程内 `plan()` 是函数调用；走服务则每次多付"服务往返 + block/unblock 仪式 + `/end_pose` 等待"，量级差几十倍，且强依赖全套真机栈（分析变成需要硬件）。

因此统一的粒度是：**策略（候选序、裁剪、中止语义）只有一份，机制（执行环境）可插拔**。fast_solve 不触碰这条边界——它只改候选排序（`_priority_candidates`），与执行环境正交，两种模式自动同时获益。

### 偏移使用方式：加权平均重锚（而非离散候选插入）

早先设计"k 个邻居偏移各自作为离散候选垫在队首"。两方案在平滑区几乎等价（平均 ≈ 最近邻，首候选都命中），差异集中在三点，重锚均占优：

- **距离门限防外推**。邻居距离必须 ≤ `resolution·√3`（0.05m 采样网格的空间对角线 ≈ 0.087m），过滤后至少 1 个，全超门限则整体跳过 fast_solve。离散插入没有门限——目标离采样区 0.5m 也会把最近的偏移硬套上去，属于无依据外推。
- **kNN 回归语义**。反距离加权平均是局部偏移场的标准估计：d_alpha 量化在 {0,±10,±20}，单邻居可能带数度噪声，平均将其抑制；且修正的是**锚点**——整个 ±20° 螺旋围绕更可能的可行中心展开，比 3 个离散姿态垫队首利用信息更充分。
- **搜索自相似**。队列永远是"一次螺旋搜索"，只是锚点被修正。日志、调试、语义只有一种形状。

代价（诚实记录）：最坏情况从 29+k=32 变为 29+29=58（修正锚 + 理想锚双螺旋，**去重**后通常远小于此；偏移数据为 (0,0) 时两螺旋重合，去重后仍 29）。偏移场不连续处（障碍边界两侧邻居分属不同姿态区），平均锚点可能落在"无人区"——但修正螺旋 ±20° 覆盖会很快扫到正确一侧，兜底还有理想锚螺旋，失败模式是**多花时间**而非失败。

### 收益

1. **典型延迟下降，机制正确**。现状：几何理想姿态常被 MoveIt 拒（碰撞/IK），平均要试几个候选才命中。fast_solve 用近邻偏移的加权平均把螺旋锚点修正到该区域已知可行的姿态附近，首个候选命中率提高。成功路径 = 1 次服务往返 + `_begin_move` 同步 + 0.05s 规划，比平均 3–12s 的失败路径和"多个候选才命中"路径都短。**偏移表示是充分的**："指向眼位"只约束法兰 Z 轴方向（2 个自由度），姿态参数化本来就是 (α, β) 二维（γ≡0 恒定，绕 Z 自转无意义），因此 (Δα, Δβ) 完整刻画"该点需要的姿态修正"，不存在被丢弃的自由度。
2. **回退保证完备**。修正锚螺旋失败时回退理想锚螺旋；无偏移数据/无合格邻居时直接走理想锚——数据缺失/过期/误导时行为精确退化为现状，无新故障模式。文件名内嵌 `get_config_label()`（含障碍物名与 bbox），障碍物变更 → 文件名变 → 加载落空 → 自动回退，无需额外交验逻辑。
3. **仿真/实测一致性升维**。统一搜索循环后，analyzer 的"可达"定义与实测 `move_to` 的"能过去"定义在代码层面是同一份——偏移数据的语义、以及工作空间分析结果本身，都与实测可比。这独立于 fast_solve 也有价值（当前 analyzer 的可达点集偏保守，β 只有 3 个离散候选）。

   **一致性的准确边界**（诚实陈述，不过度承诺）：analyzer 后端是 *plan-only*（IK + 碰撞可行），实测后端是 *plan + execute + `/end_pose` 验证*。两者共享的是**候选序与判定核心**（"该姿态下 MoveIt 能否给出无碰撞轨迹"）；执行层差异（轨迹跟踪误差、fake controller 回灌精度、2cm 验证阈值）不在保证范围内。即：**分析可达 ⇒ 大概率实测可达**（统计意义），反之不承诺。fast_solve 的回退设计正是为这一残余不确定性准备的：偏移候选失败即落入螺旋，误导只损失延迟。
4. **最坏情况有界**。修正锚螺旋 + 理想锚螺旋，最坏 29 + 29 = 58（去重后通常更小；0.05s 软超时下 ~3s 封顶）。

### 风险与边界（诚实评估）

1. **局部性假设**。偏移场在 ~`resolution`(0.05m) 邻域内平滑才有效。稀疏区/边界处最近邻可能较远或失真。缓解：距离门限 `resolution·√3`（远处邻居直接不用）+ 加权平均（抑制单点噪声）+ 双螺旋回退；不连续处的误导只损失延迟不损失正确性。
2. **不修最坏情况**。目标不可达时仍烧满全部候选。此前"搜索总预算"方案与本题正交，可后做。
3. **analyzer 结果会变化**。β 候选从 {90,45,0} 变为与 `move_to` 一致的连续螺旋（最多 29 个），可达点集**扩大**。已有 `reachable_range/*.txt` 与新数据不可比，需重新生成。
4. **analyzer 耗时增加**。不可达点从 3 候选 → 试满 29 候选。当前采样约 1000 点/臂、规划超时 0.05s（analyzer 复用 controller 的 move_group，`set_planning_time` 已生效），预估从 ~10min 涨到 ~30–50min/臂。可接受；不够再后补"分析专用激进参数"。
5. **存储为纯文本**。~1000 行 × 5 列，kNN 线性扫描纯 Python ~1–2ms，无需 numpy/kd-tree（YAGNI）。
6. **搜索循环重构的回归风险**。服务/直连两个 `move_to` 的循环体被抽走重接线，包括此前加的 `ServiceCallTimeout` 中止语义——必须保持：超时异常从后端**穿透**循环直接中止整个搜索（后端不得吞掉）。Task 1 的手动回归验证覆盖此点。

### 与备选方案对比

- **运行时缓存**（上次成功偏移存参数服务器）：零离线成本，但冷启动无效、跨会话丢失、无空间泛化。
- **学习模型**（偏移场回归）：收益边际，违背 KISS/YAGNI。
- **仅共享候选生成**：不够——循环语义仍会漂移（本计划 Task 1 因此以整循环为统一单位）。
- **搜索总预算**：修最坏情况，不修典型情况。正交，不冲突。

**结论：方案成立，批准实施。** 关键设计约束：① 统一单位是完整搜索循环（`try_pose` 后端可插拔），统一粒度是"策略一份、机制可插拔"，不合并双模式；② 偏移经门限过滤 + 加权平均后**重锚**螺旋（基础螺旋整体平移），理想锚螺旋永远保留为回退（回退语义与最坏情况界）；③ (Δα, Δβ) 是姿态修正的完整表示（γ≡0），无信息丢失。

---

## Global Constraints

- Python 3 (ROS Noetic 自带 3.8)，新代码不得用 3.9+ 语法。
- `fast_solve.py` **禁止** import rospy / moveit_commander / numpy（必须可脱离 ROS 单测）。
- 搜索循环（候选顺序、裁剪、中止、清理）只允许存在一份：`PiperArmController.search_orientation`。analyzer 与两种运行模式都不得自写遍历（DRY）。
- `ServiceCallTimeout` 必须从 `try_pose` 后端穿透 `search_orientation` 到达 wrapper（不得在循环层被捕获吞掉）。
- 注释语言与现有文件一致：中文。
- 不改动 `JointMoveitCtrl.srv` 与 `joint_moveit_ctrl_server.py`（本方案纯客户端侧）。
- 偏移文件命名复用 `get_config_label()`，保证配置变更自动失效。
- fast_solve 由模式派生：服务模式开启、直连/分析模式关闭。**不设 YAML/构造开关**——分析器是数据生产方必须采样纯螺旋，独立开关会让 `arm.fast_solve: true` 泄漏进 analyzer 造成数据集自指污染（结构上禁止，而非靠使用者小心）。
- fast_solve 邻居距离门限 `resolution·√3`（采样网格空间对角线）；过滤后至少 1 个邻居，全超门限则跳过 fast_solve 走理想锚螺旋。修正锚候选与理想锚候选必须**去重**（偏移 (0,0) 时两螺旋重合）。

---

## File Structure

| 文件 | 动作 | 职责 |
|------|------|------|
| `scripts/piper_arm_controller.py` | 修改 | ① 抽取 `alpha_beta_candidates` + `search_orientation`，两种 `move_to` 改为后端 wrapper；② `from_yaml` 在服务模式下加载偏移表（模式派生，无开关）；③ `_priority_candidates` 插入偏移候选 |
| `scripts/fast_solve.py` | 新建 | 偏移文件 save/load + kNN 查询（纯 Python） |
| `scripts/workspace_analyzer.py` | 修改 | `check_reachability` 改为"几何预筛 + `search_orientation(plan-only 后端)`"，记录成功偏移，写 sidecar |
| `scripts/tests/test_search.py` | 新建 | 候选生成 + 共享循环单测（stub 后端） |
| `scripts/tests/test_fast_solve.py` | 新建 | fast_solve 模块 + `_priority_candidates` 单测 |
| `CLAUDE.md` | 修改 | 文档同步 |

不新建共享 ROS 包、不动 CMake（同目录 import，与 `workspace_analyzer.py` import `piper_arm_controller` 同机制）。

---

### Task 1: 抽取统一搜索循环（重构，行为不变）

**Files:**
- Modify: `scripts/piper_arm_controller.py`（`_inprocess_move_to` / `_service_move_to` 整体，约 [piper_arm_controller.py:630-780](../piper_arm_controller.py#L630-L780)）
- Test: `scripts/tests/test_search.py`（新建）

**Interfaces:**
- Produces:
  - `alpha_beta_candidates(self, x, y, z) -> list[tuple[float, float, float, float]]` — `[(alpha, beta, d_alpha, d_beta), ...]`，best-first 排序，α∈[0,180]、β∈[0,180]
  - `search_orientation(self, x, y, z, try_pose, abort_check=None) -> tuple | None` — 成功返回 `(alpha, beta, d_alpha, d_beta, dist_or_None)`，全部失败/中止返回 `None`。`try_pose: (alpha_deg, beta_deg) -> bool`；`abort_check: () -> bool`
  - Task 3（analyzer 后端）与 Task 4（`_priority_candidates` 扩展）消费此接口。

- [ ] **Step 1: 写失败测试**

```python
# scripts/tests/test_search.py
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from piper_arm_controller import PiperArmController


def make_ctrl():
    """绕过 __init__ (需要 rospy/moveit)，只挂搜索所需属性。"""
    c = object.__new__(PiperArmController)
    c.yaw_step, c.yaw_levels = 10.0, 3
    c.pitch_step, c.pitch_range = 5.0, 20.0
    c.eye_position = (1.1, 0.0, -0.05)
    c._offset_records = []          # Task 4 前为空
    return c


def test_candidate_count_default():
    # 理想层 9 + 非理想 4 层 × 5 = 29
    assert len(PiperArmController.alpha_beta_candidates(make_ctrl(), 0.4, 0.0, 0.3)) == 29


def test_first_candidate_near_ideal():
    cands = PiperArmController.alpha_beta_candidates(make_ctrl(), 0.4, 0.0, 0.3)
    a, b, da, db = cands[0]
    assert da == 0.0 and db == 0.0
    assert 0.0 <= a <= 180.0 and 0.0 <= b <= 180.0


def test_candidates_sorted_by_deviation():
    cands = PiperArmController.alpha_beta_candidates(make_ctrl(), 0.4, 0.0, 0.3)
    keys = [abs(da) + abs(db) for _, _, da, db in cands]
    assert keys == sorted(keys)


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
```

说明：`search_orientation` 内部使用 `rospy.is_shutdown()` 与 `rospy.loginfo`——未初始化节点时二者均安全（分别返回 False / 打印 stdout），测试在 ROS 环境外也能跑，但统一在 `source devel/setup.bash` 后执行。

- [ ] **Step 2: 跑测试确认失败**

```bash
source /opt/ros/noetic/setup.bash && source devel/setup.bash
python3 -m pytest scripts/tests/test_search.py -v
```
Expected: FAIL — `AttributeError: ... no attribute 'alpha_beta_candidates'`

- [ ] **Step 3: 实现抽取**

在 `PiperArmController` 中新增（放在 `_compute_beta_for_alpha` 之后）。Task 1 阶段 `_priority_candidates` 就是螺旋候选的直通：

```python
    def alpha_beta_candidates(self, x, y, z):
        """围绕几何理想 (α, β) 螺旋生成候选姿态 (搜索循环的唯一候选源)。

        返回 [(alpha, beta, d_alpha, d_beta), ...]:
        - best-first: 按偏离理想程度升序
        - 非理想 α 层的 β 搜索范围减半 (与原内联逻辑一致)
        - d_alpha/d_beta: 相对理想的偏移，供 fast_solve 记录/复用
        """
        ideal_alpha, ideal_beta = self._compute_ideal_alpha_beta(x, y, z)
        ideal_alpha = max(0.0, min(180.0, ideal_alpha))

        alpha_offsets = [0.0]
        for i in range(1, int(self.yaw_levels)):
            alpha_offsets.extend([i * self.yaw_step, -i * self.yaw_step])
        alpha_offsets.sort(key=abs)

        out = []
        for alpha_off in alpha_offsets:
            alpha = ideal_alpha + alpha_off
            if not (0.0 <= alpha <= 180.0):
                continue
            local_beta = self._compute_beta_for_alpha(x, y, z, alpha)
            eff_range = self.pitch_range if abs(alpha_off) < 1e-6 else self.pitch_range * 0.5
            n = int(eff_range / self.pitch_step)
            betas = [local_beta]
            for i in range(1, n + 1):
                betas.extend([local_beta + i * self.pitch_step,
                              local_beta - i * self.pitch_step])
            for beta in betas:
                if 0.0 <= beta <= 180.0:
                    out.append((alpha, beta,
                                alpha - ideal_alpha, beta - ideal_beta))
        return out

    def _priority_candidates(self, x, y, z):
        """搜索候选序列。Task 4 在此插入 fast_solve 偏移候选 (在螺旋候选之前)。"""
        return [(a, b, da, db, None)
                for a, b, da, db in self.alpha_beta_candidates(x, y, z)]

    @_synchronized
    def search_orientation(self, x, y, z, try_pose, abort_check=None):
        """统一 (α, β) 姿态搜索循环 — 服务模式/直连模式/analyzer 三方共用。

        仿真与实测一致性锚点: 候选顺序、裁剪、中止语义只存在这一份。

        :param try_pose: (alpha_deg, beta_deg) -> bool 后端:
             服务模式 = 调服务 + /end_pose 验证; 直连 = 规划+执行; analyzer = 仅规划
        :param abort_check: () -> bool，True 时中止 (analyzer 的 keep_running 标志)
        :return: (alpha, beta, d_alpha, d_beta, dist_or_None) 首个成功候选; 失败 None
        注意: try_pose 抛出的异常原样穿透 (ServiceCallTimeout 中止整个搜索依赖此契约)。
        """
        for alpha, beta, da, db, dist in self._priority_candidates(x, y, z):
            if rospy.is_shutdown() or (abort_check is not None and abort_check()):
                return None
            if try_pose(alpha, beta):
                return (alpha, beta, da, db, dist)

        rospy.logerr(f"search_orientation 失败: ({x:.3f}, {y:.3f}, {z:.3f}) 无可行候选")
        return None
```

随后两个 `move_to` 改为 wrapper（循环体逻辑整体迁入后端闭包，逐候选日志也移入后端——analyzer 后端不日志，避免千点×29候选刷屏）：

```python
    def _inprocess_move_to(self, x, y, z):
        """直连模式: 统一搜索循环 + 规划执行后端。"""
        def _try(alpha, beta):
            q = self._quaternion_from_zxz(alpha, beta, 0.0)
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = x, y, z
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = q

            self.move_group.set_pose_target(pose)
            ok, plan, elapsed = self._plan_with_timeout()
            if plan is None:
                rospy.loginfo(f"    [{elapsed * 1000:5.0f}ms] timeout")
                self.move_group.clear_pose_targets()
                return False
            if ok:
                rospy.loginfo(f"  >> SUCCESS [{elapsed * 1000:5.0f}ms] "
                              f"α={alpha:.1f}° β={beta:.1f}° γ=0.0°")
                self._execute(plan)
                return True
            rospy.loginfo(f"    [{elapsed * 1000:5.0f}ms] no solution")
            return False

        result = self.search_orientation(x, y, z, _try)
        if result is None:
            rospy.logerr(f"move_to 失败: ({x:.3f}, {y:.3f}, {z:.3f}) 在所有候选姿态下无解。")
        return result is not None
```

服务模式 wrapper 保持入口的 `set_param` + 头部日志，后端闭包内**先 `except ServiceCallTimeout: raise` 再 `except rospy.ServiceException`**（超时穿透、普通异常跳过该候选——保持既有修复语义）：

```python
    def _service_move_to(self, x, y, z):
        """服务模式: 统一搜索循环 + 服务调用后端。"""
        ns_prefix = f"{self.can_port}/" if self.can_port else ""
        rospy.set_param(f"{ns_prefix}joint_moveit_ctrl_server/planning_time",
                        self.planning_timeout)

        ideal_alpha, ideal_beta = self._compute_ideal_alpha_beta(x, y, z)
        rospy.loginfo(f"move_to({x:.3f}, {y:.3f}, {z:.3f}) [service] | "
                      f"ideal α={ideal_alpha:.1f}° β={ideal_beta:.1f}°")

        def _try(alpha, beta):
            q = self._quaternion_from_zxz(alpha, beta, 0.0)
            t0 = time.time()
            try:
                _call_joint_moveit_ctrl_endpose(x, y, z, q[0], q[1], q[2], q[3],
                                                max_velocity=0.3, max_acceleration=0.3,
                                                planning_time=self.planning_timeout,
                                                ns=self.can_port)
            except ServiceCallTimeout:
                raise                      # 系统级故障: 穿透循环，整体中止
            except rospy.ServiceException as e:
                rospy.logwarn(f"  服务调用异常: {e}")
                return False

            elapsed = time.time() - t0
            cur_pose = self._get_end_pose_from_topic()
            if cur_pose is None:
                return False

            dist = math.sqrt((cur_pose.position.x - x) ** 2 +
                             (cur_pose.position.y - y) ** 2 +
                             (cur_pose.position.z - z) ** 2)
            if dist < 0.02:
                rospy.loginfo(f"  >> SUCCESS [service] [{elapsed*1000:5.0f}ms] "
                              f"α={alpha:.1f}° β={beta:.1f}° dist={dist:.3f}m")
                return True
            rospy.loginfo(f"    [{elapsed*1000:5.0f}ms] dist={dist:.3f}m, try next")
            return False

        result = self.search_orientation(x, y, z, _try)
        if result is None:
            rospy.logerr(f"move_to 失敗 [service]: ({x:.3f}, {y:.3f}, {z:.3f}) 无解。")
        return result is not None
```

实现时以此处代码为准迁移原日志（原 α/β 分层 INFO 属于调试噪音，随循环抽取一并精简——保留成功/失败/超时三类）。

- [ ] **Step 4: 跑测试确认通过**

```bash
python3 -m pytest scripts/tests/test_search.py -v
```
Expected: 7 PASS

- [ ] **Step 5: 行为回归验证（手动，实机或 mock）**

```bash
rosrun moveit_ctrl piper_arm_controller.py upper
```
Expected: 候选遍历顺序与重构前一致（理想层 9 个、共 29 个）；服务模式下拔 CAN 模拟假死，确认一次 `ServiceCallTimeout` 即整体中止（不再逐候选重试）。

- [ ] **Step 6: Commit**

```bash
git add scripts/piper_arm_controller.py scripts/tests/test_search.py
git commit -m "refactor: unify orientation search loop shared by both move_to modes and analyzer"
```

---

### Task 2: fast_solve 模块（纯 Python，TDD）

**Files:**
- Create: `scripts/fast_solve.py`
- Test: `scripts/tests/test_fast_solve.py`

**Interfaces:**
- Produces:
  - `FAST_SOLVE_K = 3`（模块常量：k 近邻数，算法调优常数，无按臂差异，不入 YAML）
  - `NEIGHBOR_RADIUS = 0.05 * math.sqrt(3)`（模块常量：邻居距离门限 = 采样网格空间对角线；0.05 为 analyzer `resolution` 默认值，与 YAML 不联动——改 `resolution` 时需同步改此处）
  - `save_offsets(path, records) -> None` — records 为 `(x, y, z, d_alpha, d_beta)` 列表
  - `load_offsets(path) -> list[tuple[float, float, float, float, float]]`
  - `weighted_offset(records, x, y, z, k=FAST_SOLVE_K, radius=NEIGHBOR_RADIUS) -> tuple[float, float] | None` — 门限过滤 + 反距离加权平均，无合格邻居返回 `None`

- [ ] **Step 1: 写失败测试**

```python
# scripts/tests/test_fast_solve.py
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
```

- [ ] **Step 2: 跑测试确认失败**

```bash
python3 -m pytest scripts/tests/test_fast_solve.py -v
```
Expected: FAIL — `ModuleNotFoundError: No module named 'fast_solve'`

- [ ] **Step 3: 实现**

```python
#!/usr/bin/env python3
"""fast_solve: 可达点姿态偏移的存取与加权最近邻估计。

纯 Python、无 ROS 依赖 (可独立单测)。偏移文件由 workspace_analyzer 生成，
controller 运行时用它对目标点做门限内 kNN 加权平均，把螺旋搜索的锚点
从几何理想值修正到该区域已知可行的姿态附近。

文件格式 (每行): x y z dalpha_deg dbeta_deg
  以 '#' 开头与空行忽略。坐标单位米，角度单位度。
"""

import math
from pathlib import Path

# k 近邻数: 稀疏区/障碍边界处单邻居偏移可能失真, 平均抑制噪声
FAST_SOLVE_K = 3

# 邻居距离门限 = 采样网格 (resolution 0.05m) 的空间对角线。
# 门内邻居的偏移才对目标点有代表性; 门外直接不用 (防外推)。
NEIGHBOR_RADIUS = 0.05 * math.sqrt(3)


def save_offsets(path, records):
    """records: [(x, y, z, d_alpha, d_beta), ...]"""
    with open(path, 'w') as f:
        f.write("# x y z dalpha_deg dbeta_deg (offsets relative to geometric ideal)\n")
        for x, y, z, da, db in records:
            f.write(f"{x:.3f} {y:.3f} {z:.3f} {da:.2f} {db:.2f}\n")


def load_offsets(path):
    """返回 [(x, y, z, d_alpha, d_beta), ...]，坏行跳过。"""
    records = []
    for line in Path(path).read_text().splitlines():
        line = line.strip()
        if not line or line.startswith('#'):
            continue
        parts = line.split()
        if len(parts) != 5:
            continue
        try:
            records.append(tuple(float(v) for v in parts))
        except ValueError:
            continue
    return records


def weighted_offset(records, x, y, z, k=FAST_SOLVE_K, radius=NEIGHBOR_RADIUS):
    """门限内 k 近邻的反距离加权平均偏移。

    :return: (d_alpha, d_beta) 或 None (records 空 / 全部门限外)
    权重 1/dist; 恰有邻居距离为 0 时直接返回该邻居偏移 (避免除零)。
    """
    ranked = sorted(
        ((r[3], r[4], math.sqrt((r[0] - x) ** 2 + (r[1] - y) ** 2 + (r[2] - z) ** 2))
         for r in records),
        key=lambda t: t[2])
    near = [t for t in ranked[:k] if t[2] <= radius]
    if not near:
        return None
    if near[0][2] == 0.0:
        return (near[0][0], near[0][1])
    w_sum = sum(1.0 / t[2] for t in near)
    da = sum(t[0] / t[2] for t in near) / w_sum
    db = sum(t[1] / t[2] for t in near) / w_sum
    return (da, db)
```

- [ ] **Step 4: 跑测试确认通过**

```bash
python3 -m pytest scripts/tests/test_fast_solve.py -v
```
Expected: 5 PASS

- [ ] **Step 5: Commit**

```bash
git add scripts/fast_solve.py scripts/tests/test_fast_solve.py
git commit -m "feat: add fast_solve offset store and knn lookup (no ROS deps)"
```

---

### Task 3: analyzer 接入统一搜索循环并记录偏移

**Files:**
- Modify: `scripts/workspace_analyzer.py`（`check_reachability`，[workspace_analyzer.py:93-128](../workspace_analyzer.py#L93-L128)；`analyze` 循环；`save_results`；`__init__`）

**Interfaces:**
- Consumes: `controller.search_orientation(x, y, z, try_pose, abort_check)`（Task 1）；`fast_solve.save_offsets`（Task 2）
- Produces: `reachable_range/points_{label}_{config_label}.offsets.txt`（格式由 `fast_solve.save_offsets` 定义，Task 4 运行时消费）

- [ ] **Step 1: 重写 `check_reachability` 为"预筛 + 统一循环 + plan-only 后端"**

```python
    def check_reachability(self, x, y, z):
        if not self.is_geometrically_safe(x, y, z):
            return None

        ctrl = self.controller
        x, y, z = float(x), float(y), float(z)

        def _try_plan_only(alpha, beta):
            """plan-only 后端: 与实测 move_to 同一搜索循环，仅规划不执行。"""
            q = ctrl._quaternion_from_zxz(alpha, beta, 0.0)
            target_pose = Pose()
            target_pose.position.x, target_pose.position.y, target_pose.position.z = x, y, z
            target_pose.orientation.x, target_pose.orientation.y = q[0], q[1]
            target_pose.orientation.z, target_pose.orientation.w = q[2], q[3]

            self.move_group.set_pose_target(target_pose)
            result = self.move_group.plan()
            is_success = result[0] if isinstance(result, tuple) else result
            if is_success:
                self.move_group.clear_pose_targets()
                return True
            return False

        # 与实测完全一致的搜索 (同一函数、同一候选序、同一中止语义)
        result = ctrl.search_orientation(
            x, y, z, _try_plan_only,
            abort_check=lambda: not self.keep_running)
        if result is None:
            return None
        _, _, da, db, _dist = result
        return (da, db)
```

注意：原 `check_reachability` 里的 `auto_alpha`/`[0,180]` 预判已由共享循环覆盖（理想 α 即 atan2(x, −y) 且循环内裁剪），删除。`math` 若因此不再被本方法使用则保留 import（文件其他处使用）。

`__init__` 增加 `self.offset_records = []`；`analyze` 内层改为：

```python
                            offsets = self.check_reachability(x, y, z)
                            if offsets is not None:
                                self.reachable_points.append((x, y, z))
                                self.offset_records.append((x, y, z, *offsets))
```

- [ ] **Step 2: sidecar 输出**

`save_results` 追加：

```python
        from fast_solve import save_offsets
        offset_file = self.output_file.replace('.txt', '.offsets.txt')
        save_offsets(offset_file, self.offset_records)
        rospy.loginfo(f"Saved {len(self.offset_records)} offset records to {offset_file}")
```

- [ ] **Step 3: 手动验证（必须真实跑，不可省）**

```bash
# 小采样盒快速验证: 临时把 cfg 中 sampling_box 改为 3×3×3、resolution 0.2
roslaunch moveit_ctrl run_workspace_analyzer.launch arm:=upper
```
Expected:
1. `reachable_range/` 出现 `*.offsets.txt`，行数与 points 文件一致；
2. 首列 x y z 与 points 文件逐行一致；
3. 部分行 dalpha/dbeta 非零（理想姿态不可达、偏移后可达的点）；
4. Ctrl+C 中断时两个文件都落盘（`abort_check` → `finally` 路径）；
5. 中断发生在第 1~2 个候选时进程能及时退出（`abort_check` 生效，无"每候选 0.05s×29 继续跑"的拖尾）。

验证后还原 cfg 采样盒。

- [ ] **Step 4: Commit**

```bash
git add scripts/workspace_analyzer.py
git commit -m "feat: analyzer uses the same search_orientation loop as move_to, records offsets"
```

---

### Task 4: controller 集成 fast_solve

**Files:**
- Modify: `scripts/piper_arm_controller.py`（`__init__`、`from_yaml`、`_priority_candidates`）
- Test: `scripts/tests/test_fast_solve.py`（追加）

**Interfaces:**
- Consumes: `fast_solve.load_offsets` / `fast_solve.weighted_offset`（Task 2）、`search_orientation` / `_priority_candidates`（Task 1）
- Produces: 实例属性 `self._offset_records`（服务模式由 `from_yaml` 填充，直连模式恒空）；`fast_solve.FAST_SOLVE_K = 3` 模块常量。**无构造参数、无 YAML 键**（模式派生，见 Global Constraints）。两种运行模式经 `search_orientation → _priority_candidates` 自动获得偏移候选，无需各自改动。

- [ ] **Step 1: 偏移表加载（仅服务模式；挪到 `from_yaml`，路径在构造后才存在）**

`__init__` 属性赋值块加：

```python
        # fast_solve 由模式派生: 服务模式由 from_yaml 加载偏移表, 直连/分析模式恒空
        self._offset_records = []
```

新增方法：

```python
    def _offset_file_path(self, yaml_path):
        """与 analyzer 输出命名一致: points_{yaml_stem}_{config_label}.offsets.txt"""
        stem = Path(yaml_path).stem          # e.g. piper_upper
        return _SCRIPT_DIR / 'reachable_range' / f'points_{stem}_{self.get_config_label()}.offsets.txt'

    def _load_offset_records(self, yaml_path):
        """加载偏移表; 文件缺失时告警并回退纯螺旋 (调用方继续，_offset_records 保持空)。"""
        import fast_solve
        path = self._offset_file_path(yaml_path)
        if not path.exists():
            rospy.logwarn(f"fast_solve 已开启但偏移文件不存在，回退纯螺旋搜索: {path}")
            return []
        records = fast_solve.load_offsets(path)
        rospy.loginfo(f"fast_solve: 加载 {len(records)} 条偏移记录 ({path.name})")
        return records
```

`from_yaml` 末尾（`get_config_label()` 依赖的 `safety_bbox`/`obstacles` 已在构造时就位）：

```python
        # fast_solve: 仅服务模式 (直连/分析模式是偏移数据生产方, 必须纯螺旋)
        if use_service:
            instance._offset_records = instance._load_offset_records(path)
```

- [ ] **Step 2: `_priority_candidates` 重锚（唯一改动点，两种模式自动生效）**

实现要点：修正螺旋 = 理想锚螺旋整体平移 (Δᾱ, Δβ̄) 后重裁剪 [0,180]——**不**给 `alpha_beta_candidates` 加锚点参数（偏移记录语义保持"相对几何理想"）。`d_alpha/d_beta` 字段含义不变（相对几何理想值的偏移），fast_solve 修正锚候选的 d 值 = (α, β) − 理想值，与分析器记录语义一致。两段螺旋**去重**（rounded key 集合过滤；偏移 (0,0) 或极小时两螺旋重合，不去重会把 29 个候选试两遍）：

```python
    def _priority_candidates(self, x, y, z):
        """搜索候选序列: fast_solve 修正锚螺旋在前, 理想锚螺旋兜底 (去重)。"""
        spiral = self.alpha_beta_candidates(x, y, z)   # 理想锚螺旋
        ideal_a = spiral[0][0] - spiral[0][2]          # 首候选 d=0 → 反解理想值
        ideal_b = spiral[0][1] - spiral[0][3]

        if not self._offset_records:
            return [(a, b, da, db, None) for a, b, da, db in spiral]

        import fast_solve
        off = fast_solve.weighted_offset(self._offset_records, x, y, z)
        if off is None:
            rospy.loginfo("  fast_solve: 无门限内邻居, 使用理想锚螺旋")
            return [(a, b, da, db, None) for a, b, da, db in spiral]
        da_bar, db_bar = off
        rospy.loginfo(f"  fast_solve: 加权偏移 (Δα={da_bar:+.1f}°, Δβ={db_bar:+.1f}°) 重锚螺旋")

        # 修正锚螺旋 = 理想锚螺旋整体平移 + 重裁剪 [0,180]
        # d 值相对几何理想值 (与分析器记录语义一致)
        reanchored = [
            (max(0.0, min(180.0, a + da_bar)),
             max(0.0, min(180.0, b + db_bar)),
             a + da_bar - ideal_a, b + db_bar - ideal_b, 'fast')
            for a, b, da, db in spiral]

        # 去重 (偏移极小/为 0 时两螺旋重合)
        seen, out = set(), []
        for cand in reanchored + [(a, b, da, db, None) for a, b, da, db in spiral]:
            key = (round(cand[0], 3), round(cand[1], 3))
            if key not in seen:
                seen.add(key)
                out.append(cand)
        return out
```
```

- [ ] **Step 3: 单元测试（追加到 test_fast_solve.py）**

```python
def _make_ctrl(records):
    from piper_arm_controller import PiperArmController
    c = object.__new__(PiperArmController)
    c.yaw_step, c.yaw_levels = 10.0, 3
    c.pitch_step, c.pitch_range = 5.0, 20.0
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

    first = cands[0]
    assert first[4] == 'fast'                       # 修正锚候选在前
    assert first[0] == pytest.approx(ideal_a + 15.0)  # 锚点被平移
    assert first[1] == pytest.approx(ideal_b - 10.0)
    assert first[2] == pytest.approx(15.0)          # d 值 = 相对几何理想
    assert all(c_[4] is None or c_[4] == 'fast' for c_ in cands)


def test_priority_candidates_dedup_zero_offset():
    """偏移 (0,0) 时修正螺旋与理想螺旋重合 → 去重后仍 29 个。"""
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
```

- [ ] **Step 4: 跑全部测试**

```bash
python3 -m pytest scripts/tests/ -v
```
Expected: 全部 PASS（test_search 7 + test_fast_solve 5+4）

- [ ] **Step 5: 手动验证（实机，需 demo.launch 在跑）**

先跑 Task 3 的小采样盒生成一份 `.offsets.txt`，然后：

```bash
rosrun moveit_ctrl piper_arm_controller.py upper
```
临时把 `__main__` 的 `move_to` 解开注释指向可达点（如 0.3, 0.0, 0.2），观察：
1. 首行 `fast_solve: 加载 N 条偏移记录`（服务模式自动加载）；
2. 可达目标日志先出现 `fast_solve: 加权偏移 (Δα=…, Δβ=…) 重锚螺旋`，随后修正锚螺旋首个候选即 SUCCESS；
3. 把 `.offsets.txt` 临时改名重启：出现"回退纯螺旋搜索"警告且行为正常。

- [ ] **Step 6: Commit**

```bash
git add scripts/piper_arm_controller.py scripts/tests/test_fast_solve.py
git commit -m "feat: fast_solve priority candidates from offline offset records"
```

---

### Task 5: 全量重新生成偏移数据（运维步骤，无代码）

**Files:**
- 生成: `scripts/reachable_range/points_piper_{upper,lower}_*.offsets.txt`（及更新的 points 文件）

**Interfaces:** 无代码接口。产物由 Task 3 格式定义，Task 4 运行时消费。

- [ ] **Step 1: 确认两臂 move_group 独立可用**（workspace_analyzer 用自带非命名空间 move_group，`demo.launch` 不必在跑）

- [ ] **Step 2: 依次跑两臂全量分析**

```bash
roslaunch moveit_ctrl run_workspace_analyzer.launch arm:=upper
roslaunch moveit_ctrl run_workspace_analyzer.launch arm:=lower
```
预计每臂 30–50min（候选 3 → 29 的代价，见评估风险 4）。跑完核对 `.offsets.txt` 行数 = points 行数。

- [ ] **Step 3: 旧数据归档**

```bash
cd scripts/reachable_range
mkdir -p archive_20260823
# 只归档旧版 points 文件 (无 sidecar 的)；新生成的成对文件保留
ls points_*.txt | while read f; do [ -f "${f%.txt}.offsets.txt" ] || mv "$f" archive_20260823/; done
ls
```

- [ ] **Step 4: 验证 fast_solve 命中率（可选但推荐）**

对若干已知可达点各跑一次 `move_to`，统计日志中"重锚螺旋后首个候选即 SUCCESS"的占比。>50% 即达到方案预期（典型延迟 ≈ 单候选耗时）。

- [ ] **Step 5: Commit（数据文件入库与否按团队习惯）**

```bash
git add scripts/reachable_range/
git commit -m "data: regenerate reachable points with unified search + offset records"
```

---

### Task 6: 文档同步

**Files:**
- Modify: `CLAUDE.md`

- [ ] **Step 1: 更新 Unified Arm Controller 小节**

"Key APIs" 增加两行：

```markdown
- `search_orientation(x, y, z, try_pose, abort_check)` — the **single** orientation search loop; both runtime modes and `workspace_analyzer` are thin `try_pose` backends (service+verify / plan+execute / plan-only), so sim and real share identical candidate order and abort semantics
- `fast_solve` — mode-derived (service mode: on; direct/analysis mode: off, no YAML key). Neighbors within `fast_solve.NEIGHBOR_RADIUS` (`resolution·√3`) of the target are inverse-distance-averaged into a single (Δα, Δβ) that **re-anchors** the spiral (whole spiral translated); the ideal-anchor spiral always follows as fallback, deduped. Falls back to pure spiral when the file is missing/stale (filename embeds `get_config_label()`) or no neighbor passes the radius gate
```

- [ ] **Step 2: 更新 Workspace Analysis 小节**

```markdown
The reachability check calls the **same** `search_orientation` loop as runtime `move_to` (plan-only backend), so "reachable in analysis" ≡ "move_to succeeds on the real arm". Outputs reachable point clouds **and per-point (Δα, Δβ) offset sidecars** (`points_*.offsets.txt`) consumed by `fast_solve`.
```

- [ ] **Step 3: Commit**

```bash
git add CLAUDE.md
git commit -m "docs: unified search loop + fast_solve offset-priority search"
```

---

## Self-Review 结论

1. **Spec 覆盖**：开关（Task 4）、离线记录（Task 3）、最近邻偏移作为优先目标（Task 4 Step 2，经 `_priority_candidates` 插入）、"后续流程不变"（插入而非替换 + 统一循环本身就是原有流程）；追加约束"analyzer 与实测同一搜索函数"由 Task 1 的 `search_orientation` + Task 3 的 plan-only 后端直接满足——analyzer 不再含任何自有遍历逻辑。
2. **占位符**：无 TBD/示意代码；所有步骤含完整代码或精确验证命令。
3. **类型一致性**：`alpha_beta_candidates -> [(a,b,da,db)]`（Task 1）被 `_priority_candidates` 扩展为 `(a,b,da,db,dist|None)` 5 元组；`search_orientation` 返回同构 5 元组，Task 3 解包 `_, _, da, db, _dist` 一致；`fast_solve` 三函数签名 Task 2 定义、Task 3/4 消费一致。
4. **既有修复保持**：`ServiceCallTimeout` 穿透语义写入 Global Constraints 与 Task 1 后端代码（`except ServiceCallTimeout: raise` 先于宽捕获），并有 Step 5 手动回归项；新增公开方法 `search_orientation` 同样加 `@_synchronized`——analyzer 从外部直接调用它时获得互斥保护，而 `move_to` wrapper 已持锁的嵌套调用由 `RLock` 可重入性安全覆盖。
5. **执行顺序**：Task 1、2 无依赖可并行；Task 3 依赖 1+2；Task 4 依赖 1+2；Task 5 依赖 3+4；Task 6 最后。
