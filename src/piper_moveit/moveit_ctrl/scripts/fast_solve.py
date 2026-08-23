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
