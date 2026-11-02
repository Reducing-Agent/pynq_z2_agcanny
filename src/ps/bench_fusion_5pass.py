#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""bench_fusion_5pass.py — 「融合到底省了多少」5 趟线性实验（真板）

目的
----
把「5 个算子不融合 = 5 趟访存」从**推算**变成**实测**。

Canny 链共 5 个算子（5×5 高斯 → Sobel → 方向量化 → NMS → 双阈值+滞后）。
不融合的做法是每个算子各跑一趟，每趟都要把整帧读出 DDR 再写回。
本脚本用「同一个单算子连跑 n 趟」来模拟它，再与「四级融合一趟」对比。

为什么可以用 op 1（SOBEL）代表"任意一趟"
----------------------------------------
已实测：op 1~8 单算子核内耗时 3.253~3.261 ms，彼此差不到 1%（见 validate_v8.py
阶段 5）。所以"趟数 × 单趟耗时"这个近似是成立的，本脚本顺带检验它的线性度。

口径（两档，严禁混配）
------------------------
  核内档   = sum(proc.last_pl_ms)            ap_start→ap_done，不含 ARM 打包
  端到端档 = time.perf_counter() 包住全部 run  含 ARM 打包 + cache 维护

报告 §1.3 用的是**核内档**；答辩口头讲融合收益时如果要说"整机快了多少"，
必须用**端到端档**，并且要说明含打包。

用法（板端）
------------
    source /usr/local/share/pynq-venv/bin/activate
    cd rgb2gray_v82
    python3 -u bench_fusion_5pass.py

注意：一律**不要加 sudo**：env_reset 会让 Device.devices 变空 → No Devices Found。
   迷惑点在于"纯 numpy 的代码全 PASS，一 import pynq 就崩"，很容易误判成驱动坏了。
"""

import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

try:
    # 只导入确有导出的两个名字：hw_driver 里模式表叫 _MODE_NAMES（带下划线），
    # 不要顺手加 MODE_NAMES，否则 ImportError 会让整个脚本退出。
    from hw_driver import HardwareProcessor, GAIN_UNITY
except Exception as exc:                                  # pragma: no cover
    print("[!] 无法导入 hw_driver：%r" % (exc,))
    print("    请确认在 rgb2gray_v82/ 目录下运行，且已 source pynq-venv。")
    raise SystemExit(1)


W, H = 640, 480
ROUNDS = 3                 # 每个 n 重复测几轮，取 min
PASSES = (1, 2, 3, 4, 5)
SINGLE_OP = 1              # op 1 SOBEL：作为"不融合时的一趟"代表
FUSED_OP = 9               # op 9 GAUSS5：四级融合，一趟走完四级（抽第 1 级写回）
FUSED_NAME = "GAUSS5(融合)"


def make_frame():
    """合成噪声图（BGR 640×480），与 validate_v8.py 阶段 5 同类型。

    用固定种子保证可复现；纯合成图的好处是不依赖摄像头光照，
    换一台机器跑也能得到同一量级的数。
    """
    rng = np.random.default_rng(20261031)
    return rng.integers(0, 256, (H, W, 3), dtype=np.uint8)


def bench(proc, frame, op, n):
    """连跑 n 趟 op，返回 (端到端 ms, 核内累计 ms)。取 ROUNDS 轮的最小值。"""
    best = None
    for _ in range(ROUNDS):
        for _ in range(2):              # 预热：第一轮含冷启动开销，不算
            proc.run(op, frame)
        pl_sum = 0.0
        t0 = time.perf_counter()
        for _ in range(n):
            proc.run(op, frame)
            pl_sum += proc.last_pl_ms
        e2e = (time.perf_counter() - t0) * 1e3
        if best is None or e2e < best[0]:
            best = (e2e, pl_sum)
    return best


def main():
    proc = HardwareProcessor()
    frame = make_frame()
    # 锁死增益，避免 AGC 随噪声图自动变化导致各趟不可比
    proc.gain_override = GAIN_UNITY

    print("=" * 74)
    print("融合 vs 多趟：5 趟线性实验（%d×%d，合成噪声图，增益锁 1.0×）" % (W, H))
    print("=" * 74)
    print("位流：%s" % getattr(proc, "bit_md5", "（未知）"))
    print()

    rows = []
    print("  趟数 |  端到端(ms) | 核内累计(ms) | 单趟端到端 | 单趟核内")
    print("  -----+-------------+--------------+------------+----------")
    for n in PASSES:
        e2e, pl = bench(proc, frame, SINGLE_OP, n)
        rows.append((n, e2e, pl))
        print("   %2d  |  %9.3f  |  %10.3f  | %9.3f  | %8.3f"
              % (n, e2e, pl, e2e / n, pl / n))

    f_e2e, f_pl = bench(proc, frame, FUSED_OP, 1)
    print()
    print("  融合 op %d %s 一趟：端到端 %.3f ms   核内 %.3f ms"
          % (FUSED_OP, FUSED_NAME, f_e2e, f_pl))

    n5_e2e = rows[-1][1]
    n5_pl = rows[-1][2]

    print()
    print("-" * 74)
    print("结论（可直接贴进报告 §1.3）")
    print("-" * 74)
    print("  单趟：核内 %.3f ms / 端到端 %.3f ms" % (rows[0][2], rows[0][1]))
    print("  5 趟：核内 %.3f ms / 端到端 %.3f ms" % (n5_pl, n5_e2e))
    print("  融合 1 趟：核内 %.3f ms / 端到端 %.3f ms" % (f_pl, f_e2e))
    print()
    if f_pl > 0:
        print("  融合倍率（核内口径）  = %.3f / %.3f = **%.2f×**"
              % (n5_pl, f_pl, n5_pl / f_pl))
    if f_e2e > 0:
        print("  融合倍率（端到端口径）= %.3f / %.3f = **%.2f×**"
              % (n5_e2e, f_e2e, n5_e2e / f_e2e))
    print("    （报告正文用核内口径；口头讲整机时用端到端口径并注明含打包）")

    # ---- 线性度检验：单趟端到端是否随 n 基本恒定 ----
    per = [r[1] / r[0] for r in rows]
    spread = max(per) - min(per)
    print()
    print("  线性度检验：n=1..5 的单趟端到端极差 = %.3f ms" % spread)
    if spread < 1.0:
        print("    → 极差 < 1 ms，'趟数 × 单趟' 假设**成立**，上述倍率可信。")
    else:
        print("    → 极差偏大，趟数增加时单趟耗时有漂移（可能是 cache/DDR 争用），")
        print("      报告中请按实测表格照录，不要直接引用理论倍率。")

    # ---- 与 2 趟锚点交叉验证 ----
    two = rows[1][1] if len(rows) > 1 else None
    if two:
        print()
        print("  交叉验证：本脚本 2 趟端到端 %.3f ms；"
              "validate_v8.py 阶段 5 的 2 趟级联锚点 21.764 ms" % two)
        print("    （两者口径略有差别：级联第 2 趟省掉一次 BGR 打包，会更快一些）")
    print("=" * 74)


if __name__ == "__main__":
    main()
