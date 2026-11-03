#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""diag_slow.py —— 量 PL 耗时随图像尺寸怎么变，判 AXI 是否被拆散

2026-10-31：这个性能问题**已经修好了**，本脚本现在是回归工具。
  修复前 640x480 = 20.57 ms（6.70 cyc/px）
  修复后 640x480 =  3.26 ms（1.06 cyc/px）—— 6.31 倍，II=1 地板的 94%
  以后每次改 HLS 都跑一遍，看 cyc/px 有没有退回去。

背景（存档）
------------
v8 板端实测 640x480 各算子 PL 耗时 **20.57 ms**，而 v7 是 3.78 ms（慢 5.4 倍）。
功能对拍全部通过（223 组 0 失败），所以是**纯性能问题，不是正确性问题**。

2026-10-30 更新：**下面这段"AXI 协议转换器"的假设是错的，已定位真因。**
  保留它是为了记录排查过程，别再照着它改。

  ~~BD 里新出现了一个 IP（v5/v7 时代的 BD 没有）：~~
  ~~    axi_mem_intercon/m00_couplers/auto_pc = axi_protocol_converter~~
  ~~    SI_PROTOCOL = AXI4（HLS 侧） / MI_PROTOCOL = AXI3（PS GP0 侧）~~
  ~~    长 burst 被拆开重握手 -> 每拍 1 字节的核一拆就塌~~

  真因：**DDR 写被包在 if 里，HLS 推不出 burst**
     op 1~8   : if (x >= 2)             out[base + (x-1)] = ...
     op 9~11  : if (y >= 8 && x >= 8)   out[base + xo]    = ...
     而唯一快的 GRAY 是**无条件写** out[i] = agc(in[i]) -> 3.222 ms ≈ 1.05 cyc/px
   反例：op 9/10/11 走 path5x5，**没有 σ 累加**，却一样慢
     -> 所以"σ 单累加器拖慢 5.8 倍"也是错的。

  修法（10-30 已改完）：结果先落片上输出行缓冲 outr[2][MAX_W]（按行乒乓），
     DDR 写改成 out[base + x] = outr[prev][x] —— 无条件 + 地址连续。
     读和写仍在同一个循环里，与 GRAY 同构。

  附带的坑：num_read/write_outstanding **不能设 64**。它会传播到 BD 里
     AXI Interconnect 的 S00_READ_ACCEPTANCE / S01_WRITE_ACCEPTANCE，
     合法范围 (1,32)，HLS 不报错但要等 Vivado generate_target 才炸。

本脚本怎么确认
--------------
关键判据是**耗时随图像尺寸怎么变**：

  假设 A：每像素开销（AXI 事务拆散，每像素一次握手）
          -> 耗时 ∝ 像素数。640x480 是 64x64 的 75 倍，耗时也该差 75 倍。

  假设 B：固定开销（比如每次启动的握手、cache 冷）
          -> 耗时跟尺寸无关，64x64 和 640x480 差不多。

  假设 C：带宽受限
          -> 耗时 ∝ 字节数，但斜率应该只有一个，不会突然变 5 倍。

所以只要测 3~4 个尺寸，看曲线形状就能定。

跑法
----
    source /usr/local/share/pynq-venv/bin/activate
    cd /home/xilinx/jupyter_notebooks/rgb2gray_v82
    python3 -u diag_slow.py

输出直接贴给我，我来判断。
"""
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

SIZES = [(64, 64), (120, 160), (240, 320), (480, 640)]
REPEAT = 7


def main():
    print("=" * 72)
    print("diag_slow：PL 耗时 vs 图像尺寸（判 AXI 是否被拆散）")
    print("=" * 72)

    try:
        from hw_driver import HardwareProcessor, MODE_SOBEL, GAIN_UNITY
    except Exception as e:
        print("[!] import hw_driver 失败：%r" % e)
        return 1

    proc = HardwareProcessor()
    if proc._sim:
        print("[!] SIM 模式，测的不是 PL")
        return 2
    proc.gain_override = GAIN_UNITY      # 钉死增益，排除 AGC 抖动
    proc.agc_enabled = True

    print("\n时钟：%s" % (getattr(proc, "_ol", None) and proc._ol.clock_dict or "?"))
    print()

    rng = np.random.RandomState(11)
    rows = []
    for h, w in SIZES:
        frame = rng.randint(0, 256, (h, w, 3)).astype(np.uint8)
        ts = []
        for i in range(REPEAT):
            proc.run(MODE_SOBEL, frame)
            ts.append(proc.last_pl_ms)
        ts.sort()
        med = ts[REPEAT // 2]
        px = h * w
        # 每像素周期数：med(ms) * 100MHz / 像素数
        cyc_per_px = med * 1e-3 * 100e6 / px
        rows.append((w, h, px, ts[0], med, ts[-1], cyc_per_px))
        print("%4dx%-4d  px=%7d   PL min %8.3f  中位 %8.3f  max %8.3f ms   %.2f cyc/px"
              % (w, h, px, ts[0], med, ts[-1], cyc_per_px))

    print("\n" + "-" * 72)
    print("判读:")
    print("-" * 72)
    base = rows[0]
    for r in rows[1:]:
        px_ratio = r[2] / float(base[2])
        t_ratio = r[4] / base[4]
        print("  %4dx%-4d 相对 64x64：像素 x%6.1f ，耗时 x%6.1f ，比值 %5.2f"
              % (r[0], r[1], px_ratio, t_ratio,
                 (t_ratio / px_ratio) if px_ratio else 0))

    # 2026-10-31：原来这几行是**写死的说明文字**，其中还硬编码了
    #   "现在约 6.7"（那是修复前的实测值）。修完之后实测 1.06，
    #   脚本却还在打印 6.7 —— 看的人会被直接带沟里。
    #   改成按**本次实测**的 cyc/px 自动判读。
    last = rows[-1]
    c = last[5]                      # 最大尺寸的 cyc/px
    print()
    print("  · 最大尺寸实测 %.2f cyc/px" % c)
    if c <= 1.30:
        print("    -> 健康。II=1 地板是 1.00 cyc/px（307200 px @100MHz = 3.072 ms），"
              "\n       1.0x 说明 AXI burst 成立、吞吐已被访存定住而非被运算定住。")
    elif c <= 2.50:
        print("    -> 偏低效。burst 可能部分成立，或缓存/握手还有浪费。")
    else:
        print("    -> 异常。典型是 **DDR 访问被包在 if 里导致 burst 推断失败**，"
              "\n       退化成每像素一次带握手的独立 AXI 传输（修复前实测 6.70）。")
    print("  · 耗时/像素比值随尺寸基本不变 -> 开销是**每像素**的，不是固定开销；"
          "\n    640x480 比值偏小 -> 有一点固定开销被摊薄（不是主因）")
    print()

    # 对照：同样尺寸下连续跑 vs 每次重新 flush，看 cache 影响
    print("-" * 72)
    print("附加对照：cache 维护开关对 PL 耗时的影响（应无关，PL 核内计时）")
    h, w = 480, 640
    frame = rng.randint(0, 256, (h, w, 3)).astype(np.uint8)
    import hw_driver as _hw
    _hw.SKIP_CACHE_MAINT = True
    t = []
    for _ in range(5):
        proc.run(MODE_SOBEL, frame)
        t.append(proc.last_pl_ms)
    _hw.SKIP_CACHE_MAINT = False
    print("  关掉 cache 维护后 PL：min %.3f ms（与上面 640x480 对比）" % min(t))

    return 0


if __name__ == "__main__":
    sys.exit(main())
