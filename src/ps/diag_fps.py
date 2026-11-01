#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""帧率劣化诊断：把 PIPE / 采集 / CLIENT 三层拆开量，定位掉帧发生在哪一层。

背景（10-28 18:54）：用户报告"CLIENT 刚开始 20 多帧，后面掉到十几帧"。
已知 CLIENT 有 --raw-fps（默认 25）硬限流，所以 20 多帧是正常的；
要查的是**为什么后来掉到十几**。

本脚本依次测 4 组，每组 3 秒，打印实际帧率：
    S1  只采集（不处理、不 AGC）      -> 相机本身能给多少
    S2  采集 + PL 滤镜，AGC 关        -> PL 链路帧率
    S3  采集 + PL 滤镜，AGC 开        -> AGC 到底拖不拖
    S4  同 S3，但持续报告每 0.5s 帧率 -> 看是不是**渐进劣化**

用法（板端）：
    source /usr/local/share/pynq-venv/bin/activate
    python3 -u diag_fps.py
"""
import sys
import time

import cv2
import numpy as np

sys.path.insert(0, ".")
import hw_driver as H
from session import open_camera, read_frame

DURATION = 3.0


def rate(fn, seconds=DURATION):
    """跑 fn() 直到满 seconds，返回 (次数, 实测秒)。"""
    t0 = time.perf_counter()
    n = 0
    while time.perf_counter() - t0 < seconds:
        fn()
        n += 1
    return n, time.perf_counter() - t0


def main():
    print("=" * 66)
    print("帧率诊断：S1 采集 / S2 PL(无AGC) / S3 PL(AGC) / S4 渐进观察")
    print("=" * 66)

    cap = open_camera(0)
    print("相机：%dx%d  fps=%.1f（OpenCV 报告的请求值，不一定是真实值）"
          % (cap.get(cv2.CAP_PROP_FRAME_WIDTH), cap.get(cv2.CAP_PROP_FRAME_HEIGHT),
             cap.get(cv2.CAP_PROP_FPS)))

    # 预热：第一帧常常特别慢（自动对焦/自动曝光收敛）
    for _ in range(10):
        read_frame(cap)

    # ---------------- S1 纯采集 ----------------
    n, dt = rate(lambda: read_frame(cap))
    print("\nS1 纯采集            : %6.1f fps  (%d 帧 / %.2f s)"
          % (n / dt, n, dt))

    # ---------------- S2 PL，AGC 关 ----------------
    proc = H.HardwareProcessor("rgb2gray.bit")
    if proc._sim:
        print("[!] 没加载到 PL，跑的是 SIM —— 结论只对软件路径有效")
    proc.agc_enabled = False
    n, dt = rate(lambda: proc.run(1, read_frame(cap)))
    print("S2 PL 滤镜，AGC 关   : %6.1f fps  (%d 帧 / %.2f s)"
          % (n / dt, n, dt))
    print("     其中 pack %.3f ms / kernel %.3f ms"
          % (proc.last_pack_ms, proc.last_pl_ms))

    # ---------------- S3 PL，AGC 开 ----------------
    proc.agc_enabled = True
    n, dt = rate(lambda: proc.run(1, read_frame(cap)))
    print("S3 PL 滤镜，AGC 开   : %6.1f fps  (%d 帧 / %.2f s)"
          % (n / dt, n, dt))
    print("     其中 pack %.3f ms / kernel %.3f ms   mu=%.1f gain=%d"
          % (proc.last_pack_ms, proc.last_pl_ms, proc.last_mu, proc.last_gain))

    # ---------------- S4 渐进观察 ----------------
    print("\nS4 每 0.5 s 报一次（跑 10 秒，看是不是越跑越慢）")
    print("     这期间请在镜头前正常活动，模拟现场演示")
    print("     %-8s %-10s %-10s %-10s" % ("时刻", "fps", "pack(ms)", "mu/gain"))
    t0 = time.perf_counter()
    n = 0
    seg = 0
    seg_t0 = t0
    while time.perf_counter() - t0 < 10.0:
        proc.run(1, read_frame(cap))
        n += 1
        now = time.perf_counter()
        if now - seg_t0 >= 0.5:
            fps = n / (now - seg_t0)
            print("     %-8.1f %-10.1f %-10.3f %.1f / %d"
                  % (now - t0, fps, proc.last_pack_ms, proc.last_mu, proc.last_gain))
            n = 0
            seg_t0 = now
            seg += 1

    cap.release()
    print("\n" + "=" * 66)
    print("怎么读：")
    print("  S1 就低（<20）        -> 相机/曝光问题，与我们的代码无关")
    print("  S1 高但 S2 低          -> PL 链路是瓶颈（看 pack/kernel 哪项大）")
    print("  S2 高但 S3 低          -> AGC 在拖（看 pack 是否涨了）")
    print("  S4 里 fps 一路下滑     -> 渐进劣化：查 CPU 竞争/内存/相机自动曝光")
    print("  参考：--raw-fps 默认 25，CLIENT 帧率上限就是 25")
    print("=" * 66)


if __name__ == "__main__":
    main()
