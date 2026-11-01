#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""采集层拆解诊断：定位「相机 30 fps 却只跑出 15 fps」到底耗在哪一步。

背景（10-28 19:20）：
    probe_camera.py 实测（都是裸 cap.read() 循环）：
        640x480 默认 YUYV    29.9 fps
        640x480 MJPEG       30.0 fps
        640x480 BUFFERSIZE=1 15.0 fps   <- 坑，但代码里从没设过
    而 diag_fps.py 的 S1（走 session.read_frame，= cap.read + cv2.flip）
        只有 15.0 fps
    两者唯一的代码差别就是 read_frame() 里的 cv2.flip(frame, 1)。
    所以本脚本把 read 这一步切成 4 种实现，同尺子量：

        S1a  裸 cap.read()                      -> 相机 + OpenCV 解码上限
        S1b  cap.read() + cv2.flip              -> 现在的 read_frame()
        S1c  cap.read() + np.ascontiguousarray(f[:, ::-1])  -> numpy 镜像
        S1d  cap.read() + cv2.flip(..., dst=预分配)          -> 免分配镜像

    判据：
        S1a 高、S1b 低        -> cv2.flip 是瓶颈（ARM 上它是 strided 逐元素，
                                 没有 NEON 路径，比 PC 上慢 20~50 倍）
        S1a 低                -> 相机/解码本身到顶，与镜像无关
        S1c 或 S1d 明显高于 S1b -> 换实现即可，不用动功能

    顺带打印协商到的 fourcc：确认 MJPG 真的生效了
    （YUYV 会打印 b'YUYV'，MJPG 打印 b'MJPG'）。

用法（板端）：
    source /usr/local/share/pynq-venv/bin/activate
    python3 -u diag_read.py
"""
import time

import cv2
import numpy as np

import sys
sys.path.insert(0, ".")
from session import open_camera, read_frame

DURATION = 3.0


def rate(fn, seconds=DURATION):
    """跑 fn() 直到满 seconds，返回 (次数, 实测秒, 单次 ms)。"""
    t0 = time.perf_counter()
    n = 0
    while time.perf_counter() - t0 < seconds:
        fn()
        n += 1
    dt = time.perf_counter() - t0
    return n, dt, (dt / n * 1000.0) if n else 0.0


def show(tag, n, dt, ms):
    print("  %-46s %6.1f fps   %7.2f ms/帧  (%d 帧 / %.2f s)"
          % (tag, n / dt, ms, n, dt))


def main():
    print("=" * 74)
    print("采集层拆解：S1a 裸 read / S1b cv2.flip / S1c numpy 镜像 / S1d 预分配 dst")
    print("=" * 74)
    print("OpenCV", cv2.__version__)

    cap = open_camera(0)
    fourcc = int(cap.get(cv2.CAP_PROP_FOURCC)).to_bytes(4, "little")
    print("协商结果: %dx%d  fourcc=%r  fps回报=%.1f"
          % (cap.get(cv2.CAP_PROP_FRAME_WIDTH), cap.get(cv2.CAP_PROP_FRAME_HEIGHT),
             fourcc, cap.get(cv2.CAP_PROP_FPS)))
    if fourcc != b"MJPG":
        print("  [!] fourcc 不是 MJPG —— open_camera() 的设置没生效，"
              "先查 session.py 是不是板上那一版")

    # 预热：前几帧常有自动对焦/自动曝光收敛，不预热会把结论带偏
    print("\n预热 10 帧 ...")
    for _ in range(10):
        read_frame(cap)

    # S1a 裸读
    n, dt, ms = rate(lambda: cap.read())
    show("S1a 裸 cap.read()", n, dt, ms)
    base = n / dt

    # S1b 现在的 read_frame（cap.read + cv2.flip）
    n, dt, ms = rate(lambda: read_frame(cap))
    show("S1b cap.read + cv2.flip   （现在的 read_frame）", n, dt, ms)
    cur = n / dt

    # S1c numpy 负 stride 视图 + 强制连续化
    def np_mirror():
        ok, f = cap.read()
        if not ok or f is None:
            return None
        return np.ascontiguousarray(f[:, ::-1])

    n, dt, ms = rate(np_mirror)
    show("S1c cap.read + numpy [:, ::-1] 镜像", n, dt, ms)
    npm = n / dt

    # S1d cv2.flip 写进预分配缓冲
    _ok, _f = cap.read()
    dst = np.empty_like(_f)

    def flip_dst():
        ok, f = cap.read()
        cv2.flip(f, 1, dst=dst)
        return dst

    n, dt, ms = rate(flip_dst)
    show("S1d cap.read + cv2.flip(dst=预分配)", n, dt, ms)
    pre = n / dt

    # ---------------- 单独量一次 cv2.flip 的纯耗时 ----------------
    # 读一帧固定图，反复 flip 同一块内存：排除采集干扰，纯粹测 flip 本身
    ok, fixed = cap.read()
    if ok:
        n, dt, ms = rate(lambda: cv2.flip(fixed, 1))
        print("\n  单独测 cv2.flip（同一块 640x480x3，反复翻转）: %7.2f ms/次" % ms)
        n, dt, ms = rate(lambda: np.ascontiguousarray(fixed[:, ::-1]))
        print("  单独测 numpy  [:, ::-1] + ascontiguousarray   : %7.2f ms/次" % ms)
        n, dt, ms = rate(lambda: cv2.cvtColor(fixed, cv2.COLOR_BGR2GRAY))
        print("  单独测 cv2.cvtColor BGR2GRAY（对比基准）       : %7.2f ms/次" % ms)

    cap.release()

    # ---------------- 结论 ----------------
    print("\n" + "=" * 74)
    print("怎么读：")
    print("  S1a ≈ 30，S1b ≈ 15  ->  flip 单独吃掉约 (1/S1b - 1/S1a) 秒，"
          "它就是掉帧主因")
    print("  S1c 或 S1d 接近 S1a  ->  换掉 read_frame() 里的 flip 实现即可恢复 30 fps")
    print("  S1a 本身就低         ->  相机/解码到顶，与 flip 无关，"
          "去查 MJPG 解码或 USB")
    print("  30 fps 对应预算 33.3 ms/帧；下面这些都要从这 33.3 ms 里扣：")
    print("      cv2.cvtColor 灰度化、AGC 的 cv2.mean、PL kernel 3.07 ms、")
    print("      cache 维护 0.40 ms、网页推送")
    print("=" * 74)
    print("当前: S1a=%.1f  S1b=%.1f  S1c=%.1f  S1d=%.1f" % (base, cur, npm, pre))
    print("=" * 74)


if __name__ == "__main__":
    main()
