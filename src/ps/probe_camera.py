#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""C270 采集能力探测：弄清相机在不同分辨率/格式/参数下真实能给多少 fps。

背景（10-28 19:07）：diag_fps 显示 S1 纯采集只有 15.0 fps，
而 PL 链路只占 14 ms（能跑 70 fps）—— **瓶颈 100% 在相机**。
相机向 OpenCV 报告 fps=30.0，但那是**请求值**，不是真实值。

本脚本逐个组合实测，每项 3 秒：
    A  640x480  默认（YUYV/MJPEG 由驱动决定）
    B  640x480  强制 MJPEG fourcc      <- USB 带宽占用低，通常能提速
    C  320x240  默认                  <- 数据量降到 1/4
    D  640x480  关自动曝光 + 手动曝光  <- 暗环境下自动曝光会主动降帧
    E  640x480  CAP_PROP_BUFFERSIZE=1 <- 减少 V4L2 缓冲积压

用法（板端）：
    source /usr/local/share/pynq-venv/bin/activate
    python3 -u probe_camera.py
"""
import time

import cv2

DURATION = 3.0


def bench(cap, label, extra=""):
    """预热后测 3 秒，返回 fps。"""
    for _ in range(5):
        cap.read()
    t0 = time.perf_counter()
    n = 0
    while time.perf_counter() - t0 < DURATION:
        ok, f = cap.read()
        if not ok:
            break
        n += 1
    dt = time.perf_counter() - t0
    fps = n / dt if dt > 0 else 0.0
    w = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
    h = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
    print("  %-34s %6.1f fps   (回报 %dx%d)  %s"
          % (label, fps, w, h, extra))
    return fps


def try_open(idx=0, fourcc=None, w=640, h=480, fps=30, extra_sets=None):
    cap = cv2.VideoCapture(idx, cv2.CAP_V4L2)
    if not cap.isOpened():
        return None
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
    cap.set(cv2.CAP_PROP_FPS, fps)
    if fourcc:
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
    if extra_sets:
        for prop, val in extra_sets.items():
            cap.set(prop, val)
    ok, f = cap.read()
    if not ok or f is None:
        cap.release()
        return None
    return cap


def main():
    print("=" * 74)
    print("C270 采集能力探测（每项 3 秒）")
    print("=" * 74)
    print("OpenCV", cv2.__version__)

    results = {}

    # A 默认
    cap = try_open()
    if cap is None:
        print("打不开相机（CAP_V4L2）。检查是否被其他程序占用。")
        return
    results["A 640x480 默认"] = bench(cap, "A 640x480 默认")
    print("       fourcc 回报 = %r"
          % int(cap.get(cv2.CAP_PROP_FOURCC)).to_bytes(4, "little"))
    cap.release()

    # B 强制 MJPEG
    cap = try_open(fourcc="MJPG")
    if cap:
        results["B 640x480 MJPEG"] = bench(cap, "B 640x480 MJPEG")
        cap.release()
    else:
        print("  %-34s   打不开" % "B 640x480 MJPEG")

    # C 320x240
    cap = try_open(w=320, h=240)
    if cap:
        results["C 320x240 默认"] = bench(cap, "C 320x240 默认")
        cap.release()

    # D 关自动曝光
    cap = try_open(extra_sets={cv2.CAP_PROP_AUTO_EXPOSURE: 0.25,
                               cv2.CAP_PROP_EXPOSURE: -6})
    if cap:
        results["D 640x480 手动曝光"] = bench(cap, "D 640x480 手动曝光(-6)")
        cap.release()

    # E 小缓冲
    cap = try_open(extra_sets={cv2.CAP_PROP_BUFFERSIZE: 1})
    if cap:
        results["E 640x480 BUFFERSIZE=1"] = bench(cap, "E 640x480 BUFFERSIZE=1")
        cap.release()

    # F 强制 YUYV（有些驱动默认走 MJPEG 反而慢）
    cap = try_open(fourcc="YUYV")
    if cap:
        results["F 640x480 YUYV"] = bench(cap, "F 640x480 YUYV")
        cap.release()

    print("\n" + "=" * 74)
    print("汇总（按 fps 排序）")
    print("=" * 74)
    for k, v in sorted(results.items(), key=lambda kv: -kv[1]):
        bar = "#" * int(v / 2)
        print("  %-30s %6.1f  %s" % (k, v, bar))

    best = max(results, key=results.get)
    print("\n最佳：%s = %.1f fps" % (best, results[best]))
    print("""
怎么用这个结果：
  * 若 MJPEG/YUYV 明显更快 -> 改 session.py 的 open_camera()，加 fourcc 设置
  * 若 320x240 明显更快    -> 说明 USB 带宽是瓶颈；PL 支持任意尺寸，
                               但报告主口径是 640x480，需权衡
  * 若手动曝光更快         -> 暗环境下相机自动曝光在主动降帧，
                               可在报告里作为"低照度"佐证，但要手动设曝光
  * 若全部都是 15          -> C270 本身在 PYNQ-Z2 上的实际上限，
                               把指标从"30 fps"改成"相机上限 15 fps，
                               PL 链路 70 fps，端到端延迟 8.9 ms"
""")


if __name__ == "__main__":
    main()
