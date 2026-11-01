#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""采集上限再探测：cap.read() 已是唯一瓶颈（24.5 fps），逼出它的真实上限。

背景（10-28 19:25，diag_read.py 实测）：
    S1a 裸 cap.read()                24.5 fps   40.77 ms/帧
    S1b + cv2.flip（现在的 read_frame）23.9 fps   41.83 ms/帧  <- flip 只花 1 ms，无罪
    S1c + numpy [:, ::-1] 镜像        12.9 fps   77.67 ms/帧  <- numpy 54 ms，确认更慢
    S1d + cv2.flip(dst=预分配)         24.5 fps   40.88 ms/帧
    单独测：cv2.flip 12.41 ms / numpy 54.20 ms / cvtColor BGR2GRAY 4.94 ms

    结论：MJPG 把采集从 15 fps 救到 24.5 fps；剩下的全是 cap.read() 自己的 40.77 ms。

本脚本回答三个问题：
    Q1  相机到底在产多少帧？
        -> 用 CAP_PROP_POS_MSEC 区分「相机产得慢」和「我们读得慢」。
           如果 POS_MSEC 的帧间隔 ≈ 33.3 ms，说明相机在产 30 fps，
           是我们这边读不过来（有优化空间）；
           如果 ≈ 40.8 ms，说明相机本身只产 24.5 fps，改代码没用。
    Q2  换 fourcc / 分辨率有没有用？  -> MJPG vs YUYV，640x480 vs 320x240
    Q3  是不是越跑越慢？            -> 15 秒长跑，每 3 秒报一次

用法（板端）：
    source /usr/local/share/pynq-venv/bin/activate
    python3 -u probe_read.py
"""
import time

import cv2

DUR = 4.0


def open_one(fourcc=None, w=640, h=480, fps=30, extra=None):
    cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
    if not cap.isOpened():
        return None
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
    cap.set(cv2.CAP_PROP_FPS, fps)
    if fourcc:
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
    if extra:
        for k, v in extra.items():
            cap.set(k, v)
    ok, f = cap.read()
    if not ok or f is None:
        cap.release()
        return None
    return cap


def bench(cap, label, seconds=DUR):
    """预热后测 seconds 秒，同时抓 POS_MSEC 看相机侧真实产出节奏。"""
    for _ in range(5):
        cap.read()
    pos0 = cap.get(cv2.CAP_PROP_POS_MSEC)
    t0 = time.perf_counter()
    n = 0
    while time.perf_counter() - t0 < seconds:
        ok, f = cap.read()
        if not ok:
            print("  %-30s 读取失败，中止" % label)
            return 0.0, 0.0
        n += 1
    dt = time.perf_counter() - t0
    pos1 = cap.get(cv2.CAP_PROP_POS_MSEC)
    fps = n / dt
    # POS_MSEC 是最后一帧的时间戳（毫秒）。帧间隔 = 相机侧的生产节奏。
    if pos1 > pos0 and n > 1:
        cam_gap = (pos1 - pos0) / (n - 1)
        cam_fps = 1000.0 / cam_gap if cam_gap > 0 else 0.0
        gap_txt = "%5.1f ms/帧 (相机 %.1f fps)" % (cam_gap, cam_fps)
    else:
        gap_txt = "POS_MSEC 不可用"
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fc = int(cap.get(cv2.CAP_PROP_FOURCC)).to_bytes(4, "little")
    print("  %-30s %6.1f fps  %6.2f ms/帧   %s   [%dx%d %r]"
          % (label, fps, dt / n * 1000, gap_txt, w, h, fc))
    return fps, cam_fps if "POS_MSEC 不可用" not in gap_txt else 0.0


def main():
    print("=" * 96)
    print("采集上限再探测（每项 %.0f 秒）：四组配置 + 15 秒长跑" % DUR)
    print("=" * 96)
    print("OpenCV", cv2.__version__)
    print()
    print("关键看第二列「相机 xx fps」：")
    print("  它 = 30  -> 相机在产 30 fps，是我们读不过来 -> 有优化空间")
    print("  它 = 24  -> 相机本身只产 24.5 fps -> 改代码没用，改指标描述")
    print()

    res = {}

    # A 现在的配置：MJPG 640x480（session.py 正在用的）
    cap = open_one(fourcc="MJPG")
    if cap:
        res["A MJPG 640x480 (当前)"] = bench(cap, "A MJPG 640x480 (当前)")
        cap.release()

    # B YUYV 640x480：不解码，但 USB 2 B/px
    cap = open_one(fourcc="YUYV")
    if cap:
        res["B YUYV 640x480"] = bench(cap, "B YUYV 640x480")
        cap.release()

    # C MJPG 320x240：数据量 1/4
    cap = open_one(fourcc="MJPG", w=320, h=240)
    if cap:
        res["C MJPG 320x240"] = bench(cap, "C MJPG 320x240")
        cap.release()

    # D MJPG 640x480 + 手动曝光：暗环境下自动曝光会主动降帧
    cap = open_one(fourcc="MJPG",
                   extra={cv2.CAP_PROP_AUTO_EXPOSURE: 0.25,
                          cv2.CAP_PROP_EXPOSURE: -6})
    if cap:
        res["D MJPG 手动曝光(-6)"] = bench(cap, "D MJPG 手动曝光(-6)")
        cap.release()

    # ---------------- 长跑：看是否渐进劣化 ----------------
    print("\n" + "-" * 96)
    print("E 长跑 15 秒（MJPG 640x480，每 3 秒报一次）—— 验证「越跑越慢」")
    print("-" * 96)
    cap = open_one(fourcc="MJPG")
    if cap:
        for _ in range(5):
            cap.read()
        t0 = time.perf_counter()
        seg_t0 = t0
        n = 0
        seg = 0
        while time.perf_counter() - t0 < 15.0:
            ok, f = cap.read()
            if not ok:
                break
            n += 1
            now = time.perf_counter()
            if now - seg_t0 >= 3.0:
                fps = n / (now - seg_t0)
                pos = cap.get(cv2.CAP_PROP_POS_MSEC)
                print("   t=%4.1fs   %5.1f fps   %6.2f ms/帧   POS_MSEC=%.0f"
                      % (now - t0, fps, (now - seg_t0) / n * 1000, pos))
                n = 0
                seg_t0 = now
                seg += 1
        cap.release()

    # ---------------- 汇总 ----------------
    print("\n" + "=" * 96)
    print("汇总（按 实测 fps 排序）")
    print("=" * 96)
    for k, (fps, cam) in sorted(res.items(), key=lambda kv: -kv[1][0]):
        print("  %-26s 实测 %5.1f fps   相机侧 %5.1f fps" % (k, fps, cam))
    best = max(res, key=lambda k: res[k][0])
    bfps, bcam = res[best]
    print("\n最佳：%s = %.1f fps（相机侧 %.1f fps）" % (best, bfps, bcam))
    print("""
怎么用：
  * 若最佳配置相机侧就是 30 -> 换那个 fourcc/分辨率，继续抠 cap.read()
  * 若所有配置相机侧都 ≈24  -> C270 在 PYNQ-Z2 上的实际上限，
                                报告口径改为「采集 24.5 fps / PL kernel 3.07 ms
                                (327 fps) / 端到端延迟 8.9 ms」，
                                并说明瓶颈在 USB 相机不在 PL
  * 若长跑 E 里 fps 一路下滑  -> 是热或自动曝光，查 D 组（手动曝光）
""")


if __name__ == "__main__":
    main()
