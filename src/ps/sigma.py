#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Immerkær 噪声估计 + 标定（D4/D5 的 P0：创新 30 分的核心证据）

为什么先做这个而不是先改 PL
--------------------------------
`18_噪声感知闭环_1026.md` 第五节自己写了：
    「P0 = ② 参数生成(150 LUT) + ① σ 估计(200 LUT) = 350 LUT 拿走 80% 收益」
    「P0 的 numpy 版一天能写完，先有数据再决定要不要搬 PL」

σ 估计本质是算法，**不需要等 Vivado**。先在这里拿到真实标定数据
（σ_est vs σ_gt 的相对误差 + 照度扫描曲线），报告和创新点就有实证；
搬不搬 PL 看余量。**这是把"等构建"和"拿证据"解耦的唯一办法。**

公式（Immerkær 1996, "Fast Noise Variance Estimation", CVIU）
--------------------------------
四个独立来源一致（scenesdetect / MOCS-CLAHE 论文 / IAENG / medpy）：

    噪声估计算子   N = [[ 1, -2,  1],
                       [-2,  4, -2],
                       [ 1, -2,  1]]

    σₙ = √(π/2) · 1/(6·(W-2)(H-2)) · Σ |I(x,y) ⊛ N|

    · √(π/2) 来自「用平均绝对偏差代替方差」：高斯下 E|X| = σ·√(2/π)
    · 分母的 6 = √(Σ N²) = √(1+4+1+4+16+4+1+4+1) = √36
      —— 因为对纯噪声 E[(I⊛N)²] = σ²·ΣN² = 36σ²
    · (W-2)(H-2) 是去掉边框的有效像素数

N 是可分离的：N = [1,-2,1] ⊗ [1,-2,1]
  所以两次 3 抽头一维卷积就够，**不需要 3×3 乘法器**。
  更重要的是：它和我们已经跑起来的 3×3 滑窗是同一个窗口，
  只是换了系数 —— 搬到 PL 时不新增行缓存。

用法（板端 / PC 通用）
--------------------------------
    # 1) 单帧估计（最快，先确认能跑）
    python3 -u sigma.py --one

    # 2) 标定：静止场景连拍 30 帧，对比「时域 std」ground truth
    #    对着静止画面（盖上镜头盖更好），跑完不动
    python3 -u sigma.py --calib --frames 30

    # 3) 照度扫描：逐步改变光照，看 σ 怎么变（创新主图的素材）
    python3 -u sigma.py --sweep --steps 6

    # 4) 参数生成预览：σ -> 双阈值 high/low
    python3 -u sigma.py --params
"""
import argparse
import sys
import time

import cv2
import numpy as np

# ------------------------------------------------------------------ 相机
# 与 session.open_camera 保持一致：Linux 下强制 MJPG（实测 30 fps，YUYV 只有 15）


def open_camera(index=0, w=640, h=480, fps=30):
    names = ("CAP_DSHOW", "CAP_MSMF", "CAP_ANY") if sys.platform == "win32" \
        else ("CAP_V4L2", "CAP_ANY")
    for name in names:
        backend = getattr(cv2, name, None)
        if backend is None:
            continue
        cap = cv2.VideoCapture(index, backend)
        if cap.isOpened():
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
            cap.set(cv2.CAP_PROP_FPS, fps)
            if sys.platform != "win32":
                cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
            return cap
        cap.release()
    raise RuntimeError("打不开相机 %s" % index)


# ------------------------------------------------------------ 核心算法
def laplacian(gray):
    """可分离 Laplacian：N = [1,-2,1] ⊗ [1,-2,1]。

    先横向 I(x-1,y) - 2I(x,y) + I(x+1,y)，再纵向同样抽头。
    输出 int16（|值| <= 4*255 = 1020，不会溢出）。
    返回的数组比输入小一圈（去边框），形状 (H-2, W-2)。
    """
    g = gray.astype(np.int16)
    # 横向抽头（沿列方向，x-1 / x / x+1）-> 形状 (H, W-2)
    h = g[:, :-2] - 2 * g[:, 1:-1] + g[:, 2:]
    # 纵向抽头（沿行方向，y-1 / y / y+1）-> 形状 (H-2, W-2)
    lap = h[:-2, :] - 2 * h[1:-1, :] + h[2:, :]
    return lap


def immerkaer(gray):
    """返回 (sigma_est, sum_abs_lap, n_inner)。"""
    lap = laplacian(gray)
    s = int(np.abs(lap).sum(dtype=np.int64))
    h, w = lap.shape
    n_inner = h * w
    sigma = np.sqrt(np.pi / 2.0) * s / (6.0 * n_inner)
    return sigma, s, n_inner


def ground_truth(frames):
    """ground truth：静止场景连拍 N 帧，逐像素时域 std 的均值。

    这是唯一可信的 σ_gt —— Immerkær 是单帧估计，必须和它比。
    流式累加，不在内存里堆 N 帧（N=30 时堆栈要 37 MB）。
    """
    n = 0
    s1 = None
    s2 = None
    for f in frames:
        f32 = f.astype(np.float32)
        if s1 is None:
            s1 = np.zeros_like(f32)
            s2 = np.zeros_like(f32)
        s1 += f32
        s2 += f32 * f32
        n += 1
    mean = s1 / n
    var = s2 / n - mean * mean
    var[var < 0] = 0.0            # 浮点误差可能出现极小负值
    return float(np.sqrt(var).mean())


# ------------------------------------------------------- 参数生成（P0 的②）
# 2026-10-30：原来这里写 K_HIGH=3.0 / K_LOW=0.4，和 hw_driver 的
#   CANNY_K_HI(7.56) / CANNY_K_LO(0.5) **不是同一套数** —— σ=8 时
#   一个给 24/9.6，一个给 128/64，报告里会出现两个互相矛盾的"σ→阈值"规则。
#   现在统一从 hw_driver 取，只有一处定义。
try:
    from hw_driver import CANNY_K_HI as K_HIGH, CANNY_K_LO as K_LOW
except Exception:                      # PC 上没带 hw_driver 时退回同值常量
    K_HIGH, K_LOW = 7.56, 0.5


def thresholds(sigma):
    high = K_HIGH * sigma
    low = K_LOW * high
    return high, low


# ------------------------------------------------------------------ CLI
def grab_gray(cap, mirror=True):
    ok, f = cap.read()
    if not ok or f is None:
        raise RuntimeError("相机没给出帧")
    if mirror:
        f = cv2.flip(f, 1)
    return cv2.cvtColor(f, cv2.COLOR_BGR2GRAY)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--one", action="store_true", help="单帧估计")
    ap.add_argument("--calib", action="store_true", help="静止场景标定")
    ap.add_argument("--sweep", action="store_true", help="照度扫描")
    ap.add_argument("--params", action="store_true", help="打印 sigma->阈值表")
    ap.add_argument("--frames", type=int, default=30)
    ap.add_argument("--steps", type=int, default=6)
    ap.add_argument("--seconds", type=float, default=3.0, help="扫描每档秒数")
    ap.add_argument("--no-mirror", action="store_true")
    args = ap.parse_args()

    if not any([args.one, args.calib, args.sweep, args.params]):
        args.one = True

    print("=" * 72)
    print("Immerkaer 噪声估计（可分离 Laplacian，N=[1,-2,1]⊗[1,-2,1]）")
    print("=" * 72)

    if args.params:
        print("\nsigma -> 双阈值（k_h=%.1f, k_l=%.1f）" % (K_HIGH, K_LOW))
        print("  %-10s %-12s %-12s" % ("sigma", "high", "low"))
        for s in (1, 2, 3, 5, 8, 12, 16, 20, 25, 30):
            hi, lo = thresholds(s)
            print("  %-10d %-12.1f %-12.1f" % (s, hi, lo))
        print("\n  C270 + MJPEG 的典型 sigma 见 --calib 实测；")
        print("  这张表的作用是证明「阈值来自测量，不是人试出来的常数」。")
        if not (args.one or args.calib or args.sweep):
            return

    cap = open_camera()
    print("相机：%dx%d" % (cap.get(cv2.CAP_PROP_FRAME_WIDTH),
                          cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))

    # 预热（前几帧有自动曝光收敛）
    for _ in range(8):
        grab_gray(cap, not args.no_mirror)

    if args.one:
        t0 = time.perf_counter()
        g = grab_gray(cap, not args.no_mirror)
        t_grab = (time.perf_counter() - t0) * 1000
        t0 = time.perf_counter()
        s, ssum, n = immerkaer(g)
        t_est = (time.perf_counter() - t0) * 1000
        print("\n单帧估计：")
        print("  灰度均值 mu   = %.2f" % float(g.mean()))
        print("  sigma_est     = %.3f" % s)
        print("  sum|lap|      = %d   有效像素 = %d" % (ssum, n))
        print("  耗时：采集 %.2f ms / 估计 %.2f ms" % (t_grab, t_est))
        hi, lo = thresholds(s)
        print("  推出双阈值    : high=%.1f  low=%.1f" % (hi, lo))
        cv2.imwrite("sigma_frame.png", g)
        print("  已存 sigma_frame.png")

    if args.calib:
        print("\n" + "-" * 72)
        print("标定：请对着静止画面（建议盖上镜头盖），%d 帧，期间不要动" % args.frames)
        print("-" * 72)
        frames = []
        for i in range(args.frames):
            frames.append(grab_gray(cap, not args.no_mirror))
            if (i + 1) % 5 == 0:
                print("  已采 %d/%d" % (i + 1, args.frames))
        gt = ground_truth(frames)
        ests = [immerkaer(f)[0] for f in frames]
        est = float(np.mean(ests))
        err = abs(est - gt) / gt if gt > 1e-9 else float("nan")
        print("\n标定结果：")
        print("  σ_gt  (时域 std 均值) = %.3f" % gt)
        print("  σ_est (Immerkaer 均值) = %.3f   (逐帧 %.3f ~ %.3f)"
              % (est, min(ests), max(ests)))
        print("  相对误差 = %.1f%%   %s" % (err * 100,
                                          "达标 (<15%)" if err < 0.15 else "未达标"))
        print("\n  逐帧 sigma_est：" + " ".join("%.2f" % e for e in ests))
        print("\n  注意：C270 输出 MJPEG，压缩伪影不是纯高斯噪声，")
        print("  Immerkaer 会有系统性偏差 —— **偏差本身要写进报告**，如实记录即可。")
        # 顺手存一张，报告里用
        cv2.imwrite("sigma_calib_frame.png", frames[-1])
        print("  已存 sigma_calib_frame.png")

    if args.sweep:
        print("\n" + "-" * 72)
        print("照度扫描：共 %d 档，每档 %.1f 秒。" % (args.steps, args.seconds))
        print("每档开始前会等你回车 —— 用来逐步改变光照（手电筒/遮光/关灯）")
        print("-" * 72)
        rows = []
        for i in range(args.steps):
            try:
                input("第 %d/%d 档：调好光照后回车开始 ... " % (i + 1, args.steps))
            except EOFError:
                print("（非交互环境，自动开始）")
            t0 = time.perf_counter()
            n = 0
            last_s = 0.0
            last_mu = 0.0
            while time.perf_counter() - t0 < args.seconds:
                g = grab_gray(cap, not args.no_mirror)
                last_s, _, _ = immerkaer(g)
                last_mu = float(g.mean())
                n += 1
            hi, lo = thresholds(last_s)
            rows.append((i + 1, last_mu, last_s, hi, lo))
            print("  档 %d: mu=%6.2f  sigma=%6.3f  -> high=%6.1f low=%5.1f  (%d 帧)"
                  % (i + 1, last_mu, last_s, hi, lo, n))
        print("\n扫描汇总（这张表就是创新主图的素材）：")
        print("  %-6s %-10s %-10s %-10s %-10s" % ("档", "mu", "sigma", "high", "low"))
        for r in rows:
            print("  %-6d %-10.2f %-10.3f %-10.1f %-10.1f" % r)
        print("\n  报告要讲的：**mu 掉下去时 sigma 涨上来，而阈值跟着 sigma 走，")
        print("  不是跟着人设的常数走。** 固定阈值方案在这一段会崩。")

    cap.release()
    print("\n" + "=" * 72)


if __name__ == "__main__":
    main()
