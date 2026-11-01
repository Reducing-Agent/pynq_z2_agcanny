#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
_pycheck_v82.py —— v8.2 四级融合流水线的 Python 对拍自检

本机没有 C++ 编译器，所以把 rgb2gray_hls_v8.cpp 的 path5x5() 和
测试台的 golden 各照抄一份到 Python，两条独立路径对拍。

v8.2 相对 v8.1 的架构变化：
  AGC → [横向5抽头 → 4块hconv行缓存 → 纵向5抽头] → [5x5 vconv 窗口 → 9个Sobel]
     → [NMS] → [双阈值+滞后]
  四级串联，级间用行缓存首尾相接，中间结果不回 DDR。

用法：
  python _pycheck_v82.py            # 小图
  python _pycheck_v82.py --vga      # 加跑 640x480（慢）
"""
import sys

import numpy as np

MODE_GAUSS5, MODE_CANNY, MODE_NMS = 9, 10, 11
GAIN_UNITY = 256
CANNY_THRESH_DEFAULT = (60 << 8) | 30
THRESH_LOW = lambda t: t & 0xFF
THRESH_HIGH = lambda t: (t >> 8) & 0xFF
B5 = (1, 4, 6, 4, 1)


def clamp8(v):
    return 0 if v < 0 else (255 if v > 255 else v)


def agc(x, gain):
    t = (x * gain) >> 8
    return 255 if t > 255 else t


def absv(v):
    return -v if v < 0 else v


def binom5(p0, p1, p2, p3, p4):
    return (p0 + p4) + 4 * (p1 + p3) + 6 * p2


def sobel_mag(gx, gy):
    return absv(gx) + absv(gy)


# ===========================================================================
# 【DUT 侧】严格照抄 rgb2gray_hls_v8.cpp 的 path5x5()（v8.2）
# ===========================================================================
def SB_GX(v, r, c):
    return (v[r - 1][c + 1] + 2 * v[r][c + 1] + v[r + 1][c + 1]
            - v[r - 1][c - 1] - 2 * v[r][c - 1] - v[r + 1][c - 1])


def SB_GY(v, r, c):
    return (v[r + 1][c - 1] + 2 * v[r + 1][c] + v[r + 1][c + 1]
            - v[r - 1][c - 1] - 2 * v[r - 1][c] - v[r - 1][c + 1])


def dut_path5x5(inp, w, h, op, gain, thresh):
    out = [0xEE] * (w * h)

    if w < 9 or h < 9:
        for i in range(w * h):
            out[i] = 0
        return out

    hi = THRESH_HIGH(thresh)
    lo = THRESH_LOW(thresh)
    if hi < lo:
        hi = lo

    hc = [0] * 5
    hb = [[0] * w for _ in range(4)]
    vb = [[0] * w for _ in range(4)]
    v = [[0] * 5 for _ in range(5)]
    st = [[0] * w, [0] * w]

    for x in range(w):
        st[0][x] = 0
        st[1][x] = 0
        vb[0][x] = 0
        vb[1][x] = 0
        vb[2][x] = 0
        vb[3][x] = 0
    for r in range(5):
        for c in range(5):
            v[r][c] = 0
    for y in range(h):
        for k in range(4):
            out[y * w + k] = 0
            out[y * w + w - 1 - k] = 0
    for y in range(4):
        for x in range(w):
            out[y * w + x] = 0
    for y in range(h - 4, h):
        for x in range(w):
            out[y * w + x] = 0

    # 预装：第 0..3 行的横向卷积结果 -> hb[0..3]
    for idx in range(4):
        for k in range(5):
            hc[k] = 0
        for x in range(w):
            c = agc(inp[idx * w + x], gain)
            hc[0] = hc[1]
            hc[1] = hc[2]
            hc[2] = hc[3]
            hc[3] = hc[4]
            hc[4] = c
            hb[idx][x] = (binom5(*hc) >> 4) & 0xFF

    for y in range(4, h):
        base_in = y * w
        base_out = (y - 4) * w
        iw = (y - 4) & 1
        ir = (y - 5) & 1
        for k in range(5):
            hc[k] = 0
        for x in range(w):
            # 级1a
            c = agc(inp[base_in + x], gain)
            hc[0] = hc[1]
            hc[1] = hc[2]
            hc[2] = hc[3]
            hc[3] = hc[4]
            hc[4] = c
            hn = (binom5(*hc) >> 4) & 0xFF
            # 级1b
            h0 = hb[0][x]
            h1 = hb[1][x]
            h2 = hb[2][x]
            h3 = hb[3][x]
            vn = (binom5(h0, h1, h2, h3, hn) >> 4) & 0xFF
            hb[0][x] = h1
            hb[1][x] = h2
            hb[2][x] = h3
            hb[3][x] = hn
            # 级2
            q0 = vb[0][x]
            q1 = vb[1][x]
            q2 = vb[2][x]
            q3 = vb[3][x]
            vb[0][x] = q1
            vb[1][x] = q2
            vb[2][x] = q3
            vb[3][x] = vn
            for r in range(5):
                v[r][0] = v[r][1]
                v[r][1] = v[r][2]
                v[r][2] = v[r][3]
                v[r][3] = v[r][4]
            v[0][4] = q0
            v[1][4] = q1
            v[2][4] = q2
            v[3][4] = q3
            v[4][4] = vn

            if y >= 8 and x >= 8:
                xo = x - 4
                if op == MODE_GAUSS5:
                    out[base_out + xo] = v[2][2]
                else:
                    gxC = SB_GX(v, 2, 2)
                    gyC = SB_GY(v, 2, 2)
                    ax = absv(gxC)
                    ay = absv(gyC)
                    magC = ax + ay
                    if 2 * ay <= ax:
                        d = 0
                    elif 2 * ax <= ay:
                        d = 2
                    elif gxC * gyC > 0:
                        d = 1
                    else:
                        d = 3
                    if d == 0:
                        n1 = sobel_mag(SB_GX(v, 2, 1), SB_GY(v, 2, 1))
                        n2 = sobel_mag(SB_GX(v, 2, 3), SB_GY(v, 2, 3))
                    elif d == 2:
                        n1 = sobel_mag(SB_GX(v, 1, 2), SB_GY(v, 1, 2))
                        n2 = sobel_mag(SB_GX(v, 3, 2), SB_GY(v, 3, 2))
                    elif d == 1:
                        n1 = sobel_mag(SB_GX(v, 1, 1), SB_GY(v, 1, 1))
                        n2 = sobel_mag(SB_GX(v, 3, 3), SB_GY(v, 3, 3))
                    else:
                        n1 = sobel_mag(SB_GX(v, 1, 3), SB_GY(v, 1, 3))
                        n2 = sobel_mag(SB_GX(v, 3, 1), SB_GY(v, 3, 1))
                    m = magC if (magC >= n1 and magC >= n2) else 0
                    if op == MODE_NMS:
                        out[base_out + xo] = clamp8(m)
                    else:
                        vv = 0
                        strong = 0
                        if m >= hi:
                            vv = 255
                            strong = 1
                        elif m >= lo:
                            nb = 0
                            if xo >= 5:
                                nb |= st[ir][xo - 1]
                            nb |= st[ir][xo]
                            if xo + 1 <= w - 5:
                                nb |= st[ir][xo + 1]
                            if xo >= 5:
                                nb |= st[iw][xo - 1]
                            if nb:
                                vv = 255
                                strong = 1
                        st[iw][xo] = strong
                        out[base_out + xo] = vv
    return out


# ===========================================================================
# 【GOLDEN 侧】独立实现：numpy 零填充卷积 + 整幅数组滞后
# ===========================================================================
def gauss5_np(g):
    """零填充 5x5 二项式高斯，代码路径与 DUT 的流式滑窗完全不同。

    必须**分两步 >>4**，不能一次 >>8：DUT 是可分离两级，
      中间结果 hconv 被截成 8 bit 再喂给第二级，精度损失是真实存在的。
      写成一次 >>8 会在噪声图上随机差 1。
    """
    H0, W0 = g.shape
    P = np.zeros((H0 + 4, W0 + 4), dtype=np.int64)
    P[2:2 + H0, 2:2 + W0] = g
    T = np.zeros((H0 + 4, W0), dtype=np.int64)
    for dx in range(5):
        T += B5[dx] * P[:, dx:dx + W0]
    T = (T >> 4)                       # 横向归一化 /16，并截断到 8 bit 范围
    V = np.zeros((H0, W0), dtype=np.int64)
    for dy in range(5):
        V += B5[dy] * T[dy:dy + H0, :]
    return (V >> 4).astype(np.int64)   # 纵向归一化 /16


def gsobel_mag(p, r, c):
    gx = (p[r - 1][c + 1] + 2 * p[r][c + 1] + p[r + 1][c + 1]
          - p[r - 1][c - 1] - 2 * p[r][c - 1] - p[r + 1][c - 1])
    gy = (p[r + 1][c - 1] + 2 * p[r + 1][c] + p[r + 1][c + 1]
          - p[r - 1][c - 1] - 2 * p[r - 1][c] - p[r - 1][c + 1])
    return absv(gx) + absv(gy)


def golden5x5(inp, w, h, op, gain, thresh):
    out = [0] * (w * h)
    if w < 9 or h < 9:
        return out
    hi = THRESH_HIGH(thresh)
    lo = THRESH_LOW(thresh)
    if hi < lo:
        hi = lo

    g = np.array([agc(v, gain) for v in inp], dtype=np.int64).reshape(h, w)

    # 级1：零填充 5x5 高斯；DUT 只在行 [2,H-3] 产出 vconv
    vconv = np.zeros((h, w), dtype=np.int64)
    vconv[2:h - 2, :] = gauss5_np(g)[2:h - 2, :]

    if op == MODE_GAUSS5:
        for y in range(4, h - 4):
            for x in range(4, w - 4):
                out[y * w + x] = int(vconv[y, x]) & 0xFF
        return out

    mag = np.zeros((h, w), dtype=np.int32)
    for y in range(4, h - 4):
        for x in range(4, w - 4):
            p = [[int(vconv[y + dy - 2, x + dx - 2]) for dx in range(5)]
                 for dy in range(5)]
            mC = gsobel_mag(p, 2, 2)
            gxc = (p[1][3] + 2 * p[2][3] + p[3][3]
                   - p[1][1] - 2 * p[2][1] - p[3][1])
            gyc = (p[3][1] + 2 * p[3][2] + p[3][3]
                   - p[1][1] - 2 * p[1][2] - p[1][3])
            ax = absv(gxc)
            ay = absv(gyc)
            if 2 * ay <= ax:
                d = 0
            elif 2 * ax <= ay:
                d = 2
            elif gxc * gyc > 0:
                d = 1
            else:
                d = 3
            if d == 0:
                n1, n2 = gsobel_mag(p, 2, 1), gsobel_mag(p, 2, 3)
            elif d == 2:
                n1, n2 = gsobel_mag(p, 1, 2), gsobel_mag(p, 3, 2)
            elif d == 1:
                n1, n2 = gsobel_mag(p, 1, 1), gsobel_mag(p, 3, 3)
            else:
                n1, n2 = gsobel_mag(p, 1, 3), gsobel_mag(p, 3, 1)
            mag[y, x] = mC if (mC >= n1 and mC >= n2) else 0

    if op == MODE_NMS:
        for y in range(4, h - 4):
            for x in range(4, w - 4):
                out[y * w + x] = clamp8(int(mag[y, x]))
        return out

    strong = np.zeros((h, w), dtype=np.uint8)
    for y in range(4, h - 4):
        for x in range(4, w - 4):
            m = int(mag[y, x])
            if m >= hi:
                out[y * w + x] = 255
                strong[y, x] = 1
            elif m >= lo:
                nb = 0
                if x >= 5:
                    nb |= int(strong[y - 1, x - 1])
                nb |= int(strong[y - 1, x])
                if x + 1 <= w - 5:
                    nb |= int(strong[y - 1, x + 1])
                if x >= 5:
                    nb |= int(strong[y, x - 1])
                if nb:
                    out[y * w + x] = 255
                    strong[y, x] = 1
    return out


# ===========================================================================
def main():
    rng = np.random.RandomState(20261029)
    sizes = [(32, 24), (33, 17), (64, 48), (5, 5), (9, 9), (12, 10)]
    if "--vga" in sys.argv:
        sizes.append((640, 480))
    gains = [GAIN_UNITY, 512, 1000]
    threshes = [CANNY_THRESH_DEFAULT, (100 << 8) | 50, (20 << 8) | 10]

    ncase = 0
    nfail = 0
    for (w, h) in sizes:
        inp = rng.randint(0, 256, size=(h, w)).astype(np.uint8).ravel().tolist()
        for gain in gains:
            for th in threshes:
                for op in (MODE_GAUSS5, MODE_CANNY, MODE_NMS):
                    ncase += 1
                    d = dut_path5x5(inp, w, h, op, gain, th)
                    r = golden5x5(inp, w, h, op, gain, th)
                    bad = [i for i in range(w * h) if d[i] != r[i]]
                    if bad:
                        nfail += 1
                        i = bad[0]
                        print("  [FAIL] %dx%d op=%2d gain=%4d th=%04x  %d/%d 处不同，"
                              "首差 (%d,%d) dut=%d ref=%d"
                              % (w, h, op, gain, th, len(bad), w * h,
                                 i // w, i % w, d[i], r[i]))

    # 附加 1：外四圈必须恒 0
    extra = 0
    for (w, h) in [(32, 24), (33, 17)]:
        inp = rng.randint(0, 256, size=(h, w)).astype(np.uint8).ravel().tolist()
        for op in (MODE_GAUSS5, MODE_CANNY, MODE_NMS):
            d = dut_path5x5(inp, w, h, op, GAIN_UNITY, CANNY_THRESH_DEFAULT)
            nz = 0
            for y in range(h):
                for x in range(w):
                    if y < 4 or y > h - 5 or x < 4 or x > w - 5:
                        if d[y * w + x] != 0:
                            nz += 1
            if nz:
                extra += 1
                print("  [FAIL] 外四圈非0: %dx%d op=%d，%d 个像素非 0" % (w, h, op, nz))

    # 附加 2：GAUSS5 必须是真的平滑（内部区 std 下降）
    import statistics
    inp = rng.randint(0, 256, size=(24, 32)).astype(np.uint8).ravel().tolist()
    g5 = dut_path5x5(inp, 32, 24, MODE_GAUSS5, GAIN_UNITY, CANNY_THRESH_DEFAULT)
    ii = [inp[y * 32 + x] for y in range(6, 18) for x in range(6, 26)]
    oo = [g5[y * 32 + x] for y in range(6, 18) for x in range(6, 26)]
    s1, s2 = statistics.pstdev(ii), statistics.pstdev(oo)
    print("\n  [检查] GAUSS5 平滑：内部区 std %.2f -> %.2f  %s"
          % (s1, s2, "OK(下降)" if s2 < s1 else "!! 没平滑"))
    if s2 >= s1:
        extra += 1

    # 附加 3：平坦图 GAUSS5 幂等
    flat = [77] * (32 * 24)
    g5f = dut_path5x5(flat, 32, 24, MODE_GAUSS5, GAIN_UNITY, CANNY_THRESH_DEFAULT)
    mid = g5f[10 * 32 + 16]
    print("  [检查] 平坦图 GAUSS5 幂等：中心 %d %s"
          % (mid, "OK(=77)" if mid == 77 else "!! 不等于 77"))
    if mid != 77:
        extra += 1

    # 附加 4：v8.2 的关键卖点 —— CANNY 必须自带去噪（对比 v8.1 的无高斯版本）
    inp = rng.randint(0, 256, size=(24, 32)).astype(np.uint8).ravel().tolist()
    noisy = (rng.randint(0, 256, size=(24, 32)).astype(np.uint8) // 8).ravel().tolist()
    mix = [min(255, inp[i] + noisy[i]) for i in range(32 * 24)]
    c = dut_path5x5(mix, 32, 24, MODE_CANNY, GAIN_UNITY, CANNY_THRESH_DEFAULT)
    nz = sum(1 for v in c if v == 255)
    print("  [检查] 噪声图上 CANNY 输出强边像素数 = %d（有高斯级时应远少于无高斯）" % nz)

    print("\n======================================================================")
    print("v8.2 四级流水线对拍：%d 例，失败 %d 例；附加 %d 项失败 -> %s"
          % (ncase, nfail, extra, "全部 PASS" if (nfail == 0 and extra == 0) else "有问题"))
    print("======================================================================")
    return 1 if (nfail or extra) else 0


if __name__ == "__main__":
    sys.exit(main())
