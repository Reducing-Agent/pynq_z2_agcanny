#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
_pycheck_v81.py —— 5x5 通路的 Python 对拍自检（不下板、不进 Vitis）

目的：本机没有 C++ 编译器，没法本地跑 csim。
这里把 rgb2gray_hls_v8.cpp 的 path5x5() 和 tb_rgb2gray_hls_v8.cpp
的 golden5x5() **各自照抄一份**到 Python，两条独立路径对拍。
两边都严格照抄（包括边界写法、循环缓冲、比较运算符），
所以在 Python 里对上了 => C++ 里大概率也对得上。

这不是"验证过"，只是"提前排掉算法级 bug"。
  真正的 csim / csynth 仍然要在 Vitis GUI 里跑。

用法：
  python _pycheck_v81.py
"""
import sys

MODE_GRAY, MODE_SOBEL, MODE_EMBOSS, MODE_GAUSS = 0, 1, 2, 3
MODE_SHARPEN, MODE_LAPLACE, MODE_BINARY, MODE_MEAN = 4, 5, 6, 7
MODE_DIR = 8
MODE_GAUSS5, MODE_CANNY, MODE_NMS = 9, 10, 11
MODE_MAX_IMPL = MODE_NMS

GAIN_UNITY = 256
CANNY_THRESH_DEFAULT = (60 << 8) | 30
THRESH_LOW = lambda t: t & 0xFF
THRESH_HIGH = lambda t: (t >> 8) & 0xFF


def clamp8(v):
    if v < 0:
        return 0
    if v > 255:
        return 255
    return v


def agc(x, gain):
    t = (x * gain) >> 8
    return 255 if t > 255 else t


def absv(v):
    return -v if v < 0 else v


def sobel_mag(gx, gy):
    return absv(gx) + absv(gy)


# ===========================================================================
# 【DUT 侧】严格照抄 rgb2gray_hls_v8.cpp 的 path5x5()
# ===========================================================================
def SB_GX(g, r, c):
    return (g[r - 1][c + 1] + 2 * g[r][c + 1] + g[r + 1][c + 1]
            - g[r - 1][c - 1] - 2 * g[r][c - 1] - g[r + 1][c - 1])


def SB_GY(g, r, c):
    return (g[r + 1][c - 1] + 2 * g[r + 1][c] + g[r + 1][c + 1]
            - g[r - 1][c - 1] - 2 * g[r - 1][c] - g[r - 1][c + 1])


def dut_path5x5(inp, w, h, op, gain, thresh):
    out = [0xEE] * (w * h)

    if w < 5 or h < 5:
        for i in range(w * h):
            out[i] = 0
        return out

    hi = THRESH_HIGH(thresh)
    lo = THRESH_LOW(thresh)
    if hi < lo:
        hi = lo

    gb = [[0] * w for _ in range(4)]
    g = [[0] * 5 for _ in range(5)]
    st = [[0] * w, [0] * w]

    for x in range(w):
        st[0][x] = 0
        st[1][x] = 0
    for r in range(5):
        for c in range(5):
            g[r][c] = 0
    for y in range(h):
        out[y * w] = 0
        out[y * w + 1] = 0
    for y in range(h):
        out[y * w + w - 2] = 0
        out[y * w + w - 1] = 0
    for x in range(w):
        out[x] = 0
        out[w + x] = 0
    for x in range(w):
        out[(h - 2) * w + x] = 0
        out[(h - 1) * w + x] = 0

    for x in range(w):
        gb[0][x] = agc(inp[x], gain)
    for x in range(w):
        gb[1][x] = agc(inp[w + x], gain)
    for x in range(w):
        gb[2][x] = agc(inp[2 * w + x], gain)
    for x in range(w):
        gb[3][x] = agc(inp[3 * w + x], gain)

    for y in range(4, h):
        base_in = y * w
        base_out = (y - 2) * w
        iw = (y - 2) & 1
        ir = (y - 3) & 1
        for x in range(w):
            g0 = gb[0][x]
            g1 = gb[1][x]
            g2 = gb[2][x]
            g3 = gb[3][x]
            g4 = agc(inp[base_in + x], gain)

            gb[0][x] = g1
            gb[1][x] = g2
            gb[2][x] = g3
            gb[3][x] = g4

            for r in range(5):
                g[r][0] = g[r][1]
                g[r][1] = g[r][2]
                g[r][2] = g[r][3]
                g[r][3] = g[r][4]
            g[0][4] = g0
            g[1][4] = g1
            g[2][4] = g2
            g[3][4] = g3
            g[4][4] = g4

            if x >= 4:
                xo = x - 2
                if op == MODE_GAUSS5:
                    rs = [0] * 5
                    for i in range(5):
                        rs[i] = ((g[i][0] + g[i][4])
                                 + 4 * (g[i][1] + g[i][3])
                                 + 6 * g[i][2])
                    v = (rs[0] + rs[4]) + 4 * (rs[1] + rs[3]) + 6 * rs[2]
                    out[base_out + xo] = (v >> 8) & 0xFF
                else:
                    gxC = SB_GX(g, 2, 2)
                    gyC = SB_GY(g, 2, 2)
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
                        n1 = sobel_mag(SB_GX(g, 2, 1), SB_GY(g, 2, 1))
                        n2 = sobel_mag(SB_GX(g, 2, 3), SB_GY(g, 2, 3))
                    elif d == 2:
                        n1 = sobel_mag(SB_GX(g, 1, 2), SB_GY(g, 1, 2))
                        n2 = sobel_mag(SB_GX(g, 3, 2), SB_GY(g, 3, 2))
                    elif d == 1:
                        n1 = sobel_mag(SB_GX(g, 1, 1), SB_GY(g, 1, 1))
                        n2 = sobel_mag(SB_GX(g, 3, 3), SB_GY(g, 3, 3))
                    else:
                        n1 = sobel_mag(SB_GX(g, 1, 3), SB_GY(g, 1, 3))
                        n2 = sobel_mag(SB_GX(g, 3, 1), SB_GY(g, 3, 1))

                    m = magC if (magC >= n1 and magC >= n2) else 0

                    if op == MODE_NMS:
                        out[base_out + xo] = clamp8(m)
                    else:
                        v = 0
                        strong = 0
                        if m >= hi:
                            v = 255
                            strong = 1
                        elif m >= lo:
                            nb = 0
                            if xo > 0:
                                nb |= st[ir][xo - 1]
                            nb |= st[ir][xo]
                            if xo + 1 < w:
                                nb |= st[ir][xo + 1]
                            if xo >= 3:
                                nb |= st[iw][xo - 1]
                            if nb:
                                v = 255
                                strong = 1
                        st[iw][xo] = strong
                        out[base_out + xo] = v
    return out


# ===========================================================================
# 【GOLDEN 侧】严格照抄 tb_rgb2gray_hls_v8.cpp 的 golden5x5()
# ===========================================================================
B5 = [1, 4, 6, 4, 1]


def gsobel_mag(p, r, c):
    gx = (p[r - 1][c + 1] + 2 * p[r][c + 1] + p[r + 1][c + 1]
          - p[r - 1][c - 1] - 2 * p[r][c - 1] - p[r + 1][c - 1])
    gy = (p[r + 1][c - 1] + 2 * p[r + 1][c] + p[r + 1][c + 1]
          - p[r - 1][c - 1] - 2 * p[r - 1][c] - p[r - 1][c + 1])
    return absv(gx) + absv(gy)


def golden5x5(inp, w, h, op, gain, thresh):
    out = [0] * (w * h)
    if w < 5 or h < 5:
        return out

    hi = THRESH_HIGH(thresh)
    lo = THRESH_LOW(thresh)
    if hi < lo:
        hi = lo

    mag = [0] * (w * h)
    strong = [0] * (w * h)

    for y in range(2, h - 2):
        for x in range(2, w - 2):
            p = [[0] * 5 for _ in range(5)]
            for dy in range(-2, 3):
                for dx in range(-2, 3):
                    p[dy + 2][dx + 2] = agc(inp[(y + dy) * w + (x + dx)], gain)

            if op == MODE_GAUSS5:
                s = 0
                for dy in range(5):
                    for dx in range(5):
                        s += B5[dy] * B5[dx] * p[dy][dx]
                out[y * w + x] = (s >> 8) & 0xFF
                continue

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
                n1 = gsobel_mag(p, 2, 1)
                n2 = gsobel_mag(p, 2, 3)
            elif d == 2:
                n1 = gsobel_mag(p, 1, 2)
                n2 = gsobel_mag(p, 3, 2)
            elif d == 1:
                n1 = gsobel_mag(p, 1, 1)
                n2 = gsobel_mag(p, 3, 3)
            else:
                n1 = gsobel_mag(p, 1, 3)
                n2 = gsobel_mag(p, 3, 1)

            mag[y * w + x] = mC if (mC >= n1 and mC >= n2) else 0

    if op == MODE_GAUSS5:
        return out

    for y in range(2, h - 2):
        for x in range(2, w - 2):
            m = mag[y * w + x]
            if m >= hi:
                out[y * w + x] = 255
                strong[y * w + x] = 1
            elif m >= lo:
                nb = 0
                if x >= 3:
                    nb |= strong[(y - 1) * w + (x - 1)]
                nb |= strong[(y - 1) * w + x]
                if x + 1 <= w - 3:
                    nb |= strong[(y - 1) * w + (x + 1)]
                if x >= 3:
                    nb |= strong[y * w + (x - 1)]
                if nb:
                    out[y * w + x] = 255
                    strong[y * w + x] = 1
            if op == MODE_NMS:
                out[y * w + x] = 255 if m > 255 else m
    return out


# ===========================================================================
# 对拍
# ===========================================================================
def make_src(n=96):
    src = []
    st = 12345
    for _ in range(n):
        st = (st * 1103515245 + 12345) & 0xFFFFFFFF
        src.append((st >> 16) & 0xFF)
    return src


def main():
    src = make_src()
    sizes = [(32, 24, "小图"), (64, 48, "中图"), (33, 17, "奇数尺寸"),
             (5, 5, "5x5 最小"), (6, 5, "6x5 窄图"), (3, 3, "3x3 兜底")]
    if "--vga" in sys.argv:
        sizes.append((640, 480, "VGA 满幅"))

    gains = [GAIN_UNITY, 512, 1000]
    threshes = [CANNY_THRESH_DEFAULT, (100 << 8) | 50, (20 << 8) | 10]

    ncase = 0
    nfail = 0
    for (w, h, tag) in sizes:
        inp = [src[i % len(src)] for i in range(w * h)]
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
                        y, x = i // w, i % w
                        print("  [FAIL] %-10s op=%2d gain=%4d th=%04x %dx%d  "
                              "%d/%d 处不同，首差 (%d,%d) dut=%d ref=%d"
                              % (tag, op, gain, th, w, h, len(bad), w * h,
                                 y, x, d[i], r[i]))

    # 附加检查 1：外两圈必须恒 0
    extra = 0
    for (w, h, tag) in [(32, 24, "小图"), (33, 17, "奇数尺寸")]:
        inp = [src[i % len(src)] for i in range(w * h)]
        for op in (MODE_GAUSS5, MODE_CANNY, MODE_NMS):
            d = dut_path5x5(inp, w, h, op, GAIN_UNITY, CANNY_THRESH_DEFAULT)
            nz = 0
            for y in range(h):
                for x in range(w):
                    if y < 2 or y > h - 3 or x < 2 or x > w - 3:
                        if d[y * w + x] != 0:
                            nz += 1
            if nz:
                extra += 1
                print("  [FAIL] 外两圈非0: %s op=%d，%d 个像素非 0" % (tag, op, nz))

    # 附加检查 2：GAUSS5 必须是真的平滑（平坦区幂等 + 噪声图 std 下降）
    import statistics
    inp = [src[i % len(src)] for i in range(32 * 24)]
    g5 = dut_path5x5(inp, 32, 24, MODE_GAUSS5, GAIN_UNITY, CANNY_THRESH_DEFAULT)
    inner_in = [inp[y * 32 + x] for y in range(4, 20) for x in range(4, 28)]
    inner_g5 = [g5[y * 32 + x] for y in range(4, 20) for x in range(4, 28)]
    s_in = statistics.pstdev(inner_in)
    s_g5 = statistics.pstdev(inner_g5)
    print("\n  [检查] GAUSS5 平滑：内部区 std %.2f -> %.2f  %s"
          % (s_in, s_g5, "OK(下降)" if s_g5 < s_in else "!! 没平滑"))
    if s_g5 >= s_in:
        extra += 1

    # 附加检查 3：平坦图 GAUSS5 必须幂等（常数图 -> 原值）
    flat = [77] * (32 * 24)
    g5f = dut_path5x5(flat, 32, 24, MODE_GAUSS5, GAIN_UNITY, CANNY_THRESH_DEFAULT)
    # 内部区应该还是 77（Σ系数=256）
    mid = g5f[10 * 32 + 16]
    print("  [检查] 平坦图 GAUSS5 幂等：中心 %d %s"
          % (mid, "OK(=77)" if mid == 77 else "!! 不等于 77"))
    if mid != 77:
        extra += 1

    # 附加检查 4：CANNY 输出必须只有 0/255
    for (w, h, tag) in [(32, 24, "小图")]:
        inp = [src[i % len(src)] for i in range(w * h)]
        c = dut_path5x5(inp, w, h, MODE_CANNY, GAIN_UNITY, CANNY_THRESH_DEFAULT)
        vals = set(c)
        ok = vals <= {0, 255}
        print("  [检查] CANNY 输出值域 %s  %s"
              % (sorted(vals), "OK(仅 0/255)" if ok else "!! 有其它值"))
        if not ok:
            extra += 1

    print("\n======================================================================")
    print("5x5 通路对拍：%d 例，失败 %d 例；附加检查失败 %d 项 -> %s"
          % (ncase, nfail, extra, "全部 PASS" if (nfail == 0 and extra == 0) else "有问题"))
    print("======================================================================")
    return 1 if (nfail or extra) else 0


if __name__ == "__main__":
    sys.exit(main())
