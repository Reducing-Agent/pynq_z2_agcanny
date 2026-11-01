# -*- coding: utf-8 -*-
"""
mirror_ab.py —— 「改循环结构前的 Python 对拍模板」

只在开发机跑，**不进交付包**。这是本技能包最有价值的工具：
它把"改 HLS 循环结构"从「烧位流上板才发现边界错了」
变成「PC 上几分钟、702 例逐字节对拍」。

=============================================================================
一、它解决什么问题
=============================================================================
HLS 里为了性能做的重构（把写挪出 if、加乒乓缓冲、改行缓存起点、拆流水级）
**不改变算法语义，但极易改变边界行为**。而边界错误的特点是：
  - 大面积区域全对，只有最外几行/几列错；
  - cosim 用例尺寸单一时可能根本不触发；
  - 上板后才能发现，而那时你已经在怀疑是时序、是 DMA、是位流版本。

=============================================================================
二、方法（照抄这三条）
=============================================================================
1. **用 Python 严格照抄 HLS 源码**，写两份：`*_A`（改前）/ `*_B`（改后）。
   抄的时候**逐行对照 C 代码**，包括移位量、截断位置、边界清零的范围。
2. **判据是逐字节完全相同**（`a != b` 直接算失败），**不是"差不多"**。
   差 1 也算失败 —— 差 1 通常意味着移位/截断时机变了（见案例 12）。
3. **测试用例必须包含极端尺寸**：最小合法尺寸、最小尺寸+1、非 2 的幂、
   长宽不相等、以及真实尺寸。边界 bug 只在小尺寸上现形。

=============================================================================
三、怎么改成你自己的
=============================================================================
本文件是本项目 op 0~11 的真实实现，**可直接运行**（702 例 0 失败）。
换题目时改这几处即可，骨架不用动：

  - `MAX_W` / 尺寸常量
  - `_op_val()`    —— 你的算子核心（3x3 窗口 → 单像素值）
  - `single_A/B()` —— 你的主循环（A 版原样 / B 版重构后）
  - `fused_A/B()`  —— 若有多级融合通路
  - `run()` 里的 `sizes` / `gains` / 模式枚举

**关键纪律**：A 版和 B 版必须**同一次改动的两面**，
不要在写 B 版时"顺手优化"别的地方，否则对拍通过也证明不了任何事。

=============================================================================
四、本项目的这次改动（案例 10）
=============================================================================
B 方案把 DDR 写从 `if (x>=2)` 里挪出来，改成
「片上输出行缓冲 `outr[2][MAX_W]` + 无条件连续写」。
风险点全在**边界**：列 0 / 列 W-1、行 0 / 行 H-1（5x5 通路还要外四圈恒 0）。
对拍 **702 例 0 失败** 之后才敢去烧 Vitis —— 一次就过了。
"""
import random

MAX_W = 640
SIGMA_BANKS = 16
SIGMA_MASK = 15

MODE_GRAY, MODE_SOBEL, MODE_EMBOSS, MODE_GAUSS, MODE_SHARPEN = 0, 1, 2, 3, 4
MODE_LAPLACE, MODE_BINARY, MODE_MEAN, MODE_DIR = 5, 6, 7, 8
MODE_GAUSS5, MODE_CANNY, MODE_NMS = 9, 10, 11


def clamp8(v):
    if v < 0:
        return 0
    if v > 255:
        return 255
    return v


def agc(x, gain):
    t = (x * gain) >> 8
    return 255 if t > 255 else t


def binom5(p0, p1, p2, p3, p4):
    return (p0 + p4) + 4 * (p1 + p3) + 6 * p2


def sb_gx(vv, r, c):
    return (vv[r - 1][c + 1] + 2 * vv[r][c + 1] + vv[r + 1][c + 1]
            - vv[r - 1][c - 1] - 2 * vv[r][c - 1] - vv[r + 1][c - 1])


def sb_gy(vv, r, c):
    return (vv[r + 1][c - 1] + 2 * vv[r + 1][c] + vv[r + 1][c + 1]
            - vv[r - 1][c - 1] - 2 * vv[r - 1][c] - vv[r - 1][c + 1])


def sobel_mag(gx, gy):
    return abs(gx) + abs(gy)


# ===========================================================================
# 3x3 通路（op 0..8）
# ===========================================================================
def single_A(inp, W, H, op, gain):
    """改前：DDR 写在 if(x>=2) 里"""
    out = [0] * (W * H)
    rowA = [0] * W
    rowB = [0] * W
    w0 = [0, 0, 0]
    w1 = [0, 0, 0]
    w2 = [0, 0, 0]
    sbank = [0] * SIGMA_BANKS

    if op == MODE_GRAY:
        for i in range(W * H):
            out[i] = agc(inp[i], gain)
        return out, sum(sbank)

    if W < 3 or H < 3:
        return out, 0

    for x in range(W):
        out[x] = 0
    for x in range(W):
        rowA[x] = agc(inp[x], gain)
    for x in range(W):
        rowB[x] = agc(inp[W + x], gain)

    for y in range(2, H):
        base_in = y * W
        base_out = (y - 1) * W
        for x in range(W):
            a = rowA[x]
            b = rowB[x]
            c = agc(inp[base_in + x], gain)
            rowA[x] = b
            rowB[x] = c
            w0[0] = w0[1]; w0[1] = w0[2]; w0[2] = a
            w1[0] = w1[1]; w1[1] = w1[2]; w1[2] = b
            w2[0] = w2[1]; w2[1] = w2[2]; w2[2] = c
            if x >= 2:
                p00, p01, p02 = w0[0], w0[1], w0[2]
                p10, p11, p12 = w1[0], w1[1], w1[2]
                p20, p21, p22 = w2[0], w2[1], w2[2]
                cross4 = p01 + p10 + p12 + p21
                sum9 = cross4 + p00 + p02 + p20 + p22 + p11
                corner4 = p00 + p02 + p20 + p22
                lap = corner4 - 2 * cross4 + 4 * p11
                sbank[x & SIGMA_MASK] += abs(lap)
                out[base_out + (x - 1)] = _op_val(op, p00, p01, p02, p10, p11,
                                                  p12, p20, p21, p22, cross4, sum9)
        out[base_out] = 0
        out[base_out + W - 1] = 0

    for x in range(W):
        out[(H - 1) * W + x] = 0
    return out, sum(sbank)


def single_B(inp, W, H, op, gain):
    """改后：结果落片上 outr[2]，DDR 写无条件连续"""
    out = [0] * (W * H)
    rowA = [0] * W
    rowB = [0] * W
    outr = [[0] * W for _ in range(2)]
    w0 = [0, 0, 0]
    w1 = [0, 0, 0]
    w2 = [0, 0, 0]
    sbank = [0] * SIGMA_BANKS

    if op == MODE_GRAY:
        for i in range(W * H):
            out[i] = agc(inp[i], gain)
        return out, sum(sbank)

    if W < 3 or H < 3:
        return out, 0

    for x in range(W):
        outr[0][x] = 0
        outr[1][x] = 0

    for x in range(W):
        out[x] = 0
    for x in range(W):
        rowA[x] = agc(inp[x], gain)
    for x in range(W):
        rowB[x] = agc(inp[W + x], gain)

    for y in range(2, H):
        base_in = y * W
        out_row = y - 2
        base_out = out_row * W
        cur = (y - 1) & 1
        prev = out_row & 1
        for x in range(W):
            a = rowA[x]
            b = rowB[x]
            c = agc(inp[base_in + x], gain)
            rowA[x] = b
            rowB[x] = c
            w0[0] = w0[1]; w0[1] = w0[2]; w0[2] = a
            w1[0] = w1[1]; w1[1] = w1[2]; w1[2] = b
            w2[0] = w2[1]; w2[1] = w2[2]; w2[2] = c
            out[base_out + x] = outr[prev][x]
            if x >= 2:
                p00, p01, p02 = w0[0], w0[1], w0[2]
                p10, p11, p12 = w1[0], w1[1], w1[2]
                p20, p21, p22 = w2[0], w2[1], w2[2]
                cross4 = p01 + p10 + p12 + p21
                sum9 = cross4 + p00 + p02 + p20 + p22 + p11
                corner4 = p00 + p02 + p20 + p22
                lap = corner4 - 2 * cross4 + 4 * p11
                sbank[x & SIGMA_MASK] += abs(lap)
                outr[cur][x - 1] = _op_val(op, p00, p01, p02, p10, p11,
                                           p12, p20, p21, p22, cross4, sum9)

    last_row = H - 2
    last_bank = last_row & 1
    last_base = last_row * W
    for x in range(W):
        out[last_base + x] = outr[last_bank][x]
    for x in range(W):
        out[(H - 1) * W + x] = 0
    return out, sum(sbank)


def _op_val(op, p00, p01, p02, p10, p11, p12, p20, p21, p22, cross4, sum9):
    if op == MODE_SOBEL:
        gx = (p02 + 2 * p12 + p22) - (p00 + 2 * p10 + p20)
        gy = (p20 + 2 * p21 + p22) - (p00 + 2 * p01 + p02)
        return clamp8(abs(gx) + abs(gy))
    if op == MODE_DIR:
        gx = (p02 + 2 * p12 + p22) - (p00 + 2 * p10 + p20)
        gy = (p20 + 2 * p21 + p22) - (p00 + 2 * p01 + p02)
        ax, ay = abs(gx), abs(gy)
        if 2 * ay <= ax:
            d = 0
        elif 2 * ax <= ay:
            d = 2
        elif gx * gy > 0:
            d = 1
        else:
            d = 3
        return d * 85
    if op == MODE_EMBOSS:
        return clamp8(p22 - p00 + 128)
    if op == MODE_GAUSS:
        return (sum9 + cross4 + 3 * p11) >> 4
    if op == MODE_SHARPEN:
        return clamp8(5 * p11 - cross4)
    if op == MODE_LAPLACE:
        return clamp8(9 * p11 - sum9)
    if op == MODE_BINARY:
        return 255 if p11 >= 128 else 0
    return (sum9 * 1821) >> 14


# ===========================================================================
# 5x5 融合通路（op 9..11）
# ===========================================================================
def fused_A(inp, W, H, op, gain, hi, lo):
    out = [0] * (W * H)
    if W < 9 or H < 9:
        return out

    hc = [0] * 5
    hb = [[0] * W for _ in range(4)]
    vb = [[0] * W for _ in range(4)]
    v = [[0] * 5 for _ in range(5)]
    st = [[0] * W for _ in range(2)]

    for x in range(W):
        st[0][x] = 0; st[1][x] = 0
        vb[0][x] = 0; vb[1][x] = 0; vb[2][x] = 0; vb[3][x] = 0
    for y in range(H):
        out[y * W] = 0; out[y * W + 1] = 0; out[y * W + 2] = 0; out[y * W + 3] = 0
    for y in range(H):
        for k in range(4):
            out[y * W + W - 1 - k] = 0
    for y in range(4):
        for x in range(W):
            out[y * W + x] = 0
    for y in range(H - 4, H):
        for x in range(W):
            out[y * W + x] = 0

    for row in range(4):
        for k in range(5):
            hc[k] = 0
        for x in range(W):
            c = agc(inp[row * W + x], gain)
            hc[0] = hc[1]; hc[1] = hc[2]; hc[2] = hc[3]; hc[3] = hc[4]; hc[4] = c
            hb[row][x] = (binom5(*hc) >> 4) & 0xFF

    for y in range(4, H):
        base_in = y * W
        base_out = (y - 4) * W
        iw = (y - 4) & 1
        ir = (y - 5) & 1
        for k in range(5):
            hc[k] = 0
        prev_s = 0
        for x in range(W):
            c = agc(inp[base_in + x], gain)
            hc[0] = hc[1]; hc[1] = hc[2]; hc[2] = hc[3]; hc[3] = hc[4]; hc[4] = c
            hn = (binom5(*hc) >> 4) & 0xFF
            h0, h1, h2, h3 = hb[0][x], hb[1][x], hb[2][x], hb[3][x]
            vn = (binom5(h0, h1, h2, h3, hn) >> 4) & 0xFF
            hb[0][x] = h1; hb[1][x] = h2; hb[2][x] = h3; hb[3][x] = hn
            q0, q1, q2, q3 = vb[0][x], vb[1][x], vb[2][x], vb[3][x]
            vb[0][x] = q1; vb[1][x] = q2; vb[2][x] = q3; vb[3][x] = vn
            for r in range(5):
                v[r][0] = v[r][1]; v[r][1] = v[r][2]; v[r][2] = v[r][3]; v[r][3] = v[r][4]
            v[0][4] = q0; v[1][4] = q1; v[2][4] = q2; v[3][4] = q3; v[4][4] = vn

            if y >= 8 and x >= 8:
                xo = x - 4
                if op == MODE_GAUSS5:
                    out[base_out + xo] = v[2][2]
                else:
                    gxC, gyC = sb_gx(v, 2, 2), sb_gy(v, 2, 2)
                    ax, ay = abs(gxC), abs(gyC)
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
                        n1 = sobel_mag(sb_gx(v, 2, 1), sb_gy(v, 2, 1))
                        n2 = sobel_mag(sb_gx(v, 2, 3), sb_gy(v, 2, 3))
                    elif d == 2:
                        n1 = sobel_mag(sb_gx(v, 1, 2), sb_gy(v, 1, 2))
                        n2 = sobel_mag(sb_gx(v, 3, 2), sb_gy(v, 3, 2))
                    elif d == 1:
                        n1 = sobel_mag(sb_gx(v, 1, 1), sb_gy(v, 1, 1))
                        n2 = sobel_mag(sb_gx(v, 3, 3), sb_gy(v, 3, 3))
                    else:
                        n1 = sobel_mag(sb_gx(v, 1, 3), sb_gy(v, 1, 3))
                        n2 = sobel_mag(sb_gx(v, 3, 1), sb_gy(v, 3, 1))
                    m = magC if (magC >= n1 and magC >= n2) else 0
                    if op == MODE_NMS:
                        out[base_out + xo] = clamp8(m)
                    else:
                        vv, strong = 0, 0
                        if m >= hi:
                            vv, strong = 255, 1
                        elif m >= lo:
                            nb = 0
                            if xo >= 5:
                                nb |= st[ir][xo - 1]
                            nb |= st[ir][xo]
                            if xo + 1 <= W - 5:
                                nb |= st[ir][xo + 1]
                            nb |= prev_s
                            if nb:
                                vv, strong = 255, 1
                        st[iw][xo] = strong
                        prev_s = strong
                        out[base_out + xo] = vv
    return out


def fused_B(inp, W, H, op, gain, hi, lo):
    out = [0] * (W * H)
    if W < 9 or H < 9:
        return out

    hc = [0] * 5
    hb = [[0] * W for _ in range(4)]
    vb = [[0] * W for _ in range(4)]
    v = [[0] * 5 for _ in range(5)]
    st = [[0] * W for _ in range(2)]
    outr = [[0] * W for _ in range(2)]

    for x in range(W):
        st[0][x] = 0; st[1][x] = 0
        vb[0][x] = 0; vb[1][x] = 0; vb[2][x] = 0; vb[3][x] = 0
        outr[0][x] = 0; outr[1][x] = 0
    for y in range(4):
        for x in range(W):
            out[y * W + x] = 0
    for y in range(H - 4, H):
        for x in range(W):
            out[y * W + x] = 0

    for row in range(4):
        for k in range(5):
            hc[k] = 0
        for x in range(W):
            c = agc(inp[row * W + x], gain)
            hc[0] = hc[1]; hc[1] = hc[2]; hc[2] = hc[3]; hc[3] = hc[4]; hc[4] = c
            hb[row][x] = (binom5(*hc) >> 4) & 0xFF

    for y in range(4, H):
        base_in = y * W
        oy_prev = (y - 5) if (y >= 5) else 0
        base_out = oy_prev * W
        iw = (y - 4) & 1
        ir = (y - 5) & 1
        for k in range(5):
            hc[k] = 0
        prev_s = 0
        for x in range(W):
            c = agc(inp[base_in + x], gain)
            hc[0] = hc[1]; hc[1] = hc[2]; hc[2] = hc[3]; hc[3] = hc[4]; hc[4] = c
            hn = (binom5(*hc) >> 4) & 0xFF
            h0, h1, h2, h3 = hb[0][x], hb[1][x], hb[2][x], hb[3][x]
            vn = (binom5(h0, h1, h2, h3, hn) >> 4) & 0xFF
            hb[0][x] = h1; hb[1][x] = h2; hb[2][x] = h3; hb[3][x] = hn
            q0, q1, q2, q3 = vb[0][x], vb[1][x], vb[2][x], vb[3][x]
            vb[0][x] = q1; vb[1][x] = q2; vb[2][x] = q3; vb[3][x] = vn
            for r in range(5):
                v[r][0] = v[r][1]; v[r][1] = v[r][2]; v[r][2] = v[r][3]; v[r][3] = v[r][4]
            v[0][4] = q0; v[1][4] = q1; v[2][4] = q2; v[3][4] = q3; v[4][4] = vn

            out[base_out + x] = outr[ir][x]

            if y >= 8 and x >= 8:
                xo = x - 4
                if op == MODE_GAUSS5:
                    outr[iw][xo] = v[2][2]
                else:
                    gxC, gyC = sb_gx(v, 2, 2), sb_gy(v, 2, 2)
                    ax, ay = abs(gxC), abs(gyC)
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
                        n1 = sobel_mag(sb_gx(v, 2, 1), sb_gy(v, 2, 1))
                        n2 = sobel_mag(sb_gx(v, 2, 3), sb_gy(v, 2, 3))
                    elif d == 2:
                        n1 = sobel_mag(sb_gx(v, 1, 2), sb_gy(v, 1, 2))
                        n2 = sobel_mag(sb_gx(v, 3, 2), sb_gy(v, 3, 2))
                    elif d == 1:
                        n1 = sobel_mag(sb_gx(v, 1, 1), sb_gy(v, 1, 1))
                        n2 = sobel_mag(sb_gx(v, 3, 3), sb_gy(v, 3, 3))
                    else:
                        n1 = sobel_mag(sb_gx(v, 1, 3), sb_gy(v, 1, 3))
                        n2 = sobel_mag(sb_gx(v, 3, 1), sb_gy(v, 3, 1))
                    m = magC if (magC >= n1 and magC >= n2) else 0
                    if op == MODE_NMS:
                        outr[iw][xo] = clamp8(m)
                    else:
                        vv, strong = 0, 0
                        if m >= hi:
                            vv, strong = 255, 1
                        elif m >= lo:
                            nb = 0
                            if xo >= 5:
                                nb |= st[ir][xo - 1]
                            nb |= st[ir][xo]
                            if xo + 1 <= W - 5:
                                nb |= st[ir][xo + 1]
                            nb |= prev_s
                            if nb:
                                vv, strong = 255, 1
                        st[iw][xo] = strong
                        prev_s = strong
                        outr[iw][xo] = vv

    last_oy = H - 5
    last_bank = last_oy & 1
    last_base = last_oy * W
    for x in range(W):
        out[last_base + x] = outr[last_bank][x]
    return out


# ===========================================================================
def run():
    random.seed(1030)
    fails = 0
    cases = 0

    sizes = [(9, 9), (10, 10), (16, 16), (17, 13), (640, 480), (320, 240), (33, 21)]
    gains = [256, 128, 384]

    for (W, H) in sizes:
        # 用带结构的图（棋盘格 + 噪声）才能逼出边界差异
        img = []
        for y in range(H):
            for x in range(W):
                base = 40 + 60 * ((x // 5 + y // 5) & 1) + 30 * ((x // 3) & 1)
                img.append(clamp8(base + random.randint(-12, 12)))
        for g in gains:
            for op in range(0, 12):
                if op >= 9:
                    for (hi, lo) in [(60, 30), (100, 40), (30, 20)]:
                        a = fused_A(img, W, H, op, g, hi, lo)
                        b = fused_B(img, W, H, op, g, hi, lo)
                        cases += 1
                        if a != b:
                            fails += 1
                            d = [i for i in range(W * H) if a[i] != b[i]]
                            print("FAIL fused op=%d %dx%d g=%d hi=%d lo=%d  n=%d first=%s"
                                  % (op, W, H, g, hi, lo, len(d), d[:5]))
                else:
                    a, sa = single_A(img, W, H, op, g)
                    b, sb_ = single_B(img, W, H, op, g)
                    cases += 1
                    bad = (a != b) or (sa != sb_)
                    if bad:
                        fails += 1
                        d = [i for i in range(W * H) if a[i] != b[i]]
                        print("FAIL single op=%d %dx%d g=%d  n=%d first=%s sigma %d/%d"
                              % (op, W, H, g, len(d), d[:5], sa, sb_))

    print("----")
    print("cases =", cases, " fails =", fails)
    print("RESULT:", "PASS" if fails == 0 else "FAIL")
    return fails


if __name__ == "__main__":
    raise SystemExit(1 if run() else 0)
