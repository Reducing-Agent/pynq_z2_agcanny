#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""validate_v8.py —— v8.2 板端回归（9 个单算子 + 四级融合流水线的 3 个抽头）

v8.2 相对 v8.1 的验证口径变化（这一版最容易写错的地方）
--------------------------------------------------------------
op 9 / 10 / 11 在 v8.2 里**不是三个并列算子**，而是**同一条四级流水线**的三个抽头：

    级1  AGC + 5×5 可分离高斯     ->  级2  Sobel 幅值 + 方向量化
    ->  级3  非极大值抑制 NMS     ->  级4  双阈值 + 滞后

    op 9  = 抽级1（去噪图）        op 11 = 抽级3（NMS 幅值）      op 10 = 抽级4（0/255 边缘）

所以验证时必须跟着改四件事，否则测的全是 v8.1 的旧语义：
  1. **喂原始 BGR**，不要在外面先做一次高斯 —— 去噪是链内级1，外面再做一遍等于做了两次
  2. 边界是**最外四圈**恒 0（四级各延迟 2 行 2 列），最小 **9×9**（不是 v8.1 的外两圈 / 5×5）
  3. op 9..11 的 σ **恒返回 0，且 sigma_valid 必须是 False**
     —— 那是"没测"，不是"测出来是 0"。把 0 当"噪声为 0"是 v8.1 踩过的坑
  4. 级联高斯（PL 连跑两遍 MODE_GAUSS）在 v8.2 里降级为**对照组**，不再是正路

和 validate_v7.py 的关键区别
----------------------------
1. 走 **hw_driver.py**，不再自己拼 MMIO。
   原因：v8 的寄存器偏移全变了（σ 挤占 0x10），如果这里再抄一份偏移，
   等于给"两边抄得不一样"留下第二个入口。驱动中只有一处定义。

2. 阶段 2 的 12 模式逐像素对拍：op 0..8 是单算子通路，op 9..11 是融合流水线。
   参考值一律取 `filters.process_bgr(img, mode, gain, thresh)`。

3. 阶段 3：σ 回读对拍。PL 的 ap_return 必须等于 filters.immerkaer_sigma_sum()
   的**精确整数** —— 累加是纯整数运算，两边应当一模一样。
   新增 op 0 / op 9..11 两组哨兵：这些模式下 σ 必须返回 0 且 sigma_valid=False。

4. 阶段 4：融合流水线专项
   · 阈值扫掠 —— 证明 0x38 的 thresh 寄存器**真的接上了**。
     这不是小题大做：σ 寄存器当年就是"csynth 过了、IP 也生成了，但头文件里
     根本没有那个寄存器"，特性等于没做。新增寄存器一律要这么验一次。
   · 单趟 vs 级联对照 —— 证明"一趟融合"确实等价于"两趟级联"（只差边界与截断）
   · 9×9 最小尺寸哨兵

5. 阶段 6：创新证据采集（σ 通路物理标定 + 伪边抑制率 + 真实边缘保持率）
   这一阶段产出的数字直接进设计报告 §4.2.3 和创新点章节。

跑法（在板子上，不要 sudo）
--------------------------
    source /usr/local/share/pynq-venv/bin/activate
    cd /home/xilinx/jupyter_notebooks/rgb2gray_v82
    python3 -u validate_v8.py

    -u 是必须的：不加的话 print 走 8KB 块缓冲，卡住时看不到进度。

不接板子时会怎样
----------------
阶段 0（纯 Python 自检）照跑，阶段 1~6 会因为没有 pynq 而退出并给出提示。
所以**改完 filters.py 先在 PC 上跑一遍阶段 0**，能挡掉大部分低级错误。

    python3 -u validate_v8.py --sim

加 `--sim` 会强制走纯软件参考路径，把阶段 2~6 也跑完 —— 用来在没有板子的
机器上验证**脚本逻辑本身**没问题（对拍口径、阈值扫掠、哨兵、证据采集）。
注意：--sim 下的**耗时不是 PL 耗时**，别把那些数字写进报告。
注意：阶段 0 里 op 9/10/11 用的是本脚本的**朴素循环**参考实现，
  比 op 0..8 慢很多，整机跑一次约 10~40 秒，属正常。
"""
import os
import sys
import time

import numpy as np

try:
    import cv2
    _HAS_CV2 = True
except ImportError:
    _HAS_CV2 = False

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

GAIN_UNITY = 256
GAINS = (256, 512, 1000)      # 1.0x 回归 / 2.0x 提亮 / ~3.9x 打饱和

# --sim：强制走软件参考路径，把阶段 2~6 也跑完（PC 上验证脚本逻辑用）
SIM_OK = "--sim" in sys.argv

# Canny 双阈值 (HIGH, LOW)。第一档是上电默认值，必须排在最前面。
THRESHES = ((60, 30), (100, 50), (30, 15))

# 阶段 6 的测试条件：棋盘块边长、低对比度边缘的两档灰度、注入噪声 σ
EDGE_BLOCK = 40
EDGE_LO, EDGE_HI = 116, 140      # 对比度 24 —— 与 σ=8 噪声同量级，才是真战场
NOISE_SIGMA = 8.0
# 单算子 Sobel 的阈值扫掠：证明"无论怎么调都做不到"要用一串阈值，不是一个
SOBEL_SWEEP = (30, 60, 100, 150, 200)

# v8.2：op 9/10/11 是四级融合流水线的三个抽头，不是三个并列算子
MODE_NAMES = {
    0: "GRAY", 1: "SOBEL", 2: "EMBOSS", 3: "GAUSS",
    4: "SHARPEN", 5: "LAPLACE", 6: "BINARY", 7: "MEAN",
    8: "DIR",          # v8 新增：梯度方向量化
    9: "GAUSS5",       # 抽头 A：级1 输出，5×5 二项式高斯去噪图
    10: "CANNY",       # 抽头 C：级4 输出，完整 Canny 边缘 0/255
    11: "NMS",         # 抽头 B：级3 输出，NMS 细化后的幅值
}
ALL_MODES = sorted(MODE_NAMES)                     # 0..11，共 12 个
SINGLE_MODES = [m for m in ALL_MODES if m < 9]     # 单算子 3×3 通路
FUSED_MODES = [m for m in ALL_MODES if m >= 9]     # 四级融合流水线的三个抽头
MODE_5X5_FIRST = 9


# ---------------- 合成测试图（全部 BGR 语义） ----------------
def make_gradient(h, w):
    """R 横向渐变 / G 纵向渐变 / B 常数 —— 唯一能测出 R/B 接反的图。"""
    yy, xx = np.indices((h, w))
    r = (xx * 255 // max(w - 1, 1)).astype(np.uint8)
    g = (yy * 255 // max(h - 1, 1)).astype(np.uint8)
    b = np.full((h, w), 64, dtype=np.uint8)
    return np.dstack([b, g, r]).astype(np.uint8)


def make_checker(h, w, block=16):
    yy, xx = np.indices((h, w))
    pat = (((yy // block) + (xx // block)) % 2).astype(np.uint8) * 255
    return np.dstack([pat, pat, pat]).astype(np.uint8)


def make_ramp(h, w):
    yy, xx = np.indices((h, w))
    v = (((xx + yy) * 255) // max((w + h - 2), 1)).astype(np.uint8)
    return np.dstack([v, v, v]).astype(np.uint8)


def make_noise(h, w, seed=7):
    """随机噪声是 σ 估计的主战场：σ 应当显著大于 0。"""
    rng = np.random.RandomState(seed)
    v = rng.randint(0, 256, size=(h, w)).astype(np.uint8)
    v[::4, ::4] = 255
    v[h // 3:2 * h // 3, w // 3:2 * w // 3] = 20
    return np.dstack([v, v, v]).astype(np.uint8)


def make_flat(h, w, level=128):
    """纯平坦图：σ 必须等于 0。这是 σ 通路最重要的**负例** ——
    如果平坦图也测出非零 σ，说明累加器在吃边界或初值没清。
    对融合流水线同样是负例：平坦图过 op 9 应当还是平坦的（高斯对常数幂等），
    过 op 10 应当全 0（没有边缘）。"""
    v = np.full((h, w), level, dtype=np.uint8)
    return np.dstack([v, v, v]).astype(np.uint8)


def make_rb_split(h, w):
    img = np.zeros((h, w, 3), dtype=np.uint8)
    img[:, : w // 2, 2] = 255     # 左半纯红
    img[:, w // 2:, 0] = 255      # 右半纯蓝
    return img


def make_noisy_flat(h, w, sigma=8.0, level=128, seed=11):
    """**没有真实边缘**、只有噪声的图 —— 阶段 6 测"伪边"用。

    为什么必须单独造一张：make_noise 里塞了棋盘亮点和暗块，
    那些本身就是边缘，会混进"伪边"计数里，把抑制率的口径搅浑。
    """
    rng = np.random.RandomState(seed)
    v = level + rng.normal(0.0, sigma, size=(h, w))
    v = np.clip(np.rint(v), 0, 255).astype(np.uint8)
    return np.dstack([v, v, v]).astype(np.uint8)


def make_clean_edges(h, w, block=EDGE_BLOCK, lo=40, hi=220):
    """干净的棋盘阶跃边缘图 —— 阶段 6 测"真边保持 / 伪边抑制"用。

    block 取大一点：Sobel 响应有 2 px 宽，块太小的话边缘互相挨着，
    NMS 会把它们并成一条，口径就不干净了。

    lo/hi 是两块的灰度。**阶段 6 用低对比度（116/140，差 24）**：
    高对比度（40/220）时单算子 Sobel 只要把阈值拉到 150 就能把伪边压下去，
    测不出融合链的价值。真正的战场是"边缘幅度和噪声同一量级"的场景 ——
    单算子在那儿无论怎么调阈值都是伪边和覆盖率此消彼长。
    """
    yy, xx = np.indices((h, w))
    pat = (((yy // block) + (xx // block)) % 2).astype(np.uint8)
    v = np.where(pat > 0, hi, lo).astype(np.uint8)
    return np.dstack([v, v, v]).astype(np.uint8)


# ---------------- 真值掩码（阶段 6 用） ----------------
def edge_truth(h, w, block=EDGE_BLOCK):
    """棋盘图的**真实边缘**位置（1 px 宽线），用来把检出点分成真阳/伪边。

    为什么不用"检出点总数"当指标：那个数对线宽极其敏感。
      完美平顶的合成图上，NMS 会遇到**并列**（相邻两列幅值完全相等），
      直边被算成 2~3 px 宽；一加噪声并列被打破，线宽塌成 1 px，
      总数掉 42% —— 看起来像"边缘丢了"，其实是并列假象。
      换成"落在真边 ±1 px 内才算命中"就与线宽无关了。
    """
    by = (np.arange(h) // block)[:, None]
    bx = (np.arange(w) // block)[None, :]
    ey = np.zeros((h, w), dtype=bool)
    ey[1:] = by[1:] != by[:-1]                 # 横向线
    ex = np.zeros((h, w), dtype=bool)
    ex[:, 1:] = bx[:, 1:] != bx[:, :-1]        # 纵向线
    return ey | ex


def dilate1(m):
    """4 邻域膨胀一格（±1 px 容差）。"""
    o = m.copy()
    o[1:, :] |= m[:-1, :]
    o[:-1, :] |= m[1:, :]
    o[:, 1:] |= m[:, :-1]
    o[:, :-1] |= m[:, 1:]
    return o


def score_det(det, truth, tol):
    """把检出掩码拆成：伪边 FP / 命中率 / 真边覆盖率。

    FP     = 落在真边 ±1 px **之外**的检出点 —— 这才是真正的"伪边"
    命中率 = 落在 ±1 px 内的检出点 / 全部检出点（检出的点里有多少是真的）
    覆盖率 = 真边像素里有多少被检出覆盖（真边漏没漏）
    """
    tp = int((det & tol).sum())
    fp = int((det & ~tol).sum())
    prec = 100.0 * tp / max(tp + fp, 1)
    cov = 100.0 * int((dilate1(det) & truth).sum()) / max(int(truth.sum()), 1)
    return fp, prec, cov


GENS = (("gradient", make_gradient), ("checker", make_checker),
        ("ramp", make_ramp), ("noise", make_noise),
        ("rb_split", make_rb_split))


# ---------------- ARM 参考 ----------------
def ref_gray(img):
    """灰度口径与 filters.py 同源（cv2 四舍五入）。"""
    if _HAS_CV2:
        return cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    src = img.astype(np.float32)
    return np.rint(0.299 * src[:, :, 2]
                   + 0.587 * src[:, :, 1]
                   + 0.114 * src[:, :, 0]).astype(np.uint8)


def apply_agc(gray, gain):
    if gain == GAIN_UNITY:
        return gray
    return np.clip((gray.astype(np.int32) * int(gain)) >> 8,
                   0, 255).astype(np.uint8)


def maxdiff(a, b):
    return int(np.abs(a.astype(np.int32) - b.astype(np.int32)).max())


# ---------------------------------------------------------------------------
# 融合流水线（op 9/10/11）的**独立**朴素循环参考
#
# 为什么不用 numpy 切片：filters.py 已经是"整幅切片 + 向量化"的路子了。
# 再写一遍切片版，两边会犯**同一个错**（比如轴写反、上界多切两行），
# 差值恒为 0 那是**假通过**。这里刻意用逐像素三重循环 + 显式下标，
# 是另一条完全不同的代码路径。10-28 那次 Immerkær 可分离实现把轴写反、
# 输出 478x636 而不是 478x638，就是靠独立参照才当场抓出来的。
# ---------------------------------------------------------------------------
_B5 = (1, 4, 6, 4, 1)          # 一维二项式核 [1,4,6,4,1]，系数和 = 16（不是 256！）


def _ref_vconv5(g):
    """级1：5×5 可分离二项式高斯，零填充，**两级各 >>4**。

    一维核系数和是 **16**（二维才是 16×16=256）。PL 横向 >>4、纵向 >>4，
       中间结果被截成 8 bit。写成一次 >>8 会在噪声图上随机差 1 ——
       这个精度损失真实存在，v8.1 就是栽在这上面。

    行掩蔽：V 的第 0/1 行和第 H-2/H-1 行恒 0（PL 的流水起点决定的）。
       别漏 —— op 9 的第一行有效输出是 y=4，它的 Sobel 窗口要往上摸到 V[1]，
       那个位置在 PL 里就是 0。漏了掩蔽，最上面两行有效区会对不上。

    返回 list[list[int]]，形状 (H, W)。
    """
    H, W = g.shape
    T = [[0] * W for _ in range(H)]
    V = [[0] * W for _ in range(H)]
    if H < 9 or W < 9:
        return V
    # 横向：T[y][x] = (Σ_k B5[k] * g[y][x+k-2]) >> 4，越界补 0
    for y in range(H):
        row = g[y]
        ty = T[y]
        for x in range(W):
            s = 0
            for k in range(5):
                j = x + k - 2
                if 0 <= j < W:
                    s += _B5[k] * int(row[j])
            ty[x] = s >> 4
    # 纵向：V[y][x] = (Σ_k B5[k] * T[y+k-2][x]) >> 4，越界补 0
    for y in range(H):
        vy = V[y]
        for x in range(W):
            s = 0
            for k in range(5):
                i = y + k - 2
                if 0 <= i < H:
                    s += _B5[k] * T[i][x]
            vy[x] = s >> 4
    # 行掩蔽（与 filters._vconv_gray 的 V[:2]=0 / V[H-2:]=0 对应）
    for y in list(range(2)) + list(range(H - 2, H)):
        V[y] = [0] * W
    return V


def _sobel_at(w, r, c):
    """在 5×5 窗口 w 的 (r,c) 处取 Sobel 幅值 |Gx|+|Gy|，r,c ∈ [1,3]。"""
    gx = (w[r - 1][c + 1] + 2 * w[r][c + 1] + w[r + 1][c + 1]
          - w[r - 1][c - 1] - 2 * w[r][c - 1] - w[r + 1][c - 1])
    gy = (w[r + 1][c - 1] + 2 * w[r + 1][c] + w[r + 1][c + 1]
          - w[r - 1][c - 1] - 2 * w[r - 1][c] - w[r - 1][c + 1])
    return abs(gx) + abs(gy)


def _ref_fused(g, mode, thresh):
    """op 9/10/11 的朴素循环参考。g 是**过了 AGC 的灰度**（int32，0..255）。

    有效区 = 行 [4, H-5] × 列 [4, W-5]，最外**四**圈恒 0。
    """
    H, W = g.shape
    out = np.zeros((H, W), dtype=np.uint8)
    if H < 9 or W < 9:
        return out

    V = _ref_vconv5(g)

    if mode == 9:                       # 抽头 A：级1 去噪图
        for y in range(4, H - 4):
            vy = V[y]
            oy = out[y]
            for x in range(4, W - 4):
                oy[x] = vy[x] & 0xFF
        return out

    # 级2 + 级3：中心 Sobel + 方向量化 + 9 点非极大值抑制
    # 9 个 Sobel 位置（中心 + 4 组对侧邻居）全部落在同一个 5×5 窗口内
    #   （中心 ±2），所以**不需要 7×7** —— 这是 v8.2 能省资源的关键。
    mag = np.zeros((H, W), dtype=np.int32)
    for y in range(4, H - 4):
        for x in range(4, W - 4):
            w = [[V[y - 2 + i][x - 2 + j] for j in range(5)] for i in range(5)]
            mC = _sobel_at(w, 2, 2)
            gxc = (w[1][3] + 2 * w[2][3] + w[3][3]
                   - w[1][1] - 2 * w[2][1] - w[3][1])
            gyc = (w[3][1] + 2 * w[3][2] + w[3][3]
                   - w[1][1] - 2 * w[1][2] - w[1][3])
            ax, ay = abs(gxc), abs(gyc)
            # 与 PL 完全同式：2*|·| 比较代替 tan(22.5°)/tan(67.5°)，避免浮点
            if 2 * ay <= ax:
                d = 0
            elif 2 * ax <= ay:
                d = 2
            elif gxc * gyc > 0:
                d = 1
            else:
                d = 3
            if d == 0:
                n1, n2 = _sobel_at(w, 2, 1), _sobel_at(w, 2, 3)
            elif d == 2:
                n1, n2 = _sobel_at(w, 1, 2), _sobel_at(w, 3, 2)
            elif d == 1:
                n1, n2 = _sobel_at(w, 1, 1), _sobel_at(w, 3, 3)
            else:
                n1, n2 = _sobel_at(w, 1, 3), _sobel_at(w, 3, 1)
            mag[y, x] = mC if (mC >= n1 and mC >= n2) else 0

    if mode == 11:                      # 抽头 B：NMS 细化后的幅值
        return np.clip(mag, 0, 255).astype(np.uint8)

    # 级4：双阈值 + 滞后
    hi = (int(thresh) >> 8) & 0xFF
    lo = int(thresh) & 0xFF
    if hi < lo:
        hi = lo
    strong = [[0] * W for _ in range(H)]
    # 滞后必须**按 y 递增、x 递增**逐像素处理：右邻和下一行在流式顺序里
    #   还没算出来，所以邻域只查「上一行 3 邻 + 当前行左邻」。
    #   被提升的弱边会变成强边继续向右/向下传播，顺序不能乱。
    for y in range(4, H - 4):
        sp = strong[y - 1]
        sy = strong[y]
        my = mag[y]
        oy = out[y]
        for x in range(4, W - 4):
            m = int(my[x])
            if m >= hi:
                oy[x] = 255
                sy[x] = 1
            elif m >= lo:
                nb = 0
                if x >= 5:
                    nb |= sp[x - 1]
                nb |= sp[x]
                if x + 1 <= W - 5:
                    nb |= sp[x + 1]
                if x >= 5:
                    nb |= sy[x - 1]
                if nb:
                    oy[x] = 255
                    sy[x] = 1
    return out


def reference(img, mode, gain=GAIN_UNITY, thresh=None):
    """**独立**的 ARM 黄金参考：朴素三重循环，不用任何 numpy 切片技巧。

    op 0..8  走 3×3 单算子循环（外一圈恒 0）
    op 9..11 走 _ref_fused()（外四圈恒 0，最小 9×9）

    代价：比向量化慢约 100 倍（融合链更慢），所以只在小图上跑。
    """
    h, w = img.shape[:2]
    g = apply_agc(ref_gray(img), gain).astype(np.int32)
    if thresh is None:
        thresh = (60 << 8) | 30

    if mode == 0:
        return g.astype(np.uint8)
    if mode >= MODE_5X5_FIRST:
        return _ref_fused(g, mode, thresh)

    out = np.zeros((h, w), dtype=np.uint8)
    if h < 3 or w < 3:
        return out

    def px(y, x):
        return int(g[y, x])

    for y in range(1, h - 1):
        for x in range(1, w - 1):
            p00, p01, p02 = px(y - 1, x - 1), px(y - 1, x), px(y - 1, x + 1)
            p10, p11, p12 = px(y, x - 1), px(y, x), px(y, x + 1)
            p20, p21, p22 = px(y + 1, x - 1), px(y + 1, x), px(y + 1, x + 1)

            if mode == 1:                       # Sobel |gx| + |gy|
                gx = (p02 + 2 * p12 + p22) - (p00 + 2 * p10 + p20)
                gy = (p20 + 2 * p21 + p22) - (p00 + 2 * p01 + p02)
                v = abs(gx) + abs(gy)
            elif mode == 2:                     # Emboss
                v = p22 - p00 + 128
            elif mode == 3:                     # Gaussian /16
                v = (p00 + 2 * p01 + p02
                     + 2 * p10 + 4 * p11 + 2 * p12
                     + p20 + 2 * p21 + p22) >> 4
            elif mode == 4:                     # Sharpen
                v = 5 * p11 - (p01 + p10 + p12 + p21)
            elif mode == 5:                     # Laplace
                v = 8 * p11 - (p00 + p01 + p02 + p10
                               + p12 + p20 + p21 + p22)
            elif mode == 6:                     # Binary
                v = 255 if p11 >= 128 else 0
            elif mode == 7:                     # Mean /9（与 PL 同式）
                v = ((p00 + p01 + p02 + p10 + p11 + p12
                      + p20 + p21 + p22) * 1821) >> 14
            elif mode == 8:                     # DIR 方向量化
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
                v = d * 85
            else:
                raise ValueError("mode %r" % mode)

            out[y, x] = 0 if v < 0 else (255 if v > 255 else v)
    return out


# ---------------- 阶段 0：纯 Python 自检（不需要板子） ----------------
def selfcheck(verbose=True):
    """filters.py（切片向量化） vs 本脚本的朴素循环参考，必须逐像素相等。"""
    try:
        import filters
    except Exception as e:
        print("    [!] 无法 import filters.py：%s" % e)
        return False

    ok = True
    imgs = [(n, f(32, 40)) for n, f in GENS]
    imgs += [("flat", make_flat(32, 40)), ("tiny3", make_gradient(3, 3)),
             ("tiny2", make_gradient(2, 2))]

    if verbose:
        print("    op 9/10/11 用朴素循环独立参考，比 op 0..8 慢，请稍候 ...")
    t0 = time.perf_counter()
    for name, img in imgs:
        for m in ALL_MODES:
            for g in GAINS:
                a = filters.process_bgr(img, m, g)
                b = reference(img, m, g)
                d = maxdiff(a, b)
                if d != 0:
                    ok = False
                    if verbose:
                        print("    FAIL %-9s mode %2d %-8s gain %d maxdiff=%d"
                              % (name, m, MODE_NAMES[m], g, d))
    if verbose and ok:
        print("    黄金参考自洽：%d 图 x %d 模式 x %d 增益 全部相等（%.1f s）"
              % (len(imgs), len(ALL_MODES), len(GAINS),
                 time.perf_counter() - t0))

    # ---- 融合流水线的**负例**：平坦图 ----
    flat = apply_agc(ref_gray(make_flat(64, 64)), GAIN_UNITY)
    g9 = filters.gauss5_gray(flat)
    g10 = filters.canny5_gray(flat)
    # 高斯对常数幂等：平坦 128 进去应当还是平坦（有效区内）。
    # 别拿"某个像素值变了"当判据 —— 高斯对平坦区幂等，取常数值会假失败。
    #   用"有效区内极差"当判据：幂等 -> 极差 0；写错归一化 -> 极差不为 0。
    span9 = int(g9[4:60, 4:60].max()) - int(g9[4:60, 4:60].min())
    flat_ok = (span9 == 0 and int(g10.sum()) == 0)
    if verbose:
        print("    平坦图负例：op9 有效区极差=%d（应 0）, op10 非零点=%d（应 0） %s"
              % (span9, int(g10.sum()), "OK" if flat_ok else "FAIL"))

    # ---- σ 参考实现自检：平坦图必须 0，噪声图必须显著 > 0 ----
    s_flat = filters.immerkaer_sigma_sum(flat)
    noise = apply_agc(ref_gray(make_noise(64, 64)), GAIN_UNITY)
    s_noise = filters.immerkaer_sigma_sum(noise)
    sig_ok = (s_flat == 0 and s_noise > 0)
    if verbose:
        print("    σ 参考自检：平坦图 Σ=%d（应 0）, 噪声图 Σ=%d σ=%.2f  %s"
              % (s_flat, s_noise, filters.sigma_from_sum(s_noise, 64, 64),
                 "OK" if sig_ok else "FAIL"))

    # ---- 级联高斯自检（v8.2 里它只是对照组，但仍是有效的降级路径）----
    # 判据用"逐趟都在变 + 方差单调下降"，这两个是算子层面的性质，
    #   不依赖具体像素落在哪（取单点会踩"高斯对平坦区幂等"的坑）。
    g1 = filters.gauss3_gray(noise)
    g2 = filters.gauss3_gray(g1)
    d01 = maxdiff(noise, g1)
    d12 = maxdiff(g1, g2)
    s0, s1, s2 = (float(noise.std()), float(g1.std()), float(g2.std()))
    casc_ok = (g2.shape == noise.shape and d01 > 0 and d12 > 0
               and s2 < s1 < s0)
    if verbose:
        print("    级联高斯自检：形状 %s，maxdiff 第1趟=%d 第2趟=%d，"
              "std %.2f -> %.2f -> %.2f  %s"
              % (g2.shape, d01, d12, s0, s1, s2,
                 "OK" if casc_ok else "FAIL"))

    # ---- 通道顺序断言 ----
    red = np.zeros((4, 4, 3), np.uint8); red[:, :, 2] = 255
    blue = np.zeros((4, 4, 3), np.uint8); blue[:, :, 0] = 255
    g_red, g_blue = int(ref_gray(red)[0, 0]), int(ref_gray(blue)[0, 0])
    ch_ok = (g_red == 76 and g_blue == 29)
    if verbose:
        print("    通道断言：纯红 -> %d（应 76）, 纯蓝 -> %d（应 29） %s"
              % (g_red, g_blue, "OK" if ch_ok else "FAIL R/B 接反了"))

    return ok and flat_ok and sig_ok and casc_ok and ch_ok


# ---------------- 主流程 ----------------
def main():
    print("=" * 72)
    print("rgb2gray v8.2 验证：9 个单算子 + 四级融合流水线（op 9/10/11 三抽头）")
    print("=" * 72)

    print("\n[阶段 0] 纯 Python 自检（不需要板子）")
    if not selfcheck():
        print("\n[!] 阶段 0 未通过，先修 filters.py / 本脚本再上板。")
        return 1

    try:
        import filters
    except Exception as e:
        print("[!] import filters 失败：%s" % e)
        return 1

    if SIM_OK:
        pass                      # --sim 不需要 pynq，直接走软件参考
    else:
        try:
            from pynq import Overlay
        except ModuleNotFoundError:
            print("\n[!] 当前解释器里没有 pynq：%s" % sys.executable)
            print("    多半是用了 `sudo python3` —— sudo 会 env_reset，"
                  "丢掉 pynq-venv，Device.devices 直接变空。")
            print("    改用这条重跑（**不要** sudo）：")
            print("        source /usr/local/share/pynq-venv/bin/activate")
            print("        python3 -u %s" % os.path.basename(__file__))
            print("    （只想在没有板子的机器上验脚本逻辑，就加 --sim）")
            return 2

    try:
        import hw_driver
    except Exception as e:
        print("[!] import hw_driver 失败：%s" % e)
        return 1

    # ---------------------------------------------------------- 阶段 1
    print("\n" + "-" * 72)
    print("[阶段 1] 加载位流（hw_driver 会自动校验 MD5）")
    print("-" * 72)
    try:
        proc = hw_driver.HardwareProcessor(force_sim=True) if SIM_OK \
            else hw_driver.HardwareProcessor()
    except Exception as e:
        print("[!] 位流加载失败：%r" % e)
        return 3
    if proc._sim:
        if not SIM_OK:
            print("[!] 驱动退回了 SIM 模式，下面是纯软件结果，**不是 PL 结果**")
            return 4
        print("    !! --sim：全程走软件参考路径，**下面的耗时不是 PL 耗时**")
    print("    位流 MD5 校验：%s" % (proc.bit_manifest or "skipped"))

    # AGC 保留、但用 gain_override 钉死增益（否则一阶平滑让增益依赖历史帧）
    proc.agc_enabled = True
    # 阈值同样钉死：自适应一开会随上一帧 σ 漂，对拍不可复现
    proc.thresh_override = filters.CANNY_THRESH_DEFAULT
    W, H = 64, 64
    total, fails = 0, []

    # ---------------------------------------------------------- 阶段 2
    print("\n[阶段 2] 64x64 五图 x %d 模式 x %d 增益 逐像素对拍"
          % (len(ALL_MODES), len(GAINS)))
    print("          op 0..8 = 单算子 3x3 通路（外一圈恒 0）")
    print("          op 9/10/11 = 四级融合流水线三抽头（外四圈恒 0）")
    for gname, gfun in GENS:
        img = gfun(H, W)
        for g in GAINS:
            proc.gain_override = g
            for m in ALL_MODES:
                pl = proc.run(m, img)
                rf = filters.process_bgr(img, m, g, proc.last_thresh)
                d = maxdiff(pl, rf)
                total += 1
                if d != 0:
                    fails.append((gname, m, MODE_NAMES[m], g, d))
    print("    %d 组，失败 %d 组" % (total, len(fails)))
    for f in fails[:10]:
        print("      FAIL %s mode %d (%s) gain %d maxdiff=%d" % f)

    # ---------------------------------------------------------- 阶段 3：σ
    print("\n[阶段 3] σ 回读（PL ap_return vs 参考累加和，必须精确相等）")
    sig_total, sig_fails = 0, []
    sig_rows = []
    for gname, gfun in GENS + (("flat", make_flat),):
        img = gfun(H, W)
        for g in GAINS:
            proc.gain_override = g
            for m in (1, 3, 5, 8):        # 抽 4 个 3x3 算子即可（σ 与算子无关）
                proc.run(m, img)
                pl_sum = proc.last_sigma_sum
                ref_sum = filters.immerkaer_sigma_sum(
                    apply_agc(ref_gray(img), g))
                sig_total += 1
                if pl_sum != ref_sum:
                    sig_fails.append((gname, m, g, pl_sum, ref_sum))
        # 每图记一行 gain=256 的 σ 值，供人工核对量级
        proc.gain_override = GAIN_UNITY
        proc.run(5, img)
        sig_rows.append((gname, proc.last_sigma_sum,
                         proc.last_sigma, proc.sigma_valid))
    print("    %d 组，失败 %d 组" % (sig_total, len(sig_fails)))
    for f in sig_fails[:10]:
        print("      FAIL %s mode %d gain %d  PL=%d  REF=%d  差=%d"
              % (f[0], f[1], f[2], f[3], f[4], f[4] - f[3]))
    print("    gain=256 下各图 σ（σ 应当只跟图有关，跟算子无关）：")
    for r in sig_rows:
        print("      %-9s Σ=%9d  σ=%6.2f  valid=%s" % r)

    # 哨兵：σ 无效的两类情况。
    #   这些模式下 PL **恒返回 0**，那是"没测"，不是"测出来是 0"。
    #   驱动用 sigma_valid=False 区分，这里必须验证驱动真的这么标了 ——
    #   否则 HUD 会把"没测"显示成"噪声为 0"，AGC 和 Canny 阈值会跟着跑偏。
    proc.gain_override = GAIN_UNITY
    sent_ok = True

    proc.run(0, make_noise(H, W))
    op0_ok = (proc.last_sigma_sum == 0 and proc.sigma_valid is False)
    sent_ok &= op0_ok
    print("    哨兵 op0（GRAY，逐像素没 3x3 窗口）：Σ=%d valid=%s  %s"
          % (proc.last_sigma_sum, proc.sigma_valid, "OK" if op0_ok else "FAIL"))

    for m in FUSED_MODES:
        proc.run(m, make_noise(H, W))
        m_ok = (proc.last_sigma_sum == 0 and proc.sigma_valid is False)
        sent_ok &= m_ok
        print("    哨兵 op%-2d（%-7s 融合流水线恒返回 0）：Σ=%d valid=%s  %s"
              % (m, MODE_NAMES[m], proc.last_sigma_sum, proc.sigma_valid,
                 "OK" if m_ok else "FAIL"))

    # ---------------------------------------------------------- 阶段 4
    print("\n[阶段 4] 融合流水线专项")
    thr_total, thr_fails = 0, []
    thr_outs = []
    img = make_noise(H, W)
    proc.gain_override = GAIN_UNITY
    for hi, lo in THRESHES:
        t = filters.thresh_pack(hi, lo)
        proc.thresh_override = t
        pl = proc.run(10, img)
        rf = filters.process_bgr(img, 10, GAIN_UNITY, t)
        d = maxdiff(pl, rf)
        thr_total += 1
        thr_outs.append(int((pl == 255).sum()))
        if d != 0:
            thr_fails.append((hi, lo, d))
        print("      thresh %3d/%-3d  对拍 maxdiff=%d  PL 边缘像素=%d"
              % (hi, lo, d, thr_outs[-1]))
    # 关键断言：三档阈值的输出必须**互不相同**。
    #   如果三档都一样，说明 0x38 的 thresh 寄存器根本没接上（写了也白写），
    #   op 10 一直在用某个固定值 —— σ 寄存器当年就是这么"假装实现了"的。
    thr_live = len(set(thr_outs)) == len(THRESHES)
    if not thr_live:
        thr_fails.append(("thresh 寄存器疑似未生效", 0, 0))
    print("      阈值寄存器生效性：%s（三档边缘像素数 %s）"
          % ("OK" if thr_live else "FAIL —— 0x38 可能没生成寄存器",
             thr_outs))
    proc.thresh_override = filters.CANNY_THRESH_DEFAULT

    # 单趟 op9 vs 两趟级联：数学上等价（[1,2,1]⊗[1,2,1] = [1,4,6,4,1]），
    # 只差边界零填充和截断位置，所以只比中心区，判据用平均绝对差。
    casc = proc.run_gauss_cascade(img, passes=2)
    one = proc.run(9, img)
    core = (slice(8, H - 8), slice(8, W - 8))     # 再往里缩 4 圈，避开边界差异
    mcore = int(np.abs(one[core].astype(np.int32)
                       - casc[core].astype(np.int32)).mean())
    dcore = maxdiff(one[core], casc[core])
    # 软判据：级联每趟都把外圈清零（零填充更宽），且截断位置不同，
    #   差几个灰度级是正常的。这里只挡"量级不对"（比如归一化写错 -> 差几十）。
    casc_cmp_ok = mcore <= 5
    print("      单趟 op9 vs 两趟级联：中心区平均差 %.2f  最大差 %d  %s"
          % (mcore, dcore, "OK" if casc_cmp_ok else "WARN 量级偏大"))

    # 最小尺寸哨兵：9×9 时有效区只剩中心 1 个像素
    tiny = make_gradient(9, 9)
    o9 = proc.run(9, tiny)
    tiny_ok = (o9.shape == (9, 9) and int((o9 != 0).sum()) <= 1)
    print("      9x9 最小尺寸哨兵：shape=%s 非零点=%d（应 <=1） %s"
          % (o9.shape, int((o9 != 0).sum()), "OK" if tiny_ok else "FAIL"))

    # ---------------------------------------------------------- 阶段 5：耗时
    print("\n[阶段 5] 640x480 核内耗时（II=1 地板 = 3.072 ms）")
    big = make_noise(480, 640)
    proc.gain_override = GAIN_UNITY
    for m in ALL_MODES:
        ts = []
        for _ in range(5):
            proc.run(m, big)
            ts.append(proc.last_pl_ms)
        tag = "融合" if m >= MODE_5X5_FIRST else "单算子"
        print("      mode %2d %-8s %s  PL %6.3f ms   pack %6.3f ms"
              % (m, MODE_NAMES[m], tag, min(ts), proc.last_pack_ms))
    t0 = time.perf_counter()
    proc.run_gauss_cascade(big, passes=2)
    casc_ms = (time.perf_counter() - t0) * 1000
    print("      级联高斯（2 趟，对照组）  端到端 %6.3f ms" % casc_ms)
    if SIM_OK:
        print("      （--sim：PL 耗时无意义，真板数据请跑板子上的同一阶段）")

    # ---------------------------------------------------------- 阶段 6：证据
    print("\n[阶段 6] 创新证据采集（直接进设计报告 §4.2.3 / 创新点章节）")
    proc.gain_override = GAIN_UNITY
    H2, W2 = 480, 640
    NSIG = NOISE_SIGMA
    truth = edge_truth(H2, W2)
    tol = dilate1(truth)

    base = make_clean_edges(H2, W2, lo=EDGE_LO, hi=EDGE_HI)
    # 三个通道加**同一份**噪声：等价于直接给灰度加噪，
    #   否则 RGB 三通道各自独立抖动，灰度化后噪声被 /sqrt(3) 平均掉，
    #   实际 σ 会比你以为的小，σ 标定和伪边计数全对不上。
    nz = np.rint(np.random.RandomState(3).normal(0.0, NSIG, (H2, W2)))
    noisy = np.clip(base.astype(np.int16) + nz[:, :, None].astype(np.int16),
                    0, 255).astype(np.uint8)
    flat_n = make_noisy_flat(H2, W2, sigma=NSIG)
    flat_c = make_flat(H2, W2)

    # (1) σ 通路物理标定：干净图 σ≈0，加 σ=8 噪声后应当测出 ≈8
    proc.run(5, flat_c)
    s_clean = proc.last_sigma
    proc.run(5, flat_n)
    s_noisy = proc.last_sigma
    sig_phys_ok = (s_clean < 1.0 and 0.6 * NSIG < s_noisy < 1.6 * NSIG)
    print("      (1) σ 物理标定：平坦图 σ=%.2f（应≈0）  注入 σ=%.0f 实测 σ=%.2f  %s"
          % (s_clean, NSIG, s_noisy,
             "OK" if sig_phys_ok else "WARN 量级对不上"))

    # (2) 纯噪声图（**没有**任何真实边缘）：融合链不该把噪声当边缘
    lo_t = filters.thresh_low(filters.CANNY_THRESH_DEFAULT)
    a_sob = int((proc.run(1, flat_n) >= lo_t).sum())      # 单算子 Sobel
    b_nms = int((proc.run(11, flat_n) >= lo_t).sum())     # 抽头 B（去噪+NMS）
    b_can = int((proc.run(10, flat_n) == 255).sum())      # 抽头 C（+滞后）
    fp_ok = (a_sob > 0 and b_can <= a_sob * 0.3)
    print("      (2) 纯噪声图（无真边，同一门槛 lo=%d）：" % lo_t)
    print("           未去噪 Sobel(op1)  %7d  <- 噪声全被当成边缘" % a_sob)
    print("           去噪+NMS(op11)     %7d   减少 %.1f%%"
          % (b_nms, (1 - b_nms / float(a_sob)) * 100 if a_sob else 0))
    print("           完整 Canny(op10)   %7d   减少 %.1f%%  %s"
          % (b_can, (1 - b_can / float(a_sob)) * 100 if a_sob else 0,
             "OK" if fp_ok else "FAIL 抑制不足"))

    # (3) 低对比度边缘 + 噪声：真值掩码口径
    #     比的是**等覆盖率下的伪边数** —— 直接比检出总数不公平：
    #       单算子把阈值拉到 150 也能把伪边压到几十，但真边覆盖率会从 97%
    #       塌到 45%，那不叫"更好"，那叫"把边缘一起丢了"。
    print("      (3) 低对比度边缘（灰度 %d/%d，差 %d）+ 噪声 σ=%.0f，"
          "真边 %d px：" % (EDGE_LO, EDGE_HI, EDGE_HI - EDGE_LO, NSIG,
                            int(truth.sum())))
    fp10, prec10, cov10 = score_det(proc.run(10, noisy) == 255, truth, tol)
    print("           op10 完整 Canny（阈值 60/30）  FP=%6d  命中率 %5.1f%%  "
          "真边覆盖 %5.1f%%" % (fp10, prec10, cov10))
    sob = proc.run(1, noisy)
    rows = []
    for t in SOBEL_SWEEP:
        fp1, p1, c1 = score_det(sob >= t, truth, tol)
        rows.append((t, fp1, p1, c1))
    # 在覆盖率不低于 op10 的档位里，挑伪边最少的那个 —— 那才是单算子的上限
    best = None
    for r in rows:
        if r[3] >= cov10 - 0.5 and (best is None or r[1] < best[1]):
            best = r
    for r in rows:
        print("           op1 Sobel  thr=%3d              FP=%6d  "
              "命中率 %5.1f%%  真边覆盖 %5.1f%%%s"
              % (r[0], r[1], r[2], r[3],
                 "  <- 单算子最优（覆盖不劣于 op10）" if r is best else ""))
    if best is None:
        print("           [!] 单算子没有任何一档能达到 op10 的覆盖率 —— "
              "这本身就是结论，但换张图复核一下")
        fp_ok2 = True
        red = 1.0
    else:
        red = 1.0 - fp10 / float(best[1]) if best[1] else 1.0
        fp_ok2 = fp10 <= best[1] * 0.5
        print("           等覆盖率对比：Sobel thr=%d FP=%d  ->  Canny FP=%d  "
              "伪边减少 %.1f%%  %s"
              % (best[0], best[1], fp10, red * 100,
                 "OK" if fp_ok2 else "FAIL 抑制不足"))
    cov_ok = cov10 >= 90.0
    print("           真边覆盖率 %.1f%%  %s"
          % (cov10, "OK" if cov_ok else "WARN 真边漏太多"))
    fp_ok = fp_ok and fp_ok2

    # (4) 阈值工作点扫描：给报告一张"为什么默认 60/30"的选型依据表。
    #     这张表同时是自适应阈值的**安全边界**证据：
    #       HIGH 超过 10σ 就开始丢真边，所以 CANNY_K_HI 只能取 ≈7.5σ，
    #       旧版 16σ 会把覆盖率打到 0（hw_driver 文件头有完整推导）。
    print("      (4) 阈值工作点扫描（同一张图，PL 实跑，σ 实测 %.2f）："
          % s_noisy)
    dflt = (filters.thresh_high(filters.CANNY_THRESH_DEFAULT),
            filters.thresh_low(filters.CANNY_THRESH_DEFAULT))
    for hi, lo in ((24, 12), (40, 20), (60, 30), (80, 40), (128, 64)):
        proc.thresh_override = filters.thresh_pack(hi, lo)
        fp, p, c = score_det(proc.run(10, noisy) == 255, truth, tol)
        print("           HIGH=%3d LOW=%2d (%4.1fσ)  FP=%6d  命中率 %5.1f%%  "
              "真边覆盖 %5.1f%%%s"
              % (hi, lo, hi / max(s_noisy, 1e-6), fp, p, c,
                 "  <- 上电默认值（推荐工作点）" if (hi, lo) == dflt else ""))
    proc.thresh_override = filters.CANNY_THRESH_DEFAULT

    # ---------------------------------------------------------- 汇总
    print("\n" + "=" * 72)
    hard = (len(fails) + len(sig_fails) + len(thr_fails)
            + (0 if sent_ok else 1) + (0 if tiny_ok else 1)
            + (0 if fp_ok else 1))
    soft = (0 if casc_cmp_ok else 1) + (0 if sig_phys_ok else 1) \
        + (0 if cov_ok else 1)
    if hard == 0:
        print("硬判据全部通过：%d 像素对拍 + %d σ 对拍 + %d 阈值对拍 "
              "+ 4 哨兵 + 伪边抑制" % (total, sig_total, thr_total))
    else:
        print("硬判据失败 %d 项：像素 %d / σ %d / 阈值 %d / 哨兵 %d / 伪边 %d"
              % (hard, len(fails), len(sig_fails), len(thr_fails),
                 0 if sent_ok else 1, 0 if fp_ok else 1))
    if soft:
        print("软判据告警 %d 项（不阻断，但报告数字要复核）："
              "级联对照 %s / σ 标定 %s / 真边覆盖 %s"
              % (soft, "OK" if casc_cmp_ok else "WARN",
                 "OK" if sig_phys_ok else "WARN",
                 "OK" if cov_ok else "WARN"))
    print("=" * 72)
    return 0 if hard == 0 else 5


if __name__ == "__main__":
    sys.exit(main())
