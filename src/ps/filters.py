"""filters v5 —— 8 模式 PC 参考实现（硬件侧提供，请软件组并入 filters.py）

为什么必须逐像素一致
--------------------
这是黄金参考（golden reference）。PL 的输出要和它**逐像素相等**（最大差 0），
差一个像素都说明硬件或参考实现有一边写错了。所以下面这些公式的顺序、括号、
取整方式都不能"顺手优化"——比如把 `>> 4` 改成 `// 16` 在非负数上等价，
但把 `/9` 改成 `// 9` 和 `(x*1821)>>14` 在边界值上可能差 1。

新增的 5 个算子（mode 3~7）
--------------------------
全部是 3×3 卷积/邻域运算，与 mode 1/2 共用同一套滑窗结构。
除法刻意用移位+常数乘实现，避免占用 DSP：
    /16 = >> 4                精确，零代价
    /9  = (x * 1821) >> 14    穷举 0..2295 全部等于 floor(x/9)，1821 仅 11 bit

边界语义
--------
mode 0 是逐像素算子，最外一圈正常输出。
mode 1~7 是 3×3 算子，最外一圈恒为 0（凑不出完整窗口）——与 v3/v4 完全一致。
"""
import numpy as np

try:
    import cv2
    _HAS_CV2 = True
except ImportError:
    _HAS_CV2 = False

MODE_NAMES = {
    0: "GRAY",
    1: "SOBEL",
    2: "EMBOSS",
    3: "GAUSS",
    4: "SHARPEN",
    5: "LAPLACE",
    6: "BINARY",
    7: "MEAN",
    8: "DIR",      # v8 新增：梯度方向量化 0/85/170/255
    9: "GAUSS5",   # v8.1：5x5 二项式高斯 [1,4,6,4,1]⊗[1,4,6,4,1]/256
    10: "CANNY",   # v8.1：完整边缘 Sobel->NMS->双阈值+滞后，输出 0/255
    11: "NMS",     # v8.1：只做非极大值抑制，输出细化后的幅值
}

# 3x3 算子（外一圈恒 0）与 5x5 算子（外两圈恒 0）的分界
MODE_5X5_FIRST = 9


GAIN_UNITY = 256     # Q8.8：256 = 1.0x（v7 起与 PL 的 AGC 口径一致）

# ---- Canny 双阈值打包进一个 16 bit 寄存器 ----
#   thresh = (HIGH << 8) | LOW
# Canny 原论文推荐 HIGH:LOW = 2:1 ~ 3:1，这里取 2:1。
# 可以由 ARM 每帧根据实测 σ 改写：噪声大 -> 阈值抬高，噪声小 -> 阈值降低。
CANNY_THRESH_DEFAULT = (60 << 8) | 30


def thresh_pack(hi, lo):
    return ((int(hi) & 0xFF) << 8) | (int(lo) & 0xFF)


def thresh_low(t):
    return int(t) & 0xFF


def thresh_high(t):
    return (int(t) >> 8) & 0xFF


# 5x5 二项式核 [1,4,6,4,1]
_B5 = (1, 4, 6, 4, 1)


def immerkaer_sigma_sum(gray):
    """PL 返回值 Σ|I⊛N| 的黄金参考（N = [[1,-2,1],[-2,4,-2],[1,-2,1]]）。

    与 PL 完全同式：窗口里存的是**过了 AGC** 的灰度，所以这里传进来的
    gray 也必须是 AGC 之后的。返回值是整数累加和，不是 σ；
    σ = sqrt(pi/2) * sum / (6 * (W-2) * (H-2))（见 sigma_from_sum）。
    """
    g = gray.astype(np.int32)
    p00 = g[:-2, :-2]
    p01 = g[:-2, 1:-1]
    p02 = g[:-2, 2:]
    p10 = g[1:-1, :-2]
    p11 = g[1:-1, 1:-1]
    p12 = g[1:-1, 2:]
    p20 = g[2:, :-2]
    p21 = g[2:, 1:-1]
    p22 = g[2:, 2:]
    corner4 = p00 + p02 + p20 + p22
    cross4 = p01 + p10 + p12 + p21
    lap = corner4 - 2 * cross4 + 4 * p11
    return int(np.abs(lap).sum())


def sigma_from_sum(sigma_sum, w, h):
    """把 PL 的累加和换算成 σ（Immerkær 1996）。

    σ = sqrt(pi/2) / (6*(W-2)*(H-2)) * Σ|I⊛N|
    分母的 6 = sqrt(Σ N²) = sqrt(1+4+1+4+16+4+1+4+1) = sqrt(36)
    sqrt(pi/2) 来自 E|X| = σ*sqrt(2/pi) 的反解
    """
    n = (w - 2) * (h - 2)
    if n <= 0:
        return 0.0
    return float(sigma_sum) * (np.pi / 2.0) ** 0.5 / (6.0 * n)


def process_bgr(frame, mode, gain=GAIN_UNITY, thresh=CANNY_THRESH_DEFAULT):
    if mode not in MODE_NAMES:
        raise ValueError("mode must be in %s, got %r"
                         % (sorted(MODE_NAMES), mode))
    if not isinstance(frame, np.ndarray) or frame.dtype != np.uint8:
        raise ValueError("input must be a uint8 BGR array")
    if frame.ndim != 3 or frame.shape[2] != 3:
        raise ValueError("input shape must be (height, width, 3)")
    h, w = frame.shape[:2]
    if not (1 <= w <= 640 and 1 <= h <= 480):
        raise ValueError("image must be within 640 x 480")

    # v6：灰度口径统一到 cv2.cvtColor。
    #     v5 的整式 (77R+150G+29B)>>8 是**向下取整**，OpenCV 是四舍五入，
    #     两者最多差 1 个灰度级 —— 这会让"逐像素最大差=0"直接不成立。
    #     既然 v6 的驱动侧已经改用 cvtColor，参考实现必须同源。
    if _HAS_CV2:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY).astype(np.int32)
    else:
        # 兜底：OpenCV 用浮点系数 0.299/0.587/0.114 取整，
        # 而整数式子 (77R+150G+29B)>>8 只是它的近似，两者最大差 1。
        # 所以只在**没有 cv2** 的机器上使用；有 cv2 时一律走上面那条。
        src = frame.astype(np.float32)
        gray = np.rint(0.299 * src[:, :, 2]
                       + 0.587 * src[:, :, 1]
                       + 0.114 * src[:, :, 0]).astype(np.int32)

    # v7：AGC 与 PL 完全同式 —— (x * gain) >> 8，再上限饱和。
    # gain=256 时 (x*256)>>8 == x，严格直通，所以 v6 的对拍结果不受影响。
    if gain != GAIN_UNITY:
        gray = np.clip((gray * int(gain)) >> 8, 0, 255)

    if mode == 0:
        return gray.astype(np.uint8)

    # ---- v8.2：融合流水线（op 9/10/11），最外**四**圈恒 0 ----
    # gray 已经过了 AGC，与 PL 的级1 输入一致。
    # 输入是原始灰度，去噪是链内级1 —— 不要再在外面先做一次高斯。
    if mode >= MODE_5X5_FIRST:
        gg = np.clip(gray, 0, 255)
        if h < 9 or w < 9:
            return np.zeros((h, w), dtype=np.uint8)
        if mode == 9:
            return gauss5_gray(gg)
        if mode == 11:
            return np.clip(nms5_gray(gg), 0, 255).astype(np.uint8)
        return canny5_gray(gg, thresh)

    out = np.zeros((h, w), dtype=np.uint8)
    if h < 3 or w < 3:
        return out

    g = gray
    p00 = g[:-2, :-2]
    p01 = g[:-2, 1:-1]
    p02 = g[:-2, 2:]
    p10 = g[1:-1, :-2]
    p11 = g[1:-1, 1:-1]
    p12 = g[1:-1, 2:]
    p20 = g[2:, :-2]
    p21 = g[2:, 1:-1]
    p22 = g[2:, 2:]

    if mode == 1:                                    # Sobel
        gx = (p02 + 2 * p12 + p22) - (p00 + 2 * p10 + p20)
        gy = (p20 + 2 * p21 + p22) - (p00 + 2 * p01 + p02)
        values = np.abs(gx) + np.abs(gy)

    elif mode == 8:                                  # v8：梯度方向量化
        gx = (p02 + 2 * p12 + p22) - (p00 + 2 * p10 + p20)
        gy = (p20 + 2 * p21 + p22) - (p00 + 2 * p01 + p02)
        ax = np.abs(gx)
        ay = np.abs(gy)
        # 与 PL 完全同式：用 2*|·| 比较代替 tan(22.5°)/tan(67.5°)，
        # 避免浮点，保证逐像素一致。
        d = np.where(2 * ay <= ax, 0,
            np.where(2 * ax <= ay, 2,
            np.where(gx * gy > 0, 1, 3)))
        values = d * 85

    elif mode == 2:                                  # Emboss
        values = p22 - p00 + 128

    elif mode == 3:                                  # Gaussian 3x3 / 16
        gw = (p00 + 2 * p01 + p02
              + 2 * p10 + 4 * p11 + 2 * p12
              + p20 + 2 * p21 + p22)
        values = gw >> 4

    elif mode == 4:                                  # Sharpen  [0 -1 0; -1 5 -1; 0 -1 0]
        values = 5 * p11 - (p01 + p10 + p12 + p21)

    elif mode == 5:                                  # Laplace  [-1 -1 -1; -1 8 -1; -1 -1 -1]
        values = 8 * p11 - (p00 + p01 + p02 + p10 + p12 + p20 + p21 + p22)

    elif mode == 6:                                  # Binary, threshold 128
        values = np.where(p11 >= 128, 255, 0)

    else:                                            # mode 7: Mean 3x3 / 9
        s9 = (p00 + p01 + p02 + p10 + p11 + p12 + p20 + p21 + p22)
        values = (s9 * 1821) >> 14

    out[1:-1, 1:-1] = np.clip(values, 0, 255).astype(np.uint8)
    return out


def gauss3_gray(gray):
    """3x3 高斯 [1 2 1;2 4 2;1 2 1]/16，输入/输出都是 1 B/px 灰度 uint8。

    级联高斯的积木：连跑两遍 == 5x5 二项式 [1,4,6,4,1]⊗[1,4,6,4,1]/256。
    边界语义与 process_bgr 完全一致（最外一圈恒 0），所以级联第二遍
    喂进去的图外圈是 0，结果与 process_bgr 对拍时可比。
    """
    g = gray.astype(np.int32)
    if g.shape[0] < 3 or g.shape[1] < 3:
        return np.zeros_like(gray)
    p00, p01, p02 = g[:-2, :-2], g[:-2, 1:-1], g[:-2, 2:]
    p10, p11, p12 = g[1:-1, :-2], g[1:-1, 1:-1], g[1:-1, 2:]
    p20, p21, p22 = g[2:, :-2], g[2:, 1:-1], g[2:, 2:]
    gw = (p00 + 2 * p01 + p02
          + 2 * p10 + 4 * p11 + 2 * p12
          + p20 + 2 * p21 + p22)
    out = np.zeros(g.shape, dtype=np.uint8)
    out[1:-1, 1:-1] = (gw >> 4).astype(np.uint8)
    return out


# ---------------------------------------------------------------------------
# v8.1：5x5 通路（op 9/10/11）的黄金参考
#
# 边界语义：最外**两**圈恒 0（凑不出 5x5 窗口），与 3x3 算子的一圈不同。
# 输入是**过了 AGC 的灰度**（与 PL 一致），不是 BGR。
# 这里的写法刻意与 PL 不同：PL 用常量下标宏 + 2 行循环缓冲，
#   这里用整幅切片 + numpy 向量化。两条独立路径算出同一结果才算数。
# ---------------------------------------------------------------------------

def _win5(g, i, j):
    """5x5 窗口的一个元素切片。

    窗口坐标 (i,j) ∈ [0,4]² 对应原图 (y-2+i, x-2+j)，中心区 (y,x) ∈
    **[4,H-5]×[4,W-5]**（v8.2 四级流水线，最外四圈恒 0）。
    返回形状 (H-8, W-8) 的数组，与中心区对齐。

    上界是 H-6+i 不是 H-4+i：中心最大 H-5，加偏移 i-2 得 H-7+i，
      切片上界 = H-6+i。写成 H-4+i 会多切两行，广播时形状对不上。
    """
    H, W = g.shape
    return g[2 + i:H - 6 + i, 2 + j:W - 6 + j]


def _sobel5_mag(win, r, c):
    """以窗口坐标 (r,c) 为中心的 Sobel 幅值 |Gx|+|Gy|，r,c ∈ [1,3]。"""
    gx = (win[r - 1][c + 1] + 2 * win[r][c + 1] + win[r + 1][c + 1]
          - win[r - 1][c - 1] - 2 * win[r][c - 1] - win[r + 1][c - 1])
    gy = (win[r + 1][c - 1] + 2 * win[r + 1][c] + win[r + 1][c + 1]
          - win[r - 1][c - 1] - 2 * win[r - 1][c] - win[r - 1][c + 1])
    return np.abs(gx) + np.abs(gy)


def _shift2d(a, k, axis):
    """把数组沿 axis 平移 k-2 位（中心对齐用），越界补 0（零填充）。"""
    off = k - 2
    if off == 0:
        return a
    out = np.zeros_like(a)
    n = a.shape[axis]
    if off > 0:
        src = [slice(None)] * a.ndim
        dst = [slice(None)] * a.ndim
        src[axis] = slice(0, n - off)
        dst[axis] = slice(off, n)
        out[tuple(dst)] = a[tuple(src)]
    else:
        src = [slice(None)] * a.ndim
        dst = [slice(None)] * a.ndim
        src[axis] = slice(-off, n)
        dst[axis] = slice(0, n + off)
        out[tuple(dst)] = a[tuple(src)]
    return out


def _vconv_gray(gray):
    """级1：5x5 可分离二项式高斯，零填充，**两级各 >>4**。

    一维核 [1,4,6,4,1] 系数和 = **16**，不是 256（二维才是 16×16=256）。
       PL 是横向 >>4、纵向 >>4 两级，中间结果被截成 8 bit。
       写成一次 >>8 会在噪声图上随机差 1 —— 这个精度损失是真实存在的。

    返回 int64 数组：行 [2, H-3] 有值（纵向卷积不越界），列 [0, W-1] 全有值
    （横向用零填充，所以左边"白赚"两列）。行 [0,1] 和 [H-2,H-1] 恒 0。
    """
    g = np.asarray(gray, dtype=np.int64)
    H, W = g.shape
    V = np.zeros((H, W), dtype=np.int64)
    if H < 9 or W < 9:
        return V
    T = np.zeros((H, W), dtype=np.int64)
    for k in range(5):
        T += _B5[k] * _shift2d(g, k, axis=1)
    T >>= 4                       # 横向归一化 /16 并截断
    for k in range(5):
        V += _B5[k] * _shift2d(T, k, axis=0)
    V >>= 4                       # 纵向归一化 /16
    V[:2, :] = 0                  # PL 只在行 [2, H-3] 产出 vconv
    V[H - 2:, :] = 0
    return V


def gauss5_gray(gray):
    """抽头 A（op 9）：级1 的输出，5x5 二项式高斯去噪图。

    有效区 行 [4,H-5] × 列 [4,W-5]，**最外四圈恒 0**
    （四级各延迟 2 行 2 列，累加 4 行 4 列）。
    """
    V = _vconv_gray(gray)
    H, W = V.shape
    out = np.zeros((H, W), dtype=np.uint8)
    if H < 9 or W < 9:
        return out
    out[4:H - 4, 4:W - 4] = (V[4:H - 4, 4:W - 4] & 0xFF).astype(np.uint8)
    return out


def nms5_gray(gray):
    """抽头 B（op 11）：级1 → 级2 → 级3，返回 NMS 细化后的幅值（int32）。

    输入是**原始灰度**，函数内部先跑级1 高斯再做 Sobel —— 这与 PL 的
      四级流水线一致：CANNY 模式的输入就是原始灰度，去噪是链内第一级。
      这是 v8.2 相对 v8.1 的关键变化（v8.1 的 Sobel 直接吃原始灰度）。

    9 个 Sobel 位置：中心 + 4 组对侧邻居（左右/上下/主对角/副对角）。
    这 9 个 3x3 全部落在同一个 5x5 窗口内，所以不需要 7x7。
    """
    V = _vconv_gray(gray)
    H, W = V.shape
    mag = np.zeros((H, W), dtype=np.int32)
    if H < 9 or W < 9:
        return mag
    win = [[_win5(V, i, j) for j in range(5)] for i in range(5)]

    mC = _sobel5_mag(win, 2, 2)
    gxc = (win[1][3] + 2 * win[2][3] + win[3][3]
           - win[1][1] - 2 * win[2][1] - win[3][1])
    gyc = (win[3][1] + 2 * win[3][2] + win[3][3]
           - win[1][1] - 2 * win[1][2] - win[1][3])
    ax = np.abs(gxc)
    ay = np.abs(gyc)
    # 与 PL 完全同式：2*|·| 比较代替 tan(22.5°)/tan(67.5°)，避免浮点
    d = np.where(2 * ay <= ax, 0,
                 np.where(2 * ax <= ay, 2,
                          np.where(gxc * gyc > 0, 1, 3)))

    mW, mE = _sobel5_mag(win, 2, 1), _sobel5_mag(win, 2, 3)
    mN, mS = _sobel5_mag(win, 1, 2), _sobel5_mag(win, 3, 2)
    mNW, mSE = _sobel5_mag(win, 1, 1), _sobel5_mag(win, 3, 3)
    mNE, mSW = _sobel5_mag(win, 1, 3), _sobel5_mag(win, 3, 1)

    n1 = np.where(d == 0, mW, np.where(d == 2, mN, np.where(d == 1, mNW, mNE)))
    n2 = np.where(d == 0, mE, np.where(d == 2, mS, np.where(d == 1, mSE, mSW)))

    # 有效区 行 [4,H-5] × 列 [4,W-5]，最外四圈恒 0
    mag[4:H - 4, 4:W - 4] = np.where((mC >= n1) & (mC >= n2), mC, 0)
    return mag


def canny5_gray(gray, thresh=CANNY_THRESH_DEFAULT):
    """抽头 C（op 10）：级1 → 级2 → 级3 → 级4，完整 Canny，输出 0/255。

    输入是**原始灰度**，去噪（级1）在链内完成 —— 与 PL 四级流水线一致。
    滞后必须**按 y 递增、x 递增**逐像素处理：右邻和下一行在流式
      顺序里还没算出来，所以邻域只查「上一行 3 邻 + 当前行左邻」。
      被提升的弱边会变成强边继续向右/向下传播，所以顺序不能乱。
    列 0..3 恒 0（最外四圈），所以左邻要 x-1 >= 4 才读，即 x >= 5。
    逐像素循环在 VGA 上要跑 30 万次（约 1~2 s）。validate 用 64x64，
      没有压力；要跑满幅别在实时循环里调它。
    """
    g = np.asarray(gray, dtype=np.int64)
    H, W = g.shape
    out = np.zeros((H, W), dtype=np.uint8)
    if H < 9 or W < 9:
        return out
    hi = thresh_high(thresh)
    lo = thresh_low(thresh)
    if hi < lo:
        hi = lo

    mag = nms5_gray(g).tolist()
    strong = [[0] * W for _ in range(H)]
    ol = [[0] * W for _ in range(H)]
    for y in range(4, H - 4):
        my = mag[y]
        sp = strong[y - 1]
        sy = strong[y]
        oy = ol[y]
        for x in range(4, W - 4):
            m = my[x]
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
    return np.array(ol, dtype=np.uint8)


class SoftwareProcessor:
    name = "PC SOFTWARE"

    def run(self, mode, frame, gain=GAIN_UNITY):
        return process_bgr(frame, mode, gain)
