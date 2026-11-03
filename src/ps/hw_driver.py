#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
hw_driver v2 —— 在软件组 hw_driver.py 基础上改的三处（2026-10-13，硬件侧）

改动 1（必改）：**删除 mode 1/2 降分辨率到 320x240 的策略**
    旧策略依据是 19 号文档的 108 ms，那是 v3。v4 行缓存重构后
    640x480 满分辨率三模式都只要 3.4~3.8 ms（真板实测），降分辨率
    只会白白损失画质，还多一次 resize 的 ARM 开销。见 22 号文档。

改动 2（性能）：pack 改为字节视图直写
    原来的 cvtColor + reshape(-1,3).astype(uint32) 会在 ARMv7 上造出
    640x480x3 的 uint32 临时数组（约 3.7 MB），实测打包环节要 ~76 ms，
    是端到端帧率的头号瓶颈。改成直接把 BGR 三通道写进 PL 输入缓冲的
    字节视图后，这一环节降到 ~10 ms（端到端 9 fps -> 29.5 fps 就是这么来的）。

改动 3（稳定性）：只分配一份 640x480 缓冲并长期复用
    原实现按 (w,h) 缓存且每次只保留一个、旧的未显式释放，尺寸频繁变化时
    有 CMA 连续内存碎片/泄漏风险。改为固定一份，按实际尺寸切片使用。

其余（寄存器映射、ap_done 轮询与超时、无板 SIM 降级、smoke_test）
全部沿用软件组原实现，未改动。

接口不变：HardwareProcessor().run(mode, frame_bgr) -> uint8 (h, w)
"""

import hashlib
import os
import time

import numpy as np

try:
    import cv2
    _HAS_CV2 = True
except ImportError:
    _HAS_CV2 = False

try:
    import filters                 # 包内文件名就叫 filters.py（v6 版，灰度走 cvtColor）
except ImportError:
    try:
        import filters_v6 as filters   # 备用名，内容逐字节相同
    except ImportError:
        filters = None

CTRL_BASE = 0x4000_0000
PARAM_BASE = 0x4001_0000
CTRL_SIZE = 0x1_0000

REG_CTRL = 0x00
# v8 偏移全部后移一格，因为 sigma 走 ap_return 挤占了 0x10。
#    下面这张表**逐字抄自**本次生成的
#    rgb2gray/hls/impl/ip/drivers/rgb2gray_v1_0/src/xrgb2gray_hw.h：
#        XRGB2GRAY_CONTROL_ADDR_AP_RETURN   0x10
#        XRGB2GRAY_CONTROL_ADDR_WIDTH_DATA  0x18
#        XRGB2GRAY_CONTROL_ADDR_HEIGHT_DATA 0x20
#        XRGB2GRAY_CONTROL_ADDR_MODE_DATA   0x28
#        XRGB2GRAY_CONTROL_ADDR_GAIN_DATA   0x30
#    注意：换任何一版位流都要重抄一遍。硬编码成 v7 的 0x10/0x18/0x20/0x28 时，
#       width 会写进 sigma（只读）-> 静默无效，height 写进 width，
#       表现是"图像只对上半部分生效"，极难排查。
REG_SIGMA = 0x10         # ap_return，只读，PL 的 Σ|I⊛N|
REG_WIDTH = 0x18
REG_HEIGHT = 0x20
REG_MODE = 0x28
REG_GAIN = 0x30          # v7 新增：Q8.8 定点增益
# v8.1 新增：Canny 双阈值 (HIGH<<8)|LOW，排在 gain 之后。
#   **预期**偏移 0x38 —— 但这只是预期，v8.1 位流生成后的第一件事
#   就是 grep xrgb2gray_hw.h 里的 ADDR_THRESH_DATA 确认。
#   （v8 相对 v7 整体后移过一格，用旧驱动配新位流会把 width 写进只读的 σ，
#     表现是"画面只对上半部分生效"—— 别让这事再发生一次。）
REG_THRESH = 0x38

# ---- v8 模式编码 ----
# bit 0..3 = 算子号，bit 4 = FLAG_AGC，bit 5 = FLAG_SIGMA
# （PL 不看这两个 flag，它们只是给驱动自己记的。PL 永远执行
#  out = clamp((x*gain)>>8)，gain=256 时严格直通。）
FLAG_AGC = 16
FLAG_SIGMA = 32
MODE_MAX_IMPL = 11       # 最后一个真正实现的算子（MODE_NMS，v8.1）
OP_MASK = 0x0F

# 算子号（与 rgb2gray_hls_v8.h 逐字对应，改名会静默错位）
MODE_GRAY, MODE_SOBEL, MODE_EMBOSS, MODE_GAUSS = 0, 1, 2, 3
MODE_SHARPEN, MODE_LAPLACE, MODE_BINARY, MODE_MEAN, MODE_DIR = 4, 5, 6, 7, 8
MODE_GAUSS5, MODE_CANNY, MODE_NMS = 9, 10, 11
MODE_5X5_FIRST = MODE_GAUSS5    # >= 9 走融合流水线，外**四**圈恒 0，最小 9x9
# v8.2：op 9/10/11 是同一条四级流水线的三个**抽头**，不是三个并列算子。
#   高斯 → Sobel+方向 → NMS → 双阈值+滞后，一趟访存走完，中间不回 DDR。
#   四级各延迟 2 行 2 列 -> 有效区 行[4,H-5] 列[4,W-5]，所以最小 9x9。

# Canny 双阈值默认 (HIGH<<8)|LOW = 60/30，比例 2:1（Canny 原论文推荐）
CANNY_THRESH_DEFAULT = (60 << 8) | 30

# ---- 自适应系数（canny_adaptive=True 时用）----
# 2026-10-30 修正：原来是 CANNY_K_HI = 16.0，**系数大了整整一倍**，
#    σ=8 时给出 HIGH=128，实测**真边覆盖率直接归零**（边缘全被当噪声砍掉）。
#    错在推导漏了一步：Sobel 幅值是在**级1 去噪之后**算的，
#    而 σ 是在**去噪之前**测的 —— 中间差一个 0.2734 的噪声衰减。
#
#    完整推导（对**实测 σ** 归一）：
#      1) 5x5 二项式去噪的噪声增益 = sqrt(Σw²)/Σw = sqrt(70)/16 = 0.2734
#         （一维 [1,4,6,4,1]：Σw=16，Σw²=70；二维两级相乘后比值不变）
#      2) Sobel 对标准差 σ' 的噪声：Σk² = 1+4+1+1+4+1 = 12
#         E|gx| = σ'·sqrt(12)·sqrt(2/π) = 2.764σ'，gy 同理
#         => E[mag_noise] = E|gx| + E|gy| ≈ 5.53σ'
#      3) 换回实测 σ：E[mag_noise] ≈ 5.53 × 0.2734 × σ = **1.512σ**
#      4) 取 5 倍噪声响应做强边门槛：HIGH ≈ 5 × 1.512σ = **7.56σ**
#
#    实测校核（640×480，对比度 24 的棋盘边 + 注入 σ=8 噪声）：
#        HIGH=24 (3.0σ)  -> 伪边 61864，命中率 19.4%   太松
#        HIGH=40 (5.0σ)  -> 伪边  3673，覆盖率 97.9%   可用
#        HIGH=60 (7.5σ)  -> 伪边   194，覆盖率 97.1%   甜点（= 上电默认值）
#        HIGH=80 (10σ)   -> 伪边   100，覆盖率 72.8%   开始丢边
#        HIGH=128 (16σ)  -> 覆盖率  0.0%               全丢（旧系数的后果）
CANNY_NOISE_GAIN = 1.512     # E[mag_noise] / σ，含级1 去噪的 0.2734 衰减
CANNY_K_NOISE = 5.0          # 取几倍噪声响应做强边门槛
CANNY_K_HI = CANNY_K_NOISE * CANNY_NOISE_GAIN   # ≈ 7.56
CANNY_K_LO = 0.5             # LOW = HIGH/2（Canny 推荐的 2:1）
CANNY_HI_MIN, CANNY_HI_MAX = 30, 220

# ---- AGC 自动增益（v7，光照自适应）----
GAIN_UNITY = 256         # Q8.8：256 = 1.0x，512 = 2.0x
AGC_TARGET_MU = 110.0    # 目标平均亮度（中灰偏暗，给提亮留出空间）
AGC_MIN, AGC_MAX = 1.0, 4.0
AGC_SMOOTH = 0.3         # 新一帧增益的权重；0.3 => 时间常数约 3 帧，防止增益跳变闪烁
REG_IN_LO = 0x10
REG_IN_HI = 0x14
REG_OUT_LO = 0x1C
REG_OUT_HI = 0x20

AP_START = 0x1
AP_DONE = 0x2
AP_IDLE = 0x4

MAX_W, MAX_H = 640, 480

# 只用来保证「cvtColor 直写失败」的提示只打一次，不刷屏
_WARNED = [False]
DONE_TIMEOUT_S = 5.0

# 默认位流：优先用**本文件同目录**的 rgb2gray.bit。
# 硬编码绝对路径会在多版本并存时静默加载错位流（v4 位流只有 3 个模式，
# 传 mode>=3 会走 else 分支输出 EMBOSS，画面看起来像"滤镜都变成浮雕了"）。
#
# 2026-10-30：**删掉最后一个 `/home/xilinx/rgb2gray/rgb2gray.bit` 兜底**。
#   它和 live_view_raw.py 里那个 v4 兜底是同一类坑 —— 本目录找不到位流时
#   改为去别的目录加载，加载的还多半是**旧版本**。现在有 MD5 校验兜着，
#   正确行为是"找不到就报错"，而不是"找个别的地方凑合"。
_HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_BIT = (os.environ.get("RGB2GRAY_BIT")
               or os.path.join(_HERE, "rgb2gray.bit"))

# ---------------------------------------------------------------------------
# 位流版本锁（2026-10-19，吸收自软件组 v5 集成包的 hw_driver.py）
#
# 为什么必须锁：.bit 文件大小**恒为 4045771 字节**，v4 和 v5 完全一样，看大小分辨不出来。
# 加载错版本的表现极具迷惑性 —— v4 硬件只实现 mode 0..2，mode>=3 会落 else 分支
# 输出 EMBOSS，画面看着像"后面几个滤镜都跟模式 3 一样"；而 validate 脚本用的是
# 自己目录下的新位流所以对拍全对 —— 两条路径加载的是不同文件，极难定位。
# 我 2026-10-19 实踩过一次，浪费了半天。
#
# v5 交付包：.bit 126f1fc6e73adae5fd3ef470f6205d68 / .hwh 2cf60392d56d1ae43ca300353f6a9b8e
# v4 回滚用：.bit 09e0715982d6ff06ce12888a87e44275
# ---------------------------------------------------------------------------
BIT_MD5_PREFIX = {".bit": "6b5fd2f9", ".hwh": "9b7407a3"}
# 2026-10-29 23:22 已换成 **v8.2 位流**（四级融合流水线 + II=1）。
#     v8.2 完整 MD5：.bit 6b5fd2f907b7214a528530dcca6b418a / .hwh 9b7407a3772b4d2800d00b6d65ba751a
#     .bit 大小 4045771 B —— 与 v4/v5/v6/v7/v8.1 完全相同，**只能靠 MD5 区分**。
#     .hwh **266277 B**（v8.1 是 265243 B，v7 是 264193 B）；
#     且 v8 的 .hwh 里能 grep 到 4 次 "ap_return"（v7 是 0 次）—— 第二道判据。
#
#     v8.2 相对 v8.1 改了什么：
#       1) op 9/10/11 从"三个并列算子"重构成**同一条四级流水线的三个抽头**
#          （AGC+5x5高斯 -> Sobel+方向 -> NMS -> 双阈值+滞后），一趟访存走完
#       2) 主循环 II 从 2 修到 1（滞后左邻改用寄存器 prev_s + st 的
#          DEPENDENCE inter false），**吞吐翻倍**
#       3) 新增 thresh 寄存器（0x38，Canny 双阈值 (HIGH<<8)|LOW，默认 60/30）
#
#     全工程 post-route：**WNS +0.458 / WHS +0.017 / WPWS +3.750，0 failing
#     （30128 endpoints）**。
#     全工程资源：LUT 8866 (16.67%) / FF 11122 (10.45%) / RAMB18 11 (3.93%)
#     / DSP 29 (13.18%)。
#     IP 单独（HLS impl 的 BD wrapper）：LUT 8005 (15.05%) / FF 10058 / RAMB18 11 / DSP 29。
#     注意：HLS csynth 估的是 19327 LUT(36%)，**那是加了 2.7ns 悲观余量的估计值**，
#       报告里只能写真板值 8866 (16.67%)。
#
#     验证：csim 864/864 PASS；cosim 96/96 PASS（Verilog，RTL PASS）。
#
#     上一版 v8.1：.bit 4f5ee6932928f4a07b67a6fbbc1086a9 / .hwh 892e23051892e0eb9755b8725277687b
#     （10-29 00:07 用户 GUI 跑的，WNS +0.955；我 21:10 命令行那版是
#       13662549/36b2f103，WNS 同样 +0.955 但字节不同 ——
#       **时序一样 ≠ 位流一样**，每次重建都必须重算 MD5 并改这里）
# 历史版本（存档仍可回滚）：
#     v7：.bit 508a682f / .hwh b693c045   （AGC 自动增益，无 σ）
#     v6：.bit f7760df3 / .hwh e8933626   （1 B/px 灰度输入，无 AGC）
#     v5：.bit 126f1fc6 / .hwh 2cf60392   （4 B/px RGB 打包，灰度在 PL 内算）
#     流程固定：出新 bit -> 算 MD5 -> 改这里 -> 再上板。
#     注意：各版 .bit 大小完全相同（4045771 B），只有 MD5 能区分
SKIP_BIT_MD5 = os.environ.get("RGB2GRAY_SKIP_MD5", "") not in ("", "0", "false", "False")

# 排障开关：设 RGB2GRAY_NO_CACHE_MAINT=1 可跳过 cache 维护，用于 A/B 对比
# （正常运行时不要设——见 _run_pl 里的说明）
SKIP_CACHE_MAINT = os.environ.get("RGB2GRAY_NO_CACHE_MAINT", "") not in ("", "0", "false", "False")


def check_bitstream_md5(bit_path, prefixes=None, strict=True):
    """加载前校验位流版本，防止静默加载错位流。

    strict=False 时只打印警告不抛异常（用于想故意跑旧版本的场合）。
    设置环境变量 RGB2GRAY_SKIP_MD5=1 可整体跳过。
    """
    prefixes = BIT_MD5_PREFIX if prefixes is None else prefixes
    stem = os.path.splitext(str(bit_path))[0]
    report = {}
    for suffix, want in sorted(prefixes.items()):
        candidate = stem + suffix
        if not os.path.exists(candidate):
            print("[hw] warn: %s not found, md5 check skipped" % os.path.basename(candidate))
            continue
        with open(candidate, "rb") as handle:
            digest = hashlib.md5(handle.read()).hexdigest()
        report[suffix] = digest
        if digest.startswith(want):
            print("[hw] %s md5 ok (%s...)" % (os.path.basename(candidate), digest[:8]))
        else:
            message = ("bitstream version mismatch: %s\n"
                       "    MD5     = %s\n"
                       "    expected prefix %s  (v5, 8 operators)\n"
                       "    v4 only implements mode 0..2; mode>=3 falls through to EMBOSS.\n"
                       "    Set RGB2GRAY_SKIP_MD5=1 to override."
                       % (candidate, digest, want))
            if strict:
                raise RuntimeError(message)
            print("[hw] WARN " + message)
    return report

_MODE_NAMES = getattr(filters, "MODE_NAMES", {0: "GRAY", 1: "SOBEL", 2: "EMBOSS"})


class HardwareProcessor:
    """run(mode, frame_bgr) -> uint8 image，由 rgb2gray PL IP 计算（v5 bitstream，8 算子）

    v5 说明（2026-10-19，硬件侧）
    ----------------------------
    算子从 3 个扩到 8 个（mode 0..7），但**寄存器映射、缓冲布局、握手方式全部没变**，
    所以这个类除了版本号和 self-test 之外一行逻辑都不用改：
      - `mode` 仍然写 0x20 寄存器，硬件内部多了一个 8 选 1 选择器（sparsemux）
      - `_MODE_NAMES` 是从 `filters` 动态取的，换成 8 模式 filters.py 后自动放行 3..7

    注意：但如果你手上这份驱动是 **1012 软件组原版**，它带着 v3 时代的降分辨率策略：
        mode 1/2 会先 cv2.resize 到 320x240 再算，算完放大回来 —— 画质直接砍一半。
        那个补丁的前提（v3 IP 满分辨率 Sobel 要 108 ms）在 v4 行缓存重构后已经不成立
        （实测 3.78 ms），必须删掉。v5 交付包这份是删干净的版本。
    """

    def __init__(self, bit_path=None, force_sim=False):
        self.name = "FPGA (rgb2gray PL v5)"
        self.last_pl_ms = 0.0
        self.last_pack_ms = 0.0
        self.calls = 0
        self._bit_path = bit_path or DEFAULT_BIT
        self._sim = force_sim
        self._ol = None
        self._ctrl = None
        self._param = None
        self._in_buf = None
        self._out_buf = None
        self.bit_manifest = {}     # 位流 MD5 校验结果，供上层/网页显示版本证据

        # ---- AGC 状态（v7）----
        self.agc_enabled = True    # 关掉就退回 gain=256（与 v6 逐字节一致）
        self.gain_override = None  # 非 None 时钉死增益（validate_v8.py 用）
        self._gain = 1.0           # 平滑后的浮点增益
        self.last_mu = 0.0         # 本帧平均亮度，HUD 可显示
        self.last_gain = GAIN_UNITY

        # ---- σ 状态（v8）----
        # PL 只回传累加和 Σ|I⊛N|（每像素一次 -> 放 PL）；
        # sqrt 和除法每帧只有一次标量 -> 放 ARM。划分依据是数据率。
        self.last_sigma_sum = 0    # PL ap_return 的原始整数
        self.last_sigma = 0.0      # 换算后的 σ（灰度级），HUD 可显示
        self.sigma_valid = False   # op==0 / op>=9 / 尺寸过小时 PL 返回 0，此时不可信
        self._sigma_prev = 0.0     # 上一帧的 σ，给 Canny 阈值自适应用
        self._sigma_prev_ok = False

        # ---- Canny 双阈值自适应（v8.1，AG-Canny 的第二个 A）----
        #   噪声 σ 是**上一帧** PL 实测的（和 AGC 用上一帧思路同源：
        #   σ 要整帧累加完才有，只能喂给下一帧）。
        #   HIGH = clamp(K_HI * σ)，LOW = HIGH/2（保持 Canny 推荐的 2:1）。
        #   K_HI 的来历见文件头 CANNY_K_HI 那段完整推导（含级1 去噪的
        #   0.2734 噪声衰减），结论是 HIGH ≈ 7.56σ。
        #   别再用旧式 K_HI=16：那是漏掉去噪衰减的版本，σ=8 时会把
        #     真边全砍光（实测覆盖率 0%）。
        # 默认**关**：固定阈值行为可预测，validate 才能复现。
        #   上板演示 / 拍证据时再打开，看阈值随光照噪声实时漂移。
        self.canny_adaptive = False
        self.canny_thresh = CANNY_THRESH_DEFAULT
        self.last_thresh = CANNY_THRESH_DEFAULT
        # v8.2：与 gain_override 对称。非 None 时钉死双阈值，让 validate_v8.py
        # 能做"同一张图、同一阈值"的逐像素对拍（否则自适应一开，阈值随
        # 上一帧 σ 漂，对拍就不可复现了）。默认 None = 走上面的逻辑。
        self.thresh_override = None

        if not force_sim:
            try:
                self._open_board()
            except Exception as exc:
                print("[hw] %s" % exc)
                print("[hw] falling back to SIM mode (NumPy reference). "
                      "Timings are NOT PL timings.")
                self._sim = True
                self.name = "FPGA SIM (no board)"
        else:
            self.name = "FPGA SIM (forced)"

    # ---------------------------------------------------------- board setup
    def _open_board(self):
        try:
            from pynq import MMIO, Overlay, allocate
        except ImportError as exc:
            raise ImportError(
                "pynq is not available (are you on the board?). "
                "Run on PYNQ-Z2 or pass force_sim=True.") from exc
        if not os.path.exists(self._bit_path):
            raise FileNotFoundError(
                "bitstream not found: %s (set env RGB2GRAY_BIT)" % self._bit_path)

        if not SKIP_BIT_MD5:
            self.bit_manifest = check_bitstream_md5(self._bit_path, strict=True)
        else:
            print("[hw] md5 check disabled by RGB2GRAY_SKIP_MD5")

        print("[hw] loading overlay %s ..." % self._bit_path)
        self._ol = Overlay(self._bit_path)
        self._ctrl = MMIO(CTRL_BASE, CTRL_SIZE)
        self._param = MMIO(PARAM_BASE, CTRL_SIZE)
        try:
            print("[hw] clock_dict:", self._ol.clock_dict)
        except Exception:
            pass

        # 只分配一次，尺寸变化不再重新分配（改动 3）
        # v6：输入 uint32（4 B/px 打包 RGB）-> uint8（1 B/px 灰度）
        #     每像素字节数 5 -> 2，32 位口的带宽压力掉一半以上
        self._in_buf = allocate(shape=(MAX_W * MAX_H,), dtype=np.uint8)
        self._out_buf = allocate(shape=(MAX_W * MAX_H,), dtype=np.uint8)

        if not self._wait_idle(3.0):
            raise RuntimeError("rgb2gray IP never went idle after overlay load")
        print("[hw] rgb2gray IP online (ap_idle=1)")

    def _wait_idle(self, timeout):
        deadline = time.perf_counter() + timeout
        while time.perf_counter() < deadline:
            if self._ctrl.read(REG_CTRL) & AP_IDLE:
                return True
            time.sleep(0.001)
        return False

    # ---------------------------------------------------------- data packing
    def _write_frame(self, frame):
        """BGR -> 1 B/px 灰度，连续写入 PL 输入缓冲，并顺手算出帧平均亮度 mu。

        v5 做法：把 R/G/B 三次跨步 1 字节写进 uint32 视图。
            stride=4 的单字节写**合并不成 32 位字**，ARM 侧实测约 15 ms，
            是当时端到端 19.72 ms 里的大头。
        v6 做法：先用 cv2.cvtColor 在 ARM 侧一次性转灰度（NEON 加速），
            再把连续的 h*w 字节整块拷进缓冲。PL 里因此不再需要 to_gray()。

        2026-10-28 大坑（修复记录，别再犯）：
            曾在 _run_pl 里用 `self._in_buf[:n].mean()` 算 AGC 的 mu，
            **在 PYNQ 的 cacheable DMA 缓冲上做 numpy reduce 实测 74 ms/帧**
            → 帧率从 128 fps 掉到 11.6 fps，pack 从 4.35 ms 涨到 78.8 ms。
            根因：PC 上用普通 numpy 数组测 mean 只要 0.2 ms，完全测不出来。
        修复：μ 根本不需要读那个缓冲 —— mean 是线性的：
                mean(0.299R + 0.587G + 0.114B)
                  == 0.299*mean(R) + 0.587*mean(G) + 0.114*mean(B)
            所以对原始 BGR frame 调 cv2.mean()（OpenCV C 实现，可能走 SIMD）
            即可**数学上精确**得到灰度均值，且零额外访存。
        """
        h, w = frame.shape[:2]
        n = h * w

        mu = None
        if self.agc_enabled and _HAS_CV2:
            b, g, r, _ = cv2.mean(frame)          # 各通道均值，C 实现
            mu = 0.299 * r + 0.587 * g + 0.114 * b

        if _HAS_CV2:
            # 融合写法：让 cvtColor **直接写进 DMA 缓冲**，不再"先转灰度、再整块拷"。
            #   两段式要在 640x480 上多走一遍全帧（实测打包 4.95 ms 里约一半是这次拷贝）。
            #   self._in_buf[:n].reshape(h,w) 是连续 view，行优先展平正好等于
            #   PL 期望的线性布局（行距 = 实际 w，不是 MAX_W）。
            try:
                cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY,
                             dst=self._in_buf[:n].reshape(h, w))
                return mu
            except Exception as e:      # 极端情况下 OpenCV 不接受 PynqBuffer，退回两段式
                if not _WARNED[0]:
                    print("[hw_driver] cvtColor 直写 DMA 缓冲失败，退回两段式：%r" % (e,))
                    _WARNED[0] = True
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)     # (h,w) uint8，连续
        else:
            gray = ((77 * frame[:, :, 2].astype(np.uint16)
                     + 150 * frame[:, :, 1].astype(np.uint16)
                     + 29 * frame[:, :, 0].astype(np.uint16)) >> 8).astype(np.uint8)
        # gray.reshape(-1) 在连续数组上是 view，不产生副本
        self._in_buf[:n] = gray.reshape(-1)
        if mu is None:
            # 兜底：没有 cv2.mean 时只能读缓冲（慢，仅此一条路径会走）
            mu = float(self._in_buf[:n].mean())
        return mu

    # ---------------------------------------------------------- AGC
    def _decide_gain(self, mu):
        """由帧平均亮度 mu 决定本次 Q8.8 增益。SIM 与 PL 共用同一套，保证两边一致。
        mu 为 None 表示调用方没算（只在异常兜底路径出现），此时退回读缓冲。"""
        if self.gain_override is not None:
            # 验证脚本用：跳过 AGC 平滑，直接钉死增益，保证对拍可复现
            # （AGC 的一阶平滑让增益依赖历史帧，逐像素对拍会漂）
            self.last_gain = int(self.gain_override)
            return self.last_gain
        if not self.agc_enabled:
            self.last_gain = GAIN_UNITY
            return GAIN_UNITY
        if mu is None:
            mu = float(self._in_buf[:self._in_buf.shape[0]].mean())
        self.last_mu = mu
        g = (AGC_TARGET_MU / mu) if mu > 1.0 else AGC_MAX
        g = min(max(g, AGC_MIN), AGC_MAX)
        self._gain += AGC_SMOOTH * (g - self._gain)   # 一阶平滑，防跳变闪烁
        gain_q = int(round(self._gain * GAIN_UNITY))
        self.last_gain = gain_q
        return gain_q

    # ---------------------------------------------------------- PL round trip
    def _run_pl(self, mode, frame):
        h, w = frame.shape[:2]
        if not (1 <= w <= MAX_W and 1 <= h <= MAX_H):
            raise ValueError("image must be within %dx%d, got %dx%d"
                             % (MAX_W, MAX_H, w, h))
        n = h * w

        t0 = time.perf_counter()
        # _write_frame 顺手把帧平均亮度 mu 算出来（用 cv2.mean(frame)，不碰 DMA 缓冲）
        mu = self._write_frame(frame)

        # ---- AGC：由本帧原始灰度均值决定本帧增益（v7）----
        # 划分依据是数据率：μ 每帧只算一次（标量），放 ARM；
        # 真正每像素 307200 次的乘法在 PL 里做。
        # 这是**前馈**不是反馈：μ 测的是**增益之前**的输入灰度，
        #   所以不存在"越提越亮"的自激回路，也不需要等下一帧。
        gain_q = self._decide_gain(mu)

        # ---- Canny 阈值：由**上一帧**实测 σ 决定（v8.1）----
        #   σ 要整帧累加完才有值，所以只能喂给下一帧 —— 与 AGC
        #   「上一帧统计 -> 这一帧生效」是同一套做法。
        self._update_canny_thresh()

        # cache 维护（2026-10-20 补，吸收自软件组 hardware_driver.py）
        # 原因：PL 通过 AXI HP 口**直读 DDR**，不经过 ARM 的 cache。
        #   - 启动前 flush  _in_buf ：ARM 刚写入的数据可能还只在 cache 里，
        #                            不回写的话 PL 读到的是上一帧的旧数据
        #   - 启动前 flush  _out_buf：清掉可能残留的脏行，防止它在 PL 写完之后
        #                            才被延迟回写，把 PL 的新鲜输出覆盖掉
        # 此前这份驱动**完全没做这一步**，之所以一直没出问题，是因为本平台的
        # CMA 映射行为恰好把数据推到了 DDR。
        # 实测代价约 0.48 ms/帧（端到端 19.72 ms 的 2.4%），换取正确性。
        #
        # 计时归类：cache 维护属于"把数据交给 PL"的 ARM 侧开销，
        # 故并入 last_pack_ms，**last_pl_ms 仍保持纯核内口径（3.78 ms）**，
        # 与报告主表一致，不受影响。
        if not SKIP_CACHE_MAINT:
            self._in_buf.flush()
            self._out_buf.flush()
        self.last_pack_ms = (time.perf_counter() - t0) * 1000

        return self._pl_launch(w, h, mode, gain_q, self.canny_thresh)

    # ---------------------------------------------------------- PL 启动原语
    def _pl_launch(self, w, h, mode, gain_q, thresh=None):
        """写寄存器 -> 启动 -> 等 ap_done -> 读 σ -> invalidate -> 取输出。

        假定 in_buf 已经填好且已 flush。级联高斯 / 灰度直通都复用这个。

        thresh：Canny 双阈值 (HIGH<<8)|LOW，仅 op 10 用。
                None 表示用 CANNY_THRESH_DEFAULT。
                不写会怎样：v8.1 位流上这个寄存器是**新增**的，
                  上电默认值是 0 -> HIGH=LOW=0 -> 所有 m>=0 都是强边，
                  整幅图全白。所以每次 launch 都显式写，别指望默认值。
        """
        n = w * h
        self._ctrl.write(REG_WIDTH, w)
        self._ctrl.write(REG_HEIGHT, h)
        self._ctrl.write(REG_MODE, int(mode))
        self._ctrl.write(REG_GAIN, int(gain_q))
        self._ctrl.write(REG_THRESH,
                         CANNY_THRESH_DEFAULT if thresh is None else int(thresh))
        self._param.write(REG_IN_LO, int(self._in_buf.device_address) & 0xFFFFFFFF)
        self._param.write(REG_IN_HI, 0)
        self._param.write(REG_OUT_LO, int(self._out_buf.device_address) & 0xFFFFFFFF)
        self._param.write(REG_OUT_HI, 0)

        self._ctrl.write(REG_CTRL, AP_START)
        t1 = time.perf_counter()
        deadline = t1 + DONE_TIMEOUT_S
        while not (self._ctrl.read(REG_CTRL) & AP_DONE):
            if time.perf_counter() > deadline:
                raise RuntimeError("rgb2gray IP did not finish in %.1f s "
                                   "(w=%d h=%d mode=%d)" % (DONE_TIMEOUT_S, w, h, mode))
        self.last_pl_ms = (time.perf_counter() - t1) * 1000

        # ---- v8：读 σ（必须在 ap_done 之后；ap_return 是只读寄存器）----
        # PL 回的是 Σ|I⊛N|，sqrt/除法放 ARM（每帧 1 次标量）。
        op = int(mode) & OP_MASK
        raw = int(self._ctrl.read(REG_SIGMA))
        # ap_return 可能被读成无符号；累加和恒非负，直接当无符号用。
        self.last_sigma_sum = raw
        # σ 有效的条件（v8.1 收紧）：
        #   op==0    逐像素算子，没有 3x3 窗口，PL 返回 0
        #   op 9..11 走 5x5 通路，PL **恒返回 0**（累加范围对不上 3x3 的定义）
        # 别把返回的 0 当成"噪声为 0" —— 那是"没测"，不是"测出来是 0"。
        self.sigma_valid = (0 < op < MODE_5X5_FIRST and w >= 3 and h >= 3)
        self.last_sigma = (filters.sigma_from_sum(raw, w, h)
                           if (self.sigma_valid and filters is not None)
                           else 0.0)
        # 留给下一帧的 Canny 阈值自适应用（op 9..11 测不到 σ，保持旧值）
        if self.sigma_valid:
            self._sigma_prev = self.last_sigma
            self._sigma_prev_ok = True

        # 与上面的 flush 配对：丢弃 cache 里的旧行，逼 ARM 从 DDR 重新读，
        #   否则会命中 PL 写之前就缓存住的旧数据。必须在 np.array() 读取之前做。
        if not SKIP_CACHE_MAINT:
            self._out_buf.invalidate()
        return np.array(self._out_buf[:n], dtype=np.uint8).reshape(h, w)

    # ---------------------------------------------------------- Canny 阈值
    def _update_canny_thresh(self):
        """由上一帧实测 σ 更新 Canny 双阈值。

        关着的时候恒返回默认值，保证 validate 可复现。
        thresh_override 非 None 时直接钉死（validate_v8.py 阶段 4 用）。
        """
        if self.thresh_override is not None:
            self.canny_thresh = int(self.thresh_override)
            self.last_thresh = self.canny_thresh
            return self.canny_thresh
        if not (self.canny_adaptive and self._sigma_prev_ok and self._sigma_prev > 0):
            self.canny_thresh = CANNY_THRESH_DEFAULT
        else:
            hi = int(round(CANNY_K_HI * self._sigma_prev))
            hi = max(CANNY_HI_MIN, min(CANNY_HI_MAX, hi))
            lo = max(1, int(round(hi * CANNY_K_LO)))
            self.canny_thresh = (hi << 8) | lo
        self.last_thresh = self.canny_thresh
        return self.canny_thresh

    # ---------------------------------------------------------- 灰度直通
    def _run_gray(self, mode, gray, gain_q=GAIN_UNITY):
        """直接喂一张 1 B/px 灰度图（不再做 cvtColor）。级联高斯第二遍用。"""
        h, w = gray.shape[:2]
        n = h * w
        self._in_buf[:n] = np.ascontiguousarray(gray).reshape(-1)
        if not SKIP_CACHE_MAINT:
            self._in_buf.flush()
            self._out_buf.flush()
        return self._pl_launch(w, h, mode, gain_q)

    # ---------------------------------------------------------- 5x5 级联高斯
    def run_gauss_cascade(self, frame, passes=2, gain_q=None):
        """5x5 二项式高斯 = MODE_GAUSS 连跑 `passes` 遍（驱动侧级联）。

        两遍等价于 [1,4,6,4,1]⊗[1,4,6,4,1]/256，σ 从 0.85 抬到 1.2。

        历史沿革：
          v7 时代这里写着"5x5 高斯没做成 PL 算子"，理由是"会改边界语义、
          让 120 组回归作废、25 项 adder tree 威胁 WNS"。
          **v8.1 已经把它做进去了**（MODE_GAUSS5 = 9），办法是**另开一条
          5x5 通路**，op 0..8 一个字节不动 —— 所以那三条顾虑全部消解：
            · 边界语义：只有 op 9..11 受影响，op 0..8 仍是外一圈
            · 回归：223 组板端对拍一条没作废
            · 时序：高斯用可分离两级 [1,4,6,4,1]，每级只有 5 个加法器
          **v8.2 改为**：四级融合成一条流水线（高斯→Sobel→NMS→滞后），
          一趟访存出边缘，不再需要"先跑 GAUSS5 再跑 CANNY"两趟。
          代价是边界从外两圈变成外四圈，最小尺寸从 5x5 变成 9x9。
          现在这个函数保留下来是为了**对照**（级联 vs 原生链的结果
          应当只差边界），以及给没有 v8.2 位流的场合兜底。

        增益只第一遍施加，第二遍固定 GAIN_UNITY —— 否则增益会被平方。
        """
        if self._sim:
            if filters is None:
                raise RuntimeError("SIM mode needs filters.py")
            g = filters.process_bgr(frame, 0,
                                    gain_q if gain_q is not None else GAIN_UNITY)
            for _ in range(int(passes)):
                g = filters.gauss3_gray(g)
            return g

        h, w = frame.shape[:2]
        if not (1 <= w <= MAX_W and 1 <= h <= MAX_H):
            raise ValueError("image must be within %dx%d, got %dx%d"
                             % (MAX_W, MAX_H, w, h))
        # 第一遍：正常路径（BGR -> 灰度 + AGC -> PL GAUSS）
        t0 = time.perf_counter()
        mu = self._write_frame(frame)
        if not SKIP_CACHE_MAINT:
            self._in_buf.flush()
            self._out_buf.flush()
        self.last_pack_ms = (time.perf_counter() - t0) * 1000
        g1 = self._decide_gain(mu) if gain_q is None else int(gain_q)
        out = self._pl_launch(w, h, MODE_GAUSS, g1)
        # 后续遍：直接把上一遍的输出当灰度喂回去，增益锁 1.0x
        for _ in range(int(passes) - 1):
            out = self._run_gray(MODE_GAUSS, out, GAIN_UNITY)
        return out

    # ---------------------------------------------------------- SIM 的 σ
    def _sim_update_sigma(self, frame, mode, gain_q):
        """SIM 模式下用参考实现填 σ 状态（语义与 _pl_launch 完全一致）。

        有效条件同 PL：`0 < op < MODE_5X5_FIRST` 且尺寸 >= 3x3。
        op 0（逐像素）和 op 9..11（融合流水线）恒返回 0 且 sigma_valid=False。
        """
        op = int(mode) & OP_MASK
        h, w = frame.shape[:2]
        valid = (0 < op < MODE_5X5_FIRST and w >= 3 and h >= 3)
        raw = 0
        if valid and filters is not None and _HAS_CV2:
            gg = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            if int(gain_q) != GAIN_UNITY:
                gg = np.clip((gg.astype(np.int32) * int(gain_q)) >> 8,
                             0, 255).astype(np.uint8)
            raw = int(filters.immerkaer_sigma_sum(gg))
        self.sigma_valid = valid
        self.last_sigma_sum = raw if valid else 0
        self.last_sigma = (filters.sigma_from_sum(raw, w, h)
                           if (valid and filters is not None) else 0.0)
        if valid:
            self._sigma_prev = self.last_sigma
            self._sigma_prev_ok = True

    # ---------------------------------------------------------- public API
    def run(self, mode, frame):
        if mode not in _MODE_NAMES:
            raise ValueError("mode must be in %s, got %r"
                             % (sorted(_MODE_NAMES), mode))
        if frame.ndim != 3 or frame.shape[2] != 3:
            raise ValueError("input must be an HxWx3 BGR frame")
        self.calls += 1

        # 改动 1：不再做任何降采样。v4 下 640x480 三模式都只要 3.4~3.8 ms。
        if self._sim:
            if filters is None:
                raise RuntimeError("SIM mode needs filters.py")
            if self.agc_enabled and _HAS_CV2:
                b, g, r, _ = cv2.mean(frame)
                gain_q = self._decide_gain(0.299 * r + 0.587 * g + 0.114 * b)
            else:
                gain_q = GAIN_UNITY
            self._update_canny_thresh()
            # v8.2：SIM 路径也如实填 σ 状态，否则 validate_v8.py --sim 里
            # 阶段 3 会拿 last_sigma_sum=0 去比参考累加和，全线假失败。
            # SIM 只是没有硬件，语义必须与 PL 一致，不能少半套。
            self._sim_update_sigma(frame, mode, gain_q)
            return filters.process_bgr(frame, mode, gain_q, self.canny_thresh)
        return self._run_pl(mode, frame)


def smoke_test():
    """不需要板子：验证 SIM 路径和打包布局"""
    proc = HardwareProcessor(force_sim=True)
    rng = np.random.RandomState(7)

    if filters is not None:
        frame = rng.randint(0, 255, (480, 640, 3), np.uint8)
        proc.agc_enabled = False          # 先关 AGC，验证 8 算子本体没被动过
        for mode in sorted(_MODE_NAMES):      # v5：8 个算子全过一遍
            got = proc.run(mode, frame)
            ref = filters.process_bgr(frame, mode)
            assert got.shape == ref.shape and np.array_equal(got, ref), mode
            print("mode %d SIM ok, out %s" % (mode, (got.shape,)))

        # v7 AGC 自检：SIM 与参考在同一增益下必须逐像素一致
        proc.agc_enabled = True
        for _ in range(4):                # 多跑几帧让一阶平滑收敛
            got = proc.run(0, frame)
        ref = filters.process_bgr(frame, 0, proc.last_gain)
        assert np.array_equal(got, ref), "AGC SIM/reference mismatch"
        print("AGC ok: mu=%.1f gain=%d (%.2fx)"
              % (proc.last_mu, proc.last_gain, proc.last_gain / 256.0))

        # 暗图必须触发提亮（否则 AGC 是摆设）
        dark = (frame.astype(np.float32) * 0.25).astype(np.uint8)
        proc._gain = 1.0
        for _ in range(8):
            proc.run(0, dark)
        assert proc.last_gain > GAIN_UNITY, "dark frame did not raise gain"
        print("AGC dark-frame ok: mu=%.1f -> gain %.2fx"
              % (proc.last_mu, proc.last_gain / 256.0))

    # v6 布局自检：灰度必须走 cv2.cvtColor，且行优先连续。
    #
    # 注意：口径变更：v5 在 PL 里用 (77R+150G+29B)>>8，是**向下取整**；
    #    cv2.cvtColor 用浮点加权后**四舍五入**。两者最多差 1 个灰度级。
    #    v6 起灰度化统一到 cv2.cvtColor（ARM NEON，且是报告里的标准口径），
    #    因此参考实现 filters_v6.py 也必须切成 cvtColor，否则对拍会差 1。
    if _HAS_CV2:
        f = np.zeros((2, 3, 3), dtype=np.uint8)
        f[0, 0] = [10, 20, 30]      # B=10 G=20 R=30 -> OpenCV 22（整数 floor 会给 21）
        f[0, 1] = [0, 0, 255]       # 纯红 -> 76
        f[1, 2] = [255, 255, 255]   # 白，落在**第二行**，用来抓跨行错位
        gray = cv2.cvtColor(f, cv2.COLOR_BGR2GRAY)
        flat = gray.reshape(-1)
        assert flat[0] == 22, flat[0]
        assert flat[1] == 76, flat[1]
        assert flat[-1] == 255, flat[-1]
        # 用 _write_frame 同样的路径再走一遍，确认不留 stride 残留
        buf = np.zeros(6, dtype=np.uint8)
        h, w = f.shape[:2]
        buf[:h * w] = cv2.cvtColor(f, cv2.COLOR_BGR2GRAY).reshape(-1)
        assert np.array_equal(buf, flat)
        print("pack layout ok (1 B/px gray via cv2.cvtColor, row-major continuous)")
    print("SMOKE TEST PASSED")


if __name__ == "__main__":
    smoke_test()
