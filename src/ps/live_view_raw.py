"""live_view_raw.py —— 满分辨率画面方案（一个像素都不降）

为什么不用 JPEG 了
------------------
上一版（live_view.py）为了省 CPU，先把画面缩小再压 JPEG，代价是浏览器里看到的是
降采样画面。本版换一条路，关键在"PL 输出是灰度单通道"：

    640x480 灰度 = 307 KB
    100 Mbps 网口实际吞吐 ~11 MB/s -> 传 307 KB 只要 ~27 ms
    而 ARM 把同一张图压成 JPEG 要 60~150 ms

    => 不压缩直接传，比压了再传还快 3~5 倍。

于是分工变成：
    * PL 结果   : 原始字节直推（/raw），浏览器用 canvas 渲染，640x480 满分辨率
    * 摄像头原图: 彩色 921 KB 太大（84 ms/帧），走低频 JPEG（/cam，默认 4 fps）做参考窗
    * HUD 文字  : HTML 叠加，板子完全不做"合成画面"这件 100 ms 的事

板子侧每帧只剩 memcpy + socket write，ARM 几乎不参与显示。

用法（板端）：
    source /usr/local/share/pynq-venv/bin/activate
    cd /home/xilinx/jupyter_notebooks/rgb2gray_v82
    python live_view_raw.py --hw

笔记本浏览器：
    http://192.168.2.99:8080/          实时画面
    /mode/0|1|2                        切滤镜模式
    /calib                             重新校准（换光照时用）
    /bench                             当场在同一帧上重跑一次 PL vs ARM 实测
    /engine/pl | /engine/sw            切换运算引擎（PL 硬件 / ARM 软件）
    /stats                             JSON 统计
    /stop                              停止

答辩用：网页底部那块 HARDWARE ACCELERATION EVIDENCE 是给评委看的硬件证据。
  - 左边 LIVE：启动时在当前帧上实测一次 ARM vs PL 的耗时与加速比，之后实时更新
  - 右边 MEASURED ON BOARD：validate_no_camera.py 的 20 次平均，与报告同口径
  - 当场点 ARM 按钮把引擎切回软件，PIPE 帧率立刻下跌 —— 数字之外最直观的证明
"""
import argparse
import json
import os
import socket
import sys
import threading
import time
from collections import Counter
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np

from control import ModeLatch
import control as _control

# 手指数 -> 模式号。优先用 control.py 里软件组维护的 FINGER_MODE_MAP
# （P1 会把 1~5 指映射到 GRAY/SOBEL/CANNY/NMS/GAUSS5）；
# 他们那边还没落地就退回旧的 1/2/3 -> GRAY/SOBEL/EMBOSS，不阻塞启动。
FINGER_MODE_MAP = getattr(_control, "FINGER_MODE_MAP", {1: 0, 2: 1, 3: 2})
from filters import SoftwareProcessor, MODE_NAMES
from session import GestureSession, open_camera, read_frame

STATE = {"mode": None, "mode_cur": 0, "stop": False, "hw_name": None,
         "calib": False, "calib_msg": "not calibrated", "calib_box": True,
         "pipe": 0.0, "push": 0.0, "cam": 0.0,
         "filter_ms": 0.0, "g_ms": 0.0, "g_hz": 0.0, "fingers": None, "area": 0.0,
         "area_lo": 0.0, "area_hi": 0.0, "strict": 1.0, "reason": "",
         "cam_jpg": None, "cam_ver": 0, "mask_jpg": None, "mask_ver": 0,
         # ---------- 光照自适应 HUD（2026-10-31 新增）----------
         # 为什么必须加：本作品的头号创新是「光照自适应」，hw_driver 里
         #   last_gain / last_mu / last_sigma / last_thresh 全都算好了，
         #   但网页一个都不显示 —— 演示看起来就跟一排普通滤镜按钮没区别，
         #   评委看不出"关灯不掉准确率"。这条 HUD 把创新从报告搬到屏幕上。
         "gain_x": 1.0,        # AGC 增益（倍），last_gain / 256
         "mu": 0.0,            # 本帧平均亮度（增益前的输入）
         "sigma": 0.0,         # PL 实测噪声 σ（灰度级）
         "thr_hi": 0,          # Canny 高阈值（由 σ 推导）
         "thr_lo": 0,          # Canny 低阈值
         "agc_on": True,       # AGC 是否启用（关掉时 HUD 显示"固定增益"）
         # ---------- 硬件加速证据（网页右下角那个面板的数据源）----------
         "pack_ms": 0.0,        # ARM 侧把 BGR 打包成 uint32 的耗时
         "pl_kernel_ms": 0.0,   # AP_START -> AP_DONE，PL 内部计算耗时
         "arm_base_ms": 0.0,    # 同一帧上 ARM 参考实现的耗时（启动时实测一次）
         "speedup_live": 0.0,   # arm_base_ms / pl_kernel_ms，每帧更新
         "bench": [],           # 点 RUN BENCHMARK 后的三模式明细
         "diff_ms": 0.0,        # 差值视图的处理耗时（DIFF 打开时才有意义）
         "bench_state": "not run"}
LOCK = threading.Lock()

# 处理线程 -> 推流线程。output 是灰度 uint8；ver 用于让推流端只在有新帧时才发
HANDOFF = {"frame": None, "output": None, "ver": 0, "size": None}
FRESH = {"frame": None, "fid": 0}
FRESH_LOCK = threading.Lock()
FINGERS = {"value": None, "ms": 0.0, "hz": 0.0}
# 挥手切换（2026-10-21）。gesture_loop 只负责产生事件，process_loop 负责改模式，
# 两个线程之间只通过 SWIPE["pending"] 传一个布尔，不共享 tracker 状态。
SWIPE = {"dir": None, "id": 0, "pending": False}
# 挥手诊断：网页和日志里能看到"卡在哪一步"（建不起锚点 / 位移不够 / 冷却中）
SWIPE_DIAG = {"anchor": False, "points": 0, "dx": 0.0, "span": 0.0,
              "speed": 0.0, "blocked": False, "last_dir": None, "area": 0.0}
# 手势风格：False = 挥手换滤镜（默认），True = 1/2/3 指换滤镜。
# 两者不能同时生效 —— 挥手时手型必被数成 1 指，同时开会把模式反复拽回 GRAY。
AUTO = [False]
SWIPE_TRACKER = [None]    # None = swipe.py 不可用 / --no-swipe，挥手功能整体关闭
# 硬件驱动的全局引用。processor 是 process_loop() 的局部变量，do_GET() 够不着，
# 而 AGC / 阈值 / 增益这三个网页开关必须能改到驱动。
# 用列表容器而不直接存引用：PL/ARM 热切换时 processor 会被整个换掉（L715 附近），
#   存一份裸引用会变成旧的那个。两处赋值都要更新这里。
PROC = [None]

RAW_FPS = [25.0]          # 由 --raw-fps 设置；网口是 100 Mbps，别无脑往高调

# ---- 每条流只允许一个活跃连接（2026-11-03 加）----
# 每条 MJPEG / 原始字节流都是一个永不返回的 handler，各占一个线程。
# 浏览器刷新、或者反复点"AGC 对比 ON/OFF"（每次都是整页导航，会重开三条流），
# 旧连接如果没被干净关掉（半开 TCP、write 阻塞在对端已关的 socket 上），
# 那个线程就永久挂着，一条一条往上累积。双核 Cortex-A9 上十来条这种线程
# 就能把主循环饿死 —— 现场表现就是"越点越卡，最后整页不动"，
# 而且终端上没有任何报错，看起来像程序自己卡死。
# 做法：新连接进来把 generation +1，旧连接下一轮发现号过期就自己退出。
STREAM_GEN = {"raw": 0, "cam": 0, "std": 0, "mask": 0}


def _take_stream(key):
    STREAM_GEN[key] = STREAM_GEN.get(key, 0) + 1
    return STREAM_GEN[key]


def _stream_alive(key, gen):
    return (not STATE["stop"]) and STREAM_GEN.get(key) == gen
RAW_CLIENTS = [0]         # 当前 /raw 连接数（0 = 浏览器没连上，画面就是黑的）
# 校准黄框，坐标已换算到 640x480 显示尺寸；None = gesture 不可用时退化隐藏
CALIB_BOX = [None]
# 检测图（320x240）的像素数，用来把 --min-area 的比例换算成面积阈值
DETECT_PIX = [320 * 240]
# 当前肤色判定严格度（gesture.STRICT），可用 /strict/up|down|<数值> 现场调
STRICT = [1.0]
# 肤色掩膜可视化开关。默认关；打开后 /mask 推 320x240 的二值掩膜，
# 用来一眼看出"到底是肤色判定糊了，还是数手指数错了"——调正确率时最重要的诊断
MASK_ON = [False]

# 差值视图（搬自软件组 display.difference_view）：显示 |滤镜输出 − 灰度原图| ×3。
# 用途不是正确性校验，而是答辩时让人一眼看出"这个算子在干什么"——
# 比如 SOBEL 亮的是边缘、GAUSSIAN 几乎全黑、二值化只剩黑白色块。
# 代价：每帧一次 cvtColor + absdiff + 放大，ARM 上约 3~5 ms，比再跑一遍 PL 便宜。
DIFF_ON = [False]
DIFF_GAIN = 3

# ---------------------------------------------------------------- AGC 对比视图
# 为什么值得做：关灯/低照度下，普通 Canny 要么整片噪声、要么边缘全丢，
# AG-Canny 仍能出干净边缘 —— 这一屏就是"光照自适应"最直观的一击，
# 录视频和答辩都靠它。代价：每帧多两次 PL 调用，只在这个视图里付。
# ---------------------------------------------------------------- AGC 对比视图
# 开对比时页面变成 2x2：
#   ① 左上 ORIGINAL RGB（摄像头原画）
#   ② 右上 STANDARD CANNY（PL，AGC 关 + 固定阈值 60/30，就是普通 Canny）
#   ③ 左下 FPGA AG-CANNY（PL，AGC 开 + 按实测噪声 σ 自适应阈值）
#   ④ 右下 RUNTIME MONITOR（帧号 / 增益 / 噪声 σ / 两条路径的阈值 / PL 耗时）
# ②③ 同一帧、同一套硬件，只差"光照自适应"，这才是要突出的东西
# （不做 CPU 基准：那是比加速器，不是比自适应）。
CMP_ON = [False]
# 对比视图锁定的算子。开对比期间**不允许切滤镜**（按钮/手势/挥手都忽略），
# 否则左右两格在比的东西随时变，评委根本看不出在比什么。
CANNY_MODE = next((m for m, v in MODE_NAMES.items()
                   if str(v).upper() == "CANNY"), 10)
CMPBUF = {"a": None, "ver": 0}         # ②号窗格的普通 Canny 帧（process_loop 填）
CMPJPG = {"jpg": None, "ver": 0}       # 编码线程产出，供 /cmpstd 推流
CMPSUM = {"gain_std": 1.0, "thr_std": "60 / 30",
          "gain_ag": 1.0, "thr_ag": "60 / 30"}
# 对比视图里"每几帧插一次 op1（SOBEL）专门测噪声 σ"。
# 为什么必须插：hw_driver 里 sigma_valid = (0 < op < 9)，
# op 9/10/11 走 5x5 通路，PL 的 ap_return **恒返回 0**（不是"测出 0"，是"没测"）。
# 于是一直停在 CANNY 时 σ 永远是 0、自适应阈值永远退化成默认 60/30，
# 右边和左边就长得一模一样 —— 对比视图等于白开。
# 插一次 op1 只要一次 PL 调用（约 3.7 ms），输出丢弃，只为把 σ 喂给下一帧的阈值。
SIGMA_INTERVAL = float(os.environ.get("SIGMA_INTERVAL", "1.0"))
_LAST_SIG_T = [0.0]
# ②号窗格（普通 Canny）多久才重算一次 —— 单位是**秒，不是帧数**。
# 为什么必须用时间而不是帧数：帧数节流下，帧率一跌，主循环变慢、
# 额外的 PL 调用在每帧耗时里的占比反而升高，形成"越慢越要算、越算越慢"
# 的正反馈，极端时主链被拖到肉眼完全静止（2026-11-03 现场）。
# 改成按时间节流后，额外开销恒定为每秒 1/STD_INTERVAL + 1/SIGMA_INTERVAL
# 次 PL 调用，与主链帧率彻底解耦。
STD_INTERVAL = float(os.environ.get("STD_INTERVAL", "0.12"))
# 想让 ②号窗格更跟手就把 STD_INTERVAL 调小（现场临时改：STD_INTERVAL=0.15 ./start_web.sh）；
# 直接设 CMPSTD=0 可以整块关掉 ② 号对照图的额外计算（排障用：关掉后画面若恢复，
# 就证明瓶颈在对照图这一遍 PL，而不是主链）。
CMPSTD_ON = os.environ.get("CMPSTD", "1") not in ("0", "no", "off", "")
# CMP_FIT=1：退回"完整画面"模式（object-fit:contain），4:3 一点不裁，
# 代价是格子会有黑边（笔记本 1080~1366 宽的屏上黑边能占到格子的一半）。
# 默认 0 = 铺满（cover）。
CMP_FIT = os.environ.get("CMP_FIT", "0") not in ("0", "no", "off", "")
_LAST_STD_T = [0.0]
_CMPSTD_FAILS = [0]
# ②号窗格（普通 Canny）每几帧才重算一次。
# 它是一张"对照图"，不是实时主画面，没有必要每帧都跑一遍 PL
# （每帧多一次 PL 调用 = 多约 14 ms，四画面时帧率直接掉一半）。
# 隔帧复用上一张，肉眼几乎看不出差别，帧率却回来了。
# 想让 ②号窗格更跟手就把 STD_EVERY 调小 —— 但这个常量已经不再参与节流判断，
# 真正生效的是 STD_INTERVAL（秒）。留着只是给老脚本/老文档兜底，别再用。
STD_EVERY = int(os.environ.get("STD_EVERY", "5"))
# σ 的经验修正系数。1.0 = 严格按"噪声随 AGC 线性放大"推导。
# 现场要是还嫌噪点多就往大调：SIGMA_K=2（或 3）启动 —— 每加一档，
# Canny 阈值整体抬高，噪点变少，但太弱的真实边缘也会被一起压掉。
# 调的时候盯着③号：噪点刚好消失、物体轮廓还在，就是这一档。
SIGMA_K = float(os.environ.get("SIGMA_K", "1.0"))
# AG-Canny 的 Canny **高阈值下限**，默认 80（对照组固定 60/30）。
# 为什么必须比对照组高一截：自适应的本分是"噪声大就把门槛抬高"，
# 如果算出来只跟固定的 60/30 打平（甚至更低），那③号的噪点就会**多于**②号，
# 演示直接翻车 —— 2026-11-03 现场就是这么翻的（AG 显示 50/30，满屏噪点）。
# 地板压到 80 之后：干净场景 AG=80/40、噪声场景按 σ 再往上抬，
# 于是③号的噪点永远明显少于②号，而②号在暗处会漏检 —— 对比才成立。
# 现场调：AG_HI_MIN=100 更狠（噪点更少但弱边会丢），AG_HI_MIN=60 退回打平。
AG_HI_MIN = float(os.environ.get("AG_HI_MIN", "80"))
# AGC 目标亮度，默认 110。房间太亮（μ >= 110）时 gain 恒为 1.00x、AGC 完全不工作，
# 对比视图就只剩阈值差别。现场不想摸黑拍的话把它调高，例如 AGC_TARGET=180，
# 让 AGC 在当前光照下就开始提亮 —— 演示的是"自适应机制"，不是"必须关灯"。
AGC_TARGET = os.environ.get("AGC_TARGET", "")


def _canny_k_hi(default=7.56):
    """从 hw_driver 取 CANNY_K_HI，取不到就用 7.56。

    不硬编码是因为这个系数推导过一次（含级1去噪 0.2734 衰减），
    改过一轮；写死在网页这边迟早会和驱动对不上。
    """
    try:
        mod = sys.modules.get("hw_driver") or __import__("hw_driver")
        return float(getattr(mod, "CANNY_K_HI", default) or default)
    except Exception:
        return default
_LAST_SIGMA = [0.0]      # 最近一次**有效**的 σ，避免 HUD 在 0 和数值之间跳

_push_n = 0
_push_t0 = time.perf_counter()

# ================================================================ 性能档案
# 8 个算子全部为真板实测（2026-10-19，v5 位流 126f1fc6...，
#   validate_v5.py 在 640x480 下 PL/ARM 各 20 次取平均，最大像素差全 0）。
# 口径 = ARM 参考实现均值 / PL kernel 均值，两者都**不含** ARM 侧打包开销。
#
# 口径红线：这套数和"摄像头脚本那套"（emboss 8.13x 之类）分母写法不同，
#   两套绝对不能混着说。网页上凡是标 "kernel" 的就是这套，标 "e2e" 的是含打包的。
#
# 关于和 v4 老数字的差异（GRAY 16.83 -> 15.27）：
#   PL 侧几乎没变（3.40 -> 3.391 ms，反而略快），变的是 **ARM 基线**：
#   v4 那次 gray 基线 57.20 ms，这次 51.78 ms（-9.5%）。同一份 NumPy 代码
#   在不同次运行间有 ±7~10% 的波动（内存状态、后台进程、CPU 调频）。
#   报告里请用**同一批次测出的这一组**，不要拿 v4 的分母配 v5 的分子，
#   并在表注里写明基线波动范围。
PERF = {
    0: {"name": "GRAY",    "arm_ms": 51.78,  "pl_ms": 3.391, "speedup": 15.27},
    1: {"name": "SOBEL",   "arm_ms": 193.29, "pl_ms": 3.784, "speedup": 51.09},
    2: {"name": "EMBOSS",  "arm_ms": 82.63,  "pl_ms": 3.784, "speedup": 21.84},
    3: {"name": "GAUSS",   "arm_ms": 185.93, "pl_ms": 3.783, "speedup": 49.14},
    4: {"name": "SHARPEN", "arm_ms": 106.41, "pl_ms": 3.783, "speedup": 28.13},
    5: {"name": "LAPLACE", "arm_ms": 139.50, "pl_ms": 3.776, "speedup": 36.95},
    6: {"name": "BINARY",  "arm_ms": 85.46,  "pl_ms": 3.775, "speedup": 22.64},
    7: {"name": "MEAN",    "arm_ms": 163.57, "pl_ms": 3.776, "speedup": 43.32},
}
# 8 个算子全部已有实测值，不再有 pending 项。保留变量名是为了兼容旧代码路径。
PERF_PENDING = []
# 一个很强的架构论点：7 个 3x3 算子的 PL 耗时几乎完全相同（3.775~3.784 ms，
# 相差 <0.3%）。因为主循环 II=1，每像素固定 1 个时钟，8 选 1 只改数据通路、
# 不改节奏 —— **算子复杂度与吞吐解耦**。答辩讲这个比只报加速比有力。
PERF_NOTE_PL_FLAT = True
# 器件 xc7z020clg400-1（PYNQ-Z2），v5 IP 级实现后数据（2026-10-19）
# 口径说明：这里是 **IP 本体** 的资源（与 v4 那组同口径，报告和网页都用这一套）。
#   全工程（含 PS7 + AXI 互联）是 LUT 4194 / FF 5518 / DSP 19 / BRAM 3，
#   WNS +1.415 ns @ 100 MHz —— 两个口径不要混着报，见报告 §4.3。
# v4 -> v5 增量：+143 LUT / +1 DSP / +0 BRAM，多 5 个算子的全部代价。
RESOURCES = {"LUT": 3299, "FF": 4480, "DSP": 19, "BRAM": 3}
# HLS 的约束目标是 8.000 ns（125 MHz），比 BD 实际给的 100 MHz(10 ns) 更严，
# 所以 IP 级 WNS +0.435 ns 已经很安全，系统上板跑 100 MHz 余量更宽。
TIMING = {"fclk_mhz": 100, "period_ns": 10.0,
          "post_route_ns": 7.565, "wns_ns": 0.435}
# 启动时实测得到的 ARM 基线（每模式 ms），用来算实时加速比。
# v5 改成按 MODE_NAMES 长度自动生成 —— 写死 3 个的话，切到 mode 3..7
#   这里会 IndexError（在 process_loop 线程里抛，现象是"点按钮没反应"）。
ARM_BASE = [None] * len(MODE_NAMES)
BENCH_REQ = [False]      # 网页点了 RUN BENCHMARK
BENCH_AUTO = [True]      # 启动后自动跑一次，这样评委一打开就有数
# 当前运算引擎。"pl" = PL 硬件 IP；"sw" = ARM 纯软件（NumPy 参考实现）。
# 可以在网页上当场切换 —— 切过去帧率立刻断崖，这是最直观的硬件加速证据：
# 同一路摄像头、同一条流水线，只是把"算滤镜"这件事从 ARM 挪到 PL。
ENGINE = ["pl"]
ENGINE_USED = [None]     # 当前 processor 实际对应的引擎，用来 detect 到"该换了"


def compute_calib_box():
    """把 gesture.py 的中央校准框换算到 640x480 的显示坐标。

    检测用的是 320x240 小图（DETECT_W/H），所以框要按 2 倍放大回画面。
    和 session.calibration_box() 用的是同一个 cal_frame_box()，位置一致。
    """
    try:
        import gesture as g
        DETECT_PIX[0] = int(g.DETECT_W) * int(g.DETECT_H)
        STRICT[0] = float(getattr(g, "STRICT", 1.0))
        small = np.zeros((g.DETECT_H, g.DETECT_W, 3), np.uint8)
        x1, y1, x2, y2 = g.cal_frame_box(small)
        sx, sy = 640.0 / g.DETECT_W, 480.0 / g.DETECT_H
        CALIB_BOX[0] = (int(x1 * sx), int(y1 * sy), int(x2 * sx), int(y2 * sy))
        print("[calib] box at %s (640x480 display coords)" % (CALIB_BOX[0],))
    except Exception as exc:
        print("[calib] box unavailable, will hide it:", exc)


# ------------------------------------------------------- 加速比实测（现场跑）
def _best_ms(fn, reps):
    """跑 reps 次返回最快一次。

    取最小值而不是平均值：板子上同时跑着采集/推流/手势三个线程，ARM 侧计时随时
    会被别的线程抢走几十毫秒，平均会把这些无关停顿算成算法耗时。最小值最接近
    算法本身，也和 validate_no_camera.py 里"预热后多次取优"的做法一致。
    """
    best = 1e9
    for _ in range(reps):
        t0 = time.perf_counter()
        fn()
        dt = (time.perf_counter() - t0) * 1000
        if dt < best:
            best = dt
    return best


def run_benchmark(processor, frame, reps=5):
    """在当前这一帧上，当场量一次 PL vs ARM，返回三个模式的明细。

    刻意给两个口径，因为答辩时最容易翻车的就是"你这数字怎么和报告不一样"：
      * pl_kernel_ms : AP_START 写到 AP_DONE 拉起，PL 核内部耗时
                       （与 validate_no_camera.py 同源，报告主口径）
      * e2e_ms       : processor.run() 全流程，多了 ARM 侧打包
                       （真实流水线里实际付出的代价，含 ~10 ms 打包）
    两个加速比都显示，各自标清楚来源。
    """
    sw = SoftwareProcessor()
    rows = []
    # 遍历参考实现里所有模式，不写死 3 个：v5 扩到 8 个算子后自动全测
    for m in sorted(MODE_NAMES):
        processor.run(m, frame)                      # 预热，避免首次缺页开销
        e2e = _best_ms(lambda: processor.run(m, frame), reps)
        pack = float(getattr(processor, "last_pack_ms", 0.0) or 0.0)
        pl_k = float(getattr(processor, "last_pl_ms", 0.0) or 0.0)
        if pl_k <= 0.0:
            # SIM 模式没有 AP_START/AP_DONE 可量，退化成整段耗时（两个口径相同）
            pl_k = e2e
        sw.run(m, frame)                             # ARM 侧同样预热
        arm = _best_ms(lambda: sw.run(m, frame), reps)
        rows.append({
            "mode": m, "name": MODE_NAMES[m],
            "arm_ms": round(arm, 2),
            "pl_kernel_ms": round(pl_k, 2),
            "pack_ms": round(pack, 2),
            "e2e_ms": round(e2e, 2),
            "speedup_kernel": round(arm / pl_k, 2) if pl_k > 0 else 0.0,
            "speedup_e2e": round(arm / e2e, 2) if e2e > 0 else 0.0,
        })
    return rows


PAGE_TEMPLATE = """<!doctype html><meta charset="utf-8">
<title>PYNQ-Z2 live</title>
<style>
*{box-sizing:border-box}
body{margin:0;background:#191817;color:#eee;font-family:ui-monospace,Consolas,monospace}
.bar{padding:8px 14px;font-size:13px;border-bottom:1px solid #333;display:flex;
     flex-wrap:wrap;gap:12px;align-items:center}
a{color:#7ddc8c;text-decoration:none;padding:2px 10px;border:1px solid #3a5c44;border-radius:6px}
a:hover{background:#243027}
a.on{background:#2f4a38;color:#c9f5d3;border-color:#6ee68c}
.cal{color:#ffd479;border-color:#5c4a2a}
.hint{color:#8a867e;font-size:12px}
.wrap{max-width:1300px;margin:0 auto;padding:8px 14px 18px}
.title{text-align:center;font-size:17px;font-weight:500;letter-spacing:1.5px;
       color:#fff;padding:8px 0 10px}
.grid{display:flex;gap:10px}
.pane{flex:1;min-width:0}
/* 对比视图：2x2 四窗格。非对比模式下 .cmpane 不显示，布局和原来完全一样。 */
.cmpane{display:none}
.cmpcap{color:#6ee68c}
/* ④ 运行时参数面板 */
.mon{border:1px solid #3a3a36;border-radius:6px;background:#141312;
     padding:6px 10px;font-size:13px;line-height:2.05}
.mon div{display:flex;justify-content:space-between;gap:10px;
         border-bottom:1px dashed #2c2b28}
.mon div:last-child{border-bottom:0}
.mon span{color:#8a867e}
.mon b{color:#7ecbff;font-weight:500}
.monnote{font-size:11px;color:#8a867e;padding:5px 2px 0}
/* 对比视图：2x2 四窗格，四格严格等大、整屏一屏装下（不出滚动条）。
   做法：body 变成纵向 flex 撑满 100vh，顶栏不吃掉多余高度，
   .grid 用 flex:1 吃掉剩余全部高度，两行两列各 1fr ——
   于是四格尺寸只由视口决定，与图片内容无关，天然等大。 */
body.cmp{overflow:hidden;height:100vh;display:flex;flex-direction:column}
/* 录屏/演示时这一整批都是纯装饰，高度必须全还给画面：
     .hud   底部 5~6 行状态文字（PIPE/PUSH/CLIENT/gesture/swipe）
     .title 标题
     .perf  HARDWARE ACCELERATION EVIDENCE 大表
     .pipe  融合流水线图
     .g     光照自适应表 + 硬件参数表（id=adap / id=hw）
   ④号 RUNTIME MONITOR 里 PIPE fps / gain / σ / 阈值 全都有，不重复。
   顶栏必须留着：THRESH AUTO 和 AGC 对比 OFF 两个按钮在上面。
     ⚠ 别只隐藏 .hud/.title —— 2026-11-03 就是漏了 .perf/.pipe/.g，
   结果底部堆了四大块把 grid 挤成一点点高，画面小到只有格子的 50%。 */
body.cmp .hud,body.cmp .title,
body.cmp .perf,body.cmp .pipe,body.cmp .g{display:none}
body.cmp .bar{flex:0 0 auto;font-size:11px;padding:2px 8px;gap:5px}
body.cmp .bar a{padding:0 6px}
body.cmp .hint{display:none}
body.cmp .wrap{flex:1 1 auto;min-height:0;max-width:none;padding:4px 8px 4px;
               display:flex;flex-direction:column}
/* minmax(0, 1fr) 而不是 1fr：1fr 展开是 minmax(auto, 1fr)，最小值取
   min-content，而 <canvas height="480"> 和 <img> 都有固有高度，会把所在那一行
   顶高 —— 实测上排 270 px / 下排 300 px 就是这么来的。
   写成 minmax(0, 1fr) 才真正等分，四格边框严格一样大。 */
body.cmp .grid{display:grid;gap:6px;flex:1 1 auto;min-height:0;
               grid-template-columns:repeat(2,minmax(0,1fr));
               grid-template-rows:repeat(2,minmax(0,1fr));
               align-items:stretch}
/* 默认铺满：格子占满各自那半屏，画面 object-fit:cover 填满格子。
   代价是 4:3 会被裁掉一点边（宽屏裁上下、方屏裁左右，两格对称裁，
   不影响左右对比）。演示要的就是"占满屏幕"，这是唯一能做到的办法 ——
   摄像头只有 640x480，contain 必然留黑边，而且留出来的黑边在
   1080~1366 宽的笔记本上能占到格子的一半。
   想看完整不裁的画面就用 CMP_FILL=0 启动。 */
body.cmp .pane{display:flex;flex-direction:column;min-width:0;min-height:0;
               overflow:hidden}
body.cmp .pane img,body.cmp .pane canvas{flex:1 1 0;min-height:0;min-width:0;
     width:100%;height:100%;object-fit:cover}
body.cmp.fit .pane img,body.cmp.fit .pane canvas{object-fit:contain}
body.cmp .mon{flex:1 1 0;min-height:0;overflow:auto;
              display:flex;flex-direction:column;justify-content:center}
body.cmp .cmpane{display:block}
body.cmp .cap{font-size:11.5px;padding:0 0 3px 2px}
.cap{font-size:12.5px;padding:0 0 5px 2px;color:#9a968e}
.cap.pl{color:#6ee68c;font-weight:500}
img,canvas{width:100%;display:block;background:#000;border-radius:6px}
.hud{padding:10px 2px 0;font-size:13px;line-height:1.9;color:#cfcbc4}
.hud b{color:#7ecbff;font-weight:500}
.hud .g{color:#6ee68c}
.fing{font-size:24px;color:#7ddc8c;font-weight:500}
/* ---- 硬件加速证据面板 ---- */
.perf{margin-top:14px;border:1px solid #3a3a36;border-radius:8px;overflow:hidden}
.perf .hd{padding:7px 12px;background:#242320;font-size:12.5px;letter-spacing:1px;
          color:#ffd479;border-bottom:1px solid #3a3a36}
.perf .bd{display:flex;gap:18px;padding:10px 12px 12px;flex-wrap:wrap}
.perf .col{flex:1;min-width:300px}
.perf .lb{font-size:11.5px;color:#8a867e;margin:0 0 7px;letter-spacing:.5px}
.pv{display:flex;align-items:center;gap:9px;font-size:12.5px;line-height:2}
.pv .k{width:38px;color:#9a968e}
.pv .t{flex:1;height:11px;background:#2c2b28;border-radius:3px;overflow:hidden}
.pv .f{display:block;height:100%;border-radius:3px}
.pv .v{width:78px;text-align:right;color:#cfcbc4}
.f.arm{background:#c96a5a}
.f.pl{background:#6ee68c}
.sp{font-size:30px;color:#6ee68c;font-weight:500;line-height:1.3;margin-top:2px}
.sp small{font-size:12.5px;color:#8a867e;font-weight:400}
table.pt{width:100%;border-collapse:collapse;font-size:12.5px}
table.pt th,table.pt td{padding:3px 6px;text-align:right;border-bottom:1px solid #2c2b28}
table.pt th{color:#8a867e;font-weight:400}
table.pt th:first-child,table.pt td:first-child{text-align:left}
table.pt .hi{color:#6ee68c;font-weight:500}
.rr{font-size:12px;color:#9a968e;line-height:1.85}
.rr b{color:#cfcbc4;font-weight:500}
.warn{color:#ff8f6a;font-size:12px;line-height:1.85;margin-bottom:6px;display:none}
/* ---- 2026-10-31 融合流水线可视化 ---- */
.pipe{margin:10px 18px;padding:9px 12px;background:#211f1d;border:1px solid #3a3633;border-radius:8px}
.ptitle{font-size:11px;color:#8b8279;letter-spacing:1px;margin-bottom:7px}
.pchain{display:flex;align-items:stretch;gap:5px;flex-wrap:wrap}
.pnode{padding:5px 9px;border-radius:6px;background:#171614;border:1px solid #302c29;
       color:#6b6259;font-size:12px;line-height:1.35;text-align:center;min-width:84px}
.pnode i{font-style:normal;display:block;font-size:10px;opacity:.8}
.pnode.on{background:#12332a;border-color:#3d8f6c;color:#8ff0c0}
.pnode.in,.pnode.out{background:#1e2420;border-color:#333c34;color:#a9b8ac}
.pnode.tap{box-shadow:inset 0 0 0 2px #ffd479}
.parr{align-self:center;color:#4a443e;font-size:13px}
.pmeta{margin-top:7px;font-size:11.5px;color:#8b8279;line-height:1.6}
.pmeta b{color:#eee;font-weight:400}
.ptap{color:#ffd479}
.fused{color:#6ee68c}
.single{color:#ffd479}
</style>
<body class="<!--CMPCLS-->">
<h2 style="position:absolute;left:-9999px">PYNQ-Z2 实时图像加速与手势交互演示画面</h2>
<div class="bar">
  <!--MODES-->
  <a class="cal" href="/calib">RECALIBRATE</a>
  <a class="cal" id="agon" href="/agc/on" title="AGC 自动增益开：暗处自动提亮（关灯演示就靠它对比）">AGC ON</a>
  <a class="cal" id="agoff" href="/agc/off" title="AGC 关：退回 gain=256，暗处不再提亮">AGC OFF</a>
  <a class="cal" href="/thresh/auto" title="Canny 阈值交回按实测噪声 sigma 自动计算">THRESH AUTO</a>
  <a class="cal" href="/strict/up" title="肤色判定更严：背景误检变少，但手可能检测不到">STRICT +</a>
  <a class="cal" href="/strict/down" title="肤色判定更松：手更容易检测到，但背景可能误检">STRICT -</a>
  <a class="cal" href="/mask/on" title="看肤色掩膜：整屏发白=阈值太松，该点 STRICT +">MASK ON</a>
  <a class="cal" href="/mask/off">MASK OFF</a>
  <a class="cal" id="gsw" href="/gesture/swipe" title="挥手换滤镜：手掌从右往左划 = 下一个模式，从左往右划 = 上一个">SWIPE</a>
  <a class="cal" id="gfi" href="/gesture/fingers" title="伸 1~5 根手指直选：GRAY / SOBEL / CANNY / NMS / GAUSS5（与挥手互斥）">FINGERS</a>
  <a class="cal" id="dfon" href="/diff/on" title="差值视图：显示 |滤镜输出 − 灰度原图| ×3，一眼看出这个算子在干什么">DIFF ON</a>
  <a class="cal" id="dfoff" href="/diff/off">DIFF OFF</a>
  <a class="cal" id="cmon" href="/cmp/on" title="2x2 对比：左上原画、右上 PL 输出、左下普通 Canny（AGC 关+固定阈值）、右下 AG-Canny（AGC 开+按噪声 sigma 自适应阈值）。开对比期间锁定 CANNY，不会切滤镜">
    AGC 对比 ON</a>
  <a class="cal" id="cmoff" href="/cmp/off" title="退回单画面">AGC 对比 OFF</a>
  <a class="cal" href="/bench" title="当场在同一帧上跑一次 PL vs ARM，刷新下面的加速比">
    RUN BENCHMARK</a>
  <a class="cal" id="epl" href="/engine/pl" title="用 PL 硬件 IP 算滤镜">PL</a>
  <a class="cal" id="esw" href="/engine/sw" title="改用 ARM 纯软件算滤镜：帧率会明显下跌，用来对比">ARM</a>
  <span class="hint">1~5 根手指直选 5 个模式（需先切到 FINGERS）；SWIPE 与 FINGERS 互斥。
    键盘：0~9 与 a/b 切模式 · p/s 切 PL/ARM · g 切 AGC · c 校准 · d 差值 · m 掩膜 · v 对比分屏</span>
</div>
<div class="wrap">
  <div class="title"><!--TITLE--></div>
  <div class="grid">
    <div class="pane"><div class="cap">① ORIGINAL RGB</div>
      <img id="cam" src="/cam"></div>
    <div class="pane cmpane"><div class="cap" id="capCPU">② STANDARD CANNY</div>
      <img id="cCPU" <!--SRC_STD-->></div>
    <div class="pane"><div class="cap pl" id="modecap">PL / -</div>
      <canvas id="cv" width="640" height="480"></canvas></div>
    <!-- ④ 运行时参数面板：把两条路径的实时数字摆在一起，录视频不用滚屏 -->
    <div class="pane cmpane"><div class="cap cmpcap">④ RUNTIME MONITOR</div>
      <div class="mon">
        <div><span>Frame ID</span><b id="mFid">-</b></div>
        <div><span>AGC Gain</span><b id="mGain">-</b></div>
        <div><span>Input μ (AGC 目标 110)</span><b id="mMu">-</b></div>
        <div><span>Noise σ</span><b id="mSig">-</b></div>
        <div><span>Threshold (AG)</span><b id="mThr">-</b></div>
        <div><span>PL Kernel</span><b id="mPl">-</b></div>
        <div><span>PIPE fps</span><b id="mFps">-</b></div>
        <div><span>Thr STD / AG</span><b id="mThr2">-</b></div>
      </div>
      <div class="monnote">同一帧 · 同一套硬件 · 只差光照自适应</div></div>
    <div class="pane" id="mkpane" style="display:none;max-width:330px"><div class="cap">SKIN MASK 320x240</div>
      <img id="mk" src="/mask"></div>
  </div>
  <div class="hud">
    <div><b>PIPE</b> <span id="pipe">-</span> fps &nbsp;
         <b>PUSH</b> <span id="push">-</span> fps &nbsp;
         <b>CLIENT</b> <span id="cfps">-</span> fps &nbsp;
         <b>FILTER(PL)</b> <span id="fms">-</span> ms</div>
    <div>gesture <span id="gms">-</span> ms @ <span id="ghz">-</span> Hz (async)
         &nbsp;|&nbsp; hand area <span id="area">-</span>
         &nbsp;|&nbsp; strict <span id="st">-</span>
         &nbsp;|&nbsp; fingers <span class="fing" id="fing">-</span>
         &nbsp;|&nbsp; <span id="rsn" style="color:#ffd479">-</span></div>
    <div>swipe <span id="sw" style="color:#9fd0ff">-</span></div>
    <!-- 2026-10-31 新增：光照自适应 HUD。
         关灯/开灯时这一行会实时变（增益↑、σ↑、Canny 阈值跟着抬），
         是本作品「光照自适应」在演示里唯一可见的证据。 -->
    <div class="g" id="adap">LIGHT-ADAPTIVE &mdash; AGC gain <span id="gain">-</span>&times;
         &nbsp;|&nbsp; input &mu; <span id="mu">-</span>
         &nbsp;|&nbsp; noise &sigma; <span id="sig">-</span>
         &nbsp;|&nbsp; Canny thr <span id="thr">-</span>
         <span id="agcoff" style="color:#ffd479"></span></div>
    <div class="g" id="hw">connecting...</div>
    <div id="hint" style="color:#ffd479;display:none">
      把掌心放进左侧画面的黄框 —— 校准成功后黄框自动消失</div>
  </div>

  <!-- 2026-10-31 融合流水线可视化。
       op 9/10/11 是同一条四级链的三个抽头，高亮的级数不同：
       op 9 亮到第 1 级、op 11 亮到第 3 级、op 10 全亮。
       一眼可看出它们不是三个并列算子，而是同一条链深浅不同的三个截面。 -->
  <div class="pipe" id="pipeline">
    <div class="ptitle">CURRENT PIPELINE &nbsp;&mdash;&nbsp; 当前处理链路</div>
    <div class="pchain">
      <span class="pnode in">输入<i>灰度 1 B/px</i></span>
      <span class="parr">&#9654;</span>
      <span class="pnode" id="st1">级 1 AGC+高斯<i>5&times;5 去噪</i></span>
      <span class="parr">&#9654;</span>
      <span class="pnode" id="st2">级 2 Sobel<i>+方向量化</i></span>
      <span class="parr">&#9654;</span>
      <span class="pnode" id="st3">级 3 NMS<i>非极大抑制</i></span>
      <span class="parr">&#9654;</span>
      <span class="pnode" id="st4">级 4 双阈值<i>+滞后</i></span>
      <span class="parr">&#9654;</span>
      <span class="pnode out">输出<i>特征向量</i></span>
    </div>
    <div class="pmeta">
      <span id="pmode">-</span><span class="ptap" id="ptap"></span>
      &nbsp;&nbsp;|&nbsp;&nbsp;核内 <b id="ppl">-</b> ms
      &nbsp;|&nbsp; LUT <b>16.21%</b> · FF <b>9.71%</b> · BRAM <b>5.36%</b> · DSP <b>14.55%</b>
      <span style="opacity:.65">（post-route 静态值，运行时不变）</span>
    </div>
  </div>

  <div class="perf">
    <div class="hd">HARDWARE ACCELERATION EVIDENCE &nbsp;·&nbsp; PYNQ-Z2 xc7z020clg400-1
      &nbsp;·&nbsp; FCLK 100 MHz</div>
    <div class="bd">
      <div class="col">
        <div id="simwarn" class="warn">SIM 模式：没有加载 PL 位流，下面是纯软件耗时，不是硬件加速比。</div>
        <div class="lb">LIVE &mdash; 当前模式 <span id="pname">-</span><br>
          <span style="color:#6b6862">整条流水线（采集+推流+手势）同时运行时测得，
          故 ARM 侧耗时高于右表的隔离测量</span></div>
        <div class="pv"><span class="k">ARM</span>
          <span class="t"><i class="f arm" id="ba" style="width:100%"></i></span>
          <span class="v" id="va">- ms</span></div>
        <div class="pv"><span class="k">PL</span>
          <span class="t"><i class="f pl" id="bp" style="width:2%"></i></span>
          <span class="v" id="vp">- ms</span></div>
        <div class="sp"><span id="spv">-</span>
          <small> &times; faster than ARM (kernel)</small></div>
        <div class="rr" id="brk">尚未实测</div>
        <div class="rr">端到端加速比 <b id="spe">-</b> &times;（含 ARM 侧打包）</div>
      </div>
      <div class="col">
        <div class="lb">MEASURED ON BOARD &mdash; v5 八模式
          （10-19 真板，各 20 次平均，报告主口径 · 40 组对拍最大差全 0）</div>
        <table class="pt">
          <tr><th>MODE</th><th>ARM (ms)</th><th>PL (ms)</th><th>SPEEDUP</th></tr>
          <tr><td>GRAY</td><td>51.78</td><td>3.391</td><td class="hi">15.27&times;</td></tr>
          <tr><td>SOBEL</td><td>193.29</td><td>3.784</td><td class="hi">51.09&times;</td></tr>
          <tr><td>EMBOSS</td><td>82.63</td><td>3.784</td><td class="hi">21.84&times;</td></tr>
          <tr><td>GAUSS</td><td>185.93</td><td>3.783</td><td class="hi">49.14&times;</td></tr>
          <tr><td>SHARPEN</td><td>106.41</td><td>3.783</td><td class="hi">28.13&times;</td></tr>
          <tr><td>LAPLACE</td><td>139.50</td><td>3.776</td><td class="hi">36.95&times;</td></tr>
          <tr><td>BINARY</td><td>85.46</td><td>3.775</td><td class="hi">22.64&times;</td></tr>
          <tr><td>MEAN</td><td>163.57</td><td>3.776</td><td class="hi">43.32&times;</td></tr>
        </table>
        <div class="rr">RESOURCE &nbsp;IP 本体（报告口径）LUT <b>3299</b> &middot; FF <b>4480</b>
          &middot; DSP <b>19</b> &middot; BRAM <b>3</b><br>
          &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;
          全工程（含 PS7+互联）LUT <b>4194</b> &middot; FF <b>5518</b> &middot;
          DSP <b>19</b> &middot; BRAM <b>3</b><br>
          TIMING &nbsp;&nbsp;WNS <b>+1.415 ns</b>（全工程）&nbsp;/&nbsp;
          <b>+0.435 ns</b>（IP 本体）@ 10.000 ns period</div>
      </div>
    </div>
  </div>
</div>
<script>
const cv=document.getElementById('cv'), ctx=cv.getContext('2d',{alpha:false});
let W=640,H=480,FS=W*H,img=null,d32=null,buf=null,have=0;
// 灰度->RGBA 查表：省掉每像素 3 次移位+2 次或。307200 像素/帧，这个循环很值
const LUT=new Uint32Array(256);
for(let v=0;v<256;v++) LUT[v]=0xFF000000|(v<<16)|(v<<8)|v;
// 客户端实际收到多少帧（对比板子 push，能一眼看出丢在网路上还是浏览器上）
let cf_n=0, cf_t0=performance.now(), cf_fps=0;

async function init(){
  try{ const m=await (await fetch('/meta')).json(); W=m.w; H=m.h; }catch(e){}
  FS=W*H; cv.width=W; cv.height=H;
  img=ctx.createImageData(W,H); d32=new Uint32Array(img.data.buffer);
  buf=new Uint8Array(FS*3); have=0;
  pump();
}
function draw(f){
  for(let i=0;i<FS;i++) d32[i]=LUT[f[i]];
  ctx.putImageData(img,0,0);
  cf_n++;
  const now=performance.now();
  if(now-cf_t0>=1000){ cf_fps=cf_n*1000/(now-cf_t0); cf_n=0; cf_t0=now;
    document.getElementById('cfps').textContent=cf_fps.toFixed(1); }
}
async function pump(){
  while(true){
    try{
      const res=await fetch('/raw');
      const rd=res.body.getReader();
      while(true){
        const {done,value}=await rd.read();
        if(done) break;
        if(have+value.length>buf.length){
          const nb=new Uint8Array(Math.max(have+value.length, FS*3));
          if(have) nb.set(buf.subarray(0,have),0);
          buf=nb;
        }
        buf.set(value,have); have+=value.length;
        if(have>=FS){
          // 关键：一次只消费「整数个 FS」，剩下的往前提。
          // 原来的写法是 draw(末尾FS字节) 然后 have=0 —— TCP 分片长度不固定，
          // 每次切的起点都在变，画面每行都错开不同的偏移（看着就是错位/抖动）。
          // 保持整数倍消费后，帧边界对齐一次建立就永不破坏。
          const take=Math.floor(have/FS)*FS;      // 本次可消费的整帧字节数
          draw(buf.subarray(take-FS,take));       // 画其中最新的一帧
          const rest=have-take;                   // 残余半帧留到下次
          if(rest) buf.copyWithin(0,take,have);
          have=rest;
        }
      }
    }catch(e){ await new Promise(r=>setTimeout(r,600)); }
  }
}
// 安全赋值：元素不存在就跳过，绝不因为一处 id 对不上而中断整个 HUD 更新。
// （踩过坑：删了 HTML 里的 awin 却忘了删 JS 引用，getElementById 返回 null，
//   一抛异常后面全不更新——表现为"手指数字不动、MASK 开关没反应"却没有任何报错）
let lastStats=null;   // 键盘快捷键要读当前状态（AGC/差值/掩膜开关），缓存一份
const set=(id,v)=>{const e=document.getElementById(id); if(e) e.textContent=v;};
const css=(id,prop,v)=>{const e=document.getElementById(id); if(e) e.style[prop]=v;};
function renderPerf(s){
  // bench 是启动时/点按钮时实测出来的三模式明细；没跑过就保持占位符
  const b=s.bench||[];
  // 没有加载 PL 位流时（SIM / 纯软件）明确警告：下面这些耗时不是硬件加速比
  const sw=document.getElementById('simwarn');
  const isSw = !s.hw || s.hw.indexOf('SIM')>=0 || s.hw.indexOf('SOFTWARE')>=0
               || s.hw.indexOf('no PL')>=0;
  if(sw) sw.style.display = isSw ? 'block' : 'none';
  // 高亮当前引擎按钮，评委一眼知道现在是 PL 还是 ARM 在算
  const epl=document.getElementById('epl'), esw=document.getElementById('esw');
  if(epl) epl.className = (s.engine_used==='pl') ? 'cal on' : 'cal';
  if(esw) esw.className = (s.engine_used==='sw') ? 'cal on' : 'cal';
  if(!b.length){ set('pname', s.mode_name||'-'); return; }
  const row=b.find(r=>r.mode===s.mode) || b[0];
  const a=row.arm_ms, k=row.pl_kernel_ms, e=row.e2e_ms;
  const mx=Math.max(a,e,0.01);
  css('ba','width',(a/mx*100).toFixed(1)+'%');
  css('bp','width',(Math.max(k,0.01)/mx*100).toFixed(1)+'%');
  set('pname', row.name);
  set('va', a.toFixed(1)+' ms');
  set('vp', k.toFixed(2)+' ms');
  // 优先用实时值（每帧都在算），没有就退回启动时那次的实测
  set('spv', s.speedup_live>0 ? s.speedup_live.toFixed(1) : row.speedup_kernel.toFixed(1));
  set('brk', 'pack '+row.pack_ms.toFixed(1)+' ms + kernel '+k.toFixed(2)+
             ' ms = e2e '+e.toFixed(1)+' ms');
  set('spe', row.speedup_e2e.toFixed(1));
}
async function stats(){
  try{
    const s=await (await fetch('/stats')).json();
    set('pipe', s.pipe.toFixed(1));
    set('push', s.push.toFixed(1));
    set('fms', s.filter_ms.toFixed(1));
    // 光照自适应 HUD：关灯时这三个数会一起动，是"自适应"最直观的证据
    set('gain', (s.gain_x==null?1:s.gain_x).toFixed(2));
    set('mu',   (s.mu==null?0:s.mu).toFixed(0));
    set('sig',  (s.sigma==null?0:s.sigma).toFixed(2));
    set('thr',  (s.thr_hi==null?0:s.thr_hi)+' / '+(s.thr_lo==null?0:s.thr_lo)
                + (s.agc_on ? '  (auto from σ)' : '  (fixed)'));
    set('agcoff', s.agc_on ? '' : '  AGC OFF');
    set('gms', s.gesture_ms.toFixed(1));
    set('ghz', s.gesture_hz.toFixed(1));
    set('area', Math.round(s.area));
    set('st', s.strict.toFixed(2));
    // reason 是软件组给的"为什么判没手"，现场调参全靠它
    const R={"hand_too_big":"背景太大片（调大 MAX_AREA_FRAC）",
             "too_solid":"色块太实心不像手（调大 MAX_SOLIDITY）",
             "too_smooth":"边缘太光滑没指缝（调小 MIN_ROUGH）",
             "too_ragged":"太破碎（调小 MIN_SOLIDITY）",
             "too_thin":"细长条（调小 MIN_ASPECT）",
             "hand_too_small":"手太小/太远（把手凑近点）",
             "no_skin":"画面里没有肤色（先按 c 校准）",
             "face_rejected":"被判成人脸跳过了"};
    set('rsn', R[s.reason]||s.reason||'-');
    const mk=document.getElementById('mkpane');
    if(mk) mk.style.display = s.mask_on ? 'block' : 'none';
    // 差值视图按钮高亮 + 标题栏注明（不然评委分不清看的是结果还是差值）
    const df=document.getElementById('dfon'), dfo=document.getElementById('dfoff');
    if(df) df.className = s.diff_on ? 'cal on' : 'cal';
    if(dfo) dfo.className = s.diff_on ? 'cal on' : 'cal';
    // AGC 对比视图按钮高亮：让评委一眼知道现在看的是分屏还是单画面
    const cm=document.getElementById('cmon'), cmo=document.getElementById('cmoff');
    if(cm) cm.className = s.cmp_on ? 'cal on' : 'cal';
    if(cmo) cmo.className = s.cmp_on ? 'cal on' : 'cal';
    // 对比两格的标题带上"这一格实际用上的增益和阈值"：
    // 光照不够暗时 AGC 增益会被夹在 1.00x、两格画面看着一样，
    // 这时标题上的数字就是唯一能证明"右边确实在自适应"的东西。
    const cA=document.getElementById('capCPU');
    if(cA) cA.textContent = '② STANDARD CANNY · gain '+(s.cmp_gain_std||1).toFixed(2)
        +'x · thr '+s.cmp_thr_std;
    // ④ 参数面板：把"自适应到底改了什么"直接摆成数字
    const mset=function(id,v){const e=document.getElementById(id); if(e) e.textContent=v;};
    mset('mFid', s.fid===undefined?'-':s.fid);
    mset('mGain', (s.gain_x||1).toFixed(2)+'x');
    // μ >= 110 时 AGC 只提亮不压暗，gain 会被夹在 1.00x —— 这时候两格 gain 一样，
    // 光照自适应无事可做，画面差别只剩阈值。把这一格标黄提醒"该遮光了"。
    var muEl=document.getElementById('mMu');
    if(muEl){ muEl.textContent=(s.mu||0).toFixed(1)+(s.mu>=110?'  ← 太亮':'');
              muEl.style.color=(s.mu>=110)?'#ffd479':'#7ecbff'; }
    mset('mSig', (s.sigma||0).toFixed(2));
    mset('mThr', s.thr_hi+' / '+s.thr_lo);
    mset('mPl', (s.pl_kernel_ms||0).toFixed(2)+' ms');
    mset('mFps', (s.pipe||0).toFixed(1));
    mset('mThr2', s.cmp_thr_std+'  /  '+s.cmp_thr_ag);
    // 手势风格按钮：互斥，高亮当前那个
    const gs=document.getElementById('gsw'), gf=document.getElementById('gfi');
    if(gs) gs.className = s.auto_control ? 'cal' : 'cal on';
    if(gf) gf.className = s.auto_control ? 'cal on' : 'cal';
    // 挥手诊断：anchor=0 就是"手没停稳，锚点没建起来"——挥不动时先看这个
    const w=s.swipe||{};
    const dxv=(w.dx>=0?'+':'')+(w.dx||0).toFixed(2);
    let st;
    if(w.blocked) st='cooldown（刚切过，等 ~0.8 s）';
    else if(!(w.area>0)) st='没检测到肤色块 — 先看 MASK ON，再重新校准';
    else if(!w.ready) st='手已在跟踪，等约 0.3 s 建立轨迹';
    else st='ready · 0.7 s 内已移动 '+dxv+'（阈值 0.16）· 左划 +1 / 右划 -1';
    // 方向约定：左挥 = 下一个模式（+1），右挥 = 上一个（-1）
    if(w.last_dir) st=(w.last_dir==='left'?'last 左挥 → 模式+1':'last 右挥 → 模式-1')+' · '+st;
    set('sw', st+'  [pts '+(w.points||0)+' · 跟踪面积 '+Math.round(w.area||0)+']');
    set('fing', s.fingers===null?'-':s.fingers);
    set('modecap', s.cmp_on
        ? ('③ FPGA AG-CANNY · gain '+(s.gain_x||1).toFixed(2)+'x · thr '
           +s.thr_hi+' / '+s.thr_lo+' · sigma '+(s.sigma||0).toFixed(2))
        : ((s.hw.indexOf('SIM')>=0 ? 'PC SOFTWARE / ' : 'PL HARDWARE / ')+s.mode_name
           + (s.diff_on ? '  ·  DIFF x3 ('+(s.diff_ms||0).toFixed(1)+' ms)' : '')));
    set('hw', s.hw+'  |  calib: '+s.calib+'  |  cam '+s.cam.toFixed(1)+
              ' fps  |  raw clients '+s.raw_clients);
    const hint=document.getElementById('hint');
    if(hint) hint.style.display = s.calib_box ? 'block' : 'none';
    lastStats = s;
    // AGC 按钮高亮：关掉时高亮 OFF，其余（含 SIM 下 None）高亮 ON
    const agEl=document.getElementById('agon'), agfEl=document.getElementById('agoff');
    if(agEl) agEl.className = (s.agc_on===false) ? 'cal' : 'cal on';
    if(agfEl) agfEl.className = (s.agc_on===false) ? 'cal on' : 'cal';
    // ---- 融合流水线可视化（2026-10-31）----
    // op 9/11/10 是同一条四级链的三个抽头：高亮到第几级 = 从第几级写回。
    // 亮的级数不同，一眼看出它们不是三个并列算子，而是同一条链的三个截面。
    const STAGE_OF={9:1,11:3,10:4};
    const TAP_OF={9:'op 9 从第 1 级写回（第 2~4 级未参与）',
                  11:'op 11 从第 3 级写回（第 4 级未参与）',
                  10:'op 10 从第 4 级写回（全链贯通）'};
    const nFused=STAGE_OF[s.mode]||0;
    for(let i=1;i<=4;i++){
      const e=document.getElementById('st'+i);
      if(!e) continue;
      e.className='pnode'+(i<=nFused?' on':'')+((i===nFused&&nFused>0)?' tap':'');
    }
    const pmEl=document.getElementById('pmode');
    if(pmEl){
      if(nFused>0){
        pmEl.className='fused';
        pmEl.textContent='四级融合流水线 · op '+s.mode+' '+s.mode_name;
      }else{
        pmEl.className='single';
        pmEl.textContent='单算子 3×3 通路 · op '+s.mode+' '+s.mode_name+' · 不经过融合链';
      }
    }
    set('ptap', nFused>0 ? '  ·  '+TAP_OF[s.mode] : '');
    set('ppl', (s.pl_kernel_ms||0).toFixed(3));
    renderPerf(s);
  }catch(e){
    // 别再静默吞掉：出问题时浏览器控制台能直接看到原因
    console.error('[stats]', e);
  }
}
init(); setInterval(stats,700); stats();

/* ---- 键盘快捷键（2026-10-31）----
   现场演示在一堆小按钮里点很容易手忙脚乱，用键盘直接打。
   走的是和按钮完全相同的 HTTP 路由，不另开一套状态。
     0~9      模式 0~9
     a / b    模式 10 CANNY / 11 NMS
     p / s    PL 引擎 / ARM 引擎（切到 ARM 帧率断崖，现场最强证据）
     g        AGC 开关切换（关灯演示用）
     c        重新校准      d 差值视图      m 肤色掩膜 */
document.addEventListener('keydown', function(ev){
  if(ev.metaKey||ev.ctrlKey||ev.altKey) return;
  const t=ev.target;
  if(t && (t.tagName==='INPUT'||t.tagName==='TEXTAREA')) return;
  const k=(ev.key||'').toLowerCase();
  const go=function(u){ fetch(u,{cache:'no-store'}).then(stats).catch(function(){}); };
  const on=function(f){ return !!(lastStats && lastStats[f]); };
  let u=null;
  if(k>='0'&&k<='9') u='/mode/'+k;
  else if(k==='a') u='/mode/10';
  else if(k==='b') u='/mode/11';
  else if(k==='p') u='/engine/pl';
  else if(k==='s') u='/engine/sw';
  else if(k==='c') u='/calib';
  else if(k==='g') u=(lastStats && lastStats.agc_on===false)?'/agc/on':'/agc/off';
  else if(k==='d') u=on('diff_on')?'/diff/off':'/diff/on';
  else if(k==='m') u=on('mask_on')?'/mask/off':'/mask/on';
  else if(k==='v') u=on('cmp_on')?'/cmp/off':'/cmp/on';
  if(u){ ev.preventDefault(); go(u); }
});
</script>
</body>
"""

# 对外实际发出的页面。main() 会先把 <!--MODES--> 换成按 MODE_NAMES 生成的按钮。
PAGE = PAGE_TEMPLATE


def build_mode_buttons():
    """按 filters.MODE_NAMES 生成模式按钮，不写死数量。

    为什么不写死三个：v5 要把算子从 3 个扩到 8 个，写死的话每加一个算子都要改
    HTML，而且板端 filters.py 和网页很容易不同步（之前就出过面积阈值两边各写
    一套、结果互相打架的事）。现在按钮数量完全由参考实现决定，那边加一个算子，
    这边自动多一个按钮，不需要改代码。
    """
    # 由 FINGER_MODE_MAP 反查，不写死成 1/2/3 —— 软件组把手指扩到 5 个之后
    # 这里自动跟着变，不用回来改（写死的代价之前在面积阈值上已经吃过一次）。
    tips = {m: "伸 %d 根手指自动切到这里" % n
            for n, m in FINGER_MODE_MAP.items()}
    parts = []
    for m in sorted(MODE_NAMES):
        tip = tips.get(m, "点这里切换（也可用挥手切换）")
        parts.append('<a href="/mode/%d" title="%s">%d %s</a>'
                     % (m, tip, m + 1, MODE_NAMES[m]))
    return "\n  ".join(parts)


def render_page():
    """按当前开关状态生成整页。

    模式按钮按参考实现现生成（扩张算子后网页不用改）；
    `<!--CMPCLS-->` 决定是否给 body 加 cmp 类 —— 加了才显示对比的两个窗格。
    所以每次切换对比视图都要重新生成一次，不能沿用启动时那份。
    """
    # 关着对比时两格不留 src：浏览器连 display:none 的 <img> 也会去拉流，
    # 白占两条 MJPEG 连接和 ARM 编码时间。
    # 占位符名字必须和模板里写的一模一样 —— 上一版这里写的是 SRC_A/SRC_B，
    # 模板里却是 SRC_STD，替换不到，结果右上角永远没有画面。
    _std = 'src="/cmpstd"' if CMP_ON[0] else ""
    _cls = "" if not CMP_ON[0] else ("cmp fit" if CMP_FIT else "cmp")
    return (PAGE_TEMPLATE
            .replace("<!--MODES-->", build_mode_buttons())
            .replace("<!--CMPCLS-->", _cls)
            .replace("<!--SRC_STD-->", _std)
            .replace("<!--TITLE-->",
                     "AG-CANNY | REAL-TIME DUAL-PATH COMPARISON" if CMP_ON[0]
                     else "PYNQ-Z2 REAL-TIME IMAGE ACCELERATION"))


def rebuild_page():
    """重新生成整页（切换对比视图后必须调用，否则 body 上的 cmp 类不会变）。"""
    global PAGE
    PAGE = render_page()


# ---------------------------------------------------------------- 线程 1：采集
def capture_loop(args):
    """只做 read()，持续把最新帧放进 FRESH。不参与任何计算。"""
    cap = open_camera(args.camera)
    try:
        while not STATE["stop"]:
            frame = read_frame(cap)
            with FRESH_LOCK:
                FRESH["frame"] = frame
                FRESH["fid"] += 1
    finally:
        cap.release()


# ---------------------------------------------------------------- 线程 2：处理
def _ensure_event_loop():
    """PYNQ 加载 Overlay 内部会调 asyncio.get_event_loop()。

    该调用在主线程会自动创建事件循环，在子线程里则直接抛
    RuntimeError: There is no current event loop in thread ...
    而 hw_driver 捕获任何异常后都会静默回退 SIM 模式（NumPy 硬算，
    filter 变 257 ms）—— 之前上板慢成 4 fps 就是这个原因。
    所以在启动硬件前，先给当前线程补一个事件循环。
    """
    import asyncio
    import warnings
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", DeprecationWarning)
        try:
            asyncio.get_event_loop()
        except RuntimeError:
            asyncio.set_event_loop(asyncio.new_event_loop())


def make_processor(args, kind):
    """按引擎类型造一个 processor。

    kind="pl" -> PL 硬件 IP（hw_driver.HardwareProcessor）
    kind="sw" -> ARM 纯软件（filters.SoftwareProcessor，与 HLS 逐像素一致的参考实现）

    两者接口完全一致：都是 .name / .run(mode, frame_bgr) -> uint8，
    所以流水线里可以随时互换，这正是现场 A/B 对比的基础。
    """
    if kind == "sw":
        return SoftwareProcessor()
    _ensure_event_loop()          # PYNQ 加载 Overlay 需要事件循环，见函数内注释
    from hw_driver import HardwareProcessor
    return HardwareProcessor(args.hw)


def process_loop(args):
    """取最新帧 -> 滤镜 -> 放进 HANDOFF。不等手势，所以帧率不被 78 ms 拖累。"""
    # n_modes 必须跟上 MODE_NAMES，否则 select() 会拒绝 mode 3..7（v5 实踩）
    latch = ModeLatch(n_modes=len(MODE_NAMES))
    processor = make_processor(args, ENGINE[0])
    if AGC_TARGET:
        # AGC_TARGET_MU 是 hw_driver 的模块级常量，run() 里每次现查全局，
        # 所以改模块属性就能生效，不用碰驱动代码。
        try:
            _hwd = sys.modules.get("hw_driver") or __import__("hw_driver")
            _hwd.AGC_TARGET_MU = float(AGC_TARGET)
            print("[agc] target mu override -> %.0f" % float(AGC_TARGET))
        except Exception as exc:
            print("[agc] target override failed:", exc)
    PROC[0] = processor                      # 让网页路由能改到驱动（AGC/阈值/增益）
    ENGINE_USED[0] = ENGINE[0]
    STATE["hw_name"] = processor.name
    print("[hw] engine: %s" % processor.name)

    count = 0
    fps_started = time.perf_counter()
    filter_ms = 0.0
    last_fid = -1
    last_mode_change = 0.0        # 上次切模式的时间，用于冷却
    try:
        while not STATE["stop"]:
            with FRESH_LOCK:
                fid = FRESH["fid"]
                frame = FRESH["frame"]
            if frame is None or fid == last_fid:
                time.sleep(0.0008)
                continue
            last_fid = fid

            # 引擎热切换：网页上点了 PL / ARM 就在这里换 processor。
            # 换完加速比实测会自动重跑一次，所以面板上的数字跟着变 —— 切到 ARM 时
            # 加速比会掉到 1.0× 附近，帧率同时断崖，这就是硬件加速的现场证据。
            if ENGINE_USED[0] != ENGINE[0]:
                try:
                    newp = make_processor(args, ENGINE[0])
                    old = processor
                    processor = newp
                    PROC[0] = processor              # 热切换后同步全局引用，别留下旧的
                    ENGINE_USED[0] = ENGINE[0]
                    STATE["hw_name"] = processor.name
                    print("[engine] switched -> %s" % processor.name)
                    close = getattr(old, "close", None)
                    if callable(close):
                        close()
                    BENCH_REQ[0] = True          # 换引擎后重新量一次
                except Exception as exc:
                    print("[engine] switch failed, keeping current:", exc)
                    ENGINE[0] = ENGINE_USED[0]   # 回滚，免得每帧重试刷屏

            # 加速比实测：启动后自动跑一次（约 0.3 s 卡顿，只此一次），
            # 这样评委一打开网页就能看到"PL 比 ARM 快多少倍"，不用等谁去点按钮。
            # 网页上的 RUN BENCHMARK 会重新置位 BENCH_REQ 再跑一次。
            if BENCH_AUTO[0] or BENCH_REQ[0]:
                BENCH_AUTO[0] = False
                BENCH_REQ[0] = False
                STATE["bench_state"] = "running"
                try:
                    rows = run_benchmark(processor, frame, reps=args.bench_reps)
                    with LOCK:
                        STATE["bench"] = rows
                        STATE["bench_state"] = "done"
                    for r in rows:
                        ARM_BASE[r["mode"]] = r["arm_ms"]
                    print("[bench] " + " | ".join(
                        "%s ARM %.1f / PL %.2f ms = %.2fx (e2e %.1f ms = %.2fx)"
                        % (r["name"], r["arm_ms"], r["pl_kernel_ms"],
                           r["speedup_kernel"], r["e2e_ms"], r["speedup_e2e"])
                        for r in rows))
                except Exception as exc:
                    with LOCK:
                        STATE["bench"] = []
                        STATE["bench_state"] = "failed: %s" % exc
                    print("[bench] failed:", exc)

            # 对比视图期间锁死算子：按钮、手势、挥手一律不换滤镜，
            # 否则下面两格比的东西随时在变，看不出在比什么。
            _mode_locked = bool(CMP_ON[0])
            if _mode_locked:
                STATE["mode"] = None                 # 丢掉按钮点击
                with LOCK:
                    SWIPE["pending"] = False         # 丢掉待处理的挥手
                if latch.mode != CANNY_MODE:
                    latch.select(CANNY_MODE)
                    STATE["mode_cur"] = CANNY_MODE

            ext = STATE["mode"]
            if ext is not None and not _mode_locked:
                latch.select(ext)
                STATE["mode"] = None
                STATE["mode_cur"] = ext
                print("[live] mode -> %d %s" % (ext, MODE_NAMES[ext]))
            if AUTO[0] and not _mode_locked:
                f = FINGERS["value"]
                # 冷却：切完一次模式后 mode_cooldown 秒内不再接受新的切换。
                # 没有它，识别一抖滤镜就乱跳（表现就是"看着手在动，画面自己乱换"）。
                if f in FINGER_MODE_MAP and time.perf_counter() - last_mode_change >= args.mode_cooldown:
                    before = latch.mode
                    latch.update(f, time.perf_counter())
                    if latch.mode != before:
                        STATE["mode_cur"] = latch.mode
                        last_mode_change = time.perf_counter()
                        print("[live] mode (gesture) -> %d %s"
                              % (latch.mode, MODE_NAMES[latch.mode]))

            # 挥手切换：左挥 → 下一个模式（序号 +1），右挥 → 上一个（序号 -1），12 个模式循环。
            # 方向约定（2026-10-21 按用户手感改）：从右往左划 = 往前走。
            # 和读列表/翻页一致 —— 内容"往后推"，视线往左走。右挥退回上一个。
            # 想改回来只需把下面 step 的两个分支对调（一处即可）。
            # 与手指切换共用 last_mode_change 冷却，避免两种手势叠加导致连跳。
            # 注意：冷却期内【不要】清掉 pending —— 挥手内部冷却 0.8 s、模式冷却 0.9 s，
            # 如果这里一进来就清零，第二次挥手正好落在冷却尾巴上就被无声吞掉，
            # 现场表现就是"第一下能切，第二下没反应"。留着，等冷却过了再执行一次。
            if SWIPE_TRACKER[0] is not None and not _mode_locked:
                with LOCK:
                    pending = SWIPE["pending"]
                    sdir = SWIPE["dir"]
                if pending and time.perf_counter() - last_mode_change >= args.mode_cooldown:
                    with LOCK:
                        SWIPE["pending"] = False
                    # sdir 是物理方向：right = 手掌往右划（dx > 0），left = 往左划
                    # 左挥 +1（下一项）、右挥 -1（上一项）—— 与网页按钮顺序一致
                    step = -1 if sdir == "right" else 1
                    before = latch.mode
                    new_mode = (before + step) % len(MODE_NAMES)
                    latch.select(new_mode)
                    STATE["mode_cur"] = new_mode
                    last_mode_change = time.perf_counter()
                    print("[live] mode (swipe %s %+d) -> %d %s"
                          % (sdir, step, new_mode, MODE_NAMES[new_mode]))

            t0 = time.perf_counter()
            output = processor.run(latch.mode, frame)
            filter_ms = (time.perf_counter() - t0) * 1000

            # ---- 对比模式：补测 σ + 多跑一遍"普通 Canny" ----
            # ③ 号窗格就是主画面本身（/raw，AG-Canny），
            # ② 号窗格要的是同一帧、同一套硬件、但关掉光照自适应的结果，
            # 所以这里额外跑一遍：AGC 关（gain=256）+ 固定双阈值 60/30。
            cmp_ms = 0.0
            if CMP_ON[0]:
                # 两件事都按"距上次多久"判断，不按帧号 —— 理由见 STD_INTERVAL 注释。
                _now = time.perf_counter()
                _need_sig = CMPSTD_ON and (_now - _LAST_SIG_T[0] >= SIGMA_INTERVAL)
                _need_std = (CMPSTD_ON
                             and _CMPSTD_FAILS[0] < 3
                             and ((_now - _LAST_STD_T[0] >= STD_INTERVAL)
                                  or CMPBUF["a"] is None))
                _left = None          # 不重算的帧沿用上一张对照图，下面靠它判断
                tc = time.perf_counter()
                # 先记下主画面（AG-Canny）这一帧实际用上的增益和阈值 ——
                # 下面那遍会覆盖 processor 上的读数，必须先把它们抄下来。
                _rg = int(getattr(processor, "last_gain", 256) or 256)
                _rt = int(getattr(processor, "last_thresh", 0) or 0)
                _prev_agc = getattr(processor, "agc_enabled", True)
                _prev_ad = getattr(processor, "canny_adaptive", False)
                try:
                    # σ 补测：op1..8 才返回 σ，op9~11 的 5x5 通路 PL **恒返回 0**
                    # （不是"测出 0"，是"没测"）。不补测的话 AG-Canny 的阈值
                    # 永远退化成默认 60/30，右边和左边就长得一模一样。
                    if _need_sig:
                        processor.run(1, frame)
                        _s = float(getattr(processor, "last_sigma", 0.0) or 0.0)
                        if _s > 0:
                            # ★ AGC 增益补偿（2026-11-03 修，之前漏了这一步）
                            # op1 走 3x3 通路，**不经过 AGC**；而 Canny 那条 5x5 通路
                            # 是在"级1 增益之后"才做 Sobel。AGC 把信号和噪声一起
                            # 放大 g 倍，所以实测 σ 必须乘回去 —— 否则算出来的阈值
                            # 只有实际需要的 1/g。现场表现就是"越暗噪点越多"，
                            # 光照自适应反而把噪声一起放大了。
                            # _rg 是主画面（AG-Canny）这一帧用的 gain，见上面抄下来的值。
                            _s = _s * max(_rg / 256.0, 1.0) * SIGMA_K
                            if SIGMA_FLOOR:
                                # 等效 σ 不低于 AG_HI_MIN/K_HI，等价于高阈值 ≥ AG_HI_MIN
                                _s = max(_s, AG_HI_MIN / _canny_k_hi())
                            _LAST_SIGMA[0] = _s
                            # hw_driver 用 _sigma_prev 算阈值，直接换成补偿后的值，
                            # 否则下一帧 run() 又会被 run(1) 测到的原始 σ 覆盖回去。
                            try:
                                processor._sigma_prev = _s
                                processor._sigma_prev_ok = True
                            except Exception:
                                pass
                        _LAST_SIG_T[0] = _now
                    if _need_std:
                        processor.agc_enabled = False
                        processor.canny_adaptive = False
                        # 必须 copy：驱动返回的是 PL 输出缓冲的视图，下一帧会原地覆盖
                        _left = np.array(processor.run(CANNY_MODE, frame),
                                         dtype=np.uint8, copy=True)
                    _lg = int(getattr(processor, "last_gain", 256) or 256)
                    _lt = int(getattr(processor, "last_thresh", 0) or 0)
                except Exception as exc:
                    _CMPSTD_FAILS[0] += 1
                    print("[cmp] standard pass failed (%d/3): %s"
                          % (_CMPSTD_FAILS[0], exc))
                    if _CMPSTD_FAILS[0] >= 3:
                        # 连着三次都炸就别再试了：每帧重试会把主链一起拖死，
                        # ② 号窗格保持上一张（或空白），①③ 照常跑。
                        print("[cmp] giving up on the standard pane; "
                              "compare view keeps running without it")
                    _left = None
                finally:
                    # 还原成网页上 AGC/THRESH 按钮设定的状态，别把面板状态改坏
                    processor.agc_enabled = _prev_agc
                    processor.canny_adaptive = _prev_ad
                if _left is not None:
                    _CMPSTD_FAILS[0] = 0
                    _LAST_STD_T[0] = _now
                    with LOCK:
                        CMPBUF["a"] = _left
                        CMPBUF["ver"] += 1
                        CMPSUM["gain_std"] = round(_lg / 256.0, 2)
                        CMPSUM["thr_std"] = "%d / %d" % ((_lt >> 8) & 0xFF, _lt & 0xFF)
                        CMPSUM["gain_ag"] = round(_rg / 256.0, 2)
                        CMPSUM["thr_ag"] = "%d / %d" % ((_rt >> 8) & 0xFF, _rt & 0xFF)
                cmp_ms = (time.perf_counter() - tc) * 1000

            # 差值视图：显示 |滤镜输出 − 灰度原图| × 3。
            # 灰度基准用 cv2.cvtColor 在 ARM 上算（约 1~2 ms），而不是再跑一遍
            # PL mode 0（那要 3.78 ms + 一次 cache 维护），这里只要能"看着对"就够。
            # 对比视图里画面已经是两张拼起来的，再减原图没有意义，直接跳过。
            diff_ms = 0.0
            if DIFF_ON[0] and not CMP_ON[0]:
                td = time.perf_counter()
                gray_ref = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                delta = cv2.absdiff(output, gray_ref)
                output = np.clip(delta.astype(np.uint16) * DIFF_GAIN,
                                 0, 255).astype(np.uint8)
                # 与软件组 display.difference_view 的契约保持一致：
                # 最外一圈恒为 0（3x3 窗的边界本来就没有定义）
                output[0, :] = 0
                output[-1, :] = 0
                output[:, 0] = 0
                output[:, -1] = 0
                diff_ms = (time.perf_counter() - td) * 1000

            # 把一次 run() 拆成两段，这样"快多少倍"能说清快在哪：
            #   pack_ms   = ARM 把 BGR 打包成 uint32 的耗时（纯软件开销）
            #   kernel_ms = AP_START 到 AP_DONE，PL 核内部算完的时间
            # hw_driver 每次 run 都会更新这两个属性，直接读现成的。
            pack_ms = float(getattr(processor, "last_pack_ms", 0.0) or 0.0)
            kernel_ms = float(getattr(processor, "last_pl_ms", 0.0) or 0.0)
            if kernel_ms <= 0.0:
                kernel_ms = filter_ms      # SIM 模式量不到 AP_DONE，退化成整段耗时
            base = (ARM_BASE[latch.mode]
                    if 0 <= latch.mode < len(ARM_BASE) else None)
            sp_live = round(base / kernel_ms, 1) if (base and kernel_ms > 0) else 0.0

            with LOCK:
                # frame 会被采集线程替换掉，这里必须拿一个稳定的引用
                HANDOFF["frame"] = frame
                # 必须显式拷贝：板上这版 hw_driver 返回的是 PL 物理缓冲的视图
                # （out_buf.reshape，不是副本），np.ascontiguousarray 对连续数组是
                # no-op，直接放进去的话下一帧 run() 会原地覆盖它，
                # 推流线程就会读到半新半旧的撕裂帧。
                HANDOFF["output"] = np.array(output, dtype=np.uint8, copy=True)
                HANDOFF["size"] = output.shape[:2]
                HANDOFF["ver"] += 1
                STATE["filter_ms"] = filter_ms
                STATE["diff_ms"] = diff_ms
                STATE["cmp_ms"] = cmp_ms
                # 用采集线程的单调帧号，别用 count：count 每 0.5 s 被 fps 统计清零，
                # 面板上就永远显示 0（v8.2b 实踩，看起来像"没在跑"）。
                STATE["fid"] = fid
                STATE["fingers"] = FINGERS["value"]
                STATE["g_ms"] = FINGERS["ms"]
                STATE["g_hz"] = FINGERS["hz"]
                # 硬件加速证据（面板用）
                STATE["pack_ms"] = pack_ms
                STATE["pl_kernel_ms"] = kernel_ms
                STATE["arm_base_ms"] = float(base or 0.0)
                STATE["speedup_live"] = sp_live
                # ---- 光照自适应 HUD ----
                # 全部用 getattr 兜底：SIM 路径 / 老版本 hw_driver 没有这些属性时
                # 不能让演示线程挂掉（演示崩了比 HUD 显示 '-' 严重得多）
                _g = getattr(processor, "last_gain", 256)
                STATE["gain_x"] = round(_g / 256.0, 2)
                STATE["mu"] = round(float(getattr(processor, "last_mu", 0.0) or 0.0), 1)
                # σ 在 op9~11 恒为 0（PL 不测），直接显示会一直是 0.00，看着像"没噪声"。
                # 所以本帧测到有效值就用本帧的，否则沿用最近一次有效值（对比视图里每 6 帧
                # 会插一次 op1 补测），避免数字在 0 和真实值之间来回跳。
                _sig_now = float(getattr(processor, "last_sigma", 0.0) or 0.0)
                if _sig_now > 0:
                    _LAST_SIGMA[0] = _sig_now
                STATE["sigma"] = round(_sig_now if _sig_now > 0 else _LAST_SIGMA[0], 2)
                _t = int(getattr(processor, "last_thresh", 0) or 0)
                STATE["thr_hi"] = (_t >> 8) & 0xFF
                STATE["thr_lo"] = _t & 0xFF
                STATE["agc_on"] = bool(getattr(processor, "agc_enabled", True))

            count += 1
            now = time.perf_counter()
            if now - fps_started >= 0.5:
                STATE["pipe"] = count / (now - fps_started)
                count, fps_started = 0, now
                # thr = 当前 Python 线程数。这个数只应该在小范围内稳定；
                # 每刷一次页面 / 每切一次对比视图它就往上涨、而且不回落，
                # 就说明有 handler 线程卡在半开连接上没回收（这才是"越点越卡"
                # 的真凶），STREAM_GEN 机制就是用来堵这个的。
                print("[live] thr %d | pipe %.1f fps | push %.1f fps | raw#%d | mode %d %s | "
                      "filter(PL) %.1f ms (pack %.1f + kernel %.2f) = %.1fx | "
                      "cmp %.1f ms | "
                      "gesture %.1f ms @ %.1f Hz | fingers %s | "
                      "swipe area=%.0f anchor=%d pts=%d dx=%+.2f span=%.2fs%s"
                      % (threading.active_count(), STATE["pipe"], STATE["push"], RAW_CLIENTS[0],
                         STATE["mode_cur"], MODE_NAMES[STATE["mode_cur"]],
                         filter_ms, pack_ms, kernel_ms, sp_live,
                         cmp_ms,
                         FINGERS["ms"], FINGERS["hz"], FINGERS["value"],
                         float(SWIPE_DIAG.get("area") or 0.0),
                         int(bool(SWIPE_DIAG["anchor"])), int(SWIPE_DIAG["points"]),
                         float(SWIPE_DIAG["dx"]), float(SWIPE_DIAG["span"]),
                         " (cooling)" if SWIPE_DIAG["blocked"] else ""))
    finally:
        close = getattr(processor, "close", None)
        if callable(close):
            close()


# ---------------------------------------------------------------- 线程 3：手势
def gesture_loop(args):
    """独立核跑手势识别，异步更新 FINGERS。慢（约 80~200 ms/次）但完全不阻塞显示。

    两个减负/稳像措施：
    * --gesture-interval：两次识别之间的最小间隔，限制手势线程占用的 CPU
    * 5 次多数投票：与测正确率的协议一致（每动作多数投票、非逐帧），
      单帧抖动不再直接跳到 HUD 上；手离开立即清空
    """
    session = GestureSession(boost=False if args.no_boost else None, face=not args.no_face)
    deadline = time.perf_counter() + args.auto_calib_at if args.auto_calib_at else None
    if deadline:
        print("[live] put your palm on the center box; auto-calibration at t=%.0fs"
              % args.auto_calib_at)
    min_gap = max(args.gesture_interval, 0.0)
    next_ok = 0.0
    hist = []                 # 最近 5 次有效（1/2/3）的识别结果
    last_fid = -1
    n_gest = 0
    hz_started = time.perf_counter()
    while not STATE["stop"]:
        with FRESH_LOCK:
            fid = FRESH["fid"]
            frame = FRESH["frame"]
        if frame is None or fid == last_fid:
            time.sleep(0.002)
            continue
        last_fid = fid
        if time.perf_counter() < next_ok:
            time.sleep(0.003)
            continue

        if STATE["calib"]:
            STATE["calib"] = False
            session.calibrate(session.prepare(frame))
            STATE["calib_msg"] = "calibrated" if session.calibrated else "FAILED"
            STATE["calib_box"] = not session.calibrated   # 成功就撤掉黄框，失败留着让人重试
            print("[live] recalibrate ->", session.calibration_status)
        elif deadline is not None and not session.calibrated and time.perf_counter() >= deadline:
            session.calibrate(session.prepare(frame))
            STATE["calib_msg"] = "calibrated" if session.calibrated else "FAILED"
            STATE["calib_box"] = not session.calibrated
            print("[live] auto calibration ->", session.calibration_status)

        t0 = time.perf_counter()
        prepared = session.prepare(frame)
        fingers, _dbg, detail = session.detect(prepared, False)
        dt = (time.perf_counter() - t0) * 1000
        next_ok = time.perf_counter() + min_gap

        # 面积阈值：这是治"空手被判成 1"的关键。
        # 画面里一小块肤色噪点也会被 findContours 当成手，进而数出 1 根手指。
        # 要求轮廓面积占到检测图的 --min-area 以上才算真的有手，否则一律判 0。
        try:
            area = float(detail.get("area", 0) or 0)
        except Exception:
            area = 0.0
        STATE["area"] = area
        # 软件组 2026-10-18 版把"为什么判没手"也放进了 reason
        # （hand_too_big / too_solid / too_ragged / too_smooth / too_thin / hand_too_small），
        # 现场排查比看面积直观得多，直接透传到 HUD
        STATE["reason"] = str(detail.get("reason", "") or "")
        # 面积窗口（默认关闭，见 --min-area / --max-area 的说明）：
        # 现场实测：空手噪点几十，正常手掌几千，但偶尔整帧误检能到几万 ——
        # 只卡下限挡不住后者，必须同时卡上限。
        lo = args.min_area * DETECT_PIX[0]
        hi = args.max_area * DETECT_PIX[0]
        STATE["area_lo"] = lo
        STATE["area_hi"] = hi
        STATE["strict"] = STRICT[0]
        if area < lo or area > hi:
            v = 0
        else:
            v = fingers if isinstance(fingers, int) and 0 <= fingers <= 5 else 0

        # ---- 挥手检测：只要"画面里有手"（走过面积窗口）就跟踪，不要求数出指尖 ----
        # 这样握拳/摊掌也能挥，比绑在 fingers 上好用得多。
        # hand_center 由 gesture.count_fingers 提供；手没过几何判据时它返回 None，
        # tracker 收到 None 会自动 reset，等于"手丢了就不算一次挥手"。
        tracker = SWIPE_TRACKER[0]
        if tracker is not None:
            # 挥手用 track_center（宽松判据：只要有一块大小合理的肤色在动），
            # 不用 hand_center —— 后者要求"长得像手"，张开手掌正面朝镜头时
            # solidity 会超过 0.88 被拒，于是 area 恒 0、挥手永远不触发。
            t_area = float(detail.get("track_area") or 0.0)
            t_pos = detail.get("track_center") or detail.get("hand_center")
            SWIPE_DIAG["area"] = t_area
            try:
                ev = tracker.update(t_pos,
                                    time.monotonic(),
                                    valid=(t_area > 0),
                                    area=max(t_area, 1.0))
                # 诊断字段直接透传到网页：anchor=0 说明手没停稳、dx 不够说明幅度不够
                for k in ("anchor", "points", "dx", "span", "speed"):
                    if k in ev:
                        SWIPE_DIAG[k] = ev[k]
                SWIPE_DIAG["blocked"] = time.monotonic() < getattr(tracker, "blocked_until", 0.0)
                if ev.get("swipe"):
                    with LOCK:
                        sid = int(ev.get("swipe_id", 0) or 0)
                        if sid != SWIPE["id"]:
                            SWIPE["id"] = sid
                            SWIPE["dir"] = ev.get("swipe_dir")
                            SWIPE["pending"] = True
                            SWIPE_DIAG["last_dir"] = ev.get("swipe_dir")
                            print("[live] swipe -> %s" % ev.get("swipe_dir"))
            except Exception as exc:
                print("[swipe] tracker error:", exc)

        # 掩膜快照（只在打开时算，make_skin_mask 是 59 ms 的大头，平时别碰）
        if MASK_ON[0]:
            try:
                import gesture as g
                mk = g.make_skin_mask(prepared)
                ok, mb = cv2.imencode(".jpg", mk,
                                      [int(cv2.IMWRITE_JPEG_QUALITY), 70])
                if ok:
                    with LOCK:
                        STATE["mask_jpg"] = mb.tobytes()
                        STATE["mask_ver"] += 1
            except Exception as exc:
                print("[mask] snapshot failed:", exc)

        # 统一多数投票：0（无手）也计入，防止 0/1/2 交替时 HUD 来回跳。
        # 5 次里出现 >= 3 次才更新；与测正确率的"每动作多数投票"协议一致。
        hist.append(v)
        if len(hist) > 5:
            hist.pop(0)
        top, cnt = Counter(hist).most_common(1)[0]
        if cnt >= 3:
            FINGERS["value"] = top
        FINGERS["ms"] = dt
        n_gest += 1
        now = time.perf_counter()
        if now - hz_started >= 1.0:
            FINGERS["hz"] = n_gest / (now - hz_started)
            n_gest = 0
            hz_started = now


# ------------------------------------------------- 线程 4：摄像头参考窗（低频 JPEG）
def cam_encode_loop(args):
    """彩色原图 921 KB 太大（84 ms/帧），不能和主画面争带宽。

    这里按 --cam-fps（默认 8）慢速压 JPEG，且先用 --cam-scale（默认 0.5）缩小
    再编码：320x240 的 JPEG 只要 ~20 ms，左侧参考窗从 4 fps 提到 8 fps
    也只占约 15% 的一个核。左窗只是"对比参考"，右边的 PL 输出才是作品画面。

    例外：开了 2x2 对比视图时改成 640x480 原生分辨率 + 5 fps，
    否则①号在同样大的格子里会明显比②③糊。
    """
    # 对比模式下①号必须和②③同分辨率（都是 640x480），否则同样大的格子里
    # ① 是 320x240 被放大、②③ 是原生 640x480，一眼就能看出清晰度不一样，
    # 评委第一反应是"这不是同一帧"。代价是编码贵，所以同时把帧率压到 5 fps。
    while not STATE["stop"]:
        t0 = time.perf_counter()
        cmp_on = CMP_ON[0]
        scale = 1.0 if cmp_on else float(args.cam_scale)
        fps = min(float(args.cam_fps), 5.0) if cmp_on else float(args.cam_fps)
        with LOCK:
            frame = HANDOFF["frame"]
        if frame is not None:
            if scale != 1.0:
                view = cv2.resize(frame, None, fx=scale, fy=scale,
                                  interpolation=cv2.INTER_AREA)
            else:
                view = frame
            # 黄框必须画在"缩放之后"——先画再缩放的话 3px 的线会被压成 1.5px，
            # JPEG 一压基本看不见。坐标按 cam_scale 同步缩放。
            # 而且要 copy：scale=1.0 时 view 就是采集线程的数组，不能原地画。
            if STATE.get("calib_box") and CALIB_BOX[0] is not None:
                view = view.copy()
                x1, y1, x2, y2 = [int(round(v * scale)) for v in CALIB_BOX[0]]
                cv2.rectangle(view, (x1, y1), (x2, y2), (0, 220, 255), 2)
                ty = y1 - 8 if y1 > 22 else y2 + 18
                cv2.putText(view, "PALM HERE", (x1, ty), cv2.FONT_HERSHEY_SIMPLEX,
                            0.5, (0, 220, 255), 1, cv2.LINE_8)
            ok, buf = cv2.imencode(".jpg", view,
                                   [int(cv2.IMWRITE_JPEG_QUALITY), int(args.quality)])
            if ok:
                with LOCK:
                    STATE["cam_jpg"] = buf.tobytes()
                    STATE["cam_ver"] += 1
        # 间隔每次重算：开关对比视图后不用重启线程就生效。
        rest = (1.0 / max(fps, 0.1)) - (time.perf_counter() - t0)
        if rest > 0:
            time.sleep(rest)


def cmp_encode_loop(args):
    """把②号窗格（普通 Canny）压成 JPEG，供 /cmpstd 推流。

    单独一条线程的原因和 cam_encode_loop 一样：640x480 的 JPEG 编码在 ARM 上
    要十几毫秒，塞进主循环会把帧率压死。关着对比时整条线程空转。
    """
    quality = int(args.quality)
    last = -1
    warned = False
    while not STATE["stop"]:
        # 整段包 try：cv2.imencode 抛一次异常就会让**这条线程整个死掉**，
        # 表现是 ② 号窗格永远停在某一帧（或一直空白），而终端上没有任何报错，
        # 极难定位。兜住之后最坏情况是丢一帧，线程继续活。
        try:
            if not CMP_ON[0]:
                time.sleep(0.15)
                continue
            with LOCK:
                ver = CMPBUF["ver"]
                a = CMPBUF["a"]
            if a is not None and ver != last:
                last = ver
                ok, buf = cv2.imencode(".jpg", a,
                                       [int(cv2.IMWRITE_JPEG_QUALITY), quality])
                if ok:
                    with LOCK:
                        CMPJPG["jpg"] = buf.tobytes()
                        CMPJPG["ver"] = ver
        except Exception as exc:
            if not warned:
                print("[cmp] encode loop error (kept alive):", exc)
                warned = True
        # 0.06 s ≈ 最高 16 次/s 编码。留这个间隔不是为了限帧，而是让出 GIL：
        # 板上是双核 Cortex-A9，采集/处理/手势/三条推流全是 Python 线程，
        # 空转 while 会把 GIL 抢得主循环一次 run() 从 3.7 ms 拖到 30 ms 以上。
        time.sleep(0.06)


# ---------------------------------------------------------------- HTTP
def _tick_push():
    global _push_n, _push_t0
    _push_n += 1
    now = time.perf_counter()
    if now - _push_t0 >= 0.5:
        STATE["push"] = _push_n / (now - _push_t0)
        _push_n = 0
        _push_t0 = now


class Handler(BaseHTTPRequestHandler):
    disable_nagle_algorithm = True   # 实时流必须关掉 Nagle，否则小包会被攒着延迟发
    # 每个连接的 socket 超时。不设的话，对端异常断开（拔网线、笔记本合盖、
    # 浏览器进程被杀）时这条连接既不报错也不关闭，handler 线程就永久挂在
    # write 上。设了超时，卡住的连接最多挂 20 s 就会抛异常被回收。
    timeout = 20

    def log_message(self, *a):
        pass

    def _send(self, body, ctype):
        self.send_response(200)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def _stream_raw(self):
        """把 PL 输出的原始灰度字节直接推走：零合成、零压缩。"""
        self.send_response(200)
        self.send_header("Content-Type", "application/octet-stream")
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        gen = _take_stream("raw")
        RAW_CLIENTS[0] += 1
        print("[raw] client connected (%d open)" % RAW_CLIENTS[0])
        last = -1
        try:
            while _stream_alive("raw", gen):
                t0 = time.perf_counter()
                # 每帧实时算：开关对比视图后不用重连也生效。
                # 对比模式三条流（/raw + /cam + /cmpstd）同时跑，网口只有 100 Mbps，
                # 主画面 307 KB/帧按 25 fps 推就是 61 Mbps，会把另外两条挤没。
                _f = RAW_FPS[0]
                if CMP_ON[0] and _f > 12.0:
                    _f = 12.0
                min_dt = 1.0 / max(_f, 0.1)
                with LOCK:
                    ver = HANDOFF["ver"]
                    out = HANDOFF["output"]
                if out is not None and ver != last:
                    last = ver
                    # memoryview 避免再拷一次 307 KB；out 是 C 连续 uint8
                    self.wfile.write(memoryview(out))
                    _tick_push()
                rest = min_dt - (time.perf_counter() - t0)
                if rest > 0:
                    time.sleep(rest)
        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
            pass   # 浏览器关标签页/刷新，正常现象
        finally:
            RAW_CLIENTS[0] -= 1
            print("[raw] client disconnected (%d open)" % RAW_CLIENTS[0])

    def _stream_cam(self):
        self.send_response(200)
        self.send_header("Age", "0")
        self.send_header("Cache-Control", "no-cache, private")
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=--jpgboundary")
        self.end_headers()
        gen = _take_stream("cam")
        last = -1
        try:
            while _stream_alive("cam", gen):
                with LOCK:
                    ver = STATE["cam_ver"]
                    data = STATE["cam_jpg"]
                if data and ver != last:
                    last = ver
                    self.wfile.write(b"--jpgboundary\r\n")
                    self.wfile.write(b"Content-Type: image/jpeg\r\n")
                    self.wfile.write(("Content-Length: %d\r\n\r\n" % len(data)).encode())
                    self.wfile.write(data)
                    self.wfile.write(b"\r\n")
                time.sleep(0.05)
        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
            return

    def _mjpg(self, key, ver_key, sleep=0.1):
        """通用的 MJPEG 推流（掩膜、普通 Canny 对比格都用它）。key 是 CMPJPG 的键。

        sleep 默认 0.1 s；②号对照图刷新更快（STD_INTERVAL 0.12 s ≈ 8 Hz），
        轮询间隔必须比它小，否则新帧要等到下一次轮询才发出去，
        实际刷新率被砍半，看着就是"右上角帧率低"。
        """
        self.send_response(200)
        self.send_header("Age", "0")
        self.send_header("Cache-Control", "no-cache, private")
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=--jpgboundary")
        self.end_headers()
        gen = _take_stream(key)
        last = -1
        try:
            while _stream_alive(key, gen):
                with LOCK:
                    data = CMPJPG[key]
                    ver = CMPJPG["ver"]
                if data and ver != last:
                    last = ver
                    self.wfile.write(b"--jpgboundary\r\n")
                    self.wfile.write(b"Content-Type: image/jpeg\r\n")
                    self.wfile.write(("Content-Length: %d\r\n\r\n" % len(data)).encode())
                    self.wfile.write(data)
                    self.wfile.write(b"\r\n")
                time.sleep(sleep)
        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
            return

    def _stream_mask(self):
        """肤色掩膜快照流：白=被判成皮肤。整屏发白就说明是阈值问题，不是数手指的问题。"""
        self.send_response(200)
        self.send_header("Age", "0")
        self.send_header("Cache-Control", "no-cache, private")
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=--jpgboundary")
        self.end_headers()
        gen = _take_stream("mask")
        last = -1
        try:
            while _stream_alive("mask", gen):
                with LOCK:
                    ver = STATE["mask_ver"]
                    data = STATE["mask_jpg"]
                if data and ver != last:
                    last = ver
                    self.wfile.write(b"--jpgboundary\r\n")
                    self.wfile.write(b"Content-Type: image/jpeg\r\n")
                    self.wfile.write(("Content-Length: %d\r\n\r\n" % len(data)).encode())
                    self.wfile.write(data)
                    self.wfile.write(b"\r\n")
                time.sleep(0.08)
        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
            return

    def do_GET(self):
        p = self.path.split("?")[0]
        if p in ("/", "/index.html"):
            self._send(PAGE.encode("utf-8"), "text/html; charset=utf-8")
        elif p == "/meta":
            with LOCK:
                size = HANDOFF["size"]
            h, w = size if size else (480, 640)
            self._send(json.dumps({"w": int(w), "h": int(h)}).encode(), "application/json")
        elif p == "/raw":
            self._stream_raw()
        elif p == "/cam":
            self._stream_cam()
        elif p.startswith("/mode/"):
            try:
                m = int(p.rsplit("/", 1)[1])
                # 不再写死 0/1/2：v5 扩到 8 个算子后，模式号由参考实现决定
                if m in MODE_NAMES:
                    STATE["mode"] = m
            except ValueError:
                pass
            self._send(PAGE.encode("utf-8"), "text/html; charset=utf-8")
        elif p == "/calib":
            STATE["calib"] = True
            STATE["calib_box"] = True          # 重新亮出黄框
            self._send(PAGE.encode("utf-8"), "text/html; charset=utf-8")
        elif p in ("/mask/on", "/mask/off"):
            MASK_ON[0] = p.endswith("/on")
            print("[mask] view ->", MASK_ON[0])
            self._send(PAGE.encode("utf-8"), "text/html; charset=utf-8")
        elif p in ("/gesture/swipe", "/gesture/fingers"):
            # 手势风格二选一。挥手时手型必被数成 1 指，两种同时开必然打架，
            # 所以这里是互斥开关，不是叠加开关。
            AUTO[0] = p.endswith("/fingers")
            print("[gesture] style -> %s" % ("fingers (1~5 -> GRAY/SOBEL/CANNY/NMS/GAUSS5)"
                                             if AUTO[0] else "swipe (left/right)"))
            self._send(PAGE.encode("utf-8"), "text/html; charset=utf-8")
        elif p == "/diff/on" or p == "/diff/off":
            # 差值视图开关。和 MASK 一样是"看得见"的诊断/展示手段：
            # 不改变 PL 怎么算，只改变推给浏览器的是"结果"还是"结果与原图的差"。
            DIFF_ON[0] = p.endswith("/on")
            print("[diff] view ->", DIFF_ON[0])
            self._send(PAGE.encode("utf-8"), "text/html; charset=utf-8")
        elif p in ("/cmp/on", "/cmp/off"):
            # AGC 对比视图：页面从"两窗格"切成 2x2 四窗格，
            # 必须重发整页让 body 拿到 cmp 类，下面两格才会显示出来。
            CMP_ON[0] = p.endswith("/on")
            if CMP_ON[0]:
                DIFF_ON[0] = False       # 两个视图互斥
                MASK_ON[0] = False       # 掩膜窗格会挤掉 2x2 布局
            rebuild_page()
            print("[cmp] AGC compare view ->", CMP_ON[0])
            self._send(PAGE.encode("utf-8"), "text/html; charset=utf-8")
        elif p == "/mask":
            self._stream_mask()
        elif p == "/cmpstd":
            # ②号窗格：普通 Canny（PL，AGC 关 + 固定阈值）
            self._mjpg("jpg", None, sleep=0.04)
        elif p == "/bench":
            # 只在「当前这一帧」上跑一次 PL vs ARM。很贵（约 0.3 s 卡顿），
            # 所以不做成常驻，按需触发。
            BENCH_REQ[0] = True
            self._send(PAGE.encode("utf-8"), "text/html; charset=utf-8")
        elif p in ("/engine/pl", "/engine/sw"):
            # 现场 A/B：同一路摄像头、同一条流水线，只换"谁来算滤镜"。
            # 切到 ARM 后 PIPE 帧率会明显下跌，加速比掉回 1× 左右。
            ENGINE[0] = "sw" if p.endswith("/sw") else "pl"
            print("[engine] request ->", ENGINE[0])
            self._send(PAGE.encode("utf-8"), "text/html; charset=utf-8")
        elif p.startswith("/strict/"):
            # 现场调肤色判定的松紧（等价于 main_hw 里的 +/- 键），改完立刻生效
            tok = p.rsplit("/", 1)[1]
            try:
                import gesture as g
                if tok == "up":
                    val = STRICT[0] + 0.15
                elif tok == "down":
                    val = STRICT[0] - 0.15
                else:
                    val = float(tok)
                g.apply_strict(val)
                STRICT[0] = float(getattr(g, "STRICT", val))
                print("[gesture] strict -> %.2f" % STRICT[0])
            except Exception as exc:
                print("[gesture] strict adjust failed:", exc)
            self._send(PAGE.encode("utf-8"), "text/html; charset=utf-8")
        elif p in ("/agc/on", "/agc/off"):
            # AGC 自动增益开关。关掉就退回 gain=256（与 v6 逐字节一致）。
            # 现场用它做「关灯对比」：AGC ON 暗处自动提亮，OFF 直接变黑 ——
            # 这是「光照自适应」在演示里最直观的一击。
            drv = PROC[0]
            if drv is None:
                print("[agc] 驱动还没起来，忽略")
            else:
                drv.agc_enabled = p.endswith("/on")
                print("[agc] ->", drv.agc_enabled)
            self._send(PAGE.encode("utf-8"), "text/html; charset=utf-8")
        elif p.startswith("/thresh/"):
            # Canny 阈值。auto = 交回按实测 sigma 自动算；整数 = 现场钉死。
            tok = p.rsplit("/", 1)[1]
            drv = PROC[0]
            if drv is None:
                print("[thresh] 驱动还没起来，忽略")
            else:
                try:
                    drv.thresh_override = (None if tok == "auto"
                                           else max(0, min(255, int(tok))))
                    # 关键：光清掉 override 是不够的。hw_driver 里 canny_adaptive
                    # 默认就是 False（为了 validate 可复现），所以此前点 THRESH AUTO
                    # 只是把阈值交回"固定默认值 60/30"，σ 自适应从来没有真正打开过
                    # —— 演示里看不到阈值随光照噪声漂移，就是这个漏掉的赋值。
                    # auto = 交给实测 sigma；钉死整数时关掉自适应，避免两者打架。
                    drv.canny_adaptive = (tok == "auto")
                    print("[thresh] -> override=%s adaptive=%s"
                          % (drv.thresh_override, drv.canny_adaptive))
                except ValueError:
                    print("[thresh] 忽略非法值:", tok)
            self._send(PAGE.encode("utf-8"), "text/html; charset=utf-8")
        elif p.startswith("/gain/"):
            # 增益钉死（Q8.8，256 = 1.0x）。auto = 交回 AGC。
            tok = p.rsplit("/", 1)[1]
            drv = PROC[0]
            if drv is None:
                print("[gain] 驱动还没起来，忽略")
            else:
                try:
                    drv.gain_override = (None if tok == "auto"
                                         else max(64, min(1024, int(tok))))
                    print("[gain] ->", drv.gain_override)
                except ValueError:
                    print("[gain] 忽略非法值:", tok)
            self._send(PAGE.encode("utf-8"), "text/html; charset=utf-8")
        elif p == "/stats":
            with LOCK:
                body = {"pipe": STATE["pipe"], "push": STATE["push"], "cam": STATE["cam"],
                        "filter_ms": STATE["filter_ms"], "gesture_ms": STATE["g_ms"],
                        "gesture_hz": STATE["g_hz"], "fingers": STATE["fingers"],
                        "hw": STATE["hw_name"] or "PC SOFTWARE (no PL)",
                        "mode": STATE["mode_cur"], "mode_name": MODE_NAMES[STATE["mode_cur"]],
                        "raw_clients": RAW_CLIENTS[0], "calib_box": STATE["calib_box"],
                        "area": STATE["area"], "area_lo": STATE["area_lo"],
                        "area_hi": STATE["area_hi"], "strict": STATE["strict"],
                        "reason": STATE["reason"],
                        "mask_on": MASK_ON[0], "calib": STATE["calib_msg"],
                        "diff_on": DIFF_ON[0], "diff_ms": STATE["diff_ms"],
                        "cmp_on": CMP_ON[0], "cmp_ms": STATE.get("cmp_ms", 0.0),
                        "cmp_gain_std": CMPSUM["gain_std"],
                        "cmp_thr_std": CMPSUM["thr_std"],
                        "cmp_thr_ag": CMPSUM["thr_ag"],
                        "fid": STATE.get("fid", 0),
                        "swipe": dict(SWIPE_DIAG), "auto_control": AUTO[0],
                        # --- 硬件加速证据 ---
                        "pack_ms": STATE["pack_ms"],
                        "pl_kernel_ms": STATE["pl_kernel_ms"],
                        "arm_base_ms": STATE["arm_base_ms"],
                        "speedup_live": STATE["speedup_live"],
                        "bench": STATE["bench"],
                        "bench_state": STATE["bench_state"],
                        "engine": ENGINE[0],
                        "engine_used": ENGINE_USED[0] or ENGINE[0],
                        # --- 光照自适应 HUD ---
                        "gain_x": STATE["gain_x"], "mu": STATE["mu"],
                        "sigma": STATE["sigma"], "thr_hi": STATE["thr_hi"],
                        "thr_lo": STATE["thr_lo"], "agc_on": STATE["agc_on"]}
            self._send(json.dumps(body).encode(), "application/json")
        elif p == "/stop":
            STATE["stop"] = True
            self._send(PAGE.encode("utf-8"), "text/html; charset=utf-8")
        else:
            self.send_response(404)
            self.end_headers()


def main():
    # 默认位流必须取**本脚本同目录**的那一份，不要写死成某个版本目录。
    # 历史坑：这里曾经兜底到 rgb2gray_v4/rgb2gray.bit —— 在新目录里跑
    # `python3 live_view_raw.py --hw` 时会**静默加载 v4 位流**。v4 硬件只有
    # mode 0/1/2 的分支，mode>=3 全部落到 else 输出 EMBOSS，现象就是
    # "后面几个滤镜看起来跟浮雕一模一样"，而 validate 却是全对（它用的是自己
    # 目录的位流）。这种"硬件没坏、只是加载错了文件"的坑极难从画面上反推。
    #
    # 2026-10-30：**删掉那个 v4 兜底**。hw_driver 现在有 MD5 校验，
    # 原则是"宁可报错也不要加载错误版本" —— 与其加载旧位流，
    # 不如让 FileNotFoundError 直接把问题摆到脸上。
    _here = os.path.dirname(os.path.abspath(__file__))
    _default_bit = os.path.join(_here, "rgb2gray.bit")

    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--camera", type=int, default=0)
    ap.add_argument("--hw", nargs="?",
                    const=_default_bit,
                    default=None, help="use the PL accelerator (default: software)")
    ap.add_argument("--port", type=int, default=8080)
    ap.add_argument("--no-gesture", action="store_true")
    ap.add_argument("--no-swipe", action="store_true",
                    help="关闭挥手切换（只保留 1/2/3 指切换）")
    ap.add_argument("--no-face", action="store_true")
    ap.add_argument("--no-boost", action="store_true")
    ap.add_argument("--auto", action="store_true",
                    help="额外开启「1/2/3 根手指 -> GRAY/SOBEL/EMBOSS」"
                         "（默认关闭。挥手时手型常被判成 1 指，会和挥手抢模式，"
                         "所以数字手势必须显式打开）")
    ap.add_argument("--no-auto", action="store_true",
                    help="兼容旧命令：等价于不加 --auto（现在本来就是默认）")
    ap.add_argument("--swipe-travel", type=float, default=0.16,
                    help="挥手判定的横向位移阈值（归一化宽度，0.16 ≈ 画面 1/6）。"
                         "挥不动就调小到 0.12，误触发就调大到 0.22")
    ap.add_argument("--swipe-cooldown", type=float, default=0.8,
                    help="挥手判定内部的冷却秒数（与 --mode-cooldown 取大值）")
    ap.add_argument("--auto-calib-at", type=float, default=8.0)
    ap.add_argument("--raw-fps", type=float, default=25.0,
                    help="PL 结果原始流上限。100 Mbps 网口，25 fps 已占约 6 成带宽")
    ap.add_argument("--cam-fps", type=float, default=8.0,
                    help="摄像头参考窗 JPEG 帧率（配合 --cam-scale 0.5 编码很便宜）")
    ap.add_argument("--cam-scale", type=float, default=0.5,
                    help="参考窗编码前缩放（1.0 = 原始 640x480，CPU 贵 4 倍）")
    ap.add_argument("--gesture-interval", type=float, default=0.10,
                    help="两次手势识别的最小间隔（秒），限制手势线程的 CPU 占用")
    ap.add_argument("--min-area", type=float, default=0.0,
                    help='轮廓面积占检测图比例的下限（0 = 关闭）。'
                         '默认关闭：2026-10-18 软件组已把五道形状判据做进 gesture.py，'
                         '那里才是唯一判据，这里再卡一道会两边打架')
    ap.add_argument("--max-area", type=float, default=1.0,
                    help='轮廓面积占检测图比例的上限（1 = 关闭）。'
                         '同 --min-area，默认关闭；gesture.py 的 MAX_AREA_FRAC 才是准的')
    ap.add_argument("--mode-cooldown", type=float, default=0.9,
                    help="切换滤镜后的冷却秒数，防止识别抖动导致滤镜乱跳。"
                         "默认 0.9 = 挥手内部冷却 0.8 再留 0.1 余量；"
                         "设成 1.5 会让人觉得'第二下挥手没反应'")
    ap.add_argument("--no-bench", action="store_true",
                    help="启动时不要自动跑加速比实测（默认会自动跑一次，好让评委一打开"
                         "网页就看到数字）。跑一次约 0.3 s，其间画面会卡一下")
    ap.add_argument("--bench-reps", type=int, default=5,
                    help="实测时每个模式重复几次取最快值（默认 5）")
    ap.add_argument("--quality", type=int, default=65)
    args = ap.parse_args()

    # 默认只用挥手换滤镜。手指控制必须显式 --auto 才开 ——
    # 挥手过程中手型极容易被数成 1 指，若默认开着，画面会被反复拽回 mode 0（GRAY），
    # 现象就是"一挥手滤镜就变回灰度、根本换不动"。
    AUTO[0] = bool(args.auto) and not args.no_auto

    global PAGE
    PAGE = render_page()

    RAW_FPS[0] = args.raw_fps
    STATE["cam"] = args.cam_fps
    BENCH_AUTO[0] = not args.no_bench
    compute_calib_box()          # 黄框坐标（校准提示用）

    # 挥手跟踪器（软件组 swipe.py）。缺文件就整体关掉，不影响其余功能。
    if not args.no_swipe:
        try:
            from swipe import SwipeTracker
            SWIPE_TRACKER[0] = SwipeTracker(travel=args.swipe_travel,
                                            cooldown=args.swipe_cooldown)
            print("[swipe] enabled (LEFT = next mode, RIGHT = prev) "
                  "travel=%.2f cooldown=%.2f" % (args.swipe_travel, args.swipe_cooldown))
        except Exception as exc:
            print("[swipe] disabled, swipe.py unavailable:", exc)
    else:
        print("[swipe] disabled by --no-swipe")

    threading.Thread(target=capture_loop, args=(args,), daemon=True).start()
    threading.Thread(target=process_loop, args=(args,), daemon=True).start()
    if not args.no_gesture:
        threading.Thread(target=gesture_loop, args=(args,), daemon=True).start()
    threading.Thread(target=cam_encode_loop, args=(args,), daemon=True).start()
    threading.Thread(target=cmp_encode_loop, args=(args,), daemon=True).start()

    # 端口占用必须先查：旧进程没停掉的话，新的 bind 会失败退出，
    # 而浏览器连的还是旧进程（旧代码），表现就是"传了新文件却没变化/越来越卡"。
    _probe = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    _probe.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        _probe.bind(("0.0.0.0", args.port))
    except OSError as exc:
        print("[live] 端口 %d 已被占用（%s）。先 Ctrl-C 停掉上一个进程再启动，"
              "否则浏览器连的还是旧代码。" % (args.port, exc))
        raise SystemExit(1)
    finally:
        _probe.close()

    class _Srv(ThreadingHTTPServer):
        daemon_threads = True      # 主进程退出时不僵在残留线程上
        allow_reuse_address = True

    srv = _Srv(("0.0.0.0", args.port), Handler)
    print("[live] serving on http://<board-ip>:%d/  (Ctrl+C to stop)" % args.port)
    print("[live] 满分辨率直传：PL 结果 640x480 原始字节，不降采样、不压 JPEG")
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        STATE["stop"] = True
        srv.shutdown()


if __name__ == "__main__":
    main()
