# -*- coding: utf-8 -*-
"""
手势识别原型（任务 ① / S2）
--------------------------------------------------
用途：用笔记本自带摄像头，让程序数出你伸了几根手指
      1 根 -> 灰度    2 根 -> Sobel    3 根 -> 浮雕

运行：  python gesture.py
        python gesture.py --check     （只检查摄像头，不开窗口）

按键：  q 退出
        c 校准肤色（手掌盖住中央黄框后按）
        d 开关调试画面（看程序"眼里的手"长什么样）
        b 开关软件增亮
        r 开关"只看画面中间一块"
        f 开关"忽略人脸"
        [ / ] 调曝光（[ 更快更流畅更暗，] 更亮更卡）
        + / - 调严格程度（数多了按 +，数少了按 -）
        s 保存当前画面到 snapshot.png

数手指的原理（两套办法互相验证）：
        办法一「找指尖」：以手掌圆心为原点，轮廓上"离掌心距离"曲线的
                 显著山峰 = 手指尖。没伸直的手指、指关节鼓包、手臂
                 都够不着山峰线，天然被忽略。
        办法二「数手指缝」：凸包凹陷法（老办法），作对照和兜底。

说明：代码里的中文只是注释；画面和终端上显示的是英文，
      因为 OpenCV 画不出中文字。
      Fingers = 手指数, Mode = 模式, fps = 每秒多少帧
"""

import os
import sys
import time
import math

import cv2
import numpy as np

# ==========================================================
# 第一段：能调的参数都在这，新手先只看这一段
# ==========================================================

CAMERA_INDEX = 0          # 摄像头编号，0 = 默认那个（笔记本自带的）

FRAME_W, FRAME_H = 640, 480     # 摄像头分辨率（和 PL 的上限一致）
DETECT_W, DETECT_H = 320, 240   # 找手时用小图：快 4 倍，效果几乎一样

DETECT_EVERY = 3          # 旧参数；新演示在 main.py 中允许每 1/2/3 帧识别
STABLE_TIMES = 3          # 连续 3 次认出同样的手指数，才切换模式（防抖）

#### 肤色范围：把手放进画面中央黄框按 c 自动校准 ####
SKIN_LOW = np.array([0, 18, 45], np.uint8)      # 下限
SKIN_HIGH = np.array([20, 150, 255], np.uint8)  # 上限
LAST_CALIBRATION = {"ok": False, "reason": "not_started"}
LAST_REJECT_REASON = "not_started"   # 最近一次"为什么判定画面里没手"（排查误检用）

CAL_BOX_FRAC = 0.28       # 中央校准黄框的边长（占画面宽度比例）

MIN_AREA = 3000           # 小于这个面积的色块不当成手（滤掉噪声）


######################################################################
# 下面这些数都能用环境变量现场改，不用改代码、不用重新部署。
# 板上调参示例（Windows Git Bash / Linux 通用）：
#     MAX_SOLIDITY=0.92 python main.py
# 板子(PYNQ)上：
#     export MAX_SOLIDITY=0.92; python3 main.py
######################################################################
def _env_float(name, default):
    try:
        return float(os.environ[name])
    except (KeyError, ValueError):
        return default


#### 人脸检测的开销控制（2026-10-18 新增）####
# Haar 找脸很贵：PC 上 1.47 ms，按实测 PC->板子约 20x 放大，板上约 30 ms/帧，
# 每帧都跑的话光这一项就把帧率压到 33 fps。
# 默认 1.0 = 不缩放。理由：半尺寸只快 1.7x，却会丢细节、可能漏掉人脸，
# 人脸守卫失效的代价比这点性能大。真正省钱的是 session.py 里的 FACE_EVERY。
# 只有在板上实测 fps 确实不够时，再考虑开：
#     FACE_SCALE=0.5  -> 快 1.7x，最小脸尺寸同步缩小，检测能力基本不变
#     （注意：若只缩图不缩 minSize，能测到的最小脸会凭空翻倍，远端小脸全漏，
#       代码里已经同步缩了，别改回去）
FACE_SCALE = _env_float("FACE_SCALE", 1.0)

#### 「这是不是一只手」的形状判据（2026-10-18 新增，治"没手却数出手指"） ####
# 背景里的大片暖色 / 过曝区域会被判成皮肤，面积几万，比真手还大。
# 面积只有下限挡不住它，所以要加"上限 + 形状"两道关。
MAX_AREA_FRAC = _env_float("MAX_AREA_FRAC", 0.35)
                          # 面积上限 = 检测图总面积的 35%（≈26900 px @320x240）。
                          #   实测：真手 3000~9000；整帧误检 30000+
                          #   取 0.35 而不是建议的 0.30：合成手形最大已到 22200，
                          #   手再凑近镜头一点就会被 0.30 误杀
MIN_SOLIDITY = _env_float("MIN_SOLIDITY", 0.65)
                          # 密实度 = 轮廓面积 / 凸包面积 的下限。
                          #   实测真手 0.75~0.78，建议值 0.72 只剩 0.03 余量太危险，
                          #   故取 0.65（掩膜有毛刺时密实度还会再降一点）
MIN_ASPECT = _env_float("MIN_ASPECT", 0.30)
                          # 短边/长边 下限：挡细长条（桌沿、衣领、手臂残边）。
                          #   实测真手 0.52~0.74，0.30 留足余量
MAX_SOLIDITY = _env_float("MAX_SOLIDITY", 0.88)
                          # 密实度【上限】：太"实心"的块不是手。
                          #   手一定有指缝，指缝会"抠掉"面积，所以密实度偏低；
                          #   墙壁/纸箱/衣服这种整块色斑密实度接近 1。
                          #   实测（都过 make_skin_mask 之后的真实数值）：
                          #       真手 0.772~0.841    误检色块 0.923~0.974
                          #   取 0.88 = 间隙中点，两边各留约 0.04 余量
                          #   （副作用是"握拳"会被判非手，但握拳不是我们的目标手势，
                          #    反而避免了握拳被数出怪数字，是好事）

# ---- 挥手跟踪专用窗口（与上面的"像不像手"完全独立）----
# 挥手只要"一块肤色在移动"，不要求它长得像标准手。
# 现场踩过：张开手掌正面朝镜头、或手离得稍远时，320x240 上手指被掩膜连成一片，
# solidity 冲到 0.9+ 被 MAX_SOLIDITY 拒掉 → hand area 恒为 0 → 挥手永远不触发。
TRACK_MIN_FRAC = _env_float("TRACK_MIN_FRAC", 0.012)   # 太小 = 噪点
TRACK_MAX_FRAC = _env_float("TRACK_MAX_FRAC", 0.45)    # 太大 = 整面墙/背景

MIN_ROUGH = _env_float("MIN_ROUGH", 1.14)
                          # 周长比【下限】= 轮廓周长 / 凸包周长。
                          #   手的边缘绕着指缝进进出出，周长远大于"外面套个橡皮筋"；
                          #   规整色块的周长几乎等于凸包周长（比值≈1.0）。
                          #   实测（同上，真实路径）：
                          #       真手 1.172~1.661    误检色块 1.054~1.112
                          #   取 1.14 = 间隙中点，两边各留约 0.03 余量
                          #
                          #   ！！！量这个数必须走真实路径（过 make_skin_mask）。
                          #   直接量原始合成图会高估约 0.07，阈值就会定得太紧：
                          #   曾经按原始图定 1.25，结果把 scale=0.8 的 1 指手
                          #   （实测只有 1.236）直接误杀了。
                          #
                          # 上板后如果发现"真手被误杀"（手伸出来但识别不出），
                          # 请按这个顺序放宽：先 MIN_ROUGH=1.05，再 MAX_SOLIDITY=0.92。

BORDER_MARGIN = 5         # 贴着画面边缘的候选点不要（手伸出画面时，
                          #   手和边缘的交角会被误当成指尖）

#### 数手指：两套办法一起用，互相验证 ####
# 办法一「找指尖」：轮廓上"离手重心距离"曲线的显著山峰 = 指尖
K_FRAC = 24               # 曲线平滑窗口 = 轮廓点数/K_FRAC
BASE_TIP_REACH = 0.69     # "山峰线"：只有离手重心超过最远点的 69% 才算指尖
BASE_DIP_RATIO = 0.13     # 两峰之间的"谷"要比最高峰低 13% 才算分开的两根手指
TIP_SEP_RATIO = 0.22      # （用于手指缝聚类）
FAN_DEG = 90              # 手指都朝大致同一个方向伸：偏离基准方向
                          #   90 度以上的假峰（手腕那头）剔除
                          #   2026-11-01 由 75 放宽到 90：五指自然张开时扇形可到
                          #   90~100 度，75 会把张开的小指/拇指自己人误伤。
                          #   放宽会不会放进来"手腕那头"的假峰？不会——
                          #   因为基准的选法同时改了（见 count_fingers 里的
                          #   "同向伙伴最多的簇"），不再由那个最远点（往往是手腕）
                          #   来定方向。两处要一起看。

# 办法二「数手指缝」：凸包凹陷法，老办法，作为对照/兜底
MAX_ANGLE = 90            # 手指缝的夹角上限（度）
BASE_DEFECT_RATIO = 0.058 # 手指缝深度至少是"手掌半径"的 5.8%（手远近自动适应）
DEFECT_DEPTH = 800        # 额外加的绝对下限（OpenCV 定点数，除以 256 才是像素）

#### 严格程度（唯一的旋钮）：数多了按 +，数少了按 - ####
# STRICT 越大越严格 -> 山峰线更高、谷要更深、缝要更深 -> 数出来的手指变少
STRICT = 1.0
TIP_REACH = BASE_TIP_REACH
DIP_RATIO = BASE_DIP_RATIO
DEFECT_RATIO = BASE_DEFECT_RATIO


def apply_strict(value):
    """按"严格程度"重算上面那几个阈值。改这一个数就够了。"""
    global STRICT, TIP_REACH, DIP_RATIO, DEFECT_RATIO
    STRICT = max(0.5, min(2.5, round(value, 2)))
    TIP_REACH = min(0.92, BASE_TIP_REACH * STRICT)
    DIP_RATIO = BASE_DIP_RATIO * STRICT
    DEFECT_RATIO = BASE_DEFECT_RATIO * STRICT
    print("[strict] %.2f -> reach=%.2f, dip=%.2f, gap=%.3f R"
          % (STRICT, TIP_REACH, DIP_RATIO, DEFECT_RATIO))

#### 亮度与曝光（这一组直接影响流畅度 fps）####
BRIGHT_TARGET = 110.0     # 画面平均亮度的目标值（0~255），不够就软件拉亮
CONTRAST_SPAN_OK = 90     # 灰度 5%~95% 跨度低于这个数 = 画面灰蒙蒙，需要拉对比度
EXPOSURE_LIST = [-9, -8, -7, -6, -5]   # 候选曝光，由快到慢；越慢越亮但越卡
EXP_TARGET = 70.0         # 自动挑曝光时想要的亮度
FPS_FLOOR = 25.0          # 自动挑曝光时，帧率不能低于这个值

FACE_OVERLAP = 0.60       # 皮肤块有 60% 落在脸框里 -> 当成脸，不当手
FACE_AREA_MARGIN = 2.0    # 但比脸框大 2 倍以上的大块，仍然当手（手挡在脸前）

MODE_NAME = {0: "GRAY", 1: "SOBEL", 2: "EMBOSS"}


_LUT_CACHE = {}           # 亮度查找表缓存，避免每帧重算


def get_lut(gamma):
    """按 gamma 生成/取缓存的一张 256 项查色表。"""
    key = round(gamma, 2)
    lut = _LUT_CACHE.get(key)
    if lut is None:
        lut = np.array([((i / 255.0) ** key) * 255.0
                        for i in range(256)]).astype("uint8")
        _LUT_CACHE[key] = lut
    return lut


def measure_brightness(frame):
    """量一下画面平均亮度（在小图上量，省时间）。"""
    small = frame if frame.shape[1] <= 320 else \
        cv2.resize(frame, (160, 120), interpolation=cv2.INTER_NEAREST)
    return float(cv2.cvtColor(small, cv2.COLOR_BGR2GRAY).mean())


def gamma_for(mean):
    """亮度不够时算 gamma：返回 1.0 表示不用拉亮。
    下限 0.45（最多拉亮约 2.2 倍）：拉得再狠就会把对比度也抹平，
    画面发白发灰——剩下的亮度缺口交给对比度拉伸和开灯。"""
    if mean >= BRIGHT_TARGET or mean < 1:
        return 1.0
    g = math.log(BRIGHT_TARGET / 255.0) / math.log(mean / 255.0)
    return max(0.45, min(g, 1.0))


def apply_gamma(frame, gamma):
    """按 gamma 拉亮一张图（约 0.25 ms，几乎不要钱）。"""
    if gamma >= 0.999:
        return frame
    return cv2.LUT(frame, get_lut(gamma))


def contrast_fix_params(small):
    """
    量一张小图的对比度：取灰度第 5~95 百分位的跨度。
    跨度太小（灰蒙蒙）就返回线性拉伸参数，把对比度拉回正常；
    对比度本来就够就返回 None（什么都不做）。
    """
    gray = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)
    p5, p95 = np.percentile(gray, (5, 95))
    span = float(p95 - p5)
    if span >= CONTRAST_SPAN_OK:
        return None
    # 目标：把跨度拉到 90 左右。放大倍数最多 2.2，免得噪声也被放大
    alpha = min(2.2, 90.0 / max(span, 5.0))
    beta = 128.0 * (1.0 - alpha)          # 围绕中间灰 128 拉伸，亮度不变
    return alpha, beta


def apply_contrast(frame, params):
    """按 contrast_fix_params 给的参数做线性拉伸（约 1 ms）。"""
    if params is None:
        return frame
    alpha, beta = params
    # convertScaleAbs takes abs() of negative values and can brighten black pixels.
    return np.clip(frame.astype(np.float32) * alpha + beta, 0, 255).astype(np.uint8)


def auto_brighten(frame):
    """太暗就软件拉亮。保留这个旧接口给测试用。"""
    return apply_gamma(frame, gamma_for(measure_brightness(frame)))


# ==========================================================
# 第二段：摄像头（照抄即可，不用改）
# ==========================================================

# Camera lifecycle lives in session.py; avoid old machine-specific exposure assumptions.

def make_skin_mask(bgr):
    """
    输入一张 BGR 小图，输出一张"手是白色、其他是黑色"的二值图。
    步骤：模糊降噪 -> 转 HSV -> 按肤色范围抠 -> 形态学开/闭运算去毛刺
    """
    blur = cv2.GaussianBlur(bgr, (5, 5), 0)
    hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, SKIN_LOW, SKIN_HIGH)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=1)
    return mask


def load_face_detector():
    """
    加载人脸检测器.xml（离线文件，不需要联网）。
    优先用本文件夹里自带的那个（OpenCV 5 的 wheel 不再自带了，我已下载好）。
    """
    here = os.path.dirname(os.path.abspath(__file__))
    paths = [os.path.join(here, "haarcascade_frontalface_default.xml")]
    data_dir = getattr(getattr(cv2, "data", None), "haarcascades", "")
    if data_dir:
        paths.append(os.path.join(data_dir, "haarcascade_frontalface_default.xml"))
    for path in paths:
        if os.path.exists(path):
            cascade = cv2.CascadeClassifier(path)
            if not cascade.empty():
                return cascade
    print("[face] cascade file not found, face-skip disabled")
    return None


def detect_faces(small_bgr, face_cascade):
    """
    在小图上找人脸，返回 [(x, y, w, h), ...]。

    契约（重要，别改）：进来的图是"检测尺寸"(320x240)，出去的框也是"检测尺寸"坐标。
    内部为了快会在缩小的图上找，但框会放大回原尺寸再返回，
    所以调用方（含测试和 mock）完全不用关心 FACE_SCALE。
    """
    if face_cascade is None or face_cascade.empty():
        return []
    if FACE_SCALE != 1.0:
        h = max(1, int(round(small_bgr.shape[0] * FACE_SCALE)))
        w = max(1, int(round(small_bgr.shape[1] * FACE_SCALE)))
        img = cv2.resize(small_bgr, (w, h), interpolation=cv2.INTER_AREA)
        # minSize 也要一起缩，否则"能检测到的最小脸"会凭空变大一倍，
        # 远端/小脸就漏掉了（这是容易踩的坑）
        min_side = max(20, int(round(60 * FACE_SCALE)))
        inv = 1.0 / FACE_SCALE
    else:
        img = small_bgr
        min_side = 60
        inv = 1.0
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    boxes = face_cascade.detectMultiScale(gray, scaleFactor=1.3,
                                          minNeighbors=6, minSize=(min_side, min_side))
    if inv == 1.0:
        return [tuple(map(int, box)) for box in boxes]
    return [(int(round(x * inv)), int(round(y * inv)),
             int(round(w * inv)), int(round(h * inv))) for (x, y, w, h) in boxes]


def hand_looks_real(cnt, shape=None):
    """
    判断一块皮肤色区域"像不像一只手"。返回 (是否像, 原因)。

    五道关，任何一道不过就判"不是手"：
      1. 面积上限  —— 挡"整帧误检"（背景一大片被判成皮肤，面积几万）
      2. 密实度下限—— 挡细长 / 破碎的块（桌沿、衣领、手臂残边）
      3. 密实度上限—— 挡规整实心色块（书、纸箱、墙壁、暖色衣服）
                      手一定有指缝，不可能"密不透风"
      4. 周长比下限—— 第二道保险：手边缘绕指缝凹凸，周长明显长于凸包
      5. 长短边比  —— 再挡一次细长条

    全部阈值都能用环境变量现场改（不用改代码、不用重启重装）：
      MAX_AREA_FRAC / MIN_SOLIDITY / MAX_SOLIDITY / MIN_ROUGH / MIN_ASPECT
    """
    area = cv2.contourArea(cnt)
    if shape is not None:                 # shape = 掩膜尺寸，用来算面积上限
        pix = float(shape[0] * shape[1])
        if area > MAX_AREA_FRAC * pix:
            return False, "hand_too_big"
    hull = cv2.convexHull(cnt)
    hull_area = cv2.contourArea(hull)
    if hull_area <= 0:
        return False, "not_hand_shape"
    solidity = area / hull_area
    if solidity < MIN_SOLIDITY:
        return False, "too_ragged"        # 太破碎：噪声 / 边缘残块
    if solidity > MAX_SOLIDITY:
        return False, "too_solid"         # 太实心：不像手，像一整块色斑
    hull_peri = cv2.arcLength(hull, True)
    if hull_peri <= 0:
        return False, "not_hand_shape"
    if cv2.arcLength(cnt, True) / hull_peri < MIN_ROUGH:
        return False, "too_smooth"        # 边缘太光滑：没有指缝
    x, y, w, h = cv2.boundingRect(cnt)
    if max(w, h) == 0 or min(w, h) / float(max(w, h)) < MIN_ASPECT:
        return False, "too_thin"
    return True, "ok"


def pick_hand_contour(contours, faces, shape=None):
    """
    在所有皮肤块里挑一块最像手的：面积最大，但不是脸。

    注意这里刻意"不去擦除"任何像素——老做法是把脸框直接涂黑，
    结果 Haar 经常把手误认成脸，手被涂没了，就只能识别出 0 根手指。
    现在改成：挑轮廓时跳过像脸的那块，一个像素都不改，
    最坏情况也只是跳过了错误的一块，绝不会把手的形状擦坏。
    """
    ranked = sorted(contours, key=cv2.contourArea, reverse=True)
    last_reason = None                    # 记下"最大的那块为什么被否"，方便排查
    for cnt in ranked:
        area = cv2.contourArea(cnt)
        x, y, w, h = cv2.boundingRect(cnt)
        like_face = False
        for (fx, fy, fw, fh) in faces:
            ox = max(0, min(x + w, fx + fw) - max(x, fx))
            oy = max(0, min(y + h, fy + fh) - max(y, fy))
            covered = (ox * oy) / float(max(1, w * h))
            too_big_for_face = area > FACE_AREA_MARGIN * fw * fh
            if covered > FACE_OVERLAP and not too_big_for_face:
                like_face = True
                break
        if like_face:
            continue
        # 2026-10-18：以前这里"无脑返回最大的一块"，于是整帧误检（面积几万的
        # 背景块）会被当成手，在它的边缘噪声上数出 1~2 根手指 -> 滤镜自己乱跳。
        # 现在必须过"像不像手"这一关；全都不像手就返回 None = 画面里没手。
        ok, why = hand_looks_real(cnt, shape)
        if ok:
            return cnt
        if last_reason is None:
            last_reason = why
    # 走到这里 = 一块像手的都没有。别硬凑一只出来，老老实实说"没手"。
    global LAST_REJECT_REASON
    LAST_REJECT_REASON = last_reason or ("face_rejected" if faces else "no_hand_shape")
    return None


def cal_frame_box(frame):
    """返回画面中央校准框的 (x1, y1, x2, y2)。"""
    h, w = frame.shape[:2]
    bw = int(w * CAL_BOX_FRAC)
    x1 = (w - bw) // 2
    y1 = (h - bw) // 2
    return x1, y1, x1 + bw, y1 + bw


def calibrate_skin(frame):
    """
    一键校准肤色：取画面中央黄框里的像素，统计实际肤色的 HSV 中位数，
    按它重设阈值。换光线 / 换人 / 换摄像头之后，按一下 c 就行。

    安全措施：
    1. 只统计框里"较亮的那一半"像素，减少阴影干扰
    2. 算完后先在框里验证：如果框里被认成皮肤的像素不足 35%，
       说明你还没把手掌放好，这次校准作废（保留原阈值）
    """
    global SKIN_LOW, SKIN_HIGH, LAST_CALIBRATION
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    x1, y1, x2, y2 = cal_frame_box(frame)
    box = hsv[y1:y2, x1:x2]

    v = box[:, :, 2]
    bright = box[v >= np.percentile(v, 50)]          # 较亮的一半像素，形状 (N, 3)
    h_med = float(np.median(bright[:, 0]))
    s_med = float(np.median(bright[:, 1]))
    v_med = float(np.median(bright[:, 2]))
    LAST_CALIBRATION = {"ok": False, "reason": "sampling", "h": h_med,
                        "s": s_med, "v": v_med}
    print("[calibrate] sampled HSV median: H=%.0f S=%.0f V=%.0f"
          % (h_med, s_med, v_med))

    # ---- 防呆：采到的颜色不像皮肤（太灰 / 太黑）就拒绝 ----
    # Match the default mask's saturation floor. A pale hand with S=18..29
    # must not pass detection but fail calibration solely for being pale.
    if s_med < 18 or v_med < 40:
        LAST_CALIBRATION["reason"] = "sample_too_gray_or_dark"
        print("[calibrate] FAILED: the yellow box does not look like skin "
              "(too gray or too dark).")
        print("[calibrate] -> Put your PALM over the whole yellow box, "
              "add more light, then press c again.")
        return False

    # ---- 防呆 2（2026-10-21 追加）：框里如果和画面四周几乎同色，就是采到了背景 ----
    # 现场踩过：手掌没盖住黄框，采到白墙（S=18 V=147），阈值被设成 S≥10，
    # 结果整面墙都算"皮肤" → 轮廓巨大 → gesture 永远判不出手。
    # 真手在框里时，框内中位色必然与画面周围有明显差异。
    try:
        hh, ww = frame.shape[:2]
        margin = max(8, int(min(hh, ww) * 0.04))
        border = np.concatenate([
            hsv[0:margin, :, :].reshape(-1, 3),
            hsv[hh - margin:hh, :, :].reshape(-1, 3),
            hsv[:, 0:margin, :].reshape(-1, 3),
            hsv[:, ww - margin:ww, :].reshape(-1, 3)], axis=0)
        bh = float(np.median(border[:, 0]))
        bs = float(np.median(border[:, 1]))
        bv = float(np.median(border[:, 2]))
        dh = abs(h_med - bh)
        dh = min(dh, 180 - dh)                      # 色相是环形的
        LAST_CALIBRATION.update(bg_h=bh, bg_s=bs, bg_v=bv)
        print("[calibrate] border median: H=%.0f S=%.0f V=%.0f" % (bh, bs, bv))
        if dh <= 8 and abs(s_med - bs) <= 14 and abs(v_med - bv) <= 22:
            LAST_CALIBRATION["reason"] = "box_looks_like_background"
            print("[calibrate] FAILED: 框里的颜色和画面四周几乎一样 —— "
                  "采到的是背景，不是手。")
            print("[calibrate] -> 把手掌（不是手指尖）整个盖住黄框，再按 c。")
            return False
    except Exception as exc:
        print("[calibrate] border check skipped:", exc)

    new_low = np.array([max(0, int(h_med) - 14),
                        max(10, int(s_med * 0.4)),
                        max(20, int(v_med * 0.4))], np.uint8)
    new_high = np.array([min(179, int(h_med) + 14), 255, 255], np.uint8)

    # ---- 验证：新阈值下，黄框里得有足够多的像素被认成皮肤 ----
    inside = cv2.inRange(box, new_low, new_high)
    frac = float(np.count_nonzero(inside)) / inside.size
    LAST_CALIBRATION["skin_fraction"] = frac
    if frac < 0.35:
        LAST_CALIBRATION["reason"] = "palm_does_not_cover_box"
        print("[calibrate] FAILED: only %.0f%% of the yellow box looked like "
              "skin." % (frac * 100))
        print("[calibrate] -> Cover the WHOLE yellow box with your palm, "
              "then press c again.")
        return False

    SKIN_LOW, SKIN_HIGH = new_low, new_high
    LAST_CALIBRATION.update(ok=True, reason="ok")
    print("[calibrate] OK (%.0f%% of box).  SKIN_LOW = %s  SKIN_HIGH = %s"
          % (frac * 100, SKIN_LOW.tolist(), SKIN_HIGH.tolist()))
    return True


def angle_at(far, p1, p2):
    """计算 far 这个点上的夹角（度）。手指缝的夹角通常是锐角。"""
    a = math.dist(p1, p2)
    b = math.dist(far, p1)
    c = math.dist(far, p2)
    if b * c == 0:
        return 180.0
    cosv = (b * b + c * c - a * a) / (2 * b * c)
    cosv = max(-1.0, min(1.0, cosv))
    return math.degrees(math.acos(cosv))


def cluster_points(points, sep):
    """把挨得近的点归成一簇（用于手指缝）。返回簇中心列表。"""
    groups = []
    for p in points:
        placed = False
        for g in groups:
            cx = sum(q[0] for q in g) / float(len(g))
            cy = sum(q[1] for q in g) / float(len(g))
            if math.dist(p, (cx, cy)) < sep:
                g.append(p)
                placed = True
                break
        if not placed:
            groups.append([p])
    centers = []
    for g in groups:
        cx = sum(q[0] for q in g) / float(len(g))
        cy = sum(q[1] for q in g) / float(len(g))
        centers.append((int(cx), int(cy)))
    return centers


def find_palm(mask):
    """
    找手掌圆心：距离变换 = 每个皮肤像素到"非皮肤"的最近距离，
    最大的那个点就是手掌内切圆的圆心（手掌是手上最宽的地方）。
    返回 (x, y) 和半径。
    """
    dist = cv2.distanceTransform(mask, cv2.DIST_L2, 5)
    _, maxv, _, maxloc = cv2.minMaxLoc(dist)
    return maxloc, float(maxv)


def find_fingertips(cnt, palm_c):
    """
    办法一：以手掌圆心为原点，沿手的轮廓走一圈，
    算每个点离手掌圆心的距离，得到一条曲线。手指尖 = 曲线上的"山峰"。

    以手掌圆心为原点是关键：
      - 指尖离它最远（1.5~2 倍手掌半径）
      - 手臂、手腕、指关节鼓包都够不着"山峰线"，自动出局
      - 圆滑的指尖帽天然只有一个峰，不会一个指尖数成两个
    """
    pts = cnt.reshape(-1, 2).astype(float)
    n = len(pts)
    if n < 12:
        return []
    d = np.linalg.norm(pts - np.asarray(palm_c, float), axis=1)

    # 环形平滑，去掉毛刺
    k = max(3, n // K_FRAC)
    kernel = np.ones(k) / k
    s = np.convolve(np.concatenate([d[-k:], d, d[:k]]),
                    kernel, mode="same")[k:-k]

    smax = float(s.max())
    if smax < 1e-6:
        return []
    above = s >= TIP_REACH * smax

    # 连续高于山峰线的一段 = 一根（或几根并拢的）手指
    runs = []
    i = 0
    while i < n:
        if above[i]:
            j = i
            while j < n and above[j]:
                j += 1
            runs.append([i, j])
            i = j
        else:
            i += 1
    if len(runs) > 1 and runs[0][0] == 0 and runs[-1][1] == n:
        runs[0][0] = runs.pop()[0] - n          # 首尾相接 -> 合并成一段

    # 段内再按"谷深"拆分：并得很拢的两根手指会在同一段里，
    # 但中间总有个坑。用"峰-谷交替"的状态机找峰：
    #   从峰掉下去超过 DIP -> 确认一个峰，开始找谷
    #   从谷爬上来超过 DIP -> 确认一个谷，开始找下一个峰
    dip_need = DIP_RATIO * smax
    tips = []
    for start, end in runs:
        idx = np.arange(start, end) % n          # 环绕段也能正确取到下标
        seg = s[idx]
        peak_v = float(seg[0])
        peak_i = 0
        valley_v = float(seg[0])
        looking = "peak"                         # 当前在找峰还是找谷
        for t in range(1, len(seg)):
            v = float(seg[t])
            if looking == "peak":
                if v > peak_v:
                    peak_v, peak_i = v, t
                elif peak_v - v >= dip_need:
                    tips.append(int(idx[peak_i]))
                    valley_v = v
                    looking = "valley"
            else:
                if v < valley_v:
                    valley_v = v
                elif v - valley_v >= dip_need:
                    peak_v, peak_i = v, t
                    looking = "peak"
        if looking == "peak":
            tips.append(int(idx[peak_i]))        # 收尾：最后还在找的那个峰

    uniq = []
    for t in tips:
        if t not in uniq:
            uniq.append(t)
    return [(int(pts[t][0]), int(pts[t][1])) for t in uniq]


def find_finger_gaps(cnt, radius):
    """
    办法二：数手指缝（凸包凹陷）。
    原理：手外面套一层橡皮筋，手指缝那里绷不紧会凹进去。
    关键是阈值要跟着手的大小走（手离镜头远近会变），所以乘了 DEFECT_RATIO。
    """
    hull_idx = cv2.convexHull(cnt, returnPoints=False)
    if hull_idx is None or len(hull_idx) < 4 or len(cnt) < 4:
        return []
    defects = cv2.convexityDefects(cnt, hull_idx)
    if defects is None:
        return []
    need = max(DEFECT_DEPTH, DEFECT_RATIO * radius * 256.0)
    far_pts = []
    for s, e, f, depth in np.asarray(defects).reshape(-1, 4):
        if depth < need:
            continue                                   # 凹得太浅
        start, end, far = tuple(cnt[s][0]), tuple(cnt[e][0]), tuple(cnt[f][0])
        if angle_at(far, start, end) > MAX_ANGLE:
            continue                                   # 缺口太宽
        far_pts.append(far)
    return far_pts


def inside_frame(points, shape, margin=BORDER_MARGIN):
    """丢掉贴着画面边界的点（手伸出画面时边界交角是假指尖）。"""
    if not points:
        return []
    h, w = shape[:2]
    return [p for p in points
            if margin <= p[0] <= w - 1 - margin
            and margin <= p[1] <= h - 1 - margin]


def count_fingers(mask, debug_img=None, faces=()):
    """
    数手指。返回 (手指数, 调试图, 明细字典)

    用两套办法各数一遍，再合成一个结果：
      指尖法 —— 数"伸出来的、尖尖的"手指，对"旁边还有没伸直的手指"免疫
      缝隙法 —— 数手指之间的缝 + 1，老办法，作为交叉验证
    """
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return 0, debug_img, {"tips": 0, "gaps": 0, "reason": "no_skin", "area": 0,
                              "track_center": None, "track_area": 0.0}

    # ---- 挥手专用跟踪信号（2026-10-21 追加，与"数手指"完全解耦）----
    # 挥手只需要知道"那块肤色在往哪个方向走"，不需要它长得像一只标准的手。
    # 现场踩过：张开手掌正面朝镜头时 solidity ≈0.9，被 hand_looks_real 的 0.88 上限拒掉
    # → hand area 恒为 0 → 挥手永远不触发（而掩膜其实是好的）。
    # 这里用一块独立、宽松的判据：只要最大肤色块的面积在合理范围内，就给出质心。
    track_c, track_a = None, 0.0
    try:
        big = max(contours, key=cv2.contourArea)
        track_a = float(cv2.contourArea(big))
        frac = track_a / float(mask.shape[0] * mask.shape[1])
        if TRACK_MIN_FRAC <= frac <= TRACK_MAX_FRAC:
            mm = cv2.moments(big)
            if mm["m00"] > 1e-6:
                track_c = [float(mm["m10"] / mm["m00"]) / mask.shape[1],
                           float(mm["m01"] / mm["m00"]) / mask.shape[0]]
        else:
            track_a = 0.0          # 太大（整面墙）或太小（噪点）都不跟踪
    except Exception as exc:
        print("[track] centroid failed:", exc)

    cnt = pick_hand_contour(list(contours), list(faces), mask.shape)
    if cnt is None:
        # 没有任何一块"像手"：可能是被当成脸跳过了，也可能是整帧误检/形状不像。
        # 两种情况都按"画面里没有手"处理，绝不猜一个手指数出来。
        # reason 用"最大的那块为什么被否"，现场排查误检时能直接读出是哪道关卡的。
        return 0, debug_img, {"tips": 0, "gaps": 0,
                              "reason": LAST_REJECT_REASON, "area": 0,
                              "track_center": track_c, "track_area": track_a}
    area = cv2.contourArea(cnt)
    if area < MIN_AREA:
        return 0, debug_img, {"tips": 0, "gaps": 0, "reason": "hand_too_small", "area": area,
                              "track_center": track_c, "track_area": track_a}
    ok, why = hand_looks_real(cnt, mask.shape)
    if not ok:
        # 显式分支：调试时能看出是被哪条规则拒的
        return 0, debug_img, {"tips": 0, "gaps": 0, "reason": why, "area": area,
                              "track_center": track_c, "track_area": track_a}

    radius = math.sqrt(area / math.pi)                 # 手的等效半径，用来自适应阈值
    sep = max(8, int(TIP_SEP_RATIO * radius))

    if debug_img is not None:
        cv2.drawContours(debug_img, [cnt], -1, (0, 255, 0), 2)

    # Find the palm in the selected contour, not in a larger face/background blob.
    hand_mask = np.zeros_like(mask)
    cv2.drawContours(hand_mask, [cnt], -1, 255, cv2.FILLED)
    palm_c, palm_r = find_palm(hand_mask)

    # ---- 办法一：指尖 ----
    tips = inside_frame(find_fingertips(cnt, palm_c), mask.shape)
    # A fingertip needs to extend beyond the palm itself; reject compact round blobs.
    # 2026-11-01 由 1.45 放宽到 1.30：手离镜头近时 palm_r 变大，
    # 并拢（或张得不开）的手指够不到 1.45 倍就被过滤掉了。
    tips = [p for p in tips if math.dist(p, palm_c) >= 1.30 * palm_r]

    # 扇形过滤：手指都朝着大致同一个方向，方向差超过 FAN_DEG 的剔除。
    #
    # 2026-11-01 改（治"伸 5 指只显示 1 指"）：
    #   原来取"离掌心最远的那个点"当基准方向。手腕入画时，"离掌心最远"的往往
    #   不是指尖而是手腕——基准一指向手腕，真手指就全落在反方向（夹角接近
    #   180 度），被当成"偏离方向的假峰"全部剔掉，最后只剩手腕自己那一根。
    #   演示时最自然的姿势（手掌朝下、手腕朝上）必然触发，属于必现 bug。
    #
    #   改成取「同向伙伴最多」的那一簇的方向当基准：
    #   手指是多根扎堆朝一个方向的，手腕通常只有孤零零一个峰。
    #   选"同伙最多"而不是"最远"，手腕就夺不走基准权。
    if len(tips) > 1:
        def _ad(a, b):
            """两个角度之间的最小夹角（0~180）。"""
            return abs((a - b + 180.0) % 360.0 - 180.0)

        angs = [math.degrees(math.atan2(t[1] - palm_c[1],
                                        t[0] - palm_c[0])) for t in tips]
        # 选「同向伙伴最多」的点做基准，而不是「离掌心最远」的点
        best = max(angs, key=lambda a: sum(1 for b in angs if _ad(a, b) <= FAN_DEG))
        kept = [t for t, a in zip(tips, angs) if _ad(a, best) <= FAN_DEG]
        if kept:
            tips = kept

    if debug_img is not None:
        cv2.circle(debug_img, palm_c, 3, (0, 140, 255), -1)   # 橙点 = 手掌圆心
        for t in tips:
            cv2.circle(debug_img, t, 6, (0, 255, 255), -1)     # 黄点 = 手指尖
            cv2.line(debug_img, palm_c, t, (0, 160, 160), 1)

    # ---- 办法二：手指缝 ----
    gaps = inside_frame(cluster_points(find_finger_gaps(cnt, radius), sep), mask.shape)
    if debug_img is not None:
        for f in gaps:
            cv2.circle(debug_img, f, 4, (255, 0, 255), -1)     # 紫点 = 手指缝

    # ---- 合成 ----
    n_tips, n_gaps = len(tips), len(gaps)
    # No credible fingertip means "invalid", never an automatic one-finger command.
    # Gap counts remain diagnostics. Real fists/backgrounds still require live testing.
    fingers = min(n_tips, 5)
    # 2026-10-21 追加 hand_center（归一化 0~1），为挥手切换提供位置信号。
    # 纯增量：palm_c 本来就算出来了，只是没往外传；不改变任何判据、不影响数指结果。
    return fingers, debug_img, {"tips": n_tips, "gaps": n_gaps,
                                "reason": "ok" if fingers else "no_fingertips", "area": area,
                                "hand_center": [float(palm_c[0]) / mask.shape[1],
                                                float(palm_c[1]) / mask.shape[0]],
                                "hand_valid": bool(fingers),
                                "track_center": track_c, "track_area": track_a}


# ==========================================================
# 第四段：主循环
# ==========================================================

def main():
    # One entry point; the old prototype loop has been replaced by the PC demo.
    from main import main as run_demo
    return run_demo()


if __name__ == "__main__":
    sys.exit(main())
