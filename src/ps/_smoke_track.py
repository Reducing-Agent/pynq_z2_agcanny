# -*- coding: utf-8 -*-
"""
_smoke_track.py —— 验证"张开手掌"场景下挥手信号仍然可用（2026-10-21）

背景（真机踩坑）：
  用户反馈"伸手指模式正常，但挥手完全没反应，swipe 一直显示 no hand"。
  根因：hand_center 依赖 hand_looks_real()，其中 MAX_SOLIDITY=0.88 会拒掉
        "正面朝镜头的张开手掌"（手指在 320x240 掩膜上连成一片，solidity 0.9+）
        → hand area 恒为 0 → hand_center 永不产生 → tracker 收不到任何点。

本测试验证：
  A. 张开手掌（solidity≈1.0）→ hand 判据 reject（这是预期），但 track_center 有值
  B. 用 track_center 走完整滑动序列 → tracker 触发 swipe
  C. 空手（全黑）→ track_area=0，不触发（误报红线）
  D. 整面墙（面积 60%）→ 被 TRACK_MAX_FRAC 挡掉，不跟踪
  E. 噪点（几个像素）→ 被 TRACK_MIN_FRAC 挡掉

跑法： python _smoke_track.py
"""
import sys
import numpy as np

import cv2
import gesture as g
from swipe import SwipeTracker

W, H = 320, 240
FAIL = []


def check(name, cond, extra=""):
    tag = "PASS" if cond else "FAIL"
    if not cond:
        FAIL.append(name)
    print("[%s] %-46s %s" % (tag, name, extra))


def make_palm(cx, cy, r=34):
    """实心圆当"张开的手掌" —— solidity 恒为 1.0，必定过不了 MAX_SOLIDITY。"""
    m = np.zeros((H, W), np.uint8)
    cv2.circle(m, (int(cx), int(cy)), r, 255, -1)
    return m


def centroid_of(cnt):
    mm = cv2.moments(cnt)
    return (mm["m10"] / mm["m00"], mm["m01"] / mm["m00"])


# ---------------- A. 张开手掌：hand 拒、track 过 ----------------
print("\n=== A. 张开手掌（solidity=1.0，被 hand 判据拒，但 track 应可用）===")
mask = make_palm(160, 120)
fingers, _, detail = g.count_fingers(mask, None, ())

cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
c = max(cnts, key=cv2.contourArea)
s = cv2.contourArea(c) / cv2.contourArea(cv2.convexHull(c))
print("    solidity=%.3f  reason=%s  area=%s  track_area=%.0f  track_center=%s"
      % (s, detail.get("reason"), detail.get("area"),
         detail.get("track_area") or 0, detail.get("track_center")))

check("A1 solidity > MAX_SOLIDITY", s > g.MAX_SOLIDITY, "%.3f > %.2f" % (s, g.MAX_SOLIDITY))
check("A2 hand 判据确实拒绝", (detail.get("area") or 0) == 0, "area=%s" % detail.get("area"))
check("A3 hand_center 不存在", detail.get("hand_center") is None)
check("A4 track_area > 0", (detail.get("track_area") or 0) > 0,
      "%.0f px" % (detail.get("track_area") or 0))
check("A5 track_center 有值", detail.get("track_center") is not None,
      str(detail.get("track_center")))
if detail.get("track_center"):
    tx, ty = detail["track_center"]
    check("A6 质心落在圆心附近", abs(tx - 0.5) < 0.02 and abs(ty - 0.5) < 0.02,
          "(%.3f, %.3f)" % (tx, ty))

# ---------------- B. 用 track_center 走完整滑动 ----------------
print("\n=== B. 张开手掌横向挥动 → 应触发 swipe ===")


class Tick:
    """假时钟，便于确定性复现。"""
    t = 1000.0


tr = SwipeTracker(travel=0.16, cooldown=0.8)
Tick.t = 1000.0
fired = []
# 先在左侧停 0.35 s（建立 anchor），再 0.60 s 内横移到右侧
xs = [0.22] * 3 + [0.32, 0.42, 0.52, 0.62, 0.72]
dt = 0.105          # 3.2 Hz 的手势线程 ≈ 0.31 s/点，这里取更密的 0.105 做基准
for i, x in enumerate(xs):
    m = make_palm(x * W, 120)
    f, _, d = g.count_fingers(m, None, ())
    tpos = d.get("track_center")
    ta = float(d.get("track_area") or 0.0)
    Tick.t += dt
    ev = tr.update(tpos, Tick.t, valid=(ta > 0), area=max(ta, 1.0))
    if ev.get("swipe"):
        fired.append(("right", i, ev))
        print("    t=%.2fs  x=%.2f  -> SWIPE %s" % (Tick.t - 1000.0, x, ev.get("swipe_dir")))

check("B1 触发了一次挥手", len(fired) >= 1, "fired=%d" % len(fired))
check("B2 方向是 right", fired and fired[0][2].get("swipe_dir") == "right",
      str(fired[0][2].get("swipe_dir")) if fired else "n/a")

# ---------------- C. 空手 ----------------
print("\n=== C. 空手 60 s 等效（全黑掩膜）→ 不得误报 ===")
tr2 = SwipeTracker(travel=0.16, cooldown=0.8)
empty = np.zeros((H, W), np.uint8)
false_alarm = 0
for i in range(200):                     # 200 点 × 0.3 s ≈ 60 s
    f, _, d = g.count_fingers(empty, None, ())
    ta = float(d.get("track_area") or 0.0)
    ev = tr2.update(d.get("track_center"), 2000.0 + i * 0.3,
                    valid=(ta > 0), area=max(ta, 1.0))
    if ev.get("swipe"):
        false_alarm += 1
check("C1 空手 60 s 误报 0 次", false_alarm == 0, "误报=%d" % false_alarm)
check("C2 空手 track_area=0", (d.get("track_area") or 0) == 0, str(d.get("track_area")))

# ---------------- D. 整面墙 ----------------
print("\n=== D. 整面墙（面积 60%）→ 应被 TRACK_MAX_FRAC 挡掉 ===")
wall = np.zeros((H, W), np.uint8)
cv2.rectangle(wall, (20, 20), (300, 220), 255, -1)
# 注意：track_area 在超范围时会被主动归零，所以真实面积要从掩膜本身量
raw_frac = float(np.count_nonzero(wall)) / float(W * H)
f, _, d = g.count_fingers(wall, None, ())
frac = (d.get("track_area") or 0) / float(W * H)
print("    raw_frac=%.3f  track_frac=%.3f  reason=%s  track_center=%s"
      % (raw_frac, frac, d.get("reason"), d.get("track_center")))
check("D1 实际面积超过 TRACK_MAX_FRAC", raw_frac > g.TRACK_MAX_FRAC,
      "%.3f > %.2f" % (raw_frac, g.TRACK_MAX_FRAC))
check("D2 不产生 track_center", d.get("track_center") is None)
check("D3 track_area 归零", (d.get("track_area") or 0) == 0)

# ---------------- E. 噪点 ----------------
print("\n=== E. 噪点（< TRACK_MIN_FRAC）→ 应被挡掉 ===")
noise = np.zeros((H, W), np.uint8)
cv2.circle(noise, (100, 100), 5, 255, -1)      # 78 px / 76800 = 0.001
f, _, d = g.count_fingers(noise, None, ())
frac = (d.get("track_area") or 0) / float(W * H)
check("E1 面积低于 TRACK_MIN_FRAC", frac < g.TRACK_MIN_FRAC,
      "%.4f < %.3f" % (frac, g.TRACK_MIN_FRAC))
check("E2 不产生 track_center", d.get("track_center") is None)

# ---------------- F. 回归：正常三指手仍能数出手指 ----------------
print("\n=== F. 回归：正常手掌+3 指 仍能数出 3 ===")
hand = np.zeros((H, W), np.uint8)
cv2.ellipse(hand, (160, 150), (40, 45), 0, 0, 360, 255, -1)     # 掌心
for ang in (-35, 0, 35):
    a = np.radians(-90 + ang)
    x2 = int(160 + 85 * np.cos(a))
    y2 = int(150 + 85 * np.sin(a))
    cv2.line(hand, (160, 150), (x2, y2), 255, 16)
fingers, _, d = g.count_fingers(hand, None, ())
print("    fingers=%s reason=%s area=%.0f track_area=%.0f"
      % (fingers, d.get("reason"), d.get("area") or 0, d.get("track_area") or 0))
check("F1 数出 3 指", fingers == 3, "fingers=%s" % fingers)
check("F2 track_center 同时存在", d.get("track_center") is not None)

# ---------------- 汇总 ----------------
print("\n" + "=" * 56)
if FAIL:
    print("FAILED %d 项: %s" % (len(FAIL), ", ".join(FAIL)))
    sys.exit(1)
print("ALL PASS")
