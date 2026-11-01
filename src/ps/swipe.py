"""Time-based hand tracking. Inputs are normalized positions, never frame counts.

2026-10-21 重做版（现场反馈"根本挥不动"）
--------------------------------------------------------------------------
原版（软件组 swipe.py）要求：手先**几乎完全静止** 0.16 s（x 抖动 ≤0.04、y ≤0.05，
在 320x240 检测图上只有 12.8 px），建起 anchor 后再从 anchor 位移 ≥0.20、1.1 s 内完成。
现场三个问题：

  ① 手掌质心在肤色掩膜轻微闪动时就会跳 10~20 px → **anchor 永远建不起来**，
     所以怎么挥都没反应；
  ② travel 判定是 `dx >= 0.20`，而人一次自然挥动刚好在 0.18~0.22 之间，
     加上抖动吃掉的量，**大量真实挥手落在阈值下方**；
  ③ 面积比窗口 0.45~2.2 太窄，手挥动时离镜头稍近就 reset。

本版改成**滑动时间窗**判据，不再要求"先静止"：
  在最近 `window_s`（默认 0.6 s）内，若横向净位移 ≥ travel、路径直线度 ≥ linearity、
  纵向漂移 ≤ y_drift、且手在画面里已连续存在 ≥ min_history 秒 → 判一次挥手。
  这样"慢慢漂移"因为 0.6 s 窗口内位移不够而不会误触发；
  "手从画面边缘进入"因为起点必须落在 [0.12, 0.88] 内而不算。
"""
from collections import deque
import math


class SwipeTracker:
    def __init__(self, travel=0.16, cooldown=0.8, max_seconds=1.4,
                 max_gap=0.8, window_s=0.70, min_history=0.28,
                 linearity=0.72, y_drift=0.18,
                 area_lo=0.30, area_hi=3.0, edge=0.12):
        self.travel = travel
        self.cooldown = cooldown
        self.max_seconds = max_seconds      # 历史长度（滑动窗最长回溯）
        self.max_gap = max_gap              # 掉帧超过这个间隔就清空，避免跨帧误判
        self.window_s = window_s            # 判定用的时间窗
        self.min_history = min_history      # 手至少要连续可见这么久
        self.linearity = linearity
        self.y_drift = y_drift
        self.area_lo, self.area_hi = area_lo, area_hi
        self.edge = edge                    # 起点不能在画面这么靠边的位置
        self.event_id = 0
        self.blocked_until = -math.inf
        self.reset()

    def reset(self):
        self.points = deque()
        self.last_time = None
        self.last_area = None

    @property
    def anchor(self):
        """兼容旧字段：有足够历史 = 可以判定（不再要求真的静止过）。"""
        return bool(self.points)

    def state(self):
        now = self.last_time if self.last_time is not None else 0.0
        ready = bool(self.points) and (now - self.points[0][0]) >= self.min_history
        dx = span = speed = 0.0
        if len(self.points) >= 2:
            recent = [p for p in self.points if now - p[0] <= self.window_s] or [self.points[-1]]
            dx = recent[-1][1] - recent[0][1]
            span = max(0.0, recent[-1][0] - recent[0][0])
            speed = abs(dx) / span if span > 1e-6 else 0.0
        return {"ready": ready, "anchor": ready, "points": len(self.points),
                "dx": dx, "span": span, "speed": speed,
                "blocked": now < self.blocked_until}

    def update(self, position, now, valid=True, area=1.0):
        event = {"swipe": False, "swipe_dir": None, "swipe_id": self.event_id,
                 "ready": False, "anchor": False, "points": len(self.points),
                 "dx": 0.0, "span": 0.0, "speed": 0.0}
        if (not valid or position is None or not math.isfinite(now)
                or not all(math.isfinite(float(v)) and 0 <= v <= 1 for v in position)):
            self.reset()
            return event
        x, y = map(float, position)
        if self.last_time is not None and (now <= self.last_time or now - self.last_time > self.max_gap):
            self.reset()
        if self.last_area and not self.area_lo <= area / self.last_area <= self.area_hi:
            self.reset()
        self.last_time, self.last_area = now, max(float(area), 1e-9)
        if now < self.blocked_until:
            self.points.clear()
            return event

        self.points.append((now, x, y))
        while self.points and now - self.points[0][0] > self.max_seconds:
            self.points.popleft()

        ready = len(self.points) >= 2 and (now - self.points[0][0]) >= self.min_history
        event.update({"ready": ready, "anchor": ready})
        if not ready:
            event.update(self.state())
            return event

        recent = [p for p in self.points if now - p[0] <= self.window_s]
        # 手势线程在 ARM 上只有 3~4 Hz（每次 prepare+detect 要 130~300 ms），
        # 0.7 s 窗口里往往只有 2~3 个点。原来要求 >=3 个点 → 永远凑不够。
        # 现在允许 2 个点，但只在这种情况下把位移门槛提高 12%，以抵消少一个点的抗噪损失。
        if len(recent) < 2 or (now - recent[0][0]) < 0.25:
            event.update(self.state())
            return event

        x0, y0 = recent[0][1], recent[0][2]
        dx = recent[-1][1] - x0
        path = sum(abs(b[1] - a[1]) for a, b in zip(recent, recent[1:]))
        drift = max(abs(p[2] - y0) for p in recent)
        straight = (abs(dx) / path) if path else 0.0
        need = self.travel if len(recent) >= 3 else self.travel * 1.12

        if (abs(dx) >= need and straight >= self.linearity
                and drift <= self.y_drift and self.edge <= x0 <= 1.0 - self.edge):
            self.event_id += 1
            event.update(swipe=True, swipe_dir="right" if dx > 0 else "left",
                         swipe_id=self.event_id, dx=dx, straight=straight)
            self.blocked_until = now + self.cooldown
            self.reset()
            return event

        event.update(self.state())
        event["straight"] = straight
        return event
