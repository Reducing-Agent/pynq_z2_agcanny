"""新旧挥手判据对比：解释"为什么挥不动"，验证新版能挥且不会更容易误触发。"""
import math
import random
from collections import deque

from swipe import SwipeTracker


class LegacySwipe:
    """软件组原件的行为（参数写死），用来做对照。"""

    def __init__(self):
        self.travel, self.cooldown, self.max_seconds = 0.20, 0.8, 1.1
        self.settle, self.max_gap = 0.16, 0.65
        self.event_id = 0
        self.blocked_until = -math.inf
        self.points = deque()
        self.anchor = None
        self.last_time = self.last_area = None

    def reset(self):
        self.points = deque()
        self.anchor = None
        self.last_time = self.last_area = None

    def update(self, position, now, valid=True, area=1.0):
        ev = {"swipe": False, "swipe_dir": None}
        if not valid or position is None:
            self.reset()
            return ev
        x, y = position
        if self.last_time is not None and (now <= self.last_time or now - self.last_time > self.max_gap):
            self.reset()
        if self.last_area and not 0.45 <= area / self.last_area <= 2.2:
            self.reset()
        self.last_time, self.last_area = now, max(float(area), 1e-9)
        if now < self.blocked_until:
            self.points.clear()
            self.anchor = None
            return ev
        self.points.append((now, x, y))
        while self.points and now - self.points[0][0] > self.max_seconds:
            self.points.popleft()
        if self.anchor is None:
            recent = [p for p in self.points if now - p[0] <= 0.45]
            if (len(recent) >= 3 and now - recent[0][0] >= self.settle
                    and max(p[1] for p in recent) - min(p[1] for p in recent) <= 0.04
                    and max(p[2] for p in recent) - min(p[2] for p in recent) <= 0.05):
                self.anchor = (now, x, y)
                self.points = deque([self.anchor])
            return ev
        t0, ax, ay = self.anchor
        if now - t0 > self.max_seconds or abs(y - ay) > 0.15:
            self.reset()
            return ev
        dx = x - ax
        if abs(dx) >= self.travel and len(self.points) >= 3 and now - t0 >= 0.12:
            total = sum(abs(b[1] - a[1]) for a, b in zip(self.points, list(self.points)[1:]))
            if total and abs(dx) / total >= 0.8:
                self.event_id += 1
                ev.update(swipe=True, swipe_dir="right" if dx > 0 else "left")
                self.blocked_until = now + self.cooldown
                self.reset()
        return ev


def run(tracker, xs, dt=0.10, area0=4000, area_jitter=0.15, seed=1):
    now, rnd = 0.0, random.Random(seed)
    for x in xs:
        area = area0 * (1 + rnd.uniform(-area_jitter, area_jitter))
        ev = tracker.update((x, 0.50), now, valid=True, area=area)
        if ev["swipe"]:
            return True, ev["swipe_dir"]
        now += dt
    return False, None


def steady(x, n=4, jitter=0.0, seed=0):
    r = random.Random(seed)
    return [x + r.uniform(-jitter, jitter) for _ in range(n)]


CASES = [
    ("A 手抖 ±0.06，停 0.4s 后挥 0.18", steady(0.45, 4, 0.06, 3) + [0.45 + 0.045 * i for i in range(1, 5)], True, 0.10),
    ("B 只抖不走 ±0.08，12 帧", [0.5 + random.Random(i).uniform(-0.08, 0.08) for i in range(12)], False, 0.10),
    ("C 慢漂移 0.18 / 2.0 s", [0.40 + 0.015 * i for i in range(14)], False, 0.10),
    ("D 干净挥手 0.20 / 0.4 s", steady(0.40, 4, 0.01, 7) + [0.40 + 0.05 * i for i in range(1, 5)], True, 0.10),
    ("E 左挥 0.20", steady(0.60, 4, 0.01, 9) + [0.60 - 0.05 * i for i in range(1, 5)], True, 0.10),
    ("F 从画面左边缘扫进来 0.40", [0.02, 0.10, 0.18, 0.26, 0.34, 0.42], False, 0.10),
    ("G 小幅快挥 0.14（略低于阈值）", steady(0.40, 4, 0.01, 5) + [0.40 + 0.035 * i for i in range(1, 5)], False, 0.10),
    ("H 慢挥 0.20 / 1.0 s（0.20/s，偏慢）", steady(0.40, 4, 0.01, 5) + [0.40 + 0.02 * i for i in range(1, 11)], False, 0.10),
    ("I 偏慢但正常 0.17 / 0.7 s", steady(0.40, 4, 0.01, 5) + [0.40 + 0.0425 * i for i in range(1, 5)], True, 0.10),
    # 板端真实采样率：手势线程 3.2 Hz（每帧 prepare+detect 约 300 ms）
    ("J【板端 3.2Hz】停两拍后横移 0.20", [0.42, 0.44, 0.56, 0.64], True, 0.31),
    ("K【板端 3.2Hz】只在原地抖 4 帧", [0.50, 0.46, 0.54, 0.47], False, 0.31),
    ("L【板端 3.2Hz】缓慢漂移 4 帧共 0.12", [0.40, 0.44, 0.48, 0.52], False, 0.31),
]

print("%-34s %-13s %-13s %s" % ("场景", "旧(原件)", "新(本次)", "期望"))
print("-" * 80)
bad = 0
for name, xs, expect, dt in CASES:
    o = run(LegacySwipe(), xs, dt=dt)
    n = run(SwipeTracker(), xs, dt=dt)
    hit = "触发 " + n[1] if n[0] else "未触发"
    mark = "OK" if n[0] == expect else "!! 不符"
    if n[0] != expect:
        bad += 1
    print("%-34s %-13s %-13s %s %s" % (name, "触发 " + o[1] if o[0] else "未触发", hit, "应触发" if expect else "不应触发", mark))
print()
print("新判据与期望不符：%d 项" % bad)
raise SystemExit(1 if bad else 0)
