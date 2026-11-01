"""复现硬件组报的 bug：伸 5 根手指、手腕正常入画时只数出 1 根。"""
import math
import numpy as np
import cv2
import gesture as g

g.apply_strict(1.0)
W, H = 320, 240


def skin_mask(binary):
    """把二值图变成"皮肤色"图，再走真实路径 make_skin_mask。"""
    hsv = np.zeros((H, W, 3), np.uint8)
    hsv[binary != 0] = (12, 90, 200)
    return g.make_skin_mask(cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR))


def hand(n_fingers, wrist=True, center=(160, 140), spread=75.0, flen=95, fr=10):
    """合成一只张开的手：椭圆手掌 + n 根放射状手指 + 可选手腕。

    参数经过挑选，使形状指标（密实度 0.77~0.85、周长比 1.15~1.54）
    落在真手实测区间内，确保不会先在形状关就被拦下——
    我们要测的是"数 parish 数"这一步，不是形状判据。
    """
    img = np.zeros((H, W), np.uint8)
    cx, cy = center
    cv2.ellipse(img, (cx, cy), (58, 50), 0, 0, 360, 255, -1)
    if wrist:
        cv2.rectangle(img, (cx - 20, cy + 38), (cx + 20, H - 1), 255, -1)
    if n_fingers > 0:
        n = n_fingers
        for i in range(n):
            frac = (i - (n - 1) / 2.0) / max(1, n - 1) if n > 1 else 0.0
            ang = math.radians(-90 + frac * spread)
            tx = cx + math.cos(ang) * flen
            ty = cy + math.sin(ang) * flen
            for t in np.linspace(0, 1, 300):
                px = int(round(cx + (tx - cx) * t))
                py = int(round(cy + (ty - cy) * t))
                cv2.circle(img, (px, py), fr, 255, -1)
    return img


def count(binary):
    n, _, d = g.count_fingers(skin_mask(binary))
    return n, d


def main():
    print("合成手（手掌朝下、手腕正常入画）= 演示时最自然的姿势")
    print("%-28s %8s %8s  %s" % ("场景", "期望", "实测", "明细"))
    print("-" * 72)
    bad = 0
    for n in (1, 2, 3, 4, 5):
        exp = n
        got, d = count(hand(n, wrist=True))
        flag = "OK" if got == exp else "!!! BUG"
        if got != exp:
            bad += 1
        print("%-28s %8d %8d  tips=%d gaps=%d %s  [%s]"
              % ("%d 指 + 手腕" % n, exp, got, d.get("tips", 0),
                 d.get("gaps", 0), d.get("reason", ""), flag))
    print("-" * 72)
    # 无误检回归：大面积背景色块必须判 0
    wall = np.zeros((H, W), np.uint8)
    wall[20:200, 40:280] = 255
    got, d = count(wall)
    print("%-28s %8d %8d  %s  [%s]" % ("整片背景色块(防误检)", 0, got,
                                       d.get("reason", ""), "OK" if got == 0 else "!!! 误检"))
    if got != 0:
        bad += 1
    print("-" * 72)
    print("有问题的场景数: %d" % bad)
    return bad


if __name__ == "__main__":
    raise SystemExit(1 if main() else 0)
