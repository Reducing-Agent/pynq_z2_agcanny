"""校准防呆验证：白墙（采到背景）必须被拒，真手必须通过。"""
import numpy as np
import cv2
import gesture as g


def frame_with(border_hsv, box_hsv, size=(480, 640)):
    h, w = size
    hsv = np.zeros((h, w, 3), np.uint8)
    hsv[:, :] = border_hsv
    x1, y1, x2, y2 = g.cal_frame_box(np.zeros((h, w, 3), np.uint8))
    hsv[y1:y2, x1:x2] = box_hsv
    return cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)


cases = [
    ("白墙：框内=框外 S=18 V=147", (24, 18, 147), (24, 18, 147), False),
    ("真手：框内 S=110 H=12，框外白墙 S=18", (24, 18, 147), (12, 110, 150), True),
    ("手偏黄但背景冷白：S=55", (24, 18, 147), (20, 55, 160), True),
    ("全黑", (0, 0, 5), (0, 0, 5), False),
]

print("%-40s %-8s %-26s %s" % ("场景", "期望", "结果", "reason"))
print("-" * 92)
bad = 0
for name, border, box, expect in cases:
    frame = frame_with(border, box)
    ok = g.calibrate_skin(frame)
    reason = g.LAST_CALIBRATION.get("reason")
    mark = "OK" if ok == expect else "!! 不符"
    if ok != expect:
        bad += 1
    print("%-40s %-8s %-26s %s %s" % (name, "通过" if expect else "拒绝",
                                      "通过" if ok else "拒绝", reason, mark))
print()
print("不符：%d 项" % bad)
raise SystemExit(1 if bad else 0)
