"""五指现场诊断（2026-11-01）

比出 5 根手指，看这里打印的数值，就能定位到底卡在哪一关。

用法（板端，摄像头前）：
    source /usr/local/share/pynq-venv/bin/activate
    cd /home/xilinx/jupyter_notebooks/rgb2gray_v82
    python3 -u _diag_fingers.py            # 默认 40 秒
    python3 -u _diag_fingers.py 20         # 只跑 20 秒

它每 0.4 秒把**面积最大的三块皮肤色区域**逐块列出：
    area%   solid   ROUGH   asp   判定/拒因
  其中 solid=密实度  ROUGH=周长比  asp=长短边比

两块要核对：
  * 最大的那块如果是你的手，但判定 REJECT —— 看拒因是哪一关
  * 选中手之后 fingers 是不是 5，不是就看 tips / gaps 两个数
"""
import sys
import time

import cv2
import numpy as np

import gesture as g
import session

g.apply_strict(1.0)

SECONDS = float(sys.argv[1]) if len(sys.argv) > 1 else 40.0
IDX = int(sys.argv[2]) if len(sys.argv) > 2 else 0

print("=" * 78)
print("当前阈值：MIN_ROUGH=%.2f  MIN_SOLIDITY=%.2f  MAX_SOLIDITY=%.2f  "
      "MIN_ASPECT=%.2f  MAX_AREA_FRAC=%.2f"
      % (g.MIN_ROUGH, g.MIN_SOLIDITY, g.MAX_SOLIDITY, g.MIN_ASPECT, g.MAX_AREA_FRAC))
print("可以先用环境变量试探，例如：MAX_SOLIDITY=0.95 python3 -u _diag_fingers.py")
print("=" * 78)
print("%-5s %-6s %-7s %-6s %-6s %-5s  %s"
      % ("块", "area%", "solid", "ROUGH", "asp", "选?", "判定 / 数指"))
print("-" * 78)

cap = session.open_camera(IDX)
if not cap or not cap.isOpened():
    print("摄像头打不开（index=%d）。看 --camera 参数用的是几。" % IDX)
    sys.exit(1)

t0 = time.time()
n5 = n_total = 0
hist = {}

try:
    while time.time() - t0 < SECONDS:
        frame = session.read_frame(cap)
        if frame is None:
            continue
        mask = g.make_skin_mask(frame)
        cs = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = cs[0] if len(cs) == 2 else cs[1]
        cnts = sorted(cnts, key=cv2.contourArea, reverse=True)[:3]

        if not cnts:
            continue

        n_total += 1
        chosen = g.pick_hand_contour(cnts, (), mask.shape)
        rows = []
        for i, cnt in enumerate(cnts):
            area = cv2.contourArea(cnt)
            hull = cv2.convexHull(cnt)
            ha = cv2.contourArea(hull)
            hp = cv2.arcLength(hull, True)
            solid = area / ha if ha > 0 else 0.0
            rough = cv2.arcLength(cnt, True) / hp if hp > 0 else 0.0
            rect = cv2.minAreaRect(cnt)
            rw, rh = rect[1]
            asp = min(rw, rh) / max(rw, rh) if max(rw, rh) > 0 else 0.0
            ok, why = g.hand_looks_real(cnt, mask.shape)
            mark = "" if (chosen is not None and cnt is chosen) else " "
            verdict = "ok" if ok else "REJECT(%s)" % (why or "?")
            rows.append("%-5s %-6.3f %-7.3f %-6.3f %-6.3f %-5s  %s"
                        % ("#%d%s" % (i + 1, mark), area / float(mask.shape[0] * mask.shape[1]),
                           solid, rough, asp, "选中" if mark == "" else "", verdict))

        extra = ""
        if chosen is not None:
            out = g.count_fingers(mask)
            fingers = out[0]
            d = out[-1] if isinstance(out[-1], dict) else {}
            extra = "  ==> fingers=%d tips=%s gaps=%s %s" % (
                fingers, d.get("tips"), d.get("gaps"), d.get("reason", ""))
            hist[fingers] = hist.get(fingers, 0) + 1
            if fingers == 5:
                n5 += 1
        else:
            extra = "  ==> 没选中手（拒因: %s）" % getattr(g, "LAST_REJECT_REASON", "?")

        for r in rows:
            print(r)
        print(extra)
        print("-" * 78)
        time.sleep(0.4)
except KeyboardInterrupt:
    pass
finally:
    try:
        cap.release()
    except Exception:
        pass

print()
print("共采样 %d 帧；fingers 分布：%s" % (n_total, sorted(hist.items())))
if n_total:
    print("5 指命中率：%.1f%%（%d/%d）" % (100.0 * n5 / n_total, n5, n_total))
print()
print("判读：")
print("  · 若你的手被 REJECT(too_solid)  -> 张开手掌密实度超过 MAX_SOLIDITY，")
print("    用 MAX_SOLIDITY=0.95 再跑一次本脚本确认（只是验证，别当最终方案）")
print("  · 若被 REJECT(too_smooth)       -> 周长比不够，降 MIN_ROUGH")
print("  · 若选中了但 fingers 不是 5     -> 问题在数指尖，看 tips/gaps 与真值差多少")
