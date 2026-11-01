# -*- coding: utf-8 -*-
"""
_smoke_dir.py —— 验证挥手方向映射（2026-10-21 按用户手感定的约定）

约定：**手掌从右往左划 = 下一个模式（序号 +1）**，从左往右划 = 上一个（-1）。
     理由：和读列表/翻页一致，内容"往后推"，视线往左走。

这个脚本防止两件事：
  ① 以后有人（包括我自己）手滑把两个分支对调回去
  ② 网页 HUD 文案又改回 "right = next" 那种与实现相反的说法

跑法： python _smoke_dir.py
"""
import re
import sys

import live_view_raw as L

FAIL = []


def check(name, cond, extra=""):
    print("[%s] %-44s %s" % ("PASS" if cond else "FAIL", name, extra))
    if not cond:
        FAIL.append(name)


src = open("live_view_raw.py", encoding="utf-8").read()

# ---------------- A. 源码里的映射必须只有一处、且方向正确 ----------------
print("\n=== A. 源码映射 ===")
lines = [ln.strip() for ln in src.splitlines() if ln.strip().startswith("step =")]
check("A1 只有一处 step 赋值", len(lines) == 1, str(lines))
check("A2 左挥 +1 / 右挥 -1",
      bool(lines) and lines[0] == 'step = -1 if sdir == "right" else 1', lines[0] if lines else "n/a")

# 旧的错误写法不能再出现（防止被旧版覆盖回来）
check("A3 没有残留 step = 1 if sdir == \"right\"",
      'step = 1 if sdir == "right"' not in src)

# ---------------- B. 网页文案与实现一致 ----------------
print("\n=== B. 网页/日志文案 ===")
page = L.PAGE_TEMPLATE.replace("<!--MODES-->", L.build_mode_buttons())
check("B1 HUD 写明 左划 +1", "左划 +1" in page)
check("B2 HUD 左挥标注 模式+1", "'last 左挥 → 模式+1'" in page or "last 左挥 → 模式+1" in page)
check("B3 启动日志 LEFT = next mode", "LEFT = next mode" in src)
check("B4 不再出现 right = next", "right = next" not in src)
check("B5 按钮 tooltip 说明从右往左", "从右往左划 = 下一个模式" in page)

# ---------------- C. 行为模拟：8 个模式循环 ----------------
print("\n=== C. 行为模拟（%d 个模式）===" % len(L.MODE_NAMES))
N = len(L.MODE_NAMES)


def apply(mode, sdir):
    """复刻 live_view_raw.py 里的映射，用来算期望序列。"""
    step = -1 if sdir == "right" else 1
    return (mode + step) % N


check("C1 初始 0，左挥一次 -> 1", apply(0, "left") == 1,
      "-> %d %s" % (apply(0, "left"), L.MODE_NAMES[apply(0, "left")]))
check("C2 初始 0，右挥一次 -> 回绕到 %d" % (N - 1), apply(0, "right") == N - 1,
      "-> %d %s" % (apply(0, "right"), L.MODE_NAMES[apply(0, "right")]))

m = 0
seq = []
for _ in range(N):
    m = apply(m, "left")
    seq.append(m)
check("C3 连续左挥 %d 次回到起点" % N, m == 0, "路径 " + "→".join(str(x) for x in seq))
check("C4 左挥序列是 1,2,3,...（递增）",
      seq == [(i + 1) % N for i in range(N)], str(seq))

m = 0
for _ in range(N):
    m = apply(m, "right")
check("C5 连续右挥 %d 次回到起点" % N, m == 0)

# ---------------- 汇总 ----------------
print("\n" + "=" * 56)
if FAIL:
    print("FAILED %d 项: %s" % (len(FAIL), ", ".join(FAIL)))
    sys.exit(1)
print("ALL PASS")
