"""Mode switching and trial accounting, independent of camera/FPGA libraries.

v5 改动（2026-10-19，硬件侧）
----------------------------
`ModeLatch.select()` 原来硬编码只接受 mode 0/1/2。v5 把 PL 算子扩到 8 个
（mode 0..7），网页的模式按钮会传 3..7 进来，原写法会抛
`ValueError: invalid mode` —— 而且是在 process_loop 线程里抛的，
表现为"点了按钮画面没反应"，控制台还不一定看得到。

改为 `n_modes` 参数化：默认 3 保持向后兼容（test_accuracy.py 等老脚本不受影响），
调用方（live_view_raw.py）传 `len(MODE_NAMES)` 即可。

注意：`update()` 仍然只把 1/2/3 根手指映射到 mode 0/1/2 —— 这是刻意的。
手势只能覆盖前三个算子，要切到 mode 3..7 靠网页按钮或挥手循环，
不能靠"伸出四根以上手指"（五指张开本来就不稳定，容易误触）。
"""
from collections import Counter


# 2026-11-01 改动（软件侧）
# ------------------------
# 原来 `update()` 硬写 `self.mode = fingers - 1`，手指最多只够到 mode 2，
# 而 v8.2b 的三个融合抽头 CANNY(10) / NMS(11) / GAUSS5(9) 用手指选不到。
#
# 改成查表：`FINGER_MODE_MAP` 决定"几根手指 -> 哪个模式"。
# 分配依据（不是随手排的）：3 指识别最稳（实测 98.3%），
# 所以把最主打的 CANNY 放在 3 指；5 指就算修完 bug 也是最不稳的一类，
# 放 GAUSS5 这种锦上添花的算子，误触了也不难看。
#
# 注意：`update()` 里那句 `m >= self.n_modes` 不能删：
#    __init__ 默认 n_modes=3（为兼容 test_accuracy.py 等老脚本），
#    老代码里那句 `or self.n_modes < 3` 原本就是干这个的。
#    直接查表写出 mode=10 而 n_modes 还是 3，就会把越界模式号写进去。
FINGER_MODE_MAP = {1: 0, 2: 1, 3: 10, 4: 11}
#                 GRAY  SOBEL  CANNY  NMS
#
# 2026-11-01 演示版：去掉了 5 -> GAUSS5 这一档。
#   原因：手掌完全张开时，320x240 的肤色掩膜里手指会连成一片，
#   密实度冲过 MAX_SOLIDITY=0.88 被判 "too_solid"，5 指实测最不稳。
#   去掉它的收益不只是"不演示 5 指"——比 3 指万一被误判成 5，
#   查表返回 None，不会把画面切到别的模式，演示反而更稳。
#   恢复方法：把 `5: 9` 加回来即可（GAUSS5 = op 9）。


class ModeLatch:
    def __init__(self, mode=0, stable_count=3, stable_seconds=0.18, n_modes=3):
        self.mode = mode
        self.stable_count = stable_count
        self.stable_seconds = stable_seconds
        self.n_modes = int(n_modes)
        self.reset_candidate()

    def reset_candidate(self):
        self.candidate = None
        self.count = 0
        self.since = None

    def select(self, mode):
        if not isinstance(mode, int) or not (0 <= mode < self.n_modes):
            raise ValueError("invalid mode: %r (valid 0..%d)"
                             % (mode, self.n_modes - 1))
        self.mode = mode
        self.reset_candidate()

    def update(self, fingers, now):
        # 查表得到目标模式；挡两件事：
        #   ① 0 根或 6 根以上手指（表里没有 -> None）
        #   ② n_modes 不够大时别写出越界的模式号
        m = FINGER_MODE_MAP.get(fingers)
        if m is None or m >= self.n_modes:
            self.reset_candidate()
            return self.mode
        if fingers != self.candidate:
            self.candidate, self.count, self.since = fingers, 1, now
        else:
            self.count += 1
        if self.count >= self.stable_count and now - self.since >= self.stable_seconds:
            self.mode = m
        return self.mode


class TrialVotes:
    """One vote per observed sample; all settling-period samples are excluded."""
    def __init__(self, settle_seconds=1.0):
        self.settle_seconds = settle_seconds
        self.samples = []

    def add(self, elapsed, prediction):
        if elapsed >= self.settle_seconds:
            self.samples.append(int(prediction))

    def winner(self):
        ranked = Counter(self.samples).most_common()
        if not ranked or (len(ranked) > 1 and ranked[0][1] == ranked[1][1]):
            return None
        return ranked[0][0]


def summarize_trials(records, trials_per):
    by_class = {}
    for target in (1, 2, 3):
        rows = [r for r in records if r["target"] == target]
        correct = sum(r["prediction"] == target for r in rows)
        by_class[str(target)] = {
            "correct": correct, "total": len(rows),
            "accuracy": correct / len(rows) if rows else None,
        }
    complete = all(x["total"] == trials_per for x in by_class.values())
    passed = complete and all(x["accuracy"] >= 0.9 for x in by_class.values())
    return {"complete": complete, "passed": passed, "by_class": by_class,
            "protocol": "per-gesture majority vote; not per-frame accuracy"}
