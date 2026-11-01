#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
_pycheck_filters.py —— filters.py 的 5x5 参考实现 vs HLS DUT 逻辑 对拍

本机没有 C++ 编译器，所以拿 _pycheck_v81.py 里那份"照抄 HLS path5x5()"
的 Python 实现当 DUT，跟板端 filters.py 的 numpy 参考实现对拍。
两条路径（HLS 的常量下标宏 + 循环缓冲 / numpy 的整幅切片 + 向量化）独立，
对上了就说明参考实现没写错，validate_v8.py 的期望值可信。

用法：
  python _pycheck_filters.py
"""
import os
import sys

import numpy as np

BOARD = r"C:/Users/还原剂/Desktop/2026大二上FPGA/上板上传_1020/上传_v8_1029"
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, BOARD)
sys.path.insert(0, HERE)

import filters                                      # noqa: E402
from _pycheck_v81 import (dut_path5x5, MODE_GAUSS5, MODE_CANNY, MODE_NMS,  # noqa: E402
                          GAIN_UNITY, CANNY_THRESH_DEFAULT)


def main():
    rng = np.random.RandomState(20261029)
    sizes = [(32, 24), (33, 17), (64, 48), (5, 5), (6, 5), (3, 3)]
    gains = [GAIN_UNITY, 512, 1000]
    threshes = [CANNY_THRESH_DEFAULT, (100 << 8) | 50, (20 << 8) | 10]

    ncase = 0
    nfail = 0
    for (w, h) in sizes:
        gray = rng.randint(0, 256, size=(h, w)).astype(np.uint8)
        for gain in gains:
            g_agc = np.clip((gray.astype(np.int32) * gain) >> 8, 0, 255)
            for th in threshes:
                for op, fn in ((MODE_GAUSS5, filters.gauss5_gray),
                               (MODE_NMS, None),
                               (MODE_CANNY, filters.canny5_gray)):
                    ncase += 1
                    d = np.array(dut_path5x5(gray.ravel().tolist(), w, h,
                                             op, gain, th),
                                 dtype=np.uint8).reshape(h, w)
                    if op == MODE_NMS:
                        r = np.clip(filters.nms5_gray(g_agc), 0, 255).astype(np.uint8)
                    elif op == MODE_GAUSS5:
                        r = fn(g_agc)
                    else:
                        r = fn(g_agc, th)
                    bad = int((d != r).sum())
                    if bad:
                        nfail += 1
                        ys, xs = np.nonzero(d != r)
                        print("  [FAIL] %dx%d op=%2d gain=%4d th=%04x  %d 处不同，"
                              "首差 (%d,%d) dut=%d ref=%d"
                              % (w, h, op, gain, th, bad, ys[0], xs[0],
                                 d[ys[0], xs[0]], r[ys[0], xs[0]]))

    # 附加：process_bgr 在 5x5 mode 下也不崩、且外两圈恒 0
    extra = 0
    frame = rng.randint(0, 256, size=(32, 24, 3)).astype(np.uint8)
    for mode in (9, 10, 11):
        o = filters.process_bgr(frame, mode)
        if not ((o[0:2, :] == 0).all() and (o[-2:, :] == 0).all()
                and (o[:, 0:2] == 0).all() and (o[:, -2:] == 0).all()):
            extra += 1
            print("  [FAIL] process_bgr mode=%d 外两圈非 0" % mode)

    print("\n======================================================================")
    print("filters.py 5x5 对拍：%d 例，失败 %d 例；附加 %d 项失败 -> %s"
          % (ncase, nfail, extra, "全部 PASS" if (nfail == 0 and extra == 0) else "有问题"))
    print("======================================================================")
    return 1 if (nfail or extra) else 0


if __name__ == "__main__":
    sys.exit(main())
