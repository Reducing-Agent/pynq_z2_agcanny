#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""新位流 -> 板端交付包，一键同步 MD5（放在交付包外面，不进清单）

    python update_bitstream.py

=============================================================================
为什么必须有这个脚本
=============================================================================
XC7Z020 的 .bit 是整片配置帧，**同一器件无论设计怎么改都恒为 4045771 字节**，
看文件大小完全分不出版本（见 cases/03_bitstream_version.md）。
于是版本信息散落在好几处，手工同步必漏：

  - hw_driver.py 的 BIT_MD5_PREFIX     —— 加载前校验，写错会被拒
  - check_kit.py 的 BIT_EXPECT          —— 上板自检
  - FILES.txt 的表头                    —— 交付清单

漏改任何一处的后果（本项目真事）：
  - 漏改 hw_driver：加载直接被拒，或者开 SKIP_MD5 后**静默跑错版本**
  - 漏改 check_kit：6 个文件全报 WRONG，而错的只有它自己；
    更险的是它会"建议"你换回旧位流（见 cases/06_script_drift.md）

=============================================================================
做的事（顺序固定，别跳步）
=============================================================================
  1. 从 SRC 取新 .bit / .hwh，算 MD5 + 校验大小
  2. 拷进 PKG 覆盖，并**回读校验 MD5 一致**
  3. 改 hw_driver.py 的 BIT_MD5_PREFIX（+ 注释里的完整 MD5）
  4. 改 check_kit.py 的 BIT_EXPECT
  5. 改 _gen_files_txt.py 的 HEADER（位流 MD5 / 构建时间）
  6. 重新生成 FILES.txt

换项目时改顶部三个常量：HERE / PKG / SRC，以及各文件的正则。
"""
import hashlib
import os
import re
import shutil
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.join(HERE, "rgb2gray_v82")
SRC = r"D:\vivado\deliver\rgb2gray_overlay_v8"


def md5(p):
    with open(p, "rb") as f:
        return hashlib.md5(f.read()).hexdigest()


def sub(path, pat, rep, count=1, label=""):
    with open(path, encoding="utf-8") as f:
        s = f.read()
    s2, n = re.subn(pat, rep, s, count=count)
    if n != count:
        print("!! 替换失败（%s）：%s  命中 %d 次，期望 %d 次"
              % (label or path, pat[:60], n, count))
        return False
    with open(path, "w", encoding="utf-8") as f:
        f.write(s2)
    print("  OK  %s  <- %s" % (os.path.basename(path), label or pat[:40]))
    return True


def main():
    bit = os.path.join(SRC, "rgb2gray.bit")
    hwh = os.path.join(SRC, "rgb2gray.hwh")
    for p in (bit, hwh):
        if not os.path.isfile(p):
            print("找不到 %s —— 先跑完 Vivado 构建" % p)
            return 1

    b_md5 = md5(bit)
    h_md5 = md5(hwh)
    b_sz = os.path.getsize(bit)
    h_sz = os.path.getsize(hwh)
    print("新位流：")
    print("  .bit %d B  %s" % (b_sz, b_md5))
    print("  .hwh %d B  %s" % (h_sz, h_md5))
    if b_sz != 4045771:
        print("  !! .bit 大小不是 4045771 B，跟历史版本对不上，先确认再继续")
    print()

    # ---- 2. 拷进交付包 ----
    for src, name in ((bit, "rgb2gray.bit"), (hwh, "rgb2gray.hwh")):
        dst = os.path.join(PKG, name)
        shutil.copy2(src, dst)
        assert md5(dst) == md5(src), "拷贝后 MD5 不一致：%s" % name
    print("已拷进交付包（并校验 MD5 一致）\n")

    # ---- 3. hw_driver.py ----
    ok = True
    ok &= sub(os.path.join(PKG, "hw_driver.py"),
              r'BIT_MD5_PREFIX = \{[^}]*\}',
              'BIT_MD5_PREFIX = {".bit": "%s", ".hwh": "%s"}'
              % (b_md5[:8], h_md5[:8]),
              label="BIT_MD5_PREFIX")
    ok &= sub(os.path.join(PKG, "hw_driver.py"),
              r'#\s*v8\.2 完整 MD5：\.bit [0-9a-f]+ / \.hwh [0-9a-f]+',
              '#     v8.2 完整 MD5：.bit %s / .hwh %s' % (b_md5, h_md5),
              label="注释里的完整 MD5")

    # ---- 4. check_kit.py ----
    ok &= sub(os.path.join(PKG, "check_kit.py"),
              r'BIT_EXPECT = \{[^}]*\}',
              'BIT_EXPECT = {"rgb2gray.bit": "%s", "rgb2gray.hwh": "%s"}'
              % (b_md5[:8], h_md5[:8]),
              label="BIT_EXPECT")

    # ---- 5. _gen_files_txt.py 的 HEADER ----
    stamp = time.strftime("%Y-%m-%d %H:%M")
    # HEADER 里的写法是 '# 位流：...'（一个 # 一个空格），不是 '#   位流：'
    #   第一版这里按 4 空格写的正则，命中 0 次，FILESS.txt 就没重新生成 —— 已修正
    ok &= sub(os.path.join(HERE, "_gen_files_txt.py"),
              r"# 位流：\.bit [0-9a-f]+ / \.hwh [0-9a-f]+",
              '# 位流：.bit %s / .hwh %s' % (b_md5, h_md5),
              label="FILES 头-位流 MD5")
    # 正则必须用 [^'\n]* 而不是 [^\n]*：
    #   HEADER 每行是 Python 字符串字面量，行尾是 `',`，
    #   用 [^\n]* 会把结尾的引号一起吃掉 -> _gen_files_txt.py 语法错
    ok &= sub(os.path.join(HERE, "_gen_files_txt.py"),
              r"# 构建：[^'\n]*",
              '# 构建：%s vitis(v++ -c / --impl / --package) + vivado build_v8.tcl' % stamp,
              label="FILES 头-构建时间")

    if not ok:
        print("\n有替换失败，FILES.txt 未重新生成。先看清上面哪条失败。")
        return 1

    # ---- 6. 重新生成 FILES.txt ----
    print()
    r = subprocess.call([sys.executable, os.path.join(HERE, "_gen_files_txt.py")])
    if r != 0:
        print("FILES.txt 生成失败")
        return 1

    print()
    print("=" * 60)
    print("位流已更新。现在重跑一遍 check_kit.py 确认全绿：")
    print("  cd /home/xilinx/jupyter_notebooks/rgb2gray_v82 && python3 check_kit.py")
    print("=" * 60)
    return 0


if __name__ == "__main__":
    sys.exit(main())
