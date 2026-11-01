#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""从交付包的 FILES.txt 生成标准格式的 MD5SUMS.txt（可用 `md5sum -c` 校验）

    python gen_md5sums.py <交付包目录>

为什么是从 FILES.txt 生成、而不是重新扫描目录：
  FILES.txt 是**唯一真相源**（由 _gen_files_txt.py 生成，随版本走）。
  重新扫描目录会把 __pycache__、临时文件一起扫进去，
  而且**永远校验不出"少了一个文件"**——少一个就少一行，照样全绿。
  从清单出发才能发现缺失。

输出格式（GNU coreutils md5sum 兼容）：
  <32位md5>  <文件名>
"""
import os
import sys

SRC_LINE = "-" * 118  # FILES.txt 里条目区的分隔线


def parse(files_txt):
    rows = []
    with open(files_txt, encoding="utf-8") as f:
        started = False
        for line in f:
            s = line.rstrip("\n")
            if s.startswith(SRC_LINE):
                if started:
                    break
                started = True
                continue
            if not started:
                continue
            parts = s.split()
            if len(parts) < 3:
                continue
            name, _size, md5 = parts[0], parts[1], parts[2]
            rows.append((name, md5))
    return rows


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 1
    pkg = sys.argv[1]
    ftxt = os.path.join(pkg, "FILES.txt")
    if not os.path.isfile(ftxt):
        print("找不到 %s" % ftxt)
        return 1

    rows = parse(ftxt)
    if not rows:
        print("FILES.txt 里没解析出条目 —— 检查分隔线格式")
        return 1

    out = []
    missing = []
    for name, md5 in rows:
        p = os.path.join(pkg, name)
        if not os.path.isfile(p):
            missing.append(name)
        out.append("%s  %s" % (md5, name))

    dst = os.path.join(pkg, "MD5SUMS.txt")
    with open(dst, "w", encoding="utf-8", newline="\n") as f:
        f.write("\n".join(out) + "\n")

    print("已生成 %s（%d 条）" % (dst, len(rows)))
    if missing:
        print("!! 清单里有但磁盘上不存在的文件：")
        for m in missing:
            print("   ", m)
        return 1
    print("磁盘上全部存在。校验：md5sum -c MD5SUMS.txt")
    return 0


if __name__ == "__main__":
    sys.exit(main())
