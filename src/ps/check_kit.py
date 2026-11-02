"""换文件后先跑这个：确认目录里是**我这套 v8.2**，不是软件组的 sw_v5 那套，也不是旧版。

    source /usr/local/share/pynq-venv/bin/activate
    python3 check_kit.py

两套文件同名但互不兼容 —— 混在一起最典型的现象就是
    ModuleNotFoundError: No module named 'application'
（那是软件组的 live_view_raw.py，它 import application.py；
 我这套的 live_view_raw.py 第一行是 import argparse，不 import application）

2026-10-30：期望值改为**优先从 FILES.txt 读**，不再双重维护
---------------------------------------------------------------
以前这份表是手抄的，结果 10-30 那天改了 validate_v8.py / hw_driver.py /
live_view_raw.py / sigma.py 四个文件，却漏了更新这里 —— 上板一跑，
六个文件全报 WRONG，而**真正错的只有 check_kit.py 自己**。
更危险的是它当时期望的位流是 v8.1（4f5ee693），谁要是照着"修正"
就会把 v8.2 位流换成 v8.1。

现在以 FILES.txt 为唯一依据（由 `_gen_files_txt.py` 生成），
本脚本直接读它。以后改任何文件 → 重跑一次生成脚本 → 这里自动跟上。
FILES.txt 缺失时才回退到下面的内置表（兜底用，可能过期）。
"""
import hashlib
import os
import re
import sys

# ---- 兜底表：只在本目录没有 FILES.txt 时用 ----
# 这是 v8.2（2026-10-30）的值。改文件后请重跑 _gen_files_txt.py，
#   让 FILES.txt 生效，别在这里手改。
FALLBACK = {
    "rgb2gray.bit":    (4045771, "6b5fd2f9"),
    "rgb2gray.hwh":    ( 266281, "9b7407a3"),
    "live_view_raw.py": (  67635, "6148c0a9"),
    "validate_v8.py":  (  41123, "2ea95ad2"),
    "hw_driver.py":    (  39130, "b7bbc7f0"),
    "gesture.py":      (  36833, "dd9af653"),
    "filters.py":      (  16285, "9ec44e39"),
    "sigma.py":        (  11223, "d1332a91"),
    "session.py":      (  10252, "38ed9b24"),
    "swipe.py":        (   5916, "2ee08c60"),
    "control.py":      (   3391, "845b0885"),
}

# 最关键的两条：位流版本错了后面全是白跑
BIT_EXPECT = {"rgb2gray.bit": "6b5fd2f9", "rgb2gray.hwh": "9b7407a3"}

# 软件组那套的文件：出现在同一目录就会被 import 抢走，且接口不兼容
FOREIGN = {
    "application.py": "软件组的应用层（我这套不需要，留着也不影响，但说明你混装了）",
    "hardware_driver.py": "软件组的驱动实现（我这套用单文件 hw_driver.py，两者不能配）",
    "display.py": "软件组的 1280x720 合成画布（我这套不用）",
    "web_server.py": "软件组的 HTTP 层（我这套的 HTTP 在 live_view_raw.py 里）",
    "image_contract.py": "软件组的数据契约（只被 software_driver 用）",
    "performance.py": "软件组的 PERF 表（我这套写死在 live_view_raw.py）",
}

HERE = os.path.dirname(os.path.abspath(__file__))
ok = True


def digest(path):
    with open(path, "rb") as f:
        return hashlib.md5(f.read()).hexdigest()


_NAME_RE = re.compile(r"^[\w.-]+\.[A-Za-z0-9]+$")


def load_expect_from_manifest():
    """从 FILES.txt 读期望值（唯一依据）。

    行格式：`文件名  字节  MD5(32位)  档  说明`
    三重过滤，避免把非文件行当成条目：
      · 跳过 # 注释和 --- 分隔线
      · 第二列必须是数字（挡掉表头「文件 字节 MD5...」）
      · 第一列必须长得像文件名（挡掉结尾那行「共 26 个文件：...」）
      · FILES.txt 自身不校验（它的 MD5 是自指的，永远对不上）
    """
    path = os.path.join(HERE, "FILES.txt")
    if not os.path.isfile(path):
        return None
    d = {}
    try:
        with open(path, encoding="utf-8") as fh:
            for line in fh:
                line = line.strip()
                if (not line or line.startswith("#")
                        or line.startswith("-")):
                    continue
                parts = line.split()
                if len(parts) < 3 or not parts[1].isdigit():
                    continue
                name = parts[0]
                if name == "FILES.txt" or not _NAME_RE.match(name):
                    continue
                d[name] = (int(parts[1]), parts[2])
    except Exception:
        return None
    return d or None


manifest = load_expect_from_manifest()
EXPECT = manifest if manifest else FALLBACK
src = "FILES.txt（唯一依据）" if manifest else "内置兜底表（FILES.txt 缺失，可能过期）"

print("检查目录:", HERE)
print("期望值来源:", src)
print()
print("%-20s %8s %10s  %s" % ("文件", "字节", "MD5前8", "结果"))
print("-" * 58)
for name, (size, prefix) in sorted(EXPECT.items()):
    path = os.path.join(HERE, name)
    if not os.path.isfile(path):
        # FILES.txt 里登记的文档缺了不算致命（比如没传 README）
        if name.endswith((".md", ".txt")):
            print("%-20s %8s %10s  (缺，不影响运行)" % (name, "-", "-"))
            continue
        print("%-20s %8s %10s  MISSING" % (name, "-", "-"))
        ok = False
        continue
    real_size = os.path.getsize(path)
    real_md5 = digest(path)
    flag = "OK" if (real_size == size and real_md5.startswith(prefix)) else \
           "WRONG (应为 %d B / %s)" % (size, prefix)
    if flag != "OK":
        ok = False
    print("%-20s %8d %10s  %s" % (name, real_size, real_md5[:8], flag))

# ---- 位流单独再判一次：这是唯一能一票否决的项 ----
print()
bit_bad = []
for name, prefix in BIT_EXPECT.items():
    p = os.path.join(HERE, name)
    if os.path.isfile(p) and not digest(p).startswith(prefix):
        bit_bad.append("%s 实际 %s…（应为 %s…）" % (name, digest(p)[:8], prefix))
if bit_bad:
    print("!! 位流版本不对：%s" % "；".join(bit_bad))
    print("   .bit 大小恒为 4045771 B，v4~v8.2 全都一样，**只能靠 MD5 区分**。")
    print("   载错版本的典型现象：mode>=3 全输出 EMBOSS（画面像「滤镜都变浮雕」）。")
    ok = False
else:
    # 2026-10-31：原来这里把版本号和 MD5 写死成 "v8.2，e9645754…"，
    #   换了位流后判定仍然是 OK（判定走 BIT_EXPECT，是对的），
    #   但这行**打印出来的版本是错的**，看的人会以为载的还是旧位流。
    #   改成从 BIT_EXPECT 动态取，避免这类"判定对、文案错"的情况。
    print("位流版本：OK（.bit %s… / .hwh %s…）"
          % (BIT_EXPECT["rgb2gray.bit"], BIT_EXPECT["rgb2gray.hwh"]))

print()
stray = [n for n in FOREIGN if os.path.isfile(os.path.join(HERE, n))]
if stray:
    print("检测到软件组的文件（同一目录，容易混装）：")
    for n in stray:
        print("   %-22s %s" % (n, FOREIGN[n]))
    print("   注意：只要 live_view_raw.py / hw_driver.py 是「我的」版本，它们不参与 import，")
    print("   可以先留着；但下次别再整包覆盖这个目录。")
    print()

# 自愈提示：区分「文件传错了」和「check_kit.py 自己旧了」
if not ok:
    mine = [n for n, (s, p) in EXPECT.items()
            if os.path.isfile(os.path.join(HERE, n))
            and (os.path.getsize(os.path.join(HERE, n)) != s
                 or not digest(os.path.join(HERE, n)).startswith(p))]
    if mine:
        print("提示：上面标 WRONG 的是 %s" % ", ".join(mine))
        if manifest is None:
            print("      期望值来自内置兜底表（FILES.txt 缺失），**先重传 FILES.txt**")
            print("      再跑一次，如果还 WRONG，那就是文件真的传错了。")
        else:
            print("      期望值来自 FILES.txt，所以这些文件**确实不对版** ——")
            print("      重传它们。注意别反过来把 FILES.txt 改成错的（那等于把判据改了）。")

# 最直接的判据：这个文件到底 import 了什么
_main = os.path.join(HERE, "live_view_raw.py")
head = open(_main, encoding="utf-8", errors="replace").read(400) if os.path.isfile(_main) else ""
if "from application import" in head:
    print("!! live_view_raw.py 是软件组的版本 —— 这就是报 No module named 'application' 的原因")
    ok = False

if ok:
    try:
        sys.path.insert(0, HERE)
        import live_view_raw  # noqa: F401
        print("import 自检：OK（各文件互相配套）")
        print("下一步：python3 -u validate_v8.py")
    except Exception as exc:
        print("import 自检失败：%s: %s" % (type(exc).__name__, exc))
        ok = False
else:
    print("先把标 WRONG/MISSING 的文件换成我给的版本，再跑一次。")

sys.exit(0 if ok else 1)
