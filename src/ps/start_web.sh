#!/bin/sh
# 一键起网页演示（2026-11-01）
#
# 为什么必须有这个文件：
#   gesture.py 的 MIN_ROUGH 默认 1.14 是按"张开手掌"定的，
#   "比 1 根手指"必然是握拳形态，周长比只有 1.08~1.10，够不到 1.14，会被判 too_smooth。
#   实测 6 种握拳形态有 5 种挂；把门槛降到 1.07 后 6 种全过。
#   手工启动时每次都要记得敲 MIN_ROUGH=1.07，漏一次手势就全不通。
#
# 代价与验收：
#   新放行 rough in [1.07, 1.14) 这一小段，靠其余四关兜底，
#   所以**空手 60 秒误检压测必须重跑**。若误检涨了，把下面的 1.07 改回 1.14 即可。
#
# 用法（一律不加 sudo）：
#   cd /home/xilinx/jupyter_notebooks/rgb2gray_v82 && ./start_web.sh

cd /home/xilinx/jupyter_notebooks/rgb2gray_v82 || exit 1
. /usr/local/share/pynq-venv/bin/activate

# 旧字节码缓存会让 Python 跑旧代码
rm -rf __pycache__

# 2026-11-01 更正：新版 live_view_raw.py 已经没有 --display 参数了
#   （只有 --port，默认 8080，启动即走网页）。写 --display web 会报
#   "unrecognized arguments"，所以这里不要加。
exec env MIN_ROUGH=1.07 python3 -u live_view_raw.py --auto
