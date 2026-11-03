# v8 HLS 一键构建 + 自检（Vitis HLS 命令行）
#
# 复现包要求"赛事方以队伍提交的容器与脚本抽查复现"。
# 只在 IDE 里点鼠标的流程没法复现，必须有一条命令行路径。
#
# 用法（Vitis 2026.1 自带的 shell，或把 vitis_hs 加进 PATH）：
#     vitis_hs -f run_hls_v8.tcl
#
# 依次做四件事，任何一步失败都会打印 FAIL 并不再往下走：
#     1. csim    C 仿真（跑 tb_rgb2gray_hls_v8.cpp，回归 + MODE_DIR + sigma）
#     2. csynth  综合（看 II / LUT / DSP / 时序估计）
#     3. cosim    C/RTL 联合仿真（这一步会暴露"综合过了但行为变了"）
#     4. export  出 IP catalog，供 Vivado 导入
#
# cosim 不能跳：v7 时就出现过"csynth 全绿但寄存器地址和驱动想的不一致"，
#   最后是上板实测才确认 REG_GAIN=0x28。cosim 能提前抓住这类问题。

set PROJECT  RGB2gray_v8
set SOLUTION sol1
set SRC_DIR  ../../vivado/hls/rgb2gray
set PART     xc7z020clg400-1
set CLOCK   10ns

set SRC $SRC_DIR/rgb2gray_hls_v8.cpp
set HDR $SRC_DIR/rgb2gray_hls_v8.h
set TB  $SRC_DIR/tb_rgb2gray_hls_v8.cpp

foreach f [list $SRC $HDR $TB] {
    if {![file exists $f]} {
        puts "============================================================"
        puts "FAIL: 找不到 $f"
        puts "      先确认 D:/vivado/hls/rgb2gray/ 下四个 v8 文件都在。"
        puts "============================================================"
        exit 1
    }
}

# 1) 建工程 / 解决方案（已存在就打开，保证可重复跑）
if {![file exists $PROJECT]} {
    open_project -reset $PROJECT
} else {
    open_project $PROJECT
}
set_top rgb2gray
add_files $SRC $HDR
add_files -tb $TB
open_solution -reset $SOLUTION -flow_target vitis
set_part $PART
create_clock -period $CLOCK

# 2) C 仿真
puts "============================================================"
puts "[1/4] C 仿真（csim）"
puts "============================================================"
csim_design

# 3) 综合
puts "============================================================"
puts "[2/4] 综合（csynth）"
puts "============================================================"
csynth_design

# 4) C/RTL 联合仿真
puts "============================================================"
puts "[3/4] C/RTL 联合仿真（cosim）"
puts "============================================================"
cosim_design -rtl verilog -trace_level all

# 5) 出 IP
puts "============================================================"
puts "[4/4] 导出 IP catalog"
puts "============================================================"
export_design -format ip_catalog

puts "============================================================"
puts "v8 HLS 全流程完成。"
puts ""
puts "接下来（Vivado，出位流）："
puts "  1. 打开 D:/vivado/img_accel/img_accel.xpr"
puts "  2. IP Catalog 里刷新，找到更新后的 rgb2gray"
puts "  3. 重新 Generate Block Design -> Run Implementation -> Generate Bitstream"
puts ""
puts "位流大小恒为 4045771 B（v4/v5/v6/v7/v8 全都一样），只能靠 MD5 区分："
puts "    加载前必须核 MD5，别凭大小判断版本。"
puts "============================================================"

exit 0
