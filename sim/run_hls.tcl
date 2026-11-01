# Vitis HLS 批处理脚本（GUI 不好找入口时用这个兜底）
#
# 用法：在 Vitis 命令行环境里执行
#   cd D:\vivado\hls\rgb2gray
#   vitis_hls -f run_hls.tcl
#
# 跑完后看 hls_rgb2gray/solution1/syn/report/ 下的报告

open_project hls_rgb2gray
set_top rgb2gray

add_files rgb2gray_hls_v5.cpp
add_files -tb tb_rgb2gray_hls_v5.cpp

open_solution "solution1"
set_part {xc7z020clg400-1}
# 10.0 ns = 100 MHz，与板子实际 FCLK 一致（原为 8.0，导致报告对不上真板）
create_clock -period 10.0 -name default

# 1) C 仿真：在 PC 上验证算法（返回 0 即通过）
csim_design

# 2) C 综合：把 C 变成 RTL
csynth_design

# 3) 导出 IP，供 Vivado Block Design 使用
export_design -format ip_catalog -output ./ip_repo

exit
