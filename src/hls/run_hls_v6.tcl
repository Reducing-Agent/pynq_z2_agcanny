# Vitis HLS 批处理脚本 —— v6 专用（与 v5 独立工程，保留 v5 报告做消融对比）
#
#   cd D:\vivado\hls\rgb2gray
#   vitis_hls -f run_hls_v6.tcl
#
# 跑完看 hls_rgb2gray_v6/solution1/syn/report/ 下的报告

open_project hls_rgb2gray_v6
set_top rgb2gray

add_files rgb2gray_hls_v6.cpp
add_files -tb tb_rgb2gray_hls_v6.cpp

open_solution "solution1"
set_part {xc7z020clg400-1}
# 10.0 ns = 100 MHz，与板子实际 FCLK 一致
create_clock -period 10.0 -name default

csim_design
csynth_design
export_design -format ip_catalog -output ./ip_repo_v6

exit
