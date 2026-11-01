# 只跑 csynth，看 II / latency / 资源 / 时序估计
# 用途：2026-10-30 上板实测 op 1~11 全是 20.57 ms（II=1 地板 3.072 ms 的 6.7 倍），
#       GRAY 却是 3.222 ms。需要确认主循环的真实 II 到底是多少。
#
# 用法：vitis_hs -f csynth_only_v82.tcl

set PROJECT  RGB2gray_v8
set SRC_DIR  D:/vivado/hls/rgb2gray
set PART     xc7z020clg400-1

set SRC $SRC_DIR/rgb2gray_hls_v8.cpp
set HDR $SRC_DIR/rgb2gray_hls_v8.h

foreach f [list $SRC $HDR] {
    if {![file exists $f]} {
        puts "FAIL: 找不到 $f"
        exit 1
    }
}

open_project -reset $PROJECT
set_top rgb2gray
add_files $SRC
add_files -tb $HDR

open_solution -reset sol_csynth
set_part $PART
create_clock -period 10ns -name default

csynth_design

puts "============================================================"
puts "csynth done. 看上面的 Timing Estimate / Latency / II 与 Utilization。"
puts "============================================================"
exit 0
