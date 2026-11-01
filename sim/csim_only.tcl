# 只跑 C 仿真：验证 σ 改成 16 bank 轮换后，**数值仍然精确不变**
#
# 2026-10-29 背景：单累加器 sigma_sum 是跨 307200 次迭代的循环进位链，
# 实测把主循环从 3.577 ms 拖到 20.57 ms。改成 16 bank 轮换后：
#   · 整数加法可交换 + 各 bank 都不溢出 -> 结果与原来**必须逐位相同**
#   · 所以 validate_v8.py 的期望值一个都不用改
# 这一步就是来证明这件事的。跑完看 135 例是不是全 PASS。

set PROJECT  RGB2gray_v8
set SOLUTION sol1
set SRC_DIR  D:/vivado/hls/rgb2gray
set PART     xc7z020clg400-1

foreach f [list $SRC_DIR/rgb2gray_hls_v8.cpp $SRC_DIR/rgb2gray_hls_v8.h $SRC_DIR/tb_rgb2gray_hls_v8.cpp] {
    if {![file exists $f]} {
        puts "FAIL: 找不到 $f"
        exit 1
    }
}

if {![file exists $PROJECT]} {
    open_project -reset $PROJECT
} else {
    open_project $PROJECT
}
set_top rgb2gray
add_files $SRC_DIR/rgb2gray_hls_v8.cpp $SRC_DIR/rgb2gray_hls_v8.h
add_files -tb $SRC_DIR/tb_rgb2gray_hls_v8.cpp
open_solution -reset $SOLUTION -flow_target vitis
set_part $PART
create_clock -period 10ns

puts "============================================================"
puts "C 仿真：验证 sigma 16 bank 归约后数值不变"
puts "============================================================"
csim_design

puts "============================================================"
puts "csim 结束。看上面有没有 FAIL / 多少例 PASS。"
puts "============================================================"
exit 0
