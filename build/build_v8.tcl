# =============================================================================
# build_v8.tcl —— 一键从 v8 HLS IP 到位流 + .hwh 交付包
#
# 用法（Vivado Tcl 命令行 / 或 vivado -mode batch -source build_v8.tcl）：
#     vivado -mode batch -source D:/vivado/build_v8.tcl
#
# 前置条件：D:/vitis/RGB2gray/rgb2gray/hls/impl/ip 里的 v8 IP 已经拷进
#           D:/vivado/ip_repo/rgb2gray（v7 原版已备份到 rgb2gray_v7_bak_1028）
#
# 自动跑的依据：v8 相对 v7 只在 s_axi_control 里多了一个只读的
#   ap_return 寄存器，AXI-Lite 总线接口（端口名、位宽、地址空间）完全没变，
#   所以 BD 不需要改连线，只需要"升级被锁定的 IP"。
#
# 出问题的回滚：把 ip_repo/rgb2gray 换回 rgb2gray_v7_bak_1028 再跑一遍即可。
# =============================================================================

set PROJ   "D:/vivado/img_accel/img_accel.xpr"
set OUTDIR "D:/vivado/deliver/rgb2gray_overlay_v8"
set RUNS   "D:/vivado/img_accel/img_accel.runs/impl_1"
set HWH    "D:/vivado/img_accel/img_accel.gen/sources_1/bd/design_1/hw_handoff/design_1.hwh"

puts "=========== [clock format [clock seconds]] build_v8 start ==========="
open_project $PROJ

# ---------------------------------------------------------------- 1. IP 仓库
set_property ip_repo_paths {D:/vivado/ip_repo} [current_project]
update_ip_catalog -rebuild
puts "---- IP status ----"
report_ip_status

# ------------------------------------------------------- 2. 升级 rgb2gray IP
set ips [get_ips -quiet design_1_rgb2gray_0_1]
if {[llength $ips] > 0} {
    puts "---- upgrading $ips ----"
    upgrade_ip $ips
} else {
    puts "WARNING: design_1_rgb2gray_0_1 not found; check BD"
}

# ------------------------------------------------- 3. 重新生成 BD 输出产物
set bd [get_files -quiet */design_1.bd]
if {[llength $bd] > 0} {
    generate_target all $bd
} else {
    puts "WARNING: design_1.bd not found"
}

# ---------------------------------------------- 4. 综合 + 实现 + 写位流
# reset_run 避免 Vivado 拿缓存的旧网表静默生成 v7 的位流，
# 也就是"换了 IP 但位流没变"。
reset_run synth_1
launch_runs impl_1 -to_step write_bitstream -jobs 8
wait_on_run impl_1

if {[get_property PROGRESS [get_runs impl_1]] ne "100%"} {
    puts "ERROR: impl_1 did not complete"
    puts "STATUS: [get_property STATUS [get_runs impl_1]]"
    exit 1
}

# ------------------------------------------------------------- 5. 交付
file mkdir $OUTDIR
set bitfile [lindex [glob -nocomplain $RUNS/*.bit] 0]
if {$bitfile eq ""} {
    puts "ERROR: no .bit in $RUNS"
    exit 1
}
file copy -force $bitfile $OUTDIR/rgb2gray.bit
if {[file exists $HWH]} {
    file copy -force $HWH $OUTDIR/rgb2gray.hwh
} else {
    puts "WARNING: .hwh not found at $HWH"
}

puts "=========== [clock format [clock seconds]] build_v8 done ==========="
puts "BIT: $OUTDIR/rgb2gray.bit"

# -------------------------------------------------- 6. 时序与资源速报
set f [open $OUTDIR/timing_util.txt w]
puts $f "WNS  : [get_property STATS.WNS [get_runs impl_1]]"
puts $f "WHS  : [get_property STATS.WHS [get_runs impl_1]]"
close $f
set fh [open $OUTDIR/timing_util.txt r]
puts [read $fh]
close $fh
