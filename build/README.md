# build

| 文件 | 用途 |
|---|---|
| `build_v8.tcl` | 打开已有工程，升级 IP、重生成 BD、综合实现，导出 `.bit` / `.hwh` |
| `RGB2gray.cfg` | Vitis HLS 命令行配置（器件、时钟、流程） |
| `MD5.txt` | 位流与 `.hwh` 的 MD5、时序与资源实测记录 |
| `timing_util.txt` | 实现后的 WNS / WHS |
| `ip_repo/rgb2gray/` | 打包 IP 整包：`component.xml`、Verilog/VHDL RTL、`drivers/`、`xgui/`、OOC 约束 |
| `bd/design_1.tcl` | 从零重建 Block Design 的脚本（无绝对路径，推荐用这个） |
| `bd/design_1.bd`、`bd/design_1.bda` | 原工程导出的 BD 快照，留作对照 |

`ip_repo/rgb2gray/hdl/verilog/impl.bat` 里保存的是生成 IP 时的 Vivado 安装路径（`D:/AMDDesignTools/2026.1/...`）。`component.xml` 引用了这个文件，所以按原样保留；重新 package IP 时会重写。

## 从零建工程

`build_v8.tcl` 假设工程 `img_accel.xpr` 已经存在，且 IP 仓库在 `D:/vivado/ip_repo`。换机器时按下面顺序建：

```tcl
create_project img_accel ./img_accel -part xc7z020clg400-1
set_property ip_repo_paths {<repo>/build/ip_repo} [current_project]
update_ip_catalog
source {<repo>/build/bd/design_1.tcl}
set bd [get_files design_1.bd]
generate_target all $bd
make_wrapper -files $bd -top
```

`design_1.tcl` 由 Vivado 2026.1 的 `write_bd_tcl` 导出，包含 PS7 配置、AXI 互联、`rgb2gray_0` 实例、时钟复位和地址映射（`s_axi_control` 在 `0x40000000`）。它要在**已经打开工程**的上下文里 source，而且 IP 仓库必须提前加好，否则会在 `create_bd_cell` 处报找不到 `xilinx.com:hls:rgb2gray:1.0`。

## 出位流

工程建好后跑 `build_v8.tcl` 的第 4~6 步：

```tcl
reset_run synth_1
launch_runs impl_1 -to_step write_bitstream -jobs 8
wait_on_run impl_1
```

`reset_run` 不能省。Vivado 会复用旧网表，表现为「换了 IP 但位流没变」，而所有报告都是绿的。

## 验收

位流出来先算 MD5，与 `MD5.txt` 比对。`.bit` 各版本大小恒为 4045771 B，看大小分不出版本。
