# 复现包 — AG-Canny（PYNQ-Z2 / XC7Z020）

> 目标：让一个没参与本项目的人，**照本文件从源码重建出同一份位流**，
> 并用同一套判据确认"重建出来的和交付的是同一个东西"。

---

## 0. 环境与版本（先固化）

| 项 | 版本 / 值 |
|---|---|
| 工具 | **Vivado / Vitis 2026.1**（win64） |
| 器件 | xc7z020clg400-1（PYNQ-Z2） |
| 许可 | Vivado Basic Tier + Vitis HLS（No Charge） |
| 板卡文件 | PYNQ-Z2 `A.0`（TUL，**2026.1 安装包不随附**，需手动放） |
| 板端镜像 | PYNQ v3.1.1 |
| Python（板端） | `/usr/local/share/pynq-venv`；`numpy>=1.24,<3`、`opencv-python>=4.5,<5` |
| PL 时钟 | 100 MHz（FCLK） |

**许可的坑**：若环境变量 `XILINXD_LICENSE_FILE` 指向 Alveo 专用许可，
Vivado 会把 Zynq-7000 **过滤隐藏**，报 `no licensed parts`
—— 这是字面意思，不是安装包缺器件。`env.ps1` 会主动清掉它。

---

## 1. 一键复现（Windows / PowerShell）

```powershell
cd reproduce
. .\env.ps1              # 设环境（dot-source）
.\run_hls.ps1            # compile → csim → cosim → package，约 12 min
vivado -mode batch -source .\build_bitstream.tcl   # 出 .bit + .hwh，约 10 min
```

产物：

| 阶段 | 产物 | 期望 |
|---|---|---|
| compile | `rgb2gray/hls/` | 主循环 II = 1 |
| csim | log | **864 例，0 失败** |
| cosim | log | **96 例，0 失败** + `C/RTL co-simulation finished: PASS` |
| package | `rgb2gray/rgb2gray.zip` | 含 `component.xml` |
| vivado | `.bit` + `.hwh` | WNS > 0 |

---

## 2. 每一步的验收判据（缺一不可）

### 2.1 `csynth` 报告：`M_AXI Burst Information` 主循环必须 `Inferred`

本项目是 **20 Inferred / 1 Fail**（那条 Fail 是同一访问在外层循环上的冗余条目，属正常）。

> 为什么这一条是硬判据：`csynth` 可以全绿、II 也报 1，
> 但**条件 DDR 写会让 burst 推断失败**，上板直接慢 6.7 倍。
> 详见 `skill/cases/04_hls_axi_burst.md`。

### 2.2 主循环 II = 1

II=2 意味着吞吐直接减半（本项目 5×5 通路理论耗时 3.07 → 6.14 ms）。

### 2.3 grep 生成头文件的寄存器偏移

```
grep ADDR_ rgb2gray/hls/impl/ip/.../xrgb2gray_hw.h
```

**驱动里永远不要硬编码偏移** —— 接口一变（比如多一个返回值），
偏移会整体后移一格，旧驱动会静默写错寄存器。

### 2.4 位流出来之后：算 MD5，和交付记录比对

```
.bit  6b5fd2f907b7214a528530dcca6b418a   (4045771 B)
.hwh  9b7407a3772b4d2800d00b6d65ba751a   (266281 B)
```

注意：各版本 `.bit` 大小恒为 4045771 B，看大小分不出版本**，只能靠 MD5。
重建出来的 MD5 **不一定完全相同**（布局布线有随机性），
但 `.hwh` 必须字节一致、资源/时序必须落在下表范围内。

---

## 3. 期望落点（重建结果应落在这些区间）

| 指标 | 期望值 | 说明 |
|---|---|---|
| 全工程 LUT | 8626 ± 5%（16.21%） | 偏离大说明源码改过 |
| 全工程 FF | 10331（9.71%） | |
| 全工程 RAMB18 | 15（5.36%） | 含 4 块输出行缓冲 `outr` |
| 全工程 DSP | 32（14.55%） | |
| 全工程 WNS | > 0（本项目 **+0.435 ns**） | 必须收敛 |
| 失败端点 | **0** | |
| 主循环 II | 1 | |
| 核内耗时 @640×480（op 0~8） | **3.2~3.4 ms**（≈1.06 cyc/px） | 若 >2.5 cyc/px 说明没修条件 DDR 写 |
| 核内耗时 @640×480（op 9~11 融合） | **3.6~3.8 ms**（≈1.19 cyc/px） | 融合比单算子慢约 12% 属正常，不要用 3.2~3.4 去卡 |

**"核内耗时"是最终判据**。资源对但耗时不对 = burst 没推断出来；
耗时对但资源差很多 = 源码或约束被动过。

---

## 4. 交付物完整性校验

```bash
cd <交付包目录>
md5sum -c MD5SUMS.txt       # 期望 25/25 OK
```

`MD5SUMS.txt` 由 `gen_md5sums.py` **从 `FILES.txt` 生成**，不是重新扫目录：

```bash
python gen_md5sums.py <交付包目录>
```

> 为什么从清单生成而不是扫目录：扫目录**永远校验不出"少了一个文件"**
> （少一个就少一行，照样全绿）。从清单出发才能发现缺失。

板端另有一份 `check_kit.py`，同样**从 `FILES.txt` 读期望值**，
不再手抄（手抄那份曾导致"6 个文件全报 WRONG，而错的只有脚本自己"）。

---

## 5. 上板验证（最后一环）

```bash
ssh xilinx@<板卡IP>
source /usr/local/share/pynq-venv/bin/activate   # 不要加 sudo
cd <工程目录>
python3 -u check_kit.py      # 先验 MD5
python3 -u validate_v8.py    # 12 个 op 对拍
python3 -u diag_slow.py      # 640×480 核内耗时
MIN_ROUGH=1.07 python3 -u live_view_raw.py --auto
```

> **不能加 `sudo`**：`sudo` 触发 `env_reset` → `Device.devices == {}`
> → `No Devices Found`。迷惑点在于纯 numpy 脚本加了 sudo 照样跑通，
> 只有 `import pynq` 才崩，很容易误判成"驱动没装好"。

期望：`diag_slow.py` 的 `last_pl_ms` 对 op 0~8 落在 **3.2~3.4 ms**、op 9~11 落在 **3.6~3.8 ms**。用单一区间去卡会把融合模式误判为失败。

---

## 6. 文件清单

| 文件 | 用途 |
|---|---|
| `README.md` | 本文件 |
| `env.ps1` | 环境变量 + 许可清理（dot-source 执行） |
| `run_hls.ps1` | 命令行跑 Vitis HLS 四步 |
| `build_bitstream.tcl` | HLS IP → 位流 + `.hwh`（`reset_run` 不可删） |
| `gen_md5sums.py` | 从 `FILES.txt` 生成 `MD5SUMS.txt` |

`build/ip_repo/rgb2gray/` 是打包 IP 整包，也是核对 2.3 寄存器偏移的依据：

| 路径 | 内容 |
|---|---|
| `component.xml` | AXI 接口清单、`s_axilite` 寄存器地址表、IP 文件清单 |
| `hdl/verilog/`、`hdl/vhdl/` | Vitis HLS 生成的两套 RTL |
| `drivers/`、`xgui/` | 裸机驱动与 IP 配置界面 |
| `constraints/rgb2gray_ooc.xdc` | OOC 综合时钟 `create_clock -period 10.000`（对应 100 MHz） |

把 `ip_repo_paths` 指向 `build/ip_repo` 后 `update_ip_catalog` 即可直接打开 BD；`build/bd/design_1.tcl` 能在新工程里重建整张 Block Design。要从源码重新生成 IP，仍走 `run_hls.ps1` 的 package 步骤；生成后把新的 `component.xml` 与仓库里的对照，确认 `s_axi_control` 的寄存器偏移没有移动。

排障经验见同仓库 `skill/` 目录（17 个真实案例）。

---

## 7. 已知不可用项

- **overlay 没有 HDMI**：`.hwh` 里 hdmi/vdma/vtc 各 0 次。
  `--display hdmi|both` 会**启动即崩**，只能走 `--display web`。
- **未使用 AXI DMA**：IP 自带 `m_axi_gmem0/1` 直读 DDR。
- **OpenCV 5 移除了 `CascadeClassifier`**：装 5.x 会在导入特征检测时报错，
  必须锁 `<5`。
