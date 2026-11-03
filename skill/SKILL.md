# PYNQ-Z2 HLS Image Acceleration — Reusable Engineering Skill

把本项目（PYNQ-Z2 / xc7z020clg400-1，Vitis HLS 2026.1 + Vivado 2026.1）
从零做到 3.258 ms @640×480 过程中踩过的坑、用过的判据、写出来的脚本，
整理成**换项目 / 换板卡仍然可用**的技能包。

> 目录与文件名全部为**纯英文**（保证跨平台复现与脚本自动化）。内容写作语言为中文，
> 但器件名、工具报错码、pragma 名一律保留英文原文，方便直接搜索。

---

## 1. 这套技能解决什么问题

在一个"ARM + FPGA 异构 SoC"上，把一个**逐像素的图像算法**搬到可编程逻辑里，
并且跑出性能（不是“功能对了但比 ARM 慢”）。

适用边界：

- 适用：PYNQ-Z2 / Zynq-7000 系列、Vitis HLS 导出 IP + Vivado Block Design、
  算法是**流式**的（逐像素 / 小窗口滑窗）
- 需改写：Zynq UltraScale+ / Versal（AXI 是 AXI4，burst 上限不同）、
  Versal AIE、纯 RTL 流程
- 不适用：需要大矩阵随机访问、需要片外 DDR 缓存中间结果的算法

---

## 2. 核心方法论（先于任何代码）

### 2.1 三个必须分清的口径

性能数字混用口径会让同一份材料前后矛盾。

| 档 | 含义 | 本项目数值 |
|---|---|---|
| ① 核内 kernel | `ap_start` → `ap_done`，**主口径** | **3.258 ms** |
| ② kernel + cache 维护 | 加上 ARM 侧 flush/invalidate | ≈ 3.74 ms |
| ③ 处理链路 | 再加 ARM 侧 RGBA 打包（不含网络推送） | ≈ 13.6 ms（反算约 73 fps，非端到端帧率） |

**规则**：同一份材料里三档可以并列，但**每一处都必须标明是哪一档**，
绝不出现"端到端 3 ms"这种话。

### 2.2 两个必须分清的资源口径

- **IP 本体**（Vivado `report_utilization` 里选中 IP 实例）
- **全工程**（含 PS7 硬核封装、AXI 互联、时钟复位）

两者都真实，**不可混用**。本项目：IP 本体 vs 全工程 LUT 差约 600。

### 2.3 定位性能问题：控制变量 + 找反例

> 凡"工具全绿但结果不对"，必须上板实测 + 控制变量法定位；
> 并且**先找反例证伪自己的假设**。

本项目耗时最长的一次定位（案例 10）：三样本表面上互相印证
（v7 无 σ = 3.577 ms / v8 有 σ = 20.57 ms / v8 的 GRAY 无 σ = 3.224 ms），
结论是"σ 累加器拖慢 5.8 倍"——**完全错误**。
纠正结论的是一个反例：**op 9~11 根本没有 σ 累加，却一样慢**。

---

## 3. 案例索引

完整 17 例见 `cases/`，按主题分 6 个文件。

| 文件 | 覆盖案例 | 主题 |
|---|---|---|
| `01_environment.md` | 1, 2, 9 | 环境：器件许可、板卡文件、中文路径 |
| `02_performance_method.md` | 3, 4, 5, 6, 7 | 性能测量方法论（含"建议本身也要验证"） |
| `03_bitstream_version.md` | 8 | 如何判断位流新旧（最容易静默出错的一处） |
| `04_hls_axi_burst.md` | **10**, 16 | **AXI burst / m_axi pragma**（性能主要瓶颈） |
| `05_hls_syntax_traps.md` | 11, 12, 13, 14, 15 | HLS 语言与接口陷阱 |
| `06_script_drift.md` | **17** | 脚本文案过期：判定对、说明错 |

加粗的两个覆盖本项目两次关键定位：案例 10 把核内耗时从 20.57 ms 修到 3.258 ms；
案例 16 是 HLS 全部通过、Vivado 阶段才报错的情形。

---

## 4. 可复用脚本

| 脚本 | 用途 |
|---|---|
| `scripts/build_hls.tcl` | HLS IP → 位流 + `.hwh` 一键构建（`reset_run` 不可删） |
| `scripts/run_hls_cli.md` | 命令行跑 Vitis HLS 三步（不用 GUI） |
| `scripts/mirror_ab.py` | **改 HLS 循环结构前的 Python 对拍模板** |
| `scripts/update_bitstream.py` | 新位流 → 板端交付包，一键同步 MD5（三处） |
| `templates/case_template.md` | LLM 协作案例的写作模板（含五段判定标准） |

`scripts/mirror_ab.py` 是本包最主要的工具：改循环结构先在 PC 上对拍，
不必烧位流上板才发现边界错了 —— 几分钟、702 例逐字节比对。
（它跑的是 op 0~11 的真实实现，可直接运行；头部写了怎么改成你自己的。）

---

## 5. 使用顺序（下一个项目照抄）

1. 先写 `mirror_ab.py` 的对拍（改前/改后两份 Python 镜像），**再动 HLS 代码**
2. HLS 改完：命令行跑 `v++ -c` → `--impl` → `--package`
3. **检查 csynth 报告的 `M_AXI Burst Information`**，主循环必须是 `Inferred`
4. 装 IP → `build_hls.tcl` 出位流 → **重算 MD5 并同步三处**
5. 上板：`check_kit.py` → `validate_v8.py` → `diag_slow.py`
6. `diag_slow` 的 cyc/px 收敛到 ~1 即完成

---

## 6. 一条硬约束速查（Zynq-7000）

| 参数 | 上限 | 说明 |
|---|---|---|
| `max_*_burst_length` | **16** | GP/HP 口是 AXI3；写 256 只会被拆成若干 16-beat |
| `num_*_outstanding` | **32** | 会传播到 AXI Interconnect 的 `S*_ACCEPTANCE` |
| m_axi 端口位宽 | 由 C 类型决定 | `unsigned char*` → 8 bit，HLS 不会自动加宽 |
| FCLK | 100 MHz | PYNQ-Z2 默认 |

**HLS 不校验这些范围**，越界要等 Vivado `generate_target` 才报错。
