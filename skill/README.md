# skill/ — PYNQ-Z2 HLS 图像加速 · 可复用技能包

> 本目录是**脱离本项目仍然成立**的工程经验集合，不是本项目的代码仓库。
> 方法论与索引见 **[`SKILL.md`](SKILL.md)**，本文件只做导航。

## 从哪读起

| 你想解决什么 | 读哪个 |
|---|---|
| 不知道从哪下手 | [`SKILL.md`](SKILL.md) §2 方法论 + §5 使用顺序 |
| 性能不对（综合全绿但慢） | [`cases/04_hls_axi_burst.md`](cases/04_hls_axi_burst.md) |
| 要改 HLS 循环结构 | 先跑 [`scripts/mirror_ab.py`](scripts/mirror_ab.py) 对拍 |
| HLS 编译/接口有怪问题 | [`cases/05_hls_syntax_traps.md`](cases/05_hls_syntax_traps.md) |
| 怀疑位流版本不对 | [`cases/03_bitstream_version.md`](cases/03_bitstream_version.md) |
| 环境装不起来 | [`cases/01_environment.md`](cases/01_environment.md) |
| 性能数字怎么测才可信 | [`cases/02_performance_method.md`](cases/02_performance_method.md) |
| 脚本判定对了但话不对 | [`cases/06_script_drift.md`](cases/06_script_drift.md) |
| 要写 LLM 协作案例 | [`templates/case_template.md`](templates/case_template.md) |
| 想让 LLM 稳定完成某类任务 | [`prompts/README.md`](prompts/README.md)（提示词工作流） |
| 不知算法该放 PL 还是 PS | [`prompts/01_hw_sw_partition.md`](prompts/01_hw_sw_partition.md) |
| 综合报告全绿但性能不对 | [`prompts/02_report_bottleneck.md`](prompts/02_report_bottleneck.md) |
| 要造黄金参考做对拍 | [`prompts/03_golden_reference.md`](prompts/03_golden_reference.md) |

## 三条要点

1. **吞吐被访存定住，不被运算定住** —— 先做带宽账，再谈并行度。
2. **m_axi 上的写要无条件、地址连续** —— 一个 `if` 就能把 burst 打没，慢 6.7 倍。
3. **`csynth` 的 II 是估计值** —— "综合全绿但性能不对"必须上板实测 + **先找反例证伪自己的假设**。

## 目录

```
skill/
├── README.md              本文件（导航）
├── SKILL.md               方法论 + 案例索引 + Zynq-7000 硬约束速查
├── cases/
│   ├── 01_environment.md        环境：器件许可 / 板卡文件 / 中文路径
│   ├── 02_performance_method.md 性能测量方法论（含"建议本身也要验证"）
│   ├── 03_bitstream_version.md  如何判断位流新旧
│   ├── 04_hls_axi_burst.md      AXI burst / m_axi pragma
│   ├── 05_hls_syntax_traps.md   HLS 语言与接口陷阱
│   └── 06_script_drift.md       脚本文案过期
├── prompts/             提示词工作流（经本项目验证的模板 + 使用说明）
│   ├── README.md              三条通用原则：要判据 / 先找反例 / 回读源码
│   ├── 01_hw_sw_partition.md  从算法推导软硬件划分
│   ├── 02_report_bottleneck.md 从综合报告定位瓶颈（含反例证伪步骤）
│   └── 03_golden_reference.md 生成独立黄金参考并对拍
├── scripts/
│   ├── build_hls.tcl            HLS IP → 位流 + .hwh 一键构建
│   ├── run_hls_cli.md           命令行跑 Vitis HLS（不用 GUI）
│   ├── mirror_ab.py             改循环结构前的 Python 对拍模板
│   └── update_bitstream.py      新位流 → 一键同步三处 MD5
└── templates/
    └── case_template.md         LLM 协作案例写作模板
```

## 每一项的适用范围 / 使用方法 / 失效条件

| 项 | 适用范围 | 使用方法 | 在本项目中验证过的效果 | 失效条件 |
|---|---|---|---|---|
| `cases/` 踩坑清单 | 工具链、位流版本、性能判据、HLS 语法这几类反复出现的问题 | 按现象查对应案例，照里面的"触发条件 → 排查步骤"复现；每条都带反例 | 案例 10 把核内耗时 20.57 ms 修到 3.258 ms | 现象不在已收录的六类里时，清单给不出判据，要走 `SKILL.md` §2 的控制变量法 |
| `prompts/` 提示词模板 | 软硬件划分、瓶颈定位、黄金参考生成这三类高频判断 | 复制模板、按 `【】` 填实际信息；保留"要求给出判据"那一步 | 定位到条件 DDR 写打掉 AXI burst；产出 `filters.py` 支撑 180 组逐像素对拍 | 模板里的判据依赖本项目寄存器/DDR 口径，换平台要先改判据再套用 |
| `scripts/` 脚本 | 改 HLS 循环结构、命令行综合、位流版本同步 | `mirror_ab.py` 先 PC 对拍 → 改 HLS → `build_hls.tcl` 出位流 → `update_bitstream.py` 同步 MD5 | 702 例逐字节对拍；位流 MD5 三处同步，避免旧网表静默复用 | 非 Vivado Block Design 流程（纯 RTL / Versal）、或没有 Python 黄金参考时不可用 |
| `templates/` 案例模板 | 记录带纠错轨迹的排障案例 | 按五段填空；标题标出"含一次归因错误" | 17 个案例按统一格式整理，错误假设与证伪证据都可复查 | 只适用于有"先判错、后纠正"过程的排障，不适用于纯功能实现记录 |

## 适用范围与失效条件

- 适用：PYNQ-Z2 / Zynq-7000 系列，Vitis HLS 导出 IP + Vivado Block Design，
  算法是**流式**的（逐像素 / 小窗口滑窗）
- 需改写：Zynq UltraScale+ / Versal（AXI4，burst 上限不同）、纯 RTL 流程
  —— 主要是 §6 那张硬约束速查表的数字要换
- 不适用：需要大矩阵随机访问、必须把中间结果存回片外 DDR 的算法
