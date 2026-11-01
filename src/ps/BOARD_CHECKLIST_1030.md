# v8.2 上板清单（2026-10-30 用）

> 这份只写**明天上板要干什么**，原理和排障细节看 `README_上板说明_v82.md`。
>
> 本包目录名就叫 **`rgb2gray_v82`**，scp 到板端 `jupyter_notebooks/` 下**直接就是最终目录**，
> 不用再 mkdir + cp。文件分档见 `FILES.txt`（A=最小演示集 11 个 / B=排障取证 12 个 / C=文档 3 个）。

---

## 0. 这版相对昨天（v8.1）改了什么

| 项 | v8.1 | **v8.2** |
|---|---|---|
| op 9/10/11 | 三个**并列**算子，各算各的 | **同一条四级流水线的三个抽头**：AGC+5×5高斯 → Sobel+方向 → NMS → 双阈值+滞后 |
| 主循环 II | 2 | **1**（滞后左邻改用寄存器，吞吐翻倍） |
| 新增寄存器 | — | **thresh @ 0x38**，Canny 双阈值 `(HIGH<<8)|LOW`，默认 0x3C1E = 60/30 |
| 边界 | 外两圈恒 0 | **外四圈恒 0**（多串一级高斯），最小 9×9 |
| 全工程 LUT | 4194 (7.9%) | **8866 (16.67%)** |
| WNS | +0.955 | **+0.458**（仍 0 failing） |

**op 编号**：9 = 抽级1（去噪图）／11 = 抽级3（NMS 幅值）／10 = 抽级4（0/255 边缘）。
四级硬件**全都在跑**，op 只决定从哪一级把结果写回 DDR。

---

## ⚡ 传完立刻跑（速查 6 步，不想看文档就照抄这段）

下面每一段都可以整块复制粘贴。**全程不加 sudo**（原因见 §3）。

### ① 登录 + 进环境

```bash
ssh xilinx@192.168.2.99
source /usr/local/share/pynq-venv/bin/activate
cd /home/xilinx/jupyter_notebooks/rgb2gray_v82
```

### ② 先验位流（**这步别跳**）

```bash
python3 -u check_kit.py
```

必须看到 `.bit MD5 = e964575439e1bc9603fffda0e3b69f6f`、`.hwh MD5 = 0ede3770abdf7fa87d181e674ea9456b`。
对不上就别往下跑 —— 说明传错文件了，回去看 §2。

### ③ 冒烟：12 个 op 各跑一帧（20 秒）

```bash
python3 -u -c "
import numpy as np, hw_driver
p = hw_driver.HardwareProcessor()
print('位流:', p.bit_manifest)
yy, xx = np.indices((480, 640))
v = (((yy // 40) + (xx // 40)) % 2 * 255).astype(np.uint8)
img = np.dstack([v, v, v])
for m in range(12):
    o = p.run(m, img)
    print('  op %2d   %6.3f ms   输出 %s  非零 %d'
          % (m, p.last_pl_ms, o.shape, int((o != 0).sum())))
"
```

**判据**（下面这组数是用同一张 640×480 棋盘图在参考实现上跑出来的，
真板 PL 应当与它**逐点一致** —— 差一点就说明硬件或位流有问题）：

| op | 输出非零点数（应当等于） | 含义 |
|---|---|---|
| 0 GRAY | **153600** | 棋盘 0/255 各半，一半非零 |
| 1 SOBEL | 27716 | 边缘响应 |
| 5 LAPLACE | 13858 | |
| 8 DIR | 14036 | 方向量化 |
| **9 GAUSS5** | **175896** | 抽头 A：去噪图，比原图"糊"，非零更多 |
| **10 CANNY** | **27404** | 抽头 C：二值边缘，**约 2.7 万**，不是 0 也不是 30 万 |
| **11 NMS** | **27404** | 抽头 B：与 op 10 非零点数相同（同一个极大值集合） |

耗时一列：真板应当都在 **3~4 ms**；本机没板子时这列会是 `0.000`（走 SIM，
命令行会先打印 `falling back to SIM mode`），属正常。

**最容易一眼看出问题的三种情况**：

- 打印了 `falling back to SIM mode` → 位流没加载起来，**后面的数全是假的**
- op 3~11 输出看着一模一样 → 载成了旧位流（v4 只有 mode 0/1/2，其余落 else 输出 EMBOSS）
- op 10 非零 = 0 或 = 30 万 → 阈值寄存器没写进去，回头看 §4 的阈值扫掠

### ④ 完整回归 + 创新证据（3~5 分钟，这是主脚本）

```bash
python3 -u validate_v8.py
```

最后必须打印「硬判据全部通过」。阶段 6 的四张表**截图存证**，那是创新 30 分的证据。

### ⑤ 测性能

```bash
python3 -u diag_slow.py
```

我要的是 640×480 的 `last_pl_ms`。II 修到 1 之后，5×5 通路应该和 3×3 一样落在
**~3.1 ms**。（`validate_v8.py` 阶段 5 也会打印全部 12 个模式的耗时，两个可以对着看。）

### ⑥ 开演示

```bash
python3 live_view_raw.py --hw --display web
```

笔记本浏览器开 `http://192.168.2.99:8080/`。

> ⚠️ **只能 `--display web`**。这份 overlay 里**没有 HDMI**（`.hwh` 里 hdmi/vdma/vtc
> 各 0 次），`--display hdmi|both` 启动即崩。

---

## 1. 接线与连通

| 项 | 值 |
|---|---|
| 板子 IP | **192.168.2.99**（笔记本网线直连，静态 IP） |
| 用户名 / 密码 | `xilinx` / `xilinx` |
| SSH | `ssh xilinx@192.168.2.99`（默认 22；若你之前改过端口就用那个） |
| Jupyter Lab | `http://192.168.2.99:9090` |
| 演示网页 | `http://192.168.2.99:8080` |

摄像头 **C270 直插板载 USB 口**，不要过 HUB。

Windows 上先确认能连通：

```powershell
ping 192.168.2.99
```

## 2. 传文件

### 方式 A：`scp`（推荐，一条命令）

在本机 **PowerShell / Git Bash** 里（不是板子上）：

本机包目录就叫 **`rgb2gray_v82`**，所以一条命令到位 —— 传完**不用再 mkdir + cp**：

```bash
scp -r "C:/Users/还原剂/Desktop/2026大二上FPGA/上板上传_1020/rgb2gray_v82" \
    xilinx@192.168.2.99:/home/xilinx/jupyter_notebooks/
```

传完后在板子上确认（目录名应当就叫 `rgb2gray_v82`，不是 `upload_v82_1029`）：

```bash
ls -la /home/xilinx/jupyter_notebooks/rgb2gray_v82/
```

应当看到 **26 个文件**：21 个 `.py` + `rgb2gray.bit` + `rgb2gray.hwh`
+ `FILES.txt` + 2 个 `.md`。

只传 A 档（最小演示集 11 个）也行，网速慢时用这条：

```bash
scp "C:/Users/还原剂/Desktop/2026大二上FPGA/上板上传_1020/rgb2gray_v82/{check_kit,control,filters,gesture,hw_driver,live_view_raw,session,swipe,validate_v8}.py" \
    "C:/Users/还原剂/Desktop/2026大二上FPGA/上板上传_1020/rgb2gray_v82/rgb2gray.{bit,hwh}" \
    xilinx@192.168.2.99:/home/xilinx/jupyter_notebooks/rgb2gray_v82/
```

> ⚠️ 但这个 `{a,b,c}` 花括号展开 **PowerShell 不支持**，要在 **Git Bash** 里跑；
> PowerShell 请逐个文件写完整路径，或者直接用上面的整目录 `scp -r`。

### 方式 B：Jupyter Lab 上传

浏览器打开 `http://192.168.2.99:9090`，进 `jupyter_notebooks/` 下
**新建 `rgb2gray_v82` 目录**，再全选拖进去。

> ⚠️ **Jupyter 上传同名文件会自动改名**成 `rgb2gray (1).bit`——
> 这是历史上载错位流的直接原因之一。上传完**必须**确认文件名没有空格和 `(1)`。

> ⚠️ **`.bit` 和 `.hwh` 必须同名**（`rgb2gray.bit` + `rgb2gray.hwh`），
> PYNQ 的 Overlay 是靠文件名配对找 `.hwh` 的，名字对不上会加载失败。

#### ❓ 为什么以前是 `rgb2gray_v5`？目录名和版本到底什么关系

**目录名 ≠ 位流版本。** `rgb2gray_v5` 是 10-19 那天建的目录，建完之后每次升级
都在原地覆盖文件，名字一直没改 —— 所以它里面跑的早就不是 v5 了。这是纯粹的
历史遗留，也是"为什么是 v5"这个疑问的来源。

真正的版本在**位流文件里**，而位流文件名就叫 `rgb2gray.bit`，**不带版本号**：

- `hw_driver.py` 一律取**脚本同目录**的那份 `rgb2gray.bit`（不再有任何绝对路径兜底）
- 版本靠 **MD5** 判定：`.bit` 必须是 `e9645754…`，`.hwh` 必须是 `0ede3770…`
- 各版 `.bit` 大小**恒为 4045771 字节**，v4/v5/v6/v7/v8.1/v8.2 完全一样，
  **看大小分辨不出来** —— 所以 MD5 是唯一判据

所以目录叫 `rgb2gray_v82` 也好、叫 `rgb2gray_v5` 也好，只要旁边那份
`rgb2gray.bit` 的 MD5 对得上，跑的就是 v8.2。**这次统一改成 `rgb2gray_v82`，
把"目录名暗示版本"这个误导源彻底消掉。**

> 10-30 顺手清掉的同类隐患：`live_view_raw.py` 里曾兜底到
> `rgb2gray_v4/rgb2gray.bit`、`hw_driver.py` 里曾兜底到 `/home/xilinx/rgb2gray/`。
> 本目录找不到位流时它们会**悄悄跑去加载旧版本**，画面表现为"滤镜全变成浮雕"。
> 现在全部删掉，找不到就直接报错 —— 宁可报错，不要静默跑错版本。

---

## 3. 板端环境（**一律不加 sudo**）

```bash
ssh xilinx@192.168.2.99
source /usr/local/share/pynq-venv/bin/activate
cd /home/xilinx/jupyter_notebooks/rgb2gray_v82
```

> ⚠️ 不加 sudo。加了会 env_reset → `Device.devices=={}` → No Devices Found。
> 迷惑点：纯 numpy 脚本全 PASS，一 import pynq 就崩。

---

## 3B. 先验位流（这一步别跳）

```bash
python3 check_kit.py
```

必须看到：

```
.bit  MD5 = e964575439e1bc9603fffda0e3b69f6f
.hwh  MD5 = 0ede3770abdf7fa87d181e674ea9456b
```

`.bit` 大小恒为 **4045771 B**，v4/v5/v6/v7/v8.1/v8.2 全都一样，**只有 MD5 能区分版本**。
10-19 踩过一次载错位流：mode≥3 全输出 EMBOSS，画面看着像"后几个滤镜都一样"，极难定位。

第二道判据（不看 MD5 也能分辨）：

```bash
grep -c ap_return rgb2gray.hwh      # 必须是 4（v7 是 0）
```

---

## 4. 跑性能（我最想要的数）

```bash
python3 -u diag_slow.py             # -u 必须加，不然 | tee 会 8KB 块缓冲
```

**重点看 640×480 的 `last_pl_ms`**。II 从 2 修到 1 之后，5×5 通路理论上应该和
3×3 一样落在 **~3.1 ms**。如果明显更高，把整个输出贴给我，说明还有别的瓶颈。

参考值（v5 时代，同口径）：GRAY 3.391 / 其余 3×3 算子 3.775~3.784 ms。

---

## 5. 对拍（正确性）— **主脚本**

```bash
python3 -u validate_v8.py           # 12 个模式：op 0..8 单算子 + op 9/10/11 三抽头
```

> 10-30 这版 `validate_v8.py` 已重写到 v8.2 口径。之前的版本 `MODE_NAMES` 只到 8，
> 跑它**根本测不到** op 9/10/11 —— 别再拿旧版的结果当"已验证"。

六个阶段，一路跑完约 3~5 分钟：

| 阶段 | 内容 | 判据 |
|---|---|---|
| 0 | 纯 Python 自检（filters 切片版 vs 脚本朴素循环版） | 8 图 × 12 模式 × 3 增益全等 |
| 1 | 位流加载 + MD5 | 见上 |
| 2 | 64×64 五图 × 12 模式 × 3 增益逐像素对拍 | 180 组 maxdiff=0 |
| 3 | σ 回读对拍 + **4 个哨兵** | 72 组精确相等；op0/9/10/11 必须 Σ=0 且 `sigma_valid=False` |
| 4 | 融合流水线专项：阈值扫掠 / 单趟 vs 级联 / 9×9 最小尺寸 | 三档阈值输出必须互不相同（证明 0x38 真接上了） |
| 5 | 640×480 全 12 模式核内耗时 | 看 `last_pl_ms` |
| 6 | **创新证据采集**（直接进报告） | 见下 |

PC 上没板子时可以先验脚本逻辑：

```bash
python3 -u validate_v8.py --sim     # 全程软件参考；耗时数字无意义，别写进报告
```

### 5x5 通路的正确打开方式

- **op 9（GAUSS5）** = 去噪图，应该比原图糊一点
- **op 11（NMS）** = 细化后的幅值，边缘应该是**单像素宽**的细线
- **op 10（CANNY）** = 0/255 二值边缘

注意**最外四圈恒 0**（不是两圈），这是四级流水线的固有延迟，不是 bug。
最小 9×9（9×9 时有效区只剩中心 1 个像素，阶段 4 有这个哨兵）。

### 10-30 顺手修掉的一个真 bug

`hw_driver.py` 里自适应阈值的系数原来是 `CANNY_K_HI = 16.0`，推导时**漏了级1
去噪把噪声衰减到 0.2734 倍**这一步。后果：σ=8 时给出 HIGH=128，实测**真边覆盖率
直接归零** —— 演示时一开 `canny_adaptive`，边缘全没了。

已改为 **7.56σ**（= 5 倍噪声响应 × 1.512），实测甜点正是 7.5σ。
阶段 6(4) 那张工作点扫描表就是这次修正的依据，**拍照存证**。

同一天还统一了 `sigma.py` 的 σ→阈值系数（原来 3.0σ，和驱动不是一套数），
现在从 `hw_driver` 取，只有一个真相源。

---

## 6. 采创新证据（15 分那块，最关键）

```bash
# ① 校准：开灯、手掌不进画面，采背景
python3 -u sigma.py --calib --frames 30

# ② 扫光照：从亮到暗走一遍
python3 -u sigma.py --sweep --steps 6
```

要拿到的证据：

- σ 随光照变暗而上升 → 说明噪声真的被量化出来了
- AGC 打开后 σ 回落 → 说明增益补偿生效
- **关灯前后 op 10 的边缘数不该塌** → 这是"光照自适应"最直观的一张图

σ 有效条件：`0 < op < 9`。**op 9/10/11 恒返回 0**（走融合流水线，不累加 σ），
所以要用 op 1~8 里的某个算子去测 σ。

---

## 7. 手势（验收第一优先级）

```bash
python3 -u live_view_raw.py --display web
```

上板顺序（**顺序错了会一直认不出手**）：

1. 网页上 **MASK ON**
2. 手掌**盖满黄框**
3. 点 **RECALIBRATE**
4. 掩膜应该是"只有手是白的，墙是黑的"

- 校准采到背景的坑已修（背景相似度判据 H±8 S±14 V±22），真手应得 `[0,44,60]`
- 手势线程只有 **3.2 Hz**，挥手别太快
- **左挥 = 下一个模式**，右挥 = 上一个
- 验收第一优先级 = **空手 60 秒误报 0 次**

---

## 8. 明天结束前把这几样给我

1. `diag_slow.py` 的完整输出（我要 640×480 的 `last_pl_ms`）
2. `validate_v8.py` 的**完整输出**（12 个模式的耗时 + 阶段 6 的四张证据表）
3. `sigma.py --sweep` 的数据
4. 关灯/开灯下 op 10 的截图各一张
5. 空手 60 秒误报次数
6. op 9 / op 11 / op 10 **同一帧的三张截图**（证明是同一条链的三个抽头）

我拿到就填进设计报告的 §4 性能表和创新章节。

> 阶段 6 的（3）（4）两张表是创新 30 分的核心证据，**截图 + 数字一起留档**。
> PC 侧参考值（软件实现，真板数字以实测为准）：
> · 纯噪声图伪边：Sobel 216072 → NMS 10822（−95.0%）→ Canny 0
> · 低对比度边 + σ=8：等覆盖率下 Sobel FP 5894 → Canny FP 194（**−96.7%**），真边覆盖 97.1%
> · 阈值工作点：3σ 太松 / **7.5σ 甜点** / 10σ 开始丢边 / 16σ 全丢

---

## 附：寄存器表（v8.2，已实测确认）

| 偏移 | 寄存器 | 访问 |
|---|---|---|
| 0x00 | AP_CTRL | RW |
| 0x10 | **ap_return（σ）** | **R** |
| 0x18 | width | W |
| 0x20 | height | W |
| 0x28 | mode | W |
| 0x30 | gain | W |
| 0x38 | **thresh（Canny 双阈值）** | W |

`thresh = (HIGH << 8) | LOW`，默认 `0x3C1E`（60/30）。ARM 可每帧按实测 σ 改写（默认关）。

⚠️ **v8 起偏移整体后移一格**（σ 挤占 0x10）。旧驱动配新位流：width 写进只读的 σ
→ 静默无效，height 写进 width → "画面只对上半部分生效"。**极难排查**。
