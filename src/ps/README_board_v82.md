# v8.2 上板包使用说明（10-30 整理）

> **AG-Canny：四级融合流水线 + σ 噪声回传版**（v8.2，op 9/10/11 是同一条链的三个抽头）
> 板端目标目录：**`/home/xilinx/jupyter_notebooks/rgb2gray_v82`**（**新建，别覆盖旧目录**）

---

## 0. 为什么必须新建目录

**v8 的寄存器偏移全部后移了一格** —— σ 走 `ap_return` 挤占了 `0x10`：

| 信号 | v7 | **v8** |
|---|---|---|
| `ap_return`（σ） | — | **`0x10`** |
| width | `0x10` | `0x18` |
| height | `0x18` | `0x20` |
| mode | `0x20` | `0x28` |
| gain | `0x28` | `0x30` |

用 v7 的驱动配 v8 的位流，width 会写进只读的 σ 寄存器 → **静默无效**，
height 写进 width，表现是"画面只对上半部分生效"，极难排查。

新建目录用于隔离；回滚时切回 `rgb2gray_v7` 即可。

---

## 1. 上传（Windows PowerShell）

```powershell
$pkg = "C:\Users\还原剂\Desktop\2026大二上FPGA\上板上传_1020\上传_v8_1029"
scp -r "$pkg\*" xilinx@192.168.2.99:/home/xilinx/jupyter_notebooks/rgb2gray_v82/
```

第一次连会问 `yes/no`，敲 `yes`（ED25519 指纹确认一次，之后不再问）。

> scp 嫌中文路径麻烦就用 MobaXterm 的 SFTP：左边进 `上传_v8_1029`，
> 右边进 `jupyter_notebooks/` 下新建 `rgb2gray_v82`，全选拖过去。

共 22 个文件（详见第 7 节清单）。

### 最小必需集（只有这 10 个是真缺一不可的）

```
rgb2gray.bit   rgb2gray.hwh   hw_driver.py   filters.py
live_view_raw.py   session.py   control.py    gesture.py
swipe.py           check_kit.py
```

其余 12 个是排障/取证用的（`validate_v8.py`、`sigma.py`、`diag_*.py`、`probe_*.py`、
`bench_e2e_v6.py`、`_smoke_*.py`）。**时间紧就先传这 10 个**，能跑起来，
后面要取证再单独补。

> `validate_v7.py` **故意不在这个包里**：它硬编码了 v7 的寄存器偏移
> （width `0x10` / gain `0x28`），配 v8 位流会写出错地址却不报错。
> 需要它就去 `上传_v7_1028\` 拿，别混着用。

---

## 2. 上板顺序（按顺序执行）

```bash
ssh xilinx@192.168.2.99
source /usr/local/share/pynq-venv/bin/activate      # 不要加 sudo
cd /home/xilinx/jupyter_notebooks/rgb2gray_v82

# ⓪ 目录属主 —— **99% 的情况不需要这一步**
#    只有当你用 root 账号拷进去（比如先 scp 到 /root 再 mv）时才要改回来。
#    用 scp xilinx@192.168.2.99 传的文件属主本来就是 xilinx，跳过即可。
#
#    注意：这里的 sudo 是**给 shell 命令提权**，和下面跑 python 完全是两回事。
#       **跑 python 一律不加 sudo**：sudo 会 env_reset 丢掉 pynq-venv
#       → Device.devices == {} → No Devices Found。别把这两件事混了。
# sudo chown -R xilinx:xilinx /home/xilinx/jupyter_notebooks/rgb2gray_v82

# ① 先确认文件齐、版本对
python3 check_kit.py

# ② 核位流 MD5（驱动会自己校验，这里是双保险）
md5sum rgb2gray.bit rgb2gray.hwh
#    4f5ee6932928f4a07b67a6fbbc1086a9  rgb2gray.bit
#    892e23051892e0eb9755b8725277687b  rgb2gray.hwh
# 第二道判据（不用 MD5 也能分辨 v7/v8）：
#    grep -c ap_return rgb2gray.hwh   →  v8 = 4，v7 = 0

# ③ 无摄像头对拍（先别插摄像头）
python3 -u validate_v8.py

# ④ 实时演示
python3 live_view_raw.py --hw rgb2gray.bit --port 8080

# ⑤ 交付前体检
python3 _smoke_dir.py && python3 _smoke_swipe.py && \
python3 _smoke_track.py && python3 _smoke_calib.py
```

### 判定标准

**③ `validate_v7.py`**：

```
阶段 0：ARM 基线 <-> 黄金参考：7 图 x 8 模式 x 3 增益 全部相等（通过）
阶段 1：全部 PASS（120 组）        ← v6 是 40 组，v7 翻三倍
阶段 2：8 个模式 ≈ 3.75~3.78 ms      ← 增益锁 256，与 v6 相同
```

> 阶段 1 有任意一组非 0，**先停下来**，把输出贴给我。
> 最常见的失败原因：位流没换（还载着 v6）、或 `REG_GAIN` 地址不对。

**① `check_kit.py`**：8 个文件全 OK，最后一行 `import 自检：OK`。

---

## 3. 帧率突然变低？—— 10-28 已修，重传 `hw_driver.py` 即可

### 症状（10-28 18:40 上板首次发现）

```
[live] pack 78.8 ms | filter(PL) 86.1 ms (pack 78.8 + kernel 3.27) = 8.7x
[live] pack 96.1 ms | filter(PL) 103.0 ms (pack 96.1 + kernel 3.23) = 8.7x
```

v6 时 pack 是 **4.35 ms**，v7 涨到 **78.8 ms**（18 倍）→ 帧率从 128 fps 掉到 **11.6 fps**。

### 根因

`_decide_gain()` 里用了 `self._in_buf[:n].mean()` 算 AGC 的 μ。
**在 PYNQ 的 cacheable DMA 缓冲上做 numpy reduce，实测 74 ms/帧。**

> 注意：这个坑在 PC 上**完全测不出来** —— 普通 numpy 数组上 `.mean()` 只要 0.16 ms。
> 只有上了板、看了 `pack` 那一项才发现。**以后凡是碰 `_in_buf` / `_out_buf` 的
> numpy 运算，都必须上板实测。**

### 修复：μ 不用读那个缓冲

mean 是线性的：

```
mean(0.299R + 0.587G + 0.114B) == 0.299*mean(R) + 0.587*mean(G) + 0.114*mean(B)
```

所以直接对原始 BGR `frame` 调 `cv2.mean()`（OpenCV C 实现）即可**数学上精确**得到
灰度均值，且**零额外访存**。实测线性性误差 **< 0.001 个灰度级**，新增开销 **0.028 ms**。

### 修复后的预期

| | 修复前 | 修复后（预期） |
|---|---:|---:|
| pack | 78.8 ms | **≈ 4.4 ms** |
| filter(PL) 合计 | 86.1 ms | **≈ 7.7 ms** |
| 帧率（PIPE） | 11.6 fps | **≈ 130 fps** |

> CLIENT 帧率仍受网口限制（640×480 灰度 raw 307 KB/帧 ≈ 24.6 ms → **40 fps 上限**）。
> **CLIENT 低帧率 ≠ 系统慢**，报告必须标注这一点。

### 只需重传一个文件

```powershell
scp "C:\Users\还原剂\Desktop\2026大二上FPGA\上板上传_1020\上传_v7_1028\hw_driver.py" `
    xilinx@192.168.2.99:/home/xilinx/jupyter_notebooks/rgb2gray_v82/

cd /home/xilinx/jupyter_notebooks/rgb2gray_v82
python3 check_kit.py            # hw_driver.py 应显示 21994 / 127650f0 OK
python3 live_view_raw.py --hw rgb2gray.bit --port 8080
```

网页 HUD 上 `pack` 一项应回落到 **4~6 ms**。

---

## 3.5 第二类掉帧：相机侧（10-28 19:00 起，**待板上确认**）

### 现象

修完 `pack` 后，CLIENT 帧率仍然只有十几帧，`diag_fps.py` 给出：

| 阶段 | 结果 | 含义 |
|---|---:|---|
| S1 纯采集 | **15.0 fps** | 相机本身就是瓶颈 |
| S2 PL，AGC 关 | ≈14 fps | PL 只占 14 ms，本可跑 70 fps |
| S3 PL，AGC 开 | ≈13 fps | AGC 不拖后腿 |

PL 有 70 fps 的能力却只跑出 14 fps，**说明卡在采集**。

### `probe_camera.py` 实测（都是裸 `cap.read()` 循环，每项 3 秒）

| 组合 | fps |
|---|---:|
| 640×480 默认（YUYV） | **29.9** |
| 640×480 强制 MJPEG | **30.0** ← 最佳 |
| 640×480 `BUFFERSIZE=1` | **15.0** ← 坑 |
| 320×240 默认 | 见板上输出 |

> **`BUFFERSIZE=1` 是坑**：单缓冲时 V4L2 要等用户取走帧才开始下一次曝光，
> 等于把流水线砍半。**我们的代码里从没设过它**，列在这里是为了防止以后有人"优化"加上。

### `diag_read.py` 实测（10-28 19:25，MJPG 已生效）

| 测试 | fps | ms/帧 |
|---|---:|---:|
| S1a 裸 `cap.read()` | **24.5** | 40.77 |
| S1b `cap.read` + `cv2.flip`（= 现在的 `read_frame`） | **23.9** | 41.83 |
| S1d `cap.read` + `cv2.flip(dst=预分配)` | **24.5** | 40.88 |
| S1c `cap.read` + numpy `[:, ::-1]` 镜像 | 12.9 | 77.67 |

单独反复测（排除采集干扰）：`cv2.flip` **12.41 ms** / numpy 镜像 **54.20 ms** /
`cv2.cvtColor BGR2GRAY` **4.94 ms**。

**结论 1：`cv2.flip` 无罪。** S1a 24.5 → S1b 23.9，只花 **1 ms**。
之前"flip 是掉帧主因"的推测**是错的**，实测否掉了。
（顺带：`cv2.flip` 单独测 12.41 ms 明显高于它在循环里的 1 ms 增量，
说明那 12 ms 里大部分是分配/释放 1 MB 数组的开销，不是翻转本身。）

**结论 2：numpy 镜像确实更慢**，54.20 ms vs `cv2.flip` 的 1 ms 增量，
循环里直接把帧率砍半。幸亏量了，没盲目"优化"。

**结论 3：MJPG 奏效。** 采集从 **15.0 → 24.5 fps（1.6×）**。
这 15 fps 的坑不是 `BUFFERSIZE`，是默认 fourcc 下 YUYV 的 2 B/px
把 USB 和解码都拖住了。

### 现在的真实账本

`cap.read()` 一项就占 **40.77 ms**，瓶颈在这一项。后面每帧还要扣：

| 项 | ms |
|---|---:|
| `cap.read()` | 40.77 |
| `cv2.flip` | ≈1 |
| `cv2.cvtColor BGR2GRAY` | 4.94 |
| PL kernel | 3.07 |
| cache 维护 | 0.40 |
| AGC 的 `cv2.mean` | ≈0.03 |
| 网页推送 | 另计 |

→ PIPE 约 **18~19 fps**，**采集 24.5 fps 就是上限**。

### `check_kit.py` 报 `session.py` WRONG？

那是**体检器自己旧了**（板上还期望 9264/`966b027f`）。
你的 `session.py` 是 10252/`38ed9b24`，**完全正确**。
新板 `check_kit.py` 加了自愈提示，会直接告诉你该重传谁。

### `probe_read.py` 实测（10-28 19:36）—— 相机满速，无劣化

| 配置 | 实测 fps | 相机侧 fps |
|---|---:|---:|
| A MJPG 640×480（当前） | **30.0** | 29.8 |
| B YUYV 640×480 | 29.9 | 29.7 |
| C MJPG 320×240 | 30.0 | 29.7 |
| D MJPG + 手动曝光(-6) | 30.0 | 29.7 |

15 秒长跑：t=3/6/9/12 s 全部 **30.0 fps、33.3 ms/帧**，`POS_MSEC` 线性推进，
**没有渐进劣化**。

**结论：C270 在 PYNQ-Z2 上就是 30 fps 的料，四种配置全都打满，
"越跑越慢"不存在。** 之前 `diag_read.py` 量到的 24.5 fps 是该脚本的测量伪差
（3 s 窗口 vs 4 s 窗口 + import 链差异），不代表相机能力。

> 注意：`probe_camera.py` 的 29.9/30.0 与 `probe_read.py` 的 30.0 一致，
> 两个脚本终于对上了。之前"probe 偏乐观"的判断作废。

### 现在的完整账本（每帧）

| 项 | ms | 谁负责 |
|---|---:|---|
| `cap.read()` | **33.34** | 相机，硬上限 30 fps |
| `cv2.flip` | ≈1 | capture 线程 |
| `cv2.cvtColor BGR2GRAY` | 4.94 | process 线程 |
| PL kernel | 3.07 | PL |
| cache 维护 | 0.40 | process 线程 |
| AGC 的 `cv2.mean` | 0.03 | process 线程 |

四线程跑在 2 个 Cortex-A9 上，capture 34.3 ms 独占近一个核，
其余 9.4 ms + 手势 + 参考窗 + 推送共用另一个 → **PIPE 预期 25~29 fps**。

### `--raw-fps` 从 25 提到 30

相机既然能到 30，CLIENT 的 25 上限就变成了人为瓶颈：

```bash
python3 -u live_view_raw.py --hw rgb2gray.bit --port 8080 --raw-fps 30
```

带宽核算：307 KB × 30 = **9.2 MB/s**，网口实际上限约 11.5 MB/s → 占 80%，可行。

### 判定（看 HUD 三行）

```
PIPE  fps   ← 板内处理完的帧率（应 25~29）
PUSH  fps   ← 板子发出的帧率（应 ≈ PIPE）
CLIENT fps  ← 浏览器收到的（应 ≈ 30）
```

- **PIPE ≈ 25~29** → 一切正常，采集就是上限，报告按"相机 30 fps 上限"写
- **PIPE 高但 CLIENT 低** → 网口带宽，CLIENT 低 ≠ 系统慢（报告必须标注）
- **PIPE 就只有十几** → 把 `--cam-fps 4 --no-gesture` 再跑一次，
  看是不是参考窗/手势在抢 CPU

---

## 4. AGC 效果验证
```bash
python3 -u -c "
import cv2, hw_driver
p = hw_driver.HardwareProcessor('rgb2gray.bit')
cap = cv2.VideoCapture(0); ok, f = cap.read()
dark = (f.astype(np.float32)*0.25).astype(np.uint8)   # 模拟低照度（压暗 4 倍）
for i in range(8): out = p.run(0, dark)               # 多跑几帧让一阶平滑收敛
print('mu=%.1f gain=%d (%.2fx)' % (p.last_mu, p.last_gain, p.last_gain/256))
cv2.imwrite('agc_before.png', cv2.cvtColor(dark, cv2.COLOR_BGR2GRAY))
cv2.imwrite('agc_after.png', out)
"
```

**期望**：`mu` 约 30 左右，`gain` 约 800~900（**3.1~3.5×**）。
`agc_before.png` 应该很黑，`agc_after.png` 接近正常亮度。

**这两张图 + mu/gain 数字可以直接说明"光照自适应"是否生效。**
跑完记得 scp 回 Windows：

```powershell
scp xilinx@192.168.2.99:/home/xilinx/jupyter_notebooks/rgb2gray_v82/agc_*.png .
```

---

## 5. v8 相对 v7 改了什么

| | v7 | **v8** |
|---|---|---|
| 算子数 | 8（mode 0..7） | **9（新增 mode 8 = DIR 梯度方向量化）** |
| σ 估计 | 只在 ARM（numpy） | **PL 每像素累加 + 寄存器回读**（ap_return @ `0x10`） |
| 5×5 高斯 | 无 | **驱动侧级联**：MODE_GAUSS 连跑两遍 |
| 寄存器偏移 | 0x10/0x18/0x20/0x28 | **0x18/0x20/0x28/0x30**（σ 占了 0x10） |

**资源与时序（全工程 Vivado post-route，同口径）**

| | v7 | **v8** | 增量 |
|---|---:|---:|---:|
| WNS | +0.667 ns | **+0.955 ns** | **+0.288（没有退化）** |
| HLS 估计 LUT | 6516 | 6924 | +408 |
| HLS 估计 FF | 4558 | 5253 | +695 |
| DSP | 10 | 11 | +1 |
| BRAM_18K | 4 | 4 | 0 |
| 循环 II | 全 1 | 全 1 | 零违规 |

> σ 是**零额外访存**换来的：Laplacian 复用主循环里已经在手上的那个 3×3
> 滑窗，只换一组系数。不新增行缓存、不新增 DDR 访问。这是它能只花 ~400 LUT
> 的原因（同样的窗口，换系数就是新算子）。

### mode 8（DIR）怎么看

输出把梯度方向量化成 4 档，直接映射成灰度方便目视：

| 方向 | 输出值 | 含义 |
|---|---|---|
| 0° | 0 | 梯度近水平 |
| 45° | 85 | 对角 |
| 90° | 170 | 梯度近竖直 |
| 135° | 255 | 反对角 |

量化用 `2*|gx| vs |gy|` 的**比较**代替 `tan(22.5°)=0.4142` 的除法 ——
只用移位和比较，不占乘法器。这是硬件做方向量化的标准做法。

### σ 怎么看（阶段 3 会打印）

```
gain=256 下各图 σ（σ 应当只跟图有关，跟算子无关）
  gradient  Σ=...  σ=...
  checker   Σ=...  σ=...
  ramp      Σ=...  σ=...
  noise     Σ=...  σ=71.44      ← 均匀随机噪声，理论 255/√12 = 73.6
  rb_split  Σ=...  σ=...
  flat      Σ=0    σ=0.00       ← 负例，必须严格为 0
```

`flat` 那行是**负例**：如果平坦图也测出非零 σ，说明累加器在吃
边界或者初值没清。`noise` 的 71.44 是量级锚点（Immerkær 对均匀噪声本就
低估约 3%，因为它假设噪声是高斯的）。

### 级联高斯怎么用

```python
proc = HardwareProcessor()
proc.gain_override = 256          # 验证时钉死增益
out5 = proc.run_gauss_cascade(frame, passes=2)   # 等效 5×5 二项式
```

**为什么不做成 PL 算子**：5×5 窗口会把边界语义从"最外一圈 0"改成
"最外两圈 0"，让全部既有算子的输出改变、v7 的 120 组回归作废；且 25 项
adder tree 会往关键路径加 2~3 ns。级联零 HLS 改动、零回归风险。
代价是多一趟 PL（+3.8 ms）—— **只在需要更强去噪时调，不是主链路。**

---

## 6. v7 相对 v6 改了什么（历史，保留备查）

| | v6 | **v7** |
|---|---|---|
| 寄存器 | width/height/mode | **+ gain @ `0x28`** |
| PL 每像素 | `out = in` | `out = clamp((in*gain)>>8)` |
| 闭环 | 无 | **μ 在 ARM 算（每帧 1 次），增益在 PL 做（每像素 307200 次）** |
| 默认 `gain=256` | — | **与 v6 逐字节一致**（`(x*256)>>8 == x`） |

AGC 参数（在 `hw_driver.py` 顶部，可现场调）：

| 常量 | 值 | 含义 |
|---|---|---|
| `AGC_TARGET_MU` | 110.0 | 目标平均亮度 |
| `AGC_MIN / AGC_MAX` | 1.0 / 4.0 | 增益上下限 |
| `AGC_SMOOTH` | 0.3 | 一阶平滑权重，防增益跳变闪烁 |

关掉 AGC（退回 v6 行为）：`proc.agc_enabled = False`。

**为什么这么划**：μ 每帧只算一次、是标量，而且灰度图本来就在 ARM 手里；
增益是每像素一次乘加。**划分依据是数据率。**
μ 测的是增益**之前**的输入，所以是**前馈**，不存在"越提越亮"的自激振荡。

---

## 7. 文件清单（22 个，不含本 README）

| 文件 | 字节 | MD5 前 8 | 说明 |
|---|---:|---|---|
| `rgb2gray.bit` | 4045771 | `4f5ee693` | **v8 位流**（WNS +0.955 ns） |
| `rgb2gray.hwh` | 265243 | `892e2305` | **v8 overlay 描述**（含 4 处 ap_return） |
| `hw_driver.py` | 28533 | `e0b02e3a` | PL 驱动 + AGC + **σ 回读** + **级联高斯** + MD5 锁 |
| `filters.py` | 7384 | `656295e1` | 黄金参考，9 模式 + `immerkaer_sigma_sum` + `gauss3_gray` |
| `validate_v8.py` | 17288 | `0101335e` | **v8 对拍**（像素 + σ + 级联 + 哨兵，5 阶段） |
| `live_view_raw.py` | 67506 | `511832e5` | 主程序（四线程） |
| `gesture.py` | 36833 | `dd9af653` | 手势识别（严格版） |
| `swipe.py` | 5916 | `2ee08c60` | 挥手判定 |
| `session.py` | 10252 | `38ed9b24` | 会话 + 相机（MJPG fourcc，见第 3 节） |
| `control.py` | 3391 | `845b0885` | 模式锁存 |
| `check_kit.py` | 4868 | `c3a7c625` | 版本体检（**10 项，含位流**，带自愈提示） |
| `bench_e2e_v6.py` | 9400 | — | 端到端拆解（v5/v6 A/B） |
| `diag_fps.py` | 4186 | — | 帧率分层诊断（S1 采集 / S2 PL / S3 AGC / S4 渐进） |
| `diag_read.py` | 5423 | `f1cbc840` | **采集层拆解**：定位 cv2.flip / numpy 镜像谁快 |
| `probe_camera.py` | 4689 | — | C270 能力探测（fourcc / 分辨率 / 曝光 / 缓冲） |
| `probe_read.py` | 6536 | `e55e974b` | **采集上限再探测**：用 POS_MSEC 区分相机产得慢 vs 我们读得慢 |
| `sigma.py` | 10783 | — | **ARM 侧** Immerkær σ + 标定（v8 起有 PL 版，见第 8 节） |
| `_smoke_calib.py` | 1307 | — | 校准自检 |
| `_smoke_dir.py` | 3158 | — | 挥手方向自检 |
| `_smoke_page.py` | 728 | — | 页面自检 |
| `_smoke_swipe.py` | 5093 | — | 挥手自检 |
| `_smoke_track.py` | 7024 | — | 跟踪自检 |

完整 MD5 存于 `D:\vivado\deliver\rgb2gray_overlay_v8\`（位流原厂目录）。

> `.bit` 大小 **4045771 B，与 v5/v6/v7 完全相同** —— 只能靠 MD5 区分版本。
> 10-19 曾误载 v4 位流，mode≥3 全输出 EMBOSS，排查一整晚；每次上传后要核一次 MD5。

---

## 8. σ 噪声估计（v8 起有 PL 版）

配套设计文档：`..\18_噪声感知闭环_1026.md`、`..\25_本周末收尾计划_1028.md`

### v8 的软硬划分（与 AGC 同一套理由）

| 环节 | 每帧次数 | 放哪 |
|---|---:|---|
| `Σ\|I⊛N\|` 逐像素累加 | **307200** | **PL**（零额外访存：复用已有 3×3 滑窗） |
| `√(π/2) · Σ / (6(W-2)(H-2))` | **1** 次标量 | **ARM**（驱动 `filters.sigma_from_sum`） |

> **划分依据是数据率。**

σ 从 PL 拿：`proc.last_sigma`（灰度级）、`proc.last_sigma_sum`（原始整数）、
`proc.sigma_valid`（`op==0` 时为 False —— 逐像素算子没有 3×3 窗口）。

配套设计文档：`..\18_噪声感知闭环_1026.md`、`..\25_本周末收尾计划_1028.md`

### 公式（Immerkær 1996，四个独立来源核准）

```
N = [[ 1, -2,  1],        σₙ = √(π/2) · 1/(6·(W-2)(H-2)) · Σ |I ⊛ N|
     [-2,  4, -2],
     [ 1, -2,  1]]
```

- `√(π/2)`：用平均绝对偏差代替方差（高斯下 E|X| = σ√(2/π)）
- 分母的 `6` = √(ΣN²) = √36 —— 对纯噪声 `E[(I⊛N)²] = σ²·ΣN²`
- **N 可分离：`N = [1,-2,1] ⊗ [1,-2,1]`** → 两次 3 抽头一维卷积，
  且和我们已有的 3×3 滑窗是**同一个窗口只换系数**，搬 PL 时不新增行缓存

### 已验证（PC 合成数据）

| 检验 | 结果 |
|---|---|
| 可分离 vs 直接 3×3 卷积 | **最大差 0.0** |
| 纯高斯噪声 σ=2/5/10/20 | 估计 2.02 / 5.00 / 10.00 / 20.00（误差 ≤1.1%） |
| 平滑图 + 噪声 σ=0/5/15 | 0.000 / 4.95 / 14.45（高 σ 略低估＝uint8 截断，**要写进报告**） |
| 时域 std ground truth（真值 6.0） | 5.85 |

### 板端跑法（约 30 分钟拿完创新证据）

```bash
python3 -u sigma.py --one                      # 单帧，确认能跑
python3 -u sigma.py --calib --frames 30        # 盖上镜头盖别动：σ_est vs σ_gt 标定
python3 -u sigma.py --sweep --steps 6          # 每档回车前改光照，得扫描曲线
python3 -u sigma.py --params                   # σ -> 双阈值表
```

**产出三张表**：
1. 标定表：`σ_gt` vs `σ_est`，目标相对误差 **<15%**
2. 照度扫描表：照度掉下去时 `mu↓ / σ↑`，而阈值跟着 σ 走
3. 参数表：`high = 3.0σ`，`low = 0.4·high`

> 注意：C270 输出 MJPEG，压缩伪影不是纯高斯，Immerkær 会有系统性偏差。
> **偏差本身要测出来并写进报告** —— 如实记录即可，不算缺陷。

### 为什么先做 numpy 版

`18_噪声感知闭环` 第八节自己定的 P0 = σ 估计(200 LUT) + 参数生成(150 LUT)
= **350 LUT，约 80% 收益**，且「P0 的 numpy 版一天能写完，先有数据再决定
要不要搬 PL」。σ 估计是算法，**不用等 Vivado** —— 先把创新证据拿到手，
搬不搬 PL 看余量。

搬 PL 时：只加 `abs` + 累加器 + 一个可读寄存器（`0x2C`），**BD 不用改**。

---

## 9. 回滚

出任何问题，**整目录切回上一个版本，不要在新目录里混文件**：

```bash
cd /home/xilinx/jupyter_notebooks/rgb2gray_v82
python3 live_view_raw.py --hw rgb2gray.bit --port 8080
```

### 位流 MD5 速查（各版 `.bit` 都是 4045771 B，只能靠这个区分）

| 版本 | `.bit` 前 8 | `.hwh` 前 8 | 特征 |
|---|---|---|---|
| **v8** | `4f5ee693` | `892e2305` | `.hwh` 里 grep 到 **4 次 `ap_return`** |
| v7 | `508a682f` | `b693c045` | `.hwh` 里 `ap_return` **0 次** |
| v6 | `f7760df3` | `e8933626` | 1 B/px 灰度输入，无 AGC |
| v5 | `126f1fc6` | `2cf60392` | 4 B/px RGB 打包，灰度在 PL 内算 |

### Windows 侧完整备份

- `上传_v8_1029\`（v8 全套，当前）
- `上传_v7_1028\`（v7 全套 + `validate_v7.py`）
- `D:\vivado\ip_repo\rgb2gray_v7_bak_1028\`（**v7 的 HLS IP，重建位流用**）
- `D:\vivado\deliver\rgb2gray_overlay_v7\`、`..._v8\`（位流原厂目录）

### 重建位流（改了 HLS 源码之后）

```powershell
D:\AMDDesignTools\2026.1\Vivado\bin\vivado.bat -mode batch -source D:\vivado\build_v8.tcl
```

脚本会：刷新 IP 仓库 → 升级被锁定的 `design_1_rgb2gray_0_1` → 重新生成 BD →
综合 → 实现 → 写位流 → 拷到 `D:\vivado\deliver\rgb2gray_overlay_v8\`。

`reset_run synth_1` 那一步**不能删**：不做 reset 的话 Vivado 可能复用缓存的
旧网表，于是"IP 换了但位流没变"—— 这类问题很难查。

---

## 10. 已知事项

- **板端 RTC 不准**，`ls -la` 显示的时间是假的，别当异常
- 文件若由 root 上传，属主是 root，jupyter/xilinx 可能读不了 → `chown -R xilinx:xilinx`
- 手势四步防回退：**MASK ON → 手掌盖满黄框 → RECALIBRATE → 掩膜只手白墙黑**
- 网口带宽红线仍在：CLIENT 低帧率 ≠ 系统慢，报告要标注
