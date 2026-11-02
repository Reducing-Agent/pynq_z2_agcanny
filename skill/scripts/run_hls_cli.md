# 命令行跑 Vitis HLS（不用 GUI）

摸通这套流程的价值：**省掉一次 GUI 点选**，而且能被脚本调用，
配合 `build_hls.tcl` 就做到"改代码 → 出位流"全程可复现（可复现性 15 分的证据）。

---

## 目录约定

```
D:\vitis\RGB2gray\          <- 工程根
    RGB2gray.cfg            <- 配置文件（源文件、top 函数、器件、时钟）
    rgb2gray\               <- work_dir，所有产物都在这
```

`RGB2gray.cfg` 关键内容：

```
part=xc7z020clg400-1
hls.clock=10000000          # 10 ns = 100 MHz
hls.top=rgb2gray
hls.file=rgb2gray_hls_v8.cpp
hls.file=rgb2gray_tb.cpp    # 仅 csim/cosim 需要
```

---

## 三步命令（PowerShell）

```powershell
$VITIS = 'D:\AMDDesignTools\2026.1\Vitis\bin'

# 1) compile —— C 综合（约 1.5 min）
& "$VITIS\v++.bat" -c --mode hls --config D:\vitis\RGB2gray\RGB2gray.cfg --work_dir rgb2gray

# 2) impl —— RTL 综合 + 布局布线，出 IP（约 6 min）
& "$VITIS\vitis-run.bat" --mode hls --impl --config D:\vitis\RGB2gray\RGB2gray.cfg --work_dir rgb2gray

# 3) package —— 打包成 Vivado IP（约 20 s）
& "$VITIS\vitis-run.bat" --mode hls --package --config D:\vitis\RGB2gray\RGB2gray.cfg --work_dir rgb2gray
```

产物：`rgb2gray\rgb2gray.zip`（这就是装进 `ip_repo` 的 IP）

---

## 四个常见错误

### 1. 必须用 `bin\*.bat`，不能用 `bin\unwrapped\win64.o\*.exe`

直接跑 exe 会报：

```
api-ms-win-crt-math-l1-1-0.dll 缺失
```

`unwrapped` 下的 exe 没有配好 CRT DLL 的搜索路径；`bin\*.bat` 是包装器，会先设环境。

### 2. Bash / PowerShell 都不能调 `cmd.exe`

本机安全策略拦截。不要写 `cmd.exe /c xxx.bat`，
PowerShell 里直接 `& 'D:\...\v++.bat' args...` 就行。

### 3. 没有 `--compile` 这个参数

compile 走 `v++ -c`；`vitis-run.bat --mode hls` 只接受
`--csim --cosim --impl --package`。

### 4. cosim 会跑几小时

用 `-D__HLS_COSIM__`（Vitis 编译时自动加的宏）在测试台里把用例裁掉，
本项目从 864 例裁到 96 例。**别靠 CLI 参数裁，测试台里裁最可靠。**

---

## 跑完必查的三件事

1. **`csynth` 报告里的 `M_AXI Burst Information`** —— 主循环必须是 `Inferred`。
   本项目 20 Inferred / 1 Fail 属正常（那条 Fail 是同一访问在外层循环上的冗余条目）。
2. **主循环 II** —— 应该 = 1。II=2 意味着吞吐直接减半（见案例 14）。
3. **生成的 `xrgb2gray_hw.h` 里 `ADDR_*_DATA`** ——
   寄存器偏移会随接口变化而整体移动（见案例 15），
   **驱动里永远不要硬编码偏移**，每次构建后 grep 一遍。

---

## 完整链路（改一次代码的完整闭环）

```
改 .cpp
  → v++ -c                         (1.5 min)
  → --impl                         (6 min)
  → --package                      (20 s)   → rgb2gray.zip
  → 装进 ip_repo，跑 build_hls.tcl (约 10 min) → .bit + .hwh
  → update_bitstream.py            (同步三处 MD5)
  → 上板 check_kit.py → validate → diag_slow
```
