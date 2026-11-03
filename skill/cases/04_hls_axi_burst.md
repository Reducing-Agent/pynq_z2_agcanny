# AXI burst / m_axi pragma（性能主要瓶颈）

> 案例 10 把本项目从 20.57 ms 修到 3.258 ms（**6.31 倍**）；
> 案例 16 是"HLS 全绿但 Vivado 才炸"的典型。

---

## 案例 10 · 主循环慢 6.7 倍而综合报告说 II=1（含一次归因错误）

### 现象

v8.2 上板实测，op 1~11 全部 **20.57 ms（6.70 cyc/px）**，而 II=1 的理论地板是
3.072 ms。但 `csynth` 报告主循环 II=1，**全绿，没有任何警告**。

### 第一版归因（错的）

以为是 σ 单累加器 `sigma_sum += |lap|` 的**跨迭代依赖链**。
据此改成 16 bank 轮换 —— **耗时没有变化，仍是 20.57 ms**。

### 反例

op 9/10/11 走 `path5x5()`，**根本没有 σ 累加**，
实测却和 op 1~8 **一模一样慢**。唯一变量对不上，假设被证伪。

> 三样本对照
> （v7 无 σ = 3.577 ms / v8 有 σ = 20.57 ms / v8 的 GRAY 无 σ = 3.224 ms）
> 三样本互相印证，结论却是错的。

### 真因

两条慢通路唯一的共同点是 **DDR 写被包在 `if` 里**
（`if (x>=2)` / `if (y>=8 && x>=8)`）。

**条件写让 HLS 推不出 AXI burst**，退化成"每像素一次带握手的独立传输"。

而唯一快的 op 0 是**无条件写** `out[i] = agc(in[i])`，实测 3.222 ms（1.05 cyc/px）
——**已经跑满**。这反过来证明 AXI 通路本身没问题，缺的就是 burst。

### 修复

结果先落**片上输出行缓冲** `outr[2][MAX_W]`（按输出行奇偶乒乓），
DDR 写改成 `out[base + x] = outr[prev][x]` —— **无条件 + 地址连续**。

```c
// 改前（慢 6.7 倍）
if (x >= 2) out[base + (x - 1)] = result;

// 改后（无条件、地址连续）
outr[cur_bank][x - 1] = result;          // 落片上
out[base_out + x] = outr[prev_bank][x];  // 无条件写 DDR（上一行结果）
```

要点：

- 读和写仍在**同一个循环**里，不增加循环趟数 —— 与快的 op 0 同构
- 边界靠"**清零一次**"兜底：计算循环只写 `x-1`（`x>=2`），覆盖 `1..W-2`，
  碰不到列 0 / W-1，所以这两列在初始化时清一次 0 即可
- 收尾要**刷最后一行**（主循环是"写上一行"，最后一行必须补写）

**结果：20.57 → 3.258 ms，6.31 倍。代价：4 块 RAMB18。**

### 顺带否掉的一条错误优化

当时还考虑过"一次读 4 像素"加宽通道。
**否掉**：op 0 已经跑到 1.05 cyc/px，说明**数据通路已经跑满 1 字节/拍**，
缺的是 burst 而不是位宽。csynth 报告里满屏
`Could not widen since type i8 size >= max_widen_bitwidth threshold of 0`
也印证了加宽通道这条根本没开。

### 验收点

`csynth` 报告里的 **M_AXI Burst Information → 主循环必须是 `Inferred`**。
本项目是 **20 Inferred / 1 Fail**，那条 Fail 是同一访问在外层循环上的冗余条目，
属正常。

### 可复用结论（三条）

1. **`csynth` 的 II 是估计值**。"综合全绿但性能不对"必须上板实测 + 控制变量法。
2. **先找反例证伪自己的假设**。只做互相印证的对照实验容易得出错误结论；
   纠正结论的是"op 9~11 没有 σ 却一样慢"这个反例。
3. **m_axi 上的写要无条件、地址连续**。任何 `if` 包裹的写都要警惕；
   实在要条件化，就先落片上缓冲再无条件刷出去。

---

## 案例 16 · `num_*_outstanding` 设 64：HLS 全绿，Vivado 才炸

### 现象

为提升 AXI 并行度把 `num_read_outstanding` / `num_write_outstanding`
从 16 调到 64。**Vitis HLS 的 csim / csynth / cosim / impl / package 全部 PASS**，
直到 Vivado 跑 `generate_target` 才报：

```
ERROR: [BD 41-241] Validation failed for parameter 'My S00_READ_ACCEPTANCE'.
       Value '64' is out of the range (1,32)
```

### 根因

该 pragma 的值会**传播到 Block Design 里 AXI Interconnect 的
`S00_READ_ACCEPTANCE` / `S01_WRITE_ACCEPTANCE`**，
而 Zynq-7000 上这个 IP 的合法范围是 **(1,32)**。
**HLS 侧不校验取值范围**，错误要等 Vivado 才暴露。

### 修复

改回 16（已验证可用的值）。32 是理论上限，本项目没试。

### 可复用结论（两条）

1. Zynq-7000 的 GP/HP 口是 **AXI3**：
   `max_*_burst_length` 上限 **16**（写 256 也只会被拆成若干个 16-beat），
   `num_*_outstanding` 上限 **32**。
2. **HLS 通过 ≠ 能进 Vivado**。凡是改了 `m_axi` 相关 pragma，
   必须一路跑到 Vivado `generate_target` 验证；
   只跑到 `package` 会得到表面上成功的结果。

---

## m_axi pragma 速查（本项目现行值）

```c
#pragma HLS INTERFACE m_axi port=in  offset=slave bundle=gmem0 \
    depth=MAX_PIXELS max_read_burst_length=16  num_read_outstanding=16
#pragma HLS INTERFACE m_axi port=out offset=slave bundle=gmem1 \
    depth=MAX_PIXELS max_write_burst_length=16 num_write_outstanding=16
```

`bundle` 分成 gmem0 / gmem1 两条独立总线，读写互不阻塞。
