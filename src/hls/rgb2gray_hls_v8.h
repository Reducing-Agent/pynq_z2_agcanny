#ifndef RGB2GRAY_HLS_V8_H
#define RGB2GRAY_HLS_V8_H

#define MAX_W 640
#define MAX_H 480
#define MAX_PIXELS (MAX_W * MAX_H)

// mode 编码：bit 0..3 = op，bit 4 = FLAG_AGC，bit 5 = FLAG_SIGMA。
// AGC 与 SIGMA 两位只给驱动自己记状态，PL 永远按 gain 寄存器执行。
#define MODE_GRAY    0   // 直通（灰度化在 ARM 侧做）
#define MODE_SOBEL   1   // Sobel 边缘      |gx|+|gy|
#define MODE_EMBOSS  2   // 浮雕            右下 - 左上 + 128
#define MODE_GAUSS   3   // 3x3 高斯        [1 2 1;2 4 2;1 2 1]/16
#define MODE_SHARPEN 4   // 锐化            [0 -1 0;-1 5 -1;0 -1 0]
#define MODE_LAPLACE 5   // 拉普拉斯        [-1 -1 -1;-1 8 -1;-1 -1 -1]
#define MODE_BINARY  6   // 二值化          中心 >= 128 ? 255 : 0
#define MODE_MEAN    7   // 均值模糊        3x3 全 1 / 9
#define MODE_DIR     8   // 梯度方向量化    0/1/2/3 -> 0/85/170/255

// op 9/10/11 是同一条四级流水线的三个抽头，不是三个并列算子：
//   级1 AGC + 5x5 可分离高斯 → 级2 Sobel + 方向量化 → 级3 NMS → 级4 双阈值 + 滞后
//   抽头 A (op 9)  = 级1 输出，5x5 二项式高斯去噪图
//   抽头 B (op 11) = 级3 输出，NMS 细化后的幅值
//   抽头 C (op 10) = 级4 输出，0/255 边缘，完整的 Canny
// 四级硬件全部在跑，op 只决定从哪一级把结果写回 DDR。
#define MODE_GAUSS5  9
#define MODE_CANNY  10
#define MODE_NMS    11

#define MODE_MAX_IMPL MODE_NMS   // 测试台的循环上界、驱动的参数校验都用它，加算子时同步改

#define FLAG_AGC   16    // bit 4
#define FLAG_SIGMA 32    // bit 5

#define GAIN_UNITY 256   // Q8.8 的 1.0x

// thresh = (HIGH << 8) | LOW，由 ARM 每帧按实测噪声改写（HIGH:LOW = 2:1）。
#define CANNY_THRESH_DEFAULT  ((60 << 8) | 30)
#define THRESH_PACK(hi, lo)   (((hi) << 8) | (lo))
#define THRESH_LOW(t)         ((t) & 0xFF)
#define THRESH_HIGH(t)        (((t) >> 8) & 0xFF)

// sigma 累加器的 bank 数，必须是 2 的幂。拆成多 bank 是为了打断跨迭代进位链。
#define SIGMA_BANKS 16
#define SIGMA_MASK  (SIGMA_BANKS - 1)

// 边界语义：op 1..8 最外一圈恒 0，op 9..11 最外四圈恒 0（四级各延迟 2 行 2 列）。
// 有效输出区 行 [4,H-5] × 列 [4,W-5]，所以最小 9x9，更小的图全 0 输出。

// 返回值：Σ |I ⊛ N|，N = [[1,-2,1],[-2,4,-2],[1,-2,1]]，仅 op 1..8 有效，其余返回 0。
// ARM 侧每帧算一次 sigma = sqrt(pi/2) * 返回值 / (6*(W-2)*(H-2))。
// 窗口里存的是过 AGC 的灰度，还原传感器噪声要除回去：sigma_raw = sigma * 256 / gain。
// 走 return 而不是指针出参：s_axilite 的指针出参不会生成可寻址寄存器，ARM 读不到。
// return 会挤占 0x10，其余标量整体后移一格，驱动必须用生成的 xrgb2gray_hw.h 里的
// ADDR_*_DATA 宏，不能硬编码偏移。
int rgb2gray(unsigned char *in, unsigned char *out,
             int width, int height, int mode, int gain, int thresh);

#endif
