#ifndef RGB2GRAY_HLS_H
#define RGB2GRAY_HLS_H

// 支持的最大图像尺寸（VGA），超出会越界
#define MAX_W 640
#define MAX_H 480
#define MAX_PIXELS (MAX_W * MAX_H)

// ---------------------------------------------------------------- 工作模式
// 8 个算子全部复用同一套「行缓存 + 3x3 滑窗」，新增一个算子只是在主循环里
// 多加一个分支，行缓存、滑窗、边界语义、接口契约一律不动。
#define MODE_GRAY    0   // 直通（v6 起灰度化在 ARM 侧做）
#define MODE_SOBEL   1   // Sobel 边缘      3x3 梯度，|gx|+|gy|
#define MODE_EMBOSS  2   // 浮雕            右下 - 左上 + 128
#define MODE_GAUSS   3   // 高斯模糊        [1 2 1; 2 4 2; 1 2 1] / 16
#define MODE_SHARPEN 4   // 锐化            [0 -1 0; -1 5 -1; 0 -1 0]
#define MODE_LAPLACE 5   // 拉普拉斯边缘    [-1 -1 -1; -1 8 -1; -1 -1 -1]
#define MODE_BINARY  6   // 二值化          中心像素 >= 128 ? 255 : 0
#define MODE_MEAN    7   // 均值模糊        3x3 全 1 / 9

// ---------------------------------------------------------------- AGC（v7 新增）
// gain 是 Q8.8 定点增益：gain = 256 表示 1.0x，512 表示 2.0x，1024 表示 4.0x。
//   y = clamp((x * gain) >> 8, 0, 255)
// gain=256 时 (x*256)>>8 == x，严格直通 —— 所以 v7 在默认增益下与 v6 逐字节一致。
//
// 闭环划分（重要，答辩要讲）：
//   PL  ：每像素做一次乘加（每帧 307200 次）—— 这才是 FPGA 该干的事
//   ARM ：每帧算一次均值 mu = gray.mean()，再由 mu 推出 gain（每帧 1 次）
//   划分依据是**数据率**，不是"能在 PL 做就都放 PL"。
//   用第 N 帧的 mu 决定第 N+1 帧的 gain，天然带一帧延迟，不会自激振荡（等同相机 AE）。
#define GAIN_UNITY 256

// 边界语义（对所有 3x3 算子一致）：
//   最外一圈输出恒为 0，因为那里凑不出完整的 3x3 窗口。
//   mode 0 是逐像素算子，不受此约束。
//
// 顶层函数：
//   in     : 输入图，每像素 1 字节灰度（ARM 侧已做 cvtColor）
//   out    : 输出图，每像素 1 字节
//   width  : 图像宽度（≤ MAX_W）
//   height : 图像高度（≤ MAX_H）
//   mode   : 见上面的 MODE_* 定义
//   gain   : Q8.8 定点增益，256 = 1.0x（v7 新增，s_axilite 标量）
void rgb2gray(unsigned char *in, unsigned char *out,
              int width, int height, int mode, int gain);

#endif
