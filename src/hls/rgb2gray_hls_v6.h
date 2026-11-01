#ifndef RGB2GRAY_HLS_H
#define RGB2GRAY_HLS_H

// 支持的最大图像尺寸（VGA），超出会越界
#define MAX_W 640
#define MAX_H 480
#define MAX_PIXELS (MAX_W * MAX_H)

// ---------------------------------------------------------------- 工作模式
// v5：在 v4 的 0/1/2 基础上扩到 8 个算子。
// 全部复用同一套「行缓存 + 3x3 滑窗」，新增一个算子只是在第 4) 步多加一个分支，
// 行缓存、滑窗、边界语义、接口契约一律不动 —— 这就是本设计要证明的可复用性。
//
// 接口不变：mode 仍是 s_axilite 上的一个 32 位标量，扩模式不需要改 Block Design，
//   也不需要改驱动。v3 -> v4 -> v5 三次硬件扩展，软件与 BD 始终零改动。
#define MODE_GRAY    0   // 灰度化          Y = (77R+150G+29B)>>8
#define MODE_SOBEL   1   // Sobel 边缘      3x3 梯度，|gx|+|gy|
#define MODE_EMBOSS  2   // 浮雕            右下 - 左上 + 128
#define MODE_GAUSS   3   // 高斯模糊        [1 2 1; 2 4 2; 1 2 1] / 16
#define MODE_SHARPEN 4   // 锐化            [0 -1 0; -1 5 -1; 0 -1 0]
#define MODE_LAPLACE 5   // 拉普拉斯边缘    [-1 -1 -1; -1 8 -1; -1 -1 -1]
#define MODE_BINARY  6   // 二值化          中心像素 >= 128 ? 255 : 0
#define MODE_MEAN    7   // 均值模糊        3x3 全 1 / 9

// 边界语义（对所有 3x3 算子一致，已写入接口契约）：
//   最外一圈（第 0 行 / 最后一行 / 第 0 列 / 最后一列）输出恒为 0，
//   因为那里凑不出完整的 3x3 窗口。mode 0 是逐像素算子，不受此约束。
//
// 顶层函数：
//   in     : 输入图，每像素 1 字节灰度（ARM 侧已做 cvtColor）
//            v6 起不再是 32 位打包字 —— 每像素字节数 5 -> 2
//   out    : 输出图，每像素 1 字节
//   width  : 图像宽度（≤ MAX_W）
//   height : 图像高度（≤ MAX_H）
//   mode   : 见上面的 MODE_* 定义
void rgb2gray(unsigned char *in, unsigned char *out,
              int width, int height, int mode);

#endif
