#include "rgb2gray_hls_v6.h"

//
// 图像加速核 v5：行缓存(line buffer) + 3x3 滑窗，8 个算子
//
// v3  -> v4：把"每像素 9 次访存"改成行缓存 + 滑窗，边缘 108 ms -> 3.78 ms（28.5x）
// v4  -> v5：在**同一套行缓存滑窗**上挂 8 个算子，接口与边界语义一个字节没改。
//
// v5 想证明的不是"我们会写滤波器"，而是：
//   这个滑窗结构是一个**可复用的算子模板** —— 新增一个 3x3 算子，
//   只需要在这里多加一个分支，行缓存、滑窗滚动、边界置零、接口契约全部不动。
//   加算子不重新连 BD、不改驱动、不重新定义寄存器，这是设计出来的，不是碰巧的。
//
// 8 个算子共用的中间量（都由同一个 3x3 窗口现算，不额外访存）：
//   sum9   = 9 个像素之和                -> 均值模糊
//   cross4 = 上下左右四邻之和            -> 锐化
//   p11    = 中心像素                    -> 高斯权重、拉普拉斯、二值化
//   高斯加权和 = sum9 + cross4 + 3*p11   （中心 4、四邻 2、四角 1）
//
// 关于除法（刻意不占用 DSP）：
//   /16 = >>4            —— 精确，零代价
//   /9  = (x*1821)>>14   —— 已用穷举验证：对 0..2295（9*255）全部等于 floor(x/9)，
//                           1821 只有 11 bit，综合成移位加法即可，不需要 DSP
//

// v6：灰度化已移到 ARM 侧（cv2.cvtColor），PL 直接收 1 B/px 灰度，
//     不再有 to_gray()。这样每像素字节数从 5 降到 2，32 位口的带宽不再是枷锁。

// 限幅到 0~255
static unsigned char clamp8(int v) {
#pragma HLS INLINE
    if (v < 0)   return (unsigned char)0;
    if (v > 255) return (unsigned char)255;
    return (unsigned char)v;
}

void rgb2gray(unsigned char *in, unsigned char *out,
              int width, int height, int mode) {
// v6：显式约束突发。原来没写，HLS 可能退化成一次一个字的零散读。
//     加 burst=16 之后一次搬 16 个字，不花 LUT。
#pragma HLS INTERFACE m_axi     port=in   offset=slave bundle=gmem0 depth=MAX_PIXELS max_read_burst_length=16 num_read_outstanding=16
#pragma HLS INTERFACE m_axi     port=out  offset=slave bundle=gmem1 depth=MAX_PIXELS max_write_burst_length=16 num_write_outstanding=16
#pragma HLS INTERFACE s_axilite port=width  bundle=control
#pragma HLS INTERFACE s_axilite port=height bundle=control
#pragma HLS INTERFACE s_axilite port=mode   bundle=control
#pragma HLS INTERFACE s_axilite port=return bundle=control

    int n = width * height;

    // ---------------- 模式 0：灰度化（v3 已经很快，保持不变） ----------------
    // 逐像素算子，不需要邻域，所以最外一圈也正常输出（不受"边界恒 0"约束）
    if (mode == MODE_GRAY) {
        for (int i = 0; i < n; i++) {
#pragma HLS PIPELINE II=1
            out[i] = in[i];
        }
        return;
    }

    // ---------------- 模式 1~7：行缓存 + 滑窗 ----------------

    // 尺寸太小就没法做 3x3，直接全 0 兜底（与 v3/v4 的"边界置 0"语义一致）
    if (width < 3 || height < 3) {
        for (int i = 0; i < n; i++) {
#pragma HLS PIPELINE II=1
            out[i] = 0;
        }
        return;
    }

    // 两块片上行缓存：rowA = 第 y-2 行，rowB = 第 y-1 行（各 MAX_W 字节）
    static unsigned char rowA[MAX_W];
    static unsigned char rowB[MAX_W];

    // 3x3 滑窗寄存器（完全分区 -> 每周期可同时取 9 个值）
    unsigned char w0[3], w1[3], w2[3];
#pragma HLS ARRAY_PARTITION variable=w0 complete
#pragma HLS ARRAY_PARTITION variable=w1 complete
#pragma HLS ARRAY_PARTITION variable=w2 complete

    // 第 0 行：整行输出 0
    for (int x = 0; x < width; x++) {
#pragma HLS PIPELINE II=1
        out[x] = 0;
    }

    // 预装第 0 行、第 1 行的灰度到片上缓存
    for (int x = 0; x < width; x++) {
#pragma HLS PIPELINE II=1
        rowA[x] = in[x];                           // 第 0 行
        rowB[x] = in[width + x];                   // 第 1 行
    }

    // 主循环：从 DDR 流式读入第 y 行，同时算完并写回第 y-1 行
    for (int y = 2; y < height; y++) {
        int base_in  = y * width;                  // 本轮要读入的行
        int base_out = (y - 1) * width;            // 本轮要写回的行

        for (int x = 0; x < width; x++) {
#pragma HLS PIPELINE II=1
            // 1) 取上一上一行、上一行的灰度（片上），并从 DDR 读 1 个新像素
            unsigned char a = rowA[x];
            unsigned char b = rowB[x];
            unsigned char c = in[base_in + x];

            // 2) 行缓存滚动：rowA <- 旧 rowB，rowB <- 刚读入的当前行
            //    （本行 rowA[x]/rowB[x] 已经用完，可以就地覆盖）
            rowA[x] = b;
            rowB[x] = c;

            // 3) 滑窗左移一位，把新列推进去
            w0[0] = w0[1]; w0[1] = w0[2]; w0[2] = a;
            w1[0] = w1[1]; w1[1] = w1[2]; w1[2] = b;
            w2[0] = w2[1]; w2[1] = w2[2]; w2[2] = c;

            // 4) 窗口填满（x >= 2）后，中心列 = x-1，开始算
            //    窗口三列分别对应 x-2 / x-1 / x
            if (x >= 2) {
                unsigned char p00 = w0[0], p01 = w0[1], p02 = w0[2];
                unsigned char p10 = w1[0], p11 = w1[1], p12 = w1[2];
                unsigned char p20 = w2[0], p21 = w2[1], p22 = w2[2];

                // 8 个算子共用的中间量
                int cross4 = (int)p01 + (int)p10 + (int)p12 + (int)p21;
                int sum9   = cross4 + (int)p00 + (int)p02
                                    + (int)p20 + (int)p22 + (int)p11;

                if (mode == MODE_SOBEL) {
                    int gx = (p02 + 2 * p12 + p22) - (p00 + 2 * p10 + p20);
                    int gy = (p20 + 2 * p21 + p22) - (p00 + 2 * p01 + p02);
                    int mag = (gx < 0 ? -gx : gx) + (gy < 0 ? -gy : gy);
                    out[base_out + (x - 1)] = clamp8(mag);

                } else if (mode == MODE_EMBOSS) {
                    out[base_out + (x - 1)] = clamp8((int)p22 - (int)p00 + 128);

                } else if (mode == MODE_GAUSS) {
                    // 中心 4 / 四邻 2 / 四角 1，除以 16（>>4 精确）
                    int gw = sum9 + cross4 + 3 * (int)p11;
                    out[base_out + (x - 1)] = (unsigned char)(gw >> 4);

                } else if (mode == MODE_SHARPEN) {
                    // [0 -1 0; -1 5 -1; 0 -1 0]
                    out[base_out + (x - 1)] = clamp8(5 * (int)p11 - cross4);

                } else if (mode == MODE_LAPLACE) {
                    // [-1 -1 -1; -1 8 -1; -1 -1 -1] = 9*中心 - 九宫格和
                    out[base_out + (x - 1)] = clamp8(9 * (int)p11 - sum9);

                } else if (mode == MODE_BINARY) {
                    out[base_out + (x - 1)] = (p11 >= 128) ? 255 : 0;

                } else {   // MODE_MEAN
                    // /9 用 (x*1821)>>14 精确实现，穷举 0..2295 已验证，不占 DSP
                    out[base_out + (x - 1)] = (unsigned char)((sum9 * 1821) >> 14);
                }
            }
        }

        // 每行首尾两列置 0（与 v3/v4 的边界语义一致）
        out[base_out] = 0;
        out[base_out + width - 1] = 0;
    }

    // 最后一行：整行输出 0
    for (int x = 0; x < width; x++) {
#pragma HLS PIPELINE II=1
        out[(height - 1) * width + x] = 0;
    }
}
