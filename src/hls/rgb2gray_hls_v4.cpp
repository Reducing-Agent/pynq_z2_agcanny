#include "rgb2gray_hls.h"

//
// 图像加速核 v4：行缓存(line buffer) + 3x3 滑窗
//
// v3 的问题：模式 1/2 每个像素要向 DDR 发 8 次读请求（凑 3x3 邻域），
//            640x480 一帧就是 240 万次访存 -> 实测 108 ms，浮雕甚至比 ARM 还慢。
// v4 的改法：用两块片上行缓存把"上一行 / 当前行"留在芯片里，
//            每个像素**只从 DDR 读 1 次**，3x3 邻域全部由片上滑窗寄存器拼出来。
//
// 接口与 v3 **完全一致**（m_axi gmem0/gmem1 + s_axilite control / control_r），
// 所以 Vivado 里 BD 不用重连，只需刷新 IP 重新生成 bitstream。
//
// 预期：访存量降到 1/8，边缘模式 108 ms -> 约 3~6 ms（II=1，每像素 1 拍）
//

// 32 位打包字 -> 灰度值：Y = (77R + 150G + 29B) >> 8
static unsigned char to_gray(unsigned int pix) {
#pragma HLS INLINE
    unsigned char r = (unsigned char)(pix         & 0xFF);
    unsigned char g = (unsigned char)((pix >> 8)  & 0xFF);
    unsigned char b = (unsigned char)((pix >> 16) & 0xFF);
    return (unsigned char)((77 * r + 150 * g + 29 * b) >> 8);
}

// 限幅到 0~255
static unsigned char clamp8(int v) {
#pragma HLS INLINE
    if (v < 0)   return (unsigned char)0;
    if (v > 255) return (unsigned char)255;
    return (unsigned char)v;
}

void rgb2gray(unsigned int *in, unsigned char *out,
              int width, int height, int mode) {
#pragma HLS INTERFACE m_axi     port=in   offset=slave bundle=gmem0 depth=MAX_PIXELS
#pragma HLS INTERFACE m_axi     port=out  offset=slave bundle=gmem1 depth=MAX_PIXELS
#pragma HLS INTERFACE s_axilite port=width  bundle=control
#pragma HLS INTERFACE s_axilite port=height bundle=control
#pragma HLS INTERFACE s_axilite port=mode   bundle=control
#pragma HLS INTERFACE s_axilite port=return bundle=control

    int n = width * height;

    // ---------------- 模式 0：灰度化（v3 已经很快，保持不变） ----------------
    if (mode == MODE_GRAY) {
        for (int i = 0; i < n; i++) {
#pragma HLS PIPELINE II=1
            out[i] = to_gray(in[i]);
        }
        return;
    }

    // ---------------- 模式 1/2：行缓存 + 滑窗 ----------------

    // 尺寸太小就没法做 3x3，直接全 0 兜底（与 v3 的"边界置 0"语义一致）
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
        rowA[x] = to_gray(in[x]);                  // 第 0 行
        rowB[x] = to_gray(in[width + x]);          // 第 1 行
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
            unsigned char c = to_gray(in[base_in + x]);

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
                unsigned char p10 = w1[0],             p12 = w1[2];
                unsigned char p20 = w2[0], p21 = w2[1], p22 = w2[2];

                if (mode == MODE_SOBEL) {
                    int gx = (p02 + 2 * p12 + p22) - (p00 + 2 * p10 + p20);
                    int gy = (p20 + 2 * p21 + p22) - (p00 + 2 * p01 + p02);
                    int mag = (gx < 0 ? -gx : gx) + (gy < 0 ? -gy : gy);
                    out[base_out + (x - 1)] = clamp8(mag);
                } else {
                    out[base_out + (x - 1)] = clamp8(p22 - p00 + 128);
                }
            }
        }

        // 每行首尾两列置 0（与 v3 的边界语义一致）
        out[base_out] = 0;
        out[base_out + width - 1] = 0;
    }

    // 最后一行：整行输出 0
    for (int x = 0; x < width; x++) {
#pragma HLS PIPELINE II=1
        out[(height - 1) * width + x] = 0;
    }
}
