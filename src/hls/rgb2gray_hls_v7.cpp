#include "rgb2gray_hls_v7.h"

//
// 图像加速核：行缓存(line buffer) + 3x3 滑窗，8 个算子 + AGC 自动增益
//
// v3 -> v4：把"每像素 9 次访存"改成行缓存 + 滑窗
// v4 -> v5：在同一套行缓存滑窗上挂 8 个算子，接口与边界语义不变
// v5 -> v6：灰度化移到 ARM 侧（cv2.cvtColor），PL 收 1 B/px 灰度，每像素访存 5 B -> 2 B
// v6 -> v7：加入 AGC 自动增益（光照自适应）
//
// v7 的设计要点 —— 闭环放在哪里：
//   AGC 需要两个东西：一个统计量 mu（画面有多暗），一个动作（把像素乘亮）。
//     · mu 每帧只算一次，是标量，而且灰度图本来就在 ARM 手里（cvtColor 刚算完）
//       -> 放 ARM。为它专门在 PL 里加累加器和读回寄存器不划算。
//     · 增益是每像素一次乘加，一帧 307200 次
//       -> 放 PL。这才是 FPGA 该干的事。
//   所以这是**软硬协同闭环**：PL 出逐像素结果，ARM 出每帧参数。
//   划分依据是数据率，不是"能在 PL 做就全放 PL"。
//
// v6 实测背景（为什么不再为提速加东西）：
//   640x480 在 100 MHz、II=1 下的理论地板是 3.072 ms，实测核内 3.771 ms（II≈1.23），
//   已达地板的 82%。剩余空间只有 18%，所以 v7 的目标是**加功能、不加时序压力**。
//

// 限幅到 0~255
static unsigned char clamp8(int v) {
#pragma HLS INLINE
    if (v < 0)   return (unsigned char)0;
    if (v > 255) return (unsigned char)255;
    return (unsigned char)v;
}

// AGC：Q8.8 定点增益。x >= 0、gain >= 0，所以只需上限饱和。
// gain = 256 时 (x*256)>>8 == x，严格直通（v7 与 v6 的回归一致性靠这条保证）。
static unsigned char agc(unsigned char x, int gain) {
#pragma HLS INLINE
    int t = ((int)x * gain) >> 8;
    return (t > 255) ? (unsigned char)255 : (unsigned char)t;
}

void rgb2gray(unsigned char *in, unsigned char *out,
              int width, int height, int mode, int gain) {
#pragma HLS INTERFACE m_axi     port=in   offset=slave bundle=gmem0 depth=MAX_PIXELS max_read_burst_length=16 num_read_outstanding=16
#pragma HLS INTERFACE m_axi     port=out  offset=slave bundle=gmem1 depth=MAX_PIXELS max_write_burst_length=16 num_write_outstanding=16
#pragma HLS INTERFACE s_axilite port=width  bundle=control
#pragma HLS INTERFACE s_axilite port=height bundle=control
#pragma HLS INTERFACE s_axilite port=mode   bundle=control
#pragma HLS INTERFACE s_axilite port=gain   bundle=control
#pragma HLS INTERFACE s_axilite port=return bundle=control

    int n = width * height;

    // ---------------- 模式 0：逐像素直通 + AGC ----------------
    if (mode == MODE_GRAY) {
        for (int i = 0; i < n; i++) {
#pragma HLS PIPELINE II=1
            out[i] = agc(in[i], gain);
        }
        return;
    }

    // ---------------- 模式 1~7：AGC + 行缓存 + 滑窗 ----------------

    // 尺寸太小就没法做 3x3，直接全 0 兜底（边界语义不变）
    if (width < 3 || height < 3) {
        for (int i = 0; i < n; i++) {
#pragma HLS PIPELINE II=1
            out[i] = 0;
        }
        return;
    }

    // 两块片上行缓存：rowA = 第 y-2 行，rowB = 第 y-1 行（各 MAX_W 字节）
    // 存的是**已经过 AGC** 的灰度，保证窗口里 9 个值增益一致
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

    // 预装第 0 行、第 1 行（过 AGC）
    // 刻意拆成两个循环，而不是一个循环里同时算 rowA/rowB：
    //   合在一起时 HLS 要给一个循环配 2 个乘法器，于是把预装循环降到 II=2
    //   （csynth 里会报一行 "II | Resource Limitation"，很难看）。
    //   拆开后每个循环只用 1 个乘法器，都能 II=1。
    //   代价是循环头尾开销多一点，但预装只跑 640 拍，一帧只一次，可忽略。
    for (int x = 0; x < width; x++) {
#pragma HLS PIPELINE II=1
        rowA[x] = agc(in[x], gain);                     // 第 0 行
    }
    for (int x = 0; x < width; x++) {
#pragma HLS PIPELINE II=1
        rowB[x] = agc(in[width + x], gain);             // 第 1 行
    }

    // 主循环：从 DDR 流式读入第 y 行，同时算完并写回第 y-1 行
    for (int y = 2; y < height; y++) {
        int base_in  = y * width;                  // 本轮要读入的行
        int base_out = (y - 1) * width;            // 本轮要写回的行

        for (int x = 0; x < width; x++) {
#pragma HLS PIPELINE II=1
            // 1) 取片上缓存，并从 DDR 读 1 个新像素（过 AGC）
            unsigned char a = rowA[x];
            unsigned char b = rowB[x];
            unsigned char c = agc(in[base_in + x], gain);

            // 2) 行缓存滚动：rowA <- 旧 rowB，rowB <- 刚读入的当前行
            rowA[x] = b;
            rowB[x] = c;

            // 3) 滑窗左移一位，把新列推进去
            w0[0] = w0[1]; w0[1] = w0[2]; w0[2] = a;
            w1[0] = w1[1]; w1[1] = w1[2]; w1[2] = b;
            w2[0] = w2[1]; w2[1] = w2[2]; w2[2] = c;

            // 4) 窗口填满（x >= 2）后，中心列 = x-1，开始算
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
                    int gw = sum9 + cross4 + 3 * (int)p11;
                    out[base_out + (x - 1)] = (unsigned char)(gw >> 4);

                } else if (mode == MODE_SHARPEN) {
                    out[base_out + (x - 1)] = clamp8(5 * (int)p11 - cross4);

                } else if (mode == MODE_LAPLACE) {
                    out[base_out + (x - 1)] = clamp8(9 * (int)p11 - sum9);

                } else if (mode == MODE_BINARY) {
                    out[base_out + (x - 1)] = (p11 >= 128) ? 255 : 0;

                } else {   // MODE_MEAN
                    out[base_out + (x - 1)] = (unsigned char)((sum9 * 1821) >> 14);
                }
            }
        }

        // 每行首尾两列置 0（边界语义不变）
        out[base_out] = 0;
        out[base_out + width - 1] = 0;
    }

    // 最后一行：整行输出 0
    for (int x = 0; x < width; x++) {
#pragma HLS PIPELINE II=1
        out[(height - 1) * width + x] = 0;
    }
}
