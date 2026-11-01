#include "rgb2gray_hls_v8.h"

// 图像加速核：行缓存 + 滑窗
//   3x3 通路  ：8 个单算子 + AGC + Immerkær 噪声累加（op 0..8，彼此并列）
//   融合流水线：高斯 → Sobel → NMS → 双阈值+滞后，四级串联（op 9..11，见 .h）

// 限幅到 0~255
static unsigned char clamp8(int v) {
#pragma HLS INLINE
    if (v < 0)   return (unsigned char)0;
    if (v > 255) return (unsigned char)255;
    return (unsigned char)v;
}

// AGC：Q8.8 定点增益。gain = 256 时 (x*256)>>8 == x，严格直通
static unsigned char agc(unsigned char x, int gain) {
#pragma HLS INLINE
    int t = ((int)x * gain) >> 8;
    return (t > 255) ? (unsigned char)255 : (unsigned char)t;
}

static int absv(int v) {
#pragma HLS INLINE
    return (v < 0) ? -v : v;
}

// Sobel 幅值
static int sobel_mag(int gx, int gy) {
#pragma HLS INLINE
    return absv(gx) + absv(gy);
}

// 5x5 窗口上的 Sobel，r/c 必须是编译期常量（取值 1..3）。
//   Gx = 右列三项 - 左列三项，权重 [1,2,1]
//   Gy = 下行三项 - 上行三项，权重 [1,2,1]
// v[5][5] 已完全分区成 25 个独立寄存器，只有常量下标才能直连；
// 用变量下标会生成一个 25 选 1 的 MUX 网络，白吃几百 LUT 还拖慢时序。
#define SB_GX(r,c) ((int)v[(r)-1][(c)+1] + 2*(int)v[(r)][(c)+1] + (int)v[(r)+1][(c)+1] \
                  - (int)v[(r)-1][(c)-1] - 2*(int)v[(r)][(c)-1] - (int)v[(r)+1][(c)-1])
#define SB_GY(r,c) ((int)v[(r)+1][(c)-1] + 2*(int)v[(r)+1][(c)] + (int)v[(r)+1][(c)+1] \
                  - (int)v[(r)-1][(c)-1] - 2*(int)v[(r)-1][(c)] - (int)v[(r)-1][(c)+1])

// 二项式核 [1,4,6,4,1] 的移位加法形式，5 次乘法变成 2 个加法 + 2 次移位，不占 DSP。
// 一维核的系数和是 16 不是 256，可分离实现是两级各 >>4。
// 两级分别截断，所以黄金参考也必须分两步 >>4，否则对拍会在有噪声的图上随机差 1。
#define BINOM5(p0,p1,p2,p3,p4) (((int)(p0) + (int)(p4)) \
                               + 4 * ((int)(p1) + (int)(p3)) \
                               + 6 * (int)(p2))
#define BINOM5_SHIFT 4        // 一维核的归一化移位：/16

// 四级融合流水线：op 9(GAUSS5) / 10(CANNY) / 11(NMS)
//   级1a  读 in[y][x] → AGC → 横向滑窗 hc[5] → hconv[y][x-2]
//   级1b  hb[0..3][x] = hconv[y-4..y-1]，配 hconv[y][x-2] → vconv[y-2][x-2]
//   级2   vb[0..3][x] = vconv[y-6..y-3]，配 vconv[y-2][x-2] → 5x5 窗口
//         中心 v[2][2] = vconv[y-4][x-4]
//   级3/4 9 个 Sobel → NMS → 双阈值 → 滞后
//   写回  out[(y-4)*width + (x-4)]
// 中间结果不出片，一趟访存走完四级。
static int path5x5(unsigned char *in, unsigned char *out,
                   int width, int height, int op, int gain, int thresh) {

    // 有效输出区要求 行[4,H-5] 列[4,W-5]，所以最小 9x9
    if (width < 9 || height < 9) {
        int n = width * height;
        for (int i = 0; i < n; i++) {
#pragma HLS PIPELINE II=1
            out[i] = 0;
        }
        return 0;
    }

    int hi = THRESH_HIGH(thresh);
    int lo = THRESH_LOW(thresh);
    if (hi < lo) hi = lo;          // 防止驱动传反了导致弱边永远连不上

    // ---- 级1 的存储 ----
    unsigned char hc[5];                    // 横向卷积滑窗，不跨行，每行重置
#pragma HLS ARRAY_PARTITION variable=hc complete
    static unsigned char hb[4][MAX_W];     // hconv 的 4 行（第 y-4 .. y-1 行）
#pragma HLS ARRAY_PARTITION variable=hb complete dim=1

    // ---- 级2 的存储 ----
    static unsigned char vb[4][MAX_W];     // vconv 的 4 行（第 y-6 .. y-3 行）
#pragma HLS ARRAY_PARTITION variable=vb complete dim=1
    unsigned char v[5][5];                 // vconv 的 5x5 滑窗，25 个独立寄存器
#pragma HLS ARRAY_PARTITION variable=v complete dim=0

    // 滞后用的强边标记，2 行轮转缓冲。不能写成行末 for(x) sp[x]=sc[x]：
    // 那是 640 拍 × 480 行 ≈ 3 ms，等于把整帧又跑一遍。
    static unsigned char st[2][MAX_W];
#pragma HLS ARRAY_PARTITION variable=st complete dim=1

    // 输出行缓冲，两块按 y 的奇偶轮转。
    // DDR 写只有「无条件 + 地址连续」才能推成 burst，而本流水线要到 y>=8 && x>=8
    // 才出结果，直接写就是条件写。所以结果先落片上，再由一次连续写刷出上一行。
    // 列 0..3 和 width-4..width-1 永远不被计算循环写到，清零一次即恒 0。
    static unsigned char outr[2][MAX_W];
#pragma HLS ARRAY_PARTITION variable=outr complete dim=1

    // ---------------- 清零 ----------------
    for (int x = 0; x < width; x++) {
#pragma HLS PIPELINE II=1
        st[0][x] = 0;
        st[1][x] = 0;
        vb[0][x] = 0;
        vb[1][x] = 0;
        vb[2][x] = 0;
        vb[3][x] = 0;
        outr[0][x] = 0;      // 只清一次，不是每行
        outr[1][x] = 0;
    }
    for (int r = 0; r < 5; r++) {
        for (int c = 0; c < 5; c++) {
#pragma HLS PIPELINE II=1
            v[r][c] = 0;
        }
    }
    // 第 0..3 行恒 0
    for (int y = 0; y < 4; y++) {
        for (int x = 0; x < width; x++) {
#pragma HLS PIPELINE II=1
            out[y * width + x] = 0;
        }
    }
    // 最后四行恒 0
    for (int y = height - 4; y < height; y++) {
        for (int x = 0; x < width; x++) {
#pragma HLS PIPELINE II=1
            out[y * width + x] = 0;
        }
    }

    // ---------------- 预装：第 0..3 行的横向卷积结果 -> hb[0..3] ----------------
    // 每行都要重新清一次横向滑窗：横向卷积不跨行，上一行末尾不能带到下一行开头。
    // 刻意拆成 4 段而不是一个 yy 循环：hb 用变量下标会让 HLS 生成 MUX 网络并抬高 II。

    // 第 0 行
    for (int k = 0; k < 5; k++) {
#pragma HLS PIPELINE II=1
        hc[k] = 0;
    }
    for (int x = 0; x < width; x++) {
#pragma HLS PIPELINE II=1
        unsigned char c = agc(in[x], gain);
        hc[0] = hc[1]; hc[1] = hc[2]; hc[2] = hc[3]; hc[3] = hc[4];
        hc[4] = c;
        hb[0][x] = (unsigned char)(BINOM5(hc[0],hc[1],hc[2],hc[3],hc[4]) >> BINOM5_SHIFT);
    }

    // 第 1 行
    for (int k = 0; k < 5; k++) {
#pragma HLS PIPELINE II=1
        hc[k] = 0;
    }
    for (int x = 0; x < width; x++) {
#pragma HLS PIPELINE II=1
        unsigned char c = agc(in[width + x], gain);
        hc[0] = hc[1]; hc[1] = hc[2]; hc[2] = hc[3]; hc[3] = hc[4];
        hc[4] = c;
        hb[1][x] = (unsigned char)(BINOM5(hc[0],hc[1],hc[2],hc[3],hc[4]) >> BINOM5_SHIFT);
    }

    // 第 2 行
    for (int k = 0; k < 5; k++) {
#pragma HLS PIPELINE II=1
        hc[k] = 0;
    }
    for (int x = 0; x < width; x++) {
#pragma HLS PIPELINE II=1
        unsigned char c = agc(in[2 * width + x], gain);
        hc[0] = hc[1]; hc[1] = hc[2]; hc[2] = hc[3]; hc[3] = hc[4];
        hc[4] = c;
        hb[2][x] = (unsigned char)(BINOM5(hc[0],hc[1],hc[2],hc[3],hc[4]) >> BINOM5_SHIFT);
    }

    // 第 3 行
    for (int k = 0; k < 5; k++) {
#pragma HLS PIPELINE II=1
        hc[k] = 0;
    }
    for (int x = 0; x < width; x++) {
#pragma HLS PIPELINE II=1
        unsigned char c = agc(in[3 * width + x], gain);
        hc[0] = hc[1]; hc[1] = hc[2]; hc[2] = hc[3]; hc[3] = hc[4];
        hc[4] = c;
        hb[3][x] = (unsigned char)(BINOM5(hc[0],hc[1],hc[2],hc[3],hc[4]) >> BINOM5_SHIFT);
    }

    // ---------------- 主循环：读第 y 行，写第 y-4 行 ----------------
    // y 从 4 起，这是能算出第一个 vconv 的最早时机；y = 4..7 只推进流水线不输出。
    for (int y = 4; y < height; y++) {
        int base_in  = y * width;
        // 本轮算出的输出行 = y-4，刷回 DDR 的是上一轮算好的 = y-5。
        // y=4 时退化成再写一次第 0 行，此时 outr 还是清零状态，与语义一致。
        int oy_prev  = (y >= 5) ? (y - 5) : 0;
        int base_out = oy_prev * width;
        int iw = (y - 4) & 1;      // 当前行 strong / 输出行缓冲 写进哪一块
        int ir = (y - 5) & 1;      // 上一行 strong / 输出行缓冲 从哪一块读

        // 横向卷积不跨行：每轮开头重置滑窗
        for (int k = 0; k < 5; k++) {
#pragma HLS PIPELINE II=1
            hc[k] = 0;
        }

        // 级4 滞后要用的「当前行左邻的 strong」。用寄存器而不是回读 st[iw][xo-1]：
        // 回读是同块 RAM 的读写同拍 + 跨迭代 RAW，HLS 判成 Memory Dependency，
        // 主循环 II 只能到 2。左邻永远是上一拍的值，一个 FF 就够。
        unsigned char prev_s = 0;

        for (int x = 0; x < width; x++) {
#pragma HLS PIPELINE II=1
            // st / outr 的 inter false 是可证明安全的：本轮写 iw 块、读 ir 块，
            // 而 (y-4) 与 (y-5) 奇偶性必然相反，iw != ir 恒成立，读写落在不同 RAM 上。
            // HLS 的别名分析只看到变量下标，会保守地把 II 顶到 2。
#pragma HLS DEPENDENCE variable=st inter false
#pragma HLS DEPENDENCE variable=outr inter false
            // ---------- 级1a：AGC + 横向 5 抽头 → hconv[y][x-2] ----------
            unsigned char c = agc(in[base_in + x], gain);
            hc[0] = hc[1]; hc[1] = hc[2]; hc[2] = hc[3]; hc[3] = hc[4];
            hc[4] = c;
            unsigned char hn = (unsigned char)(BINOM5(hc[0],hc[1],hc[2],hc[3],hc[4]) >> BINOM5_SHIFT);

            // ---------- 级1b：纵向 5 抽头 → vconv[y-2][x-2] ----------
            //   hb[0..3][x] = hconv[y-4 .. y-1][x-2]，hn = hconv[y][x-2]
            //   系数对称：B5 = [1,4,6,4,1] -> (h0+hn) + 4·(h1+h3) + 6·h2
            unsigned char h0 = hb[0][x];
            unsigned char h1 = hb[1][x];
            unsigned char h2 = hb[2][x];
            unsigned char h3 = hb[3][x];
            unsigned char vn = (unsigned char)(BINOM5(h0,h1,h2,h3,hn) >> BINOM5_SHIFT);
            // 行缓存滚动：丢掉 hconv[y-4]，把 hconv[y] 推进来
            hb[0][x] = h1; hb[1][x] = h2; hb[2][x] = h3; hb[3][x] = hn;

            // ---------- 级2：vconv 的 5x5 滑窗 ----------
            unsigned char q0 = vb[0][x];
            unsigned char q1 = vb[1][x];
            unsigned char q2 = vb[2][x];
            unsigned char q3 = vb[3][x];
            vb[0][x] = q1; vb[1][x] = q2; vb[2][x] = q3; vb[3][x] = vn;

            // 滑窗左移一列，再把新的一列从右边推进去
            for (int r = 0; r < 5; r++) {
#pragma HLS UNROLL
                v[r][0] = v[r][1]; v[r][1] = v[r][2];
                v[r][2] = v[r][3]; v[r][3] = v[r][4];
            }
            v[0][4] = q0; v[1][4] = q1; v[2][4] = q2;
            v[3][4] = q3; v[4][4] = vn;

            // DDR 写回：无条件 + 地址连续，写的是上一轮算完的那一行
            out[base_out + x] = outr[ir][x];

            // ---------- 级3 / 级4 ----------
            // 中心 v[2][2] = vconv[y-4][x-4]，要凑齐 5x5 必须 y>=8 且 x>=8
            if (y >= 8 && x >= 8) {
                int xo = x - 4;

                if (op == MODE_GAUSS5) {
                    // 抽头 A：级1 的输出
                    outr[iw][xo] = v[2][2];

                } else {
                    // ---- 级2：Sobel 幅值 + 方向量化（中心）----
                    int gxC  = SB_GX(2, 2);
                    int gyC  = SB_GY(2, 2);
                    int ax   = absv(gxC);
                    int ay   = absv(gyC);
                    int magC = ax + ay;

                    // 用 2·|·| 比较代替 tan(22.5°) 与 tan(67.5°)，只要移位和比较
                    int d;
                    if      (2 * ay <= ax) d = 0;    // 0°   梯度近水平
                    else if (2 * ax <= ay) d = 2;    // 90°  梯度近竖直
                    else if (gxC * gyC > 0) d = 1;   // 45°
                    else                    d = 3;   // 135°

                    // ---- 级3：NMS，沿梯度方向比较两侧邻居的幅值 ----
                    //   4 组对侧邻居，每组 2 个 Sobel，加中心一共 9 个，
                    //   全部落在同一个 5x5 窗口内（中心 ±2 行 ±2 列）。
                    //   图像坐标 y 向下为正：
                    //     d=0 比左右 (2,1)/(2,3)      d=2 比上下 (1,2)/(3,2)
                    //     d=1 比主对角 (1,1)/(3,3)    d=3 比副对角 (1,3)/(3,1)
                    int n1, n2;
                    if      (d == 0) {
                        n1 = sobel_mag(SB_GX(2, 1), SB_GY(2, 1));
                        n2 = sobel_mag(SB_GX(2, 3), SB_GY(2, 3));
                    } else if (d == 2) {
                        n1 = sobel_mag(SB_GX(1, 2), SB_GY(1, 2));
                        n2 = sobel_mag(SB_GX(3, 2), SB_GY(3, 2));
                    } else if (d == 1) {
                        n1 = sobel_mag(SB_GX(1, 1), SB_GY(1, 1));
                        n2 = sobel_mag(SB_GX(3, 3), SB_GY(3, 3));
                    } else {
                        n1 = sobel_mag(SB_GX(1, 3), SB_GY(1, 3));
                        n2 = sobel_mag(SB_GX(3, 1), SB_GY(3, 1));
                    }

                    // 严格 >= 两侧才保留（平局保留，避免把线打断）
                    int m = (magC >= n1 && magC >= n2) ? magC : 0;

                    if (op == MODE_NMS) {
                        // 抽头 B：级3 的输出
                        outr[iw][xo] = clamp8(m);

                    } else {
                        // ---- 级4：双阈值 + 滞后 ----
                        //   m >= HIGH -> 强边 255；LOW <= m < HIGH 且邻域有强边 -> 提升为 255；其余 0
                        //
                        // 只查「上一行 3 邻 + 当前行左邻」：右邻和下一行在流式顺序里还没算出来，
                        // 这是流式 Canny 硬件的标准近似，效果是边缘最多延迟一行补全。
                        //
                        // 列 0..3 恒 0，所以 xo-1 要 >= 4 才读，即 xo >= 5。
                        unsigned char vv = 0, strong = 0;
                        if (m >= hi) {
                            vv = 255; strong = 1;
                        } else if (m >= lo) {
                            unsigned char nb = 0;
                            if (xo >= 5)             nb |= st[ir][xo - 1];
                                                     nb |= st[ir][xo];
                            if (xo + 1 <= width - 5) nb |= st[ir][xo + 1];
                            nb |= prev_s;                    // 当前行左邻（寄存器）
                            if (nb) { vv = 255; strong = 1; }
                        }
                        st[iw][xo] = strong;
                        prev_s = strong;                     // 给下一列当"左邻"
                        outr[iw][xo] = vv;
                    }
                }
            }
        }
    }

    // 收尾：刷出最后一个算出来的输出行（oy = height-5），
    // 主循环每轮刷的是上一轮算的，最后一行必然留在缓冲里。
    {
        int last_oy    = height - 5;
        int last_bank  = last_oy & 1;
        int last_base  = last_oy * width;
        for (int x = 0; x < width; x++) {
#pragma HLS PIPELINE II=1
            out[last_base + x] = outr[last_bank][x];
        }
    }
    return 0;
}

int rgb2gray(unsigned char *in, unsigned char *out,
             int width, int height, int mode, int gain, int thresh) {
// AXI3 的 burst 上限是 16，写更大只会被拆成若干个 16-beat。
// num_*_outstanding 会传播到 BD 里 AXI Interconnect 的 S*_ACCEPTANCE，
// 那个参数的合法范围是 (1,32)，设 64 要进 Vivado 才会报 [BD 41-241]。
#pragma HLS INTERFACE m_axi     port=in   offset=slave bundle=gmem0 depth=MAX_PIXELS max_read_burst_length=16 num_read_outstanding=16
#pragma HLS INTERFACE m_axi     port=out  offset=slave bundle=gmem1 depth=MAX_PIXELS max_write_burst_length=16 num_write_outstanding=16
#pragma HLS INTERFACE s_axilite port=width      bundle=control
#pragma HLS INTERFACE s_axilite port=height     bundle=control
#pragma HLS INTERFACE s_axilite port=mode       bundle=control
#pragma HLS INTERFACE s_axilite port=gain       bundle=control
#pragma HLS INTERFACE s_axilite port=thresh     bundle=control
#pragma HLS INTERFACE s_axilite port=return     bundle=control

    int op = mode & 15;

    // ---------------- 融合流水线：op 9 / 10 / 11 ----------------
    if (op >= MODE_GAUSS5) {
        return path5x5(in, out, width, height, op, gain, thresh);
    }

    // Immerkær 累加器：Σ |I ⊛ N|，N = [[1,-2,1],[-2,4,-2],[1,-2,1]]
    // |lap| <= 2040，有效像素 <= 304964，和上界 6.22e8 < 2^31，32 位够用。

    // ---------------- 模式 0：逐像素直通 + AGC ----------------
    // op 0 没有 3x3 窗口，测不了噪声；驱动需要 sigma 时退回 ARM 侧实现
    if (op == MODE_GRAY) {
        int n = width * height;
        for (int i = 0; i < n; i++) {
#pragma HLS PIPELINE II=1
            out[i] = agc(in[i], gain);
        }
        return 0;
    }

    // ---------------- 模式 1~8：AGC + 行缓存 + 滑窗 ----------------

    // 尺寸太小凑不出 3x3，直接全 0 兜底
    if (width < 3 || height < 3) {
        int n = width * height;
        for (int i = 0; i < n; i++) {
#pragma HLS PIPELINE II=1
            out[i] = 0;
        }
        return 0;
    }

    // 两块片上行缓存：rowA = 第 y-2 行，rowB = 第 y-1 行。
    // 存的是已经过 AGC 的灰度，保证窗口里 9 个值增益一致。
    static unsigned char rowA[MAX_W];
    static unsigned char rowB[MAX_W];

    // 输出行缓冲（2 行乒乓），与 path5x5 同一套做法。
    // 列 0 与列 width-1 恒 0（计算循环写的是 x-1，只覆盖 1..width-2），所以只清一次。
    static unsigned char outr[2][MAX_W];
#pragma HLS ARRAY_PARTITION variable=outr complete dim=1

    // sigma 累加器拆成 16 bank 轮换，用 x 的低 4 位选 bank。
    // 单个累加器是跨 307200 次迭代的循环进位链，每拍都要等上一拍结果，
    // HLS 无法流水化；拆开后同一个 bank 的两次更新间隔 16 拍，循环能到 II=1。
    // 整数加法可交换且各 bank 不溢出，结果与单累加器逐位相同。
    int sigma_bank[SIGMA_BANKS];
#pragma HLS ARRAY_PARTITION variable=sigma_bank complete
    for (int i = 0; i < SIGMA_BANKS; i++) {
#pragma HLS PIPELINE II=1
        sigma_bank[i] = 0;
    }

    // 3x3 滑窗寄存器（完全分区 -> 每周期可同时取 9 个值）
    unsigned char w0[3], w1[3], w2[3];
#pragma HLS ARRAY_PARTITION variable=w0 complete
#pragma HLS ARRAY_PARTITION variable=w1 complete
#pragma HLS ARRAY_PARTITION variable=w2 complete

    // 输出行缓冲清零（只跑一次，不是每行）
    for (int x = 0; x < width; x++) {
#pragma HLS PIPELINE II=1
        outr[0][x] = 0;
        outr[1][x] = 0;
    }

    // 第 0 行：整行输出 0
    for (int x = 0; x < width; x++) {
#pragma HLS PIPELINE II=1
        out[x] = 0;
    }

    // 预装第 0 行、第 1 行（过 AGC）。刻意拆成两个循环：
    // 合在一起时 HLS 要给一个循环配 2 个乘法器，预装会被降到 II=2。
    for (int x = 0; x < width; x++) {
#pragma HLS PIPELINE II=1
        rowA[x] = agc(in[x], gain);                     // 第 0 行
    }
    for (int x = 0; x < width; x++) {
#pragma HLS PIPELINE II=1
        rowB[x] = agc(in[width + x], gain);             // 第 1 行
    }

    // 主循环：读第 y 行 → 算第 y-1 行（落片上）→ 同时把第 y-2 行刷回 DDR
    for (int y = 2; y < height; y++) {
        int base_in   = y * width;                 // 本轮要读入的行
        int out_row   = y - 2;                     // 本轮刷回 DDR 的行
        int base_out  = out_row * width;
        int cur_bank  = (y - 1) & 1;               // 第 y-1 行的结果写进哪一块
        int prev_bank = out_row & 1;               // 第 y-2 行从哪一块读

        for (int x = 0; x < width; x++) {
#pragma HLS PIPELINE II=1
            // outr 的 inter false 是可证明安全的：本轮写 cur_bank、读 prev_bank，
            // (y-1) 与 (y-2) 奇偶性必然相反，二者恒不相等，读写落在不同 RAM 上。
#pragma HLS DEPENDENCE variable=outr inter false
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

            // 4) DDR 写回：无条件 + 地址连续，写的是上一轮算完的那一行。
            // 与同迭代的 in[base_in+x] 读分处 gmem1 / gmem0 两个 bundle，可并行。
            out[base_out + x] = outr[prev_bank][x];

            // 5) 窗口填满（x >= 2）后，中心列 = x-1，开始算
            if (x >= 2) {
                unsigned char p00 = w0[0], p01 = w0[1], p02 = w0[2];
                unsigned char p10 = w1[0], p11 = w1[1], p12 = w1[2];
                unsigned char p20 = w2[0], p21 = w2[1], p22 = w2[2];

                // 8 个算子共用的中间量
                int cross4 = (int)p01 + (int)p10 + (int)p12 + (int)p21;
                int sum9   = cross4 + (int)p00 + (int)p02
                                    + (int)p20 + (int)p22 + (int)p11;

                // Immerkær 累加（零额外访存）：Laplacian 与窗口对齐，
                // 四角 +1、四边 -2、中心 +4，cross4 正好就是"四边"。
                int corner4 = (int)p00 + (int)p02 + (int)p20 + (int)p22;
                int lap = corner4 - 2 * cross4 + 4 * (int)p11;
                sigma_bank[x & SIGMA_MASK] += (lap < 0 ? -lap : lap);

                if (op == MODE_SOBEL) {
                    int gx = (p02 + 2 * p12 + p22) - (p00 + 2 * p10 + p20);
                    int gy = (p20 + 2 * p21 + p22) - (p00 + 2 * p01 + p02);
                    int mag = (gx < 0 ? -gx : gx) + (gy < 0 ? -gy : gy);
                    outr[cur_bank][x - 1] = clamp8(mag);

                } else if (op == MODE_DIR) {
                    // 梯度方向量化，Gx/Gy 与 MODE_SOBEL 同源，只是取方向不取幅值
                    int gx = (p02 + 2 * p12 + p22) - (p00 + 2 * p10 + p20);
                    int gy = (p20 + 2 * p21 + p22) - (p00 + 2 * p01 + p02);
                    int ax = (gx < 0 ? -gx : gx);
                    int ay = (gy < 0 ? -gy : gy);
                    int d;
                    if      (2 * ay <= ax) d = 0;    // 0°   梯度近水平
                    else if (2 * ax <= ay) d = 2;    // 90°  梯度近竖直
                    else if (gx * gy > 0)  d = 1;    // 45°
                    else                   d = 3;    // 135°
                    outr[cur_bank][x - 1] = (unsigned char)(d * 85);

                } else if (op == MODE_EMBOSS) {
                    outr[cur_bank][x - 1] = clamp8((int)p22 - (int)p00 + 128);

                } else if (op == MODE_GAUSS) {
                    int gw = sum9 + cross4 + 3 * (int)p11;
                    outr[cur_bank][x - 1] = (unsigned char)(gw >> 4);

                } else if (op == MODE_SHARPEN) {
                    outr[cur_bank][x - 1] = clamp8(5 * (int)p11 - cross4);

                } else if (op == MODE_LAPLACE) {
                    outr[cur_bank][x - 1] = clamp8(9 * (int)p11 - sum9);

                } else if (op == MODE_BINARY) {
                    outr[cur_bank][x - 1] = (p11 >= 128) ? 255 : 0;

                } else {   // MODE_MEAN
                    outr[cur_bank][x - 1] = (unsigned char)((sum9 * 1821) >> 14);
                }
            }
        }

    }

    // 收尾：刷出倒数第二行（最后一轮算的是第 height-2 行，还留在缓冲里）
    {
        int last_row  = height - 2;
        int last_bank = last_row & 1;
        int last_base = last_row * width;
        for (int x = 0; x < width; x++) {
#pragma HLS PIPELINE II=1
            out[last_base + x] = outr[last_bank][x];
        }
    }

    // 最后一行：整行输出 0
    for (int x = 0; x < width; x++) {
#pragma HLS PIPELINE II=1
        out[(height - 1) * width + x] = 0;
    }

    // 16 路归约
    int sigma_sum = 0;
    for (int i = 0; i < SIGMA_BANKS; i++) {
#pragma HLS PIPELINE II=1
        sigma_sum += sigma_bank[i];
    }
    return sigma_sum;
}
