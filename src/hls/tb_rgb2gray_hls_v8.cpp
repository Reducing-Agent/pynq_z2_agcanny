// v8 C 仿真测试台
//
// 设计原则：**黄金参考必须独立于 DUT**。
// 这里不调用 rgb2gray()，也不复用它的任何中间量，
// 全部按算子的**定义**（卷积核 / 公式）从输入图重算一遍。
// 否则"对拍通过"只是循环论证。
//
// 覆盖：
//   1) 回归：op 0..7 在 gain=256 下必须与 v7 语义逐字节一致
//      （这是 120 组板端对拍不用重做的依据）
//   2) op 8 = MODE_DIR 方向量化
//   3) sigma_out = Σ|I⊛N|，N=[[1,-2,1],[-2,4,-2],[1,-2,1]]（仅 op 1..8）
//   4) op 9 = GAUSS5，op 10 = CANNY，op 11 = NMS（5x5 通路，外两圈恒 0）
//   5) 边界：外一圈（op 1..8）/ 外两圈（op 9..11）；最小图；奇数尺寸
//
// 用法：Vitis HLS -> Test Bench 选本文件，Run C Simulation
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <vector>

#include "rgb2gray_hls_v8.h"

static int g_fail = 0;
static int g_case = 0;

// ---------------------------------------------------------------- 独立黄金参考
static unsigned char agc_ref(unsigned char x, int gain) {
    int t = ((int)x * gain) >> 8;
    return (t > 255) ? (unsigned char)255 : (unsigned char)t;
}

static int abs_ref(int v) { return v < 0 ? -v : v; }

// ---- 3x3 通路：按定义重算整幅图。w,h 为逻辑尺寸，in/out 各 w*h 字节。----
static void golden(const unsigned char *in, unsigned char *out,
                   int w, int h, int op, int gain, long long *sigma) {
    for (int i = 0; i < w * h; i++) out[i] = 0;
    long long s = 0;

    if (op == MODE_GRAY) {
        for (int i = 0; i < w * h; i++) out[i] = agc_ref(in[i], gain);
        *sigma = 0;
        return;
    }
    if (w < 3 || h < 3) { *sigma = 0; return; }

    for (int y = 1; y < h - 1; y++) {
        for (int x = 1; x < w - 1; x++) {
            // 3x3 窗口，按定义取邻域（过了 AGC）
            unsigned char p[3][3];
            for (int dy = -1; dy <= 1; dy++)
                for (int dx = -1; dx <= 1; dx++)
                    p[dy + 1][dx + 1] = agc_ref(in[(y + dy) * w + (x + dx)], gain);

            int p00 = p[0][0], p01 = p[0][1], p02 = p[0][2];
            int p10 = p[1][0], p11 = p[1][1], p12 = p[1][2];
            int p20 = p[2][0], p21 = p[2][1], p22 = p[2][2];

            int corner4 = p00 + p02 + p20 + p22;
            int edge4   = p01 + p10 + p12 + p21;
            int sum9    = corner4 + edge4 + p11;

            // Immerkær 的 Laplacian：四角 +1、四边 -2、中心 +4
            int lap = corner4 - 2 * edge4 + 4 * p11;
            s += (lap < 0 ? -lap : lap);

            unsigned char v = 0;
            switch (op) {
            case MODE_SOBEL: {
                int gx = (p02 + 2 * p12 + p22) - (p00 + 2 * p10 + p20);
                int gy = (p20 + 2 * p21 + p22) - (p00 + 2 * p01 + p02);
                int ax = gx < 0 ? -gx : gx, ay = gy < 0 ? -gy : gy;
                int m = ax + ay;
                v = (unsigned char)(m > 255 ? 255 : m);
                break;
            }
            case MODE_DIR: {
                int gx = (p02 + 2 * p12 + p22) - (p00 + 2 * p10 + p20);
                int gy = (p20 + 2 * p21 + p22) - (p00 + 2 * p01 + p02);
                int ax = gx < 0 ? -gx : gx, ay = gy < 0 ? -gy : gy;
                int d;
                if      (2 * ay <= ax) d = 0;
                else if (2 * ax <= ay) d = 2;
                else if (gx * gy > 0)  d = 1;
                else                   d = 3;
                v = (unsigned char)(d * 85);
                break;
            }
            case MODE_EMBOSS: {
                int t = p22 - p00 + 128;
                v = (unsigned char)(t < 0 ? 0 : (t > 255 ? 255 : t));
                break;
            }
            case MODE_GAUSS:                 // [1 2 1;2 4 2;1 2 1]/16
                v = (unsigned char)((corner4 + 2 * edge4 + 4 * p11) >> 4);
                break;
            case MODE_SHARPEN: {             // [0 -1 0;-1 5 -1;0 -1 0]
                int t = 5 * p11 - edge4;
                v = (unsigned char)(t < 0 ? 0 : (t > 255 ? 255 : t));
                break;
            }
            case MODE_LAPLACE: {             // 8 邻 -1，中心 9
                int t = 9 * p11 - sum9;
                v = (unsigned char)(t < 0 ? 0 : (t > 255 ? 255 : t));
                break;
            }
            case MODE_BINARY:
                v = (p11 >= 128) ? 255 : 0;
                break;
            case MODE_MEAN:                  // 3x3 全 1 / 9
                v = (unsigned char)((sum9 * 1821) >> 14);
                break;
            default:
                v = 0;
            }
            out[y * w + x] = v;
        }
    }
    *sigma = s;
}

// ---------------------------------------------------------------------------
// 5x5 通路的独立黄金参考（op 9 / 10 / 11）
//
// 这里是**最朴素**的写法：25 项双重循环、Sobel 用变量下标函数、
//   滞后用整幅二维数组。跟 DUT 的"每行 5 项 + 纵向 5 项"、
//   "常量下标宏"、"2 行循环缓冲"是完全不同的代码路径 ——
//   只有两条独立路径算出同一个结果，才说明算法是对的。
// ---------------------------------------------------------------------------
static const int B5[5] = {1, 4, 6, 4, 1};   // 二项式核 [1,4,6,4,1]

// 以 (r,c) 为中心的 Sobel 幅值，r,c ∈ [1,3]
static int gsobel_mag(const unsigned char p[5][5], int r, int c) {
    int gx = (int)p[r-1][c+1] + 2*(int)p[r][c+1] + (int)p[r+1][c+1]
           - (int)p[r-1][c-1] - 2*(int)p[r][c-1] - (int)p[r+1][c-1];
    int gy = (int)p[r+1][c-1] + 2*(int)p[r+1][c] + (int)p[r+1][c+1]
           - (int)p[r-1][c-1] - 2*(int)p[r-1][c] - (int)p[r-1][c+1];
    return abs_ref(gx) + abs_ref(gy);
}

static void golden5x5(const unsigned char *in, unsigned char *out,
                      int w, int h, int op, int gain, int thresh) {
    for (int i = 0; i < w * h; i++) out[i] = 0;
    if (w < 9 || h < 9) return;      // 同 DUT：有效区要求 9x9

    int hi = THRESH_HIGH(thresh);
    int lo = THRESH_LOW(thresh);
    if (hi < lo) hi = lo;

    // ---- 级1：5x5 可分离二项式高斯（零填充）----
    //
    // 必须**分两步 >>4**，不能一次 >>8：
    //    一维核 [1,4,6,4,1] 的系数和是 **16**，不是 256
    //    （二维才是 16×16=256）。PL 是两级各 >>4，中间结果被截成
    //    8 bit 再喂给第二级 —— 这个精度损失是真实存在的。
    //    写成一次 >>8 会在噪声图上随机差 1，csim 会零星 FAIL。
    //
    // 写法刻意与 PL 不同：PL 是流式滑窗 + 行缓存，
    // 这里是整幅数组 + 双重循环按定义累加。
    std::vector<int> hconv((size_t)w * h, 0);
    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            int s = 0;
            for (int k = 0; k < 5; k++) {
                int xx = x - 2 + k;                 // 中心在 x，窗口 x-2..x+2
                int val = (xx < 0 || xx >= w)
                        ? 0                          // 零填充
                        : (int)agc_ref(in[(size_t)y * w + xx], gain);
                s += B5[k] * val;
            }
            hconv[(size_t)y * w + x] = s >> 4;
        }
    }
    // vconv：纵向。DUT 只在行 [2, h-3] 产出 vconv（纵向卷积不越界）
    std::vector<int> vconv((size_t)w * h, 0);
    for (int y = 2; y <= h - 3; y++) {
        for (int x = 0; x < w; x++) {
            int s = 0;
            for (int k = 0; k < 5; k++) {
                int yy = y - 2 + k;
                s += B5[k] * hconv[(size_t)yy * w + x];
            }
            vconv[(size_t)y * w + x] = s >> 4;
        }
    }

    // 抽头 A：级1 的输出
    if (op == MODE_GAUSS5) {
        for (int y = 4; y <= h - 5; y++)
            for (int x = 4; x <= w - 5; x++)
                out[(size_t)y * w + x] = (unsigned char)(vconv[(size_t)y * w + x] & 0xFF);
        return;
    }

    std::vector<int> mag((size_t)w * h, 0);          // NMS 后的幅值
    std::vector<unsigned char> strong((size_t)w * h, 0);

    // ---- 级2/3：Sobel + 方向 + NMS（有效区 [4,h-5] x [4,w-5]）----
    for (int y = 4; y <= h - 5; y++) {
        for (int x = 4; x <= w - 5; x++) {
            unsigned char p[5][5];
            for (int dy = -2; dy <= 2; dy++)
                for (int dx = -2; dx <= 2; dx++)
                    p[dy + 2][dx + 2] = (unsigned char)(vconv[(size_t)(y + dy) * w + (x + dx)] & 0xFF);

            // 中心的幅值和方向
            int mC = gsobel_mag(p, 2, 2);
            int gxc = (int)p[1][3] + 2*(int)p[2][3] + (int)p[3][3]
                    - (int)p[1][1] - 2*(int)p[2][1] - (int)p[3][1];
            int gyc = (int)p[3][1] + 2*(int)p[3][2] + (int)p[3][3]
                    - (int)p[1][1] - 2*(int)p[1][2] - (int)p[1][3];
            int ax = abs_ref(gxc), ay = abs_ref(gyc);
            int d;
            if      (2 * ay <= ax) d = 0;
            else if (2 * ax <= ay) d = 2;
            else if (gxc * gyc > 0) d = 1;
            else                    d = 3;

            // NMS：沿梯度方向比较两侧邻居
            int n1, n2;
            if      (d == 0) { n1 = gsobel_mag(p, 2, 1); n2 = gsobel_mag(p, 2, 3); }
            else if (d == 2) { n1 = gsobel_mag(p, 1, 2); n2 = gsobel_mag(p, 3, 2); }
            else if (d == 1) { n1 = gsobel_mag(p, 1, 1); n2 = gsobel_mag(p, 3, 3); }
            else             { n1 = gsobel_mag(p, 1, 3); n2 = gsobel_mag(p, 3, 1); }

            mag[(size_t)y * w + x] = (mC >= n1 && mC >= n2) ? mC : 0;
        }
    }

    // ---- 级4：双阈值 + 滞后 ----
    // 必须按 y 递增、x 递增的顺序，因为滞后依赖"已确认的邻域"。
    //   邻域 = 上一行 3 邻 + 当前行左邻（右邻和下一行还没算出来）。
    // 列 0..3 恒 0（最外四圈），所以左邻要 x-1 >= 4 才读，即 x >= 5。
    for (int y = 4; y <= h - 5; y++) {
        for (int x = 4; x <= w - 5; x++) {
            int m = mag[(size_t)y * w + x];
            if (m >= hi) {
                out[y * w + x] = 255;
                strong[(size_t)y * w + x] = 1;
            } else if (m >= lo) {
                unsigned char nb = 0;
                if (x >= 5)         nb |= strong[(size_t)(y - 1) * w + (x - 1)];
                                    nb |= strong[(size_t)(y - 1) * w + x];
                if (x + 1 <= w - 5) nb |= strong[(size_t)(y - 1) * w + (x + 1)];
                if (x >= 5)         nb |= strong[(size_t)y * w + (x - 1)];
                if (nb) {
                    out[y * w + x] = 255;
                    strong[(size_t)y * w + x] = 1;
                }
            }
            if (op == MODE_NMS) {
                // NMS 模式：输出细化后的幅值本身，不做阈值
                out[y * w + x] = (unsigned char)(m > 255 ? 255 : m);
            }
        }
    }
}

// ---------------------------------------------------------------- 测试驱动
static void run_case(const char *tag, const std::vector<unsigned char> &img,
                     int w, int h, int op, int gain, int thresh) {
    g_case++;
    int n = w * h;

    // 2026-10-29 踩坑：缓冲区必须按 **MAX_PIXELS** 分配，不能按 w*h 分配
    //
    //   cosim 的自动 wrapper（hls/sim/wrapc/apatb_rgb2gray.cpp）里写死了：
    //       port8.nbytes = { 307200 };   // in_r
    //       port9.nbytes = { 307200 };   // out_r
    //   也就是**每次调用都无条件 dump depth(=MAX_PIXELS) 个字节**，
    //   完全不看这次的 w*h 是多少。
    //
    //   我第一版写成 std::vector<unsigned char> in(n)（n = w*h），
    //   结果 32x24 那种小图只有 768 字节，wrapper 却要读 307200 字节
    //   → 越界 → SIGSEGV，CodeState = ENTER_WRAPC，
    //     **864 例一例都没跑到就崩了**（cosim 报告只显示 "Fail / NA"）。
    //
    //   迷惑点：**csim 完全不受影响**（csim 直接调原函数，不走 wrapper），
    //   所以会出现"csim 864/864 全绿，cosim 一例都跑不了"的诡异现象。
    //
    //   （v5 的 TB 用的是 static in_buf[MAX_PIXELS]，当年侥幸没踩到。）
    std::vector<unsigned char> in(MAX_PIXELS, 0), out(MAX_PIXELS, 0xEE),
                               ref(MAX_PIXELS, 0xEE);
    for (int i = 0; i < n; i++) in[i] = img[i % img.size()];

    // v8 起 sigma 走函数返回值（见 .h 的踩坑注释），不用指针出参。
    //   指针版有两个坑，都踩过：
    //     1) 用 (int*)&long long 承接 -> 只写低 4 字节，高 4 字节保留 -1 的
    //        0xFFFFFFFF，150 例全因 sigma 对不上而 FAIL
    //     2) 就算类型对了，HLS 也不会给 s_axilite 指针出参建寄存器，
    //        ARM 根本读不到（cosim 也连挂两轮）
    int sigma_dut = rgb2gray(in.data(), out.data(), w, h, op, gain, thresh);

    long long sigma_ref = 0;
    if (op >= MODE_GAUSS5) {
        golden5x5(in.data(), ref.data(), w, h, op, gain, thresh);
        // 5x5 通路恒不累加 σ（见 .h 的说明）
        sigma_ref = 0;
    } else {
        golden(in.data(), ref.data(), w, h, op, gain, &sigma_ref);
    }

    int bad = 0, first = -1;
    for (int i = 0; i < n; i++) {
        if (out[i] != ref[i]) { bad++; if (first < 0) first = i; }
    }
    bool sig_ok = (sigma_dut == sigma_ref);

    if (bad || !sig_ok) {
        g_fail++;
        printf("  [FAIL] %-26s op=%2d gain=%4d th=%04x %dx%d  ", tag, op, gain, thresh, w, h);
        if (bad) {
            int y = first / w, x = first % w;
            printf("输出 %d/%d 处不同，首差 (%d,%d) dut=%d ref=%d  ",
                   bad, n, y, x, out[first], ref[first]);
        }
        if (!sig_ok) printf("sigma dut=%d ref=%lld  ", sigma_dut, sigma_ref);
        printf("\n");
    } else {
        printf("  [ ok ] %-26s op=%2d gain=%4d th=%04x %dx%d  sigma=%d\n",
               tag, op, gain, thresh, w, h, sigma_dut);
    }
}

int main() {
    printf("======================================================================\n");
#ifdef __HLS_COSIM__
    printf("v8.2 C/RTL 联合仿真（cosim）：**裁剪版** 用例集\n");
#else
    printf("v8.2 C 仿真：回归(op 0..8) + 5x5 融合流水线(op 9..11) + sigma\n");
#endif
    printf("======================================================================\n");

    // 确定性伪随机图（不用 rand()，保证跨平台可复现）
    const int NIMG = 96;
    std::vector<unsigned char> src(NIMG);
    unsigned int st = 12345u;
    for (int i = 0; i < NIMG; i++) {
        st = st * 1103515245u + 12345u;
        src[i] = (unsigned char)((st >> 16) & 0xFF);
    }

    // v8.2 的融合流水线有效区是 行[4,H-5] 列[4,W-5]，要求 w>=9 且 h>=9。
    //   所以极小图用 9x9 和 12x10；3x3 / 5x5 保留下来是为了验证
    //   "尺寸不够 -> 全 0 兜底"这条分支确实生效。
    struct { int w, h; const char *tag; } sizes[] = {
        {32, 24, "小图"},
        {64, 48, "中图"},
        {33, 17, "奇数尺寸"},
        {9,  9,  "9x9 最小"},
        {12, 10, "12x10 窄图"},
        {5,  5,  "5x5 兜底"},
        {3,  3,  "3x3 兜底"},
        {640, 480, "VGA 满幅"},
    };

    const int gains[] = {GAIN_UNITY, 512, 1000};
    const int threshes[] = {CANNY_THRESH_DEFAULT, THRESH_PACK(100, 50), THRESH_PACK(20, 10)};

    int nsizes = 8, ngains = 3, nthres = 3;

    // cosim 跑的是 RTL 仿真（XSIM），比 csim 慢几个数量级，而且每次调用
    //   都要 dump 2×307200 字节的测试向量。全量 864 例在 RTL 上要跑好几小时、
    //   写出 500MB+ 向量文件。所以 cosim 只跑**裁剪版**：
    //   12 个 op 全覆盖，尺寸 / gain / thresh 各取 2 档 = 2×2×2×12 = 96 例。
    //   全量回归由 csim 保证。
    //
    //   `-D__HLS_COSIM__` 是 Vitis 编译 cosim 版 TB 时自动加的宏
    //   （见 hls/sim/wrapc/cosim.tv.mk 的 IFLAG），不需要自己定义。
#ifdef __HLS_COSIM__
    nsizes = 2;      // 小图 + 中图
    ngains = 2;      // 256 直通 + 1000 强增益
    nthres = 2;      // 默认 60/30 + 100/50
#endif

    for (int s = 0; s < nsizes; s++) {
        for (int g = 0; g < ngains; g++) {
            for (int t = 0; t < nthres; t++) {
                for (int op = 0; op <= MODE_MAX_IMPL; op++) {
                    run_case(sizes[s].tag, src, sizes[s].w, sizes[s].h,
                             op, gains[g], threshes[t]);
                }
            }
        }
    }

    printf("\n======================================================================\n");
    printf("合计 %d 例，失败 %d 例 -> %s\n", g_case, g_fail,
           g_fail == 0 ? "全部 PASS" : "有失败");
    printf("======================================================================\n");
    return g_fail == 0 ? 0 : 1;
}
