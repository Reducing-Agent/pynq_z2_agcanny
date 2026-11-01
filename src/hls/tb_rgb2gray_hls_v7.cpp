#include <stdio.h>
#include "rgb2gray_hls_v7.h"

//
// C 仿真台 v7：8 个算子 x 3 档增益，逐像素与 golden model 比对
//
// golden 是"把公式独立再写一遍"得到的标准答案，不复用 rgb2gray() 里任何代码。
//
// 三档增益的分工：
//   256  = 1.0x  —— 回归：v7 必须与 v6 逐字节一致（(x*256)>>8 == x）
//   512  = 2.0x  —— 正常提亮
//   1000 ≈ 3.9x  —— 打满饱和，专门验证 clamp（暗图里 255 的区域会溢出）
//

#define TEST_W 64
#define TEST_H 64

static unsigned char in_buf[MAX_PIXELS];
static unsigned char out_buf[MAX_PIXELS];
static unsigned char gray_ref[MAX_PIXELS];

static int g_gain = GAIN_UNITY;

static unsigned char clamp8(int v) {
    if (v < 0)   return 0;
    if (v > 255) return 255;
    return (unsigned char)v;
}

// golden 侧的 AGC：与硬件公式相同，但独立重写
static int agc_ref(int x) {
    int t = (x * g_gain) >> 8;
    return (t > 255) ? 255 : t;
}

// 取"已过增益"的参考像素
static int G(int idx) { return agc_ref(gray_ref[idx]); }

// 生成 golden：mode 0~7
static void build_golden(int w, int h, int mode, unsigned char gld[]) {
    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            int idx = y * w + x;

            // 3x3 算子的最外一圈恒 0；mode 0 是逐像素算子，不受此约束
            if (mode != MODE_GRAY &&
                (x == 0 || y == 0 || x == w - 1 || y == h - 1)) {
                gld[idx] = 0;
                continue;
            }
            if (mode == MODE_GRAY) {
                gld[idx] = (unsigned char)G(idx);
                continue;
            }

            int r0 = (y - 1) * w, r1 = y * w, r2 = (y + 1) * w;
            int p00 = G(r0 + x - 1);
            int p01 = G(r0 + x    );
            int p02 = G(r0 + x + 1);
            int p10 = G(r1 + x - 1);
            int p11 = G(r1 + x    );
            int p12 = G(r1 + x + 1);
            int p20 = G(r2 + x - 1);
            int p21 = G(r2 + x    );
            int p22 = G(r2 + x + 1);

            int cross4 = p01 + p10 + p12 + p21;
            int sum9   = cross4 + p00 + p02 + p20 + p22 + p11;

            if (mode == MODE_SOBEL) {
                int gx = (p02 + 2 * p12 + p22) - (p00 + 2 * p10 + p20);
                int gy = (p20 + 2 * p21 + p22) - (p00 + 2 * p01 + p02);
                int mag = (gx < 0 ? -gx : gx) + (gy < 0 ? -gy : gy);
                gld[idx] = clamp8(mag);

            } else if (mode == MODE_EMBOSS) {
                gld[idx] = clamp8(p22 - p00 + 128);

            } else if (mode == MODE_GAUSS) {
                int gw = sum9 + cross4 + 3 * p11;
                gld[idx] = (unsigned char)(gw >> 4);

            } else if (mode == MODE_SHARPEN) {
                gld[idx] = clamp8(5 * p11 - cross4);

            } else if (mode == MODE_LAPLACE) {
                gld[idx] = clamp8(9 * p11 - sum9);

            } else if (mode == MODE_BINARY) {
                gld[idx] = (p11 >= 128) ? 255 : 0;

            } else {   // MODE_MEAN
                gld[idx] = (unsigned char)((sum9 * 1821) >> 14);
            }
        }
    }
}

static int check(int w, int h, int mode, int gain, const char *name) {
    static unsigned char gld[MAX_PIXELS];
    int n = w * h;

    g_gain = gain;
    build_golden(w, h, mode, gld);
    rgb2gray(in_buf, out_buf, w, h, mode, gain);

    int err = 0;
    for (int i = 0; i < n; i++) {
        if (out_buf[i] != gld[i]) {
            if (err < 5)
                printf("  Mismatch @%d : got %d, expect %d\n", i, out_buf[i], gld[i]);
            err++;
        }
    }
    if (err == 0) {
        printf("[PASS] gain=%4d mode=%d %-8s : %d pixels all match, sample=%d\n",
               gain, mode, name, n, out_buf[n / 2]);
    } else {
        printf("[FAIL] gain=%4d mode=%d %-8s : %d mismatches\n", gain, mode, name, err);
    }
    return err;
}

int main() {
    int w = TEST_W, h = TEST_H;

    // 造测试图：左半黑 / 右半白（竖直边缘），顶部 16 行横向渐变（水平边缘）
    // v6 起不再打包 RGB，直接喂灰度（测试图 r=g=b=v，故 gray_ref == v）
    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            unsigned char v;
            if (y < 16)        v = (unsigned char)((x * 4) & 0xFF);
            else if (x < w / 2) v = 0;
            else                v = 255;
            in_buf[y * w + x] = v;
            gray_ref[y * w + x] = v;
        }
    }

    const int gains[3] = {GAIN_UNITY, 512, 1000};
    const int modes[8] = {MODE_GRAY, MODE_SOBEL, MODE_EMBOSS, MODE_GAUSS,
                          MODE_SHARPEN, MODE_LAPLACE, MODE_BINARY, MODE_MEAN};
    const char *names[8] = {"gray", "sobel", "emboss", "gauss",
                            "sharpen", "laplace", "binary", "mean"};

    printf("Image %dx%d, testing 8 modes x 3 gains...\n\n", w, h);
    int err = 0;
    for (int gi = 0; gi < 3; gi++) {
        printf("---- gain = %d (%.2fx) ----\n", gains[gi], gains[gi] / 256.0);
        for (int mi = 0; mi < 8; mi++)
            err += check(w, h, modes[mi], gains[gi], names[mi]);
        printf("\n");
    }

    if (err == 0) {
        printf("ALL PASS : 8 modes x 3 gains = 24 cases verified\n");
        return 0;
    }
    printf("FAILED : total %d mismatches\n", err);
    return 1;
}
