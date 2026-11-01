#include <stdio.h>
#include "rgb2gray_hls.h"

//
// C 仿真台 v5：8 个算子各跑一遍，逐像素与 golden model 比对
//
// golden 是"把公式独立再写一遍"得到的标准答案，不复用 rgb2gray() 里任何代码，
// 这样才算交叉验证。跑通后再做 C 综合 / 协同仿真。
//
// 已经先在 PC 上用 Python 把同一套行缓存逻辑对拍过（verify_v5.py，8 模式 x 7 张图全 PASS），
// 这里再在 C 层确认一次，剩下的就只有综合/实现层面的问题了。
//

#define TEST_W 64
#define TEST_H 64

static unsigned int  in_buf[MAX_PIXELS];
static unsigned char out_buf[MAX_PIXELS];
static unsigned char gray_ref[MAX_PIXELS];

static unsigned char clamp8(int v) {
    if (v < 0)   return 0;
    if (v > 255) return 255;
    return (unsigned char)v;
}

// 与硬件同一套公式，独立再写一遍作为"标准答案"
static unsigned char gray_of(unsigned int pix) {
    unsigned char r = (unsigned char)(pix         & 0xFF);
    unsigned char g = (unsigned char)((pix >> 8)  & 0xFF);
    unsigned char b = (unsigned char)((pix >> 16) & 0xFF);
    return (unsigned char)((77 * r + 150 * g + 29 * b) >> 8);
}

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
                gld[idx] = gray_ref[idx];
                continue;
            }

            int r0 = (y - 1) * w, r1 = y * w, r2 = (y + 1) * w;
            int p00 = gray_ref[r0 + x - 1];
            int p01 = gray_ref[r0 + x    ];
            int p02 = gray_ref[r0 + x + 1];
            int p10 = gray_ref[r1 + x - 1];
            int p11 = gray_ref[r1 + x    ];
            int p12 = gray_ref[r1 + x + 1];
            int p20 = gray_ref[r2 + x - 1];
            int p21 = gray_ref[r2 + x    ];
            int p22 = gray_ref[r2 + x + 1];

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
                // [1 2 1; 2 4 2; 1 2 1] / 16 —— /16 就是 >>4，精确
                int gw = sum9 + cross4 + 3 * p11;
                gld[idx] = (unsigned char)(gw >> 4);

            } else if (mode == MODE_SHARPEN) {
                // [0 -1 0; -1 5 -1; 0 -1 0]
                gld[idx] = clamp8(5 * p11 - cross4);

            } else if (mode == MODE_LAPLACE) {
                // [-1 -1 -1; -1 8 -1; -1 -1 -1] = 9*中心 - 九宫格和
                gld[idx] = clamp8(9 * p11 - sum9);

            } else if (mode == MODE_BINARY) {
                gld[idx] = (p11 >= 128) ? 255 : 0;

            } else {   // MODE_MEAN
                // /9 用 (x*1821)>>14 精确实现（穷举 0..2295 已验证），不占 DSP
                gld[idx] = (unsigned char)((sum9 * 1821) >> 14);
            }
        }
    }
}

static int check(int w, int h, int mode, const char *name) {
    static unsigned char gld[MAX_PIXELS];
    int n = w * h;

    build_golden(w, h, mode, gld);
    rgb2gray(in_buf, out_buf, w, h, mode);

    int err = 0;
    for (int i = 0; i < n; i++) {
        if (out_buf[i] != gld[i]) {
            if (err < 5)
                printf("  Mismatch @%d : got %d, expect %d\n", i, out_buf[i], gld[i]);
            err++;
        }
    }
    if (err == 0) {
        printf("[PASS] mode=%d %-8s : %d pixels all match, sample=%d\n",
               mode, name, n, out_buf[n / 2]);
    } else {
        printf("[FAIL] mode=%d %-8s : %d mismatches\n", mode, name, err);
    }
    return err;
}

int main() {
    int w = TEST_W, h = TEST_H;

    // 造测试图：左半黑 / 右半白（竖直边缘），顶部 16 行横向渐变（水平边缘）
    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            unsigned char v;
            if (y < 16)        v = (unsigned char)((x * 4) & 0xFF);
            else if (x < w / 2) v = 0;
            else                v = 255;
            unsigned int pix = ((unsigned int)v << 16) | ((unsigned int)v << 8) | v;
            in_buf[y * w + x] = pix;
            gray_ref[y * w + x] = gray_of(pix);
        }
    }

    printf("Image %dx%d, testing 8 modes...\n\n", w, h);
    int err = 0;
    err += check(w, h, MODE_GRAY,    "gray");
    err += check(w, h, MODE_SOBEL,   "sobel");
    err += check(w, h, MODE_EMBOSS,  "emboss");
    err += check(w, h, MODE_GAUSS,   "gauss");
    err += check(w, h, MODE_SHARPEN, "sharpen");
    err += check(w, h, MODE_LAPLACE, "laplace");
    err += check(w, h, MODE_BINARY,  "binary");
    err += check(w, h, MODE_MEAN,    "mean");

    if (err == 0) {
        printf("\nALL PASS : 8 modes verified\n");
        return 0;
    }
    printf("\nFAILED : total %d mismatches\n", err);
    return 1;
}
