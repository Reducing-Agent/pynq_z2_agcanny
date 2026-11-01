#include "rgb2gray_hls_v8.h"

//
// 图像加速核：行缓存(line buffer) + 滑窗
//   3x3 通路  ：8 算子 + AGC + Immerkær 噪声累加       （op 0..8）
//   融合流水线：高斯 → Sobel → NMS → 双阈值+滞后 四级串联（op 9..11）
//
// v3 -> v4：把"每像素 9 次访存"改成行缓存 + 滑窗
// v4 -> v5：在同一套行缓存滑窗上挂 8 个算子，接口与边界语义不变
// v5 -> v6：灰度化移到 ARM 侧（cv2.cvtColor），PL 收 1 B/px 灰度，每像素访存 5 B -> 2 B
// v6 -> v7：加入 AGC 自动增益（光照自适应）
// v7 -> v8：加入 ① Immerkær 噪声累加  ② 梯度方向量化（MODE_DIR）
// v8 -> v8.1：5x5 通路（GAUSS5 / CANNY / NMS）+ 可调双阈值
// v8.1 -> v8.2：**把 5x5 通路重构成四级融合流水线**
//
// v8.2 到底改了什么：从"算子并列"到"算子串联"
//
//   v8.1 的 5x5 通路是**并列**的：op 9/10/11 各自从原始灰度出发，
//   算自己的那一截，互相不喂数据。想要"先去噪再提边缘"的效果，
//   只能驱动侧跑两趟（GAUSS5 一趟 -> 回 DDR -> CANNY 第二趟），
//   两次访存 + 一次 DDR 往返 = 6.1 ms。这是**两趟拼起来的链**。
//
//   v8.2 把四级**串起来**，级与级之间用片上行缓存首尾相接：
//
//     输入灰度
//       │ 级1  AGC + 5x5 可分离高斯（横 5 抽头 → 4 块 hconv 行缓存 → 纵 5 抽头）
//       │ 级2  Sobel 幅值 + 方向量化（9 个 Sobel 共用 5x5 vconv 窗口）
//       │ 级3  非极大值抑制 NMS
//       │ 级4  双阈值 + 滞后
//       ▼
//     输出
//
//   一趟访存走完四级，中间结果**不回 DDR**。这就是"多算子融合"。
//
// 这不等于"加大并行度"，两者的区别必须讲清楚（答辩会被问）：
//     并行度 = 单元**横向复制**，算法链没变长，只是同一拍多算几份
//     融合   = 单元**纵向串联**，链变长，单元不复制
//   这里只做后者。
//
// 四级在硬件上**全部**都在跑：op 是运行时变量，HLS 不会为你省掉
//   任何一级。op 只决定**从哪一级抽头**把结果写回 DDR：
//     op 9  GAUSS5 -> 抽级1 的输出（去噪图）
//     op 11 NMS    -> 抽级3 的输出（细化后的幅值）
//     op 10 CANNY  -> 抽级4 的输出（0/255 边缘）
//
// 为什么 3x3 通路不动（op 0..8）：
//   它们是"单算子演示"，本来就是并列关系，没有前后依赖，
//   硬串起来只会让每个算子的结果都被前一级污染。
//   把它们串起来 = 破坏 223 组板端回归，收益却是负的。
//
// 总延迟：四级各延迟 2 行 2 列
//     横向卷积 2 列 / 纵向卷积 2 行 / Sobel 2 行 2 列
//   -> 有效输出区 行 [4,H-5] 列 [4,W-5]，**最外四圈恒 0**
//   对 VGA 只损失 3.2% 的像素（298304 / 307200），换一趟出边缘，值得。

// 限幅到 0~255
static unsigned char clamp8(int v) {
#pragma HLS INLINE
    if (v < 0)   return (unsigned char)0;
    if (v > 255) return (unsigned char)255;
    return (unsigned char)v;
}

// AGC：Q8.8 定点增益。x >= 0、gain >= 0，所以只需上限饱和。
// gain = 256 时 (x*256)>>8 == x，严格直通（v7/v8 的回归一致性靠这条保证）。
static unsigned char agc(unsigned char x, int gain) {
#pragma HLS INLINE
    int t = ((int)x * gain) >> 8;
    return (t > 255) ? (unsigned char)255 : (unsigned char)t;
}

static int absv(int v) {
#pragma HLS INLINE
    return (v < 0) ? -v : v;
}

// Sobel 幅值。传进来的 gx/gy 已经是算好的，避免宏展开时把
// 同一个表达式重复写进三目运算符（虽然编译器会做 CSE，但显式更稳）。
static int sobel_mag(int gx, int gy) {
#pragma HLS INLINE
    return absv(gx) + absv(gy);
}

// ---------------------------------------------------------------------------
// vconv 5x5 窗口上的 Sobel（r,c 必须是**编译期常量**，取值 1..3）
//
//   Gx = 右列三项 - 左列三项，权重 [1,2,1]   —— 检测竖直边缘
//   Gy = 下行三项 - 上行三项，权重 [1,2,1]   —— 检测水平边缘
//
// 为什么必须是常量下标：
//   v[5][5] 被 complete 分区成 25 个独立寄存器，只有常量下标
//   HLS 才能直接连到对应寄存器。写成 v[r][c]（r,c 是变量）
//   会生成一个 25 选 1 的 MUX 网络，白白吃掉几百 LUT 还拖慢时序。
// ---------------------------------------------------------------------------
#define SB_GX(r,c) ((int)v[(r)-1][(c)+1] + 2*(int)v[(r)][(c)+1] + (int)v[(r)+1][(c)+1] \
                  - (int)v[(r)-1][(c)-1] - 2*(int)v[(r)][(c)-1] - (int)v[(r)+1][(c)-1])
#define SB_GY(r,c) ((int)v[(r)+1][(c)-1] + 2*(int)v[(r)+1][(c)] + (int)v[(r)+1][(c)+1] \
                  - (int)v[(r)-1][(c)-1] - 2*(int)v[(r)-1][(c)] - (int)v[(r)-1][(c)+1])

// 二项式核 [1,4,6,4,1] 的移位加法形式：
//   a·[p0..p4] = (p0+p4) + 4·(p1+p3) + 6·p2
// 把 5 次乘法全变成 2 个加法 + 2 次移位，不占 DSP。
//
// 系数和是 **16，不是 256**（2026-10-29 踩坑）
//   一维 [1,4,6,4,1] 求和 = 1+4+6+4+1 = 16
//   二维 [1,4,6,4,1]⊗[1,4,6,4,1] 求和 = 16×16 = 256
//   可分离实现是**两级各除以 16**，即每级 >>4，不是每级 >>8。
//   我第一版两级都写成 >>8，结果是除以了 65536，输出几乎全 0
//   （平坦图 77 进去出来是 0，Python 对拍 162 例挂了 123 例）。
//
// 两级分别截断 ≠ 一次 >>8：
//   中间结果 hconv 被截成 8 bit 再喂给第二级，会损失精度。
//   所以**黄金参考也必须分两步 >>4**，不能图省事写成一次 >>8，
//   否则对拍会在有噪声的图上随机差 1。
#define BINOM5(p0,p1,p2,p3,p4) (((int)(p0) + (int)(p4)) \
                               + 4 * ((int)(p1) + (int)(p3)) \
                               + 6 * (int)(p2))
#define BINOM5_SHIFT 4        // 一维核的归一化移位：/16

// ---------------------------------------------------------------------------
// 四级融合流水线：op 9(GAUSS5) / 10(CANNY) / 11(NMS)
//
// 级与级之间的连接物：
//   hb[4]  存 hconv（横向卷积结果）的 4 行
//   vb[4]  存 vconv（纵向卷积结果 = 高斯输出）的 4 行
//   v[5][5] vconv 的 5x5 滑窗
//   st[2]  滞后用的强边标记，2 行循环缓冲
//
// 处理第 y 行第 x 列时的数据流：
//   级1a  读 in[y][x] → AGC → 横向滑窗 hc[5] → hconv[y][x-2]
//   级1b  hb[0..3][x] = hconv[y-4..y-1]，配 hconv[y][x-2] → vconv[y-2][x-2]
//   级2   vb[0..3][x] = vconv[y-6..y-3]，配 vconv[y-2][x-2] → 5x5 窗口
//         v[r][c] = vconv[y-6+r][x-6+c]，**中心 v[2][2] = vconv[y-4][x-4]**
//   级3/4 9 个 Sobel → NMS → 双阈值 → 滞后
//   写回  out[(y-4)*width + (x-4)]
//
// 有效输出区：行 [4, H-5]，列 [4, W-5] -> 最外**四**圈恒 0
//   （v8.1 是外两圈，v8.2 因为多串了一级高斯，边界又往外推了两圈）
//
// σ 不在这条通路上累加（返回 0），原因见 .h。
// ---------------------------------------------------------------------------
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
    unsigned char hc[5];                    // 横向卷积滑窗（不跨行，每行重置）
#pragma HLS ARRAY_PARTITION variable=hc complete
    static unsigned char hb[4][MAX_W];     // hconv 的 4 行（第 y-4 .. y-1 行）
#pragma HLS ARRAY_PARTITION variable=hb complete dim=1

    // ---- 级2 的存储 ----
    static unsigned char vb[4][MAX_W];     // vconv 的 4 行（第 y-6 .. y-3 行）
#pragma HLS ARRAY_PARTITION variable=vb complete dim=1
    unsigned char v[5][5];                 // vconv 的 5x5 滑窗，25 个独立寄存器
#pragma HLS ARRAY_PARTITION variable=v complete dim=0

    // ---- 级4 的存储：滞后连通性用的「强边标记」，2 行循环缓冲 ----
    // sp 语义上是"上一行的 strong"，sc 是"当前行的 strong"。
    // 不能写成"行末 for(x) sp[x]=sc[x]"：那是 640 拍 × 480 行
    //   = 307200 拍 ≈ 3 ms，等于把整帧又跑了一遍。
    //   改成 2 行轮转（写 st[(y-4)&1]、读 st[(y-5)&1]），零拷贝。
    // 变量下标在这里很便宜：地址 x 同时送到两块 RAM，
    //   输出端只是一组 8 bit 的 2:1 MUX，十几 LUT 而已。
    static unsigned char st[2][MAX_W];
#pragma HLS ARRAY_PARTITION variable=st complete dim=1

    // ---- 输出行缓冲（2026-10-30 B 方案，2 行乒乓）----
    //   DDR 写必须是「无条件 + 地址连续」才能推成 burst，但融合流水线
    //   只在 y>=8 && x>=8 时才出结果 —— 直接写 DDR 就是条件写，burst 崩。
    //   所以先把结果落到片上，再用一个独立的连续写把上一行刷出去。
    //   两块按 y 的奇偶轮转，与 st[2] 完全同构（iw / ir 已经算好了）。
    //
    //   列 0..3 和 width-4..width-1 恒 0：计算循环写的是 xo = x-4，
    //     x 从 8 起 -> xo 从 4 到 width-5，永远碰不到这两端。
    //     所以只要**清零一次**，之后每一行这两端都自动是 0。
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
        outr[0][x] = 0;      // 只清一次，不是每行；见 outr 声明处的说明
        outr[1][x] = 0;
    }
    for (int r = 0; r < 5; r++) {
        for (int c = 0; c < 5; c++) {
#pragma HLS PIPELINE II=1
            v[r][c] = 0;
        }
    }
    // 2026-10-30：原来这里有两组「每行首尾四列恒 0」的循环，**已删除**。
    //   现在整行都由 outr 刷出，而 outr 的两端列永远不被计算循环写
    //   （xo = x-4，x>=8 -> xo>=4；x<=width-1 -> xo<=width-5），
    //   清零一次即恒 0，不必每行再写一遍。
    //   省下的是 8 次**跨行步进**的零碎 DDR 写（每行 8 次 × 480 行），
    //   这种非连续写是拉低 burst 效率的典型写法。
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
    // 每行都要**重新清一次横向滑窗**：横向卷积不跨行，
    //   上一行末尾的 4 个像素不能带到下一行开头。
    // 刻意拆成 4 段而不是一个 yy 循环：hb 用变量下标会让 HLS
    //   生成 MUX 网络，还会把 II 拉高。4 段常量下标最干净。
    //   代价只是代码长一点；预装总共 4×640 拍，一帧一次，可忽略。
    //
    // 2026-10-29 踩坑：**宏定义里不能出现 `#pragma`**
    //   第一版我把这 4 段写成 `#define PRELOAD_HCONV(IDX, ROW)`，
    //   宏体里带了 `#pragma HLS PIPELINE II=1`。结果：
    //     error: use of undeclared identifier 'PRELOAD_HCONV'
    //     error: expected ';' after expression      （共 9 个错）
    //   原因：`#` 在宏定义里是**字符串化运算符**，`#pragma` 会被当成
    //   字符串化 `pragma`，宏定义直接被破坏，等于没定义。
    //   HLS 的 pragma 想进宏只能用 `_Pragma("...")` 运算符，
    //   但为了不引入新坑，这里直接手写 4 段，最稳。

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
    // y 从 4 起：这是能算出第一个 vconv（vconv[2]）的最早时机。
    // y = 4..7 这四轮只推进流水线不输出（输出行 0..3 本来就该是 0）。
    for (int y = 4; y < height; y++) {
        int base_in  = y * width;
        // 本轮**算出**的输出行 = y-4；本轮**刷回 DDR** 的是上一轮算好的 = y-5
        // （落后一行，因为结果要先落到片上缓冲）
        // y=4 时 y-5 = -1，没有这一行：退化成再写一次第 0 行。
        // 此时 outr[ir] 还是清零状态（y<8 不出结果），写的全是 0，
        // 与「第 0..3 行恒 0」的语义一致 —— 安全，且没有负地址。
        int oy_prev  = (y >= 5) ? (y - 5) : 0;
        int base_out = oy_prev * width;
        int iw = (y - 4) & 1;      // 当前行 strong / 输出行缓冲 写进哪一块
        int ir = (y - 5) & 1;      // 上一行 strong / 输出行缓冲 从哪一块读

        // 横向卷积不跨行：每轮开头重置滑窗
        for (int k = 0; k < 5; k++) {
#pragma HLS PIPELINE II=1
            hc[k] = 0;
        }

        // 级4 滞后要用的「当前行左邻的 strong」。
        // 2026-10-29：原来写成回读 st[iw][xo-1]（同一块 RAM 上一拍刚写的值），
        //   这是**同块 RAM 的读写同拍 + 跨迭代 RAW**，HLS 判成 Memory Dependency
        //   -> 主循环 II 只能到 2（csynth 报 "II Violation: Memory Dependency"）。
        //   改成寄存器：左邻永远是上一拍的 strong，一个 FF 就够，
        //   既不占 RAM 端口，又天然是 1 拍延迟。
        //   每行开头必须清零（列 0..3 恒 0，没有"左邻"）。
        unsigned char prev_s = 0;

        for (int x = 0; x < width; x++) {
#pragma HLS PIPELINE II=1
            // st 的依赖说明（为什么可以 inter false）：
            //   本轮写 st[iw][xo]，只读 st[ir][xo-1..xo+1]。
            //   iw = (y-4)&1，ir = (y-5)&1 —— (y-4) 与 (y-5) 奇偶性必然相反，
            //   所以 **ir != iw 恒成立**，读写落在两块不同的 RAM 上，
            //   任意两次 x 迭代之间**不存在**真依赖。
            //   HLS 的别名分析看不出 "ir != iw"（它只看到变量下标），
            //   会保守地按"可能同一块 RAM"处理，把 II 顶到 2。
            //   这里显式声明 inter false 是**可证明安全**的，不是赌。
#pragma HLS DEPENDENCE variable=st inter false
            // outr 同理：本轮写 outr[iw][xo]，只读 outr[ir][x]。
            // iw = (y-4)&1，ir = (y-5)&1，(y-4) 与 (y-5) 奇偶性必然相反，
            // 所以 **ir != iw 恒成立**，读写落在两块不同的 RAM 上，无真依赖。
#pragma HLS DEPENDENCE variable=outr inter false
            // ---------- 级1a：AGC + 横向 5 抽头 → hconv[y][x-2] ----------
            unsigned char c = agc(in[base_in + x], gain);
            hc[0] = hc[1]; hc[1] = hc[2]; hc[2] = hc[3]; hc[3] = hc[4];
            hc[4] = c;
            unsigned char hn = (unsigned char)(BINOM5(hc[0],hc[1],hc[2],hc[3],hc[4]) >> BINOM5_SHIFT);

            // ---------- 级1b：纵向 5 抽头 → vconv[y-2][x-2] ----------
            //   hb[0..3][x] = hconv[y-4 .. y-1][x-2]
            //   hn          = hconv[y][x-2]
            //   vconv[y-2] = Σ B5[i]·hconv[y-4+i]  （i=0..4）
            //   系数对称：B5 = [1,4,6,4,1] -> (h0+hn) + 4·(h1+h3) + 6·h2
            unsigned char h0 = hb[0][x];
            unsigned char h1 = hb[1][x];
            unsigned char h2 = hb[2][x];
            unsigned char h3 = hb[3][x];
            unsigned char vn = (unsigned char)(BINOM5(h0,h1,h2,h3,hn) >> BINOM5_SHIFT);
            // 行缓存滚动：丢掉 hconv[y-4]，把 hconv[y] 推进来
            hb[0][x] = h1; hb[1][x] = h2; hb[2][x] = h3; hb[3][x] = hn;

            // ---------- 级2：vconv 的 5x5 滑窗 ----------
            //   vb[0..3][x] = vconv[y-6 .. y-3][x-2]
            //   vn          = vconv[y-2][x-2]
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

            // DDR 写回（本轮性能修复的核心）
            //   无条件 + 地址连续 -> HLS 能推断 burst。
            //   原来这里是被 `if (y>=8 && x>=8)` 包住的条件写，burst 因此崩掉。
            //   写的是**上一轮**算完的那一行（落后一行，因为结果先落片上）。
            //   和同一次迭代里的 in[base_in+x] 读在不同 bundle 上，可并行。
            out[base_out + x] = outr[ir][x];

            // ---------- 级3 / 级4 ----------
            // 中心 v[2][2] = vconv[y-4][x-4]，要凑齐 5x5 必须 y>=8 且 x>=8
            if (y >= 8 && x >= 8) {
                int xo = x - 4;

                if (op == MODE_GAUSS5) {
                    // 抽头 A：级1 的输出（5x5 二项式高斯去噪图）
                    outr[iw][xo] = v[2][2];

                } else {
                    // ---- 级2：Sobel 幅值 + 方向量化（中心）----
                    int gxC  = SB_GX(2, 2);
                    int gyC  = SB_GY(2, 2);
                    int ax   = absv(gxC);
                    int ay   = absv(gyC);
                    int magC = ax + ay;

                    // 方向量化：用 2·|·| 比较代替 tan(22.5°)=0.4142 和
                    // tan(67.5°)=2.4142，只要移位和比较，不占乘法器。
                    // 与 op 8(MODE_DIR) 的判据**逐字相同**，保证两者语义一致。
                    int d;
                    if      (2 * ay <= ax) d = 0;    // 0°   梯度近水平
                    else if (2 * ax <= ay) d = 2;    // 90°  梯度近竖直
                    else if (gxC * gyC > 0) d = 1;   // 45°
                    else                    d = 3;   // 135°

                    // ---- 级3：NMS，沿梯度方向比较两侧邻居的幅值 ----
                    //   4 组对侧邻居，每组 2 个 Sobel，加中心一共 9 个。
                    //   这 9 个 3x3 全部落在同一个 5x5 窗口内
                    //   （中心 ±2 行 ±2 列），所以不需要 7x7。
                    //
                    //   图像坐标 y 向下为正，所以：
                    //     d=0  梯度沿 ±x        -> 比 (2,1)/(2,3)   左右
                    //     d=2  梯度沿 ±y        -> 比 (1,2)/(3,2)   上下
                    //     d=1  gx·gy>0 同号     -> 比 (1,1)/(3,3)   主对角
                    //     d=3  gx·gy<0 异号     -> 比 (1,3)/(3,1)   副对角
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
                        // 抽头 B：级3 的输出（细化后的幅值，可直视）
                        outr[iw][xo] = clamp8(m);

                    } else {
                        // ---- 级4：双阈值 + 滞后（可流式近似）----
                        //   m >= HIGH                -> 强边，输出 255
                        //   LOW <= m < HIGH 且邻域有强边 -> 提升为强边，输出 255
                        //   其他                      -> 0
                        //
                        // 为什么只查「上一行 3 邻 + 当前行左邻」而不是完整 8 邻：
                        //   右邻和下一行在流式顺序里**还没算出来**。
                        //   这是所有流式 Canny 硬件的标准近似，
                        //   效果是边缘最多延迟一行才"补全"，视觉上看不出来。
                        //
                        // 列 0..3 恒 0（凑不出 5x5），所以 xo-1 要 >= 4 才读，
                        //   即 xo >= 5。这条同时保证 st[*][0..3] 永远不被读，
                        //   于是循环缓冲不用每行清零。
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

    // 收尾：刷出最后一个算出来的输出行（oy = height-5）。
    // 主循环里每轮刷的是「上一轮算的」，所以最后一行必然留在缓冲里没刷出去。
    // 再往后 4 行（height-4 .. height-1）由上面的「最后四行恒 0」负责。
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

// 为什么 sigma 走返回值，不走指针出参（2026-10-28 实测踩坑）
//
// 第一版写成 `int *sigma_out` + `s_axilite ... depth=1`，结果：
//   · cosim 连挂两轮（212-300 / 212-360，CodeState ENTER_WRPC → ENTER_WRC）
//   · 更致命的是 **生成的 xrgb2gray_hw.h 里根本没有这个寄存器** ——
//     寄存器表和 v7 一字不差（width 0x10 / height 0x18 / mode 0x20 / gain 0x28），
//     全文搜不到 sigma。也就是说 ARM 根本读不到它，这个特性等于没做。
//
// 结论：Vitis HLS 2026.1 对「s_axilite 的指针出参」不会生成可寻址的标量寄存器。
// 改用 **函数返回值**（ap_return）—— 这是生成可读标量寄存器的标准做法，
// 会体现在 XRGB2GRAY_CONTROL_ADDR_AP_RETURN_DATA 里。
//
// ⚠️ 代价：返回值会挤占 0x10，把它们往后推一格。
//    **驱动必须用生成的头文件里的宏，不能硬编码偏移。**
//    下一轮构建后第一件事就是 grep xrgb2gray_hw.h 确认偏移。
int rgb2gray(unsigned char *in, unsigned char *out,
             int width, int height, int mode, int gain, int thresh) {
// ---------------------------------------------------------------------------
// 2026-10-30：真板实测 op 1~11 全是 20.576 ms（II=1 地板 3.072 ms 的 6.7 倍），
//    而 GRAY（纯顺序读写）是 3.222 ms。多尺寸拟合得 t = 66.8 ns/px × N + 50 µs，
//    即每像素 6.68 个周期 —— 不是 II 的问题，是 **m_axi 访存效率**。
//
//    根因：v6 把输入从 v5 的 `unsigned int*`（4 B/px）改成 `unsigned char*`（1 B/px），
//    每次 AXI 传输只搬 1 字节有效数据。邻域循环又「读第 y 行、写第 y-1 行」，
//    读写地址错开一整行，burst 推断失败 -> 退化成每像素一次带握手的独立传输。
//
// ⚠️ 关于 max_*_burst_length：**Zynq-7000 的 HP/GP 口都是 AXI3，burst 上限就是 16**。
//    写 256 不会更快，HLS 只会把它拆成若干个 16-beat。所以这个值保持 16。
//
// ⚠️⚠️ 关于 num_*_outstanding：**不能设 64**（2026-10-30 实踩）
//    第一版把 16 改成 64，Vitis HLS 全程 PASS，但一进 Vivado 就挂：
//      ERROR: [BD 41-241] Message from IP propagation TCL of /axi_mem_intercon/xbar:
//      Validation failed for parameter 'My S00_READ_ACCEPTANCE(S00_READ_ACCEPTANCE)'
//      Value '64' is out of the range (1,32)
//    原因：HLS 的 num_read_outstanding 会传播到 BD 里 AXI Interconnect 的
//    S00_READ_ACCEPTANCE / S01_WRITE_ACCEPTANCE，而这个 IP 的合法范围是 **(1,32)**。
//    Zynq-7000 的 AXI Interconnect 最多只能接受 32 个 outstanding。
//    -> **上限就是 32**，写 64 属于越界，HLS 侧不会报错，要等 Vivado 才炸。
//    这里保持 16：B 方案（burst 推断）才是真正的修复，outstanding 只是锦上添花，
//    16 是 v8.2 已验证能跑的值，不为边际收益多冒一次风险。
//
//    如果这样还不够，下一步必须改数据宽度（一次读 4 个像素），
//    光调 pragma 解决不了 8-bit 窄传输的根本问题。
//
// 2026-10-30 追加：**真正的根因找到了，而且不是上面猜的那些。**
//
//   决定性证据：op 9/10/11 走 path5x5()，那条通路**完全没有 σ 累加**，
//   却和 op 1~8 一样是 20.57 ms。所以"σ 累加器拖慢 5.8 倍"的旧结论作废。
//
//   两条慢通路唯一的共同点：**DDR 写在 if 里面**
//       op 1~8   : if (x >= 2)          out[base_out + (x-1)] = ...
//       op 9~11  : if (y >= 8 && x >= 8) out[base_out + xo]   = ...
//   唯一快的 GRAY：**写是无条件的** out[i] = agc(in[i])
//
//   -> **条件写让 HLS 推不出 burst**，退化成每像素一次带握手的独立 AXI 传输。
//
// B 方案（本轮落地）：算出来的值先落**片上输出行缓冲**（2 行乒乓），
//   DDR 写改成 `out[base + x] = outr[bank][x]` —— **无条件 + 地址连续**，
//   burst 自然成立。读和写仍在同一个循环里（和 GRAY 完全同构），
//   GRAY 实测 1.05 cyc/px 证明这条路能跑满，所以不需要改数据宽度。
//
// 为什么不做"一次读 4 个像素"：GRAY 已经 3.222 ms ≈ II=1 地板 3.072 ms，
//   说明 8-bit 口 + burst 已经能吃满 1 字节/拍。真正缺的是 burst，不是位宽。
// ---------------------------------------------------------------------------
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
    //
    // 上界证明（所以用 32 位就够）：
    //   |lap| <= 2040（+1 系数 4 个 + +4 系数 1 个全取 255，
    //                   或 -2 系数 4 个全取 255）
    //   有效像素 <= (640-2)*(480-2) = 304964
    //   => 和 <= 6.22e8 < 2^31 = 2.147e9
    // 2026-10-29 更正：上面这段"进位链不会往关键路径上加多少"的判断
    //   **是错的**，而且错得很贵。单累加器的瓶颈不在进位链长度，
    //   在于它是**跨迭代依赖** —— 每拍必须等上一拍的结果，HLS 无法流水化。
    //   实测 640x480 从 3.577 ms 掉到 20.57 ms（5.8 倍）。
    //   修法见下面 sigma_bank[] 的注释：拆成 16 bank 轮换。
    //   （上界证明仍然成立，所以每个 bank 仍用 32 位。）

    // ---------------- 模式 0：逐像素直通 + AGC ----------------
    // op 0 是逐像素算子，没有 3x3 窗口可用，所以这里测不了噪声。
    // 驱动侧遇到 op 0 又需要 sigma 时，退回 ARM 的 numpy Immerkær
    // （sigma.py 的 immerkaer()，已用合成分数据验证过）。
    if (op == MODE_GRAY) {
        int n = width * height;
        for (int i = 0; i < n; i++) {
#pragma HLS PIPELINE II=1
            out[i] = agc(in[i], gain);
        }
        return 0;
    }

    // ---------------- 模式 1~8：AGC + 行缓存 + 滑窗 ----------------

    // 尺寸太小就没法做 3x3，直接全 0 兜底（边界语义不变）
    if (width < 3 || height < 3) {
        int n = width * height;
        for (int i = 0; i < n; i++) {
#pragma HLS PIPELINE II=1
            out[i] = 0;
        }
        return 0;
    }

    // 两块片上行缓存：rowA = 第 y-2 行，rowB = 第 y-1 行（各 MAX_W 字节）
    // 存的是**已经过 AGC** 的灰度，保证窗口里 9 个值增益一致
    static unsigned char rowA[MAX_W];
    static unsigned char rowB[MAX_W];

    // ------------------------------------------------------------------
    // 2026-10-30 B 方案：输出行缓冲（2 行乒乓）
    //
    //   原来主循环里是 `if (x >= 2) out[base_out + (x-1)] = ...`
    //   —— **条件 DDR 写**，HLS 推不出 burst，退化成每像素一次独立 AXI 传输，
    //   实测 20.57 ms = 6.70 cyc/px（II=1 地板是 3.072 ms）。
    //
    //   改法：结果先落片上，DDR 写改成**无条件 + 地址连续**
    //       out[base_out + x] = outr[prev_bank][x]
    //   这样读写仍在同一循环、地址各自连续 —— 与 GRAY 分支完全同构，
    //   而 GRAY 实测 1.05 cyc/px，证明这条路能跑满。
    //
    //   列 0 与列 width-1 恒 0：计算循环写的是 x-1（x>=2 -> 只覆盖 1..width-2），
    //     永远碰不到这两列。所以**清零一次**即可，后面每行都自动是 0，
    //     不需要（也绝不能）每行再清一遍 —— 那是 640 拍 × 480 行 ≈ 3 ms。
    //
    //   两块按输出行的奇偶轮转，写 cur、读 prev，二者必然不同块。
    // ------------------------------------------------------------------
    static unsigned char outr[2][MAX_W];
#pragma HLS ARRAY_PARTITION variable=outr complete dim=1

    // ------------------------------------------------------------------
    // σ 累加器：16 bank 轮换（2026-10-29 为修性能而改）
    //
    // 原来的写法是单个累加器：
    //     sigma_sum += (lap < 0 ? -lap : lap);
    // 这是一个**跨 307200 次迭代的 32 位循环进位链**：每一拍都要等上一拍的
    // 累加结果，HLS 没法把它流水化。
    //
    // 板端实测后果（640x480）：
    //     v7（无 σ）主循环 3.577 ms = 1.16 cyc/px
    //     v8（有 σ）主循环 20.57 ms = 6.70 cyc/px     ← 慢 5.8 倍
    //     v8 的 GRAY 分支（无 σ）3.224 ms = 1.05 cyc/px ← AXI 通路没问题
    //   三个样本对照下来，唯一变量就是 σ 累加，确诊是它。
    //   csynth 报的 II=1 是理想调度下的估计，不能信；实测才是真的。
    //
    // 修法（标准做法）：拆成 16 个 bank，用 x 的低 4 位选 bank。
    //   同一个 bank 的两次更新因此间隔 16 拍，32 位加法（约 2~3 ns）
    //   在这 16 拍里早就算完了，HLS 就能把循环流水化到 II=1。
    //   代价：16 x 32 bit 寄存器 + 16 个加法器，约几百 LUT。
    //   最后再 16 路归约，归约循环只跑 16 次，开销可忽略。
    // ------------------------------------------------------------------
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

    // 输出行缓冲清零（**只跑一次**，不是每行 —— 见 outr 声明处的说明）
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

    // 主循环：读第 y 行 → 算第 y-1 行（落片上）→ 同时把第 y-2 行刷回 DDR
    for (int y = 2; y < height; y++) {
        int base_in   = y * width;                 // 本轮要读入的行
        int out_row   = y - 2;                     // 本轮刷回 DDR 的行
        int base_out  = out_row * width;
        int cur_bank  = (y - 1) & 1;               // 第 y-1 行的结果写进哪一块
        int prev_bank = out_row & 1;               // 第 y-2 行从哪一块读

        for (int x = 0; x < width; x++) {
#pragma HLS PIPELINE II=1
            // outr 的依赖说明（为什么可以 inter false）：
            //   本轮写 outr[cur_bank][x-1]，只读 outr[prev_bank][x]。
            //   cur_bank = (y-1)&1，prev_bank = (y-2)&1 —— (y-1) 与 (y-2)
            //   奇偶性必然相反，所以 **二者恒不相等**，读写落在两块不同的 RAM 上，
            //   任意两次 x 迭代之间**不存在**真依赖。
            //   HLS 的别名分析看不出这一点（它只看到变量下标），会保守地按
            //   "可能同一块 RAM"处理，把 II 顶到 2。这里显式声明是可证明安全的。
            //   pragma 放在循环体内 —— 与 path5x5 里 st 的写法一致（已验证有效）。
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

            // DDR 写回（本轮性能修复的核心）
            //   无条件 + 地址连续 -> HLS 能推断 burst。
            //   原来这行是 `if (x>=2) out[...] = ...`，条件写把 burst 打没了。
            //   写的是**上一轮**算完的那一行（结果先落片上，所以落后一行）。
            //   与同迭代的 in[base_in+x] 读分处 gmem1 / gmem0 两个 bundle，可并行
            //   —— GRAY 分支就是这个结构，实测 1.05 cyc/px。
            out[base_out + x] = outr[prev_bank][x];

            // 4) 窗口填满（x >= 2）后，中心列 = x-1，开始算
            if (x >= 2) {
                unsigned char p00 = w0[0], p01 = w0[1], p02 = w0[2];
                unsigned char p10 = w1[0], p11 = w1[1], p12 = w1[2];
                unsigned char p20 = w2[0], p21 = w2[1], p22 = w2[2];

                // 8 个算子共用的中间量
                int cross4 = (int)p01 + (int)p10 + (int)p12 + (int)p21;
                int sum9   = cross4 + (int)p00 + (int)p02
                                    + (int)p20 + (int)p22 + (int)p11;

                // ---- v8 新增：Immerkær 累加（零额外访存）----
                // Laplacian N = [[1,-2,1],[-2,4,-2],[1,-2,1]] 与窗口对齐：
                //   四角 +1，四边 -2，中心 +4
                // cross4 正好就是"四边"，所以只差一个"四角"
                int corner4 = (int)p00 + (int)p02 + (int)p20 + (int)p22;
                int lap = corner4 - 2 * cross4 + 4 * (int)p11;
                // 16 bank 轮换，打破跨迭代依赖（详见 sigma_bank 声明处）
                sigma_bank[x & SIGMA_MASK] += (lap < 0 ? -lap : lap);

                if (op == MODE_SOBEL) {
                    int gx = (p02 + 2 * p12 + p22) - (p00 + 2 * p10 + p20);
                    int gy = (p20 + 2 * p21 + p22) - (p00 + 2 * p01 + p02);
                    int mag = (gx < 0 ? -gx : gx) + (gy < 0 ? -gy : gy);
                    outr[cur_bank][x - 1] = clamp8(mag);

                } else if (op == MODE_DIR) {
                    // ---- v8 新增：梯度方向量化 ----
                    // Gx/Gy 与 MODE_SOBEL 完全同源，只是不取幅值取方向。
                    // 量化用 2*|·| 比较代替 tan(22.5°)=0.4142 / tan(67.5°)=2.4142，
                    // 只用移位和比较，不占乘法器 —— 硬件实现的标准做法。
                    // 输出 0/85/170/255，可以直接当图像看（四张方向图叠成灰度）。
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

    // 原来这里每行还有两次 `out[base_out]=0; out[base_out+width-1]=0`。
    //   **已删除**：现在整行由 outr 刷出，而计算循环写的是 x-1（x>=2），
    //   只覆盖列 1..width-2，永远碰不到列 0 和列 width-1 ——
    //   outr 清零一次后这两列恒 0，语义完全一致。
    //   顺带省掉 2 次/行的零碎非连续 DDR 写（原来每次都会打断 burst）。

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

    // 最后一行：整行输出 0（边界语义不变）
    for (int x = 0; x < width; x++) {
#pragma HLS PIPELINE II=1
        out[(height - 1) * width + x] = 0;
    }

    // 16 路归约：只跑 16 次，开销可忽略
    int sigma_sum = 0;
    for (int i = 0; i < SIGMA_BANKS; i++) {
#pragma HLS PIPELINE II=1
        sigma_sum += sigma_bank[i];
    }
    return sigma_sum;
}
