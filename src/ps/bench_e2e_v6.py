#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""v6 端到端拆解：把「一帧从相机 BGR 到结果可用」拆成四段计时。

为什么必须量这个
----------------
validate_v6.py 只量了 PL 核内（ap_start -> ap_done），v6 是 3.75 ms，
v5 是 3.78 ms —— **几乎没变**。拆带宽墙的提速没有兑现。

但 v6 真正省下的钱**不在 PL 里**：
  v5 端到端 19.72 ms，其中约 16 ms 是 ARM 把 BGR 打成 uint32（4 字节/像素）
  v6 改成 cv2.cvtColor + 一次连续拷贝（1 字节/像素）→ 打包应大幅下降

所以本脚本把端到端拆成四段，并且**在同一块板子上同时跑 v5 老打包方式和
v6 新打包方式**，直接给出 A/B 对照 —— 这才是能进报告的数字。

    T_pack   ARM 侧把 BGR 搬到 PL 输入缓冲
    T_cache  flush(in,out) + invalidate(out)
    T_kernel PL 核内（ap_start -> ap_done）
    T_e2e    三者之和（不含摄像头采集）

用法（板端，与 rgb2gray.bit / rgb2gray.hwh 同目录）：
    source /usr/local/share/pynq-venv/bin/activate
    python3 bench_e2e_v6.py            # 不要加 sudo
"""
import time

import cv2
import numpy as np

CTRL_BASE, PARAM_BASE = 0x4000_0000, 0x4001_0000
SIZE = 0x10000
AP_CTRL, WIDTH_REG, HEIGHT_REG, MODE_REG = 0x00, 0x10, 0x18, 0x20
IN_R_LOW, IN_R_HIGH, OUT_R_LOW, OUT_R_HIGH = 0x10, 0x14, 0x1C, 0x20

W, H = 640, 480
PIX = W * H
LOOPS = 20
MODE = 1                     # SOBEL：v5 实测核内 3.784 ms，可比


# ---------------- 两种打包方式（同一块板子上 A/B） ----------------
def pack_v5_uint32(img):
    """v5 的打包：BGR -> uint32 = R|G<<8|B<<16（4 字节/像素）。
    已经是**优化后**的写法（字节视图直写，避开 ARMv7 上 astype 的 ~76 ms）。"""
    buf = np.empty((H, W, 4), np.uint8)
    buf[..., 0] = img[..., 2]      # R
    buf[..., 1] = img[..., 1]      # G
    buf[..., 2] = img[..., 0]      # B
    buf[..., 3] = 0
    return buf.view(np.uint32).reshape(-1)


def pack_v6_gray(img):
    """v6 的打包：BGR -> 每像素 1 字节灰度（cv2.cvtColor + 摊平）。"""
    return cv2.cvtColor(img, cv2.COLOR_BGR2GRAY).reshape(-1)


def pack_v6_fused(img, buf):
    """v6 融合写法：cv2.cvtColor **直接写进 DMA 缓冲**，
    省掉「先转灰度再整块拷」的第二次全帧遍历。"""
    cv2.cvtColor(img, cv2.COLOR_BGR2GRAY, dst=buf[:PIX].reshape(H, W))


def make_img():
    """造一张有内容的图（纯色会导致编译器/缓存行为失真）。"""
    yy, xx = np.indices((H, W))
    img = np.zeros((H, W, 3), np.uint8)
    img[..., 0] = (xx // 3) % 256
    img[..., 1] = (yy // 2) % 256
    img[..., 2] = ((xx + yy) // 4) % 256
    cv2.rectangle(img, (100, 90), (300, 320), (240, 240, 240), -1)
    return img


# ---------------- PL 调用 ----------------
def run_ip(ctrl, param, in_buf, out_buf, mode):
    ctrl.write(WIDTH_REG, W)
    ctrl.write(HEIGHT_REG, H)
    ctrl.write(MODE_REG, mode)
    param.write(IN_R_LOW, int(in_buf.device_address) & 0xFFFFFFFF)
    param.write(IN_R_HIGH, 0)
    param.write(OUT_R_LOW, int(out_buf.device_address) & 0xFFFFFFFF)
    param.write(OUT_R_HIGH, 0)
    ctrl.write(AP_CTRL, 0x01)
    while True:
        if ctrl.read(AP_CTRL) & 0x2:
            break


def timeit(fn, loops=LOOPS):
    fn()                                   # 预热
    t0 = time.perf_counter()
    for _ in range(loops):
        fn()
    return (time.perf_counter() - t0) / loops * 1000.0


def main():
    from pynq import Overlay, allocate, MMIO

    ol = Overlay("rgb2gray.bit")
    print("overlay loaded, ip_dict =", list(ol.ip_dict.keys()))
    ctrl = MMIO(CTRL_BASE, SIZE)
    param = MMIO(PARAM_BASE, SIZE)

    in_buf = allocate(shape=(PIX,), dtype=np.uint8)     # v6 输入：1 B/px
    out_buf = allocate(shape=(PIX,), dtype=np.uint8)
    old_buf = allocate(shape=(PIX,), dtype=np.uint32)   # 只为量 v5 打包用
    img = make_img()

    # cacheable 属性决定 flush/invalidate 是不是空操作，先印出来
    for tag, b in (("in_buf", in_buf), ("out_buf", out_buf)):
        attrs = {k: getattr(b, k) for k in dir(b) if "cache" in k.lower()}
        print("%-8s dev=0x%08X %s" % (tag, b.device_address, attrs))

    # ---------- 1) 核内 ----------
    in_buf[:] = pack_v6_gray(img)
    in_buf.flush()
    t_kernel = timeit(lambda: run_ip(ctrl, param, in_buf, out_buf, MODE))

    # ---------- 2) 打包：v5 / v6 两段式 / v6 融合式 同板 A/B ----------
    t_pack_v5 = timeit(lambda: old_buf.__setitem__(slice(None), pack_v5_uint32(img)))
    t_pack_v6 = timeit(lambda: in_buf.__setitem__(slice(None), pack_v6_gray(img)))
    try:
        t_pack_v6f = timeit(lambda: pack_v6_fused(img, in_buf))
    except Exception as e:
        print("[!] 融合直写不可用（OpenCV 不接受 PynqBuffer）：%r" % (e,))
        t_pack_v6f = float("nan")

    # ---------- 3) cache 维护 ----------
    def cache_all():
        in_buf.flush()
        out_buf.flush()
        out_buf.invalidate()

    t_cache_v6 = timeit(cache_all)

    def cache_all_v5():
        old_buf.flush()
        out_buf.flush()
        out_buf.invalidate()

    t_cache_v5 = timeit(cache_all_v5)

    # ---------- 4) 端到端 ----------
    def e2e_v6(pack_fn):
        pack_fn(img)
        cache_all()
        run_ip(ctrl, param, in_buf, out_buf, MODE)
        out_buf.invalidate()

    def e2e_v5():
        old_buf[:] = pack_v5_uint32(img)
        cache_all_v5()
        run_ip(ctrl, param, old_buf, out_buf, MODE)
        out_buf.invalidate()

    t_e2e_v5 = timeit(e2e_v5, loops=10)
    t_e2e_v6 = timeit(lambda: e2e_v6(
        lambda im: in_buf.__setitem__(slice(None), pack_v6_gray(im))), loops=10)
    t_e2e_v6f = timeit(lambda: e2e_v6(lambda im: pack_v6_fused(im, in_buf)), loops=10)

    print("\n" + "=" * 72)
    print("v5 vs v6 同板 A/B（mode=%d, %dx%d, LOOPS=%d）" % (MODE, W, H, LOOPS))
    print("=" * 72)
    print("%-20s %10s %10s %10s" % ("分项", "v5", "v6", "v6融合"))
    print("-" * 72)
    print("%-20s %10.3f %10.3f %10.3f" % ("ARM 打包", t_pack_v5, t_pack_v6, t_pack_v6f))
    print("%-20s %10.3f %10.3f %10.3f" % ("cache 维护", t_cache_v5, t_cache_v6, t_cache_v6))
    print("%-20s %10.3f %10.3f %10.3f" % ("PL 核内", t_kernel, t_kernel, t_kernel))
    print("%-20s %10.3f %10.3f %10.3f" % ("端到端(实测)", t_e2e_v5, t_e2e_v6, t_e2e_v6f))
    print("-" * 72)
    print("端到端改善：v6 %.2fx / v6融合 %.2fx  （验收线 <8 ms）"
          % (t_e2e_v5 / t_e2e_v6, t_e2e_v5 / t_e2e_v6f))
    print("每像素访存量：v5 读 4B + 写 1B = 5 B/px -> v6 读 1B + 写 1B = 2 B/px")
    print("核内理论地板（100MHz, II=1）= %.3f ms，实测核内 %.3f ms（II≈%.2f）"
          % (PIX / 100e3, t_kernel, t_kernel * 100e3 / PIX))

    print("\n可直接贴报告的表格：")
    print("| 分项 | v5 (ms) | v6 (ms) | v6 融合直写 (ms) |")
    print("|---|---:|---:|---:|")
    print("| ARM 打包 | %.2f | %.2f | %.2f |" % (t_pack_v5, t_pack_v6, t_pack_v6f))
    print("| cache 维护 | %.2f | %.2f | %.2f |" % (t_cache_v5, t_cache_v6, t_cache_v6))
    print("| PL 核内 | %.2f | %.2f | %.2f |" % (t_kernel, t_kernel, t_kernel))
    print("| **端到端** | **%.2f** | **%.2f** | **%.2f** |"
          % (t_e2e_v5, t_e2e_v6, t_e2e_v6f))

    # ---------- 5) 附加实验：non-cacheable 缓冲 ----------
    # cacheable=True 时 ARM 写 DMA 缓冲会 write-allocate（先把 64B 读进来再写），
    # 对"只写不读"的输入缓冲是纯浪费。non-cacheable 直接流式写 + flush 变空操作。
    print("\n" + "=" * 72)
    print("附加实验：缓冲改成 non-cacheable（ARM 写入不再 write-allocate）")
    print("=" * 72)
    try:
        in_nc = allocate(shape=(PIX,), dtype=np.uint8, cacheable=False)
        out_nc = allocate(shape=(PIX,), dtype=np.uint8, cacheable=False)
    except Exception as e:
        print("[!] 无法分配 non-cacheable 缓冲（PYNQ 版本不支持）：%r" % (e,))
        return

    print("in_nc.cacheable = %s, out_nc.cacheable = %s"
          % (in_nc.cacheable, out_nc.cacheable))

    t_pack_nc = timeit(lambda: pack_v6_fused(img, in_nc))

    def cache_nc():
        in_nc.flush()
        out_nc.flush()
        out_nc.invalidate()

    t_cache_nc = timeit(cache_nc)
    t_kernel_nc = timeit(lambda: run_ip(ctrl, param, in_nc, out_nc, MODE))

    def e2e_nc():
        pack_v6_fused(img, in_nc)
        cache_nc()
        run_ip(ctrl, param, in_nc, out_nc, MODE)
        out_nc.invalidate()

    t_e2e_nc = timeit(e2e_nc, loops=10)

    print("\n%-20s %14s %14s %10s"
          % ("分项", "融合+cacheable", "融合+non-cache", "再省"))
    print("-" * 72)
    for name, a, b in (("ARM 打包", t_pack_v6f, t_pack_nc),
                       ("cache 维护", t_cache_v6, t_cache_nc),
                       ("PL 核内", t_kernel, t_kernel_nc),
                       ("端到端(实测)", t_e2e_v6f, t_e2e_nc)):
        print("%-20s %14.3f %14.3f %9.2fx" % (name, a, b, a / b))
    print("-" * 72)
    print("相对 v5 端到端：融合+cacheable %.2fx / 融合+non-cache %.2fx  （验收线 <8 ms）"
          % (t_e2e_v5 / t_e2e_v6f, t_e2e_v5 / t_e2e_nc))
    print("最小可达端到端（核内地板 3.072 + 当前最优打包 %.3f）= %.3f ms"
          % (min(t_pack_v6f, t_pack_nc), PIX / 100e3 + min(t_pack_v6f, t_pack_nc)))


if __name__ == "__main__":
    main()
