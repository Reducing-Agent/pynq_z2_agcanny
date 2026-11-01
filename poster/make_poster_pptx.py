#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""生成一页英文 A4 竖版海报（PPTX）

    python make_poster_pptx.py

输出：AG_Canny_Poster_A4.pptx  (21.0 x 29.7 cm，一页)

官方要求（指南"六、其它"）：按 AMD 提供的 PPT 模板提供一页英文 PPT 海报，
  用 A4 纸彩色打印带到决赛现场，**不包含队伍信息**。
  因此本海报全部为英文，且**不出现队名 / 学校 / 成员姓名**。
"""
import os

from pptx import Presentation
from pptx.util import Cm, Pt
from pptx.dml.color import RGBColor
from pptx.enum.text import PP_ALIGN, MSO_ANCHOR

# ---------------------------------------------------------------- palette
INK = RGBColor(0x12, 0x16, 0x1C)
INK2 = RGBColor(0x3D, 0x46, 0x53)
ACCENT = RGBColor(0xC8, 0x10, 0x2E)     # AMD red
ACCENT2 = RGBColor(0x0B, 0x5F, 0xA5)
OK = RGBColor(0x0F, 0x7B, 0x3F)
PANEL = RGBColor(0xF4, 0xF6, 0xFA)
LINE = RGBColor(0xD8, 0xDE, 0xE8)
WHITE = RGBColor(0xFF, 0xFF, 0xFF)

STAGE_COLORS = [
    RGBColor(0xC8, 0x10, 0x2E),
    RGBColor(0xA3, 0x13, 0x2A),
    RGBColor(0x7D, 0x1A, 0x2B),
    RGBColor(0x5C, 0x1C, 0x2A),
]

W_CM, H_CM = 21.0, 29.7
M = Cm(1.15)          # margin
CW = W_CM - 2 * 1.15  # content width = 18.7 cm


def txbox(slide, x, y, w, h, wrap=True):
    tb = slide.shapes.add_textbox(Cm(x), Cm(y), Cm(w), Cm(h))
    tf = tb.text_frame
    tf.word_wrap = wrap
    tf.margin_left = tf.margin_right = Cm(0.05)
    tf.margin_top = tf.margin_bottom = 0
    return tf


def para(tf, text, size, color=INK, bold=False, first=False,
         space_after=2, line=1.0, align=PP_ALIGN.LEFT, italic=False):
    p = tf.paragraphs[0] if first else tf.add_paragraph()
    p.alignment = align
    p.space_after = Pt(space_after)
    p.line_spacing = line
    r = p.add_run()
    r.text = text
    r.font.size = Pt(size)
    r.font.bold = bold
    r.font.italic = italic
    r.font.color.rgb = color
    r.font.name = "Segoe UI"
    return p


def rect(slide, x, y, w, h, fill=None, line_col=None, line_w=0.75):
    from pptx.enum.shapes import MSO_SHAPE
    sh = slide.shapes.add_shape(MSO_SHAPE.ROUNDED_RECTANGLE, Cm(x), Cm(y), Cm(w), Cm(h))
    sh.adjustments[0] = 0.06
    if fill is None:
        sh.fill.background()
    else:
        sh.fill.solid()
        sh.fill.fore_color.rgb = fill
    if line_col is None:
        sh.line.fill.background()
    else:
        sh.line.color.rgb = line_col
        sh.line.width = Pt(line_w)
    sh.shadow.inherit = False
    sh.text_frame.word_wrap = True
    return sh


def section_title(slide, x, y, w, text):
    tf = txbox(slide, x, y, w, 0.55)
    para(tf, text.upper(), 9.5, ACCENT, bold=True, first=True, space_after=0)
    ln = rect(slide, x, y + 0.52, w, 0.035, fill=LINE)
    ln = rect(slide, x, y + 0.52, w * 0.22, 0.035, fill=ACCENT)


def kv_table(slide, x, y, w, rows, col0=0.62, fs=8.2, hdr_fs=7.6):
    """rows: [(k, v, highlight_bool)]"""
    rh = 0.42
    rect(slide, x, y, w, rh, fill=WHITE)
    tf0 = txbox(slide, x, y, w * col0, rh)
    tf0.vertical_anchor = MSO_ANCHOR.MIDDLE
    para(tf0, "METRIC", hdr_fs, INK2, bold=True, first=True, space_after=0)
    tf1 = txbox(slide, x + w * col0, y, w * (1 - col0), rh)
    tf1.vertical_anchor = MSO_ANCHOR.MIDDLE
    para(tf1, "VALUE", hdr_fs, INK2, bold=True, first=True, space_after=0,
         align=PP_ALIGN.RIGHT)
    for i, (k, v, hl) in enumerate(rows):
        yy = y + rh * (i + 1)
        if i % 2 == 0:
            rect(slide, x, yy, w, rh, fill=PANEL)
        tk = txbox(slide, x, yy, w * col0, rh)
        tk.vertical_anchor = MSO_ANCHOR.MIDDLE
        para(tk, k, fs, INK, bold=True, first=True, space_after=0)
        tv = txbox(slide, x + w * col0, yy, w * (1 - col0), rh)
        tv.vertical_anchor = MSO_ANCHOR.MIDDLE
        para(tv, v, fs, ACCENT if hl else INK2, bold=hl, first=True,
             space_after=0, align=PP_ALIGN.RIGHT)


def main():
    prs = Presentation()
    prs.slide_width = Cm(W_CM)
    prs.slide_height = Cm(H_CM)
    slide = prs.slides.add_slide(prs.slide_layouts[6])  # blank

    # ---------------------------------------------------------- header
    rect(slide, 0, 0, W_CM, 0.10, fill=ACCENT)

    tf = txbox(slide, M.cm, 0.45, CW, 0.4)
    para(tf, "AMD ADAPTIVE COMPUTING  ·  FPGA TRACK", 8, ACCENT, bold=True,
         first=True, space_after=0)

    tf = txbox(slide, M.cm, 0.85, CW, 2.5)
    p = para(tf, "AG-Canny: An ", 24, INK, bold=True, first=True, space_after=0,
             line=1.05)
    r = p.add_run(); r.text = "Illumination-Adaptive "
    r.font.size = Pt(24); r.font.bold = True; r.font.color.rgb = ACCENT
    r.font.name = "Segoe UI"
    r = p.add_run(); r.text = "Real-Time"
    r.font.size = Pt(24); r.font.bold = True; r.font.color.rgb = INK
    r.font.name = "Segoe UI"
    para(tf, "Canny Vision Front-End on PYNQ-Z2", 24, INK, bold=True,
         space_after=0, line=1.05)

    tf = txbox(slide, M.cm, 3.30, CW - 2.0, 1.05)
    para(tf,
         "A four-stage streaming pipeline that turns a raw 640×480 grayscale stream into a "
         "per-pixel edge feature vector — and re-tunes its own gain and Canny thresholds "
         "every frame, so edge quality holds up when the lights change.",
         9.6, INK2, first=True, space_after=0, line=1.25)

    # chips
    chips = ["640×480 @ 30 fps", "3.258 ms kernel", "94% of II=1 roof",
             "8 626 LUT (16.2%)", "Zynq-7020 · 100 MHz"]
    cx = M.cm
    for i, c in enumerate(chips):
        w = 3.55 if i < 3 else 3.9
        sh = rect(slide, cx, 4.45, w, 0.52,
                  fill=ACCENT if i == 0 else PANEL,
                  line_col=ACCENT if i == 0 else LINE)
        t = sh.text_frame
        t.vertical_anchor = MSO_ANCHOR.MIDDLE
        para(t, c, 7.4, WHITE if i == 0 else INK2, bold=True, first=True,
             space_after=0, align=PP_ALIGN.CENTER)
        cx += w + 0.16

    rect(slide, M.cm, 5.15, CW, 0.06, fill=ACCENT)

    # ---------------------------------------------------------- pipeline (left)
    Lx, Ly = M.cm, 5.55
    LW = 11.0
    section_title(slide, Lx, Ly, LW, "Fused Streaming Pipeline — One Pass, No Round-Trips")

    stages = [
        ("1", "AGC + Separable 5×5 Gaussian",
         "Q8.8 gain → horizontal 5-tap → 4 line buffers → vertical 5-tap. Each stage shifts by 4.",
         "op 9\nDENOISED"),
        ("2", "Sobel + Direction Quantisation",
         "9 Sobel gradients share one 5×5 window — no extra buffering, no extra DDR traffic.",
         None),
        ("3", "Non-Maximum Suppression",
         "Compare against the two neighbours along the quantised gradient direction.",
         "op 11\nNMS MAG"),
        ("4", "Double Threshold + Hysteresis",
         "Thresholds derived from measured noise, not hand-picked constants.",
         "op 10\n0/255 EDGE"),
    ]
    sy = Ly + 0.78
    for i, (n, t, d, tap) in enumerate(stages):
        sh = rect(slide, Lx, sy, LW, 1.16, fill=WHITE, line_col=LINE)
        nb = rect(slide, Lx, sy, 1.05, 1.16, fill=STAGE_COLORS[i])
        nt = nb.text_frame
        nt.vertical_anchor = MSO_ANCHOR.MIDDLE
        para(nt, n, 15, WHITE, bold=True, first=True, space_after=0,
             align=PP_ALIGN.CENTER)

        tfx = txbox(slide, Lx + 1.22, sy + 0.13, LW - (3.0 if tap else 1.4), 1.0)
        para(tfx, t, 9.0, INK, bold=True, first=True, space_after=1, line=1.0)
        para(tfx, d, 7.6, INK2, space_after=0, line=1.18)

        if tap:
            tb = rect(slide, Lx + LW - 2.55, sy + 0.22, 2.35, 0.72, fill=PANEL,
                      line_col=LINE)
            tt = tb.text_frame
            tt.vertical_anchor = MSO_ANCHOR.MIDDLE
            para(tt, tap, 7.0, ACCENT2, bold=True, first=True, space_after=0,
                 align=PP_ALIGN.CENTER, line=0.95)

        if i < 3:
            ar = txbox(slide, Lx, sy + 1.16, LW, 0.32)
            para(ar, "▼", 8, LINE, first=True, space_after=0, align=PP_ALIGN.CENTER)
        sy += 1.48

    tf = txbox(slide, Lx, sy + 0.05, LW, 1.5)
    para(tf, "op 9 / 10 / 11 are three taps of the same pipeline, not three parallel "
             "operators — all four stages always run; the op code only selects which stage "
             "is written back. Stage outputs are handed to the next stage through on-chip "
             "line buffers, so a pixel visits DDR exactly once.",
         7.7, INK2, first=True, space_after=0, line=1.22)

    # ---------------------------------------------------------- why (right)
    Rx = Lx + LW + 0.42
    RW = CW - LW - 0.42
    section_title(slide, Rx, Ly, RW, "Why This Is Hard")
    why = [
        ("Fixed thresholds fail in the dark.",
         "A Canny threshold tuned at desk lighting collapses under low illumination: "
         "noise crosses it, real edges do not."),
        ("Fixed thresholds fail under noise too.",
         "Lowering them to keep dim edges floods the output with false edges."),
        ("Naïve HLS is slower than ARM.",
         "Our first version read the 3×3 window from DDR nine times per pixel and "
         "came out at 0.76× — slower than NumPy."),
        ("Green tools can still be wrong.",
         "Synthesis reported II=1 and zero warnings while the board measured 20.57 ms "
         "— 6.7× off the roof."),
    ]
    wy = Ly + 0.78
    for t, d in why:
        rect(slide, Rx, wy, 0.10, 1.16, fill=ACCENT)
        tf = txbox(slide, Rx + 0.26, wy, RW - 0.26, 1.16)
        para(tf, t, 8.6, INK, bold=True, first=True, space_after=1, line=1.1)
        para(tf, d, 7.5, INK2, space_after=0, line=1.15)
        wy += 1.30

    # ---------------------------------------------------------- KPI band
    Ky = 12.55
    kpis = [
        ("3.258", "ms", "Kernel latency\n640×480 @ 100 MHz", OK),
        ("1.06", "cyc/px", "II=1 roof is 3.072 ms\n→ 94% attained", OK),
        ("6.31", "×", "20.57 ms → 3.258 ms\nafter burst fix", ACCENT2),
        ("73", "fps", "Full system incl.\nARM packing", ACCENT2),
        ("−94.9", "%", "False edges under\nσ=8 noise (5797→297)", ACCENT),
        ("16.2", "%", "LUT utilisation\n8 626 / 53 200", ACCENT),
    ]
    bw = (CW - 5 * 0.20) / 6
    kx = M.cm
    for v, unit, label, col in kpis:
        rect(slide, kx, Ky, bw, 1.72, fill=WHITE, line_col=LINE)
        rect(slide, kx, Ky, bw, 0.11, fill=col)
        tf = txbox(slide, kx, Ky + 0.28, bw, 0.72)
        p = para(tf, v, 20, INK, bold=True, first=True, space_after=0,
                 align=PP_ALIGN.CENTER)
        r = p.add_run(); r.text = " " + unit
        r.font.size = Pt(9.5); r.font.bold = True; r.font.color.rgb = INK
        r.font.name = "Segoe UI"
        tf = txbox(slide, kx, Ky + 1.02, bw, 0.66)
        para(tf, label, 6.9, INK2, first=True, space_after=0,
             align=PP_ALIGN.CENTER, line=1.12)
        kx += bw + 0.20

    # ---------------------------------------------------------- bottom three
    By = 14.65
    bw2 = (CW - 2 * 0.42) / 3

    # col 1 — adaptive threshold
    section_title(slide, M.cm, By, bw2, "Noise-Aware Adaptive Threshold")
    tf = txbox(slide, M.cm, By + 0.78, bw2, 1.0)
    para(tf, "PL measures per-pixel noise with an Immerkær operator and returns σ through "
             "an ap_return register; ARM converts it to Canny thresholds once per frame.",
         8.0, INK2, first=True, space_after=0, line=1.2)
    kv_table(slide, M.cm, By + 1.85, bw2, [
        ("HIGH ≈ 7.56σ", "sweet spot", True),
        ("3σ", "too loose", False),
        ("10σ", "edges lost", False),
        ("16σ (naïve)", "no edge survives", False),
    ], col0=0.58)
    tf = txbox(slide, M.cm, By + 4.15, bw2, 1.6)
    para(tf, "The 7.56 coefficient folds in the 0.2734 attenuation of stage 1, because σ is "
             "measured before denoising while the Sobel magnitude is computed after it. "
             "Ignoring that factor gives 16σ and kills every true edge.",
         7.5, INK2, first=True, space_after=0, line=1.2)

    # col 2 — burst fix
    x2 = M.cm + bw2 + 0.42
    section_title(slide, x2, By, bw2, "Where the 6.31× Came From")
    tf = txbox(slide, x2, By + 0.78, bw2, 0.9)
    para(tf, "Root cause: the DDR write sat inside an if. A conditional write defeats AXI "
             "burst inference and degrades to one handshaked transfer per pixel.",
         8.0, INK2, first=True, space_after=0, line=1.2)
    kv_table(slide, x2, By + 1.85, bw2, [
        ("op 0 (unconditional)", "1.05", False),
        ("op 1–8 (if x≥2)", "6.70", True),
        ("op 9–11 (if y≥8 && x≥8)", "6.70", True),
        ("op 1–11 after fix", "1.06", True),
    ], col0=0.72)
    tf = txbox(slide, x2, By + 3.75, bw2, 1.0)
    para(tf, "Fix: results land in an on-chip output line buffer outr[2][MAX_W] (ping-pong "
             "by row parity), then DDR is written unconditionally and contiguously. "
             "Cost: 4 RAMB18.",
         7.5, INK2, first=True, space_after=0, line=1.2)
    tf = txbox(slide, x2, By + 4.75, bw2, 1.1)
    para(tf, "Decisive counter-example. Our first hypothesis blamed the σ accumulator. "
             "It was wrong: op 9–11 have no σ accumulator at all, yet were exactly as slow. "
             "One counter-example beat a three-sample comparison that looked perfectly consistent.",
         7.5, INK2, first=True, space_after=0, line=1.2)

    # col 3 — measured
    x3 = x2 + bw2 + 0.42
    section_title(slide, x3, By, bw2, "Measured on Board")
    kv_table(slide, x3, By + 0.72, bw2, [
        ("Device", "xc7z020clg400-1", False),
        ("Clock", "100 MHz", False),
        ("LUT", "8 626 (16.21%)", False),
        ("Flip-flops", "10 331 (9.71%)", False),
        ("BRAM (RAMB18)", "15 (5.36%)", False),
        ("DSP48", "32 (14.55%)", False),
        ("WNS", "+0.435 ns", True),
        ("Failing endpoints", "0", True),
        ("Loop II", "1", False),
        ("Kernel + cache", "3.74 ms", False),
    ], col0=0.60, fs=7.8)
    tf = txbox(slide, x3, By + 5.35, bw2, 1.2)
    para(tf, "Throughput is decoupled from operator complexity. Grayscale pass-through takes "
             "3.222 ms; the full neighbourhood pipeline takes 3.258 ms — a 1% gap. "
             "Adding operators costs LUT, not time: the pipeline is memory-bound, not compute-bound.",
         7.5, INK2, first=True, space_after=0, line=1.2)

    # ---------------------------------------------------------- last band
    Ly2 = 21.4
    rect(slide, M.cm, Ly2, CW, 0.03, fill=LINE)
    b2w = (CW - 0.42) / 2

    section_title(slide, M.cm, Ly2 + 0.22, b2w, "Cost per Pixel Converges to the Roof")
    tf = txbox(slide, M.cm, Ly2 + 0.95, b2w, 0.4)
    para(tf, "64×64   1.45      160×120   1.22      320×240   1.08      640×480   1.06 cyc/px",
         8.6, INK, bold=True, first=True, space_after=0)
    tf = txbox(slide, M.cm, Ly2 + 1.45, b2w, 0.9)
    para(tf, "Fixed overhead is amortised as frames grow — the healthy shape. Twelve modes are "
             "supported: nine single-pass 3×3 operators plus the three pipeline taps.",
         7.6, INK2, first=True, space_after=0, line=1.2)

    xb = M.cm + b2w + 0.42
    section_title(slide, xb, Ly2 + 0.22, b2w, "Reproducible End to End")
    reps = [
        "Command-line only: v++ -c → --impl → --package → Vivado Tcl — no GUI clicks, CI-ready.",
        "Bitstream versioning by MD5: every .bit for this device is exactly 4 045 771 bytes, so file size cannot tell versions apart.",
        "Python A/B mirror before synthesis: 702 cases compared byte-for-byte, so boundary bugs never reach the board.",
        "Reusable skill package: 17 documented failure cases, each with trigger, evidence and boundary of validity.",
    ]
    ry = Ly2 + 0.90
    for t in reps:
        rect(slide, xb, ry + 0.10, 0.09, 0.42, fill=ACCENT)
        tf = txbox(slide, xb + 0.22, ry, b2w - 0.22, 0.55)
        para(tf, t, 7.5, INK2, first=True, space_after=0, line=1.16)
        ry += 0.54

    # ---------------------------------------------------------- footer
    rect(slide, M.cm, 27.55, CW, 0.03, fill=LINE)
    rect(slide, M.cm, 27.72, 0.28, 0.28, fill=ACCENT)
    tf = txbox(slide, M.cm + 0.42, 27.72, 7.0, 0.35)
    para(tf, "AMD FPGA TRACK · ADAPTIVE COMPUTING", 7.0, INK2, bold=True,
         first=True, space_after=0)
    tf = txbox(slide, W_CM - M.cm - 11.5, 27.72, 11.5, 0.35)
    para(tf, "Vitis / Vivado 2026.1 · PYNQ-Z2 (Zynq-7020) · Vitis HLS C++ → IP → Block Design",
         7.0, INK2, first=True, space_after=0, align=PP_ALIGN.RIGHT)

    out = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                       "AG_Canny_Poster_A4.pptx")
    prs.save(out)
    print("saved:", out, os.path.getsize(out), "bytes")
    return out


if __name__ == "__main__":
    main()
