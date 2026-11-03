# AG-Canny — A Noise-Aware Adaptive Real-Time Vision Front-End on PYNQ-Z2

**Registered competition title:** Gesture-Interactive Image Processor

> A note on the title. The registered name puts *gesture* first, but the
> subject of this work is the **image processor**. Gesture is only one of
> three control entries — web UI buttons, keyboard shortcuts, and hand
> gestures — and all three do exactly the same thing: write one AXI-Lite
> register. The engineering contribution is the PL pipeline itself, which
> this repository documents in full.

---

## What it is

A real-time vision front-end on AMD Zynq-7000 (PYNQ-Z2, `xc7z020clg400-1`).

A USB camera feeds 640x480 grayscale frames. On the PL side, a Vitis-HLS IP
reads directly from DDR through its own AXI master port and completes a
**four-stage cascaded pipeline in a single memory pass**:

```
5x5 separable Gaussian denoise  ->  Sobel gradient + direction quantisation
                                ->  non-maximum suppression
                                ->  dual-threshold hysteresis
```

Each pixel is read once and written once. Intermediate results never return
to DDR. The IP also estimates the per-frame noise level (sigma) in hardware;
the PS uses it to update gain and the Canny thresholds every frame, forming a
feed-forward lighting-adaptive loop with no human intervention.

**12 processing modes** are switchable at run time by writing one register —
no resynthesis, no bitstream reload.

| Mode | Output |
|---:|---|
| 0-8 | single 3x3 operators: gray / Sobel / emboss / Gaussian / sharpen / Laplacian / binary / mean / direction |
| 9 | stage 1 tap — denoised image |
| 10 | stage 4 tap — full Canny edges |
| 11 | stage 3 tap — thinned gradient magnitude |

Modes 9, 10, 11 are **not three parallel operators**. They are three taps on
one pipeline: all four stages always run, and `mode` only selects which stage
writes back. Switching between them costs nothing.

---

## Measured results (on-board, 640x480, FCLK 100 MHz)

| Metric | Value |
|---|---|
| Kernel time, modes 0-8 | 3.253 - 3.261 ms (1.06 cycles/pixel) |
| Kernel time, modes 9-11 (full four-stage fusion) | 3.655 - 3.661 ms (1.19 cycles/pixel) |
| II=1 theoretical floor | 3.072 ms |
| Achieved fraction of the floor | 94.3% (single operator) / 84.0% (fusion) |
| End-to-end including ARM packing | ~13.6 ms (~73 fps) |
| LUT / FF / BRAM / DSP (full project) | 8626 (16.21%) / 10331 (9.71%) / 15 / 32 |
| WNS / WHS | +0.435 ns / +0.020 ns, 0 failing endpoints |

At **1.19 cycles per pixel** the pipeline retires one pixel every clock cycle
while executing four cascaded stages. A fifth stage would add latency, not
reduce throughput.

---

## Repository layout

```
src/hls/       Vitis-HLS C++ source and the one-command build script
src/ps/        PYNQ drivers, live view server, gesture modules, benchmarks
sim/           C testbench and Python golden-reference cross-checks
build/         Vivado block-design script and HLS build configuration
board/         on-board run instructions, file manifest, checksums
data/          test images
reproduce/     step-by-step reproduction scripts
skill/         reusable prompt workflows, templates, check scripts, pitfall list
poster/        A4 English poster (HTML + PPTX)
report/        full design report
```

Every path and filename is ASCII-only (lowercase letters, digits, `_`, `-`),
as required by the competition submission rules.

---

## Quick start (on the board)

```bash
source /usr/local/share/pynq-venv/bin/activate
cd rgb2gray_v82
python3 -u check_kit.py      # verify bitstream checksums
./start_web.sh               # launch the web UI, served on port 8080
```

`start_web.sh` sets `MIN_ROUGH=1.07` before launching. That threshold is the
shape criterion for a closed fist: perimeter ratio of a fist measures 1.08 to
1.10, below the 1.14 tuned for an open palm, so "one finger" would otherwise
be rejected as not a hand. The equivalent manual command is:

```bash
MIN_ROUGH=1.07 python3 -u live_view_raw.py --auto
```

Do **not** run these with `sudo` — `env_reset` clears the PYNQ environment and
`Overlay` construction fails with "No Devices Found".

Bitstream identity must be checked by MD5; every build is exactly 4 045 771
bytes, so file size cannot distinguish versions.

| File | MD5 |
|---|---|
| `rgb2gray.bit` | `6b5fd2f907b7214a528530dcca6b418a` |
| `rgb2gray.hwh` | `9b7407a3772b4d2800d00b6d65ba751a` |

## AGC compare view

Click **AGC compare ON** in the top bar (or press `v`) to switch the page from
the single-pane view to a 2x2 layout:

| | left | right |
|---|---|---|
| top | 1. raw camera frame | 2. standard Canny: same PL, AGC off, fixed 60/30 |
| bottom | 3. AG-Canny: AGC on, thresholds from measured sigma | 4. runtime monitor |

Both Canny panes run on the same FPGA with the same bitstream. The only
difference is whether light adaptation is on, which is the point of the
comparison. While the compare view is active the operator is locked to Canny,
so neither a button nor a gesture can switch the filter underneath you.

Two things to know before filming it:

- AGC only brightens, it never dims (`AGC_TARGET_MU = 110`). In a bright room
  the measured mean stays above 110, the gain is clamped to 1.00x, and both
  panes come out looking the same. Dim the lights or half-cover the lens to
  push the mean below 110.
- `hw_driver` reports sigma only for op 1..8; the 5x5 Canny path leaves
  `ap_return` at 0. The compare view inserts an extra op-1 pass every
  `SIGMA_EVERY` frames (default 15) purely to measure noise, and recomputes
  pane 2 every `STD_EVERY` frames (default 5). Both are environment variables:
  smaller values make the panes more responsive and cost frame rate, e.g.
  `STD_EVERY=2 ./start_web.sh`.

---

## Reproduction

```bash
python3 -u validate_v8.py        # 6 stages, all green: 180 pixel-exact
                                 # comparisons, 72 sigma read-back checks,
                                 # threshold sweeps, and the innovation evidence
python3 -u bench_fusion_5pass.py  # pass-count linearity experiment
```

See `reproduce/` for the full ordered sequence. The recorded numbers are in
`report/design_report.md` (section 4).

This repository carries the released design only: one C++ source, one header,
one build script, one testbench. Intermediate iterations are kept out on
purpose so that every file here is load-bearing.

---

## Control path and data path are decoupled

Gesture recognition runs on the PS and consumes the **raw colour camera
frame**, not the PL edge map. Three reasons:

1. The PL output is single-channel — chroma is gone, and skin detection needs it.
2. In an edge map, wall texture and a hand are not separable.
3. A control path must not depend on the state of the thing it controls.

The PL result is only displayed; the gesture result only writes the `mode`
register. The two paths do not feed each other.

---

## Scope

Implemented: the HLS IP, the fused pipeline, the adaptive sigma loop, the 12
run-time modes, the PYNQ driver, the web UI, and PS-side gesture control.

Not implemented, and therefore not claimed anywhere: AXI DMA IP, HDMI output,
on-board push buttons. Data moves through the IP's own AXI master ports;
display is over Ethernet to a browser.
