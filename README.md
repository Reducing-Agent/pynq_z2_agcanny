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

Two things to know before typing anything:

- The board runs from **one flat directory**. `hw_driver.py` looks for
  `rgb2gray.bit` next to itself, so the Python sources, the bitstream and the
  `.hwh` have to sit together. In this repository those halves are in
  `src/ps/` (sources) and `board/` (bitstream, manifest), so a fresh deploy
  needs one copy step.
- Port **9090 is ssh**, port **8080 is the web UI**. Easy to mix up.

### Fresh deploy from this repository

```bash
BOARD=/home/xilinx/jupyter_notebooks/rgb2gray_v82
mkdir -p "$BOARD"
cp src/ps/*.py src/ps/start_web.sh          "$BOARD/"
cp board/rgb2gray.bit board/rgb2gray.hwh    "$BOARD/"
cp board/FILES.txt    board/MD5SUMS.txt     "$BOARD/"
cp src/ps/BOARD_CHECKLIST_1030.md           "$BOARD/"
```

### Run it

```bash
source /usr/local/share/pynq-venv/bin/activate
cd /home/xilinx/jupyter_notebooks/rgb2gray_v82
python3 -u check_kit.py      # verifies every file and the bitstream MD5
./start_web.sh               # web UI on port 8080
```

Open `http://192.168.2.99:8080/`.

Use the full path in `cd`. A bare `cd rgb2gray_v82` only works if you happen to
already be in `/home/xilinx/jupyter_notebooks`.

`start_web.sh` sets `MIN_ROUGH=1.07` before launching. That threshold is the
shape criterion for a closed fist: the perimeter ratio of a fist measures 1.08
to 1.10, below the 1.14 tuned for an open palm, so "one finger" would otherwise
be rejected as not a hand. The equivalent manual command is:

```bash
MIN_ROUGH=1.07 python3 -u live_view_raw.py --auto
```

Do **not** run these with `sudo` — `env_reset` clears the PYNQ environment and
`Overlay` construction fails with "No Devices Found".

`start_web.sh` has no `--display` flag any more; the server always serves the
web UI. Passing `--display web` fails with "unrecognized arguments".

Bitstream identity must be checked by MD5; every build is exactly 4 045 771
bytes, so file size cannot distinguish versions.

| File | MD5 |
|---|---|
| `rgb2gray.bit` | `6b5fd2f907b7214a528530dcca6b418a` |
| `rgb2gray.hwh` | `9b7407a3772b4d2800d00b6d65ba751a` |

### The one file a fresh deploy will not have

`check_kit.py` compares against `FILES.txt`, which records the names as they
are on the board. One documentation file carries Chinese characters in the
delivered package (`README_上板说明_v82.md`); the submission rules require
ASCII-only names, so this repository carries it as `src/ps/README_board_v82.md`.
On a fresh deploy `check_kit.py` prints `(缺，不影响运行)` — "missing, does not
affect running" — for that one line. Nothing imports it. Everything else,
including every `.py`, the bitstream and the `.hwh`, must read `OK`.