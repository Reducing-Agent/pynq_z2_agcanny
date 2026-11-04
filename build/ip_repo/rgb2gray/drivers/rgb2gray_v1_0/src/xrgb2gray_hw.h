// ==============================================================
// Vitis HLS - High-Level Synthesis from C, C++ and OpenCL v2026.1 (64-bit)
// Tool Version Limit: 2026.06
// Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
// Copyright 2022-2026 Advanced Micro Devices, Inc. All Rights Reserved.
// 
// ==============================================================
// control
// 0x00 : Control signals
//        bit 0  - ap_start (Read/Write/COH)
//        bit 1  - ap_done (Read/COR)
//        bit 2  - ap_idle (Read)
//        bit 3  - ap_ready (Read/COR)
//        bit 7  - auto_restart (Read/Write)
//        bit 9  - interrupt (Read)
//        others - reserved
// 0x04 : Global Interrupt Enable Register
//        bit 0  - Global Interrupt Enable (Read/Write)
//        others - reserved
// 0x08 : IP Interrupt Enable Register (Read/Write)
//        bit 0 - enable ap_done interrupt (Read/Write)
//        bit 1 - enable ap_ready interrupt (Read/Write)
//        others - reserved
// 0x0c : IP Interrupt Status Register (Read/TOW)
//        bit 0 - ap_done (Read/TOW)
//        bit 1 - ap_ready (Read/TOW)
//        others - reserved
// 0x10 : Data signal of ap_return
//        bit 31~0 - ap_return[31:0] (Read)
// 0x18 : Data signal of width
//        bit 31~0 - width[31:0] (Read/Write)
// 0x1c : reserved
// 0x20 : Data signal of height
//        bit 31~0 - height[31:0] (Read/Write)
// 0x24 : reserved
// 0x28 : Data signal of mode
//        bit 31~0 - mode[31:0] (Read/Write)
// 0x2c : reserved
// 0x30 : Data signal of gain
//        bit 31~0 - gain[31:0] (Read/Write)
// 0x34 : reserved
// 0x38 : Data signal of thresh
//        bit 31~0 - thresh[31:0] (Read/Write)
// 0x3c : reserved
// (SC = Self Clear, COR = Clear on Read, TOW = Toggle on Write, COH = Clear on Handshake)

#define XRGB2GRAY_CONTROL_ADDR_AP_CTRL     0x00
#define XRGB2GRAY_CONTROL_ADDR_GIE         0x04
#define XRGB2GRAY_CONTROL_ADDR_IER         0x08
#define XRGB2GRAY_CONTROL_ADDR_ISR         0x0c
#define XRGB2GRAY_CONTROL_ADDR_AP_RETURN   0x10
#define XRGB2GRAY_CONTROL_BITS_AP_RETURN   32
#define XRGB2GRAY_CONTROL_ADDR_WIDTH_DATA  0x18
#define XRGB2GRAY_CONTROL_BITS_WIDTH_DATA  32
#define XRGB2GRAY_CONTROL_ADDR_HEIGHT_DATA 0x20
#define XRGB2GRAY_CONTROL_BITS_HEIGHT_DATA 32
#define XRGB2GRAY_CONTROL_ADDR_MODE_DATA   0x28
#define XRGB2GRAY_CONTROL_BITS_MODE_DATA   32
#define XRGB2GRAY_CONTROL_ADDR_GAIN_DATA   0x30
#define XRGB2GRAY_CONTROL_BITS_GAIN_DATA   32
#define XRGB2GRAY_CONTROL_ADDR_THRESH_DATA 0x38
#define XRGB2GRAY_CONTROL_BITS_THRESH_DATA 32

// control_r
// 0x00 : reserved
// 0x04 : reserved
// 0x08 : reserved
// 0x0c : reserved
// 0x10 : Data signal of in_r
//        bit 31~0 - in_r[31:0] (Read/Write)
// 0x14 : Data signal of in_r
//        bit 31~0 - in_r[63:32] (Read/Write)
// 0x18 : reserved
// 0x1c : Data signal of out_r
//        bit 31~0 - out_r[31:0] (Read/Write)
// 0x20 : Data signal of out_r
//        bit 31~0 - out_r[63:32] (Read/Write)
// 0x24 : reserved
// (SC = Self Clear, COR = Clear on Read, TOW = Toggle on Write, COH = Clear on Handshake)

#define XRGB2GRAY_CONTROL_R_ADDR_IN_R_DATA  0x10
#define XRGB2GRAY_CONTROL_R_BITS_IN_R_DATA  64
#define XRGB2GRAY_CONTROL_R_ADDR_OUT_R_DATA 0x1c
#define XRGB2GRAY_CONTROL_R_BITS_OUT_R_DATA 64

