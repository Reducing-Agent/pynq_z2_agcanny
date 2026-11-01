# =============================================================================
# run_hls.ps1 —— 命令行跑 Vitis HLS（不用 GUI，可纳入 CI）
#
#   . .\env.ps1          # 先设环境
#   .\run_hls.ps1        # 全部四步：compile / csim / cosim / package
#   .\run_hls.ps1 -SkipCosim   # 跳过 cosim（RTL 仿真慢，改算法时用它快速迭代）
#
# 必须用 bin\*.bat 包装器，不能调 bin\unwrapped\win64.o\*.exe
#   （后者缺 CRT DLL，会报 api-ms-win-crt-math-l1-1-0.dll 缺失）
# 本环境禁止在脚本里调 cmd.exe，直接 & 'xxx.bat' 即可
# =============================================================================
param(
    [switch]$SkipCosim,
    [switch]$SkipCsim,
    [string]$Proj = 'D:\vitis\RGB2gray',
    [string]$Cfg  = 'RGB2gray.cfg',
    [string]$Work = 'rgb2gray'
)

$ErrorActionPreference = 'Stop'
$BIN = Join-Path $env:XILINX_VITIS 'bin'

if (-not $env:XILINX_VITIS) {
    Write-Host "!! 环境没设。先执行： . .\env.ps1" -ForegroundColor Red
    exit 1
}

Push-Location $Proj
try {
    $cfgPath = Join-Path $Proj $Cfg

    Write-Host "=== [1/4] compile (C synth) ===" -ForegroundColor Cyan
    & "$BIN\v++.bat" -c --mode hls --config $cfgPath --work_dir $Work
    if ($LASTEXITCODE -ne 0) { throw "compile failed" }

    if (-not $SkipCsim) {
        Write-Host "=== [2/4] csim ===" -ForegroundColor Cyan
        & "$BIN\vitis-run.bat" --mode hls --csim --config $cfgPath --work_dir $Work
        if ($LASTEXITCODE -ne 0) { throw "csim failed" }
    }

    if (-not $SkipCosim) {
        Write-Host "=== [3/4] cosim (RTL) ===" -ForegroundColor Cyan
        & "$BIN\vitis-run.bat" --mode hls --cosim --config $cfgPath --work_dir $Work
        if ($LASTEXITCODE -ne 0) { throw "cosim failed" }
    }

    Write-Host "=== [4/4] package ===" -ForegroundColor Cyan
    & "$BIN\vitis-run.bat" --mode hls --package --config $cfgPath --work_dir $Work
    if ($LASTEXITCODE -ne 0) { throw "package failed" }

    Write-Host ""
    Write-Host "产物： $(Join-Path $Proj (Join-Path $Work 'rgb2gray.zip'))" -ForegroundColor Green
    Write-Host ""
    Write-Host "跑完必查三件事（见 README.md）：" -ForegroundColor Yellow
    Write-Host "   1. csynth 报告的 M_AXI Burst Information -> 主循环必须 Inferred"
    Write-Host "   2. 主循环 II 必须为 1"
    Write-Host "   3. grep 生成的 xrgb2gray_hw.h 里 ADDR_*_DATA（寄存器偏移可能整体移动）"
}
finally {
    Pop-Location
}
