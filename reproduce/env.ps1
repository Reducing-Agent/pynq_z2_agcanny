# =============================================================================
# env.ps1 —— 复现环境（Windows / PowerShell）
#
#   . .\env.ps1
#
# 注意是「点号 + 空格」执行（dot-source），否则变量只在子进程里生效，当前 shell 拿不到。
# =============================================================================

$VITIS_ROOT  = 'D:\AMDDesignTools\2026.1\Vitis'
$VIVADO_ROOT = 'D:\AMDDesignTools\2026.1\Vivado'

$env:XILINX_VITIS  = $VITIS_ROOT
$env:XILINX_VIVADO = $VIVADO_ROOT
$env:RDI_DATADIR   = "$VITIS_ROOT\data"
$env:HDI_APPROOT   = $VITIS_ROOT

$env:PATH = (
    "$VITIS_ROOT\lib\win64.o",
    "$VIVADO_ROOT\bin\unwrapped\win64.o",
    "$VITIS_ROOT\bin\unwrapped\win64.o",
    "$VITIS_ROOT\tps\mingw\10.0.0\win64.o\nt\bin",
    "$VIVADO_ROOT\lib\win64.o",
    $env:PATH
) -join ';'

# ---------------------------------------------------------------------------
# 许可检查（案例 1）
# ---------------------------------------------------------------------------
# 如果设置了 XILINXD_LICENSE_FILE 指向 Alveo 专用许可，Vivado 会把 Zynq-7000
# 器件**过滤隐藏**，报 "no licensed parts" —— 字面意思就是"没有许可的器件"，
# 不是安装包缺器件。这里主动清掉，避免踩坑。
if ($env:XILINXD_LICENSE_FILE) {
    Write-Host "!! 清理 XILINXD_LICENSE_FILE（原值：$env:XILINXD_LICENSE_FILE）" -ForegroundColor Yellow
    Remove-Item Env:\XILINXD_LICENSE_FILE
}

Write-Host "VITIS  = $env:XILINX_VITIS"
Write-Host "VIVADO = $env:XILINX_VIVADO"
Write-Host "PATH 已前置工具链目录"
