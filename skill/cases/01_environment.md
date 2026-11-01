# 环境类：器件许可 / 板卡文件 / 中文路径

## 案例 1 · 器件"消失"：从"安装缺失"到"许可过滤"

- 现象：Vivado 器件列表找不到 `xc7z020`，提示 `no licensed parts`。
- 初始错误假设：安装包未包含 Zynq-7000 器件数据。
- 纠正：曾生成 `Vivado_Alveo_Package` 许可并存为 `D:\Xilinx .lic`，
  且设了环境变量 `XILINXD_LICENSE_FILE` 指向它。ALVEO 许可**只解锁 Alveo 加速卡**，
  其余器件被过滤隐藏——`licensed` 是字面意思，器件数据一直完好。
- 处理：删除该用户环境变量，改领 Vivado Basic Tier + Vitis HLS（No Charge）许可。
- **可复用结论**：工具报错的**字面表述往往就是真实原因**，
  先别怀疑安装完整性。遇到"器件/功能不见"，第一件事是查 `XILINXD_LICENSE_FILE`。

## 案例 2 · 板卡文件搜不到

- 现象：Vivado 2026.1 的 XHUB 中搜不到 PYNQ-Z2。
- 推断与验证：2026.1 安装包**不随附任何 board_files**；XHUB 只同步 `Xilinx`
  官方目录，而 PYNQ-Z2 属第三方厂商 TUL，
  位于 `XilinxBoardStore/boards/TUL/pynq-z2/A.0`。
- 处理：手动下载 5 个板卡文件部署到 `<vivado>/board_files/pynq-z2/A.0/`，
  用 `get_board_parts` 验证出现 `tul.com.tw:pynq-z2:part0:1.0`。
- 兜底：板卡文件是**非阻塞项**，不识别时直接用裸芯片 `xc7z020clg400-1` 手工配 PS。

## 案例 9 · 中文用户名导致的图像读取失败

- 现象：合成脚本报 `can't open/read file`，但文件确实存在。
- 根因：Windows 上 OpenCV 的 `imread` **不支持非 ASCII 路径**，而用户目录是中文。
- 修复：改用 `np.fromfile` + `cv2.imdecode`；
  `VideoWriter` 同理，先写临时 ASCII 路径再移动。
- **可复用结论**：中文 Windows 环境下，凡是"文件明明存在却读不了"，
  先怀疑路径编码，不要怀疑文件本身。
