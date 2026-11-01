# data

本项目的测试数据**全部由脚本合成**，不需要外部图片文件。

- 合成函数位于 `src/ps/validate_v8.py`（`make_noise` / `make_clean_edges` /
  `edge_truth` 等），使用固定随机种子，任何机器上都能复现同一组数据。
- 黄金参考实现是 `src/ps/filters.py`（numpy 整幅向量化），
  PL 输出与之逐像素比对，最大差值 0。

这样设计是为了可复现性：评审无需下载任何外部数据集即可跑通全部验证。
