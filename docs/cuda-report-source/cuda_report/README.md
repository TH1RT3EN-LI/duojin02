# CUDA 算法重构分析报告源文件

本报告以“相同在线时延下利用 GPU 计算与显存换取更好的算法效果”为主线。
完整报告为同级目录的 duojin02_cuda_acceleration_reconstruction_report.pdf。

## 重建 PDF

依赖：XeLaTeX、ctex、AMS 数学宏包、TikZ、Noto CJK 字体、URW Base35 字体。
运行 bash build.sh，三遍构建以解析目录、公式与文献交叉引用。
字体路径与原始论文所用 Linux 环境一致，可按本机字体安装调整 main.tex。

## 数学核验

Python 3 + NumPy，运行 python verify_math.py。
固定种子 20261015，结果输出为 math_validation.json。
这只是独立数学数值检查，不是 CUDA 性能或机器人验证。

## 证据

provenance.json 记录输入压缩包散列、原稿的 101 项源码核对和外部主要来源。
pdf_validation.json 记录 PDF 构建与渲染检查。
文献链接直接写在 references.tex 中。
原项目源码、配置及输入 ZIP 均未修改。

所有尚未实测的新方案、速度预测与质量收益都在报告中明确标为研究建议或预算算例。
