# 2026-10-20 UFLDv2 QAT 续训与 PC 测试摘要

代码、输入尺寸和输出头不变：CULane ResNet18，1600×320。12 个视频按 9/3 划分训练与验证，288/96 张图；没有人工标签，使用冻结 FP32 教师蒸馏。

从首版第 8 轮独立恢复，以 5e-6 学习率继续至第 16 轮。新增 576 次优化更新，累计 1152 次；第 15 轮最佳，教师验证 KL 从 0.060076 降至 0.057816，降低约 3.8%。训练耗时 469.6 秒。只有 Conv/Linear 使用 eager INT8 QAT；BN、残差、LayerNorm 仍浮点。

| 96 张同输入真实 QNNPACK INT8 PC 比较 | 首版 | 续训 |
| --- | ---: | ---: |
| 相对严格 FP32 的 KL | 0.060262 | 0.057994 |
| 共有本车道点平均横向差，640px 画面 | 2.138px | 1.999px |
| 存在性输出一致比例 | 98.293% | 98.271% |

用户提供的 `lane_middle_new_20261017.zip` 含 12 段、7219 个源帧，与训练/验证无视频 ID 重叠。stride=2，每个模型处理 3611 帧，回放维持 15 FPS 原时间线。CUDA 全视频使用 FP32 与续训 QAT fake quant，原始几何有效帧比例为 95.375% / 95.569%。该比例只是通过既有几何门限，不能当作准确率。

每段各取 4 帧，共 48 帧用导出的真实 QNNPACK INT8 核对：原生 INT8 vs fake quant 平均 KL 0.000545、共有本车道点横向差 0.108px；原生 INT8 vs FP32 横向差 1.650px。另用 `7679660a` 整段进行真实 INT8 连续测试，302 个处理帧中 FP32 和 INT8 均通过几何门限，未出现有效→无效事件；不能推广至其他片段。

真实 INT8 PC CPU 调用均值约 314ms，视频离线生成后按 15 FPS 回放，不等于实时推理 15 FPS。完整视频用同一几何跟踪规则，最多保持 3 个处理帧，原始与保持结果分开统计；本次只测试本车道几何，不评估虚实线分类。阴影和弱线片段仍会丢线。

RKNN 候选单独导出，使用 SDK 匹配的 Torch 1.13.1 / QNNPACK / RKNN Toolkit2 1.5，`do_quantization=False`，没有二次 PTQ。96 张板端实际 NPU 输出比较中，当前原 INT8 为 134.8ms、续训 QAT 为 199.3ms；相对 FP32 横向差 0.972px / 1.932px。因此板端保留原车道 INT8，不更换默认模型。YOLO 仍是既有 QAT INT8 模型。

复现脚本：`ufld_qat_data.py`、`train_ufld_qat.py --resume-from ...`、`export_ufld_qat.py`、`convert_ufld_qat_rknn.py`、`eval_ufld_qat_pc.py`、`test_ufld_qat_video_set.py`、`finish_ufld_qat_video_set.py`。本地生成的模型、视频、逐帧缓存和日志不随源码上传。详见 [训练记录](UFLDV2_QAT.md)。
