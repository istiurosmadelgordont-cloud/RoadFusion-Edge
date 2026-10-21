# UFLDv2 1600×320 QAT 训练与验证记录

仓库内可直接阅读的汇总：[2026-10-20 PC 与板端候选比较](PC_QAT_RESULTS_20261020.md)。下文的 `training/ufldv2_qat_20261020/...` 路径为本地生成产物，模型、视频和缓存不随源码上传。

## 2026-10-20 续训及用户素材 PC 测试

保留 2026-10-16 首版，从其第 8 轮 `last.pt` 独立续训到第 16 轮；学习率降为 5e-6，数据划分不变。新增 576 次优化更新，累计 1152 次；第 15 轮（累计 1080 次更新）的权重最佳，训练耗时 469.6 秒。最佳验证教师 KL 从 0.060076 降为 0.057816，改善约 3.8%，仍然只是教师一致性。

续训候选保存在 `training/ufldv2_qat_20261020`，原生 INT8 TorchScript 与 RKNN 均已独立导出。`train_ufld_qat.py --resume-from` 支持恢复到独立输出目录，恢复优化器后显式应用新学习率，并在新目录独立选择最佳模型。

PC 上对 96 张留出图的真实 QNNPACK INT8 同输入比较：

| 指标 | 首版 QAT | 续训 QAT |
| --- | ---: | ---: |
| 对严格 FP32 教师的平均 KL | 0.060262 | 0.057994 |
| 共有本车道点平均横向偏差，640px 视图 | 2.138px | 1.999px |
| 本车道存在性输出一致比例 | 98.293% | 98.271% |

几何偏差略减，存在性一致比例略降，不能声称所有指标都提高。没有人工标签，不能报告准确率提升。

用户随后提供 `lane_middle_new_20261017.zip`。全部 12 段素材、7219 个源帧已按 stride=2 测试，每个模型处理 3611 帧；没有与训练或验证集重叠的视频 ID。RTX 4070 CUDA 完整视频比较使用严格 FP32 与 QAT fake quant；原始几何有效帧占比分别为 95.375% 和 95.569%，该占比不是准确率。清晰片段稳定，桥下阴影等场景两者仍会丢线。

另外对全部 12 段各抽 4 帧运行真实 INT8：原生 INT8 vs fake quant KL 为 0.000545，共有本车道点横向差 0.108px；原生 INT8 vs FP32 平均横向差 1.650px。对片段 `7679660a` 整段进行真实 INT8 连续测试，302 个处理帧中 FP32 与 INT8 都通过几何检查，没有原始有效→无效事件；该单段结果不能推广到所有场景。真实 INT8 在 PC CPU 上模型调用均值 314.0ms，展示视频为离线处理后按 15 FPS 回放。

全部视频只显示本车道几何，使用相同跟踪规则；虚实线分类不在本次对照范围内。没有替换板端模型。详情和可播放视频入口：

- `training/ufldv2_qat_20261020/RESULTS.md`
- `training/ufldv2_qat_20261020/user_zip_test/index.html`
- `training/ufldv2_qat_20261020/user_zip_test/RESULTS.md`
- `training/ufldv2_qat_20261020/evaluation/pc_comparison.json`

用户询问板端替换后，分析了已经完成的 96 张同输入隔离 NPU 测试（800MHz）：当前原 INT8 / 续训 QAT 平均耗时 134.80 / 199.30ms，对 FP32 教师 KL 为 0.032223 / 0.057137，共有本车道点横向差 0.972 / 1.932px。续训比首版略有改进，仍没有超过当前原 INT8；默认模型保持原版。结果在 `training/ufldv2_qat_20261020/evaluation/comparison.json`。这不是完整应用 FPS 或人工标签准确率测试。QAT 数字输出名还需要 v9 严格形状兼容适配，不能直接只改模型路径。

## 2026-10-16 首版记录

本轮从原 CULane ResNet18 PyTorch 权重训练，保留四个输出头及原输入尺寸。板端原 INT8 RKNN 保留，候选独立导出。

- 数据：12 个不同的 NVIDIA 原始前视片段，按视频划分 9 个训练、3 个验证；288/96 张。排除原有五个演示片段，并检查图像重复。没有人工车道标签，使用固定 FP32 教师蒸馏。
- 训练：8 轮、576 次优化更新；batch=1、累积=4、SGD lr=1e-5。23 个 Conv/Linear 进行 eager QAT；全程启用 fake quant，第 2 轮后冻结 observer。BN、残差和 LayerNorm 保留浮点计算。
- 验证：最佳验证集教师 KL（T=2）0.060076；这是 fake-quant 学生对教师的一致性，不是 CULane F1，也不是交通场景检测准确率。
- 导出：SDK 匹配的 Torch 1.13.1，QNNPACK 原生 INT8 Conv/Linear；导出不重新校准。RKNN Toolkit2 1.5.0、target=rk3568，`do_quantization=False`，没有二次 PTQ。浮点节点仍存在，不能声称全部算子都为 INT8。
- 板端：3 个验证视频各取一张裁剪，原模型与 QAT 候选均成功运行，四输出头有限且尺寸相同。QAT 输出名字为数字，运行时按严格 NCHW 形状匹配，不依赖输出顺序。

3 张留出视频样本的实际 RK3568 输出对比：

| 输出差异：KL，越低越接近参考 | 原 INT8 | QAT 候选 |
| --- | --- | --- |
| 板端输出 vs 原 FP32 教师 | 0.030346 | 0.184357 |

QAT 板端输出 vs 原生 QNNPACK 输出 KL 为 0.008970。三次短时探测的推理均值范围：原模型 160–205 ms，QAT 216–224 ms；包含取输出及有限值检查，少量样本和写盘，不构成稳定性能基准。当前首版 QAT 没有优于原 INT8 的证据，不能报告精度提升或加速。

也已在测试二完整程序中用英伟达高速多车道素材独立回放 180 帧：QAT 车道推理约 246–260 ms，原模型约 171–193 ms；QAT 的更新率更低且出现过期丢弃。因此当前板端交互界面仍选择原模型。两张回放截图由异步调度在各自媒体时刻产生，并非完全同一源帧；三张裁剪的输出比较才是同输入对照。

建议后续优先核对原生转换前后的误差、Conv/BN 融合和观测范围；之后使用人工标注及完整视频验证，分别报告车道几何 F1、时序抖动、左右线型、BSD、FCW/RCW 的漏报误报。不能以“能画三车道”代替精度评价。

文件：

| 路径（相对仓库） | 内容 |
| --- | --- |
| training/ufldv2_qat_20261016/data/manifest.json | 数据来源、划分、源帧、图像哈希 |
| training/ufldv2_qat_20261016/run/best.pt、last.pt | 最佳及可恢复训练状态 |
| training/ufldv2_qat_20261016/run/metrics.jsonl | 训练/验证记录 |
| training/ufldv2_qat_20261016/export/ufldv2_culane_res18_1600x320_qat_int8.pt | 原生量化 TorchScript |
| training/ufldv2_qat_20261016/export/ufldv2_culane_res18_1600x320_qat_int8.rknn | 206,922,104 字节 RKNN 候选 |
| board_v8/output/test2_20261016/qat_check/comparison.json | 三样本板端/原生/教师输出对照 |
| board_v8/output/test2_20261016/test2_qat.png | 候选的实际整机回放截图 |

脚本依次为 `ufld_qat_data.py`、`train_ufld_qat.py`、`export_ufld_qat.py`、`convert_ufld_qat_rknn.py`。`eval_ufld_qat_board.py` 对同一裁剪的板端输出与原生/教师比较。恢复训练使用原输出目录和 `--resume`；导出脚本应使用本地 `.rknn-env` 的 Linux Torch 1.13.1 环境。
