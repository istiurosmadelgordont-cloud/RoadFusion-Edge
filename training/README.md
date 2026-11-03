# 模型训练与转换资料

当前部署以 [board_v12模型清单](../board_v12/config/model_manifest.json) 为准：YOLO14类QAT INT8、MO灯态FP16及UFLDv2原始部署INT8。

| 资料 | 内容与范围 |
| --- | --- |
| [UFLDV2_QAT.md](UFLDV2_QAT.md) | 1600×320车道模型续训、蒸馏与量化候选；未替换当前部署模型 |
| [PC_QAT_RESULTS_20261020.md](PC_QAT_RESULTS_20261020.md) | PC量化比较与板端候选检查，教师一致性不等于真实道路准确率 |
| [scripts](scripts/) | 历史ATLAS整理、21类YOLO训练与评估脚本，按脚本及数据配置使用 |

训练集、校准集、原始视频、权重及缓存不随源码完整提供。旧21类训练流程不能直接被描述成当前14类部署模型的全套可复现训练链。更换模型需核对输入尺寸、输出头、类别顺序、量化方式、RKNN Runtime以及SHA256。

RKNN是部署格式，INT8/FP16是数值精度，QAT是训练方式；三者不可互换描述。测量与验证范围见 [性能记录](../docs/PERFORMANCE.md)。
