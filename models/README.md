# 模型部署与历史产物

当前board_v12使用三份模型，文件需单独准备；以 [板端manifest](../board_v12/config/model_manifest.json) 的文件名、SHA256及类别顺序为准。

| 用途 | 文件 | 输入/精度 |
| --- | --- | --- |
| 14类道路目标 | `yolo14_960x544_qat_int8.rknn` | 960×544 / QAT INT8 |
| MO灯态9类 | `mo_fp16_baseline.rknn` | 128×128 / FP16，含红绿灯形及拒识 |
| 前视车道几何 | `ufldv2_culane_res18_1600x320_int8.rknn` | 1600×320 / 原始部署INT8 |

YOLO与MO放在同一目录；脚本先找board_v12/models，再找仓库models。UFLD模型可通过`UFLD_MODEL`覆盖。NPU模型不是FPGA位流；多个模型共享NPU调度。

旧21类V7模型说明保留在 [V7_MODEL.md](V7_MODEL.md)。V7指训练修订，不是YOLOv7；该历史模型不能替代当前14类YOLO与MO的组合。UFLD续训QAT候选没有被选为当前部署模型，见 [训练资料](../training/README.md)。
