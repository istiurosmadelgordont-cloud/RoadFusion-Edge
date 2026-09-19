# Preserved V7 model artifacts

These three files are the same 21-class YOLOv8n-P2 detector at successive
deployment stages. `V7` identifies the seventh training revision; it does not
mean YOLOv7 and the model does not have 23 classes.

| File | Purpose | SHA-256 |
| --- | --- | --- |
| `unified21_p2_v7_domestic_preserved.pt` | Preserved PyTorch training weight | `9BF7079F2D1BD971D1B4C3A5ECE00C2FFCB96D977B79DDF9705E647790D16D7E` |
| `unified21_p2_v7_640_rkopt.onnx` | 640×640 Rockchip-compatible ONNX export | `BF4A77F0F53085E8352426550C2A695CAD9DEF90139288B0432E262F6F83CBB1` |
| `unified21_p2_v7_640_int8.rknn` | RK3568 INT8 deployment model | `6A29DD1FBCC11CB1083C9617EC96040763E97F39368921322A427FE0B16DA87C` |

The fixed class order is:

```text
0 pedestrian             7 traffic_red_circle     14 traffic_green_straight
1 rider                  8 traffic_red_left       15 traffic_sign
2 car                    9 traffic_red_right      16 crosswalk
3 bus                   10 traffic_red_straight   17 guide_arrows
4 truck                 11 traffic_green_circle   18 traffic_cone
5 motorcycle            12 traffic_green_left     19 roadworks_sign
6 bicycle               13 traffic_green_right    20 delineator
```

The INT8 calibration set was selected from the V7 training split with
class-balanced quotas at 640×640. The calibration images and original training
dataset are deliberately excluded from Git because they are generated/local
data. See `tools/select_v7_calibration.py` for the reproducible selection step.

Known limitation: the preserved weight still confuses some small or blooming
arrow lamps with circle lamps. The RKNN conversion was checked against the
PyTorch output; the observed examples were already wrong in the PyTorch model,
so they are training-domain errors rather than new INT8 quantization errors.
