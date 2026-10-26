# 当前板端版本：board_v10

2026-10-26 从 RK3568 的 `/home/cat/rk3568_adas/board_v10` 同步，随后部署测试四多人危险框、自动解除模拟刹车及行人估计区域加宽。当前二进制 SHA256 为 `1b5c2036896c2b3caf047192e36f1bd5999038e8faea9bf5fdb3700b296dc402`；待发布的 108 个技术源文件已重新核对，与板端逐字节一致。未混入另一个本地工作目录中尚未部署的导航路线改动。

## 当前行为

- PCIe 四宫格保持前、后、左、右画面。右上保留图像处理预设、参数窗口、草稿/应用及 RECEIVED/APPLIED 两阶段回执控制；驱动说明见 [PCIe 图像控制](docs/PCIE_V3_IMAGE_CONTROLS.md) 和 [驱动性能记录](docs/DRIVER_PERFORMANCE.md)。
- 测试一保留用途车道选灯、左/直/右方向、车辆行驶/停止按钮和红灯优先报警。
- 测试二保留左/右变道、保持直行和强制变道按钮。新鲜虚线证据才允许对应模拟变道；实线保持直行，强制跨实线报警，未知线型不授权。暂不启用盲区联动。
- UFLDv2 1600×320 仅运行前视本车道。实线连续、虚线分段；不回退到白色旧线。低位置路面取样会跟随当前可见曲线下移；失效证据清除旧线型及候选确认，避免沿用上一场景的虚线。
- 前向 FCW 有独立按钮；使用新鲜本车道车辆观测、跟踪和单目风险估计。显示小号距离，车道以柔和绿/黄/红渐淡覆盖，取消装饰性前行箭头。顶部中间显示黄色/红色碰撞等级，风险解除后恢复标题。详见 [FCW](docs/FCW.md)。
- “图形叠加”默认关闭道路上的大块状态/变道说明，右侧原按钮保留。需要测试二方向提示时开启该按钮或追加 `--signal-overlay`；车道几何由车道检测开关和有效证据单独控制。自动车道偏离报警 LDW 继续关闭，测试一和强制实线变道报警保留。
- YOLO＋MO 作为一组使用串行 NPU 队列，UFLD 与其共享调度。目标按类别着色，跟踪/光流显示补偿不代替真实安全观测；源帧过期和真实漏检仍清理旧框。
- 右上可切换模拟导航与测试四。导航方向/距离由按钮输入，未实现导航视频 OCR。测试四包含行人/近距障碍模拟停车，多个危险目标分别标红，连续确认安全至少 1 秒、4 次新观测后自动解除模拟刹车，起步仍需手动；默认关闭。完整车道几何缺失时现有规则使用图像走廊。见 [测试四](docs/TEST4.md)。
- 后向 RCW 基础逻辑保留，默认由 `--no-ttc` 关闭；FCW 开关独立于它。后车素材筛选主要为 PC FP32 验证；测试四两段行人素材已补充板端 RKNN INT8 抽样回放，范围见下方验证记录。

## 模型

| 用途 | 文件 | 当前状态 |
| --- | --- | --- |
| 目标定位 | `yolo14_960x544_qat_int8.rknn` | 960×544、QAT INT8、NPU |
| 灯形/颜色 | `mo_fp16_baseline.rknn` | 128×128、FP16、NPU |
| 车道几何 | `ufldv2_culane_res18_1600x320_int8.rknn` | 原始 1600×320 INT8 部署模型、NPU；未替换为新续训 QAT |

三份模型哈希见 [模型清单](config/model_manifest.json)。模型和测试视频单独准备，不包含在本次源码上传中。YOLO 与 MO 放在同一模型目录。

## 构建与启动

依赖 RK3568/aarch64 Linux、CMake、C++11、OpenCV、FreeType、RKNN Runtime、RGA、EGL/GLES2、X11 和 MPP/GStreamer。中文字体使用板端既有的 `/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc`。

```sh
cd /home/cat/rk3568_adas/board_v10
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --target adas_four_view -- -j1
export DISPLAY=:0
export XAUTHORITY="$HOME/.Xauthority"
sh scripts/run.pcie.sh --enable-lanes --enable-fcw

# 普通启动默认关闭车道和 FCW，可使用界面按钮开启
sh scripts/run_pcie.sh

# 单独开启测试四，是否启动车道由演示场景决定
sh scripts/run_pcie.sh --enable-lanes --enable-test4
```

PCIe 走 `/dev/fpga_frame_<BDF>` 的 read 路径；多节点时设置 `PCIE_DEVICE`。启动脚本沿用已有的 NPU 最低频率恢复条件。测试四更新已重启应用验证，没有切换已加载驱动或替换模型。界面开关状态不持久化，重启后以启动参数为准。

相机输入变化的诊断及适配方向见 [相机输入与车道适配](docs/CAMERA_INPUTS.md)；长焦专用配置尚未部署。

## 验证范围

2026-10-26 对板端文件做逐文件哈希快照，并在板端重跑 5 项已部署回归；另外从待上传源码独立编译并运行 10 项 CPU 回归。详见 [同步验证记录](docs/VALIDATION_20261026.md)。

随后测试四更新完成板端规则、真实控件回归及两段行人素材的 RKNN INT8 抽样回放；多人素材最多同时记录 4 个危险目标。左右边缘行人扩大范围，前车危险保持原范围；检测过期保持模拟刹车，持续安全后自动解除但不自动起步。详见 [测试四部署验证](docs/TEST4_VALIDATION_20261026.md)。

本次未重新训练模型，未测量模型准确率、绝对测距误差或完整端到端延迟。之前约 3–3.5 次/秒的检测更新和约 7 FPS 的显示只是特定素材样本。行人和距离均缺少人工真值；所有车辆行驶/停车动作是界面模拟。
