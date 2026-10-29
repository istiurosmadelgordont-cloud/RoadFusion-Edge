# board_v12：DMA-BUF + RGA 实时图像识别


本目录的非 RGA 功能已同步为板端 `board_v11.before-new-baseline-20261027-195936`，保留最新 board_v12 RGA 路径及名称。先阅读 [当前版本状态](CURRENT.md) 和 [同步记录](docs/BASELINE_SYNC_20261029.md)。以下业务说明及 2026-10-27 [部署记录](docs/DEPLOYMENT_20261027.md) 属于历史基线。

# RK3568 四视角 ADAS

**继承的业务基线：四宫格 ADAS、车道过渡显示与 LDW 关闭。** 保留测试一、二、模拟导航、测试四与检测框残影修复；[历史验证记录](docs/VALIDATION_20261022.md) 属于原版本，不代表 board_v12 的硬件验证。

以下为 2026-10-17 初始接入时的测试一历史说明，当前测试二与车道功能以 CURRENT.md 为准。

版本日期：2026-10-17。本目录同步当前 RK3568 的 `board_v8` 源码，用于视频回放、方向选择、红停绿行与模拟报警测试。YOLO 和 MobileNet 的推理由板端 NPU 实际执行。

## 当前配置

- YOLO：14 类定位，QAT INT8，固定输入 960×544。
- MobileNet（MO）：128×128 FP16，红/绿各四种灯形及拒识，共 9 类；模型哈希见 [模型清单](config/model_manifest.json)。
- 四路拼接为 1920×1080，每个更新只调用一次 YOLO，再对灯组裁剪调用 MO。
- 保留 ByteTrack、中文检测标签、小图形叠加及叠加开关；未确认灯框和标签隐藏。
- 默认关闭车道线和 TTC；第一排选择用途车道，第二排单击方向开始行驶，再点同方向按钮停车；行驶中点其他方向则切换方向。
- 控制灯不依赖灯组配置：左车道看左箭头、直行车道优先直行箭头、无箭头时看圆灯、右车道看右箭头；多个圆灯优先画面中间。确认红灯时任意行驶方向都报闯红灯；绿灯下选与用途车道不符的方向报违规操作，再点同方向按钮停车并解除。

“模拟板端测试一”是本次保存的版本名称。其输入是回放视频，车辆所在方向、行驶与报警通过演示控件模拟；本版本没有完成自动车道与控制灯组归属判断。夜间素材仍存在拒识、漏判及灯形误分类，不能把单张成功截图当成完整片段通过。

## 构建与运行

依赖板端现有 CMake、C++11 编译器、OpenCV、FreeType、RKNN Runtime、Rockchip MPP/GStreamer、RGA、EGL/GLES2、X11。

```sh
cd /home/cat/rk3568_adas/board_v12
sh scripts/build.sh
export DISPLAY=:0
export XAUTHORITY="$HOME/.Xauthority"
sh scripts/run_four_view.sh

# 指定已保存的右转箭头与圆灯组合场景
sh scripts/run_four_view.sh --scene four_view_sample/H_右绿圆红

# 用相同程序进行单次无交互回放
sh scripts/run_four_view.sh --scene four_view_sample/H_右绿圆红 --headless --once
```

两个模型需位于同一 `models/` 目录：`yolo14_960x544_qat_int8.rknn` 和 `mo_fp16_baseline.rknn`。模型、视频与编译产物继续单独保存，仓库提交源码、脚本、配置和版本记录。

## 11组素材与七路原视频

当前板端菜单保留 11 组，顺序、时长、四路输入哈希、灯组配置见 [场景清单](config/simulated_test_1_scenes.json)。本机备份位于“红绿灯测试素材汇总_20261011/红绿D”，每组独立文件夹。

前 10 组英伟达素材的“七路原视频”文件夹内，各保存同车同一原始片段的七个独立相机原视频及其时间戳：前视长焦、前视广角、左侧广角、右侧广角、后视长焦、左后视、右后视。原片保留完整原始时长，当前板端回放的短片区间另行记录。第 11 组中国素材保留原有视角。七路原片保存齐全不代表每个新场景都已有鸟瞰标定。

## 版本记录与验证

当前 board_v12 固定使用 DMA-BUF + RGA 实时输入，并与回放共用无需 ROI 配置的
用途车道选灯、同方向按钮切换行驶/停车和模拟预警逻辑；默认检测间隔为 1，PCIe 显示不额外限速。
原视频回放仍使用 scripts/run_four_view.sh，实时输入使用 scripts/run_pcie.sh。
编译、启动、选灯规则及实时输入说明见 [PCIe 输入说明](PCIE_INPUT.md)。

本次板端导出清单见 [源码快照](docs/BOARD_SNAPSHOT_20261015_SIMULATED_TEST_1.json)，验证详情见 [模拟板端测试一](docs/SIMULATED_BOARD_TEST_1.md)。历史模块说明保留作参考，当前模型、输入尺寸和演示配置以本页为准。脚本使用 LF 换行。
