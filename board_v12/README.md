# board_v12 板端部署与使用

当前 RK3568 应用。主程序为 `adas_four_view`，PCIe实时采集使用驱动8.2、DMA-BUF和同步RGA。最新功能与默认开关见 [CURRENT.md](CURRENT.md)，实现分层见 [ARCHITECTURE.md](ARCHITECTURE.md)。

## 运行环境与外部文件

- 鲁班猫2 / RK3568、ARM64 Linux；已有部署记录使用4.19.232内核、GCC8.3及OpenCV3.2。内核模块必须与目标板内核匹配。
- CMake、C++11编译器、OpenCV、FreeType、RKNN Runtime及匹配的NPU驱动；图形显示需要X11、EGL/GLES2。按构建功能还需Rockchip MPP/GStreamer等依赖。
- 配套RGA头文件/库随 [third_party/librga](third_party/librga/README.md) 说明管理，不用系统旧库替换其配套依赖。
- 三份RKNN模型、视频及场景标定另行准备。模型哈希和类别表见 [config/model_manifest.json](config/model_manifest.json)。

YOLO默认查找 `models/yolo14_960x544_qat_int8.rknn`，找不到再查 `../models/`；MO文件 `mo_fp16_baseline.rknn` 必须与YOLO位于同一目录。UFLD默认查找同名模型目录，可通过 `UFLD_MODEL` 覆盖；YOLO路径可通过 `MODEL` 覆盖。

## 构建和PCIe启动

将本目录放到板端实际部署位置，以下路径需替换：

```sh
cd /path/to/RoadFusion-Edge/board_v12
sh scripts/build.sh
export DISPLAY=:0
export XAUTHORITY="$HOME/.Xauthority"
sh scripts/run_pcie.sh

# 前视车道检测与前向风险
sh scripts/run_pcie.sh --enable-lanes --enable-fcw
# 危险模拟停车；不会自动开启车道检测
sh scripts/run_test4.sh
# 导航演示并关闭测试四
sh scripts/run_navigation_demo.sh
```

`run_pcie.sh` 调用 `prepare_pcie.sh`，复用已加载的8.2驱动，或检查内核后按需编译加载，并准备设备权限及NPU环境。脚本需要时调用sudo；旧模块不匹配时先正常退出所有占用设备的程序，再按提示处理，不强制卸载。

脚本默认打开模拟导航，关闭车道、TTC、FCW和测试四。顶栏“车道保持辅助”默认关闭，只控制前视偏离提醒。条件允许时脚本尝试把NPU最低频率恢复至800 MHz，不修改最高频率或governor。

```sh
# 多设备时指定节点；不要把占位BDF原样复制
PCIE_DEVICE=/dev/fpga_frame_0000:01:00.0 sh scripts/run_pcie.sh
# 只准备驱动与权限
sh scripts/prepare_pcie.sh
# 增大应用输出图像池，FPGA仍为四DMA槽
sh scripts/run_pcie.sh --pcie-buffers 8
```

独立 [pcie_v4](../tools/pcie_v4/README.md) 与本应用不能同时占用同一采集设备。更多链路说明见 [PCIE_INPUT.md](PCIE_INPUT.md)。

## 回放与鸟瞰

```sh
sh scripts/run_four_view.sh --scene four_view_sample/实际场景目录
sh scripts/run_four_view.sh --scene four_view_sample/实际场景目录 --headless --once
# 需要特定七相机素材和标定
sh scripts/run_avm_473423e3.sh
```

场景目录、源视频和标定不随源码上传；[场景清单](config/simulated_test_1_scenes.json)记录的是历史整理集合，不是仓库自带的视频。回放合成脚本 [build_fpga_composite.sh](scripts/build_fpga_composite.sh) 按front/rear/left/right组织文件输入，不能把它的回放文件顺序当作PCIe默认槽序。AVM素材范围与标定方式见 [AVM.md](AVM.md)。

## 界面操作

| 控件 | 作用 |
| --- | --- |
| 四路均分/主视角优先/标题栏放大 | 改变ARM显示布局；`Z`切换放大与返回 |
| 三路融合 | 显示FPGA已有的融合画布；源端KEY2仍需人工匹配 |
| 用途车道、方向、行驶/停止 | 模拟车辆状态并匹配控制灯 |
| 车道检测与车道保持辅助 | 分别控制检测、偏离提醒；后者不控制转向 |
| FCW/危险监测 | 前向风险及测试四模拟停车 |
| 导航/环视/调试与主题 | 切换功能页和七套主题 |
| FPGA图像预设/详细参数 | 编辑草稿，显式应用后等待RECEIVED/APPLIED |

规则与限制见 [模拟导航](docs/NAVIGATION_DEMO.md)、[测试四](docs/TEST4.md)、[图像参数](docs/PCIE_V3_IMAGE_CONTROLS.md)。

## 检查与记录

源码变更后按 [ENGINEERING.md](ENGINEERING.md) 执行对应构建与回归。当前源码同步记录、历史板端测试和整机性能分别见 [验证索引](docs/README.md) 与 [性能记录](../docs/PERFORMANCE.md)。文档整理不代表重新进行硬件验收。
