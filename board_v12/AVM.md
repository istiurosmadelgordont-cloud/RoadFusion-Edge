# AVM本地素材鸟瞰演示

AVM以相机内外参或地面对应关系进行近似二维地面投影，与板卡一的三视角LUT前景融合分开。当前界面提供环视页、本地素材播放/暂停/复位和时间微调；鸟瞰刷新不阻塞主视频显示。

## 当前素材入口

`sh scripts/run_avm_473423e3.sh` 使用 `four_view_sample/avm_473423e3`。脚本检查`avm.yml`、`sync.yml`及rear/rear_left/rear_right/front_tele视频，并以`--avm-local-scene`接入。该入口仍通过PCIe显示实时视频，AVM本地素材使用独立手动时钟，不能描述成实时输入自动同步生成的鸟瞰。

视频、标定和下载数据不随源码完整上传。必须使用同一片段、同一相机组的标定；其他组不能只改文件名复用映射。原66b5fa4b组及近似几何是历史素材，不是所有新场景的通用标定。

## 标定与限制

[tools/build_nvidia_avm_calibration.py](tools/build_nvidia_avm_calibration.py) 用NVIDIA片段相机内外参与数据清单生成标定；Python环境需要OpenCV、pandas、pyarrow及scipy。按该脚本参数准备对应`additional_views`清单与标定文件，不能仅给任意四路视频就声称已完成标定。地面对应关系实验另见 [tests/make_avm_calibration.py](tests/make_avm_calibration.py)。

投影依赖平面地面假设，车辆、路沿与建筑会产生拉伸/接缝，未观测区域保留为空缺。该功能不是完整360度测量，也不是可旋转的真实三维重建。实时相机需单独实测标定及时间对齐。

`avm_regression`与`manual_avm_clock_regression`分别检查投影逻辑与手动时钟；具体素材仍需核对方向、接缝和车辆位置。历史与最新验证范围见 [文档索引](docs/README.md)。
