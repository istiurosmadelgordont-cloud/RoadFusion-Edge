# RoadFusion-Edge：RK3568 board_v10

当前 PCIe 四宫格板端源码，2026-10-26 从实际板端同步。包含图像参数控制、测试一红绿灯、测试二车道控制、前向 FCW 与测试四模拟停车模块。

**模型、启动方式、默认开关及验证边界以 [CURRENT.md](CURRENT.md) 为准。**

- [前向 FCW 与简洁显示](docs/FCW.md)
- [PCIe 图像参数控制](docs/PCIE_V3_IMAGE_CONTROLS.md)
- [驱动性能与限制](docs/DRIVER_PERFORMANCE.md)
- [测试四：多人标红、模拟停车与自动解除](docs/TEST4.md)
- [测试四最新板端验证](docs/TEST4_VALIDATION_20261026.md)
- [模拟导航](docs/NAVIGATION_DEMO.md)
- [本次同步与验证](docs/VALIDATION_20261026.md)
- [历史说明](docs/LEGACY_README.md)

模型、原视频和编译产物独立保存。YOLO 为 QAT INT8，UFLD 仍是原始 1600×320 INT8 部署模型；所有停车/行驶行为均为模拟。
