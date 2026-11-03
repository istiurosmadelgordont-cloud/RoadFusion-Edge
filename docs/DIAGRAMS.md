# 架构与流程图

图示按2026-10-31主分支及10月30日双板归档说明整理，更新于2026-11-03。实线表示视频或处理数据流，虚线表示控制/生命周期关系；图示不新增功能或验证结论。

| 图 | 阅读位置 | 重点 |
| --- | --- | --- |
| 整机分工 | [根README](../README.md#系统组成) | 双FPGA像素流与RK3568感知交互 |
| 双板视频流 | [双板说明](../fpga/dual_board/README.md#数据流与输入映射) | 两个SFP方向、四路来源与两级显示 |
| ARM感知流水 | [板端架构](../board_v12/ARCHITECTURE.md#数据与任务分工) | 同步RGA、共享NPU及ARM后处理 |
| FPGA图像增强 | [板卡二说明](../fpga/dual_board/board2_sfp_pcie/README.md#图像增强与旁路) | Y通道多级处理、色度/参考对齐及RGB旁路 |
| 参数生效时序 | [参数说明](../board_v12/docs/PCIE_V3_IMAGE_CONTROLS.md#寄存器与回执) | cookie、跨域握手、RECEIVED与APPLIED |
| DMA/BGR寿命 | 本页下图 | 源槽归还与消费者释放分别管理 |

## 缓冲区所有权与图像寿命

```mermaid
flowchart TB
    FREE["FPGA空闲DMA槽"] --> WRITE["FPGA写图像与帧尾token"]
    WRITE --> GET["驱动确认完整帧<br/>GET_FRAME交给应用"]
    GET --> ROOM{"BGR池有空闲输出？"}
    ROOM -->|否| SKIP["不访问像素，归还该源槽"]
    ROOM -->|是| RGA["同步RGA读取源槽<br/>写入独立BGR输出"]
    RGA -->|转换成功| SPLIT["源图读取结束"]
    SPLIT --> PUT["PUT_FRAME携完整token归还"]
    PUT --> FREE
    SKIP --> FREE
    SPLIT --> PUB["发布BGR图像<br/>推理、ROI和显示共享引用"]
    PUB --> WAIT["所有消费者释放引用"]
    WAIT --> REUSE["BGR池输出可再次复用"]
    REUSE -.-> ROOM
    RGA -->|失败| ERR["记录错误并停止采集<br/>退出时排空与回收"]
```

源DMA槽在同步RGA结束后即可归还；独立BGR图像必须等推理、ROI和显示的最后一个消费者释放后才可复用。四个源槽与应用BGR输出池是不同层次，`--pcie-buffers`不改变FPGA槽数。图中是正常交付及转换失败的简化流程，STOP、会话切换和旧token拒绝以驱动实现为准。

## 阅读边界

- 四路并行采集不代表相机曝光同步，也不代表三个模型同时在NPU执行。
- ARM融合布局只组织已有画面；空间融合由FPGA计算，KEY2模式与界面需人工匹配。
- 模拟导航、行驶、转向和停车均是应用状态，没有车辆执行控制链。
- 图像增强可配置不代表各级默认开启；时序、性能与模型准确率仍按[性能记录](PERFORMANCE.md)的来源分别判断。

历史测试二设计图仍留在[原计划](test2_adas_plan.md)，不与当前实现图混用。
