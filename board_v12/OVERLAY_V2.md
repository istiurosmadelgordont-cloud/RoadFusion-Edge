# Overlay V2 设计与开源参考

本版本保留 YOLOv8n-P2、ByteTrack 和 UFLD V2 1600×320 的现有推理链，重点改进感知结果之后的状态保持、风险表达和 UI。所有 C++ 代码均按本项目的数据结构重新实现，没有复制第三方源码。

## 采用的设计思路

- [openpilot on-road model renderer](https://github.com/commaai/openpilot/blob/master/openpilot/selfdrive/ui/onroad/model_renderer.py)：参考车道置信度表达、前车目标强调及距离/接近速度共同影响风险提示的思路。
- [openpilot selfdrived](https://github.com/commaai/openpilot/blob/master/openpilot/selfdrive/selfdrived/selfdrived.py)：参考变道时只检查目标侧盲区，以及准备、阻止、开始和结束等显式状态。
- [Autoware detected objects visualizer](https://github.com/autowarefoundation/autoware_ai_visualization/blob/master/detected_objects_visualizer/README.md)：参考用箭头表达目标运动方向和用颜色区分速度/风险层级。
- [Autoware traffic light classifier](https://github.com/autowarefoundation/autoware_universe/tree/main/perception/autoware_traffic_light_classifier)：参考先检测 ROI、再分类颜色/形状以及证据不足时输出 UNKNOWN 的分层方式。
- [Autoware lane departure checker](https://github.com/autowarefoundation/autoware_universe/blob/main/control/autoware_lane_departure_checker/README.md)：用于明确工程边界。完整量产式车道偏离还需要车辆轮廓、轨迹、定位与地图边界；当前实现属于相机视觉演示级 LDW。
- [Apollo Dreamview](https://github.com/ApolloAuto/apollo/blob/master/docs/13_Apollo%20Tool/%E5%8F%AF%E8%A7%86%E5%8C%96%E4%BA%A4%E4%BA%92%E5%B7%A5%E5%85%B7Dremview/dreamview_usage_table_cn.md?plain=1)：参考普通、关注、危险对象分层显示的原则。

## 当前行为

### 车道与 LDW

- 左、右边界独立着色；确认的偏离侧显示红色粗线，正常侧为绿色。几何或身份仍在确认时改为灰色，不用偏移阈值直接画红色车道。
- 实线连续绘制，虚线分段绘制，语义未知时使用点线。
- UFLD 车道对突然移动约一个车道宽时，短时保持原车道身份，并将移动方向作为偏离证据。
- LDW 输出 `LDW-L` 或 `LDW-R`。触发后至少保持 0.8 秒，连续获得可信且居中的新结果可解除；持续不可信的观测也会使告警过期（不代表已确认车辆居中），避免低频 UFLD 更新造成闪烁。
- 持续的新车道几何最终仍会被接纳，防止画面永久冻结在旧车道。

### FCW / RCW

- 普通目标使用原类别颜色；选中的风险目标依 TTC 使用黄色、橙色、红色粗框。
- 前视在选中目标框上标注 `FCW TARGET`，并显示距离、接近速度和 TTC；不画从车头指向目标的箭头，避免被误认为预测轨迹。
- 后视保留 RCW 目标、距离和 TTC。已撤掉固定比例的大梯形：后视车道推理关闭时没有道路几何，固定图形不能代表实际车道或危险走廊。

### BSD / LCA

- 左右视常驻显示 BSD ROI；空闲为低透明青色，占用为黄色。
- 同侧转向意图与盲区占用同时存在时，ROI 和目标变红并显示 LCA 风险。
- Lane Change HUD 将视觉观察到的左/右偏离与模拟变道请求分别显示，再展示车道规则、盲区状态和 FSM 状态。真实视频没有转向灯或驾驶员请求输入时，请求显示为“未提供”；观察到偏离不等同于驾驶员有变道意图。

### 周边示意图

- 普通目标为灰白色，关注目标为黄色，危险目标为红色。
- FCW/RCW 目标按风险等级着色；不画目标指向本车的箭头，因为二维示意位置不能表示预测轨迹。
- 该区域是基于四路二维检测框的示意图，不代表真实世界坐标或真正 360° AVM。

## 路口方向输入

没有有效转向灯或路线输入时，路口方向显示“未知”，不能把默认的 `STRAIGHT` 当作驾驶员意图。侧栏转向按钮只提供 10 秒模拟输入，供状态机演示使用。

## 视频与板端验证

- 使用 [NVIDIA PhysicalAI-Autonomous-Vehicles](https://huggingface.co/datasets/nvidia/PhysicalAI-Autonomous-Vehicles) 的四路同步真实片段 `3d045a45-0920-4580-90f3-5a83abafa025`（20.17 秒）复核右向车道偏离。板端回放约 13.3 秒时显示 `LDW-R` 和“视觉 右偏离”，同时“变道请求 未提供”；重新居中后告警解除。
- 用另一段真实片段 `050541b1_30fps` 复核左向偏离和解除。`perception_regression` 在 RK3568 上通过；GUI 实测截图约 17.6–18.7 FPS，没有设置 15 FPS 限速。
- 素材和板端截图保留在本地验证目录，不随源码分发。视频只用于演示回归，不证明任意场景下的车道识别准确率或驾驶意图推断能力。

## 安全边界

未接入 CAN、摄像头内外参、车辆尺寸、定位和高精地图前，速度、TTC、实虚线、LDW、LCA 与路口决策只用于演示和算法验证，不作为车辆控制输入。语义未知、侧视数据过期或规则证据不足时，变道状态机默认不允许变道。

系统健康状态在启动宽限期后才判断超时，避免程序初始化阶段直接显示故障。YOLO/NPU 或 UFLD 结果超时会显示 `STALE`，同时依赖该数据的决策按现有失效保护清空或阻止。
