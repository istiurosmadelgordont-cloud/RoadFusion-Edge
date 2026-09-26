# UI 开源参考与落地边界

当前 `GlPresenter` 已把四路摄像头和 UI 画布分别作为 GLES2 纹理显示；感知叠加仍先由 OpenCV 画到每路图像。以下项目用于参考绘制分层和交互方法，本仓库这一版没有拷贝它们的代码或引入新运行时依赖。

| 参考 | 可借鉴的部分 | 当前约束 |
| --- | --- | --- |
| [openpilot AugmentedRoadView](https://github.com/commaai/openpilot/blob/master/openpilot/selfdrive/ui/onroad/augmented_road_view.py) 与 [ModelRenderer](https://github.com/commaai/openpilot/blob/master/openpilot/selfdrive/ui/onroad/model_renderer.py) | 摄像头、道路模型、HUD、告警依次绘制；车道几何和前车风险的视觉主次 | 本项目 UFLD 是图像空间车道线，没有 openpilot 的标定道路模型；只沿实测线绘制，不能把走廊标成预测轨迹 |
| [Dear ImGui](https://github.com/ocornut/imgui/blob/master/docs/BACKENDS.md) | 状态卡、按钮和调试控件；官方 backend 支持 GLES2/3 | 接入前需要测量 RK3568 上的帧时，并处理现有 X11 输入与字体；目前右侧卡片仍用已有绘制链 |
| [NanoVG](https://github.com/memononen/nanovg) | 抗锯齿曲线、渐变、发光和矢量图标 | 上游自述不活跃；暂作视觉与 API 参考，不绑定整个 UI |
| [Apollo Dreamview Plus](https://github.com/ApolloAuto/apollo/tree/master/modules/dreamview_plus) | 多视角主画面与状态区的层级 | Web HMI 不直接迁入板端 |

优先逐项验证：周边风险方位 → 前视车道边界与视觉走廊 → FCW 目标与危险层级 → 侧视 BSD/LCA → 其余状态卡。每项用真实四路回放截图检查正常与风险状态；真实素材没有触发的等级用明确标注的合成输入测解除和极端情形。任何距离和 TTC 都继续来自现有估计器并保留近似标记。导航、真实车速和档位展示已排除在本轮范围之外。

开源许可资料：[openpilot MIT](https://github.com/commaai/openpilot/blob/master/LICENSE)、[Dear ImGui MIT](https://github.com/ocornut/imgui/blob/master/LICENSE.txt)、[NanoVG zlib](https://github.com/memononen/nanovg/blob/master/LICENSE.txt)。引入第三方源码时需保留相应许可文件并重新审查其具体文件的许可；当前仅参考设计。
