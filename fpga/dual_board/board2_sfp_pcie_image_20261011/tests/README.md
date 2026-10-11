# 运行功能回归

进入相应测试目录，在ModelSim Transcript执行`do run_*.do`。
命令行使用`vsim -c -modelsimini modelsim.ini -do run_*.do`，确保使用该目录的本地ini，不改ModelSim安装目录。
`image_serial`检查整条链路的配置帧边界、4496公式缩小尺寸后的时序、旁路和多项串联；`tone`检查阈值曲线、饱和及色度对应延迟。
SFP/PCIe工程另外包含`pcie_control`（严格命令解析、ACK/图像DMA竞争与CDC）及`pcie_dma`（原4缓冲协议/反压/STOP/RELEASE）。
`pcie_dma/run_dma_full.do`检查完整1080p一帧64800个图像包和帧尾；`raster`用行为FIFO模型检查整帧速率及水位。
`sfp_register`使用厂商FIFO/RAM模型检查SFP硬输出寄存和完整帧；`ddr_read_alignment`检查实际活动DDR读FIFO与同步的相位；`post_ddr_integration`检查DDR原图写入、处理后HDMI/PCIe共源及同步对齐。
`dma_timing`检查最新DMA地址区间判断与旧版同拍一致，并用实际厂商FIFO比较完整1080p像素、64800图像包和帧尾/STOP通知；`run_dma_vendor_full.do`运行完整帧，`run_command_guard.do`检查地址边界和实时ACK地址变化。
`timing_equivalence`对照天空保护、穷举YCbCr转换和双边范围权重；`atmosphere_stage`、`clahe_equivalence`分别使用旧实现作为独立参考，检查时序优化后的数值与帧内稳定性。
SFP/PCIe工程的`accounting_equivalence`对照旧实现逐拍检查回执、计数、溢出优先和cookie。
这些是功能RTL仿真，板级速率/时序以PDS报告和实际板测为准。
最新交付运行记录为验证目录内`delivery_control_checks.json`、`delivery_vendor_checks.json`、`delivery_post_ddr_check.json`和`delivery_dma_timing_checks.json`，对应日志以`delivery_`开头。
DMA地址保护DO内部切换transcript，短版与64800包参数的真实日志分别保存在`delivery_command_guard_short.log`和`delivery_command_guard_full.log`；外层日志只记录编译。
`artifact_index.json`注明每项记录的验证层、哈希和架构范围；旧模块记录及`delivery_test_layout.json`属于历史记录，不作为当前DDR后顶层的验证依据。
