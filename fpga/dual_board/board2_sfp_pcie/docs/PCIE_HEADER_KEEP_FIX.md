> 当前工程：[主工程ZIP](../sfp_ddr_image_pice_handshake_20261019_header_keep_fix.zip)、[PCIe仿真库ZIP](../sfp_ddr_image_pice_handshake_20261019_pcie_sim.zip)。两包解压到同一父目录，合并同名工程文件夹；下述源码路径相对工程根目录。

# Logos PCIe 3DW包头兼容修正

2026-10-19发布，修正于2026-10-18完成。修改位置：`source/source/pcie_dma_ctrl.v` 第778行。

Logos PCIe IP手册UG022004第44页图2-11给出的3DW Posted包头在128位接口中为X/H3/H2/H1，包头拍tkeep=1111；第78页表2-83说明高32位为占位。不能用tkeep==0111识别三DW包头。

原通用RX解析器接受keep[2:0]==111，图像路径却额外要求keep==0111，导致视频控制成功、图像0x218邮箱登记与0x200..0x214参数写入被忽略。现在已通过外层MWr32/Length/BAR/BE/对齐/时序校验的包头直接标记为有效；单DW数据拍仍要求keep==0001。没有改PCIe/SFP生成IP或视频四缓冲协议。

提供的ARM pcie_v3源码使用BAR1、0x218独立128字节邮箱、0x200/204/208/20C/210五字和0x214唯一cookie，格式与本版一致，无须为包头修正改变ARM命令。实际部署仍须保证使用相应驱动和应用；仓库其他ARM目录本次未替换。修改控件后要点击“应用参数”。

重新生成并下载匹配位流后，ARM停止并重新开始采集，以重新登记邮箱。随后提交参数，观察image_return_addr_valid、image_received_cookie、image_applied_cookie和两种dbg_image_*_sent_event。RECEIVED表示接收回复，APPLIED表示对应帧边界生效；TX完成事件不等于ARM应用已处理回包。

9项离线仿真通过：手册1111与兼容0111包头的控制CDC、两阶段回传、视频DMA握手、DMA期间并发调参、输入debug探针、keep定向复现，以及DDR后完整图像/HDMI/DMA集成。日志见`verification/20261019/header_keep_fix/`。

FIC使用debug/image_pcie_debug.fic，布局为Core0的133/161位与Core1的244/245位、各256点。本次包头修正没有改变探针布局。

此发布以桌面当前源码为准。发布过程没有运行PDS综合/布局布线/位流生成或下载；ZIP不附带构建缓存和位流。此前已有构建文件不用于证明此次发布已完成时序或板上验证。
