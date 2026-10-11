# PCIe 图像处理控制（2026-10-10）

打开 `hdmi_ddr_pice.pds`，顶层仍为 `hdmi_ddr_pcie`。旧图像处理由新版替换；DVIO已从顶层和PDS来源中移除，PCIe是本工程唯一参数入口。

## 视频处理顺序

```text
SFP RX FIFO（RAM输出寄存）→ 原始RGB565 → DDR保存原图
 → DDR读FIFO + 恢复视频VS/HS/DE
 → 对齐后的RGB888 / VS / HS / DE入口
 → DCP（固定使用平滑A；可旁路）
 → RGB→YCbCr
 → Y双边降噪
 → Gamma(Y)
 → CLAHE(Y)
 → 原高光压制 / 原暗光增强(Y)
 → 高光保护(Y)
 → YCbCr→RGB → 公共输出寄存
                    ├─ HDMI RGB888
                    └─ PCIe图像DMA RGB565
```

DDR写入原始SFP RGB565。图像处理只有一套，在DDR读出并恢复视频时序后运行；HDMI和PCIe数据来自同一公共处理输出。PCIe保持原RGB565帧格式，从处理后RGB888的R[7:3]/G[7:2]/B[7:3]取高位，不改变原四缓冲和帧尾协议。

DCP的t联合双边滤波在DCP内部。Y降噪位于Gamma和CLAHE之前。所有运行时主开关关闭时，处理链路保持还原RGB逐像素原样、相同固定延迟；还原输入沿用原DDR输出的RGB565低位补0方式，不经过颜色转换往返。

DDR读使能继续由原`de_o`驱动。当前有效读FIFO来自`source/read_ddr_fifo`，为256→16、OUTPUT_REG=0的同步RAM输出；VS/HS/DE先延1拍与它对齐，随后RGB与同步共同进入1拍入口寄存。核心固定延迟仍为`2*(1920+280)+96=4496`像素时钟。HS经过独立的等长RAM延迟，最后与核心RGB/VS/DE一起进入1拍公共输出寄存。从DDR读请求到下游采样公共输出共4499级流水，较原还原HDMI输出增加4497拍；核心输入帧边界到公共输出为4497拍。

时钟沿约定：将FIFO实际受理读请求的上升沿记为R0，入口寄存在R1后有效、核心在R2采入；公共输出寄存在R4498后有效，PCIe DMA等下游在R4499采样。4499按流水级及下游采样口径计数，相对原还原输出（R1后有效）新增4497拍。

SFP RX FIFO内部实际启用`DOB_REG=1`，解码数据、DE和对外VS相对上一版统一增加1拍。读使能被采样至下游能采样post有效输出由1拍变为2拍；内部`frame_reset`保留原时序，RX报文协议、预填门槛3456/4096和行周期2200拍保持一致。此模块位于DDR写侧，不再承担图像处理入口。

生成的`sfp_rx_32x2048`薄封装把OUTPUT_REG写成不可覆盖的localparam。当前`sfp_decode.v`直接配置PDS已列入的IPML FIFO核心，几何尺寸、CDC控制和旗标参数与原封装一致，只启用内部RAM输出寄存；原`u_sfp_rx_32x2048/U_ipml_fifo_sfp_rx_32x2048`层级名称保持一致。生成IP薄封装及IDF没有改动，重新生成该IP不会覆盖decode内的寄存配置。

A冻结和原始逐帧A旁路已删除。原始A统计仍逐帧计算，作为时间平滑输入；实际归一化与恢复统一使用平滑A。旧word0[9:10]写1也会被清零。

原高光压制、暗光增强保留其原阈值曲线，复用已有颜色转换，避免旧转换器亮色回绕：

```text
lift = darkup_en && Y<=dark_threshold ? floor((dark_threshold-Y)/8) : 0
reduce = lightdown_en && Y>=light_threshold ? floor((Y-light_threshold)/8) : 0
Ytone = clamp(Y + lift - reduce, 0, 255)
```

单项最多改变31级Y；暗光增强阈值越大，参与提亮的像素越多；高光压制阈值越小，参与压暗的像素越多。Cb/Cr保持对应像素，模块固定2拍，不新增行缓存。
它们与已有“高光保护”独立：高光保护还使用增强前Y作混回并应用soft knee。

## BAR1寄存器

每次必须是对齐、1 DW、FirstBE=F/LastBE=0的Memory Write。地址为BAR1字节偏移，表中值均为逻辑32位值。
沿用原PCIe IP的端序映射：RX payload经过`endian32()`；Linux适配器应先验证实际 `writel()` 对应端序，不能盲目额外交换一次。
RX入口的128位数据、valid、keep、last和user统一增加了一级125 MHz寄存，每拍仍可接收一个beat。它将命令解析延后一个PCIe时钟，以缩短组合路径；寄存器地址、TLP格式、端序和提交协议保持一致。

| 偏移 | 用途 |
|---|---|
| 0x200 | word0，处理开关，写入暂存区 |
| 0x204 | word1，DCP参数，写入暂存区 |
| 0x208 | word2，Y降噪和原亮度曲线阈值 |
| 0x20C | word3，CLAHE参数 |
| 0x210 | word4，CLAHE/高光保护参数 |
| 0x214 | 提交，写非零且本会话不重复的cookie |
| 0x220 | 独立64字节配置状态DMA地址，非零、64字节对齐、32位DMA/IOVA |
| 0x224 | 查询，写独立非零query cookie，回写当前状态 |

原图像DMA的0x110/130/170/180/190/1A0及4缓冲所有权协议继续使用，帧尾与STOP页的布局不变。
旧0x140/150/160接口已不控制新版图像链路；请改用上面的参数字和提交协议。
本工程没有实现BAR Memory Read Completion，**不要用readl(BAR1)读取状态或作写入flush**。

## 五个参数字

| 参数字 | 位 | 功能 | 范围 / 默认 |
|---|---|---|---|
| word0 | 0 | DCP | 0/1，默认0 |
| word0 | 1 | Gamma | 0/1，默认0；固定LUT曲线 |
| word0 | 2 | CLAHE | 0/1，默认0 |
| word0 | 3 | 高光保护 | 0/1，默认0 |
| word0 | 4 | Gamma/CLAHE前Y双边降噪 | 0/1，默认0 |
| word0 | 5 | DCP t联合双边滤波 | 0/1，默认0 |
| word0 | 6 | 天空保护 | 0/1，默认1；DCP关闭时不起作用 |
| word0 | 7 | CLAHE防光晕 | 0/1，默认1 |
| word0 | 8 | CLAHE阴影保护 | 0/1，默认1 |
| word0 | 10:9 | 保留 | 固定清0，旧A调试功能已删除 |
| word0 | 11 | 原高光压制 | 0/1，默认0 |
| word0 | 12 | 原暗光增强 | 0/1，默认0 |
| word1 | 7:0 | TX_MIN | 1~255，默认52；0钳为1 |
| word1 | 15:8 | SKY_TX_MIN | 默认128；小于TX_MIN会钳到TX_MIN |
| word1 | 23:16 | OMEGA | 0~255，默认243 |
| word2 | 8:0 | DENOISE_MIX | 0~256，默认256；0原Y，256全滤波 |
| word2 | 14:9 | DENOISE_SIGMA | 16/32/48，默认32；其余按32 |
| word2 | 23:16 | 原暗光增强阈值 | 0~255，默认0 |
| word2 | 31:24 | 原高光压制阈值 | 0~255，默认0；开启前应设置合理阈值 |
| word3 | 15:0 | CLAHE_CLIP_LIMIT | 1~65535，默认400，0钳为1 |
| word3 | 18:16 | CLAHE_STRENGTH | 0~4，默认3，0原Y |
| word3 | 25:19 | CLAHE_SHADOW_GAIN | 0~64，默认64 |
| word4 | 7:0 | CLAHE_BOOST_LIMIT | 0~255，默认64 |
| word4 | 15:8 | HIGHLIGHT_KNEE | 0~255，默认192 |
| word4 | 24:16 | HIGHLIGHT_SLOPE | 0~256，默认128，256斜率为1 |

其他位写0。超范围值按运行时控制模块钳位。分辨率、行消隐、CAPABLE、Gamma LUT、CLAHE_DONOR_MODE和BILATERAL_GUIDE_ENABLE是编译结构，不能通过这些寄存器改变。

## 上位机提交步骤

1. 用DMA API分配独立64字节coherent配置状态页；设置32位DMA mask。CPU虚拟地址不能写入0x220。
2. 验证配置页不与4个图像缓冲（各0x3F4840字节）及STOP的64字节状态页重叠。硬件也双向检查后续重新登记的地址。
3. 初始化配置页，再写0x220；发0x224查询，等待带相同query cookie的完整记录，确认地址登记成功。
4. 在同一驱动锁内写word0~4，再写0x214非零commit cookie。只允许一项未完成提交。硬件复制整包，暂存区后续写入不改变已提交包。
5. 等待APPLIED记录，或用新的query cookie查状态并核对记录中的applied_cookie。ACCEPTED只说明收到了请求。
6. 首帧尚未就绪时，QUERY可返回WAITING_INPUT，提交等待帧到来；不要反复提交同一cookie。首帧到来后若视频停流，提交仍等待下一次帧边界，QUERY可能显示ACCEPTED。图像域复位后自动重新应用最后的整包，PCIe会话复位则清除配置页地址和所有事务状态，上位机重新登记/提交。

参数在处理核心下一次输入VS下降沿整组生效。APPLIED代表参数已在核心输入帧边界锁存，从该锁存边界到核心对应输出仍需4496个像素时钟，HDMI/PCIe公共输出再延1拍；DDR读出对齐和入口寄存位于这个边界之前。APPLIED通知随后还需跨回PCIe时钟域并发送DMA记录，因此上位机收到通知的时间不能作为精确像素计时起点。改变DCP/Gamma/降噪/clip后CLAHE会重建统计，观察时至少等待3个完整视频帧。

## 64字节状态页（16个LE32）

| DW | 含义 |
|---|---|
| 0 | magic=0x494D4743，此布局为v1 |
| 1 | 每次发送递增的sequence |
| 2 | 本记录cookie（提交或查询） |
| 3 | status |
| 4 | flags |
| 5~9 | 当前已生效且已钳位word0~4 |
| 10 | accepted_cookie |
| 11 | applied_cookie |
| 12 | rejected_count |
| 13 | reply_overflow_count |
| 14 | last_rejected_cookie |
| 15 | ~sequence |

status：0复位；1接受；2已应用；3忙/拒绝本次提交；4非法命令；5非法ACK地址；6回复队列溢出；7等待输入；8恢复中。
flags[0]=图像复位释放，[1]=图像首帧就绪，[2]=像素控制pending，[3]=提交在途，[4]=配置页登记，[5]=曾回复溢出，[6]=曾提交，[7]=恢复中。
`image_ready`及flags[1]表示本次图像复位后DDR恢复视频流的首个VS高区已经到来，一旦置1就保持到下一次复位。它们用于启动和配置恢复，并不持续监测视频是否仍在传输；当前未实现VS超时或实时链路存活检测。SFP断流时DDR可能继续输出已保存的帧，此时DDR输出ready仍为1。

同一个提交cookie的ACCEPTED/APPLIED也有不同sequence。软件先读DW1，执行dma_rmb，再复制16DW，再dma_rmb并重读DW1；只接受前后sequence一致且DW15==~sequence的记录。
这避免误把同cookie的上一条尾标记作为新记录完成标记。sequence会32位回绕；驱动必须按自己的新cookie判断事务，不能只按sequence判应用成功。

8条硬件回复队列满时仍解析STOP/RELEASE，不阻塞图像DMA；溢出记数与最后丢失cookie持久保留，后续QUERY明确报告。状态页可能被后续记录覆盖，超时应查询applied_cookie恢复确认；一次一项提交可避免自发制造队列溢出。
图像与配置回写使用posted slave2，仲裁锁定整条TLP，背压时Header/Payload不变；不会把两个发送器交错拼成一个包。

## 起始配置示例

以下是调试起点，并非所有素材的最优参数。先确认原图旁路正常，再逐项打开。

| 场景 | word0 | word1 | word2 | word3 | word4 |
|---|---|---|---|---|---|
| 全旁路 | 000001C0 | 00F38034 | 00004100 | 02030190 | 0080C040 |
| 仅DCP+天空+t滤波 | 000001E1 | 00E0A060 | 00004100 | 02030190 | 0080C040 |
| DCP+Y降噪+Gamma+CLAHE+高光保护 | 000001FF | 00E0A060 | 00004080 | 010100C8 | 0080C020 |
| 夜间：降噪+Gamma+温和CLAHE+高光保护，DCP关 | 000001DE | 00F38034 | 000040C0 | 010100C8 | 0080C020 |
| 仅原暗光增强，阈值96 | 000011C0 | 00F38034 | 00604100 | 02030190 | 0080C040 |
| 仅原高光压制，阈值192 | 000009C0 | 00F38034 | C0004100 | 02030190 | 0080C040 |
| 原暗光增强96+原高光压制192 | 000019C0 | 00F38034 | C0604100 | 02030190 | 0080C040 |

word3的低噪声起点把clip减到200、strength降到1、shadow gain降到32，word4 boost限32。夜间先保持DCP=0，避免去雾增益继续放大噪声。
Gamma与暗光增强可以同时开，但会累计提亮，需根据高光和暗部噪声调整。原高光压制无法恢复输入已饱和丢失的细节。

## 最小提交示例

下面的`write_bar1()`表示实际驱动的有序32位BAR1写；`ack_dma`必须来自已分配并保持映射的独立coherent DMA页，不能把示例数值当作物理地址使用。

```c
write_bar1(0x220, ack_dma);       /* 64字节对齐、32位DMA地址 */
write_bar1(0x200, 0x000001E1);   /* DCP + 天空保护 + t联合滤波 */
write_bar1(0x204, 0x00E0A060);   /* TX_MIN=96, SKY_TX_MIN=160, OMEGA=224 */
write_bar1(0x208, 0x00004100);
write_bar1(0x20C, 0x02030190);
write_bar1(0x210, 0x0080C040);
write_bar1(0x214, commit_cookie); /* 非零、本会话不重复 */
```

等状态页中的`applied_cookie`匹配，且回传word0~4等于钳位后的参数，才确认生效。超时可写`0x224`提交新query cookie查询；没有新输入帧时提交仍在等待帧边界。图像DMA的四缓冲登记和START/STOP流程独立，继续按原握手指南执行。

## 联调观测

FLA可观察`image_cfg_active`、`image_cfg_pending/ack`、`image_rst_n`、`post_frame_vsync/de`、`image_output_vs/de`。
DCP内部`raw_cal_A_rgb/smoothed_A_rgb/src_cal_A_rgb`用于确认逐帧原始A和平滑A；无需再找已删除的a_freeze_active或a_stabilize_bypass。
SFP固定栅格要求输入分辨率/行周期与本地读时钟兼容，光口没有反压。本版额外预填1536像素、总门槛3456/4096，用已有FIFO吸收单帧内时钟漂移，不增加RAM或核心流水拍数；首次读取会稍晚开始。完整1080p行为FIFO测试覆盖读时钟、±100ppm和3/4拍水位滞后，新值12帧完整输出，峰值3617/4096；这不能替代厂商FIFO的完整CDC模型与实测。若发生短行，`g_raster_output.frame_error`抛弃该帧；应连同FIFO水位/ful抓取，不能当成算法噪声。

DMA FIFO写复位由像素域寄存发布，写使能在VS或本地复位期间关闭，释放比原组合复位晚1拍。当前1080p输出VS下降至首DE有79392拍背廊，首有效像素不受影响；这一处位于公共处理输出之后的PCIe图像DMA，不改变处理核心的4496拍延迟。改变输出同步格式时需继续保证首DE在本地复位释放之后。

测试记录见`docs/BUILD_IMAGE_PCIE_20261010.md`。寄存器接口已实现；本目录C参考头需要接入实际PCIe驱动的BAR写、coherent DMA读和屏障回调，不能直接当作已安装的上位机程序。

## 上位机适配代码

`rc/image_control_reference.h`提供C99参考接口：参数钳位、配置页地址检查、整包提交、查询、完整状态页读取和APPLIED确认。
它不包含PCIe驱动或DMA分配；将`read_ack_word`、`read_barrier`、`bar_write32`接到现有上位机驱动即可。Linux内核适配使用`READ_ONCE`/`le32_to_cpu`、`dma_rmb`及正确端序的有序MMIO写。
`rf_img_poll`用每条记录独立的sequence和尾部反码拒绝半包；同一cookie的ACCEPTED与APPLIED仍是不同sequence。查询会回写最近applied_cookie，可恢复被下一条回复覆盖的通知。
状态页在PCIe复位/驱动卸载并完成平台DMA排空之前保持映射，图像STOP并不停止参数查询回写。
参考C接口已用Clang14、C99、Wall/Wextra/Werror编译检查，mock自测试已在本地以DLL方式执行并返回0。该测试使用模拟回调验证参考接口，不代表已连接实际PCIe硬件；RTL协议仿真另外覆盖真实DMA与配置ACK交错、TX反压、非法命令、复位恢复和队列满。
