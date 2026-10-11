> 本文件用于本板工程的DVIO参数位定义。板级顶层是hdmi_ddr_pcie，实际接入方式以IMG_BEFORE_DDR.md为准；文中独立IMG PDS指算法来源工程。

# 图像处理 DVIO 上板控制（2026-10-07）

## 当前交付范围

已将IMG图像链路的开关、去雾系数和强度参数接成运行时控制信号，保留原参数模式。
`image_processing_top` 新增 `RUNTIME_CONTROL`：设置为1才启用DVIO控制；设0使用原参数模式，设1使用运行时输入。
新增输入 `cfg_data[159:0]`、`cfg_apply`；输出 `cfg_active[159:0]`、`cfg_pending`、`cfg_ack`。
已接入你生成的 `ipcore/vio/vio.v` 和 `ipcore/vio_jtag_hub/vio_jtag_hub.v`，保留厂商生成源码。
新增 `source/src/img/image_processing_dvio_top.v`，内含DVIO、默认JTAG Hub和运行时图像处理链路。
IMG PDS顶层选为 `image_processing_dvio_top`，并登记两套IP的.idf和全部source RTL；
vio.idf中已有vio.fdc约束登记，不把约束或 *_tmpl.v 模板当作RTL编译。
原 `image_processing_top` 仍可独立例化，当前文件RUNTIME_CONTROL默认值按你修改的1保留。
普通静态仿真应显式设RUNTIME_CONTROL=0，运行时仿真则接cfg_data/cfg_apply。

实际HDMI板级顶层应例化 `image_processing_dvio_top`，连接clk/rst_n、pre_*输入和post_*输出。
clk接持续运行的像素时钟，rst_n接该域的低有效复位，不能接JTAG drck。
该封装不含HDMI/DDR/PLL或板级引脚约束，输入流仍须符合下文VS协议。
已有JTAG Hub的板级工程须复用已有Hub，不能同时例化第二个默认SCANCHAIN。

## IP Compiler配置

手册：UG062007_DVIO_IP.pdf（V1.0_i1），重点第10、13—19页。当前安装包有V1.0.a，应选择这一版；手册第3页说明它修复了probe_out输出异常。

1. System → Tools → Debug → DVIO，Instance Name=`vio`。
2. Input Probe Count=6，Output Probe Count=6。输入活动检测器可勾选。
3. Probe_out0～4的宽度均为32，Probe_out5宽度为1。初始值按下表，显示格式选择Hex。
4. Probe_in0～4宽度均为32，Probe_in5宽度为8。
5. 再生成DVIO_Jtag_Hub，Instance Name=`vio_jtag_hub`，Ctrl_Port_Num=1，Select Jtag=Default JTAG。
6. 将生成器要求的顶层.v和其RTL/约束文件加入实际板级PDS。不要只加入.idf或例化模板。
7. DVIO的clk接图像处理的pixel_clk；resetn_i和算法rst_n接该时钟域有效的低有效复位。
   drck来自Hub，不能拿drck驱动图像处理逻辑。已有JTAG Hub时应复用并分配空闲通道，不重复占用SCANCHAIN。

| 输出端口 | 位宽 | 初始Hex值 | 用途 |
|---|---:|---|---|
| probe_out0 | 32 | 000001C0 | 模块开关 |
| probe_out1 | 32 | 00F38034 | 去雾参数 |
| probe_out2 | 32 | 00004100 | Y降噪参数 |
| probe_out3 | 32 | 02030190 | CLAHE参数 |
| probe_out4 | 32 | 0080C040 | CLAHE增益上限和高光参数 |
| probe_out5 | 1 | 0 | 提交翻转位，每次提交0↔1 |

初始时DCP/Gamma/CLAHE/高光/Y降噪/t滤波均关闭，视频走等延迟旁路；天空/防晕/阴影配置预置为开启，但上游模块关闭时不产生处理效果。
这些初始值匹配未覆盖参数时的算法复位值。若改编译默认参数，也要同步DVIO初始值。

## 五个参数字的位定义

`cfg_data={probe_out4,probe_out3,probe_out2,probe_out1,probe_out0}`，未使用位写0。

| 参数字 | 位 | 参数 | 范围/含义 |
|---|---|---|---|
| word0 | 0 | DCP开关 | 0关、1开；夜间测试设0 |
| word0 | 1 | Gamma开关 | 固定原Gamma曲线，0关、1开 |
| word0 | 2 | CLAHE开关 | 0关、1开 |
| word0 | 3 | 高光保护开关 | 0关、1开 |
| word0 | 4 | CLAHE前Y双边降噪 | 0关、1开 |
| word0 | 5 | 透射率t双边滤波 | 0关、1开 |
| word0 | 6 | 天空保护 | 0关、1开 |
| word0 | 7 | CLAHE防光晕保护 | 0关、1开 |
| word0 | 8 | CLAHE阴影提升保护 | 0关、1开 |
| word1 | 7:0 | TX_MIN | 1～255，0钳为1 |
| word1 | 15:8 | SKY_TX_MIN | 1～255，小于TX_MIN时钳为TX_MIN |
| word1 | 23:16 | OMEGA | 0～255，原值243；t=255−floor(OMEGA×归一化暗通道/255) |
| word2 | 8:0 | DENOISE_MIX | 0～256，256为全滤波，0为原Y；超范围钳为256 |
| word2 | 14:9 | DENOISE_SIGMA | 16、32、48；其他值按32 |
| word3 | 15:0 | CLAHE_CLIP_LIMIT | 1～65535，0钳为1；1080p原值400 |
| word3 | 18:16 | CLAHE_STRENGTH | 0～4；0原Y，4最大；超范围钳为4 |
| word3 | 25:19 | CLAHE_SHADOW_GAIN | 0～64；超范围钳为64 |
| word4 | 7:0 | CLAHE_BOOST_LIMIT | 0～255，限制受保护暗部的正向提升 |
| word4 | 15:8 | HIGHLIGHT_KNEE | 0～255，原值192 |
| word4 | 24:16 | HIGHLIGHT_SLOPE | 0～256，原值128；256表示软膝段斜率为1 |

`PIC_WIDTH/PIC_HEIGHT/H_BLANK`、各`*_CAPABLE`、`CLAHE_DONOR_MODE`、`BILATERAL_GUIDE_ENABLE`仍是编译参数。
其中DONOR_MODE改变CDF/插值结构，GUIDE_ENABLE改变滤波导引窗口结构，修改它们需重新综合。Gamma指数没有新增运行时输入，当前仍使用原LUT曲线。

## 提交与状态回读

先写word0～4，最后将probe_out5翻转一次。不要发窄脉冲，也不要把它当一直有效的使能。
控制器留出同步/稳定时间，将参数整体暂存，在下一次输入VS下降沿一次生效。
从提交到`cfg_ack`等于probe_out5之前，保持五个参数字不变，也不要再次翻转提交位。
如果暂时没有视频VS，pending会等待；恢复帧同步后应用，不在图像中途强行切换。

probe_in0～4分别回读已钳位、已提交的word0～4；这比查看probe_out请求值更能确认实际设置。
probe_in5[0]=ack，[1]=pending，[5:2]=active_config（顺序为{DCP,Gamma,CLAHE,高光}），[6]=高光硬件支持，[7]=输出VS。
ack表示输入帧配置已经锁存；输出图像还要经过固定流水延迟。改变去雾/Gamma/降噪/clip等参数后，CLAHE清除旧统计并重新建立CDF，观察时至少等待3个完整视频帧。

## 建议的第一次上板对照

下面是调试起点，需用实际镜头和曝光结果微调，不代表已经做过板上画质验证。

| 场景 | word0 | word1 | word2 | word3 | word4 |
|---|---|---|---|---|---|
| 原图旁路 | 000001C0 | 00F38034 | 00004100 | 02030190 | 0080C040 |
| 雾天，仅DCP+天空保护 | 000001C1 | 00F38034 | 00004100 | 02030190 | 0080C040 |
| 雾天，DCP→Gamma→CLAHE→高光 | 000001CF | 00F38034 | 00004100 | 02030190 | 0080C040 |
| 夜间，Y降噪→Gamma→高光，DCP关 | 000001DA | 00F38034 | 00004080 | 01020100 | 0080C018 |
| 夜间，再开启温和CLAHE | 000001DE | 00F38034 | 00004080 | 01020100 | 0080C018 |
| 全部处理和两路滤波开启 | 000001FF | 00F38034 | 00004100 | 02030190 | 0080C040 |

夜间先对照CLAHE关闭/开启，若噪声突出优先降低STRENGTH、SHADOW_GAIN或BOOST_LIMIT。Y降噪在Gamma、CLAHE之前；t滤波只属于去雾部分。

## 时序与验证边界

RUNTIME_CONTROL=1保留两套滤波硬件以便在线开关，关滤波仅选择等延迟原数据，不节省其硬件资源。
默认1920+280行周期下，输入到输出固定4454个像素时钟：DCP=2224，增强=2230。改变运行时开关不改变这一延迟。
控制参数用帧事件延迟寄存器对齐到各处理级，不为160位参数额外建立整行RAM。
输入VS下降沿是当前代码定义的帧边界；沿用原图像流协议（有效图像期间VS为高），不要未经极性/时序适配直接替换成相反极性的HDMI同步。
板级HDMI/DDR模块应使用本模块输出的同步/使能，不能继续沿用旧的22拍固定补偿。
运行时版本新增可变系数逻辑与多档LUT，资源和148.5MHz时序需在实际板级工程重新综合、布局布线确认；本次不宣称已通过板上时序或DRM预算。

小规模验证脚本：在source/tb运行`do run_dvio_tests.do`。覆盖提交握手、参数钳位，以及运行时版本与对应固定参数版本的像素/同步对照；不用1080p图片完成这项控制逻辑检查。

参数字可用同目录工具生成，例如：`python build_dvio_words.py --preset night_clahe --strength 2 --boost 16`。它只输出Hex值，不连接或控制硬件。

若你的HDMI板级PDS在其他目录，除原IMG依赖外还需登记 `source/src/common/frame_config_delay.v` 和 `source/src/img/image_runtime_control.v`，再例化image_processing_dvio_top并登记两套生成IP；本次将IMG PDS选为DVIO封装顶层。已打开的PDS需要重新载入更新后的文件列表。


## 单独调节接口的示例

五个参数接口可独立修改，拼成160位仅用于保持一帧内配置一致，并不要求每次重写全部接口。
probe_out5不参与参数拼接，它单独控制提交。其他字保持原值时，修改一个字再提交即可。
例如初始word0=000001C0，将其改为000001C2，只开启Gamma，再把probe_out5从0改1；
待probe_in5[0]=1且[1]=0后已应用。之后只调CLAHE强度，修改word3[18:16]，
保持word0/1/2/4及word3其余字段不变，再把probe_out5从1改0。
同一个32位字里有多个字段，写字时需保留不打算修改的字段。实时写入不会立即改变正在处理的一帧。
