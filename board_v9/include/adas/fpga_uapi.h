/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
#ifndef PCIE_V2_UAPI_H
#define PCIE_V2_UAPI_H
/*
 * 内核与 Qt 共用的用户 ABI；只放固定宽度数据，不放指针或 DMA 地址。
 * 视频 ABI 保持 2，0x01~0x05 的结构和编号不变。
 * 驱动 7.0 适配 20261015 FPGA：新增 0x12 单向提交、0x13 驱动能力查询。
 * 旧 0x10/0x11 仅保留编号和布局用于识别，调用返回 EOPNOTSUPP。
 */
#include <linux/types.h>
#ifdef __KERNEL__
#include <linux/ioctl.h>
#else
#include <sys/ioctl.h>
#endif

#define FPGA_ABI_VERSION 2U
#define FPGA_FORMAT_RGB565 1U
#define FPGA_FRAME_SLOTS 4

struct fpga_info {
	__u32 abi_version, width, height, stride;
	__u32 image_bytes, slot_count, pixel_format, reserved;
	__aligned_u64 map_bytes;
	__aligned_u64 offset[FPGA_FRAME_SLOTS];
};

struct fpga_frame {
	__aligned_u64 session;
	__u32 slot, token, bytes_used, sequence;
	__aligned_u64 reserved;
};

/* 旧版配置结构，仅用于保持旧 ioctl 编号；新程序不使用。 */
struct fpga_image_config {
	__u32 words[5];
	__u32 cookie;
	__u32 reserved[2];
};

/* 以下状态码和状态结构仅属于已停用的旧接口。 */
#define FPGA_CFG_NONE 0U
#define FPGA_CFG_PENDING 1U
#define FPGA_CFG_APPLIED 2U
#define FPGA_CFG_REJECTED 3U

/*
 * 旧 GET_IMAGE 返回的状态页布局，仅保留以固定原 ioctl 编码。
 * 当前驱动不再构造此结构：旧 SET_IMAGE/GET_IMAGE 均返回 EOPNOTSUPP。
 */
struct fpga_image_status {
	__u32 record[16];
	__u32 commit_cookie;
	__u32 target_words[5];
	__u32 state;
	__u32 reserved;
};

#define FPGA_IOC_START _IO('P', 0x01)
#define FPGA_IOC_STOP _IO('P', 0x02)
#define FPGA_IOC_GET_INFO _IOR('P', 0x03, struct fpga_info)
#define FPGA_IOC_GET_FRAME _IOR('P', 0x04, struct fpga_frame)
#define FPGA_IOC_PUT_FRAME _IOW('P', 0x05, struct fpga_frame)
#define FPGA_IOC_SET_IMAGE _IOWR('P', 0x10, struct fpga_image_config)
#define FPGA_IOC_GET_IMAGE _IOR('P', 0x11, struct fpga_image_status)

/*
 * 新版只输入五个参数字，reserved 全零。驱动钳位后串行写五字及 COMMIT=1。
 * 返回 0 仅表示已执行提交写操作，不保证 FPGA 已接收或在帧边界应用。
 * 没有配置 DMA、cookie 或 APPLIED 查询；应用不得伪造硬件生效状态。
 * 使用 _IOW 避免提交后 copy_to_user 失败造成结果歧义，不自动重发提交。
 */
struct fpga_image_submit {
	__u32 words[5];
	__u32 reserved[3];
};

#define FPGA_CAPS_VERSION 1U
#define FPGA_CAP_IMAGE_SUBMIT (1U << 0)
/* 仅报告驱动能力，不代表已探测 FPGA 版本；当前 FPGA 无版本读回接口。 */
struct fpga_caps {
	__u32 version;
	__u32 flags;
	__u32 reserved[2];
};

#define FPGA_IOC_SUBMIT_IMAGE _IOW('P', 0x12, struct fpga_image_submit)
#define FPGA_IOC_GET_CAPS _IOR('P', 0x13, struct fpga_caps)
#endif
