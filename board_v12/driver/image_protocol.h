/* SPDX-License-Identifier: GPL-2.0 */
#ifndef PCIE_V2_IMAGE_PROTOCOL_H
#define PCIE_V2_IMAGE_PROTOCOL_H
#include "fpga_uapi.h"

/* 与 FPGA image_runtime_control.sanitize_config 逐位对应。 */
static inline __u32 fpga_cfg_clip(__u32 n, __u32 top)
{
	return n > top ? top : n;
}

static inline void fpga_cfg_sanitize(const __u32 in[5], __u32 out[5])
{
	__u32 floor = in[1] & 255U, sky = (in[1] >> 8) & 255U;
	__u32 sigma = (in[2] >> 9) & 63U;
	__u32 clip = in[3] & 65535U;

	if (!floor)
		floor = 1;
	if (sky < floor)
		sky = floor;
	if (sigma != 16 && sigma != 48)
		sigma = 32;
	out[0] = in[0] & 0x19ffU; /* 包含两个旧亮度开关，清除保留位 */
	out[1] = floor | (sky << 8) | (in[1] & 0x00ff0000U);
	out[2] = fpga_cfg_clip(in[2] & 511U, 256U) | (sigma << 9) |
		 (in[2] & 0xffff0000U);
	out[3] = (clip ? clip : 1U) |
		 (fpga_cfg_clip((in[3] >> 16) & 7U, 4U) << 16) |
		 (fpga_cfg_clip((in[3] >> 19) & 127U, 64U) << 19);
	out[4] = (in[4] & 65535U) |
		 (fpga_cfg_clip((in[4] >> 16) & 511U, 256U) << 16);
}

#endif
