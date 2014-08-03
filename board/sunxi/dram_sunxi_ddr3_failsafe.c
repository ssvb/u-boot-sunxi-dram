/* this file is generated, don't edit it yourself */

#include "common.h"
#include <asm/arch/dram.h>

static struct dram_para dram_para = { /* DRAM timings: 6-5-5-13 (360 MHz) */
	.clock = 360,
	.type = 3,
	.rank_num = 1,
	.cas = 6,
	.zq = 0x7b,
	.odt_en = 0,
	.tpr0 = 0x248d5590,
	.tpr1 = 0xa088,
	.tpr2 = 0x22a00,
	.tpr3 = 0x0,
	.tpr4 = 0x0,
	.tpr5 = 0x0,
	.emr1 = 0x0,
	.emr2 = 0x0,
	.emr3 = 0x0,
	.active_windowing = 1,
};

unsigned long sunxi_dram_init(void)
{
	return dramc_init(&dram_para);
}
