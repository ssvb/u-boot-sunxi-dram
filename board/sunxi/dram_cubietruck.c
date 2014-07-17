/* this file is generated, don't edit it yourself */

#include <common.h>
#include <asm/arch/dram.h>

static struct dram_para dram_para = { /* DRAM timings: 9-8-8-22 (600 MHz) */
	.clock            = 600,
	.mbus_clock       = 400,
	.type             = 3,
	.rank_num         = 1,
	.density          = 4096,
	.io_width         = 8,
	.bus_width        = 32,
	.cas              = 9,
	.zq               = 0x2c,
	.odt_en           = 3,
	.tpr0             = 0x3c9688b4,
	.tpr1             = 0xa090,
	.tpr2             = 0x2be00,
	.tpr3             = 0x021111,
	.tpr4             = 0x1,
	.tpr5             = 0x0,
	.emr1             = 0x42,
	.emr2             = 0x10,
	.emr3             = 0x0,
	.dqs_gating_delay = 0x07070707,
	.active_windowing = 1,
};

unsigned long sunxi_dram_init(void)
{
	return dramc_init(&dram_para);
}
