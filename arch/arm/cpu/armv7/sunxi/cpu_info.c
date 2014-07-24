/*
 * (C) Copyright 2007-2011
 * Allwinner Technology Co., Ltd. <www.allwinnertech.com>
 * Tom Cubie <tangliang@allwinnertech.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <asm/io.h>
#include <asm/arch/cpu.h>

#ifdef CONFIG_DISPLAY_CPUINFO
int print_cpuinfo(void)
{
	if (IS_SUN4I()) {
		puts("CPU:   Allwinner A10 (SUN4I)\n");
	} else if (IS_SUN5I()) {
		u32 val = readl(SUNXI_SID_BASE + 0x08);
		switch ((val >> 12) & 0xf) {
		case 0: puts("CPU:   Allwinner A12 (SUN5I)\n"); break;
		case 3: puts("CPU:   Allwinner A13 (SUN5I)\n"); break;
		case 7: puts("CPU:   Allwinner A10s (SUN5I)\n"); break;
		default: puts("CPU:   Allwinner A1X (SUN5I)\n");
		}
	} else if (IS_SUN7I()) {
		puts("CPU:   Allwinner A20 (SUN7I)\n");
	} else {
		puts("CPU:   SUNXI Family\n");
	}
	return 0;
}
#endif
