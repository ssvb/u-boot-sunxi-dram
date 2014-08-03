/*
 * (C) Copyright 2012-2013 Henrik Nordstrom <henrik@henriknordstrom.net>
 *
 * Configuration settings for the Allwinner A10 (sun4i) CPU
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */
#ifndef __CONFIG_H
#define __CONFIG_H

#define SOC_IS_SUN4I()			(soc_is_sun4i())
#define SOC_IS_SUN5I()			(soc_is_sun5i())
#define SOC_IS_SUN7I()			(soc_is_sun7i())

#define CONFIG_SUN4I_SUN5I_SUN7I
#define CONFIG_CLK_FULL_SPEED		(SOC_IS_SUN7I() ? 912000000 : \
							  1008000000)

#define CONFIG_SYS_PROMPT		"sunxi# "

/* The Cortex-A8 CPU in sun4i/sun5i is going to fail runtime
 * check and will fallback to booting the kernel in secure mode */
#define CONFIG_ARMV7_ALLOW_SECURE_MODE_FALLBACK 1

/* This is going to be used for sun7i */
#define CONFIG_ARMV7_VIRT		1
#define CONFIG_ARMV7_NONSEC		1
#define CONFIG_ARMV7_PSCI		1
#define CONFIG_ARMV7_PSCI_NR_CPUS	2
#define CONFIG_ARMV7_SECURE_BASE	SUNXI_SRAM_B_BASE
#define CONFIG_SYS_CLK_FREQ		24000000

/*
 * Include common sunxi configuration where most the settings are
 */
#include <configs/sunxi-common.h>

#endif /* __CONFIG_H */
