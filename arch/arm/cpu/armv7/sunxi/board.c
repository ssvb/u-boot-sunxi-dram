/*
 * (C) Copyright 2012 Henrik Nordstrom <henrik@henriknordstrom.net>
 *
 * (C) Copyright 2007-2011
 * Allwinner Technology Co., Ltd. <www.allwinnertech.com>
 * Tom Cubie <tangliang@allwinnertech.com>
 *
 * Some init for sunxi platform.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <i2c.h>
#include <netdev.h>
#include <miiphy.h>
#include <serial.h>
#ifdef CONFIG_SPL_BUILD
#include <spl.h>
#endif
#include <asm/gpio.h>
#include <asm/io.h>
#include <asm/arch/clock.h>
#include <asm/arch/gpio.h>
#include <asm/arch/sys_proto.h>
#include <asm/arch/timer.h>

#include <linux/compiler.h>

#ifdef CONFIG_SPL_BUILD
/* Pointer to the global data structure for SPL */
DECLARE_GLOBAL_DATA_PTR;

/* The sunxi internal brom will try to loader external bootloader
 * from mmc0, nand flash, mmc2.
 * Unfortunately we can't check how SPL was loaded so assume
 * it's always the first SD/MMC controller
 */
u32 spl_boot_device(void)
{
	return BOOT_DEVICE_MMC1;
}

/* No confirmation data available in SPL yet. Hardcode bootmode */
u32 spl_boot_mode(void)
{
	return MMCSD_MODE_RAW;
}
#endif

static void sunxi_soc_detect_init(void)
{
	/* Enable VER_REG (set the VER_R_EN bit) */
	setbits_le32((u32 *)(SUNXI_SRAMC_BASE + 0x24), 1 << 15);
}

int soc_is_sun4i(void)
{
	return (readl((u32 *)(SUNXI_SRAMC_BASE + 0x24)) >> 16) == 0x1623;
}

int soc_is_sun5i(void)
{
	return (readl((u32 *)(SUNXI_SRAMC_BASE + 0x24)) >> 16) == 0x1625;
}

int soc_is_sun7i(void)
{
	return (readl((u32 *)(SUNXI_SRAMC_BASE + 0x24)) >> 16) == 0x1651;
}

int sunxi_cons_index(void)
{
	int cons_index = CONFIG_CONS_INDEX;

	if (cons_index == 1 && SOC_IS_SUN5I()) {
		u32 val = readl(SUNXI_SID_BASE + 0x08);
		if (((val >> 12) & 0xf) == 3) {
			/* Allwinner A13 */
			cons_index = 2;
		}
	}
	return cons_index;
}

struct serial_device *default_serial_console(void)
{
	if (sunxi_cons_index() == 1)
		return &eserial1_device;
	else
		return &eserial2_device;
}

int gpio_init(void)
{
	int cons_index = sunxi_cons_index();

	if (cons_index == 1 && (SOC_IS_SUN4I() || SOC_IS_SUN7I())) {
		sunxi_gpio_set_cfgpin(SUNXI_GPB(22), SUN4I_GPB22_UART0_TX);
		sunxi_gpio_set_cfgpin(SUNXI_GPB(23), SUN4I_GPB23_UART0_RX);
		sunxi_gpio_set_pull(SUNXI_GPB(23), 1);
	} else if (cons_index == 1 && SOC_IS_SUN5I()) {
		sunxi_gpio_set_cfgpin(SUNXI_GPB(19), SUN5I_GPB19_UART0_TX);
		sunxi_gpio_set_cfgpin(SUNXI_GPB(20), SUN5I_GPB20_UART0_RX);
		sunxi_gpio_set_pull(SUNXI_GPB(20), 1);
	} else if (cons_index == 2 && SOC_IS_SUN5I()) {
		sunxi_gpio_set_cfgpin(SUNXI_GPG(3), SUN5I_GPG3_UART1_TX);
		sunxi_gpio_set_cfgpin(SUNXI_GPG(4), SUN5I_GPG4_UART1_RX);
		sunxi_gpio_set_pull(SUNXI_GPG(4), 1);
	} else {
		/* Unsupported console port number.
		 * Please fix pin mux settings in board.c */
		hang();
	}
	return 0;
}

void reset_cpu(ulong addr)
{
	static const struct sunxi_wdog *wdog =
		 &((struct sunxi_timer_reg *)SUNXI_TIMER_BASE)->wdog;

	/* Set the watchdog for its shortest interval (.5s) and wait */
	writel(WDT_MODE_RESET_EN | WDT_MODE_EN, &wdog->mode);
	writel(WDT_CTRL_KEY | WDT_CTRL_RESTART, &wdog->ctl);

	while (1) {
		/* sun5i sometimes gets stuck without this */
		writel(WDT_MODE_RESET_EN | WDT_MODE_EN, &wdog->mode);
	}
}

/* do some early init */
void s_init(void)
{
	sunxi_soc_detect_init();
#if !defined CONFIG_SPL_BUILD
	int soc_is_sun6i = 0;
#ifdef CONFIG_SUN6I
	soc_is_sun6i = 1;
#endif
	if (SOC_IS_SUN7I() || soc_is_sun6i) {
		/* Enable SMP mode for CPU0, by setting bit 6 of
		 * Auxiliary Ctl reg */
		asm volatile(
			"mrc p15, 0, r0, c1, c0, 1\n"
			"orr r0, r0, #1 << 6\n"
			"mcr p15, 0, r0, c1, c0, 1\n" : : : "r0");
	}
#endif

	clock_init();
	timer_init();
	gpio_init();
	i2c_init_board();

#ifdef CONFIG_SPL_BUILD
	gd = &gdata;
	preloader_console_init();

#ifdef CONFIG_SPL_I2C_SUPPORT
	/* Needed early by sunxi_board_init if PMU is enabled */
	i2c_init(CONFIG_SYS_I2C_SPEED, CONFIG_SYS_I2C_SLAVE);
#endif
	sunxi_board_init();
#endif
}

#ifndef CONFIG_SYS_DCACHE_OFF
void enable_caches(void)
{
	/* Enable D-cache. I-cache is already enabled in start.S */
	dcache_enable();
}
#endif

#ifdef CONFIG_CMD_NET
/*
 * Initializes on-chip ethernet controllers.
 * to override, implement board_eth_init()
 */
int cpu_eth_init(bd_t *bis)
{
	__maybe_unused int rc;

#ifdef CONFIG_MACPWR
	gpio_direction_output(CONFIG_MACPWR, 1);
	mdelay(200);
#endif

#ifdef CONFIG_SUNXI_EMAC
	rc = sunxi_emac_initialize(bis);
	if (rc < 0) {
		printf("sunxi: failed to initialize emac\n");
		return rc;
	}
#endif

#ifdef CONFIG_SUNXI_GMAC
	rc = sunxi_gmac_initialize(bis);
	if (rc < 0) {
		printf("sunxi: failed to initialize gmac\n");
		return rc;
	}
#endif

	return 0;
}
#endif
