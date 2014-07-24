/*
 * sunxi DRAM controller initialization
 * (C) Copyright 2012 Henrik Nordstrom <henrik@henriknordstrom.net>
 * (C) Copyright 2013 Luke Kenneth Casson Leighton <lkcl@lkcl.net>
 *
 * Based on sun4i Linux kernel sources mach-sunxi/pm/standby/dram*.c
 * and earlier U-Boot Allwiner A10 SPL work
 *
 * (C) Copyright 2007-2012
 * Allwinner Technology Co., Ltd. <www.allwinnertech.com>
 * Berg Xing <bergxing@allwinnertech.com>
 * Tom Cubie <tangliang@allwinnertech.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

/*
 * Unfortunately the only documentation we have on the sun7i DRAM
 * controller is Allwinner boot0 + boot1 code, and that code uses
 * magic numbers & shifts with no explanations. Hence this code is
 * rather undocumented and full of magic.
 */

#include <common.h>
#include <asm/io.h>
#include <asm/arch/clock.h>
#include <asm/arch/dram.h>
#include <asm/arch/timer.h>
#include <asm/arch/sys_proto.h>

#define CPU_CFG_CHIP_VER(n) ((n) << 6)
#define CPU_CFG_CHIP_VER_MASK CPU_CFG_CHIP_VER(0x3)
#define CPU_CFG_CHIP_REV_A 0x0
#define CPU_CFG_CHIP_REV_C1 0x1
#define CPU_CFG_CHIP_REV_C2 0x2
#define CPU_CFG_CHIP_REV_B 0x3

/*
 * Wait up to 1s for mask to be clear in given reg.
 */
static void await_completion(u32 *reg, u32 mask)
{
	unsigned long tmo = timer_get_us() + 1000000;

	while (readl(reg) & mask) {
		if (timer_get_us() > tmo)
			panic("Timeout initialising DRAM\n");
	}
}

/*
 * Wait up to 1s for mask to be set in given reg.
 */
static void await_bits_set(u32 *reg, u32 mask)
{
	unsigned long tmo = timer_get_us() + 1000000;

	while ((readl(reg) & mask) != mask) {
		if (timer_get_us() > tmo)
			panic("Timeout initialising DRAM\n");
	}
}

/*
 * This performs the external DRAM reset by driving the RESET pin low and
 * then high again. According to the DDR3 spec, the RESET pin needs to be
 * kept low for at least 200 us.
 */
static void mctl_ddr3_reset(void)
{
	struct sunxi_dram_reg *dram =
			(struct sunxi_dram_reg *)SUNXI_DRAMC_BASE;

	struct sunxi_timer_reg *timer =
			(struct sunxi_timer_reg *)SUNXI_TIMER_BASE;
	u32 reg_val;

	if (IS_SUN4I()) {
		writel(0, &timer->cpu_cfg);
		reg_val = readl(&timer->cpu_cfg);
	}

	if (IS_SUN4I() && (reg_val & CPU_CFG_CHIP_VER_MASK) !=
	    CPU_CFG_CHIP_VER(CPU_CFG_CHIP_REV_A)) {
		setbits_le32(&dram->mcr, DRAM_MCR_RESET);
		udelay(200);
		clrbits_le32(&dram->mcr, DRAM_MCR_RESET);
	} else
	{
		clrbits_le32(&dram->mcr, DRAM_MCR_RESET);
		udelay(200);
		setbits_le32(&dram->mcr, DRAM_MCR_RESET);
	}
}

static void mctl_set_drive(void)
{
	struct sunxi_dram_reg *dram = (struct sunxi_dram_reg *)SUNXI_DRAMC_BASE;

	clrsetbits_le32(&dram->mcr, DRAM_MCR_MODE_NORM(0x3) | (0x3 << 28),
			DRAM_MCR_MODE_EN(0x3) |
			0xffc);
}

static void mctl_itm_disable(void)
{
	struct sunxi_dram_reg *dram = (struct sunxi_dram_reg *)SUNXI_DRAMC_BASE;

	clrsetbits_le32(&dram->ccr, DRAM_CCR_INIT, DRAM_CCR_ITM_OFF);
}

static void mctl_itm_enable(void)
{
	struct sunxi_dram_reg *dram = (struct sunxi_dram_reg *)SUNXI_DRAMC_BASE;

	clrbits_le32(&dram->ccr, DRAM_CCR_ITM_OFF);
}

static void mctl_itm_reset(void)
{
	mctl_itm_disable();
	udelay(1); /* ITM reset needs a bit of delay */
	mctl_itm_enable();
	udelay(1);
}

static void mctl_enable_dll0(u32 phase)
{
	struct sunxi_dram_reg *dram = (struct sunxi_dram_reg *)SUNXI_DRAMC_BASE;

	clrsetbits_le32(&dram->dllcr[0], 0x3f << 6,
			((phase >> 16) & 0x3f) << 6);
	clrsetbits_le32(&dram->dllcr[0], DRAM_DLLCR_NRESET, DRAM_DLLCR_DISABLE);
	udelay(2);

	clrbits_le32(&dram->dllcr[0], DRAM_DLLCR_NRESET | DRAM_DLLCR_DISABLE);
	udelay(22);

	clrsetbits_le32(&dram->dllcr[0], DRAM_DLLCR_DISABLE, DRAM_DLLCR_NRESET);
	udelay(22);
}

/* Get the number of DDR byte lanes */
static u32 mctl_get_number_of_lanes(void)
{
	struct sunxi_dram_reg *dram = (struct sunxi_dram_reg *)SUNXI_DRAMC_BASE;
	switch (readl(&dram->dcr) & DRAM_DCR_BUS_WIDTH_MASK) {
	case DRAM_DCR_BUS_WIDTH(DRAM_DCR_BUS_WIDTH_32BIT):
		return 4;
	case DRAM_DCR_BUS_WIDTH(DRAM_DCR_BUS_WIDTH_16BIT):
		return 2;
	default:
		return 1;
	}
}

/*
 * Note: This differs from pm/standby in that it checks the bus width
 */
static void mctl_enable_dllx(u32 phase)
{
	struct sunxi_dram_reg *dram = (struct sunxi_dram_reg *)SUNXI_DRAMC_BASE;
	u32 i, number_of_lanes;

	number_of_lanes = mctl_get_number_of_lanes();

	for (i = 1; i <= number_of_lanes; i++) {
		clrsetbits_le32(&dram->dllcr[i], 0xf << 14,
				(phase & 0xf) << 14);
		clrsetbits_le32(&dram->dllcr[i], DRAM_DLLCR_NRESET,
				DRAM_DLLCR_DISABLE);
		phase >>= 4;
	}
	udelay(2);

	for (i = 1; i <= number_of_lanes; i++)
		clrbits_le32(&dram->dllcr[i], DRAM_DLLCR_NRESET |
			     DRAM_DLLCR_DISABLE);
	udelay(22);

	for (i = 1; i <= number_of_lanes; i++)
		clrsetbits_le32(&dram->dllcr[i], DRAM_DLLCR_DISABLE,
				DRAM_DLLCR_NRESET);
	udelay(22);
}

static u32 hpcr_value_sun5i[32] = {
	0, 0, 0, 0,
	0, 0, 0, 0,
	0, 0, 0, 0,
	0, 0, 0, 0,
	0x1031, 0x1031, 0x0735, 0x1035,
	0x1035, 0x0731, 0x1031, 0,
	0x0301, 0x0301, 0x0301, 0x0301,
	0x0301, 0x0301, 0x0301, 0
};

static u32 hpcr_value_sun4i[32] = {
	0x0301, 0x0301, 0x0301, 0x0301,
	0x0301, 0x0301, 0, 0,
	0, 0, 0, 0,
	0, 0, 0, 0,
	0x1031, 0x1031, 0x0735, 0x5031,
	0x1035, 0x0731, 0x1031, 0x0735,
	0x1035, 0x1031, 0x0731, 0x1035,
	0x1031, 0x0301, 0x0301, 0x0731
};

static u32 hpcr_value_sun7i[32] = {
	0x0301, 0x0301, 0x0301, 0x0301,
	0x0301, 0x0301, 0x0301, 0x0301,
	0, 0, 0, 0,
	0, 0, 0, 0,
	0x1031, 0x1031, 0x0735, 0x1035,
	0x1035, 0x0731, 0x1031, 0x0735,
	0x1035, 0x1031, 0x0731, 0x1035,
	0x0001, 0x1031, 0, 0x1031
	/* last row differs from boot0 source table
	 * 0x1031, 0x0301, 0x0301, 0x0731
	 * but boot0 code skips #28 and #30, and sets #29 and #31 to the
	 * value from #28 entry (0x1031)
	 */
};

static void mctl_configure_hostport(void)
{
	struct sunxi_dram_reg *dram = (struct sunxi_dram_reg *)SUNXI_DRAMC_BASE;
	u32 i;
	u32 *hpcr_value;
	if (IS_SUN4I())
		hpcr_value = hpcr_value_sun4i;
	else if (IS_SUN5I())
		hpcr_value = hpcr_value_sun5i;
	else if (IS_SUN7I())
		hpcr_value = hpcr_value_sun7i;
	else
		panic("Can't detect the SoC type");

	for (i = 0; i < 32; i++)
		writel(hpcr_value[i], &dram->hpcr[i]);
}

static void mctl_setup_dram_clock(u32 clk, u32 mbus_clk)
{
	u32 reg_val;
	struct sunxi_ccm_reg *ccm = (struct sunxi_ccm_reg *)SUNXI_CCM_BASE;

	/* PLL5P and PLL6 are the potential clock sources for MBUS */
	u32 pll6x_div, pll5p_div;
	u32 pll6x_clk = clock_get_pll6() / 1000000;
	u32 pll5p_clk = clk / 24 * 24;

	if (IS_SUN7I())
		pll6x_clk *= 2; /* sun7i uses PLL6*2, sun5i uses just PLL6 */

	/* setup DRAM PLL */
	reg_val = readl(&ccm->pll5_cfg);
	reg_val &= ~CCM_PLL5_CTRL_M_MASK;		/* set M to 0 (x1) */
	reg_val &= ~CCM_PLL5_CTRL_K_MASK;		/* set K to 0 (x1) */
	reg_val &= ~CCM_PLL5_CTRL_N_MASK;		/* set N to 0 (x0) */
	reg_val &= ~CCM_PLL5_CTRL_P_MASK;		/* set P to 0 (x1) */
	if (clk >= 540 && clk < 552) {
		/* dram = 540MHz, pll5p = 540MHz */
		pll5p_clk = 540;
		reg_val |= CCM_PLL5_CTRL_M(CCM_PLL5_CTRL_M_X(2));
		reg_val |= CCM_PLL5_CTRL_K(CCM_PLL5_CTRL_K_X(3));
		reg_val |= CCM_PLL5_CTRL_N(CCM_PLL5_CTRL_N_X(15));
		reg_val |= CCM_PLL5_CTRL_P(1);
	} else if (clk >= 512 && clk < 528) {
		/* dram = 512MHz, pll5p = 384MHz */
		pll5p_clk = 384;
		reg_val |= CCM_PLL5_CTRL_M(CCM_PLL5_CTRL_M_X(3));
		reg_val |= CCM_PLL5_CTRL_K(CCM_PLL5_CTRL_K_X(4));
		reg_val |= CCM_PLL5_CTRL_N(CCM_PLL5_CTRL_N_X(16));
		reg_val |= CCM_PLL5_CTRL_P(2);
	} else if (clk >= 496 && clk < 504) {
		/* dram = 496MHz, pll5p = 372MHz */
		pll5p_clk = 372;
		reg_val |= CCM_PLL5_CTRL_M(CCM_PLL5_CTRL_M_X(3));
		reg_val |= CCM_PLL5_CTRL_K(CCM_PLL5_CTRL_K_X(2));
		reg_val |= CCM_PLL5_CTRL_N(CCM_PLL5_CTRL_N_X(31));
		reg_val |= CCM_PLL5_CTRL_P(2);
	} else if (clk >= 468 && clk < 480) {
		/* dram = 468MHz, pll5p = 468MHz */
		pll5p_clk = 468;
		reg_val |= CCM_PLL5_CTRL_M(CCM_PLL5_CTRL_M_X(2));
		reg_val |= CCM_PLL5_CTRL_K(CCM_PLL5_CTRL_K_X(3));
		reg_val |= CCM_PLL5_CTRL_N(CCM_PLL5_CTRL_N_X(13));
		reg_val |= CCM_PLL5_CTRL_P(1);
	} else if (clk >= 396 && clk < 408) {
		/* dram = 396MHz, pll5p = 396MHz */
		pll5p_clk = 396;
		reg_val |= CCM_PLL5_CTRL_M(CCM_PLL5_CTRL_M_X(2));
		reg_val |= CCM_PLL5_CTRL_K(CCM_PLL5_CTRL_K_X(3));
		reg_val |= CCM_PLL5_CTRL_N(CCM_PLL5_CTRL_N_X(11));
		reg_val |= CCM_PLL5_CTRL_P(1);
	} else 	{
		/* any other frequency that is a multiple of 24 */
		reg_val |= CCM_PLL5_CTRL_M(CCM_PLL5_CTRL_M_X(2));
		reg_val |= CCM_PLL5_CTRL_K(CCM_PLL5_CTRL_K_X(2));
		reg_val |= CCM_PLL5_CTRL_N(CCM_PLL5_CTRL_N_X(clk / 24));
		reg_val |= CCM_PLL5_CTRL_P(CCM_PLL5_CTRL_P_X(2));
	}
	reg_val &= ~CCM_PLL5_CTRL_VCO_GAIN;		/* PLL VCO Gain off */
	reg_val |= CCM_PLL5_CTRL_EN;			/* PLL On */
	writel(reg_val, &ccm->pll5_cfg);
	udelay(5500);

	setbits_le32(&ccm->pll5_cfg, CCM_PLL5_CTRL_DDR_CLK);

	if (IS_SUN4I() || IS_SUN7I()) {
		/* reset GPS */
		clrbits_le32(&ccm->gps_clk_cfg, CCM_GPS_CTRL_RESET | CCM_GPS_CTRL_GATE);
		setbits_le32(&ccm->ahb_gate0, CCM_AHB_GATE_GPS);
		udelay(1);
		clrbits_le32(&ccm->ahb_gate0, CCM_AHB_GATE_GPS);
	}

	/* setup MBUS clock */
	if (!mbus_clk)
		mbus_clk = 300;
	pll6x_div = (pll6x_clk + mbus_clk - 1) / mbus_clk;
	pll5p_div = (pll5p_clk + mbus_clk - 1) / mbus_clk;

	if (pll6x_div <= 16 && pll6x_clk / pll6x_div > pll5p_clk / pll5p_div) {
		/* use PLL6 as the MBUS clock source */
		reg_val = CCM_MBUS_CTRL_GATE |
			  CCM_MBUS_CTRL_CLK_SRC(CCM_MBUS_CTRL_CLK_SRC_PLL6) |
			  CCM_MBUS_CTRL_N(CCM_MBUS_CTRL_N_X(1)) |
			  CCM_MBUS_CTRL_M(CCM_MBUS_CTRL_M_X(pll6x_div));
	} else if (pll5p_div <= 16) {
		/* use PLL5P as the MBUS clock source */
		reg_val = CCM_MBUS_CTRL_GATE |
			  CCM_MBUS_CTRL_CLK_SRC(CCM_MBUS_CTRL_CLK_SRC_PLL5) |
			  CCM_MBUS_CTRL_N(CCM_MBUS_CTRL_N_X(1)) |
			  CCM_MBUS_CTRL_M(CCM_MBUS_CTRL_M_X(pll5p_div));
	} else {
		panic("Bad mbus_clk\n");
	}
	writel(reg_val, &ccm->mbus_clk_cfg);

	/*
	 * open DRAMC AHB & DLL register clock
	 * close it first
	 */
	clrbits_le32(&ccm->ahb_gate0, CCM_AHB_GATE_SDRAM | CCM_AHB_GATE_DLL);
	udelay(22);

	/* then open it */
	setbits_le32(&ccm->ahb_gate0, CCM_AHB_GATE_SDRAM | CCM_AHB_GATE_DLL);
	udelay(22);
}

/*
 * The data from rslrX and rdgrX registers (X=rank) is stored
 * in a single 32-bit value using the following format:
 *   bits [31:26] - DQS gating system latency for byte lane 3
 *   bits [25:24] - DQS gating phase select for byte lane 3
 *   bits [23:18] - DQS gating system latency for byte lane 2
 *   bits [17:16] - DQS gating phase select for byte lane 2
 *   bits [15:10] - DQS gating system latency for byte lane 1
 *   bits [ 9:8 ] - DQS gating phase select for byte lane 1
 *   bits [ 7:2 ] - DQS gating system latency for byte lane 0
 *   bits [ 1:0 ] - DQS gating phase select for byte lane 0
 */
static void mctl_set_dqs_gating_delay(int rank, u32 dqs_gating_delay)
{
	struct sunxi_dram_reg *dram = (struct sunxi_dram_reg *)SUNXI_DRAMC_BASE;
	u32 lane, number_of_lanes = mctl_get_number_of_lanes();
	/* rank0 gating system latency (3 bits per lane: cycles) */
	u32 slr = readl(rank == 0 ? &dram->rslr0 : &dram->rslr1);
	/* rank0 gating phase select (2 bits per lane: 90, 180, 270, 360) */
	u32 dgr = readl(rank == 0 ? &dram->rdgr0 : &dram->rdgr1);
	for (lane = 0; lane < number_of_lanes; lane++) {
		u32 tmp = dqs_gating_delay >> (lane * 8);
		slr &= ~(7 << (lane * 3));
		slr |= ((tmp >> 2) & 7) << (lane * 3);
		dgr &= ~(3 << (lane * 2));
		dgr |= (tmp & 3) << (lane * 2);
	}
	writel(slr, rank == 0 ? &dram->rslr0 : &dram->rslr1);
	writel(dgr, rank == 0 ? &dram->rdgr0 : &dram->rdgr1);
}

static int dramc_scan_readpipe(void)
{
	struct sunxi_dram_reg *dram = (struct sunxi_dram_reg *)SUNXI_DRAMC_BASE;
	u32 reg_val;

	/* data training trigger */
	clrbits_le32(&dram->csr, DRAM_CSR_FAILED);
	setbits_le32(&dram->ccr, DRAM_CCR_DATA_TRAINING);

	/* check whether data training process has completed */
	await_completion(&dram->ccr, DRAM_CCR_DATA_TRAINING);

	/* check data training result */
	reg_val = readl(&dram->csr);
	if (reg_val & DRAM_CSR_FAILED)
		return -1;

	return 0;
}

static void dramc_clock_output_en(u32 on)
{
	if (IS_SUN5I() || IS_SUN7I()) {
		struct sunxi_dram_reg *dram = (struct sunxi_dram_reg *)SUNXI_DRAMC_BASE;

		if (on)
			setbits_le32(&dram->mcr, DRAM_MCR_DCLK_OUT);
		else
			clrbits_le32(&dram->mcr, DRAM_MCR_DCLK_OUT);
	}
	if (IS_SUN4I()) {
		struct sunxi_ccm_reg *ccm = (struct sunxi_ccm_reg *)SUNXI_CCM_BASE;
		if (on)
			setbits_le32(&ccm->dram_clk_cfg, CCM_DRAM_CTRL_DCLK_OUT);
		else
			clrbits_le32(&ccm->dram_clk_cfg, CCM_DRAM_CTRL_DCLK_OUT);
	}
}

/* tRFC in nanoseconds for different densities (from the DDR3 spec) */
static const u16 tRFC_DDR3_table[6] = {
	/* 256Mb    512Mb    1Gb      2Gb      4Gb      8Gb */
	   90,      90,      110,     160,     300,     350
};

static void dramc_set_autorefresh_cycle(u32 clk, u32 density)
{
	struct sunxi_dram_reg *dram = (struct sunxi_dram_reg *)SUNXI_DRAMC_BASE;
	u32 tRFC, tREFI;

	tRFC = (tRFC_DDR3_table[density] * clk + 999) / 1000;
	tREFI = (7987 * clk) >> 10;	/* <= 7.8us */

	writel(DRAM_DRR_TREFI(tREFI) | DRAM_DRR_TRFC(tRFC), &dram->drr);
}

/* Calculate the value for A11, A10, A9 bits in MR0 (write recovery) */
static u32 ddr3_write_recovery(u32 clk)
{
	u32 twr_ns = 15; /* DDR3 spec says that it is 15ns for all speed bins */
	u32 twr_ck = (twr_ns * clk + 999) / 1000;
	if (twr_ck < 5)
		return 1;
	else if (twr_ck <= 8)
		return twr_ck - 4;
	else if (twr_ck <= 10)
		return 5;
	else
		return 6;
}

/*
 * If the dram->ppwrsctl (SDR_DPCR) register has the lowest bit set to 1, this
 * means that DRAM is currently in self-refresh mode and retaining the old
 * data. Since we have no idea what to do in this situation yet, just set this
 * register to 0 and initialize DRAM in the same way as on any normal reboot
 * (discarding whatever was stored there).
 *
 * Note: on sun7i hardware, the highest 16 bits need to be set to 0x1651 magic
 * value for this write operation to have any effect. On sun5i hadware this
 * magic value is not necessary. And on sun4i hardware the writes to this
 * register seem to have no effect at all.
 */
static void mctl_disable_power_save(void)
{
	struct sunxi_dram_reg *dram = (struct sunxi_dram_reg *)SUNXI_DRAMC_BASE;
	writel(0x16510000, &dram->ppwrsctl);
}

/*
 * After the DRAM is powered up or reset, the DDR3 spec requires to wait at
 * least 500 us before driving the CKE pin (Clock Enable) high. The dram->idct
 * (SDR_IDCR) register appears to configure this delay, which gets applied
 * right at the time when the DRAM initialization is activated in the
 * 'mctl_ddr3_initialize' function.
 */
static void mctl_set_cke_delay(void)
{
	struct sunxi_dram_reg *dram = (struct sunxi_dram_reg *)SUNXI_DRAMC_BASE;

	/* The CKE delay is represented in dram clock cycles, multiplied by N
	 * (where N=2 for sun4i/sun5i and N=3 for sun7i). We are being lazy
	 * to do proper calculations and just set it to the maximum possible
	 * value 0x1ffff. This is enough to provide the needed 500 us delay
	 * at the DRAM clock freqencies up to ~524MHz on sun4i/sun5i hardware.
	 * The sun7i hardware has even more headroom due to a larger multiplier.
	 */
	setbits_le32(&dram->idcr, 0x1ffff);
}

/*
 * This triggers the DRAM initialization. It performs sending the mode registers
 * to the DRAM among other things. Very likely the ZQCL command is also getting
 * executed (to do the initial impedance calibration on the DRAM side of the
 * wire). The memory controller and the PHY must be already configured before
 * calling this function.
 */
static void mctl_ddr3_initialize(void)
{
	struct sunxi_dram_reg *dram = (struct sunxi_dram_reg *)SUNXI_DRAMC_BASE;
	setbits_le32(&dram->ccr, DRAM_CCR_INIT);
	await_completion(&dram->ccr, DRAM_CCR_INIT);
}

/*
 * Perform impedance calibration on the DRAM controller side of the wire.
 */
static void mctl_set_impedance(u32 zq, u32 odt_en)
{
	struct sunxi_dram_reg *dram = (struct sunxi_dram_reg *)SUNXI_DRAMC_BASE;
	u32 reg_val;
	u32 zprog = zq & 0xFF, zdata = (zq >> 8) & 0xFFFFF;

	if (!IS_SUN7I()) {
		/* wait for the default impedance configuration to settle */
		await_bits_set(&dram->zqsr, DRAM_ZQSR_ZDONE);
	}

	if (!odt_en)
		return;

	if (IS_SUN7I()) {
		/* some weird magic, but sun7i fails to boot without it */
		writel((1 << 24) | (1 << 1), &dram->zqcr1);
	}

	/* needed at least for sun5i, because it does not self clear there */
	clrbits_le32(&dram->zqcr0, DRAM_ZQCR0_ZCAL);

	if (zdata) {
		/* set the user supplied impedance data */
		reg_val = DRAM_ZQCR0_ZDEN | zdata;
		writel(reg_val, &dram->zqcr0);
		/* no need to wait, this takes effect immediately */
	} else {
		/* do the calibration using the external resistor */
		reg_val = DRAM_ZQCR0_ZCAL | DRAM_ZQCR0_IMP_DIV(zprog);
		writel(reg_val, &dram->zqcr0);
		/* wait for the new impedance configuration to settle */
		await_bits_set(&dram->zqsr, DRAM_ZQSR_ZDONE);
	}

	/* needed at least for sun5i, because it does not self clear there */
	clrbits_le32(&dram->zqcr0, DRAM_ZQCR0_ZCAL);

	/* set I/O configure register */
	writel(DRAM_IOCR_ODT_EN(odt_en), &dram->iocr);
}

static unsigned long dramc_init_helper(struct dram_para *para)
{
	struct sunxi_dram_reg *dram = (struct sunxi_dram_reg *)SUNXI_DRAMC_BASE;
	u32 reg_val;
	u32 density;
	int ret_val;

	/*
	 * only single rank DDR3 is supported by this code even though the
	 * hardware can theoretically support DDR2 and up to two ranks
	 */
	if (para->type != DRAM_MEMORY_TYPE_DDR3 || para->rank_num != 1)
		return 0;

	/* setup DRAM relative clock */
	mctl_setup_dram_clock(para->clock, para->mbus_clock);

	/* Disable any pad power save control */
	mctl_disable_power_save();

	/* reset external DRAM */
	mctl_set_drive();

	/* dram clock off */
	dramc_clock_output_en(0);

	if (IS_SUN4I()) {
		/* select dram controller 1 */
		writel(DRAM_CSEL_MAGIC, &dram->csel);
	}

	mctl_itm_disable();
	mctl_enable_dll0(para->tpr3);

	/* configure external DRAM */
	reg_val = DRAM_DCR_TYPE_DDR3;
	reg_val |= DRAM_DCR_IO_WIDTH(para->io_width >> 3);

	if (para->density == 256)
		density = DRAM_DCR_CHIP_DENSITY_256M;
	else if (para->density == 512)
		density = DRAM_DCR_CHIP_DENSITY_512M;
	else if (para->density == 1024)
		density = DRAM_DCR_CHIP_DENSITY_1024M;
	else if (para->density == 2048)
		density = DRAM_DCR_CHIP_DENSITY_2048M;
	else if (para->density == 4096)
		density = DRAM_DCR_CHIP_DENSITY_4096M;
	else if (para->density == 8192)
		density = DRAM_DCR_CHIP_DENSITY_8192M;
	else
		density = DRAM_DCR_CHIP_DENSITY_256M;

	reg_val |= DRAM_DCR_CHIP_DENSITY(density);
	reg_val |= DRAM_DCR_BUS_WIDTH((para->bus_width >> 3) - 1);
	reg_val |= DRAM_DCR_RANK_SEL(para->rank_num - 1);
	reg_val |= DRAM_DCR_CMD_RANK_ALL;
	reg_val |= DRAM_DCR_MODE(DRAM_DCR_MODE_INTERLEAVE);
	writel(reg_val, &dram->dcr);

	dramc_clock_output_en(1);

	mctl_set_impedance(para->zq, para->odt_en);

	mctl_set_cke_delay();

	mctl_ddr3_reset();

	udelay(1);

	await_completion(&dram->ccr, DRAM_CCR_INIT);

	mctl_enable_dllx(para->tpr3);

	/* set refresh period */
	dramc_set_autorefresh_cycle(para->clock, density);

	/* set timing parameters */
	writel(para->tpr0, &dram->tpr0);
	writel(para->tpr1, &dram->tpr1);
	writel(para->tpr2, &dram->tpr2);

	reg_val = DRAM_MR_BURST_LENGTH(0x0);
	if (IS_SUN5I() || IS_SUN7I())
		reg_val |= DRAM_MR_POWER_DOWN;
	reg_val |= DRAM_MR_CAS_LAT(para->cas - 4);
	reg_val |= DRAM_MR_WRITE_RECOVERY(ddr3_write_recovery(para->clock));
	writel(reg_val, &dram->mr);

	writel(para->emr1, &dram->emr);
	writel(para->emr2, &dram->emr2);
	writel(para->emr3, &dram->emr3);

	/* disable drift compensation and set passive DQS window mode */
	clrsetbits_le32(&dram->ccr, DRAM_CCR_DQS_DRIFT_COMP, DRAM_CCR_DQS_GATE);

	if (IS_SUN7I()) {
		/* Command rate timing mode 2T & 1T */
		if (para->tpr4 & 0x1)
			setbits_le32(&dram->ccr, DRAM_CCR_COMMAND_RATE_1T);
	}

	/* initialize external DRAM */
	mctl_ddr3_initialize();

	/* scan read pipe value */
	mctl_itm_enable();

	/* Hardware DQS gate training */
	ret_val = dramc_scan_readpipe();

	if (ret_val < 0)
		return 0;

	/* allow to override the DQS training results with a custom delay */
	if (para->dqs_gating_delay)
		mctl_set_dqs_gating_delay(0, para->dqs_gating_delay);

	/* set the DQS gating window type */
	if (para->active_windowing)
		clrbits_le32(&dram->ccr, DRAM_CCR_DQS_GATE);
	else
		setbits_le32(&dram->ccr, DRAM_CCR_DQS_GATE);

	mctl_itm_reset();

	/* configure all host port */
	mctl_configure_hostport();

	return get_ram_size((long *)PHYS_SDRAM_0, PHYS_SDRAM_0_SIZE);
}

unsigned long dramc_init(struct dram_para *para)
{
	unsigned long dram_size, actual_density;

	/* If the dram configuration is not provided, use a default */
	if (!para)
		return 0;

	/* if everything is known, then autodetection is not necessary */
	if (para->io_width && para->bus_width && para->density)
		return dramc_init_helper(para);

	/* try to autodetect the DRAM bus width and density */
	para->io_width  = 16;
	para->bus_width = 32;

	if (IS_SUN4I() || IS_SUN5I()) {
		/* only A0-A14 address lines on A10/A13, limiting max density to 4096 */
		para->density = 4096;
	} else {
		/* all A0-A15 address lines on A20, which allow density 8192 */
		para->density = 8192;
	}

	dram_size = dramc_init_helper(para);
	if (!dram_size) {
		/* if 32-bit bus width failed, try 16-bit bus width instead */
		para->bus_width = 16;
		dram_size = dramc_init_helper(para);
		if (!dram_size) {
			/* if 16-bit bus width also failed, then bail out */
			return dram_size;
		}
	}

	/* check if we need to adjust the density */
	actual_density = (dram_size >> 17) * para->io_width / para->bus_width;

	if (actual_density != para->density) {
		/* update the density and re-initialize DRAM again */
		para->density = actual_density;
		dram_size = dramc_init_helper(para);
	}

	return dram_size;
}
