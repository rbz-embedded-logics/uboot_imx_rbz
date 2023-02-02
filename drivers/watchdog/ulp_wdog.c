// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2016 Freescale Semiconductor, Inc.
 */

#include <common.h>
#include <cpu_func.h>
#include <asm/io.h>
#include <asm/arch/imx-regs.h>

/*
 * MX7ULP WDOG Register Map
 */
struct wdog_regs {
	u32 cs;
	u32 cnt;
	u32 toval;
	u32 win;
};

#ifndef CONFIG_WATCHDOG_TIMEOUT_MSECS
#define CONFIG_WATCHDOG_TIMEOUT_MSECS 0x1500
#endif

#define REFRESH_WORD0 0xA602 /* 1st refresh word */
#define REFRESH_WORD1 0xB480 /* 2nd refresh word */

#define UNLOCK_WORD0 0xC520 /* 1st unlock word */
#define UNLOCK_WORD1 0xD928 /* 2nd unlock word */

#define UNLOCK_WORD 0xD928C520 /* unlock word */
#define REFRESH_WORD 0xB480A602 /* refresh word */

#define WDGCS_WDGE                      BIT(7)
#define WDGCS_WDGUPDATE                 BIT(5)

#define WDGCS_RCS                       BIT(10)
#define WDGCS_ULK                       BIT(11)
#define WDOG_CS_PRES                    BIT(12)
#define WDGCS_CMD32EN                   BIT(13)
#define WDGCS_FLG                       BIT(14)

#define WDG_BUS_CLK                      (0x0)
#define WDG_LPO_CLK                      (0x1)
#define WDG_32KHZ_CLK                    (0x2)
#define WDG_EXT_CLK                      (0x3)

void hw_watchdog_set_timeout(u16 val)
{
	/* setting timeout value */
	struct wdog_regs *wdog = (struct wdog_regs *)WDOG_BASE_ADDR;

	writel(val, &wdog->toval);
}

void hw_watchdog_reset(void)
{
	struct wdog_regs *wdog = (struct wdog_regs *)WDOG_BASE_ADDR;

	if (readl(&wdog->cs) & WDGCS_CMD32EN) {
		writel(REFRESH_WORD, &wdog->cnt);
	} else {
		dmb();
		__raw_writel(REFRESH_WORD0, &wdog->cnt);
		__raw_writel(REFRESH_WORD1, &wdog->cnt);
		dmb();
	}
}

void hw_watchdog_init(void)
{
	struct wdog_regs *wdog = (struct wdog_regs *)WDOG_BASE_ADDR;
	u32 cmd32 = 0;

	if (readl(&wdog->cs) & WDGCS_CMD32EN) {
		writel(UNLOCK_WORD, &wdog->cnt);
		cmd32 = WDGCS_CMD32EN;
	} else {
		dmb();
		__raw_writel(UNLOCK_WORD0, &wdog->cnt);
		__raw_writel(UNLOCK_WORD1, &wdog->cnt);
		dmb();
	}

	/* Wait WDOG Unlock */
	while (!(readl(&wdog->cs) & WDGCS_ULK))
		;

	hw_watchdog_set_timeout(CONFIG_WATCHDOG_TIMEOUT_MSECS);
	writel(0, &wdog->win);

	/* setting 1-kHz clock source, enable counter running, and clear interrupt */
#if defined(CONFIG_ARCH_IMX9)
	writel((cmd32 | WDGCS_WDGE | WDGCS_WDGUPDATE |(WDG_LPO_CLK << 8) | WDGCS_FLG | WDOG_CS_PRES), &wdog->cs);
#else
	writel((cmd32 | WDGCS_WDGE | WDGCS_WDGUPDATE |(WDG_LPO_CLK << 8) | WDGCS_FLG), &wdog->cs);
#endif

	/* Wait WDOG reconfiguration */
	while (!(readl(&wdog->cs) & WDGCS_RCS))
		;

	hw_watchdog_reset();
}

void reset_cpu(void)
{
	struct wdog_regs *wdog = (struct wdog_regs *)WDOG_BASE_ADDR;
	u32 cmd32 = 0;

	if (readl(&wdog->cs) & WDGCS_CMD32EN) {
		writel(UNLOCK_WORD, &wdog->cnt);
		cmd32 = WDGCS_CMD32EN;
	} else {
		dmb();
		__raw_writel(UNLOCK_WORD0, &wdog->cnt);
		__raw_writel(UNLOCK_WORD1, &wdog->cnt);
		dmb();
	}

	/* Wait WDOG Unlock */
	while (!(readl(&wdog->cs) & WDGCS_ULK))
		;

	hw_watchdog_set_timeout(5); /* 5ms timeout for general; 40ms timeout for imx93 */

	writel(0, &wdog->win);

	/* enable counter running */
#if defined(CONFIG_ARCH_IMX9)
	writel((cmd32 | WDGCS_WDGE | (WDG_LPO_CLK << 8) | WDOG_CS_PRES), &wdog->cs);
#else	
	writel((cmd32| WDGCS_WDGE | (WDG_LPO_CLK << 8)), &wdog->cs);
#endif

	/* Wait WDOG reconfiguration */
	while (!(readl(&wdog->cs) & WDGCS_RCS))
		;

	hw_watchdog_reset();

	while (1);
}
