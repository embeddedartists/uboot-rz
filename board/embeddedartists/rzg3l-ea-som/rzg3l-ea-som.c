// SPDX-License-Identifier: (GPL-2.0-only OR BSD-2-Clause)
/*
 *  * Copyright (C) 2023 Renesas Electronics Corp.
 */

#include <common.h>
#include <cpu_func.h>
#include <hang.h>
#include <image.h>
#include <init.h>
#include <malloc.h>
#include <netdev.h>
#include <dm.h>
#include <dm/platform_data/serial_sh.h>
#include <asm/processor.h>
#include <asm/mach-types.h>
#include <asm/io.h>
#include <linux/bitops.h>
#include <linux/errno.h>
#include <asm/arch/sys_proto.h>
#include <asm/gpio.h>
#include <asm/arch/gpio.h>
#include <asm/arch/renesas.h>
#include <asm/arch/rcar-mstp.h>
#include <asm/arch/sh_sdhi.h>
#include <mmc.h>
#include <efi_loader.h>
#include <linux/delay.h>
#include <i2c.h>

DECLARE_GLOBAL_DATA_PTR;

#define PFC_BASE			0x11030000

#define ETH0_POC			(PFC_BASE + 0x3010) /* Ether ch0 IO Voltage Mode Control Register */
#define ETH1_POC			(PFC_BASE + 0x3014) /* Ether ch1 IO Voltage Mode Control Register */
#define ETH_PVDD_3300			0x00
#define ETH_PVDD_1800			0x01
#define ETH_PVDD_2500			0x02
#define ETH_MODE			(PFC_BASE + 0x3018) /* Ether MII/RGMII Mode Control Register */

/* CPG */
#define CPG_BASE			0x11010000
#define CPG_CLKON_BASE			(CPG_BASE + 0x500)
#define CPG_CLKMON_BASE			(CPG_BASE + 0x680)
#define CPG_RESET_BASE			(CPG_BASE + 0x800)
#define CPG_CLKON_ETH			(CPG_CLKON_BASE + 0x7C)
#define CPG_CLKMON_ETH			(CPG_CLKMON_BASE + 0x7C)
#define CPG_RESET_ETH			(CPG_RESET_BASE + 0x7C)
#define CPG_RESET_I2C			(CPG_RESET_BASE + 0x80)
#define CPG_SDHI_DDIV			(CPG_BASE + 0x218)
#define CPG_SDHI_DSEL			(CPG_BASE + 0x244)
#define CPG_SPI_DDIV                   (CPG_BASE + 0x220)
#define CPG_CLKDIV_STATUS		(CPG_BASE + 0x280)
#define CPG_CLKSEL_STATUS		(CPG_BASE + 0x284)
#define CPG_RST_USB			(CPG_BASE + 0x878)
#define CPG_RSTMON_USB			(CPG_BASE + 0x9F8)
#define CPG_CLKON_USB			(CPG_BASE + 0x578)
#define CPG_CLKMON_USB			(CPG_BASE + 0x6F8)

//g3l
#define CPG_ETH_SSEL			(CPG_BASE + 0x410)
#define CPG_ETH_SDIV			(CPG_BASE + 0x434)

/* PFC */
#define	PFC_P25				(PFC_BASE + 0x25)
#define	PFC_PM25			(PFC_BASE + 0x014A)
#define	PFC_PMC25			(PFC_BASE + 0x0225)

#define	PFC_P30				(PFC_BASE + 0x30)
#define	PFC_PM30			(PFC_BASE + 0x0160)
#define	PFC_PMC30			(PFC_BASE + 0x0230)
#define	PFC_PFC30			(PFC_BASE + 0x04c0)

#define	PFC_P31				(PFC_BASE + 0x31)
#define	PFC_PM31			(PFC_BASE + 0x0162)
#define	PFC_PMC31			(PFC_BASE + 0x0231)

#define	PFC_P33				(PFC_BASE + 0x33)
#define	PFC_PM33			(PFC_BASE + 0x0166)
#define	PFC_PMC33			(PFC_BASE + 0x0233)

#define	PFC_P35				(PFC_BASE + 0x35)
#define	PFC_PM35			(PFC_BASE + 0x016A)
#define	PFC_PMC35			(PFC_BASE + 0x0235)

#define	PFC_IOLH_30_L			(PFC_BASE + 0x1180)
#define	PFC_IOLH_34_L			(PFC_BASE + 0x11A0)
#define	PFC_IEN_30_L		(PFC_BASE + 0x1980)
#define	PFC_IEN_30_H		(PFC_BASE + 0x1984)
#define	PFC_IEN_34			(PFC_BASE + 0x19A0)
#define	PFC_IEN_23			(PFC_BASE + 0x1918)
#define	PFC_IEN_24			(PFC_BASE + 0x1920)

//g3l Eth0
#define PFC_PMC2A			(PFC_BASE + 0x22A)
#define PFC_PMC2B			(PFC_BASE + 0x22B)
#define PFC_PMC2C			(PFC_BASE + 0x22C)
#define PFC_PFC2A			(PFC_BASE + 0x4A8)
#define PFC_PFC2B			(PFC_BASE + 0x4AC)
#define PFC_PFC2C			(PFC_BASE + 0x4B0)

//g3l Eth1
#define PFC_PMC2D			(PFC_BASE + 0x22D)
#define PFC_PMC2E			(PFC_BASE + 0x22E)
#define PFC_PMC2F			(PFC_BASE + 0x22F)
#define PFC_PFC2D			(PFC_BASE + 0x4B4)
#define PFC_PFC2E			(PFC_BASE + 0x4B8)
#define PFC_PFC2F			(PFC_BASE + 0x4BC)

//i2c
#define	PFC_PFC35			(PFC_BASE + 0x4D4)

//i2c3 on P20/P21
#define	PFC_PMC22			(PFC_BASE + 0x222)
#define	PFC_PFC22			(PFC_BASE + 0x488)

/* SYSC clone-channel select for I2C ch3 alternate pin group */
#define SYSC_BASE			0x11020000
#define SYS_IPCONT_SEL_CLONECH		(SYSC_BASE + 0x0E2C)

#define PFC_PWPR			(PFC_BASE + 0x3000)
#define PWPR_B0WI			BIT(7)	 /* Bit Write Disable */
#define PWPR_PFCWE			BIT(6)	/* PFC Register Write Enable */

#define USBPHY_BASE		(0x11e00000)
#define USB0_BASE		(0x11e10000)
#define USB1_BASE		(0x11e30000)
#define USBF_BASE		(0x11e20000)
#define USBPHY_RESET		(USBPHY_BASE + 0x000u)
#define COMMCTRL		0x800
#define HcRhDescriptorA		0x048
#define LPSTS			0x102
#define	AHB_BUS_CTR		0x208

void s_init(void)
{
	/* Disable GPIO Write Protect */
	(*(volatile u32 *)PFC_PWPR) &= ~(0x1u << 7);	/* PWPR.BOWI = 0 */
	(*(volatile u32 *)PFC_PWPR) |= (0x1u << 6);	/* PWPR.PFCWE = 1 */

	/* PG-PG5 function 1	*/
	*(volatile u32 *)(PFC_PFC30) = (*(volatile u32 *)(PFC_PFC30) & 0xFF000000) | 0x111111;
	*(volatile u8 *)(PFC_PMC30) |= 0x3F;	/* PG0-PG5 function	*/

	/* PG0 Drive Strength for SD1_CLK */
	*(volatile u32 *)(PFC_IOLH_30_L) = (*(volatile u32 *)(PFC_IOLH_30_L) & 0xFFFFFFFC) | 0x0; /* SD1_CLK  3.3V 4mA , 1.8V 7mA */

	/* PJ1 = 0 QSD1_IOVS=0	*/

	*(volatile u8 *)(PFC_PMC33) &= ~(0x02);
	*(volatile u16 *)(PFC_PM33) = (*(volatile u16 *)(PFC_PM33) & 0xFF) | 0x08; /*bit[5:4]='B10	*/
	*(volatile u8 *)(PFC_P33) &= ~(0x02);
	/* PJ2 = 1 QSD1_PWEN=1	*/
	*(volatile u8 *)(PFC_PMC33) &= ~(0x04);
	*(volatile u16 *)(PFC_PM33) = (*(volatile u16 *)(PFC_PM33) & 0xFF) | 0x20; /*bit[5:4]='B10	*/
	*(volatile u8 *)(PFC_P33) |= 0x04;

	/* Input Enable Control PG0-PG5 Enable	*/
	*(volatile u32 *)(PFC_IEN_30_L) = 0x01010100;
	*(volatile u32 *)(PFC_IEN_30_H) = 0x00000101;

	//g3l
	/* Pinmux for ETH0 */
	*(volatile u32 *)(PFC_PFC2A) = 0x00001111;
	*(volatile u32 *)(PFC_PFC2B) = 0x11111111;
	*(volatile u32 *)(PFC_PFC2C) = 0x00000111;
	*(volatile u8 *)(PFC_PMC2A) = 0xF;
	*(volatile u8 *)(PFC_PMC2B) = 0xFF;
	*(volatile u8 *)(PFC_PMC2C) = 0x7;

	/* Pinmux for ETH1 */
	*(volatile u32 *)(PFC_PFC2D) = 0x00001111;
	*(volatile u32 *)(PFC_PFC2E) = 0x11111111;
	*(volatile u32 *)(PFC_PFC2F) = 0x00000111;
	*(volatile u8 *)(PFC_PMC2D) = 0xF;
	*(volatile u8 *)(PFC_PMC2E) = 0xFF;
	*(volatile u8 *)(PFC_PMC2F) = 0x7;

	/* Pinmux for I2C0	*/
	*(volatile u32 *)(PFC_PFC35) |= 0x4400;
	*(volatile u8 *)(PFC_PMC35)  |= 0x0C;

	/* Select I2C ch3 alternate pin group P20/P21 (SYS_IPCONT_SEL_CLONECH bit[1]) */
	*(volatile u32 *)(SYS_IPCONT_SEL_CLONECH) |= 0x2;

	/* Pinmux for I2C1 on PA6/PA7 (function 4) */
	*(volatile u32 *)(PFC_PFC2A) |= 0x44000000;
	*(volatile u8  *)(PFC_PMC2A) |= 0xC0;

	/* Pinmux for I2C2 on PA4/PA5 (function 4) */
	*(volatile u32 *)(PFC_PFC2A) |= 0x00440000;
	*(volatile u8  *)(PFC_PMC2A) |= 0x30;

	/* Pinmux for I2C3 on P20/P21 (function 4) */
	*(volatile u32 *)(PFC_PFC22) |= 0x00000044;
	*(volatile u8  *)(PFC_PMC22) |= 0x03;

	/* can go in board_eht_init() once enabled */
	*(volatile u32 *)(ETH0_POC) = (*(volatile u32 *)(ETH0_POC) & 0xFFFFFFFC) | ETH_PVDD_1800;
	*(volatile u32 *)(ETH1_POC) = (*(volatile u32 *)(ETH1_POC) & 0xFFFFFFFC) | ETH_PVDD_1800;
	/* Enable RGMII for both ETH{0,1} */
	*(volatile u32 *)(ETH_MODE) = (*(volatile u32 *)(ETH_MODE) & 0xFFFFFFF0);

	*(volatile u32 *)(PFC_PWPR) = 0;
	*(volatile u32 *)(PFC_PWPR) = PWPR_B0WI;

	//g3l
	/* ETH CLK Setup for Eth0 and Eth1 */
	*(volatile u32 *)(CPG_CLKON_ETH) = 0x3FFF3FFF;
	while(*(volatile u32 *)(CPG_CLKMON_ETH) != 0x00003FFF);
	*(volatile u32 *)(CPG_ETH_SSEL) = 0xFFFF0202;  //source clock selection for ETH0 and ETH1
	*(volatile u32 *)(CPG_RESET_ETH) = 0x00030003;

	/*
	 * Setting SD CLKs.
	 * Currently, we use IMCLKs with output CLK rate 133 MHz, HSCLK will be considered to support later.
	 */
	*(volatile u32 *)(CPG_SDHI_DDIV) = 0x01110000;
	*(volatile u32 *)(CPG_SDHI_DSEL) = 0x01110222;
	while ((*(volatile u32 *)(CPG_CLKDIV_STATUS) != 0) || (*(volatile u32 *)(CPG_CLKSEL_STATUS) != 0))
		;
        /* I2C CLK */
        *(volatile u32 *)(CPG_RESET_I2C) = 0xF000F;

       /* Setting xSPI CLK 133 MHz */
       *(volatile u32 *)(CPG_SPI_DDIV) = 0x00010001;
       while (*(volatile u32 *)(CPG_CLKDIV_STATUS) != 0);
}

static void board_usb_init(void)
{
}

int board_early_init_f(void)
{
	return 0;
}

int board_init(void)
{
	/* Address of boot parameters */
	gd->bd->bi_boot_params = CONFIG_TEXT_BASE + 0x50000;
	board_usb_init();

	return 0;
}

static int board_gpio_configure_pin(const char* pin_name, const char* request_name, int value, struct gpio_desc* desc)
{
	int ret;
	ret = dm_gpio_lookup_name(pin_name, desc);
	if (ret) {
		printf("Failed to lookup %s as %s\n", pin_name, request_name);
		return ret;
	}
	ret = dm_gpio_request(desc, request_name);
	if (ret) {
		printf("Failed to request %s as %s\n", pin_name, request_name);
		return ret;
	}
	dm_gpio_set_dir_flags(desc, GPIOD_IS_OUT);
	dm_gpio_set_value(desc, value);
	return ret;
}

/* Control power and reset for onboard pcie
 * M1_PCIE_RST_N_1V8 - gpio PL6
 * M1_PWR_EN_1V8     - gpio PL7
 *
 * Control power and reset for m.2 m-key
 * M2BM_PWR_EN    - i2c gpio (0x21) pin io2_1   shared with b-key
 * M2M_PERST      - i2c gpio (0x21) pin io1_7
 *
 * Control power and reset for m.2 e-key
 * M2E_PWR_EN     - i2c gpio (0x21) pin io1_2
 * M2E_PERST      - i2c gpio (0x21) pin io0_7
 * M2E_WL_REG_ON  - i2c gpio (0x21) pin io0_0
 * M2E_BT_REG_ON  - i2c gpio (0x21) pin io0_3
 *
 * Control power and reset for m.2 b-key
 * M2BM_PWR_EN    - i2c gpio (0x21) pin io2_1   shared with m-key
 * M2B_PERST      - i2c gpio (0x21) pin io1_6
 * M2B_PWR_OFF    - i2c gpio (0x21) pin io1_3
 * M2B_WDISABLE   - i2c gpio (0x21) pin io1_4
 */
static void board_init_pcie_power_and_reset(void)
{
	int ret;
	/*
	struct gpio_desc desc_M1_PCIE_RST_N_1V8;
	struct gpio_desc desc_M1_PWR_EN_1V8;
	*/
	struct gpio_desc desc_M2BM_PWR_EN;
	struct gpio_desc desc_M2M_PERST;
	struct gpio_desc desc_M2E_PWR_EN;
	struct gpio_desc desc_M2E_PERST;
	struct gpio_desc desc_M2E_WL_REG_ON;
	struct gpio_desc desc_M2E_BT_REG_ON;
	struct gpio_desc desc_M2B_PERST;
	struct gpio_desc desc_M2B_PWR_OFF;
	struct gpio_desc desc_M2B_WDISABLE;

	do {
		/*
		if ((ret = board_gpio_configure_pin("gpio-216", "M1_PCIE_RST_N_1V8", 0, &desc_M1_PCIE_RST_N_1V8)) != 0) break;
		if ((ret = board_gpio_configure_pin("gpio-217", "M1_PWR_EN_1V8", 0, &desc_M1_PWR_EN_1V8)) != 0) break;
		*/
		if ((ret = board_gpio_configure_pin("gpio@21_17", "M2BM_PWR_EN", 0, &desc_M2BM_PWR_EN)) != 0) break;
		if ((ret = board_gpio_configure_pin("gpio@21_15", "M2M_PERST", 0, &desc_M2M_PERST)) != 0) break;
		if ((ret = board_gpio_configure_pin("gpio@21_10", "M2E_PWR_EN", 0, &desc_M2E_PWR_EN)) != 0) break;
		if ((ret = board_gpio_configure_pin("gpio@21_7", "M2E_PERST", 0, &desc_M2E_PERST)) != 0) break;
		if ((ret = board_gpio_configure_pin("gpio@21_0", "M2E_WL_REG_ON", 0, &desc_M2E_WL_REG_ON)) != 0) break;
		if ((ret = board_gpio_configure_pin("gpio@21_3", "M2E_BT_REG_ON", 0, &desc_M2E_BT_REG_ON)) != 0) break;
		if ((ret = board_gpio_configure_pin("gpio@21_14", "M2B_PERST", 0, &desc_M2B_PERST)) != 0) break;
		if ((ret = board_gpio_configure_pin("gpio@21_11", "M2B_PWR_OFF", 0, &desc_M2B_PWR_OFF)) != 0) break;
		if ((ret = board_gpio_configure_pin("gpio@21_12", "WDISABLE", 0, &desc_M2B_WDISABLE)) != 0) break;
		mdelay(500);
		//dm_gpio_set_value(&desc_M1_PWR_EN_1V8, 1);
		dm_gpio_set_value(&desc_M2BM_PWR_EN, 1);
		dm_gpio_set_value(&desc_M2E_PWR_EN, 1);
		mdelay(100);
		dm_gpio_set_value(&desc_M2E_WL_REG_ON, 1);
		dm_gpio_set_value(&desc_M2E_BT_REG_ON, 1);
		dm_gpio_set_value(&desc_M2B_PWR_OFF, 1);
		dm_gpio_set_value(&desc_M2B_WDISABLE, 1);
		mdelay(100);
		//dm_gpio_set_value(&desc_M1_PCIE_RST_N_1V8, 1);
		dm_gpio_set_value(&desc_M2M_PERST, 1);
		dm_gpio_set_value(&desc_M2E_PERST, 1);
		dm_gpio_set_value(&desc_M2B_PERST, 1);
		mdelay(30);
		//dm_gpio_set_value(&desc_M1_PCIE_RST_N_1V8, 0);
		dm_gpio_set_value(&desc_M2M_PERST, 0);
		dm_gpio_set_value(&desc_M2E_PERST, 0);
		dm_gpio_set_value(&desc_M2B_PERST, 0);
		mdelay(30);
		//dm_gpio_set_value(&desc_M1_PCIE_RST_N_1V8, 1);
		dm_gpio_set_value(&desc_M2M_PERST, 1);
		dm_gpio_set_value(&desc_M2E_PERST, 1);
		dm_gpio_set_value(&desc_M2B_PERST, 1);
		printf("Initialized power and reset for PCIe\n");
	} while (0);
	if (ret != 0) {
		printf("Failed to initialize power and reset for PCIe\n");
	}
}

int board_late_init(void)
{
	struct udevice *dev;
	const u8 pmic_i2c_bus = 0;
	int ret;

	printf("board_late_init\n");

	ret = i2c_get_chip_for_busnum(pmic_i2c_bus, 0x12, 1, &dev);
	if (!ret)
	{
		/* Config MPIO1 func to Output from I2C output */
		dm_i2c_reg_clrset(dev, 0x8B, 0x7, 0x7);

		/* Assert and deassert MPIO1 line to reset PHY */
		dm_i2c_reg_clrset(dev, 0x7f, 0x2, 0x0);
		udelay(2);
		dm_i2c_reg_clrset(dev, 0x7f, 0x0, 0x2);

		/* Config MPIO1 func to Reset output */
		dm_i2c_reg_clrset(dev, 0x8B, 0x7, 0x5);
		dm_i2c_reg_clrset(dev, 0x7f, 0x2, 0x0);
	}

	/* The first access of an I2C-gpio fail so we delay */
	mdelay(500);
	board_init_pcie_power_and_reset();

	return ret;
}

void reset_cpu(void)
{
}
