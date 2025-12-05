/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2025 Embedded Artists AB
 */

#ifndef __RZG3E_EA_SOM_H
#define __RZG3E_EA_SOM_H

#include <asm/arch/renesas.h>

#define CONFIG_REMAKE_ELF

#ifdef CONFIG_SPL
#define CONFIG_SPL_TARGET	"spl/u-boot-spl.scif"
#endif

/* boot option */

#define CONFIG_CMDLINE_TAG
#define CONFIG_SETUP_MEMORY_TAGS
#define CONFIG_INITRD_TAG

/* Generic Interrupt Controller Definitions */
/* RZ/G3E use GIC-v3 */
#define CONFIG_GICV3
#define GICD_BASE	0x14900000
#define GICR_BASE	0x14940000

/* console */
#define CONFIG_SYS_CBSIZE		2048
#define CONFIG_SYS_BARGSIZE		CONFIG_SYS_CBSIZE
#define CONFIG_SYS_BAUDRATE_TABLE	{ 115200, 38400 }

/* PHY needs a longer autoneg timeout */
#define PHY_ANEG_TIMEOUT		20000

/* MEMORY */
#define CONFIG_SYS_INIT_SP_ADDR		CONFIG_SYS_TEXT_BASE

#define	DRAM_RSV_SIZE			0x08000000
#define	CFG_MAX_MEM_MAPPED		(0x80000000u - DRAM_RSV_SIZE)

/* SDHI clock freq */
#define CONFIG_SH_SDHI_FREQ		133000000

/* The HF/QSPI layout permits up to 1 MiB large bootloader blob */
#define CONFIG_BOARD_SIZE_LIMIT		1048576

/* ENV setting */
#define CFG_EXTRA_ENV_SETTINGS \
	"usb_pgood_delay=2000\0" \
	"bootm_size=0x10000000 \0" \
	"prodsdbootargs=setenv bootargs rw rootwait earlycon root=/dev/mmcblk1p2 \0" \
	"prodemmcbootargs=setenv bootargs rw rootwait earlycon root=/dev/mmcblk0p2 \0" \
	"bootimage=booti 0x48080000 - 0x48000000 \0" \
	"emmcload=ext4load mmc 0:2 0x48080000 boot/Image;ext4load mmc 0:2 0x48000000 ${fdt_file};run prodemmcbootargs \0" \
	"sd1load=ext4load mmc 1:2 0x48080000 boot/Image;ext4load mmc 1:2 0x48000000 ${fdt_file};run prodsdbootargs \0" \
	"fdt_file=boot/rzg3e-ea-som-hdmi.dtb \0" \
	"enable_onoff_button=i2c dev 8;i2c mw 0x22 0x4d 0x08 0x01 \0" \
	"bootcmd_check=if mmc dev 1; then run sd1load; else run emmcload; fi \0"
#define CONFIG_BOOTCOMMAND      "run enable_onoff_button;run bootcmd_check;run bootimage"

/* The enable_onoff_button command above configure a 100kOhm PullUp on pin IO1_3
 * of the onboard PCAL6524 to make the ONOFF button on the SOM Carrier Board work. */

/* For board */
/* Ethernet RAVB */
#define CONFIG_BITBANGMII_MULTI

#endif /* __RZG3E_EA_SOM_H */
