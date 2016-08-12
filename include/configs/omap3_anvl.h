/*
 * Configuration settings for the Network Improv Anvl board.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc.
 */

#ifndef __CONFIG_H
#define __CONFIG_H

#define CONFIG_NR_DRAM_BANKS		2

#include <configs/ti_omap3_common.h>

#define CONFIG_DISPLAY_BOARDINFO	1
#define CONFIG_DISPLAY_CPUINFO		1
#define CONFIG_MISC_INIT_R
#define CONFIG_OMAP3_SPI
#define CONFIG_SPL_OMAP3_ID_NAND
#define CONFIG_SUPPORT_RAW_INITRD
#define CONFIG_SYS_BOOT_RAMDISK_HIGH
#define CONFIG_SYS_CACHELINE_SIZE	64
#define CONFIG_SYS_PTV			2

/* I2C */
#define CONFIG_I2C_MULTI_BUS
#define CONFIG_SYS_I2C_NOPROBES		{{0x0, 0x0}}

/* GPIO */
#define CONFIG_OMAP3_GPIO_2		/* GPIO32..63 is in GPIO Bank 2 */
#define CONFIG_OMAP3_GPIO_3		/* GPIO64..95 is in GPIO bank 3 */
#define CONFIG_OMAP3_GPIO_4		/* GPIO96..127 is in GPIO Bank 4 */
#define CONFIG_OMAP3_GPIO_5		/* GPIO128..159 is in GPIO bank 5 */
#define CONFIG_OMAP3_GPIO_6		/* GPIO160..191 is in GPIO bank 6 */

/* NAND */
#define CONFIG_SYS_NAND_QUIET_TEST	1
#define CONFIG_NAND_OMAP_GPMC
#define CONFIG_SYS_MAX_NAND_DEVICE	1
#define CONFIG_SYS_FLASH_BASE		NAND_BASE

/* NAND boot config */
#define CONFIG_SYS_NAND_BUSWIDTH_16BIT	16
#define CONFIG_SYS_NAND_5_ADDR_CYCLE
#define CONFIG_SYS_NAND_PAGE_COUNT	64
#define CONFIG_SYS_NAND_PAGE_SIZE	2048
#define CONFIG_SYS_NAND_OOBSIZE		64
#define CONFIG_SYS_NAND_BLOCK_SIZE	(128*1024)
#define CONFIG_SYS_NAND_BAD_BLOCK_POS	0
#define CONFIG_SYS_NAND_ECCPOS		{2, 3, 4, 5, 6, 7, 8, 9,\
	10, 11, 12, 13}
#define CONFIG_SYS_NAND_ECCSIZE		512
#define CONFIG_SYS_NAND_ECCBYTES	3
#define CONFIG_NAND_OMAP_ECCSCHEME	OMAP_ECC_HAM1_CODE_HW
#define CONFIG_SYS_NAND_U_BOOT_OFFS	0x80000

/* NAND: SPL falcon mode configs */
#ifdef CONFIG_SPL_OS_BOOT
#define CONFIG_CMD_SPL_NAND_OFS		0x240000
#define CONFIG_SYS_NAND_SPL_KERNEL_OFFS	0x280000
#define CONFIG_CMD_SPL_WRITE_SIZE	0x2000
#endif

/* USB */
#define CONFIG_MUSB_GADGET
#define CONFIG_USB_MUSB_OMAP2PLUS
#define CONFIG_MUSB_PIO_ONLY
#define CONFIG_USB_GADGET_DUALSPEED
#define CONFIG_TWL4030_USB		1
#define CONFIG_USB_ETHER
#define CONFIG_USB_ETHER_RNDIS
#define CONFIG_USB_GADGET
#define CONFIG_USB_GADGET_VBUS_DRAW	2
#define CONFIG_USBDOWNLOAD_GADGET
#define CONFIG_G_DNL_VENDOR_NUM		0x0451
#define CONFIG_G_DNL_PRODUCT_NUM	0xd022
#define CONFIG_G_DNL_MANUFACTURER	"TI"
#define CONFIG_CMD_FASTBOOT
#define CONFIG_ANDROID_BOOT_IMAGE
#define CONFIG_USB_FASTBOOT_BUF_ADDR	CONFIG_SYS_LOAD_ADDR
#define CONFIG_USB_FASTBOOT_BUF_SIZE	0x07000000
#define CONFIG_USB_STORAGE

/* ENV related config options */
#define FAT_ENV_INTERFACE		"mmc"
#define FAT_ENV_DEVICE_AND_PART		"1"
#define FAT_ENV_FILE			"uboot.env"
#define CONFIG_ENV_IS_IN_FAT
#define CONFIG_FAT_WRITE
#define CONFIG_ENV_SIZE			0x4000

/* commands to include */
#include <config_cmd_default.h>

#define CONFIG_CMD_ASKENV
#define CONFIG_CMD_CACHE
#define CONFIG_CMD_DHCP
#define CONFIG_CMD_GPIO
#define CONFIG_CMD_NAND
#define CONFIG_CMD_SETEXPR
#define CONFIG_CMD_SAVEENV

#define CONFIG_EXTRA_ENV_SETTINGS \
	DEFAULT_LINUX_BOOT_ENV \
	"fdtfile=dtb\0" \
	"bootfile=zimage\0" \
	"ramdisk=initramfs\0" \
	"fdtaddr=80a00000\0" \
	"loadaddr=80c00000\0" \
	"rdaddr=81600000\0" \
	"fdt_high=8c000000\0" \
	"initrd_high=ffffffff\0" \
	"bootpart=1:1\0" \
	"console=ttyO2,115200n8\0" \
	"mpurate=800\0" \
	"optargs=\0" \
	"mmcdev=1\0" \
	"mmcroot=/dev/mmcblk1p2 rw\0" \
	"mmcrootfstype=ext4 rootwait\0" \
	"nandroot=ubi0:rootfs ubi.mtd=7,512\0" \
	"nandrootfstype=ubifs rootwait\0" \
	"mmcargs=setenv bootargs console=${console} " \
		"${optargs} " \
		"mpurate=${mpurate} " \
		"root=${mmcroot} " \
		"rootfstype=${mmcrootfstype}\0" \
	"nandargs=setenv bootargs console=${console} " \
		"${optargs} " \
		"mpurate=${mpurate} " \
		"root=${nandroot} " \
		"rootfstype=${nandrootfstype}\0" \
	"bootenv=uEnv.txt\0" \
	"loadbootenv=fatload mmc ${mmcdev} ${loadaddr} ${bootenv}\0" \
	"importbootenv=echo Importing environment from mmc ...; " \
		"env import -t $loadaddr $filesize\0" \
	"loadramdisk=fatload mmc ${mmcdev} ${rdaddr} ${ramdisk}\0" \
	"loadzimage=fatload mmc ${mmcdev} ${loadaddr} zImage\0" \
	"loadfdt=fatload mmc ${mmcdev} ${fdtaddr} ${fdtfile}\0" \
	"distro_fdt=ext4load mmc ${mmcdev}:2 ${fdtaddr} /boot/dtbs/anvl.dtb\0" \
	"distro_kernel=ext4load mmc ${mmcdev}:2 ${loadaddr} /boot/vmlinuz-linux-anvl\0" \
	"distro_initramfs=ext4load mmc ${mmcdev}:2 ${rdaddr} /boot/initramfs-linux-anvl.img\0" \
	"distroboot=echo Booting distro kernel from mmc...;" \
		"run mmcargs; " \
		"run distro_kernel; " \
		"if run distro_initramfs; then " \
			"bootz ${loadaddr} ${rdaddr}:${filesize} ${fdtaddr}; " \
		"else " \
			"bootz ${loadaddr} - ${fdtaddr}; " \
		"fi;\0" \
	"mmcboot=echo Booting from mmc ...; " \
		"run mmcargs; " \
		"run loadfdt;" \
		"run loadramdisk;" \
		"bootz ${loadaddr} ${rdaddr}:${filesize} ${fdtaddr}\0" \
	"nandboot=echo Booting from nand ...; " \
		"run nandargs; " \
		"nandecc sw; " \
		"nand read ${loadaddr} 580000 800000; " \
		"fdt addr ${fdtaddr}; " \
		"nand read ${fdtaddr}  380000 200000; " \
		"fdt resize; " \
		"nand read ${rdaddr} d80000 1400000; " \
		"bootz ${loadaddr} ${rdaddr}:1400000 ${fdtaddr}\0" \

#define CONFIG_PREBOOT \
	"echo Checking for install script; " \
	"if source 80980000; then " \
		"echo Install done; " \
	"else " \
		"echo No install script found; " \
	"fi; "

#define CONFIG_BOOTCOMMAND \
	"mmc dev ${mmcdev}; if mmc rescan; then " \
		"if run loadbootenv; then " \
			"echo Loaded environment from ${bootenv};" \
				"run importbootenv;" \
		"fi;" \
		"if run distro_fdt; then " \
			"run distroboot; " \
		"elif run loadzimage; then " \
			"run mmcboot; " \
		"else run nandboot; " \
		"fi; " \
	"else run nandboot; fi"

#endif	/* __CONFIG_H */
