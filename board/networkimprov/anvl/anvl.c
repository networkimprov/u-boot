/*
 * Maintainer : Braden Marr <marr@pde.com>
 *
 * Derived from Beagle Board, 3430 SDP, and OMAP3EVM code by
 *	Richard Woodruff <r-woodruff2@ti.com>
 *	Syed Mohammed Khasim <khasim@ti.com>
 *	Sunil Kumar <sunilsaini05@gmail.com>
 *	Shashi Ranjan <shashiranjanmca05@gmail.com>
 *
 * (C) Copyright 2004-2008
 * Texas Instruments, <www.ti.com>
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */
#include <common.h>
#include <dm.h>
#include <ns16550.h>
#ifdef CONFIG_STATUS_LED
#include <status_led.h>
#endif
#include <twl4030.h>
#include <linux/mtd/nand.h>
#include <asm/io.h>
#include <asm/arch/mmc_host_def.h>
#include <asm/arch/mux.h>
#include <asm/arch/mem.h>
#include <asm/arch/sys_proto.h>
#include <asm/gpio.h>
#include <asm/mach-types.h>
#include <asm/omap_musb.h>
#include <asm/errno.h>
#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>
#include <linux/usb/musb.h>
#include "anvl.h"
#include <command.h>

#ifdef CONFIG_USB_EHCI
#include <usb.h>
#include <asm/ehci-omap.h>
#endif

#define TWL4030_I2C_BUS			0
#define BQ24190_I2C_BUS			0
#define BQ24190_I2C_ADDRESS		0x6B
#define PCA9632_I2C_BUS			1
#define PCA9632_I2C_ADDRESS		0x62
#define PCA9632_I2C_RESET_ADDRESS	0x03
#define PCA9632_I2C_REG_MODE1		0
#define PCA9632_I2C_REG_MODE2		1
#define PCA9632_I2C_REG_PWM0		2	/* Red		*/
#define PCA9632_I2C_REG_PWM1		3	/* Green	*/
#define PCA9632_I2C_REG_PWM2		4	/* Blue		*/
#define PCA9632_I2C_REG_PWM3		5
#define PCA9632_I2C_REG_GRPPWM		6
#define PCA9632_I2C_REG_GRPFREQ		7
#define PCA9632_I2C_REG_LEDOUT		8
#define PCA9632_I2C_REG_SUBADR1		9
#define PCA9632_I2C_REG_SUBADR2		10
#define PCA9632_I2C_REG_SUBADR3		11
#define PCA9632_I2C_REG_ALLCALL		12

#define OMAP3_GENERAL_REG_BASE		0x48002270
#define OMAP_CONTROL_DEVCONF1		(OMAP3_GENERAL_REG_BASE + 0x68)
#define OMAP_CONTROL_DEVCONF1_MMCSDIO2ADPCLKISEL	(1<<6)

#define MACH_TYPE_INVALID		65535

DECLARE_GLOBAL_DATA_PTR;

static const struct ns16550_platdata anvl_serial = {
	.base = OMAP34XX_UART3,
	.reg_shift = 2,
	.clock = V_NS16550_CLK
};

U_BOOT_DEVICE(anvl_uart) = {
	"ns16550_serial",
	&anvl_serial
};

/*
 * Routine: board_init
 * Description: Early hardware init.
 */
int board_init(void)
{
	u32 val;

	gpmc_init(); /* in SRAM or SDRAM, finish GPMC */
	/* board id for Linux */
	gd->bd->bi_arch_number = MACH_TYPE_OMAP_GENERIC;
	/* boot param addr */
	gd->bd->bi_boot_params = (OMAP34XX_SDRC_CS0 + 0x100);

	/* internal loopback of MMC2_CLK to MMC2_CLKIN */
	val = readl(OMAP_CONTROL_DEVCONF1);
	val |= OMAP_CONTROL_DEVCONF1_MMCSDIO2ADPCLKISEL;
	writel(val, OMAP_CONTROL_DEVCONF1);

	return 0;
}

static int gpio_request_get_value(int gpio)
{
	int res;

	res = gpio_request(gpio, "");
	if (res)
		return res;
	gpio_direction_input(gpio);
	res = gpio_get_value(gpio);
	gpio_free(gpio);

	return res;
}

/*
 * Routine: is_anvl_evt0
 * Description: Checks for anvl-evt0
 *
 *		sys_boot1/gpio3		sys_boot3/gpio_5
 * anvl-evt0:	1			1
 * anvl-evt1:	0			0
 */
static bool is_anvl_evt0(void)
{
	if ((gpio_request_get_value(3) > 0) &&
	    (gpio_request_get_value(5) > 0))
		return true;

	return false;
}

/*
 * Routine: get_board_revision
 * Description: Returns the board revision
 */
static int get_board_revision(void)
{
	u8 mask;
	int val;

	if (is_anvl_evt0())
		return 0;

	MUX_VAL(CP(DSS_DATA12), (IEN | PTD | EN | M4)) /*GPIO82*/
	MUX_VAL(CP(DSS_DATA14), (IEN | PTD | EN | M4)) /*GPIO84*/
	MUX_VAL(CP(DSS_DATA23), (IEN | PTD | EN | M4)) /*GPIO93*/

	val = gpio_request_get_value(82);
	if (val)
		MUX_VAL(CP(DSS_DATA12), (IEN | PTD | DIS | M4))
	else
		MUX_VAL(CP(DSS_DATA12), (IEN | PTD | EN | M4))
	mask = val;

	val = gpio_request_get_value(84);
	if (val)
		MUX_VAL(CP(DSS_DATA14), (IEN | PTD | DIS | M4))
	else
		MUX_VAL(CP(DSS_DATA14), (IEN | PTD | EN | M4))
	mask |= (val << 1);

	val = gpio_request_get_value(93);
	if (val)
		MUX_VAL(CP(DSS_DATA23), (IEN | PTD | DIS | M4))
	else
		MUX_VAL(CP(DSS_DATA23), (IEN | PTD | EN | M4))
	mask |= (val << 2);

	gpio_free(82);
	gpio_free(84);
	gpio_free(93);

	return mask + 1;
}

#ifdef CONFIG_SPL_BUILD
/*
 * Routine: get_board_mem_timings
 * Description: If we use SPL then there is no x-loader nor config header
 * so we have to setup the DDR timings ourself on both banks.
 */
void get_board_mem_timings(struct board_sdrc_timings *timings)
{
	int pop_mfr, pop_id;

	/*
	 * We need to identify what PoP memory is on the board so that
	 * we know what timings to use.  If we can't identify it then
	 * we know it's an xM.  To map the ID values please see nand_ids.c
	 */
	identify_nand_chip(&pop_mfr, &pop_id);

	timings->mr = MICRON_V_MR_165;
	switch (get_board_revision()) {
	case 0:
		if (pop_mfr == NAND_MFR_MICRON && pop_id == 0xba) {
			/* Anvl-evt0, 512MB Nand/256MB DDR*/
			timings->mcfg = MICRON_V_MCFG_165(128 << 20);
			timings->ctrla = MICRON_V_ACTIMA_165;
			timings->ctrlb = MICRON_V_ACTIMB_165;
			timings->rfr_ctrl = SDP_3430_SDRC_RFR_CTRL_165MHz;
			break;
		} else if (pop_mfr == NAND_MFR_MICRON && pop_id == 0xbc) {
			/* Anvl-evt0, 512MB Nand/512MB DDR */
			timings->mcfg = MICRON_V_MCFG_200(256 << 20);
			timings->ctrla = MICRON_V_ACTIMA_200;
			timings->ctrlb = MICRON_V_ACTIMB_200;
			timings->rfr_ctrl = SDP_3430_SDRC_RFR_CTRL_200MHz;
			break;
		}
	case 1:
	case 2:
	default:
		/* Anvl-evt1 and later 512MB DDR */
		timings->mcfg = MICRON_V_MCFG_200(256 << 20);
		timings->ctrla = MICRON_V_ACTIMA_200;
		timings->ctrlb = MICRON_V_ACTIMB_200;
		timings->rfr_ctrl = SDP_3430_SDRC_RFR_CTRL_200MHz;
		break;
	}
}
#endif

#ifdef CONFIG_USB_MUSB_OMAP2PLUS
static struct musb_hdrc_config musb_config = {
	.multipoint     = 1,
	.dyn_fifo       = 1,
	.num_eps        = 16,
	.ram_bits       = 12,
};

static struct omap_musb_board_data musb_board_data = {
	.interface_type	= MUSB_INTERFACE_ULPI,
};

static struct musb_hdrc_platform_data musb_plat = {
#if defined(CONFIG_USB_MUSB_HOST)
	.mode           = MUSB_HOST,
#elif defined(CONFIG_USB_MUSB_GADGET)
	.mode		= MUSB_PERIPHERAL,
#else
#error "Please define either CONFIG_MUSB_HOST or CONFIG_MUSB_GADGET"
#endif
	.config         = &musb_config,
	.power          = 250,
	.platform_ops	= &omap2430_ops,
	.board_data	= &musb_board_data,
};
#endif

/*
 * Routine: misc_init_r
 * Description: Configure board specific parts
 */

#define TWL4030_PM_MASTER_P_TRANSITION_STARTON_SWBUG	1<<7
#define TWL4030_PM_MASTER_P_TRANSITION_STARTON_VBUS	1<<5
#define TWL4030_PM_MASTER_P_TRANSITION_STARTON_VBAT	1<<4
#define TWL4030_PM_MASTER_P_TRANSITION_STARTON_RTC	1<<3
#define TWL4030_PM_MASTER_P_TRANSITION_STARTON_USB	1<<2
#define TWL4030_PM_MASTER_P_TRANSITION_STARTON_CHG	1<<1
#define TWL4030_PM_MASTER_P_TRANSITION_STARTON_PWRON	1

#define TWL4030_RTC_SECONDS_REG				0x1c
#define TWL4030_RTC_MINUTES_REG				0x1d
#define TWL4030_RTC_HOURS_REG				0x1e
#define TWL4030_RTC_DAYS_REG				0x1f
#define TWL4030_RTC_MONTHS_REG				0x20
#define TWL4030_RTC_YEARS_REG				0x21
#define TWL4030_RTC_WEEKS_REG				0x22
#define TWL4030_RTC_ALARM_SECONDS_REG			0x23
#define TWL4030_RTC_ALARM_MINUTES_REG			0x24
#define TWL4030_RTC_ALARM_HOURS_REG			0x25
#define TWL4030_RTC_ALARM_DAYS_REG			0x26
#define TWL4030_RTC_ALARM_MONTHS_REG			0x27
#define TWL4030_RTC_ALARM_YEARS_REG			0x28
#define TWL4030_RTC_RTC_CTRL_REG			0x29
#define TWL4030_RTC_RTC_STATUS_REG			0x2a
#define TWL4030_RTC_RTC_INTERRUPTS_REG			0x2b
#define TWL4030_RTC_RTC_COMP_LSB_REG			0x2c
#define TWL4030_RTC_RTC_COMP_MSB_REG			0x2d

#define TWL4030_RTC_RTC_CTRL_GET_TIME			1<<6
#define TWL4030_RTC_RTC_CTRL_SET_32_COUNTER		1<<5
#define TWL4030_RTC_RTC_CTRL_TEST_MODE			1<<4
#define TWL4030_RTC_RTC_CTRL_MODE_12_24			1<<3
#define TWL4030_RTC_RTC_CTRL_AUTO_COMP			1<<2
#define TWL4030_RTC_RTC_CTRL_ROUND_30S			1<<1
#define TWL4030_RTC_RTC_CTRL_STOP_RTC			1

#define TWL4030_RTC_RTC_STATUS_POWER_UP			1<<7
#define TWL4030_RTC_RTC_STATUS_ALARM			1<<6
#define TWL4030_RTC_RTC_STATUS_1D_EVENT			1<<5
#define TWL4030_RTC_RTC_STATUS_1H_EVENT			1<<4
#define TWL4030_RTC_RTC_STATUS_1M_EVENT			1<<3
#define TWL4030_RTC_RTC_STATUS_1S_EVENT			1<<2
#define TWL4030_RTC_RTC_STATUS_RUN			1<<1

#define OMAP_GPIO_OUTPUT		0
#define OMAP_GPIO_INPUT			1
#define GPIO_BANK_TWL			7

struct anvl_gpio_config {
	char *name;
	int gpio_chip;
	int gpio;
	int is_input;
	int value;
	int delay;
};

enum anvl_gpios {
	BOOT_LED,
	VBAT_ADC_START,
	WLAN_WKUPN,
	WLAN_EEPROM_WP,
	WLAN_SLEEPN,
	WLAN_HOST_WKUP,
	HW_ID_1,
	HW_ID_2,
	HW_ID_3,
	WLAN_PDN,
	WLAN_RESET,
	EMMC_RESETN,
	OTG_CTRL,
};

static struct anvl_gpio_config anvl_evt0_gpios[] = {
	/* GPIO name        GPIO chip      GPIO Direction      Value Delay */
	{ "BOOT_LED",	    0,             175, OMAP_GPIO_OUTPUT, 0,    0, },
	{ "VBAT_ADC_START", 0,              66, OMAP_GPIO_OUTPUT, 0,    0, },
	{ "WLAN_WKUPN",     0,             153, OMAP_GPIO_OUTPUT, 1,    0, },
	{ "WLAN_EEPROM_WP", 0,             176, OMAP_GPIO_OUTPUT, 1,    0, },
	{ "WLAN_SLEEPN",    0,             177, OMAP_GPIO_INPUT,  0,    0, },
	{ "WLAN_HOST_WKUP", 0,              10, OMAP_GPIO_INPUT,  0,    0, },
	{ NULL,             0,               0, 0,                0,    0, },
	{ NULL,             0,               0, 0,                0,    0, },
	{ NULL,             0,               0, 0,                0,    0, },
	{ "WLAN_PDN",       0,             155, OMAP_GPIO_OUTPUT, 0,    0, },
	{ NULL,             0,               0, 0,                0,    0, },
	{ "EMMC_RESETN",    0,              75, OMAP_GPIO_OUTPUT, 1,  100, },
	{ NULL,             0,               0, 0,                0,    0, },
};

static struct anvl_gpio_config anvl_evt1_gpios[] = {
	/* GPIO name        GPIO chip      GPIO Direction      Value Delay */
	{ "BOOT_LED",       0,              14, OMAP_GPIO_OUTPUT, 0,    0, },
	{ NULL,             0,               0, 0,                0,    0, },
	{ "WLAN_WKUPN",     0,             153, OMAP_GPIO_OUTPUT, 1,    0, },
	{ "WLAN_EEPROM_WP", 0,             176, OMAP_GPIO_OUTPUT, 1,    0, },
	{ "WLAN_SLEEPN",    0,             177, OMAP_GPIO_INPUT,  0,    0, },
	{ "WLAN_HOST_WKUP", 0,              10, OMAP_GPIO_INPUT,  0,    0, },
	{ "HW_ID_1",        0,              82, OMAP_GPIO_INPUT,  0,    0, },
	{ "HW_ID_2",        0,              84, OMAP_GPIO_INPUT,  0,    0, },
	{ "HW_ID_3",        0,              93, OMAP_GPIO_INPUT,  0,    0, },
	{ "WLAN_PDN",       GPIO_BANK_TWL,   0, OMAP_GPIO_OUTPUT, 0,    0, },
	{ "WLAN_RESET",     GPIO_BANK_TWL,   1, OMAP_GPIO_OUTPUT, 1,  100, },
	{ "EMMC_RESETN",    GPIO_BANK_TWL,   2, OMAP_GPIO_OUTPUT, 1,  100, },
	{ "OTG_CTRL",       GPIO_BANK_TWL,   6, OMAP_GPIO_OUTPUT, 1,    0, },
};

static int anvl_set_omap_gpio(struct anvl_gpio_config *g)
{
	int res;

	res = gpio_request(g->gpio, g->name);
	if (res)
		return -ENODEV;
	if (g->is_input) {
		gpio_direction_input(g->gpio);
		printf("GPIO: Set %s gpio%i to input\n",
		       g->name, g->gpio);
		return 0;
	}
	if (g->delay) {
		printf("GPIO: Toggle %s gpio%i output to %i for %ius\n",
		       g->name, g->gpio, !g->value, g->delay);
		gpio_direction_output(g->gpio, !g->value);
		udelay(g->delay);
	} else {
		gpio_direction_output(g->gpio, g->value);
	}
	gpio_set_value(g->gpio, g->value);
	printf("GPIO: Set %s gpio%i output to %i\n", g->name, g->gpio,
	       g->value);

	gpio_free(g->gpio);

	return 0;
}
/* Note that GPIOs need to be muxed using INTRBR_PMBR1 & 2 */
static int anvl_set_twl_gpio(struct anvl_gpio_config *g)
{
	int set, clear;
	u8 val;

	/* Set direction */
	set = TWL4030_BASEADD_GPIO + TWL4030_GPIO_GPIODATADIR1;
	twl4030_i2c_read_u8(TWL4030_CHIP_GPIO, set, &val);
	if (g->is_input)
		val &= ~(1 << g->gpio);
	else
		val |= (1 << g->gpio);
	twl4030_i2c_write_u8(TWL4030_CHIP_GPIO, set, val);

	if (g->is_input) {
		printf("GPIO: Set %s twl_gpio%i to input\n", g->name, g->gpio);
		return 0;
	}

	set = TWL4030_BASEADD_GPIO + TWL4030_GPIO_SETGPIODATAOUT1;
	clear = TWL4030_BASEADD_GPIO + TWL4030_GPIO_CLEARGPIODATAOUT1;
	val = (1 << g->gpio);

	/* Toggle to oppsite value first if delay is specified */
	if (g->delay) {
		printf("GPIO: Toggle %s twl_gpio%i output to %i for %ius\n",
		       g->name, g->gpio, !g->value, g->delay);
		if (g->value)
			twl4030_i2c_write_u8(TWL4030_CHIP_GPIO, clear, val);
		else
			twl4030_i2c_write_u8(TWL4030_CHIP_GPIO, set, val);
		udelay(g->delay);
	}

	/* Set the final value */
	if (g->value)
		twl4030_i2c_write_u8(TWL4030_CHIP_GPIO, set, val);
	else
		twl4030_i2c_write_u8(TWL4030_CHIP_GPIO, clear, val);
	twl4030_i2c_read_u8(TWL4030_CHIP_GPIO, set, &val);

	printf("GPIO: Set %s twl_gpio%i output to %i\n", g->name,
	       g->gpio, g->value);

	return 0;
}

static int anvl_set_gpio(struct anvl_gpio_config *g)
{
	int res;

	if (!g || !g->name)
		return -EINVAL;

	if (g->gpio_chip == GPIO_BANK_TWL)
		res = anvl_set_twl_gpio(g);
	else
		res = anvl_set_omap_gpio(g);

	if (res)
		printf("GPIO: %s gpio%i failed: %i\n", g->name,
		       g->gpio, res);

	return res;
}

static void anvl_save_dieid(void)
{
	unsigned int id[4] = { 0 };
	char s[32 + 1];

	memset(s, 0, sizeof(s));
	omap_die_id(id);
	sprintf(s, "%08x%08x%08x%08x", id[3], id[2], id[1], id[0]);
	setenv("dieid#", s);
}

int misc_init_r(void)
{
	struct anvl_gpio_config *gpios;
	int anvl_evt0, revision, res;
	u8 buf;
	int val;
	char *str;

	anvl_evt0 = is_anvl_evt0();
	if (anvl_evt0)
		gpios = anvl_evt0_gpios;
	else
		gpios = anvl_evt1_gpios;

	revision = get_board_revision();

	/* Disable boot LED N-CH FET color mixer by forcing BOOT_LED = LOW */
	anvl_set_gpio(&gpios[BOOT_LED]);

	/* Power for LED controller using VAUX2 starting with anvl-evt2 */
	if (revision >= 2) {
		i2c_set_bus_num(TWL4030_I2C_BUS);
		twl4030_pmrecv_vsel_cfg(TWL4030_PM_RECEIVER_VAUX2_DEDICATED,
					TWL4030_PM_RECEIVER_VAUX2_VSEL_25,
					TWL4030_PM_RECEIVER_VAUX2_DEV_GRP,
					TWL4030_PM_RECEIVER_DEV_GRP_P1);
	}

	/* Reset and configure LED controller */
	i2c_set_bus_num(PCA9632_I2C_BUS);
	buf = 0xff;
	i2c_write(PCA9632_I2C_RESET_ADDRESS, 0x00, 1, &buf, 1);
	udelay(500);

	i2c_read(PCA9632_I2C_ADDRESS, PCA9632_I2C_REG_MODE1, 1, &buf, 1);
	buf = (buf | 0x01) & ~0x10;	/* Normal Mode, All Call	*/
	i2c_write(PCA9632_I2C_ADDRESS, PCA9632_I2C_REG_MODE1, 1, &buf, 1);

	buf = 0x3f;	/* Enable LED[2:0] for output, disable LED3 (unused) */
	i2c_write(PCA9632_I2C_ADDRESS, PCA9632_I2C_REG_LEDOUT, 1, &buf, 1);

	i2c_read(PCA9632_I2C_ADDRESS, PCA9632_I2C_REG_MODE2, 1, &buf, 1);
	buf |= 0x20;	/* DMBLNK = 1, Enable blinking	*/
	i2c_write(PCA9632_I2C_ADDRESS, PCA9632_I2C_REG_MODE2, 1, &buf, 1);

	/* Set the boot blink color */

	buf = getenv_hex("anvl_red_brightness", 0x20);
	i2c_write(PCA9632_I2C_ADDRESS, PCA9632_I2C_REG_PWM0, 1, &buf, 1);

	buf = getenv_hex("anvl_green_brightness", 0x00);
	i2c_write(PCA9632_I2C_ADDRESS, PCA9632_I2C_REG_PWM1, 1, &buf, 1);

	buf = getenv_hex("anvl_blue_brightness", 0x20);
	i2c_write(PCA9632_I2C_ADDRESS, PCA9632_I2C_REG_PWM2, 1, &buf, 1);

	buf = getenv_hex("anvl_grppwm", 0) & 0xff;
	i2c_write(PCA9632_I2C_ADDRESS, PCA9632_I2C_REG_GRPPWM, 1, &buf, 1);

	buf = getenv_hex("anvl_grpfreq", 0) & 0xff;
	i2c_write(PCA9632_I2C_ADDRESS, PCA9632_I2C_REG_GRPFREQ, 1, &buf, 1);

	i2c_set_bus_num(TWL4030_I2C_BUS);

	/* Start RTC if it isn't already ticking */

	if (twl4030_i2c_read_u8(TWL4030_CHIP_RTC,
				TWL4030_RTC_RTC_STATUS_REG, &buf)) {
		printf("Error:TWL4030: failed to read the power register\n");
		printf("Could not read RTC status\n");
	} else if (0 == (buf & TWL4030_RTC_RTC_STATUS_RUN)) {
		if (twl4030_i2c_read_u8(TWL4030_CHIP_RTC,
					TWL4030_RTC_RTC_CTRL_REG, &buf)) {
			printf("Error:TWL4030: failed to read the power register\n");
			printf("Could not start RTC\n");
		} else {
			buf |= TWL4030_RTC_RTC_CTRL_STOP_RTC; /* unfreeze RTC */
			if (twl4030_i2c_write_u8(TWL4030_CHIP_RTC,
						 TWL4030_RTC_RTC_CTRL_REG, buf)) {
				printf("Error:TWL4030: failed to write the power register\n");
				printf("Could not start RTC\n");
			} else {
				printf("RTC: started - may have incorrect time\n");
			}
		}
	} else {
		printf("RTC: ok\n");
	}

	/* Configure power-up conditions */

	if (twl4030_i2c_read_u8(TWL4030_CHIP_PM_MASTER,
				TWL4030_PM_MASTER_CFG_P1_TRANSITION, &buf)) {
		printf("Error:TWL4030: failed to read the power register\n");
		printf("Could not configure P1 hardware power transitions\n");
	} else {
		buf &= ~(TWL4030_PM_MASTER_P_TRANSITION_STARTON_USB |
			 TWL4030_PM_MASTER_P_TRANSITION_STARTON_CHG);
		buf |= TWL4030_PM_MASTER_P_TRANSITION_STARTON_PWRON;

		if (twl4030_i2c_write_u8(TWL4030_CHIP_PM_MASTER,
					 TWL4030_PM_MASTER_CFG_P1_TRANSITION, buf)) {
			printf("Error:TWL4030: failed to write the power register\n");
			printf("Could not configure P1 hardware power transitions\n");
		} else {
			printf("PMIC: P1 startup triggers are USB, Charger, Power Switch\n");
		}
	}

	if (twl4030_i2c_read_u8(TWL4030_CHIP_PM_MASTER,
				TWL4030_PM_MASTER_CFG_P2_TRANSITION, &buf)) {
		printf("Error:TWL4030: failed to read the power register\n");
		printf("Could not configure P2 hardware power transitions\n");
	} else {
		buf &= ~(TWL4030_PM_MASTER_P_TRANSITION_STARTON_USB |
			 TWL4030_PM_MASTER_P_TRANSITION_STARTON_CHG);
		buf |= TWL4030_PM_MASTER_P_TRANSITION_STARTON_PWRON;

		if (twl4030_i2c_write_u8(TWL4030_CHIP_PM_MASTER,
					 TWL4030_PM_MASTER_CFG_P2_TRANSITION, buf)) {
			printf("Error:TWL4030: failed to write the power register\n");
			printf("Could not configure P2 hardware power transitions\n");
		} else {
			printf("PMIC: P2 startup triggers are USB, Charger, Power Switch\n");
		}
	}

	if (twl4030_i2c_read_u8(TWL4030_CHIP_PM_MASTER,
				TWL4030_PM_MASTER_CFG_P3_TRANSITION, &buf)) {
		printf("Error:TWL4030: failed to read the power register\n");
		printf("Could not configure P3 hardware power transitions\n");
	} else {
		buf &= ~(TWL4030_PM_MASTER_P_TRANSITION_STARTON_USB |
			 TWL4030_PM_MASTER_P_TRANSITION_STARTON_CHG);
		buf |= TWL4030_PM_MASTER_P_TRANSITION_STARTON_PWRON;

		if (twl4030_i2c_write_u8(TWL4030_CHIP_PM_MASTER,
					 TWL4030_PM_MASTER_CFG_P3_TRANSITION, buf)) {
			printf("Error:TWL4030: failed to write the power register\n");
			printf("Could not configure P3 hardware power transitions\n");
		} else {
			printf("PMIC: P3 startup triggers are USB, Charger, Power Switch\n");
		}
	}

	/* Enable backup battery charger */

	if (twl4030_i2c_read_u8(TWL4030_CHIP_PM_MASTER,
				TWL4030_PM_MASTER_BB_CFG, &buf)) {
		printf("Error:TWL4030: failed to read the power register\n");
		printf("Could not enable backup battery charger\n");
	} else {
		/* Enable | 3.0V | charge current at 25 uA */
		buf = (buf & 0xf0) | (1<<4) | (1<<2) | (0<<0);

		if (twl4030_i2c_write_u8(TWL4030_CHIP_PM_MASTER,
					 TWL4030_PM_MASTER_BB_CFG, buf)) {
			printf("Error:TWL4030: failed to write the power register\n");
			printf("Could not enable backup battery charger\n");
		} else {
			printf("Backup battery charger enabled\n");
		}
	}

	/* Configure power-down conditions */

	if (twl4030_i2c_read_u8(TWL4030_CHIP_PM_MASTER,
				TWL4030_PM_MASTER_P1_SW_EVENTS, &buf)) {
		printf("Error:TWL4030: failed to read the power register\n");
		printf("Could not configure power down conditions\n");
	} else {
		/* Enable PWRON switch shutdown */
		/* TODO: enable LVL_WAKEUP (bit 3) to activate nSLEEP1 */
		buf |= 1<<6;

		if (twl4030_i2c_write_u8(TWL4030_CHIP_PM_MASTER,
					 TWL4030_PM_MASTER_P1_SW_EVENTS, buf)) {
			printf("Error:TWL4030: failed to write the power register\n");
			printf("Could not configure power down conditions\n");
		} else {
			printf("PMIC: P1 power down trigger is Power Switch\n");
		}
	}

	/* Set OTG pin high for 500mA for bq24190 */
	if (!anvl_evt0)
		anvl_set_gpio(&gpios[OTG_CTRL]);

	/* Ensure reset of eMMC and bring it out of reset */
	anvl_set_gpio(&gpios[EMMC_RESETN]);

	/* Set ADC START GPIO default state */
	if (anvl_evt0)
		anvl_set_gpio(&gpios[VBAT_ADC_START]);

	/* Decide WLAN module power by environment variable */
	if ((str = getenv("gpio_wlan_pdn")) != NULL) {
		val = simple_strtoul(str, NULL, 10);
	} else {
		/* WLAN powered down by default */
		setenv("gpio_wlan_pdn", "0");
		val = 0;
	}

	/* Bring WLAN out of sleep, previous state unknown */
	anvl_set_gpio(&gpios[WLAN_SLEEPN]);

	/*
	 * Set WLAN_PDN low and do a reset. Enable the WLAN
	 * depending on the gpio_wlan_pdn environment variable.
	 * During WLAN_RESET WLAN_PDN must stay low for 300ms.
	 */
	res = anvl_set_gpio(&gpios[WLAN_PDN]);
	if (!res) {
		if (!anvl_evt0) {
			anvl_set_gpio(&gpios[WLAN_RESET]);
			mdelay(300);
		}
		gpios[WLAN_PDN].value = val;
		anvl_set_gpio(&gpios[WLAN_PDN]);
		anvl_set_gpio(&gpios[WLAN_EEPROM_WP]);
	}

	i2c_set_bus_num(TWL4030_I2C_BUS);

	printf("Board revision: anvl-evt%d\n", revision);

	omap_die_id_display();
	anvl_save_dieid();

#ifdef CONFIG_USB_MUSB_OMAP2PLUS
	musb_register(&musb_plat, &musb_board_data, (void *)MUSB_BASE);
#endif
	omap_die_id_usbethaddr();

	return 0;
}

#ifdef CONFIG_SPL_OS_BOOT
int spl_start_uboot(void)
{
	/* Break into full u-boot */
	return 1;
}
#endif

/*
 * Routine: set_muxconf_regs
 * Description: Setting up the configuration Mux registers specific to the
 *		hardware. Many pins need to be moved from protect to primary
 *		mode.
 */
void set_muxconf_regs(void)
{
	MUX_ANVL();
}

#if defined(CONFIG_GENERIC_MMC) && !defined(CONFIG_SPL_BUILD)
int board_mmc_init(bd_t *bis)
{
	omap_mmc_init(0, 0, 0, -1, -1);
	omap_mmc_init(1, 0, 0, -1, -1);
	omap_mmc_init(2, 0, 0, -1, -1);

	return 0;
}
#endif

#if defined(CONFIG_USB_ETHER) && defined(CONFIG_MUSB_GADGET)
int board_eth_init(bd_t *bis)
{
	return usb_eth_initialize(bis);
}
#endif

#if defined(CONFIG_USB_EHCI) && !defined(CONFIG_SPL_BUILD)
int ehci_hcd_init(int index, enum usb_init_type init,
		struct ehci_hccr **hccr, struct ehci_hcor **hcor)
{
	return 0;
}

int ehci_hcd_stop(int index)
{
	return 0;
}
#endif /* CONFIG_USB_EHCI */

#if 0 //!defined(CONFIG_SPL_BUILD)
/* Disable some interfaces and quiet the LEDs before booting a kernel */
void show_boot_progress(int val)
{
	u8 buf;
	unsigned int orig_bus_num;

	switch(val) {
		case BOOTSTAGE_ID_BOARD_INIT:

			/* Enable LED controller boot LED and blink */

			orig_bus_num = i2c_get_bus_num();
			i2c_set_bus_num(PCA9632_I2C_BUS);

			i2c_read(PCA9632_I2C_ADDRESS, PCA9632_I2C_REG_MODE2, 1, &buf, 1);
			buf |= 0x20;	/* DMBLNK = 1, Enable blinking	*/
			i2c_write(PCA9632_I2C_ADDRESS, PCA9632_I2C_REG_MODE2, 1, &buf, 1);

			/* Set the boot blink color	*/

			buf = 0xaa;	/* Red		*/
			i2c_write(PCA9632_I2C_ADDRESS, PCA9632_I2C_REG_PWM0, 1, &buf, 1);
			buf = 0x55;	/* Green	*/
			i2c_write(PCA9632_I2C_ADDRESS, PCA9632_I2C_REG_PWM1, 1, &buf, 1);
			buf = 0x00;	/* Blue		*/
			i2c_write(PCA9632_I2C_ADDRESS, PCA9632_I2C_REG_PWM2, 1, &buf, 1);

			buf = (12<<2) & 0xfc;	/* blink duty cycle (1 to 63 = 0 to 100%)	*/
			i2c_write(PCA9632_I2C_ADDRESS, PCA9632_I2C_REG_GRPPWM, 1, &buf, 1);

			buf = 3;	/* 12 x (1/24)s = 0.5s blink period	*/
			i2c_write(PCA9632_I2C_ADDRESS, PCA9632_I2C_REG_GRPFREQ, 1, &buf, 1);

			i2c_set_bus_num(orig_bus_num);
			break;

		case BOOTSTAGE_ID_BOARD_INIT_DONE:

			/* Enable LED controller boot LED and blink */

			orig_bus_num = i2c_get_bus_num();
			i2c_set_bus_num(PCA9632_I2C_BUS);

			i2c_read(PCA9632_I2C_ADDRESS, PCA9632_I2C_REG_MODE2, 1, &buf, 1);
			buf |= 0x20;	/* DMBLNK = 1, Enable blinking	*/
			i2c_write(PCA9632_I2C_ADDRESS, PCA9632_I2C_REG_MODE2, 1, &buf, 1);

			/* Set the boot blink color	*/

			buf = 0x00;	/* Red		*/
			i2c_write(PCA9632_I2C_ADDRESS, PCA9632_I2C_REG_PWM0, 1, &buf, 1);
			buf = 0x00;	/* Green	*/
			i2c_write(PCA9632_I2C_ADDRESS, PCA9632_I2C_REG_PWM1, 1, &buf, 1);
			buf = 0xaa;	/* Blue		*/
			i2c_write(PCA9632_I2C_ADDRESS, PCA9632_I2C_REG_PWM2, 1, &buf, 1);

			buf = (12<<2) & 0xfc;	/* blink duty cycle (1 to 63 = 0 to 100%)	*/
			i2c_write(PCA9632_I2C_ADDRESS, PCA9632_I2C_REG_GRPPWM, 1, &buf, 1);

			buf = 6;	/* 12 x (1/24)s = 0.5s blink period	*/
			i2c_write(PCA9632_I2C_ADDRESS, PCA9632_I2C_REG_GRPFREQ, 1, &buf, 1);

			i2c_set_bus_num(orig_bus_num);
			break;

		case BOOTSTAGE_ID_RUN_OS:
			orig_bus_num = i2c_get_bus_num();
			i2c_set_bus_num(PCA9632_I2C_BUS);

			i2c_read(PCA9632_I2C_ADDRESS, PCA9632_I2C_REG_MODE2, 1, &buf, 1);
			buf &= ~0x20;	/* DMBLNK = 0, Disable blinking	*/
			i2c_write(PCA9632_I2C_ADDRESS, PCA9632_I2C_REG_MODE2, 1, &buf, 1);

			/* Set RGB to a happy color */
			buf = 0x00;	/* Red		*/
			i2c_write(PCA9632_I2C_ADDRESS, PCA9632_I2C_REG_PWM0, 1, &buf, 1);
			buf = 0x11;	/* Green	*/
			i2c_write(PCA9632_I2C_ADDRESS, PCA9632_I2C_REG_PWM1, 1, &buf, 1);
			buf = 0x00;	/* Blue		*/
			i2c_write(PCA9632_I2C_ADDRESS, PCA9632_I2C_REG_PWM2, 1, &buf, 1);

			i2c_set_bus_num(orig_bus_num);
			break;

	}
}
#endif
