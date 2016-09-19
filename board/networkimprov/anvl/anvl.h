/*
 * (C) Copyright 2013
 * Pacific Design Engineering <marr@pde.com>
 * 
 * (C) Copyright 2008
 * Steve Sakoman <steve@sakoman.com>
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
#ifndef _ANVL_H_
#define _ANVL_H_

const omap3_sysinfo sysinfo = {
	DDR_STACKED,
	"Network Improv ANVL board",
	"NAND",
};

/* anvl revisions */
#define REVISION_0	0x0

/*
 * IEN  - Input Enable
 * IDIS - Input Disable
 * PTD  - Pull type Down
 * PTU  - Pull type Up
 * DIS  - Pull type selection is inactive
 * EN   - Pull type selection is active
 * M0   - Mode 0
 * M7   - SAFE_MODE
  * The commented string gives the final mux configuration for that pin
 */
#define MUX_ANVL() \
 /*SDRC PoP LPDDR */\
	MUX_VAL(CP(SDRC_D0),		(IEN  | PTD | DIS | M0)) /*SDRC_D0*/\
	MUX_VAL(CP(SDRC_D1),		(IEN  | PTD | DIS | M0)) /*SDRC_D1*/\
	MUX_VAL(CP(SDRC_D2),		(IEN  | PTD | DIS | M0)) /*SDRC_D2*/\
	MUX_VAL(CP(SDRC_D3),		(IEN  | PTD | DIS | M0)) /*SDRC_D3*/\
	MUX_VAL(CP(SDRC_D4),		(IEN  | PTD | DIS | M0)) /*SDRC_D4*/\
	MUX_VAL(CP(SDRC_D5),		(IEN  | PTD | DIS | M0)) /*SDRC_D5*/\
	MUX_VAL(CP(SDRC_D6),		(IEN  | PTD | DIS | M0)) /*SDRC_D6*/\
	MUX_VAL(CP(SDRC_D7),		(IEN  | PTD | DIS | M0)) /*SDRC_D7*/\
	MUX_VAL(CP(SDRC_D8),		(IEN  | PTD | DIS | M0)) /*SDRC_D8*/\
	MUX_VAL(CP(SDRC_D9),		(IEN  | PTD | DIS | M0)) /*SDRC_D9*/\
	MUX_VAL(CP(SDRC_D10),		(IEN  | PTD | DIS | M0)) /*SDRC_D10*/\
	MUX_VAL(CP(SDRC_D11),		(IEN  | PTD | DIS | M0)) /*SDRC_D11*/\
	MUX_VAL(CP(SDRC_D12),		(IEN  | PTD | DIS | M0)) /*SDRC_D12*/\
	MUX_VAL(CP(SDRC_D13),		(IEN  | PTD | DIS | M0)) /*SDRC_D13*/\
	MUX_VAL(CP(SDRC_D14),		(IEN  | PTD | DIS | M0)) /*SDRC_D14*/\
	MUX_VAL(CP(SDRC_D15),		(IEN  | PTD | DIS | M0)) /*SDRC_D15*/\
	MUX_VAL(CP(SDRC_D16),		(IEN  | PTD | DIS | M0)) /*SDRC_D16*/\
	MUX_VAL(CP(SDRC_D17),		(IEN  | PTD | DIS | M0)) /*SDRC_D17*/\
	MUX_VAL(CP(SDRC_D18),		(IEN  | PTD | DIS | M0)) /*SDRC_D18*/\
	MUX_VAL(CP(SDRC_D19),		(IEN  | PTD | DIS | M0)) /*SDRC_D19*/\
	MUX_VAL(CP(SDRC_D20),		(IEN  | PTD | DIS | M0)) /*SDRC_D20*/\
	MUX_VAL(CP(SDRC_D21),		(IEN  | PTD | DIS | M0)) /*SDRC_D21*/\
	MUX_VAL(CP(SDRC_D22),		(IEN  | PTD | DIS | M0)) /*SDRC_D22*/\
	MUX_VAL(CP(SDRC_D23),		(IEN  | PTD | DIS | M0)) /*SDRC_D23*/\
	MUX_VAL(CP(SDRC_D24),		(IEN  | PTD | DIS | M0)) /*SDRC_D24*/\
	MUX_VAL(CP(SDRC_D25),		(IEN  | PTD | DIS | M0)) /*SDRC_D25*/\
	MUX_VAL(CP(SDRC_D26),		(IEN  | PTD | DIS | M0)) /*SDRC_D26*/\
	MUX_VAL(CP(SDRC_D27),		(IEN  | PTD | DIS | M0)) /*SDRC_D27*/\
	MUX_VAL(CP(SDRC_D28),		(IEN  | PTD | DIS | M0)) /*SDRC_D28*/\
	MUX_VAL(CP(SDRC_D29),		(IEN  | PTD | DIS | M0)) /*SDRC_D29*/\
	MUX_VAL(CP(SDRC_D30),		(IEN  | PTD | DIS | M0)) /*SDRC_D30*/\
	MUX_VAL(CP(SDRC_D31),		(IEN  | PTD | DIS | M0)) /*SDRC_D31*/\
	MUX_VAL(CP(SDRC_CLK),		(IEN  | PTD | DIS | M0)) /*SDRC_CLK*/\
	MUX_VAL(CP(SDRC_DQS0),		(IEN  | PTD | DIS | M0)) /*SDRC_DQS0*/\
	MUX_VAL(CP(SDRC_DQS1),		(IEN  | PTD | DIS | M0)) /*SDRC_DQS1*/\
	MUX_VAL(CP(SDRC_DQS2),		(IEN  | PTD | DIS | M0)) /*SDRC_DQS2*/\
	MUX_VAL(CP(SDRC_DQS3),		(IEN  | PTD | DIS | M0)) /*SDRC_DQS3*/\
	MUX_VAL(CP(SDRC_CKE0),		(IDIS | PTU | EN  | M0)) /*SDRC_CKE0*/\
	MUX_VAL(CP(SDRC_CKE1),		(IDIS | PTU | EN  | M0)) /*SDRC_CKE1*/\
 /*GPMC PoP NAND does not use address bits but enable anyway for now */\
	MUX_VAL(CP(GPMC_A1),		(IDIS | PTU | EN  | M7)) /*SAFE_MODE*/\
	MUX_VAL(CP(GPMC_A2),		(IDIS | PTU | EN  | M7)) /*SAFE_MODE*/\
	MUX_VAL(CP(GPMC_A3),		(IDIS | PTU | EN  | M7)) /*SAFE_MODE*/\
	MUX_VAL(CP(GPMC_A4),		(IDIS | PTU | EN  | M7)) /*SAFE_MODE*/\
	MUX_VAL(CP(GPMC_A5),		(IDIS | PTU | EN  | M7)) /*SAFE_MODE*/\
	MUX_VAL(CP(GPMC_A6),		(IDIS | PTU | EN  | M7)) /*SAFE_MODE*/\
	MUX_VAL(CP(GPMC_A7),		(IDIS | PTU | EN  | M7)) /*SAFE_MODE*/\
	MUX_VAL(CP(GPMC_A8),		(IDIS | PTU | EN  | M7)) /*SAFE_MODE*/\
	MUX_VAL(CP(GPMC_A9),		(IDIS | PTU | EN  | M7)) /*SAFE_MODE*/\
	MUX_VAL(CP(GPMC_A10),		(IDIS | PTU | EN  | M7)) /*SAFE_MODE*/\
/*GPMC PoP NAND x16 GPMC_AD[15:0] */\
	MUX_VAL(CP(GPMC_D0),		(IEN  | PTU | EN  | M0)) /*GPMC_D0*/\
	MUX_VAL(CP(GPMC_D1),		(IEN  | PTU | EN  | M0)) /*GPMC_D1*/\
	MUX_VAL(CP(GPMC_D2),		(IEN  | PTU | EN  | M0)) /*GPMC_D2*/\
	MUX_VAL(CP(GPMC_D3),		(IEN  | PTU | EN  | M0)) /*GPMC_D3*/\
	MUX_VAL(CP(GPMC_D4),		(IEN  | PTU | EN  | M0)) /*GPMC_D4*/\
	MUX_VAL(CP(GPMC_D5),		(IEN  | PTU | EN  | M0)) /*GPMC_D5*/\
	MUX_VAL(CP(GPMC_D6),		(IEN  | PTU | EN  | M0)) /*GPMC_D6*/\
	MUX_VAL(CP(GPMC_D7),		(IEN  | PTU | EN  | M0)) /*GPMC_D7*/\
	MUX_VAL(CP(GPMC_D8),		(IEN  | PTU | EN  | M0)) /*GPMC_D8*/\
	MUX_VAL(CP(GPMC_D9),		(IEN  | PTU | EN  | M0)) /*GPMC_D9*/\
	MUX_VAL(CP(GPMC_D10),		(IEN  | PTU | EN  | M0)) /*GPMC_D10*/\
	MUX_VAL(CP(GPMC_D11),		(IEN  | PTU | EN  | M0)) /*GPMC_D11*/\
	MUX_VAL(CP(GPMC_D12),		(IEN  | PTU | EN  | M0)) /*GPMC_D12*/\
	MUX_VAL(CP(GPMC_D13),		(IEN  | PTU | EN  | M0)) /*GPMC_D13*/\
	MUX_VAL(CP(GPMC_D14),		(IEN  | PTU | EN  | M0)) /*GPMC_D14*/\
	MUX_VAL(CP(GPMC_D15),		(IEN  | PTU | EN  | M0)) /*GPMC_D15*/\
	MUX_VAL(CP(GPMC_NCS0),		(IDIS | PTU | EN  | M0)) /*GPMC_nCS0*/\
	MUX_VAL(CP(GPMC_NCS1),		(IDIS | PTU | EN  | M0)) /*GPMC_nCS1*/\
	MUX_VAL(CP(GPMC_NCS2),		(IEN  | PTU | DIS | M7)) /*SAFE_MODE*/\
	MUX_VAL(CP(GPMC_NCS3),		(IEN  | PTU | DIS | M7)) /*SAFE_MODE*/\
	MUX_VAL(CP(GPMC_NCS4),		(IEN  | PTU | DIS | M7)) /*SAFE_MODE*/\
	MUX_VAL(CP(GPMC_NCS5),		(IEN  | PTU | DIS | M7)) /*SAFE_MODE*/\
	MUX_VAL(CP(GPMC_NCS6),		(IEN  | PTU | DIS | M7)) /*SAFE_MODE*/\
	MUX_VAL(CP(GPMC_NCS7),		(IEN  | PTU | DIS | M7)) /*SAFE_MODE*/\
	MUX_VAL(CP(GPMC_NBE1),		(IDIS | PTD | DIS | M0)) /*GPMC_NBE1*/\
	MUX_VAL(CP(GPMC_CLK),		(IDIS | PTD | DIS | M0)) /*GPMC_CLK*/\
	MUX_VAL(CP(GPMC_NADV_ALE),	(IDIS | PTD | DIS | M0)) /*GPMC_nADV_ALE*/\
	MUX_VAL(CP(GPMC_NOE),		(IDIS | PTD | DIS | M0)) /*GPMC_nOE*/\
	MUX_VAL(CP(GPMC_NWE),		(IDIS | PTD | DIS | M0)) /*GPMC_nWE*/\
	MUX_VAL(CP(GPMC_NBE0_CLE),	(IDIS | PTD | DIS | M0)) /*GPMC_nBE0_CLE*/\
	MUX_VAL(CP(GPMC_NWP),		(IEN  | PTU | EN  | M0)) /*GPMC_nWP*/\
	MUX_VAL(CP(GPMC_WAIT0),		(IEN  | PTU | EN  | M0)) /*GPMC_WAIT0*/\
	MUX_VAL(CP(GPMC_WAIT1),		(IEN  | PTU | EN  | M0)) /*GPMC_WAIT1*/\
	MUX_VAL(CP(GPMC_WAIT2),		(IEN  | PTU | DIS | M7)) /*SAFE_MODE*/\
	MUX_VAL(CP(GPMC_WAIT3),		(IEN  | PTU | DIS | M7)) /*SAFE_MODE*/\
 /*Battery Voltage ADC conversion start */\
	MUX_VAL(CP(DSS_PCLK),		(IDIS | PTD | DIS | M4)) /*GPIO_66*/\
								 /* - VBAT_ADC_START*/\
	MUX_VAL(CP(DSS_HSYNC),		(IEN  | PTD | DIS | M7)) /*SAFE_MODE*/\
	MUX_VAL(CP(DSS_VSYNC),		(IEN  | PTD | DIS | M7)) /*SAFE_MODE*/\
	MUX_VAL(CP(DSS_ACBIAS),		(IEN  | PTD | DIS | M7)) /*SAFE_MODE*/\
	MUX_VAL(CP(DSS_DATA0),		(IEN  | PTD | DIS | M7)) /*SAFE_MODE*/\
	MUX_VAL(CP(DSS_DATA1),		(IEN  | PTD | DIS | M7)) /*SAFE_MODE*/\
	MUX_VAL(CP(DSS_DATA2),		(IEN  | PTD | DIS | M7)) /*SAFE_MODE*/\
	MUX_VAL(CP(DSS_DATA3),		(IEN  | PTD | DIS | M7)) /*SAFE_MODE*/\
	MUX_VAL(CP(DSS_DATA4),		(IEN  | PTD | DIS | M7)) /*SAFE_MODE*/\
/*eMMC Reset */\
	MUX_VAL(CP(DSS_DATA5),		(IDIS | PTD | DIS | M4)) /*GPIO_75*/\
								 /* - GPIO_EMMC_RESET*/\
	MUX_VAL(CP(DSS_DATA6),		(IEN  | PTD | DIS | M7)) /*SAFE_MODE*/\
	MUX_VAL(CP(DSS_DATA7),		(IEN  | PTD | DIS | M7)) /*SAFE_MODE*/\
	MUX_VAL(CP(DSS_DATA8),		(IEN  | PTD | DIS | M7)) /*SAFE_MODE*/\
	MUX_VAL(CP(DSS_DATA9),		(IEN  | PTD | DIS | M7)) /*SAFE_MODE*/\
	MUX_VAL(CP(DSS_DATA10),		(IEN  | PTD | DIS | M7)) /*SAFE_MODE*/\
	MUX_VAL(CP(DSS_DATA11),		(IEN  | PTD | DIS | M7)) /*SAFE_MODE*/\
	MUX_VAL(CP(DSS_DATA12),		(IEN  | PTD | DIS | M7)) /*SAFE_MODE*/\
	MUX_VAL(CP(DSS_DATA13),		(IEN  | PTD | DIS | M7)) /*SAFE_MODE*/\
	MUX_VAL(CP(DSS_DATA14),		(IEN  | PTD | DIS | M7)) /*SAFE_MODE*/\
	MUX_VAL(CP(DSS_DATA15),		(IEN  | PTD | DIS | M7)) /*SAFE_MODE*/\
	MUX_VAL(CP(DSS_DATA16),		(IEN  | PTD | DIS | M7)) /*SAFE_MODE*/\
	MUX_VAL(CP(DSS_DATA17),		(IEN  | PTD | DIS | M7)) /*SAFE_MODE*/\
	MUX_VAL(CP(DSS_DATA18),		(IEN  | PTD | DIS | M7)) /*SAFE_MODE*/\
	MUX_VAL(CP(DSS_DATA19),		(IEN  | PTD | DIS | M7)) /*SAFE_MODE*/\
	MUX_VAL(CP(DSS_DATA20),		(IEN  | PTD | DIS | M7)) /*SAFE_MODE*/\
	MUX_VAL(CP(DSS_DATA21),		(IEN  | PTD | DIS | M7)) /*SAFE_MODE*/\
	MUX_VAL(CP(DSS_DATA22),		(IEN  | PTD | DIS | M7)) /*SAFE_MODE*/\
	MUX_VAL(CP(DSS_DATA23),		(IEN  | PTD | DIS | M7)) /*SAFE_MODE*/\
 	MUX_VAL(CP(CAM_HS),		(IEN  | PTU | DIS | M7)) /*SAFE_MODE*/\
	MUX_VAL(CP(CAM_VS),		(IEN  | PTU | DIS | M7)) /*SAFE_MODE*/\
	MUX_VAL(CP(CAM_XCLKA),		(IEN  | PTU | DIS | M7)) /*SAFE_MODE*/\
	MUX_VAL(CP(CAM_PCLK),		(IEN  | PTU | DIS | M7)) /*SAFE_MODE*/\
	MUX_VAL(CP(CAM_FLD),		(IEN  | PTU | DIS | M7)) /*SAFE_MODE*/\
	MUX_VAL(CP(CAM_D0),		(IEN  | PTD | DIS | M7)) /*SAFE_MODE*/\
	MUX_VAL(CP(CAM_D1),		(IEN  | PTD | DIS | M7)) /*SAFE_MODE*/\
	MUX_VAL(CP(CAM_D2),		(IEN  | PTD | DIS | M7)) /*SAFE_MODE*/\
	MUX_VAL(CP(CAM_D3),		(IEN  | PTD | DIS | M7)) /*SAFE_MODE*/\
	MUX_VAL(CP(CAM_D4),		(IEN  | PTD | DIS | M7)) /*SAFE_MODE*/\
	MUX_VAL(CP(CAM_D5),		(IEN  | PTD | DIS | M7)) /*SAFE_MODE*/\
	MUX_VAL(CP(CAM_D6),		(IEN  | PTD | DIS | M7)) /*SAFE_MODE*/\
	MUX_VAL(CP(CAM_D7),		(IEN  | PTD | DIS | M7)) /*SAFE_MODE*/\
	MUX_VAL(CP(CAM_D8),		(IEN  | PTD | DIS | M7)) /*SAFE_MODE*/\
	MUX_VAL(CP(CAM_D9),		(IEN  | PTD | DIS | M7)) /*SAFE_MODE*/\
	MUX_VAL(CP(CAM_D10),		(IEN  | PTD | DIS | M7)) /*SAFE_MODE*/\
	MUX_VAL(CP(CAM_D11),		(IEN  | PTD | DIS | M7)) /*SAFE_MODE*/\
	MUX_VAL(CP(CAM_XCLKB),		(IDIS | PTD | DIS | M7)) /*SAFE_MODE*/\
	MUX_VAL(CP(CAM_WEN),		(IEN  | PTD | DIS | M7)) /*SAFE_MODE*/\
	MUX_VAL(CP(CAM_STROBE),		(IDIS | PTD | DIS | M7)) /*SAFE_MODE*/\
	MUX_VAL(CP(CSI2_DX0),		(IEN  | PTD | EN  | M7)) /*GPIO_112*/\
/*PoP LPDDR + NAND Interrupts and Thermistor */\
								 /* - POP_TEMP*/\
	MUX_VAL(CP(CSI2_DY0),		(IEN  | PTD | EN  | M4)) /*GPIO_113*/\
								 /* - POP_INT1*/\
	MUX_VAL(CP(CSI2_DX1),		(IEN  | PTD | EN  | M4)) /*GPIO_114*/\
								 /* - POP_INT0*/\
	MUX_VAL(CP(CSI2_DY1),		(IEN  | PTD | EN  | M4)) /*GPIO_115*/\
 /*I2S Audio Interface for speaker */\
	MUX_VAL(CP(MCBSP2_FSX),		(IEN  | PTD | DIS | M0)) /*McBSP2_FSX*/\
	MUX_VAL(CP(MCBSP2_CLKX),	(IEN  | PTD | DIS | M0)) /*McBSP2_CLKX*/\
	MUX_VAL(CP(MCBSP2_DR),		(IEN  | PTD | DIS | M0)) /*McBSP2_DR*/\
	MUX_VAL(CP(MCBSP2_DX),		(IDIS | PTD | DIS | M0)) /*McBSP2_DX*/\
 /*MicroSD card MMC1_DAT[3:0] (disabled for mainstream product) */\
	MUX_VAL(CP(MMC1_CLK),		(IDIS | PTU | EN  | M0)) /*MMC1_CLK*/\
	MUX_VAL(CP(MMC1_CMD),		(IDIS | PTU | EN  | M0)) /*MMC1_CMD*/\
	MUX_VAL(CP(MMC1_DAT0),		(IEN  | PTU | EN  | M0)) /*MMC1_DAT0*/\
	MUX_VAL(CP(MMC1_DAT1),		(IEN  | PTU | EN  | M0)) /*MMC1_DAT1*/\
	MUX_VAL(CP(MMC1_DAT2),		(IEN  | PTU | EN  | M0)) /*MMC1_DAT2*/\
	MUX_VAL(CP(MMC1_DAT3),		(IEN  | PTU | EN  | M0)) /*MMC1_DAT3*/\
	MUX_VAL(CP(MMC1_DAT4),		(IEN  | PTD | EN  | M7)) /*SAFE_MODE*/\
	MUX_VAL(CP(MMC1_DAT5),		(IEN  | PTD | EN  | M7)) /*SAFE_MODE*/\
	MUX_VAL(CP(MMC1_DAT6),		(IEN  | PTD | EN  | M7)) /*SAFE_MODE*/\
	MUX_VAL(CP(MMC1_DAT7),		(IEN  | PTD | EN  | M7)) /*SAFE_MODE*/\
 /*eMMC Managed NAND MMC2_DAT[7:0] */\
	MUX_VAL(CP(MMC2_CLK),		(IEN  | PTU | EN  | M0)) /*MMC2_CLK*/\
	MUX_VAL(CP(MMC2_CMD),		(IEN  | PTU | EN  | M0)) /*MMC2_CMD*/\
	MUX_VAL(CP(MMC2_DAT0),		(IEN  | PTU | DIS | M0)) /*MMC2_DAT0*/\
	MUX_VAL(CP(MMC2_DAT1),		(IEN  | PTU | DIS | M0)) /*MMC2_DAT1*/\
	MUX_VAL(CP(MMC2_DAT2),		(IEN  | PTU | DIS | M0)) /*MMC2_DAT2*/\
	MUX_VAL(CP(MMC2_DAT3),		(IEN  | PTU | DIS | M0)) /*MMC2_DAT3*/\
	MUX_VAL(CP(MMC2_DAT4),		(IEN  | PTU | DIS | M0)) /*MMC2_DAT4*/\
	MUX_VAL(CP(MMC2_DAT5),		(IEN  | PTU | DIS | M0)) /*MMC2_DAT5*/\
	MUX_VAL(CP(MMC2_DAT6),		(IEN  | PTU | DIS | M0)) /*MMC2_DAT6*/\
	MUX_VAL(CP(MMC2_DAT7),		(IEN  | PTU | DIS | M0)) /*MMC2_DAT7*/\
 	MUX_VAL(CP(MCBSP3_DX),		(IEN  | PTD | DIS | M7)) /*SAFE_MODE*/\
	MUX_VAL(CP(MCBSP3_DR),		(IEN  | PTD | DIS | M7)) /*SAFE_MODE*/\
	MUX_VAL(CP(MCBSP3_CLKX),	(IEN  | PTD | DIS | M7)) /*SAFE_MODE*/\
	MUX_VAL(CP(MCBSP3_FSX),		(IEN  | PTD | DIS | M7)) /*SAFE_MODE*/\
	MUX_VAL(CP(UART2_CTS),		(IEN  | PTD | DIS | M7)) /*SAFE_MODE*/\
	MUX_VAL(CP(UART2_RTS),		(IEN  | PTD | DIS | M7)) /*SAFE_MODE*/\
	MUX_VAL(CP(UART2_TX),		(IEN  | PTD | DIS | M7)) /*SAFE_MODE*/\
	MUX_VAL(CP(UART2_RX),		(IEN  | PTD | DIS | M7)) /*SAFE_MODE*/\
	MUX_VAL(CP(UART1_TX),		(IDIS | PTD | DIS | M0)) /*UART1_TX*/\
	MUX_VAL(CP(UART1_RTS),		(IDIS | PTU | DIS | M0)) /*UART1_RTS*/ \
	MUX_VAL(CP(UART1_CTS),		(IEN  | PTD | EN  | M0)) /*UART1_CTS*/\
	MUX_VAL(CP(UART1_RX),		(IEN  | PTD | DIS | M0)) /*UART1_RX*/\
/*Battery charger interrupt */\
	MUX_VAL(CP(MCBSP4_CLKX),	(IEN  | PTU | EN  | M4)) /*GPIO_152*/\
								 /* - CHARGER_INT#*/\
/*Host-to-WLAN wakeup */\
	MUX_VAL(CP(MCBSP4_DR),		(IEN  | PTD | DIS | M4)) /*GPIO_153*/\
								 /* - WLAN_WKUP#*/\
	MUX_VAL(CP(MCBSP4_DX),		(IEN  | PTD | DIS | M7)) /*SAFE_MODE*/\
/*Host-to-WLAN power down */\
	MUX_VAL(CP(MCBSP4_FSX),		(IEN  | PTD | DIS | M4)) /*GPIO_155*/\
								 /* - WLAN_PDN#*/\
	MUX_VAL(CP(MCBSP1_CLKR),	(IEN  | PTD | DIS | M7)) /*SAFE_MODE*/\
	MUX_VAL(CP(MCBSP1_FSR),		(IEN  | PTD | DIS | M7)) /*SAFE_MODE*/\
	MUX_VAL(CP(MCBSP1_DX),		(IEN  | PTD | DIS | M7)) /*SAFE_MODE*/\
	MUX_VAL(CP(MCBSP1_DR),		(IEN  | PTD | DIS | M7)) /*SAFE_MODE*/\
 /*Global audio clock */\
	MUX_VAL(CP(MCBSP_CLKS),		(IEN  | PTU | DIS | M0)) /*McBSP_CLKS*/\
	MUX_VAL(CP(MCBSP1_FSX),		(IEN  | PTD | DIS | M7)) /*SAFE_MODE*/\
	MUX_VAL(CP(MCBSP1_CLKX),	(IEN  | PTD | DIS | M7)) /*SAFE_MODE*/\
 /*Serial Interface */\
	MUX_VAL(CP(UART3_CTS_RCTX),	(IEN  | PTD | EN  | M0)) /*UART3_CTS_RCTX*/\
	MUX_VAL(CP(UART3_RTS_SD),	(IDIS | PTU | DIS | M7)) /*SAFE_MODE*/\
	MUX_VAL(CP(UART3_RX_IRRX),	(IEN  | PTU | DIS | M0)) /*UART3_RX_IRRX*/\
	MUX_VAL(CP(UART3_TX_IRTX),	(IDIS | PTD | DIS | M0)) /*UART3_TX_IRTX*/\
/*USB ULPI Interface */\
	MUX_VAL(CP(HSUSB0_CLK),		(IEN  | PTD | DIS | M0)) /*HSUSB0_CLK*/\
	MUX_VAL(CP(HSUSB0_STP),		(IDIS | PTU | EN  | M0)) /*HSUSB0_STP*/\
	MUX_VAL(CP(HSUSB0_DIR),		(IEN  | PTD | DIS | M0)) /*HSUSB0_DIR*/\
	MUX_VAL(CP(HSUSB0_NXT),		(IEN  | PTD | DIS | M0)) /*HSUSB0_NXT*/\
	MUX_VAL(CP(HSUSB0_DATA0),	(IEN  | PTD | DIS | M0)) /*HSUSB0_DATA0*/\
	MUX_VAL(CP(HSUSB0_DATA1),	(IEN  | PTD | DIS | M0)) /*HSUSB0_DATA1*/\
	MUX_VAL(CP(HSUSB0_DATA2),	(IEN  | PTD | DIS | M0)) /*HSUSB0_DATA2*/\
	MUX_VAL(CP(HSUSB0_DATA3),	(IEN  | PTD | DIS | M0)) /*HSUSB0_DATA3*/\
	MUX_VAL(CP(HSUSB0_DATA4),	(IEN  | PTD | DIS | M0)) /*HSUSB0_DATA4*/\
	MUX_VAL(CP(HSUSB0_DATA5),	(IEN  | PTD | DIS | M0)) /*HSUSB0_DATA5*/\
	MUX_VAL(CP(HSUSB0_DATA6),	(IEN  | PTD | DIS | M0)) /*HSUSB0_DATA6*/\
	MUX_VAL(CP(HSUSB0_DATA7),	(IEN  | PTD | DIS | M0)) /*HSUSB0_DATA7*/\
	MUX_VAL(CP(I2C1_SCL),		(IEN  | PTU | EN  | M0)) /*I2C1_SCL*/\
	MUX_VAL(CP(I2C1_SDA),		(IEN  | PTU | EN  | M0)) /*I2C1_SDA*/\
	MUX_VAL(CP(I2C2_SCL),		(IEN  | PTU | DIS | M0)) /*I2C2_SCL*/\
	MUX_VAL(CP(I2C2_SDA),		(IEN  | PTU | DIS | M0)) /*I2C2_SDA*/\
	MUX_VAL(CP(I2C3_SCL),		(IEN  | PTU | EN  | M7)) /*SAFE_MODE*/\
	MUX_VAL(CP(I2C3_SDA),		(IEN  | PTU | EN  | M7)) /*SAFE_MODE*/\
	MUX_VAL(CP(I2C4_SCL),		(IEN  | PTU | EN  | M0)) /*I2C4_SCL*/\
	MUX_VAL(CP(I2C4_SDA),		(IEN  | PTU | EN  | M0)) /*I2C4_SDA*/\
	MUX_VAL(CP(HDQ_SIO),		(IDIS | PTU | EN  | M7)) /*SAFE_MODE*/\
	MUX_VAL(CP(MCSPI1_CLK),		(IEN  | PTD | DIS | M7)) /*SAFE_MODE*/\
	MUX_VAL(CP(MCSPI1_SIMO),	(IEN  | PTD | DIS | M7)) /*SAFE_MODE*/\
	MUX_VAL(CP(MCSPI1_SOMI),	(IEN  | PTD | DIS | M7)) /*SAFE_MODE*/\
	MUX_VAL(CP(MCSPI1_CS0),		(IEN  | PTD | EN  | M7)) /*SAFE_MODE*/\
 /*Boot LED */\
	MUX_VAL(CP(MCSPI1_CS1),		(IDIS | PTU | DIS | M4)) /*GPIO_175*/\
								 /* - BOOT_LED */\
 /*WLAN EEPROM write protect */\
	MUX_VAL(CP(MCSPI1_CS2),		(IEN  | PTU | DIS | M4)) /*GPIO_176 */\
								 /* - WLAN_EEPROM_WP */\
 /*WLAN GPIO0 (SLEEPn?) */\
	MUX_VAL(CP(MCSPI1_CS3),		(IEN  | PTD | DIS | M4)) /*GPIO_177*/\
								 /* - WLAN_SLEEP# */\
	MUX_VAL(CP(MCSPI2_CLK),		(IEN  | PTD | DIS | M7)) /*SAFE_MODE*/\
	MUX_VAL(CP(MCSPI2_SIMO),	(IEN  | PTD | DIS | M7)) /*SAFE_MODE*/\
	MUX_VAL(CP(MCSPI2_SOMI),	(IEN  | PTD | DIS | M7)) /*SAFE_MODE*/\
	MUX_VAL(CP(MCSPI2_CS0),		(IEN  | PTD | DIS | M7)) /*SAFE_MODE*/\
	MUX_VAL(CP(MCSPI2_CS1),		(IEN  | PTD | DIS | M7)) /*SAFE_MODE*/\
 /*Control and debug */\
	MUX_VAL(CP(SYS_32K),		(IEN  | PTD | DIS | M0)) /*SYS_32K*/\
	MUX_VAL(CP(SYS_CLKREQ),		(IEN  | PTD | DIS | M0)) /*SYS_CLKREQ*/\
	MUX_VAL(CP(SYS_NIRQ),		(IEN  | PTU | EN  | M0)) /*SYS_nIRQ*/\
	MUX_VAL(CP(SYS_BOOT0),		(IEN  | PTD | DIS | M4)) /*GPIO_2*/\
	MUX_VAL(CP(SYS_BOOT1),		(IEN  | PTD | DIS | M4)) /*GPIO_3*/\
	MUX_VAL(CP(SYS_BOOT2),		(IEN  | PTD | DIS | M4)) /*GPIO_4*/\
	MUX_VAL(CP(SYS_BOOT3),		(IEN  | PTD | DIS | M4)) /*GPIO_5*/\
	MUX_VAL(CP(SYS_BOOT4),		(IEN  | PTD | DIS | M4)) /*GPIO_6*/\
	MUX_VAL(CP(SYS_BOOT5),		(IEN  | PTD | DIS | M4)) /*GPIO_7*/\
	MUX_VAL(CP(SYS_BOOT6),		(IEN  | PTD | DIS | M4)) /*GPIO_8*/\
	MUX_VAL(CP(SYS_OFF_MODE),	(IEN  | PTD | DIS | M0)) /*SYS_OFF_MODE*/\
	MUX_VAL(CP(SYS_CLKOUT1),	(IEN  | PTU | EN  | M4)) /*GPIO_10*/\
	MUX_VAL(CP(SYS_CLKOUT2),	(IEN  | PTU | DIS | M7)) /*SAFE_MODE*/\
/*WLAN MMC3_DAT[3:0] */\
	MUX_VAL(CP(ETK_CLK_ES2),	(IEN  | PTU | EN  | M2)) /*MMC3_CLK*/\
	MUX_VAL(CP(ETK_CTL_ES2),	(IEN  | PTU | EN  | M2)) /*MMC3_CMD*/\
	MUX_VAL(CP(ETK_D0_ES2),		(IEN  | PTD | EN  | M4)) /*GPIO_14*/\
	MUX_VAL(CP(ETK_D1_ES2),		(IEN  | PTD | EN  | M4)) /*GPIO_15*/\
	MUX_VAL(CP(ETK_D2_ES2),		(IEN  | PTD | EN  | M4)) /*GPIO_16*/\
	MUX_VAL(CP(ETK_D3_ES2),		(IEN  | PTU | EN  | M2)) /*MMC3_DAT3*/\
	MUX_VAL(CP(ETK_D4_ES2),		(IEN  | PTU | EN  | M2)) /*MMC3_DAT0*/\
	MUX_VAL(CP(ETK_D5_ES2),		(IEN  | PTU | EN  | M2)) /*MMC3_DAT1*/\
	MUX_VAL(CP(ETK_D6_ES2),		(IEN  | PTU | EN  | M2)) /*MMC3_DAT2*/\
	MUX_VAL(CP(ETK_D7_ES2),		(IEN  | PTD | EN  | M4)) /*GPIO_21*/\
	MUX_VAL(CP(ETK_D8_ES2),		(IEN  | PTD | EN  | M4)) /*GPIO_22*/\
	MUX_VAL(CP(ETK_D9_ES2),		(IEN  | PTD | EN  | M4)) /*GPIO_23*/\
	MUX_VAL(CP(ETK_D10_ES2),	(IEN  | PTD | EN  | M4)) /*GPIO_24*/\
	MUX_VAL(CP(ETK_D11_ES2),	(IEN  | PTD | EN  | M4)) /*GPIO_25*/\
	MUX_VAL(CP(ETK_D12_ES2),	(IEN  | PTD | EN  | M4)) /*GPIO_26*/\
	MUX_VAL(CP(ETK_D13_ES2),	(IEN  | PTD | EN  | M4)) /*GPIO_27*/\
	MUX_VAL(CP(ETK_D14_ES2),	(IEN  | PTD | EN  | M4)) /*GPIO_28*/\
	MUX_VAL(CP(ETK_D15_ES2),	(IEN  | PTD | EN  | M4)) /*GPIO_29*/\
 /* die to die */\
	MUX_VAL(CP(D2D_MCAD1),		(IEN  | PTD | EN  | M0)) /*d2d_mcad1*/\
	MUX_VAL(CP(D2D_MCAD2),		(IEN  | PTD | EN  | M0)) /*d2d_mcad2*/\
	MUX_VAL(CP(D2D_MCAD3),		(IEN  | PTD | EN  | M0)) /*d2d_mcad3*/\
	MUX_VAL(CP(D2D_MCAD4),		(IEN  | PTD | EN  | M0)) /*d2d_mcad4*/\
	MUX_VAL(CP(D2D_MCAD5),		(IEN  | PTD | EN  | M0)) /*d2d_mcad5*/\
	MUX_VAL(CP(D2D_MCAD6),		(IEN  | PTD | EN  | M0)) /*d2d_mcad6*/\
	MUX_VAL(CP(D2D_MCAD7),		(IEN  | PTD | EN  | M0)) /*d2d_mcad7*/\
	MUX_VAL(CP(D2D_MCAD8),		(IEN  | PTD | EN  | M0)) /*d2d_mcad8*/\
	MUX_VAL(CP(D2D_MCAD9),		(IEN  | PTD | EN  | M0)) /*d2d_mcad9*/\
	MUX_VAL(CP(D2D_MCAD10),		(IEN  | PTD | EN  | M0)) /*d2d_mcad10*/\
	MUX_VAL(CP(D2D_MCAD11),		(IEN  | PTD | EN  | M0)) /*d2d_mcad11*/\
	MUX_VAL(CP(D2D_MCAD12),		(IEN  | PTD | EN  | M0)) /*d2d_mcad12*/\
	MUX_VAL(CP(D2D_MCAD13),		(IEN  | PTD | EN  | M0)) /*d2d_mcad13*/\
	MUX_VAL(CP(D2D_MCAD14),		(IEN  | PTD | EN  | M0)) /*d2d_mcad14*/\
	MUX_VAL(CP(D2D_MCAD15),		(IEN  | PTD | EN  | M0)) /*d2d_mcad15*/\
	MUX_VAL(CP(D2D_MCAD16),		(IEN  | PTD | EN  | M0)) /*d2d_mcad16*/\
	MUX_VAL(CP(D2D_MCAD17),		(IEN  | PTD | EN  | M0)) /*d2d_mcad17*/\
	MUX_VAL(CP(D2D_MCAD18),		(IEN  | PTD | EN  | M0)) /*d2d_mcad18*/\
	MUX_VAL(CP(D2D_MCAD19),		(IEN  | PTD | EN  | M0)) /*d2d_mcad19*/\
	MUX_VAL(CP(D2D_MCAD20),		(IEN  | PTD | EN  | M0)) /*d2d_mcad20*/\
	MUX_VAL(CP(D2D_MCAD21),		(IEN  | PTD | EN  | M0)) /*d2d_mcad21*/\
	MUX_VAL(CP(D2D_MCAD22),		(IEN  | PTD | EN  | M0)) /*d2d_mcad22*/\
	MUX_VAL(CP(D2D_MCAD23),		(IEN  | PTD | EN  | M0)) /*d2d_mcad23*/\
	MUX_VAL(CP(D2D_MCAD24),		(IEN  | PTD | EN  | M0)) /*d2d_mcad24*/\
	MUX_VAL(CP(D2D_MCAD25),		(IEN  | PTD | EN  | M0)) /*d2d_mcad25*/\
	MUX_VAL(CP(D2D_MCAD26),		(IEN  | PTD | EN  | M0)) /*d2d_mcad26*/\
	MUX_VAL(CP(D2D_MCAD27),		(IEN  | PTD | EN  | M0)) /*d2d_mcad27*/\
	MUX_VAL(CP(D2D_MCAD28),		(IEN  | PTD | EN  | M0)) /*d2d_mcad28*/\
	MUX_VAL(CP(D2D_MCAD29),		(IEN  | PTD | EN  | M0)) /*d2d_mcad29*/\
	MUX_VAL(CP(D2D_MCAD30),		(IEN  | PTD | EN  | M0)) /*d2d_mcad30*/\
	MUX_VAL(CP(D2D_MCAD31),		(IEN  | PTD | EN  | M0)) /*d2d_mcad31*/\
	MUX_VAL(CP(D2D_MCAD32),		(IEN  | PTD | EN  | M0)) /*d2d_mcad32*/\
	MUX_VAL(CP(D2D_MCAD33),		(IEN  | PTD | EN  | M0)) /*d2d_mcad33*/\
	MUX_VAL(CP(D2D_MCAD34),		(IEN  | PTD | EN  | M0)) /*d2d_mcad34*/\
	MUX_VAL(CP(D2D_MCAD35),		(IEN  | PTD | EN  | M0)) /*d2d_mcad35*/\
	MUX_VAL(CP(D2D_MCAD36),		(IEN  | PTD | EN  | M0)) /*d2d_mcad36*/\
	MUX_VAL(CP(D2D_CLK26MI),	(IEN  | PTD | DIS | M0)) /*d2d_clk26mi*/\
	MUX_VAL(CP(D2D_NRESPWRON),	(IEN  | PTD | EN  | M0)) /*d2d_nrespwron*/\
	MUX_VAL(CP(D2D_NRESWARM),	(IEN  | PTU | EN  | M0)) /*d2d_nreswarm */\
	MUX_VAL(CP(D2D_ARM9NIRQ),	(IEN  | PTD | DIS | M0)) /*d2d_arm9nirq */\
	MUX_VAL(CP(D2D_UMA2P6FIQ),	(IEN  | PTD | DIS | M0)) /*d2d_uma2p6fiq*/\
	MUX_VAL(CP(D2D_SPINT),		(IEN  | PTD | EN  | M0)) /*d2d_spint*/\
	MUX_VAL(CP(D2D_FRINT),		(IEN  | PTD | EN  | M0)) /*d2d_frint*/\
	MUX_VAL(CP(D2D_DMAREQ0),	(IEN  | PTD | DIS | M0)) /*d2d_dmareq0*/\
	MUX_VAL(CP(D2D_DMAREQ1),	(IEN  | PTD | DIS | M0)) /*d2d_dmareq1*/\
	MUX_VAL(CP(D2D_DMAREQ2),	(IEN  | PTD | DIS | M0)) /*d2d_dmareq2*/\
	MUX_VAL(CP(D2D_DMAREQ3),	(IEN  | PTD | DIS | M0)) /*d2d_dmareq3*/\
	MUX_VAL(CP(D2D_N3GTRST),	(IEN  | PTD | DIS | M0)) /*d2d_n3gtrst*/\
	MUX_VAL(CP(D2D_N3GTDI),		(IEN  | PTD | DIS | M0)) /*d2d_n3gtdi*/\
	MUX_VAL(CP(D2D_N3GTDO),		(IEN  | PTD | DIS | M0)) /*d2d_n3gtdo*/\
	MUX_VAL(CP(D2D_N3GTMS),		(IEN  | PTD | DIS | M0)) /*d2d_n3gtms*/\
	MUX_VAL(CP(D2D_N3GTCK),		(IEN  | PTD | DIS | M0)) /*d2d_n3gtck*/\
	MUX_VAL(CP(D2D_N3GRTCK),	(IEN  | PTD | DIS | M0)) /*d2d_n3grtck*/\
	MUX_VAL(CP(D2D_MSTDBY),		(IEN  | PTU | EN  | M0)) /*d2d_mstdby*/\
	MUX_VAL(CP(D2D_SWAKEUP),	(IEN  | PTD | EN  | M0)) /*d2d_swakeup*/\
	MUX_VAL(CP(D2D_IDLEREQ),	(IEN  | PTD | DIS | M0)) /*d2d_idlereq*/\
	MUX_VAL(CP(D2D_IDLEACK),	(IEN  | PTU | EN  | M0)) /*d2d_idleack*/\
	MUX_VAL(CP(D2D_MWRITE),		(IEN  | PTD | DIS | M0)) /*d2d_mwrite*/\
	MUX_VAL(CP(D2D_SWRITE),		(IEN  | PTD | DIS | M0)) /*d2d_swrite*/\
	MUX_VAL(CP(D2D_MREAD),		(IEN  | PTD | DIS | M0)) /*d2d_mread*/\
	MUX_VAL(CP(D2D_SREAD),		(IEN  | PTD | DIS | M0)) /*d2d_sread*/\
	MUX_VAL(CP(D2D_MBUSFLAG),	(IEN  | PTD | DIS | M0)) /*d2d_mbusflag*/\
	MUX_VAL(CP(D2D_SBUSFLAG),	(IEN  | PTD | DIS | M0)) /*d2d_sbusflag*/

#endif
