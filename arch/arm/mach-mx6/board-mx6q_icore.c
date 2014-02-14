/*
 * Copyright (C) 2011-2012 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <linux/types.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/nodemask.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/fsl_devices.h>
#include <linux/spi/spi.h>
#include <linux/spi/flash.h>
#include <linux/i2c.h>
#include <linux/i2c/pca953x.h>
#include <linux/ata.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/partitions.h>
#include <linux/regulator/consumer.h>
#include <linux/pmic_external.h>
#include <linux/pmic_status.h>
#include <linux/ipu.h>
#include <linux/mxcfb.h>
#include <linux/pwm_backlight.h>
#include <linux/fec.h>
#include <linux/memblock.h>
#include <linux/gpio.h>
#include <linux/etherdevice.h>
#include <linux/regulator/anatop-regulator.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>

#include <mach/common.h>
#include <mach/hardware.h>
#include <mach/mxc_dvfs.h>
#include <mach/memory.h>
#include <mach/iomux-mx6q.h>
#include <mach/iomux-mx6dl.h>
#include <mach/imx-uart.h>
#include <mach/viv_gpu.h>
#include <mach/ahci_sata.h>
#include <mach/ipu-v3.h>
#include <mach/mxc_hdmi.h>
#include <mach/mxc_asrc.h>

#include <asm/irq.h>
#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>

#include "usb.h"
#include "devices-imx6q.h"
#include "crm_regs.h"
#include "cpu_op-mx6.h"

#include <linux/mfd/mxc-hdmi-core.h>
#include <linux/input/edt-ft5x06.h>
#include <linux/input/ili210x.h>

#define ICORE_M6_SD1_CD		IMX_GPIO_NR(1, 1)
#define ICORE_M6_SD1_WP		IMX_GPIO_NR(2, 18)
#define MAX11801_TS_IRQ         IMX_GPIO_NR(3, 31)
#define FT5X06_TS_IRQ           IMX_GPIO_NR(1, 7)
#define OFC_FT5X06_TS_IRQ       IMX_GPIO_NR(5, 30)
#define OFC_URT_TS_IRQ       	IMX_GPIO_NR(5, 30)
#define OFC_LVDS_ENABLE		IMX_GPIO_NR(1, 7)
#define STARTERKIT_CAPEDT_IRQ	IMX_GPIO_NR(1, 7)
#define UART3_CS485		IMX_GPIO_NR(3, 23)

#define ICORE_M6_SD3_CD		IMX_GPIO_NR(7, 0)
#define ICORE_M6_SD3_WP		IMX_GPIO_NR(7, 1)
#define ICORE_M6_SD4_CD		IMX_GPIO_NR(2, 6)
#define ICORE_M6_SD4_WP		IMX_GPIO_NR(2, 7)
#define ICORE_M6_ECSPI1_CS1	IMX_GPIO_NR(3, 19)
#define ICORE_M6_USB_OTG_PWR	IMX_GPIO_NR(3, 22)
#define ICORE_M6_CAP_TCH_INT1	IMX_GPIO_NR(1, 9)
#define ICORE_M6_USB_HUB_RESET	IMX_GPIO_NR(7, 12)
#define ICORE_M6_CAN1_STBY	IMX_GPIO_NR(1, 2)
#define ICORE_M6_CAN1_EN	IMX_GPIO_NR(1, 4)
#define ICORE_M6_MENU_KEY	IMX_GPIO_NR(2, 1)
#define ICORE_M6_BACK_KEY	IMX_GPIO_NR(2, 2)
#define ICORE_M6_ONOFF_KEY	IMX_GPIO_NR(2, 3)
#define ICORE_M6_HOME_KEY	IMX_GPIO_NR(2, 4)
#define ICORE_M6_VOL_UP_KEY	IMX_GPIO_NR(7, 13)
#define ICORE_M6_VOL_DOWN_KEY	IMX_GPIO_NR(4, 5)
#define ICORE_M6_CSI0_RST	IMX_GPIO_NR(1, 8)
#define ICORE_M6_CSI0_PWN	IMX_GPIO_NR(1, 6)
#define ICORE_M6_WF111_RESET_SK	IMX_GPIO_NR(1, 2)
#define ICORE_M6_WF111_RESET_MP	IMX_GPIO_NR(1, 7)
#define ICORE_M6_WF111_RESET_URT	IMX_GPIO_NR(6, 2)
#define ICORE_M6_OF_LVDS_RESET	IMX_GPIO_NR(6, 0)
#define ICORE_MODULE_VERSION    IMX_GPIO_NR(7, 2)
#define ICORE_BACKLIGHT3 	IMX_GPIO_NR(2, 9)

#define ICORE_M6_SD3_WP_PADCFG	(PAD_CTL_PKE | PAD_CTL_PUE |	\
		PAD_CTL_PUS_22K_UP | PAD_CTL_SPEED_MED |	\
		PAD_CTL_DSE_40ohm | PAD_CTL_HYS)

enum engicam_board
{
        ENGICAM_RESISTIVE_SK=0,
        ENGICAM_RESISTIVE_OF,
        ENGICAM_CAPACITIVE_OF,
	ENGICAM_CAPACITIVE_OF_AMP,
        ENGICAM_CAPACITIVE_SK,
	ENGICAM_MEDIAPLAYER_V0,
	ENGICAM_CAPACITIVE_OF_URT,

        ENGICAM_LAST_BOARD
};

#define ENGICAM_DEFAULT_BOARD 		ENGICAM_RESISTIVE_SK

static char* engi_board_str[] =
{
        "SK.RES",
        "OF.RES",
        "OF.CAP",
	"OF.AMP",
        "SK.CAP",
	"MP.000",
	"UF.CAP",	/* PCAP URT */

        /* add here a new board */
};

static char* engi_board_description_str[] =
{
        "Engicam resistive Starterkit",
        "Engicam resistive Openframe",
        "Engicam capacitive Openframe EDT",
	"Engicam capacitive Openframe Ampire",
        "Engicam capacitive Starterkit",
	"Engicam Media Player Ver. 1.00",
        "Engicam capacitive Openframe URT",

        /* add here a new description board */
};

static unsigned int engi_board = ENGICAM_DEFAULT_BOARD;

/*
 * Detect from the bootargs witch engicam custom board is if setted
 */
static int engi_board_setup(char *str)
{
        engi_board = 0;

        while( engi_board<ENGICAM_LAST_BOARD && strcmp(str, engi_board_str[engi_board] ) )
        {
                engi_board++;
        }

        // If not match RESISTIVE_SK will be the default value
        if(engi_board>=ENGICAM_LAST_BOARD)
                engi_board=ENGICAM_DEFAULT_BOARD;

        return 0;
}

__setup("engi_board=", engi_board_setup);

enum icore_modul_vers
{
        ICORE_VERS_MINOR_D=0,
        ICORE_VERS_GREATER_D,

        ICORE_VERS_LAST
};

static char* icore_module_vers_str[] =
{
        "< RevD",
        ">= RevD",
};

static unsigned int icore_mudule_vers = ICORE_VERS_LAST;

/*!
 * Get the module revision reading GPIO
 */
void icore_init_module_version (void)
{
	if(cpu_is_mx6q())
		mxc_iomux_v3_setup_pad(MX6Q_PAD_SD3_CMD__GPIO_7_2);
	else
		mxc_iomux_v3_setup_pad(MX6DL_PAD_SD3_CMD__GPIO_7_2);

	gpio_request(ICORE_MODULE_VERSION, "mod-ver");

	gpio_direction_input(ICORE_MODULE_VERSION);
	
	if(gpio_get_value(ICORE_MODULE_VERSION) == 0)
		icore_mudule_vers=ICORE_VERS_GREATER_D;
	else
		icore_mudule_vers=ICORE_VERS_MINOR_D;
	
	gpio_free(ICORE_MODULE_VERSION);
}

/* Conatins the correct value of RAM memory size. The complete value is
*  available  only after calling the fuction mx6q_icore_check_ram_size 
*  prior to reszerve any memory areas.
*/
static int iRamMemorySize=0;

void __init early_console_setup(unsigned long base, struct clk *clk);
static void mx6q_icore_check_ram_size (void);
static struct clk *sata_clk;

static int max11801_mode = 0;

extern char *gp_reg_id;
extern char *soc_reg_id;
extern char *pu_reg_id;

extern struct regulator *(*get_cpu_regulator)(void);
extern void (*put_cpu_regulator)(void);
extern void mx6_cpu_regulator_init(void);

static iomux_v3_cfg_t mx6q_icore_pads[] = {
	/* CAN1  */
	MX6Q_PAD_KEY_ROW2__CAN1_RXCAN,
	MX6Q_PAD_KEY_COL2__CAN1_TXCAN,

	/* CAN2 */
	MX6Q_PAD_KEY_ROW4__CAN2_RXCAN,
	MX6Q_PAD_KEY_COL4__CAN2_TXCAN,

	MX6Q_PAD_GPIO_7__GPIO_1_7,		/* NERR */
	MX6Q_PAD_GPIO_4__GPIO_1_4,		/* Enable */

	/* CCM  */
	MX6Q_PAD_GPIO_0__CCM_CLKO,		/* SGTL500 sys_mclk */
	MX6Q_PAD_GPIO_3__CCM_CLKO2,		/* J5 - Camera MCLK */


	MX6Q_PAD_SD4_DAT1__PWM3_PWMO,		/* PWM per back light	*/

	/* ENET */
	MX6Q_PAD_ENET_CRS_DV__ENET_RX_EN 	,
	MX6Q_PAD_GPIO_16__ENET_ANATOP_ETHERNET_REF_OUT,
	MX6Q_PAD_ENET_TX_EN__ENET_TX_EN		,
	MX6Q_PAD_ENET_RXD1__ENET_RDATA_1	,
	MX6Q_PAD_ENET_RXD0__ENET_RDATA_0	,
	MX6Q_PAD_ENET_TXD1__ENET_TDATA_1	,
	MX6Q_PAD_ENET_TXD0__ENET_TDATA_0	,
	MX6Q_PAD_ENET_MDC__ENET_MDC		,
	MX6Q_PAD_ENET_MDIO__ENET_MDIO		,
	MX6Q_PAD_ENET_REF_CLK__GPIO_1_23	,	/* TBD da capire se puo' uscire 50MHz, altrimenti GPIO input */
	MX6Q_PAD_GPIO_17__GPIO_7_12 		,

	/* GPIO7 */
	MX6Q_PAD_GPIO_17__GPIO_7_12,	/* USB Hub Reset */
	MX6Q_PAD_GPIO_18__GPIO_7_13,	/* J14 - Volume Up */

	/* I2C1, MAX11801 */
	MX6Q_PAD_EIM_D21__I2C1_SCL,	/* GPIO3[21] */
	MX6Q_PAD_EIM_D28__I2C1_SDA,	/* GPIO3[28] */

	/* I2C2 Camera, MIPI */
	MX6Q_PAD_KEY_COL3__I2C2_SCL,	/* GPIO4[12] */
	MX6Q_PAD_KEY_ROW3__I2C2_SDA,	/* GPIO4[13] */

	/* I2C3 */
	MX6Q_PAD_GPIO_5__I2C3_SCL,	
	MX6Q_PAD_EIM_D18__I2C3_SDA,	
 
	/* DISPLAY */
	MX6Q_PAD_DI0_DISP_CLK__IPU1_DI0_DISP_CLK,
	MX6Q_PAD_DI0_PIN15__IPU1_DI0_PIN15,		/* DE */
	MX6Q_PAD_DI0_PIN2__IPU1_DI0_PIN2,		/* HSync */
	MX6Q_PAD_DI0_PIN3__IPU1_DI0_PIN3,		/* VSync */
	MX6Q_PAD_DI0_PIN4__IPU1_DI0_PIN4,		/* Contrast */
	MX6Q_PAD_DISP0_DAT0__IPU1_DISP0_DAT_0,
	MX6Q_PAD_DISP0_DAT1__IPU1_DISP0_DAT_1,
	MX6Q_PAD_DISP0_DAT2__IPU1_DISP0_DAT_2,
	MX6Q_PAD_DISP0_DAT3__IPU1_DISP0_DAT_3,
	MX6Q_PAD_DISP0_DAT4__IPU1_DISP0_DAT_4,
	MX6Q_PAD_DISP0_DAT5__IPU1_DISP0_DAT_5,
	MX6Q_PAD_DISP0_DAT6__IPU1_DISP0_DAT_6,
	MX6Q_PAD_DISP0_DAT7__IPU1_DISP0_DAT_7,
	MX6Q_PAD_DISP0_DAT8__IPU1_DISP0_DAT_8,
	MX6Q_PAD_DISP0_DAT9__IPU1_DISP0_DAT_9,
	MX6Q_PAD_DISP0_DAT10__IPU1_DISP0_DAT_10,
	MX6Q_PAD_DISP0_DAT11__IPU1_DISP0_DAT_11,
	MX6Q_PAD_DISP0_DAT12__IPU1_DISP0_DAT_12,
	MX6Q_PAD_DISP0_DAT13__IPU1_DISP0_DAT_13,
	MX6Q_PAD_DISP0_DAT14__IPU1_DISP0_DAT_14,
	MX6Q_PAD_DISP0_DAT15__IPU1_DISP0_DAT_15,
	MX6Q_PAD_DISP0_DAT16__IPU1_DISP0_DAT_16,
	MX6Q_PAD_DISP0_DAT17__IPU1_DISP0_DAT_17,
	MX6Q_PAD_DISP0_DAT18__IPU1_DISP0_DAT_18,
	MX6Q_PAD_DISP0_DAT19__IPU1_DISP0_DAT_19,

	MX6Q_PAD_DISP0_DAT20__AUDMUX_AUD4_TXC,
	MX6Q_PAD_DISP0_DAT21__AUDMUX_AUD4_TXD,
	MX6Q_PAD_DISP0_DAT22__AUDMUX_AUD4_TXFS,
	MX6Q_PAD_DISP0_DAT23__AUDMUX_AUD4_RXD,

	MX6Q_PAD_GPIO_7__GPIO_1_7,		/* J7 - Display Connector GP */
	MX6Q_PAD_GPIO_9__GPIO_1_9,		/* J7 - Display Connector GP */
	MX6Q_PAD_NANDF_D0__GPIO_2_0,		/* J6 - LVDS Display contrast */

	/* UART1  */
	MX6Q_PAD_SD3_DAT7__UART1_TXD,
	MX6Q_PAD_SD3_DAT6__UART1_RXD,

	/* UART2*/
	MX6Q_PAD_EIM_D26__UART2_TXD,
	MX6Q_PAD_EIM_D27__UART2_RXD,


	/* UART3*/
	MX6Q_PAD_EIM_D23__GPIO_3_23,
	MX6Q_PAD_EIM_D24__UART3_TXD,
	MX6Q_PAD_EIM_D25__UART3_RXD,

	/* UART4*/
	MX6Q_PAD_KEY_COL0__UART4_TXD,
	MX6Q_PAD_KEY_ROW0__UART4_RXD,

	/* USB OC pin */
//	MX6Q_PAD_KEY_COL4__USBOH3_USBOTG_OC,


	/* USDHC1 */
	MX6Q_PAD_SD1_CLK__USDHC1_CLK,
	MX6Q_PAD_SD1_CMD__USDHC1_CMD,
	MX6Q_PAD_SD1_DAT0__USDHC1_DAT0,
	MX6Q_PAD_SD1_DAT1__USDHC1_DAT1,
	MX6Q_PAD_SD1_DAT2__USDHC1_DAT2,
	MX6Q_PAD_SD1_DAT3__USDHC1_DAT3,
	MX6Q_PAD_GPIO_1__GPIO_1_1,		/* SD1_CD */

	MX6Q_PAD_EIM_D31__GPIO_3_31,   /* MAX11801 irq*/


	/* USDHC2 */
	MX6Q_PAD_SD2_CLK__USDHC2_CLK,
	MX6Q_PAD_SD2_CMD__USDHC2_CMD,
	MX6Q_PAD_SD2_DAT0__USDHC2_DAT0,
	MX6Q_PAD_SD2_DAT1__USDHC2_DAT1,
	MX6Q_PAD_SD2_DAT2__USDHC2_DAT2,
	MX6Q_PAD_SD2_DAT3__USDHC2_DAT3,

	/* ipu1 csi0 */
	MX6Q_PAD_CSI0_DAT13__IPU1_CSI0_D_13,
	MX6Q_PAD_CSI0_DAT15__IPU1_CSI0_D_15,
	MX6Q_PAD_CSI0_DAT16__IPU1_CSI0_D_16,
	MX6Q_PAD_CSI0_DAT17__IPU1_CSI0_D_17,
	MX6Q_PAD_CSI0_DAT18__IPU1_CSI0_D_18,
	MX6Q_PAD_CSI0_DAT19__IPU1_CSI0_D_19,
	MX6Q_PAD_CSI0_VSYNC__IPU1_CSI0_VSYNC,
	MX6Q_PAD_CSI0_MCLK__IPU1_CSI0_HSYNC,
	MX6Q_PAD_CSI0_PIXCLK__IPU1_CSI0_PIXCLK,

	/* module revision detect */
	MX6Q_PAD_SD3_CMD__GPIO_7_2,
};

/* iCore module revision pin initialization for mx6q */

static iomux_v3_cfg_t mx6q_icore_pads_minor_D[] = {
	MX6Q_PAD_ENET_RX_ER__ENET_RX_ER,
};

static iomux_v3_cfg_t mx6q_icore_pads_greater_D[] = {
	MX6Q_PAD_ENET_RX_ER__ANATOP_USBOTG_ID,
};

static iomux_v3_cfg_t mx6q_icore_pads_forced_of_cap[] = {
	MX6Q_PAD_ENET_RX_ER__ANATOP_USBOTG_ID,
};


/* Engicam board pin initialization for mx6q */

static iomux_v3_cfg_t mx6q_icore_pads_resistive_sk[] = {
	MX6Q_PAD_CSI0_DAT12__IPU1_CSI0_D_12,
	MX6Q_PAD_CSI0_DAT14__IPU1_CSI0_D_14,
	MX6Q_PAD_GPIO_2__GPIO_1_2,		/* RESET WF111 */
};

static iomux_v3_cfg_t mx6q_icore_pads_resistive_of[] = {
	MX6Q_PAD_CSI0_DAT12__IPU1_CSI0_D_12,
	MX6Q_PAD_CSI0_DAT14__GPIO_6_0,
};

static iomux_v3_cfg_t mx6q_icore_pads_capacitive_of[] = {
	MX6Q_PAD_CSI0_DAT12__GPIO_5_30,
	MX6Q_PAD_CSI0_DAT14__IPU1_CSI0_D_14,
};

static iomux_v3_cfg_t mx6q_icore_pads_capacitive_ofamp[] = {
	MX6Q_PAD_CSI0_DAT12__GPIO_5_30,
	MX6Q_PAD_CSI0_DAT14__IPU1_CSI0_D_14,
};

static iomux_v3_cfg_t mx6q_icore_pads_capacitive_sk[] = {
	MX6Q_PAD_CSI0_DAT12__GPIO_5_30,
	MX6Q_PAD_CSI0_DAT14__IPU1_CSI0_D_14,
};

static iomux_v3_cfg_t mx6q_icore_pads_mediaplayer[] = {
	MX6Q_PAD_CSI0_DAT12__GPIO_5_30,
	MX6Q_PAD_CSI0_DAT13__GPIO_5_31,
	MX6Q_PAD_CSI0_DAT14__GPIO_6_0,
	MX6Q_PAD_GPIO_7__GPIO_1_7 ,		/* RESET WF111 */
};

static iomux_v3_cfg_t mx6q_icore_pads_capacitive_of_urt[] = {
	MX6Q_PAD_CSI0_DAT12__GPIO_5_30,
	MX6Q_PAD_CSI0_DAT13__GPIO_5_31,
	MX6Q_PAD_CSI0_DAT14__UART5_TXD,
	MX6Q_PAD_CSI0_DAT15__UART5_RXD,
	MX6Q_PAD_CSI0_DAT16__GPIO_6_2,   	/* RESET WF111 */
};

static iomux_v3_cfg_t mx6dl_icore_pads[] = {
	/* CAN1  */	
	MX6DL_PAD_KEY_ROW2__CAN1_RXCAN,
	MX6DL_PAD_KEY_COL2__CAN1_TXCAN,

	/* CAN2  */
	MX6DL_PAD_KEY_ROW4__CAN2_RXCAN,
	MX6DL_PAD_KEY_COL4__CAN2_TXCAN,

	MX6DL_PAD_GPIO_7__GPIO_1_7,		/* NERR */
	MX6DL_PAD_GPIO_4__GPIO_1_4,		/* Enable */

	/* CCM  */
	MX6DL_PAD_GPIO_0__CCM_CLKO,		/* SGTL500 sys_mclk */
	MX6DL_PAD_GPIO_3__CCM_CLKO2,		/* J5 - Camera MCLK */


	MX6DL_PAD_SD4_DAT1__PWM3_PWMO	,	/* pwm per backlight */

	/* ENET */
	MX6DL_PAD_ENET_CRS_DV__ENET_RX_EN 	,
	MX6DL_PAD_GPIO_16__ENET_ANATOP_ETHERNET_REF_OUT,
	MX6DL_PAD_ENET_TX_EN__ENET_TX_EN		,
	MX6DL_PAD_ENET_RXD1__ENET_RDATA_1	,
	MX6DL_PAD_ENET_RXD0__ENET_RDATA_0	,
	MX6DL_PAD_ENET_TXD1__ENET_TDATA_1	,
	MX6DL_PAD_ENET_TXD0__ENET_TDATA_0	,
	MX6DL_PAD_ENET_MDC__ENET_MDC		,
	MX6DL_PAD_ENET_MDIO__ENET_MDIO		,
	MX6DL_PAD_ENET_REF_CLK__GPIO_1_23	,	/* TBD da capire se puo' uscire 50MHz, altrimenti GPIO input */
	MX6DL_PAD_GPIO_17__GPIO_7_12 		,

	/* GPIO7 */
	MX6DL_PAD_GPIO_17__GPIO_7_12,	/* USB Hub Reset */
	MX6DL_PAD_GPIO_18__GPIO_7_13,	/* J14 - Volume Up */

	MX6DL_PAD_EIM_D21__I2C1_SCL,	/* GPIO3[21] */
	MX6DL_PAD_EIM_D28__I2C1_SDA,	/* GPIO3[28] */

	/* I2C2 Camera, MIPI */
	MX6DL_PAD_KEY_COL3__I2C2_SCL,	/* GPIO4[12] */
	MX6DL_PAD_KEY_ROW3__I2C2_SDA,	/* GPIO4[13] */

	/* I2C3 */
	MX6DL_PAD_GPIO_5__I2C3_SCL,	
	MX6DL_PAD_EIM_D18__I2C3_SDA,	
 
	/* DISPLAY */
	MX6DL_PAD_DI0_DISP_CLK__IPU1_DI0_DISP_CLK,
	MX6DL_PAD_DI0_PIN15__IPU1_DI0_PIN15,		/* DE */
	MX6DL_PAD_DI0_PIN2__IPU1_DI0_PIN2,		/* HSync */
	MX6DL_PAD_DI0_PIN3__IPU1_DI0_PIN3,		/* VSync */
	MX6DL_PAD_DI0_PIN4__IPU1_DI0_PIN4,		/* Contrast */
	MX6DL_PAD_DISP0_DAT0__IPU1_DISP0_DAT_0,
	MX6DL_PAD_DISP0_DAT1__IPU1_DISP0_DAT_1,
	MX6DL_PAD_DISP0_DAT2__IPU1_DISP0_DAT_2,
	MX6DL_PAD_DISP0_DAT3__IPU1_DISP0_DAT_3,
	MX6DL_PAD_DISP0_DAT4__IPU1_DISP0_DAT_4,
	MX6DL_PAD_DISP0_DAT5__IPU1_DISP0_DAT_5,
	MX6DL_PAD_DISP0_DAT6__IPU1_DISP0_DAT_6,
	MX6DL_PAD_DISP0_DAT7__IPU1_DISP0_DAT_7,
	MX6DL_PAD_DISP0_DAT8__IPU1_DISP0_DAT_8,
	MX6DL_PAD_DISP0_DAT9__IPU1_DISP0_DAT_9,
	MX6DL_PAD_DISP0_DAT10__IPU1_DISP0_DAT_10,
	MX6DL_PAD_DISP0_DAT11__IPU1_DISP0_DAT_11,
	MX6DL_PAD_DISP0_DAT12__IPU1_DISP0_DAT_12,
	MX6DL_PAD_DISP0_DAT13__IPU1_DISP0_DAT_13,
	MX6DL_PAD_DISP0_DAT14__IPU1_DISP0_DAT_14,
	MX6DL_PAD_DISP0_DAT15__IPU1_DISP0_DAT_15,
	MX6DL_PAD_DISP0_DAT16__IPU1_DISP0_DAT_16,
	MX6DL_PAD_DISP0_DAT17__IPU1_DISP0_DAT_17,
	MX6DL_PAD_DISP0_DAT18__IPU1_DISP0_DAT_18,
	MX6DL_PAD_DISP0_DAT19__IPU1_DISP0_DAT_19,

	MX6DL_PAD_DISP0_DAT20__AUDMUX_AUD4_TXC,
	MX6DL_PAD_DISP0_DAT21__AUDMUX_AUD4_TXD,
	MX6DL_PAD_DISP0_DAT22__AUDMUX_AUD4_TXFS,
	MX6DL_PAD_DISP0_DAT23__AUDMUX_AUD4_RXD,

	MX6DL_PAD_GPIO_7__GPIO_1_7,		/* J7 - Display Connector GP */
	MX6DL_PAD_GPIO_9__GPIO_1_9,		/* J7 - Display Connector GP */
	MX6DL_PAD_NANDF_D0__GPIO_2_0,		/* J6 - LVDS Display contrast */

	/* UART1  */
	MX6DL_PAD_SD3_DAT7__UART1_TXD,
	MX6DL_PAD_SD3_DAT6__UART1_RXD,

	/* UART2*/
	MX6DL_PAD_EIM_D26__UART2_TXD,
	MX6DL_PAD_EIM_D27__UART2_RXD,

	/* UART3*/
	MX6DL_PAD_EIM_D23__GPIO_3_23,
	MX6DL_PAD_EIM_D24__UART3_TXD,
	MX6DL_PAD_EIM_D25__UART3_RXD,

	/* UART4*/
	MX6DL_PAD_KEY_COL0__UART4_TXD,
	MX6DL_PAD_KEY_ROW0__UART4_RXD,

	/* USB OC pin */
//	MX6DL_PAD_KEY_COL4__USBOH3_USBOTG_OC,


	/* USDHC1 */
	MX6DL_PAD_SD1_CLK__USDHC1_CLK,
	MX6DL_PAD_SD1_CMD__USDHC1_CMD,
	MX6DL_PAD_SD1_DAT0__USDHC1_DAT0,
	MX6DL_PAD_SD1_DAT1__USDHC1_DAT1,
	MX6DL_PAD_SD1_DAT2__USDHC1_DAT2,
	MX6DL_PAD_SD1_DAT3__USDHC1_DAT3,
	MX6DL_PAD_GPIO_1__GPIO_1_1,		/* SD1_CD */

	MX6DL_PAD_EIM_D31__GPIO_3_31,   /* MAX11801 irq*/

	/* USDHC2 */
	MX6DL_PAD_SD2_CLK__USDHC2_CLK,
	MX6DL_PAD_SD2_CMD__USDHC2_CMD,
	MX6DL_PAD_SD2_DAT0__USDHC2_DAT0,
	MX6DL_PAD_SD2_DAT1__USDHC2_DAT1,
	MX6DL_PAD_SD2_DAT2__USDHC2_DAT2,
	MX6DL_PAD_SD2_DAT3__USDHC2_DAT3,

	/* ipu1 csi0 */
	MX6DL_PAD_CSI0_DAT13__IPU1_CSI0_D_13,
	MX6DL_PAD_CSI0_DAT15__IPU1_CSI0_D_15,
	MX6DL_PAD_CSI0_DAT16__IPU1_CSI0_D_16,
	MX6DL_PAD_CSI0_DAT17__IPU1_CSI0_D_17,
	MX6DL_PAD_CSI0_DAT18__IPU1_CSI0_D_18,
	MX6DL_PAD_CSI0_DAT19__IPU1_CSI0_D_19,
	MX6DL_PAD_CSI0_VSYNC__IPU1_CSI0_VSYNC,
	MX6DL_PAD_CSI0_MCLK__IPU1_CSI0_HSYNC,
	MX6DL_PAD_CSI0_PIXCLK__IPU1_CSI0_PIXCLK,

	/* module revision detect */
	MX6DL_PAD_SD3_CMD__GPIO_7_2,
};


/* iCore module revision pin initialization for mx6q */

static iomux_v3_cfg_t mx6dl_icore_pads_minor_D[] = {
	MX6DL_PAD_ENET_RX_ER__ENET_RX_ER,
};

static iomux_v3_cfg_t mx6dl_icore_pads_greater_D[] = {
	MX6DL_PAD_ENET_RX_ER__ANATOP_USBOTG_ID,
};

static iomux_v3_cfg_t mx6dl_icore_pads_forced_of_cap[] = {
	MX6DL_PAD_ENET_RX_ER__ANATOP_USBOTG_ID,
};


/* Engicam board pin initialization for mx6dl */

static iomux_v3_cfg_t mx6dl_icore_pads_resistive_sk[] = {
	MX6DL_PAD_CSI0_DAT12__IPU1_CSI0_D_12,
	MX6DL_PAD_CSI0_DAT14__IPU1_CSI0_D_14,
	MX6DL_PAD_GPIO_2__GPIO_1_2,		/* RESET WF111 */
};

static iomux_v3_cfg_t mx6dl_icore_pads_resistive_of[] = {
	MX6DL_PAD_CSI0_DAT12__IPU1_CSI0_D_12,
	MX6DL_PAD_CSI0_DAT14__GPIO_6_0,
};

static iomux_v3_cfg_t mx6dl_icore_pads_capacitive_of[] = {
	MX6DL_PAD_CSI0_DAT12__GPIO_5_30,
	MX6DL_PAD_CSI0_DAT14__IPU1_CSI0_D_14,
};

static iomux_v3_cfg_t mx6dl_icore_pads_capacitive_ofamp[] = {
	MX6DL_PAD_CSI0_DAT12__GPIO_5_30,
	MX6DL_PAD_CSI0_DAT14__IPU1_CSI0_D_14,
};

static iomux_v3_cfg_t mx6dl_icore_pads_capacitive_sk[] = {
	MX6DL_PAD_CSI0_DAT12__GPIO_5_30,
	MX6DL_PAD_CSI0_DAT14__IPU1_CSI0_D_14,
};

static iomux_v3_cfg_t mx6dl_icore_pads_mediaplayer[] = {
	MX6DL_PAD_CSI0_DAT12__GPIO_5_30,
	MX6DL_PAD_CSI0_DAT13__GPIO_5_31,
	MX6DL_PAD_CSI0_DAT14__GPIO_6_0,
	MX6DL_PAD_GPIO_7__GPIO_1_7, 		/* RESET WF111 */
};

static iomux_v3_cfg_t mx6dl_icore_pads_capacitive_of_urt[] = {
	MX6DL_PAD_CSI0_DAT12__GPIO_5_30,
	MX6DL_PAD_CSI0_DAT13__GPIO_5_31,
	MX6DL_PAD_CSI0_DAT14__UART5_TXD,
	MX6DL_PAD_CSI0_DAT15__UART5_RXD,
	MX6DL_PAD_CSI0_DAT16__GPIO_6_2,   	/* RESET WF111 */
};

/* The GPMI is conflicted with SD3, so init this in the driver. */
static iomux_v3_cfg_t mx6q_gpmi_nand[] __initdata = {
	MX6Q_PAD_NANDF_CLE__RAWNAND_CLE,
	MX6Q_PAD_NANDF_ALE__RAWNAND_ALE,
	MX6Q_PAD_NANDF_CS0__RAWNAND_CE0N,
	MX6Q_PAD_NANDF_RB0__RAWNAND_READY0,
	MX6Q_PAD_NANDF_D0__RAWNAND_D0,
	MX6Q_PAD_NANDF_D1__RAWNAND_D1,
	MX6Q_PAD_NANDF_D2__RAWNAND_D2,
	MX6Q_PAD_NANDF_D3__RAWNAND_D3,
	MX6Q_PAD_NANDF_D4__RAWNAND_D4,
	MX6Q_PAD_NANDF_D5__RAWNAND_D5,
	MX6Q_PAD_NANDF_D6__RAWNAND_D6,
	MX6Q_PAD_NANDF_D7__RAWNAND_D7,
	MX6Q_PAD_SD4_CMD__RAWNAND_RDN,
	MX6Q_PAD_SD4_CLK__RAWNAND_WRN,
	MX6Q_PAD_NANDF_WP_B__RAWNAND_RESETN,
};

/* The GPMI is conflicted with SD3, so init this in the driver. */
static iomux_v3_cfg_t mx6dl_gpmi_nand[] __initdata = {
	MX6DL_PAD_NANDF_CLE__RAWNAND_CLE,
	MX6DL_PAD_NANDF_ALE__RAWNAND_ALE,
	MX6DL_PAD_NANDF_CS0__RAWNAND_CE0N,
	MX6DL_PAD_NANDF_RB0__RAWNAND_READY0,
	MX6DL_PAD_NANDF_D0__RAWNAND_D0,
	MX6DL_PAD_NANDF_D1__RAWNAND_D1,
	MX6DL_PAD_NANDF_D2__RAWNAND_D2,
	MX6DL_PAD_NANDF_D3__RAWNAND_D3,
	MX6DL_PAD_NANDF_D4__RAWNAND_D4,
	MX6DL_PAD_NANDF_D5__RAWNAND_D5,
	MX6DL_PAD_NANDF_D6__RAWNAND_D6,
	MX6DL_PAD_NANDF_D7__RAWNAND_D7,
	MX6DL_PAD_SD4_CMD__RAWNAND_RDN,
	MX6DL_PAD_SD4_CLK__RAWNAND_WRN,
	MX6DL_PAD_NANDF_WP_B__RAWNAND_RESETN,
};

#if 0

#define MX6Q_USDHC_PAD_SETTING(id, speed)	\
mx6q_sd##id##_##speed##mhz[] = {		\
	MX6Q_PAD_SD##id##_CLK__USDHC##id##_CLK_##speed##MHZ,	\
	MX6Q_PAD_SD##id##_CMD__USDHC##id##_CMD_##speed##MHZ,	\
	MX6Q_PAD_SD##id##_DAT0__USDHC##id##_DAT0_##speed##MHZ,	\
	MX6Q_PAD_SD##id##_DAT1__USDHC##id##_DAT1_##speed##MHZ,	\
	MX6Q_PAD_SD##id##_DAT2__USDHC##id##_DAT2_##speed##MHZ,	\
	MX6Q_PAD_SD##id##_DAT3__USDHC##id##_DAT3_##speed##MHZ,	\
}

static iomux_v3_cfg_t MX6Q_USDHC_PAD_SETTING(3, 50);
static iomux_v3_cfg_t MX6Q_USDHC_PAD_SETTING(3, 100);
static iomux_v3_cfg_t MX6Q_USDHC_PAD_SETTING(3, 200);
static iomux_v3_cfg_t MX6Q_USDHC_PAD_SETTING(4, 50);
static iomux_v3_cfg_t MX6Q_USDHC_PAD_SETTING(4, 100);
static iomux_v3_cfg_t MX6Q_USDHC_PAD_SETTING(4, 200);
#else

#define MX6Q_USDHC_PAD_SETTING(id, speed)	\
mx6q_sd##id##_##speed##mhz[] = {		\
	MX6Q_PAD_SD##id##_CLK__USDHC##id##_CLK,	\
	MX6Q_PAD_SD##id##_CMD__USDHC##id##_CMD,	\
	MX6Q_PAD_SD##id##_DAT0__USDHC##id##_DAT0,	\
	MX6Q_PAD_SD##id##_DAT1__USDHC##id##_DAT1,	\
	MX6Q_PAD_SD##id##_DAT2__USDHC##id##_DAT2,	\
	MX6Q_PAD_SD##id##_DAT3__USDHC##id##_DAT3,	\
}

#define MX6DL_USDHC_PAD_SETTING(id, speed)	\
mx6dl_sd##id##_##speed##mhz[] = {		\
	MX6DL_PAD_SD##id##_CLK__USDHC##id##_CLK,	\
	MX6DL_PAD_SD##id##_CMD__USDHC##id##_CMD,	\
	MX6DL_PAD_SD##id##_DAT0__USDHC##id##_DAT0,	\
	MX6DL_PAD_SD##id##_DAT1__USDHC##id##_DAT1,	\
	MX6DL_PAD_SD##id##_DAT2__USDHC##id##_DAT2,	\
	MX6DL_PAD_SD##id##_DAT3__USDHC##id##_DAT3,	\
}


static iomux_v3_cfg_t MX6Q_USDHC_PAD_SETTING(1, 50);
static iomux_v3_cfg_t MX6Q_USDHC_PAD_SETTING(1, 100);
static iomux_v3_cfg_t MX6Q_USDHC_PAD_SETTING(1, 200);

static iomux_v3_cfg_t MX6DL_USDHC_PAD_SETTING(1, 50);
static iomux_v3_cfg_t MX6DL_USDHC_PAD_SETTING(1, 100);
static iomux_v3_cfg_t MX6DL_USDHC_PAD_SETTING(1, 200);

static iomux_v3_cfg_t MX6Q_USDHC_PAD_SETTING(2, 50);
static iomux_v3_cfg_t MX6Q_USDHC_PAD_SETTING(2, 100);
static iomux_v3_cfg_t MX6Q_USDHC_PAD_SETTING(2, 200);

static iomux_v3_cfg_t MX6DL_USDHC_PAD_SETTING(2, 50);
static iomux_v3_cfg_t MX6DL_USDHC_PAD_SETTING(2, 100);
static iomux_v3_cfg_t MX6DL_USDHC_PAD_SETTING(2, 200);
#endif






enum sd_pad_mode {
	SD_PAD_MODE_LOW_SPEED,
	SD_PAD_MODE_MED_SPEED,
	SD_PAD_MODE_HIGH_SPEED,
};

#if 0
static int plt_sd3_pad_change(int clock)
{
	static enum sd_pad_mode pad_mode = SD_PAD_MODE_LOW_SPEED;

	if (clock > 100000000) {
		if (pad_mode == SD_PAD_MODE_HIGH_SPEED)
			return 0;

		pad_mode = SD_PAD_MODE_HIGH_SPEED;
		return mxc_iomux_v3_setup_multiple_pads(mx6q_sd3_200mhz,
					ARRAY_SIZE(mx6q_sd3_200mhz));
	} else if (clock > 52000000) {
		if (pad_mode == SD_PAD_MODE_MED_SPEED)
			return 0;

		pad_mode = SD_PAD_MODE_MED_SPEED;
		return mxc_iomux_v3_setup_multiple_pads(mx6q_sd3_100mhz,
					ARRAY_SIZE(mx6q_sd3_100mhz));
	} else {
		if (pad_mode == SD_PAD_MODE_LOW_SPEED)
			return 0;

		pad_mode = SD_PAD_MODE_LOW_SPEED;
		return mxc_iomux_v3_setup_multiple_pads(mx6q_sd3_50mhz,
					ARRAY_SIZE(mx6q_sd3_50mhz));
	}
}

static int plt_sd4_pad_change(int clock)
{
	static enum sd_pad_mode pad_mode = SD_PAD_MODE_LOW_SPEED;

	if (clock > 100000000) {
		if (pad_mode == SD_PAD_MODE_HIGH_SPEED)
			return 0;

		pad_mode = SD_PAD_MODE_HIGH_SPEED;
		return mxc_iomux_v3_setup_multiple_pads(mx6q_sd4_200mhz,
					ARRAY_SIZE(mx6q_sd4_200mhz));
	} else if (clock > 52000000) {
		if (pad_mode == SD_PAD_MODE_MED_SPEED)
			return 0;

		pad_mode = SD_PAD_MODE_MED_SPEED;
		return mxc_iomux_v3_setup_multiple_pads(mx6q_sd4_100mhz,
					ARRAY_SIZE(mx6q_sd4_100mhz));
	} else {
		if (pad_mode == SD_PAD_MODE_LOW_SPEED)
			return 0;

		pad_mode = SD_PAD_MODE_LOW_SPEED;
		return mxc_iomux_v3_setup_multiple_pads(mx6q_sd4_50mhz,
					ARRAY_SIZE(mx6q_sd4_50mhz));
	}
}
#else
static int plt_sd1_pad_change(unsigned int index, int clock)
{

	static enum sd_pad_mode pad_mode = SD_PAD_MODE_LOW_SPEED;
	if (clock > 100000000) {
		if (pad_mode == SD_PAD_MODE_HIGH_SPEED)
			return 0;

		pad_mode = SD_PAD_MODE_HIGH_SPEED;
		if (cpu_is_mx6q()) 
		return mxc_iomux_v3_setup_multiple_pads(mx6q_sd1_200mhz,
					ARRAY_SIZE(mx6q_sd1_200mhz));
		else
			return mxc_iomux_v3_setup_multiple_pads(mx6dl_sd1_200mhz,			
				ARRAY_SIZE(mx6dl_sd1_200mhz));
	} else if (clock > 52000000) {
		if (pad_mode == SD_PAD_MODE_MED_SPEED)
			return 0;

		pad_mode = SD_PAD_MODE_MED_SPEED;
		if (cpu_is_mx6q()) 
		return mxc_iomux_v3_setup_multiple_pads(mx6q_sd1_100mhz,
					ARRAY_SIZE(mx6q_sd1_100mhz));
		else
			return mxc_iomux_v3_setup_multiple_pads(mx6dl_sd1_100mhz,			
				ARRAY_SIZE(mx6dl_sd1_100mhz));
	} else {
		if (pad_mode == SD_PAD_MODE_LOW_SPEED)
			return 0;

		pad_mode = SD_PAD_MODE_LOW_SPEED;
		if (cpu_is_mx6q()) 
		return mxc_iomux_v3_setup_multiple_pads(mx6q_sd1_50mhz,
					ARRAY_SIZE(mx6q_sd1_50mhz));
		else
			return mxc_iomux_v3_setup_multiple_pads(mx6dl_sd1_50mhz,			
				ARRAY_SIZE(mx6dl_sd1_50mhz));
	}
}

static int plt_sd2_pad_change(unsigned int index, int clock)
{
	static enum sd_pad_mode pad_mode = SD_PAD_MODE_LOW_SPEED;
	if (clock > 100000000) {
		if (pad_mode == SD_PAD_MODE_HIGH_SPEED)
			return 0;

		pad_mode = SD_PAD_MODE_HIGH_SPEED;
		if (cpu_is_mx6q()) 
			return mxc_iomux_v3_setup_multiple_pads(mx6q_sd2_200mhz,			
				ARRAY_SIZE(mx6q_sd2_200mhz));
		else
			return mxc_iomux_v3_setup_multiple_pads(mx6dl_sd2_200mhz,			
				ARRAY_SIZE(mx6dl_sd2_200mhz));
	} else if (clock > 52000000) {
		if (pad_mode == SD_PAD_MODE_MED_SPEED)
			return 0;

		pad_mode = SD_PAD_MODE_MED_SPEED;
		if (cpu_is_mx6q()) 
			return mxc_iomux_v3_setup_multiple_pads(mx6q_sd2_100mhz,
					ARRAY_SIZE(mx6q_sd2_100mhz));
		else
			return mxc_iomux_v3_setup_multiple_pads(mx6dl_sd2_100mhz,			
				ARRAY_SIZE(mx6dl_sd2_100mhz));
	} else {
		if (pad_mode == SD_PAD_MODE_LOW_SPEED)
			return 0;

		pad_mode = SD_PAD_MODE_LOW_SPEED;
		if (cpu_is_mx6q()) 
			return mxc_iomux_v3_setup_multiple_pads(mx6q_sd2_50mhz,
					ARRAY_SIZE(mx6q_sd2_50mhz));
		else
			return mxc_iomux_v3_setup_multiple_pads(mx6dl_sd2_50mhz,			
				ARRAY_SIZE(mx6dl_sd2_50mhz));
	}
}

#endif
#if 1
static const struct esdhc_platform_data mx6q_icore_sd1_data __initconst = {
	.cd_gpio = ICORE_M6_SD1_CD,
	.wp_gpio = ICORE_M6_SD1_WP,
	.keep_power_at_suspend = 1,
	.platform_pad_change = plt_sd1_pad_change,
};
static const struct esdhc_platform_data mx6q_icore_sd2_data __initconst = {
	.always_present = 1,
	.cd_type = ESDHC_CD_PERMANENT,
	.cd_gpio = -1,
	.wp_gpio = -1,
	.keep_power_at_suspend = 1,
	.platform_pad_change = plt_sd2_pad_change,
};
#else
static const struct esdhc_platform_data mx6q_icore_sd3_data __initconst = {
	.cd_gpio = ICORE_M6_SD3_CD,
	.wp_gpio = ICORE_M6_SD3_WP,
	.keep_power_at_suspend = 1,
	.platform_pad_change = plt_sd3_pad_change,
};

static const struct esdhc_platform_data mx6q_icore_sd4_data __initconst = {
	.cd_gpio = ICORE_M6_SD4_CD,
	.wp_gpio = ICORE_M6_SD4_WP,
	.keep_power_at_suspend = 1,
	.platform_pad_change = plt_sd4_pad_change,
};
#endif

static int __init gpmi_nand_platform_init(void)
{
	iomux_v3_cfg_t *nand_pads = NULL;
	u32 nand_pads_cnt;

	if (cpu_is_mx6q()) {
		nand_pads = mx6q_gpmi_nand;
		nand_pads_cnt = ARRAY_SIZE(mx6q_gpmi_nand);
	} else {
		nand_pads = mx6dl_gpmi_nand;
		nand_pads_cnt = ARRAY_SIZE(mx6dl_gpmi_nand);
	}
	BUG_ON(!nand_pads);
	return mxc_iomux_v3_setup_multiple_pads(nand_pads, nand_pads_cnt);
}

static struct mtd_partition imx6_icore_nand_partitions[] = {
	{
	 .name = "bootloader",
	 .offset = 0,
	 .size = 0x00400000,
	},
	{
	 .name = "kernel",
	 .offset = MTDPART_OFS_APPEND,
	 .size = 0x00700000,
	},
	{
	 .name = "rootfs",
	 .offset = MTDPART_OFS_APPEND,
	 .size = MTDPART_SIZ_FULL,
	},
};
static const struct gpmi_nand_platform_data
mx6q_gpmi_nand_platform_data __initconst = {
	.platform_init           = gpmi_nand_platform_init,
	.min_prop_delay_in_ns    = 5,
	.max_prop_delay_in_ns    = 9,
	.max_chip_count          = 1,
	.partitions = imx6_icore_nand_partitions,
	.partition_count = ARRAY_SIZE(imx6_icore_nand_partitions),
};

static const struct anatop_thermal_platform_data
	mx6q_icore_anatop_thermal_data __initconst = {
		.name = "anatop_thermal",
};

static inline void mx6q_icore_init_uart(void)
{
	imx6q_add_imx_uart(0, NULL);
	imx6q_add_imx_uart(1, NULL);	
	imx6q_add_imx_uart(2, NULL);	
	imx6q_add_imx_uart(3, NULL);
}

static int mx6q_icore_fec_phy_init(struct phy_device *phydev)
{
#if 0
	/* prefer master mode, disable 1000 Base-T capable */
	phy_write(phydev, 0x9, 0x1c00);

	/* min rx data delay */
	phy_write(phydev, 0x0b, 0x8105);
	phy_write(phydev, 0x0c, 0x0000);

	/* max rx/tx clock delay, min rx/tx control delay */
	phy_write(phydev, 0x0b, 0x8104);
	phy_write(phydev, 0x0c, 0xf0f0);
	phy_write(phydev, 0x0b, 0x104);
#endif
	return 0;
}

static struct fec_platform_data fec_data __initdata = {
	.init = mx6q_icore_fec_phy_init,
	.phy = PHY_INTERFACE_MODE_RMII,
};

static int mx6q_icore_spi_cs[] = {
	ICORE_M6_ECSPI1_CS1,
};

static const struct spi_imx_master mx6q_icore_spi_data __initconst = {
	.chipselect     = mx6q_icore_spi_cs,
	.num_chipselect = ARRAY_SIZE(mx6q_icore_spi_cs),
};
#if 0
#if defined(CONFIG_MTD_M25P80) || defined(CONFIG_MTD_M25P80_MODULE)
static struct mtd_partition imx6_icore_spi_nor_partitions[] = {
	{
	 .name = "bootloader",
	 .offset = 0,
	 .size = 0x00040000,
	},
	{
	 .name = "kernel",
	 .offset = MTDPART_OFS_APPEND,
	 .size = MTDPART_SIZ_FULL,
	},
};

static struct flash_platform_data imx6_icore__spi_flash_data = {
	.name = "m25p80",
	.parts = imx6_icore_spi_nor_partitions,
	.nr_parts = ARRAY_SIZE(imx6_icore_spi_nor_partitions),
	.type = "sst25vf016b",
};
#endif

static struct spi_board_info imx6_icore_spi_nor_device[] __initdata = {
#if defined(CONFIG_MTD_M25P80)
	{
		.modalias = "m25p80",
		.max_speed_hz = 20000000, /* max spi clock (SCK) speed in HZ */
		.bus_num = 0,
		.chip_select = 0,
		.platform_data = &imx6_icore__spi_flash_data,
	},
#endif
};
#endif
#if 0
static void spi_device_init(void)
{
	spi_register_board_info(imx6_icore_spi_nor_device,
				ARRAY_SIZE(imx6_icore_spi_nor_device));
}
#endif
static struct mxc_audio_platform_data mx6_icore_audio_data;

static int mx6_icore_sgtl5000_init(void)
{
	struct clk *clko;
	struct clk *new_parent;
	int rate;

	clko = clk_get(NULL, "clko_clk");
	if (IS_ERR(clko)) {
		pr_err("can't get CLKO clock.\n");
		return PTR_ERR(clko);
	}
	new_parent = clk_get(NULL, "ahb");
	if (!IS_ERR(new_parent)) {
		clk_set_parent(clko, new_parent);
		clk_put(new_parent);
	}
	rate = clk_round_rate(clko, 16000000);
	if (rate < 8000000 || rate > 27000000) {
		pr_err("Error:SGTL5000 mclk freq %d out of range!\n", rate);
		clk_put(clko);
		return -1;
	}

	mx6_icore_audio_data.sysclk = rate;
	clk_set_rate(clko, rate);
	clk_enable(clko);
	return 0;
}

static struct imx_ssi_platform_data mx6_icore_ssi_pdata = {
	.flags = IMX_SSI_DMA | IMX_SSI_SYN,
};

static struct mxc_audio_platform_data mx6_icore_audio_data = {
	.ssi_num = 1,
	.src_port = 2,
	.ext_port = 4,
	.init = mx6_icore_sgtl5000_init,
	.hp_gpio = -1,
};

static struct platform_device mx6_icore_audio_device = {
	.name = "imx-sgtl5000",
};

static void adv7180_pwdn(int pwdn)
{
}

static struct fsl_mxc_tvin_platform_data adv7180_data = {
	.dvddio_reg	= NULL,
	.dvdd_reg	= NULL,
	.avdd_reg	= NULL,
	.pvdd_reg	= NULL,
	.pwdn		= adv7180_pwdn,
	.reset		= NULL,
	.cvbs		= true,
};

static struct imxi2c_platform_data mx6q_icore_i2c_data = {
	.bitrate = 100000,
};


static struct i2c_board_info mxc_i2c0_board_info[] __initdata = {
	
};

static struct i2c_board_info mxc_i2c0_max11801_info[] __initdata = {
	{
		I2C_BOARD_INFO("max11801", 0x48),
		.platform_data = (void *)&max11801_mode,
		.irq  = gpio_to_irq(MAX11801_TS_IRQ),
	},
};

static struct edt_ft5x06_platform_data mx6_icore_ft5x06_data = {
	.reset_pin      = -1,   /* static high */
};

bool ili210x_get_pendown_state (void)
{
	return !gpio_get_value(OFC_FT5X06_TS_IRQ);
}

static struct ili210x_platform_data mx6_icore_ili210x_data = {          
        .irq_flags		= (IRQF_TRIGGER_FALLING | IRQF_ONESHOT),        
        .poll_period		= 20,       
        .get_pendown_state	= ili210x_get_pendown_state,
};                                      

static struct i2c_board_info mxc_i2c1_board_info[] __initdata = {
	{
		I2C_BOARD_INFO("mxc_hdmi_i2c", 0x50),
	},
};

static struct i2c_board_info mxc_i2c2_board_info[] __initdata = {
	{
		I2C_BOARD_INFO("sgtl5000", 0x0a),
	},
};

/* Engicam board I2C initialization */

static struct i2c_board_info mxc_i2c2_board_info_skres[] __initdata = {
	{
		I2C_BOARD_INFO("adv7180", 0x21),
		.platform_data = (void *)&adv7180_data,
	},
};

static struct i2c_board_info mxc_i2c2_board_info_ofres[] __initdata = {
	{
		I2C_BOARD_INFO("pcf8563", 0x51),
	},
};

static struct i2c_board_info mxc_i2c2_board_info_ofcap[] __initdata = {
	{
		I2C_BOARD_INFO("edt-ft5x06", 0x38),
		.irq  = gpio_to_irq(OFC_FT5X06_TS_IRQ),
		.platform_data = (void *) & mx6_icore_ft5x06_data,
	},
	{
		I2C_BOARD_INFO("pcf8563", 0x51),
	},
};

static struct i2c_board_info mxc_i2c2_board_info_ofamp[] __initdata = {
	{
		I2C_BOARD_INFO("ili210x", 0x41),
		.platform_data = (void *) & mx6_icore_ili210x_data,
		.irq  = gpio_to_irq(OFC_FT5X06_TS_IRQ),
	},
	{
		I2C_BOARD_INFO("pcf8563", 0x51),
	},
};

static struct i2c_board_info mxc_i2c2_board_info_skcap[] __initdata = {
	{
		I2C_BOARD_INFO("adv7180", 0x21),
		.platform_data = (void *)&adv7180_data,
	},
	{
		I2C_BOARD_INFO("edt-ft5x06", 0x38),
		.irq  = gpio_to_irq(STARTERKIT_CAPEDT_IRQ),
		.platform_data = (void *) & mx6_icore_ft5x06_data,
	},
};

static struct i2c_board_info mxc_i2c2_board_info_mediaplayer[] __initdata = {
	{
		I2C_BOARD_INFO("pcf8563", 0x51),
	},
};

static struct i2c_board_info mxc_i2c2_board_info_of_urt[] __initdata = {
	{
		I2C_BOARD_INFO("st1232-ts", 0x55),
		.irq  = gpio_to_irq(OFC_URT_TS_IRQ),
	},
};

static void imx6q_icore_usbotg_vbus(bool on)
{
#if 0
	if (on)
		gpio_set_value(ICORE_M6_USB_OTG_PWR, 1);
	else
		gpio_set_value(ICORE_M6_USB_OTG_PWR, 0);
#endif
}

static void __init imx6q_icore_init_usb(void)
{

	imx_otg_base = MX6_IO_ADDRESS(MX6Q_USB_OTG_BASE_ADDR);
	/* disable external charger detect,
	 * or it will affect signal quality at dp .
	 */
#if 0
	int ret = 0;
	ret = gpio_request(ICORE_M6_USB_OTG_PWR, "usb-pwr");
	if (ret) {
		pr_err("failed to get GPIO ICORE_M6_USB_OTG_PWR: %d\n",
			ret);
		return;
	}
	gpio_direction_output(ICORE_M6_USB_OTG_PWR, 0);
	mxc_iomux_set_gpr_register(1, 13, 1, 1);
#endif
	mx6_set_otghost_vbus_func(imx6q_icore_usbotg_vbus);
//	mx6_usb_dr_init();
}

/* HW Initialization, if return 0, initialization is successful. */
static int mx6q_icore_sata_init(struct device *dev, void __iomem *addr)
{
	u32 tmpdata;
	int ret = 0;
	struct clk *clk;

	sata_clk = clk_get(dev, "imx_sata_clk");
	if (IS_ERR(sata_clk)) {
		dev_err(dev, "no sata clock.\n");
		return PTR_ERR(sata_clk);
	}
	ret = clk_enable(sata_clk);
	if (ret) {
		dev_err(dev, "can't enable sata clock.\n");
		goto put_sata_clk;
	}

	/* Set PHY Paremeters, two steps to configure the GPR13,
	 * one write for rest of parameters, mask of first write is 0x07FFFFFD,
	 * and the other one write for setting the mpll_clk_off_b
	 *.rx_eq_val_0(iomuxc_gpr13[26:24]),
	 *.los_lvl(iomuxc_gpr13[23:19]),
	 *.rx_dpll_mode_0(iomuxc_gpr13[18:16]),
	 *.sata_speed(iomuxc_gpr13[15]),
	 *.mpll_ss_en(iomuxc_gpr13[14]),
	 *.tx_atten_0(iomuxc_gpr13[13:11]),
	 *.tx_boost_0(iomuxc_gpr13[10:7]),
	 *.tx_lvl(iomuxc_gpr13[6:2]),
	 *.mpll_ck_off(iomuxc_gpr13[1]),
	 *.tx_edgerate_0(iomuxc_gpr13[0]),
	 */
	tmpdata = readl(IOMUXC_GPR13);
	writel(((tmpdata & ~0x07FFFFFD) | 0x0593A044), IOMUXC_GPR13);

	/* enable SATA_PHY PLL */
	tmpdata = readl(IOMUXC_GPR13);
	writel(((tmpdata & ~0x2) | 0x2), IOMUXC_GPR13);

	/* Get the AHB clock rate, and configure the TIMER1MS reg later */
	clk = clk_get(NULL, "ahb");
	if (IS_ERR(clk)) {
		dev_err(dev, "no ahb clock.\n");
		ret = PTR_ERR(clk);
		goto release_sata_clk;
	}
	tmpdata = clk_get_rate(clk) / 1000;
	clk_put(clk);

	ret = sata_init(addr, tmpdata);
	if (ret == 0)
		return ret;

release_sata_clk:
	clk_disable(sata_clk);
put_sata_clk:
	clk_put(sata_clk);

	return ret;
}

static void mx6q_icore_sata_exit(struct device *dev)
{
	clk_disable(sata_clk);
	clk_put(sata_clk);
}

static struct ahci_platform_data mx6q_icore_sata_data = {
	.init = mx6q_icore_sata_init,
	.exit = mx6q_icore_sata_exit,
};

static void mx6q_icore_flexcan0_switch(int enable)
{
    return;
/*
	if (enable) {
		gpio_set_value(ICORE_M6_CAN1_EN, 1);
		gpio_set_value(ICORE_M6_CAN1_STBY, 1);
	} else {
		gpio_set_value(ICORE_M6_CAN1_EN, 0);
		gpio_set_value(ICORE_M6_CAN1_STBY, 0);
	}
*/
}


static const struct flexcan_platform_data
	mx6q_icore_flexcan0_pdata __initconst = {
	.transceiver_switch = mx6q_icore_flexcan0_switch,
};

static struct viv_gpu_platform_data imx6q_gpu_pdata __initdata = {
	.reserved_mem_size = SZ_128M,
};

static struct imx_asrc_platform_data imx_asrc_data = {
	.channel_bits = 4,
	.clk_map_ver = 2,
};

static struct ipuv3_fb_platform_data icore_fb_data[] = {
	{ /*fb0*/
	.disp_dev = "ldb",
	.interface_pix_fmt = IPU_PIX_FMT_RGB666,
	.mode_str = "HIT-LVDS",
	.default_bpp = 16,
	.int_clk = false,
	}, {
	.disp_dev = "ldb",
	.interface_pix_fmt = IPU_PIX_FMT_RGB666,
	.mode_str = "HIT-LVDS",
	.default_bpp = 16,
	.int_clk = false,
	}, {
         .disp_dev          = "hdmi",
         .interface_pix_fmt  = IPU_PIX_FMT_RGB24,
         .mode_str                  = "1280x720M@50",
         .default_bpp             = 32,
         .int_clk              = false,
	}, {
	.disp_dev = "lcd",
	.interface_pix_fmt = IPU_PIX_FMT_RGB666,
	.mode_str = "Amp-WD",
	.default_bpp = 18,
	.int_clk = false,
	},
};

static void hdmi_init(int ipu_id, int disp_id)
{
	int hdmi_mux_setting;

	if ((ipu_id > 1) || (ipu_id < 0)) {
		pr_err("Invalid IPU select for HDMI: %d. Set to 0\n", ipu_id);
		ipu_id = 0;
	}

	if ((disp_id > 1) || (disp_id < 0)) {
		pr_err("Invalid DI select for HDMI: %d. Set to 0\n", disp_id);
		disp_id = 0;
	}

	/* Configure the connection between IPU1/2 and HDMI */
	hdmi_mux_setting = 2*ipu_id + disp_id;

	/* GPR3, bits 2-3 = HDMI_MUX_CTL */
	mxc_iomux_set_gpr_register(3, 2, 2, hdmi_mux_setting);

	/* Set HDMI event as SDMA event2 while Chip version later than TO1.2 */
	if (hdmi_SDMA_check())
		mxc_iomux_set_gpr_register(0, 0, 1, 1);
}

static struct fsl_mxc_hdmi_platform_data hdmi_data = {
	.init = hdmi_init,
};

static struct fsl_mxc_hdmi_core_platform_data hdmi_core_data = {
	.ipu_id = 0,
	.disp_id = 0,
};

static struct fsl_mxc_lcd_platform_data lcdif_data = {
	.ipu_id = 0,
	.disp_id = 0,
	.default_ifmt = IPU_PIX_FMT_RGB666,
};

static struct fsl_mxc_ldb_platform_data ldb_data = {
	.ipu_id = 0,
	.disp_id = 0,
	.ext_ref = 1,
	.mode = LDB_SEP1,
	.sec_ipu_id = 0,
	.sec_disp_id = 1,
};

static struct imx_ipuv3_platform_data ipu_data[] = {
	{
		.rev		= 4,
		.csi_clk[0]	= "ccm_clk0",
	}, {
		.rev		= 4,
		.csi_clk[0]	= "ccm_clk0",
	},
};

static struct fsl_mxc_capture_platform_data capture_data[] = {
	{
		.csi = 0,
		.ipu = 0,
		.mclk_source = 0,
		.is_mipi = 0,
	}, 
#if 0
	{
		.csi = 1,
		.ipu = 0,
		.mclk_source = 0,
		.is_mipi = 1,
	},
#endif
};
static void icore_suspend_enter(void)
{
	/* suspend preparation */
}

static void icore_suspend_exit(void)
{
	/* resume restore */
}
static const struct pm_platform_data mx6q_icore_pm_data __initconst = {
	.name = "imx_pm",
	.suspend_enter = icore_suspend_enter,
	.suspend_exit = icore_suspend_exit,
};


static struct regulator_consumer_supply icore_vmmc_consumers[] = {
	REGULATOR_SUPPLY("vmmc", "sdhci-esdhc-imx.0"),
	REGULATOR_SUPPLY("vmmc", "sdhci-esdhc-imx.1"),
	REGULATOR_SUPPLY("vmmc", "sdhci-esdhc-imx.2"),
	REGULATOR_SUPPLY("vmmc", "sdhci-esdhc-imx.3"),
};

static struct regulator_init_data icore_vmmc_init = {
	.num_consumer_supplies = ARRAY_SIZE(icore_vmmc_consumers),
	.consumer_supplies = icore_vmmc_consumers,
};

static struct fixed_voltage_config icore_vmmc_reg_config = {
	.supply_name		= "vmmc",
	.microvolts		= 3300000,
	.gpio			= -1,
	.init_data		= &icore_vmmc_init,
};

static struct platform_device icore_vmmc_reg_devices = {
	.name	= "reg-fixed-voltage",
	.id	= 3,
	.dev	= {
		.platform_data = &icore_vmmc_reg_config,
	},
};


#ifdef CONFIG_SND_SOC_SGTL5000

static struct regulator_consumer_supply sgtl5000_icore_consumer_vdda = {
	.supply = "VDDA",
	.dev_name = "2-000a",
};

static struct regulator_consumer_supply sgtl5000_icore_consumer_vddio = {
	.supply = "VDDIO",
	.dev_name = "2-000a",
};

static struct regulator_consumer_supply sgtl5000_icore_consumer_vddd = {
	.supply = "VDDD",
	.dev_name = "2-000a",
};

static struct regulator_init_data sgtl5000_icore_vdda_reg_initdata = {
	.num_consumer_supplies = 1,
	.consumer_supplies = &sgtl5000_icore_consumer_vdda,
};

static struct regulator_init_data sgtl5000_icore_vddio_reg_initdata = {
	.num_consumer_supplies = 1,
	.consumer_supplies = &sgtl5000_icore_consumer_vddio,
};

static struct regulator_init_data sgtl5000_icore_vddd_reg_initdata = {
	.num_consumer_supplies = 1,
	.consumer_supplies = &sgtl5000_icore_consumer_vddd,
};

static struct fixed_voltage_config sgtl5000_icore_vdda_reg_config = {
	.supply_name		= "VDDA",
	.microvolts		= 3300000,
	.gpio			= -1,
	.init_data		= &sgtl5000_icore_vdda_reg_initdata,
};

static struct fixed_voltage_config sgtl5000_icore_vddio_reg_config = {
	.supply_name		= "VDDIO",
	.microvolts		= 3300000,
	.gpio			= -1,
	.init_data		= &sgtl5000_icore_vddio_reg_initdata,
};

static struct fixed_voltage_config sgtl5000_icore_vddd_reg_config = {
	.supply_name		= "VDDD",
	.microvolts		= 1200000,
	.gpio			= -1,
	.init_data		= &sgtl5000_icore_vddd_reg_initdata,
};

static struct platform_device sgtl5000_icore_vdda_reg_devices = {
	.name	= "reg-fixed-voltage",
	.id	= 0,
	.dev	= {
		.platform_data = &sgtl5000_icore_vdda_reg_config,
	},
};

static struct platform_device sgtl5000_icore_vddio_reg_devices = {
	.name	= "reg-fixed-voltage",
	.id	= 1,
	.dev	= {
		.platform_data = &sgtl5000_icore_vddio_reg_config,
	},
};

static struct platform_device sgtl5000_icore_vddd_reg_devices = {
	.name	= "reg-fixed-voltage",
	.id	= 2,
	.dev	= {
		.platform_data = &sgtl5000_icore_vddd_reg_config,
	},
};

#endif /* CONFIG_SND_SOC_SGTL5000 */

static int imx6q_init_audio(void)
{
	mxc_register_device(&mx6_icore_audio_device,
			    &mx6_icore_audio_data);
	imx6q_add_imx_ssi(1, &mx6_icore_ssi_pdata);
#ifdef CONFIG_SND_SOC_SGTL5000
	platform_device_register(&sgtl5000_icore_vdda_reg_devices);
	platform_device_register(&sgtl5000_icore_vddio_reg_devices);
	platform_device_register(&sgtl5000_icore_vddd_reg_devices);
#endif
	return 0;
}


static struct platform_pwm_backlight_data mx6_icore_pwm0_backlight_data = {
	.pwm_id = 2,
	.max_brightness = 255,
	.dft_brightness = 255,
	.pwm_period_ns = 100000,
};

static struct mxc_dvfs_platform_data icore_dvfscore_data = {
	.reg_id = "cpu_vddgp",
	.clk1_id = "cpu_clk",
	.clk2_id = "gpc_dvfs_clk",
	.gpc_cntr_offset = MXC_GPC_CNTR_OFFSET,
	.ccm_cdcr_offset = MXC_CCM_CDCR_OFFSET,
	.ccm_cacrr_offset = MXC_CCM_CACRR_OFFSET,
	.ccm_cdhipr_offset = MXC_CCM_CDHIPR_OFFSET,
	.prediv_mask = 0x1F800,
	.prediv_offset = 11,
	.prediv_val = 3,
	.div3ck_mask = 0xE0000000,
	.div3ck_offset = 29,
	.div3ck_val = 2,
	.emac_val = 0x08,
	.upthr_val = 25,
	.dnthr_val = 9,
	.pncthr_val = 33,
	.upcnt_val = 10,
	.dncnt_val = 10,
	.delay_time = 80,
};

static void __init fixup_mxc_board(struct machine_desc *desc, struct tag *tags,
				   char **cmdline, struct meminfo *mi)
{
}

static const struct imx_pcie_platform_data pcie_data  __initconst = {
	.pcie_pwr_en	= -EINVAL,
	.pcie_rst	= -EINVAL, //ICORE_M6_CAP_TCH_INT1,
	.pcie_wake_up	= -EINVAL,
	.pcie_dis	= -EINVAL,
};

static inline void __init mx6q_csi0_io_init(void)
{
	if (cpu_is_mx6q())
		mxc_iomux_set_gpr_register(1, 19, 1, 1);
	else if (cpu_is_mx6dl())
		mxc_iomux_set_gpr_register(13, 0, 3, 4);
}

/*!
 * Board specific initialization of pads control
 */
static void icore_customized_board_init (void)
{
	printk("%s selected.", engi_board_description_str[engi_board] );	
	
	if (cpu_is_mx6q())
	{
		switch(engi_board)
		{
			case ENGICAM_RESISTIVE_SK:
				mxc_iomux_v3_setup_multiple_pads(mx6q_icore_pads_resistive_sk, ARRAY_SIZE(mx6q_icore_pads_resistive_sk));
			break;

			case ENGICAM_RESISTIVE_OF:
				mxc_iomux_v3_setup_multiple_pads(mx6q_icore_pads_resistive_of, ARRAY_SIZE(mx6q_icore_pads_resistive_of));
			break;

			case ENGICAM_CAPACITIVE_OF:
				mxc_iomux_v3_setup_multiple_pads(mx6q_icore_pads_capacitive_of, ARRAY_SIZE(mx6q_icore_pads_capacitive_of));
			break;

			case ENGICAM_CAPACITIVE_OF_AMP:
				mxc_iomux_v3_setup_multiple_pads(mx6q_icore_pads_capacitive_ofamp, ARRAY_SIZE(mx6q_icore_pads_capacitive_ofamp));
			break;

			case ENGICAM_CAPACITIVE_SK:
				mxc_iomux_v3_setup_multiple_pads(mx6q_icore_pads_capacitive_sk, ARRAY_SIZE(mx6q_icore_pads_capacitive_sk));
			break;

			case ENGICAM_MEDIAPLAYER_V0:
				mxc_iomux_v3_setup_multiple_pads(mx6q_icore_pads_mediaplayer, ARRAY_SIZE(mx6q_icore_pads_mediaplayer));

			case ENGICAM_CAPACITIVE_OF_URT:
				mxc_iomux_v3_setup_multiple_pads(mx6q_icore_pads_capacitive_of_urt, ARRAY_SIZE(mx6q_icore_pads_capacitive_of_urt));
			break;
		}
	}
	else
	{
		switch(engi_board)
		{
			case ENGICAM_RESISTIVE_SK:
				mxc_iomux_v3_setup_multiple_pads(mx6dl_icore_pads_resistive_sk, ARRAY_SIZE(mx6dl_icore_pads_resistive_sk));
			break;

			case ENGICAM_RESISTIVE_OF:
				mxc_iomux_v3_setup_multiple_pads(mx6dl_icore_pads_resistive_of, ARRAY_SIZE(mx6dl_icore_pads_resistive_of));
			break;

			case ENGICAM_CAPACITIVE_OF:
				mxc_iomux_v3_setup_multiple_pads(mx6dl_icore_pads_capacitive_of, ARRAY_SIZE(mx6dl_icore_pads_capacitive_of));
			break;

			case ENGICAM_CAPACITIVE_OF_AMP:
				mxc_iomux_v3_setup_multiple_pads(mx6dl_icore_pads_capacitive_ofamp, ARRAY_SIZE(mx6dl_icore_pads_capacitive_ofamp));
			break;

			case ENGICAM_CAPACITIVE_SK:
				mxc_iomux_v3_setup_multiple_pads(mx6dl_icore_pads_capacitive_sk, ARRAY_SIZE(mx6dl_icore_pads_capacitive_sk));
			break;

			case ENGICAM_MEDIAPLAYER_V0:
				mxc_iomux_v3_setup_multiple_pads(mx6dl_icore_pads_mediaplayer, ARRAY_SIZE(mx6dl_icore_pads_mediaplayer));
			break;

			case ENGICAM_CAPACITIVE_OF_URT:
				mxc_iomux_v3_setup_multiple_pads(mx6dl_icore_pads_capacitive_of_urt, ARRAY_SIZE(mx6dl_icore_pads_capacitive_of_urt));
			break;
		}
	}
}

/*!
 * Sepcific inizialization for module revision
 */
void icore_customized_version_init (void)
{
	printk("Module revision %s.\n", icore_module_vers_str[icore_mudule_vers]);	
	
	if (cpu_is_mx6q())
	{
		switch(icore_mudule_vers)
		{
			case ICORE_VERS_MINOR_D:
				mxc_iomux_v3_setup_multiple_pads(mx6q_icore_pads_minor_D, ARRAY_SIZE(mx6q_icore_pads_minor_D));

				if(engi_board == ENGICAM_CAPACITIVE_OF) // For openframe capacitive forced to inizialize USB OTG on host device
					mxc_iomux_v3_setup_multiple_pads(mx6q_icore_pads_forced_of_cap, ARRAY_SIZE(mx6q_icore_pads_forced_of_cap));
			break;

			case ICORE_VERS_GREATER_D:
				mxc_iomux_v3_setup_multiple_pads(mx6q_icore_pads_greater_D, ARRAY_SIZE(mx6q_icore_pads_greater_D));
			break;
		}
	}
	else
	{
		switch(icore_mudule_vers)
		{
			case ICORE_VERS_MINOR_D:
				mxc_iomux_v3_setup_multiple_pads(mx6dl_icore_pads_minor_D, ARRAY_SIZE(mx6dl_icore_pads_minor_D));

				if(engi_board == ENGICAM_CAPACITIVE_OF) // For openframe capacitive forced to inizialize USB OTG on host device
					mxc_iomux_v3_setup_multiple_pads(mx6dl_icore_pads_forced_of_cap, ARRAY_SIZE(mx6dl_icore_pads_forced_of_cap));
			break;

			case ICORE_VERS_GREATER_D:
				mxc_iomux_v3_setup_multiple_pads(mx6dl_icore_pads_greater_D, ARRAY_SIZE(mx6dl_icore_pads_greater_D));
			break;
		}
	}	

}

/*!
 * Board specific initialization of i2c channels
 */
static void icore_customized_i2c_init (void)
{
	switch(engi_board)
	{
		case ENGICAM_RESISTIVE_SK:
			i2c_register_board_info(2, mxc_i2c2_board_info_skres,	ARRAY_SIZE(mxc_i2c2_board_info_skres));
			i2c_register_board_info(0, mxc_i2c0_max11801_info,	ARRAY_SIZE(mxc_i2c0_max11801_info));
		break;

		case ENGICAM_RESISTIVE_OF:
			i2c_register_board_info(2, mxc_i2c2_board_info_ofres,	ARRAY_SIZE(mxc_i2c2_board_info_ofres));
			i2c_register_board_info(0, mxc_i2c0_max11801_info,	ARRAY_SIZE(mxc_i2c0_max11801_info));
		break;

		case ENGICAM_CAPACITIVE_OF:
			i2c_register_board_info(2, mxc_i2c2_board_info_ofcap,	ARRAY_SIZE(mxc_i2c2_board_info_ofcap));
		break;

		case ENGICAM_CAPACITIVE_OF_AMP:
			i2c_register_board_info(2, mxc_i2c2_board_info_ofamp,	ARRAY_SIZE(mxc_i2c2_board_info_ofamp));
		break;

		case ENGICAM_CAPACITIVE_SK:
			i2c_register_board_info(2, mxc_i2c2_board_info_skcap,	ARRAY_SIZE(mxc_i2c2_board_info_skcap));
		break;

		case ENGICAM_MEDIAPLAYER_V0:
			i2c_register_board_info(2, mxc_i2c2_board_info_mediaplayer,	ARRAY_SIZE(mxc_i2c2_board_info_mediaplayer));
		break;

		case ENGICAM_CAPACITIVE_OF_URT:
			i2c_register_board_info(2, mxc_i2c2_board_info_of_urt,	ARRAY_SIZE(mxc_i2c2_board_info_of_urt));
		break;

	}
}

/*!
 * Board specific initialization.
 */
static void __init mx6_icore_board_init(void)
{
	int i;
	struct clk *clko2;
	struct clk *new_parent;
	int rate;

	if (cpu_is_mx6q()) 
	{
		printk(KERN_ERR "------------ Board type %s\n", "i.Core M6Q/D based");
		mxc_iomux_v3_setup_multiple_pads(mx6q_icore_pads, ARRAY_SIZE(mx6q_icore_pads));
	}
	else	
	{
		printk(KERN_ERR "------------ Board type %s\n", "i.Core M6DL/S based");
		mxc_iomux_v3_setup_multiple_pads(mx6dl_icore_pads, ARRAY_SIZE(mx6dl_icore_pads));
		
		if(cpu_is_mx6dl())
			printk(KERN_ERR "Test cpu_is_mx6dl PASSED\n");
		else
			printk(KERN_ERR "Test cpu_is_mx6dl FAILED\n");
	}

	icore_init_module_version();
	icore_customized_version_init();
	icore_customized_board_init();

	#ifdef CONFIG_MACH_MX6Q_MINIMUM_FREQ400
	printk("CPU Minum freq forced to 400 Mhz.\n");
	#endif

#ifdef CONFIG_FEC_1588
	/* Set GPIO_16 input for IEEE-1588 ts_clk and RMII reference clock
	 * For MX6 GPR1 bit21 meaning:
	 * Bit21:       0 - GPIO_16 pad output
	 *              1 - GPIO_16 pad input
	 */
	mxc_iomux_set_gpr_register(1, 21, 1, 1);
#endif

	gp_reg_id = icore_dvfscore_data.reg_id;
	soc_reg_id = icore_dvfscore_data.soc_id;
	pu_reg_id = icore_dvfscore_data.pu_id;

	mx6q_icore_init_uart();

	if (!cpu_is_mx6dl()) {
		ldb_data.ipu_id = 1;
		ldb_data.disp_id = 0;
		ldb_data.sec_ipu_id = 1;
		ldb_data.sec_disp_id = 1;
		hdmi_core_data.disp_id = 1;
	}

	// For compatibility with previus kernel version
	if(engi_board==ENGICAM_CAPACITIVE_OF || engi_board==ENGICAM_CAPACITIVE_OF_AMP)
	{
		ldb_data.mode = LDB_SIN0;
	}

	imx6q_add_mxc_hdmi_core(&hdmi_core_data);

	imx6q_add_ipuv3(0, &ipu_data[0]);
	if (cpu_is_mx6q()) {
		imx6q_add_ipuv3(1, &ipu_data[1]);
		for (i = 0; i < ARRAY_SIZE(icore_fb_data); i++)
			imx6q_add_ipuv3fb(i, &icore_fb_data[i]);
	} else 
		for (i = 0; i < (ARRAY_SIZE(icore_fb_data) + 1) / 2; i++)
			imx6q_add_ipuv3fb(i, &icore_fb_data[i]);


	imx6q_add_vdoa();
	imx6q_add_lcdif(&lcdif_data);
	imx6q_add_ldb(&ldb_data);
	imx6q_add_v4l2_output(0);
	imx6q_add_v4l2_capture(0, &capture_data[0]);
//	imx6q_add_v4l2_capture(1, &capture_data[1]);

	imx6q_add_imx_snvs_rtc();

//	imx6q_add_imx_caam(); //TOLTA per conflitto I2C3

	imx6q_add_imx_i2c(0, &mx6q_icore_i2c_data);
	imx6q_add_imx_i2c(1, &mx6q_icore_i2c_data);
	imx6q_add_imx_i2c(2, &mx6q_icore_i2c_data);

	i2c_register_board_info(0, mxc_i2c0_board_info,
			ARRAY_SIZE(mxc_i2c0_board_info));
	i2c_register_board_info(1, mxc_i2c1_board_info,
			ARRAY_SIZE(mxc_i2c1_board_info));
	i2c_register_board_info(2, mxc_i2c2_board_info,
			ARRAY_SIZE(mxc_i2c2_board_info));

	icore_customized_i2c_init();

	/* SPI */
//	imx6q_add_ecspi(0, &mx6q_icore_spi_data);
//	spi_device_init();

	imx6q_add_mxc_hdmi(&hdmi_data);

	imx6q_add_anatop_thermal_imx(1, &mx6q_icore_anatop_thermal_data);
	imx6_init_fec(fec_data);
	imx6q_add_pm_imx(0, &mx6q_icore_pm_data);
	imx6q_add_sdhci_usdhc_imx(0, &mx6q_icore_sd1_data);
	imx6q_add_sdhci_usdhc_imx(1, &mx6q_icore_sd2_data);
//	imx6q_add_sdhci_usdhc_imx(3, &mx6q_icore_sd4_data);

	if (!cpu_is_mx6q())		// i.Core M6Solo con 256MB RAM
		imx6q_gpu_pdata.reserved_mem_size = SZ_32M;

	imx_add_viv_gpu(&imx6_gpu_data, &imx6q_gpu_pdata);
	imx6q_icore_init_usb();
	if (cpu_is_mx6q())
		imx6q_add_ahci(0, &mx6q_icore_sata_data);
	imx6q_add_vpu();
	imx6q_init_audio();
	platform_device_register(&icore_vmmc_reg_devices);
	imx_asrc_data.asrc_core_clk = clk_get(NULL, "asrc_clk");
	imx_asrc_data.asrc_audio_clk = clk_get(NULL, "asrc_serial_clk");
	imx6q_add_asrc(&imx_asrc_data);

	/* release USB Hub reset */
#if 0
	gpio_set_value(ICORE_M6_USB_HUB_RESET, 1);

	imx6q_add_mxc_pwm(0);
	imx6q_add_mxc_pwm(1);
	imx6q_add_mxc_pwm(2);
	imx6q_add_mxc_pwm(3);
	imx6q_add_mxc_pwm_backlight(0, &mx6_icore_pwm0_backlight_data);
	imx6q_add_mxc_pwm_backlight(3, &mx6_icore_pwm_backlight_data);
#endif

	gpio_request(ICORE_BACKLIGHT3, "backlight-pwm3");
	gpio_direction_output(ICORE_BACKLIGHT3, 0);

	imx6q_add_mxc_pwm(2);
	imx6q_add_mxc_pwm_backlight(0, &mx6_icore_pwm0_backlight_data);

	imx6q_add_otp();
	imx6q_add_viim();
	imx6q_add_imx2_wdt(0, NULL);
	imx6q_add_dma();

	imx6q_add_dvfs_core(&icore_dvfscore_data);
	mx6_cpu_regulator_init();

//	icore_add_device_buttons();

	imx6q_add_hdmi_soc();
	imx6q_add_hdmi_soc_dai();

	imx6q_add_flexcan0(&mx6q_icore_flexcan0_pdata);
	imx6q_add_flexcan1(&mx6q_icore_flexcan0_pdata);

	clko2 = clk_get(NULL, "clko2_clk");
	if (IS_ERR(clko2))
		pr_err("can't get CLKO2 clock.\n");

	new_parent = clk_get(NULL, "osc_clk");
	if (!IS_ERR(new_parent)) {
		clk_set_parent(clko2, new_parent);
		clk_put(new_parent);
	}
	rate = clk_round_rate(clko2, 24000000);
	clk_set_rate(clko2, rate);
	clk_enable(clko2);
	imx6q_add_busfreq();

	imx6q_add_pcie(&pcie_data);

	imx6q_add_perfmon(0);
	imx6q_add_perfmon(1);
	imx6q_add_perfmon(2);

	imx6q_add_gpmi(&mx6q_gpmi_nand_platform_data);
	mx6q_csi0_io_init();

	switch(engi_board)
	{
		case ENGICAM_MEDIAPLAYER_V0:
			gpio_set_value(ICORE_M6_WF111_RESET_MP,1);
			break;
		case ENGICAM_CAPACITIVE_OF_URT:
			gpio_set_value(ICORE_M6_WF111_RESET_URT,1);
			break;	
		default:
			gpio_set_value(ICORE_M6_WF111_RESET_SK,1);
	}

	
	#ifdef CONFIG_SERIAL_RS485_ENABLE
	gpio_request(UART3_CS485, "UART3_CS485");
	gpio_direction_output(UART3_CS485, 0);
	gpio_set_value(UART3_CS485, 0);
	#endif

	// Init LVDS for openframe capacitive
	if(engi_board==ENGICAM_CAPACITIVE_OF || engi_board==ENGICAM_CAPACITIVE_OF_AMP)
	{
		gpio_request(OFC_LVDS_ENABLE, "OFC_LVDS_ENABLE");
		gpio_direction_output(OFC_LVDS_ENABLE, 0);
		gpio_set_value(OFC_LVDS_ENABLE, 0);
		gpio_free(OFC_LVDS_ENABLE);
	}

}


extern void __iomem *twd_base;
static void __init mx6_icore_timer_init(void)
{
	struct clk *uart_clk;
#ifdef CONFIG_LOCAL_TIMERS
	twd_base = ioremap(LOCAL_TWD_ADDR, SZ_256);
	BUG_ON(!twd_base);
#endif
	mx6_clocks_init(32768, 24000000, 0, 0);

	uart_clk = clk_get_sys("imx-uart.3", NULL);
	early_console_setup(UART4_BASE_ADDR, uart_clk);
}

static struct sys_timer mx6_icore_timer = {
	.init   = mx6_icore_timer_init,
};

static void __init mx6q_icore_reserve(void)
{
	phys_addr_t phys;

	mx6q_icore_check_ram_size();

	if (iRamMemorySize < 512)
		imx6q_gpu_pdata.reserved_mem_size = SZ_32M;
	
	if (imx6q_gpu_pdata.reserved_mem_size) {
		phys = memblock_alloc_base(imx6q_gpu_pdata.reserved_mem_size,
					   SZ_4K, SZ_1G);
		memblock_free(phys, imx6q_gpu_pdata.reserved_mem_size);
		memblock_remove(phys, imx6q_gpu_pdata.reserved_mem_size);
		imx6q_gpu_pdata.reserved_mem_base = phys;
	}
}

/* Calculate on startup time the RAM memory size for be 
*  used during initialization process
*/
static void mx6q_icore_check_ram_size (void)
{
	struct memblock_region *reg;
	 /* Since our memory may not be contiguous, calculate the
	 * real number of pages we have in this system
	 */
	num_physpages = 0;
	for_each_memblock(memory, reg) {
		unsigned long pages = memblock_region_memory_end_pfn(reg) -
			memblock_region_memory_base_pfn(reg);
		num_physpages += pages;
	}
	iRamMemorySize = num_physpages >> (20 - PAGE_SHIFT);
}

/*
 * Set the LVDS power on control
 */
void mx6q_icore_lvds_power(bool bStatus)
{
	if(engi_board==ENGICAM_RESISTIVE_OF)
	{
		static bool bFirstTime=true;
		static bool bPreviusStatus=false;

		// If there is no change with the previus status the fuction end
		if(bPreviusStatus==bStatus && bFirstTime==false)
	 		return;
		bFirstTime=false;
		bPreviusStatus=bStatus;

		if (cpu_is_mx6q())
			mxc_iomux_v3_setup_pad(MX6Q_PAD_CSI0_DAT14__GPIO_6_0);
		else
			mxc_iomux_v3_setup_pad(MX6DL_PAD_CSI0_DAT14__GPIO_6_0);
		gpio_request(ICORE_M6_OF_LVDS_RESET, "LVDS_OF_RESET");
		gpio_direction_output(ICORE_M6_OF_LVDS_RESET, 0);
		if(bStatus)
		{
			mdelay(30);
			gpio_set_value(ICORE_M6_OF_LVDS_RESET, 1);	
		}
		else
		{
			gpio_set_value(ICORE_M6_OF_LVDS_RESET, 0);
		}
		gpio_free(ICORE_M6_OF_LVDS_RESET);
	}
}

EXPORT_SYMBOL(mx6q_icore_lvds_power);

/*
 * initialize __mach_desc_MX6Q_SABRELITE data structure.
 */
MACHINE_START(MX6Q_SABRELITE, "Engicam i.Core based Board")
	/* Maintainer: Freescale Semiconductor, Inc. */
	.boot_params = MX6_PHYS_OFFSET + 0x100,
	.fixup = fixup_mxc_board,
	.map_io = mx6_map_io,
	.init_irq = mx6_init_irq,
	.init_machine = mx6_icore_board_init,
	.timer = &mx6_icore_timer,
	.reserve = mx6q_icore_reserve,
MACHINE_END
