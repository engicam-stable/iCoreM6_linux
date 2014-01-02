/*
 * Copyright (C) 2011-2013 Engicam srl All Rights Reserved.
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

#define ICORE_M6_SD1_CD		IMX_GPIO_NR(1, 1)

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

#define ICORE_M6_SD3_WP_PADCFG	(PAD_CTL_PKE | PAD_CTL_PUE |	\
		PAD_CTL_PUS_22K_UP | PAD_CTL_SPEED_MED |	\
		PAD_CTL_DSE_40ohm | PAD_CTL_HYS)

#ifdef CONFIG_MX6_ENET_IRQ_TO_GPIO
#define MX6_ENET_IRQ		IMX_GPIO_NR(1, 6)
#define IOMUX_OBSRV_MUX1_OFFSET	0x3c
#define OBSRV_MUX1_MASK			0x3f
#define OBSRV_MUX1_ENET_IRQ		0x9
#endif

enum engicam_board
{
        ENGICAM_Q7_SK=0,

        ENGICAM_LAST_BOARD
};

#define ENGICAM_DEFAULT_BOARD 		ENGICAM_Q7_SK

static char* engi_board_str[] =
{
        "SK.Q7",

        /* add here a new board */
};

static char* engi_board_description_str[] =
{
        "Engicam QSEVEN Starterkit",

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

/* Conatins the correct value of RAM memory size. The complete value is
*  available  only after calling the fuction mx6q_icore_check_ram_size 
*  prior to reszerve any memory areas.
*/
static int iRamMemorySize=0;

void __init early_console_setup(unsigned long base, struct clk *clk);
static void mx6q_icore_check_ram_size (void);
static struct clk *sata_clk;

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

	MX6Q_PAD_GPIO_4__GPIO_1_4,		/* SD3 nPWR Enable */

	/* CCM  */
	MX6Q_PAD_GPIO_0__CCM_CLKO,		/* SGTL500 sys_mclk */
//	MX6Q_PAD_GPIO_3__CCM_CLKO2,		/* J5 - Camera MCLK */


	/* ENET */
	MX6Q_PAD_ENET_MDIO__ENET_MDIO,
	MX6Q_PAD_ENET_MDC__ENET_MDC,
	MX6Q_PAD_RGMII_TXC__ENET_RGMII_TXC,
	MX6Q_PAD_RGMII_TD0__ENET_RGMII_TD0,
	MX6Q_PAD_RGMII_TD1__ENET_RGMII_TD1,
	MX6Q_PAD_RGMII_TD2__ENET_RGMII_TD2,
	MX6Q_PAD_RGMII_TD3__ENET_RGMII_TD3,
	MX6Q_PAD_RGMII_TX_CTL__ENET_RGMII_TX_CTL,
	MX6Q_PAD_ENET_REF_CLK__ENET_TX_CLK,
	MX6Q_PAD_RGMII_RXC__ENET_RGMII_RXC,
	MX6Q_PAD_RGMII_RD0__ENET_RGMII_RD0,
	MX6Q_PAD_RGMII_RD1__ENET_RGMII_RD1,
	MX6Q_PAD_RGMII_RD2__ENET_RGMII_RD2,
	MX6Q_PAD_RGMII_RD3__ENET_RGMII_RD3,
	MX6Q_PAD_RGMII_RX_CTL__ENET_RGMII_RX_CTL,
	MX6Q_PAD_ENET_TX_EN__GPIO_1_28,		/* Micrel RGMII Phy Interrupt */
	MX6Q_PAD_ENET_RX_ER__GPIO_1_24,		/* RGMII reset */


	/* I2C1*/
	MX6Q_PAD_EIM_D21__I2C1_SCL,	/* GPIO3[21] */
	MX6Q_PAD_EIM_D28__I2C1_SDA,	/* GPIO3[28] */

	/* I2C2 */
	MX6Q_PAD_KEY_COL3__I2C2_SCL,	/* GPIO4[12] */
	MX6Q_PAD_KEY_ROW3__I2C2_SDA,	/* GPIO4[13] */

	/* I2C3 */
	MX6Q_PAD_GPIO_5__I2C3_SCL,	
	MX6Q_PAD_EIM_D18__I2C3_SDA,	
 
	MX6Q_PAD_DI0_PIN4__IPU1_DI0_PIN4,		/* Contrast */
	
	/* SPI */
	MX6Q_PAD_DISP0_DAT0__ECSPI3_SCLK,
	MX6Q_PAD_DISP0_DAT1__ECSPI3_MOSI,
	MX6Q_PAD_DISP0_DAT2__ECSPI3_MISO,
	MX6Q_PAD_DISP0_DAT3__ECSPI3_SS0,
	MX6Q_PAD_DISP0_DAT4__ECSPI3_SS1,

	/* PWMs */
	MX6Q_PAD_DISP0_DAT8__PWM1_PWMO,
	MX6Q_PAD_DISP0_DAT9__PWM2_PWMO,


	/* AUDIO */
	MX6Q_PAD_DISP0_DAT20__AUDMUX_AUD4_TXC,
	MX6Q_PAD_DISP0_DAT21__AUDMUX_AUD4_TXD,
	MX6Q_PAD_DISP0_DAT22__AUDMUX_AUD4_TXFS,
	MX6Q_PAD_DISP0_DAT23__AUDMUX_AUD4_RXD,

//	MX6Q_PAD_NANDF_D0__GPIO_2_0,		/* J6 - LVDS Display contrast */

	/* UART1  */
	MX6Q_PAD_SD3_DAT7__UART1_TXD,
	MX6Q_PAD_SD3_DAT6__UART1_RXD,

	/* UART2*/
	MX6Q_PAD_EIM_D26__UART2_TXD,
	MX6Q_PAD_EIM_D27__UART2_RXD,

	/* UART4*/
	MX6Q_PAD_KEY_COL0__UART4_TXD,
	MX6Q_PAD_KEY_ROW0__UART4_RXD,


	/* USB OC pin */
	MX6Q_PAD_KEY_COL4__USBOH3_USBOTG_OC,


	/* USDHC1 */
	MX6Q_PAD_SD1_CLK__USDHC1_CLK,
	MX6Q_PAD_SD1_CMD__USDHC1_CMD,
	MX6Q_PAD_SD1_DAT0__USDHC1_DAT0,
	MX6Q_PAD_SD1_DAT1__USDHC1_DAT1,
	MX6Q_PAD_SD1_DAT2__USDHC1_DAT2,
	MX6Q_PAD_SD1_DAT3__USDHC1_DAT3,
	MX6Q_PAD_GPIO_1__GPIO_1_1,		/* SD1_CD */


	/* USDHC3 */
	MX6Q_PAD_SD3_CLK__USDHC3_CLK_50MHZ,
	MX6Q_PAD_SD3_CMD__USDHC3_CMD_50MHZ,
	MX6Q_PAD_SD3_DAT0__USDHC3_DAT0_50MHZ,
	MX6Q_PAD_SD3_DAT1__USDHC3_DAT1_50MHZ,
	MX6Q_PAD_SD3_DAT2__USDHC3_DAT2_50MHZ,
	MX6Q_PAD_SD3_DAT3__USDHC3_DAT3_50MHZ,
	MX6Q_PAD_GPIO_1__GPIO_1_1,		/* SD3_CD */
//	NEW_PAD_CTRL(MX6Q_PAD_SD3_DAT4__GPIO_7_1, MX6Q_SABRELITE_SD3_WP_PADCFG),

	/* USDHC4 */
	MX6Q_PAD_SD4_CLK__USDHC4_CLK_50MHZ,
	MX6Q_PAD_SD4_CMD__USDHC4_CMD_50MHZ,
	MX6Q_PAD_SD4_DAT0__USDHC4_DAT0_50MHZ,
	MX6Q_PAD_SD4_DAT1__USDHC4_DAT1_50MHZ,
	MX6Q_PAD_SD4_DAT2__USDHC4_DAT2_50MHZ,
	MX6Q_PAD_SD4_DAT3__USDHC4_DAT3_50MHZ,
	MX6Q_PAD_SD4_DAT4__USDHC4_DAT4_50MHZ,
	MX6Q_PAD_SD4_DAT5__USDHC4_DAT5_50MHZ,
	MX6Q_PAD_SD4_DAT6__USDHC4_DAT6_50MHZ,
	MX6Q_PAD_SD4_DAT7__USDHC4_DAT7_50MHZ,
//	MX6Q_PAD_NANDF_D6__GPIO_2_6,		/* J20 - SD4_CD */
//	MX6Q_PAD_NANDF_D7__GPIO_2_7,		/* SD4_WP */





#ifdef CONFIG_MX6_ENET_IRQ_TO_GPIO
	MX6Q_PAD_GPIO_6__OBSERVE_MUX_OBSRV_INT_OUT1,
#else
#endif

};

/* Engicam board pin initialization for mx6q*/

static iomux_v3_cfg_t mx6q_rqs_pads_sk[] =
{

};

static iomux_v3_cfg_t mx6dl_icore_pads[] = {
	/* CAN1  */
	MX6DL_PAD_KEY_ROW2__CAN1_RXCAN,
	MX6DL_PAD_KEY_COL2__CAN1_TXCAN,

	MX6DL_PAD_GPIO_4__GPIO_1_4,		/* SD3 nPWR Enable */

	/* CCM  */
	MX6DL_PAD_GPIO_0__CCM_CLKO,		/* SGTL500 sys_mclk */
//	MX6DL_PAD_GPIO_3__CCM_CLKO2,		/* J5 - Camera MCLK */


	/* ENET */
	MX6DL_PAD_ENET_MDIO__ENET_MDIO,
	MX6DL_PAD_ENET_MDC__ENET_MDC,
	MX6DL_PAD_RGMII_TXC__ENET_RGMII_TXC,
	MX6DL_PAD_RGMII_TD0__ENET_RGMII_TD0,
	MX6DL_PAD_RGMII_TD1__ENET_RGMII_TD1,
	MX6DL_PAD_RGMII_TD2__ENET_RGMII_TD2,
	MX6DL_PAD_RGMII_TD3__ENET_RGMII_TD3,
	MX6DL_PAD_RGMII_TX_CTL__ENET_RGMII_TX_CTL,
	MX6DL_PAD_ENET_REF_CLK__ENET_TX_CLK,
	MX6DL_PAD_RGMII_RXC__ENET_RGMII_RXC,
	MX6DL_PAD_RGMII_RD0__ENET_RGMII_RD0,
	MX6DL_PAD_RGMII_RD1__ENET_RGMII_RD1,
	MX6DL_PAD_RGMII_RD2__ENET_RGMII_RD2,
	MX6DL_PAD_RGMII_RD3__ENET_RGMII_RD3,
	MX6DL_PAD_RGMII_RX_CTL__ENET_RGMII_RX_CTL,
	MX6DL_PAD_ENET_TX_EN__GPIO_1_28,		/* Micrel RGMII Phy Interrupt */
	MX6DL_PAD_ENET_RX_ER__GPIO_1_24,		/* RGMII reset */


	/* I2C1*/
	MX6DL_PAD_EIM_D21__I2C1_SCL,	/* GPIO3[21] */
	MX6DL_PAD_EIM_D28__I2C1_SDA,	/* GPIO3[28] */

	/* I2C2 */
	MX6DL_PAD_KEY_COL3__I2C2_SCL,	/* GPIO4[12] */
	MX6DL_PAD_KEY_ROW3__I2C2_SDA,	/* GPIO4[13] */

	/* I2C3 */
	MX6DL_PAD_GPIO_5__I2C3_SCL,	
	MX6DL_PAD_EIM_D18__I2C3_SDA,	
 
	MX6DL_PAD_DI0_PIN4__IPU1_DI0_PIN4,		/* Contrast */
	
	/* SPI */
	MX6DL_PAD_DISP0_DAT0__ECSPI3_SCLK,
	MX6DL_PAD_DISP0_DAT1__ECSPI3_MOSI,
	MX6DL_PAD_DISP0_DAT2__ECSPI3_MISO,
	MX6DL_PAD_DISP0_DAT3__ECSPI3_SS0,
	MX6DL_PAD_DISP0_DAT4__ECSPI3_SS1,

	/* PWMs */
	MX6DL_PAD_DISP0_DAT8__PWM1_PWMO,
	MX6DL_PAD_DISP0_DAT9__PWM2_PWMO,


	/* AUDIO */
	MX6DL_PAD_DISP0_DAT20__AUDMUX_AUD4_TXC,
	MX6DL_PAD_DISP0_DAT21__AUDMUX_AUD4_TXD,
	MX6DL_PAD_DISP0_DAT22__AUDMUX_AUD4_TXFS,
	MX6DL_PAD_DISP0_DAT23__AUDMUX_AUD4_RXD,

//	MX6DL_PAD_NANDF_D0__GPIO_2_0,		/* J6 - LVDS Display contrast */

	/* UART1  */
	MX6DL_PAD_SD3_DAT7__UART1_TXD,
	MX6DL_PAD_SD3_DAT6__UART1_RXD,

	/* UART2*/
	MX6DL_PAD_EIM_D26__UART2_TXD,
	MX6DL_PAD_EIM_D27__UART2_RXD,

	/* UART4*/
	MX6DL_PAD_KEY_COL0__UART4_TXD,
	MX6DL_PAD_KEY_ROW0__UART4_RXD,


	/* USB OC pin */
	MX6DL_PAD_KEY_COL4__USBOH3_USBOTG_OC,


	/* USDHC1 */
	MX6DL_PAD_SD1_CLK__USDHC1_CLK,
	MX6DL_PAD_SD1_CMD__USDHC1_CMD,
	MX6DL_PAD_SD1_DAT0__USDHC1_DAT0,
	MX6DL_PAD_SD1_DAT1__USDHC1_DAT1,
	MX6DL_PAD_SD1_DAT2__USDHC1_DAT2,
	MX6DL_PAD_SD1_DAT3__USDHC1_DAT3,
	MX6DL_PAD_GPIO_1__GPIO_1_1,		/* SD1_CD */


	/* USDHC3 */
	MX6DL_PAD_SD3_CLK__USDHC3_CLK_50MHZ,
	MX6DL_PAD_SD3_CMD__USDHC3_CMD_50MHZ,
	MX6DL_PAD_SD3_DAT0__USDHC3_DAT0_50MHZ,
	MX6DL_PAD_SD3_DAT1__USDHC3_DAT1_50MHZ,
	MX6DL_PAD_SD3_DAT2__USDHC3_DAT2_50MHZ,
	MX6DL_PAD_SD3_DAT3__USDHC3_DAT3_50MHZ,
	MX6DL_PAD_GPIO_1__GPIO_1_1,		/* SD3_CD */
//	NEW_PAD_CTRL(MX6DL_PAD_SD3_DAT4__GPIO_7_1, MX6DL_SABRELITE_SD3_WP_PADCFG),

	/* USDHC4 */
	MX6DL_PAD_SD4_CLK__USDHC4_CLK_50MHZ,
	MX6DL_PAD_SD4_CMD__USDHC4_CMD_50MHZ,
	MX6DL_PAD_SD4_DAT0__USDHC4_DAT0_50MHZ,
	MX6DL_PAD_SD4_DAT1__USDHC4_DAT1_50MHZ,
	MX6DL_PAD_SD4_DAT2__USDHC4_DAT2_50MHZ,
	MX6DL_PAD_SD4_DAT3__USDHC4_DAT3_50MHZ,
	MX6DL_PAD_SD4_DAT4__USDHC4_DAT4_50MHZ,
	MX6DL_PAD_SD4_DAT5__USDHC4_DAT5_50MHZ,
	MX6DL_PAD_SD4_DAT6__USDHC4_DAT6_50MHZ,
	MX6DL_PAD_SD4_DAT7__USDHC4_DAT7_50MHZ,
//	MX6DL_PAD_NANDF_D6__GPIO_2_6,		/* J20 - SD4_CD */
//	MX6DL_PAD_NANDF_D7__GPIO_2_7,		/* SD4_WP */





#ifdef CONFIG_MX6_ENET_IRQ_TO_GPIO
	MX6DL_PAD_GPIO_6__OBSERVE_MUX_OBSRV_INT_OUT1,
#else
#endif

};

/* Engicam board pin initialization for mx6dl*/

static iomux_v3_cfg_t mx6dl_rqs_pads_sk[] =
{

};

#if 1

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


//static iomux_v3_cfg_t MX6Q_USDHC_PAD_SETTING(1, 50);
//static iomux_v3_cfg_t MX6Q_USDHC_PAD_SETTING(1, 100);
//static iomux_v3_cfg_t MX6Q_USDHC_PAD_SETTING(1, 200);
#endif






enum sd_pad_mode {
	SD_PAD_MODE_LOW_SPEED,
	SD_PAD_MODE_MED_SPEED,
	SD_PAD_MODE_HIGH_SPEED,
};

#if 1

static int plt_sd1_pad_change(unsigned int index, int clock)
{
        return 0;
#if 0	
	static enum sd_pad_mode pad_mode = SD_PAD_MODE_LOW_SPEED;
	if (clock > 100000000) {
		if (pad_mode == SD_PAD_MODE_HIGH_SPEED)
			return 0;

		pad_mode = SD_PAD_MODE_HIGH_SPEED;
		return mxc_iomux_v3_setup_multiple_pads(mx6q_sd1_200mhz,
					ARRAY_SIZE(mx6q_sd1_200mhz));
	} else if (clock > 52000000) {
		if (pad_mode == SD_PAD_MODE_MED_SPEED)
			return 0;

		pad_mode = SD_PAD_MODE_MED_SPEED;
		return mxc_iomux_v3_setup_multiple_pads(mx6q_sd1_100mhz,
					ARRAY_SIZE(mx6q_sd1_100mhz));
	} else {
		if (pad_mode == SD_PAD_MODE_LOW_SPEED)
			return 0;

		pad_mode = SD_PAD_MODE_LOW_SPEED;
		return mxc_iomux_v3_setup_multiple_pads(mx6q_sd1_50mhz,
					ARRAY_SIZE(mx6q_sd1_50mhz));
	}
#endif
}


static int plt_sd3_pad_change(unsigned int index, int clock)
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

static int plt_sd4_pad_change(unsigned int index, int clock)
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

#endif

static const struct esdhc_platform_data mx6q_icore_sd1_data __initconst = {
	.cd_gpio = -1,
	.wp_gpio = -1,
	.keep_power_at_suspend = 1,
	.platform_pad_change = plt_sd1_pad_change,
};

static const struct esdhc_platform_data mx6q_icore_sd3_data __initconst = {
	.cd_gpio = ICORE_M6_SD1_CD,//ICORE_M6_SD3_CD,
	.wp_gpio = -1,//ICORE_M6_SD3_WP,
	.keep_power_at_suspend = 1,
	.platform_pad_change = plt_sd3_pad_change,
};

static const struct esdhc_platform_data mx6q_icore_sd4_data __initconst = {
	.cd_gpio = -1,//ICORE_M6_SD4_CD,
	.wp_gpio = -1,//ICORE_M6_SD4_WP,
	.keep_power_at_suspend = 1,
	.platform_pad_change = plt_sd4_pad_change,
};


static const struct anatop_thermal_platform_data
	mx6q_icore_anatop_thermal_data __initconst = {
		.name = "anatop_thermal",
};

static inline void mx6q_icore_init_uart(void)
{
	imx6q_add_imx_uart(0, NULL);
	imx6q_add_imx_uart(1, NULL);
	
	imx6q_add_imx_uart(3, NULL);
}

static int mx6q_icore_fec_phy_init(struct phy_device *phydev)
{

    if (cpu_is_mx6q())
    {
       //write register 6 addr 2 TXD[0:3] skew
	phy_write(phydev, 0x0d, 0x0002);
	phy_write(phydev, 0x0e, 0x0006);
	phy_write(phydev, 0x0d, 0x4002);
	phy_write(phydev, 0x0e, 0x3333);

       //write register 5 addr 2 RXD[0:3] skew
	phy_write(phydev, 0x0d, 0x0002);
	phy_write(phydev, 0x0e, 0x0005);
	phy_write(phydev, 0x0d, 0x4002);
	phy_write(phydev, 0x0e, 0x7777);

       //write register 4 addr 2 RX_DV TX_EN skew
	phy_write(phydev, 0x0d, 0x0002);
	phy_write(phydev, 0x0e, 0x0004);
	phy_write(phydev, 0x0d, 0x4002);
	phy_write(phydev, 0x0e, 0x0037);



       //write register 8 addr 2 RX_CLK GTX_CLK skew
	phy_write(phydev, 0x0d, 0x0002);
	phy_write(phydev, 0x0e, 0x0008);
	phy_write(phydev, 0x0d, 0x4002);
	phy_write(phydev, 0x0e, 0x01E7);
     }
     else
     {
       //write register 6 addr 2 TXD[0:3] skew
	phy_write(phydev, 0x0d, 0x0002);
	phy_write(phydev, 0x0e, 0x0006);
	phy_write(phydev, 0x0d, 0x4002);
	phy_write(phydev, 0x0e, 0x1111);

       //write register 5 addr 2 RXD[0:3] skew
	phy_write(phydev, 0x0d, 0x0002);
	phy_write(phydev, 0x0e, 0x0005);
	phy_write(phydev, 0x0d, 0x4002);
	phy_write(phydev, 0x0e, 0x2222);

       //write register 4 addr 2 RX_DV TX_EN skew
	phy_write(phydev, 0x0d, 0x0002);
	phy_write(phydev, 0x0e, 0x0004);
	phy_write(phydev, 0x0d, 0x4002);
	phy_write(phydev, 0x0e, 0x0037);



       //write register 8 addr 2 RX_CLK GTX_CLK skew
	phy_write(phydev, 0x0d, 0x0002);
	phy_write(phydev, 0x0e, 0x0008);
	phy_write(phydev, 0x0d, 0x4002);
	phy_write(phydev, 0x0e, 0x01E7);
     }
	return 0;
}

static struct fec_platform_data fec_data __initdata = {
	.init = mx6q_icore_fec_phy_init,
	.phy = PHY_INTERFACE_MODE_RGMII,
#ifdef CONFIG_MX6_ENET_IRQ_TO_GPIO
	.gpio_irq = MX6_ENET_IRQ,
#endif
};

static int mx6q_icore_spi_cs[] = {
	ICORE_M6_ECSPI1_CS1,
};

static const struct spi_imx_master mx6q_icore_spi_data __initconst = {
	.chipselect     = mx6q_icore_spi_cs,
	.num_chipselect = ARRAY_SIZE(mx6q_icore_spi_cs),
};


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


static struct imxi2c_platform_data mx6q_icore_i2c_data = {
	.bitrate = 100000,
};


static struct i2c_board_info mxc_i2c0_board_info[] __initdata = {
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

static struct i2c_board_info mxc_i2c2_board_info_sk[] __initdata = {
	
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
//MP: TBD	mx6_usb_dr_init();
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

static struct gpio mx6q_icore_flexcan_gpios[] = {
	{ ICORE_M6_CAN1_EN, GPIOF_OUT_INIT_LOW, "flexcan1-en" },
	{ ICORE_M6_CAN1_STBY, GPIOF_OUT_INIT_LOW, "flexcan1-stby" },
};

static void mx6q_icore_flexcan0_switch(int enable)
{
	if (enable) {
		gpio_set_value(ICORE_M6_CAN1_EN, 1);
		gpio_set_value(ICORE_M6_CAN1_STBY, 1);
	} else {
		gpio_set_value(ICORE_M6_CAN1_EN, 0);
		gpio_set_value(ICORE_M6_CAN1_STBY, 0);
	}
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
	.interface_pix_fmt = IPU_PIX_FMT_RGB24,
	.mode_str = "LDB-XGA",
	.default_bpp = 32,
	.int_clk = false,
	}, {
	.disp_dev = "ldb",
	.interface_pix_fmt = IPU_PIX_FMT_RGB24,
	.mode_str = "LDB-XGA",
	.default_bpp = 32,
	.int_clk = false,
	}, {
         .disp_dev          = "hdmi",
         .interface_pix_fmt  = IPU_PIX_FMT_RGB24,
         .mode_str                  = "1280x720M@50",
         .default_bpp             = 32,
         .int_clk              = false,
	}, {
	.disp_dev = "lcd",
	.interface_pix_fmt = IPU_PIX_FMT_RGB565,
	.mode_str = "Amp-WD",
	.default_bpp = 16,
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
	.disp_id = 1,
};

#if 0
static struct fsl_mxc_lcd_platform_data lcdif_data = {
	.ipu_id = 0,
	.disp_id = 0,
	.default_ifmt = IPU_PIX_FMT_RGB565,
};
#endif
static struct fsl_mxc_ldb_platform_data ldb_data = {
	.ipu_id = 1,
	.disp_id = 0,
	.ext_ref = 1,
//	.mode = LDB_SEP0,
	.mode = LDB_SEP1,
	.sec_ipu_id = 1,
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
	.microvolts		= 2500000,
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
	.microvolts		= 0,
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

#if 0
static struct platform_pwm_backlight_data mx6_icore_pwm_backlight_data = {
	.pwm_id = 3,
	.max_brightness = 255,
	.dft_brightness = 128,
	.pwm_period_ns = 50000,
};
#endif
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

static void rqs_customized_board_init (void)
{
	printk("%s selected.", engi_board_description_str[engi_board] );	
	
	if (cpu_is_mx6q())
	{
		switch(engi_board)
		{
			case ENGICAM_Q7_SK:
				mxc_iomux_v3_setup_multiple_pads(mx6q_rqs_pads_sk, ARRAY_SIZE(mx6q_rqs_pads_sk));
			break;
		}
	}
	else
	{
		switch(engi_board)
		{
			case ENGICAM_Q7_SK:
				mxc_iomux_v3_setup_multiple_pads(mx6dl_rqs_pads_sk, ARRAY_SIZE(mx6dl_rqs_pads_sk));
			break;
		}
	}
}

static void rqs_customized_i2c_init (void)
{
	switch(engi_board)
	{
		case ENGICAM_Q7_SK:
			i2c_register_board_info(2, mxc_i2c2_board_info_sk,	ARRAY_SIZE(mxc_i2c2_board_info_sk));
		break;
	}
}

/*!
 * Board specific initialization.
 */
/*!
 * Board specific initialization.
 */
static void __init mx6_icore_board_init(void)
{
	int i;
	int ret;
	struct clk *clko2;
	struct clk *new_parent;
	int rate;

	if (cpu_is_mx6q()) {
		printk(KERN_ERR "------------ Board type %s\n",
        	       "i.Core M6 RQS based (Q/D CPU)");
		mxc_iomux_v3_setup_multiple_pads(mx6q_icore_pads,
					ARRAY_SIZE(mx6q_icore_pads));

	}
	else	{
		printk(KERN_ERR "------------ Board type %s\n",
        	       "i.Core M6 RQS based (DL/S CPU)");
		mxc_iomux_v3_setup_multiple_pads(mx6dl_icore_pads,
					ARRAY_SIZE(mx6dl_icore_pads));
		if(cpu_is_mx6dl())
			printk(KERN_ERR "Test cpu_is_mx6dl PASSED\n");
		else
			printk(KERN_ERR "Test cpu_is_mx6dl FAILED\n");

	}

	rqs_customized_board_init();

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

	if (!cpu_is_mx6q()) {
		ldb_data.ipu_id = 0;
		ldb_data.disp_id = 0;
		ldb_data.sec_ipu_id = 0;
		ldb_data.sec_disp_id = 1;
		hdmi_core_data.disp_id = 1;
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
//	imx6q_add_lcdif(&lcdif_data);
	imx6q_add_ldb(&ldb_data);
//	imx6q_add_v4l2_output(0);
//	imx6q_add_v4l2_capture(0, &capture_data[0]);
//	imx6q_add_v4l2_capture(1, &capture_data[1]);

	imx6q_add_imx_snvs_rtc();

//	imx6q_add_imx_caam();

	imx6q_add_imx_i2c(0, &mx6q_icore_i2c_data);
	imx6q_add_imx_i2c(1, &mx6q_icore_i2c_data);
	imx6q_add_imx_i2c(2, &mx6q_icore_i2c_data);
	i2c_register_board_info(0, mxc_i2c0_board_info,
			ARRAY_SIZE(mxc_i2c0_board_info));
	i2c_register_board_info(1, mxc_i2c1_board_info,
			ARRAY_SIZE(mxc_i2c1_board_info));
	i2c_register_board_info(2, mxc_i2c2_board_info,
			ARRAY_SIZE(mxc_i2c2_board_info));

	rqs_customized_i2c_init();

	/* SPI ... TODO su ECSPI3*/
//	imx6q_add_ecspi(0, &mx6q_icore_spi_data);
//	spi_device_init();

	imx6q_add_mxc_hdmi(&hdmi_data);

	imx6q_add_anatop_thermal_imx(1, &mx6q_icore_anatop_thermal_data);
	imx6_init_fec(fec_data);
#ifdef CONFIG_MX6_ENET_IRQ_TO_GPIO
	/* Make sure the IOMUX_OBSRV_MUX1 is set to ENET_IRQ. */
	mxc_iomux_set_specialbits_register(IOMUX_OBSRV_MUX1_OFFSET,
		OBSRV_MUX1_ENET_IRQ, OBSRV_MUX1_MASK);
#endif
	imx6q_add_pm_imx(0, &mx6q_icore_pm_data);
	imx6q_add_sdhci_usdhc_imx(0, &mx6q_icore_sd1_data);
	imx6q_add_sdhci_usdhc_imx(2, &mx6q_icore_sd3_data);
	imx6q_add_sdhci_usdhc_imx(3, &mx6q_icore_sd4_data);

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
	imx6q_add_otp();
	imx6q_add_viim();
	imx6q_add_imx2_wdt(0, NULL);
	imx6q_add_dma();

	imx6q_add_dvfs_core(&icore_dvfscore_data);
	mx6_cpu_regulator_init();

//	icore_add_device_buttons();

	imx6q_add_hdmi_soc();
	imx6q_add_hdmi_soc_dai();

	ret = gpio_request_array(mx6q_icore_flexcan_gpios,
			ARRAY_SIZE(mx6q_icore_flexcan_gpios));
	if (ret)
		pr_err("failed to request flexcan1-gpios: %d\n", ret);
	else
		imx6q_add_flexcan0(&mx6q_icore_flexcan0_pdata);

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

//	imx6q_add_gpmi(&mx6q_gpmi_nand_platform_data);
//	mx6q_csi0_io_init();


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
