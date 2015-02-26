/*
 * Copyright (C) 2011-2013 Freescale Semiconductor, Inc. All Rights Reserved.
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
#include <linux/smsc911x.h>
#include <linux/spi/spi.h>
#include <linux/spi/flash.h>
#include <linux/mtd/physmap.h>
#include <linux/i2c.h>
#include <linux/i2c/pca953x.h>
#include <linux/ata.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/partitions.h>
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
//#include <linux/mxc_asrc.h>
#include <sound/pcm.h>
#include <linux/mfd/mxc-hdmi-core.h>

#include <mach/common.h>
#include <mach/hardware.h>
#include <mach/mxc_dvfs.h>
#include <mach/memory.h>
#include <mach/iomux-mx6q.h>
#include <mach/imx-uart.h>
#include <mach/viv_gpu.h>
#include <mach/ahci_sata.h>
#include <mach/ipu-v3.h>
#include <mach/mxc_hdmi.h>
#include <mach/mxc_asrc.h>
#include <mach/mipi_dsi.h>
#include <mach/mipi_csi2.h>

#include <asm/irq.h>
#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>

#include "usb.h"
#include "devices-imx6q.h"
#include "crm_regs.h"
#include "cpu_op-mx6.h"
#include "board-mx6q_sabreauto.h"
#include "board-mx6solo_sabreauto.h"

/* sorted by GPIO_NR */

#define SABREAUTO_SD3_CD		IMX_GPIO_NR(1, 0)
#define SABREAUTO_SD2_CD		IMX_GPIO_NR(1, 4)
#define SABREAUTO_SD2_WP		IMX_GPIO_NR(1, 16)
#define PCIE_RST_B			IMX_GPIO_NR(1, 17)
#define TCH_INT				IMX_GPIO_NR(1, 20)
#define MX6_ENET_RST			IMX_GPIO_NR(1, 25)
#define MX6_ENET_IRQ			IMX_GPIO_NR(1, 26)
#define VIDEO_RST_N			IMX_GPIO_NR(1, 29)
#define VIDEO_PWRDWN_N			IMX_GPIO_NR(1, 30)

#define SABREAUTO_DISP0_PWR		IMX_GPIO_NR(2, 8)
#define SABREAUTO_DISP0_RESET		IMX_GPIO_NR(2, 9)
#define CAN_SLP				IMX_GPIO_NR(2, 31)

#define USB_OTG_OC			IMX_GPIO_NR(3, 21)
#define USB_OTG_PWR			IMX_GPIO_NR(3, 22)

#define SABREAUTO_ECSPI1_CS1		IMX_GPIO_NR(4, 10)

#define SABREAUTO_PMIC_INT		IMX_GPIO_NR(7, 13)

static int spinor_en;
static int uart1_en;
static struct clk *sata_clk;
static int mipi_sensor;
static int can0_enable;
static int max11801_mode = 1;

extern char *soc_reg_id;

#define IOMUX_OBSRV_MUX1_OFFSET		0x3c
#define OBSRV_MUX1_MASK			0x3f
#define OBSRV_MUX1_ENET_IRQ		0x9

extern bool enet_to_gpio_6;


static int __init spinor_enable(char *p)
{
       spinor_en = 1;
       return 0;
};
early_param("spi-nor", spinor_enable);


static int __init uart1_enable(char *p)
{
	uart1_en = 1;
	return 0;
};
early_param("uart1", uart1_enable);


static const struct esdhc_platform_data mx6q_sabreauto_sd3_data __initconst = {
	.cd_gpio = SABREAUTO_SD3_CD,
	.keep_power_at_suspend = 1,
};


static const struct esdhc_platform_data mx6q_sabreauto_sd2_data __initconst = {
	.cd_gpio = SABREAUTO_SD2_CD,
	.wp_gpio = SABREAUTO_SD2_WP,
	.keep_power_at_suspend = 1,
};


static int __init gpmi_nand_platform_init(void)
{
	iomux_v3_cfg_t *nand_pads = NULL;
	u32 nand_pads_cnt;

	nand_pads = mx6q_gpmi_nand;
	nand_pads_cnt = ARRAY_SIZE(mx6q_gpmi_nand);

	BUG_ON(!nand_pads);
	return mxc_iomux_v3_setup_multiple_pads(nand_pads, nand_pads_cnt);
};


static const struct gpmi_nand_platform_data
mx6q_gpmi_nand_platform_data __initconst = {
	.platform_init           = gpmi_nand_platform_init,
	.min_prop_delay_in_ns    = 5,
	.max_prop_delay_in_ns    = 9,
	.max_chip_count          = 1,
};


static const struct anatop_thermal_platform_data
mx6q_sabreauto_anatop_thermal_data __initconst = {
	.name = "anatop_thermal",
};


static const struct imxuart_platform_data mx6_bt_uart_data __initconst = {
	.flags = IMXUART_HAVE_RTSCTS | IMXUART_SDMA,
	.dma_req_rx = MX6Q_DMA_REQ_UART1_RX,
	.dma_req_tx = MX6Q_DMA_REQ_UART1_TX,
};


static inline void mx6q_sabreauto_init_uart(void)
{
	imx6q_add_imx_uart(0, &mx6_bt_uart_data);
	imx6q_add_imx_uart(1, NULL);
	imx6q_add_imx_uart(2, NULL);
	imx6q_add_imx_uart(4, NULL);
};


static int mx6q_sabreauto_fec_phy_init(struct phy_device *phydev)
{
	//Address
	phy_write(phydev, 0x0D, 0x02);
	//Register Control Signal Skew
	phy_write(phydev, 0x0E, 0x04);
	//Select register Data
	phy_write(phydev, 0x0D, 0x4002);
	//DATA
	phy_write(phydev, 0x0E, 0x00F7);

	//Address
	phy_write(phydev, 0x0D, 0x02);
	//Register RX data skew
	phy_write(phydev, 0x0E, 0x05);
	//Select register Data
	phy_write(phydev, 0x0D, 0x4002);
	//Data
	phy_write(phydev, 0x0E, 0x023E);

	//Address
	phy_write(phydev, 0x0D, 0x02);
	//Register TX data skew
	phy_write(phydev, 0x0E, 0x06);
	//Select register Data
	phy_write(phydev, 0x0D, 0x4002);
	//Data
	phy_write(phydev, 0x0E, 0x0776);

	//Address
	phy_write(phydev, 0x0D, 0x02);
	//Register ClK skew
	phy_write(phydev, 0x0E, 0x08);
	//Select register Data
	phy_write(phydev, 0x0D, 0x4002);
	//Data
	phy_write(phydev, 0x0E, 0x03FF);

	return 0;
};


static int mx6q_sabreauto_fec_power_hibernate(struct phy_device *phydev)
{
	return 0;
};


static struct fec_platform_data fec_data __initdata = {
	.init			= mx6q_sabreauto_fec_phy_init,
	.power_hibernate	= mx6q_sabreauto_fec_power_hibernate,
	.phy			= PHY_INTERFACE_MODE_RGMII,
	.gpio_irq		= MX6_ENET_IRQ,
	//.gpio_irq		= -1,
};


static int mx6q_sabreauto_spi_cs[] = {
	SABREAUTO_ECSPI1_CS1,
};


static const struct spi_imx_master mx6q_sabreauto_spi_data __initconst = {
	.chipselect     = mx6q_sabreauto_spi_cs,
	.num_chipselect = ARRAY_SIZE(mx6q_sabreauto_spi_cs),
};


static struct mtd_partition m25p32_partitions[] = {
	{
/*		.name	= "bootloader",
		.offset	= 0,
		.size	= SZ_256K,
	}, {
		.name	= "bootenv",
		.offset = MTDPART_OFS_APPEND,
		.size	= SZ_8K,
		.mask_flags = MTD_WRITEABLE,
	}, {
*/		.name	= "kernel",
		.offset	= MTDPART_OFS_NXTBLK,
		.size	= MTDPART_SIZ_FULL,
	},
};


static struct flash_platform_data m25p32_spi_flash_data = {
	.name		= "m25p32",
	.parts		= m25p32_partitions,
	.nr_parts 	= ARRAY_SIZE(m25p32_partitions),
	.type		= "m25p32",
};


static struct spi_board_info m25p32_spi1_board_info[] __initdata = {
	{
		/* The modalias must be the same as spi device driver name */
		.modalias	= "m25p80",
		.max_speed_hz	= 20000000,
		.bus_num	= 1,
		.chip_select	= 1,
		.platform_data	= &m25p32_spi_flash_data,
	},
};


static void spi_device_init(void)
{
	spi_register_board_info(m25p32_spi1_board_info,
				ARRAY_SIZE(m25p32_spi1_board_info));
};

////////////////////////////////////////////////////////////////////////////////////////Begin
static void adv7180_pwdn(void)
{
//	if (pwdn)
//	{ 
		gpio_request(VIDEO_PWRDWN_N, "video-pwr");
		gpio_direction_output(VIDEO_PWRDWN_N, 0);
//	}	
//	else 
//	{
//		gpio_request(VIDEO_PWRDWN_N, "video-pwr");
//		gpio_direction_output(VIDEO_PWRDWN_N, 1);
//	}
};


static void adv7180_rstn(void)
{
	gpio_request(VIDEO_RST_N, "video-rst");
	gpio_direction_output(VIDEO_RST_N, 0);
	
//	if (rstn) 
//	{
	 	gpio_set_value(VIDEO_RST_N, 0);
		msleep(10);
		gpio_set_value(VIDEO_RST_N, 1);
//	}
//	else
//	{ 
//	 	gpio_set_value(VIDEO_RST_N, 1);
//	}
};
////////////////////////////////////////////////////////////////////////////////////////END

static void mx6q_csi0_io_init(void)
{
	
	mxc_iomux_set_gpr_register(1, 19, 1, 1);
};	


static struct fsl_mxc_tvin_platform_data adv7180_data = {
	.dvddio_reg	= NULL,
	.dvdd_reg	= NULL,
	.avdd_reg	= NULL,
	.pvdd_reg	= NULL,
	.pwdn		= adv7180_pwdn,
	.reset		= adv7180_rstn,
	.cvbs		= true,
	.io_init	= mx6q_csi0_io_init,
};


static struct imxi2c_platform_data mx6q_sabreauto_i2c1_data = {
	.bitrate	= 100000,
};


static struct imxi2c_platform_data mx6q_sabreauto_i2c2_data = {
	.bitrate	= 100000,
};


static struct imxi2c_platform_data mx6q_sabreauto_i2c3_data = {
	.bitrate	= 100000,
};


static struct i2c_board_info mxc_i2c1_board_info[] __initdata = {
	{
	 	I2C_BOARD_INFO("imx-sgtl5000", 0x0a),
	 },{
		I2C_BOARD_INFO("adv7180", 0x21),
		.platform_data = (void *)&adv7180_data,
	},
};


static struct i2c_board_info mxc_i2c2_board_info[] __initdata = {
	{
		I2C_BOARD_INFO("mxc_hdmi_i2c", 0x50),
	}, 
};


static struct i2c_board_info mxc_i2c3_board_info[] __initdata = {
	{
		I2C_BOARD_INFO("max11801", 0x48),
		.platform_data = (void *)&max11801_mode,
		.irq = gpio_to_irq(TCH_INT),		
	},
};


static void imx6q_sabreauto_usbotg_vbus(bool on)
{
	if (on)
	{
		gpio_request(USB_OTG_PWR, "usb-otg-pwr");
		gpio_direction_output(USB_OTG_PWR, 1);
	}
	else
	{
		gpio_request(USB_OTG_PWR, "usb-otg-pwr");
		gpio_direction_output(USB_OTG_PWR, 0);
	}
};


static void __init imx6q_sabreauto_init_usb(void)
{
	int ret = 0;
	imx_otg_base = MX6_IO_ADDRESS(MX6Q_USB_OTG_BASE_ADDR);

	ret = gpio_request(USB_OTG_OC, "otg-oc");
	if (ret) {
		printk(KERN_ERR"failed to get GPIO SABREAUTO_USB_OTG_OC:"
			" %d\n", ret);
		return;
	}
	gpio_direction_input(USB_OTG_OC);

	mxc_iomux_set_gpr_register(1, 13, 1, 0);
	mx6_set_otghost_vbus_func(imx6q_sabreauto_usbotg_vbus);
};


static struct viv_gpu_platform_data imx6q_gpu_pdata __initdata = {
	.reserved_mem_size = SZ_128M,
};


/* HW Initialization, if return 0, initialization is successful. */
static int mx6q_sabreauto_sata_init(struct device *dev, void __iomem *addr)
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

#ifdef CONFIG_SATA_AHCI_PLATFORM
	ret = sata_init(addr, tmpdata);
	if (ret == 0)
		return ret;
#else
	usleep_range(1000, 2000);
	/* AHCI PHY enter into PDDQ mode if the AHCI module is not enabled */
	tmpdata = readl(addr + PORT_PHY_CTL);
	writel(tmpdata | PORT_PHY_CTL_PDDQ_LOC, addr + PORT_PHY_CTL);
	pr_info("No AHCI save PWR: PDDQ %s\n", ((readl(addr + PORT_PHY_CTL)
					>> 20) & 1) ? "enabled" : "disabled");
#endif

release_sata_clk:
	/* disable SATA_PHY PLL */
	writel((readl(IOMUXC_GPR13) & ~0x2), IOMUXC_GPR13);
	clk_disable(sata_clk);
put_sata_clk:
	clk_put(sata_clk);

	return ret;
};


#ifdef CONFIG_SATA_AHCI_PLATFORM
static void mx6q_sabreauto_sata_exit(struct device *dev)
{
	clk_disable(sata_clk);
	clk_put(sata_clk);

};


static struct ahci_platform_data mx6q_sabreauto_sata_data = {
	.init = mx6q_sabreauto_sata_init,
	.exit = mx6q_sabreauto_sata_exit,
};
#endif


static void mx6q_setup_weimcs(void)
{
	void __iomem *weim_reg = MX6_IO_ADDRESS(WEIM_BASE_ADDR);
	void __iomem *ccm_reg = MX6_IO_ADDRESS(CCM_BASE_ADDR);
//	void __iomem *gpr1_reg = MX6_IO_ADDRESS(0x020E0000);
	
	unsigned int reg;
	struct clk *clk;
	u32 rate;

	/* 
	 *CLKCTL_CCGR6: Set emi_slow_clock to be on in all modes 
	*/

	reg = readl(ccm_reg + 0x80);
	reg |= 0x00000C00;
	writel(reg, ccm_reg + 0x80);

	clk = clk_get(NULL, "emi_slow_clk");
	if (IS_ERR(clk))
		printk(KERN_ERR "emi_slow_clk not found\n");

	rate = clk_get_rate(clk);
	if (rate != 132000000)
		printk(KERN_ERR "Warning: emi_slow_clk not set to 132 MHz!"
		       " WEIM timing may be incorrect!\n");

//	/* 
//	 *  Check GPR1 reg make sure CS0 is 128MB and all others are disabled. 
//	*/
//
//	reg = readl(gpr1_reg + 0x04);
//  	printk("GPR1 0x04 %08x \n", reg);	
//	reg &= 0xFFFFF000;
//	reg |= 0x0000001B;
//	writel(reg, gpr1_reg + 0x04);
//	reg = readl(gpr1_reg + 0x04);
//	printk("GPR1 0x04 %08x \n", reg);	

	/*
	 * For EIM General Configuration registers.
	 */

	//EIM Configuration Register  Request access to WEIM config Regs
	writel(0x00000011, (weim_reg + 0x00000094));

	while(!((readl(weim_reg + 0x00000094)) & 0x00000002))
	{
		printk("wait");
	}

	//EIM Configuration Register
	writel(0x00000020, (weim_reg + 0x00000090));

	//CS0 Configuration Registers
	writel(0x0771103F, (weim_reg + 0x00000000));
	writel(0x00001002, (weim_reg + 0x00000004));
	writel(0x08023000, (weim_reg + 0x00000008));
	writel(0x00000000, (weim_reg + 0x0000000C));
      //writel(0x08080E40, (weim_reg + 0x00000010));
	writel(0x09080E40, (weim_reg + 0x00000010));
	writel(0x00000000, (weim_reg + 0x00000014));

//	//CS1 Configuration Registers
//	writel(0x0771103F, (weim_reg + 0x00000018));
//	writel(0x00001002, (weim_reg + 0x0000001C));
//	writel(0x08023000, (weim_reg + 0x00000020));
//	writel(0x00000000, (weim_reg + 0x00000024));
//	writel(0x08080E40, (weim_reg + 0x00000028));
//	writel(0x00000000, (weim_reg + 0x0000002C));

	//EIM Configuration Register  Release access to WEIM config Regs
	writel(0x00000010, (weim_reg + 0x00000094));

	printk(KERN_ERR "WEIM INIT\n");
};

////////////////////////////////////////////////////////////////////////////////////////////////////Begin
static volatile u16 *WEIM_CS0_io;
static void mx6q_FGPA_VER(void)
{
	u16 version;

	// Create WEIM CS0 Pointer	
	WEIM_CS0_io = ioremap(CS0_BASE_ADDR, SZ_64M);
	
	if (WEIM_CS0_io == 0)
	 {
	  printk("WEIM_CS0 Remap Failed\n");	
	 }
  	
	version = readw(WEIM_CS0_io + 0x00);
  	printk("FPGA 00 %04x \n", version);	

	version = readw(WEIM_CS0_io + 0x04);
  	printk("FPGA 04 %04x \n", version);	

	version = readw(WEIM_CS0_io + 0x08);
  	printk("FPGA 08 %04x \n", version);	

	version = readw(WEIM_CS0_io + 0x0A);
  	printk("FPGA 0A %04x \n", version);	

	version = readw(WEIM_CS0_io + 0x0C);
  	printk("FPGA 0C %04x \n", version);	

	version = readw(WEIM_CS0_io + 0x0E);
  	printk("FPGA 0E %04x \n", version);	

	iounmap((void *)WEIM_CS0_io);

};
////////////////////////////////////////////////////////////////////////////////////////////////////END

static struct imx_asrc_platform_data imx_asrc_data = {
	.channel_bits	= 4,
	.clk_map_ver	= 2,
};


static void mx6q_sabreauto_reset_mipi_dsi(void)
{
	gpio_set_value(SABREAUTO_DISP0_PWR, 1);
	gpio_set_value(SABREAUTO_DISP0_RESET, 1);
	udelay(10);
	gpio_set_value(SABREAUTO_DISP0_RESET, 0);
	udelay(50);
	gpio_set_value(SABREAUTO_DISP0_RESET, 1);

	/*
	 * it needs to delay 120ms minimum for reset complete
	 */
	msleep(120);
};

static struct mipi_dsi_platform_data mipi_dsi_pdata = {
	.ipu_id		= 0,
	.disp_id	= 0,
	.lcd_panel	= "TRULY-WVGA",
	.reset		= mx6q_sabreauto_reset_mipi_dsi,
};

static struct ipuv3_fb_platform_data sabr_fb_data[] = {
	{ /*fb0*/
		.disp_dev		= "ldb",
		.interface_pix_fmt	= IPU_PIX_FMT_RGB666,
		.mode_str		= "LDB-XGA",
		.default_bpp		= 16,
		.int_clk		= false,
	}, {
		.disp_dev		= "ldb",
		.interface_pix_fmt	= IPU_PIX_FMT_RGB666,
		.mode_str		= "LDB-XGA",
		.default_bpp		= 16,
		.int_clk		= false,
	}, {
		.disp_dev               = "lcd",
		.interface_pix_fmt      = IPU_PIX_FMT_RGB565,
		.mode_str               = "CLAA-WVGA",
		.default_bpp            = 16,
		.int_clk                = false,
	},
};

static void hdmi_init(int ipu_id, int disp_id)
{
	int hdmi_mux_setting;

	if ((ipu_id > 1) || (ipu_id < 0)) {
		printk(KERN_ERR"Invalid IPU select for HDMI: %d. Set to 0\n",
			ipu_id);
		ipu_id = 0;
	}

	if ((disp_id > 1) || (disp_id < 0)) {
		printk(KERN_ERR"Invalid DI select for HDMI: %d. Set to 0\n",
			disp_id);
		disp_id = 0;
	}

	/* Configure the connection between IPU1/2 and HDMI */
	hdmi_mux_setting = 2*ipu_id + disp_id;

	/* GPR3, bits 2-3 = HDMI_MUX_CTL */
	mxc_iomux_set_gpr_register(3, 2, 2, hdmi_mux_setting);

	/* Set HDMI event as SDMA event2 while Chip version later than TO1.2 */
	if (hdmi_SDMA_check())
		mxc_iomux_set_gpr_register(0, 0, 1, 1);
};

/* On mx6x sabreauto board i2c2 iomux with hdmi ddc,
 * the pins default work at i2c2 function,
 when hdcp enable, the pins should work at ddc function */

static void hdmi_enable_ddc_pin(void)
{
	mxc_iomux_v3_setup_multiple_pads(mx6q_sabreauto_hdmi_ddc_pads,
		ARRAY_SIZE(mx6q_sabreauto_hdmi_ddc_pads));
};

static void hdmi_disable_ddc_pin(void)
{
	mxc_iomux_v3_setup_multiple_pads(mx6q_sabreauto_i2c2_pads,
			ARRAY_SIZE(mx6q_sabreauto_i2c2_pads));
};

static struct fsl_mxc_hdmi_platform_data hdmi_data = {
	.init = hdmi_init,
	.enable_pins = hdmi_enable_ddc_pin,
	.disable_pins = hdmi_disable_ddc_pin,
};

static struct fsl_mxc_hdmi_core_platform_data hdmi_core_data = {
	.ipu_id		= 0,
	.disp_id	= 0,
};

static struct fsl_mxc_lcd_platform_data lcdif_data = {
	.ipu_id		= 0,
	.disp_id	= 0,
	.default_ifmt	= IPU_PIX_FMT_RGB565,
};

static struct fsl_mxc_ldb_platform_data ldb_data = {
	.ipu_id		= 1,
	.disp_id	= 0,
	.ext_ref	= 1,
	.mode 		= LDB_SEP0,
	.sec_ipu_id	= 1,
	.sec_disp_id	= 1,
};

static struct imx_ipuv3_platform_data ipu_data[] = {
	{
		.rev		= 4,
		.csi_clk[0]	= "clko_clk",
	}, {
		.rev		= 4,
		.csi_clk[0]	= "clko_clk",
	},
};

/* Backlight PWM for CPU board lvds*/
static struct platform_pwm_backlight_data mx6_arm2_pwm_backlight_data3 = {
	.pwm_id			= 2,
	.max_brightness		= 255,
	.dft_brightness		= 128,
	.pwm_period_ns		= 50000,
};

/* Backlight PWM for Main board lvds*/
static struct platform_pwm_backlight_data mx6_arm2_pwm_backlight_data4 = {
	.pwm_id			= 3,
	.max_brightness		= 255,
	.dft_brightness		= 128,
	.pwm_period_ns		= 50000,
};

static int flexcan0_en;

static void mx6q_flexcan_tran_enable(void)
{
  if (flexcan0_en) {
	printk(KERN_ERR"CAN CAN_SLP LOW THEN HIGH\n");
	gpio_request(CAN_SLP, "can-slp");
	gpio_direction_output(CAN_SLP, 0);
	msleep(10);
	gpio_direction_output(CAN_SLP, 1);
  }
};

static void mx6q_flexcan0_switch(int enable)
{
    printk(KERN_ERR"flexcan0_en = enable\n");	
    flexcan0_en = enable;
    mx6q_flexcan_tran_enable();
}

static const struct flexcan_platform_data
		mx6q_sabreauto_flexcan_pdata[] __initconst = {
	{
		.transceiver_switch = mx6q_flexcan0_switch,
	}
};

static struct mipi_csi2_platform_data mipi_csi2_pdata = {
	.ipu_id		= 0,
	.csi_id		= 1,
	.v_channel	= 1,
	.lanes		= 1,
	.dphy_clk	= "mipi_pllref_clk",
	.pixel_clk	= "emi_clk",
};

static void sabreauto_suspend_enter(void)
{
	/* suspend preparation */
};

static void sabreauto_suspend_exit(void)
{
	/* resmue resore */
};

static const struct pm_platform_data mx6q_sabreauto_pm_data __initconst = {
	.name		= "imx_pm",
	.suspend_enter	= sabreauto_suspend_enter,
	.suspend_exit	= sabreauto_suspend_exit,
};

static struct regulator_consumer_supply sabreauto_vmmc_consumers[] = {
	REGULATOR_SUPPLY("vmmc", "sdhci-esdhc-imx.1"),
	REGULATOR_SUPPLY("vmmc", "sdhci-esdhc-imx.2"),
	REGULATOR_SUPPLY("vmmc", "sdhci-esdhc-imx.3"),
};

static struct regulator_init_data sabreauto_vmmc_init = {
	.num_consumer_supplies = ARRAY_SIZE(sabreauto_vmmc_consumers),
	.consumer_supplies = sabreauto_vmmc_consumers,
};

static struct fixed_voltage_config sabreauto_vmmc_reg_config = {
	.supply_name	= "vmmc",
	.microvolts	= 3300000,
	.gpio		= -1,
	.init_data	= &sabreauto_vmmc_init,
};

static struct platform_device sabreauto_vmmc_reg_devices = {
	.name		= "reg-fixed-voltage",
	.id		= 0,
	.dev		= {
				.platform_data = &sabreauto_vmmc_reg_config,
	},
};
////////////////////////////////////////////////////////////////////////////////////////////////////Begin

static struct regulator_consumer_supply sgtl5000_sabreauto_consumer_vdda = {
	.supply = "VDDA",
	.dev_name = "1-000a",
};

static struct regulator_consumer_supply sgtl5000_sabreauto_consumer_vddio = {
	.supply = "VDDIO",
	.dev_name = "1-000a",
};

static struct regulator_consumer_supply sgtl5000_sabreauto_consumer_vddd = {
	.supply = "VDDD",
	.dev_name = "1-000a",
};

static struct regulator_init_data sgtl5000_sabreauto_vdda_reg_initdata = {
	.num_consumer_supplies = 1,
	.consumer_supplies = &sgtl5000_sabreauto_consumer_vdda,
};

static struct regulator_init_data sgtl5000_sabreauto_vddio_reg_initdata = {
	.num_consumer_supplies = 1,
	.consumer_supplies = &sgtl5000_sabreauto_consumer_vddio,
};

static struct regulator_init_data sgtl5000_sabreauto_vddd_reg_initdata = {
	.num_consumer_supplies = 1,
	.consumer_supplies = &sgtl5000_sabreauto_consumer_vddd,
};

static struct fixed_voltage_config sgtl5000_sabreauto_vdda_reg_config = {
	.supply_name		= "VDDA",
	.microvolts		= 3300000,
	.gpio			= -1,
	.init_data		= &sgtl5000_sabreauto_vdda_reg_initdata,
};

static struct fixed_voltage_config sgtl5000_sabreauto_vddio_reg_config = {
	.supply_name		= "VDDIO",
	.microvolts		= 1800000,
	.gpio			= -1,
	.init_data		= &sgtl5000_sabreauto_vddio_reg_initdata,
};

static struct fixed_voltage_config sgtl5000_sabreauto_vddd_reg_config = {
	.supply_name		= "VDDD",
	.microvolts		= 1800000,
	.gpio			= -1,
	.init_data		= &sgtl5000_sabreauto_vddd_reg_initdata,
};

static struct platform_device sgtl5000_sabreauto_vdda_reg_devices = {
	.name	= "reg-fixed-voltage",
	.id	= 1,
	.dev	= {
		.platform_data = &sgtl5000_sabreauto_vdda_reg_config,
	},
};

static struct platform_device sgtl5000_sabreauto_vddio_reg_devices = {
	.name	= "reg-fixed-voltage",
	.id	= 2,
	.dev	= {
		.platform_data = &sgtl5000_sabreauto_vddio_reg_config,
	},
};

static struct platform_device sgtl5000_sabreauto_vddd_reg_devices = {
	.name	= "reg-fixed-voltage",
	.id	= 3,
	.dev	= {
		.platform_data = &sgtl5000_sabreauto_vddd_reg_config,
	},
};

static struct mxc_audio_platform_data mx6_sabreauto_audio_data;

static struct imx_ssi_platform_data mx6_sabreauto_ssi_pdata = {
	.flags = IMX_SSI_DMA | IMX_SSI_SYN,
};

static struct mxc_audio_platform_data mx6_sabreauto_audio_data = {
	.ssi_num = 1,
	.src_port = 2,
	.ext_port = 3,
	.sysclk = 25000000,
	.hp_gpio = -1
	
};

static struct platform_device mx6_sabreauto_audio_device = {
	.name = "imx-sgtl5000",
};

static int imx6q_init_audio(void)
{

	mxc_register_device(&mx6_sabreauto_audio_device, 
			    &mx6_sabreauto_audio_data);

	imx6q_add_imx_ssi(1, &mx6_sabreauto_ssi_pdata);

	mxc_iomux_v3_setup_multiple_pads(mx6q_sabreauto_audmux_pads,
					ARRAY_SIZE(mx6q_sabreauto_audmux_pads));
 
	platform_device_register(&sgtl5000_sabreauto_vdda_reg_devices);
	platform_device_register(&sgtl5000_sabreauto_vddio_reg_devices);
	platform_device_register(&sgtl5000_sabreauto_vddd_reg_devices);
 
	return 0;
};

/////////////////////////////////////////////////////////////////////////////////////////////////END


static struct mxc_mlb_platform_data mx6_sabreauto_mlb150_data = {
	.reg_nvcc		= NULL,
	.mlb_clk		= "mlb150_clk",
	.mlb_pll_clk		= "pll6",
};

static struct mxc_dvfs_platform_data sabreauto_dvfscore_data = {
	.reg_id			= "VDDCORE",
	.soc_id			= "VDDSOC",
	.clk1_id		= "cpu_clk",
	.clk2_id 		= "gpc_dvfs_clk",
	.gpc_cntr_offset 	= MXC_GPC_CNTR_OFFSET,
	.ccm_cdcr_offset 	= MXC_CCM_CDCR_OFFSET,
	.ccm_cacrr_offset 	= MXC_CCM_CACRR_OFFSET,
	.ccm_cdhipr_offset 	= MXC_CCM_CDHIPR_OFFSET,
	.prediv_mask 		= 0x1F800,
	.prediv_offset 		= 11,
	.prediv_val 		= 3,
	.div3ck_mask 		= 0xE0000000,
	.div3ck_offset 		= 29,
	.div3ck_val 		= 2,
	.emac_val 		= 0x08,
	.upthr_val 		= 25,
	.dnthr_val 		= 9,
	.pncthr_val 		= 33,
	.upcnt_val 		= 10,
	.dncnt_val 		= 10,
	.delay_time 		= 80,
};

static void __init fixup_mxc_board(struct machine_desc *desc, struct tag *tags,
				   char **cmdline, struct meminfo *mi)
{
};

static int __init early_enable_mipi_sensor(char *p)
{
	mipi_sensor = 1;
	return 0;
};
early_param("mipi_sensor", early_enable_mipi_sensor);

static int __init early_enable_can0(char *p)
{
	can0_enable = 1;
	return 0;
};
early_param("can0", early_enable_can0);

static struct fsl_mxc_capture_platform_data capture_data[] = {
	{
		.csi = 0,
		.ipu = 0,
		.mclk_source = 0,
		.is_mipi = 0,
	}, {
		.csi = 1,
		.ipu = 0,
		.mclk_source = 0,
		.is_mipi = 1,
	},
};

static const struct imx_pcie_platform_data mx6_sabreauto_pcie_data __initconst = {
	.pcie_pwr_en	= -EINVAL,
	.pcie_rst	= PCIE_RST_B,
	.pcie_wake_up	= -EINVAL,
	.pcie_dis	= -EINVAL,
};

/*!
 * Board specific initialization.
 */
static void __init mx6_board_init(void)
{
	int i;
	int ret;
	iomux_v3_cfg_t *common_pads = NULL;
	iomux_v3_cfg_t *can0_pads = NULL;
	iomux_v3_cfg_t *mipi_sensor_pads = NULL;
	iomux_v3_cfg_t *spinor_pads = NULL;
	iomux_v3_cfg_t *weim_pads = NULL;

	int common_pads_cnt;
	int can0_pads_cnt;
	int mipi_sensor_pads_cnt;
	int spinor_pads_cnt;
	int weim_pads_cnt;

	//INIT ALL PADS //
	common_pads = mx6q_sabreauto_pads;
	can0_pads = mx6q_sabreauto_can0_pads;
	mipi_sensor_pads = mx6q_sabreauto_mipi_sensor_pads;
	spinor_pads = mx6q_spinor_pads;
	weim_pads = mx6q_weim_pads;
	
	common_pads_cnt = ARRAY_SIZE(mx6q_sabreauto_pads);
	can0_pads_cnt = ARRAY_SIZE(mx6q_sabreauto_can0_pads);
	mipi_sensor_pads_cnt = ARRAY_SIZE(mx6q_sabreauto_mipi_sensor_pads);
	spinor_pads_cnt = ARRAY_SIZE(mx6q_spinor_pads);
	weim_pads_cnt = ARRAY_SIZE(mx6q_weim_pads);
	
	mxc_iomux_v3_setup_multiple_pads(mx6q_sabreauto_i2c1_pads,
			ARRAY_SIZE(mx6q_sabreauto_i2c1_pads));

	//mxc_iomux_v3_setup_multiple_pads(mx6q_sabreauto_i2c3_pads,
	//		ARRAY_SIZE(mx6q_sabreauto_i2c3_pads));
		
	iomux_v3_cfg_t mlb_pads[] = {
			MX6Q_PAD_ENET_TXD1__MLB_MLBCLK,
			MX6Q_PAD_GPIO_6__MLB_MLBSIG,
			MX6Q_PAD_GPIO_2__MLB_MLBDAT};

	mxc_iomux_v3_setup_multiple_pads(mlb_pads,
			ARRAY_SIZE(mlb_pads));
	
	BUG_ON(!common_pads);
	mxc_iomux_v3_setup_multiple_pads(common_pads, common_pads_cnt);

	/*If at least one NOR memory is selected we don't
	 * configure IC23 PADS for rev B */
	if (spinor_en) {
		BUG_ON(!spinor_pads);
		mxc_iomux_v3_setup_multiple_pads(spinor_pads, spinor_pads_cnt);
	} 

	if (can0_enable) {
		BUG_ON(!can0_pads);
		mxc_iomux_v3_setup_multiple_pads(can0_pads,
						can0_pads_cnt);
	}


	if (mipi_sensor) {
		BUG_ON(!mipi_sensor_pads);
		mxc_iomux_v3_setup_multiple_pads(mipi_sensor_pads,
						mipi_sensor_pads_cnt);
	}
	//INIT ALL PADS END //

	gp_reg_id = sabreauto_dvfscore_data.reg_id;
	soc_reg_id = sabreauto_dvfscore_data.soc_id;

	//UART 1-4
	mx6q_sabreauto_init_uart();

	//CSI 2 Port
	imx6q_add_mipi_csi2(&mipi_csi2_pdata);

	if (cpu_is_mx6dl()) {
		mipi_dsi_pdata.ipu_id = 0;
		mipi_dsi_pdata.disp_id = 1;
		ldb_data.ipu_id = 0;
		ldb_data.disp_id = 0;
		ldb_data.sec_ipu_id = 0;
		ldb_data.sec_disp_id = 1;
		hdmi_core_data.disp_id = 1;
	}

	//HDMI 
	imx6q_add_mxc_hdmi_core(&hdmi_core_data);

	//IPU
	imx6q_add_ipuv3(0, &ipu_data[0]);
	imx6q_add_ipuv3(1, &ipu_data[1]);
		for (i = 0; i < ARRAY_SIZE(sabr_fb_data); i++)
			imx6q_add_ipuv3fb(i, &sabr_fb_data[i]);

	//????
	imx6q_add_vdoa();

	//DSI Port
	imx6q_add_mipi_dsi(&mipi_dsi_pdata);

	//????
	imx6q_add_lcdif(&lcdif_data);

	//????
	imx6q_add_ldb(&ldb_data);

	//V4L2
	imx6q_add_v4l2_output(0);
	imx6q_add_v4l2_capture(0, &capture_data[0]);
	imx6q_add_v4l2_capture(1, &capture_data[1]);

	//RTC
	imx6q_add_imx_snvs_rtc();

	//CAAM
	imx6q_add_imx_caam();

	//I2C1
	imx6q_add_imx_i2c(0, &mx6q_sabreauto_i2c1_data);

	i2c_register_board_info(0, mxc_i2c1_board_info,
			ARRAY_SIZE(mxc_i2c1_board_info));

	//I2C2
	imx6q_add_imx_i2c(1, &mx6q_sabreauto_i2c2_data);

	i2c_register_board_info(1, mxc_i2c2_board_info,
			ARRAY_SIZE(mxc_i2c2_board_info));

	//I2C3  There is no I2C3 bus.  Pins are already being used.
	//imx6q_add_imx_i2c(2, &mx6q_sabreauto_i2c3_data);

	//i2c_register_board_info(2, mxc_i2c3_board_info,
	//		ARRAY_SIZE(mxc_i2c3_board_info));

	//PMIC INT init	
	ret = gpio_request(SABREAUTO_PMIC_INT, "pFUZE-int");
	if (ret) {
		printk(KERN_ERR"request pFUZE-int error!!\n");
		return;
	} else {
		gpio_direction_input(SABREAUTO_PMIC_INT);
		mx6q_sabreauto_init_pfuze100(SABREAUTO_PMIC_INT);
	}

	/* SPI */
	imx6q_add_ecspi(0, &mx6q_sabreauto_spi_data);
		if (spinor_en)
			spi_device_init();
	
	//HDMI INIT
	imx6q_add_mxc_hdmi(&hdmi_data);

	//Thermal IMX 	
	imx6q_add_anatop_thermal_imx(1, &mx6q_sabreauto_anatop_thermal_data);

	//IMX6 Gig Ethernet
	if (enet_to_gpio_6)
		/* Make sure the IOMUX_OBSRV_MUX1 is set to ENET_IRQ. */
		mxc_iomux_set_specialbits_register(
			IOMUX_OBSRV_MUX1_OFFSET,
			OBSRV_MUX1_ENET_IRQ,
			OBSRV_MUX1_MASK);
	else
		fec_data.gpio_irq = -1;

	//IMX6 Gig Ethernet
	imx6_init_fec(fec_data);
	
	//????	
	imx6q_add_pm_imx(0, &mx6q_sabreauto_pm_data);

	//SD2, and uSD3
	imx6q_add_sdhci_usdhc_imx(1, &mx6q_sabreauto_sd2_data);
	imx6q_add_sdhci_usdhc_imx(2, &mx6q_sabreauto_sd3_data);

	//GPU
	imx_add_viv_gpu(&imx6_gpu_data, &imx6q_gpu_pdata);

	//USB
	imx6q_sabreauto_init_usb();

	//SATA
	#ifdef CONFIG_SATA_AHCI_PLATFORM
		imx6q_add_ahci(0, &mx6q_sabreauto_sata_data);
	#else
		mx6q_sabreauto_sata_init(NULL,
			(void __iomem *)ioremap(MX6Q_SATA_BASE_ADDR, SZ_4K));
	#endif

	//VPU
	imx6q_add_vpu();

////////////////////////////////////////////////////////////////////////////////////////////Begin
	//Audio SGTL5000
	imx6q_init_audio();

	//SD-MMC
	platform_device_register(&sabreauto_vmmc_reg_devices);

	//Audio Clk
	imx_asrc_data.asrc_core_clk = clk_get(NULL, "asrc_clk");
	imx_asrc_data.asrc_audio_clk = clk_get(NULL, "asrc_serial_clk");
	imx6q_add_asrc(&imx_asrc_data);
////////////////////////////////////////////////////////////////////////////////////////////END

	/* MIPI Reset */
	gpio_request(SABREAUTO_DISP0_RESET, "disp0-reset");
	gpio_direction_output(SABREAUTO_DISP0_RESET, 0);
	/* MIPI PWR */
	gpio_request(SABREAUTO_DISP0_PWR, "disp0-pwr");
	gpio_direction_output(SABREAUTO_DISP0_PWR, 1);

	//????
	imx6q_add_otp();

	//????
	imx6q_add_viim();

	//Watchdog
	imx6q_add_imx2_wdt(0, NULL);

	//DMA	
	imx6q_add_dma();
	
	//NAND FLASH
	imx6q_add_gpmi(&mx6q_gpmi_nand_platform_data);

	//????
	imx6q_add_dvfs_core(&sabreauto_dvfscore_data);

	//PWM
	imx6q_add_mxc_pwm(1);
	imx6q_add_mxc_pwm(2);

	//Backlight PWM
	imx6q_add_mxc_pwm_backlight(1, &mx6_arm2_pwm_backlight_data3);
	imx6q_add_mxc_pwm_backlight(2, &mx6_arm2_pwm_backlight_data4);

	//Can0 Enable)
	imx6q_add_flexcan0(&mx6q_sabreauto_flexcan_pdata[0]);

	//HDMI 
	imx6q_add_hdmi_soc();
	imx6q_add_hdmi_soc_dai();

	//MLB150
	imx6q_add_mlb150(&mx6_sabreauto_mlb150_data);

	//????
	imx6q_add_busfreq();

	/* Add PCIe RC interface support */
	imx6q_add_pcie(&mx6_sabreauto_pcie_data);

	//????
	imx6q_add_perfmon(0);
	imx6q_add_perfmon(1);
	imx6q_add_perfmon(2);
	
	//WEIM INIT
	mx6q_setup_weimcs();
	mx6q_FGPA_VER();

};

extern void __iomem *twd_base;
static void __init mx6_timer_init(void)
{
	struct clk *uart_clk;
#ifdef CONFIG_LOCAL_TIMERS
	twd_base = ioremap(LOCAL_TWD_ADDR, SZ_256);
	BUG_ON(!twd_base);
#endif
	mx6_clocks_init(32768, 24000000, 0, 0);

	uart_clk = clk_get_sys("imx-uart.0", NULL);
	early_console_setup(UART1_BASE_ADDR, uart_clk);
};

static struct sys_timer mxc_timer = {
	.init = mx6_timer_init,
};

static void __init mx6q_reserve(void)
{
#if defined(CONFIG_MXC_GPU_VIV) || defined(CONFIG_MXC_GPU_VIV_MODULE)
	phys_addr_t phys;

	if (imx6q_gpu_pdata.reserved_mem_size) {
		phys = memblock_alloc_base(imx6q_gpu_pdata.reserved_mem_size,
			SZ_4K, SZ_2G);
		memblock_remove(phys, imx6q_gpu_pdata.reserved_mem_size);
		imx6q_gpu_pdata.reserved_mem_base = phys;
	}
#endif
};

MACHINE_START(MX6Q_SABREAUTO, "Freescale i.MX 6Quad/DualLite/Solo Sabre Auto Board")
	.boot_params	= MX6_PHYS_OFFSET + 0x100,
	.fixup		= fixup_mxc_board,
	.map_io		= mx6_map_io,
	.init_irq	= mx6_init_irq,
	.init_machine	= mx6_board_init,
	.timer		= &mxc_timer,
	.reserve	= mx6q_reserve,
MACHINE_END
