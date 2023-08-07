// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2018 NXP
 */
#include <common.h>
#include <efi_loader.h>
#include <env.h>
#include <init.h>
#include <miiphy.h>
#include <netdev.h>
#include <asm/global_data.h>
#include <asm/mach-imx/iomux-v3.h>
#include <asm-generic/gpio.h>
#include <asm/arch/imx8mm_pins.h>
#include <asm/arch/clock.h>
#include <asm/arch/sys_proto.h>
#include <asm/mach-imx/gpio.h>
#include <asm/mach-imx/mxc_i2c.h>
#include <i2c.h>
#include <asm/io.h>
#include <usb.h>
#include <imx_sip.h>
#include <linux/arm-smccc.h>

DECLARE_GLOBAL_DATA_PTR;

#define UART_PAD_CTRL	(PAD_CTL_DSE6 | PAD_CTL_FSEL1)
#define WDOG_PAD_CTRL	(PAD_CTL_DSE6 | PAD_CTL_ODE | PAD_CTL_PUE | PAD_CTL_PE)

static iomux_v3_cfg_t const uart_pads[] = {
  IMX8MM_PAD_UART3_TXD_UART3_TX | MUX_PAD_CTRL(UART_PAD_CTRL),
  IMX8MM_PAD_UART3_RXD_UART3_RX | MUX_PAD_CTRL(UART_PAD_CTRL),
};

static iomux_v3_cfg_t const wdog_pads[] = {
	IMX8MM_PAD_GPIO1_IO02_WDOG1_WDOG_B  | MUX_PAD_CTRL(WDOG_PAD_CTRL),
};

#if CONFIG_IS_ENABLED(EFI_HAVE_CAPSULE_SUPPORT)
struct efi_fw_image fw_images[] = {
	{
		.image_type_id = IMX_BOOT_IMAGE_GUID,
		.fw_name = u"IMX8MM-MOD-RBZ-RAW",
		.image_index = 1,
	},
};

struct efi_capsule_update_info update_info = {
	.dfu_string = "mmc 2=flash-bin raw 0x42 0x2000 mmcpart 1",
	.images = fw_images,
};

u8 num_image_type_guids = ARRAY_SIZE(fw_images);
#endif /* EFI_HAVE_CAPSULE_SUPPORT */

int board_early_init_f(void)
{
	struct wdog_regs *wdog = (struct wdog_regs *)WDOG1_BASE_ADDR;

	imx_iomux_v3_setup_multiple_pads(wdog_pads, ARRAY_SIZE(wdog_pads));

	set_wdog_reset(wdog);

	imx_iomux_v3_setup_multiple_pads(uart_pads, ARRAY_SIZE(uart_pads));
	init_uart_clk(2);

	return 0;
}

#define EEPROM_MAC_OFFSET 0x9A

int board_read_rom_ethaddr(void)
{
  int ret = -EINVAL;
  unsigned char mac[6] = {};
  struct udevice *dev;
  ofnode eeprom;

  eeprom = ofnode_get_chosen_node("rbz,eeprom");
  if (!ofnode_valid(eeprom))
    return -ENODEV;

  ret = uclass_get_device_by_ofnode(UCLASS_I2C_EEPROM, eeprom, &dev);
  if (ret)
    return ret;

  ret = dm_i2c_read(dev, EEPROM_MAC_OFFSET, mac, 6);
  if (ret) {
    debug("%s: I2C EEPROM MAC address read failed\n", __func__);
    return ret;
  }
  
  if (is_valid_ethaddr(mac))
    eth_env_set_enetaddr("ethaddr", mac);

  return 0;
}

#if IS_ENABLED(CONFIG_FEC_MXC)
static int setup_fec(void)
{
	struct iomuxc_gpr_base_regs *gpr =
		(struct iomuxc_gpr_base_regs *)IOMUXC_GPR_BASE_ADDR;

	/* Use 125M anatop REF_CLK1 for ENET1, not from external */
	clrsetbits_le32(&gpr->gpr[1], 0x2000, 0);

	return 0;
}

int board_phy_config(struct phy_device *phydev)
{
	if (phydev->drv->config)
		phydev->drv->config(phydev);

  /* Config LED0 for 10/100/1000 Link status */
  phy_write(phydev, MDIO_DEVAD_NONE, 0x0d, 0x001f);
  phy_write(phydev, MDIO_DEVAD_NONE, 0x0e, 0x01e3);
  phy_write(phydev, MDIO_DEVAD_NONE, 0x0d, 0x401f);
  phy_write(phydev, MDIO_DEVAD_NONE, 0x0e, 0x0000);

  /* Config LED1 for blinking with activity */
  phy_write(phydev, MDIO_DEVAD_NONE, 0x0d, 0x001f);
  phy_write(phydev, MDIO_DEVAD_NONE, 0x0e, 0x01e4);
  phy_write(phydev, MDIO_DEVAD_NONE, 0x0d, 0x401f);
  phy_write(phydev, MDIO_DEVAD_NONE, 0x0e, 0x0000);

  phy_write(phydev, MDIO_DEVAD_NONE, 0x0d, 0x001f);
  phy_write(phydev, MDIO_DEVAD_NONE, 0x0e, 0x01e5);
  phy_write(phydev, MDIO_DEVAD_NONE, 0x0d, 0x401f);
  phy_write(phydev, MDIO_DEVAD_NONE, 0x0e, 0x0003);

  /* Invert LED0 and LED1 polarity */
  phy_write(phydev, MDIO_DEVAD_NONE, 0x1b, 0x3300);

	return 0;
}
#endif

#define DISPMIX				9
#define MIPI				10

int board_init(void)
{
	struct arm_smccc_res res;

	if (IS_ENABLED(CONFIG_FEC_MXC))
		setup_fec();

	arm_smccc_smc(IMX_SIP_GPC, IMX_SIP_GPC_PM_DOMAIN,
		      DISPMIX, true, 0, 0, 0, 0, &res);
	arm_smccc_smc(IMX_SIP_GPC, IMX_SIP_GPC_PM_DOMAIN,
		      MIPI, true, 0, 0, 0, 0, &res);

	return 0;
}

int board_late_init(void)
{
	board_late_mmc_env_init();

#ifdef CONFIG_ENV_VARS_UBOOT_RUNTIME_CONFIG
		env_set("board_name", "MOD_IMX8M_MINI");
		env_set("board_rev", "RV01");
#endif

    board_read_rom_ethaddr();
	return 0;
}

#ifdef CONFIG_ANDROID_SUPPORT
bool is_power_key_pressed(void) {
	return (bool)(!!(readl(SNVS_HPSR) & (0x1 << 6)));
}
#endif

#ifdef CONFIG_FSL_FASTBOOT
#ifdef CONFIG_ANDROID_RECOVERY
int is_recovery_key_pressing(void)
{
	return 0; /* TODO */
}
#endif /* CONFIG_ANDROID_RECOVERY */
#endif /* CONFIG_FSL_FASTBOOT */
