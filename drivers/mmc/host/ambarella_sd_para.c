/*
 * drivers/mmc/host/ambarella_sd_para.c
 *
 * Copyright (C) 2015, Ambarella, Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/slab.h>
#include <linux/blkdev.h>
#include <linux/scatterlist.h>
#include <linux/mmc/host.h>
#include <linux/mmc/card.h>
#include <linux/mmc/mmc.h>
#include <linux/irq.h>
#include <asm/dma.h>
#include <plat/rct.h>
#include <mach/hardware.h>
#include <plat/sd.h>
#include <plat/event.h>

static int ambarella_sdio_ds_value_2ma(u32 value)
{
	enum amba_sdio_ds_value {
		DS_3MA = 0,
		DS_12MA,
		DS_6MA,
		DS_18MA
	};

	switch (value) {
	case DS_3MA:
		return 3;
	case DS_12MA:
		return 12;
	case DS_6MA:
		return 6;
	case DS_18MA:
		return 18;
	default:
		printk(KERN_ERR "%s: unknown driving strength\n", __func__);
		return 0;
	}
}

static int sdio_info_proc_read(struct seq_file *m, void *v)
{
	int len = 0;
	u32 reg, ds1, ds_value, b0, b1;

	reg = amba_readl(SD2_REG(SD_HOST_OFFSET));
	pr_debug("amba_readl 0x%x = 0x%x\n", SD2_REG(SD_HOST_OFFSET), reg);
	if (reg & SD_HOST_HIGH_SPEED)
		len += seq_printf(m, "SDIO high speed mode:            yes\n");
	else
		len += seq_printf(m, "SDIO high speed mode:            no\n");

#ifdef CONFIG_PLAT_AMBARELLA_S2L
	len += seq_printf(m, "SDIO output data delay:          0\n");
	len += seq_printf(m, "SDIO output clock delay:         0\n");
	len += seq_printf(m, "SDIO input data delay:           0\n");
	len += seq_printf(m, "SDIO input clock delay:          0\n");

	reg = amba_rct_readl(GPIO_DS0_2_REG);
	pr_debug("b0 amba_rct_readl 0x%x = 0x%x\n", GPIO_DS0_2_REG, reg);
	ds1 = amba_rct_readl(GPIO_DS1_2_REG);
	pr_debug("b1 amba_rct_readl 0x%x = 0x%x\n", GPIO_DS1_2_REG, ds1);
	// [22:19] SDIO data, assume all bits have same value
	b0 = (reg & (0x1 << 19)) >> 19;
	b1 = (ds1 & (0x1 << 19)) >> 19;
	ds_value = b0 + (b1 << 0x1);
	len += seq_printf(m, "SDIO data driving strength:      %u mA(%d)\n",
		ambarella_sdio_ds_value_2ma(ds_value), ds_value);
	// [17] SDIO clock
	b0 = (reg & (0x1 << 17)) >> 17;
	b1 = (ds1 & (0x1 << 17)) >> 17;
	ds_value = b0 + (b1 << 0x1);
	len += seq_printf(m, "SDIO clock driving strength:     %u mA(%d)\n",
		ambarella_sdio_ds_value_2ma(ds_value), ds_value);
	// [18] SDIO cmd
	b0 = (reg & (0x1 << 18)) >> 18;
	b1 = (ds1 & (0x1 << 18)) >> 18;
	ds_value = b0 + (b1 << 0x1);
	len += seq_printf(m, "SDIO command driving strength:   %u mA(%d)\n",
		ambarella_sdio_ds_value_2ma(ds_value), ds_value);
	// [24:23] SDIO cdwp, assume both bits have same value
	b0 = (reg & (0x1 << 23)) >> 23;
	b1 = (ds1 & (0x1 << 23)) >> 23;
	ds_value = b0 + (b1 << 0x1);
	len += seq_printf(m, "SDIO CD and WP driving strength: %u mA(%d)\n",
		ambarella_sdio_ds_value_2ma(ds_value), ds_value);
#else
#ifndef IOCTRL_GPIO_DS0_2
#define IOCTRL_GPIO_DS0_2		RCT_REG(0x278)
#endif
#ifndef IOCTRL_GPIO_DS1_2
#define IOCTRL_GPIO_DS1_2		RCT_REG(0x28c)
#endif
	reg = amba_rct_readl(MS_DELAY_CTRL_REG);
	pr_debug("amba_rct_readl 0x%x = 0x%x\n", MS_DELAY_CTRL_REG, reg);
	//[31:29] SDIO output data delay control
	len += seq_printf(m, "SDIO output data delay:          %u\n",
		(reg & (0x7 << 29)) >> 29);
	//[23:21] SDIO output clock delay control
	len += seq_printf(m, "SDIO output clock delay:         %u\n",
		(reg & (0x7 << 21)) >> 21);
	//[15:13] SDIO input data delay control
	len += seq_printf(m, "SDIO input data delay:           %u\n",
		(reg & (0x7 << 13)) >> 13);
	//[7:5] SDIO clock input delay control
	len += seq_printf(m, "SDIO input clock delay:          %u\n",
		(reg & (0x7 << 5)) >> 5);

	reg = amba_rct_readl(IOCTRL_GPIO_DS0_2);
	pr_debug("b0 amba_rct_readl 0x%x = 0x%x\n", IOCTRL_GPIO_DS0_2, reg);
	ds1 = amba_rct_readl(IOCTRL_GPIO_DS1_2);
	pr_debug("b1 amba_rct_readl 0x%x = 0x%x\n", IOCTRL_GPIO_DS1_2, ds1);

	// [10:7] SDIO data, assume all bits have same value
	b0 = (reg & (0x1 << 7)) >> 7;
	b1 = (ds1 & (0x1 << 7)) >> 7;
	ds_value = b0 + (b1 << 0x1);
	len += seq_printf(m, "SDIO data driving strength:      %u mA(%d)\n",
		ambarella_sdio_ds_value_2ma(ds_value), ds_value);
	// [5] SDIO clock,
	b0 = (reg & (0x1 << 5)) >> 5;
	b1 = (ds1 & (0x1 << 5)) >> 5;
	ds_value = b0 + (b1 << 0x1);
	len += seq_printf(m, "SDIO clock driving strength:     %u mA(%d)\n",
		ambarella_sdio_ds_value_2ma(ds_value), ds_value);
	// [6] SDIO cmd,
	b0 = (reg & (0x1 << 6)) >> 6;
	b1 = (ds1 & (0x1 << 6)) >> 6;
	ds_value = b0 + (b1 << 0x1);
	len += seq_printf(m, "SDIO command driving strength:   %u mA(%d)\n",
		ambarella_sdio_ds_value_2ma(ds_value), ds_value);
	// [12:11] SDIO cdwp, assume both bits have same value
	b0 = (reg & (0x1 << 11)) >> 11;
	b1 = (ds1 & (0x1 << 11)) >> 11;
	ds_value = b0 + (b1 << 0x1);
	len += seq_printf(m, "SDIO CD and WP driving strength: %u mA(%d)\n",
		ambarella_sdio_ds_value_2ma(ds_value), ds_value);
#endif
	return len;
}

static int sdio_info_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, sdio_info_proc_read, NULL);
}

const struct file_operations proc_fops_sdio_info = {
	.open = sdio_info_proc_open,
	.read = seq_read,
	.llseek	= seq_lseek,
	.release = single_release,
};

/*
 * return 0 if timing changed
 */
int amba_sdio_delay_post_apply(const int odly, const int ocly,
	const int idly, const int icly)
{
#ifdef CONFIG_PLAT_AMBARELLA_S2L
	printk(KERN_ERR "err: not supported!\n");
	return 1;	//not changed
#else
	u32 reg_ori, reg_new;
	int i;
	int biti;

	reg_ori = amba_rct_readl(MS_DELAY_CTRL_REG);
	reg_new = reg_ori;
	pr_debug("readl 0x%08x = 0x%08x\n", MS_DELAY_CTRL_REG, reg_ori);

	//[31:29] SDIO output data delay control
	if (-1 != odly) {
		for (i = 0; i < 3; i++) {
			biti = odly >> i;
			if (biti & 1)
				reg_new = reg_new | BIT(29 + i);
			else
				reg_new = reg_new & ~BIT(29 + i);
		}
	}

	//[23:21] SDIO output clock delay control
	if (-1 != ocly) {
		for (i = 0; i < 3; i++) {
			biti = ocly >> i;
			if (biti & 1)
				reg_new = reg_new | BIT(21 + i);
			else
				reg_new = reg_new & ~BIT(21 + i);
		}
	}

	//[15:13] SDIO input data delay control
	if (-1 != idly) {
		for (i = 0; i < 3; i++) {
			biti = idly >> i;
			if (biti & 1)
				reg_new = reg_new | BIT(13 + i);
			else
				reg_new = reg_new & ~BIT(13 + i);
		}
	}

	//[7:5] SDIO clock input delay control
	if (-1 != icly) {
		for (i = 0; i < 3; i++) {
			biti = icly >> i;
			if (biti & 1)
				reg_new = reg_new | BIT(5 + i);
			else
				reg_new = reg_new & ~BIT(5 + i);
		}
	}

	if (reg_ori != reg_new) {
		amba_rct_writel(MS_DELAY_CTRL_REG, reg_new);
		pr_debug("amba_rct_writel 0x%x 0x%x\n", MS_DELAY_CTRL_REG, reg_new);
		return 0;
	} else
		return 1;	//not changed
#endif
}

/*
 * return 0 if timing changed
 */
int amba_sdio_ds_post_apply(const int clk_ds, const int data_ds,
	const int cmd_ds, const int cdwp_ds)
{
	int biti, ret = 1;
	u32 reg_ori, reg_new;

#ifdef CONFIG_PLAT_AMBARELLA_S2L
	enum amba_sdio_ds {
		DS_CLK_MASK = 0x00020000, /* clock [17], SMIO 26 / GPIO 81 */
		DS_DATA_MASK = 0x00780000, /* data [22:19], SMIO 28~31 / GPIO 83~86 */
		DS_CMD_MASK = 0x00040000, /* cmd [18], SMIO 27 / GPIO 82 */
		DS_CDWP_MASK = 0x01800000, /* cdwp [24:23], SMIO 32~33 / GPIO 88~89 */
	};

	/* DS0 */
	reg_ori = amba_rct_readl(GPIO_DS0_2_REG);
	reg_new = reg_ori;
	pr_debug("readl 0x%08x = 0x%08x\n", GPIO_DS0_2_REG, reg_ori);

	if (-1 != clk_ds) {
		biti = clk_ds & 0x1;
		if (biti)
			reg_new |= DS_CLK_MASK;
		else
			reg_new &= ~DS_CLK_MASK;
	}
	if (-1 != data_ds) {
		biti = data_ds & 0x1;
		if (biti)
			reg_new |= DS_DATA_MASK;
		else
			reg_new &= ~DS_DATA_MASK;
	}
	if (-1 != cmd_ds) {
		biti = cmd_ds & 0x1;
		if (biti)
			reg_new |= DS_CMD_MASK;
		else
			reg_new &= ~DS_CMD_MASK;
	}
	if (-1 != cdwp_ds) {
		biti = cdwp_ds & 0x1;
		if (biti)
			reg_new |= DS_CDWP_MASK;
		else
			reg_new &= ~DS_CDWP_MASK;
	}

	if (reg_ori != reg_new) {
		amba_rct_writel(GPIO_DS0_2_REG, reg_new);
		pr_debug("amba_rct_writel 0x%x 0x%x\n", GPIO_DS0_2_REG, reg_new);
		ret = 0;
	}

	/* DS1 */
	reg_ori = amba_rct_readl(GPIO_DS1_2_REG);
	reg_new = reg_ori;
	pr_debug("readl 0x%08x = 0x%08x\n", GPIO_DS1_2_REG, reg_ori);

	if (-1 != clk_ds) {
		biti = (clk_ds & 0x2) >> 1;
		if (biti)
			reg_new |= DS_CLK_MASK;
		else
			reg_new &= ~DS_CLK_MASK;
	}
	if (-1 != data_ds) {
		biti = (data_ds & 0x2) >> 1;
		if (biti)
			reg_new |= DS_DATA_MASK;
		else
			reg_new &= ~DS_DATA_MASK;
	}
	if (-1 != cmd_ds) {
		biti = (cmd_ds & 0x2) >> 1;
		if (biti)
			reg_new |= DS_CMD_MASK;
		else
			reg_new &= ~DS_CMD_MASK;
	}
	if (-1 != cdwp_ds) {
		biti = (cdwp_ds & 0x2) >> 1;
		if (biti)
			reg_new |= DS_CDWP_MASK;
		else
			reg_new &= ~DS_CDWP_MASK;
	}
	if (reg_ori != reg_new) {
		amba_rct_writel(GPIO_DS1_2_REG, reg_new);
		pr_debug("amba_rct_writel 0x%x 0x%x\n", GPIO_DS1_2_REG, reg_new);
		ret = 0;
	}

	return ret;
#else
#ifndef IOCTRL_GPIO_DS0_2
#define IOCTRL_GPIO_DS0_2		RCT_REG(0x278)
#endif
#ifndef IOCTRL_GPIO_DS1_2
#define IOCTRL_GPIO_DS1_2		RCT_REG(0x28c)
#endif
	enum amba_sdio_ds {
		DS_CLK_MASK = 0x00000020, /* clock [5], SMIO 38 / GPIO 69 */
		DS_DATA_MASK = 0x00000780, /* data [10:7], SMIO 40~43 / GPIO 71~74 */
		DS_CMD_MASK = 0x00000040, /* cmd [6], SMIO 39 / GPIO 70 */
		DS_CDWP_MASK = 0x00001800, /* cdwp [12:11], SMIO 44~45 / GPIO 75~76 */
	};

	/* DS0 */
	reg_ori = amba_rct_readl(IOCTRL_GPIO_DS0_2);
	reg_new = reg_ori;
	pr_debug("readl 0x%08x = 0x%08x\n", IOCTRL_GPIO_DS0_2, reg_ori);

	if (-1 != clk_ds) {
		biti = clk_ds & 0x1;
		if (biti)
			reg_new |= DS_CLK_MASK;
		else
			reg_new &= ~DS_CLK_MASK;
	}
	if (-1 != data_ds) {
		biti = data_ds & 0x1;
		if (biti)
			reg_new |= DS_DATA_MASK;
		else
			reg_new &= ~DS_DATA_MASK;
	}
	if (-1 != cmd_ds) {
		biti = cmd_ds & 0x1;
		if (biti)
			reg_new |= DS_CMD_MASK;
		else
			reg_new &= ~DS_CMD_MASK;
	}
	if (-1 != cdwp_ds) {
		biti = cdwp_ds & 0x1;
		if (biti)
			reg_new |= DS_CDWP_MASK;
		else
			reg_new &= ~DS_CDWP_MASK;
	}

	if (reg_ori != reg_new) {
		amba_rct_writel(IOCTRL_GPIO_DS0_2, reg_new);
		pr_debug("amba_rct_writel 0x%x 0x%x\n", IOCTRL_GPIO_DS0_2, reg_new);
		ret = 0;
	}

	/* DS1 */
	reg_ori = amba_rct_readl(IOCTRL_GPIO_DS1_2);
	reg_new = reg_ori;
	pr_debug("readl 0x%08x = 0x%08x\n", IOCTRL_GPIO_DS1_2, reg_ori);

	if (-1 != clk_ds) {
		biti = (clk_ds & 0x2) >> 1;
		if (biti)
			reg_new |= DS_CLK_MASK;
		else
			reg_new &= ~DS_CLK_MASK;
	}
	if (-1 != data_ds) {
		biti = (data_ds & 0x2) >> 1;
		if (biti)
			reg_new |= DS_DATA_MASK;
		else
			reg_new &= ~DS_DATA_MASK;
	}
	if (-1 != cmd_ds) {
		biti = (cmd_ds & 0x2) >> 1;
		if (biti)
			reg_new |= DS_CMD_MASK;
		else
			reg_new &= ~DS_CMD_MASK;
	}
	if (-1 != cdwp_ds) {
		biti = (cdwp_ds & 0x2) >> 1;
		if (biti)
			reg_new |= DS_CDWP_MASK;
		else
			reg_new &= ~DS_CDWP_MASK;
	}
	if (reg_ori != reg_new) {
		amba_rct_writel(IOCTRL_GPIO_DS1_2, reg_new);
		pr_debug("amba_rct_writel 0x%x 0x%x\n", IOCTRL_GPIO_DS1_2, reg_new);
		ret = 0;
	}

	return ret;
#endif
}

int ambarella_set_sdio_host_high_speed(const char *str, const struct kernel_param *kp)
{
	int retval;
	int value;
	u8 hostr;

	param_set_int(str, kp);
	retval = kstrtos32(str, 10, &value);
	hostr = amba_readb((unsigned char *)SD2_REG(SD_HOST_OFFSET));
	pr_debug("amba_readb 0x%x = 0x%x\n", SD2_REG(SD_HOST_OFFSET), hostr);

	if(value == 1)
		hostr |= SD_HOST_HIGH_SPEED;
	else if(value == 0)
		hostr &= ~SD_HOST_HIGH_SPEED;

	amba_writeb((unsigned char *)(SD2_REG(SD_HOST_OFFSET)), hostr);
	pr_debug("amba_writeb 0x%x 0x%x\n", SD2_REG(SD_HOST_OFFSET), hostr);
	return retval;
}

int ambarella_set_sdio_clk_ds(const char *str, const struct kernel_param *kp)
{
	int retval;
	int value;

	param_set_int(str, kp);
	retval = kstrtos32(str, 10, &value);
	amba_sdio_ds_post_apply(value, -1, -1, -1);

	return retval;
}

int ambarella_set_sdio_data_ds(const char *str, const struct kernel_param *kp)
{
	int retval;
	int value;

	param_set_int(str, kp);
	retval = kstrtos32(str, 10, &value);
	amba_sdio_ds_post_apply(-1, value, -1, -1);

	return retval;
}

int ambarella_set_sdio_cmd_ds(const char *str, const struct kernel_param *kp)
{
	int retval;
	int value;

	param_set_int(str, kp);
	retval = kstrtos32(str, 10, &value);
	amba_sdio_ds_post_apply(-1, -1, value, -1);

	return retval;
}

int ambarella_set_sdio_host_odly(const char *str, const struct kernel_param *kp)
{
	int retval;
	int value;

	param_set_int(str, kp);
	retval = kstrtos32(str, 10, &value);
	amba_sdio_delay_post_apply(value, -1, -1, -1);

	return retval;
}

int ambarella_set_sdio_host_ocly(const char *str, const struct kernel_param *kp)
{
	int retval;
	int value;

	param_set_int(str, kp);
	retval = kstrtos32(str, 10, &value);
	amba_sdio_delay_post_apply(-1, value, -1, -1);

	return retval;
}

int ambarella_set_sdio_host_idly(const char *str, const struct kernel_param *kp)
{
	int retval;
	int value;

	param_set_int(str, kp);
	retval = kstrtos32(str, 10, &value);
	amba_sdio_delay_post_apply(-1, -1, value, -1);

	return retval;
}

int ambarella_set_sdio_host_icly(const char *str, const struct kernel_param *kp)
{
	int retval;
	int value;

	param_set_int(str, kp);
	retval = kstrtos32(str, 10, &value);
	amba_sdio_delay_post_apply(-1, -1, -1, value);

	return retval;
}
