/*
 * Copyright (c) 2016  evilwombat
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/init.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/mm.h>
#include <linux/fb.h>
#include <linux/list.h>

#define VRAM_LEN	1536

struct st7571_data {
	spinlock_t		lock;
	struct spi_device	*spi;
	int			rst_gpio;

	struct fb_info 		*info;
	unsigned char		vram[VRAM_LEN];
};

static struct fb_var_screeninfo st7571_var = {
	.xres		= 96,
	.yres 		= 96,
	.xres_virtual 	= 96,
	.yres_virtual	= 96,
	.bits_per_pixel = 1,
	.red 		= {0, 1, 0},
	.green 		= {0, 1, 0},
	.blue 		= {0, 1, 0},
	.transp 	= {0, 0, 0},
	.height 	= -1,
	.width 		= -1,
};

static struct fb_fix_screeninfo st7571_fix = {
	.id 		= "st7571",
	.type 		= FB_TYPE_PACKED_PIXELS,
	.visual 	= FB_VISUAL_MONO10,
	.line_length 	= 16,
	.accel 		= FB_ACCEL_NONE
};

static void st7571fb_schedule_refresh(struct fb_info *info, const struct fb_fillrect *rect);

static int st7571_open(struct fb_info *info, int init)
{
	return 0;
}

static int st7571_release(struct fb_info *info, int init)
{
	return 0;
}

static int st7571_setcolreg(u_int regno, u_int red, u_int green, u_int blue,
			   u_int transp, struct fb_info *info)
{
	if (regno > 1)
		return 1;
	return 0;
}

static int st7571_pan_display(struct fb_var_screeninfo *var,
			     struct fb_info *info)
{
	st7571fb_schedule_refresh(info, NULL);
	return 0;
}

static int st7571_blank(int blank_mode, struct fb_info *info)
{
	/* TODO: Write to LCD blanking control register */
	return 0;
}

static void st7571_fillrect(struct fb_info *info, const struct fb_fillrect *rect)
{
	sys_fillrect(info, rect);
	st7571fb_schedule_refresh(info, rect);
}

static void st7571_copyarea(struct fb_info *info, const struct fb_copyarea *area)
{
	sys_copyarea(info, area);
	st7571fb_schedule_refresh(info, NULL);
}

static void st7571_imageblit(struct fb_info *info, const struct fb_image *image)
{
	sys_imageblit(info, image);
	st7571fb_schedule_refresh(info, NULL);
}

static ssize_t st7571_write(struct fb_info *info, const char __user *buf, size_t count, loff_t *ppos)
{
	ssize_t ret = fb_sys_write(info, buf, count, ppos);
	st7571fb_schedule_refresh(info, NULL);
	return ret;
}

static struct fb_ops st7571fb_ops = {
	.owner		= THIS_MODULE,
	.fb_open	= st7571_open,
	.fb_write	= st7571_write,
	.fb_release	= st7571_release,
	.fb_setcolreg	= st7571_setcolreg,
	.fb_pan_display	= st7571_pan_display,
	.fb_blank	= st7571_blank,
	.fb_fillrect	= st7571_fillrect,
	.fb_copyarea	= st7571_copyarea,
	.fb_imageblit	= st7571_imageblit,
};


static int spi_send_word(struct st7571_data *drvdata, unsigned char val)
{
	int ret = spi_write(drvdata->spi, (u8 *) &val, 1);
	return ret;
}

static void st7571_init_controller(struct st7571_data *drvdata)
{
	int i, j;

	gpio_set_value(drvdata->rst_gpio, 1);
	msleep(100);
	gpio_set_value(drvdata->rst_gpio, 0);

	/* Reset */
	spi_send_word(drvdata, 0xe2);
	spi_send_word(drvdata, 0xe2);
	spi_send_word(drvdata, 0xe2);

	msleep(100);

	spi_send_word(drvdata, 0xae); // display off

	spi_send_word(drvdata, 0x38); // mode
	spi_send_word(drvdata, 0xb8);

	spi_send_word(drvdata, 0xa1); // ADC select ?
	spi_send_word(drvdata, 0xc8); // SHL select ??

	spi_send_word(drvdata, 0x40); // start line
	spi_send_word(drvdata, 0x00);

	spi_send_word(drvdata, 0x44); // com0
	spi_send_word(drvdata, 0x00);

	spi_send_word(drvdata, 0xab); // osc on

	spi_send_word(drvdata, 0x67); // boost 8x ?
	spi_send_word(drvdata, 0x26); // 27 Resulator resistor

	spi_send_word(drvdata, 0x81);
	spi_send_word(drvdata, 0x2d); // contrast

	spi_send_word(drvdata, 0x54); // 57 bias ?


	spi_send_word(drvdata, 0xf3); // bias powersave off ??
	spi_send_word(drvdata, 0x04);

	spi_send_word(drvdata, 0x93); // FRC and PWM ??

	spi_send_word(drvdata, 0x2c); // Power control
	msleep(200);

	spi_send_word(drvdata, 0x2e); // Power control
	msleep(200);

	spi_send_word(drvdata, 0x2f); // Power control
	msleep(10);

	spi_send_word(drvdata, 0xaf); // disp on

	spi_send_word(drvdata, 0x7b); // mode 3
	spi_send_word(drvdata, 0x11); // b/w
	spi_send_word(drvdata, 0x00); // normal

	msleep(200);

	for (j = 0; j < 13; j++) {
		spi_send_word(drvdata, 0xb0 | j); // page 0
		spi_send_word(drvdata, 0x10); // col 0
		spi_send_word(drvdata, 0x00);

		spi_send_word(drvdata, 0xe8); // data length
		spi_send_word(drvdata, 127);

		for (i = 0; i < 128; i++)
			spi_send_word(drvdata, 0);

	}
}

static void st7571fb_deferred_io(struct fb_info *info, struct list_head *pagelist)
{
	unsigned int row, col, t, i;
	unsigned char *vram = info->screen_base;
	struct st7571_data *dd = info->par;

	for (col = 0; col < 12; col++) {
		spi_send_word(dd, 0xb0 | col);

		spi_send_word(dd, 0x10); // col 0
		spi_send_word(dd, 0x00);

		spi_send_word(dd, 0xe8); // data length
		spi_send_word(dd, 111);

		for (i = 0; i < 16; i++)
			spi_send_word(dd, 0);

		for (row = 0; row < 96; row++) {
			t = 0;
			for (i = 0; i < 8; i++)
				if (vram[col * 16 * 8 + i * 16 + (row >> 3)] & (1 << (row & 0x07)))
					t |= (1 << i);

			spi_send_word(dd, t);
		}
	}
}

static struct fb_deferred_io st7571fb_defio = {
	.delay		= HZ / 10,
	.deferred_io	= st7571fb_deferred_io,
};


static void st7571fb_schedule_refresh(struct fb_info *info, const struct fb_fillrect *rect)
{
	if (!info->fbdefio)
		return;

	schedule_delayed_work(&info->deferred_work, info->fbdefio->delay);
}

static int st7571_probe(struct spi_device *spi)
{
	int err, ret;
	struct st7571_data *drvdata;
	struct device_node *np = spi->dev.of_node;

	drvdata = devm_kzalloc(&spi->dev, sizeof(*drvdata), GFP_KERNEL);

	if (!drvdata)
		return -ENOMEM;

	drvdata->rst_gpio = of_get_named_gpio(np, "sitronix,reset-gpio", 0);

	spi->mode = 3;
	spi->bits_per_word = 8;
	spi->max_speed_hz = 1000000;

	err = spi_setup(spi);
	if (err < 0) {
		dev_err(&spi->dev, "spi_setup failed!\n");
		return err;
	}

	drvdata->spi = spi;
	spi_set_drvdata(spi, drvdata);

	drvdata->info = framebuffer_alloc(0, &spi->dev);
	if (!drvdata->info) {
		return -ENOMEM;
	}

	st7571_fix.smem_start = (unsigned long)drvdata->vram;
	st7571_fix.smem_len = VRAM_LEN;

	drvdata->info->par = drvdata;
	drvdata->info->flags = FBINFO_DEFAULT;
	drvdata->info->var = st7571_var;
	drvdata->info->fix = st7571_fix;
	drvdata->info->monspecs.hfmin = 0;
	drvdata->info->monspecs.hfmax = 0;
	drvdata->info->monspecs.vfmin = 10000;
	drvdata->info->monspecs.vfmax = 10000;
	drvdata->info->monspecs.dpms = 0;
	drvdata->info->fbops = &st7571fb_ops;
	drvdata->info->screen_base = drvdata->vram;
	drvdata->info->fbdefio = &st7571fb_defio;
	fb_deferred_io_init(drvdata->info);

	st7571_init_controller(drvdata);

	ret = register_framebuffer(drvdata->info);

	if (ret < 0) {
		framebuffer_release(drvdata->info);
		return -EINVAL;
	}

        printk(KERN_INFO "fb%d: %s frame buffer device\n",
	       drvdata->info->node, drvdata->info->fix.id);

	return 0;
}

static int st7571_remove(struct spi_device *spi)
{
	struct st7571_data *drvdata = spi_get_drvdata(spi);

	spin_lock_irq(&drvdata->lock);
	drvdata->spi = NULL;
	spi_set_drvdata(spi, NULL);

	if (drvdata->info) {
		unregister_framebuffer(drvdata->info);
		fb_deferred_io_cleanup(drvdata->info);
		framebuffer_release(drvdata->info);
	}
	spin_unlock_irq(&drvdata->lock);
	return 0;
}

static const struct of_device_id st7571_of_match[] = {
	{ .compatible = "sitronix,st7571", },
	{ },
};
MODULE_DEVICE_TABLE(of, st7571_of_match);

static const struct spi_device_id st7571_id[] = {
	{ "st7571", 0 },

	{ }
};
MODULE_DEVICE_TABLE(spi, st7571_id);

static struct spi_driver st7571_spi_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= "st7571",
		.of_match_table = st7571_of_match
	},
	.id_table	= st7571_id,
	.probe		= st7571_probe,
	.remove		= st7571_remove
};

module_spi_driver(st7571_spi_driver);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("evilwombat");
MODULE_DESCRIPTION("Sitronix ST7571 LCD Controller Driver");
