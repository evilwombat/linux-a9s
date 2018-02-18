/*
 * linux/drivers/spi/spi_ambarella.c
 *
 * History:
 *	2008/03/03 - [Louis Sun]  created file
 *	2009/06/19 - [Zhenwu Xue] ported from 2.6.22.10

 *
 * Copyright (C) 2004-2012, Ambarella, Inc.
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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 */

#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>
#include <asm/io.h>
#include <mach/io.h>
#include <plat/spi.h>
#include <plat/ambcache.h>

/*============================Global Variables================================*/

struct ambarella_spi {
	void __iomem				*regbase;
	struct clk				*clk;

	int					irq;
	struct tasklet_struct			tasklet;

	spinlock_t				lock;
	struct list_head			queue;
	u32					idle;
	u32					shutdown;

	struct spi_device			*c_dev;
	struct spi_message			*c_msg;
	struct spi_transfer			*c_xfer;

	u8					rw_mode, bpw, chip_select;
	u32					ridx, widx, len;
	u32					clk_freq;
};

struct ambarella_spi_private {
	struct spi_device			*spi;
	struct mutex				mtx;
	spinlock_t				lock;
	struct list_head			list;
};

static LIST_HEAD(ambarella_ps_list);

static void ambarella_spi_handle_message(struct ambarella_spi *);
static void ambarella_spi_prepare_message(struct ambarella_spi *);
static void ambarella_spi_prepare_transfer(struct ambarella_spi *);
static void ambarella_spi_finish_transfer(struct ambarella_spi *);
static void ambarella_spi_finish_message(struct ambarella_spi *);
static void ambarella_spi_start_transfer(struct ambarella_spi *);

/*============================SPI Bus Driver==================================*/
static int ambarella_spi_setup(struct spi_device *spi)
{
	return 0;
}

static int ambarella_spi_stop(struct ambarella_spi *ambspi)
{
	amba_readl(ambspi->regbase + SPI_ICR_OFFSET);
	amba_readl(ambspi->regbase + SPI_ISR_OFFSET);
	amba_writel(ambspi->regbase + SPI_SER_OFFSET, 0);
	amba_writel(ambspi->regbase + SPI_SSIENR_OFFSET, 0);

	return 0;
}

static void ambarella_spi_start_transfer(struct ambarella_spi *ambspi)
{
	void *wbuf;
	u32 widx, ridx, len, xfer_len;
	u16 i, tmp;
	u8 cs_id;

	wbuf	= (void *)ambspi->c_xfer->tx_buf;
	len	= ambspi->len;
	cs_id	= ambspi->c_dev->chip_select;
	widx	= ambspi->widx;
	ridx	= ambspi->ridx;

	/* Feed data into FIFO */
	switch (ambspi->rw_mode) {
	case SPI_WRITE_ONLY:
		xfer_len = min_t(int, len - widx, SPI_DATA_FIFO_SIZE_16);

		amba_writel(ambspi->regbase + SPI_SER_OFFSET, 0);
		if (ambspi->bpw <= 8) {
			for(i = 0; i < xfer_len; i++) {
				tmp = ((u8 *)wbuf)[widx++];
				amba_writel(ambspi->regbase + SPI_DR_OFFSET, tmp);
			}
		} else{
			for(i = 0; i < xfer_len; i++) {
				tmp = ((u16 *)wbuf)[widx++];
				amba_writel(ambspi->regbase + SPI_DR_OFFSET, tmp);
			}
		}
		amba_writel(ambspi->regbase + SPI_SER_OFFSET, 1 << cs_id);

		break;

	case SPI_WRITE_READ:
		xfer_len = min_t(int, len - widx, SPI_DATA_FIFO_SIZE_16);

		amba_writel(ambspi->regbase + SPI_SER_OFFSET, 0);
		if (ambspi->bpw <= 8) {
			for(i = 0; i < xfer_len; i++) {
				tmp = ((u8 *)wbuf)[widx++];
				amba_writel(ambspi->regbase + SPI_DR_OFFSET, tmp);
			}
		} else{
			for(i = 0; i < xfer_len; i++) {
				tmp = ((u16 *)wbuf)[widx++];
				amba_writel(ambspi->regbase + SPI_DR_OFFSET, tmp);
			}
		}
		amba_writel(ambspi->regbase + SPI_SER_OFFSET, 1 << cs_id);

		break;

	case SPI_READ_ONLY:
		xfer_len = min_t(int, len - ridx, SPI_DATA_FIFO_SIZE_16);

		amba_writel(ambspi->regbase + SPI_SER_OFFSET, 0);
		for(i = 0; i < xfer_len; i++)
			amba_writel(ambspi->regbase + SPI_DR_OFFSET, SPI_DUMMY_DATA);
		amba_writel(ambspi->regbase + SPI_SER_OFFSET, 1 << cs_id);

		break;

	default:
		break;
	}

	ambspi->widx = widx;
	enable_irq(ambspi->irq);

	return;
}

static void ambarella_spi_tasklet(unsigned long data)
{
	struct ambarella_spi *ambspi = (struct ambarella_spi *)data;
	void *rbuf;
	u32 widx, ridx, len, rxflr, xfer_len;
	u32 status, finish_transfer;
	u16 i, tmp;

	/* Wait until SPI idle */
	status = amba_readl(ambspi->regbase + SPI_SR_OFFSET);
	if (status & 0x1) {
		/* Transfer is still in progress */
		for (i = 0; i < MAX_QUERY_TIMES; i++) {
			status = amba_readl(ambspi->regbase + SPI_SR_OFFSET);
			if (!(status & 0x1))
				break;
		}
		if (status & 0x1) {
			tasklet_schedule(&ambspi->tasklet);
			return;
		}
	}

	rbuf	= (void *)ambspi->c_xfer->rx_buf;
	len	= ambspi->len;
	widx	= ambspi->widx;
	ridx	= ambspi->ridx;

	/* Fetch data from FIFO */
	switch (ambspi->rw_mode) {
	case SPI_READ_ONLY:
	case SPI_WRITE_READ:
		xfer_len	= len - ridx;
		rxflr		= amba_readl(ambspi->regbase + SPI_RXFLR_OFFSET);
		if (xfer_len > rxflr)
			xfer_len = rxflr;

		if (ambspi->bpw <= 8) {
			for(i = 0; i < xfer_len; i++) {
				tmp	= amba_readl(ambspi->regbase + SPI_DR_OFFSET);
				((u8 *)rbuf)[ridx++]	= tmp & 0xff;
			}
		} else {
			for(i = 0; i < xfer_len; i++){
				tmp	= amba_readl(ambspi->regbase + SPI_DR_OFFSET);
				((u16 *)rbuf)[ridx++]	= tmp;
			}
		}

		ambspi->ridx	= ridx;
		break;

	default:
		break;
	}

	/* Check whether the current transfer ends */
	finish_transfer = 0;
	switch (ambspi->rw_mode) {
	case SPI_WRITE_ONLY:
		if (widx == len) {
			finish_transfer = 1;
		}
		break;

	case SPI_READ_ONLY:
		if (ridx == len) {
			finish_transfer = 1;
		}
		break;

	case SPI_WRITE_READ:
		if (ridx == len && widx == len) {
			finish_transfer = 1;
		}
		break;

	default:
		break;
	}

	/* End transfer or continue filling FIFO */
	if (finish_transfer) {
		ambarella_spi_finish_transfer(ambspi);
		enable_irq(ambspi->irq);
	} else {
		ambarella_spi_start_transfer(ambspi);
	}
}

static void ambarella_spi_prepare_transfer(struct ambarella_spi *ambspi)
{
	struct spi_message *msg;
	struct spi_transfer *xfer;
	void *wbuf, *rbuf;
	u32 ctrlr0;

	msg = ambspi->c_msg;
	xfer = list_entry(msg->transfers.next, struct spi_transfer, transfer_list);
	ambspi->c_xfer	= xfer;
	list_del(msg->transfers.next);

	wbuf = (void *)xfer->tx_buf;
	rbuf = (void *)xfer->rx_buf;
	if (ambspi->bpw <= 8)
		ambspi->len = xfer->len;
	else
		ambspi->len = xfer->len >> 1;
	ambspi->widx = 0;
	ambspi->ridx = 0;
	if (wbuf && !rbuf)
		ambspi->rw_mode = SPI_WRITE_ONLY;
	if ( !wbuf && rbuf)
		ambspi->rw_mode = SPI_READ_ONLY;
	if (wbuf && rbuf)
		ambspi->rw_mode = SPI_WRITE_READ;

	ctrlr0	= amba_readl(ambspi->regbase + SPI_CTRLR0_OFFSET);
	ctrlr0	&= 0xfffff4ff;
	/* Always use write & read mode due to I1 changes */
	ctrlr0	|= (SPI_WRITE_READ << 8);
	if (ambspi->c_dev->mode & SPI_LOOP)
		ctrlr0 |= (0x1 << 11);

	amba_writel(ambspi->regbase + SPI_CTRLR0_OFFSET, ctrlr0);

	if (!ambspi->chip_select) {
		struct spi_device *spi = ambspi->c_dev;
		gpio_set_value(spi->cs_gpio, !!(spi->mode & SPI_CS_HIGH));
		ambspi->chip_select = 1;
	}

	disable_irq_nosync(ambspi->irq);
	amba_writel(ambspi->regbase + SPI_IMR_OFFSET, SPI_TXEIS_MASK);
	amba_writel(ambspi->regbase + SPI_SSIENR_OFFSET, 1);
	amba_writel(ambspi->regbase + SPI_SER_OFFSET, 0);
}

static void ambarella_spi_finish_transfer(struct ambarella_spi *ambspi)
{
	if (ambspi->c_xfer->cs_change) {
		struct spi_device *spi = ambspi->c_msg->spi;
		gpio_set_value(spi->cs_gpio, !(spi->mode & SPI_CS_HIGH));
		ambspi->chip_select = 0;
	}
	ambarella_spi_stop(ambspi);

	if (list_empty(&ambspi->c_msg->transfers)) {
		ambarella_spi_finish_message(ambspi);
	} else {
		ambarella_spi_prepare_transfer(ambspi);
		ambarella_spi_start_transfer(ambspi);
	}
}

static void ambarella_spi_finish_message(struct ambarella_spi *ambspi)
{
	struct spi_message		*msg;
	unsigned long			flags;
	u32				message_pending;

	if (ambspi->chip_select) {
		struct spi_device *spi = ambspi->c_msg->spi;
		gpio_set_value(spi->cs_gpio, !(spi->mode & SPI_CS_HIGH));
		ambspi->chip_select = 0;
	}

	msg = ambspi->c_msg;
	msg->actual_length = ambspi->c_xfer->len;
	msg->status = 0;

	/* Next Message */
	spin_lock_irqsave(&ambspi->lock, flags);
	list_del_init(&msg->queue);
	if (!list_empty(&ambspi->queue)) {
		message_pending	= 1;
	} else {
		message_pending	= 0;
		ambspi->idle = 1;
		ambspi->c_msg = NULL;
		ambspi->c_xfer = NULL;
	}
	spin_unlock_irqrestore(&ambspi->lock, flags);

	msg->complete(msg->context);
	if (message_pending) {
		ambarella_spi_handle_message(ambspi);
	}
}

static void ambarella_spi_handle_message(struct ambarella_spi *ambspi)
{
	ambarella_spi_prepare_message(ambspi);
	ambarella_spi_prepare_transfer(ambspi);
	ambarella_spi_start_transfer(ambspi);
}

static void ambarella_spi_prepare_message(struct ambarella_spi *ambspi)
{
	struct spi_message *msg;
	struct spi_device *spi;
	unsigned long flags;
	u32 ctrlr0, ssi_clk, sckdv;

	spin_lock_irqsave(&ambspi->lock, flags);
	msg = list_entry(ambspi->queue.next, struct spi_message, queue);
	spin_unlock_irqrestore(&ambspi->lock, flags);
	spi = msg->spi;

	ctrlr0 = amba_readl(ambspi->regbase + SPI_CTRLR0_OFFSET);

	if (spi->bits_per_word < 4)
		spi->bits_per_word = 4;
	if (spi->bits_per_word > 16)
		spi->bits_per_word = 16;
	ambspi->bpw = spi->bits_per_word;

	ctrlr0 &= 0xfffffff0;
	ctrlr0 |= (ambspi->bpw - 1);
	ctrlr0 &= (~((1 << 6) | (1 << 7)));
	ctrlr0 |= ((spi->mode & (SPI_CPHA | SPI_CPOL)) << 6);
	if (spi->mode & SPI_LOOP)
		ctrlr0 |= 0x00000800;

	amba_writel(ambspi->regbase + SPI_CTRLR0_OFFSET, ctrlr0);

	ssi_clk	= clk_get_rate(ambspi->clk);
	if(spi->max_speed_hz == 0 || spi->max_speed_hz > ssi_clk / 2)
		spi->max_speed_hz = ssi_clk / 2;


	sckdv = (u16)(((ssi_clk / spi->max_speed_hz) + 0x01) & 0xfffe);
	amba_writel(ambspi->regbase + SPI_BAUDR_OFFSET, sckdv);

	ambspi->chip_select = 0;
	ambspi->c_dev = spi;
	ambspi->c_msg = msg;

	gpio_set_value(spi->cs_gpio, !!(spi->mode & SPI_CS_HIGH));
}

static int ambarella_spi_main_entry(struct spi_device *spi, struct spi_message *msg)
{
	struct ambarella_spi		*ambspi;
	struct spi_transfer		*xfer;
	unsigned long			flags;
	u32				shut_down, bus_idle;

	ambspi		= spi_master_get_devdata(spi->master);
	spin_lock_irqsave(&ambspi->lock, flags);
	shut_down	= ambspi->shutdown;
	spin_unlock_irqrestore(&ambspi->lock, flags);
	if (shut_down) {
		return -ESHUTDOWN;
	}

	/* Validation */
	if (list_empty(&msg->transfers) || !spi->max_speed_hz) {
		return -EINVAL;
	}

	list_for_each_entry(xfer, &msg->transfers, transfer_list) {
		if (!xfer->tx_buf && !xfer->rx_buf) {
			return -EINVAL;
		}

		if (spi->bits_per_word > 8 && (xfer->len & 0x1)) {
			return -EINVAL;
		}
	}

	/* Queue Message */
	msg->status		= -EINPROGRESS;
	msg->actual_length	= 0;
	spin_lock_irqsave(&ambspi->lock, flags);
	list_add_tail(&msg->queue, &ambspi->queue);
	if (ambspi->idle) {
		ambspi->idle	= 0;
		bus_idle	= 1;
	} else {
		bus_idle	= 0;
	}
	spin_unlock_irqrestore(&ambspi->lock, flags);

	/* Handle message right away if bus is idle */
	if (bus_idle) {
		ambarella_spi_handle_message(ambspi);
	}

	return 0;
}

static void ambarella_spi_cleanup(struct spi_device *spi)
{
	return;
}

static int ambarella_spi_inithw(struct ambarella_spi *ambspi)
{
	u16 				sckdv;
	u32 				ctrlr0, ssi_freq;

	/* Set PLL */
	clk_set_rate(ambspi->clk, ambspi->clk_freq);

	/* Disable SPI */
	ambarella_spi_stop(ambspi);

	/* Initial Register Settings */
	ctrlr0 = ( ( SPI_CFS << 12) | (SPI_WRITE_ONLY << 8) | (SPI_SCPOL << 7) |
		(SPI_SCPH << 6)	| (SPI_FRF << 4) | (SPI_DFS)
	      );
	amba_writel(ambspi->regbase + SPI_CTRLR0_OFFSET, ctrlr0);

	ssi_freq = clk_get_rate(ambspi->clk);
	sckdv =	(u16)(((ssi_freq / SPI_BAUD_RATE) + 0x01) & 0xfffe);
	amba_writel(ambspi->regbase + SPI_BAUDR_OFFSET, sckdv);

	amba_writel(ambspi->regbase + SPI_TXFTLR_OFFSET, 0);
	amba_writel(ambspi->regbase + SPI_RXFTLR_OFFSET, 1);

	return 0;
}

static irqreturn_t ambarella_spi_isr(int irq, void *dev_data)
{
	struct ambarella_spi		*ambspi	= dev_data;

	if (amba_readl(ambspi->regbase + SPI_ISR_OFFSET)) {
		disable_irq_nosync(ambspi->irq);
		amba_writel(ambspi->regbase + SPI_ISR_OFFSET, 0);

		ambarella_spi_tasklet((unsigned long)ambspi);
	}

	return IRQ_HANDLED;
}

static int of_dev_node_match(struct device *dev, void *data)
{
	return dev->of_node == data;
}

static struct spi_device *ambarella_spi_of_find_device(struct device_node *np)
{
	struct device *dev;

	dev = bus_find_device(&spi_bus_type, NULL, np,
					 of_dev_node_match);
	if (!dev)
		return NULL;

	return to_spi_device(dev);
}

static int ambarella_spi_of_parse(struct platform_device *pdev,
			struct spi_master *master)
{
	struct device_node *np = pdev->dev.of_node;
	struct ambarella_spi *ambspi = spi_master_get_devdata(master);
	const char *clk_name;
	int rval;

	rval = of_property_read_string(np, "amb,clk-name", &clk_name);
	if (rval < 0) {
		dev_err(&pdev->dev, "Get clk-name failed! %d\n", rval);
		return rval;
	}

	ambspi->clk = clk_get(NULL, clk_name);
	if (IS_ERR(ambspi->clk)) {
		dev_err(&pdev->dev, "Get PLL failed!\n");
		return PTR_ERR(ambspi->clk);
	}

	rval = of_property_read_u32(np, "amb,clk-freq", &ambspi->clk_freq);
	if (rval < 0) {
		dev_err(&pdev->dev, "invalid clk-freq! %d\n", rval);
		return rval;
	}

	return 0;
}

static int ambarella_spi_probe(struct platform_device *pdev)
{
	struct device_node *nc;
	struct ambarella_spi *ambspi;
	struct spi_master *master;
	struct resource *res;
	void __iomem *reg;
	int irq, rval;

	/* Get Base Address */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "Get mem resource failed!\n");
		return -ENXIO;
	}

	reg = devm_ioremap(&pdev->dev, res->start, resource_size(res));
	if (!reg) {
		dev_err(&pdev->dev, "devm_ioremap() failed\n");
		return -ENOMEM;
	}

	/* Get IRQ NO. */
	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(&pdev->dev, "Get irq failed!\n");
		return -ENXIO;
	}

	/* Alocate Master */
	master = spi_alloc_master(&pdev->dev, sizeof *ambspi);
	if (!master) {
		dev_err(&pdev->dev, "no memory!\n");
		return -ENOMEM;
	}

	ambspi = spi_master_get_devdata(master);
	platform_set_drvdata(pdev, master);

	rval = ambarella_spi_of_parse(pdev, master);
	if (rval < 0)
		goto exit_err0;

	/* Initalize Device Data */
	master->dev.of_node = pdev->dev.of_node;
	master->mode_bits = SPI_CPHA | SPI_CPOL | SPI_LSB_FIRST | SPI_LOOP;
	master->setup = ambarella_spi_setup;
	master->transfer = ambarella_spi_main_entry;
	master->cleanup = ambarella_spi_cleanup;

	ambspi->regbase = reg;
	ambspi->irq = irq;
	tasklet_init(&ambspi->tasklet, ambarella_spi_tasklet, (unsigned long)ambspi);
	INIT_LIST_HEAD(&ambspi->queue);
	ambspi->idle = 1;
	ambspi->c_dev = NULL;
	ambspi->c_msg = NULL;
	ambspi->c_xfer = NULL;
	ambspi->shutdown = 0;
	spin_lock_init(&ambspi->lock);
	ambspi->bpw = 16;

	/* Inittialize Hardware*/
	ambarella_spi_inithw(ambspi);

	/* Register Master */
	rval = spi_register_master(master);
	if (rval) {
		dev_err(&pdev->dev, "failed to register: %d\n", rval);
		goto exit_err0;
	}

	if (master->bus_num < 0 || master->bus_num >= SPI_MASTER_INSTANCES) {
		dev_err(&pdev->dev, "invalid bus_num: %d\n", master->bus_num);
		goto exit_err1;
	}

	/* Request IRQ */
	rval = devm_request_irq(&pdev->dev, irq, ambarella_spi_isr,
				IRQF_TRIGGER_HIGH, dev_name(&pdev->dev), ambspi);
	if (rval)
		goto exit_err1;

	dev_info(&pdev->dev, "ambarella SPI Controller %d created \n", master->bus_num);

	/* assign private devices */
	for_each_available_child_of_node(master->dev.of_node, nc) {
		struct spi_device *spi;
		struct ambarella_spi_private *priv;

		spi = ambarella_spi_of_find_device(nc);
		if (spi == NULL)
			continue;

		if (gpio_is_valid(spi->cs_gpio)) {
			rval = devm_gpio_request(&pdev->dev,
					spi->cs_gpio, dev_name(&spi->dev));
			if (rval < 0) {
				dev_err(&pdev->dev, "can't get CS: %d\n", rval);
				goto exit_err1;
			}
			/* deactive */
			gpio_direction_output(spi->cs_gpio,
					spi->mode & SPI_CS_HIGH ? 0 : 1);
		}

		priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
		if (!priv) {
			rval = -ENOMEM;
			goto exit_err1;
		}

		priv->spi = spi;
		mutex_init(&priv->mtx);
		spin_lock_init(&priv->lock);
		list_add_tail(&priv->list, &ambarella_ps_list);
	}

	return 0;

exit_err1:
	spi_unregister_master(master);
exit_err0:
	spi_master_put(master);

	return rval;
}

static int ambarella_spi_remove(struct platform_device *pdev)
{

	struct spi_master		*master = platform_get_drvdata(pdev);
	struct ambarella_spi		*ambspi = spi_master_get_devdata(master);
	struct spi_message		*msg;
	unsigned long			flags;

	spin_lock_irqsave(&ambspi->lock, flags);
	ambspi->shutdown	= 1;
	spin_unlock_irqrestore(&ambspi->lock, flags);
	tasklet_kill(&ambspi->tasklet);
	ambarella_spi_stop(ambspi);

	spin_lock_irqsave(&ambspi->lock, flags);
	list_for_each_entry(msg, &ambspi->queue, queue) {
		msg->status	= -ESHUTDOWN;
		msg->complete(msg->context);
	}
	spin_unlock_irqrestore(&ambspi->lock, flags);

	spi_unregister_master(master);

	return 0;
}

#ifdef CONFIG_PM
static int ambarella_spi_suspend_noirq(struct device *dev)
{
	int				rval = 0;
	struct spi_master		*master;
	struct ambarella_spi		*ambspi;
	struct platform_device		*pdev;

	pdev = to_platform_device(dev);
	master = platform_get_drvdata(pdev);
	ambspi = spi_master_get_devdata(master);

	disable_irq(ambspi->irq);
	ambarella_spi_stop(ambspi);

	dev_dbg(&pdev->dev, "%s\n", __func__);

	return rval;
}

static int ambarella_spi_resume_noirq(struct device *dev)
{
	int				rval = 0;
	struct spi_master		*master;
	struct ambarella_spi		*ambspi;
	struct platform_device		*pdev;

	pdev = to_platform_device(dev);
	master = platform_get_drvdata(pdev);
	ambspi = spi_master_get_devdata(master);

	ambarella_spi_inithw(ambspi);
	enable_irq(ambspi->irq);

	dev_dbg(&pdev->dev, "%s\n", __func__);

	return rval;
}

static const struct dev_pm_ops ambarella_spi_dev_pm_ops = {
	.suspend_noirq = ambarella_spi_suspend_noirq,
	.resume_noirq = ambarella_spi_resume_noirq,
};

#define pambarella_spi_dev_pm_ops	&ambarella_spi_dev_pm_ops
#else
#define pambarella_spi_dev_pm_ops	NULL
#endif

static const struct of_device_id ambarella_spi_dt_ids[] = {
	{.compatible = "ambarella,spi", },
	{},
};
MODULE_DEVICE_TABLE(of, ambarella_spi_dt_ids);

static struct platform_driver ambarella_spi_driver = {
	.probe		= ambarella_spi_probe,
	.remove		= ambarella_spi_remove,
	.driver		= {
		.name	= "ambarella-spi",
		.owner	= THIS_MODULE,
		.pm	= pambarella_spi_dev_pm_ops,
		.of_match_table = ambarella_spi_dt_ids,
	},
};

static int __init ambarella_spi_init(void)
{
	return platform_driver_register(&ambarella_spi_driver);
}

static void __exit ambarella_spi_exit(void)
{
	platform_driver_unregister(&ambarella_spi_driver);
}

subsys_initcall(ambarella_spi_init);
module_exit(ambarella_spi_exit);

MODULE_DESCRIPTION("Ambarella Media Processor SPI Bus Controller");
MODULE_AUTHOR("Louis Sun, <louis.sun@ambarella.com>");
MODULE_LICENSE("GPL");


/*=================Utilities for Non-GPL Use==================================*/
static void ambarella_spi_complete(void *arg)
{
	complete(arg);
}

int ambarella_spi_write(amba_spi_cfg_t *spi_cfg, amba_spi_write_t *spi_write)
{
	struct ambarella_spi_private *priv;
	struct spi_device *spi;
	struct spi_master *master;
	struct spi_message msg;
	struct spi_transfer xfer;
	u8 cs_id, bus_id;
	int found = 0, rval;

	DECLARE_COMPLETION_ONSTACK(done);

	/* Validate Input Args */
	if (!spi_cfg || !spi_write || !spi_write->buffer || !spi_write->n_size)
		return -EINVAL;

	bus_id = spi_write->bus_id;
	cs_id = spi_write->cs_id;

	list_for_each_entry(priv, &ambarella_ps_list, list) {
		spi = priv->spi;
		master = priv->spi->master;
		if (master->bus_num == bus_id && spi->chip_select == cs_id) {
			found = 1;
			break;
		}
	}

	if (!found) {
		pr_err("No spi vin in FDT: bus = %d, cs = %d!\n", bus_id, cs_id);
		return -ENODEV;
	}

	/* Transfer */
	memset(&xfer, 0, sizeof(struct spi_transfer));
	xfer.tx_buf	= spi_write->buffer;
	xfer.len	= spi_write->n_size;
	xfer.cs_change	= spi_cfg->cs_change;

	/* Message */
	memset(&msg, 0, sizeof(struct spi_message));
	INIT_LIST_HEAD(&msg.transfers);
	list_add_tail(&xfer.transfer_list, &msg.transfers);
	msg.complete	= ambarella_spi_complete;
	msg.context	= &done;
	msg.spi		= spi;

	mutex_lock(&priv->mtx);

	/* Config */
	spi->mode		= spi_cfg->spi_mode;
	spi->mode		&= ~SPI_LOOP;
	spi->bits_per_word	= spi_cfg->cfs_dfs;
	spi->max_speed_hz	= spi_cfg->baud_rate;

	/* Wait */
	spin_lock_irq(&priv->lock);
	rval = spi->master->transfer(spi, &msg);
	spin_unlock_irq(&priv->lock);
	if (!rval)
		wait_for_completion(&done);

	mutex_unlock(&priv->mtx);

	return rval;
}
EXPORT_SYMBOL(ambarella_spi_write);

int ambarella_spi_read(amba_spi_cfg_t *spi_cfg, amba_spi_read_t *spi_read)
{
	struct ambarella_spi_private *priv;
	struct spi_device *spi;
	struct spi_master *master;
	struct spi_message msg;
	struct spi_transfer xfer;
	u8 cs_id, bus_id;
	int found = 0, rval;

	DECLARE_COMPLETION_ONSTACK(done);

	/* Validate Input Args */
	if (!spi_cfg || !spi_read || !spi_read->buffer || !spi_read->n_size)
		return -EINVAL;

	bus_id	= spi_read->bus_id;
	cs_id	= spi_read->cs_id;

	list_for_each_entry(priv, &ambarella_ps_list, list) {
		spi = priv->spi;
		master = priv->spi->master;
		if (master->bus_num == bus_id && spi->chip_select == cs_id) {
			found = 1;
			break;
		}
	}

	if (!found){
		pr_err("No spi vin in FDT: bus = %d, cs = %d!\n", bus_id, cs_id);
		return -ENODEV;
	}

	/* Transfer */
	memset(&xfer, 0, sizeof(struct spi_transfer));
	xfer.rx_buf	= spi_read->buffer;
	xfer.len	= spi_read->n_size;
	xfer.cs_change	= spi_cfg->cs_change;

	/* Message */
	memset(&msg, 0, sizeof(struct spi_message));
	INIT_LIST_HEAD(&msg.transfers);
	list_add_tail(&xfer.transfer_list, &msg.transfers);
	msg.complete	= ambarella_spi_complete;
	msg.context	= &done;
	msg.spi		= spi;

	mutex_lock(&priv->mtx);

	/* Config */
	spi->mode		= spi_cfg->spi_mode;
	spi->mode		&= ~SPI_LOOP;
	spi->bits_per_word	= spi_cfg->cfs_dfs;
	spi->max_speed_hz	= spi_cfg->baud_rate;

	/* Wait */
	spin_lock_irq(&priv->lock);
	rval = spi->master->transfer(spi, &msg);
	spin_unlock_irq(&priv->lock);
	if (!rval)
		wait_for_completion(&done);

	mutex_unlock(&priv->mtx);

	return rval;
}
EXPORT_SYMBOL(ambarella_spi_read);

int ambarella_spi_write_then_read(amba_spi_cfg_t *spi_cfg,
	amba_spi_write_then_read_t *spi_write_then_read)
{
	struct ambarella_spi_private *priv;
	struct spi_device *spi;
	struct spi_master *master;
	struct spi_message msg;
	struct spi_transfer xfer;
	u8 cs_id, bus_id, *buf;
	u16 size;
	int found = 0, rval;

	DECLARE_COMPLETION_ONSTACK(done);

	/* Validate Input Args */
	if (!spi_cfg || !spi_write_then_read
		|| !spi_write_then_read->w_buffer
		|| !spi_write_then_read->w_size
		|| !spi_write_then_read->r_buffer
		|| !spi_write_then_read->r_size)
		return -EINVAL;

	bus_id	= spi_write_then_read->bus_id;
	cs_id	= spi_write_then_read->cs_id;

	list_for_each_entry(priv, &ambarella_ps_list, list) {
		spi = priv->spi;
		master = priv->spi->master;
		if (master->bus_num == bus_id && spi->chip_select == cs_id) {
			found = 1;
			break;
		}
	}

	if (!found){
		pr_err("No spi vin in FDT: bus = %d, cs = %d!\n", bus_id, cs_id);
		return -ENODEV;
	}


	/* Prepare Buffer */
	size = spi_write_then_read->w_size + spi_write_then_read->r_size;
	buf = (u8 *)kmalloc(size, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	memcpy(buf, spi_write_then_read->w_buffer, spi_write_then_read->w_size);
	memset(buf + spi_write_then_read->w_size, SPI_DUMMY_DATA,
		spi_write_then_read->r_size);

	/* Transfer */
	memset(&xfer, 0, sizeof(struct spi_transfer));
	xfer.tx_buf	= buf;
	xfer.rx_buf	= buf;
	xfer.len	= size;
	xfer.cs_change	= spi_cfg->cs_change;

	/* Message */
	memset(&msg, 0, sizeof(struct spi_message));
	INIT_LIST_HEAD(&msg.transfers);
	list_add_tail(&xfer.transfer_list, &msg.transfers);
	msg.complete	= ambarella_spi_complete;
	msg.context	= &done;
	msg.spi		= spi;

	mutex_lock(&priv->mtx);

	/* Config */
	spi->mode		= spi_cfg->spi_mode;
	spi->mode		&= ~SPI_LOOP;
	spi->bits_per_word	= spi_cfg->cfs_dfs;
	spi->max_speed_hz	= spi_cfg->baud_rate;

	/* Wait */
	spin_lock_irq(&priv->lock);
	rval = spi->master->transfer(spi, &msg);
	spin_unlock_irq(&priv->lock);
	if (!rval)
		wait_for_completion(&done);

	mutex_unlock(&priv->mtx);

	/* Free Buffer */
	memcpy(spi_write_then_read->r_buffer, buf + spi_write_then_read->w_size,
		spi_write_then_read->r_size);
	kfree(buf);

	return rval;
}
EXPORT_SYMBOL(ambarella_spi_write_then_read);

int ambarella_spi_write_and_read(amba_spi_cfg_t *spi_cfg,
	amba_spi_write_and_read_t *spi_write_and_read)
{
	struct ambarella_spi_private *priv;
	struct spi_device *spi;
	struct spi_master *master;
	struct spi_message msg;
	struct spi_transfer xfer;
	u8 cs_id, bus_id;
	int found = 0, rval;

	DECLARE_COMPLETION_ONSTACK(done);

	/* Validate Input Args */
	if (!spi_cfg || !spi_write_and_read
		|| !spi_write_and_read->w_buffer
		|| !spi_write_and_read->r_buffer
		|| !spi_write_and_read->n_size)
		return -EINVAL;


	bus_id	= spi_write_and_read->bus_id;
	cs_id	= spi_write_and_read->cs_id;

	list_for_each_entry(priv, &ambarella_ps_list, list) {
		spi = priv->spi;
		master = priv->spi->master;
		if (master->bus_num == bus_id && spi->chip_select == cs_id) {
			found = 1;
			break;
		}
	}

	if (!found){
		pr_err("No spi vin in FDT: bus = %d, cs = %d!\n", bus_id, cs_id);
		return -ENODEV;
	}

	/* Transfer */
	memset(&xfer, 0, sizeof(struct spi_transfer));
	xfer.tx_buf	= spi_write_and_read->w_buffer;
	xfer.rx_buf	= spi_write_and_read->r_buffer;
	xfer.len	= spi_write_and_read->n_size;
	xfer.cs_change	= spi_cfg->cs_change;

	/* Message */
	memset(&msg, 0, sizeof(struct spi_message));
	INIT_LIST_HEAD(&msg.transfers);
	list_add_tail(&xfer.transfer_list, &msg.transfers);
	msg.complete	= ambarella_spi_complete;
	msg.context	= &done;
	msg.spi		= spi;

	mutex_lock(&priv->mtx);

	/* Config */
	spi->mode		= spi_cfg->spi_mode;
	spi->mode		&= ~SPI_LOOP;
	spi->bits_per_word	= spi_cfg->cfs_dfs;
	spi->max_speed_hz	= spi_cfg->baud_rate;

	/* Wait */
	spin_lock_irq(&priv->lock);
	rval = spi->master->transfer(spi, &msg);
	spin_unlock_irq(&priv->lock);
	if (!rval)
		wait_for_completion(&done);

	mutex_unlock(&priv->mtx);

	return rval;
}
EXPORT_SYMBOL(ambarella_spi_write_and_read);

