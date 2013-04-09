/*
 * spi-jz4740-gpio.c - Bit-banging SPI host for the Jz4740
 *
 * Written 2011, 2013 by Werner Almesberger
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2
 * as published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi_gpio.h>
#include <asm/mach-jz4740/base.h>

#define	PxPIN	(prv->port_base)
#define	PxDATS	(prv->port_base+0x14)
#define	PxDATC	(prv->port_base+0x18)
#define	PxFUNC	(prv->port_base+0x48)


struct spi_jz4740_gpio {
	const struct spi_gpio_platform_data *pdata;
	struct device		*dev;
	void __iomem		*port_base;
	struct resource		*ioarea;
	uint32_t		mosi, miso, sck;
	unsigned long		port_addr;
};


/* ----- SPI transfers ----------------------------------------------------- */


static void rx_only(const struct spi_jz4740_gpio *prv, uint8_t *buf, int len)
{
	uint32_t miso = prv->miso;
	uint32_t sck = prv->sck;
	uint8_t v;

	while (len--) {
		writel(sck, PxDATS);
		v = readl(PxPIN) & miso ? 0x80 : 0;
		writel(sck, PxDATC);

		#define	DO_BIT(m)			\
			writel(sck, PxDATS);		\
			if (readl(PxPIN) & miso)	\
				v |= (m);		\
			writel(sck, PxDATC)

		DO_BIT(0x40);
		DO_BIT(0x20);
		DO_BIT(0x10);
		DO_BIT(0x08);
		DO_BIT(0x04);
		DO_BIT(0x02);
		DO_BIT(0x01);

		#undef DO_BIT

		*buf++ = v;
	}
}


static void tx_only(const struct spi_jz4740_gpio *prv,
    const uint8_t *buf, int len)
{
	uint32_t mosi = prv->mosi;
	uint32_t sck = prv->sck;
	uint8_t tv;

	while (len--) {
		tv = *buf++;

		if (tv & 0x80) {
			writel(mosi, PxDATS);
			goto b6_1;
		} else {
			writel(mosi, PxDATC);
			goto b6_0;
		}

		#define	DO_BIT(m, this, next)				\
			this##_1:					\
				writel(sck, PxDATS);			\
				if (tv & (m)) {				\
					writel(sck, PxDATC);		\
					goto next##_1;			\
				} else {				\
					writel(mosi | sck, PxDATC);	\
					goto next##_0;			\
				}					\
			this##_0:					\
				writel(sck, PxDATS);			\
				writel(sck, PxDATC);			\
				if (tv & (m)) {				\
					writel(mosi, PxDATS);		\
					goto next##_1;			\
				} else {				\
					goto next##_0;			\
				}

		DO_BIT(0x40, b6, b5);
		DO_BIT(0x20, b5, b4);
		DO_BIT(0x10, b4, b3);
		DO_BIT(0x08, b3, b2);
		DO_BIT(0x04, b2, b1);
		DO_BIT(0x02, b1, b0);
		DO_BIT(0x01, b0, done);

		#undef DO_BIT

done_1:
done_0:
		writel(sck, PxDATS);
		writel(sck, PxDATC);

		/*
		 * Extra delay to meet AT86RF230/1 SPI timing.
		 * (Parameter t5 = 250 ns in the data sheet.)
		 */
		writel(sck, PxDATC);
	}
}


static void bidir(const struct spi_jz4740_gpio *prv,
    const uint8_t *tx, uint8_t *rx, int len)
{
	uint32_t mosi = prv->mosi;
	uint32_t miso = prv->miso;
	uint32_t sck = prv->sck;
	uint8_t mask, tv, rv = 0;

	while (len--) {
		tv = *tx++;
		for (mask = 0x80; mask; mask >>= 1) {
			if (tv & mask)
				writel(mosi, PxDATS);
			else
				writel(mosi, PxDATC);
			writel(sck, PxDATS);
			if (readl(PxPIN) & miso)
				rv |= mask;
			writel(sck, PxDATC);
		}
		*rx++ = rv;
	}
}


static inline unsigned get_nsel(struct spi_device *spi)
{
	return (unsigned int) spi->controller_data;
}


static int spi_jz4740_gpio_transfer(struct spi_device *spi,
    struct spi_message *msg)
{
	struct spi_jz4740_gpio *prv = spi_master_get_devdata(spi->master);
	struct spi_transfer *xfer;
	uint32_t nsel = get_nsel(spi);
	const uint8_t *tx;
	uint8_t *rx;

	if (unlikely(list_empty(&msg->transfers))) {
		dev_err(&spi->dev, "transfer is empty\n");
		return -EINVAL;
	}

	msg->actual_length = 0;

	gpio_set_value(nsel, 0);
	list_for_each_entry(xfer, &msg->transfers, transfer_list) {
		tx = xfer->tx_buf;
		rx = xfer->rx_buf;
		msg->actual_length += xfer->len;

		if (!tx)
			rx_only(prv, rx, xfer->len);
		else if (!rx)
			tx_only(prv, tx, xfer->len);
		else
			bidir(prv, tx, rx, xfer->len);
	}
	gpio_set_value(nsel, 1);

	msg->status = 0;
	msg->complete(msg->context);

	return 0;
}


/* ----- Device-specific setup (nSEL) -------------------------------------- */


static int get_gpio(struct spi_jz4740_gpio *prv,
    unsigned pin, const char *label, int value)
{
	int err;
	unsigned long port;

	err = gpio_request(pin, label);
	if (err)
		return err;

	if (value >= 0)
		err = gpio_direction_output(pin, value);
	else
		err = gpio_direction_input(pin);
	if (err)
		goto fail;

	err = jz_gpio_set_function(pin, 0);
	if (err)
		goto fail;

	if (!prv)
		return 0;

	port = JZ4740_GPIO_BASE_ADDR + 0x100 * (pin >> 5);
	if (prv->port_addr && prv->port_addr != port) {
		err = -EINVAL;
		goto fail;
	}
	prv->port_addr = port;

	return 0;

fail:
	gpio_free(pin);
        return err;
}

static int spi_jz4740_gpio_setup(struct spi_device *spi)
{
	unsigned nsel = get_nsel(spi);

	dev_dbg(&spi->dev, "%s\n", __func__);
	return get_gpio(NULL, nsel, dev_name(&spi->dev), 0);
}

static void spi_jz4740_gpio_cleanup(struct spi_device *spi)
{
	unsigned nsel = get_nsel(spi);

	dev_dbg(&spi->dev, "%s\n", __func__);
	gpio_free(nsel);
}


/* ----- Non-nSEL GPIO allocation and configuration ------------------------ */


static void free_gpios(struct spi_jz4740_gpio *prv)
{
	const struct spi_gpio_platform_data *pdata = prv->pdata;

	if (prv->mosi)
		gpio_free(pdata->mosi);
	if (prv->miso)
		gpio_free(pdata->miso);
	if (prv->sck)
		gpio_free(pdata->sck);
}


static int setup_gpios(struct spi_jz4740_gpio *prv, const char *label,
    uint16_t *flags)
{
	const struct spi_gpio_platform_data *pdata = prv->pdata;
	int err;

	if (pdata->mosi == -1) {
		*flags |= SPI_MASTER_NO_TX;
	} else {
		err = get_gpio(prv, pdata->mosi, label, 0);
		if (err)
			return err;
		prv->mosi = 1 << (pdata->mosi & 31);
	}

	if (pdata->miso == -1) {
		*flags |= SPI_MASTER_NO_RX;
	} else {
		err = get_gpio(prv, pdata->miso, label, -1);
		if (err)
			goto fail;
		prv->miso = 1 << (pdata->miso & 31);
	}

	err = get_gpio(prv, pdata->sck, label, 0);
	if (err)
		goto fail;
	prv->sck = 1 << (pdata->sck & 31);

	return 0;

fail:
	free_gpios(prv);
	return err;
}

/* ----- SPI master creation/removal --------------------------------------- */


static int spi_jz4740_gpio_probe(struct platform_device *pdev)
{
	const struct spi_gpio_platform_data *pdata = pdev->dev.platform_data;
	struct spi_master *master;
	struct spi_jz4740_gpio *prv;
	struct resource *regs;
	int err = -ENXIO;

	if (!pdata)
		return -ENODEV;

	master = spi_alloc_master(&pdev->dev, sizeof(*prv));
	if (!master)
		return -ENOMEM;

	prv = spi_master_get_devdata(master);
	prv->dev = &pdev->dev;
	prv->pdata = pdata;
	platform_set_drvdata(pdev, spi_master_get(master));

	err = setup_gpios(prv, dev_name(prv->dev), &master->flags);
	if (err)
		goto out_master;

	master->mode_bits	= SPI_CPOL | SPI_CPHA | SPI_CS_HIGH;
	master->bus_num		= pdev->id;
	master->num_chipselect	= pdata->num_chipselect;
	master->setup		= spi_jz4740_gpio_setup;
	master->cleanup		= spi_jz4740_gpio_cleanup;
	master->transfer	= spi_jz4740_gpio_transfer;

	dev_dbg(prv->dev, "%s\n", __func__);

	regs = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!regs) {
		dev_err(prv->dev, "no IORESOURCE_MEM\n");
		err = -ENOENT;
		goto out_master;
	}
	prv->ioarea = request_mem_region(regs->start, resource_size(regs),
					 pdev->name);
	if (!prv->ioarea) {
		dev_err(prv->dev, "can't request ioarea\n");
		goto out_master;
	}

	prv->port_base = ioremap(regs->start, resource_size(regs));
	if (!prv->port_base) {
		dev_err(prv->dev, "can't ioremap\n");
		goto out_ioarea;
	}

	err = spi_register_master(master);
	if (err) {
		dev_err(prv->dev, "can't register master\n");
		goto out_regs;
	}

	return 0;

out_regs:
	iounmap(prv->port_base);

out_ioarea:
	release_resource(prv->ioarea);
	kfree(prv->ioarea);

out_master:
	platform_set_drvdata(pdev, NULL);
	spi_master_put(master);

	return err;
}

static int spi_jz4740_gpio_remove(struct platform_device *pdev)
{
	struct spi_master *master = platform_get_drvdata(pdev);
	struct spi_jz4740_gpio *prv = spi_master_get_devdata(master);

// restore GPIOs

	spi_unregister_master(master);

	iounmap(prv->port_base);

	release_resource(prv->ioarea);
	kfree(prv->ioarea);

	platform_set_drvdata(pdev, NULL);
	spi_master_put(master);

	return 0;
}

static struct platform_driver jz4740_spi_gpio_driver = {
	.driver = {
		.name	= "j4740-gpio-spi",
		.owner	= THIS_MODULE,
	},
	.probe		= spi_jz4740_gpio_probe,
	.remove		= spi_jz4740_gpio_remove,
};
module_platform_driver(jz4740_spi_gpio_driver);

#if 0
static struct resource atben_resources[] = {
	{
		.start  = JZ4740_GPIO_BASE_ADDR+0x300,
		.end    = JZ4740_GPIO_BASE_ADDR+0x3ff,
		.flags  = IORESOURCE_MEM,
	},
	{
		/* set start and end later */
		.flags  = IORESOURCE_IRQ,
	},
};
#endif

MODULE_ALIAS("platform:jz4740-spi-gpio");
MODULE_DESCRIPTION("Jz4740 GPIO SPI Driver");
MODULE_AUTHOR("Werner Almesberger <werner@almesberger.net>");
MODULE_LICENSE("GPL");
