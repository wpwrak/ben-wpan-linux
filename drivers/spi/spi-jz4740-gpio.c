/*
 * spi-jz4740-gpio.c - Bit-banging SPI host for the Jz4740
 *
 * Written 2011, 2013 by Werner Almesberger
 * Based on spi-gpio.c, Copyright (C) 2006,2008 David Brownell
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2
 * as published by the Free Software Foundation.
 */

/*
 * This is a drop-in replacement for spi-gpio.c optimized for Jz4740-based
 * systems. It is up to about six times faster than its generic counterpart.
 *
 * There are three restrictions and usage differences:
 *
 * 1) No other configurations than SPI mode 0 and CS active-low are
 *    supported.
 *
 * 2) MOSI, MISO, and SCK must be on the same port. Driver probing fails
 *    if they are not.
 *
 * 3) struct platform_device.name must be "spi_jz4740_gpio" instead of
 *    "spi_gpio".
 */


#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi_gpio.h>
#include <asm/mach-jz4740/base.h>


#define	DRIVER_NAME	"spi_jz4740_gpio"

struct spi_jz4740_gpio {
	struct device		*dev;
	void __iomem		*port_base;
	int			mosi, miso, sck;
	uint32_t		mosi_mask, miso_mask, sck_mask;
	unsigned long		port_addr;
};

#define	PxPIN	(prv->port_base)
#define	PxDATS	(prv->port_base+0x14)
#define	PxDATC	(prv->port_base+0x18)

#define	PIN_TO_PORT(pin)	((pin) >> 5)
#define	PIN_TO_MASK(pin)	(1 << ((pin) & 31))


/* ----- SPI transfers ----------------------------------------------------- */


static void rx_only(const struct spi_jz4740_gpio *prv, uint8_t *buf, int len)
{
	uint32_t miso = prv->miso_mask;
	uint32_t sck = prv->sck_mask;
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
	uint32_t mosi = prv->mosi_mask;
	uint32_t sck = prv->sck_mask;
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
	}
}


static void bidir(const struct spi_jz4740_gpio *prv,
		const uint8_t *tx, uint8_t *rx, int len)
{
	uint32_t mosi = prv->mosi_mask;
	uint32_t miso = prv->miso_mask;
	uint32_t sck = prv->sck_mask;
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
	return (unsigned) spi->controller_data;
}


static int spi_jz4740_gpio_transfer_one(struct spi_master *master,
		struct spi_message *msg)
{
	struct spi_jz4740_gpio *prv = spi_master_get_devdata(master);
	struct spi_device *spi = msg->spi;
	uint32_t nsel = get_nsel(spi);
	struct spi_transfer *xfer;
	const uint8_t *tx;
	uint8_t *rx;

	if (unlikely(list_empty(&msg->transfers))) {
		dev_err(&spi->dev, "transfer is empty\n");
		msg->status = -EINVAL;
		goto out;
	}

	msg->actual_length = 0;
	msg->status = 0;

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

out:
	spi_finalize_current_message(master);

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

	err = jz_gpio_set_function(pin, JZ_GPIO_FUNC_NONE);
	if (err)
		goto fail;

	if (!prv)
		return 0;

	port = JZ4740_GPIO_BASE_ADDR + 0x100 * PIN_TO_PORT(pin);
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
	if (prv->mosi_mask)
		gpio_free(prv->mosi);
	if (prv->miso_mask)
		gpio_free(prv->miso);
	if (prv->sck_mask)
		gpio_free(prv->sck);
}


static int setup_gpios(struct spi_jz4740_gpio *prv, const char *label,
		uint16_t *flags)
{
	int err;

	if (prv->mosi == -1) {
		*flags |= SPI_MASTER_NO_TX;
	} else {
		err = get_gpio(prv, prv->mosi, label, 0);
		if (err)
			return err;
		prv->mosi_mask = PIN_TO_MASK(prv->mosi);
	}

	if (prv->miso == -1) {
		*flags |= SPI_MASTER_NO_RX;
	} else {
		err = get_gpio(prv, prv->miso, label, -1);
		if (err)
			goto fail;
		prv->miso_mask = PIN_TO_MASK(prv->miso);
	}

	err = get_gpio(prv, prv->sck, label, 0);
	if (err)
		goto fail;
	prv->sck_mask = PIN_TO_MASK(prv->sck);

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
	int err;

	dev_dbg(prv->dev, "%s\n", __func__);
	if (!pdata)
		return -ENODEV;

	master = spi_alloc_master(&pdev->dev, sizeof(*prv));
	if (!master)
		return -ENOMEM;

	prv = spi_master_get_devdata(master);
	prv->dev = &pdev->dev;
	platform_set_drvdata(pdev, spi_master_get(master));

	prv->mosi = pdata->mosi;
	prv->miso = pdata->miso;
	prv->sck = pdata->sck;

	err = setup_gpios(prv, dev_name(prv->dev), &master->flags);
	if (err)
		goto out;

	master->mode_bits	= 0;	/* SPI_MODE_0 only */
	master->bus_num		= pdev->id;
	master->num_chipselect	= pdata->num_chipselect;
	master->setup		= spi_jz4740_gpio_setup;
	master->cleanup		= spi_jz4740_gpio_cleanup;
	master->transfer_one_message = spi_jz4740_gpio_transfer_one;

	/*
	 * We don't [devm_]request_mem_region here since we don't need
	 * exclusive access to port registers. Pin access conflicts have
	 * already been resolved by gpiolib.
	 */

	prv->port_base = devm_ioremap(&pdev->dev, prv->port_addr, 0x100);
	if (!prv->port_base) {
		dev_err(prv->dev, "can't ioremap\n");
		goto out_busy;
	}

	err = spi_register_master(master);
	if (err) {
		dev_err(prv->dev, "can't register master\n");
		goto out_master;
	}

	return 0;

out_busy:
	err = -EBUSY;
out_master:
	free_gpios(prv);
out:
	platform_set_drvdata(pdev, NULL);
	spi_master_put(master);

	return err;
}

static int spi_jz4740_gpio_remove(struct platform_device *pdev)
{
	struct spi_master *master = platform_get_drvdata(pdev);
	struct spi_jz4740_gpio *prv = spi_master_get_devdata(master);

	spi_unregister_master(master);

	free_gpios(prv);

	platform_set_drvdata(pdev, NULL);
	spi_master_put(master);

	return 0;
}

static struct platform_driver spi_jz4740_gpio_driver = {
	.driver = {
		.name	= DRIVER_NAME,
		.owner	= THIS_MODULE,
	},
	.probe		= spi_jz4740_gpio_probe,
	.remove		= spi_jz4740_gpio_remove,
};
module_platform_driver(spi_jz4740_gpio_driver);

MODULE_ALIAS("platform:" DRIVER_NAME);
MODULE_DESCRIPTION("Jz4740 GPIO SPI Driver");
MODULE_AUTHOR("Werner Almesberger <werner@almesberger.net>");
MODULE_LICENSE("GPL");
