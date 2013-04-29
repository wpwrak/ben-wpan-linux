/*
 * atben-qi_lb60.c - SPI-GPIO framework for ATBEN on Jz4740
 *
 * Written 2011, 2013 by Werner Almesberger
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2
 * as published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi_gpio.h>
#include <linux/spi/at86rf230.h>
#include <asm/mach-jz4740/gpio.h>


#if defined(CONFIG_SPI_JZ4740_GPIO) || defined(CONFIG_SPI_JZ4740_GPIO_MODULE)
#define	SPI_GPIO_DRIVER	"spi_jz4740_gpio"
#else
#define	SPI_GPIO_DRIVER	"spi_gpio"
#endif

enum {
	VDD_OFF	= 2,	/* VDD disable, PD02 */
	MOSI	= 8,	/* CMD, PD08 */
	SLP_TR	= 9,	/* CLK, PD09 */
	MISO	= 10,	/* DAT0, PD10 */
	SCLK	= 11,	/* DAT1, PD11 */
	IRQ	= 12,	/* DAT2, PD12 */
	nSEL    = 13,	/* DAT3/CD, PD13 */
};


/* ----- ATBEN reset ------------------------------------------------------- */


static void atben_reset(void *dummy)
{
	const int charge = 1 << nSEL | 1 << MOSI | 1 << SLP_TR | 1 << SCLK;
	const int discharge = charge | 1 << IRQ | 1 << MISO;

	/*
	 * Note: we need the next 6 lines only if using spi-gpio.
	 * Both jz4740_mmc and spi-jz4740-gpio take care of the function
	 * setting and neither need nor are bothered by having the pins
	 * configured here.
	 */

	jz_gpio_set_function(JZ_GPIO_PORTD(MOSI), JZ_GPIO_FUNC_NONE);
	jz_gpio_set_function(JZ_GPIO_PORTD(MISO), JZ_GPIO_FUNC_NONE);
	jz_gpio_set_function(JZ_GPIO_PORTD(SCLK), JZ_GPIO_FUNC_NONE);
	jz_gpio_set_function(JZ_GPIO_PORTD(nSEL), JZ_GPIO_FUNC_NONE);
	jz_gpio_set_function(JZ_GPIO_PORTD(SLP_TR), JZ_GPIO_FUNC_NONE);
	jz_gpio_set_function(JZ_GPIO_PORTD(IRQ), JZ_GPIO_FUNC_NONE);

	jz_gpio_port_set_value(JZ_GPIO_PORTD(0), 1 << VDD_OFF, 1 << VDD_OFF);
	jz_gpio_port_direction_output(JZ_GPIO_PORTD(0), discharge);
	jz_gpio_port_set_value(JZ_GPIO_PORTD(0), 0, discharge);
	msleep(100);    /* let power drop */

	/*
	 * Hack: PD12/DAT2/IRQ is an active-high interrupt input, which is
	 * indicated by setting its direction bit to 1. We thus must not
	 * configure it as an "input".
	 */
	jz_gpio_port_direction_input(JZ_GPIO_PORTD(0), 1 << MISO);
	jz_gpio_port_set_value(JZ_GPIO_PORTD(0), charge, charge);
	msleep(10);     /* precharge caps */

	jz_gpio_port_set_value(JZ_GPIO_PORTD(0), 0,
			       1 << VDD_OFF | 1 << SLP_TR | 1 << SCLK);
	msleep(10);
}


/* ----- SPI driver/device matching ---------------------------------------- */


static const struct at86rf230_platform_data at86rf230_platform_data = {
	.rstn		= -1,	/* use reset function instead */
	.slp_tr		= -1,	/* not used */
	.dig2		= -1,
	.irq_type	= IRQF_TRIGGER_HIGH,
	.reset		= atben_reset,
};

static struct spi_board_info atben_board_info = {
	.modalias	= "at86rf230",
	/* set .irq later */
	.chip_select	= 0,
	.bus_num	= 2,
	.max_speed_hz	= 8 * 1000 * 1000,
	.controller_data = (void *) JZ_GPIO_PORTD(nSEL),
	.platform_data	= &at86rf230_platform_data,
};

struct spi_gpio_platform_data atben_spi_gpio_platform_data = {
	.mosi		= JZ_GPIO_PORTD(MOSI),
	.miso		= JZ_GPIO_PORTD(MISO),
	.sck		= JZ_GPIO_PORTD(SCLK),
	.num_chipselect	= 1,
};

static struct platform_device atben_device = {
	.name	= SPI_GPIO_DRIVER,
	.id	= 2,
	.dev	= {
		.platform_data = &atben_spi_gpio_platform_data,
	},
};

static int __init atben_init(void)
{
	atben_board_info.irq = gpio_to_irq(JZ_GPIO_PORTD(IRQ));
	spi_register_board_info(&atben_board_info, 1);

	return platform_device_register(&atben_device);
}
device_initcall(atben_init);
