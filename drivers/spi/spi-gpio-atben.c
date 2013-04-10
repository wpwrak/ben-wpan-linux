/*
 * spi-gpio-atben.c - Bit-banging SPI host for ATBEN in the Ben NanoNote
 *
 * Written 2013 by Werner Almesberger
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2
 * as published by the Free Software Foundation.
 */

/*
 * Replace gpio_{get,set}_value with optimized macros.
 */

#define __ASM_MACH_GENERIC_GPIO_H

#define	PORT_BASE(gpio)	\
	((void __iomem *) KSEG1ADDR(JZ4740_GPIO_BASE_ADDR+0x100*((gpio) >> 5)))
#define	GPIO_MASK(gpio)	(1 << ((gpio) & 31))

#define	gpio_get_value(gpio) \
	(readl(PORT_BASE(gpio)) & GPIO_MASK(gpio))
#define	gpio_set_value(gpio, value) \
	writel(GPIO_MASK(gpio), \
	    (value) ? PORT_BASE(gpio)+0x14 : PORT_BASE(gpio)+0x18)

/* from mach-generic/gpio.h */

#define gpio_cansleep	__gpio_cansleep
int gpio_to_irq(unsigned gpio);
int irq_to_gpio(unsigned irq);
#include <asm-generic/gpio.h>		/* cansleep wrappers */

#include <asm/mach-jz4740/base.h>
#include <asm/mach-jz4740/gpio.h>


/*
 * Make specialized SPI-GPIO with inlined GPIO register access.
 */

#define	DRIVER_NAME	"spi_gpio_atben"

#define	ATBEN_MISO	10	/* DAT0, PD10 */
#define	ATBEN_MOSI	8	/* CMD, PD08 */
#define	ATBEN_SCK	11	/* DAT1, PD11 */

#define	SPI_MISO_GPIO	JZ_GPIO_PORTD(ATBEN_MISO)
#define	SPI_MOSI_GPIO	JZ_GPIO_PORTD(ATBEN_MOSI)
#define	SPI_SCK_GPIO	JZ_GPIO_PORTD(ATBEN_SCK)
#define	SPI_N_CHIPSEL	1

#include "spi-gpio.c"
