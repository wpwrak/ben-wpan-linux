/*
 * spi-gpio-atben.c - Bit-banging SPI host for ATBEN in the Ben NanoNote
 *
 * Written 2013 by Werner Almesberger
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2
 * as published by the Free Software Foundation.
 */

#include <asm/mach-jz4740/gpio.h>


#define	DRIVER_NAME	"spi_gpio_atben"

#define	ATBEN_MISO	10	/* DAT0, PD10 */
#define	ATBEN_MOSI	8	/* CMD, PD08 */
#define	ATBEN_SCK	11	/* DAT1, PD11 */

#define	SPI_MISO_GPIO	JZ_GPIO_PORTD(ATBEN_MOSI)
#define	SPI_MOSI_GPIO	JZ_GPIO_PORTD(ATBEN_MISO)
#define	SPI_SCK_GPIO	JZ_GPIO_PORTD(ATBEN_SCK)
#define	SPI_N_CHIPSEL	1

#include "spi-gpio.c"
