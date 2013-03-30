/*
 * atusb.c - Driver for the ATUSB IEEE 802.15.4 dongle
 *
 * Written 2013 by Werner Almesberger <werner@almesberger.net>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation, version 2
 *
 * Based on at86rf230.c and spi_atusb.c.
 * at86rf230.c is
 * Copyright (C) 2009 Siemens AG
 * Written by: Dmitry Eremin-Solenikov <dmitry.baryshkov@siemens.com>
 *
 * spi_atusb.c is
 * Copyright (c) 2011 Richard Sharpe <realrichardsharpe@gmail.com>
 * Copyright (c) 2011 Stefan Schmidt <stefan@datenfreihafen.org>
 * Copyright (c) 2011 Werner Almesberger <werner@almesberger.net>
 *
 * USB initialization is
 * Copyright (c) 2013 Alexander Aring <alex.aring@gmail.com>
 */

/*
 * To do:
 * - disentangle pointers between the various devices (USB, wpan, atusb)
 * - review/add locking of atusb_local
 * - harmonize indentation style
 * - check module load/unload
 * - review dev_* severity levels and error reporting in general
 * - buffer protection (we should need only dynamic)
 * - AACK mode (needs firmware support)
 * - address filtering
 * - Q: move more severe on/off operation into start/stop ? e.g., reset
 * - think about setting power levels
 */

#define	DEBUG

#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/jiffies.h>
#include <linux/semaphore.h>
#include <linux/usb.h>
#include <linux/skbuff.h>

#include <net/mac802154.h>
#include <net/wpan-phy.h>

#include "at86rf230.h"


#define VENDOR_ID	0x20b7	/* Qi Hardware*/
#define PRODUCT_ID	0x1540	/* ATUSB */

#define	JEDEC_ATMEL	0x1f	/* JEDEC manufacturer ID */

#define ATUSB_BUILD_SIZE 256
struct atusb_local {
	struct ieee802154_dev *wpan_dev;
	struct usb_device *usb_dev;
	/* The interface to the RF part info, if applicable */
	struct urb *rx_urb;
	struct sk_buff *rx_skb;
	struct semaphore tx_sem;
	struct completion tx_complete;
};

/* Commands to our device. Make sure this is synced with the firmware */
enum atspi_requests {
	ATUSB_ID			= 0x00,	/* system status/control grp */
	ATUSB_BUILD,
	ATUSB_RESET,
	ATUSB_RF_RESET			= 0x10,	/* debug/test group */
	ATUSB_POLL_INT,
	ATUSB_TEST,			/* atusb-sil only */
	ATUSB_TIMER,
	ATUSB_GPIO,
	ATUSB_SLP_TR,
	ATUSB_GPIO_CLEANUP,
	ATUSB_REG_WRITE			= 0x20,	/* transceiver group */
	ATUSB_REG_READ,
	ATUSB_BUF_WRITE,
	ATUSB_BUF_READ,
	ATUSB_SRAM_WRITE,
	ATUSB_SRAM_READ,
	ATUSB_SPI_WRITE			= 0x30,	/* SPI group */
	ATUSB_SPI_READ1,
	ATUSB_SPI_READ2,
	ATUSB_SPI_WRITE2_SYNC,
	ATUSB_RX_MODE			= 0x40, /* HardMAC group */
	ATUSB_TX,
};

/*
 * Direction	bRequest		wValue		wIndex	wLength
 *
 * ->host	ATUSB_ID		-		-	3
 * ->host	ATUSB_BUILD		-		-	#bytes
 * host->	ATUSB_RESET		-		-	0
 *
 * host->	ATUSB_RF_RESET		-		-	0
 * ->host	ATUSB_POLL_INT		-		-	1
 * host->	ATUSB_TEST		-		-	0
 * ->host	ATUSB_TIMER		-		-	#bytes (6)
 * ->host	ATUSB_GPIO		dir+data	mask+p#	3
 * host->	ATUSB_SLP_TR		-		-	0
 * host->	ATUSB_GPIO_CLEANUP	-		-	0
 *
 * host->	ATUSB_REG_WRITE		value		addr	0
 * ->host	ATUSB_REG_READ		-		addr	1
 * host->	ATUSB_BUF_WRITE		-		-	#bytes
 * ->host	ATUSB_BUF_READ		-		-	#bytes
 * host->	ATUSB_SRAM_WRITE	-		addr	#bytes
 * ->host	ATUSB_SRAM_READ		-		addr	#bytes
 *
 * host->	ATUSB_SPI_WRITE		byte0		byte1	#bytes
 * ->host	ATUSB_SPI_READ1		byte0		-	#bytes
 * ->host	ATUSB_SPI_READ2		byte0		byte1	#bytes
 * ->host	ATUSB_SPI_WRITE2_SYNC	byte0		byte1	0/1
 *
 * host->	ATUSB_RX_MODE		on		-	0
 * host->	ATUSB_TX		flags		0	#bytes
 */

#define ATUSB_FROM_DEV (USB_TYPE_VENDOR | USB_DIR_IN)
#define ATUSB_TO_DEV (USB_TYPE_VENDOR | USB_DIR_OUT)


/* ----- USB commands without data ----------------------------------------- */


static int atusb_command(struct atusb_local *atusb, uint8_t cmd, uint8_t arg)
{
	struct usb_device *usb_dev = atusb->usb_dev;

	dev_dbg(&usb_dev->dev, "atusb_command: cmd = 0x%x\n", cmd);
	return usb_control_msg(usb_dev, usb_sndctrlpipe(usb_dev, 0),
	    cmd, ATUSB_TO_DEV, arg, 0, NULL, 0, 1000);
}

static int atusb_write_reg(struct atusb_local *atusb, uint8_t reg,
    uint8_t value)
{
	struct usb_device *usb_dev = atusb->usb_dev;

	dev_dbg(&usb_dev->dev, "atusb_write_reg: 0x%02x <- 0x%02x\n",
	    reg, value);
	return usb_control_msg(usb_dev, usb_sndctrlpipe(usb_dev, 0),
	    ATUSB_REG_WRITE, ATUSB_TO_DEV, value, reg, NULL, 0, 1000);
}

static int atusb_read_reg(struct atusb_local *atusb, uint8_t reg)
{
	struct usb_device *usb_dev = atusb->usb_dev;
	int ret;
	uint8_t value;

	dev_dbg(&usb_dev->dev, "atusb_read_reg: reg = 0x%x\n", reg);
	ret = usb_control_msg(usb_dev, usb_rcvctrlpipe(usb_dev, 0),
	    ATUSB_REG_READ, ATUSB_FROM_DEV, 0, reg, &value, 1, 1000);
	if (ret < 0)
		return ret;
	return value;
}


/* ----- Asynchronous USB -------------------------------------------------- */


#define	MAX_PSDU	127
#define	MAX_RX_XFER	(1+MAX_PSDU+2+1)	/* PHR+PSDU+CRC+LQI */


static void atusb_in(struct urb *urb);

static void atusb_tx_done(struct atusb_local *atusb)
{
	struct usb_device *usb_dev = atusb->usb_dev;

	dev_dbg(&usb_dev->dev, "atusb_tx_done\n");
	complete(&atusb->tx_complete);
}

static struct sk_buff *atusb_alloc_skb(struct device *dev)
{
	struct sk_buff *skb;

	skb = alloc_skb(MAX_RX_XFER, GFP_KERNEL);
	if (!skb) {
		dev_err(dev, "atusb_in: can't allocate skb\n");
		return NULL;
        }
	skb_put(skb, MAX_RX_XFER);
	return skb;
}

static int submit_rx_urb(struct atusb_local *atusb,
    struct urb *urb, struct sk_buff *skb)
{
	struct usb_device *usb_dev = atusb->usb_dev;
	int ret;

	usb_fill_bulk_urb(urb, usb_dev, usb_rcvbulkpipe(usb_dev, 1),
	    skb->data, MAX_RX_XFER, atusb_in, atusb);

	atusb->rx_skb = skb;

	ret = usb_submit_urb(urb, GFP_KERNEL);
	if (ret) {
		kfree_skb(skb);
		return ret;
	}

	return 0;
}

static void atusb_in(struct urb *urb)
{
	struct atusb_local *atusb = urb->context;
	struct usb_device *usb_dev = urb->dev;
	struct sk_buff *skb = atusb->rx_skb;
	uint8_t len, lqi;

	dev_dbg(&usb_dev->dev, "atusb_in: status %d len %d\n",
	    urb->status, urb->actual_length);
	if (urb->status) {
		if (urb->status == -ENOENT) { /* being killed */
			kfree_skb(skb);
			return;
		}
		dev_dbg(&usb_dev->dev, "atusb_in: URB error %d\n", urb->status);
		goto recycle;
	}

	len = *skb->data;
	if (!urb->actual_length || len+1 > urb->actual_length-1) {
		dev_dbg(&usb_dev->dev, "atusb_in: frame len %d+1 > URB %u-1\n",
		    len, urb->actual_length);
		goto recycle;
	}

	switch (len) {
	case 0:
		atusb_tx_done(atusb);
		break;
	case 1:
		dev_dbg(&usb_dev->dev, "atusb_in: frame is too small\n");
		break;
	default:
		lqi = skb->data[len+1];
		dev_dbg(&usb_dev->dev, "atusb_in: rx len %d lqi 0x%02x\n",
		    len, lqi);
		skb_pull(skb, 1);	/* remove PHR */
		skb_trim(skb, len-2);	/* remove CRC */
		ieee802154_rx_irqsafe(atusb->wpan_dev, skb, lqi);
		skb = atusb_alloc_skb(&usb_dev->dev);
		if (!skb)
			return;
		break;
	}

recycle:
	if (submit_rx_urb(atusb, urb, skb) < 0)
		dev_err(&usb_dev->dev, "atusb_in: can't submit URB\n");
}

static int start_rx_urb(struct atusb_local *atusb)
{
	struct usb_device *usb_dev = atusb->usb_dev;
	struct sk_buff *skb;
	int ret;

	dev_dbg(&usb_dev->dev, "start_rx_urb\n");
	skb = atusb_alloc_skb(&usb_dev->dev);
	if (!skb)
		return -ENOMEM;

	ret = submit_rx_urb(atusb, atusb->rx_urb, skb);
	if (ret)
		dev_err(&usb_dev->dev,
		    "start_rx_urb: can't submit URB, error %d\n",
		    ret);
	return ret;
}


/* ----- IEEE 802.15.4 interface operations -------------------------------- */


static int atusb_xmit(struct ieee802154_dev *wpan_dev, struct sk_buff *skb)
{
	struct atusb_local *atusb = wpan_dev->priv;
	struct usb_device *usb_dev = atusb->usb_dev;
	int ret;

	dev_dbg(&usb_dev->dev, "atusb_xmit\n");
	if (down_trylock(&atusb->tx_sem))
		return -EBUSY;
	INIT_COMPLETION(atusb->tx_complete);
	ret = usb_control_msg(usb_dev, usb_sndctrlpipe(usb_dev, 0),
	    ATUSB_TX, ATUSB_TO_DEV, 0, 0, skb->data, skb->len, 1000);
	if (ret < 0) {
		printk(KERN_WARNING "atusb_xmit: sending failed, error %d\n",
		    ret);
		goto done;
	}

	ret = wait_for_completion_interruptible_timeout(&atusb->tx_complete,
	    msecs_to_jiffies(1000));
done:
	up(&atusb->tx_sem);
	return ret;
}

static int atusb_channel(struct ieee802154_dev *wpan_dev, int page, int channel)
{
	struct atusb_local *atusb = wpan_dev->priv;
	int ret;

	if (page || channel < 11 || channel > 26) {
		dev_err(&atusb->usb_dev->dev,
		    "invalid channel: page %d channel %d\n", page, channel);
		return -EINVAL;
	}
	ret = atusb_write_reg(atusb, RG_PHY_CC_CCA, channel);
	if (ret < 0)
		return ret;
	msleep(1);	/* @@@ ugly synchronization */
	wpan_dev->phy->current_channel = channel;
/* @@@ set CCA mode */
	return 0;
}

static int atusb_ed(struct ieee802154_dev *wpan_dev, u8 *level)
{
	/* @@@ not used by the stack yet */
	*level = 0;
	return 0;
}

static int atusb_set_hw_addr_filt(struct ieee802154_dev *wpan_dev,
				  struct ieee802154_hw_addr_filt *filt,
				  unsigned long changed)
{
	return -EIO;
}

static int atusb_start(struct ieee802154_dev *wpan_dev)
{
	struct atusb_local *atusb = wpan_dev->priv;
	struct usb_device *usb_dev = atusb->usb_dev;
	int ret;

	dev_dbg(&usb_dev->dev, "atusb_start\n");
	ret = start_rx_urb(atusb);
	if (ret)
		return ret;
	ret = atusb_command(atusb, ATUSB_RX_MODE, 1);
	if (ret < 0)
		usb_kill_urb(atusb->rx_urb);
	return ret;
}

static void atusb_stop(struct ieee802154_dev *wpan_dev)
{
	struct atusb_local *atusb = wpan_dev->priv;
	struct usb_device *usb_dev = atusb->usb_dev;

	dev_dbg(&usb_dev->dev, "atusb_stop\n");
	usb_kill_urb(atusb->rx_urb);
	atusb_command(atusb, ATUSB_RX_MODE, 0);
}

static struct ieee802154_ops atusb_ops = {
	.owner			= THIS_MODULE,
        .xmit			= atusb_xmit,
        .ed			= atusb_ed,
        .set_channel		= atusb_channel,
        .start			= atusb_start,
        .stop			= atusb_stop,
//	.set_hw_addr_filt	= atusb_set_hw_addr_filt,
};


/* ----- Firmware and chip version information ----------------------------- */


static int atusb_get_and_show_revision(struct atusb_local *atusb)
{
	struct usb_device *usb_dev = atusb->usb_dev;
	unsigned char buffer[3];
	int ret;

	/* Get a couple of the ATMega Firmware values */
	ret = usb_control_msg(usb_dev,
	    usb_rcvctrlpipe(usb_dev, 0),
	    ATUSB_ID, ATUSB_FROM_DEV, 0, 0,
	    buffer, 3, 1000);
	if (ret < 0) {
		dev_info(&usb_dev->dev,
		    "failed submitting urb for ATUSB_ID, error %d\n", ret);
		return ret == -ENOMEM ? ret : -EIO;
	}

	dev_info(&usb_dev->dev,
	    "Firmware: major: %u, minor: %u, hardware type: %u\n",
	    buffer[0], buffer[1], buffer[2]);

	return 0;
}

static int atusb_get_and_show_build(struct atusb_local *atusb)
{
	struct usb_device *usb_dev = atusb->usb_dev;
	char build[ATUSB_BUILD_SIZE+1];
	int ret;

	ret = usb_control_msg(usb_dev,
	    usb_rcvctrlpipe(usb_dev, 0),
	    ATUSB_BUILD, ATUSB_FROM_DEV, 0, 0,
	    build, ATUSB_BUILD_SIZE, 1000);
	if (ret < 0) {
		dev_err(&usb_dev->dev,
		    "failed submitting urb for ATUSB_BUILD, error %d\n",
		    ret);
		return ret == -ENOMEM ? ret : -EIO;
	}

	build[ret] = 0;
	dev_info(&usb_dev->dev, "Firmware: build %s\n", build);

	return 0;
}

static int atusb_get_and_show_chip(struct atusb_local *atusb)
{
	struct usb_device *usb_dev = atusb->usb_dev;
	int man_id_0, man_id_1, part_num, version_num;

	man_id_0 = atusb_read_reg(atusb, RG_MAN_ID_0);
	man_id_1 = atusb_read_reg(atusb, RG_MAN_ID_1);
	part_num = atusb_read_reg(atusb, RG_PART_NUM);
	version_num = atusb_read_reg(atusb, RG_VERSION_NUM);

	if (man_id_0 < 0 || man_id_1 < 0 || part_num < 0 || version_num < 0) {
		dev_err(&usb_dev->dev, "can't read chip ID\n");
		return -EIO;
	}

	if ((man_id_1 << 8 | man_id_0) != JEDEC_ATMEL) {
		dev_err(&usb_dev->dev,
		    "non-Atmel transceiver xxxx%02x%02x\n",
		    man_id_1, man_id_0);
		return -ENODEV;
	}
	if (part_num != 3) {
		dev_err(&usb_dev->dev,
		    "unexpected transceiver, part 0x%02x version 0x%02x\n",
		    part_num, version_num);
		return -ENODEV;
	}

	dev_info(&usb_dev->dev, "ATUSB: AT86RF231 version %d\n", version_num);

	return 0;
}


/* ----- Setup ------------------------------------------------------------- */


static int atusb_probe(struct usb_interface *interface,
		       const struct usb_device_id *id)
{
	struct usb_device *usb_dev = interface_to_usbdev(interface);
	struct ieee802154_dev *wpan_dev;
	struct atusb_local *atusb = NULL;
	int ret = -ENOMEM;

	wpan_dev =
	    ieee802154_alloc_device(sizeof(struct atusb_local), &atusb_ops);
	if (!wpan_dev)
		return -ENOMEM;

	atusb = wpan_dev->priv;
	atusb->wpan_dev = wpan_dev;
	atusb->usb_dev = usb_get_dev(usb_dev);
	usb_set_intfdata(interface, atusb);

	init_completion(&atusb->tx_complete);
	sema_init(&atusb->tx_sem, 1);
	atusb->rx_urb = usb_alloc_urb(0, GFP_KERNEL);
	if (!atusb->rx_urb)
		goto fail_nourb;

	wpan_dev->parent = &usb_dev->dev;
	wpan_dev->extra_tx_headroom = 0;
	wpan_dev->flags = IEEE802154_HW_OMIT_CKSUM;

	wpan_dev->phy->current_page = 0;
	wpan_dev->phy->current_channel = 11;	/* reset default */
	wpan_dev->phy->channels_supported[0] = 0x7FFF800;

	ret = atusb_command(atusb, ATUSB_RF_RESET, 0);
	if (ret < 0) {
		dev_err(&atusb->usb_dev->dev,
			"%s: reset failed, error = %d\n",
			__func__, ret);
		goto fail;
	}

	ret = atusb_get_and_show_chip(atusb);
	if (ret)
		goto fail;
	ret = atusb_get_and_show_revision(atusb);
	if (ret)
		goto fail;
	ret = atusb_get_and_show_build(atusb);
	if (ret)
		goto fail;

	ret = ieee802154_register_device(wpan_dev);
	if (ret)
		goto fail;

	/*
	 * If we just powered on, we're now in P_ON and need to enter TRX_OFF
	 * explicitly. Any resets after that will send us straight to TRX_OFF,
	 * making the command below redundant.
	 */
	ret = atusb_write_reg(atusb, RG_TRX_STATE, STATE_FORCE_TRX_OFF);
	if (ret < 0)
		goto fail_registered;
	msleep(1);	/* reset => TRX_OFF, tTR13 = 37 us */

	ret = atusb_write_reg(atusb, RG_IRQ_MASK, 0xff);
	if (ret >= 0)
		return 0;

fail_registered:
	ieee802154_unregister_device(wpan_dev);
fail:
	usb_free_urb(atusb->rx_urb);
fail_nourb:
	usb_put_dev(usb_dev);
	ieee802154_free_device(wpan_dev);
	return ret;
}

static void atusb_disconnect(struct usb_interface *interface)
{
	struct atusb_local *atusb = usb_get_intfdata(interface);

	/* @@@ this needs some extra protecion - wa */
	usb_kill_urb(atusb->rx_urb);

	ieee802154_unregister_device(atusb->wpan_dev);

	usb_free_urb(atusb->rx_urb);
	/* @@@ do we need to check tx_sem here ? */

	ieee802154_free_device(atusb->wpan_dev);

	usb_set_intfdata(interface, NULL);
	usb_put_dev(atusb->usb_dev);
}

/* The devices we work with */
static const struct usb_device_id atusb_device_table[] = {
{
	.match_flags		= USB_DEVICE_ID_MATCH_DEVICE |
				  USB_DEVICE_ID_MATCH_INT_INFO,
	.idVendor		= VENDOR_ID,
	.idProduct		= PRODUCT_ID,
	.bInterfaceClass	= USB_CLASS_VENDOR_SPEC
},
	/* end with null element */
	{}
};
MODULE_DEVICE_TABLE(usb, atusb_device_table);

static struct usb_driver atusb_driver = {
	.name		= "atusb",
	.probe		= atusb_probe,
	.disconnect	= atusb_disconnect,
	.id_table	= atusb_device_table,
};
module_usb_driver(atusb_driver);

MODULE_AUTHOR("Alexander Aring <alex.aring@gmail.com>");
MODULE_AUTHOR("Richard Sharpe <realrichardsharpe@gmail.com>");
MODULE_AUTHOR("Stefan Schmidt <stefan@datenfreihafen.org>");
MODULE_AUTHOR("Werner Almesberger <werner@almesberger.net>");
MODULE_DESCRIPTION("ATUSB IEEE 802.15.4 Driver");
MODULE_LICENSE("GPL");
