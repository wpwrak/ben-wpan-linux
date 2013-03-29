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
 * - add locking of atusb_local
 * - avoid "dev" name
 * - harmonize indentation style
 * - check module load/unload
 * - review dev_* severity levels and error reporting in general
 */

#define	DEBUG

#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/jiffies.h>
#include <linux/interrupt.h>
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
	struct ieee802154_dev *dev;
	struct usb_device *udev;
	/* The interface to the RF part info, if applicable */
	struct urb *rx_urb;
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
	dev_dbg(&atusb->udev->dev, "atusb_command: cmd = 0x%x\n", cmd);
	return usb_control_msg(atusb->udev, usb_sndctrlpipe(atusb->udev, 0),
	    cmd, ATUSB_TO_DEV, arg, 0, NULL, 0, 1000);
}

static int atusb_write_reg(struct atusb_local *atusb, uint8_t reg,
    uint8_t value)
{
	dev_dbg(&atusb->udev->dev, "atusb_write_reg: 0x%02x <- 0x%02x\n",
	    reg, value);
	return usb_control_msg(atusb->udev, usb_sndctrlpipe(atusb->udev, 0),
	    ATUSB_REG_WRITE, ATUSB_TO_DEV, value, reg, NULL, 0, 1000);
}

static int atusb_read_reg(struct atusb_local *atusb, uint8_t reg)
{
	int retval;
	uint8_t value;

	dev_dbg(&atusb->udev->dev, "atusb_read_reg: reg = 0x%x\n", reg);
	retval = usb_control_msg(atusb->udev, usb_rcvctrlpipe(atusb->udev, 0),
	    ATUSB_REG_READ, ATUSB_FROM_DEV, 0, reg, &value, 1, 1000);
	if (retval < 0)
		return retval;
	return value;
}


/* ----- Asynchronous USB -------------------------------------------------- */


#define	MAX_PDU	128

static void atusb_in(struct urb *urb);

static void atusb_tx_done(struct atusb_local *atusb)
{
	struct usb_device *dev = atusb->udev;

	dev_dbg(&dev->dev, "atusb_tx_done\n");
	complete(&atusb->tx_complete);
}

static struct sk_buff *atusb_alloc_skb(struct device *dev)
{
	struct sk_buff *skb;

	skb = alloc_skb(MAX_PDU, GFP_KERNEL);
	if (!skb) {
		dev_err(dev, "atusb_in: can't allocate skb\n");
		return NULL;
        }
	skb_put(skb, MAX_PDU);
	return skb;
}

static int submit_rx_urb(struct atusb_local *atusb,
    struct urb *urb, struct sk_buff *skb)
{
	struct usb_device *dev = atusb->udev;
	int retval;

	usb_fill_bulk_urb(urb, dev, usb_rcvbulkpipe(dev, 1),
	    skb->data, MAX_PDU, atusb_in, atusb);

	retval = usb_submit_urb(urb, GFP_KERNEL);
	if (retval) {
		kfree_skb(skb);
		return retval;
	}

	return 0;
}

static void atusb_in(struct urb *urb)
{
	struct sk_buff *skb = urb->context;
	struct usb_device *dev = urb->dev;
	struct atusb_local *atusb =
	    usb_get_intfdata(to_usb_interface(&dev->dev));
	uint8_t len;

	dev_dbg(&dev->dev, "atusb_in: status %d len %d\n",
	    urb->status, urb->actual_length);
	if (urb->status) {
		if (urb->status == -ENOENT) { /* being killed */
			kfree_skb(skb);
			return;
		}
		dev_dbg(&dev->dev, "atusb_in: URB error %d\n", urb->status);
		goto recycle;
	}

	len = *skb->data;
	if (!urb->actual_length || len + 1 > urb->actual_length) {
		dev_dbg(&dev->dev, "atusb_in: frame len %d + 1 > URB %u\n",
		    len, urb->actual_length);
		goto recycle;
	}

	switch (len) {
	case 0:
		atusb_tx_done(atusb);
		break;
	case 1:
		dev_dbg(&dev->dev, "atusb_in: frame is too small\n");
		break;
	default:
		skb_trim(skb, len - 2); /* remove CRC */
		ieee802154_rx_irqsafe(atusb->dev, skb, 0); /* @@@ lqi */
		skb = atusb_alloc_skb(&dev->dev);
		if (!skb)
			return;
		break;
	}

recycle:
	if (submit_rx_urb(atusb, urb, skb) < 0)
		dev_err(&dev->dev, "atusb_in: can't submit URB\n");
}

static int start_rx_urb(struct atusb_local *atusb)
{
	struct usb_device *dev = atusb->udev;
	struct sk_buff *skb;
	int retval;

	dev_dbg(&dev->dev, "start_rx_urb\n");
	skb = atusb_alloc_skb(&dev->dev);
	if (!skb)
		return -ENOMEM;

	retval = submit_rx_urb(atusb, atusb->rx_urb, skb);
	if (retval)
		dev_err(&dev->dev, "start_rx_urb: can't submit URB, error %d\n",
		    retval);
	return retval;
}


/* ----- IEEE 802.15.4 interface operations -------------------------------- */


static int atusb_xmit(struct ieee802154_dev *dev, struct sk_buff *skb)
{
	struct atusb_local *atusb = dev->priv;
	struct usb_device *usb_dev = atusb->udev;
	int retval;

	dev_dbg(&usb_dev->dev, "atusb_xmit\n");
	if (!completion_done(&atusb->tx_complete))
		return -EBUSY;
	INIT_COMPLETION(atusb->tx_complete);
	retval = usb_control_msg(atusb->udev, usb_sndctrlpipe(atusb->udev, 0),
	    ATUSB_TX, ATUSB_TO_DEV, 0, 0, skb->data, skb->len, 1000);
	if (retval < 0) {
		printk(KERN_WARNING "atusb_xmit: sending failed, error %d\n",
		    retval);
		return retval;
	}

	return wait_for_completion_interruptible_timeout(&atusb->tx_complete,
	    msecs_to_jiffies(1000));
}

static int atusb_channel(struct ieee802154_dev *dev, int page, int channel)
{
	struct atusb_local *atusb = dev->priv;
	int retval;

	if (page || channel < 11 || channel > 26) {
		dev_err(&atusb->udev->dev,
		    "invalid channel: page %d channel %d\n", page, channel);
		return -EINVAL;
	}
	retval = atusb_write_reg(atusb, RG_PHY_CC_CCA, channel);
	if (retval)
		return retval;
	msleep(1);	/* @@@ ugly synchronization */
	dev->phy->current_page = page;
	dev->phy->current_channel = channel;
/* @@@ set CCA mode */
	return 0;
}

static int atusb_ed(struct ieee802154_dev *dev, u8 *level)
{
	might_sleep();
	*level = 0xbe;	/* @@@ implement */
	return 0;
}

static int atusb_set_hw_addr_filt(struct ieee802154_dev *dev,
				  struct ieee802154_hw_addr_filt *filt,
				  unsigned long changed)
{
	return -EIO;
}

static int atusb_start(struct ieee802154_dev *dev)
{
	struct atusb_local *atusb = dev->priv;
	struct usb_device *usb_dev = atusb->udev;
	int retval;

	dev_dbg(&usb_dev->dev, "atusb_start\n");
	retval = start_rx_urb(atusb);
	if (retval)
		return retval;
	retval = atusb_command(atusb, ATUSB_RX_MODE, 1);
	if (retval < 0)
		usb_kill_urb(atusb->rx_urb);
	return retval;
}

static void atusb_stop(struct ieee802154_dev *dev)
{
	struct atusb_local *atusb = dev->priv;
	struct usb_device *usb_dev = atusb->udev;

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
	struct usb_device *dev = atusb->udev;
	unsigned char buffer[3];
	int retval;

	/* Get a couple of the ATMega Firmware values */
	retval = usb_control_msg(dev,
	    usb_rcvctrlpipe(dev, 0),
	    ATUSB_ID, ATUSB_FROM_DEV, 0, 0,
	    buffer, 3, 1000);
	if (retval < 0) {
		dev_info(&dev->dev,
		    "failed submitting urb for ATUSB_ID, error %d\n", retval);
		return retval == -ENOMEM ? retval : -EIO;
	}


	dev_info(&dev->dev,
	    "Firmware: major: %u, minor: %u, hardware type: %u\n",
	    buffer[0], buffer[1], buffer[2]);

	return 0;
}

static int atusb_get_and_show_build(struct atusb_local *atusb)
{
	struct usb_device *dev = atusb->udev;
	char build[ATUSB_BUILD_SIZE + 1];
	int retval;

	retval = usb_control_msg(dev,
	    usb_rcvctrlpipe(atusb->udev, 0),
	    ATUSB_BUILD, ATUSB_FROM_DEV, 0, 0,
	    build, ATUSB_BUILD_SIZE, 1000);
	if (retval < 0) {
		dev_err(&dev->dev,
		    "failed submitting urb for ATUSB_BUILD, error %d\n",
		    retval);
		return retval == -ENOMEM ? retval : -EIO;
	}

	build[retval] = 0;
	dev_info(&dev->dev, "Firmware: build %s\n", build);

	return 0;
}

static int atusb_get_and_show_chip(struct atusb_local *atusb)
{
	struct usb_device *dev = atusb->udev;
	int man_id_0, man_id_1, part_num, version_num;

	man_id_0 = atusb_read_reg(atusb, RG_MAN_ID_0);
	man_id_1 = atusb_read_reg(atusb, RG_MAN_ID_1);
	part_num = atusb_read_reg(atusb, RG_PART_NUM);
	version_num = atusb_read_reg(atusb, RG_VERSION_NUM);

	if (man_id_0 < 0 || man_id_1 < 0 || part_num < 0 || version_num < 0) {
		dev_err(&dev->dev, "can't read chip ID\n");
		return -EIO;
	}

	if ((man_id_1 << 8 | man_id_0) != JEDEC_ATMEL) {
		dev_err(&dev->dev,
		    "non-Atmel transceiver xxxx%02x%02x\n",
		    man_id_1, man_id_0);
		return -ENODEV;
	}
	if (part_num != 3) {
		dev_err(&dev->dev,
		    "unexpected transceiver, part 0x%02x version 0x%02x\n",
		    part_num, version_num);
		return -ENODEV;
	}

	dev_info(&dev->dev, "ATUSB: AT86RF231 version %d\n", version_num);

	return 0;
}


/* ----- Setup ------------------------------------------------------------- */


static int atusb_probe(struct usb_interface *interface,
		       const struct usb_device_id *id)
{
	struct usb_device *udev = interface_to_usbdev(interface);
	struct ieee802154_dev *dev;
	struct atusb_local *atusb = NULL;
	int retval = -ENOMEM;

	dev = ieee802154_alloc_device(sizeof(struct atusb_local), &atusb_ops);
	if (!dev)
		return -ENOMEM;

	atusb = dev->priv;
	atusb->dev = dev;
	atusb->udev = usb_get_dev(udev);
	usb_set_intfdata(interface, atusb);

	init_completion(&atusb->tx_complete);
	atusb->rx_urb = usb_alloc_urb(0, GFP_KERNEL);
	if (!atusb->rx_urb)
		goto fail_nourb;

	dev->parent = &udev->dev;
	dev->extra_tx_headroom = 0;
	dev->phy->channels_supported[0] = 0x7FFF800;
	dev->flags = IEEE802154_HW_OMIT_CKSUM;

	retval = atusb_command(atusb, ATUSB_RF_RESET, 0);
	if (retval < 0) {
		dev_err(&atusb->udev->dev,
			"%s: error doing reset retval = %d\n",
			__func__, retval);
		goto fail;
	}

	retval = atusb_get_and_show_chip(atusb);
	if (retval)
		goto fail;
	retval = atusb_get_and_show_revision(atusb);
	if (retval)
		goto fail;
	retval = atusb_get_and_show_build(atusb);
	if (retval)
		goto fail;

	retval = ieee802154_register_device(dev);
	if (retval)
		goto fail;

	retval = atusb_write_reg(atusb, RG_TRX_STATE, STATE_FORCE_TRX_OFF);
	if (retval)
		goto fail_registered;
	retval = atusb_write_reg(atusb, RG_IRQ_MASK, 0xff);
	if (!retval)
		return 0;

fail_registered:
	ieee802154_unregister_device(dev);
fail:
	usb_free_urb(atusb->rx_urb);
fail_nourb:
	usb_put_dev(udev);
	ieee802154_free_device(dev);
	return retval;
}

static void atusb_disconnect(struct usb_interface *interface)
{
	struct atusb_local *atusb = usb_get_intfdata(interface);

	/* @@@ this needs some extra protecion - wa */
	usb_kill_urb(atusb->rx_urb);
	usb_free_urb(atusb->rx_urb);
#if 0 /* @@@ check xmit synchronization */
	if (atusb->tx_urb)
		usb_kill_urb(atusb->tx_urb);
#endif

	ieee802154_unregister_device(atusb->dev);
	ieee802154_free_device(atusb->dev);

	usb_set_intfdata(interface, NULL);
	usb_put_dev(atusb->udev);
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
