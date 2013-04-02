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

/* To do:
 * - disentangle pointers between the various devices (USB, wpan, atusb)
 * - review/add locking of struct atusb
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
#include <linux/spinlock.h>
#include <linux/semaphore.h>
#include <linux/usb.h>
#include <linux/skbuff.h>

#include <net/mac802154.h>
#include <net/wpan-phy.h>

#include "at86rf230.h"
#include "atusb.h"


#define	JEDEC_ATMEL	0x1f	/* JEDEC manufacturer ID */

//#define	NUM_RX_URBS	4	/* allow for a bit of local latency */
#define	NUM_RX_URBS	1	/* allow for a bit of local latency */

struct atusb {
	struct ieee802154_dev *wpan_dev;
	struct usb_device *usb_dev;
	struct delayed_work work;	/* memory allocations */
	struct usb_anchor idle_urbs;	/* URBs waiting to be submitted */
	struct usb_anchor rx_urbs;	/* URBs waiting for reception */
	spinlock_t lock;		/* protect tx_ack_seq */
	int shutdown;			/* non-zero if shutting down */
	struct semaphore tx_sem;
	struct completion tx_complete;
	uint8_t tx_ack_seq;		/* current TX ACK sequence number */
};

/* at86rf230.h defines values as <reg, mask, shift> tuples. We use the more
 * traditional style of having registers and or-able values. SR_VALUE uses
 * the shift to prepare a value accordinly.
 */

#define	__SR_VALUE(reg, mask, shift, val)	((val) << (shift))
#define	SR_VALUE(sr, val)			__SR_VALUE(sr, (val))


/* ----- USB commands without data ----------------------------------------- */


static int atusb_command(struct atusb *atusb, uint8_t cmd, uint8_t arg)
{
	struct usb_device *usb_dev = atusb->usb_dev;

	dev_dbg(&usb_dev->dev, "atusb_command: cmd = 0x%x\n", cmd);
	return usb_control_msg(usb_dev, usb_sndctrlpipe(usb_dev, 0),
	    cmd, ATUSB_REQ_TO_DEV, arg, 0, NULL, 0, 1000);
}

static int atusb_write_reg(struct atusb *atusb, uint8_t reg, uint8_t value)
{
	struct usb_device *usb_dev = atusb->usb_dev;

	dev_dbg(&usb_dev->dev, "atusb_write_reg: 0x%02x <- 0x%02x\n",
	    reg, value);
	return usb_control_msg(usb_dev, usb_sndctrlpipe(usb_dev, 0),
	    ATUSB_REG_WRITE, ATUSB_REQ_TO_DEV, value, reg, NULL, 0, 1000);
}

static int atusb_read_reg(struct atusb *atusb, uint8_t reg)
{
	struct usb_device *usb_dev = atusb->usb_dev;
	int ret;
	uint8_t value;

	dev_dbg(&usb_dev->dev, "atusb_read_reg: reg = 0x%x\n", reg);
	ret = usb_control_msg(usb_dev, usb_rcvctrlpipe(usb_dev, 0),
	    ATUSB_REG_READ, ATUSB_REQ_FROM_DEV, 0, reg, &value, 1, 1000);
	if (ret < 0)
		return ret;
	return value;
}


/* ----- skb allocation ---------------------------------------------------- */


#define	MAX_PSDU	127
#define	MAX_RX_XFER	(1 + MAX_PSDU + 2 + 1)	/* PHR+PSDU+CRC+LQI */


#define	SKB_ATUSB(skb)	(*(struct atusb **) (skb)->cb)

static void atusb_in(struct urb *urb);

static int submit_rx_urb(struct atusb *atusb, struct urb *urb)
{
	struct usb_device *usb_dev = atusb->usb_dev;
	struct sk_buff *skb = urb->context;
	int ret;

	if (!skb) {
		skb = alloc_skb(MAX_RX_XFER, GFP_KERNEL);
		if (!skb) {
			dev_err(&usb_dev->dev,
			    "atusb_in: can't allocate skb\n");
			return -ENOMEM;
		}
		skb_put(skb, MAX_RX_XFER);
		SKB_ATUSB(skb) = atusb;
	}

	usb_fill_bulk_urb(urb, usb_dev, usb_rcvbulkpipe(usb_dev, 1),
	    skb->data, MAX_RX_XFER, atusb_in, skb);
	usb_anchor_urb(urb, &atusb->rx_urbs);

	ret = usb_submit_urb(urb, GFP_KERNEL);
	if (ret) {
		usb_unanchor_urb(urb);
		kfree_skb(skb);
		urb->context = NULL;
	}
	return ret;
}

static void work_urbs(struct work_struct *work)
{
	struct atusb *atusb =
	    container_of(to_delayed_work(work), struct atusb, work);
	struct usb_device *usb_dev = atusb->usb_dev;
	struct urb *urb;
	int ret;

	if (atusb->shutdown)
		return;
msleep(1000);
	do {
		urb = usb_get_from_anchor(&atusb->idle_urbs);
		if (!urb)
			return;
		ret = submit_rx_urb(atusb, urb);
	} while (!ret);

	usb_anchor_urb(urb, &atusb->idle_urbs);
	dev_err(&usb_dev->dev, "atusb_in: can't allocate/submit URB (%d)\n",
	    ret);
	/* try again in one jiffie */
	schedule_delayed_work(&atusb->work, 1);
}


/* ----- Asynchronous USB -------------------------------------------------- */


static void atusb_tx_done(struct atusb *atusb, uint8_t seq)
{
	struct usb_device *usb_dev = atusb->usb_dev;
	uint8_t expect;

	spin_lock(&atusb->lock);
	expect = atusb->tx_ack_seq;
	spin_unlock(&atusb->lock);

	dev_dbg(&usb_dev->dev, "atusb_tx_done (0x%02x/0x%02x)\n", seq, expect);
	if (seq == expect)
		complete(&atusb->tx_complete);
}

static void atusb_in_good(struct urb *urb)
{
	struct usb_device *usb_dev = urb->dev;
	struct sk_buff *skb = urb->context;
	struct atusb *atusb = SKB_ATUSB(skb);
	uint8_t len, lqi;

	if (!urb->actual_length) {
		dev_dbg(&usb_dev->dev, "atusb_in: zero-sized URB ?\n");
		return;
	}

	len = *skb->data;

	if (urb->actual_length == 1) {
		atusb_tx_done(atusb, len);
		return;
	}

	if (len + 1 > urb->actual_length - 1) {
		dev_dbg(&usb_dev->dev, "atusb_in: frame len %d+1 > URB %u-1\n",
		    len, urb->actual_length);
		return;
	}

	if (len < 2) {
		dev_dbg(&usb_dev->dev, "atusb_in: frame is too small\n");
		return;
	}

	lqi = skb->data[len + 1];
	dev_dbg(&usb_dev->dev, "atusb_in: rx len %d lqi 0x%02x\n", len, lqi);
	skb_pull(skb, 1);	/* remove PHR */
	skb_trim(skb, len - 2);	/* remove CRC */
	ieee802154_rx_irqsafe(atusb->wpan_dev, skb, lqi);
	urb->context = NULL;	/* skb is gone */
}

static void atusb_in(struct urb *urb)
{
	struct usb_device *usb_dev = urb->dev;
	struct sk_buff *skb = urb->context;
	struct atusb *atusb = SKB_ATUSB(skb);

	dev_dbg(&usb_dev->dev, "atusb_in: status %d len %d\n",
	    urb->status, urb->actual_length);
	if (urb->status) {
		if (urb->status == -ENOENT) { /* being killed */
			kfree_skb(skb);
			urb->context = NULL;
			return;
		}
		dev_dbg(&usb_dev->dev, "atusb_in: URB error %d\n", urb->status);
	} else {
		atusb_in_good(urb);
	}

	usb_anchor_urb(urb, &atusb->idle_urbs);
	if (!atusb->shutdown)
		schedule_delayed_work(&atusb->work, 0);
}


/* ----- URB allocation/deallocation --------------------------------------- */


static void free_urbs(struct atusb *atusb)
{
	struct urb *urb;

	while (1) {
		urb = usb_get_from_anchor(&atusb->idle_urbs);
		if (!urb)
			break;
		if (urb->context)
			kfree_skb(urb->context);
		usb_free_urb(urb);
	}
}

static int alloc_urbs(struct atusb *atusb, int n)
{
	struct urb *urb;

	while (n) {
		urb = usb_alloc_urb(0, GFP_KERNEL);
		if (!urb) {
			free_urbs(atusb);
			return -ENOMEM;
		}
		usb_anchor_urb(urb, &atusb->idle_urbs);
		n--;
	}
	return 0;
}


/* ----- IEEE 802.15.4 interface operations -------------------------------- */


static int atusb_xmit(struct ieee802154_dev *wpan_dev, struct sk_buff *skb)
{
	struct atusb *atusb = wpan_dev->priv;
	struct usb_device *usb_dev = atusb->usb_dev;
	unsigned long flags;
	int ret;

	dev_dbg(&usb_dev->dev, "atusb_xmit (%d)\n", skb->len);
	if (down_trylock(&atusb->tx_sem)) {
		dev_dbg(&usb_dev->dev, "atusb_xmit busy\n");
		return -EBUSY;
	}
	INIT_COMPLETION(atusb->tx_complete);
	ret = usb_control_msg(usb_dev, usb_sndctrlpipe(usb_dev, 0),
	    ATUSB_TX, ATUSB_REQ_TO_DEV, 0, atusb->tx_ack_seq,
	    skb->data, skb->len, 1000);
	if (ret < 0) {
		printk(KERN_WARNING "atusb_xmit: sending failed, error %d\n",
		    ret);
		goto done;
	}

	ret = wait_for_completion_interruptible_timeout(&atusb->tx_complete,
	    msecs_to_jiffies(1000));
	if (!ret)
		ret = -ETIMEDOUT;
	if (ret > 0)
		ret = 0;

done:
	spin_lock_irqsave(&atusb->lock, flags);
	atusb->tx_ack_seq++;
	spin_unlock_irqrestore(&atusb->lock, flags);

	up(&atusb->tx_sem);
	dev_dbg(&usb_dev->dev, "atusb_xmit done (%d)\n", ret);
	return ret;
}

static int atusb_channel(struct ieee802154_dev *wpan_dev, int page, int channel)
{
	struct atusb *atusb = wpan_dev->priv;
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
	struct atusb *atusb = wpan_dev->priv;
	struct usb_device *usb_dev = atusb->usb_dev;
	int ret;

	dev_dbg(&usb_dev->dev, "atusb_start\n");
	schedule_delayed_work(&atusb->work, 0);
	ret = atusb_command(atusb, ATUSB_RX_MODE, 1);
	if (ret < 0)
		usb_kill_anchored_urbs(&atusb->idle_urbs);
	return ret;
}

static void atusb_stop(struct ieee802154_dev *wpan_dev)
{
	struct atusb *atusb = wpan_dev->priv;
	struct usb_device *usb_dev = atusb->usb_dev;

	dev_dbg(&usb_dev->dev, "atusb_stop\n");
	usb_kill_anchored_urbs(&atusb->idle_urbs);
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


static int atusb_get_and_show_revision(struct atusb *atusb)
{
	struct usb_device *usb_dev = atusb->usb_dev;
	unsigned char buffer[3];
	int ret;

	/* Get a couple of the ATMega Firmware values */
	ret = usb_control_msg(usb_dev,
	    usb_rcvctrlpipe(usb_dev, 0),
	    ATUSB_ID, ATUSB_REQ_FROM_DEV, 0, 0,
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

static int atusb_get_and_show_build(struct atusb *atusb)
{
	struct usb_device *usb_dev = atusb->usb_dev;
	char build[ATUSB_BUILD_SIZE + 1];
	int ret;

	ret = usb_control_msg(usb_dev,
	    usb_rcvctrlpipe(usb_dev, 0),
	    ATUSB_BUILD, ATUSB_REQ_FROM_DEV, 0, 0,
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

static int atusb_get_and_show_chip(struct atusb *atusb)
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
	struct atusb *atusb = NULL;
	int ret = -ENOMEM;

	wpan_dev = ieee802154_alloc_device(sizeof(struct atusb), &atusb_ops);
	if (!wpan_dev)
		return -ENOMEM;

	atusb = wpan_dev->priv;
	atusb->wpan_dev = wpan_dev;
	atusb->usb_dev = usb_get_dev(usb_dev);
	usb_set_intfdata(interface, atusb);

	init_completion(&atusb->tx_complete);
	sema_init(&atusb->tx_sem, 1);
	spin_lock_init(&atusb->lock);
	init_usb_anchor(&atusb->idle_urbs);
	init_usb_anchor(&atusb->rx_urbs);
	INIT_DELAYED_WORK(&atusb->work, work_urbs);
	atusb->shutdown = 0;

	if (alloc_urbs(atusb, NUM_RX_URBS))
		goto fail;

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

	/* If we just powered on, we're now in P_ON and need to enter TRX_OFF
	 * explicitly. Any resets after that will send us straight to TRX_OFF,
	 * making the command below redundant.
	 */
	ret = atusb_write_reg(atusb, RG_TRX_STATE, STATE_FORCE_TRX_OFF);
	if (ret < 0)
		goto fail_registered;
	msleep(1);	/* reset => TRX_OFF, tTR13 = 37 us */

	ret = atusb_write_reg(atusb, RG_TRX_CTRL_2,
	    SR_VALUE(SR_RX_SAFE_MODE, 1));
	if (ret < 0)
		goto fail_registered;

	ret = atusb_write_reg(atusb, RG_IRQ_MASK, 0xff);
	if (ret >= 0)
		return 0;

fail_registered:
	ieee802154_unregister_device(wpan_dev);
fail:
	free_urbs(atusb);
	usb_put_dev(usb_dev);
	ieee802154_free_device(wpan_dev);
	return ret;
}

static void atusb_disconnect(struct usb_interface *interface)
{
	struct atusb *atusb = usb_get_intfdata(interface);

	dev_dbg(&atusb->usb_dev->dev, "atusb_disconnect\n");

	atusb->shutdown = 1;
	cancel_delayed_work_sync(&atusb->work);

	usb_kill_anchored_urbs(&atusb->rx_urbs);
	free_urbs(atusb);

	ieee802154_unregister_device(atusb->wpan_dev);

	ieee802154_free_device(atusb->wpan_dev);

	usb_set_intfdata(interface, NULL);
	usb_put_dev(atusb->usb_dev);

	printk(KERN_DEBUG "atusb_disconnect done\n");
}

/* The devices we work with */
static const struct usb_device_id atusb_device_table[] = {
{
	.match_flags		= USB_DEVICE_ID_MATCH_DEVICE |
				  USB_DEVICE_ID_MATCH_INT_INFO,
	.idVendor		= ATUSB_VENDOR_ID,
	.idProduct		= ATUSB_PRODUCT_ID,
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
