/*
 * Copyright 2007, Frank A Kingswood <frank@kingswood-consulting.co.uk>
 * Copyright 2007, Werner Cornelius <werner@cornelius-consult.de>
 * Copyright 2009, Boris Hajduk <boris@hajduk.org>
 *
 * ch341.c implements a serial port driver for the Winchiphead CH341.
 *
 * The CH341 device can be used to implement an RS232 asynchronous
 * serial port, an IEEE-1284 parallel printer port or a memory-like
 * interface. In all cases the CH341 supports an I2C interface as well.
 * This driver only supports the asynchronous serial interface.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version
 * 2 as published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/tty.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/usb.h>
#include <linux/usb/serial.h>
#include <linux/serial.h>
#include <asm/unaligned.h>

#define DEFAULT_BAUD_RATE 9600
#define DEFAULT_TIMEOUT   1000

#define CH341_REQ_VERSION	0x5f
#define CH341_REQ_READ_REG	0x95
#define CH341_REQ_WRITE_REG	0x9A
#define CH341_REQ_SERIAL	0xa1
#define CH341_REQ_MODEM		0xa4

/* flags for IO-Bits */
#define CH341_BIT_RTS (1 << 6)
#define CH341_BIT_DTR (1 << 5)

/******************************/
/* interrupt pipe definitions */
/******************************/
/* always 4 interrupt bytes */
/* first irq byte normally 0x08 */
/* second irq byte base 0x7d + below */
/* third irq byte base 0x94 + below */
/* fourth irq byte normally 0xee */

/* second interrupt byte */
#define CH341_MULT_STAT 0x04 /* multiple status since last interrupt event */

/* status returned in third interrupt answer byte, inverted in data
   from irq */
#define CH341_BIT_CTS 0x01
#define CH341_BIT_DSR 0x02
#define CH341_BIT_RI  0x04
#define CH341_BIT_DCD 0x08
#define CH341_BITS_MODEM_STAT 0x0f /* all bits */

/*******************************/
/* baudrate calculation factor */
/*******************************/
#define CH341_BAUDBASE_FACTOR 1532620800
#define CH341_BAUDBASE_DIVMAX 3

/* Break support - the information used to implement this was gleaned from
 * the Net/FreeBSD uchcom.c driver by Takanori Watanabe.  Domo arigato.
 */

#define CH341_REG_BREAK1       0x05
#define CH341_NBREAK_BITS_REG1 0x01


/* Mostly as per <linux/serial_reg.h> except for top bits */
#define CH341_REG_LCR	0x18
#define CH341_LCR_RX		0x80 /* enable uart rx */
#define CH341_LCR_TX		0x40 /* enable uart tx */
#define CH341_LCR_RXTX		(CH341_LCR_RX | CH341_LCR_TX)
#define CH341_LCR_SPAR		0x20 /* Stick parity */
#define CH341_LCR_EPAR		0x10 /* Even parity select */
#define CH341_LCR_PARITY	0x08 /* Parity Enable */
#define CH341_LCR_STOP		0x04 /* Stop bits: 0=1 bit, 1=2 bits */
#define CH341_LCR_WLEN5		0x00 /* Wordlength: 5 bits */
#define CH341_LCR_WLEN6		0x01 /* Wordlength: 6 bits */
#define CH341_LCR_WLEN7		0x02 /* Wordlength: 7 bits */
#define CH341_LCR_WLEN8		0x03 /* Wordlength: 8 bits */
#define CH341_LCR_WLENMASK	0x03


static const struct usb_device_id id_table[] = {
	{ USB_DEVICE(0x4348, 0x5523) },
	{ USB_DEVICE(0x1a86, 0x7523) }, /* ch340 */
	{ USB_DEVICE(0x1a86, 0x5523) }, /* ch341 */
	{ },
};
MODULE_DEVICE_TABLE(usb, id_table);

struct ch341_private {
	spinlock_t lock; /* access lock */
	unsigned baud_rate; /* set baud rate */
	u8 line_control; /* set line control value RTS/DTR */
	u8 line_status; /* active status of modem control inputs */
};

static int ch341_control_out(struct usb_device *dev, u8 request,
			     u16 value, u16 index)
{
	int r;

	dev_dbg(&dev->dev, "ch341_control_out(%02x,%04x,%04x)\n",
		(int)request, (int)value, (int)index);

	r = usb_control_msg(dev, usb_sndctrlpipe(dev, 0), request,
			    USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_DIR_OUT,
			    value, index, NULL, 0, DEFAULT_TIMEOUT);

	return r;
}

static int ch341_control_in(struct usb_device *dev,
			    u8 request, u16 value, u16 index,
			    char *buf, unsigned bufsize)
{
	int r;

	r = usb_control_msg(dev, usb_rcvctrlpipe(dev, 0), request,
			    USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_DIR_IN,
			    value, index, buf, bufsize, DEFAULT_TIMEOUT);
        if (r != bufsize) {
                dev_err(&dev->dev, "%s - failed to read [%04x]: %d\n",
			__func__, value, r);
                if (r >= 0)
                        r = -EIO;

                return r;
        }
	dev_dbg(&dev->dev, "ch341_control_in (%02x,%04x,%04x:%u->%*ph)\n",
		(int)request, (int)value, (int)index, (int)bufsize, bufsize, buf);

	return r;
}


/*
 * Read the presently configured baud rate out of the device itself
 */
static int ch341_get_baudrate(struct usb_serial_port *port,
			      unsigned int *baud)
{
	int r;
	char *buffer;
	short a, b;
	unsigned long factor;
	short divisor;
	const unsigned size = 2;
	struct usb_device *dev = port->serial->dev;

	buffer = kmalloc(size, GFP_KERNEL);
	if (!buffer)
		return -ENOMEM;

	r = ch341_control_in(dev, CH341_REQ_READ_REG, 0x1312, 0, buffer, size);
	if (r < 0) {
		dev_err(&port->dev, "%s - USB control read error (%d)\n", __func__, r);
		goto out;
	}
	a = buffer[1] << 8 | buffer[0];
	r = ch341_control_in(dev, CH341_REQ_READ_REG, 0x0f2c, 0, buffer, size);
	if (r < 0) {
		dev_err(&port->dev, "%s - USB control read error (%d)\n", __func__, r);
		goto out;
	}
	b = buffer[0];
	divisor = a & 0xff; /* Probably actually 0x03 */
	factor = (a & 0xff00) | b;
	dev_dbg(&port->dev, "%s: raw factor 0x%04lx, div: 0x%02x\n", __func__, factor, divisor);
	factor = 0x10000 - factor;
	factor <<= 3 * (3 - divisor);
	*baud = CH341_BAUDBASE_FACTOR / factor;
	dev_dbg(&port->dev, "%s: factor: %lu, divisor:%u -> baud: %u\n",
		__func__, factor, divisor, *baud);

out:
	kfree(buffer);
	return r;
}

static int ch341_get_lcr(struct usb_serial_port *port, u8 *lcr)
{
	int r;
	char *buffer;
	const unsigned size = 2;
	struct usb_device *dev = port->serial->dev;

	buffer = kmalloc(size, GFP_KERNEL);
	if (!buffer)
		return -ENOMEM;

	r = ch341_control_in(dev, CH341_REQ_READ_REG, CH341_REG_LCR, 0, buffer, size);
	if (r < 0) {
		dev_err(&port->dev, "%s - USB control read error (%d)\n", __func__, r);
		goto out;
	}
	*lcr = buffer[0];
	dev_dbg(&port->dev, "%s: raw lcr: 0x%02x\n", __func__, *lcr);

out:
	kfree(buffer);
	return r;

}

/*
 * ch340 and ch341 both claim 50-2Mbps
 */
static int ch341_set_baudrate(struct usb_serial_port *port,
			      struct ch341_private *priv)
{
	short a, b;
	int r;
	unsigned long factor;
	short divisor;

	if (!priv->baud_rate) {
		dev_err(&port->dev, "%s: karl, no baud rate to set?!\n", __func__);
		return -EINVAL;
	}
	factor = (CH341_BAUDBASE_FACTOR / priv->baud_rate);
	divisor = CH341_BAUDBASE_DIVMAX;
	dev_dbg(&port->dev, "%s: requested baudrate: %u, factor:%lu, divisor:%u\n",
		__func__, priv->baud_rate, factor, divisor);

	while ((factor > 0xfff0) && divisor) {
		factor >>= 3;
		divisor--;
	}

	if (factor > 0xfff0)
		return -EINVAL;

	factor = 0x10000 - factor;
	a = (factor & 0xff00) | divisor;
	b = factor & 0xff;
	dev_dbg(&port->dev, "%s: final a: 0x%02x b: 0x%02x\n", __func__, a, b);

	r = ch341_control_out(port->serial->dev, CH341_REQ_WRITE_REG, 0x1312, a);
	if (!r)
		r = ch341_control_out(port->serial->dev, CH341_REQ_WRITE_REG, 0x0f2c, b);

	return r;
}


static int ch341_get_termios_port(struct usb_serial_port *port,
	unsigned int *cflagp, unsigned int *baudp)
{
	int r;
	struct device *dev = &port->dev;
	struct usb_serial *serial = port->serial;
	unsigned int cflag, modem_ctl[4];
	u8 lcr;
	unsigned int baud = 0;
	unsigned int bits;

	ch341_get_baudrate(port, &baud);

	dev_dbg(dev, "%s - baud rate = %d\n", __func__, baud);
	*baudp = baud;

	cflag = *cflagp;
	if ((r = ch341_get_lcr(port, &lcr)) < 0) {
		return r;
	}
	cflag &= ~CSIZE;
	switch (lcr & CH341_LCR_WLENMASK) {
	case CH341_LCR_WLEN5:
		dev_dbg(dev, "%s - data bits = 5\n", __func__);
                cflag |= CS5;
                break;
	case CH341_LCR_WLEN6:
		dev_dbg(dev, "%s - data bits = 6\n", __func__);
                cflag |= CS6;
                break;
	case CH341_LCR_WLEN7:
		dev_dbg(dev, "%s - data bits = 7\n", __func__);
                cflag |= CS7;
                break;
	case CH341_LCR_WLEN8:
		dev_dbg(dev, "%s - data bits = 8\n", __func__);
                cflag |= CS8;
                break;
	}

	cflag &= ~CSTOPB;
	if (lcr & CH341_LCR_STOP) {
		dev_dbg(dev, "%s - stop bits = 2\n", __func__);
		cflag |= CSTOPB;
	}

	/* todo - switch and add defines instead? */
	cflag &= ~(PARENB | PARODD | CMSPAR);
	if (lcr & CH341_LCR_PARITY) {
		cflag |= PARENB;
	}
	if (!(lcr & CH341_LCR_EPAR)) {
		cflag |= PARODD;
	}
	if (lcr & CH341_LCR_SPAR) {
		cflag |= CMSPAR;
	}

        *cflagp = cflag;
	return 0;
}


/*
 * TODO - do this with what we've learnt
 */
static void ch341_get_termios(struct tty_struct *tty,
	struct usb_serial_port *port)
{
	unsigned int baud;

	if (tty) {
		ch341_get_termios_port(tty->driver_data,
			&tty->termios.c_cflag, &baud);
		tty_encode_baud_rate(tty, baud, baud);
	} else {
		int cflag;
		cflag = 0;
		ch341_get_termios_port(port, &cflag, &baud);
	}
}


static int ch341_set_handshake(struct usb_device *dev, u8 control)
{
	return ch341_control_out(dev, CH341_REQ_MODEM, ~control, 0);
}

static int ch341_get_status(struct usb_device *dev, struct ch341_private *priv)
{
	char *buffer;
	int r;
	const unsigned size = 2;
	unsigned long flags;

	buffer = kmalloc(size, GFP_KERNEL);
	if (!buffer)
		return -ENOMEM;

	r = ch341_control_in(dev, CH341_REQ_READ_REG, 0x0706, 0, buffer, size);
	if (r < 0)
		goto out;
	dev_dbg(&dev->dev, "%s received: %*ph\n", __func__, size, buffer);

	/* setup the private status if available */
	if (r == 2) {
		r = 0;
		spin_lock_irqsave(&priv->lock, flags);
		priv->line_status = (~(*buffer)) & CH341_BITS_MODEM_STAT;
		spin_unlock_irqrestore(&priv->lock, flags);
	} else
		r = -EPROTO;

out:	kfree(buffer);
	return r;
}

/* -------------------------------------------------------------------------- */

static int ch341_configure(struct usb_device *dev, struct ch341_private *priv)
{
	char *buffer;
	int r;
	const unsigned size = 2;

	buffer = kmalloc(size, GFP_KERNEL);
	if (!buffer)
		return -ENOMEM;

	r = ch341_control_in(dev, CH341_REQ_VERSION, 0, 0, buffer, size);
	if (r < 0)
		goto out;
	/* want more version codes, to work out the magic for ch341/ch340 */
	dev_dbg(&dev->dev, "version response = %*ph\n", size, buffer);

	/* Believe this resets the termios, should move to probe/attach....*/
	r = ch341_control_out(dev, CH341_REQ_SERIAL, 0, 0);
	if (r < 0)
		goto out;

	/* expect two bytes 0x56 0x00 */
	r = ch341_control_in(dev, CH341_REQ_READ_REG, 0x2518, 0, buffer, size);
	if (r < 0)
		goto out;

	r = ch341_control_out(dev, CH341_REQ_WRITE_REG, 0x2518, 0x0050);
	if (r < 0)
		goto out;

	/* expect 0xff 0xee */
	r = ch341_get_status(dev, priv);
	if (r < 0)
		goto out;

	r = ch341_control_out(dev, CH341_REQ_SERIAL, 0x501f, 0xd90a);
	if (r < 0)
		goto out;

	// FIXME after cleanup ch341_configure(), priv->line_control should be accessed under priv->lock
	r = ch341_set_handshake(dev, priv->line_control);
	if (r < 0)
		goto out;

	/* expect 0x9f 0xee */
	r = ch341_get_status(dev, priv);

out:	kfree(buffer);
	return r;
}

static int ch341_port_probe(struct usb_serial_port *port)
{
	struct ch341_private *priv;

	dev_dbg(&port->dev, "%s entered\n", __func__);
	priv = kzalloc(sizeof(struct ch341_private), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	spin_lock_init(&priv->lock);
	priv->line_control = CH341_BIT_RTS | CH341_BIT_DTR;

	usb_set_serial_port_data(port, priv);
	return 0;
}

static int ch341_port_remove(struct usb_serial_port *port)
{
	struct ch341_private *priv;

	priv = usb_get_serial_port_data(port);
	kfree(priv);

	return 0;
}

static int ch341_carrier_raised(struct usb_serial_port *port)
{
	struct ch341_private *priv = usb_get_serial_port_data(port);
	if (priv->line_status & CH341_BIT_DCD)
		return 1;
	return 0;
}

static void ch341_dtr_rts(struct usb_serial_port *port, int on)
{
	struct ch341_private *priv = usb_get_serial_port_data(port);
	unsigned long flags;
	u8 control;

	/* drop DTR and RTS */
	spin_lock_irqsave(&priv->lock, flags);
	if (on)
		priv->line_control |= CH341_BIT_RTS | CH341_BIT_DTR;
	else
		priv->line_control &= ~(CH341_BIT_RTS | CH341_BIT_DTR);
	control = priv->line_control;
	spin_unlock_irqrestore(&priv->lock, flags);
	ch341_set_handshake(port->serial->dev, control);
}

static void ch341_close(struct usb_serial_port *port)
{
	dev_dbg(&port->dev, "%s entered\n", __func__);
	usb_serial_generic_close(port);
	usb_kill_urb(port->interrupt_in_urb);
}


/* open this device, set default parameters */
static int ch341_open(struct tty_struct *tty, struct usb_serial_port *port)
{
	struct usb_serial *serial = port->serial;
	int r;

	dev_dbg(&port->dev, ">>%s entry\n", __func__);

	/* Configure the termios structure */
	ch341_get_termios(tty, port);

	dev_dbg(&port->dev, "%s - submitting interrupt urb\n", __func__);
	r = usb_submit_urb(port->interrupt_in_urb, GFP_KERNEL);
	if (r) {
		dev_err(&port->dev, "%s - failed to submit interrupt urb: %d\n",
			__func__, r);
		ch341_close(port);
		goto out;
	}

	r = usb_serial_generic_open(tty, port);
	dev_dbg(&port->dev, "<<%s exit\n", __func__);

out:	return r;
}

static void ch341_calc_parity(struct usb_serial_port *port,
	struct tty_struct *tty, u8 *lcr)
{
	if (C_PARENB(tty)) {
		*lcr |= CH341_LCR_PARITY;
		if (C_CMSPAR(tty)) {
			*lcr |= CH341_LCR_SPAR;
			if (C_PARODD(tty)) {
				dev_dbg(&port->dev, "set parity = mark\n");
				*lcr &= ~CH341_LCR_EPAR;
			} else {
				dev_dbg(&port->dev, "set parity = space\n");
				*lcr |= CH341_LCR_EPAR;
			}
		} else {
			*lcr &= ~CH341_LCR_SPAR;
			if (C_PARODD(tty)) {
				dev_dbg(&port->dev, "set parity = odd\n");
				*lcr &= ~CH341_LCR_EPAR;
			} else {
				dev_dbg(&port->dev, "set parity = even\n");
				*lcr |= CH341_LCR_EPAR;
			}
		}
	} else {
		*lcr &= ~(CH341_LCR_PARITY | CH341_LCR_SPAR | CH341_LCR_EPAR);
		dev_dbg(&port->dev, "set parity = none\n");
	}
}

/* Old_termios contains the original termios settings and
 * tty->termios contains the new setting to be used.
 */
static void ch341_set_termios(struct tty_struct *tty,
		struct usb_serial_port *port, struct ktermios *old_termios)
{
	struct ch341_private *priv = usb_get_serial_port_data(port);
	unsigned baud_rate;
	unsigned long flags;
	u8 lcr, control;

	lcr = CH341_LCR_RXTX;
	dev_dbg(&port->dev, "%s - cflag:0x%04x, oflag:0x%04x, iflag:0x%04x\n",
		__func__, tty->termios.c_cflag, tty->termios.c_oflag, tty->termios.c_iflag);

	baud_rate = tty_get_baud_rate(tty);
	dev_dbg(&port->dev, "karl - tty_get_baud_rate => %u\n", baud_rate);

	priv->baud_rate = baud_rate;

	if (baud_rate) {
		spin_lock_irqsave(&priv->lock, flags);
		priv->line_control |= (CH341_BIT_DTR | CH341_BIT_RTS);
		control = priv->line_control;
		spin_unlock_irqrestore(&priv->lock, flags);
		ch341_set_baudrate(port, priv);
	} else {
		spin_lock_irqsave(&priv->lock, flags);
		priv->line_control &= ~(CH341_BIT_DTR | CH341_BIT_RTS);
		control = priv->line_control;
		spin_unlock_irqrestore(&priv->lock, flags);
	}

	ch341_set_handshake(port->serial->dev, control);

	ch341_calc_parity(port, tty, &lcr);

	/* ch340 device datasheets doesn't mention variable data/stop bits */
	/* todo - keep version info in "priv" ? */
	lcr &= ~CH341_LCR_WLENMASK;
	switch (C_CSIZE(tty)) {
	case CS5:
		lcr |= CH341_LCR_WLEN5;
		dev_dbg(&port->dev, "set data bits = 5\n");
		break;
	case CS6:
		lcr |= CH341_LCR_WLEN6;
		dev_dbg(&port->dev, "set data bits = 6\n");
		break;
	case CS7:
		lcr |= CH341_LCR_WLEN7;
		dev_dbg(&port->dev, "set data bits = 7\n");
		break;
	case CS8:
	default:
		lcr |= CH341_LCR_WLEN8;
		dev_dbg(&port->dev, "set data bits = 8\n");
		break;
	}
	if (C_CSTOPB(tty)) {
		dev_dbg(&port->dev, "set stop bits = 2\n");
		lcr |= CH341_LCR_STOP;
	}

	ch341_control_out(port->serial->dev, CH341_REQ_WRITE_REG, CH341_REG_LCR, lcr);

	dev_dbg(&port->dev, "%s exit\n", __func__);
}

static void ch341_break_ctl(struct tty_struct *tty, int break_state)
{
	const uint16_t ch341_break_reg =
		CH341_REG_BREAK1 | ((uint16_t) CH341_REG_LCR << 8);
	struct usb_serial_port *port = tty->driver_data;
	int r;
	uint16_t reg_contents;
	uint8_t *break_reg;

	break_reg = kmalloc(2, GFP_KERNEL);
	if (!break_reg)
		return;

	r = ch341_control_in(port->serial->dev, CH341_REQ_READ_REG,
			ch341_break_reg, 0, break_reg, 2);
	if (r < 0) {
		dev_err(&port->dev, "%s - USB control read error (%d)\n",
				__func__, r);
		goto out;
	}
	dev_dbg(&port->dev, "%s - initial ch341 break register contents - reg1: %x, reg2: %x\n",
		__func__, break_reg[0], break_reg[1]);
	if (break_state != 0) {
		dev_dbg(&port->dev, "%s - Enter break state requested\n", __func__);
		break_reg[0] &= ~CH341_NBREAK_BITS_REG1;
		break_reg[1] &= ~CH341_LCR_TX;
	} else {
		dev_dbg(&port->dev, "%s - Leave break state requested\n", __func__);
		break_reg[0] |= CH341_NBREAK_BITS_REG1;
		break_reg[1] |= CH341_LCR_TX;
	}
	dev_dbg(&port->dev, "%s - New ch341 break register contents - reg1: %x, reg2: %x\n",
		__func__, break_reg[0], break_reg[1]);
	reg_contents = get_unaligned_le16(break_reg);
	r = ch341_control_out(port->serial->dev, CH341_REQ_WRITE_REG,
			ch341_break_reg, reg_contents);
	if (r < 0)
		dev_err(&port->dev, "%s - USB control write error (%d)\n",
				__func__, r);
out:
	kfree(break_reg);
}

static int ch341_tiocmset(struct tty_struct *tty,
			  unsigned int set, unsigned int clear)
{
	struct usb_serial_port *port = tty->driver_data;
	struct ch341_private *priv = usb_get_serial_port_data(port);
	unsigned long flags;
	u8 control;

	spin_lock_irqsave(&priv->lock, flags);
	if (set & TIOCM_RTS)
		priv->line_control |= CH341_BIT_RTS;
	if (set & TIOCM_DTR)
		priv->line_control |= CH341_BIT_DTR;
	if (clear & TIOCM_RTS)
		priv->line_control &= ~CH341_BIT_RTS;
	if (clear & TIOCM_DTR)
		priv->line_control &= ~CH341_BIT_DTR;
	control = priv->line_control;
	spin_unlock_irqrestore(&priv->lock, flags);
	dev_dbg(&port->dev, "%s - control = 0x%.4x\n", __func__, control);

	return ch341_set_handshake(port->serial->dev, control);
}

static void ch341_update_line_status(struct usb_serial_port *port,
					unsigned char *data, size_t len)
{
	struct ch341_private *priv = usb_get_serial_port_data(port);
	struct tty_struct *tty;
	unsigned long flags;
	u8 status;
	u8 delta;

	if (len < 4)
		return;

	status = ~data[2] & CH341_BITS_MODEM_STAT;

	spin_lock_irqsave(&priv->lock, flags);
	delta = status ^ priv->line_status;
	priv->line_status = status;
	spin_unlock_irqrestore(&priv->lock, flags);

	if (data[1] & CH341_MULT_STAT)
		dev_dbg(&port->dev, "%s - multiple status change\n", __func__);

	if (!delta)
		return;

	if (delta & CH341_BIT_CTS)
		port->icount.cts++;
	if (delta & CH341_BIT_DSR)
		port->icount.dsr++;
	if (delta & CH341_BIT_RI)
		port->icount.rng++;
	if (delta & CH341_BIT_DCD) {
		port->icount.dcd++;
		tty = tty_port_tty_get(&port->port);
		if (tty) {
			usb_serial_handle_dcd_change(port, tty,
						status & CH341_BIT_DCD);
			tty_kref_put(tty);
		}
	}

	wake_up_interruptible(&port->port.delta_msr_wait);
}

static void ch341_read_int_callback(struct urb *urb)
{
	struct usb_serial_port *port = urb->context;
	unsigned char *data = urb->transfer_buffer;
	unsigned int len = urb->actual_length;
	int status;

	switch (urb->status) {
	case 0:
		/* success */
		break;
	case -ECONNRESET:
	case -ENOENT:
	case -ESHUTDOWN:
		/* this urb is terminated, clean up */
		dev_dbg(&urb->dev->dev, "%s - urb shutting down: %d\n",
			__func__, urb->status);
		return;
	default:
		dev_dbg(&urb->dev->dev, "%s - nonzero urb status: %d\n",
			__func__, urb->status);
		goto exit;
	}

	usb_serial_debug_data(&port->dev, __func__, len, data);
	ch341_update_line_status(port, data, len);
exit:
	status = usb_submit_urb(urb, GFP_ATOMIC);
	if (status) {
		dev_err(&urb->dev->dev, "%s - usb_submit_urb failed: %d\n",
			__func__, status);
	}
}

static int ch341_tiocmget(struct tty_struct *tty)
{
	struct usb_serial_port *port = tty->driver_data;
	struct ch341_private *priv = usb_get_serial_port_data(port);
	unsigned long flags;
	u8 mcr;
	u8 status;
	unsigned int result;

	spin_lock_irqsave(&priv->lock, flags);
	mcr = priv->line_control;
	status = priv->line_status;
	dev_dbg(&port->dev, "%s - mcr: %#x, status: %#x\n", __func__, mcr, status);
	spin_unlock_irqrestore(&priv->lock, flags);

	result = ((mcr & CH341_BIT_DTR)		? TIOCM_DTR : 0)
		  | ((mcr & CH341_BIT_RTS)	? TIOCM_RTS : 0)
		  | ((status & CH341_BIT_CTS)	? TIOCM_CTS : 0)
		  | ((status & CH341_BIT_DSR)	? TIOCM_DSR : 0)
		  | ((status & CH341_BIT_RI)	? TIOCM_RI  : 0)
		  | ((status & CH341_BIT_DCD)	? TIOCM_CD  : 0);

	dev_dbg(&port->dev, "%s - result = %x\n", __func__, result);

	return result;
}

static int ch341_reset_resume(struct usb_serial *serial)
{
	struct ch341_private *priv;

	priv = usb_get_serial_port_data(serial->port[0]);

	/* reconfigure ch341 serial port after bus-reset */
	dev_dbg(&serial->dev->dev, "%s\n", __func__);
	// FIXME - this will definitely fail...
	ch341_configure(serial->dev, priv);

	return 0;
}

static struct usb_serial_driver ch341_device = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= "ch341-uart",
	},
	.id_table          = id_table,
	.num_ports         = 1,
	.open              = ch341_open,
	.dtr_rts	   = ch341_dtr_rts,
	.carrier_raised	   = ch341_carrier_raised,
	.close             = ch341_close,
	.set_termios       = ch341_set_termios,
	.break_ctl         = ch341_break_ctl,
	.tiocmget          = ch341_tiocmget,
	.tiocmset          = ch341_tiocmset,
	.tiocmiwait        = usb_serial_generic_tiocmiwait,
	.read_int_callback = ch341_read_int_callback,
	.port_probe        = ch341_port_probe,
	.port_remove       = ch341_port_remove,
	.reset_resume      = ch341_reset_resume,
};

static struct usb_serial_driver * const serial_drivers[] = {
	&ch341_device, NULL
};

module_usb_serial_driver(serial_drivers, id_table);

MODULE_LICENSE("GPL");
