#include <linux/slab.h>
#include <linux/module.h>
#include <linux/usb/input.h>


//data[3]
#define BUTTON_A 0x10
#define BUTTON_B 0x20
#define BUTTON_X 0x40
#define BUTTON_Y 0x80

//data[2]
#define BUTTON_UP 		0x01
#define BUTTON_DOWN 	0x02
#define BUTTON_LEFT 	0x04
#define BUTTON_RIGHT	0x08

//data[2]
#define BUTTON_SELECT 	0x20
#define BUTTON_START	0x10
#define BUTTON_MODE		0x04

//data[3]
#define BUTTON_L1 	0x01
#define BUTTON_R1	0x02

//data[2]
#define BUTTON_L3 	0x40
#define BUTTON_R3	0x80

#define GAMEPAD_PKT_LEN 64
#define USB_GAMEPAD_MINOR_BASE	192
#define MAX_TRANSFER (PAGE_SIZE - 512)
#define WRITES_IN_FLIGHT 8

#define DRIVER_AUTHOR "Suslikov Daniil"
#define DRIVER_DESC "My  Gamepad driver"

#define XPAD_PKT_LEN 64

/* xbox d-pads should map to buttons, as is required for DDR pads
   but we map them to axes when possible to simplify things */
#define MAP_DPAD_TO_BUTTONS		(1 << 0)
#define MAP_TRIGGERS_TO_BUTTONS		(1 << 1)
#define MAP_STICKS_TO_NULL		(1 << 2)
#define DANCEPAD_MAP_CONFIG	(MAP_DPAD_TO_BUTTONS |			\
				MAP_TRIGGERS_TO_BUTTONS | MAP_STICKS_TO_NULL)

#define XTYPE_XBOX        0
#define XTYPE_XBOX360     1
#define XTYPE_XBOX360W    2
#define XTYPE_XBOXONE     3
#define XTYPE_UNKNOWN     4

static bool dpad_to_buttons;
module_param(dpad_to_buttons, bool, S_IRUGO);
MODULE_PARM_DESC(dpad_to_buttons, "Map D-PAD to buttons rather than axes for unknown pads");

static bool triggers_to_buttons;
module_param(triggers_to_buttons, bool, S_IRUGO);
MODULE_PARM_DESC(triggers_to_buttons, "Map triggers to buttons rather than axes for unknown pads");

static bool sticks_to_null;
module_param(sticks_to_null, bool, S_IRUGO);
MODULE_PARM_DESC(sticks_to_null, "Do not map sticks at all for unknown pads");

static const struct xpad_device {
	u16 idVendor;
	u16 idProduct;
	char *name;
	u8 mapping;
	u8 xtype;
} xpad_device[] = {
	{ 0x046d, 0xc21d, "Logitech Gamepad F310", 0, XTYPE_XBOX360 },
	{ 0x046d, 0xc21e, "Logitech Gamepad F510", 0, XTYPE_XBOX360 },
	{ 0x046d, 0xc21f, "Logitech Gamepad F710", 0, XTYPE_XBOX360 }
};

/* buttons shared with xbox and xbox360 */
static const signed short xpad_common_btn[] = {
	BTN_A, BTN_B, BTN_X, BTN_Y,			/* "analog" buttons */
	BTN_START, BTN_SELECT, BTN_THUMBL, BTN_THUMBR,	/* start/back/sticks */
	-1						/* terminating entry */
};

/* original xbox controllers only */
static const signed short xpad_btn[] = {
	BTN_C, BTN_Z,		/* "analog" buttons */
	-1			/* terminating entry */
};

/* used when dpad is mapped to buttons */
static const signed short xpad_btn_pad[] = {
	BTN_TRIGGER_HAPPY1, BTN_TRIGGER_HAPPY2,		/* d-pad left, right */
	BTN_TRIGGER_HAPPY3, BTN_TRIGGER_HAPPY4,		/* d-pad up, down */
	-1				/* terminating entry */
};

/* used when triggers are mapped to buttons */
static const signed short xpad_btn_triggers[] = {
	BTN_TL2, BTN_TR2,		/* triggers left/right */
	-1
};


static const signed short xpad360_btn[] = {  /* buttons for x360 controller */
	BTN_TL, BTN_TR,		/* Button LB/RB */
	BTN_MODE,		/* The big X button */
	-1
};

static const signed short xpad_abs[] = {
	ABS_X, ABS_Y,		/* left stick */
	ABS_RX, ABS_RY,		/* right stick */
	-1			/* terminating entry */
};

/* used when dpad is mapped to axes */
static const signed short xpad_abs_pad[] = {
	ABS_HAT0X, ABS_HAT0Y,	/* d-pad axes */
	-1			/* terminating entry */
};

/* used when triggers are mapped to axes */
static const signed short xpad_abs_triggers[] = {
	ABS_Z, ABS_RZ,		/* triggers left/right */
	-1
};

static const signed short gamepad_buttons[] = {
	BTN_LEFT, BTN_RIGHT, BTN_MIDDLE, BTN_SIDE,
	KEY_ESC, KEY_LEFTCTRL, KEY_LEFTALT,
	KEY_PAGEDOWN, KEY_PAGEUP,
	KEY_LEFTSHIFT, KEY_ENTER,	
	-1 };

static const signed short directional_buttons[] = {
	KEY_LEFT, KEY_RIGHT,		/* d-pad left, right */
	KEY_UP, KEY_DOWN,		/* d-pad up, down */
	-1 };

static const signed short trigger_buttons[] = {
	BTN_TL2, BTN_TR2,		/* triggers left/right */
	-1 };

static const signed short trigger_bumpers[] = {
	ABS_Z, ABS_RZ,		/* triggers left/right */
	-1 };

static const signed short gamepad_abs[] = {
	ABS_X, ABS_Y, 		/* left stick */
	ABS_RX,	ABS_WHEEL,	/* right stick */
	-1 };


/*
 * Xbox 360 has a vendor-specific class, so we cannot match it with only
 * USB_INTERFACE_INFO (also specifically refused by USB subsystem), so we
 * match against vendor id as well. Wired Xbox 360 devices have protocol 1,
 * wireless controllers have protocol 129.
 */
#define XPAD_XBOX360_VENDOR_PROTOCOL(vend,pr) \
	.match_flags = USB_DEVICE_ID_MATCH_VENDOR | USB_DEVICE_ID_MATCH_INT_INFO, \
	.idVendor = (vend), \
	.bInterfaceClass = USB_CLASS_VENDOR_SPEC, \
	.bInterfaceSubClass = 93, \
	.bInterfaceProtocol = (pr)
#define XPAD_XBOX360_VENDOR(vend) \
	{ XPAD_XBOX360_VENDOR_PROTOCOL(vend,1) }, \
	{ XPAD_XBOX360_VENDOR_PROTOCOL(vend,129) }

/* The Xbox One controller uses subclass 71 and protocol 208. */
#define XPAD_XBOXONE_VENDOR_PROTOCOL(vend, pr) \
	.match_flags = USB_DEVICE_ID_MATCH_VENDOR | USB_DEVICE_ID_MATCH_INT_INFO, \
	.idVendor = (vend), \
	.bInterfaceClass = USB_CLASS_VENDOR_SPEC, \
	.bInterfaceSubClass = 71, \
	.bInterfaceProtocol = (pr)
#define XPAD_XBOXONE_VENDOR(vend) \
	{ XPAD_XBOXONE_VENDOR_PROTOCOL(vend, 208) }

static struct usb_device_id xpad_table[] = {
	{ USB_INTERFACE_INFO('X', 'B', 0) },	/* X-Box USB-IF not approved class */
	XPAD_XBOX360_VENDOR(0x046d),		/* Logitech X-Box 360 style controllers */
	{ }
};

MODULE_DEVICE_TABLE(usb, xpad_table);

struct usb_xpad {
	struct input_dev *dev;		/* input device interface */
	struct usb_device *udev;	/* usb device */
	struct usb_interface *intf;	/* usb interface */

	int pad_present;

	struct urb *irq_in;		/* urb for interrupt in report */
	unsigned char *idata;		/* input data */
	dma_addr_t idata_dma;

	struct urb *bulk_out;
	unsigned char *bdata;

	struct urb *irq_out;		/* urb for interrupt out report */
	unsigned char *odata;		/* output data */
	dma_addr_t odata_dma;
	struct mutex odata_mutex;

	char phys[64];			/* physical device path */

	int mapping;			/* map d-pad to buttons or to axes */
	int xtype;			/* type of xbox device */
};

static void xpad_irq_in(struct urb *urb)
{
	unsigned char *data = urb->transfer_buffer;
	struct usb_xpad *xpad = urb->context;
	struct input_dev *dev = xpad->dev;

	int retval;

	switch (urb->status) {
	case 0:
		/* success */
		break;
	case -ECONNRESET:
	case -ENOENT:
	case -ESHUTDOWN:
		/* this urb is terminated, clean up */
		printk("%s - urb shutting down with status: %d", __func__, urb->status);
		return;
	default:
		printk("%s - nonzero urb status received: %d", __func__, urb->status);
		goto exit;
	}

	input_report_key(dev, KEY_LEFT, data[2] & BUTTON_LEFT);
	input_report_key(dev, KEY_RIGHT, data[2] & BUTTON_RIGHT);
	input_report_key(dev, KEY_UP, data[2] & BUTTON_UP);
	input_report_key(dev, KEY_DOWN, data[2] & BUTTON_DOWN);
	

	input_report_key(dev, BTN_LEFT,  data[3] & BUTTON_A);
	input_report_key(dev, BTN_RIGHT,  data[3] & BUTTON_B);
	//input_report_key(dev, BTN_MIDDLE,  data[3] & BTN_X);
	//input_report_key(dev, BTN_SIDE,  data[3] & BTN_Y);
	input_report_key(dev, KEY_LEFTSHIFT, data[3] & BUTTON_L1);
	input_report_key(dev, KEY_ENTER, data[3] & BUTTON_R1);
	
	/* start/back buttons */
	input_report_key(dev, KEY_ESC,  data[2] & BUTTON_START);
	input_report_key(dev, KEY_LEFTCTRL, data[2] & BUTTON_SELECT);
	input_report_key(dev, KEY_LEFTALT, data[3] & BUTTON_MODE);

	/* stick press left/right */
	input_report_key(dev, KEY_PAGEDOWN, data[2] & BUTTON_L3);
	input_report_key(dev, KEY_PAGEUP, data[2] & BUTTON_R3);

	//digital bumpers
	// input_report_key(dev, BTN_TL2, data[4]);
	// input_report_key(dev, BTN_TR2, data[5]);

	//analog bumpers
	// input_report_abs(dev, ABS_VOLUME, data[4]);
	// input_report_abs(dev, ABS_VOLUME, data[5]);

	/* left stick */
	input_report_rel(dev, REL_X, (__s16) le16_to_cpup((__le16 *)(data + 6))/2048);
	input_report_rel(dev, REL_Y, ~(__s16) le16_to_cpup((__le16 *)(data + 8))/2048);

	/* right stick */
	input_report_rel(dev, REL_HWHEEL, (__s16) le16_to_cpup((__le16 *)(data + 10))/8192);
	input_report_rel(dev, ABS_WHEEL, ~(__s16) le16_to_cpup((__le16 *)(data + 12))/8192);

	input_sync(dev);

exit:
	retval = usb_submit_urb(urb, GFP_KERNEL);
	if (retval)
		dev_err(&urb->dev->dev, "%s - Error %d submitting interrupt urb\n", __func__, retval);
}

static void xpad_bulk_out(struct urb *urb)
{
	printk("Check7.\n");
	struct usb_xpad *xpad = urb->context;
	struct device *dev = &xpad->intf->dev;

	switch (urb->status) {
	case 0:
		/* success */
		break;
	case -ECONNRESET:
	case -ENOENT:
	case -ESHUTDOWN:
		/* this urb is terminated, clean up */
		dev_dbg(dev, "%s - urb shutting down with status: %d\n",
			__func__, urb->status);
		break;
	default:
		dev_dbg(dev, "%s - nonzero urb status received: %d\n",
			__func__, urb->status);
	}
}

static void xpad_irq_out(struct urb *urb)
{
	printk("Check8.\n");
	struct usb_xpad *xpad = urb->context;
	struct device *dev = &xpad->intf->dev;
	int retval, status;

	status = urb->status;

	switch (status) {
	case 0:
		/* success */
		return;

	case -ECONNRESET:
	case -ENOENT:
	case -ESHUTDOWN:
		/* this urb is terminated, clean up */
		dev_dbg(dev, "%s - urb shutting down with status: %d\n",
			__func__, status);
		return;

	default:
		dev_dbg(dev, "%s - nonzero urb status received: %d\n",
			__func__, status);
		goto exit;
	}

exit:
	retval = usb_submit_urb(urb, GFP_ATOMIC);
	if (retval)
		dev_err(dev, "%s - usb_submit_urb failed with result %d\n",
			__func__, retval);
}

static int xpad_init_output(struct usb_interface *intf, struct usb_xpad *xpad)
{
	struct usb_endpoint_descriptor *ep_irq_out;
	int ep_irq_out_idx;
	int error;

	if (xpad->xtype == XTYPE_UNKNOWN)
		return 0;

	xpad->odata = usb_alloc_coherent(xpad->udev, XPAD_PKT_LEN,
					 GFP_KERNEL, &xpad->odata_dma);
	if (!xpad->odata) {
		error = -ENOMEM;
		goto fail1;
	}

	mutex_init(&xpad->odata_mutex);

	xpad->irq_out = usb_alloc_urb(0, GFP_KERNEL);
	if (!xpad->irq_out) {
		error = -ENOMEM;
		goto fail2;
	}

	/* Xbox One controller has in/out endpoints swapped. */
	ep_irq_out_idx = xpad->xtype == XTYPE_XBOXONE ? 0 : 1;
	ep_irq_out = &intf->cur_altsetting->endpoint[ep_irq_out_idx].desc;

	usb_fill_int_urb(xpad->irq_out, xpad->udev,
			 usb_sndintpipe(xpad->udev, ep_irq_out->bEndpointAddress),
			 xpad->odata, XPAD_PKT_LEN,
			 xpad_irq_out, xpad, ep_irq_out->bInterval);
	xpad->irq_out->transfer_dma = xpad->odata_dma;
	xpad->irq_out->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;

	return 0;

 fail2:	usb_free_coherent(xpad->udev, XPAD_PKT_LEN, xpad->odata, xpad->odata_dma);
 fail1:	return error;
}

static void xpad_stop_output(struct usb_xpad *xpad)
{
	if (xpad->xtype != XTYPE_UNKNOWN)
		usb_kill_urb(xpad->irq_out);
}

static void xpad_deinit_output(struct usb_xpad *xpad)
{
	if (xpad->xtype != XTYPE_UNKNOWN) {
		usb_free_urb(xpad->irq_out);
		usb_free_coherent(xpad->udev, XPAD_PKT_LEN,
				xpad->odata, xpad->odata_dma);
	}
}

static int xpad_open(struct input_dev *dev)
{
	struct usb_xpad *xpad = input_get_drvdata(dev);

	/* URB was submitted in probe */
	if (xpad->xtype == XTYPE_XBOX360W)
		return 0;

	xpad->irq_in->dev = xpad->udev;
	if (usb_submit_urb(xpad->irq_in, GFP_KERNEL))
		return -EIO;

	if (xpad->xtype == XTYPE_XBOXONE) {
		/* Xbox one controller needs to be initialized. */
		xpad->odata[0] = 0x05;
		xpad->odata[1] = 0x20;
		xpad->irq_out->transfer_buffer_length = 2;
		return usb_submit_urb(xpad->irq_out, GFP_KERNEL);
	}

	return 0;
}

static void xpad_close(struct input_dev *dev)
{
	struct usb_xpad *xpad = input_get_drvdata(dev);

	if (xpad->xtype != XTYPE_XBOX360W)
		usb_kill_urb(xpad->irq_in);

	xpad_stop_output(xpad);
}

static void xpad_set_up_abs(struct input_dev *input_dev, signed short abs)
{
	printk("Check11.\n");
	struct usb_xpad *xpad = input_get_drvdata(input_dev);
	set_bit(abs, input_dev->absbit);

	switch (abs) {
	case ABS_X:
	case ABS_Y:
	case ABS_RX:
	case ABS_RY:	/* the two sticks */
		input_set_abs_params(input_dev, abs, -32768, 32767, 16, 128);
		break;
	case ABS_Z:
	case ABS_RZ:	/* the triggers (if mapped to axes) */
		if (xpad->xtype == XTYPE_XBOXONE)
			input_set_abs_params(input_dev, abs, 0, 1023, 0, 0);
		else
			input_set_abs_params(input_dev, abs, 0, 255, 0, 0);
		break;
	case ABS_HAT0X:
	case ABS_HAT0Y:	/* the d-pad (only if dpad is mapped to axes */
		input_set_abs_params(input_dev, abs, -1, 1, 0, 0);
		break;
	}
}

static int xpad_probe(struct usb_interface *intf, const struct usb_device_id *id)
{
	printk("My XPAD is Connected");

	struct usb_device *udev = interface_to_usbdev(intf);
	struct usb_xpad *xpad;
	struct input_dev *input_dev;
	struct usb_endpoint_descriptor *ep_irq_in;
	int ep_irq_in_idx;
	int i, error;

	for (i = 0; xpad_device[i].idVendor; i++) {
		if ((le16_to_cpu(udev->descriptor.idVendor) == xpad_device[i].idVendor) &&
		    (le16_to_cpu(udev->descriptor.idProduct) == xpad_device[i].idProduct))
			break;
	}

	if (xpad_device[i].xtype == XTYPE_XBOXONE &&
	    intf->cur_altsetting->desc.bInterfaceNumber != 0) {
		return -ENODEV;
	}

	xpad = kzalloc(sizeof(struct usb_xpad), GFP_KERNEL);
	input_dev = input_allocate_device();
	if (!xpad || !input_dev) {
		error = -ENOMEM;
		goto fail1;
	}

	xpad->idata = usb_alloc_coherent(udev, XPAD_PKT_LEN,
					 GFP_KERNEL, &xpad->idata_dma);
	if (!xpad->idata) {
		error = -ENOMEM;
		goto fail1;
	}

	xpad->irq_in = usb_alloc_urb(0, GFP_KERNEL);
	if (!xpad->irq_in) {
		error = -ENOMEM;
		goto fail2;
	}

	xpad->udev = udev;
	xpad->intf = intf;
	xpad->mapping = xpad_device[i].mapping;
	xpad->xtype = xpad_device[i].xtype;

	if (xpad->xtype == XTYPE_UNKNOWN) {
		if (intf->cur_altsetting->desc.bInterfaceClass == USB_CLASS_VENDOR_SPEC) {
			if (intf->cur_altsetting->desc.bInterfaceProtocol == 129)
				xpad->xtype = XTYPE_XBOX360W;
			else
				xpad->xtype = XTYPE_XBOX360;
		} else
			xpad->xtype = XTYPE_XBOX;

		if (dpad_to_buttons)
			xpad->mapping |= MAP_DPAD_TO_BUTTONS;
		if (triggers_to_buttons)
			xpad->mapping |= MAP_TRIGGERS_TO_BUTTONS;
		if (sticks_to_null)
			xpad->mapping |= MAP_STICKS_TO_NULL;
	}

	xpad->dev = input_dev;
	usb_make_path(udev, xpad->phys, sizeof(xpad->phys));
	strlcat(xpad->phys, "/input0", sizeof(xpad->phys));

	input_dev->name = xpad_device[i].name;
	input_dev->phys = xpad->phys;
	usb_to_input_id(udev, &input_dev->id);
	input_dev->dev.parent = &intf->dev;

	input_set_drvdata(input_dev, xpad);

	input_dev->open = xpad_open;
	input_dev->close = xpad_close;

	input_dev->evbit[0] = BIT_MASK(EV_KEY);

	if (!(xpad->mapping & MAP_STICKS_TO_NULL)) {
		input_dev->evbit[0] |= BIT_MASK(EV_ABS);
		/* set up axes */
		for (i = 0; xpad_abs[i] >= 0; i++)
			xpad_set_up_abs(input_dev, xpad_abs[i]);
	}

	/* set up standard buttons */
	for (i = 0; xpad_common_btn[i] >= 0; i++)
		__set_bit(xpad_common_btn[i], input_dev->keybit);

	/* set up model-specific ones */
	if (xpad->xtype == XTYPE_XBOX360 || xpad->xtype == XTYPE_XBOX360W ||
	    xpad->xtype == XTYPE_XBOXONE) {
		for (i = 0; xpad360_btn[i] >= 0; i++)
			__set_bit(xpad360_btn[i], input_dev->keybit);
	} else {
		for (i = 0; xpad_btn[i] >= 0; i++)
			__set_bit(xpad_btn[i], input_dev->keybit);
	}

	if (xpad->mapping & MAP_DPAD_TO_BUTTONS) {
		for (i = 0; xpad_btn_pad[i] >= 0; i++)
			__set_bit(xpad_btn_pad[i], input_dev->keybit);
	} else {
		for (i = 0; xpad_abs_pad[i] >= 0; i++)
			xpad_set_up_abs(input_dev, xpad_abs_pad[i]);
	}

	if (xpad->mapping & MAP_TRIGGERS_TO_BUTTONS) {
		for (i = 0; xpad_btn_triggers[i] >= 0; i++)
			__set_bit(xpad_btn_triggers[i], input_dev->keybit);
	} else {
		for (i = 0; xpad_abs_triggers[i] >= 0; i++)
			xpad_set_up_abs(input_dev, xpad_abs_triggers[i]);
	}

	for (i = 0; gamepad_buttons[i] >= 0; i++)
		input_set_capability(input_dev, EV_KEY, gamepad_buttons[i]);

	for (i = 0; directional_buttons[i] >= 0; i++)
		input_set_capability(input_dev, EV_KEY, directional_buttons[i]);

	for (i = 0; gamepad_abs[i] >= 0; i++)
		input_set_capability(input_dev, EV_REL, gamepad_abs[i]);

	error = xpad_init_output(intf, xpad);
	if (error)
		goto fail3;

	/* Xbox One controller has in/out endpoints swapped. */
	ep_irq_in_idx = xpad->xtype == XTYPE_XBOXONE ? 1 : 0;
	ep_irq_in = &intf->cur_altsetting->endpoint[ep_irq_in_idx].desc;

	usb_fill_int_urb(xpad->irq_in, udev,
			 usb_rcvintpipe(udev, ep_irq_in->bEndpointAddress),
			 xpad->idata, XPAD_PKT_LEN, xpad_irq_in,
			 xpad, ep_irq_in->bInterval);
	xpad->irq_in->transfer_dma = xpad->idata_dma;
	xpad->irq_in->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;

	error = input_register_device(xpad->dev);
	if (error)
		goto fail5;

	usb_set_intfdata(intf, xpad);

	return 0;

 fail9:	kfree(xpad->bdata);
 fail8:	usb_free_urb(xpad->bulk_out);
 fail7:	input_unregister_device(input_dev);
	input_dev = NULL;
 fail5:	if (input_dev)
		input_ff_destroy(input_dev);
 fail4:	xpad_deinit_output(xpad);
 fail3:	usb_free_urb(xpad->irq_in);
 fail2:	usb_free_coherent(udev, XPAD_PKT_LEN, xpad->idata, xpad->idata_dma);
 fail1:	input_free_device(input_dev);
	kfree(xpad);

	return error;

}

static void xpad_disconnect(struct usb_interface *intf)
{
	struct usb_xpad *xpad = usb_get_intfdata (intf);

	input_unregister_device(xpad->dev);
	xpad_deinit_output(xpad);

	if (xpad->xtype == XTYPE_XBOX360W) {
		usb_kill_urb(xpad->bulk_out);
		usb_free_urb(xpad->bulk_out);
		usb_kill_urb(xpad->irq_in);
	}

	usb_free_urb(xpad->irq_in);
	usb_free_coherent(xpad->udev, XPAD_PKT_LEN,
			xpad->idata, xpad->idata_dma);

	kfree(xpad->bdata);
	kfree(xpad);

	usb_set_intfdata(intf, NULL);
}

static struct usb_driver xpad_driver = {
	.name		= "myxpad",
	.probe		= xpad_probe,
	.disconnect	= xpad_disconnect,
	.id_table	= xpad_table,
	.supports_autosuspend = 1,
};

module_usb_driver(xpad_driver);

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");
