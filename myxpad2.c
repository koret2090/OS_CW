#include <linux/slab.h>
#include <linux/module.h>
#include <linux/usb/input.h>
#include <linux/input/mt.h>
#include <linux/random.h>

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
#define DRIVER_DESC "My Gamepad Driver"

#define XPAD_PKT_LEN 64
//#define EV_MAX 0x1f
/* xbox d-pads should map to buttons, as is required for DDR pads
   but we map them to axes when possible to simplify things */
#define MAP_DPAD_TO_BUTTONS		(1 << 0)
#define MAP_TRIGGERS_TO_BUTTONS		(1 << 1)
#define MAP_STICKS_TO_NULL		(1 << 2)
#define DANCEPAD_MAP_CONFIG	(MAP_DPAD_TO_BUTTONS |			\
				MAP_TRIGGERS_TO_BUTTONS | MAP_STICKS_TO_NULL)

#define XTYPE_XBOX        0
#define XTYPE_XBOX360     1
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

	unsigned char *bdata;

	struct urb *irq_out;		/* urb for interrupt out report */
	unsigned char *odata;		/* output data */
	dma_addr_t odata_dma;
	struct mutex odata_mutex;

	char phys[64];			/* physical device path */

	int mapping;			/* map d-pad to buttons or to axes */
	int xtype;			/* type of xbox device */
};

static inline int is_event_supported(unsigned int code,
				     unsigned long *bm, unsigned int max)
{
	return code <= max && test_bit(code, bm);
}

static void input_stop_autorepeat(struct input_dev *dev)
{
	del_timer(&dev->timer);
}

static void input_start_autorepeat(struct input_dev *dev, int code)
{
	if (test_bit(EV_REP, dev->evbit) &&
	    dev->rep[REP_PERIOD] && dev->rep[REP_DELAY] &&
	    dev->timer.function) {
		dev->repeat_key = code;
		mod_timer(&dev->timer,
			  jiffies + msecs_to_jiffies(dev->rep[REP_DELAY]));
	}
}
static const struct input_value input_value_sync = { EV_SYN, SYN_REPORT, 1 };

/*
 * Pass event first through all filters and then, if event has not been
 * filtered out, through all open handles. This function is called with
 * dev->event_lock held and interrupts disabled.
 */
static unsigned int input_to_handler(struct input_handle *handle,
			struct input_value *vals, unsigned int count)
{
	struct input_handler *handler = handle->handler;
	struct input_value *end = vals;
	struct input_value *v;

	if (handler->filter) {
		for (v = vals; v != vals + count; v++) {
			if (handler->filter(handle, v->type, v->code, v->value))
				continue;
			if (end != v)
				*end = *v;
			end++;
		}
		count = end - vals;
	}

	if (!count)
		return 0;

	if (handler->events)
		handler->events(handle, vals, count);
	else if (handler->event)
		for (v = vals; v != vals + count; v++)
			handler->event(handle, v->type, v->code, v->value);

	return count;
}

static void input_pass_values(struct input_dev *dev,
			      struct input_value *vals, unsigned int count)
{
	struct input_handle *handle;
	struct input_value *v;

	if (!count)
		return;

	rcu_read_lock();

	handle = rcu_dereference(dev->grab);
	if (handle) {
		count = input_to_handler(handle, vals, count);
	} else {
		list_for_each_entry_rcu(handle, &dev->h_list, d_node)
			if (handle->open) {
				count = input_to_handler(handle, vals, count);
				if (!count)
					break;
			}
	}

	rcu_read_unlock();

	/* trigger auto repeat for key events */
	if (test_bit(EV_REP, dev->evbit) && test_bit(EV_KEY, dev->evbit)) {
		for (v = vals; v != vals + count; v++) {
			if (v->type == EV_KEY && v->value != 2) {
				if (v->value)
					input_start_autorepeat(dev, v->code);
				else
					input_stop_autorepeat(dev);
			}
		}
	}
}

static int input_defuzz_abs_event(int value, int old_val, int fuzz)
{
	if (fuzz) {
		if (value > old_val - fuzz / 2 && value < old_val + fuzz / 2)
			return old_val;

		if (value > old_val - fuzz && value < old_val + fuzz)
			return (old_val * 3 + value) / 4;

		if (value > old_val - fuzz * 2 && value < old_val + fuzz * 2)
			return (old_val + value) / 2;
	}

	return value;
}

#define INPUT_IGNORE_EVENT	0
#define INPUT_PASS_TO_HANDLERS	1
#define INPUT_PASS_TO_DEVICE	2
#define INPUT_SLOT		4
#define INPUT_FLUSH		8
#define INPUT_PASS_TO_ALL	(INPUT_PASS_TO_HANDLERS | INPUT_PASS_TO_DEVICE)

static int input_handle_abs_event(struct input_dev *dev,
				  unsigned int code, int *pval)
{
	struct input_mt *mt = dev->mt;
	bool is_mt_event;
	int *pold;

	if (code == ABS_MT_SLOT) {
		/*
		 * "Stage" the event; we'll flush it later, when we
		 * get actual touch data.
		 */
		if (mt && *pval >= 0 && *pval < mt->num_slots)
			mt->slot = *pval;

		return INPUT_IGNORE_EVENT;
	}

	is_mt_event = input_is_mt_value(code);

	if (!is_mt_event) {
		pold = &dev->absinfo[code].value;
	} else if (mt) {
		pold = &mt->slots[mt->slot].abs[code - ABS_MT_FIRST];
	} else {
		/*
		 * Bypass filtering for multi-touch events when
		 * not employing slots.
		 */
		pold = NULL;
	}

	if (pold) {
		*pval = input_defuzz_abs_event(*pval, *pold,
						dev->absinfo[code].fuzz);
		if (*pold == *pval)
			return INPUT_IGNORE_EVENT;

		*pold = *pval;
	}

	/* Flush pending "slot" event */
	if (is_mt_event && mt && mt->slot != input_abs_get_val(dev, ABS_MT_SLOT)) {
		input_abs_set_val(dev, ABS_MT_SLOT, mt->slot);
		return INPUT_PASS_TO_HANDLERS | INPUT_SLOT;
	}

	return INPUT_PASS_TO_HANDLERS;
}

static int input_get_disposition(struct input_dev *dev,
			  unsigned int type, unsigned int code, int *pval)
{
	int disposition = INPUT_IGNORE_EVENT;
	int value = *pval;

	switch (type) {

	case EV_SYN:
		switch (code) {
		case SYN_CONFIG:
			disposition = INPUT_PASS_TO_ALL;
			break;

		case SYN_REPORT:
			disposition = INPUT_PASS_TO_HANDLERS | INPUT_FLUSH;
			break;
		case SYN_MT_REPORT:
			disposition = INPUT_PASS_TO_HANDLERS;
			break;
		}
		break;

	case EV_KEY:
		if (is_event_supported(code, dev->keybit, KEY_MAX)) {

			/* auto-repeat bypasses state updates */
			if (value == 2) {
				disposition = INPUT_PASS_TO_HANDLERS;
				break;
			}

			if (!!test_bit(code, dev->key) != !!value) {

				__change_bit(code, dev->key);
				disposition = INPUT_PASS_TO_HANDLERS;
			}
		}
		break;

	case EV_SW:
		if (is_event_supported(code, dev->swbit, SW_MAX) &&
		    !!test_bit(code, dev->sw) != !!value) {

			__change_bit(code, dev->sw);
			disposition = INPUT_PASS_TO_HANDLERS;
		}
		break;

	case EV_ABS:
		if (is_event_supported(code, dev->absbit, ABS_MAX))
			disposition = input_handle_abs_event(dev, code, &value);

		break;

	case EV_REL:
		if (is_event_supported(code, dev->relbit, REL_MAX) && value)
			disposition = INPUT_PASS_TO_HANDLERS;

		break;

	case EV_MSC:
		if (is_event_supported(code, dev->mscbit, MSC_MAX))
			disposition = INPUT_PASS_TO_ALL;

		break;

	case EV_LED:
		if (is_event_supported(code, dev->ledbit, LED_MAX) &&
		    !!test_bit(code, dev->led) != !!value) {

			__change_bit(code, dev->led);
			disposition = INPUT_PASS_TO_ALL;
		}
		break;

	case EV_SND:
		if (is_event_supported(code, dev->sndbit, SND_MAX)) {

			if (!!test_bit(code, dev->snd) != !!value)
				__change_bit(code, dev->snd);
			disposition = INPUT_PASS_TO_ALL;
		}
		break;

	case EV_REP:
		if (code <= REP_MAX && value >= 0 && dev->rep[code] != value) {
			dev->rep[code] = value;
			disposition = INPUT_PASS_TO_ALL;
		}
		break;

	case EV_FF:
		if (value >= 0)
			disposition = INPUT_PASS_TO_ALL;
		break;

	case EV_PWR:
		disposition = INPUT_PASS_TO_ALL;
		break;
	}

	*pval = value;
	return disposition;
}

static void input_handle_event(struct input_dev *dev,
			       unsigned int type, unsigned int code, int value)
{
	int disposition;

	/* filter-out events from inhibited devices */
	if (dev->inhibited)
		return;

	disposition = input_get_disposition(dev, type, code, &value);
	if (disposition != INPUT_IGNORE_EVENT && type != EV_SYN)
		add_input_randomness(type, code, value);

	if ((disposition & INPUT_PASS_TO_DEVICE) && dev->event)
		dev->event(dev, type, code, value);

	if (!dev->vals)
		return;

	if (disposition & INPUT_PASS_TO_HANDLERS) {
		struct input_value *v;

		if (disposition & INPUT_SLOT) {
			v = &dev->vals[dev->num_vals++];
			v->type = EV_ABS;
			v->code = ABS_MT_SLOT;
			v->value = dev->mt->slot;
		}

		v = &dev->vals[dev->num_vals++];
		v->type = type;
		v->code = code;
		v->value = value;
	}

	if (disposition & INPUT_FLUSH) {
		if (dev->num_vals >= 2)
			input_pass_values(dev, dev->vals, dev->num_vals);
		dev->num_vals = 0;
		/*
		 * Reset the timestamp on flush so we won't end up
		 * with a stale one. Note we only need to reset the
		 * monolithic one as we use its presence when deciding
		 * whether to generate a synthetic timestamp.
		 */
		dev->timestamp[INPUT_CLK_MONO] = ktime_set(0, 0);
	} else if (dev->num_vals >= dev->max_vals - 2) {
		dev->vals[dev->num_vals++] = input_value_sync;
		input_pass_values(dev, dev->vals, dev->num_vals);
		dev->num_vals = 0;
	}

}

void do_action(struct input_dev *dev, unsigned int type, unsigned int code, int value)
{
	unsigned long flags;
	
	if (type <= EV_MAX && test_bit(type, dev->evbit))
	{
		spin_lock_irqsave(&dev->event_lock, flags);
		input_handle_event(dev, type, code, value);
		spin_unlock_irqrestore(&dev->event_lock, flags);
	}
}

static void xpad_irq_in(struct urb *urb)
{
	printk("+ irq in");
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
		retval = usb_submit_urb(urb, GFP_KERNEL);
		if (retval)
			dev_err(&urb->dev->dev, "%s - Error %d submitting interrupt urb\n", __func__, retval);
			return;
	}

	do_action(dev, EV_KEY, KEY_LEFT, data[2] & BUTTON_LEFT);
	do_action(dev, EV_KEY, KEY_RIGHT, data[2] & BUTTON_RIGHT);
	do_action(dev, EV_KEY, KEY_UP, data[2] & BUTTON_UP);
	do_action(dev, EV_KEY, KEY_DOWN, data[2] & BUTTON_DOWN);
	
	do_action(dev, EV_KEY, BTN_LEFT, data[3] & BUTTON_A);
	do_action(dev, EV_KEY, BTN_RIGHT, data[3] & BUTTON_B);

	do_action(dev,EV_KEY, KEY_LEFTSHIFT, data[3] & BUTTON_L1);
	do_action(dev, EV_KEY, KEY_ENTER, data[3] & BUTTON_R1);

	do_action(dev,EV_KEY, KEY_ESC,  data[2] & BUTTON_START);
	do_action(dev, EV_KEY, KEY_LEFTCTRL, data[2] & BUTTON_SELECT);
	do_action(dev, EV_KEY, KEY_LEFTALT, data[3] & BUTTON_MODE);

	do_action(dev, EV_KEY, KEY_PAGEDOWN, data[2] & BUTTON_L3);
	do_action(dev, EV_KEY, KEY_PAGEUP, data[2] & BUTTON_R3);

	do_action(dev, EV_REL, REL_X, (__s16) le16_to_cpup((__le16 *)(data + 6))/2048);
	do_action(dev, EV_REL, REL_Y, ~(__s16) le16_to_cpup((__le16 *)(data + 8))/2048);

	do_action(dev, EV_REL, REL_HWHEEL, (__s16) le16_to_cpup((__le16 *)(data + 10))/8192);
	do_action(dev, EV_REL, ABS_WHEEL, ~(__s16) le16_to_cpup((__le16 *)(data + 12))/8192);

	input_sync(dev);

	retval = usb_submit_urb(urb, GFP_KERNEL);
	if (retval)
		dev_err(&urb->dev->dev, "%s - Error %d submitting interrupt urb\n", __func__, retval);
}

static void xpad_irq_out(struct urb *urb)
{
	printk("+ irq out.\n");
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
	}

	retval = usb_submit_urb(urb, GFP_ATOMIC);
	if (retval)
		dev_err(dev, "%s - usb_submit_urb failed with result %d\n",
			__func__, retval);
}

static int xpad_init_output(struct usb_interface *intf, struct usb_xpad *xpad)
{
	printk("+ init output");
	struct usb_endpoint_descriptor *ep_irq_out;
	int ep_irq_out_idx;
	int error;

	if (xpad->xtype == XTYPE_UNKNOWN)
		return 0;

	xpad->odata = usb_alloc_coherent(xpad->udev, XPAD_PKT_LEN,
					 GFP_KERNEL, &xpad->odata_dma);
	if (!xpad->odata) {
		error = -ENOMEM;
		return error;
	}

	mutex_init(&xpad->odata_mutex);

	xpad->irq_out = usb_alloc_urb(0, GFP_KERNEL);
	if (!xpad->irq_out) {
		error = -ENOMEM;
		usb_free_coherent(xpad->udev, XPAD_PKT_LEN, xpad->odata, xpad->odata_dma);
		return error;
	}

	ep_irq_out = &intf->cur_altsetting->endpoint[ep_irq_out_idx].desc;

	usb_fill_int_urb(xpad->irq_out, xpad->udev,
			 usb_sndintpipe(xpad->udev, ep_irq_out->bEndpointAddress),
			 xpad->odata, XPAD_PKT_LEN,
			 xpad_irq_out, xpad, ep_irq_out->bInterval);
	xpad->irq_out->transfer_dma = xpad->odata_dma;
	xpad->irq_out->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;

	return 0;
}

static void xpad_stop_output(struct usb_xpad *xpad)
{
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
	printk("+ Gamepad is opened");
	struct usb_xpad *xpad = input_get_drvdata(dev);

	xpad->irq_in->dev = xpad->udev;
	if (usb_submit_urb(xpad->irq_in, GFP_KERNEL))
		return -EIO;

	return 0;
}

static void xpad_close(struct input_dev *dev)
{
	printk("+ Closing");
	struct usb_xpad *xpad = input_get_drvdata(dev);
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

	xpad = kzalloc(sizeof(struct usb_xpad), GFP_KERNEL);
	input_dev = input_allocate_device();
	if (!xpad || !input_dev) {
		error = -ENOMEM;
		input_free_device(input_dev);
		kfree(xpad);
		return error;
	}

	xpad->idata = usb_alloc_coherent(udev, XPAD_PKT_LEN,
					 GFP_KERNEL, &xpad->idata_dma);
	if (!xpad->idata) {
		error = -ENOMEM;
		input_free_device(input_dev);
		kfree(xpad);
		return error;
	}

	xpad->irq_in = usb_alloc_urb(0, GFP_KERNEL);
	if (!xpad->irq_in) {
		error = -ENOMEM;
		usb_free_coherent(udev, XPAD_PKT_LEN, xpad->idata, xpad->idata_dma);
		input_free_device(input_dev);
		kfree(xpad);
		return error;
	}

	xpad->udev = udev;
	xpad->intf = intf;
	xpad->mapping = xpad_device[i].mapping;
	xpad->xtype = xpad_device[i].xtype;

	if (xpad->xtype == XTYPE_UNKNOWN) {
		if (intf->cur_altsetting->desc.bInterfaceClass == USB_CLASS_VENDOR_SPEC) {
			if (intf->cur_altsetting->desc.bInterfaceProtocol == 129)
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
	if (xpad->xtype == XTYPE_XBOX360) {
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
	if (error){
		usb_free_urb(xpad->irq_in);
		usb_free_coherent(udev, XPAD_PKT_LEN, xpad->idata, xpad->idata_dma);
		input_free_device(input_dev);
		kfree(xpad);
		return error;
	}

	ep_irq_in = &intf->cur_altsetting->endpoint[ep_irq_in_idx].desc;

	usb_fill_int_urb(xpad->irq_in, udev,
			 usb_rcvintpipe(udev, ep_irq_in->bEndpointAddress),
			 xpad->idata, XPAD_PKT_LEN, xpad_irq_in,
			 xpad, ep_irq_in->bInterval);
	xpad->irq_in->transfer_dma = xpad->idata_dma;
	xpad->irq_in->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;

	error = input_register_device(xpad->dev);
	if (error){
		if (input_dev)
			input_ff_destroy(input_dev);
		xpad_deinit_output(xpad);
		usb_free_urb(xpad->irq_in);
		usb_free_coherent(udev, XPAD_PKT_LEN, xpad->idata, xpad->idata_dma);
		input_free_device(input_dev);
		kfree(xpad);
		return error;
	}

	usb_set_intfdata(intf, xpad);

	return 0;
}

static void xpad_disconnect(struct usb_interface *intf)
{
	printk("+ Gamepad is disconnected");
	struct usb_xpad *xpad = usb_get_intfdata (intf);

	input_unregister_device(xpad->dev);
	xpad_deinit_output(xpad);									

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
};

module_usb_driver(xpad_driver);

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");
