/*
 * Common power driver for PDAs and phones with one or two external
 * power supplies (AC/USB) connected to main and backup batteries,
 * and optional builtin charger.
 *
 * Copyright © 2007 Anton Vorontsov <cbou@mail.ru>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/notifier.h>
#include <linux/power_supply.h>
#include <linux/pda_power.h>
#include <linux/regulator/consumer.h>
#include <linux/timer.h>
#include <linux/jiffies.h>
#include <linux/usb/otg.h>

struct pda_power {
	struct device *dev;
	struct pda_power_pdata *pdata;
	int ac_irq, usb_irq;
	struct delayed_work polling_work;
	struct delayed_work supply_work;
	struct power_supply *pda_psy_ac, *pda_psy_usb;
	struct regulator *ac_draw;
	bool regulator_enabled;
	bool polling;
	int new_ac_status;
	int new_usb_status;
	int ac_status;
	int usb_status;

	unsigned int wait_for_status; /* msecs, default is 500 */
	unsigned int wait_for_charger; /* msecs, default is 500 */
	unsigned int polling_interval; /* msecs, default is 2000 */
	unsigned long ac_max_uA; /* current to draw when on AC */

#if IS_ENABLED(CONFIG_USB_PHY)
	struct usb_phy *transceiver;
	struct notifier_block otg_nb;
#endif
};

enum {
	PDA_PSY_OFFLINE = 0,
	PDA_PSY_ONLINE = 1,
	PDA_PSY_TO_CHANGE,
};

static bool pda_power_is_ac_online(struct pda_power *pp)
{
	if (pp->pdata->is_ac_online)
		return pp->pdata->is_ac_online();

#if IS_ENABLED(CONFIG_USB_PHY)
	if (pp->transceiver)
		return pp->transceiver->last_event == USB_EVENT_CHARGER;
#endif

	return false;
}

static bool pda_power_is_usb_online(struct pda_power *pp)
{
	if (pp->pdata->is_usb_online)
		return pp->pdata->is_usb_online();

#if IS_ENABLED(CONFIG_USB_PHY)
	if (pp->transceiver)
		return pp->transceiver->last_event == USB_EVENT_VBUS ||
		       pp->transceiver->last_event == USB_EVENT_ENUMERATED;
#endif

	return false;
}

static int pda_power_get_property(struct power_supply *psy,
				  enum power_supply_property psp,
				  union power_supply_propval *val)
{
	struct pda_power *pp = psy->drv_data;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		if (psy->desc->type == POWER_SUPPLY_TYPE_MAINS)
			val->intval = pda_power_is_ac_online(pp);
		else
			val->intval = pda_power_is_usb_online(pp);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static enum power_supply_property pda_power_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static char *pda_power_supplied_to[] = {
	"main-battery",
	"backup-battery",
};

static const struct power_supply_desc pda_psy_ac_desc = {
	.name = "ac",
	.type = POWER_SUPPLY_TYPE_MAINS,
	.properties = pda_power_props,
	.num_properties = ARRAY_SIZE(pda_power_props),
	.get_property = pda_power_get_property,
};

static const struct power_supply_desc pda_psy_usb_desc = {
	.name = "usb",
	.type = POWER_SUPPLY_TYPE_USB,
	.properties = pda_power_props,
	.num_properties = ARRAY_SIZE(pda_power_props),
	.get_property = pda_power_get_property,
};

static void update_status(struct pda_power *pp)
{
	pp->new_ac_status = pda_power_is_ac_online(pp);
	pp->new_usb_status = pda_power_is_usb_online(pp);
}

static void update_charger(struct pda_power *pp)
{
	int max_uA = pp->ac_max_uA;

	if (pp->pdata->set_charge) {
		if (pp->new_ac_status > 0) {
			dev_dbg(pp->dev, "charger on (AC)\n");
			pp->pdata->set_charge(PDA_POWER_CHARGE_AC);
		} else if (pp->new_usb_status > 0) {
			dev_dbg(pp->dev, "charger on (USB)\n");
			pp->pdata->set_charge(PDA_POWER_CHARGE_USB);
		} else {
			dev_dbg(pp->dev, "charger off\n");
			pp->pdata->set_charge(0);
		}
	} else if (pp->ac_draw) {
		if (pp->new_ac_status > 0) {
			regulator_set_current_limit(pp->ac_draw,
						    max_uA, max_uA);
			if (!pp->regulator_enabled) {
				dev_dbg(pp->dev, "charger on (AC)\n");
				WARN_ON(regulator_enable(pp->ac_draw));
				pp->regulator_enabled = true;
			}
		} else {
			if (pp->regulator_enabled) {
				dev_dbg(pp->dev, "charger off\n");
				WARN_ON(regulator_disable(pp->ac_draw));
				pp->regulator_enabled = false;
			}
		}
	}
}

static void supply_work_func(struct work_struct *work)
{
	struct pda_power *pp =
		container_of(work, struct pda_power, supply_work.work);

	if (pp->ac_status == PDA_PSY_TO_CHANGE) {
		pp->ac_status = pp->new_ac_status;
		power_supply_changed(pp->pda_psy_ac);
	}

	if (pp->usb_status == PDA_PSY_TO_CHANGE) {
		pp->usb_status = pp->new_usb_status;
		power_supply_changed(pp->pda_psy_usb);
	}
}

static void psy_changed(struct pda_power *pp)
{
	update_charger(pp);

	/*
	 * Okay, charger set. Now wait a bit before notifying supplicants,
	 * charge power should stabilize.
	 */
	cancel_delayed_work(&pp->supply_work);
	schedule_delayed_work(&pp->supply_work,
			      msecs_to_jiffies(pp->wait_for_charger));
}

static irqreturn_t ac_power_changed_tread_fn(int irq, void *context)
{
	struct pda_power *pp = context;

	usleep_range(pp->wait_for_status * 1000,
		     pp->wait_for_status * 2000);

	pp->ac_status = PDA_PSY_TO_CHANGE;
	update_status(pp);
	psy_changed(pp);

	return IRQ_HANDLED;
}

static irqreturn_t usb_power_changed_tread_fn(int irq, void *context)
{
	struct pda_power *pp = context;

	usleep_range(pp->wait_for_status * 1000,
		     pp->wait_for_status * 2000);

	pp->usb_status = PDA_PSY_TO_CHANGE;
	update_status(pp);
	psy_changed(pp);

	return IRQ_HANDLED;
}

static void polling_work_func(struct work_struct *work)
{
	bool changed = false;
	struct pda_power *pp =
		container_of(work, struct pda_power, polling_work.work);

	dev_dbg(pp->dev, "polling...\n");

	update_status(pp);

	if (pp->ac_irq < 0 && pp->new_ac_status != pp->ac_status) {
		pp->ac_status = PDA_PSY_TO_CHANGE;
		changed = true;
	}

	if (pp->usb_irq < 0 && pp->new_usb_status != pp->usb_status) {
		pp->usb_status = PDA_PSY_TO_CHANGE;
		changed = true;
	}

	if (changed)
		psy_changed(pp);

	schedule_delayed_work(&pp->polling_work,
			      msecs_to_jiffies(pp->polling_interval));
}

#if IS_ENABLED(CONFIG_USB_PHY)
static int otg_handle_notification(struct notifier_block *nb,
		unsigned long event, void *context)
{
	struct pda_power *pp = context;

	switch (event) {
	case USB_EVENT_CHARGER:
		pp->ac_status = PDA_PSY_TO_CHANGE;
		break;
	case USB_EVENT_VBUS:
	case USB_EVENT_ENUMERATED:
		pp->usb_status = PDA_PSY_TO_CHANGE;
		break;
	case USB_EVENT_NONE:
		pp->ac_status = PDA_PSY_TO_CHANGE;
		pp->usb_status = PDA_PSY_TO_CHANGE;
		break;
	default:
		return NOTIFY_OK;
	}

	update_status(pp);
	psy_changed(pp);

	return NOTIFY_OK;
}
#endif

static int pda_power_probe(struct platform_device *pdev)
{
	unsigned int ac_irq_flags, usb_irq_flags;
	struct power_supply_config psy_cfg = {};
	struct pda_power_pdata *pdata;
	struct resource *res;
	struct pda_power *pp;
	struct device *dev;
	int ret = 0;

	dev = &pdev->dev;

	if (pdev->id != -1) {
		dev_err(dev, "it's meaningless to register several "
			"pda_powers; use id = -1\n");
		return -EINVAL;
	}

	pp = devm_kzalloc(dev, sizeof(*pp), GFP_KERNEL);
	if (!pp)
		return -ENOMEM;

	pdata = pdev->dev.platform_data;

	if (pdata->init) {
		ret = pdata->init(dev);
		if (ret < 0)
			return ret;
	}

	pp->ac_draw = devm_regulator_get(dev, "ac_draw");
	if (IS_ERR(pp->ac_draw)) {
		dev_dbg(dev, "couldn't get ac_draw regulator\n");
		pp->ac_draw = NULL;
	}

	pp->dev = dev;
	pp->pdata = pdata;
	pp->new_ac_status = -1;
	pp->new_usb_status = -1;
	pp->ac_status = -1;
	pp->usb_status = -1;
	pp->wait_for_status = pdata->wait_for_status;
	pp->wait_for_charger = pdata->wait_for_charger;
	pp->polling_interval = pdata->polling_interval;
	pp->ac_max_uA = pdata->ac_max_uA;

	platform_set_drvdata(pdev, pp);

	update_status(pp);
	update_charger(pp);

	if (!pp->wait_for_status)
		pp->wait_for_status = 500;

	if (!pp->wait_for_charger)
		pp->wait_for_charger = 500;

	if (!pp->polling_interval)
		pp->polling_interval = 2000;

	if (!pp->ac_max_uA)
		pp->ac_max_uA = 500000;

	INIT_DELAYED_WORK(&pp->supply_work, supply_work_func);
	INIT_DELAYED_WORK(&pp->polling_work, polling_work_func);

	ac_irq_flags = IRQF_SHARED | IRQF_ONESHOT;
	usb_irq_flags = IRQF_SHARED | IRQF_ONESHOT;

	res = platform_get_resource_byname(pdev, IORESOURCE_IRQ, "ac");
	if (res) {
		pp->ac_irq = res->start;
		ac_irq_flags |= res->flags & IRQF_TRIGGER_MASK;
	} else {
		pp->ac_irq = -1;
		pp->polling = true;
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_IRQ, "usb");
	if (res) {
		pp->usb_irq = res->start;
		usb_irq_flags |= res->flags & IRQF_TRIGGER_MASK;
	} else {
		pp->usb_irq = -1;
		pp->polling = true;
	}

	psy_cfg.drv_data = pp;

	if (pdata->supplied_to) {
		psy_cfg.supplied_to = pdata->supplied_to;
		psy_cfg.num_supplicants = pdata->num_supplicants;
	} else {
		psy_cfg.supplied_to = pda_power_supplied_to;
		psy_cfg.num_supplicants = ARRAY_SIZE(pda_power_supplied_to);
	}

#if IS_ENABLED(CONFIG_USB_PHY)
	pp->transceiver = devm_usb_get_phy(dev, USB_PHY_TYPE_USB2);
#endif

	if (pdata->is_ac_online) {
		if (pp->ac_irq >= 0) {
			ret = devm_request_threaded_irq(dev, pp->ac_irq,
							NULL,
							ac_power_changed_tread_fn,
							ac_irq_flags, "ac", pp);
			if (ret < 0) {
				dev_err(dev, "Unable to request AC irq: %d\n", ret);
				goto error_out;
			}
		}

		pp->pda_psy_ac =
			devm_power_supply_register(dev, &pda_psy_ac_desc,
						   &psy_cfg);
		if (IS_ERR(pp->pda_psy_ac)) {
			dev_err(dev, "failed to register %s power supply\n",
				pda_psy_ac_desc.name);
			ret = PTR_ERR(pp->pda_psy_ac);
			goto error_out;
		}
	}

	if (pdata->is_usb_online) {
		if (pp->usb_irq >= 0) {
			ret = devm_request_threaded_irq(dev, pp->usb_irq,
							NULL,
							usb_power_changed_tread_fn,
							usb_irq_flags, "usb", pp);
			if (ret) {
				dev_err(dev, "Unable to request USB irq: %d\n", ret);
				goto error_out;
			}
		}

		pp->pda_psy_usb =
			devm_power_supply_register(dev, &pda_psy_usb_desc,
						   &psy_cfg);
		if (IS_ERR(pp->pda_psy_usb)) {
			dev_err(dev, "failed to register %s power supply\n",
				pda_psy_usb_desc.name);
			ret = PTR_ERR(pp->pda_psy_usb);
			goto error_out;
		}
	}

#if IS_ENABLED(CONFIG_USB_PHY)
	if (!IS_ERR_OR_NULL(pp->transceiver)) {
		pp->otg_nb.notifier_call = otg_handle_notification;
		ret = usb_register_notifier(pp->transceiver, &pp->otg_nb);
		if (ret) {
			dev_err(dev, "failure to register otg notifier\n");
			goto error_out;
		}
		pp->polling = 0;
	} else {
		pp->transceiver = NULL;
	}
#endif

	if (pp->polling) {
		dev_dbg(dev, "will poll for status\n");
		schedule_delayed_work(&pp->polling_work,
				      msecs_to_jiffies(pp->polling_interval));
	}

	if (pp->ac_irq >= 0) {
		enable_irq_wake(pp->ac_irq);
		device_init_wakeup(dev, 1);
	}

	if (pp->usb_irq >= 0)
		enable_irq_wake(pp->usb_irq);
		device_init_wakeup(dev, 1);
	}

	return 0;

error_out:
	if (pdata->exit)
		pdata->exit(dev);

	return ret;
}

static int pda_power_remove(struct platform_device *pdev)
{
	struct pda_power *pp = platform_get_drvdata(pdev);

	cancel_delayed_work_sync(&pp->polling_work);
	cancel_delayed_work_sync(&pp->supply_work);

	if (pp->pdata->exit)
		pp->pdata->exit(&pdev->dev);

	return 0;
}

#ifdef CONFIG_PM
static int pda_power_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct pda_power *pp = platform_get_drvdata(pdev);

	if (pp->pdata->suspend) {
		int ret = pp->pdata->suspend(state);

		if (ret)
			return ret;
	}

	return 0;
}

static int pda_power_resume(struct platform_device *pdev)
{
	struct pda_power *pp = platform_get_drvdata(pdev);

	if (pp->pdata->resume)
		return pp->pdata->resume();

	return 0;
}
#else
#define pda_power_suspend NULL
#define pda_power_resume NULL
#endif /* CONFIG_PM */

static struct platform_driver pda_power_pdrv = {
	.driver = {
		.name = "pda-power",
	},
	.probe = pda_power_probe,
	.remove = pda_power_remove,
	.suspend = pda_power_suspend,
	.resume = pda_power_resume,
};

module_platform_driver(pda_power_pdrv);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Anton Vorontsov <cbou@mail.ru>");
MODULE_ALIAS("platform:pda-power");
