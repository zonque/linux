/*
 * rotary_encoder.c
 *
 * (c) 2009 Daniel Mack <daniel@caiaq.de>
 * Copyright (C) 2011 Johan Hovold <jhovold@gmail.com>
 *
 * state machine code inspired by code from Tim Ruetz
 *
 * A generic driver for rotary encoders connected to GPIO lines.
 * See file:Documentation/input/rotary-encoder.txt for more information
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/rotary_encoder.h>
#include <linux/slab.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>

#define DRV_NAME "rotary-encoder"

struct rotary_encoder {
	struct input_dev *input;
	struct rotary_encoder_platform_data pdata;

	unsigned int axis;
	unsigned int pos;

	unsigned int irq_a;
	unsigned int irq_b;

	bool armed;
	unsigned char dir;	/* 0 - clockwise, 1 - CCW */

	char last_stable;
};

static int rotary_encoder_get_state(struct rotary_encoder_platform_data *pdata)
{
	int a = !!gpio_get_value(pdata->gpio_a);
	int b = !!gpio_get_value(pdata->gpio_b);

	a ^= pdata->inverted_a;
	b ^= pdata->inverted_b;

	return ((a << 1) | b);
}

static void rotary_encoder_report_event(struct rotary_encoder *encoder)
{
	struct rotary_encoder_platform_data *pdata = &encoder->pdata;

	if (pdata->relative_axis) {
		input_report_rel(encoder->input,
				 pdata->axis, encoder->dir ? -1 : 1);
	} else {
		unsigned int pos = encoder->pos;

		if (encoder->dir) {
			/* turning counter-clockwise */
			if (pdata->rollover)
				pos += pdata->steps;
			if (pos)
				pos--;
		} else {
			/* turning clockwise */
			if (pdata->rollover || pos < pdata->steps)
				pos++;
		}

		if (pdata->rollover)
			pos %= pdata->steps;

		encoder->pos = pos;
		input_report_abs(encoder->input, pdata->axis, encoder->pos);
	}

	input_sync(encoder->input);
}

static irqreturn_t rotary_encoder_irq(int irq, void *dev_id)
{
	struct rotary_encoder *encoder = dev_id;
	int state;

	state = rotary_encoder_get_state(&encoder->pdata);

	switch (state) {
	case 0x0:
		if (encoder->armed) {
			rotary_encoder_report_event(encoder);
			encoder->armed = false;
		}
		break;

	case 0x1:
	case 0x2:
		if (encoder->armed)
			encoder->dir = state - 1;
		break;

	case 0x3:
		encoder->armed = true;
		break;
	}

	return IRQ_HANDLED;
}

static irqreturn_t rotary_encoder_half_period_irq(int irq, void *dev_id)
{
	struct rotary_encoder *encoder = dev_id;
	int state;

	state = rotary_encoder_get_state(&encoder->pdata);

	switch (state) {
	case 0x00:
	case 0x03:
		if (state != encoder->last_stable) {
			rotary_encoder_report_event(encoder);
			encoder->last_stable = state;
		}
		break;

	case 0x01:
	case 0x02:
		encoder->dir = (encoder->last_stable + state) & 0x01;
		break;
	}

	return IRQ_HANDLED;
}

#ifdef CONFIG_OF
static struct of_device_id rotary_encoder_of_match[] = {
	{ .compatible = "rotary-encoder", },
	{ },
};
MODULE_DEVICE_TABLE(of, rotary_encoder_of_match);

static int rotary_encoder_probe_dt(struct platform_device *pdev,
				   struct rotary_encoder *encoder)
{
	int tmp;
	enum of_gpio_flags flags;
	struct rotary_encoder_platform_data *pdata = &encoder->pdata;
	struct device_node *np = pdev->dev.of_node;
	const struct of_device_id *of_id =
		of_match_device(rotary_encoder_of_match, &pdev->dev);

	if (!of_id)
		return 0;

	if (of_property_read_u32(np, "rotary-encoder,steps", &tmp) == 0)
		pdata->steps = tmp;
	if (of_property_read_u32(np, "linux,axis", &tmp) == 0)
		pdata->axis = tmp;

	pdata->gpio_a = of_get_gpio_flags(np, 0, &flags);
	pdata->inverted_a = flags & OF_GPIO_ACTIVE_LOW;

	pdata->gpio_b = of_get_gpio_flags(np, 1, &flags);
	pdata->inverted_b = flags & OF_GPIO_ACTIVE_LOW;

	if (of_get_property(np, "rotary-encoder,relative-axis", NULL))
		pdata->relative_axis = 1;
	if (of_get_property(np, "rotary-encoder,rollover", NULL))
		pdata->rollover = 1;
	if (of_get_property(np, "rotary-encoder,half-period", NULL))
		pdata->half_period = 1;

	return 1;
}
#else
static inline int rotary_encoder_probe_dt(struct platform_device *)
{
	return 0;
}
#endif

static int __devinit rotary_encoder_probe(struct platform_device *pdev)
{
	struct rotary_encoder_platform_data *pdata;
	struct rotary_encoder *encoder;
	struct input_dev *input;
	struct device *dev = &pdev->dev;
	irq_handler_t handler;
	bool use_of = 0;
	int err;

	encoder = kzalloc(sizeof(struct rotary_encoder), GFP_KERNEL);
	if (!encoder)
		return -ENOMEM;

	err = rotary_encoder_probe_dt(pdev, encoder);
	if (err < 0)
		return err;
	if (err > 0)
		use_of = 1;

	if (!&pdev->dev.platform_data && !use_of) {
		dev_err(&pdev->dev, "missing platform data\n");
		return -ENOENT;
	}

	pdata = &encoder->pdata;

	/* if kernel data was provided, copy it over to our local copy */
	if (pdev->dev.platform_data)
		memcpy(pdata, &pdev->dev.platform_data, sizeof(*pdata));

	/* create and register the input driver */
	input = input_allocate_device();
	if (!input) {
		dev_err(&pdev->dev, "failed to allocate input device\n");
		err = -ENOMEM;
		goto exit_free_encoder;
	}

	input->name = pdev->name;
	input->id.bustype = BUS_HOST;
	input->dev.parent = dev;
	encoder->input = input;

	if (pdata->relative_axis) {
		input->evbit[0] = BIT_MASK(EV_REL);
		input->relbit[0] = BIT_MASK(pdata->axis);
	} else {
		input->evbit[0] = BIT_MASK(EV_ABS);
		input_set_abs_params(encoder->input,
				     pdata->axis, 0, pdata->steps, 0, 1);
	}

	err = input_register_device(input);
	if (err) {
		dev_err(dev, "failed to register input device\n");
		goto exit_free_input;
	}

	/* request the GPIOs */
	err = gpio_request_one(pdata->gpio_a, GPIOF_IN, dev_name(dev));
	if (err) {
		dev_err(dev, "unable to request GPIO %d\n", pdata->gpio_a);
		goto exit_unregister_input;
	}

	err = gpio_request_one(pdata->gpio_b, GPIOF_IN, dev_name(dev));
	if (err) {
		dev_err(dev, "unable to request GPIO %d\n", pdata->gpio_b);
		goto exit_free_gpio_a;
	}

	encoder->irq_a = gpio_to_irq(pdata->gpio_a);
	encoder->irq_b = gpio_to_irq(pdata->gpio_b);

	/* request the IRQs */
	if (pdata->half_period) {
		handler = &rotary_encoder_half_period_irq;
		encoder->last_stable = rotary_encoder_get_state(pdata);
	} else {
		handler = &rotary_encoder_irq;
	}

	err = request_irq(encoder->irq_a, handler,
			  IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
			  DRV_NAME, encoder);
	if (err) {
		dev_err(dev, "unable to request IRQ %d\n", encoder->irq_a);
		goto exit_free_gpio_b;
	}

	err = request_irq(encoder->irq_b, handler,
			  IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
			  DRV_NAME, encoder);
	if (err) {
		dev_err(dev, "unable to request IRQ %d\n", encoder->irq_b);
		goto exit_free_irq_a;
	}

	platform_set_drvdata(pdev, encoder);

	return 0;

exit_free_irq_a:
	free_irq(encoder->irq_a, encoder);
exit_free_gpio_b:
	gpio_free(pdata->gpio_b);
exit_free_gpio_a:
	gpio_free(pdata->gpio_a);
exit_unregister_input:
	input_unregister_device(input);
	input = NULL; /* so we don't try to free it */
exit_free_input:
	input_free_device(input);
exit_free_encoder:
	kfree(encoder);
	return err;
}

static int __devexit rotary_encoder_remove(struct platform_device *pdev)
{
	struct rotary_encoder *encoder = platform_get_drvdata(pdev);
	struct rotary_encoder_platform_data *pdata = &encoder->pdata;

	free_irq(encoder->irq_a, encoder);
	free_irq(encoder->irq_b, encoder);
	gpio_free(pdata->gpio_a);
	gpio_free(pdata->gpio_b);
	input_unregister_device(encoder->input);
	platform_set_drvdata(pdev, NULL);
	kfree(encoder);

	return 0;
}

static struct platform_driver rotary_encoder_driver = {
	.probe		= rotary_encoder_probe,
	.remove		= __devexit_p(rotary_encoder_remove),
	.driver		= {
		.name	= DRV_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(rotary_encoder_of_match),
	}
};
module_platform_driver(rotary_encoder_driver);

MODULE_ALIAS("platform:" DRV_NAME);
MODULE_DESCRIPTION("GPIO rotary encoder driver");
MODULE_AUTHOR("Daniel Mack <daniel@caiaq.de>, Johan Hovold");
MODULE_LICENSE("GPL v2");
