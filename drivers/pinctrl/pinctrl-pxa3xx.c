/*
 *  linux/drivers/pinctrl/pinctrl-pxa3xx.c
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  publishhed by the Free Software Foundation.
 *
 *  Copyright (C) 2011, Marvell Technology Group Ltd.
 *
 *  Author: Haojian Zhuang <haojian.zhuang@marvell.com>
 *
 */

#include <linux/module.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/pinctrl/machine.h>
#include <linux/pinctrl/pinconf.h>
#include "pinctrl-pxa3xx.h"

static struct pinctrl_gpio_range pxa3xx_pinctrl_gpio_range = {
	.name		= "PXA3xx GPIO",
	.id		= 0,
	.base		= 0,
	.pin_base	= 0,
};

static int pxa3xx_get_groups_count(struct pinctrl_dev *pctrldev)
{
	struct pxa3xx_pinmux_info *info = pinctrl_dev_get_drvdata(pctrldev);

	return info->num_grps;
}

static const char *pxa3xx_get_group_name(struct pinctrl_dev *pctrldev,
					 unsigned selector)
{
	struct pxa3xx_pinmux_info *info = pinctrl_dev_get_drvdata(pctrldev);

	return info->grps[selector].name;
}

static int pxa3xx_get_group_pins(struct pinctrl_dev *pctrldev,
				 unsigned selector,
				 const unsigned **pins,
				 unsigned *num_pins)
{
	struct pxa3xx_pinmux_info *info = pinctrl_dev_get_drvdata(pctrldev);

	*pins = info->grps[selector].pins;
	*num_pins = info->grps[selector].npins;
	return 0;
}

static int pxa3xx_pinctrl_dt_node_to_map(struct pinctrl_dev *pctrl_dev,
					 struct device_node *np_config,
					 struct pinctrl_map **map,
					 unsigned *num_maps)
{
	struct pxa3xx_pinmux_info *info = pinctrl_dev_get_drvdata(pctrl_dev);
	struct device_node *np;
	struct property *prop;
	const char *func, *group;
	int ret, count = 0, i = 0, pin_num, size;
	u32 ds, cfg;

	/* verify subnode */
	for_each_child_of_node(np_config, np) {
		ret = of_property_read_string(np, "marvell,function", &func);
		if (ret < 0)
			return ret;
		ret = of_property_count_strings(np, "marvell,pins");
		if (ret < 0)
			return ret;
		count += ret;
		pin_num = ret;

		if (!of_property_read_u32(np, "marvell,drive-strength", &ds))
			count += pin_num;

		if (of_find_property(np, "marvell,pull-up", &size)
			|| of_find_property(np, "marvell,pull-down", &size))
			count += pin_num;

		if (of_find_property(np, "marvell,lowpower-pull-up", &size)
			|| of_find_property(np, "marvell,lowpower-pull-down",
				&size)
			|| of_find_property(np, "marvell,lowpower-drive-high",
				&size)
			|| of_find_property(np, "marvell,lowpower-drive-low",
				&size)
			|| of_find_property(np, "marvell,lowpower-float", &size)
			|| of_find_property(np, "marvell,lowpower-zero", &size))
			count += pin_num;
	}

	if (!count) {
		dev_err(info->dev, "No child nodes passed via DT\n");
		return -ENODEV;
	}

	*map = kzalloc(sizeof(**map) * count, GFP_KERNEL);
	if (!*map)
		return -ENOMEM;

	for_each_child_of_node(np_config, np) {
		of_property_read_string(np, "marvell,function", &func);
		of_property_for_each_string(np, "marvell,pins", prop, group) {
			(*map)[i].type = PIN_MAP_TYPE_MUX_GROUP;
			(*map)[i].data.mux.group = group;
			(*map)[i].data.mux.function = func;
			i++;

			cfg = 0;
			if (of_find_property(np, "marvell,pull-up", &size))
				cfg = PXA3XX_PINCONF_PULL_UP;
			else if (of_find_property(np,
				"marvell,pull-down", &size))
				cfg = PXA3XX_PINCONF_PULL_DOWN;
			if (cfg) {
				(*map)[i].type = PIN_MAP_TYPE_CONFIGS_GROUP;
				(*map)[i].data.configs.configs =
					kmemdup(&cfg, sizeof(cfg), GFP_KERNEL);
				(*map)[i].data.configs.group_or_pin = group;
				(*map)[i].data.configs.num_configs = 1;
				i++;
			}

			cfg = 0;
			if (of_find_property(np, "marvell,lowpower-pull-up",
				&size))
				cfg = PXA3XX_PINCONF_LOWPOWER_PULL_UP;
			else if (of_find_property(np,
				"marvell,lowpower-pull-down", &size))
				cfg = PXA3XX_PINCONF_LOWPOWER_PULL_DOWN;
			else if (of_find_property(np,
				"marvell,lowpower-drive-high", &size))
				cfg = PXA3XX_PINCONF_LOWPOWER_DRIVE_HIGH;
			else if (of_find_property(np,
				"marvell,lowpower-drive-low", &size))
				cfg = PXA3XX_PINCONF_LOWPOWER_DRIVE_LOW;
			else if (of_find_property(np,
				"marvell,lowpower-float", &size))
				cfg = PXA3XX_PINCONF_LOWPOWER_FLOAT;
			else if (of_find_property(np,
				"marvell,lowpower-zero", &size))
				cfg = PXA3XX_PINCONF_LOWPOWER_ZERO;
			if (cfg) {
				(*map)[i].type = PIN_MAP_TYPE_CONFIGS_GROUP;
				(*map)[i].data.configs.configs =
					kmemdup(&cfg, sizeof(cfg), GFP_KERNEL);
				(*map)[i].data.configs.group_or_pin = group;
				(*map)[i].data.configs.num_configs = 1;
				i++;
			}

			cfg = 0;
			if (!of_property_read_u32(np,
				"marvell,drive-strength", &ds))
				cfg = PXA3XX_PINCONF_DRIVE_STRENGTH
					| (ds << PXA3XX_PINCONF_DS_SHIFT);
			if (cfg) {
				(*map)[i].type = PIN_MAP_TYPE_CONFIGS_GROUP;
				(*map)[i].data.configs.configs =
					kmemdup(&cfg, sizeof(cfg), GFP_KERNEL);
				(*map)[i].data.configs.group_or_pin = group;
				(*map)[i].data.configs.num_configs = 1;
				i++;
			}
		}
	}
	*num_maps = count;
	return 0;
}

static void pxa3xx_pinctrl_dt_free_map(struct pinctrl_dev *pctrl_dev,
				       struct pinctrl_map *map,
				       unsigned num_maps)
{
	int i;

	for (i = 0; i < num_maps; i++)
		if (map[i].type == PIN_MAP_TYPE_CONFIGS_GROUP)
			kfree(map[i].data.configs.configs);
	kfree(map);
}

static struct pinctrl_ops pxa3xx_pctrl_ops = {
	.get_groups_count = pxa3xx_get_groups_count,
	.get_group_name	= pxa3xx_get_group_name,
	.get_group_pins	= pxa3xx_get_group_pins,
	.dt_node_to_map = pxa3xx_pinctrl_dt_node_to_map,
	.dt_free_map = pxa3xx_pinctrl_dt_free_map,
};

static int pxa3xx_pmx_get_funcs_count(struct pinctrl_dev *pctrldev)
{
	struct pxa3xx_pinmux_info *info = pinctrl_dev_get_drvdata(pctrldev);

	return info->num_funcs;
}

static const char *pxa3xx_pmx_get_func_name(struct pinctrl_dev *pctrldev,
					    unsigned func)
{
	struct pxa3xx_pinmux_info *info = pinctrl_dev_get_drvdata(pctrldev);
	return info->funcs[func].name;
}

static int pxa3xx_pmx_get_groups(struct pinctrl_dev *pctrldev, unsigned func,
				 const char * const **groups,
				 unsigned * const num_groups)
{
	struct pxa3xx_pinmux_info *info = pinctrl_dev_get_drvdata(pctrldev);
	*groups = info->funcs[func].groups;
	*num_groups = info->funcs[func].num_groups;
	return 0;
}

/* Return function number. If failure, return negative value. */
static int match_mux(struct pxa3xx_mfp_pin *mfp, unsigned mux)
{
	int i;
	for (i = 0; i < PXA3xx_MAX_MUX; i++) {
		if (mfp->func[i] == mux)
			break;
	}
	if (i >= PXA3xx_MAX_MUX)
		return -EINVAL;
	return i;
}

/* check whether current pin configuration is valid. Negative for failure */
static int match_group_mux(struct pxa3xx_pin_group *grp,
			   struct pxa3xx_pinmux_info *info,
			   unsigned mux)
{
	int i, pin, ret = 0;
	for (i = 0; i < grp->npins; i++) {
		pin = grp->pins[i];
		ret = match_mux(&info->mfp[pin], mux);
		if (ret < 0) {
			dev_err(info->dev, "Can't find mux %d on pin%d\n",
				mux, pin);
			break;
		}
	}
	return ret;
}

static int pxa3xx_pmx_enable(struct pinctrl_dev *pctrldev, unsigned func,
			     unsigned group)
{
	struct pxa3xx_pinmux_info *info = pinctrl_dev_get_drvdata(pctrldev);
	struct pxa3xx_pin_group *pin_grp = &info->grps[group];
	unsigned int data;
	int i, mfpr, pin, pin_func;

	if (!pin_grp->npins ||
		(match_group_mux(pin_grp, info, pin_grp->mux) < 0)) {
		dev_err(info->dev, "Failed to set the pin group: %d\n", group);
		return -EINVAL;
	}
	for (i = 0; i < pin_grp->npins; i++) {
		pin = pin_grp->pins[i];
		pin_func = match_mux(&info->mfp[pin], pin_grp->mux);
		mfpr = info->mfp[pin].mfpr;
		data = readl_relaxed(info->virt_base + mfpr);
		data &= ~MFPR_FUNC_MASK;
		data |= pin_func;
		writel_relaxed(data, info->virt_base + mfpr);
	}
	return 0;
}

static int pxa3xx_pmx_request_gpio(struct pinctrl_dev *pctrldev,
				   struct pinctrl_gpio_range *range,
				   unsigned pin)
{
	struct pxa3xx_pinmux_info *info = pinctrl_dev_get_drvdata(pctrldev);
	unsigned int data;
	int pin_func, mfpr;

	pin_func = match_mux(&info->mfp[pin], PXA3xx_MUX_GPIO);
	if (pin_func < 0) {
		dev_err(info->dev, "No GPIO function on pin%d (%s)\n",
			pin, info->pads[pin].name);
		return -EINVAL;
	}
	mfpr = info->mfp[pin].mfpr;
	/* write gpio function into mfpr register */
	data = readl_relaxed(info->virt_base + mfpr) & ~MFPR_FUNC_MASK;
	data |= pin_func;
	writel_relaxed(data, info->virt_base + mfpr);
	return 0;
}

static struct pinmux_ops pxa3xx_pmx_ops = {
	.get_functions_count	= pxa3xx_pmx_get_funcs_count,
	.get_function_name	= pxa3xx_pmx_get_func_name,
	.get_function_groups	= pxa3xx_pmx_get_groups,
	.enable			= pxa3xx_pmx_enable,
	.gpio_request_enable	= pxa3xx_pmx_request_gpio,
};

static int pxa3xx_pinconf_get(struct pinctrl_dev *pctrldev,
			      unsigned pin, unsigned long *config)
{
	struct pxa3xx_pinmux_info *info = pinctrl_dev_get_drvdata(pctrldev);
	unsigned int data, mask;
	int mfpr;

	mfpr = info->mfp[pin].mfpr;
	data = readl_relaxed(info->virt_base + mfpr);

	*config = 0;
	mask = PXA3XX_MFPR_PULL_SEL | PXA3XX_MFPR_PULL_UP;
	if ((data & mask) == mask)
		*config |= PXA3XX_PINCONF_PULL_UP;
	mask = PXA3XX_MFPR_PULL_SEL | PXA3XX_MFPR_PULL_DOWN;
	if ((data & mask) == mask)
		*config |= PXA3XX_PINCONF_PULL_DOWN;
	mask = info->ds_mask;
	if (data & mask) {
		*config |= PXA3XX_PINCONF_DRIVE_STRENGTH;
		*config |= ((data & mask) >> info->ds_shift)
				<< PXA3XX_PINCONF_DS_SHIFT;
	}
	mask = info->slp_mask;
	if (data & mask) {
		if (data & info->slp_input_low)
			*config |= PXA3XX_PINCONF_LOWPOWER_PULL_DOWN;
		if (data & info->slp_input_high)
			*config |= PXA3XX_PINCONF_LOWPOWER_PULL_UP;
		if (data & info->slp_output_low)
			*config |= PXA3XX_PINCONF_LOWPOWER_DRIVE_LOW;
		if (data & info->slp_output_high)
			*config |= PXA3XX_PINCONF_LOWPOWER_DRIVE_HIGH;
		if (data & info->slp_float)
			*config |= PXA3XX_PINCONF_LOWPOWER_FLOAT;
	} else
		*config |= PXA3XX_PINCONF_LOWPOWER_ZERO;
	return 0;
}

static int pxa3xx_pinconf_set(struct pinctrl_dev *pctrldev,
			      unsigned pin, unsigned long config)
{
	struct pxa3xx_pinmux_info *info = pinctrl_dev_get_drvdata(pctrldev);
	unsigned int data;
	int mfpr;

	mfpr = info->mfp[pin].mfpr;
	data = readl_relaxed(info->virt_base + mfpr);
	switch (config & PXA3XX_PINCONF_MASK) {
	case PXA3XX_PINCONF_PULL_DOWN:
		data |= PXA3XX_MFPR_PULL_SEL | PXA3XX_MFPR_PULL_DOWN;
		break;
	case PXA3XX_PINCONF_PULL_UP:
		data |= PXA3XX_MFPR_PULL_SEL | PXA3XX_MFPR_PULL_UP;
		break;
	case PXA3XX_PINCONF_DRIVE_STRENGTH:
		data &= ~info->ds_mask;
		data |= (config >> PXA3XX_PINCONF_DS_SHIFT) << info->ds_shift;
		break;
	case PXA3XX_PINCONF_LOWPOWER_DRIVE_HIGH:
		data &= ~info->slp_mask;
		data |= info->slp_output_high;
		break;
	case PXA3XX_PINCONF_LOWPOWER_DRIVE_LOW:
		data &= ~info->slp_mask;
		data |= info->slp_output_low;
		break;
	case PXA3XX_PINCONF_LOWPOWER_PULL_UP:
		data &= ~info->slp_mask;
		data |= info->slp_input_high;
		break;
	case PXA3XX_PINCONF_LOWPOWER_PULL_DOWN:
		data &= ~info->slp_mask;
		data |= info->slp_input_low;
		break;
	case PXA3XX_PINCONF_LOWPOWER_FLOAT:
		data &= ~info->slp_mask;
		data |= info->slp_float;
		break;
	case PXA3XX_PINCONF_LOWPOWER_ZERO:
		data &= ~info->slp_mask;
		break;
	default:
		return -ENOTSUPP;
	}
	writel_relaxed(data, info->virt_base + mfpr);
	return 0;
}

static int pxa3xx_pinconf_group_get(struct pinctrl_dev *pctrldev,
				    unsigned group, unsigned long *config)
{
	struct pxa3xx_pinmux_info *info = pinctrl_dev_get_drvdata(pctrldev);
	const unsigned *pins = info->grps[group].pins;
	int npins, i;
	unsigned long conf, match = 0;

	npins = info->grps[group].npins;
	for (i = 0; i < npins; i++) {
		pxa3xx_pinconf_get(pctrldev, pins[i], &conf);
		if (!match)
			match = conf;
		else if (match != conf) {
			*config = conf;
			return -EINVAL;
		}
	}
	*config = conf;
	return 0;
}

static int pxa3xx_pinconf_group_set(struct pinctrl_dev *pctrldev,
				    unsigned group, unsigned long config)
{
	struct pxa3xx_pinmux_info *info = pinctrl_dev_get_drvdata(pctrldev);
	const unsigned *pins = info->grps[group].pins;
	int npins, i;

	npins = info->grps[group].npins;
	for (i = 0; i < npins; i++)
		pxa3xx_pinconf_set(pctrldev, pins[i], config);
	return 0;
}

static void pxa3xx_pinconf_dbg_show(struct pinctrl_dev *pctrldev,
				    struct seq_file *s, unsigned offset)
{
	struct pxa3xx_pinmux_info *info = pinctrl_dev_get_drvdata(pctrldev);
	unsigned long config;
	char buf[80];
	int i = 0, mfpr;

	pxa3xx_pinconf_get(pctrldev, offset, &config);
	memset(buf, 0, 80);
	if (config & PXA3XX_PINCONF_PULL_UP)
		i += sprintf(buf + i, "PULL UP, ");
	if (config & PXA3XX_PINCONF_PULL_DOWN)
		i += sprintf(buf + i, "PULL DOWN, ");
	if (config & PXA3XX_PINCONF_DRIVE_STRENGTH)
		i += sprintf(buf + i, "DRIVE STRENGTH (%ld), ",
			config >> PXA3XX_PINCONF_DS_SHIFT);
	if (config & PXA3XX_PINCONF_LOWPOWER_PULL_UP)
		i += sprintf(buf + i, "LP PULL UP, ");
	if (config & PXA3XX_PINCONF_LOWPOWER_PULL_DOWN)
		i += sprintf(buf + i, "LP PULL DOWN, ");
	if (config & PXA3XX_PINCONF_LOWPOWER_DRIVE_HIGH)
		i += sprintf(buf + i, "LP DRIVE HIGH, ");
	if (config & PXA3XX_PINCONF_LOWPOWER_DRIVE_LOW)
		i += sprintf(buf + i, "LP DRIVE LOW, ");
	if (config & PXA3XX_PINCONF_LOWPOWER_FLOAT)
		i += sprintf(buf + i, "LP FLOAT, ");
	if (config & PXA3XX_PINCONF_LOWPOWER_ZERO)
		i += sprintf(buf + i, "LP ZERO, ");
	seq_printf(s, "%s\n", buf);

	mfpr = info->mfp[offset].mfpr;
	seq_printf(s, "reg[0x%x]:0x%x\n", info->phy_base + mfpr,
		   readl_relaxed(info->virt_base + mfpr));
}

static void pxa3xx_pinconf_group_dbg_show(struct pinctrl_dev *pctrldev,
					  struct seq_file *s, unsigned group)
{
	struct pxa3xx_pinmux_info *info = pinctrl_dev_get_drvdata(pctrldev);
	const unsigned *pins = info->grps[group].pins;
	int ret;
	unsigned long config;

	ret = pxa3xx_pinconf_group_get(pctrldev, group, &config);
	if (ret < 0) {
		seq_printf(s, "group config is not consistent\n");
		return;
	}
	pxa3xx_pinconf_dbg_show(pctrldev, s, pins[0]);
}

static struct pinconf_ops pxa3xx_pinconf_ops = {
	.pin_config_get		= pxa3xx_pinconf_get,
	.pin_config_set		= pxa3xx_pinconf_set,
	.pin_config_group_get	= pxa3xx_pinconf_group_get,
	.pin_config_group_set	= pxa3xx_pinconf_group_set,
	.pin_config_dbg_show	= pxa3xx_pinconf_dbg_show,
	.pin_config_group_dbg_show = pxa3xx_pinconf_group_dbg_show,
};

int pxa3xx_pinctrl_register(struct platform_device *pdev,
			    struct pxa3xx_pinmux_info *info)
{
	struct pinctrl_desc *desc;
	struct resource *res;
	int ret = 0;

	if (!info || !info->cputype)
		return -EINVAL;
	desc = info->desc;
	desc->pins = info->pads;
	desc->npins = info->num_pads;
	desc->pctlops = &pxa3xx_pctrl_ops;
	desc->pmxops = &pxa3xx_pmx_ops;
	desc->confops = &pxa3xx_pinconf_ops;
	info->dev = &pdev->dev;
	pxa3xx_pinctrl_gpio_range.npins = info->num_gpio;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENOENT;
	info->phy_base = res->start;
	info->phy_size = resource_size(res);
	info->virt_base = ioremap(info->phy_base, info->phy_size);
	if (!info->virt_base)
		return -ENOMEM;
	info->pctrl = pinctrl_register(desc, &pdev->dev, info);
	if (!info->pctrl) {
		dev_err(&pdev->dev, "failed to register PXA pinmux driver\n");
		ret = -EINVAL;
		goto err;
	}
	pinctrl_add_gpio_range(info->pctrl, &pxa3xx_pinctrl_gpio_range);
	platform_set_drvdata(pdev, info);
	return 0;
err:
	iounmap(info->virt_base);
	return ret;
}

int pxa3xx_pinctrl_unregister(struct platform_device *pdev)
{
	struct pxa3xx_pinmux_info *info = platform_get_drvdata(pdev);

	pinctrl_unregister(info->pctrl);
	iounmap(info->virt_base);
	platform_set_drvdata(pdev, NULL);
	return 0;
}

static int __init pxa3xx_pinctrl_init(void)
{
	pr_info("pxa3xx-pinctrl: PXA3xx pinctrl driver initializing\n");
	return 0;
}
core_initcall_sync(pxa3xx_pinctrl_init);

static void __exit pxa3xx_pinctrl_exit(void)
{
}
module_exit(pxa3xx_pinctrl_exit);

MODULE_AUTHOR("Haojian Zhuang <haojian.zhuang@marvell.com>");
MODULE_DESCRIPTION("PXA3xx pin control driver");
MODULE_LICENSE("GPL v2");
