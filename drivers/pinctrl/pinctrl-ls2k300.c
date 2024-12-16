#include <linux/module.h>
#include <linux/syscore_ops.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/of_irq.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/of_address.h>
#include <linux/regmap.h>
#include <linux/mfd/syscon.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/pinctrl/pinconf.h>
#include <linux/pinctrl/consumer.h>
#include <linux/pinctrl/pinconf-generic.h>
#include <linux/slab.h>
#include <dt-bindings/pinctrl/ls2k300-pinfunc.h>

#include "pinctrl-loongson.h"

static void loongson_gpio_set_func(struct loongson_gpio_chip *chip, enum gpio_function func,u32 start,u32 end)
{
	unsigned long flags;

	spin_lock_irqsave(&chip->lock, flags);

	loongson_func_writel(chip,func,start,end);

	spin_unlock_irqrestore(&chip->lock, flags);

}

static int loongson_get_group_count(struct pinctrl_dev *pctldev)
{
        struct loongson_pinctrl *pctl = pinctrl_dev_get_drvdata(pctldev);
        return pctl->num_groups;
}

static const char* loongson_get_group_name(struct pinctrl_dev *pctldev,
                unsigned selector)
{
        struct loongson_pinctrl *pctl = pinctrl_dev_get_drvdata(pctldev);

        BUG_ON(selector >= pctl->num_groups);
        return pctl->groups[selector].name;
}

static int loongson_get_group_pins(struct pinctrl_dev *pctldev,
                unsigned selector,
                const unsigned **pins,
                unsigned *num_pins)
{
        struct loongson_pinctrl *pctl = pinctrl_dev_get_drvdata(pctldev);

        BUG_ON(selector >= pctl->num_groups);
        *pins = pctl->groups[selector].pins;
        *num_pins = pctl->groups[selector].num_pins;

	return 0;
}

static int loongson_dt_node_to_map(struct pinctrl_dev *pctldev,
		struct device_node *np_config,
		struct pinctrl_map **maps, unsigned *num_maps)
{
	struct loongson_pinctrl *pctl = pinctrl_dev_get_drvdata(pctldev);
	struct gpio_chip *gc;
	struct loongson_gpio_chip *lsgc;
	struct device_node *node;
	struct loongson_pinctrl_group *grp;
	struct loongson_pinctrl_func *func;
	struct pinctrl_map *new_map;
	unsigned map_cnt = 0, cfg_map_cnt = 0;

	grp = find_group_by_of_node(pctl, np_config);
	if (!grp)
		return -EINVAL;

	if ((node = of_get_parent(np_config)) == pctl->of_node)
		func = find_func_by_of_node(pctl, np_config);
	else
		func = find_func_by_of_node(pctl, node);

	of_node_put(node);

	if (!func)
		return -EINVAL;

	if (of_find_property(np_config, "loongson,pinmux", NULL) &&
			of_find_property(np_config, "loongson,pinmux-funcsel", NULL))
		map_cnt++;
	if (!map_cnt) return -EINVAL;

	*maps = new_map = kzalloc(sizeof(*new_map) * map_cnt, GFP_KERNEL);
	if (!new_map)
		return -ENOMEM;

	*num_maps = cfg_map_cnt;
	if (!of_find_property(np_config, "loongson,pinmux", NULL))
		return 0;

	lsgc = gc_to_loongson_gc(grp->gc);
	lsgc->used_pins_bitmap |= grp->pinmux_bitmap;
	new_map[*num_maps].type = PIN_MAP_TYPE_MUX_GROUP;
	new_map[*num_maps].data.mux.function = func->name;
	new_map[*num_maps].data.mux.group = grp->name;
	(*num_maps)++;
	return 0;
}

static void loongson_dt_free_map(struct pinctrl_dev *pctldev,
		struct pinctrl_map *maps, unsigned num_maps)
{
	unsigned long *configs = NULL;
	int idx;
	for (idx = 0; idx < num_maps; idx++) {
		if (maps[idx].type == PIN_MAP_TYPE_CONFIGS_GROUP) {
			if (NULL == configs)
				configs = maps[idx].data.configs.configs;
			kfree(maps[idx].data.configs.group_or_pin);
		}
	}
	kfree(configs);
	kfree(maps);
}

static const struct pinctrl_ops loongson_pctl_ops = {
        .get_groups_count       = loongson_get_group_count,
        .get_group_name         = loongson_get_group_name,
        .get_group_pins         = loongson_get_group_pins,
        .dt_node_to_map         = loongson_dt_node_to_map,       /* TODO: use generic map*/
        .dt_free_map            = loongson_dt_free_map,
};

const unsigned char *func_map[] = {"gpio","func1","func2","main"};

static ssize_t dump_gpio(struct device *dev,
                struct device_attribute *attr,char *buf)
{
	struct loongson_pinctrl *pctl =dev_get_drvdata(dev);
	struct loongson_gpio_chip *gpio_chips = pctl->gpio_chips;
	int idx = 0;
	int i = 0;
	ssize_t len = 0;

	for(i=0; i<16;i++){
		for(idx = 0; idx < pctl->num_chips; idx++){
			unsigned int func = loongson_func_readl(&gpio_chips[idx],i);
			len += sprintf(buf + len, "PA%d-%02d:%04s \t", 0 + idx, i, func_map[func]);
		}

		len +=sprintf(buf+len,"\n");
	}

	return len;
}

static DEVICE_ATTR(dump_gpio, 0444, dump_gpio, NULL);

static struct attribute *loongson_pinctrl_attrs[] = {
        &dev_attr_dump_gpio.attr,
        NULL,
};

static const struct attribute_group loongson_pinctrl_attr_group = {
        .attrs = (struct attribute **)loongson_pinctrl_attrs,
};



static void loongson_gpio_set(struct gpio_chip *chip,
                unsigned pin, int value)
{
        struct loongson_gpio_chip *lsgc = gc_to_loongson_gc(chip);

        BUG_ON(pin > lsgc->gc.ngpio);

	loongson_gpio_out_set(lsgc, pin, value);
}

static int loongson_gpio_get(struct gpio_chip *chip, unsigned pin)
{
	struct loongson_gpio_chip *lsgc = gc_to_loongson_gc(chip);

	BUG_ON(pin > chip->ngpio);

	return loongson_gpio_in_get(lsgc,pin);
}

static int loongson_gpio_direction_input(struct gpio_chip *chip,
		unsigned pin)
{
	BUG_ON(pin > chip->ngpio);
	return pinctrl_gpio_direction_input(chip->base + pin);
}

static int loongson_gpio_direction_output(struct gpio_chip *chip,
		unsigned pin, int value)
{
	struct loongson_gpio_chip *lsgc = gc_to_loongson_gc(chip);

	pinctrl_gpio_direction_output(chip->base + pin);

	loongson_gpio_set(chip,pin,value);

	return 0;
}

static int loongson_gpio_to_irq(struct gpio_chip *chip,
		unsigned pin)
{
	struct loongson_gpio_chip *lsgc = gc_to_loongson_gc(chip);
	unsigned int virq;

	BUG_ON(pin > chip->ngpio);
	if (NULL == lsgc->irq_domain)
		return -ENXIO;
	virq = irq_create_mapping(lsgc->irq_domain, pin);

	return (virq) ?: -ENXIO;
}
static int loongson_gpio_request(struct gpio_chip *chip, unsigned offset)
{
	struct loongson_gpio_chip *lsgc = gc_to_loongson_gc(chip);
	unsigned gpio = chip->base + offset;

	if(lsgc->used_pins_bitmap & (1 << offset)) {
		printk("%s: %s  used_pins_bitmap: 0X%08X\n", __func__,lsgc->name, lsgc->used_pins_bitmap);
		printk("current gpio request pin: chip->name %s, gpio: 0X%08X\n", chip->of_node->name, 1 << offset);
		dump_stack();
		printk("%s:gpio functions has redefinition\n", __FILE__);
	}

	lsgc->used_pins_bitmap |= 1 << offset;

	return pinctrl_gpio_request(gpio);
}
static void loongson_gpio_free(struct gpio_chip *chip, unsigned offset)
{
	struct loongson_gpio_chip *lsgc = gc_to_loongson_gc(chip);
	unsigned gpio = chip->base + offset;
	pinctrl_gpio_free(gpio);
	lsgc->used_pins_bitmap &= ~(1 << offset);
}

static const struct gpio_chip loongson_gpiolib_chip = {
	.owner = THIS_MODULE,
        .set = loongson_gpio_set,
        .get = loongson_gpio_get,
        .direction_input = loongson_gpio_direction_input,
        .direction_output =loongson_gpio_direction_output,
        .to_irq = loongson_gpio_to_irq,
        .request = loongson_gpio_request,
        .free = loongson_gpio_free,
};

static int loongson_pinmux_get_functions_count(struct pinctrl_dev *pctldev)
{
        struct loongson_pinctrl *pctl = pinctrl_dev_get_drvdata(pctldev);

	return pctl->num_funs;
}

static const char *loongson_pinmux_get_function_name(struct pinctrl_dev *pctldev,
                unsigned selector)
{
        struct loongson_pinctrl *pctl = pinctrl_dev_get_drvdata(pctldev);

	return pctl->functions[selector].name;
}

static int loongson_pinmux_get_groups(struct pinctrl_dev *pctldev,
                unsigned selector,
                const char * const **groups,
                unsigned * const num_groups)
{
        struct loongson_pinctrl *pctl = pinctrl_dev_get_drvdata(pctldev);

        BUG_ON(selector > pctl->num_funs);
        *groups = pctl->functions[selector].groups;
        *num_groups = pctl->functions[selector].num_groups;

	return 0;
}

static int loongson_pinmux_enable(struct pinctrl_dev *pctldev,
		unsigned func_selector,
		unsigned group_selector)
{

	struct loongson_pinctrl *pctl = pinctrl_dev_get_drvdata(pctldev);
	struct loongson_pinctrl_group *grp = NULL;
	struct loongson_gpio_chip *lsgc;

	if (func_selector > pctl->num_funs ||
			group_selector > pctl->num_groups)
		return -EINVAL;

	grp = &pctl->groups[group_selector];
	lsgc = gc_to_loongson_gc(grp->gc);

	loongson_gpio_set_func(lsgc,grp->pinmux_func,grp->pinmux_start,grp->pinmux_end);

	return 0;
}

static int loongson_pinmux_gpio_set_dir(struct pinctrl_dev *pctldev,
		struct pinctrl_gpio_range *range, unsigned gpio,
		bool input)
{
	struct loongson_gpio_chip *lsgc = gc_to_loongson_gc(range->gc);
	unsigned pin = gpio - range->gc->base;
	enum gpio_function func = GPIO_FUNC_GPIO;

	loongson_gpio_set_func(lsgc,func,pin,pin);

	if(input){
		loongson_gpio_oen_set(lsgc,pin,1);
	}else{
		loongson_gpio_oen_set(lsgc,pin,0);
	}

	return 0;
}

static const struct pinmux_ops loongson_pinmux_ops = {
	.get_functions_count    = loongson_pinmux_get_functions_count,
        .get_function_name      = loongson_pinmux_get_function_name,
        .get_function_groups    = loongson_pinmux_get_groups,
        .set_mux                = loongson_pinmux_enable,
        .gpio_set_direction     = loongson_pinmux_gpio_set_dir,
};

static int loongson_gc_match(struct gpio_chip *chip, void *data)
{
        return chip->of_node == data;
}

static int loongson_init_group(struct device *dev, struct device_node *np,
                struct loongson_pinctrl_group *grp)
{
        struct device_node *gpio_np = NULL;
        struct of_phandle_args out_args;
        u32 func;
        int pin, idx = 0, index = 0;

        grp->name = np->name;
        grp->of_node = np;

	while(!of_parse_phandle_with_args(np, "loongson,pinmux",
				"#loongson,pinmux-cells", index++, &out_args)) {
		if (gpio_np != NULL && out_args.np != gpio_np){
			return -EINVAL;
		}
		if (of_property_read_u32(np, "loongson,pinmux-funcsel", &func)){
			return -EINVAL;
		}
		gpio_np = out_args.np;

		grp->pinmux_start =out_args.args[PIN_ARGS_FROM_INDEX];
		grp->pinmux_end   =out_args.args[PIN_ARGS_TO_INDEX];
		grp->pinmux_bitmap |= pin_bitmap((out_args.args[PIN_ARGS_FROM_INDEX]),
				(out_args.args[PIN_ARGS_TO_INDEX]));

	}
	if (!gpio_np)
		return 0;
	grp->gc = gpiochip_find(gpio_np, loongson_gc_match);
	if (!grp->gc){
		return -EINVAL;
	}
	grp->num_pins = bit_count(grp->pinmux_bitmap);
	grp->pins = devm_kzalloc(dev, sizeof(unsigned) * grp->num_pins,
			GFP_KERNEL);
	if (!grp->pins)
		return -ENOMEM;

	switch (func) {
		case PINCTL_FUNCTIONGPIO:  grp->pinmux_func = GPIO_FUNC_GPIO; break;
		case PINCTL_FUNCTION1:     grp->pinmux_func = GPIO_FUNC_1;    break;
		case PINCTL_FUNCTION2:     grp->pinmux_func = GPIO_FUNC_2;    break;
		case PINCTL_FUNCTIONMAIN:  grp->pinmux_func = GPIO_FUNC_MAIN; break;
		default:
			return -EINVAL;
	}
	for_each_set_bit(pin, (unsigned long *)&grp->pinmux_bitmap, grp->gc->ngpio)
	        grp->pins[idx++] = grp->gc->base + pin;

	return 0;
}

static void loongson_gpio_irq_mask(struct irq_data *irqd)
{
        struct loongson_gpio_chip *lsgc = irq_data_get_irq_chip_data(irqd);

	loongson_gpio_inten_set(lsgc,irqd->hwirq,0);
}

static void loongson_gpio_irq_unmask(struct irq_data *irqd)
{
        struct loongson_gpio_chip *lsgc = irq_data_get_irq_chip_data(irqd);

	loongson_gpio_inten_set(lsgc,irqd->hwirq,1);
}

static void loongson_gpio_irq_ack(struct irq_data *irqd)
{
        struct loongson_gpio_chip *lsgc = irq_data_get_irq_chip_data(irqd);

	loongson_gpio_int_clr(lsgc,irqd->hwirq+(lsgc->idx*16));
}

static int loongson_gpio_irq_set_type(struct irq_data *irqd, unsigned int flow_type)
{
        struct loongson_gpio_chip *lsgc = irq_data_get_irq_chip_data(irqd);

        if (!(flow_type & IRQ_TYPE_SENSE_MASK))
                return 0;

        if (flow_type & IRQ_TYPE_EDGE_BOTH)
                irq_set_handler_locked(irqd, handle_edge_irq);
        else
                irq_set_handler_locked(irqd, handle_level_irq);

        switch (flow_type & IRQD_TRIGGER_MASK) {
        case IRQ_TYPE_LEVEL_HIGH:
		loongson_gpio_pol_set(lsgc,irqd->hwirq,1);
		loongson_gpio_edge_set(lsgc,irqd->hwirq,0);
		break;
        case IRQ_TYPE_LEVEL_LOW:
		loongson_gpio_pol_set(lsgc,irqd->hwirq,0);
		loongson_gpio_edge_set(lsgc,irqd->hwirq,0);
		break;
        case IRQ_TYPE_EDGE_RISING:
		loongson_gpio_pol_set(lsgc,irqd->hwirq,1);
		loongson_gpio_edge_set(lsgc,irqd->hwirq,1);
		break;
        case IRQ_TYPE_EDGE_FALLING:
		loongson_gpio_pol_set(lsgc,irqd->hwirq,0);
		loongson_gpio_edge_set(lsgc,irqd->hwirq,1);
		break;
        case IRQ_TYPE_EDGE_BOTH:
		loongson_gpio_edge_set(lsgc,irqd->hwirq,1);
		loongson_gpio_dual_set(lsgc,irqd->hwirq,1);
		break;
        default:
                pr_err("unsupported external interrupt type\n");
                return -EINVAL;
        }

        return 0;
}

static int loongson_gpio_irq_set_wake(struct irq_data *irqd, unsigned int on)
{
        struct loongson_gpio_chip *lsgc = irq_data_get_irq_chip_data(irqd);
#if 0
	/* 2p500 not sleep.*/
        if (on)
                lsgc->wakeup_bitmap |= BIT(irqd->hwirq);
        else
                lsgc->wakeup_bitmap &= ~BIT(irqd->hwirq);

        return irq_set_irq_wake(lsgc->irq, on);
#else
	return 0;
#endif
}

static void loongson_gpio_irq_suspend(struct irq_data *irqd)
{

        /* handle wakeup irq in pinctrl suspend procedure. */
#if 0
        struct loongson_gpio_chip *lsgc = irq_data_get_irq_chip_data(irqd);
#endif
}

static void loongson_gpio_irq_resume(struct irq_data *irqd)
{
#if 0
        struct loongson_gpio_chip *lsgc = irq_data_get_irq_chip_data(irqd);
#endif
}

static int loongson_irq_request_resources(struct irq_data *irqd)
{
        struct gpio_chip *chip = irq_data_get_irq_chip_data(irqd);

        if (!try_module_get(chip->owner))
                return -ENODEV;

        if (gpiochip_lock_as_irq(chip, irqd->hwirq)) {
                pr_err("GPIO chip %s: unable to lock HW IRQ %lu for IRQ\n",
                        chip->label,
                        irqd->hwirq);
                module_put(chip->owner);
                return -EINVAL;
        }
        return 0;
}

static void loongson_irq_release_resources(struct irq_data *irqd)
{
        struct gpio_chip *chip = irq_data_get_irq_chip_data(irqd);

        gpiochip_unlock_as_irq(chip, irqd->hwirq);
        module_put(chip->owner);
}

static struct irq_chip loongson_gpio_irq_chip = {
        .name           = "GPIO",
        .irq_unmask     = loongson_gpio_irq_unmask,
        .irq_mask       = loongson_gpio_irq_mask,
        .irq_ack        = loongson_gpio_irq_ack,
        .irq_set_type   = loongson_gpio_irq_set_type,
        .irq_set_wake   = loongson_gpio_irq_set_wake,
        .irq_suspend    = loongson_gpio_irq_suspend,
        .irq_resume     = loongson_gpio_irq_resume,
        .irq_request_resources = loongson_irq_request_resources,
        .irq_release_resources = loongson_irq_release_resources,
};

static void loongson_gpio_irq_handler(struct irq_desc *desc)
{
        struct loongson_gpio_chip *lsgc = irq_desc_get_handler_data(desc);
        unsigned long pend=0,mask=0;
        unsigned int j;

	for(j =0;j<16;j++){
		mask = loongson_gpio_readl(lsgc,GPIO_INT_EN+j);
		pend = loongson_gpio_readl(lsgc,GPIO_INT_STS+j);
		pend = pend & mask;
		if(pend)
			break;
	}

	pend =j;

	if (irq_find_mapping(lsgc->irq_domain, pend))
		generic_handle_irq(irq_find_mapping(lsgc->irq_domain, pend));

        return;
}

static int loongson_irq_domain_xlate(struct irq_domain *d,
                struct device_node *ctrlr,
                const u32 *intspec, unsigned int intsize,
                unsigned long *out_hwirq, unsigned int *out_type)
{
        u32 pin;

        if (WARN_ON(intsize < 3))
                return -EINVAL;

	pin = *out_hwirq = intspec[0];
        *out_type = (intsize > 1) ? intspec[1] : IRQ_TYPE_NONE;

        return 0;
}


static int loongson_irq_domain_map(struct irq_domain *dm, unsigned int virq, irq_hw_number_t hwnum)
{
        struct loongson_gpio_chip *lsgc = dm->host_data;

	irq_set_chip_data(virq, lsgc);
        irq_set_chip_and_handler(virq, &loongson_gpio_irq_chip,
                                handle_level_irq);
        return 0;
}

static const struct irq_domain_ops loongson_irq_domain_ops = {
        .map = loongson_irq_domain_map,
	.xlate = loongson_irq_domain_xlate,
};

static int loongson_gpio_irq_init(struct loongson_pinctrl *pctl,
                struct device_node *np, int idx)
{
        struct loongson_gpio_chip *lsgc = &pctl->gpio_chips[idx];

        lsgc->irq = irq_of_parse_and_map(np, 0);
        if (!lsgc->irq){
                return -EINVAL;
	}
        lsgc->irq_domain = irq_domain_add_linear(np, lsgc->gc.ngpio,
                        &loongson_irq_domain_ops, (void *)lsgc);
        if (!lsgc->irq_domain)
                return -ENOMEM;

	irq_set_handler_data(lsgc->irq, lsgc);
        irq_set_chained_handler(lsgc->irq, loongson_gpio_irq_handler);

	return 0;
}

static int loongson_gpio_chip_add(struct loongson_pinctrl *pctl, struct device_node *np, int base,int idx)
{
	struct loongson_gpio_chip *lsgc = &pctl->gpio_chips[idx];
	struct gpio_chip *gc = &lsgc->gc;
	u32 ngpio;

	lsgc->gc = loongson_gpiolib_chip;
	snprintf(lsgc->name, sizeof(lsgc->name), "GPA%d",idx*16);

	if (of_property_read_u32(np, "loongson,num-gpios", &ngpio))
		ngpio = MAX_GPIOS_ON_CHIP;

	if (of_property_read_u32(np, "#gpio-cells", &gc->of_gpio_n_cells))
		gc->of_gpio_n_cells = 2;

	lsgc->used_pins_bitmap = 0;
	pr_debug("%s (%d) config:\ncells=%d\nbase%d\n",
                lsgc->name,
                ngpio,
                gc->of_gpio_n_cells,
                base);

	lsgc->of_node = np;
	lsgc->idx = idx;
	lsgc->pctl = pctl;
	gc->ngpio = (u16)ngpio;
	gc->of_node = np;
	gc->base = base;
	gc->parent= pctl->dev;
	gc->label = lsgc->name;

	return gpiochip_add(gc);
}

static int loongson_gpio_register(struct loongson_pinctrl *pctl)
{
        struct loongson_gpio_chip *gpio_chips;
        struct device_node *np;
        int idx = 0, ret;
        u32 offset;

        if (!of_property_read_u32(pctl->of_node,"loongson,regs-offset", &offset)){
                pctl->group_regs_offset = offset;
        }

        if (of_property_read_u32(pctl->of_node,"loongson,num-chips", &pctl->num_chips)){
                return -EINVAL;
	}
	pctl->bitmap_priv = devm_kzalloc(pctl->dev, sizeof(u32) * pctl->num_chips,
                        GFP_KERNEL);

	if (!pctl->bitmap_priv)
                return -ENOMEM;

        gpio_chips = devm_kzalloc(pctl->dev,
                        pctl->num_chips * sizeof(struct loongson_gpio_chip),
                        GFP_KERNEL);
        pctl->gpio_chips = gpio_chips;
        pctl->total_pins = 0;

	for_each_child_of_node(pctl->of_node, np) {
		if (!of_find_property(np, "gpio-controller", NULL))
		        continue;

		if (WARN_ON(idx >= pctl->num_chips))
			break;

		ret = loongson_gpio_chip_add(pctl, np, pctl->total_pins, idx);
		if (ret) {
			dev_err(pctl->dev, "%s gpio add failed\n", gpio_chips[idx].name);
			return ret;
		}
		pctl->total_pins += gpio_chips[idx].gc.ngpio;
		spin_lock_init(&gpio_chips[idx].lock);

		if (!of_find_property(np, "interrupt-controller", NULL))
		        goto next_chip;

		ret = loongson_gpio_irq_init(pctl, np, idx);
                if (ret)
                        dev_err(pctl->dev, "%s irq init failed\n",
                                        gpio_chips[idx].gc.label);

next_chip:
                of_node_get(np);
                idx++;
        }

        dev_info(pctl->dev, "%d gpio chip add success, pins %d\n",
                        idx, pctl->total_pins);
        return 0;
}

static struct loongson_pinctrl_group* loongson_pinctrl_create_groups(
                struct device *dev, unsigned *cnt)
{
        struct device_node *np, *subnp;
        struct loongson_pinctrl_group *grp;
        unsigned int i = 0;
	int  ret;

        *cnt = 0;
        for_each_child_of_node(dev->of_node, np) {
                unsigned count = 0;
                if (!!of_find_property(np, "gpio-controller", NULL))
                        continue;
                if (!!(count = of_get_child_count(np)))
                        *cnt += count;
                else
                        (*cnt)++;
        }

        grp = devm_kzalloc(dev, sizeof(*grp) * (*cnt), GFP_KERNEL);
        if (!grp)
                return ERR_PTR(-ENOMEM);

        for_each_child_of_node(dev->of_node, np) {
                if (!!of_find_property(np, "gpio-controller", NULL))
                        continue;

		ret = of_get_child_count(np);
                if (!ret) {
                        loongson_init_group(dev, np, &grp[i++]);
                        continue;
                }

                for_each_child_of_node(np, subnp) {
                        loongson_init_group(dev, subnp, &grp[i++]);
                }
        }
        return grp;
}

static struct loongson_pinctrl_func* loongson_pinctrl_create_functions(
                struct device *dev, unsigned int *cnt)
{
        struct device_node *np, *subnp;
        struct loongson_pinctrl_func *func;
        int i = 0;

        *cnt = 0;
        for_each_child_of_node(dev->of_node, np) {
                if (!!of_find_property(np, "gpio-controller", NULL))
                        continue;
                (*cnt)++;
        }

        func = devm_kzalloc(dev, sizeof(*func) * (*cnt), GFP_KERNEL);
        if (!func)
                return ERR_PTR(-ENOMEM);

        for_each_child_of_node(dev->of_node, np) {
                u8 num_child, j = 0;
                if (!!of_find_property(np, "gpio-controller", NULL))
                        continue;
                func[i].name = np->name;
                func[i].of_node = np;
                num_child = of_get_child_count(np);
                func[i].num_groups = num_child ?:1;
                func[i].groups = devm_kzalloc(dev, sizeof(char *) * func[i].num_groups,
                                GFP_KERNEL);
                if (!func[i].groups)
                        return ERR_PTR(-ENOMEM);
                if (num_child) {
                        for_each_child_of_node(np, subnp)
                                func[i].groups[j++] = subnp->name;
                } else
                        func[i].groups[0] = func[i].name;
                i++;
        }
        return func;
}

static int loongson_pinctrl_parse_dt(struct loongson_pinctrl *pctl)
{
        struct loongson_pinctrl_group *groups;
        struct loongson_pinctrl_func *functions;
        unsigned int grp_cnt = 0, func_cnt = 0;

        groups = loongson_pinctrl_create_groups(pctl->dev, &grp_cnt);
        if (IS_ERR(groups)) {
                dev_err(pctl->dev, "failed to parse pin groups\n");
                return PTR_ERR(groups);
        }

        functions = loongson_pinctrl_create_functions(pctl->dev, &func_cnt);
        if (IS_ERR(functions)) {
                dev_err(pctl->dev, "failed to parse pin functions\n");
                return PTR_ERR(groups);
        }

        pctl->groups = groups;
        pctl->num_groups = grp_cnt;
        pctl->functions = functions;
        pctl->num_funs = func_cnt;
        return 0;
}

static int loongson_pinctrl_register(struct loongson_pinctrl *pctl)
{
        struct pinctrl_desc *pctl_desc = &pctl->pctl_desc;
        struct pinctrl_pin_desc *pindesc, *pdesc;
        char *pin_names;
        unsigned chip, pin;
        int ret;

	pctl_desc->name = "loongson pinctrl";
	pctl_desc->owner = THIS_MODULE;
	pctl_desc->pctlops = &loongson_pctl_ops;
	pctl_desc->pmxops = &loongson_pinmux_ops;
	pctl_desc->confops = NULL;
	pctl_desc->npins = pctl->total_pins;
	pindesc = devm_kzalloc(pctl->dev,
			sizeof(*pindesc) * pctl_desc->npins, GFP_KERNEL);
	if (!pindesc)
		return -ENOMEM;
	pctl_desc->pins = pindesc;

	for (pin = 0, pdesc = pindesc; pin < pctl_desc->npins; pin++, pdesc++)
        pdesc->number = pin;

	pin_names = devm_kzalloc(pctl->dev,
			sizeof(char) * 15 * pctl_desc->npins,
			GFP_KERNEL);
	if (!pin_names)
		return -ENOMEM;

	for (chip = 0; chip < pctl->num_chips; chip++) {
		struct loongson_gpio_chip *lsgc = &pctl->gpio_chips[chip];
		for (pin = 0; pin < lsgc->gc.ngpio; pin++) {
			sprintf(pin_names, "%s-%d", lsgc->name, pin);
			pdesc = pindesc++;
			pdesc->name = pin_names;
			pin_names += 15;
		}
	}

	ret = loongson_pinctrl_parse_dt(pctl);
	if (ret)
		return ret;

	pctl->pctl_dev = pinctrl_register(pctl_desc, pctl->dev, pctl);
	if (!pctl->pctl_dev){
		return -EINVAL;
	}
	for (chip = 0; chip < pctl->num_chips; chip++) {
		struct loongson_gpio_chip *lsgc = &pctl->gpio_chips[chip];

		lsgc->grange.name = lsgc->name;
		lsgc->grange.id = lsgc->idx;
		lsgc->grange.pin_base = lsgc->gc.base;
		lsgc->grange.base = lsgc->gc.base;
		lsgc->grange.npins = lsgc->gc.ngpio;
		lsgc->grange.gc = &lsgc->gc;
		pinctrl_add_gpio_range(pctl->pctl_dev, &lsgc->grange);
	}

	return 0;
}

static int loongson_pinctrl_probe(struct platform_device *pdev)
{
        struct loongson_pinctrl *pctl;
        struct resource *res;
        struct resource *gpio_res;
	int ret;

        pctl = devm_kzalloc(&pdev->dev, sizeof(*pctl), GFP_KERNEL);
        if (!pctl)
                return -ENOMEM;

        pctl->dev = &pdev->dev;
        pctl->of_node = pdev->dev.of_node;


        res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
        if (!res)
                return -ENOMEM;

        pctl->func_base = devm_ioremap_resource(&pdev->dev, res);
        if (IS_ERR(pctl->func_base))
                return PTR_ERR(pctl->func_base);

	gpio_res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
        if (!gpio_res)
                return -ENOMEM;

	pctl->io_base = devm_ioremap_resource(&pdev->dev, gpio_res);
        if (IS_ERR(pctl->func_base))
                return PTR_ERR(pctl->func_base);

	platform_set_drvdata(pdev, pctl);

	ret =loongson_gpio_register(pctl);
	if(ret)
		return ret;

	ret =loongson_pinctrl_register(pctl);
	if(ret)
		return ret;

	ret = sysfs_create_group(&pdev->dev.kobj, &loongson_pinctrl_attr_group);
	if (ret)
		dev_info(&pdev->dev, "loongson pinctrl attr create failed\n");

	dev_info(&pdev->dev, "loongson pinctrl probe success\n");

	return 0;
}

static const struct of_device_id loongson_pinctrl_dt_match[] = {
        { .compatible = "loongson,ls2k300-pinctrl", .data = NULL},
        {},
};
MODULE_DEVICE_TABLE(of, loongson_pinctrl_dt_match);

static struct platform_driver loongson_pinctrl_drv = {
        .probe = loongson_pinctrl_probe,
        .driver = {
                .name = "loongson pinctrl",
                .owner = THIS_MODULE,
                .of_match_table = of_match_ptr(loongson_pinctrl_dt_match),
        },
};

static int __init loongson_pinctrl_drv_init(void)
{
	return platform_driver_register(&loongson_pinctrl_drv);
}
postcore_initcall(loongson_pinctrl_drv_init);

static void __exit loongson_pinctrl_drv_exit(void)
{
	platform_driver_unregister(&loongson_pinctrl_drv);
        return;
}
module_exit(loongson_pinctrl_drv_exit);

MODULE_AUTHOR("ruizhao <zhaorui@loongson.cn>");
MODULE_DESCRIPTION("loongson pinctrl driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:pinctrl");
