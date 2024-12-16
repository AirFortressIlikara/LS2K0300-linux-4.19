#ifndef __PINCTRL_LOONGSON_H__
#define __PINCTRL_LOONGSON_H__

#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/irqdomain.h>
#include <linux/gpio.h>
#include <linux/pinctrl/pinctrl.h>
#include <dt-bindings/pinctrl/ls2k300-pinfunc.h>
#include <linux/types.h>
#include <linux/bitops.h>
#include "core.h"

#define MAX_GPIOS_ON_CHIP  16
#define PIN_ARGS_FROM_INDEX 0
#define PIN_ARGS_FROM_INDEX     0
#define PIN_ARGS_TO_INDEX       1
#define PIN_ARGS_CFG_INDEX      2

enum gpio_function {
        GPIO_FUNC_GPIO     = 0x00,  //0000, GPIO as function GPIO / device 0
        GPIO_FUNC_1        = 0x01,  //0001, GPIO as function 1 / device 1
        GPIO_FUNC_2        = 0x02,  //0010, GPIO as function 2 / device 2
        GPIO_FUNC_MAIN     = 0x03,  //0011, GPIO as function MAIN / device 3
};

#define GPIO_OEN           0x800
#define GPIO_OUT           0x900
#define GPIO_IN            0xA00
#define GPIO_INT_EN        0xB00
#define GPIO_INT_POL       0xC00
#define GPIO_INT_EDGE      0xD00
#define GPIO_INT_CLR       0xE00
#define GPIO_INT_STS       0xf00
#define GPIO_INT_DUAL      0xf80

static inline u32 pin_bitmap(u32 from, u32 to)
{
        if (to == 31)
                return ~((1<<(from))-1);
        return ((~((1<<(from))-1))&((1 << ((to)+1))-1));
}

static inline unsigned bit_count(unsigned v)
{
        unsigned int c;
        for (c = 0; v; c++)
                v &= v - 1;
        return c;
}

struct loongson_gpio_chip {
        struct gpio_chip                gc;
        char                            name[6];                        /*name format "GPAX"(X = '0' \ "16" ......)*/
        u8                              idx;                            /*gpio index of this chip*/
        const struct device_node        *of_node;                       /*gpio chip device node*/
        int                             irq;                            /*gpio chip irq*/
        struct irq_domain               *irq_domain;
        struct pinctrl_gpio_range       grange;
        struct loongson_pinctrl          *pctl;
        spinlock_t                      lock;                           /*gpio func write lock*/

	u32                             used_pins_bitmap;               /*bitmap of pins for been had requested*/
        u32                             resume_pending;                 /*bitmap of pins wakeup pending when sleep*/
        u32                             sleep_level;                    /*bitmap of pins level when sleep*/
        u32                             wakeup_bitmap;                  /*bitmap of pins wakeup pins*/
};

#define gc_to_loongson_gc(gpiochip) container_of(gpiochip, struct loongson_gpio_chip, gc)

struct loongson_pinctrl_func {
        const char              *name;
        struct device_node      *of_node;       /* function device node */
        const char              **groups;       /* all sub groups name*/
        u8                      num_groups;     /* num groups of this function */
};

struct loongson_pinctrl_group {
        const char              *name;
        struct device_node      *of_node;       /* group device node */
        struct gpio_chip        *gc;            /* corresponding gpio chip*/
        unsigned                *pins;          /* Contained pins software gpio num*/
        u8                      num_pins;       /* num gpios of the set group*/
	u32                     pinmux_bitmap;  /* pins bitmap of this group*/
        u32                     pinmux_start;   /* pins start of this group*/
        u32                     pinmux_end;     /* pins end of this group*/
        enum gpio_function      pinmux_func;    /* pins function select of this group*/
};

struct loongson_pinctrl {
        void __iomem                    *func_base;
        void __iomem                    *io_base;
        struct device_node              *of_node;               /*pinctrl device_node*/
        struct device                   *dev;

        struct loongson_gpio_chip       *gpio_chips;            /*gpio chips of this pinctrl*/
        unsigned                        num_chips;              /*num gpio chips*/
        unsigned                        total_pins;             /*total pins of this pinctrl*/
        u32                             *bitmap_priv;           /*******private*******/

        struct pinctrl_desc             pctl_desc;
        struct pinctrl_dev              *pctl_dev;

        struct loongson_pinctrl_group    *groups;
        unsigned                        num_groups;

        struct loongson_pinctrl_func     *functions;
        unsigned                        num_funs;
        u32                             group_regs_offset;              /* gpio group regs offset get from dts  */

};

static inline struct loongson_pinctrl_group *find_group_by_of_node(
                struct loongson_pinctrl *pctl, struct device_node *np)
{
        int i;
        for (i = 0; i < pctl->num_groups; i++) {
                if (pctl->groups[i].of_node == np)
                        return &pctl->groups[i];
        }
        return NULL;
}

static inline struct loongson_pinctrl_func *find_func_by_of_node(
                struct loongson_pinctrl *pctl, struct device_node *np)
{
        int i;
        for (i = 0; i < pctl->num_funs; i++) {
                if (pctl->functions[i].of_node == np)
                        return &pctl->functions[i];
        }
        return NULL;
}

static inline void loongson_gpio_writel(struct loongson_gpio_chip *chip, int offset, u64 value)
{
        struct loongson_pinctrl *pctl = chip->pctl;

	writeb(value, pctl->io_base + (offset +chip->idx *16));
}

static inline u64 loongson_gpio_readl(struct loongson_gpio_chip *chip, int offset)
{
        struct loongson_pinctrl *pctl = chip->pctl;

	return readb(pctl->io_base + (offset + chip->idx *16));
}

static inline void loongson_gpio_oen_set(struct loongson_gpio_chip *chip,int offset, u64 value)
{
	loongson_gpio_writel(chip,GPIO_OEN+offset,value);
}

static inline void loongson_gpio_out_set(struct loongson_gpio_chip *chip,int offset, u64 value)
{
	loongson_gpio_writel(chip,GPIO_OUT+offset,value);
}

static inline u64 loongson_gpio_in_get(struct loongson_gpio_chip *chip,int offset)
{
	return loongson_gpio_readl(chip,offset+GPIO_IN);
}

static inline void loongson_gpio_inten_set(struct loongson_gpio_chip *chip,int offset, u64 value)
{
	loongson_gpio_writel(chip,GPIO_INT_EN+offset,value);
}

static inline void loongson_gpio_pol_set(struct loongson_gpio_chip *chip,int offset, u64 value)
{
	loongson_gpio_writel(chip,GPIO_INT_POL+offset,value);
}

static inline void loongson_gpio_edge_set(struct loongson_gpio_chip *chip,int offset, u64 value)
{
	loongson_gpio_writel(chip,GPIO_INT_EDGE+offset,value);
}

static inline void loongson_gpio_int_clr(struct loongson_gpio_chip *chip,int offset)
{
	struct loongson_pinctrl *pctl = chip->pctl;

	writeb(1,pctl->io_base+offset+GPIO_INT_CLR);
}

static inline u64 loongson_gpio_stats_get(struct loongson_gpio_chip *chip,int offset)
{
	return loongson_gpio_readl(chip,offset+GPIO_INT_STS);
}

static inline void loongson_gpio_dual_set(struct loongson_gpio_chip *chip,int offset, u64 value)
{

	loongson_gpio_writel(chip,GPIO_INT_DUAL+offset,value);
}

static inline void loongson_func_writel(struct loongson_gpio_chip *chip,int value,int start,int end)
{
        struct loongson_pinctrl *pctl = chip->pctl;
	u32 tmp;
	u32 i,j;

	tmp =readl(pctl->func_base + (chip->idx * pctl->group_regs_offset));

	for(i=start;i<=end;i++){
		j=i*2;
		tmp &=~(0x3 <<j);
		tmp |=(value << j);
	}

        writel(tmp ,pctl->func_base + (chip->idx * pctl->group_regs_offset));
}

static inline int loongson_func_readl(struct loongson_gpio_chip *chip,int pin)
{
        struct loongson_pinctrl *pctl = chip->pctl;
	u32 tmp;

	pin =pin*2;
	tmp=readl(pctl->func_base + (chip->idx * pctl->group_regs_offset));
	tmp =((tmp >>pin)&0x3);

	return tmp;
}

#endif




