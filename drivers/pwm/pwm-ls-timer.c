// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) STMicroelectronics 2016
 *
 * Author: Gerald Baeza <gerald.baeza@st.com>
 *
 * Inspired by timer-stm32.c from Maxime Coquelin
 *             pwm-atmel.c from Bo Shen
 * 
 * Copyright (C) 2025 Ilikara <3435193369@qq.com>
 * Modified by: Ilikara <3435193369@qq.com>
 */

#include <linux/bitfield.h>
#include <linux/mfd/loongson-timers.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/pinctrl/consumer.h>
#include <linux/platform_device.h>
#include <linux/pwm.h>

#define CCMR_CHANNEL_SHIFT 8
#define CCMR_CHANNEL_MASK 0xFF
#define MAX_BREAKINPUT 2

struct loongson_breakinput {
	u32 index;
	u32 level;
	u32 filter;
};

struct loongson_pwm {
	struct pwm_chip chip;
	struct mutex lock; /* protect pwm config/enable */
	u64	clock_frequency;
	struct regmap *regmap;
	u32 max_arr;
	bool have_complementary_output;
	struct loongson_breakinput breakinputs[MAX_BREAKINPUT];
	unsigned int num_breakinputs;
	u32 capture[6] ____cacheline_aligned; /* DMA'able buffer */
};

static inline struct loongson_pwm *to_loongson_pwm_dev(struct pwm_chip *chip)
{
	return container_of(chip, struct loongson_pwm, chip);
}

static u32 active_channels(struct loongson_pwm *dev)
{
	u32 ccer;

	regmap_read(dev->regmap, TIM_CCER, &ccer);

	return ccer & TIM_CCER_CCXE;
}

#define TIM_CCER_CC12P (TIM_CCER_CC1P | TIM_CCER_CC2P)
#define TIM_CCER_CC12E (TIM_CCER_CC1E | TIM_CCER_CC2E)
#define TIM_CCER_CC34P (TIM_CCER_CC3P | TIM_CCER_CC4P)
#define TIM_CCER_CC34E (TIM_CCER_CC3E | TIM_CCER_CC4E)

/*
 * Capture using PWM input mode:
 *                              ___          ___
 * TI[1, 2, 3 or 4]: ........._|   |________|
 *                             ^0  ^1       ^2
 *                              .   .        .
 *                              .   .        XXXXX
 *                              .   .   XXXXX     |
 *                              .  XXXXX     .    |
 *                            XXXXX .        .    |
 * COUNTER:        ______XXXXX  .   .        .    |_XXX
 *                 start^       .   .        .        ^stop
 *                      .       .   .        .
 *                      v       v   .        v
 *                                  v
 * CCR1/CCR3:       tx..........t0...........t2
 * CCR2/CCR4:       tx..............t1.........
 *
 * DMA burst transfer:          |            |
 *                              v            v
 * DMA buffer:                  { t0, tx }   { t2, t1 }
 * DMA done:                                 ^
 *
 * 0: IC1/3 snapchot on rising edge: counter value -> CCR1/CCR3
 *    + DMA transfer CCR[1/3] & CCR[2/4] values (t0, tx: doesn't care)
 * 1: IC2/4 snapchot on falling edge: counter value -> CCR2/CCR4
 * 2: IC1/3 snapchot on rising edge: counter value -> CCR1/CCR3
 *    + DMA transfer CCR[1/3] & CCR[2/4] values (t2, t1)
 *
 * DMA done, compute:
 * - Period     = t2 - t0
 * - Duty cycle = t1 - t0
 */
static int loongson_pwm_raw_capture(struct pwm_chip *chip,
				    struct pwm_device *pwm,
				    unsigned long tmo_ms, u32 *raw_prd,
				    u32 *raw_dty)
{
	struct loongson_pwm *priv = to_loongson_pwm_dev(chip);
	struct device *parent = priv->chip.dev->parent;
	enum loongson_timers_dmas dma_id;
	u32 ccen, ccr;
	int ret;
	u32 capture[4];

	/* Ensure registers have been updated, enable counter and capture */
	regmap_update_bits(priv->regmap, TIM_EGR, TIM_EGR_UG, TIM_EGR_UG);
	regmap_update_bits(priv->regmap, TIM_CR1, TIM_CR1_CEN, TIM_CR1_CEN);

	/* Use cc1 or cc3 DMA resp for PWM input channels 1 & 2 or 3 & 4 */
	dma_id = pwm->hwpwm < 2 ? LS_TIMERS_DMA_CH1 : LS_TIMERS_DMA_CH3;
	ccen = pwm->hwpwm < 2 ? TIM_CCER_CC12E : TIM_CCER_CC34E;
	ccr = pwm->hwpwm < 2 ? TIM_CCR1 : TIM_CCR3;
	regmap_update_bits(priv->regmap, TIM_CCER, ccen, ccen);

	/*
	 * Timer DMA burst mode. Request 2 registers, 2 bursts, to get both
	 * CCR1 & CCR2 (or CCR3 & CCR4) on each capture event.
	 * We'll get two capture snapchots: { CCR1, CCR2 }, { CCR1, CCR2 }
	 * or { CCR3, CCR4 }, { CCR3, CCR4 }
	 */
	ret = loongson_timers_dma_burst_read(parent, priv->capture, dma_id, ccr,
					     2, 3, tmo_ms);
	if (ret)
		goto stop;
	capture[0] = priv->capture[1];
	capture[1] = priv->capture[4];
	capture[2] = priv->capture[2];
	capture[3] = priv->capture[5];

	/* Period: t2 - t0 (take care of counter overflow) */
	if (capture[0] <= capture[2])
		*raw_prd = capture[2] - capture[0];
	else
		*raw_prd = priv->max_arr - capture[0] + capture[2];

	/* Duty cycle capture requires at least two capture units */
	if (pwm->chip->npwm < 2)
		*raw_dty = 0;
	else if (capture[0] <= capture[3])
		*raw_dty = capture[3] - capture[0];
	else
		*raw_dty = priv->max_arr - capture[0] + capture[3];

	if (*raw_dty > *raw_prd) {
		/*
		 * Race beetween PWM input and DMA: it may happen
		 * falling edge triggers new capture on TI2/4 before DMA
		 * had a chance to read CCR2/4. It means capture[1]
		 * contains period + duty_cycle. So, subtract period.
		 */
		*raw_dty -= *raw_prd;
	}

stop:
	regmap_update_bits(priv->regmap, TIM_CCER, ccen, 0);
	regmap_update_bits(priv->regmap, TIM_CR1, TIM_CR1_CEN, 0);

	return ret;
}

static int loongson_pwm_capture(struct pwm_chip *chip, struct pwm_device *pwm,
				struct pwm_capture *result, unsigned long tmo_ms)
{
	struct loongson_pwm *priv = to_loongson_pwm_dev(chip);
	unsigned long long prd, div, dty;
	unsigned long rate;
	unsigned int psc = 0, icpsc, scale;
	u32 raw_prd = 0, raw_dty = 0;
	int ret = 0;

	mutex_lock(&priv->lock);

	if (active_channels(priv)) {
		ret = -EBUSY;
		goto unlock;
	}

	rate = priv->clock_frequency;

	/* prescaler: fit timeout window provided by upper layer */
	div = (unsigned long long)rate * (unsigned long long)tmo_ms;
	do_div(div, MSEC_PER_SEC);
	prd = div;
	while ((div > priv->max_arr) && (psc < MAX_TIM_PSC)) {
		psc++;
		div = prd;
		do_div(div, psc + 1);
	}
	regmap_write(priv->regmap, TIM_ARR, priv->max_arr);
	regmap_write(priv->regmap, TIM_PSC, psc);

	/* Reset input selector to its default input and disable slave mode */
	regmap_write(priv->regmap, TIM_SMCR, 0x0);

	/* Map TI1 or TI2 PWM input to IC1 & IC2 (or TI3/4 to IC3 & IC4) */
	regmap_update_bits(priv->regmap, pwm->hwpwm < 2 ? TIM_CCMR1 : TIM_CCMR2,
			   TIM_CCMR_CC1S | TIM_CCMR_CC2S,
			   pwm->hwpwm & 0x1 ?
				   TIM_CCMR_CC1S_TI2 | TIM_CCMR_CC2S_TI2 :
				   TIM_CCMR_CC1S_TI1 | TIM_CCMR_CC2S_TI1);

	/* Capture period on IC1/3 rising edge, duty cycle on IC2/4 falling. */
	regmap_update_bits(priv->regmap, TIM_CCER,
			   pwm->hwpwm < 2 ? TIM_CCER_CC12P : TIM_CCER_CC34P,
			   pwm->hwpwm < 2 ? TIM_CCER_CC2P : TIM_CCER_CC4P);

	ret = loongson_pwm_raw_capture(chip, pwm, tmo_ms, &raw_prd, &raw_dty);
	if (ret)
		goto stop;

	/*
	 * Got a capture. Try to improve accuracy at high rates:
	 * - decrease counter clock prescaler, scale up to max rate.
	 * - use input prescaler, capture once every /2 /4 or /8 edges.
	 */
	if (raw_prd) {
		u32 max_arr = priv->max_arr - 0x1000; /* arbitrary margin */

		scale = max_arr / min(max_arr, raw_prd);
	} else {
		scale = priv->max_arr; /* below resolution, use max scale */
	}

	if (psc && scale > 1) {
		/* 2nd measure with new scale */
		psc /= scale;
		regmap_write(priv->regmap, TIM_PSC, psc);
		ret = loongson_pwm_raw_capture(chip, pwm, tmo_ms, &raw_prd,
					       &raw_dty);
		if (ret)
			goto stop;
	}

	/* Compute intermediate period not to exceed timeout at low rates */
	prd = (unsigned long long)raw_prd * (psc + 1) * NSEC_PER_SEC;
	do_div(prd, rate);

	for (icpsc = 0; icpsc < MAX_TIM_ICPSC ; icpsc++) {
		/* input prescaler: also keep arbitrary margin */
		if (raw_prd >= (priv->max_arr - 0x1000) >> (icpsc + 1))
			break;
		if (prd >= (tmo_ms * NSEC_PER_MSEC) >> (icpsc + 2))
			break;
	}

	if (!icpsc)
		goto done;

	/* Last chance to improve period accuracy, using input prescaler */
	regmap_update_bits(priv->regmap,
			   pwm->hwpwm < 2 ? TIM_CCMR1 : TIM_CCMR2,
			   TIM_CCMR_IC1PSC | TIM_CCMR_IC2PSC,
			   FIELD_PREP(TIM_CCMR_IC1PSC, icpsc) |
				   FIELD_PREP(TIM_CCMR_IC2PSC, icpsc));

	ret = loongson_pwm_raw_capture(chip, pwm, tmo_ms, &raw_prd, &raw_dty);
	if (ret)
		goto stop;

	if (raw_dty >= (raw_prd >> icpsc)) {
		/*
		 * We may fall here using input prescaler, when input
		 * capture starts on high side (before falling edge).
		 * Example with icpsc to capture on each 4 events:
		 *
		 *       start   1st capture                     2nd capture
		 *         v     v                               v
		 *         ___   _____   _____   _____   _____   ____
		 * TI1..4     |__|    |__|    |__|    |__|    |__|
		 *            v  v    .  .    .  .    .       v  v
		 * icpsc1/3:  .  0    .  1    .  2    .  3    .  0
		 * icpsc2/4:  0       1       2       3       0
		 *            v  v                            v  v
		 * CCR1/3  ......t0..............................t2
		 * CCR2/4  ..t1..............................t1'...
		 *               .                            .  .
		 * Capture0:     .<----------------------------->.
		 * Capture1:     .<-------------------------->.  .
		 *               .                            .  .
		 * Period:       .<------>                    .  .
		 * Low side:                                  .<>.
		 *
		 * Result:
		 * - Period = Capture0 / icpsc
		 * - Duty = Period - Low side = Period - (Capture0 - Capture1)
		 */
		raw_dty = (raw_prd >> icpsc) - (raw_prd - raw_dty);
	}

done:
	prd = (unsigned long long)raw_prd * (psc + 1) * NSEC_PER_SEC;
	result->period = DIV_ROUND_UP_ULL(prd, rate << icpsc);
	dty = (unsigned long long)raw_dty * (psc + 1) * NSEC_PER_SEC;
	result->duty_cycle = DIV_ROUND_UP_ULL(dty, rate);
stop:
	regmap_write(priv->regmap, TIM_CCER, 0);
	regmap_write(priv->regmap, pwm->hwpwm < 2 ? TIM_CCMR1 : TIM_CCMR2, 0);
	regmap_write(priv->regmap, TIM_PSC, 0);
clk_dis:
unlock:
	mutex_unlock(&priv->lock);

	return ret;
}

static int loongson_pwm_config(struct loongson_pwm *priv, int ch, u64 duty_ns,
			       u64 period_ns)
{
	unsigned long long prd, div, dty;
	unsigned int prescaler = 0;
	u32 ccmr, mask, shift;

	/* Period and prescaler values depends on clock rate */
	div = (unsigned long long)priv->clock_frequency * period_ns;

	do_div(div, NSEC_PER_SEC);
	prd = div;

	while (div > priv->max_arr) {
		prescaler++;
		div = prd;
		do_div(div, prescaler + 1);
	}

	prd = div;

	if (prescaler > MAX_TIM_PSC)
		return -EINVAL;

	/*
	 * All channels share the same prescaler and counter so when two
	 * channels are active at the same time we can't change them
	 */
	if (active_channels(priv) & ~(1 << ch * 4)) {
		u32 psc, arr;

		regmap_read(priv->regmap, TIM_PSC, &psc);
		regmap_read(priv->regmap, TIM_ARR, &arr);

		if ((psc != prescaler) || (arr != prd - 1))
			return -EBUSY;
	}

	regmap_write(priv->regmap, TIM_PSC, prescaler);
	regmap_write(priv->regmap, TIM_ARR, prd - 1);
	regmap_update_bits(priv->regmap, TIM_CR1, TIM_CR1_ARPE, TIM_CR1_ARPE);

	/* Calculate the duty cycles */
	dty = prd * duty_ns;
	do_div(dty, period_ns);

	regmap_write(priv->regmap, TIM_CCRx(ch + 1), dty);

	/* Configure output mode */
	shift = (ch & 0x1) * CCMR_CHANNEL_SHIFT;
	ccmr = (TIM_CCMR_PE | TIM_CCMR_M1) << shift;
	mask = CCMR_CHANNEL_MASK << shift;

	if (ch < 2)
		regmap_update_bits(priv->regmap, TIM_CCMR1, mask, ccmr);
	else
		regmap_update_bits(priv->regmap, TIM_CCMR2, mask, ccmr);

	regmap_update_bits(priv->regmap, TIM_BDTR,
		TIM_BDTR_MOE | TIM_BDTR_AOE,
		TIM_BDTR_MOE | TIM_BDTR_AOE);

	return 0;
}

static int loongson_pwm_set_polarity(struct loongson_pwm *priv, unsigned int ch,
				     enum pwm_polarity polarity)
{
	u32 mask;

	mask = TIM_CCER_CCxP(ch + 1);
	if (priv->have_complementary_output)
		mask |= TIM_CCER_CCxNP(ch + 1);

	regmap_update_bits(priv->regmap, TIM_CCER, mask,
			   polarity == PWM_POLARITY_NORMAL ? 0 : mask);

	return 0;
}

static int loongson_pwm_enable(struct loongson_pwm *priv, unsigned int ch)
{
	u32 mask;
	int ret;

	/* Enable channel */
	mask = TIM_CCER_CCxE(ch + 1);
	if (priv->have_complementary_output)
		mask |= TIM_CCER_CCxNE(ch + 1);

	regmap_update_bits(priv->regmap, TIM_CCER, mask, mask);

	/* Make sure that registers are updated */
	regmap_update_bits(priv->regmap, TIM_EGR, TIM_EGR_UG, TIM_EGR_UG);

	/* Enable controller */
	regmap_update_bits(priv->regmap, TIM_CR1, TIM_CR1_CEN, TIM_CR1_CEN);

	return 0;
}

static void loongson_pwm_disable(struct loongson_pwm *priv, unsigned int ch)
{
	u32 mask;

	/* Disable channel */
	mask = TIM_CCER_CCxE(ch + 1);
	if (priv->have_complementary_output)
		mask |= TIM_CCER_CCxNE(ch + 1);

	regmap_update_bits(priv->regmap, TIM_CCER, mask, 0);

	/* When all channels are disabled, we can disable the controller */
	if (!active_channels(priv))
		regmap_update_bits(priv->regmap, TIM_CR1, TIM_CR1_CEN, 0);
}

static int loongson_pwm_apply(struct pwm_chip *chip, struct pwm_device *pwm,
			      struct pwm_state *state)
{
	bool enabled;
	struct loongson_pwm *priv = to_loongson_pwm_dev(chip);
	int ret;

	enabled = pwm->state.enabled;

	if (!state->enabled) {
		if (enabled)
			loongson_pwm_disable(priv, pwm->hwpwm);
		return 0;
	}

	if (state->polarity != pwm->state.polarity)
		loongson_pwm_set_polarity(priv, pwm->hwpwm, state->polarity);

	ret = loongson_pwm_config(priv, pwm->hwpwm, state->duty_cycle,
				  state->period);
	if (ret)
		return ret;

	if (!enabled && state->enabled)
		ret = loongson_pwm_enable(priv, pwm->hwpwm);

	return ret;
}

static int loongson_pwm_apply_locked(struct pwm_chip *chip,
				     struct pwm_device *pwm,
				     struct pwm_state *state)
{
	struct loongson_pwm *priv = to_loongson_pwm_dev(chip);
	int ret;

	/* protect common prescaler for all active channels */
	mutex_lock(&priv->lock);
	ret = loongson_pwm_apply(chip, pwm, state);
	mutex_unlock(&priv->lock);

	return ret;
}

static const struct pwm_ops ls_timer_pwm_ops = {
	.owner = THIS_MODULE,
	.apply = loongson_pwm_apply_locked,
	.capture = IS_ENABLED(CONFIG_DMA_ENGINE) ? loongson_pwm_capture : NULL,
};

static int loongson_pwm_set_breakinput(struct loongson_pwm *priv,
				       const struct loongson_breakinput *bi)
{
	u32 shift = TIM_BDTR_BKF_SHIFT(bi->index);
	u32 bke = TIM_BDTR_BKE(bi->index);
	u32 bkp = TIM_BDTR_BKP(bi->index);
	u32 bkf = TIM_BDTR_BKF(bi->index);
	u32 mask = bkf | bkp | bke;
	u32 bdtr;

	bdtr = (bi->filter & TIM_BDTR_BKF_MASK) << shift | bke;

	if (bi->level)
		bdtr |= bkp;

	regmap_update_bits(priv->regmap, TIM_BDTR, mask, bdtr);

	regmap_read(priv->regmap, TIM_BDTR, &bdtr);

	return (bdtr & bke) ? 0 : -EINVAL;
}

static int loongson_pwm_apply_breakinputs(struct loongson_pwm *priv,
					  struct device_node *np)
{
	int nb, ret, array_size;
	unsigned int i;

	nb = of_property_count_elems_of_size(
		np, "loongson,breakinput", sizeof(struct loongson_breakinput));

	/*
	 * Because "loongson,breakinput" parameter is optional do not make probe
	 * failed if it doesn't exist.
	 */
	if (nb <= 0)
		return 0;

	if (nb > MAX_BREAKINPUT)
		return -EINVAL;

	priv->num_breakinputs = nb;
	array_size = nb * sizeof(struct loongson_breakinput) / sizeof(u32);
	ret = of_property_read_u32_array(np, "loongson,breakinput",
					 (u32 *)priv->breakinputs, array_size);
	if (ret)
		return ret;

	for (i = 0; i < priv->num_breakinputs; i++) {
		if (priv->breakinputs[i].index > 1 ||
		    priv->breakinputs[i].level > 1 ||
		    priv->breakinputs[i].filter > 15)
			return -EINVAL;
	}

	for (i = 0; i < nb && !ret; i++) {
		ret = loongson_pwm_set_breakinput(priv, &priv->breakinputs[i]);
	}

	return ret;
}

static void loongson_pwm_detect_complementary(struct loongson_pwm *priv)
{
	u32 ccer;

	/*
	 * If complementary bit doesn't exist writing 1 will have no
	 * effect so we can detect it.
	 */
	regmap_update_bits(priv->regmap, TIM_CCER, TIM_CCER_CC1NE, TIM_CCER_CC1NE);
	regmap_read(priv->regmap, TIM_CCER, &ccer);
	regmap_update_bits(priv->regmap, TIM_CCER, TIM_CCER_CC1NE, 0);

	priv->have_complementary_output = (ccer != 0);
}

static int loongson_pwm_detect_channels(struct loongson_pwm *priv)
{
	u32 ccer, ccer_backup;
	int npwm = 0;

	/*
	 * If channels enable bits don't exist writing 1 will have no
	 * effect so we can detect and count them.
	 */
	regmap_read(priv->regmap, TIM_CCER, &ccer_backup);
	regmap_update_bits(priv->regmap, TIM_CCER, TIM_CCER_CCXE, TIM_CCER_CCXE);
	regmap_read(priv->regmap, TIM_CCER, &ccer);
	regmap_update_bits(priv->regmap, TIM_CCER, TIM_CCER_CCXE, ccer_backup);

	if (ccer & TIM_CCER_CC1E)
		npwm++;

	if (ccer & TIM_CCER_CC2E)
		npwm++;

	if (ccer & TIM_CCER_CC3E)
		npwm++;

	if (ccer & TIM_CCER_CC4E)
		npwm++;

	return npwm;
}

static int loongson_pwm_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct loongson_timers *ddata = dev_get_drvdata(pdev->dev.parent);
	struct loongson_pwm *priv;
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	mutex_init(&priv->lock);
	priv->regmap = ddata->regmap;
	priv->clock_frequency = ddata->clock_frequency;
	priv->max_arr = ddata->max_arr;

	if (!priv->regmap)
		return dev_err_probe(dev, -EINVAL, "Failed to get %s\n", "regmap");

	ret = loongson_pwm_apply_breakinputs(priv, np);
	if (ret)
		return dev_err_probe(dev, ret,
				     "Failed to configure breakinputs\n");

	loongson_pwm_detect_complementary(priv);

	priv->chip.base = -1;
	priv->chip.dev = dev;
	priv->chip.ops = &ls_timer_pwm_ops;
	priv->chip.npwm = loongson_pwm_detect_channels(priv);

	ret = pwmchip_add(&priv->chip);
	if (ret < 0)
		return dev_err_probe(dev, ret, "Failed to register pwmchip\n");

	platform_set_drvdata(pdev, priv);

	return 0;
}

static int loongson_pwm_remove(struct platform_device *pdev)
{
	struct loongson_pwm *priv = platform_get_drvdata(pdev);
	unsigned int i;

	for (i = 0; i < priv->chip.npwm; i++)
		pwm_disable(&priv->chip.pwms[i]);

	pwmchip_remove(&priv->chip);

	return 0;
}

static const struct of_device_id loongson_pwm_of_match[] = {
	{ .compatible = "loongson,ls2k-pwm-timer", },
	{ /* end node */ },
};
MODULE_DEVICE_TABLE(of, loongson_pwm_of_match);

static struct platform_driver loongson_pwm_driver = {
	.probe	= loongson_pwm_probe,
	.remove	= loongson_pwm_remove,
	.driver	= {
		.name = "loongson-pwm-timer",
		.of_match_table = loongson_pwm_of_match,
	},
};
module_platform_driver(loongson_pwm_driver);

MODULE_ALIAS("platform:loongson-pwm-timer");
MODULE_DESCRIPTION("Loongson loongson2k PWM driver");
MODULE_LICENSE("GPL v2");
