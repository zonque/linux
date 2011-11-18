/*
 * raumfeld_audio.c  --  SoC audio for Raumfeld DDX audio devices
 *
 * Copyright (c) 2011 Daniel Mack <zonque@gmail.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#include <asm/mach-types.h>

#include "pxa-ssp.h"

#define GPIO_CODEC_RESET	(120)

#define RAUMFELD_SOC_DAI_FMT (SND_SOC_DAIFMT_I2S   |		\
			      SND_SOC_DAIFMT_NB_NF |		\
			      SND_SOC_DAIFMT_CBS_CFS)

static void raumfeld_enable_audio(bool en)
{
	gpio_set_value(GPIO_CODEC_RESET, en);
}

static int raumfeld_analog_suspend(struct snd_soc_card *card)
{
	raumfeld_enable_audio(0);
	return 0;
}

static int raumfeld_analog_resume(struct snd_soc_card *card)
{
	raumfeld_enable_audio(1);
	return 0;
}

/* STA32X */
static int raumfeld_sta32x_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;

	/* fixed MCLK of 11.2896MHz */
	snd_pcm_hw_constraint_mask64(substream->runtime,
				    SNDRV_PCM_HW_PARAM_RATE,
				    SNDRV_PCM_RATE_44100 |
				    SNDRV_PCM_RATE_88200 |
				    SNDRV_PCM_RATE_176400);

	/*
	 * While both the CPU and the codec can actually handle
	 * 24bit material, the combination of both can't, due to
	 * constraints of the clock ratios and the external clock
	 * configuration.
	 */ 

	snd_pcm_hw_constraint_mask64(substream->runtime,
		    SNDRV_PCM_HW_PARAM_FORMAT,
		    SNDRV_PCM_FMTBIT_S16_LE  | SNDRV_PCM_FMTBIT_S16_BE |
		    SNDRV_PCM_FMTBIT_S32_LE  | SNDRV_PCM_FMTBIT_S32_BE);

	/* we have a fixed MCLK, set it here so ALSA knows
	 * the supported sample rates and can resample if necessary
	 */
	return snd_soc_dai_set_sysclk(codec_dai, 0, 11289600, 0);
}

static void raumfeld_sta32x_shutdown(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;

	/* set freq to 0 to enable all possible codec sample rates */
	snd_soc_dai_set_sysclk(codec_dai, 0, 0, 0);
}

static int raumfeld_sta32x_hw_params(struct snd_pcm_substream *substream,
				    struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	unsigned int clk = 0;
	int ret = 0;

	/* fixed MCLK of 11.2896MHz */
	switch (params_rate(params)) {
	case 44100:
	case 88200:
	case 176400:
		clk = 11289600;
		break;
	default:
		return -EINVAL;
	}

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S32_LE:
	case SNDRV_PCM_FORMAT_S32_BE:
		/* this enables network mode for 2 * 32bit samples */
		ret = snd_soc_dai_set_tdm_slot(cpu_dai, 3, 0, 2, 32);
		if (ret < 0)
			return ret;
		break;
	case SNDRV_PCM_FORMAT_S16_LE:
	case SNDRV_PCM_FORMAT_S16_BE:
		/* this disables network mode */
		ret = snd_soc_dai_set_tdm_slot(cpu_dai, 0, 0, 0, 16);
		if (ret < 0)
			return ret;
		break;
	default:
		return -EINVAL;
	}

	/* setup the CODEC DAI */
	ret = snd_soc_dai_set_sysclk(codec_dai, 0, clk, 0);
	if (ret < 0)
		return ret;

	/* setup the CPU DAI */
	ret = snd_soc_dai_set_pll(cpu_dai, 0, 0, 0, clk);
	if (ret < 0)
		return ret;

	ret = snd_soc_dai_set_clkdiv(cpu_dai, PXA_SSP_DIV_SCR, 4);
	if (ret < 0)
		return ret;

	ret = snd_soc_dai_set_sysclk(cpu_dai, PXA_SSP_CLK_EXT, clk, 1);
	if (ret < 0)
		return ret;

	return 0;
}

static struct snd_soc_ops raumfeld_sta32x_ops = {
	.startup = raumfeld_sta32x_startup,
	.shutdown = raumfeld_sta32x_shutdown,
	.hw_params = raumfeld_sta32x_hw_params,
};

/* WM8782 */
static int raumfeld_wm8782_hw_params(struct snd_pcm_substream *substream,
				     struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	unsigned int clk = 0;
	int ret = 0;

	// fixed MCLK of 11.2896MHz
	switch (params_rate(params)) {
	case 44100:
		clk = 11289600;
		break;
	default:
		return -EINVAL;
	}

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
	case SNDRV_PCM_FORMAT_S16_BE:
		break;
	default:
		return -EINVAL;
	}

	/* setup the CPU DAI */
	ret = snd_soc_dai_set_pll(cpu_dai, 0, 0, 0, clk);
	if (ret < 0)
		return ret;

	ret = snd_soc_dai_set_clkdiv(cpu_dai, PXA_SSP_DIV_SCR, 4);
	if (ret < 0)
		return ret;

	ret = snd_soc_dai_set_sysclk(cpu_dai, PXA_SSP_CLK_EXT, clk, 1);
	if (ret < 0)
		return ret;

	return 0;
}

static struct snd_soc_ops raumfeld_wm8782_ops = {
	.hw_params = raumfeld_wm8782_hw_params,
};

#define DAI_LINK_STA32X		\
{							\
	.name 		= "STA32X",			\
	.stream_name	= "Playback",			\
	.cpu_dai_name	= "pxa-ssp-dai.0",		\
	.platform_name	= "pxa-pcm-audio",		\
	.codec_dai_name	= "STA32X",			\
	.codec_name	= "sta32x.0-001a",		\
	.ops		= &raumfeld_sta32x_ops,		\
	.dai_fmt	= RAUMFELD_SOC_DAI_FMT,		\
}

#define DAI_LINK_WM8782		\
{							\
	.name		= "wm8782",			\
	.stream_name	= "wm8782",			\
	.cpu_dai_name	= "pxa-ssp-dai.0",		\
	.platform_name	= "pxa-pcm-audio",		\
	.codec_dai_name	= "wm8782",			\
	.codec_name	= "wm8782.0",			\
	.ops		= &raumfeld_wm8782_ops,		\
	.dai_fmt	= RAUMFELD_SOC_DAI_FMT,		\
}

static struct snd_soc_dai_link snd_soc_raumfeld_ddx_dai[] =
{
	DAI_LINK_STA32X,
	DAI_LINK_WM8782,
};

static struct platform_device *raumfeld_audio_device, *wm8782_device;

static int raumfeld_ddx_audio_probe(struct snd_soc_card *card)
{
	printk(KERN_ERR "%s()\n", __func__);

	wm8782_device = platform_device_alloc("wm8782", 0);
	if (!wm8782_device)
		return -ENOMEM;

	platform_device_add(wm8782_device);
	raumfeld_enable_audio(true);

	return 0;
}

static int raumfeld_ddx_audio_remove(struct snd_soc_card *card)
{
	raumfeld_enable_audio(false);
	platform_device_unregister(wm8782_device);
	gpio_free(GPIO_CODEC_RESET);

	return 0;
}

static struct snd_soc_card snd_soc_raumfeld_ddx = {
	.probe		= raumfeld_ddx_audio_probe,
	.remove		= raumfeld_ddx_audio_remove,
	.name		= "Raumfeld DDX",
	.dai_link	= snd_soc_raumfeld_ddx_dai,
	.num_links	= ARRAY_SIZE(snd_soc_raumfeld_ddx_dai),
	.suspend_post	= raumfeld_analog_suspend,
	.resume_pre	= raumfeld_analog_resume,
};

static int __init raumfeld_ddx_audio_init(void)
{
	int ret;
	
	if (!machine_is_raumfeld_speaker() &&
	    !machine_is_raumfeld_connector())
		return 0;

	raumfeld_audio_device = platform_device_alloc("soc-audio", -1);
	if (!raumfeld_audio_device)
		return -ENOMEM;

	platform_set_drvdata(raumfeld_audio_device,
			     &snd_soc_raumfeld_ddx);

	ret = platform_device_add(raumfeld_audio_device);
	if (ret < 0)
		return ret;

	return 0;
}

static void __exit raumfeld_ddx_audio_exit(void)
{
	platform_device_unregister(raumfeld_audio_device);
}

module_init(raumfeld_ddx_audio_init);
module_exit(raumfeld_ddx_audio_exit);

/* Module information */
MODULE_AUTHOR("Daniel Mack <zonque@gmail.com>");
MODULE_DESCRIPTION("Raumfeld DDX audio SoC");
MODULE_LICENSE("GPL");
