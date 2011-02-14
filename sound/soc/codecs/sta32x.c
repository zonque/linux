/*
 * Codec driver for ST STA32x 2.1-channel high-efficiency digital audio system
 *
 * Copyright: 2011 Raumfeld GmbH
 * Author: Johannes Stezenbach <js@sig21.net>
 *
 * based on code from:
 *	Wolfson Microelectronics PLC.
 *	  Mark Brown <broonie@opensource.wolfsonmicro.com>
 *	Freescale Semiconductor, Inc.
 *	  Timur Tabi <timur@freescale.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#define DEBUG
#define pr_fmt(fmt) KBUILD_MODNAME ":%s:%d: " fmt, __func__, __LINE__

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <sound/tlv.h>

#include "sta32x.h"


#define STA32X_NUM_RATES 7
#define STA32X_RATES (SNDRV_PCM_RATE_32000 | \
		      SNDRV_PCM_RATE_44100 | \
		      SNDRV_PCM_RATE_48000 | \
		      SNDRV_PCM_RATE_88200 | \
		      SNDRV_PCM_RATE_96000 | \
		      SNDRV_PCM_RATE_176400 | \
		      SNDRV_PCM_RATE_192000)

/*
 * The codec isn't really big-endian or little-endian, since the I2S
 * interface requires data to be sent serially with the MSbit first.
 * However, to support BE and LE I2S devices, we specify both here.  That
 * way, ALSA will always match the bit patterns.
 */
#define STA32X_FORMATS \
	(SNDRV_PCM_FMTBIT_S16_LE  | SNDRV_PCM_FMTBIT_S16_BE  | \
	 SNDRV_PCM_FMTBIT_S18_3LE | SNDRV_PCM_FMTBIT_S18_3BE | \
	 SNDRV_PCM_FMTBIT_S20_3LE | SNDRV_PCM_FMTBIT_S20_3BE | \
	 SNDRV_PCM_FMTBIT_S24_3LE | SNDRV_PCM_FMTBIT_S24_3BE | \
	 SNDRV_PCM_FMTBIT_S24_LE  | SNDRV_PCM_FMTBIT_S24_BE  | \
	 SNDRV_PCM_FMTBIT_S32_LE  | SNDRV_PCM_FMTBIT_S32_BE)


/* regulator power supply names */
static const char *sta32x_supply_names[] = {
	"Vdda",	/* analog supply, 3.3VV */
	"Vdd3",	/* digital supply, 3.3V */
	"Vcc"	/* power amp spply, 10V - 36V */
};


/* codec private data */
struct sta32x_priv {
	u16 reg_cache[STA32X_REGISTER_COUNT];
	struct regulator_bulk_data supplies[ARRAY_SIZE(sta32x_supply_names)];
	struct snd_soc_codec *codec;

	unsigned int mclk;
	unsigned int format;
};


static const DECLARE_TLV_DB_SCALE(mvol_tlv, -12700, 50, 1);
static const DECLARE_TLV_DB_SCALE(chvol_tlv, -7950, 50, 1);
static const DECLARE_TLV_DB_SCALE(tone_tlv, -120, 200, 0);

static const char *sta32x_drc_ac[] = {
	"Anti-Clipping", "Dynamic Range Compression" };
static const char *sta32x_auto_eq_mode[] = {
	"User", "Preset", "Loudness" };
static const char *sta32x_auto_gc_mode[] = {
	"User", "AC no clipping", "AC limited clipping (10%)",
	"DRC nighttime listening mode" };
static const char *sta32x_auto_xo_mode[] = {
	"User", "80Hz", "100Hz", "120Hz", "140Hz", "160Hz", "180Hz", "200Hz",
	"220Hz", "240Hz", "260Hz", "280Hz", "300Hz", "320Hz", "340Hz", "360Hz" };
static const char *sta32x_preset_eq_mode[] = {
	"Flat", "Rock", "Soft Rock", "Jazz", "Classical", "Dance", "Pop", "Soft",
	"Hard", "Party", "Vocal", "Hip-Hop", "Dialog", "Bass-boost #1",
	"Bass-boost #2", "Bass-boost #3", "Loudness 1", "Loudness 2",
	"Loudness 3", "Loudness 4", "Loudness 5", "Loudness 6", "Loudness 7",
	"Loudness 8", "Loudness 9", "Loudness 10", "Loudness 11", "Loudness 12",
	"Loudness 13", "Loudness 14", "Loudness 15", "Loudness 16" };
static const char *sta32x_limiter_select[] = {
	"Limiter Disabled", "Limiter #1", "Limiter #2" };
#if 0
static const char *sta32x_pwm_output_mapping[] = {
	"Channel 1", "Channel 2", "Channel 3" };
#endif
static const char *sta32x_limiter_attack_rate[] = {
	"3.1584", "2.7072", "2.2560", "1.8048", "1.3536", "0.9024",
	"0.4512", "0.2256", "0.1504", "0.1123", "0.0902", "0.0752",
	"0.0645", "0.0564", "0.0501", "0.0451" };
static const char *sta32x_limiter_release_rate[] = {
	"0.5116", "0.1370", "0.0744", "0.0499", "0.0360", "0.0299",
	"0.0264", "0.0208", "0.0198", "0.0172", "0.0147", "0.0137",
	"0.0134", "0.0117", "0.0110", "0.0104" };
static const char *sta32x_limiter_ac_attack_thr[] = {
	"-12dB", "-10dB", "-8dB", "-6dB", "-4dB", "-2dB", "0dB", "+2dB",
	"+3dB", "+4dB", "+5dB", "+6dB", "+7dB", "+8dB", "+9dB", "+10dB" };
static const char *sta32x_limiter_ac_release_thr[] = {
	"-inf", "-29dB", "-20dB", "-16dB", "-14dB", "-12dB", "-10dB", "-8dB",
	"-7dB", "-6dB", "-5dB", "-4dB", "-3dB", "-2dB", "-1dB", "0dB" };
static const char *sta32x_limiter_drc_attack_thr[] = {
	"-31dB", "-29dB", "-27dB", "-25dB", "-23dB", "-21dB", "-19dB", "-17dB",
	"-16dB", "-15dB", "-14dB", "-13dB", "-12dB", "-10dB", "-7dB", "-4dB" };
static const char *sta32x_limiter_drc_release_thr[] = {
	"-inf", "-38dB", "-36dB", "-33dB", "-31dB", "-30dB", "-28dB", "-26dB",
	"-24dB", "-22dB", "-20dB", "-18dB", "-15dB", "-12dB", "-9dB", "-6dB" };


static const struct soc_enum sta32x_drc_ac_enum =
	SOC_ENUM_SINGLE(STA32X_CONFD, STA32X_CONFD_DRC_SHIFT,
			2, sta32x_drc_ac);
static const struct soc_enum sta32x_auto_eq_enum =
	SOC_ENUM_SINGLE(STA32X_AUTO1, STA32X_AUTO1_AMEQ_SHIFT,
			3, sta32x_auto_eq_mode);
static const struct soc_enum sta32x_auto_gc_enum =
	SOC_ENUM_SINGLE(STA32X_AUTO1, STA32X_AUTO1_AMGC_SHIFT,
			4, sta32x_auto_gc_mode);
static const struct soc_enum sta32x_auto_xo_enum =
	SOC_ENUM_SINGLE(STA32X_AUTO2, STA32X_AUTO2_XO_SHIFT,
			16, sta32x_auto_xo_mode);
static const struct soc_enum sta32x_preset_eq_enum =
	SOC_ENUM_SINGLE(STA32X_AUTO3, STA32X_AUTO3_PEQ_SHIFT,
			32, sta32x_preset_eq_mode);
static const struct soc_enum sta32x_limiter_ch1_enum =
	SOC_ENUM_SINGLE(STA32X_C1CFG, STA32X_CxCFG_LS_SHIFT,
			3, sta32x_limiter_select);
static const struct soc_enum sta32x_limiter_ch2_enum =
	SOC_ENUM_SINGLE(STA32X_C2CFG, STA32X_CxCFG_LS_SHIFT,
			3, sta32x_limiter_select);
static const struct soc_enum sta32x_limiter_ch3_enum =
	SOC_ENUM_SINGLE(STA32X_C3CFG, STA32X_CxCFG_LS_SHIFT,
			3, sta32x_limiter_select);
#if 0
static const struct soc_enum sta32x_pwm_out_ch1_enum =
	SOC_ENUM_SINGLE(STA32X_C1CFG, STA32X_CxCFG_OM_SHIFT,
			2, sta32x_pwm_output_mapping);
static const struct soc_enum sta32x_pwm_out_ch2_enum =
	SOC_ENUM_SINGLE(STA32X_C2CFG, STA32X_CxCFG_OM_SHIFT,
			2, sta32x_pwm_output_mapping);
static const struct soc_enum sta32x_pwm_out_ch3_enum =
	SOC_ENUM_SINGLE(STA32X_C3CFG, STA32X_CxCFG_OM_SHIFT,
			2, sta32x_pwm_output_mapping);
#endif
static const struct soc_enum sta32x_limiter1_attack_rate_enum =
	SOC_ENUM_SINGLE(STA32X_L1AR, STA32X_LxA_SHIFT,
			16, sta32x_limiter_attack_rate);
static const struct soc_enum sta32x_limiter2_attack_rate_enum =
	SOC_ENUM_SINGLE(STA32X_L2AR, STA32X_LxA_SHIFT,
			16, sta32x_limiter_attack_rate);
static const struct soc_enum sta32x_limiter1_release_rate_enum =
	SOC_ENUM_SINGLE(STA32X_L1AR, STA32X_LxR_SHIFT,
			16, sta32x_limiter_release_rate);
static const struct soc_enum sta32x_limiter2_release_rate_enum =
	SOC_ENUM_SINGLE(STA32X_L2AR, STA32X_LxR_SHIFT,
			16, sta32x_limiter_release_rate);
/* depending on mode, the attack/release thresholds have
 * two different enum definitions; provide both
 */
static const struct soc_enum sta32x_limiter1_ac_attack_thr_enum =
	SOC_ENUM_SINGLE(STA32X_L1ATRT, STA32X_LxA_SHIFT,
			16, sta32x_limiter_ac_attack_thr);
static const struct soc_enum sta32x_limiter2_ac_attack_thr_enum =
	SOC_ENUM_SINGLE(STA32X_L2ATRT, STA32X_LxA_SHIFT,
			16, sta32x_limiter_ac_attack_thr);
static const struct soc_enum sta32x_limiter1_ac_release_thr_enum =
	SOC_ENUM_SINGLE(STA32X_L1ATRT, STA32X_LxR_SHIFT,
			16, sta32x_limiter_ac_release_thr);
static const struct soc_enum sta32x_limiter2_ac_release_thr_enum =
	SOC_ENUM_SINGLE(STA32X_L2ATRT, STA32X_LxR_SHIFT,
			16, sta32x_limiter_ac_release_thr);
static const struct soc_enum sta32x_limiter1_drc_attack_thr_enum =
	SOC_ENUM_SINGLE(STA32X_L1ATRT, STA32X_LxA_SHIFT,
			16, sta32x_limiter_drc_attack_thr);
static const struct soc_enum sta32x_limiter2_drc_attack_thr_enum =
	SOC_ENUM_SINGLE(STA32X_L2ATRT, STA32X_LxA_SHIFT,
			16, sta32x_limiter_drc_attack_thr);
static const struct soc_enum sta32x_limiter1_drc_release_thr_enum =
	SOC_ENUM_SINGLE(STA32X_L1ATRT, STA32X_LxR_SHIFT,
			16, sta32x_limiter_drc_release_thr);
static const struct soc_enum sta32x_limiter2_drc_release_thr_enum =
	SOC_ENUM_SINGLE(STA32X_L2ATRT, STA32X_LxR_SHIFT,
			16, sta32x_limiter_drc_release_thr);

static const struct snd_kcontrol_new sta32x_snd_controls[] = {
SOC_SINGLE_TLV("Master Volume", STA32X_MVOL, 0, 0xff, 1, mvol_tlv),
SOC_SINGLE("Master Mute", STA32X_MMUTE, 0, 1, 0),
SOC_SINGLE("Ch1 Mute", STA32X_MMUTE, 1, 1, 0),
SOC_SINGLE("Ch2 Mute", STA32X_MMUTE, 2, 1, 0),
SOC_SINGLE("Ch3 Mute", STA32X_MMUTE, 3, 1, 0),
SOC_SINGLE_TLV("Ch1 Volume", STA32X_C1VOL, 0, 0xff, 1, chvol_tlv),
SOC_SINGLE_TLV("Ch2 Volume", STA32X_C2VOL, 0, 0xff, 1, chvol_tlv),
SOC_SINGLE_TLV("Ch3 Volume", STA32X_C3VOL, 0, 0xff, 1, chvol_tlv),
SOC_SINGLE("De-emphasis Filter Switch", STA32X_CONFD, STA32X_CONFD_DEMP_SHIFT, 1, 0),
SOC_ENUM("Compressor/Limiter Switch", sta32x_drc_ac_enum),
SOC_SINGLE("Miami Mode Switch", STA32X_CONFD, STA32X_CONFD_MME_SHIFT, 1, 0),
SOC_SINGLE("Zero Cross Switch", STA32X_CONFE, STA32X_CONFE_ZCE_SHIFT, 1, 0),
SOC_SINGLE("Soft Ramp Switch", STA32X_CONFE, STA32X_CONFE_SVE_SHIFT, 1, 0),
SOC_SINGLE("Auto-Mute Switch", STA32X_CONFF, STA32X_CONFF_IDE_SHIFT, 1, 0),
SOC_ENUM("Automode EQ", sta32x_auto_eq_enum),
SOC_ENUM("Automode GC", sta32x_auto_gc_enum),
SOC_ENUM("Automode XO", sta32x_auto_xo_enum),
SOC_ENUM("Preset EQ", sta32x_preset_eq_enum),
SOC_SINGLE("Ch1 Tone Control Bypass Switch", STA32X_C1CFG, STA32X_CxCFG_TCB_SHIFT, 1, 0),
SOC_SINGLE("Ch2 Tone Control Bypass Switch", STA32X_C2CFG, STA32X_CxCFG_TCB_SHIFT, 1, 0),
SOC_SINGLE("Ch1 EQ Bypass Switch", STA32X_C1CFG, STA32X_CxCFG_EQBP_SHIFT, 1, 0),
SOC_SINGLE("Ch2 EQ Bypass Switch", STA32X_C2CFG, STA32X_CxCFG_EQBP_SHIFT, 1, 0),
SOC_SINGLE("Ch1 Master Volume Bypass Switch", STA32X_C1CFG, STA32X_CxCFG_VBP_SHIFT, 1, 0),
SOC_SINGLE("Ch2 Master Volume Bypass Switch", STA32X_C1CFG, STA32X_CxCFG_VBP_SHIFT, 1, 0),
SOC_SINGLE("Ch3 Master Volume Bypass Switch", STA32X_C1CFG, STA32X_CxCFG_VBP_SHIFT, 1, 0),
SOC_ENUM("Ch1 Limiter Select", sta32x_limiter_ch1_enum),
SOC_ENUM("Ch2 Limiter Select", sta32x_limiter_ch2_enum),
SOC_ENUM("Ch3 Limiter Select", sta32x_limiter_ch3_enum),
#if 0
SOC_ENUM("Ch1 PWM Output Mapping", sta32x_pwm_out_ch1_enum),
SOC_ENUM("Ch2 PWM Output Mapping", sta32x_pwm_out_ch2_enum),
SOC_ENUM("Ch3 PWM Output Mapping", sta32x_pwm_out_ch3_enum),
#endif
SOC_SINGLE_TLV("Bass Tone Control", STA32X_TONE, STA32X_TONE_BTC_SHIFT, 15, 0, tone_tlv),
SOC_SINGLE_TLV("Treble Tone Control", STA32X_TONE, STA32X_TONE_TTC_SHIFT, 15, 0, tone_tlv),
SOC_ENUM("Limiter1 Attack Rate (dB/ms)", sta32x_limiter1_attack_rate_enum),
SOC_ENUM("Limiter2 Attack Rate (dB/ms)", sta32x_limiter2_attack_rate_enum),
SOC_ENUM("Limiter1 Release Rate (dB/ms)", sta32x_limiter1_release_rate_enum),
SOC_ENUM("Limiter2 Release Rate (dB/ms)", sta32x_limiter1_release_rate_enum),
SOC_ENUM("Limiter1 Attack Threshold (AC Mode)", sta32x_limiter1_ac_attack_thr_enum),
SOC_ENUM("Limiter2 Attack Threshold (AC Mode)", sta32x_limiter2_ac_attack_thr_enum),
SOC_ENUM("Limiter1 Release Threshold (AC Mode)", sta32x_limiter1_ac_release_thr_enum),
SOC_ENUM("Limiter2 Release Threshold (AC Mode)", sta32x_limiter2_ac_release_thr_enum),
SOC_ENUM("Limiter1 Attack Threshold (DRC Mode)", sta32x_limiter1_drc_attack_thr_enum),
SOC_ENUM("Limiter2 Attack Threshold (DRC Mode)", sta32x_limiter2_drc_attack_thr_enum),
SOC_ENUM("Limiter1 Release Threshold (DRC Mode)", sta32x_limiter1_drc_release_thr_enum),
SOC_ENUM("Limiter2 Release Threshold (DRC Mode)", sta32x_limiter2_drc_release_thr_enum),
};

static const struct snd_soc_dapm_widget sta32x_dapm_widgets[] = {
SND_SOC_DAPM_DAC("DAC", "Playback", SND_SOC_NOPM, 0, 0),
SND_SOC_DAPM_OUTPUT("LEFT"),
SND_SOC_DAPM_OUTPUT("RIGHT"),
SND_SOC_DAPM_OUTPUT("SUB"),
};

static const struct snd_soc_dapm_route intercon[] = {
	{ "LEFT", NULL, "DAC" },
	{ "RIGHT", NULL, "DAC" },
	{ "SUB", NULL, "DAC" },
};

/* MCLK interpolation ratio per fs */
static struct {
	int fs;
	int ir;
} interpolation_ratios[] = {
	{ 32000, 0 },
	{ 44100, 0 },
	{ 48000, 0 },
	{ 88200, 1 },
	{ 96000, 1 },
	{ 176400, 2 },
	{ 192000, 2 },
};

/* MCLK to fs clock ratios */
static struct {
	int ratio;
	int mcs;
} mclk_ratios[3][7] = {
	{ { 768, 0 }, { 512, 1 }, { 384, 2 }, { 256, 3 },
	  { 128, 4 }, { 576, 5 }, { 0, 0 } },
	{ { 384, 2 }, { 256, 3 }, { 192, 4 }, { 128, 5 }, {64, 0 }, { 0, 0 } },
	{ { 384, 2 }, { 256, 3 }, { 192, 4 }, { 128, 5 }, {64, 0 }, { 0, 0 } },
};


/**
 * sta32x_set_dai_sysclk - configure MCLK
 * @codec_dai: the codec DAI
 * @clk_id: the clock ID (ignored)
 * @freq: the MCLK input frequency
 * @dir: the clock direction (ignored)
 *
 * The value of MCLK is used to determine which sample rates are supported
 * by the STA32X, based on the mclk_ratios table.
 *
 * This function must be called by the machine driver's 'startup' function,
 * otherwise the list of supported sample rates will not be available in
 * time for ALSA.
 *
 * For setups with variable MCLKs, pass 0 as 'freq' argument. This will cause
 * theoretically possible sample rates to be enabled. Call it again with a
 * proper value set one the external clock is set (most probably you would do
 * that from a machine's driver 'hw_param' hook.
 */
static int sta32x_set_dai_sysclk(struct snd_soc_dai *codec_dai,
		int clk_id, unsigned int freq, int dir)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct sta32x_priv *sta32x = snd_soc_codec_get_drvdata(codec);
	int i, j, ir, fs;
	unsigned int rates = 0;
	unsigned int rate_min = -1;
	unsigned int rate_max = 0;

	pr_debug("mclk=%u\n", freq);
	sta32x->mclk = freq;

	if (sta32x->mclk) {
		for (i = 0; i < ARRAY_SIZE(interpolation_ratios); i++) {
			ir = interpolation_ratios[i].ir;
			fs = interpolation_ratios[i].fs;
			for (j = 0; mclk_ratios[ir][j].ratio; j++) {
				if (mclk_ratios[ir][j].ratio * fs == freq) {
					rates |= snd_pcm_rate_to_rate_bit(fs);
					if (fs < rate_min)
						rate_min = fs;
					if (fs > rate_max)
						rate_max = fs;
				}
			}
		}
		/* FIXME: soc should support a rate list */
		rates &= ~SNDRV_PCM_RATE_KNOT;

		if (!rates) {
			dev_err(codec->dev, "could not find a valid sample rate\n");
			return -EINVAL;
		}
	} else {
		/* enable all possible rates */
		rates = STA32X_RATES;
		rate_min = 32000;
		rate_max = 192000;
	}

	codec_dai->driver->playback.rates = rates;
	codec_dai->driver->playback.rate_min = rate_min;
	codec_dai->driver->playback.rate_max = rate_max;
	return 0;
}

/**
 * sta32x_set_dai_fmt - configure the codec for the selected audio format
 * @codec_dai: the codec DAI
 * @fmt: a SND_SOC_DAIFMT_x value indicating the data format
 *
 * This function takes a bitmask of SND_SOC_DAIFMT_x bits and programs the
 * codec accordingly.
 */
static int sta32x_set_dai_fmt(struct snd_soc_dai *codec_dai,
			      unsigned int fmt)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct sta32x_priv *sta32x = snd_soc_codec_get_drvdata(codec);
	u8 confb = snd_soc_read(codec, STA32X_CONFB);

	pr_debug("\n");
	confb &= ~(STA32X_CONFB_C1IM | STA32X_CONFB_C2IM);

	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBS_CFS:
		break;
	default:
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
	case SND_SOC_DAIFMT_RIGHT_J:
	case SND_SOC_DAIFMT_LEFT_J:
		sta32x->format = fmt & SND_SOC_DAIFMT_FORMAT_MASK;
		break;
	default:
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		confb |= STA32X_CONFB_C2IM;
		break;
	case SND_SOC_DAIFMT_NB_IF:
		confb |= STA32X_CONFB_C1IM;
		break;
	default:
		return -EINVAL;
	}

	snd_soc_write(codec, STA32X_CONFB, confb);
	return 0;
}

/**
 * sta32x_hw_params - program the STA32X with the given hardware parameters.
 * @substream: the audio stream
 * @params: the hardware parameters to set
 * @dai: the SOC DAI (ignored)
 *
 * This function programs the hardware with the values provided.
 * Specifically, the sample rate and the data format.
 */
static int sta32x_hw_params(struct snd_pcm_substream *substream,
			    struct snd_pcm_hw_params *params,
			    struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec *codec = rtd->codec;
	struct sta32x_priv *sta32x = snd_soc_codec_get_drvdata(codec);
	unsigned int rate;
	int i, mcs = -1, ir = -1;
	u8 confa, confb;

	rate = params_rate(params);
	pr_debug("rate: %u\n", rate);
	for (i = 0; i < ARRAY_SIZE(interpolation_ratios); i++)
		if (interpolation_ratios[i].fs == rate)
			ir = interpolation_ratios[i].ir;
	if (ir < 0)
		return -EINVAL;
	for (i = 0; mclk_ratios[ir][i].ratio; i++)
		if (mclk_ratios[ir][i].ratio * rate == sta32x->mclk)
			mcs = mclk_ratios[ir][i].mcs;
	if (mcs < 0)
		return -EINVAL;

	confa = snd_soc_read(codec, STA32X_CONFA);
	confa &= ~(STA32X_CONFA_MCS_MASK | STA32X_CONFA_IR_MASK);
	confa |= (ir << STA32X_CONFA_IR_SHIFT) | (mcs << STA32X_CONFA_MCS_SHIFT);

	confb = snd_soc_read(codec, STA32X_CONFB);
	confb &= ~(STA32X_CONFB_SAI_MASK | STA32X_CONFB_SAIFB);
	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S24_LE:
	case SNDRV_PCM_FORMAT_S24_BE:
	case SNDRV_PCM_FORMAT_S24_3LE:
	case SNDRV_PCM_FORMAT_S24_3BE:
		pr_debug("24bit\n");
	case SNDRV_PCM_FORMAT_S32_LE:
	case SNDRV_PCM_FORMAT_S32_BE:
		pr_debug("24bit or 32bit\n");
		if (sta32x->format == SND_SOC_DAIFMT_I2S)
			confb |= 0x0;
		else if (sta32x->format == SND_SOC_DAIFMT_LEFT_J)
			confb |= 0x1;
		else if (sta32x->format == SND_SOC_DAIFMT_RIGHT_J)
			confb |= 0x2;
		break;
	case SNDRV_PCM_FORMAT_S20_3LE:
	case SNDRV_PCM_FORMAT_S20_3BE:
		pr_debug("20bit\n");
		if (sta32x->format == SND_SOC_DAIFMT_I2S)
			confb |= 0x4;
		else if (sta32x->format == SND_SOC_DAIFMT_LEFT_J)
			confb |= 0x5;
		else if (sta32x->format == SND_SOC_DAIFMT_RIGHT_J)
			confb |= 0x6;
		break;
	case SNDRV_PCM_FORMAT_S18_3LE:
	case SNDRV_PCM_FORMAT_S18_3BE:
		pr_debug("18bit\n");
		if (sta32x->format == SND_SOC_DAIFMT_I2S)
			confb |= 0x8;
		else if (sta32x->format == SND_SOC_DAIFMT_LEFT_J)
			confb |= 0x9;
		else if (sta32x->format == SND_SOC_DAIFMT_RIGHT_J)
			confb |= 0xa;
		break;
	case SNDRV_PCM_FORMAT_S16_LE:
	case SNDRV_PCM_FORMAT_S16_BE:
		pr_debug("16bit\n");
		if (sta32x->format == SND_SOC_DAIFMT_I2S)
			confb |= 0x0;
		else if (sta32x->format == SND_SOC_DAIFMT_LEFT_J)
			confb |= 0xd;
		else if (sta32x->format == SND_SOC_DAIFMT_RIGHT_J)
			confb |= 0xe;
		break;
	default:
		return -EINVAL;
	}

	snd_soc_write(codec, STA32X_CONFA, confa);
	snd_soc_write(codec, STA32X_CONFB, confb);
	return 0;
}

/**
 * sta32x_set_bias_level - DAPM callback
 * @codec: the codec device
 * @level: DAPM power level
 *
 * This is called by ALSA to put the codec into low power mode
 * or to wake it up.  If the codec is powered off completely
 * all registers must be restored after power on.
 */
static int sta32x_set_bias_level(struct snd_soc_codec *codec,
				 enum snd_soc_bias_level level)
{
	int ret, i;
	struct sta32x_priv *sta32x = snd_soc_codec_get_drvdata(codec);

	pr_debug("level = %d\n", level);
	switch (level) {
	case SND_SOC_BIAS_ON:
		break;

	case SND_SOC_BIAS_PREPARE:
		/* Full power on */
		snd_soc_update_bits(codec, STA32X_CONFF,
				    STA32X_CONFF_PWDN | STA32X_CONFF_EAPD,
				    STA32X_CONFF_PWDN | STA32X_CONFF_EAPD);
		break;

	case SND_SOC_BIAS_STANDBY:
		if (codec->dapm.bias_level == SND_SOC_BIAS_OFF) {
			ret = regulator_bulk_enable(ARRAY_SIZE(sta32x->supplies),
						    sta32x->supplies);
			if (ret != 0) {
				dev_err(codec->dev,
					"Failed to enable supplies: %d\n", ret);
				return ret;
			}

			/* Sync back default/cached values */
			for (i = 0; i < STA32X_REGISTER_COUNT; i++)
				snd_soc_write(codec, i, sta32x->reg_cache[i]);
		}

		/* Power up to mute */
		//FIXME
		snd_soc_update_bits(codec, STA32X_CONFF,
				    STA32X_CONFF_PWDN | STA32X_CONFF_EAPD,
				    STA32X_CONFF_PWDN | STA32X_CONFF_EAPD);

		break;

	case SND_SOC_BIAS_OFF:
		/* The chip runs through the power down sequence for us. */
		snd_soc_update_bits(codec, STA32X_CONFF,
				    STA32X_CONFF_PWDN | STA32X_CONFF_EAPD,
				    STA32X_CONFF_PWDN);
		msleep(300);

		regulator_bulk_disable(ARRAY_SIZE(sta32x->supplies),
				       sta32x->supplies);
		break;
	}
	codec->dapm.bias_level = level;
	return 0;
}

static struct snd_soc_dai_ops sta32x_dai_ops = {
	.hw_params	= sta32x_hw_params,
	.set_sysclk	= sta32x_set_dai_sysclk,
	.set_fmt	= sta32x_set_dai_fmt,
};

static struct snd_soc_dai_driver sta32x_dai = {
	.name = "STA32X",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 2,
		.channels_max = 2,
		.rates = STA32X_RATES,
		.formats = STA32X_FORMATS,
	},
	.ops = &sta32x_dai_ops,
};

#ifdef CONFIG_PM
static int sta32x_suspend(struct snd_soc_codec *codec, pm_message_t state)
{
	sta32x_set_bias_level(codec, SND_SOC_BIAS_OFF);
	return 0;
}

static int sta32x_resume(struct snd_soc_codec *codec)
{
	sta32x_set_bias_level(codec, SND_SOC_BIAS_STANDBY);
	return 0;
}
#else
#define sta32x_suspend NULL
#define sta32x_resume NULL
#endif

static int sta32x_probe(struct snd_soc_codec *codec)
{
	struct sta32x_priv *sta32x = snd_soc_codec_get_drvdata(codec);
	struct snd_soc_dapm_context *dapm = &codec->dapm;
	int i, ret = 0;

	sta32x->codec = codec;

	/* Tell ASoC what kind of I/O to use to read the registers.  ASoC will
	 * then do the I2C transactions itself.
	 */
	ret = snd_soc_codec_set_cache_io(codec, 8, 8, SND_SOC_I2C);
	if (ret < 0) {
		dev_err(codec->dev, "failed to set cache I/O (ret=%i)\n", ret);
		return ret;
	}

	snd_soc_add_controls(codec, sta32x_snd_controls,
			     ARRAY_SIZE(sta32x_snd_controls));

	snd_soc_dapm_new_controls(dapm, sta32x_dapm_widgets,
				  ARRAY_SIZE(sta32x_dapm_widgets));
	snd_soc_dapm_add_routes(dapm, intercon, ARRAY_SIZE(intercon));

	/* read reg reset values into cache */
	for (i = 0; i < STA32X_REGISTER_COUNT; i++)
		sta32x->reg_cache[i] = snd_soc_read(codec, i);

	/* FIXME enable thermal warning adjustment and recovery  */
	sta32x->reg_cache[STA32X_CONFA] &= ~(STA32X_CONFA_TWAB | STA32X_CONFA_TWRB);
	/* FIXME select 2.1 mode  */
	sta32x->reg_cache[STA32X_CONFF] &= ~STA32X_CONFF_OCFG_MASK;
	sta32x->reg_cache[STA32X_CONFF] |= 1 << STA32X_CONFF_OCFG_SHIFT;
	/* FIXME channel to output mapping */
	sta32x->reg_cache[STA32X_C1CFG] &= ~STA32X_CxCFG_OM_MASK;
	sta32x->reg_cache[STA32X_C1CFG] |= 0 << STA32X_CxCFG_OM_SHIFT;
	sta32x->reg_cache[STA32X_C2CFG] &= ~STA32X_CxCFG_OM_MASK;
	sta32x->reg_cache[STA32X_C2CFG] |= 1 << STA32X_CxCFG_OM_SHIFT;
	sta32x->reg_cache[STA32X_C3CFG] &= ~STA32X_CxCFG_OM_MASK;
	sta32x->reg_cache[STA32X_C3CFG] |= 2 << STA32X_CxCFG_OM_SHIFT;

	sta32x_set_bias_level(codec, SND_SOC_BIAS_STANDBY);

	/* Bias level configuration will have done an extra enable */
	regulator_bulk_disable(ARRAY_SIZE(sta32x->supplies), sta32x->supplies);

	/* regulators */
	for (i = 0; i < ARRAY_SIZE(sta32x->supplies); i++)
		sta32x->supplies[i].supply = sta32x_supply_names[i];

	ret = regulator_bulk_get(codec->dev, ARRAY_SIZE(sta32x->supplies),
				 sta32x->supplies);
	if (ret != 0) {
		dev_err(codec->dev, "Failed to request supplies: %d\n", ret);
		goto err;
	}

	ret = regulator_bulk_enable(ARRAY_SIZE(sta32x->supplies),
				    sta32x->supplies);
	if (ret != 0) {
		dev_err(codec->dev, "Failed to enable supplies: %d\n", ret);
		goto err_get;
	}

	return 0;

err_get:
	regulator_bulk_free(ARRAY_SIZE(sta32x->supplies), sta32x->supplies);
err:
	return ret;
}

static int sta32x_remove(struct snd_soc_codec *codec)
{
	struct sta32x_priv *sta32x = snd_soc_codec_get_drvdata(codec);

	regulator_bulk_disable(ARRAY_SIZE(sta32x->supplies), sta32x->supplies);
	regulator_bulk_free(ARRAY_SIZE(sta32x->supplies), sta32x->supplies);

	return 0;
}

static const struct snd_soc_codec_driver sta32x_codec = {
	.probe =		sta32x_probe,
	.remove =		sta32x_remove,
	.suspend =		sta32x_suspend,
	.resume =		sta32x_resume,
	.reg_cache_size =	STA32X_REGISTER_COUNT,
	.reg_word_size =	sizeof(u8),
	.set_bias_level =	sta32x_set_bias_level,
};

static __devinit int sta32x_i2c_probe(struct i2c_client *i2c,
				      const struct i2c_device_id *id)
{
	struct sta32x_priv *sta32x;
	int ret;

	sta32x = kzalloc(sizeof(struct sta32x_priv), GFP_KERNEL);
	if (!sta32x)
		return -ENOMEM;

	i2c_set_clientdata(i2c, sta32x);

	ret = snd_soc_register_codec(&i2c->dev, &sta32x_codec, &sta32x_dai, 1);
	if (ret != 0) {
		dev_err(&i2c->dev, "Failed to register codec (%d)\n", ret);
		return ret;
	}

	return 0;
}

static __devexit int sta32x_i2c_remove(struct i2c_client *client)
{
	struct sta32x_priv *sta32x = i2c_get_clientdata(client);
	struct snd_soc_codec *codec = sta32x->codec;

	if (codec)
		sta32x_set_bias_level(codec, SND_SOC_BIAS_OFF);

	regulator_bulk_free(ARRAY_SIZE(sta32x->supplies), sta32x->supplies);

	if (codec) {
		snd_soc_unregister_codec(&client->dev);
		snd_soc_codec_set_drvdata(codec, NULL);
	}

	kfree(sta32x);
	return 0;
}

static const struct i2c_device_id sta32x_i2c_id[] = {
	{ "sta32x", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, sta32x_i2c_id);

static struct i2c_driver sta32x_i2c_driver = {
	.driver = {
		.name = "sta32x",
		.owner = THIS_MODULE,
	},
	.probe =    sta32x_i2c_probe,
	.remove =   __devexit_p(sta32x_i2c_remove),
	.id_table = sta32x_i2c_id,
};

static int __init sta32x_init(void)
{
	return i2c_add_driver(&sta32x_i2c_driver);
}
module_init(sta32x_init);

static void __exit sta32x_exit(void)
{
	i2c_del_driver(&sta32x_i2c_driver);
}
module_exit(sta32x_exit);

MODULE_DESCRIPTION("ASoC STA32X driver");
MODULE_AUTHOR("Johannes Stezenbach <js@sig21.net>");
MODULE_LICENSE("GPL");
