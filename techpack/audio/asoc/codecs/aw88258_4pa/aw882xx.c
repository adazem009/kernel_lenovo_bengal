/*
 * aw882xx.c   aw882xx codec module
 *
 * Version: 4pa_v0.1.20
 *
 * Copyright (c) 2020 AWINIC Technology CO., LTD
 *
 *  Author: Nick Li <liweilei@awinic.com.cn>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

//#define DEBUG
#include <linux/module.h>
#include <linux/i2c.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/firmware.h>
#include <linux/debugfs.h>
#include <linux/version.h>
#include <linux/workqueue.h>
#include <linux/syscalls.h>
#include <sound/control.h>
#include <linux/uaccess.h>

#include "aw882xx.h"
#include "aw_log.h"
#include "aw882xx_reg.h"

#define AW882XX_VERSION "4pa_v0.1.22"
#define AW882XX_I2C_NAME "aw882xx_smartpa"

#define AW882XX_ACF_FILE  "aw882xx_acf.bin"

#define AW_READ_CHIPID_RETRIES		5	/* 5 times */
#define AW_READ_CHIPID_RETRY_DELAY	5	/* 5 ms */


static unsigned int g_aw882xx_dev_cnt = 0;
static LIST_HEAD(g_aw882xx_device_list);
static DEFINE_MUTEX(g_aw882xx_mutex_lock);
static aw_acf_t *g_awinic_acf = NULL;


static const char *const aw882xx_switch[] = {"Disable", "Enable"};
static const char *const aw882xx_spin[] = {"spin_0", "spin_90", "spin_180", "spin_270"};


/******************************************************
 *
 * aw882xx distinguish between codecs and components by version
 *
 ******************************************************/
#ifdef AW_KERNEL_VER_OVER_4_19_1
static struct aw_componet_codec_ops aw_componet_codec_ops = {
	.kcontrol_codec = snd_soc_kcontrol_component,
	.codec_get_drvdata = snd_soc_component_get_drvdata,
	.add_codec_controls = snd_soc_add_component_controls,
	.unregister_codec = snd_soc_unregister_component,
	.register_codec = snd_soc_register_component,
};
#else
static struct aw_componet_codec_ops aw_componet_codec_ops = {
	.kcontrol_codec = snd_soc_kcontrol_codec,
	.codec_get_drvdata = snd_soc_codec_get_drvdata,
	.add_codec_controls = snd_soc_add_codec_controls,
	.unregister_codec = snd_soc_unregister_codec,
	.register_codec = snd_soc_register_codec,
};
#endif

static aw_snd_soc_codec_t *aw_get_codec(struct snd_soc_dai *dai)
{
#ifdef AW_KERNEL_VER_OVER_4_19_1
	return dai->component;
#else
	return dai->codec;
#endif
}


/******************************************************
 *
 * aw882xx i2c write/read
 *
 ******************************************************/
static int aw882xx_i2c_writes(struct aw882xx *aw882xx,
	unsigned char reg_addr, unsigned char *buf, unsigned int len)
{
	int ret = -1;
	unsigned char *data;

	data = kmalloc(len+1, GFP_KERNEL);
	if (data == NULL) {
		aw_dev_err(aw882xx->dev, "%s: can not allocate memory\n",
			__func__);
		return -ENOMEM;
	}

	data[0] = reg_addr;
	memcpy(&data[1], buf, len);

	ret = i2c_master_send(aw882xx->i2c, data, len+1);
	if (ret < 0)
		aw_dev_err(aw882xx->dev,
			"%s: i2c master send error\n", __func__);

	kfree(data);

	return ret;
}

static int aw882xx_i2c_reads(struct aw882xx *aw882xx,
	unsigned char reg_addr, unsigned char *data_buf, unsigned int data_len)
{
	int ret;
	struct i2c_msg msg[] = {
		[0] = {
			.addr = aw882xx->i2c->addr,
			.flags = 0,
			.len = sizeof(uint8_t),
			.buf = &reg_addr,
			},
		[1] = {
			.addr = aw882xx->i2c->addr,
			.flags = I2C_M_RD,
			.len = data_len,
			.buf = data_buf,
			},
	};

	ret = i2c_transfer(aw882xx->i2c->adapter, msg, ARRAY_SIZE(msg));
	if (ret < 0) {
		aw_dev_err(aw882xx->dev, "%s: transfer failed.", __func__);
		return ret;
	} else if (ret != AW882XX_I2C_READ_MSG_NUM) {
		aw_dev_err(aw882xx->dev, "%s: transfer failed(size error).\n",
				__func__);
		return -ENXIO;
	}

	return 0;
}

int aw882xx_i2c_write(struct aw882xx *aw882xx,
	unsigned char reg_addr, unsigned int reg_data)
{
	int ret = -1;
	unsigned char cnt = 0;
	unsigned char buf[2];

	buf[0] = (reg_data&0xff00)>>8;
	buf[1] = (reg_data&0x00ff)>>0;

	while (cnt < AW_I2C_RETRIES) {
		ret = aw882xx_i2c_writes(aw882xx, reg_addr, buf, 2);
		if (ret < 0)
			aw_dev_err(aw882xx->dev, "%s: i2c_write cnt=%d error=%d\n",
				__func__, cnt, ret);
		else
			break;
		cnt++;
	}

	return ret;
}

int aw882xx_i2c_read(struct aw882xx *aw882xx,
	unsigned char reg_addr, unsigned int *reg_data)
{
	int ret = -1;
	unsigned char cnt = 0;
	unsigned char buf[2];

	while (cnt < AW_I2C_RETRIES) {
		ret = aw882xx_i2c_reads(aw882xx, reg_addr, buf, 2);
		if (ret < 0) {
			aw_dev_err(aw882xx->dev, "%s: i2c_read cnt=%d error=%d\n",
				__func__, cnt, ret);
		} else {
			*reg_data = (buf[0]<<8) | (buf[1]<<0);
			break;
		}
		cnt++;
	}

	return ret;
}

static int aw882xx_i2c_write_bits(struct aw882xx *aw882xx,
	unsigned char reg_addr, unsigned int mask, unsigned int reg_data)
{
	int ret = -1;
	unsigned int reg_val = 0;

	ret = aw882xx_i2c_read(aw882xx, reg_addr, &reg_val);
	if (ret < 0) {
		aw_dev_err(aw882xx->dev,
			"%s: i2c read error, ret=%d\n", __func__, ret);
		return ret;
	}
	reg_val &= mask;
	reg_val |= reg_data;
	ret = aw882xx_i2c_write(aw882xx, reg_addr, reg_val);
	if (ret < 0) {
		aw_dev_err(aw882xx->dev,
			"%s: i2c read error, ret=%d\n", __func__, ret);
		return ret;
	}

	return 0;
}

int aw882xx_dev_i2c_write_bits(struct aw_device *aw_dev,
	unsigned char reg_addr, unsigned int mask, unsigned int reg_data)
{
	struct aw882xx *aw882xx = (struct aw882xx *)aw_dev->private_data;

	return aw882xx_i2c_write_bits(aw882xx, reg_addr, mask, reg_data);
}

int aw882xx_dev_i2c_write(struct aw_device *aw_dev,
	unsigned char reg_addr, unsigned int reg_data)
{
	struct aw882xx *aw882xx = (struct aw882xx *)aw_dev->private_data;

	return aw882xx_i2c_write(aw882xx, reg_addr, reg_data);
}

int aw882xx_dev_i2c_read(struct aw_device *aw_dev,
	unsigned char reg_addr, unsigned int *reg_data)
{
	struct aw882xx *aw882xx = (struct aw882xx *)aw_dev->private_data;

	return aw882xx_i2c_read(aw882xx, reg_addr, reg_data);
}

static int aw882xx_set_volume(struct aw882xx *aw882xx, unsigned int value)
{
	unsigned int reg_value = 0;
	//[7 : 4]: -6DB ; [3 : 0]: -0.5DB
	unsigned int real_value = ((value / AW882XX_VOLUME_STEP_DB) << 4) + (value % AW882XX_VOLUME_STEP_DB);

	/* cal real value */
	aw882xx_i2c_read(aw882xx, AW882XX_HAGCCFG4_REG, &reg_value);

	aw_dev_dbg(aw882xx->dev, "%s : value %d , 0x%x", __func__, value, real_value);

	//15 : 8] volume
	real_value = (real_value << 8) | (reg_value & 0x00ff);

	/* write value */
	aw882xx_i2c_write(aw882xx, AW882XX_HAGCCFG4_REG, real_value);
	return 0;
}

static int aw882xx_get_volume(struct aw882xx *aw882xx, unsigned int* value)
{
	unsigned int reg_value = 0;
	unsigned int real_value = 0;

	/* read value */
	aw882xx_i2c_read(aw882xx, AW882XX_HAGCCFG4_REG, &reg_value);

	//[15 : 8] volume
	real_value = reg_value >> 8;

	//[7 : 4]: -6DB ; [3 : 0]: 0.5DB  real_value = value * 2 : 0.5db --> 1
	real_value = (real_value >> 4) * AW882XX_VOLUME_STEP_DB + (real_value & 0x0f);
	*value = real_value;

	return 0;
}

static int aw882xx_dev_set_volume(struct aw_device *aw_dev, unsigned int value)
{
	struct aw882xx *aw882xx = (struct aw882xx *)aw_dev->private_data;
	return aw882xx_set_volume(aw882xx, value);
}

static int aw882xx_dev_get_volume(struct aw_device *aw_dev, unsigned int *value)
{
	struct aw882xx *aw882xx = (struct aw882xx *)aw_dev->private_data;
	return aw882xx_get_volume(aw882xx, value);
}

static void *aw882xx_devm_kstrdup(struct device *dev, char *buf)
{
	char *str;

	str = devm_kzalloc(dev, strlen(buf) + 1, GFP_KERNEL);
	if (!str)
		return str;

	memcpy(str, buf, strlen(buf));
	return str;
}

/*****************************************************
 *
 * snd_soc_dai_driver ops
 *
 *****************************************************/
static int aw882xx_startup(struct snd_pcm_substream *substream,
	struct snd_soc_dai *dai)
{
	aw_snd_soc_codec_t *codec = aw_get_codec(dai);
	struct aw882xx *aw882xx =
		aw_componet_codec_ops.codec_get_drvdata(codec);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		aw_dev_info(aw882xx->dev, "%s: playback enter\n", __func__);
		mutex_lock(&aw882xx->lock);
		aw_dev_load_fw(aw882xx->aw_pa);
		mutex_unlock(&aw882xx->lock);
	} else {
		aw_dev_info(aw882xx->dev, "%s: capture enter\n", __func__);
	}

	return 0;
}

static int aw882xx_set_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	/*struct aw882xx *aw882xx = aw_snd_soc_codec_get_drvdata(dai->codec);*/
	aw_snd_soc_codec_t *codec = aw_get_codec(dai);

	aw_dev_info(codec->dev, "%s: fmt=0x%x\n", __func__, fmt);

	/* Supported mode: regular I2S, slave, or PDM */
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		if ((fmt & SND_SOC_DAIFMT_MASTER_MASK) !=
			SND_SOC_DAIFMT_CBS_CFS) {
			aw_dev_err(codec->dev, "%s: invalid codec master mode\n",
				__func__);
			return -EINVAL;
		}
		break;
	default:
		aw_dev_err(codec->dev, "%s: unsupported DAI format %d\n",
			__func__, fmt & SND_SOC_DAIFMT_FORMAT_MASK);
		return -EINVAL;
	}
	return 0;
}

static int aw882xx_set_dai_sysclk(struct snd_soc_dai *dai,
	int clk_id, unsigned int freq, int dir)
{
	aw_snd_soc_codec_t *codec = aw_get_codec(dai);
	struct aw882xx *aw882xx =
		aw_componet_codec_ops.codec_get_drvdata(codec);

	aw_dev_info(aw882xx->dev, "%s: freq=%d\n", __func__, freq);

	aw882xx->sysclk = freq;
	return 0;
}

static int aw882xx_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params,
	struct snd_soc_dai *dai)
{
	aw_snd_soc_codec_t *codec = aw_get_codec(dai);
	struct aw882xx *aw882xx =
		aw_componet_codec_ops.codec_get_drvdata(codec);
	unsigned int rate = 0;
	uint32_t cco_mux_value;
	int reg_value = 0;
	int width = 0;

return 0;

	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
		aw_dev_dbg(aw882xx->dev, "%s: requested rate: %d, sample size: %d\n",
				__func__, rate,
				snd_pcm_format_width(params_format(params)));
		return 0;
	}
	/* get rate param */
	aw882xx->rate = rate = params_rate(params);
	aw_dev_dbg(aw882xx->dev, "%s: requested rate: %d, sample size: %d\n",
		__func__, rate, snd_pcm_format_width(params_format(params)));

	/* match rate */
	switch (rate) {
	case 8000:
		reg_value = AW882XX_I2SSR_8KHZ_VALUE;
		cco_mux_value = AW882XX_I2S_CCO_MUX_8_16_32KHZ_VALUE;
		break;
	case 16000:
		reg_value = AW882XX_I2SSR_16KHZ_VALUE;
		cco_mux_value = AW882XX_I2S_CCO_MUX_8_16_32KHZ_VALUE;
		break;
	case 32000:
		reg_value = AW882XX_I2SSR_32KHZ_VALUE;
		cco_mux_value = AW882XX_I2S_CCO_MUX_8_16_32KHZ_VALUE;
		break;
	case 44100:
		reg_value = AW882XX_I2SSR_44P1KHZ_VALUE;
		cco_mux_value = AW882XX_I2S_CCO_MUX_EXC_8_16_32KHZ_VALUE;
		break;
	case 48000:
		reg_value = AW882XX_I2SSR_48KHZ_VALUE;
		cco_mux_value = AW882XX_I2S_CCO_MUX_EXC_8_16_32KHZ_VALUE;
		break;
	case 96000:
		reg_value = AW882XX_I2SSR_96KHZ_VALUE;
		cco_mux_value = AW882XX_I2S_CCO_MUX_EXC_8_16_32KHZ_VALUE;
		break;
	case 192000:
		reg_value = AW882XX_I2SSR_192KHZ_VALUE;
		cco_mux_value = AW882XX_I2S_CCO_MUX_EXC_8_16_32KHZ_VALUE;
		break;
	default:
		reg_value = AW882XX_I2SSR_48KHZ_VALUE;
		cco_mux_value = AW882XX_I2S_CCO_MUX_EXC_8_16_32KHZ_VALUE;
		aw_dev_err(aw882xx->dev, "%s: rate can not support\n",
			__func__);
		break;
	}
	aw882xx_i2c_write_bits(aw882xx, AW882XX_PLLCTRL1_REG,
				AW882XX_I2S_CCO_MUX_MASK, cco_mux_value);

	/* set chip rate */
	if (-1 != reg_value) {
		aw882xx_i2c_write_bits(aw882xx, AW882XX_I2SCTRL_REG,
				AW882XX_I2SSR_MASK, reg_value);
	}


	/* get bit width */
	width = params_width(params);
	aw_dev_dbg(aw882xx->dev, "%s: width = %d\n", __func__, width);
	switch (width) {
	case 16:
		reg_value = AW882XX_I2SFS_16_BITS_VALUE;
		break;
	case 20:
		reg_value = AW882XX_I2SFS_20_BITS_VALUE;
		break;
	case 24:
		reg_value = AW882XX_I2SFS_24_BITS_VALUE;
		break;
	case 32:
		reg_value = AW882XX_I2SFS_32_BITS_VALUE;
		break;
	default:
		reg_value = AW882XX_I2SFS_16_BITS_VALUE;
		aw_dev_err(aw882xx->dev,
			"%s: width can not support\n", __func__);
		break;
	}

	/* get width */
	if (-1 != reg_value) {
		aw882xx_i2c_write_bits(aw882xx, AW882XX_I2SCTRL_REG,
				AW882XX_I2SFS_MASK, reg_value);
	}

	return 0;
}

static void aw882xx_shutdown(struct snd_pcm_substream *substream,
	struct snd_soc_dai *dai)
{
	aw_snd_soc_codec_t *codec = aw_get_codec(dai);
	struct aw882xx *aw882xx =
		aw_componet_codec_ops.codec_get_drvdata(codec);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		aw882xx->rate = 0;
		aw_dev_info(aw882xx->dev, "%s: stream playback\n", __func__);
	} else {
		aw_dev_info(aw882xx->dev, "%s: stream capture\n", __func__);
	}
}

static void aw882xx_start(struct aw882xx *aw882xx)
{
	int ret = 0;

	if (aw882xx->fw_status == AW_DEV_FW_OK) {
		if (aw882xx->allow_pw == false) {
			aw_dev_info(aw882xx->dev, "%s: dev can not allow power \n", __func__);
			return;
		}
		mutex_lock(&aw882xx->lock);
		ret = aw_device_start(aw882xx->aw_pa, AW_START_TYPE_STREAM);
		if (ret) {
			aw_dev_info(aw882xx->dev, "%s: start up failded \n", __func__);
		} else {
			aw_dev_info(aw882xx->dev, "%s: start up success \n", __func__);
		}
		mutex_unlock(&aw882xx->lock);
	} else {
		aw_dev_info(aw882xx->dev, "%s: dev acf load failed \n", __func__);
	}
}

static int aw882xx_mute(struct snd_soc_dai *dai, int mute, int stream)
{
	aw_snd_soc_codec_t *codec = aw_get_codec(dai);
	struct aw882xx *aw882xx =
		aw_componet_codec_ops.codec_get_drvdata(codec);

	aw_dev_info(aw882xx->dev, "%s: mute state=%d\n", __func__, mute);

	if (stream != SNDRV_PCM_STREAM_PLAYBACK) {
		aw_dev_info(aw882xx->dev, "%s: capture\n", __func__);
		return 0;
	}

	if (mute) {
		aw882xx->pstream = 0;
		cancel_delayed_work_sync(&aw882xx->start_work);
		mutex_lock(&aw882xx->lock);
		aw_device_stop(aw882xx->aw_pa);
		mutex_unlock(&aw882xx->lock);
	} else {
		aw882xx->pstream = 1;
		aw882xx_start(aw882xx);
	}

	return 0;
}

static const struct snd_soc_dai_ops aw882xx_dai_ops = {
	.startup = aw882xx_startup,
	.set_fmt = aw882xx_set_fmt,
	.set_sysclk = aw882xx_set_dai_sysclk,
	.hw_params = aw882xx_hw_params,
	.mute_stream = aw882xx_mute,
	.shutdown = aw882xx_shutdown,
};

/*****************************************************
 *
 * snd_soc_codec_driver | snd_soc_component_driver|
 *
 *****************************************************/
 #if 0
static int aw882xx_profile_info(struct snd_kcontrol *kcontrol,
			 struct snd_ctl_elem_info *uinfo)
{
	int count, ret;
	char *name = NULL;
	aw_snd_soc_codec_t *codec =
		aw_componet_codec_ops.kcontrol_codec(kcontrol);
	struct aw882xx *aw882xx =
		aw_componet_codec_ops.codec_get_drvdata(codec);

	uinfo->type = SNDRV_CTL_ELEM_TYPE_ENUMERATED;
	uinfo->count = 1;

	count = aw_dev_get_profile_count(aw882xx->aw_pa);
	if (count <= 0) {
		uinfo->value.enumerated.items = 0;
		aw_dev_err(aw882xx->dev, "get count[%d] failed \n", count);
		return 0;
	}

	uinfo->value.enumerated.items = count;

	if (uinfo->value.enumerated.item > count) {
		uinfo->value.enumerated.item = count -1;
	}

	name = uinfo->value.enumerated.name;
	count = uinfo->value.enumerated.item;
	ret = aw_dev_get_profile_name(aw882xx->aw_pa, name, count);
	if (ret) {
		strcpy(uinfo->value.enumerated.name, "null");
		return 0;
	}

	return 0;
}

static int aw882xx_profile_get(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	aw_snd_soc_codec_t *codec =
		aw_componet_codec_ops.kcontrol_codec(kcontrol);
	struct aw882xx *aw882xx =
		aw_componet_codec_ops.codec_get_drvdata(codec);

	ucontrol->value.integer.value[0] = aw_dev_get_profile_index(aw882xx->aw_pa);
	aw_dev_dbg(codec->dev, "%s: profile index [%d]", __func__, aw_dev_get_profile_index(aw882xx->aw_pa));
	return 0;

}

static int aw882xx_profile_set(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	aw_snd_soc_codec_t *codec =
		aw_componet_codec_ops.kcontrol_codec(kcontrol);
	struct aw882xx *aw882xx =
		aw_componet_codec_ops.codec_get_drvdata(codec);
	int ret;
	int cur_index;

	if (aw882xx->dbg_en_prof == false) {
		aw_dev_info(codec->dev, "%s: profile close \n", __func__);
		return 0;
	}

	/* check value valid */
	ret = aw_dev_check_profile_index(aw882xx->aw_pa, ucontrol->value.integer.value[0]);
	if (ret) {
		aw_dev_info(codec->dev, "%s: unsupported index %d \n",
			__func__, (int)ucontrol->value.integer.value[0]);
		return 0;
	}

	/*check cur_index == set value*/
	cur_index = aw_dev_get_profile_index(aw882xx->aw_pa);
	if (cur_index == ucontrol->value.integer.value[0]) {
		aw_dev_info(codec->dev, "%s: index no change \n", __func__);
		return 0;
	}


	mutex_lock(&aw882xx->lock);
	aw_dev_set_profile_index(aw882xx->aw_pa, ucontrol->value.integer.value[0]);
	//pstream = 1 no pcm just set status
	if (aw882xx->pstream) {
		if (aw882xx->allow_pw == true) {
			aw_dev_prof_update(aw882xx->aw_pa);
		}
	}
	mutex_unlock(&aw882xx->lock);

	aw_dev_info(codec->dev, "%s: profile id %d \n", __func__, (int)ucontrol->value.integer.value[0]);
	return 0;
}
#endif

static int aw882xx_switch_info(struct snd_kcontrol *kcontrol,
			 struct snd_ctl_elem_info *uinfo)
{
	int count;

	uinfo->type = SNDRV_CTL_ELEM_TYPE_ENUMERATED;
	uinfo->count = 1;
	count = 2;

	uinfo->value.enumerated.items = count;

	if (uinfo->value.enumerated.item > count) {
		uinfo->value.enumerated.item = count -1;
	}

	strcpy(uinfo->value.enumerated.name,
		aw882xx_switch[uinfo->value.enumerated.item]);

	return 0;
}

static int aw882xx_switch_get(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	aw_snd_soc_codec_t *codec =
		aw_componet_codec_ops.kcontrol_codec(kcontrol);
	struct aw882xx *aw882xx =
		aw_componet_codec_ops.codec_get_drvdata(codec);

	ucontrol->value.integer.value[0] = aw882xx->allow_pw;

	return 0;

}

static int aw882xx_switch_set(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	aw_snd_soc_codec_t *codec =
		aw_componet_codec_ops.kcontrol_codec(kcontrol);
	struct aw882xx *aw882xx =
		aw_componet_codec_ops.codec_get_drvdata(codec);

	//stream not open
	if (aw882xx->pstream) {
		if (ucontrol->value.integer.value[0] == 0) {
			cancel_delayed_work_sync(&aw882xx->start_work);
			mutex_lock(&aw882xx->lock);
			aw_device_stop(aw882xx->aw_pa);
			mutex_unlock(&aw882xx->lock);
		} else {
			//wait start and switch profile ,restart PA
			cancel_delayed_work_sync(&aw882xx->start_work);
			mutex_lock(&aw882xx->lock);
			aw_device_start(aw882xx->aw_pa, AW_START_TYPE_SWITCH);
			mutex_unlock(&aw882xx->lock);
		}
	}

	if (ucontrol->value.integer.value[0]) {
		aw882xx->allow_pw = true;
	} else {
		aw882xx->allow_pw = false;
	}
	return 0;
}


static int aw882xx_dynamic_create_controls(struct aw882xx *aw882xx)
{
	struct snd_kcontrol_new *aw882xx_dev_control = NULL;
	char *kctl_name;

	aw882xx_dev_control = devm_kzalloc(aw882xx->codec->dev, sizeof(struct snd_kcontrol_new) * 2, GFP_KERNEL);
	if (aw882xx_dev_control == NULL) {
		aw_dev_err(aw882xx->codec->dev, "%s: kcontrol malloc failed! \n", __func__);
		return -ENOMEM;
	}
/*
	kctl_name = devm_kzalloc(aw882xx->codec->dev, AW_NAME_BUF_MAX, GFP_KERNEL);
	if (!kctl_name)
		return -ENOMEM;

	snprintf(kctl_name, AW_NAME_BUF_MAX, "aw_dev_%d_profile", aw882xx->index);

	aw882xx_dev_control[0].name = kctl_name;
	aw882xx_dev_control[0].iface = SNDRV_CTL_ELEM_IFACE_MIXER;
	aw882xx_dev_control[0].info = aw882xx_profile_info;
	aw882xx_dev_control[0].get = aw882xx_profile_get;
	aw882xx_dev_control[0].put = aw882xx_profile_set;
*/
	kctl_name = devm_kzalloc(aw882xx->codec->dev, AW_NAME_BUF_MAX, GFP_KERNEL);
	if (!kctl_name)
		return -ENOMEM;

	snprintf(kctl_name, AW_NAME_BUF_MAX, "aw_dev_%d_switch", aw882xx->index);

	aw882xx_dev_control[0].name = kctl_name;
	aw882xx_dev_control[0].iface = SNDRV_CTL_ELEM_IFACE_MIXER;
	aw882xx_dev_control[0].info = aw882xx_switch_info;
	aw882xx_dev_control[0].get = aw882xx_switch_get;
	aw882xx_dev_control[0].put = aw882xx_switch_set;

	aw_componet_codec_ops.add_codec_controls(aw882xx->codec,
						aw882xx_dev_control, 1);

	return 0;
}


static void aw882xx_firmware_acf_loaded(const struct firmware *cont, void *context)
{
	int ret;
	struct aw882xx *aw882xx = context;
	aw_acf_t *acf_data;
	int acf_size;

	aw882xx->fw_status = AW_DEV_FW_FAILED;
	if (!cont) {
		aw_dev_err(aw882xx->dev, "%s:load [%s] failed!", __func__, AW882XX_ACF_FILE);
		return;
	}

	aw_dev_info(aw882xx->dev, "%s: load [%s] , file size: [%zu] \n", __func__,
		AW882XX_ACF_FILE, cont ? cont->size : 0);

	mutex_lock(&g_aw882xx_mutex_lock);
	if (g_awinic_acf == NULL) {
		acf_data = kzalloc(cont->size + sizeof(int), GFP_KERNEL);
		if (acf_data == NULL) {
			mutex_unlock(&g_aw882xx_mutex_lock);
			release_firmware(cont);
			aw_dev_err(aw882xx->dev, "%s:malloc failed \n", __func__);
			return;
		}
		acf_size = cont->size;
		memcpy(acf_data, cont->data, acf_size);
		release_firmware(cont);
		ret = aw_load_acf_check(acf_data, acf_size);
		if (ret) {
			aw_dev_err(aw882xx->dev, "%s:Load [%s] failed ....!\n", __func__, AW882XX_ACF_FILE);
			kfree(acf_data);
			mutex_unlock(&g_aw882xx_mutex_lock);
			return;
		}
		g_awinic_acf = acf_data;
	} else {
		acf_data = g_awinic_acf;
		acf_size = cont->size;
		release_firmware(cont);
		aw_dev_info (aw882xx->dev, "%s: [%s] already loaded...\n", __func__, AW882XX_ACF_FILE);
	}
	mutex_unlock(&g_aw882xx_mutex_lock);

	mutex_lock(&aw882xx->lock);
	//aw device init
	ret = aw_device_init(aw882xx->aw_pa, acf_data);
	if (ret) {
		aw_dev_info (aw882xx->dev, "%s: dev init failed\n", __func__);
		mutex_unlock(&aw882xx->lock);
		return;
	}

	//create kcontrol by profile
	aw882xx_dynamic_create_controls(aw882xx);

	aw882xx->fw_status = AW_DEV_FW_OK;

	mutex_unlock(&aw882xx->lock);
}

static int aw882xx_request_firmware_file(struct aw882xx *aw882xx)
{
	aw_dev_info(aw882xx->dev, "%s: load [%s] \n", __func__, AW882XX_ACF_FILE);
	return request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG,
		AW882XX_ACF_FILE,
		aw882xx->dev, GFP_KERNEL,
		aw882xx, aw882xx_firmware_acf_loaded);
}

static void aw882xx_startup_work(struct work_struct *work)
{
	struct aw882xx *aw882xx = container_of(work, struct aw882xx, start_work.work);
	int ret = 0;

	if (aw882xx->fw_status == AW_DEV_FW_OK) {
		if (aw882xx->allow_pw == false) {
			aw_dev_info(aw882xx->dev, "%s: dev can not allow power \n", __func__);
			return;
		}
		mutex_lock(&aw882xx->lock);
		ret = aw_device_start(aw882xx->aw_pa, AW_START_TYPE_STREAM);
		if (ret) {
			aw_dev_info(aw882xx->dev, "%s: start up failded \n", __func__);
		} else {
			aw_dev_info(aw882xx->dev, "%s: start up success \n", __func__);
		}
		mutex_unlock(&aw882xx->lock);
	} else {
		aw_dev_info(aw882xx->dev, "%s: dev acf load failed \n", __func__);
	}
}

static int aw882xx_set_rx_en(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	int ret = -EINVAL;
	uint32_t ctrl_value = 0;
	struct aw_device *aw_dev;
	aw_snd_soc_codec_t *codec =
		aw_componet_codec_ops.kcontrol_codec(kcontrol);
	struct aw882xx *aw882xx =
		aw_componet_codec_ops.codec_get_drvdata(codec);

	aw_dev_dbg(aw882xx->dev, "%s: ucontrol->value.integer.value[0]=%ld\n",
		__func__, ucontrol->value.integer.value[0]);

	aw_dev = aw882xx->aw_pa;

	ctrl_value = ucontrol->value.integer.value[0];
	ret = aw_dev->ops.aw_set_afe_module_en(AW_RX_MODULE, ctrl_value);
	if (ret)
		aw_dev_err(aw882xx->dev, "%s: dsp_msg error, ret=%d\n",
			__func__, ret);

	return 0;
}

static int aw882xx_get_rx_en(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	int ret = -EINVAL;
	uint32_t ctrl_value = 0;
	struct aw_device *aw_dev;

	aw_snd_soc_codec_t *codec =
		aw_componet_codec_ops.kcontrol_codec(kcontrol);
	struct aw882xx *aw882xx =
		aw_componet_codec_ops.codec_get_drvdata(codec);

	aw_dev = aw882xx->aw_pa;

	ret = aw_dev->ops.aw_get_afe_module_en(AW_RX_MODULE, &ctrl_value);
	if (ret)
		aw_dev_err(aw882xx->dev, "%s: dsp_msg error, ret=%d\n",
			__func__, ret);

	ucontrol->value.integer.value[0] = ctrl_value;

	aw_dev_dbg(aw882xx->dev, "%s: aw882xx_rx_enable %d\n",
		__func__, ctrl_value);
	return 0;
}


static int aw882xx_set_tx_en(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	int ret = -EINVAL;
	uint32_t ctrl_value = 0;
	struct aw_device *aw_dev;
	aw_snd_soc_codec_t *codec =
		aw_componet_codec_ops.kcontrol_codec(kcontrol);
	struct aw882xx *aw882xx =
		aw_componet_codec_ops.codec_get_drvdata(codec);

	aw_dev_dbg(aw882xx->dev, "%s: ucontrol->value.integer.value[0]=%ld\n",
		__func__, ucontrol->value.integer.value[0]);

	aw_dev = aw882xx->aw_pa;

	ctrl_value = ucontrol->value.integer.value[0];
	ret = aw_dev->ops.aw_set_afe_module_en(AW_TX_MODULE, ctrl_value);
	if (ret)
		aw_dev_err(aw882xx->dev, "%s: dsp_msg error, ret=%d\n",
			__func__, ret);

	return 0;
}

static int aw882xx_get_tx_en(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	int ret = -EINVAL;
	uint32_t ctrl_value = 0;
	struct aw_device *aw_dev;

	aw_snd_soc_codec_t *codec =
		aw_componet_codec_ops.kcontrol_codec(kcontrol);
	struct aw882xx *aw882xx =
		aw_componet_codec_ops.codec_get_drvdata(codec);
	aw_dev = aw882xx->aw_pa;

	ret = aw_dev->ops.aw_get_afe_module_en(AW_TX_MODULE, &ctrl_value);
	if (ret)
		aw_dev_err(aw882xx->dev, "%s: dsp_msg error, ret=%d\n",
			__func__, ret);

	ucontrol->value.integer.value[0] = ctrl_value;

	aw_dev_dbg(aw882xx->dev, "%s: aw882xx_tx_enable %d\n",
		__func__, ctrl_value);
	return 0;
}

static int aw882xx_set_copp_en(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	int ret = -EINVAL;
	uint32_t ctrl_value = 0;
	struct aw_device *aw_dev;
	aw_snd_soc_codec_t *codec =
		aw_componet_codec_ops.kcontrol_codec(kcontrol);
	struct aw882xx *aw882xx =
		aw_componet_codec_ops.codec_get_drvdata(codec);

	aw_dev_dbg(aw882xx->dev, "%s: ucontrol->value.integer.value[0]=%ld\n",
		__func__, ucontrol->value.integer.value[0]);

	aw_dev = aw882xx->aw_pa;

	ctrl_value = ucontrol->value.integer.value[0];
	ret = aw_dev->ops.aw_set_copp_module_en( ctrl_value);
	if (ret)
		aw_dev_err(aw882xx->dev, "%s: dsp_msg error, ret=%d\n",
			__func__, ret);

	return 0;
}

static int aw882xx_get_copp_en(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	aw_snd_soc_codec_t *codec =
		aw_componet_codec_ops.kcontrol_codec(kcontrol);
	struct aw882xx *aw882xx =
		aw_componet_codec_ops.codec_get_drvdata(codec);


	ucontrol->value.integer.value[0] = 0;

	aw_dev_dbg(aw882xx->dev, "%s: done nothing\n",
		__func__);
	return 0;
}

static int aw882xx_set_spin(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	int ret = -EINVAL;
	uint32_t ctrl_value = 0;
	struct aw_device *aw_dev;
	aw_snd_soc_codec_t *codec =
		aw_componet_codec_ops.kcontrol_codec(kcontrol);
	struct aw882xx *aw882xx =
		aw_componet_codec_ops.codec_get_drvdata(codec);

	aw_dev_dbg(aw882xx->dev, "%s: ucontrol->value.integer.value[0]=%ld\n",
		__func__, ucontrol->value.integer.value[0]);

	aw_dev = aw882xx->aw_pa;

	ctrl_value = ucontrol->value.integer.value[0];
	ret = aw_dev->ops.aw_set_spin_mode(ctrl_value);
	if (ret)
		aw_dev_err(aw882xx->dev, "%s: set spin error, ret=%d\n",
			__func__, ret);

	return 0;
}

static int aw882xx_get_spin(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct aw_device *aw_dev;
	aw_snd_soc_codec_t *codec =
		aw_componet_codec_ops.kcontrol_codec(kcontrol);
	struct aw882xx *aw882xx =
		aw_componet_codec_ops.codec_get_drvdata(codec);
	int ctrl_value;

	aw_dev = aw882xx->aw_pa;

	aw_dev->ops.aw_get_spin_mode(&ctrl_value);

	ucontrol->value.integer.value[0] = ctrl_value;

	aw_dev_dbg(aw882xx->dev, "%s: done nothing\n",
		__func__);
	return 0;
}


static const struct soc_enum aw882xx_snd_enum[] = {
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(aw882xx_switch), aw882xx_switch),
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(aw882xx_spin), aw882xx_spin),
};

static struct snd_kcontrol_new aw882xx_controls[] = {
	SOC_ENUM_EXT("aw882xx_rx_switch", aw882xx_snd_enum[0],
		aw882xx_get_rx_en, aw882xx_set_rx_en),
	SOC_ENUM_EXT("aw882xx_tx_switch", aw882xx_snd_enum[0],
		aw882xx_get_tx_en, aw882xx_set_tx_en),
	SOC_ENUM_EXT("aw882xx_copp_switch", aw882xx_snd_enum[0],
		aw882xx_get_copp_en, aw882xx_set_copp_en),
	SOC_ENUM_EXT("aw882xx_spin_switch", aw882xx_snd_enum[1],
		aw882xx_get_spin, aw882xx_set_spin),
};

static void aw882xx_add_codec_controls(struct aw882xx *aw882xx)
{
	aw_dev_info(aw882xx->dev, "%s: enter\n", __func__);

	aw_componet_codec_ops.add_codec_controls(aw882xx->codec,
				&aw882xx_controls[0], ARRAY_SIZE(aw882xx_controls));
}

static int aw882xx_codec_probe(aw_snd_soc_codec_t *aw_codec)
{
	struct aw882xx *aw882xx =
		aw_componet_codec_ops.codec_get_drvdata(aw_codec);
	aw_dev_info(aw882xx->dev, "%s: enter\n", __func__);

	aw882xx->work_queue = create_singlethread_workqueue("aw882xx");
	if (!aw882xx->work_queue) {
		aw_dev_err(aw882xx->dev, "%s: create workqueue failed !", __func__);
		return -EINVAL;
	}

	INIT_DELAYED_WORK(&aw882xx->start_work, aw882xx_startup_work);

	aw882xx->codec = aw_codec;

	aw882xx_add_codec_controls(aw882xx);

	//load fw bin
	aw882xx_request_firmware_file(aw882xx);
	aw_load_monitor_cfg(&aw882xx->aw_pa->monitor_desc);


	return 0;
}

#ifdef AW_KERNEL_VER_OVER_4_19_1
static void aw882xx_codec_remove( aw_snd_soc_codec_t *aw_codec)
{
	struct aw882xx *aw882xx =
		aw_componet_codec_ops.codec_get_drvdata(aw_codec);
	aw_dev_info(aw882xx->dev, "%s: enter\n", __func__);
	aw_dev_deinit(aw882xx->aw_pa);
}
#else
static int aw882xx_codec_remove(aw_snd_soc_codec_t *aw_codec)
{
	struct aw882xx *aw882xx =
		aw_componet_codec_ops.codec_get_drvdata(aw_codec);

	aw_dev_info(aw882xx->dev, "%s: enter\n", __func__);
	aw_dev_deinit(aw882xx->aw_pa);
	return 0;
}
#endif

static int aw882xx_dai_drv_append_suffix(struct aw882xx *aw882xx,
				struct snd_soc_dai_driver *dai_drv,
				int num_dai)
{
	char buf[50];
	int i;
	int i2cbus = aw882xx->i2c->adapter->nr;
	int addr = aw882xx->i2c->addr;

	if ((dai_drv != NULL) && (num_dai > 0))
		for(i = 0; i < num_dai; i++) {
			snprintf(buf, 50, "%s-%x-%x",dai_drv[i].name, i2cbus,
				addr);
			dai_drv[i].name = aw882xx_devm_kstrdup(aw882xx->dev, buf);

			snprintf(buf, 50, "%s-%x-%x",
						dai_drv[i].playback.stream_name,
						i2cbus, addr);
			dai_drv[i].playback.stream_name = aw882xx_devm_kstrdup(aw882xx->dev, buf);

			snprintf(buf, 50, "%s-%x-%x",
						dai_drv[i].capture.stream_name,
						i2cbus, addr);
			dai_drv[i].capture.stream_name = aw882xx_devm_kstrdup(aw882xx->dev, buf);
			aw_dev_info(aw882xx->dev,"%s: dai name [%s] \n", __func__, dai_drv[i].name);
			aw_dev_info(aw882xx->dev,"%s: pstream_name name [%s] \n", __func__, dai_drv[i].playback.stream_name);
			aw_dev_info(aw882xx->dev,"%s: cstream_name name [%s] \n", __func__, dai_drv[i].capture.stream_name);
		}

	return 0;
}


static struct snd_soc_dai_driver aw882xx_dai[] = {
	{
		.name = "aw882xx-aif",
		.id = 1,
		.playback = {
			.stream_name = "Speaker_Playback",
			.channels_min = 1,
			.channels_max = 2,
			.rates = SNDRV_PCM_RATE_8000_48000,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE | \
				SNDRV_PCM_FMTBIT_S24_LE | \
				SNDRV_PCM_FMTBIT_S32_LE),
		},
		.capture = {
			.stream_name = "Speaker_Capture",
			.channels_min = 1,
			.channels_max = 2,
			.rates = SNDRV_PCM_RATE_8000_48000,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE | \
				SNDRV_PCM_FMTBIT_S24_LE | \
				SNDRV_PCM_FMTBIT_S32_LE),
		 },
		.ops = &aw882xx_dai_ops,
		.symmetric_rates = 1,
	},
};


#ifdef AW_KERNEL_VER_OVER_4_19_1
static struct snd_soc_component_driver soc_codec_dev_aw882xx = {
	.probe = aw882xx_codec_probe,
	.remove = aw882xx_codec_remove,
};
#else
static struct snd_soc_codec_driver soc_codec_dev_aw882xx = {
	.probe = aw882xx_codec_probe,
	.remove = aw882xx_codec_remove,
};
#endif


int aw_componet_codec_register(struct aw882xx *aw882xx)
{
	struct snd_soc_dai_driver *dai_drv;
	int ret;
	dai_drv = devm_kzalloc(aw882xx->dev, sizeof(aw882xx_dai), GFP_KERNEL);
	if (dai_drv == NULL) {
		aw_dev_err(aw882xx->dev,"%s: dai_driver malloc failed \n", __func__);
		return -ENOMEM;
	}

	memcpy(dai_drv, aw882xx_dai, sizeof(aw882xx_dai));

	aw882xx_dai_drv_append_suffix(aw882xx, dai_drv, ARRAY_SIZE(aw882xx_dai));

	ret = aw882xx->codec_ops->register_codec(aw882xx->dev,
			&soc_codec_dev_aw882xx,
			dai_drv, ARRAY_SIZE(aw882xx_dai));
	if (ret < 0) {
		aw_dev_err(aw882xx->dev, "%s failed to register aw882xx: %d\n",
			__func__, ret);
		return -EINVAL;
	}
	return 0;
}
/*****************************************************
 *
 * device tree
 *
 *****************************************************/
static int aw882xx_parse_gpio_dt(struct aw882xx *aw882xx,
	struct device_node *np)
{
	int ret = 0;;

	if (!np) {
		aw882xx->reset_gpio = -1;
		aw882xx->irq_gpio = -1;
		return -EINVAL;
	}

	aw882xx->reset_gpio = of_get_named_gpio(np, "reset-gpio", 0);
	if (aw882xx->reset_gpio < 0) {
		aw_dev_err(aw882xx->dev,"%s: no reset gpio provided, will not HW reset device\n",
			__func__);
		ret = -EFAULT;
	} else {
		aw_dev_info(aw882xx->dev,"%s: reset gpio provided ok\n",
			__func__);
	}
	aw882xx->irq_gpio = of_get_named_gpio(np, "irq-gpio", 0);
	if (aw882xx->irq_gpio < 0) {
		aw_dev_err(aw882xx->dev, "%s: no irq gpio provided.\n", __func__);
		ret = -EFAULT;
	} else {
		aw_dev_info(aw882xx->dev, "%s: irq gpio provided ok.\n", __func__);
	}

	return ret;
}

static struct aw882xx *aw882xx_malloc_init(struct i2c_client *i2c)
{
	struct aw882xx *aw882xx = devm_kzalloc(&i2c->dev, sizeof(struct aw882xx), GFP_KERNEL);
	if (aw882xx == NULL) {
		dev_err(&i2c->dev, "%s: devm_kzalloc failed.\n", __func__);
		return NULL;
	}

	aw882xx->dev = &i2c->dev;
	aw882xx->i2c = i2c;
	aw882xx->aw_pa = NULL;
	aw882xx->codec = NULL;
	aw882xx->codec_ops = &aw_componet_codec_ops;
	aw882xx->fw_status = AW_DEV_FW_FAILED;
	aw882xx->dbg_en_prof = true;
	aw882xx->allow_pw = true;

	INIT_LIST_HEAD(&aw882xx->list);
	mutex_init(&aw882xx->lock);

	return aw882xx;
}

int first_flag=1;
static int aw882xx_gpio_request(struct aw882xx *aw882xx)
{
	int ret;
	if(first_flag==1)
	{
		first_flag=0;
		if (gpio_is_valid(aw882xx->reset_gpio)) {
			ret = devm_gpio_request_one(aw882xx->dev, aw882xx->reset_gpio,
				GPIOF_OUT_INIT_LOW, "aw882xx_rst");
			if (ret) {
				aw_dev_err(aw882xx->dev, "%s: rst request failed\n",
					__func__);
				return ret;
			}
		}
	}

	if (gpio_is_valid(aw882xx->irq_gpio)) {
		ret = devm_gpio_request_one(aw882xx->dev, aw882xx->irq_gpio,
			GPIOF_DIR_IN, "aw882xx_int");
		if (ret) {
			aw_dev_err(aw882xx->dev, "%s: int request failed\n",
				__func__);
			return ret;
		}
	}

	return 0;
}

static int aw882xx_parse_dt(struct device *dev, struct aw882xx *aw882xx,
		struct device_node *np)
{
	int ret;

	/*gpio dts parser*/
	ret = aw882xx_parse_gpio_dt(aw882xx, np);
	if (ret)
		return ret;

	return 0;
}

int aw882xx_hw_reset(struct aw882xx *aw882xx)
{
	aw_dev_info(aw882xx->dev, "%s: enter\n", __func__);

	if (aw882xx && gpio_is_valid(aw882xx->reset_gpio)) {
		gpio_set_value_cansleep(aw882xx->reset_gpio, 0);
		mdelay(1);
		gpio_set_value_cansleep(aw882xx->reset_gpio, 1);
		mdelay(2);
	} else {
		aw_dev_err(aw882xx->dev, "%s: failed\n", __func__);
	}
	return 0;
}

static int aw882xx_read_chipid(struct aw882xx *aw882xx)
{
	int ret = -1;
	unsigned int cnt = 0;
	unsigned int reg_value = 0;

	while (cnt < AW_READ_CHIPID_RETRIES) {
		ret = aw882xx_i2c_read(aw882xx, AW882XX_ID_REG, &reg_value);
		if (ret < 0) {
			aw_dev_err(aw882xx->dev,
				"%s: failed to read REG_ID: %d\n",
				__func__, ret);
			return -EIO;
		}
		switch (reg_value) {
		case AW882XX_ID:
			aw_dev_info(aw882xx->dev, "%s: aw882xx detected\n",
				__func__);
			aw882xx->chip_id = reg_value;
			return 0;
		default:
			aw_dev_info(aw882xx->dev, "%s: unsupported device revision (0x%x)\n",
				__func__, reg_value);
			break;
		}
		cnt++;

		msleep(AW_READ_CHIPID_RETRY_DELAY);
	}

	return -EINVAL;
}

static int aw882xx_aw_device_init(struct aw882xx *aw882xx)
{
	struct aw_device *aw_pa;
	aw_pa = devm_kzalloc(aw882xx->dev, sizeof(struct aw_device), GFP_KERNEL);
	if (aw_pa == NULL) {
		aw_dev_err(aw882xx->dev, "%s: dev kalloc failed\n", __func__);
		return -ENOMEM;
	}

	//call aw device init func
	aw_pa->acf = NULL;
	aw_pa->prof_info.desc = NULL;
	aw_pa->prof_info.count = 0;
	aw_pa->channel = 0;

	aw_pa->index = g_aw882xx_dev_cnt;
	aw_pa->private_data = (void *)aw882xx;
	aw_pa->dev = aw882xx->dev;
	aw_pa->ops.aw_i2c_read = aw882xx_dev_i2c_read;
	aw_pa->ops.aw_i2c_write = aw882xx_dev_i2c_write;
	aw_pa->ops.aw_i2c_write_bits = aw882xx_dev_i2c_write_bits;
	aw_pa->ops.aw_get_volume = aw882xx_dev_get_volume;
	aw_pa->ops.aw_set_volume = aw882xx_dev_set_volume;

	aw_pa->int_desc.reg = AW882XX_SYSINTM_REG;
	aw_pa->int_desc.reg_default = AW882XX_SYSINTM_DEFAULT;

	aw_pa->pwd_desc.reg = AW882XX_SYSCTRL_REG;
	aw_pa->pwd_desc.mask = AW882XX_PWDN_MASK;
	aw_pa->pwd_desc.enable = AW882XX_PWDN_POWER_DOWN_VALUE;
	aw_pa->pwd_desc.disable = AW882XX_PWDN_NORMAL_WORKING_VALUE;

	aw_pa->mute_desc.reg = AW882XX_SYSCTRL2_REG;
	aw_pa->mute_desc.mask = AW882XX_HMUTE_MASK;
	aw_pa->mute_desc.enable = AW882XX_HMUTE_ENABLE_VALUE;
	aw_pa->mute_desc.disable = AW882XX_HMUTE_DISABLE_VALUE;

	aw_pa->vcalb_desc.vcalb_reg = AW882XX_VTMCTRL3_REG;
	aw_pa->vcalb_desc.vcal_factor = AW882XX_VCAL_FACTOR;
	aw_pa->vcalb_desc.cabl_base_value = AW882XX_CABL_BASE_VALUE;

	aw_pa->vcalb_desc.icalk_value_factor = AW882XX_ICABLK_FACTOR;
	aw_pa->vcalb_desc.icalk_reg = AW882XX_EFRM1_REG;
	aw_pa->vcalb_desc.icalk_reg_mask = AW882XX_EF_ISN_GESLP_MASK;
	aw_pa->vcalb_desc.icalk_sign_mask = AW882XX_EF_ISN_GESLP_SIGN_MASK;
	aw_pa->vcalb_desc.icalk_neg_mask = AW882XX_EF_ISN_GESLP_NEG;

	aw_pa->vcalb_desc.vcalk_reg = AW882XX_EFRH_REG;
	aw_pa->vcalb_desc.vcalk_reg_mask = AW882XX_EF_VSN_GESLP_MASK;
	aw_pa->vcalb_desc.vcalk_sign_mask = AW882XX_EF_VSN_GESLP_SIGN_MASK;
	aw_pa->vcalb_desc.vcalk_neg_mask = AW882XX_EF_VSN_GESLP_NEG;
	aw_pa->vcalb_desc.vcalk_value_factor = AW882XX_VCABLK_FACTOR;

	aw_pa->int_status_reg = AW882XX_SYSINT_REG;

	aw_pa->sysst_desc.reg = AW882XX_SYSST_REG;
	aw_pa->sysst_desc.mask = AW882XX_SYSST_CHECK_MASK;
	aw_pa->sysst_desc.check = AW882XX_SYSST_CHECK;

	aw_pa->volume_desc.reg = AW882XX_HAGCCFG4_REG;
	aw_pa->volume_desc.mask = 0x00ff;
	aw_pa->volume_desc.shift = 8;
	aw_pa->volume_desc.step = AW882XX_VOLUME_STEP_DB << 1;
	aw_pa->volume_desc.in_step_time = AW_1000_US / 10;
	aw_pa->volume_desc.out_step_time = AW_1000_US >> 1;

	aw_device_probe(aw_pa);

	aw882xx->aw_pa = aw_pa;
	return 0;
}

static ssize_t aw882xx_reg_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct aw882xx *aw882xx = dev_get_drvdata(dev);
	unsigned int databuf[2] = {0};

	if (2 == sscanf(buf, "%x %x", &databuf[0], &databuf[1]))
		aw882xx_i2c_write(aw882xx, databuf[0], databuf[1]);

	return count;
}

static ssize_t aw882xx_reg_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct aw882xx *aw882xx = dev_get_drvdata(dev);
	ssize_t len = 0;
	unsigned char i = 0;
	unsigned int reg_val = 0;

	for (i = 0; i < AW882XX_REG_MAX; i++) {
		if (aw882xx_reg_access[i]&REG_RD_ACCESS) {
			aw882xx_i2c_read(aw882xx, i, &reg_val);
			len += snprintf(buf+len, PAGE_SIZE-len,
				"reg:0x%02x=0x%04x\n", i, reg_val);
		}
	}

	return len;
}

static ssize_t aw882xx_rw_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct aw882xx *aw882xx = dev_get_drvdata(dev);

	unsigned int databuf[2] = {0};

	if (2 == sscanf(buf, "%x %x", &databuf[0], &databuf[1])) {
		aw882xx->rw_reg_addr = (unsigned char)databuf[0];
		aw882xx_i2c_write(aw882xx, databuf[0], databuf[1]);
	} else if (1 == sscanf(buf, "%x", &databuf[0])) {
		aw882xx->rw_reg_addr = (unsigned char)databuf[0];
	}

	return count;
}

static ssize_t aw882xx_rw_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct aw882xx *aw882xx = dev_get_drvdata(dev);
	ssize_t len = 0;
	unsigned int reg_val = 0;

	if (aw882xx_reg_access[aw882xx->rw_reg_addr] & REG_RD_ACCESS) {
		aw882xx_i2c_read(aw882xx, aw882xx->rw_reg_addr, &reg_val);
		len += snprintf(buf+len, PAGE_SIZE-len,
			"reg:0x%02x=0x%04x\n", aw882xx->rw_reg_addr, reg_val);
	}
	return len;
}

static ssize_t aw882xx_drv_ver_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;

	len += snprintf(buf+len, PAGE_SIZE-len,
		"driver_ver: %s \n", AW882XX_VERSION);

	return len;
}

static ssize_t aw882xx_dsp_re_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;
	unsigned int dsp_re;
	struct aw882xx *aw882xx = dev_get_drvdata(dev);

	dsp_re = aw_dev_get_dsp_re(aw882xx->aw_pa);

	len += snprintf(buf+len, PAGE_SIZE-len,
		"%d \n", dsp_re);

	return len;
}

static ssize_t aw882xx_fade_step_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct aw882xx *aw882xx = dev_get_drvdata(dev);

	unsigned int databuf[2] = {0};

	/*step 0 - 12*/
	if (1 == sscanf(buf, "%d", &databuf[0])) {
		if (databuf[0] > (AW_FADE_OUT_TARGET_VOL)) {
			aw_dev_info(aw882xx->dev, "%s: step overflow %d Db", __func__, databuf[0]);
			return count;
		}
		aw_dev_set_volume_step(aw882xx->aw_pa, databuf[0]);
	}
	aw_dev_info(aw882xx->dev, "%s: set step %d DB Done", __func__, databuf[0]);

	return count;
}

static ssize_t aw882xx_fade_step_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct aw882xx *aw882xx = dev_get_drvdata(dev);
	ssize_t len = 0;

	len += snprintf(buf+len, PAGE_SIZE-len,
		"step: %d \n", aw_dev_get_volume_step(aw882xx->aw_pa));

	return len;
}

static ssize_t aw882xx_dbg_prof_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct aw882xx *aw882xx = dev_get_drvdata(dev);

	unsigned int databuf[2] = {0};

	if (1 == sscanf(buf, "%d", &databuf[0])) {
		if (databuf[0]) {
			aw882xx->dbg_en_prof = true;
		} else {
			aw882xx->dbg_en_prof = false;
		}
	}
	aw_dev_info(aw882xx->dev, "%s: en_prof %d  Done", __func__, databuf[0]);

	return count;
}

static ssize_t aw882xx_dbg_prof_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct aw882xx *aw882xx = dev_get_drvdata(dev);
	ssize_t len = 0;

	len += snprintf(buf+len, PAGE_SIZE-len,
		" %d \n", aw882xx->dbg_en_prof);

	return len;
}

static DEVICE_ATTR(reg, S_IWUSR | S_IRUGO,
	aw882xx_reg_show, aw882xx_reg_store);
static DEVICE_ATTR(rw, S_IWUSR | S_IRUGO,
	aw882xx_rw_show, aw882xx_rw_store);
static DEVICE_ATTR(drv_ver, S_IRUGO,
	aw882xx_drv_ver_show, NULL);
static DEVICE_ATTR(dsp_re, S_IRUGO,
	aw882xx_dsp_re_show, NULL);
static DEVICE_ATTR(fade_step, S_IWUSR | S_IRUGO,
	aw882xx_fade_step_show, aw882xx_fade_step_store);
static DEVICE_ATTR(dbg_prof, S_IWUSR | S_IRUGO,
	aw882xx_dbg_prof_show, aw882xx_dbg_prof_store);

static struct attribute *aw882xx_attributes[] = {
	&dev_attr_reg.attr,
	&dev_attr_rw.attr,
	&dev_attr_drv_ver.attr,
	&dev_attr_fade_step.attr,
	&dev_attr_dbg_prof.attr,
	&dev_attr_dsp_re.attr,
	NULL
};

static struct attribute_group aw882xx_attribute_group = {
	.attrs = aw882xx_attributes,
};

static int aw882xx_i2c_probe(struct i2c_client *i2c,
				const struct i2c_device_id *id)
{
	struct aw882xx *aw882xx;
	struct device_node *np = i2c->dev.of_node;
	int ret;

	pr_info("%s:enter addr=0x%x \n", __func__, i2c->addr);

	if (!i2c_check_functionality(i2c->adapter, I2C_FUNC_I2C)) {
		dev_err(&i2c->dev, "check_functionality failed\n");
		return -EIO;
	}

	//dev free all auto free
	aw882xx = aw882xx_malloc_init(i2c);
	if (aw882xx == NULL) {
		dev_err(&i2c->dev, "%s: failed to parse device tree node\n",
			__func__);
		return -ENOMEM;
	}

	i2c_set_clientdata(i2c, aw882xx);

	ret = aw882xx_parse_dt(&i2c->dev, aw882xx, np);
	if (ret) {
		aw_dev_err(&i2c->dev, "%s: failed to parse device tree node\n",
			__func__);
		return ret;
	}

	//get gpio resource
	ret = aw882xx_gpio_request(aw882xx);
	if (ret)
		return ret;

	/* hardware reset */
	aw882xx_hw_reset(aw882xx);

	/* aw882xx chip id */
	ret = aw882xx_read_chipid(aw882xx);
	if (ret < 0) {
		aw_dev_err(&i2c->dev, "%s: aw882xx_read_chipid failed ret=%d\n",
			__func__, ret);
		return ret;
	}

	/* read pa number*/
	/*ret = of_property_read_string(np, "pa_number", &pa_number);
	if (ret < 0) {
		dev_info(aw882xx->dev,
			"%s:read sound-channel failed,use default\n", __func__);
		return ret;
	}*/

	/* set aw882xx device name */
	/*if (i2c->dev.of_node) {
		if (pa_number)
			dev_set_name(&i2c->dev, "%s_%s", "aw882xx_smartpa",
				pa_number);
		else
			dev_set_name(&i2c->dev, "%s", "aw882xx_smartpa");
	} else {
		dev_err(&i2c->dev, "%s failed to set device name: %d\n",
			__func__, ret);
	}*/

	/*aw pa init*/
	ret = aw882xx_aw_device_init(aw882xx);
	if (ret) {
		return ret;
	}

	/*aw882xx irq*/

	/*codec register*/
	ret = aw_componet_codec_register(aw882xx);
	if (ret) {
		aw_dev_err(&i2c->dev, "%s: codec register failde\n", __func__);
		return ret;
	}

	/*create attr*/
	ret = sysfs_create_group(&i2c->dev.kobj, &aw882xx_attribute_group);
	if (ret < 0) {
		aw_dev_err(aw882xx->dev, "%s error creating sysfs attr files\n",
			__func__);
		goto err_sysfs;
	}

	/*set aw882xx to dev private*/
	dev_set_drvdata(&i2c->dev, aw882xx);

	aw882xx->index = g_aw882xx_dev_cnt;
	/*add device to total list*/
	mutex_lock(&g_aw882xx_mutex_lock);
	g_aw882xx_dev_cnt++;
	list_add(&aw882xx->list, &g_aw882xx_device_list);
	mutex_unlock(&g_aw882xx_mutex_lock);
	aw_dev_info(&i2c->dev, "%s: dev_cnt %d \n", __func__, g_aw882xx_dev_cnt);
	return ret;
err_sysfs:
	aw_componet_codec_ops.unregister_codec(&i2c->dev);

	return ret;
}

static int aw882xx_i2c_remove(struct i2c_client *i2c)
{
	struct aw882xx *aw882xx = i2c_get_clientdata(i2c);

	aw_dev_info(aw882xx->dev, "%s: enter\n", __func__);

	/*rm irq*/
	/*if (gpio_to_irq(aw882xx->irq_gpio))
		devm_free_irq(&i2c->dev,
			gpio_to_irq(aw882xx->irq_gpio),
			aw882xx);*/

	/*free gpio*/
	if (gpio_is_valid(aw882xx->irq_gpio))
		devm_gpio_free(&i2c->dev, aw882xx->irq_gpio);
	if (gpio_is_valid(aw882xx->reset_gpio))
		devm_gpio_free(&i2c->dev, aw882xx->reset_gpio);

	/*rm attr node*/
	sysfs_remove_group(&i2c->dev.kobj, &aw882xx_attribute_group);

	/*free device resource */
	aw_device_remove(aw882xx->aw_pa);

	/*unregister codec*/
	aw882xx->codec_ops->unregister_codec(&i2c->dev);

	/*remove device to total list*/
	mutex_lock(&g_aw882xx_mutex_lock);
	list_del(&aw882xx->list);
	g_aw882xx_dev_cnt--;
	if (g_aw882xx_dev_cnt == 0) {
		kfree(g_awinic_acf);
		g_awinic_acf = NULL;
	}
	mutex_unlock(&g_aw882xx_mutex_lock);

	return 0;

}


static const struct i2c_device_id aw882xx_i2c_id[] = {
	{ AW882XX_I2C_NAME, 0 },
	{ }
};
	
MODULE_DEVICE_TABLE(i2c, aw882xx_i2c_id);

static struct of_device_id aw882xx_dt_match[] = {
	{ .compatible = "awinic,aw882xx_smartpa" },
	{ .compatible = "awinic,aw882xx_smartpa_l" },
	{ .compatible = "awinic,aw882xx_smartpa_r" },
	{ },
};

static struct i2c_driver aw882xx_i2c_driver = {
	.driver = {
		.name = AW882XX_I2C_NAME,
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(aw882xx_dt_match),
	},
	.probe = aw882xx_i2c_probe,
	.remove = aw882xx_i2c_remove,
	.id_table = aw882xx_i2c_id,
};


static int __init aw882xx_i2c_init(void)
{
	int ret = -1;

	pr_info("%s: aw882xx driver version %s\n", __func__, AW882XX_VERSION);

	ret = i2c_add_driver(&aw882xx_i2c_driver);
	if (ret)
		pr_err("%s: fail to add aw882xx device into i2c\n", __func__);

	return ret;
}
module_init(aw882xx_i2c_init);


static void __exit aw882xx_i2c_exit(void)
{
	i2c_del_driver(&aw882xx_i2c_driver);
}
module_exit(aw882xx_i2c_exit);


MODULE_DESCRIPTION("ASoC AW882XX Smart PA Driver");
MODULE_LICENSE("GPL v2");



