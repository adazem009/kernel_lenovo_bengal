/*
 * awinic_monitor.c monitor_module
 *
 * Version: v0.1.17
 *
 * Copyright (c) 2019 AWINIC Technology CO., LTD
 *
 *  Author: Nick Li <liweilei@awinic.com.cn>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */
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
#include <linux/of.h>
#include <linux/version.h>
#include <linux/input.h>
#include <linux/timer.h>
#include <linux/workqueue.h>
#include <linux/hrtimer.h>
#include "aw882xx.h"
#include "aw882xx_reg.h"
#include "aw_monitor.h"
#include "aw_log.h"


#define AW_MONITOR_FILE  "aw882xx_monitor.bin"
static DEFINE_MUTEX(g_aw_monitor_lock);

struct aw_monitor_cfg g_monitor_cfg;

int aw_monitor_start(struct aw_monitor_desc *monitor_desc)
{
	struct aw_device *aw_dev = container_of(monitor_desc,
			struct aw_device, monitor_desc);
	struct aw882xx *aw882xx = dev_get_drvdata(aw_dev->dev);
	int real_prof_id;

	aw_dev_info(aw_dev->dev, "%s: enter\n", __func__);

	real_prof_id = aw_dev->prof_info.desc[aw_dev->cur_prof].id;
	if (aw_dev->monitor_desc.is_enable &&
		(real_prof_id != AW_PROFILE_RECEIVER) &&
		(g_monitor_cfg.monitor_status == AW_MON_CFG_OK) &&
		g_monitor_cfg.monitor_flag) {
		queue_delayed_work(aw882xx->work_queue,
			&aw882xx->monitor_work, msecs_to_jiffies(g_monitor_cfg.monitor_time));
	}

	return 0;
}

int aw_monitor_stop(struct aw_monitor_desc *monitor_desc)
{
	struct aw_device *aw_dev = container_of(monitor_desc,
			struct aw_device, monitor_desc);
	struct aw882xx *aw882xx = dev_get_drvdata(aw_dev->dev);

	aw_dev_info(aw_dev->dev, "%s: enter\n", __func__);

	cancel_delayed_work_sync(&aw882xx->monitor_work);

	return 0;
}

static int aw_monitor_get_voltage(struct aw_device *aw_dev,
					unsigned int *voltage)
{
	int ret = -1, i;
	uint16_t sum_vol = 0;
	struct aw_voltage_desc *desc = &aw_dev->monitor_desc.voltage_desc;

	for (i = 0; i < g_monitor_cfg.monitor_count; i++) {
		ret = aw_dev->ops.aw_i2c_read(aw_dev, desc->reg, voltage);
		if (ret < 0) {
			aw_dev_err(aw_dev->dev, "%s: read voltage failed !\n",
				__func__);
			return ret;
		}

		sum_vol += ((*voltage) * desc->vbat_range) / desc->int_bit;
	}

	*voltage = sum_vol / g_monitor_cfg.monitor_count;

	aw_dev_info(aw_dev->dev, "%s: chip voltage is %d\n",
		__func__, *voltage);

	return 0;
}

static int aw_monitor_voltage(struct aw_device *aw_dev)
{
	int ret = -1;
	int i;
	unsigned int voltage = 0;
	struct aw_monitor_desc *monitor_desc = &aw_dev->monitor_desc;
	struct aw_table_info *vol_info = &g_monitor_cfg.vol_info;

	if (!g_monitor_cfg.vol_flag) {
		monitor_desc->vol_ipeak = IPEAK_NONE;
		monitor_desc->vol_gain = GAIN_NONE;
		return 0;
	}

	if (vol_info->aw_table == NULL) {
		aw_dev_err(aw_dev->dev, "%s: vol_info->aw_table is NULL\n",
			__func__);
		return -ENOMEM;
	}

#ifdef AW_DEBUG
	if (monitor_desc->test_vol == 0) {
		ret = aw_monitor_get_voltage(aw_dev, &voltage);
		if (ret < 0)
			return ret;
	} else {
		voltage = monitor_desc->test_vol;
	}
#else
	ret = aw_monitor_get_voltage(aw_dev, &voltage);
	if (ret < 0)
		return ret;
#endif



	if (monitor_desc->pre_vol > voltage) {
		/* vol down*/
		for (i = vol_info->table_num - 1; i >= 0; i--) {
			if (voltage < vol_info->aw_table[i].val_max) {
				monitor_desc->vol_ipeak = vol_info->aw_table[i].ipeak;
				monitor_desc->vol_gain = vol_info->aw_table[i].gain;
				break;
			}
		}
		if (i < 0) {
			monitor_desc->vol_ipeak = IPEAK_NONE;
			monitor_desc->vol_ipeak = GAIN_NONE;
		}
	} else if (monitor_desc->pre_vol < voltage) {
		/*vol up*/
		for (i = 0; i < vol_info->table_num; i++) {
			if (voltage > vol_info->aw_table[i].val_min) {
				monitor_desc->vol_ipeak = vol_info->aw_table[i].ipeak;
				monitor_desc->vol_gain = vol_info->aw_table[i].gain;
				break;
			}
		}
		if (i == vol_info->table_num) {
			monitor_desc->vol_ipeak = IPEAK_NONE;
			monitor_desc->vol_ipeak = GAIN_NONE;
		}
	} else {
		//aw_dev_info(aw_dev->dev, "%s: vol no change", __func__);
	}

	monitor_desc->pre_vol = voltage;
	return 0;
}

static int aw_monitor_get_temperature(struct aw_device *aw_dev, int *temp)
{
	int ret = -1, i;
	unsigned int reg_val = 0;
	uint16_t local_temp;
	struct aw_temperature_desc *desc = &aw_dev->monitor_desc.temp_desc;

	*temp = 0;
	for (i = 0; i < g_monitor_cfg.monitor_count; i++) {
		ret = aw_dev->ops.aw_i2c_read(aw_dev, desc->reg, &reg_val);
		if (ret < 0) {
			aw_dev_err(aw_dev->dev, "%s: get temperature failed !\n",
				__func__);
			return ret;
		}

		local_temp = reg_val;
		if (local_temp & desc->sign_mask)
			local_temp = local_temp | desc->neg_mask;

		*temp += (int)local_temp;
	}

	*temp = *temp / g_monitor_cfg.monitor_count;
	aw_dev_info(aw_dev->dev, "%s: chip temperature = %d\n",
		__func__, local_temp);
	return 0;
}

static int aw_monitor_temperature(struct aw_device *aw_dev)
{
	int ret;
	int i;
	struct aw_monitor_desc *monitor_desc = &aw_dev->monitor_desc;
	int  current_temp = 0;
	struct aw_table_info *temp_info = &g_monitor_cfg.temp_info;

	if (!g_monitor_cfg.temp_flag) {
		monitor_desc->temp_ipeak = IPEAK_NONE;
		monitor_desc->temp_gain = GAIN_NONE;
		return 0;
	}

	if (temp_info->aw_table == NULL) {
		aw_dev_err(aw_dev->dev, "%s: temp_info->aw_table  is NULL\n",
			__func__);
		return -ENOMEM;
	}

#ifdef AW_DEBUG
	if (monitor_desc->test_temp == 0) {
		ret = aw_monitor_get_temperature(aw_dev, &current_temp);
		if (ret)
			return ret;
	} else {
		current_temp = monitor_desc->test_temp;
	}
#else
	ret = aw_monitor_get_temperature(aw_dev, &current_temp);
	if (ret < 0)
		return ret;
#endif


	if (monitor_desc->pre_temp > current_temp) {
		/*temp down*/
		for (i = temp_info->table_num - 1; i >= 0; i--) {
			if (current_temp < temp_info->aw_table[i].val_max) {
				monitor_desc->temp_ipeak = temp_info->aw_table[i].ipeak;
				monitor_desc->temp_gain = temp_info->aw_table[i].gain;
				break;
			}
		}

		if (i < 0) {
			monitor_desc->temp_ipeak = IPEAK_NONE;
			monitor_desc->temp_gain = GAIN_NONE;
		}
	} else if (monitor_desc->pre_temp < current_temp) {
		/*temp up*/
		for (i = 0; i < temp_info->table_num; i++) {
			if (current_temp > temp_info->aw_table[i].val_min) {
				monitor_desc->temp_ipeak = temp_info->aw_table[i].ipeak;
				monitor_desc->temp_gain = temp_info->aw_table[i].gain;
				break;
			}
		}
		if (i == temp_info->table_num) {
			monitor_desc->temp_ipeak = temp_info->aw_table[i].ipeak;
			monitor_desc->temp_gain = temp_info->aw_table[i].gain;
		}
	} else {
		/*temp no change*/
		//aw_dev_info(aw_dev->dev, "%s: temp no change", __func__);
	}
	monitor_desc->pre_temp = current_temp;
	return 0;
}

static void aw_monitor_get_cfg(struct aw_device *aw_dev,
				uint16_t *ipeak, uint16_t *gain)
{
	struct aw_monitor_desc *monitor_desc = &aw_dev->monitor_desc;


	aw_dev_info(aw_dev->dev, "%s: vol: ipeak = 0x%x, gain = 0x%x\n",
		__func__, monitor_desc->vol_ipeak, monitor_desc->vol_gain);

	aw_dev_info(aw_dev->dev, "%s: temp: ipeak = 0x%x, gain = 0x%x\n",
		__func__, monitor_desc->temp_ipeak, monitor_desc->temp_gain);

	/* get min Ipeak */
	*ipeak = (monitor_desc->vol_ipeak < monitor_desc->temp_ipeak ?
			monitor_desc->vol_ipeak : monitor_desc->temp_ipeak);

	/* get min gain */
	if (monitor_desc->vol_gain == GAIN_NONE || monitor_desc->temp_gain == GAIN_NONE) {
		if (monitor_desc->vol_gain == GAIN_NONE &&
			monitor_desc->temp_gain == GAIN_NONE)
			*gain = GAIN_NONE;
		else if (monitor_desc->vol_gain == GAIN_NONE)
			*gain = monitor_desc->temp_gain;
		else
			*gain = monitor_desc->vol_gain;
	} else {
		*gain = (monitor_desc->vol_gain > monitor_desc->temp_gain ?
				monitor_desc->vol_gain : monitor_desc->temp_gain);
	}

}

static void aw_monitor_set_ipeak(struct aw_device *aw_dev, uint16_t ipeak)
{
	unsigned int reg_val = 0;
	unsigned int read_reg_val;
	int ret;
	struct aw_ipeak_desc *desc = &aw_dev->monitor_desc.ipeak_desc;

	if (ipeak == IPEAK_NONE)
		return;

	ret = aw_dev->ops.aw_i2c_read(aw_dev, desc->reg, &reg_val);
	if (ret < 0) {
		aw_dev_err(aw_dev->dev, "%s: read ipeak failed\n", __func__);
		return;
	}

	read_reg_val = reg_val;
	read_reg_val &= desc->mask;

	if (read_reg_val == ipeak) {
		aw_dev_dbg(aw_dev->dev, "%s: ipeak = 0x%x, no change\n",
					__func__, read_reg_val);
		return;
	}
	reg_val &= (~desc->mask);
	read_reg_val = ipeak;
	reg_val |= read_reg_val;

	ret = aw_dev->ops.aw_i2c_write(aw_dev, desc->reg, reg_val);
	if (ret < 0) {
		aw_dev_err(aw_dev->dev, "%s: write ipeak failed\n", __func__);
		return;
	}
	aw_dev_info(aw_dev->dev, "%s: set reg val = 0x%x, ipeak = 0x%x\n",
					__func__, reg_val, ipeak);
}

static void aw_monitor_set_gain(struct aw_device *aw_dev, uint16_t gain)
{
	unsigned int reg_val = 0;
	unsigned int read_reg_val;
	int ret;
	struct aw_gain_desc *desc = &aw_dev->monitor_desc.gain_desc;

	if (gain == GAIN_NONE)
		return;

	ret = aw_dev->ops.aw_i2c_read(aw_dev, desc->reg, &reg_val);
	if (ret < 0) {
		aw_dev_err(aw_dev->dev, "%s: read gain failed\n", __func__);
		return;
	}

	read_reg_val = reg_val;
	read_reg_val = read_reg_val >> desc->shift;

	if (read_reg_val == gain) {
		aw_dev_dbg(aw_dev->dev, "%s: gain = 0x%x, no change\n",
				__func__, read_reg_val);
		return;
	}
	reg_val &= desc->mask;
	reg_val |= (gain << desc->shift);

	ret = aw_dev->ops.aw_i2c_write(aw_dev, desc->reg, reg_val);
	if (ret < 0) {
		aw_dev_err(aw_dev->dev, "%s: write gain failed\n", __func__);
		return;
	}
	aw_dev_info(aw_dev->dev, "%s: set reg val = 0x%x, gain = 0x%x\n",
			__func__, reg_val, gain);
}

static void aw_monitor_work(struct aw_device *aw_dev)
{
	int ret;
	uint16_t real_ipeak = 0;
	uint16_t real_gain = 0;

	if (aw_dev->cali_desc.status != 0) {
		aw_dev_info(aw_dev->dev, "%s: done nothing while start cali\n",
			__func__);
		return;
	}
	mutex_lock(&g_aw_monitor_lock);
	ret = aw_monitor_voltage(aw_dev);
	if (ret < 0) {
		aw_dev_err(aw_dev->dev, "%s: monitor voltage failed\n",
			__func__);
		mutex_unlock(&g_aw_monitor_lock);
		return;
	}

	ret = aw_monitor_temperature(aw_dev);
	if (ret < 0) {
		aw_dev_err(aw_dev->dev, "%s: monitor temperature failed\n",
			__func__);
		mutex_unlock(&g_aw_monitor_lock);
		return;
	}
	mutex_unlock(&g_aw_monitor_lock);

	aw_monitor_get_cfg(aw_dev, &real_ipeak, &real_gain);

	aw_monitor_set_ipeak(aw_dev, real_ipeak);

	aw_monitor_set_gain(aw_dev, real_gain);
}

static int aw_get_hmute(struct aw_device *aw_dev)
{
	unsigned int reg_val = 0;
	int ret;
	struct aw_mute_desc *desc = &aw_dev->mute_desc;

	aw_dev_dbg(aw_dev->dev, "%s: enter\n", __func__);

	ret = aw_dev->ops.aw_i2c_read(aw_dev, desc->reg, &reg_val);
	if (ret < 0)
		return ret;

	if (reg_val & (~desc->mask))
		ret = 1;
	else
		ret = 0;

	return ret;
}
static void aw_monitor_work_func(struct work_struct *work)
{
	struct aw882xx *aw882xx = container_of(work,
		struct aw882xx, monitor_work.work);
	struct aw_device *aw_dev = aw882xx->aw_pa;
	struct aw_monitor_desc *monitor_desc = &aw_dev->monitor_desc;

	mutex_lock(&aw882xx->lock);
	if (!aw_get_hmute(aw_dev)) {
		aw_monitor_work(aw_dev);
		aw_monitor_start(monitor_desc);
	}
	mutex_unlock(&aw882xx->lock);
}



/*****************************************************
 * load monitor config
 *****************************************************/
static int aw_check_monitor_cfg(struct aw_device *aw_dev,
					const struct firmware *cont)
{
	struct aw_monitor_hdr *monitor_hdr =
		(struct aw_monitor_hdr *)cont->data;
	int temp_size, vol_size, actual_size;

	if (cont->size < sizeof(struct aw_monitor_hdr)) {
		aw_dev_err(aw_dev->dev, "%s:params size[%d] < struct aw_monitor_hdr size[%d]!\n",
			__func__, (int)cont->size, (int)sizeof(struct aw_monitor_hdr));
		return -ENOMEM;
	}

	if (monitor_hdr->temp_offset > cont->size) {
		aw_dev_err(aw_dev->dev, "%s:temp_offset[%d] overflow file size[%d]!\n",
			__func__, monitor_hdr->temp_offset, (int)cont->size);
		return -ENOMEM;
	}

	if (monitor_hdr->vol_offset > cont->size) {
		aw_dev_err(aw_dev->dev, "%s:vol_offset[%d] overflow file size[%d]!\n",
			__func__, monitor_hdr->vol_offset, (int)cont->size);
		return -ENOMEM;
	}

	temp_size = monitor_hdr->temp_num * sizeof(struct aw_table);
	actual_size = monitor_hdr->vol_offset - monitor_hdr->temp_offset;
	if (actual_size != temp_size) {
		aw_dev_err(aw_dev->dev, "%s:temp_size:[%d] is not equal to actual_size[%d]!\n",
			__func__, temp_size, actual_size);
		return -ENOMEM;
	}

	vol_size = monitor_hdr->vol_num * sizeof(struct aw_table);
	actual_size = cont->size - monitor_hdr->vol_offset;
	if (actual_size != vol_size) {
		aw_dev_err(aw_dev->dev, "%s:vol_size:[%d] is not equal to actual_size[%d]\n",
			__func__, vol_size, actual_size);
		return -ENOMEM;
	}

	return 0;
}

static void aw_parse_monitor_hdr_by_0_0_1(struct aw_device *aw_dev,
					const struct firmware *cont)
{
	struct aw_monitor_hdr *monitor_hdr =
			(struct aw_monitor_hdr *)cont->data;

	g_monitor_cfg.monitor_flag = monitor_hdr->monitor_flag;
	g_monitor_cfg.monitor_time = monitor_hdr->monitor_time;
	g_monitor_cfg.monitor_count = monitor_hdr->monitor_count;
	g_monitor_cfg.temp_flag = monitor_hdr->temp_flag;
	g_monitor_cfg.vol_flag = monitor_hdr->vol_flag;

	aw_dev_info(aw_dev->dev, "%s:monitor_flag:%d, monitor_time:%d (ms), monitor_count:%d, temp_flag:%d, vol_flag:%d",
		__func__, g_monitor_cfg.monitor_flag,
		g_monitor_cfg.monitor_time, g_monitor_cfg.monitor_count,
		g_monitor_cfg.temp_flag, g_monitor_cfg.vol_flag);
}

static void aw_populate_data_to_table(struct aw_device *aw_dev,
			struct aw_table_info *table_info, const char *offset_ptr)
{
	int i = 0;

	for (i = 0; i < table_info->table_num * AW_TABLE_SIZE; i += AW_TABLE_SIZE) {
		table_info->aw_table[i / AW_TABLE_SIZE].val_min =
				GET_16_DATA(offset_ptr[1 + i], offset_ptr[i]);
		table_info->aw_table[i / AW_TABLE_SIZE].val_max =
				GET_16_DATA(offset_ptr[3 + i], offset_ptr[2 + i]);
		table_info->aw_table[i / AW_TABLE_SIZE].ipeak =
				GET_16_DATA(offset_ptr[5 + i], offset_ptr[4 + i]);
		table_info->aw_table[i / AW_TABLE_SIZE].gain =
				GET_16_DATA(offset_ptr[7 + i], offset_ptr[6 + i]);
	}

	for (i = 0; i < table_info->table_num; i++) {
		aw_dev_info(aw_dev->dev, "val_min:%d, val_max:%d, ipeak:0x%x, gain:0x%x",
			table_info->aw_table[i].val_min, table_info->aw_table[i].val_max,
			table_info->aw_table[i].ipeak, table_info->aw_table[i].gain);
	}

}

static int aw_parse_temp_data_by_0_0_1(struct aw_device *aw_dev,
			const struct firmware *cont)
{
	struct aw_monitor_hdr *monitor_hdr =
			(struct aw_monitor_hdr *)cont->data;
	struct aw_table_info *temp_info = &g_monitor_cfg.temp_info;

	aw_dev_info(aw_dev->dev, "%s: ======== start =======", __func__);

	if (temp_info->aw_table != NULL)
		kfree(temp_info->aw_table);

	temp_info->aw_table = kzalloc((monitor_hdr->temp_num * AW_TABLE_SIZE),
			GFP_KERNEL);
	if (!temp_info->aw_table) {
		aw_dev_err(aw_dev->dev, "%s:g_temp_info kzalloc faild\n", __func__);
		return -ENOMEM;
	}

	temp_info->table_num = monitor_hdr->temp_num;
	aw_populate_data_to_table(aw_dev, temp_info,
			&cont->data[monitor_hdr->temp_offset]);
	aw_dev_info(aw_dev->dev, "%s: ======== end =======", __func__);
	return 0;
}

static int aw_parse_vol_data_by_0_0_1(struct aw_device *aw_dev,
			const struct firmware *cont)
{
	struct aw_monitor_hdr *monitor_hdr =
			(struct aw_monitor_hdr *)cont->data;
	struct aw_table_info *vol_info = &g_monitor_cfg.vol_info;

	aw_dev_info(aw_dev->dev, "%s: ======== start =======", __func__);
	if (vol_info->aw_table != NULL)
		kfree(vol_info->aw_table);

	vol_info->aw_table = kzalloc((monitor_hdr->vol_num * AW_TABLE_SIZE),
			GFP_KERNEL);
	if (!vol_info->aw_table) {
		aw_dev_err(aw_dev->dev, "%s:g_vol_info kzalloc faild\n", __func__);
		return -ENOMEM;
	}

	vol_info->table_num = monitor_hdr->vol_num;
	aw_populate_data_to_table(aw_dev, vol_info,
			&cont->data[monitor_hdr->vol_offset]);
	aw_dev_info(aw_dev->dev, "%s: ======== end =======", __func__);
	return 0;
}


static int aw_parse_mcfg_by_0_0_1(struct aw_device *aw_dev,
					const struct firmware *cont)
{
	int ret;

	ret = aw_check_monitor_cfg(aw_dev, cont);
	if (ret < 0) {
		aw_dev_err(aw_dev->dev, "%s:check %s failed\n",
				__func__, AW_MONITOR_FILE);
		return ret;
	}

	aw_parse_monitor_hdr_by_0_0_1(aw_dev, cont);

	ret = aw_parse_temp_data_by_0_0_1(aw_dev, cont);
	if (ret < 0)
		return ret;

	ret = aw_parse_vol_data_by_0_0_1(aw_dev, cont);
	if (ret < 0) {
		kfree(g_monitor_cfg.temp_info.aw_table);
		g_monitor_cfg.temp_info.table_num = 0;
		g_monitor_cfg.temp_info.aw_table = NULL;
		return ret;
	}

	g_monitor_cfg.monitor_status = AW_MON_CFG_OK;
	return 0;
}

static int aw_monitor_param_check_sum(struct aw_device *aw_dev,
					const struct firmware *cont)
{
	int i, check_sum = 0;
	struct aw_monitor_hdr *monitor_hdr =
		(struct aw_monitor_hdr *)cont->data;

	for(i = 4 ;i < cont->size; i++)
		check_sum += (uint8_t)cont->data[i];

	if (monitor_hdr->check_sum != check_sum) {
		aw_dev_err(aw_dev->dev, "%s:check_sum[%d] is not equal to actual check_sum[%d]",
			__func__, monitor_hdr->check_sum, check_sum);
		return -ENOMEM;
	}

	return 0;
}


static int aw_parse_monitor_cfg(struct aw_device *aw_dev,
					const struct firmware *cont)
{
	struct aw_monitor_hdr *monitor_hdr =
			(struct aw_monitor_hdr *)cont->data;
	int ret;

	ret = aw_monitor_param_check_sum(aw_dev, cont);
	if (ret)
		return ret;

	switch (monitor_hdr->monitor_ver) {
	case AW_MONITOR_HDR_VER_0_0_1:
		return aw_parse_mcfg_by_0_0_1(aw_dev, cont);
	default:
		aw_dev_err(aw_dev->dev, "%s:cfg version:0x%x unsupported\n",
				__func__, monitor_hdr->monitor_ver);
		return -EINVAL;
	}

}

static void aw_monitor_cfg_loaded(const struct firmware *cont,
					void *context)
{
	struct aw_device *aw_dev = context;
	int ret;

	mutex_lock(&g_aw_monitor_lock);
	if (g_monitor_cfg.monitor_status == AW_MON_CFG_ST) {
		if (!cont) {
			aw_dev_err(aw_dev->dev, "%s:failed to read %s\n",
					__func__, AW_MONITOR_FILE);
			goto exit;
		}

		aw_dev_info(aw_dev->dev, "%s: loaded %s - size: %zu\n",
			__func__, AW_MONITOR_FILE, cont ? cont->size : 0);

		ret = aw_parse_monitor_cfg(aw_dev, cont);
		if (ret < 0)
			aw_dev_err(aw_dev->dev, "%s:parse monitor cfg failed\n",
				__func__);
	}
exit:
	release_firmware(cont);
	mutex_unlock(&g_aw_monitor_lock);
}

int aw_load_monitor_cfg(struct aw_monitor_desc *monitor_desc)
{
	int ret;
	struct aw_device *aw_dev =
		container_of(monitor_desc, struct aw_device, monitor_desc);

	if (!monitor_desc->is_enable) {
		aw_dev_info(aw_dev->dev, "%s: monitor flag:%d, monitor bin noload\n",
			__func__, monitor_desc->is_enable);
		ret = 0;
	} else {
		ret = request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG,
				AW_MONITOR_FILE,
				aw_dev->dev, GFP_KERNEL, aw_dev,
				aw_monitor_cfg_loaded);
	}

	return ret;
}

void aw_deinit_monitor_cfg(struct aw_monitor_desc *monitor_desc)
{
	g_monitor_cfg.monitor_status = AW_MON_CFG_ST;

	if (g_monitor_cfg.temp_info.aw_table != NULL)
		kfree(g_monitor_cfg.temp_info.aw_table);

	if (g_monitor_cfg.vol_info.aw_table != NULL)
		kfree(g_monitor_cfg.vol_info.aw_table);

	memset(&g_monitor_cfg, 0, sizeof(g_monitor_cfg));


}

/*****************************************************
 * monitor init
 *****************************************************/
#ifdef AW_DEBUG
static ssize_t aw_vol_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct aw882xx *aw882xx = dev_get_drvdata(dev);
	struct aw_device *aw_dev = aw882xx->aw_pa;
	uint32_t vol = 0;
	int ret = -1;

	if (count == 0)
		return 0;

	ret = kstrtouint(buf, 0, &vol);
	if (ret < 0)
		return ret;

	aw_dev_info(aw_dev->dev, "%s: vol set =%d\n", __func__, vol);
	aw_dev->monitor_desc.test_vol = vol;

	return count;
}

static ssize_t aw_vol_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct aw882xx *aw882xx = dev_get_drvdata(dev);
	struct aw_device *aw_dev = aw882xx->aw_pa;
	ssize_t len = 0;

	len += snprintf(buf+len, PAGE_SIZE-len,
		"vol: %d\n",
		aw_dev->monitor_desc.test_vol);
	return len;
}

static ssize_t aw_temp_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct aw882xx *aw882xx = dev_get_drvdata(dev);
	struct aw_device *aw_dev = aw882xx->aw_pa;
	int32_t temp = 0;
	int ret = -1;

	if (count == 0)
		return 0;

	ret = kstrtoint(buf, 0, &temp);
	if (ret < 0)
		return ret;

	aw_dev_info(aw_dev->dev, "%s: temp set =%d\n",
		__func__, temp);

	aw_dev->monitor_desc.test_temp = temp;

	return count;
}

static ssize_t aw_temp_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct aw882xx *aw882xx = dev_get_drvdata(dev);
	struct aw_device *aw_dev = aw882xx->aw_pa;
	ssize_t len = 0;

	len += snprintf(buf+len, PAGE_SIZE-len,
		"aw882xx vol: %d\n",
		aw_dev->monitor_desc.test_temp);

	return len;
}

static DEVICE_ATTR(vol, S_IWUSR | S_IRUGO,
	aw_vol_show, aw_vol_store);
static DEVICE_ATTR(temp, S_IWUSR | S_IRUGO,
	aw_temp_show, aw_temp_store);
#endif

static ssize_t aw_monitor_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct aw882xx *aw882xx = dev_get_drvdata(dev);
	struct aw_device *aw_dev = aw882xx->aw_pa;
	uint32_t enable = 0;
	int ret = -1;

	if (count == 0)
		return 0;

	ret = kstrtouint(buf, 0, &enable);
	if (ret < 0)
		return ret;

	aw_dev_info(aw_dev->dev, "%s:monitor enable set =%d\n",
		__func__, enable);

	if (aw_dev->monitor_desc.is_enable == enable) {
		return count;
	} else {
		aw_dev->monitor_desc.is_enable = enable;
		if (enable)
			aw_monitor_start(&aw_dev->monitor_desc);
	}

	return count;
}

static ssize_t aw_monitor_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct aw882xx *aw882xx = dev_get_drvdata(dev);
	struct aw_device *aw_dev = aw882xx->aw_pa;
	ssize_t len = 0;


	len += snprintf(buf+len, PAGE_SIZE-len,
		"monitor enable: %d\n",
		aw_dev->monitor_desc.is_enable);
	return len;
}

static ssize_t aw_monitor_update_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct aw882xx *aw882xx = dev_get_drvdata(dev);
	struct aw_device *aw_dev = aw882xx->aw_pa;
	uint32_t update = 0;
	int ret = -1;

	if (count == 0)
		return 0;

	ret = kstrtouint(buf, 0, &update);
	if (ret < 0)
		return ret;

	aw_dev_info(aw_dev->dev, "%s:monitor update = %d\n",
		__func__, update);

	if (update) {
		mutex_lock(&g_aw_monitor_lock);
		aw_deinit_monitor_cfg(&aw_dev->monitor_desc);
		aw_load_monitor_cfg(&aw_dev->monitor_desc);
		mutex_unlock(&g_aw_monitor_lock);
	}

	return count;
}

static DEVICE_ATTR(monitor, S_IWUSR | S_IRUGO,
	aw_monitor_show, aw_monitor_store);
static DEVICE_ATTR(monitor_update, S_IWUSR,
	NULL, aw_monitor_update_store);


static struct attribute *aw_monitor_attr[] = {
	&dev_attr_monitor.attr,
	&dev_attr_monitor_update.attr,
#ifdef AW_DEBUG
	&dev_attr_vol.attr,
	&dev_attr_temp.attr,
#endif
	NULL
};

static struct attribute_group aw_monitor_attr_group = {
	.attrs = aw_monitor_attr,
};

void aw_parse_monitor_dt(struct aw_device *aw_dev)
{
	int ret;
	struct aw_monitor_desc *monitor_desc = &aw_dev->monitor_desc;
	struct device_node *np = aw_dev->dev->of_node;

	ret = of_property_read_u32(np, "monitor-flag", &monitor_desc->is_enable);
	if (ret) {
		monitor_desc->is_enable = AW_MONITOR_DEFAULT_FLAG;
		dev_err(aw_dev->dev,
			"%s: monitor-flag get failed ,user default value!\n",
			__func__);
	} else {
		dev_info(aw_dev->dev, "%s: monitor-flag = %d\n",
			__func__, monitor_desc->is_enable);
	}
}

static void aw_monitor_reg_init(struct aw_monitor_desc *monitor_desc)
{
	monitor_desc->voltage_desc.reg = AW882XX_VBAT_REG;
	monitor_desc->voltage_desc.vbat_range = AW882XX_MONITOR_VBAT_RANGE;
	monitor_desc->voltage_desc.int_bit = AW882XX_MONITOR_INT_10BIT;

	monitor_desc->temp_desc.reg = AW882XX_TEMP_REG;
	monitor_desc->temp_desc.sign_mask = AW882XX_MONITOR_TEMP_SIGN_MASK;
	monitor_desc->temp_desc.neg_mask = AW882XX_MONITOR_TEMP_NEG_MASK;

	monitor_desc->gain_desc.reg = AW882XX_HAGCCFG4_REG;
	monitor_desc->gain_desc.mask = AW882XX_BIT_HAGCCFG4_GAIN_MASK;
	monitor_desc->gain_desc.shift = AW882XX_BIT_HAGCCFG4_GAIN_SHIFT;

	monitor_desc->ipeak_desc.reg = AW882XX_SYSCTRL2_REG;
	monitor_desc->ipeak_desc.mask = AW882XX_BIT_SYSCTRL2_BST_IPEAK_MASK;
}

void aw_monitor_init(struct aw_monitor_desc *monitor_desc)
{
	int ret;
	struct aw_device *aw_dev = container_of(monitor_desc,
				struct aw_device, monitor_desc);
	struct aw882xx *aw882xx = dev_get_drvdata(aw_dev->dev);

	aw_dev_info(aw_dev->dev, "%s: enter\n", __func__);

	monitor_desc->pre_vol = 9000;
	monitor_desc->vol_ipeak = 0x08;
	monitor_desc->vol_gain = 0x00;

	monitor_desc->pre_temp = 400;
	monitor_desc->temp_ipeak = 0x08;
	monitor_desc->temp_gain = 0x00;
#ifdef AW_DEBUG
	monitor_desc->test_vol = 0;
	monitor_desc->test_temp = 0;
#endif

	aw_monitor_reg_init(monitor_desc);
	aw_parse_monitor_dt(aw_dev);

	INIT_DELAYED_WORK(&aw882xx->monitor_work, aw_monitor_work_func);

	ret = sysfs_create_group(&aw_dev->dev->kobj,
				&aw_monitor_attr_group);
	if (ret < 0)
		aw_dev_err(aw_dev->dev, "%s error creating sysfs attr files\n",
			__func__);
}

void aw_monitor_deinit(struct aw_monitor_desc *monitor_desc)
{
	struct aw_device *aw_dev =
		container_of(monitor_desc, struct aw_device, monitor_desc);

	mutex_lock(&g_aw_monitor_lock);
	aw_monitor_stop(monitor_desc);
	aw_deinit_monitor_cfg(monitor_desc);
	mutex_unlock(&g_aw_monitor_lock);

	sysfs_remove_group(&aw_dev->dev->kobj, &aw_monitor_attr_group);
}

