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


#include "aw_data_type.h"
#include "aw_log.h"
#include "aw_device.h"
#include "aw_dsp.h"

#define AW_DEV_SYSST_CHECK_MAX   (10)

enum {
	AW_EXT_DSP_WRITE_NONE = 0,
	AW_EXT_DSP_WRITE,
};

static char *profile_name[AW_PROFILE_MAX] = {"Music", "Voice", "Voip", "Ringtone", "Ringtone_hs", "lowpower",
						"bypass", "mmi", "fm", "Notification", "receiver"};


static char ext_dsp_prof_write = AW_EXT_DSP_WRITE_NONE;
static DEFINE_MUTEX(g_ext_dsp_prof_wr_lock);	//lock ext wr flag

/*********************************awinic acf*************************************************/
int aw_load_acf_check(aw_acf_t *acf_data, int data_size)
{
	acf_hdr_t *hdr;
	uint32_t hdr_sum_size = 0;
	uint32_t sec_data_size = 0;
	acf_shdr_t *shdr_table = NULL;
	int i;

	if (acf_data == NULL) {
		pr_err("[Awinic] %s: data buffer is NULL \n", __func__);
		return -ENOMEM;
	}

	//check data size > hdr size
	if (data_size < sizeof(aw_acf_t)) {
		pr_err("[Awinic] %s: data size is smaller than min value [%d] \n", __func__, (int)sizeof(aw_acf_t));
		return -ENOEXEC;
	}

	//check file type id is awinic acf file
	if (acf_data->hdr.a_id != ACF_FILE_ID) {
		pr_err("[Awinic] %s: not acf type file", __func__);
		return -EINVAL;
	}

	hdr = &acf_data->hdr;

	//check section table offset not overflow
	if (hdr->a_sh_offset > data_size) {
		pr_err("[Awinic] %s:a_sh_offset[%d] overflow file size[%d]!", __func__, hdr->a_sh_offset, data_size);
		return -EINVAL;
	}

	//check data size > hdr + shdr_table
	hdr_sum_size = hdr->a_sh_offset + sizeof(acf_shdr_t) * hdr->a_entry_num;
	if (hdr_sum_size > data_size) {
		pr_err("[Awinic] %s: sec table[%d] overflow file size[%d]!", __func__, hdr_sum_size, data_size);
		return -EINVAL;
	}

	// check section entry is acf_shdr_t
	if (hdr->a_entry_size != sizeof(acf_shdr_t)) {
		pr_err("[Awinic] %s:a_entry_size[%d] != request[%d]!", __func__, hdr->a_entry_size, (int)sizeof(acf_shdr_t));
		return -EINVAL;
	}

	//get data section sum size
	shdr_table = (acf_shdr_t *)((char *)acf_data + hdr->a_sh_offset);
	for (i = 0; i < hdr->a_entry_num; i++) {
		sec_data_size += shdr_table[i].a_size;
	}

	//check data size
	hdr_sum_size = sizeof(acf_hdr_t) + sizeof(acf_shdr_t) * hdr->a_entry_num;
	if (sec_data_size > (data_size - hdr_sum_size)) {
		pr_err("[Awinic] %s: sum_sec_len[%d] > file section size[%d]!",
			__func__, sec_data_size, (data_size - hdr_sum_size));
		return -EINVAL;
	}

	/* crc check */
	/*for (i = sizeof(Acf_hdr_t); i < bytes; i++) {
	    crc_sum += (int8_t)param_buf[i];
	}
	if (crc_sum != param_hdr->crc_data) {
	    AWLOGD("crc check failed, read crc = %d, local_crc = %d !", param_hdr->crc_data, crc_sum);
	    return AW_FAIL;
	}*/

	pr_info("[Awinic] %s: project name [%s] \n", __func__, hdr->a_project);
	pr_info("[Awinic] %s: custom name [%s] \n", __func__, hdr->a_custom);
	pr_info("[Awinic] %s: version name [%d.%d.%d.%d] \n", __func__, hdr->a_version[0], hdr->a_version[1], hdr->a_version[2], hdr->a_version[3]);
	pr_info("[Awinic] %s: author id %d \n", __func__, hdr->a_author_id);


	return 0;
}

static struct aw_prof_desc *aw_dev_prof_desc_init(struct aw_device *aw_dev, int desc_num)
{
	int i, j;
	struct aw_prof_desc *prof_desc = NULL;

	prof_desc = kzalloc(sizeof(struct aw_prof_desc) * desc_num, GFP_KERNEL);
	if (prof_desc == NULL) {
		aw_dev_err(aw_dev->dev, "%s: pro_desc is NULL \n", __func__);
		return NULL;
	}

	for (i = 0; i < desc_num; i++) {
		prof_desc[i].id = AW_PROFILE_MAX;
		for (j = 0; j < AW_PROFILE_DATA_TYPE_MAX; j++) {
			prof_desc[i].sec_desc[j].len = 0;
			prof_desc[i].sec_desc[j].data = NULL;
		}
	}
	return prof_desc;
}

static int aw_dev_acf_reg_load(struct aw_device *aw_dev, acf_shdr_t *sec_table, char *base_offset)
{
	int i;
	int profile_num = aw_dev->prof_info.count;
	struct aw_prof_desc *prof_desc = aw_dev->prof_info.desc;

	for (i = 0; i < profile_num; i++) {
		if (sec_table[i].info.reg_info.profile_id >= AW_PROFILE_MAX) {
			aw_dev_err(aw_dev->dev, "%s: profile [%d] is not support \n",
				__func__, sec_table[i].info.reg_info.profile_id);
			return -EINVAL;
		}
		prof_desc[i].id = sec_table[i].info.reg_info.profile_id;
		prof_desc[i].sec_desc[AW_PROFILE_DATA_TYPE_REG].data = sec_table[i].a_offset + base_offset;
		prof_desc[i].sec_desc[AW_PROFILE_DATA_TYPE_REG].len = sec_table[i].a_size;
		aw_dev_dbg(aw_dev->dev, "%s: [%s] data_ptr %p, len %d \n", __func__,
			profile_name[prof_desc[i].id], prof_desc[i].sec_desc[AW_PROFILE_DATA_TYPE_REG].data,
			prof_desc[i].sec_desc[AW_PROFILE_DATA_TYPE_REG].len);
	}

	return 0;
}

static int aw_dev_acf_dsp_load(struct aw_device *aw_dev, acf_shdr_t *sec_table, char *base_offset)
{
	int i;
	int profile_num = aw_dev->prof_info.count;
	struct aw_prof_desc *prof_desc = aw_dev->prof_info.desc;

	for (i = 0; i < profile_num; i++) {
		if (sec_table[i].info.dsp_info.profile_id >= AW_PROFILE_MAX) {
			aw_dev_err(aw_dev->dev, "%s: profile [%d] is not support \n",
				__func__, sec_table[i].info.dsp_info.profile_id);
			return -EINVAL;
		}
		prof_desc[i].id = sec_table[i].info.dsp_info.profile_id;
		prof_desc[i].sec_desc[AW_PROFILE_DATA_TYPE_DSP].data = sec_table[i].a_offset + base_offset;
		prof_desc[i].sec_desc[AW_PROFILE_DATA_TYPE_DSP].len = sec_table[i].a_size;
		aw_dev_dbg(aw_dev->dev, "%s: [%s] data_ptr %p, len %d \n", __func__,
			profile_name[prof_desc[i].id], prof_desc[i].sec_desc[AW_PROFILE_DATA_TYPE_DSP].data,
			prof_desc[i].sec_desc[AW_PROFILE_DATA_TYPE_DSP].len);
	}

	return 0;
}


static int aw_dev_reg_raw_load(struct aw_device *aw_dev, aw_acf_t *acf_data)
{
	int profile_num = acf_data->hdr.a_entry_num;
	char *base_offset = (char *)&acf_data->hdr;
	acf_shdr_t *reg_sec_table = (acf_shdr_t *)(base_offset + acf_data->hdr.a_sh_offset);
	int ret;

	struct aw_prof_desc *prof_desc = NULL;

	prof_desc = aw_dev_prof_desc_init(aw_dev, profile_num);
	if (prof_desc == NULL) {
		aw_dev_err(aw_dev->dev, "%s: init prof_desc failed \n", __func__);
		return -EINVAL;
	}
	aw_dev->prof_info.desc = prof_desc;
	aw_dev->prof_info.count = profile_num;

	ret = aw_dev_acf_reg_load(aw_dev, reg_sec_table, base_offset);
	if (ret) {
		aw_dev_err(aw_dev->dev, "%s: load reg data failed \n", __func__);
		kfree(prof_desc);
		aw_dev->prof_info.desc = NULL;
		aw_dev->prof_info.count = 0;
		return -EINVAL;
	}
	aw_dev_dbg(aw_dev->dev, "%s: load reg data done\n", __func__);

	return 0;
}

static int aw_dev_find_sec(struct aw_device *aw_dev, acf_shdr_t *sec_tbl, int sec_num)
{
	int dev_num = aw_dev->index;
	int i;

	for (i = 0; i < sec_num; i++) {
		if (sec_tbl[i].a_type == ACF_SEC_TYPE_DEVICE) {
			if (sec_tbl[i].info.dev_info.dev_index == dev_num) {
				aw_dev_dbg(aw_dev->dev,"find dev[%d] section \n", dev_num);
				return i;
			}
		}
	}
	aw_dev_dbg(aw_dev->dev,"can not find dev[%d] section \n", dev_num);
	return -EINVAL;
}

static int aw_dev_find_dsp_sec(struct aw_device *aw_dev, acf_shdr_t *sec_tbl, int sec_num)
{
	int dev_num = aw_dev->index;
	int i;

	for (i = 0; i < sec_num; i++) {
		if (sec_tbl[i].a_type == ACF_SEC_TYPE_DSP) {
			return i;
		}
	}
	aw_dev_dbg(aw_dev->dev,"can not find dev[%d] section \n", dev_num);
	return -EINVAL;
}


static int aw_dev_load_prof_acf(struct aw_device *aw_dev, aw_acf_t *acf_data)
{
	char *base_offset = (char *)&acf_data->hdr;
	acf_shdr_t *sec_table = (acf_shdr_t *)(base_offset + acf_data->hdr.a_sh_offset);
	acf_shdr_t *reg_sec_tbl = NULL;
	acf_shdr_t *dsp_sec_tbl = NULL;
	int sec_num = acf_data->hdr.a_entry_num;
	int index = 0, ret;
	int prof_num;
	struct aw_prof_desc *prof_desc = NULL;

	//get dev sec index
	index = aw_dev_find_sec(aw_dev, sec_table, sec_num);
	if (index < 0) {
		aw_dev_dbg(aw_dev->dev,"dev find reg data failed \n");
		return -EINVAL;
	}

	//get sub sec tbl and entry
	prof_num = sec_table[index].info.dev_info.prof_num;
	reg_sec_tbl = (acf_shdr_t *)(sec_table[index].a_offset + base_offset);

	//alloc prof desc
	prof_desc = aw_dev_prof_desc_init(aw_dev, prof_num);
	if (prof_desc == NULL) {
		aw_dev_err(aw_dev->dev, "%s: init prof_desc failed \n", __func__);
		return -EINVAL;
	}

	//set info
	aw_dev->prof_info.desc = prof_desc;
	aw_dev->prof_info.count = prof_num;

	//create reg acf desc
	base_offset = (char *)reg_sec_tbl;
	ret = aw_dev_acf_reg_load(aw_dev, reg_sec_tbl, base_offset);
	if (ret) {
		aw_dev_err(aw_dev->dev, "%s: load reg data failed \n", __func__);
		kfree(prof_desc);
		aw_dev->prof_info.desc = NULL;
		aw_dev->prof_info.count = 0;
		return -EINVAL;
	}

	//load dsp data
	index = aw_dev_find_dsp_sec(aw_dev, sec_table, sec_num);
	if (index < 0) {
		aw_dev_dbg(aw_dev->dev,"dev find dsp data failed \n");
	} else {
		base_offset = (char *)&acf_data->hdr;
		dsp_sec_tbl = (acf_shdr_t *)(sec_table[index].a_offset + base_offset);
		base_offset = (char *)dsp_sec_tbl;
		ret = aw_dev_acf_dsp_load(aw_dev, dsp_sec_tbl, base_offset);
		if (ret) {
			aw_dev_err(aw_dev->dev, "%s: load dsp data failed \n", __func__);
		}
	}

	aw_dev_dbg(aw_dev->dev, "%s: load reg data done\n", __func__);
	return 0;

}

static int aw_dev_scan_acf_shdr(struct aw_device *aw_dev, acf_shdr_t *shdr_tbl, int num)
{
	int type = shdr_tbl[0].a_type;
	int i;

	if (type == ACF_SEC_TYPE_REG) {
		for (i = 0; i < num; i++) {
			if (shdr_tbl[i].a_type != type) {
				aw_dev_err(aw_dev->dev, "%s: unsupported reg section", __func__);
				return -EINVAL;
			}
		}
		return ACF_SEC_TYPE_REG;
	}

	if (type == ACF_SEC_TYPE_DEVICE) {
		return ACF_SEC_TYPE_DEVICE;
	}

	return -EINVAL;
}


static int aw_dev_acf_load(struct aw_device *aw_dev, aw_acf_t *acf_data)
{
	acf_shdr_t *shdr_table = NULL;
	acf_hdr_t *acf_hdr = NULL;
	int ret;

	if (aw_dev == NULL || acf_data == NULL) {
		aw_dev_err(aw_dev->dev, "%s: aw_dev is NULL", __func__);
		return -ENOMEM;
	}

	acf_hdr = &acf_data->hdr;

	shdr_table = (acf_shdr_t *)((char *)(acf_hdr) + acf_hdr->a_sh_offset);

	ret = aw_dev_scan_acf_shdr(aw_dev, shdr_table, acf_hdr->a_entry_num);
	if (ret < 0) {
		aw_dev_err(aw_dev->dev, "%s: section type error", __func__);
		return -EINVAL;;
	}

	switch (ret) {
		case ACF_SEC_TYPE_REG : {
			ret = aw_dev_reg_raw_load(aw_dev, acf_data);
		} break;
		case ACF_SEC_TYPE_DEVICE : {
			ret = aw_dev_load_prof_acf(aw_dev, acf_data);
		} break;
		default : {
			aw_dev_err(aw_dev->dev, "%s: unsupported type [%d]", __func__, ret);
			return -EINVAL;
		}
	}
	if (ret) {
		aw_dev_err(aw_dev->dev, "%s: load acf failed", __func__);
		return ret;
	} else {
		aw_dev->acf = (char *)acf_data;
		return 0;
	}
}

static struct aw_sec_data_desc *aw_dev_get_prof_data(struct aw_device *aw_dev, int index, int data_type)
{

	struct aw_sec_data_desc * sec_data = NULL;
	struct aw_prof_desc *prof_desc = NULL;

	if (index >= aw_dev->prof_info.count) {
		aw_dev_err(aw_dev->dev, "%s: index[%d] overflow count[%d]\n",
			__func__, index, aw_dev->prof_info.count);
		return NULL;
	}

	prof_desc = &aw_dev->prof_info.desc[index];

	if (data_type >= AW_PROFILE_DATA_TYPE_MAX) {
		aw_dev_err(aw_dev->dev, "%s: unsupport data type id [%d] \n", __func__, data_type);
		return NULL;
	}

	sec_data = &prof_desc->sec_desc[data_type];

	aw_dev_dbg(aw_dev->dev, "%s:get prof[%s] data len[%d] \n", __func__, profile_name[prof_desc->id], sec_data->len);

	return sec_data;
}


/*********************************awinic device*************************************************/
static int aw_dev_update_dsp_prof(struct aw_device *aw_dev)
{
	int  ret;
	struct aw_sec_data_desc *dsp_data;

	dsp_data = aw_dev_get_prof_data(aw_dev, aw_dev->set_prof, AW_PROFILE_DATA_TYPE_DSP);
	if (dsp_data == NULL || dsp_data->data == NULL || dsp_data->len == 0) {
		aw_dev_info(aw_dev->dev, "%s: dsp data is NULL \n", __func__);
		return 0;
	}

	mutex_lock(&g_ext_dsp_prof_wr_lock);
	if (ext_dsp_prof_write == AW_EXT_DSP_WRITE_NONE) {
		ret = aw_dsp_write_params(aw_dev, dsp_data->data, dsp_data->len);
		if (ret) {
			aw_dev_err(aw_dev->dev, "%s:dsp params update failed !", __func__);
			mutex_unlock(&g_ext_dsp_prof_wr_lock);
			return ret;
		}
		ext_dsp_prof_write = AW_EXT_DSP_WRITE;
	} else {
		aw_dev_dbg(aw_dev->dev, "%s:dsp params already update !", __func__);
	}
	mutex_unlock(&g_ext_dsp_prof_wr_lock);
	return 0;
}


/*pwd enable update reg*/
int aw_dev_reg_update(struct aw_device *aw_dev)
{
	int i = 0;
	unsigned int reg_addr = 0;
	unsigned int reg_val = 0;
	unsigned int read_val;
	int ret = -1;
	unsigned int init_volume = 0;
	struct aw_int_desc *int_desc = &aw_dev->int_desc;
	struct aw_sec_data_desc *reg_data;

	reg_data = aw_dev_get_prof_data(aw_dev, aw_dev->set_prof, AW_PROFILE_DATA_TYPE_REG);
	if (reg_data == NULL) {
		return -EINVAL;
	}

	for (i = 0; i < reg_data->len; i += 4) {
		reg_addr = (reg_data->data[i+1]<<8) +
			reg_data->data[i+0];
		reg_val = (reg_data->data[i+3]<<8) +
			reg_data->data[i+2];
		aw_dev_dbg(aw_dev->dev, "%s: reg=0x%04x, val = 0x%04x\n",
			__func__, reg_addr, reg_val);
		if (reg_addr == int_desc->reg) {
			int_desc->int_reg_mask = reg_val;
			reg_val = int_desc->reg_default;
		}
		//keep pwd status
		if (reg_addr == aw_dev->pwd_desc.reg) {
			aw_dev->ops.aw_i2c_read(aw_dev,
			(unsigned char)reg_addr,
			(unsigned int *)&read_val);
			read_val &= (~aw_dev->pwd_desc.mask);
			reg_val &= aw_dev->pwd_desc.mask;
			reg_val |= read_val;
		}
		// keep mute status
		if (reg_addr == aw_dev->mute_desc.reg) {
			aw_dev->ops.aw_i2c_read(aw_dev,
			(unsigned char)reg_addr,
			(unsigned int *)&read_val);
			read_val &= (~aw_dev->mute_desc.mask);
			reg_val &= aw_dev->mute_desc.mask;
			reg_val |= read_val;
		}

		ret = aw_dev->ops.aw_i2c_write(aw_dev,
			(unsigned char)reg_addr,
			(unsigned int)reg_val);
		if (ret < 0)
			break;
	}

	aw_dev->ops.aw_get_volume(aw_dev, &init_volume);
	aw_dev->volume_desc.init_volume = init_volume;

	//keep min volume
	aw_dev->ops.aw_set_volume(aw_dev, AW_FADE_OUT_TARGET_VOL);

	aw_dev_info(aw_dev->dev, "%s: done\n", __func__);

	return ret;
}

static int aw_dev_fade_in_out(struct aw_device *aw_dev, bool fade_in)
{
	int i = 0;
	unsigned start_volume = 0;
	struct aw_volume_desc *desc = &aw_dev->volume_desc;

	//step zero means close fade in/out
	if (desc->step == 0) {
		if (fade_in) {
			aw_dev->ops.aw_set_volume(aw_dev, desc->init_volume);
		} else {
			aw_dev->ops.aw_set_volume(aw_dev, AW_FADE_OUT_TARGET_VOL);
		}
		return 0;
	}

	//volume up
	if (fade_in) {
		for (i = AW_FADE_OUT_TARGET_VOL; i >= desc->init_volume; i-= desc->step) {
			if (i < desc->init_volume) {
				i = desc->init_volume;
			}
			aw_dev->ops.aw_set_volume(aw_dev, i);
			usleep_range(desc->in_step_time, desc->in_step_time + 100);
		}
		if (i != desc->init_volume) {
			aw_dev->ops.aw_set_volume(aw_dev, desc->init_volume);
		}
	} else {  // volume down
		aw_dev->ops.aw_get_volume(aw_dev, &start_volume);
		i = start_volume;
		for (i = start_volume; i <= AW_FADE_OUT_TARGET_VOL; i+= desc->step) {
			if (i > AW_FADE_OUT_TARGET_VOL) {
				i = AW_FADE_OUT_TARGET_VOL;
			}
			aw_dev->ops.aw_set_volume(aw_dev, i);
			usleep_range(desc->out_step_time, desc->out_step_time + 100);
		}
		if (i != AW_FADE_OUT_TARGET_VOL) {
			aw_dev->ops.aw_set_volume(aw_dev, AW_FADE_OUT_TARGET_VOL);
		}
	}
	return 0;
}

static void aw_dev_pwd(struct aw_device *aw_dev, bool pwd)
{
	struct aw_pwd_desc *pwd_desc = &aw_dev->pwd_desc;
	aw_dev_dbg(aw_dev->dev, "%s: enter\n", __func__);

	if (pwd) {
		aw_dev->ops.aw_i2c_write_bits(aw_dev, pwd_desc->reg,
				pwd_desc->mask,
				pwd_desc->enable);
	} else {
		aw_dev->ops.aw_i2c_write_bits(aw_dev, pwd_desc->reg,
				pwd_desc->mask,
				pwd_desc->disable);
	}
	aw_dev_info(aw_dev->dev, "%s: done \n", __func__);
}

static void aw_dev_mute(struct aw_device *aw_dev, bool mute, int type)
{
	struct aw_mute_desc *mute_desc = &aw_dev->mute_desc;
	aw_dev_dbg(aw_dev->dev, "%s: enter\n", __func__);

	if (mute) {
		aw_dev_fade_in_out(aw_dev, false);
		aw_dev->ops.aw_i2c_write_bits(aw_dev, mute_desc->reg,
				mute_desc->mask,
				mute_desc->enable);
	} else {
		aw_dev->ops.aw_i2c_write_bits(aw_dev, mute_desc->reg,
				mute_desc->mask,
				mute_desc->disable);
		if (type == AW_START_TYPE_STREAM) {
			aw_dev->ops.aw_set_volume(aw_dev, aw_dev->volume_desc.init_volume);
		} else {
			aw_dev_fade_in_out(aw_dev, true);
		}
	}
	aw_dev_info(aw_dev->dev, "%s: done \n", __func__);
}

static int aw_dev_get_icalk(struct aw_device *aw_dev, int16_t *icalk)
{
	int ret = -1;
	unsigned int reg_val = 0;
	uint16_t reg_icalk = 0;
	struct aw_vcalb_desc *desc = &aw_dev->vcalb_desc;

	ret = aw_dev->ops.aw_i2c_read(aw_dev, desc->icalk_reg, &reg_val);
	reg_icalk = (uint16_t)reg_val & desc->icalk_reg_mask;

	if (reg_icalk & desc->icalk_sign_mask)
		reg_icalk = reg_icalk | desc->icalk_neg_mask;

	*icalk = (int16_t)reg_icalk;

	return ret;
}

static int aw_dev_get_vcalk(struct aw_device *aw_dev, int16_t *vcalk)
{
	int ret = -1;
	unsigned int reg_val = 0;
	uint16_t reg_vcalk = 0;
	struct aw_vcalb_desc *desc = &aw_dev->vcalb_desc;

	ret = aw_dev->ops.aw_i2c_read(aw_dev, desc->vcalk_reg, &reg_val);
	reg_vcalk = (uint16_t)reg_val & desc->vcalk_reg_mask;

	if (reg_vcalk & desc->vcalk_sign_mask)
		reg_vcalk = reg_vcalk | desc->vcalk_neg_mask;

	*vcalk = (int16_t)reg_vcalk;

	return ret;
}

static int aw_dev_set_vcalb(struct aw_device *aw_dev)
{
	int ret = -1;
	unsigned int reg_val;
	int vcalb;
	int icalk;
	int vcalk;
	int16_t icalk_val = 0;
	int16_t vcalk_val = 0;

	struct aw_vcalb_desc *desc = &aw_dev->vcalb_desc;

	ret = aw_dev_get_icalk(aw_dev, &icalk_val);
	ret = aw_dev_get_vcalk(aw_dev, &vcalk_val);

	icalk = desc->cabl_base_value + desc->icalk_value_factor * icalk_val;
	vcalk = desc->cabl_base_value + desc->vcalk_value_factor * vcalk_val;

	vcalb = desc->vcal_factor * icalk / vcalk;

	reg_val = (unsigned int)vcalb;
	aw_dev_dbg(aw_dev->dev, "%s: icalk=%d, vcalk=%d, vcalb=%d, reg_val=%d\n",
		__func__, icalk, vcalk, vcalb, reg_val);

	ret =  aw_dev->ops.aw_i2c_write(aw_dev, desc->vcalb_reg, reg_val);

	aw_dev_info(aw_dev->dev, "%s: done \n", __func__);

	return ret;
}

static int aw_get_int_status(struct aw_device *aw_dev, uint16_t *int_status)
{
	int ret = -1;
	unsigned int reg_val = 0;

	ret = aw_dev->ops.aw_i2c_read(aw_dev, aw_dev->int_status_reg, &reg_val);
	if (ret < 0)
		aw_dev_err(aw_dev->dev, "%s: read interrupt reg fail, ret=%d\n",
				__func__, ret);
	else
		*int_status = reg_val;

	aw_dev_dbg(aw_dev->dev, "%s: read interrupt reg = 0x%04x\n",
				__func__, *int_status);
	return ret;
}

static void aw_dev_clear_int_status(struct aw_device *aw_dev)
{
	uint16_t int_status = 0;

	//read int status and clear
	aw_get_int_status(aw_dev, &int_status);
	//make suer int status is clear
	aw_get_int_status(aw_dev, &int_status);
	aw_dev_dbg(aw_dev->dev, "%s: done \n", __func__);
}

static int aw_dev_sysst_check(struct aw_device *aw_dev)
{
	int ret = -1;
	unsigned char i;
	unsigned int reg_val = 0;
	struct aw_sysst_desc *desc = &aw_dev->sysst_desc;

	for (i = 0; i < AW_DEV_SYSST_CHECK_MAX; i++) {
		aw_dev->ops.aw_i2c_read(aw_dev, desc->reg, &reg_val);
		if (((reg_val & (~desc->mask)) & desc->check) == desc->check) {
			ret = 0;
			break;
		} else {
			aw_dev_dbg(aw_dev->dev, "%s: check fail, cnt=%d, reg_val=0x%04x\n",
				__func__, i, reg_val);
			msleep(2);
		}
	}
	if (ret < 0)
		aw_dev_info(aw_dev->dev, "%s: check fail\n", __func__);

	aw_dev_info(aw_dev->dev, "%s: done", __func__);
	return ret;
}

static int aw_dev_set_intmask(struct aw_device *aw_dev, bool flag)
{
	struct aw_int_desc *desc = &aw_dev->int_desc;
	int ret = -1;

	if (flag)
		ret = aw_dev->ops.aw_i2c_write(aw_dev, desc->reg,
					desc->int_reg_mask);
	else
		ret = aw_dev->ops.aw_i2c_write(aw_dev, desc->reg,
					desc->reg_default);
	aw_dev_dbg(aw_dev->dev, "%s: done", __func__);
	return ret;
}

int aw_dev_get_profile_count(struct aw_device *aw_dev)
{
	if (aw_dev == NULL) {
		aw_dev_err(aw_dev->dev, "%s: aw_dev is NULL", __func__);
		return -ENOMEM;
	}

	return aw_dev->prof_info.count;
}

int aw_dev_get_profile_name(struct aw_device *aw_dev, char *name, int index)
{
	int dev_profile_id;
	if (index > aw_dev->prof_info.count) {
		aw_dev_err(aw_dev->dev, "%s: index[%d] overflow dev prof num[%d]",
			__func__, index, aw_dev->prof_info.count);
		return -EINVAL;
	}

	if (aw_dev->prof_info.desc[index].id >= AW_PROFILE_MAX) {
		aw_dev_err(aw_dev->dev, "%s: can not find match id ", __func__);
		return -EINVAL;
	}

	dev_profile_id = aw_dev->prof_info.desc[index].id;

	strcpy(name, profile_name[dev_profile_id]);
	//aw_dev_dbg(aw_dev->dev, "%s: get name [%s]", __func__, profile_name[dev_profile_id]);
	return 0;
}

int aw_dev_check_profile_index(struct aw_device *aw_dev, int index)
{

	if(index >= aw_dev->prof_info.count) {
		return -EINVAL;
	} else {
		return 0;
	}
}

int aw_dev_get_profile_index(struct aw_device *aw_dev)
{
	return aw_dev->cur_prof;
}

int aw_dev_set_profile_index(struct aw_device *aw_dev, int index)
{

	if(index >= aw_dev->prof_info.count) {
		return -EINVAL;
	} else {
		aw_dev->set_prof = index;
		if (aw_dev->status == AW_DEV_PW_OFF)
			aw_dev->cur_prof = index;
		
		aw_dev_info(aw_dev->dev, "%s: set prof[%s]", __func__,
			profile_name[aw_dev->prof_info.desc[index].id]);
	}
	mutex_lock(&g_ext_dsp_prof_wr_lock);
	ext_dsp_prof_write = AW_EXT_DSP_WRITE_NONE;
	mutex_unlock(&g_ext_dsp_prof_wr_lock);

	return 0;
}

int aw_dev_get_status(struct aw_device *aw_dev)
{
	return aw_dev->status;
}

int aw_dev_get_volume_step(struct aw_device *aw_dev)
{
	return aw_dev->volume_desc.step;
}

void aw_dev_set_volume_step(struct aw_device *aw_dev, unsigned int step)
{
	aw_dev->volume_desc.step =step;
}

//init aw_device
void aw_dev_deinit(struct aw_device *aw_dev)
{
	if (aw_dev->prof_info.desc != NULL) {
		kfree(aw_dev->prof_info.desc);
		aw_dev->prof_info.desc = NULL;
		aw_dev->prof_info.count = 0;
	}
}

unsigned int aw_dev_get_dsp_re(struct aw_device *aw_dev)
{
	int32_t dsp_re = 0;
	int ret;

	ret = aw_dsp_read_cali_re(aw_dev, (char *)&dsp_re, sizeof(int32_t));
	if (ret) {
		return AW_ERRO_CALI_VALUE;
	}
	dsp_re = AW_DSP_RE_TO_SHOW_RE(dsp_re);

	return dsp_re;
}

int aw_device_init(struct aw_device *aw_dev, aw_acf_t *acf_data)
{
	//acf_hdr_t *hdr;
	int ret;

	if (aw_dev == NULL) {
		aw_dev_err(aw_dev->dev, "%s: aw_dev is NULL \n", __func__);
		return -ENOMEM;
	}

	ret = aw_dev_acf_load(aw_dev, acf_data);
	if (ret) {
		aw_dev_deinit(aw_dev);
		aw_dev_err(aw_dev->dev, "%s: aw_dev acf load failed \n", __func__);
		return -EINVAL;
	}

	aw_dev->cur_prof = AW_PROFILE_MUSIC;
	aw_dev->set_prof = AW_PROFILE_MUSIC;
	ret = aw_dev_reg_update(aw_dev);
	if (ret < 0) {
		return ret;
	}

	aw_dev_set_vcalb(aw_dev);
	aw_dev->status = AW_DEV_PW_ON;
	aw_device_stop(aw_dev);

	ret = aw_cali_read_re_from_nvram(&aw_dev->cali_desc.cali_re, aw_dev->channel);
	if (ret) {
		aw_dev->cali_desc.cali_re = AW_ERRO_CALI_VALUE;
	}

	aw_dev_info(aw_dev->dev, "%s: init done \n", __func__);
	return 0;
}


int aw_dev_load_fw(struct aw_device *aw_dev)
{
	int ret;

	if (aw_dev->status == AW_DEV_PW_ON) {
		aw_dev_dbg(aw_dev->dev, "%s: already power on\n", __func__);
		return 0;
	}

	//reg switch
	if (aw_dev->cur_prof != aw_dev->set_prof) {
		ret = aw_dev_reg_update(aw_dev);
		if (ret < 0) {
			return ret;
		}
	}

	aw_dev_info(aw_dev->dev, "%s: done \n", __func__);
	return 0;
}

int aw_dev_load_dsp(struct aw_device *aw_dev)
{
	if (aw_dev->status == AW_DEV_PW_ON) {
		aw_dev_dbg(aw_dev->dev, "%s: already power on\n", __func__);
		return 0;
	}

	aw_dev_update_dsp_prof(aw_dev);
	aw_dev->cur_prof = aw_dev->set_prof;

	aw_dev_info(aw_dev->dev, "%s: done \n", __func__);
	return 0;
}

int aw_dev_prof_update(struct aw_device *aw_dev)
{
	int ret;

	if (aw_dev->status == AW_DEV_PW_ON) {
		aw_device_stop(aw_dev);
	}

	ret = aw_dev_load_fw(aw_dev);
	if (ret) {
		aw_dev_err(aw_dev->dev, "%s: reg update failed \n", __func__);
		return ret;
	}

	ret = aw_device_start(aw_dev, AW_START_TYPE_PROF);
	if (ret) {
		aw_dev_err(aw_dev->dev, "%s: start failed \n", __func__);
		return ret;
	}

	aw_dev_info(aw_dev->dev, "%s: update done !", __func__);
	return 0;
}


int aw_device_start(struct aw_device *aw_dev, int type)
{
	//struct aw882xx *aw882xx = aw_dev->private_data;
	int ret;
	aw_dev_dbg(aw_dev->dev, "%s: enter\n", __func__);

	if (aw_dev->status == AW_DEV_PW_ON) {
		aw_dev_dbg(aw_dev->dev, "%s: already power on\n", __func__);
		return 0;
	}

	aw_dev_load_dsp(aw_dev);

	// power on
	aw_dev_pwd(aw_dev, false);
	//clear inturrupt
	aw_dev_clear_int_status(aw_dev);
	//set inturrupt mask
	aw_dev_set_intmask(aw_dev, true);

	//check i2s status
	ret = aw_dev_sysst_check(aw_dev);
	if (ret < 0) {				//check failed
		//clear interrupt
		aw_dev_clear_int_status(aw_dev);
		//set inturrupt mask default
		aw_dev_set_intmask(aw_dev, false);
		//set mute
		aw_dev_mute(aw_dev, true, type);
		//power down
		aw_dev_pwd(aw_dev, true);
		aw_dev->status = AW_DEV_PW_OFF;
		return -EINVAL;
	} else {
		//close mute
		aw_dev_mute(aw_dev, false, type);
		aw_dev->status = AW_DEV_PW_ON;
	}

	/*zhaolei*/
	aw_monitor_start(&aw_dev->monitor_desc);

	if (AW_ERRO_CALI_VALUE != aw_dev->cali_desc.cali_re) {
		aw_dsp_write_cali_re(aw_dev, (char *)&aw_dev->cali_desc.cali_re, sizeof(uint32_t));
	}

	aw_dev_info(aw_dev->dev, "%s: done \n", __func__);
	return 0;
}

int aw_device_stop(struct aw_device *aw_dev)
{
	//struct aw882xx *aw882xx = aw_dev->private_data;

	aw_dev_dbg(aw_dev->dev, "%s: enter\n", __func__);

	if (aw_dev->status == AW_DEV_PW_OFF) {
		aw_dev_dbg(aw_dev->dev, "%s: already power off\n", __func__);
		return 0;
	}

	aw_dev->status = AW_DEV_PW_OFF;

	aw_monitor_stop(&aw_dev->monitor_desc);
	//clear interrupt
	aw_dev_clear_int_status(aw_dev);

	//set defaut int mask
	aw_dev_set_intmask(aw_dev, false);

	//set mute
	aw_dev_mute(aw_dev, true, 0);

	//set power down
	aw_dev_pwd(aw_dev, true);

	aw_dev_info(aw_dev->dev, "%s: done \n", __func__);
	return 0;
}


static int aw_device_parse_dt(struct aw_device *aw_dev)
{
	int ret;
	uint32_t channel_value;

	aw_dev->channel = AW_DEV_CH_PRI_L;
	ret = of_property_read_u32(aw_dev->dev->of_node, "sound-channel", &channel_value);
	if (ret < 0) {
		aw_dev_info(aw_dev->dev,
			"%s:read sound-channel failed,use default\n", __func__);
		return -EINVAL;
	}

	aw_dev_dbg(aw_dev->dev,
		"%s: read sound-channel value is : %d\n",
			__func__, channel_value);
	if (channel_value >= AW_DEV_CH_MAX) {
		channel_value = AW_DEV_CH_PRI_L;
	}
	aw_dev->channel = channel_value;

	return 0; 
}

int aw_device_probe(struct aw_device *aw_dev)
{
	aw_dev->ops.aw_get_afe_module_en = aw_dsp_get_afe_module_en;
	aw_dev->ops.aw_set_afe_module_en = aw_dsp_set_afe_module_en;
	aw_dev->ops.aw_set_copp_module_en = aw_dsp_set_copp_module_en;
	aw_dev->ops.aw_set_spin_mode = aw_dsp_write_spin;
	aw_dev->ops.aw_get_spin_mode = aw_dsp_read_spin;
	aw_device_parse_dt(aw_dev);
	aw_cali_init(&aw_dev->cali_desc);

	aw_monitor_init(&aw_dev->monitor_desc);
	return 0;
}

int aw_device_remove(struct aw_device *aw_dev)
{
	aw_monitor_deinit(&aw_dev->monitor_desc);
	aw_cali_deinit(&aw_dev->cali_desc);
	return 0;
}

