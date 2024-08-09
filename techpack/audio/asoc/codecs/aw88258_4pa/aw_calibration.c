/*
 * aw_calibration.c cali_module
 *
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
#include <linux/debugfs.h>
#include <asm/ioctls.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/version.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#include "aw882xx.h"
#include "aw_calibration.h"
#include "aw_dsp.h"
#include "aw_log.h"



 /*****************************cali re show/store***************************************************/
#ifdef AW_CALI_STORE_EXAMPLE
 /*write cali to persist file example*/
#define AW_CALI_FILE  "/mnt/vendor/persist/factory/audio/aw_cali.bin"
#define AW_INT_DEC_DIGIT (10)

static int aw_cali_write_re_to_file(int32_t cali_re, int ch_index)
{
	struct file *fp;
	char buf[50] = {0};
	loff_t pos = 0;
	mm_segment_t fs;

	fp = filp_open(AW_CALI_FILE, O_RDWR | O_CREAT, 0644);
	if (IS_ERR(fp)) {
		pr_err("%s:channel:%d open %s failed!\n",
			 __func__, ch_index, AW_CALI_FILE);
		return -EINVAL;
	}

	pos = ch_index * AW_INT_DEC_DIGIT;

	cali_re = AW_DSP_RE_TO_SHOW_RE(cali_re);
	snprintf(buf, PAGE_SIZE, "%10d", cali_re);

	fs = get_fs();
	set_fs(KERNEL_DS);

	vfs_write(fp, buf, strlen(buf), &pos);

	set_fs(fs);

	pr_info("%s: channel:%d buf:%s cali_re:%d\n",
			__func__, ch_index, buf, cali_re);

	filp_close(fp, NULL);
	return 0;
}

static int aw_cali_get_re_from_file(int32_t *cali_re, int ch_index)
{
	struct file *fp;
	/*struct inode *node;*/
	int f_size;
	char *buf;
	int32_t int_cali_re = 0;

	loff_t pos = 0;
	mm_segment_t fs;

	fp = filp_open(AW_CALI_FILE, O_RDONLY, 0);
	if (IS_ERR(fp)) {
		pr_err("%s:channel:%d open %s failed!\n",
			 __func__, ch_index, AW_CALI_FILE);
		return -EINVAL;
	}

	 pos = ch_index * AW_INT_DEC_DIGIT;

	/*node = fp->f_dentry->d_inode;*/
	/*f_size = node->i_size;*/
	f_size = AW_INT_DEC_DIGIT;

	buf = kzalloc(f_size + 1, GFP_ATOMIC);
	if (!buf) {
		pr_err("%s: channel:%d malloc mem %d failed!\n",
		 __func__, ch_index, f_size);
		filp_close(fp, NULL);
		return -EINVAL;
	}

	fs = get_fs();
	set_fs(KERNEL_DS);

	vfs_read(fp, buf, f_size, &pos);

	set_fs(fs);

	if (sscanf(buf, "%d", &int_cali_re) == 1)
		*cali_re = AW_SHOW_RE_TO_DSP_RE(int_cali_re);
	else
		*cali_re = AW_ERRO_CALI_VALUE;

	pr_info("%s: channel:%d buf:%s int_cali_re: %d\n",
		__func__, ch_index, buf, int_cali_re);

	kfree(buf);
	filp_close(fp, NULL);

	return 0;
}
#endif

 /*custom need add to set/get cali_re form/to nv*/
int aw_cali_write_re_to_nvram(int32_t cali_re, int32_t ch_index)
{
	/*custom add, if success return value is 0, else -1*/
#ifdef AW_CALI_STORE_EXAMPLE
	return aw_cali_write_re_to_file(cali_re, ch_index);
#else
	return -EBUSY;
#endif
}
int aw_cali_read_re_from_nvram(int32_t *cali_re, int32_t ch_index)
{
	/*custom add, if success return value is 0 , else -1*/
#ifdef AW_CALI_STORE_EXAMPLE
	return aw_cali_get_re_from_file(cali_re, ch_index);
#else
	return -EBUSY;
#endif
}

int aw_cali_store_cali_re(struct aw_device *aw_dev, int32_t re)
{
	aw_dev->cali_desc.cali_re = re;
	aw_dev_info(aw_dev->dev, "%s : set aw_dev->cali_desc.cali_re=%d !\n", __func__, re);
	return 0;
}

static void aw_cali_set_cali_status(struct aw_device *aw_dev, int status)
{
	if (status) {
		aw_dev->cali_desc.status = true;
	} else {
		aw_dev->cali_desc.status = false;
	}

	aw_dev_info(aw_dev->dev, "%s:cali %s", __func__,
		(status == 0) ? ("disable") : ("enable"));
}

/*****************************misc device start***************************************************/
static int aw_misc_cali_ops(struct aw_device *aw_dev,
			unsigned int cmd, unsigned long arg);

static int aw_misc_file_open(struct inode *inode, struct file *file)
{
	struct miscdevice *device;
	struct aw_cali_desc *cali_ptr = NULL;
	struct aw_device *aw_dev;

	if (!try_module_get(THIS_MODULE))
		return -ENODEV;

	device = (struct miscdevice *)file->private_data;

	cali_ptr = container_of(device, struct aw_cali_desc, misc);
	aw_dev = container_of(cali_ptr, struct aw_device, cali_desc);

	file->private_data = (void *)aw_dev;

	aw_dev_dbg(aw_dev->dev, "%s: misc open success\n", __func__);
	return 0;
}

static int aw_misc_file_release(struct inode *inode, struct file *file)
{
	file->private_data = (void *)NULL;

	pr_debug("misc release successi\n");
	return 0;
}

static int aw_misc_cali_ops_write(struct aw_device *aw_dev,
			unsigned int cmd, unsigned long arg)
{
	unsigned int data_len = _IOC_SIZE(cmd);
	char *data_ptr = NULL;
	int ret = 0;

	aw_dev_info(aw_dev->dev, "%s : opcode %d !\n", __func__, cmd);
	
	data_ptr = kmalloc(data_len, GFP_KERNEL);
	if (!data_ptr) {
		aw_dev_err(aw_dev->dev, "%s : malloc failed !\n", __func__);
		return -ENOMEM;
	}

	if (copy_from_user(data_ptr, (void __user *)arg, data_len)) {
		ret = -EFAULT;
		goto exit;
	}

	switch (cmd) {
		case AW_IOCTL_ENABLE_CALI : {
			aw_cali_set_cali_status(aw_dev, data_ptr[0]);
		} break;
		case AW_IOCTL_SET_CALI_CFG : {
			ret = aw_dsp_write_cali_cfg(aw_dev, data_ptr, data_len);
		} break;
		case AW_IOCTL_SET_NOISE : {
			ret = aw_dsp_write_noise(aw_dev, data_ptr, data_len);
		} break;
		case AW_IOCTL_SET_VMAX : {
			ret = aw_dsp_write_vmax(aw_dev, data_ptr, data_len);
		} break;
		case AW_IOCTL_SET_PARAM : {
			ret = aw_dsp_write_params(aw_dev, data_ptr, data_len);
		} break;
		case AW_IOCTL_SET_PTR_PARAM_NUM: {
			aw_dev_err(aw_dev->dev, "%s :not read  cmd %d\n",
				__func__, cmd);
			ret = -EINVAL;
			//>>>>>
		} break;
		case AW_IOCTL_SET_CALI_RE : {
			aw_cali_store_cali_re(aw_dev, *((int32_t *)data_ptr));
		} break;
		case AW_IOCTL_SET_DSP_HMUTE : {
			ret = aw_dsp_write_hmute(aw_dev, data_ptr, data_len);
		} break;
		case AW_IOCTL_SET_CALI_CFG_FLAG : {
			aw_cali_set_cali_status(aw_dev, *((int32_t *)data_ptr));
			ret = aw_dsp_write_cali_en(aw_dev, data_ptr, data_len);
		} break;
		default:{
			aw_dev_err(aw_dev->dev, "%s :unsupported  cmd %d\n",
				__func__, cmd);
			ret = -EINVAL;
		} break;
	}

exit:
	kfree(data_ptr);
	return ret;
}

static int aw_misc_cali_ops_read(struct aw_device *aw_dev,
			unsigned int cmd, unsigned long arg)
{
	int16_t data_len = _IOC_SIZE(cmd);
	char *data_ptr = NULL;
	int ret = 0;

	data_ptr = kmalloc(data_len, GFP_KERNEL);
	if (!data_ptr) {
		aw_dev_err(aw_dev->dev, "%s : malloc failed !\n", __func__);
		return -ENOMEM;
	}

	switch (cmd) {
		case AW_IOCTL_GET_CALI_CFG : {
			ret = aw_dsp_read_cali_cfg(aw_dev, data_ptr, data_len);
		} break;
		case AW_IOCTL_GET_CALI_DATA : {
			ret = aw_dsp_read_cali_data(aw_dev, data_ptr, data_len);
		} break;
		case AW_IOCTL_GET_F0 : {
			ret = aw_dsp_read_f0(aw_dev, data_ptr, data_len);
		} break;
		case AW_IOCTL_GET_CALI_RE : {
			ret = aw_dsp_read_cali_re(aw_dev, data_ptr, data_len);
		} break;
		case AW_IOCTL_GET_VMAX : {
			ret = aw_dsp_read_vmax(aw_dev, data_ptr, data_len);
		} break;
		case AW_IOCTL_GET_F0_Q : {
			ret = aw_dsp_read_f0_q(aw_dev, data_ptr, data_len);
		} break;
		default:{
			aw_dev_err(aw_dev->dev, "%s :unsupported  cmd %d\n",
				__func__, cmd);
			ret = -EINVAL;
		} break;
	}

	if (copy_to_user((void __user *)arg,
		data_ptr, data_len)) {
		ret = -EFAULT;
	}

	kfree(data_ptr);
	return ret;

}

static int aw_misc_ops_read_dsp(struct aw_device *aw_dev, aw_ioctl_msg_t *msg)
{
	char __user* user_data = (char __user*)msg->data_buf;
	uint32_t dsp_msg_id = (uint32_t)msg->opcode_id;
	int data_len = msg->data_len;
	int ret;
	char *data_ptr;

	data_ptr = kmalloc(data_len, GFP_KERNEL);
	if (!data_ptr) {
		aw_dev_err(aw_dev->dev, "%s : malloc failed !\n", __func__);
		return -ENOMEM;
	}

	ret = aw_dsp_read_msg(aw_dev, dsp_msg_id, data_ptr, data_len);
	if (ret) {
		aw_dev_err(aw_dev->dev, "%s : write failed\n", __func__);
		goto exit;
	}

	if (copy_to_user((void __user *)user_data,
		data_ptr, data_len)) {
		ret = -EFAULT;
	}
exit:
	kfree(data_ptr);
	return ret;
}

static int aw_misc_ops_write_dsp(struct aw_device *aw_dev, aw_ioctl_msg_t *msg)
{
	char __user* user_data = (char __user*)msg->data_buf;
	uint32_t dsp_msg_id = (uint32_t)msg->opcode_id;
	int data_len = msg->data_len;
	int ret;
	char *data_ptr;

	data_ptr = kmalloc(data_len, GFP_KERNEL);
	if (!data_ptr) {
		aw_dev_err(aw_dev->dev, "%s : malloc failed !\n", __func__);
		return -ENOMEM;
	}

	if (copy_from_user(data_ptr, (void __user *)user_data, data_len)) {
		aw_dev_err(aw_dev->dev, "%s : copy data failed\n", __func__);
		ret = -EFAULT;
		goto exit;
	}

	ret = aw_dsp_write_msg(aw_dev, dsp_msg_id, data_ptr, data_len);
	if (ret) {
		aw_dev_err(aw_dev->dev, "%s : write failed\n", __func__);
	}
exit:
	kfree(data_ptr);
	return ret;
}

static int aw_misc_ops_msg(struct aw_device *aw_dev, unsigned long arg)
{
	aw_ioctl_msg_t ioctl_msg;

	if (copy_from_user(&ioctl_msg, (void __user *)arg, sizeof(aw_ioctl_msg_t))) {
		return -EFAULT;
	}

	if(ioctl_msg.version != AW_IOCTL_MSG_VERSION) {
		aw_dev_err(aw_dev->dev, "unsupported msg version %d", ioctl_msg.version);
		return -EINVAL;
	}

	if (ioctl_msg.type == AW_IOCTL_MSG_RD_DSP) {
		return aw_misc_ops_read_dsp(aw_dev, &ioctl_msg);
	} else if (ioctl_msg.type == AW_IOCTL_MSG_WR_DSP) {
		return aw_misc_ops_write_dsp(aw_dev, &ioctl_msg);
	} else if (ioctl_msg.type == AW_IOCTL_MSG_IOCTL) {
		return aw_misc_cali_ops(aw_dev, ioctl_msg.opcode_id, (unsigned long)ioctl_msg.data_buf);
	} else {
		aw_dev_err(aw_dev->dev, "unsupported msg type %d", ioctl_msg.type);
		return -EINVAL;
	}
}

static int aw_misc_cali_ops(struct aw_device *aw_dev,
			unsigned int cmd, unsigned long arg)
{
	int ret = 0;

	switch (cmd) {
	case AW_IOCTL_ENABLE_CALI:
	case AW_IOCTL_SET_CALI_CFG:
	case AW_IOCTL_SET_NOISE:
	case AW_IOCTL_SET_VMAX:
	case AW_IOCTL_SET_PARAM:
	case AW_IOCTL_SET_PTR_PARAM_NUM:
	case AW_IOCTL_SET_CALI_RE:
	case AW_IOCTL_SET_DSP_HMUTE:
	case AW_IOCTL_SET_CALI_CFG_FLAG:
		ret = aw_misc_cali_ops_write(aw_dev, cmd, arg);
		break;
	case AW_IOCTL_GET_CALI_CFG:
	case AW_IOCTL_GET_CALI_DATA:
	case AW_IOCTL_GET_F0:
	case AW_IOCTL_GET_CALI_RE:
	case AW_IOCTL_GET_VMAX:
	case AW_IOCTL_GET_F0_Q:
		ret = aw_misc_cali_ops_read(aw_dev, cmd, arg);
		break;
	case AW_IOCTL_MSG : {
		ret = aw_misc_ops_msg(aw_dev, arg);
	} break;
	default:
		aw_dev_err(aw_dev->dev, "%s :unsupported  cmd %d\n",
			__func__, cmd);
		ret = -EINVAL;
		break;
	}

	return ret;
}

static long aw_misc_file_unlocked_ioctl(struct file *file,
			unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	struct aw_device *aw_dev = NULL;

	if (((_IOC_TYPE(cmd)) != (AW_IOCTL_MAGIC))) {
		aw_dev_err(aw_dev->dev, "%s: cmd magic err\n", __func__);
		return -EINVAL;
	}
	aw_dev = (struct aw_device *)file->private_data;
	ret = aw_misc_cali_ops(aw_dev, cmd, arg);
	if (ret)
		return -EINVAL;

	return 0;
}

#ifdef CONFIG_COMPAT
static long aw_misc_file_compat_ioctl(struct file *file,
	unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	struct aw_device *aw_dev = NULL;

	if (((_IOC_TYPE(cmd)) != (AW_IOCTL_MAGIC))) {
		aw_dev_err(aw_dev->dev, "%s: cmd magic err\n", __func__);
		return -EINVAL;
	}
	aw_dev = (struct aw_device *)file->private_data;
	ret = aw_misc_cali_ops(aw_dev, cmd, arg);
	if (ret)
		return -EINVAL;

	return 0;
}
#endif


static const struct file_operations aw_misc_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = aw_misc_file_unlocked_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = aw_misc_file_compat_ioctl,
#endif
	.open = aw_misc_file_open,
	.release = aw_misc_file_release,
};

static int aw_cali_misc_init(struct aw_device *aw_dev)
{
	struct miscdevice *misc_cali = NULL;
	char *name = NULL;
	int ret;

	misc_cali =  &aw_dev->cali_desc.misc;

	misc_cali->minor = MISC_DYNAMIC_MINOR;
	misc_cali->fops  = &aw_misc_fops;
	misc_cali->name = NULL;

	name = kzalloc(AW_NAME_BUF_MAX, GFP_KERNEL);
	if (name == NULL) {
		aw_dev_err(aw_dev->dev, "%s: name malloc failed \n", __func__);
		return -ENOMEM;
	}

	snprintf(name, AW_NAME_BUF_MAX, "aw_dev_%d", aw_dev->index);
	misc_cali->name = name;

	ret = misc_register(misc_cali);
	if (ret) {
		aw_dev_err(aw_dev->dev, "%s: misc register fail: %d\n",
			__func__, ret);
		kfree(name);
		misc_cali->name = NULL;
		return -EINVAL;
	}
	aw_dev_dbg(aw_dev->dev, "%s: misc register success\n", __func__);

	return 0;
}

static void aw_cali_misc_deinit(struct aw_device *aw_dev)
{

	if (aw_dev->cali_desc.misc.name) {
		misc_deregister(&aw_dev->cali_desc.misc);
		kfree(aw_dev->cali_desc.misc.name);
		aw_dev->cali_desc.misc.name = NULL;
	}

	aw_dev_dbg(aw_dev->dev, "%s: misc unregister done\n", __func__);
}
/*****************************misc device end***************************************************/

/*****************************cali common start***************************************************/

static int aw_cali_cali_mode(struct aw_device *aw_dev, bool enable)
{
	int ret;
	int32_t en_cali = enable;

	aw_dev_dbg(aw_dev->dev, "%s: enter flag:%d\n",
			 __func__, enable);

	ret = aw_dsp_write_cali_en(aw_dev, (char *)&en_cali, sizeof(int32_t));
	if (ret < 0) {
		aw_dev_err(aw_dev->dev, "%s:start cali_mode failed!\n",
				__func__);
		return ret;
	}

	return 0;
}

static int aw_cali_get_re(struct aw_device *aw_dev ,int32_t *cali_re, bool hmute)
{
	int ret;
	struct cali_data cali_data;
	int32_t hmute_value = hmute;

	aw_cali_set_cali_status(aw_dev, true);

	if (hmute) {
		ret = aw_dsp_write_hmute(aw_dev, (char *)&hmute_value, sizeof(int32_t));
		if (ret < 0) {
			goto mute_failed;
		}
	}

	ret = aw_cali_cali_mode(aw_dev, true);
	if (ret < 0) {
		goto cali_mode_failed;
	}

	/*wait time*/
	msleep(aw_dev->cali_desc.time);
	/*get cali data*/
	ret = aw_dsp_read_cali_data(aw_dev, (char *)&cali_data, sizeof(struct cali_data));
	if (ret < 0) {
		goto cali_data_failed;
	}

	aw_dev_info(aw_dev->dev, "%s:cali_re : 0x%x\n",
		__func__, cali_data.data[0]);
	*cali_re = cali_data.data[0];

	/*repair cali cfg to normal status*/
	aw_cali_cali_mode(aw_dev, false);
	if (hmute) {
		hmute_value = 0;
		aw_dsp_write_hmute(aw_dev, (char *)&hmute_value, sizeof(int32_t));
	}
	aw_dev->cali_desc.status = false;
	return 0;

cali_data_failed:
	aw_cali_cali_mode(aw_dev, false);
cali_mode_failed:
	if (hmute) {
		hmute_value = false;
		aw_dsp_write_hmute(aw_dev, (char *)&hmute_value, sizeof(int32_t));
	}
mute_failed:
	aw_dev->cali_desc.status = false;
	return ret;
}

static int aw_cali_get_f0(struct aw_device *aw_dev ,int32_t *cali_f0, bool noise)
{
	int ret;
	int32_t read_f0;
	int32_t noise_value = noise;

	aw_cali_set_cali_status(aw_dev, true);

	if (noise) {
		ret = aw_dsp_write_noise(aw_dev, (char *)&noise_value, sizeof(int32_t));
		if (ret < 0) {
			goto noise_failed;
		}
	}

	ret = aw_cali_cali_mode(aw_dev, true);
	if (ret < 0) {
		goto cali_mode_failed;
	}

	/*wait time*/
	msleep(5 * 1000);

	/*get cali data*/
	ret = aw_dsp_read_f0(aw_dev, (char *)&read_f0, sizeof(int32_t));
	if (ret < 0) {
		goto read_f0_failed;
	}

	aw_dev_info(aw_dev->dev, "%s:f0 : %d\n", __func__, read_f0);
	*cali_f0 = read_f0;

	/*repair cali cfg to normal status*/
	aw_cali_cali_mode(aw_dev, false);
	if (noise) {
		noise_value = false;
		aw_dsp_write_noise(aw_dev, (char *)&noise_value, sizeof(int32_t));
	}
	aw_dev->cali_desc.status = false;
	return 0;

read_f0_failed:
	aw_cali_cali_mode(aw_dev, false);
cali_mode_failed:
	if (noise) {
		noise_value = false;
		aw_dsp_write_noise(aw_dev, (char *)&noise_value, sizeof(int32_t));
	}
noise_failed:
	aw_dev->cali_desc.status = false;
	return ret;
}

/*****************************cali common end***************************************************/



/*****************************attr   start***************************************************/
static ssize_t aw_cali_time_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;
	struct aw882xx *aw882xx = dev_get_drvdata(dev);
	struct aw_device *aw_dev = aw882xx->aw_pa;
	uint32_t time;

	ret = kstrtoint(buf, 0, &time);
	if (ret < 0) {
		aw_dev_err(aw_dev->dev, "%s, read buf %s failed\n",
			__func__, buf);
		return ret;
	}

	if (time < 400) {
		aw_dev_err(aw_dev->dev, "%s:time:%d is too short, no set\n",
			__func__, time);
		return -EINVAL;
	}

	aw_dev->cali_desc.time = time;
	aw_dev_dbg(aw_dev->dev, "%s:time:%d\n",
			__func__, time);

	return count;
}


static ssize_t aw_cali_time_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct aw882xx *aw882xx = dev_get_drvdata(dev);
	ssize_t len = 0;
	struct aw_device *aw_dev = aw882xx->aw_pa;

	len += snprintf(buf+len, PAGE_SIZE-len,
		"time: %d \n", aw_dev->cali_desc.time);

	return len;
}

static ssize_t aw_cali_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int32_t local_re = 0;
	int32_t local_f0 = 0;
	int ret;
	struct aw882xx *aw882xx = dev_get_drvdata(dev);
	struct aw_device *aw_dev = aw882xx->aw_pa;

	if (!strncmp("start_cali", buf, strlen("start_cali"))) {
		ret = aw_cali_get_re(aw_dev, &local_re, true);
		if (ret) {
			aw_dev_err(aw_dev->dev, "%s: get cali re failed! \n", __func__);
			aw_dev->cali_desc.t_re = 0;
			return -EPERM;
		}
		local_re = AW_DSP_RE_TO_SHOW_RE(local_re);
		aw_dev->cali_desc.t_re = local_re;

		ret = aw_cali_get_f0(aw_dev, &local_f0, true);
		if (ret) {
			aw_dev_err(aw_dev->dev, "%s: get cali re failed! \n", __func__);
			aw_dev->cali_desc.t_f0 = 0;
			return -EPERM;;
		}
		aw_dev->cali_desc.t_f0 = local_f0;

		return count;
	}else if (!strncmp("cali_re", buf, strlen("cali_re"))) {
		ret = aw_cali_get_re(aw_dev, &local_re, true);
		if (ret) {
			aw_dev_err(aw_dev->dev, "%s: get cali re failed! \n", __func__);
			aw_dev->cali_desc.t_re = 0;
			return -EPERM;
		}
		local_re = AW_DSP_RE_TO_SHOW_RE(local_re);
		aw_dev->cali_desc.t_re = local_re;
		return count;
	} else if (!strncmp("cali_f0", buf, strlen("cali_f0"))) {
		ret = aw_cali_get_f0(aw_dev, &local_f0, true);
		if (ret) {
			aw_dev_err(aw_dev->dev, "%s: get cali re failed! \n", __func__);
			aw_dev->cali_desc.t_f0 = 0;
			return -EPERM;
		}
		aw_dev->cali_desc.t_f0 = local_f0;
		return count;
	} else {
		aw_dev_err(aw_dev->dev, "%s: supported cmd [%s]! \n", __func__, buf);
	}

	return -EPERM;
}

static ssize_t aw_re_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct aw882xx *aw882xx = dev_get_drvdata(dev);
	struct aw_device *aw_dev = aw882xx->aw_pa;
	ssize_t len = 0;

	len += snprintf(buf+len, PAGE_SIZE-len, "%d\n", aw_dev->cali_desc.t_re);

	return len;
}

static ssize_t aw_f0_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct aw882xx *aw882xx = dev_get_drvdata(dev);
	struct aw_device *aw_dev = aw882xx->aw_pa;
	ssize_t len = 0;

	len += snprintf(buf+len, PAGE_SIZE-len, "%d\n", aw_dev->cali_desc.t_f0);

	return len;
}

static ssize_t aw_dsp_re_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct aw882xx *aw882xx = dev_get_drvdata(dev);
	struct aw_device *aw_dev = aw882xx->aw_pa;
	int ret = -1;
	ssize_t len = 0;
	uint32_t re = 0;

	ret = aw_dsp_read_cali_re(aw_dev, (char *)&re, sizeof(uint32_t));
	if (ret < 0)
		aw_dev_err(aw_dev->dev, "%s : get dsp re failed\n",
			__func__);

	re = AW_DSP_RE_TO_SHOW_RE(re);
	len += snprintf(buf+len, PAGE_SIZE-len, "dsp_re:%d\n", re);

	return len;
}

static ssize_t aw_dsp_re_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;
	struct aw882xx *aw882xx = dev_get_drvdata(dev);
	struct aw_device *aw_dev = aw882xx->aw_pa;
	int32_t data;
	int32_t cali_re;

	ret = kstrtoint(buf, 0, &data);
	if (ret < 0) {
		aw_dev_err(aw_dev->dev, "%s, read buf %s failed\n", __func__, buf);
		return ret;
	}

	cali_re = AW_SHOW_RE_TO_DSP_RE(data);

	ret = aw_cali_store_cali_re(aw_dev, cali_re);
	if (ret < 0) {
		aw_dev_err(aw_dev->dev, "%s: store cali re error\n", __func__);
		return -EPERM;
	}

	ret = aw_dsp_write_cali_re(aw_dev, (char *)&cali_re, sizeof(uint32_t));
	if (ret) {
		aw_dev_err(aw_dev->dev, "%s: write cali_re to dsp failed\n", __func__);
		return -EBUSY;
	}

	aw_dev_dbg(aw_dev->dev, "%s: re:0x%x",
			__func__, aw_dev->cali_desc.cali_re);

	return count;
}


/*set cali time*/
static DEVICE_ATTR(cali_time, S_IWUSR | S_IRUGO,
	aw_cali_time_show, aw_cali_time_store);
/*start cali*/
static DEVICE_ATTR(cali, S_IWUSR,
	NULL, aw_cali_store);
/*show cali_re*/
static DEVICE_ATTR(re_show, S_IRUGO,
	aw_re_show, NULL);
/*show cali_f0*/
static DEVICE_ATTR(f0_show, S_IRUGO,
	aw_f0_show, NULL);
static DEVICE_ATTR(dsp_re, S_IWUSR | S_IRUGO,
	aw_dsp_re_show, aw_dsp_re_store);


static struct attribute *aw_cali_attr[] = {
	&dev_attr_cali_time.attr,
	&dev_attr_cali.attr,
	&dev_attr_re_show.attr,
	&dev_attr_f0_show.attr,
	&dev_attr_dsp_re.attr,
	NULL
};

static struct attribute_group aw_cali_attr_group = {
	.attrs = aw_cali_attr
};

static void aw_cali_attr_init(struct aw_device *aw_dev)
{
	int ret;

	ret = sysfs_create_group(&aw_dev->dev->kobj, &aw_cali_attr_group);
	if (ret < 0) {
		aw_dev_info(aw_dev->dev, "%s error creating sysfs cali attr files\n",
			__func__);
	}
}

static void aw_cali_attr_deinit(struct aw_device *aw_dev)
{
	sysfs_remove_group(&aw_dev->dev->kobj, &aw_cali_attr_group);
	aw_dev_info(aw_dev->dev, "%s attr files deinit\n", __func__);
}

/*****************************attr   end***************************************************/

/*****************************debufs   start***************************************************/
/*unit mOhms*/
static int R0_MAX = AW_CALI_RE_MAX;
static int R0_MIN = AW_CALI_RE_MIN;

int  aw_cali_range_open(struct inode *inode, struct file *file)
{
	struct aw_device *aw_dev = (struct aw_device *)(void *)inode->i_private;
	file->private_data = (void *)inode->i_private;
	aw_dev_info(aw_dev->dev, "%s: open success", __func__);
	return 0;
}

ssize_t aw_cali_range_read(struct file *file,
	char __user *buf, size_t len, loff_t *ppos)
{
	int ret;
	char local_buf[50];
	struct aw_device *aw_dev = (struct aw_device *)file->private_data;

	if (*ppos)
		return 0;
	memset(local_buf, 0, sizeof(local_buf));
	if (len < sizeof(local_buf)) {
		aw_dev_err(aw_dev->dev, "%s: buf len not enough\n", __func__);
		return -ENOSPC;
	}

	ret = snprintf(local_buf, PAGE_SIZE,
			" Min:%d mOhms, Max:%d mOhms\n", R0_MIN, R0_MAX);

	ret = simple_read_from_buffer(buf, len, ppos, local_buf, ret);
	if (ret < 0) {
		aw_dev_err(aw_dev->dev, "%s: copy failed!\n", __func__);
		return -ENOMEM;
	}
	return ret;
}

ssize_t aw_cali_range_write(struct file *file,
	const char __user *buf, size_t len, loff_t *ppos)
{
	struct aw_device *aw_dev = (struct aw_device *)file->private_data;
	uint32_t time;
	int ret;

	if (*ppos)
		return 0;

	ret = kstrtouint_from_user(buf, len, 0, &time);
	if (ret)
		return len;

	if (time < 400) {
		aw_dev_err(aw_dev->dev, "%s:time:%d is too short, no set\n",
			__func__, time);
		return -EINVAL;
	}

	aw_dev->cali_desc.time = time;

	return len;
}

int  aw_f0_open(struct inode *inode, struct file *file)
{
	struct aw_device *aw_dev = (struct aw_device *)(void *)inode->i_private;
	file->private_data = (void *)inode->i_private;
	aw_dev_dbg(aw_dev->dev, "%s: open success\n", __func__);
	return 0;
}

ssize_t aw_f0_read(struct file *file,
	char __user *buf, size_t len, loff_t *ppos)
{
	int ret;
	char ret_value[20];
	int local_len = 0;
	int32_t ret_f0 = 0;
	struct aw_device *aw_dev = (struct aw_device *)file->private_data;

	if (*ppos)
		return 0;

	memset(ret_value, 0, sizeof(ret_value));
	if (len < sizeof(ret_value)) {
		aw_dev_err(aw_dev->dev, "%s:buf len no enough\n", __func__);
		aw_dev->cali_desc.t_f0 = 0;
		return -ENOMEM;
	}

	ret = aw_cali_get_f0(aw_dev,&ret_f0, true);
	if (ret < 0) {
		aw_dev_err(aw_dev->dev, "%s:cali failed\n", __func__);
		aw_dev->cali_desc.t_f0 = 0;
		return ret;
	}

	aw_dev->cali_desc.t_f0 = ret_f0;

	ret = snprintf(ret_value + local_len,
		PAGE_SIZE - local_len, "%d\n", ret_f0);

	ret = simple_read_from_buffer(buf, len, ppos, ret_value, ret);
	if (ret < 0) {
		aw_dev_err(aw_dev->dev, "%s:copy failed!\n", __func__);
	}
	return ret;
}

int  aw_cali_status_open(struct inode *inode, struct file *file)
{
	struct aw_device *aw_dev = (struct aw_device *)(void *)inode->i_private;
	file->private_data = (void *)inode->i_private;
	aw_dev_dbg(aw_dev->dev, "%s: open success\n", __func__);
	return 0;
}

ssize_t aw_cali_status_read(struct file *file,
	char __user *buf, size_t len, loff_t *ppos)
{
	int ret;
	char status_value[20];
	int local_len = 0;
	struct cali_data cali_data;
	int32_t real_r0;
	struct aw_device *aw_dev = (struct aw_device *)file->private_data;

	if (*ppos)
		return 0;

	if (len < sizeof(status_value)) {
		aw_dev_err(aw_dev->dev, "%s:buf len no enough\n", __func__);
		return -ENOSPC;
	}

	/*get cali data*/
	ret = aw_dsp_read_cali_data(aw_dev, (char *)&cali_data, sizeof(struct cali_data));
	if (ret) {
		aw_dev_err(aw_dev->dev, "%s:read speaker status failed!\n",
			__func__);
		return -EBUSY;
	}

	/*R0 factor form 4096 to 1000*/
	real_r0 = AW_DSP_RE_TO_SHOW_RE(cali_data.data[0]);

	ret = snprintf(status_value + local_len, PAGE_SIZE - local_len,
				"%d : %d\n", real_r0, cali_data.data[1]);

	ret = simple_read_from_buffer(buf, len, ppos, status_value, ret);
	if (ret < 0) {
		aw_dev_err(aw_dev->dev, "%s:copy failed!", __func__);
	}
	return ret;
}

int  aw_cali_open(struct inode *inode, struct file *file)
{
	struct aw_device *aw_dev = (struct aw_device *)(void *)inode->i_private;
	file->private_data = (void *)inode->i_private;
	aw_dev_dbg(aw_dev->dev, "%s: open success\n", __func__);
	return 0;
}

ssize_t aw_cali_read(struct file *file,
	char __user *buf, size_t len, loff_t *ppos)
{
	int ret;
	char ret_value[20];
	int local_len = 0;
	int32_t re_cali = 0;
	struct aw_device *aw_dev = (struct aw_device *)file->private_data;

	if (*ppos)
		return 0;
	memset(ret_value, 0, sizeof(ret_value));
	if (len < sizeof(ret_value)) {
		aw_dev_err(aw_dev->dev, "%s:buf len no enough\n", __func__);
		aw_dev->cali_desc.t_re = 0;
		return -ENOMEM;
	}

	ret = aw_cali_get_re(aw_dev, &re_cali, true);
	if (ret < 0) {
		aw_dev_err(aw_dev->dev, "%s:cali failed\n", __func__);
		aw_dev->cali_desc.t_re = 0;
		return ret;
	}

	/*factor form 12bit(4096) to 1000*/
	re_cali = AW_DSP_RE_TO_SHOW_RE(re_cali);

	aw_dev->cali_desc.t_re = re_cali;

	ret = snprintf(ret_value + local_len,
		PAGE_SIZE - local_len, "%d\n", re_cali);

	return simple_read_from_buffer(buf, len, ppos, ret_value, ret);
}


static const struct file_operations aw_cali_fops = {
	.open = aw_cali_open,
	.read = aw_cali_read,
};

static const struct file_operations aw_cali_range_fops = {
	.open = aw_cali_range_open,
	.read = aw_cali_range_read,
	.write = aw_cali_range_write,
};

static const struct file_operations aw_f0_fops = {
	.open = aw_f0_open,
	.read = aw_f0_read,
};

static const struct file_operations aw_cali_status_fops = {
	.open = aw_cali_status_open,
	.read = aw_cali_status_read,
};

static void aw_cali_debugfs_init(struct aw_device *aw_dev)
{
	char *debugfs_dir = NULL;
	struct aw_dbg_cali *dbg_fs = &aw_dev->cali_desc.dbg;


	debugfs_dir = devm_kzalloc(aw_dev->dev, AW_NAME_BUF_MAX, GFP_KERNEL);
	if (!debugfs_dir) {
		aw_dev_err(aw_dev->dev, "%s:debugfs kzalloc failed\n",
			__func__);
		return;
	}

	snprintf(debugfs_dir, AW_NAME_BUF_MAX, "aw_dev_%d_dbg", aw_dev->index);

	dbg_fs->dbg_dir = debugfs_create_dir(debugfs_dir, NULL);
	if (dbg_fs->dbg_dir == NULL) {
		aw_dev_err(aw_dev->dev, "create cali debugfs failed !\n");
		return;
	}

	dbg_fs->dbg_range = debugfs_create_file("range", S_IFREG|S_IRUGO,
			dbg_fs->dbg_dir, aw_dev, &aw_cali_range_fops);
	if (dbg_fs->dbg_range == NULL) {
		aw_dev_err(aw_dev->dev, "create cali debugfs range failed !\n");
		return;
	}

	dbg_fs->dbg_cali = debugfs_create_file("cali", S_IFREG|S_IRUGO|S_IWUGO,
			dbg_fs->dbg_dir, aw_dev, &aw_cali_fops);
	if (dbg_fs->dbg_cali == NULL) {
		aw_dev_err(aw_dev->dev, "create cali debugfs cali failed !\n");
		return;
	}

	dbg_fs->dbg_f0 = debugfs_create_file("f0", S_IFREG|S_IRUGO,
			dbg_fs->dbg_dir, aw_dev, &aw_f0_fops);
	if (dbg_fs->dbg_f0 == NULL) {
		aw_dev_err(aw_dev->dev, "create cali debugfs cali failed !\n");
		return;
	}

	dbg_fs->dbg_status = debugfs_create_file("status", S_IFREG|S_IRUGO,
			dbg_fs->dbg_dir, aw_dev, &aw_cali_status_fops);
	if (dbg_fs->dbg_status == NULL) {
		aw_dev_err(aw_dev->dev, "create cali debugfs status failed !\n");
		return;
	}
}

void aw_cali_debugfs_deinit(struct aw_device *aw_dev)
{
	struct aw_dbg_cali *dbg_fs = &aw_dev->cali_desc.dbg;

	debugfs_remove(dbg_fs->dbg_range);
	debugfs_remove(dbg_fs->dbg_cali);
	debugfs_remove(dbg_fs->dbg_f0);
	debugfs_remove(dbg_fs->dbg_status);
	debugfs_remove(dbg_fs->dbg_dir);
}


/*****************************debufs   end***************************************************/


static void aw_cali_parse_dt(struct aw_device *aw_dev)
{
	struct device_node *np = aw_dev->dev->of_node;
	int ret = -1;
	const char *cali_mode_str;
	struct aw_cali_desc *desc = &aw_dev->cali_desc;

	ret = of_property_read_string(np, "aw-cali-mode", &cali_mode_str);
	if (ret < 0) {
		dev_info(aw_dev->dev, "%s: aw-cali-mode get failed ,user default attr way\n",
				__func__);
		desc->mode = AW_CALI_MODE_MISC;
		return;
	}

	if (!strcmp(cali_mode_str, "aw_dbgfs"))
		desc->mode = AW_CALI_MODE_DBGFS;
	else if (!strcmp(cali_mode_str, "aw_misc"))
		desc->mode = AW_CALI_MODE_MISC;
	else if (!strcmp(cali_mode_str, "aw_attr"))
		desc->mode = AW_CALI_MODE_ATTR;
	else 
		desc->mode = AW_CALI_MODE_MISC;     //default misc

	aw_dev_info(aw_dev->dev, "%s:cali mode str:%s num:%d\n",
			__func__, cali_mode_str, desc->mode);
}

void aw_cali_init(struct aw_cali_desc *cali_desc)
{

	struct aw_device *aw_dev =
		container_of(cali_desc, struct aw_device, cali_desc);

	cali_desc->cali_f0 = 0;
	cali_desc->cali_re = 0;
	cali_desc->t_f0 = 0;
	cali_desc->t_re = 0;
	cali_desc->status = 0;
	cali_desc->time = AW_CALI_RE_DEFAULT_TIMER;

	aw_cali_parse_dt(aw_dev);

	if (cali_desc->mode == AW_CALI_MODE_DBGFS) {
		aw_cali_debugfs_init(aw_dev);
	} else if (cali_desc->mode == AW_CALI_MODE_ATTR) {
		aw_cali_attr_init(aw_dev);
	}

	//misc must open
	aw_cali_misc_init(aw_dev);
}


void aw_cali_deinit(struct aw_cali_desc *cali_desc)
{
	struct aw_device *aw_dev =
		container_of(cali_desc, struct aw_device, cali_desc);
	if (cali_desc->mode == AW_CALI_MODE_DBGFS) {
		aw_cali_debugfs_deinit(aw_dev);
	} else if (cali_desc->mode == AW_CALI_MODE_ATTR) {
		aw_cali_attr_deinit(aw_dev);
	}

	aw_cali_misc_deinit(aw_dev);
}


