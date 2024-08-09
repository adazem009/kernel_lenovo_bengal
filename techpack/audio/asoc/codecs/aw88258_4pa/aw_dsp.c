
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

#include "aw_device.h"
#include "aw_dsp.h"
#include "aw_log.h"

static DEFINE_MUTEX(g_aw_dsp_msg_lock);
static DEFINE_MUTEX(g_aw_dsp_lock);


#define AW_COPP_MODULE_ID (0X10013D02)			/*SKT module id*/
#define AW_COPP_MODULE_PARAMS_ID_EN (0X10013D14)	/*SKT enable param id*/


#define AW_MSG_ID_ENABLE_CALI		(0x00000001)
#define AW_MSG_ID_ENABLE_HMUTE		(0x00000002)
#define AW_MSG_ID_F0_Q			(0x00000003)

/*dsp params id*/
#define AW_MSG_ID_RX_SET_ENABLE		(0x10013D11)
#define AW_MSG_ID_PARAMS		(0x10013D12)
#define AW_MSG_ID_TX_SET_ENABLE		(0x10013D13)
#define AW_MSG_ID_VMAX_L		(0X10013D17)
#define AW_MSG_ID_VMAX_R		(0X10013D18)
#define AW_MSG_ID_CALI_CFG_L		(0X10013D19)
#define AW_MSG_ID_CALI_CFG_R		(0x10013d1A)
#define AW_MSG_ID_RE_L			(0x10013d1B)
#define AW_MSG_ID_RE_R			(0X10013D1C)
#define AW_MSG_ID_NOISE_L		(0X10013D1D)
#define AW_MSG_ID_NOISE_R		(0X10013D1E)
#define AW_MSG_ID_F0_L			(0X10013D1F)
#define AW_MSG_ID_F0_R			(0X10013D20)
#define AW_MSG_ID_REAL_DATA_L		(0X10013D21)
#define AW_MSG_ID_REAL_DATA_R		(0X10013D22)

#define AFE_PARAM_ID_AWDSP_RX_MSG_0	(0X10013D2A)
#define AFE_PARAM_ID_AWDSP_RX_MSG_1	(0X10013D2B)
#define AW_MSG_ID_PARAMS_1		(0x10013D2D)

#define AW_MSG_ID_SPIN		(0x10013D2E)

enum {
	MSG_PARAM_ID_0 = 0,
	MSG_PARAM_ID_1,
	MSG_PARAM_ID_MAX,
};

static uint32_t afe_param_msg_id[MSG_PARAM_ID_MAX] = {
	AFE_PARAM_ID_AWDSP_RX_MSG_0,
	AFE_PARAM_ID_AWDSP_RX_MSG_1,
};

#define AWINIC_ADSP_ENABLE
#ifdef AWINIC_ADSP_ENABLE
extern int afe_get_topology(int port_id);
extern int aw_send_afe_cal_apr(uint32_t param_id,
	void *buf, int cmd_size, bool write);
extern int aw_send_afe_rx_module_enable(void *buf, int size);
extern int aw_send_afe_tx_module_enable(void *buf, int size);

#if  0
//#include <dsp/q6adm-v2.h>
//#include <dsp/apr_audio-v2.h>

#include <sound/q6adm-v2.h>
#include <sound/apr_audio-v2.h>
static int aw_adm_param_enable(int port_id, int module_id,  int param_id, int enable)
{
        int copp_idx = 0;
        uint32_t enable_param;
        struct param_hdr_v3 param_hdr;
        int rc = 0;

        pr_debug("%s port_id %d, module_id 0x%x, enable %d\n",
                __func__, port_id, module_id, enable);

        copp_idx = adm_get_default_copp_idx(port_id);
        if (copp_idx < 0 || copp_idx >= MAX_COPPS_PER_PORT) {
                pr_err("%s: Invalid copp_num: %d\n", __func__, copp_idx);
                return -EINVAL;
        }

        if (enable < 0 || enable > 1) {
                pr_err("%s: Invalid value for enable %d\n", __func__, enable);
                return -EINVAL;
        }

        pr_debug("%s port_id %d, module_id 0x%x, copp_idx 0x%x, enable %d\n",
                __func__, port_id, module_id, copp_idx, enable);

        memset(&param_hdr, 0, sizeof(param_hdr));
        param_hdr.module_id = module_id;
        param_hdr.instance_id = INSTANCE_ID_0;
        param_hdr.param_id = param_id;
        param_hdr.param_size = sizeof(enable_param);
        enable_param = enable;

        rc = adm_pack_and_set_one_pp_param(port_id, copp_idx, param_hdr,
                                                (uint8_t *) &enable_param);
        if (rc)
                pr_err("%s: Failed to set enable of module(%d) instance(%d) to %d, err %d\n",
                                __func__, module_id, INSTANCE_ID_0, enable, rc);
        return rc;
}
#else
static int aw_adm_param_enable(int port_id, int module_id,  int param_id, int enable)
{

	return 0;
}
#endif

#else
static int afe_get_topology(int port_id)
{

	return -1;
}

static int aw_send_afe_cal_apr(uint32_t param_id,
	void *buf, int cmd_size, bool write)
{
	return 0;
}
static int aw_send_afe_rx_module_enable(void *buf, int size)
{
	return 0;
}
static int aw_send_afe_tx_module_enable(void *buf, int size)
{
	return 0;
}
static int aw_adm_param_enable(int port_id, int module_id,  int param_id, int enable)
{

	return 0;
}

#endif

static int aw_get_msg_num(int dev_ch, int *msg_num)
{
	switch (dev_ch) {
		case AW_DEV_CH_PRI_L : {
			*msg_num = MSG_PARAM_ID_0;
		} break;
		case AW_DEV_CH_PRI_R : {
			*msg_num = MSG_PARAM_ID_0;
		} break;
		case AW_DEV_CH_SEC_L : {
			*msg_num = MSG_PARAM_ID_1;
		} break;
		case AW_DEV_CH_SEC_R : {
			*msg_num = MSG_PARAM_ID_1;
		} break;
		default : {
			pr_err("[Awinic] %s: can not find msg num, channel %d \n", __func__, dev_ch);
			return -1;
		}
	}

	pr_debug("[Awinic] %s: msg num[%d] \n", __func__, *msg_num);
	return 0;
}


static int aw_check_dsp_ready(void)
{
	int ret;
	ret = afe_get_topology(AFE_PORT_ID_AWDSP_RX);

	pr_debug("[Awinic] %s: topo_id 0x%x \n", __func__, ret);

	if (ret <= 0) {
		return false;
	} else {
		return true;
	}
}

static int aw_write_data_to_dsp(uint32_t param_id, void *data, int len)
{
	int ret;
	int try = 0;

	mutex_lock(&g_aw_dsp_lock);
	while (try < AW_DSP_TRY_TIME) {
		if (aw_check_dsp_ready()) {
			ret = aw_send_afe_cal_apr(param_id, data, len, true);
			mutex_unlock(&g_aw_dsp_lock);
			return ret;
		} else {
			try++;
			msleep(AW_DSP_SLEEP_TIME);
			pr_debug("[Awinic] %s: afe not ready try again \n", __func__);
		}
	}
	mutex_unlock(&g_aw_dsp_lock);

	return -EINVAL;
}

static int aw_read_data_from_dsp(uint32_t param_id, void *data, int len)
{
	int ret;
	int try = 0;
	mutex_lock(&g_aw_dsp_lock);
	while (try < AW_DSP_TRY_TIME) {
		if (aw_check_dsp_ready()) {
			ret = aw_send_afe_cal_apr(param_id, data, len, false);
			mutex_unlock(&g_aw_dsp_lock);
			return ret;
		} else {
			try++;
			msleep(AW_DSP_SLEEP_TIME);
			pr_debug("[Awinic] %s: afe not ready try again \n", __func__);
		}
	}
	mutex_unlock(&g_aw_dsp_lock);

	return -EINVAL;
}

static int aw_write_msg_to_dsp(int msg_num, uint32_t msg_id, char *data_ptr, unsigned int data_size)
{
	int32_t *dsp_msg;
	int ret = 0;
	int msg_len = (int)(sizeof(aw_dsp_msg_t) + data_size);

	mutex_lock(&g_aw_dsp_msg_lock);
	dsp_msg = kzalloc(msg_len, GFP_KERNEL);
	if (!dsp_msg) {
		pr_err("[Awinic] %s: msg_id:0x%x kzalloc dsp_msg error\n",
			__func__, msg_id);
		ret = -ENOMEM;
		goto w_mem_err;
	}
	dsp_msg[0] = AW_DSP_MSG_TYPE_DATA;
	dsp_msg[1] = msg_id;
	dsp_msg[2] = AW_DSP_MSG_HDR_VER;

	memcpy(dsp_msg + (sizeof(aw_dsp_msg_t) / sizeof(int32_t)),
		data_ptr, data_size);

	ret = aw_write_data_to_dsp(afe_param_msg_id[msg_num],
			(void *)dsp_msg, msg_len);
	if (ret < 0) {
		pr_err("[Awinic] %s:msg_id:0x%x, write data to dsp failed\n", __func__, msg_id);
		kfree(dsp_msg);
		goto w_mem_err;
	}

	pr_debug("[Awinic] %s:msg_id:0x%x, write data[%d] to dsp success\n", __func__, msg_id, msg_len);
	mutex_unlock(&g_aw_dsp_msg_lock);
	kfree(dsp_msg);
	return 0;
w_mem_err:
	mutex_unlock(&g_aw_dsp_msg_lock);
	return ret;
}

static int aw_read_msg_from_dsp(int msg_num, uint32_t msg_id, char *data_ptr, unsigned int data_size)
{
	aw_dsp_msg_t cmd_msg;
	int ret;

	mutex_lock(&g_aw_dsp_msg_lock);
	cmd_msg.type = AW_DSP_MSG_TYPE_CMD;
	cmd_msg.opcode_id = msg_id;
	cmd_msg.version = AW_DSP_MSG_HDR_VER;

	ret = aw_write_data_to_dsp(afe_param_msg_id[msg_num],
			&cmd_msg, sizeof(aw_dsp_msg_t));
	if (ret < 0) {
		pr_err("[Awinic] %s:msg_id:0x%x, write cmd to dsp failed\n",
			__func__, msg_id);
		goto dsp_msg_failed;
	}

	ret = aw_read_data_from_dsp(afe_param_msg_id[msg_num],
			data_ptr, (int)data_size);
	if (ret < 0) {
		pr_err("[Awinic] %s:msg_id:0x%x, read data from dsp failed\n",
			__func__, msg_id);
		goto dsp_msg_failed;
	}

	pr_debug("[Awinic] %s:msg_id:0x%x, read data[%d] from dsp success \n",
			__func__, msg_id, data_size);
	mutex_unlock(&g_aw_dsp_msg_lock);
	return 0;
dsp_msg_failed:
	mutex_unlock(&g_aw_dsp_msg_lock);
	return ret;
}

int aw_dsp_write_msg(struct aw_device *aw_dev,
	uint32_t msg_id, char *data_ptr, unsigned int data_size)
{
	int ret;
	int msg_num;

	ret = aw_get_msg_num(aw_dev->channel, &msg_num);
	if (ret < 0) {
		aw_dev_err(aw_dev->dev, "%s: get msg_num failed \n", __func__);
		return ret;
	}

	return aw_write_msg_to_dsp(msg_num, msg_id, data_ptr, data_size);
}

int aw_dsp_read_msg(struct aw_device *aw_dev,
	uint32_t msg_id, char *data_ptr, unsigned int data_size)
{
	int ret;
	int msg_num;

	ret = aw_get_msg_num(aw_dev->channel, &msg_num);
	if (ret < 0) {
		aw_dev_err(aw_dev->dev, "%s: get msg_num failed \n", __func__);
		return ret;
	}

	return aw_read_msg_from_dsp(msg_num, msg_id, data_ptr, data_size);
}

int aw_dsp_write_cali_cfg(struct aw_device *aw_dev, char *data, unsigned int data_len)
{
	uint32_t msg_id;
	int ret;
	int msg_num;

	ret = aw_get_msg_num(aw_dev->channel, &msg_num);
	if (ret < 0) {
		aw_dev_err(aw_dev->dev, "%s: get msg_num failed \n", __func__);
		return ret;
	}

	if (aw_dev->channel == AW_DEV_CH_PRI_L || aw_dev->channel == AW_DEV_CH_SEC_L) {
		msg_id = AW_MSG_ID_CALI_CFG_L;
	} else if(aw_dev->channel == AW_DEV_CH_PRI_R || aw_dev->channel == AW_DEV_CH_SEC_R){
		msg_id = AW_MSG_ID_CALI_CFG_R;
	}

	ret = aw_write_msg_to_dsp(msg_num, msg_id, data, data_len);
	if (ret) {
		aw_dev_err(aw_dev->dev, "%s: write cali_cfg failed \n", __func__);
		return ret;
	}

	aw_dev_dbg(aw_dev->dev,"%s: write cali_cfg done\n", __func__);
	return 0;
}

int aw_dsp_read_cali_cfg(struct aw_device *aw_dev, char *data, unsigned int data_len)
{
	uint32_t msg_id;
	int ret;
	int msg_num;

	ret = aw_get_msg_num(aw_dev->channel, &msg_num);
	if (ret < 0) {
		aw_dev_err(aw_dev->dev, "%s: get msg_num failed \n", __func__);
		return ret;
	}

	if (aw_dev->channel == AW_DEV_CH_PRI_L || aw_dev->channel == AW_DEV_CH_SEC_L) {
		msg_id = AW_MSG_ID_CALI_CFG_L;
	} else if(aw_dev->channel == AW_DEV_CH_PRI_R || aw_dev->channel == AW_DEV_CH_SEC_R){
		msg_id = AW_MSG_ID_CALI_CFG_R;
	}

	ret = aw_read_msg_from_dsp(msg_num, msg_id, data, data_len);
	if (ret) {
		aw_dev_err(aw_dev->dev, "%s: read cali_cfg failed \n", __func__);
		return ret;
	}
	aw_dev_dbg(aw_dev->dev,"%s: read cali_cfg done\n", __func__);
	return 0;
}

int aw_dsp_write_noise(struct aw_device *aw_dev, char *data, unsigned int data_len)
{
	uint32_t msg_id;
	int ret;
	int msg_num;

	ret = aw_get_msg_num(aw_dev->channel, &msg_num);
	if (ret < 0) {
		aw_dev_err(aw_dev->dev, "%s: get msg_num failed \n", __func__);
		return ret;
	}

	if (aw_dev->channel == AW_DEV_CH_PRI_L || aw_dev->channel == AW_DEV_CH_SEC_L) {
		msg_id = AW_MSG_ID_NOISE_L;
	} else if (aw_dev->channel == AW_DEV_CH_PRI_R || aw_dev->channel == AW_DEV_CH_SEC_R){
		msg_id = AW_MSG_ID_NOISE_R;
	}

	ret = aw_write_msg_to_dsp(msg_num, msg_id, data, data_len);
	if (ret) {
		aw_dev_err(aw_dev->dev, "%s: write noise failed \n", __func__);
		return ret;
	}
	aw_dev_dbg(aw_dev->dev,"%s: write noise[%d] done\n", __func__, data[0]);
	return 0;
}

int aw_dsp_write_vmax(struct aw_device *aw_dev, char *data, unsigned int data_len)
{
	uint32_t msg_id;
	int ret;
	int msg_num;

	ret = aw_get_msg_num(aw_dev->channel, &msg_num);
	if (ret < 0) {
		aw_dev_err(aw_dev->dev, "%s: get msg_num failed \n", __func__);
		return ret;
	}

	if (aw_dev->channel == AW_DEV_CH_PRI_L || aw_dev->channel == AW_DEV_CH_SEC_L) {
		msg_id = AW_MSG_ID_VMAX_L;
	} else if (aw_dev->channel == AW_DEV_CH_PRI_R || aw_dev->channel == AW_DEV_CH_SEC_R){
		msg_id = AW_MSG_ID_VMAX_R;
	}

	ret = aw_write_msg_to_dsp(msg_num, msg_id, data, data_len);
	if (ret) {
		aw_dev_err(aw_dev->dev, "%s: write vmax failed \n", __func__);
		return ret;
	}
	aw_dev_dbg(aw_dev->dev,"%s: write vmax done\n", __func__);
	return 0;
}

int aw_dsp_read_vmax(struct aw_device *aw_dev, char *data, unsigned int data_len)
{
	uint32_t msg_id;
	int ret;
	int msg_num;

	ret = aw_get_msg_num(aw_dev->channel, &msg_num);
	if (ret < 0) {
		aw_dev_err(aw_dev->dev, "%s: get msg_num failed \n", __func__);
		return ret;
	}

	if (aw_dev->channel == AW_DEV_CH_PRI_L || aw_dev->channel == AW_DEV_CH_SEC_L) {
		msg_id = AW_MSG_ID_VMAX_L;
	} else if (aw_dev->channel == AW_DEV_CH_PRI_R || aw_dev->channel == AW_DEV_CH_SEC_R){
		msg_id = AW_MSG_ID_VMAX_R;
	}

	ret = aw_read_msg_from_dsp(msg_num, msg_id, data, data_len);
	if (ret) {
		aw_dev_err(aw_dev->dev, "%s: read vmax failed \n", __func__);
		return ret;
	}
	aw_dev_dbg(aw_dev->dev,"%s: read vmax done\n", __func__);
	return 0;
}

int aw_dsp_write_params(struct aw_device *aw_dev, char *data, unsigned int data_len)
{
	uint32_t msg_id;
	int ret;
	int msg_num;

	ret = aw_get_msg_num(aw_dev->channel, &msg_num);
	if (ret < 0) {
		aw_dev_err(aw_dev->dev, "%s: get msg_num failed \n", __func__);
		return ret;
	}

	if (msg_num == MSG_PARAM_ID_0) {
		msg_id = AW_MSG_ID_PARAMS;
	} else {
		msg_id = AW_MSG_ID_PARAMS_1;
	}

	ret = aw_write_data_to_dsp(msg_id, data, data_len);
	if (ret) {
		aw_dev_err(aw_dev->dev, "%s: write params failed \n", __func__);
		return ret;
	}
	aw_dev_dbg(aw_dev->dev,"%s: write params done\n", __func__);
	return 0;
}

int aw_dsp_write_cali_re(struct aw_device *aw_dev, char *data, unsigned int data_len)
{
	uint32_t msg_id;
	int ret;
	int msg_num;

	ret = aw_get_msg_num(aw_dev->channel, &msg_num);
	if (ret < 0) {
		aw_dev_err(aw_dev->dev, "%s: get msg_num failed \n", __func__);
		return ret;
	}

	if (aw_dev->channel == AW_DEV_CH_PRI_L || aw_dev->channel == AW_DEV_CH_SEC_L) {
		msg_id = AW_MSG_ID_RE_L;
	} else if (aw_dev->channel == AW_DEV_CH_PRI_R || aw_dev->channel == AW_DEV_CH_SEC_R){
		msg_id = AW_MSG_ID_RE_R;
	}

	ret = aw_write_msg_to_dsp(msg_num, msg_id, data, data_len);
	if (ret) {
		aw_dev_err(aw_dev->dev, "%s: write cali re failed \n", __func__);
		return ret;
	}
	aw_dev_dbg(aw_dev->dev,"%s: write cali re done\n", __func__);
	return 0;
}

int aw_dsp_read_cali_re(struct aw_device *aw_dev, char *data, unsigned int data_len)
{
	uint32_t msg_id;
	int ret;
	int msg_num;

	ret = aw_get_msg_num(aw_dev->channel, &msg_num);
	if (ret < 0) {
		aw_dev_err(aw_dev->dev, "%s: get msg_num failed \n", __func__);
		return ret;
	}

	if (aw_dev->channel == AW_DEV_CH_PRI_L || aw_dev->channel == AW_DEV_CH_SEC_L) {
		msg_id = AW_MSG_ID_RE_L;
	} else if (aw_dev->channel == AW_DEV_CH_PRI_R || aw_dev->channel == AW_DEV_CH_SEC_R){
		msg_id = AW_MSG_ID_RE_R;
	}

	ret = aw_read_msg_from_dsp(msg_num, msg_id, data, data_len);
	if (ret) {
		aw_dev_err(aw_dev->dev, "%s: read cali re failed \n", __func__);
		return ret;
	}
	aw_dev_dbg(aw_dev->dev,"%s: read cali re done\n", __func__);
	return 0;
}

int aw_dsp_write_hmute(struct aw_device *aw_dev, char *data, unsigned int data_len)
{
	int ret;
	int msg_num;

	ret = aw_get_msg_num(aw_dev->channel, &msg_num);
	if (ret < 0) {
		aw_dev_err(aw_dev->dev, "%s: get msg_num failed \n", __func__);
		return ret;
	}

	ret = aw_write_msg_to_dsp(msg_num, AW_MSG_ID_ENABLE_HMUTE, data, data_len);
	if (ret) {
		aw_dev_err(aw_dev->dev, "%s: write hmue failed \n", __func__);
		return ret;
	}
	aw_dev_dbg(aw_dev->dev,"%s: write hmute[%d]\n", __func__, data[0]);
	return 0;
}

int aw_dsp_write_cali_en(struct aw_device *aw_dev, char *data, unsigned int data_len)
{
	int ret;
	int msg_num;

	ret = aw_get_msg_num(aw_dev->channel, &msg_num);
	if (ret < 0) {
		aw_dev_err(aw_dev->dev, "%s: get msg_num failed \n", __func__);
		return ret;
	}

	ret = aw_write_msg_to_dsp(msg_num, AW_MSG_ID_ENABLE_CALI, data, data_len);
	if (ret) {
		aw_dev_err(aw_dev->dev, "%s: write cali en failed \n", __func__);
		return ret;
	}
	aw_dev_dbg(aw_dev->dev,"%s: write cali_en[%d]\n", __func__, data[0]);
	return 0;
}

int aw_dsp_read_f0(struct aw_device *aw_dev, char *data, unsigned int data_len)
{
	uint32_t msg_id;
	int ret;
	int msg_num;

	ret = aw_get_msg_num(aw_dev->channel, &msg_num);
	if (ret < 0) {
		aw_dev_err(aw_dev->dev, "%s: get msg_num failed \n", __func__);
		return ret;
	}

	if (aw_dev->channel == AW_DEV_CH_PRI_L || aw_dev->channel == AW_DEV_CH_SEC_L) {
		msg_id = AW_MSG_ID_F0_L;
	} else if (aw_dev->channel == AW_DEV_CH_PRI_R || aw_dev->channel == AW_DEV_CH_SEC_R){
		msg_id = AW_MSG_ID_F0_R;
	}

	ret = aw_read_msg_from_dsp(msg_num, msg_id, data, data_len);
	if (ret) {
		aw_dev_err(aw_dev->dev, "%s: read f0 failed \n", __func__);
		return ret;
	}
	aw_dev_dbg(aw_dev->dev,"%s: read f0 \n", __func__);
	return 0;
}

int aw_dsp_read_f0_q(struct aw_device *aw_dev, char *data, unsigned int data_len)
{
	int ret;
	int msg_num;

	ret = aw_get_msg_num(aw_dev->channel, &msg_num);
	if (ret < 0) {
		aw_dev_err(aw_dev->dev, "%s: get msg_num failed \n", __func__);
		return ret;
	}

	ret = aw_read_msg_from_dsp(msg_num, AW_MSG_ID_F0_Q, data, data_len);
	if (ret) {
		aw_dev_err(aw_dev->dev, "%s: read f0 & q failed \n", __func__);
		return ret;
	}
	aw_dev_dbg(aw_dev->dev,"%s: read f0 & q\n", __func__);
	return 0;
}

int aw_dsp_read_cali_data(struct aw_device *aw_dev, char *data, unsigned int data_len)
{
	uint32_t msg_id;
	int ret;
	int msg_num;

	ret = aw_get_msg_num(aw_dev->channel, &msg_num);

	aw_dev_info(aw_dev->dev,"%s: channel=%d, msg_num=%d\n", __func__, aw_dev->channel, msg_num);

	if (ret < 0) {
		aw_dev_err(aw_dev->dev, "%s: get msg_num failed \n", __func__);
		return ret;
	}

	if (aw_dev->channel == AW_DEV_CH_PRI_L || aw_dev->channel == AW_DEV_CH_SEC_L) {
		msg_id = AW_MSG_ID_REAL_DATA_L;
	} else if (aw_dev->channel == AW_DEV_CH_PRI_R || aw_dev->channel == AW_DEV_CH_SEC_R){
		msg_id = AW_MSG_ID_REAL_DATA_R;
	}

	ret = aw_read_msg_from_dsp(msg_num, msg_id, data, data_len);
	if (ret) {
		aw_dev_err(aw_dev->dev, "%s: read cali dara failed \n", __func__);
		return ret;
	}
	aw_dev_dbg(aw_dev->dev,"%s: read cali_data\n", __func__);
	return 0;
}

int aw_dsp_set_afe_module_en(int type, int enable)
{
	int32_t value = enable;
	if (type == AW_RX_MODULE) {
		return aw_send_afe_rx_module_enable(&value, sizeof(int32_t));
	} else {
		return aw_send_afe_tx_module_enable(&value, sizeof(int32_t));
	}
}

int aw_dsp_get_afe_module_en(int type, int *status)
{
	int ret;
	int32_t value;
	if (type == AW_RX_MODULE) {
		ret = aw_read_data_from_dsp(AW_MSG_ID_RX_SET_ENABLE, &value, sizeof(int32_t));
		if (ret) {
			pr_err("Awinic %s: read afe rx failed \n", __func__);
			return ret;
		}
		*status = value;
		return 0;
	} else {
		ret = aw_read_data_from_dsp(AW_MSG_ID_TX_SET_ENABLE, &value, sizeof(int32_t));
		if (ret) {
			pr_err("Awinic %s: read afe tx failed \n", __func__);
			return ret;
		}
		*status = value;
		return 0;
	}
}

int aw_dsp_set_copp_module_en(bool enable)
{
	int ret;
	int port_id = AFE_PORT_ID_AWDSP_RX;
	int module_id = AW_COPP_MODULE_ID;
	int param_id =  AW_COPP_MODULE_PARAMS_ID_EN;

	ret = aw_adm_param_enable(port_id, module_id, param_id, enable);
	if (ret) {
		return -EINVAL;
	}

	pr_info("%s: set skt %s", __func__, enable == 1 ? "enable" : "disable");
	return 0;
}

int aw_dsp_write_spin(int spin_mode)
{
	int ret;
	int32_t spin = spin_mode;

	if (spin >= AW_SPIN_MAX) {
		pr_err("[Awinic] %s: spin [%d] unsupported \n", __func__, spin);
		return -EINVAL;
	}

	ret = aw_write_data_to_dsp(AW_MSG_ID_SPIN, &spin, sizeof(int32_t));
	if (ret) {
		pr_err("[Awinic] %s: write spin failed \n", __func__);
		return ret;
	}
	pr_debug("[Awinic] %s: write spin done\n", __func__);
	return 0;
}

int aw_dsp_read_spin(int *spin_mode)
{
	int ret;
	int32_t spin;

	ret = aw_read_data_from_dsp(AW_MSG_ID_SPIN, &spin, sizeof(int32_t));
	if (ret) {
		*spin_mode = 0;
		pr_err("[Awinic] %s: read spin failed \n", __func__);
		return ret;
	}
	*spin_mode = spin;
	pr_debug("[Awinic] %s: read spin done\n", __func__);
	return 0;
}


