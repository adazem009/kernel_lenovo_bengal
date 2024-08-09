#ifndef __AWINIC_DSP_H__
#define __AWINIC_DSP_H__

#define AFE_PORT_ID_AWDSP_RX  (0x9000)     //AFE_PORT_ID_QUATERNARY_MI2S_RX


#define AW_DSP_TRY_TIME   (3)
#define AW_DSP_SLEEP_TIME (10)

enum aw_dsp_msg_type {
	AW_DSP_MSG_TYPE_DATA = 0,
	AW_DSP_MSG_TYPE_CMD = 1,
};

enum {
	AW_SPIN_0 = 0,
	AW_SPIN_90,
	AW_SPIN_180,
	AW_SPIN_270,
	AW_SPIN_MAX,
};

#define AW_DSP_MSG_HDR_VER (1)
typedef struct aw_msg_hdr aw_dsp_msg_t;

int aw_dsp_write_msg(struct aw_device *aw_dev, uint32_t msg_id, char *data_ptr, unsigned int data_size);
int aw_dsp_read_msg(struct aw_device *aw_dev, uint32_t msg_id, char *data_ptr, unsigned int data_size);
int aw_dsp_write_cali_cfg(struct aw_device *aw_dev, char *data, unsigned int data_len);
int aw_dsp_read_cali_cfg(struct aw_device *aw_dev, char *data, unsigned int data_len);
int aw_dsp_write_noise(struct aw_device *aw_dev, char *data, unsigned int data_len);
int aw_dsp_write_vmax(struct aw_device *aw_dev, char *data, unsigned int data_len);
int aw_dsp_read_vmax(struct aw_device *aw_dev, char *data, unsigned int data_len);
int aw_dsp_write_params(struct aw_device *aw_dev, char *data, unsigned int data_len);
int aw_dsp_write_cali_re(struct aw_device *aw_dev, char *data, unsigned int data_len);
int aw_dsp_read_cali_re(struct aw_device *aw_dev, char *data, unsigned int data_len);
int aw_dsp_write_hmute(struct aw_device *aw_dev, char *data, unsigned int data_len);
int aw_dsp_write_cali_en(struct aw_device *aw_dev, char *data, unsigned int data_len);
int aw_dsp_read_f0(struct aw_device *aw_dev, char *data, unsigned int data_len);
int aw_dsp_read_f0_q(struct aw_device *aw_dev, char *data, unsigned int data_len);
int aw_dsp_read_cali_data(struct aw_device *aw_dev, char *data, unsigned int data_len);
int aw_dsp_set_afe_module_en(int type, int enable);
int aw_dsp_get_afe_module_en(int type, int *status);
int aw_dsp_set_copp_module_en(bool enable);
int aw_dsp_write_spin(int spin_mode);
int aw_dsp_read_spin(int* spin_mode);

#endif

