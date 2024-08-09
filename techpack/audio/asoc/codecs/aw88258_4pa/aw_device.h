#ifndef __AWINIC_DEVICE_FILE_H__
#define __AWINIC_DEVICE_FILE_H__
#include "aw_data_type.h"
#include "aw_calibration.h"
#include "aw_monitor.h"

#define AW_FADE_OUT_TARGET_VOL  (90 * 2)
#define AW_1000_US    (1000)


struct aw_device;


enum {
	AW_DEV_CH_PRI_L = 0,
	AW_DEV_CH_PRI_R = 1,
	AW_DEV_CH_SEC_L = 2,
	AW_DEV_CH_SEC_R = 3,
	AW_DEV_CH_MAX,
};

enum AW_DEV_INIT {
	AW_DEV_INIT_ST = 0,
	AW_DEV_INIT_OK = 1,
	AW_DEV_INIT_NG = 2,
};

enum AW_DEV_STATUS{
	AW_DEV_PW_OFF = 0,
	AW_DEV_PW_ON,
};

enum AW_DEV_FW_STATUS{
	AW_DEV_FW_FAILED = 0,
	AW_DEV_FW_OK,
};

enum {
	AW_START_TYPE_STREAM = 0,
	AW_START_TYPE_PROF,
	AW_START_TYPE_SWITCH,
};

#define IS_DEVICE_PWD(dev)   (dev->status)

struct aw_device_ops{
	int (*aw_i2c_write)(struct aw_device *aw_dev, unsigned char reg_addr, unsigned int reg_data);
	int (*aw_i2c_read)(struct aw_device *aw_dev, unsigned char reg_addr, unsigned int *reg_data);
	int (*aw_i2c_write_bits)(struct aw_device *aw_dev, unsigned char reg_addr, unsigned int mask, unsigned int reg_data);
	int (*aw_set_volume)(struct aw_device *aw_dev, unsigned int value);
	int (*aw_get_volume)(struct aw_device *aw_dev, unsigned int *value);
	int (*aw_set_afe_module_en)(int, int);
	int (*aw_get_afe_module_en)(int, int*);
	int (*aw_set_copp_module_en)(bool enable);
	int (*aw_set_spin_mode)(int spin_mode);
	int (*aw_get_spin_mode)(int *spin_mode);
};

struct aw_int_desc{
	int reg;
	int reg_default;
	unsigned int int_reg_mask;
};

struct aw_pwd_desc{
	unsigned int reg;
	unsigned int mask;
	unsigned int enable;
	unsigned int disable;
};

struct aw_vcalb_desc{
	unsigned int icalk_reg;
	unsigned int icalk_reg_mask;
	unsigned int icalk_sign_mask;
	unsigned int icalk_neg_mask;
	int icalk_value_factor;

	unsigned int vcalk_reg;
	unsigned int vcalk_reg_mask;
	unsigned int vcalk_sign_mask;
	unsigned int vcalk_neg_mask;
	int vcalk_value_factor;

	unsigned int vcalb_reg;
	int cabl_base_value;
	int vcal_factor;
};

struct aw_mute_desc{
	unsigned int reg;
	unsigned int mask;
	unsigned int enable;
	unsigned int disable;
};

struct aw_sysst_desc{
	unsigned int reg;
	unsigned int mask;
	unsigned int check;
};

struct aw_volume_desc{
	unsigned int reg;
	unsigned int mask;
	unsigned int shift;
	int init_volume;
	unsigned int step;
	unsigned int in_step_time;
	unsigned int out_step_time;
};

struct aw_prof_info{
	int count;
	struct aw_prof_desc* desc;
};

struct aw_device{
	int index;
	int status;

	unsigned char cur_prof;  /*current profile index*/
	unsigned char set_prof;  /*set profile index*/
	unsigned int channel;    /*pa channel select*/

	struct device *dev;
	char *acf;
	void *private_data;
	unsigned int int_status_reg;
	struct aw_int_desc int_desc;
	struct aw_pwd_desc pwd_desc;
	struct aw_mute_desc mute_desc;
	struct aw_vcalb_desc vcalb_desc;
	struct aw_sysst_desc sysst_desc;
	struct aw_volume_desc volume_desc;
	struct aw_prof_info prof_info;
	struct aw_cali_desc cali_desc;
	struct aw_monitor_desc monitor_desc;
	struct aw_device_ops ops;
};


int aw_load_acf_check(aw_acf_t *acf_data, int data_size);
void aw_dev_deinit(struct aw_device *aw_dev);
int aw_device_init(struct aw_device *aw_dev, aw_acf_t *acf_data);
int aw_device_start(struct aw_device *aw_dev, int type);
int aw_device_stop(struct aw_device *aw_dev);
int aw_dev_load_fw(struct aw_device *aw_dev);
int aw_dev_get_profile_count(struct aw_device *aw_dev);
int aw_dev_get_profile_name(struct aw_device *aw_dev, char *name, int index);
int aw_dev_check_profile_index(struct aw_device *aw_dev, int index);
int aw_dev_get_profile_index(struct aw_device *aw_dev);
int aw_dev_set_profile_index(struct aw_device *aw_dev, int index);
int aw_dev_prof_update(struct aw_device *aw_dev);
int aw_dev_get_status(struct aw_device *aw_dev);
unsigned int aw_dev_get_dsp_re(struct aw_device *aw_dev);
void aw_dev_set_volume_step(struct aw_device *aw_dev, unsigned int step);
int aw_dev_get_volume_step(struct aw_device *aw_dev);
int aw_device_probe(struct aw_device *aw_dev);
int aw_device_remove(struct aw_device *aw_dev);

#endif

