#ifndef __AWINIC_CALIBRATION_H__
#define __AWINIC_CALIBRATION_H__

#include <linux/miscdevice.h>

#define AW_CALI_STORE_EXAMPLE
#define AW_ERRO_CALI_VALUE (0)


#define AW_CALI_RE_DEFAULT_TIMER	(3000)

#define AW_CALI_RE_MAX     (15000)
#define AW_CALI_RE_MIN     (4000)

#define AW_CALI_CFG_NUM (3)
#define AW_CALI_DATA_NUM (6)
#define AW_PARAMS_NUM (400)
#define AW_KILO_PARAMS_NUM (1000)

/*factor form 12bit(4096) to 1000*/
#define AW_DSP_RE_TO_SHOW_RE(re)  (((re) * (1000)) >> (12))
#define AW_SHOW_RE_TO_DSP_RE(re)  (((re) << 12) / (1000))

enum afe_module_type {
	AW_RX_MODULE = 0,
	AW_TX_MODULE = 1,
};


struct cali_cfg {
	int32_t data[AW_CALI_CFG_NUM];
};
struct cali_data {
	int32_t data[AW_CALI_DATA_NUM];
};
struct params_data {
	int32_t data[AW_PARAMS_NUM];
};
struct ptr_params_data {
	int len;
	int32_t *data;
};
struct f0_q_data{
	int32_t data[4];
};

enum {
	AW_IOCTL_MSG_IOCTL = 0,
	AW_IOCTL_MSG_RD_DSP,
	AW_IOCTL_MSG_WR_DSP
};

#define AW_IOCTL_MSG_VERSION (0)
typedef struct {
	int32_t type;
	int32_t opcode_id;
	int32_t version;
	int32_t data_len;
	char *data_buf;
	int32_t reseriver[2];
}aw_ioctl_msg_t;



#define AW_IOCTL_MAGIC				'a'
#define AW_IOCTL_SET_CALI_CFG			_IOWR(AW_IOCTL_MAGIC, 1, struct cali_cfg)
#define AW_IOCTL_GET_CALI_CFG			_IOWR(AW_IOCTL_MAGIC, 2, struct cali_cfg)
#define AW_IOCTL_GET_CALI_DATA			_IOWR(AW_IOCTL_MAGIC, 3, struct cali_data)
#define AW_IOCTL_SET_NOISE			_IOWR(AW_IOCTL_MAGIC, 4, int32_t)
#define AW_IOCTL_GET_F0				_IOWR(AW_IOCTL_MAGIC, 5, int32_t)
#define AW_IOCTL_SET_CALI_RE			_IOWR(AW_IOCTL_MAGIC, 6, int32_t)
#define AW_IOCTL_GET_CALI_RE			_IOWR(AW_IOCTL_MAGIC, 7, int32_t)
#define AW_IOCTL_SET_VMAX			_IOWR(AW_IOCTL_MAGIC, 8, int32_t)
#define AW_IOCTL_GET_VMAX			_IOWR(AW_IOCTL_MAGIC, 9, int32_t)
#define AW_IOCTL_SET_PARAM			_IOWR(AW_IOCTL_MAGIC, 10, struct params_data)
#define AW_IOCTL_ENABLE_CALI			_IOWR(AW_IOCTL_MAGIC, 11, int8_t)
#define AW_IOCTL_SET_PTR_PARAM_NUM		_IOWR(AW_IOCTL_MAGIC, 12, struct ptr_params_data)
#define AW_IOCTL_GET_F0_Q			_IOWR(AW_IOCTL_MAGIC, 13, struct f0_q_data)
#define AW_IOCTL_SET_DSP_HMUTE			_IOWR(AW_IOCTL_MAGIC, 14, int32_t)
#define AW_IOCTL_SET_CALI_CFG_FLAG		_IOWR(AW_IOCTL_MAGIC, 15, int32_t)
#define AW_IOCTL_MSG				_IOWR(AW_IOCTL_MAGIC, 16, aw_ioctl_msg_t)


enum{
	AW_CALI_MODE_NONE = 0,
	AW_CALI_MODE_ATTR = 0,
	AW_CALI_MODE_DBGFS,
	AW_CALI_MODE_MISC,
	AW_CALI_MODE_MAX
};

struct aw_dbg_cali {
	struct dentry *dbg_dir;
	struct dentry *dbg_range;
	struct dentry *dbg_cali;
	struct dentry *dbg_status;
	struct dentry *dbg_f0;
};

struct aw_cali_desc{
	unsigned char status;
	unsigned char mode;	/*0:ATTR 1:DBGFS 2:MISC */
	int32_t cali_re;	/*store cali_re*/
	int32_t cali_f0;	/*store cali_f0*/
	int32_t t_re;		/*store cali temp re*/
	int32_t t_f0;		/*store cali temp f0*/
	uint32_t time;		/*cali wait time before read cali data*/

	struct miscdevice misc;
	struct aw_dbg_cali dbg;
};


void aw_cali_init(struct aw_cali_desc *cali_desc);
void aw_cali_deinit(struct aw_cali_desc *cali_desc);
int aw_cali_read_re_from_nvram(int32_t *cali_re, int32_t ch_index);

#endif


