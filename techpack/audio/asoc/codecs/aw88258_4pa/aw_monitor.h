#ifndef __AW_MONITOR_H__
#define __AW_MONITOR_H__


struct aw_table;
#define AW_MONITOR_DEFAULT_FLAG (0)
#define AW_MONITOR_DEFAULT_TIMER_VAL (30000)
#define AW882XX_MONITOR_VBAT_RANGE (6025)
#define AW882XX_MONITOR_INT_10BIT (1023)
#define AW882XX_MONITOR_TEMP_SIGN_MASK (1<<9)
#define AW882XX_MONITOR_TEMP_NEG_MASK (0XFC00)
#define AW882XX_BIT_SYSCTRL2_BST_IPEAK_MASK (15<<0)
#define AW882XX_BIT_HAGCCFG4_GAIN_SHIFT (8)
#define AW882XX_BIT_HAGCCFG4_GAIN_MASK (0x00ff)

#define MONITOR_CFG_VER_MAX		(4)
#define AW_TABLE_SIZE	sizeof(struct aw_table)
#define IPEAK_NONE	(0xFF)
#define GAIN_NONE	(0xFF)

#define GET_32_DATA(w, x, y, z) \
	((unsigned int)(((w) << 24) | ((x) << 16) | ((y) << 8) | (z)))
#define GET_16_DATA(x, y) \
		((uint16_t)(((x) << 8) | y))



enum aw_monitor_hdr_ver {
	AW_MONITOR_HDR_VER_0_0_1 = 0x00000100,
};

enum aw_monitor_init {
	AW_MON_CFG_ST = 0,
	AW_MON_CFG_OK = 1,
};


struct aw_monitor_hdr {
	uint32_t check_sum;
	uint32_t monitor_ver;
	char chip_type[8];
	uint32_t ui_ver;
	uint32_t monitor_flag;
	uint32_t monitor_time;
	uint32_t monitor_count;
	uint32_t temp_flag;
	uint32_t vol_flag;
	uint32_t temp_num;
	uint32_t temp_offset;
	uint32_t vol_num;
	uint32_t vol_offset;
	uint32_t reserve[3];
};

struct aw_table {
	int16_t val_min;
	int16_t val_max;
	uint16_t ipeak;
	uint16_t gain;
};

struct aw_table_info {
	uint8_t table_num;
	struct aw_table *aw_table;
};

struct aw_monitor_cfg {
	uint8_t monitor_status;
	uint32_t monitor_flag;
	uint32_t monitor_time;
	uint32_t monitor_count;
	uint32_t temp_flag;
	uint32_t vol_flag;
	struct aw_table_info temp_info;
	struct aw_table_info vol_info;
};

struct aw_voltage_desc {
	unsigned int reg;
	unsigned int vbat_range;
	unsigned int int_bit;
};

struct aw_temperature_desc {
	unsigned int reg;
	unsigned int sign_mask;
	unsigned int neg_mask;
};

struct aw_gain_desc {
	unsigned int reg;
	unsigned int shift;
	unsigned int mask;
};

struct aw_ipeak_desc {
	unsigned int reg;
	unsigned int mask;
};


/******************************************************************
* struct aw882xx monitor
*******************************************************************/
struct aw_monitor_desc {
	struct hrtimer timer;
	struct work_struct work;
	struct aw_voltage_desc voltage_desc;
	struct aw_temperature_desc temp_desc;
	struct aw_gain_desc gain_desc;
	struct aw_ipeak_desc ipeak_desc;
	uint32_t is_enable;

	uint16_t pre_vol;
	uint16_t vol_ipeak;
	uint16_t vol_gain;

	int16_t pre_temp;
	uint16_t temp_ipeak;
	uint16_t temp_gain;

#ifdef AW_DEBUG
	uint16_t test_vol;
	int16_t test_temp;
#endif
};

/******************************************************************
* aw882xx monitor functions
*******************************************************************/
int aw_monitor_start(struct aw_monitor_desc *monitor_desc);
int aw_monitor_stop(struct aw_monitor_desc *monitor_desc);
void aw_monitor_init(struct aw_monitor_desc *monitor_desc);
void aw_monitor_deinit(struct aw_monitor_desc *monitor_desc);
int aw_load_monitor_cfg(struct aw_monitor_desc *monitor_desc);




#endif
