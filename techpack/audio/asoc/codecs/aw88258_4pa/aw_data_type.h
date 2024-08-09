#ifndef __AWINIC_DATA_TYPE_H__
#define __AWINIC_DATA_TYPE_H__

#define AW_NAME_BUF_MAX (50)

struct aw_msg_hdr {
	int32_t type;
	int32_t opcode_id;
	int32_t version;
	int32_t reseriver[3];
};


/******************************************************************
* aw profile
*******************************************************************/
enum {
	AW_PROFILE_MUSIC = 0,
	AW_PROFILE_VOICE,
	AW_PROFILE_VOIP,
	AW_PROFILE_RINGTONE,
	AW_PROFILE_RINGTONE_HS,
	AW_PROFILE_LOWPOWER,
	AW_PROFILE_BYPASS,
	AW_PROFILE_MMI,
	AW_PROFILE_FM,
	AW_PROFILE_NOTIFICATION,
	AW_PROFILE_RECEIVER,
	AW_PROFILE_MAX,
};

enum {
	AW_PROFILE_DATA_TYPE_REG = 0,
	AW_PROFILE_DATA_TYPE_DSP,
	AW_PROFILE_DATA_TYPE_FW,
	AW_PROFILE_DATA_TYPE_MAX,
};

struct aw_sec_data_desc{
	char *data;
	uint32_t len;
};

struct aw_prof_desc{
	uint32_t id;
	struct aw_sec_data_desc sec_desc[AW_PROFILE_DATA_TYPE_MAX];
};

#define PROJECT_NAME_MAX (24)
#define CUSTOMER_NAME_MAX (16)
#define CFG_VERSION_MAX (4)

#define ACF_FILE_ID	(0xa15f908)

typedef struct {
	int32_t a_id;					/*acf file ID 0xa15f908*/
	char a_project[PROJECT_NAME_MAX];		/*project name*/
	char a_custom[CUSTOMER_NAME_MAX];		/*custom name :huawei xiaomi vivo oppo*/
	char a_version[CFG_VERSION_MAX];		/*author update version*/
	int32_t a_author_id;				/*author id*/
	int32_t a_entry_size;				/*sub section table entry size*/
	int32_t a_entry_num;				/*sub section table entry num*/
	int32_t a_sh_offset;				/*sub section table offset in file*/
	int32_t reserve[4];
}acf_hdr_t;

enum {
	ACF_SEC_TYPE_DEVICE  = 0x00000001,
	ACF_SEC_TYPE_DEV_VER = 0x00000002,
	ACF_SEC_TYPE_SCENE	 = 0x00000003,
	ACF_SEC_TYPE_REG	 = 0x00000004,
	ACF_SEC_TYPE_DSP	 = 0x00000005,
	ACF_SEC_TYPE_FW		 = 0x00000006,
};


//sec info keep 3*4 size
struct aw_sec_suffix1{
	int32_t profile_id;
	int32_t reserve[2];
};

struct aw_sec_suffix2{
	int32_t dev_index;
	int32_t prof_num;
	int32_t reserve[1];
};

typedef union {
	struct aw_sec_suffix1 reg_info;
	struct aw_sec_suffix1 dsp_info;
	struct aw_sec_suffix2 dev_info;
	int32_t reserve[3];
}sec_info_u;

typedef struct {
	int32_t a_type;					/*section type id*/
	int32_t a_offset;				/*section offset in file*/
	int32_t a_size;					/*section size in file*/
	sec_info_u info;
}acf_shdr_t;


typedef struct {
	acf_hdr_t hdr;
} aw_acf_t;


#endif

