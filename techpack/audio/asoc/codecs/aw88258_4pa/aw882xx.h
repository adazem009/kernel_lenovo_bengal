#ifndef _AW882XX_H_
#define _AW882XX_H_
#include <linux/kernel.h>
#include <linux/list.h>
#include "aw_device.h"
#include "aw_dsp.h"

/*
 * i2c transaction on Linux limited to 64k
 * (See Linux kernel documentation: Documentation/i2c/writing-clients)
*/
#define MAX_I2C_BUFFER_SIZE					65536
#define AW882XX_I2C_READ_MSG_NUM		2

#define AW_I2C_RETRIES			5	/* 5 times */
#define AW_I2C_RETRY_DELAY		5	/* 5 ms */


#define AW882XX_RATES SNDRV_PCM_RATE_8000_48000
#define AW882XX_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | \
						SNDRV_PCM_FMTBIT_S24_LE | \
						SNDRV_PCM_FMTBIT_S32_LE)

#define AW882XX_VOLUME_STEP_DB   (6 * 2)

enum aw882xx_chipid {
	AW882XX_ID = 0x1852,
};

#if 1  // LINUX_VERSION_CODE >= KERNEL_VERSION(4, 19, 1)
#define AW_KERNEL_VER_OVER_4_19_1
#endif

#ifdef  AW_KERNEL_VER_OVER_4_19_1
typedef struct snd_soc_component aw_snd_soc_codec_t;
typedef struct snd_soc_component_driver aw_snd_soc_codec_driver_t;
#else
typedef struct snd_soc_codec aw_snd_soc_codec_t;
typedef struct snd_soc_codec_driver aw_snd_soc_codec_driver_t;
#endif

struct aw_componet_codec_ops {
	aw_snd_soc_codec_t *(*kcontrol_codec)(struct snd_kcontrol *kcontrol);
	void *(*codec_get_drvdata)(aw_snd_soc_codec_t *codec);
	int (*add_codec_controls)(aw_snd_soc_codec_t *codec,
		const struct snd_kcontrol_new *controls, unsigned int num_controls);
	void (*unregister_codec)(struct device *dev);
	int (*register_codec)(struct device *dev,
			const aw_snd_soc_codec_driver_t *codec_drv,
			struct snd_soc_dai_driver *dai_drv,
			int num_dai);
};


/********************************************
 * struct aw882xx
 *******************************************/
struct aw882xx {
	int sysclk;
	int rate;
	int pstream;
	int cstream;

	int index;
	unsigned char dbg_en_prof;     /*debug enable/disable profile function*/
	unsigned int allow_pw;         /*allow power*/
	int reset_gpio;
	int irq_gpio;
	unsigned int chip_id;
	unsigned char fw_status;
	unsigned char rw_reg_addr;   /*rw attr node store read addr*/

	struct list_head list; 
	aw_snd_soc_codec_t *codec;
	struct aw_componet_codec_ops *codec_ops;

	struct i2c_client *i2c;
	struct device *dev;
	struct aw_device *aw_pa;

	struct workqueue_struct *work_queue;
	struct delayed_work start_work;
	struct delayed_work monitor_work;

	struct mutex lock;
};


#endif

