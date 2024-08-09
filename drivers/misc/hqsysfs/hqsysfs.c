/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <linux/hqsysfs.h>

#include "hqsys_misc.h"
#include "hqsys_pcba.h"
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/gpio.h>

#define SDCARD_DET_N_GPIO  1255
#define HQ_SYS_FS_VER "2018-05-14 V0.3"

extern char *fcam_name;
extern char *rcam_name;

static HW_INFO(HWID_VER,ver);
static HW_INFO(HWID_SUMMARY,hw_summary);
static HW_INFO(HWID_DDR,ram);
static HW_INFO(HWID_EMMC,emmc);
static HW_INFO(HWID_LCM,lcm);
//static HW_INFO(HWID_BIAS_IC,lcm_bias_ic);
static HW_INFO(HWID_CTP,ctp);
static HW_INFO(HWID_MAIN_CAM,main_cam);
static HW_INFO(HWID_MAIN_CAM_2,main_cam_2);
static HW_INFO(HWID_MAIN_LENS,main_cam_len);
static HW_INFO(HWID_FLASHLIGHT,flashlight);
static HW_INFO(HWID_SUB_CAM,sub_cam);
static HW_INFO(HWID_GSENSOR,gsensor);
static HW_INFO(HWID_ALSPS,alsps);
static HW_INFO(HWID_MSENSOR,msensor);
static HW_INFO(HWID_GYRO,gyro);
static HW_INFO(HWID_IRDA,irda);
static HW_INFO(HWID_CAPSENSOR,cap_sensor);
static HW_INFO(HWID_BATERY,battery_manufacturer);
static HW_INFO(HWID_FUEL_GAUGE_IC,fuel_gauge_ic);
static HW_INFO(HWID_NFC,nfc);
static HW_INFO(HWID_FP,fingerprint);
//static HW_INFO(HWID_TEE,tee);
static HW_INFO(HWID_WIFI,wifi);
static HW_INFO(HWID_BT,bt);
static HW_INFO(HWID_FM,fm);
static HW_INFO(HWID_GPS,gps);
static HW_INFO(HWID_PCBA,pcba_config);

struct pcba_info pcba[]={
	{PCBA_WIFI,"PCBA_WIFI"},
	{PCBA_ROW_LTE_DATA,"PCBA_ROW_LTE_DATA"},
	{PCBA_PRC_LTE_DATA,"PCBA_PRC_LTE_DATA"},
	{PCBA_UNKNOW,"PCBA_UNKNOW"},

};

struct pcba_state_info pcba_state[]={
	{PCBA_STATE_EVT,"EVT"},
	{PCBA_STATE_DVT1,"DVT1"},
	{PCBA_STATE_DVT2,"DVT2"},
	{PCBA_STATE_PVT,"PVT"},
	{PCBA_STATE_UNKNOW,"STATE_UNKNOW"},

};


static PCBA_CONFIG huaqin_pcba_config=PCBA_UNKNOW;
static PCBA_STATE_CONFIG huaqin_pcba_state_config=PCBA_STATE_UNKNOW;
int hq_get_hw_state(void);
//extern char *battery_manufacturer_info;
static struct attribute *huaqin_attrs[] = {
	&hw_info_ver.attr,
	&hw_info_hw_summary.attr,
	&hw_info_ram.attr,
	&hw_info_emmc.attr,
	&hw_info_lcm.attr,
//	&hw_info_lcm_bias_ic.attr,
	&hw_info_ctp.attr,
	&hw_info_main_cam.attr,
	&hw_info_main_cam_2.attr,
	&hw_info_main_cam_len.attr,
	&hw_info_flashlight.attr,
	&hw_info_sub_cam.attr,
	&hw_info_gsensor.attr,
	&hw_info_alsps.attr,
	&hw_info_msensor.attr,
	&hw_info_gyro.attr,
	&hw_info_irda.attr,
	&hw_info_cap_sensor.attr,
	&hw_info_battery_manufacturer.attr,
	&hw_info_fuel_gauge_ic.attr,
	&hw_info_nfc.attr,
	&hw_info_fingerprint.attr,
//	&hw_info_tee.attr,
	&hw_info_wifi.attr,
	&hw_info_bt.attr,
	&hw_info_fm.attr,
	&hw_info_gps.attr,
	&hw_info_pcba_config.attr,
	NULL
};

static ssize_t huaqin_show(struct kobject *kobj, struct attribute *a, char *buf)
{
	ssize_t count = 0;
	int i=0,j=0;
	struct hw_info *hw = container_of(a, struct hw_info , attr);

	if(NULL == hw){
		return sprintf(buf, "Data error\n");
	}

	if(HWID_VER == hw->hw_id){
		count = sprintf(buf, "%s\n", HQ_SYS_FS_VER);
	}else if(HWID_SUMMARY == hw->hw_id){
		//iterate all device and output the detail
		int iterator = 0;
		struct hw_info *curent_hw = NULL;
		struct attribute *attr = huaqin_attrs[iterator];

		while(attr){
			curent_hw = container_of(attr, struct hw_info , attr);
			iterator += 1;
			attr = huaqin_attrs[iterator];

			if(curent_hw->hw_exist && (NULL != curent_hw->hw_device_name)){
				count += sprintf(buf+count, "%s: %s\n" ,curent_hw->attr.name,curent_hw->hw_device_name);
			}
		}

	}
	else if(HWID_PCBA == hw->hw_id){
		huaqin_pcba_config=get_huaqin_pcba_config();
		printk(" huaqin_pcba_config:%d \n", huaqin_pcba_config );
		if(huaqin_pcba_config  > PCBA_UNKNOW )
		{
			huaqin_pcba_config=PCBA_UNKNOW;
			printk(" error PCBA_UNKNOW \n " );
		}

		for(i=0;i<sizeof(pcba)/sizeof(struct pcba_info);i++)
		{
			
			if( huaqin_pcba_config == pcba[i].pcba_config )
			{
				// count = sprintf(buf,"%s ",pcba[i].pcba_name);
				//return count;
					huaqin_pcba_state_config=hq_get_hw_state();
					printk(" huaqin_pcba_state_config:%d \n", huaqin_pcba_state_config );
					if(    hq_get_hw_state() >=  PCBA_STATE_UNKNOW  )
					{
						huaqin_pcba_state_config=PCBA_STATE_UNKNOW;
					}

					for(j=0;j<sizeof(pcba_state)/sizeof(struct pcba_info);j++)
					{
						if( huaqin_pcba_state_config == pcba_state[j].pcba_state_config )
						{
							count += sprintf(buf,"%s %s",pcba_state[j].pcba_state_name,pcba[i].pcba_name);
							return count;
						}
					}
			}
		}

		printk(" PCBA_STATE_UNKNOW\n" );
		count = sprintf(buf,"%s\n","PCBA_STATE_UNKNOW");
		return count;
	}
/*
	else if(HWID_BATERY == hw->hw_id){
		count = sprintf(buf,"%s\n",battery_manufacturer_info);
	}
*/
	else if(HWID_MAIN_CAM == hw->hw_id){
		count = sprintf(buf,"%s\n",rcam_name);
	}
	else if(HWID_SUB_CAM == hw->hw_id){
		count = sprintf(buf,"%s\n",fcam_name);
	}
	else if(HWID_EMMC == hw->hw_id){
		hwinfo_emmc *hwinfo_emmc_info = (hwinfo_emmc*)kzalloc(sizeof(hwinfo_emmc),GFP_KERNEL);
		get_hwinfo_emmc_info(hwinfo_emmc_info);
		count = sprintf(buf, "Vendor:%s Model:%s \n",hwinfo_emmc_info->vendor_name,
			hwinfo_emmc_info->emmc_name);
		kfree(hwinfo_emmc_info);
	}else{

		if(0 == hw->hw_exist){
			count = sprintf(buf, "Not support\n");
		}else if(NULL == hw->hw_device_name){
			count = sprintf(buf, "Installed with no device Name\n");
		}else{
			count = sprintf(buf, "%s\n" ,hw->hw_device_name);
		}
	}

	return count;
}

static ssize_t huaqin_store(struct kobject *kobj, struct attribute *a, const char *buf, size_t count)
{
	return count;
}

/* huaqin object */
static struct kobject huaqin_kobj;
static const struct sysfs_ops huaqin_sysfs_ops = {
	.show = huaqin_show,
	.store = huaqin_store,
};

/* huaqin type */
static struct kobj_type huaqin_ktype = {
	.sysfs_ops = &huaqin_sysfs_ops,
	.default_attrs = huaqin_attrs
};

/* huaqin device class */
static struct class  *huaqin_class;
static struct device *huaqin_hw_device;


int register_kboj_under_hqsysfs(struct kobject *kobj, struct kobj_type *ktype, const char *fmt, ...){
	return kobject_init_and_add(kobj, ktype, &(huaqin_hw_device->kobj), fmt);
}

static int __init create_sysfs(void)
{
	int ret;

	/* create class (device model) */
	huaqin_class = class_create(THIS_MODULE, HUAQIN_CLASS_NAME);
	if (IS_ERR(huaqin_class)) {
		pr_err("%s fail to create class\n",__func__);
		return -1;
	}

	huaqin_hw_device = device_create(huaqin_class, NULL, MKDEV(0, 0), NULL, HUAIN_INTERFACE_NAME);
	if (IS_ERR(huaqin_hw_device)) {
		pr_warn("fail to create device\n");
		return -1;
	}

	/* add kobject */
	ret = kobject_init_and_add(&huaqin_kobj, &huaqin_ktype, &(huaqin_hw_device->kobj), HUAQIN_HWID_NAME);
	if (ret < 0) {
		pr_err("%s fail to add kobject\n",__func__);
		return ret;
	}

	/* add wifi/bt/fm/gps */
	hq_regiser_hw_info(HWID_WIFI,"WCN3998");
	hq_regiser_hw_info(HWID_BT,"WCN3998");
	hq_regiser_hw_info(HWID_FM,"WCN3998");
	hq_regiser_hw_info(HWID_GPS,"WCN3998");

	return 0;
}

int hq_deregister_hw_info(enum hardware_id id,char *device_name){
	int ret = 0;
	int find_hw_id = 0;
	int iterator = 0;

	struct hw_info *hw = NULL;
	struct attribute *attr = huaqin_attrs[iterator];

	if(NULL == device_name){
		pr_err("[%s]: device_name does not allow empty\n",__func__);
		ret = -2;
		goto err;
	}

	while(attr){
		hw = container_of(attr, struct hw_info , attr);

		iterator += 1;
		attr = huaqin_attrs[iterator];

		if(NULL == hw){
			continue;
		}

		if(id == hw->hw_id){
			find_hw_id = 1;

			if(0 == hw->hw_exist){
				pr_err("[%s]: device has not registed hw->id:0x%x . Cant be deregistered\n"
					,__func__
					,hw->hw_id);

				ret = -4;
				goto err;
			}else if(NULL == hw->hw_device_name){

				pr_err("[%s]:hw_id is 0x%x Device name cant be NULL\n"
					,__func__
					,hw->hw_id);
				ret = -5;
				goto err;
			}
			else{
				if(0 == strncmp(hw->hw_device_name,device_name,strlen(hw->hw_device_name))){
					hw->hw_device_name = NULL;
					hw->hw_exist = 0;
				}else{
					pr_err("[%s]: hw_id is 0x%x Registered device name %s , want to deregister: %s\n"
						,__func__
						,hw->hw_id
						,hw->hw_device_name
						,device_name);
					ret = -6;
					goto err;
				}
			}

			goto err;

		}else
			continue;

	}

	if(0 == find_hw_id){
		pr_err("[%s]: Cant find correct hardware_id: 0x%x\n",__func__,id);
		ret = -3;
	}

err:
	return ret;

}


int hq_regiser_hw_info(enum hardware_id id,char *device_name){

	int ret = 0;
	int find_hw_id = 0;
	int iterator = 0;

	struct hw_info *hw = NULL;
	struct attribute *attr = huaqin_attrs[iterator];

	if(NULL == device_name){
		pr_err("[%s]: device_name does not allow empty\n",__func__);
		ret = -2;
		goto err;
	}

	while(attr){
		hw = container_of(attr, struct hw_info , attr);

		iterator += 1;
		attr = huaqin_attrs[iterator];

		if(NULL == hw){
			continue;
		}

		if(id == hw->hw_id){
			find_hw_id = 1;

			if(hw->hw_exist){
				pr_err("[%s]: device has already registed hw->id:0x%x hw_device_name:%s\n"
					,__func__
					,hw->hw_id
					,hw->hw_device_name);
				ret = -4;
				goto err;
			}

			switch(hw->hw_id){
			    case HWID_LCM:
			        hw->hw_device_name = device_name;
			        break;
			    case HWID_CTP:
			        hw->hw_device_name = device_name;
			        break;
			    case HWID_CAPSENSOR:
				hw->hw_device_name = device_name;
                                break;
			    case HWID_MAIN_CAM:
			    case HWID_SUB_CAM:
			    case HWID_MAIN_CAM_2:
			    case HWID_SUB_CAM_2:
			        hw->hw_device_name = device_name;
			        break;
				/*
                    if(map_cam_drv_to_vendor(device_name))
                        hw->hw_device_name = map_cam_drv_to_vendor(device_name);
                    else
                        hw->hw_device_name = "Can't find Camera Vendor";
			        break;
				*/
			    default:
			        hw->hw_device_name = device_name;
			        break;
			}


			hw->hw_exist = 1;
			goto err;

		}else
			continue;

	}

	if(0 == find_hw_id){
		pr_err("[%s]: Cant find correct hardware_id: 0x%x\n",__func__,id);
		ret = -3;
	}

err:
	return ret;
}

#include <linux/proc_fs.h>

#define PROC_BOOT_REASON_FILE "boot_status"
static struct proc_dir_entry *boot_reason_proc = NULL;
static unsigned int boot_into_factory = 0;
static int boot_reason_proc_show(struct seq_file *file, void* data)
{
	char temp[40] = {0};

	sprintf(temp, "%d\n", boot_into_factory );
	seq_printf(file, "%s\n", temp);
	return 0;
}

static int boot_reason_proc_open (struct inode* inode, struct file* file)
{
	return single_open(file, boot_reason_proc_show, inode->i_private);
}

static const struct file_operations boot_reason_proc_fops =
{
	.open = boot_reason_proc_open,
	.read = seq_read,
};

//Add node for sdc_det_gpio_status
static struct proc_dir_entry *sdc_det_gpio_status = NULL;

#define SDC_DET_GPIO_STATUS "sdc_det_gpio_status"

static int sdc_det_gpio_proc_show(struct seq_file *file, void* data)
{
	int sdc_detect_status = 0;

	if (gpio_is_valid(SDCARD_DET_N_GPIO)) {
		sdc_detect_status = gpio_get_value(SDCARD_DET_N_GPIO);
		printk("gpio_get_value of sdc_detect_status is %d\n",sdc_detect_status);
	} else {
		printk("gpio of SDC_DET_N_GPIO is not valid\n");
	}

	seq_printf(file, "%d\n", sdc_detect_status);

	return 0;
}

static int sdc_det_gpio_proc_open(struct inode* inode, struct file* file)
{
	return single_open(file, sdc_det_gpio_proc_show, inode->i_private);
}

static const struct file_operations sdc_det_gpio_status_ops =
{
	.open = sdc_det_gpio_proc_open,
	.read = seq_read,
};

static int __init get_boot_rease(char *str)
{
	if (strcmp("boot_with_factory", str) == 0) {
		boot_into_factory = 1;
	}

	return 0;
}
__setup("androidboot.boot_reason=", get_boot_rease);

static int __init hq_harware_init(void)
{
	/* create sysfs entry at /sys/class/huaqin/interface/hw_info */
	create_sysfs();

	sdc_det_gpio_status = proc_create(SDC_DET_GPIO_STATUS, 0644, NULL, &sdc_det_gpio_status_ops);
	if (sdc_det_gpio_status == NULL)
	{
		printk("tpd, create_proc_entry sdc_det_gpio_status_ops failed\n");
	}

	boot_reason_proc = proc_create(PROC_BOOT_REASON_FILE, 0644, NULL, &boot_reason_proc_fops);
	if (boot_reason_proc == NULL)
	{
		pr_err("[%s]: create_proc_entry boot_reason_proc failed\n", __func__);
	}

	return 0;
}

core_initcall(hq_harware_init);
MODULE_AUTHOR("KaKa Ni <nigang@huaqin.com>");
MODULE_DESCRIPTION("Huaqin Hardware Info Driver");
MODULE_LICENSE("GPL");
