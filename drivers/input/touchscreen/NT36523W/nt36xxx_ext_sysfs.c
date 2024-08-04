/*
 * Copyright (C) 2010 - 2018 Novatek, Inc.
 *
 * $Revision$
 * $Date$
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 */

#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/version.h>

#include "nt36xxx.h"

#if NVT_TOUCH_EXT_SYSFS

#define DRIVER_NAME "nova_penraw_driver"
#define DEVICE_NAME "goodix_penraw"
static const unsigned int MINOR_NUMBER_START = 0; /* the minor number starts at */
static const unsigned int NUMBER_MINOR_NUMBER = 1; /* the number of minor numbers */
static unsigned int major_number; /* the major number of the device */
/* ioctl for direct access */
static struct cdev penraw_char_dev; /* character device */
static struct class* penraw_char_dev_class = NULL; /* class object */

extern struct nova_pen_info pen_buffer[NOVA_MAX_BUFFER];
extern unsigned char pen_report_num;
extern unsigned char pen_buffer_wp;


// Linux 2.0/2.2
static int penraw_open(struct inode * inode, struct file * file)
{
	return 0;
}

// Linux 2.1: int type 
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 1, 0)
static int penraw_close(struct inode * inode, struct file * file)
{
	return 0;
}
#else
static void penraw_close(struct inode * inode, struct file * file)
{
}
#endif

#define PENRAW_IOC_TYPE 'P'
#define PENRAW_GET_VALUES _IOR(PENRAW_IOC_TYPE, 0, struct io_pen_report)

static struct io_pen_report pen_report;	// return report
static long penraw_ioctl(struct file *file, 
	      unsigned int cmd, unsigned long arg)
{
	int len = 0;
	unsigned long	flags;
	struct nova_pen_info *ppen_info;
	unsigned char cnt;
	unsigned char pen_buffer_rp;
	unsigned char wp;
	unsigned char num;

	//NVT_LOG("penraw: ioctl: cmd=%04X, arg=%08lX\n",cmd,arg);

	switch(cmd)
	{
		case PENRAW_GET_VALUES:
			local_irq_save(flags);
			wp = pen_buffer_wp;
			num = pen_report_num;
			local_irq_restore(flags);
			if(MAX_IO_CONTROL_REPORT <= num) {
				pen_buffer_rp = (unsigned char)((wp + (NOVA_MAX_BUFFER - MAX_IO_CONTROL_REPORT)) % NOVA_MAX_BUFFER);
			} else {
				pen_buffer_rp = 0;
			}
			memset(&pen_report, 0, sizeof(pen_report));
			pen_report.report_num = num;
			ppen_info = (struct nova_pen_info *)&pen_report.pen_info[0];
			for(cnt = 0; cnt < num; cnt++) {
				memcpy(ppen_info, &pen_buffer[pen_buffer_rp], sizeof(struct nova_pen_info));
				ppen_info++;
				pen_buffer_rp++;
				if(NOVA_MAX_BUFFER == pen_buffer_rp) {
					pen_buffer_rp = 0;
				}
				//NVT_LOG("cnt=%d, state=%d,frame=%d\n",cnt, pen_report.pen_info[cnt].coords.status, pen_report.pen_info[cnt].frame_no);
			}
			
			if (copy_to_user((void __user *)arg, &pen_report, sizeof(pen_report))) {
				return -EFAULT;
			}
			break;
		default:
			NVT_ERR("unsupported command %d\n", cmd);
			return -EFAULT;
	}
	return len;
}

static struct file_operations penraw_fops = {
	.owner = THIS_MODULE,
	.open = penraw_open,
	.release = penraw_close,
	.unlocked_ioctl = penraw_ioctl,
};

int32_t nvt_extra_sysfs_init(void)
{
	int retval = 0;
	int alloc_ret;
	dev_t dev;
	int cdev_err;
	
	/* get not assigned major numbers */
	alloc_ret = alloc_chrdev_region(&dev, MINOR_NUMBER_START, NUMBER_MINOR_NUMBER, DRIVER_NAME);
	if (alloc_ret != 0) {
		NVT_ERR("failed to alloc_chrdev_region()\n");
		return -EINVAL;
	}
	
	/* get one number from the not-assigend numbers */
	major_number = MAJOR(dev);
	
	/* initialize cdev and function table */
	cdev_init(&penraw_char_dev, &penraw_fops);
	penraw_char_dev.owner = THIS_MODULE;
	
	/* register the driver */
	cdev_err = cdev_add(&penraw_char_dev, dev, NUMBER_MINOR_NUMBER);
	if (cdev_err != 0) {
		NVT_ERR("failed to cdev_add()\n");
		unregister_chrdev_region(dev, NUMBER_MINOR_NUMBER);
		return -EINVAL;
	}
	
	/* register a class */
	penraw_char_dev_class = class_create(THIS_MODULE, DEVICE_NAME);
	if (IS_ERR(penraw_char_dev_class)) {
		NVT_ERR("Failed to create class\n");
		cdev_del(&penraw_char_dev);
		unregister_chrdev_region(dev, NUMBER_MINOR_NUMBER);
		return -EINVAL;
	}
	
	/* create "/sys/class/my_device/my_device" */
	device_create(penraw_char_dev_class, NULL, MKDEV(major_number, 0), NULL, DEVICE_NAME);

	return retval;
}

void nvt_extra_sysfs_deinit(void)
{
	dev_t dev = MKDEV(major_number, MINOR_NUMBER_START);

	/* remove "/dev/nova_penraw */
	device_destroy(penraw_char_dev_class, MKDEV(major_number, 0));

	/* remove class */
	class_destroy(penraw_char_dev_class);

	/* remove driver */
	cdev_del(&penraw_char_dev);

	/* release the major number */
	unregister_chrdev_region(dev, NUMBER_MINOR_NUMBER);
	
	return;
}

#endif //NVT_TOUCH_EXT_SYSFS
