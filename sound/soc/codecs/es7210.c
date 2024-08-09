/*
 * es7210_adc.c
 *
 * Copyright (C) 2014 Everest Semiconductor Co., Ltd
 *
 * Authors:  yangxiaohua(yangxiaohua@everest-semi.com)
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#define DEBUG
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/slab.h>
//#include <linux/regmap.h>
#include <linux/stddef.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/tlv.h>
#include <sound/soc.h>
#include <sound/initval.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/mutex.h>
#include <linux/types.h>
#include <linux/regulator/consumer.h>

struct es7210_snd_private {
	struct i2c_client	*client;
	struct regulator *vregulator;

};
struct es7210_snd_private *es7210_private;



static int es7210_i2c_write(struct i2c_client *client, u8 addr, u8 value)
{


	struct i2c_msg msgs[2];
	u8 msg_count = 1;
	int rc;
	u8 buffer[3];

	buffer[0] = addr;
	buffer[1] = value;
	msgs[0].addr = client->addr;
	msgs[0].flags = client->flags & I2C_M_TEN;
	msgs[0].len = 2;
	msgs[0].buf = buffer;
//	msgs[0].scl_rate = 300000;

	rc = i2c_transfer(client->adapter, msgs, msg_count);

	if (rc < 0 || rc != msg_count){

		printk("%s, es7210_i2c_write error [0x%02x] = 0x%02x\n", __func__, addr, value);

		return (rc < 0) ? rc : -EIO;
	}


	return 0;

}


/***************************************************************************************************************/
/***************************************************************************************************************/

static int es7210_i2c_read(struct i2c_client *client, u8 addr, u8* data)
{
	struct i2c_msg msgs[2];
	u8 msg_count = 1;
	int rc;
	u8 buffer[3];

	printk("%s, [0x%02x\n", __func__, addr);


	buffer[0] = addr;

	msgs[0].addr = client->addr;
	msgs[0].flags = client->flags & I2C_M_TEN;
	msgs[0].len = 1;
	msgs[0].buf = buffer;
//	msgs[0].scl_rate = 300000;

	rc = i2c_transfer(client->adapter, msgs, msg_count);

	if (rc < 0 || rc != msg_count)
		return (rc < 0) ? rc : -EIO;


	//read
	memset(buffer, 0, 3);
	rc = i2c_master_recv(client, buffer, 1);

	*data = buffer[0];
	printk("%s, i2c_master_recv data = 0x%02x\n", __func__, buffer[0]);
	return (rc < 0) ? rc : rc != (int)1 ? -EIO : 0;


}
/*
*	ES7210 resume from sleep mode
*/
static int es7210_resume(struct device *dev)
{

	struct i2c_client *i2c_client = es7210_private->client;
	int ret = 0;

	if(i2c_client == NULL)
		return -EINVAL;

	ret = regulator_enable(es7210_private->vregulator);
	if (ret != 0) {
		printk( "%s: Failed to enable regulator: %d\n", __func__, ret);
	}


	es7210_i2c_write(i2c_client, 0x14, 0x03);
	es7210_i2c_write(i2c_client, 0x15, 0x03);
	es7210_i2c_write(i2c_client, 0x06, 0x00);
	es7210_i2c_write(i2c_client, 0x01, 0x20);
	es7210_i2c_write(i2c_client, 0x40, 0x42);
	es7210_i2c_write(i2c_client, 0x0b, 0x02);
	es7210_i2c_write(i2c_client, 0x4b, 0x00);
	es7210_i2c_write(i2c_client, 0x4c, 0x00);
	es7210_i2c_write(i2c_client, 0x14, 0x00);
	es7210_i2c_write(i2c_client, 0x15, 0x00);
	return 0;
}
/*
* ES7210 Enter into sleep mode
* Note:
*      Before ES7210 is ready to enter into sleep mode, the I2S MCLK and LRCK must be active.
*      After es7210_standby() performed, I2S MCLK and LRCK can be stop to minimize power consumption
*/
static int es7210_suspend(struct device *dev)
{
	struct i2c_client *i2c_client = es7210_private->client;
	int ret = 0;

	if(i2c_client == NULL)
		return -EINVAL;
	/*
	* I2S MCLK and LRCK must be active here
	*/
	es7210_i2c_write(i2c_client, 0x14, 0x03);
	es7210_i2c_write(i2c_client, 0x15, 0x03);
	es7210_i2c_write(i2c_client, 0x06, 0x00);
	es7210_i2c_write(i2c_client, 0x4b, 0xff);
	es7210_i2c_write(i2c_client, 0x4c, 0xff);
	es7210_i2c_write(i2c_client, 0x0b, 0xd0);
	es7210_i2c_write(i2c_client, 0x40, 0x80);
	es7210_i2c_write(i2c_client, 0x01, 0x7f);
	es7210_i2c_write(i2c_client, 0x06, 0x07);
  /*
  * Now, you can stop I2S MCLK and LRCK here
  */
	ret = regulator_disable(es7210_private->vregulator);
	if (ret != 0) {
		printk( "%s: Failed to enable regulator: %d\n", __func__, ret);
	}
	return 0;
}

static ssize_t es7210_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	printk("echo flag|reg|val > es7210\n");
	printk("eg read star addres=0x06,count 0x10:echo 0610 >es7210\n");
	printk("eg write star addres=0x90,value=0x3c,count=4:echo 4903c >es7210\n");

	return 0;
}

static ssize_t es7210_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int val=0, flag=0;
	u8 i=0, reg, num, value_w, value_r;

	struct es7210_snd_private *es7210 = dev_get_drvdata(dev);
	val = simple_strtol(buf, NULL, 16);
	flag = (val >> 16) & 0xFF;

	if (flag) {
		reg = (val >> 8) & 0xFF;
		value_w = val & 0xFF;
		printk("\nWrite: start REG:0x%02x,val:0x%02x,count:0x%02x\n", reg, value_w, flag);
		while(flag--) {
			es7210_i2c_write(es7210->client, reg, value_w);
			printk("Write 0x%02x to REG:0x%02x\n", value_w, reg);
			reg++;
		}
	} else {
		reg = (val >> 8) & 0xFF;
		num = val & 0xff;
		printk("\nRead: start REG:0x%02x,count:0x%02x\n", reg, num);
		do {
			value_r = 0;
			es7210_i2c_read(es7210->client, reg, &value_r);
			printk("REG[0x%02x]: 0x%02x;  ", reg, value_r);
			reg++;
			i++;
			if ((i==num) || (i%4==0))	printk("\n");
		} while (i<num);
	}
	return count;
}

DEVICE_ATTR(es7210, 0644, es7210_show, es7210_store);

static struct attribute *es7210_debug_attrs[] = {
	&dev_attr_es7210.attr,
	NULL,
};

static struct attribute_group es7210_debug_attr_group = {
	.name   = "es7210_debug",
	.attrs  = es7210_debug_attrs,
};

static int es7210_i2c_probe(struct i2c_client *i2c_client,
			    const struct i2c_device_id *id)
{
	int ret;

	printk(KERN_ERR"%s %d\n", __func__, __LINE__);
	es7210_private = devm_kzalloc(&i2c_client->dev, sizeof(struct es7210_snd_private),
			      GFP_KERNEL);
	if (!es7210_private) {
		dev_err(&i2c_client->dev, "could not allocate codec\n");

		return -ENOMEM;
	}

	es7210_private->vregulator = devm_regulator_get(&i2c_client->dev, "vcc");
	if (IS_ERR(es7210_private->vregulator)) {
		printk( "%s: Can't get voltage regulator\n",
			__func__);
	}

	ret = regulator_enable(es7210_private->vregulator);
	if (ret != 0) {
		printk( "%s: Failed to enable regulator: %d\n", __func__, ret);
	}

	i2c_set_clientdata(i2c_client, es7210_private);
	es7210_private->client = i2c_client;
		/* initialize es7210 */
  es7210_i2c_write(i2c_client, 0x00, 0xFF);
  msleep(1);
  es7210_i2c_write(i2c_client, 0x00, 0x32);
  es7210_i2c_write(i2c_client, 0x0D, 0x09);
  es7210_i2c_write(i2c_client, 0x09, 0x20);
  es7210_i2c_write(i2c_client, 0x0A, 0x20);
  es7210_i2c_write(i2c_client, 0x23, 0x2a);
  es7210_i2c_write(i2c_client, 0x22, 0x0a);
  es7210_i2c_write(i2c_client, 0x21, 0x2a);
  es7210_i2c_write(i2c_client, 0x20, 0x0a);
  es7210_i2c_write(i2c_client, 0x08, 0x14);
  es7210_i2c_write(i2c_client, 0x11, 0x63); //I2S 32bits
  es7210_i2c_write(i2c_client, 0x12, 0x01); //Normal mode
  es7210_i2c_write(i2c_client, 0x40, 0xC3);
  es7210_i2c_write(i2c_client, 0x41, 0x70);
  es7210_i2c_write(i2c_client, 0x42, 0x70);
  es7210_i2c_write(i2c_client, 0x1b, 0xb9); //+3db pga gain
  es7210_i2c_write(i2c_client, 0x1c, 0xb9); //+3db pga gain
  es7210_i2c_write(i2c_client, 0x1d, 0xb9); //+3db pga gain
  es7210_i2c_write(i2c_client, 0x1e, 0xb9); //+3db pga gain
  es7210_i2c_write(i2c_client, 0x43, 0x10); //+3db pga gain
  es7210_i2c_write(i2c_client, 0x44, 0x10); //+3db pga gain
  es7210_i2c_write(i2c_client, 0x45, 0x10); //+3db pga gain
  es7210_i2c_write(i2c_client, 0x46, 0x10); //+3db pga gain
  es7210_i2c_write(i2c_client, 0x47, 0x08);
  es7210_i2c_write(i2c_client, 0x48, 0x08);
  es7210_i2c_write(i2c_client, 0x49, 0x08);
  es7210_i2c_write(i2c_client, 0x4A, 0x08);
  es7210_i2c_write(i2c_client, 0x07, 0x20);
  es7210_i2c_write(i2c_client, 0x02, 0x41); //SCLK/LRCK=64，CLK_MULT_FACTOR = x8
  es7210_i2c_write(i2c_client, 0x06, 0x00);
  es7210_i2c_write(i2c_client, 0x4B, 0x0F);
  es7210_i2c_write(i2c_client, 0x4C, 0x0F);
  es7210_i2c_write(i2c_client, 0x00, 0x71);
  es7210_i2c_write(i2c_client, 0x00, 0x41);

	//debug
	ret = sysfs_create_group(&i2c_client->dev.kobj, &es7210_debug_attr_group);
	if (ret) {
		pr_err("failed to create attr group\n");
	}
	return ret;
}


void es7210_start(void)
{
    struct i2c_client *i2c_client = es7210_private->client;

	if(i2c_client == NULL)
		pr_err("%s i2c_client err\n", __func__);

	es7210_i2c_write(i2c_client, 0x14, 0x03);
	es7210_i2c_write(i2c_client, 0x15, 0x03);
	es7210_i2c_write(i2c_client, 0x06, 0x00);
	es7210_i2c_write(i2c_client, 0x01, 0x20);
	es7210_i2c_write(i2c_client, 0x40, 0x42);
	es7210_i2c_write(i2c_client, 0x0b, 0x02);
	es7210_i2c_write(i2c_client, 0x4b, 0x00);
	es7210_i2c_write(i2c_client, 0x4c, 0x00);
	es7210_i2c_write(i2c_client, 0x14, 0x00);
	es7210_i2c_write(i2c_client, 0x15, 0x00);

	pr_err("%s\n", __func__);
}
EXPORT_SYMBOL_GPL(es7210_start);

void es7210_stop(void)
{
    struct i2c_client *i2c_client = es7210_private->client;

    if(i2c_client == NULL)
		pr_err("%s i2c_client err\n", __func__);

	/*
	*     * I2S MCLK and LRCK must be active here
	*         */
	es7210_i2c_write(i2c_client, 0x14, 0x03);
	es7210_i2c_write(i2c_client, 0x15, 0x03);
	es7210_i2c_write(i2c_client, 0x06, 0x00);
	es7210_i2c_write(i2c_client, 0x4b, 0xff);
	es7210_i2c_write(i2c_client, 0x4c, 0xff);
	es7210_i2c_write(i2c_client, 0x0b, 0xd0);
	es7210_i2c_write(i2c_client, 0x40, 0x80);
	es7210_i2c_write(i2c_client, 0x01, 0x7f);
	es7210_i2c_write(i2c_client, 0x06, 0x07);
    /*
	 *   * Now, you can stop I2S MCLK and LRCK here
	 *     */
	pr_err("%s\n", __func__);
}
EXPORT_SYMBOL_GPL(es7210_stop);


/***************************************************************************************************************/
/***************************************************************************************************************/
static void es7210_i2c_shutdown(struct i2c_client *client)
{


}


/***************************************************************************************************************/
/***************************************************************************************************************/
static int es7210_i2c_remove(struct i2c_client *client)
{
	i2c_set_clientdata(client, NULL);
	return 0;
}

static const struct i2c_device_id es7210_id[] = {
	{"es7210", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, es7210_id);

static const struct dev_pm_ops es7210_pm_ops = {
	.suspend = es7210_suspend,
	.resume = es7210_resume,
};

static struct i2c_driver es7210_i2c_driver = {
	.driver = {
		   .name = "es7210",
		   .owner = THIS_MODULE,
		   .pm = &es7210_pm_ops,
		   },
	.id_table = es7210_id,
	.probe = es7210_i2c_probe,
	.shutdown = es7210_i2c_shutdown,
	.remove = es7210_i2c_remove,
};




/***************************************************************************************************************/
/***************************************************************************************************************/
static int __init es7210_init(void)
{
	return i2c_add_driver(&es7210_i2c_driver);
}

static void __exit es7210_exit(void)
{
	i2c_del_driver(&es7210_i2c_driver);
}

module_init(es7210_init);
module_exit(es7210_exit);

MODULE_DESCRIPTION("ASoC ES7210 ADC driver");
MODULE_AUTHOR("yangxiaohua, <yangxiaohua@everest-semi.com>");
MODULE_LICENSE("GPL");
