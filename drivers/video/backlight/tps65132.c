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

#include <linux/string.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/irq.h>
/* #include <linux/jiffies.h> */
/* #include <linux/delay.h> */
#include <linux/uaccess.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/backlight.h>
#if defined(CONFIG_LCD_BIAS_TPS65132)
#include <linux/tps65132.h>
#endif
#include <linux/gpio.h>
#include <linux/of_gpio.h>

#define LOG_TAG "LCM_BIAS"
#define LCM_LOGI(fmt, args...)  pr_info("[KERNEL/"LOG_TAG"]"fmt, ##args)
#define LCM_LOGD(fmt, args...)  pr_debug("[KERNEL/"LOG_TAG"]"fmt, ##args)


/*****************************************************************************
 * Define
 *****************************************************************************/

#define I2C_ID_NAME "tps65132"
static unsigned int GPIO_LCD_BIAS_ENN;
static unsigned int GPIO_LCD_BIAS_ENP;

/*****************************************************************************
 * GLobal Variable
 *****************************************************************************/

static struct i2c_client *tps65132_i2c_client;

/*****************************************************************************
 * Function Prototype
 *****************************************************************************/
static int tps65132_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int tps65132_remove(struct i2c_client *client);
/*****************************************************************************
 * Data Structure
 *****************************************************************************/

struct tps65132_dev {
	struct i2c_client *client;

};

static const struct i2c_device_id tps65132_id[] = {
	{I2C_ID_NAME, 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, tps65132_id);

static struct of_device_id TPS65132_dt_match[] = {
	{ .compatible = "tps,i2c_lcd_bias" },
	{ },
};

static struct i2c_driver tps65132_iic_driver = {
	.id_table = tps65132_id,
	.probe = tps65132_probe,
	.remove = tps65132_remove,
	/* .detect               = mt6605_detect, */
	.driver = {
		.owner = THIS_MODULE,
		.name = "tps65132",
		.of_match_table = of_match_ptr(TPS65132_dt_match),
	},
};

/*****************************************************************************
 * Function
 *****************************************************************************/

 int tps65132_write_bytes(unsigned char addr, unsigned char value)
{
	int ret = 0;
	struct i2c_client *client = tps65132_i2c_client;
	char write_data[2] = { 0 };

	write_data[0] = addr;
	write_data[1] = value;
	ret = i2c_master_send(client, write_data, 2);
	if (ret < 0)
		LCM_LOGI("tps65132 write data fail !!\n");
	return ret;
}

int tps65132_read_bytes(unsigned char reg, unsigned char *val)
{
    int ret = -EINVAL, retry_times = 0;
    struct i2c_client *client = tps65132_i2c_client;

    do{
        ret = i2c_smbus_read_byte_data(client, reg);
        retry_times ++;
        if(retry_times == 5)
            break;
    }while (ret < 0);

    if (ret < 0)
        return ret;

    *val = ret;
    return 0;
}

extern int Himax_gesture_status(void);

#if defined(CONFIG_LCD_BIAS_TPS65132)
void tps65132_enable(void)
{
	LCM_LOGD("tps65132_enable\n");

	gpio_set_value(GPIO_LCD_BIAS_ENP, 1);
	tps65132_write_bytes(0x00, 0x11);
	msleep(5);
	gpio_set_value(GPIO_LCD_BIAS_ENN, 1);
	tps65132_write_bytes(0x01, 0x11);

}

void tps65132_disable(void)
{
	int ret = 0;

	LCM_LOGD("tps65132_disable\n");

	ret = Himax_gesture_status();
	if (ret != 1) {
		gpio_set_value(GPIO_LCD_BIAS_ENN, 0);
		msleep(2);
		gpio_set_value(GPIO_LCD_BIAS_ENP, 0);
	}
}
#endif

static int tps65132_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct device* dev = &client->dev;
	struct device_node* node;
	int rc;
	unsigned char val1;
	unsigned char val2;

	node = dev->of_node;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "i2c functionality check fail.\n");
		return -EOPNOTSUPP;
	}

	LCM_LOGI("tps65132_iic_probe\n");
	LCM_LOGI("TPS: info==>name=%s addr=0x%x\n", client->name, client->addr);
	tps65132_i2c_client = client;
	//bias_get_dts_info();

	GPIO_LCD_BIAS_ENN = of_get_named_gpio(node, "qcom,lcd-bisa-enn-gpio", 0);
	GPIO_LCD_BIAS_ENP = of_get_named_gpio(node, "qcom,lcd-bisa-enp-gpio", 0);

	rc = gpio_request(GPIO_LCD_BIAS_ENP, "bias_gpio_enp");
	gpio_direction_output(GPIO_LCD_BIAS_ENP, 1);
	msleep(5);
	rc = gpio_request(GPIO_LCD_BIAS_ENN, "bias_gpio_enn");
	gpio_direction_output(GPIO_LCD_BIAS_ENN, 1);
	msleep(1);

	tps65132_read_bytes(0x03, &val1);
	printk("BIAS ADDR 0x03 = %0x",val1);
	tps65132_read_bytes(0x00, &val2);
	printk("BIAS ADDR 0x00 = %0x",val2);

	if(val2 != 0x11) {
		tps65132_write_bytes(0x00, 0x11);
		tps65132_write_bytes(0x01, 0x11);

		if(val1 == 0x33)
			tps65132_write_bytes(0xFF, 0x80);

		if(val1 == 0x43) {
			tps65132_write_bytes(0x02, 0x82);
			tps65132_write_bytes(0x03, 0x5B);
			tps65132_write_bytes(0xFF, 0x80);
		}
	}

	return 0;
}

static int tps65132_remove(struct i2c_client *client)
{
	LCM_LOGI("tps65132_remove\n");
	tps65132_i2c_client = NULL;
	i2c_unregister_device(client);
	return 0;
}


static int __init tps65132_iic_init(void)
{
	int ret = 0;

	LCM_LOGI("tps65132_iic_init\n");
	ret = i2c_add_driver(&tps65132_iic_driver);
	if (ret < 0)
	{
		LCM_LOGI("tps65132 i2c driver add fail !!\n");
		return ret ;
	}

	LCM_LOGI("tps65132_iic_init success\n");
	return 0;
}

static void __exit tps65132_iic_exit(void)
{
	LCM_LOGI("tps65132_iic_exit\n");
	i2c_del_driver(&tps65132_iic_driver);
}

module_init(tps65132_iic_init);
module_exit(tps65132_iic_exit);
MODULE_DESCRIPTION("tps65132 I2C Driver");
MODULE_AUTHOR("TS");
MODULE_LICENSE("GPL");
