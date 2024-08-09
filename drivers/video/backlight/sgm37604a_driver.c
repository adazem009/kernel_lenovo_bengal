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

#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/types.h>
#include <linux/platform_device.h>
#include <linux/leds.h>
#include <linux/backlight.h>
#if defined(CONFIG_BACKLIGHT_SGM37604)
#include <linux/sgm37604a.h>
#endif
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#define SGM37604A_DEV_NAME "sgm37604a"

#define BACKLIGHT_TAG                  "[SGM37604A] "
#define BACKLIGHT_FUN(f)               pr_debug("[SGM37604A] %s\n", __func__)
#define BACKLIGHT_ERR(fmt, args...)    pr_debug("[SGM37604A] %s %d : "fmt, __func__, __LINE__, ##args)
#define BACKLIGHT_LOG(fmt, args...)    pr_debug(fmt, ##args)

#define SGM37604A_MIN_VALUE_SETTINGS 16		/* value min leds_brightness_set */
#define SGM37604A_MAX_VALUE_SETTINGS 4095	/* value max leds_brightness_set */
#define MIN_MAX_SCALE(x) (((x) < SGM37604A_MIN_VALUE_SETTINGS) ? SGM37604A_MIN_VALUE_SETTINGS :\
(((x) > SGM37604A_MAX_VALUE_SETTINGS) ? SGM37604A_MAX_VALUE_SETTINGS:(x)))

struct sgm37604a_chip_data {
	struct device *dev;
	//struct backlight_device *bled;
	struct i2c_client *client;
	int en_gpio;
};

struct sgm37604a_chip_data *pchip;

/* static unsigned char current_brightness; */
static unsigned char is_suspend;

struct semaphore SGM37604A_lock;

/* SGM37604A Register Map */
#define MAX_BRIGHTNESS			(4095)
#define MID_BRIGHTNESS          (16)

#define SGM37604A_CTL_BRIGHTNESS_LSB_REG			0x1A
#define SGM37604A_CTL_BRIGHTNESS_MSB_REG			0x19
#define SGM37604A_CTL_BACKLIGHT_MODE_REG                       	0x11
#define SGM37604A_CTL_BACKLIGHT_LED_REG                        	0x10
#define SGM37604A_CTL_BACKLIGHT_CURRENT_REG                     0x1B
/*
//backlight i2c control type
unsigned char backlight_mode_reg = 0x15;
unsigned char backlight_led_reg = 0x1f;
unsigned char backlight_current_reg = 0x00;
*/
//backlight LCD PWM control type
unsigned char backlight_mode_reg = 0x65;
unsigned char backlight_led_reg = 0x07;
unsigned char backlight_current_reg = 0xD7;
static char SGM37604A_i2c_write(struct i2c_client *client, u8 reg_addr, u8 *data, u8 len)
{
	s32 dummy;

	if (client == NULL)
		return -1;

	while (len-- != 0) {
		dummy = i2c_smbus_write_byte_data(client, reg_addr, *data);

		reg_addr++;
		data++;
		if (dummy < 0)
			return -1;

	}

	return 0;
}

static int SGM37604A_smbus_write_byte(struct i2c_client *client,
				   unsigned char reg_addr, unsigned char *data)
{
	int ret_val = 0;
	int i = 0;

	ret_val = SGM37604A_i2c_write(client, reg_addr, data, 1);

	for (i = 0; i < 5; i++) {
		if (ret_val != 0)
			SGM37604A_i2c_write(client, reg_addr, data, 1);
		else
			return ret_val;
	}

	return ret_val;
}

unsigned int SGM37604A_set_brightness_level(unsigned int level_a)
{
	int ret_code = 0;
	unsigned char data0,data1;

	data0 = (level_a & 0x0F);
	data1 = (level_a >> 4);

	/* Control A Brightness LSB & MSB */
	ret_code = SGM37604A_smbus_write_byte(pchip->client, SGM37604A_CTL_BRIGHTNESS_LSB_REG, &data0);
	if (ret_code != 0) {
		BACKLIGHT_ERR("[Cust_SetBacklight] CTL_LSB fail: %d\n", ret_code);
		return ret_code;
	}

	ret_code = SGM37604A_smbus_write_byte(pchip->client, SGM37604A_CTL_BRIGHTNESS_MSB_REG, &data1);
	if (ret_code != 0) {
		BACKLIGHT_ERR("[Cust_SetBacklight] CTL_MSB fail: %d\n", ret_code);
		return ret_code;
	}

	return 0;
}

void SGM37604A_set_backlight_reg_init(void)
{
	int ret_code = 0;
	unsigned char data0, data1, data2;

	data0 =	backlight_mode_reg;
	data1 = backlight_led_reg;
	data2 = backlight_current_reg;

	gpio_set_value(pchip->en_gpio,1);

	ret_code = SGM37604A_smbus_write_byte(pchip->client, SGM37604A_CTL_BACKLIGHT_MODE_REG, &data0);
	if (ret_code != 0) {
		printk("[Cust_SetBacklight] CTL_MODE fail: %d\n", ret_code);
	}
	BACKLIGHT_LOG("[SGM37604A] set backlight mode reg = 0x%x \n", data0);

	//backlight i2c control type
	//ret_code = SGM37604A_smbus_write_byte(pchip->client, SGM37604A_CTL_BACKLIGHT_LED_REG, &data1);
	//backlight LCD PWM control type
	ret_code = SGM37604A_smbus_write_byte(pchip->client, SGM37604A_CTL_BRIGHTNESS_LSB_REG, &data1);
	if (ret_code != 0) {
		printk("[Cust_SetBacklight] CTL_LED fail: %d\n", ret_code);
	}
	BACKLIGHT_LOG("[SGM37604A] set backlight led reg = 0x%x \n", data1);

	//backlight i2c control type
	//ret_code = SGM37604A_smbus_write_byte(pchip->client, SGM37604A_CTL_BACKLIGHT_CURRENT_REG, &data2);
	//backlight LCD PWM control type
	ret_code = SGM37604A_smbus_write_byte(pchip->client, SGM37604A_CTL_BRIGHTNESS_MSB_REG, &data2);
	if (ret_code != 0) {
		printk("[Cust_SetBacklight] CTL_CURRENT fail: %d\n", ret_code);
	}
	BACKLIGHT_LOG("[SGM37604A] set backlight current reg = 0x%x \n", data2);
}


int sgm_last_level;
int sgm37604a_set_backlight_level(unsigned int level)
{
	unsigned int level_a = 0;
	unsigned int ret_code = 0;

	BACKLIGHT_LOG("[SGM37604A] sgm37604a_set_backlight_level  [%d]\n", level);
	sgm_last_level = level;

	if (level == 0) {
		ret_code = down_interruptible(&SGM37604A_lock);

		level_a = 0;
		SGM37604A_set_brightness_level(level_a);
		up(&SGM37604A_lock);

		is_suspend = 1;
	} else {

		level_a = MIN_MAX_SCALE(level);

		if (is_suspend == 1) {
			is_suspend = 0;

			mdelay(10);
			ret_code = down_interruptible(&SGM37604A_lock);

			BACKLIGHT_LOG("[Brightness] level_a=%d\n", level_a);

			up(&SGM37604A_lock);
		}

		ret_code = down_interruptible(&SGM37604A_lock);

		ret_code = SGM37604A_set_brightness_level(level_a);
		if (ret_code != 0)
			BACKLIGHT_LOG("[Brightness] setting fail ----------\n");

		up(&SGM37604A_lock);

	}

	return 0;
}


#if 0
static unsigned int GPIO_LCM_BL_ENABLE;
static void backlight_get_dts_info(void)
{
	static struct device_node *node = NULL;

	node = of_find_compatible_node(NULL, NULL, "mediatek,sgm37604a");
	GPIO_LCM_BL_ENABLE = of_get_named_gpio(node, "lcm_bl_gpio_enable", 0);
	gpio_request(GPIO_LCM_BL_ENABLE, "lcm_bl_gpio_enable");
}

static void set_gpio_output(unsigned int GPIO, unsigned int output)
{
	gpio_direction_output(GPIO, output);
	gpio_set_value(GPIO, output);
}

void BACKLIGHT_GPIO_enable(void)
{
	set_gpio_output(GPIO_LCM_BL_ENABLE, 1);
}

void BACKLIGHT_GPIO_disable(void)
{
	set_gpio_output(GPIO_LCM_BL_ENABLE, 0);
}
#endif

/* update and get brightness 
static int sgm37604a_bled_update_status(struct backlight_device *bl)
{

	//sgm37604a_set_backlight_level(bl->props.brightness);

	return 0;
}*/

/*static int sgm37604a_bled_get_brightness(struct backlight_device *bl)
{
	printk("================leon===========bl->props.brightness[%d]======================\r\n",bl->props.brightness);


	return bl->props.brightness;
}*/

#if defined(CONFIG_BACKLIGHT_SGM37604)
void sgm37604a_bled_enable(void)
{
	BACKLIGHT_LOG("[SGM37604A] %s \n", __func__);

	SGM37604A_set_backlight_reg_init();
}

void sgm37604a_bled_disable(void)
{
	unsigned char data1 = 0;
	BACKLIGHT_LOG("[SGM37604A] %s \n", __func__);

	SGM37604A_smbus_write_byte(pchip->client, SGM37604A_CTL_BRIGHTNESS_LSB_REG, &data1);
	msleep(2);
	gpio_set_value(pchip->en_gpio,0);
}
#endif

/*
static int sgm37604a_bled_init(struct backlight_device *bl,int value)
{
	if(value == 1)
	{
		SGM37604A_set_backlight_reg_init();
	}
	else
	{
		msleep(2);
		gpio_set_value(pchip->en_gpio,0);
	}
	return 0;
}*/

/*
static int bias_bled_switch(struct backlight_device *bl,int value)
{
	//tps65132_switch(value);
	return 0;
}*/


/*static int sgm37604a_bled_write_cmd(struct backlight_device *bl,unsigned char addr, unsigned char value)
{
	printk("================leon===========addr[%02x]===========value[%02x]==============\r\n",addr,value);
	tps65132_write_bytes(addr,value);

	return 0;
}*/

/*
static const struct backlight_ops sgm37604a_bled_ops = {
	.options = BL_CORE_SUSPENDRESUME,
	.update_status = sgm37604a_bled_update_status,
	//.get_brightness = sgm37604a_bled_get_brightness,
	.init = sgm37604a_bled_init,
	//.write_cmd = sgm37604a_bled_write_cmd,
	.bias_switch = bias_bled_switch,
};*/



static int SGM37604A_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	//struct backlight_properties props;
	struct device* dev = &client->dev;
	struct device_node* np;
	int rc;

	np = dev->of_node;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "i2c functionality check fail.\n");
		return -EOPNOTSUPP;
	}

	pchip = devm_kzalloc(&client->dev,sizeof(struct sgm37604a_chip_data), GFP_KERNEL);
	if (!pchip)
	{
		return -ENOMEM;
	}

	pchip->client = client;

	pchip->en_gpio = of_get_named_gpio(np,"qcom,platform-bklight-en-gpio",0);

	if (gpio_is_valid(pchip->en_gpio)) {
		rc = gpio_request(pchip->en_gpio, "bklight-en-gpio");
		if(rc)
		{
			printk("request for reset_gpio failed, rc=%d\n", rc);
		}
		else
		{
			rc = gpio_direction_output(pchip->en_gpio,1);
		}
	}

	sema_init(&SGM37604A_lock, 1);

	/*SGM37604A register config init*/
	SGM37604A_set_backlight_reg_init();
	//backlight_get_dts_info();

	pchip->dev = &client->dev;

/*
	props.type = BACKLIGHT_EXTERNAL;
	props.brightness = 2000;
	props.max_brightness = 4095;

	pchip->bled =
	    devm_backlight_device_register(pchip->dev, "sgm37604a",
					   pchip->dev, pchip, &sgm37604a_bled_ops,
					   &props);
	pchip->bled->touch_status = 0;
*/

	return 0;
}


static int SGM37604A_remove(struct i2c_client *client)
{

	return 0;
}


static int __attribute__ ((unused)) SGM37604A_detect(struct i2c_client *client, int kind,
						  struct i2c_board_info *info)
{
	return 0;
}

static struct of_device_id sgm37604a_of_match[] = {
	{.compatible = "sgm,sgm37604a"},
	{},
};
MODULE_DEVICE_TABLE(of,sgm37604a_of_match);

static const struct i2c_device_id sgm37604a_i2c_id[] = {
	{ "sgm37604a", 0 },
	{},
};
MODULE_DEVICE_TABLE(i2c, sgm37604a_i2c_id);

static struct i2c_driver SGM37604A_i2c_driver = {
	.driver = {
 			.owner  = THIS_MODULE,
		   	.name = SGM37604A_DEV_NAME,
#ifdef CONFIG_OF
		   	.of_match_table = sgm37604a_of_match,
#endif
		   },

	.id_table = sgm37604a_i2c_id,

	.probe = SGM37604A_probe,
	.remove = SGM37604A_remove,
};

module_i2c_driver(SGM37604A_i2c_driver);

MODULE_AUTHOR("YeQiang Li <liyeqiang.com>");
MODULE_DESCRIPTION("SGM37604A Driver");
MODULE_LICENSE("GPL");
