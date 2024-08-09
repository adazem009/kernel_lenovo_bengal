#include <linux/module.h>
#include <linux/param.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/idr.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <asm/unaligned.h>

#define DRIVER_VERSION                      "1.0.0"

#define REG_TEMPERATURE                     0x06
#define REG_VOLTAGE                         0x08
#define REG_CURRENT                         0x14
#define REG_RSOC                            0x2c
#define REG_BLOCKDATAOFFSET                 0x3e
#define REG_BLOCKDATA                       0x40


bool has_8013 = false;
struct mm8013_chip {
    struct i2c_client *client;
    struct power_supply_desc battery;
};


struct mm8013_chip *chip;

int battery_id=1;
static int mm8013_write_reg(struct i2c_client *client, u8 reg, u16 value)
{
    int ret = i2c_smbus_write_word_data(client, reg, value);

    if (ret < 0)
        dev_err(&client->dev, "%s: err %d\n", __func__, ret);

    msleep(4);
    return ret;
}

static int mm8013_read_reg(struct i2c_client *client, u8 reg)
{
    int ret = i2c_smbus_read_word_data(client, reg);

    if (ret < 0)
        dev_err(&client->dev, "%s: err %d\n", __func__, ret);

    msleep(4);
    return ret;
}

static int mm8013_soc(struct mm8013_chip *chip, int *val)
{
    int soc = mm8013_read_reg(chip->client, REG_RSOC);

    if (soc < 0) return soc;

    *val = soc;

    return 0;
}

static int mm8013_voltage(struct mm8013_chip *chip, int *val)
{
    int volt = mm8013_read_reg(chip->client, REG_VOLTAGE);

    if (volt < 0) return volt;

    *val = volt;

    return 0;
}

static int mm8013_current(struct mm8013_chip *chip, int *val)
{
    int curr = mm8013_read_reg(chip->client, REG_CURRENT);

    if (curr < 0) return curr;

    if (curr > 32767) {
        curr -= 65536;
    }
    *val = 0 - curr;

    return 0;
}

static int mm8013_temperature(struct mm8013_chip *chip, int *val)
{
    int temp = mm8013_read_reg(chip->client, REG_TEMPERATURE);

    if (temp < 0)
    {
	*val = 2550;

	return 0;
    }
    *val = temp - 2731;

    return 0;
}

static int mm8013_checkdevice(struct mm8013_chip *chip)
{
	int ret;
    	ret = mm8013_write_reg(chip->client, 0x0, 0x0008);
    	if (ret < 0)
	{
		return ret;
	}

    	ret = mm8013_read_reg(chip->client, 0x0);
    	if (ret < 0)
	{
		return ret;
	}

	if( ret== 0x0102)
	{
		battery_id=2;
	}
	else if( ret== 0x0101)
	{
		battery_id=1;
	}
	else
	{
		printk("====unknow_battery==== \r\n");
   		return -1;

	}
	printk("8013 0x0008:[0x%x]  battery_id:%d \r\n",ret,battery_id);

        return 0;
}

static enum power_supply_property mm8013_battery_props[] = {
    POWER_SUPPLY_PROP_batt_cap,
    POWER_SUPPLY_PROP_VOLTAGE_NOW,
    POWER_SUPPLY_PROP_CURRENT_NOW,
    POWER_SUPPLY_PROP_TEMP,
    POWER_SUPPLY_PROP_battery_id,
};



static int mm8013_get_property(struct power_supply *psy,
                    enum power_supply_property psp,
                    union power_supply_propval *val)
{
    int ret  = 0;


    switch (psp) {
    case POWER_SUPPLY_PROP_batt_cap:
	ret = mm8013_soc(chip, &(val->intval));
	break;
    case POWER_SUPPLY_PROP_VOLTAGE_NOW:
	ret = mm8013_voltage(chip, &(val->intval));
	break;
    case POWER_SUPPLY_PROP_CURRENT_NOW:
	ret = mm8013_current(chip, &(val->intval));
	break;
    case POWER_SUPPLY_PROP_TEMP:
	ret = mm8013_temperature(chip, &(val->intval));
	break;
     case POWER_SUPPLY_PROP_battery_id:
	val->intval = battery_id;
	break;
    default:
	return -EINVAL;
    }

    return ret;
}


static const struct power_supply_desc mm8013_desc = {
	.name			= "mm8013",
	.type			= POWER_SUPPLY_TYPE_BATTERY,
	.properties		= mm8013_battery_props,
	.num_properties		= 5,
	.get_property		= mm8013_get_property,
	.set_property		= NULL,
	.property_is_writeable	= NULL,
};




static int mm8013_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{


	struct device *cdev = &client->dev;
	int ret = 0;

	if (!i2c_check_functionality(client->adapter,I2C_FUNC_SMBUS_WORD_DATA)) {
		printk("smbus data not supported!\n");
		return -EIO;
	}

	chip = devm_kzalloc(cdev, sizeof(struct mm8013_chip), GFP_KERNEL);
	if (!chip) {
		printk("can't alloc fusb301_chip\n");
		return -ENOMEM;
	}

	chip->client = client;

	i2c_set_clientdata(client, chip);

	ret = mm8013_checkdevice(chip);
	if (ret)
	{
		dev_err(&client->dev, "================failed to access platform don't has mm8013!\n");
		return -EPROBE_DEFER;;
	}
        power_supply_register(&client->dev, &mm8013_desc,NULL);
	printk("mm8013-init down platform has mm8013!\n");


	return 0;
}

static const struct i2c_device_id mm8013_id_table[] = {
	{"mm8013", 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, mm8013_id_table);

static struct of_device_id mm8013_match_table[] = {
	{ .compatible = "qcom,mm8013",},
	{ },
};

static struct i2c_driver mm8013_i2c_driver = {
	.driver = {
		.name = "mm8013",
		.owner = THIS_MODULE,
		.of_match_table = mm8013_match_table,
	},
	.probe = mm8013_probe,
	.id_table = mm8013_id_table,
};
static __init int mm8013_i2c_init(void)
{
	return i2c_add_driver(&mm8013_i2c_driver);
}
static __exit void mm8013_i2c_exit(void)
{
	i2c_del_driver(&mm8013_i2c_driver);
}
fs_initcall(mm8013_i2c_init);
module_exit(mm8013_i2c_exit);
MODULE_AUTHOR("leon@lge.com");
MODULE_DESCRIPTION("I2C bus driver for mm8013X gauge");
MODULE_LICENSE("GPL v2");
