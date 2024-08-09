#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>

#include <linux/slab.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <asm/uaccess.h>
#include <linux/types.h>

#include <linux/miscdevice.h>

#include <linux/of.h>
#include <linux/device.h>
#include <linux/string.h>

#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>

#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/of_irq.h>


int hq_hw_id=0;
static int hwid_probe(struct platform_device *pdev)
{
	int ret = -1;
	struct device* dev = &pdev->dev;
	struct device_node* np;
	int pin_id0,pin_id1,pin_id2;

	np = dev->of_node;
	pin_id0 = of_get_named_gpio(np, "hq,pin_id1", 0);
	pin_id1 = of_get_named_gpio(np, "hq,pin_id2", 0);
	pin_id2 = of_get_named_gpio(np, "hq,pin_id3", 0);

	ret=gpio_request(pin_id0, "hq,pin_id0" );
	if(ret < 0)
	{
		printk("gpio_request  pin_id0  fail  \n");
		goto gpio_id0_fail;
	}

	ret=gpio_request(pin_id1, "hq,pin_id1" );
	if(ret < 0)
	{
		printk("gpio_request  pin_id1  fail  \n");
		goto gpio_id1_fail;
	}

	ret=gpio_request(pin_id2, "hq,pin_id2" );
	if(ret < 0)
	{
		printk("gpio_request  pin_id2  fail  \n");
		goto gpio_id2_fail;
	}

	hq_hw_id= (gpio_get_value(pin_id2) << 2) | (gpio_get_value(pin_id1) << 1) | gpio_get_value(pin_id0);
	printk( "pin id0:%d id1:%d id2:%d hq_hw_id:%d",pin_id0, pin_id1,  pin_id2, hq_hw_id  );
	return 0;

gpio_id2_fail:
    gpio_free(pin_id2);
gpio_id1_fail:
    gpio_free(pin_id1);
gpio_id0_fail:
    gpio_free(pin_id0);

    return -1;
}

static int hwid_remove(struct platform_device *pdev)
{
	return 0;
}
static struct of_device_id hwid_dtb_table[] = {
	{.compatible = "hq,hwid",},
	{ },  //末尾需要定义一个空元素
};
//平台设备
static struct platform_driver hwid_driver = {
	.probe = hwid_probe,
	.remove = hwid_remove,
	.driver = {
		.name = "hq,hwid",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(hwid_dtb_table),
	},
};

//驱动加载函数
static int __init hwid_init(void)
{
    platform_driver_register(&hwid_driver);
    return 0;
}


//驱动卸载函数
static void __exit hwid_exit(void)
{
	platform_driver_unregister(&hwid_driver);
}

MODULE_AUTHOR("<dengzhiyong@huaqin.com>");
MODULE_DESCRIPTION("Huaqin Hardware Info Driver");
MODULE_LICENSE("GPL");


module_init(hwid_init);
module_exit(hwid_exit);
