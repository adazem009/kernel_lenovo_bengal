#include <linux/module.h>
#include <linux/param.h>
#include <linux/jiffies.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/idr.h>
#include <linux/slab.h>
#include <linux/kobject.h>
#include <asm/unaligned.h>
#include <linux/gpio.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/of_gpio.h>
#include <linux/interrupt.h>
#include <linux/extcon.h>
#include <linux/extcon-provider.h>


static const unsigned int usb_extcon_cable[] = {
	EXTCON_USB,
	EXTCON_USB_HOST,
	EXTCON_DISP_DP,
	EXTCON_NONE,
};




int dock_status = -1;
int usb_charge_status = -1;
int mcu_switch = -1;
int mcu_reset = -1;
int charge_enable = -1;
int usb_vbus = -1;
int usb_status = 0;//0:idle 1:devices 2:host
int pogo_typec_mode = 0;
int pogo_typec_rotation = 0;
struct platform_device *pogo_dev;
struct work_struct irq_wq;
struct extcon_dev *extcon;

static enum power_supply_property dock_pogo_props[] = {
	POWER_SUPPLY_PROP_charge_dock,
	POWER_SUPPLY_PROP_charge_usb,
	POWER_SUPPLY_PROP_mcu_switch,
	POWER_SUPPLY_PROP_mcu_reset,
	POWER_SUPPLY_PROP_charge_enable,
	POWER_SUPPLY_PROP_usb_vbus_ctl,
	POWER_SUPPLY_PROP_usb_device_ctl,
	POWER_SUPPLY_PROP_usb_host_ctl,
    	POWER_SUPPLY_PROP_TYPEC_MODE,
    	POWER_SUPPLY_PROP_TYPEC_CC_ORIENTATION,
};

static int dock_pogo_property(struct power_supply *psy,
                    enum power_supply_property psp,
                    union power_supply_propval *val)
{

    switch (psp) {
    case POWER_SUPPLY_PROP_charge_dock:
        val->intval = !gpio_get_value(dock_status);
        break;
    case POWER_SUPPLY_PROP_charge_usb:
        val->intval = !gpio_get_value(usb_charge_status);
        break;
    case POWER_SUPPLY_PROP_mcu_switch:
        val->intval = gpio_get_value(mcu_switch);
        break;
    case POWER_SUPPLY_PROP_mcu_reset:
        val->intval = gpio_get_value(mcu_reset);
        break;
    case POWER_SUPPLY_PROP_charge_enable:
        val->intval = !gpio_get_value(charge_enable);
        break;
    case POWER_SUPPLY_PROP_usb_vbus_ctl:
        val->intval = gpio_get_value(usb_vbus);
        break;
    case POWER_SUPPLY_PROP_usb_device_ctl:
    case POWER_SUPPLY_PROP_usb_host_ctl:
        val->intval = usb_status;
        break;
    case POWER_SUPPLY_PROP_TYPEC_MODE:
	 val->intval = pogo_typec_mode;
	 break;
    case POWER_SUPPLY_PROP_TYPEC_CC_ORIENTATION:
	 val->intval = pogo_typec_rotation;
	 break;
    default:
        return -EINVAL;
    }

    return 0;
}


void set_usb_source_host()
{
#ifndef FACTORY_VERSION
	struct power_supply *ext_psy = power_supply_get_by_name("usb");
        union power_supply_propval reta = {1,};
	power_supply_set_property(ext_psy,POWER_SUPPLY_PROP_set_typec_host,&reta);
#endif
}

void set_usb_sink_device()
{
#ifndef FACTORY_VERSION
	struct power_supply *ext_psy = power_supply_get_by_name("usb");
        union power_supply_propval reta = {1,};
	power_supply_set_property(ext_psy,POWER_SUPPLY_PROP_set_typec_device,&reta);
#endif
}

void partner_delete()
{
#ifndef FACTORY_VERSION
	struct power_supply *ext_psy = power_supply_get_by_name("usb");
        union power_supply_propval reta = {0,};
	power_supply_set_property(ext_psy,POWER_SUPPLY_PROP_set_typec_device,&reta);	
#endif
}


void stop_usb_host()
{
	pr_err("===============%s=============\r\n", __func__);
	extcon_set_state_sync(extcon, EXTCON_USB_HOST, 0);
	mdelay(50);
}

void start_usb_host()
{
	union extcon_property_value val;
	pr_err("===============%s=============\r\n", __func__);
	val.intval = false;
	extcon_set_property(extcon, EXTCON_USB_HOST,EXTCON_PROP_USB_SS, val);
	extcon_set_state_sync(extcon, EXTCON_USB_HOST, 1);
	mdelay(50);
}

void stop_usb_device()
{
	pr_err("===============%s=============\r\n", __func__);
	extcon_set_state_sync(extcon, EXTCON_USB, 0);
	mdelay(50);
}

void start_usb_device()
{
	union extcon_property_value val;
	pr_err("===============%s=============\r\n", __func__);
	val.intval = false;
	extcon_set_property(extcon,EXTCON_USB,EXTCON_PROP_USB_SS, val);
	extcon_set_state_sync(extcon, EXTCON_USB, 1);
	mdelay(50);
}


static int dock_pogo_set_property(struct power_supply *psy,
		enum power_supply_property psp,
		const union power_supply_propval *val)
{

    switch (psp) {
    case POWER_SUPPLY_PROP_mcu_switch:
	 if(val->intval == 0)
	 {
	 	gpio_set_value(mcu_switch,0);
	 }
	 else
	 {
		gpio_set_value(mcu_switch,1);
	 }
        break;

    case POWER_SUPPLY_PROP_mcu_reset:
	 if(val->intval == 0)
	 {
	 	gpio_set_value(mcu_reset,0);
	 }
	 else
	 {
		gpio_set_value(mcu_reset,1);
	 }
        break;

    case POWER_SUPPLY_PROP_charge_enable:
	 if(val->intval == 0)
	 {
	 	gpio_set_value(charge_enable,1);
	 }
	 else
	 {
	 	gpio_set_value(charge_enable,1);
		mdelay(50);
		gpio_set_value(charge_enable,0);
	 }
	 pr_err("========charge_enable_set[%d]====is_usb_charger_in[%d]=======charge_enable_get[%d]======\r\n",val->intval, !gpio_get_value(usb_charge_status),!gpio_get_value(charge_enable));
        break;

    case POWER_SUPPLY_PROP_usb_vbus_ctl:
	 if(val->intval == 0)
	 {
		pr_err("===============usb disable boost =============\r\n");
	 	gpio_set_value(usb_vbus,0);
	 }
	 else
	 {
		pr_err("===============usb enable boost =============\r\n");
		gpio_set_value(usb_vbus,1);
	 }
        break;

    case POWER_SUPPLY_PROP_usb_device_ctl:

	 if(val->intval == 0)
	 {

		if(usb_status == 0)
		{
			pr_err("===============usb already in idle mode =============\r\n");
		}
		else if(usb_status == 1)
		{
			pr_err("===============change usb from device to idle =============\r\n");
			stop_usb_device();
			usb_status = 0;
			partner_delete();
		}
		else if(usb_status == 2)
		{
			pr_err("===============change usb from host to idle by device not allow=============\r\n");
			//stop_usb_host();
		}

	 }
	 else
	 {
		if(usb_status == 1)
		{
			pr_err("===============usb already in device mode =============\r\n");
		}
		else if(usb_status == 2)
		{
			pr_err("===============change usb from host to device not allow=============\r\n");
			stop_usb_host();
			start_usb_host();
		}
		else if(usb_status == 0)
		{
			pr_err("===============change usb from idle to device =============\r\n");
			start_usb_device();
			stop_usb_device();
			start_usb_device();
			usb_status = 1;
			set_usb_sink_device();
		}
	 }
        break;
    case POWER_SUPPLY_PROP_usb_host_ctl:
	 if(val->intval == 0)
	 {
		if(usb_status == 0)
		{
			pr_err("===============usb already in idle mode =============\r\n");
		}
		else if(usb_status == 1)
		{
			pr_err("===============change usb from device to idle by host not allow=============\r\n");
			//stop_usb_device();
		}
		else if(usb_status == 2)
		{
			pr_err("===============change usb from host to idle =============\r\n");
			stop_usb_host();
			usb_status = 0;
			partner_delete();
		}

	 }
	 else
	 {
		if(usb_status == 0)
		{
			pr_err("===============change usb from idle to host =============\r\n");
			start_usb_host();
			set_usb_source_host();

		}
		else if(usb_status == 1)
		{
			pr_err("===============change usb from device to host =============\r\n");
			stop_usb_device();
			start_usb_host();
			set_usb_source_host();
		}
		else if(usb_status == 2)
		{
			pr_err("===============usb already in host mode =============\r\n");
		}
		usb_status = 2;
	 }
        break;
    case POWER_SUPPLY_PROP_TYPEC_MODE:
	 pogo_typec_mode = val->intval;
         power_supply_changed(psy);
	 break;
    case POWER_SUPPLY_PROP_TYPEC_CC_ORIENTATION:
	 pogo_typec_rotation = val->intval;

    default:
        return -EINVAL;
    }

    return 0;
}



static const struct power_supply_desc dock_pogo_desc = {
	.name			= "customer",
	.type			= POWER_SUPPLY_TYPE_CUST,
	.properties		= dock_pogo_props,
	.num_properties		= 10,
	.get_property		= dock_pogo_property,
	.set_property		= dock_pogo_set_property,
	.property_is_writeable	= NULL,
};

void uevent_send(struct work_struct *work)
{
	//int usb_value,dock_value;
	//bool is_otg,is_pogo_en;
        char *env[] = { "dock-pogo", "status-change", NULL };
	kobject_uevent_env(&pogo_dev->dev.kobj, KOBJ_CHANGE,env);
	gpio_set_value(charge_enable,0);
	/*usb_value = !gpio_get_value(usb_charge_status);
	dock_value = !gpio_get_value(dock_status);
	is_otg = charger_dev_is_otg(primary_charger);
	is_pogo_en = charger_dev_is_pogo_enable(primary_charger);
	printk("=============pogo_state_changed=================is_otg[%d]=========is_pogo_en[%d]========\r\n",is_otg,is_pogo_en);
	if((is_otg == 0)&&(is_pogo_en == 1))
	{
		printk("=========leon====enable pogo when power change=================\r\n");
		mdelay(1000);
		charger_dev_pogo_enable(primary_charger,true);
	}
	else
	{
		printk("=========leon====nothing when power change=================\r\n");
	}*/

}

static irqreturn_t dock_interrupt(int irq, void *data)
{
	printk("=============pogo_state_changed---dock=================\r\n");
	schedule_work(&irq_wq);
	return IRQ_HANDLED;
}


static irqreturn_t usb_interrupt(int irq, void *data)
{
	printk("=============pogo_state_changed--usb=================\r\n");
	schedule_work(&irq_wq);
	return IRQ_HANDLED;
}

static int dock_pogo_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *node = dev->of_node;

	int ret;
	int irq_dock;

	int irq_usb;

	//extcon = devm_kzalloc(dev, sizeof(struct extcon_dev), GFP_KERNEL);
	pogo_dev = devm_kzalloc(dev, sizeof(struct platform_device), GFP_KERNEL);
	pogo_dev = pdev;

	dock_status = of_get_named_gpio(node, "dock_status", 0);
	if (dock_status < 0)
	{
		printk( "dock_status is not available\n");
		return -1;
	}

	usb_charge_status = of_get_named_gpio(node, "usb_charge_status", 0);
	if (usb_charge_status < 0)
	{
		printk( "usb_charge_status is not available\n");
		return -1;
	}

	mcu_switch = of_get_named_gpio(node, "mcu_switch", 0);
	if (mcu_switch < 0)
	{
		printk( "mcu_switch is not available\n");
		return -1;
	}

	mcu_reset = of_get_named_gpio(node, "mcu_reset", 0);
	if (mcu_reset < 0)
	{
		printk( "mcu_reset is not available\n");
		return -1;
	}

	charge_enable = of_get_named_gpio(node, "charge_enable", 0);
	if (charge_enable < 0)
	{
		printk( "charge_enable is not available\n");
		return -1;
	}

	usb_vbus = of_get_named_gpio(node, "usb_vbus", 0);
	if (usb_vbus < 0)
	{
		printk( "usb_vbus is not available\n");
		return -1;
	}

	node = of_find_compatible_node(NULL, NULL, "qcom,usb_int");
	if (node) {
		mdelay(10);
		printk( "usb ===irq_node available\n");
	}
	else {
		mdelay(10);
		printk( "usb ===irq_node is not available\n");
	}
	irq_usb = irq_of_parse_and_map(node, 0);



	node = of_find_compatible_node(NULL, NULL, "qcom,pogo_int");
	if (node) {
		printk( "pogo ===irq_node available\r\n");
	}
	else {
		printk( "pogo===irq_node is not available\r\n");
	}
	irq_dock = irq_of_parse_and_map(node, 0);

	gpio_request(dock_status,"dock_status");
	gpio_direction_input(dock_status);

	gpio_request(usb_charge_status,"usb_charge_status");
	gpio_direction_input(usb_charge_status);

	gpio_request(mcu_switch,"mcu_switch");
	gpio_direction_output(mcu_switch,0);

	gpio_request(mcu_reset,"mcu_reset");
	gpio_direction_output(mcu_reset,1);

	gpio_request(charge_enable,"charge_enable");
	gpio_direction_output(charge_enable,0);


	gpio_request(usb_vbus,"usb_vbus");
	gpio_direction_output(usb_vbus,0);


	INIT_WORK(&irq_wq,uevent_send);

	request_irq(irq_dock, dock_interrupt, IRQ_TYPE_EDGE_BOTH, "pogo_dock_irq", NULL);
	request_irq(irq_usb, usb_interrupt, IRQ_TYPE_EDGE_BOTH, "pogo_usb_irq", NULL);


	extcon = devm_extcon_dev_allocate(&pdev->dev, usb_extcon_cable);
	if (!extcon) {
		printk("=============Error: Unable to allocate memory for extcon!\n",__func__);
		return -ENOMEM;
	}

	ret = devm_extcon_dev_register(&pdev->dev, extcon);
	if (ret) {
		printk("=======pogo failed to register extcon device\n");
		return -1;
	}


	extcon_set_property_capability(extcon, EXTCON_USB,EXTCON_PROP_USB_TYPEC_POLARITY);
	extcon_set_property_capability(extcon, EXTCON_USB,EXTCON_PROP_USB_SS);
	extcon_set_property_capability(extcon, EXTCON_USB,EXTCON_PROP_USB_TYPEC_MED_HIGH_CURRENT);
	extcon_set_property_capability(extcon, EXTCON_USB_HOST,EXTCON_PROP_USB_TYPEC_POLARITY);
	extcon_set_property_capability(extcon, EXTCON_USB_HOST,EXTCON_PROP_USB_SS);


  	power_supply_register(&pdev->dev, &dock_pogo_desc,NULL);
    return 0;
}


static struct of_device_id dock_pogo_table[] = {
	{ .compatible = "qcom,dock_pogo",},
	{ },
};

static struct platform_driver dock_pogo_driver = {
	.probe = dock_pogo_probe,
	.driver = {
		.name = "dock_pogo",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(dock_pogo_table),
	},
};


static __init int dock_pogo_init(void)
{
	return platform_driver_register(&dock_pogo_driver);
}
static __exit void dock_pogo_exit(void)
{
	//i2c_del_driver(&mm8013_i2c_driver);
}
module_init(dock_pogo_init);
module_exit(dock_pogo_exit);
MODULE_AUTHOR("leon");
MODULE_DESCRIPTION("dock pogo driver for levono m11");
MODULE_LICENSE("GPL v2");
