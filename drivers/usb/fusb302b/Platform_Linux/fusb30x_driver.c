/* 
 * File:   fusb30x_driver.c
 * Company: Fairchild Semiconductor
 *
 * Created on September 2, 2015, 10:22 AM
 */

/* Standard Linux includes */
#include <linux/init.h>                                                         // __init, __initdata, etc
#include <linux/module.h>                                                       // Needed to be a module
#include <linux/kernel.h>                                                       // Needed to be a kernel module
#include <linux/i2c.h>                                                          // I2C functionality
#include <linux/slab.h>                                                         // devm_kzalloc
#include <linux/types.h>                                                        // Kernel datatypes
#include <linux/errno.h>                                                        // EINVAL, ERANGE, etc
#include <linux/of_device.h>                                                    // Device tree functionality
#include <linux/extcon.h>
#include <linux/extcon-provider.h>
#include <linux/regulator/consumer.h>
#include <linux/power_supply.h>

/* Driver-specific includes */
#include "fusb30x_global.h"                                                     // Driver-specific structures/types
#include "platform_helpers.h"                                                   // I2C R/W, GPIO, misc, etc
#include "../core/core.h"                                                       // GetDeviceTypeCStatus
#include "class-dual-role.h"
#ifdef FSC_DEBUG
#include "dfs.h"
#endif // FSC_DEBUG

#include "fusb30x_driver.h"


extern int typec_mode;
extern int typec_rotation;
/******************************************************************************
* Driver functions
******************************************************************************/
static int __init fusb30x_init(void)
{
    	int tmp;
    	pr_info("==================FUSB  %s - Start driver initialization...\n", __func__);
	tmp = i2c_add_driver(&fusb30x_driver);
    	pr_info("===============aaaaaaaaaaaaaaaaaaaa===FUSB  %d - Start driver initialization...\n", tmp);
	return tmp;
}

static void __exit fusb30x_exit(void)
{
	i2c_del_driver(&fusb30x_driver);
    pr_debug("FUSB  %s - Driver deleted...\n", __func__);
}

static int fusb302_i2c_resume(struct device* dev)
{
    struct fusb30x_chip *chip;
        struct i2c_client *client = to_i2c_client(dev);

        if (client) {
            chip = i2c_get_clientdata(client);
                if (chip)
                up(&chip->suspend_lock);
        }
     return 0;
}

static int fusb302_i2c_suspend(struct device* dev)
{
    struct fusb30x_chip* chip;
        struct i2c_client* client =  to_i2c_client(dev);

        if (client) {
             chip = i2c_get_clientdata(client);
                 if (chip)
                    down(&chip->suspend_lock);
        }
        return 0;
}

 enum dual_role_property fusb_drp_properties[] = {
	DUAL_ROLE_PROP_SUPPORTED_MODES,
	DUAL_ROLE_PROP_MODE,
	DUAL_ROLE_PROP_PR,
	DUAL_ROLE_PROP_DR,
};
static const unsigned int usbpd_extcon_cable[] = {
	EXTCON_USB,
	EXTCON_USB_HOST,
	EXTCON_DISP_DP,
	EXTCON_NONE,
};


bool is_poweroff_charge()
{

	char *ptr;
	pr_err("=====================saved_command_line=%s (%s)\n", __func__, saved_command_line);
	ptr = strstr(saved_command_line, "androidboot.mode=charger");
	if (ptr != NULL)
	{
		pr_err("==========power_off_charge=======true====\r\n");
		return true;
	}
		pr_err("==========power_off_charge=====false======\r\n");
	return false;
}



static int fusb30x_probe(struct i2c_client* client,
                          const struct i2c_device_id* id)
{
    int ret = 0;
    struct fusb30x_chip* chip;
    struct i2c_adapter* adapter;
	struct power_supply *usb_psy;
	struct power_supply *pogo_psy;
//#ifdef CONFIG_DUAL_ROLE_USB_INTF
	struct dual_role_phy_desc *dual_desc;
	struct dual_role_phy_instance *dual_role;
//#endif
	pr_info("FUSB - %s\n", __func__);

	if(is_poweroff_charge())
	{
		pr_err("==========power_off_charger do not load fusb302 adriver======\r\n");
		return -ENODEV;
	}


    if (!client) {
        pr_err("===========FUSB  %s - Error: Client structure is NULL!\n", __func__);
        return -EINVAL;
    }

    dev_info(&client->dev, "%s\n", __func__);

    /* Make sure probe was called on a compatible device */
	if (!of_match_device(fusb302_match_table, &client->dev)) {
		dev_err(&client->dev,
			"FUSB  %s - Error: Device tree mismatch!\n",
			__func__);
		return -EINVAL;
	}



	/* register power supply */
	usb_psy = power_supply_get_by_name("usb");
	if (!usb_psy) {
		pr_info("FUSB - %s Could not get USB power_supply, deferring probe\n",
			__func__);
		return -EPROBE_DEFER;
	}

	pogo_psy = power_supply_get_by_name("customer");
	if (!pogo_psy) {
		pr_info("FUSB - %s Could not get USB power_supply, deferring probe\n",
			__func__);
		return -EPROBE_DEFER;
	}

    /* Allocate space for our chip structure (devm_* is managed by the device) */
    chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);
    if (!chip) {
		dev_err(&client->dev,
			"FUSB  %s - Error: Unable to allocate memory for g_chip!\n",
			__func__);

		power_supply_put(usb_psy);
		power_supply_put(pogo_psy);
		return -ENOMEM;
	}

    chip->client = client;                                                      // Assign our client handle to our chip
    fusb30x_SetChip(chip);                                                      // Set our global chip's address to the newly allocated memory
    pr_debug("==========FUSB  %s - Chip structure is set! Chip: %p ... g_chip: %p\n", __func__, chip, fusb30x_GetChip());

    /* Initialize the chip lock */
    mutex_init(&chip->lock);

    /* Initialize the chip's data members */
    fusb_InitChipData();
    pr_debug("FUSB  %s - Chip struct data initialized!\n", __func__);


//#ifdef CONFIG_DUAL_ROLE_USB_INTF
  //  if (IS_ENABLED(CONFIG_DUAL_ROLE_USB_INTF)) {
		dual_desc = devm_kzalloc(&client->dev, sizeof(struct dual_role_phy_desc),
					GFP_KERNEL);
		if (!dual_desc) {
			dev_err(&client->dev,
				"%s: unable to allocate dual role descriptor\n",
				__func__);
			return -ENOMEM;
		}
		dual_desc->name = "otg_default";
		dual_desc->supported_modes = DUAL_ROLE_SUPPORTED_MODES_DFP_AND_UFP;
		dual_desc->get_property = dual_role_get_local_prop;
		dual_desc->set_property = dual_role_set_prop;
		dual_desc->properties = fusb_drp_properties;
		dual_desc->num_properties = ARRAY_SIZE(fusb_drp_properties);
		dual_desc->property_is_writeable = dual_role_is_writeable;
		dual_role = devm_dual_role_instance_register(&client->dev, dual_desc);
		dual_role->drv_data = client;
		chip->dual_role = dual_role;
		chip->dr_desc = dual_desc;
	//}
//#endif

	chip->is_vbus_present = FALSE;
	chip->usb_psy = usb_psy;
	chip->pogo_psy = pogo_psy;
	fusb_init_event_handler();

    /* Verify that the system has our required I2C/SMBUS functionality (see <linux/i2c.h> for definitions) */
    adapter = to_i2c_adapter(client->dev.parent);
    if (i2c_check_functionality(adapter, FUSB30X_I2C_SMBUS_BLOCK_REQUIRED_FUNC))
    {
        chip->use_i2c_blocks = true;
    }
    else
    {
        // If the platform doesn't support block reads, try with block writes and single reads (works with eg. RPi)
        // NOTE: It is likely that this may result in non-standard behavior, but will often be 'close enough' to work for most things
        dev_warn(&client->dev, "FUSB  %s - Warning: I2C/SMBus block read/write functionality not supported, checking single-read mode...\n", __func__);
        if (!i2c_check_functionality(adapter, FUSB30X_I2C_SMBUS_REQUIRED_FUNC))
        {
            dev_err(&client->dev, "FUSB  %s - Error: Required I2C/SMBus functionality not supported!\n", __func__);
            dev_err(&client->dev, "FUSB  %s - I2C Supported Functionality Mask: 0x%x\n", __func__, i2c_get_functionality(adapter));
            return -EIO;
        }
    }
    pr_debug("=============FUSB  %s - I2C Functionality check passed! Block reads: %s\n", __func__, chip->use_i2c_blocks ? "YES" : "NO");

    /* Assign our struct as the client's driverdata */
    i2c_set_clientdata(client, chip);
    pr_debug("==============FUSB  %s - I2C client data set!\n", __func__);

    /* Verify that our device exists and that it's what we expect */
    if (!fusb_IsDeviceValid())
    {
        dev_err(&client->dev, "FUSB  %s - Error: Unable to communicate with device!\n", __func__);
        return -EIO;
    }
    pr_debug("============FUSB  %s - Device check passed!\n", __func__);

    /* reset fusb302*/
    fusb_reset();

    /* Initialize semaphore*/
    sema_init(&chip->suspend_lock, 1);

    /* Initialize the platform's GPIO pins and IRQ */
    ret = fusb_InitializeGPIO();
    if (ret)
    {
        dev_err(&client->dev, "FUSB  %s - Error: Unable to initialize GPIO!\n", __func__);
        return ret;
    }
    pr_debug("==========FUSB  %s - GPIO initialized!\n", __func__);

    /* Initialize sysfs file accessors */
    fusb_Sysfs_Init();
    pr_debug("===========FUSB  %s - Sysfs nodes created!\n", __func__);

#ifdef FSC_DEBUG
    /* Initialize debugfs file accessors */
    fusb_DFS_Init();
    pr_debug("============FUSB  %s - DebugFS nodes created!\n", __func__);
#endif // FSC_DEBUG

    /* Enable interrupts after successful core/GPIO initialization */
    ret = fusb_EnableInterrupts();
    if (ret)
    {
        dev_err(&client->dev, "FUSB  %s - Error: Unable to enable interrupts! Error code: %d\n", __func__, ret);
        return -EIO;
    }

    /* Initialize the core and enable the state machine (NOTE: timer and GPIO must be initialized by now)
    *  Interrupt must be enabled before starting 302 initialization */
    fusb_InitializeCore();
    pr_debug("================FUSB  %s - Core is initialized!\n", __func__);



    dev_info(&client->dev, "FUSB  %s - FUSB30X Driver loaded successfully!\n", __func__);
	return ret;
}

static int fusb30x_remove(struct i2c_client* client)
{
    pr_debug("FUSB  %s - Removing fusb30x device!\n", __func__);

#ifdef FSC_DEBUG
    /* Remove debugfs file accessors */
    fusb_DFS_Cleanup();
    pr_debug("FUSB  %s - DebugFS nodes removed.\n", __func__);
#endif // FSC_DEBUG

    fusb_GPIO_Cleanup();
    pr_debug("FUSB  %s - FUSB30x device removed from driver...\n", __func__);
    return 0;
}

static void fusb30x_shutdown(struct i2c_client *client)
{
    fusb_reset();
        pr_debug("FUSB	%s - fusb302 shutdown\n", __func__);
}


/*******************************************************************************
 * Driver macros
 ******************************************************************************/
module_init(fusb30x_init);                                                      // Defines the module's entrance function
module_exit(fusb30x_exit);                                                      // Defines the module's exit function

MODULE_LICENSE("GPL");                                                          // Exposed on call to modinfo
MODULE_DESCRIPTION("Fairchild FUSB30x Driver");                                 // Exposed on call to modinfo
MODULE_AUTHOR("Fairchild");                        								// Exposed on call to modinfo
