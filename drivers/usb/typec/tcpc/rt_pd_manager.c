/*
 * Copyright (C) 2020 Richtek Inc.
 *
 * Richtek RT PD Manager
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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>

#include "inc/tcpm.h"

#define RT_PD_MANAGER_VERSION	"0.0.2_G"

bool has_rt1711 = true;
struct rt_pd_manager_data {
	struct device *dev;
	struct mutex param_lock;
	struct tcpc_device *tcpc_dev;
	struct notifier_block pd_nb;
	int sink_mv_new;
	int sink_ma_new;
	int sink_mv_old;
	int sink_ma_old;
};



void switch_usb_host(bool enable)
{

        union power_supply_propval reta = {0,};
	struct power_supply *pogo_psy;
	pr_err("======================RT1711 - %s=======enable[%d]=============\n", __func__,enable);
	if(enable)
	{
		reta.intval = 1;
	}
	else
	{
		reta.intval = 0;
	}
	pogo_psy = power_supply_get_by_name("customer");
	power_supply_set_property(pogo_psy,POWER_SUPPLY_PROP_usb_host_ctl,&reta);
}

void switch_usb_vbus(bool enable)
{
        union power_supply_propval reta = {0,};
	struct power_supply *pogo_psy;

	pr_err("======================RT1711 - %s=======enable[%d]=============\n", __func__,enable);
	if(enable)
	{
		reta.intval = 1;
	}
	else
	{
		reta.intval = 0;
	}
	pogo_psy = power_supply_get_by_name("customer");
	power_supply_set_property(pogo_psy,POWER_SUPPLY_PROP_usb_vbus_ctl,&reta);
}






static int pd_tcp_notifier_call(struct notifier_block *nb,
				unsigned long event, void *data)
{

	struct tcp_notify *noti = data;
	struct rt_pd_manager_data *rpmd =
		container_of(nb, struct rt_pd_manager_data, pd_nb);
        struct power_supply *ext_psy = power_supply_get_by_name("customer");
        union power_supply_propval reta = {0,};

	uint8_t old_state = TYPEC_UNATTACHED, new_state = TYPEC_UNATTACHED;


	pr_err("===========================%s (%s)\n", __func__, RT_PD_MANAGER_VERSION);

	switch (event) {
	case TCP_NOTIFY_SINK_VBUS:
		mutex_lock(&rpmd->param_lock);
		rpmd->sink_mv_new = noti->vbus_state.mv;
		rpmd->sink_ma_new = noti->vbus_state.ma;
		pr_err( "===================%s sink vbus %dmV %dmA type(0x%02X)\n",
				    __func__, rpmd->sink_mv_new,
				    rpmd->sink_ma_new, noti->vbus_state.type);

		if ((rpmd->sink_mv_new != rpmd->sink_mv_old) ||
		    (rpmd->sink_ma_new != rpmd->sink_ma_old)) {
			rpmd->sink_mv_old = rpmd->sink_mv_new;
			rpmd->sink_ma_old = rpmd->sink_ma_new;
			if (rpmd->sink_mv_new && rpmd->sink_ma_new) {
				/* enable VBUS power path */
				//switch_usb_vbus(true);
			} else {
				/* disable VBUS power path */
				//switch_usb_vbus(false);
			}
		}
		mutex_unlock(&rpmd->param_lock);
		break;
	case TCP_NOTIFY_SOURCE_VBUS:
		pr_err("%s source vbus %dmV\n",
				    __func__, noti->vbus_state.mv);
		if (noti->vbus_state.mv)
		{
			switch_usb_vbus(true);
			//switch_usb_host(true);
		}
		else
		{
			switch_usb_vbus(false);
		}
		/* enable/disable OTG power output */
		break;
	case TCP_NOTIFY_TYPEC_STATE:

		old_state = noti->typec_state.old_state;
		new_state = noti->typec_state.new_state;

		pr_err("===========================old_state%d  new_state(%d)\n", old_state, new_state);

		if(noti->typec_state.polarity == 0)
		{
			//notifier type-c rotation 1
		    reta.intval = 1;
		    power_supply_set_property(ext_psy,POWER_SUPPLY_PROP_TYPEC_CC_ORIENTATION,&reta);

		}
		else
		{
			//notifier type-c rotation 2
		    reta.intval = 2;
		    power_supply_set_property(ext_psy,POWER_SUPPLY_PROP_TYPEC_CC_ORIENTATION,&reta);
		}

		if (old_state == TYPEC_UNATTACHED &&
		    (new_state == TYPEC_ATTACHED_SNK ||
		     new_state == TYPEC_ATTACHED_NORP_SRC ||
		     new_state == TYPEC_ATTACHED_CUSTOM_SRC)) {
			dev_info(rpmd->dev,
				 "%s Charger plug in, polarity = %d\n",
				 __func__, noti->typec_state.polarity);
			/* start charger type detection,
			 * and enable device connection */
		} else if ((old_state == TYPEC_ATTACHED_SNK ||
			    old_state == TYPEC_ATTACHED_NORP_SRC ||
			    old_state == TYPEC_ATTACHED_CUSTOM_SRC) &&
			    new_state == TYPEC_UNATTACHED) {
			dev_info(rpmd->dev,
				 "%s OTG plug in, polarity = %d\n",
				 __func__, noti->typec_state.polarity);

		    	reta.intval = 0;
		    	power_supply_set_property(ext_psy,POWER_SUPPLY_PROP_TYPEC_CC_ORIENTATION,&reta);
			//notifier type-c rotation 0

			/* report charger plug-out,
			 * and disable device connection */
		} else if (old_state == TYPEC_UNATTACHED &&
			   new_state == TYPEC_ATTACHED_SRC) {
			pr_err("%s OTG plug in\n", __func__);
				switch_usb_host(true);
			/* enable host connection */
		} else if (old_state == TYPEC_ATTACHED_SRC &&
			   new_state == TYPEC_UNATTACHED) {
			pr_err("%s OTG plug out\n", __func__);
				switch_usb_host(false);
			/* disable host connection */
		} else if (old_state == TYPEC_UNATTACHED &&
			   new_state == TYPEC_ATTACHED_AUDIO) {
			pr_err("%s Audio plug in\n", __func__);
    			reta.intval = POWER_SUPPLY_TYPEC_SINK_AUDIO_ADAPTER;
    			power_supply_set_property(ext_psy,POWER_SUPPLY_PROP_TYPEC_MODE,&reta);

			/* enable AudioAccessory connection */
		} else if (old_state == TYPEC_ATTACHED_AUDIO &&
			   new_state == TYPEC_UNATTACHED) {
			pr_err("%s Audio plug out\n", __func__);
    			reta.intval = POWER_SUPPLY_TYPEC_NONE;
    			power_supply_set_property(ext_psy,POWER_SUPPLY_PROP_TYPEC_MODE,&reta);
			/* disable AudioAccessory connection */
		}
		break;
	case TCP_NOTIFY_PR_SWAP:
		pr_err("%s power role swap, new role = %d\n",
				    __func__, noti->swap_state.new_role);
		if (noti->swap_state.new_role == PD_ROLE_SINK) {
			pr_err("%s swap power role to sink\n",
					    __func__);
			/* report charger plug-in without charger type detection
			 * to not interfering with USB2.0 communication */
		} else if (noti->swap_state.new_role == PD_ROLE_SOURCE) {
			pr_err("%s swap power role to source\n",
					    __func__);
			/* report charger plug-out */
		}
		break;
	case TCP_NOTIFY_DR_SWAP:
		pr_err("%s data role swap, new role = %d\n",
				    __func__, noti->swap_state.new_role);
		if (noti->swap_state.new_role == PD_ROLE_UFP) {
			pr_err("%s swap data role to device\n",
					    __func__);
			/* disable host connection,
			 * and enable device connection */
			switch_usb_host(false);
		} else if (noti->swap_state.new_role == PD_ROLE_DFP) {
			pr_err("%s swap data role to host\n",
					    __func__);
			/* disable device connection,
			 * and enable host connection */
			switch_usb_host(true);
		}
		break;
	case TCP_NOTIFY_EXT_DISCHARGE:
		dev_info(rpmd->dev, "%s ext discharge = %d\n",
				    __func__, noti->en_state.en);
		/* enable/disable VBUS discharge */
		break;
	case TCP_NOTIFY_PD_STATE:
		dev_info(rpmd->dev, "%s pd state = %d\n",
				    __func__, noti->pd_state.connected);
		switch (noti->pd_state.connected) {
		case PD_CONNECT_NONE:
			break;
		case PD_CONNECT_HARD_RESET:
			break;
		case PD_CONNECT_PE_READY_SNK:
		case PD_CONNECT_PE_READY_SNK_PD30:
		case PD_CONNECT_PE_READY_SNK_APDO:
		case PD_CONNECT_TYPEC_ONLY_SNK:
			break;
		};
		break;
	default:
		break;
	};
	return NOTIFY_OK;
}

static int rt_pd_manager_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct rt_pd_manager_data *rpmd = NULL;

	pr_err("===========================%s (%s)\n", __func__, RT_PD_MANAGER_VERSION);

	if(!has_rt1711)
	{
		return -ENODEV;
	}
	
	rpmd = devm_kzalloc(&pdev->dev, sizeof(*rpmd), GFP_KERNEL);
	if (!rpmd)
		return -ENOMEM;

	rpmd->dev = &pdev->dev;
	mutex_init(&rpmd->param_lock);
	rpmd->tcpc_dev = tcpc_dev_get_by_name("type_c_port0");
	if (!rpmd->tcpc_dev) {
		dev_err(rpmd->dev, "%s get tcpc device fail\n", __func__);
		ret = -EPROBE_DEFER;
		goto err_get_tcpc_dev;
	}
	rpmd->pd_nb.notifier_call = pd_tcp_notifier_call;
	ret = register_tcp_dev_notifier(rpmd->tcpc_dev, &rpmd->pd_nb,
					TCP_NOTIFY_TYPE_ALL);
	if (ret < 0) {
		dev_err(rpmd->dev, "%s register tcpc notifer fail(%d)\n",
				   __func__, ret);
		ret = -EPROBE_DEFER;
		goto err_reg_tcpc_notifier;
	}

	platform_set_drvdata(pdev, rpmd);
	dev_info(rpmd->dev, "%s OK!!\n", __func__);
	return 0;

err_reg_tcpc_notifier:
err_get_tcpc_dev:
	mutex_destroy(&rpmd->param_lock);
	return ret;
}

static int rt_pd_manager_remove(struct platform_device *pdev)
{
	int ret = 0;
	struct rt_pd_manager_data *rpmd = platform_get_drvdata(pdev);

	if (!rpmd)
		return -EINVAL;

	ret = unregister_tcp_dev_notifier(rpmd->tcpc_dev, &rpmd->pd_nb,
					  TCP_NOTIFY_TYPE_ALL);
	if (ret < 0)
		dev_err(rpmd->dev, "%s register tcpc notifer fail(%d)\n",
				   __func__, ret);
	mutex_destroy(&rpmd->param_lock);

	return ret;
}

static const struct of_device_id rt_pd_manager_of_match[] = {
	{ .compatible = "richtek,rt-pd-manager" },
	{ }
};
MODULE_DEVICE_TABLE(of, rt_pd_manager_of_match);

static struct platform_driver rt_pd_manager_driver = {
	.driver = {
		.name = "rt-pd-manager",
		.of_match_table = of_match_ptr(rt_pd_manager_of_match),
	},
	.probe = rt_pd_manager_probe,
	.remove = rt_pd_manager_remove,
};
module_platform_driver(rt_pd_manager_driver);

MODULE_AUTHOR("Jeff Chang");
MODULE_DESCRIPTION("Richtek pd manager driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(RT_PD_MANAGER_VERSION);
/*
 * 0.0.2
 * (1) Initialize old_state and new_state
 *
 * Release Note
 * 0.0.1
 * (1) Add all possible notification events
 */
