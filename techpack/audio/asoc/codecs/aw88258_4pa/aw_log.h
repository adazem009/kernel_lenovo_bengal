#ifndef __AWINIC_LOG_H__
#define __AWINIC_LOG_H__

/********************************************
 * print information control
 *******************************************/
#define aw_dev_err(dev, format, ...) \
			pr_err("[Awinic] [%s] " format , dev_name(dev), ##__VA_ARGS__)

#define aw_dev_info(dev, format, ...) \
			pr_info("[Awinic] [%s] " format , dev_name(dev), ##__VA_ARGS__)

#define aw_dev_dbg(dev, format, ...) \
			pr_debug("[Awinic] [%s] " format , dev_name(dev), ##__VA_ARGS__)

#endif

