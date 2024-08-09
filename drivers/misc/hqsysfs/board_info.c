/* Copyright © 2020, The Lenovo. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/of.h>
#include <linux/module.h>

int hq_hw_state=-1;
int get_board_id(void)
{
	struct device_node *rootnp;
	int board_id_buf[2];
	int rc = 0;

	rootnp = of_find_node_by_path("/");
	if (!rootnp) {
		pr_err("Failed to get the root node\n");
		return -1;
	}

	rc = of_property_read_u32_array(rootnp, "qcom,board-id",
					board_id_buf, 2);
	if (rc) {
		pr_err("Failed to get qcom,board-id property\n");
		return -1;
	}

	pr_err("board_id: %d %d\n", board_id_buf[0], board_id_buf[1]);

	return board_id_buf[1];
}
EXPORT_SYMBOL(get_board_id);

int hq_get_hw_state(void)
{
	if( hq_hw_state == -1)
	{
		hq_hw_state=get_board_id();
	}
	pr_err("hq_hw_state:  %d\n",hq_hw_state );
	return  hq_hw_state;
}
EXPORT_SYMBOL(hq_get_hw_state);
#if 0
static int board_id_get(char *str, const struct kernel_param *kp)
{
	int board_id = 0;

	board_id = get_board_id();
	return sprintf(str, "%d\n", board_id);
}

module_param_call(board_id, NULL, board_id_get, NULL, 0444);
#endif

