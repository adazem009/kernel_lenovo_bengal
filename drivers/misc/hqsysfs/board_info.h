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

#ifndef __BOARD_INFO_H__
#define __BOARD_INFO_H__

#define BOARD_ID_EVT    0
#define BOARD_ID_DVT1   1
#define BOARD_ID_DVT2   2
#define BOARD_ID_PVT1   3

int get_board_id(void);

#endif
