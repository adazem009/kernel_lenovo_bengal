/*
 * Copyright (C) 2010 - 2021 Novatek, Inc.
 *
 * $Revision: 83893 $
 * $Date: 2021-06-21 10:52:25 +0800 (週一, 21 六月 2021) $
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 */


#include <linux/proc_fs.h>
#include <linux/seq_file.h>

#include "nt36xxx.h"

#if NVT_TOUCH_EXT_PROC
#define NVT_FW_VERSION "nvt_fw_version"
#define NVT_BASELINE "nvt_baseline"
#define NVT_RAW "nvt_raw"
#define NVT_DIFF "nvt_diff"
#define NVT_PEN_DIFF "nvt_pen_diff"
//+OAK700,shenwenbin.wt,add,20211204,add charge mode
#define PANEL_DIRECTION "panel_direction"
//-OAK700,shenwenbin.wt,add,20211204,add charge mode
//+OAK4146,shenwenbin.wt,ADD,20220125,add TP game mode
#define GAME_MODE_NOTIFY "game_mode"
#define EDGE_GRID_ZONE "edge_grid_zone"
//-OAK4146,shenwenbin.wt,ADD,20220125,add TP game mode

#define BUS_TRANSFER_LENGTH  256

#define NORMAL_MODE 0x00
#define TEST_MODE_2 0x22
#define HANDSHAKING_HOST_READY 0xBB

//+OAK4146,shenwenbin.wt,ADD,20220125,add TP game mode
#define GAME_MODE_PALM_CMD 0x79
#define EXTEND_CUSTOMIZED_CMD 0x7F
//-OAK4146,shenwenbin.wt,ADD,20220125,add TP game mode
//+OAK700,shenwenbin.wt,add,20211204,add charge mode
#define EDGE_REJECT_VERTICLE_CMD 0xBA
//+OAK700,shenwenbin.wt,modify,20220224,turn panel direction
#define EDGE_REJECT_LEFT_UP 0xBC
#define EDGE_REJECT_RIGHT_UP 0xBB
//-OAK700,shenwenbin.wt,modify,20220224,turn panel direction
//-OAK700,shenwenbin.wt,add,20211204,add charge mode

#define XDATA_SECTOR_SIZE   256

//+OAK4146,shenwenbin.wt,ADD,20220125,add TP game mode
typedef enum {
	EDGE_REJECT_L = 5,
	EDGE_REJECT_H,
}FUNCT_BIT;
//-OAK4146,shenwenbin.wt,ADD,20220125,add TP game mode

static uint8_t xdata_tmp[5000] = {0};
static int32_t xdata[2500] = {0};
static int32_t xdata_pen_tip_x[256] = {0};
static int32_t xdata_pen_tip_y[256] = {0};
static int32_t xdata_pen_ring_x[256] = {0};
static int32_t xdata_pen_ring_y[256] = {0};

static struct proc_dir_entry *NVT_proc_fw_version_entry;
static struct proc_dir_entry *NVT_proc_baseline_entry;
static struct proc_dir_entry *NVT_proc_raw_entry;
static struct proc_dir_entry *NVT_proc_diff_entry;
static struct proc_dir_entry *NVT_proc_pen_diff_entry;
static struct proc_dir_entry *NVT_proc_edge_reject_entry;		//OAK700,shenwenbin.wt,add,20211204,add charge mode
//+OAK4146,shenwenbin.wt,ADD,20220125,add TP game mode
static struct proc_dir_entry *NVT_proc_game_mode_entry;
static struct proc_dir_entry *NVT_proc_edge_grid_zone_entry;
//-OAK4146,shenwenbin.wt,ADD,20220125,add TP game mode

/*******************************************************
Description:
	Novatek touchscreen change mode function.

return:
	n.a.
*******************************************************/
void nvt_change_mode(uint8_t mode)
{
	uint8_t buf[8] = {0};

	//---set xdata index to EVENT BUF ADDR---
	nvt_set_page(ts->mmap->EVENT_BUF_ADDR | EVENT_MAP_HOST_CMD);

	//---set mode---
	buf[0] = EVENT_MAP_HOST_CMD;
	buf[1] = mode;
	CTP_SPI_WRITE(ts->client, buf, 2);

	if (mode == NORMAL_MODE) {
		usleep_range(20000, 20000);
		buf[0] = EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE;
		buf[1] = HANDSHAKING_HOST_READY;
		CTP_SPI_WRITE(ts->client, buf, 2);
		usleep_range(20000, 20000);
	}
}

int32_t nvt_set_pen_inband_mode_1(uint8_t freq_idx, uint8_t x_term)
{
	uint8_t buf[8] = {0};
	int32_t i = 0;
	const int32_t retry = 5;

	//---set xdata index to EVENT BUF ADDR---
	nvt_set_page(ts->mmap->EVENT_BUF_ADDR | EVENT_MAP_HOST_CMD);

	//---set mode---
	buf[0] = EVENT_MAP_HOST_CMD;
	buf[1] = 0xC1;
	buf[2] = 0x02;
	buf[3] = freq_idx;
	buf[4] = x_term;
	CTP_SPI_WRITE(ts->client, buf, 5);

	for (i = 0; i < retry; i++) {
		buf[0] = EVENT_MAP_HOST_CMD;
		buf[1] = 0xFF;
		CTP_SPI_READ(ts->client, buf, 2);

		if (buf[1] == 0x00)
			break;

		usleep_range(10000, 10000);
	}

	if (i >= retry) {
		NVT_ERR("failed, i=%d, buf[1]=0x%02X\n", i, buf[1]);
		return -1;
	} else {
		return 0;
	}
}

int32_t nvt_set_pen_normal_mode(void)
{
	uint8_t buf[8] = {0};
	int32_t i = 0;
	const int32_t retry = 5;

	//---set xdata index to EVENT BUF ADDR---
	nvt_set_page(ts->mmap->EVENT_BUF_ADDR | EVENT_MAP_HOST_CMD);

	//---set mode---
	buf[0] = EVENT_MAP_HOST_CMD;
	buf[1] = 0xC1;
	buf[2] = 0x04;
	CTP_SPI_WRITE(ts->client, buf, 3);

	for (i = 0; i < retry; i++) {
		buf[0] = EVENT_MAP_HOST_CMD;
		buf[1] = 0xFF;
		CTP_SPI_READ(ts->client, buf, 2);

		if (buf[1] == 0x00)
			break;

		usleep_range(10000, 10000);
	}

	if (i >= retry) {
		NVT_ERR("failed, i=%d, buf[1]=0x%02X\n", i, buf[1]);
		return -1;
	} else {
		return 0;
	}
}

/*******************************************************
Description:
	Novatek touchscreen get firmware pipe function.

return:
	Executive outcomes. 0---pipe 0. 1---pipe 1.
*******************************************************/
uint8_t nvt_get_fw_pipe(void)
{
	uint8_t buf[8]= {0};

	//---set xdata index to EVENT BUF ADDR---
	nvt_set_page(ts->mmap->EVENT_BUF_ADDR | EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE);

	//---read fw status---
	buf[0] = EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE;
	buf[1] = 0x00;
	CTP_SPI_READ(ts->client, buf, 2);

	//NVT_LOG("FW pipe=%d, buf[1]=0x%02X\n", (buf[1]&0x01), buf[1]);

	return (buf[1] & 0x01);
}

/*******************************************************
Description:
	Novatek touchscreen read meta data function.

return:
	n.a.
*******************************************************/
void nvt_read_mdata(uint32_t xdata_addr, uint32_t xdata_btn_addr)
{
	int32_t i = 0;
	int32_t j = 0;
	int32_t k = 0;
	uint8_t buf[BUS_TRANSFER_LENGTH + 2] = {0};
	uint32_t head_addr = 0;
	int32_t dummy_len = 0;
	int32_t data_len = 0;
	int32_t residual_len = 0;

	//---set xdata sector address & length---
	head_addr = xdata_addr - (xdata_addr % XDATA_SECTOR_SIZE);
	dummy_len = xdata_addr - head_addr;
	data_len = ts->x_num * ts->y_num * 2;
	residual_len = (head_addr + dummy_len + data_len) % XDATA_SECTOR_SIZE;

	//printk("head_addr=0x%05X, dummy_len=0x%05X, data_len=0x%05X, residual_len=0x%05X\n", head_addr, dummy_len, data_len, residual_len);

	//read xdata : step 1
	for (i = 0; i < ((dummy_len + data_len) / XDATA_SECTOR_SIZE); i++) {
		//---change xdata index---
		nvt_set_page(head_addr + XDATA_SECTOR_SIZE * i);

		//---read xdata by BUS_TRANSFER_LENGTH
		for (j = 0; j < (XDATA_SECTOR_SIZE / BUS_TRANSFER_LENGTH); j++) {
			//---read data---
			buf[0] = BUS_TRANSFER_LENGTH * j;
			CTP_SPI_READ(ts->client, buf, BUS_TRANSFER_LENGTH + 1);

			//---copy buf to xdata_tmp---
			for (k = 0; k < BUS_TRANSFER_LENGTH; k++) {
				xdata_tmp[XDATA_SECTOR_SIZE * i + BUS_TRANSFER_LENGTH * j + k] = buf[k + 1];
				//printk("0x%02X, 0x%04X\n", buf[k+1], (XDATA_SECTOR_SIZE*i + BUS_TRANSFER_LENGTH*j + k));
			}
		}
		//printk("addr=0x%05X\n", (head_addr+XDATA_SECTOR_SIZE*i));
	}

	//read xdata : step2
	if (residual_len != 0) {
		//---change xdata index---
		nvt_set_page(xdata_addr + data_len - residual_len);

		//---read xdata by BUS_TRANSFER_LENGTH
		for (j = 0; j < (residual_len / BUS_TRANSFER_LENGTH + 1); j++) {
			//---read data---
			buf[0] = BUS_TRANSFER_LENGTH * j;
			CTP_SPI_READ(ts->client, buf, BUS_TRANSFER_LENGTH + 1);

			//---copy buf to xdata_tmp---
			for (k = 0; k < BUS_TRANSFER_LENGTH; k++) {
				xdata_tmp[(dummy_len + data_len - residual_len) + BUS_TRANSFER_LENGTH * j + k] = buf[k + 1];
				//printk("0x%02X, 0x%04x\n", buf[k+1], ((dummy_len+data_len-residual_len) + BUS_TRANSFER_LENGTH*j + k));
			}
		}
		//printk("addr=0x%05X\n", (xdata_addr+data_len-residual_len));
	}

	//---remove dummy data and 2bytes-to-1data---
	for (i = 0; i < (data_len / 2); i++) {
		xdata[i] = (int16_t)(xdata_tmp[dummy_len + i * 2] + 256 * xdata_tmp[dummy_len + i * 2 + 1]);
	}

#if TOUCH_KEY_NUM > 0
	//read button xdata : step3
	//---change xdata index---
	nvt_set_page(xdata_btn_addr);

	//---read data---
	buf[0] = (xdata_btn_addr & 0xFF);
	CTP_SPI_READ(ts->client, buf, (TOUCH_KEY_NUM * 2 + 1));

	//---2bytes-to-1data---
	for (i = 0; i < TOUCH_KEY_NUM; i++) {
		xdata[ts->x_num * ts->y_num + i] = (int16_t)(buf[1 + i * 2] + 256 * buf[1 + i * 2 + 1]);
	}
#endif

	//---set xdata index to EVENT BUF ADDR---
	nvt_set_page(ts->mmap->EVENT_BUF_ADDR);
}

/*******************************************************
Description:
    Novatek touchscreen get meta data function.

return:
    n.a.
*******************************************************/
void nvt_get_mdata(int32_t *buf, uint8_t *m_x_num, uint8_t *m_y_num)
{
    *m_x_num = ts->x_num;
    *m_y_num = ts->y_num;
    memcpy(buf, xdata, ((ts->x_num * ts->y_num + TOUCH_KEY_NUM) * sizeof(int32_t)));
}

/*******************************************************
Description:
	Novatek touchscreen read and get number of meta data function.

return:
	n.a.
*******************************************************/
void nvt_read_get_num_mdata(uint32_t xdata_addr, int32_t *buffer, uint32_t num)
{
	int32_t i = 0;
	int32_t j = 0;
	int32_t k = 0;
	uint8_t buf[BUS_TRANSFER_LENGTH + 2] = {0};
	uint32_t head_addr = 0;
	int32_t dummy_len = 0;
	int32_t data_len = 0;
	int32_t residual_len = 0;

	//---set xdata sector address & length---
	head_addr = xdata_addr - (xdata_addr % XDATA_SECTOR_SIZE);
	dummy_len = xdata_addr - head_addr;
	data_len = num * 2;
	residual_len = (head_addr + dummy_len + data_len) % XDATA_SECTOR_SIZE;

	//printk("head_addr=0x%05X, dummy_len=0x%05X, data_len=0x%05X, residual_len=0x%05X\n", head_addr, dummy_len, data_len, residual_len);

	//read xdata : step 1
	for (i = 0; i < ((dummy_len + data_len) / XDATA_SECTOR_SIZE); i++) {
		//---change xdata index---
		nvt_set_page(head_addr + XDATA_SECTOR_SIZE * i);

		//---read xdata by BUS_TRANSFER_LENGTH
		for (j = 0; j < (XDATA_SECTOR_SIZE / BUS_TRANSFER_LENGTH); j++) {
			//---read data---
			buf[0] = BUS_TRANSFER_LENGTH * j;
			CTP_SPI_READ(ts->client, buf, BUS_TRANSFER_LENGTH + 1);

			//---copy buf to xdata_tmp---
			for (k = 0; k < BUS_TRANSFER_LENGTH; k++) {
				xdata_tmp[XDATA_SECTOR_SIZE * i + BUS_TRANSFER_LENGTH * j + k] = buf[k + 1];
				//printk("0x%02X, 0x%04X\n", buf[k+1], (XDATA_SECTOR_SIZE*i + BUS_TRANSFER_LENGTH*j + k));
			}
		}
		//printk("addr=0x%05X\n", (head_addr+XDATA_SECTOR_SIZE*i));
	}

	//read xdata : step2
	if (residual_len != 0) {
		//---change xdata index---
		nvt_set_page(xdata_addr + data_len - residual_len);

		//---read xdata by BUS_TRANSFER_LENGTH
		for (j = 0; j < (residual_len / BUS_TRANSFER_LENGTH + 1); j++) {
			//---read data---
			buf[0] = BUS_TRANSFER_LENGTH * j;
			CTP_SPI_READ(ts->client, buf, BUS_TRANSFER_LENGTH + 1);

			//---copy buf to xdata_tmp---
			for (k = 0; k < BUS_TRANSFER_LENGTH; k++) {
				xdata_tmp[(dummy_len + data_len - residual_len) + BUS_TRANSFER_LENGTH * j + k] = buf[k + 1];
				//printk("0x%02X, 0x%04x\n", buf[k+1], ((dummy_len+data_len-residual_len) + BUS_TRANSFER_LENGTH*j + k));
			}
		}
		//printk("addr=0x%05X\n", (xdata_addr+data_len-residual_len));
	}

	//---remove dummy data and 2bytes-to-1data---
	for (i = 0; i < (data_len / 2); i++) {
		buffer[i] = (int16_t)(xdata_tmp[dummy_len + i * 2] + 256 * xdata_tmp[dummy_len + i * 2 + 1]);
	}

	//---set xdata index to EVENT BUF ADDR---
	nvt_set_page(ts->mmap->EVENT_BUF_ADDR);
}

/*******************************************************
Description:
	Novatek touchscreen firmware version show function.

return:
	Executive outcomes. 0---succeed.
*******************************************************/
static int32_t c_fw_version_show(struct seq_file *m, void *v)
{
	seq_printf(m, "fw_ver=%d, x_num=%d, y_num=%d, button_num=%d\n", ts->fw_ver, ts->x_num, ts->y_num, ts->max_button_num);
	return 0;
}

/*******************************************************
Description:
	Novatek touchscreen xdata sequence print show
	function.

return:
	Executive outcomes. 0---succeed.
*******************************************************/
static int32_t c_show(struct seq_file *m, void *v)
{
	int32_t i = 0;
	int32_t j = 0;

	for (i = 0; i < ts->y_num; i++) {
		for (j = 0; j < ts->x_num; j++) {
			seq_printf(m, "%5d, ", xdata[i * ts->x_num + j]);
		}
		seq_puts(m, "\n");
	}

#if TOUCH_KEY_NUM > 0
	for (i = 0; i < TOUCH_KEY_NUM; i++) {
		seq_printf(m, "%5d, ", xdata[ts->x_num * ts->y_num + i]);
	}
	seq_puts(m, "\n");
#endif

	seq_printf(m, "\n\n");
	return 0;
}

/*******************************************************
Description:
	Novatek pen 1D diff xdata sequence print show
	function.

return:
	Executive outcomes. 0---succeed.
*******************************************************/
static int32_t c_pen_1d_diff_show(struct seq_file *m, void *v)
{
	int32_t i = 0;

	seq_printf(m, "Tip X:\n");
	for (i = 0; i < ts->x_num; i++) {
		seq_printf(m, "%5d, ", xdata_pen_tip_x[i]);
	}
	seq_puts(m, "\n");
	seq_printf(m, "Tip Y:\n");
	for (i = 0; i < ts->y_num; i++) {
		seq_printf(m, "%5d, ", xdata_pen_tip_y[i]);
	}
	seq_puts(m, "\n");
	seq_printf(m, "Ring X:\n");
	for (i = 0; i < ts->x_num; i++) {
		seq_printf(m, "%5d, ", xdata_pen_ring_x[i]);
	}
	seq_puts(m, "\n");
	seq_printf(m, "Ring Y:\n");
	for (i = 0; i < ts->y_num; i++) {
		seq_printf(m, "%5d, ", xdata_pen_ring_y[i]);
	}
	seq_puts(m, "\n");

	seq_printf(m, "\n\n");
	return 0;
}

/*******************************************************
Description:
	Novatek touchscreen xdata sequence print start
	function.

return:
	Executive outcomes. 1---call next function.
	NULL---not call next function and sequence loop
	stop.
*******************************************************/
static void *c_start(struct seq_file *m, loff_t *pos)
{
	return *pos < 1 ? (void *)1 : NULL;
}

/*******************************************************
Description:
	Novatek touchscreen xdata sequence print next
	function.

return:
	Executive outcomes. NULL---no next and call sequence
	stop function.
*******************************************************/
static void *c_next(struct seq_file *m, void *v, loff_t *pos)
{
	++*pos;
	return NULL;
}

/*******************************************************
Description:
	Novatek touchscreen xdata sequence print stop
	function.

return:
	n.a.
*******************************************************/
static void c_stop(struct seq_file *m, void *v)
{
	return;
}

const struct seq_operations nvt_fw_version_seq_ops = {
	.start  = c_start,
	.next   = c_next,
	.stop   = c_stop,
	.show   = c_fw_version_show
};

const struct seq_operations nvt_seq_ops = {
	.start  = c_start,
	.next   = c_next,
	.stop   = c_stop,
	.show   = c_show
};

const struct seq_operations nvt_pen_diff_seq_ops = {
	.start  = c_start,
	.next   = c_next,
	.stop   = c_stop,
	.show   = c_pen_1d_diff_show
};

/*******************************************************
Description:
	Novatek touchscreen /proc/nvt_fw_version open
	function.

return:
	n.a.
*******************************************************/
static int32_t nvt_fw_version_open(struct inode *inode, struct file *file)
{
	if (mutex_lock_interruptible(&ts->lock)) {
		return -ERESTARTSYS;
	}

	NVT_LOG("++\n");

#if NVT_TOUCH_ESD_PROTECT
	nvt_esd_check_enable(false);
#endif /* #if NVT_TOUCH_ESD_PROTECT */

	if (nvt_get_fw_info()) {
		mutex_unlock(&ts->lock);
		return -EAGAIN;
	}

	mutex_unlock(&ts->lock);

	NVT_LOG("--\n");

	return seq_open(file, &nvt_fw_version_seq_ops);
}

static const struct file_operations nvt_fw_version_fops = {
	.owner = THIS_MODULE,
	.open = nvt_fw_version_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release,
};

/*******************************************************
Description:
	Novatek touchscreen /proc/nvt_baseline open function.

return:
	Executive outcomes. 0---succeed.
*******************************************************/
static int32_t nvt_baseline_open(struct inode *inode, struct file *file)
{
	if (mutex_lock_interruptible(&ts->lock)) {
		return -ERESTARTSYS;
	}

	NVT_LOG("++\n");

#if NVT_TOUCH_ESD_PROTECT
	nvt_esd_check_enable(false);
#endif /* #if NVT_TOUCH_ESD_PROTECT */

	if (nvt_clear_fw_status()) {
		mutex_unlock(&ts->lock);
		return -EAGAIN;
	}

	nvt_change_mode(TEST_MODE_2);

	if (nvt_check_fw_status()) {
		mutex_unlock(&ts->lock);
		return -EAGAIN;
	}

	if (nvt_get_fw_info()) {
		mutex_unlock(&ts->lock);
		return -EAGAIN;
	}

	nvt_read_mdata(ts->mmap->BASELINE_ADDR, ts->mmap->BASELINE_BTN_ADDR);

	nvt_change_mode(NORMAL_MODE);

	mutex_unlock(&ts->lock);

	NVT_LOG("--\n");

	return seq_open(file, &nvt_seq_ops);
}

static const struct file_operations nvt_baseline_fops = {
	.owner = THIS_MODULE,
	.open = nvt_baseline_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release,
};

/*******************************************************
Description:
	Novatek touchscreen /proc/nvt_raw open function.

return:
	Executive outcomes. 0---succeed.
*******************************************************/
static int32_t nvt_raw_open(struct inode *inode, struct file *file)
{
	if (mutex_lock_interruptible(&ts->lock)) {
		return -ERESTARTSYS;
	}

	NVT_LOG("++\n");

#if NVT_TOUCH_ESD_PROTECT
	nvt_esd_check_enable(false);
#endif /* #if NVT_TOUCH_ESD_PROTECT */

	if (nvt_clear_fw_status()) {
		mutex_unlock(&ts->lock);
		return -EAGAIN;
	}

	nvt_change_mode(TEST_MODE_2);

	if (nvt_check_fw_status()) {
		mutex_unlock(&ts->lock);
		return -EAGAIN;
	}

	if (nvt_get_fw_info()) {
		mutex_unlock(&ts->lock);
		return -EAGAIN;
	}

	if (nvt_get_fw_pipe() == 0)
		nvt_read_mdata(ts->mmap->RAW_PIPE0_ADDR, ts->mmap->RAW_BTN_PIPE0_ADDR);
	else
		nvt_read_mdata(ts->mmap->RAW_PIPE1_ADDR, ts->mmap->RAW_BTN_PIPE1_ADDR);

	nvt_change_mode(NORMAL_MODE);

	mutex_unlock(&ts->lock);

	NVT_LOG("--\n");

	return seq_open(file, &nvt_seq_ops);
}

static const struct file_operations nvt_raw_fops = {
	.owner = THIS_MODULE,
	.open = nvt_raw_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release,
};

/*******************************************************
Description:
	Novatek touchscreen /proc/nvt_diff open function.

return:
	Executive outcomes. 0---succeed. negative---failed.
*******************************************************/
static int32_t nvt_diff_open(struct inode *inode, struct file *file)
{
	if (mutex_lock_interruptible(&ts->lock)) {
		return -ERESTARTSYS;
	}

	NVT_LOG("++\n");

#if NVT_TOUCH_ESD_PROTECT
	nvt_esd_check_enable(false);
#endif /* #if NVT_TOUCH_ESD_PROTECT */

	if (nvt_clear_fw_status()) {
		mutex_unlock(&ts->lock);
		return -EAGAIN;
	}

	nvt_change_mode(TEST_MODE_2);

	if (nvt_check_fw_status()) {
		mutex_unlock(&ts->lock);
		return -EAGAIN;
	}

	if (nvt_get_fw_info()) {
		mutex_unlock(&ts->lock);
		return -EAGAIN;
	}

	if (nvt_get_fw_pipe() == 0)
		nvt_read_mdata(ts->mmap->DIFF_PIPE0_ADDR, ts->mmap->DIFF_BTN_PIPE0_ADDR);
	else
		nvt_read_mdata(ts->mmap->DIFF_PIPE1_ADDR, ts->mmap->DIFF_BTN_PIPE1_ADDR);

	nvt_change_mode(NORMAL_MODE);

	mutex_unlock(&ts->lock);

	NVT_LOG("--\n");

	return seq_open(file, &nvt_seq_ops);
}

static const struct file_operations nvt_diff_fops = {
	.owner = THIS_MODULE,
	.open = nvt_diff_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release,
};

/*******************************************************
Description:
	Novatek touchscreen /proc/nvt_pen_diff open function.

return:
	Executive outcomes. 0---succeed. negative---failed.
*******************************************************/
static int32_t nvt_pen_diff_open(struct inode *inode, struct file *file)
{
	if (mutex_lock_interruptible(&ts->lock)) {
		return -ERESTARTSYS;
	}

	NVT_LOG("++\n");

#if NVT_TOUCH_ESD_PROTECT
	nvt_esd_check_enable(false);
#endif /* #if NVT_TOUCH_ESD_PROTECT */

	if (nvt_set_pen_inband_mode_1(0xFF, 0x00)) {
		mutex_unlock(&ts->lock);
		return -EAGAIN;
	}

	if (nvt_check_fw_reset_state(RESET_STATE_NORMAL_RUN)) {
		mutex_unlock(&ts->lock);
		return -EAGAIN;
	}

	if (nvt_clear_fw_status()) {
		mutex_unlock(&ts->lock);
		return -EAGAIN;
	}

	nvt_change_mode(TEST_MODE_2);

	if (nvt_check_fw_status()) {
		mutex_unlock(&ts->lock);
		return -EAGAIN;
	}

	if (nvt_get_fw_info()) {
		mutex_unlock(&ts->lock);
		return -EAGAIN;
	}

	nvt_read_get_num_mdata(ts->mmap->PEN_1D_DIFF_TIP_X_ADDR, xdata_pen_tip_x, ts->x_num);
	nvt_read_get_num_mdata(ts->mmap->PEN_1D_DIFF_TIP_Y_ADDR, xdata_pen_tip_y, ts->y_num);
	nvt_read_get_num_mdata(ts->mmap->PEN_1D_DIFF_RING_X_ADDR, xdata_pen_ring_x, ts->x_num);
	nvt_read_get_num_mdata(ts->mmap->PEN_1D_DIFF_RING_Y_ADDR, xdata_pen_ring_y, ts->y_num);

	nvt_change_mode(NORMAL_MODE);

	nvt_set_pen_normal_mode();

	nvt_check_fw_reset_state(RESET_STATE_NORMAL_RUN);

	mutex_unlock(&ts->lock);

	NVT_LOG("--\n");

	return seq_open(file, &nvt_pen_diff_seq_ops);
}

static const struct file_operations nvt_pen_diff_fops = {
	.owner = THIS_MODULE,
	.open = nvt_pen_diff_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release,
};

//+OAK4146,shenwenbin.wt,ADD,20220125,add TP game mode
int32_t nvt_cmd_store(uint8_t u8Cmd){
	int i, retry = 5;
	uint8_t buf[3] = {0};
	int32_t ret = 0;

	if (mutex_lock_interruptible(&ts->lock)) {
		return -ERESTARTSYS;
	}

	//---set xdata index to EVENT BUF ADDR---
	ret = nvt_set_page(ts->mmap->EVENT_BUF_ADDR | EVENT_MAP_HOST_CMD);
	if (ret < 0) {
		NVT_ERR("Set event buffer index fail!\n");
		mutex_unlock(&ts->lock);
		return ret;
	}

	for (i = 0; i < retry; i++) {
		//---set cmd status---
		buf[0] = EVENT_MAP_HOST_CMD;
		buf[1] = u8Cmd;
		CTP_SPI_WRITE(ts->client, buf, 2);
		msleep(40);
		//---read cmd status---
		buf[0] = EVENT_MAP_HOST_CMD;
		buf[1] = 0xFF;
		CTP_SPI_READ(ts->client, buf, 2);
		if (buf[1] == 0x00)
			break;
	}

	if (unlikely(i == retry)) {
		NVT_LOG("send Cmd 0x%02X failed, buf[1]=0x%02X\n", u8Cmd, buf[1]);
		ret = -1;
	} else {
		NVT_LOG("send Cmd 0x%02X success, tried %d times\n", u8Cmd, i);
	}

	mutex_unlock(&ts->lock);

	return ret;
}

int32_t nvt_ext_cmd_store(uint8_t u8Cmd, uint8_t u8subCmd){
	int i, retry = 5;
	uint8_t buf[4] = {0};
	int32_t ret = 0;

	if (mutex_lock_interruptible(&ts->lock)) {
		return -ERESTARTSYS;
	}

	//---set xdata index to EVENT BUF ADDR---
	ret = nvt_set_page(ts->mmap->EVENT_BUF_ADDR | EVENT_MAP_HOST_CMD);
	if (ret < 0) {
		NVT_ERR("Set event buffer index fail!\n");
		mutex_unlock(&ts->lock);
		return ret;
	}

	for (i = 0; i < retry; i++) {
		//---set cmd status---
		buf[0] = EVENT_MAP_HOST_CMD;
		buf[1] = u8Cmd;
		buf[2] = u8subCmd;
		CTP_SPI_WRITE(ts->client, buf, 3);

		msleep(40);

		//---read cmd status---
		buf[0] = EVENT_MAP_HOST_CMD;
		buf[1] = 0xFF;
		CTP_SPI_READ(ts->client, buf, 2);
		if (buf[1] == 0x00)
			break;
	}

	if (unlikely(i == retry)) {
		NVT_LOG("send Cmd 0x%02X 0x%02X failed, buf[1]=0x%02X\n", u8Cmd, u8subCmd, buf[1]);
		ret = -1;
	} else {
		NVT_LOG("send Cmd 0x%02X 0x%02X success, tried %d times\n", u8Cmd, u8subCmd, i);
	}

	mutex_unlock(&ts->lock);

	return ret;
}

int32_t nvt_edge_reject_set(int32_t status) {
	int ret = 0;

	if(status == 1)//rotate 0/180 degree
		ret = nvt_cmd_store(EDGE_REJECT_VERTICLE_CMD);
	else if(status == 2) //rotate 90 degree
		ret = nvt_cmd_store(EDGE_REJECT_LEFT_UP);
	else if(status == 3) //rotate 270 degree
		ret = nvt_cmd_store(EDGE_REJECT_RIGHT_UP);
	else
		NVT_LOG("Not Support state %d!\n", status);

	return ret;
}

static ssize_t nvt_edge_reject_store(struct file *file, const char *buffer, size_t count, loff_t *pos) {
	char dbg[10] = { 0 };
	int res = 0;
	uint8_t state;
	
	res = copy_from_user(dbg, (uint8_t *) buffer, sizeof(uint8_t));
	if (res)
		return -EINVAL;

	res = kstrtou8(dbg, 16, &state);
	if (res < 0)
		return res;

	ts->edge_reject_state = state;
		
	nvt_edge_reject_set(ts->edge_reject_state);

	return count;
}

static int nvt_edge_reject_show(struct seq_file *sfile, void *v) {
	uint8_t buf[4] = {0};

	//---set xdata index to EVENT BUF ADDR---
	nvt_set_page(ts->mmap->EVENT_BUF_ADDR);

	//---read cmd status---
	buf[0] = 0x5C;
	buf[1] = 0xFF;
	CTP_SPI_READ(ts->client, buf, 2);
	
	ts->edge_reject_state = ((buf[1] >> EDGE_REJECT_L) & 0x03);
	
	if(ts->edge_reject_state == 1)
		seq_printf(sfile, "Vertical Direction!\n");
	else if(ts->edge_reject_state == 2)
		seq_printf(sfile, "Left Up Direction!\n");
	else if(ts->edge_reject_state == 3)
		seq_printf(sfile, "Right Up Direction!\n");
	else
		seq_printf(sfile, "Not Support!\n");

	return 0;
}

static int32_t nvt_edge_reject_open(struct inode *inode, struct file *file) {
	return single_open(file, nvt_edge_reject_show, NULL);
}
static const struct file_operations nvt_edge_reject_fops = {
	.owner = THIS_MODULE,
	.open = nvt_edge_reject_open,
	.read = seq_read,
	.write = nvt_edge_reject_store,
	.llseek = seq_lseek,
	.release = seq_release,
};

int32_t nvt_game_mode_set(uint8_t status) {
	int ret = 0;

	if(status == 1)
		ret = nvt_ext_cmd_store(GAME_MODE_PALM_CMD, status);
	else if(status == 0)
		ret = nvt_ext_cmd_store(GAME_MODE_PALM_CMD, status);
	else
		NVT_LOG("Not Support state %d!\n", status);

	return ret;
}
static ssize_t nvt_game_mode_store(struct file *file, const char *buffer, size_t count, loff_t *pos) {
	char dbg[10] = { 0 };
	uint8_t state;
	uint8_t ret;
	
	ret = copy_from_user(dbg, (uint8_t *) buffer, sizeof(uint8_t));
	if (ret)
		return -EINVAL;

	ret = kstrtou8(dbg, 16, &state);
	if (ret < 0)
		return ret;

	NVT_LOG("moto_apk_state %d!\n", state);
	ts->game_mode_state = state;
	nvt_game_mode_set(ts->game_mode_state);

	return count;
}
static int nvt_game_mode_show(struct seq_file *sfile, void *v) {
	seq_printf(sfile, "Game Mode state %d!\n", ts->game_mode_state);
	return 0;
}
static int32_t nvt_game_mode_open(struct inode *inode, struct file *file) {
	return single_open(file, nvt_game_mode_show, NULL);
}
static const struct file_operations nvt_game_mode_fops = {
	.owner = THIS_MODULE,
	.open = nvt_game_mode_open,
	.read = seq_read,
	.write = nvt_game_mode_store,
	.llseek = seq_lseek,
	.release = seq_release,
};

/*
 *	cmd_param
 *      [0], 1, 0 degree
 *			 2, 90 degree
 *			 3, 180 degree
 *			 4, 270 degree
 *
 *		[1], 1, left side
 *		     2, right side
 *
 *		[2], start Y coordinate
 *
 *		[3], end Y coordinate 
 *
 */
int32_t nvt_edge_grid_zone_set(void) {
	int ret = 0;
	uint8_t buf[8] = {0};
	int i, retry = 5;

	if (mutex_lock_interruptible(&ts->lock)) {
		return -ERESTARTSYS;
	}

	//---set xdata index to EVENT BUF ADDR---
	ret = nvt_set_page(ts->mmap->EVENT_BUF_ADDR | EVENT_MAP_HOST_CMD);
	if (ret < 0) {
		NVT_ERR("Set event buffer index fail!\n");
		mutex_unlock(&ts->lock);
		return ret;
	}

	for (i = 0; i < retry; i++) {
		//---set cmd status---
		buf[0] = EVENT_MAP_HOST_CMD;
		buf[1] = EXTEND_CUSTOMIZED_CMD;
		buf[2] = (uint8_t)(ts->edge_cmd_param[0] & 0xFF);
		buf[3] = (uint8_t)(ts->edge_cmd_param[1] & 0xFF);
		buf[4] = (uint8_t)(ts->edge_cmd_param[2] & 0xFF);
		buf[5] = (uint8_t)((ts->edge_cmd_param[2] >> 8) & 0xFF);
		buf[6] = (uint8_t)(ts->edge_cmd_param[3] & 0xFF);
		buf[7] = (uint8_t)((ts->edge_cmd_param[3] >> 8) & 0xFF);
		CTP_SPI_WRITE(ts->client, buf, 8);

		msleep(40);

		//---read cmd status---
		buf[0] = EVENT_MAP_HOST_CMD;
		buf[1] = 0xFF;
		CTP_SPI_READ(ts->client, buf, 2);
		if (buf[1] == 0x00)
			break;
	}

	if (unlikely(i == retry)) {
		NVT_LOG("send cmd 7F,%02X,%02X,%02X,%02X,%02X,%02X failed, buf[1]=0x%02X\n",  buf[2], buf[3], buf[4], buf[5], buf[6], buf[7], buf[1]);
		ret = -1;
	} else {
		NVT_LOG("send cmd 7F,%02X,%02X,%02X,%02X,%02X,%02X success\n", buf[2], buf[3], buf[4], buf[5], buf[6], buf[7]);
	}

	mutex_unlock(&ts->lock);


	return ret;
}
static ssize_t nvt_edge_grid_zone_store(struct file *file, const char *buffer, size_t count, loff_t *pos) {
	char *tmp_buf;
	int ret = 0;

	NVT_LOG("++\n");

	tmp_buf = kzalloc((count+1), GFP_KERNEL);
	if (!tmp_buf) {
		NVT_ERR("Allocate tmp_buf fail!\n");
		ret = -ENOMEM;
		goto out;
	}

	if (copy_from_user(tmp_buf, buffer, count)) {
		NVT_ERR("copy_from_user() error!\n");
		ret=  -EINVAL;
		goto out;
	}

	ret = sscanf(tmp_buf, "%d,%d,%d,%d", ts->edge_cmd_param, ts->edge_cmd_param+1, ts->edge_cmd_param+2, ts->edge_cmd_param+3);

	NVT_LOG("cmd_parm = %d,%d,%d,%d\n", ts->edge_cmd_param[0], ts->edge_cmd_param[1], ts->edge_cmd_param[2], ts->edge_cmd_param[3]);

	ret = nvt_edge_grid_zone_set();
	if(ret == 0) {
		ret = count;
	}

out:
	if (tmp_buf)
		kfree(tmp_buf);

	NVT_LOG("--\n");

	return ret;
}
static int nvt_edge_grid_zone_show(struct seq_file *sfile, void *v) {
	
	seq_printf(sfile, "cmd_parm = %d,%d,%d,%d\n", ts->edge_cmd_param[0], ts->edge_cmd_param[1], ts->edge_cmd_param[2], ts->edge_cmd_param[3]);
	return 0;
}
static int32_t nvt_edge_grid_zone_open(struct inode *inode, struct file *file) {
	return single_open(file, nvt_edge_grid_zone_show, NULL);
}
static const struct file_operations nvt_edge_grid_zone_fops = {
	.owner = THIS_MODULE,
	.open = nvt_edge_grid_zone_open,
	.read = seq_read,
	.write = nvt_edge_grid_zone_store,
	.llseek = seq_lseek,
	.release = seq_release,
};
//-OAK4146,shenwenbin.wt,ADD,20220125,add TP game mode

/*******************************************************
Description:
	Novatek touchscreen extra function proc. file node
	initial function.

return:
	Executive outcomes. 0---succeed. -12---failed.
*******************************************************/
int32_t nvt_extra_proc_init(void)
{
	NVT_proc_fw_version_entry = proc_create(NVT_FW_VERSION, 0444, NULL,&nvt_fw_version_fops);
	if (NVT_proc_fw_version_entry == NULL) {
		NVT_ERR("create proc/%s Failed!\n", NVT_FW_VERSION);
		return -ENOMEM;
	} else {
		NVT_LOG("create proc/%s Succeeded!\n", NVT_FW_VERSION);
	}

	NVT_proc_baseline_entry = proc_create(NVT_BASELINE, 0444, NULL,&nvt_baseline_fops);
	if (NVT_proc_baseline_entry == NULL) {
		NVT_ERR("create proc/%s Failed!\n", NVT_BASELINE);
		return -ENOMEM;
	} else {
		NVT_LOG("create proc/%s Succeeded!\n", NVT_BASELINE);
	}

	NVT_proc_raw_entry = proc_create(NVT_RAW, 0444, NULL,&nvt_raw_fops);
	if (NVT_proc_raw_entry == NULL) {
		NVT_ERR("create proc/%s Failed!\n", NVT_RAW);
		return -ENOMEM;
	} else {
		NVT_LOG("create proc/%s Succeeded!\n", NVT_RAW);
	}

	NVT_proc_diff_entry = proc_create(NVT_DIFF, 0444, NULL,&nvt_diff_fops);
	if (NVT_proc_diff_entry == NULL) {
		NVT_ERR("create proc/%s Failed!\n", NVT_DIFF);
		return -ENOMEM;
	} else {
		NVT_LOG("create proc/%s Succeeded!\n", NVT_DIFF);
	}

	if (ts->pen_support) {
		NVT_proc_pen_diff_entry = proc_create(NVT_PEN_DIFF, 0444, NULL,&nvt_pen_diff_fops);
		if (NVT_proc_pen_diff_entry == NULL) {
			NVT_ERR("create proc/%s Failed!\n", NVT_PEN_DIFF);
			return -ENOMEM;
		} else {
			NVT_LOG("create proc/%s Succeeded!\n", NVT_PEN_DIFF);
		}
	}
	//+OAK700,shenwenbin.wt,add,20211204,add charge mode
	NVT_proc_edge_reject_entry = proc_create(PANEL_DIRECTION, 0666, NULL, &nvt_edge_reject_fops);	//OAK700,shenwenbin.wt,modify,20211227,modify panel direction black display issues
	if (NVT_proc_edge_reject_entry == NULL) {
		NVT_ERR("create proc/%s Failed!\n", PANEL_DIRECTION);
		return -ENOMEM;
	} else {
		NVT_LOG("create proc/%s Succeeded!\n", PANEL_DIRECTION);
	}
	//-OAK700,shenwenbin.wt,add,20211204,add charge mode

	//+OAK4146,shenwenbin.wt,ADD,20220125,add TP game mode
	NVT_proc_game_mode_entry = proc_create(GAME_MODE_NOTIFY, 0666, NULL, &nvt_game_mode_fops);
	if (NVT_proc_game_mode_entry == NULL) {
		NVT_ERR("create proc/game_mode Failed!\n");
		return -ENOMEM;
	} else {
		NVT_LOG("create proc/game_mode Succeeded!\n");
	} 

	NVT_proc_edge_grid_zone_entry = proc_create(EDGE_GRID_ZONE, 0666, NULL, &nvt_edge_grid_zone_fops);
	if (NVT_proc_edge_grid_zone_entry == NULL) {
		NVT_ERR("create proc/%s Failed!\n", EDGE_GRID_ZONE);
		return -ENOMEM;
	} else {
		NVT_LOG("create proc/%s Succeeded!\n", EDGE_GRID_ZONE);
	} 
	//-OAK4146,shenwenbin.wt,ADD,20220125,add TP game mode

	return 0;
}

/*******************************************************
Description:
	Novatek touchscreen extra function proc. file node
	deinitial function.

return:
	n.a.
*******************************************************/
void nvt_extra_proc_deinit(void)
{
	if (NVT_proc_fw_version_entry != NULL) {
		remove_proc_entry(NVT_FW_VERSION, NULL);
		NVT_proc_fw_version_entry = NULL;
		NVT_LOG("Removed /proc/%s\n", NVT_FW_VERSION);
	}

	if (NVT_proc_baseline_entry != NULL) {
		remove_proc_entry(NVT_BASELINE, NULL);
		NVT_proc_baseline_entry = NULL;
		NVT_LOG("Removed /proc/%s\n", NVT_BASELINE);
	}

	if (NVT_proc_raw_entry != NULL) {
		remove_proc_entry(NVT_RAW, NULL);
		NVT_proc_raw_entry = NULL;
		NVT_LOG("Removed /proc/%s\n", NVT_RAW);
	}

	if (NVT_proc_diff_entry != NULL) {
		remove_proc_entry(NVT_DIFF, NULL);
		NVT_proc_diff_entry = NULL;
		NVT_LOG("Removed /proc/%s\n", NVT_DIFF);
	}

	if (ts->pen_support) {
		if (NVT_proc_pen_diff_entry != NULL) {
			remove_proc_entry(NVT_PEN_DIFF, NULL);
			NVT_proc_pen_diff_entry = NULL;
			NVT_LOG("Removed /proc/%s\n", NVT_PEN_DIFF);
		}
	}
	//+OAK700,shenwenbin.wt,add,20211204,add charge mode
	if (NVT_proc_edge_reject_entry != NULL) {
		remove_proc_entry(PANEL_DIRECTION, NULL);
		NVT_proc_edge_reject_entry = NULL;
		NVT_LOG("Removed /proc/%s \n", PANEL_DIRECTION);
	}
	//-OAK700,shenwenbin.wt,add,20211204,add charge mode

	//+OAK4146,shenwenbin.wt,ADD,20220125,add TP game mode
	if (NVT_proc_game_mode_entry != NULL) {
		remove_proc_entry(GAME_MODE_NOTIFY, NULL);
		NVT_proc_game_mode_entry = NULL;
		NVT_LOG("Removed /proc/%s\n", GAME_MODE_NOTIFY);
	}
	
	if (NVT_proc_edge_grid_zone_entry != NULL) {
		remove_proc_entry(EDGE_GRID_ZONE, NULL);
		NVT_proc_edge_grid_zone_entry = NULL;
		NVT_LOG("Removed /proc/%s \n", EDGE_GRID_ZONE);
	}
	//-OAK4146,shenwenbin.wt,ADD,20220125,add TP game mode
	
}
#endif
