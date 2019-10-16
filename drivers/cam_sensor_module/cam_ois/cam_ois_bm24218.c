/* Copyright (c) 2019, The Linux Foundation. All rights reserved.
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

#include <linux/module.h>
#include <linux/firmware.h>
#include <linux/dma-contiguous.h>
#include <cam_sensor_cmn_header.h>
#include "cam_ois_core.h"
#include "cam_ois_soc.h"
#include "cam_sensor_util.h"
#include "cam_debug_util.h"
#include "cam_res_mgr_api.h"
#include "cam_common_util.h"

#define CCI_TXN_MAX_DATA_SIZE 200

int cam_ois_bm24218_start_dl(struct cam_ois_ctrl_t *o_ctrl)
{
	struct cam_sensor_i2c_reg_setting  i2c_reg_setting;
	struct cam_sensor_i2c_reg_array    i2c_reg_array;

	i2c_reg_array.reg_addr = 0xF010;
	i2c_reg_array.reg_data = 0x00;
	i2c_reg_array.data_mask = 0;
	i2c_reg_array.delay = 0;
	i2c_reg_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD;
	i2c_reg_setting.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	i2c_reg_setting.reg_setting = &i2c_reg_array;
	i2c_reg_setting.size = 1;
	i2c_reg_setting.delay = 1;
	return camera_io_dev_write(&o_ctrl->io_master_info, &i2c_reg_setting);
}

int cam_ois_bm24218_fw_checksum(struct cam_ois_ctrl_t *o_ctrl)
{
	uint32_t fwChecksum = 0;
	int32_t rc = 0;

	rc = camera_io_dev_read(&o_ctrl->io_master_info, 0xF008, &fwChecksum,
			CAMERA_SENSOR_I2C_TYPE_WORD,
			CAMERA_SENSOR_I2C_TYPE_DWORD);

	//TODO: Need actual check here with value from xml
	CAM_ERR(CAM_OIS, "OIS FW CHECKSUM: %x", fwChecksum);

	return rc;
}

int cam_ois_bm24218_fw_download(struct cam_ois_ctrl_t *o_ctrl)
{
	uint16_t                           total_bytes = 0;
	uint8_t                           *ptr = NULL;
	int32_t                            rc = 0, total_idx, packet_idx;
	uint32_t                           txn_data_size = CCI_TXN_MAX_DATA_SIZE;
	uint32_t                           txn_regsetting_size;
	const struct firmware             *fw = NULL;
	const char                        *fw_name_prog = NULL;
	const char                        *fw_name_coeff = NULL;
	char                               name_prog[32] = {0};
	char                               name_coeff[32] = {0};
	struct device                     *dev = &(o_ctrl->pdev->dev);
	struct cam_sensor_i2c_reg_setting  i2c_reg_setting;
	struct page                       *page = NULL;

	snprintf(name_coeff, 32, "%s.coeff", o_ctrl->ois_name);

	snprintf(name_prog, 32, "%s.prog", o_ctrl->ois_name);

	fw_name_prog = name_prog;
	fw_name_coeff = name_coeff;

	rc = request_firmware(&fw, fw_name_prog, dev);
	if (rc) {
		CAM_ERR(CAM_OIS, "Failed to locate %s", fw_name_prog);
		return rc;
	}

	total_bytes = fw->size;
	i2c_reg_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD;
	i2c_reg_setting.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	i2c_reg_setting.delay = 0;
	txn_regsetting_size = PAGE_ALIGN(sizeof(struct cam_sensor_i2c_reg_array) *
		txn_data_size) >> PAGE_SHIFT;
	page = cma_alloc(dev_get_cma_area((o_ctrl->soc_info.dev)),
		txn_regsetting_size, 0, GFP_KERNEL);
	if (!page) {
		CAM_ERR(CAM_OIS, "Failed in allocating i2c_array");
		release_firmware(fw);
		return -ENOMEM;
	}

	i2c_reg_setting.reg_setting = (struct cam_sensor_i2c_reg_array *) (
		page_address(page));
	/*
	 * Must split into smaller, valid txns or else cci driver will split
	 * into smaller, possibly invalid txns
	 */
	for (total_idx = 0, ptr = (uint8_t *)fw->data; total_idx < total_bytes;) {
		for(packet_idx = 0;
			packet_idx < txn_data_size && total_idx + packet_idx < total_bytes;
			packet_idx++, ptr++)
		{
			i2c_reg_setting.reg_setting[packet_idx].reg_addr =
				o_ctrl->opcode.prog + total_idx + packet_idx;
			i2c_reg_setting.reg_setting[packet_idx].reg_data = *ptr;
			i2c_reg_setting.reg_setting[packet_idx].delay = 0;
			i2c_reg_setting.reg_setting[packet_idx].data_mask = 0;
		}
		i2c_reg_setting.size = packet_idx;
		rc = camera_io_dev_write_continuous(&(o_ctrl->io_master_info),
			&i2c_reg_setting, 1);
		if (rc < 0) {
			CAM_ERR(CAM_OIS, "OIS FW download failed %d", rc);
			goto release_firmware;
		}
		total_idx += packet_idx;
	}

	cma_release(dev_get_cma_area((o_ctrl->soc_info.dev)),
		page, txn_regsetting_size);
	page = NULL;
	txn_regsetting_size = 0;
	release_firmware(fw);

	rc = request_firmware(&fw, fw_name_coeff, dev);
	if (rc) {
		CAM_ERR(CAM_OIS, "Failed to locate %s", fw_name_coeff);
		return rc;
	}

	total_bytes = fw->size;
	i2c_reg_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD;
	i2c_reg_setting.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	i2c_reg_setting.delay = 0;
	txn_regsetting_size = PAGE_ALIGN(sizeof(struct cam_sensor_i2c_reg_array) *
		txn_data_size) >> PAGE_SHIFT;
	page = cma_alloc(dev_get_cma_area((o_ctrl->soc_info.dev)),
		txn_regsetting_size, 0, GFP_KERNEL);
	if (!page) {
		CAM_ERR(CAM_OIS, "Failed in allocating i2c_array");
		release_firmware(fw);
		return -ENOMEM;
	}

	i2c_reg_setting.reg_setting = (struct cam_sensor_i2c_reg_array *) (
		page_address(page));

	for (total_idx = 0, ptr = (uint8_t *)fw->data; total_idx < total_bytes;) {
		for(packet_idx = 0;
			packet_idx < txn_data_size && total_idx + packet_idx < total_bytes;
			packet_idx++, ptr++)
		{
			i2c_reg_setting.reg_setting[packet_idx].reg_addr =
				o_ctrl->opcode.coeff + packet_idx + total_idx;
			i2c_reg_setting.reg_setting[packet_idx].reg_data = *ptr;
			i2c_reg_setting.reg_setting[packet_idx].delay = 0;
			i2c_reg_setting.reg_setting[packet_idx].data_mask = 0;
		}
		i2c_reg_setting.size = packet_idx;
		rc = camera_io_dev_write_continuous(&(o_ctrl->io_master_info),
			&i2c_reg_setting, 1);
		if (rc < 0) {
			CAM_ERR(CAM_OIS, "OIS FW download failed %d", rc);
			goto release_firmware;
		}
		total_idx += packet_idx;
	}

	rc = cam_ois_bm24218_fw_checksum(o_ctrl);
	if (rc < 0)
		CAM_ERR(CAM_OIS, "OIS FW checksum failed");

release_firmware:
	cma_release(dev_get_cma_area((o_ctrl->soc_info.dev)),
		page, txn_regsetting_size);
	release_firmware(fw);

	return rc;
}

//FIXME: implement calib data from OTP on valid EEPROM
int cam_ois_bm24218_calib(struct cam_ois_ctrl_t *o_ctrl) {
	struct cam_sensor_i2c_reg_setting  i2c_reg_setting;
	struct cam_sensor_i2c_reg_array    calib_reg_array[] = {
		{
			.reg_addr = 0x1DC0,
			.reg_data = 0x00,
			.data_mask = 0,
			.delay = 0,
		},
		{
			.reg_addr = 0x1DC1,
			.reg_data = 0x1B,
			.data_mask = 0,
			.delay = 0,
		},
		{
			.reg_addr = 0x1DC2,
			.reg_data = 0x00,
			.data_mask = 0,
			.delay = 0,
		},
		{
			.reg_addr = 0x1DC3,
			.reg_data = 0x1C,
			.data_mask = 0,
			.delay = 0,
		},
		{
			.reg_addr = 0x1DC4,
			.reg_data = 0x01,
			.data_mask = 0,
			.delay = 0,
		},
		{
			.reg_addr = 0x1DC5,
			.reg_data = 0xEE,
			.data_mask = 0,
			.delay = 0,
		},
		{
			.reg_addr = 0x1DC6,
			.reg_data = 0x01,
			.data_mask = 0,
			.delay = 0,
		},
		{
			.reg_addr = 0x1DC7,
			.reg_data = 0xD5,
			.data_mask = 0,
			.delay = 0,
		},
		{
			.reg_addr = 0x1DC8,
			.reg_data = 0xFF,
			.data_mask = 0,
			.delay = 0,
		},
		{
			.reg_addr = 0x1DC9,
			.reg_data = 0xF5,
			.data_mask = 0,
			.delay = 0,
		},
		{
			.reg_addr = 0x1DCA,
			.reg_data = 0x00,
			.data_mask = 0,
			.delay = 0,
		},
		{
			.reg_addr = 0x1DCB,
			.reg_data = 0xA4,
			.data_mask = 0,
			.delay = 0,
		},
		{
			.reg_addr = 0x1DCC,
			.reg_data = 0x00,
			.data_mask = 0,
			.delay = 0,
		},
		{
			.reg_addr = 0x1DCD,
			.reg_data = 0x26,
			.data_mask = 0,
			.delay = 0,
		},
		{
			.reg_addr = 0x1DCE,
			.reg_data = 0x00,
			.data_mask = 0,
			.delay = 0,
		},
		{
			.reg_addr = 0x1DCF,
			.reg_data = 0x25,
			.data_mask = 0,
			.delay = 0,
		},
		{
			.reg_addr = 0x1DD0,
			.reg_data = 0x08,
			.data_mask = 0,
			.delay = 0,
		},
		{
			.reg_addr = 0x1DD1,
			.reg_data = 0x55,
			.data_mask = 0,
			.delay = 0,
		},
		{
			.reg_addr = 0x1DD2,
			.reg_data = 0x08,
			.data_mask = 0,
			.delay = 0,
		},
		{
			.reg_addr = 0x1DD3,
			.reg_data = 0x09,
			.data_mask = 0,
			.delay = 0,
		},
		{
			.reg_addr = 0x1DD4,
			.reg_data = 0x00,
			.data_mask = 0,
			.delay = 0,
		},
		{
			.reg_addr = 0x1DD5,
			.reg_data = 0x74,
			.data_mask = 0,
			.delay = 0,
		},
		{
			.reg_addr = 0x1DD6,
			.reg_data = 0x25,
			.data_mask = 0,
			.delay = 0,
		},
		{
			.reg_addr = 0x1DD7,
			.reg_data = 0x0F,
			.data_mask = 0,
			.delay = 0,
		},
		{
			.reg_addr = 0x1DD8,
			.reg_data = 0x29,
			.data_mask = 0,
			.delay = 0,
		},
		{
			.reg_addr = 0x1DD9,
			.reg_data = 0x68,
			.data_mask = 0,
			.delay = 0,
		},
		{
			.reg_addr = 0x1DDA,
			.reg_data = 0xC9,
			.data_mask = 0,
			.delay = 0,
		},
		{
			.reg_addr = 0x1DDB,
			.reg_data = 0x0A,
			.data_mask = 0,
			.delay = 0,
		},
		{
			.reg_addr = 0x1DDC,
			.reg_data = 0xC9,
			.data_mask = 0,
			.delay = 0,
		},
		{
			.reg_addr = 0x1DDD,
			.reg_data = 0x0A,
			.data_mask = 0,
			.delay = 0,
		},
		{
			.reg_addr = 0x1DDE,
			.reg_data = 0x20,
			.data_mask = 0,
			.delay = 0,
		},
		{
			.reg_addr = 0x1DDF,
			.reg_data = 0x00,
			.data_mask = 0,
			.delay = 0,
		},
		{
			.reg_addr = 0x1DE0,
			.reg_data = 0x20,
			.data_mask = 0,
			.delay = 0,
		},
		{
			.reg_addr = 0x1DE1,
			.reg_data = 0x00,
			.data_mask = 0,
			.delay = 0,
		},
		{
			.reg_addr = 0x1DE2,
			.reg_data = 0x00,
			.data_mask = 0,
			.delay = 0,
		},
		{
			.reg_addr = 0x1DE3,
			.reg_data = 0x00,
			.data_mask = 0,
			.delay = 0,
		},
		{
			.reg_addr = 0x1DE4,
			.reg_data = 0x00,
			.data_mask = 0,
			.delay = 0,
		},
		{
			.reg_addr = 0x1DE5,
			.reg_data = 0x00,
			.data_mask = 0,
			.delay = 0,
		},
		{
			.reg_addr = 0x1DE6,
			.reg_data = 0x00,
			.data_mask = 0,
			.delay = 0,
		},
		{
			.reg_addr = 0x1DE7,
			.reg_data = 0x00,
			.data_mask = 0,
			.delay = 0,
		}
	};
	i2c_reg_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD;
	i2c_reg_setting.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	i2c_reg_setting.reg_setting = calib_reg_array;
	i2c_reg_setting.size = ARRAY_SIZE(calib_reg_array);
	i2c_reg_setting.delay = 0;
	CAM_DBG(CAM_OIS, "calib data write");
	return camera_io_dev_write_continuous(&(o_ctrl->io_master_info),
		&i2c_reg_setting, 1);
}

int cam_ois_bm24218_complete_dl(struct cam_ois_ctrl_t *o_ctrl)
{
	struct cam_sensor_i2c_reg_setting  i2c_reg_setting;
	struct cam_sensor_i2c_reg_array    i2c_reg_array;

	i2c_reg_array.reg_addr = 0xF006;
	i2c_reg_array.reg_data = 0x00;
	i2c_reg_array.data_mask = 0;
	i2c_reg_array.delay = 0;
	i2c_reg_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD;
	i2c_reg_setting.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	i2c_reg_setting.reg_setting = &i2c_reg_array;
	i2c_reg_setting.size = 1;
	i2c_reg_setting.delay = 1;
	return camera_io_dev_write(&o_ctrl->io_master_info, &i2c_reg_setting);
}

int cam_ois_bm24218_read_status(struct cam_ois_ctrl_t *o_ctrl, uint32_t *status)
{
	int32_t rc = 0;

	if (status == NULL || o_ctrl == NULL)
		return -EINVAL;

	rc = camera_io_dev_read(&o_ctrl->io_master_info, 0x6024, status,
			CAMERA_SENSOR_I2C_TYPE_WORD,
			CAMERA_SENSOR_I2C_TYPE_BYTE);
	return rc;
}

int cam_ois_bm24218_poll_status(struct cam_ois_ctrl_t *o_ctrl)
{
	uint32_t oisStatus = 0;
	uint32_t statusCount = 0;
	int32_t rc = 0;

	while (oisStatus == 0) {
		if (statusCount == 100 ) {
			CAM_ERR(CAM_OIS, "cam OIS status timeout");
			rc = -EBUSY;
			break;
		}
		statusCount++;
		usleep_range(5000, 6000);
		rc = cam_ois_bm24218_read_status(o_ctrl, &oisStatus);
		if (rc < 0) {
			CAM_ERR(CAM_OIS, "Cannot read OIS status: %d", rc);
			break;
		}
	}
	return rc;
}

int cam_ois_bm24218_enable_servo_program_gyro(struct cam_ois_ctrl_t *o_ctrl)
{
	int32_t rc = 0;
	uint32_t i = 0;
	struct cam_sensor_i2c_reg_setting  i2c_reg_setting;
	struct cam_sensor_i2c_reg_array    gyro_reg_array[] = {
		{
			.reg_addr = 0x6020,
			.reg_data = 0x01,
			.data_mask = 0,
			.delay = 0,
		},
		{
			.reg_addr = 0x6023,
			.reg_data = 0x02,
			.data_mask = 0,
			.delay = 0,
		},
		{
			.reg_addr = 0x602c,
			.reg_data = 0x76,
			.data_mask = 0,
			.delay = 0,
		},
		{
			.reg_addr = 0x602d,
			.reg_data = 0x00,
			.data_mask = 0,
			.delay = 0,
		},
		{
			.reg_addr = 0x602c,
			.reg_data = 0x11,
			.data_mask = 0,
			.delay = 0,
		},
		{
			.reg_addr = 0x602d,
			.reg_data = 0x01,
			.data_mask = 0,
			.delay = 110,
		},
		{
			.reg_addr = 0x602c,
			.reg_data = 0x76,
			.data_mask = 0,
			.delay = 0
		},
		{
			.reg_addr = 0x602d,
			.reg_data = 0x01,
			.data_mask = 0,
			.delay = 0
		},
		{
			.reg_addr = 0x602c,
			.reg_data = 0x7A,
			.data_mask = 0,
			.delay = 0
		},
		{
			.reg_addr = 0x602d,
			.reg_data = 0x02,
			.data_mask = 0,
			.delay = 0
		},
		{
			.reg_addr = 0x602c,
			.reg_data = 0x7B,
			.data_mask = 0,
			.delay = 0
		},
		{
			.reg_addr = 0x602d,
			.reg_data = 0x02,
			.data_mask = 0,
			.delay = 0
		},
		{
			.reg_addr = 0x602c,
			.reg_data = 0x76,
			.data_mask = 0,
			.delay = 0
		},
		{
			.reg_addr = 0x602d,
			.reg_data = 0x00,
			.data_mask = 0,
			.delay = 0
		},
		{
			.reg_addr = 0x602c,
			.reg_data = 0x4C,
			.data_mask = 0,
			.delay = 0
		},
		{
			.reg_addr = 0x602d,
			.reg_data = 0x03,
			.data_mask = 0,
			.delay = 0
		},
		{
			.reg_addr = 0x602c,
			.reg_data = 0x13,
			.data_mask = 0,
			.delay = 0
		},
		{
			.reg_addr = 0x602d,
			.reg_data = 0x05,
			.data_mask = 0,
			.delay = 0
		},
		{
			.reg_addr = 0x602c,
			.reg_data = 0x14,
			.data_mask = 0,
			.delay = 0
		},
		{
			.reg_addr = 0x602d,
			.reg_data = 0x1B,
			.data_mask = 0,
			.delay = 0
		},
		{
			.reg_addr = 0x602c,
			.reg_data = 0x64,
			.data_mask = 0,
			.delay = 0
		},
		{
			.reg_addr = 0x602d,
			.reg_data = 0x60,
			.data_mask = 0,
			.delay = 0
		},
		{
			.reg_addr = 0x602c,
			.reg_data = 0x65,
			.data_mask = 0,
			.delay = 0
		},
		{
			.reg_addr = 0x602d,
			.reg_data = 0x08,
			.data_mask = 0,
			.delay = 0
		},
		{
			.reg_addr = 0x602c,
			.reg_data = 0x4F,
			.data_mask = 0,
			.delay = 0
		},
		{
			.reg_addr = 0x602d,
			.reg_data = 0x63,
			.data_mask = 0,
			.delay = 0
		},
		{
			.reg_addr = 0x602c,
			.reg_data = 0x4E,
			.data_mask = 0,
			.delay = 0
		},
		{
			.reg_addr = 0x602d,
			.reg_data = 0x0C,
			.data_mask = 0,
			.delay = 20
		},
		{
			.reg_addr = 0x6023,
			.reg_data = 0x00,
			.data_mask = 0,
			.delay = 0
		},
		{
			.reg_addr = 0x6021,
			.reg_data = 0x7B,
			.data_mask = 0,
			.delay = 0
		}
	};
	i2c_reg_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD;
	i2c_reg_setting.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	i2c_reg_setting.size = 1;
	i2c_reg_setting.delay = 0;

	/*
	 * Qualcomm delay in the reg settings don't actually do anything so have to
	 * include delays manually
	 */
	for(i = 0; i < ARRAY_SIZE(gyro_reg_array); i++) {
		i2c_reg_setting.reg_setting = &gyro_reg_array[i];

		rc = camera_io_dev_write(&o_ctrl->io_master_info, &i2c_reg_setting);
		if (rc < 0) {
			CAM_ERR(CAM_OIS, "%d OIS gyro settings failed", rc);
			return rc;
		}
		if(gyro_reg_array[i].reg_addr == 0x602D && gyro_reg_array[i].reg_data == 0x01) {
			msleep(110);
		}
		if(gyro_reg_array[i].reg_addr == 0x602D && gyro_reg_array[i].reg_data == 0x0E) {
			usleep_range(20*1000, 21*1000);
		}
	}

	return rc;
}

int cam_ois_bm24218_enable_ois(struct cam_ois_ctrl_t *o_ctrl)
{
	int32_t rc = 0;
	struct cam_sensor_i2c_reg_setting  i2c_reg_setting;
	struct cam_sensor_i2c_reg_array ois_on_reg = {
		.reg_addr = 0x6020,
		.reg_data = 0x02,
		.data_mask = 0,
		.delay = 0
	};

	i2c_reg_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD;
	i2c_reg_setting.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	i2c_reg_setting.reg_setting = &ois_on_reg;
	i2c_reg_setting.size = 1;
	i2c_reg_setting.delay = 0;

	rc = camera_io_dev_write(&o_ctrl->io_master_info, &i2c_reg_setting);
	if (rc < 0) {
		CAM_ERR(CAM_OIS, "%d OIS On failed", rc);
		return rc;
	}
	return rc;
}

int cam_ois_bm24218_after_download(struct cam_ois_ctrl_t *o_ctrl)
{
	int32_t rc = 0;
	usleep_range(1*1000, 2*1000);
	rc = cam_ois_bm24218_poll_status(o_ctrl);
	if (rc < 0) {
		CAM_ERR(CAM_OIS, "%d poll status 0 failed", rc);
		return rc;
	}

	rc = cam_ois_bm24218_enable_servo_program_gyro(o_ctrl);
	if (rc < 0) {
		CAM_ERR(CAM_OIS, "%d OIS Enable servo failed", rc);
		return rc;
	}

	rc = cam_ois_bm24218_poll_status(o_ctrl);
	if (rc < 0) {
		CAM_ERR(CAM_OIS, "%d poll status 1 failed", rc);
		return rc;
	}

	return rc;
}

int cam_ois_bm24218_boot_sequence(struct cam_ois_ctrl_t *o_ctrl)
{
	int32_t rc = 0;

	if (o_ctrl->ois_fw_flag || o_ctrl->is_ois_calib) {
		rc = cam_ois_bm24218_start_dl(o_ctrl);
		if (rc < 0) {
			CAM_ERR(CAM_OIS, "Can't start dl");
			return rc;
		}
		usleep_range(200,210);

		if (o_ctrl->ois_fw_flag) {
			rc = cam_ois_bm24218_fw_download(o_ctrl);
			if (rc) {
				CAM_ERR(CAM_OIS, "Failed OIS FW Download");
				return rc;
			}
			CAM_ERR(CAM_OIS, "Fw dl success");
		}

		if (o_ctrl->is_ois_calib) {
			rc = cam_ois_bm24218_calib(o_ctrl);
			if (rc) {
				CAM_ERR(CAM_OIS, "Cannot apply calib data");
				return rc;
			}
		}

		cam_ois_bm24218_complete_dl(o_ctrl);
		if (rc < 0) {
			CAM_ERR(CAM_OIS, "Can't stop dl");
			return rc;
		}
	}

	rc = cam_ois_bm24218_after_download(o_ctrl);
	if (rc < 0)
		CAM_ERR(CAM_OIS, "Cannot apply Init settings");
	else
		CAM_ERR(CAM_OIS, "Fw dl success");

	return rc;
}
