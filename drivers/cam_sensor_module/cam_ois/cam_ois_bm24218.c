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
			CAMERA_SENSOR_I2C_TYPE_BYTE);

	//TODO: Need actual check here
	CAM_ERR(CAM_OIS, "OIS FW CHECKSUM: %x", fwChecksum);

	return rc;
}

int cam_ois_bm24218_fw_download(struct cam_ois_ctrl_t *o_ctrl)
{
	uint16_t                           total_bytes = 0;
	uint8_t                           *ptr = NULL;
	int32_t                            rc = 0, cnt;
	uint32_t                           fw_size;
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
	i2c_reg_setting.size = total_bytes;
	i2c_reg_setting.delay = 1;
	fw_size = PAGE_ALIGN(sizeof(struct cam_sensor_i2c_reg_array) *
		total_bytes) >> PAGE_SHIFT;
	page = cma_alloc(dev_get_cma_area((o_ctrl->soc_info.dev)),
		fw_size, 0, GFP_KERNEL);
	if (!page) {
		CAM_ERR(CAM_OIS, "Failed in allocating i2c_array");
		release_firmware(fw);
		return -ENOMEM;
	}

	i2c_reg_setting.reg_setting = (struct cam_sensor_i2c_reg_array *) (
		page_address(page));

	for (cnt = 0, ptr = (uint8_t *)fw->data; cnt < total_bytes;
		cnt++, ptr++) {
		i2c_reg_setting.reg_setting[cnt].reg_addr =
			o_ctrl->opcode.prog;
		i2c_reg_setting.reg_setting[cnt].reg_data = *ptr;
		i2c_reg_setting.reg_setting[cnt].delay = 0;
		i2c_reg_setting.reg_setting[cnt].data_mask = 0;
	}

	rc = camera_io_dev_write_continuous(&(o_ctrl->io_master_info),
		&i2c_reg_setting, 1);
	if (rc < 0) {
		CAM_ERR(CAM_OIS, "OIS FW download failed %d", rc);
		goto release_firmware;
	}
	cma_release(dev_get_cma_area((o_ctrl->soc_info.dev)),
		page, fw_size);
	page = NULL;
	fw_size = 0;
	release_firmware(fw);

	rc = request_firmware(&fw, fw_name_coeff, dev);
	if (rc) {
		CAM_ERR(CAM_OIS, "Failed to locate %s", fw_name_coeff);
		return rc;
	}

	total_bytes = fw->size;
	i2c_reg_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	i2c_reg_setting.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	i2c_reg_setting.size = total_bytes;
	i2c_reg_setting.delay = 0;
	fw_size = PAGE_ALIGN(sizeof(struct cam_sensor_i2c_reg_array) *
		total_bytes) >> PAGE_SHIFT;
	page = cma_alloc(dev_get_cma_area((o_ctrl->soc_info.dev)),
		fw_size, 0, GFP_KERNEL);
	if (!page) {
		CAM_ERR(CAM_OIS, "Failed in allocating i2c_array");
		release_firmware(fw);
		return -ENOMEM;
	}

	i2c_reg_setting.reg_setting = (struct cam_sensor_i2c_reg_array *) (
		page_address(page));

	for (cnt = 0, ptr = (uint8_t *)fw->data; cnt < total_bytes;
		cnt++, ptr++) {
		i2c_reg_setting.reg_setting[cnt].reg_addr =
			o_ctrl->opcode.coeff;
		i2c_reg_setting.reg_setting[cnt].reg_data = *ptr;
		i2c_reg_setting.reg_setting[cnt].delay = 0;
		i2c_reg_setting.reg_setting[cnt].data_mask = 0;
	}

	rc = camera_io_dev_write_continuous(&(o_ctrl->io_master_info),
		&i2c_reg_setting, 1);
	if (rc < 0) {
		CAM_ERR(CAM_OIS, "OIS FW download failed %d", rc);
		goto release_firmware;
	}

	rc = cam_ois_bm24218_fw_checksum(o_ctrl);
	if (rc < 0)
		CAM_ERR(CAM_OIS, "OIS FW checksum failed");

release_firmware:
	cma_release(dev_get_cma_area((o_ctrl->soc_info.dev)),
		page, fw_size);
	release_firmware(fw);

	return rc;
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

	rc = camera_io_dev_read(&o_ctrl->io_master_info, 0xF008, status,
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
		if (statusCount == 3000) {
			CAM_ERR(CAM_OIS, "cam OIS status timeout");
			rc = -EBUSY;
			break;
		}
		statusCount++;
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
			.reg_data = 0x02,
			.data_mask = 0,
			.delay = 0
		},
		{
			.reg_addr = 0x602c,
			.reg_data = 0x44,
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
			.reg_data = 0x45,
			.data_mask = 0,
			.delay = 0
		},
		{
			.reg_addr = 0x602d,
			.reg_data = 0x58,
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
	i2c_reg_setting.reg_setting = gyro_reg_array;
	i2c_reg_setting.size = ARRAY_SIZE(gyro_reg_array);
	i2c_reg_setting.delay = 0;

	rc = camera_io_dev_write(&o_ctrl->io_master_info, &i2c_reg_setting);
	if (rc < 0)
		CAM_ERR(CAM_OIS, "%d OIS gyro settings failed", rc);

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

	rc = cam_ois_bm24218_enable_ois(o_ctrl);

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

		if (o_ctrl->ois_fw_flag) {
			rc = cam_ois_bm24218_fw_download(o_ctrl);
			if (rc) {
				CAM_ERR(CAM_OIS, "Failed OIS FW Download");
				return rc;
			}
			CAM_ERR(CAM_OIS, "Fw dl success");
		}

//		if (o_ctrl->is_ois_calib) {
//		//needs sequential write
//			rc = cam_ois_apply_settings(o_ctrl,
//				&o_ctrl->i2c_calib_data);
//			if (rc) {
//				CAM_ERR(CAM_OIS, "Cannot apply calib data");
//				return rc;
//			}
//		}

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
