/*
 * Copyright (C) 2021 Motorola Mobility LLC.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include "cam_ois_core.h"
#include "cam_ois_soc.h"
#include "cam_sensor_util.h"
#include "cam_debug_util.h"
#include "cam_common_util.h"

#define DW9781C_CHIP_ID_ADDRESS 0x7000
#define DW9781C_CHECKSUM_ADDRESS 0x7007
#define DW9781C_CHIP_ID 0x9781
#define FW_VER_CURR_ADDR 0x7001
#define FW_TYPE_ADDR 0x700D
#define SET_FW 0x8001

#define EOK             0

#define FW_VERSION_OFFSET 10235
#define FW_CHECKSUM_OFFSET 10234

typedef struct
{
	unsigned int driverIc;
	unsigned int size;
	const uint16_t *fwContentPtr;
	uint16_t version;
	uint16_t checksum;
} FirmwareContex;

static FirmwareContex g_dw9781FirmwareContext;

static int32_t dw9781_cci_write(struct camera_io_master * io_master_info, uint16_t reg, uint16_t val)
{
	int32_t rc = 0;
	struct cam_sensor_i2c_reg_array reg_setting;
	struct cam_sensor_i2c_reg_setting wr_setting;

	reg_setting.reg_addr = reg;
	reg_setting.reg_data = val;
	reg_setting.delay = 0;
	reg_setting.data_mask = 0;
	wr_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD;
	wr_setting.data_type = CAMERA_SENSOR_I2C_TYPE_WORD;
	wr_setting.reg_setting = &reg_setting;
	wr_setting.size = 1;
	wr_setting.delay = 0;
	rc = camera_io_dev_write(io_master_info, &wr_setting);
	return rc;
}

static int32_t dw9781_cci_read(struct camera_io_master * io_master_info, uint16_t reg, uint16_t *val)
{
	int32_t rc = 0;
	uint32_t regVal = 0;
	rc = camera_io_dev_read(io_master_info, reg, &regVal, CAMERA_SENSOR_I2C_TYPE_WORD, CAMERA_SENSOR_I2C_TYPE_WORD);
	if (!rc) {
		*val = (uint16_t)regVal;
	}
	return rc;
}

static void dw9781_delay_ms(uint32_t ms)
{
	usleep_range(ms*1000, ms*1000+10);
	return;
}

void dw9781_ois_reset(struct camera_io_master * io_master_info)
{
	CAM_DBG(CAM_OIS, "[dw9781c_ois_reset] ois reset");
	dw9781_cci_write(io_master_info, 0xD002, 0x0001); /* DW9781_DBGc reset */
	dw9781_delay_ms(4);
	dw9781_cci_write(io_master_info, 0xD001, 0x0001); /* Active mode (DSP ON) */
	dw9781_delay_ms(25);                              /* ST gyro - over wait 25ms, default Servo On */
	dw9781_cci_write(io_master_info, 0xEBF1, 0x56FA); /* User protection release */
}

void dw9781_ois_ready_check(struct camera_io_master * io_master_info)
{
	uint16_t fw_flag;
	dw9781_cci_write(io_master_info, 0xD001, 0x0000);  /* dsp off mode */
	dw9781_cci_write(io_master_info, 0xFAFA, 0x98AC);  /* All protection(1) */
	dw9781_cci_write(io_master_info, 0xF053, 0x70BD);  /* All protection(2) */
	dw9781_cci_read(io_master_info, 0xA7F9, &fw_flag); /* check checksum flag */
	CAM_DBG(CAM_OIS, "[dw9781c_ois_ready_check] checksum flag : 0x%04x", fw_flag);
	if(fw_flag == 0xCC33)
	{
		CAM_DBG(CAM_OIS, "[dw9781c_ois_ready_check] checksum flag is ok");
		dw9781_ois_reset(io_master_info);                 /* ois reset */
	} else {
		dw9781_cci_write(io_master_info, 0xD002, 0x0001); /* dw9781c reset */
		dw9781_delay_ms(4);
		CAM_ERR(CAM_OIS, "[dw9781c_ois_ready_check] previous firmware download fail");
	}
}

void dw9781_all_protection_release(struct camera_io_master * io_master_info)
{
	CAM_DBG(CAM_OIS, "[dw9781c_all_protection_release] execution");
	/* release all protection */
	dw9781_cci_write(io_master_info, 0xFAFA, 0x98AC);
	dw9781_delay_ms(1);
	dw9781_cci_write(io_master_info, 0xF053, 0x70BD);
	dw9781_delay_ms(1);
}

static uint16_t dw9781_fw_checksum_verify(struct camera_io_master * io_master_info)
{
	uint16_t data;

	/* FW checksum command */
	dw9781_cci_write(io_master_info, 0x7011, 0x2000);
	/* command  start */
	dw9781_cci_write(io_master_info, 0x7010, 0x8000);
	dw9781_delay_ms(10);
	/* calc the checksum to write the 0x7005 */
	dw9781_cci_read(io_master_info, 0x7005, &data);
	CAM_DBG(CAM_OIS, "F/W Checksum calculated value : 0x%04X", data);

	return data;
}

static int dw9781_erase_mtp_rewritefw(struct camera_io_master * io_master_info)
{
	CAM_DBG(CAM_OIS, "dw9781c erase for rewritefw starting..");

	/* 512 byte page */
	dw9781_cci_write(io_master_info, 0xde03, 0x0027);
	/* page erase */
	dw9781_cci_write(io_master_info, 0xde04, 0x0008);
	dw9781_delay_ms(10);

	CAM_DBG(CAM_OIS, "dw9781c checksum flag erase : 0xCC33");
	return 0;
}

static void dw9781_erase_mtp(struct camera_io_master * io_master_info)
{
	CAM_DBG(CAM_OIS, "[dw9781c_erase_mtp] start erasing firmware flash");
	/* 12c level adjust */
	dw9781_cci_write(io_master_info, 0xd005, 0x0001);
	dw9781_cci_write(io_master_info, 0xdd03, 0x0002);
	dw9781_cci_write(io_master_info, 0xdd04, 0x0002);

	/* 4k Sector_0 Erase*/
	dw9781_cci_write(io_master_info, 0xde03, 0x0000);
	dw9781_cci_write(io_master_info, 0xde04, 0x0002);
	dw9781_delay_ms(10);
	/* 4k Sector_1 Erase*/
	dw9781_cci_write(io_master_info, 0xde03, 0x0008);
	dw9781_cci_write(io_master_info, 0xde04, 0x0002);
	dw9781_delay_ms(10);
	/* 4k Sector_2 Erase*/
	dw9781_cci_write(io_master_info, 0xde03, 0x0010);
	dw9781_cci_write(io_master_info, 0xde04, 0x0002);
	dw9781_delay_ms(10);
	/* 4k Sector_3 Erase*/
	dw9781_cci_write(io_master_info, 0xde03, 0x0018);
	dw9781_cci_write(io_master_info, 0xde04, 0x0002);
	dw9781_delay_ms(10);
	/* 4k Sector_4 Erase*/
	dw9781_cci_write(io_master_info, 0xde03, 0x0020);
	dw9781_cci_write(io_master_info, 0xde04, 0x0002);
	dw9781_delay_ms(10);
	CAM_DBG(CAM_OIS, "[dw9781c_erase_mtp] complete erasing firmware flash");
}

static int dw9781_prepare_fw_download(struct camera_io_master * io_master_info)
{
	/* step 1: MTP Erase and DSP Disable for firmware 0x8000 write */
	/* step 2: MTP setup                                           */
	/* step 3: firmware sequential write to flash                  */
	/* step 4: firmware sequential read from flash                 */
	/* step 5: firmware verify                                     */
	dw9781_cci_write(io_master_info, 0xd001, 0x0000);
	dw9781_all_protection_release(io_master_info);
	dw9781_erase_mtp(io_master_info);
	CAM_DBG(CAM_OIS, "[dw9781c_download_fw] start firmware download");
	return 0;
}

int dw9781c_check_fw_download(struct camera_io_master * io_master_info, const uint8_t *fwData, uint32_t fwSize)
{
	uint8_t ret;
	uint8_t needDownload = 0;
	uint16_t fwchecksum = 0;
	uint16_t first_chip_id = 0;
	uint16_t chip_checksum = 0;
	uint16_t second_chip_id = 0;
	uint16_t fw_version_current = 0;
	uint16_t fw_version_latest = 0;

	if (io_master_info == NULL) {
		printk("FATAL: OIS CCI context error!!!");
		return -1;
	}

	if (fwData == NULL || fwSize < FW_VERSION_OFFSET*sizeof(uint16_t)) {
		printk("FATAL: firmware buffer(%p) is NULL or size(%d) abnormal!!!", fwData, fwSize);
		return -1;
	}

	g_dw9781FirmwareContext.driverIc = 0x9781;
	g_dw9781FirmwareContext.fwContentPtr = (const uint16_t *)fwData;
	g_dw9781FirmwareContext.size = fwSize;
	g_dw9781FirmwareContext.version = *(g_dw9781FirmwareContext.fwContentPtr+FW_VERSION_OFFSET);
	g_dw9781FirmwareContext.version = ((g_dw9781FirmwareContext.version << 8) & 0xff00) |
	                                  ((g_dw9781FirmwareContext.version >> 8) & 0xff);
	g_dw9781FirmwareContext.checksum = *(g_dw9781FirmwareContext.fwContentPtr+FW_CHECKSUM_OFFSET);
	g_dw9781FirmwareContext.checksum = ((g_dw9781FirmwareContext.checksum << 8) & 0xff00) |
	                                   ((g_dw9781FirmwareContext.checksum >> 8) & 0xff);

	dw9781_ois_ready_check(io_master_info);

	dw9781_cci_read(io_master_info, DW9781C_CHIP_ID_ADDRESS, &first_chip_id);
	dw9781_cci_read(io_master_info, 0x7001, &chip_checksum);
	CAM_DBG(CAM_OIS, "[dw9781c] FW_VER_PHONE_MAKER : 0x%x", chip_checksum);
	dw9781_cci_read(io_master_info, 0x7002, &chip_checksum);
	CAM_DBG(CAM_OIS, "[dw9781c] FW_DATE_PHONE_MAKER : 0x%x", chip_checksum);

	CAM_DBG(CAM_OIS, "[dw9781c] first_chip_id : 0x%x", first_chip_id);
	if (first_chip_id != DW9781C_CHIP_ID) { /* first_chip_id verification failed */
		dw9781_all_protection_release(io_master_info);
		dw9781_cci_read(io_master_info, 0xd060, &second_chip_id); /* second_chip_id: 0x0020 */
		if ( second_chip_id == 0x0020 )                           /* Check the second_chip_id*/
		{
			CAM_DBG(CAM_OIS, "[dw9781c] start flash download:: size:%d, version:0x%x",
				g_dw9781FirmwareContext.size, g_dw9781FirmwareContext.version);
			needDownload = 1;
			ret = dw9781_prepare_fw_download(io_master_info); /* Need to forced update OIS firmware again. */
		} else {
			dw9781_cci_write(io_master_info, 0xd000, 0x0000); /* Shutdown mode */
			CAM_ERR(CAM_OIS, "[dw9781c] second_chip_id check fail");
			CAM_ERR(CAM_OIS, "[dw9781c] change dw9781c state to shutdown mode");
			return needDownload;
		}
	} else {
		fwchecksum = dw9781_fw_checksum_verify(io_master_info);
		if(fwchecksum != g_dw9781FirmwareContext.checksum)
		{
			needDownload = 1;
			CAM_DBG(CAM_OIS, "[dw9781c] firmware checksum error 0x%04X, 0x%04X",
			                  g_dw9781FirmwareContext.checksum, fwchecksum);
		}

		dw9781_cci_read(io_master_info, FW_VER_CURR_ADDR, &fw_version_current);
		fw_version_latest = g_dw9781FirmwareContext.version; /*Firmware version read from file content.*/

		CAM_DBG(CAM_OIS, "[dw9781c] fw_version_current = 0x%x, fw_version_latest = 0x%x",
			              fw_version_current, fw_version_latest);

		/* download firmware, check if need update, download firmware to flash */
		if (needDownload || ((fw_version_current & 0xFF) != (fw_version_latest & 0xFF))) {
			CAM_DBG(CAM_OIS, "[dw9781c] start flash download:: size:%d, version:0x%x needDownload %d",
			                 g_dw9781FirmwareContext.size, g_dw9781FirmwareContext.version, needDownload);

			ret = dw9781_prepare_fw_download(io_master_info);
			CAM_DBG(CAM_OIS, "[dw9781c] flash download::vendor_dw9781c");
			if (ret != EOK) {
				dw9781_erase_mtp_rewritefw(io_master_info);
				dw9781_cci_write(io_master_info, 0xd000, 0x0000); /* Shut download mode */
				CAM_ERR(CAM_OIS, "[dw9781c] firmware download error, ret = 0x%x", ret);
				CAM_ERR(CAM_OIS, "[dw9781c] change dw9781c state to shutdown mode");
				needDownload = 1;
			}
		} else {
			CAM_DBG(CAM_OIS, "[dw9781c] ois firmware version is updated, skip download");
		}
	}
	return needDownload;
}
EXPORT_SYMBOL(dw9781c_check_fw_download);

void dw9781_post_firmware_download(struct camera_io_master * io_master_info, const uint8_t *fwData, uint32_t fwSize)
{
	uint16_t fwchecksum = 0;

	g_dw9781FirmwareContext.checksum = *(g_dw9781FirmwareContext.fwContentPtr+FW_CHECKSUM_OFFSET);
	g_dw9781FirmwareContext.checksum = ((g_dw9781FirmwareContext.checksum << 8) & 0xff00) |
	                                   ((g_dw9781FirmwareContext.checksum >> 8) & 0xff);

	dw9781_ois_ready_check(io_master_info);

	fwchecksum = dw9781_fw_checksum_verify(io_master_info);
	if(fwchecksum != g_dw9781FirmwareContext.checksum)
	{
		CAM_ERR(CAM_OIS, "[dw9781c] firmware checksum error 0x%04X, 0x%04X", g_dw9781FirmwareContext.checksum, fwchecksum);
		dw9781_cci_write(io_master_info, 0xd000, 0x0000); /* Shutdown mode */
		dw9781_delay_ms(100);
		dw9781_cci_write(io_master_info, 0xd000, 0x0001); /* Standby mode */
		dw9781_delay_ms(100);
		dw9781_all_protection_release(io_master_info);    /* Disable write protect */
		dw9781_erase_mtp_rewritefw(io_master_info);
		dw9781_cci_write(io_master_info, 0xd000, 0x0000); /* Shutdown mode */
		CAM_ERR(CAM_OIS, "[dw9781c] firmware download error");
		CAM_ERR(CAM_OIS, "[dw9781c] change dw9781c state to shutdown mode");
	} else {
		CAM_DBG(CAM_OIS, "[dw9781c] Firmware download succes...");
	}

	return;
}

EXPORT_SYMBOL(dw9781_post_firmware_download);
