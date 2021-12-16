/*
 * ILITEK Touch IC driver
 *
 * Copyright (C) 2011 ILI Technology Corporation.
 *
 * Author: Dicky Chiang <dicky_chiang@ilitek.com>
 * Based on TDD v7.0 implemented by Mstar & ILITEK
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 */

#ifndef __FIRMWARE_H
#define __FIRMWARE_H

struct core_firmware_data {
	uint8_t new_fw_ver[4];
	uint8_t old_fw_ver[4];

	uint32_t start_addr;
	uint32_t end_addr;
	uint32_t checksum;
	uint32_t crc32;
	uint32_t new_fw_cb;
	uint32_t old_fw_cb;

	uint32_t update_status;
	uint32_t max_count;

	int delay_after_upgrade;

	bool isUpgrading;
	bool isCRC;
	bool isboot;
	bool hasBlockInfo;

	int (*upgrade_func)(bool isIRAM);
};

extern struct core_firmware_data *core_firmware;

#ifdef BOOT_FW_UPGRADE
extern int core_firmware_boot_upgrade(void);
#endif
extern int tddi_fw_upgrade(bool isIRAM);
/* extern int core_firmware_iram_upgrade(const char* fpath); */
extern int core_firmware_upgrade(const char *, bool isIRAM);
extern int core_firmware_init(void);

#endif /* __FIRMWARE_H */
