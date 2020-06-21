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

#include "../common.h"
#include "../platform.h"
#include "finger_report.h"
#include "firmware.h"
#include "gesture.h"
#include "protocol.h"
#include "config.h"

struct core_gesture_data *core_gesture = NULL;
extern uint8_t ap_fw[MAX_AP_FIRMWARE_SIZE];

int core_gesture_match_key(uint8_t gdata)
{
	int gcode;

	switch (gdata) {
	case GESTURE_LEFT:
		gcode = KEY_GESTURE_LEFT;
		break;
	case GESTURE_RIGHT:
		gcode = KEY_GESTURE_RIGHT;
		break;
	case GESTURE_UP:
		gcode = KEY_GESTURE_UP;
		break;
	case GESTURE_DOWN:
		gcode = KEY_GESTURE_DOWN;
		break;
	case GESTURE_DOUBLECLICK:
		/* huaqin modify for ZQL1830-81 by liufurong at 20180813 start */
		gcode = KEY_GESTURE_DOUBLECLICK;
		/* huaqin modify for ZQL1830-81 by liufurong at 20180813 end */
		break;
	case GESTURE_O:
		gcode = KEY_GESTURE_O;
		break;
	case GESTURE_W:
		gcode = KEY_GESTURE_W;
		break;
	case GESTURE_M:
		gcode = KEY_GESTURE_M;
		break;
	case GESTURE_E:
		gcode = KEY_GESTURE_E;
		break;
	case GESTURE_S:
		gcode = KEY_GESTURE_S;
		break;
	case GESTURE_V:
		gcode = KEY_GESTURE_V;
		break;
	case GESTURE_Z:
		gcode = KEY_GESTURE_Z;
		break;
	case GESTURE_C:
		gcode = KEY_GESTURE_C;
		break;
	default:
		gcode = -1;
		break;
	}

	ipio_debug(DEBUG_GESTURE, "gcode = %d\n", gcode);
	return gcode;
}
EXPORT_SYMBOL(core_gesture_match_key);

void core_gesture_set_key(struct core_fr_data *fr_data)
{
	struct input_dev *input_dev = fr_data->input_device;
	if (input_dev != NULL) {
		input_set_capability(input_dev, EV_KEY, KEY_POWER);
		/* huaqin modify for ZQL1830-81 by liufurong at 20180813 start */
		input_set_capability(input_dev, EV_KEY, KEY_GESTURE_DOUBLECLICK);
		/* huaqin modify for ZQL1830-81 by liufurong at 20180813 end */
		input_set_capability(input_dev, EV_KEY, KEY_GESTURE_UP);
		input_set_capability(input_dev, EV_KEY, KEY_GESTURE_DOWN);
		input_set_capability(input_dev, EV_KEY, KEY_GESTURE_LEFT);
		input_set_capability(input_dev, EV_KEY, KEY_GESTURE_RIGHT);
		input_set_capability(input_dev, EV_KEY, KEY_GESTURE_O);
		input_set_capability(input_dev, EV_KEY, KEY_GESTURE_E);
		input_set_capability(input_dev, EV_KEY, KEY_GESTURE_M);
		input_set_capability(input_dev, EV_KEY, KEY_GESTURE_W);
		input_set_capability(input_dev, EV_KEY, KEY_GESTURE_S);
		input_set_capability(input_dev, EV_KEY, KEY_GESTURE_V);
		input_set_capability(input_dev, EV_KEY, KEY_GESTURE_Z);
		input_set_capability(input_dev, EV_KEY, KEY_GESTURE_C);

		__set_bit(KEY_POWER, input_dev->keybit);
		/* huaqin modify for ZQL1830-81 by liufurong at 20180813 start */
		__set_bit(KEY_GESTURE_DOUBLECLICK, input_dev->keybit);
		/* huaqin modify for ZQL1830-81 by liufurong at 20180813 end */
		__set_bit(KEY_GESTURE_UP, input_dev->keybit);
		__set_bit(KEY_GESTURE_DOWN, input_dev->keybit);
		__set_bit(KEY_GESTURE_LEFT, input_dev->keybit);
		__set_bit(KEY_GESTURE_RIGHT, input_dev->keybit);
		__set_bit(KEY_GESTURE_O, input_dev->keybit);
		__set_bit(KEY_GESTURE_E, input_dev->keybit);
		__set_bit(KEY_GESTURE_M, input_dev->keybit);
		__set_bit(KEY_GESTURE_W, input_dev->keybit);
		__set_bit(KEY_GESTURE_S, input_dev->keybit);
		__set_bit(KEY_GESTURE_V, input_dev->keybit);
		__set_bit(KEY_GESTURE_Z, input_dev->keybit);
		__set_bit(KEY_GESTURE_C, input_dev->keybit);
		return;
	}

	ipio_err("GESTURE: input dev is NULL\n");
}
EXPORT_SYMBOL(core_gesture_set_key);

int core_gesture_init(void)
{
	core_gesture = devm_kzalloc(ipd->dev, sizeof(*core_gesture), GFP_KERNEL);
	if (ERR_ALLOC_MEM(core_gesture)) {
		ipio_err("Failed to allocate core_gesture mem, %ld\n", PTR_ERR(core_gesture));
		return -ENOMEM;
	}

	core_gesture->entry = false;

	return 0;
}
EXPORT_SYMBOL(core_gesture_init);

void core_gesture_remove(void)
{
	ipio_info("Remove core-gesture members\n");
	ipio_kfree((void **)&core_gesture);
}
EXPORT_SYMBOL(core_gesture_remove);