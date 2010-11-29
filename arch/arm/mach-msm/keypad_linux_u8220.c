/*
 * Copyright (C) 2007 Google, Inc.
 * Copyright (c) 2008 QUALCOMM USA, INC.
 * Author: Brian Swetland <swetland@google.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <asm/mach-types.h>
#include <linux/platform_device.h>
#include <linux/gpio_event.h>

/* don't turn this on without updating the ffa support */
#define SCAN_FUNCTION_KEYS 0

static unsigned int keypad_row_gpios[] = { 35, 34, 33 };

static unsigned int keypad_col_gpios[] = { 39, 38, 40 };

#define KEYMAP_INDEX(row, col) ((row)*ARRAY_SIZE(keypad_col_gpios) + (col))

static const unsigned short keypad_keymap[ARRAY_SIZE(keypad_col_gpios) *
					      ARRAY_SIZE(keypad_row_gpios)] = {
    
    
    [KEYMAP_INDEX(0, 0)] = 61,   /* CALL 61 */ 
    [KEYMAP_INDEX(0, 1)] = 62,   /* ENDCALL 62 */ 
    [KEYMAP_INDEX(0, 2)] = 115,  /* KEY_VOLUMEUP 115 */ 
    [KEYMAP_INDEX(1, 0)] = 229,  /* KEY_MENU  229  */ 
    /* [KEYMAP_INDEX(1, 1)] = 158,*/  /* KEY_BACK 158, 158 will be used in u8230*/ 
    [KEYMAP_INDEX(1, 1)] = 250,  /* KEY_BACKLIGHT 250 ,250 will be deleted in u8230 */
    [KEYMAP_INDEX(1, 2)] = 114,  /* KEY_VOLUMEDOWN 114 */ 
    [KEYMAP_INDEX(2, 0)] = 232,  /* DPAD_CENTER 232 */ 
    [KEYMAP_INDEX(2, 1)] = 249,  /* KEY_FOCUS  249   */ 
    [KEYMAP_INDEX(2, 2)] = 212,  /* KEY_CAMERA 212 */
               
    
};

static const unsigned short keypad_virtual_keys[] = {
	KEY_END,
	KEY_POWER
};


static struct gpio_event_matrix_info keypad_matrix_info = {
	.info.func	= gpio_event_matrix_func,
	.keymap		= keypad_keymap,
	.output_gpios	= keypad_row_gpios,
	.input_gpios	= keypad_col_gpios,
	.noutputs	= ARRAY_SIZE(keypad_row_gpios),
	.ninputs	= ARRAY_SIZE(keypad_col_gpios),
	.settle_time.tv.nsec = 0,
	.poll_time.tv.nsec = 20 * NSEC_PER_MSEC,
	/*.flags		= GPIOKPF_LEVEL_TRIGGERED_IRQ | GPIOKPF_DRIVE_INACTIVE |
			  GPIOKPF_PRINT_UNMAPPED_KEYS*/
	
	.flags		= GPIOKPF_LEVEL_TRIGGERED_IRQ | GPIOKPF_PRINT_UNMAPPED_KEYS
};

struct gpio_event_info *keypad_info[] = {
	&keypad_matrix_info.info
};

static struct gpio_event_platform_data keypad_data = {
	//.name		= "surf_ffa_keypad",
	.name		= "surf_keypad",

	.info		= keypad_info,
	.info_count	= ARRAY_SIZE(keypad_info)
};

static struct platform_device keypad_device = {
	.name	= GPIO_EVENT_DEV_NAME,
	.id	= -1,
	.dev	= {
		.platform_data	= &keypad_data,
	},
};

static struct input_dev *keypad_dev;

struct input_dev *msm_keypad_get_input_dev(void)
{
	return keypad_dev;
}

int init_u8220_keypad(int keypad)
{
	if (keypad)
		keypad_matrix_info.keymap = keypad_keymap;
	return platform_device_register(&keypad_device);
}

