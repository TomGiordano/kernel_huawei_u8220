/*
 * Copyright (C) 2007 Google, Inc.
 * Copyright (c) 2008 QUALCOMM USA, INC.
 * Author: cyy
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
#include <linux/jogball_driver.h>
#include <linux/input.h>

#ifdef  CONFIG_MACH_MSM7201A_SURF
#define  JOGBALL_CTL  36
#define GPIO_UP      108
#define GPIO_DOWN    82
#define GPIO_LEFT    84
#define GPIO_RIGHT   85
#else
#define JOGBALL_CTL  38
#define GPIO_UP      36
#define GPIO_DOWN    32
#define GPIO_LEFT    31
#define GPIO_RIGHT   37
#endif

static struct jogball_button jogball_buttons[] = {

      {
		.gpio		= GPIO_UP,  /*108   */
		//.code		= KEY_UP,   /*103 up*/
		//.active_low	= 0,
		.desc		= "jogball_up",
		//.type              = EV_KEY,
	},

    {
		.gpio		= GPIO_DOWN,  /*82    */
		//.code		= KEY_DOWN,   /*108 down*/
		//.active_low	= 0,
		.desc		= "jogball_down",
		//.type              = EV_KEY,
	},

	{
		.gpio		= GPIO_LEFT,   /* 84     */
		//.code		= KEY_LEFT,    /* 105  left*/
		//.active_low	= 0,
		.desc		= "jogball_left",
		//.type              = EV_KEY,
	},
	
	{
		.gpio		= GPIO_RIGHT,  /* 85     */
		//.code		= KEY_RIGHT,   /* 106 right*/
		//.active_low	= 0,     
		.desc		= "jogball_right",
		//.type              = EV_KEY,
	},
	
};

static struct jogball_platform_data jogball_data = {
	.buttons	= jogball_buttons,
	.nbuttons	= ARRAY_SIZE(jogball_buttons),
	.gpio_ctl  = JOGBALL_CTL,
};

struct platform_device jogball_device = {
	.name		= "jogball",
	.id		= -1,
	.dev		= {
		.platform_data	= &jogball_data,
	},
};

#if 0
int init_jogball(void)
{
   return platform_device_register(&jogball_device);
}
#endif

