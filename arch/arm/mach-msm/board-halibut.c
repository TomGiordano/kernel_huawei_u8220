/* linux/arch/arm/mach-msm/board-halibut.c
 *
 * Copyright (C) 2007 Google, Inc.
 * Copyright (c) 2008-2009, Code Aurora Forum. All rights reserved.
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
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/bootmem.h>
#include <linux/i2c.h>
#include <linux/android_pmem.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <linux/power_supply.h>

#include <mach/hardware.h>
#include <mach/irqs.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/flash.h>
#include <asm/setup.h>

#include <asm/mach/mmc.h>
#include <mach/vreg.h>
#include <mach/mpp.h>
#include <mach/gpio.h>
#include <mach/irqs.h>
#include <mach/board.h>
#include <mach/msm_iomap.h>
#include <mach/msm_serial_hs.h>
#include <mach/msm_hsusb.h>
#include <mach/vreg.h>
#include <mach/msm_rpcrouter.h>
#include <mach/memory.h>
#include <mach/msm_battery.h>
#include <mach/camera.h>

#ifdef CONFIG_USB_FUNCTION
#include <linux/usb/mass_storage_function.h>
#endif
#ifdef CONFIG_USB_ANDROID
#include <linux/usb/android.h>
#include <mach/rpc_hsusb.h>
#endif

#include "devices.h"
#include "socinfo.h"
#include "clock.h"
#include "msm-keypad-devices.h"
#include "pm.h"
#ifdef CONFIG_HUAWEI_U8220_KEYBOARD
#include "keypad_linux_u8220.h"
#endif
#ifdef CONFIG_TOUCHSCREEN_INNOLUX
#include <linux/cypress_innolux_i2c_ts.h>
#endif


#ifdef CONFIG_HUAWEI_MSM_VIBRATOR
#include "msm_vibrator.h"
#endif

#ifdef CONFIG_HUAWEI_JOGBALL
#include "jogball_device.h"
#endif
#ifdef CONFIG_USB_AUTO_PID_ADAPTER
#include "../../../drivers/usb/function/usb_switch_huawei.h"
#include "../../../arch/arm/mach-msm/proc_comm.h"
#include "smd_private.h"

#define USB_SERIAL_LEN 20
static unsigned char usb_serial_num[USB_SERIAL_LEN]={'H', 'W'};

/* keep the boot mode transfered from APPSBL */
unsigned int usb_boot_mode = 0;

/* keep usb parameters transfered from modem */
app_usb_para usb_para_info;

/* all the pid used by mobile */
usb_pid_stru usb_pid_array[]={
    {PID_ONLY_CDROM,     PID_NORMAL,     PID_UDISK, PID_AUTH},     /* for COMMON products */
    {PID_ONLY_CDROM_TMO, PID_NORMAL_TMO, PID_UDISK, PID_AUTH_TMO}, /* for TMO products */
};

/* pointer to the member of usb_pid_array[], according to the current product */
usb_pid_stru *curr_usb_pid_ptr = &usb_pid_array[0];

void set_usb_sn(char *sn_ptr);

#endif 

#ifdef CONFIG_TOUCHSCREEN_MELFAS
#include <linux/melfas_i2c_ts.h>
#endif

#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_I2C_RMI_TM1319
#include <linux/synaptics_i2c_rmi.h>
#endif

#ifdef CONFIG_MSM_STACKED_MEMORY
#define MSM_SMI_BASE		0x100000
#define MSM_SMI_SIZE		0x800000

#define MSM_PMEM_GPU0_BASE	MSM_SMI_BASE
#ifdef CONFIG_MSM_HW3D
#define MSM_PMEM_GPU0_SIZE	0x700000
#else
#define MSM_PMEM_GPU0_SIZE	0x800000
#endif
#endif

#ifndef CONFIG_HUAWEI_SMI_64M 
#define MSM_PMEM_MDP_SIZE	0x800000
#define MSM_PMEM_CAMERA_SIZE	0xa00000
#define MSM_PMEM_ADSP_SIZE	0x800000
#define MSM_PMEM_GPU1_SIZE	0x800000
#define MSM_FB_SIZE		0x200000

#define PMEM_KERNEL_EBI1_SIZE	0x200000
#else //CONFIG_HUAWEI_SMI_64M

//              ______________________
//             ^         |	FB 2M		|0x400,0000
//             |		|_______________|0x3E0,0000
//             |          |	ADSP 12M	|
//            32M	|_______________|0x320,0000
//             |          |				|
//             |		|				|
//             |          |				|
//             |          |	MODEM 19M	|
//             V_____ |_______________|0x200,0000


/*GPU1 can't migrate to SMI  */

#define MSM_PMEM_GPU1_SIZE	0x800000

#define  MSM_FB_BASE      0x3E00000
#define  MSM_FB_SIZE	0x200000	// 2 M

//#define MSM_PMEM_CAMERA_BASE 0x3400000
#define MSM_PMEM_CAMERA_SIZE	0xa00000 //10M

#define MSM_PMEM_ADSP_BASE    0x3200000
#define MSM_PMEM_ADSP_SIZE    0xc00000 //12M

//#define  MSM_PMEM_MDP_BASE   0x2300000
#define MSM_PMEM_MDP_SIZE	0x800000
#define PMEM_KERNEL_EBI1_SIZE	0x200000
#endif

#ifdef CONFIG_MSM_HW3D
#define MSM_PMEM_GPU1_BASE  0x17800000//0x10000000+121*1204*1024,
static struct msm_hw3d_meminfo hw3d_gpu_setting = {
	.pmem_gpu0_start = MSM_PMEM_GPU0_BASE,
	.pmem_gpu0_size =  MSM_PMEM_GPU0_SIZE,
	.pmem_gpu1_start = MSM_PMEM_GPU1_BASE,	//no matter, any value is ok
	.pmem_gpu1_size = MSM_PMEM_GPU1_SIZE,	//no matter, any value is ok
};
#endif


static struct resource smc91x_resources[] = {
	[0] = {
		.start	= 0x9C004300,
		.end	= 0x9C0043ff,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= MSM_GPIO_TO_INT(49),
		.end	= MSM_GPIO_TO_INT(49),
		.flags	= IORESOURCE_IRQ,
	},
};

#ifdef CONFIG_USB_FUNCTION
#ifndef CONFIG_USB_AUTO_PID_ADAPTER
static struct usb_mass_storage_platform_data usb_mass_storage_pdata = {
#ifndef	CONFIG_HUAWEI_USB_FUNCTION
	.nluns          = 0x02,
	.buf_size       = 16384,
	.vendor         = "GOOGLE",
	.product        = "Mass storage",
#elif defined(CONFIG_HUAWEI_USB_FUNCTION_TMO)
	.nluns          = 0x01,
	.buf_size       = 16384,
	.vendor 		= "T-Mobile",
	.product		= "3G Phone",
#else
    .nluns          = 0x01,
    .buf_size       = 16384,
    .vendor         = "Android",
    .product        = "Adapter",
#endif	
	.release        = 0xffff,
};
#else
/* add a lun for MS in autorun feature */
static struct usb_mass_storage_platform_data usb_mass_storage_pdata = {
    .nluns          = 0x01,
    .buf_size       = 16384,
    .vendor         = "Android",
    .product        = "Adapter",
	.release        = 0xffff,
};

static struct usb_mass_storage_platform_data usb_mass_storage_tmo_pdata = {
    .nluns          = 0x01,
    .buf_size       = 16384,
    .vendor         = "T-Mobile",
    .product        = "3G Phone",
	.release        = 0xffff,
};
#endif

static struct platform_device mass_storage_device = {
	.name           = "usb_mass_storage",
	.id             = -1,
	.dev            = {
		.platform_data          = &usb_mass_storage_pdata,
	},
};
#endif

#ifdef CONFIG_USB_ANDROID
static struct android_usb_platform_data android_usb_pdata = {
	.vendor_id	= 0x05C6,
	.product_id	= 0xF000,
	.adb_product_id	= 0x9015,
	.version	= 0x0100,
	.product_name	= "Qualcomm HSUSB Device",
	.manufacturer_name = "Qualcomm Incorporated",
	.nluns = 1,
};

static struct platform_device android_usb_device = {
	.name	= "android_usb",
	.id		= -1,
	.dev		= {
		.platform_data = &android_usb_pdata,
	},
};
#endif

static struct platform_device hs_device = {
	.name   = "msm-handset",
	.id     = -1,
};

#ifdef CONFIG_USB_FUNCTION
static void hsusb_gpio_init(void)
{
	if (gpio_request(111, "ulpi_data_0"))
		pr_err("failed to request gpio ulpi_data_0\n");
	if (gpio_request(112, "ulpi_data_1"))
		pr_err("failed to request gpio ulpi_data_1\n");
	if (gpio_request(113, "ulpi_data_2"))
		pr_err("failed to request gpio ulpi_data_2\n");
	if (gpio_request(114, "ulpi_data_3"))
		pr_err("failed to request gpio ulpi_data_3\n");
	if (gpio_request(115, "ulpi_data_4"))
		pr_err("failed to request gpio ulpi_data_4\n");
	if (gpio_request(116, "ulpi_data_5"))
		pr_err("failed to request gpio ulpi_data_5\n");
	if (gpio_request(117, "ulpi_data_6"))
		pr_err("failed to request gpio ulpi_data_6\n");
	if (gpio_request(118, "ulpi_data_7"))
		pr_err("failed to request gpio ulpi_data_7\n");
	if (gpio_request(119, "ulpi_dir"))
		pr_err("failed to request gpio ulpi_dir\n");
	if (gpio_request(120, "ulpi_next"))
		pr_err("failed to request gpio ulpi_next\n");
	if (gpio_request(121, "ulpi_stop"))
		pr_err("failed to request gpio ulpi_stop\n");
}

static unsigned usb_gpio_lpm_config[] = {
	GPIO_CFG(111, 1, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA),	/* DATA 0 */
	GPIO_CFG(112, 1, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA),	/* DATA 1 */
	GPIO_CFG(113, 1, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA),	/* DATA 2 */
	GPIO_CFG(114, 1, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA),	/* DATA 3 */
	GPIO_CFG(115, 1, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA),	/* DATA 4 */
	GPIO_CFG(116, 1, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA),	/* DATA 5 */
	GPIO_CFG(117, 1, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA),	/* DATA 6 */
	GPIO_CFG(118, 1, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA),	/* DATA 7 */
	GPIO_CFG(119, 1, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA),	/* DIR */
	GPIO_CFG(120, 1, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA),	/* NEXT */
	GPIO_CFG(121, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),	/* STOP */
};

static unsigned usb_gpio_lpm_unconfig[] = {
	GPIO_CFG(111, 1, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DATA 0 */
	GPIO_CFG(112, 1, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DATA 1 */
	GPIO_CFG(113, 1, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DATA 2 */
	GPIO_CFG(114, 1, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DATA 3 */
	GPIO_CFG(115, 1, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DATA 4 */
	GPIO_CFG(116, 1, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DATA 5 */
	GPIO_CFG(117, 1, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DATA 6 */
	GPIO_CFG(118, 1, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DATA 7 */
	GPIO_CFG(119, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DIR */
	GPIO_CFG(120, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* NEXT */
	GPIO_CFG(121, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_2MA), /* STOP */
};

static int usb_config_gpio(int config)
{
	int pin, rc;

	if (config) {
		for (pin = 0; pin < ARRAY_SIZE(usb_gpio_lpm_config); pin++) {
			rc = gpio_tlmm_config(usb_gpio_lpm_config[pin],
					      GPIO_ENABLE);
			if (rc) {
				printk(KERN_ERR
				       "%s: gpio_tlmm_config(%#x)=%d\n",
				       __func__, usb_gpio_lpm_config[pin], rc);
				return -EIO;
			}
		}
	} else {
		for (pin = 0; pin < ARRAY_SIZE(usb_gpio_lpm_unconfig); pin++) {
			rc = gpio_tlmm_config(usb_gpio_lpm_unconfig[pin],
					      GPIO_ENABLE);
			if (rc) {
				printk(KERN_ERR
				       "%s: gpio_tlmm_config(%#x)=%d\n",
				       __func__, usb_gpio_lpm_config[pin], rc);
				return -EIO;
			}
		}
	}

	return 0;
}
#endif


static struct platform_device smc91x_device = {
	.name		= "smc91x",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(smc91x_resources),
	.resource	= smc91x_resources,
};

#ifdef CONFIG_USB_FUNCTION
#ifndef CONFIG_USB_AUTO_PID_ADAPTER
static struct usb_function_map usb_functions_map[] = {
#ifndef CONFIG_HUAWEI_USB_FUNCTION
	{"diag", 0},
	{"adb", 1},
	{"modem", 2},
	{"nmea", 3},
	{"mass_storage", 4},
	{"ethernet", 5},
#else  //else HUAWEI_USB_FUNCTION
	{"modem", 0},
	#ifdef CONFIG_HUAWEI_USB_FUNCTION_PCUI
	{"pcui", 1},
	#endif // endif CONFIG_HUAWEI_USB_FUNCTION_PCUI
	{"mass_storage", 2},
	{"adb", 3},
	{"diag", 4},
	#ifdef CONFIG_HUAWEI_USB_CONSOLE
	{"serial_console", 5},
	#endif    
#endif	// end of HUAWEI_USB_FUNCTION
};

/* dynamic composition */
static struct usb_composition usb_func_composition[] = {
	{
		.product_id         = 0x9012,
		.functions	    = 0x5, /* 0101 */
	},

	{
		.product_id         = 0x9013,
		.functions	    = 0x15, /* 10101 */
	},

	{
		.product_id         = 0x9014,
		.functions	    = 0x30, /* 110000 */
	},

	{
		.product_id         = 0x9015,
		.functions          = 0x12, /* 10010 */
	},

	{
		.product_id         = 0x9016,
		.functions	    = 0xD, /* 01101 */
	},

	{
		.product_id         = 0x9017,
		.functions	    = 0x1D, /* 11101 */
	},

	{
		.product_id         = 0xF000,
		.functions	    = 0x10, /* 10000 */
	},

	{
		.product_id         = 0xF009,
		.functions	    = 0x20, /* 100000 */
	},

	{
#ifndef CONFIG_HUAWEI_USB_FUNCTION	    
		.product_id         = 0x9018,
#elif defined(CONFIG_HUAWEI_USB_FUNCTION_TMO)
		.product_id         = PID_NORMAL_TMO,
#else
        	.product_id         = PID_NORMAL,
#endif
#ifdef CONFIG_HUAWEI_USB_CONSOLE
	   .functions	    = 0x3F, /* 00111111 */
#else  // else CONFIG_HUAWEI_USB_CONSOLE
		.functions	    = 0x1F, /* 011111 */
#endif // endif CONFIG_HUAWEI_USB_CONSOLE     
	},

	{
		.product_id         = 0x901A,
		.functions          = 0x0F, /* 01111 */
	},

};
#else
static struct usb_function_map usb_functions_map[] = {
	{"modem", 0},
#ifdef CONFIG_HUAWEI_USB_FUNCTION_PCUI
	{"pcui", 1},
#endif // endif CONFIG_HUAWEI_USB_FUNCTION_PCUI
	{"mass_storage", 2},
	{"adb", 3},
	{"diag", 4},
#ifdef CONFIG_HUAWEI_USB_CONSOLE
	{"serial_console", 5},
#endif    
};

/* dynamic composition */
static struct usb_composition usb_tmo_func_composition[] = {
	{
		.product_id         = 0x9012,
		.functions	    = 0x5, /* 0101 */
	},

	{
		.product_id         = 0x9013,
		.functions	    = 0x15, /* 10101 */
	},

	{
		.product_id         = 0x9014,
		.functions	    = 0x30, /* 110000 */
	},

	{
		.product_id         = 0x9016,
		.functions	    = 0xD, /* 01101 */
	},

	{
		.product_id         = 0x9017,
		.functions	    = 0x1D, /* 11101 */
	},

	{
		.product_id         = 0xF000,
		.functions	    = 0x10, /* 10000 */
	},

	{
		.product_id         = 0xF009,
		.functions	    = 0x20, /* 100000 */
	},
/* add a usb composition for the only MS setting */
#ifdef CONFIG_USB_AUTO_INSTALL
	{
		.product_id         = PID_ONLY_CDROM_TMO,
		.functions	    = 0x04, /* 000100, ONLY CDROM */
	},
	{
		.product_id         = PID_UDISK,
		.functions	    = 0x04, /* 000100, ONLY UDISK */
	},
#endif
	{
        .product_id         = PID_NORMAL_TMO,
#ifdef CONFIG_HUAWEI_DIAG_DEBUG
#ifdef CONFIG_HUAWEI_USB_CONSOLE
	   .functions	    = 0x3F, /* 00111111 */
#else  // else CONFIG_HUAWEI_USB_CONSOLE
       .functions	    = 0x1F, /* 00011111 */
#endif // endif CONFIG_HUAWEI_USB_CONSOLE     

#else  // else CONFIG_HUAWEI_DIAG_DEBUG
       .functions       = 0x0F, /* 00001111 */
#endif // endif CONFIG_HUAWEI_USB_FUNCTION_PCUI
	},
	{
		.product_id     = PID_AUTH_TMO,
		.functions	    = 0x01F, /* 011111 */
	},
};

/* dynamic composition */
static struct usb_composition usb_func_composition[] = {
	{
		.product_id         = 0x9012,
		.functions	    = 0x5, /* 0101 */
	},

	{
		.product_id         = 0x9013,
		.functions	    = 0x15, /* 10101 */
	},

	{
		.product_id         = 0x9014,
		.functions	    = 0x30, /* 110000 */
	},

	{
		.product_id         = 0x9015,
		.functions          = 0x12, /* 10010 */
	},

	{
		.product_id         = 0x9016,
		.functions	    = 0xD, /* 01101 */
	},

	{
		.product_id         = 0x9017,
		.functions	    = 0x1D, /* 11101 */
	},

	{
		.product_id         = 0xF000,
		.functions	    = 0x10, /* 10000 */
	},

	{
		.product_id         = 0xF009,
		.functions	    = 0x20, /* 100000 */
	},

/* add a usb composition for the only MS setting */
#ifdef CONFIG_USB_AUTO_INSTALL
	{
		.product_id         = PID_ONLY_CDROM,
		.functions	    = 0x04, /* 000100, ONLY CDROM */
	},
	{
		.product_id         = PID_UDISK,
		.functions	    = 0x04, /* 000100, ONLY UDISK */
	},
#endif
	{
        .product_id         = PID_NORMAL,
#ifdef CONFIG_HUAWEI_DIAG_DEBUG
#ifdef CONFIG_HUAWEI_USB_CONSOLE
	   .functions	    = 0x3F, /* 00111111 */
#else  // else CONFIG_HUAWEI_USB_CONSOLE
       .functions	    = 0x1F, /* 00011111 */
#endif // endif CONFIG_HUAWEI_USB_CONSOLE     

#else  // else CONFIG_HUAWEI_DIAG_DEBUG
       .functions       = 0x0F, /* 00001111 */
#endif // endif CONFIG_HUAWEI_USB_FUNCTION_PCUI
	},
	{
		.product_id     = PID_AUTH,
		.functions	    = 0x01F, /* 011111 */
	},
};
#endif /*CONFIG_USB_AUTO_PID_ADAPTER*/
#endif
#ifdef CONFIG_USB_ANDROID
static void hsusb_phy_reset(void)
{
	msm_hsusb_phy_reset();
}
#endif

#ifndef CONFIG_USB_AUTO_PID_ADAPTER
static struct msm_hsusb_platform_data msm_hsusb_pdata = {
#ifdef CONFIG_USB_ANDROID
	.phy_reset	= hsusb_phy_reset,
#endif
#ifdef CONFIG_USB_FUNCTION
	.version	= 0x0100,
	.phy_info	= USB_PHY_EXTERNAL,
#ifndef CONFIG_HUAWEI_USB_FUNCTION	
	.vendor_id          = 0x5c6,
	.product_name       = "Qualcomm HSUSB Device",
	.serial_number      = "1234567890ABCDEF",
	.manufacturer_name  = "Qualcomm Incorporated",
#elif defined(CONFIG_HUAWEI_USB_FUNCTION_TMO)	
	.vendor_id			= 0x12D1,
	.product_name		= "T-Mobile 3G Phone",
	.serial_number		= NULL,
	.manufacturer_name	= "Huawei Incorporated",
#else
    .vendor_id          = 0x12D1,
    .product_name       = "Android Mobile Adapter",
    .serial_number      = NULL,
    .manufacturer_name  = "Huawei Incorporated",
#endif
	.compositions	= usb_func_composition,
	.num_compositions = ARRAY_SIZE(usb_func_composition),
	.function_map   = usb_functions_map,
	.num_functions	= ARRAY_SIZE(usb_functions_map),
	.ulpi_data_1_pin = 112,
	.ulpi_data_3_pin = 114,
	.config_gpio 	= usb_config_gpio,
#endif
};
#else
static struct msm_hsusb_platform_data msm_hsusb_pdata = {
#ifdef CONFIG_USB_FUNCTION
	.version	= 0x0100,
	.phy_info	= USB_PHY_EXTERNAL,
    .vendor_id          = 0x12D1,
    .product_name       = "Android Mobile Adapter",
    .serial_number      = NULL,
    .manufacturer_name  = "Huawei Incorporated",
	.compositions	= usb_func_composition,
	.num_compositions = ARRAY_SIZE(usb_func_composition),
	.function_map   = usb_functions_map,
	.num_functions	= ARRAY_SIZE(usb_functions_map),
    .ulpi_data_1_pin = 112,
    .ulpi_data_3_pin = 114,
	.config_gpio    = usb_config_gpio,
#endif
};

static struct msm_hsusb_platform_data msm_hsusb_tmo_pdata = {
#ifdef CONFIG_USB_FUNCTION
	.version	= 0x0100,
	.phy_info	= USB_PHY_EXTERNAL,
    .vendor_id          = 0x12D1,
    .product_name       = "T-Mobile 3G Phone",
    .serial_number      = NULL,
    .manufacturer_name  = "Huawei Incorporated",
	.compositions	= usb_tmo_func_composition,
	.num_compositions = ARRAY_SIZE(usb_func_composition),
	.function_map   = usb_functions_map,
	.num_functions	= ARRAY_SIZE(usb_functions_map),
    .ulpi_data_1_pin = 112,
    .ulpi_data_3_pin = 114,
	.config_gpio    = usb_config_gpio,
#endif
};
#endif
static struct i2c_board_info i2c_devices[] = {
#ifndef CONFIG_HUAWEI_CAMERA
#ifdef CONFIG_MT9D112
	{
		I2C_BOARD_INFO("mt9d112", 0x78 >> 1),
	},
#endif
#ifdef CONFIG_S5K3E2FX
	{
		I2C_BOARD_INFO("s5k3e2fx", 0x20 >> 1),
	},
#endif
#ifdef CONFIG_MT9P012
	{
		I2C_BOARD_INFO("mt9p012", 0x6C >> 1),
	},
#endif
#if defined(CONFIG_MT9T013) || defined(CONFIG_SENSORS_MT9T013)
	{
		I2C_BOARD_INFO("mt9t013", 0x6C),
	},
#endif
#else /*CONFIG_HUAWEI_CAMERA*/
	{
		I2C_BOARD_INFO("mt9t013_liteon", 0x6C >> 1),
	},
	{
		I2C_BOARD_INFO("mt9t013_byd", 0x6C),
	},
#ifdef CONFIG_HUAWEI_CAMERA_SENSOR_OV3647
	{
		I2C_BOARD_INFO("ov3647", 0x90 >> 1),
	},
#endif //CONFIG_HUAWEI_CAMERA_SENSOR_OV3647

#ifdef CONFIG_HUAWEI_CAMERA_SENSOR_OV7690
	{
		I2C_BOARD_INFO("ov7690", 0x42 >> 1),
	},
#endif //CONFIG_HUAWEI_CAMERA_SENSOR_OV7690

#endif //CONFIG_HUAWEI_CAMERA

#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CPT
	{
		I2C_BOARD_INFO("cpt_ts", 0x0a),	
		.irq = MSM_GPIO_TO_INT(57)
	},
#endif //CONFIG_TOUCHSCREEN_CYPRESS_CPT

#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_I2C_RMI_TM1319
        {
                I2C_BOARD_INFO(SYNAPTICS_I2C_RMI_NAME, 0x20),                
                .irq = MSM_GPIO_TO_INT(57)
        },
#endif
#ifdef CONFIG_TOUCHSCREEN_INNOLUX
        {
                I2C_BOARD_INFO(CYPRESS_I2C_NAME, 0x25),                
                .irq = MSM_GPIO_TO_INT(57)  /*gpio 57 is interupt for touchscreen.*/
        },
#endif

#ifdef CONFIG_TOUCHSCREEN_MELFAS
        {
                I2C_BOARD_INFO(MELFAS_I2C_NAME, 0x22),                
                .irq = MSM_GPIO_TO_INT(57)  /*gpio 57 is interupt for touchscreen.*/
        },
#endif

#ifdef CONFIG_ACCELEROMETER_ADXL345
	{
	   I2C_BOARD_INFO("GS", 0xA6 >> 1),	  
	    .irq = MSM_GPIO_TO_INT(31)
	},	
#endif

#ifdef CONFIG_ACCELEROMETER_ST_L1S35DE
	{
	    I2C_BOARD_INFO("gs_st", 0x3a >> 1),	  
	   .irq = MSM_GPIO_TO_INT(31)
	},	  
#endif

#ifdef CONFIG_ACCELEROMETER_MMA7455L

	{
	    I2C_BOARD_INFO("freescale", 0x1c),	  
	    .irq = MSM_GPIO_TO_INT(31)
	},
#endif	

#ifdef CONFIG_SENSORS_AKM8973
	{
		I2C_BOARD_INFO("akm8973", 0x3c>>1),//7 bit addr, no write bit
	 	.irq = MSM_GPIO_TO_INT(88)
	}	
#endif 
};

#ifdef CONFIG_MSM_CAMERA
static uint32_t camera_off_gpio_table[] = {
	/* parallel CAMERA interfaces */
	GPIO_CFG(0,  0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT0 */
	GPIO_CFG(1,  0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT1 */
	GPIO_CFG(2,  0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT2 */
	GPIO_CFG(3,  0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT3 */
	GPIO_CFG(4,  0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT4 */
	GPIO_CFG(5,  0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT5 */
	GPIO_CFG(6,  0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT6 */
	GPIO_CFG(7,  0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT7 */
	GPIO_CFG(8,  0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT8 */
	GPIO_CFG(9,  0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT9 */
	GPIO_CFG(10, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT10 */
	GPIO_CFG(11, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT11 */
	GPIO_CFG(12, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* PCLK */
	GPIO_CFG(13, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* HSYNC_IN */
	GPIO_CFG(14, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* VSYNC_IN */
	GPIO_CFG(15, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA), /* MCLK */
};

static uint32_t camera_on_gpio_table[] = {
	/* parallel CAMERA interfaces */
#ifndef CONFIG_HUAWEI_CAMERA
	GPIO_CFG(0,  1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT0 */
	GPIO_CFG(1,  1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT1 */
	GPIO_CFG(2,  1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT2 */
	GPIO_CFG(3,  1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT3 */
	GPIO_CFG(4,  1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT4 */
	GPIO_CFG(5,  1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT5 */
	GPIO_CFG(6,  1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT6 */
	GPIO_CFG(7,  1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT7 */
	GPIO_CFG(8,  1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT8 */
	GPIO_CFG(9,  1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT9 */
	GPIO_CFG(10, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT10 */
	GPIO_CFG(11, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT11 */
	GPIO_CFG(12, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_16MA), /* PCLK */
	GPIO_CFG(13, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* HSYNC_IN */
	GPIO_CFG(14, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* VSYNC_IN */
	GPIO_CFG(15, 1, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_16MA), /* MCLK */
#else //#CONFIG_HUAWEI_CAMERA
  GPIO_CFG(0,  1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT0 */
   GPIO_CFG(1,  1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT1 */
   GPIO_CFG(2,  1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT2 */
   GPIO_CFG(3,  1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT3 */
   GPIO_CFG(4,  1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT4 */
   GPIO_CFG(5,  1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT5 */
   GPIO_CFG(6,  1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT6 */
   GPIO_CFG(7,  1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT7 */
   GPIO_CFG(8,  1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT8 */
   GPIO_CFG(9,  1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT9 */
   GPIO_CFG(10, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT10 */
   GPIO_CFG(11, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT11 */
   GPIO_CFG(12, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_16MA), /* PCLK */
   GPIO_CFG(13, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* HSYNC_IN */
   GPIO_CFG(14, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* VSYNC_IN */
   GPIO_CFG(15, 1, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_16MA), /* MCLK */
   GPIO_CFG(21, 0, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA), /* CAMIF_SHDN_INS */
   GPIO_CFG(29, 0, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA), /* CAMIF_SHDN_OUTS */
   GPIO_CFG(89, 0, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA), /* reset */
   GPIO_CFG(109, 0, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA), /* vcm */
   GPIO_CFG(30, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* module */
#endif //#CONFIG_HUAWEI_CAMERA
};

static void config_gpio_table(uint32_t *table, int len)
{
	int n, rc;
	for (n = 0; n < len; n++) {
		rc = gpio_tlmm_config(table[n], GPIO_ENABLE);
		if (rc) {
			printk(KERN_ERR "%s: gpio_tlmm_config(%#x)=%d\n",
				__func__, table[n], rc);
			break;
		}
	}
}

static void config_camera_on_gpios(void)
{
	config_gpio_table(camera_on_gpio_table,
		ARRAY_SIZE(camera_on_gpio_table));
}

static void config_camera_off_gpios(void)
{
	config_gpio_table(camera_off_gpio_table,
		ARRAY_SIZE(camera_off_gpio_table));
}

#ifdef CONFIG_HUAWEI_CAMERA
static int32_t sensor_vreg_enable
(
    struct msm_camera_sensor_vreg *sensor_vreg,
    uint8_t vreg_num
)
{
    struct vreg *vreg_handle;
    uint8_t temp_vreg_sum;
    int32_t rc;
    
    if(sensor_vreg == NULL)
    {
        return 0;
    }
    
    for(temp_vreg_sum = 0; temp_vreg_sum < vreg_num;temp_vreg_sum++)
    {
        vreg_handle = vreg_get(0, sensor_vreg[temp_vreg_sum].vreg_name);
    	if (!vreg_handle) {
    		printk(KERN_ERR "vreg_handle get failed\n");
    		return -EIO;
    	}
    	rc = vreg_set_level(vreg_handle, sensor_vreg[temp_vreg_sum].mv);
    	if (rc) {
    		printk(KERN_ERR "vreg_handle set level failed\n");
    		return -EIO;
    	}
    	rc = vreg_enable(vreg_handle);
    	if (rc) {
    		printk(KERN_ERR "vreg_handle enable failed\n");
    		return -EIO;
    	}
    }
    return 0;
}

static int32_t sensor_vreg_disable
(
    struct msm_camera_sensor_vreg *sensor_vreg,
    uint8_t vreg_num
)
{
    struct vreg *vreg_handle;
    uint8_t temp_vreg_sum;
    int32_t rc;
    
    if(sensor_vreg == NULL)
    {
        return 0;
    }

    for(temp_vreg_sum = 0; temp_vreg_sum < vreg_num;temp_vreg_sum++)
    {
        vreg_handle = vreg_get(0, sensor_vreg[temp_vreg_sum].vreg_name);
    	if (!vreg_handle) {
    		printk(KERN_ERR "vreg_handle get failed\n");
    		return -EIO;
    	}
    	rc = vreg_disable(vreg_handle);
    	if (rc) {
    		printk(KERN_ERR "vreg_handle disable failed\n");
    		return -EIO;
    	}
    }
    return 0;
}
 struct msm_camera_sensor_vreg sensor_vreg_array[] = {
    {
		.vreg_name   = "gp3",
		.mv	  = 2600,
	}, 
    {
		.vreg_name   = "gp1",
		.mv	  = 2800,
	},    
    {
		.vreg_name   = "gp2",
		.mv	  = 1800,
	},
   
};
#endif //CONFIG_HUAWEI_CAMERA

static struct msm_camera_device_platform_data msm_camera_device_data = {
	.camera_gpio_on  = config_camera_on_gpios,
	.camera_gpio_off = config_camera_off_gpios,
	.ioext.mdcphy = MSM_MDC_PHYS,
	.ioext.mdcsz  = MSM_MDC_SIZE,
	.ioext.appphy = MSM_CLK_CTL_PHYS,
	.ioext.appsz  = MSM_CLK_CTL_SIZE,
};

#ifndef CONFIG_HUAWEI_CAMERA

#ifdef CONFIG_MT9D112
static struct msm_camera_sensor_info msm_camera_sensor_mt9d112_data = {
	.sensor_name	= "mt9d112",
	.sensor_reset	= 89,
	.sensor_pwd	= 85,
	.vcm_pwd	= 0,
	.pdata		= &msm_camera_device_data,
	.flash_type     = MSM_CAMERA_FLASH_LED
};

static struct platform_device msm_camera_sensor_mt9d112 = {
	.name	   = "msm_camera_mt9d112",
	.dev	    = {
		.platform_data = &msm_camera_sensor_mt9d112_data,
	},
};
#endif

#ifdef CONFIG_S5K3E2FX
static struct msm_camera_sensor_info msm_camera_sensor_s5k3e2fx_data = {
	.sensor_name	= "s5k3e2fx",
	.sensor_reset	= 89,
	.sensor_pwd	= 85,
	.vcm_pwd	= 0,
	.pdata		= &msm_camera_device_data,
	.flash_type     = MSM_CAMERA_FLASH_LED
};

static struct platform_device msm_camera_sensor_s5k3e2fx = {
	.name	   = "msm_camera_s5k3e2fx",
	.dev	    = {
		.platform_data = &msm_camera_sensor_s5k3e2fx_data,
	},
};
#endif

#ifdef CONFIG_MT9P012
static struct msm_camera_sensor_info msm_camera_sensor_mt9p012_data = {
	.sensor_name	= "mt9p012",
	.sensor_reset	= 89,
	.sensor_pwd	= 85,
	.vcm_pwd	= 88,
	.pdata		= &msm_camera_device_data,
	.flash_type     = MSM_CAMERA_FLASH_LED
};

static struct platform_device msm_camera_sensor_mt9p012 = {
	.name	   = "msm_camera_mt9p012",
	.dev	    = {
		.platform_data = &msm_camera_sensor_mt9p012_data,
	},
};
#endif

#ifdef CONFIG_MT9T013
static struct msm_camera_sensor_info msm_camera_sensor_mt9t013_data = {
	.sensor_name	= "mt9t013",
	.sensor_reset	= 89,
	.sensor_pwd	= 85,
	.vcm_pwd	= 0,
	.pdata		= &msm_camera_device_data,
	.flash_type     = MSM_CAMERA_FLASH_LED
};

static struct platform_device msm_camera_sensor_mt9t013 = {
	.name	   = "msm_camera_mt9t013",
	.dev	    = {
		.platform_data = &msm_camera_sensor_mt9t013_data,
	},
};
#endif
#else //CONFIG_HUAWEI_CAMERA

#ifdef CONFIG_MT9T013
static struct msm_camera_sensor_info msm_camera_sensor_mt9t013_byd_data = {
	.sensor_name	= "mt9t013_byd",
	.sensor_reset	= 89,
	.sensor_pwd	= /*17*/29,
	.vcm_pwd	= /*0*/109,
	.pdata		= &msm_camera_device_data,
    .flash_type		= MSM_CAMERA_FLASH_NONE,
#ifdef CONFIG_HUAWEI_CAMERA
        .sensor_module_id  = 30,
        .sensor_module_value  = 1,
        .sensor_vreg  = sensor_vreg_array,
        .vreg_num     = ARRAY_SIZE(sensor_vreg_array),
        .vreg_enable_func = sensor_vreg_enable,
        .vreg_disable_func = sensor_vreg_disable,
        .slave_sensor = 0,
//        .master_init_control_slave = sensor_master_init_control_slave,
#endif        
};


static struct platform_device msm_camera_sensor_mt9t013_byd = {
	.name	   = "msm_camera_byd3m",
	.id        = -1,
	.dev	    = {
		.platform_data = &msm_camera_sensor_mt9t013_byd_data,
	},
};
#endif //CONFIG_MT9T013

#ifdef CONFIG_MT9T013
static struct msm_camera_sensor_info msm_camera_sensor_mt9t013_liteon_data = {
	.sensor_name	= "mt9t013_liteon",
	.sensor_reset	= 89,
	.sensor_pwd	= /*17*/29,
	.vcm_pwd	= 109,
	.pdata		= &msm_camera_device_data,
    .flash_type		= MSM_CAMERA_FLASH_NONE,
#ifdef CONFIG_HUAWEI_CAMERA
        .sensor_module_id  = 30,
        .sensor_module_value  = 0,
        .sensor_vreg  = sensor_vreg_array,
        .vreg_num     = ARRAY_SIZE(sensor_vreg_array),
        .vreg_enable_func = sensor_vreg_enable,
        .vreg_disable_func = sensor_vreg_disable,
        .slave_sensor = 0,
//        .master_init_control_slave = sensor_master_init_control_slave,
#endif        
};

static struct platform_device msm_camera_sensor_mt9t013_liteon = {
	.name	   = "msm_camera_liteon3m",
	.id        = -1,
	.dev	    = {
		.platform_data = &msm_camera_sensor_mt9t013_liteon_data,
	},
};
#endif //CONFIG_MT9T013

#ifdef CONFIG_HUAWEI_CAMERA_SENSOR_OV3647
static struct msm_camera_sensor_info msm_camera_sensor_ov3647_data = {
	.sensor_name	= "ov3647",
	.sensor_reset	= 89,
	.sensor_pwd	= 29,
	.vcm_pwd	= 109,
	.pdata		= &msm_camera_device_data,
    .flash_type		= MSM_CAMERA_FLASH_NONE,
#ifdef CONFIG_HUAWEI_CAMERA
        .sensor_module_id  = 30,
        .sensor_module_value  = 0,
        .sensor_vreg  = sensor_vreg_array,
        .vreg_num     = ARRAY_SIZE(sensor_vreg_array),
        .vreg_enable_func = sensor_vreg_enable,
        .vreg_disable_func = sensor_vreg_disable,
        .slave_sensor = 0,
//        .master_init_control_slave = sensor_master_init_control_slave,
#endif        
};

static struct platform_device msm_camera_sensor_ov3647 = {
	.name	   = "msm_camera_ov3647",
	.id        = -1,
	.dev	    = {
		.platform_data = &msm_camera_sensor_ov3647_data,
	},
};
#endif

#ifdef CONFIG_HUAWEI_CAMERA_SENSOR_OV7690	
static struct msm_camera_sensor_info msm_camera_sensor_ov7690_data = {
	.sensor_name	= "ov7690",
	.sensor_reset	= 89,
	.sensor_pwd	= 21,
	.vcm_pwd	= 0,
	.pdata		= &msm_camera_device_data,
    .flash_type		= MSM_CAMERA_FLASH_NONE,
#ifdef CONFIG_HUAWEI_CAMERA
        .sensor_module_id  = 30,
        .sensor_module_value  = 0,
        .sensor_vreg  = sensor_vreg_array,
        .vreg_num     = ARRAY_SIZE(sensor_vreg_array),
        .vreg_enable_func = sensor_vreg_enable,
        .vreg_disable_func = sensor_vreg_disable,
        .slave_sensor = 1,
//        .master_init_control_slave = sensor_master_init_control_slave,
#endif        
};

static struct platform_device msm_camera_sensor_ov7690 = {
	.name	   = "msm_camera_ov7690",
	.id        = -1,
	.dev	    = {
		.platform_data = &msm_camera_sensor_ov7690_data,
	},
};
#endif
#endif/* CONFIG_HUAWEI_CAMERA*/

#endif /*CONFIG_MSM_CAMERA*/

#define SND(desc, num) { .name = #desc, .id = num }
static struct snd_endpoint snd_endpoints_list[] = {
	SND(HANDSET, 0),
	SND(MONO_HEADSET, 2),
	SND(HEADSET, 3),
	SND(SPEAKER, 6),
	SND(TTY_HEADSET, 8),
	SND(TTY_VCO, 9),
	SND(TTY_HCO, 10),
	SND(BT, 12),
	SND(IN_S_SADC_OUT_HANDSET, 16),
	SND(IN_S_SADC_OUT_SPEAKER_PHONE, 25),
	SND(HEADSET_AND_SPEAKER,26), 
	SND(BT_EC_OFF,27),
	SND(CURRENT, 29),
};
#undef SND

static struct msm_snd_endpoints halibut_snd_endpoints = {
	.endpoints = snd_endpoints_list,
	.num = sizeof(snd_endpoints_list) / sizeof(struct snd_endpoint)
};

static struct platform_device halibut_snd = {
	.name = "msm_snd",
	.id = -1,
	.dev    = {
		.platform_data = &halibut_snd_endpoints
	},
};

#ifdef CONFIG_HUAWEI_EBI_DEVICE
static struct android_pmem_platform_data android_pmem_kernel_ebi1_pdata = {
	.name = PMEM_KERNEL_EBI1_DATA_NAME,
	/* if no allocator_type, defaults to PMEM_ALLOCATORTYPE_BITMAP,
	 * the only valid choice at this time. The board structure is
	 * set to all zeros by the C runtime initialization and that is now
	 * the enum value of PMEM_ALLOCATORTYPE_BITMAP, now forced to 0 in
	 * include/linux/android_pmem.h.
	 */
    .allocator_type = PMEM_ALLOCATORTYPE_BUDDYBESTFIT,
	.cached = 0,
};
#endif
#if 0//CONFIG_MSM_HW3D 
// we modify pmem.c,so remove these pmem devices
static struct android_pmem_platform_data android_pmem_pdata = {
	.name = "pmem",
	.allocator_type = PMEM_ALLOCATORTYPE_BUDDYBESTFIT,
	.cached = 1,
};

static struct android_pmem_platform_data android_pmem_camera_pdata = {
	.name = "pmem_camera",
	.allocator_type = PMEM_ALLOCATORTYPE_ALLORNOTHING,
	.cached = 0,
};

static struct android_pmem_platform_data android_pmem_adsp_pdata = {
	.name = "pmem_adsp",
	.allocator_type = PMEM_ALLOCATORTYPE_BUDDYBESTFIT,
	.cached = 0,
};

#ifdef CONFIG_MSM_STACKED_MEMORY
static struct android_pmem_platform_data android_pmem_gpu0_pdata = {
	.name = "pmem_gpu0",
	.start = MSM_PMEM_GPU0_BASE,
	.size = MSM_PMEM_GPU0_SIZE,
	.allocator_type = PMEM_ALLOCATORTYPE_ALLORNOTHING,
	.cached = 0,
};
#endif

static struct android_pmem_platform_data android_pmem_gpu1_pdata = {
	.name = "pmem_gpu1",
	.allocator_type = PMEM_ALLOCATORTYPE_ALLORNOTHING,
	.cached = 0,
};
#else //CONFIG_MSM_HW3D 
static struct android_pmem_platform_data android_pmem_pdata = {
	.name = "pmem",
	.size = MSM_PMEM_MDP_SIZE,
	.allocator_type = PMEM_ALLOCATORTYPE_BUDDYBESTFIT,
	.cached = 1,
};
#if 0
static struct android_pmem_platform_data android_pmem_camera_pdata = {
	.name = "pmem_camera",
	//.allocator_type = PMEM_ALLOCATORTYPE_ALLORNOTHING,
	.no_allocator = 1,
	.cached = 1,
};
#endif
static struct android_pmem_platform_data android_pmem_adsp_pdata = {
	.name = "pmem_adsp",
	.allocator_type = PMEM_ALLOCATORTYPE_BITMAP,
	.cached = 0,
};
#endif//CONFIG_MSM_HW3D 

static struct platform_device android_pmem_device = {
	.name = "android_pmem",
	.id = 0,
	.dev = { .platform_data = &android_pmem_pdata },
};

static struct platform_device android_pmem_adsp_device = {
	.name = "android_pmem",
	.id = 1,
	.dev = { .platform_data = &android_pmem_adsp_pdata },
};

#if 0//CONFIG_MSM_HW3D
#ifdef CONFIG_MSM_STACKED_MEMORY
static struct platform_device android_pmem_gpu0_device = {
	.name = "android_pmem",
	.id = 2,
	.dev = { .platform_data = &android_pmem_gpu0_pdata },
};
#endif

static struct platform_device android_pmem_gpu1_device = {
	.name = "android_pmem",
	.id = 3,
	.dev = { .platform_data = &android_pmem_gpu1_pdata },
};
static struct platform_device android_pmem_camera_device = {
	.name = "android_pmem",
	.id = 4,
	.dev = { .platform_data = &android_pmem_camera_pdata },
};
#else//CONFIG_MSM_HW3D
#if 0
static struct platform_device android_pmem_camera_device = {
	.name = "android_pmem",
	.id = 2,
	.dev = { .platform_data = &android_pmem_camera_pdata },
};
#endif
#endif//CONFIG_MSM_HW3D

#ifdef CONFIG_HUAWEI_EBI_DEVICE
static struct platform_device android_pmem_kernel_ebi1_device = {
	.name = "android_pmem",
	.id = 5,
	.dev = { .platform_data = &android_pmem_kernel_ebi1_pdata },
};
#endif

#if defined(CONFIG_FB_MSM_MDDI_TC358721XBG_VGA_QCIF) || \
	defined(CONFIG_FB_MSM_MDDI_TC358723XBG_VGA_QCIF)	
static void msm_fb_mddi_power_save(int on)
{           
    
} 
#else
static char *msm_fb_vreg[] = {
	"gp5"
};

#define MSM_FB_VREG_OP(name, op) \
do { \
	vreg = vreg_get(0, name); \
	if (vreg_##op(vreg)) \
		printk(KERN_ERR "%s: %s vreg operation failed \n", \
			(vreg_##op == vreg_enable) ? "vreg_enable" \
				: "vreg_disable", name); \
} while (0)

static int mddi_power_save_on;
static void msm_fb_mddi_power_save(int on)
{
	struct vreg *vreg;
	int i;
	int flag_on = !!on;

	if (mddi_power_save_on == flag_on)
		return;

	mddi_power_save_on = flag_on;

	if (machine_is_msm7201a_ffa())
		gpio_direction_output(88, flag_on);

	for (i = 0; i < ARRAY_SIZE(msm_fb_vreg); i++) {
		if (flag_on)
			MSM_FB_VREG_OP(msm_fb_vreg[i], enable);
		else
			MSM_FB_VREG_OP(msm_fb_vreg[i], disable);
	}
}
#endif



#define PM_VID_EN_CONFIG_PROC          24
#define PM_VID_EN_API_PROG             0x30000061
#define PM_VID_EN_API_VERS             0x00010001

static struct msm_rpc_endpoint *pm_vid_en_ep;

static int msm_fb_pm_vid_en(int on)
{
	int rc = 0;
	struct msm_fb_pm_vid_en_req {
		struct rpc_request_hdr hdr;
		uint32_t on;
	} req;

	pm_vid_en_ep = msm_rpc_connect(PM_VID_EN_API_PROG,
					PM_VID_EN_API_VERS, 0);
	if (IS_ERR(pm_vid_en_ep)) {
		printk(KERN_ERR "%s: msm_rpc_connect failed! rc = %ld\n",
			__func__, PTR_ERR(pm_vid_en_ep));
		return -EINVAL;
	}

	req.on = cpu_to_be32(on);
	rc = msm_rpc_call(pm_vid_en_ep,
			PM_VID_EN_CONFIG_PROC,
			&req, sizeof(req),
			5 * HZ);
	if (rc)
		printk(KERN_ERR
			"%s: msm_rpc_call failed! rc = %d\n", __func__, rc);

	msm_rpc_close(pm_vid_en_ep);
	return rc;
}

static int mddi_get_panel_num(void)
{
	if (machine_is_msm7201a_surf())
		return 2;
	else
		return 1;
}

static int mddi_toshiba_backlight_level(int level, int max, int min)
{
	int out_val;

	if (!max)
		return 0;

	if (machine_is_msm7201a_ffa()) {
		out_val = 2200 + (((max - level) * (4000 - 2200)) / max);
	} else {
		out_val = (level * 4999) / max;
	}

	return out_val;
}

static int mddi_sharp_backlight_level(int level, int max, int min)
{
	if (machine_is_msm7201a_ffa())
		return level;
	else
		return -1;
}
static uint32_t mddi_get_panel_type(void)
{
    uint32_t *lcd_type_p = (uint32_t*)smem_alloc(SMEM_LCD_CUR_PANEL, sizeof(uint32_t));
	if(lcd_type_p)
	{
        return *lcd_type_p;
	}
    else
    {
        return 0;
    }

}
static struct tvenc_platform_data tvenc_pdata = {
	.pm_vid_en = msm_fb_pm_vid_en,
};

static struct mddi_platform_data mddi_pdata = {
	.mddi_power_save = msm_fb_mddi_power_save,
};

static struct msm_panel_common_pdata mddi_toshiba_pdata = {
	.backlight_level = mddi_toshiba_backlight_level,
	.panel_num = mddi_get_panel_num,
};


#ifdef CONFIG_FB_MSM_MDDI_TC358721XBG_VGA_QCIF
static struct msm_panel_common_pdata mddi_tc358721xbg_pdata = {
	.backlight_level = mddi_tc358721xbg_backlight_level,
};
#endif // CONFIG_FB_MSM_MDDI_TC358721XBG_VGA_QCIF

#ifdef CONFIG_FB_MSM_MDDI_TC358723XBG_VGA_QCIF
static int mddi_tc358723xbg_backlight_level(int level ,int max,int min)
{
	//0~15
	return level;
}

static struct msm_panel_common_pdata mddi_tc358723xbg_pdata = {
	.backlight_level = mddi_tc358723xbg_backlight_level,
	.get_panel_type = mddi_get_panel_type,  
};
#endif // CONFIG_FB_MSM_MDDI_TC358723XBG_VGA_QCIF

static struct msm_panel_common_pdata mddi_sharp_pdata = {
	.backlight_level = mddi_sharp_backlight_level,
};

static struct resource msm_fb_resources[] = {
	{
		.flags  = IORESOURCE_DMA,
	}
};

static struct platform_device msm_fb_device = {
	.name   = "msm_fb",
	.id     = 0,
	.num_resources  = ARRAY_SIZE(msm_fb_resources),
	.resource       = msm_fb_resources,
};

static struct platform_device mddi_toshiba_device = {
	.name   = "mddi_toshiba",
	.id     = 0,
	.dev    = {
		.platform_data = &mddi_toshiba_pdata,
	}
};

#ifdef CONFIG_FB_MSM_MDDI_TC358721XBG_VGA_QCIF
static struct platform_device mddi_tc358721xbg_device = {
	.name   = "mddi_tc21xbg_vga",
	.id     = 0,
	.dev    = {
		.platform_data = &mddi_tc358721xbg_pdata,
	}
};
#endif //CONFIG_FB_MSM_MDDI_TC358721XBG_VGA_QCIF

#ifdef CONFIG_FB_MSM_MDDI_TC358723XBG_VGA_QCIF
static struct platform_device mddi_tc358723xbg_device = {
	.name   = "mddi_tc23xbg_vga",
	.id     = 0,
	.dev    = {
		.platform_data = &mddi_tc358723xbg_pdata,
	}
};
#endif //CONFIG_FB_MSM_MDDI_TC358723XBG_VGA_QCIF

static struct platform_device mddi_sharp_device = {
	.name   = "mddi_sharp_qvga",
	.id     = 0,
	.dev    = {
		.platform_data = &mddi_sharp_pdata,
	}
};

#ifdef CONFIG_BT
static struct platform_device msm_bt_power_device = {
	.name = "bt_power",
};

enum {
	BT_WAKE,
	BT_RFR,
	BT_CTS,
	BT_RX,
	BT_TX,
	BT_PCM_DOUT,
	BT_PCM_DIN,
	BT_PCM_SYNC,
	BT_PCM_CLK,
	BT_HOST_WAKE,
};

static unsigned bt_config_power_on[] = {
#if 0 
	GPIO_CFG(42, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),	/* WAKE */
	GPIO_CFG(43, 2, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),	/* RFR */
	GPIO_CFG(44, 2, GPIO_INPUT,  GPIO_NO_PULL, GPIO_2MA),	/* CTS */
	GPIO_CFG(45, 2, GPIO_INPUT,  GPIO_NO_PULL, GPIO_2MA),	/* Rx */
	GPIO_CFG(46, 3, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),	/* Tx */
	GPIO_CFG(68, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),	/* PCM_DOUT */
	GPIO_CFG(69, 1, GPIO_INPUT,  GPIO_NO_PULL, GPIO_2MA),	/* PCM_DIN */
	GPIO_CFG(70, 2, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),	/* PCM_SYNC */
	GPIO_CFG(71, 2, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),	/* PCM_CLK */
	GPIO_CFG(83, 0, GPIO_INPUT,  GPIO_NO_PULL, GPIO_2MA),	/* HOST_WAKE */
#else  
       GPIO_CFG(42, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),	/* WAKE */
	GPIO_CFG(43, 2, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),	/* RFR */
	GPIO_CFG(44, 2, GPIO_INPUT,  GPIO_NO_PULL, GPIO_2MA),	        /* CTS */
	GPIO_CFG(45, 2, GPIO_INPUT,  GPIO_NO_PULL, GPIO_2MA),	        /* Rx */
	GPIO_CFG(46, 3, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),	/* Tx */
	GPIO_CFG(68, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),	/* PCM_DOUT */
	GPIO_CFG(69, 1, GPIO_INPUT,  GPIO_NO_PULL, GPIO_2MA),	       /* PCM_DIN */
	GPIO_CFG(70, 2, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),	/* PCM_SYNC */
	GPIO_CFG(71, 2, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),	/* PCM_CLK */
      GPIO_CFG(92, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),
	GPIO_CFG(83, 0, GPIO_INPUT,  GPIO_NO_PULL, GPIO_2MA),	/* HOST_WAKE*/	
#endif  
};
static unsigned bt_config_power_off[] = {
#if 0	 
	GPIO_CFG(42, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA),	/* WAKE */
	GPIO_CFG(43, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA),	/* RFR */
	GPIO_CFG(44, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA),	/* CTS */
	GPIO_CFG(45, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA),	/* Rx */
	GPIO_CFG(46, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA),	/* Tx */
	GPIO_CFG(68, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA),	/* PCM_DOUT */
	GPIO_CFG(69, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA),	/* PCM_DIN */
	GPIO_CFG(70, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA),	/* PCM_SYNC */
	GPIO_CFG(71, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA),	/* PCM_CLK */
	GPIO_CFG(83, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA),	/* HOST_WAKE */
#else 
	GPIO_CFG(42, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA),	/* wake*/
       GPIO_CFG(43, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA),	/* RFR */
	GPIO_CFG(44, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA),	/* CTS */
	GPIO_CFG(45, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA),	/* Rx */
	GPIO_CFG(46, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA),	/* Tx */
	GPIO_CFG(68, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA),	/* PCM_DOUT */
	GPIO_CFG(69, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA),	/* PCM_DIN */
	GPIO_CFG(70, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA),	/* PCM_SYNC */
	GPIO_CFG(71, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA),	/* PCM_CLK */
       GPIO_CFG(92, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA),	/* reset */
	GPIO_CFG(83, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA),	/* HOST_WAKE */
#endif
};

static int bluetooth_power(int on)
{
#if 0
	struct vreg *vreg_bt;
#endif
	int pin, rc;

	printk(KERN_DEBUG "%s\n", __func__);

	/* do not have vreg bt defined, gp6 is the same */
	/* vreg_get parameter 1 (struct device *) is ignored */
#if 0
	vreg_bt = vreg_get(0, "gp6");

	if (!vreg_bt) {
		printk(KERN_ERR "%s: vreg get failed\n", __func__);
		return -EIO;
	}
#endif
	if (on) {
		for (pin = 0; pin < ARRAY_SIZE(bt_config_power_on); pin++) {
			rc = gpio_tlmm_config(bt_config_power_on[pin],
					      GPIO_ENABLE);
			if (rc) {
				printk(KERN_ERR
				       "%s: gpio_tlmm_config(%#x)=%d\n",
				       __func__, bt_config_power_on[pin], rc);
				return -EIO;
			}
		}
#if 0
		/* units of mV, steps of 50 mV */
		rc = vreg_set_level(vreg_bt, 2600);
		if (rc) {
			printk(KERN_ERR "%s: vreg set level failed (%d)\n",
			       __func__, rc);
			return -EIO;
		}
		rc = vreg_enable(vreg_bt);
		if (rc) {
			printk(KERN_ERR "%s: vreg enable failed (%d)\n",
			       __func__, rc);
			return -EIO;
		}
#else
            rc = gpio_direction_output(92, 1);  /*92 -->1*/
            if (rc) {
			printk(KERN_ERR "%s: generation BTS4020 main clock is failed (%d)\n",
			       __func__, rc);
			return -EIO;
		}
#endif
	} else {
#if 0		
		rc = vreg_disable(vreg_bt);
		if (rc) {
			printk(KERN_ERR "%s: vreg disable failed (%d)\n",
			       __func__, rc);
			return -EIO;
		}
#else
           rc = gpio_direction_output(92, 0);  
#endif
		for (pin = 0; pin < ARRAY_SIZE(bt_config_power_off); pin++) {
			rc = gpio_tlmm_config(bt_config_power_off[pin],
					      GPIO_ENABLE);
			if (rc) {
				printk(KERN_ERR
				       "%s: gpio_tlmm_config(%#x)=%d\n",
				       __func__, bt_config_power_off[pin], rc);
				return -EIO;
			}
		}
	}
	return 0;
}

static void __init bt_power_init(void)
{
	msm_bt_power_device.dev.platform_data = &bluetooth_power;
}
#else
#define bt_power_init(x) do {} while (0)
#endif

static struct resource bluesleep_resources[] = {
	{
		.name	= "gpio_host_wake",
		.start	= 83,
		.end	= 83,
		.flags	= IORESOURCE_IO,
	},
	{
		.name	= "gpio_ext_wake",
		.start	= 42,
		.end	= 42,
		.flags	= IORESOURCE_IO,
	},
	{
		.name	= "host_wake",
		.start	= MSM_GPIO_TO_INT(83),
		.end	= MSM_GPIO_TO_INT(83),
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device msm_bluesleep_device = {
	.name = "bluesleep",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(bluesleep_resources),
	.resource	= bluesleep_resources,
};

static u32 msm_calculate_batt_capacity(u32 current_voltage);

static struct msm_psy_batt_pdata msm_psy_batt_data = {
	.voltage_min_design 	= 3200,
	.voltage_max_design	= 4200,
	.avail_chg_sources   	= AC_CHG | USB_CHG ,
	.batt_technology        = POWER_SUPPLY_TECHNOLOGY_LION,
	.calculate_capacity	= &msm_calculate_batt_capacity,
};

static u32 msm_calculate_batt_capacity(u32 current_voltage)
{
	u32 low_voltage   = msm_psy_batt_data.voltage_min_design;
	u32 high_voltage  = msm_psy_batt_data.voltage_max_design;

	return (current_voltage - low_voltage) * 100
		/ (high_voltage - low_voltage);
}

#ifndef CONFIG_HUAWEI_BATTERY
static struct platform_device msm_batt_device = {
	.name 		    = "msm-battery",
	.id		    = -1,
	.dev.platform_data  = &msm_psy_batt_data,
};
#else
static struct platform_device huawei_battery_device = {
	.name = "huawei_battery",
	.id		= -1,
};
#endif
#ifdef CONFIG_LEDS_MSM_PMIC
static struct platform_device msm_device_pmic_leds = {
	.name = "pmic-leds",
	.id		= -1,
};
#endif //CONFIG_LEDS_MSM_PMIC

#ifdef CONFIG_HUAWEI_WIFI_SDCC 
static struct platform_device msm_wlan_ar6000 = {
	.name		= "wlan_ar6000",
	.id		= 1,
	.num_resources	= 0,
	.resource	= NULL,
};
#endif
static struct platform_device *devices[] __initdata = {
#ifdef CONFIG_HUAWEI_WIFI_SDCC 
	&msm_wlan_ar6000,
#endif

#if !defined(CONFIG_MSM_SERIAL_DEBUGGER)
	&msm_device_uart3,
#endif
	&msm_device_uart_dm1,
	&msm_device_smd,
	&msm_device_dmov,
	&msm_device_nand,
	&msm_device_i2c,
	&smc91x_device,
	&msm_device_tssc,
#ifdef CONFIG_HUAWEI_EBI_DEVICE
	&android_pmem_kernel_ebi1_device,
#endif	
	&android_pmem_device,
	&android_pmem_adsp_device,
#if 0	
	&android_pmem_camera_device,	
#endif
#if 0
#ifdef CONFIG_MSM_STACKED_MEMORY
	&android_pmem_gpu0_device,
#endif
	&android_pmem_gpu1_device,
#endif
	&msm_device_hsusb_otg,
	&msm_device_hsusb_host,
#if defined(CONFIG_USB_FUNCTION) || defined(CONFIG_USB_ANDROID)
	&msm_device_hsusb_peripheral,
#endif
#ifdef CONFIG_USB_FUNCTION
	&mass_storage_device,
#endif
#ifdef CONFIG_USB_ANDROID
	&android_usb_device,
#endif

#ifdef CONFIG_BT
	&msm_bt_power_device,
#endif

#ifndef CONFIG_HUAWEI_CAMERA
#ifdef CONFIG_MT9T013
	&msm_camera_sensor_mt9t013,
#endif
#ifdef CONFIG_MT9D112
	&msm_camera_sensor_mt9d112,
#endif
#ifdef CONFIG_S5K3E2FX
	&msm_camera_sensor_s5k3e2fx,
#endif
#ifdef CONFIG_MT9P012
	&msm_camera_sensor_mt9p012,
#endif
#else /*CONFIG_HUAWEI_CAMERA*/
#ifdef CONFIG_MT9T013
	&msm_camera_sensor_mt9t013_byd,
#endif
#ifdef CONFIG_MT9T013
	&msm_camera_sensor_mt9t013_liteon,
#endif
#ifdef CONFIG_HUAWEI_CAMERA_SENSOR_OV3647
	&msm_camera_sensor_ov3647,
#endif
#ifdef CONFIG_HUAWEI_CAMERA_SENSOR_OV7690
	&msm_camera_sensor_ov7690,
#endif
#endif /* #ifndef CONFIG_HUAWEI_CAMERA*/
	&halibut_snd,
	&msm_bluesleep_device,
	&msm_fb_device,
#ifdef CONFIG_FB_MSM_MDDI_TC358723XBG_VGA_QCIF
	&mddi_tc358723xbg_device,
#endif //CONFIG_FB_MSM_MDDI_TC358723XBG_VGA_QCIF
	&mddi_toshiba_device,
	&mddi_sharp_device,
#ifndef CONFIG_HUAWEI_BATTERY
	&msm_batt_device,
#else
	&huawei_battery_device,
#endif

	&hs_device,
#ifdef CONFIG_LEDS_MSM_PMIC
	&msm_device_pmic_leds,
#endif	
};

extern struct sys_timer msm_timer;

static void __init halibut_init_irq(void)
{
	msm_init_irq();
}

static struct msm_acpu_clock_platform_data halibut_clock_data = {
	.acpu_switch_time_us = 50,
	.max_speed_delta_khz = 256000,
	.vdd_switch_time_us = 62,
	.power_collapse_khz = 19200000,
	.wait_for_irq_khz = 128000000,
	.max_axi_khz = 128000,
};

void msm_serial_debug_init(unsigned int base, int irq,
				struct device *clk_device, int signal_irq);
static void sdcc_gpio_init(void)
{
#ifdef CONFIG_MMC_MSM_CARD_HW_DETECTION
	int rc = 0;
	if (gpio_request(49, "sdc1_status_irq"))
		pr_err("failed to request gpio sdc1_status_irq\n");
	rc = gpio_tlmm_config(GPIO_CFG(49, 0, GPIO_INPUT, GPIO_PULL_UP,
				GPIO_2MA), GPIO_ENABLE);
	if (rc)
		printk(KERN_ERR "%s: Failed to configure GPIO %d\n",
				__func__, rc);
#endif
#ifdef CONFIG_HUAWEI_WIFI_SDCC 
	int rc = 0;	
	if (gpio_request(28, "wifi_wow_irq"))		
	    printk("failed to request gpio wifi_wow_irq\n");	

	rc = gpio_tlmm_config(GPIO_CFG(28, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), GPIO_ENABLE);	
	if (rc)		
	    printk(KERN_ERR "%s: Failed to configure GPIO %d\n",__func__, rc);
#endif
	/* SDC1 GPIOs */
#ifdef CONFIG_MMC_MSM_SDC1_SUPPORT
	if (gpio_request(51, "sdc1_data_3"))
		pr_err("failed to request gpio sdc1_data_3\n");
	if (gpio_request(52, "sdc1_data_2"))
		pr_err("failed to request gpio sdc1_data_2\n");
	if (gpio_request(53, "sdc1_data_1"))
		pr_err("failed to request gpio sdc1_data_1\n");
	if (gpio_request(54, "sdc1_data_0"))
		pr_err("failed to request gpio sdc1_data_0\n");
	if (gpio_request(55, "sdc1_cmd"))
		pr_err("failed to request gpio sdc1_cmd\n");
	if (gpio_request(56, "sdc1_clk"))
		pr_err("failed to request gpio sdc1_clk\n");
#endif

	if (machine_is_msm7201a_ffa())
		return;

	/* SDC2 GPIOs */
#ifdef CONFIG_MMC_MSM_SDC2_SUPPORT
	if (gpio_request(62, "sdc2_clk"))
		pr_err("failed to request gpio sdc2_clk\n");
	if (gpio_request(63, "sdc2_cmd"))
		pr_err("failed to request gpio sdc2_cmd\n");
	if (gpio_request(64, "sdc2_data_3"))
		pr_err("failed to request gpio sdc2_data_3\n");
	if (gpio_request(65, "sdc2_data_2"))
		pr_err("failed to request gpio sdc2_data_2\n");
	if (gpio_request(66, "sdc2_data_1"))
		pr_err("failed to request gpio sdc2_data_1\n");
	if (gpio_request(67, "sdc2_data_0"))
		pr_err("failed to request gpio sdc2_data_0\n");
#endif

	/* SDC4 GPIOs */
#ifdef CONFIG_MMC_MSM_SDC4_SUPPORT
	if (gpio_request(19, "sdc4_data_3"))
		pr_err("failed to request gpio sdc4_data_3\n");
	if (gpio_request(20, "sdc4_data_2"))
		pr_err("failed to request gpio sdc4_data_2\n");
	if (gpio_request(21, "sdc4_data_1"))
		pr_err("failed to request gpio sdc4_data_1\n");
	if (gpio_request(107, "sdc4_cmd"))
		pr_err("failed to request gpio sdc4_cmd\n");
	if (gpio_request(108, "sdc4_data_0"))
		pr_err("failed to request gpio sdc4_data_0\n");
	if (gpio_request(109, "sdc4_clk"))
		pr_err("failed to request gpio sdc4_clk\n");
#endif
}

static unsigned sdcc_cfg_data[][6] = {
	/* SDC1 configs */
	{
	GPIO_CFG(51, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA),
	GPIO_CFG(52, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA),
	GPIO_CFG(53, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA),
	GPIO_CFG(54, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA),
	GPIO_CFG(55, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA),
	GPIO_CFG(56, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA),
	},
	/* SDC2 configs */
	{
	GPIO_CFG(62, 2, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA),
	GPIO_CFG(63, 2, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA),
	GPIO_CFG(64, 2, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA),
	GPIO_CFG(65, 2, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA),
	GPIO_CFG(66, 2, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA),
	GPIO_CFG(67, 2, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA),
	},
	{
	/* SDC3 configs */
	},
	/* SDC4 configs */
	{
#ifdef CONFIG_MMC_MSM_SDC4_SUPPORT	
	GPIO_CFG(19, 3, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA),
	GPIO_CFG(20, 3, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA),
	GPIO_CFG(21, 3, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA),
	GPIO_CFG(107, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA),
	GPIO_CFG(108, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA),
	GPIO_CFG(109, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA),
#endif	
	}
};

static unsigned long vreg_sts, gpio_sts;
static struct mpp *mpp_mmc;
static struct vreg *vreg_mmc;

static void msm_sdcc_setup_gpio(int dev_id, unsigned int enable)
{
	int i, rc;

	if (!(test_bit(dev_id, &gpio_sts)^enable))
		return;

	if (enable)
		set_bit(dev_id, &gpio_sts);
	else
		clear_bit(dev_id, &gpio_sts);

	for (i = 0; i < ARRAY_SIZE(sdcc_cfg_data[dev_id - 1]); i++) {
		rc = gpio_tlmm_config(sdcc_cfg_data[dev_id - 1][i],
			enable ? GPIO_ENABLE : GPIO_DISABLE);
		if (rc) {
			printk(KERN_ERR "%s: gpio_tlmm_config(%#x)=%d\n",
				__func__, sdcc_cfg_data[dev_id - 1][i], rc);
		}
	}
}

static uint32_t msm_sdcc_setup_power(struct device *dv, unsigned int vdd)
{
	int rc = 0;
	struct platform_device *pdev;

	pdev = container_of(dv, struct platform_device, dev);
	msm_sdcc_setup_gpio(pdev->id, !!vdd);

	if (vdd == 0) {
		if (!vreg_sts)
			return 0;

		clear_bit(pdev->id, &vreg_sts);

		if (!vreg_sts) {
			if (machine_is_msm7201a_ffa())
				rc = mpp_config_digital_out(mpp_mmc,
				     MPP_CFG(MPP_DLOGIC_LVL_MSMP,
				     MPP_DLOGIC_OUT_CTRL_LOW));
			else
#ifdef CONFIG_HUAWEI_APPS
				rc = 0;
#else
				rc = vreg_disable(vreg_mmc);
#endif
			if (rc)
				printk(KERN_ERR "%s: return val: %d \n",
					__func__, rc);
		}
		return 0;
	}

	if (!vreg_sts) {
		if (machine_is_msm7201a_ffa())
			rc = mpp_config_digital_out(mpp_mmc,
			     MPP_CFG(MPP_DLOGIC_LVL_MSMP,
			     MPP_DLOGIC_OUT_CTRL_HIGH));
		else {
			rc = vreg_set_level(vreg_mmc, 2850);
			if (!rc)
				rc = vreg_enable(vreg_mmc);
		}
		if (rc)
			printk(KERN_ERR "%s: return val: %d \n",
					__func__, rc);
	}
	set_bit(pdev->id, &vreg_sts);
	return 0;
}

#ifdef CONFIG_MMC_MSM_CARD_HW_DETECTION
static unsigned int halibut_sdcc_slot_status(struct device *dev)
{
	return (unsinged int) gpio_get_value(49);
}
#endif

static struct mmc_platform_data halibut_sdcc_data = {
	.ocr_mask	= MMC_VDD_28_29,
	.translate_vdd	= msm_sdcc_setup_power,
#ifdef CONFIG_MMC_MSM_CARD_HW_DETECTION
	.status         = halibut_sdcc_slot_status,
	.status_irq	= MSM_GPIO_TO_INT(49),
	.irq_flags      = IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
#endif
    .mmc_bus_width = MMC_CAP_4_BIT_DATA,
};

#ifdef CONFIG_HUAWEI_WIFI_SDCC 
static uint32_t msm_sdcc_setup_power_wifi(struct device *dv, unsigned int vdd)
{
	struct platform_device *pdev;

	pdev = container_of(dv, struct platform_device, dev);
	msm_sdcc_setup_gpio(pdev->id, !!vdd);

	return 0;
}

static struct mmc_platform_data halibut_sdcc_data_wifi = {
	.ocr_mask	= MMC_VDD_28_29,
	.translate_vdd	= msm_sdcc_setup_power_wifi
};
#endif 
static void __init halibut_init_mmc(void)
{
	if (machine_is_msm7201a_ffa()) {
		mpp_mmc = mpp_get(NULL, "mpp3");
		if (!mpp_mmc) {
			printk(KERN_ERR "%s: mpp get failed (%ld)\n",
			       __func__, PTR_ERR(vreg_mmc));
			return;
		}
	} else {
#ifndef CONFIG_HUAWEI_APPS
		vreg_mmc = vreg_get(NULL, "mmc");
#else
		vreg_mmc = vreg_get(NULL, "wlan");
#endif
		if (IS_ERR(vreg_mmc)) {
			printk(KERN_ERR "%s: vreg get failed (%ld)\n",
			       __func__, PTR_ERR(vreg_mmc));
			return;
		}
	}

	sdcc_gpio_init();
#ifdef CONFIG_MMC_MSM_SDC1_SUPPORT
	msm_add_sdcc(1, &halibut_sdcc_data, 0, 0);
#endif
#ifdef CONFIG_MMC_MSM_CARD_HW_DETECTION
	msm_add_sdcc(1, &halibut_sdcc_data, MSM_GPIO_TO_INT(49),
			IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING);
#endif

	if (machine_is_msm7201a_surf()) {
#ifdef CONFIG_MMC_MSM_SDC2_SUPPORT
#ifdef CONFIG_HUAWEI_WIFI_SDCC 
	msm_add_sdcc(2, &halibut_sdcc_data_wifi,0,0);
#else
	msm_add_sdcc(2, &halibut_sdcc_data,0,0);
#endif 
#endif
#ifdef CONFIG_MMC_MSM_SDC4_SUPPORT
		msm_add_sdcc(4, &halibut_sdcc_data, 0, 0);
#endif
	}
}

static struct msm_panel_common_pdata mdp_pdata = {
	.gpio = 97,
};

static void __init msm_fb_add_devices(void)
{
	msm_fb_register_device("mdp", &mdp_pdata);
	//msm_fb_register_device("ebi2", 0); 
	msm_fb_register_device("pmdh", &mddi_pdata);
	msm_fb_register_device("emdh", 0);
	msm_fb_register_device("tvenc", &tvenc_pdata);
}

static struct msm_pm_platform_data msm_pm_data[MSM_PM_SLEEP_MODE_NR] = {
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE].latency = 16000,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN].latency = 12000,
	[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].latency = 2000,
};

static void
msm_i2c_gpio_config(int iface, int config_type)
{
	int gpio_scl;
	int gpio_sda;
	if (iface) {
		gpio_scl = 95;
		gpio_sda = 96;
	} else {
		gpio_scl = 60;
		gpio_sda = 61;
	}
	if (config_type) {
		gpio_tlmm_config(GPIO_CFG(gpio_scl, 1, GPIO_INPUT,
					GPIO_NO_PULL, GPIO_16MA), GPIO_ENABLE);
		gpio_tlmm_config(GPIO_CFG(gpio_sda, 1, GPIO_INPUT,
					GPIO_NO_PULL, GPIO_16MA), GPIO_ENABLE);
	} else {
		gpio_tlmm_config(GPIO_CFG(gpio_scl, 0, GPIO_OUTPUT,
					GPIO_NO_PULL, GPIO_16MA), GPIO_ENABLE);
		gpio_tlmm_config(GPIO_CFG(gpio_sda, 0, GPIO_OUTPUT,
					GPIO_NO_PULL, GPIO_16MA), GPIO_ENABLE);
	}
}

static struct msm_i2c_platform_data msm_i2c_pdata = {
	.clk_freq = 100000,
	.rmutex = NULL,
	.pri_clk = 60,
	.pri_dat = 61,
	.aux_clk = 95,
	.aux_dat = 96,
	.msm_i2c_config_gpio = msm_i2c_gpio_config,
};

static void __init msm_device_i2c_init(void)
{
	if (gpio_request(60, "i2c_pri_clk"))
		pr_err("failed to request gpio i2c_pri_clk\n");
	if (gpio_request(61, "i2c_pri_dat"))
		pr_err("failed to request gpio i2c_pri_dat\n");
	if (gpio_request(95, "i2c_sec_clk"))
		pr_err("failed to request gpio i2c_sec_clk\n");
	if (gpio_request(96, "i2c_sec_dat"))
		pr_err("failed to request gpio i2c_sec_dat\n");

	msm_i2c_pdata.pm_lat =
		msm_pm_data[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN]
		.latency;
	msm_device_i2c.dev.platform_data = &msm_i2c_pdata;
}

#ifdef CONFIG_USB_AUTO_PID_ADAPTER
/* provide a method to map pid_index to usb_pid, 
 * pid_index is kept in NV(4526). 
 * At power up, pid_index is read in modem and transfer to app in share memory.
 * pid_index can be modified through write file fixusb(msm_hsusb_store_fixusb).
*/
u16 pid_index_to_pid(u32 pid_index)
{
    u16 usb_pid = 0xFFFF;
    
    switch(pid_index)
    {
        case CDROM_INDEX:
            usb_pid = curr_usb_pid_ptr->cdrom_pid;
            break;
        case NORM_INDEX:
            usb_pid = curr_usb_pid_ptr->norm_pid;
            break;
        case AUTH_INDEX:
            usb_pid = curr_usb_pid_ptr->auth_pid;
            break;
            
        /* set the USB pid to multiport when the index is 0
           This is happened when the NV is not set or set 
           to zero 
        */
        case ORI_INDEX:
        default:
            usb_pid = curr_usb_pid_ptr->norm_pid;
            break;
    }

    USB_PR("%s, pid_index=%d, usb_pid=0x%x\n", __func__, pid_index, usb_pid);
    
    return usb_pid;
}

/*  
 * Get usb parameter from share memory and set usb serial number accordingly.
 */

static void proc_usb_para(void)
{
    smem_huawei_vender *usb_para_ptr;
    char *vender_name="t-mobile";

    USB_PR("< %s\n", __func__);

    /* initialize */
    usb_para_info.usb_pid_index = 0;
    usb_para_info.usb_pid = PID_NORMAL;
    
    /* now the smem_id_vendor0 smem id is a new struct */
    usb_para_ptr = (smem_huawei_vender*)smem_alloc(SMEM_ID_VENDOR0, sizeof(smem_huawei_vender));
    if (!usb_para_ptr)
    {
    	USB_PR("%s: Can't find usb parameter\n", __func__);
        return;
    }

    USB_PR("vendor:%s,country:%s\n", usb_para_ptr->vender_para.vender_name, usb_para_ptr->vender_para.country_name);

    /* decide usb pid array according to the vender name */
    if(!memcmp(usb_para_ptr->vender_para.vender_name, vender_name, strlen(vender_name)))
    {
        curr_usb_pid_ptr = &usb_pid_array[1];
        USB_PR("USB setting is TMO\n");
    }
    else
    {
        curr_usb_pid_ptr = &usb_pid_array[0];
        USB_PR("USB setting is NORMAL\n");
    }

    USB_PR("smem usb_serial=%s, usb_pid_index=%d\n", usb_para_ptr->usb_para.usb_serial, usb_para_ptr->usb_para.usb_pid_index);

    usb_para_info.usb_pid_index = usb_para_ptr->usb_para.usb_pid_index;
#ifdef CONFIG_USB_AUTO_INSTALL
    usb_para_info.usb_pid = pid_index_to_pid(usb_para_ptr->usb_para.usb_pid_index);
#else    
    usb_para_info.usb_pid = pid_index_to_pid(NORM_INDEX);
#endif    
    USB_PR("curr_usb_pid_ptr: 0x%x, 0x%x, 0x%x\n", curr_usb_pid_ptr->cdrom_pid, curr_usb_pid_ptr->norm_pid, curr_usb_pid_ptr->udisk_pid);
    USB_PR("usb_para_info: usb_pid_index=%d, usb_pid = 0x%x>\n", usb_para_info.usb_pid_index, usb_para_info.usb_pid);

}

/* set usb serial number */
void set_usb_sn(char *sn_ptr)
{
    if(sn_ptr == NULL)
    {
        ((struct msm_hsusb_platform_data *)(msm_device_hsusb_peripheral.dev.platform_data))->serial_number = NULL;
        USB_PR("set USB SN to NULL\n");
    }
    else
    {
        memcpy(usb_serial_num, sn_ptr, strlen(sn_ptr));
        ((struct msm_hsusb_platform_data *)(msm_device_hsusb_peripheral.dev.platform_data))->serial_number = (char *)usb_serial_num;
        USB_PR("set USB SN to %s\n", usb_serial_num);

    }
}
#endif

static void __init halibut_init(void)
{
#ifdef CONFIG_TOUCHSCREEN_MELFAS
    int ret;
    struct mpp *mpp_ts_reset;
      
    mpp_ts_reset = mpp_get(NULL, "mpp14");
    if (!mpp_ts_reset)
    {
        printk(KERN_ERR "%s: mpp14 get failed\n", __func__);
    }
    ret = mpp_config_digital_out(mpp_ts_reset,
          MPP_CFG(MPP_DLOGIC_LVL_MSMP,MPP_DLOGIC_OUT_CTRL_LOW));    
    if (ret) 
    {
        printk(KERN_ERR "%s: Failed to configure mpp (%d)\n",__func__, ret);
    }
#endif

#ifdef CONFIG_HUAWEI_CAMERA
    sensor_vreg_disable(sensor_vreg_array,ARRAY_SIZE(sensor_vreg_array));
#endif

	if (socinfo_init() < 0)
		BUG();

	if (machine_is_msm7201a_ffa()) {
		smc91x_resources[0].start = 0x98000300;
		smc91x_resources[0].end = 0x980003ff;
		smc91x_resources[1].start = MSM_GPIO_TO_INT(85);
		smc91x_resources[1].end = MSM_GPIO_TO_INT(85);
	}

	/* All 7x01 2.0 based boards are expected to have RAM chips capable
	 * of 160 MHz. */
	if (cpu_is_msm7x01()
	    && SOCINFO_VERSION_MAJOR(socinfo_get_version()) == 2)
		halibut_clock_data.max_axi_khz = 160000;

	msm_acpu_clock_init(&halibut_clock_data);

#if defined(CONFIG_MSM_SERIAL_DEBUGGER)
	msm_serial_debug_init(MSM_UART3_PHYS, INT_UART3,
			      &msm_device_uart3.dev, 1);
#endif
	msm_hsusb_pdata.max_axi_khz = clk_get_max_axi_khz();
	msm_hsusb_pdata.soc_version = socinfo_get_version();
#ifdef CONFIG_MSM_CAMERA
	config_camera_off_gpios(); /* might not be necessary */
#endif
	i2c_register_board_info(0, i2c_devices, ARRAY_SIZE(i2c_devices));
	msm_device_hsusb_peripheral.dev.platform_data = &msm_hsusb_pdata,
	msm_device_hsusb_host.dev.platform_data = &msm_hsusb_pdata,
#ifdef CONFIG_USB_AUTO_PID_ADAPTER
    proc_usb_para();

    if(&usb_pid_array[1] == curr_usb_pid_ptr)
    {
    	msm_hsusb_tmo_pdata.max_axi_khz = clk_get_max_axi_khz();
    	msm_hsusb_tmo_pdata.soc_version = socinfo_get_version();

        msm_device_hsusb_peripheral.dev.platform_data = &msm_hsusb_tmo_pdata;
        mass_storage_device.dev.platform_data = &usb_mass_storage_tmo_pdata;
    }
    else
    {
    	msm_hsusb_pdata.max_axi_khz = clk_get_max_axi_khz();
	    msm_hsusb_pdata.soc_version = socinfo_get_version();

        msm_device_hsusb_peripheral.dev.platform_data = &msm_hsusb_pdata;
    }
    
    if(NORM_INDEX == usb_para_info.usb_pid_index)
    {
        set_usb_sn(USB_SN_STRING);
    }
#endif  /* #ifdef CONFIG_USB_AUTO_PID_ADAPTER */
	platform_add_devices(devices, ARRAY_SIZE(devices));
	msm_device_i2c_init();

#ifndef CONFIG_HUAWEI_U8220_KEYBOARD
	#ifdef CONFIG_SURF_FFA_GPIO_KEYPAD
	if (machine_is_msm7201a_ffa())
		platform_device_register(&keypad_device_7k_ffa);
	else
		platform_device_register(&keypad_device_surf);
	#endif
#else //CONFIG_HUAWEI_U8220_KEYBOARD
	init_u8220_keypad(1);
#endif //CONFIG_HUAWEI_U8220_KEYBOARD

#ifdef CONFIG_HUAWEI_JOGBALL
		platform_device_register(&jogball_device);
#endif 
	halibut_init_mmc();
#ifdef CONFIG_USB_FUNCTION
	hsusb_gpio_init();
#endif
	msm_fb_add_devices();
	bt_power_init();
#ifdef CONFIG_USB_ANDROID
	msm_hsusb_rpc_connect();
	msm_hsusb_set_vbus_state(1) ;
#endif
#ifdef CONFIG_HUAWEI_MSM_VIBRATOR
	msm_init_pmic_vibrator();
#endif
	msm_pm_set_platform_data(msm_pm_data);
#ifdef CONFIG_MSM_HW3D
    msm_add_gpu_devices(&hw3d_gpu_setting);
#endif
}

#ifdef CONFIG_HUAWEI_EBI_DEVICE
static unsigned pmem_kernel_ebi1_size = PMEM_KERNEL_EBI1_SIZE;
static void __init pmem_kernel_ebi1_size_setup(char **p)
{
	pmem_kernel_ebi1_size = memparse(*p, p);
}
__early_param("pmem_kernel_ebi1_size=", pmem_kernel_ebi1_size_setup);
#endif

static unsigned pmem_mdp_size = MSM_PMEM_MDP_SIZE;
static void __init pmem_mdp_size_setup(char **p)
{
	pmem_mdp_size = memparse(*p, p);
}
__early_param("pmem_mdp_size=", pmem_mdp_size_setup);

static unsigned pmem_camera_size = MSM_PMEM_CAMERA_SIZE;
static void __init pmem_camera_size_setup(char **p)
{
	pmem_camera_size = memparse(*p, p);
}
__early_param("pmem_camera_size=", pmem_camera_size_setup);

static unsigned pmem_adsp_size = MSM_PMEM_ADSP_SIZE;
static void __init pmem_adsp_size_setup(char **p)
{
	pmem_adsp_size = memparse(*p, p);
}
__early_param("pmem_adsp_size=", pmem_adsp_size_setup);

static unsigned pmem_gpu1_size = MSM_PMEM_GPU1_SIZE;
static void __init pmem_gpu1_size_setup(char **p)
{
	pmem_gpu1_size = memparse(*p, p);
}
__early_param("pmem_gpu1_size=", pmem_gpu1_size_setup);

static unsigned fb_size = MSM_FB_SIZE;
static void __init fb_size_setup(char **p)
{
	fb_size = memparse(*p, p);
}
__early_param("fb_size=", fb_size_setup);

static void __init msm_halibut_allocate_memory_regions(void)
{
	void *addr=NULL;
	unsigned long size;
	struct resource *res=NULL;

#ifndef CONFIG_HUAWEI_SMI_64M
	#ifdef CONFIG_HUAWEI_EBI_DEVICE
	size = pmem_kernel_ebi1_size;
	if (size) {
		addr = alloc_bootmem_aligned(size, 0x100000);
		android_pmem_kernel_ebi1_pdata.start = __pa(addr);
		android_pmem_kernel_ebi1_pdata.size = size;
		pr_info("allocating %lu bytes at %p (%lx physical) for kernel"
			" ebi1 pmem arena\n", size, addr, __pa(addr));
	}
	#endif

	size = pmem_mdp_size;
	if (size) {
		addr = alloc_bootmem(size);
		android_pmem_pdata.start = __pa(addr);
		android_pmem_pdata.size = size;
		pr_info("allocating %lu bytes at %p (%lx physical) for mdp "
			"pmem arena\n", size, addr, __pa(addr));
	}

#if 0
	size = pmem_camera_size;
	if (size) {
		addr = alloc_bootmem(size);
		android_pmem_camera_pdata.start = __pa(addr);
		android_pmem_camera_pdata.size = size;
		pr_info("allocating %lu bytes at %p (%lx physical) for camera"
			" pmem arena\n", size, addr, __pa(addr));
	}
#endif
	

	size = pmem_adsp_size;
	if (size) {
		addr = alloc_bootmem(size);
		android_pmem_adsp_pdata.start = __pa(addr);
		android_pmem_adsp_pdata.size = size;
		pr_info("allocating %lu bytes at %p (%lx physical) for adsp "
			"pmem arena\n", size, addr, __pa(addr));
	}

	size = pmem_gpu1_size;
	if (size) {
		addr = alloc_bootmem_aligned(size, 0x100000);
		android_pmem_gpu1_pdata.start = __pa(addr);
		android_pmem_gpu1_pdata.size = size;
		pr_info("allocating %lu bytes at %p (%lx physical) for gpu1 "
			"pmem arena\n", size, addr, __pa(addr));
	}

	size = fb_size ? : MSM_FB_SIZE;
	addr = alloc_bootmem(size);
	msm_fb_resources[0].start = __pa(addr);
	msm_fb_resources[0].end = msm_fb_resources[0].start + size - 1;
	pr_info("allocating %lu bytes at %p (%lx physical) for fb\n",
		size, addr, __pa(addr));
	
#else 	/*CONFIG_HUAWEI_SMI_64M */

	size = pmem_mdp_size;
	if (size) {
		addr = alloc_bootmem(size);
		android_pmem_pdata.start = __pa(addr);
		android_pmem_pdata.size = size;
		pr_info("allocating %lu bytes at %p (%lx physical) for mdp "
			"pmem arena\n", size, addr, __pa(addr));
	}

#if 0
	size = pmem_camera_size;
	if (size) {
		addr = alloc_bootmem(size);
		android_pmem_camera_pdata.start = __pa(addr);
		android_pmem_camera_pdata.size = size;
		pr_info("allocating %lu bytes at %p (%lx physical) for camera"
			" pmem arena\n", size, addr, __pa(addr));
	}
#endif


	size = pmem_adsp_size;
	if (size) {
		//addr = alloc_bootmem(size);
		android_pmem_adsp_pdata.start =  MSM_PMEM_ADSP_BASE;
		android_pmem_adsp_pdata.size = size;
		pr_info("allocating %lu bytes at %p (%lx physical) for adsp "
			"pmem arena\n", size, addr, __pa(addr));
	}

#ifdef CONFIG_MSM_HW3D
size = pmem_gpu1_size;
res=platform_get_resource_byname(&hw3d_device, IORESOURCE_MEM,"ebi");
if (size) {
	addr = alloc_bootmem_aligned(size, 0x100000);
	res->start = __pa(addr);
	res->end = res->start+size-1;
	pr_info("allocating %lu bytes at %p (%lx physical) for gpu1 "
		"pmem arena\n", size, addr, __pa(addr));
}
#endif

	size = fb_size ? : MSM_FB_SIZE;
	//addr =alloc_bootmem(size);
	msm_fb_resources[0].start = MSM_FB_BASE;
	msm_fb_resources[0].end = msm_fb_resources[0].start + size - 1;
	pr_info("allocating %lu bytes at %p (%lx physical) for fb\n",
		size, addr, __pa(addr));
	
#endif /*CONFIG_HUAWEI_SMI_64M*/
}

static void __init halibut_map_io(void)
{
	msm_shared_ram_phys = 0x01F00000;

	msm_map_common_io();
	msm_clock_init(msm_clocks_7x01a, msm_num_clocks_7x01a);
	msm_halibut_allocate_memory_regions();
}

MACHINE_START(HALIBUT, "Halibut Board (QCT SURF7200A)")
#ifdef CONFIG_MSM_DEBUG_UART
	.phys_io        = MSM_DEBUG_UART_PHYS,
	.io_pg_offst    = ((MSM_DEBUG_UART_BASE) >> 18) & 0xfffc,
#endif
	.boot_params	= 0x10000100,
	.map_io		= halibut_map_io,
	.init_irq	= halibut_init_irq,
	.init_machine	= halibut_init,
	.timer		= &msm_timer,
MACHINE_END

MACHINE_START(MSM7201A_FFA, "QCT FFA7201A Board")
#ifdef CONFIG_MSM_DEBUG_UART
	.phys_io        = MSM_DEBUG_UART_PHYS,
	.io_pg_offst    = ((MSM_DEBUG_UART_BASE) >> 18) & 0xfffc,
#endif
	.boot_params	= 0x10000100,
	.map_io		= halibut_map_io,
	.init_irq	= halibut_init_irq,
	.init_machine	= halibut_init,
	.timer		= &msm_timer,
MACHINE_END

MACHINE_START(MSM7201A_SURF, "QCT SURF7201A Board")
#ifdef CONFIG_MSM_DEBUG_UART
	.phys_io        = MSM_DEBUG_UART_PHYS,
	.io_pg_offst    = ((MSM_DEBUG_UART_BASE) >> 18) & 0xfffc,
#endif
	.boot_params	= 0x10000100,
	.map_io		= halibut_map_io,
	.init_irq	= halibut_init_irq,
	.init_machine	= halibut_init,
	.timer		= &msm_timer,
MACHINE_END
