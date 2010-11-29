/*
 *  linux/fs/proc/app_info.c
 *
 *
 * Changes:
 */

#include <linux/types.h>
#include <linux/errno.h>
#include <linux/time.h>
#include <linux/kernel.h>
#include <linux/kernel_stat.h>
#include <linux/fs.h>
#include <linux/tty.h>
#include <linux/string.h>
#include <linux/mman.h>
#include <linux/quicklist.h>
#include <linux/proc_fs.h>
#include <linux/ioport.h>
#include <linux/mm.h>
#include <linux/mmzone.h>
#include <linux/pagemap.h>
#include <linux/interrupt.h>
#include <linux/swap.h>
#include <linux/slab.h>
#include <linux/genhd.h>
#include <linux/smp.h>
#include <linux/signal.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/seq_file.h>
#include <linux/times.h>
#include <linux/profile.h>
#include <linux/utsname.h>
#include <linux/blkdev.h>
#include <linux/hugetlb.h>
#include <linux/jiffies.h>
#include <linux/sysrq.h>
#include <linux/vmalloc.h>
#include <linux/crash_dump.h>
#include <linux/pid_namespace.h>
#include <linux/bootmem.h>
#include <asm/uaccess.h>
#include <asm/pgtable.h>
#include <asm/io.h>
#include <asm/tlb.h>
#include <asm/div64.h>
#include "internal.h"
#include <asm/setup.h>
#include <asm/mach-types.h>
#include <linux/hardware_self_adapt.h>

#define PROC_MANUFACTURER_STR_LEN 30
#define MAX_VERSION_CHAR 40
#define BOARD_ID_LEN 20
#define LCD_NAME_LEN 20

static char appsboot_version[MAX_VERSION_CHAR + 1];
static char str_flash_nand_id[PROC_MANUFACTURER_STR_LEN] = {0};

#ifndef CONFIG_MACH_MSM7201A_SURF
static u32 camera_id;
static u32 ts_id;
#endif


/* same as in proc_misc.c */
static int
proc_calc_metrics(char *page, char **start, off_t off, int count, int *eof, int len)
{
	if (len <= off + count)
		*eof = 1;
	*start = page + off;
	len -= off;
	if (len > count)
		len = count;
	if (len < 0)
		len = 0;
	return len;
}

#define ATAG_BOOT_READ_FLASH_ID 0x4d534D72
static int __init parse_tag_boot_flash_id(const struct tag *tag)
{
    char *tag_flash_id=(char*)&tag->u;
    memset(str_flash_nand_id, 0, PROC_MANUFACTURER_STR_LEN);
    memcpy(str_flash_nand_id, tag_flash_id, PROC_MANUFACTURER_STR_LEN);
    
    printk("########proc_misc.c: tag_boot_flash_id= %s\n", tag_flash_id);

    return 0;
}
__tagtable(ATAG_BOOT_READ_FLASH_ID, parse_tag_boot_flash_id);

#define ATAG_BOOT_VERSION 0x4d534D71 /* ATAG BOOT VERSION */
static int __init parse_tag_boot_version(const struct tag *tag)
{
    char *tag_boot_ver=(char*)&tag->u;
    memset(appsboot_version, 0, MAX_VERSION_CHAR + 1);
    memcpy(appsboot_version, tag_boot_ver, MAX_VERSION_CHAR);
     
    //printk("nand_partitions.c: appsboot_version= %s\n\n", appsboot_version);

    return 0;
}
__tagtable(ATAG_BOOT_VERSION, parse_tag_boot_version);

#ifndef CONFIG_MACH_MSM7201A_SURF
#define ATAG_CAMERA_ID 0x4d534D74
static int __init parse_tag_camera_id(const struct tag *tag)
{
    char *tag_boot_ver=(char*)&tag->u;
	
    memcpy((void*)&camera_id, tag_boot_ver, sizeof(u32));
     
    return 0;
}
__tagtable(ATAG_CAMERA_ID, parse_tag_camera_id);


#define ATAG_TS_ID 0x4d534D75
static int __init parse_tag_ts_id(const struct tag *tag)
{
    char *tag_boot_ver=(char*)&tag->u;
	
    memcpy((void*)&ts_id, tag_boot_ver, sizeof(u32));
     
    return 0;
}
__tagtable(ATAG_TS_ID, parse_tag_ts_id);
#endif

#ifdef CONFIG_MACH_MSM7201A_SURF
extern const char* mddi_get_lcd_panel(void);
#endif
static int app_version_read_proc(char *page, char **start, off_t off,
				 int count, int *eof, void *data)
{
	int len;
	char *ker_ver = HUAWEI_KERNEL_VERSION;
	char *lcd_name = NULL;
	char s_board_id[BOARD_ID_LEN] = {0};

	switch(machine_arch_type)
	{
		case MACH_TYPE_MSM7X25_C8600:
			strcpy(s_board_id, "MSM7X25_C8600");
			break;
		case MACH_TYPE_MSM7X25_U8110:
			strcpy(s_board_id, "MSM7X25_U8110");
			break;
		case MACH_TYPE_MSM7X25_U8120:
			strcpy(s_board_id, "MSM7X25_U8120");
			break;
		case MACH_TYPE_MSM7X25_U7610:
			strcpy(s_board_id, "MSM7X25_U7610");
			break;
		
		case MACH_TYPE_MSM7201A_SURF:
		    	strcpy(s_board_id, "MSM7201A_SURF");
		    	break;
		default:
			strcpy(s_board_id, "ERROR");
			break;
	}
#ifndef CONFIG_MACH_MSM7201A_SURF
	lcd_name = get_lcd_panel_name();
#else
	lcd_name = (char*)mddi_get_lcd_panel();
#endif	
	len = snprintf(page, PAGE_SIZE, "APPSBOOT:\n"
	"%s\n"
	"KERNEL_VER:\n"
	"%s\n"
	 "FLASH_ID:\n"
	"%s\n"
	"board_id:\n%s\n"
#ifndef CONFIG_MACH_MSM7201A_SURF
	"lcd_id:\n%s\n"
	"cam_id:\n%d\n"
	"ts_id:\n%d\n",
	appsboot_version, ker_ver, str_flash_nand_id, s_board_id, lcd_name, camera_id, ts_id);
#else	
	"lcd_id:\n%s\n",
	appsboot_version, ker_ver, str_flash_nand_id, s_board_id, lcd_name);
#endif
	return proc_calc_metrics(page, start, off, count, eof, len);
}

void __init proc_app_info_init(void)
{
	static struct {
		char *name;
		int (*read_proc)(char*,char**,off_t,int,int*,void*);
	} *p, simple_ones[] = {
		
        {"app_info", app_version_read_proc},
		{NULL,}
	};
	for (p = simple_ones; p->name; p++)
		create_proc_read_entry(p->name, 0, NULL, p->read_proc, NULL);

}


