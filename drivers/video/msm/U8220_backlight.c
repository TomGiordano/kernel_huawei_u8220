/* drivers\video\msm\U8220_backlight.c
 * backlight driver for TC358723XBG
 * seperate from mddi_tc358723xbg.c
 *
 * 
 */

#include <mach/gpio.h>
#include <linux/delay.h>

#include <linux/earlysuspend.h>
#include <linux/spinlock.h>
#include <linux/semaphore.h>

static atomic_t suspend_flag = ATOMIC_INIT(0); 

#define LCD_GPIO_BL_CL (94)
static int restore_level=1;
#define MAX_BRIGHTENSS_LEVEL 17
void U8220_set_backlight(int level)
{
	int counter=0;
	
	if(level == 0){
		gpio_direction_output(LCD_GPIO_BL_CL,0);
		mdelay(3);
		return;
	}

	if(atomic_read(&suspend_flag)) 
    	{
    	    restore_level = level;
	    return;
    	}

	gpio_direction_output(LCD_GPIO_BL_CL,0);
	mdelay(3);

	// condition test
	if(level<0) level =0;
	if(level>15) level = 15;

	counter = MAX_BRIGHTENSS_LEVEL - level;
	while(counter > 0){
		counter--;
		udelay(1);
		gpio_direction_output(LCD_GPIO_BL_CL,0);
		udelay(1);
		gpio_direction_output(LCD_GPIO_BL_CL,1);
	};				

	return;
}

static void U8220_resume_backlight(int level)
{
	int counter=0;

	gpio_direction_output(LCD_GPIO_BL_CL,0);
	mdelay(3);

	// condition test
	if(level<0) level =0;
	if(level>15) level = 15;

	counter = MAX_BRIGHTENSS_LEVEL - level;
	while(counter > 0){
		counter--;
		udelay(1);
		gpio_direction_output(LCD_GPIO_BL_CL,0);
		udelay(1);
		gpio_direction_output(LCD_GPIO_BL_CL,1);
	};				

        return;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void u8220_backlight_suspend( struct early_suspend *h)
{
        gpio_direction_output(LCD_GPIO_BL_CL,0);
        mdelay(3);
	atomic_set(&suspend_flag,1);
}

static void u8220_backlight_resume( struct early_suspend *h)
{
	U8220_resume_backlight(restore_level);
	atomic_set(&suspend_flag,0);	
}
static struct early_suspend u8220_backlight_early_suspend = {
	.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN,
	.suspend = u8220_backlight_suspend,
	.resume = u8220_backlight_resume,
};
#endif

static int __init U8220_backlight_init(void)
{
	gpio_tlmm_config(GPIO_CFG
		(LCD_GPIO_BL_CL, 0, GPIO_OUTPUT,
		GPIO_PULL_DOWN, GPIO_2MA),
		GPIO_ENABLE);

//	spin_lock_init(&backlight_spinlock);
	
#ifdef CONFIG_HAS_EARLYSUSPEND
	register_early_suspend(&u8220_backlight_early_suspend);
#endif

	return 0;
}

module_init(U8220_backlight_init);
