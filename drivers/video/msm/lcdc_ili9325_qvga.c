/* Copyright (c) 2009, Code HUAWEI. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Code Aurora Forum nor
 *       the names of its contributors may be used to endorse or promote
 *       products derived from this software without specific prior written
 *       permission.
 *
 * Alternatively, provided that this notice is retained in full, this software
 * may be relicensed by the recipient under the terms of the GNU General Public
 * License version 2 ("GPL") and only version 2, in which case the provisions of
 * the GPL apply INSTEAD OF those given above.  If the recipient relicenses the
 * software under the GPL, then the identification text in the MODULE_LICENSE
 * macro must be changed to reflect "GPLv2" instead of "Dual BSD/GPL".  Once a
 * recipient changes the license terms to the GPL, subsequent recipients shall
 * not relicense under alternate licensing terms, including the BSD or dual
 * BSD/GPL terms.  In addition, the following license statement immediately
 * below and between the words START and END shall also then apply when this
 * software is relicensed under the GPL:
 *
 * START
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License version 2 and only version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * END
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <linux/delay.h>
#include <mach/gpio.h>
#include "msm_fb.h"
#include "lcdc_huawei_config.h"

struct lcd_ili9325_state_type{
	boolean disp_initialized;
	boolean display_on;
	boolean disp_powered_up;
};

static int lcd_reset_gpio;
static struct lcd_ili9325_state_type lcd_ili9325_state = { 0 };
static struct msm_panel_common_pdata *lcdc_ili9325_pdata;
static lcd_panel_type lcd_panel_qvga = LCD_NONE;
static uint16 ili9325_innolux_init[][3] = 
{
        {0x00E5, 0x8000, 0},
        {0x00E3, 0x3008, 0},
        {0x00E7, 0x0012, 0},
        {0x00EF, 0x1231, 0},
        {0x0001, 0x0100, 0},
        {0x0002, 0x0700, 0},
        {0x0003, 0x1030, 0},
        {0x0004, 0x0000, 0},
        
        {0x0008, 0x0505, 0},
        {0x0009, 0x0000, 0},
        {0x000A, 0x0008, 0},
        
        {0x000C, 0x0110, 0},
        {0x000D, 0x0000, 0},
        //{0x000F, 0x0002, 0},
        {0x000F, 0x001A, 0},
        /*power on sequence*/
        {0x0010, 0x0000, 0},
        {0x0011, 0x0007, 0},
        {0x0012, 0x0000, 0},
        {0x0013, 0x0000, 200},
        {0x0010, 0x1190, 0},
        {0x0011, 0x0227, 0},
        {0x00ec, 0x266F, 50},
        {0x0012, 0x009a, 50},
        {0x0013, 0x1800, 0},
        {0x0029, 0x0021, 0},
        {0x0020, 0x0000, 0},
        {0x0021, 0x0000, 0},
        {0x002B, 0x000C, 0},
        /*adjust the gamma curve*/
        {0x0030, 0x0401, 0},
        {0x0031, 0x0307, 0},
        {0x0032, 0x0302, 0},
        {0x0035, 0x0002, 0},
        {0x0036, 0x1407, 0},
        {0x0037, 0x0504, 0},
        {0x0038, 0x0004, 0},
        {0x0039, 0x0603, 0},
        {0x003C, 0x0200, 0},
        {0x003D, 0x060E, 0},
        /*set GRAM area*/
        {0x0050, 0x0000, 0},
        {0x0051, 0x00EF, 0},
        {0x0052, 0x0000, 0},
        {0x0053, 0x013F, 0},
        {0x0060, 0xA700, 0},
        {0x0061, 0x0001, 0},
        {0x006A, 0x0000, 0},
        /*partial display control*/
        {0x0080, 0x0014, 0},
        {0x0081, 0x0014, 0},
        {0x0082, 0x007C, 0},
        {0x0083, 0x0090, 0},
        {0x0084, 0x0090, 0},
        {0x0085, 0x00f8, 0},
        /*panel control*/
        {0x0090, 0x0010, 0},
        {0x0092, 0x0000, 0},
        {0x0007, 0x0133, 0},
};

static uint16 ili9325_innolux_disp_off[][3] = 
{
        {0x0007, 0x0131, 10},
        {0x0007, 0x0130, 10},
        {0x0007, 0x0000, 0},
        /*power off sequence*/
        {0x0010, 0x0080, 0},
        {0x0011, 0x0000, 0},
        {0x0012, 0x0000, 0},
        {0x0013, 0x0000, 200},
        {0x0010, 0x0082, 0},
};

static uint16 ili9325_innolux_disp_on[][3] = 
{
        /*power on sequence*/
        {0x0010, 0x0080, 0},
        {0x0011, 0x0000, 0},
        {0x0012, 0x0000, 0},
        {0x0013, 0x0000, 200},
        {0x0010, 0x1190, 0},
        {0x0011, 0x0225, 20},
        {0x0011, 0x0227, 50},
        {0x0012, 0x009a, 50},
        {0x0013, 0x1800, 0},
        {0x0029, 0x0021, 50},
        {0x0007, 0x0133, 0},

};


static uint16 ili9325_wintek_init[][3] = 
{
        {0x0000, 0x0001, 0},
        {0x0001, 0x0100, 0},
        {0x0002, 0x0700, 0},
        {0x0003, 0x1030, 0},
        {0x0004, 0x0000, 0},
        {0x0008, 0x0208, 0},
        {0x0009, 0x0000, 0},
        {0x000A, 0x0000, 0},
        {0x000C, 0x0110, 0},
        {0x000D, 0x0000, 0},
        {0x000F, 0x0002, 0},
        {0x0010, 0x1490, 0},
        {0x0011, 0x0227, 0},
        {0x0012, 0x009F, 0},
        {0x0013, 0x1000, 0},
        {0x0029, 0x0019, 0},
        {0x002B, 0x000d, 80},
        {0x0020, 0x0000, 0},
        {0x0021, 0x0000, 0},
        {0x0030, 0x0000, 0},
        {0x0031, 0x0406, 0},
        {0x0032, 0x0104, 0},
        {0x0035, 0x0204, 0},
        {0x0036, 0x0004, 0},
        {0x0037, 0x0306, 0},
        {0x0038, 0x0103, 0},
        {0x0039, 0x0707, 0},
        {0x003C, 0x0402, 0},
        {0x003D, 0x0004, 0},
        {0x0050, 0x0000, 0},
        {0x0051, 0x00ef, 0},
        {0x0052, 0x0000, 0},
        {0x0053, 0x013f, 0},
        {0x0060, 0xa700, 0},
        {0x0061, 0x0001, 0},
        {0x006a, 0x0000, 0},
        {0x0080, 0x0000, 0},
        {0x0081, 0x0000, 0},
        {0x0082, 0x0000, 0},
        {0x0083, 0x0000, 0},
        {0x0084, 0x0000, 0},
        {0x0085, 0x0000, 0},
        {0x0090, 0x0010, 0},
        {0x0092, 0x0600, 0},
        {0x0007, 0x0101, 50},
        {0x0007, 0x0121, 0},
        {0x0007, 0x0123, 50},
        {0x0007, 0x0133, 0},
};

static uint16 ili9325_wintek_disp_off[][3] = 
{
        {0x0007, 0x0131, 10},
        {0x0007, 0x0130, 10},
        {0x0007, 0x0000, 0},
        /*power off sequence*/
        {0x0010, 0x0080, 0},
        {0x0011, 0x0000, 0},
        {0x0012, 0x0000, 0},
        {0x0013, 0x0000, 200},
        {0x0010, 0x0082, 0},
};

static uint16 ili9325_wintek_disp_on[][3] = 
{
        /*power on sequence*/
        {0x0010, 0x0080, 0},
        {0x0011, 0x0000, 0},
        {0x0012, 0x0000, 0},
        {0x0013, 0x0000, 100},
        {0x0010, 0x1490, 0},
        {0x0011, 0x0225, 20},
        {0x0011, 0x0227, 50},
        {0x0012, 0x009F, 50},
        {0x0013, 0x1000, 0},
        {0x0029, 0x0019, 50},
        {0x0007, 0x0133, 0},

};

/* there is no byd LCD init code, innolux LCD init code take the place temporarily */
static uint16 ili9325_byd_init[][3] = 
{
        {0x00e5, 0x78f0, 0},
        {0x0001, 0x0100, 0},
        {0x0002, 0x0700, 0},
        {0x0003, 0x1030, 0},
        {0x0004, 0x0000, 0},
        {0x0008, 0x0808, 0},
        {0x0009, 0x0000, 0},
        {0x000a, 0x0000, 0},
        {0x000c, 0x0000, 0},
        {0x000d, 0x0000, 0},
        {0x000f, 0x0003, 0},
        {0x0010, 0x0000, 0},
        {0x0011, 0x0007, 0},
        {0x0012, 0x0000, 0},
        {0x0013, 0x0000, 0},
        {0x0007, 0x0001, 50},
        {0x0010, 0x1590, 0},
        {0x0011, 0x0227, 50},
        {0x0012, 0x008c, 50},
        {0x0013, 0x1b00, 0},
        {0x0029, 0x0032, 0},
        {0x002B, 0x000d, 50},
        {0x0020, 0x0000, 0},
        {0x0021, 0x0000, 0},
        {0x0030, 0x0000, 0},
        {0x0031, 0x0705, 0},
        {0x0032, 0x0104, 0},
        {0x0035, 0x0302, 0},
        {0x0036, 0x0004, 0},
        {0x0037, 0x0306, 0},
        {0x0038, 0x0202, 0},
        {0x0039, 0x0707, 0},
        {0x003c, 0x0302, 0},
        {0x003d, 0x0004, 0},
        {0x0050, 0x0000, 0},
        {0x0051, 0x00EF, 0},
        {0x0052, 0x0000, 0},
        {0x0053, 0x013f, 0},
        {0x0060, 0Xa700, 0},
        {0x0061, 0x0001, 0},
        {0x006A, 0x0000, 0},
        {0x0080, 0x0000, 0},
        {0x0081, 0x0000, 0},
        {0x0082, 0x0000, 0},
        {0x0083, 0x0000, 0},
        {0x0084, 0x0000, 0},
        {0x0085, 0x0000, 0},
        {0x0090, 0x0010, 0},
        {0x0092, 0x0600, 0},
        {0x0007, 0x0133, 0},

};

static uint16 ili9325_byd_disp_off[][3] = 
{
        {0x0007, 0x0131, 10},
        {0x0007, 0x0130, 10},
        {0x0007, 0x0000, 0},
        /*power off sequence*/
        {0x0010, 0x0080, 0},
        {0x0011, 0x0000, 0},
        {0x0012, 0x0000, 0},
        {0x0013, 0x0000, 200},
        {0x0010, 0x0082, 0},
};

static uint16 ili9325_byd_disp_on[][3] = 
{
        /*power on sequence*/
        {0x0010, 0x0080, 0},
        {0x0011, 0x0000, 0},
        {0x0012, 0x0000, 0},
        {0x0013, 0x0000, 200},
        {0x0010, 0x1590, 0},
        {0x0011, 0x0227, 50},
        {0x0012, 0x008c, 50},
        {0x0013, 0x1b00, 0},
        {0x0029, 0x0032, 50},
        {0x0007, 0x0133, 0},

};
#define NUM_ILI9325_INNOLUX_INIT             (sizeof(ili9325_innolux_init)/(sizeof(uint16) * 3))
#define NUM_ILI9325_INNOLUX_DISP_OFF         (sizeof(ili9325_innolux_disp_off)/(sizeof(uint16) * 3))
#define NUM_ILI9325_INNOLUX_DISP_ON          (sizeof(ili9325_innolux_disp_on)/(sizeof(uint16) * 3)) 

#define NUM_ILI9325_WINTEK_INIT              (sizeof(ili9325_wintek_init)/(sizeof(uint16) * 3))
#define NUM_ILI9325_WINTEK_DISP_OFF          (sizeof(ili9325_wintek_disp_off)/(sizeof(uint16) * 3))
#define NUM_ILI9325_WINTEK_DISP_ON           (sizeof(ili9325_wintek_disp_on)/(sizeof(uint16) * 3)) 

#define NUM_ILI9325_BYD_INIT                 (sizeof(ili9325_byd_init)/(sizeof(uint16) * 3))
#define NUM_ILI9325_BYD_DISP_OFF             (sizeof(ili9325_byd_disp_off)/(sizeof(uint16) * 3))
#define NUM_ILI9325_BYD_DISP_ON              (sizeof(ili9325_byd_disp_on)/(sizeof(uint16) * 3)) 

static void seriout(uint16 reg, uint16 data)
{
    uint8 start_byte_reg = 0x70;
    uint8 start_byte_data = 0x72;
    
    seriout_cmd(reg, start_byte_reg);
    seriout_data(data, start_byte_data);
}

static void lcd_ili9325_disp_powerup(void)
{
	if (!lcd_ili9325_state.disp_powered_up && !lcd_ili9325_state.display_on) 
    {
		/* Reset the hardware first */
		/* Include DAC power up implementation here */
	    lcd_ili9325_state.disp_powered_up = TRUE;
	}
}

static void lcd_ili9325_disp_on(void)
{
	unsigned char i = 0;
    
	if (lcd_ili9325_state.disp_powered_up && !lcd_ili9325_state.display_on)
    {
        switch (lcd_panel_qvga) 
        {
            case LCD_ILI9325_INNOLUX_QVGA:
                for(i = 0; i < NUM_ILI9325_INNOLUX_INIT; i++)
                {
                    seriout(ili9325_innolux_init[i][0], ili9325_innolux_init[i][1]);
                    mdelay(ili9325_innolux_init[i][2]);
                }
                break;
              
            case LCD_ILI9325_BYD_QVGA:
                for(i = 0; i < NUM_ILI9325_BYD_INIT; i++)
                {
                    seriout(ili9325_byd_init[i][0], ili9325_byd_init[i][1]);
                    mdelay(ili9325_byd_init[i][2]);
                }
                break;
              
            case LCD_ILI9325_WINTEK_QVGA:
                for(i = 0; i < NUM_ILI9325_WINTEK_INIT; i++)
                {
                    seriout(ili9325_wintek_init[i][0], ili9325_wintek_init[i][1]);
                    mdelay(ili9325_wintek_init[i][2]);
                }
                break;

            default :
                for(i = 0; i < NUM_ILI9325_INNOLUX_INIT; i++)
                {
                    seriout(ili9325_innolux_init[i][0], ili9325_innolux_init[i][1]);
                    mdelay(ili9325_innolux_init[i][2]);
                }
                break;
              
        }
        seriout_cmd(0x0022, 0x70);

		lcd_ili9325_state.display_on = TRUE;
	}
}

static void lcd_ili9325_disp_exit_sleep(void)
{
	unsigned char i = 0;
    
	if (lcd_ili9325_state.disp_powered_up && !lcd_ili9325_state.display_on)
    {
        switch (lcd_panel_qvga) 
        {
            case LCD_ILI9325_INNOLUX_QVGA:
                for(i = 0; i < NUM_ILI9325_INNOLUX_DISP_ON; i++)
                {
                    seriout(ili9325_innolux_disp_on[i][0], ili9325_innolux_disp_on[i][1]);
                    mdelay(ili9325_innolux_disp_on[i][2]);
                }
                break;
              
            case LCD_ILI9325_BYD_QVGA:
                for(i = 0; i < NUM_ILI9325_BYD_DISP_ON; i++)
                {
                    seriout(ili9325_byd_disp_on[i][0], ili9325_byd_disp_on[i][1]);
                    mdelay(ili9325_byd_disp_on[i][2]);
                }
                break;
              
            case LCD_ILI9325_WINTEK_QVGA:
                for(i = 0; i < NUM_ILI9325_WINTEK_DISP_ON; i++)
                {
                    seriout(ili9325_wintek_disp_on[i][0], ili9325_wintek_disp_on[i][1]);
                    mdelay(ili9325_wintek_disp_on[i][2]);
                }
                break;

            default :
                for(i = 0; i < NUM_ILI9325_INNOLUX_DISP_ON; i++)
                {
                    seriout(ili9325_innolux_disp_on[i][0], ili9325_innolux_disp_on[i][1]);
                    mdelay(ili9325_innolux_disp_on[i][2]);
                }
                break;
              
        }
    
        lcd_ili9325_state.display_on = TRUE;
    }   

}
static int lcdc_ili9325_panel_on(struct platform_device *pdev)
{
	if (!lcd_ili9325_state.disp_initialized) 
    {
		/* Configure reset GPIO that drives DAC */
		lcdc_ili9325_pdata->panel_config_gpio(1);
		lcd_reset_gpio = *(lcdc_ili9325_pdata->gpio_num + 4);
		mdelay(5);
		gpio_set_value(lcd_reset_gpio, 0);
		mdelay(15);
		gpio_set_value(lcd_reset_gpio, 1);
        mdelay(150);
		lcd_spi_init(lcdc_ili9325_pdata);	/* LCD needs SPI */
		lcd_ili9325_disp_powerup();
		lcd_ili9325_disp_on();
		lcd_ili9325_state.disp_initialized = TRUE;
	}
    else if(!lcd_ili9325_state.display_on)
    {
        lcd_ili9325_disp_exit_sleep();
    }
	return 0;
}

static int lcdc_ili9325_panel_off(struct platform_device *pdev)
{
	unsigned char i = 0;

	if (lcd_ili9325_state.disp_powered_up && lcd_ili9325_state.display_on)
    {
        switch (lcd_panel_qvga) 
        {
            case LCD_ILI9325_INNOLUX_QVGA:
                for(i = 0; i < NUM_ILI9325_INNOLUX_DISP_OFF; i++)
                {
                    seriout(ili9325_innolux_disp_off[i][0], ili9325_innolux_disp_off[i][1]);
                    mdelay(ili9325_innolux_disp_off[i][2]);
                }
                break;
              
            case LCD_ILI9325_BYD_QVGA:
                for(i = 0; i < NUM_ILI9325_BYD_DISP_OFF; i++)
                {
                    seriout(ili9325_byd_disp_off[i][0], ili9325_byd_disp_off[i][1]);
                    mdelay(ili9325_byd_disp_off[i][2]);
                }
                break;
              
            case LCD_ILI9325_WINTEK_QVGA:
                for(i = 0; i < NUM_ILI9325_WINTEK_DISP_OFF; i++)
                {
                    seriout(ili9325_wintek_disp_off[i][0], ili9325_wintek_disp_off[i][1]);
                    mdelay(ili9325_wintek_disp_off[i][2]);
                }
                break;

            default :
                for(i = 0; i < NUM_ILI9325_INNOLUX_DISP_OFF; i++)
                {
                    seriout(ili9325_innolux_disp_off[i][0], ili9325_innolux_disp_off[i][1]);
                    mdelay(ili9325_innolux_disp_off[i][2]);
                }
                break;
              
        }
		lcd_ili9325_state.display_on = FALSE;
	}
	return 0;
}

static void lcdc_ili9325_panel_set_backlight(struct msm_fb_data_type *mfd)
{
    int bl_level = mfd->bl_level;
       
    lcd_set_backlight_pwm(bl_level);
    
    return;
}
static int __init lcdc_ili9325_probe(struct platform_device *pdev)
{
	if (pdev->id == 0) {
		lcdc_ili9325_pdata = pdev->dev.platform_data;
		return 0;
	}
	msm_fb_add_device(pdev);
	return 0;
}

static struct platform_driver this_driver = 
{
	.probe  = lcdc_ili9325_probe,
	.driver = {
		.name   = "lcdc_ili9325_qvga",
	},
};

static struct msm_fb_panel_data ili9325_panel_data =
{
	.on = lcdc_ili9325_panel_on,
	.off = lcdc_ili9325_panel_off,
	.set_backlight = lcdc_ili9325_panel_set_backlight,
};

static struct platform_device this_device = 
{
	.name   = "lcdc_ili9325_qvga",
	.id	= 1,
	.dev	=
	{
		.platform_data = &ili9325_panel_data,
	}
};

static int __init lcdc_ili9325_panel_init(void)
{
	int ret;
	struct msm_panel_info *pinfo;


    lcd_panel_qvga = lcd_panel_probe();
    if((LCD_ILI9325_INNOLUX_QVGA != lcd_panel_qvga) && \
       (LCD_ILI9325_BYD_QVGA != lcd_panel_qvga) &&     \
       (LCD_ILI9325_WINTEK_QVGA != lcd_panel_qvga) &&  \
       (msm_fb_detect_client("lcdc_ili9325_qvga"))
      )
    {
        return 0;
    }

	ret = platform_driver_register(&this_driver);
	if (ret)
		return ret;

	pinfo = &ili9325_panel_data.panel_info;
    pinfo->xres = 240;
	pinfo->yres = 320;
	pinfo->type = LCDC_PANEL;
	pinfo->pdest = DISPLAY_1;
	pinfo->wait_cycle = 0;
	pinfo->bpp = 18;
	pinfo->fb_num = 2;
    /*the pixel clk is different for different Resolution LCD*/
	//pinfo->clk_rate = 24500000; /*for VGA pixel clk*/
    pinfo->clk_rate = 6125000;  /*for QVGA pixel clk*/   
    pinfo->lcdc.h_back_porch = 5;
	pinfo->lcdc.h_front_porch = 5;
	pinfo->lcdc.h_pulse_width = 5;    
	pinfo->lcdc.v_back_porch = 3;
	pinfo->lcdc.v_front_porch = 3;
	pinfo->lcdc.v_pulse_width = 3;
	pinfo->lcdc.border_clr = 0;     /* blk */
	pinfo->lcdc.underflow_clr = 0xff;       /* blue */
	pinfo->lcdc.hsync_skew = 0;
    pinfo->bl_max = 255;

	ret = platform_device_register(&this_device);
	if (ret)
		platform_driver_unregister(&this_driver);

	return ret;
}


module_init(lcdc_ili9325_panel_init);

