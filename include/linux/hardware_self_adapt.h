#ifndef HARDWARE_SELF_ADAPT_H
#define HARDWARE_SELF_ADAPT_H

/*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*

                    GPIO/BIO   S E R V I C E S

GENERAL DESCRIPTION

REFERENCES

EXTERNALIZED FUNCTIONS
  None.

INITIALIZATION AND SEQUENCING REQUIREMENTS

Copyright (c) 2009 by HUAWEI, Incorporated.  All Rights Reserved.
*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*/
/*===========================================================================

                      EDIT HISTORY FOR FILE

 This section contains comments describing changes made to this file.
  Notice that changes are listed in reverse chronological order.


when       who      what, where, why
-------------------------------------------------------------------------------
=========================================================================== */


  /*del old product type*/

typedef enum
{
    LCD_S6D74A0_SAMSUNG_HVGA,
    LCD_ILI9325_INNOLUX_QVGA,
    LCD_ILI9325_BYD_QVGA,
    LCD_ILI9325_WINTEK_QVGA,
    LCD_SPFD5408B_KGM_QVGA,

    LCD_MAX_NUM,
    LCD_NONE =0xFF
}lcd_panel_type;


lcd_panel_type lcd_panel_probe(void);
int board_use_tssc_touch(bool * use_touch_key);

char *get_lcd_panel_name(void);

#endif

