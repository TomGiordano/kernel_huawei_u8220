/* drivers\video\msm\mddi_tc358721xbg.c
 *  MDDI2RGB driver for TC358721XBG
 *
 * 
 */
 
/* this file was borrowed from mddi_toshiba.c */

#include "msm_fb.h"
#include "mddihost.h"
#include "mddihosti.h"

#include "mddi_tc358721xbg.h"
#include <mach/gpio.h>

/* all panels supported tc358721xbg */
extern s_panel_seq_t panel_lms350df01;
/* add others connected to tc358721xbg here */

/* current panel connected */
static ps_panel_seq_t panel_sel; 


#define TC358721XBG_VGA_PRIM 1
#define TC358721XBG_VGA_SECD 2

static struct msm_panel_common_pdata *mddi_tc358721xbg_pdata;


/* about refresh */
//#define MDDI_TC358721_61HZ_REFRESH

static uint32 mddi_tc358721xbg_curr_vpos;
static boolean mddi_tc358721xbg_monitor_refresh_value = FALSE;
static boolean mddi_tc358721xbg_report_refresh_measurements = FALSE;

/* Modifications to timing to increase refresh rate to > 60Hz.
 *   20MHz dot clock.
 *   646 total rows.
 *   506 total columns.
 *   refresh rate = 61.19Hz
 */
#ifdef MDDI_TC358721_61HZ_REFRESH
static uint32 mddi_tc358721xbg_rows_per_second = 39526;
static uint32 mddi_tc358721xbg_rows_per_refresh = 646;
static uint32 mddi_tc358721xbg_usecs_per_refresh = 16344;
#else
//static uint32 mddi_tc358723xbg_rows_per_second = 37313;
//static uint32 mddi_tc358723xbg_rows_per_refresh = 646;
//static uint32 mddi_tc358723xbg_usecs_per_refresh = 17313;
/*
 *   12.5MHz dot clock
 *    486 total rows
 *    360 total cols
 *     refresh rate = 71.44Hz
 */
static uint32 mddi_tc358723xbg_rows_per_second = 34722; // = 12500000/360
static uint32 mddi_tc358723xbg_rows_per_refresh = 486;
static uint32 mddi_tc358723xbg_usecs_per_refresh = 13997; // = 486/34722
#endif

extern boolean mddi_vsync_detect_enabled;

/* vsync handle */
static msm_fb_vsync_handler_type mddi_tc358721xbg_vsync_handler = NULL;
static void *mddi_tc358721xbg_vsync_handler_arg;
static uint16 mddi_tc358721xbg_vsync_attempts;

/* state machine op */
typedef enum {
	TC358721_STATE_OFF,
	TC358721_STATE_PRIM_SEC_STANDBY,
	TC358721_STATE_PRIM_SEC_READY,
	TC358721_STATE_PRIM_NORMAL_MODE,
	TC358721_STATE_SEC_NORMAL_MODE
} mddi_tc358721xbg_state_t;
static mddi_tc358721xbg_state_t tc358721xbg_state = TC358721_STATE_OFF;
static void mddi_tc358721xbg_state_transition(mddi_tc358721xbg_state_t a,
					  mddi_tc358721xbg_state_t b)
{
	if (tc358721xbg_state != a) {
		MDDI_MSG_ERR("toshiba state trans. (%d->%d) found %d\n", a, b,
			     tc358721xbg_state);
	}
	tc358721xbg_state = b;
}

/* lcd panel register access func */
#define write_client_reg(__X,__Y,__Z) {\
  mddi_queue_register_write(__X,__Y,TRUE,0);\
}
static uint32 read_client_reg(uint32 addr)
{
	uint32 val;
	mddi_queue_register_read(addr, &val, TRUE, 0);
	return val;
}

/* register setting */ 
typedef struct {
	uint32 reg;
	uint32 val;
} s_reg_val_pair_t;

static void s_reg_val_pair_op(uint32 reg, uint32 val)
{
	if(0 == reg)
		mddi_wait(val);
	else
		write_client_reg(reg, val, TRUE);
}

/* write image to eDram 1 for test */
//#define TEST_WR_IMAGE
#ifdef TEST_WR_IMAGE

static unsigned short *FBuff;
static unsigned short BGCOLOR = 0xF800; //red
static mddi_linked_list_type *mlist;
static void s_wr32(void *_dst, unsigned n)
{
    unsigned char *src = (unsigned char*) &n;
    unsigned char *dst = _dst;

    dst[0] = src[0];
    dst[1] = src[1];
    dst[2] = src[2];
    dst[3] = src[3];
};
static int s_mddi_update_done(void)
{
	mddi_host_type host_idx = MDDI_HOST_PRIM;
	return !!(mddi_host_reg_in(STAT) & MDDI_STAT_PRI_LINK_LIST_DONE);
}
static void s_write_image_to_edram1(void)
{
	unsigned n;
   	mddi_host_type host_idx = MDDI_HOST_PRIM;
	FBuff = kzalloc(2 * PRIM_WIDTH* PRIM_HEIGHT, GFP_KERNEL);
	mlist = kzalloc(sizeof(mddi_linked_list_type) * (PRIM_HEIGHT / 8), GFP_KERNEL);

	for(n = 0; n < (PRIM_WIDTH* PRIM_HEIGHT); n++) FBuff[n] = BGCOLOR;
	
	for(n = 0; n < (PRIM_HEIGHT / 8); n++) {
		unsigned y = n * 8;
		unsigned pixels = PRIM_WIDTH * 8;
		mddi_video_stream_packet_type *vs = &(mlist[n].packet_header.video_pkt);

		vs->packet_length = sizeof(mddi_video_stream_packet_type) - 2 + (pixels * 2);
		vs->packet_type = 16;
		vs->bClient_ID = 0;
		vs->video_data_format_descriptor = 0x5565; // FORMAT_16BPP;
		vs->pixel_data_attributes = 3 | (3<<6);
		
		vs->x_left_edge = 0;
		vs->x_right_edge = PRIM_WIDTH - 1;
		vs->y_top_edge = y;
		vs->y_bottom_edge = y + 7;
		
		vs->x_start = 0;
		vs->y_start = y;
		
		vs->pixel_count = pixels;
		vs->parameter_CRC = 0;
		vs->reserved = 0;
		
		mlist[n].packet_header_count = sizeof(mddi_video_stream_packet_type) - 2;
		mlist[n].packet_data_count = pixels * 2;
		mlist[n].reserved = 0;
		s_wr32(&mlist[n].packet_data_pointer , ((unsigned) FBuff) + (y * PRIM_WIDTH* 2));

		mlist[n].link_controller_flags = 0;
		s_wr32(&mlist[n].next_packet_pointer, (unsigned) (mlist + n + 1));
	}

	mlist[n-1].link_controller_flags = 1;
	s_wr32(&mlist[n-1].next_packet_pointer, 0);

	mddi_host_reg_out(CMD, MDDI_CMD_HIBERNATE);
	mddi_host_reg_out(CMD, MDDI_CMD_LINK_ACTIVE);

	mddi_host_reg_out(PRI_PTR, (unsigned) mlist);

	while(!s_mddi_update_done()) ;

}

static struct timer_list image_timer;
static void s_image_refresh_work(unsigned long d)
{
	int i;
	init_timer(&image_timer);
	image_timer.function = s_image_refresh_work;
	image_timer.data = d;
	image_timer.expires =
	    jiffies + (HZ/1000);
	add_timer(&image_timer);

	for(i=0; i<2; i++) // write 2 images successively
		s_write_image_to_edram1();
}

#endif //TEST_WR_IMAGE
/* end test code */


#define PANEL_LMS

/* sequence op followed */
static s_reg_val_pair_t 
s_seq_init_setup[]={
#ifdef PANEL_LMS
	{DPSET0,			0x4BEC0066},	// # MDC.DPSET0  # Setup DPLL parameters
	{DPSET1,			0x00000113},	//	 # MDC.DPSET1 
	{DPSUS, 			0x00000000},	//	 # MDC.DPSUS  # Set DPLL oscillation enable
	{DPRUN, 			0x00000001},	//	 # MDC.DPRUN  # Release reset signal for DPLL
	{0,				500		},	// wait_ms(500)
	{SYSCKENA, 		0x00000001},	//   # MDC.SYSCKENA  # Enable system clock output

#ifdef SPI_SW_SIMULATE
	{CLKENB,			0x000000E1},	//	 # SYS.CLKENB  # Enable clocks for each module (without DCLK , i2cCLK, eDram)
#else
	{CLKENB,			0x000000E3},	//	 # SYS.CLKENB  # Enable clocks for each module (without DCLK , i2cCLK, eDram)
#endif

	{GPIODATA, 		0x03FF0000},	//   # GPI .GPIODATA  # GPIO2(RESET_LCD_N) set to 0 , GPIO3(eDRAM_Power) set to 0
	{GPIODIR, 		0x0000024D},	//   # GPI .GPIODIR  # Select direction of GPIO port (0,2,3,6,9 output)
	{GPIOSEL, 		0x00000173},	//   # SYS.GPIOSEL  # GPIO port multiplexing control
	{GPIOPC, 		0x03C300C0},	//   # GPI .GPIOPC  # GPIO2,3 PD cut
	{WKREQ, 		0x00000000},	//   # SYS.WKREQ  # Wake-up request event is VSYNC alignment
	{GPIOIS, 			0x00000000},	//   # GPI .GPIOIS  # Set interrupt sense of GPIO
	{GPIOIEV, 		0x00000001},	//   # GPI .GPIOIEV  # Set interrupt event of GPIO
	{GPIOIC, 		0x000003FF},	//   # GPI .GPIOIC  # GPIO interrupt clear
	{GPIODATA, 		0x00060006},	//   # GPI .GPIODATA  # Release LCDD reset
	{GPIODATA, 		0x00080008},	//   # GPI .GPIODATA  # eDRAM VD supply
	{GPIODATA, 		0x02000200},	//   # GPI .GPIODATA  # TEST LED ON
	{DRAMPWR, 		0x00000001},	//   # SYS.DRAMPWR  # eDRAM power up

#ifdef SPI_SW_SIMULATE
	{CLKENB,			0x000000E3},	//	 # SYS.CLKENB  # Enable clocks for  eDram
#else
	{CLKENB,			0x000000EB},	//	 # SYS.CLKENB  # Enable clocks for	eDram
#endif

	{TIMER0CTRL, 	0x00000060},	//   # PWM.Timer0Control  # PWM0 output stop
	{TIMER0LOAD, 	0x00001388},	//   # PWM.Timer0Load  # PWM0 10kHz , Duty 99 (BackLight OFF)
	{PWM0OFF, 		0x00000000}, 	//   # PWM.PWM0OFF  
//	{TIMER1CTRL, 	0x00000060},	//   # PWM.Timer1Control  # PWM1 output stop
//	{TIMER1LOAD, 	0x00001388},	//   # PWM.Timer1Load  # PWM1 10kHz , Duty 99 (BackLight OFF)
//	{PWM1OFF, 		0x00000001}, 	//	 # PWM.PWM1OFF
	{TIMER0CTRL, 	0x000000E0},	//   # PWM.Timer0Control  # PWM0 output start
//	{TIMER1CTRL, 	0x000000E0},	//   # PWM.Timer1Control  # PWM1 output start
	{PWMCR, 		0x00000003},	//   # PWM.PWMCR  # PWM output enable
	{0, 				1		},	//  wait_ms(1);
	
#ifndef SPI_SW_SIMULATE
/*	{SSICTL,			0x00063111},	//	 # SPI .SSICTL	# SPI operation mode setting
	{SSITIME,		0x00000100},	//	 # SPI .SSITIME  # SPI serial interface timing setting
	{CNT_DIS,		0x00000000},	//	 # SYS.CNT_DIS	# Enable control pins to LCD/SPI module
	{SSICTL,			0x00063113},	//	 # SPI .SSICTL	# Set SPI active mode
*/	{SSICTL,			0x00000171},	//	 # SPI .SSICTL	# SPI operation mode setting
	{SSITIME,		0x00000100},	//	 # SPI .SSITIME  # SPI serial interface timing setting
//	{CNT_DIS, 		0x00000000},	//	 # SYS.CNT_DIS  # Enable control pins to LCD/SPI module
	{SSICTL,			0x00000173},	//	 # SPI .SSICTL	# Set SPI active mode
	{SSITX, 			0x00000000},
	{0, 				1		},	//  wait_ms(1);
	{SSITX, 			0x00000000},
	{0, 				1		},	//  wait_ms(1);
	{SSITX, 			0x00000000},
	{0, 				1		},	//  wait_ms(1);
#endif

#else
	{DPSET0, 		0x4BEC0066},	// # MDC.DPSET0  # Setup DPLL parameters
	{DPSET1, 		0x00000113},	//   # MDC.DPSET1 
	{DPSUS, 			0x00000000},	//   # MDC.DPSUS  # Set DPLL oscillation enable
	{DPRUN, 			0x00000001},	//   # MDC.DPRUN  # Release reset signal for DPLL
	{0, 				14		},	// wait_ms(14)
	{SYSCKENA, 		0x00000001},	//   # MDC.SYSCKENA  # Enable system clock output
	{CLKENB, 		0x000000EF},	//   # SYS.CLKENB  # Enable clocks for each module (without DCLK , i2cCLK)
	{GPIODATA, 		0x03FF0000},	//   # GPI .GPIODATA  # GPIO2(RESET_LCD_N) set to 0 , GPIO3(eDRAM_Power) set to 0
	{GPIODIR, 		0x0000024D},	//   # GPI .GPIODIR  # Select direction of GPIO port (0,2,3,6,9 output)
	{GPIOSEL, 		0x00000173},	//   # SYS.GPIOSEL  # GPIO port multiplexing control
	{GPIOPC, 		0x03C300C0},	//   # GPI .GPIOPC  # GPIO2,3 PD cut
	{WKREQ, 		0x00000000},	//   # SYS.WKREQ  # Wake-up request event is VSYNC alignment
	{GPIOIS, 			0x00000000},	//   # GPI .GPIOIS  # Set interrupt sense of GPIO
	{GPIOIEV, 		0x00000001},	//   # GPI .GPIOIEV  # Set interrupt event of GPIO
	{GPIOIC, 		0x000003FF},	//   # GPI .GPIOIC  # GPIO interrupt clear
	{GPIODATA, 		0x00060006},	//   # GPI .GPIODATA  # Release LCDD reset
	{GPIODATA, 		0x00080008},	//   # GPI .GPIODATA  # eDRAM VD supply
	{GPIODATA, 		0x02000200},	//   # GPI .GPIODATA  # TEST LED ON
	{DRAMPWR, 		0x00000001},	//   # SYS.DRAMPWR  # eDRAM power up
	{TIMER0CTRL, 	0x00000060},	//   # PWM.Timer0Control  # PWM0 output stop
	{TIMER0LOAD, 	0x00001388},	//   # PWM.Timer0Load  # PWM0 10kHz , Duty 99 (BackLight OFF)
#if 0
	{ PWM0OFF, 		0x00001387}, 	// SURF 100% backlight
	{ PWM0OFF, 		0x00000000}, 	// FFA 100% backlight
#endif
//	{ PWM0OFF, 		0x000009C3},	// 50% BL
	{TIMER1CTRL, 	0x00000060},	//   # PWM.Timer1Control  # PWM1 output stop
	{TIMER1LOAD, 	0x00001388},	//   # PWM.Timer1Load  # PWM1 10kHz , Duty 99 (BackLight OFF)
	{PWM1OFF,		0x00001387},	//	 # PWM.PWM1OFF
	{TIMER0CTRL, 	0x000000E0},	//   # PWM.Timer0Control  # PWM0 output start
	{TIMER1CTRL, 	0x000000E0},	//   # PWM.Timer1Control  # PWM1 output start
	{PWMCR, 		0x00000003},	//   # PWM.PWMCR  # PWM output enable
	{0, 				1		},	//  wait_ms(1);
	{SSICTL, 			0x00000799},	//   # SPI .SSICTL  # SPI operation mode setting
	{SSITIME, 		0x00000100},	//   # SPI .SSITIME  # SPI serial interface timing setting
	{SSICTL, 			0x0000079b},	//   # SPI .SSICTL  # Set SPI active mode
	{SSITX, 			0x00000000},	//   # SPI.SSITX  # Release from Deep Stanby mode

	{0, 				1		},	//  wait_ms(1);
	{SSITX, 			0x00000000},	//   # SPI.SSITX 
	{0, 				1		},	//  wait_ms(1);
	{SSITX, 			0x00000000},	//   # SPI.SSITX
	{0, 				1		},	//  wait_ms(1);
	
	{SSITX, 			0x000800BA},	//   # SPI.SSITX          *NOTE 1  # Command setting of SPI block
	{SSITX, 			0x00000111},	//     # Display mode setup(1) : Normaly Black
	{SSITX, 			0x00080036},	//     # Command setting of SPI block
	{SSITX, 			0x00000100},	//     # Memory access control
	{0, 				2		},	//  wait_ms(2);    //      #  Wait SPI fifo empty
	{SSITX, 			0x000800BB},	//   # Command setting of SPI block
	{SSITX, 			0x00000100},	//   # Display mode setup(2)
	{SSITX, 			0x0008003A},	//   # Command setting of SPI block
	{SSITX, 			0x00000160},	//   # RGB Interface data format
	{0,				2		},	//  wait_ms(2);      //    #  Wait SPI fifo empty
	{SSITX, 			0x000800BF},	//   # Command setting of SPI block
	{SSITX, 			0x00000100},	//   # Drivnig method
	{SSITX, 			0x000800B1},	//   # Command setting of SPI block
	{SSITX, 			0x0000015D},	//   # Booster operation setup
	{0, 				2		},	//  wait_ms(2);      //    #  Wait SPI fifo empty
	{SSITX, 			0x000800B2},	//   # Command setting of SPI block
	{SSITX, 			0x00000133},	//   # Booster mode setup
	{SSITX,			0x000800B3},	//   # Command setting of SPI block
	{SSITX, 			0x00000122},	//     # Booster frequencies setup
	{0,				2		},	//  wait_ms(2);      //    #  Wait SPI fifo empty
	{SSITX, 			0x000800B4},	//     # Command setting of SPI block
	{SSITX, 			0x00000102},	//     # OP-amp capability/System clock freq. division setup
	{SSITX, 			0x000800B5},	//     # Command setting of SPI block
	{SSITX, 			0x0000011F},	//     # VCS Voltage adjustment  (1C->1F for Rev 2)
	{0,				2		},	//  wait_ms(2);      //    #  Wait SPI fifo empty
	{SSITX, 			0x000800B6},	//     # Command setting of SPI block
	{SSITX, 			0x00000128},	//     # VCOM Voltage adjustment
	{SSITX, 			0x000800B7},	//     # Command setting of SPI block
	{SSITX, 			0x00000103},	//     # Configure an external display signal
	{0, 				2		},	//  wait_ms(2);      //    #  Wait SPI fifo empty
	{SSITX, 			0x000800B9},	//     # Command setting of SPI block
	{SSITX, 			0x00000120},	//     # DCCK/DCEV timing setup
	{SSITX, 			0x000800BD},	//     # Command setting of SPI block
	{SSITX, 			0x00000102},	//     # ASW signal control
	{0, 				2		},	//  wait_ms(2);      //    #  Wait SPI fifo empty
	{SSITX, 			0x000800BE},	//     # Command setting of SPI block
	{SSITX, 			0x00000100},	//     # Dummy display (white/black) count setup for QUAD Data operation
	{SSITX, 			0x000800C0},	//     # Command setting of SPI block
	{SSITX, 			0x00000111},	//     # wait_ms(-out FR count setup (A)
	{0, 				2		},	//  wait_ms(2);      //    #  Wait SPI fifo empty
	{SSITX, 			0x000800C1},	//     # Command setting of SPI block
	{SSITX, 			0x00000111},	//     # wait_ms(-out FR count setup (B)
	{SSITX, 			0x000800C2},	//     # Command setting of SPI block
	{SSITX, 			0x00000111},	//     # wait_ms(-out FR count setup (C)
	{0, 				2		},	//  wait_ms(2);      //    #  Wait SPI fifo empty
	{SSITX, 			0x000800C3},	//     # Command setting of SPI block
	{SSITX, 			0x0008010A},	//     # wait_ms(-in line clock count setup (D)
	{SSITX, 			0x0000010A},	//
	{0, 				2		},	//  wait_ms(2);      //    #  Wait SPI fifo empty
	{SSITX, 			0x000800C4},	//     # Command setting of SPI block
	{SSITX, 			0x00080160},	//     # Seep-in line clock count setup (E)
	{SSITX, 			0x00000160},	//
	{0, 				2		},	//  wait_ms(2);      //    #  Wait SPI fifo empty
	{SSITX, 			0x000800C5},	//     # Command setting of SPI block
	{SSITX, 			0x00080160},	//     # wait_ms(-in line clock count setup (F)
	{SSITX, 			0x00000160},	//
	{0, 				2		},	//  wait_ms(2);      //    #  Wait SPI fifo empty
	{SSITX, 			0x000800C6},	//     # Command setting of SPI block
	{SSITX, 			0x00080160},	//     # wait_ms(-in line clock setup (G)
	{SSITX, 			0x00000160},	//
	{0, 				2		},	//  wait_ms(2);      //    #  Wait SPI fifo empty
	{SSITX, 			0x000800C7},	//     # Command setting of SPI block
	{SSITX, 			0x00080133},	//     # Gamma 1 fine tuning (1)
	{SSITX, 			0x00000143},	//
	{0, 				2		},	//  wait_ms(2);      //    #  Wait SPI fifo empty
	{SSITX, 			0x000800C8},	//     # Command setting of SPI block
	{SSITX, 			0x00000144},	//     # Gamma 1 fine tuning (2)
	{SSITX, 			0x000800C9},	//     # Command setting of SPI block
	{SSITX, 			0x00000133},	//     # Gamma 1 inclination adjustment
	{0, 				2		},	//  wait_ms(2);      //    #  Wait SPI fifo empty
	{SSITX, 			0x000800CA},	//     # Command setting of SPI block
	{SSITX, 			0x00000100},	//     # Gamma 1 blue offset adjustment
	{0, 				2		},	//  wait_ms(2);      //    #  Wait SPI fifo empty
	{SSITX, 			0x000800EC},	//     # Command setting of SPI block
	{SSITX, 			0x00080102},	//     # Total number of horizontal clock cycles (1) [PCLK Sync. VGA setting]
	{SSITX, 			0x00000118},	//
	{0, 				2		},	//  wait_ms(2);      //    #  Wait SPI fifo empty
	{SSITX, 			0x000800CF},	//     # Command setting of SPI block
	{SSITX, 			0x00000101},	//     # Blanking period control (1) [PCLK Sync. Table1 for VGA]
	{0, 				2		},	//  wait_ms(2);      //    #  Wait SPI fifo empty
	{SSITX, 			0x000800D0},	//     # Command setting of SPI block
	{SSITX, 			0x00080110},	//     # Blanking period control (2) [PCLK Sync. Table1 for VGA]
	{SSITX, 			0x00000104},	//
	{0, 				2		},	//  wait_ms(2);      //    #  Wait SPI fifo empty
	{SSITX, 			0x000800D1},	//     # Command setting of SPI block
	{SSITX, 			0x00000101},	//     # CKV timing control on/off [PCLK Sync. Table1 for VGA]
	{0, 				2		},	//  wait_ms(2);      //    #  Wait SPI fifo empty
	{SSITX, 			0x000800D2},	//     # Command setting of SPI block
	{SSITX, 			0x00080100},	//     # CKV1,2 timing control [PCLK Sync. Table1 for VGA]
	{SSITX, 			0x0000013A},	//
	{0, 				2		},	//  wait_ms(2);      //    #  Wait SPI fifo empty
	{SSITX, 			0x000800D3},	//     # Command setting of SPI block
	{SSITX, 			0x00080100},	//     # OEV timing control [PCLK Sync. Table1 for VGA]
	{SSITX, 			0x0000013A},	//
	{0, 				2		},	//  wait_ms(2);      //    #  Wait SPI fifo empty
	{SSITX, 			0x000800D4},	//     # Command setting of SPI block
	{SSITX, 			0x00080124},	//     # ASW timing control (1) [PCLK Sync. Table1 for VGA]
	{SSITX, 			0x0000016E},	//
	{0, 				1		},	//  wait_ms(1);      //    #  Wait SPI fifo empty
	{SSITX, 			0x000800D5},	//     # Command setting of SPI block
	{SSITX, 			0x00000124},	//     # ASW timing control (2) [PCLK Sync. Table1 for VGA]
	{0, 				2		},	//  wait_ms(2);      //    #  Wait SPI fifo empty
	{SSITX, 			0x000800ED},	//     # Command setting of SPI block
	{SSITX, 			0x00080101},	//     # Total number of horizontal clock cycles (2) [PCLK Sync. Table1 for QVGA ]
	{SSITX, 			0x0000010A},	//
	{0, 				2		},	//  wait_ms(2);      //    #  Wait SPI fifo empty
	{SSITX, 			0x000800D6},	//     # Command setting of SPI block
	{SSITX, 			0x00000101},	//     # Blanking period control (1) [PCLK Sync. Table2 for QVGA]
	{0, 				2		},	//  wait_ms(2);      //    #  Wait SPI fifo empty
	{SSITX, 			0x000800D7},	//     # Command setting of SPI block
	{SSITX, 			0x00080110},	//     # Blanking period control (2) [PCLK Sync. Table2 for QVGA]
	{SSITX, 			0x0000010A},	//
	{0, 				2		},	//  wait_ms(2);      //    #  Wait SPI fifo empty
	{SSITX, 			0x000800D8},	//     # Command setting of SPI block
	{SSITX, 			0x00000101},	//     # CKV timing control on/off [PCLK Sync. Table2 for QVGA]
	{0, 				2		},	//  wait_ms(2);      //    #  Wait SPI fifo empty
	{SSITX, 			0x000800D9},	//     # Command setting of SPI block
	{SSITX, 			0x00080100},	//     # CKV1,2 timing control [PCLK Sync. Table2 for QVGA]
	{SSITX, 			0x00000114},	//
	{0, 				2		},	//  wait_ms(2);      //    #  Wait SPI fifo empty
	{SSITX, 			0x000800DE},	//     # Command setting of SPI block
	{SSITX, 			0x00080100},	//     # OEV timing control [PCLK Sync. Table2 for QVGA]
	{SSITX, 			0x00000114},	//
	{0, 				2		},	//  wait_ms(2);      //    #  Wait SPI fifo empty
	{SSITX, 			0x000800DF},	//     # Command setting of SPI block
	{SSITX, 			0x00080112},	//     # ASW timing control (1) [PCLK Sync. Table2 for QVGA]
	{SSITX, 			0x0000013F},	//
	{0, 				2		},	//  wait_ms(2);      //    #  Wait SPI fifo empty
	{SSITX, 			0x000800E0},	//     # Command setting of SPI block
	{SSITX, 			0x0000010B},	//     # ASW timing control (2) [PCLK Sync. Table2 for QVGA]
	{SSITX, 			0x000800E2},	//     # Command setting of SPI block
	{SSITX, 			0x00000101},	//     # Built-in oscillator frequency division setup [Frequency division ratio : 2 (60Hq)
	{0, 				2		},	//  wait_ms(2);      //    #  Wait SPI fifo empty
	{SSITX, 			0x000800E3},	//     # Command setting of SPI block
	{SSITX, 			0x00000136},	//     # Built-in oscillator clock count setup
	{0, 				2		},	//  wait_ms(2);      //    #  Wait SPI fifo empty
	{SSITX, 			0x000800E4},	//     # Command setting of SPI block
	{SSITX, 			0x00080100},	//     # CKV timing control for using build-in osc
	{SSITX, 			0x00000103},	//
	{0, 				2		},	//  wait_ms(2);      //    #  Wait SPI fifo empty
	{SSITX, 			0x000800E5},	//     # Command setting of SPI block
	{SSITX, 			0x00080102},	//     # OEV timing control for using build-in osc
	{SSITX, 			0x00000104},	//
	{0, 				2		},	//  wait_ms(2);      //    #  Wait SPI fifo empty
	{SSITX, 			0x000800E6},	//     # Command setting of SPI block
	{SSITX, 			0x00000103},	//     # DCEV timing control for using build-in osc
	{0, 				2		},	//  wait_ms(2);      //    #  Wait SPI fifo empty
	{SSITX, 			0x000800E7},	//     # Command setting of SPI block
	{SSITX, 			0x00080104},	//     # ASW timing setup for using build-in osc(1)
	{SSITX, 			0x0000010A},	//
	{0, 				2		},	//  wait_ms(2);      //    #  Wait SPI fifo empty
	{SSITX, 			0x000800E8},	//     # Command setting of SPI block
	{SSITX, 			0x00000104},	//     # ASW timing setup for using build-in osc(2)
	
	{CLKENB, 		0x000001EF},	//   # SYS.CLKENB  # DCLK enable
	{START, 			0x00000000},	//   # LCD.START  # LCDC wait_ms( mode
	{WRSTB, 			0x0000003F},	//   # LCD.WRSTB  # write_client_reg( strobe
	{RDSTB, 			0x00000432},	//   # LCD.RDSTB  # Read strobe
	{PORT_ENB, 		0x00000002},	//   # LCD.PORT_ENB  # Asynchronous port enable
	{VSYNIF, 			0x00000000},	//   # LCD.VSYNCIF  # VSYNC I/F mode set
	{ASY_DATA, 		0x80000000},	//   # LCD.ASY_DATx  # Index setting of SUB LCDD
	{ASY_DATB, 		0x00000001},	//     # Oscillator start
	{ASY_CMDSET, 	0x00000005},	//   # LCD.ASY_CMDSET  # Direct command transfer enable
	{ASY_CMDSET, 	0x00000004},	//   # LCD.ASY_CMDSET  # Direct command transfer disable
	{0, 				10		},	//  wait_ms(10);
	{ASY_DATA, 		0x80000000},	//   # LCD.ASY_DATx  # DUMMY write_client_reg(Å@*NOTE2
	{ASY_DATB, 		0x80000000},	//
	{ASY_DATC, 		0x80000000},	//
	{ASY_DATD, 		0x80000000},	//
	{ASY_CMDSET, 	0x00000009},	//  # LCD.ASY_CMDSET
	{ASY_CMDSET, 	0x00000008},	//   # LCD.ASY_CMDSET
	{ASY_DATA, 		0x80000007},	//   # LCD.ASY_DATx  # Index setting of SUB LCDD
	{ASY_DATB, 		0x00004005},	//     # LCD driver control
	{ASY_CMDSET, 	0x00000005},	//   # LCD.ASY_CMDSET  # Direct command transfer enable
	{ASY_CMDSET, 	0x00000004},	//   # LCD.ASY_CMDSET  # Direct command transfer disable
	{0, 				20		},	//  wait_ms(20);
	{ASY_DATA, 		0x80000059},	//   # LCD.ASY_DATx  # Index setting of SUB LCDD
	{ASY_DATB, 		0x00000000},	//     # LTPS I/F control
	{ASY_CMDSET, 	0x00000005},	//   # LCD.ASY_CMDSET  # Direct command transfer enable
	{ASY_CMDSET, 	0x00000004},	//   # LCD.ASY_CMDSET  # Direct command transfer disable
	{VSYNIF, 			0x00000001},	//   # LCD.VSYNCIF  # VSYNC I/F mode OFF
	{PORT_ENB, 		0x00000001}	//   # LCD.PORT_ENB  # SYNC I/F  output select
#endif
};

static void tc358721xbg_common_initial_setup(void)
{
	int i = 0;
	int count = sizeof(s_seq_init_setup) / 
		sizeof(s_reg_val_pair_t);
	
	for(i=0; i<count; i++)
		s_reg_val_pair_op(s_seq_init_setup[i].reg, 
			s_seq_init_setup[i].val);

#if 0 // write and read register if setting is right or not
{
	uint32 dpset0=0, dpset1=0,dpsus=0,dprun=0,sysckena=0,clkenb=0;
	//{DPSET0,			0x4BEC0066},	// # MDC.DPSET0  # Setup DPLL parameters (100MHz)
	//{DPSET1,			0x00000113},	//	 # MDC.DPSET1 
	//{DPSUS, 			0x00000000},	//	 # MDC.DPSUS  # Set DPLL oscillation enable
	//{DPRUN, 			0x00000001},	//	 # MDC.DPRUN  # Release reset signal for DPLL
	//{SYSCKENA,		0x00000001},	//	 # MDC.SYSCKENA  # Enable system clock output
	//{CLKENB,			0x000001EB} //	 # SYS.CLKENB  # Enable clocks for each module (without DCLK , i2cCLK, eDram)
	dpset0 = read_client_reg(DPSET0);
	dpset1 = read_client_reg(DPSET1);
	dpsus = read_client_reg(DPSUS);
	dprun = read_client_reg(DPRUN);
	sysckena= read_client_reg(SYSCKENA);
	clkenb= read_client_reg(CLKENB);
	printk("\nDPLL configure:dpset0=%x, dpset1=%x, dpsus=%x, dprun=%x, sysckena=%x, clkenb=%x\n",
			dpset0,dpset1,dpsus,dprun,sysckena,clkenb);
}
#endif

	// primary driver setting
	if(panel_sel && panel_sel->power_on)
		panel_sel->power_on();

	mddi_tc358721xbg_state_transition(TC358721_STATE_PRIM_SEC_STANDBY,
				      TC358721_STATE_PRIM_SEC_READY);
}

static s_reg_val_pair_t
s_seq_sec_cont_update_start[] = {
	{VSYNIF, 			0x00000000},
	{PORT_ENB, 		0x00000002},
	{INTMASK, 		0x00000001},
	{TTBUSSEL, 		0x0000000B},
	{MONI, 			0x00000008},
	{CLKENB, 		0x000000EF},
	{CLKENB, 		0x000010EF},
	{CLKENB, 		0x000011EF},
	{BITMAP4, 		0x00DC00B0},
	{HCYCLE,	 		0x0000006B},
	{HSW, 			0x00000003},
	{HDE_START, 		0x00000002},
	{HDE_SIZE, 		0x00000057},
	{VCYCLE, 		0x000000E6},
	{VSW, 			0x00000001},
	{VDE_START, 		0x00000003},
	{VDE_SIZE, 		0x000000DB},
	{WRSTB, 			0x00000015},
	{MPLFBUF, 		0x00000004},
	{ASY_DATA, 		0x80000021},
	{ASY_DATB, 		0x00000000},
	{ASY_DATC, 		0x80000022},
	{ASY_CMDSET, 	0x00000007},
	{PXL, 			0x00000089},
	{VSYNIF, 			0x00000001},
	{0, 				2		}
};

static void tc358721xbg_sec_cont_update_start(void)
{
	int i = 0;
	int count = sizeof(s_seq_sec_cont_update_start) / 
		sizeof(s_reg_val_pair_t);
	
	for(i=0; i<count; i++)
		s_reg_val_pair_op(s_seq_sec_cont_update_start[i].reg, 
			s_seq_sec_cont_update_start[i].val);
}

static s_reg_val_pair_t
s_seq_sec_cont_update_stop[]={
	{PXL, 			0x00000000},
	{VSYNIF, 			0x00000000},
	{START, 			0x00000000},
	{ASY_CMDSET, 	0x00000000},
	{0, 				3		},
	{SRST, 			0x00000002},
	{0, 				3		},
	{SRST, 			0x00000003}
};

static void tc358721xbg_sec_cont_update_stop(void)
{
	int i = 0;
	int count = sizeof(s_seq_sec_cont_update_stop) / 
		sizeof(s_reg_val_pair_t);
	
	for(i=0; i<count; i++)
		s_reg_val_pair_op(s_seq_sec_cont_update_stop[i].reg, 
			s_seq_sec_cont_update_stop[i].val);
}

static s_reg_val_pair_t
s_seq_sec_sleep_in[]={
	{VSYNIF, 			0x00000000},
	{PORT_ENB, 		0x00000002},
	{ASY_DATA, 		0x80000007},
	{ASY_DATB, 		0x00004016},
	{ASY_CMDSET, 	0x00000005},
	{ASY_CMDSET, 	0x00000004},
	{0, 				2		},
	{ASY_DATA, 		0x80000059},
	{ASY_DATB, 		0x00000019},
	{ASY_CMDSET, 	0x00000005},
	{ASY_CMDSET, 	0x00000004},
	{0, 				2		},
	{ASY_DATA, 		0x80000059},
	{ASY_DATB, 		0x0000000B},
	{ASY_CMDSET, 	0x00000005},
	{ASY_CMDSET, 	0x00000004},
	{0, 				2		},
	{ASY_DATA, 		0x80000059},
	{ASY_DATB, 		0x00000002},
	{ASY_CMDSET, 	0x00000005},
	{ASY_CMDSET, 	0x00000004},
	{0, 				4		},
	{ASY_DATA, 		0x80000010},
	{ASY_DATB, 		0x00000300},
	{ASY_CMDSET, 	0x00000005},
	{ASY_CMDSET, 	0x00000004},
	{0, 				4		},
	{ASY_DATA, 		0x80000059},
	{ASY_DATB, 		0x00000000},
	{ASY_CMDSET, 	0x00000005},
	{ASY_CMDSET, 	0x00000004},
	{0, 				2		},
	{ASY_DATA, 		0x80000007},
	{ASY_DATB, 		0x00004004},
	{ASY_CMDSET, 	0x00000005},
	{ASY_CMDSET, 	0x00000004},
	{0, 				2		},
	{PORT, 			0x00000000},
	{PXL, 			0x00000000},
	{START,		 	0x00000000},
	{REGENB, 		0x00000001},
	/* Sleep in sequence */
	{ASY_DATA, 		0x80000010},
	{ASY_DATB, 		0x00000302},
	{ASY_CMDSET, 	0x00000005},
	{ASY_CMDSET, 	0x00000004}
};

static void tc358721xbg_sec_sleep_in(void)
{
	int i = 0;
	int count = sizeof(s_seq_sec_sleep_in) / 
		sizeof(s_reg_val_pair_t);
	
	for(i=0; i<count; i++)
		s_reg_val_pair_op(s_seq_sec_sleep_in[i].reg, 
			s_seq_sec_sleep_in[i].val);
}

static s_reg_val_pair_t
s_seq_sec_sleep_out[]={
	{VSYNIF, 			0x00000000},
	{PORT_ENB, 		0x00000002},
	{ASY_DATA, 		0x80000010},
	{ASY_DATB, 		0x00000300},
	{ASY_CMDSET, 	0x00000005},
	{ASY_CMDSET, 	0x00000004},
	/*  Display ON sequence */
	{ASY_DATA, 		0x80000011},
	{ASY_DATB, 		0x00000812},
	{ASY_DATC, 		0x80000012},
	{ASY_DATD, 		0x00000003},
	{ASY_DATE, 		0x80000013},
	{ASY_DATF, 		0x00000909},
	{ASY_DATG, 		0x80000010},
	{ASY_DATH, 		0x00000040},
	{ASY_CMDSET, 	0x00000001},
	{ASY_CMDSET, 	0x00000000},
	{0, 				4		},
	{ASY_DATA, 		0x80000010},
	{ASY_DATB, 		0x00000340},
	{ASY_CMDSET, 	0x00000005},
	{ASY_CMDSET, 	0x00000004},
	{0, 				6		},
	{ASY_DATA, 		0x80000010},
	{ASY_DATB, 		0x00003340},
	{ASY_DATC, 		0x80000007},
	{ASY_DATD, 		0x00004007},
	{ASY_CMDSET, 	0x00000009},
	{ASY_CMDSET, 	0x00000008},
	{0, 				1		},
	{ASY_DATA, 		0x80000007},
	{ASY_DATB, 		0x00004017},
	{ASY_DATC, 		0x8000005B},
	{ASY_DATD, 		0x00000000},
	{ASY_DATE, 		0x80000059},
	{ASY_DATF, 		0x00000011},
	{ASY_CMDSET, 	0x0000000D},
	{ASY_CMDSET, 	0x0000000C},
	{0, 				2		},
	{ASY_DATA, 		0x80000059},
	{ASY_DATB, 		0x00000019},
	{ASY_CMDSET, 	0x00000005},
	{ASY_CMDSET, 	0x00000004},
	{0, 				2		},
	{ASY_DATA, 		0x80000059},
	{ASY_DATB, 		0x00000079},
	{ASY_CMDSET, 	0x00000005},
	{ASY_CMDSET, 	0x00000004},
	{0, 				2		},
	{ASY_DATA, 		0x80000059},
	{ASY_DATB, 		0x000003FD},
	{ASY_CMDSET, 	0x00000005},
	{ASY_CMDSET, 	0x00000004},
	{0, 				2		}
};

static void tc358721xbg_sec_sleep_out(void)
{
	int i = 0;
	int count = sizeof(s_seq_sec_sleep_out) / 
		sizeof(s_reg_val_pair_t);
	
	for(i=0; i<count; i++)
		s_reg_val_pair_op(s_seq_sec_sleep_out[i].reg, 
			s_seq_sec_sleep_out[i].val);
}

static s_reg_val_pair_t
s_seq_prim_lcd_off[]={
#ifdef PANEL_LMS
	{PORT,		0x00000000},
	{REGENB,		0x00000001},
	{0, 			16		},
	{PXL,		0x00000000},
	{START, 		0x00000000},
	{REGENB,		0x00000001},
	{0, 			32		},

#else
	/* Main panel power off (Deep standby in) */
	{SSITX, 		0x000800BC},
	{SSITX, 		0x00000100},
	{SSITX, 		0x00000028},
	{0, 			1		},
	{SSITX, 		0x000800B8},
	{SSITX, 		0x00000180},
	{SSITX, 		0x00000102},
	{SSITX, 		0x00000010},
		
	{PORT, 		0x00000003},
	{REGENB, 	0x00000001},
	{0, 			1		},
	{PXL, 		0x00000000},
	{START, 		0x00000000},
	{REGENB, 	0x00000001},
	{0, 			3		},
	
	{SSITX, 		0x000800B0},
	{SSITX, 		0x00000100}
#endif
};

static void tc358721xbg_prim_lcd_off(void)
{
	int i = 0;
	int count = sizeof(s_seq_prim_lcd_off) / 
		sizeof(s_reg_val_pair_t);
	
	// primary sleep
	if(panel_sel && panel_sel->display_off)
		panel_sel->display_off();
	
	for(i=0; i<count; i++)
		s_reg_val_pair_op(s_seq_prim_lcd_off[i].reg, 
			s_seq_prim_lcd_off[i].val);

	// primary standby
	if(panel_sel && panel_sel->standby_in)
		panel_sel->standby_in();
	
	mddi_tc358721xbg_state_transition(TC358721_STATE_PRIM_NORMAL_MODE,
				      TC358721_STATE_PRIM_SEC_STANDBY);
}

static s_reg_val_pair_t
s_seq_sec_lcd_off[]={
	{VSYNIF, 			0x00000000},
	{PORT_ENB, 		0x00000002},
	{ASY_DATA, 		0x80000007},
	{ASY_DATB, 		0x00004016},
	{ASY_CMDSET, 	0x00000005},
	{ASY_CMDSET, 	0x00000004},
	{0, 				2		},
	{ASY_DATA, 		0x80000059},
	{ASY_DATB, 		0x00000019},
	{ASY_CMDSET, 	0x00000005},
	{ASY_CMDSET, 	0x00000004},
	{0, 				2		},
	{ASY_DATA, 		0x80000059},
	{ASY_DATB, 		0x0000000B},
	{ASY_CMDSET, 	0x00000005},
	{ASY_CMDSET, 	0x00000004},
	{0, 				2		},
	{ASY_DATA, 		0x80000059},
	{ASY_DATB, 		0x00000002},
	{ASY_CMDSET, 	0x00000005},
	{ASY_CMDSET, 	0x00000004},
	{0, 				4		},
	{ASY_DATA, 		0x80000010},
	{ASY_DATB, 		0x00000300},
	{ASY_CMDSET, 	0x00000005},
	{ASY_CMDSET, 	0x00000004},
	{0, 				4		},
	{ASY_DATA, 		0x80000059},
	{ASY_DATB, 		0x00000000},
	{ASY_CMDSET, 	0x00000005},
	{ASY_CMDSET, 	0x00000004},
	{0, 				2		},
	{ASY_DATA, 		0x80000007},
	{ASY_DATB, 		0x00004004},
	{ASY_CMDSET, 	0x00000005},
	{ASY_CMDSET, 	0x00000004},
	{0, 				2		},
	{PORT, 			0x00000000},
	{PXL, 			0x00000000},
	{START, 			0x00000000},
	{VSYNIF, 			0x00000001},
	{PORT_ENB, 		0x00000001},
	{REGENB, 		0x00000001}
};

static void tc358721xbg_sec_lcd_off(void)
{
	int i = 0;
	int count = sizeof(s_seq_sec_lcd_off) / 
		sizeof(s_reg_val_pair_t);
	
	for(i=0; i<count; i++)
		s_reg_val_pair_op(s_seq_sec_lcd_off[i].reg, 
			s_seq_sec_lcd_off[i].val);

	mddi_tc358721xbg_state_transition(TC358721_STATE_SEC_NORMAL_MODE,
				      TC358721_STATE_PRIM_SEC_STANDBY);
}

static s_reg_val_pair_t
s_seq_prim_start[]={
#ifdef PANEL_LMS
	{VSYNIF,			0x00000001},	// VSYNC I/F mode OFF
	{PORT_ENB,		0x00000001},	// SYNC I/F mode ON
//	{BITMAP0,		0x028001E0},	// MDC.BITMAP1	); // Setup of PITCH size to Frame buffer1(VGA)
	{BITMAP0,		0x01E00140},	// MDC.BITMAP1	); // Setup of PITCH size to Frame buffer1(VGA)
	{BITMAP1,		0x01E000A0},	// MDC.BITMAP2	); // Setup of PITCH size to Frame buffer1
	{BITMAP2,		0x01E000A0},	// MDC.BITMAP3	); // Setup of PITCH size to Frame buffer2
	{BITMAP3,		0x01E000A0},	// MDC.BITMAP4	); // Setup of PITCH size to Frame buffer3
	{BITMAP4,		0x00DC00B0},	// MDC.BITMAP5	); // Setup of PITCH size to Frame buffer4

#ifdef SPI_SW_SIMULATE
	{CLKENB, 		0x000031E3},	// SYS.CLKENB  ); // DCLK supply
#else
	{CLKENB,			0x000021EB},	// SYS.CLKENB  ); // DCLK supply 12.5MHz
	//{CLKENB,			0x000031EB},	// SYS.CLKENB  ); // DCLK supply 6.25MHz
	//{CLKENB,			0x000001EB},	// SYS.CLKENB  ); // DCLK supply 20MHz
#endif

	{PORT_ENB, 		0x00000001},	// LCD.PORT_ENB  ); // Synchronous port enable
	{PORT, 			0x00000004},	// LCD.PORT  ); // Polarity of DE is set to high active
	{PXL, 			0x00000002},	// LCD.PXL  ); // ACTMODE 2 set (1st frame black data output)
	{MPLFBUF, 		0x00000000},	// LCD.MPLFBUF  ); // Select the reading buffer

#ifdef MDDI_TC358721_61HZ_REFRESH
//	{HCYCLE,			0x000000FC},	// LCD.HCYCLE  ); // Setup to VGA size
	//{HCYCLE,			0x000000AC},	// LCD.HCYCLE  ); // Setup to VGA size
	{HCYCLE,			0x000000b3},	// LCD.HCYCLE  ); // 360=30+320+10
#else
	//{HCYCLE,			0x0000010b},
	{HCYCLE,			0x000000b3},	// LCD.HCYCLE  ); // 360=30+320+10
#endif

	//{HSW,			0x00000003},	// LCD.HSW 
	//{HDE_START, 		0x00000007},	// LCD.HDE_START 
	//{HDE_SIZE,		0x00000140},	// LCD.HDE_SIZE//320
	//{VCYCLE,			0x000001E6},	// LCD.VCYCLE//486
	//{VSW,			0x00000002},	// LCD.VSW //2
	//{VDE_START, 		0x00000003},	// LCD.VDE_START
	//{VDE_SIZE,		0x000001DF},	// LCD.VDE_SIZE//480
	{HSW,			0x00000004},	// LCD.HSW //10
	{HDE_START, 		0x0000000e},	// LCD.HDE_START //30
//	{HDE_SIZE,		0x000000EF},	// LCD.HDE_SIZE
	{HDE_SIZE,		0x0000009F},	// LCD.HDE_SIZE
//	{VCYCLE,			0x00000285},	// LCD.VCYCLE
	{VCYCLE,			0x000001E5},	// LCD.VCYCLE
	{VSW,			0x00000001},	// LCD.VSW 
	{VDE_START, 		0x00000005},	// LCD.VDE_START//6
//	{VDE_SIZE,		0x0000027F},	// LCD.VDE_SIZE
	{VDE_SIZE,		0x000001DF},	// LCD.VDE_SIZE
//	{CNT_DIS, 		0x00000000},	//	 # SYS.CNT_DIS  # Enable control pins to LCD/SPI module
	{REGENB, 		0x00000001},
	{START, 			0x00000001},	// LCD.START  ); // LCDC - Pixel data transfer start
	{0,				32		},	//  wait_ms( 32  );
	
#else
	{VSYNIF, 			0x00000001},	// VSYNC I/F mode OFF
	{PORT_ENB, 		0x00000001},	// SYNC I/F mode ON
	{BITMAP1, 		0x01E000F0},	// MDC.BITMAP2  ); // Setup of PITCH size to Frame buffer1
	{BITMAP2, 		0x01E000F0},	// MDC.BITMAP3  ); // Setup of PITCH size to Frame buffer2
	{BITMAP3, 		0x01E000F0},	// MDC.BITMAP4  ); // Setup of PITCH size to Frame buffer3
	{BITMAP4, 		0x00DC00B0},	// MDC.BITMAP5  ); // Setup of PITCH size to Frame buffer4
	{CLKENB, 		0x000001EF},	// SYS.CLKENB  ); // DCLK supply
	{PORT_ENB, 		0x00000001},	// LCD.PORT_ENB  ); // Synchronous port enable
	{PORT, 			0x00000004},	// LCD.PORT  ); // Polarity of DE is set to high active
	{PXL, 			0x00000002},	// LCD.PXL  ); // ACTMODE 2 set (1st frame black data output)
	{MPLFBUF, 		0x00000000},	// LCD.MPLFBUF  ); // Select the reading buffer
#ifdef MDDI_TC358721_61HZ_REFRESH
	{HCYCLE, 		0x000000FC},	// LCD.HCYCLE  ); // Setup to VGA size
#else
	{HCYCLE, 		0x0000010b},
#endif
	{HSW, 			0x00000003},	// LCD.HSW 
	{HDE_START, 		0x00000007},	// LCD.HDE_START 
	{HDE_SIZE, 		0x000000EF},	// LCD.HDE_SIZE
	{VCYCLE, 		0x00000285},	// LCD.VCYCLE
	{VSW, 			0x00000001},	// LCD.VSW 
	{VDE_START, 		0x00000003},	// LCD.VDE_START
	{VDE_SIZE, 		0x0000027F},	// LCD.VDE_SIZE
	{START, 			0x00000001},	// LCD.START  ); // LCDC - Pixel data transfer start
	
	{0,				10		},	//  wait_ms( 10  );
	{SSITX, 			0x000800BC},	// SPI.SSITX  ); // Command setting of SPI block
	{SSITX, 			0x00000180},	// Display data setup
	{SSITX, 			0x0008003B},	// Command setting of SPI block
	{SSITX, 			0x00000100},	// Quad Data configuration - VGA
	{0,				1		},	//  wait_ms( 1          ); //  Wait SPI fifo empty
	{SSITX, 			0x000800B0},	// Command setting of SPI block
	{SSITX, 			0x00000116},	// Power supply ON/OFF control
	{0,				1		},	//  wait_ms( 1          ); //  Wait SPI fifo empty
	{SSITX, 			0x000800B8},	// Command setting of SPI block
	{SSITX, 			0x000801FF},	// Output control
	{SSITX, 			0x000001F5},	//
	{0,				1		},	//  wait_ms( 1);         //  Wait SPI fifo empty
	{SSITX, 			0x00000011},	// wait_ms(-out (Command only)
	{SSITX, 			0x00000029},	// Display on (Command only)
	{WKREQ, 		0x00000000},	//    # wakeREQ -> GPIO
	{WAKEUP, 		0x00000000},
	{INTMSK, 		0x00000001}
#endif
};

static void tc358721xbg_prim_start(void)
{
	int i = 0;
	int count = sizeof(s_seq_prim_start) / 
		sizeof(s_reg_val_pair_t);
	
	for(i=0; i<count; i++)
		s_reg_val_pair_op(s_seq_prim_start[i].reg, 
			s_seq_prim_start[i].val);

	// primary display on
	if(panel_sel && panel_sel->display_on)
		panel_sel->display_on();
	
	mddi_tc358721xbg_state_transition(TC358721_STATE_PRIM_SEC_READY,
				      TC358721_STATE_PRIM_NORMAL_MODE);
}

static s_reg_val_pair_t
s_seq_sec_start[]={
	{VSYNIF, 			0x00000000},
	{PORT_ENB, 		0x00000002},
	{CLKENB, 		0x000011EF},
	{BITMAP0, 		0x028001E0},
	{BITMAP1, 		0x00000000},
	{BITMAP2, 		0x00000000},
	{BITMAP3, 		0x00000000},
	{BITMAP4, 		0x00DC00B0},
	{PORT, 			0x00000000},
	{PXL, 			0x00000000},
	{MPLFBUF, 		0x00000004},
	{HCYCLE, 		0x0000006B},
	{HSW, 			0x00000003},
	{HDE_START, 		0x00000007},
	{HDE_SIZE, 		0x00000057},
	{VCYCLE, 		0x000000E6},
	{VSW, 			0x00000001},
	{VDE_START, 		0x00000003},
	{VDE_SIZE, 		0x000000DB},
	{ASY_DATA, 		0x80000001},
	{ASY_DATB, 		0x0000011B},
	{ASY_DATC, 		0x80000002},
	{ASY_DATD, 		0x00000700},
	{ASY_DATE, 		0x80000003},
	{ASY_DATF, 		0x00000230},
	{ASY_DATG, 		0x80000008},
	{ASY_DATH, 		0x00000402},
	{ASY_CMDSET, 	0x00000001},
	{ASY_CMDSET, 	0x00000000},
	{ASY_DATA, 		0x80000009},
	{ASY_DATB, 		0x00000000},
	{ASY_DATC, 		0x8000000B},
	{ASY_DATD, 		0x00000000},
	{ASY_DATE, 		0x8000000C},
	{ASY_DATF, 		0x00000000},
	{ASY_DATG, 		0x8000000D},
	{ASY_DATH, 		0x00000409},
	{ASY_CMDSET, 	0x00000001},
	{ASY_CMDSET, 	0x00000000},
	{ASY_DATA, 		0x8000000E},
	{ASY_DATB, 		0x00000409},
	{ASY_DATC, 		0x80000030},
	{ASY_DATD, 		0x00000000},
	{ASY_DATE, 		0x80000031},
	{ASY_DATF, 		0x00000100},
	{ASY_DATG, 		0x80000032},
	{ASY_DATH, 		0x00000104},
	{ASY_CMDSET, 	0x00000001},
	{ASY_CMDSET, 	0x00000000},
	{ASY_DATA, 		0x80000033},
	{ASY_DATB, 		0x00000400},
	{ASY_DATC, 		0x80000034},
	{ASY_DATD, 		0x00000306},
	{ASY_DATE, 		0x80000035},
	{ASY_DATF, 		0x00000706},
	{ASY_DATG, 		0x80000036},
	{ASY_DATH, 		0x00000707},
	{ASY_CMDSET, 	0x00000001},
	{ASY_CMDSET, 	0x00000000},
	{ASY_DATA, 		0x80000037},
	{ASY_DATB, 		0x00000004},
	{ASY_DATC, 		0x80000038},
	{ASY_DATD, 		0x00000000},
	{ASY_DATE, 		0x80000039},
	{ASY_DATF, 		0x00000000},
	{ASY_DATG, 		0x8000003A},
	{ASY_DATH, 		0x00000001},
	{ASY_CMDSET, 	0x00000001},
	{ASY_CMDSET, 	0x00000000},
	{ASY_DATA, 		0x80000044},
	{ASY_DATB, 		0x0000AF00},
	{ASY_DATC, 		0x80000045},
	{ASY_DATD, 		0x0000DB00},
	{ASY_DATE, 		0x08000042},
	{ASY_DATF, 		0x0000DB00},
	{ASY_DATG, 		0x80000021},
	{ASY_DATH, 		0x00000000},
	{ASY_CMDSET, 	0x00000001},
	{ASY_CMDSET, 	0x00000000},
	{PXL, 			0x0000000C},
	{VSYNIF, 			0x00000001},
	{ASY_DATA, 		0x80000022},
	{ASY_CMDSET, 	0x00000003},
	{START, 			0x00000001},
	{0,				60		},
	{PXL, 			0x00000000},
	{VSYNIF, 			0x00000000},
	{START, 			0x00000000},
	{ASY_CMDSET, 	0x00000000},
	{ASY_DATA, 		0x80000050},
	{ASY_DATB, 		0x00000000},
	{ASY_DATC, 		0x80000051},
	{ASY_DATD, 		0x00000E00},
	{ASY_DATE, 		0x80000052},
	{ASY_DATF, 		0x00000D01},
	{ASY_DATG, 		0x80000053},
	{ASY_DATH, 		0x00000000},
	{ASY_CMDSET, 	0x00000001},
	{ASY_CMDSET, 	0x00000000},
	{ASY_DATA, 		0x80000058},
	{ASY_DATB, 		0x00000000},
	{ASY_DATC, 		0x8000005A},
	{ASY_DATD, 		0x00000E01},
	{ASY_CMDSET, 	0x00000009},
	{ASY_CMDSET, 	0x00000008},
	{ASY_DATA, 		0x80000011},
	{ASY_DATB, 		0x00000812},
	{ASY_DATC, 		0x80000012},
	{ASY_DATD, 		0x00000003},
	{ASY_DATE, 		0x80000013},
	{ASY_DATF, 		0x00000909},
	{ASY_DATG, 		0x80000010},
	{ASY_DATH, 		0x00000040},
	{ASY_CMDSET, 	0x00000001},
	{ASY_CMDSET, 	0x00000000},
	{0,				40		},
	{ASY_DATA, 		0x80000010},
	{ASY_DATB, 		0x00000340},
	{ASY_CMDSET, 	0x00000005},
	{ASY_CMDSET, 	0x00000004},
	{0,				60		},
	{ASY_DATA, 		0x80000010},
	{ASY_DATB, 		0x00003340},
	{ASY_DATC, 		0x80000007},
	{ASY_DATD, 		0x00004007},
	{ASY_CMDSET, 	0x00000009},
	{ASY_CMDSET, 	0x00000008},
	{0,				1		},
	{ASY_DATA, 		0x80000007},
	{ASY_DATB, 		0x00004017},
	{ASY_DATC, 		0x8000005B},
	{ASY_DATD, 		0x00000000},
	{ASY_DATE, 		0x80000059},
	{ASY_DATF, 		0x00000011},
	{ASY_CMDSET, 	0x0000000D},
	{ASY_CMDSET, 	0x0000000C},
	{0,				20		},
	{ASY_DATA, 		0x80000059},
	{ASY_DATB, 		0x00000019},	// LTPS I/F control
	{ASY_CMDSET, 	0x00000005},	// Direct cmd transfer enable
	{ASY_CMDSET, 	0x00000004},	// Direct cmd transfer disable
	{0,				20		},
	{ASY_DATA, 		0x80000059},	// Index setting of SUB LCDD
	{ASY_DATB, 		0x00000079},	// LTPS I/F control
	{ASY_CMDSET, 	0x00000005},	// Direct cmd transfer enable
	{ASY_CMDSET, 	0x00000004},	// Direct cmd transfer disable
	{0,				20		},
	{ASY_DATA, 		0x80000059},	// Index setting of SUB LCDD
	{ASY_DATB, 		0x000003FD},	// LTPS I/F control
	{ASY_CMDSET, 	0x00000005},	// Direct cmd transfer enable
	{ASY_CMDSET, 	0x00000004},	// Direct cmd transfer disable
	{0,				20		}
};

static void tc358721xbg_sec_start(void)
{
	int i = 0;
	int count = sizeof(s_seq_sec_start) / 
		sizeof(s_reg_val_pair_t);
	
	for(i=0; i<count; i++)
		s_reg_val_pair_op(s_seq_sec_start[i].reg, 
			s_seq_sec_start[i].val);

	mddi_tc358721xbg_state_transition(TC358721_STATE_PRIM_SEC_READY,
				      TC358721_STATE_SEC_NORMAL_MODE);
}

static s_reg_val_pair_t
s_seq_sec_backlight_on[]={
	{TIMER0CTRL, 	0x00000060},
	{TIMER0LOAD, 	0x00001388},
	{PWM0OFF, 		0x00000001},
	{TIMER1CTRL, 	0x00000060},
	{TIMER1LOAD, 	0x00001388},
	{PWM1OFF, 		0x00001387},
	{TIMER0CTRL, 	0x000000E0},
	{TIMER1CTRL, 	0x000000E0},
	{PWMCR, 		0x00000003}
};

static void tc358721xbg_sec_backlight_on(void)
{
	int i = 0;
	int count = sizeof(s_seq_sec_backlight_on) / 
		sizeof(s_reg_val_pair_t);
	
	for( i=0; i<count; i++)
		s_reg_val_pair_op(s_seq_sec_backlight_on[i].reg, 
			s_seq_sec_backlight_on[i].val);
}

static void mddi_tc358721xbg_prim_init(void)
{

	switch (tc358721xbg_state) {
	case TC358721_STATE_PRIM_SEC_READY:
		break;
	case TC358721_STATE_OFF:
		tc358721xbg_state = TC358721_STATE_PRIM_SEC_STANDBY;
		tc358721xbg_common_initial_setup();
		break;
	case TC358721_STATE_PRIM_SEC_STANDBY:
		tc358721xbg_common_initial_setup();
		break;
	case TC358721_STATE_SEC_NORMAL_MODE:
		tc358721xbg_sec_cont_update_stop();
		tc358721xbg_sec_sleep_in();
		tc358721xbg_sec_sleep_out();
		tc358721xbg_sec_lcd_off();
		tc358721xbg_common_initial_setup();
		break;
	default:
		MDDI_MSG_ERR("mddi_tc358721xbg_prim_init from state %d\n",
			     tc358721xbg_state);
	}

	tc358721xbg_prim_start();

#ifdef TEST_WR_IMAGE
	// write images for test
	s_image_refresh_work(0);
#endif

	mddi_host_write_pix_attr_reg(0x00C3);
}

static void mddi_tc358721xbg_sec_init(void)
{

	switch (tc358721xbg_state) {
	case TC358721_STATE_PRIM_SEC_READY:
		break;
	case TC358721_STATE_PRIM_SEC_STANDBY:
		tc358721xbg_common_initial_setup();
		break;
	case TC358721_STATE_PRIM_NORMAL_MODE:
		tc358721xbg_prim_lcd_off();
		tc358721xbg_common_initial_setup();
		break;
	default:
		MDDI_MSG_ERR("mddi_tc358721xbg_sec_init from state %d\n",
			     tc358721xbg_state);
	}

	tc358721xbg_sec_start();
	tc358721xbg_sec_backlight_on();
	tc358721xbg_sec_cont_update_start();
	mddi_host_write_pix_attr_reg(0x0400);
}

static void mddi_tc358721xbg_lcd_powerdown(void)
{
	switch (tc358721xbg_state) {
	case TC358721_STATE_PRIM_SEC_READY:
		mddi_tc358721xbg_prim_init();
		mddi_tc358721xbg_lcd_powerdown();
		return;
	case TC358721_STATE_PRIM_SEC_STANDBY:
		break;
	case TC358721_STATE_PRIM_NORMAL_MODE:
		tc358721xbg_prim_lcd_off();
		break;
	case TC358721_STATE_SEC_NORMAL_MODE:
		tc358721xbg_sec_cont_update_stop();
		tc358721xbg_sec_sleep_in();
		tc358721xbg_sec_sleep_out();
		tc358721xbg_sec_lcd_off();
		break;
	default:
		MDDI_MSG_ERR("mddi_tc358721xbg_lcd_powerdown from state %d\n",
			     tc358721xbg_state);
	}
}

static void mddi_tc358721xbg_lcd_vsync_detected(boolean detected)
{
	// static timetick_type start_time = 0;
	static struct timeval start_time;
	static boolean first_time = TRUE;
	// uint32 mdp_cnt_val = 0;
	// timetick_type elapsed_us;
	struct timeval now;
	uint32 elapsed_us;
	uint32 num_vsyncs;

	if ((detected) || (mddi_tc358721xbg_vsync_attempts > 5)) {
		if ((detected) && (mddi_tc358721xbg_monitor_refresh_value)) {
			// if (start_time != 0)
			if (!first_time) {
				// elapsed_us = timetick_get_elapsed(start_time, T_USEC);
				jiffies_to_timeval(jiffies, &now);
				elapsed_us =
				    (now.tv_sec - start_time.tv_sec) * 1000000 +
				    now.tv_usec - start_time.tv_usec;
				/* LCD is configured for a refresh every * usecs, so to
				 * determine the number of vsyncs that have occurred
				 * since the last measurement add half that to the
				 * time difference and divide by the refresh rate. */
				num_vsyncs = (elapsed_us +
					      (mddi_tc358721xbg_usecs_per_refresh >>
					       1)) /
				    mddi_tc358721xbg_usecs_per_refresh;
				/* LCD is configured for * hsyncs (rows) per refresh cycle.
				 * Calculate new rows_per_second value based upon these
				 * new measurements. MDP can update with this new value. */
				mddi_tc358721xbg_rows_per_second =
				    (mddi_tc358721xbg_rows_per_refresh * 1000 *
				     num_vsyncs) / (elapsed_us / 1000);
			}
			// start_time = timetick_get();
			first_time = FALSE;
			jiffies_to_timeval(jiffies, &start_time);
			if (mddi_tc358721xbg_report_refresh_measurements) {
				(void)mddi_queue_register_read_int(VPOS,
								   &mddi_tc358721xbg_curr_vpos);
				// mdp_cnt_val = MDP_LINE_COUNT;
			}
		}
		/* if detected = TRUE, client initiated wakeup was detected */
		if (mddi_tc358721xbg_vsync_handler != NULL) {
			(*mddi_tc358721xbg_vsync_handler)
			    (mddi_tc358721xbg_vsync_handler_arg);
			mddi_tc358721xbg_vsync_handler = NULL;
		}
		mddi_vsync_detect_enabled = FALSE;
		mddi_tc358721xbg_vsync_attempts = 0;
		/* need to disable the interrupt wakeup */
		if (!mddi_queue_register_write_int(INTMSK, 0x0001)) {
			MDDI_MSG_ERR("Vsync interrupt disable failed!\n");
		}
		if (!detected) {
			/* give up after 5 failed attempts but show error */
			MDDI_MSG_NOTICE("Vsync detection failed!\n");
		} else if ((mddi_tc358721xbg_monitor_refresh_value) &&
			   (mddi_tc358721xbg_report_refresh_measurements)) {
			MDDI_MSG_NOTICE("  Last Line Counter=%d!\n",
					mddi_tc358721xbg_curr_vpos);
			// MDDI_MSG_NOTICE("  MDP Line Counter=%d!\n",mdp_cnt_val);
			MDDI_MSG_NOTICE("  Lines Per Second=%d!\n",
					mddi_tc358721xbg_rows_per_second);
		}
		/* clear the interrupt */
		if (!mddi_queue_register_write_int(INTFLG, 0x0001)) {
			MDDI_MSG_ERR("Vsync interrupt clear failed!\n");
		}
	} else {
		/* if detected = FALSE, we woke up from hibernation, but did not
		 * detect client initiated wakeup.
		 */
		mddi_tc358721xbg_vsync_attempts++;
	}
}


static int panel_detect ;
static void mddi_tc358721xbg_panel_detect(void)
{
	mddi_host_type host_idx = MDDI_HOST_PRIM;
	if (!panel_detect) {
		/* Toshiba display requires larger drive_lo value */
		mddi_host_reg_out(DRIVE_LO, 0x0050);

		panel_sel = &panel_lms350df01;
		panel_detect = 1;
	}

}

// lcd on
static int mddi_tc358721xbg_lcd_on(struct platform_device *pdev)
{
	struct msm_fb_data_type *mfd;

	mfd = (struct msm_fb_data_type *)platform_get_drvdata(pdev);

	if (!mfd)
		return -ENODEV;

	if (mfd->key != MFD_KEY)
		return -EINVAL;

	mddi_tc358721xbg_panel_detect();

	if (mfd->panel.id == TC358721XBG_VGA_PRIM) {
		mddi_tc358721xbg_prim_init();
	} else {
		mddi_tc358721xbg_sec_init();
	}

	return 0;
}

// lcd off
static int mddi_tc358721xbg_lcd_off(struct platform_device *pdev)
{
	mddi_tc358721xbg_panel_detect();
	mddi_tc358721xbg_lcd_powerdown();

#ifdef TEST_WR_IMAGE
	del_timer(&image_timer);
#endif
	
	return 0;
}

// set backlight
static void mddi_tc358721xbg_lcd_set_backlight(struct msm_fb_data_type *mfd)
{
	int32 level;

	if (mddi_tc358721xbg_pdata && mddi_tc358721xbg_pdata->backlight_level) {
		level = mddi_tc358721xbg_pdata->backlight_level(mfd->bl_level);

		if (level < 0)
			return;

		// Comment the line followed if let led on always.
		//write_client_reg(PWM0OFF, level, TRUE);
		//write_client_reg(PWM0OFF, 4000, TRUE);
	}
}

// set handler
static void mddi_tc358721xbg_vsync_set_handler(msm_fb_vsync_handler_type handler,	/* ISR to be executed */
					   void *arg)
{
	boolean error = FALSE;
	unsigned long flags;

	/* Disable interrupts */
	spin_lock_irqsave(&mddi_host_spin_lock, flags);
	// INTLOCK();

	if (mddi_tc358721xbg_vsync_handler != NULL) {
		error = TRUE;
	} else {
		/* Register the handler for this particular GROUP interrupt source */
		mddi_tc358721xbg_vsync_handler = handler;
		mddi_tc358721xbg_vsync_handler_arg = arg;
	}

	/* Restore interrupts */
	spin_unlock_irqrestore(&mddi_host_spin_lock, flags);
	// MDDI_INTFREE();
	if (error) {
		MDDI_MSG_ERR("MDDI: Previous Vsync handler never called\n");
	} else {
		/* Enable the vsync wakeup */
		mddi_queue_register_write(INTMSK, 0x0000, FALSE, 0);

		mddi_tc358721xbg_vsync_attempts = 1;
		mddi_vsync_detect_enabled = TRUE;
	}
}				/* mddi_toshiba_vsync_set_handler */

static int __init mddi_tc358721xbg_lcd_probe(struct platform_device *pdev)
{
	if (pdev->id == 0) {
		mddi_tc358721xbg_pdata = pdev->dev.platform_data;
		return 0;
	}

	msm_fb_add_device(pdev);

	return 0;
}


static struct platform_driver this_driver = {
	.probe  = mddi_tc358721xbg_lcd_probe,
	.driver = {
		.name   = "mddi_tc21xbg_vga",
	},
};

static struct msm_fb_panel_data tc358721xbg_panel_data0 = {
	.on 		= mddi_tc358721xbg_lcd_on,
	.off 		= mddi_tc358721xbg_lcd_off,
	.set_backlight 	= mddi_tc358721xbg_lcd_set_backlight,
	.set_vsync_notifier = mddi_tc358721xbg_vsync_set_handler,
};

static struct platform_device this_device_0 = {
	.name   = "mddi_tc21xbg_vga",
	.id	= TC358721XBG_VGA_PRIM,
	.dev	= {
		.platform_data = &tc358721xbg_panel_data0,
	}
};

#if defined(CONFIG_FB_MSM_MDDI_TC358721XBG_VGA_QCIF) || \
	defined(CONFIG_FB_MSM_MDDI_AUTO_DETECT)
static struct msm_fb_panel_data tc358721xbg_panel_data1 = {
	.on 		= mddi_tc358721xbg_lcd_on,
	.off 		= mddi_tc358721xbg_lcd_off,
};

static struct platform_device this_device_1 = {
	.name   = "mddi_tc21xbg_vga",
	.id	= TC358721XBG_VGA_SECD,
	.dev	= {
		.platform_data = &tc358721xbg_panel_data1,
	}
};
#endif

#ifdef SPI_SW_SIMULATE
static int s_gpio_request(void)
{
	//int err;

	gpio_tlmm_config(GPIO_CFG
		(17, 0, GPIO_OUTPUT,
		GPIO_PULL_UP, GPIO_16MA),
		GPIO_ENABLE);
	gpio_tlmm_config(GPIO_CFG
		(26, 0, GPIO_OUTPUT,
		GPIO_PULL_UP, GPIO_16MA),
		GPIO_ENABLE);
	gpio_tlmm_config(GPIO_CFG
		(88, 0, GPIO_OUTPUT,
		GPIO_PULL_UP, GPIO_16MA),
		GPIO_ENABLE);
	/*
	err = gpio_request(17, "gpio_lms_ncs");
	if (err) {
		printk(KERN_ERR "gpio_request failed for 17\n");
		return -1;
	}
	err = gpio_request(26, "gpio_lms_scl");
	if (err) {
		printk(KERN_ERR "gpio_request failed for 26\n");
		return -1;
	}
	err = gpio_request(88, "gpio_lms_sdi");
	if (err) {
		printk(KERN_ERR "gpio_request failed for 88\n");
		return -1;
	}
	*/
	return 0;
}
#endif

static int __init mddi_tc358721xbg_lcd_init(void)
{
	int ret;
	struct msm_panel_info *pinfo;

#ifdef SPI_SW_SIMULATE
	s_gpio_request();
#endif

	// config gpio102/94
	gpio_tlmm_config(GPIO_CFG
		(102, 0, GPIO_OUTPUT,
		GPIO_NO_PULL, GPIO_10MA),
		GPIO_ENABLE);
	gpio_tlmm_config(GPIO_CFG
		(94, 0, GPIO_OUTPUT,
		GPIO_NO_PULL, GPIO_2MA),
		GPIO_ENABLE);
	
#if 0
	gpio_tlmm_config(GPIO_CFG
		(17, 0, GPIO_OUTPUT,
		GPIO_PULL_UP, GPIO_10MA),
		GPIO_ENABLE);
#endif
	
	// LCD_EN, LCD_VDD3(3.3v) on
	gpio_direction_output(94,1);

	// LCD_RESET for tc358721xbg
	mddi_wait(100);
	gpio_direction_output(102,1);
	mddi_wait(1);
	gpio_direction_output(102,0);
	mddi_wait(50);
	gpio_direction_output(102,1);
	mddi_wait(10);

	// LCD_RESET for lms530df01
#if 0
	gpio_direction_output(17,0);
	//mddi_wait(1);
	udelay(100);
	gpio_direction_output(17,1);
	//mddi_wait(10);
#endif
	
	
#ifdef CONFIG_FB_MSM_MDDI_AUTO_DETECT
{
	u32 id;

	id = mddi_get_client_id();
	if ((id >> 16) != 0xD263)
		return 0;
}
#endif

	ret = platform_driver_register(&this_driver);
	if (!ret) {
		pinfo = &tc358721xbg_panel_data0.panel_info;
		pinfo->xres = 320;
		pinfo->yres = 480;
		pinfo->type = MDDI_PANEL;
		pinfo->pdest = DISPLAY_1;
		pinfo->mddi.vdopkt = MDDI_DEFAULT_PRIM_PIX_ATTR;
		pinfo->wait_cycle = 0;
		pinfo->bpp = 18;
		pinfo->lcd.vsync_enable = TRUE;
		/*pinfo->lcd.refx100 =
		    (mddi_tc358721xbg_rows_per_second * 100) /
		    mddi_tc358721xbg_rows_per_refresh;*/
		pinfo->lcd.refx100 =7144;
		pinfo->lcd.v_back_porch = 6;
		pinfo->lcd.v_front_porch = 0;
		pinfo->lcd.v_pulse_width = 0;
		pinfo->lcd.hw_vsync_mode = FALSE;
		pinfo->lcd.vsync_notifier_period = (1 * HZ);
		pinfo->bl_max = 4;
		pinfo->bl_min = 1;
		pinfo->clk_rate = 122880000;
		pinfo->clk_min = 120000000;
		pinfo->clk_max = 125000000;
//		pinfo->clk_rate = 122880000;
/*		pinfo->clk_rate = 192000000;
		pinfo->clk_min =  190000000;
		pinfo->clk_max =  200000000;
*/		pinfo->fb_num = 2;

		ret = platform_device_register(&this_device_0);
		if (ret)
			platform_driver_unregister(&this_driver);

#if defined(CONFIG_FB_MSM_MDDI_TC358721XBG_VGA_QCIF) || \
	defined(CONFIG_FB_MSM_MDDI_AUTO_DETECT)
		pinfo = &tc358721xbg_panel_data1.panel_info;
		pinfo->xres = 176;
		pinfo->yres = 220;
		pinfo->type = MDDI_PANEL;
		pinfo->pdest = DISPLAY_2;
		pinfo->wait_cycle = 0;
		pinfo->mddi.vdopkt = 0x400;
		pinfo->bpp = 18;
		//pinfo->clk_rate = 122880000;
		pinfo->clk_rate = 192000000;
		pinfo->clk_min =  190000000;
		pinfo->clk_max =  200000000;
		pinfo->fb_num = 2;

		ret = platform_device_register(&this_device_1);
		if (ret) {
			platform_device_unregister(&this_device_0);
			platform_driver_unregister(&this_driver);
		}
#endif
	}

	if (!ret)
		mddi_lcd.vsync_detected = mddi_tc358721xbg_lcd_vsync_detected;

	return ret;
}

module_init(mddi_tc358721xbg_lcd_init);

