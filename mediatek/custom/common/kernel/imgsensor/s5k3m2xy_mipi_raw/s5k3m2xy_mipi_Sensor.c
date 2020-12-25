/*****************************************************************************
 *
 * Filename:
 * ---------
 *   S5k3M2mipi_Sensor.c
 *
 * Project:
 * --------
 *   ALPS
 *
 * Description:
 * ------------
 *   Source code of Sensor driver
 *
 *
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/

#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
#include <asm/system.h>
#include <linux/xlog.h>

#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"

#include "s5k3m2xy_mipi_Sensor.h"

#define PFX "S5k3M2XY_camera_sensor"
//#define LOG_WRN(format, args...) xlog_printk(ANDROID_LOG_WARN ,PFX, "[%S] " format, __FUNCTION__, ##args)
//#defineLOG_INF(format, args...) xlog_printk(ANDROID_LOG_INFO ,PFX, "[%s] " format, __FUNCTION__, ##args)
//#define LOG_DBG(format, args...) xlog_printk(ANDROID_LOG_DEBUG ,PFX, "[%S] " format, __FUNCTION__, ##args)
#define LOG_INF(format, args...)    xlog_printk(ANDROID_LOG_INFO   , PFX, "[%s] " format, __FUNCTION__, ##args)
#define LOGE(format, args...)   xlog_printk(ANDROID_LOG_ERROR, PFX, "[%s] " format, __FUNCTION__, ##args)


static DEFINE_SPINLOCK(imgsensor_drv_lock);


static imgsensor_info_struct imgsensor_info = { 
    .sensor_id = S5K3M2XY_SENSOR_ID,
    
	.checksum_value = 0xc3dd012b,
    
    .pre = {
		.pclk = 400000000,				//record different mode's pclk
		.linelength = 7408,				//record different mode's linelength
		.framelength =1780, //3168,			//record different mode's framelength
        .startx = 0,                    //record different mode's startx of grabwindow
        .starty = 0,                    //record different mode's starty of grabwindow
		.grabwindow_width = 2104,		//record different mode's width of grabwindow
		.grabwindow_height = 1560,		//record different mode's height of grabwindow
        /*   following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario   */
		.mipi_data_lp2hs_settle_dc = 14,//unit , ns
        /*   following for GetDefaultFramerateByScenario()  */
        .max_framerate = 300,   
    },
	.cap = {
		.pclk = 400000000,
		.linelength =7408,
		.framelength = 3188,
		.startx = 0,   
		.starty = 0,
		.grabwindow_width = 4192,//5334, 4192
		.grabwindow_height = 3104,  //3104
		.mipi_data_lp2hs_settle_dc = 14,//unit , ns
		.max_framerate = 170,
		},
    .normal_video = {
		.pclk = 400000000,
		.linelength = 7408,
		.framelength = 1800,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 1920,//5334,
		.grabwindow_height = 1080,
		.mipi_data_lp2hs_settle_dc = 14,//unit , ns
        .max_framerate = 300,
    },
	.margin = 4,
    .min_shutter = 2,
    .max_frame_length = 0xffff,
    .ae_shut_delay_frame = 0,
    .ae_sensor_gain_delay_frame = 0,
    .ae_ispGain_delay_frame = 2,

    .cap_delay_frame = 2, 
    .pre_delay_frame = 2, 
    .video_delay_frame = 2,
    
    .isp_driving_current = ISP_DRIVING_6MA,
    .sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
    .sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_Gb,
    .mclk = 24,
    .mipi_lane_num = SENSOR_MIPI_4_LANE,
    .i2c_addr_table = {0x5A, 0xff, 0xff, 0xff},
};


static imgsensor_struct imgsensor = {
    .mirror = IMAGE_NORMAL,             //mirrorflip information
    .sensor_mode = IMGSENSOR_MODE_INIT, //IMGSENSOR_MODE enum value,record current sensor mode,such as: INIT, Preview, Capture, Video,High Speed Video, Slim Video
	.shutter = 0x200,					//current shutter
    .gain = 0x100,                      //current gain
    .dummy_pixel = 0,                   //current dummypixel
    .dummy_line = 0,                    //current dummyline
    .current_fps = 0,  //full size current fps : 24fps for PIP, 30fps for Normal or ZSD
    .autoflicker_en = KAL_FALSE,  //auto flicker enable: KAL_FALSE for disable auto flicker, KAL_TRUE for enable auto flicker
	.test_pattern = KAL_FALSE,		//test pattern mode or not. KAL_FALSE for in test pattern mode, KAL_TRUE for normal output
    .current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW,//current scenario id
    .i2c_write_id = 0x5A,
};


/* Sensor output window information */


static kal_uint16 read_cmos_sensor(kal_uint32 addr)
{
    kal_uint16 get_byte=0;
    char pusendcmd[2] = {(char)(addr >> 8) , (char)(addr & 0xFF) };
    iReadRegI2C(pusendcmd , 2, (u8*)&get_byte, 2, imgsensor.i2c_write_id);
    return ((get_byte<<8)&0xff00)|((get_byte>>8)&0x00ff);
}


static void write_cmos_sensor(kal_uint16 addr, kal_uint16 para)
{
    char pusendcmd[4] = {(char)(addr >> 8) , (char)(addr & 0xFF) ,(char)(para >> 8),(char)(para & 0xFF)};
    iWriteRegI2C(pusendcmd , 4, imgsensor.i2c_write_id);
}

static kal_uint16 read_cmos_sensor_8(kal_uint16 addr)
{
    kal_uint16 get_byte=0;
    char pusendcmd[2] = {(char)(addr >> 8) , (char)(addr & 0xFF) };
    iReadRegI2C(pusendcmd , 2, (u8*)&get_byte,1,imgsensor.i2c_write_id);
    return get_byte;
}

static void write_cmos_sensor_8(kal_uint16 addr, kal_uint8 para)
{
    char pusendcmd[4] = {(char)(addr >> 8) , (char)(addr & 0xFF) ,(char)(para & 0xFF)};
    iWriteRegI2C(pusendcmd , 3, imgsensor.i2c_write_id);
}


static void set_dummy()
{
	#if 1
	LOG_INF("dummyline = %d, dummypixels = %d ", imgsensor.dummy_line, imgsensor.dummy_pixel);
    write_cmos_sensor_8(0x0104, 0x01); 
    write_cmos_sensor(0x0340, imgsensor.frame_length);
    write_cmos_sensor(0x0342, imgsensor.line_length);
    write_cmos_sensor_8(0x0104, 0x00); 
	#endif
  
}   /*  set_dummy  */


static void set_max_framerate(UINT16 framerate,kal_bool min_framelength_en)
{
	kal_int16 dummy_line;
	kal_uint32 frame_length = imgsensor.frame_length;
	//unsigned long flags;

	LOG_INF("framerate = %d, min framelength should enable? \n", framerate,min_framelength_en);
   
	frame_length = imgsensor.pclk / framerate * 10 / imgsensor.line_length;
	spin_lock(&imgsensor_drv_lock);
	imgsensor.frame_length = (frame_length > imgsensor.min_frame_length) ? frame_length : imgsensor.min_frame_length; 
	imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;
	//dummy_line = frame_length - imgsensor.min_frame_length;
	//if (dummy_line < 0)
		//imgsensor.dummy_line = 0;
	//else
		//imgsensor.dummy_line = dummy_line;
	//imgsensor.frame_length = frame_length + imgsensor.dummy_line;
	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
	{
		imgsensor.frame_length = imgsensor_info.max_frame_length;
		imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;
	}
	if (min_framelength_en)
		imgsensor.min_frame_length = imgsensor.frame_length;
	spin_unlock(&imgsensor_drv_lock);
	set_dummy();
}	/*	set_max_framerate  */


static void write_shutter(kal_uint16 shutter)
{
	#if 1
    kal_uint16 realtime_fps = 0;
    kal_uint32 frame_length = 0;
    spin_lock(&imgsensor_drv_lock);
    if (shutter > imgsensor.min_frame_length - imgsensor_info.margin)       
        imgsensor.frame_length = shutter + imgsensor_info.margin;
    else
        imgsensor.frame_length = imgsensor.min_frame_length;
    if (imgsensor.frame_length > imgsensor_info.max_frame_length)
        imgsensor.frame_length = imgsensor_info.max_frame_length;
    spin_unlock(&imgsensor_drv_lock);
	shutter = (shutter < imgsensor_info.min_shutter) ? imgsensor_info.min_shutter : shutter;
	shutter = (shutter > (imgsensor_info.max_frame_length - imgsensor_info.margin)) ? (imgsensor_info.max_frame_length - imgsensor_info.margin) : shutter;
    
    if (imgsensor.autoflicker_en) { 
		realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length;
		if(realtime_fps >= 297 && realtime_fps <= 305)
			set_max_framerate(296,0);
		else if(realtime_fps >= 147 && realtime_fps <= 150)
			set_max_framerate(146,0);	
    } else {
        // Extend frame length
        write_cmos_sensor_8(0x0104,0x01);
        write_cmos_sensor(0x0340, imgsensor.frame_length);
        write_cmos_sensor_8(0x0104,0x00);
    }

    // Update Shutter
    write_cmos_sensor_8(0x0104,0x01);
	write_cmos_sensor(0x0340, imgsensor.frame_length);
    write_cmos_sensor(0x0202, shutter);
    write_cmos_sensor_8(0x0104,0x00);
	LOG_INF("Exit! shutter =%d, framelength =%d\n", shutter,imgsensor.frame_length);
    #endif
    //LOG_INF("frame_length = %d ", frame_length);
    
}   /*  write_shutter  */



/*************************************************************************
* FUNCTION
*   set_shutter
*
* DESCRIPTION
*   This function set e-shutter of sensor to change exposure time.
*
* PARAMETERS
*   iShutter : exposured lines
*
* RETURNS
*   None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static void set_shutter(kal_uint16 shutter)
{
    unsigned long flags;
    spin_lock_irqsave(&imgsensor_drv_lock, flags);
    imgsensor.shutter = shutter;
    spin_unlock_irqrestore(&imgsensor_drv_lock, flags);
    
    write_shutter(shutter);
}   /*  set_shutter */



static kal_uint16 gain2reg(const kal_uint16 gain)
{
    kal_uint16 reg_gain = 0x0;
    
    reg_gain = gain/2;
    return (kal_uint16)reg_gain;
}

/*************************************************************************
* FUNCTION
*   set_gain
*
* DESCRIPTION
*   This function is to set global gain to sensor.
*
* PARAMETERS
*   iGain : sensor global gain(base: 0x40)
*
* RETURNS
*   the actually gain set to sensor.
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint16 set_gain(kal_uint16 gain)
{   
    #if 1
    kal_uint16 reg_gain;

    /* 0x350A[0:1], 0x350B[0:7] AGC real gain */
    /* [0:3] = N meams N /16 X  */
    /* [4:9] = M meams M X       */
    /* Total gain = M + N /16 X   */

    //
    if (gain < BASEGAIN || gain > 32 * BASEGAIN) {
        LOG_INF("Error gain setting");

        if (gain < BASEGAIN)
            gain = BASEGAIN;
        else if (gain > 32 * BASEGAIN)
            gain = 32 * BASEGAIN;        
    }
 
    reg_gain = gain2reg(gain);
    spin_lock(&imgsensor_drv_lock);
    imgsensor.gain = reg_gain; 
    spin_unlock(&imgsensor_drv_lock);
    LOG_INF("gain = %d , reg_gain = 0x%x ", gain, reg_gain);

    write_cmos_sensor_8(0x0104, 0x01);
    write_cmos_sensor_8(0x0204,(reg_gain>>8));
    write_cmos_sensor_8(0x0205,(reg_gain&0xff));
    write_cmos_sensor_8(0x0104, 0x00);
    
    return gain;
	#endif
}   /*  set_gain  */




static void set_mirror_flip(kal_uint8 image_mirror)
{
	#if 1
	LOG_INF("image_mirror = %d", image_mirror);

    /********************************************************
       *
       *   0x3820[2] ISP Vertical flip
       *   0x3820[1] Sensor Vertical flip
       *
       *   0x3821[2] ISP Horizontal mirror
       *   0x3821[1] Sensor Horizontal mirror
       *
       *   ISP and Sensor flip or mirror register bit should be the same!!
       *
       ********************************************************/
    image_mirror = IMAGE_HV_MIRROR;
	spin_lock(&imgsensor_drv_lock);
    imgsensor.mirror= image_mirror; 
    spin_unlock(&imgsensor_drv_lock);
    switch (image_mirror) {

        case IMAGE_NORMAL:
            write_cmos_sensor_8(0x0101,0x00);   // Gr
            break;
        case IMAGE_H_MIRROR:
            write_cmos_sensor_8(0x0101,0x01);
            break;
        case IMAGE_V_MIRROR:
            write_cmos_sensor_8(0x0101,0x02);
            break;
        case IMAGE_HV_MIRROR:
            write_cmos_sensor_8(0x0101,0x03);//Gb
            break;
        default:
			LOG_INF("Error image_mirror setting\n");
    }
	#endif
}

/*************************************************************************
* FUNCTION
*   night_mode
*
* DESCRIPTION
*   This function night mode of sensor.
*
* PARAMETERS
*   bEnable: KAL_TRUE -> enable night mode, otherwise, disable night mode
*
* RETURNS
*   None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static void sensor_init(void)
{
	  LOG_INF("E\n");  
	
	write_cmos_sensor(0x6028, 0x4000);
	write_cmos_sensor(0x602A, 0x6214);
	write_cmos_sensor(0x6F12, 0x7971);
	write_cmos_sensor(0x602A, 0x6218);
	write_cmos_sensor(0x6F12, 0x0100);	  // clock on
	
	
	// Start T&P part
	// DO NOT DELETE T&P SECTION COMMENTS! They are required to debug T&P related issues.
	// https://ssemi-fsrv0331:8443/svn/3M2_FW_REP/branches/3M2_EVT1_TnP/3M2X_OutputFiles/3M2X_EVT0_Release
	// SVN Rev: 608-608
	// ROM Rev: 3M2X_EVT0_Release
	// Signature:
	// md5 a53ff50c4f018a9ee4e69556691f4167 .btp
	// md5 d23cf46e101fb6f136767d60b0f8547c .htp
	// md5 a21dc0b5cfdfb98f6116671879379abf .RegsMap.h
	// md5 54da9462c10b9eeebfdb34be89ed354f .RegsMap.bin
	//
	
	 write_cmos_sensor(0x6028,	  0x2000);	   // Page pointer
	 write_cmos_sensor(0x602A,	  0x448C);	   // Start address
	 write_cmos_sensor(0x6F12,	  0x0000);	  
	 write_cmos_sensor(0x6F12,	  0x0000);	  
	 write_cmos_sensor(0x6F12,	  0x0448);	  
	 write_cmos_sensor(0x6F12,	  0x0349);	  
	 write_cmos_sensor(0x6F12,	  0x0160);	  
	 write_cmos_sensor(0x6F12,	  0xC26A);	  
	 write_cmos_sensor(0x6F12,	  0x511A);	  
	 write_cmos_sensor(0x6F12,	  0x8180);	  
	 write_cmos_sensor(0x6F12,	  0x00F0);	  
	 write_cmos_sensor(0x6F12,	  0x2CB8);	  
	 write_cmos_sensor(0x6F12,	  0x2000);	  
	 write_cmos_sensor(0x6F12,	  0x4538);	  
	 write_cmos_sensor(0x6F12,	  0x2000);	  
	 write_cmos_sensor(0x6F12,	  0x1FA0);	  
	 write_cmos_sensor(0x6F12,	  0x0000);	  
	 write_cmos_sensor(0x6F12,	  0x0000);	  
	 write_cmos_sensor(0x6F12,	  0x0000);	  
	 write_cmos_sensor(0x6F12,	  0x0000);	  
	 write_cmos_sensor(0x6F12,	  0x2DE9);	  
	 write_cmos_sensor(0x6F12,	  0xF041);	  
	 write_cmos_sensor(0x6F12,	  0x0546);	  
	 write_cmos_sensor(0x6F12,	  0x1348);	  
	 write_cmos_sensor(0x6F12,	  0x134E);	  
	 write_cmos_sensor(0x6F12,	  0x018A);	  
	 write_cmos_sensor(0x6F12,	  0x4069);	  
	 write_cmos_sensor(0x6F12,	  0x06F1);	  
	 write_cmos_sensor(0x6F12,	  0x2007);	  
	 write_cmos_sensor(0x6F12,	  0x4143);	  
	 write_cmos_sensor(0x6F12,	  0x4FEA);	  
	 write_cmos_sensor(0x6F12,	  0x1138);	  
	 write_cmos_sensor(0x6F12,	  0x0024);	  
	 write_cmos_sensor(0x6F12,	  0x06EB);	  
	 write_cmos_sensor(0x6F12,	  0xC402);	  
	 write_cmos_sensor(0x6F12,	  0x0423);	  
	 write_cmos_sensor(0x6F12,	  0x3946);	  
	 write_cmos_sensor(0x6F12,	  0x4046);	  
	 write_cmos_sensor(0x6F12,	  0x00F0);	  
	 write_cmos_sensor(0x6F12,	  0x1EF8);	  
	 write_cmos_sensor(0x6F12,	  0x25F8);	  
	 write_cmos_sensor(0x6F12,	  0x1400);	  
	 write_cmos_sensor(0x6F12,	  0x641C);	  
	 write_cmos_sensor(0x6F12,	  0x042C);	  
	 write_cmos_sensor(0x6F12,	  0xF3DB);	  
	 write_cmos_sensor(0x6F12,	  0x0A48);	  
	 write_cmos_sensor(0x6F12,	  0x2988);	  
	 write_cmos_sensor(0x6F12,	  0x0180);	  
	 write_cmos_sensor(0x6F12,	  0x6988);	  
	 write_cmos_sensor(0x6F12,	  0x4180);	  
	 write_cmos_sensor(0x6F12,	  0xA988);	  
	 write_cmos_sensor(0x6F12,	  0x8180);	  
	 write_cmos_sensor(0x6F12,	  0xE988);	  
	 write_cmos_sensor(0x6F12,	  0xC180);	  
	 write_cmos_sensor(0x6F12,	  0xBDE8);	  
	 write_cmos_sensor(0x6F12,	  0xF081);	  
	 write_cmos_sensor(0x6F12,	  0x0022);	  
	 write_cmos_sensor(0x6F12,	  0xAFF2);	  
	 write_cmos_sensor(0x6F12,	  0x4B01);	  
	 write_cmos_sensor(0x6F12,	  0x0448);	  
	 write_cmos_sensor(0x6F12,	  0x00F0);	  
	 write_cmos_sensor(0x6F12,	  0x0DB8);	  
	 write_cmos_sensor(0x6F12,	  0x2000);	  
	 write_cmos_sensor(0x6F12,	  0x34D0);	  
	 write_cmos_sensor(0x6F12,	  0x2000);	  
	 write_cmos_sensor(0x6F12,	  0x7900);	  
	 write_cmos_sensor(0x6F12,	  0x4000);	  
	 write_cmos_sensor(0x6F12,	  0xD22E);	  
	 write_cmos_sensor(0x6F12,	  0x0000);	  
	 write_cmos_sensor(0x6F12,	  0x2941);	  
	 write_cmos_sensor(0x6F12,	  0x40F2);	  
	 write_cmos_sensor(0x6F12,	  0xFD7C);	  
	 write_cmos_sensor(0x6F12,	  0xC0F2);	  
	 write_cmos_sensor(0x6F12,	  0x000C);	  
	 write_cmos_sensor(0x6F12,	  0x6047);	  
	 write_cmos_sensor(0x6F12,	  0x4DF2);	  
	 write_cmos_sensor(0x6F12,	  0x474C);	  
	 write_cmos_sensor(0x6F12,	  0xC0F2);	  
	 write_cmos_sensor(0x6F12,	  0x000C);	  
	 write_cmos_sensor(0x6F12,	  0x6047);	  
	 write_cmos_sensor(0x6F12,	  0x0000);	  
	 write_cmos_sensor(0x6F12,	  0x0000);	  
	 write_cmos_sensor(0x6F12,	  0x0000);	  
	 write_cmos_sensor(0x6F12,	  0x0000);	  
	 write_cmos_sensor(0x6F12,	  0x30D2);	  
	 write_cmos_sensor(0x6F12,	  0x029C);	  
	 write_cmos_sensor(0x6F12,	  0x0000);	  
	 write_cmos_sensor(0x6F12,	  0x0001);	  
	 write_cmos_sensor(0x602A,	  0x7900);	   // Alpha control, added on 150106
	 write_cmos_sensor(0x6F12, 0x4000);   //Gain_1	   
	 write_cmos_sensor(0x6F12, 0x3000);   //Gain_2	  
	 write_cmos_sensor(0x6F12, 0x2000);   //Gain_3	  
	 write_cmos_sensor(0x6F12, 0x1300);   //Gain_4	  
	 write_cmos_sensor(0x6F12, 0x4000);   //Gain_1	   
	 write_cmos_sensor(0x6F12, 0x3000);   //Gain_2	  
	 write_cmos_sensor(0x6F12, 0x2000);   //Gain_3	  
	 write_cmos_sensor(0x6F12, 0x0A00);   //Gain_4	  
	 write_cmos_sensor(0x6F12, 0x4000);   //Gain_1	   
	 write_cmos_sensor(0x6F12, 0x3000);   //Gain_2	  
	 write_cmos_sensor(0x6F12, 0x2000);   //Gain_3	  
	 write_cmos_sensor(0x6F12, 0x1300);   //Gain_4	  
	 write_cmos_sensor(0x6F12, 0x4000);   //Gain_1		 
	 write_cmos_sensor(0x6F12, 0x3000);   //Gain_2	  
	 write_cmos_sensor(0x6F12, 0x2000);   //Gain_3	  
	 write_cmos_sensor(0x6F12, 0x1300);   //Gain_4	  
	 write_cmos_sensor(0x6F12, 0x0400);   //Gain_1 AGx1 //1120 TEMP
	 write_cmos_sensor(0x6F12, 0x0600);   //Gain_2 AGx2
	 write_cmos_sensor(0x6F12, 0x0800);   //Gain_3 AGx4
	 write_cmos_sensor(0x6F12, 0x0A00);   //Gain_4 AGx8 //1120 TEMP
	
	 write_cmos_sensor(0x602A,	  0x43F0);	  
	 write_cmos_sensor(0x6F12,	  0x0128);	   // Ex_Regs_usMinDRClneqsizeX 	
	 write_cmos_sensor(0x6F12,	  0x00DC);	   // Ex_Regs_usMinDRClneqsizeY 	
	 write_cmos_sensor(0x6F12,	  0x5590);	   // Ex_Regs_usMinDRClneqsizePowXd2
	 write_cmos_sensor(0x6F12,	  0x3644);	   // Ex_Regs_usMinDRClneqsizePowYd2
	 write_cmos_sensor(0x602A,	  0x1B50);	  
	 write_cmos_sensor(0x6F12,	  0x0000);	  
	 write_cmos_sensor(0x602A,	  0x1B54);	  
	 write_cmos_sensor(0x6F12,	  0x0000);	  
	 write_cmos_sensor(0x602A,	  0x1B64);	  
	 write_cmos_sensor(0x6F12,	  0x0800);	  
	 write_cmos_sensor(0x602A,	  0x1926);	  
	 write_cmos_sensor(0x6F12,	  0x0011);	  
	 write_cmos_sensor(0x602A,	  0x14FA);	  
	 write_cmos_sensor_8(0x6F12,  0x0F);
	 write_cmos_sensor(0x602A,	  0x4473);	  
	 write_cmos_sensor_8(0x6F12,  0x02);
	// End T&P part
	
	//Global start
	 write_cmos_sensor(0x6028,	  0x4000);
	 write_cmos_sensor(0x602A,	0x3089);  //add moses
	 write_cmos_sensor_8(0x6F12,  0x01);  //add moses non-continuous mode
	 write_cmos_sensor_8(0x0B04,  0x01);
	 write_cmos_sensor(0x3B22,	  0x1110);
	 write_cmos_sensor(0xF42E,	  0x200C);
	 write_cmos_sensor(0xF49E,	  0x004C);
	 write_cmos_sensor(0xF4A6,	  0x00F0);
	 write_cmos_sensor(0x3AFA,	  0xFBB8);
	 write_cmos_sensor(0xF49C,	  0x0000);
	 write_cmos_sensor(0xF496,	  0x0000);
	 write_cmos_sensor(0xF476,	  0x0040);
	 write_cmos_sensor_8(0x3AAA,  0x02);
	 write_cmos_sensor(0x3AFE,	  0x07DF);
	 write_cmos_sensor(0xF47A,	  0x001B);
	 write_cmos_sensor(0xF462,	  0x0003);
	 write_cmos_sensor(0xF460,	  0x0020);
	 write_cmos_sensor(0x3B06,	  0x000E);
	 write_cmos_sensor(0x3AD0,	  0x0080);
	 write_cmos_sensor(0x3B02,	  0x0020);
	 write_cmos_sensor(0xF468,	  0x0001);
	 write_cmos_sensor(0xF494,	  0x000E);
	 write_cmos_sensor(0xF40C,	  0x2180);
	 write_cmos_sensor(0x3870,	  0x004C);
	 write_cmos_sensor(0x3876,	  0x0011);
	 write_cmos_sensor(0x3366,	  0x0128);
	 write_cmos_sensor(0x3852,	  0x00EA);
	 write_cmos_sensor(0x623E,	  0x0004);
	 write_cmos_sensor(0x3B5C,	  0x0006);
	
	
	
	//=====================================================================================
	// End Analog/APS settings
	//=====================================================================================
	  mdelay(2);
	// Stream On
	//write_cmos_sensor(0x602A, 0x0100);
	//write_cmos_sensor_8(0x6F12, 0x01);
	
	//  otp_wb_update();//vivo zcw++ 20140910
	//  otp_lsc_update();
	
	  mdelay(5);//vivo zcw++ 20141027 Add for prevent WAIT_IRQ timeout	  
	  LOG_INF("Exit\n");

 }  /*  sensor_init  */
//=====================================================     
// 3M2XX EVT 0.1                                            
// FHD binning 16:9 Normal Mode                             
// X_output size : 2100                                     
// Y_output size : 1556                                     
// Frame_rate : 46.6 fps                                    
// Output_format : RAW 10                                   
// Output_lanes : 4                                         
// Output_clock_mhz : 696 Mhz                               
// System_clock_mhz : 448 Mhz , VT_PIX_clk : 89.6Mhz        
// Input_clock_mhz : 24 Mhz                                 
//=====================================================     
static void preview_setting(void)
{
	LOG_INF("hesong 3 Preview E! ");
		//p200
		mdelay(60);
		
	write_cmos_sensor_8(0x0100, 0x00);
	//check_output_stream_off();
	write_cmos_sensor(0x6028,0x2000);
	write_cmos_sensor(0x602A,0x14F0);
	write_cmos_sensor(0x6F12,0x0000);
	write_cmos_sensor(0x6F12,0x0000);
	write_cmos_sensor(0x6028,0x4000);
	write_cmos_sensor(0x32CA,0x0449);
	write_cmos_sensor(0x32D6,0x0449);
	write_cmos_sensor(0x0344,0x0004);
	write_cmos_sensor(0x0346,0x0000);
	write_cmos_sensor(0x0348,0x107B);
	write_cmos_sensor(0x034A,0x0C3F);
	write_cmos_sensor(0x034C,0x0838);
	write_cmos_sensor(0x034E,0x0618);
	write_cmos_sensor_8(0x0900,0x01  );
	write_cmos_sensor_8(0x0901,0x12  );
	write_cmos_sensor(0x0380,0x0001);
	write_cmos_sensor(0x0382,0x0001);
	write_cmos_sensor(0x0384,0x0001);
	write_cmos_sensor(0x0386,0x0003);
	write_cmos_sensor(0x0400,0x0001);
	write_cmos_sensor(0x0404,0x0020);
	write_cmos_sensor_8(0x0114,0x03  );
	write_cmos_sensor_8(0x0111,0x02  );
	write_cmos_sensor(0x112C,0x0000);
	write_cmos_sensor(0x112E,0x0000);
	write_cmos_sensor(0x0136,0x1800);
	write_cmos_sensor(0x0304,0x0006);
	write_cmos_sensor(0x0306,0x0064);
	write_cmos_sensor(0x0302,0x0001);
	write_cmos_sensor(0x0300,0x0004);
	write_cmos_sensor(0x030C,0x0004);
	write_cmos_sensor(0x030E,0x0042);
	write_cmos_sensor(0x030A,0x0001);
	write_cmos_sensor(0x0308,0x0008);
	write_cmos_sensor(0x0342,0x1CF0);
	write_cmos_sensor(0x0340,0x06F4);
	write_cmos_sensor(0x0202,0x0200);
	write_cmos_sensor(0x0200,0x0400);
	write_cmos_sensor_8(0x0B05,0x01  );
	write_cmos_sensor_8(0x0B08,0x01  );
	write_cmos_sensor_8(0x0B00,0x01  );
	write_cmos_sensor_8(0x3B3C,0x01  );
	write_cmos_sensor(0x3B34,0x3030);
	write_cmos_sensor(0x3B36,0x3030);
	write_cmos_sensor(0x3B38,0x3030);
	write_cmos_sensor(0x3B3A,0x3030);
	write_cmos_sensor(0x306A,0x0068);
	mdelay(60);//vivo zcw++ 20141027 Add for prevent WAIT_IRQ timeout
	write_cmos_sensor_8(0x0100,0x01);
	mdelay(10);//vivo zcw++ 20141027 Add for prevent WAIT_IRQ timeout
	//check_output_stream_on();
	LOG_INF("Exit");

}   /*  preview_setting  */
//===================================================== 
// 3M2XX EVT 0.1										
// Full Resolution Normal Mode 24fps					
// X_output size : 4200 								
// Y_output size : 3112 								
// Frame_rate : 24.07 fps								
// Output_format : RAW 10								
// Output_lanes : 4 									
// Output_clock_mhz : 900 Mhz							
// System_clock_mhz : 448 Mhz , VT_PIX_clk : 89.6Mhz	
// Input_clock_mhz : 24 Mhz 							
//===================================================== 
static void capture_setting(void)
{
	LOG_INF("hesong 3 Normal capture E! ");
	mdelay(60);
	write_cmos_sensor_8(0x0100, 0x00);
	//check_output_stream_off();
	
	write_cmos_sensor(0x6028,0x2000);
	write_cmos_sensor(0x602A,0x14F0);
	write_cmos_sensor(0x6F12,0x0000);
	write_cmos_sensor(0x6F12,0x0000);
	write_cmos_sensor(0x6028,0x4000);
	write_cmos_sensor(0x32CA,0x0449);
	write_cmos_sensor(0x32D6,0x0449);
	write_cmos_sensor(0x0344,0x000c);
	write_cmos_sensor(0x0346,0x000c);
	write_cmos_sensor(0x0348,0x1073);
	write_cmos_sensor(0x034A,0x0C33);
	write_cmos_sensor(0x034C,0x1060);
	write_cmos_sensor(0x034E,0x0C20);
	write_cmos_sensor_8(0x0900,0x01  );
	write_cmos_sensor_8(0x0901,0x11  );
	write_cmos_sensor(0x0380,0x0001);
	write_cmos_sensor(0x0382,0x0001);
	write_cmos_sensor(0x0384,0x0001);
	write_cmos_sensor(0x0386,0x0001);
	write_cmos_sensor(0x0400,0x0002);
	write_cmos_sensor(0x0404,0x0010);
	write_cmos_sensor_8(0x0114,0x03  );
	write_cmos_sensor_8(0x0111,0x02  );
	write_cmos_sensor(0x112C,0x0000);
	write_cmos_sensor(0x112E,0x0000);
	write_cmos_sensor(0x0136,0x1800);
	write_cmos_sensor(0x0304,0x0006);
	write_cmos_sensor(0x0306,0x0064);
	write_cmos_sensor(0x0302,0x0001);
	write_cmos_sensor(0x0300,0x0004);
	write_cmos_sensor(0x030C,0x0004);
	write_cmos_sensor(0x030E,0x0042);
	write_cmos_sensor(0x030A,0x0001);
	write_cmos_sensor(0x0308,0x0008);
	write_cmos_sensor(0x0342,0x1CF0);
	write_cmos_sensor(0x0340,0x0C74);
	write_cmos_sensor(0x0202,0x0200);
	write_cmos_sensor(0x0200,0x0400);
	write_cmos_sensor_8(0x0B05,0x01  );
	write_cmos_sensor_8(0x0B08,0x01  );
	write_cmos_sensor_8(0x0B00,0x01  );
	write_cmos_sensor_8(0x3B3C,0x01  );
	write_cmos_sensor(0x3B34,0x3030);
	write_cmos_sensor(0x3B36,0x3030);
	write_cmos_sensor(0x3B38,0x3030);
	write_cmos_sensor(0x3B3A,0x3030);
	write_cmos_sensor(0x306A,0x0068);	
	mdelay(60);//vivo zcw++ 20141027 Add for prevent WAIT_IRQ timeout
	
	write_cmos_sensor_8(0x0100,0x01);	
		mdelay(10);//vivo zcw++ 20141027 Add for prevent WAIT_IRQ timeout
			//yw++
		LOG_INF("register0x0002= %x",read_cmos_sensor(0x0002));
		write_cmos_sensor(0x0a02, 0x0000);
		write_cmos_sensor(0x0a00, 0x0100);
		mdelay(10);
		LOG_INF("register0x0a22= %x",read_cmos_sensor(0x0a22));
		LOG_INF("register0x0a24= %x",read_cmos_sensor(0x0a24));
		LOG_INF("register0x0a26= %x",read_cmos_sensor(0x0a26));
		LOG_INF("register0x0a28= %x",read_cmos_sensor(0x0a28));
	    write_cmos_sensor_8(0x0a00, 0x00);
		//yw++
		//check_output_stream_on();
		LOG_INF( "Exit!");	

}

static void normal_video_setting(void)
{
    LOG_INF("E! \n");
	preview_setting(); 
}



/*************************************************************************
* FUNCTION
*   get_imgsensor_id
*
* DESCRIPTION
*   This function get the sensor ID 
*
* PARAMETERS
*   *sensorID : return the sensor ID 
*
* RETURNS
*   None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 get_imgsensor_id(UINT32 *sensor_id) 
{
    kal_uint8 i = 0;
    kal_uint8 retry = 2;
    /* static kal_uint8 have_found_sensor = 0x0; */

    /* if (have_found_sensor) */
    /*     return ERROR_SENSOR_CONNECT_FAIL; */

    /*sensor have two i2c address 0x6c 0x6d & 0x21 0x20, we should detect the module used i2c address*/
    while (imgsensor_info.i2c_addr_table[i] != 0xff) {
        spin_lock(&imgsensor_drv_lock);
        imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
        spin_unlock(&imgsensor_drv_lock);
        do {
			write_cmos_sensor(0x602C,0x4000);
			write_cmos_sensor(0x602E,0x0000);
			*sensor_id = read_cmos_sensor(0x6F12);
            if (*sensor_id == S5K3M2_SENSOR_ID /*imgsensor_info.sensor_id*/) {               
                /* have_found_sensor = 0x1; */
				*sensor_id = S5K3M2XY_SENSOR_ID;
                LOG_INF(" klutu addr: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id, *sensor_id);      
                return ERROR_NONE;
            }   
            LOG_INF("Read sensor id fail, id: 0x%x\n", imgsensor.i2c_write_id,*sensor_id);
        } while(--retry > 0);

        i++;
        retry = 2;
    }

    if (*sensor_id != S5K3M2_SENSOR_ID /*imgsensor_info.sensor_id*/) {
        // if Sensor ID is not correct, Must set *sensor_id to 0xFFFFFFFF 
        *sensor_id = 0xFFFFFFFF;
        return ERROR_SENSOR_CONNECT_FAIL;
    }
    return ERROR_NONE;
}


/*************************************************************************
* FUNCTION
*   open
*
* DESCRIPTION
*   This function initialize the registers of CMOS sensor
*
* PARAMETERS
*   None
*
* RETURNS
*   None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 open(void)
{
    //const kal_uint8 i2c_addr[] = {IMGSENSOR_WRITE_ID_1, IMGSENSOR_WRITE_ID_2};
    kal_uint8 i = 0;
    kal_uint8 retry = 2;
    kal_uint16 sensor_id = 0; 
    LOG_INF("PLATFORM:MT6592,MIPI 4LANE\n");
    LOG_INF("preview 2104*1560@30fps,900Mbps/lane;\n video2104*1560@30fps,900Mbps/lane;\n capture 13M@24fps,900Mbps/lane");
    
    //sensor have two i2c address 0x6c 0x6d & 0x21 0x20, we should detect the module used i2c address
    while (imgsensor_info.i2c_addr_table[i] != 0xff) {
        spin_lock(&imgsensor_drv_lock);
        imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
        spin_unlock(&imgsensor_drv_lock);
        do {
			write_cmos_sensor(0x602C,0x4000);
			write_cmos_sensor(0x602E,0x0000);
			sensor_id = read_cmos_sensor(0x6F12);
            if (sensor_id == S5K3M2_SENSOR_ID/*imgsensor_info.sensor_id*/) {
                LOG_INF(" klutu addr: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id, sensor_id);
                //return ERROR_NONE;
				sensor_id = S5K3M2XY_SENSOR_ID;
				break;
            }
            LOG_INF("Read sensor id fail, id: 0x%x\n", imgsensor.i2c_write_id,sensor_id);
        } while(--retry > 0);

		if (imgsensor_info.sensor_id == sensor_id)
			break;
		
        i++;
        retry = 2;
    }

    if (imgsensor_info.sensor_id != sensor_id)
        return ERROR_SENSOR_CONNECT_FAIL;
    
    /* initail sequence write in  */
    sensor_init();

    spin_lock(&imgsensor_drv_lock);

    imgsensor.autoflicker_en= KAL_TRUE;
    imgsensor.sensor_mode = IMGSENSOR_MODE_INIT;
    imgsensor.shutter = 0x3D0;
    imgsensor.gain = 0x100;
    imgsensor.pclk = imgsensor_info.pre.pclk;
    imgsensor.frame_length = imgsensor_info.pre.framelength;
    imgsensor.line_length = imgsensor_info.pre.linelength;
    imgsensor.min_frame_length = imgsensor_info.pre.framelength;
    imgsensor.dummy_pixel = 0;
    imgsensor.dummy_line = 0;
    imgsensor.current_fps=300;
	imgsensor.test_pattern = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);

    return ERROR_NONE;
}   /*  open  */



/*************************************************************************
* FUNCTION
*   close
*
* DESCRIPTION
*   
*
* PARAMETERS
*   None
*
* RETURNS
*   None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 close(void)
{
    LOG_INF("E");

    /*No Need to implement this function*/ 
    
    return ERROR_NONE;
}   /*  close  */


/*************************************************************************
* FUNCTION
* preview
*
* DESCRIPTION
*   This function start the sensor preview.
*
* PARAMETERS
*   *image_window : address pointer of pixel numbers in one period of HSYNC
*  *sensor_config_data : address pointer of line numbers in one period of VSYNC
*
* RETURNS
*   None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 preview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("E");

    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_PREVIEW;
    imgsensor.pclk = imgsensor_info.pre.pclk;
    //imgsensor.video_mode = KAL_FALSE;
    imgsensor.line_length = imgsensor_info.pre.linelength;
    imgsensor.frame_length = imgsensor_info.pre.framelength; 
    imgsensor.min_frame_length = imgsensor_info.pre.framelength;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    preview_setting();
	set_mirror_flip(IMAGE_NORMAL);

    return ERROR_NONE;
}   /*  preview   */

/*************************************************************************
* FUNCTION
*   capture
*
* DESCRIPTION
*   This function setup the CMOS sensor in capture MY_OUTPUT mode
*
* PARAMETERS
*
* RETURNS
*   None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 capture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                          MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("E");
    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_CAPTURE;
    imgsensor.pclk = imgsensor_info.cap.pclk;
    imgsensor.line_length = imgsensor_info.cap.linelength;
    imgsensor.frame_length = imgsensor_info.cap.framelength;  
    imgsensor.min_frame_length = imgsensor_info.cap.framelength;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);

    capture_setting(); 
    set_mirror_flip(IMAGE_NORMAL);
    
    return ERROR_NONE;
}   /* capture() */
static kal_uint32 normal_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("E");
    
    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_VIDEO;
    imgsensor.pclk = imgsensor_info.normal_video.pclk;
    imgsensor.line_length = imgsensor_info.normal_video.linelength;
    imgsensor.frame_length = imgsensor_info.normal_video.framelength;  
    imgsensor.min_frame_length = imgsensor_info.normal_video.framelength;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    normal_video_setting();
    set_mirror_flip(IMAGE_NORMAL);
    
    return ERROR_NONE;
}   /*  normal_video   */




static kal_uint32 get_resolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *sensor_resolution)
{
    LOG_INF("E");
    sensor_resolution->SensorFullWidth = imgsensor_info.cap.grabwindow_width;
    sensor_resolution->SensorFullHeight = imgsensor_info.cap.grabwindow_height;
    
    sensor_resolution->SensorPreviewWidth = imgsensor_info.pre.grabwindow_width;
    sensor_resolution->SensorPreviewHeight = imgsensor_info.pre.grabwindow_height;

    sensor_resolution->SensorVideoWidth = imgsensor_info.normal_video.grabwindow_width;
    sensor_resolution->SensorVideoHeight = imgsensor_info.normal_video.grabwindow_height;       

    return ERROR_NONE;
}   /*  get_resolution  */

static kal_uint32 get_info(MSDK_SCENARIO_ID_ENUM scenario_id,
                      MSDK_SENSOR_INFO_STRUCT *sensor_info,
                      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("scenario_id = %d", scenario_id);

    
    //sensor_info->SensorVideoFrameRate = imgsensor_info.normal_video.max_framerate/10; /* not use */
    //sensor_info->SensorStillCaptureFrameRate= imgsensor_info.cap.max_framerate/10; /* not use */
    //imgsensor_info->SensorWebCamCaptureFrameRate= imgsensor_info.v.max_framerate; /* not use */

    sensor_info->SensorClockPolarity = SENSOR_CLOCK_POLARITY_LOW;
    sensor_info->SensorClockFallingPolarity = SENSOR_CLOCK_POLARITY_LOW; /* not use */
    sensor_info->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW; // inverse with datasheet
    sensor_info->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
    sensor_info->SensorInterruptDelayLines = 4; /* not use */
    sensor_info->SensorResetActiveHigh = FALSE; /* not use */
    sensor_info->SensorResetDelayCount = 5; /* not use */

    sensor_info->SensroInterfaceType = imgsensor_info.sensor_interface_type;

    sensor_info->SensorOutputDataFormat = imgsensor_info.sensor_output_dataformat;

    sensor_info->CaptureDelayFrame = imgsensor_info.cap_delay_frame; 
    sensor_info->PreviewDelayFrame = imgsensor_info.pre_delay_frame; 
    sensor_info->VideoDelayFrame = imgsensor_info.video_delay_frame; 

    sensor_info->SensorMasterClockSwitch = 0; /* not use */
    sensor_info->SensorDrivingCurrent = imgsensor_info.isp_driving_current;
    
    sensor_info->AEShutDelayFrame = imgsensor_info.ae_shut_delay_frame;          /* The frame of setting shutter default 0 for TG int */
    sensor_info->AESensorGainDelayFrame = imgsensor_info.ae_sensor_gain_delay_frame;    /* The frame of setting sensor gain */
    sensor_info->AEISPGainDelayFrame = imgsensor_info.ae_ispGain_delay_frame;   
    
    sensor_info->SensorMIPILaneNumber = imgsensor_info.mipi_lane_num; 
    sensor_info->SensorClockFreq = imgsensor_info.mclk;
    sensor_info->SensorClockDividCount = 3; /* not use */
    sensor_info->SensorClockRisingCount = 0;
    sensor_info->SensorClockFallingCount = 2; /* not use */
    sensor_info->SensorPixelClockCount = 3; /* not use */
    sensor_info->SensorDataLatchCount = 2; /* not use */
    
    sensor_info->MIPIDataLowPwr2HighSpeedTermDelayCount = 0; 
    sensor_info->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
    sensor_info->SensorWidthSampling = 0;  // 0 is default 1x
    sensor_info->SensorHightSampling = 0;   // 0 is default 1x 
    sensor_info->SensorPacketECCOrder = 1;

    switch (scenario_id) {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
            sensor_info->SensorGrabStartX = imgsensor_info.pre.startx; 
            sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;      
            
            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.pre.mipi_data_lp2hs_settle_dc;
            
            break;
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
            sensor_info->SensorGrabStartX = imgsensor_info.cap.startx; 
            sensor_info->SensorGrabStartY = imgsensor_info.cap.starty;
                  
            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.cap.mipi_data_lp2hs_settle_dc; 

            break;   
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
            
            sensor_info->SensorGrabStartX = imgsensor_info.normal_video.startx; 
            sensor_info->SensorGrabStartY = imgsensor_info.normal_video.starty;
       
            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.normal_video.mipi_data_lp2hs_settle_dc; 

            break;    
        default:            
            sensor_info->SensorGrabStartX = imgsensor_info.pre.startx; 
            sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;      
            
            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.pre.mipi_data_lp2hs_settle_dc;
            break;
    }
    
    return ERROR_NONE;
}   /*  get_info  */


static kal_uint32 control(MSDK_SCENARIO_ID_ENUM scenario_id, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("scenario_id = %d", scenario_id);
    spin_lock(&imgsensor_drv_lock);
    imgsensor.current_scenario_id = scenario_id;
    spin_unlock(&imgsensor_drv_lock);
    switch (scenario_id) {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
            preview(image_window, sensor_config_data);
            break;
        case MSDK_SCENARIO_ID_CAMERA_ZSD:
            LOGE("Does Not Support ZSD Feature\n");
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:      
            capture(image_window, sensor_config_data);
            break;  
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
            normal_video(image_window, sensor_config_data);
            break;    
  
        default:
            LOG_INF("Error ScenarioId setting");
            preview(image_window, sensor_config_data);
            return ERROR_INVALID_SCENARIO_ID;
    }
    return ERROR_NONE;
}   /* control() */



static kal_uint32 set_video_mode(UINT16 framerate)
{
    LOG_INF("framerate = %d ", framerate);
    // SetVideoMode Function should fix framerate
    if (framerate == 0)
        // Dynamic frame rate
        return ERROR_NONE;
    spin_lock(&imgsensor_drv_lock);
    if ((framerate == 30) && (imgsensor.autoflicker_en == KAL_TRUE))
        imgsensor.current_fps = 296;
    else if ((framerate == 15) && (imgsensor.autoflicker_en == KAL_TRUE))
        imgsensor.current_fps = 146;
    else
        imgsensor.current_fps = 10 * framerate;
    spin_unlock(&imgsensor_drv_lock);
	set_max_framerate(imgsensor.current_fps,1);

    return ERROR_NONE;
}

static kal_uint32 set_auto_flicker_mode(kal_bool enable, UINT16 framerate)
{
    LOG_INF("enable = %d, framerate = %d ", enable, framerate);
    spin_lock(&imgsensor_drv_lock);
    if (enable)       
        imgsensor.autoflicker_en = KAL_TRUE;
    else //Cancel Auto flick
        imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    return ERROR_NONE;
}


static kal_uint32 set_max_framerate_by_scenario(MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 framerate) 
{
    
  	kal_uint32 frame_length;
    LOG_INF("scenario_id = %d, framerate = %d", scenario_id, framerate);

    switch (scenario_id) {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			frame_length = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
            spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.pre.framelength) ? (frame_length - imgsensor_info.pre.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
            set_dummy();            
            break;          
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			frame_length = imgsensor_info.normal_video.pclk / framerate * 10 / imgsensor_info.normal_video.linelength;
            spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.normal_video.framelength) ? (frame_length - imgsensor_info.normal_video.framelength) : 0;			
			imgsensor.frame_length = imgsensor_info.normal_video.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
            set_dummy();            
            break;
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
        case MSDK_SCENARIO_ID_CAMERA_ZSD:           
			frame_length = imgsensor_info.cap.pclk / framerate * 10 / imgsensor_info.cap.linelength;
            spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.cap.framelength) ? (frame_length - imgsensor_info.cap.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.cap.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
            set_dummy();            
            break;  
            
        default:            
            LOG_INF("error scenario_id = %d", scenario_id);
            break;
    }   
    return ERROR_NONE;
}


static kal_uint32 get_default_framerate_by_scenario(MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 *framerate) 
{
	LOG_INF("scenario_id = %d\n", scenario_id);

    switch (scenario_id) {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
            *framerate = imgsensor_info.pre.max_framerate;
            break;
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
            *framerate = imgsensor_info.normal_video.max_framerate;
            break;
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
            *framerate = imgsensor_info.cap.max_framerate;
            break;      
        default:
            break;
    }

    return ERROR_NONE;
}

static kal_uint32 set_test_pattern_mode(kal_bool enable)
{
	LOG_INF("enable: %d\n", enable);

    if (enable) {
        // 0x5E00[8]: 1 enable,  0 disable
        // 0x5E00[1:0]; 00 Color bar, 01 Random Data, 10 Square, 11 BLACK
        write_cmos_sensor(0x0600, 0x0002);
    } else {
        // 0x5E00[8]: 1 enable,  0 disable
        // 0x5E00[1:0]; 00 Color bar, 01 Random Data, 10 Square, 11 BLACK
        write_cmos_sensor(0x0600, 0x0000);
    }    
	spin_lock(&imgsensor_drv_lock);
	imgsensor.test_pattern = enable;
	spin_unlock(&imgsensor_drv_lock);
    return ERROR_NONE;
}

static kal_uint32 feature_control(MSDK_SENSOR_FEATURE_ENUM feature_id,
                             UINT8 *feature_para,UINT32 *feature_para_len)
{
    UINT16 *feature_return_para_16=(UINT16 *) feature_para;
    UINT16 *feature_data_16=(UINT16 *) feature_para;
    UINT32 *feature_return_para_32=(UINT32 *) feature_para;
    UINT32 *feature_data_32=(UINT32 *) feature_para;
      
    MSDK_SENSOR_REG_INFO_STRUCT *sensor_reg_data=(MSDK_SENSOR_REG_INFO_STRUCT *) feature_para;
 
    LOG_INF("feature_id = %d", feature_id);
    switch (feature_id) {
        case SENSOR_FEATURE_GET_PERIOD:
            *feature_return_para_16++ = imgsensor.line_length;
            *feature_return_para_16 = imgsensor.frame_length;
            *feature_para_len=4;
            break;
        case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:    
            *feature_return_para_32 = imgsensor.pclk;
            *feature_para_len=4;
            break;         
        case SENSOR_FEATURE_SET_ESHUTTER:
            set_shutter(*feature_data_16);
            break;
        case SENSOR_FEATURE_SET_NIGHTMODE:
            
            break;
        case SENSOR_FEATURE_SET_GAIN:       
            set_gain((UINT16) *feature_data_16);
            break;
        case SENSOR_FEATURE_SET_FLASHLIGHT:
            break;
        case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
            break;
        case SENSOR_FEATURE_SET_REGISTER:
            if((sensor_reg_data->RegData>>8)!=0)
                write_cmos_sensor(sensor_reg_data->RegAddr, sensor_reg_data->RegData);
            else
                write_cmos_sensor_8(sensor_reg_data->RegAddr, sensor_reg_data->RegData);
            break;
            break;
        case SENSOR_FEATURE_GET_REGISTER:
            sensor_reg_data->RegData = read_cmos_sensor(sensor_reg_data->RegAddr);
            break;
        case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
            // get the lens driver ID from EEPROM or just return LENS_DRIVER_ID_DO_NOT_CARE
            // if EEPROM does not exist in camera module.
            *feature_return_para_32=LENS_DRIVER_ID_DO_NOT_CARE;
            *feature_para_len=4;
            break;
        case SENSOR_FEATURE_SET_VIDEO_MODE:
            set_video_mode(*feature_data_16);
            break; 
        case SENSOR_FEATURE_CHECK_SENSOR_ID:
            get_imgsensor_id(feature_return_para_32); 
            break; 
        case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE:
            set_auto_flicker_mode((BOOL)*feature_data_16,*(feature_data_16+1));
            break;
        case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
            set_max_framerate_by_scenario((MSDK_SCENARIO_ID_ENUM)*feature_data_32, *(feature_data_32+1));
            break;
        case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
            get_default_framerate_by_scenario((MSDK_SCENARIO_ID_ENUM)*feature_data_32, (MUINT32 *)(*(feature_data_32+1)));
            break;
        case SENSOR_FEATURE_SET_TEST_PATTERN:
            set_test_pattern_mode((BOOL)*feature_data_16);
            break;
        case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE: //for factory mode auto testing             
            *feature_return_para_32 = imgsensor_info.checksum_value;
            *feature_para_len=4;                             
            break;              
        default:
            break;
    }
  
    return ERROR_NONE;
}   /*  feature_control()  */

SENSOR_FUNCTION_STRUCT xy_sensor_func = {
    open,
    get_info,
    get_resolution,
    feature_control,
    control,
    close
};

UINT32 S5K3M2XY_MIPI_RAW_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
    /* To Do : Check Sensor status here */
    if (pfFunc!=NULL)
        *pfFunc=&xy_sensor_func;
    return ERROR_NONE;
}   /*  S5K3M2_MIPI_RAW_SensorInit  */
