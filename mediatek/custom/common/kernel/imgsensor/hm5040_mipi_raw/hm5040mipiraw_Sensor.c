/*****************************************************************************
*  Copyright Statement:
*  --------------------
*  This software is protected by Copyright and the information contained
*  herein is confidential. The software may not be copied and the information
*  contained herein may not be used or disclosed except with the written
*  permission of MediaTek Inc. (C) 2008
*
*  BY OPENING THIS FILE, BUYER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
*  THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
*  RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO BUYER ON
*  AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
*  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
*  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
*  NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
*  SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
*  SUPPLIED WITH THE MEDIATEK SOFTWARE, AND BUYER AGREES TO LOOK ONLY TO SUCH
*  THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. MEDIATEK SHALL ALSO
*  NOT BE RESPONSIBLE FOR ANY MEDIATEK SOFTWARE RELEASES MADE TO BUYER'S
*  SPECIFICATION OR TO CONFORM TO A PARTICULAR STANDARD OR OPEN FORUM.
*
*  BUYER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND CUMULATIVE
*  LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
*  AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
*  OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY BUYER TO
*  MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
*
*  THE TRANSACTION CONTEMPLATED HEREUNDER SHALL BE CONSTRUED IN ACCORDANCE
*  WITH THE LAWS OF THE STATE OF CALIFORNIA, USA, EXCLUDING ITS CONFLICT OF
*  LAWS PRINCIPLES.  ANY DISPUTES, CONTROVERSIES OR CLAIMS ARISING THEREOF AND
*  RELATED THERETO SHALL BE SETTLED BY ARBITRATION IN SAN FRANCISCO, CA, UNDER
*  THE RULES OF THE INTERNATIONAL CHAMBER OF COMMERCE (ICC).
*
*****************************************************************************/
/*****************************************************************************

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

#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"

#include "hm5040mipiraw_Sensor.h"
#include "hm5040mipiraw_Camera_Sensor_para.h"
#include "hm5040mipiraw_CameraCustomized.h"

//kal_bool HM5040MIPI_MPEG4_encode_mode = KAL_FALSE;
//kal_bool HM5040MIPI_Auto_Flicker_mode = KAL_FALSE;

//kal_uint8 HM5040MIPI_sensor_write_I2C_address = HM5040MIPI_WRITE_ID;
//kal_uint8 HM5040MIPI_sensor_read_I2C_address = HM5040MIPI_READ_ID;
static kal_bool HM5040MIPIAutoFlicKerMode = KAL_FALSE;

#define DELAY_FRAME 10
static kal_uint8 frame_count = DELAY_FRAME;
static kal_uint8 current_gain = 0;
static kal_uint16 current_shutter = 0;
static kal_uint8 shutterFlag=0;
#if 0
static struct HM5040MIPI_sensor_STRUCT HM5040MIPI_sensor={HM5040MIPI_WRITE_ID,HM5040MIPI_READ_ID,KAL_TRUE,KAL_FALSE,KAL_TRUE,KAL_FALSE,
KAL_FALSE,KAL_FALSE,KAL_FALSE,168000000,168000000,168000000,0,0,0,64,64,64,HM5040MIPI_PV_FULL_PIXELS,HM5040MIPI_PV_FULL_LINES,
HM5040MIPI_VIDEO_FULL_PIXELS,HM5040MIPI_VIDEO_FULL_LINES,HM5040MIPI_CAP_FULL_PIXELS,HM5040MIPI_CAP_FULL_LINES,0,0,0,0,0,0,
30,HM5040MIPI_PV_FULL_LINES,HM5040MIPI_PV_FULL_PIXELS,0};
#endif

#if 1
static struct HM5040MIPI_sensor_STRUCT HM5040MIPI_sensor =
{
    .i2c_write_id = HM5040MIPI_WRITE_ID,
    .i2c_read_id = HM5040MIPI_READ_ID,
    .first_init = KAL_TRUE,
    .fix_video_fps = KAL_FALSE,
    .pv_mode = KAL_TRUE,
    .video_mode = KAL_FALSE,
    .capture_mode = KAL_FALSE,//True: Preview Mode; False: Capture Mode
    .night_mode = KAL_FALSE,//True: Night Mode; False: Auto Mode
    .mirror_flip = KAL_FALSE,
    .pv_pclk = 120000000,//Preview Pclk                 //use
    .video_pclk = 120000000,//video Pclk                //use
    .cp_pclk = 168000000,//Capture Pclk                 //use
    .pv_shutter = 0,
    .video_shutter = 0,
    .cp_shutter = 0,
    .pv_gain = 64,
    .video_gain = 64,
    .cp_gain = 64,
    .pv_line_length = HM5040MIPI_PV_FULL_PIXELS,        //use
    .pv_frame_length = HM5040MIPI_PV_FULL_LINES,        //use
    .video_line_length = HM5040MIPI_VIDEO_FULL_PIXELS,  //use
    .video_frame_length = HM5040MIPI_VIDEO_FULL_LINES,  //use
    .cp_line_length = HM5040MIPI_CAP_FULL_PIXELS,       //use
    .cp_frame_length = HM5040MIPI_CAP_FULL_LINES,       //use
    .pv_dummy_pixels = 0,//Dummy Pixels:must be 12s
    .pv_dummy_lines = 0,//Dummy Lines
    .video_dummy_pixels = 0,//Dummy Pixels:must be 12s
    .video_dummy_lines = 0,//Dummy Lines
    .cp_dummy_pixels = 0,//Dummy Pixels:must be 12s
    .cp_dummy_lines = 0,//Dummy Lines
    .video_current_frame_rate = 30,
    .frame_height = HM5040MIPI_PV_FULL_LINES,
    .line_length = HM5040MIPI_PV_FULL_PIXELS,
    .FixedFps = 0,
};
#endif

static struct HM5040_GAIN HM5040_gain_table[]=
{
//0x0204/0x0205
    {0x0000, 100},
    {0x0010, 106},
    {0x0020, 109},
    {0x0030, 118},
    {0x0040, 129},
    {0x0050, 150},
    {0x0060, 159},
    {0x0070, 179},
    {0x0080, 200},
    {0x0090, 229},
    {0x00A0, 268},
    {0x00B0, 318},
    {0x00C0, 400},
    {0x00D0, 532},
    {0x00E0, 800},
    {0x00E4, 909},
    {0x00E8, 1068},
    {0x00EC, 1279},
    {0x00F0, 1600},
    {0xFFFF, 0},
};

static struct HM5040_GAIN HM5040_D_gain_table[]=
{
    {0x0000, 100},
	{0x0008, 103},
	{0x0010, 106},
	{0x0018, 109},
	{0x0020, 113},
	{0x0028, 116},
	{0x0030, 119},
	{0x0038, 122},
	{0x0040, 125},
	{0x0048, 128},
	{0x0050, 131},
	{0x0058, 134},
	{0x0060, 138},
	{0x0068, 141},
	{0x0070, 144},
	{0x0078, 147},
	{0x0080, 150},
	{0x0088, 153},
	{0x0090, 156},
	{0x0098, 159},
	{0x00A0, 163},
	{0x00A8, 166},
	{0x00B0, 169},
	{0x00B8, 172},
	{0x00C0, 175},
	{0x00C8, 178},
	{0x00D0, 181},
	{0x00D8, 184},
	{0x00E0, 188},
	{0x00E8, 191},
	{0x00F0, 194},
	{0x00F8, 197},
    {0xFFFF, 0},
};
MSDK_SCENARIO_ID_ENUM HM5040_CurrentScenarioId = MSDK_SCENARIO_ID_CAMERA_PREVIEW;
//kal_uint16 HM5040MIPI_sensor_gain_base = 0x0;
/* MAX/MIN Explosure Lines Used By AE Algorithm */
kal_uint16 HM5040MIPI_MAX_EXPOSURE_LINES = HM5040MIPI_PV_DUMMY_LINES-5;//650;
kal_uint8  HM5040MIPI_MIN_EXPOSURE_LINES = 2;
kal_uint32 HM5040MIPI_isp_master_clock; //use

UINT32 HM5040MIPISetMaxFrameRate(UINT16 u2FrameRate);

static DEFINE_SPINLOCK(HM5040_drv_lock);

#define HIMAX_DEBUG
#ifdef HIMAX_DEBUG
#define SENSORDB(fmt, arg...) printk( "***Walter debug: @%s " fmt "\n", __FUNCTION__, ##arg)
#else
#define SENSORDB(fmt, arg...) printk( "[HM5040MIPIRaw] %s " fmt "\n", __FUNCTION__, ##arg)
#endif
//#define RETAILMSG(x,...)
//#define TEXT
//UINT8 HM5040MIPIPixelClockDivider = 0;
kal_uint16 HM5040MIPI_sensor_id = 0;
MSDK_SENSOR_CONFIG_STRUCT HM5040MIPISensorConfigData; //use
kal_uint32 HM5040MIPI_FAC_SENSOR_REG;
kal_uint16 HM5040MIPI_sensor_flip_value;
/* FIXME: old factors and DIDNOT use now. s*/
SENSOR_REG_STRUCT HM5040MIPISensorCCT[]=CAMERA_SENSOR_CCT_DEFAULT_VALUE; //use
SENSOR_REG_STRUCT HM5040MIPISensorReg[ENGINEER_END]=CAMERA_SENSOR_REG_DEFAULT_VALUE; //use
/* FIXME: old factors and DIDNOT use now. e*/
extern int iReadReg(u16 a_u2Addr , u8 * a_puBuff , u16 i2cId);
extern int iWriteReg(u16 a_u2Addr , u32 a_u4Data , u32 a_u4Bytes , u16 i2cId);
#define HM5040MIPI_write_cmos_sensor(addr, para) iWriteReg((u16) addr , (u32) para , 1, HM5040MIPI_WRITE_ID)

kal_uint16 HM5040MIPI_read_cmos_sensor(kal_uint32 addr)
{
    kal_uint16 get_byte=0;
    iReadReg((u16) addr ,(u8*)&get_byte,HM5040MIPI_WRITE_ID);
    return get_byte;
}

void HM5040MIPI_write_shutter(kal_uint16 shutter)
{
    kal_uint32 dummy_line = 0,dummy_pixel=0,shutter1=0,shutter2=0;
    kal_uint32 extra_lines = 0;
    kal_uint32 max_exp_shutter = 0;
    unsigned long flags;
    //SENSORDB("[HM5040MIPI]enter HM5040MIPI_write_shutter function,shutter=%d\n",shutter);
    SENSORDB("[S], shutter = 0x%x", shutter);
    if(HM5040MIPIAutoFlicKerMode)
    {
        if(HM5040MIPI_sensor.video_mode == KAL_FALSE)
        {
            if(HM5040_CurrentScenarioId == MSDK_SCENARIO_ID_CAMERA_ZSD)
            {
                //Change frame 14.7fps ~ 14.9fps to do auto flick
                HM5040MIPISetMaxFrameRate(148);
            }
            else
            {
                //Change frame 29.5fps ~ 29.8fps to do auto flick
                HM5040MIPISetMaxFrameRate(148);
            }
        }
    }
    if (HM5040MIPI_sensor.pv_mode == KAL_TRUE)
    {
        max_exp_shutter = HM5040MIPI_PV_FULL_LINES + HM5040MIPI_sensor.pv_dummy_lines;
    }
    else if (HM5040MIPI_sensor.video_mode== KAL_TRUE)
    {
        max_exp_shutter = HM5040MIPI_VIDEO_FULL_LINES + HM5040MIPI_sensor.video_dummy_lines;
    }
    else if (HM5040MIPI_sensor.capture_mode== KAL_TRUE)
    {
        max_exp_shutter = HM5040MIPI_CAP_FULL_LINES + HM5040MIPI_sensor.cp_dummy_lines;
    }
    else
    {
        SENSORDB("sensor mode error");
    }

    if(shutter > max_exp_shutter)
        extra_lines = shutter - max_exp_shutter;
    else
        extra_lines = 0;
    if (HM5040MIPI_sensor.pv_mode == KAL_TRUE)
    {
        SENSORDB("PV shutter, shutter = 0x%x",shutter);
        dummy_line =HM5040MIPI_PV_DUMMY_LINES+ HM5040MIPI_sensor.pv_dummy_lines + extra_lines;
        dummy_pixel = HM5040MIPI_PV_DUMMY_PIXELS+ HM5040MIPI_sensor.pv_dummy_pixels;
        spin_lock_irqsave(&HM5040_drv_lock,flags);
        HM5040MIPI_sensor.pv_line_length  = HM5040MIPI_PV_FULL_PIXELS+dummy_pixel;
        HM5040MIPI_sensor.pv_frame_length = HM5040MIPI_PV_FULL_LINES+dummy_line;
        HM5040MIPI_sensor.frame_height= HM5040MIPI_sensor.pv_frame_length;
        HM5040MIPI_sensor.line_length = HM5040MIPI_sensor.pv_line_length;
        spin_unlock_irqrestore(&HM5040_drv_lock,flags);
    }
    else if (HM5040MIPI_sensor.video_mode== KAL_TRUE)
    {
        SENSORDB("video shutter, shutter = 0x%x",shutter);
        dummy_line = HM5040MIPI_VIDEO_DUMMY_LINES+ HM5040MIPI_sensor.video_dummy_lines + extra_lines;
        dummy_pixel =HM5040MIPI_VIDEO_DUMMY_PIXELS + HM5040MIPI_sensor.video_dummy_pixels;
        spin_lock_irqsave(&HM5040_drv_lock,flags);
        HM5040MIPI_sensor.video_line_length  = HM5040MIPI_VIDEO_FULL_PIXELS+dummy_pixel;
        HM5040MIPI_sensor.video_frame_length = HM5040MIPI_VIDEO_FULL_LINES+dummy_line;
        HM5040MIPI_sensor.line_length  = HM5040MIPI_sensor.video_line_length; 
        HM5040MIPI_sensor.frame_height = HM5040MIPI_sensor.video_frame_length;
        spin_unlock_irqrestore(&HM5040_drv_lock,flags);
    }
    else if(HM5040MIPI_sensor.capture_mode== KAL_TRUE)
    {
#if 0
        dummy_line = HM5040MIPI_FULL_DUMMY_LINES+ HM5040MIPI_sensor.cp_dummy_lines + extra_lines;
        dummy_pixel =HM5040MIPI_FULL_DUMMY_PIXELS + HM5040MIPI_sensor.cp_dummy_pixels;
        spin_lock_irqsave(&HM5040_drv_lock,flags);
        HM5040MIPI_sensor.cp_line_length = HM5040MIPI_REAL_CAP_WIDTH+dummy_pixel;
        HM5040MIPI_sensor.cp_frame_length = HM5040MIPI_REAL_CAP_HEIGHT+dummy_line;
        HM5040MIPI_sensor.frame_height = HM5040MIPI_sensor.cp_frame_length;
        HM5040MIPI_sensor.line_length  = HM5040MIPI_sensor.cp_line_length;
        spin_unlock_irqrestore(&HM5040_drv_lock,flags);
#else
        SENSORDB("cap shutter, shutter = 0x%x",shutter);
        dummy_line = HM5040MIPI_PV_DUMMY_LINES+ HM5040MIPI_sensor.pv_dummy_lines + extra_lines;
        dummy_pixel =HM5040MIPI_PV_DUMMY_PIXELS+ HM5040MIPI_sensor.pv_dummy_pixels;
        spin_lock_irqsave(&HM5040_drv_lock,flags);
        HM5040MIPI_sensor.cp_line_length = HM5040MIPI_CAP_FULL_PIXELS+dummy_pixel;
        HM5040MIPI_sensor.cp_frame_length = HM5040MIPI_CAP_FULL_LINES+dummy_line;
        HM5040MIPI_sensor.frame_height = HM5040MIPI_sensor.cp_frame_length;
        HM5040MIPI_sensor.line_length  = HM5040MIPI_sensor.cp_line_length;
        spin_unlock_irqrestore(&HM5040_drv_lock,flags);
#endif
    }
    else
    {
        SENSORDB("sensor mode error");
    }
    HM5040MIPI_write_cmos_sensor(0x0104, 0x01);
       if(shutterFlag==0)
       {
        HM5040MIPI_write_cmos_sensor(0x0201, 0xAF);
        shutterFlag=1;}
       else
       {
       HM5040MIPI_write_cmos_sensor(0x0201, 0xAE);
       shutterFlag=0;}
    HM5040MIPI_write_cmos_sensor(0x0202, (shutter >> 8) & 0xFF);
    HM5040MIPI_write_cmos_sensor(0x0203,  shutter  & 0xFF);




    shutter2=shutter+0x0A;

    if (shutter2<0x07C4)// && HM5040MIPI_sensor.capture_mode== KAL_TRUE)
    {
        HM5040MIPI_write_cmos_sensor(0x0340,  0x07);
        HM5040MIPI_write_cmos_sensor(0x0341,  0xC4);
    }
    //else if ( (HM5040MIPI_sensor.video_mode== KAL_TRUE || HM5040MIPI_sensor.pv_mode== KAL_TRUE) && shutter2< 1280)
    //{
    //HM5040MIPI_write_cmos_sensor(0x0340,  0x05);
    //HM5040MIPI_write_cmos_sensor(0x0341,  0x00);
    //}
    else
    {
        HM5040MIPI_write_cmos_sensor(0x0340,  (shutter2 >> 8) & 0xFF);
        HM5040MIPI_write_cmos_sensor(0x0341,  shutter2  & 0xFF);
//        HM5040MIPI_write_cmos_sensor(0x0340,  (0x174B >> 8) & 0xFF);
//        HM5040MIPI_write_cmos_sensor(0x0341,  0x174B  & 0xFF);
    }





    SENSORDB("[E]");
}   /* write_HM5040MIPI_shutter */

#if 0
static kal_uint16 HM5040MIPIReg2Gain(const kal_uint8 iReg)
{
    kal_uint16 Reg_Cgain=0,Reg_Fgain=0,i=0;
    float gain_tmp0 =0;
    SENSORDB("[HM5040MIPI]enter HM5040MIPIReg2Gain function\n");
    Reg_Cgain=HM5040MIPI_read_cmos_sensor(0x0204);
    Reg_Fgain=HM5040MIPI_read_cmos_sensor(0x0205);
    switch (Reg_Fgain)
    {
        case 0x00:
            //gain_tmp0=Reg_Fgain;
            gain_tmp0=1.00;
            break;
        case 0x10:
            //gain_tmp0=2*Reg_Fgain;
            gain_tmp0=1.07;
            break;
        case 0x20:
            //gain_tmp0=4*Reg_Fgain;
            gain_tmp0=1.14;
            break;
        case 0x30:
            //gain_tmp0=8*Reg_Fgain;
            gain_tmp0=1.23;
            break;
        case 0x40:
            //gain_tmp0=8*Reg_Fgain;
            gain_tmp0=1.33;
            break;
        case 0x50:
            //gain_tmp0=8*Reg_Fgain;
            gain_tmp0=1.45;
            break;
        case 0x60:
            //gain_tmp0=8*Reg_Fgain;
            gain_tmp0=1.60;
            break;
        case 0x70:
            //gain_tmp0=8*Reg_Fgain;
            gain_tmp0=1.78;
            break;
        case 0x80:
            //gain_tmp0=8*Reg_Fgain;
            gain_tmp0=2.00;
            break;
        case 0x90:
            //gain_tmp0=8*Reg_Fgain;
            gain_tmp0=2.29;
            break;
        case 0xA0:
            //gain_tmp0=8*Reg_Fgain;
            gain_tmp0=2.66;
            break;
        case 0xB0:
            //gain_tmp0=8*Reg_Fgain;
            gain_tmp0=3.20;
            break;
        case 0xC0:
            //gain_tmp0=8*Reg_Fgain;
            gain_tmp0=4.00;
            break;
        case 0xD0:
            //gain_tmp0=8*Reg_Fgain;
            gain_tmp0=5.31;
            break;
        case 0xE0:
            //gain_tmp0=8*Reg_Fgain;
            gain_tmp0=8.03;
            break;
        case 0xE4:
            //gain_tmp0=8*Reg_Fgain;
            gain_tmp0=9.12;
            break;
        case 0xE8:
            //gain_tmp0=8*Reg_Fgain;
            gain_tmp0=10.72;
            break;
        case 0xEC:
            //gain_tmp0=8*Reg_Fgain;
            gain_tmp0=12.74;
            break;
        case 0xF0:
            //gain_tmp0=8*Reg_Fgain;
            gain_tmp0=16.03;
            break;
        default:
           gain_tmp0=0;
    }
    SENSORDB("[HM5040MIPI]exit HM5040MIPIReg2Gain function\n");
    return (kal_uint8)gain_tmp0;
    //SENSORDB("[HM5040MIPI]mycat11 enter HM5040MIPIReg2Gain function iGain=%d\n",gain_tmp0);
}
#endif

/*************************************************************************
* FUNCTION
*    HM5040MIPIGain2Reg
*
* DESCRIPTION
*    This function is to set global gain to sensor.
*
* PARAMETERS
*    gain : sensor global gain(base: 0x40)
*
* RETURNS
*    the actually gain set to sensor.
*gain=2^Reg_Cgain +Reg_Fgain/64
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint8 HM5040MIPIGain2Reg(const kal_uint16 iGain)
{
    /*******************gain=2^Reg_Cgain +Reg_Fgain/64*****************************/
    //SENSORDB("[HM5040MIPI]mycat enter HM5040MIPIReg2Gain function iGain=%d\n",iGain);
    SENSORDB("[S], iGain = %d", iGain);
    kal_uint8 i = 0;
    kal_uint8 gain_tmp1=0,Reg_Cgain=0,Reg_Dgain=0;
    kal_uint16 gain_tmp0 = 0;
    //iGain_temp=(float)iGain;
#if 1
    gain_tmp0=(iGain*100) / BASEGAIN;
    //SENSORDB("[HM5040MIPI]mycat BASEGAIN=%d\n",BASEGAIN);
    
    //gain_tmp1=(iGain % BASEGAIN);
    do {
        if (gain_tmp0 < HM5040_gain_table[i].analoggain)
            break;
        i++;
    } while (HM5040_gain_table[i].analoggain != 0);

    if (i == 0)
    {
        Reg_Cgain = 0x00;
        SENSORDB("[brad_gain] iGain=%d gain_tmp0=%d  analoggain=%d  Reg_Cgain = 0x%x    \n",iGain,gain_tmp0,HM5040_gain_table[i].analoggain,Reg_Cgain);

    }
    else
    {
        Reg_Cgain = HM5040_gain_table[i-1].gainvalue & 0xFF;
        SENSORDB("[brad_gain] iGain=%d gain_tmp0=%d  analoggain=%d  Reg_Cgain = 0x%x    \n",iGain,gain_tmp0,HM5040_gain_table[i-1].analoggain,Reg_Cgain);

    }
    //Reg_Fgain = Reg_Fgain & 0xFF;
    //HM5040MIPI_write_cmos_sensor(0x0204,0x00);
//    HM5040MIPI_write_cmos_sensor(0x0205,Reg_Cgain);
//    HM5040MIPI_write_cmos_sensor(0x0104, 0x00); 
	SENSORDB("[E], analog Reg_Cgain = 0x%x", Reg_Cgain);
#endif
 


#if 1
	//gain_tmp0 = (HM5040_gain_table[i].analoggain * 100) / gain_tmp0;
	gain_tmp0 = (gain_tmp0 * 100) /  HM5040_gain_table[i-1].analoggain;
	i = 0;
	do {
        if (gain_tmp0 < HM5040_D_gain_table[i].analoggain)
            break;
		SENSORDB("[1], HM5040_D_gain_table[i].analoggain = %d", HM5040_D_gain_table[i].analoggain);
        i++;
    } while (HM5040_D_gain_table[i].analoggain != 0);
	if (i == 0)
    {
        Reg_Dgain = 0x00;
    }
    else
    {
        Reg_Dgain = HM5040_D_gain_table[i-1].gainvalue & 0xFF;
    }


HM5040MIPI_write_cmos_sensor(0x020f,Reg_Dgain);
HM5040MIPI_write_cmos_sensor(0x0211,Reg_Dgain);
HM5040MIPI_write_cmos_sensor(0x0213,Reg_Dgain);
HM5040MIPI_write_cmos_sensor(0x0215,Reg_Dgain);
    HM5040MIPI_write_cmos_sensor(0x0205,Reg_Cgain);

	SENSORDB("[brad_digitG], digital Reg_Dgain = 0x%x", Reg_Dgain);


#endif

//mdelay(30);

    HM5040MIPI_write_cmos_sensor(0x0104, 0x00); 

    return NONE;
}

/*************************************************************************
* FUNCTION
*    HM5040MIPI_SetGain
*
* DESCRIPTION
*    This function is to set global gain to sensor.
*
* PARAMETERS
*    gain : sensor global gain(base: 0x40)
*
* RETURNS
*    the actually gain set to sensor.
*
* GLOBALS AFFECTED
*
*************************************************************************/
void HM5040MIPI_SetGain(UINT16 iGain)
{
    //SENSORDB("[HM5040MIPI]enter HM5040MIPI_SetGain function iGain =%d\n",iGain);
    SENSORDB("[S], iGain = 0x%x", iGain);
HM5040MIPI_write_cmos_sensor(0x0104, 0x01);//20140523
    HM5040MIPIGain2Reg(iGain);
HM5040MIPI_write_cmos_sensor(0x0104, 0x00);//20140523
    //static kal_uint8 frame_count = 3;
    //static UINT16 current_gain = 0;
#if 0
    if(iGain == current_gain)
    {
        frame_count--;
    }
    else
    {
        current_gain = iGain;
        frame_count = DELAY_FRAME;
        SENSORDB("Walter debug: disable BLC\n");
        HM5040MIPI_write_cmos_sensor(0xA651,0x01);
        HM5040MIPIGain2Reg(iGain);
    }
    if(frame_count == 0)
    {
        SENSORDB("Walter debug: enable BLC\n");
        HM5040MIPI_write_cmos_sensor(0xA651,0x02);
    }
#endif
    //SENSORDB("[HM5040MIPI]exit HM5040MIPI_SetGain function\n");
    SENSORDB("[E]");
}   /*  HM5040MIPI_SetGain_SetGain  */

/*************************************************************************
* FUNCTION
*    read_HM5040MIPI_gain
*
* DESCRIPTION
*    This function is to set global gain to sensor.
*
* PARAMETERS
*    None
*
* RETURNS
*    gain : sensor global gain(base: 0x40)
*
* GLOBALS AFFECTED
*
*************************************************************************/
kal_uint16 read_HM5040MIPI_gain(void)
{
    kal_uint16 Reg_Cgain=0,Reg_Fgain=0,i=0;
    float gain_tmp0 =0;
    SENSORDB("[HM5040MIPI]enter read_HM5040MIPI_gain function\n");
    Reg_Cgain=HM5040MIPI_read_cmos_sensor(0x0204);
    Reg_Fgain=HM5040MIPI_read_cmos_sensor(0x0205);
    switch (Reg_Fgain)
    {
        case 0x00:
            //gain_tmp0=Reg_Fgain;
            gain_tmp0=1.00;
            break;
        case 0x10:
            //gain_tmp0=2*Reg_Fgain;
            gain_tmp0=1.07;
            break;
        case 0x20:


            //gain_tmp0=4*Reg_Fgain;
            gain_tmp0=1.14;
            break;
        case 0x30:
            //gain_tmp0=8*Reg_Fgain;
            gain_tmp0=1.23;
            break;
        case 0x40:
            //gain_tmp0=8*Reg_Fgain;
            gain_tmp0=1.33;
            break;
        case 0x50:
            //gain_tmp0=8*Reg_Fgain;
            gain_tmp0=1.45;
            break;
        case 0x60:
            //gain_tmp0=8*Reg_Fgain;
            gain_tmp0=1.60;
            break;
        case 0x70:
            //gain_tmp0=8*Reg_Fgain;
            gain_tmp0=1.78;
            break;
        case 0x80:
            //gain_tmp0=8*Reg_Fgain;
            gain_tmp0=2.00;
            break;
        case 0x90:
            //gain_tmp0=8*Reg_Fgain;
            gain_tmp0=2.29;
            break;
        case 0xA0:
            //gain_tmp0=8*Reg_Fgain;
            gain_tmp0=2.66;
            break;
        case 0xB0:
            //gain_tmp0=8*Reg_Fgain;
            gain_tmp0=3.20;
            break;
        case 0xC0:
            //gain_tmp0=8*Reg_Fgain;
            gain_tmp0=4.00;
            break;
        case 0xD0:
            //gain_tmp0=8*Reg_Fgain;
            gain_tmp0=5.31;
            break;
        case 0xE0:
            //gain_tmp0=8*Reg_Fgain;
            gain_tmp0=8.03;
            break;
        case 0xE4:
            //gain_tmp0=8*Reg_Fgain;
            gain_tmp0=9.12;
            break;
        case 0xE8:
            //gain_tmp0=8*Reg_Fgain;
            gain_tmp0=10.72;
            break;
        case 0xEC:
            //gain_tmp0=8*Reg_Fgain;
            gain_tmp0=12.74;
            break;
        case 0xF0:
            //gain_tmp0=8*Reg_Fgain;
            gain_tmp0=16.03;
            break;
        default:
            gain_tmp0=0;
    }
    SENSORDB("[HM5040MIPI]exit read_HM5040MIPI_gain function \n");
    return gain_tmp0;
}  /* read_HM5040MIPI_gain */

void HM5040MIPI_camera_para_to_sensor(void)
{
    kal_uint32 i;
    SENSORDB("[HM5040MIPI]enter HM5040MIPI_camera_para_to_sensor function\n");
    for(i=0; 0xFFFFFFFF!=HM5040MIPISensorReg[i].Addr; i++)
    {
        HM5040MIPI_write_cmos_sensor(HM5040MIPISensorReg[i].Addr, HM5040MIPISensorReg[i].Para);
    }
    for(i=ENGINEER_START_ADDR; 0xFFFFFFFF!=HM5040MIPISensorReg[i].Addr; i++)
    {
        HM5040MIPI_write_cmos_sensor(HM5040MIPISensorReg[i].Addr, HM5040MIPISensorReg[i].Para);
    }
    for(i=FACTORY_START_ADDR; i<FACTORY_END_ADDR; i++)
    {
        HM5040MIPI_write_cmos_sensor(HM5040MIPISensorCCT[i].Addr, HM5040MIPISensorCCT[i].Para);
    }
    SENSORDB("[HM5040MIPI]exit HM5040MIPI_camera_para_to_sensor function\n");
}

/*************************************************************************
* FUNCTION
*    HM5040MIPI_sensor_to_camera_para
*
* DESCRIPTION
*    // update camera_para from sensor register
*
* PARAMETERS
*    None
*
* RETURNS
*    gain : sensor global gain(base: 0x40)
*
* GLOBALS AFFECTED
*
*************************************************************************/
void HM5040MIPI_sensor_to_camera_para(void)
{
    kal_uint32    i,temp_data;
    SENSORDB("[HM5040MIPI]enter HM5040MIPI_sensor_to_camera_para function\n");
    for(i=0; 0xFFFFFFFF!=HM5040MIPISensorReg[i].Addr; i++)
    {
        temp_data=HM5040MIPI_read_cmos_sensor(HM5040MIPISensorReg[i].Addr);
        spin_lock(&HM5040_drv_lock);
        HM5040MIPISensorReg[i].Para = temp_data;
        spin_unlock(&HM5040_drv_lock);
    }
    for(i=ENGINEER_START_ADDR; 0xFFFFFFFF!=HM5040MIPISensorReg[i].Addr; i++)
    {
        temp_data=HM5040MIPI_read_cmos_sensor(HM5040MIPISensorReg[i].Addr);
        spin_lock(&HM5040_drv_lock);
        HM5040MIPISensorReg[i].Para = temp_data;
        spin_unlock(&HM5040_drv_lock);
    }
    SENSORDB("[HM5040MIPI]exit HM5040MIPI_sensor_to_camera_para function\n");
}

/*************************************************************************
* FUNCTION
*    HM5040MIPI_get_sensor_group_count
*
* DESCRIPTION
*    //
*
* PARAMETERS
*    None
*
* RETURNS
*    gain : sensor global gain(base: 0x40)
*
* GLOBALS AFFECTED
*
*************************************************************************/
kal_int32  HM5040MIPI_get_sensor_group_count(void)
{
    return GROUP_TOTAL_NUMS;
}

void HM5040MIPI_get_sensor_group_info(kal_uint16 group_idx, kal_int8* group_name_ptr, kal_int32* item_count_ptr)
{
    switch (group_idx)
    {
        case PRE_GAIN:
            sprintf((char *)group_name_ptr, "CCT");
            *item_count_ptr = 2;
            break;
        case CMMCLK_CURRENT:
            sprintf((char *)group_name_ptr, "CMMCLK Current");
            *item_count_ptr = 1;
            break;
        case FRAME_RATE_LIMITATION:
            sprintf((char *)group_name_ptr, "Frame Rate Limitation");
            *item_count_ptr = 2;
            break;
        case REGISTER_EDITOR:
            sprintf((char *)group_name_ptr, "Register Editor");
            *item_count_ptr = 2;
            break;
        default:
            ASSERT(0);
    }
}

void HM5040MIPI_get_sensor_item_info(kal_uint16 group_idx,kal_uint16 item_idx, MSDK_SENSOR_ITEM_INFO_STRUCT* info_ptr)
{
}

kal_bool HM5040MIPI_set_sensor_item_info(kal_uint16 group_idx, kal_uint16 item_idx, kal_int32 ItemValue)
{
    kal_uint16 temp_addr=0, temp_para=0;
    switch (group_idx)
    {
        case PRE_GAIN:
            switch (item_idx)
            {
                case 0:
                    temp_addr = PRE_GAIN_R_INDEX;
                    break;
                case 1:
                    temp_addr = PRE_GAIN_Gr_INDEX;
                    break;
                case 2:
                    temp_addr = PRE_GAIN_Gb_INDEX;
                    break;
                case 3:
                    temp_addr = PRE_GAIN_B_INDEX;
                    break;
                case 4:
                    temp_addr = SENSOR_BASEGAIN;
                    break;
                default:
                    SENSORDB("[IMX105MIPI][Error]set_sensor_item_info error!!!\n");
            }
            temp_para = HM5040MIPIGain2Reg(ItemValue);
            spin_lock(&HM5040_drv_lock);
            HM5040MIPISensorCCT[temp_addr].Para = temp_para;
            spin_unlock(&HM5040_drv_lock);
            HM5040MIPI_write_cmos_sensor(HM5040MIPISensorCCT[temp_addr].Addr,temp_para);
            temp_para=read_HM5040MIPI_gain();
            //spin_lock(&HM5040_drv_lock);
            //HM5040MIPI_sensor_gain_base=temp_para;
            //spin_unlock(&HM5040_drv_lock);
            break;
        case CMMCLK_CURRENT:
            switch (item_idx)
            {
                case 0:
                    if(ItemValue==2)
                    {
                        spin_lock(&HM5040_drv_lock);
                        HM5040MIPISensorReg[CMMCLK_CURRENT_INDEX].Para = ISP_DRIVING_2MA;
                        spin_unlock(&HM5040_drv_lock);
                        //HM5040MIPI_set_isp_driving_current(ISP_DRIVING_2MA);
                    }
                    else if(ItemValue==3 || ItemValue==4)
                    {
                        spin_lock(&HM5040_drv_lock);
                        HM5040MIPISensorReg[CMMCLK_CURRENT_INDEX].Para = ISP_DRIVING_4MA;
                        spin_unlock(&HM5040_drv_lock);
                        //HM5040MIPI_set_isp_driving_current(ISP_DRIVING_4MA);
                    }
                    else if(ItemValue==5 || ItemValue==6)
                    {
                        spin_lock(&HM5040_drv_lock);
                        HM5040MIPISensorReg[CMMCLK_CURRENT_INDEX].Para = ISP_DRIVING_6MA;
                        spin_unlock(&HM5040_drv_lock);
                        //HM5040MIPI_set_isp_driving_current(ISP_DRIVING_6MA);
                    }
                    else
                    {
                        spin_lock(&HM5040_drv_lock);
                        HM5040MIPISensorReg[CMMCLK_CURRENT_INDEX].Para = ISP_DRIVING_8MA;
                        spin_unlock(&HM5040_drv_lock);
                        //HM5040MIPI_set_isp_driving_current(ISP_DRIVING_8MA);
                    }
                    break;
                default:
                    ASSERT(0);
            }
            break;
        case FRAME_RATE_LIMITATION:
            ASSERT(0);
            break;
        case REGISTER_EDITOR:
            switch (item_idx)
            {
                case 0:
                    spin_lock(&HM5040_drv_lock);
                    HM5040MIPI_FAC_SENSOR_REG=ItemValue;
                    spin_unlock(&HM5040_drv_lock);
                    break;
                case 1:
                    HM5040MIPI_write_cmos_sensor(HM5040MIPI_FAC_SENSOR_REG,ItemValue);
                    break;
                default:
                    ASSERT(0);
            }
            break;
        default:
            ASSERT(0);
    }
    return KAL_TRUE;
}

static void HM5040MIPI_SetDummy(const kal_uint16 iPixels, const kal_uint16 iLines)
{
    //kal_uint32 frame_length = 0, line_length = 0,dummy_tmp0=0,dummy_tmp1=0;
    //SENSORDB("[HM5040MIPI]enter HM5040MIPI_SetDummy function,iPixels=%d,iLines=%d\n",iPixels,iLines);
    SENSORDB("[S], iPixels=%d, iLines=%d" ,iPixels, iLines);
    if(HM5040MIPI_sensor.pv_mode == KAL_TRUE)
    {
        SENSORDB("pv_mode");
        spin_lock(&HM5040_drv_lock);
        HM5040MIPI_sensor.pv_dummy_pixels =  HM5040MIPI_PV_DUMMY_PIXELS+iPixels;
        HM5040MIPI_sensor.pv_dummy_lines  =  HM5040MIPI_PV_DUMMY_LINES + iLines;
        HM5040MIPI_sensor.pv_line_length  =  HM5040MIPI_PV_FULL_PIXELS + iPixels;
        HM5040MIPI_sensor.pv_frame_length =  HM5040MIPI_PV_FULL_LINES + iLines;
        HM5040MIPI_sensor.frame_height    =  HM5040MIPI_sensor.pv_frame_length;
        HM5040MIPI_sensor.line_length     =  HM5040MIPI_sensor.pv_line_length;
        spin_unlock(&HM5040_drv_lock);
        //line_length = HM5040MIPI_sensor.pv_dummy_pixels-164;
        //frame_length = HM5040MIPI_sensor.pv_dummy_lines-97;
    }
    else if(HM5040MIPI_sensor.video_mode== KAL_TRUE)
    {
        SENSORDB("video_mode");
        spin_lock(&HM5040_drv_lock);
        HM5040MIPI_sensor.video_dummy_pixels = HM5040MIPI_VIDEO_DUMMY_PIXELS + iPixels;
        HM5040MIPI_sensor.video_dummy_lines  = HM5040MIPI_VIDEO_DUMMY_LINES + iLines;
        HM5040MIPI_sensor.video_line_length  = HM5040MIPI_VIDEO_FULL_PIXELS + iPixels;
        HM5040MIPI_sensor.video_frame_length = HM5040MIPI_VIDEO_FULL_LINES + iLines;
        HM5040MIPI_sensor.frame_height       = HM5040MIPI_sensor.video_frame_length;
        HM5040MIPI_sensor.line_length        = HM5040MIPI_sensor.video_line_length;
        spin_unlock(&HM5040_drv_lock);
        //line_length = HM5040MIPI_sensor.video_dummy_pixels-164;
        //frame_length = HM5040MIPI_sensor.video_dummy_lines-97;
    }
    else if(HM5040MIPI_sensor.capture_mode== KAL_TRUE) 
    {
        SENSORDB("capture_mode");
        spin_lock(&HM5040_drv_lock);
        HM5040MIPI_sensor.cp_dummy_pixels = HM5040MIPI_FULL_DUMMY_PIXELS + iPixels;;
        HM5040MIPI_sensor.cp_dummy_lines  = HM5040MIPI_FULL_DUMMY_LINES + iLines;
        HM5040MIPI_sensor.cp_line_length  = HM5040MIPI_CAP_FULL_PIXELS + iPixels;
        HM5040MIPI_sensor.cp_frame_length = HM5040MIPI_CAP_FULL_LINES + iLines;
        HM5040MIPI_sensor.frame_height    = HM5040MIPI_sensor.cp_frame_length;
        HM5040MIPI_sensor.line_length     = HM5040MIPI_sensor.cp_line_length;
        spin_unlock(&HM5040_drv_lock);
        //line_length = HM5040MIPI_sensor.cp_dummy_pixels-164;
        //frame_length = HM5040MIPI_sensor.cp_dummy_lines-97;
    }
    else
    {
        SENSORDB("sensor mode error");
    }
    //dummy_tmp0=line_length%16;
    //dummy_tmp1=line_length/16;
    //HM5040MIPI_write_cmos_sensor(0x0010, (frame_length >>8) & 0xFF);
    //HM5040MIPI_write_cmos_sensor(0x0011, frame_length & 0xFF);
    //HM5040MIPI_write_cmos_sensor(0x0012, (dummy_tmp0 & 0xFF));
    //HM5040MIPI_write_cmos_sensor(0x0013, (dummy_tmp1 & 0xFF));
    //SENSORDB("[HM5040MIPI]exit HM5040MIPI_SetDummy function\n");
    SENSORDB("[E]");
}   /*  HM5040MIPI_SetDummy */

static void HM5040MIPI_Sensor_Init(void)
{
    //SENSORDB("[HM5040MIPI]enter HM5040MIPI_Sensor_Init function\n");
    SENSORDB("[S]");
#if 0
    HM5040MIPI_write_cmos_sensor(0x0103,0x01);
    HM5040MIPI_write_cmos_sensor(0x0100,0x00);
    HM5040MIPI_write_cmos_sensor(0x3002,0x32);
    HM5040MIPI_write_cmos_sensor(0x3016,0x46);
    HM5040MIPI_write_cmos_sensor(0x3017,0x29);
    HM5040MIPI_write_cmos_sensor(0x3003,0x03);
    HM5040MIPI_write_cmos_sensor(0x3045,0x03);
    HM5040MIPI_write_cmos_sensor(0xFBD7,0x4C);
    HM5040MIPI_write_cmos_sensor(0xFBD8,0x89);
    HM5040MIPI_write_cmos_sensor(0xFBD9,0x40);
    HM5040MIPI_write_cmos_sensor(0xFBDA,0xE1);
    HM5040MIPI_write_cmos_sensor(0xFBDB,0x4C);
    HM5040MIPI_write_cmos_sensor(0xFBDC,0x73);
    HM5040MIPI_write_cmos_sensor(0xFBDD,0x40);
    HM5040MIPI_write_cmos_sensor(0xFBDE,0xD9);
    HM5040MIPI_write_cmos_sensor(0xFBDF,0x4C);
    HM5040MIPI_write_cmos_sensor(0xFBE0,0x74);
    HM5040MIPI_write_cmos_sensor(0xFBE1,0x40);
    HM5040MIPI_write_cmos_sensor(0xFBE2,0xD9);
    HM5040MIPI_write_cmos_sensor(0xFBE3,0x4C);
    HM5040MIPI_write_cmos_sensor(0xFBE4,0x87);
    HM5040MIPI_write_cmos_sensor(0xFBE5,0x40);
    HM5040MIPI_write_cmos_sensor(0xFBE6,0xD3);
    HM5040MIPI_write_cmos_sensor(0xFBE7,0x4C);
    HM5040MIPI_write_cmos_sensor(0xFBE8,0xB1);
    HM5040MIPI_write_cmos_sensor(0xFBE9,0x40);
    HM5040MIPI_write_cmos_sensor(0xFBEA,0xC7);
    HM5040MIPI_write_cmos_sensor(0xFBEB,0x4C);
    HM5040MIPI_write_cmos_sensor(0xFBEC,0xCC);
    HM5040MIPI_write_cmos_sensor(0xFBED,0x41);
    HM5040MIPI_write_cmos_sensor(0xFBEE,0x13);
    HM5040MIPI_write_cmos_sensor(0xFBEF,0x4C);
    HM5040MIPI_write_cmos_sensor(0xFBF0,0xB9);
    HM5040MIPI_write_cmos_sensor(0xFBF1,0x41);
    HM5040MIPI_write_cmos_sensor(0xFBF2,0x11);
    HM5040MIPI_write_cmos_sensor(0xFBF3,0x4C);
    HM5040MIPI_write_cmos_sensor(0xFBF4,0xB9);
    HM5040MIPI_write_cmos_sensor(0xFBF5,0x41);
    HM5040MIPI_write_cmos_sensor(0xFBF6,0x12);
    HM5040MIPI_write_cmos_sensor(0xFBF7,0x4C);
    HM5040MIPI_write_cmos_sensor(0xFBF8,0xBA);
    HM5040MIPI_write_cmos_sensor(0xFBF9,0x41);
    HM5040MIPI_write_cmos_sensor(0xFBFA,0x40);
    HM5040MIPI_write_cmos_sensor(0xFBFB,0x4C);
    HM5040MIPI_write_cmos_sensor(0xFBFC,0xD3);
    HM5040MIPI_write_cmos_sensor(0xFBFD,0x41);
    HM5040MIPI_write_cmos_sensor(0xFBFE,0x44);
    HM5040MIPI_write_cmos_sensor(0xFB00,0x51);
    HM5040MIPI_write_cmos_sensor(0xF800,0xC0);
    HM5040MIPI_write_cmos_sensor(0xF801,0x24);
    HM5040MIPI_write_cmos_sensor(0xF802,0x7C);
    HM5040MIPI_write_cmos_sensor(0xF803,0xFB);
    HM5040MIPI_write_cmos_sensor(0xF804,0x7D);
    HM5040MIPI_write_cmos_sensor(0xF805,0xC7);
    HM5040MIPI_write_cmos_sensor(0xF806,0x7B);
    HM5040MIPI_write_cmos_sensor(0xF807,0x10);
    HM5040MIPI_write_cmos_sensor(0xF808,0x7F);
    HM5040MIPI_write_cmos_sensor(0xF809,0x72);
    HM5040MIPI_write_cmos_sensor(0xF80A,0x7E);
    HM5040MIPI_write_cmos_sensor(0xF80B,0x30);
    HM5040MIPI_write_cmos_sensor(0xF80C,0x12);
    HM5040MIPI_write_cmos_sensor(0xF80D,0x09);
    HM5040MIPI_write_cmos_sensor(0xF80E,0x47);
    HM5040MIPI_write_cmos_sensor(0xF80F,0xD0);
    HM5040MIPI_write_cmos_sensor(0xF810,0x24);
    HM5040MIPI_write_cmos_sensor(0xF811,0x90);
    HM5040MIPI_write_cmos_sensor(0xF812,0x02);
    HM5040MIPI_write_cmos_sensor(0xF813,0x05);
    HM5040MIPI_write_cmos_sensor(0xF814,0xE0);
    HM5040MIPI_write_cmos_sensor(0xF815,0xF5);
    HM5040MIPI_write_cmos_sensor(0xF816,0x77);
    HM5040MIPI_write_cmos_sensor(0xF817,0xE5);
    HM5040MIPI_write_cmos_sensor(0xF818,0x77);
    HM5040MIPI_write_cmos_sensor(0xF819,0xC3);
    HM5040MIPI_write_cmos_sensor(0xF81A,0x94);
    HM5040MIPI_write_cmos_sensor(0xF81B,0x80);
    HM5040MIPI_write_cmos_sensor(0xF81C,0x50);
    HM5040MIPI_write_cmos_sensor(0xF81D,0x08);
    HM5040MIPI_write_cmos_sensor(0xF81E,0x75);
    HM5040MIPI_write_cmos_sensor(0xF81F,0x7A);
    HM5040MIPI_write_cmos_sensor(0xF820,0xFB);
    HM5040MIPI_write_cmos_sensor(0xF821,0x75);
    HM5040MIPI_write_cmos_sensor(0xF822,0x7B);
    HM5040MIPI_write_cmos_sensor(0xF823,0xD7);
    HM5040MIPI_write_cmos_sensor(0xF824,0x80);
    HM5040MIPI_write_cmos_sensor(0xF825,0x33);
    HM5040MIPI_write_cmos_sensor(0xF826,0xE5);
    HM5040MIPI_write_cmos_sensor(0xF827,0x77);
    HM5040MIPI_write_cmos_sensor(0xF828,0xC3);
    HM5040MIPI_write_cmos_sensor(0xF829,0x94);
    HM5040MIPI_write_cmos_sensor(0xF82A,0xC0);
    HM5040MIPI_write_cmos_sensor(0xF82B,0x50);
    HM5040MIPI_write_cmos_sensor(0xF82C,0x08);
    HM5040MIPI_write_cmos_sensor(0xF82D,0x75);
    HM5040MIPI_write_cmos_sensor(0xF82E,0x7A);
    HM5040MIPI_write_cmos_sensor(0xF82F,0xFB);
    HM5040MIPI_write_cmos_sensor(0xF830,0x75);
    HM5040MIPI_write_cmos_sensor(0xF831,0x7B);
    HM5040MIPI_write_cmos_sensor(0xF832,0xDB);
    HM5040MIPI_write_cmos_sensor(0xF833,0x80);
    HM5040MIPI_write_cmos_sensor(0xF834,0x24);
    HM5040MIPI_write_cmos_sensor(0xF835,0xE5);
    HM5040MIPI_write_cmos_sensor(0xF836,0x77);
    HM5040MIPI_write_cmos_sensor(0xF837,0xC3);
    HM5040MIPI_write_cmos_sensor(0xF838,0x94);
    HM5040MIPI_write_cmos_sensor(0xF839,0xE0);
    HM5040MIPI_write_cmos_sensor(0xF83A,0x50);
    HM5040MIPI_write_cmos_sensor(0xF83B,0x08);
    HM5040MIPI_write_cmos_sensor(0xF83C,0x75);
    HM5040MIPI_write_cmos_sensor(0xF83D,0x7A);
    HM5040MIPI_write_cmos_sensor(0xF83E,0xFB);
    HM5040MIPI_write_cmos_sensor(0xF83F,0x75);
    HM5040MIPI_write_cmos_sensor(0xF840,0x7B);
    HM5040MIPI_write_cmos_sensor(0xF841,0xDF);
    HM5040MIPI_write_cmos_sensor(0xF842,0x80);
    HM5040MIPI_write_cmos_sensor(0xF843,0x15);
    HM5040MIPI_write_cmos_sensor(0xF844,0xE5);
    HM5040MIPI_write_cmos_sensor(0xF845,0x77);
    HM5040MIPI_write_cmos_sensor(0xF846,0xC3);
    HM5040MIPI_write_cmos_sensor(0xF847,0x94);
    HM5040MIPI_write_cmos_sensor(0xF848,0xF0);
    HM5040MIPI_write_cmos_sensor(0xF849,0x50);
    HM5040MIPI_write_cmos_sensor(0xF84A,0x08);
    HM5040MIPI_write_cmos_sensor(0xF84B,0x75);
    HM5040MIPI_write_cmos_sensor(0xF84C,0x7A);
    HM5040MIPI_write_cmos_sensor(0xF84D,0xFB);
    HM5040MIPI_write_cmos_sensor(0xF84E,0x75);
    HM5040MIPI_write_cmos_sensor(0xF84F,0x7B);
    HM5040MIPI_write_cmos_sensor(0xF850,0xE3);
    HM5040MIPI_write_cmos_sensor(0xF851,0x80);
    HM5040MIPI_write_cmos_sensor(0xF852,0x06);
    HM5040MIPI_write_cmos_sensor(0xF853,0x75);
    HM5040MIPI_write_cmos_sensor(0xF854,0x7A);
    HM5040MIPI_write_cmos_sensor(0xF855,0xFB);
    HM5040MIPI_write_cmos_sensor(0xF856,0x75);
    HM5040MIPI_write_cmos_sensor(0xF857,0x7B);
    HM5040MIPI_write_cmos_sensor(0xF858,0xE7);
    HM5040MIPI_write_cmos_sensor(0xF859,0xE5);
    HM5040MIPI_write_cmos_sensor(0xF85A,0x55);
    HM5040MIPI_write_cmos_sensor(0xF85B,0x7F);
    HM5040MIPI_write_cmos_sensor(0xF85C,0x00);
    HM5040MIPI_write_cmos_sensor(0xF85D,0xB4);
    HM5040MIPI_write_cmos_sensor(0xF85E,0x22);
    HM5040MIPI_write_cmos_sensor(0xF85F,0x02);
    HM5040MIPI_write_cmos_sensor(0xF860,0x7F);
    HM5040MIPI_write_cmos_sensor(0xF861,0x01);
    HM5040MIPI_write_cmos_sensor(0xF862,0xE5);
    HM5040MIPI_write_cmos_sensor(0xF863,0x53);
    HM5040MIPI_write_cmos_sensor(0xF864,0x5F);
    HM5040MIPI_write_cmos_sensor(0xF865,0x60);
    HM5040MIPI_write_cmos_sensor(0xF866,0x05);
    HM5040MIPI_write_cmos_sensor(0xF867,0x74);
    HM5040MIPI_write_cmos_sensor(0xF868,0x14);
    HM5040MIPI_write_cmos_sensor(0xF869,0x12);
    HM5040MIPI_write_cmos_sensor(0xF86A,0xFA);
    HM5040MIPI_write_cmos_sensor(0xF86B,0x4C);
    HM5040MIPI_write_cmos_sensor(0xF86C,0x75);
    HM5040MIPI_write_cmos_sensor(0xF86D,0x7C);
    HM5040MIPI_write_cmos_sensor(0xF86E,0xFB);
    HM5040MIPI_write_cmos_sensor(0xF86F,0x75);
    HM5040MIPI_write_cmos_sensor(0xF870,0x7D);
    HM5040MIPI_write_cmos_sensor(0xF871,0xC7);
    HM5040MIPI_write_cmos_sensor(0xF872,0x75);
    HM5040MIPI_write_cmos_sensor(0xF873,0x7E);
    HM5040MIPI_write_cmos_sensor(0xF874,0x30);
    HM5040MIPI_write_cmos_sensor(0xF875,0x75);
    HM5040MIPI_write_cmos_sensor(0xF876,0x7F);
    HM5040MIPI_write_cmos_sensor(0xF877,0x62);
    HM5040MIPI_write_cmos_sensor(0xF878,0xE4);
    HM5040MIPI_write_cmos_sensor(0xF879,0xF5);
    HM5040MIPI_write_cmos_sensor(0xF87A,0x77);
    HM5040MIPI_write_cmos_sensor(0xF87B,0xE5);
    HM5040MIPI_write_cmos_sensor(0xF87C,0x77);
    HM5040MIPI_write_cmos_sensor(0xF87D,0xC3);
    HM5040MIPI_write_cmos_sensor(0xF87E,0x94);
    HM5040MIPI_write_cmos_sensor(0xF87F,0x08);
    HM5040MIPI_write_cmos_sensor(0xF880,0x40);
    HM5040MIPI_write_cmos_sensor(0xF881,0x03);
    HM5040MIPI_write_cmos_sensor(0xF882,0x02);
    HM5040MIPI_write_cmos_sensor(0xF883,0xF9);
    HM5040MIPI_write_cmos_sensor(0xF884,0x0E);
    HM5040MIPI_write_cmos_sensor(0xF885,0x85);
    HM5040MIPI_write_cmos_sensor(0xF886,0x7D);
    HM5040MIPI_write_cmos_sensor(0xF887,0x82);
    HM5040MIPI_write_cmos_sensor(0xF888,0x85);
    HM5040MIPI_write_cmos_sensor(0xF889,0x7C);
    HM5040MIPI_write_cmos_sensor(0xF88A,0x83);
    HM5040MIPI_write_cmos_sensor(0xF88B,0xE0);
    HM5040MIPI_write_cmos_sensor(0xF88C,0xFE);
    HM5040MIPI_write_cmos_sensor(0xF88D,0xA3);
    HM5040MIPI_write_cmos_sensor(0xF88E,0xE0);
    HM5040MIPI_write_cmos_sensor(0xF88F,0xFF);
    HM5040MIPI_write_cmos_sensor(0xF890,0x12);
    HM5040MIPI_write_cmos_sensor(0xF891,0x21);
    HM5040MIPI_write_cmos_sensor(0xF892,0x22);
    HM5040MIPI_write_cmos_sensor(0xF893,0x8E);
    HM5040MIPI_write_cmos_sensor(0xF894,0x78);
    HM5040MIPI_write_cmos_sensor(0xF895,0x8F);
    HM5040MIPI_write_cmos_sensor(0xF896,0x79);
    HM5040MIPI_write_cmos_sensor(0xF897,0x12);
    HM5040MIPI_write_cmos_sensor(0xF898,0xFA);
    HM5040MIPI_write_cmos_sensor(0xF899,0x40);
    HM5040MIPI_write_cmos_sensor(0xF89A,0x12);
    HM5040MIPI_write_cmos_sensor(0xF89B,0x22);
    HM5040MIPI_write_cmos_sensor(0xF89C,0x93);
    HM5040MIPI_write_cmos_sensor(0xF89D,0x50);
    HM5040MIPI_write_cmos_sensor(0xF89E,0x07);
    HM5040MIPI_write_cmos_sensor(0xF89F,0xE4);
    HM5040MIPI_write_cmos_sensor(0xF8A0,0xF5);
    HM5040MIPI_write_cmos_sensor(0xF8A1,0x78);
    HM5040MIPI_write_cmos_sensor(0xF8A2,0xF5);
    HM5040MIPI_write_cmos_sensor(0xF8A3,0x79);
    HM5040MIPI_write_cmos_sensor(0xF8A4,0x80);
    HM5040MIPI_write_cmos_sensor(0xF8A5,0x33);
    HM5040MIPI_write_cmos_sensor(0xF8A6,0x12);
    HM5040MIPI_write_cmos_sensor(0xF8A7,0xFA);
    HM5040MIPI_write_cmos_sensor(0xF8A8,0x40);
    HM5040MIPI_write_cmos_sensor(0xF8A9,0x7B);
    HM5040MIPI_write_cmos_sensor(0xF8AA,0x01);
    HM5040MIPI_write_cmos_sensor(0xF8AB,0xAF);
    HM5040MIPI_write_cmos_sensor(0xF8AC,0x79);
    HM5040MIPI_write_cmos_sensor(0xF8AD,0xAE);
    HM5040MIPI_write_cmos_sensor(0xF8AE,0x78);
    HM5040MIPI_write_cmos_sensor(0xF8AF,0x12);
    HM5040MIPI_write_cmos_sensor(0xF8B0,0x22);
    HM5040MIPI_write_cmos_sensor(0xF8B1,0x4F);
    HM5040MIPI_write_cmos_sensor(0xF8B2,0x74);
    HM5040MIPI_write_cmos_sensor(0xF8B3,0x02);
    HM5040MIPI_write_cmos_sensor(0xF8B4,0x12);
    HM5040MIPI_write_cmos_sensor(0xF8B5,0xFA);
    HM5040MIPI_write_cmos_sensor(0xF8B6,0x4C);
    HM5040MIPI_write_cmos_sensor(0xF8B7,0x85);
    HM5040MIPI_write_cmos_sensor(0xF8B8,0x7B);
    HM5040MIPI_write_cmos_sensor(0xF8B9,0x82);
    HM5040MIPI_write_cmos_sensor(0xF8BA,0xF5);
    HM5040MIPI_write_cmos_sensor(0xF8BB,0x83);
    HM5040MIPI_write_cmos_sensor(0xF8BC,0xE0);
    HM5040MIPI_write_cmos_sensor(0xF8BD,0xFE);
    HM5040MIPI_write_cmos_sensor(0xF8BE,0xA3);
    HM5040MIPI_write_cmos_sensor(0xF8BF,0xE0);
    HM5040MIPI_write_cmos_sensor(0xF8C0,0xFF);
    HM5040MIPI_write_cmos_sensor(0xF8C1,0x7D);
    HM5040MIPI_write_cmos_sensor(0xF8C2,0x03);
    HM5040MIPI_write_cmos_sensor(0xF8C3,0x12);
    HM5040MIPI_write_cmos_sensor(0xF8C4,0x17);
    HM5040MIPI_write_cmos_sensor(0xF8C5,0xD8);
    HM5040MIPI_write_cmos_sensor(0xF8C6,0x12);
    HM5040MIPI_write_cmos_sensor(0xF8C7,0x1B);
    HM5040MIPI_write_cmos_sensor(0xF8C8,0x9B);
    HM5040MIPI_write_cmos_sensor(0xF8C9,0x8E);
    HM5040MIPI_write_cmos_sensor(0xF8CA,0x78);
    HM5040MIPI_write_cmos_sensor(0xF8CB,0x8F);
    HM5040MIPI_write_cmos_sensor(0xF8CC,0x79);
    HM5040MIPI_write_cmos_sensor(0xF8CD,0x74);
    HM5040MIPI_write_cmos_sensor(0xF8CE,0xFE);
    HM5040MIPI_write_cmos_sensor(0xF8CF,0x25);
    HM5040MIPI_write_cmos_sensor(0xF8D0,0x7B);
    HM5040MIPI_write_cmos_sensor(0xF8D1,0xF5);
    HM5040MIPI_write_cmos_sensor(0xF8D2,0x7B);
    HM5040MIPI_write_cmos_sensor(0xF8D3,0x74);
    HM5040MIPI_write_cmos_sensor(0xF8D4,0xFF);
    HM5040MIPI_write_cmos_sensor(0xF8D5,0x35);
    HM5040MIPI_write_cmos_sensor(0xF8D6,0x7A);
    HM5040MIPI_write_cmos_sensor(0xF8D7,0xF5);
    HM5040MIPI_write_cmos_sensor(0xF8D8,0x7A);
    HM5040MIPI_write_cmos_sensor(0xF8D9,0x78);
    HM5040MIPI_write_cmos_sensor(0xF8DA,0x24);
    HM5040MIPI_write_cmos_sensor(0xF8DB,0xE6);
    HM5040MIPI_write_cmos_sensor(0xF8DC,0xFF);
    HM5040MIPI_write_cmos_sensor(0xF8DD,0xC3);
    HM5040MIPI_write_cmos_sensor(0xF8DE,0x74);
    HM5040MIPI_write_cmos_sensor(0xF8DF,0x20);
    HM5040MIPI_write_cmos_sensor(0xF8E0,0x9F);
    HM5040MIPI_write_cmos_sensor(0xF8E1,0x7E);
    HM5040MIPI_write_cmos_sensor(0xF8E2,0x00);
    HM5040MIPI_write_cmos_sensor(0xF8E3,0x25);
    HM5040MIPI_write_cmos_sensor(0xF8E4,0x79);
    HM5040MIPI_write_cmos_sensor(0xF8E5,0xFF);
    HM5040MIPI_write_cmos_sensor(0xF8E6,0xEE);
    HM5040MIPI_write_cmos_sensor(0xF8E7,0x35);
    HM5040MIPI_write_cmos_sensor(0xF8E8,0x78);
    HM5040MIPI_write_cmos_sensor(0xF8E9,0x85);
    HM5040MIPI_write_cmos_sensor(0xF8EA,0x7F);
    HM5040MIPI_write_cmos_sensor(0xF8EB,0x82);
    HM5040MIPI_write_cmos_sensor(0xF8EC,0x85);
    HM5040MIPI_write_cmos_sensor(0xF8ED,0x7E);
    HM5040MIPI_write_cmos_sensor(0xF8EE,0x83);
    HM5040MIPI_write_cmos_sensor(0xF8EF,0xF0);
    HM5040MIPI_write_cmos_sensor(0xF8F0,0xA3);
    HM5040MIPI_write_cmos_sensor(0xF8F1,0xEF);
    HM5040MIPI_write_cmos_sensor(0xF8F2,0xF0);
    HM5040MIPI_write_cmos_sensor(0xF8F3,0x05);
    HM5040MIPI_write_cmos_sensor(0xF8F4,0x77);
    HM5040MIPI_write_cmos_sensor(0xF8F5,0x74);
    HM5040MIPI_write_cmos_sensor(0xF8F6,0x02);
    HM5040MIPI_write_cmos_sensor(0xF8F7,0x25);
    HM5040MIPI_write_cmos_sensor(0xF8F8,0x7D);
    HM5040MIPI_write_cmos_sensor(0xF8F9,0xF5);
    HM5040MIPI_write_cmos_sensor(0xF8FA,0x7D);
    HM5040MIPI_write_cmos_sensor(0xF8FB,0xE4);
    HM5040MIPI_write_cmos_sensor(0xF8FC,0x35);
    HM5040MIPI_write_cmos_sensor(0xF8FD,0x7C);
    HM5040MIPI_write_cmos_sensor(0xF8FE,0xF5);
    HM5040MIPI_write_cmos_sensor(0xF8FF,0x7C);
    HM5040MIPI_write_cmos_sensor(0xF900,0x74);
    HM5040MIPI_write_cmos_sensor(0xF901,0x02);
    HM5040MIPI_write_cmos_sensor(0xF902,0x25);
    HM5040MIPI_write_cmos_sensor(0xF903,0x7F);
    HM5040MIPI_write_cmos_sensor(0xF904,0xF5);
    HM5040MIPI_write_cmos_sensor(0xF905,0x7F);
    HM5040MIPI_write_cmos_sensor(0xF906,0xE4);
    HM5040MIPI_write_cmos_sensor(0xF907,0x35);
    HM5040MIPI_write_cmos_sensor(0xF908,0x7E);
    HM5040MIPI_write_cmos_sensor(0xF909,0xF5);
    HM5040MIPI_write_cmos_sensor(0xF90A,0x7E);
    HM5040MIPI_write_cmos_sensor(0xF90B,0x02);
    HM5040MIPI_write_cmos_sensor(0xF90C,0xF8);
    HM5040MIPI_write_cmos_sensor(0xF90D,0x7B);
    HM5040MIPI_write_cmos_sensor(0xF90E,0x22);
    HM5040MIPI_write_cmos_sensor(0xF90F,0x90);
    HM5040MIPI_write_cmos_sensor(0xF910,0x30);
    HM5040MIPI_write_cmos_sensor(0xF911,0x47);
    HM5040MIPI_write_cmos_sensor(0xF912,0x74);
    HM5040MIPI_write_cmos_sensor(0xF913,0x98);
    HM5040MIPI_write_cmos_sensor(0xF914,0xF0);
    HM5040MIPI_write_cmos_sensor(0xF915,0x90);
    HM5040MIPI_write_cmos_sensor(0xF916,0x30);
    HM5040MIPI_write_cmos_sensor(0xF917,0x36);
    HM5040MIPI_write_cmos_sensor(0xF918,0x74);
    HM5040MIPI_write_cmos_sensor(0xF919,0x1E);
    HM5040MIPI_write_cmos_sensor(0xF91A,0xF0);
    HM5040MIPI_write_cmos_sensor(0xF91B,0x90);
    HM5040MIPI_write_cmos_sensor(0xF91C,0x30);
    HM5040MIPI_write_cmos_sensor(0xF91D,0x42);
    HM5040MIPI_write_cmos_sensor(0xF91E,0x74);
    HM5040MIPI_write_cmos_sensor(0xF91F,0x24);
    HM5040MIPI_write_cmos_sensor(0xF920,0xF0);
    HM5040MIPI_write_cmos_sensor(0xF921,0xE5);
    HM5040MIPI_write_cmos_sensor(0xF922,0x53);
    HM5040MIPI_write_cmos_sensor(0xF923,0x60);
    HM5040MIPI_write_cmos_sensor(0xF924,0x42);
    HM5040MIPI_write_cmos_sensor(0xF925,0x78);
    HM5040MIPI_write_cmos_sensor(0xF926,0x2B);
    HM5040MIPI_write_cmos_sensor(0xF927,0x76);
    HM5040MIPI_write_cmos_sensor(0xF928,0x01);
    HM5040MIPI_write_cmos_sensor(0xF929,0xE5);
    HM5040MIPI_write_cmos_sensor(0xF92A,0x55);
    HM5040MIPI_write_cmos_sensor(0xF92B,0xB4);
    HM5040MIPI_write_cmos_sensor(0xF92C,0x22);
    HM5040MIPI_write_cmos_sensor(0xF92D,0x17);
    HM5040MIPI_write_cmos_sensor(0xF92E,0x90);
    HM5040MIPI_write_cmos_sensor(0xF92F,0x30);
    HM5040MIPI_write_cmos_sensor(0xF930,0x36);
    HM5040MIPI_write_cmos_sensor(0xF931,0x74);
    HM5040MIPI_write_cmos_sensor(0xF932,0x46);
    HM5040MIPI_write_cmos_sensor(0xF933,0xF0);
    HM5040MIPI_write_cmos_sensor(0xF934,0x78);
    HM5040MIPI_write_cmos_sensor(0xF935,0x28);
    HM5040MIPI_write_cmos_sensor(0xF936,0x76);
    HM5040MIPI_write_cmos_sensor(0xF937,0x31);
    HM5040MIPI_write_cmos_sensor(0xF938,0x90);
    HM5040MIPI_write_cmos_sensor(0xF939,0x30);
    HM5040MIPI_write_cmos_sensor(0xF93A,0x0E);
    HM5040MIPI_write_cmos_sensor(0xF93B,0xE0);
    HM5040MIPI_write_cmos_sensor(0xF93C,0xC3);
    HM5040MIPI_write_cmos_sensor(0xF93D,0x13);
    HM5040MIPI_write_cmos_sensor(0xF93E,0x30);
    HM5040MIPI_write_cmos_sensor(0xF93F,0xE0);
    HM5040MIPI_write_cmos_sensor(0xF940,0x04);
    HM5040MIPI_write_cmos_sensor(0xF941,0x78);
    HM5040MIPI_write_cmos_sensor(0xF942,0x26);
    HM5040MIPI_write_cmos_sensor(0xF943,0x76);
    HM5040MIPI_write_cmos_sensor(0xF944,0x40);
    HM5040MIPI_write_cmos_sensor(0xF945,0xE5);
    HM5040MIPI_write_cmos_sensor(0xF946,0x55);
    HM5040MIPI_write_cmos_sensor(0xF947,0xB4);
    HM5040MIPI_write_cmos_sensor(0xF948,0x44);
    HM5040MIPI_write_cmos_sensor(0xF949,0x21);
    HM5040MIPI_write_cmos_sensor(0xF94A,0x90);
    HM5040MIPI_write_cmos_sensor(0xF94B,0x30);
    HM5040MIPI_write_cmos_sensor(0xF94C,0x47);
    HM5040MIPI_write_cmos_sensor(0xF94D,0x74);
    HM5040MIPI_write_cmos_sensor(0xF94E,0x9A);
    HM5040MIPI_write_cmos_sensor(0xF94F,0xF0);
    HM5040MIPI_write_cmos_sensor(0xF950,0x90);
    HM5040MIPI_write_cmos_sensor(0xF951,0x30);
    HM5040MIPI_write_cmos_sensor(0xF952,0x42);
    HM5040MIPI_write_cmos_sensor(0xF953,0x74);
    HM5040MIPI_write_cmos_sensor(0xF954,0x64);
    HM5040MIPI_write_cmos_sensor(0xF955,0xF0);
    HM5040MIPI_write_cmos_sensor(0xF956,0x90);
    HM5040MIPI_write_cmos_sensor(0xF957,0x30);
    HM5040MIPI_write_cmos_sensor(0xF958,0x0E);
    HM5040MIPI_write_cmos_sensor(0xF959,0xE0);
    HM5040MIPI_write_cmos_sensor(0xF95A,0x13);
    HM5040MIPI_write_cmos_sensor(0xF95B,0x13);
    HM5040MIPI_write_cmos_sensor(0xF95C,0x54);
    HM5040MIPI_write_cmos_sensor(0xF95D,0x3F);
    HM5040MIPI_write_cmos_sensor(0xF95E,0x30);
    HM5040MIPI_write_cmos_sensor(0xF95F,0xE0);
    HM5040MIPI_write_cmos_sensor(0xF960,0x0A);
    HM5040MIPI_write_cmos_sensor(0xF961,0x78);
    HM5040MIPI_write_cmos_sensor(0xF962,0x24);
    HM5040MIPI_write_cmos_sensor(0xF963,0xE4);
    HM5040MIPI_write_cmos_sensor(0xF964,0xF6);
    HM5040MIPI_write_cmos_sensor(0xF965,0x80);
    HM5040MIPI_write_cmos_sensor(0xF966,0x04);
    HM5040MIPI_write_cmos_sensor(0xF967,0x78);
    HM5040MIPI_write_cmos_sensor(0xF968,0x2B);
    HM5040MIPI_write_cmos_sensor(0xF969,0xE4);
    HM5040MIPI_write_cmos_sensor(0xF96A,0xF6);
    HM5040MIPI_write_cmos_sensor(0xF96B,0x90);
    HM5040MIPI_write_cmos_sensor(0xF96C,0x30);
    HM5040MIPI_write_cmos_sensor(0xF96D,0x88);
    HM5040MIPI_write_cmos_sensor(0xF96E,0x02);
    HM5040MIPI_write_cmos_sensor(0xF96F,0x1D);
    HM5040MIPI_write_cmos_sensor(0xF970,0x4F);
    HM5040MIPI_write_cmos_sensor(0xF971,0x22);
    HM5040MIPI_write_cmos_sensor(0xF972,0x90);
    HM5040MIPI_write_cmos_sensor(0xF973,0x0C);
    HM5040MIPI_write_cmos_sensor(0xF974,0x1A);
    HM5040MIPI_write_cmos_sensor(0xF975,0xE0);
    HM5040MIPI_write_cmos_sensor(0xF976,0x30);
    HM5040MIPI_write_cmos_sensor(0xF977,0xE2);
    HM5040MIPI_write_cmos_sensor(0xF978,0x18);
    HM5040MIPI_write_cmos_sensor(0xF979,0x90);
    HM5040MIPI_write_cmos_sensor(0xF97A,0x33);
    HM5040MIPI_write_cmos_sensor(0xF97B,0x68);
    HM5040MIPI_write_cmos_sensor(0xF97C,0xE0);
    HM5040MIPI_write_cmos_sensor(0xF97D,0x64);
    HM5040MIPI_write_cmos_sensor(0xF97E,0x05);
    HM5040MIPI_write_cmos_sensor(0xF97F,0x70);
    HM5040MIPI_write_cmos_sensor(0xF980,0x2F);
    HM5040MIPI_write_cmos_sensor(0xF981,0x90);
    HM5040MIPI_write_cmos_sensor(0xF982,0x30);
    HM5040MIPI_write_cmos_sensor(0xF983,0x38);
    HM5040MIPI_write_cmos_sensor(0xF984,0xE0);
    HM5040MIPI_write_cmos_sensor(0xF985,0x70);
    HM5040MIPI_write_cmos_sensor(0xF986,0x02);
    HM5040MIPI_write_cmos_sensor(0xF987,0xA3);
    HM5040MIPI_write_cmos_sensor(0xF988,0xE0);
    HM5040MIPI_write_cmos_sensor(0xF989,0xC3);
    HM5040MIPI_write_cmos_sensor(0xF98A,0x70);
    HM5040MIPI_write_cmos_sensor(0xF98B,0x01);
    HM5040MIPI_write_cmos_sensor(0xF98C,0xD3);
    HM5040MIPI_write_cmos_sensor(0xF98D,0x40);
    HM5040MIPI_write_cmos_sensor(0xF98E,0x21);
    HM5040MIPI_write_cmos_sensor(0xF98F,0x80);
    HM5040MIPI_write_cmos_sensor(0xF990,0x1B);
    HM5040MIPI_write_cmos_sensor(0xF991,0x90);
    HM5040MIPI_write_cmos_sensor(0xF992,0x33);
    HM5040MIPI_write_cmos_sensor(0xF993,0x68);
    HM5040MIPI_write_cmos_sensor(0xF994,0xE0);
    HM5040MIPI_write_cmos_sensor(0xF995,0xB4);
    HM5040MIPI_write_cmos_sensor(0xF996,0x05);
    HM5040MIPI_write_cmos_sensor(0xF997,0x18);
    HM5040MIPI_write_cmos_sensor(0xF998,0xC3);
    HM5040MIPI_write_cmos_sensor(0xF999,0x90);
    HM5040MIPI_write_cmos_sensor(0xF99A,0x30);
    HM5040MIPI_write_cmos_sensor(0xF99B,0x3B);
    HM5040MIPI_write_cmos_sensor(0xF99C,0xE0);
    HM5040MIPI_write_cmos_sensor(0xF99D,0x94);
    HM5040MIPI_write_cmos_sensor(0xF99E,0x0D);
    HM5040MIPI_write_cmos_sensor(0xF99F,0x90);
    HM5040MIPI_write_cmos_sensor(0xF9A0,0x30);
    HM5040MIPI_write_cmos_sensor(0xF9A1,0x3A);
    HM5040MIPI_write_cmos_sensor(0xF9A2,0xE0);
    HM5040MIPI_write_cmos_sensor(0xF9A3,0x94);
    HM5040MIPI_write_cmos_sensor(0xF9A4,0x00);
    HM5040MIPI_write_cmos_sensor(0xF9A5,0x50);
    HM5040MIPI_write_cmos_sensor(0xF9A6,0x02);
    HM5040MIPI_write_cmos_sensor(0xF9A7,0x80);
    HM5040MIPI_write_cmos_sensor(0xF9A8,0x01);
    HM5040MIPI_write_cmos_sensor(0xF9A9,0xC3);
    HM5040MIPI_write_cmos_sensor(0xF9AA,0x40);
    HM5040MIPI_write_cmos_sensor(0xF9AB,0x04);
    HM5040MIPI_write_cmos_sensor(0xF9AC,0x75);
    HM5040MIPI_write_cmos_sensor(0xF9AD,0x10);
    HM5040MIPI_write_cmos_sensor(0xF9AE,0x01);
    HM5040MIPI_write_cmos_sensor(0xF9AF,0x22);
    HM5040MIPI_write_cmos_sensor(0xF9B0,0x02);
    HM5040MIPI_write_cmos_sensor(0xF9B1,0x16);
    HM5040MIPI_write_cmos_sensor(0xF9B2,0xE1);
    HM5040MIPI_write_cmos_sensor(0xF9B3,0x22);
    HM5040MIPI_write_cmos_sensor(0xF9B4,0x90);
    HM5040MIPI_write_cmos_sensor(0xF9B5,0xFF);
    HM5040MIPI_write_cmos_sensor(0xF9B6,0x33);
    HM5040MIPI_write_cmos_sensor(0xF9B7,0xE0);
    HM5040MIPI_write_cmos_sensor(0xF9B8,0x90);
    HM5040MIPI_write_cmos_sensor(0xF9B9,0xFF);
    HM5040MIPI_write_cmos_sensor(0xF9BA,0x34);
    HM5040MIPI_write_cmos_sensor(0xF9BB,0xE0);
    HM5040MIPI_write_cmos_sensor(0xF9BC,0x60);
    HM5040MIPI_write_cmos_sensor(0xF9BD,0x0D);
    HM5040MIPI_write_cmos_sensor(0xF9BE,0x7C);
    HM5040MIPI_write_cmos_sensor(0xF9BF,0xFB);
    HM5040MIPI_write_cmos_sensor(0xF9C0,0x7D);
    HM5040MIPI_write_cmos_sensor(0xF9C1,0xD7);
    HM5040MIPI_write_cmos_sensor(0xF9C2,0x7B);
    HM5040MIPI_write_cmos_sensor(0xF9C3,0x28);
    HM5040MIPI_write_cmos_sensor(0xF9C4,0x7F);
    HM5040MIPI_write_cmos_sensor(0xF9C5,0x34);
    HM5040MIPI_write_cmos_sensor(0xF9C6,0x7E);
    HM5040MIPI_write_cmos_sensor(0xF9C7,0xFF);
    HM5040MIPI_write_cmos_sensor(0xF9C8,0x12);
    HM5040MIPI_write_cmos_sensor(0xF9C9,0x09);
    HM5040MIPI_write_cmos_sensor(0xF9CA,0x47);
    HM5040MIPI_write_cmos_sensor(0xF9CB,0x7F);
    HM5040MIPI_write_cmos_sensor(0xF9CC,0x20);
    HM5040MIPI_write_cmos_sensor(0xF9CD,0x7E);
    HM5040MIPI_write_cmos_sensor(0xF9CE,0x01);
    HM5040MIPI_write_cmos_sensor(0xF9CF,0x7D);
    HM5040MIPI_write_cmos_sensor(0xF9D0,0x00);
    HM5040MIPI_write_cmos_sensor(0xF9D1,0x7C);
    HM5040MIPI_write_cmos_sensor(0xF9D2,0x00);
    HM5040MIPI_write_cmos_sensor(0xF9D3,0x12);
    HM5040MIPI_write_cmos_sensor(0xF9D4,0x12);
    HM5040MIPI_write_cmos_sensor(0xF9D5,0xA4);
    HM5040MIPI_write_cmos_sensor(0xF9D6,0xE4);
    HM5040MIPI_write_cmos_sensor(0xF9D7,0x90);
    HM5040MIPI_write_cmos_sensor(0xF9D8,0x3E);
    HM5040MIPI_write_cmos_sensor(0xF9D9,0x44);
    HM5040MIPI_write_cmos_sensor(0xF9DA,0xF0);
    HM5040MIPI_write_cmos_sensor(0xF9DB,0x02);
    HM5040MIPI_write_cmos_sensor(0xF9DC,0x16);
    HM5040MIPI_write_cmos_sensor(0xF9DD,0x7E);
    HM5040MIPI_write_cmos_sensor(0xF9DE,0x22);
    HM5040MIPI_write_cmos_sensor(0xF9DF,0xE5);
    HM5040MIPI_write_cmos_sensor(0xF9E0,0x44);
    HM5040MIPI_write_cmos_sensor(0xF9E1,0x60);
    HM5040MIPI_write_cmos_sensor(0xF9E2,0x10);
    HM5040MIPI_write_cmos_sensor(0xF9E3,0x90);
    HM5040MIPI_write_cmos_sensor(0xF9E4,0xF6);
    HM5040MIPI_write_cmos_sensor(0xF9E5,0x2C);
    HM5040MIPI_write_cmos_sensor(0xF9E6,0x74);
    HM5040MIPI_write_cmos_sensor(0xF9E7,0x04);
    HM5040MIPI_write_cmos_sensor(0xF9E8,0xF0);
    HM5040MIPI_write_cmos_sensor(0xF9E9,0x90);
    HM5040MIPI_write_cmos_sensor(0xF9EA,0xF6);
    HM5040MIPI_write_cmos_sensor(0xF9EB,0x34);
    HM5040MIPI_write_cmos_sensor(0xF9EC,0xF0);
    HM5040MIPI_write_cmos_sensor(0xF9ED,0x90);
    HM5040MIPI_write_cmos_sensor(0xF9EE,0xF6);
    HM5040MIPI_write_cmos_sensor(0xF9EF,0x3C);
    HM5040MIPI_write_cmos_sensor(0xF9F0,0xF0);
    HM5040MIPI_write_cmos_sensor(0xF9F1,0x80);
    HM5040MIPI_write_cmos_sensor(0xF9F2,0x0E);
    HM5040MIPI_write_cmos_sensor(0xF9F3,0x90);
    HM5040MIPI_write_cmos_sensor(0xF9F4,0xF5);
    HM5040MIPI_write_cmos_sensor(0xF9F5,0xC0);
    HM5040MIPI_write_cmos_sensor(0xF9F6,0x74);
    HM5040MIPI_write_cmos_sensor(0xF9F7,0x04);
    HM5040MIPI_write_cmos_sensor(0xF9F8,0xF0);
    HM5040MIPI_write_cmos_sensor(0xF9F9,0x90);
    HM5040MIPI_write_cmos_sensor(0xF9FA,0xF5);
    HM5040MIPI_write_cmos_sensor(0xF9FB,0xC8);
    HM5040MIPI_write_cmos_sensor(0xF9FC,0xF0);
    HM5040MIPI_write_cmos_sensor(0xF9FD,0x90);
    HM5040MIPI_write_cmos_sensor(0xF9FE,0xF5);
    HM5040MIPI_write_cmos_sensor(0xF9FF,0xD0);
    HM5040MIPI_write_cmos_sensor(0xFA00,0xF0);
    HM5040MIPI_write_cmos_sensor(0xFA01,0x90);
    HM5040MIPI_write_cmos_sensor(0xFA02,0xFB);
    HM5040MIPI_write_cmos_sensor(0xFA03,0x7F);
    HM5040MIPI_write_cmos_sensor(0xFA04,0x02);
    HM5040MIPI_write_cmos_sensor(0xFA05,0x19);
    HM5040MIPI_write_cmos_sensor(0xFA06,0x0B);
    HM5040MIPI_write_cmos_sensor(0xFA07,0x22);
    HM5040MIPI_write_cmos_sensor(0xFA08,0x90);
    HM5040MIPI_write_cmos_sensor(0xFA09,0x0C);
    HM5040MIPI_write_cmos_sensor(0xFA0A,0x1A);
    HM5040MIPI_write_cmos_sensor(0xFA0B,0xE0);
    HM5040MIPI_write_cmos_sensor(0xFA0C,0x20);
    HM5040MIPI_write_cmos_sensor(0xFA0D,0xE2);
    HM5040MIPI_write_cmos_sensor(0xFA0E,0x15);
    HM5040MIPI_write_cmos_sensor(0xFA0F,0xE4);
    HM5040MIPI_write_cmos_sensor(0xFA10,0x90);
    HM5040MIPI_write_cmos_sensor(0xFA11,0x30);
    HM5040MIPI_write_cmos_sensor(0xFA12,0xF8);
    HM5040MIPI_write_cmos_sensor(0xFA13,0xF0);
    HM5040MIPI_write_cmos_sensor(0xFA14,0xA3);
    HM5040MIPI_write_cmos_sensor(0xFA15,0xF0);
    HM5040MIPI_write_cmos_sensor(0xFA16,0x90);
    HM5040MIPI_write_cmos_sensor(0xFA17,0x30);
    HM5040MIPI_write_cmos_sensor(0xFA18,0xF1);
    HM5040MIPI_write_cmos_sensor(0xFA19,0xE0);
    HM5040MIPI_write_cmos_sensor(0xFA1A,0x44);
    HM5040MIPI_write_cmos_sensor(0xFA1B,0x08);
    HM5040MIPI_write_cmos_sensor(0xFA1C,0xF0);
    HM5040MIPI_write_cmos_sensor(0xFA1D,0x90);
    HM5040MIPI_write_cmos_sensor(0xFA1E,0x30);
    HM5040MIPI_write_cmos_sensor(0xFA1F,0xF0);
    HM5040MIPI_write_cmos_sensor(0xFA20,0xE0);
    HM5040MIPI_write_cmos_sensor(0xFA21,0x44);
    HM5040MIPI_write_cmos_sensor(0xFA22,0x08);
    HM5040MIPI_write_cmos_sensor(0xFA23,0xF0);
    HM5040MIPI_write_cmos_sensor(0xFA24,0x02);
    HM5040MIPI_write_cmos_sensor(0xFA25,0x03);
    HM5040MIPI_write_cmos_sensor(0xFA26,0xDE);
    HM5040MIPI_write_cmos_sensor(0xFA27,0x22);
    HM5040MIPI_write_cmos_sensor(0xFA28,0x90);
    HM5040MIPI_write_cmos_sensor(0xFA29,0x0C);
    HM5040MIPI_write_cmos_sensor(0xFA2A,0x1A);
    HM5040MIPI_write_cmos_sensor(0xFA2B,0xE0);
    HM5040MIPI_write_cmos_sensor(0xFA2C,0x30);
    HM5040MIPI_write_cmos_sensor(0xFA2D,0xE2);
    HM5040MIPI_write_cmos_sensor(0xFA2E,0x0D);
    HM5040MIPI_write_cmos_sensor(0xFA2F,0xE0);
    HM5040MIPI_write_cmos_sensor(0xFA30,0x20);
    HM5040MIPI_write_cmos_sensor(0xFA31,0xE0);
    HM5040MIPI_write_cmos_sensor(0xFA32,0x06);
    HM5040MIPI_write_cmos_sensor(0xFA33,0x90);
    HM5040MIPI_write_cmos_sensor(0xFA34,0xFB);
    HM5040MIPI_write_cmos_sensor(0xFA35,0x85);
    HM5040MIPI_write_cmos_sensor(0xFA36,0x74);
    HM5040MIPI_write_cmos_sensor(0xFA37,0x00);
    HM5040MIPI_write_cmos_sensor(0xFA38,0xA5);
    HM5040MIPI_write_cmos_sensor(0xFA39,0x12);
    HM5040MIPI_write_cmos_sensor(0xFA3A,0x16);
    HM5040MIPI_write_cmos_sensor(0xFA3B,0xA0);
    HM5040MIPI_write_cmos_sensor(0xFA3C,0x02);
    HM5040MIPI_write_cmos_sensor(0xFA3D,0x18);
    HM5040MIPI_write_cmos_sensor(0xFA3E,0xAC);
    HM5040MIPI_write_cmos_sensor(0xFA3F,0x22);
    HM5040MIPI_write_cmos_sensor(0xFA40,0x85);
    HM5040MIPI_write_cmos_sensor(0xFA41,0x7B);
    HM5040MIPI_write_cmos_sensor(0xFA42,0x82);
    HM5040MIPI_write_cmos_sensor(0xFA43,0x85);
    HM5040MIPI_write_cmos_sensor(0xFA44,0x7A);
    HM5040MIPI_write_cmos_sensor(0xFA45,0x83);
    HM5040MIPI_write_cmos_sensor(0xFA46,0xE0);
    HM5040MIPI_write_cmos_sensor(0xFA47,0xFC);
    HM5040MIPI_write_cmos_sensor(0xFA48,0xA3);
    HM5040MIPI_write_cmos_sensor(0xFA49,0xE0);
    HM5040MIPI_write_cmos_sensor(0xFA4A,0xFD);
    HM5040MIPI_write_cmos_sensor(0xFA4B,0x22);
    HM5040MIPI_write_cmos_sensor(0xFA4C,0x25);
    HM5040MIPI_write_cmos_sensor(0xFA4D,0x7B);
    HM5040MIPI_write_cmos_sensor(0xFA4E,0xF5);
    HM5040MIPI_write_cmos_sensor(0xFA4F,0x7B);
    HM5040MIPI_write_cmos_sensor(0xFA50,0xE4);
    HM5040MIPI_write_cmos_sensor(0xFA51,0x35);
    HM5040MIPI_write_cmos_sensor(0xFA52,0x7A);
    HM5040MIPI_write_cmos_sensor(0xFA53,0xF5);
    HM5040MIPI_write_cmos_sensor(0xFA54,0x7A);
    HM5040MIPI_write_cmos_sensor(0xFA55,0x22);
    HM5040MIPI_write_cmos_sensor(0xFA56,0xC0);
    HM5040MIPI_write_cmos_sensor(0xFA57,0xD0);
    HM5040MIPI_write_cmos_sensor(0xFA58,0x90);
    HM5040MIPI_write_cmos_sensor(0xFA59,0x35);
    HM5040MIPI_write_cmos_sensor(0xFA5A,0xB5);
    HM5040MIPI_write_cmos_sensor(0xFA5B,0xE0);
    HM5040MIPI_write_cmos_sensor(0xFA5C,0x54);
    HM5040MIPI_write_cmos_sensor(0xFA5D,0xFC);
    HM5040MIPI_write_cmos_sensor(0xFA5E,0x44);
    HM5040MIPI_write_cmos_sensor(0xFA5F,0x01);
    HM5040MIPI_write_cmos_sensor(0xFA60,0xF0);
    HM5040MIPI_write_cmos_sensor(0xFA61,0x12);
    HM5040MIPI_write_cmos_sensor(0xFA62,0x1F);
    HM5040MIPI_write_cmos_sensor(0xFA63,0x5F);
    HM5040MIPI_write_cmos_sensor(0xFA64,0xD0);
    HM5040MIPI_write_cmos_sensor(0xFA65,0xD0);
    HM5040MIPI_write_cmos_sensor(0xFA66,0x02);
    HM5040MIPI_write_cmos_sensor(0xFA67,0x0A);
    HM5040MIPI_write_cmos_sensor(0xFA68,0x16);
    HM5040MIPI_write_cmos_sensor(0xFA69,0x22);
    HM5040MIPI_write_cmos_sensor(0xFA6A,0x90);
    HM5040MIPI_write_cmos_sensor(0xFA6B,0x0C);
    HM5040MIPI_write_cmos_sensor(0xFA6C,0x1A);
    HM5040MIPI_write_cmos_sensor(0xFA6D,0xE0);
    HM5040MIPI_write_cmos_sensor(0xFA6E,0x20);
    HM5040MIPI_write_cmos_sensor(0xFA6F,0xE0);
    HM5040MIPI_write_cmos_sensor(0xFA70,0x06);
    HM5040MIPI_write_cmos_sensor(0xFA71,0x90);
    HM5040MIPI_write_cmos_sensor(0xFA72,0xFB);
    HM5040MIPI_write_cmos_sensor(0xFA73,0x85);
    HM5040MIPI_write_cmos_sensor(0xFA74,0x74);
    HM5040MIPI_write_cmos_sensor(0xFA75,0x00);
    HM5040MIPI_write_cmos_sensor(0xFA76,0xA5);
    HM5040MIPI_write_cmos_sensor(0xFA77,0xE5);
    HM5040MIPI_write_cmos_sensor(0xFA78,0x10);
    HM5040MIPI_write_cmos_sensor(0xFA79,0x02);
    HM5040MIPI_write_cmos_sensor(0xFA7A,0x1E);
    HM5040MIPI_write_cmos_sensor(0xFA7B,0x8F);
    HM5040MIPI_write_cmos_sensor(0xFA7C,0x22);
    HM5040MIPI_write_cmos_sensor(0xFA7D,0x90);
    HM5040MIPI_write_cmos_sensor(0xFA7E,0xFB);
    HM5040MIPI_write_cmos_sensor(0xFA7F,0x85);
    HM5040MIPI_write_cmos_sensor(0xFA80,0x74);
    HM5040MIPI_write_cmos_sensor(0xFA81,0x00);
    HM5040MIPI_write_cmos_sensor(0xFA82,0xA5);
    HM5040MIPI_write_cmos_sensor(0xFA83,0xE5);
    HM5040MIPI_write_cmos_sensor(0xFA84,0x1A);
    HM5040MIPI_write_cmos_sensor(0xFA85,0x60);
    HM5040MIPI_write_cmos_sensor(0xFA86,0x03);
    HM5040MIPI_write_cmos_sensor(0xFA87,0x02);
    HM5040MIPI_write_cmos_sensor(0xFA88,0x17);
    HM5040MIPI_write_cmos_sensor(0xFA89,0x47);
    HM5040MIPI_write_cmos_sensor(0xFA8A,0x22);
    HM5040MIPI_write_cmos_sensor(0xFA8B,0x90);
    HM5040MIPI_write_cmos_sensor(0xFA8C,0xFB);
    HM5040MIPI_write_cmos_sensor(0xFA8D,0x84);
    HM5040MIPI_write_cmos_sensor(0xFA8E,0x02);
    HM5040MIPI_write_cmos_sensor(0xFA8F,0x18);
    HM5040MIPI_write_cmos_sensor(0xFA90,0xD9);
    HM5040MIPI_write_cmos_sensor(0xFA91,0x22);
    HM5040MIPI_write_cmos_sensor(0xFA92,0x02);
    HM5040MIPI_write_cmos_sensor(0xFA93,0x1F);
    HM5040MIPI_write_cmos_sensor(0xFA94,0xB1);
    HM5040MIPI_write_cmos_sensor(0xFA95,0x22);
    HM5040MIPI_write_cmos_sensor(0x35D8,0x01);
    HM5040MIPI_write_cmos_sensor(0x35D9,0x0F);
    HM5040MIPI_write_cmos_sensor(0x35DA,0x01);
    HM5040MIPI_write_cmos_sensor(0x35DB,0x72);
    HM5040MIPI_write_cmos_sensor(0x35DC,0x01);
    HM5040MIPI_write_cmos_sensor(0x35DD,0xB4);
    HM5040MIPI_write_cmos_sensor(0x35DE,0x01);
    HM5040MIPI_write_cmos_sensor(0x35DF,0xDF);
    HM5040MIPI_write_cmos_sensor(0x35E0,0x02);
    HM5040MIPI_write_cmos_sensor(0x35E1,0x08);
    HM5040MIPI_write_cmos_sensor(0x35E2,0x02);
    HM5040MIPI_write_cmos_sensor(0x35E3,0x28);
    HM5040MIPI_write_cmos_sensor(0x35E4,0x02);
    HM5040MIPI_write_cmos_sensor(0x35E5,0x56);
    HM5040MIPI_write_cmos_sensor(0x35E6,0x02);
    HM5040MIPI_write_cmos_sensor(0x35E7,0x6A);
    HM5040MIPI_write_cmos_sensor(0x35E8,0x02);
    HM5040MIPI_write_cmos_sensor(0x35E9,0x7D);
    HM5040MIPI_write_cmos_sensor(0x35EA,0x02);
    HM5040MIPI_write_cmos_sensor(0x35EB,0x8B);
    HM5040MIPI_write_cmos_sensor(0x35EC,0x02);
    HM5040MIPI_write_cmos_sensor(0x35ED,0x92);
    HM5040MIPI_write_cmos_sensor(0x35EF,0x22);
    HM5040MIPI_write_cmos_sensor(0x35F1,0x23);
    HM5040MIPI_write_cmos_sensor(0x35F3,0x22);
    HM5040MIPI_write_cmos_sensor(0x35F6,0x19);
    HM5040MIPI_write_cmos_sensor(0x35F7,0x55);
    HM5040MIPI_write_cmos_sensor(0x35F8,0x1D);
    HM5040MIPI_write_cmos_sensor(0x35F9,0x4C);
    HM5040MIPI_write_cmos_sensor(0x35FA,0x16);
    HM5040MIPI_write_cmos_sensor(0x35FB,0xC7);
    HM5040MIPI_write_cmos_sensor(0x35FC,0x1A);
    HM5040MIPI_write_cmos_sensor(0x35FD,0xA0);
    HM5040MIPI_write_cmos_sensor(0x35FE,0x18);
    HM5040MIPI_write_cmos_sensor(0x35FF,0xD6);
    HM5040MIPI_write_cmos_sensor(0x3600,0x03);
    HM5040MIPI_write_cmos_sensor(0x3601,0xD4);
    HM5040MIPI_write_cmos_sensor(0x3602,0x18);
    HM5040MIPI_write_cmos_sensor(0x3603,0x8A);
    HM5040MIPI_write_cmos_sensor(0x3604,0x0A);
    HM5040MIPI_write_cmos_sensor(0x3605,0x0D);
    HM5040MIPI_write_cmos_sensor(0x3606,0x1E);
    HM5040MIPI_write_cmos_sensor(0x3607,0x8D);
    HM5040MIPI_write_cmos_sensor(0x3608,0x17);
    HM5040MIPI_write_cmos_sensor(0x3609,0x43);
    HM5040MIPI_write_cmos_sensor(0x360A,0x19);
    HM5040MIPI_write_cmos_sensor(0x360B,0x16);
    HM5040MIPI_write_cmos_sensor(0x360C,0x1F);
    HM5040MIPI_write_cmos_sensor(0x360D,0xAD);
    HM5040MIPI_write_cmos_sensor(0x360E,0x19);
    HM5040MIPI_write_cmos_sensor(0x360F,0x08);
    HM5040MIPI_write_cmos_sensor(0x3610,0x14);
    HM5040MIPI_write_cmos_sensor(0x3611,0x26);
    HM5040MIPI_write_cmos_sensor(0x3612,0x1A);
    HM5040MIPI_write_cmos_sensor(0x3613,0xB3);
    HM5040MIPI_write_cmos_sensor(0x35D2,0x7F);
    HM5040MIPI_write_cmos_sensor(0x35D3,0xFF);
    HM5040MIPI_write_cmos_sensor(0x35D4,0x70);
    HM5040MIPI_write_cmos_sensor(0x35D0,0x01);
    HM5040MIPI_write_cmos_sensor(0x3E44,0x01);
    HM5040MIPI_write_cmos_sensor(0x3570,0x01);
    HM5040MIPI_write_cmos_sensor(0x0111,0x02);//CSI2
    HM5040MIPI_write_cmos_sensor(0x0114,0x01);// 2 lane
    HM5040MIPI_write_cmos_sensor(0x2136,0x0C);
    HM5040MIPI_write_cmos_sensor(0x2137,0x00);
    HM5040MIPI_write_cmos_sensor(0x0112,0x0A);//csi_data_format 10 bit
    HM5040MIPI_write_cmos_sensor(0x0113,0x0A);//csi_data_format 10 bit
    HM5040MIPI_write_cmos_sensor(0x3016,0x46);
    HM5040MIPI_write_cmos_sensor(0x3017,0x29);
    HM5040MIPI_write_cmos_sensor(0x3003,0x03);
    HM5040MIPI_write_cmos_sensor(0x3045,0x03);
    HM5040MIPI_write_cmos_sensor(0x3047,0x98);
    HM5040MIPI_write_cmos_sensor(0x034C,0x0A);//;x_output_size
    HM5040MIPI_write_cmos_sensor(0x034D,0x28);//   2600
    HM5040MIPI_write_cmos_sensor(0x034E,0x07);//;Y_output_size
    HM5040MIPI_write_cmos_sensor(0x034F,0xA0);//   1952
    HM5040MIPI_write_cmos_sensor(0x0383,0x01);//x odd
    HM5040MIPI_write_cmos_sensor(0x0387,0x01);//y odd
    HM5040MIPI_write_cmos_sensor(0x0900,0x00);//disable binning
    HM5040MIPI_write_cmos_sensor(0x0901,0x00);//binning type
    HM5040MIPI_write_cmos_sensor(0x0902,0x00);//binning weighting
    HM5040MIPI_write_cmos_sensor(0x0344,0x00);//x start
    HM5040MIPI_write_cmos_sensor(0x0345,0x00);
    HM5040MIPI_write_cmos_sensor(0x0346,0x00);//y start
    HM5040MIPI_write_cmos_sensor(0x0347,0x00);
    HM5040MIPI_write_cmos_sensor(0x0348,0x0A);//x end
    HM5040MIPI_write_cmos_sensor(0x0349,0x27);// 2599
    HM5040MIPI_write_cmos_sensor(0x034A,0x07);//y end
    HM5040MIPI_write_cmos_sensor(0x034B,0x9F);// 1951
    HM5040MIPI_write_cmos_sensor(0x040C,0x0A);//image width after digital crop
    HM5040MIPI_write_cmos_sensor(0x040D,0x28);// HEX(A28) = 2600
    HM5040MIPI_write_cmos_sensor(0x040E,0x07);//image high after digital crop
    HM5040MIPI_write_cmos_sensor(0x040F,0xA0);// HEX(7A0) = 1952
    HM5040MIPI_write_cmos_sensor(0x0305,0x04);//pre_pll_div
    HM5040MIPI_write_cmos_sensor(0x0306,0x01);//pll_mult_dummy_hi
    HM5040MIPI_write_cmos_sensor(0x0307,0x18);//pll_mult
    HM5040MIPI_write_cmos_sensor(0x0301,0x0A);//vt_pix_clk_div
    HM5040MIPI_write_cmos_sensor(0x0309,0x0A);//op_pix_clk_div
    HM5040MIPI_write_cmos_sensor(0x0340,0x08);//frame length
    HM5040MIPI_write_cmos_sensor(0x0341,0x24);// Hex(824) = 2084
    HM5040MIPI_write_cmos_sensor(0x0202,0x08);//coarse integration time
    HM5040MIPI_write_cmos_sensor(0x0203,0x00);// 2048
    HM5040MIPI_write_cmos_sensor(0x0205,0xC0);//gain code
    HM5040MIPI_write_cmos_sensor(0x0101,0x00);//flip+mirror
    HM5040MIPI_write_cmos_sensor(0x0100,0x01);//Streaming
#endif
#if 1
    HM5040MIPI_write_cmos_sensor(0x0103,0x01); //2 1
    mdelay(50);   //20140523
    HM5040MIPI_write_cmos_sensor(0x0100,0x00); //2 1
    HM5040MIPI_write_cmos_sensor(0x3002,0x32); //2 1
    HM5040MIPI_write_cmos_sensor(0x3016,0x46); //2 1
    HM5040MIPI_write_cmos_sensor(0x3017,0x29); //2 1
    HM5040MIPI_write_cmos_sensor(0x3003,0x03); //2 1
    HM5040MIPI_write_cmos_sensor(0x3045,0x03); //2 1
    HM5040MIPI_write_cmos_sensor(0xFBD7,0x00); //2 1  // FF_SUB_G1 HI
    HM5040MIPI_write_cmos_sensor(0xFBD8,0x00); //2 1  // FF_SUB_G1 LO
    HM5040MIPI_write_cmos_sensor(0xFBD9,0x00); //2 1  // FF_DIV_G1 HI
    HM5040MIPI_write_cmos_sensor(0xFBDA,0x00); //2 1  // FF_DIV_G1 LO
    HM5040MIPI_write_cmos_sensor(0xFBDB,0x00); //2 1  // FF_SUB_G2 HI
    HM5040MIPI_write_cmos_sensor(0xFBDC,0x00); //2 1  // FF_SUB_G2 LO
    HM5040MIPI_write_cmos_sensor(0xFBDD,0x00); //2 1  // FF_DIV_G2 HI
    HM5040MIPI_write_cmos_sensor(0xFBDE,0x00); //2 1  // FF_DIV_G2 LO
    HM5040MIPI_write_cmos_sensor(0xFBDF,0x00); //2 1  // FF_SUB_G4 HI
    HM5040MIPI_write_cmos_sensor(0xFBE0,0x00); //2 1  // FF_SUB_G4 LO
    HM5040MIPI_write_cmos_sensor(0xFBE1,0x00); //2 1  // FF_DIV_G4 HI
    HM5040MIPI_write_cmos_sensor(0xFBE2,0x00); //2 1  // FF_DIV_G4 LO
    HM5040MIPI_write_cmos_sensor(0xFBE3,0x00); //2 1  // FF_SUB_G8 HI
    HM5040MIPI_write_cmos_sensor(0xFBE4,0x00); //2 1  // FF_SUB_G8 LO
    HM5040MIPI_write_cmos_sensor(0xFBE5,0x00); //2 1  // FF_DIV_G8 HI
    HM5040MIPI_write_cmos_sensor(0xFBE6,0x00); //2 1  // FF_DIV_G8 LO
    HM5040MIPI_write_cmos_sensor(0xFBE7,0x00); //2 1  // FF_SUB_G16 HI
    HM5040MIPI_write_cmos_sensor(0xFBE8,0x00); //2 1  // FF_SUB_G16 LO
    HM5040MIPI_write_cmos_sensor(0xFBE9,0x00); //2 1  // FF_DIV_G16 HI
    HM5040MIPI_write_cmos_sensor(0xFBEA,0x00); //2 1  // FF_DIV_G16 LO
    //AB2 Dark Shading Setting
    HM5040MIPI_write_cmos_sensor(0xFBEB,0x4E); //2 1  // AB_SUB_G1 HI
    HM5040MIPI_write_cmos_sensor(0xFBEC,0xC5); //2 1  // AB_SUB_G1 LO
    HM5040MIPI_write_cmos_sensor(0xFBED,0x43); //2 1  // AB_DIV_G1 HI
    HM5040MIPI_write_cmos_sensor(0xFBEE,0xDA); //2 1  // AB_DIV_G1 LO
    HM5040MIPI_write_cmos_sensor(0xFBEF,0x4E); //2 1  // AB_SUB_G2 HI
    HM5040MIPI_write_cmos_sensor(0xFBF0,0xB0); //2 1  // AB_SUB_G2 LO
    HM5040MIPI_write_cmos_sensor(0xFBF1,0x43); //2 1  // AB_DIV_G2 HI
    HM5040MIPI_write_cmos_sensor(0xFBF2,0x8E); //2 1  // AB_DIV_G2 LO
    HM5040MIPI_write_cmos_sensor(0xFBF3,0x4E); //2 1  // AB_SUB_G4 HI
    HM5040MIPI_write_cmos_sensor(0xFBF4,0xAF); //2 1  // AB_SUB_G4 LO
    HM5040MIPI_write_cmos_sensor(0xFBF5,0x43); //2 1  // AB_DIV_G4 HI
    HM5040MIPI_write_cmos_sensor(0xFBF6,0x76); //2 1  // AB_DIV_G4 LO
    HM5040MIPI_write_cmos_sensor(0xFBF7,0x4E); //2 1  // AB_SUB_G8 HI
    HM5040MIPI_write_cmos_sensor(0xFBF8,0xB7); //2 1  // AB_SUB_G8 LO
    HM5040MIPI_write_cmos_sensor(0xFBF9,0x43); //2 1  // AB_DIV_G8 HI
    HM5040MIPI_write_cmos_sensor(0xFBFA,0x80); //2 1  // AB_DIV_G8 LO
    HM5040MIPI_write_cmos_sensor(0xFBFB,0x4E); //2 1  // AB_SUB_G16 HI
    HM5040MIPI_write_cmos_sensor(0xFBFC,0xC4); //2 1  // AB_SUB_G16 LO
    HM5040MIPI_write_cmos_sensor(0xFBFD,0x43); //2 1  // AB_DIV_G16 HI
    HM5040MIPI_write_cmos_sensor(0xFBFE,0x87); //2 1  // AB_DIV_G16 LO
    HM5040MIPI_write_cmos_sensor(0xFB00,0x51); //2 1
    HM5040MIPI_write_cmos_sensor(0xF800,0xc0); //2 1
    HM5040MIPI_write_cmos_sensor(0xF801,0x24); //2 1
    HM5040MIPI_write_cmos_sensor(0xF802,0x7c); //2 1
    HM5040MIPI_write_cmos_sensor(0xF803,0xfb); //2 1
    HM5040MIPI_write_cmos_sensor(0xF804,0x7d); //2 1
    HM5040MIPI_write_cmos_sensor(0xF805,0xc7); //2 1
    HM5040MIPI_write_cmos_sensor(0xF806,0x7b); //2 1
    HM5040MIPI_write_cmos_sensor(0xF807,0x10); //2 1
    HM5040MIPI_write_cmos_sensor(0xF808,0x7f); //2 1
    HM5040MIPI_write_cmos_sensor(0xF809,0x72); //2 1
    HM5040MIPI_write_cmos_sensor(0xF80A,0x7e); //2 1
    HM5040MIPI_write_cmos_sensor(0xF80B,0x30); //2 1
    HM5040MIPI_write_cmos_sensor(0xF80C,0x12); //2 1
    HM5040MIPI_write_cmos_sensor(0xF80D,0x09); //2 1
    HM5040MIPI_write_cmos_sensor(0xF80E,0x47); //2 1
    HM5040MIPI_write_cmos_sensor(0xF80F,0xd0); //2 1
    HM5040MIPI_write_cmos_sensor(0xF810,0x24); //2 1
    HM5040MIPI_write_cmos_sensor(0xF811,0x90); //2 1
    HM5040MIPI_write_cmos_sensor(0xF812,0x02); //2 1
    HM5040MIPI_write_cmos_sensor(0xF813,0x05); //2 1
    HM5040MIPI_write_cmos_sensor(0xF814,0xe0); //2 1
    HM5040MIPI_write_cmos_sensor(0xF815,0xf5); //2 1
    HM5040MIPI_write_cmos_sensor(0xF816,0x77); //2 1
    HM5040MIPI_write_cmos_sensor(0xF817,0xe5); //2 1
    HM5040MIPI_write_cmos_sensor(0xF818,0x77); //2 1
    HM5040MIPI_write_cmos_sensor(0xF819,0xc3); //2 1
    HM5040MIPI_write_cmos_sensor(0xF81A,0x94); //2 1
    HM5040MIPI_write_cmos_sensor(0xF81B,0x80); //2 1
    HM5040MIPI_write_cmos_sensor(0xF81C,0x50); //2 1
    HM5040MIPI_write_cmos_sensor(0xF81D,0x08); //2 1
    HM5040MIPI_write_cmos_sensor(0xF81E,0x75); //2 1
    HM5040MIPI_write_cmos_sensor(0xF81F,0x7a); //2 1
    HM5040MIPI_write_cmos_sensor(0xF820,0xfb); //2 1
    HM5040MIPI_write_cmos_sensor(0xF821,0x75); //2 1
    HM5040MIPI_write_cmos_sensor(0xF822,0x7b); //2 1
    HM5040MIPI_write_cmos_sensor(0xF823,0xd7); //2 1
    HM5040MIPI_write_cmos_sensor(0xF824,0x80); //2 1
    HM5040MIPI_write_cmos_sensor(0xF825,0x33); //2 1
    HM5040MIPI_write_cmos_sensor(0xF826,0xe5); //2 1
    HM5040MIPI_write_cmos_sensor(0xF827,0x77); //2 1
    HM5040MIPI_write_cmos_sensor(0xF828,0xc3); //2 1
    HM5040MIPI_write_cmos_sensor(0xF829,0x94); //2 1
    HM5040MIPI_write_cmos_sensor(0xF82A,0xc0); //2 1
    HM5040MIPI_write_cmos_sensor(0xF82B,0x50); //2 1
    HM5040MIPI_write_cmos_sensor(0xF82C,0x08); //2 1
    HM5040MIPI_write_cmos_sensor(0xF82D,0x75); //2 1
    HM5040MIPI_write_cmos_sensor(0xF82E,0x7a); //2 1
    HM5040MIPI_write_cmos_sensor(0xF82F,0xfb); //2 1
    HM5040MIPI_write_cmos_sensor(0xF830,0x75); //2 1
    HM5040MIPI_write_cmos_sensor(0xF831,0x7b); //2 1
    HM5040MIPI_write_cmos_sensor(0xF832,0xdb); //2 1
    HM5040MIPI_write_cmos_sensor(0xF833,0x80); //2 1
    HM5040MIPI_write_cmos_sensor(0xF834,0x24); //2 1
    HM5040MIPI_write_cmos_sensor(0xF835,0xe5); //2 1
    HM5040MIPI_write_cmos_sensor(0xF836,0x77); //2 1
    HM5040MIPI_write_cmos_sensor(0xF837,0xc3); //2 1
    HM5040MIPI_write_cmos_sensor(0xF838,0x94); //2 1
    HM5040MIPI_write_cmos_sensor(0xF839,0xe0); //2 1
    HM5040MIPI_write_cmos_sensor(0xF83A,0x50); //2 1
    HM5040MIPI_write_cmos_sensor(0xF83B,0x08); //2 1
    HM5040MIPI_write_cmos_sensor(0xF83C,0x75); //2 1
    HM5040MIPI_write_cmos_sensor(0xF83D,0x7a); //2 1
    HM5040MIPI_write_cmos_sensor(0xF83E,0xfb); //2 1
    HM5040MIPI_write_cmos_sensor(0xF83F,0x75); //2 1
    HM5040MIPI_write_cmos_sensor(0xF840,0x7b); //2 1
    HM5040MIPI_write_cmos_sensor(0xF841,0xdf); //2 1
    HM5040MIPI_write_cmos_sensor(0xF842,0x80); //2 1
    HM5040MIPI_write_cmos_sensor(0xF843,0x15); //2 1
    HM5040MIPI_write_cmos_sensor(0xF844,0xe5); //2 1
    HM5040MIPI_write_cmos_sensor(0xF845,0x77); //2 1
    HM5040MIPI_write_cmos_sensor(0xF846,0xc3); //2 1
    HM5040MIPI_write_cmos_sensor(0xF847,0x94); //2 1
    HM5040MIPI_write_cmos_sensor(0xF848,0xf0); //2 1
    HM5040MIPI_write_cmos_sensor(0xF849,0x50); //2 1
    HM5040MIPI_write_cmos_sensor(0xF84A,0x08); //2 1
    HM5040MIPI_write_cmos_sensor(0xF84B,0x75); //2 1
    HM5040MIPI_write_cmos_sensor(0xF84C,0x7a); //2 1
    HM5040MIPI_write_cmos_sensor(0xF84D,0xfb); //2 1
    HM5040MIPI_write_cmos_sensor(0xF84E,0x75); //2 1
    HM5040MIPI_write_cmos_sensor(0xF84F,0x7b); //2 1
    HM5040MIPI_write_cmos_sensor(0xF850,0xe3); //2 1
    HM5040MIPI_write_cmos_sensor(0xF851,0x80); //2 1
    HM5040MIPI_write_cmos_sensor(0xF852,0x06); //2 1
    HM5040MIPI_write_cmos_sensor(0xF853,0x75); //2 1
    HM5040MIPI_write_cmos_sensor(0xF854,0x7a); //2 1
    HM5040MIPI_write_cmos_sensor(0xF855,0xfb); //2 1
    HM5040MIPI_write_cmos_sensor(0xF856,0x75); //2 1
    HM5040MIPI_write_cmos_sensor(0xF857,0x7b); //2 1
    HM5040MIPI_write_cmos_sensor(0xF858,0xe7); //2 1
    HM5040MIPI_write_cmos_sensor(0xF859,0xe5); //2 1
    HM5040MIPI_write_cmos_sensor(0xF85A,0x55); //2 1
    HM5040MIPI_write_cmos_sensor(0xF85B,0x7f); //2 1
    HM5040MIPI_write_cmos_sensor(0xF85C,0x00); //2 1
    HM5040MIPI_write_cmos_sensor(0xF85D,0xb4); //2 1
    HM5040MIPI_write_cmos_sensor(0xF85E,0x22); //2 1
    HM5040MIPI_write_cmos_sensor(0xF85F,0x02); //2 1
    HM5040MIPI_write_cmos_sensor(0xF860,0x7f); //2 1
    HM5040MIPI_write_cmos_sensor(0xF861,0x01); //2 1
    HM5040MIPI_write_cmos_sensor(0xF862,0xe5); //2 1
    HM5040MIPI_write_cmos_sensor(0xF863,0x53); //2 1
    HM5040MIPI_write_cmos_sensor(0xF864,0x5f); //2 1
    HM5040MIPI_write_cmos_sensor(0xF865,0x60); //2 1
    HM5040MIPI_write_cmos_sensor(0xF866,0x05); //2 1
    HM5040MIPI_write_cmos_sensor(0xF867,0x74); //2 1
    HM5040MIPI_write_cmos_sensor(0xF868,0x14); //2 1
    HM5040MIPI_write_cmos_sensor(0xF869,0x12); //2 1
    HM5040MIPI_write_cmos_sensor(0xF86A,0xfa); //2 1
    HM5040MIPI_write_cmos_sensor(0xF86B,0x4c); //2 1
    HM5040MIPI_write_cmos_sensor(0xF86C,0x75); //2 1
    HM5040MIPI_write_cmos_sensor(0xF86D,0x7c); //2 1
    HM5040MIPI_write_cmos_sensor(0xF86E,0xfb); //2 1
    HM5040MIPI_write_cmos_sensor(0xF86F,0x75); //2 1
    HM5040MIPI_write_cmos_sensor(0xF870,0x7d); //2 1
    HM5040MIPI_write_cmos_sensor(0xF871,0xc7); //2 1
    HM5040MIPI_write_cmos_sensor(0xF872,0x75); //2 1
    HM5040MIPI_write_cmos_sensor(0xF873,0x7e); //2 1
    HM5040MIPI_write_cmos_sensor(0xF874,0x30); //2 1
    HM5040MIPI_write_cmos_sensor(0xF875,0x75); //2 1
    HM5040MIPI_write_cmos_sensor(0xF876,0x7f); //2 1
    HM5040MIPI_write_cmos_sensor(0xF877,0x62); //2 1
    HM5040MIPI_write_cmos_sensor(0xF878,0xe4); //2 1
    HM5040MIPI_write_cmos_sensor(0xF879,0xf5); //2 1
    HM5040MIPI_write_cmos_sensor(0xF87A,0x77); //2 1
    HM5040MIPI_write_cmos_sensor(0xF87B,0xe5); //2 1
    HM5040MIPI_write_cmos_sensor(0xF87C,0x77); //2 1
    HM5040MIPI_write_cmos_sensor(0xF87D,0xc3); //2 1
    HM5040MIPI_write_cmos_sensor(0xF87E,0x94); //2 1
    HM5040MIPI_write_cmos_sensor(0xF87F,0x08); //2 1
    HM5040MIPI_write_cmos_sensor(0xF880,0x40); //2 1
    HM5040MIPI_write_cmos_sensor(0xF881,0x03); //2 1
    HM5040MIPI_write_cmos_sensor(0xF882,0x02); //2 1
    HM5040MIPI_write_cmos_sensor(0xF883,0xf9); //2 1
    HM5040MIPI_write_cmos_sensor(0xF884,0x0e); //2 1
    HM5040MIPI_write_cmos_sensor(0xF885,0x85); //2 1
    HM5040MIPI_write_cmos_sensor(0xF886,0x7d); //2 1
    HM5040MIPI_write_cmos_sensor(0xF887,0x82); //2 1
    HM5040MIPI_write_cmos_sensor(0xF888,0x85); //2 1
    HM5040MIPI_write_cmos_sensor(0xF889,0x7c); //2 1
    HM5040MIPI_write_cmos_sensor(0xF88A,0x83); //2 1
    HM5040MIPI_write_cmos_sensor(0xF88B,0xe0); //2 1
    HM5040MIPI_write_cmos_sensor(0xF88C,0xfe); //2 1
    HM5040MIPI_write_cmos_sensor(0xF88D,0xa3); //2 1
    HM5040MIPI_write_cmos_sensor(0xF88E,0xe0); //2 1
    HM5040MIPI_write_cmos_sensor(0xF88F,0xff); //2 1
    HM5040MIPI_write_cmos_sensor(0xF890,0x12); //2 1
    HM5040MIPI_write_cmos_sensor(0xF891,0x21); //2 1
    HM5040MIPI_write_cmos_sensor(0xF892,0x22); //2 1
    HM5040MIPI_write_cmos_sensor(0xF893,0x8e); //2 1
    HM5040MIPI_write_cmos_sensor(0xF894,0x78); //2 1
    HM5040MIPI_write_cmos_sensor(0xF895,0x8f); //2 1
    HM5040MIPI_write_cmos_sensor(0xF896,0x79); //2 1
    HM5040MIPI_write_cmos_sensor(0xF897,0x12); //2 1
    HM5040MIPI_write_cmos_sensor(0xF898,0xfa); //2 1
    HM5040MIPI_write_cmos_sensor(0xF899,0x40); //2 1
    HM5040MIPI_write_cmos_sensor(0xF89A,0x12); //2 1
    HM5040MIPI_write_cmos_sensor(0xF89B,0x22); //2 1
    HM5040MIPI_write_cmos_sensor(0xF89C,0x93); //2 1
    HM5040MIPI_write_cmos_sensor(0xF89D,0x50); //2 1
    HM5040MIPI_write_cmos_sensor(0xF89E,0x07); //2 1
    HM5040MIPI_write_cmos_sensor(0xF89F,0xe4); //2 1
    HM5040MIPI_write_cmos_sensor(0xF8A0,0xf5); //2 1
    HM5040MIPI_write_cmos_sensor(0xF8A1,0x78); //2 1
    HM5040MIPI_write_cmos_sensor(0xF8A2,0xf5); //2 1
    HM5040MIPI_write_cmos_sensor(0xF8A3,0x79); //2 1
    HM5040MIPI_write_cmos_sensor(0xF8A4,0x80); //2 1
    HM5040MIPI_write_cmos_sensor(0xF8A5,0x33); //2 1
    HM5040MIPI_write_cmos_sensor(0xF8A6,0x12); //2 1
    HM5040MIPI_write_cmos_sensor(0xF8A7,0xfa); //2 1
    HM5040MIPI_write_cmos_sensor(0xF8A8,0x40); //2 1
    HM5040MIPI_write_cmos_sensor(0xF8A9,0x7b); //2 1
    HM5040MIPI_write_cmos_sensor(0xF8AA,0x01); //2 1
    HM5040MIPI_write_cmos_sensor(0xF8AB,0xaf); //2 1
    HM5040MIPI_write_cmos_sensor(0xF8AC,0x79); //2 1
    HM5040MIPI_write_cmos_sensor(0xF8AD,0xae); //2 1
    HM5040MIPI_write_cmos_sensor(0xF8AE,0x78); //2 1
    HM5040MIPI_write_cmos_sensor(0xF8AF,0x12); //2 1
    HM5040MIPI_write_cmos_sensor(0xF8B0,0x22); //2 1
    HM5040MIPI_write_cmos_sensor(0xF8B1,0x4f); //2 1
    HM5040MIPI_write_cmos_sensor(0xF8B2,0x74); //2 1
    HM5040MIPI_write_cmos_sensor(0xF8B3,0x02); //2 1
    HM5040MIPI_write_cmos_sensor(0xF8B4,0x12); //2 1
    HM5040MIPI_write_cmos_sensor(0xF8B5,0xfa); //2 1
    HM5040MIPI_write_cmos_sensor(0xF8B6,0x4c); //2 1
    HM5040MIPI_write_cmos_sensor(0xF8B7,0x85); //2 1
    HM5040MIPI_write_cmos_sensor(0xF8B8,0x7b); //2 1
    HM5040MIPI_write_cmos_sensor(0xF8B9,0x82); //2 1
    HM5040MIPI_write_cmos_sensor(0xF8BA,0xf5); //2 1
    HM5040MIPI_write_cmos_sensor(0xF8BB,0x83); //2 1
    HM5040MIPI_write_cmos_sensor(0xF8BC,0xe0); //2 1
    HM5040MIPI_write_cmos_sensor(0xF8BD,0xfe); //2 1
    HM5040MIPI_write_cmos_sensor(0xF8BE,0xa3); //2 1
    HM5040MIPI_write_cmos_sensor(0xF8BF,0xe0); //2 1
    HM5040MIPI_write_cmos_sensor(0xF8C0,0xff); //2 1
    HM5040MIPI_write_cmos_sensor(0xF8C1,0x7d); //2 1
    HM5040MIPI_write_cmos_sensor(0xF8C2,0x03); //2 1
    HM5040MIPI_write_cmos_sensor(0xF8C3,0x12); //2 1
    HM5040MIPI_write_cmos_sensor(0xF8C4,0x17); //2 1
    HM5040MIPI_write_cmos_sensor(0xF8C5,0xd8); //2 1
    HM5040MIPI_write_cmos_sensor(0xF8C6,0x12); //2 1
    HM5040MIPI_write_cmos_sensor(0xF8C7,0x1b); //2 1
    HM5040MIPI_write_cmos_sensor(0xF8C8,0x9b); //2 1
    HM5040MIPI_write_cmos_sensor(0xF8C9,0x8e); //2 1
    HM5040MIPI_write_cmos_sensor(0xF8CA,0x78); //2 1
    HM5040MIPI_write_cmos_sensor(0xF8CB,0x8f); //2 1
    HM5040MIPI_write_cmos_sensor(0xF8CC,0x79); //2 1
    HM5040MIPI_write_cmos_sensor(0xF8CD,0x74); //2 1
    HM5040MIPI_write_cmos_sensor(0xF8CE,0xfe); //2 1
    HM5040MIPI_write_cmos_sensor(0xF8CF,0x25); //2 1
    HM5040MIPI_write_cmos_sensor(0xF8D0,0x7b); //2 1
    HM5040MIPI_write_cmos_sensor(0xF8D1,0xf5); //2 1
    HM5040MIPI_write_cmos_sensor(0xF8D2,0x7b); //2 1
    HM5040MIPI_write_cmos_sensor(0xF8D3,0x74); //2 1
    HM5040MIPI_write_cmos_sensor(0xF8D4,0xff); //2 1
    HM5040MIPI_write_cmos_sensor(0xF8D5,0x35); //2 1
    HM5040MIPI_write_cmos_sensor(0xF8D6,0x7a); //2 1
    HM5040MIPI_write_cmos_sensor(0xF8D7,0xf5); //2 1
    HM5040MIPI_write_cmos_sensor(0xF8D8,0x7a); //2 1
    HM5040MIPI_write_cmos_sensor(0xF8D9,0x78); //2 1
    HM5040MIPI_write_cmos_sensor(0xF8DA,0x24); //2 1
    HM5040MIPI_write_cmos_sensor(0xF8DB,0xe6); //2 1
    HM5040MIPI_write_cmos_sensor(0xF8DC,0xff); //2 1
    HM5040MIPI_write_cmos_sensor(0xF8DD,0xc3); //2 1
    HM5040MIPI_write_cmos_sensor(0xF8DE,0x74); //2 1
    HM5040MIPI_write_cmos_sensor(0xF8DF,0x20); //2 1
    HM5040MIPI_write_cmos_sensor(0xF8E0,0x9f); //2 1
    HM5040MIPI_write_cmos_sensor(0xF8E1,0x7e); //2 1
    HM5040MIPI_write_cmos_sensor(0xF8E2,0x00); //2 1
    HM5040MIPI_write_cmos_sensor(0xF8E3,0x25); //2 1
    HM5040MIPI_write_cmos_sensor(0xF8E4,0x79); //2 1
    HM5040MIPI_write_cmos_sensor(0xF8E5,0xff); //2 1
    HM5040MIPI_write_cmos_sensor(0xF8E6,0xee); //2 1
    HM5040MIPI_write_cmos_sensor(0xF8E7,0x35); //2 1
    HM5040MIPI_write_cmos_sensor(0xF8E8,0x78); //2 1
    HM5040MIPI_write_cmos_sensor(0xF8E9,0x85); //2 1
    HM5040MIPI_write_cmos_sensor(0xF8EA,0x7f); //2 1
    HM5040MIPI_write_cmos_sensor(0xF8EB,0x82); //2 1
    HM5040MIPI_write_cmos_sensor(0xF8EC,0x85); //2 1
    HM5040MIPI_write_cmos_sensor(0xF8ED,0x7e); //2 1
    HM5040MIPI_write_cmos_sensor(0xF8EE,0x83); //2 1
    HM5040MIPI_write_cmos_sensor(0xF8EF,0xf0); //2 1
    HM5040MIPI_write_cmos_sensor(0xF8F0,0xa3); //2 1
    HM5040MIPI_write_cmos_sensor(0xF8F1,0xef); //2 1
    HM5040MIPI_write_cmos_sensor(0xF8F2,0xf0); //2 1
    HM5040MIPI_write_cmos_sensor(0xF8F3,0x05); //2 1
    HM5040MIPI_write_cmos_sensor(0xF8F4,0x77); //2 1
    HM5040MIPI_write_cmos_sensor(0xF8F5,0x74); //2 1
    HM5040MIPI_write_cmos_sensor(0xF8F6,0x02); //2 1
    HM5040MIPI_write_cmos_sensor(0xF8F7,0x25); //2 1
    HM5040MIPI_write_cmos_sensor(0xF8F8,0x7d); //2 1
    HM5040MIPI_write_cmos_sensor(0xF8F9,0xf5); //2 1
    HM5040MIPI_write_cmos_sensor(0xF8FA,0x7d); //2 1
    HM5040MIPI_write_cmos_sensor(0xF8FB,0xe4); //2 1
    HM5040MIPI_write_cmos_sensor(0xF8FC,0x35); //2 1
    HM5040MIPI_write_cmos_sensor(0xF8FD,0x7c); //2 1
    HM5040MIPI_write_cmos_sensor(0xF8FE,0xf5); //2 1
    HM5040MIPI_write_cmos_sensor(0xF8FF,0x7c); //2 1
    HM5040MIPI_write_cmos_sensor(0xF900,0x74); //2 1
    HM5040MIPI_write_cmos_sensor(0xF901,0x02); //2 1
    HM5040MIPI_write_cmos_sensor(0xF902,0x25); //2 1
    HM5040MIPI_write_cmos_sensor(0xF903,0x7f); //2 1
    HM5040MIPI_write_cmos_sensor(0xF904,0xf5); //2 1
    HM5040MIPI_write_cmos_sensor(0xF905,0x7f); //2 1
    HM5040MIPI_write_cmos_sensor(0xF906,0xe4); //2 1
    HM5040MIPI_write_cmos_sensor(0xF907,0x35); //2 1
    HM5040MIPI_write_cmos_sensor(0xF908,0x7e); //2 1
    HM5040MIPI_write_cmos_sensor(0xF909,0xf5); //2 1
    HM5040MIPI_write_cmos_sensor(0xF90A,0x7e); //2 1
    HM5040MIPI_write_cmos_sensor(0xF90B,0x02); //2 1
    HM5040MIPI_write_cmos_sensor(0xF90C,0xf8); //2 1
    HM5040MIPI_write_cmos_sensor(0xF90D,0x7b); //2 1
    HM5040MIPI_write_cmos_sensor(0xF90E,0x22); //2 1
    HM5040MIPI_write_cmos_sensor(0xF90F,0x90); //2 1
    HM5040MIPI_write_cmos_sensor(0xF910,0x30); //2 1
    HM5040MIPI_write_cmos_sensor(0xF911,0x47); //2 1
    HM5040MIPI_write_cmos_sensor(0xF912,0x74); //2 1
    HM5040MIPI_write_cmos_sensor(0xF913,0x98); //2 1
    HM5040MIPI_write_cmos_sensor(0xF914,0xf0); //2 1
    HM5040MIPI_write_cmos_sensor(0xF915,0x90); //2 1
    HM5040MIPI_write_cmos_sensor(0xF916,0x30); //2 1
    HM5040MIPI_write_cmos_sensor(0xF917,0x36); //2 1
    HM5040MIPI_write_cmos_sensor(0xF918,0x74); //2 1
    HM5040MIPI_write_cmos_sensor(0xF919,0x1e); //2 1
    HM5040MIPI_write_cmos_sensor(0xF91A,0xf0); //2 1
    HM5040MIPI_write_cmos_sensor(0xF91B,0x90); //2 1
    HM5040MIPI_write_cmos_sensor(0xF91C,0x30); //2 1
    HM5040MIPI_write_cmos_sensor(0xF91D,0x42); //2 1
    HM5040MIPI_write_cmos_sensor(0xF91E,0x74); //2 1
    HM5040MIPI_write_cmos_sensor(0xF91F,0x24); //2 1
    HM5040MIPI_write_cmos_sensor(0xF920,0xf0); //2 1
    HM5040MIPI_write_cmos_sensor(0xF921,0xe5); //2 1
    HM5040MIPI_write_cmos_sensor(0xF922,0x53); //2 1
    HM5040MIPI_write_cmos_sensor(0xF923,0x60); //2 1
    HM5040MIPI_write_cmos_sensor(0xF924,0x42); //2 1
    HM5040MIPI_write_cmos_sensor(0xF925,0x78); //2 1
    HM5040MIPI_write_cmos_sensor(0xF926,0x2b); //2 1
    HM5040MIPI_write_cmos_sensor(0xF927,0x76); //2 1
    HM5040MIPI_write_cmos_sensor(0xF928,0x01); //2 1
    HM5040MIPI_write_cmos_sensor(0xF929,0xe5); //2 1
    HM5040MIPI_write_cmos_sensor(0xF92A,0x55); //2 1
    HM5040MIPI_write_cmos_sensor(0xF92B,0xb4); //2 1
    HM5040MIPI_write_cmos_sensor(0xF92C,0x22); //2 1
    HM5040MIPI_write_cmos_sensor(0xF92D,0x17); //2 1
    HM5040MIPI_write_cmos_sensor(0xF92E,0x90); //2 1
    HM5040MIPI_write_cmos_sensor(0xF92F,0x30); //2 1
    HM5040MIPI_write_cmos_sensor(0xF930,0x36); //2 1
    HM5040MIPI_write_cmos_sensor(0xF931,0x74); //2 1
    HM5040MIPI_write_cmos_sensor(0xF932,0x46); //2 1
    HM5040MIPI_write_cmos_sensor(0xF933,0xf0); //2 1
    HM5040MIPI_write_cmos_sensor(0xF934,0x78); //2 1
    HM5040MIPI_write_cmos_sensor(0xF935,0x28); //2 1
    HM5040MIPI_write_cmos_sensor(0xF936,0x76); //2 1
    HM5040MIPI_write_cmos_sensor(0xF937,0x31); //2 1
    HM5040MIPI_write_cmos_sensor(0xF938,0x90); //2 1
    HM5040MIPI_write_cmos_sensor(0xF939,0x30); //2 1
    HM5040MIPI_write_cmos_sensor(0xF93A,0x0e); //2 1
    HM5040MIPI_write_cmos_sensor(0xF93B,0xe0); //2 1
    HM5040MIPI_write_cmos_sensor(0xF93C,0xc3); //2 1
    HM5040MIPI_write_cmos_sensor(0xF93D,0x13); //2 1
    HM5040MIPI_write_cmos_sensor(0xF93E,0x30); //2 1
    HM5040MIPI_write_cmos_sensor(0xF93F,0xe0); //2 1
    HM5040MIPI_write_cmos_sensor(0xF940,0x04); //2 1
    HM5040MIPI_write_cmos_sensor(0xF941,0x78); //2 1
    HM5040MIPI_write_cmos_sensor(0xF942,0x26); //2 1
    HM5040MIPI_write_cmos_sensor(0xF943,0x76); //2 1
    HM5040MIPI_write_cmos_sensor(0xF944,0x40); //2 1
    HM5040MIPI_write_cmos_sensor(0xF945,0xe5); //2 1
    HM5040MIPI_write_cmos_sensor(0xF946,0x55); //2 1
    HM5040MIPI_write_cmos_sensor(0xF947,0xb4); //2 1
    HM5040MIPI_write_cmos_sensor(0xF948,0x44); //2 1
    HM5040MIPI_write_cmos_sensor(0xF949,0x21); //2 1
    HM5040MIPI_write_cmos_sensor(0xF94A,0x90); //2 1
    HM5040MIPI_write_cmos_sensor(0xF94B,0x30); //2 1
    HM5040MIPI_write_cmos_sensor(0xF94C,0x47); //2 1
    HM5040MIPI_write_cmos_sensor(0xF94D,0x74); //2 1
    HM5040MIPI_write_cmos_sensor(0xF94E,0x9a); //2 1
    HM5040MIPI_write_cmos_sensor(0xF94F,0xf0); //2 1
    HM5040MIPI_write_cmos_sensor(0xF950,0x90); //2 1
    HM5040MIPI_write_cmos_sensor(0xF951,0x30); //2 1
    HM5040MIPI_write_cmos_sensor(0xF952,0x42); //2 1
    HM5040MIPI_write_cmos_sensor(0xF953,0x74); //2 1
    HM5040MIPI_write_cmos_sensor(0xF954,0x64); //2 1
    HM5040MIPI_write_cmos_sensor(0xF955,0xf0); //2 1
    HM5040MIPI_write_cmos_sensor(0xF956,0x90); //2 1
    HM5040MIPI_write_cmos_sensor(0xF957,0x30); //2 1
    HM5040MIPI_write_cmos_sensor(0xF958,0x0e); //2 1
    HM5040MIPI_write_cmos_sensor(0xF959,0xe0); //2 1
    HM5040MIPI_write_cmos_sensor(0xF95A,0x13); //2 1
    HM5040MIPI_write_cmos_sensor(0xF95B,0x13); //2 1
    HM5040MIPI_write_cmos_sensor(0xF95C,0x54); //2 1
    HM5040MIPI_write_cmos_sensor(0xF95D,0x3f); //2 1
    HM5040MIPI_write_cmos_sensor(0xF95E,0x30); //2 1
    HM5040MIPI_write_cmos_sensor(0xF95F,0xe0); //2 1
    HM5040MIPI_write_cmos_sensor(0xF960,0x0a); //2 1
    HM5040MIPI_write_cmos_sensor(0xF961,0x78); //2 1
    HM5040MIPI_write_cmos_sensor(0xF962,0x24); //2 1
    HM5040MIPI_write_cmos_sensor(0xF963,0xe4); //2 1
    HM5040MIPI_write_cmos_sensor(0xF964,0xf6); //2 1
    HM5040MIPI_write_cmos_sensor(0xF965,0x80); //2 1
    HM5040MIPI_write_cmos_sensor(0xF966,0x04); //2 1
    HM5040MIPI_write_cmos_sensor(0xF967,0x78); //2 1
    HM5040MIPI_write_cmos_sensor(0xF968,0x2b); //2 1
    HM5040MIPI_write_cmos_sensor(0xF969,0xe4); //2 1
    HM5040MIPI_write_cmos_sensor(0xF96A,0xf6); //2 1
    HM5040MIPI_write_cmos_sensor(0xF96B,0x90); //2 1
    HM5040MIPI_write_cmos_sensor(0xF96C,0x30); //2 1
    HM5040MIPI_write_cmos_sensor(0xF96D,0x88); //2 1
    HM5040MIPI_write_cmos_sensor(0xF96E,0x02); //2 1
    HM5040MIPI_write_cmos_sensor(0xF96F,0x1d); //2 1
    HM5040MIPI_write_cmos_sensor(0xF970,0x4f); //2 1
    HM5040MIPI_write_cmos_sensor(0xF971,0x22); //2 1
    HM5040MIPI_write_cmos_sensor(0xF972,0x90); //2 1
    HM5040MIPI_write_cmos_sensor(0xF973,0x0c); //2 1
    HM5040MIPI_write_cmos_sensor(0xF974,0x1a); //2 1
    HM5040MIPI_write_cmos_sensor(0xF975,0xe0); //2 1
    HM5040MIPI_write_cmos_sensor(0xF976,0x30); //2 1
    HM5040MIPI_write_cmos_sensor(0xF977,0xe2); //2 1
    HM5040MIPI_write_cmos_sensor(0xF978,0x18); //2 1
    HM5040MIPI_write_cmos_sensor(0xF979,0x90); //2 1
    HM5040MIPI_write_cmos_sensor(0xF97A,0x33); //2 1
    HM5040MIPI_write_cmos_sensor(0xF97B,0x68); //2 1
    HM5040MIPI_write_cmos_sensor(0xF97C,0xe0); //2 1
    HM5040MIPI_write_cmos_sensor(0xF97D,0x64); //2 1
    HM5040MIPI_write_cmos_sensor(0xF97E,0x05); //2 1
    HM5040MIPI_write_cmos_sensor(0xF97F,0x70); //2 1
    HM5040MIPI_write_cmos_sensor(0xF980,0x2f); //2 1
    HM5040MIPI_write_cmos_sensor(0xF981,0x90); //2 1
    HM5040MIPI_write_cmos_sensor(0xF982,0x30); //2 1
    HM5040MIPI_write_cmos_sensor(0xF983,0x38); //2 1
    HM5040MIPI_write_cmos_sensor(0xF984,0xe0); //2 1
    HM5040MIPI_write_cmos_sensor(0xF985,0x70); //2 1
    HM5040MIPI_write_cmos_sensor(0xF986,0x02); //2 1
    HM5040MIPI_write_cmos_sensor(0xF987,0xa3); //2 1
    HM5040MIPI_write_cmos_sensor(0xF988,0xe0); //2 1
    HM5040MIPI_write_cmos_sensor(0xF989,0xc3); //2 1
    HM5040MIPI_write_cmos_sensor(0xF98A,0x70); //2 1
    HM5040MIPI_write_cmos_sensor(0xF98B,0x01); //2 1
    HM5040MIPI_write_cmos_sensor(0xF98C,0xd3); //2 1
    HM5040MIPI_write_cmos_sensor(0xF98D,0x40); //2 1
    HM5040MIPI_write_cmos_sensor(0xF98E,0x21); //2 1
    HM5040MIPI_write_cmos_sensor(0xF98F,0x80); //2 1
    HM5040MIPI_write_cmos_sensor(0xF990,0x1b); //2 1
    HM5040MIPI_write_cmos_sensor(0xF991,0x90); //2 1
    HM5040MIPI_write_cmos_sensor(0xF992,0x33); //2 1
    HM5040MIPI_write_cmos_sensor(0xF993,0x68); //2 1
    HM5040MIPI_write_cmos_sensor(0xF994,0xe0); //2 1
    HM5040MIPI_write_cmos_sensor(0xF995,0xb4); //2 1
    HM5040MIPI_write_cmos_sensor(0xF996,0x05); //2 1
    HM5040MIPI_write_cmos_sensor(0xF997,0x18); //2 1
    HM5040MIPI_write_cmos_sensor(0xF998,0xc3); //2 1
    HM5040MIPI_write_cmos_sensor(0xF999,0x90); //2 1
    HM5040MIPI_write_cmos_sensor(0xF99A,0x30); //2 1
    HM5040MIPI_write_cmos_sensor(0xF99B,0x3b); //2 1
    HM5040MIPI_write_cmos_sensor(0xF99C,0xe0); //2 1
    HM5040MIPI_write_cmos_sensor(0xF99D,0x94); //2 1
    HM5040MIPI_write_cmos_sensor(0xF99E,0x0d); //2 1
    HM5040MIPI_write_cmos_sensor(0xF99F,0x90); //2 1
    HM5040MIPI_write_cmos_sensor(0xF9A0,0x30); //2 1
    HM5040MIPI_write_cmos_sensor(0xF9A1,0x3a); //2 1
    HM5040MIPI_write_cmos_sensor(0xF9A2,0xe0); //2 1
    HM5040MIPI_write_cmos_sensor(0xF9A3,0x94); //2 1
    HM5040MIPI_write_cmos_sensor(0xF9A4,0x00); //2 1
    HM5040MIPI_write_cmos_sensor(0xF9A5,0x50); //2 1
    HM5040MIPI_write_cmos_sensor(0xF9A6,0x02); //2 1
    HM5040MIPI_write_cmos_sensor(0xF9A7,0x80); //2 1
    HM5040MIPI_write_cmos_sensor(0xF9A8,0x01); //2 1
    HM5040MIPI_write_cmos_sensor(0xF9A9,0xc3); //2 1
    HM5040MIPI_write_cmos_sensor(0xF9AA,0x40); //2 1
    HM5040MIPI_write_cmos_sensor(0xF9AB,0x04); //2 1
    HM5040MIPI_write_cmos_sensor(0xF9AC,0x75); //2 1
    HM5040MIPI_write_cmos_sensor(0xF9AD,0x10); //2 1
    HM5040MIPI_write_cmos_sensor(0xF9AE,0x01); //2 1
    HM5040MIPI_write_cmos_sensor(0xF9AF,0x22); //2 1
    HM5040MIPI_write_cmos_sensor(0xF9B0,0x02); //2 1
    HM5040MIPI_write_cmos_sensor(0xF9B1,0x16); //2 1
    HM5040MIPI_write_cmos_sensor(0xF9B2,0xe1); //2 1
    HM5040MIPI_write_cmos_sensor(0xF9B3,0x22); //2 1
    HM5040MIPI_write_cmos_sensor(0xF9B4,0x90); //2 1
    HM5040MIPI_write_cmos_sensor(0xF9B5,0xff); //2 1
    HM5040MIPI_write_cmos_sensor(0xF9B6,0x33); //2 1
    HM5040MIPI_write_cmos_sensor(0xF9B7,0xe0); //2 1
    HM5040MIPI_write_cmos_sensor(0xF9B8,0x90); //2 1
    HM5040MIPI_write_cmos_sensor(0xF9B9,0xff); //2 1
    HM5040MIPI_write_cmos_sensor(0xF9BA,0x34); //2 1
    HM5040MIPI_write_cmos_sensor(0xF9BB,0xe0); //2 1
    HM5040MIPI_write_cmos_sensor(0xF9BC,0x60); //2 1
    HM5040MIPI_write_cmos_sensor(0xF9BD,0x0d); //2 1
    HM5040MIPI_write_cmos_sensor(0xF9BE,0x7c); //2 1
    HM5040MIPI_write_cmos_sensor(0xF9BF,0xfb); //2 1
    HM5040MIPI_write_cmos_sensor(0xF9C0,0x7d); //2 1
    HM5040MIPI_write_cmos_sensor(0xF9C1,0xd7); //2 1
    HM5040MIPI_write_cmos_sensor(0xF9C2,0x7b); //2 1
    HM5040MIPI_write_cmos_sensor(0xF9C3,0x28); //2 1
    HM5040MIPI_write_cmos_sensor(0xF9C4,0x7f); //2 1
    HM5040MIPI_write_cmos_sensor(0xF9C5,0x34); //2 1
    HM5040MIPI_write_cmos_sensor(0xF9C6,0x7e); //2 1
    HM5040MIPI_write_cmos_sensor(0xF9C7,0xff); //2 1
    HM5040MIPI_write_cmos_sensor(0xF9C8,0x12); //2 1
    HM5040MIPI_write_cmos_sensor(0xF9C9,0x09); //2 1
    HM5040MIPI_write_cmos_sensor(0xF9CA,0x47); //2 1
    HM5040MIPI_write_cmos_sensor(0xF9CB,0x7f); //2 1
    HM5040MIPI_write_cmos_sensor(0xF9CC,0x20); //2 1
    HM5040MIPI_write_cmos_sensor(0xF9CD,0x7e); //2 1
    HM5040MIPI_write_cmos_sensor(0xF9CE,0x01); //2 1
    HM5040MIPI_write_cmos_sensor(0xF9CF,0x7d); //2 1
    HM5040MIPI_write_cmos_sensor(0xF9D0,0x00); //2 1
    HM5040MIPI_write_cmos_sensor(0xF9D1,0x7c); //2 1
    HM5040MIPI_write_cmos_sensor(0xF9D2,0x00); //2 1
    HM5040MIPI_write_cmos_sensor(0xF9D3,0x12); //2 1
    HM5040MIPI_write_cmos_sensor(0xF9D4,0x12); //2 1
    HM5040MIPI_write_cmos_sensor(0xF9D5,0xa4); //2 1
    HM5040MIPI_write_cmos_sensor(0xF9D6,0xe4); //2 1
    HM5040MIPI_write_cmos_sensor(0xF9D7,0x90); //2 1
    HM5040MIPI_write_cmos_sensor(0xF9D8,0x3e); //2 1
    HM5040MIPI_write_cmos_sensor(0xF9D9,0x44); //2 1
    HM5040MIPI_write_cmos_sensor(0xF9DA,0xf0); //2 1
    HM5040MIPI_write_cmos_sensor(0xF9DB,0x02); //2 1
    HM5040MIPI_write_cmos_sensor(0xF9DC,0x16); //2 1
    HM5040MIPI_write_cmos_sensor(0xF9DD,0x7e); //2 1
    HM5040MIPI_write_cmos_sensor(0xF9DE,0x22); //2 1
    HM5040MIPI_write_cmos_sensor(0xF9DF,0xe5); //2 1
    HM5040MIPI_write_cmos_sensor(0xF9E0,0x44); //2 1
    HM5040MIPI_write_cmos_sensor(0xF9E1,0x60); //2 1
    HM5040MIPI_write_cmos_sensor(0xF9E2,0x10); //2 1
    HM5040MIPI_write_cmos_sensor(0xF9E3,0x90); //2 1
    HM5040MIPI_write_cmos_sensor(0xF9E4,0xf6); //2 1
    HM5040MIPI_write_cmos_sensor(0xF9E5,0x2c); //2 1
    HM5040MIPI_write_cmos_sensor(0xF9E6,0x74); //2 1
    HM5040MIPI_write_cmos_sensor(0xF9E7,0x04); //2 1
    HM5040MIPI_write_cmos_sensor(0xF9E8,0xf0); //2 1
    HM5040MIPI_write_cmos_sensor(0xF9E9,0x90); //2 1
    HM5040MIPI_write_cmos_sensor(0xF9EA,0xf6); //2 1
    HM5040MIPI_write_cmos_sensor(0xF9EB,0x34); //2 1
    HM5040MIPI_write_cmos_sensor(0xF9EC,0xf0); //2 1
    HM5040MIPI_write_cmos_sensor(0xF9ED,0x90); //2 1
    HM5040MIPI_write_cmos_sensor(0xF9EE,0xf6); //2 1
    HM5040MIPI_write_cmos_sensor(0xF9EF,0x3c); //2 1
    HM5040MIPI_write_cmos_sensor(0xF9F0,0xf0); //2 1
    HM5040MIPI_write_cmos_sensor(0xF9F1,0x80); //2 1
    HM5040MIPI_write_cmos_sensor(0xF9F2,0x0e); //2 1
    HM5040MIPI_write_cmos_sensor(0xF9F3,0x90); //2 1
    HM5040MIPI_write_cmos_sensor(0xF9F4,0xf5); //2 1
    HM5040MIPI_write_cmos_sensor(0xF9F5,0xc0); //2 1
    HM5040MIPI_write_cmos_sensor(0xF9F6,0x74); //2 1
    HM5040MIPI_write_cmos_sensor(0xF9F7,0x04); //2 1
    HM5040MIPI_write_cmos_sensor(0xF9F8,0xf0); //2 1
    HM5040MIPI_write_cmos_sensor(0xF9F9,0x90); //2 1
    HM5040MIPI_write_cmos_sensor(0xF9FA,0xf5); //2 1
    HM5040MIPI_write_cmos_sensor(0xF9FB,0xc8); //2 1
    HM5040MIPI_write_cmos_sensor(0xF9FC,0xf0); //2 1
    HM5040MIPI_write_cmos_sensor(0xF9FD,0x90); //2 1
    HM5040MIPI_write_cmos_sensor(0xF9FE,0xf5); //2 1
    HM5040MIPI_write_cmos_sensor(0xF9FF,0xd0); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA00,0xf0); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA01,0x90); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA02,0xfb); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA03,0x7f); //2 1


    HM5040MIPI_write_cmos_sensor(0xFA04,0x02); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA05,0x19); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA06,0x0b); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA07,0x22); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA08,0x90); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA09,0x0c); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA0A,0x1a); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA0B,0xe0); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA0C,0x20); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA0D,0xe2); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA0E,0x15); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA0F,0xe4); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA10,0x90); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA11,0x30); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA12,0xf8); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA13,0xf0); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA14,0xa3); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA15,0xf0); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA16,0x90); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA17,0x30); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA18,0xf1); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA19,0xe0); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA1A,0x44); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA1B,0x08); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA1C,0xf0); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA1D,0x90); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA1E,0x30); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA1F,0xf0); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA20,0xe0); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA21,0x44); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA22,0x08); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA23,0xf0); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA24,0x02); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA25,0x03); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA26,0xde); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA27,0x22); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA28,0x90); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA29,0x0c); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA2A,0x1a); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA2B,0xe0); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA2C,0x30); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA2D,0xe2); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA2E,0x0d); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA2F,0xe0); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA30,0x20); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA31,0xe0); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA32,0x06); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA33,0x90); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA34,0xfb); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA35,0x85); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA36,0x74); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA37,0x00); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA38,0xa5); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA39,0x12); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA3A,0x16); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA3B,0xa0); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA3C,0x02); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA3D,0x18); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA3E,0xac); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA3F,0x22); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA40,0x85); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA41,0x7b); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA42,0x82); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA43,0x85); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA44,0x7a); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA45,0x83); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA46,0xe0); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA47,0xfc); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA48,0xa3); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA49,0xe0); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA4A,0xfd); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA4B,0x22); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA4C,0x25); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA4D,0x7b); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA4E,0xf5); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA4F,0x7b); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA50,0xe4); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA51,0x35); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA52,0x7a); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA53,0xf5); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA54,0x7a); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA55,0x22); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA56,0xc0); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA57,0xd0); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA58,0x90); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA59,0x35); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA5A,0xb5); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA5B,0xe0); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA5C,0x54); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA5D,0xfc); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA5E,0x44); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA5F,0x01); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA60,0xf0); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA61,0x12); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA62,0x1f); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA63,0x5f); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA64,0xd0); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA65,0xd0); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA66,0x02); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA67,0x0a); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA68,0x16); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA69,0x22); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA6A,0x90); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA6B,0x0c); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA6C,0x1a); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA6D,0xe0); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA6E,0x20); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA6F,0xe0); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA70,0x06); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA71,0x90); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA72,0xfb); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA73,0x85); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA74,0x74); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA75,0x00); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA76,0xa5); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA77,0xe5); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA78,0x10); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA79,0x02); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA7A,0x1e); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA7B,0x8f); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA7C,0x22); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA7D,0x90); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA7E,0xfb); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA7F,0x85); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA80,0x74); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA81,0x00); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA82,0xa5); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA83,0xe5); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA84,0x1a); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA85,0x60); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA86,0x03); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA87,0x02); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA88,0x17); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA89,0x47); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA8A,0x22); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA8B,0x90); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA8C,0xfb); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA8D,0x84); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA8E,0x02); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA8F,0x18); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA90,0xd9); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA91,0x22); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA92,0x02); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA93,0x1f); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA94,0xb1); //2 1
    HM5040MIPI_write_cmos_sensor(0xFA95,0x22); //2 1
    HM5040MIPI_write_cmos_sensor(0x35D8,0x01); //2 1
    HM5040MIPI_write_cmos_sensor(0x35D9,0x0F); //2 1
    HM5040MIPI_write_cmos_sensor(0x35DA,0x01); //2 1
    HM5040MIPI_write_cmos_sensor(0x35DB,0x72); //2 1
    HM5040MIPI_write_cmos_sensor(0x35DC,0x01); //2 1
    HM5040MIPI_write_cmos_sensor(0x35DD,0xB4); //2 1
    HM5040MIPI_write_cmos_sensor(0x35DE,0x01); //2 1
    HM5040MIPI_write_cmos_sensor(0x35DF,0xDF); //2 1
    HM5040MIPI_write_cmos_sensor(0x35E0,0x02); //2 1
    HM5040MIPI_write_cmos_sensor(0x35E1,0x08); //2 1
    HM5040MIPI_write_cmos_sensor(0x35E2,0x02); //2 1
    HM5040MIPI_write_cmos_sensor(0x35E3,0x28); //2 1
    HM5040MIPI_write_cmos_sensor(0x35E4,0x02); //2 1
    HM5040MIPI_write_cmos_sensor(0x35E5,0x56); //2 1
    HM5040MIPI_write_cmos_sensor(0x35E6,0x02); //2 1
    HM5040MIPI_write_cmos_sensor(0x35E7,0x6A); //2 1
    HM5040MIPI_write_cmos_sensor(0x35E8,0x02); //2 1
    HM5040MIPI_write_cmos_sensor(0x35E9,0x7D); //2 1
    HM5040MIPI_write_cmos_sensor(0x35EA,0x02); //2 1
    HM5040MIPI_write_cmos_sensor(0x35EB,0x8B); //2 1
    HM5040MIPI_write_cmos_sensor(0x35EC,0x02); //2 1
    HM5040MIPI_write_cmos_sensor(0x35ED,0x92); //2 1
    HM5040MIPI_write_cmos_sensor(0x35EF,0x22); //2 1
    HM5040MIPI_write_cmos_sensor(0x35F1,0x23); //2 1
    HM5040MIPI_write_cmos_sensor(0x35F3,0x22); //2 1
    HM5040MIPI_write_cmos_sensor(0x35F6,0x19); //2 1
    HM5040MIPI_write_cmos_sensor(0x35F7,0x55); //2 1
    HM5040MIPI_write_cmos_sensor(0x35F8,0x1D); //2 1
    HM5040MIPI_write_cmos_sensor(0x35F9,0x4C); //2 1
    HM5040MIPI_write_cmos_sensor(0x35FA,0x16); //2 1
    HM5040MIPI_write_cmos_sensor(0x35FB,0xC7); //2 1
    HM5040MIPI_write_cmos_sensor(0x35FC,0x1A); //2 1
    HM5040MIPI_write_cmos_sensor(0x35FD,0xA0); //2 1
    HM5040MIPI_write_cmos_sensor(0x35FE,0x18); //2 1
    HM5040MIPI_write_cmos_sensor(0x35FF,0xD6); //2 1
    HM5040MIPI_write_cmos_sensor(0x3600,0x03); //2 1
    HM5040MIPI_write_cmos_sensor(0x3601,0xD4); //2 1
    HM5040MIPI_write_cmos_sensor(0x3602,0x18); //2 1
    HM5040MIPI_write_cmos_sensor(0x3603,0x8A); //2 1
    HM5040MIPI_write_cmos_sensor(0x3604,0x0A); //2 1
    HM5040MIPI_write_cmos_sensor(0x3605,0x0D); //2 1
    HM5040MIPI_write_cmos_sensor(0x3606,0x1E); //2 1
    HM5040MIPI_write_cmos_sensor(0x3607,0x8D); //2 1
    HM5040MIPI_write_cmos_sensor(0x3608,0x17); //2 1
    HM5040MIPI_write_cmos_sensor(0x3609,0x43); //2 1
    HM5040MIPI_write_cmos_sensor(0x360A,0x19); //2 1
    HM5040MIPI_write_cmos_sensor(0x360B,0x16); //2 1
    HM5040MIPI_write_cmos_sensor(0x360C,0x1F); //2 1
    HM5040MIPI_write_cmos_sensor(0x360D,0xAD); //2 1
    HM5040MIPI_write_cmos_sensor(0x360E,0x19); //2 1
    HM5040MIPI_write_cmos_sensor(0x360F,0x08); //2 1
    HM5040MIPI_write_cmos_sensor(0x3610,0x14); //2 1
    HM5040MIPI_write_cmos_sensor(0x3611,0x26); //2 1
    HM5040MIPI_write_cmos_sensor(0x3612,0x1A); //2 1
    HM5040MIPI_write_cmos_sensor(0x3613,0xB3); //2 1
    HM5040MIPI_write_cmos_sensor(0x35D2,0x7F); //2 1
    HM5040MIPI_write_cmos_sensor(0x35D3,0xFF); //2 1
    HM5040MIPI_write_cmos_sensor(0x35D4,0x70); //2 1
    HM5040MIPI_write_cmos_sensor(0x35D0,0x01); //2 1
    HM5040MIPI_write_cmos_sensor(0x3E44,0x01); //2 1
//checked below
    HM5040MIPI_write_cmos_sensor(0x0111,0x02);//CSI2
    HM5040MIPI_write_cmos_sensor(0x0114,0x01);// 2 lane
    HM5040MIPI_write_cmos_sensor(0x2136,0x0C); //2 1
    HM5040MIPI_write_cmos_sensor(0x2137,0x00); //2 1
    HM5040MIPI_write_cmos_sensor(0x0112,0x0A);//csi_data_format 10 bit
    HM5040MIPI_write_cmos_sensor(0x0113,0x0A);//csi_data_format 10 bit
    HM5040MIPI_write_cmos_sensor(0x3016,0x46); //2 1
    HM5040MIPI_write_cmos_sensor(0x3017,0x29); //2 1
    HM5040MIPI_write_cmos_sensor(0x3003,0x03); //2 1
    HM5040MIPI_write_cmos_sensor(0x3045,0x03); //2 1
    HM5040MIPI_write_cmos_sensor(0x3047,0x98); //2 1
    //HM5040MIPI_write_cmos_sensor(0x0305,0x02);//pre_pll_div
    HM5040MIPI_write_cmos_sensor(0x0305,0x04);//pre_pll_div
    HM5040MIPI_write_cmos_sensor(0x0306,0x01);//pll_mult_dummy_hi
    HM5040MIPI_write_cmos_sensor(0x0307,0x18);//pll_mult
    HM5040MIPI_write_cmos_sensor(0x0301,0x0A);//vt_pix_clk_div
    HM5040MIPI_write_cmos_sensor(0x0309,0x0A);//op_pix_clk_div
    //HM5040MIPI_write_cmos_sensor(0x0340,0x07);//frame length
    //HM5040MIPI_write_cmos_sensor(0x0341,0xb6);// Hex(7b6) = 1974
    HM5040MIPI_write_cmos_sensor(0x0340,0x08);//frame length
    HM5040MIPI_write_cmos_sensor(0x0341,0x24);// Hex(824) = 2084
    HM5040MIPI_write_cmos_sensor(0x0344,0x00);//x start
    HM5040MIPI_write_cmos_sensor(0x0345,0x00);
    HM5040MIPI_write_cmos_sensor(0x0346,0x00);//y start
    HM5040MIPI_write_cmos_sensor(0x0347,0x00);
    HM5040MIPI_write_cmos_sensor(0x0348,0x0A);//x end
    HM5040MIPI_write_cmos_sensor(0x0349,0x27);// 2599
    HM5040MIPI_write_cmos_sensor(0x034A,0x07);//y end
    HM5040MIPI_write_cmos_sensor(0x034B,0x9F);// 1951
    HM5040MIPI_write_cmos_sensor(0x034C,0x0A);//;x_output_size
    HM5040MIPI_write_cmos_sensor(0x034D,0x28);//   2600
    HM5040MIPI_write_cmos_sensor(0x034E,0x07);//;Y_output_size
    HM5040MIPI_write_cmos_sensor(0x034F,0xA0);//   1952
    HM5040MIPI_write_cmos_sensor(0x0383,0x01);//x odd
    HM5040MIPI_write_cmos_sensor(0x0387,0x01);//y odd
    HM5040MIPI_write_cmos_sensor(0x0202,0x07);//coarse integration time
    HM5040MIPI_write_cmos_sensor(0x0203,0xad);// 1965
    HM5040MIPI_write_cmos_sensor(0x0205,0xC0);//gain code
    HM5040MIPI_write_cmos_sensor(0x0900,0x00);//disable binning
    HM5040MIPI_write_cmos_sensor(0x0901,0x00);//binning type
    HM5040MIPI_write_cmos_sensor(0x0902,0x00);//binning weighting
    HM5040MIPI_write_cmos_sensor(0x040C,0x0A);//image width after digital crop
    HM5040MIPI_write_cmos_sensor(0x040D,0x28);// HEX(A28) = 2600
    HM5040MIPI_write_cmos_sensor(0x040E,0x07);//image high after digital crop
    HM5040MIPI_write_cmos_sensor(0x040F,0xA0);// HEX(7A0) = 1952
    HM5040MIPI_write_cmos_sensor(0x0101,0x03);//flip+mirror
    HM5040MIPI_write_cmos_sensor(0x0100,0x01);//Streaming
#endif
    // The register only need to enable 1 time.
    //spin_lock(&HM5040_drv_lock);
    //HM5040MIPI_Auto_Flicker_mode = KAL_FALSE; // reset the flicker status
    //spin_unlock(&HM5040_drv_lock);
    //SENSORDB("[HM5040MIPI]exit HM5040MIPI_Sensor_Init function\n");
    SENSORDB("[E]");
}   /*  HM5040MIPI_Sensor_Init  */

void HM5040MIPI_VideoSizeSetting(void)
{
    //SENSORDB("[HM5040MIPI]enter HM5040MIPI_VideoSizeSetting function\n");
    SENSORDB("[S]");

    //SENSORDB("[HM5040MIPI]exit HM5040MIPI_VideoSizeSetting function\n");
    SENSORDB("[E]");
}

void HM5040MIPI_PreviewSetting(void)
{
    //SENSORDB("[HM5040MIPI]enter HM5040MIPI_PreviewSetting function\n");
    SENSORDB("[S]");
		HM5040MIPI_write_cmos_sensor(0x0100,0x00);
		mdelay(50);
    HM5040MIPI_write_cmos_sensor(0x3570,0x01);
    HM5040MIPI_write_cmos_sensor(0x0900,0x01);
    HM5040MIPI_write_cmos_sensor(0x0901,0x22); 
    HM5040MIPI_write_cmos_sensor(0x0300,0x00); 
    HM5040MIPI_write_cmos_sensor(0x0302,0x00);
    HM5040MIPI_write_cmos_sensor(0x0303,0x01); 
    HM5040MIPI_write_cmos_sensor(0x0304,0x00);
    HM5040MIPI_write_cmos_sensor(0x0306,0x00); 
    HM5040MIPI_write_cmos_sensor(0x0307,0xC8);  //20140825 Add by Zeroy   
    HM5040MIPI_write_cmos_sensor(0x0308,0x00);
    HM5040MIPI_write_cmos_sensor(0x030a,0x00); 
    HM5040MIPI_write_cmos_sensor(0x030b,0x01);
    HM5040MIPI_write_cmos_sensor(0x0340,0x05);  //03
    HM5040MIPI_write_cmos_sensor(0x0341,0xAF);  //E6 
    HM5040MIPI_write_cmos_sensor(0x034c,0x05); //x_output_size	***
    HM5040MIPI_write_cmos_sensor(0x034d,0x10); // 1296		***
    HM5040MIPI_write_cmos_sensor(0x034e,0x03); //Y_output_size	***
    HM5040MIPI_write_cmos_sensor(0x034f,0xd0); 
    HM5040MIPI_write_cmos_sensor(0x040c,0x05); //image width after digital crop ***
    HM5040MIPI_write_cmos_sensor(0x040d,0x10); // 1296		***
    HM5040MIPI_write_cmos_sensor(0x040e,0x03); //image high after digital crop ***
    HM5040MIPI_write_cmos_sensor(0x040f,0xd0); // 976               ***
    HM5040MIPI_write_cmos_sensor(0x0202,0x03); //coarse integration time ***
    HM5040MIPI_write_cmos_sensor(0x0203,0xdd); // 989		***
    HM5040MIPI_write_cmos_sensor(0x0205,0x80); //gain code		***

		HM5040MIPI_write_cmos_sensor(0x0100,0x01);
		mdelay(50);
    // The register only need to enable 1 time.
    //spin_lock(&HM5040_drv_lock);
    //HM5040MIPI_Auto_Flicker_mode = KAL_FALSE; // reset the flicker status
    //spin_unlock(&HM5040_drv_lock);
    //SENSORDB("[HM5040MIPI]exit HM5040MIPI_PreviewSetting function\n");
    SENSORDB("[E]");
}

void HM5040MIPI_set_5M(void)
{
    //SENSORDB("[HM5040MIPI]enter HM5040MIPI_set_5M function\n");
    SENSORDB("[S]");
    HM5040MIPI_write_cmos_sensor(0x0100,0x00);
    mdelay(50);
    //HM5040MIPI_write_cmos_sensor(0x3570,0x);
    HM5040MIPI_write_cmos_sensor(0x0900,0x00); //2 1
    HM5040MIPI_write_cmos_sensor(0x0901,0x00); //2 1
    HM5040MIPI_write_cmos_sensor(0x0300,0x00); 
    HM5040MIPI_write_cmos_sensor(0x0302,0x00);
    HM5040MIPI_write_cmos_sensor(0x0303,0x01); 
    HM5040MIPI_write_cmos_sensor(0x0304,0x00);
    HM5040MIPI_write_cmos_sensor(0x0306,0x01);  //20140825 Add by Zeroy 
    HM5040MIPI_write_cmos_sensor(0x0307,0x18);    
    HM5040MIPI_write_cmos_sensor(0x0308,0x00);
    HM5040MIPI_write_cmos_sensor(0x030a,0x00); 
    HM5040MIPI_write_cmos_sensor(0x030b,0x01);
    HM5040MIPI_write_cmos_sensor(0x0340,0x07); //2 1
    HM5040MIPI_write_cmos_sensor(0x0341,0xc4); //2 1
    HM5040MIPI_write_cmos_sensor(0x034C,0x0A); //2 1
    HM5040MIPI_write_cmos_sensor(0x034D,0x28); //2 1   ;x_output_size -2600
    HM5040MIPI_write_cmos_sensor(0x034E,0x07); //2 1
    HM5040MIPI_write_cmos_sensor(0x034F,0xA0); //2 1   ;Y_output_size -1952
    HM5040MIPI_write_cmos_sensor(0x040C,0x0A); //2 1
    HM5040MIPI_write_cmos_sensor(0x040D,0x28); //2 1
    HM5040MIPI_write_cmos_sensor(0x040E,0x07); //2 1
    HM5040MIPI_write_cmos_sensor(0x040F,0xA0); //2 1
    HM5040MIPI_write_cmos_sensor(0x0202,0x0e); //2 1
    HM5040MIPI_write_cmos_sensor(0x0203,0x50); //2 1
    HM5040MIPI_write_cmos_sensor(0x0205,0xC0); //2 1
    HM5040MIPI_write_cmos_sensor(0x0100,0x01);
    mdelay(50);
    //SENSORDB("[HM5040MIPI]exit HM5040MIPI_set_5M function\n");
    SENSORDB("[E]");
}

/*************************************************************************
* FUNCTION
*   HM5040MIPIGetSensorID
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
extern bool camera_pdn_reverse;

UINT32 HM5040MIPIGetSensorID(UINT32 *sensorID)
{
    int  retry = 3;
    //SENSORDB("[HM5040MIPI]enter HM5040MIPIGetSensorID function\n");
    SENSORDB("[S], *sensorID = 0x%x", *sensorID);
	//mt_set_gpio_mode(GPIO_CAMERA_CMPDN_PIN, GPIO_CAMERA_CMPDN_PIN_M_GPIO);
	//mt_set_gpio_out(GPIO_CAMERA_CMPDN_PIN, GPIO_OUT_ZERO);	
	//mt_set_gpio_out(GPIO_CAMERA_CMPDN_PIN, GPIO_OUT_ZERO);	
	
    // check if sensor ID correct
    do {
        *sensorID =((HM5040MIPI_read_cmos_sensor(0x2016) << 8) | HM5040MIPI_read_cmos_sensor(0x2017));
		SENSORDB("*sensorID = 0x%x", *sensorID);
         if(*sensorID != HM5040MIPI_SENSOR_ID)
		{
            *sensorID =0xffffffff;
			return ERROR_SENSOR_CONNECT_FAIL;
		}
          else
		{
			*sensorID=HM5040MIPI_SENSOR_ID;
			//camera_pdn_reverse = TRUE;
		        break;
		}
        
        retry--;
    } while (retry > 0);
    
    SENSORDB("[E], *sensorID = 0x%x", *sensorID);
    return ERROR_NONE;
}

/*****************************************************************************/
/* Windows Mobile Sensor Interface */
/*****************************************************************************/
/*************************************************************************
* FUNCTION
*   HM5040MIPIOpen
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
UINT32 HM5040MIPIOpen(void)
{
    //int  retry = 0;
    kal_uint32 sensorid = 0;
    //kal_uint16 sensorgain;
    // check if sensor ID correct
    //retry = 3;
    //SENSORDB("[HM5040MIPI]enter HM5040MIPIOpen function\n");
    SENSORDB("[S]");
    if (HM5040MIPIGetSensorID(&sensorid) == ERROR_SENSOR_CONNECT_FAIL)
        return ERROR_SENSOR_CONNECT_FAIL;
#if 0
    do {
        sensorid = ((HM5040MIPI_read_cmos_sensor(0x0000) << 8) | HM5040MIPI_read_cmos_sensor(0x0001));
        spin_lock(&HM5040_drv_lock);
        HM5040MIPI_sensor_id = sensorid;
        spin_unlock(&HM5040_drv_lock);
        //if (HM5040MIPI_sensor_id == HM5040MIPI_SENSOR_ID)
        if (HM5040MIPI_sensor_id == 0x0000)
            break; 
        retry--; 
    }while (retry > 0);
    SENSORDB("Read Sensor ID = 0x%04x\n", HM5040MIPI_sensor_id);
    //if (HM5040MIPI_sensor_id != HM5040MIPI_SENSOR_ID)
    if (HM5040MIPI_sensor_id != 0x0000)
        return ERROR_SENSOR_CONNECT_FAIL;
#endif
    HM5040MIPI_Sensor_Init();
    //sensorgain = read_HM5040MIPI_gain();
    spin_lock(&HM5040_drv_lock);
    //HM5040MIPI_sensor_gain_base = sensorgain;
    HM5040MIPIAutoFlicKerMode   = KAL_FALSE;
    spin_unlock(&HM5040_drv_lock);
    //SENSORDB("[HM5040MIPI]exit HM5040MIPIOpen function\n");
    SENSORDB("[E]");
    return ERROR_NONE;
}

#if 0
/*************************************************************************
* FUNCTION
*   HM5040MIPIGetSensorID
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
UINT32 HM5040MIPIGetSensorID(UINT32 *sensorID)
{
    int  retry = 3;
    //SENSORDB("[HM5040MIPI]enter HM5040MIPIGetSensorID function\n");
    SENSORDB("[S] *sensorID = 0x%x", *sensorID);
    // check if sensor ID correct
    do {
        *sensorID =((HM5040MIPI_read_cmos_sensor(0x0000) << 8) | HM5040MIPI_read_cmos_sensor(0x0001));
        //if (*sensorID == HM5040MIPI_SENSOR_ID)
        if (*sensorID == 0x0000)
            break;
        retry--;
    } while (retry > 0);
    //if (*sensorID != HM5040MIPI_SENSOR_ID) {
    if (*sensorID != 0x0000) {
        *sensorID = 0xFFFFFFFF;
        return ERROR_SENSOR_CONNECT_FAIL;
    }
//0x0000 is reserved for special using.
    else
    {
        *sensorID = 0x5040;
    }
    //SENSORDB("[HM5040MIPI]exit HM5040MIPIGetSensorID function\n");
    SENSORDB("[E] *sensorID = 0x%x", *sensorID);
    return ERROR_NONE;
}
#endif

/*Avoid Folat, frame rate =10 * u2FrameRate */
UINT32 HM5040MIPISetMaxFrameRate(UINT16 u2FrameRate)
{
    kal_int16 dummy_line=0;
    kal_uint16 FrameHeight = HM5040MIPI_sensor.frame_height;
    unsigned long flags;
    SENSORDB("[soso][OV5647MIPISetMaxFrameRate]u2FrameRate=%d \n",u2FrameRate);
    FrameHeight= (10 * HM5040MIPI_sensor.pv_pclk) / u2FrameRate / HM5040MIPI_sensor.line_length;
    if(KAL_FALSE == HM5040MIPI_sensor.pv_mode)
    {
        if(KAL_FALSE == HM5040MIPI_sensor.video_mode)
        {
            if(FrameHeight < HM5040MIPI_CAP_FULL_LINES)
                FrameHeight = HM5040MIPI_CAP_FULL_LINES;
        }
        else
        {
            if(FrameHeight < HM5040MIPI_VIDEO_FULL_LINES)
                FrameHeight = HM5040MIPI_VIDEO_FULL_LINES;
        }
    }
    spin_lock_irqsave(&HM5040_drv_lock,flags);
    HM5040MIPI_sensor.frame_height = FrameHeight;
    spin_unlock_irqrestore(&HM5040_drv_lock,flags);
    if((HM5040_CurrentScenarioId == MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG)||(HM5040_CurrentScenarioId == MSDK_SCENARIO_ID_CAMERA_ZSD))
    {
        dummy_line = FrameHeight - HM5040MIPI_CAP_FULL_LINES;
    }
    else if(HM5040_CurrentScenarioId==MSDK_SCENARIO_ID_CAMERA_PREVIEW)
    {
        dummy_line = FrameHeight - HM5040MIPI_PV_FULL_LINES;
    }
    else if(HM5040_CurrentScenarioId==MSDK_SCENARIO_ID_VIDEO_PREVIEW)
    {
        dummy_line = FrameHeight - HM5040MIPI_VIDEO_FULL_LINES;
    }
    SENSORDB("[soso][OV5647MIPISetMaxFrameRate]frameheight = %d, dummy_line=%d \n",HM5040MIPI_sensor.frame_height,dummy_line);
    if(dummy_line<0)
    {
        dummy_line = 0;
    }
    /* to fix VSYNC, to fix frame rate */
    HM5040MIPI_SetDummy(0, dummy_line); /* modify dummy_pixel must gen AE table again */
    return ERROR_NONE;
}

/*************************************************************************
* FUNCTION
*   HM5040MIPI_SetShutter
*
* DESCRIPTION
*   This function set e-shutter of HM5040MIPI to change exposure time.
*
* PARAMETERS
*   shutter : exposured lines
*
* RETURNS
*   None
*
* GLOBALS AFFECTED
*
*************************************************************************/
void HM5040MIPI_SetShutter(kal_uint16 iShutter)
{
    //SENSORDB("[HM5040MIPI]enter HM5040MIPI_SetShutter function\n");
    //SENSORDB("[HM5040MIPI]%s():shutter=%d\n",__FUNCTION__,iShutter);
    SENSORDB("[S], iShutter = 0x%x", iShutter);
    HM5040MIPI_write_cmos_sensor(0x0104, 0x01);//20140523
    if (iShutter < 1)
        iShutter = 1;
    else if(iShutter > 0xffff)
        iShutter = 0xffff;
    unsigned long flags;
    spin_lock_irqsave(&HM5040_drv_lock,flags);
    HM5040MIPI_sensor.pv_shutter = iShutter;
    spin_unlock_irqrestore(&HM5040_drv_lock,flags);
    HM5040MIPI_write_shutter(iShutter);
    HM5040MIPI_write_cmos_sensor(0x0104, 0x00);//20140523
    //SENSORDB("[HM5040MIPI]exit HM5040MIPI_SetShutter function\n");
    SENSORDB("[E]");
}   /*  HM5040MIPI_SetShutter   */

/*************************************************************************
* FUNCTION
*   HM5040MIPI_read_shutter
*
* DESCRIPTION
*   This function to  Get exposure time.
*
* PARAMETERS
*   None
*
* RETURNS
*   shutter : exposured lines
*
* GLOBALS AFFECTED
*
*************************************************************************/
UINT16 HM5040MIPI_read_shutter(void)
{
    return (UINT16)( (HM5040MIPI_read_cmos_sensor(0x0202)<<8) | HM5040MIPI_read_cmos_sensor(0x0203) );
}

/*************************************************************************
* FUNCTION
*   HM5040MIPI_night_mode
*
* DESCRIPTION
*   This function night mode of HM5040MIPI.
*
* PARAMETERS
*   none
*
* RETURNS
*   None
*
* GLOBALS AFFECTED
*
*************************************************************************/
void HM5040MIPI_NightMode(kal_bool bEnable)
{
    SENSORDB("[HM5040MIPI]enter HM5040MIPI_NightMode function\n");
#if 0
    /************************************************************************/
    /*                      Auto Mode: 30fps                                */
    /*                      Night Mode:15fps                                */
    /************************************************************************/
    if(bEnable)
    {
        if(OV5642_MPEG4_encode_mode==KAL_TRUE)
        {
            OV5642_MAX_EXPOSURE_LINES = (kal_uint16)((OV5642_sensor_pclk/15)/(OV5642_PV_PERIOD_PIXEL_NUMS+OV5642_PV_dummy_pixels));
            OV5642_write_cmos_sensor(0x350C, (OV5642_MAX_EXPOSURE_LINES >> 8) & 0xFF);
            OV5642_write_cmos_sensor(0x350D, OV5642_MAX_EXPOSURE_LINES & 0xFF);
            OV5642_CURRENT_FRAME_LINES = OV5642_MAX_EXPOSURE_LINES;
            OV5642_MAX_EXPOSURE_LINES = OV5642_CURRENT_FRAME_LINES - OV5642_SHUTTER_LINES_GAP;
        }
    }
    else// Fix video framerate 30 fps
    {
        if(OV5642_MPEG4_encode_mode==KAL_TRUE)
        {
            OV5642_MAX_EXPOSURE_LINES = (kal_uint16)((OV5642_sensor_pclk/30)/(OV5642_PV_PERIOD_PIXEL_NUMS+OV5642_PV_dummy_pixels));
            if(OV5642_pv_exposure_lines < (OV5642_MAX_EXPOSURE_LINES - OV5642_SHUTTER_LINES_GAP)) // for avoid the shutter > frame_lines,move the frame lines setting to shutter function
            {
                OV5642_write_cmos_sensor(0x350C, (OV5642_MAX_EXPOSURE_LINES >> 8) & 0xFF);
                OV5642_write_cmos_sensor(0x350D, OV5642_MAX_EXPOSURE_LINES & 0xFF);
                OV5642_CURRENT_FRAME_LINES = OV5642_MAX_EXPOSURE_LINES;
            }
            OV5642_MAX_EXPOSURE_LINES = OV5642_MAX_EXPOSURE_LINES - OV5642_SHUTTER_LINES_GAP;
        }
    }
#endif
    SENSORDB("[HM5040MIPI]exit HM5040MIPI_NightMode function\n");
}/*	HM5040MIPI_NightMode */

/*************************************************************************
* FUNCTION
*   HM5040MIPIClose
*
* DESCRIPTION
*   This function is to turn off sensor module power.
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
UINT32 HM5040MIPIClose(void)
{
    SENSORDB("[S]");
    SENSORDB("[E]");
    return ERROR_NONE;
}   /* HM5040MIPIClose() */

void HM5040MIPISetFlipMirror(kal_int32 imgMirror)
{
    kal_uint8  iTemp; 
    SENSORDB("[HM5040MIPI]enter HM5040MIPISetFlipMirror function\n");
    //iTemp = HM5040MIPI_read_cmos_sensor(0x0101) & 0x03; //Clear the mirror and flip bits.
    switch (imgMirror)
    {
        case IMAGE_NORMAL:
            //HM5040MIPI_write_cmos_sensor(0x0101, 0x03); //Set normal
            break;
        case IMAGE_V_MIRROR:
            //HM5040MIPI_write_cmos_sensor(0x0101, iTemp | 0x01); //Set flip
            break;
        case IMAGE_H_MIRROR:
            //HM5040MIPI_write_cmos_sensor(0x0101, iTemp | 0x02); //Set mirror
            break;
        case IMAGE_HV_MIRROR:
            //HM5040MIPI_write_cmos_sensor(0x0101, 0x00); //Set mirror and flip
            break;
    }
    SENSORDB("[HM5040MIPI]exit HM5040MIPISetFlipMirror function\n");
}

/*************************************************************************
* FUNCTION
*   HM5040MIPIPreview
*
* DESCRIPTION
*   This function start the sensor preview.
*
* PARAMETERS
*   *image_window : address pointer of pixel numbers in one period of HSYNC
*   *sensor_config_data : address pointer of line numbers in one period of VSYNC
*
* RETURNS
*   None
*
* GLOBALS AFFECTED
*
*************************************************************************/
UINT32 HM5040MIPIPreview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                                  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    //SENSORDB("[HM5040MIPI]enter HM5040MIPIPreview function\n");
    SENSORDB("[S]");
    spin_lock(&HM5040_drv_lock);
    //HM5040MIPI_MPEG4_encode_mode = KAL_FALSE;
    HM5040MIPI_sensor.video_mode = KAL_FALSE;
    HM5040MIPI_sensor.pv_mode = KAL_TRUE;
    HM5040MIPI_sensor.capture_mode = KAL_FALSE;
    spin_unlock(&HM5040_drv_lock);

    HM5040MIPI_PreviewSetting();
    spin_lock(&HM5040_drv_lock);
    HM5040MIPI_sensor.cp_dummy_pixels = 0;
    HM5040MIPI_sensor.cp_dummy_lines = 0;
    HM5040MIPI_sensor.pv_dummy_pixels = 0;
    HM5040MIPI_sensor.pv_dummy_lines = 0;
    HM5040MIPI_sensor.video_dummy_pixels = 0;
    HM5040MIPI_sensor.video_dummy_lines = 0;
    HM5040MIPI_sensor.pv_line_length  = HM5040MIPI_PV_FULL_PIXELS+HM5040MIPI_sensor.pv_dummy_pixels;
    HM5040MIPI_sensor.pv_frame_length = HM5040MIPI_PV_FULL_LINES+HM5040MIPI_sensor.pv_dummy_lines;
    HM5040MIPI_sensor.frame_height    = HM5040MIPI_sensor.pv_frame_length;
    HM5040MIPI_sensor.line_length     = HM5040MIPI_sensor.pv_line_length;
    spin_unlock(&HM5040_drv_lock);

    HM5040MIPI_SetDummy(HM5040MIPI_sensor.pv_dummy_pixels,HM5040MIPI_sensor.pv_dummy_lines);
    HM5040MIPI_SetShutter(HM5040MIPI_sensor.pv_shutter);
    spin_lock(&HM5040_drv_lock);
    memcpy(&HM5040MIPISensorConfigData, sensor_config_data, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
    spin_unlock(&HM5040_drv_lock);

    //SENSORDB("[HM5040MIPI]exit HM5040MIPIPreview function\n");
    SENSORDB("[S]");
    return ERROR_NONE;
}   /* HM5040MIPIPreview() */

/*************************************************************************
* FUNCTION
* RETURNS
*   None
*
* GLOBALS AFFECTED
*
*************************************************************************/
UINT32 HM5040MIPIVideo(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                                MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    kal_uint16 iStartX = 0, iStartY = 0;
    SENSORDB("[HM5040MIPI]enter HM5040MIPIVideo function\n");
    spin_lock(&HM5040_drv_lock);
    //HM5040MIPI_MPEG4_encode_mode = KAL_TRUE;
    HM5040MIPI_sensor.video_mode=KAL_TRUE;
    HM5040MIPI_sensor.pv_mode=KAL_FALSE;
    HM5040MIPI_sensor.capture_mode=KAL_FALSE;
    spin_unlock(&HM5040_drv_lock);

    HM5040MIPI_VideoSizeSetting();
    spin_lock(&HM5040_drv_lock);
    HM5040MIPI_sensor.cp_dummy_pixels = 0;
    HM5040MIPI_sensor.cp_dummy_lines = 0;
    HM5040MIPI_sensor.pv_dummy_pixels = 0;
    HM5040MIPI_sensor.pv_dummy_lines = 0;
    HM5040MIPI_sensor.video_dummy_pixels = 0;
    HM5040MIPI_sensor.video_dummy_lines = 0;
    HM5040MIPI_sensor.video_line_length  = HM5040MIPI_VIDEO_FULL_PIXELS+HM5040MIPI_sensor.video_dummy_pixels;
    HM5040MIPI_sensor.video_frame_length = HM5040MIPI_VIDEO_FULL_LINES+HM5040MIPI_sensor.video_dummy_lines;
    HM5040MIPI_sensor.frame_height       = HM5040MIPI_sensor.video_frame_length;
    HM5040MIPI_sensor.line_length        = HM5040MIPI_sensor.video_line_length;
    spin_unlock(&HM5040_drv_lock);

    HM5040MIPI_SetDummy(HM5040MIPI_sensor.video_dummy_pixels,HM5040MIPI_sensor.video_dummy_lines);
    HM5040MIPI_SetShutter(HM5040MIPI_sensor.video_shutter);
    spin_lock(&HM5040_drv_lock);
    memcpy(&HM5040MIPISensorConfigData, sensor_config_data, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
    spin_unlock(&HM5040_drv_lock);

    SENSORDB("[HM5040MIPI]exit HM5040MIPIVideo function\n");
    return ERROR_NONE;
}   /* HM5040MIPIPreview() */

UINT32 HM5040MIPICapture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                                  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    //SENSORDB("[HM5040MIPI]enter HM5040MIPICapture function\n");
    SENSORDB("[S]");
    spin_lock(&HM5040_drv_lock);
    HM5040MIPI_sensor.video_mode=KAL_FALSE;
    HM5040MIPI_sensor.pv_mode=KAL_FALSE;
    HM5040MIPI_sensor.capture_mode=KAL_TRUE;
    HM5040MIPIAutoFlicKerMode = KAL_FALSE;
    //HM5040MIPI_MPEG4_encode_mode = KAL_FALSE;
    //HM5040MIPI_Auto_Flicker_mode = KAL_FALSE;
    HM5040MIPI_sensor.cp_dummy_pixels = 0;
    HM5040MIPI_sensor.cp_dummy_lines = 0;
    spin_unlock(&HM5040_drv_lock);
    
    HM5040MIPI_set_5M();
    //HM5040MIPISetFlipMirror(sensor_config_data->SensorImageMirror);
    spin_lock(&HM5040_drv_lock);
    HM5040MIPI_sensor.cp_line_length  = HM5040MIPI_CAP_FULL_PIXELS+HM5040MIPI_sensor.cp_dummy_pixels;
    HM5040MIPI_sensor.cp_frame_length = HM5040MIPI_CAP_FULL_LINES+HM5040MIPI_sensor.cp_dummy_lines;
    HM5040MIPI_sensor.frame_height    = HM5040MIPI_sensor.cp_frame_length;
    HM5040MIPI_sensor.line_length     = HM5040MIPI_sensor.cp_line_length;
    spin_unlock(&HM5040_drv_lock);
    
    HM5040MIPI_SetDummy(HM5040MIPI_sensor.cp_dummy_pixels, HM5040MIPI_sensor.cp_dummy_lines);
    spin_lock(&HM5040_drv_lock);
    memcpy(&HM5040MIPISensorConfigData, sensor_config_data, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
    spin_unlock(&HM5040_drv_lock);
    
    //SENSORDB("[HM5040MIPI]exit HM5040MIPICapture function\n");
    SENSORDB("[E]");
    return ERROR_NONE;
}   /* HM5040MIPICapture() */

UINT32 HM5040MIPIGetResolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *pSensorResolution)
{
    //SENSORDB("[HM5040MIPI]enter HM5040MIPIGetResolution function\n");
    SENSORDB("[S]");
    pSensorResolution->SensorPreviewWidth   = HM5040MIPI_REAL_PV_WIDTH;
    pSensorResolution->SensorPreviewHeight  = HM5040MIPI_REAL_PV_HEIGHT;
    pSensorResolution->SensorFullWidth      = HM5040MIPI_REAL_CAP_WIDTH;
    pSensorResolution->SensorFullHeight     = HM5040MIPI_REAL_CAP_HEIGHT;
    pSensorResolution->SensorVideoWidth     = HM5040MIPI_REAL_VIDEO_WIDTH;
    pSensorResolution->SensorVideoHeight    = HM5040MIPI_REAL_VIDEO_HEIGHT;
    //SENSORDB("[HM5040MIPI]exit HM5040MIPIGetResolution function\n");
    SENSORDB("[E]");
    return ERROR_NONE;
}/* HM5040MIPIGetResolution() */

UINT32 HM5040MIPIGetInfo(MSDK_SCENARIO_ID_ENUM ScenarioId,
                       MSDK_SENSOR_INFO_STRUCT *pSensorInfo,
                     MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
    //SENSORDB("[HM5040MIPI]enter HM5040MIPIGetInfo function\n");
    SENSORDB("[S], ScenarioId = 0x%x", ScenarioId);
    switch(ScenarioId)
    {
        case MSDK_SCENARIO_ID_CAMERA_ZSD:
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
            pSensorInfo->SensorFullResolutionX=HM5040MIPI_REAL_CAP_WIDTH;
            pSensorInfo->SensorFullResolutionY=HM5040MIPI_REAL_CAP_HEIGHT;
            pSensorInfo->SensorStillCaptureFrameRate=30;
            break;
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
            pSensorInfo->SensorPreviewResolutionX=HM5040MIPI_REAL_VIDEO_WIDTH;
            pSensorInfo->SensorPreviewResolutionY=HM5040MIPI_REAL_VIDEO_HEIGHT;
            pSensorInfo->SensorCameraPreviewFrameRate=30;
            break;
        default:
            pSensorInfo->SensorPreviewResolutionX=HM5040MIPI_REAL_PV_WIDTH;
            pSensorInfo->SensorPreviewResolutionY=HM5040MIPI_REAL_PV_HEIGHT;
            pSensorInfo->SensorCameraPreviewFrameRate=30;
            break;
    }
    pSensorInfo->SensorVideoFrameRate=30;
    pSensorInfo->SensorStillCaptureFrameRate=30;
    pSensorInfo->SensorWebCamCaptureFrameRate=30;
    pSensorInfo->SensorClockPolarity=SENSOR_CLOCK_POLARITY_LOW;
    pSensorInfo->SensorClockFallingPolarity=SENSOR_CLOCK_POLARITY_LOW;
    pSensorInfo->SensorHsyncPolarity=SENSOR_CLOCK_POLARITY_LOW;
    pSensorInfo->SensorVsyncPolarity=SENSOR_CLOCK_POLARITY_LOW;//HIGH
    pSensorInfo->SensorInterruptDelayLines=1;
    pSensorInfo->SensorResetActiveHigh=FALSE;
    pSensorInfo->SensorResetDelayCount=5;
    pSensorInfo->SensroInterfaceType=SENSOR_INTERFACE_TYPE_MIPI;
    //pSensorInfo->SensorOutputDataFormat=SENSOR_OUTPUT_FORMAT_RAW_B;
    pSensorInfo->SensorOutputDataFormat=SENSOR_OUTPUT_FORMAT_RAW_R;
    pSensorInfo->CaptureDelayFrame=2; //2
    pSensorInfo->PreviewDelayFrame=2; //3->1 20140523
    pSensorInfo->VideoDelayFrame=2;//2
    pSensorInfo->SensorDrivingCurrent=ISP_DRIVING_2MA;
    //pSensorInfo->SensorDrivingCurrent=ISP_DRIVING_8MA;
    pSensorInfo->SensorMasterClockSwitch=0;
    pSensorInfo->AEShutDelayFrame=0;/* The frame of setting shutter default 0 for TG int */
    pSensorInfo->AESensorGainDelayFrame=0;/* The frame of setting sensor gain */
    pSensorInfo->AEISPGainDelayFrame=2;//4->2  20140520
#if 0
    switch(ScenarioId)
    {
        case MSDK_SCENARIO_ID_CAMERA_ZSD:
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
            pSensorInfo->SensorFullResolutionX=HM5040MIPI_REAL_CAP_WIDTH;
            pSensorInfo->SensorFullResolutionY=HM5040MIPI_REAL_CAP_HEIGHT;
            pSensorInfo->SensorStillCaptureFrameRate=30;
            break;
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
            pSensorInfo->SensorPreviewResolutionX=HM5040MIPI_REAL_VIDEO_WIDTH;
            pSensorInfo->SensorPreviewResolutionY=HM5040MIPI_REAL_VIDEO_HEIGHT;
            pSensorInfo->SensorCameraPreviewFrameRate=30;
            break;
        default:
            pSensorInfo->SensorPreviewResolutionX=HM5040MIPI_REAL_PV_WIDTH;
            pSensorInfo->SensorPreviewResolutionY=HM5040MIPI_REAL_PV_HEIGHT;
            pSensorInfo->SensorCameraPreviewFrameRate=30;
            break;
    }
#endif
    switch (ScenarioId)
    {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
            pSensorInfo->SensorClockFreq=24;
            pSensorInfo->SensorClockDividCount=5;
            pSensorInfo->SensorClockRisingCount= 0;
            pSensorInfo->SensorClockFallingCount= 2;
            pSensorInfo->SensorPixelClockCount= 3;
            pSensorInfo->SensorDataLatchCount= 2;
            pSensorInfo->SensorGrabStartX = HM5040MIPI_IMAGE_SENSOR_PV_STARTX;
            pSensorInfo->SensorGrabStartY = HM5040MIPI_IMAGE_SENSOR_PV_STARTY;
            pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_2_LANE;
            pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
            pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 14;
            pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
            pSensorInfo->SensorWidthSampling = 0; // 0 is default 1x
            pSensorInfo->SensorHightSampling = 0; // 0 is default 1x
            pSensorInfo->SensorPacketECCOrder = 1;
            break;
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
            pSensorInfo->SensorClockFreq=24;
            pSensorInfo->SensorClockDividCount= 5;
            pSensorInfo->SensorClockRisingCount= 0;
            pSensorInfo->SensorClockFallingCount= 2;
            pSensorInfo->SensorPixelClockCount= 3;
            pSensorInfo->SensorDataLatchCount= 2;
            pSensorInfo->SensorGrabStartX = HM5040MIPI_IMAGE_SENSOR_VIDEO_STARTX;
            pSensorInfo->SensorGrabStartY = HM5040MIPI_IMAGE_SENSOR_VIDEO_STARTY;
            pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_2_LANE;
            pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
            pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 14;
            pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
            pSensorInfo->SensorWidthSampling = 0; // 0 is default 1x
            pSensorInfo->SensorHightSampling = 0; // 0 is default 1x
            pSensorInfo->SensorPacketECCOrder = 1;
            break;
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
        case MSDK_SCENARIO_ID_CAMERA_ZSD:
            pSensorInfo->SensorClockFreq=24;
            pSensorInfo->SensorClockDividCount=5;
            pSensorInfo->SensorClockRisingCount= 0;
            pSensorInfo->SensorClockFallingCount= 2;
            pSensorInfo->SensorPixelClockCount= 3;
            pSensorInfo->SensorDataLatchCount= 2;
            pSensorInfo->SensorGrabStartX = HM5040MIPI_IMAGE_SENSOR_CAP_STARTX; // 2*HM5040MIPI_IMAGE_SENSOR_PV_STARTX;
            pSensorInfo->SensorGrabStartY = HM5040MIPI_IMAGE_SENSOR_CAP_STARTY; // 2*HM5040MIPI_IMAGE_SENSOR_PV_STARTY;
            pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_2_LANE;
            pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
            pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 14;
            pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
            pSensorInfo->SensorWidthSampling = 0; // 0 is default 1x
            pSensorInfo->SensorHightSampling = 0; // 0 is default 1x
            pSensorInfo->SensorPacketECCOrder = 1;
            break;
        default:
            pSensorInfo->SensorClockFreq=24;
            pSensorInfo->SensorClockDividCount= 5;
            pSensorInfo->SensorClockRisingCount= 0;
            pSensorInfo->SensorClockFallingCount= 2;
            pSensorInfo->SensorPixelClockCount= 3;
            pSensorInfo->SensorDataLatchCount= 2;
            pSensorInfo->SensorGrabStartX = HM5040MIPI_IMAGE_SENSOR_PV_STARTX;
            pSensorInfo->SensorGrabStartY = HM5040MIPI_IMAGE_SENSOR_PV_STARTY;
            pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_2_LANE;
            pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
            pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 14;
            pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
            pSensorInfo->SensorWidthSampling = 0; // 0 is default 1x
            pSensorInfo->SensorHightSampling = 0; // 0 is default 1x
            pSensorInfo->SensorPacketECCOrder = 1;
            break;
    }
    spin_lock(&HM5040_drv_lock);
    //HM5040MIPIPixelClockDivider=pSensorInfo->SensorPixelClockCount;
    memcpy(pSensorConfigData, &HM5040MIPISensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
    spin_unlock(&HM5040_drv_lock);
    //SENSORDB("[HM5040MIPI]exit HM5040MIPIGetInfo function\n");
    SENSORDB("[E]");
    return ERROR_NONE;
}   /* HM5040MIPIGetInfo() */

UINT32 HM5040MIPIControl(MSDK_SCENARIO_ID_ENUM ScenarioId,
            MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *pImageWindow,
                     MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
    spin_lock(&HM5040_drv_lock);
    HM5040_CurrentScenarioId = ScenarioId;
    spin_unlock(&HM5040_drv_lock);
    //SENSORDB("[HM5040MIPI]enter HM5040MIPIControl function\n");
    SENSORDB("[S]");
    switch (ScenarioId)
    {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
            HM5040MIPIPreview(pImageWindow, pSensorConfigData);
            break;
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
            HM5040MIPIVideo(pImageWindow, pSensorConfigData);
            break;
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
        case MSDK_SCENARIO_ID_CAMERA_ZSD:
            HM5040MIPICapture(pImageWindow, pSensorConfigData);
            break;
        default:
            return ERROR_INVALID_SCENARIO_ID;
    }
    //SENSORDB("[HM5040MIPI]exit HM5040MIPIControl function\n");
    SENSORDB("[E]");
    return ERROR_NONE;
}   /* HM5040MIPIControl() */

UINT32 HM5040MIPISetVideoMode(UINT16 u2FrameRate)
{
    SENSORDB("[HM5040MIPISetVideoMode] frame rate = %d\n", u2FrameRate);
    kal_uint16 HM5040MIPI_Video_Max_Expourse_Time = 0;
    SENSORDB("[HM5040MIPI]%s():fix_frame_rate=%d\n",__FUNCTION__,u2FrameRate);
    spin_lock(&HM5040_drv_lock);
    HM5040MIPI_sensor.fix_video_fps = KAL_TRUE;
    spin_unlock(&HM5040_drv_lock);
    u2FrameRate=u2FrameRate*10;//10*FPS
    if(u2FrameRate==300)
        u2FrameRate=296;
    SENSORDB("[HM5040MIPI][Enter Fix_fps func] HM5040MIPI_Fix_Video_Frame_Rate = %d\n", u2FrameRate/10);
    HM5040MIPI_Video_Max_Expourse_Time = (kal_uint16)((HM5040MIPI_sensor.video_pclk*10/u2FrameRate)/HM5040MIPI_sensor.video_line_length);
    if (HM5040MIPI_Video_Max_Expourse_Time > HM5040MIPI_VIDEO_FULL_LINES/*HM5040MIPI_sensor.pv_frame_length*/)
    {
        spin_lock(&HM5040_drv_lock);
        HM5040MIPI_sensor.video_frame_length = HM5040MIPI_Video_Max_Expourse_Time;
        HM5040MIPI_sensor.video_dummy_lines = HM5040MIPI_sensor.video_frame_length-HM5040MIPI_VIDEO_FULL_LINES;
        spin_unlock(&HM5040_drv_lock);
        SENSORDB("[HM5040MIPI]%s():frame_length=%d,dummy_lines=%d\n",__FUNCTION__,HM5040MIPI_sensor.video_frame_length,HM5040MIPI_sensor.video_dummy_lines);
        HM5040MIPI_SetDummy(HM5040MIPI_sensor.video_dummy_pixels,HM5040MIPI_sensor.video_dummy_lines);
    }
    spin_lock(&HM5040_drv_lock);
    //HM5040MIPI_MPEG4_encode_mode = KAL_TRUE;
    HM5040MIPI_sensor.FixedFps   = u2FrameRate;
    spin_unlock(&HM5040_drv_lock);
    SENSORDB("[HM5040MIPI]exit HM5040MIPISetVideoMode function\n");
    return ERROR_NONE;
}

UINT32 HM5040MIPISetAutoFlickerMode(kal_bool bEnable, UINT16 u2FrameRate)
{
    SENSORDB("OV5647MIPISetAutoFlickerMode:%d",bEnable);
    if(bEnable)
    {
        spin_lock(&HM5040_drv_lock);
        HM5040MIPIAutoFlicKerMode = KAL_TRUE;
        spin_unlock(&HM5040_drv_lock);
        /*Change frame rate 29.5fps to 29.8fps to do Auto flick*/
        if((HM5040MIPI_sensor.FixedFps == 30)&&(HM5040MIPI_sensor.video_mode==KAL_TRUE))
            HM5040MIPISetMaxFrameRate(296);
    }
    else
    {//Cancel Auto flick
        spin_lock(&HM5040_drv_lock);
        HM5040MIPIAutoFlicKerMode = KAL_FALSE;
        spin_unlock(&HM5040_drv_lock);
        if((HM5040MIPI_sensor.FixedFps == 30)&&(HM5040MIPI_sensor.video_mode==KAL_TRUE))
            HM5040MIPISetMaxFrameRate(300);
    }
    return ERROR_NONE;
}

UINT32 HM5040MIPISetMaxFramerateByScenario(MSDK_SCENARIO_ID_ENUM scenarioId, MUINT32 frameRate)
{
    kal_uint32 pclk;
    kal_int16 dummyLine;
    kal_uint16 lineLength,frameHeight;	
    SENSORDB("HM5040MIPISetMaxFramerateByScenario: scenarioId = %d, frame rate = %d\n",scenarioId,frameRate);
    switch (scenarioId)
    {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
            pclk = 132000000;//120000000;
            lineLength = HM5040MIPI_PV_DUMMY_PIXELS;
            frameHeight = (10 * pclk)/frameRate/lineLength;
            dummyLine = frameHeight - HM5040MIPI_PV_DUMMY_LINES;
            if(dummyLine<0)
                dummyLine = 0;
            spin_lock(&HM5040_drv_lock);
            HM5040MIPI_sensor.pv_mode=TRUE;
            spin_unlock(&HM5040_drv_lock);
            HM5040MIPI_SetDummy(0, dummyLine);
            break;
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
            pclk = 132000000;//120000000;
            lineLength = HM5040MIPI_VIDEO_DUMMY_PIXELS;
            frameHeight = (10 * pclk)/frameRate/lineLength;
            dummyLine = frameHeight - HM5040MIPI_VIDEO_DUMMY_LINES;
            if(dummyLine<0)
                dummyLine = 0;
            spin_lock(&HM5040_drv_lock);
            HM5040MIPI_sensor.pv_mode=TRUE;
            spin_unlock(&HM5040_drv_lock);
            HM5040MIPI_SetDummy(0, dummyLine);
            break;
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
        case MSDK_SCENARIO_ID_CAMERA_ZSD:
            pclk = 132000000;//120000000;
            lineLength = HM5040MIPI_FULL_DUMMY_PIXELS;
            frameHeight = (10 * pclk)/frameRate/lineLength;
            dummyLine = frameHeight - HM5040MIPI_FULL_DUMMY_LINES;
            if(dummyLine<0)
                dummyLine = 0;
            spin_lock(&HM5040_drv_lock);
            HM5040MIPI_sensor.pv_mode=FALSE;
            spin_unlock(&HM5040_drv_lock);
            HM5040MIPI_SetDummy(0, dummyLine);
            break;
        case MSDK_SCENARIO_ID_CAMERA_3D_PREVIEW: //added
            break;
        case MSDK_SCENARIO_ID_CAMERA_3D_VIDEO:
            break;
        case MSDK_SCENARIO_ID_CAMERA_3D_CAPTURE: //added
            break;
        default:
            break;
    }
    SENSORDB("[HM5040MIPI]exit HM5040MIPISetMaxFramerateByScenario function\n");
    return ERROR_NONE;
}

UINT32 HM5040MIPIGetDefaultFramerateByScenario(MSDK_SCENARIO_ID_ENUM scenarioId, MUINT32 *pframeRate)
{
    switch (scenarioId)
    {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
            *pframeRate = 300;
            break;
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
        case MSDK_SCENARIO_ID_CAMERA_ZSD:
            *pframeRate = 150;
            break;//hhl 2-28
        case MSDK_SCENARIO_ID_CAMERA_3D_PREVIEW: //added
        case MSDK_SCENARIO_ID_CAMERA_3D_VIDEO:
        case MSDK_SCENARIO_ID_CAMERA_3D_CAPTURE: //added   
            *pframeRate = 300;
            break;
        default:
            break;
    }
    return ERROR_NONE;
}

UINT32 HM5040MIPISetTestPatternMode(kal_bool bEnable)
{
    SENSORDB("[HM5040MIPISetTestPatternMode] Test pattern enable:%d\n", bEnable);
    if(bEnable) { // enable color bar
        //HM5040MIPI_write_cmos_sensor(0x30D8, 0x10); // color bar test pattern
        //HM5040MIPI_write_cmos_sensor(0x0600, 0x00); // color bar test pattern
        //HM5040MIPI_write_cmos_sensor(0x0601, 0x02); // color bar test pattern
    } else {
       // HM5040MIPI_write_cmos_sensor(0x30D8, 0x00); // disable color bar test pattern
    }
    return ERROR_NONE;
}

UINT32 HM5040MIPIFeatureControl(MSDK_SENSOR_FEATURE_ENUM FeatureId,
                                                   UINT8 *pFeaturePara,
                                                  UINT32 *pFeatureParaLen)
{
    UINT16 *pFeatureReturnPara16=(UINT16 *) pFeaturePara;
    UINT16 *pFeatureData16=(UINT16 *) pFeaturePara;
    UINT32 *pFeatureReturnPara32=(UINT32 *) pFeaturePara;
    UINT32 *pFeatureData32=(UINT32 *) pFeaturePara;
    UINT32 SensorRegNumber;
    UINT32 i;
    PNVRAM_SENSOR_DATA_STRUCT pSensorDefaultData=(PNVRAM_SENSOR_DATA_STRUCT) pFeaturePara;
    MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData=(MSDK_SENSOR_CONFIG_STRUCT *) pFeaturePara;
    MSDK_SENSOR_REG_INFO_STRUCT *pSensorRegData=(MSDK_SENSOR_REG_INFO_STRUCT *) pFeaturePara;
    MSDK_SENSOR_GROUP_INFO_STRUCT *pSensorGroupInfo=(MSDK_SENSOR_GROUP_INFO_STRUCT *) pFeaturePara;
    MSDK_SENSOR_ITEM_INFO_STRUCT *pSensorItemInfo=(MSDK_SENSOR_ITEM_INFO_STRUCT *) pFeaturePara;
    MSDK_SENSOR_ENG_INFO_STRUCT *pSensorEngInfo=(MSDK_SENSOR_ENG_INFO_STRUCT *) pFeaturePara;
    SENSORDB("[S], FeatureId = %d", FeatureId);
    switch (FeatureId)
    {
        case SENSOR_FEATURE_GET_RESOLUTION:     //3001
            *pFeatureReturnPara16++=HM5040MIPI_REAL_CAP_WIDTH;
            *pFeatureReturnPara16=HM5040MIPI_REAL_CAP_HEIGHT;
            *pFeatureParaLen=4;
            break;
        case SENSOR_FEATURE_GET_PERIOD:             //3002
            switch(HM5040_CurrentScenarioId)
            {
                case MSDK_SCENARIO_ID_CAMERA_ZSD:
                case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
                    *pFeatureReturnPara16++=HM5040MIPI_sensor.cp_line_length;
                    *pFeatureReturnPara16=HM5040MIPI_sensor.cp_frame_length;
                    SENSORDB("Sensor cap period:%d %d", HM5040MIPI_sensor.cp_line_length, HM5040MIPI_sensor.cp_frame_length);
                    *pFeatureParaLen=4;
                    break;
                case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
                    *pFeatureReturnPara16++=HM5040MIPI_sensor.video_line_length;
                    *pFeatureReturnPara16=HM5040MIPI_sensor.video_frame_length;
                    SENSORDB("Sensor video period:%d %d", HM5040MIPI_sensor.video_line_length, HM5040MIPI_sensor.video_frame_length);
                    *pFeatureParaLen=4;
                    break;
                default:
                    *pFeatureReturnPara16++=HM5040MIPI_sensor.pv_line_length;
                    *pFeatureReturnPara16=HM5040MIPI_sensor.pv_frame_length;
                    SENSORDB("Sensor pv period:%d %d", HM5040MIPI_sensor.pv_line_length, HM5040MIPI_sensor.pv_frame_length);
                    *pFeatureParaLen=4;
                    break;
            }
            break;
        case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:   //3003
            switch(HM5040_CurrentScenarioId)
            {
                case MSDK_SCENARIO_ID_CAMERA_ZSD:
                case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
                    *pFeatureReturnPara32 = HM5040MIPI_sensor.cp_pclk;
                    *pFeatureParaLen=4;
                    SENSORDB("Sensor CPCLK:%d",HM5040MIPI_sensor.cp_pclk);
                    break;
                case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
                    *pFeatureReturnPara32 = HM5040MIPI_sensor.video_pclk;
                    *pFeatureParaLen=4;
                    SENSORDB("Sensor videoCLK:%d",HM5040MIPI_sensor.video_pclk);
                    break;
                default:
                    *pFeatureReturnPara32 = HM5040MIPI_sensor.pv_pclk;
                    *pFeatureParaLen=4;
                    SENSORDB("Sensor pvclk:%d",HM5040MIPI_sensor.pv_pclk);
                    break;
             }
             break;
        case SENSOR_FEATURE_SET_ESHUTTER:           //3004
            HM5040MIPI_SetShutter(*pFeatureData16);
            break;
        case SENSOR_FEATURE_SET_NIGHTMODE:          //3005
            HM5040MIPI_NightMode((BOOL) *pFeatureData16);
            break;
        case SENSOR_FEATURE_SET_GAIN:               //3006
            HM5040MIPI_SetGain((UINT16) *pFeatureData16);
            break;
        case SENSOR_FEATURE_SET_FLASHLIGHT:         //3007
            break;
        case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:  //3008
            spin_lock(&HM5040_drv_lock);
            HM5040MIPI_isp_master_clock=*pFeatureData32;
            spin_unlock(&HM5040_drv_lock);
            break;
        case SENSOR_FEATURE_SET_REGISTER:           //3009
            HM5040MIPI_write_cmos_sensor(pSensorRegData->RegAddr, pSensorRegData->RegData);
            break;
        case SENSOR_FEATURE_GET_REGISTER:           //3010
            pSensorRegData->RegData = HM5040MIPI_read_cmos_sensor(pSensorRegData->RegAddr);
            break;
        case SENSOR_FEATURE_SET_CCT_REGISTER:       //3011
            SensorRegNumber=FACTORY_END_ADDR;
            for (i=0;i<SensorRegNumber;i++)
            {
                spin_lock(&HM5040_drv_lock);
                HM5040MIPISensorCCT[i].Addr=*pFeatureData32++;
                HM5040MIPISensorCCT[i].Para=*pFeatureData32++;
                spin_unlock(&HM5040_drv_lock);
            }
            break;
        case SENSOR_FEATURE_GET_CCT_REGISTER:       //3012
            SensorRegNumber=FACTORY_END_ADDR;
            if (*pFeatureParaLen<(SensorRegNumber*sizeof(SENSOR_REG_STRUCT)+4))
                return FALSE;
            *pFeatureData32++=SensorRegNumber;
            for (i=0;i<SensorRegNumber;i++)
            {
                *pFeatureData32++=HM5040MIPISensorCCT[i].Addr;
                *pFeatureData32++=HM5040MIPISensorCCT[i].Para; 
            }
            break;
        case SENSOR_FEATURE_SET_ENG_REGISTER:       //3013
            SensorRegNumber=ENGINEER_END;
            for (i=0;i<SensorRegNumber;i++)
            {   spin_lock(&HM5040_drv_lock);
                HM5040MIPISensorReg[i].Addr=*pFeatureData32++;
                HM5040MIPISensorReg[i].Para=*pFeatureData32++;
                spin_unlock(&HM5040_drv_lock);
            }
            break;
        case SENSOR_FEATURE_GET_ENG_REGISTER:       //3014
            SensorRegNumber=ENGINEER_END;
            if (*pFeatureParaLen<(SensorRegNumber*sizeof(SENSOR_REG_STRUCT)+4))
                return FALSE;
            *pFeatureData32++=SensorRegNumber;
            for (i=0;i<SensorRegNumber;i++)
            {
                *pFeatureData32++=HM5040MIPISensorReg[i].Addr;
                *pFeatureData32++=HM5040MIPISensorReg[i].Para;
            }
            break;
        case SENSOR_FEATURE_GET_REGISTER_DEFAULT:   //3015
            if (*pFeatureParaLen>=sizeof(NVRAM_SENSOR_DATA_STRUCT))
            {
                pSensorDefaultData->Version=NVRAM_CAMERA_SENSOR_FILE_VERSION;
                pSensorDefaultData->SensorId=HM5040MIPI_SENSOR_ID;
                memcpy(pSensorDefaultData->SensorEngReg, HM5040MIPISensorReg, sizeof(SENSOR_REG_STRUCT)*ENGINEER_END);
                memcpy(pSensorDefaultData->SensorCCTReg, HM5040MIPISensorCCT, sizeof(SENSOR_REG_STRUCT)*FACTORY_END_ADDR);
            }
            else
                return FALSE;
            *pFeatureParaLen=sizeof(NVRAM_SENSOR_DATA_STRUCT);
            break;
        case SENSOR_FEATURE_GET_CONFIG_PARA:        //3016
            memcpy(pSensorConfigData, &HM5040MIPISensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));

            *pFeatureParaLen=sizeof(MSDK_SENSOR_CONFIG_STRUCT);
            break;
        case SENSOR_FEATURE_CAMERA_PARA_TO_SENSOR:  //3017
            HM5040MIPI_camera_para_to_sensor();
            break;
        case SENSOR_FEATURE_SENSOR_TO_CAMERA_PARA:  //3018
            HM5040MIPI_sensor_to_camera_para();
            break;
        case SENSOR_FEATURE_GET_GROUP_COUNT:        //3019
            *pFeatureReturnPara32++=HM5040MIPI_get_sensor_group_count();
            *pFeatureParaLen=4;
            break;
        case SENSOR_FEATURE_GET_GROUP_INFO:         //3020
            HM5040MIPI_get_sensor_group_info(pSensorGroupInfo->GroupIdx, pSensorGroupInfo->GroupNamePtr, &pSensorGroupInfo->ItemCount);
            *pFeatureParaLen=sizeof(MSDK_SENSOR_GROUP_INFO_STRUCT);
            break;
        case SENSOR_FEATURE_GET_ITEM_INFO:          //3021
            HM5040MIPI_get_sensor_item_info(pSensorItemInfo->GroupIdx,pSensorItemInfo->ItemIdx, pSensorItemInfo);
            *pFeatureParaLen=sizeof(MSDK_SENSOR_ITEM_INFO_STRUCT);
            break;
        case SENSOR_FEATURE_SET_ITEM_INFO:          //3022
            HM5040MIPI_set_sensor_item_info(pSensorItemInfo->GroupIdx, pSensorItemInfo->ItemIdx, pSensorItemInfo->ItemValue);
            *pFeatureParaLen=sizeof(MSDK_SENSOR_ITEM_INFO_STRUCT);
            break;
        case SENSOR_FEATURE_GET_ENG_INFO:           //3023
            pSensorEngInfo->SensorId = 129;
            pSensorEngInfo->SensorType = CMOS_SENSOR;
            pSensorEngInfo->SensorOutputDataFormat=SENSOR_OUTPUT_FORMAT_RAW_B;
            *pFeatureParaLen=sizeof(MSDK_SENSOR_ENG_INFO_STRUCT);
            break;
        case SENSOR_FEATURE_GET_LENS_DRIVER_ID:     //3024
            // get the lens driver ID from EEPROM or just return LENS_DRIVER_ID_DO_NOT_CARE
            // if EEPROM does not exist in camera module.
            *pFeatureReturnPara32=LENS_DRIVER_ID_DO_NOT_CARE;
            *pFeatureParaLen=4;
            break;
        case SENSOR_FEATURE_SET_YUV_CMD:            //3025
            break;
        case SENSOR_FEATURE_SET_VIDEO_MODE:         //3026
            HM5040MIPISetVideoMode(*pFeatureData16);
            break;
        case SENSOR_FEATURE_SET_CALIBRATION_DATA:   //3027
            break;
        case SENSOR_FEATURE_SET_SENSOR_SYNC:        //3028
            break;
        case SENSOR_FEATURE_INITIALIZE_AF:          //3029
            break;
        case SENSOR_FEATURE_CONSTANT_AF:            //3030
            break;
        case SENSOR_FEATURE_MOVE_FOCUS_LENS:        //3031
            break;
        case SENSOR_FEATURE_GET_AF_STATUS:          //3032
            break;
        case SENSOR_FEATURE_GET_AF_INF:             //3033
            break;
        case SENSOR_FEATURE_GET_AF_MACRO:           //3034
            break;
        case SENSOR_FEATURE_CHECK_SENSOR_ID:        //3035
            HM5040MIPIGetSensorID(pFeatureReturnPara32);
            break;
        case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE:  //3036
            HM5040MIPISetAutoFlickerMode((BOOL)*pFeatureData16, *(pFeatureData16+1));
            break;
        case SENSOR_FEATURE_SET_TEST_PATTERN:       //3037
            HM5040MIPISetTestPatternMode((BOOL)*pFeatureData16);
            break;
        case SENSOR_FEATURE_SET_SOFTWARE_PWDN:      //3038
            break;
        case SENSOR_FEATURE_SINGLE_FOCUS_MODE:      //3039
            break;
        case SENSOR_FEATURE_CANCEL_AF:              //3040
            break;
        case SENSOR_FEATURE_SET_AF_WINDOW:          //3041
            break;
        case SENSOR_FEATURE_GET_EV_AWB_REF:         //3042
            break;
        case SENSOR_FEATURE_GET_SHUTTER_GAIN_AWB_GAIN:          //3043
            break;
        case SENSOR_FEATURE_GET_AF_MAX_NUM_FOCUS_AREAS:         //3044
            break;
        case SENSOR_FEATURE_GET_AE_MAX_NUM_METERING_AREAS:      //3045
            break;
        case SENSOR_FEATURE_SET_AE_WINDOW:          //3046
            break;
        case SENSOR_FEATURE_GET_EXIF_INFO:          //3047
            break;
        case SENSOR_FEATURE_GET_DELAY_INFO:         //3048
            break;
        case SENSOR_FEATURE_SET_SLAVE_I2C_ID:       //3049
            break;
        case SENSOR_FEATURE_SUSPEND:                //3050
            break;
        case SENSOR_FEATURE_RESUME:                 //3051
            break;
        case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:     //3052
            HM5040MIPISetMaxFramerateByScenario((MSDK_SCENARIO_ID_ENUM)*pFeatureData32, *(pFeatureData32+1));
            break;
        case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO: //3053
            HM5040MIPIGetDefaultFramerateByScenario((MSDK_SCENARIO_ID_ENUM)*pFeatureData32, (MUINT32 *)(*(pFeatureData32+1)));
            break;
        case SENSOR_FEATURE_GET_AE_AWB_LOCK_INFO:   //3054
            break;
        default:
            SENSORDB("No %d Command", FeatureId);
            break;
    }
    SENSORDB("[E]");
    return ERROR_NONE;
}   /* HM5040MIPIFeatureControl() */

SENSOR_FUNCTION_STRUCT SensorFuncHM5040MIPI=
{
    HM5040MIPIOpen,
    HM5040MIPIGetInfo,
    HM5040MIPIGetResolution,
    HM5040MIPIFeatureControl,
    HM5040MIPIControl,
    HM5040MIPIClose
};

UINT32 HM5040MIPI_RAW_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
    /* To Do : Check Sensor status here */
    if (pfFunc!=NULL)
        *pfFunc=&SensorFuncHM5040MIPI;
    return ERROR_NONE;
}   /* SensorInit() */
