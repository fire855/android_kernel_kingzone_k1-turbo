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

#include "hm8030mipiraw_Sensor.h"
#include "hm8030mipiraw_Camera_Sensor_para.h"
#include "hm8030mipiraw_CameraCustomized.h"

kal_bool  HM8030MIPI_MPEG4_encode_mode = KAL_FALSE;
kal_bool HM8030MIPI_Auto_Flicker_mode = KAL_FALSE;


kal_uint8 HM8030MIPI_sensor_write_I2C_address = HM8030MIPI_WRITE_ID;
kal_uint8 HM8030MIPI_sensor_read_I2C_address = HM8030MIPI_READ_ID;
static kal_bool HM8030MIPIAutoFlicKerMode = KAL_FALSE;
#define DELAY_FRAME 10
static kal_uint8 frame_count = DELAY_FRAME;
static kal_uint8 current_gain = 0;
static kal_uint16 current_shutter = 0;


	
static struct HM8030MIPI_sensor_STRUCT HM8030MIPI_sensor={HM8030MIPI_WRITE_ID,HM8030MIPI_READ_ID,KAL_TRUE,KAL_FALSE,KAL_TRUE,KAL_FALSE,
KAL_FALSE,KAL_FALSE,KAL_FALSE,132000000,132000000,132000000,0,0,0,64,64,64,HM8030MIPI_PV_FULL_PIXELS,HM8030MIPI_PV_FULL_LINES,
HM8030MIPI_VIDEO_FULL_PIXELS,HM8030MIPI_VIDEO_FULL_LINES,HM8030MIPI_CAP_FULL_PIXELS,HM8030MIPI_CAP_FULL_LINES,0,0,0,0,0,0,
30,HM8030MIPI_PV_FULL_LINES,HM8030MIPI_PV_FULL_PIXELS,0};
static struct HM8030_GAIN HM8030_gain_table[]=
{
//0x0204/0x0205
    {0x0000, 100},
    {0x0010, 107},
    {0x0020, 114},
    {0x0030, 123},
    {0x0040, 133},
    {0x0050, 145},
    {0x0060, 160},
    {0x0070, 178},
    {0x0080, 200},
    {0x0090, 229},
    {0x00A0, 266},
    {0x00B0, 320},
    {0x00C0, 400},
    {0x00D0, 531},
    {0x00E0, 803},
    {0x00E4, 912},
    {0x00E8, 1072},
    {0x00EC, 1274},
    {0x00F0, 1603},
    {0xFFFF, 0},
};
MSDK_SCENARIO_ID_ENUM HM8030_CurrentScenarioId = MSDK_SCENARIO_ID_CAMERA_PREVIEW;	
kal_uint16	HM8030MIPI_sensor_gain_base=0x0;
/* MAX/MIN Explosure Lines Used By AE Algorithm */
kal_uint16 HM8030MIPI_MAX_EXPOSURE_LINES = HM8030MIPI_PV_DUMMY_LINES-5;//650;
kal_uint8  HM8030MIPI_MIN_EXPOSURE_LINES = 2;
kal_uint32 HM8030MIPI_isp_master_clock;

UINT32 HM8030MIPISetMaxFrameRate(UINT16 u2FrameRate);

static DEFINE_SPINLOCK(HM8030_drv_lock);

#define SENSORDB(fmt, arg...) printk( "[HM8030MIPIRaw] "  fmt, ##arg)
#define RETAILMSG(x,...)
#define TEXT
UINT8 HM8030MIPIPixelClockDivider=0;
kal_uint16 HM8030MIPI_sensor_id=0;
MSDK_SENSOR_CONFIG_STRUCT HM8030MIPISensorConfigData;
kal_uint32 HM8030MIPI_FAC_SENSOR_REG;
kal_uint16 HM8030MIPI_sensor_flip_value; 
/* FIXME: old factors and DIDNOT use now. s*/
SENSOR_REG_STRUCT HM8030MIPISensorCCT[]=CAMERA_SENSOR_CCT_DEFAULT_VALUE;
SENSOR_REG_STRUCT HM8030MIPISensorReg[ENGINEER_END]=CAMERA_SENSOR_REG_DEFAULT_VALUE;
/* FIXME: old factors and DIDNOT use now. e*/
extern int iReadReg(u16 a_u2Addr , u8 * a_puBuff , u16 i2cId);
extern int iWriteReg(u16 a_u2Addr , u32 a_u4Data , u32 a_u4Bytes , u16 i2cId);
#define HM8030MIPI_write_cmos_sensor(addr, para) iWriteReg((u16) addr , (u32) para , 1, HM8030MIPI_WRITE_ID)

kal_uint16 HM8030MIPI_read_cmos_sensor(kal_uint32 addr)
{
	kal_uint16 get_byte=0;
    iReadReg((u16) addr ,(u8*)&get_byte,HM8030MIPI_WRITE_ID);
    return get_byte;
}

void HM8030MIPI_write_shutter(kal_uint16 shutter)
{
	kal_uint32 dummy_line = 0,dummy_pixel=0,shutter1=0,shutter2=0;
    kal_uint32 extra_lines = 0;
	kal_uint32 max_exp_shutter = 0;
	unsigned long flags;	
	SENSORDB("[HM8030MIPI]enter HM8030MIPI_write_shutter function,shutter=%d\n",shutter); 
	if(HM8030MIPIAutoFlicKerMode)
	{
		if(HM8030MIPI_sensor.video_mode == KAL_FALSE)
		{
			if(HM8030_CurrentScenarioId== MSDK_SCENARIO_ID_CAMERA_ZSD)
			{
				//Change frame 14.7fps ~ 14.9fps to do auto flick
				HM8030MIPISetMaxFrameRate(148);
			}
			else
			{
				//Change frame 29.5fps ~ 29.8fps to do auto flick
				HM8030MIPISetMaxFrameRate(148);
			}
		}
	}
    if (HM8030MIPI_sensor.pv_mode == KAL_TRUE) 
	 {
	   max_exp_shutter = HM8030MIPI_PV_FULL_LINES + HM8030MIPI_sensor.pv_dummy_lines;
     }
     else if (HM8030MIPI_sensor.video_mode== KAL_TRUE) 
     {
       max_exp_shutter = HM8030MIPI_VIDEO_FULL_LINES + HM8030MIPI_sensor.video_dummy_lines;
	 }	 
     else if (HM8030MIPI_sensor.capture_mode== KAL_TRUE) 
     {
       max_exp_shutter = HM8030MIPI_CAP_FULL_LINES + HM8030MIPI_sensor.cp_dummy_lines;
	 }	 
	 else
	 	{
	 	
		SENSORDB("sensor mode error\n");
	 	}
	 
	 if(shutter > max_exp_shutter)
	   extra_lines = shutter - max_exp_shutter;
	 else 
	   extra_lines = 0;
	 if (HM8030MIPI_sensor.pv_mode == KAL_TRUE) 
	 {
	 	SENSORDB("[HM8030MIPI]PV shutter HM8030MIPI_write_shutter function,shutter=%d\n",shutter);
       dummy_line =HM8030MIPI_PV_DUMMY_LINES+ HM8030MIPI_sensor.pv_dummy_lines + extra_lines;
	   dummy_pixel = HM8030MIPI_PV_DUMMY_PIXELS+ HM8030MIPI_sensor.pv_dummy_pixels;
	   spin_lock_irqsave(&HM8030_drv_lock,flags);
	   HM8030MIPI_sensor.pv_line_length  = HM8030MIPI_PV_FULL_PIXELS+dummy_pixel;
	   HM8030MIPI_sensor.pv_frame_length = HM8030MIPI_PV_FULL_LINES+dummy_line;
	   HM8030MIPI_sensor.frame_height= HM8030MIPI_sensor.pv_frame_length;
	   HM8030MIPI_sensor.line_length = HM8030MIPI_sensor.pv_line_length;
	   spin_unlock_irqrestore(&HM8030_drv_lock,flags);
	 }
	 else if (HM8030MIPI_sensor.video_mode== KAL_TRUE) 
     {
	    dummy_line = HM8030MIPI_VIDEO_DUMMY_LINES+ HM8030MIPI_sensor.video_dummy_lines + extra_lines;
		dummy_pixel =HM8030MIPI_VIDEO_DUMMY_PIXELS + HM8030MIPI_sensor.video_dummy_pixels;
		spin_lock_irqsave(&HM8030_drv_lock,flags);
		HM8030MIPI_sensor.video_line_length  = HM8030MIPI_VIDEO_FULL_PIXELS+dummy_pixel;
	    HM8030MIPI_sensor.video_frame_length = HM8030MIPI_VIDEO_FULL_LINES+dummy_line;
		HM8030MIPI_sensor.line_length  = HM8030MIPI_sensor.video_line_length; 
		HM8030MIPI_sensor.frame_height = HM8030MIPI_sensor.video_frame_length;
		spin_unlock_irqrestore(&HM8030_drv_lock,flags);
	 } 
	 else if(HM8030MIPI_sensor.capture_mode== KAL_TRUE)
	 	{
	 	#if 0
	    dummy_line = HM8030MIPI_FULL_DUMMY_LINES+ HM8030MIPI_sensor.cp_dummy_lines + extra_lines;
		dummy_pixel =HM8030MIPI_FULL_DUMMY_PIXELS + HM8030MIPI_sensor.cp_dummy_pixels;
		spin_lock_irqsave(&HM8030_drv_lock,flags);
		HM8030MIPI_sensor.cp_line_length = HM8030MIPI_REAL_CAP_WIDTH+dummy_pixel;
	  HM8030MIPI_sensor.cp_frame_length = HM8030MIPI_REAL_CAP_HEIGHT+dummy_line;
		HM8030MIPI_sensor.frame_height = HM8030MIPI_sensor.cp_frame_length;
		HM8030MIPI_sensor.line_length  = HM8030MIPI_sensor.cp_line_length;
		spin_unlock_irqrestore(&HM8030_drv_lock,flags);
		#else
		SENSORDB("[HM8030MIPI]CAP shutter HM8030MIPI_write_shutter function,shutter=%d\n",shutter);
		dummy_line = HM8030MIPI_PV_DUMMY_LINES+ HM8030MIPI_sensor.pv_dummy_lines + extra_lines;
		dummy_pixel =HM8030MIPI_PV_DUMMY_PIXELS+ HM8030MIPI_sensor.pv_dummy_pixels;
		spin_lock_irqsave(&HM8030_drv_lock,flags);
		HM8030MIPI_sensor.cp_line_length = HM8030MIPI_CAP_FULL_PIXELS+dummy_pixel;
	  HM8030MIPI_sensor.cp_frame_length = HM8030MIPI_CAP_FULL_LINES+dummy_line;
		HM8030MIPI_sensor.frame_height = HM8030MIPI_sensor.cp_frame_length;
		HM8030MIPI_sensor.line_length  = HM8030MIPI_sensor.cp_line_length;
		spin_unlock_irqrestore(&HM8030_drv_lock,flags);
		#endif
	 }
	 else
	 {	 	
		SENSORDB("sensor mode error\n");
	 }
		//HM8030MIPI_write_cmos_sensor(0x0104, 0x01);
		HM8030MIPI_write_cmos_sensor(0x0202, (shutter >> 8) & 0xFF);
    HM8030MIPI_write_cmos_sensor(0x0203,  shutter  & 0xFF); 
	current_shutter = shutter;
		HM8030MIPI_write_cmos_sensor(0x0204,0x00);
	HM8030MIPI_write_cmos_sensor(0x0205,current_gain);
	//HM8030MIPI_write_cmos_sensor(0x0104, 0x00);
    shutter2=shutter+0x0A;
	if (shutter2<0x09EC && HM8030MIPI_sensor.capture_mode== KAL_TRUE)
		{
		HM8030MIPI_write_cmos_sensor(0x0340,  0x09);
		HM8030MIPI_write_cmos_sensor(0x0341,  0xEC);
		}
else if ( (HM8030MIPI_sensor.video_mode== KAL_TRUE || HM8030MIPI_sensor.pv_mode== KAL_TRUE) && shutter2< 1280)
{
HM8030MIPI_write_cmos_sensor(0x0340,  0x05);
HM8030MIPI_write_cmos_sensor(0x0341,  0x00);
}
	else
		{
		HM8030MIPI_write_cmos_sensor(0x0340,  (shutter2 >> 8) & 0xFF);
		HM8030MIPI_write_cmos_sensor(0x0341,  shutter2  & 0xFF);
		}

    	SENSORDB("[HM8030MIPI]exit HM8030MIPI_write_shutter function\n");
}   /* write_HM8030MIPI_shutter */

static kal_uint16 HM8030MIPIReg2Gain(const kal_uint8 iReg)
{
	kal_uint16 Reg_Cgain=0,Reg_Fgain=0,i=0;
	float gain_tmp0 =0;
	SENSORDB("[HM8030MIPI]enter HM8030MIPIReg2Gain function\n");
	Reg_Cgain=HM8030MIPI_read_cmos_sensor(0x0204);
	Reg_Fgain=HM8030MIPI_read_cmos_sensor(0x0205);
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
	SENSORDB("[HM8030MIPI]exit HM8030MIPIReg2Gain function \n");
	return (kal_uint8)gain_tmp0;
	SENSORDB("[HM8030MIPI]mycat11 enter HM8030MIPIReg2Gain function iGain=%d\n",gain_tmp0);
}
/*************************************************************************
* FUNCTION
*    HM8030MIPIGain2Reg
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
static kal_uint8 HM8030MIPIGain2Reg(const kal_uint16 iGain)
{
   /*******************gain=2^Reg_Cgain +Reg_Fgain/64*****************************/
    SENSORDB("[HM8030MIPI]mycat enter HM8030MIPIReg2Gain function iGain=%d\n",iGain);
    kal_uint8 i = 0;
   kal_uint8 gain_tmp1=0,Reg_Cgain=0,Reg_Fgain=0;
   kal_uint16 gain_tmp0 = 0, result1 = 0, result2 = 0;
	//iGain_temp=(float)iGain;
	#if 1
	gain_tmp0=(iGain*100) / BASEGAIN;
	SENSORDB("[HM8030MIPI]mycat BASEGAIN=%d\n",BASEGAIN);
	SENSORDB("[HM8030MIPI]mycat enter HM8030MIPIReg2Gain function gain_tmp0=%d\n",gain_tmp0);
	//gain_tmp1=(iGain % BASEGAIN);
    do {
        if (gain_tmp0 < HM8030_gain_table[i].analoggain)
            break;
        i++;
    } while (HM8030_gain_table[i].analoggain != 0);
    if (i == 0)
    {
        Reg_Cgain = 0x00;
    }
    else
	{
	    result1 = HM8030_gain_table[i].analoggain - gain_tmp0;
		result2 = gain_tmp0 - HM8030_gain_table[i-1].analoggain;
		if (result1 < result2)
		    Reg_Cgain = HM8030_gain_table[i].gainvalue & 0xFF;
		else
			Reg_Cgain = HM8030_gain_table[i-1].gainvalue & 0xFF;
	    //Reg_Cgain = HM8030_gain_table[i-1].gainvalue & 0xFF;
	}
#if 0
	if((gain_tmp0>=100)&&(gain_tmp0<107))
	{
		Reg_Cgain=0x00;
		//Reg_Fgain=iGain;
	}
	else if((gain_tmp0>=107)&&(gain_tmp0<114))
	{
		Reg_Cgain=0x10;
	  //Reg_Fgain=iGain/2;
	}
	else if((gain_tmp0>=114)&&(gain_tmp0<123))
	{
		Reg_Cgain=0x20;
	    //Reg_Fgain=iGain/4;
	}
	else if((gain_tmp0>=123)&&(gain_tmp0<133))
	{
		Reg_Cgain=0x30;
	    //Reg_Fgain=iGain/4;
	}
	else if((gain_tmp0>=133)&&(gain_tmp0<145))
	{
		Reg_Cgain=0x40;
	    //Reg_Fgain=iGain/4;
	}
	else if((gain_tmp0>=145)&&(gain_tmp0<160))
	{
		Reg_Cgain=0x50;
	    //Reg_Fgain=iGain/4;
	}
	else if((gain_tmp0>=160)&&(gain_tmp0<178))
	{
		Reg_Cgain=0x60;
	    //Reg_Fgain=iGain/4;
	}
	else if((gain_tmp0>=178)&&(gain_tmp0<200))
	{
		Reg_Cgain=0x70;
	    //Reg_Fgain=iGain/4;
	}
	else if((gain_tmp0>=200)&&(gain_tmp0<229))
	{
		Reg_Cgain=0x80;
	    //Reg_Fgain=iGain/4;
	}
	else if((gain_tmp0>=229)&&(gain_tmp0<266))
	{
		Reg_Cgain=0x90;
	    //Reg_Fgain=iGain/4;
	}
	else if((gain_tmp0>=266)&&(gain_tmp0<320))
	{
		Reg_Cgain=0xA0;
	    //Reg_Fgain=iGain/4;
	}
	else if((gain_tmp0>=320)&&(gain_tmp0<400))
	{
		Reg_Cgain=0xB0;
	    //Reg_Fgain=iGain/4;
	}
	else if((gain_tmp0>=400)&&(gain_tmp0<531))
	{
		Reg_Cgain=0xC0;
	    //Reg_Fgain=iGain/4;
	}
	else if((gain_tmp0>=531)&&(gain_tmp0<803))
	{
		Reg_Cgain=0xD0;
	    //Reg_Fgain=iGain/4;
	}
	else if((gain_tmp0>=803)&&(gain_tmp0<912))
	{
		Reg_Cgain=0xE0;
	    //Reg_Fgain=iGain/4;
	}
	else if((gain_tmp0>=912)&&(gain_tmp0<1072))
	{
		Reg_Cgain=0xE4;
	    //Reg_Fgain=iGain/4;
	}
	else if((gain_tmp0>=1072)&&(gain_tmp0<1274))
	{
		Reg_Cgain=0xE8;
	    //Reg_Fgain=iGain/4;
	}
	else if((gain_tmp0>=1274)&&(gain_tmp0<1603))
	{
		Reg_Cgain=0xEC;
	    //Reg_Fgain=iGain/4;
	}
	
	else
	{
		Reg_Cgain=0xF0;
	    //Reg_Fgain=iGain/8;
	}
#endif
	#else
	gain_tmp0=iGain / BASEGAIN;
	if(gain_tmp0<2)
		{
			Reg_Cgain=0x00;
			//Reg_Fgain=iGain;
		}
    else if((gain_tmp0>=2)&&(gain_tmp0<4))
	{
		Reg_Cgain=0x80;
	    //Reg_Fgain=iGain/4;
	}
	else if((gain_tmp0>=4)&&(gain_tmp0<8))
	{
		Reg_Cgain=0xC0;
	    //Reg_Fgain=iGain/4;
	}
	else if((gain_tmp0>=8)&&(gain_tmp0<16))
	{
		Reg_Cgain=0xE0;
	    //Reg_Fgain=iGain/4;
	}
	else if(gain_tmp0>=16)
	{
		Reg_Cgain=0xF0;
	    //Reg_Fgain=iGain/4;
	}

	#endif
if(Reg_Cgain == current_gain)
    {
        frame_count--;
    }
    else
    {
        current_gain = Reg_Cgain;
        frame_count = DELAY_FRAME;
        SENSORDB("Walter debug: disable BLC\n");
        //HM8030MIPI_write_cmos_sensor(0xA651,0x01);
        //HM8030MIPI_write_cmos_sensor(0xA651,0x02);
	//HM8030MIPI_write_cmos_sensor(0x0104, 0x01);
HM8030MIPI_write_cmos_sensor(0x0202, (current_shutter >> 8) & 0xFF);
HM8030MIPI_write_cmos_sensor(0x0203,  current_shutter  & 0xFF); 
	HM8030MIPI_write_cmos_sensor(0x0204,0x00);
	HM8030MIPI_write_cmos_sensor(0x0205,Reg_Cgain);
	//HM8030MIPI_write_cmos_sensor(0x0104, 0x00);
    }
    if(frame_count == 0)
    {
        SENSORDB("Walter debug: enable BLC\n");
        //HM8030MIPI_write_cmos_sensor(0xA651,0x02);

    }
	//Reg_Cgain = Reg_Cgain & 0xFF;
	//Reg_Fgain = Reg_Fgain & 0xFF;
	//HM8030MIPI_write_cmos_sensor(0x0204,0x00);
	//HM8030MIPI_write_cmos_sensor(0x0205,Reg_Cgain);
	//HM8030MIPI_write_cmos_sensor(0x0104, 0x00);

	SENSORDB("[HM8030MIPI]mycat exit HM8030MIPIGain2Reg function Reg_Cgain=%d,Reg_Fgain=%d\n",Reg_Cgain,Reg_Fgain);
	return NONE;
}

/*************************************************************************
* FUNCTION
*    HM8030MIPI_SetGain
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
void HM8030MIPI_SetGain(UINT16 iGain)
{   
	SENSORDB("[HM8030MIPI]enter HM8030MIPI_SetGain function iGain =%d\n",iGain);
    HM8030MIPIGain2Reg(iGain);
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
        HM8030MIPI_write_cmos_sensor(0xA651,0x01);
        HM8030MIPIGain2Reg(iGain);
    }
    if(frame_count == 0)
    {
        SENSORDB("Walter debug: enable BLC\n");
        HM8030MIPI_write_cmos_sensor(0xA651,0x02);

    }
#endif
    SENSORDB("[HM8030MIPI]exit HM8030MIPI_SetGain function\n");
}   /*  HM8030MIPI_SetGain_SetGain  */


/*************************************************************************
* FUNCTION
*    read_HM8030MIPI_gain
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
kal_uint16 read_HM8030MIPI_gain(void)
{  
	kal_uint16 Reg_Cgain=0,Reg_Fgain=0,i=0;
	float gain_tmp0 =0;
	SENSORDB("[HM8030MIPI]enter read_HM8030MIPI_gain function\n");

		Reg_Cgain=HM8030MIPI_read_cmos_sensor(0x0204);
	    Reg_Fgain=HM8030MIPI_read_cmos_sensor(0x0205);
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
	SENSORDB("[HM8030MIPI]exit read_HM8030MIPI_gain function \n");
	return gain_tmp0;
}  /* read_HM8030MIPI_gain */

void HM8030MIPI_camera_para_to_sensor(void)
{

	kal_uint32    i;
	SENSORDB("[HM8030MIPI]enter HM8030MIPI_camera_para_to_sensor function\n");
    for(i=0; 0xFFFFFFFF!=HM8030MIPISensorReg[i].Addr; i++)
    {
        HM8030MIPI_write_cmos_sensor(HM8030MIPISensorReg[i].Addr, HM8030MIPISensorReg[i].Para);
    }
    for(i=ENGINEER_START_ADDR; 0xFFFFFFFF!=HM8030MIPISensorReg[i].Addr; i++)
    {
        HM8030MIPI_write_cmos_sensor(HM8030MIPISensorReg[i].Addr, HM8030MIPISensorReg[i].Para);
    }
    for(i=FACTORY_START_ADDR; i<FACTORY_END_ADDR; i++)
    {
        HM8030MIPI_write_cmos_sensor(HM8030MIPISensorCCT[i].Addr, HM8030MIPISensorCCT[i].Para);
    }
	SENSORDB("[HM8030MIPI]exit HM8030MIPI_camera_para_to_sensor function\n");

}


/*************************************************************************
* FUNCTION
*    HM8030MIPI_sensor_to_camera_para
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
void HM8030MIPI_sensor_to_camera_para(void)
{

	kal_uint32    i,temp_data;
	SENSORDB("[HM8030MIPI]enter HM8030MIPI_sensor_to_camera_para function\n");
    for(i=0; 0xFFFFFFFF!=HM8030MIPISensorReg[i].Addr; i++)
    {
		temp_data=HM8030MIPI_read_cmos_sensor(HM8030MIPISensorReg[i].Addr);
		spin_lock(&HM8030_drv_lock);
		HM8030MIPISensorReg[i].Para = temp_data;
		spin_unlock(&HM8030_drv_lock);
    }
    for(i=ENGINEER_START_ADDR; 0xFFFFFFFF!=HM8030MIPISensorReg[i].Addr; i++)
    {
    	temp_data=HM8030MIPI_read_cmos_sensor(HM8030MIPISensorReg[i].Addr);
         spin_lock(&HM8030_drv_lock);
        HM8030MIPISensorReg[i].Para = temp_data;
		spin_unlock(&HM8030_drv_lock);
    }
	SENSORDB("[HM8030MIPI]exit HM8030MIPI_sensor_to_camera_para function\n");

}

/*************************************************************************
* FUNCTION
*    HM8030MIPI_get_sensor_group_count
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
kal_int32  HM8030MIPI_get_sensor_group_count(void)
{
    return GROUP_TOTAL_NUMS;
}

void HM8030MIPI_get_sensor_group_info(kal_uint16 group_idx, kal_int8* group_name_ptr, kal_int32* item_count_ptr)
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

void HM8030MIPI_get_sensor_item_info(kal_uint16 group_idx,kal_uint16 item_idx, MSDK_SENSOR_ITEM_INFO_STRUCT* info_ptr)
{
}
kal_bool HM8030MIPI_set_sensor_item_info(kal_uint16 group_idx, kal_uint16 item_idx, kal_int32 ItemValue)
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
            temp_para = HM8030MIPIGain2Reg(ItemValue);
            spin_lock(&HM8030_drv_lock);    
            HM8030MIPISensorCCT[temp_addr].Para = temp_para;
			spin_unlock(&HM8030_drv_lock);
            HM8030MIPI_write_cmos_sensor(HM8030MIPISensorCCT[temp_addr].Addr,temp_para);
			temp_para=read_HM8030MIPI_gain();	
            spin_lock(&HM8030_drv_lock);    
            HM8030MIPI_sensor_gain_base=temp_para;
			spin_unlock(&HM8030_drv_lock);

            break;
        case CMMCLK_CURRENT:
            switch (item_idx)
            {
                case 0:
                    if(ItemValue==2)
                    {			
                    spin_lock(&HM8030_drv_lock);    
                        HM8030MIPISensorReg[CMMCLK_CURRENT_INDEX].Para = ISP_DRIVING_2MA;
					spin_unlock(&HM8030_drv_lock);
                        //HM8030MIPI_set_isp_driving_current(ISP_DRIVING_2MA);
                    }
                    else if(ItemValue==3 || ItemValue==4)
                    {
                    	spin_lock(&HM8030_drv_lock);    
                        HM8030MIPISensorReg[CMMCLK_CURRENT_INDEX].Para = ISP_DRIVING_4MA;
						spin_unlock(&HM8030_drv_lock);
                        //HM8030MIPI_set_isp_driving_current(ISP_DRIVING_4MA);
                    }
                    else if(ItemValue==5 || ItemValue==6)
                    {
                    	spin_lock(&HM8030_drv_lock);    
                        HM8030MIPISensorReg[CMMCLK_CURRENT_INDEX].Para = ISP_DRIVING_6MA;
						spin_unlock(&HM8030_drv_lock);
                        //HM8030MIPI_set_isp_driving_current(ISP_DRIVING_6MA);
                    }
                    else
                    {
                    	spin_lock(&HM8030_drv_lock);    
                        HM8030MIPISensorReg[CMMCLK_CURRENT_INDEX].Para = ISP_DRIVING_8MA;
						spin_unlock(&HM8030_drv_lock);
                        //HM8030MIPI_set_isp_driving_current(ISP_DRIVING_8MA);
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
					spin_lock(&HM8030_drv_lock);    
                    HM8030MIPI_FAC_SENSOR_REG=ItemValue;
					spin_unlock(&HM8030_drv_lock);
                    break;
                case 1:
                    HM8030MIPI_write_cmos_sensor(HM8030MIPI_FAC_SENSOR_REG,ItemValue);
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
static void HM8030MIPI_SetDummy(const kal_uint16 iPixels, const kal_uint16 iLines)
{
	kal_uint32 frame_length = 0, line_length = 0,dummy_tmp0=0,dummy_tmp1=0;
	SENSORDB("[HM8030MIPI]enter HM8030MIPI_SetDummy function,iPixels=%d,iLines=%d\n",iPixels,iLines);
	 if(HM8030MIPI_sensor.pv_mode == KAL_TRUE)
	 {
		 SENSORDB("[HM8030MIPI]enter pv_mode HM8030MIPI_SetDummy\n");
		 spin_lock(&HM8030_drv_lock);    
	   	 HM8030MIPI_sensor.pv_dummy_pixels =  HM8030MIPI_PV_DUMMY_PIXELS+iPixels;
		 HM8030MIPI_sensor.pv_dummy_lines  =  HM8030MIPI_PV_DUMMY_LINES + iLines;
	   	 HM8030MIPI_sensor.pv_line_length  =  HM8030MIPI_PV_FULL_PIXELS + iPixels;
		 HM8030MIPI_sensor.pv_frame_length =  HM8030MIPI_PV_FULL_LINES + iLines;
		 HM8030MIPI_sensor.frame_height    =  HM8030MIPI_sensor.pv_frame_length;
		 HM8030MIPI_sensor.line_length     =  HM8030MIPI_sensor.pv_line_length;
		 spin_unlock(&HM8030_drv_lock);
		 line_length = HM8030MIPI_sensor.pv_dummy_pixels-164;
		 frame_length = HM8030MIPI_sensor.pv_dummy_lines-97;	 	
	 }
   else if(HM8030MIPI_sensor.video_mode== KAL_TRUE)
   	{
   	 SENSORDB("[HM8030MIPI]enter video_mode HM8030MIPI_SetDummy\n");
   	 spin_lock(&HM8030_drv_lock);    
   	 HM8030MIPI_sensor.video_dummy_pixels = HM8030MIPI_VIDEO_DUMMY_PIXELS + iPixels;
	 HM8030MIPI_sensor.video_dummy_lines  = HM8030MIPI_VIDEO_DUMMY_LINES + iLines;
   	 HM8030MIPI_sensor.video_line_length  = HM8030MIPI_VIDEO_FULL_PIXELS + iPixels;
	 HM8030MIPI_sensor.video_frame_length = HM8030MIPI_VIDEO_FULL_LINES + iLines;
	 HM8030MIPI_sensor.frame_height       = HM8030MIPI_sensor.video_frame_length;
	 HM8030MIPI_sensor.line_length        = HM8030MIPI_sensor.video_line_length;
	 spin_unlock(&HM8030_drv_lock);
	 line_length = HM8030MIPI_sensor.video_dummy_pixels-164;
	 frame_length = HM8030MIPI_sensor.video_dummy_lines-97;
   	}
	else if(HM8030MIPI_sensor.capture_mode== KAL_TRUE) 
	{	
	  SENSORDB("[HM8030MIPI]enter capture_mode HM8030MIPI_SetDummy\n");
	  spin_lock(&HM8030_drv_lock);	
   	  HM8030MIPI_sensor.cp_dummy_pixels = HM8030MIPI_FULL_DUMMY_PIXELS + iPixels;;
	  HM8030MIPI_sensor.cp_dummy_lines  = HM8030MIPI_FULL_DUMMY_LINES + iLines;
	  HM8030MIPI_sensor.cp_line_length  = HM8030MIPI_CAP_FULL_PIXELS + iPixels;
	  HM8030MIPI_sensor.cp_frame_length = HM8030MIPI_CAP_FULL_LINES + iLines;
	  HM8030MIPI_sensor.frame_height    = HM8030MIPI_sensor.cp_frame_length;
	  HM8030MIPI_sensor.line_length     = HM8030MIPI_sensor.cp_line_length;
	   spin_unlock(&HM8030_drv_lock);
	  line_length = HM8030MIPI_sensor.cp_dummy_pixels-164;
	  frame_length = HM8030MIPI_sensor.cp_dummy_lines-97;
    }
	else
	{
	     SENSORDB("[HM8030MIPI]%s(),sensor mode error",__FUNCTION__);
	}   
	  dummy_tmp0=line_length%16;
	  dummy_tmp1=line_length/16;
      //HM8030MIPI_write_cmos_sensor(0x0010, (frame_length >>8) & 0xFF);
      //HM8030MIPI_write_cmos_sensor(0x0011, frame_length & 0xFF);	
      //HM8030MIPI_write_cmos_sensor(0x0012, (dummy_tmp0 & 0xFF));
      //HM8030MIPI_write_cmos_sensor(0x0013, (dummy_tmp1 & 0xFF));
	  SENSORDB("[HM8030MIPI]exit HM8030MIPI_SetDummy function\n");
}   /*  HM8030MIPI_SetDummy */
static void HM8030MIPI_Sensor_Init(void)
{
	SENSORDB("[HM8030MIPI]enter HM8030MIPI_Sensor_Init function\n");
		HM8030MIPI_write_cmos_sensor(0x0103,0x01);//Software reset	 
    HM8030MIPI_write_cmos_sensor(0xffff,0x01);
    HM8030MIPI_write_cmos_sensor(0x8500,0x03);
    HM8030MIPI_write_cmos_sensor(0x9000,0xF0);
    HM8030MIPI_write_cmos_sensor(0x9001,0x90);
    HM8030MIPI_write_cmos_sensor(0x9002,0x52);
    HM8030MIPI_write_cmos_sensor(0x9003,0x00);
    HM8030MIPI_write_cmos_sensor(0x9004,0xF0);
    HM8030MIPI_write_cmos_sensor(0x9005,0x02);
    HM8030MIPI_write_cmos_sensor(0x9006,0x13);
    HM8030MIPI_write_cmos_sensor(0x9007,0x1A);
    HM8030MIPI_write_cmos_sensor(0x9008,0x90);
    HM8030MIPI_write_cmos_sensor(0x9009,0x30);
    HM8030MIPI_write_cmos_sensor(0x900A,0x6c);
    HM8030MIPI_write_cmos_sensor(0x900B,0xE0);
    HM8030MIPI_write_cmos_sensor(0x900C,0x90);
    HM8030MIPI_write_cmos_sensor(0x900D,0x00);
    HM8030MIPI_write_cmos_sensor(0x900E,0x02);
    HM8030MIPI_write_cmos_sensor(0x900F,0xF0);
    HM8030MIPI_write_cmos_sensor(0x9010,0x22);
    HM8030MIPI_write_cmos_sensor(0x9011,0x24);
    HM8030MIPI_write_cmos_sensor(0x9012,0xFE);
    HM8030MIPI_write_cmos_sensor(0x9013,0xF0);
    HM8030MIPI_write_cmos_sensor(0x9014,0x90);
    HM8030MIPI_write_cmos_sensor(0x9015,0xA5);
    HM8030MIPI_write_cmos_sensor(0x9016,0xB5);
    HM8030MIPI_write_cmos_sensor(0x9017,0xE0);
    HM8030MIPI_write_cmos_sensor(0x9018,0x34);
    HM8030MIPI_write_cmos_sensor(0x9019,0xFF);
    HM8030MIPI_write_cmos_sensor(0x901A,0x02);
    HM8030MIPI_write_cmos_sensor(0x901B,0x12);
    HM8030MIPI_write_cmos_sensor(0x901C,0x3F);
    HM8030MIPI_write_cmos_sensor(0x901D,0x90);
    HM8030MIPI_write_cmos_sensor(0x901E,0xA6);
    HM8030MIPI_write_cmos_sensor(0x901F,0x49);
    HM8030MIPI_write_cmos_sensor(0x9020,0xE0);
    HM8030MIPI_write_cmos_sensor(0x9021,0xFF);
    HM8030MIPI_write_cmos_sensor(0x9022,0x64);
    HM8030MIPI_write_cmos_sensor(0x9023,0x01);
    HM8030MIPI_write_cmos_sensor(0x9024,0x60);
    HM8030MIPI_write_cmos_sensor(0x9025,0x05);
    HM8030MIPI_write_cmos_sensor(0x9026,0xEF);
    HM8030MIPI_write_cmos_sensor(0x9027,0x64);
    HM8030MIPI_write_cmos_sensor(0x9028,0x04);
    HM8030MIPI_write_cmos_sensor(0x9029,0x70);
    HM8030MIPI_write_cmos_sensor(0x902A,0x7D);
    HM8030MIPI_write_cmos_sensor(0x902B,0x90);
    HM8030MIPI_write_cmos_sensor(0x902C,0xA5);
    HM8030MIPI_write_cmos_sensor(0x902D,0xB9);
    HM8030MIPI_write_cmos_sensor(0x902E,0xE0);
    HM8030MIPI_write_cmos_sensor(0x902F,0xFF);
    HM8030MIPI_write_cmos_sensor(0x9030,0xC3);
    HM8030MIPI_write_cmos_sensor(0x9031,0x94);
    HM8030MIPI_write_cmos_sensor(0x9032,0x80);
    HM8030MIPI_write_cmos_sensor(0x9033,0x50);
    HM8030MIPI_write_cmos_sensor(0x9034,0x13);
    HM8030MIPI_write_cmos_sensor(0x9035,0xA3);
    HM8030MIPI_write_cmos_sensor(0x9036,0x74);
    HM8030MIPI_write_cmos_sensor(0x9037,0x4D);
    HM8030MIPI_write_cmos_sensor(0x9038,0xF0);
    HM8030MIPI_write_cmos_sensor(0x9039,0xA3);
    HM8030MIPI_write_cmos_sensor(0x903A,0x74);
    HM8030MIPI_write_cmos_sensor(0x903B,0xB3);
    HM8030MIPI_write_cmos_sensor(0x903C,0xF0);
    HM8030MIPI_write_cmos_sensor(0x903D,0xA3);
    HM8030MIPI_write_cmos_sensor(0x903E,0x74);
    HM8030MIPI_write_cmos_sensor(0x903F,0x40);
    HM8030MIPI_write_cmos_sensor(0x9040,0xF0);
    HM8030MIPI_write_cmos_sensor(0x9041,0xA3);
    HM8030MIPI_write_cmos_sensor(0x9042,0x74);
    HM8030MIPI_write_cmos_sensor(0x9043,0x9B);
    HM8030MIPI_write_cmos_sensor(0x9044,0xF0);
    HM8030MIPI_write_cmos_sensor(0x9045,0x02);
    HM8030MIPI_write_cmos_sensor(0x9046,0x51);
    HM8030MIPI_write_cmos_sensor(0x9047,0x1F);
    HM8030MIPI_write_cmos_sensor(0x9048,0xEF);
    HM8030MIPI_write_cmos_sensor(0x9049,0xC3);
    HM8030MIPI_write_cmos_sensor(0x904A,0x94);
    HM8030MIPI_write_cmos_sensor(0x904B,0xC0);
    HM8030MIPI_write_cmos_sensor(0x904C,0x50);
    HM8030MIPI_write_cmos_sensor(0x904D,0x15);
    HM8030MIPI_write_cmos_sensor(0x904E,0x90);
    HM8030MIPI_write_cmos_sensor(0x904F,0xA5);
    HM8030MIPI_write_cmos_sensor(0x9050,0xBA);
    HM8030MIPI_write_cmos_sensor(0x9051,0x74);
    HM8030MIPI_write_cmos_sensor(0x9052,0x4D);
    HM8030MIPI_write_cmos_sensor(0x9053,0xF0);
    HM8030MIPI_write_cmos_sensor(0x9054,0xA3);
    HM8030MIPI_write_cmos_sensor(0x9055,0x74);
    HM8030MIPI_write_cmos_sensor(0x9056,0xCD);
    HM8030MIPI_write_cmos_sensor(0x9057,0xF0);
    HM8030MIPI_write_cmos_sensor(0x9058,0xA3);
    HM8030MIPI_write_cmos_sensor(0x9059,0x74);
    HM8030MIPI_write_cmos_sensor(0x905A,0x40);
    HM8030MIPI_write_cmos_sensor(0x905B,0xF0);
    HM8030MIPI_write_cmos_sensor(0x905C,0xA3);
    HM8030MIPI_write_cmos_sensor(0x905D,0x74);
    HM8030MIPI_write_cmos_sensor(0x905E,0x8D);
    HM8030MIPI_write_cmos_sensor(0x905F,0xF0);
    HM8030MIPI_write_cmos_sensor(0x9060,0x02);
    HM8030MIPI_write_cmos_sensor(0x9061,0x51);
    HM8030MIPI_write_cmos_sensor(0x9062,0x1F);
    HM8030MIPI_write_cmos_sensor(0x9063,0xEF);
    HM8030MIPI_write_cmos_sensor(0x9064,0xC3);
    HM8030MIPI_write_cmos_sensor(0x9065,0x94);
    HM8030MIPI_write_cmos_sensor(0x9066,0xE0);
    HM8030MIPI_write_cmos_sensor(0x9067,0x50);
    HM8030MIPI_write_cmos_sensor(0x9068,0x15);
    HM8030MIPI_write_cmos_sensor(0x9069,0x90);
    HM8030MIPI_write_cmos_sensor(0x906A,0xA5);
    HM8030MIPI_write_cmos_sensor(0x906B,0xBA);
    HM8030MIPI_write_cmos_sensor(0x906C,0x74);
    HM8030MIPI_write_cmos_sensor(0x906D,0x4D);
    HM8030MIPI_write_cmos_sensor(0x906E,0xF0);
    HM8030MIPI_write_cmos_sensor(0x906F,0xA3);
    HM8030MIPI_write_cmos_sensor(0x9070,0x74);
    HM8030MIPI_write_cmos_sensor(0x9071,0xF3);
    HM8030MIPI_write_cmos_sensor(0x9072,0xF0);
    HM8030MIPI_write_cmos_sensor(0x9073,0xA3);
    HM8030MIPI_write_cmos_sensor(0x9074,0x74);
    HM8030MIPI_write_cmos_sensor(0x9075,0x40);
    HM8030MIPI_write_cmos_sensor(0x9076,0xF0);
    HM8030MIPI_write_cmos_sensor(0x9077,0xA3);
    HM8030MIPI_write_cmos_sensor(0x9078,0x74);
    HM8030MIPI_write_cmos_sensor(0x9079,0x88);
    HM8030MIPI_write_cmos_sensor(0x907A,0xF0);
    HM8030MIPI_write_cmos_sensor(0x907B,0x02);
    HM8030MIPI_write_cmos_sensor(0x907C,0x51);
    HM8030MIPI_write_cmos_sensor(0x907D,0x1F);
    HM8030MIPI_write_cmos_sensor(0x907E,0xEF);
    HM8030MIPI_write_cmos_sensor(0x907F,0xC3);
    HM8030MIPI_write_cmos_sensor(0x9080,0x94);
    HM8030MIPI_write_cmos_sensor(0x9081,0xF0);
    HM8030MIPI_write_cmos_sensor(0x9082,0x90);
    HM8030MIPI_write_cmos_sensor(0x9083,0xA5);
    HM8030MIPI_write_cmos_sensor(0x9084,0xBA);
    HM8030MIPI_write_cmos_sensor(0x9085,0x74);
    HM8030MIPI_write_cmos_sensor(0x9086,0x4E);
    HM8030MIPI_write_cmos_sensor(0x9087,0x50);
    HM8030MIPI_write_cmos_sensor(0x9088,0x10);
    HM8030MIPI_write_cmos_sensor(0x9089,0xF0);
    HM8030MIPI_write_cmos_sensor(0x908A,0xA3);
    HM8030MIPI_write_cmos_sensor(0x908B,0x74);
    HM8030MIPI_write_cmos_sensor(0x908C,0x16);
    HM8030MIPI_write_cmos_sensor(0x908D,0xF0);
    HM8030MIPI_write_cmos_sensor(0x908E,0xA3);
    HM8030MIPI_write_cmos_sensor(0x908F,0x74);
    HM8030MIPI_write_cmos_sensor(0x9090,0x40);
    HM8030MIPI_write_cmos_sensor(0x9091,0xF0);
    HM8030MIPI_write_cmos_sensor(0x9092,0xA3);
    HM8030MIPI_write_cmos_sensor(0x9093,0x74);
    HM8030MIPI_write_cmos_sensor(0x9094,0x78);
    HM8030MIPI_write_cmos_sensor(0x9095,0xF0);
    HM8030MIPI_write_cmos_sensor(0x9096,0x02);
    HM8030MIPI_write_cmos_sensor(0x9097,0x51);
    HM8030MIPI_write_cmos_sensor(0x9098,0x1F);
    HM8030MIPI_write_cmos_sensor(0x9099,0xF0);
    HM8030MIPI_write_cmos_sensor(0x909A,0xA3);
    HM8030MIPI_write_cmos_sensor(0x909B,0x74);
    HM8030MIPI_write_cmos_sensor(0x909C,0x3C);
    HM8030MIPI_write_cmos_sensor(0x909D,0xF0);
    HM8030MIPI_write_cmos_sensor(0x909E,0xA3);
    HM8030MIPI_write_cmos_sensor(0x909F,0x74);
    HM8030MIPI_write_cmos_sensor(0x90A0,0x40);
    HM8030MIPI_write_cmos_sensor(0x90A1,0xF0);
    HM8030MIPI_write_cmos_sensor(0x90A2,0xA3);
    HM8030MIPI_write_cmos_sensor(0x90A3,0x74);
    HM8030MIPI_write_cmos_sensor(0x90A4,0x7E);
    HM8030MIPI_write_cmos_sensor(0x90A5,0xF0);
    HM8030MIPI_write_cmos_sensor(0x90A6,0x80);
    HM8030MIPI_write_cmos_sensor(0x90A7,0x77);
    HM8030MIPI_write_cmos_sensor(0x90A8,0x90);
    HM8030MIPI_write_cmos_sensor(0x90A9,0xA5);
    HM8030MIPI_write_cmos_sensor(0x90AA,0xB9);
    HM8030MIPI_write_cmos_sensor(0x90AB,0xE0);
    HM8030MIPI_write_cmos_sensor(0x90AC,0xFF);
    HM8030MIPI_write_cmos_sensor(0x90AD,0xC3);
    HM8030MIPI_write_cmos_sensor(0x90AE,0x94);
    HM8030MIPI_write_cmos_sensor(0x90AF,0x80);
    HM8030MIPI_write_cmos_sensor(0x90B0,0x50);
    HM8030MIPI_write_cmos_sensor(0x90B1,0x12);
    HM8030MIPI_write_cmos_sensor(0x90B2,0xA3);
    HM8030MIPI_write_cmos_sensor(0x90B3,0x74);
    HM8030MIPI_write_cmos_sensor(0x90B4,0x4B);
    HM8030MIPI_write_cmos_sensor(0x90B5,0xF0);
    HM8030MIPI_write_cmos_sensor(0x90B6,0xA3);
    HM8030MIPI_write_cmos_sensor(0x90B7,0x74);
    HM8030MIPI_write_cmos_sensor(0x90B8,0xAA);
    HM8030MIPI_write_cmos_sensor(0x90B9,0xF0);
    HM8030MIPI_write_cmos_sensor(0x90BA,0xA3);
    HM8030MIPI_write_cmos_sensor(0x90BB,0x74);
    HM8030MIPI_write_cmos_sensor(0x90BC,0x40);
    HM8030MIPI_write_cmos_sensor(0x90BD,0xF0);
    HM8030MIPI_write_cmos_sensor(0x90BE,0xA3);
    HM8030MIPI_write_cmos_sensor(0x90BF,0x74);
    HM8030MIPI_write_cmos_sensor(0x90C0,0x9A);
    HM8030MIPI_write_cmos_sensor(0x90C1,0xF0);
    HM8030MIPI_write_cmos_sensor(0x90C2,0x80);
    HM8030MIPI_write_cmos_sensor(0x90C3,0x5B);
    HM8030MIPI_write_cmos_sensor(0x90C4,0xEF);
    HM8030MIPI_write_cmos_sensor(0x90C5,0xC3);
    HM8030MIPI_write_cmos_sensor(0x90C6,0x94);
    HM8030MIPI_write_cmos_sensor(0x90C7,0xC0);
    HM8030MIPI_write_cmos_sensor(0x90C8,0x50);
    HM8030MIPI_write_cmos_sensor(0x90C9,0x14);
    HM8030MIPI_write_cmos_sensor(0x90CA,0x90);
    HM8030MIPI_write_cmos_sensor(0x90CB,0xA5);
    HM8030MIPI_write_cmos_sensor(0x90CC,0xBA);
    HM8030MIPI_write_cmos_sensor(0x90CD,0x74);
    HM8030MIPI_write_cmos_sensor(0x90CE,0x4B);
    HM8030MIPI_write_cmos_sensor(0x90CF,0xF0);
    HM8030MIPI_write_cmos_sensor(0x90D0,0xA3);
    HM8030MIPI_write_cmos_sensor(0x90D1,0x74);
    HM8030MIPI_write_cmos_sensor(0x90D2,0xC4);
    HM8030MIPI_write_cmos_sensor(0x90D3,0xF0);
    HM8030MIPI_write_cmos_sensor(0x90D4,0xA3);
    HM8030MIPI_write_cmos_sensor(0x90D5,0x74);
    HM8030MIPI_write_cmos_sensor(0x90D6,0x40);
    HM8030MIPI_write_cmos_sensor(0x90D7,0xF0);
    HM8030MIPI_write_cmos_sensor(0x90D8,0xA3);
    HM8030MIPI_write_cmos_sensor(0x90D9,0x74);
    HM8030MIPI_write_cmos_sensor(0x90DA,0x9C);
    HM8030MIPI_write_cmos_sensor(0x90DB,0xF0);
    HM8030MIPI_write_cmos_sensor(0x90DC,0x80);
    HM8030MIPI_write_cmos_sensor(0x90DD,0x41);
    HM8030MIPI_write_cmos_sensor(0x90DE,0xEF);
    HM8030MIPI_write_cmos_sensor(0x90DF,0xC3);
    HM8030MIPI_write_cmos_sensor(0x90E0,0x94);
    HM8030MIPI_write_cmos_sensor(0x90E1,0xE0);
    HM8030MIPI_write_cmos_sensor(0x90E2,0x50);
    HM8030MIPI_write_cmos_sensor(0x90E3,0x14);
    HM8030MIPI_write_cmos_sensor(0x90E4,0x90);
    HM8030MIPI_write_cmos_sensor(0x90E5,0xA5);
    HM8030MIPI_write_cmos_sensor(0x90E6,0xBA);
    HM8030MIPI_write_cmos_sensor(0x90E7,0x74);
    HM8030MIPI_write_cmos_sensor(0x90E8,0x4B);
    HM8030MIPI_write_cmos_sensor(0x90E9,0xF0);
    HM8030MIPI_write_cmos_sensor(0x90EA,0xA3);
    HM8030MIPI_write_cmos_sensor(0x90EB,0x74);
    HM8030MIPI_write_cmos_sensor(0x90EC,0xEB);
    HM8030MIPI_write_cmos_sensor(0x90ED,0xF0);
    HM8030MIPI_write_cmos_sensor(0x90EE,0xA3);
    HM8030MIPI_write_cmos_sensor(0x90EF,0x74);
    HM8030MIPI_write_cmos_sensor(0x90F0,0x40);
    HM8030MIPI_write_cmos_sensor(0x90F1,0xF0);
    HM8030MIPI_write_cmos_sensor(0x90F2,0xA3);
    HM8030MIPI_write_cmos_sensor(0x90F3,0x74);
    HM8030MIPI_write_cmos_sensor(0x90F4,0x8D);
    HM8030MIPI_write_cmos_sensor(0x90F5,0xF0);
    HM8030MIPI_write_cmos_sensor(0x90F6,0x80);
    HM8030MIPI_write_cmos_sensor(0x90F7,0x27);
    HM8030MIPI_write_cmos_sensor(0x90F8,0xEF);
    HM8030MIPI_write_cmos_sensor(0x90F9,0xC3);
    HM8030MIPI_write_cmos_sensor(0x90FA,0x94);
    HM8030MIPI_write_cmos_sensor(0x90FB,0xF0);
    HM8030MIPI_write_cmos_sensor(0x90FC,0x90);
    HM8030MIPI_write_cmos_sensor(0x90FD,0xA5);
    HM8030MIPI_write_cmos_sensor(0x90FE,0xBA);
    HM8030MIPI_write_cmos_sensor(0x90FF,0x74);
    HM8030MIPI_write_cmos_sensor(0x9100,0x4C);
    HM8030MIPI_write_cmos_sensor(0x9101,0x50);
    HM8030MIPI_write_cmos_sensor(0x9102,0x0F);
    HM8030MIPI_write_cmos_sensor(0x9103,0xF0);
    HM8030MIPI_write_cmos_sensor(0x9104,0xA3);
    HM8030MIPI_write_cmos_sensor(0x9105,0x74);
    HM8030MIPI_write_cmos_sensor(0x9106,0x0F);
    HM8030MIPI_write_cmos_sensor(0x9107,0xF0);
    HM8030MIPI_write_cmos_sensor(0x9108,0xA3);
    HM8030MIPI_write_cmos_sensor(0x9109,0x74);
    HM8030MIPI_write_cmos_sensor(0x910A,0x40);
    HM8030MIPI_write_cmos_sensor(0x910B,0xF0);
    HM8030MIPI_write_cmos_sensor(0x910C,0xA3);
    HM8030MIPI_write_cmos_sensor(0x910D,0x74);
    HM8030MIPI_write_cmos_sensor(0x910E,0x8A);
    HM8030MIPI_write_cmos_sensor(0x910F,0xF0);
    HM8030MIPI_write_cmos_sensor(0x9110,0x80);
    HM8030MIPI_write_cmos_sensor(0x9111,0x0D);
    HM8030MIPI_write_cmos_sensor(0x9112,0xF0);
    HM8030MIPI_write_cmos_sensor(0x9113,0xA3);
    HM8030MIPI_write_cmos_sensor(0x9114,0x74);
    HM8030MIPI_write_cmos_sensor(0x9115,0x1E);
    HM8030MIPI_write_cmos_sensor(0x9116,0xF0);
    HM8030MIPI_write_cmos_sensor(0x9117,0xA3);
    HM8030MIPI_write_cmos_sensor(0x9118,0x74);
    HM8030MIPI_write_cmos_sensor(0x9119,0x40);
    HM8030MIPI_write_cmos_sensor(0x911A,0xF0);
    HM8030MIPI_write_cmos_sensor(0x911B,0xA3);
    HM8030MIPI_write_cmos_sensor(0x911C,0x74);
    HM8030MIPI_write_cmos_sensor(0x911D,0xA4);
    HM8030MIPI_write_cmos_sensor(0x911E,0xF0);
    HM8030MIPI_write_cmos_sensor(0x911F,0x90);
    HM8030MIPI_write_cmos_sensor(0x9120,0xA5);
    HM8030MIPI_write_cmos_sensor(0x9121,0xB5);
    HM8030MIPI_write_cmos_sensor(0x9122,0x02);
    HM8030MIPI_write_cmos_sensor(0x9123,0x11);
    HM8030MIPI_write_cmos_sensor(0x9124,0xA0);
    HM8030MIPI_write_cmos_sensor(0x9125,0x02);
    HM8030MIPI_write_cmos_sensor(0x9126,0x12);
    HM8030MIPI_write_cmos_sensor(0x9127,0x4F);
    HM8030MIPI_write_cmos_sensor(0x9128,0x74);
    HM8030MIPI_write_cmos_sensor(0x9129,0x00);
    HM8030MIPI_write_cmos_sensor(0x912A,0xF0);
    HM8030MIPI_write_cmos_sensor(0x912B,0x90);
    HM8030MIPI_write_cmos_sensor(0x912C,0xA6);
    HM8030MIPI_write_cmos_sensor(0x912D,0x7D);
    HM8030MIPI_write_cmos_sensor(0x912E,0x74);
    HM8030MIPI_write_cmos_sensor(0x912F,0x02);
    HM8030MIPI_write_cmos_sensor(0x9130,0xF0);
    HM8030MIPI_write_cmos_sensor(0x9131,0x90);
    HM8030MIPI_write_cmos_sensor(0x9132,0xA6);
    HM8030MIPI_write_cmos_sensor(0x9133,0x7F);
    HM8030MIPI_write_cmos_sensor(0x9134,0xF0);
    HM8030MIPI_write_cmos_sensor(0x9135,0x90);
    HM8030MIPI_write_cmos_sensor(0x9136,0x31);
    HM8030MIPI_write_cmos_sensor(0x9137,0x27);
    HM8030MIPI_write_cmos_sensor(0x9138,0x74);
    HM8030MIPI_write_cmos_sensor(0x9139,0x32);
    HM8030MIPI_write_cmos_sensor(0x913A,0xF0);
    HM8030MIPI_write_cmos_sensor(0x913B,0x02);
    HM8030MIPI_write_cmos_sensor(0x913C,0x0B);
    HM8030MIPI_write_cmos_sensor(0x913D,0x58);
    HM8030MIPI_write_cmos_sensor(0x913E,0xE0);
    HM8030MIPI_write_cmos_sensor(0x913F,0xFE);
    HM8030MIPI_write_cmos_sensor(0x9140,0xA3);
    HM8030MIPI_write_cmos_sensor(0x9141,0xE0);
    HM8030MIPI_write_cmos_sensor(0x9142,0xFF);
    HM8030MIPI_write_cmos_sensor(0x9143,0x90);
    HM8030MIPI_write_cmos_sensor(0x9144,0xA6);
    HM8030MIPI_write_cmos_sensor(0x9145,0x24);
    HM8030MIPI_write_cmos_sensor(0x9146,0xEE);
    HM8030MIPI_write_cmos_sensor(0x9147,0xF0);
    HM8030MIPI_write_cmos_sensor(0x9148,0xA3);
    HM8030MIPI_write_cmos_sensor(0x9149,0xEF);
    HM8030MIPI_write_cmos_sensor(0x914A,0xF0);
    HM8030MIPI_write_cmos_sensor(0x914B,0xC3);
    HM8030MIPI_write_cmos_sensor(0x914C,0xEE);
    HM8030MIPI_write_cmos_sensor(0x914D,0x64);
    HM8030MIPI_write_cmos_sensor(0x914E,0x80);
    HM8030MIPI_write_cmos_sensor(0x914F,0x94);
    HM8030MIPI_write_cmos_sensor(0x9150,0x80);
    HM8030MIPI_write_cmos_sensor(0x9151,0x50);
    HM8030MIPI_write_cmos_sensor(0x9152,0x0F);
    HM8030MIPI_write_cmos_sensor(0x9153,0x90);
    HM8030MIPI_write_cmos_sensor(0x9154,0xA5);
    HM8030MIPI_write_cmos_sensor(0x9155,0xCA);
    HM8030MIPI_write_cmos_sensor(0x9156,0xE0);
    HM8030MIPI_write_cmos_sensor(0x9157,0xFF);
    HM8030MIPI_write_cmos_sensor(0x9158,0xA3);
    HM8030MIPI_write_cmos_sensor(0x9159,0xE0);
    HM8030MIPI_write_cmos_sensor(0x915A,0x90);
    HM8030MIPI_write_cmos_sensor(0x915B,0xA6);
    HM8030MIPI_write_cmos_sensor(0x915C,0x24);
    HM8030MIPI_write_cmos_sensor(0x915D,0xCF);
    HM8030MIPI_write_cmos_sensor(0x915E,0xF0);
    HM8030MIPI_write_cmos_sensor(0x915F,0xA3);
    HM8030MIPI_write_cmos_sensor(0x9160,0xEF);
    HM8030MIPI_write_cmos_sensor(0x9161,0xF0);
    HM8030MIPI_write_cmos_sensor(0x9162,0x90);
    HM8030MIPI_write_cmos_sensor(0x9163,0xA5);
    HM8030MIPI_write_cmos_sensor(0x9164,0xCE);
    HM8030MIPI_write_cmos_sensor(0x9165,0xE0);
    HM8030MIPI_write_cmos_sensor(0x9166,0xFA);
    HM8030MIPI_write_cmos_sensor(0x9167,0xA3);
    HM8030MIPI_write_cmos_sensor(0x9168,0xE0);
    HM8030MIPI_write_cmos_sensor(0x9169,0xFB);
    HM8030MIPI_write_cmos_sensor(0x916A,0xF5);
    HM8030MIPI_write_cmos_sensor(0x916B,0x82);
    HM8030MIPI_write_cmos_sensor(0x916C,0x8A);
    HM8030MIPI_write_cmos_sensor(0x916D,0x83);
    HM8030MIPI_write_cmos_sensor(0x916E,0xA3);
    HM8030MIPI_write_cmos_sensor(0x916F,0xA3);
    HM8030MIPI_write_cmos_sensor(0x9170,0xE0);
    HM8030MIPI_write_cmos_sensor(0x9171,0xFE);
    HM8030MIPI_write_cmos_sensor(0x9172,0xA3);
    HM8030MIPI_write_cmos_sensor(0x9173,0xE0);
    HM8030MIPI_write_cmos_sensor(0x9174,0xFF);
    HM8030MIPI_write_cmos_sensor(0x9175,0x90);
    HM8030MIPI_write_cmos_sensor(0x9176,0xA6);
    HM8030MIPI_write_cmos_sensor(0x9177,0x1E);
    HM8030MIPI_write_cmos_sensor(0x9178,0xEE);
    HM8030MIPI_write_cmos_sensor(0x9179,0xF0);
    HM8030MIPI_write_cmos_sensor(0x917A,0xA3);
    HM8030MIPI_write_cmos_sensor(0x917B,0xEF);
    HM8030MIPI_write_cmos_sensor(0x917C,0xF0);
    HM8030MIPI_write_cmos_sensor(0x917D,0x4E);
    HM8030MIPI_write_cmos_sensor(0x917E,0x70);
    HM8030MIPI_write_cmos_sensor(0x917F,0x18);
    HM8030MIPI_write_cmos_sensor(0x9180,0x90);
    HM8030MIPI_write_cmos_sensor(0x9181,0xA6);
    HM8030MIPI_write_cmos_sensor(0x9182,0x24);
    HM8030MIPI_write_cmos_sensor(0x9183,0xE0);
    HM8030MIPI_write_cmos_sensor(0x9184,0xFE);
    HM8030MIPI_write_cmos_sensor(0x9185,0xA3);
    HM8030MIPI_write_cmos_sensor(0x9186,0xE0);
    HM8030MIPI_write_cmos_sensor(0x9187,0xFF);
    HM8030MIPI_write_cmos_sensor(0x9188,0x90);
    HM8030MIPI_write_cmos_sensor(0x9189,0xA6);
    HM8030MIPI_write_cmos_sensor(0x918A,0x24);
    HM8030MIPI_write_cmos_sensor(0x918B,0xEE);
    HM8030MIPI_write_cmos_sensor(0x918C,0x00);
    HM8030MIPI_write_cmos_sensor(0x918D,0xA3);
    HM8030MIPI_write_cmos_sensor(0x918E,0xEF);
    HM8030MIPI_write_cmos_sensor(0x918F,0x00);
    HM8030MIPI_write_cmos_sensor(0x9190,0x90);
    HM8030MIPI_write_cmos_sensor(0x9191,0xA6);
    HM8030MIPI_write_cmos_sensor(0x9192,0x1E);
    HM8030MIPI_write_cmos_sensor(0x9193,0xEE);
    HM8030MIPI_write_cmos_sensor(0x9194,0xF0);
    HM8030MIPI_write_cmos_sensor(0x9195,0xA3);
    HM8030MIPI_write_cmos_sensor(0x9196,0xEF);
    HM8030MIPI_write_cmos_sensor(0x9197,0xF0);
    HM8030MIPI_write_cmos_sensor(0x9198,0x90);
    HM8030MIPI_write_cmos_sensor(0x9199,0xA6);
    HM8030MIPI_write_cmos_sensor(0x919A,0x1E);
    HM8030MIPI_write_cmos_sensor(0x919B,0xE0);
    HM8030MIPI_write_cmos_sensor(0x919C,0xFC);
    HM8030MIPI_write_cmos_sensor(0x919D,0xA3);
    HM8030MIPI_write_cmos_sensor(0x919E,0xE0);
    HM8030MIPI_write_cmos_sensor(0x919F,0xFD);
    HM8030MIPI_write_cmos_sensor(0x91A0,0xAE);
    HM8030MIPI_write_cmos_sensor(0x91A1,0x04);
    HM8030MIPI_write_cmos_sensor(0x91A2,0x78);
HM8030MIPI_write_cmos_sensor(0x91A3, 0x04);
    HM8030MIPI_write_cmos_sensor(0x91A4,0xCE);
    HM8030MIPI_write_cmos_sensor(0x91A5,0xC3);
    HM8030MIPI_write_cmos_sensor(0x91A6,0x13);
    HM8030MIPI_write_cmos_sensor(0x91A7,0xCE);
    HM8030MIPI_write_cmos_sensor(0x91A8,0x13);
    HM8030MIPI_write_cmos_sensor(0x91A9,0xD8);
    HM8030MIPI_write_cmos_sensor(0x91AA,0xF9);
    HM8030MIPI_write_cmos_sensor(0x91AB,0xFF);
    HM8030MIPI_write_cmos_sensor(0x91AC,0xC3);
    HM8030MIPI_write_cmos_sensor(0x91AD,0xED);
    HM8030MIPI_write_cmos_sensor(0x91AE,0x9F);
    HM8030MIPI_write_cmos_sensor(0x91AF,0xFD);
    HM8030MIPI_write_cmos_sensor(0x91B0,0xEC);
    HM8030MIPI_write_cmos_sensor(0x91B1,0x9E);
    HM8030MIPI_write_cmos_sensor(0x91B2,0xFC);
    HM8030MIPI_write_cmos_sensor(0x91B3,0x90);
    HM8030MIPI_write_cmos_sensor(0x91B4,0xA6);
    HM8030MIPI_write_cmos_sensor(0x91B5,0x24);
    HM8030MIPI_write_cmos_sensor(0x91B6,0xE0);
    HM8030MIPI_write_cmos_sensor(0x91B7,0xFE);
    HM8030MIPI_write_cmos_sensor(0x91B8,0xA3);
    HM8030MIPI_write_cmos_sensor(0x91B9,0xE0);
    HM8030MIPI_write_cmos_sensor(0x91BA,0x78);
HM8030MIPI_write_cmos_sensor(0x91BB, 0x04);
    HM8030MIPI_write_cmos_sensor(0x91BC,0xCE);
    HM8030MIPI_write_cmos_sensor(0x91BD,0xC3);
    HM8030MIPI_write_cmos_sensor(0x91BE,0x13);
    HM8030MIPI_write_cmos_sensor(0x91BF,0xCE);
    HM8030MIPI_write_cmos_sensor(0x91C0,0x13);
    HM8030MIPI_write_cmos_sensor(0x91C1,0xD8);
    HM8030MIPI_write_cmos_sensor(0x91C2,0xF9);
    HM8030MIPI_write_cmos_sensor(0x91C3,0x2D);
    HM8030MIPI_write_cmos_sensor(0x91C4,0xFF);
    HM8030MIPI_write_cmos_sensor(0x91C5,0xEC);
    HM8030MIPI_write_cmos_sensor(0x91C6,0x3E);
    HM8030MIPI_write_cmos_sensor(0x91C7,0xFE);
    HM8030MIPI_write_cmos_sensor(0x91C8,0xE4);
    HM8030MIPI_write_cmos_sensor(0x91C9,0xFC);
    HM8030MIPI_write_cmos_sensor(0x91CA,0xFD);
    HM8030MIPI_write_cmos_sensor(0x91CB,0x8B);
    HM8030MIPI_write_cmos_sensor(0x91CC,0x82);
    HM8030MIPI_write_cmos_sensor(0x91CD,0x8A);
    HM8030MIPI_write_cmos_sensor(0x91CE,0x83);
    HM8030MIPI_write_cmos_sensor(0x91CF,0x02);
    HM8030MIPI_write_cmos_sensor(0x91D0,0x12);
    HM8030MIPI_write_cmos_sensor(0x91D1,0x96);
    HM8030MIPI_write_cmos_sensor(0x91d6,0x74);
    HM8030MIPI_write_cmos_sensor(0x91d7,0x00);
    HM8030MIPI_write_cmos_sensor(0x91d8,0x90);
    HM8030MIPI_write_cmos_sensor(0x91d9,0x42);
    HM8030MIPI_write_cmos_sensor(0x91dA,0x04);
    HM8030MIPI_write_cmos_sensor(0x91dB,0xF0);
    HM8030MIPI_write_cmos_sensor(0x91dC,0xA3);
    HM8030MIPI_write_cmos_sensor(0x91dD,0xF0);
    HM8030MIPI_write_cmos_sensor(0x91dE,0xA3);
    HM8030MIPI_write_cmos_sensor(0x91dF,0xA3);
    HM8030MIPI_write_cmos_sensor(0x91e0,0xA3);
    HM8030MIPI_write_cmos_sensor(0x91e1,0xF0);
    HM8030MIPI_write_cmos_sensor(0x91e2,0xA3);
    HM8030MIPI_write_cmos_sensor(0x91e3,0xF0);
    HM8030MIPI_write_cmos_sensor(0x91e4,0xA3);
    HM8030MIPI_write_cmos_sensor(0x91e5,0xA3);
    HM8030MIPI_write_cmos_sensor(0x91e6,0xA3);
    HM8030MIPI_write_cmos_sensor(0x91e7,0xF0);
    HM8030MIPI_write_cmos_sensor(0x91e8,0xA3);
    HM8030MIPI_write_cmos_sensor(0x91e9,0xF0);
    HM8030MIPI_write_cmos_sensor(0x91eA,0xA3);
    HM8030MIPI_write_cmos_sensor(0x91eB,0xA3);
    HM8030MIPI_write_cmos_sensor(0x91eC,0xA3);
    HM8030MIPI_write_cmos_sensor(0x91eD,0xF0);
    HM8030MIPI_write_cmos_sensor(0x91eE,0xA3);
    HM8030MIPI_write_cmos_sensor(0x91eF,0xF0);
    HM8030MIPI_write_cmos_sensor(0x91f0,0x90);
    HM8030MIPI_write_cmos_sensor(0x91f1,0x42);
    HM8030MIPI_write_cmos_sensor(0x91f2,0x00);
    HM8030MIPI_write_cmos_sensor(0x91f3,0x74);
    HM8030MIPI_write_cmos_sensor(0x91f4,0x0D);
    HM8030MIPI_write_cmos_sensor(0x91f5,0xF0);
    HM8030MIPI_write_cmos_sensor(0x91f6,0x22);
    HM8030MIPI_write_cmos_sensor(0x91f7,0x90);
    HM8030MIPI_write_cmos_sensor(0x91f8,0xA3);
    HM8030MIPI_write_cmos_sensor(0x91f9,0xE5);
    HM8030MIPI_write_cmos_sensor(0x91fa,0xE0);
    HM8030MIPI_write_cmos_sensor(0x91fb,0xB4);
    HM8030MIPI_write_cmos_sensor(0x91fc,0x03);
    HM8030MIPI_write_cmos_sensor(0x91fd,0x0F);
    HM8030MIPI_write_cmos_sensor(0x91fe,0xA3);
    HM8030MIPI_write_cmos_sensor(0x91ff,0xE0);
    HM8030MIPI_write_cmos_sensor(0x9200,0xB4);
    HM8030MIPI_write_cmos_sensor(0x9201,0x02);
    HM8030MIPI_write_cmos_sensor(0x9202,0x0A);
    HM8030MIPI_write_cmos_sensor(0x9203,0x90);
    HM8030MIPI_write_cmos_sensor(0x9204,0x85);
    HM8030MIPI_write_cmos_sensor(0x9205,0x2c);
    HM8030MIPI_write_cmos_sensor(0x9206,0x74);
    HM8030MIPI_write_cmos_sensor(0x9207,0x03);
    HM8030MIPI_write_cmos_sensor(0x9208,0xf0);
    HM8030MIPI_write_cmos_sensor(0x9209,0x90);
    HM8030MIPI_write_cmos_sensor(0x920a,0x85);
    HM8030MIPI_write_cmos_sensor(0x920b,0x32);
    HM8030MIPI_write_cmos_sensor(0x920c,0xf0);
    HM8030MIPI_write_cmos_sensor(0x920d,0x78);
    HM8030MIPI_write_cmos_sensor(0x920e,0x11);
    HM8030MIPI_write_cmos_sensor(0x920f,0x76);
    HM8030MIPI_write_cmos_sensor(0x9210,0x01);
    HM8030MIPI_write_cmos_sensor(0x9211,0x22);
    HM8030MIPI_write_cmos_sensor(0x9213,0x90);
    HM8030MIPI_write_cmos_sensor(0x9214,0x31);
    HM8030MIPI_write_cmos_sensor(0x9215,0x7D);
    HM8030MIPI_write_cmos_sensor(0x9216,0xE0);
    HM8030MIPI_write_cmos_sensor(0x9217,0xFF);
    HM8030MIPI_write_cmos_sensor(0x9218,0x90);
    HM8030MIPI_write_cmos_sensor(0x9219,0xA6);
    HM8030MIPI_write_cmos_sensor(0x921A,0x20);
    HM8030MIPI_write_cmos_sensor(0x921B,0xE0);
    HM8030MIPI_write_cmos_sensor(0x921C,0x6F);
    HM8030MIPI_write_cmos_sensor(0x921D,0x60);
    HM8030MIPI_write_cmos_sensor(0x921E,0x21);
    HM8030MIPI_write_cmos_sensor(0x921F,0x90);
    HM8030MIPI_write_cmos_sensor(0x9220,0x31);
    HM8030MIPI_write_cmos_sensor(0x9221,0x7D);
    HM8030MIPI_write_cmos_sensor(0x9222,0xE0);
    HM8030MIPI_write_cmos_sensor(0x9223,0x90);
    HM8030MIPI_write_cmos_sensor(0x9224,0xA6);
    HM8030MIPI_write_cmos_sensor(0x9225,0x20);
    HM8030MIPI_write_cmos_sensor(0x9226,0xF0);
    HM8030MIPI_write_cmos_sensor(0x9227,0xE4);
    HM8030MIPI_write_cmos_sensor(0x9228,0x90);
    HM8030MIPI_write_cmos_sensor(0x9229,0x42);
    HM8030MIPI_write_cmos_sensor(0x922A,0x04);
    HM8030MIPI_write_cmos_sensor(0x922B,0xF0);
    HM8030MIPI_write_cmos_sensor(0x922C,0xA3);
    HM8030MIPI_write_cmos_sensor(0x922D,0xF0);
    HM8030MIPI_write_cmos_sensor(0x922E,0x90);
    HM8030MIPI_write_cmos_sensor(0x922F,0x42);
    HM8030MIPI_write_cmos_sensor(0x9230,0x08);
    HM8030MIPI_write_cmos_sensor(0x9231,0xF0);
    HM8030MIPI_write_cmos_sensor(0x9232,0xA3);
    HM8030MIPI_write_cmos_sensor(0x9233,0xF0);
    HM8030MIPI_write_cmos_sensor(0x9234,0x90);
    HM8030MIPI_write_cmos_sensor(0x9235,0x42);
    HM8030MIPI_write_cmos_sensor(0x9236,0x0C);
    HM8030MIPI_write_cmos_sensor(0x9237,0xF0);
    HM8030MIPI_write_cmos_sensor(0x9238,0xA3);
    HM8030MIPI_write_cmos_sensor(0x9239,0xF0);
    HM8030MIPI_write_cmos_sensor(0x923A,0x90);
    HM8030MIPI_write_cmos_sensor(0x923B,0x42);
    HM8030MIPI_write_cmos_sensor(0x923C,0x10);
    HM8030MIPI_write_cmos_sensor(0x923D,0xF0);
    HM8030MIPI_write_cmos_sensor(0x923E,0xA3);
    HM8030MIPI_write_cmos_sensor(0x923F,0xF0);
    HM8030MIPI_write_cmos_sensor(0x9240,0x22);
    HM8030MIPI_write_cmos_sensor(0x9241,0x90);
    HM8030MIPI_write_cmos_sensor(0x9242,0x31);
    HM8030MIPI_write_cmos_sensor(0x9243,0x45);
    HM8030MIPI_write_cmos_sensor(0x9244,0xE0);
    HM8030MIPI_write_cmos_sensor(0x9245,0xFF);
    HM8030MIPI_write_cmos_sensor(0x9246,0x90);
    HM8030MIPI_write_cmos_sensor(0x9247,0x31);
    HM8030MIPI_write_cmos_sensor(0x9248,0x7D);
    HM8030MIPI_write_cmos_sensor(0x9249,0xE0);
    HM8030MIPI_write_cmos_sensor(0x924A,0x6F);
    HM8030MIPI_write_cmos_sensor(0x924B,0x60);
    HM8030MIPI_write_cmos_sensor(0x924C,0x07);
    HM8030MIPI_write_cmos_sensor(0x924D,0x90);
    HM8030MIPI_write_cmos_sensor(0x924E,0x42);
    HM8030MIPI_write_cmos_sensor(0x924F,0x00);
    HM8030MIPI_write_cmos_sensor(0x9250,0xE0);
    HM8030MIPI_write_cmos_sensor(0x9251,0x44);
    HM8030MIPI_write_cmos_sensor(0x9252,0x08);
    HM8030MIPI_write_cmos_sensor(0x9253,0xF0);
    HM8030MIPI_write_cmos_sensor(0x9254,0x22);
    HM8030MIPI_write_cmos_sensor(0x8504,0x39);
    HM8030MIPI_write_cmos_sensor(0x8505,0x17);
    HM8030MIPI_write_cmos_sensor(0x8506,0x00);
    HM8030MIPI_write_cmos_sensor(0x8507,0xF0);
    HM8030MIPI_write_cmos_sensor(0x8508,0x03);
    HM8030MIPI_write_cmos_sensor(0x850a,0x13);
    HM8030MIPI_write_cmos_sensor(0x850b,0x19);
    HM8030MIPI_write_cmos_sensor(0x850c,0x00);
    HM8030MIPI_write_cmos_sensor(0x850d,0x00);
    HM8030MIPI_write_cmos_sensor(0x850e,0x02);
    HM8030MIPI_write_cmos_sensor(0x8510,0x43);
    HM8030MIPI_write_cmos_sensor(0x8511,0x8B);
    HM8030MIPI_write_cmos_sensor(0x8512,0x00);
    HM8030MIPI_write_cmos_sensor(0x8513,0x08);
    HM8030MIPI_write_cmos_sensor(0x8514,0x02);
    HM8030MIPI_write_cmos_sensor(0x8516,0x12);
    HM8030MIPI_write_cmos_sensor(0x8517,0x36);
    HM8030MIPI_write_cmos_sensor(0x8518,0x00);
    HM8030MIPI_write_cmos_sensor(0x8519,0x11);
    HM8030MIPI_write_cmos_sensor(0x851A,0x02);
    HM8030MIPI_write_cmos_sensor(0x851C,0x10);
    HM8030MIPI_write_cmos_sensor(0x851D,0x01);
    HM8030MIPI_write_cmos_sensor(0x851E,0x00);
    HM8030MIPI_write_cmos_sensor(0x851F,0x1D);
    HM8030MIPI_write_cmos_sensor(0x8520,0x02);
    HM8030MIPI_write_cmos_sensor(0x8522,0x12);
    HM8030MIPI_write_cmos_sensor(0x8523,0x40);
    HM8030MIPI_write_cmos_sensor(0x8524,0x01);
    HM8030MIPI_write_cmos_sensor(0x8525,0x25);
    HM8030MIPI_write_cmos_sensor(0x8526,0x02);
    HM8030MIPI_write_cmos_sensor(0x8528,0x14);
    HM8030MIPI_write_cmos_sensor(0x8529,0x60);
    HM8030MIPI_write_cmos_sensor(0x852A,0x00);
    HM8030MIPI_write_cmos_sensor(0x852B,0xE4);
    HM8030MIPI_write_cmos_sensor(0x852C,0x00);
    HM8030MIPI_write_cmos_sensor(0x852E,0x15);
    HM8030MIPI_write_cmos_sensor(0x852F,0xA0);
    HM8030MIPI_write_cmos_sensor(0x8530,0x00);
    HM8030MIPI_write_cmos_sensor(0x8531,0xE2);
    HM8030MIPI_write_cmos_sensor(0x8532,0x00);
    HM8030MIPI_write_cmos_sensor(0x8534,0x0B);
    HM8030MIPI_write_cmos_sensor(0x8535,0x55);
    HM8030MIPI_write_cmos_sensor(0x8536,0x01);
    HM8030MIPI_write_cmos_sensor(0x8537,0x28);
    HM8030MIPI_write_cmos_sensor(0x8538,0x02);
    HM8030MIPI_write_cmos_sensor(0x853A,0x12);
    HM8030MIPI_write_cmos_sensor(0x853B,0x80);
    HM8030MIPI_write_cmos_sensor(0x853C,0x01);
    HM8030MIPI_write_cmos_sensor(0x853D,0x3E);
    HM8030MIPI_write_cmos_sensor(0x853E,0x02);
    HM8030MIPI_write_cmos_sensor(0x8540,0x3E);
    HM8030MIPI_write_cmos_sensor(0x8541,0x29);
    HM8030MIPI_write_cmos_sensor(0x8542,0x01);
    HM8030MIPI_write_cmos_sensor(0x8543,0xD6);
    HM8030MIPI_write_cmos_sensor(0x8544,0x02);
    HM8030MIPI_write_cmos_sensor(0x8546,0x43);
    HM8030MIPI_write_cmos_sensor(0x8547,0x87);
    HM8030MIPI_write_cmos_sensor(0x8548,0x01);
    HM8030MIPI_write_cmos_sensor(0x8549,0xF7);
    HM8030MIPI_write_cmos_sensor(0x854A,0x02);
    HM8030MIPI_write_cmos_sensor(0x854c,0x45);
    HM8030MIPI_write_cmos_sensor(0x854d,0x08);
    HM8030MIPI_write_cmos_sensor(0x854e,0x02);
    HM8030MIPI_write_cmos_sensor(0x854f,0x13);
    HM8030MIPI_write_cmos_sensor(0x8550,0x02);
    HM8030MIPI_write_cmos_sensor(0x8552,0x4A);
    HM8030MIPI_write_cmos_sensor(0x8553,0x87);
    HM8030MIPI_write_cmos_sensor(0x8554,0x02);
    HM8030MIPI_write_cmos_sensor(0x8555,0x41);
    HM8030MIPI_write_cmos_sensor(0x8556,0x02);
HM8030MIPI_write_cmos_sensor(0x9037, 0x4D);
HM8030MIPI_write_cmos_sensor(0x903B, 0xAF);
HM8030MIPI_write_cmos_sensor(0x903F, 0xC5);
HM8030MIPI_write_cmos_sensor(0x9043, 0xDF);
HM8030MIPI_write_cmos_sensor(0x9052, 0x4D);
HM8030MIPI_write_cmos_sensor(0x9056, 0xC9);
HM8030MIPI_write_cmos_sensor(0x905A, 0xC4);
HM8030MIPI_write_cmos_sensor(0x905E, 0xEF);
HM8030MIPI_write_cmos_sensor(0x906D, 0x4D);
HM8030MIPI_write_cmos_sensor(0x9071, 0xE7);
HM8030MIPI_write_cmos_sensor(0x9075, 0xC4);
HM8030MIPI_write_cmos_sensor(0x9079, 0xA9);
HM8030MIPI_write_cmos_sensor(0x9086, 0x4E);
HM8030MIPI_write_cmos_sensor(0x908C, 0x15);
HM8030MIPI_write_cmos_sensor(0x9090, 0xC4);
HM8030MIPI_write_cmos_sensor(0x9094, 0x9D);
HM8030MIPI_write_cmos_sensor(0x909C, 0x3E);
HM8030MIPI_write_cmos_sensor(0x90A0, 0xC4);
HM8030MIPI_write_cmos_sensor(0x90A4, 0x31);
HM8030MIPI_write_cmos_sensor(0x90B4, 0x4B);
HM8030MIPI_write_cmos_sensor(0x90B8, 0x9E);
HM8030MIPI_write_cmos_sensor(0x90BC, 0xC4);
HM8030MIPI_write_cmos_sensor(0x90C0, 0xA9);
HM8030MIPI_write_cmos_sensor(0x90CE, 0x4B);
HM8030MIPI_write_cmos_sensor(0x90D2, 0xBE);
HM8030MIPI_write_cmos_sensor(0x90D6, 0xC4);
HM8030MIPI_write_cmos_sensor(0x90DA, 0x9C);
HM8030MIPI_write_cmos_sensor(0x90E8, 0x4B);
HM8030MIPI_write_cmos_sensor(0x90EC, 0xEE);
HM8030MIPI_write_cmos_sensor(0x90F0, 0xC4);
HM8030MIPI_write_cmos_sensor(0x90F4, 0x76);
HM8030MIPI_write_cmos_sensor(0x9100, 0x4C);
HM8030MIPI_write_cmos_sensor(0x9106, 0x1B);
HM8030MIPI_write_cmos_sensor(0x910A, 0xC4);
HM8030MIPI_write_cmos_sensor(0x910E, 0x90);
HM8030MIPI_write_cmos_sensor(0x9115, 0x69);
HM8030MIPI_write_cmos_sensor(0x9119, 0xC4);
HM8030MIPI_write_cmos_sensor(0x911D, 0x56);
    HM8030MIPI_write_cmos_sensor(0x8500,0x01);
    HM8030MIPI_write_cmos_sensor(0xffff,0x00);

mDELAY(200);

HM8030MIPI_write_cmos_sensor(0x0101,0x00);
// 20140129 BRAD modified for 03


HM8030MIPI_write_cmos_sensor(0x0112,0x0A);
HM8030MIPI_write_cmos_sensor(0x0113,0x0A);
HM8030MIPI_write_cmos_sensor(0x0111,0x02);
HM8030MIPI_write_cmos_sensor(0x0304,0x00);
HM8030MIPI_write_cmos_sensor(0x0305,0x04);//02
HM8030MIPI_write_cmos_sensor(0x0306,0x00);
//HM8030MIPI_write_cmos_sensor(0x0307,0x64);//44
HM8030MIPI_write_cmos_sensor(0x0307,0x6E);
HM8030MIPI_write_cmos_sensor(0x0302,0x00);
HM8030MIPI_write_cmos_sensor(0x0303,0x05);
HM8030MIPI_write_cmos_sensor(0x0300,0x00);
HM8030MIPI_write_cmos_sensor(0x0301,0x01);
HM8030MIPI_write_cmos_sensor(0xA649,0x00);
HM8030MIPI_write_cmos_sensor(0x0200,0x02);
HM8030MIPI_write_cmos_sensor(0x0201,0xB0);
HM8030MIPI_write_cmos_sensor(0x0202,0x01);
HM8030MIPI_write_cmos_sensor(0x0203,0x90);
HM8030MIPI_write_cmos_sensor(0x0204,0x00);
HM8030MIPI_write_cmos_sensor(0x0205,0x60);
HM8030MIPI_write_cmos_sensor(0x3020,0x0F);
HM8030MIPI_write_cmos_sensor(0x801A,0x05);
HM8030MIPI_write_cmos_sensor(0x8023,0x11);
HM8030MIPI_write_cmos_sensor(0x8027,0x11);
HM8030MIPI_write_cmos_sensor(0x8011,0x06);
HM8030MIPI_write_cmos_sensor(0x034C,0x0C);
HM8030MIPI_write_cmos_sensor(0x034D,0xE0);
HM8030MIPI_write_cmos_sensor(0x034E,0x09);
HM8030MIPI_write_cmos_sensor(0x034F,0xB0);
HM8030MIPI_write_cmos_sensor(0x5300,0x01);
HM8030MIPI_write_cmos_sensor(0x5318,0x0C);
HM8030MIPI_write_cmos_sensor(0x5319,0xE0);
HM8030MIPI_write_cmos_sensor(0x531C,0x09);
HM8030MIPI_write_cmos_sensor(0x531D,0xB0);
HM8030MIPI_write_cmos_sensor(0x5320,0x0C);
HM8030MIPI_write_cmos_sensor(0x5321,0xE0);
HM8030MIPI_write_cmos_sensor(0x5324,0x09);
HM8030MIPI_write_cmos_sensor(0x5325,0xB0);
HM8030MIPI_write_cmos_sensor(0x0344,0x00);
HM8030MIPI_write_cmos_sensor(0x0345,0x00);
HM8030MIPI_write_cmos_sensor(0x0348,0x0C);
HM8030MIPI_write_cmos_sensor(0x0349,0xDF);
HM8030MIPI_write_cmos_sensor(0x0346,0x00);
HM8030MIPI_write_cmos_sensor(0x0347,0x00);
HM8030MIPI_write_cmos_sensor(0x034A,0x09);
HM8030MIPI_write_cmos_sensor(0x034B,0xAF);
HM8030MIPI_write_cmos_sensor(0x0380,0x00);
HM8030MIPI_write_cmos_sensor(0x0381,0x01);
HM8030MIPI_write_cmos_sensor(0x0382,0x00);
HM8030MIPI_write_cmos_sensor(0x0383,0x01);
HM8030MIPI_write_cmos_sensor(0x0384,0x00);
HM8030MIPI_write_cmos_sensor(0x0385,0x01);
HM8030MIPI_write_cmos_sensor(0x0386,0x00);
HM8030MIPI_write_cmos_sensor(0x0387,0x01);
HM8030MIPI_write_cmos_sensor(0xA608,0x04);
HM8030MIPI_write_cmos_sensor(0xA607,0x01);
HM8030MIPI_write_cmos_sensor(0xA651,0x02); //20140117
HM8030MIPI_write_cmos_sensor(0x0100,0x01);

	SENSORDB("HM8030MIPI_globle_setting end \n");
	// The register only need to enable 1 time.    
	spin_lock(&HM8030_drv_lock);  
	HM8030MIPI_Auto_Flicker_mode = KAL_FALSE;	  // reset the flicker status	 
	spin_unlock(&HM8030_drv_lock);
	SENSORDB("[HM8030MIPI]exit HM8030MIPI_Sensor_Init function\n");
}   /*  HM8030MIPI_Sensor_Init  */
void HM8030MIPI_VideoSizeSetting(void)//16:9   6M
{
	SENSORDB("[HM8030MIPI]enter HM8030MIPI_VideoSizeSetting function\n");
HM8030MIPI_write_cmos_sensor(0x0100,0x00);//
mdelay(50);	
HM8030MIPI_write_cmos_sensor(0xA649,0x01);//04
HM8030MIPI_write_cmos_sensor(0x0382,0x00);//
HM8030MIPI_write_cmos_sensor(0x0383,0x01);//x_odd_inc
HM8030MIPI_write_cmos_sensor(0x0386,0x00);//
HM8030MIPI_write_cmos_sensor(0x0387,0x03);//y_odd_inc
    HM8030MIPI_write_cmos_sensor(0x0340,0x05);
    HM8030MIPI_write_cmos_sensor(0x0341,0x00);
HM8030MIPI_write_cmos_sensor(0x0344,0x00);//x start size
HM8030MIPI_write_cmos_sensor(0x0345,0x00);//
HM8030MIPI_write_cmos_sensor(0x0346,0x00);//y start size 
HM8030MIPI_write_cmos_sensor(0x0347,0x00);//
HM8030MIPI_write_cmos_sensor(0x0348,0x0C);//
HM8030MIPI_write_cmos_sensor(0x0349,0xDF);//x end size
HM8030MIPI_write_cmos_sensor(0x034a,0x09);//
HM8030MIPI_write_cmos_sensor(0x034B,0xAF);//y end size
HM8030MIPI_write_cmos_sensor(0x034C,0x06);//
HM8030MIPI_write_cmos_sensor(0x034D,0x70);//x output size
HM8030MIPI_write_cmos_sensor(0x034E,0x04);//
HM8030MIPI_write_cmos_sensor(0x034F,0xD8);//y output size
HM8030MIPI_write_cmos_sensor(0x0100,0x01);//
mdelay(50);	
	SENSORDB("[HM8030MIPI]exit HM8030MIPI_VideoSizeSetting function\n");
}
void HM8030MIPI_PreviewSetting(void)
{
	SENSORDB("[HM8030MIPI]enter HM8030MIPI_PreviewSetting function\n");
HM8030MIPI_write_cmos_sensor(0x0100,0x00);//
mdelay(50);	
HM8030MIPI_write_cmos_sensor(0xA649,0x01);//04
HM8030MIPI_write_cmos_sensor(0x0382,0x00);//
HM8030MIPI_write_cmos_sensor(0x0383,0x01);//x_odd_inc
HM8030MIPI_write_cmos_sensor(0x0386,0x00);//
HM8030MIPI_write_cmos_sensor(0x0387,0x03);//y_odd_inc
    HM8030MIPI_write_cmos_sensor(0x0340,0x05);
    HM8030MIPI_write_cmos_sensor(0x0341,0x00);
HM8030MIPI_write_cmos_sensor(0x0344,0x00);//x start size
HM8030MIPI_write_cmos_sensor(0x0345,0x00);//
HM8030MIPI_write_cmos_sensor(0x0346,0x00);//y start size 
HM8030MIPI_write_cmos_sensor(0x0347,0x00);//
HM8030MIPI_write_cmos_sensor(0x0348,0x0C);//
HM8030MIPI_write_cmos_sensor(0x0349,0xDF);//x end size
HM8030MIPI_write_cmos_sensor(0x034a,0x09);//
HM8030MIPI_write_cmos_sensor(0x034B,0xAF);//y end size
HM8030MIPI_write_cmos_sensor(0x034C,0x06);//
HM8030MIPI_write_cmos_sensor(0x034D,0x70);//x output size
HM8030MIPI_write_cmos_sensor(0x034E,0x04);//
HM8030MIPI_write_cmos_sensor(0x034F,0xD8);//y output size
HM8030MIPI_write_cmos_sensor(0x0100,0x01);//
mdelay(50);
	// The register only need to enable 1 time.    
	spin_lock(&HM8030_drv_lock);  
	HM8030MIPI_Auto_Flicker_mode = KAL_FALSE;	  // reset the flicker status	 
	spin_unlock(&HM8030_drv_lock);
	SENSORDB("[HM8030MIPI]exit HM8030MIPI_PreviewSetting function\n");
}

void HM8030MIPI_set_8M(void)
{	//77 capture setting
	SENSORDB("[HM8030MIPI]enter HM8030MIPI_set_8M function\n");
HM8030MIPI_write_cmos_sensor(0x0100,0x00);//
mdelay(50);
HM8030MIPI_write_cmos_sensor(0xA649,0x00);//
HM8030MIPI_write_cmos_sensor(0x0382,0x00);//
HM8030MIPI_write_cmos_sensor(0x0383,0x01);//x_odd_inc
HM8030MIPI_write_cmos_sensor(0x0386,0x00);//
HM8030MIPI_write_cmos_sensor(0x0387,0x01);//y_odd_inc
HM8030MIPI_write_cmos_sensor(0x0340,0x09);
HM8030MIPI_write_cmos_sensor(0x0341,0xEC);
HM8030MIPI_write_cmos_sensor(0x0344,0x00);//x start size
HM8030MIPI_write_cmos_sensor(0x0345,0x00);//
HM8030MIPI_write_cmos_sensor(0x0346,0x00);//y start size 
HM8030MIPI_write_cmos_sensor(0x0347,0x00);//
HM8030MIPI_write_cmos_sensor(0x0348,0x0C);//
HM8030MIPI_write_cmos_sensor(0x0349,0xDF);//x end size
HM8030MIPI_write_cmos_sensor(0x034a,0x09);//
HM8030MIPI_write_cmos_sensor(0x034B,0xAF);//y end size
HM8030MIPI_write_cmos_sensor(0x034C,0x0C);//
HM8030MIPI_write_cmos_sensor(0x034D,0xC0);//x output size//E0
HM8030MIPI_write_cmos_sensor(0x034E,0x09);//
HM8030MIPI_write_cmos_sensor(0x034F,0x90);//y output size//B0
HM8030MIPI_write_cmos_sensor(0x0100,0x01);//
mdelay(50);

	SENSORDB("[HM8030MIPI]exit HM8030MIPI_set_8M function\n"); 
}
/*****************************************************************************/
/* Windows Mobile Sensor Interface */
/*****************************************************************************/
/*************************************************************************
* FUNCTION
*   HM8030MIPIOpen
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

UINT32 HM8030MIPIOpen(void)
{
    int  retry = 0; 
	kal_uint16 sensorid;
    // check if sensor ID correct
    retry = 3; 
	SENSORDB("[HM8030MIPI]enter HM8030MIPIOpen function\n");
    do {
	    sensorid = ((HM8030MIPI_read_cmos_sensor(0x0000) << 8) | HM8030MIPI_read_cmos_sensor(0x0001));
	   spin_lock(&HM8030_drv_lock);    
	   HM8030MIPI_sensor_id =sensorid;
	   spin_unlock(&HM8030_drv_lock);
		if (HM8030MIPI_sensor_id == HM8030MIPI_SENSOR_ID)
			break; 
		retry--; 
	    }
	while (retry > 0);
    SENSORDB("Read Sensor ID = 0x%04x\n", HM8030MIPI_sensor_id);
    if (HM8030MIPI_sensor_id != HM8030MIPI_SENSOR_ID)
        return ERROR_SENSOR_CONNECT_FAIL;
    HM8030MIPI_Sensor_Init();
	sensorid=read_HM8030MIPI_gain();
	spin_lock(&HM8030_drv_lock);	
    HM8030MIPI_sensor_gain_base = sensorid;
	HM8030MIPIAutoFlicKerMode   = KAL_FALSE;
	spin_unlock(&HM8030_drv_lock);
	SENSORDB("[HM8030MIPI]exit HM8030MIPIOpen function\n");
    return ERROR_NONE;
}

/*************************************************************************
* FUNCTION
*   HM8030MIPIGetSensorID
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
UINT32 HM8030MIPIGetSensorID(UINT32 *sensorID) 
{
    int  retry = 3; 
	SENSORDB("[HM8030MIPI]enter HM8030MIPIGetSensorID function\n");
    // check if sensor ID correct
    do {		
	   *sensorID =((HM8030MIPI_read_cmos_sensor(0x0000) << 8) | HM8030MIPI_read_cmos_sensor(0x0001));
        if (*sensorID == HM8030MIPI_SENSOR_ID)
            break;
        retry--; 
    } while (retry > 0);

    if (*sensorID != HM8030MIPI_SENSOR_ID) {
        *sensorID = 0xFFFFFFFF; 
        return ERROR_SENSOR_CONNECT_FAIL;
    }
	SENSORDB("[HM8030MIPI]exit HM8030MIPIGetSensorID function\n");
    return ERROR_NONE;
}
/*Avoid Folat, frame rate =10 * u2FrameRate */
UINT32 HM8030MIPISetMaxFrameRate(UINT16 u2FrameRate)
{
	kal_int16 dummy_line=0;
	kal_uint16 FrameHeight = HM8030MIPI_sensor.frame_height;
	  unsigned long flags;	
	SENSORDB("[soso][OV5647MIPISetMaxFrameRate]u2FrameRate=%d \n",u2FrameRate);
	FrameHeight= (10 * HM8030MIPI_sensor.pv_pclk) / u2FrameRate / HM8030MIPI_sensor.line_length;
	if(KAL_FALSE == HM8030MIPI_sensor.pv_mode)
	{
		if(KAL_FALSE == HM8030MIPI_sensor.video_mode)
		{
			if(FrameHeight < HM8030MIPI_CAP_FULL_LINES)
				FrameHeight = HM8030MIPI_CAP_FULL_LINES;
		}
		else
		{
			if(FrameHeight < HM8030MIPI_VIDEO_FULL_LINES)
				FrameHeight = HM8030MIPI_VIDEO_FULL_LINES;
		}
	}
	spin_lock_irqsave(&HM8030_drv_lock,flags);
	HM8030MIPI_sensor.frame_height = FrameHeight;
	spin_unlock_irqrestore(&HM8030_drv_lock,flags);
	if((HM8030_CurrentScenarioId == MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG)||(HM8030_CurrentScenarioId == MSDK_SCENARIO_ID_CAMERA_ZSD))
	{
		dummy_line = FrameHeight - HM8030MIPI_CAP_FULL_LINES;
	}
	else if(HM8030_CurrentScenarioId==MSDK_SCENARIO_ID_CAMERA_PREVIEW)
	{
		dummy_line = FrameHeight - HM8030MIPI_PV_FULL_LINES;
	}
	else if(HM8030_CurrentScenarioId==MSDK_SCENARIO_ID_VIDEO_PREVIEW) 
	{
		dummy_line = FrameHeight - HM8030MIPI_VIDEO_FULL_LINES;
	}
	SENSORDB("[soso][OV5647MIPISetMaxFrameRate]frameheight = %d, dummy_line=%d \n",HM8030MIPI_sensor.frame_height,dummy_line);
	if(dummy_line<0) 
	{
		dummy_line = 0;
	}
	/* to fix VSYNC, to fix frame rate */
	HM8030MIPI_SetDummy(0, dummy_line); /* modify dummy_pixel must gen AE table again */
	return ERROR_NONE;
}


/*************************************************************************
* FUNCTION
*   HM8030MIPI_SetShutter
*
* DESCRIPTION
*   This function set e-shutter of HM8030MIPI to change exposure time.
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
void HM8030MIPI_SetShutter(kal_uint16 iShutter)
{
	SENSORDB("[HM8030MIPI]enter HM8030MIPI_SetShutter function\n");
	SENSORDB("[HM8030MIPI]%s():shutter=%d\n",__FUNCTION__,iShutter);
    if (iShutter < 1)
        iShutter = 1; 
	else if(iShutter > 0xffff)
		iShutter = 0xffff;
	unsigned long flags;
	spin_lock_irqsave(&HM8030_drv_lock,flags);
    HM8030MIPI_sensor.pv_shutter = iShutter;	
	spin_unlock_irqrestore(&HM8030_drv_lock,flags);
    HM8030MIPI_write_shutter(iShutter);
	SENSORDB("[HM8030MIPI]exit HM8030MIPI_SetShutter function\n");
}   /*  HM8030MIPI_SetShutter   */



/*************************************************************************
* FUNCTION
*   HM8030MIPI_read_shutter
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
UINT16 HM8030MIPI_read_shutter(void)
{
    return (UINT16)( (HM8030MIPI_read_cmos_sensor(0x0202)<<8) | HM8030MIPI_read_cmos_sensor(0x0203) );
}

/*************************************************************************
* FUNCTION
*   HM8030MIPI_night_mode
*
* DESCRIPTION
*   This function night mode of HM8030MIPI.
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
void HM8030MIPI_NightMode(kal_bool bEnable)
{
	SENSORDB("[HM8030MIPI]enter HM8030MIPI_NightMode function\n");
#if 0
    /************************************************************************/
    /*                      Auto Mode: 30fps                                                                                          */
    /*                      Night Mode:15fps                                                                                          */
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
	SENSORDB("[HM8030MIPI]exit HM8030MIPI_NightMode function\n");
}/*	HM8030MIPI_NightMode */



/*************************************************************************
* FUNCTION
*   HM8030MIPIClose
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
UINT32 HM8030MIPIClose(void)
{
    return ERROR_NONE;
}	/* HM8030MIPIClose() */

void HM8030MIPISetFlipMirror(kal_int32 imgMirror)
{
    kal_uint8  iTemp; 
	SENSORDB("[HM8030MIPI]enter HM8030MIPISetFlipMirror function\n");
    iTemp = HM8030MIPI_read_cmos_sensor(0x0101) & 0x03;	//Clear the mirror and flip bits.
    switch (imgMirror)
    {
        case IMAGE_NORMAL:
            HM8030MIPI_write_cmos_sensor(0x0101, 0x03);	//Set normal
            break;
        case IMAGE_V_MIRROR:
            HM8030MIPI_write_cmos_sensor(0x0101, iTemp | 0x01);	//Set flip
            break;
        case IMAGE_H_MIRROR:
            HM8030MIPI_write_cmos_sensor(0x0101, iTemp | 0x02);	//Set mirror
            break;
        case IMAGE_HV_MIRROR:
            HM8030MIPI_write_cmos_sensor(0x0101, 0x00);	//Set mirror and flip
            break;
    }
	SENSORDB("[HM8030MIPI]exit HM8030MIPISetFlipMirror function\n");
}


/*************************************************************************
* FUNCTION
*   HM8030MIPIPreview
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
UINT32 HM8030MIPIPreview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                                                MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{	
	SENSORDB("[HM8030MIPI]enter HM8030MIPIPreview function\n");
	spin_lock(&HM8030_drv_lock);    
	HM8030MIPI_MPEG4_encode_mode = KAL_FALSE;
	HM8030MIPI_sensor.video_mode=KAL_FALSE;
	HM8030MIPI_sensor.pv_mode=KAL_TRUE;
	HM8030MIPI_sensor.capture_mode=KAL_FALSE;
	spin_unlock(&HM8030_drv_lock);
	HM8030MIPI_PreviewSetting();
#if defined(CREATOR_K3C)
	HM8030MIPISetFlipMirror(IMAGE_NORMAL); 
#endif	
	//HM8030MIPI_write_cmos_sensor(0x0600, 0x00);  // color bar test pattern
  //HM8030MIPI_write_cmos_sensor(0x0601, 0x02);  // color bar test pattern
  
	spin_lock(&HM8030_drv_lock);	
	HM8030MIPI_sensor.cp_dummy_pixels = 0;
	HM8030MIPI_sensor.cp_dummy_lines = 0;

	HM8030MIPI_sensor.pv_dummy_pixels = 0;
	HM8030MIPI_sensor.pv_dummy_lines = 0;
	HM8030MIPI_sensor.video_dummy_pixels = 0;
	HM8030MIPI_sensor.video_dummy_lines = 0;
	HM8030MIPI_sensor.pv_line_length  = HM8030MIPI_PV_FULL_PIXELS+HM8030MIPI_sensor.pv_dummy_pixels; 
	HM8030MIPI_sensor.pv_frame_length = HM8030MIPI_PV_FULL_LINES+HM8030MIPI_sensor.pv_dummy_lines;
	HM8030MIPI_sensor.frame_height    = HM8030MIPI_sensor.pv_frame_length;
	HM8030MIPI_sensor.line_length     = HM8030MIPI_sensor.pv_line_length; 
	spin_unlock(&HM8030_drv_lock);

	HM8030MIPI_SetDummy(HM8030MIPI_sensor.pv_dummy_pixels,HM8030MIPI_sensor.pv_dummy_lines);
	HM8030MIPI_SetShutter(HM8030MIPI_sensor.pv_shutter);
	spin_lock(&HM8030_drv_lock);	
	memcpy(&HM8030MIPISensorConfigData, sensor_config_data, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
	spin_unlock(&HM8030_drv_lock);
	SENSORDB("[HM8030MIPI]eXIT HM8030MIPIPreview function\n"); 
	return ERROR_NONE;
	}	/* HM8030MIPIPreview() */

/*************************************************************************
* FUNCTION
* RETURNS
*   None
*
* GLOBALS AFFECTED
*
*************************************************************************/
UINT32 HM8030MIPIVideo(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                                                MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	kal_uint16 iStartX = 0, iStartY = 0;
	SENSORDB("[HM8030MIPI]enter HM8030MIPIVideo function\n"); 
	spin_lock(&HM8030_drv_lock);    
    HM8030MIPI_MPEG4_encode_mode = KAL_TRUE;  
	HM8030MIPI_sensor.video_mode=KAL_TRUE;
	HM8030MIPI_sensor.pv_mode=KAL_FALSE;
	HM8030MIPI_sensor.capture_mode=KAL_FALSE;
	spin_unlock(&HM8030_drv_lock);
	HM8030MIPI_VideoSizeSetting();
	spin_lock(&HM8030_drv_lock);
#if defined(CREATOR_K3C)
	HM8030MIPISetFlipMirror(IMAGE_NORMAL); 
#endif	
	HM8030MIPI_sensor.cp_dummy_pixels = 0;
	HM8030MIPI_sensor.cp_dummy_lines = 0;
	HM8030MIPI_sensor.pv_dummy_pixels = 0;
	HM8030MIPI_sensor.pv_dummy_lines = 0;
	HM8030MIPI_sensor.video_dummy_pixels = 0;
	HM8030MIPI_sensor.video_dummy_lines = 0;
	HM8030MIPI_sensor.video_line_length  = HM8030MIPI_VIDEO_FULL_PIXELS+HM8030MIPI_sensor.video_dummy_pixels; 
	HM8030MIPI_sensor.video_frame_length = HM8030MIPI_VIDEO_FULL_LINES+HM8030MIPI_sensor.video_dummy_lines;
	HM8030MIPI_sensor.frame_height       = HM8030MIPI_sensor.video_frame_length; 
	HM8030MIPI_sensor.line_length        = HM8030MIPI_sensor.video_line_length;
	spin_unlock(&HM8030_drv_lock);
	
	HM8030MIPI_SetDummy(HM8030MIPI_sensor.video_dummy_pixels,HM8030MIPI_sensor.video_dummy_lines);
	HM8030MIPI_SetShutter(HM8030MIPI_sensor.video_shutter);
	spin_lock(&HM8030_drv_lock);	
	memcpy(&HM8030MIPISensorConfigData, sensor_config_data, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
	spin_unlock(&HM8030_drv_lock);   
    SENSORDB("[HM8030MIPI]eXIT HM8030MIPIVideo function\n"); 
	return ERROR_NONE;
}	/* HM8030MIPIPreview() */

UINT32 HM8030MIPICapture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                                                MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	SENSORDB("[HM8030MIPI]enter HM8030MIPICapture function\n");
	spin_lock(&HM8030_drv_lock);	
	HM8030MIPI_sensor.video_mode=KAL_FALSE;
	HM8030MIPI_sensor.pv_mode=KAL_FALSE;
	HM8030MIPI_sensor.capture_mode=KAL_TRUE;
	HM8030MIPIAutoFlicKerMode = KAL_FALSE;
	HM8030MIPI_MPEG4_encode_mode = KAL_FALSE; 
	HM8030MIPI_Auto_Flicker_mode = KAL_FALSE;       
	HM8030MIPI_sensor.cp_dummy_pixels = 0;
	HM8030MIPI_sensor.cp_dummy_lines = 0;
	spin_unlock(&HM8030_drv_lock);
	HM8030MIPI_set_8M();
#if defined(CREATOR_K3C)
	HM8030MIPISetFlipMirror(IMAGE_NORMAL); 
#endif
	spin_lock(&HM8030_drv_lock);    
	HM8030MIPI_sensor.cp_line_length  = HM8030MIPI_CAP_FULL_PIXELS+HM8030MIPI_sensor.cp_dummy_pixels;
	HM8030MIPI_sensor.cp_frame_length = HM8030MIPI_CAP_FULL_LINES+HM8030MIPI_sensor.cp_dummy_lines;
	HM8030MIPI_sensor.frame_height    = HM8030MIPI_sensor.cp_frame_length;
	HM8030MIPI_sensor.line_length     = HM8030MIPI_sensor.cp_line_length;
	spin_unlock(&HM8030_drv_lock);
	HM8030MIPI_SetDummy(HM8030MIPI_sensor.cp_dummy_pixels, HM8030MIPI_sensor.cp_dummy_lines);   
	spin_lock(&HM8030_drv_lock);	
	memcpy(&HM8030MIPISensorConfigData, sensor_config_data, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
	spin_unlock(&HM8030_drv_lock);
	SENSORDB("[HM8030MIPI]exit HM8030MIPICapture function\n");
	return ERROR_NONE;
}	/* HM8030MIPICapture() */

UINT32 HM8030MIPIGetResolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *pSensorResolution)
{
    SENSORDB("[HM8030MIPI]eXIT HM8030MIPIGetResolution function\n");
    pSensorResolution->SensorPreviewWidth	= HM8030MIPI_REAL_PV_WIDTH;
    pSensorResolution->SensorPreviewHeight	= HM8030MIPI_REAL_PV_HEIGHT;
    pSensorResolution->SensorFullWidth		= HM8030MIPI_REAL_CAP_WIDTH;
    pSensorResolution->SensorFullHeight		= HM8030MIPI_REAL_CAP_HEIGHT;
    pSensorResolution->SensorVideoWidth		= HM8030MIPI_REAL_VIDEO_WIDTH;
    pSensorResolution->SensorVideoHeight    = HM8030MIPI_REAL_VIDEO_HEIGHT;
    SENSORDB("[HM8030MIPI]Exit HM8030MIPIGetResolution function!\n");   
    return ERROR_NONE;
}   /* HM8030MIPIGetResolution() */
UINT32 HM8030MIPIGetInfo(MSDK_SCENARIO_ID_ENUM ScenarioId,
                                                MSDK_SENSOR_INFO_STRUCT *pSensorInfo,
                                                MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{ 
	SENSORDB("[HM8030MIPI]enter HM8030MIPIGetInfo function\n");
	switch(ScenarioId){
			case MSDK_SCENARIO_ID_CAMERA_ZSD:
			case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG://hhl 2-28
				pSensorInfo->SensorFullResolutionX=HM8030MIPI_REAL_CAP_WIDTH;
				pSensorInfo->SensorFullResolutionY=HM8030MIPI_REAL_CAP_HEIGHT;
				pSensorInfo->SensorStillCaptureFrameRate=30;//22

			break;//hhl 2-28
			case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
				pSensorInfo->SensorPreviewResolutionX=HM8030MIPI_REAL_VIDEO_WIDTH;
				pSensorInfo->SensorPreviewResolutionY=HM8030MIPI_REAL_VIDEO_HEIGHT;
				pSensorInfo->SensorCameraPreviewFrameRate=30;
			break;
		default:
        pSensorInfo->SensorPreviewResolutionX=HM8030MIPI_REAL_PV_WIDTH;
        pSensorInfo->SensorPreviewResolutionY=HM8030MIPI_REAL_PV_HEIGHT;
				pSensorInfo->SensorCameraPreviewFrameRate=30;
			break;
	}
    pSensorInfo->SensorVideoFrameRate=30;	
    pSensorInfo->SensorStillCaptureFrameRate=30;//15
    pSensorInfo->SensorWebCamCaptureFrameRate=30;
    pSensorInfo->SensorResetActiveHigh=FALSE;
    pSensorInfo->SensorResetDelayCount=5;
#if defined(CREATOR_K3C)
    pSensorInfo->SensorOutputDataFormat=SENSOR_OUTPUT_FORMAT_RAW_R;
#else
    pSensorInfo->SensorOutputDataFormat=SENSOR_OUTPUT_FORMAT_RAW_B;
#endif
//    pSensorInfo->SensorOutputDataFormat=SENSOR_OUTPUT_FORMAT_RAW_R; //2014/1/29 brad modified for R

    pSensorInfo->SensorClockPolarity=SENSOR_CLOCK_POLARITY_LOW; /*??? */
    pSensorInfo->SensorClockFallingPolarity=SENSOR_CLOCK_POLARITY_LOW;
    pSensorInfo->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
    pSensorInfo->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_HIGH;
    pSensorInfo->SensorInterruptDelayLines = 1;
    pSensorInfo->SensroInterfaceType=SENSOR_INTERFACE_TYPE_MIPI;

    pSensorInfo->CaptureDelayFrame = 2; //2
    pSensorInfo->PreviewDelayFrame = 2; //2
    pSensorInfo->VideoDelayFrame = 2; //2
    pSensorInfo->SensorMasterClockSwitch = 0; 
    pSensorInfo->SensorDrivingCurrent = ISP_DRIVING_2MA; //8     
    pSensorInfo->AEShutDelayFrame = 0;		    /* The frame of setting shutter default 0 for TG int */
    pSensorInfo->AESensorGainDelayFrame = 0;     /* The frame of setting sensor gain */
    pSensorInfo->AEISPGainDelayFrame = 2;	//2
	   
    switch (ScenarioId)
    {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
            pSensorInfo->SensorClockFreq=24;
            pSensorInfo->SensorClockDividCount=	5;
            pSensorInfo->SensorClockRisingCount= 0;
            pSensorInfo->SensorClockFallingCount= 2;
            pSensorInfo->SensorPixelClockCount= 3;
            pSensorInfo->SensorDataLatchCount= 2;
            pSensorInfo->SensorGrabStartX = HM8030MIPI_IMAGE_SENSOR_PV_STARTX; 
            pSensorInfo->SensorGrabStartY = HM8030MIPI_IMAGE_SENSOR_PV_STARTY;           		
            pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_2_LANE;			
            pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0; 
	     	pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 14; 
	    	pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
            pSensorInfo->SensorWidthSampling = 0;  // 0 is default 1x
            pSensorInfo->SensorHightSampling = 0;   // 0 is default 1x 
            pSensorInfo->SensorPacketECCOrder = 1;
            break;	
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			   pSensorInfo->SensorClockFreq=24;
			   pSensorInfo->SensorClockDividCount= 5;
			   pSensorInfo->SensorClockRisingCount= 0;
			   pSensorInfo->SensorClockFallingCount= 2;
			   pSensorInfo->SensorPixelClockCount= 3;
			   pSensorInfo->SensorDataLatchCount= 2;
			   pSensorInfo->SensorGrabStartX = HM8030MIPI_IMAGE_SENSOR_VIDEO_STARTX; 
			   pSensorInfo->SensorGrabStartY = HM8030MIPI_IMAGE_SENSOR_VIDEO_STARTY;				   
			   pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_2_LANE;		   
			   pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0; 
			pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 14; 
			pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
			   pSensorInfo->SensorWidthSampling = 0;  // 0 is default 1x
			   pSensorInfo->SensorHightSampling = 0;   // 0 is default 1x 
			   pSensorInfo->SensorPacketECCOrder = 1;

			break;
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
            pSensorInfo->SensorClockFreq=24;
            pSensorInfo->SensorClockDividCount=	5;
            pSensorInfo->SensorClockRisingCount= 0;
            pSensorInfo->SensorClockFallingCount= 2;
            pSensorInfo->SensorPixelClockCount= 3;
            pSensorInfo->SensorDataLatchCount= 2;
            pSensorInfo->SensorGrabStartX = HM8030MIPI_IMAGE_SENSOR_CAP_STARTX;	//2*HM8030MIPI_IMAGE_SENSOR_PV_STARTX; 
            pSensorInfo->SensorGrabStartY = HM8030MIPI_IMAGE_SENSOR_CAP_STARTY;	//2*HM8030MIPI_IMAGE_SENSOR_PV_STARTY;          			
            pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_2_LANE;			
            pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0; 
            pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 14; 
            pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0; 
            pSensorInfo->SensorWidthSampling = 0;  // 0 is default 1x
            pSensorInfo->SensorHightSampling = 0;   // 0 is default 1x
            pSensorInfo->SensorPacketECCOrder = 1;
            break;
        default:
			 pSensorInfo->SensorClockFreq=24;
			 pSensorInfo->SensorClockDividCount= 5;
			 pSensorInfo->SensorClockRisingCount= 0;
			 pSensorInfo->SensorClockFallingCount= 2;
			 pSensorInfo->SensorPixelClockCount= 3;
			 pSensorInfo->SensorDataLatchCount= 2;
			 pSensorInfo->SensorGrabStartX = HM8030MIPI_IMAGE_SENSOR_PV_STARTX; 
			 pSensorInfo->SensorGrabStartY = HM8030MIPI_IMAGE_SENSOR_PV_STARTY; 				 
			 pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_2_LANE;		 
			 pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0; 
		     pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 14; 
		  	 pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
			 pSensorInfo->SensorWidthSampling = 0;	// 0 is default 1x
			 pSensorInfo->SensorHightSampling = 0;	 // 0 is default 1x 
			 pSensorInfo->SensorPacketECCOrder = 1;

            break;
    }
	spin_lock(&HM8030_drv_lock);	
    HM8030MIPIPixelClockDivider=pSensorInfo->SensorPixelClockCount;
    memcpy(pSensorConfigData, &HM8030MIPISensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
	spin_unlock(&HM8030_drv_lock);
    SENSORDB("[HM8030MIPI]exit HM8030MIPIGetInfo function\n");
    return ERROR_NONE;
}   /* HM8030MIPIGetInfo() */


UINT32 HM8030MIPIControl(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *pImageWindow,
                                                MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{    
		spin_lock(&HM8030_drv_lock);	
		HM8030_CurrentScenarioId = ScenarioId;
		spin_unlock(&HM8030_drv_lock);
		SENSORDB("[HM8030MIPI]enter HM8030MIPIControl function\n");
    switch (ScenarioId)
    {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
            HM8030MIPIPreview(pImageWindow, pSensorConfigData);
            break;
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			HM8030MIPIVideo(pImageWindow, pSensorConfigData);
            break;
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
	    case MSDK_SCENARIO_ID_CAMERA_ZSD:
            HM8030MIPICapture(pImageWindow, pSensorConfigData);//hhl 2-28
            break;
        default:
            return ERROR_INVALID_SCENARIO_ID;
    }
	SENSORDB("[HM8030MIPI]exit HM8030MIPIControl function\n");
    return ERROR_NONE;
} /* HM8030MIPIControl() */

UINT32 HM8030MIPISetVideoMode(UINT16 u2FrameRate)
{
    SENSORDB("[HM8030MIPISetVideoMode] frame rate = %d\n", u2FrameRate);
	kal_uint16 HM8030MIPI_Video_Max_Expourse_Time = 0;
	SENSORDB("[HM8030MIPI]%s():fix_frame_rate=%d\n",__FUNCTION__,u2FrameRate);
	spin_lock(&HM8030_drv_lock);
	HM8030MIPI_sensor.fix_video_fps = KAL_TRUE;
	spin_unlock(&HM8030_drv_lock);
	u2FrameRate=u2FrameRate*10;//10*FPS
	if(u2FrameRate==300)
		u2FrameRate=296;
	SENSORDB("[HM8030MIPI][Enter Fix_fps func] HM8030MIPI_Fix_Video_Frame_Rate = %d\n", u2FrameRate/10);
	HM8030MIPI_Video_Max_Expourse_Time = (kal_uint16)((HM8030MIPI_sensor.video_pclk*10/u2FrameRate)/HM8030MIPI_sensor.video_line_length);
	
	if (HM8030MIPI_Video_Max_Expourse_Time > HM8030MIPI_VIDEO_FULL_LINES/*HM8030MIPI_sensor.pv_frame_length*/) 
	{
		spin_lock(&HM8030_drv_lock);    
		HM8030MIPI_sensor.video_frame_length = HM8030MIPI_Video_Max_Expourse_Time;
		HM8030MIPI_sensor.video_dummy_lines = HM8030MIPI_sensor.video_frame_length-HM8030MIPI_VIDEO_FULL_LINES;
		spin_unlock(&HM8030_drv_lock);
		SENSORDB("[HM8030MIPI]%s():frame_length=%d,dummy_lines=%d\n",__FUNCTION__,HM8030MIPI_sensor.video_frame_length,HM8030MIPI_sensor.video_dummy_lines);
		HM8030MIPI_SetDummy(HM8030MIPI_sensor.video_dummy_pixels,HM8030MIPI_sensor.video_dummy_lines);
	}
	spin_lock(&HM8030_drv_lock);    
	HM8030MIPI_MPEG4_encode_mode = KAL_TRUE;
	HM8030MIPI_sensor.FixedFps   = u2FrameRate;
	spin_unlock(&HM8030_drv_lock);
	SENSORDB("[HM8030MIPI]exit HM8030MIPISetVideoMode function\n");
	return ERROR_NONE;
}

UINT32 HM8030MIPISetAutoFlickerMode(kal_bool bEnable, UINT16 u2FrameRate)
{
	SENSORDB("OV5647MIPISetAutoFlickerMode:%d",bEnable);
	if(bEnable)
	{
		spin_lock(&HM8030_drv_lock);
		HM8030MIPIAutoFlicKerMode = KAL_TRUE;
		spin_unlock(&HM8030_drv_lock);
		/*Change frame rate 29.5fps to 29.8fps to do Auto flick*/
		if((HM8030MIPI_sensor.FixedFps == 30)&&(HM8030MIPI_sensor.video_mode==KAL_TRUE))
			HM8030MIPISetMaxFrameRate(296);
	}
	else
	{//Cancel Auto flick
		spin_lock(&HM8030_drv_lock);
		HM8030MIPIAutoFlicKerMode = KAL_FALSE;
		spin_unlock(&HM8030_drv_lock);
		if((HM8030MIPI_sensor.FixedFps == 30)&&(HM8030MIPI_sensor.video_mode==KAL_TRUE))
			HM8030MIPISetMaxFrameRate(300);
	}
	return ERROR_NONE;
}

UINT32 HM8030MIPISetMaxFramerateByScenario(MSDK_SCENARIO_ID_ENUM scenarioId, MUINT32 frameRate) {
	kal_uint32 pclk;
	kal_int16 dummyLine;
	kal_uint16 lineLength,frameHeight;	
	SENSORDB("HM8030MIPISetMaxFramerateByScenario: scenarioId = %d, frame rate = %d\n",scenarioId,frameRate);
	switch (scenarioId) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			pclk = 132000000;//120000000;
			lineLength = HM8030MIPI_PV_DUMMY_PIXELS;
			frameHeight = (10 * pclk)/frameRate/lineLength;
			dummyLine = frameHeight - HM8030MIPI_PV_DUMMY_LINES;

			
			if(dummyLine<0)
				dummyLine = 0;
			spin_lock(&HM8030_drv_lock);	
			HM8030MIPI_sensor.pv_mode=TRUE;
			spin_unlock(&HM8030_drv_lock);
			HM8030MIPI_SetDummy(0, dummyLine);			
			break;			
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			pclk = 132000000;//120000000;
			lineLength = HM8030MIPI_VIDEO_DUMMY_PIXELS;
			frameHeight = (10 * pclk)/frameRate/lineLength;
			dummyLine = frameHeight - HM8030MIPI_VIDEO_DUMMY_LINES;
			if(dummyLine<0)
				dummyLine = 0;
			spin_lock(&HM8030_drv_lock);	
			HM8030MIPI_sensor.pv_mode=TRUE;
			spin_unlock(&HM8030_drv_lock);
			HM8030MIPI_SetDummy(0, dummyLine);			
			break;			
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_CAMERA_ZSD:			
			pclk = 132000000;//120000000;
			lineLength = HM8030MIPI_FULL_DUMMY_PIXELS;
			frameHeight = (10 * pclk)/frameRate/lineLength;
			dummyLine = frameHeight - HM8030MIPI_FULL_DUMMY_LINES;
			if(dummyLine<0)
				dummyLine = 0;
			
			spin_lock(&HM8030_drv_lock);	
			HM8030MIPI_sensor.pv_mode=FALSE;
			spin_unlock(&HM8030_drv_lock);
			HM8030MIPI_SetDummy(0, dummyLine);			
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
	SENSORDB("[HM8030MIPI]exit HM8030MIPISetMaxFramerateByScenario function\n");
	return ERROR_NONE;
}
UINT32 HM8030MIPIGetDefaultFramerateByScenario(MSDK_SCENARIO_ID_ENUM scenarioId, MUINT32 *pframeRate) 
{

	switch (scenarioId) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			 *pframeRate = 300;
			 break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
			 *pframeRate = 150;
			break;		//hhl 2-28
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
UINT32 HM8030MIPISetTestPatternMode(kal_bool bEnable)
{
    SENSORDB("[HM8030MIPISetTestPatternMode] Test pattern enable:%d\n", bEnable);
    
    if(bEnable) {   // enable color bar   
        //HM8030MIPI_write_cmos_sensor(0x30D8, 0x10);  // color bar test pattern
        //HM8030MIPI_write_cmos_sensor(0x0600, 0x00);  // color bar test pattern
        //HM8030MIPI_write_cmos_sensor(0x0601, 0x02);  // color bar test pattern 
    } else {
       // HM8030MIPI_write_cmos_sensor(0x30D8, 0x00);  // disable color bar test pattern
    }
    return ERROR_NONE;
}

UINT32 HM8030MIPIFeatureControl(MSDK_SENSOR_FEATURE_ENUM FeatureId,
                                                                UINT8 *pFeaturePara,UINT32 *pFeatureParaLen)
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
    MSDK_SENSOR_ENG_INFO_STRUCT	*pSensorEngInfo=(MSDK_SENSOR_ENG_INFO_STRUCT *) pFeaturePara;

    switch (FeatureId)
    {
        case SENSOR_FEATURE_GET_RESOLUTION:
            *pFeatureReturnPara16++=HM8030MIPI_REAL_CAP_WIDTH;
            *pFeatureReturnPara16=HM8030MIPI_REAL_CAP_HEIGHT;
            *pFeatureParaLen=4;
            break;
        case SENSOR_FEATURE_GET_PERIOD:
        		switch(HM8030_CurrentScenarioId)
        		{
        			case MSDK_SCENARIO_ID_CAMERA_ZSD:
        		    case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
 		            *pFeatureReturnPara16++=HM8030MIPI_sensor.cp_line_length;  
 		            *pFeatureReturnPara16=HM8030MIPI_sensor.cp_frame_length;
		            SENSORDB("Sensor period:%d %d\n",HM8030MIPI_sensor.cp_line_length, HM8030MIPI_sensor.cp_frame_length); 
		            *pFeatureParaLen=4;        				
        				break;
        			case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
					*pFeatureReturnPara16++=HM8030MIPI_sensor.video_line_length;  
					*pFeatureReturnPara16=HM8030MIPI_sensor.video_frame_length;
					 SENSORDB("Sensor period:%d %d\n", HM8030MIPI_sensor.video_line_length, HM8030MIPI_sensor.video_frame_length); 
					 *pFeatureParaLen=4;
						break;
        			default:	
					*pFeatureReturnPara16++=HM8030MIPI_sensor.pv_line_length;  
					*pFeatureReturnPara16=HM8030MIPI_sensor.pv_frame_length;
		            SENSORDB("Sensor period:%d %d\n", HM8030MIPI_sensor.pv_line_length, HM8030MIPI_sensor.pv_frame_length); 
		            *pFeatureParaLen=4;
	            break;
          	}
          	break;
        case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
        		switch(HM8030_CurrentScenarioId)
        		{
        			case MSDK_SCENARIO_ID_CAMERA_ZSD:
        			case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		            *pFeatureReturnPara32 = HM8030MIPI_sensor.cp_pclk; 
		            *pFeatureParaLen=4;		         	
					
		            SENSORDB("Sensor CPCLK:%dn",HM8030MIPI_sensor.cp_pclk); 
		         		break; //hhl 2-28
					case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
						*pFeatureReturnPara32 = HM8030MIPI_sensor.video_pclk;
						*pFeatureParaLen=4;
						SENSORDB("Sensor videoCLK:%d\n",HM8030MIPI_sensor.video_pclk); 
						break;
		         		default:
		            *pFeatureReturnPara32 = HM8030MIPI_sensor.pv_pclk;
		            *pFeatureParaLen=4;
					SENSORDB("Sensor pvclk:%d\n",HM8030MIPI_sensor.pv_pclk); 
		            break;
		         }
		         break;
        case SENSOR_FEATURE_SET_ESHUTTER:
			SENSORDB("Walter debug SHUTTER\n"); 
            HM8030MIPI_SetShutter(*pFeatureData16); 
            break;
		case SENSOR_FEATURE_SET_SENSOR_SYNC: 
			break;
        case SENSOR_FEATURE_SET_NIGHTMODE:
            HM8030MIPI_NightMode((BOOL) *pFeatureData16);
            break;
        case SENSOR_FEATURE_SET_GAIN:
			SENSORDB("Walter debug GAiN\n");
           HM8030MIPI_SetGain((UINT16) *pFeatureData16); 
            break;
        case SENSOR_FEATURE_SET_FLASHLIGHT:
            break;
        case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
			spin_lock(&HM8030_drv_lock);    
            HM8030MIPI_isp_master_clock=*pFeatureData32;
			spin_unlock(&HM8030_drv_lock);
            break;
        case SENSOR_FEATURE_SET_REGISTER:
			HM8030MIPI_write_cmos_sensor(pSensorRegData->RegAddr, pSensorRegData->RegData);
            break;
        case SENSOR_FEATURE_GET_REGISTER:
            pSensorRegData->RegData = HM8030MIPI_read_cmos_sensor(pSensorRegData->RegAddr);
            break;
        case SENSOR_FEATURE_SET_CCT_REGISTER:
            SensorRegNumber=FACTORY_END_ADDR;
            for (i=0;i<SensorRegNumber;i++)
            {
            	spin_lock(&HM8030_drv_lock);    
                HM8030MIPISensorCCT[i].Addr=*pFeatureData32++;
                HM8030MIPISensorCCT[i].Para=*pFeatureData32++; 
				spin_unlock(&HM8030_drv_lock);
            }
            break;
        case SENSOR_FEATURE_GET_CCT_REGISTER:
            SensorRegNumber=FACTORY_END_ADDR;
            if (*pFeatureParaLen<(SensorRegNumber*sizeof(SENSOR_REG_STRUCT)+4))
                return FALSE;
            *pFeatureData32++=SensorRegNumber;
            for (i=0;i<SensorRegNumber;i++)
            {
                *pFeatureData32++=HM8030MIPISensorCCT[i].Addr;
                *pFeatureData32++=HM8030MIPISensorCCT[i].Para; 
            }
            break;
        case SENSOR_FEATURE_SET_ENG_REGISTER:
            SensorRegNumber=ENGINEER_END;
            for (i=0;i<SensorRegNumber;i++)
            {	spin_lock(&HM8030_drv_lock);    
                HM8030MIPISensorReg[i].Addr=*pFeatureData32++;
                HM8030MIPISensorReg[i].Para=*pFeatureData32++;
				spin_unlock(&HM8030_drv_lock);
            }
            break;
        case SENSOR_FEATURE_GET_ENG_REGISTER:
            SensorRegNumber=ENGINEER_END;
            if (*pFeatureParaLen<(SensorRegNumber*sizeof(SENSOR_REG_STRUCT)+4))
                return FALSE;
            *pFeatureData32++=SensorRegNumber;
            for (i=0;i<SensorRegNumber;i++)
            {
                *pFeatureData32++=HM8030MIPISensorReg[i].Addr;
                *pFeatureData32++=HM8030MIPISensorReg[i].Para;
            }
            break;
        case SENSOR_FEATURE_GET_REGISTER_DEFAULT:
            if (*pFeatureParaLen>=sizeof(NVRAM_SENSOR_DATA_STRUCT))
            {
                pSensorDefaultData->Version=NVRAM_CAMERA_SENSOR_FILE_VERSION;
                pSensorDefaultData->SensorId=HM8030MIPI_SENSOR_ID;
                memcpy(pSensorDefaultData->SensorEngReg, HM8030MIPISensorReg, sizeof(SENSOR_REG_STRUCT)*ENGINEER_END);
                memcpy(pSensorDefaultData->SensorCCTReg, HM8030MIPISensorCCT, sizeof(SENSOR_REG_STRUCT)*FACTORY_END_ADDR);
            }
            else
                return FALSE;
            *pFeatureParaLen=sizeof(NVRAM_SENSOR_DATA_STRUCT);
            break;
        case SENSOR_FEATURE_GET_CONFIG_PARA:
            memcpy(pSensorConfigData, &HM8030MIPISensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
            *pFeatureParaLen=sizeof(MSDK_SENSOR_CONFIG_STRUCT);
            break;
        case SENSOR_FEATURE_CAMERA_PARA_TO_SENSOR:
            HM8030MIPI_camera_para_to_sensor();
            break;

        case SENSOR_FEATURE_SENSOR_TO_CAMERA_PARA:
            HM8030MIPI_sensor_to_camera_para();
            break;
        case SENSOR_FEATURE_GET_GROUP_COUNT:
            *pFeatureReturnPara32++=HM8030MIPI_get_sensor_group_count();
            *pFeatureParaLen=4;
            break;
        case SENSOR_FEATURE_GET_GROUP_INFO:
            HM8030MIPI_get_sensor_group_info(pSensorGroupInfo->GroupIdx, pSensorGroupInfo->GroupNamePtr, &pSensorGroupInfo->ItemCount);
            *pFeatureParaLen=sizeof(MSDK_SENSOR_GROUP_INFO_STRUCT);
            break;
        case SENSOR_FEATURE_GET_ITEM_INFO:
            HM8030MIPI_get_sensor_item_info(pSensorItemInfo->GroupIdx,pSensorItemInfo->ItemIdx, pSensorItemInfo);
            *pFeatureParaLen=sizeof(MSDK_SENSOR_ITEM_INFO_STRUCT);
            break;

        case SENSOR_FEATURE_SET_ITEM_INFO:
            HM8030MIPI_set_sensor_item_info(pSensorItemInfo->GroupIdx, pSensorItemInfo->ItemIdx, pSensorItemInfo->ItemValue);
            *pFeatureParaLen=sizeof(MSDK_SENSOR_ITEM_INFO_STRUCT);
            break;

        case SENSOR_FEATURE_GET_ENG_INFO:
            pSensorEngInfo->SensorId = 129;
            pSensorEngInfo->SensorType = CMOS_SENSOR;
#if defined(CREATOR_K3C)
            pSensorEngInfo->SensorOutputDataFormat=SENSOR_OUTPUT_FORMAT_RAW_R;
#else
            pSensorEngInfo->SensorOutputDataFormat=SENSOR_OUTPUT_FORMAT_RAW_B;
#endif
            *pFeatureParaLen=sizeof(MSDK_SENSOR_ENG_INFO_STRUCT);
            break;
        case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
            // get the lens driver ID from EEPROM or just return LENS_DRIVER_ID_DO_NOT_CARE
            // if EEPROM does not exist in camera module.
            *pFeatureReturnPara32=LENS_DRIVER_ID_DO_NOT_CARE;
            *pFeatureParaLen=4;
            break;

        case SENSOR_FEATURE_INITIALIZE_AF:
            break;
        case SENSOR_FEATURE_CONSTANT_AF:
            break;
        case SENSOR_FEATURE_MOVE_FOCUS_LENS:
            break;
        case SENSOR_FEATURE_SET_VIDEO_MODE:
            HM8030MIPISetVideoMode(*pFeatureData16);
            break;
        case SENSOR_FEATURE_CHECK_SENSOR_ID:
            HM8030MIPIGetSensorID(pFeatureReturnPara32); 
            break;             
        case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE:
            HM8030MIPISetAutoFlickerMode((BOOL)*pFeatureData16, *(pFeatureData16+1));            
	        break;
        case SENSOR_FEATURE_SET_TEST_PATTERN:
            HM8030MIPISetTestPatternMode((BOOL)*pFeatureData16);        	
            break;
		case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
			HM8030MIPISetMaxFramerateByScenario((MSDK_SCENARIO_ID_ENUM)*pFeatureData32, *(pFeatureData32+1));
			break;
		case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
			HM8030MIPIGetDefaultFramerateByScenario((MSDK_SCENARIO_ID_ENUM)*pFeatureData32, (MUINT32 *)(*(pFeatureData32+1)));
			break;
        default:
            break;
    }
    return ERROR_NONE;
}	/* HM8030MIPIFeatureControl() */


SENSOR_FUNCTION_STRUCT	SensorFuncHM8030MIPI=
{
    HM8030MIPIOpen,
    HM8030MIPIGetInfo,
    HM8030MIPIGetResolution,
    HM8030MIPIFeatureControl,
    HM8030MIPIControl,
    HM8030MIPIClose
};

UINT32 HM8030MIPI_RAW_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
    /* To Do : Check Sensor status here */
    if (pfFunc!=NULL)
        *pfFunc=&SensorFuncHM8030MIPI;
    return ERROR_NONE;
}   /* SensorInit() */

