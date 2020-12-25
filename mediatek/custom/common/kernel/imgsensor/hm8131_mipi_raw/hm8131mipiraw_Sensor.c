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

#include <linux/timer.h>
#include <linux/jiffies.h>
 


#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"

#include "hm8131mipiraw_Sensor.h"
#include "hm8131mipiraw_Camera_Sensor_para.h"
#include "hm8131mipiraw_CameraCustomized.h"

//kal_bool HM8131MIPI_MPEG4_encode_mode = KAL_FALSE;
//kal_bool HM8131MIPI_Auto_Flicker_mode = KAL_FALSE;

//kal_uint8 HM8131MIPI_sensor_write_I2C_address = HM8131MIPI_WRITE_ID;
//kal_uint8 HM8131MIPI_sensor_read_I2C_address = HM8131MIPI_READ_ID;
static kal_bool HM8131MIPIAutoFlicKerMode = KAL_FALSE;

#define for_module 1
#define mm 0
#define DELAY_FRAME 10
static kal_uint8 frame_count = DELAY_FRAME;
static kal_uint8 current_gain = 0;
static kal_uint16 current_shutter = 0;

 
static char status[200]="";
static kal_uint8 st_i=0;


static unsigned long js,je;
kal_uint8 cpCounter=0;

static kal_uint8 restoreGain = 0;
static kal_uint16 restoreFL = 0;
static kal_uint16 restoreShutter = 0;



static struct HM8131MIPI_sensor_STRUCT HM8131MIPI_sensor =
{
    .i2c_write_id = HM8131MIPI_WRITE_ID,
    .i2c_read_id = HM8131MIPI_READ_ID,
    .first_init = KAL_TRUE,
    .fix_video_fps = KAL_FALSE,
    .pv_mode = KAL_TRUE,
    .video_mode = KAL_FALSE,
    .capture_mode = KAL_FALSE,//True: Preview Mode; False: Capture Mode
    .night_mode = KAL_FALSE,//True: Night Mode; False: Auto Mode
    .mirror_flip = KAL_FALSE,
    .pv_pclk = 34000000,//real pclk   (24/12*68/2)/2=34, 34M/888/1286=29.77fps              
    .video_pclk = 34000000,//video Pclk                
    .cp_pclk = 34000000,//Capture Pclk                 
    .pv_shutter = 0,
    .video_shutter = 0,
    .cp_shutter = 0,
    .pv_gain = 64,
    .video_gain = 64,
    .cp_gain = 64,
    .pv_line_length = HM8131MIPI_PV_FULL_PIXELS,        //use
    .pv_frame_length = HM8131MIPI_PV_FULL_LINES,        //use
    .video_line_length = HM8131MIPI_VIDEO_FULL_PIXELS,  //use
    .video_frame_length = HM8131MIPI_VIDEO_FULL_LINES,  //use
    .cp_line_length = HM8131MIPI_CAP_FULL_PIXELS,       //use
    .cp_frame_length = HM8131MIPI_CAP_FULL_LINES,       //use
    .pv_dummy_pixels = 0,//Dummy Pixels:must be 12s
    .pv_dummy_lines = 0,//Dummy Lines
    .video_dummy_pixels = 0,//Dummy Pixels:must be 12s
    .video_dummy_lines = 0,//Dummy Lines
    .cp_dummy_pixels = 0,//Dummy Pixels:must be 12s
    .cp_dummy_lines = 0,//Dummy Lines
    .video_current_frame_rate = 30,
    .frame_height = HM8131MIPI_PV_FULL_LINES,
    .line_length = HM8131MIPI_PV_FULL_PIXELS,
    .FixedFps = 0,
};

static struct HM8131_GAIN HM8131_gain_table[]=
{
//0x0204/0x0205
    {0x0000, 10000},
    {0x0001, 10625},
    {0x0002, 11250},
    {0x0003, 11875},
    {0x0004, 12500},
    {0x0005, 13125},
    {0x0006, 13750},
    {0x0007, 14375},
    {0x0008, 15000},
    {0x0009, 15625},
    {0x000A, 16250},
    {0x000B, 16875},
    {0x000C, 17500},
    {0x000D, 18125},
    {0x000E, 18750},
    {0x000F, 19375},
    {0x0010, 20000},
    {0x0012, 21250},
    {0x0014, 22500},
    {0x0016, 23750},
    {0x0018, 25000},
    {0x001A, 26250},
    {0x001C, 27500},
    {0x001E, 28750},
    {0x0020, 30000},
    {0x0022, 31250},
    {0x0024, 32500},
    {0x0026, 33750},
    {0x0028, 35000},
    {0x002A, 36250},
    {0x002C, 37500},
    {0x002E, 38750},
    {0x0030, 40000},
    {0x0034, 42500},
    {0x0038, 45000},
    {0x003C, 47500},
    {0x0040, 50000},
    {0x0044, 52500},
    {0x0048, 55000},
    {0x004C, 57500},
    {0x0050, 60000},
    {0x0054, 62500},
    {0x0058, 65000},
    {0x005C, 67500},
    {0x0060, 70000},
    {0x0064, 72500},
    {0x0068, 75000},
    {0x006C, 77500},
    {0x0070, 80000},
    {0x0078, 85000},
    {0x0080, 90000},
    {0x0088, 95000},
    {0x0090, 100000},
    {0x0098, 105000},
    {0x00A0, 110000},
    {0x00A8, 115000},
    {0x00B0, 120000},
    {0x00B8, 125000},
    {0x00C0, 130000},
    {0x00C8, 135000},
    {0x00D0, 140000},
    {0x00D8, 145000},
    {0x00E0, 150000},
    {0x00E8, 155000},
    {0x00F0, 160000},
    {0xFFFF, 0},
};

MSDK_SCENARIO_ID_ENUM HM8131_CurrentScenarioId = MSDK_SCENARIO_ID_CAMERA_PREVIEW;
//kal_uint16 HM8131MIPI_sensor_gain_base = 0x0;
/* MAX/MIN Explosure Lines Used By AE Algorithm */
kal_uint16 HM8131MIPI_MAX_EXPOSURE_LINES = HM8131MIPI_PV_DUMMY_LINES-5;//650;
kal_uint8  HM8131MIPI_MIN_EXPOSURE_LINES = 2;
kal_uint32 HM8131MIPI_isp_master_clock; //use

UINT32 HM8131MIPISetMaxFrameRate(UINT16 u2FrameRate);

static DEFINE_SPINLOCK(HM8131_drv_lock);

#define HIMAX_DEBUG
#ifdef HIMAX_DEBUG
#define SENSORDB(fmt, arg...) printk( "***Walter debug: @%s " fmt "\n", __FUNCTION__, ##arg)
#else
#define SENSORDB(fmt, arg...) printk( "[HM8131MIPIRaw] %s " fmt "\n", __FUNCTION__, ##arg)
#endif
//#define RETAILMSG(x,...)
//#define TEXT
//UINT8 HM8131MIPIPixelClockDivider = 0;
kal_uint16 HM8131MIPI_sensor_id = 0;
MSDK_SENSOR_CONFIG_STRUCT HM8131MIPISensorConfigData; //use
kal_uint32 HM8131MIPI_FAC_SENSOR_REG;
kal_uint16 HM8131MIPI_sensor_flip_value;
/* FIXME: old factors and DIDNOT use now. s*/
SENSOR_REG_STRUCT HM8131MIPISensorCCT[]=CAMERA_SENSOR_CCT_DEFAULT_VALUE; //use
SENSOR_REG_STRUCT HM8131MIPISensorReg[ENGINEER_END]=CAMERA_SENSOR_REG_DEFAULT_VALUE; //use
/* FIXME: old factors and DIDNOT use now. e*/
extern int iReadReg(u16 a_u2Addr , u8 * a_puBuff , u16 i2cId);
extern int iWriteReg(u16 a_u2Addr , u32 a_u4Data , u32 a_u4Bytes , u16 i2cId);
#define HM8131MIPI_write_cmos_sensor(addr, para) iWriteReg((u16) addr , (u32) para , 1, HM8131MIPI_WRITE_ID)

kal_uint16 HM8131MIPI_read_cmos_sensor(kal_uint32 addr)
{
    kal_uint16 get_byte=0;
    iReadReg((u16) addr ,(u8*)&get_byte,HM8131MIPI_WRITE_ID);
    return get_byte;
}

void HM8131MIPI_write_shutter(kal_uint16 shutter)
{
    kal_uint32 dummy_line = 0,dummy_pixel=0,shutter1=0,shutter2=0;
    kal_uint32 extra_lines = 0;
    kal_uint32 max_exp_shutter = 0;
    unsigned long flags;
    //SENSORDB("[HM8131MIPI]enter HM8131MIPI_write_shutter function,shutter=%d\n",shutter);
    SENSORDB("[brad], shutter = %d", shutter);
    if(HM8131MIPIAutoFlicKerMode)
    {
        if(HM8131MIPI_sensor.video_mode == KAL_FALSE)
        {
            if(HM8131_CurrentScenarioId == MSDK_SCENARIO_ID_CAMERA_ZSD)
            {
                //Change frame 14.7fps ~ 14.9fps to do auto flick
                HM8131MIPISetMaxFrameRate(148);
            }
            else
            {
                //Change frame 29.5fps ~ 29.8fps to do auto flick
                HM8131MIPISetMaxFrameRate(296);// Used to be 148. If input=296,then frameheight=1293,dummy_line=7.
                                               //If input=148, then frameheight=2587,dummy_line=1301. These values are not
                                               //proper although 0340,0341 were not set in function set_dummy like OVxxxx
            }
        }
    }
    if (HM8131MIPI_sensor.pv_mode == KAL_TRUE)
    {
        max_exp_shutter = HM8131MIPI_PV_FULL_LINES + HM8131MIPI_sensor.pv_dummy_lines;
    }
    else if (HM8131MIPI_sensor.video_mode== KAL_TRUE)
    {
        max_exp_shutter = HM8131MIPI_VIDEO_FULL_LINES + HM8131MIPI_sensor.video_dummy_lines;
    }
    else if (HM8131MIPI_sensor.capture_mode== KAL_TRUE)
    {
        max_exp_shutter = HM8131MIPI_CAP_FULL_LINES + HM8131MIPI_sensor.cp_dummy_lines;
    }
    else
    {
        SENSORDB("sensor mode error");
    }

    if(shutter > max_exp_shutter)
        extra_lines = shutter - max_exp_shutter;
    else
        extra_lines = 0;
    if (HM8131MIPI_sensor.pv_mode == KAL_TRUE)
    {
        SENSORDB("PV shutter, shutter = 0x%x",shutter);
        dummy_line =HM8131MIPI_PV_DUMMY_LINES+ HM8131MIPI_sensor.pv_dummy_lines + extra_lines;
        dummy_pixel = HM8131MIPI_PV_DUMMY_PIXELS+ HM8131MIPI_sensor.pv_dummy_pixels;
        spin_lock_irqsave(&HM8131_drv_lock,flags);
        HM8131MIPI_sensor.pv_line_length  = HM8131MIPI_PV_FULL_PIXELS+dummy_pixel;
        HM8131MIPI_sensor.pv_frame_length = HM8131MIPI_PV_FULL_LINES+dummy_line;
        HM8131MIPI_sensor.frame_height= HM8131MIPI_sensor.pv_frame_length;
        HM8131MIPI_sensor.line_length = HM8131MIPI_sensor.pv_line_length;
        spin_unlock_irqrestore(&HM8131_drv_lock,flags);
    }
    else if (HM8131MIPI_sensor.video_mode== KAL_TRUE)
    {
        SENSORDB("video shutter, shutter = 0x%x",shutter);
        dummy_line = HM8131MIPI_VIDEO_DUMMY_LINES+ HM8131MIPI_sensor.video_dummy_lines + extra_lines;
        dummy_pixel =HM8131MIPI_VIDEO_DUMMY_PIXELS + HM8131MIPI_sensor.video_dummy_pixels;
        spin_lock_irqsave(&HM8131_drv_lock,flags);
        HM8131MIPI_sensor.video_line_length  = HM8131MIPI_VIDEO_FULL_PIXELS+dummy_pixel;
        HM8131MIPI_sensor.video_frame_length = HM8131MIPI_VIDEO_FULL_LINES+dummy_line;
        HM8131MIPI_sensor.line_length  = HM8131MIPI_sensor.video_line_length;
        HM8131MIPI_sensor.frame_height = HM8131MIPI_sensor.video_frame_length;
        spin_unlock_irqrestore(&HM8131_drv_lock,flags);
    }
    else if(HM8131MIPI_sensor.capture_mode== KAL_TRUE)
    {
        SENSORDB("cap shutter, shutter = 0x%x",shutter);
        dummy_line = HM8131MIPI_PV_DUMMY_LINES+ HM8131MIPI_sensor.pv_dummy_lines + extra_lines;
        dummy_pixel =HM8131MIPI_PV_DUMMY_PIXELS+ HM8131MIPI_sensor.pv_dummy_pixels;
        spin_lock_irqsave(&HM8131_drv_lock,flags);
        HM8131MIPI_sensor.cp_line_length = HM8131MIPI_CAP_FULL_PIXELS+dummy_pixel;
        HM8131MIPI_sensor.cp_frame_length = HM8131MIPI_CAP_FULL_LINES+dummy_line;
        HM8131MIPI_sensor.frame_height = HM8131MIPI_sensor.cp_frame_length;
        HM8131MIPI_sensor.line_length  = HM8131MIPI_sensor.cp_line_length;
        spin_unlock_irqrestore(&HM8131_drv_lock,flags);
    }
    else
    {
        SENSORDB("sensor mode error");
    }


    restoreShutter=shutter;
    HM8131MIPI_write_cmos_sensor(0x0202, (shutter >> 8) & 0xFF);
    HM8131MIPI_write_cmos_sensor(0x0203,  shutter  & 0xFF);


    shutter2=shutter+0x0A;

    if ((shutter2<0x09E6) && HM8131MIPI_sensor.capture_mode== KAL_TRUE)
    {
        HM8131MIPI_write_cmos_sensor(0x0340,  0x09);
        HM8131MIPI_write_cmos_sensor(0x0341,  0xE6);
    }
    else if ( (HM8131MIPI_sensor.video_mode== KAL_TRUE || HM8131MIPI_sensor.pv_mode== KAL_TRUE) && shutter2< 0X051A)
    {
        HM8131MIPI_write_cmos_sensor(0x0340,  0x05);
        HM8131MIPI_write_cmos_sensor(0x0341,  0x1A);
    }
    else
    {
        HM8131MIPI_write_cmos_sensor(0x0340,  (shutter2 >> 8) & 0xFF);
        HM8131MIPI_write_cmos_sensor(0x0341,  shutter2  & 0xFF);
    }

    SENSORDB("[E]");
}   /* write_HM8131MIPI_shutter */


/*************************************************************************
* FUNCTION
*    HM8131MIPIGain2Reg
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
static kal_uint8 HM8131MIPIGain2Reg(const kal_uint16 iGain)
{
    SENSORDB("[S], iGain = %d", iGain);
    kal_uint8 i = 0;
    kal_uint8 gain_tmp1=0,Reg_Cgain=0,Reg_Fgain=0;
    kal_uint32 gain_tmp0 = 0;

    gain_tmp0=((kal_uint32)iGain*10000) / BASEGAIN;
    do
    {
        if (gain_tmp0 < HM8131_gain_table[i].analoggain)
            break;
        i++;
    }
    while (HM8131_gain_table[i].analoggain != 0);

    if (i == 0)
    {
        Reg_Cgain = 0x00;
    }
    else
    {
        Reg_Cgain = HM8131_gain_table[i-1].gainvalue & 0xFF;
    }


    restoreGain=Reg_Cgain;

    HM8131MIPI_write_cmos_sensor(0x0205,Reg_Cgain);
    HM8131MIPI_write_cmos_sensor(0x0104, 0x00);
    SENSORDB("[E], Reg_Cgain = 0x%x", Reg_Cgain);
    return NONE;
}

/*************************************************************************
* FUNCTION
*    HM8131MIPI_SetGain
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
void HM8131MIPI_SetGain(UINT16 iGain)
{
    SENSORDB("[S], iGain = 0x%x", iGain);
    HM8131MIPIGain2Reg(iGain);
    SENSORDB("[E]");
}   /*  HM8131MIPI_SetGain_SetGain  */

/*************************************************************************
* FUNCTION
*    read_HM8131MIPI_gain
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
kal_uint16 read_HM8131MIPI_gain(void)
{
    kal_uint16 Reg_Cgain=0,Reg_Fgain=0,i=0;
    float gain_tmp0 =0;
    SENSORDB("[HM8131MIPI]enter read_HM8131MIPI_gain function\n");
    Reg_Cgain=HM8131MIPI_read_cmos_sensor(0x0204);
    Reg_Fgain=HM8131MIPI_read_cmos_sensor(0x0205);
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
    SENSORDB("[HM8131MIPI]exit read_HM8131MIPI_gain function \n");
    return gain_tmp0;
}  /* read_HM8131MIPI_gain */

void HM8131MIPI_camera_para_to_sensor(void)
{
    kal_uint32 i;
    SENSORDB("[HM8131MIPI]enter HM8131MIPI_camera_para_to_sensor function\n");
    for(i=0; 0xFFFFFFFF!=HM8131MIPISensorReg[i].Addr; i++)
    {
        HM8131MIPI_write_cmos_sensor(HM8131MIPISensorReg[i].Addr, HM8131MIPISensorReg[i].Para);
    }
    for(i=ENGINEER_START_ADDR; 0xFFFFFFFF!=HM8131MIPISensorReg[i].Addr; i++)
    {
        HM8131MIPI_write_cmos_sensor(HM8131MIPISensorReg[i].Addr, HM8131MIPISensorReg[i].Para);
    }
    for(i=FACTORY_START_ADDR; i<FACTORY_END_ADDR; i++)
    {
        HM8131MIPI_write_cmos_sensor(HM8131MIPISensorCCT[i].Addr, HM8131MIPISensorCCT[i].Para);
    }
    SENSORDB("[HM8131MIPI]exit HM8131MIPI_camera_para_to_sensor function\n");
}

/*************************************************************************
* FUNCTION
*    HM8131MIPI_sensor_to_camera_para
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
void HM8131MIPI_sensor_to_camera_para(void)
{
    kal_uint32    i,temp_data;
    SENSORDB("[HM8131MIPI]enter HM8131MIPI_sensor_to_camera_para function\n");
    for(i=0; 0xFFFFFFFF!=HM8131MIPISensorReg[i].Addr; i++)
    {
        temp_data=HM8131MIPI_read_cmos_sensor(HM8131MIPISensorReg[i].Addr);
        spin_lock(&HM8131_drv_lock);
        HM8131MIPISensorReg[i].Para = temp_data;
        spin_unlock(&HM8131_drv_lock);
    }
    for(i=ENGINEER_START_ADDR; 0xFFFFFFFF!=HM8131MIPISensorReg[i].Addr; i++)
    {
        temp_data=HM8131MIPI_read_cmos_sensor(HM8131MIPISensorReg[i].Addr);
        spin_lock(&HM8131_drv_lock);
        HM8131MIPISensorReg[i].Para = temp_data;
        spin_unlock(&HM8131_drv_lock);
    }
    SENSORDB("[HM8131MIPI]exit HM8131MIPI_sensor_to_camera_para function\n");
}

/*************************************************************************
* FUNCTION
*    HM8131MIPI_get_sensor_group_count
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
kal_int32  HM8131MIPI_get_sensor_group_count(void)
{
    return GROUP_TOTAL_NUMS;
}

void HM8131MIPI_get_sensor_group_info(kal_uint16 group_idx, kal_int8* group_name_ptr, kal_int32* item_count_ptr)
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

void HM8131MIPI_get_sensor_item_info(kal_uint16 group_idx,kal_uint16 item_idx, MSDK_SENSOR_ITEM_INFO_STRUCT* info_ptr)
{
}

kal_bool HM8131MIPI_set_sensor_item_info(kal_uint16 group_idx, kal_uint16 item_idx, kal_int32 ItemValue)
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
        temp_para = HM8131MIPIGain2Reg(ItemValue);
        spin_lock(&HM8131_drv_lock);
        HM8131MIPISensorCCT[temp_addr].Para = temp_para;
        spin_unlock(&HM8131_drv_lock);
        HM8131MIPI_write_cmos_sensor(HM8131MIPISensorCCT[temp_addr].Addr,temp_para);
        temp_para=read_HM8131MIPI_gain();
        //spin_lock(&HM8131_drv_lock);
        //HM8131MIPI_sensor_gain_base=temp_para;
        //spin_unlock(&HM8131_drv_lock);
        break;
    case CMMCLK_CURRENT:
        switch (item_idx)
        {
        case 0:
            if(ItemValue==2)
            {
                spin_lock(&HM8131_drv_lock);
                HM8131MIPISensorReg[CMMCLK_CURRENT_INDEX].Para = ISP_DRIVING_2MA;
                spin_unlock(&HM8131_drv_lock);
                //HM8131MIPI_set_isp_driving_current(ISP_DRIVING_2MA);
            }
            else if(ItemValue==3 || ItemValue==4)
            {
                spin_lock(&HM8131_drv_lock);
                HM8131MIPISensorReg[CMMCLK_CURRENT_INDEX].Para = ISP_DRIVING_4MA;
                spin_unlock(&HM8131_drv_lock);
                //HM8131MIPI_set_isp_driving_current(ISP_DRIVING_4MA);
            }
            else if(ItemValue==5 || ItemValue==6)
            {
                spin_lock(&HM8131_drv_lock);
                HM8131MIPISensorReg[CMMCLK_CURRENT_INDEX].Para = ISP_DRIVING_6MA;
                spin_unlock(&HM8131_drv_lock);
                //HM8131MIPI_set_isp_driving_current(ISP_DRIVING_6MA);
            }
            else
            {
                spin_lock(&HM8131_drv_lock);
                HM8131MIPISensorReg[CMMCLK_CURRENT_INDEX].Para = ISP_DRIVING_8MA;
                spin_unlock(&HM8131_drv_lock);
                //HM8131MIPI_set_isp_driving_current(ISP_DRIVING_8MA);
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
            spin_lock(&HM8131_drv_lock);
            HM8131MIPI_FAC_SENSOR_REG=ItemValue;
            spin_unlock(&HM8131_drv_lock);
            break;
        case 1:
            HM8131MIPI_write_cmos_sensor(HM8131MIPI_FAC_SENSOR_REG,ItemValue);
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


static void HM8131MIPI_SetDummy(const kal_uint16 iPixels, const kal_uint16 iLines)
{
    //kal_uint32 frame_length = 0, line_length = 0,dummy_tmp0=0,dummy_tmp1=0;
    //SENSORDB("[HM8131MIPI]enter HM8131MIPI_SetDummy function,iPixels=%d,iLines=%d\n",iPixels,iLines);
    SENSORDB("[S], iPixels=%d, iLines=%d" ,iPixels, iLines);
    if(HM8131MIPI_sensor.pv_mode == KAL_TRUE)
    {
        SENSORDB("pv_mode");
        spin_lock(&HM8131_drv_lock);
        HM8131MIPI_sensor.pv_dummy_pixels =  HM8131MIPI_PV_DUMMY_PIXELS+iPixels;
        HM8131MIPI_sensor.pv_dummy_lines  =  HM8131MIPI_PV_DUMMY_LINES + iLines;
        HM8131MIPI_sensor.pv_line_length  =  HM8131MIPI_PV_FULL_PIXELS + iPixels;
        HM8131MIPI_sensor.pv_frame_length =  HM8131MIPI_PV_FULL_LINES + iLines;
        HM8131MIPI_sensor.frame_height    =  HM8131MIPI_sensor.pv_frame_length;
        HM8131MIPI_sensor.line_length     =  HM8131MIPI_sensor.pv_line_length;
        spin_unlock(&HM8131_drv_lock);
        //line_length = HM8131MIPI_sensor.pv_dummy_pixels-164;
        //frame_length = HM8131MIPI_sensor.pv_dummy_lines-97;
    }
    else if(HM8131MIPI_sensor.video_mode== KAL_TRUE)
    {
        SENSORDB("video_mode");
        spin_lock(&HM8131_drv_lock);
        HM8131MIPI_sensor.video_dummy_pixels = HM8131MIPI_VIDEO_DUMMY_PIXELS + iPixels;
        HM8131MIPI_sensor.video_dummy_lines  = HM8131MIPI_VIDEO_DUMMY_LINES + iLines;
        HM8131MIPI_sensor.video_line_length  = HM8131MIPI_VIDEO_FULL_PIXELS + iPixels;
        HM8131MIPI_sensor.video_frame_length = HM8131MIPI_VIDEO_FULL_LINES + iLines;
        HM8131MIPI_sensor.frame_height       = HM8131MIPI_sensor.video_frame_length;
        HM8131MIPI_sensor.line_length        = HM8131MIPI_sensor.video_line_length;
        spin_unlock(&HM8131_drv_lock);
        //line_length = HM8131MIPI_sensor.video_dummy_pixels-164;
        //frame_length = HM8131MIPI_sensor.video_dummy_lines-97;
    }
    else if(HM8131MIPI_sensor.capture_mode== KAL_TRUE)
    {
        SENSORDB("capture_mode");
        spin_lock(&HM8131_drv_lock);
        HM8131MIPI_sensor.cp_dummy_pixels = HM8131MIPI_FULL_DUMMY_PIXELS + iPixels;;
        HM8131MIPI_sensor.cp_dummy_lines  = HM8131MIPI_FULL_DUMMY_LINES + iLines;
        HM8131MIPI_sensor.cp_line_length  = HM8131MIPI_CAP_FULL_PIXELS + iPixels;
        HM8131MIPI_sensor.cp_frame_length = HM8131MIPI_CAP_FULL_LINES + iLines;
        HM8131MIPI_sensor.frame_height    = HM8131MIPI_sensor.cp_frame_length;
        HM8131MIPI_sensor.line_length     = HM8131MIPI_sensor.cp_line_length;
        spin_unlock(&HM8131_drv_lock);
        //line_length = HM8131MIPI_sensor.cp_dummy_pixels-164;
        //frame_length = HM8131MIPI_sensor.cp_dummy_lines-97;
    }
    else
    {
        SENSORDB("sensor mode error");
    }

    SENSORDB("[E]");
}   /*  HM8131MIPI_SetDummy */

static void analog(void)
{
    HM8131MIPI_write_cmos_sensor(0x414A, 0x02);
    HM8131MIPI_write_cmos_sensor(0x4147, 0x03);
    HM8131MIPI_write_cmos_sensor(0x4144, 0x03);
    HM8131MIPI_write_cmos_sensor(0x4145, 0x31);
    HM8131MIPI_write_cmos_sensor(0x4146, 0x51);
    HM8131MIPI_write_cmos_sensor(0x4149, 0x57);
    HM8131MIPI_write_cmos_sensor(0x4260, 0x00);
    HM8131MIPI_write_cmos_sensor(0x4261, 0x00);
    HM8131MIPI_write_cmos_sensor(0x426A, 0x01);
    HM8131MIPI_write_cmos_sensor(0x4270, 0x08);


    HM8131MIPI_write_cmos_sensor(0x4271, 0xBF);
    HM8131MIPI_write_cmos_sensor(0x4272, 0x00);
    HM8131MIPI_write_cmos_sensor(0x4383, 0x98);
    HM8131MIPI_write_cmos_sensor(0x4387, 0x17);
    HM8131MIPI_write_cmos_sensor(0x4386, 0x32);
    HM8131MIPI_write_cmos_sensor(0x438A, 0x00);

    HM8131MIPI_write_cmos_sensor(0x427D, 0x00);
    HM8131MIPI_write_cmos_sensor(0x427E, 0x03);
    HM8131MIPI_write_cmos_sensor(0x427F, 0x00);
    HM8131MIPI_write_cmos_sensor(0x4380, 0xA6);
    HM8131MIPI_write_cmos_sensor(0x4381, 0x7B);
    HM8131MIPI_write_cmos_sensor(0x4382, 0x00);
    HM8131MIPI_write_cmos_sensor(0x4388, 0x9F);
    HM8131MIPI_write_cmos_sensor(0x4389, 0x15);
    HM8131MIPI_write_cmos_sensor(0x438C, 0x0F);
    HM8131MIPI_write_cmos_sensor(0x4384, 0x14);
    HM8131MIPI_write_cmos_sensor(0x438B, 0x00);
    HM8131MIPI_write_cmos_sensor(0x4385, 0xA5);
    HM8131MIPI_write_cmos_sensor(0x438F, 0x00);
    HM8131MIPI_write_cmos_sensor(0x438D, 0xA0);

    HM8131MIPI_write_cmos_sensor(0x4B11, 0x1F);

    HM8131MIPI_write_cmos_sensor(0x4B44, 0x00);//mipi enable phy to LDO: 1.5=>1.2
    HM8131MIPI_write_cmos_sensor(0x4B46, 0x03);
    HM8131MIPI_write_cmos_sensor(0x4B47, 0xC9);


//===========
    HM8131MIPI_write_cmos_sensor(0x44B0, 0x03);
    HM8131MIPI_write_cmos_sensor(0x44B1, 0x01);
    HM8131MIPI_write_cmos_sensor(0x44B2, 0x00);
    HM8131MIPI_write_cmos_sensor(0x44B3, 0x04);
    HM8131MIPI_write_cmos_sensor(0x44B4, 0x14);
    HM8131MIPI_write_cmos_sensor(0x44B5, 0x24);
  

    HM8131MIPI_write_cmos_sensor(0x44B8, 0x03);
    HM8131MIPI_write_cmos_sensor(0x44B9, 0x01);
    HM8131MIPI_write_cmos_sensor(0x44BA, 0x05);
    HM8131MIPI_write_cmos_sensor(0x44BB, 0x15);
    HM8131MIPI_write_cmos_sensor(0x44BC, 0x25);
    HM8131MIPI_write_cmos_sensor(0x44BD, 0x35);

//===========

    HM8131MIPI_write_cmos_sensor(0x44D0, 0xC0);
    HM8131MIPI_write_cmos_sensor(0x44D1, 0x80);
    HM8131MIPI_write_cmos_sensor(0x44D2, 0x40);
    HM8131MIPI_write_cmos_sensor(0x44D3, 0x40);
    HM8131MIPI_write_cmos_sensor(0x44D4, 0x40);
    HM8131MIPI_write_cmos_sensor(0x44D5, 0x40);
    HM8131MIPI_write_cmos_sensor(0x4B07, 0xF0);
    HM8131MIPI_write_cmos_sensor(0x4131, 0x01);

    HM8131MIPI_write_cmos_sensor(0x060B, 0x01);
}

static void HM8131MIPI_Sensor_Init(void)
{

  



     status[st_i]='I'; st_i++;
     SENSORDB("STATUS:: %s \n",status);
}   /*  HM8131MIPI_Sensor_Init  */


static void HM8131MIPI_Sensor_8M_nonReset(void)
{


    //SENSORDB("[HM8131MIPI]enter HM8131MIPI_Sensor_Init function\n");
    SENSORDB("[S]");
  mdelay(10);


    HM8131MIPI_write_cmos_sensor(0x3519,0x00);
    HM8131MIPI_write_cmos_sensor(0x351A,0x05);
    HM8131MIPI_write_cmos_sensor(0x351B,0x1E);
    HM8131MIPI_write_cmos_sensor(0x351C,0x90);
    HM8131MIPI_write_cmos_sensor(0x351E,0x15);
    HM8131MIPI_write_cmos_sensor(0x351D,0x15);
    HM8131MIPI_write_cmos_sensor(0x4001,0x80);
    mdelay(10);

    HM8131MIPI_write_cmos_sensor(0xBA93,0x01);
    mdelay(10);


    HM8131MIPI_write_cmos_sensor(0x412A,0x8A);
    HM8131MIPI_write_cmos_sensor(0xBAA2,0xC0);
    HM8131MIPI_write_cmos_sensor(0xBAA2,0xC0);
    HM8131MIPI_write_cmos_sensor(0xBAA2,0xC0);
    HM8131MIPI_write_cmos_sensor(0xBAA2,0x40);
    HM8131MIPI_write_cmos_sensor(0x412A,0x8A);
    HM8131MIPI_write_cmos_sensor(0x412A,0x8A);
    HM8131MIPI_write_cmos_sensor(0x412A,0x0A);

    mdelay(10);

    HM8131MIPI_write_cmos_sensor(0xBA93,0x03);
    HM8131MIPI_write_cmos_sensor(0xBA93,0x02);
    HM8131MIPI_write_cmos_sensor(0xBA90,0x01);

    mdelay(10);
    //mipi 2-lane 1216 version
    HM8131MIPI_write_cmos_sensor(0x0303,0x02); //2 1 ; PLL adc1 - enable PLL -> was 002A
    HM8131MIPI_write_cmos_sensor(0x0305,0x0C); //2 1 ; PLL N, mclk 24mhz
    HM8131MIPI_write_cmos_sensor(0x0307,0x44); //2 1 ; PLL M, pclk_raw=68mhz
    HM8131MIPI_write_cmos_sensor(0x030D,0x0C); //2 1 ; PLL N,
    HM8131MIPI_write_cmos_sensor(0x030F,0x5E); //2 1 ; PLL M, pkt_clk=94mhz
    mdelay(5);

    HM8131MIPI_write_cmos_sensor(0x4001,0x00);
    analog();

    mdelay(30);
    HM8131MIPI_write_cmos_sensor(0x0350,0x33);

    //new!!
    HM8131MIPI_write_cmos_sensor(0x3110,0x23);
    HM8131MIPI_write_cmos_sensor(0x3111,0x00);
    HM8131MIPI_write_cmos_sensor(0x3130,0x01);
    HM8131MIPI_write_cmos_sensor(0x3131,0x80);
    //

    HM8131MIPI_write_cmos_sensor(0x4274,0x33); //2 1 ; [5] mask out bad frame due to mode (flip/mirror) change
    HM8131MIPI_write_cmos_sensor(0x4002,0x23); //2 1 ; output BPC
    HM8131MIPI_write_cmos_sensor(0x4B18,0x28); //2 1 ; [7:0] FULL_TRIGGER_TIME
    HM8131MIPI_write_cmos_sensor(0x4B08,0xA0); //2 1 ; VSIZE (2464)
    HM8131MIPI_write_cmos_sensor(0x4B09,0x09); //2 1 ;
    HM8131MIPI_write_cmos_sensor(0x4B0A,0xD0); //2 1 ; HSIZE (3280)
    HM8131MIPI_write_cmos_sensor(0x4B0B,0x0C); //2 1 ;
    HM8131MIPI_write_cmos_sensor(0x0111,0x00); //2 1 ; B5: 1'b0:2lane, 1'b1:4lane (org 4B19)
    HM8131MIPI_write_cmos_sensor(0x4B20,0xDE); //2 1 ; clock always on(9E) / clock always on while sending packet(BE)
    HM8131MIPI_write_cmos_sensor(0x4B0E,0x03);
    HM8131MIPI_write_cmos_sensor(0x4B42,0x05);

#if mm
    HM8131MIPI_write_cmos_sensor(0x4B02,0x05); //2 1 ; lpx_width
#endif
//=====================


    HM8131MIPI_write_cmos_sensor(0x4B03,0x0E); //2 1 ; hs_zero_width
    HM8131MIPI_write_cmos_sensor(0x4B04,0x05); //2 1 ; hs_trail_width


#if mm

    HM8131MIPI_write_cmos_sensor(0x4B05,0x1B); //2 1 ; clk_zero_width
#endif
//=====================


    HM8131MIPI_write_cmos_sensor(0x4B06,0x06); //2 1 ; clk_trail_width

#if mm
    HM8131MIPI_write_cmos_sensor(0x4B0F,0x0A); //2 1 ; clk_back_porch
    HM8131MIPI_write_cmos_sensor(0x4B39,0x08); //2 1 ; clk_width_exit
#endif

    HM8131MIPI_write_cmos_sensor(0x4B3F,0x18); //2 1 ; [3:0] mipi_req_dly

#if mm
    HM8131MIPI_write_cmos_sensor(0x4B42,0x07); //2 1 ; HS_PREPARE_WIDTH
    HM8131MIPI_write_cmos_sensor(0x4B43,0x06); //2 1 ; CLK_PREPARE_WIDTH
#endif


    HM8131MIPI_write_cmos_sensor(0x4024,0x40); //2 1 ; enabled MIPI -> was 0024

#if for_module
    HM8131MIPI_write_cmos_sensor(0x0101,0x03); //flip+mirror
#else
    HM8131MIPI_write_cmos_sensor(0x0101,0x00); //flip+mirror
#endif


    HM8131MIPI_write_cmos_sensor(0x4800,0x00);
    HM8131MIPI_write_cmos_sensor(0x0104,0x01);
    HM8131MIPI_write_cmos_sensor(0x0104,0x00);
    HM8131MIPI_write_cmos_sensor(0x4801,0x00);
    HM8131MIPI_write_cmos_sensor(0x0000,0x00);

    HM8131MIPI_write_cmos_sensor(0xBA94,0x01);
    HM8131MIPI_write_cmos_sensor(0xBA94,0x00);
    HM8131MIPI_write_cmos_sensor(0xBA93,0x02);


    SENSORDB("[E]");
}   /*  HM8131MIPI_Sensor_Init  */



static void previewSetting(void)
{
    SENSORDB("[S]");




    mdelay(10);


    HM8131MIPI_write_cmos_sensor(0x3519,0x00);
    HM8131MIPI_write_cmos_sensor(0x351A,0x05);
    HM8131MIPI_write_cmos_sensor(0x351B,0x1E);
    HM8131MIPI_write_cmos_sensor(0x351C,0x90);
    HM8131MIPI_write_cmos_sensor(0x351E,0x15);
    HM8131MIPI_write_cmos_sensor(0x351D,0x15);
    HM8131MIPI_write_cmos_sensor(0x4001,0x80);
    mdelay(10);

    HM8131MIPI_write_cmos_sensor(0xBA93,0x01);
    mdelay(10);


    HM8131MIPI_write_cmos_sensor(0x412A,0x8A);
    HM8131MIPI_write_cmos_sensor(0xBAA2,0xC0);
    HM8131MIPI_write_cmos_sensor(0xBAA2,0xC0);
    HM8131MIPI_write_cmos_sensor(0xBAA2,0xC0);
    HM8131MIPI_write_cmos_sensor(0xBAA2,0x40);
    HM8131MIPI_write_cmos_sensor(0x412A,0x8A);
    HM8131MIPI_write_cmos_sensor(0x412A,0x8A);
    HM8131MIPI_write_cmos_sensor(0x412A,0x0A);

    mdelay(10);

    HM8131MIPI_write_cmos_sensor(0xBA93,0x03);
    HM8131MIPI_write_cmos_sensor(0xBA93,0x02);
    HM8131MIPI_write_cmos_sensor(0xBA90,0x01);

    mdelay(10);


    //mipi 2-lane 1216 version
    HM8131MIPI_write_cmos_sensor(0x0303,0x02); //2 1 ; PLL adc1 - enable PLL -> was 002A
    HM8131MIPI_write_cmos_sensor(0x0305,0x0C); //2 1 ; PLL N, mclk 24mhz
    HM8131MIPI_write_cmos_sensor(0x0307,0x44); //2 1 ; PLL M, pclk_raw=68mhz
    HM8131MIPI_write_cmos_sensor(0x030D,0x0C); //2 1 ; PLL N,
    HM8131MIPI_write_cmos_sensor(0x030F,0x62); //2 1 ; PLL M, pkt_clk=94mhz
mdelay(5);
    HM8131MIPI_write_cmos_sensor(0x4001,0x00);
    analog();
mdelay(30);

    //new!!
    HM8131MIPI_write_cmos_sensor(0x3110,0x01);
    HM8131MIPI_write_cmos_sensor(0x3111,0x01);
    HM8131MIPI_write_cmos_sensor(0x3130,0x01);
    HM8131MIPI_write_cmos_sensor(0x3131,0x26);
    //


    HM8131MIPI_write_cmos_sensor(0x0309, 0x02);
    HM8131MIPI_write_cmos_sensor(0x400D, 0x04);
    HM8131MIPI_write_cmos_sensor(0x0383, 0x03);
    HM8131MIPI_write_cmos_sensor(0x0387, 0x03);
    HM8131MIPI_write_cmos_sensor(0x0390, 0x11);

    mdelay(30);



    HM8131MIPI_write_cmos_sensor(0x4274,0x33); //2 1 ; [5] mask out bad frame due to mode (flip/mirror) change
    HM8131MIPI_write_cmos_sensor(0x4002,0x23); //2 1 ; output BPC
/*//
    HM8131MIPI_write_cmos_sensor(0x4B08,0xD0); //2 1 ; VSIZE (2464)
    HM8131MIPI_write_cmos_sensor(0x4B09,0x04); //2 1 ;
    HM8131MIPI_write_cmos_sensor(0x4B0A,0x68); //2 1 ; HSIZE (3280)
    HM8131MIPI_write_cmos_sensor(0x4B0B,0x06); //2 1 ;
*/
    HM8131MIPI_write_cmos_sensor(0x0350,0x33);
    HM8131MIPI_write_cmos_sensor(0x0348,0x0C);
    HM8131MIPI_write_cmos_sensor(0x0349,0xCD);
    HM8131MIPI_write_cmos_sensor(0x034A,0x09);
    HM8131MIPI_write_cmos_sensor(0x034B,0x9D);
    HM8131MIPI_write_cmos_sensor(0x0340,0x05);
    HM8131MIPI_write_cmos_sensor(0x0341,0x06);

    HM8131MIPI_write_cmos_sensor(0x034C,0x06);//Image size X end Hb
    HM8131MIPI_write_cmos_sensor(0x034D,0x68);//Image size X end Lb
    HM8131MIPI_write_cmos_sensor(0x034E,0x04);//Image size Y end Hb
    HM8131MIPI_write_cmos_sensor(0x034F,0xD0);// Image size Y end Lb

     //suspect :gj say both 06
    HM8131MIPI_write_cmos_sensor(0x4B31,0x06); //SUSPECT
    HM8131MIPI_write_cmos_sensor(0x4B18,0x18); //2 1 ; [7:0] FULL_TRIGGER_TIME//SUSPECT
    
    HM8131MIPI_write_cmos_sensor(0x0111,0x00); //2 1 ; B5: 1'b0:2lane, 1'b1:4lane (org 4B19)
    HM8131MIPI_write_cmos_sensor(0x4B20,0xDE); //2 1 ; clock always on(9E) / clock always on while sending packet(BE)
    HM8131MIPI_write_cmos_sensor(0x4B0E,0x01);
    HM8131MIPI_write_cmos_sensor(0x4B42,0x02);
    //HM8131MIPI_write_cmos_sensor(0x4B02,0x02); //2 1 ; lpx_width
    //HM8131MIPI_write_cmos_sensor(0x4B03,0x05); //2 1 ; hs_zero_width
    HM8131MIPI_write_cmos_sensor(0x4B04,0x02); //2 1 ; hs_trail_width
    //HM8131MIPI_write_cmos_sensor(0x4B05,0x0C); //2 1 ; clk_zero_width
    HM8131MIPI_write_cmos_sensor(0x4B06,0x03); //2 1 ; clk_trail_width
    //HM8131MIPI_write_cmos_sensor(0x4B0F,0x07); //2 1 ; clk_back_porch
    //HM8131MIPI_write_cmos_sensor(0x4B39,0x02); //2 1 ; clk_width_exit
    HM8131MIPI_write_cmos_sensor(0x4B3F,0x10); //2 1 ; [3:0] mipi_req_dly
    HM8131MIPI_write_cmos_sensor(0x4B42,0x02); //2 1 ; HS_PREPARE_WIDTH
    HM8131MIPI_write_cmos_sensor(0x4B43,0x02); //2 1 ; CLK_PREPARE_WIDTH
    HM8131MIPI_write_cmos_sensor(0x4024,0x40); //2 1 ; enabled MIPI -> was 0024

#if for_module
    HM8131MIPI_write_cmos_sensor(0x0101,0x03); //flip+mirror
#else
    HM8131MIPI_write_cmos_sensor(0x0101,0x00); //flip+mirror
#endif


    HM8131MIPI_write_cmos_sensor(0x4800,0x00);
    HM8131MIPI_write_cmos_sensor(0x0104,0x01);
    HM8131MIPI_write_cmos_sensor(0x0104,0x00);
    HM8131MIPI_write_cmos_sensor(0x4801,0x00);
    HM8131MIPI_write_cmos_sensor(0x0000,0x00);

    HM8131MIPI_write_cmos_sensor(0xBA94,0x01);
    HM8131MIPI_write_cmos_sensor(0xBA94,0x00);
    HM8131MIPI_write_cmos_sensor(0xBA93,0x02);


    SENSORDB("[E]");
}


void HM8131MIPI_VideoSizeSetting(void)
{
    //SENSORDB("[HM8131MIPI]enter HM8131MIPI_VideoSizeSetting function\n");
    SENSORDB("[S]");

    //SENSORDB("[HM8131MIPI]exit HM8131MIPI_VideoSizeSetting function\n");
    SENSORDB("[E]");
}



/*************************************************************************
* FUNCTION
*   HM8131MIPIGetSensorID
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
UINT32 HM8131MIPIGetSensorID(UINT32 *sensorID)
{
    int  retry = 3;
    //SENSORDB("[HM8131MIPI]enter HM8131MIPIGetSensorID function\n");
    SENSORDB("[S], *sensorID = 0x%x", *sensorID);
    // check if sensor ID correct
    do
    {
        *sensorID =((HM8131MIPI_read_cmos_sensor(0x0000) << 8) | HM8131MIPI_read_cmos_sensor(0x0001));
        if (*sensorID == HM8131MIPI_SENSOR_ID)
            break;
        retry--;
    }
    while (retry > 0);
    if (*sensorID != HM8131MIPI_SENSOR_ID)
    {
        *sensorID = 0xFFFFFFFF;
        return ERROR_SENSOR_CONNECT_FAIL;
    }
    //SENSORDB("[HM8131MIPI]exit HM8131MIPIGetSensorID function\n");
    SENSORDB("[E], *sensorID = 0x%x", *sensorID);
    return ERROR_NONE;
}

/*****************************************************************************/
/* Windows Mobile Sensor Interface */
/*****************************************************************************/
/*************************************************************************
* FUNCTION
*   HM8131MIPIOpen
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
UINT32 HM8131MIPIOpen(void)
{
    //int  retry = 0;
    kal_uint32 sensorid = 0;
    SENSORDB("[S]");
    if (HM8131MIPIGetSensorID(&sensorid) == ERROR_SENSOR_CONNECT_FAIL)
        return ERROR_SENSOR_CONNECT_FAIL;
    HM8131MIPI_Sensor_Init();
    spin_lock(&HM8131_drv_lock);
    HM8131MIPIAutoFlicKerMode   = KAL_FALSE;
    spin_unlock(&HM8131_drv_lock);
    SENSORDB("[E]");
    return ERROR_NONE;
}

/*Avoid Folat, frame rate =10 * u2FrameRate */
UINT32 HM8131MIPISetMaxFrameRate(UINT16 u2FrameRate)
{
    kal_int16 dummy_line=0;
    kal_uint16 FrameHeight = HM8131MIPI_sensor.frame_height;
    unsigned long flags;
    SENSORDB("[soso][OV5647MIPISetMaxFrameRate]u2FrameRate=%d \n",u2FrameRate);
    FrameHeight= (10 * HM8131MIPI_sensor.pv_pclk) / u2FrameRate / HM8131MIPI_sensor.line_length;
    if(KAL_FALSE == HM8131MIPI_sensor.pv_mode)
    {
        if(KAL_FALSE == HM8131MIPI_sensor.video_mode)
        {
            if(FrameHeight < HM8131MIPI_CAP_FULL_LINES)
                FrameHeight = HM8131MIPI_CAP_FULL_LINES;
        }
        else
        {
            if(FrameHeight < HM8131MIPI_VIDEO_FULL_LINES)
                FrameHeight = HM8131MIPI_VIDEO_FULL_LINES;
        }
    }
    spin_lock_irqsave(&HM8131_drv_lock,flags);
    HM8131MIPI_sensor.frame_height = FrameHeight;
    spin_unlock_irqrestore(&HM8131_drv_lock,flags);
    if((HM8131_CurrentScenarioId == MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG)||(HM8131_CurrentScenarioId == MSDK_SCENARIO_ID_CAMERA_ZSD))
    {
        dummy_line = FrameHeight - HM8131MIPI_CAP_FULL_LINES;
    }
    else if(HM8131_CurrentScenarioId==MSDK_SCENARIO_ID_CAMERA_PREVIEW)
    {
        dummy_line = FrameHeight - HM8131MIPI_PV_FULL_LINES;
    }
    else if(HM8131_CurrentScenarioId==MSDK_SCENARIO_ID_VIDEO_PREVIEW)
    {
        dummy_line = FrameHeight - HM8131MIPI_VIDEO_FULL_LINES;
    }
    SENSORDB("[brad:][HM8131MIPISetMaxFrameRate]frameheight = %d, dummy_line=%d \n",HM8131MIPI_sensor.frame_height,dummy_line);
    if(dummy_line<0)
    {
        dummy_line = 0;
    }
    /* to fix VSYNC, to fix frame rate */
    HM8131MIPI_SetDummy(0, dummy_line); /* modify dummy_pixel must gen AE table again */
    return ERROR_NONE;
}

/*************************************************************************
* FUNCTION
*   HM8131MIPI_SetShutter
*
* DESCRIPTION
*   This function set e-shutter of HM8131MIPI to change exposure time.
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
void HM8131MIPI_SetShutter(kal_uint16 iShutter)
{
    //SENSORDB("[HM8131MIPI]enter HM8131MIPI_SetShutter function\n");
    //SENSORDB("[HM8131MIPI]%s():shutter=%d\n",__FUNCTION__,iShutter);
    SENSORDB("[S], iShutter = 0x%x", iShutter);
    if (iShutter < 1)
        iShutter = 1;
    else if(iShutter > 0xffff)
        iShutter = 0xffff;
    unsigned long flags;
    spin_lock_irqsave(&HM8131_drv_lock,flags);
    HM8131MIPI_sensor.pv_shutter = iShutter;
    spin_unlock_irqrestore(&HM8131_drv_lock,flags);
    HM8131MIPI_write_shutter(iShutter);
    //SENSORDB("[HM8131MIPI]exit HM8131MIPI_SetShutter function\n");
    SENSORDB("[E]");
}   /*  HM8131MIPI_SetShutter   */

/*************************************************************************
* FUNCTION
*   HM8131MIPI_read_shutter
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
UINT16 HM8131MIPI_read_shutter(void)
{
    return (UINT16)( (HM8131MIPI_read_cmos_sensor(0x0202)<<8) | HM8131MIPI_read_cmos_sensor(0x0203) );
}

/*************************************************************************
* FUNCTION
*   HM8131MIPI_night_mode
*
* DESCRIPTION
*   This function night mode of HM8131MIPI.
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
void HM8131MIPI_NightMode(kal_bool bEnable)
{
    SENSORDB("[HM8131MIPI]enter HM8131MIPI_NightMode function\n");
    SENSORDB("[HM8131MIPI]exit HM8131MIPI_NightMode function\n");
}/*	HM8131MIPI_NightMode */

/*************************************************************************
* FUNCTION
*   HM8131MIPIClose
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
UINT32 HM8131MIPIClose(void)
{
    SENSORDB("[S]");
    SENSORDB("[E]");
    return ERROR_NONE;
}   /* HM8131MIPIClose() */

void HM8131MIPISetFlipMirror(kal_int32 imgMirror)
{
    kal_uint8  iTemp;
    SENSORDB("[HM8131MIPI]enter HM8131MIPISetFlipMirror function\n");
    //iTemp = HM8131MIPI_read_cmos_sensor(0x0101) & 0x03; //Clear the mirror and flip bits.
    switch (imgMirror)
    {
    case IMAGE_NORMAL:
        //HM8131MIPI_write_cmos_sensor(0x0101, 0x03); //Set normal
        break;
    case IMAGE_V_MIRROR:
        //HM8131MIPI_write_cmos_sensor(0x0101, iTemp | 0x01); //Set flip
        break;
    case IMAGE_H_MIRROR:
        //HM8131MIPI_write_cmos_sensor(0x0101, iTemp | 0x02); //Set mirror
        break;
    case IMAGE_HV_MIRROR:
        //HM8131MIPI_write_cmos_sensor(0x0101, 0x00); //Set mirror and flip
        break;
    }
    SENSORDB("[HM8131MIPI]exit HM8131MIPISetFlipMirror function\n");
}

/*************************************************************************
* FUNCTION
*   HM8131MIPIPreview
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
UINT32 HM8131MIPIPreview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                         MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    //SENSORDB("[HM8131MIPI]enter HM8131MIPIPreview function\n");
    SENSORDB("[S]");


    spin_lock(&HM8131_drv_lock);
    //HM8131MIPI_MPEG4_encode_mode = KAL_FALSE;
    HM8131MIPI_sensor.video_mode = KAL_FALSE;
    HM8131MIPI_sensor.pv_mode = KAL_TRUE;
    HM8131MIPI_sensor.capture_mode = KAL_FALSE;
    spin_unlock(&HM8131_drv_lock);


    HM8131MIPI_write_cmos_sensor(0x0100,0x00);
    mdelay(72);

//   SENSORDB("PREVIEW START: %d \n",cpCounter);


    HM8131MIPI_write_cmos_sensor(0x0103,0x00);
    HM8131MIPI_write_cmos_sensor(0x0100,0x02);
    //  mdelay(10);
    previewSetting();
    HM8131MIPI_write_cmos_sensor(0x0100,0x01);

//   SENSORDB("PREVIEW FINISHED: %d \n",cpCounter);

     status[st_i]='P'; st_i++;
     SENSORDB("STATUS:: %s \n",status);
    spin_lock(&HM8131_drv_lock);
    HM8131MIPI_sensor.cp_dummy_pixels = 0;
    HM8131MIPI_sensor.cp_dummy_lines = 0;
    HM8131MIPI_sensor.pv_dummy_pixels = 0;
    HM8131MIPI_sensor.pv_dummy_lines = 0;
    HM8131MIPI_sensor.video_dummy_pixels = 0;
    HM8131MIPI_sensor.video_dummy_lines = 0;
    HM8131MIPI_sensor.pv_line_length  = HM8131MIPI_PV_FULL_PIXELS+HM8131MIPI_sensor.pv_dummy_pixels;
    HM8131MIPI_sensor.pv_frame_length = HM8131MIPI_PV_FULL_LINES+HM8131MIPI_sensor.pv_dummy_lines;
    HM8131MIPI_sensor.frame_height    = HM8131MIPI_sensor.pv_frame_length;
    HM8131MIPI_sensor.line_length     = HM8131MIPI_sensor.pv_line_length;
    spin_unlock(&HM8131_drv_lock);

    HM8131MIPI_SetDummy(HM8131MIPI_sensor.pv_dummy_pixels,HM8131MIPI_sensor.pv_dummy_lines);
    HM8131MIPI_SetShutter(HM8131MIPI_sensor.pv_shutter);
    spin_lock(&HM8131_drv_lock);

    memcpy(&HM8131MIPISensorConfigData, sensor_config_data, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
    spin_unlock(&HM8131_drv_lock);

    //SENSORDB("[HM8131MIPI]exit HM8131MIPIPreview function\n");
    SENSORDB("[S]");
    return ERROR_NONE;
}   /* HM8131MIPIPreview() */

/*************************************************************************
* FUNCTION
* RETURNS
*   None
*
* GLOBALS AFFECTED
*
*************************************************************************/
UINT32 HM8131MIPIVideo(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                       MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    kal_uint16 iStartX = 0, iStartY = 0;
    SENSORDB("[HM8131MIPI]enter HM8131MIPIVideo function\n");
    spin_lock(&HM8131_drv_lock);
    //HM8131MIPI_MPEG4_encode_mode = KAL_TRUE;
    HM8131MIPI_sensor.video_mode=KAL_TRUE;
    HM8131MIPI_sensor.pv_mode=KAL_FALSE;
    HM8131MIPI_sensor.capture_mode=KAL_FALSE;
    spin_unlock(&HM8131_drv_lock);


//    HM8131MIPI_VideoSizeSetting();



    HM8131MIPI_write_cmos_sensor(0x0100,0x00);
    mdelay(72);
    HM8131MIPI_write_cmos_sensor(0x0103,0x00);
    HM8131MIPI_write_cmos_sensor(0x0100,0x02);
    //  mdelay(10);
    previewSetting();
    HM8131MIPI_write_cmos_sensor(0x0100,0x01);






    spin_lock(&HM8131_drv_lock);
    HM8131MIPI_sensor.cp_dummy_pixels = 0;
    HM8131MIPI_sensor.cp_dummy_lines = 0;
    HM8131MIPI_sensor.pv_dummy_pixels = 0;
    HM8131MIPI_sensor.pv_dummy_lines = 0;
    HM8131MIPI_sensor.video_dummy_pixels = 0;
    HM8131MIPI_sensor.video_dummy_lines = 0;
    HM8131MIPI_sensor.video_line_length  = HM8131MIPI_VIDEO_FULL_PIXELS+HM8131MIPI_sensor.video_dummy_pixels;
    HM8131MIPI_sensor.video_frame_length = HM8131MIPI_VIDEO_FULL_LINES+HM8131MIPI_sensor.video_dummy_lines;
    HM8131MIPI_sensor.frame_height       = HM8131MIPI_sensor.video_frame_length;
    HM8131MIPI_sensor.line_length        = HM8131MIPI_sensor.video_line_length;
    


    spin_unlock(&HM8131_drv_lock);

    HM8131MIPI_SetDummy(HM8131MIPI_sensor.video_dummy_pixels,HM8131MIPI_sensor.video_dummy_lines);
    HM8131MIPI_SetShutter(HM8131MIPI_sensor.video_shutter);
    spin_lock(&HM8131_drv_lock);
    memcpy(&HM8131MIPISensorConfigData, sensor_config_data, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
    spin_unlock(&HM8131_drv_lock);

    SENSORDB("[HM8131MIPI]exit HM8131MIPIVideo function\n");
    return ERROR_NONE;
}   /* HM8131MIPIPreview() */

UINT32 HM8131MIPICapture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                         MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    //SENSORDB("[HM8131MIPI]enter HM8131MIPICapture function\n");
    SENSORDB("[S]");





    spin_lock(&HM8131_drv_lock);
    HM8131MIPI_sensor.video_mode=KAL_FALSE;
    HM8131MIPI_sensor.pv_mode=KAL_FALSE;
    HM8131MIPI_sensor.capture_mode=KAL_TRUE;
    HM8131MIPIAutoFlicKerMode = KAL_FALSE;
    HM8131MIPI_sensor.cp_dummy_pixels = 0;
    HM8131MIPI_sensor.cp_dummy_lines = 0;
    spin_unlock(&HM8131_drv_lock);




    HM8131MIPI_write_cmos_sensor(0x0100,0x00);
    mdelay(38);// very important!! Without this, it fails!

//   SENSORDB("CAPTURE START: %d \n",cpCounter);

    HM8131MIPI_write_cmos_sensor(0x0103,0x00);

    HM8131MIPI_write_cmos_sensor(0x0100,0x02);
//mdelay(10);
    HM8131MIPI_Sensor_8M_nonReset();

    HM8131MIPI_write_cmos_sensor(0x0100,0x01);


        status[st_i]='C'; st_i++;
        SENSORDB("STATUS:: %s \n",status);

        if(st_i==200)
        {st_i=0;  status[st_i]='\0';}

    spin_lock(&HM8131_drv_lock);
    HM8131MIPI_sensor.cp_line_length  = HM8131MIPI_CAP_FULL_PIXELS+HM8131MIPI_sensor.cp_dummy_pixels;
    HM8131MIPI_sensor.cp_frame_length = HM8131MIPI_CAP_FULL_LINES+HM8131MIPI_sensor.cp_dummy_lines;
    HM8131MIPI_sensor.frame_height    = HM8131MIPI_sensor.cp_frame_length;
    HM8131MIPI_sensor.line_length     = HM8131MIPI_sensor.cp_line_length;
    spin_unlock(&HM8131_drv_lock);

    HM8131MIPI_SetDummy(HM8131MIPI_sensor.cp_dummy_pixels, HM8131MIPI_sensor.cp_dummy_lines);
    spin_lock(&HM8131_drv_lock);
    memcpy(&HM8131MIPISensorConfigData, sensor_config_data, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
    spin_unlock(&HM8131_drv_lock);

    //SENSORDB("[HM8131MIPI]exit HM8131MIPICapture function\n");
    SENSORDB("[E]");
    return ERROR_NONE;
}   /* HM8131MIPICapture() */

UINT32 HM8131MIPIGetResolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *pSensorResolution)
{

    //SENSORDB("[HM8131MIPI]enter HM8131MIPIGetResolution function\n");
    SENSORDB("[S]");
    pSensorResolution->SensorPreviewWidth   = HM8131MIPI_REAL_PV_WIDTH;
    pSensorResolution->SensorPreviewHeight  = HM8131MIPI_REAL_PV_HEIGHT;
    pSensorResolution->SensorFullWidth      = HM8131MIPI_REAL_CAP_WIDTH;
    pSensorResolution->SensorFullHeight     = HM8131MIPI_REAL_CAP_HEIGHT;
    pSensorResolution->SensorVideoWidth     = HM8131MIPI_REAL_VIDEO_WIDTH;
    pSensorResolution->SensorVideoHeight    = HM8131MIPI_REAL_VIDEO_HEIGHT;
    //SENSORDB("[HM8131MIPI]exit HM8131MIPIGetResolution function\n");
    SENSORDB("[E]");
    return ERROR_NONE;
}/* HM8131MIPIGetResolution() */

UINT32 HM8131MIPIGetInfo(MSDK_SCENARIO_ID_ENUM ScenarioId,
                         MSDK_SENSOR_INFO_STRUCT *pSensorInfo,
                         MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
    //SENSORDB("[HM8131MIPI]enter HM8131MIPIGetInfo function\n");
    SENSORDB("[S], ScenarioId = 0x%x", ScenarioId);
    switch(ScenarioId)
    {
    case MSDK_SCENARIO_ID_CAMERA_ZSD:
    case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
        pSensorInfo->SensorFullResolutionX=HM8131MIPI_REAL_CAP_WIDTH;
        pSensorInfo->SensorFullResolutionY=HM8131MIPI_REAL_CAP_HEIGHT;
        pSensorInfo->SensorStillCaptureFrameRate=30;
        break;
    case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
        pSensorInfo->SensorPreviewResolutionX=HM8131MIPI_REAL_VIDEO_WIDTH;
        pSensorInfo->SensorPreviewResolutionY=HM8131MIPI_REAL_VIDEO_HEIGHT;
        pSensorInfo->SensorCameraPreviewFrameRate=15;
        break;
    default:
        pSensorInfo->SensorPreviewResolutionX=HM8131MIPI_REAL_PV_WIDTH;
        pSensorInfo->SensorPreviewResolutionY=HM8131MIPI_REAL_PV_HEIGHT;
        pSensorInfo->SensorCameraPreviewFrameRate=15;
        break;
    }
    pSensorInfo->SensorVideoFrameRate=30;
    pSensorInfo->SensorStillCaptureFrameRate=15;
    pSensorInfo->SensorWebCamCaptureFrameRate=30;
    pSensorInfo->SensorClockPolarity=SENSOR_CLOCK_POLARITY_LOW;
    pSensorInfo->SensorClockFallingPolarity=SENSOR_CLOCK_POLARITY_LOW;
    pSensorInfo->SensorHsyncPolarity=SENSOR_CLOCK_POLARITY_LOW;
    pSensorInfo->SensorVsyncPolarity=SENSOR_CLOCK_POLARITY_HIGH;//HIGH
    pSensorInfo->SensorInterruptDelayLines=1;
    pSensorInfo->SensorResetActiveHigh=FALSE;
    pSensorInfo->SensorResetDelayCount=5;
    pSensorInfo->SensroInterfaceType=SENSOR_INTERFACE_TYPE_MIPI;
    //pSensorInfo->SensorOutputDataFormat=SENSOR_OUTPUT_FORMAT_RAW_B;
#if for_module
    pSensorInfo->SensorOutputDataFormat=SENSOR_OUTPUT_FORMAT_RAW_Gb;//for module
#else
    pSensorInfo->SensorOutputDataFormat=SENSOR_OUTPUT_FORMAT_RAW_R;
#endif
    pSensorInfo->CaptureDelayFrame=1; //1
    pSensorInfo->PreviewDelayFrame=2; //0
    pSensorInfo->VideoDelayFrame=2;//2
    pSensorInfo->SensorDrivingCurrent=ISP_DRIVING_2MA;
    //pSensorInfo->SensorDrivingCurrent=ISP_DRIVING_8MA;
    pSensorInfo->SensorMasterClockSwitch=0;
    pSensorInfo->AEShutDelayFrame=0;//0/* The frame of setting shutter default 0 for TG int */
    pSensorInfo->AESensorGainDelayFrame=0;//1;/* The frame of setting sensor gain */
    pSensorInfo->AEISPGainDelayFrame=2;//2;//2

    switch (ScenarioId)
    {
    case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
        pSensorInfo->SensorClockFreq=24;
        pSensorInfo->SensorClockDividCount=5;
        pSensorInfo->SensorClockRisingCount= 0;
        pSensorInfo->SensorClockFallingCount= 2;
        pSensorInfo->SensorPixelClockCount= 3;
        pSensorInfo->SensorDataLatchCount= 2;
        pSensorInfo->SensorGrabStartX = HM8131MIPI_IMAGE_SENSOR_PV_STARTX;
        pSensorInfo->SensorGrabStartY = HM8131MIPI_IMAGE_SENSOR_PV_STARTY;
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
        pSensorInfo->SensorGrabStartX = HM8131MIPI_IMAGE_SENSOR_VIDEO_STARTX;
        pSensorInfo->SensorGrabStartY = HM8131MIPI_IMAGE_SENSOR_VIDEO_STARTY;
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
        pSensorInfo->SensorGrabStartX = HM8131MIPI_IMAGE_SENSOR_CAP_STARTX; // 2*HM8131MIPI_IMAGE_SENSOR_PV_STARTX;
        pSensorInfo->SensorGrabStartY = HM8131MIPI_IMAGE_SENSOR_CAP_STARTY; // 2*HM8131MIPI_IMAGE_SENSOR_PV_STARTY;
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
        pSensorInfo->SensorGrabStartX = HM8131MIPI_IMAGE_SENSOR_PV_STARTX;
        pSensorInfo->SensorGrabStartY = HM8131MIPI_IMAGE_SENSOR_PV_STARTY;
        pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_2_LANE;
        pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
        pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 14;
        pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
        pSensorInfo->SensorWidthSampling = 0; // 0 is default 1x
        pSensorInfo->SensorHightSampling = 0; // 0 is default 1x
        pSensorInfo->SensorPacketECCOrder = 1;
        break;
    }
    spin_lock(&HM8131_drv_lock);
    //HM8131MIPIPixelClockDivider=pSensorInfo->SensorPixelClockCount;
    memcpy(pSensorConfigData, &HM8131MIPISensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
    spin_unlock(&HM8131_drv_lock);
    //SENSORDB("[HM8131MIPI]exit HM8131MIPIGetInfo function\n");
    SENSORDB("[E]");
    return ERROR_NONE;
}   /* HM8131MIPIGetInfo() */

UINT32 HM8131MIPIControl(MSDK_SCENARIO_ID_ENUM ScenarioId,
                         MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *pImageWindow,
                         MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
    spin_lock(&HM8131_drv_lock);
    HM8131_CurrentScenarioId = ScenarioId;
    spin_unlock(&HM8131_drv_lock);
    //SENSORDB("[HM8131MIPI]enter HM8131MIPIControl function\n");
    SENSORDB("[S]");
    switch (ScenarioId)
    {
    case MSDK_SCENARIO_ID_CAMERA_PREVIEW:

        SENSORDB("PREVIEW START: %d \n",cpCounter);
        HM8131MIPIPreview(pImageWindow, pSensorConfigData);
        SENSORDB("PREVIEW FINISHED: %d \n",cpCounter);

        break;
    case MSDK_SCENARIO_ID_VIDEO_PREVIEW:


        HM8131MIPIVideo(pImageWindow, pSensorConfigData);
        break;
    case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
    case MSDK_SCENARIO_ID_CAMERA_ZSD:

        SENSORDB("CAPTURE START: %d \n",cpCounter);
        HM8131MIPICapture(pImageWindow, pSensorConfigData);
        SENSORDB("CAPTURE FINISHED: %d \n",cpCounter);
        cpCounter++;

        break;
    default:
        return ERROR_INVALID_SCENARIO_ID;
    }
    //SENSORDB("[HM8131MIPI]exit HM8131MIPIControl function\n");
    SENSORDB("[E]");
    return ERROR_NONE;
}   /* HM8131MIPIControl() */

UINT32 HM8131MIPISetVideoMode(UINT16 u2FrameRate)
{
    SENSORDB("[HM8131MIPISetVideoMode] frame rate = %d\n", u2FrameRate);
    kal_uint16 HM8131MIPI_Video_Max_Expourse_Time = 0;
    SENSORDB("[HM8131MIPI]%s():fix_frame_rate=%d\n",__FUNCTION__,u2FrameRate);
    spin_lock(&HM8131_drv_lock);
    HM8131MIPI_sensor.fix_video_fps = KAL_TRUE;
    spin_unlock(&HM8131_drv_lock);
    u2FrameRate=u2FrameRate*10;//10*FPS
    if(u2FrameRate==300)
        u2FrameRate=296;
    SENSORDB("[HM8131MIPI][Enter Fix_fps func] HM8131MIPI_Fix_Video_Frame_Rate = %d\n", u2FrameRate/10);
    HM8131MIPI_Video_Max_Expourse_Time = (kal_uint16)((HM8131MIPI_sensor.video_pclk*10/u2FrameRate)/HM8131MIPI_sensor.video_line_length);
    if (HM8131MIPI_Video_Max_Expourse_Time > HM8131MIPI_VIDEO_FULL_LINES/*HM8131MIPI_sensor.pv_frame_length*/)
    {
        spin_lock(&HM8131_drv_lock);
        HM8131MIPI_sensor.video_frame_length = HM8131MIPI_Video_Max_Expourse_Time;
        HM8131MIPI_sensor.video_dummy_lines = HM8131MIPI_sensor.video_frame_length-HM8131MIPI_VIDEO_FULL_LINES;
        spin_unlock(&HM8131_drv_lock);
        SENSORDB("[HM8131MIPI]%s():frame_length=%d,dummy_lines=%d\n",__FUNCTION__,HM8131MIPI_sensor.video_frame_length,HM8131MIPI_sensor.video_dummy_lines);
        HM8131MIPI_SetDummy(HM8131MIPI_sensor.video_dummy_pixels,HM8131MIPI_sensor.video_dummy_lines);
    }
    spin_lock(&HM8131_drv_lock);
    //HM8131MIPI_MPEG4_encode_mode = KAL_TRUE;
    HM8131MIPI_sensor.FixedFps   = u2FrameRate;
    spin_unlock(&HM8131_drv_lock);
    SENSORDB("[HM8131MIPI]exit HM8131MIPISetVideoMode function\n");
    return ERROR_NONE;
}

UINT32 HM8131MIPISetAutoFlickerMode(kal_bool bEnable, UINT16 u2FrameRate)
{
    SENSORDB("OV5647MIPISetAutoFlickerMode:%d",bEnable);
    if(bEnable)
    {
        spin_lock(&HM8131_drv_lock);
        HM8131MIPIAutoFlicKerMode = KAL_TRUE;
        spin_unlock(&HM8131_drv_lock);
        /*Change frame rate 29.5fps to 29.8fps to do Auto flick*/
        if((HM8131MIPI_sensor.FixedFps == 30)&&(HM8131MIPI_sensor.video_mode==KAL_TRUE))
            HM8131MIPISetMaxFrameRate(296);
    }
    else
    {
        //Cancel Auto flick
        spin_lock(&HM8131_drv_lock);
        HM8131MIPIAutoFlicKerMode = KAL_FALSE;
        spin_unlock(&HM8131_drv_lock);
        if((HM8131MIPI_sensor.FixedFps == 30)&&(HM8131MIPI_sensor.video_mode==KAL_TRUE))
            HM8131MIPISetMaxFrameRate(300);
    }
    return ERROR_NONE;
}

UINT32 HM8131MIPISetMaxFramerateByScenario(MSDK_SCENARIO_ID_ENUM scenarioId, MUINT32 frameRate)// this function seems never invoked
{
    kal_uint32 pclk;
    kal_int16 dummyLine;
    kal_uint16 lineLength,frameHeight;
    SENSORDB("HM8131MIPISetMaxFramerateByScenario: scenarioId = %d, frame rate = %d\n",scenarioId,frameRate);
    switch (scenarioId)
    {
    case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
        pclk = 132000000;//120000000;
        lineLength = HM8131MIPI_PV_DUMMY_PIXELS;
        frameHeight = (10 * pclk)/frameRate/lineLength;
        dummyLine = frameHeight - HM8131MIPI_PV_DUMMY_LINES;
        if(dummyLine<0)
            dummyLine = 0;
        spin_lock(&HM8131_drv_lock);
        HM8131MIPI_sensor.pv_mode=TRUE;
        spin_unlock(&HM8131_drv_lock);
        HM8131MIPI_SetDummy(0, dummyLine);
        break;
    case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
        pclk = 132000000;//120000000;
        lineLength = HM8131MIPI_VIDEO_DUMMY_PIXELS;
        frameHeight = (10 * pclk)/frameRate/lineLength;
        dummyLine = frameHeight - HM8131MIPI_VIDEO_DUMMY_LINES;
        if(dummyLine<0)
            dummyLine = 0;
        spin_lock(&HM8131_drv_lock);
        HM8131MIPI_sensor.pv_mode=TRUE;
        spin_unlock(&HM8131_drv_lock);
        HM8131MIPI_SetDummy(0, dummyLine);
        break;
    case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
    case MSDK_SCENARIO_ID_CAMERA_ZSD:
        pclk = 132000000;//120000000;
        lineLength = HM8131MIPI_FULL_DUMMY_PIXELS;
        frameHeight = (10 * pclk)/frameRate/lineLength;
        dummyLine = frameHeight - HM8131MIPI_FULL_DUMMY_LINES;
        if(dummyLine<0)
            dummyLine = 0;
        spin_lock(&HM8131_drv_lock);
        HM8131MIPI_sensor.pv_mode=FALSE;
        spin_unlock(&HM8131_drv_lock);
        HM8131MIPI_SetDummy(0, dummyLine);
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
    SENSORDB("[HM8131MIPI]exit HM8131MIPISetMaxFramerateByScenario function\n");
    return ERROR_NONE;
}

UINT32 HM8131MIPIGetDefaultFramerateByScenario(MSDK_SCENARIO_ID_ENUM scenarioId, MUINT32 *pframeRate)
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

UINT32 HM8131MIPISetTestPatternMode(kal_bool bEnable)
{
    SENSORDB("[HM8131MIPISetTestPatternMode] Test pattern enable:%d\n", bEnable);
    if(bEnable)   // enable color bar
    {
        //HM8131MIPI_write_cmos_sensor(0x30D8, 0x10); // color bar test pattern
        //HM8131MIPI_write_cmos_sensor(0x0600, 0x00); // color bar test pattern
        //HM8131MIPI_write_cmos_sensor(0x0601, 0x02); // color bar test pattern
    }
    else
    {
        // HM8131MIPI_write_cmos_sensor(0x30D8, 0x00); // disable color bar test pattern
    }
    return ERROR_NONE;
}

UINT32 HM8131MIPIFeatureControl(MSDK_SENSOR_FEATURE_ENUM FeatureId,
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
        *pFeatureReturnPara16++=HM8131MIPI_REAL_CAP_WIDTH;
        *pFeatureReturnPara16=HM8131MIPI_REAL_CAP_HEIGHT;
        *pFeatureParaLen=4;
        break;
    case SENSOR_FEATURE_GET_PERIOD:             //3002
        switch(HM8131_CurrentScenarioId)
        {
        case MSDK_SCENARIO_ID_CAMERA_ZSD:
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
            *pFeatureReturnPara16++=HM8131MIPI_sensor.cp_line_length;
            *pFeatureReturnPara16=HM8131MIPI_sensor.cp_frame_length;
            SENSORDB("Sensor cap period:%d %d", HM8131MIPI_sensor.cp_line_length, HM8131MIPI_sensor.cp_frame_length);
            *pFeatureParaLen=4;
            break;
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
            *pFeatureReturnPara16++=HM8131MIPI_sensor.video_line_length;
            *pFeatureReturnPara16=HM8131MIPI_sensor.video_frame_length;
            SENSORDB("Sensor video period:%d %d", HM8131MIPI_sensor.video_line_length, HM8131MIPI_sensor.video_frame_length);
            *pFeatureParaLen=4;
            break;
        default:
            *pFeatureReturnPara16++=HM8131MIPI_sensor.pv_line_length;
            *pFeatureReturnPara16=HM8131MIPI_sensor.pv_frame_length;
            SENSORDB("Sensor pv period:%d %d", HM8131MIPI_sensor.pv_line_length, HM8131MIPI_sensor.pv_frame_length);
            *pFeatureParaLen=4;
            break;
        }
        break;
    case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:   //3003
        switch(HM8131_CurrentScenarioId)
        {
        case MSDK_SCENARIO_ID_CAMERA_ZSD:
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
            *pFeatureReturnPara32 = HM8131MIPI_sensor.cp_pclk;
            *pFeatureParaLen=4;
            SENSORDB("Sensor CPCLK:%d",HM8131MIPI_sensor.cp_pclk);
            break;
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
            *pFeatureReturnPara32 = HM8131MIPI_sensor.video_pclk;
            *pFeatureParaLen=4;
            SENSORDB("Sensor videoCLK:%d",HM8131MIPI_sensor.video_pclk);
            break;
        default:
            *pFeatureReturnPara32 = HM8131MIPI_sensor.pv_pclk;
            *pFeatureParaLen=4;
            SENSORDB("Sensor pvclk:%d",HM8131MIPI_sensor.pv_pclk);
            break;
        }

        break;
    case SENSOR_FEATURE_SET_ESHUTTER:           //3004
        HM8131MIPI_SetShutter(*pFeatureData16);
        break;
    case SENSOR_FEATURE_SET_NIGHTMODE:          //3005
        HM8131MIPI_NightMode((BOOL) *pFeatureData16);
        break;
    case SENSOR_FEATURE_SET_GAIN:               //3006
        HM8131MIPI_SetGain((UINT16) *pFeatureData16);
        break;

    case SENSOR_FEATURE_SET_FLASHLIGHT:         //3007
        break;
    case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:  //3008
        spin_lock(&HM8131_drv_lock);
        HM8131MIPI_isp_master_clock=*pFeatureData32;
        spin_unlock(&HM8131_drv_lock);
        break;
    case SENSOR_FEATURE_SET_REGISTER:           //3009
        HM8131MIPI_write_cmos_sensor(pSensorRegData->RegAddr, pSensorRegData->RegData);
        break;
    case SENSOR_FEATURE_GET_REGISTER:           //3010
        pSensorRegData->RegData = HM8131MIPI_read_cmos_sensor(pSensorRegData->RegAddr);
        break;
    case SENSOR_FEATURE_SET_CCT_REGISTER:       //3011
        SensorRegNumber=FACTORY_END_ADDR;
        for (i=0; i<SensorRegNumber; i++)
        {
            spin_lock(&HM8131_drv_lock);
            HM8131MIPISensorCCT[i].Addr=*pFeatureData32++;
            HM8131MIPISensorCCT[i].Para=*pFeatureData32++;
            spin_unlock(&HM8131_drv_lock);
        }
        break;
    case SENSOR_FEATURE_GET_CCT_REGISTER:       //3012
        SensorRegNumber=FACTORY_END_ADDR;
        if (*pFeatureParaLen<(SensorRegNumber*sizeof(SENSOR_REG_STRUCT)+4))
            return FALSE;
        *pFeatureData32++=SensorRegNumber;
        for (i=0; i<SensorRegNumber; i++)
        {
            *pFeatureData32++=HM8131MIPISensorCCT[i].Addr;
            *pFeatureData32++=HM8131MIPISensorCCT[i].Para;
        }
        break;
    case SENSOR_FEATURE_SET_ENG_REGISTER:       //3013
        SensorRegNumber=ENGINEER_END;
        for (i=0; i<SensorRegNumber; i++)
        {
            spin_lock(&HM8131_drv_lock);
            HM8131MIPISensorReg[i].Addr=*pFeatureData32++;
            HM8131MIPISensorReg[i].Para=*pFeatureData32++;
            spin_unlock(&HM8131_drv_lock);
        }
        break;
    case SENSOR_FEATURE_GET_ENG_REGISTER:       //3014
        SensorRegNumber=ENGINEER_END;
        if (*pFeatureParaLen<(SensorRegNumber*sizeof(SENSOR_REG_STRUCT)+4))
            return FALSE;
        *pFeatureData32++=SensorRegNumber;
        for (i=0; i<SensorRegNumber; i++)
        {
            *pFeatureData32++=HM8131MIPISensorReg[i].Addr;
            *pFeatureData32++=HM8131MIPISensorReg[i].Para;
        }
        break;
    case SENSOR_FEATURE_GET_REGISTER_DEFAULT:   //3015
        if (*pFeatureParaLen>=sizeof(NVRAM_SENSOR_DATA_STRUCT))
        {
            pSensorDefaultData->Version=NVRAM_CAMERA_SENSOR_FILE_VERSION;
            pSensorDefaultData->SensorId=HM8131MIPI_SENSOR_ID;
            memcpy(pSensorDefaultData->SensorEngReg, HM8131MIPISensorReg, sizeof(SENSOR_REG_STRUCT)*ENGINEER_END);
            memcpy(pSensorDefaultData->SensorCCTReg, HM8131MIPISensorCCT, sizeof(SENSOR_REG_STRUCT)*FACTORY_END_ADDR);
        }
        else
            return FALSE;
        *pFeatureParaLen=sizeof(NVRAM_SENSOR_DATA_STRUCT);
        break;
    case SENSOR_FEATURE_GET_CONFIG_PARA:        //3016
        memcpy(pSensorConfigData, &HM8131MIPISensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
        *pFeatureParaLen=sizeof(MSDK_SENSOR_CONFIG_STRUCT);
        break;
    case SENSOR_FEATURE_CAMERA_PARA_TO_SENSOR:  //3017
        HM8131MIPI_camera_para_to_sensor();
        break;
    case SENSOR_FEATURE_SENSOR_TO_CAMERA_PARA:  //3018
        HM8131MIPI_sensor_to_camera_para();
        break;
    case SENSOR_FEATURE_GET_GROUP_COUNT:        //3019
        *pFeatureReturnPara32++=HM8131MIPI_get_sensor_group_count();
        *pFeatureParaLen=4;
        break;
    case SENSOR_FEATURE_GET_GROUP_INFO:         //3020
        HM8131MIPI_get_sensor_group_info(pSensorGroupInfo->GroupIdx, pSensorGroupInfo->GroupNamePtr, &pSensorGroupInfo->ItemCount);
        *pFeatureParaLen=sizeof(MSDK_SENSOR_GROUP_INFO_STRUCT);
        break;
    case SENSOR_FEATURE_GET_ITEM_INFO:          //3021
        HM8131MIPI_get_sensor_item_info(pSensorItemInfo->GroupIdx,pSensorItemInfo->ItemIdx, pSensorItemInfo);
        *pFeatureParaLen=sizeof(MSDK_SENSOR_ITEM_INFO_STRUCT);
        break;
    case SENSOR_FEATURE_SET_ITEM_INFO:          //3022
        HM8131MIPI_set_sensor_item_info(pSensorItemInfo->GroupIdx, pSensorItemInfo->ItemIdx, pSensorItemInfo->ItemValue);
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
        HM8131MIPISetVideoMode(*pFeatureData16);
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
        HM8131MIPIGetSensorID(pFeatureReturnPara32);
        break;
    case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE:  //3036
        HM8131MIPISetAutoFlickerMode((BOOL)*pFeatureData16, *(pFeatureData16+1));
        break;
    case SENSOR_FEATURE_SET_TEST_PATTERN:       //3037
        HM8131MIPISetTestPatternMode((BOOL)*pFeatureData16);
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
        HM8131MIPISetMaxFramerateByScenario((MSDK_SCENARIO_ID_ENUM)*pFeatureData32, *(pFeatureData32+1));
        break;
    case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO: //3053
        HM8131MIPIGetDefaultFramerateByScenario((MSDK_SCENARIO_ID_ENUM)*pFeatureData32, (MUINT32 *)(*(pFeatureData32+1)));
        break;
    case SENSOR_FEATURE_GET_AE_AWB_LOCK_INFO:   //3054
        break;
    default:
        SENSORDB("No %d Command", FeatureId);
        break;
    }
    SENSORDB("[E]");
    return ERROR_NONE;
}   /* HM8131MIPIFeatureControl() */

SENSOR_FUNCTION_STRUCT SensorFuncHM8131MIPI=
{
    HM8131MIPIOpen,
    HM8131MIPIGetInfo,


    HM8131MIPIGetResolution,
    HM8131MIPIFeatureControl,
    HM8131MIPIControl,
    HM8131MIPIClose
};

UINT32 HM8131MIPI_RAW_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
    /* To Do : Check Sensor status here */
    if (pfFunc!=NULL)
        *pfFunc=&SensorFuncHM8131MIPI;
    return ERROR_NONE;
}   /* SensorInit() */
