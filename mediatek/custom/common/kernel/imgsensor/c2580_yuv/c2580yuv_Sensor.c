/*****************************************************************************
*  Copyright Statement:
*  --------------------
*  This software is protected by Copyright and the information contained
*  herein is confidential. The software may not be copied and the information
*  contained herein may not be used or disclosed except with the written
*  permission of MediaTek Inc. (C) 2010
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
 *
 * Filename:
 * ---------
 *   sensor.c
 *
 * Project:
 * --------
 *  
 *
 * Description:
 * ------------
 *   Source code of Sensor driver
 *
 *
 *
 *============================================================================
 *             HISTORY
 * Below this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *------------------------------------------------------------------------------
 * $Revision:$
 * $Modtime:$
 * $Log:$
 *
 * [C2580YUV V1.0.0]
 * .First Release
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by GalaxyCoreinc. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/
//#include <windows.h>
//#include <memory.h>
//#include <nkintr.h>
//#include <ceddk.h>
//#include <ceddk_exp.h>

//#include "kal_release.h"
//#include "i2c_exp.h"
//#include "gpio_exp.h"
//#include "msdk_exp.h"
//#include "msdk_sensor_exp.h"
//#include "msdk_isp_exp.h"
//#include "base_regs.h"
//#include "Sensor.h"
//#include "camera_sensor_para.h"
//#include "CameraCustomized.h"

#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
//#include <mach/mt6516_pll.h>

#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"
#include "kd_camera_feature.h"

#include "c2580yuv_Sensor.h"
#include "c2580yuv_Camera_Sensor_para.h"
#include "c2580yuv_CameraCustomized.h"

#define C2580YUV_DEBUG
#ifdef C2580YUV_DEBUG
#define SENSORDB printk
#else
#define SENSORDB(x,...)
#endif
#define C2580_MIPI
extern int iReadReg(u16 a_u2Addr , u8 * a_puBuff , u16 i2cId);
extern int iWriteReg(u16 a_u2Addr , u32 a_u4Data , u32 a_u4Bytes , u16 i2cId);
#define C2580_write_cmos_sensor(addr, para) iWriteReg((u16) addr , (u32) para ,1,C2580_WRITE_ID)
kal_uint16 C2580_read_cmos_sensor(kal_uint32 addr)
{
	kal_uint16 get_byte=0;
	iReadReg((u16) addr ,(u8*)&get_byte,C2580_WRITE_ID);
	return get_byte;
}
/*8 BIT ADDR;8 BIT DATA
extern int iReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 * a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
extern int iWriteRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u16 i2cId);
static void C2580_write_cmos_sensor(kal_uint8 addr, kal_uint8 para)
{
kal_uint8 out_buff[2];

    out_buff[0] = addr;
    out_buff[1] = para;

    iWriteRegI2C((u8*)out_buff , (u16)sizeof(out_buff), C2580_WRITE_ID); 

#if (defined(__C2580_DEBUG_TRACE__))
  if (sizeof(out_buff) != rt) printk("I2C write %x, %x error\n", addr, para);
#endif
}
static kal_uint8 C2580_read_cmos_sensor(kal_uint8 addr)
{
  kal_uint8 in_buff[1] = {0xFF};
  kal_uint8 out_buff[1];
  
  out_buff[0] = addr;

    if (0 != iReadRegI2C((u8*)out_buff , (u16) sizeof(out_buff), (u8*)in_buff, (u16) sizeof(in_buff), C2580_WRITE_ID)) {
        SENSORDB("ERROR: C2580_read_cmos_sensor \n");
    }

#if (defined(__C2580_DEBUG_TRACE__))
  if (size != rt) printk("I2C read %x error\n", addr);
#endif

  return in_buff[0];
}  
*/

/*******************************************************************************
* // Adapter for Winmo typedef 
********************************************************************************/
#define Sleep(ms) mdelay(ms)
#define mDELAY(ms) mdelay(ms)
#define RETAILMSG(x,...)
#define TEXT


/*******************************************************************************
* // End Adapter for Winmo typedef 
********************************************************************************/
/* Global Valuable */
static kal_uint32 zoom_factor = 0; 
static kal_bool C2580_VEDIO_encode_mode = KAL_FALSE; //Picture(Jpeg) or Video(Mpeg4)
static kal_bool C2580_sensor_cap_state = KAL_FALSE; //Preview or Capture
static kal_uint16 C2580_exposure_lines=0, C2580_extra_exposure_lines = 0;
static kal_uint16 C2580_Capture_Shutter=0;
static kal_uint16 C2580_Capture_Extra_Lines=0;
C2580_isp_master_clock=0;
//static kal_uint32  C2580_sensor_pclk=315;
static kal_int8 C2580_DELAY_AFTER_PREVIEW = -1;
static kal_uint32 Preview_Shutter = 0;
static kal_uint32 Capture_Shutter = 0;
static MSDK_SENSOR_CONFIG_STRUCT C2580SensorConfigData;
/*************************************************************************
* FUNCTION
*	C2580_read/write_shutter
*
* DESCRIPTION
*	This function read/write shutte of C2580.
*
* PARAMETERS
*	none
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/

kal_uint16 C2580_read_shutter(void)
{
return  (C2580_read_cmos_sensor(0x0202) << 8)|C2580_read_cmos_sensor(0x0203) ;
} /* C2580 read_shutter */


static void C2580_write_shutter(kal_uint32 shutter)
{
 	C2580_write_cmos_sensor(0x0203, (shutter & 0xFF));          
	C2580_write_cmos_sensor(0x0202, ((shutter & 0xFF00) >>8));

} /* C2580_write_shutter */
/*************************************************************************
* FUNCTION
*	C2580_set_mirror_flip
*
* DESCRIPTION
*	This function mirror/flip of C2580.
*
* PARAMETERS
*	none
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static void C2580_set_mirror_flip(kal_uint8 image_mirror)
{
	kal_uint8 hvmirror = 0;
  	hvmirror = C2580_read_cmos_sensor(0x0101);
	switch (image_mirror)
	{
	case IMAGE_NORMAL:
		C2580_write_cmos_sensor(0x0101, hvmirror&0x00);  //image normal   
		break;
		
	case IMAGE_H_MIRROR:
		C2580_write_cmos_sensor(0x0101, hvmirror|0x10);  
		break;
	case IMAGE_V_MIRROR: 
		C2580_write_cmos_sensor(0x0101, hvmirror|0x20);   
		break;		
	case IMAGE_HV_MIRROR:
		C2580_write_cmos_sensor(0x0101, hvmirror|0x30); 
		break; 		
	default:
		break;
	}
}
/*************************************************************************
* FUNCTION
*	C2580_set_AE_mode
*
* DESCRIPTION
*	This function ae mode of C2580.
*
* PARAMETERS
*	none
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/

static void C2580_set_AE_mode(kal_bool AE_enable)
{
    kal_uint8 AE_Temp;
	AE_Temp = C2580_read_cmos_sensor(0x3087);
#if 0
    if (AE_enable == KAL_TRUE)
    {
        // enable AEC/AGC
      C2580_write_cmos_sensor(0x3087,(AE_Temp&(~0x01)));
    }
    else
    {
        // disable AEC/AGC
      C2580_write_cmos_sensor(0x3087,(AE_Temp|0x80));
    }
	#endif
}

/*************************************************************************
* FUNCTION
*	C2580_set_AWB_mode
*
* DESCRIPTION
*	This function awb mode of C2580.
*
* PARAMETERS
*	none
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/

static void C2580_set_AWB_mode(kal_bool AWB_enable)
{
    kal_uint8 AWB_Temp;
	AWB_Temp = C2580_read_cmos_sensor(0x3F8C);   

    if (AWB_enable == KAL_TRUE)
    {
             
	C2580_write_cmos_sensor(0x3F8C,AWB_Temp|0x20); 

		
    }
    else
    {        
	C2580_write_cmos_sensor(0x3F8C,AWB_Temp&(~0x20)); 
    }
}
/*************************************************************************
* FUNCTION
*	C2580_night_mode
*
* DESCRIPTION
*	This function night mode of C2580.
*
* PARAMETERS
*	none
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
void C2580_night_mode(kal_bool enable)
{
	kal_uint16 Night_Temp1 = C2580_read_cmos_sensor(0x3000); 
	kal_uint16 Night_Temp2 = C2580_read_cmos_sensor(0x3086); 
	Night_Temp2 = Night_Temp1 & 0xf0;
	if (enable)
	{ 	 /* camera night mode */
        C2580_write_cmos_sensor(0x3000,Night_Temp1 | 0x01);		 //enable
        C2580_write_cmos_sensor(0x3086,Night_Temp2 | 0x01);		//1/(n+1)
    }
	else
	{    /* camera normal mode */                            
		C2580_write_cmos_sensor(0x3000, Night_Temp1 & (~0x01));//disable
		C2580_write_cmos_sensor(0x3086, Night_Temp2 & 0xf0);
 	}
	
}	/* C2580_night_mode */
/*************************************************************************
* FUNCTION
*	C2580_GetSensorID
*
* DESCRIPTION
*	This function get the sensor ID
*
* PARAMETERS
*	None
*
* RETURNS77
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 C2580_GetSensorID(kal_uint32 *sensorID)

{
    int  retry = 3; 
//	C2580_write_cmos_sensor(0x0103,0x01);		// Reset sensor
	//Sleep(30);
    do {
        *sensorID=(C2580_read_cmos_sensor(0x0000) << 8) | C2580_read_cmos_sensor(0x0001);
        if (*sensorID == C2580_SENSOR_ID)
            break; 
        SENSORDB("C2580_GetSensorID:Read Sensor ID Fail = 0x%04x\n", *sensorID); 
        retry--; 
    } while (retry > 0);

    if (*sensorID != C2580_SENSOR_ID) {
        *sensorID = 0xFFFFFFFF; 
        return ERROR_SENSOR_CONNECT_FAIL;
    }
    return ERROR_NONE;    
}   /* C2580Open  */
/*************************************************************************
* FUNCTION
*	C2580_Sensor_Init
*
* DESCRIPTION
*	This function init the sensor 
*
* PARAMETERS
*	None
*
* RETURNS77
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static void C2580_Sensor_Init(void)
{
	#if 1
	C2580_write_cmos_sensor(0x0103,0x01);
	C2580_write_cmos_sensor(0x0304,0x00);
	C2580_write_cmos_sensor(0x0342,0x0a);
	C2580_write_cmos_sensor(0x0343,0x10); 
	C2580_write_cmos_sensor(0x3087,0x00);
	C2580_write_cmos_sensor(0x3089,0x14);
	C2580_write_cmos_sensor(0x308b,0x08);
	C2580_write_cmos_sensor(0x3084,0x35);
	C2580_write_cmos_sensor(0x3085,0x54);
	C2580_write_cmos_sensor(0x3088,0xf8);
	//MIRROR
	C2580_write_cmos_sensor(0x0101,0x03);
	C2580_write_cmos_sensor(0x034b,0xb7);
	C2580_write_cmos_sensor(0x3001,0x44);
	C2580_write_cmos_sensor(0x3109,0x04);
	C2580_write_cmos_sensor(0x310a,0x42);
	C2580_write_cmos_sensor(0x3180,0xf0);
	C2580_write_cmos_sensor(0x3c00,0x03);
	C2580_write_cmos_sensor(0x380e,0x01); 
	C2580_write_cmos_sensor(0x3286,0x5c);
	C2580_write_cmos_sensor(0x3287,0x55);
	C2580_write_cmos_sensor(0x3289,0x38);
	C2580_write_cmos_sensor(0x3282,0x3d);
	C2580_write_cmos_sensor(0x3283,0x12);
	C2580_write_cmos_sensor(0x3280,0x07);
	C2580_write_cmos_sensor(0x3281,0x0a);
	C2580_write_cmos_sensor(0x328B,0x44);
	C2580_write_cmos_sensor(0x328d,0x55);
	C2580_write_cmos_sensor(0x328e,0x00);
	//lens
	#if 0
	C2580_write_cmos_sensor(0x3c80,0x9f);
	C2580_write_cmos_sensor(0x3c81,0x9f);
	C2580_write_cmos_sensor(0x3c82,0x9f);
	C2580_write_cmos_sensor(0x3c83,0xca);
	C2580_write_cmos_sensor(0x3c84,0x88);
	C2580_write_cmos_sensor(0x3c85,0xff);
	C2580_write_cmos_sensor(0x3c86,0xff);
	C2580_write_cmos_sensor(0x3c87,0x08);
	C2580_write_cmos_sensor(0x3c88,0x88);
	C2580_write_cmos_sensor(0x3c89,0x7b);
	C2580_write_cmos_sensor(0x3c8a,0x7b);
	C2580_write_cmos_sensor(0x3c8b,0x8a);
	C2580_write_cmos_sensor(0x3c8c,0x04);
	C2580_write_cmos_sensor(0x3c8d,0x01);
	C2580_write_cmos_sensor(0x3c8e,0xd0);
	C2580_write_cmos_sensor(0x3c8f,0xff);
	C2580_write_cmos_sensor(0x3c90,0x3f);
	C2580_write_cmos_sensor(0x3c91,0x41);
	C2580_write_cmos_sensor(0x3c92,0x41);
	C2580_write_cmos_sensor(0x3c93,0x30);
	C2580_write_cmos_sensor(0x3c94,0x55);
	C2580_write_cmos_sensor(0x3c95,0x5e);
	C2580_write_cmos_sensor(0x3c96,0x5e);
	C2580_write_cmos_sensor(0x3c97,0x58);
	C2580_write_cmos_sensor(0x3c98,0xaa);
	#else
	C2580_write_cmos_sensor(0x3c80,0xc0);
	C2580_write_cmos_sensor(0x3c81,0xac);
	C2580_write_cmos_sensor(0x3c82,0xac);
	C2580_write_cmos_sensor(0x3c83,0xd0);
	C2580_write_cmos_sensor(0x3c84,0x88);
	C2580_write_cmos_sensor(0x3c85,0xff);
	C2580_write_cmos_sensor(0x3c86,0xff);
	C2580_write_cmos_sensor(0x3c87,0x08);
	C2580_write_cmos_sensor(0x3c88,0x83);
	C2580_write_cmos_sensor(0x3c89,0x6d);
	C2580_write_cmos_sensor(0x3c8a,0x6d);
	C2580_write_cmos_sensor(0x3c8b,0x7f);
	C2580_write_cmos_sensor(0x3c8c,0x04);
	C2580_write_cmos_sensor(0x3c8d,0x01);
	C2580_write_cmos_sensor(0x3c8e,0xd0);
	C2580_write_cmos_sensor(0x3c8f,0xff);
	C2580_write_cmos_sensor(0x3c90,0x3f);
	C2580_write_cmos_sensor(0x3c91,0x4a);
	C2580_write_cmos_sensor(0x3c92,0x4a);
	C2580_write_cmos_sensor(0x3c93,0x32);
	C2580_write_cmos_sensor(0x3c94,0x60);
	C2580_write_cmos_sensor(0x3c95,0x68);
	C2580_write_cmos_sensor(0x3c96,0x60);
	C2580_write_cmos_sensor(0x3c97,0x68);
	C2580_write_cmos_sensor(0x3c98,0xaa);
	#endif
	//awb pre_gain
	C2580_write_cmos_sensor(0x0210,0x05);
	C2580_write_cmos_sensor(0x0211,0x75);
	C2580_write_cmos_sensor(0x0212,0x04);
	C2580_write_cmos_sensor(0x0213,0x28);
	C2580_write_cmos_sensor(0x0214,0x04);
	C2580_write_cmos_sensor(0x0215,0x00);
	//awb region
	C2580_write_cmos_sensor(0x3f80,0x00);
	C2580_write_cmos_sensor(0x3f81,0xc0);
	C2580_write_cmos_sensor(0x3f82,0x6c);
	C2580_write_cmos_sensor(0x3f83,0x62);
	C2580_write_cmos_sensor(0x3f84,0x7a);
	C2580_write_cmos_sensor(0x3f85,0x98);
	C2580_write_cmos_sensor(0x3f86,0x30);
	C2580_write_cmos_sensor(0x3f87,0x0c);
	C2580_write_cmos_sensor(0x3f88,0x74);
	C2580_write_cmos_sensor(0x3f89,0x86);
	C2580_write_cmos_sensor(0x3f8a,0x4d);
	C2580_write_cmos_sensor(0x3f8b,0x0c);
	C2580_write_cmos_sensor(0x3f08,0x10);
	C2580_write_cmos_sensor(0x3f8c,0x65);
	//gamma
	#if 0 
	C2580_write_cmos_sensor(0x3d80,0x0e);
	C2580_write_cmos_sensor(0x3d81,0x19);
	C2580_write_cmos_sensor(0x3d82,0x2d);
	C2580_write_cmos_sensor(0x3d83,0x4d);
	C2580_write_cmos_sensor(0x3d84,0x5c);
	C2580_write_cmos_sensor(0x3d85,0x6c);
	C2580_write_cmos_sensor(0x3d86,0x7c);
	C2580_write_cmos_sensor(0x3d87,0x8b);
	C2580_write_cmos_sensor(0x3d88,0x9a);
	C2580_write_cmos_sensor(0x3d89,0xa6);
	C2580_write_cmos_sensor(0x3d8a,0xb9);
	C2580_write_cmos_sensor(0x3d8b,0xc8);
	C2580_write_cmos_sensor(0x3d8c,0xdc);
	C2580_write_cmos_sensor(0x3d8d,0xed);
	C2580_write_cmos_sensor(0x3d8e,0xf6);
	C2580_write_cmos_sensor(0x3d8f,0x09);
	#else
	//denoise more
	C2580_write_cmos_sensor(0x3d80,0x15);
	C2580_write_cmos_sensor(0x3d81,0x27);
	C2580_write_cmos_sensor(0x3d82,0x3e);
	C2580_write_cmos_sensor(0x3d83,0x63);
	C2580_write_cmos_sensor(0x3d84,0x71);
	C2580_write_cmos_sensor(0x3d85,0x7f);
	C2580_write_cmos_sensor(0x3d86,0x8a);
	C2580_write_cmos_sensor(0x3d87,0x95);
	C2580_write_cmos_sensor(0x3d88,0x9f);
	C2580_write_cmos_sensor(0x3d89,0xa9);
	C2580_write_cmos_sensor(0x3d8a,0xb8);
	C2580_write_cmos_sensor(0x3d8b,0xc3);
	C2580_write_cmos_sensor(0x3d8c,0xd6);
	C2580_write_cmos_sensor(0x3d8d,0xe5);
	C2580_write_cmos_sensor(0x3d8e,0xf1);
	C2580_write_cmos_sensor(0x3d8f,0x0e);
	
	/*C2580_write_cmos_sensor(0x3d80,0x08);
	C2580_write_cmos_sensor(0x3d81,0x0d);
	C2580_write_cmos_sensor(0x3d82,0x1a);
	C2580_write_cmos_sensor(0x3d83,0x32);
	C2580_write_cmos_sensor(0x3d84,0x41);
	C2580_write_cmos_sensor(0x3d85,0x4e);
	C2580_write_cmos_sensor(0x3d86,0x5c);
	C2580_write_cmos_sensor(0x3d87,0x68);
	C2580_write_cmos_sensor(0x3d88,0x74);
	C2580_write_cmos_sensor(0x3d89,0x7f);
	C2580_write_cmos_sensor(0x3d8a,0x94);
	C2580_write_cmos_sensor(0x3d8b,0xa7);
	C2580_write_cmos_sensor(0x3d8c,0xc2);
	C2580_write_cmos_sensor(0x3d8d,0xd9);
	C2580_write_cmos_sensor(0x3d8e,0xec);
	C2580_write_cmos_sensor(0x3d8f,0x13);*/
	#endif
	//dns
	C2580_write_cmos_sensor(0x3d00,0x8f);
	C2580_write_cmos_sensor(0x3d01,0x38);
	C2580_write_cmos_sensor(0x3d02,0xff);
	C2580_write_cmos_sensor(0x3d03,0x38);
	C2580_write_cmos_sensor(0x3d04,0xff);
	C2580_write_cmos_sensor(0x3d05,0x10);
	C2580_write_cmos_sensor(0x3d06,0x13);
	C2580_write_cmos_sensor(0x3d07,0x10);
	C2580_write_cmos_sensor(0x3d08,0x10);
	C2580_write_cmos_sensor(0x3d09,0x50);
	C2580_write_cmos_sensor(0x3d0a,0x00);
	C2580_write_cmos_sensor(0x3d0b,0x00);
	C2580_write_cmos_sensor(0x3d0c,0x00);
	C2580_write_cmos_sensor(0x3d0d,0xff);
	C2580_write_cmos_sensor(0x3d0e,0x01);
	C2580_write_cmos_sensor(0x3d0f,0x00);
	C2580_write_cmos_sensor(0x3d10,0x00);
	C2580_write_cmos_sensor(0x3d11,0x10);
	C2580_write_cmos_sensor(0x3d12,0x00);
	C2580_write_cmos_sensor(0x3d13,0xf0);
	C2580_write_cmos_sensor(0x3d20,0x03);
	C2580_write_cmos_sensor(0x3d21,0x08);
	C2580_write_cmos_sensor(0x3d22,0x80);
	//sde
	C2580_write_cmos_sensor(0x3e80,0x04);
	C2580_write_cmos_sensor(0x3e83,0x01);
	C2580_write_cmos_sensor(0x3e84,0x00);
	C2580_write_cmos_sensor(0x3e85,0x00);
	C2580_write_cmos_sensor(0x3e88,0x80);
	C2580_write_cmos_sensor(0x3e89,0x00);
	//cmc
	C2580_write_cmos_sensor(0x3e00,0x9e);
	C2580_write_cmos_sensor(0x3e01,0x01);
	C2580_write_cmos_sensor(0x3e02,0x88);
	C2580_write_cmos_sensor(0x3e03,0x8b);
	C2580_write_cmos_sensor(0x3e04,0x82);
	C2580_write_cmos_sensor(0x3e05,0x95);
	//alpha sar for low light
	C2580_write_cmos_sensor(0x3c02,0x31);
	C2580_write_cmos_sensor(0x3e8b,0x10);
	C2580_write_cmos_sensor(0x3e8c,0x20);
	C2580_write_cmos_sensor(0x3e8d,0x80);
	C2580_write_cmos_sensor(0x3e8e,0x48);
	//banding filcker
	C2580_write_cmos_sensor(0x3083,0x7a);
	//mipi timing
	C2580_write_cmos_sensor(0x3805,0x06);
	C2580_write_cmos_sensor(0x3806,0x06);
	C2580_write_cmos_sensor(0x3807,0x06);
	C2580_write_cmos_sensor(0x3808,0x14);
	C2580_write_cmos_sensor(0x3809,0xC4);
	C2580_write_cmos_sensor(0x380a,0x6C);
	C2580_write_cmos_sensor(0x380b,0x8C);
	C2580_write_cmos_sensor(0x380c,0x21);
	//ananog
	#if 1
	C2580_write_cmos_sensor(0x3200,0x04);
	C2580_write_cmos_sensor(0x3201,0xb0);
	C2580_write_cmos_sensor(0x3202,0x04);
	C2580_write_cmos_sensor(0x3203,0xd0);
	C2580_write_cmos_sensor(0x3204,0x01);
	C2580_write_cmos_sensor(0x3205,0x98);
	C2580_write_cmos_sensor(0x3206,0x01);
	C2580_write_cmos_sensor(0x3207,0x57);
	C2580_write_cmos_sensor(0x3208,0xc3);
	C2580_write_cmos_sensor(0x3214,0x40);
	C2580_write_cmos_sensor(0x321b,0x10);
	C2580_write_cmos_sensor(0x3285,0x0f);
	C2580_write_cmos_sensor(0x328b,0x04);
	C2580_write_cmos_sensor(0x3880,0x40);
	#else
	C2580_write_cmos_sensor(0x3200,0x04);
	C2580_write_cmos_sensor(0x3201,0x50);
	C2580_write_cmos_sensor(0x3202,0x04);
	C2580_write_cmos_sensor(0x3203,0x70);
	C2580_write_cmos_sensor(0x3208,0xc3);
	C2580_write_cmos_sensor(0x3285,0x0f);
	C2580_write_cmos_sensor(0x328b,0x04);
	C2580_write_cmos_sensor(0x3880,0x00);
	#endif
	C2580_write_cmos_sensor(0x3d00,0x8f);
	C2580_write_cmos_sensor(0x0100,0x01);
	C2580_write_cmos_sensor(0x0205,0x01);
	#else
	C2580_write_cmos_sensor(0x0103,0x01);
	C2580_write_cmos_sensor(0x3880,0x00);
	C2580_write_cmos_sensor(0x3806,0x06);
	C2580_write_cmos_sensor(0x3809,0xC4);
	C2580_write_cmos_sensor(0x380a,0x6C);
	C2580_write_cmos_sensor(0x380b,0x8C);
	C2580_write_cmos_sensor(0x0342,0x0c);
	C2580_write_cmos_sensor(0x034b,0xb9);
	C2580_write_cmos_sensor(0x0101,0x03);
	C2580_write_cmos_sensor(0x3200,0x04);
	C2580_write_cmos_sensor(0x3201,0x50);
	C2580_write_cmos_sensor(0x3202,0x04);
	C2580_write_cmos_sensor(0x3203,0x70);
	C2580_write_cmos_sensor(0x3208,0xc3);
	C2580_write_cmos_sensor(0x3212,0x20);
	C2580_write_cmos_sensor(0x321b,0x02);
	C2580_write_cmos_sensor(0x3280,0x07);
	C2580_write_cmos_sensor(0x3282,0x8f);
	C2580_write_cmos_sensor(0x3283,0x12);
	C2580_write_cmos_sensor(0x3285,0x0f);
	C2580_write_cmos_sensor(0x3287,0x55);
	C2580_write_cmos_sensor(0x3289,0x38);
	C2580_write_cmos_sensor(0x328b,0x04);
	C2580_write_cmos_sensor(0x328d,0x55);
	C2580_write_cmos_sensor(0x3087,0x00);
	C2580_write_cmos_sensor(0x3089,0x14);
	C2580_write_cmos_sensor(0x308b,0x0f);
	C2580_write_cmos_sensor(0x3083,0x61);
	C2580_write_cmos_sensor(0x3084,0x40);
	C2580_write_cmos_sensor(0x3085,0x54);
	C2580_write_cmos_sensor(0x3088,0xf8);
	C2580_write_cmos_sensor(0x3000,0x81);
	C2580_write_cmos_sensor(0x3001,0x44);
	C2580_write_cmos_sensor(0x3109,0x04);
	C2580_write_cmos_sensor(0x310a,0x42);
	C2580_write_cmos_sensor(0x3180,0xf0);
	C2580_write_cmos_sensor(0x3c00,0x03);
	C2580_write_cmos_sensor(0x380e,0x01);
	C2580_write_cmos_sensor(0x3f08,0x10);
	C2580_write_cmos_sensor(0x0210,0x05);
	C2580_write_cmos_sensor(0x0211,0x75);
	C2580_write_cmos_sensor(0x0212,0x04);
	C2580_write_cmos_sensor(0x0213,0x28);
	C2580_write_cmos_sensor(0x0214,0x04);
	C2580_write_cmos_sensor(0x0215,0x00);
	C2580_write_cmos_sensor(0x3f8c,0x65);
	C2580_write_cmos_sensor(0x3f80,0x00);
	C2580_write_cmos_sensor(0x3f81,0xc0);
	C2580_write_cmos_sensor(0x3f82,0x6c);
	C2580_write_cmos_sensor(0x3f83,0x62);
	C2580_write_cmos_sensor(0x3f84,0x7a);
	C2580_write_cmos_sensor(0x3f85,0x98);
	C2580_write_cmos_sensor(0x3f86,0x30);
	C2580_write_cmos_sensor(0x3f87,0x0c);
	C2580_write_cmos_sensor(0x3f88,0x74);
	C2580_write_cmos_sensor(0x3f89,0x86);
	C2580_write_cmos_sensor(0x3f8a,0x4d);
	C2580_write_cmos_sensor(0x3f8b,0x0c);
	C2580_write_cmos_sensor(0x3f82,0x66);
	C2580_write_cmos_sensor(0x3f83,0x64);
	C2580_write_cmos_sensor(0x3d80,0x19);
	C2580_write_cmos_sensor(0x3d81,0x25);
	C2580_write_cmos_sensor(0x3d82,0x36);
	C2580_write_cmos_sensor(0x3d83,0x50);
	C2580_write_cmos_sensor(0x3d84,0x5a);
	C2580_write_cmos_sensor(0x3d85,0x64);
	C2580_write_cmos_sensor(0x3d86,0x6d);
	C2580_write_cmos_sensor(0x3d87,0x76);
	C2580_write_cmos_sensor(0x3d88,0x7e);
	C2580_write_cmos_sensor(0x3d89,0x85);
	C2580_write_cmos_sensor(0x3d8a,0x93);
	C2580_write_cmos_sensor(0x3d8b,0xa1);
	C2580_write_cmos_sensor(0x3d8c,0xb9);
	C2580_write_cmos_sensor(0x3d8d,0xcf);
	C2580_write_cmos_sensor(0x3d8e,0xe3);
	C2580_write_cmos_sensor(0x3d8f,0x1c);
	C2580_write_cmos_sensor(0x3d00,0x8f);
	C2580_write_cmos_sensor(0x3d01,0x30);
	C2580_write_cmos_sensor(0x3d03,0x30);
	C2580_write_cmos_sensor(0x3d05,0x0A);
	C2580_write_cmos_sensor(0x3d0E,0x00);
	C2580_write_cmos_sensor(0x3d0F,0xC0);
	C2580_write_cmos_sensor(0x3D22,0x70);
	C2580_write_cmos_sensor(0x3e80,0x04);
	C2580_write_cmos_sensor(0x3e83,0x01);
	C2580_write_cmos_sensor(0x3e88,0x80);
	C2580_write_cmos_sensor(0x3e00,0x9e);
	C2580_write_cmos_sensor(0x3e01,0x01);
	C2580_write_cmos_sensor(0x3e02,0x88);
	C2580_write_cmos_sensor(0x3e03,0x8b);
	C2580_write_cmos_sensor(0x3e04,0x82);
	C2580_write_cmos_sensor(0x3e05,0x95);
	C2580_write_cmos_sensor(0x0100,0x01);
	C2580_write_cmos_sensor(0x0205,0x01);
	#endif	
}
/*************************************************************************
* FUNCTION
*	C2580_Sensor_VGA
*
* DESCRIPTION
*	This function get the sensor vga setting 
*
* PARAMETERS
*	None
*
* RETURNS77
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static void C2580_Sensor_VGA(void)
{
#if 0
	C2580_write_cmos_sensor(0x0383,0x03);
	C2580_write_cmos_sensor(0x0387,0x03);
	
	C2580_write_cmos_sensor(0x0303,0x02);
	C2580_write_cmos_sensor(0x0304,0x01);
	C2580_write_cmos_sensor(0x0305,0x01);
	C2580_write_cmos_sensor(0x0307,0x30);
	C2580_write_cmos_sensor(0x0309,0x20);
	
	
	C2580_write_cmos_sensor(0x0344,0x00);
	C2580_write_cmos_sensor(0x0345,0x00);
	C2580_write_cmos_sensor(0x0346,0x00);
	C2580_write_cmos_sensor(0x0347,0x00);
	C2580_write_cmos_sensor(0x0348,0x06);
	C2580_write_cmos_sensor(0x0349,0x4f);
	C2580_write_cmos_sensor(0x034a,0x04);
	C2580_write_cmos_sensor(0x034b,0xbb);
	C2580_write_cmos_sensor(0x034c,0x02);
	C2580_write_cmos_sensor(0x034d,0x80);
	C2580_write_cmos_sensor(0x034e,0x01);
	C2580_write_cmos_sensor(0x034f,0xE0);
	C2580_write_cmos_sensor(0x3c07,0x00);
	
	C2580_write_cmos_sensor(0x0340,0x04);
	C2580_write_cmos_sensor(0x0341,0xC8);
	C2580_write_cmos_sensor(0x0342,0x05);
	C2580_write_cmos_sensor(0x0343,0xBf);
	
	C2580_write_cmos_sensor(0x3286,0x5c);
	C2580_write_cmos_sensor(0x3287,0x55);
	C2580_write_cmos_sensor(0x3289,0x38);
	C2580_write_cmos_sensor(0x3282,0x95);
	C2580_write_cmos_sensor(0x3283,0x14);
	C2580_write_cmos_sensor(0x3280,0x02);
	C2580_write_cmos_sensor(0x3281,0x0a);
	C2580_write_cmos_sensor(0x328B,0x44);
	C2580_write_cmos_sensor(0x328d,0x55);
	
	C2580_write_cmos_sensor(0x3087,0x00);
	C2580_write_cmos_sensor(0x3080,0x00);
	C2580_write_cmos_sensor(0x3081,0x5c);
	C2580_write_cmos_sensor(0x3082,0x00);
	C2580_write_cmos_sensor(0x3083,0x5C);
	
	
	C2580_write_cmos_sensor(0x3880,0x00);
	C2580_write_cmos_sensor(0x3881,0x80);
	C2580_write_cmos_sensor(0x0100,0x01);
	#endif

}

/*************************************************************************
* FUNCTION
*	C2580_Sensor_SVGA
*
* DESCRIPTION
*	This function get the sensor svga setting 
*
* PARAMETERS
*	None
*
* RETURNS77
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static void C2580_Sensor_SVGA(void)
{
	#if 0
	C2580_write_cmos_sensor(0x0303,0x00);
	C2580_write_cmos_sensor(0x0304,0x02);//01
	C2580_write_cmos_sensor(0x0309,0x10);
	C2580_write_cmos_sensor(0x0340,0x03);//2
	C2580_write_cmos_sensor(0x0341,0x92);//64
	C2580_write_cmos_sensor(0x0383,0x03);
	C2580_write_cmos_sensor(0x0387,0x03);
	C2580_write_cmos_sensor(0x034c,0x03);
	C2580_write_cmos_sensor(0x034d,0x20);
	C2580_write_cmos_sensor(0x034e,0x02);
	C2580_write_cmos_sensor(0x034f,0x58);
	C2580_write_cmos_sensor(0x3009,0x04);
	C2580_write_cmos_sensor(0x300b,0x02);
	C2580_write_cmos_sensor(0x3021,0x08);
	C2580_write_cmos_sensor(0x3022,0x01);
	C2580_write_cmos_sensor(0x308b,0x07);
	C2580_write_cmos_sensor(0x3880,0x40);
	C2580_write_cmos_sensor(0x3806,0x06);
	C2580_write_cmos_sensor(0x3809,0xC4);
	C2580_write_cmos_sensor(0x380a,0x6C);
	C2580_write_cmos_sensor(0x380b,0x8C);
	C2580_write_cmos_sensor(0x3d00,0x8f);
	C2580_write_cmos_sensor(0x3f03,0x02);
	C2580_write_cmos_sensor(0x3f04,0x03);
	C2580_write_cmos_sensor(0x3f05,0x20);
	C2580_write_cmos_sensor(0x3f06,0x02);
	C2580_write_cmos_sensor(0x3f07,0x58);
	C2580_write_cmos_sensor(0x3200,0x04);
	C2580_write_cmos_sensor(0x3201,0x50);
	C2580_write_cmos_sensor(0x3202,0x04);
	C2580_write_cmos_sensor(0x3203,0x70);
	C2580_write_cmos_sensor(0x3208,0xc3);
	C2580_write_cmos_sensor(0x3285,0x0f);
	C2580_write_cmos_sensor(0x328b,0x04);
	C2580_write_cmos_sensor(0x0100,0x01);
	#else
	/*
	C2580_write_cmos_sensor(0x0100,0x00);
	C2580_write_cmos_sensor(0x0303,0x00);
	C2580_write_cmos_sensor(0x0304,0x01);
	C2580_write_cmos_sensor(0x0309,0x10);
	C2580_write_cmos_sensor(0x0340,0x02);
	C2580_write_cmos_sensor(0x0341,0x64);
	C2580_write_cmos_sensor(0x0387,0x03);
	C2580_write_cmos_sensor(0x034c,0x03);
	C2580_write_cmos_sensor(0x034d,0x20);
	C2580_write_cmos_sensor(0x034e,0x02);
	C2580_write_cmos_sensor(0x034f,0x58);
	C2580_write_cmos_sensor(0x3009,0x04);
	C2580_write_cmos_sensor(0x300b,0x02);
	C2580_write_cmos_sensor(0x3021,0x08);
	C2580_write_cmos_sensor(0x308b,0x07);
	C2580_write_cmos_sensor(0x3880,0x40);
	C2580_write_cmos_sensor(0x3806,0x06);
	C2580_write_cmos_sensor(0x3809,0xC4);
	C2580_write_cmos_sensor(0x380a,0x6C);
	C2580_write_cmos_sensor(0x380b,0x8C);
	C2580_write_cmos_sensor(0x3d00,0x8f);
	C2580_write_cmos_sensor(0x3c07,0x01);
	C2580_write_cmos_sensor(0x3c08,0x40);
	C2580_write_cmos_sensor(0x3c0c,0x01);
	C2580_write_cmos_sensor(0x3f03,0x02);
	C2580_write_cmos_sensor(0x3f04,0x03);
	C2580_write_cmos_sensor(0x3f05,0x20);
	C2580_write_cmos_sensor(0x3f06,0x02);
	C2580_write_cmos_sensor(0x3f07,0x58);
	C2580_write_cmos_sensor(0x3200,0x04);
	C2580_write_cmos_sensor(0x3201,0x50);
	C2580_write_cmos_sensor(0x3202,0x04);
	C2580_write_cmos_sensor(0x3203,0x70);
	C2580_write_cmos_sensor(0x3208,0xc3);
	C2580_write_cmos_sensor(0x3285,0x0f);
	C2580_write_cmos_sensor(0x328b,0x04);
	C2580_write_cmos_sensor(0x0342,0x06);
	C2580_write_cmos_sensor(0x3083,0x5c);
	C2580_write_cmos_sensor(0x0100,0x01);*/
	#endif
}
/*************************************************************************
* FUNCTION
*	C2580_Sensor_UXGA
*
* DESCRIPTION
*	This function get the sensor 2m setting 
*
* PARAMETERS
*	None
*
* RETURNS77
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static void C2580_Sensor_2M(void)
{
		#if 0
		C2580_write_cmos_sensor(0x0100,0x00);
		C2580_write_cmos_sensor(0x0303,0x02);
		C2580_write_cmos_sensor(0x0304,0x00);
		C2580_write_cmos_sensor(0x0309,0x20);
		C2580_write_cmos_sensor(0x0340,0x04);
		C2580_write_cmos_sensor(0x0341,0xc8);
		C2580_write_cmos_sensor(0x0387,0x01);
		C2580_write_cmos_sensor(0x034c,0x06);
		C2580_write_cmos_sensor(0x034d,0x40);
		C2580_write_cmos_sensor(0x034e,0x04);
		C2580_write_cmos_sensor(0x034f,0xb0);
		C2580_write_cmos_sensor(0x3009,0x04);
		C2580_write_cmos_sensor(0x300b,0x04);
		C2580_write_cmos_sensor(0x3021,0x00);
		C2580_write_cmos_sensor(0x308b,0x07);
		C2580_write_cmos_sensor(0x3880,0x40);
		C2580_write_cmos_sensor(0x3806,0x06);
		C2580_write_cmos_sensor(0x3809,0xC4);
		C2580_write_cmos_sensor(0x380a,0x6C);
		C2580_write_cmos_sensor(0x380b,0x8C);
		C2580_write_cmos_sensor(0x3d00,0x8f);
		C2580_write_cmos_sensor(0x3c07,0x57);
		C2580_write_cmos_sensor(0x3c08,0x00);
		C2580_write_cmos_sensor(0x3c0c,0x00);
		C2580_write_cmos_sensor(0x3f03,0x04);
		C2580_write_cmos_sensor(0x3f04,0x06);
		C2580_write_cmos_sensor(0x3f05,0x40);
		C2580_write_cmos_sensor(0x3f06,0x04);
		C2580_write_cmos_sensor(0x3f07,0xb0);
		C2580_write_cmos_sensor(0x3200,0x04);
		C2580_write_cmos_sensor(0x3201,0x50);
		C2580_write_cmos_sensor(0x3202,0x04);
		C2580_write_cmos_sensor(0x3203,0x70);
		C2580_write_cmos_sensor(0x3208,0xc3);
		C2580_write_cmos_sensor(0x3285,0x0f);
		C2580_write_cmos_sensor(0x328b,0x04);
		//C2580_write_cmos_sensor(0x0342,0x0c);
		C2580_write_cmos_sensor(0x3083,0x61);
		C2580_write_cmos_sensor(0x0100,0x01);
		#endif

}
/*************************************************************************
* FUNCTION
*	C2580Open
*
* DESCRIPTION
*	This function initialize the registers of CMOS sensor
*
* PARAMETERS
*	None
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
UINT32 C2580Open(void)
{
	volatile signed char i;
	kal_uint16 sensor_id=0;
	zoom_factor = 0; 
//	C2580_write_cmos_sensor(0x0103,0x01);// Reset sensor
	Sleep(10);

	//  Read sensor ID to adjust I2C is OK?
	for(i=0;i<3;i++)
	{
		sensor_id = (C2580_read_cmos_sensor(0x0000) << 8) | C2580_read_cmos_sensor(0x0001);
		SENSORDB("C2580Open: READ ID :%x",sensor_id);
		if(sensor_id != C2580_SENSOR_ID)
		{
			return ERROR_SENSOR_CONNECT_FAIL;
		}
	}
	SENSORDB("C2580Open: Sensor Read ID OK \r\n");
	C2580_Sensor_Init();
	SENSORDB("C2580Open: Sensor init ready");
	return ERROR_NONE;
}	/* C2580Open() */

/*************************************************************************
* FUNCTION
*	C2580Close
*
* DESCRIPTION
*	This function is to turn off sensor module power.
*
* PARAMETERS
*	None
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
UINT32 C2580Close(void)
{
	//CISModulePowerOn(FALSE);
	//C2580_write_cmos_sensor(0x0103,0x00);
	return ERROR_NONE;
}	/* C2580Close() */

/*************************************************************************
* FUNCTION
*	C2580Preview
*
* DESCRIPTION
*	This function start the sensor preview.
*
* PARAMETERS
*	*image_window : address pointer of pixel numbers in one period of HSYNC
*  *sensor_config_data : address pointer of line numbers in one period of VSYNC
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
UINT32 C2580Preview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	kal_uint8 iTemp, temp_AE_reg, temp_AWB_reg;
	kal_uint16 iDummyPixels = 0, iDummyLines = 0, iStartX = 0, iStartY = 0;
	SENSORDB("C2580Previe\n");
	C2580_sensor_cap_state = KAL_FALSE;
	//C2580_set_AE_mode(KAL_TRUE); 
	//C2580_set_AWB_mode(KAL_TRUE);
	//C2580_Sensor_2M();
	C2580_set_AE_mode(KAL_TRUE); 
	C2580_set_AWB_mode(KAL_TRUE);
	SENSORDB("C2580Preview: Sensor pre ready");
	//image_window->GrabStartX=8;
	//image_window->GrabStartY=8;
	//image_window->ExposureWindowWidth=C2580_IMAGE_SENSOR_PV_WIDTH;		//-16
	//image_window->ExposureWindowHeight=C2580_IMAGE_SENSOR_PV_HEIGHT;		//-12 
	// copy sensor_config_data
	memcpy(&C2580SensorConfigData, sensor_config_data, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
	return ERROR_NONE;
}	/* C2580Preview() */
/*************************************************************************
* FUNCTION
*	C2580Capture
*
* DESCRIPTION
*	This function start the sensor Capture.
*
* PARAMETERS
*	
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
UINT32 C2580Capture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
  
        volatile kal_uint32 shutter = 0,Capture_Shutter=0;
				kal_uint32 VTS_new=0;
					#if 0
    	if(C2580_sensor_cap_state == KAL_FALSE)
		{
			C2580_set_AE_mode(KAL_FALSE);
			shutter = C2580_read_shutter();
			C2580_Sensor_2M();  ////  2M_setting
			mDELAY(10);
		
			Capture_Shutter = shutter*(pre_clk*cap_line_length*cap_row_length)/(cap_clk*pre_line_lenth*pre_row_length);											
			if(Capture_Shutter  <= C2580_FULL_PERIOD_LINE_NUMS)
			 {
			 	C2580_write_shutter(Capture_Shutter);
			 	mDELAY(30);
				}
			else
			{
				VTS_new = Capture_Shutter + 2;
				C2580_write_cmos_sensor(0x0340, (VTS_new >> 8) & 0xff);
				C2580_write_cmos_sensor(0x0341, VTS_new & 0xff);
				C2580_write_shutter(Capture_Shutter);
			 	mDELAY(30);
			}
			
    		}
    		#endif
		C2580_sensor_cap_state = KAL_TRUE;

		 /* 2M FULL Mode */
		SENSORDB("C2580Capture 2M\n");
		image_window->GrabStartX=2;
		image_window->GrabStartY=2;
		image_window->ExposureWindowWidth=C2580_IMAGE_SENSOR_FULL_WIDTH - image_window->GrabStartX;		//-32
		image_window->ExposureWindowHeight=C2580_IMAGE_SENSOR_FULL_HEIGHT - image_window->GrabStartY;    	//-24 

    	memcpy(&C2580SensorConfigData, sensor_config_data, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
	return ERROR_NONE;  
}	/* C2580Capture() */
/*************************************************************************
* FUNCTION
*	C2580GetResolution
*
* DESCRIPTION
*	This function get the sensor Resolution.  
*
* PARAMETERS
*	
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
UINT32 C2580GetResolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *pSensorResolution)
{
	pSensorResolution->SensorFullWidth=C2580_IMAGE_SENSOR_FULL_WIDTH - 8 * IMAGE_SENSOR_START_GRAB_X;
	pSensorResolution->SensorFullHeight=C2580_IMAGE_SENSOR_FULL_HEIGHT- 8 * IMAGE_SENSOR_START_GRAB_Y;
	pSensorResolution->SensorPreviewWidth=C2580_IMAGE_SENSOR_PV_WIDTH - 4 * IMAGE_SENSOR_START_GRAB_X;
	pSensorResolution->SensorPreviewHeight=C2580_IMAGE_SENSOR_PV_HEIGHT- 4 * IMAGE_SENSOR_START_GRAB_Y;
  pSensorResolution->SensorVideoWidth=C2580_IMAGE_SENSOR_PV_WIDTH - 4 * IMAGE_SENSOR_START_GRAB_X;																																																	
  pSensorResolution->SensorVideoHeight=C2580_IMAGE_SENSOR_PV_HEIGHT- 4 * IMAGE_SENSOR_START_GRAB_Y;																																																	
	return ERROR_NONE;
}	/* C2580GetResolution() */
/*************************************************************************
* FUNCTION
*	C2580GetInfo
*
* DESCRIPTION
*	This function get the sensor info.
*
* PARAMETERS
*	
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
UINT32 C2580GetInfo(MSDK_SCENARIO_ID_ENUM ScenarioId,
					  MSDK_SENSOR_INFO_STRUCT *pSensorInfo,
					  MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
	pSensorInfo->SensorPreviewResolutionX=C2580_IMAGE_SENSOR_PV_WIDTH - 4 * IMAGE_SENSOR_START_GRAB_X; 
	pSensorInfo->SensorPreviewResolutionY=C2580_IMAGE_SENSOR_PV_HEIGHT - 4 * IMAGE_SENSOR_START_GRAB_Y;;
	pSensorInfo->SensorFullResolutionX=C2580_IMAGE_SENSOR_FULL_WIDTH - 8 * IMAGE_SENSOR_START_GRAB_X;
	pSensorInfo->SensorFullResolutionY=C2580_IMAGE_SENSOR_FULL_HEIGHT - 8 * IMAGE_SENSOR_START_GRAB_Y;

	pSensorInfo->SensorCameraPreviewFrameRate=30;
	pSensorInfo->SensorVideoFrameRate=30;
	pSensorInfo->SensorStillCaptureFrameRate=10;
	pSensorInfo->SensorWebCamCaptureFrameRate=15;
	pSensorInfo->SensorResetActiveHigh=FALSE;
	pSensorInfo->SensorResetDelayCount=4;
	pSensorInfo->SensorOutputDataFormat=SENSOR_OUTPUT_FORMAT_YVYU;//VYUY;
	pSensorInfo->SensorClockPolarity=SENSOR_CLOCK_POLARITY_LOW;	/*??? */
	pSensorInfo->SensorClockFallingPolarity=SENSOR_CLOCK_POLARITY_LOW;
	pSensorInfo->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	pSensorInfo->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	pSensorInfo->SensorInterruptDelayLines = 1;
	pSensorInfo->CaptureDelayFrame = 4; 
	pSensorInfo->PreviewDelayFrame = 1; 
	pSensorInfo->VideoDelayFrame = 0; 
	pSensorInfo->SensorMasterClockSwitch = 0; 
	pSensorInfo->SensorDrivingCurrent = ISP_DRIVING_6MA;
	pSensorInfo->YUVAwbDelayFrame = 2; 
	pSensorInfo->YUVEffectDelayFrame = 2;  
	
	pSensorInfo->SensroInterfaceType = SENSOR_INTERFACE_TYPE_MIPI;
	#if 0
	pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_100_MODE].MaxWidth=CAM_SIZE_2M_WIDTH;
	pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_100_MODE].MaxHeight=CAM_SIZE_2M_HEIGHT;
	pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_100_MODE].ISOSupported=TRUE;
	pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_100_MODE].BinningEnable=FALSE;

	pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_200_MODE].MaxWidth=CAM_SIZE_2M_WIDTH;
	pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_200_MODE].MaxHeight=CAM_SIZE_2M_HEIGHT;
	pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_200_MODE].ISOSupported=TRUE;
	pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_200_MODE].BinningEnable=FALSE;

	pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_400_MODE].MaxWidth=CAM_SIZE_2M_WIDTH;
	pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_400_MODE].MaxHeight=CAM_SIZE_2M_HEIGHT;
	pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_400_MODE].ISOSupported=TRUE;
	pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_400_MODE].BinningEnable=FALSE;

	pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_800_MODE].MaxWidth=CAM_SIZE_2M_WIDTH;
	pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_800_MODE].MaxHeight=CAM_SIZE_2M_HEIGHT;
	pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_800_MODE].ISOSupported=TRUE;
	pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_800_MODE].BinningEnable=FALSE;

	pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_1600_MODE].MaxWidth=CAM_SIZE_2M_WIDTH;
	pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_1600_MODE].MaxHeight=CAM_SIZE_2M_HEIGHT;
	pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_1600_MODE].ISOSupported=TRUE;
	pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_1600_MODE].BinningEnable=FALSE;	
	#endif
	switch (ScenarioId)
	{
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		//case MSDK_SCENARIO_ID_VIDEO_CAPTURE_MPEG4:
			pSensorInfo->SensorClockFreq=24;
			pSensorInfo->SensorClockDividCount=3;			//5
			pSensorInfo->SensorClockRisingCount= 0;
			pSensorInfo->SensorClockFallingCount= 2;
			pSensorInfo->SensorPixelClockCount= 3;
			pSensorInfo->SensorDataLatchCount= 2;
	        pSensorInfo->SensorGrabStartX = 2; 
	        pSensorInfo->SensorGrabStartY = 2;
			//pSensorInfo->SensorPreviewResolutionX=C2580_IMAGE_SENSOR_PV_WIDTH;
			//pSensorInfo->SensorPreviewResolutionY=C2580_IMAGE_SENSOR_PV_HEIGHT;
		//	pSensorInfo->SensorFullResolutionX=C2580_IMAGE_SENSOR_FULL_WIDTH;
		//	pSensorInfo->SensorFullResolutionY=C2580_IMAGE_SENSOR_FULL_HEIGHT;

			pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_1_LANE;		
			pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0; 
			pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 14;
			pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
			pSensorInfo->SensorWidthSampling = 0;  // 0 is default 1x
			pSensorInfo->SensorHightSampling = 0;   // 0 is default 1x 
			pSensorInfo->SensorPacketECCOrder = 1;
		break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		//case MSDK_SCENARIO_ID_CAMERA_CAPTURE_MEM:
			pSensorInfo->SensorClockFreq=24;
			pSensorInfo->SensorClockDividCount=3;			//5
			pSensorInfo->SensorClockRisingCount= 0;
			pSensorInfo->SensorClockFallingCount= 2;
			pSensorInfo->SensorPixelClockCount= 3;
			pSensorInfo->SensorDataLatchCount= 2;
            pSensorInfo->SensorGrabStartX = 2; 
            pSensorInfo->SensorGrabStartY = 2;
			//pSensorInfo->SensorPreviewResolutionX=C2580_IMAGE_SENSOR_PV_WIDTH;
			//pSensorInfo->SensorPreviewResolutionY=C2580_IMAGE_SENSOR_PV_HEIGHT;
			//pSensorInfo->SensorFullResolutionX=C2580_IMAGE_SENSOR_FULL_WIDTH;
			//pSensorInfo->SensorFullResolutionY=C2580_IMAGE_SENSOR_FULL_HEIGHT;

			pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_1_LANE;		
			pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0; 
			pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 14;
			pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
			pSensorInfo->SensorWidthSampling = 0;  // 0 is default 1x
			pSensorInfo->SensorHightSampling = 0;   // 0 is default 1x 
			pSensorInfo->SensorPacketECCOrder = 1;	
		break;
		default:
			pSensorInfo->SensorClockFreq=24;
			pSensorInfo->SensorClockDividCount=3;			//5
			pSensorInfo->SensorClockRisingCount=0;
			pSensorInfo->SensorClockFallingCount=2;
			pSensorInfo->SensorPixelClockCount=3;
			pSensorInfo->SensorDataLatchCount=2;
            pSensorInfo->SensorGrabStartX = 2; 
            pSensorInfo->SensorGrabStartY = 2;             
			//pSensorInfo->SensorPreviewResolutionX=C2580_IMAGE_SENSOR_PV_WIDTH;
			//pSensorInfo->SensorPreviewResolutionY=C2580_IMAGE_SENSOR_PV_HEIGHT;
			//pSensorInfo->SensorFullResolutionX=C2580_IMAGE_SENSOR_FULL_WIDTH;
			//pSensorInfo->SensorFullResolutionY=C2580_IMAGE_SENSOR_FULL_HEIGHT;
			
			pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_1_LANE;		
			pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0; 
			pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 14;
			pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
			pSensorInfo->SensorWidthSampling = 0;  // 0 is default 1x
			pSensorInfo->SensorHightSampling = 0;   // 0 is default 1x 
			pSensorInfo->SensorPacketECCOrder = 1;
		break;
	}
	memcpy(pSensorConfigData, &C2580SensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
	return ERROR_NONE;
}	/* C2580GetInfo() */
/*************************************************************************
* FUNCTION
*	C2580Control
*
* DESCRIPTION
*	This function control the sensor pre/cap.
*
* PARAMETERS
*	
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
UINT32 C2580Control(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *pImageWindow,
					  MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
	switch (ScenarioId)
	{
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		//case MSDK_SCENARIO_ID_VIDEO_CAPTURE_MPEG4:
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
		   	C2580Preview(pImageWindow, pSensorConfigData);
			   break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		//case MSDK_SCENARIO_ID_CAMERA_CAPTURE_MEM:
			C2580Capture(pImageWindow, pSensorConfigData);
		break;
		default:
		    break; 
	}
	return ERROR_NONE;
}	/* C2580Control() */
/*************************************************************************
* FUNCTION
*	C2580_set_param_wb
*
* DESCRIPTION
*	This function set the awb.
*
* PARAMETERS
*	
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
BOOL C2580_set_param_wb(UINT16 para)
{
    switch (para)
    {                   
        case AWB_MODE_AUTO:		
			C2580_set_AWB_mode(KAL_TRUE);
			break;
        case AWB_MODE_CLOUDY_DAYLIGHT: 		
			C2580_write_cmos_sensor(0x3F8C,0x45); 
			C2580_write_cmos_sensor(0x0210,0x05); 
			C2580_write_cmos_sensor(0x0211,0xC9); 
			C2580_write_cmos_sensor(0x0212,0x04); 
			C2580_write_cmos_sensor(0x0213,0xD6); 
			C2580_write_cmos_sensor(0x0214,0x04); 
			C2580_write_cmos_sensor(0x0215,0x00); 
            break;
        case AWB_MODE_DAYLIGHT: 
			C2580_write_cmos_sensor(0x3F8C,0x45);
			C2580_write_cmos_sensor(0x0210,0x06);
			C2580_write_cmos_sensor(0x0211,0x00);
			C2580_write_cmos_sensor(0x0212,0x04);
			C2580_write_cmos_sensor(0x0213,0x6B);
			C2580_write_cmos_sensor(0x0214,0x04);
			C2580_write_cmos_sensor(0x0215,0x00);
			break;
        case AWB_MODE_INCANDESCENT:
			C2580_write_cmos_sensor(0x3F8C,0x45);	
			C2580_write_cmos_sensor(0x0210,0x04);	
			C2580_write_cmos_sensor(0x0211,0x32);	
			C2580_write_cmos_sensor(0x0212,0x05);	
			C2580_write_cmos_sensor(0x0213,0xFD);	
			C2580_write_cmos_sensor(0x0214,0x04);	
			C2580_write_cmos_sensor(0x0215,0x00);	
			break; 
		case AWB_MODE_TUNGSTEN:
           
          break;
		case AWB_MODE_FLUORESCENT:
			C2580_write_cmos_sensor(0x3F8C,0x45);
			C2580_write_cmos_sensor(0x0210,0x05);
			C2580_write_cmos_sensor(0x0211,0x60);
			C2580_write_cmos_sensor(0x0212,0x05);
			C2580_write_cmos_sensor(0x0213,0xAB);
			C2580_write_cmos_sensor(0x0214,0x04);
			C2580_write_cmos_sensor(0x0215,0x00);
   			 break;
    default:
    return FALSE;
    }
       return TRUE;
} /* C2580_set_param_wb */
/*************************************************************************
* FUNCTION
*	C2580_set_param_effect
*
* DESCRIPTION
*	This function set the effect.
*
* PARAMETERS
*	
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
BOOL C2580_set_param_effect(UINT16 para)
{
	switch (para)
	{
		case MEFFECT_OFF:		
			C2580_write_cmos_sensor(0x3C02,0x31);
			C2580_write_cmos_sensor(0x3E86,0x00);
			C2580_write_cmos_sensor(0x3E83,0x01);
			C2580_write_cmos_sensor(0x3E81,0x00);
			C2580_write_cmos_sensor(0x3E82,0x00);
			
		break;
		case MEFFECT_SEPIA:
			C2580_write_cmos_sensor(0x3C02,0x33);
			C2580_write_cmos_sensor(0x3E86,0x00);
			C2580_write_cmos_sensor(0x3E83,0x03);
			C2580_write_cmos_sensor(0x3E81,0x40);
			C2580_write_cmos_sensor(0x3E82,0x10);
		break;  
		case MEFFECT_NEGATIVE:				
			C2580_write_cmos_sensor(0x3C02,0x37);
			C2580_write_cmos_sensor(0x3E86,0x00);
			C2580_write_cmos_sensor(0x3E83,0x01);
			C2580_write_cmos_sensor(0x3E81,0x00);
			C2580_write_cmos_sensor(0x3E82,0x00);
		break; 
		case MEFFECT_SEPIAGREEN:		
			C2580_write_cmos_sensor(0x3C02,0x33);
			C2580_write_cmos_sensor(0x3E86,0x00);
			C2580_write_cmos_sensor(0x3E83,0x07);
			C2580_write_cmos_sensor(0x3E81,0x20);
			C2580_write_cmos_sensor(0x3E82,0x20);
		break;
		case MEFFECT_SEPIABLUE:			
			C2580_write_cmos_sensor(0x3C02,0x33); 
			C2580_write_cmos_sensor(0x3E86,0x00); 
			C2580_write_cmos_sensor(0x3E83,0x05); 
			C2580_write_cmos_sensor(0x3E81,0x20); 
			C2580_write_cmos_sensor(0x3E82,0x20); 
		break;
		case MEFFECT_MONO:						
			C2580_write_cmos_sensor(0x3C02,0x33); 
			C2580_write_cmos_sensor(0x3E86,0x00); 
			C2580_write_cmos_sensor(0x3E83,0x01); 
			C2580_write_cmos_sensor(0x3E81,0x00); 
			C2580_write_cmos_sensor(0x3E82,0x00); 
		break;

		default:
		return FALSE;
    }
	return TRUE;
} /* C2580_set_param_effect */
/*************************************************************************
* FUNCTION
*	C2580_set_param_banding
*
* DESCRIPTION
*	This function set the banding flicker.
*
* PARAMETERS
*	
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
BOOL C2580_set_param_banding(UINT16 para)
{
    printk("C2580_set_param_banding  para= %d\n", para);
    switch (para)
    {
        case AE_FLICKER_MODE_50HZ: 
		{		
			C2580_write_cmos_sensor(0x3087, 0x00);
			C2580_write_cmos_sensor(0x3082, 0x00);
			C2580_write_cmos_sensor(0x3083, 0x7a);
		    break;
		}
        case AE_FLICKER_MODE_60HZ:
		{
			C2580_write_cmos_sensor(0x3087, 0x01); 
			C2580_write_cmos_sensor(0x3080, 0x00); 
			C2580_write_cmos_sensor(0x3081, 0x66); 
			break;
		}
         default:
        return FALSE;
    }

    return TRUE;
} /* C2580_set_param_banding */
/*************************************************************************
* FUNCTION
*	C2580_set_param_exposure
*
* DESCRIPTION
*	This function set the exposure.
*
* PARAMETERS
*	
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
BOOL C2580_set_param_exposure(UINT16 para)
{
	kal_uint8 reg0x3e83;
	//reg0x3e83 = C2580_read_cmos_sensor(0x3e83);
	switch (para)
	{
		case AE_EV_COMP_n13:
			//C2580_write_cmos_sensor(0x3e83, reg0x3e83 | 0x01);
		
			C2580_write_cmos_sensor(0x3e89, 0xc0);
		break;
		case AE_EV_COMP_n10:
			//C2580_write_cmos_sensor(0x3e83, reg0x3e83 | 0x01);
		
			C2580_write_cmos_sensor(0x3e89, 0xd0);
		break;
		case AE_EV_COMP_n07:
			//C2580_write_cmos_sensor(0x3e83, reg0x3e83 | 0x01);
	
			C2580_write_cmos_sensor(0x3e89, 0xe0);	
		break;
		case AE_EV_COMP_n03:
			//C2580_write_cmos_sensor(0x3e83, reg0x3e83 | 0x01);
		
			C2580_write_cmos_sensor(0x3e89, 0xf0);	
		break;
		case AE_EV_COMP_00:
			//C2580_write_cmos_sensor(0x3e83, reg0x3e83 | 0x01);
			C2580_write_cmos_sensor(0x3e89, 0x00);
		break;
		case AE_EV_COMP_03:
			//C2580_write_cmos_sensor(0x3e83, reg0x3e83 & 0xfe);
			C2580_write_cmos_sensor(0x3e89, 0x10);	
		break;
		case AE_EV_COMP_07:
			//C2580_write_cmos_sensor(0x3e83, reg0x3e83 & 0xfe);
			C2580_write_cmos_sensor(0x3e89, 0x20);  
		break;
		case AE_EV_COMP_10:
			//C2580_write_cmos_sensor(0x3e83, reg0x3e83 & 0xfe);
			C2580_write_cmos_sensor(0x3e89, 0x30);
		break;
		case AE_EV_COMP_13:
			//C2580_write_cmos_sensor(0x3e83, reg0x3e83 & 0xfe);
			C2580_write_cmos_sensor(0x3e89, 0x4f);
		break;
		default:
		return FALSE;
	}
	return TRUE;
} /* C2580_set_param_exposure */
/*************************************************************************
* FUNCTION
*	C2580YUVSensorSetting
*
* DESCRIPTION
*	This function set the exposure.
*
* PARAMETERS
*	
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
UINT32 C2580YUVSensorSetting(FEATURE_ID iCmd, UINT32 iPara)
{
		switch (iCmd) {
		case FID_SCENE_MODE:	    
			//printk("Set Scene Mode:%d\n", iPara); 
			if (iPara == SCENE_MODE_OFF)
			{
			C2580_night_mode(KAL_FALSE); 
			}
			else if (iPara == SCENE_MODE_NIGHTSCENE)
			{
			C2580_night_mode(KAL_TRUE); 
			}	    
			break; 	    
		case FID_AWB_MODE:
			//printk("Set AWB Mode:%d\n", iPara); 	    
			C2580_set_param_wb(iPara);
			break;
		case FID_COLOR_EFFECT:
			//printk("Set Color Effect:%d\n", iPara); 	    	    
			C2580_set_param_effect(iPara);
			break;
		case FID_AE_EV:
			//printk("Set EV:%d\n", iPara); 	    	    
			C2580_set_param_exposure(iPara);
			break;
		case FID_AE_FLICKER:
			//printk("Set Flicker:%d\n", iPara); 	    	    	    
			C2580_set_param_banding(iPara);
			break;
		case FID_AE_SCENE_MODE: 
			if (iPara == AE_MODE_OFF) {
			    C2580_set_AE_mode(KAL_FALSE);
			}
			else {
			    C2580_set_AE_mode(KAL_TRUE);
			}
			break; 
		case FID_ZOOM_FACTOR:
			zoom_factor = iPara; 
			break; 
		default:
			break;
		}
		return TRUE;
}   /* C2580YUVSensorSetting */
/*************************************************************************
* FUNCTION
*	C2580YUVSetVideoMode
*
* DESCRIPTION
*	This function set the VideoMode.
*
* PARAMETERS
*	
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
UINT32 C2580YUVSetVideoMode(UINT16 u2FrameRate)
{
    kal_uint8 iTemp;
    //printk("Set YUV Video Mode \n");  
    if (u2FrameRate == 30)
    {
    ;
    }
    else if (u2FrameRate == 15)       
    {
    ;
    }
    else 
    {
     printk("Wrong frame rate setting \n");
    }
    C2580_VEDIO_encode_mode = KAL_TRUE; 
        
    return TRUE;
}
/*************************************************************************
* FUNCTION
*	C2580FeatureControl
*
* DESCRIPTION
*	This function get  the feature of control.
*
* PARAMETERS
*	
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
UINT32 C2580FeatureControl(MSDK_SENSOR_FEATURE_ENUM FeatureId,
							 UINT8 *pFeaturePara,UINT32 *pFeatureParaLen)
{
	UINT16 *pFeatureReturnPara16=(UINT16 *) pFeaturePara;
	UINT16 *pFeatureData16=(UINT16 *) pFeaturePara;
	UINT32 *pFeatureReturnPara32=(UINT32 *) pFeaturePara;
	UINT32 *pFeatureData32=(UINT32 *) pFeaturePara;
	MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData=(MSDK_SENSOR_CONFIG_STRUCT *) pFeaturePara;
	MSDK_SENSOR_REG_INFO_STRUCT *pSensorRegData=(MSDK_SENSOR_REG_INFO_STRUCT *) pFeaturePara;

	switch (FeatureId)
	{
		case SENSOR_FEATURE_GET_RESOLUTION:
			*pFeatureReturnPara16++=C2580_IMAGE_SENSOR_FULL_WIDTH;
			*pFeatureReturnPara16=C2580_IMAGE_SENSOR_FULL_HEIGHT;
			*pFeatureParaLen=4;
		break;
		case SENSOR_FEATURE_GET_PERIOD:
			*pFeatureReturnPara16++=C2580_PV_PERIOD_PIXEL_NUMS;
			*pFeatureReturnPara16=C2580_PV_PERIOD_LINE_NUMS;
			*pFeatureParaLen=4;
		break;
		case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
			*pFeatureReturnPara32 =24;
			*pFeatureParaLen=4;
		break;
		case SENSOR_FEATURE_SET_ESHUTTER:
		break;
		case SENSOR_FEATURE_SET_NIGHTMODE:
			C2580_night_mode((BOOL) *pFeatureData16);
		break;
		case SENSOR_FEATURE_SET_GAIN:
		break;
		case SENSOR_FEATURE_SET_FLASHLIGHT:
		break;
		case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
			C2580_isp_master_clock=*pFeatureData32;
		break;
		case SENSOR_FEATURE_SET_REGISTER:
			C2580_write_cmos_sensor(pSensorRegData->RegAddr, pSensorRegData->RegData);
		break;
		case SENSOR_FEATURE_GET_REGISTER:
			pSensorRegData->RegData = C2580_read_cmos_sensor(pSensorRegData->RegAddr);
		break;
		case SENSOR_FEATURE_GET_CONFIG_PARA:
			memcpy(pSensorConfigData, &C2580SensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
			*pFeatureParaLen=sizeof(MSDK_SENSOR_CONFIG_STRUCT);
		break;
		case SENSOR_FEATURE_SET_CCT_REGISTER:
		case SENSOR_FEATURE_GET_CCT_REGISTER:
		case SENSOR_FEATURE_SET_ENG_REGISTER:
		case SENSOR_FEATURE_GET_ENG_REGISTER:
		case SENSOR_FEATURE_GET_REGISTER_DEFAULT:

		case SENSOR_FEATURE_CAMERA_PARA_TO_SENSOR:
		case SENSOR_FEATURE_SENSOR_TO_CAMERA_PARA:
		case SENSOR_FEATURE_GET_GROUP_INFO:
		case SENSOR_FEATURE_GET_ITEM_INFO:
		case SENSOR_FEATURE_SET_ITEM_INFO:
		case SENSOR_FEATURE_GET_ENG_INFO:
		break;
		case SENSOR_FEATURE_GET_GROUP_COUNT:
            *pFeatureReturnPara32++=0;
            *pFeatureParaLen=4;	    
		    break; 
		case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
			// get the lens driver ID from EEPROM or just return LENS_DRIVER_ID_DO_NOT_CARE
			// if EEPROM does not exist in camera module.
			*pFeatureReturnPara32=LENS_DRIVER_ID_DO_NOT_CARE;
			*pFeatureParaLen=4;
		break;
		case SENSOR_FEATURE_CHECK_SENSOR_ID:  
			 C2580_GetSensorID(pFeatureData32);
			 break;
		case SENSOR_FEATURE_SET_YUV_CMD:
		    //printk("C2580 YUV sensor Setting:%d, %d \n", *pFeatureData32,  *(pFeatureData32+1));
			C2580YUVSensorSetting((FEATURE_ID)*pFeatureData32, *(pFeatureData32+1));
		break;
		case SENSOR_FEATURE_SET_VIDEO_MODE:
	       C2580YUVSetVideoMode(*pFeatureData16);
	       break; 
		default:
			break;			
	}
	return ERROR_NONE;
}	/* C2580FeatureControl() */
/*************************************************************************
* FUNCTION
*	SensorFuncC2580
*
* DESCRIPTION
*	This function set  SensorFuncC2580 structl.
*
* PARAMETERS
*	
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
SENSOR_FUNCTION_STRUCT	SensorFuncC2580= 
{
	C2580Open,
	C2580GetInfo,
	C2580GetResolution,
	C2580FeatureControl,
	C2580Control,
	C2580Close
};
/*************************************************************************
* FUNCTION
*	C2580_YUV_SensorInit
*
* DESCRIPTION
*	This function set  entry of sensor.
*
* PARAMETERS
*	
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
UINT32 C2580_YUV_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
	/* To Do : Check Sensor status here */
	if (pfFunc!=NULL)
	*pfFunc=&SensorFuncC2580;
	return ERROR_NONE;
}	/* SensorInit() */
