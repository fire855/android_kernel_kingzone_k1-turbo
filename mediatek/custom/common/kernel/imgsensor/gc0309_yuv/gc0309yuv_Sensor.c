/*****************************************************************************
 *  Copyright Statement:
 *  --------------------
 *  This software is protected by Copyright and the information contained
 *  herein is confidential. The software may not be copied and the information
 *  contained herein may not be used or disclosed except with the written
 *  permission of MediaTek Inc. (C) 2005
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
 *   GC0309yuv_Sensor.c
 *
 * Project:
 * --------
 *   MAUI
 *
 * Description:
 * ------------
 *   Image sensor driver function
 *   V1.0.0
 *
 * Author:
 * -------
 *   Mormo
 *
 *=============================================================
 *             HISTORY
 * Below this line, this part is controlled by PVCS VM. DO NOT MODIFY!!
 *------------------------------------------------------------------------------
 * $Log: GC0309yuv_Sensor.c,v $
 * Revision 1.1  2012/04/28 10:15:54  zhuyaozhong
 * ics projct
 *
 * Revision 1.1  2012/04/11 03:29:04  birdlibo
 * no message
 *
 * 2011/10/25 Firsty Released By Mormo(using "GC0309.set Revision1721" )
 *   
 *
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by PVCS VM. DO NOT MODIFY!!
 *=============================================================
 ******************************************************************************/
#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>

#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"
#include "kd_camera_feature.h"

#include "gc0309yuv_Sensor.h"
#include "gc0309yuv_Camera_Sensor_para.h"
#include "gc0309yuv_CameraCustomized.h"

//#define GC0309YUV_DEBUG
#ifdef GC0309YUV_DEBUG
#define SENSORDB printk
#else
#define SENSORDB(x,...)
#endif

//extern int iReadReg_Byte(u8 addr, u8 *buf, u8 i2cId);
//extern int iWriteReg_Byte(u8 addr, u8 buf, u32 size, u16 i2cId);
extern int iReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 * a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
extern int iWriteRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u16 i2cId);
/*
static void GC0309_write_cmos_sensor(addr, para)
{
    
    iWriteReg((u8)addr, (u8)para, 1, GC0309_WRITE_ID);
}

kal_uint8 GC0309_read_cmos_sensor(kal_uint32 addr)
{
    kal_uint8 get_byte = 0;
    iReadReg((u8)addr, &get_byte, GC0309_WRITE_ID);
    return get_byte;
}
*/
static void GC0309_write_cmos_sensor(addr, para)
{
    char puSendCmd[2] = {(char)(addr & 0xFF) , (char)(para & 0xFF)};
	
	iWriteRegI2C(puSendCmd , 2, GC0309_WRITE_ID);

}
kal_uint8 GC0309_read_cmos_sensor(kal_uint32 addr)
{
	kal_uint16 get_byte=0;
    char puSendCmd = { (char)(addr & 0xFF) };
	iReadRegI2C(&puSendCmd , 1, (u8*)&get_byte, 1, GC0309_WRITE_ID);
	
    return get_byte;
}

/*******************************************************************************
 * // Adapter for Winmo typedef
 ********************************************************************************/
#define WINMO_USE 0

#define Sleep(ms) mdelay(ms)
#define RETAILMSG(x,...)
#define TEXT

kal_bool   GC0309_MPEG4_encode_mode = KAL_FALSE;
kal_uint16 GC0309_dummy_pixels = 0, GC0309_dummy_lines = 0;
kal_bool   GC0309_MODE_CAPTURE = KAL_FALSE;
kal_bool   GC0309_CAM_BANDING_50HZ = KAL_FALSE;

kal_uint32 GC0309_isp_master_clock;
static kal_uint32 GC0309_g_fPV_PCLK = 26;

kal_uint8 GC0309_sensor_write_I2C_address = GC0309_WRITE_ID;
kal_uint8 GC0309_sensor_read_I2C_address = GC0309_READ_ID;

UINT8 GC0309PixelClockDivider=0;

MSDK_SENSOR_CONFIG_STRUCT GC0309SensorConfigData;

#define GC0309_SET_PAGE0 	GC0309_write_cmos_sensor(0xfe, 0x00)
#define GC0309_SET_PAGE1 	GC0309_write_cmos_sensor(0xfe, 0x01)


/*************************************************************************
 * FUNCTION
 *	GC0309_SetShutter
 *
 * DESCRIPTION
 *	This function set e-shutter of GC0309 to change exposure time.
 *
 * PARAMETERS
 *   iShutter : exposured lines
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
void GC0309_Set_Shutter(kal_uint16 iShutter)
{
} /* Set_GC0309_Shutter */


/*************************************************************************
 * FUNCTION
 *	GC0309_read_Shutter
 *
 * DESCRIPTION
 *	This function read e-shutter of GC0309 .
 *
 * PARAMETERS
 *  None
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
kal_uint16 GC0309_Read_Shutter(void)
{
    	kal_uint8 temp_reg1, temp_reg2;
	kal_uint16 shutter;

	temp_reg1 = GC0309_read_cmos_sensor(0x04);
	temp_reg2 = GC0309_read_cmos_sensor(0x03);

	shutter = (temp_reg1 & 0xFF) | (temp_reg2 << 8);

	return shutter;
} /* GC0309_read_shutter */


/*************************************************************************
 * FUNCTION
 *	GC0309_write_reg
 *
 * DESCRIPTION
 *	This function set the register of GC0309.
 *
 * PARAMETERS
 *	addr : the register index of GC0309
 *  para : setting parameter of the specified register of GC0309
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
void GC0309_write_reg(kal_uint32 addr, kal_uint32 para)
{
	GC0309_write_cmos_sensor(addr, para);
} /* GC0309_write_reg() */


/*************************************************************************
 * FUNCTION
 *	GC0309_read_cmos_sensor
 *
 * DESCRIPTION
 *	This function read parameter of specified register from GC0309.
 *
 * PARAMETERS
 *	addr : the register index of GC0309
 *
 * RETURNS
 *	the data that read from GC0309
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
kal_uint32 GC0309_read_reg(kal_uint32 addr)
{
	return GC0309_read_cmos_sensor(addr);
} /* OV7670_read_reg() */


/*************************************************************************
* FUNCTION
*	GC0309_awb_enable
*
* DESCRIPTION
*	This function enable or disable the awb (Auto White Balance).
*
* PARAMETERS
*	1. kal_bool : KAL_TRUE - enable awb, KAL_FALSE - disable awb.
*
* RETURNS
*	kal_bool : It means set awb right or not.
*
*************************************************************************/
static void GC0309_awb_enable(kal_bool enalbe)
{	 
	kal_uint16 temp_AWB_reg = 0;

	temp_AWB_reg = GC0309_read_cmos_sensor(0x22);
	
	if (enalbe)
	{
		GC0309_write_cmos_sensor(0x22, (temp_AWB_reg |0x02));
	}
	else
	{
		GC0309_write_cmos_sensor(0x22, (temp_AWB_reg & (~0x02)));
	}

}


/*************************************************************************
 * FUNCTION
 *	GC0309_config_window
 *
 * DESCRIPTION
 *	This function config the hardware window of GC0309 for getting specified
 *  data of that window.
 *
 * PARAMETERS
 *	start_x : start column of the interested window
 *  start_y : start row of the interested window
 *  width  : column widht of the itnerested window
 *  height : row depth of the itnerested window
 *
 * RETURNS
 *	the data that read from GC0309
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
void GC0309_config_window(kal_uint16 startx, kal_uint16 starty, kal_uint16 width, kal_uint16 height)
{
} /* GC0309_config_window */


/*************************************************************************
 * FUNCTION
 *	GC0309_SetGain
 *
 * DESCRIPTION
 *	This function is to set global gain to sensor.
 *
 * PARAMETERS
 *   iGain : sensor global gain(base: 0x40)
 *
 * RETURNS
 *	the actually gain set to sensor.
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
kal_uint16 GC0309_SetGain(kal_uint16 iGain)
{
	return iGain;
}


/*************************************************************************
 * FUNCTION
 *	GC0309_NightMode
 *
 * DESCRIPTION
 *	This function night mode of GC0309.
 *
 * PARAMETERS
 *	bEnable: KAL_TRUE -> enable night mode, otherwise, disable night mode
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
void GC0309_night_mode(kal_bool bEnable)
{
	if(bEnable)
	{
            GC0309_write_cmos_sensor(0xec ,0x30);	 //exp level 3                      
	}
	else
    	{
	    GC0309_write_cmos_sensor(0xec ,0x20);	 //exp level 2
	}
} /* GC0309_NightMode */


/*************************************************************************
* FUNCTION
*	GC0309_Sensor_Init
*
* DESCRIPTION
*	This function apply all of the initial setting to sensor.
*
* PARAMETERS
*	None
*
* RETURNS
*	None
*
*************************************************************************/
void GC0309_Sensor_Init(void)
{
	GC0309_write_cmos_sensor(0xfe,0x80);	 // soft reset	
		
	GC0309_SET_PAGE0;		// set page0
	
	GC0309_write_cmos_sensor(0x1a,0x16);		
	GC0309_write_cmos_sensor(0xd2,0x10);	 // close AEC
	GC0309_write_cmos_sensor(0x22,0x55);	 // close AWB

	GC0309_write_cmos_sensor(0x5a,0x56); 
	GC0309_write_cmos_sensor(0x5b,0x40);
	GC0309_write_cmos_sensor(0x5c,0x4a);			

	GC0309_write_cmos_sensor(0x22,0x57); 
		
	GC0309_write_cmos_sensor(0x01,0xfa); 
	GC0309_write_cmos_sensor(0x02,0x70); 
	GC0309_write_cmos_sensor(0x0f,0x01); 

	GC0309_write_cmos_sensor(0xe2,0x00); 
	GC0309_write_cmos_sensor(0xe3,0x64); 

	GC0309_write_cmos_sensor(0x03,0x01); 
	GC0309_write_cmos_sensor(0x04,0x2c); 

	/*
	GC0309_write_cmos_sensor(0x01,0x6a); 
	GC0309_write_cmos_sensor(0x02,0x25); 
	GC0309_write_cmos_sensor(0x0f,0x00);

	GC0309_write_cmos_sensor(0xe2,0x00); 
	GC0309_write_cmos_sensor(0xe3,0x4b); 
		
	GC0309_write_cmos_sensor(0xe4,0x02); 
	GC0309_write_cmos_sensor(0xe5,0x0d); 
	GC0309_write_cmos_sensor(0xe6,0x02); 
	GC0309_write_cmos_sensor(0xe7,0x0d); 
	GC0309_write_cmos_sensor(0xe8,0x02); 
	GC0309_write_cmos_sensor(0xe9,0x0d); 
	GC0309_write_cmos_sensor(0xea,0x05); 
	GC0309_write_cmos_sensor(0xeb,0xdc); 
	*/

	GC0309_write_cmos_sensor(0x05,0x00);
	GC0309_write_cmos_sensor(0x06,0x00);
	GC0309_write_cmos_sensor(0x07,0x00); 
	GC0309_write_cmos_sensor(0x08,0x00); 
	GC0309_write_cmos_sensor(0x09,0x01); 
	GC0309_write_cmos_sensor(0x0a,0xe8); 
	GC0309_write_cmos_sensor(0x0b,0x02); 
	GC0309_write_cmos_sensor(0x0c,0x88); 
	GC0309_write_cmos_sensor(0x0d,0x02); 
	GC0309_write_cmos_sensor(0x0e,0x02); 
	GC0309_write_cmos_sensor(0x10,0x22); 
	GC0309_write_cmos_sensor(0x11,0x0d); 
	GC0309_write_cmos_sensor(0x12,0x2a); 
	GC0309_write_cmos_sensor(0x13,0x00); 
	if(0 == strncmp(VANZO_SUB_CAM_ROTATION, "180", 3))
	GC0309_write_cmos_sensor(0x14,0x10);	//mirror
	else
	GC0309_write_cmos_sensor(0x14,0x13);	//mirror
	GC0309_write_cmos_sensor(0x15,0x0a); 
	GC0309_write_cmos_sensor(0x16,0x05); 
	GC0309_write_cmos_sensor(0x17,0x01); 

	GC0309_write_cmos_sensor(0x1b,0x03); 
	GC0309_write_cmos_sensor(0x1c,0xc1); 
	GC0309_write_cmos_sensor(0x1d,0x08); 
	GC0309_write_cmos_sensor(0x1e,0x20);
	GC0309_write_cmos_sensor(0x1f,0x16); 

	GC0309_write_cmos_sensor(0x20,0xff); 
	GC0309_write_cmos_sensor(0x21,0xf8); 
	GC0309_write_cmos_sensor(0x24,0xa0); //a2
	GC0309_write_cmos_sensor(0x25,0x0f);
	//output sync_mode
	GC0309_write_cmos_sensor(0x26,0x02); 
	GC0309_write_cmos_sensor(0x2f,0x01); 
	/////////////////////////////////////////////////////////////////////
	/////////////////////////// grab_t ////////////////////////////////
	/////////////////////////////////////////////////////////////////////
	GC0309_write_cmos_sensor(0x30,0xf7); 
	GC0309_write_cmos_sensor(0x31,0x40);
	GC0309_write_cmos_sensor(0x32,0x00); 
	GC0309_write_cmos_sensor(0x39,0x04); 
	GC0309_write_cmos_sensor(0x3a,0x20); 
	GC0309_write_cmos_sensor(0x3b,0x20); 
	GC0309_write_cmos_sensor(0x3c,0x02); 
	GC0309_write_cmos_sensor(0x3d,0x02); 
	GC0309_write_cmos_sensor(0x3e,0x02);
	GC0309_write_cmos_sensor(0x3f,0x02); 
	
	//gain
	GC0309_write_cmos_sensor(0x50,0x24); 
	
	GC0309_write_cmos_sensor(0x53,0x82); 
	GC0309_write_cmos_sensor(0x54,0x80); 
	GC0309_write_cmos_sensor(0x55,0x80); 
	GC0309_write_cmos_sensor(0x56,0x82); 
	
	/////////////////////////////////////////////////////////////////////
	/////////////////////////// LSC_t  ////////////////////////////////
	/////////////////////////////////////////////////////////////////////
	GC0309_write_cmos_sensor(0x8b,0x20); 
	GC0309_write_cmos_sensor(0x8c,0x20); 
	GC0309_write_cmos_sensor(0x8d,0x20); 
	GC0309_write_cmos_sensor(0x8e,0x10); 
	GC0309_write_cmos_sensor(0x8f,0x10); 
	GC0309_write_cmos_sensor(0x90,0x10); 
	GC0309_write_cmos_sensor(0x91,0x3c); 
	GC0309_write_cmos_sensor(0x92,0x50); 
	GC0309_write_cmos_sensor(0x5d,0x12); 
	GC0309_write_cmos_sensor(0x5e,0x1a); 
	GC0309_write_cmos_sensor(0x5f,0x24); 
	/////////////////////////////////////////////////////////////////////
	/////////////////////////// DNDD_t	///////////////////////////////
	/////////////////////////////////////////////////////////////////////
	GC0309_write_cmos_sensor(0x60,0x07); 
	GC0309_write_cmos_sensor(0x61,0x0e); 
	GC0309_write_cmos_sensor(0x62,0x0c); 
	GC0309_write_cmos_sensor(0x64,0x03); 
	GC0309_write_cmos_sensor(0x66,0xe8); 
	GC0309_write_cmos_sensor(0x67,0x86); 
	GC0309_write_cmos_sensor(0x68,0xa2); 
	
	/////////////////////////////////////////////////////////////////////
	/////////////////////////// asde_t ///////////////////////////////
	/////////////////////////////////////////////////////////////////////
	GC0309_write_cmos_sensor(0x69,0x20); 
	GC0309_write_cmos_sensor(0x6a,0x0f); 
	GC0309_write_cmos_sensor(0x6b,0x00); 
	GC0309_write_cmos_sensor(0x6c,0x53); 
	GC0309_write_cmos_sensor(0x6d,0x83); 
	GC0309_write_cmos_sensor(0x6e,0xac); 
	GC0309_write_cmos_sensor(0x6f,0xac); 
	GC0309_write_cmos_sensor(0x70,0x15); 
	GC0309_write_cmos_sensor(0x71,0x33); 
	/////////////////////////////////////////////////////////////////////
	/////////////////////////// eeintp_t///////////////////////////////
	/////////////////////////////////////////////////////////////////////
	GC0309_write_cmos_sensor(0x72,0xdc);	
	GC0309_write_cmos_sensor(0x73,0x80);	
	//for high resolution in light scene
	GC0309_write_cmos_sensor(0x74,0x02); 
	GC0309_write_cmos_sensor(0x75,0x3f); 
	GC0309_write_cmos_sensor(0x76,0x02); 
	GC0309_write_cmos_sensor(0x77,0x54); 
	GC0309_write_cmos_sensor(0x78,0x88); 
	GC0309_write_cmos_sensor(0x79,0x81); 
	GC0309_write_cmos_sensor(0x7a,0x81); 
	GC0309_write_cmos_sensor(0x7b,0x22); 
	GC0309_write_cmos_sensor(0x7c,0xff);
	
	
	/////////////////////////////////////////////////////////////////////
	///////////////////////////CC_t///////////////////////////////
	/////////////////////////////////////////////////////////////////////
	GC0309_write_cmos_sensor(0x93,0x45); 
	GC0309_write_cmos_sensor(0x94,0x00); 
	GC0309_write_cmos_sensor(0x95,0x00); 
	GC0309_write_cmos_sensor(0x96,0x00); 
	GC0309_write_cmos_sensor(0x97,0x45); 
	GC0309_write_cmos_sensor(0x98,0xf0); 
	GC0309_write_cmos_sensor(0x9c,0x00); 
	GC0309_write_cmos_sensor(0x9d,0x03); 
	GC0309_write_cmos_sensor(0x9e,0x00); 

	
	
	/////////////////////////////////////////////////////////////////////
	///////////////////////////YCP_t///////////////////////////////
	/////////////////////////////////////////////////////////////////////
	GC0309_write_cmos_sensor(0xb1,0x40); 
	GC0309_write_cmos_sensor(0xb2,0x40); 
	GC0309_write_cmos_sensor(0xb8,0x20); 
	GC0309_write_cmos_sensor(0xbe,0x36); 
	GC0309_write_cmos_sensor(0xbf,0x00); 
	/////////////////////////////////////////////////////////////////////
	///////////////////////////AEC_t///////////////////////////////
	/////////////////////////////////////////////////////////////////////
	GC0309_write_cmos_sensor(0xd0,0xc9);	
	GC0309_write_cmos_sensor(0xd1,0x10);	
//	GC0309_write_cmos_sensor(0xd2,0x90);	
	GC0309_write_cmos_sensor(0xd3,0x80);	
	GC0309_write_cmos_sensor(0xd5,0xf2); 
	GC0309_write_cmos_sensor(0xd6,0x16);	
	GC0309_write_cmos_sensor(0xdb,0x92); 
	GC0309_write_cmos_sensor(0xdc,0xa5);	
	GC0309_write_cmos_sensor(0xdf,0x23);	 
	GC0309_write_cmos_sensor(0xd9,0x00);	
	GC0309_write_cmos_sensor(0xda,0x00);	
	GC0309_write_cmos_sensor(0xe0,0x09);

	GC0309_write_cmos_sensor(0xec,0x00);	
	GC0309_write_cmos_sensor(0xed,0x04);	
	GC0309_write_cmos_sensor(0xee,0xa0);	
	GC0309_write_cmos_sensor(0xef,0x40);	
	///////////////////////////////////////////////////////////////////
	///////////////////////////GAMMA//////////////////////////////////
	///////////////////////////////////////////////////////////////////
#if 0	
	GC0309_write_cmos_sensor(0x9F,0x0F);			 
	GC0309_write_cmos_sensor(0xA0,0x1D);	
	GC0309_write_cmos_sensor(0xA1,0x2D);
	GC0309_write_cmos_sensor(0xA2,0x3B);
	GC0309_write_cmos_sensor(0xA3,0x46);
	GC0309_write_cmos_sensor(0xA4,0x50);
	GC0309_write_cmos_sensor(0xA5,0x5A);
	GC0309_write_cmos_sensor(0xA6,0x6B);
	GC0309_write_cmos_sensor(0xA7,0x7B);
	GC0309_write_cmos_sensor(0xA8,0x8A);
	GC0309_write_cmos_sensor(0xA9,0x98);
	GC0309_write_cmos_sensor(0xAA,0xA5);
	GC0309_write_cmos_sensor(0xAB,0xB2);
	GC0309_write_cmos_sensor(0xAC,0xBE);
	GC0309_write_cmos_sensor(0xAD,0xD5);
	GC0309_write_cmos_sensor(0xAE,0xEB);
	GC0309_write_cmos_sensor(0xAF,0xFE);
#endif
	//Y_gamma
	GC0309_write_cmos_sensor(0xc0,0x00);
	GC0309_write_cmos_sensor(0xc1,0x0B);
	GC0309_write_cmos_sensor(0xc2,0x15);
	GC0309_write_cmos_sensor(0xc3,0x27);
	GC0309_write_cmos_sensor(0xc4,0x39);
	GC0309_write_cmos_sensor(0xc5,0x49);
	GC0309_write_cmos_sensor(0xc6,0x5A);
	GC0309_write_cmos_sensor(0xc7,0x6A);
	GC0309_write_cmos_sensor(0xc8,0x89);
	GC0309_write_cmos_sensor(0xc9,0xA8);
	GC0309_write_cmos_sensor(0xca,0xC6);
	GC0309_write_cmos_sensor(0xcb,0xE3);
	GC0309_write_cmos_sensor(0xcc,0xFF);

	/////////////////////////////////////////////////////////////////
	/////////////////////////// ABS_t ///////////////////////////////
	/////////////////////////////////////////////////////////////////
	GC0309_write_cmos_sensor(0xf0,0x02);
	GC0309_write_cmos_sensor(0xf1,0x01);
	GC0309_write_cmos_sensor(0xf2,0x00); 
	GC0309_write_cmos_sensor(0xf3,0x30); 
	
	/////////////////////////////////////////////////////////////////
	/////////////////////////// Measure Window ///////////////////////
	/////////////////////////////////////////////////////////////////
	GC0309_write_cmos_sensor(0xf7,0x04); 
	GC0309_write_cmos_sensor(0xf8,0x02); 
	GC0309_write_cmos_sensor(0xf9,0x9f);
	GC0309_write_cmos_sensor(0xfa,0x78);

	//---------------------------------------------------------------
	GC0309_SET_PAGE1;

	/////////////////////////////////////////////////////////////////
	///////////////////////////AWB_p/////////////////////////////////
	/////////////////////////////////////////////////////////////////
	GC0309_write_cmos_sensor(0x00,0xf5); 
	//GC0309_write_cmos_sensor(0x01,0x0a);  
	GC0309_write_cmos_sensor(0x02,0x1a); 
	GC0309_write_cmos_sensor(0x0a,0xa0); 
	GC0309_write_cmos_sensor(0x0b,0x60); 
	GC0309_write_cmos_sensor(0x0c,0x08);
	GC0309_write_cmos_sensor(0x0e,0x4c); 
	GC0309_write_cmos_sensor(0x0f,0x39); 
	GC0309_write_cmos_sensor(0x11,0x3f); 
	GC0309_write_cmos_sensor(0x12,0x72); 
	GC0309_write_cmos_sensor(0x13,0x13); 
	GC0309_write_cmos_sensor(0x14,0x42);	
	GC0309_write_cmos_sensor(0x15,0x43); 
	GC0309_write_cmos_sensor(0x16,0xc2); 
	GC0309_write_cmos_sensor(0x17,0xa8); 
	GC0309_write_cmos_sensor(0x18,0x18);	
	GC0309_write_cmos_sensor(0x19,0x40);	
	GC0309_write_cmos_sensor(0x1a,0xd0); 
	GC0309_write_cmos_sensor(0x1b,0xf5);	

	GC0309_write_cmos_sensor(0x70,0x40); 
	GC0309_write_cmos_sensor(0x71,0x58);	
	GC0309_write_cmos_sensor(0x72,0x30);	
	GC0309_write_cmos_sensor(0x73,0x48);	
	GC0309_write_cmos_sensor(0x74,0x20);	
	GC0309_write_cmos_sensor(0x75,0x60);	
	
	GC0309_SET_PAGE0;

	GC0309_write_cmos_sensor(0xd2,0x90);  // Open AEC at last.  

}


/*************************************************************************
* FUNCTION
*	GC329_Lens_Select
*
* DESCRIPTION
*	This function is served for FAE to select the appropriate lens parameter.
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
#if 0
void GC0309_Lens_Select(kal_uint8 Lens_Tag)
{
	switch(Lens_Tag)
	{
		case CHT_806C_2:
			GC0309_write_cmos_sensor(0xfe, 0x01);
			GC0309_write_cmos_sensor(0xa0, 0x00);
			GC0309_write_cmos_sensor(0xa1, 0x3c);
			GC0309_write_cmos_sensor(0xa2, 0x50);
			GC0309_write_cmos_sensor(0xa3, 0x00);
			GC0309_write_cmos_sensor(0xa4, 0x00);
			GC0309_write_cmos_sensor(0xa5, 0x00);
			GC0309_write_cmos_sensor(0xa6, 0x00);
			GC0309_write_cmos_sensor(0xa7, 0x04);
			
			GC0309_write_cmos_sensor(0xa8, 0x0f);
			GC0309_write_cmos_sensor(0xa9, 0x08);
			GC0309_write_cmos_sensor(0xaa, 0x00);
			GC0309_write_cmos_sensor(0xab, 0x04);
			GC0309_write_cmos_sensor(0xac, 0x00);
			GC0309_write_cmos_sensor(0xad, 0x07);
			GC0309_write_cmos_sensor(0xae, 0x0e);
			GC0309_write_cmos_sensor(0xaf, 0x00);
			GC0309_write_cmos_sensor(0xb0, 0x00);
			GC0309_write_cmos_sensor(0xb1, 0x09);
			GC0309_write_cmos_sensor(0xb2, 0x00);
			GC0309_write_cmos_sensor(0xb3, 0x00);

			GC0309_write_cmos_sensor(0xb4, 0x30);
			GC0309_write_cmos_sensor(0xb5, 0x19);
			GC0309_write_cmos_sensor(0xb6, 0x21);
			GC0309_write_cmos_sensor(0xba, 0x3e);
			GC0309_write_cmos_sensor(0xbb, 0x26);
			GC0309_write_cmos_sensor(0xbc, 0x2f);
			GC0309_write_cmos_sensor(0xc0, 0x15);
			GC0309_write_cmos_sensor(0xc1, 0x11);
			GC0309_write_cmos_sensor(0xc2, 0x15);
			GC0309_write_cmos_sensor(0xc6, 0x1f);
			GC0309_write_cmos_sensor(0xc7, 0x16);
			GC0309_write_cmos_sensor(0xc8, 0x16);

			GC0309_write_cmos_sensor(0xb7, 0x00);
			GC0309_write_cmos_sensor(0xb8, 0x00);
			GC0309_write_cmos_sensor(0xb9, 0x00);
			GC0309_write_cmos_sensor(0xbd, 0x00);
			GC0309_write_cmos_sensor(0xbe, 0x00);
			GC0309_write_cmos_sensor(0xbf, 0x00);
			GC0309_write_cmos_sensor(0xc3, 0x00);
			GC0309_write_cmos_sensor(0xc4, 0x00);
			GC0309_write_cmos_sensor(0xc5, 0x00);
			GC0309_write_cmos_sensor(0xc9, 0x0d);
			GC0309_write_cmos_sensor(0xca, 0x00);
			GC0309_write_cmos_sensor(0xcb, 0x00);
			
			GC0309_write_cmos_sensor(0xfe, 0x00);
			break;

		case CHT_808C_2:
			GC0309_write_cmos_sensor(0xfe, 0x01);
			GC0309_write_cmos_sensor(0xa0, 0x00);
			GC0309_write_cmos_sensor(0xa1, 0x3c);
			GC0309_write_cmos_sensor(0xa2, 0x50);
			GC0309_write_cmos_sensor(0xa3, 0x00);
			GC0309_write_cmos_sensor(0xa4, 0x00);
			GC0309_write_cmos_sensor(0xa5, 0x02);
			GC0309_write_cmos_sensor(0xa6, 0x00);
			GC0309_write_cmos_sensor(0xa7, 0x00);

			GC0309_write_cmos_sensor(0xa8, 0x0c);
			GC0309_write_cmos_sensor(0xa9, 0x03);
			GC0309_write_cmos_sensor(0xaa, 0x00);
			GC0309_write_cmos_sensor(0xab, 0x05);
			GC0309_write_cmos_sensor(0xac, 0x01);
			GC0309_write_cmos_sensor(0xad, 0x07);
			GC0309_write_cmos_sensor(0xae, 0x0e);
			GC0309_write_cmos_sensor(0xaf, 0x00);
			GC0309_write_cmos_sensor(0xb0, 0x00);
			GC0309_write_cmos_sensor(0xb1, 0x08);
			GC0309_write_cmos_sensor(0xb2, 0x02);
			GC0309_write_cmos_sensor(0xb3, 0x00);

			GC0309_write_cmos_sensor(0xb4, 0x30);
			GC0309_write_cmos_sensor(0xb5, 0x0f);
			GC0309_write_cmos_sensor(0xb6, 0x16);
			GC0309_write_cmos_sensor(0xba, 0x44);
			GC0309_write_cmos_sensor(0xbb, 0x24);
			GC0309_write_cmos_sensor(0xbc, 0x2a);
			GC0309_write_cmos_sensor(0xc0, 0x13);
			GC0309_write_cmos_sensor(0xc1, 0x0e);
			GC0309_write_cmos_sensor(0xc2, 0x11);
			GC0309_write_cmos_sensor(0xc6, 0x28);
			GC0309_write_cmos_sensor(0xc7, 0x21);
			GC0309_write_cmos_sensor(0xc8, 0x20);

			GC0309_write_cmos_sensor(0xb7, 0x00);
			GC0309_write_cmos_sensor(0xb8, 0x00);
			GC0309_write_cmos_sensor(0xb9, 0x01);
			GC0309_write_cmos_sensor(0xbd, 0x00);
			GC0309_write_cmos_sensor(0xbe, 0x00);
			GC0309_write_cmos_sensor(0xbf, 0x00);
			GC0309_write_cmos_sensor(0xc3, 0x00);
			GC0309_write_cmos_sensor(0xc4, 0x00);
			GC0309_write_cmos_sensor(0xc5, 0x00);
			GC0309_write_cmos_sensor(0xc9, 0x00);
			GC0309_write_cmos_sensor(0xca, 0x00);
			GC0309_write_cmos_sensor(0xcb, 0x00);

			GC0309_write_cmos_sensor(0xfe, 0x00);
			break;
			
		case LY_982A_H114:
			GC0309_write_cmos_sensor(0xfe, 0x01);
			GC0309_write_cmos_sensor(0xa0, 0x00);
			GC0309_write_cmos_sensor(0xa1, 0x3c);
			GC0309_write_cmos_sensor(0xa2, 0x50);
			GC0309_write_cmos_sensor(0xa3, 0x00);
			GC0309_write_cmos_sensor(0xa4, 0x00);
			GC0309_write_cmos_sensor(0xa5, 0x00);
			GC0309_write_cmos_sensor(0xa6, 0x00);
			GC0309_write_cmos_sensor(0xa7, 0x00);

			GC0309_write_cmos_sensor(0xa8, 0x0c);
			GC0309_write_cmos_sensor(0xa9, 0x06);
			GC0309_write_cmos_sensor(0xaa, 0x02);
			GC0309_write_cmos_sensor(0xab, 0x13);
			GC0309_write_cmos_sensor(0xac, 0x06);
			GC0309_write_cmos_sensor(0xad, 0x05);
			GC0309_write_cmos_sensor(0xae, 0x0b);
			GC0309_write_cmos_sensor(0xaf, 0x03);
			GC0309_write_cmos_sensor(0xb0, 0x00);
			GC0309_write_cmos_sensor(0xb1, 0x08);
			GC0309_write_cmos_sensor(0xb2, 0x01);
			GC0309_write_cmos_sensor(0xb3, 0x00);

			GC0309_write_cmos_sensor(0xb4, 0x34);
			GC0309_write_cmos_sensor(0xb5, 0x29);
			GC0309_write_cmos_sensor(0xb6, 0x2e);
			GC0309_write_cmos_sensor(0xba, 0x30);
			GC0309_write_cmos_sensor(0xbb, 0x24);
			GC0309_write_cmos_sensor(0xbc, 0x28);
			GC0309_write_cmos_sensor(0xc0, 0x1c);
			GC0309_write_cmos_sensor(0xc1, 0x19);
			GC0309_write_cmos_sensor(0xc2, 0x19);
			GC0309_write_cmos_sensor(0xc6, 0x1a);
			GC0309_write_cmos_sensor(0xc7, 0x19);
			GC0309_write_cmos_sensor(0xc8, 0x1b);

			GC0309_write_cmos_sensor(0xb7, 0x01);
			GC0309_write_cmos_sensor(0xb8, 0x01);
			GC0309_write_cmos_sensor(0xb9, 0x00);
			GC0309_write_cmos_sensor(0xbd, 0x00);
			GC0309_write_cmos_sensor(0xbe, 0x00);
			GC0309_write_cmos_sensor(0xbf, 0x00);
			GC0309_write_cmos_sensor(0xc3, 0x00);
			GC0309_write_cmos_sensor(0xc4, 0x00);
			GC0309_write_cmos_sensor(0xc5, 0x03);
			GC0309_write_cmos_sensor(0xc9, 0x00);
			GC0309_write_cmos_sensor(0xca, 0x00);
			GC0309_write_cmos_sensor(0xcb, 0x00);

			GC0309_write_cmos_sensor(0xfe, 0x00);
			break;

		case XY_046A:
			GC0309_write_cmos_sensor(0xfe, 0x01);
			GC0309_write_cmos_sensor(0xa0, 0x00);
			GC0309_write_cmos_sensor(0xa1, 0x3c);
			GC0309_write_cmos_sensor(0xa2, 0x50);
			GC0309_write_cmos_sensor(0xa3, 0x00);
			GC0309_write_cmos_sensor(0xa4, 0x00);
			GC0309_write_cmos_sensor(0xa5, 0x00);
			GC0309_write_cmos_sensor(0xa6, 0x10);
			GC0309_write_cmos_sensor(0xa7, 0x00);

			GC0309_write_cmos_sensor(0xa8, 0x11);
			GC0309_write_cmos_sensor(0xa9, 0x0a);
			GC0309_write_cmos_sensor(0xaa, 0x05);
			GC0309_write_cmos_sensor(0xab, 0x04);
			GC0309_write_cmos_sensor(0xac, 0x03);
			GC0309_write_cmos_sensor(0xad, 0x00);
			GC0309_write_cmos_sensor(0xae, 0x08);
			GC0309_write_cmos_sensor(0xaf, 0x01);
			GC0309_write_cmos_sensor(0xb0, 0x00);
			GC0309_write_cmos_sensor(0xb1, 0x09);
			GC0309_write_cmos_sensor(0xb2, 0x02);
			GC0309_write_cmos_sensor(0xb3, 0x03);

			GC0309_write_cmos_sensor(0xb4, 0x2e);
			GC0309_write_cmos_sensor(0xb5, 0x16);
			GC0309_write_cmos_sensor(0xb6, 0x24);
			GC0309_write_cmos_sensor(0xba, 0x3a);
			GC0309_write_cmos_sensor(0xbb, 0x1e);
			GC0309_write_cmos_sensor(0xbc, 0x24);
			GC0309_write_cmos_sensor(0xc0, 0x09);
			GC0309_write_cmos_sensor(0xc1, 0x02);
			GC0309_write_cmos_sensor(0xc2, 0x06);
			GC0309_write_cmos_sensor(0xc6, 0x25);
			GC0309_write_cmos_sensor(0xc7, 0x21);
			GC0309_write_cmos_sensor(0xc8, 0x23);

			GC0309_write_cmos_sensor(0xb7, 0x00);
			GC0309_write_cmos_sensor(0xb8, 0x00);
			GC0309_write_cmos_sensor(0xb9, 0x0f);
			GC0309_write_cmos_sensor(0xbd, 0x00);
			GC0309_write_cmos_sensor(0xbe, 0x00);
			GC0309_write_cmos_sensor(0xbf, 0x00);
			GC0309_write_cmos_sensor(0xc3, 0x00);
			GC0309_write_cmos_sensor(0xc4, 0x00);
			GC0309_write_cmos_sensor(0xc5, 0x00);
			GC0309_write_cmos_sensor(0xc9, 0x00);
			GC0309_write_cmos_sensor(0xca, 0x00);
			GC0309_write_cmos_sensor(0xcb, 0x00);

			GC0309_write_cmos_sensor(0xfe, 0x00);
			break;

		case XY_0620:
			GC0309_write_cmos_sensor(0xfe, 0x01);
			GC0309_write_cmos_sensor(0xa0, 0x00);
			GC0309_write_cmos_sensor(0xa1, 0x3c);
			GC0309_write_cmos_sensor(0xa2, 0x50);
			GC0309_write_cmos_sensor(0xa3, 0x00);
			GC0309_write_cmos_sensor(0xa4, 0x00);
			GC0309_write_cmos_sensor(0xa5, 0x00);
			GC0309_write_cmos_sensor(0xa6, 0x00);
			GC0309_write_cmos_sensor(0xa7, 0x00);

			GC0309_write_cmos_sensor(0xa8, 0x0f);
			GC0309_write_cmos_sensor(0xa9, 0x06);
			GC0309_write_cmos_sensor(0xaa, 0x00);
			GC0309_write_cmos_sensor(0xab, 0x07);
			GC0309_write_cmos_sensor(0xac, 0x05);
			GC0309_write_cmos_sensor(0xad, 0x08);
			GC0309_write_cmos_sensor(0xae, 0x13);
			GC0309_write_cmos_sensor(0xaf, 0x06);
			GC0309_write_cmos_sensor(0xb0, 0x00);
			GC0309_write_cmos_sensor(0xb1, 0x06);
			GC0309_write_cmos_sensor(0xb2, 0x01);
			GC0309_write_cmos_sensor(0xb3, 0x04);

			GC0309_write_cmos_sensor(0xb4, 0x2d);
			GC0309_write_cmos_sensor(0xb5, 0x18);
			GC0309_write_cmos_sensor(0xb6, 0x22);
			GC0309_write_cmos_sensor(0xba, 0x45);
			GC0309_write_cmos_sensor(0xbb, 0x2d);
			GC0309_write_cmos_sensor(0xbc, 0x34);
			GC0309_write_cmos_sensor(0xc0, 0x16);
			GC0309_write_cmos_sensor(0xc1, 0x13);
			GC0309_write_cmos_sensor(0xc2, 0x19);
			GC0309_write_cmos_sensor(0xc6, 0x21);
			GC0309_write_cmos_sensor(0xc7, 0x1c);
			GC0309_write_cmos_sensor(0xc8, 0x18);

			GC0309_write_cmos_sensor(0xb7, 0x00);
			GC0309_write_cmos_sensor(0xb8, 0x00);
			GC0309_write_cmos_sensor(0xb9, 0x00);
			GC0309_write_cmos_sensor(0xbd, 0x00);
			GC0309_write_cmos_sensor(0xbe, 0x00);
			GC0309_write_cmos_sensor(0xbf, 0x08);
			GC0309_write_cmos_sensor(0xc3, 0x00);
			GC0309_write_cmos_sensor(0xc4, 0x00);
			GC0309_write_cmos_sensor(0xc5, 0x01);
			GC0309_write_cmos_sensor(0xc9, 0x00);
			GC0309_write_cmos_sensor(0xca, 0x00);
			GC0309_write_cmos_sensor(0xcb, 0x10);

			GC0309_write_cmos_sensor(0xfe, 0x00);
			break;

		case XY_078V: 
			GC0309_write_cmos_sensor(0xfe, 0x01);
			GC0309_write_cmos_sensor(0xa0, 0x00);
			GC0309_write_cmos_sensor(0xa1, 0x3c);
			GC0309_write_cmos_sensor(0xa2, 0x50);
			GC0309_write_cmos_sensor(0xa3, 0x00);
			GC0309_write_cmos_sensor(0xa4, 0x00);
			GC0309_write_cmos_sensor(0xa5, 0x00);
			GC0309_write_cmos_sensor(0xa6, 0x00);
			GC0309_write_cmos_sensor(0xa7, 0x00);

			GC0309_write_cmos_sensor(0xa8, 0x14);
			GC0309_write_cmos_sensor(0xa9, 0x08);
			GC0309_write_cmos_sensor(0xaa, 0x0a);
			GC0309_write_cmos_sensor(0xab, 0x11);
			GC0309_write_cmos_sensor(0xac, 0x05);
			GC0309_write_cmos_sensor(0xad, 0x07);
			GC0309_write_cmos_sensor(0xae, 0x0b);
			GC0309_write_cmos_sensor(0xaf, 0x03);
			GC0309_write_cmos_sensor(0xb0, 0x00);
			GC0309_write_cmos_sensor(0xb1, 0x09);
			GC0309_write_cmos_sensor(0xb2, 0x04);
			GC0309_write_cmos_sensor(0xb3, 0x01);

			GC0309_write_cmos_sensor(0xb4, 0x2f);
			GC0309_write_cmos_sensor(0xb5, 0x2a);
			GC0309_write_cmos_sensor(0xb6, 0x2c);
			GC0309_write_cmos_sensor(0xba, 0x3a);
			GC0309_write_cmos_sensor(0xbb, 0x2b);
			GC0309_write_cmos_sensor(0xbc, 0x32);
			GC0309_write_cmos_sensor(0xc0, 0x1b);
			GC0309_write_cmos_sensor(0xc1, 0x18);
			GC0309_write_cmos_sensor(0xc2, 0x1a);
			GC0309_write_cmos_sensor(0xc6, 0x12);
			GC0309_write_cmos_sensor(0xc7, 0x10);
			GC0309_write_cmos_sensor(0xc8, 0x12);

			GC0309_write_cmos_sensor(0xb7, 0x0a);
			GC0309_write_cmos_sensor(0xb8, 0x00);
			GC0309_write_cmos_sensor(0xb9, 0x00);
			GC0309_write_cmos_sensor(0xbd, 0x00);
			GC0309_write_cmos_sensor(0xbe, 0x00);
			GC0309_write_cmos_sensor(0xbf, 0x00);
			GC0309_write_cmos_sensor(0xc3, 0x00);
			GC0309_write_cmos_sensor(0xc4, 0x00);
			GC0309_write_cmos_sensor(0xc5, 0x00);
			GC0309_write_cmos_sensor(0xc9, 0x0d);
			GC0309_write_cmos_sensor(0xca, 0x00);
			GC0309_write_cmos_sensor(0xcb, 0x00);

			GC0309_write_cmos_sensor(0xfe, 0x00);
			break;

		case YG1001A_F:
			GC0309_write_cmos_sensor(0xfe, 0x01);
			GC0309_write_cmos_sensor(0xa0, 0x00);
			GC0309_write_cmos_sensor(0xa1, 0x3c);
			GC0309_write_cmos_sensor(0xa2, 0x50);
			GC0309_write_cmos_sensor(0xa3, 0x00);
			GC0309_write_cmos_sensor(0xa4, 0x00);
			GC0309_write_cmos_sensor(0xa5, 0x00);
			GC0309_write_cmos_sensor(0xa6, 0x00);
			GC0309_write_cmos_sensor(0xa7, 0x00);

			GC0309_write_cmos_sensor(0xa8, 0x0e);
			GC0309_write_cmos_sensor(0xa9, 0x05);
			GC0309_write_cmos_sensor(0xaa, 0x01);
			GC0309_write_cmos_sensor(0xab, 0x07);
			GC0309_write_cmos_sensor(0xac, 0x00);
			GC0309_write_cmos_sensor(0xad, 0x07);
			GC0309_write_cmos_sensor(0xae, 0x0e);
			GC0309_write_cmos_sensor(0xaf, 0x02);
			GC0309_write_cmos_sensor(0xb0, 0x00);
			GC0309_write_cmos_sensor(0xb1, 0x0d);
			GC0309_write_cmos_sensor(0xb2, 0x00);
			GC0309_write_cmos_sensor(0xb3, 0x00);

			GC0309_write_cmos_sensor(0xb4, 0x2a);
			GC0309_write_cmos_sensor(0xb5, 0x0f);
			GC0309_write_cmos_sensor(0xb6, 0x14);
			GC0309_write_cmos_sensor(0xba, 0x40);
			GC0309_write_cmos_sensor(0xbb, 0x26);
			GC0309_write_cmos_sensor(0xbc, 0x2a);
			GC0309_write_cmos_sensor(0xc0, 0x0e);
			GC0309_write_cmos_sensor(0xc1, 0x0a);
			GC0309_write_cmos_sensor(0xc2, 0x0d);
			GC0309_write_cmos_sensor(0xc6, 0x27);
			GC0309_write_cmos_sensor(0xc7, 0x20);
			GC0309_write_cmos_sensor(0xc8, 0x1f);

			GC0309_write_cmos_sensor(0xb7, 0x00);
			GC0309_write_cmos_sensor(0xb8, 0x00);
			GC0309_write_cmos_sensor(0xb9, 0x00);
			GC0309_write_cmos_sensor(0xbd, 0x00);
			GC0309_write_cmos_sensor(0xbe, 0x00);
			GC0309_write_cmos_sensor(0xbf, 0x00);
			GC0309_write_cmos_sensor(0xc3, 0x00);
			GC0309_write_cmos_sensor(0xc4, 0x00);
			GC0309_write_cmos_sensor(0xc5, 0x00);
			GC0309_write_cmos_sensor(0xc9, 0x00);
			GC0309_write_cmos_sensor(0xca, 0x00);
			GC0309_write_cmos_sensor(0xcb, 0x00);

			GC0309_write_cmos_sensor(0xfe, 0x00);
			break;

		default:
			break;
	}
}
#endif
/*************************************************************************
* FUNCTION
*	GC0309_GAMMA_Select
*
* DESCRIPTION
*	This function is served for FAE to select the appropriate GAMMA curve.
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
extern bool camera_pdn1_reverse;
UINT32 GC0309GetSensorID(UINT32 *sensorID)
{
    kal_uint16 sensor_id=0;
    int i;
	mt_set_gpio_mode(GPIO_CAMERA_CMPDN1_PIN, GPIO_CAMERA_CMPDN1_PIN_M_GPIO);
	mt_set_gpio_out(GPIO_CAMERA_CMPDN1_PIN, GPIO_OUT_ZERO);
    do
    {
        	// check if sensor ID correct
        	for(i = 0; i < 3; i++)
		{
	            	sensor_id = GC0309_read_cmos_sensor(0x00);
	            	printk("GC0309 Sensor id = %x\n", sensor_id);
	            	if (sensor_id == GC0309_SENSOR_ID)
			{
	               	break;
	            	}
        	}
        	mdelay(50);
    }while(0);

    if(sensor_id != GC0309_SENSOR_ID)
    {
        SENSORDB("GC0309 Sensor id read failed, ID = %x\n", sensor_id);
        return ERROR_SENSOR_CONNECT_FAIL;
    }

    *sensorID = sensor_id;
	camera_pdn1_reverse = 1;
    RETAILMSG(1, (TEXT("Sensor Read ID OK \r\n")));
    return ERROR_NONE;
}

void GC0309_GAMMA_Select(kal_uint32 GammaLvl)
{
	switch(GammaLvl)
	{
		case RGB_Gamma_m2:                                             //smallest gamma curve
			GC0309_write_cmos_sensor(0xBF, 0x08);
			GC0309_write_cmos_sensor(0xc0, 0x0F);
			GC0309_write_cmos_sensor(0xc1, 0x21);
			GC0309_write_cmos_sensor(0xc2, 0x32);
			GC0309_write_cmos_sensor(0xc3, 0x43);
			GC0309_write_cmos_sensor(0xc4, 0x50);
			GC0309_write_cmos_sensor(0xc5, 0x5E);
			GC0309_write_cmos_sensor(0xc6, 0x78);
			GC0309_write_cmos_sensor(0xc7, 0x90);
			GC0309_write_cmos_sensor(0xc8, 0xA6);
			GC0309_write_cmos_sensor(0xc9, 0xB9);
			GC0309_write_cmos_sensor(0xcA, 0xC9);
			GC0309_write_cmos_sensor(0xcB, 0xD6);
			GC0309_write_cmos_sensor(0xcC, 0xE0);
			GC0309_write_cmos_sensor(0xcD, 0xEE);
			GC0309_write_cmos_sensor(0xcE, 0xF8);
			GC0309_write_cmos_sensor(0xcF, 0xFF);
			break;
			
		case RGB_Gamma_m3:			
			GC0309_write_cmos_sensor(0xBF, 0x0B);
			GC0309_write_cmos_sensor(0xc0, 0x16);
			GC0309_write_cmos_sensor(0xc1, 0x29);
			GC0309_write_cmos_sensor(0xc2, 0x3C);
			GC0309_write_cmos_sensor(0xc3, 0x4F);
			GC0309_write_cmos_sensor(0xc4, 0x5F);
			GC0309_write_cmos_sensor(0xc5, 0x6F);
			GC0309_write_cmos_sensor(0xc6, 0x8A);
			GC0309_write_cmos_sensor(0xc7, 0x9F);
			GC0309_write_cmos_sensor(0xc8, 0xB4);
			GC0309_write_cmos_sensor(0xc9, 0xC6);
			GC0309_write_cmos_sensor(0xcA, 0xD3);
			GC0309_write_cmos_sensor(0xcB, 0xDD);
			GC0309_write_cmos_sensor(0xcC, 0xE5);
			GC0309_write_cmos_sensor(0xcD, 0xF1);
			GC0309_write_cmos_sensor(0xcE, 0xFA);
			GC0309_write_cmos_sensor(0xcF, 0xFF);
			break;
			
		case RGB_Gamma_m4:
			GC0309_write_cmos_sensor(0xBF, 0x0E);
			GC0309_write_cmos_sensor(0xc0, 0x1C);
			GC0309_write_cmos_sensor(0xc1, 0x34);
			GC0309_write_cmos_sensor(0xc2, 0x48);
			GC0309_write_cmos_sensor(0xc3, 0x5A);
			GC0309_write_cmos_sensor(0xc4, 0x6B);
			GC0309_write_cmos_sensor(0xc5, 0x7B);
			GC0309_write_cmos_sensor(0xc6, 0x95);
			GC0309_write_cmos_sensor(0xc7, 0xAB);
			GC0309_write_cmos_sensor(0xc8, 0xBF);
			GC0309_write_cmos_sensor(0xc9, 0xCE);
			GC0309_write_cmos_sensor(0xcA, 0xD9);
			GC0309_write_cmos_sensor(0xcB, 0xE4);
			GC0309_write_cmos_sensor(0xcC, 0xEC);
			GC0309_write_cmos_sensor(0xcD, 0xF7);
			GC0309_write_cmos_sensor(0xcE, 0xFD);
			GC0309_write_cmos_sensor(0xcF, 0xFF);
			break;
			
		case RGB_Gamma_m5:
			GC0309_write_cmos_sensor(0xBF, 0x10);
			GC0309_write_cmos_sensor(0xc0, 0x20);
			GC0309_write_cmos_sensor(0xc1, 0x38);
			GC0309_write_cmos_sensor(0xc2, 0x4E);
			GC0309_write_cmos_sensor(0xc3, 0x63);
			GC0309_write_cmos_sensor(0xc4, 0x76);
			GC0309_write_cmos_sensor(0xc5, 0x87);
			GC0309_write_cmos_sensor(0xc6, 0xA2);
			GC0309_write_cmos_sensor(0xc7, 0xB8);
			GC0309_write_cmos_sensor(0xc8, 0xCA);
			GC0309_write_cmos_sensor(0xc9, 0xD8);
			GC0309_write_cmos_sensor(0xcA, 0xE3);
			GC0309_write_cmos_sensor(0xcB, 0xEB);
			GC0309_write_cmos_sensor(0xcC, 0xF0);
			GC0309_write_cmos_sensor(0xcD, 0xF8);
			GC0309_write_cmos_sensor(0xcE, 0xFD);
			GC0309_write_cmos_sensor(0xcF, 0xFF);
			break;
			
		case RGB_Gamma_m6:
			GC0309_write_cmos_sensor(0xBF, 0x14);
			GC0309_write_cmos_sensor(0xc0, 0x28);
			GC0309_write_cmos_sensor(0xc1, 0x44);
			GC0309_write_cmos_sensor(0xc2, 0x5D);
			GC0309_write_cmos_sensor(0xc3, 0x72);
			GC0309_write_cmos_sensor(0xc4, 0x86);
			GC0309_write_cmos_sensor(0xc5, 0x95);
			GC0309_write_cmos_sensor(0xc6, 0xB1);
			GC0309_write_cmos_sensor(0xc7, 0xC6);
			GC0309_write_cmos_sensor(0xc8, 0xD5);
			GC0309_write_cmos_sensor(0xc9, 0xE1);
			GC0309_write_cmos_sensor(0xcA, 0xEA);
			GC0309_write_cmos_sensor(0xcB, 0xF1);
			GC0309_write_cmos_sensor(0xcC, 0xF5);
			GC0309_write_cmos_sensor(0xcD, 0xFB);
			GC0309_write_cmos_sensor(0xcE, 0xFE);
			GC0309_write_cmos_sensor(0xcF, 0xFF);							// largest gamma curve
			break;
			
		default:
			break;
	}
}


/*************************************************************************
* FUNCTION
*	GC0309_Write_More_Registers
*
* DESCRIPTION
*	This function is served for FAE to modify the necessary Init Regs. Do not modify the regs
*     in init_GC0309() directly.
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
void GC0309_Write_More_Registers(void)
{
   	//  TODO: FAE Modify the Init Regs here!!! 
/******* 5/11 daemon update for image performance *****************/
	GC0309_write_cmos_sensor(0x8b,0x22);   // lsc r 
	GC0309_write_cmos_sensor(0x71,0x43);   // auto sat limit
	
	//cc
	GC0309_write_cmos_sensor(0x93,0x48); 
	GC0309_write_cmos_sensor(0x94,0x00); 
	GC0309_write_cmos_sensor(0x95,0x05); 
	GC0309_write_cmos_sensor(0x96,0xe8); 
	GC0309_write_cmos_sensor(0x97,0x40); 
	GC0309_write_cmos_sensor(0x98,0xf8); 
	GC0309_write_cmos_sensor(0x9c,0x00); 
	GC0309_write_cmos_sensor(0x9d,0x00); 
	GC0309_write_cmos_sensor(0x9e,0x00); 
	
	GC0309_write_cmos_sensor(0xd0,0xcb);  // aec before gamma
	GC0309_write_cmos_sensor(0xd3,0x50);  // ae target
	
//	GC0309_SET_PAGE1;
	// awb update
//	GC0309_write_cmos_sensor(0x02,0x20); 
//	GC0309_write_cmos_sensor(0x04,0x06);
//	GC0309_write_cmos_sensor(0x05,0x20);
//	GC0309_write_cmos_sensor(0x06,0x20);
//	GC0309_write_cmos_sensor(0x10,0x41); 
//	GC0309_write_cmos_sensor(0x13,0x19); 
//	GC0309_write_cmos_sensor(0x1b,0xe0); 
	
//	GC0309_SET_PAGE0;
/******* end update for image performance *****************/


/******* 2010/07/19 Mormo Update ********** *****************/
	GC0309_write_cmos_sensor(0x31,0x60); 

	GC0309_write_cmos_sensor(0x1c,0x49); 
	GC0309_write_cmos_sensor(0x1d,0x98); 
	GC0309_write_cmos_sensor(0x10,0x26); 
	GC0309_write_cmos_sensor(0x1a,0x26);  
/**************** Mormo Update End *************************/	
	
    /*Customer can adjust GAMMA, MIRROR & UPSIDEDOWN here!*/
    GC0309_GAMMA_Select(2);
}


/*************************************************************************
 * FUNCTION
 *	GC0309Open
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
UINT32 GC0309Open(void)
{
    kal_uint16 sensor_id=0;
    int i;
    do
    {
        	// check if sensor ID correct
        	for(i = 0; i < 3; i++)
		{
	            	sensor_id = GC0309_read_cmos_sensor(0x00);
	            	printk("GC0309 Sensor id = %x\n", sensor_id);
	            	if (sensor_id == GC0309_SENSOR_ID)
			{
	               	break;
	            	}
        	}
        	mdelay(50);
    }while(0);

    if(sensor_id != GC0309_SENSOR_ID)
    {
        SENSORDB("GC0309 Sensor id read failed, ID = %x\n", sensor_id);
        return ERROR_SENSOR_CONNECT_FAIL;
    }

    RETAILMSG(1, (TEXT("Sensor Read ID OK \r\n")));
    // initail sequence write in
    GC0309_Sensor_Init();
    GC0309_Write_More_Registers();
	
    return ERROR_NONE;
} /* GC0309Open */


/*************************************************************************
 * FUNCTION
 *	GC0309Close
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
UINT32 GC0309Close(void)
{
    return ERROR_NONE;
} /* GC0309Close */


/*************************************************************************
 * FUNCTION
 * GC0309Preview
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
UINT32 GC0309Preview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
        MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)

{
    kal_uint32 iTemp;
    kal_uint16 iStartX = 0, iStartY = 1;

    if(sensor_config_data->SensorOperationMode == MSDK_SENSOR_OPERATION_MODE_VIDEO)		// MPEG4 Encode Mode
    {
        RETAILMSG(1, (TEXT("Camera Video preview\r\n")));
        GC0309_MPEG4_encode_mode = KAL_TRUE;
       
    }
    else
    {
        RETAILMSG(1, (TEXT("Camera preview\r\n")));
        GC0309_MPEG4_encode_mode = KAL_FALSE;
    }

    image_window->GrabStartX= IMAGE_SENSOR_VGA_GRAB_PIXELS;
    image_window->GrabStartY= IMAGE_SENSOR_VGA_GRAB_LINES;
    image_window->ExposureWindowWidth = IMAGE_SENSOR_PV_WIDTH;
    image_window->ExposureWindowHeight =IMAGE_SENSOR_PV_HEIGHT;

    // copy sensor_config_data
    memcpy(&GC0309SensorConfigData, sensor_config_data, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
    return ERROR_NONE;
} /* GC0309Preview */


/*************************************************************************
 * FUNCTION
 *	GC0309Capture
 *
 * DESCRIPTION
 *	This function setup the CMOS sensor in capture MY_OUTPUT mode
 *
 * PARAMETERS
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
UINT32 GC0309Capture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
        MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)

{
    GC0309_MODE_CAPTURE=KAL_TRUE;

    image_window->GrabStartX = IMAGE_SENSOR_VGA_GRAB_PIXELS;
    image_window->GrabStartY = IMAGE_SENSOR_VGA_GRAB_LINES;
    image_window->ExposureWindowWidth= IMAGE_SENSOR_FULL_WIDTH;
    image_window->ExposureWindowHeight = IMAGE_SENSOR_FULL_HEIGHT;

    // copy sensor_config_data
    memcpy(&GC0309SensorConfigData, sensor_config_data, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
    return ERROR_NONE;
} /* GC0309_Capture() */



UINT32 GC0309GetResolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *pSensorResolution)
{
    pSensorResolution->SensorFullWidth=IMAGE_SENSOR_FULL_WIDTH;
    pSensorResolution->SensorFullHeight=IMAGE_SENSOR_FULL_HEIGHT;
    pSensorResolution->SensorPreviewWidth=IMAGE_SENSOR_PV_WIDTH;
    pSensorResolution->SensorPreviewHeight=IMAGE_SENSOR_PV_HEIGHT;
    pSensorResolution->SensorVideoWidth=IMAGE_SENSOR_PV_WIDTH;
    pSensorResolution->SensorVideoHeight=IMAGE_SENSOR_PV_HEIGHT;
    return ERROR_NONE;
} /* GC0309GetResolution() */


UINT32 GC0309GetInfo(MSDK_SCENARIO_ID_ENUM ScenarioId,
        MSDK_SENSOR_INFO_STRUCT *pSensorInfo,
        MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
    pSensorInfo->SensorPreviewResolutionX=IMAGE_SENSOR_PV_WIDTH;
    pSensorInfo->SensorPreviewResolutionY=IMAGE_SENSOR_PV_HEIGHT;
    pSensorInfo->SensorFullResolutionX=IMAGE_SENSOR_FULL_WIDTH;
    pSensorInfo->SensorFullResolutionY=IMAGE_SENSOR_FULL_HEIGHT;

    pSensorInfo->SensorCameraPreviewFrameRate=30;
    pSensorInfo->SensorVideoFrameRate=30;
    pSensorInfo->SensorStillCaptureFrameRate=10;
    pSensorInfo->SensorWebCamCaptureFrameRate=15;
    pSensorInfo->SensorResetActiveHigh=FALSE;
    pSensorInfo->SensorResetDelayCount=1;
    pSensorInfo->SensorOutputDataFormat=SENSOR_OUTPUT_FORMAT_UYVY;//YUYV;
    pSensorInfo->SensorClockPolarity=SENSOR_CLOCK_POLARITY_LOW;
    pSensorInfo->SensorClockFallingPolarity=SENSOR_CLOCK_POLARITY_LOW;
    pSensorInfo->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
    pSensorInfo->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
    pSensorInfo->SensorInterruptDelayLines = 1;
    pSensorInfo->SensroInterfaceType=SENSOR_INTERFACE_TYPE_PARALLEL;

//    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_100_MODE].MaxWidth=CAM_SIZE_5M_WIDTH;
//    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_100_MODE].MaxHeight=CAM_SIZE_5M_HEIGHT;
//    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_100_MODE].ISOSupported=TRUE;
//    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_100_MODE].BinningEnable=FALSE;

//    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_200_MODE].MaxWidth=CAM_SIZE_5M_WIDTH;
//    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_200_MODE].MaxHeight=CAM_SIZE_5M_HEIGHT;
//    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_200_MODE].ISOSupported=TRUE;
//    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_200_MODE].BinningEnable=FALSE;

//    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_400_MODE].MaxWidth=CAM_SIZE_5M_WIDTH;
//    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_400_MODE].MaxHeight=CAM_SIZE_5M_HEIGHT;
//    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_400_MODE].ISOSupported=TRUE;
//    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_400_MODE].BinningEnable=FALSE;

//    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_800_MODE].MaxWidth=CAM_SIZE_1M_WIDTH;
//    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_800_MODE].MaxHeight=CAM_SIZE_1M_HEIGHT;
//    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_800_MODE].ISOSupported=TRUE;
//    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_800_MODE].BinningEnable=FALSE;

//    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_1600_MODE].MaxWidth=CAM_SIZE_1M_WIDTH;
//    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_1600_MODE].MaxHeight=CAM_SIZE_1M_HEIGHT;
//    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_1600_MODE].ISOSupported=TRUE;
//    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_1600_MODE].BinningEnable=FALSE;
    pSensorInfo->CaptureDelayFrame = 1;
    pSensorInfo->PreviewDelayFrame = 0;
    pSensorInfo->VideoDelayFrame = 4;
    pSensorInfo->SensorMasterClockSwitch = 0;
    pSensorInfo->SensorDrivingCurrent = ISP_DRIVING_2MA;

    switch (ScenarioId)
    {
    case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
    case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
    //case MSDK_SCENARIO_ID_VIDEO_CAPTURE_MPEG4:
        pSensorInfo->SensorClockFreq=26;
        pSensorInfo->SensorClockDividCount=	3;
        pSensorInfo->SensorClockRisingCount= 0;
        pSensorInfo->SensorClockFallingCount= 2;
        pSensorInfo->SensorPixelClockCount= 3;
        pSensorInfo->SensorDataLatchCount= 2;
        pSensorInfo->SensorGrabStartX = IMAGE_SENSOR_VGA_GRAB_PIXELS;
        pSensorInfo->SensorGrabStartY = IMAGE_SENSOR_VGA_GRAB_LINES;

        break;
    case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
    //case MSDK_SCENARIO_ID_CAMERA_CAPTURE_MEM:
        pSensorInfo->SensorClockFreq=26;
        pSensorInfo->SensorClockDividCount= 3;
        pSensorInfo->SensorClockRisingCount=0;
        pSensorInfo->SensorClockFallingCount=2;
        pSensorInfo->SensorPixelClockCount=3;
        pSensorInfo->SensorDataLatchCount=2;
        pSensorInfo->SensorGrabStartX = IMAGE_SENSOR_VGA_GRAB_PIXELS;
        pSensorInfo->SensorGrabStartY = IMAGE_SENSOR_VGA_GRAB_LINES;
        break;
    default:
        pSensorInfo->SensorClockFreq=26;
        pSensorInfo->SensorClockDividCount= 3;
        pSensorInfo->SensorClockRisingCount=0;
        pSensorInfo->SensorClockFallingCount=2;
        pSensorInfo->SensorPixelClockCount=3;
        pSensorInfo->SensorDataLatchCount=2;
        pSensorInfo->SensorGrabStartX = IMAGE_SENSOR_VGA_GRAB_PIXELS;
        pSensorInfo->SensorGrabStartY = IMAGE_SENSOR_VGA_GRAB_LINES;
        break;
    }
    GC0309PixelClockDivider=pSensorInfo->SensorPixelClockCount;
    memcpy(pSensorConfigData, &GC0309SensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
    return ERROR_NONE;
} /* GC0309GetInfo() */


UINT32 GC0309Control(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *pImageWindow,
        MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
    switch (ScenarioId)
    {
    case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
    case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
    //case MSDK_SCENARIO_ID_VIDEO_CAPTURE_MPEG4:
        GC0309Preview(pImageWindow, pSensorConfigData);
        break;
    case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
  //  case MSDK_SCENARIO_ID_CAMERA_CAPTURE_MEM:
        GC0309Capture(pImageWindow, pSensorConfigData);
        break;
    }


    return TRUE;
}	/* GC0309Control() */

BOOL GC0309_set_param_wb(UINT16 para)
{

	switch (para)
	{
		case AWB_MODE_OFF:

		break;
		
		case AWB_MODE_AUTO:
			GC0309_write_cmos_sensor(0x5a,0x56); //for AWB can adjust back
			GC0309_write_cmos_sensor(0x5b,0x40);
			GC0309_write_cmos_sensor(0x5c,0x4a);	
			GC0309_awb_enable(KAL_TRUE);
		break;
		
		case AWB_MODE_CLOUDY_DAYLIGHT: //cloudy
			GC0309_awb_enable(KAL_FALSE);
			GC0309_write_cmos_sensor(0x5a,0x8c); //WB_manual_gain 
			GC0309_write_cmos_sensor(0x5b,0x50);
			GC0309_write_cmos_sensor(0x5c,0x40);
		break;
		
		case AWB_MODE_DAYLIGHT: //sunny
			GC0309_awb_enable(KAL_FALSE);
			GC0309_write_cmos_sensor(0x5a,0x74); 
			GC0309_write_cmos_sensor(0x5b,0x52);
			GC0309_write_cmos_sensor(0x5c,0x40);		
		break;
		
		case AWB_MODE_INCANDESCENT: //office
			GC0309_awb_enable(KAL_FALSE);
			GC0309_write_cmos_sensor(0x5a,0x48);
			GC0309_write_cmos_sensor(0x5b,0x40);
			GC0309_write_cmos_sensor(0x5c,0x5c);
		break;
		
		case AWB_MODE_TUNGSTEN: //home
			GC0309_awb_enable(KAL_FALSE);
			GC0309_write_cmos_sensor(0x5a,0x40);
			GC0309_write_cmos_sensor(0x5b,0x54);
			GC0309_write_cmos_sensor(0x5c,0x70);
		break;
		
		case AWB_MODE_FLUORESCENT:
			GC0309_awb_enable(KAL_FALSE);
			GC0309_write_cmos_sensor(0x5a,0x40);
			GC0309_write_cmos_sensor(0x5b,0x42);
			GC0309_write_cmos_sensor(0x5c,0x50);
		break;
		
		default:
		return FALSE;
	}

	return TRUE;
} /* GC0309_set_param_wb */


BOOL GC0309_set_param_effect(UINT16 para)
{
	kal_uint32  ret = KAL_TRUE;

	switch (para)
	{
		case MEFFECT_OFF:
			GC0309_write_cmos_sensor(0x23,0x00);
			GC0309_write_cmos_sensor(0x2d,0x0a); // 0x08
			GC0309_write_cmos_sensor(0x20,0xff);
			GC0309_write_cmos_sensor(0xd2,0x90);
			GC0309_write_cmos_sensor(0x73,0x00);
			GC0309_write_cmos_sensor(0x77,0x54);
			
			GC0309_write_cmos_sensor(0xb3,0x40);
			GC0309_write_cmos_sensor(0xb4,0x80);
			GC0309_write_cmos_sensor(0xba,0x00);
			GC0309_write_cmos_sensor(0xbb,0x00);
		break;
		
		case MEFFECT_SEPIA:
	            	GC0309_write_cmos_sensor(0x23,0x02);		
			GC0309_write_cmos_sensor(0x2d,0x0a);
			GC0309_write_cmos_sensor(0x20,0xff);
			GC0309_write_cmos_sensor(0xd2,0x90);
			GC0309_write_cmos_sensor(0x73,0x00);

			GC0309_write_cmos_sensor(0xb3,0x40);
			GC0309_write_cmos_sensor(0xb4,0x80);
			GC0309_write_cmos_sensor(0xba,0xd0);
			GC0309_write_cmos_sensor(0xbb,0x28);
		break;
		
		case MEFFECT_NEGATIVE:
			GC0309_write_cmos_sensor(0x23,0x03);	
			GC0309_write_cmos_sensor(0x2d,0x0a);
			GC0309_write_cmos_sensor(0x20,0xff);
			GC0309_write_cmos_sensor(0xd2,0x90);
			GC0309_write_cmos_sensor(0x73,0x00);

			GC0309_write_cmos_sensor(0xb3,0x40);
			GC0309_write_cmos_sensor(0xb4,0x80);
			GC0309_write_cmos_sensor(0xba,0x00);
			GC0309_write_cmos_sensor(0xbb,0x00);
		break;
		
		case MEFFECT_SEPIAGREEN:
	            	GC0309_write_cmos_sensor(0x23,0x02);	
			GC0309_write_cmos_sensor(0x2d,0x0a);
			GC0309_write_cmos_sensor(0x20,0xff);
			GC0309_write_cmos_sensor(0xd2,0x90);
			GC0309_write_cmos_sensor(0x77,0x88);

			GC0309_write_cmos_sensor(0xb3,0x40);
			GC0309_write_cmos_sensor(0xb4,0x80);
			GC0309_write_cmos_sensor(0xba,0xc0);
			GC0309_write_cmos_sensor(0xbb,0xc0);
		break;
		
		case MEFFECT_SEPIABLUE:
			GC0309_write_cmos_sensor(0x23,0x02);	
			GC0309_write_cmos_sensor(0x2d,0x0a);
			GC0309_write_cmos_sensor(0x20,0xff);
			GC0309_write_cmos_sensor(0xd2,0x90);
			GC0309_write_cmos_sensor(0x73,0x00);

			GC0309_write_cmos_sensor(0xb3,0x40);
			GC0309_write_cmos_sensor(0xb4,0x80);
			GC0309_write_cmos_sensor(0xba,0x50);
			GC0309_write_cmos_sensor(0xbb,0xe0);
		break;

		case MEFFECT_MONO:
			GC0309_write_cmos_sensor(0x23,0x02);	
			GC0309_write_cmos_sensor(0x2d,0x0a);
			GC0309_write_cmos_sensor(0x20,0xff);
			GC0309_write_cmos_sensor(0xd2,0x90);
			GC0309_write_cmos_sensor(0x73,0x00);

			GC0309_write_cmos_sensor(0xb3,0x40);
			GC0309_write_cmos_sensor(0xb4,0x80);
			GC0309_write_cmos_sensor(0xba,0x00);
			GC0309_write_cmos_sensor(0xbb,0x00);
		break;
		default:
			ret = FALSE;
	}

	return ret;

} /* GC0309_set_param_effect */


BOOL GC0309_set_param_banding(UINT16 para)
{
	switch (para)
	{
		case AE_FLICKER_MODE_50HZ:
		{
			GC0309_write_cmos_sensor(0x01  ,0x26); 	
			GC0309_write_cmos_sensor(0x02  ,0x98); 
			GC0309_write_cmos_sensor(0x0f  ,0x03);

			GC0309_write_cmos_sensor(0xe2  ,0x00); 	//anti-flicker step [11:8]
			GC0309_write_cmos_sensor(0xe3  ,0x50);   //anti-flicker step [7:0]
			
			GC0309_write_cmos_sensor(0xe4  ,0x02);   //exp level 0  12.5fps
			GC0309_write_cmos_sensor(0xe5  ,0x80); 
			GC0309_write_cmos_sensor(0xe6  ,0x03);   //exp level 1  10fps
			GC0309_write_cmos_sensor(0xe7  ,0x20); 
			GC0309_write_cmos_sensor(0xe8  ,0x04);   //exp level 2  7.69fps
			GC0309_write_cmos_sensor(0xe9  ,0x10); 
			GC0309_write_cmos_sensor(0xea  ,0x06);   //exp level 3  5.00fps
			GC0309_write_cmos_sensor(0xeb  ,0x40); 

			GC0309_CAM_BANDING_50HZ = KAL_TRUE;
		}	
			break;

		case AE_FLICKER_MODE_60HZ:
		{
			GC0309_write_cmos_sensor(0x01  ,0x97); 	
			GC0309_write_cmos_sensor(0x02  ,0x84); 
			GC0309_write_cmos_sensor(0x0f  ,0x03);

			GC0309_write_cmos_sensor(0xe2  ,0x00); 	//anti-flicker step [11:8]
			GC0309_write_cmos_sensor(0xe3  ,0x3e);   //anti-flicker step [7:0]
			
			GC0309_write_cmos_sensor(0xe4  ,0x02);   //exp level 0  12.00fps
			GC0309_write_cmos_sensor(0xe5  ,0x6c); 
			GC0309_write_cmos_sensor(0xe6  ,0x02);   //exp level 1  10.00fps
			GC0309_write_cmos_sensor(0xe7  ,0xe8); 
			GC0309_write_cmos_sensor(0xe8  ,0x03);   //exp level 2  7.50fps
			GC0309_write_cmos_sensor(0xe9  ,0xe0); 
			GC0309_write_cmos_sensor(0xea  ,0x05);   //exp level 3  5.00fps
			GC0309_write_cmos_sensor(0xeb  ,0xd0); 
			GC0309_CAM_BANDING_50HZ = KAL_FALSE;
		}
		break;
		default:
		return FALSE;
	}

	return TRUE;
} /* GC0309_set_param_banding */


BOOL GC0309_set_param_exposure(UINT16 para)
{
	switch (para)
	{
		case AE_EV_COMP_13:  //+4 EV
			GC0309_write_cmos_sensor(0xb5, 0x40);
			GC0309_write_cmos_sensor(0xd3, 0x90);
			break;  
		case AE_EV_COMP_10:  //+3 EV
			GC0309_write_cmos_sensor(0xb5, 0x30);
			GC0309_write_cmos_sensor(0xd3, 0x80);
			break;    
		case AE_EV_COMP_07:  //+2 EV
			GC0309_write_cmos_sensor(0xb5, 0x20);
			GC0309_write_cmos_sensor(0xd3, 0x70);
			break;    
		case AE_EV_COMP_03:	 //	+1 EV	
			GC0309_write_cmos_sensor(0xb5, 0x10);
			GC0309_write_cmos_sensor(0xd3, 0x60);	
			break;    
		case AE_EV_COMP_00:  // +0 EV
		    	GC0309_write_cmos_sensor(0xb5, 0x04);
			GC0309_write_cmos_sensor(0xd3, 0x60);
			break;    
		case AE_EV_COMP_n03:  // -1 EV
			GC0309_write_cmos_sensor(0xb5, 0xf0);
			GC0309_write_cmos_sensor(0xd3, 0x48);
			break;    
		case AE_EV_COMP_n07:	// -2 EV		
			GC0309_write_cmos_sensor(0xb5, 0xe0);
			GC0309_write_cmos_sensor(0xd3, 0x40);	
			break;    
		case AE_EV_COMP_n10:   //-3 EV
			GC0309_write_cmos_sensor(0xb5, 0xd0);
			GC0309_write_cmos_sensor(0xd3, 0x38);
			break;
		case AE_EV_COMP_n13:  // -4 EV
			GC0309_write_cmos_sensor(0xb5, 0xc0);
			GC0309_write_cmos_sensor(0xd3, 0x30);
			break;
		default:
			return KAL_FALSE;
	}

	return TRUE;
} /* GC0309_set_param_exposure */


UINT32 GC0309YUVSensorSetting(FEATURE_ID iCmd, UINT16 iPara)
{
    switch (iCmd) {
       case FID_SCENE_MODE:	    //auto mode or night mode
       
       if (iPara == SCENE_MODE_OFF)//auto mode
       {
           GC0309_night_mode(FALSE); 
       }
       else if (iPara == SCENE_MODE_NIGHTSCENE)//night mode
       {
           GC0309_night_mode(TRUE); 
       }	
       
       break; 	
    case FID_AWB_MODE:
        GC0309_set_param_wb(iPara);
        break;
    case FID_COLOR_EFFECT:
        GC0309_set_param_effect(iPara);
        break;
    case FID_AE_EV:
        GC0309_set_param_exposure(iPara);
        break;
    case FID_AE_FLICKER:
        GC0309_set_param_banding(iPara);
        break;
    default:
        break;
    }
    return TRUE;
} /* GC0309YUVSensorSetting */

static void GC0309_FlashTriggerCheck(unsigned int *pFeatureReturnPara32)
{
	unsigned int shutter,shutter_limit,dg_pre,dg_post;
	GC0309_write_cmos_sensor(0xfe,0x00);
	dg_pre = GC0309_read_cmos_sensor(0x51);
	dg_post = GC0309_read_cmos_sensor(0x52);
	if((dg_pre >= 0x48)&&(dg_post >= 0x60))
		*pFeatureReturnPara32 = TRUE;
	else
		*pFeatureReturnPara32 = FALSE;
	return;
}
UINT32 GC0309FeatureControl(MSDK_SENSOR_FEATURE_ENUM FeatureId,
        UINT8 *pFeaturePara,UINT32 *pFeatureParaLen)
{
    UINT16 *pFeatureReturnPara16=(UINT16 *) pFeaturePara;
    UINT16 *pFeatureData16=(UINT16 *) pFeaturePara;
    UINT32 *pFeatureReturnPara32=(UINT32 *) pFeaturePara;
    UINT32 *pFeatureData32=(UINT32 *) pFeaturePara;
    UINT32 GC0309SensorRegNumber;
    UINT32 i;
    MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData=(MSDK_SENSOR_CONFIG_STRUCT *) pFeaturePara;
    MSDK_SENSOR_REG_INFO_STRUCT *pSensorRegData=(MSDK_SENSOR_REG_INFO_STRUCT *) pFeaturePara;

    RETAILMSG(1, (_T("gaiyang GC0309FeatureControl FeatureId=%d\r\n"), FeatureId));

    switch (FeatureId)
    {
    case SENSOR_FEATURE_GET_RESOLUTION:
        *pFeatureReturnPara16++=IMAGE_SENSOR_FULL_WIDTH;
        *pFeatureReturnPara16=IMAGE_SENSOR_FULL_HEIGHT;
        *pFeatureParaLen=4;
        break;
    case SENSOR_FEATURE_GET_PERIOD:
        *pFeatureReturnPara16++=(VGA_PERIOD_PIXEL_NUMS)+GC0309_dummy_pixels;
        *pFeatureReturnPara16=(VGA_PERIOD_LINE_NUMS)+GC0309_dummy_lines;
        *pFeatureParaLen=4;
        break;
    case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
        *pFeatureReturnPara32 = GC0309_g_fPV_PCLK;
        *pFeatureParaLen=4;
        break;
    case SENSOR_FEATURE_SET_ESHUTTER:
        break;
    case SENSOR_FEATURE_SET_NIGHTMODE:
        GC0309_night_mode((BOOL) *pFeatureData16);
        break;
    case SENSOR_FEATURE_SET_GAIN:
    case SENSOR_FEATURE_SET_FLASHLIGHT:
        break;
    case SENSOR_FEATURE_GET_TRIGGER_FLASHLIGHT_INFO:
         GC0309_FlashTriggerCheck(pFeatureData32);
         break;        
    case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
        GC0309_isp_master_clock=*pFeatureData32;
        break;
    case SENSOR_FEATURE_SET_REGISTER:
        GC0309_write_cmos_sensor(pSensorRegData->RegAddr, pSensorRegData->RegData);
        break;
    case SENSOR_FEATURE_GET_REGISTER:
        pSensorRegData->RegData = GC0309_read_cmos_sensor(pSensorRegData->RegAddr);
        break;
    case SENSOR_FEATURE_GET_CONFIG_PARA:
        memcpy(pSensorConfigData, &GC0309SensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
        *pFeatureParaLen=sizeof(MSDK_SENSOR_CONFIG_STRUCT);
        break;
    case SENSOR_FEATURE_SET_CCT_REGISTER:
    case SENSOR_FEATURE_GET_CCT_REGISTER:
    case SENSOR_FEATURE_SET_ENG_REGISTER:
    case SENSOR_FEATURE_GET_ENG_REGISTER:
    case SENSOR_FEATURE_GET_REGISTER_DEFAULT:
    case SENSOR_FEATURE_CAMERA_PARA_TO_SENSOR:
    case SENSOR_FEATURE_SENSOR_TO_CAMERA_PARA:
    case SENSOR_FEATURE_GET_GROUP_COUNT:
    case SENSOR_FEATURE_GET_GROUP_INFO:
    case SENSOR_FEATURE_GET_ITEM_INFO:
    case SENSOR_FEATURE_SET_ITEM_INFO:
    case SENSOR_FEATURE_GET_ENG_INFO:
        break;
    case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
        // get the lens driver ID from EEPROM or just return LENS_DRIVER_ID_DO_NOT_CARE
        // if EEPROM does not exist in camera module.
        *pFeatureReturnPara32=LENS_DRIVER_ID_DO_NOT_CARE;
        *pFeatureParaLen=4;
        break;
    case SENSOR_FEATURE_SET_YUV_CMD:
        GC0309YUVSensorSetting((FEATURE_ID)*pFeatureData32, *(pFeatureData32+1));
        break;
    case SENSOR_FEATURE_CHECK_SENSOR_ID:
	GC0309GetSensorID(pFeatureData32);
	break;
    default:
        break;
	}
return ERROR_NONE;
}	/* GC0309FeatureControl() */


SENSOR_FUNCTION_STRUCT	SensorFuncGC0309YUV=
{
	GC0309Open,
	GC0309GetInfo,
	GC0309GetResolution,
	GC0309FeatureControl,
	GC0309Control,
	GC0309Close
};


UINT32 GC0309_YUV_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
	/* To Do : Check Sensor status here */
	if (pfFunc!=NULL)
		*pfFunc=&SensorFuncGC0309YUV;
	return ERROR_NONE;
} /* SensorInit() */
