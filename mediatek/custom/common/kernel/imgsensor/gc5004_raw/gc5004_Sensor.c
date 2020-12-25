/*****************************************************************************
 *
 * Filename:
 * ---------
 *   sensor.c
 *
 * Project:
 * --------
 *   RAW
 *
 * Description:
 * ------------  
 *   Source code of Sensor driver
 *
 *
 * Author:
 * -------
 *   Leo Lee
 *
 *============================================================================
 *             HISTORY
 *------------------------------------------------------------------------------
 * $Revision:$
 * $Modtime:$
 * $Log:$
 *
 * 04 10  2013
 * First release MT6589 GC5004 driver Version 1.0
 *
 *------------------------------------------------------------------------------
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

#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"

#include "gc5004_Sensor.h"
#include "gc5004_Camera_Sensor_para.h"
#include "gc5004_CameraCustomized.h"

#ifdef GC5004_DEBUG
#define SENSORDB printk
#else
#define SENSORDB(x,...)
#endif

static DEFINE_SPINLOCK(gc5004_drv_lock);

extern int iReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 * a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
extern int iWriteRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u16 i2cId);


static GC5004_sensor_struct GC5004_sensor =
{
	.eng_info =
	{
		.SensorId = 128,
		.SensorType = CMOS_SENSOR,
		.SensorOutputDataFormat = GC5004_COLOR_FORMAT,
	},
	.Mirror = GC5004_IMAGE_H_MIRROR,
	.shutter = 0x20,  
	.gain = 0x20,
	.pclk = GC5004_PREVIEW_CLK,
	.frame_height = GC5004_PV_PERIOD_LINE_NUMS,
	.line_length = GC5004_PV_PERIOD_PIXEL_NUMS,
};


/*************************************************************************
* FUNCTION
*    GC5004_write_cmos_sensor
*
* DESCRIPTION
*    This function wirte data to CMOS sensor through I2C
*
* PARAMETERS
*    addr: the 16bit address of register
*    para: the 8bit value of register
*
* RETURNS
*    None
*
* LOCAL AFFECTED
*
*************************************************************************/
static void GC5004_write_cmos_sensor(kal_uint8 addr, kal_uint8 para)
{
	kal_uint8 out_buff[2];

	out_buff[0] = addr;
	out_buff[1] = para;
	
	iWriteRegI2C((u8*)out_buff , (u16)sizeof(out_buff), GC5004_WRITE_ID); 
}

/*************************************************************************
* FUNCTION
*    GC2035_read_cmos_sensor
*
* DESCRIPTION
*    This function read data from CMOS sensor through I2C.
*
* PARAMETERS
*    addr: the 16bit address of register
*
* RETURNS
*    8bit data read through I2C
*
* LOCAL AFFECTED
*
*************************************************************************/
static kal_uint8 GC5004_read_cmos_sensor(kal_uint8 addr)
{
	kal_uint8 in_buff[1] = {0xFF};
	kal_uint8 out_buff[1];

	out_buff[0] = addr;

	if (0 != iReadRegI2C((u8*)out_buff , (u16) sizeof(out_buff), (u8*)in_buff, (u16) sizeof(in_buff), GC5004_WRITE_ID)) {
	SENSORDB("ERROR: GC5004_read_cmos_sensor \n");
	}
	return in_buff[0];
}

/*************************************************************************
* FUNCTION
*	GC5004_SetShutter
*
* DESCRIPTION
*	This function set e-shutter of GC5004 to change exposure time.
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
void GC5004_set_shutter(kal_uint16 iShutter)
{
	kal_uint8 shutter_temp;
	
	spin_lock(&gc5004_drv_lock);
	GC5004_sensor.shutter = iShutter;
	spin_unlock(&gc5004_drv_lock);

	if (!iShutter) iShutter = 4; /* avoid 0 */
	
	if(iShutter < 4) iShutter = 4;
	if(iShutter > 8191) iShutter = 8191;//2^13

	//align 4
	iShutter = iShutter / 4;
	iShutter = iShutter * 4;
	
	shutter_temp = iShutter%4;
	if(shutter_temp>2)
		iShutter+=4;

	#ifdef GC5004_DRIVER_TRACE
	SENSORDB("GC5004_set_shutter iShutter = %d \n",iShutter);
	#endif

	//Update Shutter	
	GC5004_write_cmos_sensor(0x04, (iShutter) & 0xFF);
	GC5004_write_cmos_sensor(0x03, (iShutter >> 8) & 0x1F);	
}   /*  Set_GC5004_Shutter */

kal_uint16 GC5004_read_shutter(void)
{
	kal_uint16 shutter,temp_reg1,temp_reg2;
	temp_reg1 = GC5004_read_cmos_sensor(0x03);
	temp_reg2 = GC5004_read_cmos_sensor(0x04);
	shutter = ((temp_reg1<<8)&0x1F00)|(temp_reg2&0xFF);
	return shutter;
}
/*************************************************************************
* FUNCTION
*	GC5004_SetGain
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
#define ANALOG_GAIN_1 64  // 1.00x
#define ANALOG_GAIN_2 90  // 1.41x
#define ANALOG_GAIN_3 128  // 2.00x
#define ANALOG_GAIN_4 178  // 2.78x
#define ANALOG_GAIN_5 247  // 3.85x
#define ANALOG_GAIN_6 332  // 5.18x
#define ANALOG_GAIN_7 435  // 6.80x

kal_uint16 GC5004_SetGain(kal_uint16 iGain)
{
	kal_uint16 iReg,temp;

	#ifdef GC5004_DRIVER_TRACE
	SENSORDB("GC5004_SetGain iGain = %d \n",iGain);
	#endif
	GC5004_write_cmos_sensor(0xb1, 0x01);
	GC5004_write_cmos_sensor(0xb2, 0x00);

	if(iGain>=192)
		GC5004_sensor.LowLightMode = KAL_TRUE;
	else
		GC5004_sensor.LowLightMode = KAL_FALSE;

	iReg = iGain;
#if 0 //digital gain
	GC5004_write_cmos_sensor(0xb1, iReg>>6);
	GC5004_write_cmos_sensor(0xb2, (iReg<<2)&0xfc);
#else //analog gain
	if(iReg < 0x40)
		iReg = 0x40;
	else if((ANALOG_GAIN_1<= iReg)&&(iReg < ANALOG_GAIN_2))
	{
		//analog gain
		GC5004_write_cmos_sensor(0xb6,  0x00);// 
		temp = iReg;
		GC5004_write_cmos_sensor(0xb1, temp>>6);
		GC5004_write_cmos_sensor(0xb2, (temp<<2)&0xfc);
		//SENSORDB("GC5004 analogic gain 1x , GC5004 add pregain = %d\n",temp);
	}
	else if((ANALOG_GAIN_2<= iReg)&&(iReg < ANALOG_GAIN_3))
	{
		//analog gain
		GC5004_write_cmos_sensor(0xb6,  0x01);// 
		temp = 64*iReg/ANALOG_GAIN_2;
		GC5004_write_cmos_sensor(0xb1, temp>>6);
		GC5004_write_cmos_sensor(0xb2, (temp<<2)&0xfc);
		//SENSORDB("GC5004 analogic gain 1.45x , GC5004 add pregain = %d\n",temp);
	}

	else if((ANALOG_GAIN_3<= iReg)&&(iReg < ANALOG_GAIN_4))
	{
		//analog gain
		GC5004_write_cmos_sensor(0xb6,  0x02);//
		temp = 64*iReg/ANALOG_GAIN_3;
		GC5004_write_cmos_sensor(0xb1, temp>>6);
		GC5004_write_cmos_sensor(0xb2, (temp<<2)&0xfc);
		//SENSORDB("GC5004 analogic gain 2.02x , GC5004 add pregain = %d\n",temp);
	}
	else if((ANALOG_GAIN_4<= iReg)&&(iReg < ANALOG_GAIN_5))
	{
		//analog gain
		GC5004_write_cmos_sensor(0xb6,  0x03);//
		temp = 64*iReg/ANALOG_GAIN_4;
		GC5004_write_cmos_sensor(0xb1, temp>>6);
		GC5004_write_cmos_sensor(0xb2, (temp<<2)&0xfc);
		//SENSORDB("GC5004 analogic gain 2.86x , GC5004 add pregain = %d\n",temp);
	}

	//else if((ANALOG_GAIN_5<= iReg)&&(iReg)&&(iReg < ANALOG_GAIN_6))
	else if(ANALOG_GAIN_5<= iReg)
	{
		//analog gain
		GC5004_write_cmos_sensor(0xb6,  0x04);//
		temp = 64*iReg/ANALOG_GAIN_5;
		GC5004_write_cmos_sensor(0xb1, temp>>6);
		GC5004_write_cmos_sensor(0xb2, (temp<<2)&0xfc);
		//SENSORDB("GC5004 analogic gain 3.95x , GC5004 add pregain = %d\n",temp);
	}

/*
	else if((ANALOG_GAIN_6<= iReg)&&(iReg < ANALOG_GAIN_7))
	{
		//analog gain
		GC5004_write_cmos_sensor(0xb6,  0x05);//
		temp = 64*iReg/ANALOG_GAIN_6;
		GC5004_write_cmos_sensor(0xb1, temp>>6);
		GC5004_write_cmos_sensor(0xb2, (temp<<2)&0xfc);
		SENSORDB("GC5004 analogic gain 5.46x , GC5004 add pregain = %d\n",temp);
	}
	else if(ANALOG_GAIN_7<= iReg)
	{
		//analog gain
		GC5004_write_cmos_sensor(0xb6,  0x06);//
		temp = 64*iReg/ANALOG_GAIN_7;
		GC5004_write_cmos_sensor(0xb1, temp>>6);
		GC5004_write_cmos_sensor(0xb2, (temp<<2)&0xfc);
		SENSORDB("GC5004 analogic gain 7.5x");
	}
*/
#endif
}
/*************************************************************************
* FUNCTION
*	GC5004_NightMode
*
* DESCRIPTION
*	This function night mode of GC5004.
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
void GC5004_night_mode(kal_bool enable)
{
/*No Need to implement this function*/
#if 0 
	const kal_uint16 dummy_pixel = GC5004_sensor.line_length - GC5004_PV_PERIOD_PIXEL_NUMS;
	const kal_uint16 pv_min_fps =  enable ? GC5004_sensor.night_fps : GC5004_sensor.normal_fps;
	kal_uint16 dummy_line = GC5004_sensor.frame_height - GC5004_PV_PERIOD_LINE_NUMS;
	kal_uint16 max_exposure_lines;
	
	printk("[GC5004_night_mode]enable=%d",enable);
	if (!GC5004_sensor.video_mode) return;
	max_exposure_lines = GC5004_sensor.pclk * GC5004_FPS(1) / (pv_min_fps * GC5004_sensor.line_length);
	if (max_exposure_lines > GC5004_sensor.frame_height) /* fix max frame rate, AE table will fix min frame rate */
//	{
//	  dummy_line = max_exposure_lines - GC5004_PV_PERIOD_LINE_NUMS;
//	}
#endif
}   /*  GC5004_NightMode    */


/* write camera_para to sensor register */
static void GC5004_camera_para_to_sensor(void)
{
  kal_uint32 i;
#ifdef GC5004_DRIVER_TRACE
	 SENSORDB("GC5004_camera_para_to_sensor\n");
#endif
  for (i = 0; 0xFFFFFFFF != GC5004_sensor.eng.reg[i].Addr; i++)
  {
    GC5004_write_cmos_sensor(GC5004_sensor.eng.reg[i].Addr, GC5004_sensor.eng.reg[i].Para);
  }
  for (i = GC5004_FACTORY_START_ADDR; 0xFFFFFFFF != GC5004_sensor.eng.reg[i].Addr; i++)
  {
    GC5004_write_cmos_sensor(GC5004_sensor.eng.reg[i].Addr, GC5004_sensor.eng.reg[i].Para);
  }
  GC5004_SetGain(GC5004_sensor.gain); /* update gain */
}

/* update camera_para from sensor register */
static void GC5004_sensor_to_camera_para(void)
{
  kal_uint32 i,temp_data;
#ifdef GC5004_DRIVER_TRACE
   SENSORDB("GC5004_sensor_to_camera_para\n");
#endif
  for (i = 0; 0xFFFFFFFF != GC5004_sensor.eng.reg[i].Addr; i++)
  {
	temp_data = GC5004_read_cmos_sensor(GC5004_sensor.eng.reg[i].Addr);
	spin_lock(&gc5004_drv_lock);
	GC5004_sensor.eng.reg[i].Para = temp_data;
	spin_unlock(&gc5004_drv_lock);
  }
  for (i = GC5004_FACTORY_START_ADDR; 0xFFFFFFFF != GC5004_sensor.eng.reg[i].Addr; i++)
  {
	temp_data = GC5004_read_cmos_sensor(GC5004_sensor.eng.reg[i].Addr);
	spin_lock(&gc5004_drv_lock);
	GC5004_sensor.eng.reg[i].Para = temp_data;
	spin_unlock(&gc5004_drv_lock);
  }
}

/* ------------------------ Engineer mode ------------------------ */
inline static void GC5004_get_sensor_group_count(kal_int32 *sensor_count_ptr)
{
#ifdef GC5004_DRIVER_TRACE
   SENSORDB("GC5004_get_sensor_group_count\n");
#endif
  *sensor_count_ptr = GC5004_GROUP_TOTAL_NUMS;
}

inline static void GC5004_get_sensor_group_info(MSDK_SENSOR_GROUP_INFO_STRUCT *para)
{
#ifdef GC5004_DRIVER_TRACE
   SENSORDB("GC5004_get_sensor_group_info\n");
#endif
  switch (para->GroupIdx)
  {
  case GC5004_PRE_GAIN:
    sprintf(para->GroupNamePtr, "CCT");
    para->ItemCount = 5;
    break;
  case GC5004_CMMCLK_CURRENT:
    sprintf(para->GroupNamePtr, "CMMCLK Current");
    para->ItemCount = 1;
    break;
  case GC5004_FRAME_RATE_LIMITATION:
    sprintf(para->GroupNamePtr, "Frame Rate Limitation");
    para->ItemCount = 2;
    break;
  case GC5004_REGISTER_EDITOR:
    sprintf(para->GroupNamePtr, "Register Editor");
    para->ItemCount = 2;
    break;
  default:
    ASSERT(0);
  }
}

inline static void GC5004_get_sensor_item_info(MSDK_SENSOR_ITEM_INFO_STRUCT *para)
{

  const static kal_char *cct_item_name[] = {"SENSOR_BASEGAIN", "Pregain-R", "Pregain-Gr", "Pregain-Gb", "Pregain-B"};
  const static kal_char *editer_item_name[] = {"REG addr", "REG value"};
  
#ifdef GC5004_DRIVER_TRACE
	 SENSORDB("GC5004_get_sensor_item_info\n");
#endif
  switch (para->GroupIdx)
  {
  case GC5004_PRE_GAIN:
    switch (para->ItemIdx)
    {
    case GC5004_SENSOR_BASEGAIN:
    case GC5004_PRE_GAIN_R_INDEX:
    case GC5004_PRE_GAIN_Gr_INDEX:
    case GC5004_PRE_GAIN_Gb_INDEX:
    case GC5004_PRE_GAIN_B_INDEX:
      break;
    default:
      ASSERT(0);
    }
    sprintf(para->ItemNamePtr, cct_item_name[para->ItemIdx - GC5004_SENSOR_BASEGAIN]);
    para->ItemValue = GC5004_sensor.eng.cct[para->ItemIdx].Para * 1000 / BASEGAIN;
    para->IsTrueFalse = para->IsReadOnly = para->IsNeedRestart = KAL_FALSE;
    para->Min = GC5004_MIN_ANALOG_GAIN * 1000;
    para->Max = GC5004_MAX_ANALOG_GAIN * 1000;
    break;
  case GC5004_CMMCLK_CURRENT:
    switch (para->ItemIdx)
    {
    case 0:
      sprintf(para->ItemNamePtr, "Drv Cur[2,4,6,8]mA");
      switch (GC5004_sensor.eng.reg[GC5004_CMMCLK_CURRENT_INDEX].Para)
      {
      case ISP_DRIVING_2MA:
        para->ItemValue = 2;
        break;
      case ISP_DRIVING_4MA:
        para->ItemValue = 4;
        break;
      case ISP_DRIVING_6MA:
        para->ItemValue = 6;
        break;
      case ISP_DRIVING_8MA:
        para->ItemValue = 8;
        break;
      default:
        ASSERT(0);
      }
      para->IsTrueFalse = para->IsReadOnly = KAL_FALSE;
      para->IsNeedRestart = KAL_TRUE;
      para->Min = 2;
      para->Max = 8;
      break;
    default:
      ASSERT(0);
    }
    break;
  case GC5004_FRAME_RATE_LIMITATION:
    switch (para->ItemIdx)
    {
    case 0:
      sprintf(para->ItemNamePtr, "Max Exposure Lines");
      para->ItemValue = 5998;
      break;
    case 1:
      sprintf(para->ItemNamePtr, "Min Frame Rate");
      para->ItemValue = 5;
      break;
    default:
      ASSERT(0);
    }
    para->IsTrueFalse = para->IsNeedRestart = KAL_FALSE;
    para->IsReadOnly = KAL_TRUE;
    para->Min = para->Max = 0;
    break;
  case GC5004_REGISTER_EDITOR:
    switch (para->ItemIdx)
    {
    case 0:
    case 1:
      sprintf(para->ItemNamePtr, editer_item_name[para->ItemIdx]);
      para->ItemValue = 0;
      para->IsTrueFalse = para->IsReadOnly = para->IsNeedRestart = KAL_FALSE;
      para->Min = 0;
      para->Max = (para->ItemIdx == 0 ? 0xFFFF : 0xFF);
      break;
    default:
      ASSERT(0);
    }
    break;
  default:
    ASSERT(0);
  }
}

inline static kal_bool GC5004_set_sensor_item_info(MSDK_SENSOR_ITEM_INFO_STRUCT *para)
{
  kal_uint16 temp_para;
#ifdef GC5004_DRIVER_TRACE
   SENSORDB("GC5004_set_sensor_item_info\n");
#endif
  switch (para->GroupIdx)
  {
  case GC5004_PRE_GAIN:
    switch (para->ItemIdx)
    {
    case GC5004_SENSOR_BASEGAIN:
    case GC5004_PRE_GAIN_R_INDEX:
    case GC5004_PRE_GAIN_Gr_INDEX:
    case GC5004_PRE_GAIN_Gb_INDEX:
    case GC5004_PRE_GAIN_B_INDEX:
	spin_lock(&gc5004_drv_lock);
	GC5004_sensor.eng.cct[para->ItemIdx].Para = para->ItemValue * BASEGAIN / 1000;
	spin_unlock(&gc5004_drv_lock);
	GC5004_SetGain(GC5004_sensor.gain); /* update gain */
      break;
    default:
      ASSERT(0);
    }
    break;
  case GC5004_CMMCLK_CURRENT:
    switch (para->ItemIdx)
    {
    case 0:
      switch (para->ItemValue)
      {
      case 2:
        temp_para = ISP_DRIVING_2MA;
        break;
      case 3:
      case 4:
        temp_para = ISP_DRIVING_4MA;
        break;
      case 5:
      case 6:
        temp_para = ISP_DRIVING_6MA;
        break;
      default:
        temp_para = ISP_DRIVING_8MA;
        break;
      }
      //GC5004_set_isp_driving_current(temp_para);
      spin_lock(&gc5004_drv_lock);
      GC5004_sensor.eng.reg[GC5004_CMMCLK_CURRENT_INDEX].Para = temp_para;
	  spin_unlock(&gc5004_drv_lock);
      break;
    default:
      ASSERT(0);
    }
    break;
  case GC5004_FRAME_RATE_LIMITATION:
    ASSERT(0);
    break;
  case GC5004_REGISTER_EDITOR:
    switch (para->ItemIdx)
    {
      static kal_uint32 fac_sensor_reg;
    case 0:
      if (para->ItemValue < 0 || para->ItemValue > 0xFFFF) return KAL_FALSE;
      fac_sensor_reg = para->ItemValue;
      break;
    case 1:
      if (para->ItemValue < 0 || para->ItemValue > 0xFF) return KAL_FALSE;
      GC5004_write_cmos_sensor(fac_sensor_reg, para->ItemValue);
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

void GC5004_SetMirrorFlip(GC5004_IMAGE_MIRROR Mirror)
{
	switch(Mirror)
	{
		case GC5004_IMAGE_NORMAL://IMAGE_V_MIRROR:
		   GC5004_write_cmos_sensor(0x17,0x14);
		   GC5004_write_cmos_sensor(0x92,0x03);
		   GC5004_write_cmos_sensor(0x94,0x07);
		    break;
		case GC5004_IMAGE_H_MIRROR://IMAGE_NORMAL:
		   GC5004_write_cmos_sensor(0x17,0x15);
		   GC5004_write_cmos_sensor(0x92,0x03);
		   GC5004_write_cmos_sensor(0x94,0x06);
		    break;
		case GC5004_IMAGE_V_MIRROR://IMAGE_HV_MIRROR:
		   GC5004_write_cmos_sensor(0x17,0x16);
		   GC5004_write_cmos_sensor(0x92,0x02);
		   GC5004_write_cmos_sensor(0x94,0x07);
		    break;
		case GC5004_IMAGE_HV_MIRROR://IMAGE_H_MIRROR:
		   GC5004_write_cmos_sensor(0x17,0x17);
		   GC5004_write_cmos_sensor(0x92,0x02);
		   GC5004_write_cmos_sensor(0x94,0x06);
		    break;
	}
}

static void GC5004_Init_Settings(void)
{
	//2592x1944
	/////////////////////////////////////////////////////
	//////////////////////   SYS   //////////////////////
	/////////////////////////////////////////////////////
	GC5004_write_cmos_sensor(0xfe, 0x80);
	GC5004_write_cmos_sensor(0xfe, 0x80);
	GC5004_write_cmos_sensor(0xfe, 0x80);
	GC5004_write_cmos_sensor(0xf2, 0x00); //sync_pad_io_ebi
	GC5004_write_cmos_sensor(0xf6, 0x00);
	GC5004_write_cmos_sensor(0xfc, 0x06);
	GC5004_write_cmos_sensor(0xf7, 0x9d); //Pll enable
	GC5004_write_cmos_sensor(0xf8, 0x83); //Pll mode 2
	GC5004_write_cmos_sensor(0xf9, 0xfe); //[0] pll enable
	GC5004_write_cmos_sensor(0xfa, 0x00); //div
	GC5004_write_cmos_sensor(0xfe, 0x00);

	/////////////////////////////////////////////////////
	////////////////   ANALOG & CISCTL   ////////////////
	/////////////////////////////////////////////////////
	GC5004_write_cmos_sensor(0x00, 0x40); //[4]rowskip_skip_sh
	GC5004_write_cmos_sensor(0x01, 0x10); // 20140324 txlow buffer
	GC5004_write_cmos_sensor(0x03, 0x06);
	GC5004_write_cmos_sensor(0x04, 0xd6);
	GC5004_write_cmos_sensor(0x05, 0x01); //HB
	GC5004_write_cmos_sensor(0x06, 0x0a);
	GC5004_write_cmos_sensor(0x07, 0x00); //VB
	GC5004_write_cmos_sensor(0x08, 0x1c);
	GC5004_write_cmos_sensor(0x0a, 0x02); //row start
	GC5004_write_cmos_sensor(0x0c, 0x00); //col start
	GC5004_write_cmos_sensor(0x0d, 0x07); //Window setting
	GC5004_write_cmos_sensor(0x0e, 0xa8);
	GC5004_write_cmos_sensor(0x0f, 0x0a);
	GC5004_write_cmos_sensor(0x10, 0x50);
	GC5004_write_cmos_sensor(0x17, 0x15); //[0]mirror [1]flip
	GC5004_write_cmos_sensor(0x18, 0x02); //sdark off
	GC5004_write_cmos_sensor(0x19, 0x0c);
	GC5004_write_cmos_sensor(0x1a, 0x13);
	GC5004_write_cmos_sensor(0x1b, 0x48);
	GC5004_write_cmos_sensor(0x1c, 0x05);
	GC5004_write_cmos_sensor(0x1e, 0xb8);
	GC5004_write_cmos_sensor(0x1f, 0x78);
	GC5004_write_cmos_sensor(0x20, 0xc5); //[7:6]ref_r [3:1]comv_r
	GC5004_write_cmos_sensor(0x21, 0x4f);
	GC5004_write_cmos_sensor(0x22, 0x82);
	GC5004_write_cmos_sensor(0x23, 0x43); //[7:3]opa_r [1:0]sRef
	GC5004_write_cmos_sensor(0x24, 0x2f); //PAD drive
	GC5004_write_cmos_sensor(0x2b, 0x01);
	GC5004_write_cmos_sensor(0x2c, 0x68); //[6:4]rsgh_r

	/////////////////////////////////////////////////////
	//////////////////////   ISP   //////////////////////
	/////////////////////////////////////////////////////
	GC5004_write_cmos_sensor(0x86, 0x8e); //[7] data_delay_half
	GC5004_write_cmos_sensor(0x89, 0x03);
	GC5004_write_cmos_sensor(0x8a, 0x83);
	GC5004_write_cmos_sensor(0x8b, 0x61);
	GC5004_write_cmos_sensor(0x8c, 0x10);
	GC5004_write_cmos_sensor(0x8d, 0x01);
	GC5004_write_cmos_sensor(0x90, 0x01);
	GC5004_write_cmos_sensor(0x92, 0x00); //crop win y
	GC5004_write_cmos_sensor(0x94, 0x0d); //crop win x
	GC5004_write_cmos_sensor(0x95, 0x07); //crop win height
	GC5004_write_cmos_sensor(0x96, 0x98);
	GC5004_write_cmos_sensor(0x97, 0x0a); //crop win width
	GC5004_write_cmos_sensor(0x98, 0x20);

	/////////////////////////////////////////////////////
	//////////////////////   BLK   //////////////////////
	/////////////////////////////////////////////////////
	GC5004_write_cmos_sensor(0x40, 0x22);
	GC5004_write_cmos_sensor(0x41, 0x00);
	GC5004_write_cmos_sensor(0x50, 0x00);
	GC5004_write_cmos_sensor(0x51, 0x00);
	GC5004_write_cmos_sensor(0x52, 0x00);
	GC5004_write_cmos_sensor(0x53, 0x00);
	GC5004_write_cmos_sensor(0x54, 0x00);
	GC5004_write_cmos_sensor(0x55, 0x00);
	GC5004_write_cmos_sensor(0x56, 0x00);
	GC5004_write_cmos_sensor(0x57, 0x00);
	GC5004_write_cmos_sensor(0x58, 0x00);
	GC5004_write_cmos_sensor(0x59, 0x00);
	GC5004_write_cmos_sensor(0x5a, 0x00);
	GC5004_write_cmos_sensor(0x5b, 0x00);
	GC5004_write_cmos_sensor(0x5c, 0x00);
	GC5004_write_cmos_sensor(0x5d, 0x00);
	GC5004_write_cmos_sensor(0x5e, 0x00);
	GC5004_write_cmos_sensor(0x5f, 0x00);
	GC5004_write_cmos_sensor(0xd0, 0x00);
	GC5004_write_cmos_sensor(0xd1, 0x00);
	GC5004_write_cmos_sensor(0xd2, 0x00);
	GC5004_write_cmos_sensor(0xd3, 0x00);
	GC5004_write_cmos_sensor(0xd4, 0x00);
	GC5004_write_cmos_sensor(0xd5, 0x00);
	GC5004_write_cmos_sensor(0xd6, 0x00);
	GC5004_write_cmos_sensor(0xd7, 0x00);
	GC5004_write_cmos_sensor(0xd8, 0x00);
	GC5004_write_cmos_sensor(0xd9, 0x00);
	GC5004_write_cmos_sensor(0xda, 0x00);
	GC5004_write_cmos_sensor(0xdb, 0x00);
	GC5004_write_cmos_sensor(0xdc, 0x00);
	GC5004_write_cmos_sensor(0xdd, 0x00);
	GC5004_write_cmos_sensor(0xde, 0x00);
	GC5004_write_cmos_sensor(0xdf, 0x00);
	GC5004_write_cmos_sensor(0x70, 0x00);
	GC5004_write_cmos_sensor(0x71, 0x00);
	GC5004_write_cmos_sensor(0x72, 0x00);
	GC5004_write_cmos_sensor(0x73, 0x00);
	GC5004_write_cmos_sensor(0x74, 0x20);
	GC5004_write_cmos_sensor(0x75, 0x20);
	GC5004_write_cmos_sensor(0x76, 0x20);
	GC5004_write_cmos_sensor(0x77, 0x20);

	/////////////////////////////////////////////////////
	//////////////////////   GAIN   /////////////////////
	/////////////////////////////////////////////////////
	GC5004_write_cmos_sensor(0xb0, 0x50); 
	GC5004_write_cmos_sensor(0xb1, 0x01); 
	GC5004_write_cmos_sensor(0xb2, 0x02); 
	GC5004_write_cmos_sensor(0xb3, 0x40); 
	GC5004_write_cmos_sensor(0xb4, 0x40); 
	GC5004_write_cmos_sensor(0xb5, 0x40);
	GC5004_write_cmos_sensor(0xb6, 0x00);
	
	/////////////////////////////////////////////////////
	//////////////////////   DNDD   /////////////////////
	/////////////////////////////////////////////////////
	GC5004_write_cmos_sensor(0xfe, 0x02);
	GC5004_write_cmos_sensor(0x89, 0x15);
	GC5004_write_cmos_sensor(0xfe, 0x00);
	
	/////////////////////////////////////////////////////
	////////////////////    scalar   ////////////////////
	/////////////////////////////////////////////////////
	GC5004_write_cmos_sensor(0x18, 0x42);
	GC5004_write_cmos_sensor(0x80, 0x18);
	GC5004_write_cmos_sensor(0x84, 0x23);
	GC5004_write_cmos_sensor(0x87, 0x12);
	GC5004_write_cmos_sensor(0xfe, 0x02);
	GC5004_write_cmos_sensor(0x86, 0x00);	
	
	GC5004_write_cmos_sensor(0x95, 0x07);
	GC5004_write_cmos_sensor(0x96, 0x98);
	GC5004_write_cmos_sensor(0x97, 0x0a);
	GC5004_write_cmos_sensor(0x98, 0x20);

	/////////////////////////////////////////////////////
	//////////////////////   MIPI   /////////////////////
	/////////////////////////////////////////////////////
	GC5004_write_cmos_sensor(0xfe, 0x03); 
	GC5004_write_cmos_sensor(0x01, 0x00);
	GC5004_write_cmos_sensor(0x02, 0x00);
	GC5004_write_cmos_sensor(0x03, 0x00);
	GC5004_write_cmos_sensor(0x06, 0x00);
	GC5004_write_cmos_sensor(0x10, 0x00);
	GC5004_write_cmos_sensor(0x15, 0x00);
	GC5004_write_cmos_sensor(0x18, 0x00);
	GC5004_write_cmos_sensor(0x19, 0x00);
	GC5004_write_cmos_sensor(0x1a, 0x00);
	
	/////////////////////////////////////////////////////
	//////////////////////  outbuf  /////////////////////
	/////////////////////////////////////////////////////
	GC5004_write_cmos_sensor(0xfe, 0x03);
	GC5004_write_cmos_sensor(0x06, 0xc0);
	GC5004_write_cmos_sensor(0x10, 0x94);
	GC5004_write_cmos_sensor(0x15, 0x10);
	GC5004_write_cmos_sensor(0x12, 0x40);
	GC5004_write_cmos_sensor(0x13, 0x06);
	GC5004_write_cmos_sensor(0x04, 0x10);
	GC5004_write_cmos_sensor(0x05, 0x00);
	GC5004_write_cmos_sensor(0x42, 0x10);
	GC5004_write_cmos_sensor(0x43, 0x05);
	GC5004_write_cmos_sensor(0xfe, 0x00);
	GC5004_write_cmos_sensor(0x80, 0x19);

	/////////////////////////////////////////////////////
	/////////////////////   pad enable   ////////////////
	/////////////////////////////////////////////////////
	GC5004_write_cmos_sensor(0xfe, 0x00); 
	GC5004_write_cmos_sensor(0xf2, 0x0f);    
}
static void GC5004PreviewSettings(void)
{
	//1296x972
	GC5004_write_cmos_sensor(0x18, 0x42); //skip on
	GC5004_write_cmos_sensor(0x80, 0x19); //scaler en
  	GC5004_write_cmos_sensor(0xf7, 0x9d);
	GC5004_write_cmos_sensor(0xf8, 0x84);
	GC5004_write_cmos_sensor(0x03, 0x06);
	GC5004_write_cmos_sensor(0x04, 0xd6);
	GC5004_write_cmos_sensor(0x05, 0x01);
	GC5004_write_cmos_sensor(0x06, 0xfa);
	
	GC5004_write_cmos_sensor(0x0a, 0x03);
	GC5004_write_cmos_sensor(0x17, 0x35);

	GC5004_write_cmos_sensor(0x92, 0x00);
   	GC5004_write_cmos_sensor(0x94, 0x07);
	GC5004_write_cmos_sensor(0x95, 0x03);
	GC5004_write_cmos_sensor(0x96, 0xcc);
	GC5004_write_cmos_sensor(0x97, 0x05);
	GC5004_write_cmos_sensor(0x98, 0x10);  
	GC5004_write_cmos_sensor(0xfe, 0x03);
	GC5004_write_cmos_sensor(0x42, 0x10);
	GC5004_write_cmos_sensor(0x43, 0x05);
	GC5004_write_cmos_sensor(0xfe, 0x00);
}

static void GC5004CaptureSettings(void)
{
	//2592x1944
	GC5004_write_cmos_sensor(0x18, 0x02); //skip off
	GC5004_write_cmos_sensor(0x80, 0x10); //scaler off
	GC5004_write_cmos_sensor(0xf7, 0x9d);
	GC5004_write_cmos_sensor(0xf8, 0x83);
	GC5004_write_cmos_sensor(0x03, 0x06);
	GC5004_write_cmos_sensor(0x04, 0xd6);
	GC5004_write_cmos_sensor(0x05, 0x01);
	GC5004_write_cmos_sensor(0x06, 0x0a); 
	
	GC5004_write_cmos_sensor(0x0a, 0x02);
	GC5004_write_cmos_sensor(0x17, 0x15);
	
	GC5004_write_cmos_sensor(0x92, 0x04);
   	GC5004_write_cmos_sensor(0x94, 0x0d);
	GC5004_write_cmos_sensor(0x95, 0x07);
	GC5004_write_cmos_sensor(0x96, 0x98);
	GC5004_write_cmos_sensor(0x97, 0x0a);
	GC5004_write_cmos_sensor(0x98, 0x20);
	GC5004_write_cmos_sensor(0xfe, 0x03);
	GC5004_write_cmos_sensor(0x42, 0x20);
	GC5004_write_cmos_sensor(0x43, 0x0a);
	GC5004_write_cmos_sensor(0xfe, 0x00);
}
/*****************************************************************************/
/* Windows Mobile Sensor Interface */
/*****************************************************************************/
/*************************************************************************
* FUNCTION
*	GC5004Open
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

UINT32 GC5004Open(void)
{
	kal_uint16 sensor_id=0; 

	// check if sensor ID correct
	sensor_id=((GC5004_read_cmos_sensor(0xf0) << 8) | GC5004_read_cmos_sensor(0xf1));   
#ifdef GC5004_DRIVER_TRACE
	SENSORDB("GC5004Open, sensor_id:%x \n",sensor_id);
#endif		
	if (sensor_id != GC5004_SENSOR_ID)
		return ERROR_SENSOR_CONNECT_FAIL;
	
	/* initail sequence write in  */
	GC5004_Init_Settings();

	//GC5004_SetMirrorFlip(GC5004_sensor.Mirror);

	return ERROR_NONE;
}   /* GC5004Open  */

/*************************************************************************
* FUNCTION
*   GC5004GetSensorID
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
extern bool camera_pdn1_reverse;
UINT32 GC5004GetSensorID(UINT32 *sensorID) 
{
    mt_set_gpio_mode(GPIO_CAMERA_CMPDN1_PIN, GPIO_CAMERA_CMPDN1_PIN_M_GPIO);
	mt_set_gpio_dir(GPIO_CAMERA_CMPDN1_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_CAMERA_CMPDN1_PIN, GPIO_OUT_ZERO);
	msleep(10);
	// check if sensor ID correct
	*sensorID=((GC5004_read_cmos_sensor(0xf0) << 8) | GC5004_read_cmos_sensor(0xf1));	
#ifdef GC5004_DRIVER_TRACE
	SENSORDB("GC5004GetSensorID:%x \n",*sensorID);
#endif		
	if (*sensorID != GC5004_SENSOR_ID) {		
		*sensorID = 0xFFFFFFFF;		
		return ERROR_SENSOR_CONNECT_FAIL;
	}
    camera_pdn1_reverse = 1;
   return ERROR_NONE;
}

/*************************************************************************
* FUNCTION
*	GC5004Close
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
UINT32 GC5004Close(void)
{
#ifdef GC5004_DRIVER_TRACE
   SENSORDB("GC5004Close\n");
#endif
  //kdCISModulePowerOn(SensorIdx,currSensorName,0,mode_name);
//	DRV_I2CClose(GC5004hDrvI2C);
	return ERROR_NONE;
}   /* GC5004Close */

/*************************************************************************
* FUNCTION
* GC5004Preview
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
UINT32 GC5004Preview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{	
	spin_lock(&gc5004_drv_lock);	
	GC5004_sensor.pv_mode = KAL_TRUE;
	GC5004_sensor.cap_mode = KAL_FALSE;

	//GC5004_set_mirror(sensor_config_data->SensorImageMirror);
	switch (sensor_config_data->SensorOperationMode)
	{
	  case MSDK_SENSOR_OPERATION_MODE_VIDEO: 	  	
		GC5004_sensor.video_mode = KAL_TRUE;
	  default: /* ISP_PREVIEW_MODE */
		GC5004_sensor.video_mode = KAL_FALSE;
	}
	GC5004_sensor.line_length = GC5004_PV_PERIOD_PIXEL_NUMS;
	GC5004_sensor.frame_height = GC5004_PV_PERIOD_LINE_NUMS;
	GC5004_sensor.pclk = GC5004_PREVIEW_CLK;
	spin_unlock(&gc5004_drv_lock);
	GC5004PreviewSettings();
	//GC5004_Write_Shutter(GC5004_sensor.shutter);
#ifdef GC5004_DRIVER_TRACE
	SENSORDB("GC5004Preview\n");
#endif

	return ERROR_NONE;
}   /*  GC5004Preview   */

/*************************************************************************
* FUNCTION
*	GC5004Capture
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
UINT32 GC5004Capture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
						  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	kal_uint16 Cap_shutter;
	spin_lock(&gc5004_drv_lock);
	GC5004_sensor.video_mode = KAL_FALSE;
	GC5004_sensor.pv_mode = KAL_FALSE;
	GC5004_sensor.cap_mode = KAL_TRUE;
	GC5004_sensor.line_length = GC5004_FULL_PERIOD_PIXEL_NUMS;
	GC5004_sensor.frame_height = GC5004_FULL_PERIOD_LINE_NUMS;
	GC5004_sensor.pclk = GC5004_CAPTURE_CLK;
	spin_unlock(&gc5004_drv_lock);
	//Cap_shutter = GC5004_read_shutter();
	GC5004CaptureSettings();
	//Cap_shutter = Cap_shutter / 2;
	//GC5004_set_shutter(Cap_shutter);
#ifdef GC5004_DRIVER_TRACE
	SENSORDB("GC5004Capture\n");
#endif
	return ERROR_NONE;
}   /* GC5004_Capture() */

UINT32 GC5004GetResolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *pSensorResolution)
{
	pSensorResolution->SensorFullWidth=GC5004_IMAGE_SENSOR_FULL_WIDTH;
	pSensorResolution->SensorFullHeight=GC5004_IMAGE_SENSOR_FULL_HEIGHT;
	pSensorResolution->SensorPreviewWidth=GC5004_IMAGE_SENSOR_PV_WIDTH;
	pSensorResolution->SensorPreviewHeight=GC5004_IMAGE_SENSOR_PV_HEIGHT;
	pSensorResolution->SensorVideoWidth	= GC5004_IMAGE_SENSOR_VIDEO_WIDTH;
	pSensorResolution->SensorVideoHeight    = GC5004_IMAGE_SENSOR_VIDEO_HEIGHT;
	return ERROR_NONE;
}	/* GC5004GetResolution() */

UINT32 GC5004GetInfo(MSDK_SCENARIO_ID_ENUM ScenarioId,
					  MSDK_SENSOR_INFO_STRUCT *pSensorInfo,
					  MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
	pSensorInfo->SensorPreviewResolutionX=GC5004_IMAGE_SENSOR_PV_WIDTH;
	pSensorInfo->SensorPreviewResolutionY=GC5004_IMAGE_SENSOR_PV_HEIGHT;
	pSensorInfo->SensorFullResolutionX=GC5004_IMAGE_SENSOR_FULL_WIDTH;
	pSensorInfo->SensorFullResolutionY=GC5004_IMAGE_SENSOR_FULL_HEIGHT;

	pSensorInfo->SensorCameraPreviewFrameRate=30;
	pSensorInfo->SensorVideoFrameRate=30;
	pSensorInfo->SensorStillCaptureFrameRate=10;
	pSensorInfo->SensorWebCamCaptureFrameRate=15;
	pSensorInfo->SensorResetActiveHigh=TRUE; //low active
	pSensorInfo->SensorResetDelayCount=5; 
	pSensorInfo->SensorOutputDataFormat=GC5004_COLOR_FORMAT;
	pSensorInfo->SensorClockPolarity=SENSOR_CLOCK_POLARITY_LOW;	
	pSensorInfo->SensorClockFallingPolarity=SENSOR_CLOCK_POLARITY_LOW;
	pSensorInfo->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	pSensorInfo->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	pSensorInfo->SensroInterfaceType=SENSOR_INTERFACE_TYPE_PARALLEL;

	pSensorInfo->CaptureDelayFrame = 2; 
	pSensorInfo->PreviewDelayFrame = 1;
	pSensorInfo->VideoDelayFrame = 1;

	pSensorInfo->SensorMasterClockSwitch = 0; 
	pSensorInfo->SensorDrivingCurrent = ISP_DRIVING_8MA;
	pSensorInfo->AEShutDelayFrame =0;		   /* The frame of setting shutter default 0 for TG int */
	pSensorInfo->AESensorGainDelayFrame = 0;   /* The frame of setting sensor gain */
	pSensorInfo->AEISPGainDelayFrame = 4;  

	switch (ScenarioId)
	{
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			pSensorInfo->SensorClockFreq=24;
			pSensorInfo->SensorClockDividCount=3;
			pSensorInfo->SensorClockRisingCount= 0;
			pSensorInfo->SensorClockFallingCount= 2;
			pSensorInfo->SensorPixelClockCount= 3;
			pSensorInfo->SensorDataLatchCount= 2;
			pSensorInfo->SensorGrabStartX = GC5004_PV_X_START; 
			pSensorInfo->SensorGrabStartY = GC5004_PV_Y_START; 

		break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			pSensorInfo->SensorClockFreq=24;
			pSensorInfo->SensorClockDividCount=3;
			pSensorInfo->SensorClockRisingCount= 0;
			pSensorInfo->SensorClockFallingCount= 2;
			pSensorInfo->SensorPixelClockCount= 3;
			pSensorInfo->SensorDataLatchCount= 2;
			pSensorInfo->SensorGrabStartX = GC5004_VIDEO_X_START; 
			pSensorInfo->SensorGrabStartY = GC5004_VIDEO_Y_START; 

		break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
			pSensorInfo->SensorClockFreq=24;
			pSensorInfo->SensorClockDividCount= 3;
			pSensorInfo->SensorClockRisingCount=0;
			pSensorInfo->SensorClockFallingCount=2;
			pSensorInfo->SensorPixelClockCount=3;
			pSensorInfo->SensorDataLatchCount=2;
			pSensorInfo->SensorGrabStartX = GC5004_FULL_X_START; 
			pSensorInfo->SensorGrabStartY = GC5004_FULL_Y_START; 
		break;
		default:
			pSensorInfo->SensorClockFreq=24;
			pSensorInfo->SensorClockDividCount=3;
			pSensorInfo->SensorClockRisingCount=0;
			pSensorInfo->SensorClockFallingCount=2;		
			pSensorInfo->SensorPixelClockCount=3;
			pSensorInfo->SensorDataLatchCount=2;
			pSensorInfo->SensorGrabStartX = GC5004_PV_X_START; 
			pSensorInfo->SensorGrabStartY = GC5004_PV_Y_START; 
		break;
	}
	memcpy(pSensorConfigData, &GC5004_sensor.cfg_data, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
  return ERROR_NONE;
}	/* GC5004GetInfo() */


UINT32 GC5004Control(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *pImageWindow,
					  MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{

#ifdef GC5004_DRIVER_TRACE
	SENSORDB("GC5004Control ScenarioId = %d \n",ScenarioId);
#endif
	switch (ScenarioId)
	{
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			GC5004Preview(pImageWindow, pSensorConfigData);
		break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
			GC5004Capture(pImageWindow, pSensorConfigData);
		break;		
	        default:
	            return ERROR_INVALID_SCENARIO_ID;
	}
	return ERROR_NONE;
}	/* GC5004Control() */



UINT32 GC5004SetVideoMode(UINT16 u2FrameRate)
{};

UINT32 GC5004FeatureControl(MSDK_SENSOR_FEATURE_ENUM FeatureId,
							 UINT8 *pFeaturePara,UINT32 *pFeatureParaLen)
{
	UINT16 *pFeatureReturnPara16=(UINT16 *) pFeaturePara;
	UINT16 *pFeatureData16=(UINT16 *) pFeaturePara;
	UINT32 *pFeatureReturnPara32=(UINT32 *) pFeaturePara;
	//UINT32 *pFeatureData32=(UINT32 *) pFeaturePara;
	//UINT32 GC5004SensorRegNumber;
	//UINT32 i;
	//PNVRAM_SENSOR_DATA_STRUCT pSensorDefaultData=(PNVRAM_SENSOR_DATA_STRUCT) pFeaturePara;
	//MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData=(MSDK_SENSOR_CONFIG_STRUCT *) pFeaturePara;
	MSDK_SENSOR_REG_INFO_STRUCT *pSensorRegData=(MSDK_SENSOR_REG_INFO_STRUCT *) pFeaturePara;
	//MSDK_SENSOR_GROUP_INFO_STRUCT *pSensorGroupInfo=(MSDK_SENSOR_GROUP_INFO_STRUCT *) pFeaturePara;
	//MSDK_SENSOR_ITEM_INFO_STRUCT *pSensorItemInfo=(MSDK_SENSOR_ITEM_INFO_STRUCT *) pFeaturePara;
	//MSDK_SENSOR_ENG_INFO_STRUCT	*pSensorEngInfo=(MSDK_SENSOR_ENG_INFO_STRUCT *) pFeaturePara;

	switch (FeatureId)
	{
		case SENSOR_FEATURE_GET_RESOLUTION:
			*pFeatureReturnPara16++=GC5004_IMAGE_SENSOR_FULL_WIDTH;
			*pFeatureReturnPara16=GC5004_IMAGE_SENSOR_FULL_HEIGHT;
			*pFeatureParaLen=4;
		break;
		case SENSOR_FEATURE_GET_PERIOD:	/* 3 */
			*pFeatureReturnPara16++=GC5004_sensor.line_length;
			*pFeatureReturnPara16=GC5004_sensor.frame_height;
			*pFeatureParaLen=4;
		break;
		case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:  /* 3 */
			*pFeatureReturnPara32 = GC5004_sensor.pclk;
			*pFeatureParaLen=4;
		break;
		case SENSOR_FEATURE_SET_ESHUTTER:	/* 4 */
			GC5004_set_shutter(*pFeatureData16);
		break;
		case SENSOR_FEATURE_SET_NIGHTMODE:
			GC5004_night_mode((BOOL) *pFeatureData16);
		break;
		case SENSOR_FEATURE_SET_GAIN:	/* 6 */			
			GC5004_SetGain((UINT16) *pFeatureData16);
		break;
		case SENSOR_FEATURE_SET_FLASHLIGHT:
		break;
		case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
		break;
		case SENSOR_FEATURE_SET_REGISTER:
			GC5004_write_cmos_sensor(pSensorRegData->RegAddr, pSensorRegData->RegData);
		break;
		case SENSOR_FEATURE_GET_REGISTER:
			pSensorRegData->RegData = GC5004_read_cmos_sensor(pSensorRegData->RegAddr);
		break;
		case SENSOR_FEATURE_SET_CCT_REGISTER:
			memcpy(&GC5004_sensor.eng.cct, pFeaturePara, sizeof(GC5004_sensor.eng.cct));
			break;
		break;
		case SENSOR_FEATURE_GET_CCT_REGISTER:	/* 12 */
			if (*pFeatureParaLen >= sizeof(GC5004_sensor.eng.cct) + sizeof(kal_uint32))
			{
			  *((kal_uint32 *)pFeaturePara++) = sizeof(GC5004_sensor.eng.cct);
			  memcpy(pFeaturePara, &GC5004_sensor.eng.cct, sizeof(GC5004_sensor.eng.cct));
			}
			break;
		case SENSOR_FEATURE_SET_ENG_REGISTER:
			memcpy(&GC5004_sensor.eng.reg, pFeaturePara, sizeof(GC5004_sensor.eng.reg));
			break;
		case SENSOR_FEATURE_GET_ENG_REGISTER:	/* 14 */
			if (*pFeatureParaLen >= sizeof(GC5004_sensor.eng.reg) + sizeof(kal_uint32))
			{
			  *((kal_uint32 *)pFeaturePara++) = sizeof(GC5004_sensor.eng.reg);
			  memcpy(pFeaturePara, &GC5004_sensor.eng.reg, sizeof(GC5004_sensor.eng.reg));
			}
		case SENSOR_FEATURE_GET_REGISTER_DEFAULT:
			((PNVRAM_SENSOR_DATA_STRUCT)pFeaturePara)->Version = NVRAM_CAMERA_SENSOR_FILE_VERSION;
			((PNVRAM_SENSOR_DATA_STRUCT)pFeaturePara)->SensorId = GC5004_SENSOR_ID;
			memcpy(((PNVRAM_SENSOR_DATA_STRUCT)pFeaturePara)->SensorEngReg, &GC5004_sensor.eng.reg, sizeof(GC5004_sensor.eng.reg));
			memcpy(((PNVRAM_SENSOR_DATA_STRUCT)pFeaturePara)->SensorCCTReg, &GC5004_sensor.eng.cct, sizeof(GC5004_sensor.eng.cct));
			*pFeatureParaLen = sizeof(NVRAM_SENSOR_DATA_STRUCT);
			break;
		case SENSOR_FEATURE_GET_CONFIG_PARA:
			memcpy(pFeaturePara, &GC5004_sensor.cfg_data, sizeof(GC5004_sensor.cfg_data));
			*pFeatureParaLen = sizeof(GC5004_sensor.cfg_data);
			break;
		case SENSOR_FEATURE_CAMERA_PARA_TO_SENSOR:
		     GC5004_camera_para_to_sensor();
			break;
		case SENSOR_FEATURE_SENSOR_TO_CAMERA_PARA:
			GC5004_sensor_to_camera_para();
			break;							
		case SENSOR_FEATURE_GET_GROUP_COUNT:
			GC5004_get_sensor_group_count((kal_uint32 *)pFeaturePara);
			*pFeatureParaLen = 4;
			break;
		case SENSOR_FEATURE_GET_GROUP_INFO:
			GC5004_get_sensor_group_info((MSDK_SENSOR_GROUP_INFO_STRUCT *)pFeaturePara);
			*pFeatureParaLen = sizeof(MSDK_SENSOR_GROUP_INFO_STRUCT);
			break;
		case SENSOR_FEATURE_GET_ITEM_INFO:
			GC5004_get_sensor_item_info((MSDK_SENSOR_ITEM_INFO_STRUCT *)pFeaturePara);
			*pFeatureParaLen = sizeof(MSDK_SENSOR_ITEM_INFO_STRUCT);
			break;
		case SENSOR_FEATURE_SET_ITEM_INFO:
			GC5004_set_sensor_item_info((MSDK_SENSOR_ITEM_INFO_STRUCT *)pFeaturePara);
			*pFeatureParaLen = sizeof(MSDK_SENSOR_ITEM_INFO_STRUCT);
			break;
		case SENSOR_FEATURE_GET_ENG_INFO:
			memcpy(pFeaturePara, &GC5004_sensor.eng_info, sizeof(GC5004_sensor.eng_info));
			*pFeatureParaLen = sizeof(GC5004_sensor.eng_info);
			break;
		case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
			// get the lens driver ID from EEPROM or just return LENS_DRIVER_ID_DO_NOT_CARE
			// if EEPROM does not exist in camera module.
			*pFeatureReturnPara32=LENS_DRIVER_ID_DO_NOT_CARE;
			*pFeatureParaLen=4;
		break;
		case SENSOR_FEATURE_SET_VIDEO_MODE:
			//GC5004SetVideoMode(*pFeatureData16);
			break; 
		case SENSOR_FEATURE_CHECK_SENSOR_ID:
			GC5004GetSensorID(pFeatureReturnPara32); 
			break; 
		case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE:
			break;
		default:
			break;
	}
	return ERROR_NONE;
}	/* GC5004FeatureControl() */
SENSOR_FUNCTION_STRUCT	SensorFuncGC5004=
{
	GC5004Open,
	GC5004GetInfo,
	GC5004GetResolution,
	GC5004FeatureControl,
	GC5004Control,
	GC5004Close
};

UINT32 GC5004_RAW_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
	/* To Do : Check Sensor status here */
	if (pfFunc!=NULL)
		*pfFunc=&SensorFuncGC5004;

	return ERROR_NONE;
}	/* SensorInit() */
