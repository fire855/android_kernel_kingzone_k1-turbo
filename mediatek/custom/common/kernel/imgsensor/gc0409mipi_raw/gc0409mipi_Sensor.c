/*****************************************************************************
 *
 * Filename:
 * ---------
 *   gc0409mipi_Sensor.c
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
 *   
 *
 *============================================================================
 *             HISTORY
 *------------------------------------------------------------------------------
 * $Revision:$
 * $Modtime:$
 * $Log:$
 *
 * 20150414	Beta1.0	Travis		
 * 20150428	Beta2.1		
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

#include "gc0409mipi_Sensor.h"
#include "gc0409mipi_Camera_Sensor_para.h"
#include "gc0409mipi_CameraCustomized.h"

#ifdef GC0409MIPI_DEBUG
#define SENSORDB printk
#else
#define SENSORDB(x,...)
#endif

#define GC0409MIPI_TEST_PATTERN_CHECKSUM 0x54ddf19b 

//#define IMAGE_NORMAL 
//#define IMAGE_H_MIRROR 
//#define IMAGE_V_MIRROR 
#define IMAGE_HV_MIRROR 

#ifdef IMAGE_NORMAL
#define MIRROR 0x54
#define BLK_VAL 0x3c
#define STARTX 0x01
#define STARTY 0x01
#endif

#ifdef IMAGE_H_MIRROR
#define MIRROR 0x55
#define BLK_VAL 0x3c
#define STARTX 0x01
#define STARTY 0x02
#endif

#ifdef IMAGE_V_MIRROR
#define MIRROR 0x56
#define BLK_VAL 0xc3
#define STARTX 0x02
#define STARTY 0x01
#endif

#ifdef IMAGE_HV_MIRROR
#define MIRROR 0x57
#define BLK_VAL 0xc3
#define STARTX 0x02
#define STARTY 0x02
#endif

static DEFINE_SPINLOCK(gc0409mipi_drv_lock);

static kal_bool b_Enable_Half = false;
extern int iReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 * a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
extern int iWriteRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u16 i2cId);


static GC0409MIPI_sensor_struct GC0409MIPI_sensor =
{
	.eng_info =
	{
		.SensorId = 128,
		.SensorType = CMOS_SENSOR,
		.SensorOutputDataFormat = GC0409MIPI_COLOR_FORMAT,
	},
	.Mirror = GC0409MIPI_IMAGE_V_MIRROR,
	.shutter = 0x20,  
	.gain = 0x20,
	.pclk = GC0409MIPI_PREVIEW_CLK,
	.frame_height = GC0409MIPI_PV_PERIOD_LINE_NUMS,
	.line_length = GC0409MIPI_PV_PERIOD_PIXEL_NUMS,
};


/*************************************************************************
* FUNCTION
*    GC0409MIPI_write_cmos_sensor
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
static void GC0409MIPI_write_cmos_sensor(kal_uint8 addr, kal_uint8 para)
{
kal_uint8 out_buff[2];

    out_buff[0] = addr;
    out_buff[1] = para;

    iWriteRegI2C((u8*)out_buff , (u16)sizeof(out_buff), GC0409MIPI_WRITE_ID); 
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
static kal_uint8 GC0409MIPI_read_cmos_sensor(kal_uint8 addr)
{
  kal_uint8 in_buff[1] = {0xFF};
  kal_uint8 out_buff[1];
  
  out_buff[0] = addr;

    if (0 != iReadRegI2C((u8*)out_buff , (u16) sizeof(out_buff), (u8*)in_buff, (u16)sizeof(in_buff), GC0409MIPI_WRITE_ID)) {
        SENSORDB("ERROR: GC0409MIPI_read_cmos_sensor \n");
    }
  return in_buff[0];
}

/*************************************************************************
* FUNCTION
*	GC0409MIPI_SetShutter
*
* DESCRIPTION
*	This function set e-shutter of GC0409MIPI to change exposure time.
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
void GC0409MIPI_set_shutter(kal_uint16 iShutter)
{
	spin_lock(&gc0409mipi_drv_lock);
	GC0409MIPI_sensor.shutter = iShutter;
	spin_unlock(&gc0409mipi_drv_lock);

	if (!iShutter) iShutter = 6; /* avoid 0 */
	
#ifdef GC0409MIPI_DRIVER_TRACE
	SENSORDB("GC0409MIPI_set_shutter iShutter = %d \n",iShutter);
#endif

	if(iShutter < 6) iShutter = 6;
	if(iShutter > 4095) iShutter = 4095;//2^12
/*
	if(iShutter < 480)
	{
		GC0409MIPI_write_cmos_sensor(0x18,0x02);
	}
	else
	{
		GC0409MIPI_write_cmos_sensor(0x18,0x12);		
	}
*/	
	//Update Shutter
	GC0409MIPI_write_cmos_sensor(0x04, (iShutter) & 0xFF);
	GC0409MIPI_write_cmos_sensor(0x03, (iShutter >> 8) & 0x0F);	
}   /*  Set_GC0409MIPI_Shutter */

/*************************************************************************
* FUNCTION
*	GC0409MIPI_SetGain
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
#define ANALOG_GAIN_2 87  // 1.36x
#define ANALOG_GAIN_3 119  // 1.86x
#define ANALOG_GAIN_4 161  // 2.52x
#define ANALOG_GAIN_5 243  // 3.79x
#define ANALOG_GAIN_6 340  // 5.32x
#define ANALOG_GAIN_7 484  // 7.57x

kal_uint16  GC0409MIPI_SetGain(kal_uint16 iGain)
{
	kal_uint16 iReg,temp,luma_value;
		
	#ifdef GC0409MIPI_DRIVER_TRACE
	SENSORDB("GC0409MIPI_SetGain iGain = %d \n",iGain);
	#endif
	
	iReg = iGain;
	if(iReg < 0x40)
		iReg = 0x40;
	if((ANALOG_GAIN_1<= iReg)&&(iReg < ANALOG_GAIN_2))
	{
		//analog gain
		GC0409MIPI_write_cmos_sensor(0xb6,  0x00); 
		temp = iReg;
		GC0409MIPI_write_cmos_sensor(0xb1, temp>>6);
		GC0409MIPI_write_cmos_sensor(0xb2, (temp<<2)&0xfc);
		//SENSORDB("GC0409 analogic gain 1x , add pregain = %d\n",temp);	
	}
	else if(ANALOG_GAIN_2<= iReg)
	{
		//analog gain
		GC0409MIPI_write_cmos_sensor(0xb6,  0x01); 
		temp = 64*iReg/ANALOG_GAIN_2;
		GC0409MIPI_write_cmos_sensor(0xb1, temp>>6);
		GC0409MIPI_write_cmos_sensor(0xb2, (temp<<2)&0xfc);
		//SENSORDB("GC0409 analogic gain 1.36x , add pregain = %d\n",temp);
	}
	
/* Again > 4x
	if(iReg < 0x40)
		iReg = 0x40;
	if((ANALOG_GAIN_1<= iReg)&&(iReg < ANALOG_GAIN_2))
	{
		GC0409MIPI_write_cmos_sensor(0x29, 0x0f); 
		GC0409MIPI_write_cmos_sensor(0xb0, 0x79);	
		//analog gain
		GC0409MIPI_write_cmos_sensor(0xb6,  0x00); 
		temp = iReg;
		GC0409MIPI_write_cmos_sensor(0xb1, temp>>6);
		GC0409MIPI_write_cmos_sensor(0xb2, (temp<<2)&0xfc);
		//SENSORDB("GC0409 analogic gain 1x , add pregain = %d\n",temp);	
	}
	else if((ANALOG_GAIN_2<= iReg)&&(iReg < ANALOG_GAIN_3))
	{
		GC0409MIPI_write_cmos_sensor(0x29, 0x0f); 
		GC0409MIPI_write_cmos_sensor(0xb0, 0x79);	
		//analog gain
		GC0409MIPI_write_cmos_sensor(0xb6,  0x01); 
		temp = 64*iReg/ANALOG_GAIN_2;
		GC0409MIPI_write_cmos_sensor(0xb1, temp>>6);
		GC0409MIPI_write_cmos_sensor(0xb2, (temp<<2)&0xfc);
		//SENSORDB("GC0409 analogic gain 1.36x , add pregain = %d\n",temp);
	}
	else if((ANALOG_GAIN_3<= iReg)&&(iReg < ANALOG_GAIN_4))
	{
		GC0409MIPI_write_cmos_sensor(0x29, 0x14); 
		GC0409MIPI_write_cmos_sensor(0xb0, 0x79);	
		//analog gain
		GC0409MIPI_write_cmos_sensor(0xb6,  0x02);
		temp = 64*iReg/ANALOG_GAIN_3;
		GC0409MIPI_write_cmos_sensor(0xb1, temp>>6);
		GC0409MIPI_write_cmos_sensor(0xb2, (temp<<2)&0xfc);
		//SENSORDB("GC0409 analogic gain 1.86x , add pregain = %d\n",temp);
	}	
	else if((ANALOG_GAIN_4<= iReg)&&(iReg < ANALOG_GAIN_5))
	{
		GC0409MIPI_write_cmos_sensor(0x29, 0x14); 
		GC0409MIPI_write_cmos_sensor(0xb0, 0x79);
		//analog gain
		GC0409MIPI_write_cmos_sensor(0xb6,	0x03);
		temp = 64*iReg/ANALOG_GAIN_4;
		GC0409MIPI_write_cmos_sensor(0xb1, temp>>6);
		GC0409MIPI_write_cmos_sensor(0xb2, (temp<<2)&0xfc);
		//SENSORDB("GC0409 analogic gain 2.52x , add pregain = %d\n",temp);
	}
	else if((ANALOG_GAIN_5<= iReg)&&(iReg < ANALOG_GAIN_6))
	{
		GC0409MIPI_write_cmos_sensor(0x29, 0x14); 
		GC0409MIPI_write_cmos_sensor(0xb0, 0x7c);	
		//analog gain
		GC0409MIPI_write_cmos_sensor(0xb6,  0x04);
		temp = 64*iReg/ANALOG_GAIN_5;
		GC0409MIPI_write_cmos_sensor(0xb1, temp>>6);
		GC0409MIPI_write_cmos_sensor(0xb2, (temp<<2)&0xfc);
		//SENSORDB("GC0409 analogic gain 3.79x , add pregain = %d\n",temp);
	}
	else if((ANALOG_GAIN_6<= iReg)&&(iReg < ANALOG_GAIN_7))
	{
		GC0409MIPI_write_cmos_sensor(0x29, 0x14); 
		GC0409MIPI_write_cmos_sensor(0xb0, 0x7c);	
		//analog gain
		GC0409MIPI_write_cmos_sensor(0xb6,  0x05);
		temp = 64*iReg/ANALOG_GAIN_6;
		GC0409MIPI_write_cmos_sensor(0xb1, temp>>6);
		GC0409MIPI_write_cmos_sensor(0xb2, (temp<<2)&0xfc);
		//SENSORDB("GC0409 analogic gain 5.32x , add pregain = %d\n",temp);
	}
	else if(ANALOG_GAIN_7<= iReg)
	{	
		GC0409MIPI_write_cmos_sensor(0x29, 0x14); 
		GC0409MIPI_write_cmos_sensor(0xb0, 0x7c);
		//analog gain
		GC0409MIPI_write_cmos_sensor(0xb6,  0x06);
		temp = 64*iReg/ANALOG_GAIN_7;
		GC0409MIPI_write_cmos_sensor(0xb1, temp>>6);
		GC0409MIPI_write_cmos_sensor(0xb2, (temp<<2)&0xfc);
		//SENSORDB("GC0409 analogic gain 7.57x , add pregain = %d\n",temp);
	}
*/
	return iGain;
}
/*************************************************************************
* FUNCTION
*	GC0409MIPI_NightMode
*
* DESCRIPTION
*	This function night mode of GC0409MIPI.
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
void GC0409MIPI_night_mode(kal_bool enable)
{
/*No Need to implement this function*/
#if 0 
	const kal_uint16 dummy_pixel = GC0409MIPI_sensor.line_length - GC0409MIPI_PV_PERIOD_PIXEL_NUMS;
	const kal_uint16 pv_min_fps =  enable ? GC0409MIPI_sensor.night_fps : GC0409MIPI_sensor.normal_fps;
	kal_uint16 dummy_line = GC0409MIPI_sensor.frame_height - GC0409MIPI_PV_PERIOD_LINE_NUMS;
	kal_uint16 max_exposure_lines;
	
	printk("[GC0409MIPI_night_mode]enable=%d",enable);
	if (!GC0409MIPI_sensor.video_mode) return;
	max_exposure_lines = GC0409MIPI_sensor.pclk * GC0409MIPI_FPS(1) / (pv_min_fps * GC0409MIPI_sensor.line_length);
	if (max_exposure_lines > GC0409MIPI_sensor.frame_height) /* fix max frame rate, AE table will fix min frame rate */
//	{
//	  dummy_line = max_exposure_lines - GC0409MIPI_PV_PERIOD_LINE_NUMS;
//	}
#endif
}   /*  GC0409MIPI_NightMode    */


/* write camera_para to sensor register */
static void GC0409MIPI_camera_para_to_sensor(void)
{
  kal_uint32 i;
#ifdef GC0409MIPI_DRIVER_TRACE
	 SENSORDB("GC0409MIPI_camera_para_to_sensor\n");
#endif
  for (i = 0; 0xFFFFFFFF != GC0409MIPI_sensor.eng.reg[i].Addr; i++)
  {
    GC0409MIPI_write_cmos_sensor(GC0409MIPI_sensor.eng.reg[i].Addr, GC0409MIPI_sensor.eng.reg[i].Para);
  }
  for (i = GC0409MIPI_FACTORY_START_ADDR; 0xFFFFFFFF != GC0409MIPI_sensor.eng.reg[i].Addr; i++)
  {
    GC0409MIPI_write_cmos_sensor(GC0409MIPI_sensor.eng.reg[i].Addr, GC0409MIPI_sensor.eng.reg[i].Para);
  }
  GC0409MIPI_SetGain(GC0409MIPI_sensor.gain); /* update gain */
}

/* update camera_para from sensor register */
static void GC0409MIPI_sensor_to_camera_para(void)
{
  kal_uint32 i,temp_data;
#ifdef GC0409MIPI_DRIVER_TRACE
   SENSORDB("GC0409MIPI_sensor_to_camera_para\n");
#endif
  for (i = 0; 0xFFFFFFFF != GC0409MIPI_sensor.eng.reg[i].Addr; i++)
  {
  	temp_data = GC0409MIPI_read_cmos_sensor(GC0409MIPI_sensor.eng.reg[i].Addr);

	spin_lock(&gc0409mipi_drv_lock);
    GC0409MIPI_sensor.eng.reg[i].Para = temp_data;
	spin_unlock(&gc0409mipi_drv_lock);
	
  }
  for (i = GC0409MIPI_FACTORY_START_ADDR; 0xFFFFFFFF != GC0409MIPI_sensor.eng.reg[i].Addr; i++)
  {
  	temp_data = GC0409MIPI_read_cmos_sensor(GC0409MIPI_sensor.eng.reg[i].Addr);

	spin_lock(&gc0409mipi_drv_lock);
    GC0409MIPI_sensor.eng.reg[i].Para = temp_data;
	spin_unlock(&gc0409mipi_drv_lock);
  }
}

/* ------------------------ Engineer mode ------------------------ */
inline static void GC0409MIPI_get_sensor_group_count(kal_int32 *sensor_count_ptr)
{
#ifdef GC0409MIPI_DRIVER_TRACE
   SENSORDB("GC0409MIPI_get_sensor_group_count\n");
#endif
  *sensor_count_ptr = GC0409MIPI_GROUP_TOTAL_NUMS;
}

inline static void GC0409MIPI_get_sensor_group_info(MSDK_SENSOR_GROUP_INFO_STRUCT *para)
{
#ifdef GC0409MIPI_DRIVER_TRACE
   SENSORDB("GC0409MIPI_get_sensor_group_info\n");
#endif
  switch (para->GroupIdx)
  {
  case GC0409MIPI_PRE_GAIN:
    sprintf(para->GroupNamePtr, "CCT");
    para->ItemCount = 5;
    break;
  case GC0409MIPI_CMMCLK_CURRENT:
    sprintf(para->GroupNamePtr, "CMMCLK Current");
    para->ItemCount = 1;
    break;
  case GC0409MIPI_FRAME_RATE_LIMITATION:
    sprintf(para->GroupNamePtr, "Frame Rate Limitation");
    para->ItemCount = 2;
    break;
  case GC0409MIPI_REGISTER_EDITOR:
    sprintf(para->GroupNamePtr, "Register Editor");
    para->ItemCount = 2;
    break;
  default:
    ASSERT(0);
  }
}

inline static void GC0409MIPI_get_sensor_item_info(MSDK_SENSOR_ITEM_INFO_STRUCT *para)
{

  const static kal_char *cct_item_name[] = {"SENSOR_BASEGAIN", "Pregain-R", "Pregain-Gr", "Pregain-Gb", "Pregain-B"};
  const static kal_char *editer_item_name[] = {"REG addr", "REG value"};
  
#ifdef GC0409MIPI_DRIVER_TRACE
	 SENSORDB("GC0409MIPI_get_sensor_item_info\n");
#endif
  switch (para->GroupIdx)
  {
  case GC0409MIPI_PRE_GAIN:
    switch (para->ItemIdx)
    {
    case GC0409MIPI_SENSOR_BASEGAIN:
    case GC0409MIPI_PRE_GAIN_R_INDEX:
    case GC0409MIPI_PRE_GAIN_Gr_INDEX:
    case GC0409MIPI_PRE_GAIN_Gb_INDEX:
    case GC0409MIPI_PRE_GAIN_B_INDEX:
      break;
    default:
      ASSERT(0);
    }
    sprintf(para->ItemNamePtr, cct_item_name[para->ItemIdx - GC0409MIPI_SENSOR_BASEGAIN]);
    para->ItemValue = GC0409MIPI_sensor.eng.cct[para->ItemIdx].Para * 1000 / BASEGAIN;
    para->IsTrueFalse = para->IsReadOnly = para->IsNeedRestart = KAL_FALSE;
    para->Min = GC0409MIPI_MIN_ANALOG_GAIN * 1000;
    para->Max = GC0409MIPI_MAX_ANALOG_GAIN * 1000;
    break;
  case GC0409MIPI_CMMCLK_CURRENT:
    switch (para->ItemIdx)
    {
    case 0:
      sprintf(para->ItemNamePtr, "Drv Cur[2,4,6,8]mA");
      switch (GC0409MIPI_sensor.eng.reg[GC0409MIPI_CMMCLK_CURRENT_INDEX].Para)
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
  case GC0409MIPI_FRAME_RATE_LIMITATION:
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
  case GC0409MIPI_REGISTER_EDITOR:
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

inline static kal_bool GC0409MIPI_set_sensor_item_info(MSDK_SENSOR_ITEM_INFO_STRUCT *para)
{
  kal_uint16 temp_para;
#ifdef GC0409MIPI_DRIVER_TRACE
   SENSORDB("GC0409MIPI_set_sensor_item_info\n");
#endif
  switch (para->GroupIdx)
  {
  case GC0409MIPI_PRE_GAIN:
    switch (para->ItemIdx)
    {
    case GC0409MIPI_SENSOR_BASEGAIN:
    case GC0409MIPI_PRE_GAIN_R_INDEX:
    case GC0409MIPI_PRE_GAIN_Gr_INDEX:
    case GC0409MIPI_PRE_GAIN_Gb_INDEX:
    case GC0409MIPI_PRE_GAIN_B_INDEX:

	spin_lock(&gc0409mipi_drv_lock);
      GC0409MIPI_sensor.eng.cct[para->ItemIdx].Para = para->ItemValue * BASEGAIN / 1000;
	spin_unlock(&gc0409mipi_drv_lock);
	  
      GC0409MIPI_SetGain(GC0409MIPI_sensor.gain); /* update gain */
      break;
    default:
      ASSERT(0);
    }
    break;
  case GC0409MIPI_CMMCLK_CURRENT:
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
      //GC0409MIPI_set_isp_driving_current(temp_para);

	spin_lock(&gc0409mipi_drv_lock);
      GC0409MIPI_sensor.eng.reg[GC0409MIPI_CMMCLK_CURRENT_INDEX].Para = temp_para;
	spin_unlock(&gc0409mipi_drv_lock);

      break;
    default:
      ASSERT(0);
    }
    break;
  case GC0409MIPI_FRAME_RATE_LIMITATION:
    ASSERT(0);
    break;
  case GC0409MIPI_REGISTER_EDITOR:
    switch (para->ItemIdx)
    {
      static kal_uint32 fac_sensor_reg;
    case 0:
      if (para->ItemValue < 0 || para->ItemValue > 0xFFFF) return KAL_FALSE;
      fac_sensor_reg = para->ItemValue;
      break;
    case 1:
      if (para->ItemValue < 0 || para->ItemValue > 0xFF) return KAL_FALSE;
      GC0409MIPI_write_cmos_sensor(fac_sensor_reg, para->ItemValue);
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

void GC0409MIPI_SetMirrorFlip(GC0409MIPI_IMAGE_MIRROR Mirror)
{


}

static void GC0409MIPI_Sensor_Init(void)
{
////////////////////////////////////////////////////
/////////////////////	SYS		////////////////////
////////////////////////////////////////////////////
	GC0409MIPI_write_cmos_sensor(0xfe, 0x80);
	GC0409MIPI_write_cmos_sensor(0xfe, 0x80);
	GC0409MIPI_write_cmos_sensor(0xfe, 0x80);
	GC0409MIPI_write_cmos_sensor(0xf7, 0x01);
	GC0409MIPI_write_cmos_sensor(0xf8, 0x05);
	GC0409MIPI_write_cmos_sensor(0xf9, 0x0f);//[0] not_use_pll
	GC0409MIPI_write_cmos_sensor(0xfa, 0x00);
	GC0409MIPI_write_cmos_sensor(0xfc, 0x0f);//[0] apwd
	GC0409MIPI_write_cmos_sensor(0xfe, 0x00);
		
////////////////////////////////////////////////////
/////////////// ANALOG & CISCTL/////////////////////
////////////////////////////////////////////////////
	GC0409MIPI_write_cmos_sensor(0xfe, 0x00);
	GC0409MIPI_write_cmos_sensor(0x03, 0x01);//01
	GC0409MIPI_write_cmos_sensor(0x04, 0xf4);//e0//Exp_time
	GC0409MIPI_write_cmos_sensor(0x05, 0x06);//02
	GC0409MIPI_write_cmos_sensor(0x06, 0x08);//84//HB
	GC0409MIPI_write_cmos_sensor(0x07, 0x00);
	GC0409MIPI_write_cmos_sensor(0x08, 0x04);//24//VB
	GC0409MIPI_write_cmos_sensor(0x0a, 0x00);//row_start
	GC0409MIPI_write_cmos_sensor(0x0c, 0x04);
	GC0409MIPI_write_cmos_sensor(0x0d, 0x01);
	GC0409MIPI_write_cmos_sensor(0x0e, 0xe8);
	GC0409MIPI_write_cmos_sensor(0x0f, 0x03);
	GC0409MIPI_write_cmos_sensor(0x10, 0x30);//win_width
	GC0409MIPI_write_cmos_sensor(0x17, MIRROR);//Don't Change Here!!!
	GC0409MIPI_write_cmos_sensor(0x18, 0x12);
	GC0409MIPI_write_cmos_sensor(0x19, 0x0b);//0d//AD_pipe_number
	GC0409MIPI_write_cmos_sensor(0x1a, 0x1b);
	GC0409MIPI_write_cmos_sensor(0x1d, 0x4c);
	GC0409MIPI_write_cmos_sensor(0x1e, 0x50);
	GC0409MIPI_write_cmos_sensor(0x1f, 0x80);
	GC0409MIPI_write_cmos_sensor(0x23, 0x01);//00//[0]vpix_sw
	GC0409MIPI_write_cmos_sensor(0x24, 0xc8);
	GC0409MIPI_write_cmos_sensor(0x25, 0xe2);
	GC0409MIPI_write_cmos_sensor(0x27, 0xaf);
	GC0409MIPI_write_cmos_sensor(0x28, 0x24);
	GC0409MIPI_write_cmos_sensor(0x29, 0x0f);//0d//[5:0]buf_EQ_post_width
	GC0409MIPI_write_cmos_sensor(0x2f, 0x14);
	GC0409MIPI_write_cmos_sensor(0x3f, 0x18);//tx en
	GC0409MIPI_write_cmos_sensor(0x72, 0x98);//58//[7]vrefrh_en [6:4]rsgh_r
	GC0409MIPI_write_cmos_sensor(0x73, 0x9a);
	GC0409MIPI_write_cmos_sensor(0x74, 0x47);//49//[3]RESTH_sw [2:0]noican
	GC0409MIPI_write_cmos_sensor(0x76, 0xb2);
	GC0409MIPI_write_cmos_sensor(0x7a, 0xcb);//fb//[5:4]vpix_r
	GC0409MIPI_write_cmos_sensor(0xc2, 0x0c);
	GC0409MIPI_write_cmos_sensor(0xd0, 0x10);
	GC0409MIPI_write_cmos_sensor(0xdc, 0x75);
	GC0409MIPI_write_cmos_sensor(0xeb, 0x78);

////////////////////////////////////////////////////
/////////////////////	ISP 	  //////////////////
////////////////////////////////////////////////////
	GC0409MIPI_write_cmos_sensor(0xfe, 0x00);
	GC0409MIPI_write_cmos_sensor(0x90, 0x01);
	GC0409MIPI_write_cmos_sensor(0x92, STARTX);//Don't Change Here!!!
	GC0409MIPI_write_cmos_sensor(0x94, STARTY);//Don't Change Here!!!
	GC0409MIPI_write_cmos_sensor(0x95, 0x01);
	GC0409MIPI_write_cmos_sensor(0x96, 0xe0);
	GC0409MIPI_write_cmos_sensor(0x97, 0x03);
	GC0409MIPI_write_cmos_sensor(0x98, 0x20);
	GC0409MIPI_write_cmos_sensor(0xb0, 0x79);//global_gain[7:3]
	GC0409MIPI_write_cmos_sensor(0x67, 0x02);//global_gain[2:0]	
	GC0409MIPI_write_cmos_sensor(0xb1, 0x01);
	GC0409MIPI_write_cmos_sensor(0xb2, 0x00);	
	GC0409MIPI_write_cmos_sensor(0xb6, 0x00);
	GC0409MIPI_write_cmos_sensor(0xb3, 0x40);
	GC0409MIPI_write_cmos_sensor(0xb4, 0x40);
	GC0409MIPI_write_cmos_sensor(0xb5, 0x40);
	
////////////////////////////////////////////////////
/////////////////////	 BLK		////////////////
////////////////////////////////////////////////////
	GC0409MIPI_write_cmos_sensor(0x40, 0x26); 
	GC0409MIPI_write_cmos_sensor(0x4f, BLK_VAL);//Don't Change Here!!!
	
/////////////////////////////////////////////////////     
//////////////////////	dark sun	/////////////////     
/////////////////////////////////////////////////////
	GC0409MIPI_write_cmos_sensor(0xfe, 0x00);
	GC0409MIPI_write_cmos_sensor(0xe0, 0x9f);  
	GC0409MIPI_write_cmos_sensor(0xe4, 0x0f);
	GC0409MIPI_write_cmos_sensor(0xe5, 0xff);

////////////////////////////////////////////////////
/////////////////////	 MIPI	////////////////////
////////////////////////////////////////////////////
	GC0409MIPI_write_cmos_sensor(0xfe, 0x03);
	GC0409MIPI_write_cmos_sensor(0x10, 0x80);	
	GC0409MIPI_write_cmos_sensor(0x01, 0x03);
	GC0409MIPI_write_cmos_sensor(0x02, 0x22);
	GC0409MIPI_write_cmos_sensor(0x03, 0x96);
	GC0409MIPI_write_cmos_sensor(0x04, 0x01);
	GC0409MIPI_write_cmos_sensor(0x05, 0x00);
	GC0409MIPI_write_cmos_sensor(0x06, 0x80);
	GC0409MIPI_write_cmos_sensor(0x11, 0x2b);
	GC0409MIPI_write_cmos_sensor(0x12, 0xe8);
	GC0409MIPI_write_cmos_sensor(0x13, 0x03);
	GC0409MIPI_write_cmos_sensor(0x15, 0x00);
	GC0409MIPI_write_cmos_sensor(0x21, 0x10);
	GC0409MIPI_write_cmos_sensor(0x22, 0x00);
	GC0409MIPI_write_cmos_sensor(0x23, 0x0a);
	GC0409MIPI_write_cmos_sensor(0x24, 0x10);
	GC0409MIPI_write_cmos_sensor(0x25, 0x10);
	GC0409MIPI_write_cmos_sensor(0x26, 0x03);
	GC0409MIPI_write_cmos_sensor(0x29, 0x01);
	GC0409MIPI_write_cmos_sensor(0x2a, 0x0a);
	GC0409MIPI_write_cmos_sensor(0x2b, 0x03);
	GC0409MIPI_write_cmos_sensor(0x10, 0x90);
	GC0409MIPI_write_cmos_sensor(0xfe, 0x00);
	GC0409MIPI_write_cmos_sensor(0xf9, 0x0e);//[0] not_use_pll
	GC0409MIPI_write_cmos_sensor(0xfc, 0x0e);//[0] apwd
	GC0409MIPI_write_cmos_sensor(0xfe, 0x00);
	
}   /*  GC0409MIPI_Sensor_Init  */

/*****************************************************************************/
/* Windows Mobile Sensor Interface */
/*****************************************************************************/
/*************************************************************************
* FUNCTION
*	GC0409MIPIOpen
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

UINT32 GC0409MIPIOpen(void)
{
	kal_uint16 sensor_id=0; 

	// check if sensor ID correct
	sensor_id=((GC0409MIPI_read_cmos_sensor(0xf0) << 8) | GC0409MIPI_read_cmos_sensor(0xf1));
	SENSORDB("GC0409MIPIOpen, sensor_id:%x \n",sensor_id);
	if (sensor_id != GC0409_SENSOR_ID)
		return ERROR_SENSOR_CONNECT_FAIL;
	
	/* initail sequence write in  */
	GC0409MIPI_Sensor_Init();

	return ERROR_NONE;
}   /* GC0409MIPIOpen  */

/*************************************************************************
* FUNCTION
*   GC0409MIPIGetSensorID
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
UINT32 GC0409MIPIGetSensorID(UINT32 *sensorID) 
{
	// check if sensor ID correct
	mt_set_gpio_mode(GPIO_CAMERA_CMPDN1_PIN,GPIO_CAMERA_CMPDN1_PIN_M_GPIO);
	mt_set_gpio_dir(GPIO_CAMERA_CMPDN1_PIN,GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_CAMERA_CMPDN1_PIN,GPIO_OUT_ZERO);
	msleep(10);

	*sensorID=((GC0409MIPI_read_cmos_sensor(0xf0) << 8) | GC0409MIPI_read_cmos_sensor(0xf1));		
	SENSORDB("GC0409MIPIGetSensorID:%x \n",*sensorID);
	if (*sensorID != GC0409_SENSOR_ID) {		
		*sensorID = 0xFFFFFFFF;		
		return ERROR_SENSOR_CONNECT_FAIL;
	}
   camera_pdn1_reverse = 1;
   return ERROR_NONE;
}

/*************************************************************************
* FUNCTION
*	GC0409MIPIClose
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
UINT32 GC0409MIPIClose(void)
{
#ifdef GC0409MIPI_DRIVER_TRACE
   SENSORDB("GC0409MIPIClose\n");
#endif
	return ERROR_NONE;
}   /* GC0409MIPIClose */

/*************************************************************************
* FUNCTION
* GC0409MIPIPreview
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
UINT32 GC0409MIPIPreview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	
	spin_lock(&gc0409mipi_drv_lock);	
	
	GC0409MIPI_sensor.pv_mode = KAL_TRUE;
	
	switch (sensor_config_data->SensorOperationMode)
	{
	  case MSDK_SENSOR_OPERATION_MODE_VIDEO: 	  	
		GC0409MIPI_sensor.video_mode = KAL_TRUE;
	  default: /* ISP_PREVIEW_MODE */
		GC0409MIPI_sensor.video_mode = KAL_FALSE;
	}
	GC0409MIPI_sensor.line_length = GC0409MIPI_PV_PERIOD_PIXEL_NUMS;
	GC0409MIPI_sensor.frame_height = GC0409MIPI_PV_PERIOD_LINE_NUMS;
	GC0409MIPI_sensor.pclk = GC0409MIPI_PREVIEW_CLK;

	spin_unlock(&gc0409mipi_drv_lock);

	return ERROR_NONE;
}   /*  GC0409MIPIPreview   */

/*************************************************************************
* FUNCTION
*	GC0409MIPICapture
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
UINT32 GC0409MIPICapture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
						  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	kal_uint32 shutter = (kal_uint32)GC0409MIPI_sensor.shutter;

	spin_lock(&gc0409mipi_drv_lock);

	GC0409MIPI_sensor.video_mode = KAL_FALSE;
	GC0409MIPI_sensor.pv_mode = KAL_FALSE;

	spin_unlock(&gc0409mipi_drv_lock);

	return ERROR_NONE;
}   /* GC0409MIPI_Capture() */

UINT32 GC0409MIPIGetResolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *pSensorResolution)
{
	pSensorResolution->SensorFullWidth=GC0409MIPI_IMAGE_SENSOR_FULL_WIDTH;
	pSensorResolution->SensorFullHeight=GC0409MIPI_IMAGE_SENSOR_FULL_HEIGHT;
	pSensorResolution->SensorPreviewWidth=GC0409MIPI_IMAGE_SENSOR_PV_WIDTH;
	pSensorResolution->SensorPreviewHeight=GC0409MIPI_IMAGE_SENSOR_PV_HEIGHT;
	pSensorResolution->SensorVideoWidth=GC0409MIPI_IMAGE_SENSOR_VIDEO_WIDTH;
	pSensorResolution->SensorVideoHeight=GC0409MIPI_IMAGE_SENSOR_VIDEO_HEIGHT;
	return ERROR_NONE;
}	/* GC0409MIPIGetResolution() */

UINT32 GC0409MIPIGetInfo(MSDK_SCENARIO_ID_ENUM ScenarioId,
					  MSDK_SENSOR_INFO_STRUCT *pSensorInfo,
					  MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
	pSensorInfo->SensorPreviewResolutionX=GC0409MIPI_IMAGE_SENSOR_PV_WIDTH;
	pSensorInfo->SensorPreviewResolutionY=GC0409MIPI_IMAGE_SENSOR_PV_HEIGHT;
	pSensorInfo->SensorFullResolutionX=GC0409MIPI_IMAGE_SENSOR_FULL_WIDTH;
	pSensorInfo->SensorFullResolutionY=GC0409MIPI_IMAGE_SENSOR_FULL_HEIGHT;

	pSensorInfo->SensorCameraPreviewFrameRate=30;
	pSensorInfo->SensorVideoFrameRate=30;
	pSensorInfo->SensorStillCaptureFrameRate=10;
	pSensorInfo->SensorWebCamCaptureFrameRate=15;
	pSensorInfo->SensorResetActiveHigh=TRUE; //low active
	pSensorInfo->SensorResetDelayCount=5; 
	pSensorInfo->SensorOutputDataFormat=GC0409MIPI_COLOR_FORMAT;
	pSensorInfo->SensorClockPolarity=SENSOR_CLOCK_POLARITY_LOW;	
	pSensorInfo->SensorClockFallingPolarity=SENSOR_CLOCK_POLARITY_LOW;
	pSensorInfo->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	pSensorInfo->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;

	pSensorInfo->CaptureDelayFrame = 2; 
	pSensorInfo->PreviewDelayFrame = 1;
	pSensorInfo->VideoDelayFrame = 1;

	pSensorInfo->SensorMasterClockSwitch = 0; 
	pSensorInfo->SensorDrivingCurrent = ISP_DRIVING_6MA;
	pSensorInfo->AEShutDelayFrame =0;		   /* The frame of setting shutter default 0 for TG int */
	pSensorInfo->AESensorGainDelayFrame = 0;   /* The frame of setting sensor gain */
	pSensorInfo->AEISPGainDelayFrame = 2;  

	pSensorInfo->SensroInterfaceType=SENSOR_INTERFACE_TYPE_MIPI;
	pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_1_LANE;
	//pSensorInfo->SensorRawType = RAW_TYPE_10BIT;

	pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0; 
	pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 14; 
	pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
	pSensorInfo->SensorWidthSampling = 0;
	pSensorInfo->SensorHightSampling = 0;
	pSensorInfo->SensorPacketECCOrder = 1;
	switch (ScenarioId)
	{
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			pSensorInfo->SensorClockFreq=24;
			pSensorInfo->SensorClockDividCount=3;
			pSensorInfo->SensorClockRisingCount= 0;
			pSensorInfo->SensorClockFallingCount= 2;
			pSensorInfo->SensorPixelClockCount= 3;
			pSensorInfo->SensorDataLatchCount= 2;
			pSensorInfo->SensorGrabStartX = GC0409MIPI_PV_X_START; 
			pSensorInfo->SensorGrabStartY = GC0409MIPI_PV_Y_START; 

		break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			pSensorInfo->SensorClockFreq=24;
			pSensorInfo->SensorClockDividCount=3;
			pSensorInfo->SensorClockRisingCount= 0;
			pSensorInfo->SensorClockFallingCount= 2;
			pSensorInfo->SensorPixelClockCount= 3;
			pSensorInfo->SensorDataLatchCount= 2;
			pSensorInfo->SensorGrabStartX = GC0409MIPI_VIDEO_X_START; 
			pSensorInfo->SensorGrabStartY = GC0409MIPI_VIDEO_Y_START; 

		break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
			pSensorInfo->SensorClockFreq=24;
			pSensorInfo->SensorClockDividCount= 3;
			pSensorInfo->SensorClockRisingCount=0;
			pSensorInfo->SensorClockFallingCount=2;
			pSensorInfo->SensorPixelClockCount=3;
			pSensorInfo->SensorDataLatchCount=2;
			pSensorInfo->SensorGrabStartX = GC0409MIPI_FULL_X_START; 
			pSensorInfo->SensorGrabStartY = GC0409MIPI_FULL_Y_START; 
		break;
		default:
			pSensorInfo->SensorClockFreq=24;
			pSensorInfo->SensorClockDividCount=3;
			pSensorInfo->SensorClockRisingCount=0;
			pSensorInfo->SensorClockFallingCount=2;		
			pSensorInfo->SensorPixelClockCount=3;
			pSensorInfo->SensorDataLatchCount=2;
			pSensorInfo->SensorGrabStartX = GC0409MIPI_PV_X_START; 
			pSensorInfo->SensorGrabStartY = GC0409MIPI_PV_Y_START; 
		break;
	}
	memcpy(pSensorConfigData, &GC0409MIPI_sensor.cfg_data, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
	return ERROR_NONE;
}	/* GC0409MIPIGetInfo() */


UINT32 GC0409MIPIControl(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *pImageWindow,
					  MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{

#ifdef GC0409_DRIVER_TRACE
	SENSORDB("GC0409MIPIControl ScenarioId = %d \n",ScenarioId);
#endif
	switch (ScenarioId)
	{
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			GC0409MIPIPreview(pImageWindow, pSensorConfigData);
		break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
			GC0409MIPICapture(pImageWindow, pSensorConfigData);
		break;		
        default:
            return ERROR_INVALID_SCENARIO_ID;
	}
	return ERROR_NONE;
}	/* GC0409MIPIControl() */

UINT32 GC0409MIPISetVideoMode(UINT16 u2FrameRate)
{};

UINT32 GC0409MIPISetTestPatternMode(kal_bool bEnable)
{
    SENSORDB("[GC0409SetTestPatternMode]test pattern bEnable:=%d\n",bEnable);
    if(bEnable)
    {
	GC0409MIPI_write_cmos_sensor(0xfe, 0x00);
	GC0409MIPI_write_cmos_sensor(0x8b, 0x30);
    }
    else
    {
	GC0409MIPI_write_cmos_sensor(0xfe, 0x00); 
	GC0409MIPI_write_cmos_sensor(0x8b, 0x20);
    }
    return ERROR_NONE;
}

UINT32 GC0409MIPIFeatureControl(MSDK_SENSOR_FEATURE_ENUM FeatureId,
							 UINT8 *pFeaturePara,UINT32 *pFeatureParaLen)
{
	UINT16 *pFeatureReturnPara16=(UINT16 *) pFeaturePara;
	UINT16 *pFeatureData16=(UINT16 *) pFeaturePara;
	UINT32 *pFeatureReturnPara32=(UINT32 *) pFeaturePara;
	//UINT32 *pFeatureData32=(UINT32 *) pFeaturePara;
	//UINT32 GC0409MIPISensorRegNumber;
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
			*pFeatureReturnPara16++=GC0409MIPI_IMAGE_SENSOR_FULL_WIDTH;
			*pFeatureReturnPara16=GC0409MIPI_IMAGE_SENSOR_FULL_HEIGHT;
			*pFeatureParaLen=4;
		break;
		case SENSOR_FEATURE_GET_PERIOD:	/* 3 */
			*pFeatureReturnPara16++=GC0409MIPI_sensor.line_length;
			*pFeatureReturnPara16=GC0409MIPI_sensor.frame_height;
			*pFeatureParaLen=4;
		break;
		case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:  /* 3 */
			*pFeatureReturnPara32 = GC0409MIPI_sensor.pclk;
			*pFeatureParaLen=4;
		break;
		case SENSOR_FEATURE_SET_ESHUTTER:	/* 4 */
			GC0409MIPI_set_shutter(*pFeatureData16);
		break;
		case SENSOR_FEATURE_SET_NIGHTMODE:
			GC0409MIPI_night_mode((BOOL) *pFeatureData16);
		break;
		case SENSOR_FEATURE_SET_GAIN:	/* 6 */			
			GC0409MIPI_SetGain((UINT16) *pFeatureData16);
		break;
		case SENSOR_FEATURE_SET_FLASHLIGHT:
		break;
		case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
		break;
		case SENSOR_FEATURE_SET_REGISTER:
			GC0409MIPI_write_cmos_sensor(pSensorRegData->RegAddr, pSensorRegData->RegData);
		break;
		case SENSOR_FEATURE_GET_REGISTER:
			pSensorRegData->RegData = GC0409MIPI_read_cmos_sensor(pSensorRegData->RegAddr);
		break;
		case SENSOR_FEATURE_SET_CCT_REGISTER:
			memcpy(&GC0409MIPI_sensor.eng.cct, pFeaturePara, sizeof(GC0409MIPI_sensor.eng.cct));
			break;
		break;
		case SENSOR_FEATURE_GET_CCT_REGISTER:	/* 12 */
			if (*pFeatureParaLen >= sizeof(GC0409MIPI_sensor.eng.cct) + sizeof(kal_uint32))
			{
			  *((kal_uint32 *)pFeaturePara++) = sizeof(GC0409MIPI_sensor.eng.cct);
			  memcpy(pFeaturePara, &GC0409MIPI_sensor.eng.cct, sizeof(GC0409MIPI_sensor.eng.cct));
			}
			break;
		case SENSOR_FEATURE_SET_ENG_REGISTER:
			memcpy(&GC0409MIPI_sensor.eng.reg, pFeaturePara, sizeof(GC0409MIPI_sensor.eng.reg));
			break;
		case SENSOR_FEATURE_GET_ENG_REGISTER:	/* 14 */
			if (*pFeatureParaLen >= sizeof(GC0409MIPI_sensor.eng.reg) + sizeof(kal_uint32))
			{
			  *((kal_uint32 *)pFeaturePara++) = sizeof(GC0409MIPI_sensor.eng.reg);
			  memcpy(pFeaturePara, &GC0409MIPI_sensor.eng.reg, sizeof(GC0409MIPI_sensor.eng.reg));
			}
		case SENSOR_FEATURE_GET_REGISTER_DEFAULT:
			((PNVRAM_SENSOR_DATA_STRUCT)pFeaturePara)->Version = NVRAM_CAMERA_SENSOR_FILE_VERSION;
			((PNVRAM_SENSOR_DATA_STRUCT)pFeaturePara)->SensorId = GC0409_SENSOR_ID;
			memcpy(((PNVRAM_SENSOR_DATA_STRUCT)pFeaturePara)->SensorEngReg, &GC0409MIPI_sensor.eng.reg, sizeof(GC0409MIPI_sensor.eng.reg));
			memcpy(((PNVRAM_SENSOR_DATA_STRUCT)pFeaturePara)->SensorCCTReg, &GC0409MIPI_sensor.eng.cct, sizeof(GC0409MIPI_sensor.eng.cct));
			*pFeatureParaLen = sizeof(NVRAM_SENSOR_DATA_STRUCT);
			break;
		case SENSOR_FEATURE_GET_CONFIG_PARA:
			memcpy(pFeaturePara, &GC0409MIPI_sensor.cfg_data, sizeof(GC0409MIPI_sensor.cfg_data));
			*pFeatureParaLen = sizeof(GC0409MIPI_sensor.cfg_data);
			break;
		case SENSOR_FEATURE_CAMERA_PARA_TO_SENSOR:
		     GC0409MIPI_camera_para_to_sensor();
			break;
		case SENSOR_FEATURE_SENSOR_TO_CAMERA_PARA:
			GC0409MIPI_sensor_to_camera_para();
			break;							
		case SENSOR_FEATURE_GET_GROUP_COUNT:
			GC0409MIPI_get_sensor_group_count((kal_uint32 *)pFeaturePara);
			*pFeatureParaLen = 4;
			break;
		case SENSOR_FEATURE_GET_GROUP_INFO:
			GC0409MIPI_get_sensor_group_info((MSDK_SENSOR_GROUP_INFO_STRUCT *)pFeaturePara);
			*pFeatureParaLen = sizeof(MSDK_SENSOR_GROUP_INFO_STRUCT);
			break;
		case SENSOR_FEATURE_GET_ITEM_INFO:
			GC0409MIPI_get_sensor_item_info((MSDK_SENSOR_ITEM_INFO_STRUCT *)pFeaturePara);
			*pFeatureParaLen = sizeof(MSDK_SENSOR_ITEM_INFO_STRUCT);
			break;
		case SENSOR_FEATURE_SET_ITEM_INFO:
			GC0409MIPI_set_sensor_item_info((MSDK_SENSOR_ITEM_INFO_STRUCT *)pFeaturePara);
			*pFeatureParaLen = sizeof(MSDK_SENSOR_ITEM_INFO_STRUCT);
			break;
		case SENSOR_FEATURE_GET_ENG_INFO:
			memcpy(pFeaturePara, &GC0409MIPI_sensor.eng_info, sizeof(GC0409MIPI_sensor.eng_info));
			*pFeatureParaLen = sizeof(GC0409MIPI_sensor.eng_info);
			break;
		case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
			// get the lens driver ID from EEPROM or just return LENS_DRIVER_ID_DO_NOT_CARE
			// if EEPROM does not exist in camera module.
			*pFeatureReturnPara32=LENS_DRIVER_ID_DO_NOT_CARE;
			*pFeatureParaLen=4;
		break;
		case SENSOR_FEATURE_SET_VIDEO_MODE:
		   //GC0409MIPISetVideoMode(*pFeatureData16);
			break; 
		case SENSOR_FEATURE_CHECK_SENSOR_ID:
			GC0409MIPIGetSensorID(pFeatureReturnPara32); 
			break; 
		case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE:
			break;
		case SENSOR_FEATURE_SET_TEST_PATTERN:	        
		       GC0409MIPISetTestPatternMode((BOOL)*pFeatureData16);    
		       break;
		case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE:	
		       *pFeatureReturnPara32=GC0409MIPI_TEST_PATTERN_CHECKSUM;
		       *pFeatureParaLen=4;
		       break;
		default:
			break;
	}
	return ERROR_NONE;
}	/* GC0409MIPIFeatureControl() */
SENSOR_FUNCTION_STRUCT	SensorFuncGC0409MIPI=
{
	GC0409MIPIOpen,
	GC0409MIPIGetInfo,
	GC0409MIPIGetResolution,
	GC0409MIPIFeatureControl,
	GC0409MIPIControl,
	GC0409MIPIClose
};

UINT32 GC0409MIPI_RAW_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
	/* To Do : Check Sensor status here */
	if (pfFunc!=NULL)
		*pfFunc=&SensorFuncGC0409MIPI;

	return ERROR_NONE;
}	/* SensorInit() */
