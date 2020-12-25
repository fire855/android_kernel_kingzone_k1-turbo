/*******************************************************************************************/
// schedule
//   getsensorid ok
//   open ok
//   setting(pv,cap,video) ok

/*******************************************************************************************/

#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
#include <linux/xlog.h>
#include <asm/system.h>

#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"

#include "hi545mipiraw_Sensor.h"
#include "hi545mipiraw_Camera_Sensor_para.h"
#include "hi545mipiraw_CameraCustomized.h"
static DEFINE_SPINLOCK(HI545mipiraw_drv_lock);

#define HI545_TEST_PATTERN_CHECKSUM (0x9FC8832A)//do rotate will change this value

#define HI545_DEBUG
//#define HI545_DEBUG_SOFIA

#ifdef HI545_DEBUG
	#define HI545DB(fmt, arg...) xlog_printk(ANDROID_LOG_DEBUG, "[HI545Raw] ",  fmt, ##arg)
#else
	#define HI545DB(fmt, arg...)
#endif

#ifdef HI545_DEBUG_SOFIA
	#define HI545DBSOFIA(fmt, arg...) xlog_printk(ANDROID_LOG_DEBUG, "[HI545Raw] ",  fmt, ##arg)
#else
	#define HI545DBSOFIA(fmt, arg...)
#endif

#define mDELAY(ms)  mdelay(ms)

kal_uint32 HI545_FeatureControl_PERIOD_PixelNum=HI545_PV_PERIOD_PIXEL_NUMS;
kal_uint32 HI545_FeatureControl_PERIOD_LineNum=HI545_PV_PERIOD_LINE_NUMS;

UINT16 VIDEO_MODE_TARGET_FPS = 30;
static BOOL ReEnteyCamera = KAL_FALSE;


MSDK_SENSOR_CONFIG_STRUCT HI545SensorConfigData;

kal_uint32 HI545_FAC_SENSOR_REG;

MSDK_SCENARIO_ID_ENUM HI545CurrentScenarioId = MSDK_SCENARIO_ID_CAMERA_PREVIEW;

/* FIXME: old factors and DIDNOT use now. s*/
SENSOR_REG_STRUCT HI545SensorCCT[]=CAMERA_SENSOR_CCT_DEFAULT_VALUE;
SENSOR_REG_STRUCT HI545SensorReg[ENGINEER_END]=CAMERA_SENSOR_REG_DEFAULT_VALUE;
/* FIXME: old factors and DIDNOT use now. e*/

static HI545_PARA_STRUCT HI545;

extern int iReadReg(u16 a_u2Addr , u8 * a_puBuff , u16 i2cId);
extern int iWriteReg(u16 a_u2Addr , u32 a_u4Data , u32 a_u4Bytes , u16 i2cId);

//#define HI545_write_cmos_sensor(addr, para) iWriteReg((u16) addr , (u32) para , 1, HI545MIPI_WRITE_ID)
// modify by yfx
//extern int iMultiWriteReg(u8 *pData, u16 lens);
#if 1
#define HI545_write_cmos_sensor(addr, para) iWriteReg((u16) addr , (u32) para , 2, HI545MIPI_WRITE_ID)
#else
void HI545_write_cmos_sensor(u16 addr, u16 para) 
{
	u8 senddata[4];
	senddata[0] = (addr >> 8) & 0xff;
	senddata[1] = addr & 0xff;
	senddata[2] = (para >> 8) & 0xff;
	senddata[3] = para & 0xff;
	iMultiWriteReg(senddata, 4)
}
#endif
// end

kal_uint16 HI545_read_cmos_sensor(kal_uint32 addr)
{
kal_uint16 get_byte=0;
    iReadReg((u16) addr ,(u8*)&get_byte,HI545MIPI_WRITE_ID);
    return get_byte;
}

#define Sleep(ms) mdelay(ms)

void HI545_write_shutter(kal_uint32 shutter)
{
	kal_uint32 min_framelength = HI545_PV_PERIOD_PIXEL_NUMS, max_shutter=0;
	kal_uint32 extra_lines = 0;
	kal_uint32 line_length = 0;
	kal_uint32 frame_length = 0;
	unsigned long flags;

	HI545DBSOFIA("!!shutter=%d!!!!!\n", shutter);

	if(HI545.HI545AutoFlickerMode == KAL_TRUE)
	{

		if ( SENSOR_MODE_PREVIEW == HI545.sensorMode )  //(g_iHI545_Mode == HI545_MODE_PREVIEW)	//SXGA size output
		{
			line_length = HI545_PV_PERIOD_PIXEL_NUMS + HI545.DummyPixels;
			max_shutter = HI545_PV_PERIOD_LINE_NUMS + HI545.DummyLines ;
		}
		else if( SENSOR_MODE_VIDEO == HI545.sensorMode ) //add for video_6M setting
		{
			line_length = HI545_VIDEO_PERIOD_PIXEL_NUMS + HI545.DummyPixels;
			max_shutter = HI545_VIDEO_PERIOD_LINE_NUMS + HI545.DummyLines ;
		}
		else
		{
			line_length = HI545_FULL_PERIOD_PIXEL_NUMS + HI545.DummyPixels;
			max_shutter = HI545_FULL_PERIOD_LINE_NUMS + HI545.DummyLines ;
		}

		switch(HI545CurrentScenarioId)
		{
        	case MSDK_SCENARIO_ID_CAMERA_ZSD:
			case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
				HI545DBSOFIA("AutoFlickerMode!!! MSDK_SCENARIO_ID_CAMERA_ZSD  0!!\n");
				min_framelength = max_shutter;// capture max_fps 24,no need calculate
				break;
			case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
				if(VIDEO_MODE_TARGET_FPS==30)
				{
					min_framelength = (HI545.videoPclk*10000) /(HI545_VIDEO_PERIOD_PIXEL_NUMS + HI545.DummyPixels)/304*10 ;
				}
				else if(VIDEO_MODE_TARGET_FPS==15)
				{
					min_framelength = (HI545.videoPclk*10000) /(HI545_VIDEO_PERIOD_PIXEL_NUMS + HI545.DummyPixels)/148*10 ;
				}
				else
				{
					min_framelength = max_shutter;
				}
				break;
			default:
				min_framelength = (HI545.pvPclk*10000) /(HI545_PV_PERIOD_PIXEL_NUMS + HI545.DummyPixels)/296*10 ;
    			break;
		}

		HI545DBSOFIA("AutoFlickerMode!!! min_framelength for AutoFlickerMode = %d (0x%x)\n",min_framelength,min_framelength);
		HI545DBSOFIA("max framerate(10 base) autofilker = %d\n",(HI545.pvPclk*10000)*10 /line_length/min_framelength);

		if (shutter < 4)
			shutter = 4;

		if (shutter > (max_shutter-4) )
			extra_lines = shutter - max_shutter + 4;
		else
			extra_lines = 0;

		if ( SENSOR_MODE_PREVIEW == HI545.sensorMode )	//SXGA size output
		{
			frame_length = HI545_PV_PERIOD_LINE_NUMS+ HI545.DummyLines + extra_lines ;
		}
		else if(SENSOR_MODE_VIDEO == HI545.sensorMode)
		{
			frame_length = HI545_VIDEO_PERIOD_LINE_NUMS+ HI545.DummyLines + extra_lines ;
		}
		else				//QSXGA size output
		{
			frame_length = HI545_FULL_PERIOD_LINE_NUMS + HI545.DummyLines + extra_lines ;
		}
		HI545DBSOFIA("frame_length 0= %d\n",frame_length);

		if (frame_length < min_framelength)
		{
			//shutter = min_framelength - 4;

			switch(HI545CurrentScenarioId)
			{
        	case MSDK_SCENARIO_ID_CAMERA_ZSD:
			case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
				extra_lines = min_framelength- (HI545_FULL_PERIOD_LINE_NUMS+ HI545.DummyLines);
				break;
			case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
				extra_lines = min_framelength- (HI545_VIDEO_PERIOD_LINE_NUMS+ HI545.DummyLines);
				break;
			default:
				extra_lines = min_framelength- (HI545_PV_PERIOD_LINE_NUMS+ HI545.DummyLines);
    			break;
			}
			frame_length = min_framelength;
		}

		HI545DBSOFIA("frame_length 1= %d\n",frame_length);

		ASSERT(line_length < HI545_MAX_LINE_LENGTH);		//0xCCCC
		ASSERT(frame_length < HI545_MAX_FRAME_LENGTH); 	//0xFFFF
		
		spin_lock_irqsave(&HI545mipiraw_drv_lock,flags);
		HI545.maxExposureLines = frame_length - 4;
		HI545_FeatureControl_PERIOD_PixelNum = line_length;
		HI545_FeatureControl_PERIOD_LineNum = frame_length;
		spin_unlock_irqrestore(&HI545mipiraw_drv_lock,flags);


		HI545_write_cmos_sensor(0x0046, 0x0100);
		//Set total frame length
		HI545_write_cmos_sensor(0x0006, frame_length);
//		HI545_write_cmos_sensor(0x0007, frame_length & 0xFF);

		//Set shutter (Coarse integration time, uint: lines.)
		HI545_write_cmos_sensor(0x0004, shutter);
		//HI545_write_cmos_sensor(0x0005, (shutter) & 0xFF);
		
		HI545_write_cmos_sensor(0x0046, 0x0000);

		HI545DBSOFIA("frame_length 2= %d\n",frame_length);
		HI545DB("framerate(10 base) = %d\n",(HI545.pvPclk*10000)*10 /line_length/frame_length);

		HI545DB("shutter=%d, extra_lines=%d, line_length=%d, frame_length=%d\n", shutter, extra_lines, line_length, frame_length);

	}
	else
	{
		if ( SENSOR_MODE_PREVIEW == HI545.sensorMode )  //(g_iHI545_Mode == HI545_MODE_PREVIEW)	//SXGA size output
		{
			max_shutter = HI545_PV_PERIOD_LINE_NUMS + HI545.DummyLines ;
		}
		else if( SENSOR_MODE_VIDEO == HI545.sensorMode ) //add for video_6M setting
		{
			max_shutter = HI545_VIDEO_PERIOD_LINE_NUMS + HI545.DummyLines ;
		}
		else
		{
			max_shutter = HI545_FULL_PERIOD_LINE_NUMS + HI545.DummyLines ;
		}

		if (shutter < 4)
			shutter = 4;

		if (shutter > (max_shutter-4) )
			extra_lines = shutter - max_shutter + 4;
		else
			extra_lines = 0;

		if ( SENSOR_MODE_PREVIEW == HI545.sensorMode )	//SXGA size output
		{
			line_length = HI545_PV_PERIOD_PIXEL_NUMS + HI545.DummyPixels;
			frame_length = HI545_PV_PERIOD_LINE_NUMS+ HI545.DummyLines + extra_lines ;
		}
		else if( SENSOR_MODE_VIDEO == HI545.sensorMode )
		{
			line_length = HI545_VIDEO_PERIOD_PIXEL_NUMS + HI545.DummyPixels;
			frame_length = HI545_VIDEO_PERIOD_LINE_NUMS + HI545.DummyLines + extra_lines ;
		}
		else				//QSXGA size output
		{
			line_length = HI545_FULL_PERIOD_PIXEL_NUMS + HI545.DummyPixels;
			frame_length = HI545_FULL_PERIOD_LINE_NUMS + HI545.DummyLines + extra_lines ;
		}

		ASSERT(line_length < HI545_MAX_LINE_LENGTH);		//0xCCCC
		ASSERT(frame_length < HI545_MAX_FRAME_LENGTH); 	//0xFFFF

		//Set total frame length
		HI545_write_cmos_sensor(0x0046, 0x0100);		
		HI545_write_cmos_sensor(0x0006, frame_length);
		//HI545_write_cmos_sensor(0x0007, frame_length & 0xFF);
		HI545_write_cmos_sensor(0x0046, 0x0000);		

		spin_lock_irqsave(&HI545mipiraw_drv_lock,flags);
		HI545.maxExposureLines = frame_length -4;
		HI545_FeatureControl_PERIOD_PixelNum = line_length;
		HI545_FeatureControl_PERIOD_LineNum = frame_length;
		spin_unlock_irqrestore(&HI545mipiraw_drv_lock,flags);


		//Set shutter (Coarse integration time, uint: lines.)
		HI545_write_cmos_sensor(0x0046, 0x0100);		
		HI545_write_cmos_sensor(0x0004, shutter);
		//HI545_write_cmos_sensor(0x0005, (shutter) & 0xFF);
		HI545_write_cmos_sensor(0x0046, 0x0000);		
		
		//HI545DB("framerate(10 base) = %d\n",(HI545.pvPclk*10000)*10 /line_length/frame_length);
		HI545DB("shutter=%d, extra_lines=%d, line_length=%d, frame_length=%d\n", shutter, extra_lines, line_length, frame_length);
	}

}   /* write_HI545_shutter */

/*******************************************************************************
*
********************************************************************************/
static kal_uint16 HI545Reg2Gain(const kal_uint8 iReg)
{
	kal_uint16 iGain = 64;

	iGain = 4 * iReg + 64;

	return iGain;
}

/*******************************************************************************
*
********************************************************************************/
static kal_uint16 HI545Gain2Reg(const kal_uint16 Gain)
{
	kal_uint16 iReg;
	kal_uint8 iBaseGain = 64;
	
	iReg = Gain / 4 - 16;
    return iReg;//HI545. sensorGlobalGain

}


void write_HI545_gain(kal_uint8 gain)
{
#if 1
	HI545_write_cmos_sensor(0x003a, (gain << 8));
#endif
	return;
}

/*************************************************************************
* FUNCTION
*    HI545_SetGain
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
void HI545_SetGain(UINT16 iGain)
{
	unsigned long flags;
    kal_uint16 HI545GlobalGain=0;
	kal_uint16 DigitalGain = 0;

    
	// AG = (regvalue / 16) + 1
	if(iGain > 1024)  
	{
		iGain = 1024;
	}
	if(iGain < 64)  // gain的reg最大值是255
	{
		iGain = 64;
	}
		
	HI545GlobalGain = HI545Gain2Reg(iGain); 
	spin_lock(&HI545mipiraw_drv_lock);
	HI545.realGain = iGain;
	HI545.sensorGlobalGain =HI545GlobalGain;
	spin_unlock(&HI545mipiraw_drv_lock);

	HI545DB("[HI545_SetGain]HI545.sensorGlobalGain=0x%x,HI545.realGain=%d\n",HI545.sensorGlobalGain,HI545.realGain);

	HI545_write_cmos_sensor(0x0046, 0x0100);
	write_HI545_gain(HI545.sensorGlobalGain);	
	HI545_write_cmos_sensor(0x0046, 0x0000);

	
	return;	
}   /*  HI545_SetGain_SetGain  */


/*************************************************************************
* FUNCTION
*    read_HI545_gain
*
* DESCRIPTION
*    This function is to set global gain to sensor.
*
* PARAMETERS
*    None
*
* RETURNS
*    gain : sensor global gain
*
* GLOBALS AFFECTED
*
*************************************************************************/
kal_uint16 read_HI545_gain(void)
{
	kal_uint16 read_gain_anolog=0;
	kal_uint16 HI545RealGain_anolog =0;
	kal_uint16 HI545RealGain =0;

	read_gain_anolog=HI545_read_cmos_sensor(0x003a);
	
	HI545RealGain_anolog = HI545Reg2Gain(read_gain_anolog);
	

	spin_lock(&HI545mipiraw_drv_lock);
	HI545.sensorGlobalGain = read_gain_anolog;
	HI545.realGain = HI545RealGain;
	spin_unlock(&HI545mipiraw_drv_lock);
	HI545DB("[read_HI545_gain]HI545RealGain_anolog=0x%x\n",HI545RealGain_anolog);

	return HI545.sensorGlobalGain;
}  /* read_HI545_gain */


void HI545_camera_para_to_sensor(void)
{
    kal_uint32    i;
    for(i=0; 0xFFFFFFFF!=HI545SensorReg[i].Addr; i++)
    {
        HI545_write_cmos_sensor(HI545SensorReg[i].Addr, HI545SensorReg[i].Para);
    }
    for(i=ENGINEER_START_ADDR; 0xFFFFFFFF!=HI545SensorReg[i].Addr; i++)
    {
        HI545_write_cmos_sensor(HI545SensorReg[i].Addr, HI545SensorReg[i].Para);
    }
    for(i=FACTORY_START_ADDR; i<FACTORY_END_ADDR; i++)
    {
        HI545_write_cmos_sensor(HI545SensorCCT[i].Addr, HI545SensorCCT[i].Para);
    }
}


/*************************************************************************
* FUNCTION
*    HI545_sensor_to_camera_para
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
void HI545_sensor_to_camera_para(void)
{
    kal_uint32    i, temp_data;
    for(i=0; 0xFFFFFFFF!=HI545SensorReg[i].Addr; i++)
    {
         temp_data = HI545_read_cmos_sensor(HI545SensorReg[i].Addr);
		 spin_lock(&HI545mipiraw_drv_lock);
		 HI545SensorReg[i].Para =temp_data;
		 spin_unlock(&HI545mipiraw_drv_lock);
    }
    for(i=ENGINEER_START_ADDR; 0xFFFFFFFF!=HI545SensorReg[i].Addr; i++)
    {
        temp_data = HI545_read_cmos_sensor(HI545SensorReg[i].Addr);
		spin_lock(&HI545mipiraw_drv_lock);
		HI545SensorReg[i].Para = temp_data;
		spin_unlock(&HI545mipiraw_drv_lock);
    }
}

/*************************************************************************
* FUNCTION
*    HI545_get_sensor_group_count
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
kal_int32  HI545_get_sensor_group_count(void)
{
    return GROUP_TOTAL_NUMS;
}

void HI545_get_sensor_group_info(kal_uint16 group_idx, kal_int8* group_name_ptr, kal_int32* item_count_ptr)
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

void HI545_get_sensor_item_info(kal_uint16 group_idx,kal_uint16 item_idx, MSDK_SENSOR_ITEM_INFO_STRUCT* info_ptr)
{
    kal_int16 temp_reg=0;
    kal_uint16 temp_gain=0, temp_addr=0, temp_para=0;

    switch (group_idx)
    {
        case PRE_GAIN:
           switch (item_idx)
          {
              case 0:
                sprintf((char *)info_ptr->ItemNamePtr,"Pregain-R");
                  temp_addr = PRE_GAIN_R_INDEX;
              break;
              case 1:
                sprintf((char *)info_ptr->ItemNamePtr,"Pregain-Gr");
                  temp_addr = PRE_GAIN_Gr_INDEX;
              break;
              case 2:
                sprintf((char *)info_ptr->ItemNamePtr,"Pregain-Gb");
                  temp_addr = PRE_GAIN_Gb_INDEX;
              break;
              case 3:
                sprintf((char *)info_ptr->ItemNamePtr,"Pregain-B");
                  temp_addr = PRE_GAIN_B_INDEX;
              break;
              case 4:
                 sprintf((char *)info_ptr->ItemNamePtr,"SENSOR_BASEGAIN");
                 temp_addr = SENSOR_BASEGAIN;
              break;
              default:
                 ASSERT(0);
          }

            temp_para= HI545SensorCCT[temp_addr].Para;
			//temp_gain= (temp_para/HI545.sensorBaseGain) * 1000;

            info_ptr->ItemValue=temp_gain;
            info_ptr->IsTrueFalse=KAL_FALSE;
            info_ptr->IsReadOnly=KAL_FALSE;
            info_ptr->IsNeedRestart=KAL_FALSE;
            info_ptr->Min= HI545_MIN_ANALOG_GAIN * 1000;
            info_ptr->Max= HI545_MAX_ANALOG_GAIN * 1000;
            break;
        case CMMCLK_CURRENT:
            switch (item_idx)
            {
                case 0:
                    sprintf((char *)info_ptr->ItemNamePtr,"Drv Cur[2,4,6,8]mA");

                    //temp_reg=MT9P017SensorReg[CMMCLK_CURRENT_INDEX].Para;
                    temp_reg = ISP_DRIVING_2MA;
                    if(temp_reg==ISP_DRIVING_2MA)
                    {
                        info_ptr->ItemValue=2;
                    }
                    else if(temp_reg==ISP_DRIVING_4MA)
                    {
                        info_ptr->ItemValue=4;
                    }
                    else if(temp_reg==ISP_DRIVING_6MA)
                    {
                        info_ptr->ItemValue=6;
                    }
                    else if(temp_reg==ISP_DRIVING_8MA)
                    {
                        info_ptr->ItemValue=8;
                    }

                    info_ptr->IsTrueFalse=KAL_FALSE;
                    info_ptr->IsReadOnly=KAL_FALSE;
                    info_ptr->IsNeedRestart=KAL_TRUE;
                    info_ptr->Min=2;
                    info_ptr->Max=8;
                    break;
                default:
                    ASSERT(0);
            }
            break;
        case FRAME_RATE_LIMITATION:
            switch (item_idx)
            {
                case 0:
                    sprintf((char *)info_ptr->ItemNamePtr,"Max Exposure Lines");
                    info_ptr->ItemValue=    111;  //MT9P017_MAX_EXPOSURE_LINES;
                    info_ptr->IsTrueFalse=KAL_FALSE;
                    info_ptr->IsReadOnly=KAL_TRUE;
                    info_ptr->IsNeedRestart=KAL_FALSE;
                    info_ptr->Min=0;
                    info_ptr->Max=0;
                    break;
                case 1:
                    sprintf((char *)info_ptr->ItemNamePtr,"Min Frame Rate");
                    info_ptr->ItemValue=12;
                    info_ptr->IsTrueFalse=KAL_FALSE;
                    info_ptr->IsReadOnly=KAL_TRUE;
                    info_ptr->IsNeedRestart=KAL_FALSE;
                    info_ptr->Min=0;
                    info_ptr->Max=0;
                    break;
                default:
                    ASSERT(0);
            }
            break;
        case REGISTER_EDITOR:
            switch (item_idx)
            {
                case 0:
                    sprintf((char *)info_ptr->ItemNamePtr,"REG Addr.");
                    info_ptr->ItemValue=0;
                    info_ptr->IsTrueFalse=KAL_FALSE;
                    info_ptr->IsReadOnly=KAL_FALSE;
                    info_ptr->IsNeedRestart=KAL_FALSE;
                    info_ptr->Min=0;
                    info_ptr->Max=0xFFFF;
                    break;
                case 1:
                    sprintf((char *)info_ptr->ItemNamePtr,"REG Value");
                    info_ptr->ItemValue=0;
                    info_ptr->IsTrueFalse=KAL_FALSE;
                    info_ptr->IsReadOnly=KAL_FALSE;
                    info_ptr->IsNeedRestart=KAL_FALSE;
                    info_ptr->Min=0;
                    info_ptr->Max=0xFFFF;
                    break;
                default:
                ASSERT(0);
            }
            break;
        default:
            ASSERT(0);
    }
}



kal_bool HI545_set_sensor_item_info(kal_uint16 group_idx, kal_uint16 item_idx, kal_int32 ItemValue)
{
//   kal_int16 temp_reg;
   kal_uint16  temp_gain=0,temp_addr=0, temp_para=0;

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
                 ASSERT(0);
          }

		 temp_gain=((ItemValue*BASEGAIN+500)/1000);			//+500:get closed integer value

		  if(temp_gain>=1*BASEGAIN && temp_gain<=16*BASEGAIN)
          {
//             temp_para=(temp_gain * HI545.sensorBaseGain + BASEGAIN/2)/BASEGAIN;
          }
          else
			  ASSERT(0);

			 HI545DBSOFIA("HI545????????????????????? :\n ");
		  spin_lock(&HI545mipiraw_drv_lock);
          HI545SensorCCT[temp_addr].Para = temp_para;
		  spin_unlock(&HI545mipiraw_drv_lock);
          HI545_write_cmos_sensor(HI545SensorCCT[temp_addr].Addr,temp_para);

            break;
        case CMMCLK_CURRENT:
            switch (item_idx)
            {
                case 0:
                    //no need to apply this item for driving current
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
					spin_lock(&HI545mipiraw_drv_lock);
                    HI545_FAC_SENSOR_REG=ItemValue;
					spin_unlock(&HI545mipiraw_drv_lock);
                    break;
                case 1:
                    HI545_write_cmos_sensor(HI545_FAC_SENSOR_REG,ItemValue);
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

static void HI545_SetDummy( const kal_uint32 iPixels, const kal_uint32 iLines )
{
	kal_uint32 line_length = 0;
	kal_uint32 frame_length = 0;

	if ( SENSOR_MODE_PREVIEW == HI545.sensorMode )	//SXGA size output
	{
		line_length = HI545_PV_PERIOD_PIXEL_NUMS + iPixels;
		frame_length = HI545_PV_PERIOD_LINE_NUMS + iLines;
	}
	else if( SENSOR_MODE_VIDEO== HI545.sensorMode )
	{
		line_length = HI545_VIDEO_PERIOD_PIXEL_NUMS + iPixels;
		frame_length = HI545_VIDEO_PERIOD_LINE_NUMS + iLines;
	}
	else//QSXGA size output
	{
		line_length = HI545_FULL_PERIOD_PIXEL_NUMS + iPixels;
		frame_length = HI545_FULL_PERIOD_LINE_NUMS + iLines;
	}

	//if(HI545.maxExposureLines > frame_length -4 )
	//	return;

	//ASSERT(line_length < HI545_MAX_LINE_LENGTH);		//0xCCCC
	//ASSERT(frame_length < HI545_MAX_FRAME_LENGTH);	//0xFFFF

	//Set total frame length
	HI545_write_cmos_sensor(0x0006, frame_length);
	//HI545_write_cmos_sensor(0x0007, frame_length & 0xFF);

	spin_lock(&HI545mipiraw_drv_lock);
	HI545.maxExposureLines = frame_length -4;
	HI545_FeatureControl_PERIOD_PixelNum = line_length;
	HI545_FeatureControl_PERIOD_LineNum = frame_length;
	spin_unlock(&HI545mipiraw_drv_lock);

	//Set total line length
	HI545_write_cmos_sensor(0x0008, line_length);
	//HI545_write_cmos_sensor(0x0009, line_length & 0xFF);

}   /*  HI545_SetDummy */

void HI545PreviewSetting(void)
{	
#if 1
	HI545DB("HI545PreviewSetting_2lane_30fps:\n ");
	
	
	//////////////////////////////////////////////////////////////////////////
	//	Sensor			 : Hi-545
	//		Mode			 : Preview 
	//		Size			 : 2560 * 1920
	//////////////////////////////////////////////////////////////////////////

	HI545_write_cmos_sensor(0x0118, 0x0000); //sleep On
	
	HI545_write_cmos_sensor(0x0B16, 0x4A0B); 
	HI545_write_cmos_sensor(0x004C, 0x0100); 
	HI545_write_cmos_sensor(0x0032, 0x0101); 
	HI545_write_cmos_sensor(0x001E, 0x0101); 
	
	HI545_write_cmos_sensor(0x000C, 0x0000); 
	
	HI545_write_cmos_sensor(0x0902, 0x4101); 
	HI545_write_cmos_sensor(0x090A, 0x03E4); 
	HI545_write_cmos_sensor(0x090C, 0x0020); 
	HI545_write_cmos_sensor(0x090E, 0x0020); 
	HI545_write_cmos_sensor(0x0910, 0x5D07); 
	HI545_write_cmos_sensor(0x0912, 0x061e); 
	HI545_write_cmos_sensor(0x0914, 0x0407); 
	HI545_write_cmos_sensor(0x0916, 0x0b0a); 
	HI545_write_cmos_sensor(0x0918, 0x0e09); 

	//HI545_write_cmos_sensor(0x020A, 0x0100);
	
	//--- Pixel Array Addressing ------//
	HI545_write_cmos_sensor(0x0012, 0x00A4); //x_addr_start_hact_h. 
	HI545_write_cmos_sensor(0x0018, 0x0AD3); //x_addr_end_hact_h.   
	
	HI545_write_cmos_sensor(0x0026, 0x0010); //y_addr_start_vact_h. 
	HI545_write_cmos_sensor(0x002C, 0x07B7); //y_addr_end_vact_h.   
	
	HI545_write_cmos_sensor(0x0128, 0x0002); //digital_crop_x_offset_l  
	HI545_write_cmos_sensor(0x012A, 0x0000); //digital_crop_y_offset_l  
	HI545_write_cmos_sensor(0x012C, 0x0A00); //2560 digital_crop_image_width 
	HI545_write_cmos_sensor(0x012E, 0x0780); //1920 digital_crop_image_height 
	
	//Image size 2560x1920
	HI545_write_cmos_sensor(0x0110, 0x0A00); //X_output_size_h 
	HI545_write_cmos_sensor(0x0112, 0x0780); //Y_output_size_h 
	
	HI545_write_cmos_sensor(0x0006, 0x07e1); //07c0); //frame_length_h 2017
	HI545_write_cmos_sensor(0x0008, 0x0B40); //line_length_h 2880
	HI545_write_cmos_sensor(0x000A, 0x0DB0); 
	HI545_write_cmos_sensor(0x0002, 0x04b0); //Fine.int 
	HI545_write_cmos_sensor(0x0700, 0x0590); 

	HI545_write_cmos_sensor(0x0A04, 0x0113); // from 0112 to 0113
	
	HI545_write_cmos_sensor(0x0118, 0x0100); //sleep Off 

	mDELAY(50);

	ReEnteyCamera = KAL_FALSE;

    HI545DB("HI545PreviewSetting_2lane exit :\n ");
#endif
}


void HI545VideoSetting(void)
{

	HI545DB("HI545VideoSetting:\n ");
	
	//////////////////////////////////////////////////////////////////////////
	//	Sensor			 : Hi-545
	//		Mode			 : Video 
	//		Size			 : 2560 * 1920
	//////////////////////////////////////////////////////////////////////////

	HI545_write_cmos_sensor(0x0118, 0x0000); //sleep On
	
	HI545_write_cmos_sensor(0x0B16, 0x4A0B); 
	HI545_write_cmos_sensor(0x004C, 0x0100); 
	HI545_write_cmos_sensor(0x0032, 0x0101); 
	HI545_write_cmos_sensor(0x001E, 0x0101); 
	
	HI545_write_cmos_sensor(0x000C, 0x0000); 
	
	HI545_write_cmos_sensor(0x0902, 0x4101); 
	HI545_write_cmos_sensor(0x090A, 0x03E4); 
	HI545_write_cmos_sensor(0x090C, 0x0020); 
	HI545_write_cmos_sensor(0x090E, 0x0020); 
	HI545_write_cmos_sensor(0x0910, 0x5D07); 
	HI545_write_cmos_sensor(0x0912, 0x061e); 
	HI545_write_cmos_sensor(0x0914, 0x0407); 
	HI545_write_cmos_sensor(0x0916, 0x0b0a); 
	HI545_write_cmos_sensor(0x0918, 0x0e09); 
	
	//--- Pixel Array Addressing ------//
	HI545_write_cmos_sensor(0x0012, 0x00A4); //x_addr_start_hact_h. 
	HI545_write_cmos_sensor(0x0018, 0x0AD3); //x_addr_end_hact_h.   
	
	HI545_write_cmos_sensor(0x0026, 0x0010); //y_addr_start_vact_h. 
	HI545_write_cmos_sensor(0x002C, 0x07B7); //y_addr_end_vact_h.   
	
	HI545_write_cmos_sensor(0x0128, 0x0002); //digital_crop_x_offset_l  
	HI545_write_cmos_sensor(0x012A, 0x0000); //digital_crop_y_offset_l  
	HI545_write_cmos_sensor(0x012C, 0x0A00); //2560 digital_crop_image_width 
	HI545_write_cmos_sensor(0x012E, 0x0780); //1920 digital_crop_image_height 
	
	//Image size 2560x1920
	HI545_write_cmos_sensor(0x0110, 0x0A00); //X_output_size_h 
	HI545_write_cmos_sensor(0x0112, 0x0780); //Y_output_size_h 
	
	HI545_write_cmos_sensor(0x0006, 0x07e1); //07c0); //frame_length_h 2017
	HI545_write_cmos_sensor(0x0008, 0x0B40); //line_length_h 2880
	HI545_write_cmos_sensor(0x000A, 0x0DB0); 
	HI545_write_cmos_sensor(0x0002, 0x04b0); //Fine.int 
	HI545_write_cmos_sensor(0x0700, 0x0590); 

	HI545_write_cmos_sensor(0x0A04, 0x0113); // from 0112 to 0113
	
	HI545_write_cmos_sensor(0x0118, 0x0100); //sleep Off 

	
	mDELAY(50);

	ReEnteyCamera = KAL_FALSE;

	HI545DB("HI545VideoSetting_4:3 exit :\n ");
}


void HI545CaptureSetting(void)
{

    if(ReEnteyCamera == KAL_TRUE)
    {
		HI545DB("HI545CaptureSetting_2lane_SleepIn :\n ");
    }
	else
	{
		HI545DB("HI545CaptureSetting_2lane_streamOff :\n ");
	}

	HI545DB("HI545CaptureSetting_2lane_OB:\n ");
	
	//////////////////////////////////////////////////////////////////////////
	//	Sensor			 : Hi-545
	//		Mode			 : Capture 
	//		Size			 : 2560 * 1920
	//////////////////////////////////////////////////////////////////////////

	HI545_write_cmos_sensor(0x0118, 0x0000); //sleep On
	
	HI545_write_cmos_sensor(0x0B16, 0x4A0B); 
	HI545_write_cmos_sensor(0x004C, 0x0100); 
	HI545_write_cmos_sensor(0x0032, 0x0101); 
	HI545_write_cmos_sensor(0x001E, 0x0101); 
	
	HI545_write_cmos_sensor(0x000C, 0x0000); 
	
	HI545_write_cmos_sensor(0x0902, 0x4101); 
	HI545_write_cmos_sensor(0x090A, 0x03E4); 
	HI545_write_cmos_sensor(0x090C, 0x0020); 
	HI545_write_cmos_sensor(0x090E, 0x0020); 
	HI545_write_cmos_sensor(0x0910, 0x5D07); 
	HI545_write_cmos_sensor(0x0912, 0x061e); 
	HI545_write_cmos_sensor(0x0914, 0x0407); 
	HI545_write_cmos_sensor(0x0916, 0x0b0a); 
	HI545_write_cmos_sensor(0x0918, 0x0e09); 
	
	//--- Pixel Array Addressing ------//
	HI545_write_cmos_sensor(0x0012, 0x00A4); //x_addr_start_hact_h. 
	HI545_write_cmos_sensor(0x0018, 0x0AD3); //x_addr_end_hact_h.   
	
	HI545_write_cmos_sensor(0x0026, 0x0010); //y_addr_start_vact_h. 
	HI545_write_cmos_sensor(0x002C, 0x07B7); //y_addr_end_vact_h.   
	
	HI545_write_cmos_sensor(0x0128, 0x0002); //digital_crop_x_offset_l  
	HI545_write_cmos_sensor(0x012A, 0x0000); //digital_crop_y_offset_l  
	HI545_write_cmos_sensor(0x012C, 0x0A00); //2560 digital_crop_image_width 
	HI545_write_cmos_sensor(0x012E, 0x0780); //1920 digital_crop_image_height 
	
	//Image size 2560x1920
	HI545_write_cmos_sensor(0x0110, 0x0A00); //X_output_size_h 
	HI545_write_cmos_sensor(0x0112, 0x0780); //Y_output_size_h 
	
	HI545_write_cmos_sensor(0x0006, 0x07e1); //07c0); //frame_length_h 2017
	HI545_write_cmos_sensor(0x0008, 0x0B40); //line_length_h 2880
	HI545_write_cmos_sensor(0x000A, 0x0DB0); 
	HI545_write_cmos_sensor(0x0002, 0x04b0); //Fine.int 
	HI545_write_cmos_sensor(0x0700, 0x0590); 

	HI545_write_cmos_sensor(0x0A04, 0x0113); // from 0112 to 0113
	
	HI545_write_cmos_sensor(0x0118, 0x0100); //sleep Off 

	mDELAY(50);

	ReEnteyCamera = KAL_FALSE;
	
	HI545DB("HI545CaptureSetting_2lane exit :\n ");
}

static void HI545_Sensor_Init(void)
{

	HI545DB("HI545_Sensor_Init 2lane_OB:\n ");	
	
    ReEnteyCamera = KAL_TRUE;
	mDELAY(20);

	//////////////////////////////////////////////////////////////////////////
	//			Sensor        	 : Hi-545
	//      Mode             : Init 
	//      Size             : 2604x1956
	//			set file				 : v0.31
	//			Date						 : 20140616
	//////////////////////////////////////////////////////////////////////////

	HI545_write_cmos_sensor(0x0118, 0x0000); //sleep On

	//--- SRAM timing control---//
	HI545_write_cmos_sensor(0x0E00, 0x0101);
	HI545_write_cmos_sensor(0x0E02, 0x0101);
	HI545_write_cmos_sensor(0x0E04, 0x0101);
	HI545_write_cmos_sensor(0x0E06, 0x0101);
	HI545_write_cmos_sensor(0x0E08, 0x0101);
	HI545_write_cmos_sensor(0x0E0A, 0x0101);
	HI545_write_cmos_sensor(0x0E0C, 0x0101);
	HI545_write_cmos_sensor(0x0E0E, 0x0101);

	HI545_write_cmos_sensor(0x2000, 0x4031);
	HI545_write_cmos_sensor(0x2002, 0x83F8);
	HI545_write_cmos_sensor(0x2004, 0x4104);
	HI545_write_cmos_sensor(0x2006, 0x4307);
	HI545_write_cmos_sensor(0x2008, 0x4382);
	HI545_write_cmos_sensor(0x200a, 0x80C8);
	HI545_write_cmos_sensor(0x200c, 0x4382);
	HI545_write_cmos_sensor(0x200e, 0x8070);
	HI545_write_cmos_sensor(0x2010, 0x43A2);
	HI545_write_cmos_sensor(0x2012, 0x0B80);
	HI545_write_cmos_sensor(0x2014, 0x0C0A);
	HI545_write_cmos_sensor(0x2016, 0x4382);
	HI545_write_cmos_sensor(0x2018, 0x0B90);
	HI545_write_cmos_sensor(0x201a, 0x0C0A);
	HI545_write_cmos_sensor(0x201c, 0x4382);
	HI545_write_cmos_sensor(0x201e, 0x0B9C);
	HI545_write_cmos_sensor(0x2020, 0x0C0A);
	HI545_write_cmos_sensor(0x2022, 0x93D2);
	HI545_write_cmos_sensor(0x2024, 0x003D);
	HI545_write_cmos_sensor(0x2026, 0x2002);
	HI545_write_cmos_sensor(0x2028, 0x4030);
	HI545_write_cmos_sensor(0x202a, 0xF684);
	HI545_write_cmos_sensor(0x202c, 0x43C2);
	HI545_write_cmos_sensor(0x202e, 0x0F82);
	HI545_write_cmos_sensor(0x2030, 0x425F);
	HI545_write_cmos_sensor(0x2032, 0x0118);
	HI545_write_cmos_sensor(0x2034, 0xF37F);
	HI545_write_cmos_sensor(0x2036, 0x930F);
	HI545_write_cmos_sensor(0x2038, 0x2002);
	HI545_write_cmos_sensor(0x203a, 0x0CC8);
	HI545_write_cmos_sensor(0x203c, 0x3FF9);
	HI545_write_cmos_sensor(0x203e, 0x4F82);
	HI545_write_cmos_sensor(0x2040, 0x8096);
	HI545_write_cmos_sensor(0x2042, 0x43D2);
	HI545_write_cmos_sensor(0x2044, 0x0A80);
	HI545_write_cmos_sensor(0x2046, 0x43D2);
	HI545_write_cmos_sensor(0x2048, 0x0180);
	HI545_write_cmos_sensor(0x204a, 0x43D2);
	HI545_write_cmos_sensor(0x204c, 0x019A);
	HI545_write_cmos_sensor(0x204e, 0x40F2);
	HI545_write_cmos_sensor(0x2050, 0x0009);
	HI545_write_cmos_sensor(0x2052, 0x019B);
	HI545_write_cmos_sensor(0x2054, 0x12B0);
	HI545_write_cmos_sensor(0x2056, 0xFCE2);
	HI545_write_cmos_sensor(0x2058, 0x93D2);
	HI545_write_cmos_sensor(0x205a, 0x003E);
	HI545_write_cmos_sensor(0x205c, 0x2002);
	HI545_write_cmos_sensor(0x205e, 0x4030);
	HI545_write_cmos_sensor(0x2060, 0xF5A6);
	HI545_write_cmos_sensor(0x2062, 0x4308);
	HI545_write_cmos_sensor(0x2064, 0x5038);
	HI545_write_cmos_sensor(0x2066, 0x0030);
	HI545_write_cmos_sensor(0x2068, 0x480F);
	HI545_write_cmos_sensor(0x206a, 0x12B0);
	HI545_write_cmos_sensor(0x206c, 0xFCF4);
	HI545_write_cmos_sensor(0x206e, 0x403B);
	HI545_write_cmos_sensor(0x2070, 0x7606);
	HI545_write_cmos_sensor(0x2072, 0x4B29);
	HI545_write_cmos_sensor(0x2074, 0x5318);
	HI545_write_cmos_sensor(0x2076, 0x480F);
	HI545_write_cmos_sensor(0x2078, 0x12B0);
	HI545_write_cmos_sensor(0x207a, 0xFCF4);
	HI545_write_cmos_sensor(0x207c, 0x4B2A);
	HI545_write_cmos_sensor(0x207e, 0x5318);
	HI545_write_cmos_sensor(0x2080, 0x480F);
	HI545_write_cmos_sensor(0x2082, 0x12B0);
	HI545_write_cmos_sensor(0x2084, 0xFCF4);
	HI545_write_cmos_sensor(0x2086, 0x4A0D);
	HI545_write_cmos_sensor(0x2088, 0xF03D);
	HI545_write_cmos_sensor(0x208a, 0x000F);
	HI545_write_cmos_sensor(0x208c, 0x108D);
	HI545_write_cmos_sensor(0x208e, 0x4B2E);
	HI545_write_cmos_sensor(0x2090, 0x5E0E);
	HI545_write_cmos_sensor(0x2092, 0x5E0E);
	HI545_write_cmos_sensor(0x2094, 0x5E0E);
	HI545_write_cmos_sensor(0x2096, 0x5E0E);
	HI545_write_cmos_sensor(0x2098, 0x4A0F);
	HI545_write_cmos_sensor(0x209a, 0xC312);
	HI545_write_cmos_sensor(0x209c, 0x100F);
	HI545_write_cmos_sensor(0x209e, 0x110F);
	HI545_write_cmos_sensor(0x20a0, 0x110F);
	HI545_write_cmos_sensor(0x20a2, 0x110F);
	HI545_write_cmos_sensor(0x20a4, 0x590D);
	HI545_write_cmos_sensor(0x20a6, 0x4D87);
	HI545_write_cmos_sensor(0x20a8, 0x5000);
	HI545_write_cmos_sensor(0x20aa, 0x5F0E);
	HI545_write_cmos_sensor(0x20ac, 0x4E87);
	HI545_write_cmos_sensor(0x20ae, 0x6000);
	HI545_write_cmos_sensor(0x20b0, 0x5327);
	HI545_write_cmos_sensor(0x20b2, 0x5038);
	HI545_write_cmos_sensor(0x20b4, 0xFFD1);
	HI545_write_cmos_sensor(0x20b6, 0x9038);
	HI545_write_cmos_sensor(0x20b8, 0x0300);
	HI545_write_cmos_sensor(0x20ba, 0x2BD4);
	HI545_write_cmos_sensor(0x20bc, 0x0261);
	HI545_write_cmos_sensor(0x20be, 0x0000);
	HI545_write_cmos_sensor(0x20c0, 0x43A2);
	HI545_write_cmos_sensor(0x20c2, 0x0384);
	HI545_write_cmos_sensor(0x20c4, 0x42B2);
	HI545_write_cmos_sensor(0x20c6, 0x0386);
	HI545_write_cmos_sensor(0x20c8, 0x43C2);
	HI545_write_cmos_sensor(0x20ca, 0x0180);
	HI545_write_cmos_sensor(0x20cc, 0x43D2);
	HI545_write_cmos_sensor(0x20ce, 0x003D);
	HI545_write_cmos_sensor(0x20d0, 0x40B2);
	HI545_write_cmos_sensor(0x20d2, 0x808B);
	HI545_write_cmos_sensor(0x20d4, 0x0B88);
	HI545_write_cmos_sensor(0x20d6, 0x0C0A);
	HI545_write_cmos_sensor(0x20d8, 0x40B2);
	HI545_write_cmos_sensor(0x20da, 0x1009);
	HI545_write_cmos_sensor(0x20dc, 0x0B8A);
	HI545_write_cmos_sensor(0x20de, 0x0C0A);
	HI545_write_cmos_sensor(0x20e0, 0x40B2);
	HI545_write_cmos_sensor(0x20e2, 0xC40C);
	HI545_write_cmos_sensor(0x20e4, 0x0B8C);
	HI545_write_cmos_sensor(0x20e6, 0x0C0A);
	HI545_write_cmos_sensor(0x20e8, 0x40B2);
	HI545_write_cmos_sensor(0x20ea, 0xC9E1);
	HI545_write_cmos_sensor(0x20ec, 0x0B8E);
	HI545_write_cmos_sensor(0x20ee, 0x0C0A);
	HI545_write_cmos_sensor(0x20f0, 0x40B2);
	HI545_write_cmos_sensor(0x20f2, 0x0C1E);
	HI545_write_cmos_sensor(0x20f4, 0x0B92);
	HI545_write_cmos_sensor(0x20f6, 0x0C0A);
	HI545_write_cmos_sensor(0x20f8, 0x43D2);
	HI545_write_cmos_sensor(0x20fa, 0x0F82);
	HI545_write_cmos_sensor(0x20fc, 0x0C3C);
	HI545_write_cmos_sensor(0x20fe, 0x0C3C);
	HI545_write_cmos_sensor(0x2100, 0x0C3C);
	HI545_write_cmos_sensor(0x2102, 0x0C3C);
	HI545_write_cmos_sensor(0x2104, 0x421F);
	HI545_write_cmos_sensor(0x2106, 0x00A6);
	HI545_write_cmos_sensor(0x2108, 0x503F);
	HI545_write_cmos_sensor(0x210a, 0x07D0);
	HI545_write_cmos_sensor(0x210c, 0x3811);
	HI545_write_cmos_sensor(0x210e, 0x4F82);
	HI545_write_cmos_sensor(0x2110, 0x7100);
	HI545_write_cmos_sensor(0x2112, 0x0004);
	HI545_write_cmos_sensor(0x2114, 0x0C0D);
	HI545_write_cmos_sensor(0x2116, 0x0005);
	HI545_write_cmos_sensor(0x2118, 0x0C04);
	HI545_write_cmos_sensor(0x211a, 0x000D);
	HI545_write_cmos_sensor(0x211c, 0x0C09);
	HI545_write_cmos_sensor(0x211e, 0x003D);
	HI545_write_cmos_sensor(0x2120, 0x0C1D);
	HI545_write_cmos_sensor(0x2122, 0x003C);
	HI545_write_cmos_sensor(0x2124, 0x0C13);
	HI545_write_cmos_sensor(0x2126, 0x0004);
	HI545_write_cmos_sensor(0x2128, 0x0C09);
	HI545_write_cmos_sensor(0x212a, 0x0004);
	HI545_write_cmos_sensor(0x212c, 0x533F);
	HI545_write_cmos_sensor(0x212e, 0x37EF);
	HI545_write_cmos_sensor(0x2130, 0x4392);
	HI545_write_cmos_sensor(0x2132, 0x8094);
	HI545_write_cmos_sensor(0x2134, 0x4382);
	HI545_write_cmos_sensor(0x2136, 0x8098);
	HI545_write_cmos_sensor(0x2138, 0x4382);
	HI545_write_cmos_sensor(0x213a, 0x80B0);
	HI545_write_cmos_sensor(0x213c, 0x4382);
	HI545_write_cmos_sensor(0x213e, 0x80B6);
	HI545_write_cmos_sensor(0x2140, 0x4382);
	HI545_write_cmos_sensor(0x2142, 0x809C);
	HI545_write_cmos_sensor(0x2144, 0x40B2);
	HI545_write_cmos_sensor(0x2146, 0x0028);
	HI545_write_cmos_sensor(0x2148, 0x7000);
	HI545_write_cmos_sensor(0x214a, 0x43A2);
	HI545_write_cmos_sensor(0x214c, 0x809A);
	HI545_write_cmos_sensor(0x214e, 0xB3E2);
	HI545_write_cmos_sensor(0x2150, 0x00B4);
	HI545_write_cmos_sensor(0x2152, 0x2402);
	HI545_write_cmos_sensor(0x2154, 0x4392);
	HI545_write_cmos_sensor(0x2156, 0x809A);
	HI545_write_cmos_sensor(0x2158, 0x4329);
	HI545_write_cmos_sensor(0x215a, 0xB3D2);
	HI545_write_cmos_sensor(0x215c, 0x00B4);
	HI545_write_cmos_sensor(0x215e, 0x2002);
	HI545_write_cmos_sensor(0x2160, 0x4030);
	HI545_write_cmos_sensor(0x2162, 0xF596);
	HI545_write_cmos_sensor(0x2164, 0x4309);
	HI545_write_cmos_sensor(0x2166, 0x4384);
	HI545_write_cmos_sensor(0x2168, 0x0002);
	HI545_write_cmos_sensor(0x216a, 0x4384);
	HI545_write_cmos_sensor(0x216c, 0x0006);
	HI545_write_cmos_sensor(0x216e, 0x40B2);
	HI545_write_cmos_sensor(0x2170, 0x0005);
	HI545_write_cmos_sensor(0x2172, 0x7320);
	HI545_write_cmos_sensor(0x2174, 0x4392);
	HI545_write_cmos_sensor(0x2176, 0x7326);
	HI545_write_cmos_sensor(0x2178, 0x12B0);
	HI545_write_cmos_sensor(0x217a, 0xF912);
	HI545_write_cmos_sensor(0x217c, 0x4392);
	HI545_write_cmos_sensor(0x217e, 0x731C);
	HI545_write_cmos_sensor(0x2180, 0x9382);
	HI545_write_cmos_sensor(0x2182, 0x8094);
	HI545_write_cmos_sensor(0x2184, 0x200A);
	HI545_write_cmos_sensor(0x2186, 0x0B00);
	HI545_write_cmos_sensor(0x2188, 0x7302);
	HI545_write_cmos_sensor(0x218a, 0x02BC);
	HI545_write_cmos_sensor(0x218c, 0x4382);
	HI545_write_cmos_sensor(0x218e, 0x7004);
	HI545_write_cmos_sensor(0x2190, 0x430F);
	HI545_write_cmos_sensor(0x2192, 0x12B0);
	HI545_write_cmos_sensor(0x2194, 0xF716);
	HI545_write_cmos_sensor(0x2196, 0x12B0);
	HI545_write_cmos_sensor(0x2198, 0xF912);
	HI545_write_cmos_sensor(0x219a, 0x4392);
	HI545_write_cmos_sensor(0x219c, 0x80B4);
	HI545_write_cmos_sensor(0x219e, 0x4382);
	HI545_write_cmos_sensor(0x21a0, 0x740E);
	HI545_write_cmos_sensor(0x21a2, 0xB3E2);
	HI545_write_cmos_sensor(0x21a4, 0x0080);
	HI545_write_cmos_sensor(0x21a6, 0x2402);
	HI545_write_cmos_sensor(0x21a8, 0x4392);
	HI545_write_cmos_sensor(0x21aa, 0x740E);
	HI545_write_cmos_sensor(0x21ac, 0x431F);
	HI545_write_cmos_sensor(0x21ae, 0x12B0);
	HI545_write_cmos_sensor(0x21b0, 0xF716);
	HI545_write_cmos_sensor(0x21b2, 0x4392);
	HI545_write_cmos_sensor(0x21b4, 0x7004);
	HI545_write_cmos_sensor(0x21b6, 0x4982);
	HI545_write_cmos_sensor(0x21b8, 0x7110);
	HI545_write_cmos_sensor(0x21ba, 0x9382);
	HI545_write_cmos_sensor(0x21bc, 0x8092);
	HI545_write_cmos_sensor(0x21be, 0x2005);
	HI545_write_cmos_sensor(0x21c0, 0x9392);
	HI545_write_cmos_sensor(0x21c2, 0x7110);
	HI545_write_cmos_sensor(0x21c4, 0x2402);
	HI545_write_cmos_sensor(0x21c6, 0x4030);
	HI545_write_cmos_sensor(0x21c8, 0xF49A);
	HI545_write_cmos_sensor(0x21ca, 0x9392);
	HI545_write_cmos_sensor(0x21cc, 0x7110);
	HI545_write_cmos_sensor(0x21ce, 0x20AD);
	HI545_write_cmos_sensor(0x21d0, 0x0B00);
	HI545_write_cmos_sensor(0x21d2, 0x7302);
	HI545_write_cmos_sensor(0x21d4, 0x0032);
	HI545_write_cmos_sensor(0x21d6, 0x4382);
	HI545_write_cmos_sensor(0x21d8, 0x7004);
	HI545_write_cmos_sensor(0x21da, 0x422F);
	HI545_write_cmos_sensor(0x21dc, 0x12B0);
	HI545_write_cmos_sensor(0x21de, 0xF716);
	HI545_write_cmos_sensor(0x21e0, 0x0800);
	HI545_write_cmos_sensor(0x21e2, 0x7114);
	HI545_write_cmos_sensor(0x21e4, 0x403F);
	HI545_write_cmos_sensor(0x21e6, 0x0003);
	HI545_write_cmos_sensor(0x21e8, 0x12B0);
	HI545_write_cmos_sensor(0x21ea, 0xF716);
	HI545_write_cmos_sensor(0x21ec, 0x425F);
	HI545_write_cmos_sensor(0x21ee, 0x0C9C);
	HI545_write_cmos_sensor(0x21f0, 0x4F4E);
	HI545_write_cmos_sensor(0x21f2, 0x430F);
	HI545_write_cmos_sensor(0x21f4, 0x4E0D);
	HI545_write_cmos_sensor(0x21f6, 0x430C);
	HI545_write_cmos_sensor(0x21f8, 0x421F);
	HI545_write_cmos_sensor(0x21fa, 0x0C9A);
	HI545_write_cmos_sensor(0x21fc, 0xDF0C);
	HI545_write_cmos_sensor(0x21fe, 0x1204);
	HI545_write_cmos_sensor(0x2200, 0x440F);
	HI545_write_cmos_sensor(0x2202, 0x532F);
	HI545_write_cmos_sensor(0x2204, 0x120F);
	HI545_write_cmos_sensor(0x2206, 0x1212);
	HI545_write_cmos_sensor(0x2208, 0x0CA2);
	HI545_write_cmos_sensor(0x220a, 0x403E);
	HI545_write_cmos_sensor(0x220c, 0x80B8);
	HI545_write_cmos_sensor(0x220e, 0x403F);
	HI545_write_cmos_sensor(0x2210, 0x8072);
	HI545_write_cmos_sensor(0x2212, 0x12B0);
	HI545_write_cmos_sensor(0x2214, 0xF77A);
	HI545_write_cmos_sensor(0x2216, 0x4F0A);
	HI545_write_cmos_sensor(0x2218, 0x403F);
	HI545_write_cmos_sensor(0x221a, 0x0003);
	HI545_write_cmos_sensor(0x221c, 0x12B0);
	HI545_write_cmos_sensor(0x221e, 0xF716);
	HI545_write_cmos_sensor(0x2220, 0x425F);
	HI545_write_cmos_sensor(0x2222, 0x0CA0);
	HI545_write_cmos_sensor(0x2224, 0x4F4E);
	HI545_write_cmos_sensor(0x2226, 0x430F);
	HI545_write_cmos_sensor(0x2228, 0x4E0D);
	HI545_write_cmos_sensor(0x222a, 0x430C);
	HI545_write_cmos_sensor(0x222c, 0x421F);
	HI545_write_cmos_sensor(0x222e, 0x0C9E);
	HI545_write_cmos_sensor(0x2230, 0xDF0C);
	HI545_write_cmos_sensor(0x2232, 0x440F);
	HI545_write_cmos_sensor(0x2234, 0x522F);
	HI545_write_cmos_sensor(0x2236, 0x120F);
	HI545_write_cmos_sensor(0x2238, 0x532F);
	HI545_write_cmos_sensor(0x223a, 0x120F);
	HI545_write_cmos_sensor(0x223c, 0x1212);
	HI545_write_cmos_sensor(0x223e, 0x0CA4);
	HI545_write_cmos_sensor(0x2240, 0x403E);
	HI545_write_cmos_sensor(0x2242, 0x809E);
	HI545_write_cmos_sensor(0x2244, 0x403F);
	HI545_write_cmos_sensor(0x2246, 0x8050);
	HI545_write_cmos_sensor(0x2248, 0x12B0);
	HI545_write_cmos_sensor(0x224a, 0xF77A);
	HI545_write_cmos_sensor(0x224c, 0x4F0B);
	HI545_write_cmos_sensor(0x224e, 0x403F);
	HI545_write_cmos_sensor(0x2250, 0x0003);
	HI545_write_cmos_sensor(0x2252, 0x12B0);
	HI545_write_cmos_sensor(0x2254, 0xF716);
	HI545_write_cmos_sensor(0x2256, 0x430D);
	HI545_write_cmos_sensor(0x2258, 0x441E);
	HI545_write_cmos_sensor(0x225a, 0x0004);
	HI545_write_cmos_sensor(0x225c, 0x442F);
	HI545_write_cmos_sensor(0x225e, 0x5031);
	HI545_write_cmos_sensor(0x2260, 0x000C);
	HI545_write_cmos_sensor(0x2262, 0x9E0F);
	HI545_write_cmos_sensor(0x2264, 0x2C01);
	HI545_write_cmos_sensor(0x2266, 0x431D);
	HI545_write_cmos_sensor(0x2268, 0x8E0F);
	HI545_write_cmos_sensor(0x226a, 0x930F);
	HI545_write_cmos_sensor(0x226c, 0x3402);
	HI545_write_cmos_sensor(0x226e, 0xE33F);
	HI545_write_cmos_sensor(0x2270, 0x531F);
	HI545_write_cmos_sensor(0x2272, 0x421E);
	HI545_write_cmos_sensor(0x2274, 0x0CA2);
	HI545_write_cmos_sensor(0x2276, 0xC312);
	HI545_write_cmos_sensor(0x2278, 0x100E);
	HI545_write_cmos_sensor(0x227a, 0x9E0F);
	HI545_write_cmos_sensor(0x227c, 0x2804);
	HI545_write_cmos_sensor(0x227e, 0x930D);
	HI545_write_cmos_sensor(0x2280, 0x2001);
	HI545_write_cmos_sensor(0x2282, 0x531A);
	HI545_write_cmos_sensor(0x2284, 0x5D0B);
	HI545_write_cmos_sensor(0x2286, 0x90B2);
	HI545_write_cmos_sensor(0x2288, 0x0007);
	HI545_write_cmos_sensor(0x228a, 0x80B6);
	HI545_write_cmos_sensor(0x228c, 0x2449);
	HI545_write_cmos_sensor(0x228e, 0x90B2);
	HI545_write_cmos_sensor(0x2290, 0x012C);
	HI545_write_cmos_sensor(0x2292, 0x80B0);
	HI545_write_cmos_sensor(0x2294, 0x2445);
	HI545_write_cmos_sensor(0x2296, 0x92E2);
	HI545_write_cmos_sensor(0x2298, 0x00A0);
	HI545_write_cmos_sensor(0x229a, 0x2423);
	HI545_write_cmos_sensor(0x229c, 0xB3E2);
	HI545_write_cmos_sensor(0x229e, 0x0C81);
	HI545_write_cmos_sensor(0x22a0, 0x2426);
	HI545_write_cmos_sensor(0x22a2, 0x425F);
	HI545_write_cmos_sensor(0x22a4, 0x0C92);
	HI545_write_cmos_sensor(0x22a6, 0xF37F);
	HI545_write_cmos_sensor(0x22a8, 0x503F);
	HI545_write_cmos_sensor(0x22aa, 0x0011);
	HI545_write_cmos_sensor(0x22ac, 0x421E);
	HI545_write_cmos_sensor(0x22ae, 0x0CAE);
	HI545_write_cmos_sensor(0x22b0, 0x9E0F);
	HI545_write_cmos_sensor(0x22b2, 0x2C07);
	HI545_write_cmos_sensor(0x22b4, 0x425F);
	HI545_write_cmos_sensor(0x22b6, 0x0C92);
	HI545_write_cmos_sensor(0x22b8, 0xF37F);
	HI545_write_cmos_sensor(0x22ba, 0x503F);
	HI545_write_cmos_sensor(0x22bc, 0x0010);
	HI545_write_cmos_sensor(0x22be, 0x4F82);
	HI545_write_cmos_sensor(0x22c0, 0x0CAE);
	HI545_write_cmos_sensor(0x22c2, 0x425F);
	HI545_write_cmos_sensor(0x22c4, 0x0C92);
	HI545_write_cmos_sensor(0x22c6, 0xF37F);
	HI545_write_cmos_sensor(0x22c8, 0x503F);
	HI545_write_cmos_sensor(0x22ca, 0x0011);
	HI545_write_cmos_sensor(0x22cc, 0x421E);
	HI545_write_cmos_sensor(0x22ce, 0x0CAC);
	HI545_write_cmos_sensor(0x22d0, 0x9E0F);
	HI545_write_cmos_sensor(0x22d2, 0x2C07);
	HI545_write_cmos_sensor(0x22d4, 0x425F);
	HI545_write_cmos_sensor(0x22d6, 0x0C92);
	HI545_write_cmos_sensor(0x22d8, 0xF37F);
	HI545_write_cmos_sensor(0x22da, 0x503F);
	HI545_write_cmos_sensor(0x22dc, 0x0010);
	HI545_write_cmos_sensor(0x22de, 0x4F82);
	HI545_write_cmos_sensor(0x22e0, 0x0CAC);
	HI545_write_cmos_sensor(0x22e2, 0x432F);
	HI545_write_cmos_sensor(0x22e4, 0x12B0);
	HI545_write_cmos_sensor(0x22e6, 0xF716);
	HI545_write_cmos_sensor(0x22e8, 0x12B0);
	HI545_write_cmos_sensor(0x22ea, 0xF6B6);
	HI545_write_cmos_sensor(0x22ec, 0x3F66);
	HI545_write_cmos_sensor(0x22ee, 0x421F);
	HI545_write_cmos_sensor(0x22f0, 0x0CAE);
	HI545_write_cmos_sensor(0x22f2, 0xC312);
	HI545_write_cmos_sensor(0x22f4, 0x100F);
	HI545_write_cmos_sensor(0x22f6, 0x110F);
	HI545_write_cmos_sensor(0x22f8, 0x110F);
	HI545_write_cmos_sensor(0x22fa, 0x110F);
	HI545_write_cmos_sensor(0x22fc, 0x930F);
	HI545_write_cmos_sensor(0x22fe, 0x2403);
	HI545_write_cmos_sensor(0x2300, 0x40B2);
	HI545_write_cmos_sensor(0x2302, 0x0010);
	HI545_write_cmos_sensor(0x2304, 0x0CAE);
	HI545_write_cmos_sensor(0x2306, 0x421F);
	HI545_write_cmos_sensor(0x2308, 0x0CAC);
	HI545_write_cmos_sensor(0x230a, 0xC312);
	HI545_write_cmos_sensor(0x230c, 0x100F);
	HI545_write_cmos_sensor(0x230e, 0x110F);
	HI545_write_cmos_sensor(0x2310, 0x110F);
	HI545_write_cmos_sensor(0x2312, 0x110F);
	HI545_write_cmos_sensor(0x2314, 0x930F);
	HI545_write_cmos_sensor(0x2316, 0x27E5);
	HI545_write_cmos_sensor(0x2318, 0x40B2);
	HI545_write_cmos_sensor(0x231a, 0x0010);
	HI545_write_cmos_sensor(0x231c, 0x0CAC);
	HI545_write_cmos_sensor(0x231e, 0x3FE1);
	HI545_write_cmos_sensor(0x2320, 0x4A82);
	HI545_write_cmos_sensor(0x2322, 0x0CAC);
	HI545_write_cmos_sensor(0x2324, 0x4B82);
	HI545_write_cmos_sensor(0x2326, 0x0CAE);
	HI545_write_cmos_sensor(0x2328, 0x3FB6);
	HI545_write_cmos_sensor(0x232a, 0x0B00);
	HI545_write_cmos_sensor(0x232c, 0x7302);
	HI545_write_cmos_sensor(0x232e, 0x0002);
	HI545_write_cmos_sensor(0x2330, 0x069A);
	HI545_write_cmos_sensor(0x2332, 0x0C1F);
	HI545_write_cmos_sensor(0x2334, 0x0403);
	HI545_write_cmos_sensor(0x2336, 0x0C05);
	HI545_write_cmos_sensor(0x2338, 0x0001);
	HI545_write_cmos_sensor(0x233a, 0x0C01);
	HI545_write_cmos_sensor(0x233c, 0x0003);
	HI545_write_cmos_sensor(0x233e, 0x0C03);
	HI545_write_cmos_sensor(0x2340, 0x000B);
	HI545_write_cmos_sensor(0x2342, 0x0C33);
	HI545_write_cmos_sensor(0x2344, 0x0003);
	HI545_write_cmos_sensor(0x2346, 0x0C03);
	HI545_write_cmos_sensor(0x2348, 0x0653);
	HI545_write_cmos_sensor(0x234a, 0x0C03);
	HI545_write_cmos_sensor(0x234c, 0x065B);
	HI545_write_cmos_sensor(0x234e, 0x0C13);
	HI545_write_cmos_sensor(0x2350, 0x065F);
	HI545_write_cmos_sensor(0x2352, 0x0C43);
	HI545_write_cmos_sensor(0x2354, 0x0657);
	HI545_write_cmos_sensor(0x2356, 0x0C03);
	HI545_write_cmos_sensor(0x2358, 0x0653);
	HI545_write_cmos_sensor(0x235a, 0x0C03);
	HI545_write_cmos_sensor(0x235c, 0x0643);
	HI545_write_cmos_sensor(0x235e, 0x0C0F);
	HI545_write_cmos_sensor(0x2360, 0x067D);
	HI545_write_cmos_sensor(0x2362, 0x0C01);
	HI545_write_cmos_sensor(0x2364, 0x077F);
	HI545_write_cmos_sensor(0x2366, 0x0C01);
	HI545_write_cmos_sensor(0x2368, 0x0677);
	HI545_write_cmos_sensor(0x236a, 0x0C01);
	HI545_write_cmos_sensor(0x236c, 0x0673);
	HI545_write_cmos_sensor(0x236e, 0x0C67);
	HI545_write_cmos_sensor(0x2370, 0x0677);
	HI545_write_cmos_sensor(0x2372, 0x0C03);
	HI545_write_cmos_sensor(0x2374, 0x077D);
	HI545_write_cmos_sensor(0x2376, 0x0C19);
	HI545_write_cmos_sensor(0x2378, 0x0013);
	HI545_write_cmos_sensor(0x237a, 0x0C27);
	HI545_write_cmos_sensor(0x237c, 0x0003);
	HI545_write_cmos_sensor(0x237e, 0x0C45);
	HI545_write_cmos_sensor(0x2380, 0x0675);
	HI545_write_cmos_sensor(0x2382, 0x0C01);
	HI545_write_cmos_sensor(0x2384, 0x0671);
	HI545_write_cmos_sensor(0x2386, 0x4392);
	HI545_write_cmos_sensor(0x2388, 0x7004);
	HI545_write_cmos_sensor(0x238a, 0x430F);
	HI545_write_cmos_sensor(0x238c, 0x9382);
	HI545_write_cmos_sensor(0x238e, 0x80B4);
	HI545_write_cmos_sensor(0x2390, 0x2001);
	HI545_write_cmos_sensor(0x2392, 0x431F);
	HI545_write_cmos_sensor(0x2394, 0x4F82);
	HI545_write_cmos_sensor(0x2396, 0x80B4);
	HI545_write_cmos_sensor(0x2398, 0x930F);
	HI545_write_cmos_sensor(0x239a, 0x2473);
	HI545_write_cmos_sensor(0x239c, 0x0B00);
	HI545_write_cmos_sensor(0x239e, 0x7302);
	HI545_write_cmos_sensor(0x23a0, 0x033A);
	HI545_write_cmos_sensor(0x23a2, 0x0675);
	HI545_write_cmos_sensor(0x23a4, 0x0C02);
	HI545_write_cmos_sensor(0x23a6, 0x0339);
	HI545_write_cmos_sensor(0x23a8, 0xAE0C);
	HI545_write_cmos_sensor(0x23aa, 0x0C01);
	HI545_write_cmos_sensor(0x23ac, 0x003C);
	HI545_write_cmos_sensor(0x23ae, 0x0C01);
	HI545_write_cmos_sensor(0x23b0, 0x0004);
	HI545_write_cmos_sensor(0x23b2, 0x0C01);
	HI545_write_cmos_sensor(0x23b4, 0x0642);
	HI545_write_cmos_sensor(0x23b6, 0x0B00);
	HI545_write_cmos_sensor(0x23b8, 0x7302);
	HI545_write_cmos_sensor(0x23ba, 0x0386);
	HI545_write_cmos_sensor(0x23bc, 0x0643);
	HI545_write_cmos_sensor(0x23be, 0x0C05);
	HI545_write_cmos_sensor(0x23c0, 0x0001);
	HI545_write_cmos_sensor(0x23c2, 0x0C01);
	HI545_write_cmos_sensor(0x23c4, 0x0003);
	HI545_write_cmos_sensor(0x23c6, 0x0C03);
	HI545_write_cmos_sensor(0x23c8, 0x000B);
	HI545_write_cmos_sensor(0x23ca, 0x0C33);
	HI545_write_cmos_sensor(0x23cc, 0x0003);
	HI545_write_cmos_sensor(0x23ce, 0x0C03);
	HI545_write_cmos_sensor(0x23d0, 0x0653);
	HI545_write_cmos_sensor(0x23d2, 0x0C03);
	HI545_write_cmos_sensor(0x23d4, 0x065B);
	HI545_write_cmos_sensor(0x23d6, 0x0C13);
	HI545_write_cmos_sensor(0x23d8, 0x065F);
	HI545_write_cmos_sensor(0x23da, 0x0C43);
	HI545_write_cmos_sensor(0x23dc, 0x0657);
	HI545_write_cmos_sensor(0x23de, 0x0C03);
	HI545_write_cmos_sensor(0x23e0, 0x0653);
	HI545_write_cmos_sensor(0x23e2, 0x0C03);
	HI545_write_cmos_sensor(0x23e4, 0x0643);
	HI545_write_cmos_sensor(0x23e6, 0x0C0F);
	HI545_write_cmos_sensor(0x23e8, 0x067D);
	HI545_write_cmos_sensor(0x23ea, 0x0C01);
	HI545_write_cmos_sensor(0x23ec, 0x077F);
	HI545_write_cmos_sensor(0x23ee, 0x0C01);
	HI545_write_cmos_sensor(0x23f0, 0x0677);
	HI545_write_cmos_sensor(0x23f2, 0x0C01);
	HI545_write_cmos_sensor(0x23f4, 0x0673);
	HI545_write_cmos_sensor(0x23f6, 0x0C67);
	HI545_write_cmos_sensor(0x23f8, 0x0677);
	HI545_write_cmos_sensor(0x23fa, 0x0C03);
	HI545_write_cmos_sensor(0x23fc, 0x077D);
	HI545_write_cmos_sensor(0x23fe, 0x0C19);
	HI545_write_cmos_sensor(0x2400, 0x0013);
	HI545_write_cmos_sensor(0x2402, 0x0C27);
	HI545_write_cmos_sensor(0x2404, 0x0003);
	HI545_write_cmos_sensor(0x2406, 0x0C45);
	HI545_write_cmos_sensor(0x2408, 0x0675);
	HI545_write_cmos_sensor(0x240a, 0x0C01);
	HI545_write_cmos_sensor(0x240c, 0x0671);
	HI545_write_cmos_sensor(0x240e, 0x12B0);
	HI545_write_cmos_sensor(0x2410, 0xF6B6);
	HI545_write_cmos_sensor(0x2412, 0x930F);
	HI545_write_cmos_sensor(0x2414, 0x2405);
	HI545_write_cmos_sensor(0x2416, 0x4292);
	HI545_write_cmos_sensor(0x2418, 0x8094);
	HI545_write_cmos_sensor(0x241a, 0x8098);
	HI545_write_cmos_sensor(0x241c, 0x4382);
	HI545_write_cmos_sensor(0x241e, 0x8094);
	HI545_write_cmos_sensor(0x2420, 0x9382);
	HI545_write_cmos_sensor(0x2422, 0x80B4);
	HI545_write_cmos_sensor(0x2424, 0x241E);
	HI545_write_cmos_sensor(0x2426, 0x0B00);
	HI545_write_cmos_sensor(0x2428, 0x7302);
	HI545_write_cmos_sensor(0x242a, 0x069E);
	HI545_write_cmos_sensor(0x242c, 0x0675);
	HI545_write_cmos_sensor(0x242e, 0x0C02);
	HI545_write_cmos_sensor(0x2430, 0x0339);
	HI545_write_cmos_sensor(0x2432, 0xAE0C);
	HI545_write_cmos_sensor(0x2434, 0x0C01);
	HI545_write_cmos_sensor(0x2436, 0x003C);
	HI545_write_cmos_sensor(0x2438, 0x0C01);
	HI545_write_cmos_sensor(0x243a, 0x0004);
	HI545_write_cmos_sensor(0x243c, 0x0C01);
	HI545_write_cmos_sensor(0x243e, 0x0642);
	HI545_write_cmos_sensor(0x2440, 0x0C01);
	HI545_write_cmos_sensor(0x2442, 0x06A1);
	HI545_write_cmos_sensor(0x2444, 0x0C03);
	HI545_write_cmos_sensor(0x2446, 0x06A0);
	HI545_write_cmos_sensor(0x2448, 0x9382);
	HI545_write_cmos_sensor(0x244a, 0x80C8);
	HI545_write_cmos_sensor(0x244c, 0x2004);
	HI545_write_cmos_sensor(0x244e, 0x930F);
	HI545_write_cmos_sensor(0x2450, 0x26B4);
	HI545_write_cmos_sensor(0x2452, 0x4030);
	HI545_write_cmos_sensor(0x2454, 0xF180);
	HI545_write_cmos_sensor(0x2456, 0x43C2);
	HI545_write_cmos_sensor(0x2458, 0x0A80);
	HI545_write_cmos_sensor(0x245a, 0x0B00);
	HI545_write_cmos_sensor(0x245c, 0x7302);
	HI545_write_cmos_sensor(0x245e, 0xFFF0);
	HI545_write_cmos_sensor(0x2460, 0x3EAC);
	HI545_write_cmos_sensor(0x2462, 0x0B00);
	HI545_write_cmos_sensor(0x2464, 0x7302);
	HI545_write_cmos_sensor(0x2466, 0x069E);
	HI545_write_cmos_sensor(0x2468, 0x0675);
	HI545_write_cmos_sensor(0x246a, 0x0C02);
	HI545_write_cmos_sensor(0x246c, 0x0301);
	HI545_write_cmos_sensor(0x246e, 0xAE0C);
	HI545_write_cmos_sensor(0x2470, 0x0C01);
	HI545_write_cmos_sensor(0x2472, 0x0004);
	HI545_write_cmos_sensor(0x2474, 0x0C03);
	HI545_write_cmos_sensor(0x2476, 0x0642);
	HI545_write_cmos_sensor(0x2478, 0x0C01);
	HI545_write_cmos_sensor(0x247a, 0x06A1);
	HI545_write_cmos_sensor(0x247c, 0x0C03);
	HI545_write_cmos_sensor(0x247e, 0x06A0);
	HI545_write_cmos_sensor(0x2480, 0x3FE3);
	HI545_write_cmos_sensor(0x2482, 0x0B00);
	HI545_write_cmos_sensor(0x2484, 0x7302);
	HI545_write_cmos_sensor(0x2486, 0x033A);
	HI545_write_cmos_sensor(0x2488, 0x0675);
	HI545_write_cmos_sensor(0x248a, 0x0C02);
	HI545_write_cmos_sensor(0x248c, 0x0301);
	HI545_write_cmos_sensor(0x248e, 0xAE0C);
	HI545_write_cmos_sensor(0x2490, 0x0C01);
	HI545_write_cmos_sensor(0x2492, 0x0004);
	HI545_write_cmos_sensor(0x2494, 0x0C03);
	HI545_write_cmos_sensor(0x2496, 0x0642);
	HI545_write_cmos_sensor(0x2498, 0x3F8E);
	HI545_write_cmos_sensor(0x249a, 0x0B00);
	HI545_write_cmos_sensor(0x249c, 0x7302);
	HI545_write_cmos_sensor(0x249e, 0x0002);
	HI545_write_cmos_sensor(0x24a0, 0x069A);
	HI545_write_cmos_sensor(0x24a2, 0x0C1F);
	HI545_write_cmos_sensor(0x24a4, 0x0402);
	HI545_write_cmos_sensor(0x24a6, 0x0C05);
	HI545_write_cmos_sensor(0x24a8, 0x0001);
	HI545_write_cmos_sensor(0x24aa, 0x0C01);
	HI545_write_cmos_sensor(0x24ac, 0x0003);
	HI545_write_cmos_sensor(0x24ae, 0x0C03);
	HI545_write_cmos_sensor(0x24b0, 0x000B);
	HI545_write_cmos_sensor(0x24b2, 0x0C33);
	HI545_write_cmos_sensor(0x24b4, 0x0003);
	HI545_write_cmos_sensor(0x24b6, 0x0C03);
	HI545_write_cmos_sensor(0x24b8, 0x0653);
	HI545_write_cmos_sensor(0x24ba, 0x0C03);
	HI545_write_cmos_sensor(0x24bc, 0x065B);
	HI545_write_cmos_sensor(0x24be, 0x0C13);
	HI545_write_cmos_sensor(0x24c0, 0x065F);
	HI545_write_cmos_sensor(0x24c2, 0x0C43);
	HI545_write_cmos_sensor(0x24c4, 0x0657);
	HI545_write_cmos_sensor(0x24c6, 0x0C03);
	HI545_write_cmos_sensor(0x24c8, 0x0653);
	HI545_write_cmos_sensor(0x24ca, 0x0C03);
	HI545_write_cmos_sensor(0x24cc, 0x0643);
	HI545_write_cmos_sensor(0x24ce, 0x0C0F);
	HI545_write_cmos_sensor(0x24d0, 0x077D);
	HI545_write_cmos_sensor(0x24d2, 0x0C01);
	HI545_write_cmos_sensor(0x24d4, 0x067F);
	HI545_write_cmos_sensor(0x24d6, 0x0C01);
	HI545_write_cmos_sensor(0x24d8, 0x0677);
	HI545_write_cmos_sensor(0x24da, 0x0C01);
	HI545_write_cmos_sensor(0x24dc, 0x0673);
	HI545_write_cmos_sensor(0x24de, 0x0C5F);
	HI545_write_cmos_sensor(0x24e0, 0x0663);
	HI545_write_cmos_sensor(0x24e2, 0x0C6F);
	HI545_write_cmos_sensor(0x24e4, 0x0667);
	HI545_write_cmos_sensor(0x24e6, 0x0C01);
	HI545_write_cmos_sensor(0x24e8, 0x0677);
	HI545_write_cmos_sensor(0x24ea, 0x0C01);
	HI545_write_cmos_sensor(0x24ec, 0x077D);
	HI545_write_cmos_sensor(0x24ee, 0x0C33);
	HI545_write_cmos_sensor(0x24f0, 0x0013);
	HI545_write_cmos_sensor(0x24f2, 0x0C27);
	HI545_write_cmos_sensor(0x24f4, 0x0003);
	HI545_write_cmos_sensor(0x24f6, 0x0C4F);
	HI545_write_cmos_sensor(0x24f8, 0x0675);
	HI545_write_cmos_sensor(0x24fa, 0x0C01);
	HI545_write_cmos_sensor(0x24fc, 0x0671);
	HI545_write_cmos_sensor(0x24fe, 0x0CFF);
	HI545_write_cmos_sensor(0x2500, 0x0C78);
	HI545_write_cmos_sensor(0x2502, 0x0661);
	HI545_write_cmos_sensor(0x2504, 0x4392);
	HI545_write_cmos_sensor(0x2506, 0x7004);
	HI545_write_cmos_sensor(0x2508, 0x430F);
	HI545_write_cmos_sensor(0x250a, 0x9382);
	HI545_write_cmos_sensor(0x250c, 0x80B4);
	HI545_write_cmos_sensor(0x250e, 0x2001);
	HI545_write_cmos_sensor(0x2510, 0x431F);
	HI545_write_cmos_sensor(0x2512, 0x4F82);
	HI545_write_cmos_sensor(0x2514, 0x80B4);
	HI545_write_cmos_sensor(0x2516, 0x12B0);
	HI545_write_cmos_sensor(0x2518, 0xF6B6);
	HI545_write_cmos_sensor(0x251a, 0x930F);
	HI545_write_cmos_sensor(0x251c, 0x2405);
	HI545_write_cmos_sensor(0x251e, 0x4292);
	HI545_write_cmos_sensor(0x2520, 0x8094);
	HI545_write_cmos_sensor(0x2522, 0x8098);
	HI545_write_cmos_sensor(0x2524, 0x4382);
	HI545_write_cmos_sensor(0x2526, 0x8094);
	HI545_write_cmos_sensor(0x2528, 0x9382);
	HI545_write_cmos_sensor(0x252a, 0x80B4);
	HI545_write_cmos_sensor(0x252c, 0x2019);
	HI545_write_cmos_sensor(0x252e, 0x0B00);
	HI545_write_cmos_sensor(0x2530, 0x7302);
	HI545_write_cmos_sensor(0x2532, 0x0562);
	HI545_write_cmos_sensor(0x2534, 0x0665);
	HI545_write_cmos_sensor(0x2536, 0x0C02);
	HI545_write_cmos_sensor(0x2538, 0x0301);
	HI545_write_cmos_sensor(0x253a, 0xA60C);
	HI545_write_cmos_sensor(0x253c, 0x0204);
	HI545_write_cmos_sensor(0x253e, 0xAE0C);
	HI545_write_cmos_sensor(0x2540, 0x0C03);
	HI545_write_cmos_sensor(0x2542, 0x0642);
	HI545_write_cmos_sensor(0x2544, 0x0C13);
	HI545_write_cmos_sensor(0x2546, 0x06A1);
	HI545_write_cmos_sensor(0x2548, 0x0C03);
	HI545_write_cmos_sensor(0x254a, 0x06A0);
	HI545_write_cmos_sensor(0x254c, 0x9382);
	HI545_write_cmos_sensor(0x254e, 0x80C8);
	HI545_write_cmos_sensor(0x2550, 0x277E);
	HI545_write_cmos_sensor(0x2552, 0x43C2);
	HI545_write_cmos_sensor(0x2554, 0x0A80);
	HI545_write_cmos_sensor(0x2556, 0x0B00);
	HI545_write_cmos_sensor(0x2558, 0x7302);
	HI545_write_cmos_sensor(0x255a, 0xFFF0);
	HI545_write_cmos_sensor(0x255c, 0x4030);
	HI545_write_cmos_sensor(0x255e, 0xF1BA);
	HI545_write_cmos_sensor(0x2560, 0x0B00);
	HI545_write_cmos_sensor(0x2562, 0x7302);
	HI545_write_cmos_sensor(0x2564, 0x0562);
	HI545_write_cmos_sensor(0x2566, 0x0665);
	HI545_write_cmos_sensor(0x2568, 0x0C02);
	HI545_write_cmos_sensor(0x256a, 0x0339);
	HI545_write_cmos_sensor(0x256c, 0xA60C);
	HI545_write_cmos_sensor(0x256e, 0x023C);
	HI545_write_cmos_sensor(0x2570, 0xAE0C);
	HI545_write_cmos_sensor(0x2572, 0x0C01);
	HI545_write_cmos_sensor(0x2574, 0x0004);
	HI545_write_cmos_sensor(0x2576, 0x0C01);
	HI545_write_cmos_sensor(0x2578, 0x0642);
	HI545_write_cmos_sensor(0x257a, 0x0C13);
	HI545_write_cmos_sensor(0x257c, 0x06A1);
	HI545_write_cmos_sensor(0x257e, 0x0C03);
	HI545_write_cmos_sensor(0x2580, 0x06A0);
	HI545_write_cmos_sensor(0x2582, 0x9382);
	HI545_write_cmos_sensor(0x2584, 0x80C8);
	HI545_write_cmos_sensor(0x2586, 0x2763);
	HI545_write_cmos_sensor(0x2588, 0x43C2);
	HI545_write_cmos_sensor(0x258a, 0x0A80);
	HI545_write_cmos_sensor(0x258c, 0x0B00);
	HI545_write_cmos_sensor(0x258e, 0x7302);
	HI545_write_cmos_sensor(0x2590, 0xFFF0);
	HI545_write_cmos_sensor(0x2592, 0x4030);
	HI545_write_cmos_sensor(0x2594, 0xF1BA);
	HI545_write_cmos_sensor(0x2596, 0xB3E2);
	HI545_write_cmos_sensor(0x2598, 0x00B4);
	HI545_write_cmos_sensor(0x259a, 0x2002);
	HI545_write_cmos_sensor(0x259c, 0x4030);
	HI545_write_cmos_sensor(0x259e, 0xF166);
	HI545_write_cmos_sensor(0x25a0, 0x4319);
	HI545_write_cmos_sensor(0x25a2, 0x4030);
	HI545_write_cmos_sensor(0x25a4, 0xF166);
	HI545_write_cmos_sensor(0x25a6, 0x4392);
	HI545_write_cmos_sensor(0x25a8, 0x760E);
	HI545_write_cmos_sensor(0x25aa, 0x425F);
	HI545_write_cmos_sensor(0x25ac, 0x0118);
	HI545_write_cmos_sensor(0x25ae, 0xF37F);
	HI545_write_cmos_sensor(0x25b0, 0x930F);
	HI545_write_cmos_sensor(0x25b2, 0x2005);
	HI545_write_cmos_sensor(0x25b4, 0x43C2);
	HI545_write_cmos_sensor(0x25b6, 0x0A80);
	HI545_write_cmos_sensor(0x25b8, 0x0B00);
	HI545_write_cmos_sensor(0x25ba, 0x7302);
	HI545_write_cmos_sensor(0x25bc, 0xFFF0);
	HI545_write_cmos_sensor(0x25be, 0x9382);
	HI545_write_cmos_sensor(0x25c0, 0x760C);
	HI545_write_cmos_sensor(0x25c2, 0x2002);
	HI545_write_cmos_sensor(0x25c4, 0x0C64);
	HI545_write_cmos_sensor(0x25c6, 0x3FF1);
	HI545_write_cmos_sensor(0x25c8, 0x4F82);
	HI545_write_cmos_sensor(0x25ca, 0x8096);
	HI545_write_cmos_sensor(0x25cc, 0x12B0);
	HI545_write_cmos_sensor(0x25ce, 0xFCE2);
	HI545_write_cmos_sensor(0x25d0, 0x421F);
	HI545_write_cmos_sensor(0x25d2, 0x760A);
	HI545_write_cmos_sensor(0x25d4, 0x903F);
	HI545_write_cmos_sensor(0x25d6, 0x0200);
	HI545_write_cmos_sensor(0x25d8, 0x244A);
	HI545_write_cmos_sensor(0x25da, 0x930F);
	HI545_write_cmos_sensor(0x25dc, 0x2448);
	HI545_write_cmos_sensor(0x25de, 0x903F);
	HI545_write_cmos_sensor(0x25e0, 0x0100);
	HI545_write_cmos_sensor(0x25e2, 0x23E1);
	HI545_write_cmos_sensor(0x25e4, 0x40B2);
	HI545_write_cmos_sensor(0x25e6, 0x0005);
	HI545_write_cmos_sensor(0x25e8, 0x7600);
	HI545_write_cmos_sensor(0x25ea, 0x4382);
	HI545_write_cmos_sensor(0x25ec, 0x7602);
	HI545_write_cmos_sensor(0x25ee, 0x0262);
	HI545_write_cmos_sensor(0x25f0, 0x0000);
	HI545_write_cmos_sensor(0x25f2, 0x0222);
	HI545_write_cmos_sensor(0x25f4, 0x0000);
	HI545_write_cmos_sensor(0x25f6, 0x0262);
	HI545_write_cmos_sensor(0x25f8, 0x0000);
	HI545_write_cmos_sensor(0x25fa, 0x0260);
	HI545_write_cmos_sensor(0x25fc, 0x0000);
	HI545_write_cmos_sensor(0x25fe, 0x4308);
	HI545_write_cmos_sensor(0x2600, 0x430E);
	HI545_write_cmos_sensor(0x2602, 0x4292);
	HI545_write_cmos_sensor(0x2604, 0x018A);
	HI545_write_cmos_sensor(0x2606, 0x7600);
	HI545_write_cmos_sensor(0x2608, 0x4882);
	HI545_write_cmos_sensor(0x260a, 0x7602);
	HI545_write_cmos_sensor(0x260c, 0x425F);
	HI545_write_cmos_sensor(0x260e, 0x0186);
	HI545_write_cmos_sensor(0x2610, 0xF37F);
	HI545_write_cmos_sensor(0x2612, 0x4F82);
	HI545_write_cmos_sensor(0x2614, 0x7604);
	HI545_write_cmos_sensor(0x2616, 0x0264);
	HI545_write_cmos_sensor(0x2618, 0x0000);
	HI545_write_cmos_sensor(0x261a, 0x0224);
	HI545_write_cmos_sensor(0x261c, 0x0000);
	HI545_write_cmos_sensor(0x261e, 0x0264);
	HI545_write_cmos_sensor(0x2620, 0x0000);
	HI545_write_cmos_sensor(0x2622, 0x0260);
	HI545_write_cmos_sensor(0x2624, 0x0000);
	HI545_write_cmos_sensor(0x2626, 0x0268);
	HI545_write_cmos_sensor(0x2628, 0x0000);
	HI545_write_cmos_sensor(0x262a, 0x0C5A);
	HI545_write_cmos_sensor(0x262c, 0x02E8);
	HI545_write_cmos_sensor(0x262e, 0x0000);
	HI545_write_cmos_sensor(0x2630, 0x0CB5);
	HI545_write_cmos_sensor(0x2632, 0x02A8);
	HI545_write_cmos_sensor(0x2634, 0x0000);
	HI545_write_cmos_sensor(0x2636, 0x0CB5);
	HI545_write_cmos_sensor(0x2638, 0x0CB5);
	HI545_write_cmos_sensor(0x263a, 0x0CB5);
	HI545_write_cmos_sensor(0x263c, 0x0CB5);
	HI545_write_cmos_sensor(0x263e, 0x0CB5);
	HI545_write_cmos_sensor(0x2640, 0x0CB5);
	HI545_write_cmos_sensor(0x2642, 0x0CB5);
	HI545_write_cmos_sensor(0x2644, 0x0CB5);
	HI545_write_cmos_sensor(0x2646, 0x0C00);
	HI545_write_cmos_sensor(0x2648, 0x02E8);
	HI545_write_cmos_sensor(0x264a, 0x0000);
	HI545_write_cmos_sensor(0x264c, 0x0CB5);
	HI545_write_cmos_sensor(0x264e, 0x0268);
	HI545_write_cmos_sensor(0x2650, 0x0000);
	HI545_write_cmos_sensor(0x2652, 0x0C5A);
	HI545_write_cmos_sensor(0x2654, 0x0260);
	HI545_write_cmos_sensor(0x2656, 0x0000);
	HI545_write_cmos_sensor(0x2658, 0x0C5A);
	HI545_write_cmos_sensor(0x265a, 0x531E);
	HI545_write_cmos_sensor(0x265c, 0x903E);
	HI545_write_cmos_sensor(0x265e, 0x0010);
	HI545_write_cmos_sensor(0x2660, 0x2BD0);
	HI545_write_cmos_sensor(0x2662, 0x5318);
	HI545_write_cmos_sensor(0x2664, 0x9238);
	HI545_write_cmos_sensor(0x2666, 0x2BCC);
	HI545_write_cmos_sensor(0x2668, 0x0261);
	HI545_write_cmos_sensor(0x266a, 0x0000);
	HI545_write_cmos_sensor(0x266c, 0x3F9C);
	HI545_write_cmos_sensor(0x266e, 0x421F);
	HI545_write_cmos_sensor(0x2670, 0x018A);
	HI545_write_cmos_sensor(0x2672, 0x12B0);
	HI545_write_cmos_sensor(0x2674, 0xFCF4);
	HI545_write_cmos_sensor(0x2676, 0x421F);
	HI545_write_cmos_sensor(0x2678, 0x7606);
	HI545_write_cmos_sensor(0x267a, 0x4FC2);
	HI545_write_cmos_sensor(0x267c, 0x0188);
	HI545_write_cmos_sensor(0x267e, 0x0261);
	HI545_write_cmos_sensor(0x2680, 0x0000);
	HI545_write_cmos_sensor(0x2682, 0x3F91);
	HI545_write_cmos_sensor(0x2684, 0x4382);
	HI545_write_cmos_sensor(0x2686, 0x0B88);
	HI545_write_cmos_sensor(0x2688, 0x0C0A);
	HI545_write_cmos_sensor(0x268a, 0x4382);
	HI545_write_cmos_sensor(0x268c, 0x0B8A);
	HI545_write_cmos_sensor(0x268e, 0x0C0A);
	HI545_write_cmos_sensor(0x2690, 0x40B2);
	HI545_write_cmos_sensor(0x2692, 0x000C);
	HI545_write_cmos_sensor(0x2694, 0x0B8C);
	HI545_write_cmos_sensor(0x2696, 0x0C0A);
	HI545_write_cmos_sensor(0x2698, 0x40B2);
	HI545_write_cmos_sensor(0x269a, 0xB5E1);
	HI545_write_cmos_sensor(0x269c, 0x0B8E);
	HI545_write_cmos_sensor(0x269e, 0x0C0A);
	HI545_write_cmos_sensor(0x26a0, 0x40B2);
	HI545_write_cmos_sensor(0x26a2, 0x641C);
	HI545_write_cmos_sensor(0x26a4, 0x0B92);
	HI545_write_cmos_sensor(0x26a6, 0x0C0A);
	HI545_write_cmos_sensor(0x26a8, 0x43C2);
	HI545_write_cmos_sensor(0x26aa, 0x003D);
	HI545_write_cmos_sensor(0x26ac, 0x4030);
	HI545_write_cmos_sensor(0x26ae, 0xF02C);
	HI545_write_cmos_sensor(0x26b0, 0x5231);
	HI545_write_cmos_sensor(0x26b2, 0x4030);
	HI545_write_cmos_sensor(0x26b4, 0xFD10);
	HI545_write_cmos_sensor(0x26b6, 0xE3B2);
	HI545_write_cmos_sensor(0x26b8, 0x740E);
	HI545_write_cmos_sensor(0x26ba, 0x425F);
	HI545_write_cmos_sensor(0x26bc, 0x0118);
	HI545_write_cmos_sensor(0x26be, 0xF37F);
	HI545_write_cmos_sensor(0x26c0, 0x4F82);
	HI545_write_cmos_sensor(0x26c2, 0x8096);
	HI545_write_cmos_sensor(0x26c4, 0x930F);
	HI545_write_cmos_sensor(0x26c6, 0x2005);
	HI545_write_cmos_sensor(0x26c8, 0x93C2);
	HI545_write_cmos_sensor(0x26ca, 0x0A82);
	HI545_write_cmos_sensor(0x26cc, 0x2402);
	HI545_write_cmos_sensor(0x26ce, 0x4392);
	HI545_write_cmos_sensor(0x26d0, 0x80C8);
	HI545_write_cmos_sensor(0x26d2, 0x9382);
	HI545_write_cmos_sensor(0x26d4, 0x8096);
	HI545_write_cmos_sensor(0x26d6, 0x2002);
	HI545_write_cmos_sensor(0x26d8, 0x4392);
	HI545_write_cmos_sensor(0x26da, 0x8070);
	HI545_write_cmos_sensor(0x26dc, 0x421F);
	HI545_write_cmos_sensor(0x26de, 0x710E);
	HI545_write_cmos_sensor(0x26e0, 0x93A2);
	HI545_write_cmos_sensor(0x26e2, 0x7110);
	HI545_write_cmos_sensor(0x26e4, 0x2411);
	HI545_write_cmos_sensor(0x26e6, 0x9382);
	HI545_write_cmos_sensor(0x26e8, 0x710E);
	HI545_write_cmos_sensor(0x26ea, 0x240C);
	HI545_write_cmos_sensor(0x26ec, 0x5292);
	HI545_write_cmos_sensor(0x26ee, 0x809A);
	HI545_write_cmos_sensor(0x26f0, 0x7110);
	HI545_write_cmos_sensor(0x26f2, 0x4382);
	HI545_write_cmos_sensor(0x26f4, 0x740E);
	HI545_write_cmos_sensor(0x26f6, 0x9382);
	HI545_write_cmos_sensor(0x26f8, 0x80B2);
	HI545_write_cmos_sensor(0x26fa, 0x2402);
	HI545_write_cmos_sensor(0x26fc, 0x4392);
	HI545_write_cmos_sensor(0x26fe, 0x740E);
	HI545_write_cmos_sensor(0x2700, 0x4392);
	HI545_write_cmos_sensor(0x2702, 0x80B4);
	HI545_write_cmos_sensor(0x2704, 0x430F);
	HI545_write_cmos_sensor(0x2706, 0x4130);
	HI545_write_cmos_sensor(0x2708, 0xF31F);
	HI545_write_cmos_sensor(0x270a, 0x27ED);
	HI545_write_cmos_sensor(0x270c, 0x40B2);
	HI545_write_cmos_sensor(0x270e, 0x0003);
	HI545_write_cmos_sensor(0x2710, 0x7110);
	HI545_write_cmos_sensor(0x2712, 0x431F);
	HI545_write_cmos_sensor(0x2714, 0x4130);
	HI545_write_cmos_sensor(0x2716, 0x4F0E);
	HI545_write_cmos_sensor(0x2718, 0x421D);
	HI545_write_cmos_sensor(0x271a, 0x8070);
	HI545_write_cmos_sensor(0x271c, 0x425F);
	HI545_write_cmos_sensor(0x271e, 0x0118);
	HI545_write_cmos_sensor(0x2720, 0xF37F);
	HI545_write_cmos_sensor(0x2722, 0x903E);
	HI545_write_cmos_sensor(0x2724, 0x0003);
	HI545_write_cmos_sensor(0x2726, 0x2403);
	HI545_write_cmos_sensor(0x2728, 0x0B00);
	HI545_write_cmos_sensor(0x272a, 0x7302);
	HI545_write_cmos_sensor(0x272c, 0x03E8);
	HI545_write_cmos_sensor(0x272e, 0x930F);
	HI545_write_cmos_sensor(0x2730, 0x241A);
	HI545_write_cmos_sensor(0x2732, 0x930D);
	HI545_write_cmos_sensor(0x2734, 0x2018);
	HI545_write_cmos_sensor(0x2736, 0x9382);
	HI545_write_cmos_sensor(0x2738, 0x7308);
	HI545_write_cmos_sensor(0x273a, 0x2402);
	HI545_write_cmos_sensor(0x273c, 0x930E);
	HI545_write_cmos_sensor(0x273e, 0x2419);
	HI545_write_cmos_sensor(0x2740, 0x9382);
	HI545_write_cmos_sensor(0x2742, 0x7328);
	HI545_write_cmos_sensor(0x2744, 0x2402);
	HI545_write_cmos_sensor(0x2746, 0x931E);
	HI545_write_cmos_sensor(0x2748, 0x2414);
	HI545_write_cmos_sensor(0x274a, 0x9382);
	HI545_write_cmos_sensor(0x274c, 0x710E);
	HI545_write_cmos_sensor(0x274e, 0x2402);
	HI545_write_cmos_sensor(0x2750, 0x932E);
	HI545_write_cmos_sensor(0x2752, 0x240F);
	HI545_write_cmos_sensor(0x2754, 0x9382);
	HI545_write_cmos_sensor(0x2756, 0x7114);
	HI545_write_cmos_sensor(0x2758, 0x2402);
	HI545_write_cmos_sensor(0x275a, 0x922E);
	HI545_write_cmos_sensor(0x275c, 0x240A);
	HI545_write_cmos_sensor(0x275e, 0x903E);
	HI545_write_cmos_sensor(0x2760, 0x0003);
	HI545_write_cmos_sensor(0x2762, 0x23DC);
	HI545_write_cmos_sensor(0x2764, 0x3C06);
	HI545_write_cmos_sensor(0x2766, 0x43C2);
	HI545_write_cmos_sensor(0x2768, 0x0A80);
	HI545_write_cmos_sensor(0x276a, 0x0B00);
	HI545_write_cmos_sensor(0x276c, 0x7302);
	HI545_write_cmos_sensor(0x276e, 0xFFF0);
	HI545_write_cmos_sensor(0x2770, 0x3FD5);
	HI545_write_cmos_sensor(0x2772, 0x4F82);
	HI545_write_cmos_sensor(0x2774, 0x8096);
	HI545_write_cmos_sensor(0x2776, 0x431F);
	HI545_write_cmos_sensor(0x2778, 0x4130);
	HI545_write_cmos_sensor(0x277a, 0x120B);
	HI545_write_cmos_sensor(0x277c, 0x120A);
	HI545_write_cmos_sensor(0x277e, 0x1209);
	HI545_write_cmos_sensor(0x2780, 0x1208);
	HI545_write_cmos_sensor(0x2782, 0x1207);
	HI545_write_cmos_sensor(0x2784, 0x1206);
	HI545_write_cmos_sensor(0x2786, 0x1205);
	HI545_write_cmos_sensor(0x2788, 0x1204);
	HI545_write_cmos_sensor(0x278a, 0x8221);
	HI545_write_cmos_sensor(0x278c, 0x403B);
	HI545_write_cmos_sensor(0x278e, 0x0016);
	HI545_write_cmos_sensor(0x2790, 0x510B);
	HI545_write_cmos_sensor(0x2792, 0x4F08);
	HI545_write_cmos_sensor(0x2794, 0x4E09);
	HI545_write_cmos_sensor(0x2796, 0x4BA1);
	HI545_write_cmos_sensor(0x2798, 0x0000);
	HI545_write_cmos_sensor(0x279a, 0x4B1A);
	HI545_write_cmos_sensor(0x279c, 0x0002);
	HI545_write_cmos_sensor(0x279e, 0x4B91);
	HI545_write_cmos_sensor(0x27a0, 0x0004);
	HI545_write_cmos_sensor(0x27a2, 0x0002);
	HI545_write_cmos_sensor(0x27a4, 0x4304);
	HI545_write_cmos_sensor(0x27a6, 0x4305);
	HI545_write_cmos_sensor(0x27a8, 0x4306);
	HI545_write_cmos_sensor(0x27aa, 0x4307);
	HI545_write_cmos_sensor(0x27ac, 0x9382);
	HI545_write_cmos_sensor(0x27ae, 0x80AE);
	HI545_write_cmos_sensor(0x27b0, 0x2425);
	HI545_write_cmos_sensor(0x27b2, 0x438A);
	HI545_write_cmos_sensor(0x27b4, 0x0000);
	HI545_write_cmos_sensor(0x27b6, 0x430B);
	HI545_write_cmos_sensor(0x27b8, 0x4B0F);
	HI545_write_cmos_sensor(0x27ba, 0x5F0F);
	HI545_write_cmos_sensor(0x27bc, 0x5F0F);
	HI545_write_cmos_sensor(0x27be, 0x580F);
	HI545_write_cmos_sensor(0x27c0, 0x4C8F);
	HI545_write_cmos_sensor(0x27c2, 0x0000);
	HI545_write_cmos_sensor(0x27c4, 0x4D8F);
	HI545_write_cmos_sensor(0x27c6, 0x0002);
	HI545_write_cmos_sensor(0x27c8, 0x4B0F);
	HI545_write_cmos_sensor(0x27ca, 0x5F0F);
	HI545_write_cmos_sensor(0x27cc, 0x590F);
	HI545_write_cmos_sensor(0x27ce, 0x41AF);
	HI545_write_cmos_sensor(0x27d0, 0x0000);
	HI545_write_cmos_sensor(0x27d2, 0x531B);
	HI545_write_cmos_sensor(0x27d4, 0x923B);
	HI545_write_cmos_sensor(0x27d6, 0x2BF0);
	HI545_write_cmos_sensor(0x27d8, 0x430B);
	HI545_write_cmos_sensor(0x27da, 0x4B0F);
	HI545_write_cmos_sensor(0x27dc, 0x5F0F);
	HI545_write_cmos_sensor(0x27de, 0x5F0F);
	HI545_write_cmos_sensor(0x27e0, 0x580F);
	HI545_write_cmos_sensor(0x27e2, 0x5F34);
	HI545_write_cmos_sensor(0x27e4, 0x6F35);
	HI545_write_cmos_sensor(0x27e6, 0x4B0F);
	HI545_write_cmos_sensor(0x27e8, 0x5F0F);
	HI545_write_cmos_sensor(0x27ea, 0x590F);
	HI545_write_cmos_sensor(0x27ec, 0x4F2E);
	HI545_write_cmos_sensor(0x27ee, 0x430F);
	HI545_write_cmos_sensor(0x27f0, 0x5E06);
	HI545_write_cmos_sensor(0x27f2, 0x6F07);
	HI545_write_cmos_sensor(0x27f4, 0x531B);
	HI545_write_cmos_sensor(0x27f6, 0x923B);
	HI545_write_cmos_sensor(0x27f8, 0x2BF0);
	HI545_write_cmos_sensor(0x27fa, 0x3C18);
	HI545_write_cmos_sensor(0x27fc, 0x4A2E);
	HI545_write_cmos_sensor(0x27fe, 0x4E0F);
	HI545_write_cmos_sensor(0x2800, 0x5F0F);
	HI545_write_cmos_sensor(0x2802, 0x5F0F);
	HI545_write_cmos_sensor(0x2804, 0x580F);
	HI545_write_cmos_sensor(0x2806, 0x4C8F);
	HI545_write_cmos_sensor(0x2808, 0x0000);
	HI545_write_cmos_sensor(0x280a, 0x4D8F);
	HI545_write_cmos_sensor(0x280c, 0x0002);
	HI545_write_cmos_sensor(0x280e, 0x5E0E);
	HI545_write_cmos_sensor(0x2810, 0x590E);
	HI545_write_cmos_sensor(0x2812, 0x41AE);
	HI545_write_cmos_sensor(0x2814, 0x0000);
	HI545_write_cmos_sensor(0x2816, 0x4A2F);
	HI545_write_cmos_sensor(0x2818, 0x903F);
	HI545_write_cmos_sensor(0x281a, 0x0007);
	HI545_write_cmos_sensor(0x281c, 0x2404);
	HI545_write_cmos_sensor(0x281e, 0x531F);
	HI545_write_cmos_sensor(0x2820, 0x4F8A);
	HI545_write_cmos_sensor(0x2822, 0x0000);
	HI545_write_cmos_sensor(0x2824, 0x3FD9);
	HI545_write_cmos_sensor(0x2826, 0x438A);
	HI545_write_cmos_sensor(0x2828, 0x0000);
	HI545_write_cmos_sensor(0x282a, 0x3FD6);
	HI545_write_cmos_sensor(0x282c, 0x440C);
	HI545_write_cmos_sensor(0x282e, 0x450D);
	HI545_write_cmos_sensor(0x2830, 0x460A);
	HI545_write_cmos_sensor(0x2832, 0x470B);
	HI545_write_cmos_sensor(0x2834, 0x12B0);
	HI545_write_cmos_sensor(0x2836, 0xFD4A);
	HI545_write_cmos_sensor(0x2838, 0x4C08);
	HI545_write_cmos_sensor(0x283a, 0x4D09);
	HI545_write_cmos_sensor(0x283c, 0x4C0E);
	HI545_write_cmos_sensor(0x283e, 0x430F);
	HI545_write_cmos_sensor(0x2840, 0x4E0A);
	HI545_write_cmos_sensor(0x2842, 0x4F0B);
	HI545_write_cmos_sensor(0x2844, 0x460C);
	HI545_write_cmos_sensor(0x2846, 0x470D);
	HI545_write_cmos_sensor(0x2848, 0x12B0);
	HI545_write_cmos_sensor(0x284a, 0xFD2A);
	HI545_write_cmos_sensor(0x284c, 0x8E04);
	HI545_write_cmos_sensor(0x284e, 0x7F05);
	HI545_write_cmos_sensor(0x2850, 0x440E);
	HI545_write_cmos_sensor(0x2852, 0x450F);
	HI545_write_cmos_sensor(0x2854, 0xC312);
	HI545_write_cmos_sensor(0x2856, 0x100F);
	HI545_write_cmos_sensor(0x2858, 0x100E);
	HI545_write_cmos_sensor(0x285a, 0x110F);
	HI545_write_cmos_sensor(0x285c, 0x100E);
	HI545_write_cmos_sensor(0x285e, 0x110F);
	HI545_write_cmos_sensor(0x2860, 0x100E);
	HI545_write_cmos_sensor(0x2862, 0x411D);
	HI545_write_cmos_sensor(0x2864, 0x0002);
	HI545_write_cmos_sensor(0x2866, 0x4E8D);
	HI545_write_cmos_sensor(0x2868, 0x0000);
	HI545_write_cmos_sensor(0x286a, 0x480F);
	HI545_write_cmos_sensor(0x286c, 0x5221);
	HI545_write_cmos_sensor(0x286e, 0x4134);
	HI545_write_cmos_sensor(0x2870, 0x4135);
	HI545_write_cmos_sensor(0x2872, 0x4136);
	HI545_write_cmos_sensor(0x2874, 0x4137);
	HI545_write_cmos_sensor(0x2876, 0x4138);
	HI545_write_cmos_sensor(0x2878, 0x4139);
	HI545_write_cmos_sensor(0x287a, 0x413A);
	HI545_write_cmos_sensor(0x287c, 0x413B);
	HI545_write_cmos_sensor(0x287e, 0x4130);
	HI545_write_cmos_sensor(0x2880, 0x120A);
	HI545_write_cmos_sensor(0x2882, 0x4F0D);
	HI545_write_cmos_sensor(0x2884, 0x4E0C);
	HI545_write_cmos_sensor(0x2886, 0x425F);
	HI545_write_cmos_sensor(0x2888, 0x00BA);
	HI545_write_cmos_sensor(0x288a, 0x4F4A);
	HI545_write_cmos_sensor(0x288c, 0x503A);
	HI545_write_cmos_sensor(0x288e, 0x0010);
	HI545_write_cmos_sensor(0x2890, 0x931D);
	HI545_write_cmos_sensor(0x2892, 0x242B);
	HI545_write_cmos_sensor(0x2894, 0x932D);
	HI545_write_cmos_sensor(0x2896, 0x2421);
	HI545_write_cmos_sensor(0x2898, 0x903D);
	HI545_write_cmos_sensor(0x289a, 0x0003);
	HI545_write_cmos_sensor(0x289c, 0x2418);
	HI545_write_cmos_sensor(0x289e, 0x922D);
	HI545_write_cmos_sensor(0x28a0, 0x2413);
	HI545_write_cmos_sensor(0x28a2, 0x903D);
	HI545_write_cmos_sensor(0x28a4, 0x0005);
	HI545_write_cmos_sensor(0x28a6, 0x2407);
	HI545_write_cmos_sensor(0x28a8, 0x903D);
	HI545_write_cmos_sensor(0x28aa, 0x0006);
	HI545_write_cmos_sensor(0x28ac, 0x2028);
	HI545_write_cmos_sensor(0x28ae, 0xC312);
	HI545_write_cmos_sensor(0x28b0, 0x100A);
	HI545_write_cmos_sensor(0x28b2, 0x110A);
	HI545_write_cmos_sensor(0x28b4, 0x3C24);
	HI545_write_cmos_sensor(0x28b6, 0x4A0E);
	HI545_write_cmos_sensor(0x28b8, 0xC312);
	HI545_write_cmos_sensor(0x28ba, 0x100E);
	HI545_write_cmos_sensor(0x28bc, 0x110E);
	HI545_write_cmos_sensor(0x28be, 0x4E0F);
	HI545_write_cmos_sensor(0x28c0, 0x110F);
	HI545_write_cmos_sensor(0x28c2, 0x4E0A);
	HI545_write_cmos_sensor(0x28c4, 0x5F0A);
	HI545_write_cmos_sensor(0x28c6, 0x3C1B);
	HI545_write_cmos_sensor(0x28c8, 0xC312);
	HI545_write_cmos_sensor(0x28ca, 0x100A);
	HI545_write_cmos_sensor(0x28cc, 0x3C18);
	HI545_write_cmos_sensor(0x28ce, 0x4A0E);
	HI545_write_cmos_sensor(0x28d0, 0xC312);
	HI545_write_cmos_sensor(0x28d2, 0x100E);
	HI545_write_cmos_sensor(0x28d4, 0x4E0F);
	HI545_write_cmos_sensor(0x28d6, 0x110F);
	HI545_write_cmos_sensor(0x28d8, 0x3FF3);
	HI545_write_cmos_sensor(0x28da, 0x4A0F);
	HI545_write_cmos_sensor(0x28dc, 0xC312);
	HI545_write_cmos_sensor(0x28de, 0x100F);
	HI545_write_cmos_sensor(0x28e0, 0x4F0E);
	HI545_write_cmos_sensor(0x28e2, 0x110E);
	HI545_write_cmos_sensor(0x28e4, 0x4F0A);
	HI545_write_cmos_sensor(0x28e6, 0x5E0A);
	HI545_write_cmos_sensor(0x28e8, 0x3C0A);
	HI545_write_cmos_sensor(0x28ea, 0x4A0F);
	HI545_write_cmos_sensor(0x28ec, 0xC312);
	HI545_write_cmos_sensor(0x28ee, 0x100F);
	HI545_write_cmos_sensor(0x28f0, 0x4F0E);
	HI545_write_cmos_sensor(0x28f2, 0x110E);
	HI545_write_cmos_sensor(0x28f4, 0x4F0A);
	HI545_write_cmos_sensor(0x28f6, 0x5E0A);
	HI545_write_cmos_sensor(0x28f8, 0x4E0F);
	HI545_write_cmos_sensor(0x28fa, 0x110F);
	HI545_write_cmos_sensor(0x28fc, 0x3FE3);
	HI545_write_cmos_sensor(0x28fe, 0x12B0);
	HI545_write_cmos_sensor(0x2900, 0xFD14);
	HI545_write_cmos_sensor(0x2902, 0x4E0F);
	HI545_write_cmos_sensor(0x2904, 0xC312);
	HI545_write_cmos_sensor(0x2906, 0x100F);
	HI545_write_cmos_sensor(0x2908, 0x110F);
	HI545_write_cmos_sensor(0x290a, 0x110F);
	HI545_write_cmos_sensor(0x290c, 0x110F);
	HI545_write_cmos_sensor(0x290e, 0x413A);
	HI545_write_cmos_sensor(0x2910, 0x4130);
	HI545_write_cmos_sensor(0x2912, 0x120B);
	HI545_write_cmos_sensor(0x2914, 0x120A);
	HI545_write_cmos_sensor(0x2916, 0x1209);
	HI545_write_cmos_sensor(0x2918, 0x1208);
	HI545_write_cmos_sensor(0x291a, 0x425F);
	HI545_write_cmos_sensor(0x291c, 0x0080);
	HI545_write_cmos_sensor(0x291e, 0xF36F);
	HI545_write_cmos_sensor(0x2920, 0x4F0E);
	HI545_write_cmos_sensor(0x2922, 0xF32E);
	HI545_write_cmos_sensor(0x2924, 0x4E82);
	HI545_write_cmos_sensor(0x2926, 0x80B2);
	HI545_write_cmos_sensor(0x2928, 0xB3D2);
	HI545_write_cmos_sensor(0x292a, 0x0786);
	HI545_write_cmos_sensor(0x292c, 0x2402);
	HI545_write_cmos_sensor(0x292e, 0x4392);
	HI545_write_cmos_sensor(0x2930, 0x7002);
	HI545_write_cmos_sensor(0x2932, 0x4392);
	HI545_write_cmos_sensor(0x2934, 0x80B4);
	HI545_write_cmos_sensor(0x2936, 0x4382);
	HI545_write_cmos_sensor(0x2938, 0x740E);
	HI545_write_cmos_sensor(0x293a, 0x9382);
	HI545_write_cmos_sensor(0x293c, 0x80B2);
	HI545_write_cmos_sensor(0x293e, 0x2402);
	HI545_write_cmos_sensor(0x2940, 0x4392);
	HI545_write_cmos_sensor(0x2942, 0x740E);
	HI545_write_cmos_sensor(0x2944, 0x93C2);
	HI545_write_cmos_sensor(0x2946, 0x00C6);
	HI545_write_cmos_sensor(0x2948, 0x2406);
	HI545_write_cmos_sensor(0x294a, 0xB392);
	HI545_write_cmos_sensor(0x294c, 0x732A);
	HI545_write_cmos_sensor(0x294e, 0x2403);
	HI545_write_cmos_sensor(0x2950, 0xB3D2);
	HI545_write_cmos_sensor(0x2952, 0x00C7);
	HI545_write_cmos_sensor(0x2954, 0x2412);
	HI545_write_cmos_sensor(0x2956, 0x4292);
	HI545_write_cmos_sensor(0x2958, 0x01A8);
	HI545_write_cmos_sensor(0x295a, 0x0688);
	HI545_write_cmos_sensor(0x295c, 0x4292);
	HI545_write_cmos_sensor(0x295e, 0x01AA);
	HI545_write_cmos_sensor(0x2960, 0x068A);
	HI545_write_cmos_sensor(0x2962, 0x4292);
	HI545_write_cmos_sensor(0x2964, 0x01AC);
	HI545_write_cmos_sensor(0x2966, 0x068C);
	HI545_write_cmos_sensor(0x2968, 0x4292);
	HI545_write_cmos_sensor(0x296a, 0x01AE);
	HI545_write_cmos_sensor(0x296c, 0x068E);
	HI545_write_cmos_sensor(0x296e, 0x4292);
	HI545_write_cmos_sensor(0x2970, 0x0190);
	HI545_write_cmos_sensor(0x2972, 0x0A92);
	HI545_write_cmos_sensor(0x2974, 0x4292);
	HI545_write_cmos_sensor(0x2976, 0x0192);
	HI545_write_cmos_sensor(0x2978, 0x0A94);
	HI545_write_cmos_sensor(0x297a, 0x430E);
	HI545_write_cmos_sensor(0x297c, 0x425F);
	HI545_write_cmos_sensor(0x297e, 0x00C7);
	HI545_write_cmos_sensor(0x2980, 0xF35F);
	HI545_write_cmos_sensor(0x2982, 0xF37F);
	HI545_write_cmos_sensor(0x2984, 0xF21F);
	HI545_write_cmos_sensor(0x2986, 0x732A);
	HI545_write_cmos_sensor(0x2988, 0x200C);
	HI545_write_cmos_sensor(0x298a, 0xB3D2);
	HI545_write_cmos_sensor(0x298c, 0x00C7);
	HI545_write_cmos_sensor(0x298e, 0x2003);
	HI545_write_cmos_sensor(0x2990, 0xB392);
	HI545_write_cmos_sensor(0x2992, 0x809C);
	HI545_write_cmos_sensor(0x2994, 0x2006);
	HI545_write_cmos_sensor(0x2996, 0xB3A2);
	HI545_write_cmos_sensor(0x2998, 0x732A);
	HI545_write_cmos_sensor(0x299a, 0x2003);
	HI545_write_cmos_sensor(0x299c, 0x9382);
	HI545_write_cmos_sensor(0x299e, 0x8094);
	HI545_write_cmos_sensor(0x29a0, 0x2401);
	HI545_write_cmos_sensor(0x29a2, 0x431E);
	HI545_write_cmos_sensor(0x29a4, 0x4E82);
	HI545_write_cmos_sensor(0x29a6, 0x80AE);
	HI545_write_cmos_sensor(0x29a8, 0x930E);
	HI545_write_cmos_sensor(0x29aa, 0x257F);
	HI545_write_cmos_sensor(0x29ac, 0x4382);
	HI545_write_cmos_sensor(0x29ae, 0x80B6);
	HI545_write_cmos_sensor(0x29b0, 0x421F);
	HI545_write_cmos_sensor(0x29b2, 0x732A);
	HI545_write_cmos_sensor(0x29b4, 0xF31F);
	HI545_write_cmos_sensor(0x29b6, 0x4F82);
	HI545_write_cmos_sensor(0x29b8, 0x809C);
	HI545_write_cmos_sensor(0x29ba, 0x425F);
	HI545_write_cmos_sensor(0x29bc, 0x008C);
	HI545_write_cmos_sensor(0x29be, 0x4FC2);
	HI545_write_cmos_sensor(0x29c0, 0x8092);
	HI545_write_cmos_sensor(0x29c2, 0x43C2);
	HI545_write_cmos_sensor(0x29c4, 0x8093);
	HI545_write_cmos_sensor(0x29c6, 0x425F);
	HI545_write_cmos_sensor(0x29c8, 0x009E);
	HI545_write_cmos_sensor(0x29ca, 0x4F4A);
	HI545_write_cmos_sensor(0x29cc, 0x425F);
	HI545_write_cmos_sensor(0x29ce, 0x009F);
	HI545_write_cmos_sensor(0x29d0, 0xF37F);
	HI545_write_cmos_sensor(0x29d2, 0x5F0A);
	HI545_write_cmos_sensor(0x29d4, 0x110A);
	HI545_write_cmos_sensor(0x29d6, 0x110A);
	HI545_write_cmos_sensor(0x29d8, 0x425F);
	HI545_write_cmos_sensor(0x29da, 0x00B2);
	HI545_write_cmos_sensor(0x29dc, 0x4F4B);
	HI545_write_cmos_sensor(0x29de, 0x425F);
	HI545_write_cmos_sensor(0x29e0, 0x00B3);
	HI545_write_cmos_sensor(0x29e2, 0xF37F);
	HI545_write_cmos_sensor(0x29e4, 0x5F0B);
	HI545_write_cmos_sensor(0x29e6, 0x110B);
	HI545_write_cmos_sensor(0x29e8, 0x110B);
	HI545_write_cmos_sensor(0x29ea, 0x403F);
	HI545_write_cmos_sensor(0x29ec, 0x00BA);
	HI545_write_cmos_sensor(0x29ee, 0x4F6E);
	HI545_write_cmos_sensor(0x29f0, 0x403C);
	HI545_write_cmos_sensor(0x29f2, 0x0813);
	HI545_write_cmos_sensor(0x29f4, 0x403D);
	HI545_write_cmos_sensor(0x29f6, 0x007F);
	HI545_write_cmos_sensor(0x29f8, 0x90FF);
	HI545_write_cmos_sensor(0x29fa, 0x0011);
	HI545_write_cmos_sensor(0x29fc, 0x0000);
	HI545_write_cmos_sensor(0x29fe, 0x2D03);
	HI545_write_cmos_sensor(0x2a00, 0x403C);
	HI545_write_cmos_sensor(0x2a02, 0x1009);
	HI545_write_cmos_sensor(0x2a04, 0x430D);
	HI545_write_cmos_sensor(0x2a06, 0x425E);
	HI545_write_cmos_sensor(0x2a08, 0x00BA);
	HI545_write_cmos_sensor(0x2a0a, 0x4E4F);
	HI545_write_cmos_sensor(0x2a0c, 0x108F);
	HI545_write_cmos_sensor(0x2a0e, 0xDD0F);
	HI545_write_cmos_sensor(0x2a10, 0x4F82);
	HI545_write_cmos_sensor(0x2a12, 0x0B90);
	HI545_write_cmos_sensor(0x2a14, 0x0C0A);
	HI545_write_cmos_sensor(0x2a16, 0x4C82);
	HI545_write_cmos_sensor(0x2a18, 0x0B8A);
	HI545_write_cmos_sensor(0x2a1a, 0x0C0A);
	HI545_write_cmos_sensor(0x2a1c, 0x425F);
	HI545_write_cmos_sensor(0x2a1e, 0x0C87);
	HI545_write_cmos_sensor(0x2a20, 0x4F4E);
	HI545_write_cmos_sensor(0x2a22, 0x425F);
	HI545_write_cmos_sensor(0x2a24, 0x0C88);
	HI545_write_cmos_sensor(0x2a26, 0xF37F);
	HI545_write_cmos_sensor(0x2a28, 0x12B0);
	HI545_write_cmos_sensor(0x2a2a, 0xF880);
	HI545_write_cmos_sensor(0x2a2c, 0x4F82);
	HI545_write_cmos_sensor(0x2a2e, 0x0C8C);
	HI545_write_cmos_sensor(0x2a30, 0x425F);
	HI545_write_cmos_sensor(0x2a32, 0x0C85);
	HI545_write_cmos_sensor(0x2a34, 0x4F4E);
	HI545_write_cmos_sensor(0x2a36, 0x425F);
	HI545_write_cmos_sensor(0x2a38, 0x0C89);
	HI545_write_cmos_sensor(0x2a3a, 0xF37F);
	HI545_write_cmos_sensor(0x2a3c, 0x12B0);
	HI545_write_cmos_sensor(0x2a3e, 0xF880);
	HI545_write_cmos_sensor(0x2a40, 0x4F82);
	HI545_write_cmos_sensor(0x2a42, 0x0C8A);
	HI545_write_cmos_sensor(0x2a44, 0x425E);
	HI545_write_cmos_sensor(0x2a46, 0x00B7);
	HI545_write_cmos_sensor(0x2a48, 0x5E4E);
	HI545_write_cmos_sensor(0x2a4a, 0x4EC2);
	HI545_write_cmos_sensor(0x2a4c, 0x0CB0);
	HI545_write_cmos_sensor(0x2a4e, 0x425F);
	HI545_write_cmos_sensor(0x2a50, 0x00B8);
	HI545_write_cmos_sensor(0x2a52, 0x5F4F);
	HI545_write_cmos_sensor(0x2a54, 0x4FC2);
	HI545_write_cmos_sensor(0x2a56, 0x0CB1);
	HI545_write_cmos_sensor(0x2a58, 0x4A0E);
	HI545_write_cmos_sensor(0x2a5a, 0x5E0E);
	HI545_write_cmos_sensor(0x2a5c, 0x5E0E);
	HI545_write_cmos_sensor(0x2a5e, 0x5E0E);
	HI545_write_cmos_sensor(0x2a60, 0x5E0E);
	HI545_write_cmos_sensor(0x2a62, 0x4B0F);
	HI545_write_cmos_sensor(0x2a64, 0x5F0F);
	HI545_write_cmos_sensor(0x2a66, 0x5F0F);
	HI545_write_cmos_sensor(0x2a68, 0x5F0F);
	HI545_write_cmos_sensor(0x2a6a, 0x5F0F);
	HI545_write_cmos_sensor(0x2a6c, 0x5F0F);
	HI545_write_cmos_sensor(0x2a6e, 0x5F0F);
	HI545_write_cmos_sensor(0x2a70, 0x5F0F);
	HI545_write_cmos_sensor(0x2a72, 0xDF0E);
	HI545_write_cmos_sensor(0x2a74, 0x4E82);
	HI545_write_cmos_sensor(0x2a76, 0x0A8E);
	HI545_write_cmos_sensor(0x2a78, 0xB22B);
	HI545_write_cmos_sensor(0x2a7a, 0x2401);
	HI545_write_cmos_sensor(0x2a7c, 0x533B);
	HI545_write_cmos_sensor(0x2a7e, 0xB3E2);
	HI545_write_cmos_sensor(0x2a80, 0x0080);
	HI545_write_cmos_sensor(0x2a82, 0x2403);
	HI545_write_cmos_sensor(0x2a84, 0x40F2);
	HI545_write_cmos_sensor(0x2a86, 0x0003);
	HI545_write_cmos_sensor(0x2a88, 0x00B5);
	HI545_write_cmos_sensor(0x2a8a, 0x40B2);
	HI545_write_cmos_sensor(0x2a8c, 0x1001);
	HI545_write_cmos_sensor(0x2a8e, 0x7500);
	HI545_write_cmos_sensor(0x2a90, 0x40B2);
	HI545_write_cmos_sensor(0x2a92, 0x0803);
	HI545_write_cmos_sensor(0x2a94, 0x7502);
	HI545_write_cmos_sensor(0x2a96, 0x40B2);
	HI545_write_cmos_sensor(0x2a98, 0x080F);
	HI545_write_cmos_sensor(0x2a9a, 0x7504);
	HI545_write_cmos_sensor(0x2a9c, 0x40B2);
	HI545_write_cmos_sensor(0x2a9e, 0x6003);
	HI545_write_cmos_sensor(0x2aa0, 0x7506);
	HI545_write_cmos_sensor(0x2aa2, 0x40B2);
	HI545_write_cmos_sensor(0x2aa4, 0x0801);
	HI545_write_cmos_sensor(0x2aa6, 0x7508);
	HI545_write_cmos_sensor(0x2aa8, 0x40B2);
	HI545_write_cmos_sensor(0x2aaa, 0x0800);
	HI545_write_cmos_sensor(0x2aac, 0x750A);
	HI545_write_cmos_sensor(0x2aae, 0x403F);
	HI545_write_cmos_sensor(0x2ab0, 0x0003);
	HI545_write_cmos_sensor(0x2ab2, 0x12B0);
	HI545_write_cmos_sensor(0x2ab4, 0xF716);
	HI545_write_cmos_sensor(0x2ab6, 0x421F);
	HI545_write_cmos_sensor(0x2ab8, 0x0098);
	HI545_write_cmos_sensor(0x2aba, 0x821F);
	HI545_write_cmos_sensor(0x2abc, 0x0092);
	HI545_write_cmos_sensor(0x2abe, 0x531F);
	HI545_write_cmos_sensor(0x2ac0, 0xC312);
	HI545_write_cmos_sensor(0x2ac2, 0x100F);
	HI545_write_cmos_sensor(0x2ac4, 0x4F82);
	HI545_write_cmos_sensor(0x2ac6, 0x0A86);
	HI545_write_cmos_sensor(0x2ac8, 0x421F);
	HI545_write_cmos_sensor(0x2aca, 0x00AC);
	HI545_write_cmos_sensor(0x2acc, 0x821F);
	HI545_write_cmos_sensor(0x2ace, 0x00A6);
	HI545_write_cmos_sensor(0x2ad0, 0x531F);
	HI545_write_cmos_sensor(0x2ad2, 0x4F82);
	HI545_write_cmos_sensor(0x2ad4, 0x0A88);
	HI545_write_cmos_sensor(0x2ad6, 0xB0B2);
	HI545_write_cmos_sensor(0x2ad8, 0x0010);
	HI545_write_cmos_sensor(0x2ada, 0x0A84);
	HI545_write_cmos_sensor(0x2adc, 0x2485);
	HI545_write_cmos_sensor(0x2ade, 0x421E);
	HI545_write_cmos_sensor(0x2ae0, 0x068C);
	HI545_write_cmos_sensor(0x2ae2, 0xC312);
	HI545_write_cmos_sensor(0x2ae4, 0x100E);
	HI545_write_cmos_sensor(0x2ae6, 0x4E82);
	HI545_write_cmos_sensor(0x2ae8, 0x0782);
	HI545_write_cmos_sensor(0x2aea, 0x4292);
	HI545_write_cmos_sensor(0x2aec, 0x068E);
	HI545_write_cmos_sensor(0x2aee, 0x0784);
	HI545_write_cmos_sensor(0x2af0, 0xB3D2);
	HI545_write_cmos_sensor(0x2af2, 0x0CB6);
	HI545_write_cmos_sensor(0x2af4, 0x2418);
	HI545_write_cmos_sensor(0x2af6, 0x421A);
	HI545_write_cmos_sensor(0x2af8, 0x0CB8);
	HI545_write_cmos_sensor(0x2afa, 0x430B);
	HI545_write_cmos_sensor(0x2afc, 0x425F);
	HI545_write_cmos_sensor(0x2afe, 0x0CBA);
	HI545_write_cmos_sensor(0x2b00, 0x4F4E);
	HI545_write_cmos_sensor(0x2b02, 0x430F);
	HI545_write_cmos_sensor(0x2b04, 0x4E0F);
	HI545_write_cmos_sensor(0x2b06, 0x430E);
	HI545_write_cmos_sensor(0x2b08, 0xDE0A);
	HI545_write_cmos_sensor(0x2b0a, 0xDF0B);
	HI545_write_cmos_sensor(0x2b0c, 0x421F);
	HI545_write_cmos_sensor(0x2b0e, 0x0CBC);
	HI545_write_cmos_sensor(0x2b10, 0x4F0C);
	HI545_write_cmos_sensor(0x2b12, 0x430D);
	HI545_write_cmos_sensor(0x2b14, 0x421F);
	HI545_write_cmos_sensor(0x2b16, 0x0CBE);
	HI545_write_cmos_sensor(0x2b18, 0x430E);
	HI545_write_cmos_sensor(0x2b1a, 0xDE0C);
	HI545_write_cmos_sensor(0x2b1c, 0xDF0D);
	HI545_write_cmos_sensor(0x2b1e, 0x12B0);
	HI545_write_cmos_sensor(0x2b20, 0xFD4A);
	HI545_write_cmos_sensor(0x2b22, 0x4C82);
	HI545_write_cmos_sensor(0x2b24, 0x0194);
	HI545_write_cmos_sensor(0x2b26, 0xB2A2);
	HI545_write_cmos_sensor(0x2b28, 0x0A84);
	HI545_write_cmos_sensor(0x2b2a, 0x2412);
	HI545_write_cmos_sensor(0x2b2c, 0x421E);
	HI545_write_cmos_sensor(0x2b2e, 0x0A96);
	HI545_write_cmos_sensor(0x2b30, 0xC312);
	HI545_write_cmos_sensor(0x2b32, 0x100E);
	HI545_write_cmos_sensor(0x2b34, 0x110E);
	HI545_write_cmos_sensor(0x2b36, 0x110E);
	HI545_write_cmos_sensor(0x2b38, 0x43C2);
	HI545_write_cmos_sensor(0x2b3a, 0x0A98);
	HI545_write_cmos_sensor(0x2b3c, 0x431D);
	HI545_write_cmos_sensor(0x2b3e, 0x4E0F);
	HI545_write_cmos_sensor(0x2b40, 0x9F82);
	HI545_write_cmos_sensor(0x2b42, 0x0194);
	HI545_write_cmos_sensor(0x2b44, 0x2846);
	HI545_write_cmos_sensor(0x2b46, 0x5E0F);
	HI545_write_cmos_sensor(0x2b48, 0x531D);
	HI545_write_cmos_sensor(0x2b4a, 0x903D);
	HI545_write_cmos_sensor(0x2b4c, 0x0009);
	HI545_write_cmos_sensor(0x2b4e, 0x2BF8);
	HI545_write_cmos_sensor(0x2b50, 0x4292);
	HI545_write_cmos_sensor(0x2b52, 0x0084);
	HI545_write_cmos_sensor(0x2b54, 0x7524);
	HI545_write_cmos_sensor(0x2b56, 0x4292);
	HI545_write_cmos_sensor(0x2b58, 0x0088);
	HI545_write_cmos_sensor(0x2b5a, 0x7316);
	HI545_write_cmos_sensor(0x2b5c, 0x9382);
	HI545_write_cmos_sensor(0x2b5e, 0x8092);
	HI545_write_cmos_sensor(0x2b60, 0x2403);
	HI545_write_cmos_sensor(0x2b62, 0x4292);
	HI545_write_cmos_sensor(0x2b64, 0x008A);
	HI545_write_cmos_sensor(0x2b66, 0x7316);
	HI545_write_cmos_sensor(0x2b68, 0x430E);
	HI545_write_cmos_sensor(0x2b6a, 0x421F);
	HI545_write_cmos_sensor(0x2b6c, 0x0086);
	HI545_write_cmos_sensor(0x2b6e, 0x832F);
	HI545_write_cmos_sensor(0x2b70, 0x9F82);
	HI545_write_cmos_sensor(0x2b72, 0x0084);
	HI545_write_cmos_sensor(0x2b74, 0x2801);
	HI545_write_cmos_sensor(0x2b76, 0x431E);
	HI545_write_cmos_sensor(0x2b78, 0x4292);
	HI545_write_cmos_sensor(0x2b7a, 0x0086);
	HI545_write_cmos_sensor(0x2b7c, 0x7314);
	HI545_write_cmos_sensor(0x2b7e, 0x93C2);
	HI545_write_cmos_sensor(0x2b80, 0x00BC);
	HI545_write_cmos_sensor(0x2b82, 0x2007);
	HI545_write_cmos_sensor(0x2b84, 0xB31E);
	HI545_write_cmos_sensor(0x2b86, 0x2405);
	HI545_write_cmos_sensor(0x2b88, 0x421F);
	HI545_write_cmos_sensor(0x2b8a, 0x0084);
	HI545_write_cmos_sensor(0x2b8c, 0x532F);
	HI545_write_cmos_sensor(0x2b8e, 0x4F82);
	HI545_write_cmos_sensor(0x2b90, 0x7314);
	HI545_write_cmos_sensor(0x2b92, 0x425F);
	HI545_write_cmos_sensor(0x2b94, 0x00BC);
	HI545_write_cmos_sensor(0x2b96, 0xF37F);
	HI545_write_cmos_sensor(0x2b98, 0xFE0F);
	HI545_write_cmos_sensor(0x2b9a, 0x2406);
	HI545_write_cmos_sensor(0x2b9c, 0x421E);
	HI545_write_cmos_sensor(0x2b9e, 0x0086);
	HI545_write_cmos_sensor(0x2ba0, 0x503E);
	HI545_write_cmos_sensor(0x2ba2, 0xFFFD);
	HI545_write_cmos_sensor(0x2ba4, 0x4E82);
	HI545_write_cmos_sensor(0x2ba6, 0x7524);
	HI545_write_cmos_sensor(0x2ba8, 0x430E);
	HI545_write_cmos_sensor(0x2baa, 0x421F);
	HI545_write_cmos_sensor(0x2bac, 0x7524);
	HI545_write_cmos_sensor(0x2bae, 0x9F82);
	HI545_write_cmos_sensor(0x2bb0, 0x7528);
	HI545_write_cmos_sensor(0x2bb2, 0x2C01);
	HI545_write_cmos_sensor(0x2bb4, 0x431E);
	HI545_write_cmos_sensor(0x2bb6, 0x430F);
	HI545_write_cmos_sensor(0x2bb8, 0x9382);
	HI545_write_cmos_sensor(0x2bba, 0x8094);
	HI545_write_cmos_sensor(0x2bbc, 0x2001);
	HI545_write_cmos_sensor(0x2bbe, 0x431F);
	HI545_write_cmos_sensor(0x2bc0, 0xFE0F);
	HI545_write_cmos_sensor(0x2bc2, 0x40B2);
	HI545_write_cmos_sensor(0x2bc4, 0x0032);
	HI545_write_cmos_sensor(0x2bc6, 0x7522);
	HI545_write_cmos_sensor(0x2bc8, 0x2485);
	HI545_write_cmos_sensor(0x2bca, 0x50B2);
	HI545_write_cmos_sensor(0x2bcc, 0x0032);
	HI545_write_cmos_sensor(0x2bce, 0x7522);
	HI545_write_cmos_sensor(0x2bd0, 0x3C81);
	HI545_write_cmos_sensor(0x2bd2, 0x431F);
	HI545_write_cmos_sensor(0x2bd4, 0x4D0E);
	HI545_write_cmos_sensor(0x2bd6, 0x533E);
	HI545_write_cmos_sensor(0x2bd8, 0x930E);
	HI545_write_cmos_sensor(0x2bda, 0x2403);
	HI545_write_cmos_sensor(0x2bdc, 0x5F0F);
	HI545_write_cmos_sensor(0x2bde, 0x831E);
	HI545_write_cmos_sensor(0x2be0, 0x23FD);
	HI545_write_cmos_sensor(0x2be2, 0x4FC2);
	HI545_write_cmos_sensor(0x2be4, 0x0A98);
	HI545_write_cmos_sensor(0x2be6, 0x3FB4);
	HI545_write_cmos_sensor(0x2be8, 0x4292);
	HI545_write_cmos_sensor(0x2bea, 0x0A86);
	HI545_write_cmos_sensor(0x2bec, 0x0782);
	HI545_write_cmos_sensor(0x2bee, 0x421F);
	HI545_write_cmos_sensor(0x2bf0, 0x0A88);
	HI545_write_cmos_sensor(0x2bf2, 0x4B0E);
	HI545_write_cmos_sensor(0x2bf4, 0x930E);
	HI545_write_cmos_sensor(0x2bf6, 0x2404);
	HI545_write_cmos_sensor(0x2bf8, 0xC312);
	HI545_write_cmos_sensor(0x2bfa, 0x100F);
	HI545_write_cmos_sensor(0x2bfc, 0x831E);
	HI545_write_cmos_sensor(0x2bfe, 0x23FC);
	HI545_write_cmos_sensor(0x2c00, 0x4F82);
	HI545_write_cmos_sensor(0x2c02, 0x0784);
	HI545_write_cmos_sensor(0x2c04, 0x3F75);
	HI545_write_cmos_sensor(0x2c06, 0x90F2);
	HI545_write_cmos_sensor(0x2c08, 0x0011);
	HI545_write_cmos_sensor(0x2c0a, 0x00BA);
	HI545_write_cmos_sensor(0x2c0c, 0x280A);
	HI545_write_cmos_sensor(0x2c0e, 0x90F2);
	HI545_write_cmos_sensor(0x2c10, 0x0051);
	HI545_write_cmos_sensor(0x2c12, 0x00BA);
	HI545_write_cmos_sensor(0x2c14, 0x2C06);
	HI545_write_cmos_sensor(0x2c16, 0x403C);
	HI545_write_cmos_sensor(0x2c18, 0x0A13);
	HI545_write_cmos_sensor(0x2c1a, 0x425E);
	HI545_write_cmos_sensor(0x2c1c, 0x00BA);
	HI545_write_cmos_sensor(0x2c1e, 0x535E);
	HI545_write_cmos_sensor(0x2c20, 0x3EF4);
	HI545_write_cmos_sensor(0x2c22, 0x90F2);
	HI545_write_cmos_sensor(0x2c24, 0x0051);
	HI545_write_cmos_sensor(0x2c26, 0x00BA);
	HI545_write_cmos_sensor(0x2c28, 0x2804);
	HI545_write_cmos_sensor(0x2c2a, 0x90F2);
	HI545_write_cmos_sensor(0x2c2c, 0xFF81);
	HI545_write_cmos_sensor(0x2c2e, 0x00BA);
	HI545_write_cmos_sensor(0x2c30, 0x2808);
	HI545_write_cmos_sensor(0x2c32, 0x90F2);
	HI545_write_cmos_sensor(0x2c34, 0xFF81);
	HI545_write_cmos_sensor(0x2c36, 0x00BA);
	HI545_write_cmos_sensor(0x2c38, 0x2807);
	HI545_write_cmos_sensor(0x2c3a, 0x90F2);
	HI545_write_cmos_sensor(0x2c3c, 0xFF91);
	HI545_write_cmos_sensor(0x2c3e, 0x00BA);
	HI545_write_cmos_sensor(0x2c40, 0x2C03);
	HI545_write_cmos_sensor(0x2c42, 0x403C);
	HI545_write_cmos_sensor(0x2c44, 0x0813);
	HI545_write_cmos_sensor(0x2c46, 0x3FE9);
	HI545_write_cmos_sensor(0x2c48, 0x90F2);
	HI545_write_cmos_sensor(0x2c4a, 0xFF91);
	HI545_write_cmos_sensor(0x2c4c, 0x00BA);
	HI545_write_cmos_sensor(0x2c4e, 0x280A);
	HI545_write_cmos_sensor(0x2c50, 0x90F2);
	HI545_write_cmos_sensor(0x2c52, 0xFFB1);
	HI545_write_cmos_sensor(0x2c54, 0x00BA);
	HI545_write_cmos_sensor(0x2c56, 0x2C06);
	HI545_write_cmos_sensor(0x2c58, 0x403D);
	HI545_write_cmos_sensor(0x2c5a, 0x0060);
	HI545_write_cmos_sensor(0x2c5c, 0x425E);
	HI545_write_cmos_sensor(0x2c5e, 0x00BA);
	HI545_write_cmos_sensor(0x2c60, 0x536E);
	HI545_write_cmos_sensor(0x2c62, 0x3ED3);
	HI545_write_cmos_sensor(0x2c64, 0x90F2);
	HI545_write_cmos_sensor(0x2c66, 0xFFB1);
	HI545_write_cmos_sensor(0x2c68, 0x00BA);
	HI545_write_cmos_sensor(0x2c6a, 0x2804);
	HI545_write_cmos_sensor(0x2c6c, 0x90F2);
	HI545_write_cmos_sensor(0x2c6e, 0xFFC1);
	HI545_write_cmos_sensor(0x2c70, 0x00BA);
	HI545_write_cmos_sensor(0x2c72, 0x2808);
	HI545_write_cmos_sensor(0x2c74, 0x90F2);
	HI545_write_cmos_sensor(0x2c76, 0xFFC1);
	HI545_write_cmos_sensor(0x2c78, 0x00BA);
	HI545_write_cmos_sensor(0x2c7a, 0x280B);
	HI545_write_cmos_sensor(0x2c7c, 0x90F2);
	HI545_write_cmos_sensor(0x2c7e, 0xFFD1);
	HI545_write_cmos_sensor(0x2c80, 0x00BA);
	HI545_write_cmos_sensor(0x2c82, 0x2C07);
	HI545_write_cmos_sensor(0x2c84, 0x403D);
	HI545_write_cmos_sensor(0x2c86, 0x0050);
	HI545_write_cmos_sensor(0x2c88, 0x425E);
	HI545_write_cmos_sensor(0x2c8a, 0x00BA);
	HI545_write_cmos_sensor(0x2c8c, 0x507E);
	HI545_write_cmos_sensor(0x2c8e, 0x0003);
	HI545_write_cmos_sensor(0x2c90, 0x3EBC);
	HI545_write_cmos_sensor(0x2c92, 0x403D);
	HI545_write_cmos_sensor(0x2c94, 0x003C);
	HI545_write_cmos_sensor(0x2c96, 0x403F);
	HI545_write_cmos_sensor(0x2c98, 0x00BA);
	HI545_write_cmos_sensor(0x2c9a, 0x4F6E);
	HI545_write_cmos_sensor(0x2c9c, 0x526E);
	HI545_write_cmos_sensor(0x2c9e, 0x90FF);
	HI545_write_cmos_sensor(0x2ca0, 0xFFFB);
	HI545_write_cmos_sensor(0x2ca2, 0x0000);
	HI545_write_cmos_sensor(0x2ca4, 0x2AB2);
	HI545_write_cmos_sensor(0x2ca6, 0x437E);
	HI545_write_cmos_sensor(0x2ca8, 0x3EB0);
	HI545_write_cmos_sensor(0x2caa, 0x421F);
	HI545_write_cmos_sensor(0x2cac, 0x80B6);
	HI545_write_cmos_sensor(0x2cae, 0x903F);
	HI545_write_cmos_sensor(0x2cb0, 0x0009);
	HI545_write_cmos_sensor(0x2cb2, 0x2C04);
	HI545_write_cmos_sensor(0x2cb4, 0x531F);
	HI545_write_cmos_sensor(0x2cb6, 0x4F82);
	HI545_write_cmos_sensor(0x2cb8, 0x80B6);
	HI545_write_cmos_sensor(0x2cba, 0x3E7A);
	HI545_write_cmos_sensor(0x2cbc, 0x421F);
	HI545_write_cmos_sensor(0x2cbe, 0x80B0);
	HI545_write_cmos_sensor(0x2cc0, 0x903F);
	HI545_write_cmos_sensor(0x2cc2, 0x012E);
	HI545_write_cmos_sensor(0x2cc4, 0x2C04);
	HI545_write_cmos_sensor(0x2cc6, 0x531F);
	HI545_write_cmos_sensor(0x2cc8, 0x4F82);
	HI545_write_cmos_sensor(0x2cca, 0x80B0);
	HI545_write_cmos_sensor(0x2ccc, 0x3E71);
	HI545_write_cmos_sensor(0x2cce, 0x4382);
	HI545_write_cmos_sensor(0x2cd0, 0x80B0);
	HI545_write_cmos_sensor(0x2cd2, 0x3E6E);
	HI545_write_cmos_sensor(0x2cd4, 0xD392);
	HI545_write_cmos_sensor(0x2cd6, 0x7102);
	HI545_write_cmos_sensor(0x2cd8, 0x4138);
	HI545_write_cmos_sensor(0x2cda, 0x4139);
	HI545_write_cmos_sensor(0x2cdc, 0x413A);
	HI545_write_cmos_sensor(0x2cde, 0x413B);
	HI545_write_cmos_sensor(0x2ce0, 0x4130);
	HI545_write_cmos_sensor(0x2ce2, 0x0260);
	HI545_write_cmos_sensor(0x2ce4, 0x0000);
	HI545_write_cmos_sensor(0x2ce6, 0x0C5A);
	HI545_write_cmos_sensor(0x2ce8, 0x0240);
	HI545_write_cmos_sensor(0x2cea, 0x0000);
	HI545_write_cmos_sensor(0x2cec, 0x0260);
	HI545_write_cmos_sensor(0x2cee, 0x0000);
	HI545_write_cmos_sensor(0x2cf0, 0x0C14);
	HI545_write_cmos_sensor(0x2cf2, 0x4130);
	HI545_write_cmos_sensor(0x2cf4, 0x4382);
	HI545_write_cmos_sensor(0x2cf6, 0x7602);
	HI545_write_cmos_sensor(0x2cf8, 0x4F82);
	HI545_write_cmos_sensor(0x2cfa, 0x7600);
	HI545_write_cmos_sensor(0x2cfc, 0x0270);
	HI545_write_cmos_sensor(0x2cfe, 0x0000);
	HI545_write_cmos_sensor(0x2d00, 0x0C1B);
	HI545_write_cmos_sensor(0x2d02, 0x0270);
	HI545_write_cmos_sensor(0x2d04, 0x0001);
	HI545_write_cmos_sensor(0x2d06, 0x421F);
	HI545_write_cmos_sensor(0x2d08, 0x7606);
	HI545_write_cmos_sensor(0x2d0a, 0x4FC2);
	HI545_write_cmos_sensor(0x2d0c, 0x0188);
	HI545_write_cmos_sensor(0x2d0e, 0x4130);
	HI545_write_cmos_sensor(0x2d10, 0xDF02);
	HI545_write_cmos_sensor(0x2d12, 0x3FFE);
	HI545_write_cmos_sensor(0x2d14, 0x430E);
	HI545_write_cmos_sensor(0x2d16, 0x930A);
	HI545_write_cmos_sensor(0x2d18, 0x2407);
	HI545_write_cmos_sensor(0x2d1a, 0xC312);
	HI545_write_cmos_sensor(0x2d1c, 0x100C);
	HI545_write_cmos_sensor(0x2d1e, 0x2801);
	HI545_write_cmos_sensor(0x2d20, 0x5A0E);
	HI545_write_cmos_sensor(0x2d22, 0x5A0A);
	HI545_write_cmos_sensor(0x2d24, 0x930C);
	HI545_write_cmos_sensor(0x2d26, 0x23F7);
	HI545_write_cmos_sensor(0x2d28, 0x4130);
	HI545_write_cmos_sensor(0x2d2a, 0x430E);
	HI545_write_cmos_sensor(0x2d2c, 0x430F);
	HI545_write_cmos_sensor(0x2d2e, 0x3C08);
	HI545_write_cmos_sensor(0x2d30, 0xC312);
	HI545_write_cmos_sensor(0x2d32, 0x100D);
	HI545_write_cmos_sensor(0x2d34, 0x100C);
	HI545_write_cmos_sensor(0x2d36, 0x2802);
	HI545_write_cmos_sensor(0x2d38, 0x5A0E);
	HI545_write_cmos_sensor(0x2d3a, 0x6B0F);
	HI545_write_cmos_sensor(0x2d3c, 0x5A0A);
	HI545_write_cmos_sensor(0x2d3e, 0x6B0B);
	HI545_write_cmos_sensor(0x2d40, 0x930C);
	HI545_write_cmos_sensor(0x2d42, 0x23F6);
	HI545_write_cmos_sensor(0x2d44, 0x930D);
	HI545_write_cmos_sensor(0x2d46, 0x23F4);
	HI545_write_cmos_sensor(0x2d48, 0x4130);
	HI545_write_cmos_sensor(0x2d4a, 0xEF0F);
	HI545_write_cmos_sensor(0x2d4c, 0xEE0E);
	HI545_write_cmos_sensor(0x2d4e, 0x4039);
	HI545_write_cmos_sensor(0x2d50, 0x0021);
	HI545_write_cmos_sensor(0x2d52, 0x3C0A);
	HI545_write_cmos_sensor(0x2d54, 0x1008);
	HI545_write_cmos_sensor(0x2d56, 0x6E0E);
	HI545_write_cmos_sensor(0x2d58, 0x6F0F);
	HI545_write_cmos_sensor(0x2d5a, 0x9B0F);
	HI545_write_cmos_sensor(0x2d5c, 0x2805);
	HI545_write_cmos_sensor(0x2d5e, 0x2002);
	HI545_write_cmos_sensor(0x2d60, 0x9A0E);
	HI545_write_cmos_sensor(0x2d62, 0x2802);
	HI545_write_cmos_sensor(0x2d64, 0x8A0E);
	HI545_write_cmos_sensor(0x2d66, 0x7B0F);
	HI545_write_cmos_sensor(0x2d68, 0x6C0C);
	HI545_write_cmos_sensor(0x2d6a, 0x6D0D);
	HI545_write_cmos_sensor(0x2d6c, 0x6808);
	HI545_write_cmos_sensor(0x2d6e, 0x8319);
	HI545_write_cmos_sensor(0x2d70, 0x23F1);
	HI545_write_cmos_sensor(0x2d72, 0x4130);
	HI545_write_cmos_sensor(0x2d74, 0x0000);
	HI545_write_cmos_sensor(0x2ffe, 0xf000);
	HI545_write_cmos_sensor(0x3000, 0x00AE);
	HI545_write_cmos_sensor(0x3002, 0x00AE);
	HI545_write_cmos_sensor(0x3004, 0x00AE);
	HI545_write_cmos_sensor(0x3006, 0x00AE);
	HI545_write_cmos_sensor(0x3008, 0x00AE);
	HI545_write_cmos_sensor(0x4000, 0x0400);
	HI545_write_cmos_sensor(0x4002, 0x0400);
	HI545_write_cmos_sensor(0x4004, 0x0C04);
	HI545_write_cmos_sensor(0x4006, 0x0C04);
	HI545_write_cmos_sensor(0x4008, 0x0C04);

	//--- FW End ---//


	//--- Initial Set file ---//
	HI545_write_cmos_sensor(0x0B04, 0x07CB);
	HI545_write_cmos_sensor(0x0B06, 0x5ED7);
	HI545_write_cmos_sensor(0x0B14, 0x370B); //PLL Main Div[15:8]. 0x1b = Pclk(86.4mhz), 0x37 = Pclk(176mhz)
	HI545_write_cmos_sensor(0x0B16, 0x4A0B);
	HI545_write_cmos_sensor(0x0B18, 0x0000); //ADD 20140620

	HI545_write_cmos_sensor(0x004C, 0x0100); //tg_enable.
	HI545_write_cmos_sensor(0x0032, 0x0101); //Normal
	HI545_write_cmos_sensor(0x0036, 0x0048); //ramp_rst_offset
	HI545_write_cmos_sensor(0x0038, 0x4800); //ramp_sig_poffset
	HI545_write_cmos_sensor(0x0138, 0x0004); //pxl_drv_pwr
	HI545_write_cmos_sensor(0x013A, 0x0100); //tx_idle
	HI545_write_cmos_sensor(0x0C00, 0x3BC1); //BLC_ctl1. Line BLC on = 0x3b, off = 0x2A //LBLC_ctl1. [0]en_blc, [1]en_lblc_dpc, [2]en_channel_blc, [3]en_adj_pxl_dpc, [4]en_adp_dead_pxl_th
	HI545_write_cmos_sensor(0x0C0E, 0x0500); //0x07 BLC display On, 0x05 BLC D off //BLC_ctl3. Frame BLC On = 0x05, off=0x04 //FBLC_ctl3. [0]en_fblc, [1]en_blc_bypass, [2]en_fblc_dpc, [5]en_fobp_dig_offset, [7]en_lobp_dpc_bypass 
	HI545_write_cmos_sensor(0x0C10, 0x0510); //dig_blc_offset_h
	HI545_write_cmos_sensor(0x0C16, 0x0000); //fobp_dig_b_offset. red b[7] sign(0:+,1:-), b[6:0] dc offset 128(max)
	HI545_write_cmos_sensor(0x0C18, 0x0000); //fobp_dig_Gb_offset. Gr b[7] sign(0:+,1:-), b[6:0] dc offset 128(max)
	HI545_write_cmos_sensor(0x0C36, 0x0100); //r_g_sum_ctl. [0]:g_sum_en. '1'enable, '0'disable.

	//--- MIPI blank time --------------//
	HI545_write_cmos_sensor(0x0902, 0x4101); //mipi_value_clk_trail. MIPI CLK mode [1]'1'cont(0x43),'0'non-cont(0x41) [6]'1'2lane(0x41), '0'1lane(0x01) 
	HI545_write_cmos_sensor(0x090A, 0x03E4); //mipi_vblank_delay_h.
	HI545_write_cmos_sensor(0x090C, 0x0020); //mipi_hblank_short_delay_h.
	HI545_write_cmos_sensor(0x090E, 0x0020); //mipi_hblank_long_delay_h.
	HI545_write_cmos_sensor(0x0910, 0x5D07); //05 mipi_LPX
	HI545_write_cmos_sensor(0x0912, 0x061e); //05 mipi_CLK_prepare
	HI545_write_cmos_sensor(0x0914, 0x0407); //02 mipi_clk_pre
	HI545_write_cmos_sensor(0x0916, 0x0b0a); //09 mipi_data_zero
	HI545_write_cmos_sensor(0x0918, 0x0e09); //0c mipi_clk_post
	//----------------------------------//

	//--- Pixel Array Addressing ------//
	HI545_write_cmos_sensor(0x000E, 0x0000); //x_addr_start_lobp_h.
	HI545_write_cmos_sensor(0x0014, 0x003F); //x_addr_end_lobp_h.
	HI545_write_cmos_sensor(0x0010, 0x0050); //x_addr_start_robp_h.
	HI545_write_cmos_sensor(0x0016, 0x008F); //x_addr_end_robp_h.
	HI545_write_cmos_sensor(0x0012, 0x00A4); //x_addr_start_hact_h.
	HI545_write_cmos_sensor(0x0018, 0x0AD3); //x_addr_end_hact_h.
	HI545_write_cmos_sensor(0x0020, 0x0700); //x_regin_sel
	HI545_write_cmos_sensor(0x0022, 0x0004); //y_addr_start_fobp_h.
	HI545_write_cmos_sensor(0x0028, 0x000B); //y_addr_end_fobp_h.  
	HI545_write_cmos_sensor(0x0024, 0xFFFA); //y_addr_start_dummy_h.
	HI545_write_cmos_sensor(0x002A, 0xFFFF); //y_addr_end_dummy_h.  
	HI545_write_cmos_sensor(0x0026, 0x0010); //y_addr_start_vact_h.
	HI545_write_cmos_sensor(0x002C, 0x07B7); //y_addr_end_vact_h.  
	HI545_write_cmos_sensor(0x0034, 0x0700); //Y_region_sel
	//------------------------------//

	//--Crop size 2604x1956 ----///
	HI545_write_cmos_sensor(0x0128, 0x0002); // digital_crop_x_offset_l
	HI545_write_cmos_sensor(0x012A, 0x0000); // digital_crop_y_offset_l
	HI545_write_cmos_sensor(0x012C, 0x0A2C); // digital_crop_image_width
	HI545_write_cmos_sensor(0x012E, 0x07A4); // digital_crop_image_height
	//------------------------------//

	//----< Image FMT Size >--------------------//
	//Image size 2604x1956
	HI545_write_cmos_sensor(0x0110, 0x0A2C); //X_output_size_h
	HI545_write_cmos_sensor(0x0112, 0x07A4); //Y_output_size_h
	//------------------------------------------//

	//----< Frame / Line Length >--------------//
	HI545_write_cmos_sensor(0x0006, 0x07c0); //frame_length_h 1984
	HI545_write_cmos_sensor(0x0008, 0x0B40); //line_length_h 2880
	HI545_write_cmos_sensor(0x000A, 0x0DB0); //line_length for binning 3504
	//---------------------------------------//

	//--- ETC set ----//
	HI545_write_cmos_sensor(0x003C, 0x0000); //fixed frame off. b[0] '1'enable, '0'disable
	HI545_write_cmos_sensor(0x0000, 0x0000); //orientation. [0]:x-flip, [1]:y-flip.
	HI545_write_cmos_sensor(0x0500, 0x0000); //DGA_ctl.  b[1]'0'OTP_color_ratio_disable, '1' OTP_color_ratio_enable, b[2]'0'data_pedestal_en, '1'data_pedestal_dis.
	//----------------//

	//--- AG / DG control ----------------//
	//AG
	HI545_write_cmos_sensor(0x003A, 0x0000); //Analog Gain.  0x00=x1, 0x70=x8, 0xf0=x16.

	//DG
	HI545_write_cmos_sensor(0x0508, 0x0100); //DG_Gr_h.  0x01=x1, 0x07=x8.
	HI545_write_cmos_sensor(0x050a, 0x0100); //DG_Gb_h.  0x01=x1, 0x07=x8.
	HI545_write_cmos_sensor(0x050c, 0x0100); //DG_R_h.  0x01=x1, 0x07=x8.
	HI545_write_cmos_sensor(0x050e, 0x0100); //DG_B_h.  0x01=x1, 0x07=x8.
	//----------------------------------//


	//-----< Exp.Time >------------------------//
	// Pclk_88Mhz @ Line_length_pclk : 2880 @Exp.Time 33.33ms
	HI545_write_cmos_sensor(0x0002, 0x04b0);	//Fine_int : 33.33ms@Pclk88mhz@Line_length2880 
	HI545_write_cmos_sensor(0x0004, 0x07F4); //coarse_int : 33.33ms@Pclk88mhz@Line_length2880


	//--- ISP enable Selection ---------------//
	HI545_write_cmos_sensor(0x0A04, 0x011A); //isp_en. [9]s-gamma,[8]MIPI_en,[6]compresion10to8,[5]Scaler,[4]window,[3]DG,[2]LSC,[1]adpc,[0]tpg
	//----------------------------------------//
	HI545_write_cmos_sensor(0x0118, 0x0100); //sleep Off
	HI545DB("HI545_Sensor_Init exit :\n ");
}   /*  HI545_Sensor_Init  */

/*************************************************************************
* FUNCTION
*   HI545Open
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

UINT32 HI545Open(void)
{

	volatile signed int i;
	kal_uint16 sensor_id = 0;

	HI545DB("HI545Open enter :\n ");

	for(i=0;i<3;i++)
	{
		sensor_id = (HI545_read_cmos_sensor(0x0F17)<<8)|HI545_read_cmos_sensor(0x0F16);
		HI545DB("OHI545 READ ID :%x",sensor_id);
		if(sensor_id != HI545MIPI_SENSOR_ID)
		{
			return ERROR_SENSOR_CONNECT_FAIL;
		}
		else
		{
			break;
		}
	}
	
	spin_lock(&HI545mipiraw_drv_lock);
	HI545.sensorMode = SENSOR_MODE_INIT;
	HI545.HI545AutoFlickerMode = KAL_FALSE;
	HI545.HI545VideoMode = KAL_FALSE;
	spin_unlock(&HI545mipiraw_drv_lock);
	HI545_Sensor_Init();

	spin_lock(&HI545mipiraw_drv_lock);
	HI545.DummyLines= 0;
	HI545.DummyPixels= 0;
	HI545.pvPclk =  ( HI545_PV_CLK / 10000); 
	HI545.videoPclk = ( HI545_VIDEO_CLK / 10000);
	HI545.capPclk = (HI545_CAP_CLK / 10000);

	HI545.shutter = 0x4EA;
	HI545.pvShutter = 0x4EA;
	HI545.maxExposureLines =HI545_PV_PERIOD_LINE_NUMS -4;

	HI545.ispBaseGain = BASEGAIN;//0x40
	HI545.sensorGlobalGain = 0x1f;//sensor gain read from 0x350a 0x350b; 0x1f as 3.875x
	HI545.pvGain = 0x1f;
	HI545.realGain = 0x1f;//ispBaseGain as 1x
	spin_unlock(&HI545mipiraw_drv_lock);


	HI545DB("HI545Open exit :\n ");

    return ERROR_NONE;
}

/*************************************************************************
* FUNCTION
*   HI545GetSensorID
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
UINT32 HI545GetSensorID(UINT32 *sensorID)
{
    int  retry = 3;

	HI545DB("HI545GetSensorID enter :\n ");

    // check if sensor ID correct
    do {
        *sensorID = (HI545_read_cmos_sensor(0x0F17)<<8)|HI545_read_cmos_sensor(0x0F16);
        if (*sensorID == HI545MIPI_SENSOR_ID)
        	{
        		HI545DB("Sensor ID = 0x%04x\n", *sensorID);
            	break;
        	}
        HI545DB("Read Sensor ID Fail = 0x%04x\n", *sensorID);
//      mDELAY(1000);
        retry--;
    } while (retry > 0);
//    } while (1);

    if (*sensorID != HI545MIPI_SENSOR_ID) {
        *sensorID = 0xFFFFFFFF;
        return ERROR_SENSOR_CONNECT_FAIL;
    }
    return ERROR_NONE;
}


/*************************************************************************
* FUNCTION
*   HI545_SetShutter
*
* DESCRIPTION
*   This function set e-shutter of HI545 to change exposure time.
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
void HI545_SetShutter(kal_uint32 iShutter)
{

//   if(HI545.shutter == iShutter)
//   		return;

   spin_lock(&HI545mipiraw_drv_lock);
   HI545.shutter= iShutter;
   spin_unlock(&HI545mipiraw_drv_lock);

   HI545_write_shutter(iShutter);
   return;
 
}   /*  HI545_SetShutter   */



/*************************************************************************
* FUNCTION
*   HI545_read_shutter
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
UINT32 HI545_read_shutter(void)
{

	kal_uint16 temp_reg1, temp_reg2;
	UINT32 shutter =0;
	temp_reg1 = HI545_read_cmos_sensor(0x0004);  
	//temp_reg2 = HI545_read_cmos_sensor(0x0005);    
	//read out register value and divide 16;
	shutter  = temp_reg1;

	return shutter;
}

/*************************************************************************
* FUNCTION
*   HI545_night_mode
*
* DESCRIPTION
*   This function night mode of HI545.
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
void HI545_NightMode(kal_bool bEnable)
{
}/*	HI545_NightMode */



/*************************************************************************
* FUNCTION
*   HI545Close
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
UINT32 HI545Close(void)
{
    //  CISModulePowerOn(FALSE);
    //s_porting
    //  DRV_I2CClose(HI545hDrvI2C);
    //e_porting
    ReEnteyCamera = KAL_FALSE;
    return ERROR_NONE;
}	/* HI545Close() */

void HI545SetFlipMirror(kal_int32 imgMirror)
{
#if 1
	
    switch (imgMirror)
    {
        case IMAGE_NORMAL://IMAGE_NOMAL:
            HI545_write_cmos_sensor(0x0000, 0x0000);//Set normal
            break;    
        case IMAGE_V_MIRROR://IMAGE_V_MIRROR:
            HI545_write_cmos_sensor(0x0000, 0x0200);	//Set flip
            break;
        case IMAGE_H_MIRROR://IMAGE_H_MIRROR:
            HI545_write_cmos_sensor(0x0000, 0x0100);//Set mirror
            break;
        case IMAGE_HV_MIRROR://IMAGE_H_MIRROR:
            HI545_write_cmos_sensor(0x0000, 0x0300);	//Set mirror & flip
            break;            
    }
#endif	
}


/*************************************************************************
* FUNCTION
*   HI545Preview
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
UINT32 HI545Preview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                                                MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{

	HI545DB("HI545Preview enter:");

	// preview size
	if(HI545.sensorMode == SENSOR_MODE_PREVIEW)
	{
		// do nothing
		// FOR CCT PREVIEW
	}
	else
	{
		//HI545DB("HI545Preview setting!!\n");
		HI545PreviewSetting();
	}
	
	spin_lock(&HI545mipiraw_drv_lock);
	HI545.sensorMode = SENSOR_MODE_PREVIEW; // Need set preview setting after capture mode
	HI545.DummyPixels = 0;//define dummy pixels and lines
	HI545.DummyLines = 0 ;
	HI545_FeatureControl_PERIOD_PixelNum=HI545_PV_PERIOD_PIXEL_NUMS+ HI545.DummyPixels;
	HI545_FeatureControl_PERIOD_LineNum=HI545_PV_PERIOD_LINE_NUMS+HI545.DummyLines;
	spin_unlock(&HI545mipiraw_drv_lock);

	spin_lock(&HI545mipiraw_drv_lock);
	HI545.imgMirror = sensor_config_data->SensorImageMirror;
	//HI545.imgMirror =IMAGE_NORMAL; //by module layout
	spin_unlock(&HI545mipiraw_drv_lock);
	
	//HI545SetFlipMirror(sensor_config_data->SensorImageMirror);
	HI545SetFlipMirror(IMAGE_H_MIRROR);
	

	HI545DBSOFIA("[HI545Preview]frame_len=%x\n", ((HI545_read_cmos_sensor(0x380e)<<8)+HI545_read_cmos_sensor(0x380f)));

 //   mDELAY(40);
	HI545DB("HI545Preview exit:\n");
    return ERROR_NONE;
}	/* HI545Preview() */



UINT32 HI545Video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                                                MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{

	HI545DB("HI545Video enter:");

	if(HI545.sensorMode == SENSOR_MODE_VIDEO)
	{
		// do nothing
	}
	else
	{
		HI545VideoSetting();

	}
	spin_lock(&HI545mipiraw_drv_lock);
	HI545.sensorMode = SENSOR_MODE_VIDEO;
	HI545_FeatureControl_PERIOD_PixelNum=HI545_VIDEO_PERIOD_PIXEL_NUMS+ HI545.DummyPixels;
	HI545_FeatureControl_PERIOD_LineNum=HI545_VIDEO_PERIOD_LINE_NUMS+HI545.DummyLines;
	spin_unlock(&HI545mipiraw_drv_lock);


	spin_lock(&HI545mipiraw_drv_lock);
	HI545.imgMirror = sensor_config_data->SensorImageMirror;
	//HI545.imgMirror =IMAGE_NORMAL; //by module layout
	spin_unlock(&HI545mipiraw_drv_lock);
	//HI545SetFlipMirror(sensor_config_data->SensorImageMirror);
	HI545SetFlipMirror(IMAGE_H_MIRROR);

//    mDELAY(40);
	HI545DB("HI545Video exit:\n");
    return ERROR_NONE;
}


UINT32 HI545Capture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                                                MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{

 	kal_uint32 shutter = HI545.shutter;
	kal_uint32 temp_data;
	//kal_uint32 pv_line_length , cap_line_length,

	if( SENSOR_MODE_CAPTURE== HI545.sensorMode)
	{
		HI545DB("HI545Capture BusrtShot!!!\n");
	}
	else
	{
		HI545DB("HI545Capture enter:\n");
#if 0
		//Record Preview shutter & gain
		shutter=HI545_read_shutter();
		temp_data =  read_HI545_gain();
		spin_lock(&HI545mipiraw_drv_lock);
		HI545.pvShutter =shutter;
		HI545.sensorGlobalGain = temp_data;
		HI545.pvGain =HI545.sensorGlobalGain;
		spin_unlock(&HI545mipiraw_drv_lock);

		HI545DB("[HI545Capture]HI545.shutter=%d, read_pv_shutter=%d, read_pv_gain = 0x%x\n",HI545.shutter, shutter,HI545.sensorGlobalGain);
#endif
		// Full size setting
		HI545CaptureSetting();
	//    mDELAY(20);

		spin_lock(&HI545mipiraw_drv_lock);
		HI545.sensorMode = SENSOR_MODE_CAPTURE;
		HI545.imgMirror = sensor_config_data->SensorImageMirror;
		//HI545.imgMirror =IMAGE_NORMAL; //by module layout
		HI545.DummyPixels = 0;//define dummy pixels and lines
		HI545.DummyLines = 0 ;
		HI545_FeatureControl_PERIOD_PixelNum = HI545_FULL_PERIOD_PIXEL_NUMS + HI545.DummyPixels;
		HI545_FeatureControl_PERIOD_LineNum = HI545_FULL_PERIOD_LINE_NUMS + HI545.DummyLines;
		spin_unlock(&HI545mipiraw_drv_lock);

		//HI545SetFlipMirror(sensor_config_data->SensorImageMirror);

		HI545SetFlipMirror(IMAGE_H_MIRROR);

		HI545DB("HI545Capture exit:\n");
	}

    return ERROR_NONE;
}	/* HI545Capture() */

UINT32 HI545GetResolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *pSensorResolution)
{

    HI545DB("HI545GetResolution!!\n");

	pSensorResolution->SensorPreviewWidth	= HI545_IMAGE_SENSOR_FULL_WIDTH;
    pSensorResolution->SensorPreviewHeight	= HI545_IMAGE_SENSOR_FULL_HEIGHT;
    pSensorResolution->SensorFullWidth		= HI545_IMAGE_SENSOR_FULL_WIDTH;
    pSensorResolution->SensorFullHeight		= HI545_IMAGE_SENSOR_FULL_HEIGHT;
    pSensorResolution->SensorVideoWidth		= HI545_IMAGE_SENSOR_FULL_WIDTH;
    pSensorResolution->SensorVideoHeight    = HI545_IMAGE_SENSOR_FULL_HEIGHT;
//    HI545DB("SensorPreviewWidth:  %d.\n", pSensorResolution->SensorPreviewWidth);
//    HI545DB("SensorPreviewHeight: %d.\n", pSensorResolution->SensorPreviewHeight);
//    HI545DB("SensorFullWidth:  %d.\n", pSensorResolution->SensorFullWidth);
//    HI545DB("SensorFullHeight: %d.\n", pSensorResolution->SensorFullHeight);
    return ERROR_NONE;
}   /* HI545GetResolution() */

UINT32 HI545GetInfo(MSDK_SCENARIO_ID_ENUM ScenarioId,
                                                MSDK_SENSOR_INFO_STRUCT *pSensorInfo,
                                                MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{

	pSensorInfo->SensorPreviewResolutionX= HI545_IMAGE_SENSOR_FULL_WIDTH;
	pSensorInfo->SensorPreviewResolutionY= HI545_IMAGE_SENSOR_FULL_HEIGHT;

	pSensorInfo->SensorFullResolutionX= HI545_IMAGE_SENSOR_FULL_WIDTH;
    pSensorInfo->SensorFullResolutionY= HI545_IMAGE_SENSOR_FULL_HEIGHT;

	spin_lock(&HI545mipiraw_drv_lock);
	HI545.imgMirror = pSensorConfigData->SensorImageMirror ;
	spin_unlock(&HI545mipiraw_drv_lock);

   	pSensorInfo->SensorOutputDataFormat= SENSOR_OUTPUT_FORMAT_RAW_Gb;//B,R
    pSensorInfo->SensorClockPolarity =SENSOR_CLOCK_POLARITY_LOW;
    pSensorInfo->SensorClockFallingPolarity=SENSOR_CLOCK_POLARITY_LOW;
    pSensorInfo->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
    pSensorInfo->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;

    pSensorInfo->SensroInterfaceType=SENSOR_INTERFACE_TYPE_MIPI;

    pSensorInfo->CaptureDelayFrame = 4;  // from 2 to 3 for test shutter linearity
    pSensorInfo->PreviewDelayFrame = 4;  // from 442 to 666
    pSensorInfo->VideoDelayFrame = 2;

    pSensorInfo->SensorDrivingCurrent = ISP_DRIVING_8MA;
    pSensorInfo->AEShutDelayFrame = 0;//0;		    /* The frame of setting shutter default 0 for TG int */
    pSensorInfo->AESensorGainDelayFrame = 0;//0;     /* The frame of setting sensor gain */
    pSensorInfo->AEISPGainDelayFrame = 2;

    switch (ScenarioId)
    {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
            pSensorInfo->SensorClockFreq=24;
            pSensorInfo->SensorClockRisingCount= 0;

            pSensorInfo->SensorGrabStartX = HI545_PV_X_START;
            pSensorInfo->SensorGrabStartY = HI545_PV_Y_START;

            pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_2_LANE;
            pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
	     	pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = MIPI_DELAY_COUNT;
	    	pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
            pSensorInfo->SensorPacketECCOrder = 1;
            break;
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
            pSensorInfo->SensorClockFreq=24;
            pSensorInfo->SensorClockRisingCount= 0;

            pSensorInfo->SensorGrabStartX = HI545_VIDEO_X_START;
            pSensorInfo->SensorGrabStartY = HI545_VIDEO_Y_START;

            pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_2_LANE;
            pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
	     	pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = MIPI_DELAY_COUNT;
	    	pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
            pSensorInfo->SensorPacketECCOrder = 1;
            break;
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
            pSensorInfo->SensorClockFreq=24;
            pSensorInfo->SensorClockRisingCount= 0;

            pSensorInfo->SensorGrabStartX = HI545_FULL_X_START;	//2*HI545_IMAGE_SENSOR_PV_STARTX;
            pSensorInfo->SensorGrabStartY = HI545_FULL_Y_START;	//2*HI545_IMAGE_SENSOR_PV_STARTY;

            pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_2_LANE;
            pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
            pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = MIPI_DELAY_COUNT;
            pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
            pSensorInfo->SensorPacketECCOrder = 1;
            break;
        default:
			pSensorInfo->SensorClockFreq=24;
            pSensorInfo->SensorClockRisingCount= 0;

            pSensorInfo->SensorGrabStartX = HI545_PV_X_START;
            pSensorInfo->SensorGrabStartY = HI545_PV_Y_START;

            pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_2_LANE;
            pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
	     	pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = MIPI_DELAY_COUNT;
	    	pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
            pSensorInfo->SensorPacketECCOrder = 1;
            break;
    }

    memcpy(pSensorConfigData, &HI545SensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));

    return ERROR_NONE;
}   /* HI545GetInfo() */


UINT32 HI545Control(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *pImageWindow,
                                                MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
		spin_lock(&HI545mipiraw_drv_lock);
		HI545CurrentScenarioId = ScenarioId;
		spin_unlock(&HI545mipiraw_drv_lock);
		//HI545DB("ScenarioId=%d\n",ScenarioId);
		HI545DB("HI545CurrentScenarioId=%d\n",HI545CurrentScenarioId);

	switch (ScenarioId)
    {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
            HI545Preview(pImageWindow, pSensorConfigData);
            break;
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			HI545Video(pImageWindow, pSensorConfigData);
			break;
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
            HI545Capture(pImageWindow, pSensorConfigData);
            break;

        default:
            return ERROR_INVALID_SCENARIO_ID;

    }
    return ERROR_NONE;
} /* HI545Control() */


UINT32 HI545SetVideoMode(UINT16 u2FrameRate)
{

    kal_uint32 MIN_Frame_length =0,frameRate=0,extralines=0;
    HI545DB("[HI545SetVideoMode] frame rate = %d\n", u2FrameRate);

	spin_lock(&HI545mipiraw_drv_lock);
	VIDEO_MODE_TARGET_FPS=u2FrameRate;
	spin_unlock(&HI545mipiraw_drv_lock);

	if(u2FrameRate==0)
	{
		HI545DB("Disable Video Mode or dynimac fps\n");
		return KAL_TRUE;
	}
	if(u2FrameRate >30 || u2FrameRate <5)
	    HI545DB("error frame rate seting\n");

    if(HI545.sensorMode == SENSOR_MODE_VIDEO)//video ScenarioId recording
    {
    	if(HI545.HI545AutoFlickerMode == KAL_TRUE)
    	{
    		if (u2FrameRate==30)
				frameRate= 304;
			else if(u2FrameRate==15)
				frameRate= 148;//148;
			else
				frameRate=u2FrameRate*10;

			MIN_Frame_length = (HI545.videoPclk*10000)/(HI545_VIDEO_PERIOD_PIXEL_NUMS + HI545.DummyPixels)/frameRate*10;
    	}
		else
		{
    		if (u2FrameRate==30)
				MIN_Frame_length= HI545_VIDEO_PERIOD_LINE_NUMS;
			else	
				MIN_Frame_length = (HI545.videoPclk*10000) /(HI545_VIDEO_PERIOD_PIXEL_NUMS + HI545.DummyPixels)/u2FrameRate;
		}

		if((MIN_Frame_length <=HI545_VIDEO_PERIOD_LINE_NUMS))
		{
			MIN_Frame_length = HI545_VIDEO_PERIOD_LINE_NUMS;
			HI545DB("[HI545SetVideoMode]current fps = %d\n", (HI545.videoPclk*10000)  /(HI545_VIDEO_PERIOD_PIXEL_NUMS)/HI545_VIDEO_PERIOD_LINE_NUMS);
		}
		HI545DB("[HI545SetVideoMode]current fps (10 base)= %d\n", (HI545.videoPclk*10000)*10/(HI545_VIDEO_PERIOD_PIXEL_NUMS + HI545.DummyPixels)/MIN_Frame_length);

		if(HI545.shutter + 4 > MIN_Frame_length)
				MIN_Frame_length = HI545.shutter + 4;

		extralines = MIN_Frame_length - HI545_VIDEO_PERIOD_LINE_NUMS;
		
		spin_lock(&HI545mipiraw_drv_lock);
		HI545.DummyPixels = 0;//define dummy pixels and lines
		HI545.DummyLines = extralines ;
		spin_unlock(&HI545mipiraw_drv_lock);
		
		HI545_SetDummy(HI545.DummyPixels,extralines);
    }
	else if(HI545.sensorMode == SENSOR_MODE_CAPTURE)
	{
		HI545DB("-------[HI545SetVideoMode]ZSD???---------\n");
		if(HI545.HI545AutoFlickerMode == KAL_TRUE)
    	{
    		if (u2FrameRate==15)
			    frameRate= 148;
			else
				frameRate=u2FrameRate*10;

			MIN_Frame_length = (HI545.capPclk*10000) /(HI545_FULL_PERIOD_PIXEL_NUMS + HI545.DummyPixels)/frameRate*10;
    	}
		else
			MIN_Frame_length = (HI545.capPclk*10000) /(HI545_FULL_PERIOD_PIXEL_NUMS + HI545.DummyPixels)/u2FrameRate;

		if((MIN_Frame_length <=HI545_FULL_PERIOD_LINE_NUMS))
		{
			MIN_Frame_length = HI545_FULL_PERIOD_LINE_NUMS;
			HI545DB("[HI545SetVideoMode]current fps = %d\n", (HI545.capPclk*10000) /(HI545_FULL_PERIOD_PIXEL_NUMS)/HI545_FULL_PERIOD_LINE_NUMS);

		}
		HI545DB("[HI545SetVideoMode]current fps (10 base)= %d\n", (HI545.capPclk*10000)*10/(HI545_FULL_PERIOD_PIXEL_NUMS + HI545.DummyPixels)/MIN_Frame_length);

		if(HI545.shutter + 4 > MIN_Frame_length)
				MIN_Frame_length = HI545.shutter + 4;


		extralines = MIN_Frame_length - HI545_FULL_PERIOD_LINE_NUMS;

		spin_lock(&HI545mipiraw_drv_lock);
		HI545.DummyPixels = 0;//define dummy pixels and lines
		HI545.DummyLines = extralines ;
		spin_unlock(&HI545mipiraw_drv_lock);

		HI545_SetDummy(HI545.DummyPixels,extralines);
	}
	HI545DB("[HI545SetVideoMode]MIN_Frame_length=%d,HI545.DummyLines=%d\n",MIN_Frame_length,HI545.DummyLines);

    return KAL_TRUE;
}

UINT32 HI545SetAutoFlickerMode(kal_bool bEnable, UINT16 u2FrameRate)
{
	//return ERROR_NONE;

    //HI545DB("[HI545SetAutoFlickerMode] frame rate(10base) = %d %d\n", bEnable, u2FrameRate);
	if(bEnable) {   // enable auto flicker
		spin_lock(&HI545mipiraw_drv_lock);
		HI545.HI545AutoFlickerMode = KAL_TRUE;
		spin_unlock(&HI545mipiraw_drv_lock);
    } else {
    	spin_lock(&HI545mipiraw_drv_lock);
        HI545.HI545AutoFlickerMode = KAL_FALSE;
		spin_unlock(&HI545mipiraw_drv_lock);
        HI545DB("Disable Auto flicker\n");
    }

    return ERROR_NONE;
}

UINT32 HI545SetTestPatternMode(kal_bool bEnable)
{
    HI545DB("[HI545SetTestPatternMode] Test pattern enable:%d\n", bEnable);
#if 1
    if(bEnable) 
    {
        HI545_write_cmos_sensor(0x020a,0x0200);        
    }
    else
    {
        HI545_write_cmos_sensor(0x020a,0x0000);  

    }
#endif
    return ERROR_NONE;
}

UINT32 HI545MIPISetMaxFramerateByScenario(MSDK_SCENARIO_ID_ENUM scenarioId, MUINT32 frameRate) {
	kal_uint32 pclk;
	kal_int16 dummyLine;
	kal_uint16 lineLength,frameHeight;
		
	HI545DB("HI545MIPISetMaxFramerateByScenario: scenarioId = %d, frame rate = %d\n",scenarioId,frameRate);
	switch (scenarioId) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			pclk =  HI545_PV_CLK;
			lineLength = HI545_PV_PERIOD_PIXEL_NUMS;
			frameHeight = (10 * pclk)/frameRate/lineLength;
			dummyLine = frameHeight - HI545_PV_PERIOD_LINE_NUMS;
			if(dummyLine<0)
				dummyLine = 0;
			spin_lock(&HI545mipiraw_drv_lock);
			HI545.sensorMode = SENSOR_MODE_PREVIEW;
			spin_unlock(&HI545mipiraw_drv_lock);
			HI545_SetDummy(0, dummyLine);			
			break;			
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			pclk =  HI545_VIDEO_CLK;
			lineLength = HI545_VIDEO_PERIOD_PIXEL_NUMS;
			frameHeight = (10 * pclk)/frameRate/lineLength;
			dummyLine = frameHeight - HI545_VIDEO_PERIOD_LINE_NUMS;
			if(dummyLine<0)
				dummyLine = 0;
			spin_lock(&HI545mipiraw_drv_lock);
			HI545.sensorMode = SENSOR_MODE_VIDEO;
			spin_unlock(&HI545mipiraw_drv_lock);
			HI545_SetDummy(0, dummyLine);			
			break;			
			 break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_CAMERA_ZSD:			
			pclk = HI545_CAP_CLK;
			lineLength = HI545_FULL_PERIOD_PIXEL_NUMS;
			frameHeight = (10 * pclk)/frameRate/lineLength;
			dummyLine = frameHeight - HI545_FULL_PERIOD_LINE_NUMS;
			if(dummyLine<0)
				dummyLine = 0;
			spin_lock(&HI545mipiraw_drv_lock);
			HI545.sensorMode = SENSOR_MODE_CAPTURE;
			spin_unlock(&HI545mipiraw_drv_lock);
			HI545_SetDummy(0, dummyLine);			
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
	return ERROR_NONE;
}


UINT32 HI545MIPIGetDefaultFramerateByScenario(MSDK_SCENARIO_ID_ENUM scenarioId, MUINT32 *pframeRate) 
{

	switch (scenarioId) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			 *pframeRate = 300;
			 break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
			 *pframeRate = 240;  // modify by yfx for zsd cc
			break;		
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



UINT32 HI545FeatureControl(MSDK_SENSOR_FEATURE_ENUM FeatureId,
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
            *pFeatureReturnPara16++= HI545_IMAGE_SENSOR_FULL_WIDTH;
            *pFeatureReturnPara16= HI545_IMAGE_SENSOR_FULL_HEIGHT;
            *pFeatureParaLen=4;
            break;
        case SENSOR_FEATURE_GET_PERIOD:
				*pFeatureReturnPara16++= HI545_FeatureControl_PERIOD_PixelNum;
				*pFeatureReturnPara16= HI545_FeatureControl_PERIOD_LineNum;
				*pFeatureParaLen=4;
				break;
        case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
			switch(HI545CurrentScenarioId)
			{
				case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
					*pFeatureReturnPara32 =  HI545_PV_CLK;
					*pFeatureParaLen=4;
					break;
				case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
					*pFeatureReturnPara32 =  HI545_VIDEO_CLK;
					*pFeatureParaLen=4;
					break;
				case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
				case MSDK_SCENARIO_ID_CAMERA_ZSD:
					*pFeatureReturnPara32 = HI545_CAP_CLK;
					*pFeatureParaLen=4;
					break;
				default:
					*pFeatureReturnPara32 =  HI545_CAP_CLK;
					*pFeatureParaLen=4;
					break;
			}
		    break;

        case SENSOR_FEATURE_SET_ESHUTTER:
            HI545_SetShutter(*pFeatureData16);
            break;
        case SENSOR_FEATURE_SET_NIGHTMODE:
            HI545_NightMode((BOOL) *pFeatureData16);
            break;
        case SENSOR_FEATURE_SET_GAIN:
            HI545_SetGain((UINT16) *pFeatureData16);
            break;
        case SENSOR_FEATURE_SET_FLASHLIGHT:
            break;
        case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
            //HI545_isp_master_clock=*pFeatureData32;
            break;
        case SENSOR_FEATURE_SET_REGISTER:
            HI545_write_cmos_sensor(pSensorRegData->RegAddr, pSensorRegData->RegData);
            break;
        case SENSOR_FEATURE_GET_REGISTER:
            pSensorRegData->RegData = HI545_read_cmos_sensor(pSensorRegData->RegAddr);
            break;
        case SENSOR_FEATURE_SET_CCT_REGISTER:
            SensorRegNumber=FACTORY_END_ADDR;
            for (i=0;i<SensorRegNumber;i++)
            {
            	spin_lock(&HI545mipiraw_drv_lock);
                HI545SensorCCT[i].Addr=*pFeatureData32++;
                HI545SensorCCT[i].Para=*pFeatureData32++;
				spin_unlock(&HI545mipiraw_drv_lock);
            }
            break;
        case SENSOR_FEATURE_GET_CCT_REGISTER:
            SensorRegNumber=FACTORY_END_ADDR;
            if (*pFeatureParaLen<(SensorRegNumber*sizeof(SENSOR_REG_STRUCT)+4))
                return FALSE;
            *pFeatureData32++=SensorRegNumber;
            for (i=0;i<SensorRegNumber;i++)
            {
                *pFeatureData32++=HI545SensorCCT[i].Addr;
                *pFeatureData32++=HI545SensorCCT[i].Para;
            }
            break;
        case SENSOR_FEATURE_SET_ENG_REGISTER:
            SensorRegNumber=ENGINEER_END;
            for (i=0;i<SensorRegNumber;i++)
            {
            	spin_lock(&HI545mipiraw_drv_lock);
                HI545SensorReg[i].Addr=*pFeatureData32++;
                HI545SensorReg[i].Para=*pFeatureData32++;
				spin_unlock(&HI545mipiraw_drv_lock);
            }
            break;
        case SENSOR_FEATURE_GET_ENG_REGISTER:
            SensorRegNumber=ENGINEER_END;
            if (*pFeatureParaLen<(SensorRegNumber*sizeof(SENSOR_REG_STRUCT)+4))
                return FALSE;
            *pFeatureData32++=SensorRegNumber;
            for (i=0;i<SensorRegNumber;i++)
            {
                *pFeatureData32++=HI545SensorReg[i].Addr;
                *pFeatureData32++=HI545SensorReg[i].Para;
            }
            break;
        case SENSOR_FEATURE_GET_REGISTER_DEFAULT:
            if (*pFeatureParaLen>=sizeof(NVRAM_SENSOR_DATA_STRUCT))
            {
                pSensorDefaultData->Version=NVRAM_CAMERA_SENSOR_FILE_VERSION;
                pSensorDefaultData->SensorId=HI545MIPI_SENSOR_ID;
                memcpy(pSensorDefaultData->SensorEngReg, HI545SensorReg, sizeof(SENSOR_REG_STRUCT)*ENGINEER_END);
                memcpy(pSensorDefaultData->SensorCCTReg, HI545SensorCCT, sizeof(SENSOR_REG_STRUCT)*FACTORY_END_ADDR);
            }
            else
                return FALSE;
            *pFeatureParaLen=sizeof(NVRAM_SENSOR_DATA_STRUCT);
            break;
        case SENSOR_FEATURE_GET_CONFIG_PARA:
            memcpy(pSensorConfigData, &HI545SensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
            *pFeatureParaLen=sizeof(MSDK_SENSOR_CONFIG_STRUCT);
            break;
        case SENSOR_FEATURE_CAMERA_PARA_TO_SENSOR:
            HI545_camera_para_to_sensor();
            break;

        case SENSOR_FEATURE_SENSOR_TO_CAMERA_PARA:
            HI545_sensor_to_camera_para();
            break;
        case SENSOR_FEATURE_GET_GROUP_COUNT:
            *pFeatureReturnPara32++=HI545_get_sensor_group_count();
            *pFeatureParaLen=4;
            break;
        case SENSOR_FEATURE_GET_GROUP_INFO:
            HI545_get_sensor_group_info(pSensorGroupInfo->GroupIdx, pSensorGroupInfo->GroupNamePtr, &pSensorGroupInfo->ItemCount);
            *pFeatureParaLen=sizeof(MSDK_SENSOR_GROUP_INFO_STRUCT);
            break;
        case SENSOR_FEATURE_GET_ITEM_INFO:
            HI545_get_sensor_item_info(pSensorItemInfo->GroupIdx,pSensorItemInfo->ItemIdx, pSensorItemInfo);
            *pFeatureParaLen=sizeof(MSDK_SENSOR_ITEM_INFO_STRUCT);
            break;

        case SENSOR_FEATURE_SET_ITEM_INFO:
            HI545_set_sensor_item_info(pSensorItemInfo->GroupIdx, pSensorItemInfo->ItemIdx, pSensorItemInfo->ItemValue);
            *pFeatureParaLen=sizeof(MSDK_SENSOR_ITEM_INFO_STRUCT);
            break;

        case SENSOR_FEATURE_GET_ENG_INFO:
            pSensorEngInfo->SensorId = 129;
            pSensorEngInfo->SensorType = CMOS_SENSOR;
            pSensorEngInfo->SensorOutputDataFormat=SENSOR_OUTPUT_FORMAT_RAW_B;
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
            HI545SetVideoMode(*pFeatureData16);
            break;
        case SENSOR_FEATURE_CHECK_SENSOR_ID:
            HI545GetSensorID(pFeatureReturnPara32);
            break;
        case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE:
            HI545SetAutoFlickerMode((BOOL)*pFeatureData16, *(pFeatureData16+1));
	        break;
        case SENSOR_FEATURE_SET_TEST_PATTERN:
            HI545SetTestPatternMode((BOOL)*pFeatureData16);
            break;
		case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
			HI545MIPISetMaxFramerateByScenario((MSDK_SCENARIO_ID_ENUM)*pFeatureData32, *(pFeatureData32+1));
			break;
		case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
			HI545MIPIGetDefaultFramerateByScenario((MSDK_SCENARIO_ID_ENUM)*pFeatureData32, (MUINT32 *)(*(pFeatureData32+1)));
            break;       
        case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE://for factory mode auto testing             
            *pFeatureReturnPara32=HI545_TEST_PATTERN_CHECKSUM;           
            *pFeatureParaLen=4;                             
        break;     
        default:
            break;
    }
    return ERROR_NONE;
}	/* HI545FeatureControl() */


SENSOR_FUNCTION_STRUCT	SensorFuncHI545=
{
    HI545Open,
    HI545GetInfo,
    HI545GetResolution,
    HI545FeatureControl,
    HI545Control,
    HI545Close
};

UINT32 HI545_MIPI_RAW_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
    /* To Do : Check Sensor status here */
    if (pfFunc!=NULL)
        *pfFunc=&SensorFuncHI545;

    return ERROR_NONE;
}   /* SensorInit() */

