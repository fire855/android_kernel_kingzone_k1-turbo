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

#include "hi551mipiraw_Sensor.h"
#include "hi551mipiraw_Camera_Sensor_para.h"
#include "hi551mipiraw_CameraCustomized.h"
static DEFINE_SPINLOCK(HI551mipiraw_drv_lock);

#define HI551_TEST_PATTERN_CHECKSUM (0x9FC8832A)//do rotate will change this value

#define HI551_DEBUG
//#define HI551_DEBUG_SOFIA

#ifdef HI551_DEBUG
	#define HI551DB(fmt, arg...) xlog_printk(ANDROID_LOG_DEBUG, "[HI551Raw] ",  fmt, ##arg)
#else
	#define HI551DB(fmt, arg...)
#endif

#ifdef HI551_DEBUG_SOFIA
	#define HI551DBSOFIA(fmt, arg...) xlog_printk(ANDROID_LOG_DEBUG, "[HI551Raw] ",  fmt, ##arg)
#else
	#define HI551DBSOFIA(fmt, arg...)
#endif

#define mDELAY(ms)  mdelay(ms)

kal_uint32 HI551_FeatureControl_PERIOD_PixelNum=HI551_PV_PERIOD_PIXEL_NUMS;
kal_uint32 HI551_FeatureControl_PERIOD_LineNum=HI551_PV_PERIOD_LINE_NUMS;

UINT32 HI551GetSensorID(UINT32 *sensorID);

UINT16 VIDEO_MODE_TARGET_FPS = 30;
static BOOL ReEnteyCamera = KAL_FALSE;


MSDK_SENSOR_CONFIG_STRUCT HI551SensorConfigData;

kal_uint32 HI551_FAC_SENSOR_REG;

MSDK_SCENARIO_ID_ENUM HI551CurrentScenarioId = MSDK_SCENARIO_ID_CAMERA_PREVIEW;

/* FIXME: old factors and DIDNOT use now. s*/
SENSOR_REG_STRUCT HI551SensorCCT[]=CAMERA_SENSOR_CCT_DEFAULT_VALUE;
SENSOR_REG_STRUCT HI551SensorReg[ENGINEER_END]=CAMERA_SENSOR_REG_DEFAULT_VALUE;
/* FIXME: old factors and DIDNOT use now. e*/

static HI551_PARA_STRUCT HI551;

extern int iReadReg(u16 a_u2Addr , u8 * a_puBuff , u16 i2cId);
extern int iWriteReg(u16 a_u2Addr , u32 a_u4Data , u32 a_u4Bytes , u16 i2cId);

//#define HI551_write_cmos_sensor(addr, para) iWriteReg((u16) addr , (u32) para , 1, HI551MIPI_WRITE_ID)
// modify by yfx
//extern int iMultiWriteReg(u8 *pData, u16 lens);
#if 1
#define HI551_write_cmos_sensor(addr, para) iWriteReg((u16) addr , (u32) para , 2, HI551MIPI_WRITE_ID)
#else
void HI551_write_cmos_sensor(u16 addr, u16 para) 
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

kal_uint16 HI551_read_cmos_sensor(kal_uint32 addr)
{
kal_uint16 get_byte=0;
    iReadReg((u16) addr ,(u8*)&get_byte,HI551MIPI_WRITE_ID);
    return get_byte;
}

#define Sleep(ms) mdelay(ms)

void HI551_write_shutter(kal_uint32 shutter)
{
	kal_uint32 min_framelength = HI551_PV_PERIOD_PIXEL_NUMS, max_shutter=0;
	kal_uint32 extra_lines = 0;
	kal_uint32 line_length = 0;
	kal_uint32 frame_length = 0;
	unsigned long flags;

	HI551DBSOFIA("!!shutter=%d!!!!!\n", shutter);

	if(HI551.HI551AutoFlickerMode == KAL_TRUE)
	{

		if ( SENSOR_MODE_PREVIEW == HI551.sensorMode )  //(g_iHI551_Mode == HI551_MODE_PREVIEW)	//SXGA size output
		{
			line_length = HI551_PV_PERIOD_PIXEL_NUMS + HI551.DummyPixels;
			max_shutter = HI551_PV_PERIOD_LINE_NUMS + HI551.DummyLines ;
		}
		else if( SENSOR_MODE_VIDEO == HI551.sensorMode ) //add for video_6M setting
		{
			line_length = HI551_VIDEO_PERIOD_PIXEL_NUMS + HI551.DummyPixels;
			max_shutter = HI551_VIDEO_PERIOD_LINE_NUMS + HI551.DummyLines ;
		}
		else
		{
			line_length = HI551_FULL_PERIOD_PIXEL_NUMS + HI551.DummyPixels;
			max_shutter = HI551_FULL_PERIOD_LINE_NUMS + HI551.DummyLines ;
		}

		switch(HI551CurrentScenarioId)
		{
        	case MSDK_SCENARIO_ID_CAMERA_ZSD:
			case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
				HI551DBSOFIA("AutoFlickerMode!!! MSDK_SCENARIO_ID_CAMERA_ZSD  0!!\n");
				min_framelength = max_shutter;// capture max_fps 24,no need calculate
				break;
			case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
				if(VIDEO_MODE_TARGET_FPS==30)
				{
					min_framelength = (HI551.videoPclk*10000) /(HI551_VIDEO_PERIOD_PIXEL_NUMS + HI551.DummyPixels)/304*10 ;
				}
				else if(VIDEO_MODE_TARGET_FPS==15)
				{
					min_framelength = (HI551.videoPclk*10000) /(HI551_VIDEO_PERIOD_PIXEL_NUMS + HI551.DummyPixels)/148*10 ;
				}
				else
				{
					min_framelength = max_shutter;
				}
				break;
			default:
				min_framelength = (HI551.pvPclk*10000) /(HI551_PV_PERIOD_PIXEL_NUMS + HI551.DummyPixels)/296*10 ;
    			break;
		}

		HI551DBSOFIA("AutoFlickerMode!!! min_framelength for AutoFlickerMode = %d (0x%x)\n",min_framelength,min_framelength);
		HI551DBSOFIA("max framerate(10 base) autofilker = %d\n",(HI551.pvPclk*10000)*10 /line_length/min_framelength);

		if (shutter < 4)
			shutter = 4;

		if (shutter > (max_shutter-4) )
			extra_lines = shutter - max_shutter + 4;
		else
			extra_lines = 0;

		if ( SENSOR_MODE_PREVIEW == HI551.sensorMode )	//SXGA size output
		{
			frame_length = HI551_PV_PERIOD_LINE_NUMS+ HI551.DummyLines + extra_lines ;
		}
		else if(SENSOR_MODE_VIDEO == HI551.sensorMode)
		{
			frame_length = HI551_VIDEO_PERIOD_LINE_NUMS+ HI551.DummyLines + extra_lines ;
		}
		else				//QSXGA size output
		{
			frame_length = HI551_FULL_PERIOD_LINE_NUMS + HI551.DummyLines + extra_lines ;
		}
		HI551DBSOFIA("frame_length 0= %d\n",frame_length);

		if (frame_length < min_framelength)
		{
			//shutter = min_framelength - 4;

			switch(HI551CurrentScenarioId)
			{
        	case MSDK_SCENARIO_ID_CAMERA_ZSD:
			case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
				extra_lines = min_framelength- (HI551_FULL_PERIOD_LINE_NUMS+ HI551.DummyLines);
				break;
			case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
				extra_lines = min_framelength- (HI551_VIDEO_PERIOD_LINE_NUMS+ HI551.DummyLines);
				break;
			default:
				extra_lines = min_framelength- (HI551_PV_PERIOD_LINE_NUMS+ HI551.DummyLines);
    			break;
			}
			frame_length = min_framelength;
		}

		HI551DBSOFIA("frame_length 1= %d\n",frame_length);

		ASSERT(line_length < HI551_MAX_LINE_LENGTH);		//0xCCCC
		ASSERT(frame_length < HI551_MAX_FRAME_LENGTH); 	//0xFFFF
		
		spin_lock_irqsave(&HI551mipiraw_drv_lock,flags);
		HI551.maxExposureLines = frame_length - 4;
		HI551_FeatureControl_PERIOD_PixelNum = line_length;
		HI551_FeatureControl_PERIOD_LineNum = frame_length;
		spin_unlock_irqrestore(&HI551mipiraw_drv_lock,flags);


		HI551_write_cmos_sensor(0x0046, 0x0100);
		//Set total frame length
		HI551_write_cmos_sensor(0x0006, frame_length);
//		HI551_write_cmos_sensor(0x0007, frame_length & 0xFF);

		//Set shutter (Coarse integration time, uint: lines.)
		HI551_write_cmos_sensor(0x0004, shutter);
		//HI551_write_cmos_sensor(0x0005, (shutter) & 0xFF);
		
		HI551_write_cmos_sensor(0x0046, 0x0000);

		HI551DBSOFIA("frame_length 2= %d\n",frame_length);
		HI551DB("framerate(10 base) = %d\n",(HI551.pvPclk*10000)*10 /line_length/frame_length);

		HI551DB("shutter=%d, extra_lines=%d, line_length=%d, frame_length=%d\n", shutter, extra_lines, line_length, frame_length);

	}
	else
	{
		if ( SENSOR_MODE_PREVIEW == HI551.sensorMode )  //(g_iHI551_Mode == HI551_MODE_PREVIEW)	//SXGA size output
		{
			max_shutter = HI551_PV_PERIOD_LINE_NUMS + HI551.DummyLines ;
		}
		else if( SENSOR_MODE_VIDEO == HI551.sensorMode ) //add for video_6M setting
		{
			max_shutter = HI551_VIDEO_PERIOD_LINE_NUMS + HI551.DummyLines ;
		}
		else
		{
			max_shutter = HI551_FULL_PERIOD_LINE_NUMS + HI551.DummyLines ;
		}

		if (shutter < 4)
			shutter = 4;

		if (shutter > (max_shutter-4) )
			extra_lines = shutter - max_shutter + 4;
		else
			extra_lines = 0;

		if ( SENSOR_MODE_PREVIEW == HI551.sensorMode )	//SXGA size output
		{
			line_length = HI551_PV_PERIOD_PIXEL_NUMS + HI551.DummyPixels;
			frame_length = HI551_PV_PERIOD_LINE_NUMS+ HI551.DummyLines + extra_lines ;
		}
		else if( SENSOR_MODE_VIDEO == HI551.sensorMode )
		{
			line_length = HI551_VIDEO_PERIOD_PIXEL_NUMS + HI551.DummyPixels;
			frame_length = HI551_VIDEO_PERIOD_LINE_NUMS + HI551.DummyLines + extra_lines ;
		}
		else				//QSXGA size output
		{
			line_length = HI551_FULL_PERIOD_PIXEL_NUMS + HI551.DummyPixels;
			frame_length = HI551_FULL_PERIOD_LINE_NUMS + HI551.DummyLines + extra_lines ;
		}

		ASSERT(line_length < HI551_MAX_LINE_LENGTH);		//0xCCCC
		ASSERT(frame_length < HI551_MAX_FRAME_LENGTH); 	//0xFFFF

		//Set total frame length
		HI551_write_cmos_sensor(0x0046, 0x0100);		
		HI551_write_cmos_sensor(0x0006, frame_length);
		//HI551_write_cmos_sensor(0x0007, frame_length & 0xFF);
		HI551_write_cmos_sensor(0x0046, 0x0000);		

		spin_lock_irqsave(&HI551mipiraw_drv_lock,flags);
		HI551.maxExposureLines = frame_length -4;
		HI551_FeatureControl_PERIOD_PixelNum = line_length;
		HI551_FeatureControl_PERIOD_LineNum = frame_length;
		spin_unlock_irqrestore(&HI551mipiraw_drv_lock,flags);


		//Set shutter (Coarse integration time, uint: lines.)
		HI551_write_cmos_sensor(0x0046, 0x0100);		
		HI551_write_cmos_sensor(0x0004, shutter);
		//HI551_write_cmos_sensor(0x0005, (shutter) & 0xFF);
		HI551_write_cmos_sensor(0x0046, 0x0000);		
		
		//HI551DB("framerate(10 base) = %d\n",(HI551.pvPclk*10000)*10 /line_length/frame_length);
		HI551DB("shutter=%d, extra_lines=%d, line_length=%d, frame_length=%d\n", shutter, extra_lines, line_length, frame_length);
	}

}   /* write_HI551_shutter */

/*******************************************************************************
*
********************************************************************************/
static kal_uint16 HI551Reg2Gain(const kal_uint8 iReg)
{
	kal_uint16 iGain = 64;

	iGain = 4 * iReg + 64;

	return iGain;
}

/*******************************************************************************
*
********************************************************************************/
static kal_uint16 HI551Gain2Reg(const kal_uint16 Gain)
{
	kal_uint16 iReg;
	kal_uint8 iBaseGain = 64;
	
	iReg = Gain / 4 - 16;
    return iReg;//HI551. sensorGlobalGain

}


void write_HI551_gain(kal_uint8 gain)
{
#if 1
	HI551_write_cmos_sensor(0x003a, (gain << 8));
#endif
	return;
}

/*************************************************************************
* FUNCTION
*    HI551_SetGain
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
void HI551_SetGain(UINT16 iGain)
{
	unsigned long flags;
    kal_uint16 HI551GlobalGain=0;
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
		
	HI551GlobalGain = HI551Gain2Reg(iGain); 
	spin_lock(&HI551mipiraw_drv_lock);
	HI551.realGain = iGain;
	HI551.sensorGlobalGain =HI551GlobalGain;
	spin_unlock(&HI551mipiraw_drv_lock);

	HI551DB("[HI551_SetGain]HI551.sensorGlobalGain=0x%x,HI551.realGain=%d\n",HI551.sensorGlobalGain,HI551.realGain);

	HI551_write_cmos_sensor(0x0046, 0x0100);
	write_HI551_gain(HI551.sensorGlobalGain);	
	HI551_write_cmos_sensor(0x0046, 0x0000);

	
	return;	
}   /*  HI551_SetGain_SetGain  */


/*************************************************************************
* FUNCTION
*    read_HI551_gain
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
kal_uint16 read_HI551_gain(void)
{
	kal_uint16 read_gain_anolog=0;
	kal_uint16 HI551RealGain_anolog =0;
	kal_uint16 HI551RealGain =0;

	read_gain_anolog=HI551_read_cmos_sensor(0x003a);
	
	HI551RealGain_anolog = HI551Reg2Gain(read_gain_anolog);
	

	spin_lock(&HI551mipiraw_drv_lock);
	HI551.sensorGlobalGain = read_gain_anolog;
	HI551.realGain = HI551RealGain;
	spin_unlock(&HI551mipiraw_drv_lock);
	HI551DB("[read_HI551_gain]HI551RealGain_anolog=0x%x\n",HI551RealGain_anolog);

	return HI551.sensorGlobalGain;
}  /* read_HI551_gain */


void HI551_camera_para_to_sensor(void)
{
    kal_uint32    i;
    for(i=0; 0xFFFFFFFF!=HI551SensorReg[i].Addr; i++)
    {
        HI551_write_cmos_sensor(HI551SensorReg[i].Addr, HI551SensorReg[i].Para);
    }
    for(i=ENGINEER_START_ADDR; 0xFFFFFFFF!=HI551SensorReg[i].Addr; i++)
    {
        HI551_write_cmos_sensor(HI551SensorReg[i].Addr, HI551SensorReg[i].Para);
    }
    for(i=FACTORY_START_ADDR; i<FACTORY_END_ADDR; i++)
    {
        HI551_write_cmos_sensor(HI551SensorCCT[i].Addr, HI551SensorCCT[i].Para);
    }
}


/*************************************************************************
* FUNCTION
*    HI551_sensor_to_camera_para
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
void HI551_sensor_to_camera_para(void)
{
    kal_uint32    i, temp_data;
    for(i=0; 0xFFFFFFFF!=HI551SensorReg[i].Addr; i++)
    {
         temp_data = HI551_read_cmos_sensor(HI551SensorReg[i].Addr);
		 spin_lock(&HI551mipiraw_drv_lock);
		 HI551SensorReg[i].Para =temp_data;
		 spin_unlock(&HI551mipiraw_drv_lock);
    }
    for(i=ENGINEER_START_ADDR; 0xFFFFFFFF!=HI551SensorReg[i].Addr; i++)
    {
        temp_data = HI551_read_cmos_sensor(HI551SensorReg[i].Addr);
		spin_lock(&HI551mipiraw_drv_lock);
		HI551SensorReg[i].Para = temp_data;
		spin_unlock(&HI551mipiraw_drv_lock);
    }
}

/*************************************************************************
* FUNCTION
*    HI551_get_sensor_group_count
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
kal_int32  HI551_get_sensor_group_count(void)
{
    return GROUP_TOTAL_NUMS;
}

void HI551_get_sensor_group_info(kal_uint16 group_idx, kal_int8* group_name_ptr, kal_int32* item_count_ptr)
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

void HI551_get_sensor_item_info(kal_uint16 group_idx,kal_uint16 item_idx, MSDK_SENSOR_ITEM_INFO_STRUCT* info_ptr)
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

            temp_para= HI551SensorCCT[temp_addr].Para;
			//temp_gain= (temp_para/HI551.sensorBaseGain) * 1000;

            info_ptr->ItemValue=temp_gain;
            info_ptr->IsTrueFalse=KAL_FALSE;
            info_ptr->IsReadOnly=KAL_FALSE;
            info_ptr->IsNeedRestart=KAL_FALSE;
            info_ptr->Min= HI551_MIN_ANALOG_GAIN * 1000;
            info_ptr->Max= HI551_MAX_ANALOG_GAIN * 1000;
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



kal_bool HI551_set_sensor_item_info(kal_uint16 group_idx, kal_uint16 item_idx, kal_int32 ItemValue)
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
//             temp_para=(temp_gain * HI551.sensorBaseGain + BASEGAIN/2)/BASEGAIN;
          }
          else
			  ASSERT(0);

			 HI551DBSOFIA("HI551????????????????????? :\n ");
		  spin_lock(&HI551mipiraw_drv_lock);
          HI551SensorCCT[temp_addr].Para = temp_para;
		  spin_unlock(&HI551mipiraw_drv_lock);
          HI551_write_cmos_sensor(HI551SensorCCT[temp_addr].Addr,temp_para);

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
					spin_lock(&HI551mipiraw_drv_lock);
                    HI551_FAC_SENSOR_REG=ItemValue;
					spin_unlock(&HI551mipiraw_drv_lock);
                    break;
                case 1:
                    HI551_write_cmos_sensor(HI551_FAC_SENSOR_REG,ItemValue);
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

static void HI551_SetDummy( const kal_uint32 iPixels, const kal_uint32 iLines )
{
	kal_uint32 line_length = 0;
	kal_uint32 frame_length = 0;

	if ( SENSOR_MODE_PREVIEW == HI551.sensorMode )	//SXGA size output
	{
		line_length = HI551_PV_PERIOD_PIXEL_NUMS + iPixels;
		frame_length = HI551_PV_PERIOD_LINE_NUMS + iLines;
	}
	else if( SENSOR_MODE_VIDEO== HI551.sensorMode )
	{
		line_length = HI551_VIDEO_PERIOD_PIXEL_NUMS + iPixels;
		frame_length = HI551_VIDEO_PERIOD_LINE_NUMS + iLines;
	}
	else//QSXGA size output
	{
		line_length = HI551_FULL_PERIOD_PIXEL_NUMS + iPixels;
		frame_length = HI551_FULL_PERIOD_LINE_NUMS + iLines;
	}

	//if(HI551.maxExposureLines > frame_length -4 )
	//	return;

	//ASSERT(line_length < HI551_MAX_LINE_LENGTH);		//0xCCCC
	//ASSERT(frame_length < HI551_MAX_FRAME_LENGTH);	//0xFFFF

	//Set total frame length
	HI551_write_cmos_sensor(0x0006, frame_length);
	//HI551_write_cmos_sensor(0x0007, frame_length & 0xFF);

	spin_lock(&HI551mipiraw_drv_lock);
	HI551.maxExposureLines = frame_length -4;
	HI551_FeatureControl_PERIOD_PixelNum = line_length;
	HI551_FeatureControl_PERIOD_LineNum = frame_length;
	spin_unlock(&HI551mipiraw_drv_lock);

	//Set total line length
	HI551_write_cmos_sensor(0x0008, line_length);
	//HI551_write_cmos_sensor(0x0009, line_length & 0xFF);

}   /*  HI551_SetDummy */

void HI551PreviewSetting(void)
	{	
#if 1
	HI551DB("HI551PreviewSetting_2lane_30fps:\n ");


	//////////////////////////////////////////////////////////////////////////
	//	Sensor			 : Hi-544
	//		Mode			 : Preview
	//		Size			 : 1296 * 972
	//////////////////////////////////////////////////////////////////////////

	HI551_write_cmos_sensor(0x0118, 0x0000); //sleep On

    HI551_write_cmos_sensor(0x0012, 0x00c0);       // 212
	HI551_write_cmos_sensor(0x0018, 0x0adf);		// 2771
	HI551_write_cmos_sensor(0x0026, 0x0030);		//72
	HI551_write_cmos_sensor(0x002c, 0x07c7);		//1991
	//Image size 1296x972
	HI551_write_cmos_sensor(0x0700, 0xA0A3);		// byrscl_ctrl1 / byrscl_fifo_read_delay
	HI551_write_cmos_sensor(0x0112, 0x0510);		// x_output_size_h / l 1296
	HI551_write_cmos_sensor(0x0114, 0x03CC);		// y_output_size_h / l 972
	//HI551_write_cmos_sensor(0x0B14, 0x300b);   
	HI551_write_cmos_sensor(0x0B16, 0x448F);       // pll_mipi_clk_div
	HI551_write_cmos_sensor(0x090A, 0x0384);		// mipi_vblank_delay_h / l
	HI551_write_cmos_sensor(0x090C, 0x000A);		// mipi_hblank_short_delay_h / l
	HI551_write_cmos_sensor(0x090E, 0x03E8);		// mipi_hblank_long_delay_h / l
	HI551_write_cmos_sensor(0x0910, 0x5D04);		// mipi_value_exit_seq / mipi_value_lpx
	HI551_write_cmos_sensor(0x0912, 0x030E);		// mipi_value_clk_prepare / mipi_value_clk_zeo
	HI551_write_cmos_sensor(0x0914, 0x0404);		// mipi_value_clk_pre / mipi_value_data_prepare
	HI551_write_cmos_sensor(0x0916, 0x0709);		// mipi_value_data_zero / mipi_value_data_trail
	HI551_write_cmos_sensor(0x0918, 0x0c05);		// mipi_value_clk_post / mipi_value_clk_trail
	HI551_write_cmos_sensor(0x091A, 0x0800);		// mipi_value_exit / null
    HI551_write_cmos_sensor(0x0118, 0x0100);
	//HI551_write_cmos_sensor(0x0A00, 0x0100); //sleep Off 

	mDELAY(200);

	ReEnteyCamera = KAL_FALSE;

	HI551DB("HI551PreviewSetting_2lane exit :\n ");
#endif
	}


void HI551VideoSetting(void)
{

	HI551DB("HI551VideoSetting:\n ");
	
	//////////////////////////////////////////////////////////////////////////
	//	Sensor			 : Hi-544
	//		Mode			 : video
	//		Size			 : 1296 * 972
	//////////////////////////////////////////////////////////////////////////
	
	//Image size 1296x972
	HI551_write_cmos_sensor(0x0118, 0x0000); //sleep On

    HI551_write_cmos_sensor(0x0012, 0x00c0);       // 212
	HI551_write_cmos_sensor(0x0018, 0x0adf);		// 2771
	HI551_write_cmos_sensor(0x0026, 0x0030);		//72
	HI551_write_cmos_sensor(0x002c, 0x07c7);		//1991
	//Image size 1296x972
	HI551_write_cmos_sensor(0x0700, 0xA0A3);		// byrscl_ctrl1 / byrscl_fifo_read_delay
	HI551_write_cmos_sensor(0x0112, 0x0510);		// x_output_size_h / l 1296
	HI551_write_cmos_sensor(0x0114, 0x03CC);		// y_output_size_h / l 972
	//HI551_write_cmos_sensor(0x0B14, 0x300b);   
	HI551_write_cmos_sensor(0x0B16, 0x448F);       // pll_mipi_clk_div
	HI551_write_cmos_sensor(0x090A, 0x0384);		// mipi_vblank_delay_h / l
	HI551_write_cmos_sensor(0x090C, 0x000A);		// mipi_hblank_short_delay_h / l
	HI551_write_cmos_sensor(0x090E, 0x03E8);		// mipi_hblank_long_delay_h / l
	HI551_write_cmos_sensor(0x0910, 0x5D04);		// mipi_value_exit_seq / mipi_value_lpx
	HI551_write_cmos_sensor(0x0912, 0x030E);		// mipi_value_clk_prepare / mipi_value_clk_zeo
	HI551_write_cmos_sensor(0x0914, 0x0404);		// mipi_value_clk_pre / mipi_value_data_prepare
	HI551_write_cmos_sensor(0x0916, 0x0709);		// mipi_value_data_zero / mipi_value_data_trail
	HI551_write_cmos_sensor(0x0918, 0x0c05);		// mipi_value_clk_post / mipi_value_clk_trail
	HI551_write_cmos_sensor(0x091A, 0x0800);		// mipi_value_exit / null
    HI551_write_cmos_sensor(0x0118, 0x0100);
	

	//mDELAY(200);
	
	mDELAY(50);

	ReEnteyCamera = KAL_FALSE;

	HI551DB("HI551VideoSetting_4:3 exit :\n ");
}


void HI551CaptureSetting(void)
{

    if(ReEnteyCamera == KAL_TRUE)
    {
		HI551DB("HI551CaptureSetting_2lane_SleepIn :\n ");
    }
	else
	{
		HI551DB("HI551CaptureSetting_2lane_streamOff :\n ");
	}

	HI551DB("HI551CaptureSetting_2lane_OB:\n ");
	
	//////////////////////////////////////////////////////////////////////////
	//	Sensor			 : Hi-544
	//		Mode			 : Capture 
	//		Size			 : 2592 * 1944
	//////////////////////////////////////////////////////////////////////////
#if 0
	HI551_write_cmos_sensor(0x0A00, 0x0000);	//sleep On

HI551_write_cmos_sensor(0x000E, 0x0000);		// x_addr_start_lobp_h / l
HI551_write_cmos_sensor(0x0014, 0x003F);		// x_addr_end_lobp_h / l
HI551_write_cmos_sensor(0x0010, 0x0060);		// x_addr_start_robp_h / l
HI551_write_cmos_sensor(0x0016, 0x009F);		// x_addr_end_robp_h / l
HI551_write_cmos_sensor(0x0012, 0x00C0);		// x_addr_start_hact_h / l 192
HI551_write_cmos_sensor(0x0018, 0x0ADF);		// x_addr_end_hact_h / l 2783
HI551_write_cmos_sensor(0x0022, 0x0004);		// y_addr_start_fobp_h / l
HI551_write_cmos_sensor(0x0028, 0x000B);		// y_addr_end_fobp_h / l
HI551_write_cmos_sensor(0x0024, 0xFFFA);		// y_addr_start_dummy_h / l
HI551_write_cmos_sensor(0x002A, 0xFFFF);		// y_addr_end_dummy_h / l
HI551_write_cmos_sensor(0x0026, 0x0030);		// y_addr_start_vact_h / l 48
HI551_write_cmos_sensor(0x002C, 0x07C7);		// y_addr_end_vact_h / l 1991
HI551_write_cmos_sensor(0x0032, 0x0101);		// y_odd_inc_vact / y_even_inc_vact
//mipi
HI551_write_cmos_sensor(0x090A, 0x03E8);		// mipi_vblank_delay_h / l
HI551_write_cmos_sensor(0x090C, 0x0020);		// mipi_hblank_short_delay_h / l
HI551_write_cmos_sensor(0x090E, 0x0E00);		// mipi_hblank_long_delay_h / l
HI551_write_cmos_sensor(0x0902, 0x4101);		// mipi_tx_op_mode1 / mipi_tx_op_mode2


// Interval Time (2byte write)
HI551_write_cmos_sensor(0x0910, 0x5D07); // mipi_Exit_sequence / mipi_LPX
HI551_write_cmos_sensor(0x0912, 0x061e); // mipi_CLK_prepare   / mipi_clk_zero
HI551_write_cmos_sensor(0x0914, 0x0407); // mipi_clk_pre		/ mipi_data_prepare
HI551_write_cmos_sensor(0x0916, 0x0b0a); // mipi_data_zero		/ mipi_data_trail	
HI551_write_cmos_sensor(0x0918, 0x0e09); // mipi_clk_post		/ mipi_clk_trail

// size info -----------------------------------------------------
HI551_write_cmos_sensor(0x0006, 0x07ca);		// frame_length_lines_h / l 1994 
HI551_write_cmos_sensor(0x0008, 0x0B7C);		// line_length_pck_h / l 2940
HI551_write_cmos_sensor(0x0020, 0x0700);		// x_region_sel / x_region_orient
HI551_write_cmos_sensor(0x0034, 0x0700);		// y_region_sel / y_resion_orient
//HI551_write_cmos_sensor(0x0A12, 0x0A2C);		// x_output_size_h / l
//HI551_write_cmos_sensor(0x0A14, 0x07A8);		// y_output_size_h / l
HI551_write_cmos_sensor(0x0b16, 0x440f);	

HI551_write_cmos_sensor(0x0112, 0x0A20);		// x_output_size_h / l 2592
HI551_write_cmos_sensor(0x0114, 0x0798);		// y_output_size_h / l 1944 
HI551_write_cmos_sensor(0x0A00, 0x0100);	

	
	mDELAY(50);
	#else
	//HI551_write_cmos_sensor(0x0A00, 0x0000); //sleep On


	//Image size 1296x972
	HI551_write_cmos_sensor(0x0700, 0x00A3);		// byrscl_ctrl1 / byrscl_fifo_read_delay
	HI551_write_cmos_sensor(0x0112, 0x0a20);		// x_output_size_h / l 1296
	HI551_write_cmos_sensor(0x0114, 0x0798);		// y_output_size_h / l 972
	//HI551_write_cmos_sensor(0x0B14, 0x270b);       //   jacky add
	HI551_write_cmos_sensor(0x0B16, 0x440F);       // pll_mipi_clk_div
	HI551_write_cmos_sensor(0x090A, 0x03E8);		// mipi_vblank_delay_h / l
	HI551_write_cmos_sensor(0x090C, 0x0020);		// mipi_hblank_short_delay_h / l
	HI551_write_cmos_sensor(0x090E, 0x0e00);		// mipi_hblank_long_delay_h / l
	HI551_write_cmos_sensor(0x0910, 0x5D07);		// mipi_value_exit_seq / mipi_value_lpx
	HI551_write_cmos_sensor(0x0912, 0x061E);		// mipi_value_clk_prepare / mipi_value_clk_zeo  0x030E
	HI551_write_cmos_sensor(0x0914, 0x0407);		// mipi_value_clk_pre / mipi_value_data_prepare
	HI551_write_cmos_sensor(0x0916, 0x0b0a);		// mipi_value_data_zero / mipi_value_data_trail
	HI551_write_cmos_sensor(0x0918, 0x0e08);		// mipi_value_clk_post / mipi_value_clk_trail
	HI551_write_cmos_sensor(0x091A, 0x0a00);		// mipi_value_exit / null

	//HI551_write_cmos_sensor(0x0A00, 0x0100); //sleep Off 

	mDELAY(300);
	
#endif
	ReEnteyCamera = KAL_FALSE;
	
	HI551DB("HI551CaptureSetting_2lane exit :\n ");
}

static void HI551_Sensor_Init(void)
{

	HI551DB("HI551_Sensor_Init 2lane_OB:\n ");	
	
    ReEnteyCamera = KAL_TRUE;
	mDELAY(20);
//====================================================================
// Hi-551 Initialize Register
// Ver 140701
// MTK6571 Validation
//====================================================================

HI551_write_cmos_sensor(0x0134, 0x1C1F);       // Extra Dynamic Signal control 
//====================================================================
// software code 
//	- d2a_signal ver. 20140628
// 	- tg ver. 20140628
//====================================================================
// Firmware tg_pt
HI551_write_cmos_sensor(0x2000, 0x4031);
HI551_write_cmos_sensor(0x2002, 0x8400);
HI551_write_cmos_sensor(0x2004, 0x4307);
HI551_write_cmos_sensor(0x2006, 0x4382);
HI551_write_cmos_sensor(0x2008, 0x8144);
HI551_write_cmos_sensor(0x200a, 0x4382);
HI551_write_cmos_sensor(0x200c, 0x8104);
HI551_write_cmos_sensor(0x200e, 0x4392);
HI551_write_cmos_sensor(0x2010, 0x731C);
HI551_write_cmos_sensor(0x2012, 0x4392);
HI551_write_cmos_sensor(0x2014, 0x8118);
HI551_write_cmos_sensor(0x2016, 0x4382);
HI551_write_cmos_sensor(0x2018, 0x8128);
HI551_write_cmos_sensor(0x201a, 0x4382);
HI551_write_cmos_sensor(0x201c, 0x8126);
HI551_write_cmos_sensor(0x201e, 0x4382);
HI551_write_cmos_sensor(0x2020, 0x8100);
HI551_write_cmos_sensor(0x2022, 0x4382);
HI551_write_cmos_sensor(0x2024, 0x811E);
HI551_write_cmos_sensor(0x2026, 0x4382);
HI551_write_cmos_sensor(0x2028, 0x811C);
HI551_write_cmos_sensor(0x202a, 0x4382);
HI551_write_cmos_sensor(0x202c, 0x811A);
HI551_write_cmos_sensor(0x202e, 0x4382);
HI551_write_cmos_sensor(0x2030, 0x8108);
HI551_write_cmos_sensor(0x2032, 0x4382);
HI551_write_cmos_sensor(0x2034, 0x8124);
HI551_write_cmos_sensor(0x2036, 0x430B);
HI551_write_cmos_sensor(0x2038, 0x93D2);
HI551_write_cmos_sensor(0x203a, 0x003F);
HI551_write_cmos_sensor(0x203c, 0x2002);
HI551_write_cmos_sensor(0x203e, 0x4030);
HI551_write_cmos_sensor(0x2040, 0xF71C);
HI551_write_cmos_sensor(0x2042, 0x43C2);
HI551_write_cmos_sensor(0x2044, 0x0F82);
HI551_write_cmos_sensor(0x2046, 0x425F);
HI551_write_cmos_sensor(0x2048, 0x0198);
HI551_write_cmos_sensor(0x204a, 0xF37F);
HI551_write_cmos_sensor(0x204c, 0x930F);
HI551_write_cmos_sensor(0x204e, 0x2002);
HI551_write_cmos_sensor(0x2050, 0x0CC8);
HI551_write_cmos_sensor(0x2052, 0x3FF9);
HI551_write_cmos_sensor(0x2054, 0x4F82);
HI551_write_cmos_sensor(0x2056, 0x8122);
HI551_write_cmos_sensor(0x2058, 0x43D2);
HI551_write_cmos_sensor(0x205a, 0x0A80);
HI551_write_cmos_sensor(0x205c, 0x43C2);
HI551_write_cmos_sensor(0x205e, 0x1093);
HI551_write_cmos_sensor(0x2060, 0x43D2);
HI551_write_cmos_sensor(0x2062, 0x0180);
HI551_write_cmos_sensor(0x2064, 0x4392);
HI551_write_cmos_sensor(0x2066, 0x019A);
HI551_write_cmos_sensor(0x2068, 0x40B2);
HI551_write_cmos_sensor(0x206a, 0x0009);
HI551_write_cmos_sensor(0x206c, 0x019B);
HI551_write_cmos_sensor(0x206e, 0x12B0);
HI551_write_cmos_sensor(0x2070, 0xFD8C);
HI551_write_cmos_sensor(0x2072, 0x93D2);
HI551_write_cmos_sensor(0x2074, 0x003E);
HI551_write_cmos_sensor(0x2076, 0x2002);
HI551_write_cmos_sensor(0x2078, 0x4030);
HI551_write_cmos_sensor(0x207a, 0xF5D6);
HI551_write_cmos_sensor(0x207c, 0x4382);
HI551_write_cmos_sensor(0x207e, 0x8120);
HI551_write_cmos_sensor(0x2080, 0x4382);
HI551_write_cmos_sensor(0x2082, 0x8102);
HI551_write_cmos_sensor(0x2084, 0x421F);
HI551_write_cmos_sensor(0x2086, 0x8120);
HI551_write_cmos_sensor(0x2088, 0x521F);
HI551_write_cmos_sensor(0x208a, 0x8102);
HI551_write_cmos_sensor(0x208c, 0x503F);
HI551_write_cmos_sensor(0x208e, 0x0030);
HI551_write_cmos_sensor(0x2090, 0x12B0);
HI551_write_cmos_sensor(0x2092, 0xFD9E);
HI551_write_cmos_sensor(0x2094, 0x421E);
HI551_write_cmos_sensor(0x2096, 0x8102);
HI551_write_cmos_sensor(0x2098, 0x4E0F);
HI551_write_cmos_sensor(0x209a, 0x5F0F);
HI551_write_cmos_sensor(0x209c, 0x429F);
HI551_write_cmos_sensor(0x209e, 0x7606);
HI551_write_cmos_sensor(0x20a0, 0x8132);
HI551_write_cmos_sensor(0x20a2, 0x531E);
HI551_write_cmos_sensor(0x20a4, 0x4E82);
HI551_write_cmos_sensor(0x20a6, 0x8102);
HI551_write_cmos_sensor(0x20a8, 0x903E);
HI551_write_cmos_sensor(0x20aa, 0x0003);
HI551_write_cmos_sensor(0x20ac, 0x3BEB);
HI551_write_cmos_sensor(0x20ae, 0x421F);
HI551_write_cmos_sensor(0x20b0, 0x8134);
HI551_write_cmos_sensor(0x20b2, 0x4F0D);
HI551_write_cmos_sensor(0x20b4, 0xF03D);
HI551_write_cmos_sensor(0x20b6, 0x000F);
HI551_write_cmos_sensor(0x20b8, 0x108D);
HI551_write_cmos_sensor(0x20ba, 0x521D);
HI551_write_cmos_sensor(0x20bc, 0x8132);
HI551_write_cmos_sensor(0x20be, 0x4D82);
HI551_write_cmos_sensor(0x20c0, 0x8130);
HI551_write_cmos_sensor(0x20c2, 0x421E);
HI551_write_cmos_sensor(0x20c4, 0x8136);
HI551_write_cmos_sensor(0x20c6, 0x5E0E);
HI551_write_cmos_sensor(0x20c8, 0x5E0E);
HI551_write_cmos_sensor(0x20ca, 0x5E0E);
HI551_write_cmos_sensor(0x20cc, 0x5E0E);
HI551_write_cmos_sensor(0x20ce, 0xC312);
HI551_write_cmos_sensor(0x20d0, 0x100F);
HI551_write_cmos_sensor(0x20d2, 0x110F);
HI551_write_cmos_sensor(0x20d4, 0x110F);
HI551_write_cmos_sensor(0x20d6, 0x110F);
HI551_write_cmos_sensor(0x20d8, 0x5F0E);
HI551_write_cmos_sensor(0x20da, 0x4E82);
HI551_write_cmos_sensor(0x20dc, 0x812E);
HI551_write_cmos_sensor(0x20de, 0x4D8B);
HI551_write_cmos_sensor(0x20e0, 0x5000);
HI551_write_cmos_sensor(0x20e2, 0x429B);
HI551_write_cmos_sensor(0x20e4, 0x812E);
HI551_write_cmos_sensor(0x20e6, 0x6000);
HI551_write_cmos_sensor(0x20e8, 0x532B);
HI551_write_cmos_sensor(0x20ea, 0x421F);
HI551_write_cmos_sensor(0x20ec, 0x8120);
HI551_write_cmos_sensor(0x20ee, 0x503F);
HI551_write_cmos_sensor(0x20f0, 0x0003);
HI551_write_cmos_sensor(0x20f2, 0x4F82);
HI551_write_cmos_sensor(0x20f4, 0x8120);
HI551_write_cmos_sensor(0x20f6, 0x903F);
HI551_write_cmos_sensor(0x20f8, 0x0300);
HI551_write_cmos_sensor(0x20fa, 0x3BC2);
HI551_write_cmos_sensor(0x20fc, 0x0261);
HI551_write_cmos_sensor(0x20fe, 0x0000);
HI551_write_cmos_sensor(0x2100, 0x43C2);
HI551_write_cmos_sensor(0x2102, 0x0180);
HI551_write_cmos_sensor(0x2104, 0x43D2);
HI551_write_cmos_sensor(0x2106, 0x003F);
HI551_write_cmos_sensor(0x2108, 0x403F);
HI551_write_cmos_sensor(0x210a, 0x0B80);
HI551_write_cmos_sensor(0x210c, 0x4F2E);
HI551_write_cmos_sensor(0x210e, 0xD03E);
HI551_write_cmos_sensor(0x2110, 0x8000);
HI551_write_cmos_sensor(0x2112, 0x4E82);
HI551_write_cmos_sensor(0x2114, 0x0B20);
HI551_write_cmos_sensor(0x2116, 0xD0BF);
HI551_write_cmos_sensor(0x2118, 0x8000);
HI551_write_cmos_sensor(0x211a, 0x0000);
HI551_write_cmos_sensor(0x211c, 0x0C0A);
HI551_write_cmos_sensor(0x211e, 0x40B2);
HI551_write_cmos_sensor(0x2120, 0x8085);
HI551_write_cmos_sensor(0x2122, 0x0B20);
HI551_write_cmos_sensor(0x2124, 0x40B2);
HI551_write_cmos_sensor(0x2126, 0x8085);
HI551_write_cmos_sensor(0x2128, 0x0B88);
HI551_write_cmos_sensor(0x212a, 0x0C0A);
HI551_write_cmos_sensor(0x212c, 0x40B2);
HI551_write_cmos_sensor(0x212e, 0xAC49);
HI551_write_cmos_sensor(0x2130, 0x0B20);
HI551_write_cmos_sensor(0x2132, 0x40B2);
HI551_write_cmos_sensor(0x2134, 0xAC49);
HI551_write_cmos_sensor(0x2136, 0x0B8A);
HI551_write_cmos_sensor(0x2138, 0x0C0A);
HI551_write_cmos_sensor(0x213a, 0x40B2);
HI551_write_cmos_sensor(0x213c, 0xE1BC);
HI551_write_cmos_sensor(0x213e, 0x0B20);
HI551_write_cmos_sensor(0x2140, 0x40B2);
HI551_write_cmos_sensor(0x2142, 0xE1BC);
HI551_write_cmos_sensor(0x2144, 0x0B8C);
HI551_write_cmos_sensor(0x2146, 0x0C0A);
HI551_write_cmos_sensor(0x2148, 0x40B2);
HI551_write_cmos_sensor(0x214a, 0x0500);
HI551_write_cmos_sensor(0x214c, 0x0B20);
HI551_write_cmos_sensor(0x214e, 0x40B2);
HI551_write_cmos_sensor(0x2150, 0x0500);
HI551_write_cmos_sensor(0x2152, 0x0B9E);
HI551_write_cmos_sensor(0x2154, 0x0C0A);
HI551_write_cmos_sensor(0x2156, 0x43D2);
HI551_write_cmos_sensor(0x2158, 0x0F82);
HI551_write_cmos_sensor(0x215a, 0x0C3C);
HI551_write_cmos_sensor(0x215c, 0x0C3C);
HI551_write_cmos_sensor(0x215e, 0x0C3C);
HI551_write_cmos_sensor(0x2160, 0x0C3C);
HI551_write_cmos_sensor(0x2162, 0x421F);
HI551_write_cmos_sensor(0x2164, 0x00A6);
HI551_write_cmos_sensor(0x2166, 0x503F);
HI551_write_cmos_sensor(0x2168, 0x07D0);
HI551_write_cmos_sensor(0x216a, 0x3811);
HI551_write_cmos_sensor(0x216c, 0x4F82);
HI551_write_cmos_sensor(0x216e, 0x7100);
HI551_write_cmos_sensor(0x2170, 0x0004);
HI551_write_cmos_sensor(0x2172, 0x0C0D);
HI551_write_cmos_sensor(0x2174, 0x0005);
HI551_write_cmos_sensor(0x2176, 0x0C04);
HI551_write_cmos_sensor(0x2178, 0x000D);
HI551_write_cmos_sensor(0x217a, 0x0C09);
HI551_write_cmos_sensor(0x217c, 0x003D);
HI551_write_cmos_sensor(0x217e, 0x0C1D);
HI551_write_cmos_sensor(0x2180, 0x003C);
HI551_write_cmos_sensor(0x2182, 0x0C13);
HI551_write_cmos_sensor(0x2184, 0x0004);
HI551_write_cmos_sensor(0x2186, 0x0C09);
HI551_write_cmos_sensor(0x2188, 0x0004);
HI551_write_cmos_sensor(0x218a, 0x533F);
HI551_write_cmos_sensor(0x218c, 0x37EF);
HI551_write_cmos_sensor(0x218e, 0x40B2);
HI551_write_cmos_sensor(0x2190, 0x0028);
HI551_write_cmos_sensor(0x2192, 0x7000);
HI551_write_cmos_sensor(0x2194, 0x43A2);
HI551_write_cmos_sensor(0x2196, 0x812C);
HI551_write_cmos_sensor(0x2198, 0xB3E2);
HI551_write_cmos_sensor(0x219a, 0x00B4);
HI551_write_cmos_sensor(0x219c, 0x2402);
HI551_write_cmos_sensor(0x219e, 0x4392);
HI551_write_cmos_sensor(0x21a0, 0x812C);
HI551_write_cmos_sensor(0x21a2, 0x4328);
HI551_write_cmos_sensor(0x21a4, 0xB3D2);
HI551_write_cmos_sensor(0x21a6, 0x00B4);
HI551_write_cmos_sensor(0x21a8, 0x2002);
HI551_write_cmos_sensor(0x21aa, 0x4030);
HI551_write_cmos_sensor(0x21ac, 0xF5C6);
HI551_write_cmos_sensor(0x21ae, 0x4308);
HI551_write_cmos_sensor(0x21b0, 0x40B2);
HI551_write_cmos_sensor(0x21b2, 0x0005);
HI551_write_cmos_sensor(0x21b4, 0x7320);
HI551_write_cmos_sensor(0x21b6, 0x4392);
HI551_write_cmos_sensor(0x21b8, 0x7326);
HI551_write_cmos_sensor(0x21ba, 0x12B0);
HI551_write_cmos_sensor(0x21bc, 0xF928);
HI551_write_cmos_sensor(0x21be, 0x4392);
HI551_write_cmos_sensor(0x21c0, 0x731C);
HI551_write_cmos_sensor(0x21c2, 0x9382);
HI551_write_cmos_sensor(0x21c4, 0x8118);
HI551_write_cmos_sensor(0x21c6, 0x200A);
HI551_write_cmos_sensor(0x21c8, 0x0B00);
HI551_write_cmos_sensor(0x21ca, 0x7302);
HI551_write_cmos_sensor(0x21cc, 0x02BC);
HI551_write_cmos_sensor(0x21ce, 0x4382);
HI551_write_cmos_sensor(0x21d0, 0x7004);
HI551_write_cmos_sensor(0x21d2, 0x430F);
HI551_write_cmos_sensor(0x21d4, 0x12B0);
HI551_write_cmos_sensor(0x21d6, 0xF7D6);
HI551_write_cmos_sensor(0x21d8, 0x12B0);
HI551_write_cmos_sensor(0x21da, 0xF928);
HI551_write_cmos_sensor(0x21dc, 0x4392);
HI551_write_cmos_sensor(0x21de, 0x8138);
HI551_write_cmos_sensor(0x21e0, 0x4382);
HI551_write_cmos_sensor(0x21e2, 0x740E);
HI551_write_cmos_sensor(0x21e4, 0xB3E2);
HI551_write_cmos_sensor(0x21e6, 0x0080);
HI551_write_cmos_sensor(0x21e8, 0x2402);
HI551_write_cmos_sensor(0x21ea, 0x4392);
HI551_write_cmos_sensor(0x21ec, 0x740E);
HI551_write_cmos_sensor(0x21ee, 0x431F);
HI551_write_cmos_sensor(0x21f0, 0x12B0);
HI551_write_cmos_sensor(0x21f2, 0xF7D6);
HI551_write_cmos_sensor(0x21f4, 0x4392);
HI551_write_cmos_sensor(0x21f6, 0x7004);
HI551_write_cmos_sensor(0x21f8, 0x4882);
HI551_write_cmos_sensor(0x21fa, 0x7110);
HI551_write_cmos_sensor(0x21fc, 0x403F);
HI551_write_cmos_sensor(0x21fe, 0x732A);
HI551_write_cmos_sensor(0x2200, 0x4F2E);
HI551_write_cmos_sensor(0x2202, 0xF31E);
HI551_write_cmos_sensor(0x2204, 0x4E82);
HI551_write_cmos_sensor(0x2206, 0x8100);
HI551_write_cmos_sensor(0x2208, 0x4F2F);
HI551_write_cmos_sensor(0x220a, 0xF32F);
HI551_write_cmos_sensor(0x220c, 0x4F82);
HI551_write_cmos_sensor(0x220e, 0x811C);
HI551_write_cmos_sensor(0x2210, 0xD21F);
HI551_write_cmos_sensor(0x2212, 0x811A);
HI551_write_cmos_sensor(0x2214, 0x4F82);
HI551_write_cmos_sensor(0x2216, 0x8116);
HI551_write_cmos_sensor(0x2218, 0x9382);
HI551_write_cmos_sensor(0x221a, 0x810C);
HI551_write_cmos_sensor(0x221c, 0x2403);
HI551_write_cmos_sensor(0x221e, 0x9392);
HI551_write_cmos_sensor(0x2220, 0x7110);
HI551_write_cmos_sensor(0x2222, 0x2128);
HI551_write_cmos_sensor(0x2224, 0x9392);
HI551_write_cmos_sensor(0x2226, 0x7110);
HI551_write_cmos_sensor(0x2228, 0x2098);
HI551_write_cmos_sensor(0x222a, 0x0B00);
HI551_write_cmos_sensor(0x222c, 0x7302);
HI551_write_cmos_sensor(0x222e, 0x0032);
HI551_write_cmos_sensor(0x2230, 0x4382);
HI551_write_cmos_sensor(0x2232, 0x7004);
HI551_write_cmos_sensor(0x2234, 0x422F);
HI551_write_cmos_sensor(0x2236, 0x12B0);
HI551_write_cmos_sensor(0x2238, 0xF7D6);
HI551_write_cmos_sensor(0x223a, 0x0800);
HI551_write_cmos_sensor(0x223c, 0x7114);
HI551_write_cmos_sensor(0x223e, 0x403F);
HI551_write_cmos_sensor(0x2240, 0x0003);
HI551_write_cmos_sensor(0x2242, 0x12B0);
HI551_write_cmos_sensor(0x2244, 0xF7D6);
HI551_write_cmos_sensor(0x2246, 0x425F);
HI551_write_cmos_sensor(0x2248, 0x0CA7);
HI551_write_cmos_sensor(0x224a, 0xF37F);
HI551_write_cmos_sensor(0x224c, 0x421A);
HI551_write_cmos_sensor(0x224e, 0x8114);
HI551_write_cmos_sensor(0x2250, 0x4F0C);
HI551_write_cmos_sensor(0x2252, 0x12B0);
HI551_write_cmos_sensor(0x2254, 0xFDBE);
HI551_write_cmos_sensor(0x2256, 0x4E0A);
HI551_write_cmos_sensor(0x2258, 0xC312);
HI551_write_cmos_sensor(0x225a, 0x100A);
HI551_write_cmos_sensor(0x225c, 0x110A);
HI551_write_cmos_sensor(0x225e, 0x110A);
HI551_write_cmos_sensor(0x2260, 0x110A);
HI551_write_cmos_sensor(0x2262, 0x903A);
HI551_write_cmos_sensor(0x2264, 0x0010);
HI551_write_cmos_sensor(0x2266, 0x2C02);
HI551_write_cmos_sensor(0x2268, 0x403A);
HI551_write_cmos_sensor(0x226a, 0x0010);
HI551_write_cmos_sensor(0x226c, 0x425F);
HI551_write_cmos_sensor(0x226e, 0x0CA6);
HI551_write_cmos_sensor(0x2270, 0xF37F);
HI551_write_cmos_sensor(0x2272, 0x5F0F);
HI551_write_cmos_sensor(0x2274, 0x5F0F);
HI551_write_cmos_sensor(0x2276, 0x5F0F);
HI551_write_cmos_sensor(0x2278, 0x5F0F);
HI551_write_cmos_sensor(0x227a, 0x4F0C);
HI551_write_cmos_sensor(0x227c, 0x12B0);
HI551_write_cmos_sensor(0x227e, 0xFDD8);
HI551_write_cmos_sensor(0x2280, 0x4C82);
HI551_write_cmos_sensor(0x2282, 0x8140);
HI551_write_cmos_sensor(0x2284, 0x425F);
HI551_write_cmos_sensor(0x2286, 0x0C9C);
HI551_write_cmos_sensor(0x2288, 0x4F4E);
HI551_write_cmos_sensor(0x228a, 0x430F);
HI551_write_cmos_sensor(0x228c, 0x4E0F);
HI551_write_cmos_sensor(0x228e, 0x430E);
HI551_write_cmos_sensor(0x2290, 0x421A);
HI551_write_cmos_sensor(0x2292, 0x0C9A);
HI551_write_cmos_sensor(0x2294, 0x430B);
HI551_write_cmos_sensor(0x2296, 0xDE0A);
HI551_write_cmos_sensor(0x2298, 0xDF0B);
HI551_write_cmos_sensor(0x229a, 0x425F);
HI551_write_cmos_sensor(0x229c, 0x0CA0);
HI551_write_cmos_sensor(0x229e, 0x4F4E);
HI551_write_cmos_sensor(0x22a0, 0x430F);
HI551_write_cmos_sensor(0x22a2, 0x4E0F);
HI551_write_cmos_sensor(0x22a4, 0x430E);
HI551_write_cmos_sensor(0x22a6, 0x421D);
HI551_write_cmos_sensor(0x22a8, 0x0C9E);
HI551_write_cmos_sensor(0x22aa, 0xDD0E);
HI551_write_cmos_sensor(0x22ac, 0x5E0A);
HI551_write_cmos_sensor(0x22ae, 0x6F0B);
HI551_write_cmos_sensor(0x22b0, 0x421E);
HI551_write_cmos_sensor(0x22b2, 0x0CA2);
HI551_write_cmos_sensor(0x22b4, 0x521E);
HI551_write_cmos_sensor(0x22b6, 0x0CA4);
HI551_write_cmos_sensor(0x22b8, 0x4382);
HI551_write_cmos_sensor(0x22ba, 0x8110);
HI551_write_cmos_sensor(0x22bc, 0xB3D2);
HI551_write_cmos_sensor(0x22be, 0x0C8E);
HI551_write_cmos_sensor(0x22c0, 0x2430);
HI551_write_cmos_sensor(0x22c2, 0xB3D2);
HI551_write_cmos_sensor(0x22c4, 0x0C80);
HI551_write_cmos_sensor(0x22c6, 0x242D);
HI551_write_cmos_sensor(0x22c8, 0x430F);
HI551_write_cmos_sensor(0x22ca, 0x1209);
HI551_write_cmos_sensor(0x22cc, 0x4E0C);
HI551_write_cmos_sensor(0x22ce, 0x4F0D);
HI551_write_cmos_sensor(0x22d0, 0x4A0E);
HI551_write_cmos_sensor(0x22d2, 0x4B0F);
HI551_write_cmos_sensor(0x22d4, 0x12B0);
HI551_write_cmos_sensor(0x22d6, 0xF840);
HI551_write_cmos_sensor(0x22d8, 0x4F09);
HI551_write_cmos_sensor(0x22da, 0x403F);
HI551_write_cmos_sensor(0x22dc, 0x0003);
HI551_write_cmos_sensor(0x22de, 0x12B0);
HI551_write_cmos_sensor(0x22e0, 0xF7D6);
HI551_write_cmos_sensor(0x22e2, 0x4982);
HI551_write_cmos_sensor(0x22e4, 0x8110);
HI551_write_cmos_sensor(0x22e6, 0x5321);
HI551_write_cmos_sensor(0x22e8, 0x90B2);
HI551_write_cmos_sensor(0x22ea, 0x0019);
HI551_write_cmos_sensor(0x22ec, 0x813C);
HI551_write_cmos_sensor(0x22ee, 0x2831);
HI551_write_cmos_sensor(0x22f0, 0x4982);
HI551_write_cmos_sensor(0x22f2, 0x8106);
HI551_write_cmos_sensor(0x22f4, 0x421F);
HI551_write_cmos_sensor(0x22f6, 0x8110);
HI551_write_cmos_sensor(0x22f8, 0x523F);
HI551_write_cmos_sensor(0x22fa, 0x110F);
HI551_write_cmos_sensor(0x22fc, 0x110F);
HI551_write_cmos_sensor(0x22fe, 0x110F);
HI551_write_cmos_sensor(0x2300, 0x110F);
HI551_write_cmos_sensor(0x2302, 0x4F82);
HI551_write_cmos_sensor(0x2304, 0x8110);
HI551_write_cmos_sensor(0x2306, 0x903F);
HI551_write_cmos_sensor(0x2308, 0x001E);
HI551_write_cmos_sensor(0x230a, 0x3821);
HI551_write_cmos_sensor(0x230c, 0x503F);
HI551_write_cmos_sensor(0x230e, 0xFFE2);
HI551_write_cmos_sensor(0x2310, 0x4F82);
HI551_write_cmos_sensor(0x2312, 0x8110);
HI551_write_cmos_sensor(0x2314, 0x4F82);
HI551_write_cmos_sensor(0x2316, 0x0CAC);
HI551_write_cmos_sensor(0x2318, 0x4F82);
HI551_write_cmos_sensor(0x231a, 0x0CAE);
HI551_write_cmos_sensor(0x231c, 0x42D2);
HI551_write_cmos_sensor(0x231e, 0x0C93);
HI551_write_cmos_sensor(0x2320, 0x0C96);
HI551_write_cmos_sensor(0x2322, 0xC3E2);
HI551_write_cmos_sensor(0x2324, 0x0C81);
HI551_write_cmos_sensor(0x2326, 0x4292);
HI551_write_cmos_sensor(0x2328, 0x8118);
HI551_write_cmos_sensor(0x232a, 0x8128);
HI551_write_cmos_sensor(0x232c, 0x421F);
HI551_write_cmos_sensor(0x232e, 0x8126);
HI551_write_cmos_sensor(0x2330, 0xD21F);
HI551_write_cmos_sensor(0x2332, 0x8118);
HI551_write_cmos_sensor(0x2334, 0x5F0F);
HI551_write_cmos_sensor(0x2336, 0xF03F);
HI551_write_cmos_sensor(0x2338, 0x000E);
HI551_write_cmos_sensor(0x233a, 0x4F82);
HI551_write_cmos_sensor(0x233c, 0x8126);
HI551_write_cmos_sensor(0x233e, 0x4382);
HI551_write_cmos_sensor(0x2340, 0x8118);
HI551_write_cmos_sensor(0x2342, 0x432F);
HI551_write_cmos_sensor(0x2344, 0x12B0);
HI551_write_cmos_sensor(0x2346, 0xF7D6);
HI551_write_cmos_sensor(0x2348, 0x12B0);
HI551_write_cmos_sensor(0x234a, 0xF776);
HI551_write_cmos_sensor(0x234c, 0x3F65);
HI551_write_cmos_sensor(0x234e, 0x430F);
HI551_write_cmos_sensor(0x2350, 0x3FDF);
HI551_write_cmos_sensor(0x2352, 0x4292);
HI551_write_cmos_sensor(0x2354, 0x8106);
HI551_write_cmos_sensor(0x2356, 0x8110);
HI551_write_cmos_sensor(0x2358, 0x3FCD);
HI551_write_cmos_sensor(0x235a, 0x0B00);
HI551_write_cmos_sensor(0x235c, 0x7302);
HI551_write_cmos_sensor(0x235e, 0x0002);
HI551_write_cmos_sensor(0x2360, 0x0404);
HI551_write_cmos_sensor(0x2362, 0x0C01);
HI551_write_cmos_sensor(0x2364, 0x0001);
HI551_write_cmos_sensor(0x2366, 0x0C01);
HI551_write_cmos_sensor(0x2368, 0x0003);
HI551_write_cmos_sensor(0x236a, 0x0C01);
HI551_write_cmos_sensor(0x236c, 0x004B);
HI551_write_cmos_sensor(0x236e, 0x0C0A);
HI551_write_cmos_sensor(0x2370, 0x0243);
HI551_write_cmos_sensor(0x2372, 0x791F);
HI551_write_cmos_sensor(0x2374, 0x0C01);
HI551_write_cmos_sensor(0x2376, 0x0405);
HI551_write_cmos_sensor(0x2378, 0x0C41);
HI551_write_cmos_sensor(0x237a, 0x0647);
HI551_write_cmos_sensor(0x237c, 0x0C03);
HI551_write_cmos_sensor(0x237e, 0x0672);
HI551_write_cmos_sensor(0x2380, 0x0C01);
HI551_write_cmos_sensor(0x2382, 0x0670);
HI551_write_cmos_sensor(0x2384, 0x0C01);
HI551_write_cmos_sensor(0x2386, 0x0778);
HI551_write_cmos_sensor(0x2388, 0x0C0B);
HI551_write_cmos_sensor(0x238a, 0x067C);
HI551_write_cmos_sensor(0x238c, 0x0C07);
HI551_write_cmos_sensor(0x238e, 0x0686);
HI551_write_cmos_sensor(0x2390, 0x0C01);
HI551_write_cmos_sensor(0x2392, 0x066C);
HI551_write_cmos_sensor(0x2394, 0x0C7D);
HI551_write_cmos_sensor(0x2396, 0x0684);
HI551_write_cmos_sensor(0x2398, 0x0C8E);
HI551_write_cmos_sensor(0x239a, 0x0203);
HI551_write_cmos_sensor(0x239c, 0x4E1F);
HI551_write_cmos_sensor(0x239e, 0x0393);
HI551_write_cmos_sensor(0x23a0, 0x7A1F);
HI551_write_cmos_sensor(0x23a2, 0x0C0B);
HI551_write_cmos_sensor(0x23a4, 0x0083);
HI551_write_cmos_sensor(0x23a6, 0x0C55);
HI551_write_cmos_sensor(0x23a8, 0x0686);
HI551_write_cmos_sensor(0x23aa, 0x0C01);
HI551_write_cmos_sensor(0x23ac, 0x0664);
HI551_write_cmos_sensor(0x23ae, 0x0CFF);
HI551_write_cmos_sensor(0x23b0, 0x0CC2);
HI551_write_cmos_sensor(0x23b2, 0x0684);
HI551_write_cmos_sensor(0x23b4, 0x4392);
HI551_write_cmos_sensor(0x23b6, 0x7004);
HI551_write_cmos_sensor(0x23b8, 0x430F);
HI551_write_cmos_sensor(0x23ba, 0x9382);
HI551_write_cmos_sensor(0x23bc, 0x8138);
HI551_write_cmos_sensor(0x23be, 0x2001);
HI551_write_cmos_sensor(0x23c0, 0x431F);
HI551_write_cmos_sensor(0x23c2, 0x4F82);
HI551_write_cmos_sensor(0x23c4, 0x8138);
HI551_write_cmos_sensor(0x23c6, 0x12B0);
HI551_write_cmos_sensor(0x23c8, 0xF776);
HI551_write_cmos_sensor(0x23ca, 0x930F);
HI551_write_cmos_sensor(0x23cc, 0x2405);
HI551_write_cmos_sensor(0x23ce, 0x4292);
HI551_write_cmos_sensor(0x23d0, 0x8118);
HI551_write_cmos_sensor(0x23d2, 0x8128);
HI551_write_cmos_sensor(0x23d4, 0x4382);
HI551_write_cmos_sensor(0x23d6, 0x8118);
HI551_write_cmos_sensor(0x23d8, 0x9382);
HI551_write_cmos_sensor(0x23da, 0x8138);
HI551_write_cmos_sensor(0x23dc, 0x242C);
HI551_write_cmos_sensor(0x23de, 0x9382);
HI551_write_cmos_sensor(0x23e0, 0x8108);
HI551_write_cmos_sensor(0x23e2, 0x2003);
HI551_write_cmos_sensor(0x23e4, 0x9382);
HI551_write_cmos_sensor(0x23e6, 0x8124);
HI551_write_cmos_sensor(0x23e8, 0x2416);
HI551_write_cmos_sensor(0x23ea, 0x0B00);
HI551_write_cmos_sensor(0x23ec, 0x7302);
HI551_write_cmos_sensor(0x23ee, 0x04FE);
HI551_write_cmos_sensor(0x23f0, 0x0674);
HI551_write_cmos_sensor(0x23f2, 0x0C03);
HI551_write_cmos_sensor(0x23f4, 0x0509);
HI551_write_cmos_sensor(0x23f6, 0x0C01);
HI551_write_cmos_sensor(0x23f8, 0x003C);
HI551_write_cmos_sensor(0x23fa, 0x0C01);
HI551_write_cmos_sensor(0x23fc, 0x040A);
HI551_write_cmos_sensor(0x23fe, 0x9382);
HI551_write_cmos_sensor(0x2400, 0x8144);
HI551_write_cmos_sensor(0x2402, 0x2003);
HI551_write_cmos_sensor(0x2404, 0x930F);
HI551_write_cmos_sensor(0x2406, 0x2708);
HI551_write_cmos_sensor(0x2408, 0x3EDC);
HI551_write_cmos_sensor(0x240a, 0x43C2);
HI551_write_cmos_sensor(0x240c, 0x0A80);
HI551_write_cmos_sensor(0x240e, 0x0B00);
HI551_write_cmos_sensor(0x2410, 0x7302);
HI551_write_cmos_sensor(0x2412, 0xFFF0);
HI551_write_cmos_sensor(0x2414, 0x3F01);
HI551_write_cmos_sensor(0x2416, 0x0B00);
HI551_write_cmos_sensor(0x2418, 0x7302);
HI551_write_cmos_sensor(0x241a, 0x0580);
HI551_write_cmos_sensor(0x241c, 0x0407);
HI551_write_cmos_sensor(0x241e, 0x0C02);
HI551_write_cmos_sensor(0x2420, 0x033D);
HI551_write_cmos_sensor(0x2422, 0xB810);
HI551_write_cmos_sensor(0x2424, 0x0C01);
HI551_write_cmos_sensor(0x2426, 0x003C);
HI551_write_cmos_sensor(0x2428, 0x0C01);
HI551_write_cmos_sensor(0x242a, 0x0004);
HI551_write_cmos_sensor(0x242c, 0x0C01);
HI551_write_cmos_sensor(0x242e, 0x06A5);
HI551_write_cmos_sensor(0x2430, 0x0C03);
HI551_write_cmos_sensor(0x2432, 0x06A1);
HI551_write_cmos_sensor(0x2434, 0x3FE4);
HI551_write_cmos_sensor(0x2436, 0x9382);
HI551_write_cmos_sensor(0x2438, 0x8108);
HI551_write_cmos_sensor(0x243a, 0x2003);
HI551_write_cmos_sensor(0x243c, 0x9382);
HI551_write_cmos_sensor(0x243e, 0x8124);
HI551_write_cmos_sensor(0x2440, 0x240B);
HI551_write_cmos_sensor(0x2442, 0x0B00);
HI551_write_cmos_sensor(0x2444, 0x7302);
HI551_write_cmos_sensor(0x2446, 0x04FE);
HI551_write_cmos_sensor(0x2448, 0x0674);
HI551_write_cmos_sensor(0x244a, 0x0C03);
HI551_write_cmos_sensor(0x244c, 0x0508);
HI551_write_cmos_sensor(0x244e, 0x0C01);
HI551_write_cmos_sensor(0x2450, 0x0004);
HI551_write_cmos_sensor(0x2452, 0x0C01);
HI551_write_cmos_sensor(0x2454, 0x06A1);
HI551_write_cmos_sensor(0x2456, 0x3FD3);
HI551_write_cmos_sensor(0x2458, 0x0B00);
HI551_write_cmos_sensor(0x245a, 0x7302);
HI551_write_cmos_sensor(0x245c, 0x0580);
HI551_write_cmos_sensor(0x245e, 0x0406);
HI551_write_cmos_sensor(0x2460, 0x0C02);
HI551_write_cmos_sensor(0x2462, 0x0305);
HI551_write_cmos_sensor(0x2464, 0xB810);
HI551_write_cmos_sensor(0x2466, 0x0C01);
HI551_write_cmos_sensor(0x2468, 0x0004);
HI551_write_cmos_sensor(0x246a, 0x0C03);
HI551_write_cmos_sensor(0x246c, 0x06A5);
HI551_write_cmos_sensor(0x246e, 0x0C03);
HI551_write_cmos_sensor(0x2470, 0x06A1);
HI551_write_cmos_sensor(0x2472, 0x3FC5);
HI551_write_cmos_sensor(0x2474, 0x0B00);
HI551_write_cmos_sensor(0x2476, 0x7302);
HI551_write_cmos_sensor(0x2478, 0x0002);
HI551_write_cmos_sensor(0x247a, 0x040B);
HI551_write_cmos_sensor(0x247c, 0x0C1F);
HI551_write_cmos_sensor(0x247e, 0x06A1);
HI551_write_cmos_sensor(0x2480, 0x0C05);
HI551_write_cmos_sensor(0x2482, 0x0001);
HI551_write_cmos_sensor(0x2484, 0x0C01);
HI551_write_cmos_sensor(0x2486, 0x0003);
HI551_write_cmos_sensor(0x2488, 0x0C01);
HI551_write_cmos_sensor(0x248a, 0x004B);
HI551_write_cmos_sensor(0x248c, 0x0C17);
HI551_write_cmos_sensor(0x248e, 0x0043);
HI551_write_cmos_sensor(0x2490, 0x0C01);
HI551_write_cmos_sensor(0x2492, 0x0672);
HI551_write_cmos_sensor(0x2494, 0x0243);
HI551_write_cmos_sensor(0x2496, 0x79DF);
HI551_write_cmos_sensor(0x2498, 0x0C3B);
HI551_write_cmos_sensor(0x249a, 0x0647);
HI551_write_cmos_sensor(0x249c, 0x0C05);
HI551_write_cmos_sensor(0x249e, 0x0672);
HI551_write_cmos_sensor(0x24a0, 0x0C01);
HI551_write_cmos_sensor(0x24a2, 0x0670);
HI551_write_cmos_sensor(0x24a4, 0x0C0D);
HI551_write_cmos_sensor(0x24a6, 0x0778);
HI551_write_cmos_sensor(0x24a8, 0x0C0B);
HI551_write_cmos_sensor(0x24aa, 0x067C);
HI551_write_cmos_sensor(0x24ac, 0x0C13);
HI551_write_cmos_sensor(0x24ae, 0x0686);
HI551_write_cmos_sensor(0x24b0, 0x0C01);
HI551_write_cmos_sensor(0x24b2, 0x066C);
HI551_write_cmos_sensor(0x24b4, 0x0C67);
HI551_write_cmos_sensor(0x24b6, 0x067C);
HI551_write_cmos_sensor(0x24b8, 0x0C01);
HI551_write_cmos_sensor(0x24ba, 0x0003);
HI551_write_cmos_sensor(0x24bc, 0x0393);
HI551_write_cmos_sensor(0x24be, 0x7A1F);
HI551_write_cmos_sensor(0x24c0, 0x0C17);
HI551_write_cmos_sensor(0x24c2, 0x0083);
HI551_write_cmos_sensor(0x24c4, 0x0C6F);
HI551_write_cmos_sensor(0x24c6, 0x0686);
HI551_write_cmos_sensor(0x24c8, 0x0C01);
HI551_write_cmos_sensor(0x24ca, 0x0664);
HI551_write_cmos_sensor(0x24cc, 0x4392);
HI551_write_cmos_sensor(0x24ce, 0x7004);
HI551_write_cmos_sensor(0x24d0, 0x430F);
HI551_write_cmos_sensor(0x24d2, 0x9382);
HI551_write_cmos_sensor(0x24d4, 0x8138);
HI551_write_cmos_sensor(0x24d6, 0x2001);
HI551_write_cmos_sensor(0x24d8, 0x431F);
HI551_write_cmos_sensor(0x24da, 0x4F82);
HI551_write_cmos_sensor(0x24dc, 0x8138);
HI551_write_cmos_sensor(0x24de, 0x930F);
HI551_write_cmos_sensor(0x24e0, 0x2468);
HI551_write_cmos_sensor(0x24e2, 0x0B00);
HI551_write_cmos_sensor(0x24e4, 0x7302);
HI551_write_cmos_sensor(0x24e6, 0x033A);
HI551_write_cmos_sensor(0x24e8, 0x0674);
HI551_write_cmos_sensor(0x24ea, 0x0C02);
HI551_write_cmos_sensor(0x24ec, 0x0339);
HI551_write_cmos_sensor(0x24ee, 0xB810);
HI551_write_cmos_sensor(0x24f0, 0x0C01);
HI551_write_cmos_sensor(0x24f2, 0x003C);
HI551_write_cmos_sensor(0x24f4, 0x0C01);
HI551_write_cmos_sensor(0x24f6, 0x0004);
HI551_write_cmos_sensor(0x24f8, 0x0B00);
HI551_write_cmos_sensor(0x24fa, 0x7302);
HI551_write_cmos_sensor(0x24fc, 0x0366);
HI551_write_cmos_sensor(0x24fe, 0x040C);
HI551_write_cmos_sensor(0x2500, 0x0C25);
HI551_write_cmos_sensor(0x2502, 0x0001);
HI551_write_cmos_sensor(0x2504, 0x0C01);
HI551_write_cmos_sensor(0x2506, 0x0003);
HI551_write_cmos_sensor(0x2508, 0x0C01);
HI551_write_cmos_sensor(0x250a, 0x004B);
HI551_write_cmos_sensor(0x250c, 0x0C17);
HI551_write_cmos_sensor(0x250e, 0x0043);
HI551_write_cmos_sensor(0x2510, 0x0C01);
HI551_write_cmos_sensor(0x2512, 0x0672);
HI551_write_cmos_sensor(0x2514, 0x0243);
HI551_write_cmos_sensor(0x2516, 0x79DF);
HI551_write_cmos_sensor(0x2518, 0x0C3B);
HI551_write_cmos_sensor(0x251a, 0x0647);
HI551_write_cmos_sensor(0x251c, 0x0C05);
HI551_write_cmos_sensor(0x251e, 0x0672);
HI551_write_cmos_sensor(0x2520, 0x0C01);
HI551_write_cmos_sensor(0x2522, 0x0670);
HI551_write_cmos_sensor(0x2524, 0x0C0D);
HI551_write_cmos_sensor(0x2526, 0x0778);
HI551_write_cmos_sensor(0x2528, 0x0C0B);
HI551_write_cmos_sensor(0x252a, 0x067C);
HI551_write_cmos_sensor(0x252c, 0x0C13);
HI551_write_cmos_sensor(0x252e, 0x0686);
HI551_write_cmos_sensor(0x2530, 0x0C01);
HI551_write_cmos_sensor(0x2532, 0x066C);
HI551_write_cmos_sensor(0x2534, 0x0C67);
HI551_write_cmos_sensor(0x2536, 0x067C);
HI551_write_cmos_sensor(0x2538, 0x0C01);
HI551_write_cmos_sensor(0x253a, 0x0003);
HI551_write_cmos_sensor(0x253c, 0x0393);
HI551_write_cmos_sensor(0x253e, 0x7A1F);
HI551_write_cmos_sensor(0x2540, 0x0C17);
HI551_write_cmos_sensor(0x2542, 0x0083);
HI551_write_cmos_sensor(0x2544, 0x0C6F);
HI551_write_cmos_sensor(0x2546, 0x0686);
HI551_write_cmos_sensor(0x2548, 0x0C01);
HI551_write_cmos_sensor(0x254a, 0x0664);
HI551_write_cmos_sensor(0x254c, 0x12B0);
HI551_write_cmos_sensor(0x254e, 0xF776);
HI551_write_cmos_sensor(0x2550, 0x930F);
HI551_write_cmos_sensor(0x2552, 0x2405);
HI551_write_cmos_sensor(0x2554, 0x4292);
HI551_write_cmos_sensor(0x2556, 0x8118);
HI551_write_cmos_sensor(0x2558, 0x8128);
HI551_write_cmos_sensor(0x255a, 0x4382);
HI551_write_cmos_sensor(0x255c, 0x8118);
HI551_write_cmos_sensor(0x255e, 0x9382);
HI551_write_cmos_sensor(0x2560, 0x8138);
HI551_write_cmos_sensor(0x2562, 0x2419);
HI551_write_cmos_sensor(0x2564, 0x0B00);
HI551_write_cmos_sensor(0x2566, 0x7302);
HI551_write_cmos_sensor(0x2568, 0x069E);
HI551_write_cmos_sensor(0x256a, 0x0674);
HI551_write_cmos_sensor(0x256c, 0x0C02);
HI551_write_cmos_sensor(0x256e, 0x0339);
HI551_write_cmos_sensor(0x2570, 0xB810);
HI551_write_cmos_sensor(0x2572, 0x0C01);
HI551_write_cmos_sensor(0x2574, 0x003C);
HI551_write_cmos_sensor(0x2576, 0x0C01);
HI551_write_cmos_sensor(0x2578, 0x0004);
HI551_write_cmos_sensor(0x257a, 0x0C15);
HI551_write_cmos_sensor(0x257c, 0x06A5);
HI551_write_cmos_sensor(0x257e, 0x0C05);
HI551_write_cmos_sensor(0x2580, 0x06A1);
HI551_write_cmos_sensor(0x2582, 0x9382);
HI551_write_cmos_sensor(0x2584, 0x8144);
HI551_write_cmos_sensor(0x2586, 0x273E);
HI551_write_cmos_sensor(0x2588, 0x43C2);
HI551_write_cmos_sensor(0x258a, 0x0A80);
HI551_write_cmos_sensor(0x258c, 0x0B00);
HI551_write_cmos_sensor(0x258e, 0x7302);
HI551_write_cmos_sensor(0x2590, 0xFFF0);
HI551_write_cmos_sensor(0x2592, 0x4030);
HI551_write_cmos_sensor(0x2594, 0xF218);
HI551_write_cmos_sensor(0x2596, 0x0B00);
HI551_write_cmos_sensor(0x2598, 0x7302);
HI551_write_cmos_sensor(0x259a, 0x069E);
HI551_write_cmos_sensor(0x259c, 0x0674);
HI551_write_cmos_sensor(0x259e, 0x0C02);
HI551_write_cmos_sensor(0x25a0, 0x0301);
HI551_write_cmos_sensor(0x25a2, 0xB810);
HI551_write_cmos_sensor(0x25a4, 0x0C01);
HI551_write_cmos_sensor(0x25a6, 0x0004);
HI551_write_cmos_sensor(0x25a8, 0x0C17);
HI551_write_cmos_sensor(0x25aa, 0x06A5);
HI551_write_cmos_sensor(0x25ac, 0x0C05);
HI551_write_cmos_sensor(0x25ae, 0x06A1);
HI551_write_cmos_sensor(0x25b0, 0x3FE8);
HI551_write_cmos_sensor(0x25b2, 0x0B00);
HI551_write_cmos_sensor(0x25b4, 0x7302);
HI551_write_cmos_sensor(0x25b6, 0x033A);
HI551_write_cmos_sensor(0x25b8, 0x0674);
HI551_write_cmos_sensor(0x25ba, 0x0C02);
HI551_write_cmos_sensor(0x25bc, 0x0301);
HI551_write_cmos_sensor(0x25be, 0xB810);
HI551_write_cmos_sensor(0x25c0, 0x0C01);
HI551_write_cmos_sensor(0x25c2, 0x0004);
HI551_write_cmos_sensor(0x25c4, 0x3F99);
HI551_write_cmos_sensor(0x25c6, 0xB3E2);
HI551_write_cmos_sensor(0x25c8, 0x00B4);
HI551_write_cmos_sensor(0x25ca, 0x2002);
HI551_write_cmos_sensor(0x25cc, 0x4030);
HI551_write_cmos_sensor(0x25ce, 0xF1B0);
HI551_write_cmos_sensor(0x25d0, 0x4318);
HI551_write_cmos_sensor(0x25d2, 0x4030);
HI551_write_cmos_sensor(0x25d4, 0xF1B0);
HI551_write_cmos_sensor(0x25d6, 0x4392);
HI551_write_cmos_sensor(0x25d8, 0x760E);
HI551_write_cmos_sensor(0x25da, 0x425F);
HI551_write_cmos_sensor(0x25dc, 0x0198);
HI551_write_cmos_sensor(0x25de, 0xF37F);
HI551_write_cmos_sensor(0x25e0, 0x930F);
HI551_write_cmos_sensor(0x25e2, 0x2005);
HI551_write_cmos_sensor(0x25e4, 0x43C2);
HI551_write_cmos_sensor(0x25e6, 0x0A80);
HI551_write_cmos_sensor(0x25e8, 0x0B00);
HI551_write_cmos_sensor(0x25ea, 0x7302);
HI551_write_cmos_sensor(0x25ec, 0xFFF0);
HI551_write_cmos_sensor(0x25ee, 0x9382);
HI551_write_cmos_sensor(0x25f0, 0x760C);
HI551_write_cmos_sensor(0x25f2, 0x2007);
HI551_write_cmos_sensor(0x25f4, 0x0C64);
HI551_write_cmos_sensor(0x25f6, 0x4292);
HI551_write_cmos_sensor(0x25f8, 0x7110);
HI551_write_cmos_sensor(0x25fa, 0x1082);
HI551_write_cmos_sensor(0x25fc, 0x53D2);
HI551_write_cmos_sensor(0x25fe, 0x1093);
HI551_write_cmos_sensor(0x2600, 0x3FEC);
HI551_write_cmos_sensor(0x2602, 0x4F82);
HI551_write_cmos_sensor(0x2604, 0x8122);
HI551_write_cmos_sensor(0x2606, 0x12B0);
HI551_write_cmos_sensor(0x2608, 0xFD8C);
HI551_write_cmos_sensor(0x260a, 0x4292);
HI551_write_cmos_sensor(0x260c, 0x760A);
HI551_write_cmos_sensor(0x260e, 0x813A);
HI551_write_cmos_sensor(0x2610, 0x421F);
HI551_write_cmos_sensor(0x2612, 0x813A);
HI551_write_cmos_sensor(0x2614, 0x903F);
HI551_write_cmos_sensor(0x2616, 0x0200);
HI551_write_cmos_sensor(0x2618, 0x2476);
HI551_write_cmos_sensor(0x261a, 0x930F);
HI551_write_cmos_sensor(0x261c, 0x2474);
HI551_write_cmos_sensor(0x261e, 0x903F);
HI551_write_cmos_sensor(0x2620, 0x0100);
HI551_write_cmos_sensor(0x2622, 0x23D9);
HI551_write_cmos_sensor(0x2624, 0x40B2);
HI551_write_cmos_sensor(0x2626, 0x0005);
HI551_write_cmos_sensor(0x2628, 0x7600);
HI551_write_cmos_sensor(0x262a, 0x4382);
HI551_write_cmos_sensor(0x262c, 0x7602);
HI551_write_cmos_sensor(0x262e, 0x0262);
HI551_write_cmos_sensor(0x2630, 0x0000);
HI551_write_cmos_sensor(0x2632, 0x0222);
HI551_write_cmos_sensor(0x2634, 0x0000);
HI551_write_cmos_sensor(0x2636, 0x0262);
HI551_write_cmos_sensor(0x2638, 0x0000);
HI551_write_cmos_sensor(0x263a, 0x0260);
HI551_write_cmos_sensor(0x263c, 0x0000);
HI551_write_cmos_sensor(0x263e, 0x425F);
HI551_write_cmos_sensor(0x2640, 0x0186);
HI551_write_cmos_sensor(0x2642, 0x4F4A);
HI551_write_cmos_sensor(0x2644, 0x4219);
HI551_write_cmos_sensor(0x2646, 0x018A);
HI551_write_cmos_sensor(0x2648, 0x93D2);
HI551_write_cmos_sensor(0x264a, 0x018F);
HI551_write_cmos_sensor(0x264c, 0x245A);
HI551_write_cmos_sensor(0x264e, 0x425F);
HI551_write_cmos_sensor(0x2650, 0x018F);
HI551_write_cmos_sensor(0x2652, 0x4F4B);
HI551_write_cmos_sensor(0x2654, 0x4382);
HI551_write_cmos_sensor(0x2656, 0x8120);
HI551_write_cmos_sensor(0x2658, 0x430C);
HI551_write_cmos_sensor(0x265a, 0x421D);
HI551_write_cmos_sensor(0x265c, 0x8102);
HI551_write_cmos_sensor(0x265e, 0x4C08);
HI551_write_cmos_sensor(0x2660, 0x431F);
HI551_write_cmos_sensor(0x2662, 0x4C0E);
HI551_write_cmos_sensor(0x2664, 0x930E);
HI551_write_cmos_sensor(0x2666, 0x2403);
HI551_write_cmos_sensor(0x2668, 0x5F0F);
HI551_write_cmos_sensor(0x266a, 0x831E);
HI551_write_cmos_sensor(0x266c, 0x23FD);
HI551_write_cmos_sensor(0x266e, 0xFA0F);
HI551_write_cmos_sensor(0x2670, 0x2431);
HI551_write_cmos_sensor(0x2672, 0x430D);
HI551_write_cmos_sensor(0x2674, 0x931B);
HI551_write_cmos_sensor(0x2676, 0x282E);
HI551_write_cmos_sensor(0x2678, 0x4C0E);
HI551_write_cmos_sensor(0x267a, 0x430F);
HI551_write_cmos_sensor(0x267c, 0x4982);
HI551_write_cmos_sensor(0x267e, 0x7600);
HI551_write_cmos_sensor(0x2680, 0x4E82);
HI551_write_cmos_sensor(0x2682, 0x7602);
HI551_write_cmos_sensor(0x2684, 0x4A82);
HI551_write_cmos_sensor(0x2686, 0x7604);
HI551_write_cmos_sensor(0x2688, 0x0264);
HI551_write_cmos_sensor(0x268a, 0x0000);
HI551_write_cmos_sensor(0x268c, 0x0224);
HI551_write_cmos_sensor(0x268e, 0x0000);
HI551_write_cmos_sensor(0x2690, 0x0264);
HI551_write_cmos_sensor(0x2692, 0x0000);
HI551_write_cmos_sensor(0x2694, 0x0260);
HI551_write_cmos_sensor(0x2696, 0x0000);
HI551_write_cmos_sensor(0x2698, 0x0268);
HI551_write_cmos_sensor(0x269a, 0x0000);
HI551_write_cmos_sensor(0x269c, 0x0C5A);
HI551_write_cmos_sensor(0x269e, 0x02E8);
HI551_write_cmos_sensor(0x26a0, 0x0000);
HI551_write_cmos_sensor(0x26a2, 0x0CB5);
HI551_write_cmos_sensor(0x26a4, 0x02A8);
HI551_write_cmos_sensor(0x26a6, 0x0000);
HI551_write_cmos_sensor(0x26a8, 0x0CB5);
HI551_write_cmos_sensor(0x26aa, 0x0CB5);
HI551_write_cmos_sensor(0x26ac, 0x0CB5);
HI551_write_cmos_sensor(0x26ae, 0x0CB5);
HI551_write_cmos_sensor(0x26b0, 0x0CB5);
HI551_write_cmos_sensor(0x26b2, 0x0CB5);
HI551_write_cmos_sensor(0x26b4, 0x0CB5);
HI551_write_cmos_sensor(0x26b6, 0x0CB5);
HI551_write_cmos_sensor(0x26b8, 0x0C00);
HI551_write_cmos_sensor(0x26ba, 0x02E8);
HI551_write_cmos_sensor(0x26bc, 0x0000);
HI551_write_cmos_sensor(0x26be, 0x0CB5);
HI551_write_cmos_sensor(0x26c0, 0x0268);
HI551_write_cmos_sensor(0x26c2, 0x0000);
HI551_write_cmos_sensor(0x26c4, 0x0C5A);
HI551_write_cmos_sensor(0x26c6, 0x0260);
HI551_write_cmos_sensor(0x26c8, 0x0000);
HI551_write_cmos_sensor(0x26ca, 0x0C5A);
HI551_write_cmos_sensor(0x26cc, 0x531F);
HI551_write_cmos_sensor(0x26ce, 0x9B0F);
HI551_write_cmos_sensor(0x26d0, 0x2BD5);
HI551_write_cmos_sensor(0x26d2, 0x4F0D);
HI551_write_cmos_sensor(0x26d4, 0x480C);
HI551_write_cmos_sensor(0x26d6, 0x531C);
HI551_write_cmos_sensor(0x26d8, 0x923C);
HI551_write_cmos_sensor(0x26da, 0x3BC1);
HI551_write_cmos_sensor(0x26dc, 0x4D82);
HI551_write_cmos_sensor(0x26de, 0x8102);
HI551_write_cmos_sensor(0x26e0, 0x4C82);
HI551_write_cmos_sensor(0x26e2, 0x8120);
HI551_write_cmos_sensor(0x26e4, 0x0261);
HI551_write_cmos_sensor(0x26e6, 0x0000);
HI551_write_cmos_sensor(0x26e8, 0x12B0);
HI551_write_cmos_sensor(0x26ea, 0xFD8C);
HI551_write_cmos_sensor(0x26ec, 0x470F);
HI551_write_cmos_sensor(0x26ee, 0x12B0);
HI551_write_cmos_sensor(0x26f0, 0xFD9E);
HI551_write_cmos_sensor(0x26f2, 0x421F);
HI551_write_cmos_sensor(0x26f4, 0x7606);
HI551_write_cmos_sensor(0x26f6, 0x4FC2);
HI551_write_cmos_sensor(0x26f8, 0x0188);
HI551_write_cmos_sensor(0x26fa, 0x4907);
HI551_write_cmos_sensor(0x26fc, 0x0261);
HI551_write_cmos_sensor(0x26fe, 0x0000);
HI551_write_cmos_sensor(0x2700, 0x3F6A);
HI551_write_cmos_sensor(0x2702, 0x432B);
HI551_write_cmos_sensor(0x2704, 0x3FA7);
HI551_write_cmos_sensor(0x2706, 0x421F);
HI551_write_cmos_sensor(0x2708, 0x018A);
HI551_write_cmos_sensor(0x270a, 0x12B0);
HI551_write_cmos_sensor(0x270c, 0xFD9E);
HI551_write_cmos_sensor(0x270e, 0x421F);
HI551_write_cmos_sensor(0x2710, 0x7606);
HI551_write_cmos_sensor(0x2712, 0x4FC2);
HI551_write_cmos_sensor(0x2714, 0x0188);
HI551_write_cmos_sensor(0x2716, 0x0261);
HI551_write_cmos_sensor(0x2718, 0x0000);
HI551_write_cmos_sensor(0x271a, 0x3F5D);
HI551_write_cmos_sensor(0x271c, 0x403F);
HI551_write_cmos_sensor(0x271e, 0x0B80);
HI551_write_cmos_sensor(0x2720, 0x4F2E);
HI551_write_cmos_sensor(0x2722, 0xF03E);
HI551_write_cmos_sensor(0x2724, 0x7FFF);
HI551_write_cmos_sensor(0x2726, 0x4E82);
HI551_write_cmos_sensor(0x2728, 0x0B20);
HI551_write_cmos_sensor(0x272a, 0xF0BF);
HI551_write_cmos_sensor(0x272c, 0x7FFF);
HI551_write_cmos_sensor(0x272e, 0x0000);
HI551_write_cmos_sensor(0x2730, 0x0C0A);
HI551_write_cmos_sensor(0x2732, 0x40B2);
HI551_write_cmos_sensor(0x2734, 0x8084);
HI551_write_cmos_sensor(0x2736, 0x0B20);
HI551_write_cmos_sensor(0x2738, 0x40B2);
HI551_write_cmos_sensor(0x273a, 0x8084);
HI551_write_cmos_sensor(0x273c, 0x0B88);
HI551_write_cmos_sensor(0x273e, 0x0C0A);
HI551_write_cmos_sensor(0x2740, 0x40B2);
HI551_write_cmos_sensor(0x2742, 0xAC48);
HI551_write_cmos_sensor(0x2744, 0x0B20);
HI551_write_cmos_sensor(0x2746, 0x40B2);
HI551_write_cmos_sensor(0x2748, 0xAC48);
HI551_write_cmos_sensor(0x274a, 0x0B8A);
HI551_write_cmos_sensor(0x274c, 0x0C0A);
HI551_write_cmos_sensor(0x274e, 0x40B2);
HI551_write_cmos_sensor(0x2750, 0x01BC);
HI551_write_cmos_sensor(0x2752, 0x0B20);
HI551_write_cmos_sensor(0x2754, 0x40B2);
HI551_write_cmos_sensor(0x2756, 0x01BC);
HI551_write_cmos_sensor(0x2758, 0x0B8C);
HI551_write_cmos_sensor(0x275a, 0x0C0A);
HI551_write_cmos_sensor(0x275c, 0x40B2);
HI551_write_cmos_sensor(0x275e, 0x0500);
HI551_write_cmos_sensor(0x2760, 0x0B20);
HI551_write_cmos_sensor(0x2762, 0x40B2);
HI551_write_cmos_sensor(0x2764, 0x0500);
HI551_write_cmos_sensor(0x2766, 0x0B9E);
HI551_write_cmos_sensor(0x2768, 0x0C0A);
HI551_write_cmos_sensor(0x276a, 0x43C2);
HI551_write_cmos_sensor(0x276c, 0x003F);
HI551_write_cmos_sensor(0x276e, 0x4030);
HI551_write_cmos_sensor(0x2770, 0xF042);
HI551_write_cmos_sensor(0x2772, 0x4030);
HI551_write_cmos_sensor(0x2774, 0xFDBA);
HI551_write_cmos_sensor(0x2776, 0xE3B2);
HI551_write_cmos_sensor(0x2778, 0x740E);
HI551_write_cmos_sensor(0x277a, 0x425F);
HI551_write_cmos_sensor(0x277c, 0x0198);
HI551_write_cmos_sensor(0x277e, 0xF37F);
HI551_write_cmos_sensor(0x2780, 0x4F82);
HI551_write_cmos_sensor(0x2782, 0x8122);
HI551_write_cmos_sensor(0x2784, 0x930F);
HI551_write_cmos_sensor(0x2786, 0x2005);
HI551_write_cmos_sensor(0x2788, 0x93C2);
HI551_write_cmos_sensor(0x278a, 0x0A82);
HI551_write_cmos_sensor(0x278c, 0x2402);
HI551_write_cmos_sensor(0x278e, 0x4392);
HI551_write_cmos_sensor(0x2790, 0x8144);
HI551_write_cmos_sensor(0x2792, 0x9382);
HI551_write_cmos_sensor(0x2794, 0x8122);
HI551_write_cmos_sensor(0x2796, 0x2002);
HI551_write_cmos_sensor(0x2798, 0x4392);
HI551_write_cmos_sensor(0x279a, 0x8104);
HI551_write_cmos_sensor(0x279c, 0x421F);
HI551_write_cmos_sensor(0x279e, 0x710E);
HI551_write_cmos_sensor(0x27a0, 0x93A2);
HI551_write_cmos_sensor(0x27a2, 0x7110);
HI551_write_cmos_sensor(0x27a4, 0x2411);
HI551_write_cmos_sensor(0x27a6, 0x9382);
HI551_write_cmos_sensor(0x27a8, 0x710E);
HI551_write_cmos_sensor(0x27aa, 0x240C);
HI551_write_cmos_sensor(0x27ac, 0x5292);
HI551_write_cmos_sensor(0x27ae, 0x812C);
HI551_write_cmos_sensor(0x27b0, 0x7110);
HI551_write_cmos_sensor(0x27b2, 0x4382);
HI551_write_cmos_sensor(0x27b4, 0x740E);
HI551_write_cmos_sensor(0x27b6, 0xB3E2);
HI551_write_cmos_sensor(0x27b8, 0x0080);
HI551_write_cmos_sensor(0x27ba, 0x2402);
HI551_write_cmos_sensor(0x27bc, 0x4392);
HI551_write_cmos_sensor(0x27be, 0x740E);
HI551_write_cmos_sensor(0x27c0, 0x4392);
HI551_write_cmos_sensor(0x27c2, 0x8138);
HI551_write_cmos_sensor(0x27c4, 0x430F);
HI551_write_cmos_sensor(0x27c6, 0x4130);
HI551_write_cmos_sensor(0x27c8, 0xF31F);
HI551_write_cmos_sensor(0x27ca, 0x27ED);
HI551_write_cmos_sensor(0x27cc, 0x40B2);
HI551_write_cmos_sensor(0x27ce, 0x0003);
HI551_write_cmos_sensor(0x27d0, 0x7110);
HI551_write_cmos_sensor(0x27d2, 0x431F);
HI551_write_cmos_sensor(0x27d4, 0x4130);
HI551_write_cmos_sensor(0x27d6, 0x4F0E);
HI551_write_cmos_sensor(0x27d8, 0x421D);
HI551_write_cmos_sensor(0x27da, 0x8104);
HI551_write_cmos_sensor(0x27dc, 0x425F);
HI551_write_cmos_sensor(0x27de, 0x0198);
HI551_write_cmos_sensor(0x27e0, 0xF37F);
HI551_write_cmos_sensor(0x27e2, 0x903E);
HI551_write_cmos_sensor(0x27e4, 0x0003);
HI551_write_cmos_sensor(0x27e6, 0x2403);
HI551_write_cmos_sensor(0x27e8, 0x0B00);
HI551_write_cmos_sensor(0x27ea, 0x7302);
HI551_write_cmos_sensor(0x27ec, 0x03E8);
HI551_write_cmos_sensor(0x27ee, 0x930F);
HI551_write_cmos_sensor(0x27f0, 0x2402);
HI551_write_cmos_sensor(0x27f2, 0x930D);
HI551_write_cmos_sensor(0x27f4, 0x2403);
HI551_write_cmos_sensor(0x27f6, 0x9392);
HI551_write_cmos_sensor(0x27f8, 0x7110);
HI551_write_cmos_sensor(0x27fa, 0x2018);
HI551_write_cmos_sensor(0x27fc, 0x9382);
HI551_write_cmos_sensor(0x27fe, 0x7308);
HI551_write_cmos_sensor(0x2800, 0x2402);
HI551_write_cmos_sensor(0x2802, 0x930E);
HI551_write_cmos_sensor(0x2804, 0x2419);
HI551_write_cmos_sensor(0x2806, 0x9382);
HI551_write_cmos_sensor(0x2808, 0x7328);
HI551_write_cmos_sensor(0x280a, 0x2402);
HI551_write_cmos_sensor(0x280c, 0x931E);
HI551_write_cmos_sensor(0x280e, 0x2414);
HI551_write_cmos_sensor(0x2810, 0x9382);
HI551_write_cmos_sensor(0x2812, 0x710E);
HI551_write_cmos_sensor(0x2814, 0x2402);
HI551_write_cmos_sensor(0x2816, 0x932E);
HI551_write_cmos_sensor(0x2818, 0x240F);
HI551_write_cmos_sensor(0x281a, 0x9382);
HI551_write_cmos_sensor(0x281c, 0x7114);
HI551_write_cmos_sensor(0x281e, 0x2402);
HI551_write_cmos_sensor(0x2820, 0x922E);
HI551_write_cmos_sensor(0x2822, 0x240A);
HI551_write_cmos_sensor(0x2824, 0x903E);
HI551_write_cmos_sensor(0x2826, 0x0003);
HI551_write_cmos_sensor(0x2828, 0x23D9);
HI551_write_cmos_sensor(0x282a, 0x3C06);
HI551_write_cmos_sensor(0x282c, 0x43C2);
HI551_write_cmos_sensor(0x282e, 0x0A80);
HI551_write_cmos_sensor(0x2830, 0x0B00);
HI551_write_cmos_sensor(0x2832, 0x7302);
HI551_write_cmos_sensor(0x2834, 0xFFF0);
HI551_write_cmos_sensor(0x2836, 0x3FD2);
HI551_write_cmos_sensor(0x2838, 0x4F82);
HI551_write_cmos_sensor(0x283a, 0x8122);
HI551_write_cmos_sensor(0x283c, 0x431F);
HI551_write_cmos_sensor(0x283e, 0x4130);
HI551_write_cmos_sensor(0x2840, 0x120B);
HI551_write_cmos_sensor(0x2842, 0x120A);
HI551_write_cmos_sensor(0x2844, 0x1209);
HI551_write_cmos_sensor(0x2846, 0x1208);
HI551_write_cmos_sensor(0x2848, 0x1207);
HI551_write_cmos_sensor(0x284a, 0x1206);
HI551_write_cmos_sensor(0x284c, 0x403B);
HI551_write_cmos_sensor(0x284e, 0x000E);
HI551_write_cmos_sensor(0x2850, 0x510B);
HI551_write_cmos_sensor(0x2852, 0x4C09);
HI551_write_cmos_sensor(0x2854, 0x4D0A);
HI551_write_cmos_sensor(0x2856, 0x4B26);
HI551_write_cmos_sensor(0x2858, 0x5E0E);
HI551_write_cmos_sensor(0x285a, 0x6F0F);
HI551_write_cmos_sensor(0x285c, 0x5E0E);
HI551_write_cmos_sensor(0x285e, 0x6F0F);
HI551_write_cmos_sensor(0x2860, 0x5E0E);
HI551_write_cmos_sensor(0x2862, 0x6F0F);
HI551_write_cmos_sensor(0x2864, 0x5E0E);
HI551_write_cmos_sensor(0x2866, 0x6F0F);
HI551_write_cmos_sensor(0x2868, 0x4E0C);
HI551_write_cmos_sensor(0x286a, 0x4F0D);
HI551_write_cmos_sensor(0x286c, 0x4A0B);
HI551_write_cmos_sensor(0x286e, 0x490A);
HI551_write_cmos_sensor(0x2870, 0x12B0);
HI551_write_cmos_sensor(0x2872, 0xFDF4);
HI551_write_cmos_sensor(0x2874, 0x4C0F);
HI551_write_cmos_sensor(0x2876, 0x9382);
HI551_write_cmos_sensor(0x2878, 0x811E);
HI551_write_cmos_sensor(0x287a, 0x2031);
HI551_write_cmos_sensor(0x287c, 0x9382);
HI551_write_cmos_sensor(0x287e, 0x8116);
HI551_write_cmos_sensor(0x2880, 0x202E);
HI551_write_cmos_sensor(0x2882, 0x9382);
HI551_write_cmos_sensor(0x2884, 0x8118);
HI551_write_cmos_sensor(0x2886, 0x202B);
HI551_write_cmos_sensor(0x2888, 0x9382);
HI551_write_cmos_sensor(0x288a, 0x8126);
HI551_write_cmos_sensor(0x288c, 0x2028);
HI551_write_cmos_sensor(0x288e, 0x4217);
HI551_write_cmos_sensor(0x2890, 0x8140);
HI551_write_cmos_sensor(0x2892, 0x4C0A);
HI551_write_cmos_sensor(0x2894, 0x470C);
HI551_write_cmos_sensor(0x2896, 0x430B);
HI551_write_cmos_sensor(0x2898, 0x430D);
HI551_write_cmos_sensor(0x289a, 0x12B0);
HI551_write_cmos_sensor(0x289c, 0xFDD4);
HI551_write_cmos_sensor(0x289e, 0x4E08);
HI551_write_cmos_sensor(0x28a0, 0x4F09);
HI551_write_cmos_sensor(0x28a2, 0x403F);
HI551_write_cmos_sensor(0x28a4, 0x0100);
HI551_write_cmos_sensor(0x28a6, 0x870F);
HI551_write_cmos_sensor(0x28a8, 0x460A);
HI551_write_cmos_sensor(0x28aa, 0x4F0C);
HI551_write_cmos_sensor(0x28ac, 0x430B);
HI551_write_cmos_sensor(0x28ae, 0x430D);
HI551_write_cmos_sensor(0x28b0, 0x12B0);
HI551_write_cmos_sensor(0x28b2, 0xFDD4);
HI551_write_cmos_sensor(0x28b4, 0x5E08);
HI551_write_cmos_sensor(0x28b6, 0x6F09);
HI551_write_cmos_sensor(0x28b8, 0x1088);
HI551_write_cmos_sensor(0x28ba, 0x1089);
HI551_write_cmos_sensor(0x28bc, 0xE948);
HI551_write_cmos_sensor(0x28be, 0xE908);
HI551_write_cmos_sensor(0x28c0, 0xF379);
HI551_write_cmos_sensor(0x28c2, 0x480F);
HI551_write_cmos_sensor(0x28c4, 0x421E);
HI551_write_cmos_sensor(0x28c6, 0x8106);
HI551_write_cmos_sensor(0x28c8, 0x980E);
HI551_write_cmos_sensor(0x28ca, 0x2C05);
HI551_write_cmos_sensor(0x28cc, 0x480D);
HI551_write_cmos_sensor(0x28ce, 0x8E0D);
HI551_write_cmos_sensor(0x28d0, 0x4D82);
HI551_write_cmos_sensor(0x28d2, 0x813C);
HI551_write_cmos_sensor(0x28d4, 0x3C09);
HI551_write_cmos_sensor(0x28d6, 0x880E);
HI551_write_cmos_sensor(0x28d8, 0x4E82);
HI551_write_cmos_sensor(0x28da, 0x813C);
HI551_write_cmos_sensor(0x28dc, 0x3C05);
HI551_write_cmos_sensor(0x28de, 0x4F82);
HI551_write_cmos_sensor(0x28e0, 0x8106);
HI551_write_cmos_sensor(0x28e2, 0x40B2);
HI551_write_cmos_sensor(0x28e4, 0x00FF);
HI551_write_cmos_sensor(0x28e6, 0x813C);
HI551_write_cmos_sensor(0x28e8, 0x4136);
HI551_write_cmos_sensor(0x28ea, 0x4137);
HI551_write_cmos_sensor(0x28ec, 0x4138);
HI551_write_cmos_sensor(0x28ee, 0x4139);
HI551_write_cmos_sensor(0x28f0, 0x413A);
HI551_write_cmos_sensor(0x28f2, 0x413B);
HI551_write_cmos_sensor(0x28f4, 0x4130);
HI551_write_cmos_sensor(0x28f6, 0x120A);
HI551_write_cmos_sensor(0x28f8, 0x4E0D);
HI551_write_cmos_sensor(0x28fa, 0x421A);
HI551_write_cmos_sensor(0x28fc, 0x8114);
HI551_write_cmos_sensor(0x28fe, 0x4F0C);
HI551_write_cmos_sensor(0x2900, 0x12B0);
HI551_write_cmos_sensor(0x2902, 0xFDBE);
HI551_write_cmos_sensor(0x2904, 0x4E0F);
HI551_write_cmos_sensor(0x2906, 0xC312);
HI551_write_cmos_sensor(0x2908, 0x100F);
HI551_write_cmos_sensor(0x290a, 0x110F);
HI551_write_cmos_sensor(0x290c, 0x110F);
HI551_write_cmos_sensor(0x290e, 0x110F);
HI551_write_cmos_sensor(0x2910, 0x4F0A);
HI551_write_cmos_sensor(0x2912, 0x4D0C);
HI551_write_cmos_sensor(0x2914, 0x12B0);
HI551_write_cmos_sensor(0x2916, 0xFDBE);
HI551_write_cmos_sensor(0x2918, 0x4E0F);
HI551_write_cmos_sensor(0x291a, 0xC312);
HI551_write_cmos_sensor(0x291c, 0x100F);
HI551_write_cmos_sensor(0x291e, 0x110F);
HI551_write_cmos_sensor(0x2920, 0x110F);
HI551_write_cmos_sensor(0x2922, 0x110F);
HI551_write_cmos_sensor(0x2924, 0x413A);
HI551_write_cmos_sensor(0x2926, 0x4130);
HI551_write_cmos_sensor(0x2928, 0x120B);
HI551_write_cmos_sensor(0x292a, 0x120A);
HI551_write_cmos_sensor(0x292c, 0x1209);
HI551_write_cmos_sensor(0x292e, 0x1208);
HI551_write_cmos_sensor(0x2930, 0x4292);
HI551_write_cmos_sensor(0x2932, 0x0190);
HI551_write_cmos_sensor(0x2934, 0x0A84);
HI551_write_cmos_sensor(0x2936, 0x4292);
HI551_write_cmos_sensor(0x2938, 0x0192);
HI551_write_cmos_sensor(0x293a, 0x0A92);
HI551_write_cmos_sensor(0x293c, 0x4292);
HI551_write_cmos_sensor(0x293e, 0x0194);
HI551_write_cmos_sensor(0x2940, 0x0A94);
HI551_write_cmos_sensor(0x2942, 0x425F);
HI551_write_cmos_sensor(0x2944, 0x008C);
HI551_write_cmos_sensor(0x2946, 0x4FC2);
HI551_write_cmos_sensor(0x2948, 0x810C);
HI551_write_cmos_sensor(0x294a, 0x43C2);
HI551_write_cmos_sensor(0x294c, 0x810D);
HI551_write_cmos_sensor(0x294e, 0x4382);
HI551_write_cmos_sensor(0x2950, 0x0384);
HI551_write_cmos_sensor(0x2952, 0x40B2);
HI551_write_cmos_sensor(0x2954, 0x0020);
HI551_write_cmos_sensor(0x2956, 0x0386);
HI551_write_cmos_sensor(0x2958, 0xB3D2);
HI551_write_cmos_sensor(0x295a, 0x0080);
HI551_write_cmos_sensor(0x295c, 0x2403);
HI551_write_cmos_sensor(0x295e, 0x40B2);
HI551_write_cmos_sensor(0x2960, 0x8006);
HI551_write_cmos_sensor(0x2962, 0x0384);
HI551_write_cmos_sensor(0x2964, 0x4392);
HI551_write_cmos_sensor(0x2966, 0x8138);
HI551_write_cmos_sensor(0x2968, 0x4382);
HI551_write_cmos_sensor(0x296a, 0x740E);
HI551_write_cmos_sensor(0x296c, 0xB3E2);
HI551_write_cmos_sensor(0x296e, 0x0080);
HI551_write_cmos_sensor(0x2970, 0x2402);
HI551_write_cmos_sensor(0x2972, 0x4392);
HI551_write_cmos_sensor(0x2974, 0x740E);
HI551_write_cmos_sensor(0x2976, 0x40F2);
HI551_write_cmos_sensor(0x2978, 0x001E);
HI551_write_cmos_sensor(0x297a, 0x0C92);
HI551_write_cmos_sensor(0x297c, 0xD0F2);
HI551_write_cmos_sensor(0x297e, 0x0006);
HI551_write_cmos_sensor(0x2980, 0x0C81);
HI551_write_cmos_sensor(0x2982, 0x90F2);
HI551_write_cmos_sensor(0x2984, 0x0010);
HI551_write_cmos_sensor(0x2986, 0x00BA);
HI551_write_cmos_sensor(0x2988, 0x2DE7);
HI551_write_cmos_sensor(0x298a, 0x403F);
HI551_write_cmos_sensor(0x298c, 0x001C);
HI551_write_cmos_sensor(0x298e, 0x108F);
HI551_write_cmos_sensor(0x2990, 0x5F0F);
HI551_write_cmos_sensor(0x2992, 0x5F0F);
HI551_write_cmos_sensor(0x2994, 0x5F0F);
HI551_write_cmos_sensor(0x2996, 0x403E);
HI551_write_cmos_sensor(0x2998, 0x0B88);
HI551_write_cmos_sensor(0x299a, 0x4E2D);
HI551_write_cmos_sensor(0x299c, 0xF03D);
HI551_write_cmos_sensor(0x299e, 0x07FF);
HI551_write_cmos_sensor(0x29a0, 0xDF0D);
HI551_write_cmos_sensor(0x29a2, 0x4D82);
HI551_write_cmos_sensor(0x29a4, 0x0B20);
HI551_write_cmos_sensor(0x29a6, 0x4E2D);
HI551_write_cmos_sensor(0x29a8, 0xF03D);
HI551_write_cmos_sensor(0x29aa, 0x07FF);
HI551_write_cmos_sensor(0x29ac, 0xDD0F);
HI551_write_cmos_sensor(0x29ae, 0x4F8E);
HI551_write_cmos_sensor(0x29b0, 0x0000);
HI551_write_cmos_sensor(0x29b2, 0x0C0A);
HI551_write_cmos_sensor(0x29b4, 0x403B);
HI551_write_cmos_sensor(0x29b6, 0x00BA);
HI551_write_cmos_sensor(0x29b8, 0x4B6F);
HI551_write_cmos_sensor(0x29ba, 0xF37F);
HI551_write_cmos_sensor(0x29bc, 0x503F);
HI551_write_cmos_sensor(0x29be, 0x0010);
HI551_write_cmos_sensor(0x29c0, 0x4F82);
HI551_write_cmos_sensor(0x29c2, 0x8114);
HI551_write_cmos_sensor(0x29c4, 0x425F);
HI551_write_cmos_sensor(0x29c6, 0x0C87);
HI551_write_cmos_sensor(0x29c8, 0x4F4E);
HI551_write_cmos_sensor(0x29ca, 0x425F);
HI551_write_cmos_sensor(0x29cc, 0x0C88);
HI551_write_cmos_sensor(0x29ce, 0xF37F);
HI551_write_cmos_sensor(0x29d0, 0x12B0);
HI551_write_cmos_sensor(0x29d2, 0xF8F6);
HI551_write_cmos_sensor(0x29d4, 0x4F82);
HI551_write_cmos_sensor(0x29d6, 0x0C8C);
HI551_write_cmos_sensor(0x29d8, 0x425F);
HI551_write_cmos_sensor(0x29da, 0x0C85);
HI551_write_cmos_sensor(0x29dc, 0x4F4E);
HI551_write_cmos_sensor(0x29de, 0x425F);
HI551_write_cmos_sensor(0x29e0, 0x0C89);
HI551_write_cmos_sensor(0x29e2, 0xF37F);
HI551_write_cmos_sensor(0x29e4, 0x12B0);
HI551_write_cmos_sensor(0x29e6, 0xF8F6);
HI551_write_cmos_sensor(0x29e8, 0x4F82);
HI551_write_cmos_sensor(0x29ea, 0x0C8A);
HI551_write_cmos_sensor(0x29ec, 0x425D);
HI551_write_cmos_sensor(0x29ee, 0x00B7);
HI551_write_cmos_sensor(0x29f0, 0x5D4D);
HI551_write_cmos_sensor(0x29f2, 0x4DC2);
HI551_write_cmos_sensor(0x29f4, 0x0CB0);
HI551_write_cmos_sensor(0x29f6, 0x425E);
HI551_write_cmos_sensor(0x29f8, 0x00B8);
HI551_write_cmos_sensor(0x29fa, 0x5E4E);
HI551_write_cmos_sensor(0x29fc, 0x4EC2);
HI551_write_cmos_sensor(0x29fe, 0x0CB1);
HI551_write_cmos_sensor(0x2a00, 0x4B6F);
HI551_write_cmos_sensor(0x2a02, 0xF37F);
HI551_write_cmos_sensor(0x2a04, 0x108F);
HI551_write_cmos_sensor(0x2a06, 0x403D);
HI551_write_cmos_sensor(0x2a08, 0x0B90);
HI551_write_cmos_sensor(0x2a0a, 0x4D2E);
HI551_write_cmos_sensor(0x2a0c, 0xF37E);
HI551_write_cmos_sensor(0x2a0e, 0xDE0F);
HI551_write_cmos_sensor(0x2a10, 0x4F82);
HI551_write_cmos_sensor(0x2a12, 0x0B20);
HI551_write_cmos_sensor(0x2a14, 0x4B6F);
HI551_write_cmos_sensor(0x2a16, 0xF37F);
HI551_write_cmos_sensor(0x2a18, 0x108F);
HI551_write_cmos_sensor(0x2a1a, 0x4D2E);
HI551_write_cmos_sensor(0x2a1c, 0xF37E);
HI551_write_cmos_sensor(0x2a1e, 0xDE0F);
HI551_write_cmos_sensor(0x2a20, 0x4F8D);
HI551_write_cmos_sensor(0x2a22, 0x0000);
HI551_write_cmos_sensor(0x2a24, 0x0C0A);
HI551_write_cmos_sensor(0x2a26, 0x425F);
HI551_write_cmos_sensor(0x2a28, 0x009E);
HI551_write_cmos_sensor(0x2a2a, 0x4F4A);
HI551_write_cmos_sensor(0x2a2c, 0x425F);
HI551_write_cmos_sensor(0x2a2e, 0x009F);
HI551_write_cmos_sensor(0x2a30, 0xF37F);
HI551_write_cmos_sensor(0x2a32, 0x5F0A);
HI551_write_cmos_sensor(0x2a34, 0x110A);
HI551_write_cmos_sensor(0x2a36, 0x110A);
HI551_write_cmos_sensor(0x2a38, 0x425F);
HI551_write_cmos_sensor(0x2a3a, 0x00B2);
HI551_write_cmos_sensor(0x2a3c, 0x4F4B);
HI551_write_cmos_sensor(0x2a3e, 0x425F);
HI551_write_cmos_sensor(0x2a40, 0x00B3);
HI551_write_cmos_sensor(0x2a42, 0xF37F);
HI551_write_cmos_sensor(0x2a44, 0x5F0B);
HI551_write_cmos_sensor(0x2a46, 0x110B);
HI551_write_cmos_sensor(0x2a48, 0x110B);
HI551_write_cmos_sensor(0x2a4a, 0x4A0E);
HI551_write_cmos_sensor(0x2a4c, 0x5E0E);
HI551_write_cmos_sensor(0x2a4e, 0x5E0E);
HI551_write_cmos_sensor(0x2a50, 0x5E0E);
HI551_write_cmos_sensor(0x2a52, 0x5E0E);
HI551_write_cmos_sensor(0x2a54, 0x4B0F);
HI551_write_cmos_sensor(0x2a56, 0x5F0F);
HI551_write_cmos_sensor(0x2a58, 0x5F0F);
HI551_write_cmos_sensor(0x2a5a, 0x5F0F);
HI551_write_cmos_sensor(0x2a5c, 0x5F0F);
HI551_write_cmos_sensor(0x2a5e, 0x5F0F);
HI551_write_cmos_sensor(0x2a60, 0x5F0F);
HI551_write_cmos_sensor(0x2a62, 0x5F0F);
HI551_write_cmos_sensor(0x2a64, 0xDF0E);
HI551_write_cmos_sensor(0x2a66, 0x4E82);
HI551_write_cmos_sensor(0x2a68, 0x0A8E);
HI551_write_cmos_sensor(0x2a6a, 0x403E);
HI551_write_cmos_sensor(0x2a6c, 0x00BD);
HI551_write_cmos_sensor(0x2a6e, 0x4E6F);
HI551_write_cmos_sensor(0x2a70, 0xF35F);
HI551_write_cmos_sensor(0x2a72, 0x4F0D);
HI551_write_cmos_sensor(0x2a74, 0xF31D);
HI551_write_cmos_sensor(0x2a76, 0x4D82);
HI551_write_cmos_sensor(0x2a78, 0x8108);
HI551_write_cmos_sensor(0x2a7a, 0x4E6F);
HI551_write_cmos_sensor(0x2a7c, 0xF36F);
HI551_write_cmos_sensor(0x2a7e, 0x4F0E);
HI551_write_cmos_sensor(0x2a80, 0xF32E);
HI551_write_cmos_sensor(0x2a82, 0x4E82);
HI551_write_cmos_sensor(0x2a84, 0x8124);
HI551_write_cmos_sensor(0x2a86, 0xB3E2);
HI551_write_cmos_sensor(0x2a88, 0x0080);
HI551_write_cmos_sensor(0x2a8a, 0x2403);
HI551_write_cmos_sensor(0x2a8c, 0x40F2);
HI551_write_cmos_sensor(0x2a8e, 0x0003);
HI551_write_cmos_sensor(0x2a90, 0x00B5);
HI551_write_cmos_sensor(0x2a92, 0x40B2);
HI551_write_cmos_sensor(0x2a94, 0x1001);
HI551_write_cmos_sensor(0x2a96, 0x7500);
HI551_write_cmos_sensor(0x2a98, 0x40B2);
HI551_write_cmos_sensor(0x2a9a, 0x0803);
HI551_write_cmos_sensor(0x2a9c, 0x7502);
HI551_write_cmos_sensor(0x2a9e, 0x40B2);
HI551_write_cmos_sensor(0x2aa0, 0x080F);
HI551_write_cmos_sensor(0x2aa2, 0x7504);
HI551_write_cmos_sensor(0x2aa4, 0x40B2);
HI551_write_cmos_sensor(0x2aa6, 0x3003);
HI551_write_cmos_sensor(0x2aa8, 0x7506);
HI551_write_cmos_sensor(0x2aaa, 0x40B2);
HI551_write_cmos_sensor(0x2aac, 0x0802);
HI551_write_cmos_sensor(0x2aae, 0x7508);
HI551_write_cmos_sensor(0x2ab0, 0x40B2);
HI551_write_cmos_sensor(0x2ab2, 0x0800);
HI551_write_cmos_sensor(0x2ab4, 0x750A);
HI551_write_cmos_sensor(0x2ab6, 0x403F);
HI551_write_cmos_sensor(0x2ab8, 0x752C);
HI551_write_cmos_sensor(0x2aba, 0x40BF);
HI551_write_cmos_sensor(0x2abc, 0x0006);
HI551_write_cmos_sensor(0x2abe, 0x0000);
HI551_write_cmos_sensor(0x2ac0, 0x40B2);
HI551_write_cmos_sensor(0x2ac2, 0x3800);
HI551_write_cmos_sensor(0x2ac4, 0x750C);
HI551_write_cmos_sensor(0x2ac6, 0x40B2);
HI551_write_cmos_sensor(0x2ac8, 0x0801);
HI551_write_cmos_sensor(0x2aca, 0x750E);
HI551_write_cmos_sensor(0x2acc, 0x40B2);
HI551_write_cmos_sensor(0x2ace, 0x080F);
HI551_write_cmos_sensor(0x2ad0, 0x7510);
HI551_write_cmos_sensor(0x2ad2, 0x40B2);
HI551_write_cmos_sensor(0x2ad4, 0x080E);
HI551_write_cmos_sensor(0x2ad6, 0x7512);
HI551_write_cmos_sensor(0x2ad8, 0x40B2);
HI551_write_cmos_sensor(0x2ada, 0x0800);
HI551_write_cmos_sensor(0x2adc, 0x7514);
HI551_write_cmos_sensor(0x2ade, 0x40B2);
HI551_write_cmos_sensor(0x2ae0, 0x1000);
HI551_write_cmos_sensor(0x2ae2, 0x7516);
HI551_write_cmos_sensor(0x2ae4, 0x40B2);
HI551_write_cmos_sensor(0x2ae6, 0x000B);
HI551_write_cmos_sensor(0x2ae8, 0x752E);
HI551_write_cmos_sensor(0x2aea, 0x43BF);
HI551_write_cmos_sensor(0x2aec, 0x0000);
HI551_write_cmos_sensor(0x2aee, 0x43B2);
HI551_write_cmos_sensor(0x2af0, 0x750C);
HI551_write_cmos_sensor(0x2af2, 0x43B2);
HI551_write_cmos_sensor(0x2af4, 0x7530);
HI551_write_cmos_sensor(0x2af6, 0x4382);
HI551_write_cmos_sensor(0x2af8, 0x7532);
HI551_write_cmos_sensor(0x2afa, 0x43A2);
HI551_write_cmos_sensor(0x2afc, 0x7534);
HI551_write_cmos_sensor(0x2afe, 0x403F);
HI551_write_cmos_sensor(0x2b00, 0x0003);
HI551_write_cmos_sensor(0x2b02, 0x12B0);
HI551_write_cmos_sensor(0x2b04, 0xF7D6);
HI551_write_cmos_sensor(0x2b06, 0x421F);
HI551_write_cmos_sensor(0x2b08, 0x0098);
HI551_write_cmos_sensor(0x2b0a, 0x821F);
HI551_write_cmos_sensor(0x2b0c, 0x0092);
HI551_write_cmos_sensor(0x2b0e, 0x531F);
HI551_write_cmos_sensor(0x2b10, 0xC312);
HI551_write_cmos_sensor(0x2b12, 0x100F);
HI551_write_cmos_sensor(0x2b14, 0x421E);
HI551_write_cmos_sensor(0x2b16, 0x00AC);
HI551_write_cmos_sensor(0x2b18, 0x821E);
HI551_write_cmos_sensor(0x2b1a, 0x00A6);
HI551_write_cmos_sensor(0x2b1c, 0x531E);
HI551_write_cmos_sensor(0x2b1e, 0x4A0D);
HI551_write_cmos_sensor(0x2b20, 0x930D);
HI551_write_cmos_sensor(0x2b22, 0x2404);
HI551_write_cmos_sensor(0x2b24, 0xC312);
HI551_write_cmos_sensor(0x2b26, 0x100F);
HI551_write_cmos_sensor(0x2b28, 0x831D);
HI551_write_cmos_sensor(0x2b2a, 0x23FC);
HI551_write_cmos_sensor(0x2b2c, 0x531F);
HI551_write_cmos_sensor(0x2b2e, 0xC31F);
HI551_write_cmos_sensor(0x2b30, 0x4F82);
HI551_write_cmos_sensor(0x2b32, 0x0A86);
HI551_write_cmos_sensor(0x2b34, 0x4B0F);
HI551_write_cmos_sensor(0x2b36, 0x930F);
HI551_write_cmos_sensor(0x2b38, 0x2404);
HI551_write_cmos_sensor(0x2b3a, 0xC312);
HI551_write_cmos_sensor(0x2b3c, 0x100E);
HI551_write_cmos_sensor(0x2b3e, 0x831F);
HI551_write_cmos_sensor(0x2b40, 0x23FC);
HI551_write_cmos_sensor(0x2b42, 0x531E);
HI551_write_cmos_sensor(0x2b44, 0xC31E);
HI551_write_cmos_sensor(0x2b46, 0x4E82);
HI551_write_cmos_sensor(0x2b48, 0x0A88);
HI551_write_cmos_sensor(0x2b4a, 0xB0B2);
HI551_write_cmos_sensor(0x2b4c, 0x0010);
HI551_write_cmos_sensor(0x2b4e, 0x0A84);
HI551_write_cmos_sensor(0x2b50, 0x24FC);
HI551_write_cmos_sensor(0x2b52, 0x421F);
HI551_write_cmos_sensor(0x2b54, 0x068C);
HI551_write_cmos_sensor(0x2b56, 0xC312);
HI551_write_cmos_sensor(0x2b58, 0x100F);
HI551_write_cmos_sensor(0x2b5a, 0x4F82);
HI551_write_cmos_sensor(0x2b5c, 0x0784);
HI551_write_cmos_sensor(0x2b5e, 0x4292);
HI551_write_cmos_sensor(0x2b60, 0x068E);
HI551_write_cmos_sensor(0x2b62, 0x0786);
HI551_write_cmos_sensor(0x2b64, 0xB3D2);
HI551_write_cmos_sensor(0x2b66, 0x0CB6);
HI551_write_cmos_sensor(0x2b68, 0x2417);
HI551_write_cmos_sensor(0x2b6a, 0x421A);
HI551_write_cmos_sensor(0x2b6c, 0x0CB8);
HI551_write_cmos_sensor(0x2b6e, 0x430B);
HI551_write_cmos_sensor(0x2b70, 0x425F);
HI551_write_cmos_sensor(0x2b72, 0x0CBA);
HI551_write_cmos_sensor(0x2b74, 0x4F4E);
HI551_write_cmos_sensor(0x2b76, 0x430F);
HI551_write_cmos_sensor(0x2b78, 0x4E0F);
HI551_write_cmos_sensor(0x2b7a, 0x430E);
HI551_write_cmos_sensor(0x2b7c, 0xDE0A);
HI551_write_cmos_sensor(0x2b7e, 0xDF0B);
HI551_write_cmos_sensor(0x2b80, 0x421F);
HI551_write_cmos_sensor(0x2b82, 0x0CBC);
HI551_write_cmos_sensor(0x2b84, 0x4F0C);
HI551_write_cmos_sensor(0x2b86, 0x430D);
HI551_write_cmos_sensor(0x2b88, 0x421F);
HI551_write_cmos_sensor(0x2b8a, 0x0CBE);
HI551_write_cmos_sensor(0x2b8c, 0x430E);
HI551_write_cmos_sensor(0x2b8e, 0xDE0C);
HI551_write_cmos_sensor(0x2b90, 0xDF0D);
HI551_write_cmos_sensor(0x2b92, 0x12B0);
HI551_write_cmos_sensor(0x2b94, 0xFDF4);
HI551_write_cmos_sensor(0x2b96, 0x4C09);
HI551_write_cmos_sensor(0x2b98, 0xB2A2);
HI551_write_cmos_sensor(0x2b9a, 0x0A84);
HI551_write_cmos_sensor(0x2b9c, 0x2418);
HI551_write_cmos_sensor(0x2b9e, 0x421B);
HI551_write_cmos_sensor(0x2ba0, 0x0A96);
HI551_write_cmos_sensor(0x2ba2, 0xC312);
HI551_write_cmos_sensor(0x2ba4, 0x100B);
HI551_write_cmos_sensor(0x2ba6, 0x110B);
HI551_write_cmos_sensor(0x2ba8, 0x110B);
HI551_write_cmos_sensor(0x2baa, 0x43C2);
HI551_write_cmos_sensor(0x2bac, 0x0A98);
HI551_write_cmos_sensor(0x2bae, 0x4392);
HI551_write_cmos_sensor(0x2bb0, 0x8112);
HI551_write_cmos_sensor(0x2bb2, 0x421D);
HI551_write_cmos_sensor(0x2bb4, 0x8112);
HI551_write_cmos_sensor(0x2bb6, 0x4B0A);
HI551_write_cmos_sensor(0x2bb8, 0x4D0C);
HI551_write_cmos_sensor(0x2bba, 0x12B0);
HI551_write_cmos_sensor(0x2bbc, 0xFDBE);
HI551_write_cmos_sensor(0x2bbe, 0x9E09);
HI551_write_cmos_sensor(0x2bc0, 0x28B9);
HI551_write_cmos_sensor(0x2bc2, 0x531D);
HI551_write_cmos_sensor(0x2bc4, 0x4D82);
HI551_write_cmos_sensor(0x2bc6, 0x8112);
HI551_write_cmos_sensor(0x2bc8, 0x903D);
HI551_write_cmos_sensor(0x2bca, 0x0009);
HI551_write_cmos_sensor(0x2bcc, 0x3BF2);
HI551_write_cmos_sensor(0x2bce, 0xB3D2);
HI551_write_cmos_sensor(0x2bd0, 0x0788);
HI551_write_cmos_sensor(0x2bd2, 0x2427);
HI551_write_cmos_sensor(0x2bd4, 0xB3D2);
HI551_write_cmos_sensor(0x2bd6, 0x07BC);
HI551_write_cmos_sensor(0x2bd8, 0x2424);
HI551_write_cmos_sensor(0x2bda, 0xB0B2);
HI551_write_cmos_sensor(0x2bdc, 0x0020);
HI551_write_cmos_sensor(0x2bde, 0x0A84);
HI551_write_cmos_sensor(0x2be0, 0x2420);
HI551_write_cmos_sensor(0x2be2, 0x43C2);
HI551_write_cmos_sensor(0x2be4, 0x07AD);
HI551_write_cmos_sensor(0x2be6, 0x43C2);
HI551_write_cmos_sensor(0x2be8, 0x07AF);
HI551_write_cmos_sensor(0x2bea, 0x43C2);
HI551_write_cmos_sensor(0x2bec, 0x07B1);
HI551_write_cmos_sensor(0x2bee, 0x421E);
HI551_write_cmos_sensor(0x2bf0, 0x0084);
HI551_write_cmos_sensor(0x2bf2, 0x421F);
HI551_write_cmos_sensor(0x2bf4, 0x07B8);
HI551_write_cmos_sensor(0x2bf6, 0x9F0E);
HI551_write_cmos_sensor(0x2bf8, 0x286F);
HI551_write_cmos_sensor(0x2bfa, 0x43F2);
HI551_write_cmos_sensor(0x2bfc, 0x07AC);
HI551_write_cmos_sensor(0x2bfe, 0x40F2);
HI551_write_cmos_sensor(0x2c00, 0x0053);
HI551_write_cmos_sensor(0x2c02, 0x07AE);
HI551_write_cmos_sensor(0x2c04, 0x40F2);
HI551_write_cmos_sensor(0x2c06, 0x002C);
HI551_write_cmos_sensor(0x2c08, 0x07B0);
HI551_write_cmos_sensor(0x2c0a, 0x40F2);
HI551_write_cmos_sensor(0x2c0c, 0x001F);
HI551_write_cmos_sensor(0x2c0e, 0x07B2);
HI551_write_cmos_sensor(0x2c10, 0x425E);
HI551_write_cmos_sensor(0x2c12, 0x00BA);
HI551_write_cmos_sensor(0x2c14, 0x425F);
HI551_write_cmos_sensor(0x2c16, 0x07B4);
HI551_write_cmos_sensor(0x2c18, 0x9F4E);
HI551_write_cmos_sensor(0x2c1a, 0x2850);
HI551_write_cmos_sensor(0x2c1c, 0x40F2);
HI551_write_cmos_sensor(0x2c1e, 0x0040);
HI551_write_cmos_sensor(0x2c20, 0x0789);
HI551_write_cmos_sensor(0x2c22, 0x403F);
HI551_write_cmos_sensor(0x2c24, 0x7524);
HI551_write_cmos_sensor(0x2c26, 0x4FA2);
HI551_write_cmos_sensor(0x2c28, 0x7528);
HI551_write_cmos_sensor(0x2c2a, 0x429F);
HI551_write_cmos_sensor(0x2c2c, 0x0084);
HI551_write_cmos_sensor(0x2c2e, 0x0000);
HI551_write_cmos_sensor(0x2c30, 0x4292);
HI551_write_cmos_sensor(0x2c32, 0x0088);
HI551_write_cmos_sensor(0x2c34, 0x7316);
HI551_write_cmos_sensor(0x2c36, 0x93C2);
HI551_write_cmos_sensor(0x2c38, 0x008C);
HI551_write_cmos_sensor(0x2c3a, 0x2403);
HI551_write_cmos_sensor(0x2c3c, 0x4292);
HI551_write_cmos_sensor(0x2c3e, 0x008A);
HI551_write_cmos_sensor(0x2c40, 0x7316);
HI551_write_cmos_sensor(0x2c42, 0x430E);
HI551_write_cmos_sensor(0x2c44, 0x421F);
HI551_write_cmos_sensor(0x2c46, 0x0086);
HI551_write_cmos_sensor(0x2c48, 0x503F);
HI551_write_cmos_sensor(0x2c4a, 0xFFFB);
HI551_write_cmos_sensor(0x2c4c, 0x9F82);
HI551_write_cmos_sensor(0x2c4e, 0x0084);
HI551_write_cmos_sensor(0x2c50, 0x2801);
HI551_write_cmos_sensor(0x2c52, 0x431E);
HI551_write_cmos_sensor(0x2c54, 0x4292);
HI551_write_cmos_sensor(0x2c56, 0x0086);
HI551_write_cmos_sensor(0x2c58, 0x7314);
HI551_write_cmos_sensor(0x2c5a, 0x93C2);
HI551_write_cmos_sensor(0x2c5c, 0x00BC);
HI551_write_cmos_sensor(0x2c5e, 0x2008);
HI551_write_cmos_sensor(0x2c60, 0xB31E);
HI551_write_cmos_sensor(0x2c62, 0x2406);
HI551_write_cmos_sensor(0x2c64, 0x421D);
HI551_write_cmos_sensor(0x2c66, 0x0084);
HI551_write_cmos_sensor(0x2c68, 0x503D);
HI551_write_cmos_sensor(0x2c6a, 0x0005);
HI551_write_cmos_sensor(0x2c6c, 0x4D82);
HI551_write_cmos_sensor(0x2c6e, 0x7314);
HI551_write_cmos_sensor(0x2c70, 0x425F);
HI551_write_cmos_sensor(0x2c72, 0x00BC);
HI551_write_cmos_sensor(0x2c74, 0xF37F);
HI551_write_cmos_sensor(0x2c76, 0xFE0F);
HI551_write_cmos_sensor(0x2c78, 0x241B);
HI551_write_cmos_sensor(0x2c7a, 0x421E);
HI551_write_cmos_sensor(0x2c7c, 0x0086);
HI551_write_cmos_sensor(0x2c7e, 0x503E);
HI551_write_cmos_sensor(0x2c80, 0xFFFB);
HI551_write_cmos_sensor(0x2c82, 0x4E82);
HI551_write_cmos_sensor(0x2c84, 0x7524);
HI551_write_cmos_sensor(0x2c86, 0x430E);
HI551_write_cmos_sensor(0x2c88, 0x421F);
HI551_write_cmos_sensor(0x2c8a, 0x7524);
HI551_write_cmos_sensor(0x2c8c, 0x9F82);
HI551_write_cmos_sensor(0x2c8e, 0x7528);
HI551_write_cmos_sensor(0x2c90, 0x2C01);
HI551_write_cmos_sensor(0x2c92, 0x431E);
HI551_write_cmos_sensor(0x2c94, 0x430F);
HI551_write_cmos_sensor(0x2c96, 0x9382);
HI551_write_cmos_sensor(0x2c98, 0x8118);
HI551_write_cmos_sensor(0x2c9a, 0x2001);
HI551_write_cmos_sensor(0x2c9c, 0x431F);
HI551_write_cmos_sensor(0x2c9e, 0xFE0F);
HI551_write_cmos_sensor(0x2ca0, 0x40B2);
HI551_write_cmos_sensor(0x2ca2, 0x001A);
HI551_write_cmos_sensor(0x2ca4, 0x7522);
HI551_write_cmos_sensor(0x2ca6, 0x245B);
HI551_write_cmos_sensor(0x2ca8, 0x40B2);
HI551_write_cmos_sensor(0x2caa, 0x0035);
HI551_write_cmos_sensor(0x2cac, 0x7522);
HI551_write_cmos_sensor(0x2cae, 0x3C57);
HI551_write_cmos_sensor(0x2cb0, 0x9382);
HI551_write_cmos_sensor(0x2cb2, 0x7524);
HI551_write_cmos_sensor(0x2cb4, 0x23E8);
HI551_write_cmos_sensor(0x2cb6, 0x4382);
HI551_write_cmos_sensor(0x2cb8, 0x7524);
HI551_write_cmos_sensor(0x2cba, 0x3FE5);
HI551_write_cmos_sensor(0x2cbc, 0x425E);
HI551_write_cmos_sensor(0x2cbe, 0x00BA);
HI551_write_cmos_sensor(0x2cc0, 0x425F);
HI551_write_cmos_sensor(0x2cc2, 0x07B5);
HI551_write_cmos_sensor(0x2cc4, 0x9F4E);
HI551_write_cmos_sensor(0x2cc6, 0x2804);
HI551_write_cmos_sensor(0x2cc8, 0x40F2);
HI551_write_cmos_sensor(0x2cca, 0x0020);
HI551_write_cmos_sensor(0x2ccc, 0x0789);
HI551_write_cmos_sensor(0x2cce, 0x3FA9);
HI551_write_cmos_sensor(0x2cd0, 0x40F2);
HI551_write_cmos_sensor(0x2cd2, 0x0010);
HI551_write_cmos_sensor(0x2cd4, 0x0789);
HI551_write_cmos_sensor(0x2cd6, 0x3FA5);
HI551_write_cmos_sensor(0x2cd8, 0x421F);
HI551_write_cmos_sensor(0x2cda, 0x0084);
HI551_write_cmos_sensor(0x2cdc, 0x9F82);
HI551_write_cmos_sensor(0x2cde, 0x07BA);
HI551_write_cmos_sensor(0x2ce0, 0x2810);
HI551_write_cmos_sensor(0x2ce2, 0x425F);
HI551_write_cmos_sensor(0x2ce4, 0x00BA);
HI551_write_cmos_sensor(0x2ce6, 0x9FC2);
HI551_write_cmos_sensor(0x2ce8, 0x07B7);
HI551_write_cmos_sensor(0x2cea, 0x280B);
HI551_write_cmos_sensor(0x2cec, 0x42E2);
HI551_write_cmos_sensor(0x2cee, 0x0789);
HI551_write_cmos_sensor(0x2cf0, 0x43C2);
HI551_write_cmos_sensor(0x2cf2, 0x07AC);
HI551_write_cmos_sensor(0x2cf4, 0x43C2);
HI551_write_cmos_sensor(0x2cf6, 0x07AE);
HI551_write_cmos_sensor(0x2cf8, 0x43C2);
HI551_write_cmos_sensor(0x2cfa, 0x07B0);
HI551_write_cmos_sensor(0x2cfc, 0x43C2);
HI551_write_cmos_sensor(0x2cfe, 0x07B2);
HI551_write_cmos_sensor(0x2d00, 0x3F90);
HI551_write_cmos_sensor(0x2d02, 0x421F);
HI551_write_cmos_sensor(0x2d04, 0x07BA);
HI551_write_cmos_sensor(0x2d06, 0x9F82);
HI551_write_cmos_sensor(0x2d08, 0x0084);
HI551_write_cmos_sensor(0x2d0a, 0x2B8B);
HI551_write_cmos_sensor(0x2d0c, 0x421E);
HI551_write_cmos_sensor(0x2d0e, 0x0084);
HI551_write_cmos_sensor(0x2d10, 0x421F);
HI551_write_cmos_sensor(0x2d12, 0x07B8);
HI551_write_cmos_sensor(0x2d14, 0x9F0E);
HI551_write_cmos_sensor(0x2d16, 0x2F85);
HI551_write_cmos_sensor(0x2d18, 0x42F2);
HI551_write_cmos_sensor(0x2d1a, 0x0789);
HI551_write_cmos_sensor(0x2d1c, 0x43F2);
HI551_write_cmos_sensor(0x2d1e, 0x07AC);
HI551_write_cmos_sensor(0x2d20, 0x40F2);
HI551_write_cmos_sensor(0x2d22, 0x0024);
HI551_write_cmos_sensor(0x2d24, 0x07AE);
HI551_write_cmos_sensor(0x2d26, 0x40F2);
HI551_write_cmos_sensor(0x2d28, 0x001A);
HI551_write_cmos_sensor(0x2d2a, 0x07B0);
HI551_write_cmos_sensor(0x2d2c, 0x40F2);
HI551_write_cmos_sensor(0x2d2e, 0x0005);
HI551_write_cmos_sensor(0x2d30, 0x07B2);
HI551_write_cmos_sensor(0x2d32, 0x3F77);
HI551_write_cmos_sensor(0x2d34, 0x431F);
HI551_write_cmos_sensor(0x2d36, 0x4D0C);
HI551_write_cmos_sensor(0x2d38, 0x533C);
HI551_write_cmos_sensor(0x2d3a, 0x930C);
HI551_write_cmos_sensor(0x2d3c, 0x2403);
HI551_write_cmos_sensor(0x2d3e, 0x5F0F);
HI551_write_cmos_sensor(0x2d40, 0x831C);
HI551_write_cmos_sensor(0x2d42, 0x23FD);
HI551_write_cmos_sensor(0x2d44, 0x4FC2);
HI551_write_cmos_sensor(0x2d46, 0x0A98);
HI551_write_cmos_sensor(0x2d48, 0x3F42);
HI551_write_cmos_sensor(0x2d4a, 0x4292);
HI551_write_cmos_sensor(0x2d4c, 0x0A86);
HI551_write_cmos_sensor(0x2d4e, 0x0784);
HI551_write_cmos_sensor(0x2d50, 0x4292);
HI551_write_cmos_sensor(0x2d52, 0x0A88);
HI551_write_cmos_sensor(0x2d54, 0x0786);
HI551_write_cmos_sensor(0x2d56, 0x3F06);
HI551_write_cmos_sensor(0x2d58, 0x403F);
HI551_write_cmos_sensor(0x2d5a, 0x0010);
HI551_write_cmos_sensor(0x2d5c, 0x3E18);
HI551_write_cmos_sensor(0x2d5e, 0x421F);
HI551_write_cmos_sensor(0x2d60, 0x811E);
HI551_write_cmos_sensor(0x2d62, 0xD21F);
HI551_write_cmos_sensor(0x2d64, 0x8100);
HI551_write_cmos_sensor(0x2d66, 0x5F0F);
HI551_write_cmos_sensor(0x2d68, 0xF32F);
HI551_write_cmos_sensor(0x2d6a, 0x4F82);
HI551_write_cmos_sensor(0x2d6c, 0x811E);
HI551_write_cmos_sensor(0x2d6e, 0x421F);
HI551_write_cmos_sensor(0x2d70, 0x811A);
HI551_write_cmos_sensor(0x2d72, 0xD21F);
HI551_write_cmos_sensor(0x2d74, 0x811C);
HI551_write_cmos_sensor(0x2d76, 0x5F0F);
HI551_write_cmos_sensor(0x2d78, 0xF32F);
HI551_write_cmos_sensor(0x2d7a, 0x4F82);
HI551_write_cmos_sensor(0x2d7c, 0x811A);
HI551_write_cmos_sensor(0x2d7e, 0xD392);
HI551_write_cmos_sensor(0x2d80, 0x7102);
HI551_write_cmos_sensor(0x2d82, 0x4138);
HI551_write_cmos_sensor(0x2d84, 0x4139);
HI551_write_cmos_sensor(0x2d86, 0x413A);
HI551_write_cmos_sensor(0x2d88, 0x413B);
HI551_write_cmos_sensor(0x2d8a, 0x4130);
HI551_write_cmos_sensor(0x2d8c, 0x0260);
HI551_write_cmos_sensor(0x2d8e, 0x0000);
HI551_write_cmos_sensor(0x2d90, 0x0C5A);
HI551_write_cmos_sensor(0x2d92, 0x0240);
HI551_write_cmos_sensor(0x2d94, 0x0000);
HI551_write_cmos_sensor(0x2d96, 0x0260);
HI551_write_cmos_sensor(0x2d98, 0x0000);
HI551_write_cmos_sensor(0x2d9a, 0x0C14);
HI551_write_cmos_sensor(0x2d9c, 0x4130);
HI551_write_cmos_sensor(0x2d9e, 0x4382);
HI551_write_cmos_sensor(0x2da0, 0x7602);
HI551_write_cmos_sensor(0x2da2, 0x4F82);
HI551_write_cmos_sensor(0x2da4, 0x7600);
HI551_write_cmos_sensor(0x2da6, 0x0270);
HI551_write_cmos_sensor(0x2da8, 0x0000);
HI551_write_cmos_sensor(0x2daa, 0x0C1B);
HI551_write_cmos_sensor(0x2dac, 0x0270);
HI551_write_cmos_sensor(0x2dae, 0x0001);
HI551_write_cmos_sensor(0x2db0, 0x421F);
HI551_write_cmos_sensor(0x2db2, 0x7606);
HI551_write_cmos_sensor(0x2db4, 0x4FC2);
HI551_write_cmos_sensor(0x2db6, 0x0188);
HI551_write_cmos_sensor(0x2db8, 0x4130);
HI551_write_cmos_sensor(0x2dba, 0xDF02);
HI551_write_cmos_sensor(0x2dbc, 0x3FFE);
HI551_write_cmos_sensor(0x2dbe, 0x430E);
HI551_write_cmos_sensor(0x2dc0, 0x930A);
HI551_write_cmos_sensor(0x2dc2, 0x2407);
HI551_write_cmos_sensor(0x2dc4, 0xC312);
HI551_write_cmos_sensor(0x2dc6, 0x100C);
HI551_write_cmos_sensor(0x2dc8, 0x2801);
HI551_write_cmos_sensor(0x2dca, 0x5A0E);
HI551_write_cmos_sensor(0x2dcc, 0x5A0A);
HI551_write_cmos_sensor(0x2dce, 0x930C);
HI551_write_cmos_sensor(0x2dd0, 0x23F7);
HI551_write_cmos_sensor(0x2dd2, 0x4130);
HI551_write_cmos_sensor(0x2dd4, 0x4030);
HI551_write_cmos_sensor(0x2dd6, 0xFE1E);
HI551_write_cmos_sensor(0x2dd8, 0xEE0E);
HI551_write_cmos_sensor(0x2dda, 0x403B);
HI551_write_cmos_sensor(0x2ddc, 0x0011);
HI551_write_cmos_sensor(0x2dde, 0x3C05);
HI551_write_cmos_sensor(0x2de0, 0x100D);
HI551_write_cmos_sensor(0x2de2, 0x6E0E);
HI551_write_cmos_sensor(0x2de4, 0x9A0E);
HI551_write_cmos_sensor(0x2de6, 0x2801);
HI551_write_cmos_sensor(0x2de8, 0x8A0E);
HI551_write_cmos_sensor(0x2dea, 0x6C0C);
HI551_write_cmos_sensor(0x2dec, 0x6D0D);
HI551_write_cmos_sensor(0x2dee, 0x831B);
HI551_write_cmos_sensor(0x2df0, 0x23F7);
HI551_write_cmos_sensor(0x2df2, 0x4130);
HI551_write_cmos_sensor(0x2df4, 0xEF0F);
HI551_write_cmos_sensor(0x2df6, 0xEE0E);
HI551_write_cmos_sensor(0x2df8, 0x4039);
HI551_write_cmos_sensor(0x2dfa, 0x0021);
HI551_write_cmos_sensor(0x2dfc, 0x3C0A);
HI551_write_cmos_sensor(0x2dfe, 0x1008);
HI551_write_cmos_sensor(0x2e00, 0x6E0E);
HI551_write_cmos_sensor(0x2e02, 0x6F0F);
HI551_write_cmos_sensor(0x2e04, 0x9B0F);
HI551_write_cmos_sensor(0x2e06, 0x2805);
HI551_write_cmos_sensor(0x2e08, 0x2002);
HI551_write_cmos_sensor(0x2e0a, 0x9A0E);
HI551_write_cmos_sensor(0x2e0c, 0x2802);
HI551_write_cmos_sensor(0x2e0e, 0x8A0E);
HI551_write_cmos_sensor(0x2e10, 0x7B0F);
HI551_write_cmos_sensor(0x2e12, 0x6C0C);
HI551_write_cmos_sensor(0x2e14, 0x6D0D);
HI551_write_cmos_sensor(0x2e16, 0x6808);
HI551_write_cmos_sensor(0x2e18, 0x8319);
HI551_write_cmos_sensor(0x2e1a, 0x23F1);
HI551_write_cmos_sensor(0x2e1c, 0x4130);
HI551_write_cmos_sensor(0x2e1e, 0x430E);
HI551_write_cmos_sensor(0x2e20, 0x430F);
HI551_write_cmos_sensor(0x2e22, 0x3C08);
HI551_write_cmos_sensor(0x2e24, 0xC312);
HI551_write_cmos_sensor(0x2e26, 0x100D);
HI551_write_cmos_sensor(0x2e28, 0x100C);
HI551_write_cmos_sensor(0x2e2a, 0x2802);
HI551_write_cmos_sensor(0x2e2c, 0x5A0E);
HI551_write_cmos_sensor(0x2e2e, 0x6B0F);
HI551_write_cmos_sensor(0x2e30, 0x5A0A);
HI551_write_cmos_sensor(0x2e32, 0x6B0B);
HI551_write_cmos_sensor(0x2e34, 0x930C);
HI551_write_cmos_sensor(0x2e36, 0x23F6);
HI551_write_cmos_sensor(0x2e38, 0x930D);
HI551_write_cmos_sensor(0x2e3a, 0x23F4);
HI551_write_cmos_sensor(0x2e3c, 0x4130);
HI551_write_cmos_sensor(0x2e3e, 0x0000);
HI551_write_cmos_sensor(0x2ffe, 0xf000);
HI551_write_cmos_sensor(0x3000, 0x02B0);
HI551_write_cmos_sensor(0x3002, 0x02B0);
HI551_write_cmos_sensor(0x3004, 0x0678);
HI551_write_cmos_sensor(0x3006, 0x0278);
HI551_write_cmos_sensor(0x3008, 0x0678);
HI551_write_cmos_sensor(0x300A, 0x0279);
HI551_write_cmos_sensor(0x300C, 0x024A);
HI551_write_cmos_sensor(0x300E, 0x024A);
HI551_write_cmos_sensor(0x3010, 0x0AB8);
HI551_write_cmos_sensor(0x3012, 0x0AB8);
HI551_write_cmos_sensor(0x3014, 0x02B8);
HI551_write_cmos_sensor(0x3016, 0x0678);
HI551_write_cmos_sensor(0x3018, 0x0278);
HI551_write_cmos_sensor(0x301A, 0x0678);
HI551_write_cmos_sensor(0x301C, 0x0278);
HI551_write_cmos_sensor(0x4000, 0x0005);
HI551_write_cmos_sensor(0x4002, 0x003D);
HI551_write_cmos_sensor(0x4004, 0x0F00);
HI551_write_cmos_sensor(0x4006, 0xCF41);
HI551_write_cmos_sensor(0x4008, 0x1F04);
HI551_write_cmos_sensor(0x400A, 0xDF43);
HI551_write_cmos_sensor(0x400C, 0x1F87);
HI551_write_cmos_sensor(0x400E, 0x1F87);
HI551_write_cmos_sensor(0x4010, 0x1001);
HI551_write_cmos_sensor(0x4012, 0x1039);
HI551_write_cmos_sensor(0x4014, 0x1004);
HI551_write_cmos_sensor(0x4016, 0x1F04);
HI551_write_cmos_sensor(0x4018, 0x1F04);
HI551_write_cmos_sensor(0x401A, 0x1F04);
HI551_write_cmos_sensor(0x401C, 0x1F04);


//    EOFIRM
//--------------------------------------------------------------------
// end of software code
//--------------------------------------------------------------------

//====================================================================
// sreg setting
//====================================================================
HI551_write_cmos_sensor(0x0B00, 0xE1DA);
HI551_write_cmos_sensor(0x0B02, 0x0000);
HI551_write_cmos_sensor(0x0B04, 0x6DAA);	// PCP ON : 3.6V
HI551_write_cmos_sensor(0x0B06, 0x5BAB);	// NCP ON : -1.4V
HI551_write_cmos_sensor(0x0B08, 0x8085);
HI551_write_cmos_sensor(0x0B0A, 0xAA0B);
HI551_write_cmos_sensor(0x0B0C, 0xC40C);
//HI551_write_cmos_sensor(0x0B0E, 0x9F61);
HI551_write_cmos_sensor(0x0B0E, 0xB361);  // input rng : 600mv
HI551_write_cmos_sensor(0x0B10, 0x0010);
HI551_write_cmos_sensor(0x0B12, 0x0FFC);	// Multi-2, noise
HI551_write_cmos_sensor(0x0B14, 0x370b);//370B);	
HI551_write_cmos_sensor(0x0B16, 0x4A0F);
HI551_write_cmos_sensor(0x0B18, 0x0000);
HI551_write_cmos_sensor(0x0B1A, 0x9000);
HI551_write_cmos_sensor(0x0B1C, 0x1000);
HI551_write_cmos_sensor(0x0B1E, 0x0500);
//--------------------------------------------------------------------
// end of sreg setting
//--------------------------------------------------------------------


// mipi --------------------------------------------------------------
HI551_write_cmos_sensor(0x0902, 0x4101);		// mipi_tx_op_mode1 / mipi_tx_op_mode2
HI551_write_cmos_sensor(0x090A, 0x03E8);		// mipi_vblank_delay_h / l
HI551_write_cmos_sensor(0x090C, 0x0020);		// mipi_hblank_short_delay_h / l
HI551_write_cmos_sensor(0x090E, 0x0E00);		// mipi_hblank_long_delay_h / l
// Interval Time (2byte write)
HI551_write_cmos_sensor(0x0910, 0x5D07); // mipi_Exit_sequence / mipi_LPX
HI551_write_cmos_sensor(0x0912, 0x061e); // mipi_CLK_prepare   / mipi_clk_zero
HI551_write_cmos_sensor(0x0914, 0x0407); // mipi_clk_pre		/ mipi_data_prepare
HI551_write_cmos_sensor(0x0916, 0x0b0d); // mipi_data_zero		/ mipi_data_trail	
HI551_write_cmos_sensor(0x0918, 0x0f09); // mipi_clk_post		/ mipi_clk_trail
//HI551_write_cmos_sensor(0x0916, 0x0b0a); // mipi_data_zero		/ mipi_data_trail	
//HI551_write_cmos_sensor(0x0918, 0x0e09); // mipi_clk_post		/ mipi_clk_trail

// system ------------------------------------------------------------
HI551_write_cmos_sensor(0x0F02, 0x0106);		// pll_cfg1 / pll_cfg2
HI551_write_cmos_sensor(0x0C36, 0x0100);		// g_sum_ctl / null

// tg ----------------------------------------------------------------
HI551_write_cmos_sensor(0x0000, 0x0100);		// image_orient / null
HI551_write_cmos_sensor(0x003C, 0x0000);		// fixed_frame / tg_ctl_0
HI551_write_cmos_sensor(0x003E, 0x0000);		// tg_ctl1 / tg_ctl2
HI551_write_cmos_sensor(0x004C, 0x0100);		// tg_enable / null
HI551_write_cmos_sensor(0x0C10, 0x0520);		// dig_blc_offset_h / l

// blc  --------------------------------------------------------------
HI551_write_cmos_sensor(0x0C00, 0x3300);		// blc_ctl1,2 - lblc,lblc_dpc,adp_dead_pxl_th,dither enable
HI551_write_cmos_sensor(0x0C02, 0x0000);		// blc_ctl1,2 - lblc,lblc_dpc,adp_dead_pxl_th,dither enable
HI551_write_cmos_sensor(0x0C0E, 0x8500);		// blc_ctl3,4 - fblc,fblc_dpc,fobp_offset,obp_bypass enable
HI551_write_cmos_sensor(0x0C04, 0x800D);		// analog_ofs_man / blc_dead_pixel_th 
HI551_write_cmos_sensor(0x0C26, 0x4C18);		// curr_frm_obp_avg_wgt (x0.6),frm_obp_avg_pga_wgt (x0.75)
//HI551_write_cmos_sensor(0x0C12, 0x0040);		// lobp_dig_gb_offset0 / lobp_dig_gr_offset0

// pixel address -----------------------------------------------------
HI551_write_cmos_sensor(0x000E, 0x0000);		// x_addr_start_lobp_h / l
HI551_write_cmos_sensor(0x0014, 0x003F);		// x_addr_end_lobp_h / l
HI551_write_cmos_sensor(0x0010, 0x0060);		// x_addr_start_robp_h / l
HI551_write_cmos_sensor(0x0016, 0x009F);		// x_addr_end_robp_h / l
HI551_write_cmos_sensor(0x0012, 0x00B8);		// x_addr_start_hact_h / l
HI551_write_cmos_sensor(0x0018, 0x0AE7);		// x_addr_end_hact_h / l
HI551_write_cmos_sensor(0x0022, 0x0004);		// y_addr_start_fobp_h / l
HI551_write_cmos_sensor(0x0028, 0x000B);		// y_addr_end_fobp_h / l
HI551_write_cmos_sensor(0x0024, 0xFFFA);		// y_addr_start_dummy_h / l
HI551_write_cmos_sensor(0x002A, 0xFFFF);		// y_addr_end_dummy_h / l
HI551_write_cmos_sensor(0x0026, 0x0028);		// y_addr_start_vact_h / l
HI551_write_cmos_sensor(0x002C, 0x07CF);		// y_addr_end_vact_h / l
HI551_write_cmos_sensor(0x0032, 0x0101);		// y_odd_inc_vact / y_even_inc_vact

// size info -----------------------------------------------------
HI551_write_cmos_sensor(0x0006, 0x07bc);//ca);		// frame_length_lines_h / l
HI551_write_cmos_sensor(0x0008, 0x0B7C);		// line_length_pck_h / l
HI551_write_cmos_sensor(0x0020, 0x0700);		// x_region_sel / x_region_orient
HI551_write_cmos_sensor(0x0034, 0x0700);		// y_region_sel / y_resion_orient
//HI551_write_cmos_sensor(0x0A12, 0x0A2C);		// x_output_size_h / l
//HI551_write_cmos_sensor(0x0A14, 0x07A8);		// y_output_size_h / l
HI551_write_cmos_sensor(0x0112, 0x0A2C);		// x_output_size_h / l
HI551_write_cmos_sensor(0x0114, 0x07A4);		// y_output_size_h / l


// isp control ---------------------------------------------------
//HI551_write_cmos_sensor(0x0A04, 0x0101);		// isp_en_h / l
HI551_write_cmos_sensor(0x0110, 0x0101);		// isp_en_h / l
HI551_write_cmos_sensor(0x020A, 0x0002);		// test_pattern_mode

// luminance control ---------------------------------------------
HI551_write_cmos_sensor(0x0002, 0x02b2);//200);		// fine_integ_time_h / l
HI551_write_cmos_sensor(0x0004, 0x07b7);//1D3A);		// coarse_integ_time_h / l  125ms
HI551_write_cmos_sensor(0x003A, 0x0000);		// analog_gain_code_global / null

//==============================================================
// Analog setting tuning Start
//==============================================================

// Analog setting tuning @2013.11.02
HI551_write_cmos_sensor(0x0036, 0x007F);		// ramp_init_pofs / ramp_rst_pofs
HI551_write_cmos_sensor(0x0038, 0x7F00);		// ramp_sig_pofs / null
HI551_write_cmos_sensor(0x0B0A, 0xB847);		// Bias setting
HI551_write_cmos_sensor(0x0138, 0x0004);		// d2a_pxl_drv_pwr_hv / d2a_pxl_drv_pwr_lv
HI551_write_cmos_sensor(0x013A, 0x0100);		// d2a_row_idle_ctrl_en
HI551_write_cmos_sensor(0x0B04, 0xA9AA);		// PCP setting : PCP off, NCP max pumping
HI551_write_cmos_sensor(0x0B10, 0xF000);		// vrst

// Analog Noise block tuning @2013.11.02 ---------------------------------------------------
HI551_write_cmos_sensor(0x0B0C, 0xE26C);		// noise en, noise idac
HI551_write_cmos_sensor(0x0B12, 0x0800);		// noise cap, noise res

//==============================================================
// Analog setting tuning End
//==============================================================

// REV.AB setting
HI551_write_cmos_sensor(0x0B04, 0x69AA);       // Max pumping disable, PCP/NCP level = 3.6/-1.4 V 
HI551_write_cmos_sensor(0x0B08, 0x8085);       // Reset Clamp Level 
HI551_write_cmos_sensor(0x0B0A, 0xAC49);       // Comparator/Pixel current 
HI551_write_cmos_sensor(0x0B16, 0x440F);       // Analog Clock Speed
HI551_write_cmos_sensor(0x0C10, 0x0532);       // BLC offset for DDS operation 
HI551_write_cmos_sensor(0x0138, 0x0104);       // Rx pcp enable 

HI551_write_cmos_sensor(0x0120, 0x0004);       // Extra Dynamic Signal Timing Setting 
HI551_write_cmos_sensor(0x0122, 0x0582);

HI551_write_cmos_sensor(0x0C00, 0x7B06);		// blc_ctl1,2 - lblc,lblc_dpc,adp_dead_pxl_th,dither enable
HI551_write_cmos_sensor(0x0C04, 0x8004);		// analog_ofs_man / lblc_dead_pixel_th 
HI551_write_cmos_sensor(0x0C06, 0x0008);		// fblc_ofs_wgt / fblc_dead_pixel_th 
HI551_write_cmos_sensor(0x0C08, 0x0302);		// dpc pga weight   16 = 1x,   x/16
HI551_write_cmos_sensor(0x0C02, 0x1000);       // blc_ofs_wgt
HI551_write_cmos_sensor(0x0C0E, 0xE500);		// blc_ctl3,4 - fblc,fobp_offset,obp_bypass enable
HI551_write_cmos_sensor(0x0C26, 0x4C10);		// curr_frm_obp_avg_wgt (x0.6),frm_obp_avg_pga_wgt (x1)

//========== 20140421 analog tun ===============
HI551_write_cmos_sensor(0x0120, 0x0064);       // Extra Dynamic Signal Timing Setting 
HI551_write_cmos_sensor(0x0138, 0x0304);       // Rx,Sx pcp enable 
HI551_write_cmos_sensor(0x0B04, 0x69AE);		// PCP ON : 3.6V, pcp pumping cap. control
HI551_write_cmos_sensor(0x0B06, 0x5BAF);		// NCP ON : -1.4V, ncp pumping cap. control
HI551_write_cmos_sensor(0x0B0C, 0xE1BC);		// NC control
HI551_write_cmos_sensor(0x013C, 0x0001);		// pll clk inversion 
HI551_write_cmos_sensor(0x0B12, 0x5000);		// ramp clk_div2, 4lsb control enable 

//========== 20140429 adpc, dpc enable =========
HI551_write_cmos_sensor(0x0110, 0x0123);		// isp_en_h / l
HI551_write_cmos_sensor(0x0708, 0x0100);		// dpc_ctrl1, dpc enable 
HI551_write_cmos_sensor(0x073C, 0x0100);		// adaptive dpc th control 
//== analog tun
//HI551_write_cmos_sensor(0x0138, 0x0204);		// channel diif dec. 
//HI551_write_cmos_sensor(0x0B04, 0x692E);		// channel diff dec. 

//========== 20140606 mtk tun =========
HI551_write_cmos_sensor(0x0C0E, 0xE501);		// blc_ctl3,4 - line blc on  
HI551_write_cmos_sensor(0x0C12, 0x0040);		// blc offset 64 
HI551_write_cmos_sensor(0x0B06, 0x59AF);		// NCP ON : -0.9V, ncp pumping cap. control

//========== 20140616 soft standby & dpc tun =========
HI551_write_cmos_sensor(0x0708, 0x0100); // dpc disable
HI551_write_cmos_sensor(0x070a, 0x390b);
HI551_write_cmos_sensor(0x0712, 0x643c);
HI551_write_cmos_sensor(0x0720, 0xa050);
HI551_write_cmos_sensor(0x0726, 0x1428);
HI551_write_cmos_sensor(0x0728, 0x2867);
HI551_write_cmos_sensor(0x072a, 0x5437);
HI551_write_cmos_sensor(0x0734, 0xf070);
HI551_write_cmos_sensor(0x0736, 0x0000);
HI551_write_cmos_sensor(0x0738, 0x1d3a);
HI551_write_cmos_sensor(0x073a, 0x01f2);
HI551_write_cmos_sensor(0x073c, 0x0100); // mcu on/of
//========== 20140628 dpc disable =========
HI551_write_cmos_sensor(0x0708, 0x0000); // dpc disable
HI551_write_cmos_sensor(0x0a02, 0x0100); // fast sleep

	HI551DB("HI551_Sensor_Init exit :\n ");

}   /*  HI551_Sensor_Init  */

/*************************************************************************
* FUNCTION
*   HI551Open
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

UINT32 HI551Open(void)
{

	volatile signed int i;
	kal_uint16 sensor_id = 0;

	HI551DB("HI551Open enter :\n ");


HI551_write_cmos_sensor(0x0110, 0x0123);		// isp_en_h / l
HI551_write_cmos_sensor(0x0708, 0x0001);		// dpc_ctrl1, dpc enable 
HI551_write_cmos_sensor(0x073C, 0x0100);		// adaptive dpc th control 
mdelay(50);
HI551DB("--------hi551 I2C test %X   %X   %X\n",HI551_read_cmos_sensor(0x0110),HI551_read_cmos_sensor(0x0111),HI551_read_cmos_sensor(0x0709));


	
	
	for(i=0;i<3;i++)
	{
		//sensor_id = (HI551_read_cmos_sensor(0x0F16)<<8)|HI551_read_cmos_sensor(0x0F17);
		HI551GetSensorID(&sensor_id);
		HI551DB("OHI551 READ ID :%x",sensor_id);
		if(sensor_id != HI551MIPI_SENSOR_ID)                       //0x0f16=0x05,0x0f17=0x51    hi551
		{
			return ERROR_SENSOR_CONNECT_FAIL;
		}
		else
		{
			break;
		}
	}
	
	spin_lock(&HI551mipiraw_drv_lock);
	HI551.sensorMode = SENSOR_MODE_INIT;
	HI551.HI551AutoFlickerMode = KAL_FALSE;
	HI551.HI551VideoMode = KAL_FALSE;
	spin_unlock(&HI551mipiraw_drv_lock);
	HI551_Sensor_Init();

	spin_lock(&HI551mipiraw_drv_lock);
	HI551.DummyLines= 0;
	HI551.DummyPixels= 0;
	HI551.pvPclk =  ( HI551_PV_CLK / 10000); 
	HI551.videoPclk = ( HI551_VIDEO_CLK / 10000);
	HI551.capPclk = (HI551_CAP_CLK / 10000);

	HI551.shutter = 0x4EA;
	HI551.pvShutter = 0x4EA;
	HI551.maxExposureLines =HI551_PV_PERIOD_LINE_NUMS -4;

	HI551.ispBaseGain = BASEGAIN;//0x40
	HI551.sensorGlobalGain = 0x1f;//sensor gain read from 0x350a 0x350b; 0x1f as 3.875x
	HI551.pvGain = 0x1f;
	HI551.realGain = 0x1f;//ispBaseGain as 1x
	spin_unlock(&HI551mipiraw_drv_lock);


	HI551DB("HI551Open exit :\n ");

    return ERROR_NONE;
}

/*************************************************************************
* FUNCTION
*   HI551GetSensorID
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
//extern bool camera_pdn_reverse;
UINT32 HI551GetSensorID(UINT32 *sensorID)
{
    int  retry = 3;

	HI551DB("HI551GetSensorID enter :\n ");
	
	//mt_set_gpio_mode(GPIO_CAMERA_CMPDN_PIN, GPIO_CAMERA_CMPDN_PIN_M_GPIO);
	//mt_set_gpio_out(GPIO_CAMERA_CMPDN_PIN, GPIO_OUT_ONE);
	mDELAY(10);

    // check if sensor ID correct
    do {
        *sensorID = (HI551_read_cmos_sensor(0x0F16)<<8)|HI551_read_cmos_sensor(0x0F17);
        if (*sensorID == HI551MIPI_SENSOR_ID)
        	{
        		HI551DB("Sensor ID = 0x%04x\n", *sensorID);
            	break;
        	}
        HI551DB("Read Sensor ID Fail = 0x%04x\n", *sensorID);
//      mDELAY(1000);
        retry--;
    } while (retry > 0);
//    } while (1);

    if (*sensorID != HI551MIPI_SENSOR_ID) {
        *sensorID = 0xFFFFFFFF;
        return ERROR_SENSOR_CONNECT_FAIL;
    }
    return ERROR_NONE;
}


/*************************************************************************
* FUNCTION
*   HI551_SetShutter
*
* DESCRIPTION
*   This function set e-shutter of HI551 to change exposure time.
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
void HI551_SetShutter(kal_uint32 iShutter)
{

//   if(HI551.shutter == iShutter)
//   		return;

   spin_lock(&HI551mipiraw_drv_lock);
   HI551.shutter= iShutter;
   spin_unlock(&HI551mipiraw_drv_lock);

   HI551_write_shutter(iShutter);
   return;
 
}   /*  HI551_SetShutter   */



/*************************************************************************
* FUNCTION
*   HI551_read_shutter
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
UINT32 HI551_read_shutter(void)
{

	kal_uint16 temp_reg1, temp_reg2;
	UINT32 shutter =0;
	temp_reg1 = HI551_read_cmos_sensor(0x0004);  
	//temp_reg2 = HI551_read_cmos_sensor(0x0005);    
	//read out register value and divide 16;
	shutter  = temp_reg1;

	return shutter;
}

/*************************************************************************
* FUNCTION
*   HI551_night_mode
*
* DESCRIPTION
*   This function night mode of HI551.
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
void HI551_NightMode(kal_bool bEnable)
{
}/*	HI551_NightMode */



/*************************************************************************
* FUNCTION
*   HI551Close
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
UINT32 HI551Close(void)
{
    //  CISModulePowerOn(FALSE);
    //s_porting
    //  DRV_I2CClose(HI551hDrvI2C);
    //e_porting
    ReEnteyCamera = KAL_FALSE;
    return ERROR_NONE;
}	/* HI551Close() */

void HI551SetFlipMirror(kal_int32 imgMirror)
{
#if 1
	
    switch (imgMirror)
    {
        case IMAGE_NORMAL://IMAGE_NOMAL:
            HI551_write_cmos_sensor(0x0000, 0x0000);//Set normal
            break;    
        case IMAGE_V_MIRROR://IMAGE_V_MIRROR:
            HI551_write_cmos_sensor(0x0000, 0x0200);	//Set flip
            break;
        case IMAGE_H_MIRROR://IMAGE_H_MIRROR:
            HI551_write_cmos_sensor(0x0000, 0x0100);//Set mirror
            break;
        case IMAGE_HV_MIRROR://IMAGE_H_MIRROR:
            HI551_write_cmos_sensor(0x0000, 0x0300);	//Set mirror & flip
            break;            
    }
#endif	
}


/*************************************************************************
* FUNCTION
*   HI551Preview
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
UINT32 HI551Preview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                                                MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{

	HI551DB("HI551Preview enter:");
   
	// preview size
	if(HI551.sensorMode == SENSOR_MODE_PREVIEW)
	{
		// do nothing
		// FOR CCT PREVIEW
		
	}
	else
	{
		HI551DB("HI551Preview setting!!\n");
		HI551PreviewSetting();
		 
		
	}
	
	spin_lock(&HI551mipiraw_drv_lock);
	HI551.sensorMode = SENSOR_MODE_PREVIEW; // Need set preview setting after capture mode
	HI551.DummyPixels = 0;//define dummy pixels and lines
	HI551.DummyLines = 0 ;
	HI551_FeatureControl_PERIOD_PixelNum=HI551_PV_PERIOD_PIXEL_NUMS+ HI551.DummyPixels;
	HI551_FeatureControl_PERIOD_LineNum=HI551_PV_PERIOD_LINE_NUMS+HI551.DummyLines;
	spin_unlock(&HI551mipiraw_drv_lock);

	spin_lock(&HI551mipiraw_drv_lock);
	HI551.imgMirror = sensor_config_data->SensorImageMirror;
	//HI551.imgMirror =IMAGE_NORMAL; //by module layout
	spin_unlock(&HI551mipiraw_drv_lock);
	
	//HI551SetFlipMirror(sensor_config_data->SensorImageMirror);
	//HI551SetFlipMirror(IMAGE_H_MIRROR);
	

	HI551DBSOFIA("[HI551Preview]frame_len=%x\n", ((HI551_read_cmos_sensor(0x380e)<<8)+HI551_read_cmos_sensor(0x380f)));

 //   mDELAY(40);
	HI551DB("HI551Preview exit:\n");
    return ERROR_NONE;
}	/* HI551Preview() */



UINT32 HI551Video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                                                MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{

	HI551DB("HI551Video enter:");

	if(HI551.sensorMode == SENSOR_MODE_VIDEO)
	{
		// do nothing
	}
	else
	{
		//HI551VideoSetting();

	}
	spin_lock(&HI551mipiraw_drv_lock);
	HI551.sensorMode = SENSOR_MODE_VIDEO;
	HI551_FeatureControl_PERIOD_PixelNum=HI551_VIDEO_PERIOD_PIXEL_NUMS+ HI551.DummyPixels;
	HI551_FeatureControl_PERIOD_LineNum=HI551_VIDEO_PERIOD_LINE_NUMS+HI551.DummyLines;
	spin_unlock(&HI551mipiraw_drv_lock);


	spin_lock(&HI551mipiraw_drv_lock);
	HI551.imgMirror = sensor_config_data->SensorImageMirror;
	//HI551.imgMirror =IMAGE_NORMAL; //by module layout
	spin_unlock(&HI551mipiraw_drv_lock);
	//HI551SetFlipMirror(sensor_config_data->SensorImageMirror);
	//HI551SetFlipMirror(IMAGE_H_MIRROR);

//    mDELAY(40);
	HI551DB("HI551Video exit:\n");
    return ERROR_NONE;
}


UINT32 HI551Capture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                                                MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{

 	kal_uint32 shutter = HI551.shutter;
	kal_uint32 temp_data;
	//kal_uint32 pv_line_length , cap_line_length,

	HI551CaptureSetting();

	if( SENSOR_MODE_CAPTURE== HI551.sensorMode)
	{
		HI551DB("HI551Capture BusrtShot!!!\n");
	}
	else
	{
		HI551DB("HI551Capture enter:\n");
#if 0
		//Record Preview shutter & gain
		shutter=HI551_read_shutter();
		temp_data =  read_HI551_gain();
		spin_lock(&HI551mipiraw_drv_lock);
		HI551.pvShutter =shutter;
		HI551.sensorGlobalGain = temp_data;
		HI551.pvGain =HI551.sensorGlobalGain;
		spin_unlock(&HI551mipiraw_drv_lock);

		HI551DB("[HI551Capture]HI551.shutter=%d, read_pv_shutter=%d, read_pv_gain = 0x%x\n",HI551.shutter, shutter,HI551.sensorGlobalGain);
#endif
		// Full size setting
		//HI551CaptureSetting();
	    mDELAY(20);

		spin_lock(&HI551mipiraw_drv_lock);
		HI551.sensorMode = SENSOR_MODE_CAPTURE;
		HI551.imgMirror = sensor_config_data->SensorImageMirror;
		//HI551.imgMirror =IMAGE_NORMAL; //by module layout
		HI551.DummyPixels = 200;//0;//define dummy pixels and lines
		HI551.DummyLines = 100; //0;
		HI551_FeatureControl_PERIOD_PixelNum = HI551_FULL_PERIOD_PIXEL_NUMS + HI551.DummyPixels;
		HI551_FeatureControl_PERIOD_LineNum = HI551_FULL_PERIOD_LINE_NUMS + HI551.DummyLines;
		spin_unlock(&HI551mipiraw_drv_lock);

		//HI551SetFlipMirror(sensor_config_data->SensorImageMirror);

		//HI551SetFlipMirror(IMAGE_H_MIRROR);

		HI551DB("HI551Capture exit:\n");
	}
	

    return ERROR_NONE;
}	/* HI551Capture() */

UINT32 HI551GetResolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *pSensorResolution)
{

    HI551DB("HI551GetResolution!!\n");

	pSensorResolution->SensorPreviewWidth	= HI551_IMAGE_SENSOR_PV_WIDTH;
    pSensorResolution->SensorPreviewHeight	= HI551_IMAGE_SENSOR_PV_HEIGHT;
    pSensorResolution->SensorFullWidth		= HI551_IMAGE_SENSOR_FULL_WIDTH;
    pSensorResolution->SensorFullHeight		= HI551_IMAGE_SENSOR_FULL_HEIGHT;
    pSensorResolution->SensorVideoWidth		= HI551_IMAGE_SENSOR_VIDEO_WIDTH;
    pSensorResolution->SensorVideoHeight    = HI551_IMAGE_SENSOR_VIDEO_HEIGHT;
//    HI551DB("SensorPreviewWidth:  %d.\n", pSensorResolution->SensorPreviewWidth);
//    HI551DB("SensorPreviewHeight: %d.\n", pSensorResolution->SensorPreviewHeight);
//    HI551DB("SensorFullWidth:  %d.\n", pSensorResolution->SensorFullWidth);
//    HI551DB("SensorFullHeight: %d.\n", pSensorResolution->SensorFullHeight);
    return ERROR_NONE;
}   /* HI551GetResolution() */

UINT32 HI551GetInfo(MSDK_SCENARIO_ID_ENUM ScenarioId,
                                                MSDK_SENSOR_INFO_STRUCT *pSensorInfo,
                                                MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{

	pSensorInfo->SensorPreviewResolutionX= HI551_IMAGE_SENSOR_PV_WIDTH;
	pSensorInfo->SensorPreviewResolutionY= HI551_IMAGE_SENSOR_PV_HEIGHT;

	pSensorInfo->SensorFullResolutionX= HI551_IMAGE_SENSOR_FULL_WIDTH;
    pSensorInfo->SensorFullResolutionY= HI551_IMAGE_SENSOR_FULL_HEIGHT;

	spin_lock(&HI551mipiraw_drv_lock);
	HI551.imgMirror = pSensorConfigData->SensorImageMirror ;
	spin_unlock(&HI551mipiraw_drv_lock);

   	pSensorInfo->SensorOutputDataFormat= SENSOR_OUTPUT_FORMAT_RAW_Gb;
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

            pSensorInfo->SensorGrabStartX = HI551_PV_X_START;
            pSensorInfo->SensorGrabStartY = HI551_PV_Y_START;

            pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_2_LANE;
            pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
	     	pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = MIPI_DELAY_COUNT;
	    	pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
            pSensorInfo->SensorPacketECCOrder = 1;
            break;
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
            pSensorInfo->SensorClockFreq=24;
            pSensorInfo->SensorClockRisingCount= 0;

            pSensorInfo->SensorGrabStartX = HI551_VIDEO_X_START;
            pSensorInfo->SensorGrabStartY = HI551_VIDEO_Y_START;

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

            pSensorInfo->SensorGrabStartX = HI551_FULL_X_START;	//2*HI551_IMAGE_SENSOR_PV_STARTX;
            pSensorInfo->SensorGrabStartY = HI551_FULL_Y_START;	//2*HI551_IMAGE_SENSOR_PV_STARTY;

            pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_2_LANE;
            pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
            pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = MIPI_DELAY_COUNT;
            pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
            pSensorInfo->SensorPacketECCOrder = 1;
            break;
        default:
			pSensorInfo->SensorClockFreq=24;
            pSensorInfo->SensorClockRisingCount= 0;

            pSensorInfo->SensorGrabStartX = HI551_PV_X_START;
            pSensorInfo->SensorGrabStartY = HI551_PV_Y_START;

            pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_2_LANE;
            pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
	     	pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = MIPI_DELAY_COUNT;
	    	pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
            pSensorInfo->SensorPacketECCOrder = 1;
            break;
    }

    memcpy(pSensorConfigData, &HI551SensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));

    return ERROR_NONE;
}   /* HI551GetInfo() */


UINT32 HI551Control(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *pImageWindow,
                                                MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
		spin_lock(&HI551mipiraw_drv_lock);
		HI551CurrentScenarioId = ScenarioId;
		spin_unlock(&HI551mipiraw_drv_lock);
		//HI551DB("ScenarioId=%d\n",ScenarioId);
		HI551DB("HI551CurrentScenarioId=%d\n",HI551CurrentScenarioId);

	switch (ScenarioId)
    {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
            HI551Preview(pImageWindow, pSensorConfigData);
            break;
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			HI551Video(pImageWindow, pSensorConfigData);
			break;
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
            HI551Capture(pImageWindow, pSensorConfigData);
            break;

        default:
            return ERROR_INVALID_SCENARIO_ID;

    }
    return ERROR_NONE;
} /* HI551Control() */


UINT32 HI551SetVideoMode(UINT16 u2FrameRate)
{

    kal_uint32 MIN_Frame_length =0,frameRate=0,extralines=0;
    HI551DB("[HI551SetVideoMode] frame rate = %d\n", u2FrameRate);

	spin_lock(&HI551mipiraw_drv_lock);
	VIDEO_MODE_TARGET_FPS=u2FrameRate;
	spin_unlock(&HI551mipiraw_drv_lock);

	if(u2FrameRate==0)
	{
		HI551DB("Disable Video Mode or dynimac fps\n");
		return KAL_TRUE;
	}
	if(u2FrameRate >30 || u2FrameRate <5)
	    HI551DB("error frame rate seting\n");

    if(HI551.sensorMode == SENSOR_MODE_VIDEO)//video ScenarioId recording
    {
    	if(HI551.HI551AutoFlickerMode == KAL_TRUE)
    	{
    		if (u2FrameRate==30)
				frameRate= 304;
			else if(u2FrameRate==15)
				frameRate= 148;//148;
			else
				frameRate=u2FrameRate*10;

			MIN_Frame_length = (HI551.videoPclk*10000)/(HI551_VIDEO_PERIOD_PIXEL_NUMS + HI551.DummyPixels)/frameRate*10;
    	}
		else
		{
    		if (u2FrameRate==30)
				MIN_Frame_length= HI551_VIDEO_PERIOD_LINE_NUMS;
			else	
				MIN_Frame_length = (HI551.videoPclk*10000) /(HI551_VIDEO_PERIOD_PIXEL_NUMS + HI551.DummyPixels)/u2FrameRate;
		}

		if((MIN_Frame_length <=HI551_VIDEO_PERIOD_LINE_NUMS))
		{
			MIN_Frame_length = HI551_VIDEO_PERIOD_LINE_NUMS;
			HI551DB("[HI551SetVideoMode]current fps = %d\n", (HI551.videoPclk*10000)  /(HI551_VIDEO_PERIOD_PIXEL_NUMS)/HI551_VIDEO_PERIOD_LINE_NUMS);
		}
		HI551DB("[HI551SetVideoMode]current fps (10 base)= %d\n", (HI551.videoPclk*10000)*10/(HI551_VIDEO_PERIOD_PIXEL_NUMS + HI551.DummyPixels)/MIN_Frame_length);

		if(HI551.shutter + 4 > MIN_Frame_length)
				MIN_Frame_length = HI551.shutter + 4;

		extralines = MIN_Frame_length - HI551_VIDEO_PERIOD_LINE_NUMS;
		
		spin_lock(&HI551mipiraw_drv_lock);
		HI551.DummyPixels = 0;//define dummy pixels and lines
		HI551.DummyLines = extralines ;
		spin_unlock(&HI551mipiraw_drv_lock);
		
		HI551_SetDummy(HI551.DummyPixels,extralines);
    }
	else if(HI551.sensorMode == SENSOR_MODE_CAPTURE)
	{
		HI551DB("-------[HI551SetVideoMode]ZSD???---------\n");
		if(HI551.HI551AutoFlickerMode == KAL_TRUE)
    	{
    		if (u2FrameRate==15)
			    frameRate= 148;
			else
				frameRate=u2FrameRate*10;

			MIN_Frame_length = (HI551.capPclk*10000) /(HI551_FULL_PERIOD_PIXEL_NUMS + HI551.DummyPixels)/frameRate*10;
    	}
		else
			MIN_Frame_length = (HI551.capPclk*10000) /(HI551_FULL_PERIOD_PIXEL_NUMS + HI551.DummyPixels)/u2FrameRate;

		if((MIN_Frame_length <=HI551_FULL_PERIOD_LINE_NUMS))
		{
			MIN_Frame_length = HI551_FULL_PERIOD_LINE_NUMS;
			HI551DB("[HI551SetVideoMode]current fps = %d\n", (HI551.capPclk*10000) /(HI551_FULL_PERIOD_PIXEL_NUMS)/HI551_FULL_PERIOD_LINE_NUMS);

		}
		HI551DB("[HI551SetVideoMode]current fps (10 base)= %d\n", (HI551.capPclk*10000)*10/(HI551_FULL_PERIOD_PIXEL_NUMS + HI551.DummyPixels)/MIN_Frame_length);

		if(HI551.shutter + 4 > MIN_Frame_length)
				MIN_Frame_length = HI551.shutter + 4;


		extralines = MIN_Frame_length - HI551_FULL_PERIOD_LINE_NUMS;

		spin_lock(&HI551mipiraw_drv_lock);
		HI551.DummyPixels = 0;//define dummy pixels and lines
		HI551.DummyLines = extralines ;
		spin_unlock(&HI551mipiraw_drv_lock);

		HI551_SetDummy(HI551.DummyPixels,extralines);
	}
	HI551DB("[HI551SetVideoMode]MIN_Frame_length=%d,HI551.DummyLines=%d\n",MIN_Frame_length,HI551.DummyLines);

    return KAL_TRUE;
}

UINT32 HI551SetAutoFlickerMode(kal_bool bEnable, UINT16 u2FrameRate)
{
	//return ERROR_NONE;

    //HI551DB("[HI551SetAutoFlickerMode] frame rate(10base) = %d %d\n", bEnable, u2FrameRate);
	if(bEnable) {   // enable auto flicker
		spin_lock(&HI551mipiraw_drv_lock);
		HI551.HI551AutoFlickerMode = KAL_TRUE;
		spin_unlock(&HI551mipiraw_drv_lock);
    } else {
    	spin_lock(&HI551mipiraw_drv_lock);
        HI551.HI551AutoFlickerMode = KAL_FALSE;
		spin_unlock(&HI551mipiraw_drv_lock);
        HI551DB("Disable Auto flicker\n");
    }

    return ERROR_NONE;
}

UINT32 HI551SetTestPatternMode(kal_bool bEnable)
{
    HI551DB("[HI551SetTestPatternMode] Test pattern enable:%d\n", bEnable);
#if 1
    if(bEnable) 
    {
        HI551_write_cmos_sensor(0x020a,0x0200);        
    }
    else
    {
        HI551_write_cmos_sensor(0x020a,0x0000);  

    }
#endif
    return ERROR_NONE;
}

UINT32 HI551MIPISetMaxFramerateByScenario(MSDK_SCENARIO_ID_ENUM scenarioId, MUINT32 frameRate) {
	kal_uint32 pclk;
	kal_int16 dummyLine;
	kal_uint16 lineLength,frameHeight;
		
	HI551DB("HI551MIPISetMaxFramerateByScenario: scenarioId = %d, frame rate = %d\n",scenarioId,frameRate);
	switch (scenarioId) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			pclk =  HI551_PV_CLK;
			lineLength = HI551_PV_PERIOD_PIXEL_NUMS;
			frameHeight = (10 * pclk)/frameRate/lineLength;
			dummyLine = frameHeight - HI551_PV_PERIOD_LINE_NUMS;
			if(dummyLine<0)
				dummyLine = 0;
			spin_lock(&HI551mipiraw_drv_lock);
			HI551.sensorMode = SENSOR_MODE_PREVIEW;
			spin_unlock(&HI551mipiraw_drv_lock);
			HI551_SetDummy(0, dummyLine);			
			break;			
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			pclk =  HI551_VIDEO_CLK;
			lineLength = HI551_VIDEO_PERIOD_PIXEL_NUMS;
			frameHeight = (10 * pclk)/frameRate/lineLength;
			dummyLine = frameHeight - HI551_VIDEO_PERIOD_LINE_NUMS;
			if(dummyLine<0)
				dummyLine = 0;
			spin_lock(&HI551mipiraw_drv_lock);
			HI551.sensorMode = SENSOR_MODE_VIDEO;
			spin_unlock(&HI551mipiraw_drv_lock);
			HI551_SetDummy(0, dummyLine);			
			break;			
			 break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_CAMERA_ZSD:			
			pclk = HI551_CAP_CLK;
			lineLength = HI551_FULL_PERIOD_PIXEL_NUMS;
			frameHeight = (10 * pclk)/frameRate/lineLength;
			dummyLine = frameHeight - HI551_FULL_PERIOD_LINE_NUMS;
			if(dummyLine<0)
				dummyLine = 0;
			spin_lock(&HI551mipiraw_drv_lock);
			HI551.sensorMode = SENSOR_MODE_CAPTURE;
			spin_unlock(&HI551mipiraw_drv_lock);
			HI551_SetDummy(0, dummyLine);			
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


UINT32 HI551MIPIGetDefaultFramerateByScenario(MSDK_SCENARIO_ID_ENUM scenarioId, MUINT32 *pframeRate) 
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



UINT32 HI551FeatureControl(MSDK_SENSOR_FEATURE_ENUM FeatureId,
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
            *pFeatureReturnPara16++= HI551_IMAGE_SENSOR_FULL_WIDTH;
            *pFeatureReturnPara16= HI551_IMAGE_SENSOR_FULL_HEIGHT;
            *pFeatureParaLen=4;
            break;
        case SENSOR_FEATURE_GET_PERIOD:
				*pFeatureReturnPara16++= HI551_FeatureControl_PERIOD_PixelNum;
				*pFeatureReturnPara16= HI551_FeatureControl_PERIOD_LineNum;
				*pFeatureParaLen=4;
				break;
        case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
			switch(HI551CurrentScenarioId)
			{
				case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
					*pFeatureReturnPara32 =  HI551_PV_CLK;
					*pFeatureParaLen=4;
					break;
				case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
					*pFeatureReturnPara32 =  HI551_VIDEO_CLK;
					*pFeatureParaLen=4;
					break;
				case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
				case MSDK_SCENARIO_ID_CAMERA_ZSD:
					*pFeatureReturnPara32 = HI551_CAP_CLK;
					*pFeatureParaLen=4;
					break;
				default:
					*pFeatureReturnPara32 =  HI551_CAP_CLK;
					*pFeatureParaLen=4;
					break;
			}
		    break;

        case SENSOR_FEATURE_SET_ESHUTTER:
            HI551_SetShutter(*pFeatureData16);
            break;
        case SENSOR_FEATURE_SET_NIGHTMODE:
            HI551_NightMode((BOOL) *pFeatureData16);
            break;
        case SENSOR_FEATURE_SET_GAIN:
            HI551_SetGain((UINT16) *pFeatureData16);
            break;
        case SENSOR_FEATURE_SET_FLASHLIGHT:
            break;
        case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
            //HI551_isp_master_clock=*pFeatureData32;
            break;
        case SENSOR_FEATURE_SET_REGISTER:
            HI551_write_cmos_sensor(pSensorRegData->RegAddr, pSensorRegData->RegData);
            break;
        case SENSOR_FEATURE_GET_REGISTER:
            pSensorRegData->RegData = HI551_read_cmos_sensor(pSensorRegData->RegAddr);
            break;
        case SENSOR_FEATURE_SET_CCT_REGISTER:
            SensorRegNumber=FACTORY_END_ADDR;
            for (i=0;i<SensorRegNumber;i++)
            {
            	spin_lock(&HI551mipiraw_drv_lock);
                HI551SensorCCT[i].Addr=*pFeatureData32++;
                HI551SensorCCT[i].Para=*pFeatureData32++;
				spin_unlock(&HI551mipiraw_drv_lock);
            }
            break;
        case SENSOR_FEATURE_GET_CCT_REGISTER:
            SensorRegNumber=FACTORY_END_ADDR;
            if (*pFeatureParaLen<(SensorRegNumber*sizeof(SENSOR_REG_STRUCT)+4))
                return FALSE;
            *pFeatureData32++=SensorRegNumber;
            for (i=0;i<SensorRegNumber;i++)
            {
                *pFeatureData32++=HI551SensorCCT[i].Addr;
                *pFeatureData32++=HI551SensorCCT[i].Para;
            }
            break;
        case SENSOR_FEATURE_SET_ENG_REGISTER:
            SensorRegNumber=ENGINEER_END;
            for (i=0;i<SensorRegNumber;i++)
            {
            	spin_lock(&HI551mipiraw_drv_lock);
                HI551SensorReg[i].Addr=*pFeatureData32++;
                HI551SensorReg[i].Para=*pFeatureData32++;
				spin_unlock(&HI551mipiraw_drv_lock);
            }
            break;
        case SENSOR_FEATURE_GET_ENG_REGISTER:
            SensorRegNumber=ENGINEER_END;
            if (*pFeatureParaLen<(SensorRegNumber*sizeof(SENSOR_REG_STRUCT)+4))
                return FALSE;
            *pFeatureData32++=SensorRegNumber;
            for (i=0;i<SensorRegNumber;i++)
            {
                *pFeatureData32++=HI551SensorReg[i].Addr;
                *pFeatureData32++=HI551SensorReg[i].Para;
            }
            break;
        case SENSOR_FEATURE_GET_REGISTER_DEFAULT:
            if (*pFeatureParaLen>=sizeof(NVRAM_SENSOR_DATA_STRUCT))
            {
                pSensorDefaultData->Version=NVRAM_CAMERA_SENSOR_FILE_VERSION;
                pSensorDefaultData->SensorId=HI551MIPI_SENSOR_ID;
                memcpy(pSensorDefaultData->SensorEngReg, HI551SensorReg, sizeof(SENSOR_REG_STRUCT)*ENGINEER_END);
                memcpy(pSensorDefaultData->SensorCCTReg, HI551SensorCCT, sizeof(SENSOR_REG_STRUCT)*FACTORY_END_ADDR);
            }
            else
                return FALSE;
            *pFeatureParaLen=sizeof(NVRAM_SENSOR_DATA_STRUCT);
            break;
        case SENSOR_FEATURE_GET_CONFIG_PARA:
            memcpy(pSensorConfigData, &HI551SensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
            *pFeatureParaLen=sizeof(MSDK_SENSOR_CONFIG_STRUCT);
            break;
        case SENSOR_FEATURE_CAMERA_PARA_TO_SENSOR:
            HI551_camera_para_to_sensor();
            break;

        case SENSOR_FEATURE_SENSOR_TO_CAMERA_PARA:
            HI551_sensor_to_camera_para();
            break;
        case SENSOR_FEATURE_GET_GROUP_COUNT:
            *pFeatureReturnPara32++=HI551_get_sensor_group_count();
            *pFeatureParaLen=4;
            break;
        case SENSOR_FEATURE_GET_GROUP_INFO:
            HI551_get_sensor_group_info(pSensorGroupInfo->GroupIdx, pSensorGroupInfo->GroupNamePtr, &pSensorGroupInfo->ItemCount);
            *pFeatureParaLen=sizeof(MSDK_SENSOR_GROUP_INFO_STRUCT);
            break;
        case SENSOR_FEATURE_GET_ITEM_INFO:
            HI551_get_sensor_item_info(pSensorItemInfo->GroupIdx,pSensorItemInfo->ItemIdx, pSensorItemInfo);
            *pFeatureParaLen=sizeof(MSDK_SENSOR_ITEM_INFO_STRUCT);
            break;

        case SENSOR_FEATURE_SET_ITEM_INFO:
            HI551_set_sensor_item_info(pSensorItemInfo->GroupIdx, pSensorItemInfo->ItemIdx, pSensorItemInfo->ItemValue);
            *pFeatureParaLen=sizeof(MSDK_SENSOR_ITEM_INFO_STRUCT);
            break;

        case SENSOR_FEATURE_GET_ENG_INFO:
            pSensorEngInfo->SensorId = 129;
            pSensorEngInfo->SensorType = CMOS_SENSOR;
            pSensorEngInfo->SensorOutputDataFormat=SENSOR_OUTPUT_FORMAT_RAW_Gb;
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
            HI551SetVideoMode(*pFeatureData16);
            break;
        case SENSOR_FEATURE_CHECK_SENSOR_ID:
            HI551GetSensorID(pFeatureReturnPara32);
            break;
        case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE:
            HI551SetAutoFlickerMode((BOOL)*pFeatureData16, *(pFeatureData16+1));
	        break;
        case SENSOR_FEATURE_SET_TEST_PATTERN:
            HI551SetTestPatternMode((BOOL)*pFeatureData16);
            break;
		case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
			HI551MIPISetMaxFramerateByScenario((MSDK_SCENARIO_ID_ENUM)*pFeatureData32, *(pFeatureData32+1));
			break;
		case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
			HI551MIPIGetDefaultFramerateByScenario((MSDK_SCENARIO_ID_ENUM)*pFeatureData32, (MUINT32 *)(*(pFeatureData32+1)));
            break;       
        case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE://for factory mode auto testing             
            *pFeatureReturnPara32=HI551_TEST_PATTERN_CHECKSUM;           
            *pFeatureParaLen=4;                             
        break;     
        default:
            break;
    }
    return ERROR_NONE;
}	/* HI551FeatureControl() */


SENSOR_FUNCTION_STRUCT	SensorFuncHI551=
{
    HI551Open,
    HI551GetInfo,
    HI551GetResolution,
    HI551FeatureControl,
    HI551Control,
    HI551Close
};

UINT32 HI551_MIPI_RAW_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
    /* To Do : Check Sensor status here */
    if (pfFunc!=NULL)
        *pfFunc=&SensorFuncHI551;

    return ERROR_NONE;
}   /* SensorInit() */

