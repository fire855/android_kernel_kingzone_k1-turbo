/*******************************************************************************************/
      
   
/*******************************************************************************************/

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

#include "s5k3h5xamipiraw_Sensor.h"
#include "s5k3h5xamipiraw_Camera_Sensor_para.h"
#include "s5k3h5xamipiraw_CameraCustomized.h"
static DEFINE_SPINLOCK(s5k3h5xamipiraw_drv_lock);

#define S5K3H5XA_DEBUG

#ifdef S5K3H5XA_DEBUG
	#define S5K3H5XADB(fmt, arg...) xlog_printk(ANDROID_LOG_DEBUG, "[S5K3H5XAMIPI]" , fmt, ##arg)
#else
	#define S5K3H5XADB(x,...)
#endif

#define mDELAY(ms)  mdelay(ms)
#define S5K3H5XA_TEST_PATTERN_CHECKSUM (0xe6828796)

kal_uint32 s5k3h5xa_video_frame_length = 2530;
kal_uint32 s5k3h5xa_video_line_length = 3688;
kal_uint32 s5k3h5xa_preview_frame_length = 2530;
kal_uint32 s5k3h5xa_preview_line_length = 3688;
kal_uint32 s5k3h5xa_capture_frame_length =2530;
kal_uint32 s5k3h5xa_capture_line_length = 3688;


kal_uint32 S5K3H5XA_FeatureControl_PERIOD_PixelNum;
kal_uint32 S5K3H5XA_FeatureControl_PERIOD_LineNum;
kal_uint32 S5H3H5XA_FeatureControl_PERIOD_PixelNum;

kal_uint32 cont_preview_line_length = 3688;
kal_uint32 cont_preview_frame_length = 2530;
kal_uint32 cont_capture_line_length = 3688;
kal_uint32 cont_capture_frame_length = 2530;

kal_uint32 cont_video_frame_length = 2530;
MSDK_SENSOR_CONFIG_STRUCT S5K3H5XASensorConfigData;

kal_uint32 S5K3H5XA_FAC_SENSOR_REG;

MSDK_SCENARIO_ID_ENUM S5K3H5XACurrentScenarioId = MSDK_SCENARIO_ID_CAMERA_PREVIEW;

/* FIXME: old factors and DIDNOT use now. s*/
SENSOR_REG_STRUCT S5K3H5XASensorCCT[]=CAMERA_SENSOR_CCT_DEFAULT_VALUE;
SENSOR_REG_STRUCT S5K3H5XASensorReg[ENGINEER_END]=CAMERA_SENSOR_REG_DEFAULT_VALUE;
/* FIXME: old factors and DIDNOT use now. e*/

static S5K3H5XA_PARA_STRUCT s5k3h5xa;

extern int iWriteRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u16 i2cId);
extern int iReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 * a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
UINT32 S5K3H5XAMIPISetMaxFramerateByScenario(MSDK_SCENARIO_ID_ENUM scenarioId, MUINT32 frameRate);

inline kal_uint16 S5K3H5XA_read_cmos_sensor(kal_uint32 addr)
{
	kal_uint16 get_byte=0;
	char puSendCmd[2] = {(char)(addr >> 8) , (char)(addr & 0xFF) };
	//iReadRegI2C(puSendCmd , 2, (u8*)&get_byte,2,S5K3H5XAMIPI_WRITE_ID);
	//return ((get_byte<<8)&0xff00)|((get_byte>>8)&0x00ff);
	iReadRegI2C(puSendCmd , 2, (u8*)&get_byte,1,S5K3H5XAMIPI_WRITE_ID);
	return get_byte;
	
}

inline void S5K3H5XA_wordwrite_cmos_sensor(u16 addr, u32 para)
{
	char puSendCmd[4] = {(char)(addr >> 8) , (char)(addr & 0xFF) ,  (char)(para >> 8),	(char)(para & 0xFF) };
	iWriteRegI2C(puSendCmd , 4,S5K3H5XAMIPI_WRITE_ID);
}

inline void S5K3H5XA_bytewrite_cmos_sensor(u16 addr, u32 para)
{
	char puSendCmd[4] = {(char)(addr >> 8) , (char)(addr & 0xFF)  ,	(char)(para & 0xFF) };
	iWriteRegI2C(puSendCmd , 3,S5K3H5XAMIPI_WRITE_ID);
}

inline void S5K3H5XA_write_cmos_sensor(u16 addr, u32 para)
{
	if(para & 0xff00 == 0)
		S5K3H5XA_bytewrite_cmos_sensor(addr,para);
	else
		S5K3H5XA_wordwrite_cmos_sensor(addr,para);
}
#define Sleep(ms) mdelay(ms)

//#define S5K3H5XA_USE_AWB_OTP
/*
#if defined(S5K3H5XA_USE_AWB_OTP)

#define RG_TYPICAL 0x2a1
#define BG_TYPICAL 0x23f


kal_uint32 tRG_Ratio_typical = RG_TYPICAL;
kal_uint32 tBG_Ratio_typical = BG_TYPICAL;


void S5K3H5XA_MIPI_read_otp_wb(struct S5K3H5XA_MIPI_otp_struct *otp)
{
	kal_uint32 R_to_G, B_to_G, G_to_G;
	kal_uint16 PageCount;
	for(PageCount = 4; PageCount>=1; PageCount--)
	{
		S5K3H5XADB("[S5K3H5XA] [S5K3H5XA_MIPI_read_otp_wb] PageCount=%d\n", PageCount);	
		S5K3H5XA_write_cmos_sensor(0x3a02, PageCount); //page set
		S5K3H5XA_write_cmos_sensor(0x3a00, 0x01); //otp enable read
		R_to_G = (S5K3H5XA_read_cmos_sensor(0x3a09)<<8)+S5K3H5XA_read_cmos_sensor(0x3a0a);
		B_to_G = (S5K3H5XA_read_cmos_sensor(0x3a0b)<<8)+S5K3H5XA_read_cmos_sensor(0x3a0c);
		G_to_G = (S5K3H5XA_read_cmos_sensor(0x3a0d)<<8)+S5K3H5XA_read_cmos_sensor(0x3a0e);
		S5K3H5XA_write_cmos_sensor(0x3a00, 0x00); //otp disable read

		if((R_to_G != 0) && (B_to_G != 0) && (G_to_G != 0))
			break;	

		if(PageCount == 1)
			S5K3H5XADB("[S5K3H5XA] [S5K3H5XA_MIPI_read_otp_wb] otp all value is zero");
	}

	otp->R_to_G = R_to_G;
	otp->B_to_G = B_to_G;
	otp->G_to_G = 0x400;
	S5K3H5XADB("[S5K3H5XA] [S5K3H5XA_MIPI_read_otp_wb] otp->R_to_G=0x%x\n", otp->R_to_G);	
	S5K3H5XADB("[S5K3H5XA] [S5K3H5XA_MIPI_read_otp_wb] otp->B_to_G=0x%x\n", otp->B_to_G);	
	S5K3H5XADB("[S5K3H5XA] [S5K3H5XA_MIPI_read_otp_wb] otp->G_to_G=0x%x\n", otp->G_to_G);	
}

void S5K3H5XA_MIPI_algorithm_otp_wb1(struct S5K3H5XA_MIPI_otp_struct *otp)
{
	kal_uint32 R_to_G, B_to_G, G_to_G;
	kal_uint32 R_Gain, B_Gain, G_Gain;
	kal_uint32 G_gain_R, G_gain_B;
	
	R_to_G = otp->R_to_G;
	B_to_G = otp->B_to_G;
	G_to_G = otp->G_to_G;

	S5K3H5XADB("[S5K3H5XA] [S5K3H5XA_MIPI_read_otp_wb] R_to_G=%d\n", R_to_G);	
	S5K3H5XADB("[S5K3H5XA] [S5K3H5XA_MIPI_read_otp_wb] B_to_G=%d\n", B_to_G);	
	S5K3H5XADB("[S5K3H5XA] [S5K3H5XA_MIPI_read_otp_wb] G_to_G=%d\n", G_to_G);

	if(B_to_G < tBG_Ratio_typical)
		{
			if(R_to_G < tRG_Ratio_typical)
				{
					G_Gain = 0x100;
					B_Gain = 0x100 * tBG_Ratio_typical / B_to_G;
					R_Gain = 0x100 * tRG_Ratio_typical / R_to_G;
				}
			else
				{
			        R_Gain = 0x100;
					G_Gain = 0x100 * R_to_G / tRG_Ratio_typical;
					B_Gain = G_Gain * tBG_Ratio_typical / B_to_G;	        
				}
		}
	else
		{
			if(R_to_G < tRG_Ratio_typical)
				{
			        B_Gain = 0x100;
					G_Gain = 0x100 * B_to_G / tBG_Ratio_typical;
					R_Gain = G_Gain * tRG_Ratio_typical / R_to_G;
				}
			else
				{
			        G_gain_B = 0x100*B_to_G / tBG_Ratio_typical;
				    G_gain_R = 0x100*R_to_G / tRG_Ratio_typical;
					
					if(G_gain_B > G_gain_R)
						{
							B_Gain = 0x100;
							G_Gain = G_gain_B;
							R_Gain = G_Gain * tRG_Ratio_typical / R_to_G;
						}
					else
						{
							R_Gain = 0x100;
							G_Gain = G_gain_R;
							B_Gain = G_Gain * tBG_Ratio_typical / B_to_G;
						}        
				}	
		}

	otp->R_Gain = R_Gain;
	otp->B_Gain = B_Gain;
	otp->G_Gain = G_Gain;

	S5K3H5XADB("[S5K3H5XA] [S5K3H5XA_MIPI_read_otp_wb] R_gain=0x%x\n", otp->R_Gain);	
	S5K3H5XADB("[S5K3H5XA] [S5K3H5XA_MIPI_read_otp_wb] B_gain=0x%x\n", otp->B_Gain);	
	S5K3H5XADB("[S5K3H5XA] [S5K3H5XA_MIPI_read_otp_wb] G_gain=0x%x\n", otp->G_Gain);
}



void S5K3H5XA_MIPI_write_otp_wb(struct S5K3H5XA_MIPI_otp_struct *otp)
{
	kal_uint16 R_GainH, B_GainH, G_GainH;
	kal_uint16 R_GainL, B_GainL, G_GainL;
	kal_uint32 temp;

	temp = otp->R_Gain;
	R_GainH = (temp & 0xff00)>>8;
	temp = otp->R_Gain;
	R_GainL = (temp & 0x00ff);

	temp = otp->B_Gain;
	B_GainH = (temp & 0xff00)>>8;
	temp = otp->B_Gain;
	B_GainL = (temp & 0x00ff);

	temp = otp->G_Gain;
	G_GainH = (temp & 0xff00)>>8;
	temp = otp->G_Gain;
	G_GainL = (temp & 0x00ff);

	S5K3H5XADB("[S5K3H5XA] [S5K3H5XA_MIPI_read_otp_wb] R_GainH=0x%x\n", R_GainH);	
	S5K3H5XADB("[S5K3H5XA] [S5K3H5XA_MIPI_read_otp_wb] R_GainL=0x%x\n", R_GainL);	
	S5K3H5XADB("[S5K3H5XA] [S5K3H5XA_MIPI_read_otp_wb] B_GainH=0x%x\n", B_GainH);
	S5K3H5XADB("[S5K3H5XA] [S5K3H5XA_MIPI_read_otp_wb] B_GainL=0x%x\n", B_GainL);	
	S5K3H5XADB("[S5K3H5XA] [S5K3H5XA_MIPI_read_otp_wb] G_GainH=0x%x\n", G_GainH);	
	S5K3H5XADB("[S5K3H5XA] [S5K3H5XA_MIPI_read_otp_wb] G_GainL=0x%x\n", G_GainL);

	S5K3H5XA_write_cmos_sensor(0x020e, G_GainH);
	S5K3H5XA_write_cmos_sensor(0x020f, G_GainL);
	S5K3H5XA_write_cmos_sensor(0x0210, R_GainH);
	S5K3H5XA_write_cmos_sensor(0x0211, R_GainL);
	S5K3H5XA_write_cmos_sensor(0x0212, B_GainH);
	S5K3H5XA_write_cmos_sensor(0x0213, B_GainL);
	S5K3H5XA_write_cmos_sensor(0x0214, G_GainH);
	S5K3H5XA_write_cmos_sensor(0x0215, G_GainL);

	S5K3H5XADB("[S5K3H5XA] [S5K3H5XA_MIPI_read_otp_wb] [0x020e,0x%x]\n", S5K3H5XA_read_cmos_sensor(0x020e));	
	S5K3H5XADB("[S5K3H5XA] [S5K3H5XA_MIPI_read_otp_wb] [0x020f,0x%x]\n", S5K3H5XA_read_cmos_sensor(0x020f));	
	S5K3H5XADB("[S5K3H5XA] [S5K3H5XA_MIPI_read_otp_wb] [0x0210,0x%x]\n", S5K3H5XA_read_cmos_sensor(0x0210));
	S5K3H5XADB("[S5K3H5XA] [S5K3H5XA_MIPI_read_otp_wb] [0x0211,0x%x]\n", S5K3H5XA_read_cmos_sensor(0x0211));	
	S5K3H5XADB("[S5K3H5XA] [S5K3H5XA_MIPI_read_otp_wb] [0x0212,0x%x]\n", S5K3H5XA_read_cmos_sensor(0x0212));	
	S5K3H5XADB("[S5K3H5XA] [S5K3H5XA_MIPI_read_otp_wb] [0x0213,0x%x]\n", S5K3H5XA_read_cmos_sensor(0x0213));
	S5K3H5XADB("[S5K3H5XA] [S5K3H5XA_MIPI_read_otp_wb] [0x0214,0x%x]\n", S5K3H5XA_read_cmos_sensor(0x0214));	
	S5K3H5XADB("[S5K3H5XA] [S5K3H5XA_MIPI_read_otp_wb] [0x0215,0x%x]\n", S5K3H5XA_read_cmos_sensor(0x0215));
}

void S5K3H5XA_MIPI_update_wb_register_from_otp(void)
{
	struct S5K3H5XA_MIPI_otp_struct current_otp;
	S5K3H5XA_MIPI_read_otp_wb(&current_otp);
	S5K3H5XA_MIPI_algorithm_otp_wb1(&current_otp);
	S5K3H5XA_MIPI_write_otp_wb(&current_otp);
}
#endif
*/

void S5K3H5XA_write_shutter(kal_uint32 shutter)
{
	kal_uint32 frame_length = 0;
	kal_uint32 line_length_read;

	S5K3H5XADB("[S5K3H5XA] [S5K3H5XA_write_shutter] shutter=%d\n", shutter);

  if (shutter < 3)
	  shutter = 3;

  if (s5k3h5xa.sensorMode == SENSOR_MODE_PREVIEW) 
  {
  	S5K3H5XADB("[S5K3H5XA] [S5K3H5XA_write_shutter] SENSOR_MODE_PREVIEW \n");
	  if(shutter > (cont_preview_frame_length - 16))
		  frame_length = shutter + 16;
	  else 
		  frame_length = cont_preview_frame_length;

  }
  else if(s5k3h5xa.sensorMode==SENSOR_MODE_VIDEO)
  {
	   	if(cont_video_frame_length <= (shutter + 16))
			frame_length = shutter + 16;
		else
			frame_length = cont_video_frame_length;
	   S5K3H5XADB("[S5K3H5XA] [S5K3H5XA_write_shutter] SENSOR_MODE_VIDEO , shutter = %d, frame_length = %d\n",shutter, frame_length);
  }
  else
  {
  	S5K3H5XADB("[S5K3H5XA] [S5K3H5XA_write_shutter] SENSOR_MODE_CAPTURE \n");
	  if(shutter > (cont_capture_frame_length - 16))
		  frame_length = shutter + 16;
	  else 
		  frame_length = cont_capture_frame_length;
  }
  
 	S5K3H5XA_bytewrite_cmos_sensor(0x0104, 0x01);    //Grouped parameter hold    

	S5K3H5XA_bytewrite_cmos_sensor(0x0340, (frame_length >>8) & 0xFF);
 	S5K3H5XA_bytewrite_cmos_sensor(0x0341, frame_length & 0xFF);	 

 	S5K3H5XA_bytewrite_cmos_sensor(0x0202, (shutter >> 8) & 0xFF);
 	S5K3H5XA_bytewrite_cmos_sensor(0x0203, shutter  & 0xFF);
 
 	S5K3H5XA_bytewrite_cmos_sensor(0x0104, 0x00);    //Grouped parameter release

	line_length_read = ((S5K3H5XA_read_cmos_sensor(0x0342)<<8)+S5K3H5XA_read_cmos_sensor(0x0343));

	S5K3H5XADB("[S5K3H5XA] [S5K3H5XA_write_shutter] shutter=%d,  line_length_read=%d, frame_length=%d\n", shutter, line_length_read, frame_length);
}   /* write_S5K3H5XA_shutter */


void write_S5K3H5XA_gain(kal_uint16 gain)
{
	S5K3H5XADB("[S5K3H5XA] [write_S5K3H5XA_gain] gain=%d\n", gain);
	gain = gain / 2;
	S5K3H5XA_bytewrite_cmos_sensor(0x0104, 0x01);	
	S5K3H5XA_bytewrite_cmos_sensor(0x0204,(gain>>8));
	S5K3H5XA_bytewrite_cmos_sensor(0x0205,(gain&0xff));
	S5K3H5XA_bytewrite_cmos_sensor(0x0104, 0x00);
	return;
}

/*************************************************************************
* FUNCTION
*    S5K3H5XA_SetGain
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
void S5K3H5XA_SetGain(UINT16 iGain)
{
	unsigned long flags;
	spin_lock_irqsave(&s5k3h5xamipiraw_drv_lock,flags);
	s5k3h5xa.realGain = iGain;
	spin_unlock_irqrestore(&s5k3h5xamipiraw_drv_lock,flags);
	S5K3H5XADB("[S5K3H5XA] [S5K3H5XA_SetGain] gain=%d\n", iGain);
	write_S5K3H5XA_gain(iGain);
}   /*  S5K3H5XA_SetGain_SetGain  */


/*************************************************************************
* FUNCTION
*    read_S5K3H5XA_gain
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
kal_uint16 read_S5K3H5XA_gain(void)
{
	kal_uint16 read_gain=0;

	read_gain=((S5K3H5XA_read_cmos_sensor(0x0204) << 8) | S5K3H5XA_read_cmos_sensor(0x0205));
	S5K3H5XADB("[S5K3H5XA] [read_S5K3H5XA_gain] gain=%d\n", read_gain);
	spin_lock(&s5k3h5xamipiraw_drv_lock);
	s5k3h5xa.sensorGlobalGain = read_gain ;
	spin_unlock(&s5k3h5xamipiraw_drv_lock);
	return s5k3h5xa.sensorGlobalGain;
}  /* read_S5K3H5XA_gain */


void S5K3H5XA_camera_para_to_sensor(void)
{
    kal_uint32    i;
    for(i=0; 0xFFFFFFFF!=S5K3H5XASensorReg[i].Addr; i++)
    {
        S5K3H5XA_write_cmos_sensor(S5K3H5XASensorReg[i].Addr, S5K3H5XASensorReg[i].Para);
    }
    for(i=ENGINEER_START_ADDR; 0xFFFFFFFF!=S5K3H5XASensorReg[i].Addr; i++)
    {
        S5K3H5XA_write_cmos_sensor(S5K3H5XASensorReg[i].Addr, S5K3H5XASensorReg[i].Para);
    }
    for(i=FACTORY_START_ADDR; i<FACTORY_END_ADDR; i++)
    {
        S5K3H5XA_write_cmos_sensor(S5K3H5XASensorCCT[i].Addr, S5K3H5XASensorCCT[i].Para);
    }
}


/*************************************************************************
* FUNCTION
*    S5K3H5XA_sensor_to_camera_para
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
void S5K3H5XA_sensor_to_camera_para(void)
{
    kal_uint32    i, temp_data;
    for(i=0; 0xFFFFFFFF!=S5K3H5XASensorReg[i].Addr; i++)
    {
         temp_data = S5K3H5XA_read_cmos_sensor(S5K3H5XASensorReg[i].Addr);
		 spin_lock(&s5k3h5xamipiraw_drv_lock);
		 S5K3H5XASensorReg[i].Para =temp_data;
		 spin_unlock(&s5k3h5xamipiraw_drv_lock);
    }
    for(i=ENGINEER_START_ADDR; 0xFFFFFFFF!=S5K3H5XASensorReg[i].Addr; i++)
    {
        temp_data = S5K3H5XA_read_cmos_sensor(S5K3H5XASensorReg[i].Addr);
		spin_lock(&s5k3h5xamipiraw_drv_lock);
		S5K3H5XASensorReg[i].Para = temp_data;
		spin_unlock(&s5k3h5xamipiraw_drv_lock);
    }
}

/*************************************************************************
* FUNCTION
*    S5K3H5XA_get_sensor_group_count
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
kal_int32  S5K3H5XA_get_sensor_group_count(void)
{
    return GROUP_TOTAL_NUMS;
}

void S5K3H5XA_get_sensor_group_info(kal_uint16 group_idx, kal_int8* group_name_ptr, kal_int32* item_count_ptr)
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

void S5K3H5XA_get_sensor_item_info(kal_uint16 group_idx,kal_uint16 item_idx, MSDK_SENSOR_ITEM_INFO_STRUCT* info_ptr)
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

            temp_para= S5K3H5XASensorCCT[temp_addr].Para;
			//temp_gain= (temp_para/s5k3h5xa.sensorBaseGain) * 1000;

            info_ptr->ItemValue=temp_gain;
            info_ptr->IsTrueFalse=KAL_FALSE;
            info_ptr->IsReadOnly=KAL_FALSE;
            info_ptr->IsNeedRestart=KAL_FALSE;
            info_ptr->Min= S5K3H5XA_MIN_ANALOG_GAIN * 1000;
            info_ptr->Max= S5K3H5XA_MAX_ANALOG_GAIN * 1000;
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



kal_bool S5K3H5XA_set_sensor_item_info(kal_uint16 group_idx, kal_uint16 item_idx, kal_int32 ItemValue)
{
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
//             temp_para=(temp_gain * S5K3H5XA.sensorBaseGain + BASEGAIN/2)/BASEGAIN;
          }
          else
			  ASSERT(0);
		  spin_lock(&s5k3h5xamipiraw_drv_lock);
          S5K3H5XASensorCCT[temp_addr].Para = temp_para;
		  spin_unlock(&s5k3h5xamipiraw_drv_lock);
          S5K3H5XA_write_cmos_sensor(S5K3H5XASensorCCT[temp_addr].Addr,temp_para);

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
					spin_lock(&s5k3h5xamipiraw_drv_lock);
                    S5K3H5XA_FAC_SENSOR_REG=ItemValue;
					spin_unlock(&s5k3h5xamipiraw_drv_lock);
                    break;
                case 1:
                    S5K3H5XA_write_cmos_sensor(S5K3H5XA_FAC_SENSOR_REG,ItemValue);
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

static void S5K3H5XA_SetDummy( const kal_uint32 iPixels, const kal_uint32 iLines )
{
kal_uint16 line_length = 0;
kal_uint16 frame_length = 0;

S5K3H5XADB("[S5K3H5XA] [S5K3H5XA_SetDummy] iPixels=%d\n", iPixels);
S5K3H5XADB("[S5K3H5XA] [S5K3H5XA_SetDummy] iLines=%d\n", iLines);

if ( SENSOR_MODE_PREVIEW == s5k3h5xa.sensorMode )	
{
	S5K3H5XADB("[S5K3H5XA] [S5K3H5XA_SetDummy] SENSOR_MODE_PREVIEW\n");
	line_length = s5k3h5xa_preview_line_length ;
	frame_length = s5k3h5xa_preview_frame_length + iLines;
}
else if( SENSOR_MODE_VIDEO == s5k3h5xa.sensorMode )		
{
	S5K3H5XADB("[S5K3H5XA] [S5K3H5XA_SetDummy] SENSOR_MODE_VIDEO\n");
	line_length = s5k3h5xa_video_line_length;
	frame_length = s5k3h5xa_video_frame_length + iLines; //modify video night mode fps
	cont_video_frame_length = frame_length;
}
else
{
	S5K3H5XADB("[S5K3H5XA] [S5K3H5XA_SetDummy] SENSOR_MODE_CAPTURE\n");
	line_length = s5k3h5xa_capture_line_length ;
	frame_length = s5k3h5xa_capture_frame_length + iLines;
}

if(s5k3h5xa.maxExposureLines > frame_length - 16)
{
	S5K3H5XADB("[S5K3H5XA] [S5K3H5XA_SetDummy] maxExposureLines > frame_length - 16\n");
	return;
}

ASSERT(line_length < S5K3H5XA_MAX_LINE_LENGTH); 	//0xCCCC
ASSERT(frame_length < S5K3H5XA_MAX_FRAME_LENGTH);	//0xFFFF

S5K3H5XA_bytewrite_cmos_sensor(0x0104, 0x01);	//Grouped parameter hold

//Set total frame length
S5K3H5XADB("[S5K3H5XA] [S5K3H5XA_SetDummy] frame_length = %d\n", frame_length);

S5K3H5XA_bytewrite_cmos_sensor(0x0340, (frame_length >> 8) & 0xFF);
S5K3H5XA_bytewrite_cmos_sensor(0x0341, frame_length & 0xFF);

//Set total line length
S5K3H5XADB("[S5K3H5XA] [S5K3H5XA_SetDummy] line_length = %d\n", line_length);

S5K3H5XA_bytewrite_cmos_sensor(0x0342, (line_length >> 8) & 0xFF);
S5K3H5XA_bytewrite_cmos_sensor(0x0343, line_length & 0xFF);

S5K3H5XA_bytewrite_cmos_sensor(0x0104, 0x00);	//Grouped parameter release
}   /*  S5K3H5XA_SetDummy */

void S5K3H5XAPreviewSetting(void)
{
  S5K3H5XADB("[S5K3H5XA] [S5K3H5XAPreviewSetting]\n");
  S5K3H5XA_wordwrite_cmos_sensor(0xFCFC, 0xD000);
  S5K3H5XA_wordwrite_cmos_sensor(0x6010, 0x0001);
  Sleep(10);
  S5K3H5XA_wordwrite_cmos_sensor(0x6028, 0x7000);
  S5K3H5XA_wordwrite_cmos_sensor(0x602a, 0x1870);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x2DE9);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xF84F);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x9FE5);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x6081);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xA0E1);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0090);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x98E5);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x00A0);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x9FE5);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x5071);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x9FE5);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x5401);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xD7E1);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xBE5E);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xD0E1);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xBC03);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xDAE5);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x9D10);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xDAE5);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x9FB0);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x40E0);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0100);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x00E0);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x9500);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x55E1);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0B00);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xA0E1);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x2005);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x002A);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x1B00);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x5BE1);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0000);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x008A);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0B00);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xA0E3);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0040);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xC7E1);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xBEBE);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x87E0);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0461);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x96E5);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xF800);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xA0E1);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0B10);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x00E0);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x9500);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x00EB);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x4800);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x84E2);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0140);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x54E3);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0400);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x86E5);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xF800);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xFF3A);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xF6FF);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x00EA);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0D00);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xDAE5);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x9E00);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x50E1);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0500);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x002A);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0A00);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xA0E3);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0040);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xC7E1);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xBE0E);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x87E0);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0461);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x96E5);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xF800);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xDAE5);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x9E10);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x00E0);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x9500);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x00EB);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x3900);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x84E2);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0140);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x54E3);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0400);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x86E5);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xF800);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xFF3A);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xF6FF);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x9FE5);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xC000);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x87E2);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xC830);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x8DE5);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0030);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xD0E5);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xB920);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xD0E1);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xB618);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x9FE5);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xB000);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xA0E1);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0930);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x90E5);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x28C0);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x87E2);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xE800);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xA0E1);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0FE0);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x2FE1);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x1CFF);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x50E3);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0000);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x001B);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x2A00);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x9FE5);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x9400);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x98E5);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0010);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xD0E1);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xBA05);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xD1E5);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x9C10);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x9FE5);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x8830);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x50E1);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0100);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xA0E3);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0000);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x002A);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x1300);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x00EA);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0500);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xD1E1);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xB210);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x82E2);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0D22);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xC2E1);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xB010);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x80E2);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0100);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x50E3);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x1400);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x00AA);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0400);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x80E0);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x8010);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x83E0);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x8110);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xD1E1);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xB020);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x52E3);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0000);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xFF1A);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xF4FF);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xBDE8);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xF84F);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x2FE1);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x1EFF);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xD1E1);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xB410);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x82E2);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0D22);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xC2E1);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xB010);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x80E2);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0100);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x50E3);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x1400);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xFFAA);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xF7FF);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x80E0);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x8010);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x83E0);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x8110);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xD1E1);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xB020);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x52E3);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0000);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xFF1A);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xF4FF);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xFFEA);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xF1FF);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0070);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x7014);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0070);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0000);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0070);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x800C);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0070);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x3013);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0070);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xF804);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0070);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x7015);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0070);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x002D);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x1FE5);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x04F0);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0000);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x20E1);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x1FE5);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x04F0);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0000);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x641C);
  S5K3H5XA_wordwrite_cmos_sensor(0x6028, 0xD000);
  S5K3H5XA_wordwrite_cmos_sensor(0x3902, 0x0002);
  S5K3H5XA_wordwrite_cmos_sensor(0x3158, 0x0215);
  S5K3H5XA_wordwrite_cmos_sensor(0x32B4, 0xF4B6);
  S5K3H5XA_wordwrite_cmos_sensor(0x32B6, 0xF466);
  S5K3H5XA_wordwrite_cmos_sensor(0x32B8, 0xF456);
  S5K3H5XA_wordwrite_cmos_sensor(0x32BA, 0xF45E);
  S5K3H5XA_bytewrite_cmos_sensor(0x32BC, 0x10);
  S5K3H5XA_bytewrite_cmos_sensor(0x32BD, 0x00);
  S5K3H5XA_bytewrite_cmos_sensor(0x32BE, 0x00);
  S5K3H5XA_wordwrite_cmos_sensor(0x3338, 0x0214);
  S5K3H5XA_wordwrite_cmos_sensor(0x6218, 0xF1D0);
  S5K3H5XA_wordwrite_cmos_sensor(0x6214, 0xF9F0);
  S5K3H5XA_wordwrite_cmos_sensor(0x6226, 0x0001);
  S5K3H5XA_wordwrite_cmos_sensor(0xF446, 0x0029);
  S5K3H5XA_wordwrite_cmos_sensor(0xF448, 0x001D);
  S5K3H5XA_wordwrite_cmos_sensor(0xF440, 0x0071);
  S5K3H5XA_wordwrite_cmos_sensor(0xF42E, 0x00C1);
  S5K3H5XA_wordwrite_cmos_sensor(0xF42A, 0x0802);
  S5K3H5XA_wordwrite_cmos_sensor(0xB0C8, 0x0044);
  S5K3H5XA_wordwrite_cmos_sensor(0x6226, 0x0000);
  S5K3H5XA_wordwrite_cmos_sensor(0x6218, 0xF9F0);
  S5K3H5XA_wordwrite_cmos_sensor(0x34A2, 0x00D6);
  S5K3H5XA_wordwrite_cmos_sensor(0x34B2, 0x01FA);
  S5K3H5XA_wordwrite_cmos_sensor(0x34CA, 0x00D6);
  S5K3H5XA_wordwrite_cmos_sensor(0x34DA, 0x01FA);
  S5K3H5XA_wordwrite_cmos_sensor(0x3522, 0x00D6);
  S5K3H5XA_wordwrite_cmos_sensor(0x3532, 0x01FA);
  S5K3H5XA_wordwrite_cmos_sensor(0x3254, 0x79D3);
  S5K3H5XA_wordwrite_cmos_sensor(0x3256, 0x79D3);
  S5K3H5XA_wordwrite_cmos_sensor(0x3258, 0x79D3);
  S5K3H5XA_wordwrite_cmos_sensor(0x325A, 0x79D3);
  S5K3H5XA_wordwrite_cmos_sensor(0x325C, 0x79D3);
  S5K3H5XA_wordwrite_cmos_sensor(0x325E, 0x79D3);
  S5K3H5XA_wordwrite_cmos_sensor(0x357A, 0x00BD);
  S5K3H5XA_wordwrite_cmos_sensor(0x32F6, 0x1110);
  S5K3H5XA_bytewrite_cmos_sensor(0x012C, 0x60);
  S5K3H5XA_bytewrite_cmos_sensor(0x012D, 0x4F);
  S5K3H5XA_bytewrite_cmos_sensor(0x012E, 0x2F);
  S5K3H5XA_bytewrite_cmos_sensor(0x012F, 0x40);
  S5K3H5XA_wordwrite_cmos_sensor(0x6028, 0x7000);
  S5K3H5XA_wordwrite_cmos_sensor(0x602A, 0x2D00);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x30F4);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xD370);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xD379);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x12F4);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0500);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0100);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x4638);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0007);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xF004);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x5038);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0000);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x1002);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xF838);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xFAFF);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0000);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x4C38);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x7805);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x9C04);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x78F4);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0700);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0700);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x9AF4);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x3100);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x3100);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x36F4);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0600);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0600);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0000);
  S5K3H5XA_wordwrite_cmos_sensor(0x6028, 0xD000);
  S5K3H5XA_wordwrite_cmos_sensor(0x6226, 0x0001);
  S5K3H5XA_wordwrite_cmos_sensor(0x6100, 0x0003);
  S5K3H5XA_wordwrite_cmos_sensor(0x6110, 0x1CA0);
  S5K3H5XA_wordwrite_cmos_sensor(0x6112, 0x1CA4);
  S5K3H5XA_wordwrite_cmos_sensor(0x6150, 0x172C);
  S5K3H5XA_wordwrite_cmos_sensor(0x6152, 0x1730);
  S5K3H5XA_wordwrite_cmos_sensor(0x6028, 0x7000);
  S5K3H5XA_wordwrite_cmos_sensor(0x602A, 0x172C);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x1FE5);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x04F0);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0070);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x7018);
  S5K3H5XA_wordwrite_cmos_sensor(0x6028, 0xD000);
  S5K3H5XA_wordwrite_cmos_sensor(0x6226, 0x0000);
  S5K3H5XA_bytewrite_cmos_sensor(0x0101, 0x00);
  S5K3H5XA_wordwrite_cmos_sensor(0x0112, 0x0A0A);
  S5K3H5XA_bytewrite_cmos_sensor(0x0114, 0x03);
  S5K3H5XA_bytewrite_cmos_sensor(0x0120, 0x00);
  S5K3H5XA_wordwrite_cmos_sensor(0x0200, 0x0BEF);
  S5K3H5XA_wordwrite_cmos_sensor(0x0202, 0x09D9);
  S5K3H5XA_wordwrite_cmos_sensor(0x0204, 0x0020);
  S5K3H5XA_wordwrite_cmos_sensor(0x0300, 0x0002);
  S5K3H5XA_wordwrite_cmos_sensor(0x0302, 0x0001);
  S5K3H5XA_wordwrite_cmos_sensor(0x0304, 0x0006);
  S5K3H5XA_wordwrite_cmos_sensor(0x0306, 0x008C);
  S5K3H5XA_wordwrite_cmos_sensor(0x0308, 0x0008);
  S5K3H5XA_wordwrite_cmos_sensor(0x030A, 0x0001);
  S5K3H5XA_wordwrite_cmos_sensor(0x030C, 0x0006);
  S5K3H5XA_wordwrite_cmos_sensor(0x030E, 0x00A5);
  S5K3H5XA_wordwrite_cmos_sensor(0x0340, 0x09E2);
  S5K3H5XA_wordwrite_cmos_sensor(0x0342, 0x0E68);
  S5K3H5XA_wordwrite_cmos_sensor(0x32CE, 0x0094);
  S5K3H5XA_wordwrite_cmos_sensor(0x32D0, 0x0024);
  S5K3H5XA_wordwrite_cmos_sensor(0x0344, 0x0008);
  S5K3H5XA_wordwrite_cmos_sensor(0x0346, 0x0008);
  S5K3H5XA_wordwrite_cmos_sensor(0x0348, 0x0CC7);
  S5K3H5XA_wordwrite_cmos_sensor(0x034A, 0x0997);
  S5K3H5XA_wordwrite_cmos_sensor(0x034C, 0x0660);
  S5K3H5XA_wordwrite_cmos_sensor(0x034E, 0x04C8);
  S5K3H5XA_wordwrite_cmos_sensor(0x0380, 0x0001);
  S5K3H5XA_wordwrite_cmos_sensor(0x0382, 0x0001);
  S5K3H5XA_wordwrite_cmos_sensor(0x0384, 0x0001);
  S5K3H5XA_wordwrite_cmos_sensor(0x0386, 0x0003);
  S5K3H5XA_bytewrite_cmos_sensor(0x0900, 0x01);
  S5K3H5XA_bytewrite_cmos_sensor(0x0901, 0x12);
  S5K3H5XA_bytewrite_cmos_sensor(0x0902, 0x01);
  S5K3H5XA_bytewrite_cmos_sensor(0x3011, 0x02);
  S5K3H5XA_bytewrite_cmos_sensor(0x3293, 0x00);
  S5K3H5XA_bytewrite_cmos_sensor(0x0100, 0x01);
  Sleep(50);
}

	
void S5K3H5XAVideoSetting(void)
{
	S5K3H5XADB("[S5K3H5XA] [S5K3H5XAVideoSetting]\n");
  S5K3H5XA_wordwrite_cmos_sensor(0xFCFC, 0xD000);
  S5K3H5XA_wordwrite_cmos_sensor(0x6010, 0x0001);
  Sleep(10);
  S5K3H5XA_wordwrite_cmos_sensor(0x6028, 0x7000);
  S5K3H5XA_wordwrite_cmos_sensor(0x602a, 0x1870);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x2DE9);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xF84F);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x9FE5);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x6081);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xA0E1);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0090);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x98E5);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x00A0);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x9FE5);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x5071);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x9FE5);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x5401);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xD7E1);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xBE5E);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xD0E1);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xBC03);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xDAE5);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x9D10);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xDAE5);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x9FB0);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x40E0);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0100);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x00E0);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x9500);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x55E1);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0B00);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xA0E1);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x2005);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x002A);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x1B00);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x5BE1);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0000);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x008A);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0B00);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xA0E3);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0040);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xC7E1);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xBEBE);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x87E0);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0461);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x96E5);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xF800);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xA0E1);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0B10);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x00E0);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x9500);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x00EB);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x4800);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x84E2);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0140);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x54E3);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0400);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x86E5);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xF800);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xFF3A);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xF6FF);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x00EA);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0D00);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xDAE5);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x9E00);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x50E1);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0500);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x002A);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0A00);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xA0E3);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0040);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xC7E1);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xBE0E);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x87E0);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0461);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x96E5);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xF800);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xDAE5);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x9E10);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x00E0);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x9500);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x00EB);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x3900);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x84E2);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0140);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x54E3);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0400);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x86E5);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xF800);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xFF3A);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xF6FF);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x9FE5);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xC000);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x87E2);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xC830);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x8DE5);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0030);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xD0E5);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xB920);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xD0E1);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xB618);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x9FE5);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xB000);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xA0E1);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0930);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x90E5);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x28C0);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x87E2);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xE800);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xA0E1);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0FE0);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x2FE1);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x1CFF);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x50E3);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0000);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x001B);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x2A00);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x9FE5);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x9400);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x98E5);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0010);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xD0E1);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xBA05);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xD1E5);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x9C10);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x9FE5);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x8830);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x50E1);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0100);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xA0E3);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0000);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x002A);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x1300);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x00EA);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0500);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xD1E1);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xB210);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x82E2);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0D22);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xC2E1);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xB010);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x80E2);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0100);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x50E3);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x1400);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x00AA);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0400);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x80E0);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x8010);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x83E0);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x8110);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xD1E1);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xB020);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x52E3);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0000);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xFF1A);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xF4FF);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xBDE8);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xF84F);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x2FE1);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x1EFF);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xD1E1);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xB410);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x82E2);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0D22);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xC2E1);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xB010);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x80E2);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0100);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x50E3);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x1400);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xFFAA);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xF7FF);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x80E0);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x8010);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x83E0);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x8110);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xD1E1);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xB020);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x52E3);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0000);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xFF1A);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xF4FF);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xFFEA);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xF1FF);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0070);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x7014);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0070);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0000);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0070);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x800C);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0070);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x3013);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0070);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xF804);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0070);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x7015);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0070);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x002D);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x1FE5);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x04F0);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0000);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x20E1);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x1FE5);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x04F0);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0000);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x641C);
  S5K3H5XA_wordwrite_cmos_sensor(0x6028, 0xD000);
  S5K3H5XA_wordwrite_cmos_sensor(0x3902, 0x0002);
  S5K3H5XA_wordwrite_cmos_sensor(0x3158, 0x0215);
  S5K3H5XA_wordwrite_cmos_sensor(0x32B4, 0xF4B6);
  S5K3H5XA_wordwrite_cmos_sensor(0x32B6, 0xF466);
  S5K3H5XA_wordwrite_cmos_sensor(0x32B8, 0xF456);
  S5K3H5XA_wordwrite_cmos_sensor(0x32BA, 0xF45E);
  S5K3H5XA_bytewrite_cmos_sensor(0x32BC, 0x10);
  S5K3H5XA_bytewrite_cmos_sensor(0x32BD, 0x00);
  S5K3H5XA_bytewrite_cmos_sensor(0x32BE, 0x00);
  S5K3H5XA_wordwrite_cmos_sensor(0x3338, 0x0214);
  S5K3H5XA_wordwrite_cmos_sensor(0x6218, 0xF1D0);
  S5K3H5XA_wordwrite_cmos_sensor(0x6214, 0xF9F0);
  S5K3H5XA_wordwrite_cmos_sensor(0x6226, 0x0001);
  S5K3H5XA_wordwrite_cmos_sensor(0xF446, 0x0029);
  S5K3H5XA_wordwrite_cmos_sensor(0xF448, 0x001D);
  S5K3H5XA_wordwrite_cmos_sensor(0xF440, 0x0071);
  S5K3H5XA_wordwrite_cmos_sensor(0xF42E, 0x00C1);
  S5K3H5XA_wordwrite_cmos_sensor(0xF42A, 0x0802);
  S5K3H5XA_wordwrite_cmos_sensor(0xB0C8, 0x0044);
  S5K3H5XA_wordwrite_cmos_sensor(0x6226, 0x0000);
  S5K3H5XA_wordwrite_cmos_sensor(0x6218, 0xF9F0);
  S5K3H5XA_wordwrite_cmos_sensor(0x34A2, 0x00D6);
  S5K3H5XA_wordwrite_cmos_sensor(0x34B2, 0x01FA);
  S5K3H5XA_wordwrite_cmos_sensor(0x34CA, 0x00D6);
  S5K3H5XA_wordwrite_cmos_sensor(0x34DA, 0x01FA);
  S5K3H5XA_wordwrite_cmos_sensor(0x3522, 0x00D6);
  S5K3H5XA_wordwrite_cmos_sensor(0x3532, 0x01FA);
  S5K3H5XA_wordwrite_cmos_sensor(0x3254, 0x79D3);
  S5K3H5XA_wordwrite_cmos_sensor(0x3256, 0x79D3);
  S5K3H5XA_wordwrite_cmos_sensor(0x3258, 0x79D3);
  S5K3H5XA_wordwrite_cmos_sensor(0x325A, 0x79D3);
  S5K3H5XA_wordwrite_cmos_sensor(0x325C, 0x79D3);
  S5K3H5XA_wordwrite_cmos_sensor(0x325E, 0x79D3);
  S5K3H5XA_wordwrite_cmos_sensor(0x357A, 0x00BD);
  S5K3H5XA_wordwrite_cmos_sensor(0x32F6, 0x1110);
  S5K3H5XA_bytewrite_cmos_sensor(0x012C, 0x60);
  S5K3H5XA_bytewrite_cmos_sensor(0x012D, 0x4F);
  S5K3H5XA_bytewrite_cmos_sensor(0x012E, 0x2F);
  S5K3H5XA_bytewrite_cmos_sensor(0x012F, 0x40);
  S5K3H5XA_wordwrite_cmos_sensor(0x6028, 0x7000);
  S5K3H5XA_wordwrite_cmos_sensor(0x602A, 0x2D00);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x30F4);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xD370);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xD379);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x12F4);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0500);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0100);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x4638);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0007);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xF004);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x5038);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0000);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x1002);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xF838);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xFAFF);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0000);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x4C38);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x7805);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x9C04);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x78F4);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0700);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0700);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x9AF4);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x3100);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x3100);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x36F4);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0600);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0600);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0000);
  S5K3H5XA_wordwrite_cmos_sensor(0x6028, 0xD000);
  S5K3H5XA_wordwrite_cmos_sensor(0x6226, 0x0001);
  S5K3H5XA_wordwrite_cmos_sensor(0x6100, 0x0003);
  S5K3H5XA_wordwrite_cmos_sensor(0x6110, 0x1CA0);
  S5K3H5XA_wordwrite_cmos_sensor(0x6112, 0x1CA4);
  S5K3H5XA_wordwrite_cmos_sensor(0x6150, 0x172C);
  S5K3H5XA_wordwrite_cmos_sensor(0x6152, 0x1730);
  S5K3H5XA_wordwrite_cmos_sensor(0x6028, 0x7000);
  S5K3H5XA_wordwrite_cmos_sensor(0x602A, 0x172C);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x1FE5);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x04F0);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0070);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x7018);
  S5K3H5XA_wordwrite_cmos_sensor(0x6028, 0xD000);
  S5K3H5XA_wordwrite_cmos_sensor(0x6226, 0x0000);
  S5K3H5XA_bytewrite_cmos_sensor(0x0101, 0x00);
  S5K3H5XA_wordwrite_cmos_sensor(0x0112, 0x0A0A);
  S5K3H5XA_bytewrite_cmos_sensor(0x0114, 0x03);
  S5K3H5XA_bytewrite_cmos_sensor(0x0120, 0x00);
  S5K3H5XA_wordwrite_cmos_sensor(0x0200, 0x0BEF);
  S5K3H5XA_wordwrite_cmos_sensor(0x0202, 0x09D9);
  S5K3H5XA_wordwrite_cmos_sensor(0x0204, 0x0020);
  S5K3H5XA_wordwrite_cmos_sensor(0x0300, 0x0002);
  S5K3H5XA_wordwrite_cmos_sensor(0x0302, 0x0001);
  S5K3H5XA_wordwrite_cmos_sensor(0x0304, 0x0006);
  S5K3H5XA_wordwrite_cmos_sensor(0x0306, 0x008C);
  S5K3H5XA_wordwrite_cmos_sensor(0x0308, 0x0008);
  S5K3H5XA_wordwrite_cmos_sensor(0x030A, 0x0001);
  S5K3H5XA_wordwrite_cmos_sensor(0x030C, 0x0006);
  S5K3H5XA_wordwrite_cmos_sensor(0x030E, 0x00A5);
  S5K3H5XA_wordwrite_cmos_sensor(0x0340, 0x09E2);
  S5K3H5XA_wordwrite_cmos_sensor(0x0342, 0x0E68);
  S5K3H5XA_wordwrite_cmos_sensor(0x32CE, 0x0094);
  S5K3H5XA_wordwrite_cmos_sensor(0x32D0, 0x0024);
  S5K3H5XA_wordwrite_cmos_sensor(0x0344, 0x0008);
  S5K3H5XA_wordwrite_cmos_sensor(0x0346, 0x013A);
  S5K3H5XA_wordwrite_cmos_sensor(0x0348, 0x0CC7);
  S5K3H5XA_wordwrite_cmos_sensor(0x034A, 0x0865);
  S5K3H5XA_wordwrite_cmos_sensor(0x034C, 0x0CC0);
  S5K3H5XA_wordwrite_cmos_sensor(0x034E, 0x072C);
  S5K3H5XA_wordwrite_cmos_sensor(0x0380, 0x0001);
  S5K3H5XA_wordwrite_cmos_sensor(0x0382, 0x0001);
  S5K3H5XA_wordwrite_cmos_sensor(0x0384, 0x0001);
  S5K3H5XA_wordwrite_cmos_sensor(0x0386, 0x0001);
  S5K3H5XA_bytewrite_cmos_sensor(0x0900, 0x01);
  S5K3H5XA_bytewrite_cmos_sensor(0x0901, 0x11);
  S5K3H5XA_bytewrite_cmos_sensor(0x0902, 0x01);
  S5K3H5XA_bytewrite_cmos_sensor(0x3011, 0x01);
  S5K3H5XA_bytewrite_cmos_sensor(0x3293, 0x00);
  S5K3H5XA_bytewrite_cmos_sensor(0x0100, 0x01); 
  Sleep(50);
}

void S5K3H5XACaptureSetting(void)
{
  S5K3H5XADB("[S5K3H5XA] [S5K3H5XACaptureSetting]\n");
  S5K3H5XA_wordwrite_cmos_sensor(0xFCFC, 0xD000);
  S5K3H5XA_wordwrite_cmos_sensor(0x6010, 0x0001);
  Sleep(10);
  S5K3H5XA_wordwrite_cmos_sensor(0x6028, 0x7000);
  S5K3H5XA_wordwrite_cmos_sensor(0x602a, 0x1870);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x2DE9);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xF84F);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x9FE5);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x6081);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xA0E1);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0090);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x98E5);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x00A0);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x9FE5);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x5071);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x9FE5);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x5401);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xD7E1);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xBE5E);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xD0E1);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xBC03);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xDAE5);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x9D10);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xDAE5);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x9FB0);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x40E0);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0100);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x00E0);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x9500);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x55E1);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0B00);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xA0E1);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x2005);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x002A);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x1B00);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x5BE1);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0000);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x008A);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0B00);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xA0E3);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0040);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xC7E1);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xBEBE);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x87E0);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0461);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x96E5);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xF800);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xA0E1);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0B10);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x00E0);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x9500);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x00EB);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x4800);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x84E2);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0140);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x54E3);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0400);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x86E5);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xF800);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xFF3A);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xF6FF);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x00EA);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0D00);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xDAE5);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x9E00);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x50E1);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0500);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x002A);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0A00);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xA0E3);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0040);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xC7E1);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xBE0E);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x87E0);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0461);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x96E5);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xF800);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xDAE5);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x9E10);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x00E0);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x9500);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x00EB);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x3900);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x84E2);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0140);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x54E3);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0400);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x86E5);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xF800);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xFF3A);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xF6FF);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x9FE5);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xC000);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x87E2);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xC830);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x8DE5);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0030);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xD0E5);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xB920);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xD0E1);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xB618);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x9FE5);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xB000);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xA0E1);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0930);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x90E5);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x28C0);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x87E2);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xE800);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xA0E1);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0FE0);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x2FE1);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x1CFF);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x50E3);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0000);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x001B);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x2A00);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x9FE5);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x9400);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x98E5);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0010);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xD0E1);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xBA05);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xD1E5);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x9C10);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x9FE5);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x8830);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x50E1);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0100);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xA0E3);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0000);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x002A);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x1300);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x00EA);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0500);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xD1E1);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xB210);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x82E2);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0D22);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xC2E1);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xB010);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x80E2);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0100);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x50E3);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x1400);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x00AA);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0400);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x80E0);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x8010);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x83E0);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x8110);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xD1E1);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xB020);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x52E3);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0000);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xFF1A);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xF4FF);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xBDE8);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xF84F);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x2FE1);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x1EFF);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xD1E1);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xB410);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x82E2);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0D22);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xC2E1);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xB010);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x80E2);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0100);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x50E3);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x1400);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xFFAA);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xF7FF);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x80E0);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x8010);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x83E0);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x8110);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xD1E1);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xB020);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x52E3);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0000);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xFF1A);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xF4FF);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xFFEA);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xF1FF);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0070);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x7014);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0070);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0000);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0070);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x800C);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0070);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x3013);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0070);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xF804);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0070);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x7015);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0070);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x002D);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x1FE5);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x04F0);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0000);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x20E1);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x1FE5);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x04F0);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0000);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x641C);
  S5K3H5XA_wordwrite_cmos_sensor(0x6028, 0xD000);
  S5K3H5XA_wordwrite_cmos_sensor(0x3902, 0x0002);
  S5K3H5XA_wordwrite_cmos_sensor(0x3158, 0x0215);
  S5K3H5XA_wordwrite_cmos_sensor(0x32B4, 0xF4B6);
  S5K3H5XA_wordwrite_cmos_sensor(0x32B6, 0xF466);
  S5K3H5XA_wordwrite_cmos_sensor(0x32B8, 0xF456);
  S5K3H5XA_wordwrite_cmos_sensor(0x32BA, 0xF45E);
  S5K3H5XA_bytewrite_cmos_sensor(0x32BC, 0x10);
  S5K3H5XA_bytewrite_cmos_sensor(0x32BD, 0x00);
  S5K3H5XA_bytewrite_cmos_sensor(0x32BE, 0x00);
  S5K3H5XA_wordwrite_cmos_sensor(0x3338, 0x0214);
  S5K3H5XA_wordwrite_cmos_sensor(0x6218, 0xF1D0);
  S5K3H5XA_wordwrite_cmos_sensor(0x6214, 0xF9F0);
  S5K3H5XA_wordwrite_cmos_sensor(0x6226, 0x0001);
  S5K3H5XA_wordwrite_cmos_sensor(0xF446, 0x0029);
  S5K3H5XA_wordwrite_cmos_sensor(0xF448, 0x001D);
  S5K3H5XA_wordwrite_cmos_sensor(0xF440, 0x0071);
  S5K3H5XA_wordwrite_cmos_sensor(0xF42E, 0x00C1);
  S5K3H5XA_wordwrite_cmos_sensor(0xF42A, 0x0802);
  S5K3H5XA_wordwrite_cmos_sensor(0xB0C8, 0x0044);
  S5K3H5XA_wordwrite_cmos_sensor(0x6226, 0x0000);
  S5K3H5XA_wordwrite_cmos_sensor(0x6218, 0xF9F0);
  S5K3H5XA_wordwrite_cmos_sensor(0x34A2, 0x00D6);
  S5K3H5XA_wordwrite_cmos_sensor(0x34B2, 0x01FA);
  S5K3H5XA_wordwrite_cmos_sensor(0x34CA, 0x00D6);
  S5K3H5XA_wordwrite_cmos_sensor(0x34DA, 0x01FA);
  S5K3H5XA_wordwrite_cmos_sensor(0x3522, 0x00D6);
  S5K3H5XA_wordwrite_cmos_sensor(0x3532, 0x01FA);
  S5K3H5XA_wordwrite_cmos_sensor(0x3254, 0x79D3);
  S5K3H5XA_wordwrite_cmos_sensor(0x3256, 0x79D3);
  S5K3H5XA_wordwrite_cmos_sensor(0x3258, 0x79D3);
  S5K3H5XA_wordwrite_cmos_sensor(0x325A, 0x79D3);
  S5K3H5XA_wordwrite_cmos_sensor(0x325C, 0x79D3);
  S5K3H5XA_wordwrite_cmos_sensor(0x325E, 0x79D3);
  S5K3H5XA_wordwrite_cmos_sensor(0x357A, 0x00BD);
  S5K3H5XA_wordwrite_cmos_sensor(0x32F6, 0x1110);
  S5K3H5XA_bytewrite_cmos_sensor(0x012C, 0x60);
  S5K3H5XA_bytewrite_cmos_sensor(0x012D, 0x4F);
  S5K3H5XA_bytewrite_cmos_sensor(0x012E, 0x2F);
  S5K3H5XA_bytewrite_cmos_sensor(0x012F, 0x40);
  S5K3H5XA_wordwrite_cmos_sensor(0x6028, 0x7000);
  S5K3H5XA_wordwrite_cmos_sensor(0x602A, 0x2D00);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x30F4);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xD370);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xD379);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x12F4);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0500);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0100);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x4638);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0007);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xF004);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x5038);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0000);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x1002);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xF838);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0xFAFF);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0000);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x4C38);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x7805);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x9C04);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x78F4);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0700);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0700);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x9AF4);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x3100);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x3100);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x36F4);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0600);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0600);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0000);
  S5K3H5XA_wordwrite_cmos_sensor(0x6028, 0xD000);
  S5K3H5XA_wordwrite_cmos_sensor(0x6226, 0x0001);
  S5K3H5XA_wordwrite_cmos_sensor(0x6100, 0x0003);
  S5K3H5XA_wordwrite_cmos_sensor(0x6110, 0x1CA0);
  S5K3H5XA_wordwrite_cmos_sensor(0x6112, 0x1CA4);
  S5K3H5XA_wordwrite_cmos_sensor(0x6150, 0x172C);
  S5K3H5XA_wordwrite_cmos_sensor(0x6152, 0x1730);
  S5K3H5XA_wordwrite_cmos_sensor(0x6028, 0x7000);
  S5K3H5XA_wordwrite_cmos_sensor(0x602A, 0x172C);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x1FE5);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x04F0);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x0070);
  S5K3H5XA_wordwrite_cmos_sensor(0x6F12, 0x7018);
  S5K3H5XA_wordwrite_cmos_sensor(0x6028, 0xD000);
  S5K3H5XA_wordwrite_cmos_sensor(0x6226, 0x0000);
  S5K3H5XA_bytewrite_cmos_sensor(0x0101, 0x00);
  S5K3H5XA_wordwrite_cmos_sensor(0x0112, 0x0A0A);
  S5K3H5XA_bytewrite_cmos_sensor(0x0114, 0x03);
  S5K3H5XA_bytewrite_cmos_sensor(0x0120, 0x00);
  S5K3H5XA_wordwrite_cmos_sensor(0x0200, 0x0BEF);
  S5K3H5XA_wordwrite_cmos_sensor(0x0202, 0x09D9);
  S5K3H5XA_wordwrite_cmos_sensor(0x0204, 0x0020);
  S5K3H5XA_wordwrite_cmos_sensor(0x0300, 0x0002);
  S5K3H5XA_wordwrite_cmos_sensor(0x0302, 0x0001);
  S5K3H5XA_wordwrite_cmos_sensor(0x0304, 0x0006);
  S5K3H5XA_wordwrite_cmos_sensor(0x0306, 0x008C);
  S5K3H5XA_wordwrite_cmos_sensor(0x0308, 0x0008);
  S5K3H5XA_wordwrite_cmos_sensor(0x030A, 0x0001);
  S5K3H5XA_wordwrite_cmos_sensor(0x030C, 0x0006);
  S5K3H5XA_wordwrite_cmos_sensor(0x030E, 0x00A5);
  S5K3H5XA_wordwrite_cmos_sensor(0x0340, 0x09E2);
  S5K3H5XA_wordwrite_cmos_sensor(0x0342, 0x0E68);
  S5K3H5XA_wordwrite_cmos_sensor(0x32CE, 0x0094);
  S5K3H5XA_wordwrite_cmos_sensor(0x32D0, 0x0024);
  S5K3H5XA_wordwrite_cmos_sensor(0x0344, 0x0008);
  S5K3H5XA_wordwrite_cmos_sensor(0x0346, 0x0008);
  S5K3H5XA_wordwrite_cmos_sensor(0x0348, 0x0CC7);
  S5K3H5XA_wordwrite_cmos_sensor(0x034A, 0x0997);
  S5K3H5XA_wordwrite_cmos_sensor(0x034C, 0x0CC0);
  S5K3H5XA_wordwrite_cmos_sensor(0x034E, 0x0990);
  S5K3H5XA_wordwrite_cmos_sensor(0x0380, 0x0001);
  S5K3H5XA_wordwrite_cmos_sensor(0x0382, 0x0001);
  S5K3H5XA_wordwrite_cmos_sensor(0x0384, 0x0001);
  S5K3H5XA_wordwrite_cmos_sensor(0x0386, 0x0001);
  S5K3H5XA_bytewrite_cmos_sensor(0x0900, 0x01);
  S5K3H5XA_bytewrite_cmos_sensor(0x0901, 0x11);
  S5K3H5XA_bytewrite_cmos_sensor(0x0902, 0x01);
  S5K3H5XA_bytewrite_cmos_sensor(0x3011, 0x01);
  S5K3H5XA_bytewrite_cmos_sensor(0x3293, 0x00);
  S5K3H5XA_bytewrite_cmos_sensor(0x0100, 0x01);
  Sleep(50);
}

/*************************************************************************
* FUNCTION
*   S5K3H5XAOpen
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

UINT32 S5K3H5XAOpen(void)
{

	volatile signed int i;
	kal_uint16 sensor_id = 0;

	S5K3H5XADB("[S5K3H5XA] [S5K3H5XAOpen]\n");

	//  Read sensor ID to adjust I2C is OK?
	for(i=0;i<3;i++)
	{
		sensor_id = (S5K3H5XA_read_cmos_sensor(0x0000)<<8)|S5K3H5XA_read_cmos_sensor(0x0001);
		S5K3H5XADB("[S5K3H5XA] [S5K3H5XAOpen] sensor_id=%x\n",sensor_id);
		if(sensor_id != S5K3H5XA_SENSOR_ID)
		{
			return ERROR_SENSOR_CONNECT_FAIL;
		}else
			break;
	}
	spin_lock(&s5k3h5xamipiraw_drv_lock);
	s5k3h5xa.sensorMode = SENSOR_MODE_INIT;
	s5k3h5xa.S5K3H5XAAutoFlickerMode = KAL_FALSE;
	s5k3h5xa.S5K3H5XAVideoMode = KAL_FALSE;
	s5k3h5xa.DummyLines= 0;
	s5k3h5xa.DummyPixels= 0;

	s5k3h5xa.pvPclk =  (28000);  
	s5k3h5xa.videoPclk = (28000); 
	
	spin_unlock(&s5k3h5xamipiraw_drv_lock);

    	switch(S5K3H5XACurrentScenarioId)
		{
			case MSDK_SCENARIO_ID_CAMERA_ZSD:
				spin_lock(&s5k3h5xamipiraw_drv_lock);
				s5k3h5xa.capPclk = (28000);
				spin_unlock(&s5k3h5xamipiraw_drv_lock);
				break;
        	default:
				spin_lock(&s5k3h5xamipiraw_drv_lock);
				s5k3h5xa.capPclk = (28000);
				spin_unlock(&s5k3h5xamipiraw_drv_lock);
				break;
          }
	spin_lock(&s5k3h5xamipiraw_drv_lock);
	s5k3h5xa.shutter = 0x4EA;
	s5k3h5xa.pvShutter = 0x4EA;
	s5k3h5xa.maxExposureLines =2530 -16;

	
	s5k3h5xa.ispBaseGain = BASEGAIN;//0x40
	s5k3h5xa.sensorGlobalGain = 0x1f;//sensor gain read from 0x350a 0x350b; 0x1f as 3.875x
	s5k3h5xa.pvGain = 0x1f;
	spin_unlock(&s5k3h5xamipiraw_drv_lock);
    return ERROR_NONE;
}

/*************************************************************************
* FUNCTION
*   S5K3H5XAGetSensorID
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
UINT32 S5K3H5XAGetSensorID(UINT32 *sensorID)
{
	int  retry = 3;
	UINT16 temp;
	S5K3H5XADB("[S5K3H5XA] [S5K3H5XAGetSensorID]\n");
	//S5K3H5XA_write_cmos_sensor(0x0103,0x01);// Reset sensor
    //mDELAY(10);

    // check if sensor ID correct
    do {

	temp = S5K3H5XA_read_cmos_sensor(0x0000);
	printk("reg_0x0000 = 0x%x",temp);
	
	temp = S5K3H5XA_read_cmos_sensor(0x0001);
	printk("reg_0x0001 = 0x%x",temp);
        *sensorID = (S5K3H5XA_read_cmos_sensor(0x0000)<<8) | (S5K3H5XA_read_cmos_sensor(0x0001));
	 printk("'sensorID = 0x%x\n",*sensorID);
        if (*sensorID == S5K3H5XA_SENSOR_ID)
        	{
        		S5K3H5XADB("[S5K3H5XA] [S5K3H5XAGetSensorID] Sensor ID = 0x%04x\n", *sensorID);
            	break;
        	}
        S5K3H5XADB("[S5K3H5XA] [S5K3H5XAGetSensorID] Read Sensor ID Fasil = 0x%04x\n", *sensorID);
        retry--;
    } while (retry > 0);

    if (*sensorID != S5K3H5XA_SENSOR_ID) {
        *sensorID = 0xFFFFFFFF;
        return ERROR_SENSOR_CONNECT_FAIL;
    }
    return ERROR_NONE;
}


/*************************************************************************
* FUNCTION
*   S5K3H5XA_SetShutter
*
* DESCRIPTION
*   This function set e-shutter of S5K3H5XA to change exposure time.
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
void S5K3H5XA_SetShutter(kal_uint32 iShutter)
{
	S5K3H5XADB("[S5K3H5XA] [S5K3H5XA_SetShutter] shutter = %d\n", iShutter);
	spin_lock(&s5k3h5xamipiraw_drv_lock);
	s5k3h5xa.shutter= iShutter;
	spin_unlock(&s5k3h5xamipiraw_drv_lock);
	S5K3H5XA_write_shutter(iShutter);
}   /*  S5K3H5XA_SetShutter   */



/*************************************************************************
* FUNCTION
*   S5K3H5XA_read_shutter
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
UINT32 S5K3H5XA_read_shutter(void)
{
	UINT32 shutter =0;

	shutter = (S5K3H5XA_read_cmos_sensor(0x0202) << 8) | S5K3H5XA_read_cmos_sensor(0x0203);
	S5K3H5XADB("[S5K3H5XA] [S5K3H5XA_read_shutter] shutter = %d\n", shutter);
	return shutter;
}

/*************************************************************************
* FUNCTION
*   S5K3H5XA_night_mode
*
* DESCRIPTION
*   This function night mode of S5K3H5XA.
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
void S5K3H5XA_NightMode(kal_bool bEnable)
{
}/*	S5K3H5XA_NightMode */



/*************************************************************************
* FUNCTION
*   S5K3H5XAClose
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
UINT32 S5K3H5XAClose(void)
{
    return ERROR_NONE;
}	/* S5K3H5XAClose() */ 
void S5K3H5XASetFlipMirror(kal_int32 imgMirror)
{
	S5K3H5XADB("[S5K3H5XA] [S5K3H5XASetFlipMirror] imgMirror = %d\n", imgMirror);
	switch (imgMirror)
    {
        case IMAGE_NORMAL: //B
            S5K3H5XA_bytewrite_cmos_sensor(0x0101, 0x03);	//Set normal
            break;
        case IMAGE_V_MIRROR: //Gr X
            S5K3H5XA_bytewrite_cmos_sensor(0x0101, 0x01);	//Set flip
            break;
        case IMAGE_H_MIRROR: //Gb
            S5K3H5XA_bytewrite_cmos_sensor(0x0101, 0x02);	//Set mirror
            break;
        case IMAGE_HV_MIRROR: //R
            S5K3H5XA_bytewrite_cmos_sensor(0x0101, 0x00);	//Set mirror and flip
            break;
    }
}
/*************************************************************************
* FUNCTION
*   S5K3H5XAPreview
 
* DESCRIPTION
*   This function start the sensor preview.
 
* PARAMETERS
*   *image_window : address pointer of pixel numbers in one period of HSYNC
*  *sensor_config_data : address pointer of line numbers in one period of VSYNC
 
* RETURNS
*   None
 
* GLOBALS AFFECTED
 
*************************************************************************/
UINT32 S5K3H5XAPreview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                                                MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	S5K3H5XADB("[S5K3H5XA] [S5K3H5XAPreview]\n"); 
	// preview size
	if(s5k3h5xa.sensorMode == SENSOR_MODE_PREVIEW)
	{
	}
	else
	{
		S5K3H5XAPreviewSetting();
	}
	spin_lock(&s5k3h5xamipiraw_drv_lock);
	s5k3h5xa.sensorMode = SENSOR_MODE_PREVIEW; 
	s5k3h5xa.DummyPixels = 0;
	s5k3h5xa.DummyLines = 0 ;
	cont_preview_line_length=s5k3h5xa_preview_line_length;
	cont_preview_frame_length=s5k3h5xa_preview_frame_length+s5k3h5xa.DummyLines;
	S5K3H5XA_FeatureControl_PERIOD_PixelNum = s5k3h5xa_preview_line_length;
	S5K3H5XA_FeatureControl_PERIOD_LineNum = cont_preview_frame_length;
	spin_unlock(&s5k3h5xamipiraw_drv_lock); 
	spin_lock(&s5k3h5xamipiraw_drv_lock);
	s5k3h5xa.imgMirror = sensor_config_data->SensorImageMirror;
	spin_unlock(&s5k3h5xamipiraw_drv_lock);
	S5K3H5XASetFlipMirror(IMAGE_HV_MIRROR);
    return ERROR_NONE;
}	/* S5K3H5XAPreview() */

UINT32 S5K3H5XAVideo(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                                                MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	S5K3H5XADB("[S5K3H5XA] [S5K3H5XAVideo]\n"); 
	if(s5k3h5xa.sensorMode == SENSOR_MODE_VIDEO)
	{
		// do nothing
	}
	else
	{
		S5K3H5XAVideoSetting(); 
	}
	spin_lock(&s5k3h5xamipiraw_drv_lock);
	s5k3h5xa.sensorMode = SENSOR_MODE_VIDEO;
	S5H3H5XA_FeatureControl_PERIOD_PixelNum = s5k3h5xa_video_line_length;
	S5K3H5XA_FeatureControl_PERIOD_LineNum = s5k3h5xa_video_frame_length;
	cont_video_frame_length = s5k3h5xa_video_frame_length;
	s5k3h5xa.imgMirror = sensor_config_data->SensorImageMirror;
	spin_unlock(&s5k3h5xamipiraw_drv_lock);
	S5K3H5XASetFlipMirror(IMAGE_HV_MIRROR);
    return ERROR_NONE;
}   

UINT32 S5K3H5XACapture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                                                MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
 	kal_uint32 shutter = s5k3h5xa.shutter;
	kal_uint32 temp_data; 
	S5K3H5XADB("[S5K3H5XA] [S5K3H5XACapture]\n"); 
	if( SENSOR_MODE_CAPTURE== s5k3h5xa.sensorMode)
	{
		S5K3H5XADB("[S5K3H5XA] [S5K3H5XACapture] BusrtShot!!\n");
	}else{ 
	//Record Preview shutter & gain
	shutter=S5K3H5XA_read_shutter();
	temp_data =  read_S5K3H5XA_gain();
	spin_lock(&s5k3h5xamipiraw_drv_lock);
	s5k3h5xa.pvShutter =shutter;
	s5k3h5xa.sensorGlobalGain = temp_data;
	s5k3h5xa.pvGain =s5k3h5xa.sensorGlobalGain;
	s5k3h5xa.sensorMode = SENSOR_MODE_CAPTURE;	
	spin_unlock(&s5k3h5xamipiraw_drv_lock);

	S5K3H5XADB("[S5K3H5XA] [S5K3H5XACapture] s5k3h5xa.shutter=%d, read_pv_shutter=%d, read_pv_gain = 0x%x\n",s5k3h5xa.shutter, shutter,s5k3h5xa.sensorGlobalGain);

	// Full size setting
	S5K3H5XACaptureSetting();

    //rewrite pixel number to Register ,for mt6589 line start/end;
	S5K3H5XA_SetDummy(s5k3h5xa.DummyPixels,s5k3h5xa.DummyLines);

	spin_lock(&s5k3h5xamipiraw_drv_lock);

	s5k3h5xa.imgMirror = sensor_config_data->SensorImageMirror;
	s5k3h5xa.DummyPixels = 0;//define dummy pixels and lines
	s5k3h5xa.DummyLines = 0 ;
	cont_capture_line_length = s5k3h5xa_capture_line_length;
	cont_capture_frame_length = s5k3h5xa_capture_frame_length + s5k3h5xa.DummyLines;
	S5K3H5XA_FeatureControl_PERIOD_PixelNum = s5k3h5xa_capture_line_length;
	S5K3H5XA_FeatureControl_PERIOD_LineNum = cont_capture_frame_length;
	spin_unlock(&s5k3h5xamipiraw_drv_lock);

	S5K3H5XASetFlipMirror(IMAGE_HV_MIRROR);


    if(S5K3H5XACurrentScenarioId==MSDK_SCENARIO_ID_CAMERA_ZSD)
    {
		S5K3H5XADB("[S5K3H5XA] [S5K3H5XACapture] S5K3H5XACapture exit ZSD!\n");
		return ERROR_NONE;
    }
	}

    return ERROR_NONE;
}	/* S5K3H5XACapture() */

UINT32 S5K3H5XAGetResolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *pSensorResolution)
{

    S5K3H5XADB("[S5K3H5XA] [S5K3H5XAGetResolution]\n");


	pSensorResolution->SensorPreviewWidth	= 1600;
    pSensorResolution->SensorPreviewHeight	= 1200;
	pSensorResolution->SensorFullWidth		= 3200;
    pSensorResolution->SensorFullHeight		= 2400;
    pSensorResolution->SensorVideoWidth		= S5K3H5XA_IMAGE_SENSOR_VIDEO_WIDTH;
    pSensorResolution->SensorVideoHeight    = S5K3H5XA_IMAGE_SENSOR_VIDEO_HEIGHT;
    return ERROR_NONE;
}   /* S5K3H5XAGetResolution() */


UINT32 S5K3H5XAGetInfo(MSDK_SCENARIO_ID_ENUM ScenarioId,
                                                MSDK_SENSOR_INFO_STRUCT *pSensorInfo,
                                                MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
	 S5K3H5XADB("[S5K3H5XA] [S5K3H5XAGetInfo]\n");
	pSensorInfo->SensorPreviewResolutionX= 1600;
	pSensorInfo->SensorPreviewResolutionY= 1200;
	pSensorInfo->SensorFullResolutionX= 3200;
    pSensorInfo->SensorFullResolutionY= 2400;
	
	spin_lock(&s5k3h5xamipiraw_drv_lock);
	s5k3h5xa.imgMirror = pSensorConfigData->SensorImageMirror ;
	spin_unlock(&s5k3h5xamipiraw_drv_lock);



   		pSensorInfo->SensorOutputDataFormat= SENSOR_OUTPUT_FORMAT_RAW_Gr; //Gb r

   	
    pSensorInfo->SensorClockPolarity =SENSOR_CLOCK_POLARITY_LOW;
    pSensorInfo->SensorClockFallingPolarity=SENSOR_CLOCK_POLARITY_LOW;
    pSensorInfo->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
    pSensorInfo->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;

    pSensorInfo->SensroInterfaceType=SENSOR_INTERFACE_TYPE_MIPI;

    pSensorInfo->CaptureDelayFrame = 1;
    pSensorInfo->PreviewDelayFrame = 1;
    pSensorInfo->VideoDelayFrame = 2;

    pSensorInfo->SensorDrivingCurrent = ISP_DRIVING_8MA;
    pSensorInfo->AEShutDelayFrame = 0;//0;		    /* The frame of setting shutter default 0 for TG int */
    pSensorInfo->AESensorGainDelayFrame = 0 ;//0;     /* The frame of setting sensor gain */
    pSensorInfo->AEISPGainDelayFrame = 2;

    switch (ScenarioId)
    {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
            pSensorInfo->SensorClockFreq=24;
            pSensorInfo->SensorClockRisingCount= 0;
		
			pSensorInfo->SensorGrabStartX = S5K3H5XA_PV_X_START;
            pSensorInfo->SensorGrabStartY = S5K3H5XA_PV_Y_START;
	
            pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_4_LANE;
            pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
	     	pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 6;//14
	    	pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
            pSensorInfo->SensorPacketECCOrder = 1;
            break;
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
            pSensorInfo->SensorClockFreq=24;
            pSensorInfo->SensorClockRisingCount= 0;

            pSensorInfo->SensorGrabStartX = S5K3H5XA_VIDEO_X_START;
            pSensorInfo->SensorGrabStartY = S5K3H5XA_VIDEO_Y_START;
			
            pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_4_LANE;
            pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
	     	pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 6;//14
	    	pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
            pSensorInfo->SensorPacketECCOrder = 1;
            break;
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
            pSensorInfo->SensorClockFreq=24;
            pSensorInfo->SensorClockRisingCount= 0;
			
			pSensorInfo->SensorGrabStartX = S5K3H5XA_FULL_X_START;	//2*S5K3H5XA_IMAGE_SENSOR_PV_STARTX;
            pSensorInfo->SensorGrabStartY = S5K3H5XA_FULL_Y_START;	//2*S5K3H5XA_IMAGE_SENSOR_PV_STARTY;
	
            pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_4_LANE;
            pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
            pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 6;//14
            pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
            pSensorInfo->SensorPacketECCOrder = 1;
            break;
        default:
			pSensorInfo->SensorClockFreq=24;
            pSensorInfo->SensorClockRisingCount= 0;

            pSensorInfo->SensorGrabStartX = S5K3H5XA_PV_X_START;
            pSensorInfo->SensorGrabStartY = S5K3H5XA_PV_Y_START;

            pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_4_LANE;
            pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
	     	pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 6;//14
	    	pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
            pSensorInfo->SensorPacketECCOrder = 1;
            break;
    }

    memcpy(pSensorConfigData, &S5K3H5XASensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));

    return ERROR_NONE;
}   /* S5K3H5XAGetInfo() */


UINT32 S5K3H5XAControl(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *pImageWindow,
                                                MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
		spin_lock(&s5k3h5xamipiraw_drv_lock);
		S5K3H5XACurrentScenarioId = ScenarioId;
		spin_unlock(&s5k3h5xamipiraw_drv_lock);
		S5K3H5XADB("[S5K3H5XA] [S5K3H5XAControl]\n");
		S5K3H5XADB("[S5K3H5XA] [S5K3H5XAControl] ScenarioId=%d\n",S5K3H5XACurrentScenarioId);
    switch (ScenarioId)
    {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
            S5K3H5XAPreview(pImageWindow, pSensorConfigData);
            break;
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			S5K3H5XAVideo(pImageWindow, pSensorConfigData);
			break;   
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
            S5K3H5XACapture(pImageWindow, pSensorConfigData);
            break;

        default:
            return ERROR_INVALID_SCENARIO_ID;

    }
    return ERROR_NONE;
} /* S5K3H5XAControl() */


UINT32 S5K3H5XASetVideoMode(UINT16 u2FrameRate)
{

    kal_uint32 MIN_Frame_length =0,frameRate=0,extralines=0;
    S5K3H5XADB("[S5K3H5XA] [S5K3H5XASetVideoMode] frame rate = %d\n", u2FrameRate);
	
	if(u2FrameRate==0)
	{
		S5K3H5XADB("[S5K3H5XA] [S5K3H5XASetVideoMode] Disable Video Mode or dynimac fps\n");
		return KAL_TRUE;
	}
	if(u2FrameRate >30 || u2FrameRate <5)
	    S5K3H5XADB("[S5K3H5XA] [S5K3H5XASetVideoMode] error frame rate seting\n");

    if(s5k3h5xa.sensorMode == SENSOR_MODE_VIDEO)//video ScenarioId recording
    {
    	S5K3H5XADB("[S5K3H5XA] [S5K3H5XASetVideoMode] SENSOR_MODE_VIDEO\n");
    	if(s5k3h5xa.S5K3H5XAAutoFlickerMode == KAL_TRUE)
    	{
    		if (u2FrameRate==30)
				frameRate= 306;
			else if(u2FrameRate==15)
				frameRate= 148;//148;
			else
				frameRate=u2FrameRate*10;

			MIN_Frame_length = (s5k3h5xa.videoPclk*10000)/(S5K3H5XA_VIDEO_PERIOD_PIXEL_NUMS + s5k3h5xa.DummyPixels)/frameRate*10;
    	}
		else
			MIN_Frame_length = (s5k3h5xa.videoPclk*10000) /(S5K3H5XA_VIDEO_PERIOD_PIXEL_NUMS + s5k3h5xa.DummyPixels)/u2FrameRate;

		if((MIN_Frame_length <=S5K3H5XA_VIDEO_PERIOD_LINE_NUMS))
		{
			MIN_Frame_length = S5K3H5XA_VIDEO_PERIOD_LINE_NUMS;
		}
		extralines = MIN_Frame_length - S5K3H5XA_VIDEO_PERIOD_LINE_NUMS;

		spin_lock(&s5k3h5xamipiraw_drv_lock);
		s5k3h5xa.DummyPixels = 0;//define dummy pixels and lines
		s5k3h5xa.DummyLines = extralines ;
		spin_unlock(&s5k3h5xamipiraw_drv_lock);

		S5K3H5XA_SetDummy(s5k3h5xa.DummyPixels,extralines);
    }
	else if(s5k3h5xa.sensorMode == SENSOR_MODE_CAPTURE)
	{
		S5K3H5XADB("[S5K3H5XA] [S5K3H5XASetVideoMode] SENSOR_MODE_CAPTURE\n");
		if(s5k3h5xa.S5K3H5XAAutoFlickerMode == KAL_TRUE)
    	{
    		if (u2FrameRate==15)
			    frameRate= 148;
			else
				frameRate=u2FrameRate*10;
			
			MIN_Frame_length = (s5k3h5xa.capPclk*10000) /(S5K3H5XA_FULL_PERIOD_PIXEL_NUMS + s5k3h5xa.DummyPixels)/frameRate*10;
    	}
		else
			MIN_Frame_length = (s5k3h5xa.capPclk*10000) /(S5K3H5XA_FULL_PERIOD_PIXEL_NUMS + s5k3h5xa.DummyPixels)/u2FrameRate;

		if((MIN_Frame_length <=S5K3H5XA_FULL_PERIOD_LINE_NUMS))
		{
			MIN_Frame_length = S5K3H5XA_FULL_PERIOD_LINE_NUMS;
			S5K3H5XADB("[S5K3H5XA] [S5K3H5XASetVideoMode] current fps = %d\n", (s5k3h5xa.capPclk*10000) /(S5K3H5XA_FULL_PERIOD_PIXEL_NUMS)/S5K3H5XA_FULL_PERIOD_LINE_NUMS);

		}
		S5K3H5XADB("[S5K3H5XA] [S5K3H5XASetVideoMode] current fps (10 base)= %d\n", (s5k3h5xa.pvPclk*10000)*10/(S5K3H5XA_FULL_PERIOD_PIXEL_NUMS + s5k3h5xa.DummyPixels)/MIN_Frame_length);

		extralines = MIN_Frame_length - S5K3H5XA_FULL_PERIOD_LINE_NUMS;

		spin_lock(&s5k3h5xamipiraw_drv_lock);
		s5k3h5xa.DummyPixels = 0;//define dummy pixels and lines
		s5k3h5xa.DummyLines = extralines ;
		spin_unlock(&s5k3h5xamipiraw_drv_lock);

		S5K3H5XA_SetDummy(s5k3h5xa.DummyPixels,extralines);
	}
	S5K3H5XADB("[S5K3H5XA] [S5K3H5XASetVideoMode] MIN_Frame_length=%d,s5k3h5xa.DummyLines=%d\n",MIN_Frame_length,s5k3h5xa.DummyLines);

    return KAL_TRUE;
}

UINT32 S5K3H5XASetAutoFlickerMode(kal_bool bEnable, UINT16 u2FrameRate)
{
	if(bEnable) {   // enable auto flicker
		S5K3H5XADB("[S5K3H5XA] [S5K3H5XASetAutoFlickerMode] enable\n");
		spin_lock(&s5k3h5xamipiraw_drv_lock);
		s5k3h5xa.S5K3H5XAAutoFlickerMode = KAL_TRUE;
		spin_unlock(&s5k3h5xamipiraw_drv_lock);
    } else {
    	S5K3H5XADB("[S5K3H5XA] [S5K3H5XASetAutoFlickerMode] disable\n");
    	spin_lock(&s5k3h5xamipiraw_drv_lock);
        s5k3h5xa.S5K3H5XAAutoFlickerMode = KAL_FALSE;
		spin_unlock(&s5k3h5xamipiraw_drv_lock);
    }

    return ERROR_NONE;
}



UINT32 S5K3H5XASetTestPatternMode(kal_bool bEnable)
{ 
		S5K3H5XADB("[S5K3H5XASetTestPatternMode] Test pattern enable:%d\n", bEnable);	 

		if(bEnable)     
		{ 
			S5K3H5XA_bytewrite_cmos_sensor(0x0601, 0x02);
		}    
		else    
		{     
			S5K3H5XA_bytewrite_cmos_sensor(0x0601, 0x00);  
		} 

    return TRUE;
}



UINT32 S5K3H5XAMIPISetMaxFramerateByScenario(MSDK_SCENARIO_ID_ENUM scenarioId, MUINT32 frameRate) 
{
	kal_uint32 pclk;
	kal_int16 dummyLine;
	kal_uint16 lineLength,frameHeight;
		
	switch (scenarioId) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			S5K3H5XADB("[S5K3H5XA] [S5K3H5XAMIPISetMaxFramerateByScenario] MSDK_SCENARIO_ID_CAMERA_PREVIEW\n");
			pclk = 280000000;
			lineLength = S5K3H5XA_PV_PERIOD_PIXEL_NUMS;
			frameHeight = (10 * pclk)/frameRate/lineLength;
			dummyLine = frameHeight - S5K3H5XA_PV_PERIOD_LINE_NUMS;
			s5k3h5xa.sensorMode = SENSOR_MODE_PREVIEW;
			S5K3H5XA_SetDummy(0, dummyLine);	
			break;			
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			S5K3H5XADB("[S5K3H5XA] [S5K3H5XAMIPISetMaxFramerateByScenario] MSDK_SCENARIO_ID_VIDEO_PREVIEW\n");
		
			pclk = 280000000;
			lineLength = S5K3H5XA_VIDEO_PERIOD_PIXEL_NUMS;
			frameHeight = (10 * pclk)/frameRate/lineLength;
			dummyLine = frameHeight - S5K3H5XA_VIDEO_PERIOD_LINE_NUMS;
			s5k3h5xa.sensorMode = SENSOR_MODE_VIDEO;
			S5K3H5XA_SetDummy(0, dummyLine);
			break;			
			 break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_CAMERA_ZSD:	
			S5K3H5XADB("[S5K3H5XA] [S5K3H5XAMIPISetMaxFramerateByScenario] MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG or MSDK_SCENARIO_ID_CAMERA_ZSD\n");
			pclk = 280000000;
			lineLength = S5K3H5XA_FULL_PERIOD_PIXEL_NUMS;
			frameHeight = (10 * pclk)/frameRate/lineLength;
			dummyLine = frameHeight - S5K3H5XA_FULL_PERIOD_LINE_NUMS;
			s5k3h5xa.sensorMode = SENSOR_MODE_CAPTURE;
			S5K3H5XA_SetDummy(0, dummyLine);
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


UINT32 S5K3H5XAMIPIGetDefaultFramerateByScenario(MSDK_SCENARIO_ID_ENUM scenarioId, MUINT32 *pframeRate) 
{

	switch (scenarioId) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			 *pframeRate = 300;
			 break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
			 *pframeRate = 300;
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

//void S5K3H5XAMIPIGetAEAWBLock(UINT32 *pAElockRet32,UINT32 *pAWBlockRet32)
//{
//    *pAElockRet32 = 1;
//	*pAWBlockRet32 = 1;
//    //SENSORDB("S5K8AAYX_MIPIGetAEAWBLock,AE=%d ,AWB=%d\n,",*pAElockRet32,*pAWBlockRet32);
//}


UINT32 S5K3H5XAFeatureControl(MSDK_SENSOR_FEATURE_ENUM FeatureId,
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
            *pFeatureReturnPara16++= 3200;
            *pFeatureReturnPara16= 2400;
            *pFeatureParaLen=4;
            break;
        case SENSOR_FEATURE_GET_PERIOD:
				*pFeatureReturnPara16++= S5K3H5XA_FeatureControl_PERIOD_PixelNum;
				*pFeatureReturnPara16= S5K3H5XA_FeatureControl_PERIOD_LineNum;
				*pFeatureParaLen=4;
				break;
        case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
			switch(S5K3H5XACurrentScenarioId)
			{
				case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
					*pFeatureReturnPara32 = 280000000;
					*pFeatureParaLen=4;
					break;
				case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
					*pFeatureReturnPara32 = 280000000;
					*pFeatureParaLen=4;
					break;	 
				case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
				case MSDK_SCENARIO_ID_CAMERA_ZSD:
					*pFeatureReturnPara32 = 280000000;
					*pFeatureParaLen=4;
					break;
				default:
					*pFeatureReturnPara32 = 280000000;
					*pFeatureParaLen=4;
					break;
			}
		    break;
        case SENSOR_FEATURE_SET_ESHUTTER:
            S5K3H5XA_SetShutter(*pFeatureData16);
            break;
        case SENSOR_FEATURE_SET_NIGHTMODE:
            S5K3H5XA_NightMode((BOOL) *pFeatureData16);
            break;
        case SENSOR_FEATURE_SET_GAIN:
            S5K3H5XA_SetGain((UINT16) *pFeatureData16);
            break;
        case SENSOR_FEATURE_SET_FLASHLIGHT:
            break;
        case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
            //S5K3H5XA_isp_master_clock=*pFeatureData32;
            break;
        case SENSOR_FEATURE_SET_REGISTER:
            S5K3H5XA_write_cmos_sensor(pSensorRegData->RegAddr, pSensorRegData->RegData);
            break;
        case SENSOR_FEATURE_GET_REGISTER:
            pSensorRegData->RegData = S5K3H5XA_read_cmos_sensor(pSensorRegData->RegAddr);
            break;
        case SENSOR_FEATURE_SET_CCT_REGISTER:
            SensorRegNumber=FACTORY_END_ADDR;
            for (i=0;i<SensorRegNumber;i++)
            {
            	spin_lock(&s5k3h5xamipiraw_drv_lock);
                S5K3H5XASensorCCT[i].Addr=*pFeatureData32++;
                S5K3H5XASensorCCT[i].Para=*pFeatureData32++;
				spin_unlock(&s5k3h5xamipiraw_drv_lock);
            }
            break;
        case SENSOR_FEATURE_GET_CCT_REGISTER:
            SensorRegNumber=FACTORY_END_ADDR;
            if (*pFeatureParaLen<(SensorRegNumber*sizeof(SENSOR_REG_STRUCT)+4))
                return FALSE;
            *pFeatureData32++=SensorRegNumber;
            for (i=0;i<SensorRegNumber;i++)
            {
                *pFeatureData32++=S5K3H5XASensorCCT[i].Addr;
                *pFeatureData32++=S5K3H5XASensorCCT[i].Para;
            }
            break;
        case SENSOR_FEATURE_SET_ENG_REGISTER:
            SensorRegNumber=ENGINEER_END;
            for (i=0;i<SensorRegNumber;i++)
            {
            	spin_lock(&s5k3h5xamipiraw_drv_lock);
                S5K3H5XASensorReg[i].Addr=*pFeatureData32++;
                S5K3H5XASensorReg[i].Para=*pFeatureData32++;
				spin_unlock(&s5k3h5xamipiraw_drv_lock);
            }
            break;
        case SENSOR_FEATURE_GET_ENG_REGISTER:
            SensorRegNumber=ENGINEER_END;
            if (*pFeatureParaLen<(SensorRegNumber*sizeof(SENSOR_REG_STRUCT)+4))
                return FALSE;
            *pFeatureData32++=SensorRegNumber;
            for (i=0;i<SensorRegNumber;i++)
            {
                *pFeatureData32++=S5K3H5XASensorReg[i].Addr;
                *pFeatureData32++=S5K3H5XASensorReg[i].Para;
            }
            break;
        case SENSOR_FEATURE_GET_REGISTER_DEFAULT:
            if (*pFeatureParaLen>=sizeof(NVRAM_SENSOR_DATA_STRUCT))
            {
                pSensorDefaultData->Version=NVRAM_CAMERA_SENSOR_FILE_VERSION;
                pSensorDefaultData->SensorId=S5K3H5XA_SENSOR_ID;
                memcpy(pSensorDefaultData->SensorEngReg, S5K3H5XASensorReg, sizeof(SENSOR_REG_STRUCT)*ENGINEER_END);
                memcpy(pSensorDefaultData->SensorCCTReg, S5K3H5XASensorCCT, sizeof(SENSOR_REG_STRUCT)*FACTORY_END_ADDR);
            }
            else
                return FALSE;
            *pFeatureParaLen=sizeof(NVRAM_SENSOR_DATA_STRUCT);
            break;
        case SENSOR_FEATURE_GET_CONFIG_PARA:
            memcpy(pSensorConfigData, &S5K3H5XASensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
            *pFeatureParaLen=sizeof(MSDK_SENSOR_CONFIG_STRUCT);
            break;
        case SENSOR_FEATURE_CAMERA_PARA_TO_SENSOR:
            S5K3H5XA_camera_para_to_sensor();
            break;

        case SENSOR_FEATURE_SENSOR_TO_CAMERA_PARA:
            S5K3H5XA_sensor_to_camera_para();
            break;
        case SENSOR_FEATURE_GET_GROUP_COUNT:
            *pFeatureReturnPara32++=S5K3H5XA_get_sensor_group_count();
            *pFeatureParaLen=4;
            break;
        case SENSOR_FEATURE_GET_GROUP_INFO:
            S5K3H5XA_get_sensor_group_info(pSensorGroupInfo->GroupIdx, pSensorGroupInfo->GroupNamePtr, &pSensorGroupInfo->ItemCount);
            *pFeatureParaLen=sizeof(MSDK_SENSOR_GROUP_INFO_STRUCT);
            break;
        case SENSOR_FEATURE_GET_ITEM_INFO:
            S5K3H5XA_get_sensor_item_info(pSensorItemInfo->GroupIdx,pSensorItemInfo->ItemIdx, pSensorItemInfo);
            *pFeatureParaLen=sizeof(MSDK_SENSOR_ITEM_INFO_STRUCT);
            break;

        case SENSOR_FEATURE_SET_ITEM_INFO:
            S5K3H5XA_set_sensor_item_info(pSensorItemInfo->GroupIdx, pSensorItemInfo->ItemIdx, pSensorItemInfo->ItemValue);
            *pFeatureParaLen=sizeof(MSDK_SENSOR_ITEM_INFO_STRUCT);
            break;

        case SENSOR_FEATURE_GET_ENG_INFO:
            pSensorEngInfo->SensorId = 129;
            pSensorEngInfo->SensorType = CMOS_SENSOR;
           
   			pSensorEngInfo->SensorOutputDataFormat=SENSOR_OUTPUT_FORMAT_RAW_Gr; //gb r

            *pFeatureParaLen=sizeof(MSDK_SENSOR_ENG_INFO_STRUCT);
            break;
        case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
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
            S5K3H5XASetVideoMode(*pFeatureData16);
            break;
        case SENSOR_FEATURE_CHECK_SENSOR_ID:
            S5K3H5XAGetSensorID(pFeatureReturnPara32);
            break;
        case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE:
            S5K3H5XASetAutoFlickerMode((BOOL)*pFeatureData16, *(pFeatureData16+1));
	        break;
        case SENSOR_FEATURE_SET_TEST_PATTERN:
            S5K3H5XASetTestPatternMode((BOOL)*pFeatureData16);
            break;
		case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE://for factory mode auto testing             
            *pFeatureReturnPara32= S5K3H5XA_TEST_PATTERN_CHECKSUM;
            *pFeatureParaLen=4;                             
            break;
		case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
			S5K3H5XAMIPISetMaxFramerateByScenario((MSDK_SCENARIO_ID_ENUM)*pFeatureData32, *(pFeatureData32+1));
			break;
		case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
			S5K3H5XAMIPIGetDefaultFramerateByScenario((MSDK_SCENARIO_ID_ENUM)*pFeatureData32, (MUINT32 *)(*(pFeatureData32+1)));
			break;
		//case SENSOR_FEATURE_GET_AE_AWB_LOCK_INFO:
			//S5K3H5XAMIPIGetAEAWBLock((*pFeatureData32),*(pFeatureData32+1));
			//break;
        default:
            break;
    }
    return ERROR_NONE;
}	/* S5K3H5XAFeatureControl() */


SENSOR_FUNCTION_STRUCT	SensorFuncS5K3H5XA=
{
    S5K3H5XAOpen,
    S5K3H5XAGetInfo,
    S5K3H5XAGetResolution,
    S5K3H5XAFeatureControl,
    S5K3H5XAControl,
    S5K3H5XAClose
};

UINT32 S5K3H5XA_MIPI_RAW_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
    /* To Do : Check Sensor status here */
    if (pfFunc!=NULL)
        *pfFunc=&SensorFuncS5K3H5XA;

    return ERROR_NONE;
}   /* SensorInit() */

