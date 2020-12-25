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

#include "gs8604mipiraw_2lane_Sensor.h"
#include "gs8604mipiraw_2lane_Camera_Sensor_para.h"
#include "gs8604mipiraw_2lane_CameraCustomized.h"

#if 0
#include <linux/hw_module_info.h>

static hw_module_info hw_info={
			.type = HW_MODULE_TYPE_SUB_CAMERA,
			.id = 0x219,
			.priority = HW_MODULE_PRIORITY_SUB_CAMERA,
			.name = "gs8604_2lane",
			.vendor = "sony",
			.more = "8M"
};
#endif

typedef struct _GC8604_2LANE_OTP_STRUCT
{
	kal_uint8 otp_head;
	
	kal_uint8 SensorBrand[2];

	kal_uint8 ModuleOriginPlace[1];

	kal_uint8 ModuleFactoryNum[2];

	kal_uint8 ModuleNum[5];

	kal_uint8 Reseverd[1];
	
	kal_uint8 ModuleCheck[1];
}GC8604_2LANE_OTP_STRUCT, *PGC8604_2LANE_OTP_STRUCT;

static kal_bool  GS8604MIPI_MPEG4_encode_mode = KAL_FALSE;
static kal_bool GS8604MIPI_Auto_Flicker_mode = KAL_FALSE;


static kal_uint8 GS8604MIPI_sensor_write_I2C_address = GS8604MIPI_WRITE_ID;
static kal_uint8 GS8604MIPI_sensor_read_I2C_address = GS8604MIPI_READ_ID;

static UINT16  GS8604VIDEO_MODE_TARGET_FPS = 30;

static struct GS8604MIPI_sensor_STRUCT GS8604MIPI_sensor={
	GS8604MIPI_WRITE_ID,//i2c_write_id
	GS8604MIPI_READ_ID,//i2c_read_id
	KAL_TRUE,//first_init
	KAL_FALSE,//fix_video_fps
	KAL_TRUE,//pv_mode
	KAL_FALSE,//video_mode
	KAL_FALSE,//capture_mode
	KAL_FALSE,//night_mode
	KAL_FALSE,//mirror_flip
	GS8604MIPI_PREVIEW_CLK,//pv_pclk
	GS8604MIPI_VIDEO_CLK,//video_pclk
	GS8604MIPI_CAPTURE_CLK,//capture_pclk
	0,//pv_shutter
	0,//video_shutter
	0,//cp_shutter
	GS8604MIPI_BASE_GAIN,//pv_gain
	GS8604MIPI_BASE_GAIN,//video_gain
	GS8604MIPI_BASE_GAIN,//cp_gain
	GS8604MIPI_PV_LINE_LENGTH_PIXELS,//pv_line_length
	GS8604MIPI_PV_FRAME_LENGTH_LINES,//pv_frame_length
	GS8604MIPI_VIDEO_LINE_LENGTH_PIXELS,//video_line_length
	GS8604MIPI_VIDEO_FRAME_LENGTH_LINES,//video_frame_length
	GS8604MIPI_FULL_LINE_LENGTH_PIXELS,//cp_line_length
	GS8604MIPI_FULL_FRAME_LENGTH_LINES,//cp_frame_length
	0,//pv_dummy_pixels
	0,//pv_dummy_lines
	0,//video_dummy_pixels
	0,//video_dummy_lines
	0,//cp_dummy_pixels
	0,//cp_dummy_lines
	GS8604MIPI_VIDEO_MAX_FRAMERATE//video_current_frame_rate
};
static MSDK_SCENARIO_ID_ENUM CurrentScenarioId = MSDK_SCENARIO_ID_CAMERA_PREVIEW;
static kal_uint16	GS8604MIPI_sensor_gain_base=0x0;
/* MAX/MIN Explosure Lines Used By AE Algorithm */
static kal_uint16 GS8604MIPI_MAX_EXPOSURE_LINES = GS8604MIPI_PV_FRAME_LENGTH_LINES-5;//650;
static kal_uint8  GS8604MIPI_MIN_EXPOSURE_LINES = 2;
static kal_uint8  test_pattern_flag=0;
#define GS8604_TEST_PATTERN_CHECKSUM (0x9e08861c)

static kal_uint32 GS8604MIPI_isp_master_clock;
static DEFINE_SPINLOCK(GS8604_drv_lock);

#define GS8604_DEBUG
#ifdef  GS8604_DEBUG
    #define SENSORDB(fmt, arg...) printk( "[GS8604MIPIRaw] "  fmt, ##arg)
#else
    #define SENSORDB(fmt, arg...)
#endif

#define RETAILMSG(x,...)
#define TEXT
static UINT8 GS8604MIPIPixelClockDivider=0;
static kal_uint16 GS8604MIPI_sensor_id=0;
static MSDK_SENSOR_CONFIG_STRUCT GS8604MIPISensorConfigData;
static kal_uint32 GS8604MIPI_FAC_SENSOR_REG;
static kal_uint16 GS8604MIPI_sensor_flip_value;
#define GS8604MIPI_MaxGainIndex (97)
static kal_uint16 GS8604MIPI_sensorGainMapping[GS8604MIPI_MaxGainIndex][2] ={
{ 64 ,0  },
{ 68 ,12 },
{ 71 ,23 },
{ 74 ,33 },
{ 77 ,42 },
{ 81 ,52 },
{ 84 ,59 },
{ 87 ,66 },
{ 90 ,73 },
{ 93 ,79 },
{ 96 ,85 },
{ 100,91 },
{ 103,96 },
{ 106,101},
{ 109,105},
{ 113,110},
{ 116,114},
{ 120,118},
{ 122,121},
{ 125,125},
{ 128,128},
{ 132,131},
{ 135,134},
{ 138,137},
{ 141,139},
{ 144,142},
{ 148,145},
{ 151,147},
{ 153,149},
{ 157,151},
{ 160,153},
{ 164,156},
{ 168,158},
{ 169,159},
{ 173,161},
{ 176,163},
{ 180,165},
{ 182,166},
{ 187,168},
{ 189,169},
{ 193,171},
{ 196,172},
{ 200,174},
{ 203,175},
{ 205,176},
{ 208,177},
{ 213,179},
{ 216,180},
{ 219,181},
{ 222,182},
{ 225,183},
{ 228,184},
{ 232,185},
{ 235,186},
{ 238,187},
{ 241,188},
{ 245,189},
{ 249,190},
{ 253,191},
{ 256,192},
{ 260,193},
{ 265,194},
{ 269,195},
{ 274,196},
{ 278,197},
{ 283,198},
{ 288,199},
{ 293,200},
{ 298,201},
{ 304,202},
{ 310,203},
{ 315,204},
{ 322,205},
{ 328,206},
{ 335,207},
{ 342,208},
{ 349,209},
{ 357,210},
{ 365,211},
{ 373,212},
{ 381,213},
{ 400,215},
{ 420,217},
{ 432,218},
{ 443,219},
{ 468,221},
{ 482,222},
{ 497,223},
{ 512,224},
{ 529,225},
{ 546,226},
{ 566,227},
{ 585,228},
{ 607,229},
{ 631,230},
{ 656,231},
{ 683,232}
};
/* FIXME: old factors and DIDNOT use now. s*/
static SENSOR_REG_STRUCT GS8604MIPISensorCCT[]=CAMERA_SENSOR_CCT_DEFAULT_VALUE;
static SENSOR_REG_STRUCT GS8604MIPISensorReg[ENGINEER_END]=CAMERA_SENSOR_REG_DEFAULT_VALUE;
/* FIXME: old factors and DIDNOT use now. e*/
extern int iReadReg(u16 a_u2Addr , u8 * a_puBuff , u16 i2cId);
extern int iWriteReg(u16 a_u2Addr , u32 a_u4Data , u32 a_u4Bytes , u16 i2cId);
#define GS8604MIPI_write_cmos_sensor(addr, para) iWriteReg((u16) addr , (u32) para , 1, GS8604MIPI_WRITE_ID)

static kal_uint16 GS8604MIPI_read_cmos_sensor(kal_uint32 addr)
{
	kal_uint16 get_byte=0;
    iReadReg((u16) addr ,(u8*)&get_byte,GS8604MIPI_WRITE_ID);
    return get_byte;
}



#define GS8604_2lane_OTP
#ifdef GS8604_2lane_OTP

static kal_uint8 m_mir_flag = false;

static kal_uint8 gs8604_2lane_otp_get_checker(GC8604_2LANE_OTP_STRUCT check);
static BOOL gs8604_check_moudule(GC8604_2LANE_OTP_STRUCT check);
static BOOL OnReadOtpCheckSensor();

static BOOL OnReadOtpGS8604();
static BOOL GS8604_ReadIDFromOtp();
static BOOL GS8604_ReadAWBFromOtp();
static BOOL GS8604_ReadLSCFromOtp();
static BOOL TestAWBforGS8604(BYTE RoverG_dec,BYTE BoverG_dec,BYTE GboverGr_dec);
static kal_uint16 CheckOTPFlagGS8604(kal_uint16 page,kal_uint16 address, kal_uint16 i);
static BOOL ReadOTPGS8604(kal_uint16 page,kal_uint16 address,BYTE* value,kal_uint16 length,BOOL ecc);



static kal_uint8 gs8604_2lane_otp_get_checker(GC8604_2LANE_OTP_STRUCT check)
{
	kal_uint16 m_num = 0;
	SENSORDB("check.otp_head = 0x%x\n", check.otp_head);
	
	SENSORDB("check.SensorBrand[0] 		 = 0x%x\n", check.SensorBrand[0]);
	SENSORDB("check.SensorBrand[1]       = 0x%x\n", check.SensorBrand[1]);

	SENSORDB("check.ModuleOriginPlace[0] = 0x%x\n", check.ModuleOriginPlace[0]);

	SENSORDB("check.ModuleFactoryNum[0]  = 0x%x\n", check.ModuleFactoryNum[0]);
	SENSORDB("check.ModuleFactoryNum[1]  = 0x%x\n", check.ModuleFactoryNum[1]);

	SENSORDB("check.ModuleNum[0]         = 0x%x\n", check.ModuleNum[0]);
	SENSORDB("check.ModuleNum[1]         = 0x%x\n", check.ModuleNum[1]);
	SENSORDB("check.ModuleNum[2]         = 0x%x\n", check.ModuleNum[2]);
	SENSORDB("check.ModuleNum[3]         = 0x%x\n", check.ModuleNum[3]);
	SENSORDB("check.ModuleNum[4]         = 0x%x\n", check.ModuleNum[4]);

	SENSORDB("check.Reseverd[0] = 0x%x\n", check.Reseverd[0]);
	SENSORDB("check.ModuleCheck[0] = 0x%x\n", check.ModuleCheck[0]);

	
	m_num += check.SensorBrand[0];
	m_num += check.SensorBrand[1];

	m_num += check.ModuleOriginPlace[0];

	m_num += check.ModuleFactoryNum[0];
	m_num += check.ModuleFactoryNum[1];

	m_num += check.ModuleNum[0];
	m_num += check.ModuleNum[1];
	m_num += check.ModuleNum[2];
	m_num += check.ModuleNum[3];
	m_num += check.ModuleNum[4];

	m_num += check.Reseverd[0];

	return (kal_uint8)(m_num % 255 + 1);
}

static BOOL gs8604_check_moudule(GC8604_2LANE_OTP_STRUCT check)
{
	kal_uint8 m_check;

	m_check = gs8604_2lane_otp_get_checker(check);
		
	if(0x66 == check.otp_head)
	{
		if(('G' == check.SensorBrand[0]) && ('C' == check.SensorBrand[1]))
		{

			if( 0xa == check.ModuleOriginPlace[0])
			{
				if((0x00 == check.ModuleFactoryNum[0]) && (0x01 == check.ModuleFactoryNum[1]))
				{
					if((0x01 == check.ModuleNum[0])  &&
						(0x00 == check.ModuleNum[1]) &&
						(0x00 == check.ModuleNum[2]) &&
						(0x00 == check.ModuleNum[3]) &&
						(0x00 == check.ModuleNum[4]))
					{
						if(0x00 == check.Reseverd[0])
						{
							if(m_check == check.ModuleCheck[0])
								return true;
						}
					}
				}
			}

		}
	}

	return false;
}

static BOOL OnReadOtpCheckSensor()
{
	kal_uint8 i;
	kal_uint16 reVal; 
	kal_uint16 reBaseAddress;

	GC8604_2LANE_OTP_STRUCT otp_checker;
	
	GS8604MIPI_write_cmos_sensor(0x0100,0x0);		//set SW_standby mode 

	GS8604MIPI_write_cmos_sensor(0x3302, 0x02);		//set otp write clock 24M
	GS8604MIPI_write_cmos_sensor(0x3303, 0x58);

	GS8604MIPI_write_cmos_sensor(0x012a, 0x18);		//set input clock frequency
	GS8604MIPI_write_cmos_sensor(0x012b, 0x00);

	GS8604MIPI_write_cmos_sensor(0x3300, 0x08);		//set ecc off

	GS8604MIPI_write_cmos_sensor(0x3200, 0x01);

	while(1)
	{
		reVal = GS8604MIPI_read_cmos_sensor(0x3201);

		if(reVal &0x01)
			break;
	}

	GS8604MIPI_write_cmos_sensor(0x3202, 0);		//set page 0

	reBaseAddress = 0x3204;
/*
	kal_uint8* pointer = (kal_uint8*)&otp_checker;
	
	for(i = 0; i < sizeof(GC8604_2LANE_OTP_STRUCT) / sizeof(kal_uint8); i++)
	{
		reVal = GS8604MIPI_read_cmos_sensor(reBaseAddress + i);
		
		*(pointer + i) = (kal_uint8)reVal;
	}
*/
	otp_checker.otp_head	   = GS8604MIPI_read_cmos_sensor(reBaseAddress + 0);
	otp_checker.SensorBrand[0] = GS8604MIPI_read_cmos_sensor(reBaseAddress + 1);
	otp_checker.SensorBrand[1] = GS8604MIPI_read_cmos_sensor(reBaseAddress + 2);

	otp_checker.ModuleOriginPlace[0] = GS8604MIPI_read_cmos_sensor(reBaseAddress + 3);

	otp_checker.ModuleFactoryNum[0] = GS8604MIPI_read_cmos_sensor(reBaseAddress + 4);
	otp_checker.ModuleFactoryNum[1] = GS8604MIPI_read_cmos_sensor(reBaseAddress + 5);

	otp_checker.ModuleNum[0] = GS8604MIPI_read_cmos_sensor(reBaseAddress + 6);
	otp_checker.ModuleNum[1] = GS8604MIPI_read_cmos_sensor(reBaseAddress + 7);
	otp_checker.ModuleNum[2] = GS8604MIPI_read_cmos_sensor(reBaseAddress + 8);
	otp_checker.ModuleNum[3] = GS8604MIPI_read_cmos_sensor(reBaseAddress + 9);
	otp_checker.ModuleNum[4] = GS8604MIPI_read_cmos_sensor(reBaseAddress + 10);

	otp_checker.Reseverd[0] = GS8604MIPI_read_cmos_sensor(reBaseAddress + 11);
	
	otp_checker.ModuleCheck[0] = GS8604MIPI_read_cmos_sensor(reBaseAddress + 12);
	
	return gs8604_check_moudule(otp_checker);
}

static BOOL OnReadOtpGS8604()  //the main function
{
	//off lsc
	GS8604MIPI_write_cmos_sensor(0x0190,0x0);

	if (TRUE!= GS8604_ReadIDFromOtp())
	{
		SENSORDB("GS8604 update_otp ID data fail\n");
		return FALSE;
	}
	if (TRUE!= GS8604_ReadLSCFromOtp())
	{
		SENSORDB("GS8604 update_otp lsc data fail\n");
		return FALSE;
	}
	if (TRUE!= GS8604_ReadAWBFromOtp())
	{
		SENSORDB("GS8604 update_otp awb data fail\n");
		return FALSE;
	}
	SENSORDB("GS8604 update_otp success\n");
	return TRUE;
}

static kal_uint16 CheckOTPFlagGS8604(kal_uint16 page,kal_uint16 address, kal_uint16 i)
{
	kal_uint16 reVal;
	kal_uint16 tempBuf[1];
	BOOL ecc = FALSE;
	ReadOTPGS8604(page,address,tempBuf,1,ecc);

	if (i==0)//first
	{
		reVal = (tempBuf[0]>>6)&0x03;
	}
	else if (i==1)//second
	{
		reVal = (tempBuf[0]>>4)&0x03;
	}

	if (reVal == 0)
	{
		return 0; //empty
	}
	else if (reVal&0x02)
	{
		return 1; //invalid
	}
	else
	{
		return 2; //valid
	}
}

static BOOL ReadOTPGS8604(kal_uint16 page,kal_uint16 address,BYTE* value,kal_uint16 length,BOOL ecc)
{
	BYTE tempValue;
	kal_uint16 i = 0;
	kal_uint16 k = 0;
	SENSORDB("start read otp data \n");

	if (ecc==TRUE)
	{
		GS8604MIPI_write_cmos_sensor(0x3300,0x00);
		if (address!=0x3204)
		{
			address = 0x3204;
		}
	}
	else if (ecc == FALSE)
	{
		GS8604MIPI_write_cmos_sensor(0x3300,0x08);
	}
	GS8604MIPI_write_cmos_sensor(0x3200,0x01);
	tempValue=GS8604MIPI_read_cmos_sensor(0x3201);
	if ((tempValue&0x01) ==0x01)
	{
		SENSORDB("check status OK! \n");
	}
	else
	{
		SENSORDB("check status NG! \n");
		return FALSE;
	}
	while (k<length)
	{
		i=0;
		GS8604MIPI_write_cmos_sensor(0x3202,page);
		while(i<64)
		{
			tempValue=GS8604MIPI_read_cmos_sensor(address+i);
			*(value+k) = tempValue;
			SENSORDB("Read,page=0x%x,address=0x%x,val=0x%x\n",page,(address+i),tempValue);
			i++;
			k++;
			if (k>=length)
			{
				break;
			}
		}
		page++;
	}
	SENSORDB("end read otp data \n");
	return TRUE;
}


static BOOL TestAWBforGS8604(BYTE RoverG_dec,BYTE BoverG_dec,BYTE GboverGr_dec)
{
	BYTE RoverG_dec_base = 0x225;//the typcical value
	BYTE BoverG_dec_base = 0x234;//the typcical value
	//BYTE GboverGr_dec_base = ;//the typcical value
	//BYTE Range = ;
	#if 0
	if(((abs(RoverG_dec-RoverG_dec_base)*100)/RoverG_dec_base)>Range
		||((abs(BoverG_dec-BoverG_dec_base)*100)/BoverG_dec_base)>Range
		||((abs(GboverGr_dec-GboverGr_dec_base)*100)/GboverGr_dec_base)>Range)
	{
		SENSORDB("awb data bad \n");
		return FALSE;
	}
	#endif
	SENSORDB("test awb do nothing because gs8604_2lane no r g b dig_gain_reg \n");
	return TRUE;
}

static BOOL GS8604_ReadIDFromOtp()
{
	BYTE page = 0;
	kal_uint16 address[] = {0x3205,0x3211/*,0x321d*/};
	//module info. different module is different
	BYTE MID = 0x01;
	#if 0
	BYTE OTP_Version = 0x01;
	BYTE LensID = 0x01;
	BYTE VCMID = 0x0;
	BYTE DriverID = 0x0;
	BYTE IR_BG = 0x01;
	BYTE CT = 0x01; //5100K
	BYTE ff_af = 0x01;
	BYTE LSF = 0x01;//DNP
	#endif
	int i,temp1;
	BYTE pTemp[12]={0};
	kal_int8 index = -1;
	kal_int16 checksum = 0;
	BOOL ecc = false;
	for(i=0;i<2;i++)
	{
		temp1 = CheckOTPFlagGS8604(0,0x3204,i);
		if (temp1==2)
		{
			index = i;
			break;
		}
	}
	if (index == -1)
	{
		SENSORDB("no module ID data in OTP \n");
		return FALSE;
	}

	if(TRUE != ReadOTPGS8604(page,address[index],pTemp,12,ecc))
	{
		SENSORDB("I2C error in OTP \n");
		return FALSE;
	}

	for (i=0;i<11;i++)
	{
		checksum+=pTemp[i];
	}

	if (pTemp[11] != (checksum%255+1))
	{
		SENSORDB("checksum error in OTP \n");
		return FALSE;
	}
	if (pTemp[0]!=MID)
	{
		SENSORDB("mid error in OTP \n");
		return FALSE;
	}
	#if 0
	if (pTemp[1]!=OTP_Version)
	{
		SENSORDB("otp version error in OTP \n");
		return FALSE;
	}
	if (pTemp[5]!=LensID)
	{
		SENSORDB("lens id error in OTP \n");
		return FALSE;
	}
	if (pTemp[6]!=VCMID)
	{
		SENSORDB("vcm id error in OTP \n");
		return FALSE;
	}
	if (pTemp[7]!=DriverID)
	{
		SENSORDB("driver ic id error in OTP \n");
		return FALSE;
	}
	if (pTemp[8]!=IR_BG)
	{
		SENSORDB("IR_BG error in OTP \n");
		return FALSE;
	}
	if (pTemp[9]!=((CT<<4)|(ff_af<<2)|LSF))
	{
		m_system_interface->AddString("É«ÎÂÖµ »òÕß FF/AF±êÖ¾ »òÕß ¹âÔ´ÀàÐÍ ´íÎó£¡");
		SENSORDB("color temperature or light error in OTP \n");
		return CHECKINFO_FAIL;
	}
	#endif
	return TRUE;
}

static BOOL GS8604_ReadLSCFromOtp()
{
	int i,temp1;
	kal_int8 index = -1;
	BYTE checksum = 0;
	for(i=0;i<2;i++)
	{
		temp1 = CheckOTPFlagGS8604(0,0x3243,i);
		if (temp1==2) {
			index = i;
			SENSORDB("%dth shading is valid\n",index+1);
			break;
		}
	}
	if (index == -1) {
		SENSORDB("no lsc data\n");
		return FALSE;
	}

	if (index == 1)//shading auto load must ecc on ,so only two groups shading
	{
		index = 2;
	}

	GS8604MIPI_write_cmos_sensor(0x0190,0x01);  //shading enable
	GS8604MIPI_write_cmos_sensor(0x0192,index); //auto shading table
	GS8604MIPI_write_cmos_sensor(0x0191,0x00);  //lsc color mode
	GS8604MIPI_write_cmos_sensor(0x0193,0x00);  //lsc tuning enable
	GS8604MIPI_write_cmos_sensor(0x01a4,0x03);  //knot point format A

	SENSORDB("lsc data ok\n");
	return TRUE;
}


static BOOL GS8604_ReadAWBFromOtp()
{
	BYTE page = 1;
	kal_uint16 address[] = {0x3205,0x321a/*,0x322f*/};
	BYTE RoverGr_dec,BoverGr_dec,GboverGr_dec;
	int i,temp1;
	BYTE pTemp[21]={0};
	kal_int8 index = -1;
	kal_uint16 checksum = 0;
	BOOL ecc = false;
	for(i=0;i<2;i++)
	{
		temp1 = CheckOTPFlagGS8604(1,0x3204,i);
		if (temp1==2) {
			index = i;
			break;
		}
	}
	if (index == -1) {
		SENSORDB("no awb data in otpl\n");
		return FALSE;
	}
	if(TRUE != ReadOTPGS8604(page,address[index],pTemp,21,ecc))
	{
		SENSORDB("i2c error in otpl\n");
		return FALSE;
	}

	for (i=0;i<20;i++)
	{
		checksum+=pTemp[i];
	}

	if (pTemp[20] != (checksum%255+1))
	{
		SENSORDB("wb checksum error in otpl\n");
		return FALSE;
	}


	RoverGr_dec = (pTemp[0]<<2)+(pTemp[1]>>6);
	BoverGr_dec = (pTemp[2]<<2)+(pTemp[3]>>6);
	GboverGr_dec = (pTemp[4]<<2)+(pTemp[5]>>6);
	if(TRUE!=TestAWBforGS8604(RoverGr_dec,BoverGr_dec,GboverGr_dec))   //module factory test awb
	{
		SENSORDB("awb bad in otpl\n");
		return FALSE;
	}
	return TRUE;

}
#endif

static void GS8604MIPI_write_shutter(kal_uint16 shutter)
{
	kal_uint32 frame_length = 0,line_length=0,shutter1=0;
    kal_uint32 extra_lines = 0;
	kal_uint32 max_exp_shutter = 0;
	kal_uint32 min_framelength = GS8604MIPI_PV_LINE_LENGTH_PIXELS;
	unsigned long flags;
	//SENSORDB("[GS8604MIPI_2LANE]enter GS8604MIPI_write_shutter function\n");
	//SENSORDB("[GS8604MIPI_2LANE]GS8604MIPI_write_shutter shutter:%d \n", shutter);

	    if (GS8604MIPI_sensor.pv_mode == KAL_TRUE)
		 {
		   max_exp_shutter = GS8604MIPI_PV_FRAME_LENGTH_LINES + GS8604MIPI_sensor.pv_dummy_lines-4;
	     }
	     else if (GS8604MIPI_sensor.video_mode== KAL_TRUE)
	     {
	       max_exp_shutter = GS8604MIPI_VIDEO_FRAME_LENGTH_LINES + GS8604MIPI_sensor.video_dummy_lines-4;
		 }
	     else if (GS8604MIPI_sensor.capture_mode== KAL_TRUE)
	     {
	       max_exp_shutter = GS8604MIPI_FULL_FRAME_LENGTH_LINES + GS8604MIPI_sensor.cp_dummy_lines-4;
		 }
		 else
		 	{

			SENSORDB("sensor mode error\n");
		 	}

		 if(shutter > max_exp_shutter)
		   extra_lines = shutter - max_exp_shutter;
		 else
		   extra_lines = 0;
		 if (GS8604MIPI_sensor.pv_mode == KAL_TRUE)
		 {
	       frame_length =GS8604MIPI_PV_FRAME_LENGTH_LINES+ GS8604MIPI_sensor.pv_dummy_lines + extra_lines;
		   line_length = GS8604MIPI_PV_LINE_LENGTH_PIXELS+ GS8604MIPI_sensor.pv_dummy_pixels;
		   spin_lock_irqsave(&GS8604_drv_lock,flags);
		   GS8604MIPI_sensor.pv_line_length = line_length;
		   GS8604MIPI_sensor.pv_frame_length = frame_length;
		   spin_unlock_irqrestore(&GS8604_drv_lock,flags);
		 }
		 else if (GS8604MIPI_sensor.video_mode== KAL_TRUE)
	     {
		    frame_length = GS8604MIPI_VIDEO_FRAME_LENGTH_LINES+ GS8604MIPI_sensor.video_dummy_lines + extra_lines;
			line_length =GS8604MIPI_VIDEO_LINE_LENGTH_PIXELS + GS8604MIPI_sensor.video_dummy_pixels;
			spin_lock_irqsave(&GS8604_drv_lock,flags);
			GS8604MIPI_sensor.video_line_length = line_length;
		    GS8604MIPI_sensor.video_frame_length = frame_length;
			spin_unlock_irqrestore(&GS8604_drv_lock,flags);
		 }
		 else if(GS8604MIPI_sensor.capture_mode== KAL_TRUE)
		 	{
		    frame_length = GS8604MIPI_FULL_FRAME_LENGTH_LINES+ GS8604MIPI_sensor.cp_dummy_lines + extra_lines;
			line_length =GS8604MIPI_FULL_LINE_LENGTH_PIXELS + GS8604MIPI_sensor.cp_dummy_pixels;
			spin_lock_irqsave(&GS8604_drv_lock,flags);
			GS8604MIPI_sensor.cp_line_length = line_length;
		    GS8604MIPI_sensor.cp_frame_length = frame_length;
			spin_unlock_irqrestore(&GS8604_drv_lock,flags);
		 }
		 else
		 	{

			SENSORDB("sensor mode error\n");
		 	}
		//GS8604MIPI_write_cmos_sensor(0x0100,0x00);// STREAM STop

		GS8604MIPI_write_cmos_sensor(0x0160, (frame_length >>8) & 0xFF);
	    GS8604MIPI_write_cmos_sensor(0x0161, frame_length & 0xFF);
	    GS8604MIPI_write_cmos_sensor(0x015a, (shutter >> 8) & 0xFF);
	    GS8604MIPI_write_cmos_sensor(0x015b, shutter  & 0xFF);
		SENSORDB("[GS8604MIPI_2LANE]man-flicker frame_length=%d, shutter=%d\n", frame_length, shutter);
    SENSORDB("[GS8604MIPI_2LANE]exit GS8604MIPI_write_shutter function\n");
}   /* write_GS8604MIPI_shutter */

static kal_uint16 GS8604MIPIReg2Gain(const kal_uint8 iReg)
{
	SENSORDB("[GS8604MIPI_2LANE]enter GS8604MIPIReg2Gain function\n");
    kal_uint8 iI;
    // Range: 1x to 8x
    for (iI = 0; iI < GS8604MIPI_MaxGainIndex; iI++)
	{
        if(iReg < GS8604MIPI_sensorGainMapping[iI][1])
		{
            break;
        }
		if(iReg == GS8604MIPI_sensorGainMapping[iI][1])
		{
			return GS8604MIPI_sensorGainMapping[iI][0];
		}
    }
	SENSORDB("[GS8604MIPI_2LANE]exit GS8604MIPIReg2Gain function\n");
    return GS8604MIPI_sensorGainMapping[iI-1][0];
}
static kal_uint8 GS8604MIPIGain2Reg(const kal_uint16 iGain)
{
	kal_uint8 iI;
	SENSORDB("[GS8604MIPI_2LANE]enter GS8604MIPIGain2Reg function\n");
	for (iI = 0; iI < (GS8604MIPI_MaxGainIndex-1); iI++)
	{
		if(iGain <GS8604MIPI_sensorGainMapping[iI][0])
		{
		    break;
		}
		if(iGain == GS8604MIPI_sensorGainMapping[iI][0])
		{
			return GS8604MIPI_sensorGainMapping[iI][1];
		}
	}
	if(iGain != GS8604MIPI_sensorGainMapping[iI][0])
	{
		printk("[GS8604MIPIGain2Reg] Gain mapping don't correctly:%d %d \n", iGain, GS8604MIPI_sensorGainMapping[iI][0]);
	}
	SENSORDB("[GS8604MIPI_2LANE]exit GS8604MIPIGain2Reg function\n");
	return GS8604MIPI_sensorGainMapping[iI-1][1];
	//return NONE;
}

/*************************************************************************
* FUNCTION
*    GS8604MIPI_SetGain
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
static UINT16 iPreGain = 0;
static void GS8604MIPI_SetGain(UINT16 iGain)
{
	kal_uint8 iReg;
	SENSORDB("[GS8604MIPI_2LANE]enter GS8604MIPI_SetGain function iGain = %d\n",iGain);

	//if (iPreGain != iGain)
	//{
		SENSORDB("[GS8604MIPI_2LANE]enter GS8604MIPI_SetGain function\n");
		iPreGain = iGain;
		iReg = GS8604MIPIGain2Reg(iGain);
		GS8604MIPI_write_cmos_sensor(0x0157, (kal_uint8)iReg);
	//}
	SENSORDB("[GS8604MIPI_2LANE]exit GS8604MIPI_SetGain function\n");
}   /*  GS8604MIPI_SetGain_SetGain  */


/*************************************************************************
* FUNCTION
*    read_GS8604MIPI_gain
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
static kal_uint16 read_GS8604MIPI_gain(void)
{
	SENSORDB("[GS8604MIPI_2LANE]enter read_GS8604MIPI_gain function\n");
	//xb.pang need check
    return (kal_uint16)(GS8604MIPI_read_cmos_sensor(0x0157)) ;
}  /* read_GS8604MIPI_gain */

static void write_GS8604MIPI_gain(kal_uint16 gain)
{
    GS8604MIPI_SetGain(gain);
}
static void GS8604MIPI_camera_para_to_sensor(void)
{

}


/*************************************************************************
* FUNCTION
*    GS8604MIPI_sensor_to_camera_para
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
static void GS8604MIPI_sensor_to_camera_para(void)
{

}

/*************************************************************************
* FUNCTION
*    GS8604MIPI_get_sensor_group_count
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
static kal_int32  GS8604MIPI_get_sensor_group_count(void)
{
    return GROUP_TOTAL_NUMS;
}

static void GS8604MIPI_get_sensor_group_info(kal_uint16 group_idx, kal_int8* group_name_ptr, kal_int32* item_count_ptr)
{

}

static void GS8604MIPI_get_sensor_item_info(kal_uint16 group_idx,kal_uint16 item_idx, MSDK_SENSOR_ITEM_INFO_STRUCT* info_ptr)
{

}
static kal_bool GS8604MIPI_set_sensor_item_info(kal_uint16 group_idx, kal_uint16 item_idx, kal_int32 ItemValue)
{

    return KAL_TRUE;
}
static void GS8604MIPI_SetDummy(const kal_uint16 iPixels, const kal_uint16 iLines)
{

	//xb.pang need check
	kal_uint32 frame_length = 0, line_length = 0;
    if(GS8604MIPI_sensor.pv_mode == KAL_TRUE)
   	{
   	 spin_lock(&GS8604_drv_lock);
   	 GS8604MIPI_sensor.pv_dummy_pixels = iPixels;
	 GS8604MIPI_sensor.pv_dummy_lines = iLines;
   	 GS8604MIPI_sensor.pv_line_length = GS8604MIPI_PV_LINE_LENGTH_PIXELS + iPixels;
	 GS8604MIPI_sensor.pv_frame_length = GS8604MIPI_PV_FRAME_LENGTH_LINES + iLines;
	 spin_unlock(&GS8604_drv_lock);
	 line_length = GS8604MIPI_sensor.pv_line_length;
	 frame_length = GS8604MIPI_sensor.pv_frame_length;
   	}
   else if(GS8604MIPI_sensor.video_mode== KAL_TRUE)
   	{
   	 spin_lock(&GS8604_drv_lock);
   	 GS8604MIPI_sensor.video_dummy_pixels = iPixels;
	 GS8604MIPI_sensor.video_dummy_lines = iLines;
   	 GS8604MIPI_sensor.video_line_length = GS8604MIPI_VIDEO_LINE_LENGTH_PIXELS + iPixels;
	 GS8604MIPI_sensor.video_frame_length = GS8604MIPI_VIDEO_FRAME_LENGTH_LINES + iLines;
	 spin_unlock(&GS8604_drv_lock);
	 line_length = GS8604MIPI_sensor.video_line_length;
	 frame_length = GS8604MIPI_sensor.video_frame_length;
   	}
	else if(GS8604MIPI_sensor.capture_mode== KAL_TRUE)
		{
	  spin_lock(&GS8604_drv_lock);
   	  GS8604MIPI_sensor.cp_dummy_pixels = iPixels;
	  GS8604MIPI_sensor.cp_dummy_lines = iLines;
	  GS8604MIPI_sensor.cp_line_length = GS8604MIPI_FULL_LINE_LENGTH_PIXELS + iPixels;
	  GS8604MIPI_sensor.cp_frame_length = GS8604MIPI_FULL_FRAME_LENGTH_LINES + iLines;
	   spin_unlock(&GS8604_drv_lock);
	  line_length = GS8604MIPI_sensor.cp_line_length;
	  frame_length = GS8604MIPI_sensor.cp_frame_length;
    }
	else
	{
	 SENSORDB("[GS8604MIPI_2LANE][GS8604MIPI_SetDummy] error \n");
	}
      SENSORDB("[GS8604MIPI_2LANE][GS8604MIPI_SetDummy] line_length:%d, frame_length:%d \n", line_length, frame_length);
      GS8604MIPI_write_cmos_sensor(0x0160, (frame_length >>8) & 0xFF);
      GS8604MIPI_write_cmos_sensor(0x0161, frame_length & 0xFF);
      GS8604MIPI_write_cmos_sensor(0x0162, (line_length >>8) & 0xFF);
      GS8604MIPI_write_cmos_sensor(0x0163, line_length & 0xFF);

}   /*  GS8604MIPI_SetDummy */
static void GS8604MIPI_Sensor_Init(void)
{
	SENSORDB("[GS8604MIPI_2LANE]enter GS8604MIPI_Sensor_Init function\n");

	// The register only need to enable 1 time.
	spin_lock(&GS8604_drv_lock);
	GS8604MIPI_Auto_Flicker_mode = KAL_FALSE;	  // reset the flicker status
	spin_unlock(&GS8604_drv_lock);
	SENSORDB("[GS8604MIPI_2LANE]exit GS8604MIPI_Sensor_Init function\n");
}   /*  GS8604MIPI_Sensor_Init  */
static void VideoFullSizeSetting(void)//16:9   6M
{
	SENSORDB("[GS8604MIPI_2LANE]enter VideoFullSizeSetting function\n");
	GS8604MIPI_write_cmos_sensor(0x0100,   0x00);

	GS8604MIPI_write_cmos_sensor(0x30EB, 0x05);
	GS8604MIPI_write_cmos_sensor(0x30EB, 0x0C);
	GS8604MIPI_write_cmos_sensor(0x300A, 0xFF);
	GS8604MIPI_write_cmos_sensor(0x300B, 0xFF);
	GS8604MIPI_write_cmos_sensor(0x30EB, 0x05);
	GS8604MIPI_write_cmos_sensor(0x30EB, 0x09);
	GS8604MIPI_write_cmos_sensor(0x0114, 0x01);
	GS8604MIPI_write_cmos_sensor(0x0128, 0x00);
	GS8604MIPI_write_cmos_sensor(0x012A, 0x18);
	GS8604MIPI_write_cmos_sensor(0x012B, 0x00);



	GS8604MIPI_write_cmos_sensor(0x0160, ((GS8604MIPI_VIDEO_FRAME_LENGTH_LINES >> 8) & 0xFF));
	GS8604MIPI_write_cmos_sensor(0x0161, (GS8604MIPI_VIDEO_FRAME_LENGTH_LINES & 0xFF));
	GS8604MIPI_write_cmos_sensor(0x0162, ((GS8604MIPI_VIDEO_LINE_LENGTH_PIXELS >> 8) & 0xFF));
	GS8604MIPI_write_cmos_sensor(0x0163, (GS8604MIPI_VIDEO_LINE_LENGTH_PIXELS & 0xFF));


	GS8604MIPI_write_cmos_sensor(0x0164, 0x00);
	GS8604MIPI_write_cmos_sensor(0x0165, 0x00);
	GS8604MIPI_write_cmos_sensor(0x0166, 0x0C);
	GS8604MIPI_write_cmos_sensor(0x0167, 0xCF);
	GS8604MIPI_write_cmos_sensor(0x0168, 0x01);
	GS8604MIPI_write_cmos_sensor(0x0169, 0x36);
	GS8604MIPI_write_cmos_sensor(0x016A, 0x08);
	GS8604MIPI_write_cmos_sensor(0x016B, 0x6B);
	GS8604MIPI_write_cmos_sensor(0x016C, 0x0C);
	GS8604MIPI_write_cmos_sensor(0x016D, 0xD0);
	GS8604MIPI_write_cmos_sensor(0x016E, 0x07);
	GS8604MIPI_write_cmos_sensor(0x016F, 0x36);
	GS8604MIPI_write_cmos_sensor(0x0170, 0x01);
	GS8604MIPI_write_cmos_sensor(0x0171, 0x01);

	if(m_mir_flag)
		GS8604MIPI_write_cmos_sensor(0x0172, 0x03);
	else
		GS8604MIPI_write_cmos_sensor(0x0172, 0x00);
	
	GS8604MIPI_write_cmos_sensor(0x0174, 0x00);
	GS8604MIPI_write_cmos_sensor(0x0175, 0x00);
	GS8604MIPI_write_cmos_sensor(0x018C, 0x0A);
	GS8604MIPI_write_cmos_sensor(0x018D, 0x0A);
	GS8604MIPI_write_cmos_sensor(0x0301, 0x05);
	GS8604MIPI_write_cmos_sensor(0x0303, 0x01);
	GS8604MIPI_write_cmos_sensor(0x0304, 0x03);
	GS8604MIPI_write_cmos_sensor(0x0305, 0x03);
	GS8604MIPI_write_cmos_sensor(0x0306, 0x00);
	GS8604MIPI_write_cmos_sensor(0x0307, 0x36);
	GS8604MIPI_write_cmos_sensor(0x0309, 0x0A);
	GS8604MIPI_write_cmos_sensor(0x030B, 0x01);
	GS8604MIPI_write_cmos_sensor(0x030C, 0x00);
	GS8604MIPI_write_cmos_sensor(0x030D, 0x6c);
	GS8604MIPI_write_cmos_sensor(0x455E, 0x00);
	GS8604MIPI_write_cmos_sensor(0x471E, 0x4B);
	GS8604MIPI_write_cmos_sensor(0x4767, 0x0F);
	GS8604MIPI_write_cmos_sensor(0x4750, 0x14);
	GS8604MIPI_write_cmos_sensor(0x4540, 0x00);
	GS8604MIPI_write_cmos_sensor(0x47B4, 0x14);
	GS8604MIPI_write_cmos_sensor(0x4713, 0x30);
	GS8604MIPI_write_cmos_sensor(0x478B, 0x10);
	GS8604MIPI_write_cmos_sensor(0x478F, 0x10);
	GS8604MIPI_write_cmos_sensor(0x4793, 0x10);
	GS8604MIPI_write_cmos_sensor(0x4797, 0x0E);
	GS8604MIPI_write_cmos_sensor(0x479B, 0x0E);
	GS8604MIPI_write_cmos_sensor(0x0100, 0x01);


	SENSORDB("[GS8604MIPI_2LANE]exit VideoFullSizeSetting function\n");
}
static void PreviewSetting(void)
{
	SENSORDB("[GS8604MIPI_2LANE]enter PreviewSetting function\n");
	GS8604MIPI_write_cmos_sensor(0x0100,   0x00);


	GS8604MIPI_write_cmos_sensor(0x30EB,  0x05);
	GS8604MIPI_write_cmos_sensor(0x30EB,  0x0C);
	GS8604MIPI_write_cmos_sensor(0x300A,  0xFF);
	GS8604MIPI_write_cmos_sensor(0x300B,  0xFF);
	GS8604MIPI_write_cmos_sensor(0x30EB,  0x05);
	GS8604MIPI_write_cmos_sensor(0x30EB,  0x09);
	GS8604MIPI_write_cmos_sensor(0x0114,  0x01);
	GS8604MIPI_write_cmos_sensor(0x0128,  0x00);
	GS8604MIPI_write_cmos_sensor(0x012A,  0x18);
	GS8604MIPI_write_cmos_sensor(0x012B,  0x00);



	GS8604MIPI_write_cmos_sensor(0x0160,  ((GS8604MIPI_PV_FRAME_LENGTH_LINES >> 8) & 0xFF));
	GS8604MIPI_write_cmos_sensor(0x0161,  (GS8604MIPI_PV_FRAME_LENGTH_LINES & 0xFF));
	GS8604MIPI_write_cmos_sensor(0x0162,  ((GS8604MIPI_PV_LINE_LENGTH_PIXELS >> 8) & 0xFF));
	GS8604MIPI_write_cmos_sensor(0x0163,  (GS8604MIPI_PV_LINE_LENGTH_PIXELS & 0xFF));


	GS8604MIPI_write_cmos_sensor(0x0164,  0x00);
	GS8604MIPI_write_cmos_sensor(0x0165,  0x00);
	GS8604MIPI_write_cmos_sensor(0x0166,  0x0C);
	GS8604MIPI_write_cmos_sensor(0x0167,  0xCF);
	GS8604MIPI_write_cmos_sensor(0x0168,  0x00);
	GS8604MIPI_write_cmos_sensor(0x0169,  0x00);
	GS8604MIPI_write_cmos_sensor(0x016A,  0x09);
	GS8604MIPI_write_cmos_sensor(0x016B,  0x9F);
	GS8604MIPI_write_cmos_sensor(0x016C,  0x06);
	GS8604MIPI_write_cmos_sensor(0x016D,  0x68);
	GS8604MIPI_write_cmos_sensor(0x016E,  0x04);
	GS8604MIPI_write_cmos_sensor(0x016F,  0xD0);
	GS8604MIPI_write_cmos_sensor(0x0170,  0x01);
	GS8604MIPI_write_cmos_sensor(0x0171,  0x01);

	if(m_mir_flag)
		GS8604MIPI_write_cmos_sensor(0x0172,  0x03);
	else
		GS8604MIPI_write_cmos_sensor(0x0172,  0x00);


	GS8604MIPI_write_cmos_sensor(0x0174,  0x01);
	GS8604MIPI_write_cmos_sensor(0x0175,  0x01);
	GS8604MIPI_write_cmos_sensor(0x018C,  0x0A);
	GS8604MIPI_write_cmos_sensor(0x018D,  0x0A);
	GS8604MIPI_write_cmos_sensor(0x0301,  0x05);
	GS8604MIPI_write_cmos_sensor(0x0303,  0x01);
	GS8604MIPI_write_cmos_sensor(0x0304,  0x03);
	GS8604MIPI_write_cmos_sensor(0x0305,  0x03);
	GS8604MIPI_write_cmos_sensor(0x0306,  0x00);
	GS8604MIPI_write_cmos_sensor(0x0307,  0x36);
	GS8604MIPI_write_cmos_sensor(0x0309,  0x0A);
	GS8604MIPI_write_cmos_sensor(0x030B,  0x01);
	GS8604MIPI_write_cmos_sensor(0x030C,  0x00);
	GS8604MIPI_write_cmos_sensor(0x030D,  0x6c);
	GS8604MIPI_write_cmos_sensor(0x455E,  0x00);
	GS8604MIPI_write_cmos_sensor(0x471E,  0x4B);
	GS8604MIPI_write_cmos_sensor(0x4767,  0x0F);
	GS8604MIPI_write_cmos_sensor(0x4750,  0x14);
	GS8604MIPI_write_cmos_sensor(0x4540,  0x00);
	GS8604MIPI_write_cmos_sensor(0x47B4,  0x14);
	GS8604MIPI_write_cmos_sensor(0x4713,  0x30);
	GS8604MIPI_write_cmos_sensor(0x478B,  0x10);
	GS8604MIPI_write_cmos_sensor(0x478F,  0x10);
	GS8604MIPI_write_cmos_sensor(0x4793,  0x10);
	GS8604MIPI_write_cmos_sensor(0x4797,  0x0E);
	GS8604MIPI_write_cmos_sensor(0x479B,  0x0E);
	GS8604MIPI_write_cmos_sensor(0x0100,  0x01);

	// The register only need to enable 1 time.
	spin_lock(&GS8604_drv_lock);
	//GS8604MIPI_Auto_Flicker_mode = KAL_FALSE;	  // reset the flicker status
	spin_unlock(&GS8604_drv_lock);
	SENSORDB("[GS8604MIPI_2LANE]exit PreviewSetting function\n");
}

static void GS8604MIPI_set_8M(void)
{	//77 capture setting
	SENSORDB("[GS8604MIPI_2LANE]enter GS8604MIPI_set_8M function\n");
	GS8604MIPI_write_cmos_sensor(0x0100,   0x00);

	GS8604MIPI_write_cmos_sensor(0x30EB,   0x05);
	GS8604MIPI_write_cmos_sensor(0x30EB,   0x0C);
	GS8604MIPI_write_cmos_sensor(0x300A,   0xFF);
	GS8604MIPI_write_cmos_sensor(0x300B,   0xFF);
	GS8604MIPI_write_cmos_sensor(0x30EB,   0x05);
	GS8604MIPI_write_cmos_sensor(0x30EB,   0x09);
	GS8604MIPI_write_cmos_sensor(0x0114,   0x01);
	GS8604MIPI_write_cmos_sensor(0x0128,   0x00);
	GS8604MIPI_write_cmos_sensor(0x012A,   0x18);
	GS8604MIPI_write_cmos_sensor(0x012B,   0x00);



	GS8604MIPI_write_cmos_sensor(0x0160,   ((GS8604MIPI_FULL_FRAME_LENGTH_LINES >> 8) & 0xFF));
	GS8604MIPI_write_cmos_sensor(0x0161,   (GS8604MIPI_FULL_FRAME_LENGTH_LINES & 0xFF));
	//GS8604MIPI_write_cmos_sensor(0x0160,   0x0e);
	//GS8604MIPI_write_cmos_sensor(0x0161,   0xae);

	GS8604MIPI_write_cmos_sensor(0x0162,   ((GS8604MIPI_FULL_LINE_LENGTH_PIXELS >> 8) & 0xFF));
	GS8604MIPI_write_cmos_sensor(0x0163,   (GS8604MIPI_FULL_LINE_LENGTH_PIXELS & 0xFF));

	GS8604MIPI_write_cmos_sensor(0x0164,   0x00);
	GS8604MIPI_write_cmos_sensor(0x0165,   0x00);
	GS8604MIPI_write_cmos_sensor(0x0166,   0x0C);
	GS8604MIPI_write_cmos_sensor(0x0167,   0xCF);
	GS8604MIPI_write_cmos_sensor(0x0168,   0x00);
	GS8604MIPI_write_cmos_sensor(0x0169,   0x00);
	GS8604MIPI_write_cmos_sensor(0x016A,   0x09);
	GS8604MIPI_write_cmos_sensor(0x016B,   0x9F);
	GS8604MIPI_write_cmos_sensor(0x016C,   0x0C);
	GS8604MIPI_write_cmos_sensor(0x016D,   0xD0);

	GS8604MIPI_write_cmos_sensor(0x016E,   0x09);
	GS8604MIPI_write_cmos_sensor(0x016F,   0xA0);
	//GS8604MIPI_write_cmos_sensor(0x016E,   0x07);
	//GS8604MIPI_write_cmos_sensor(0x016F,   0x36);

	GS8604MIPI_write_cmos_sensor(0x0170,   0x01);
	GS8604MIPI_write_cmos_sensor(0x0171,   0x01);

	if(m_mir_flag)
		GS8604MIPI_write_cmos_sensor(0x0172,   0x03);
	else
		GS8604MIPI_write_cmos_sensor(0x0172,   0x00);
	
	GS8604MIPI_write_cmos_sensor(0x0174,   0x00);
	GS8604MIPI_write_cmos_sensor(0x0175,   0x00);
	GS8604MIPI_write_cmos_sensor(0x018C,   0x0A);
	GS8604MIPI_write_cmos_sensor(0x018D,   0x0A);
	GS8604MIPI_write_cmos_sensor(0x0301,   0x05);
	GS8604MIPI_write_cmos_sensor(0x0303,   0x01);
	GS8604MIPI_write_cmos_sensor(0x0304,   0x03);
	GS8604MIPI_write_cmos_sensor(0x0305,   0x03);
	GS8604MIPI_write_cmos_sensor(0x0306,   0x00);
	GS8604MIPI_write_cmos_sensor(0x0307,   0x36);
	GS8604MIPI_write_cmos_sensor(0x0309,   0x0A);
	GS8604MIPI_write_cmos_sensor(0x030B,   0x01);
	GS8604MIPI_write_cmos_sensor(0x030C,   0x00);
	GS8604MIPI_write_cmos_sensor(0x030D,   0x6c);
	GS8604MIPI_write_cmos_sensor(0x455E,   0x00);
	GS8604MIPI_write_cmos_sensor(0x471E,   0x4B);
	GS8604MIPI_write_cmos_sensor(0x4767,   0x0F);
	GS8604MIPI_write_cmos_sensor(0x4750,   0x14);
	GS8604MIPI_write_cmos_sensor(0x4540,   0x00);
	GS8604MIPI_write_cmos_sensor(0x47B4,   0x14);
	GS8604MIPI_write_cmos_sensor(0x4713,   0x30);
	GS8604MIPI_write_cmos_sensor(0x478B,   0x10);
	GS8604MIPI_write_cmos_sensor(0x478F,   0x10);
	GS8604MIPI_write_cmos_sensor(0x4793,   0x10);
	GS8604MIPI_write_cmos_sensor(0x4797,   0x0E);
	GS8604MIPI_write_cmos_sensor(0x479B,   0x0E);
	GS8604MIPI_write_cmos_sensor(0x0100,   0x01);

	SENSORDB("[GS8604MIPI_2LANE]exit GS8604MIPI_set_8M function\n");
}
/*****************************************************************************/
/* Windows Mobile Sensor Interface */
/*****************************************************************************/
/*************************************************************************
* FUNCTION
*   GS8604MIPIOpen
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

UINT32 GS8604MIPIOpen_2lane(void)
{
    int  retry = 0;
	kal_uint16 sensorid;
    // check if sensor ID correct
    retry = 3;
	SENSORDB("[GS8604MIPI_2LANE]enter GS8604MIPIOpen function\n");
    do {
		sensorid=(kal_uint16)(((GS8604MIPI_read_cmos_sensor(0x0000)&0x0f)<<8) | GS8604MIPI_read_cmos_sensor(0x0001));
		spin_lock(&GS8604_drv_lock);
		GS8604MIPI_sensor_id =sensorid;
		spin_unlock(&GS8604_drv_lock);
		if (GS8604MIPI_sensor_id == GS8604MIPI_SENSOR_ID)
			break;
	} while (--retry > 0);

    SENSORDB("Read Sensor ID = 0x%04x\n", GS8604MIPI_sensor_id);

    if (GS8604MIPI_sensor_id != GS8604MIPI_SENSOR_ID)
        return ERROR_SENSOR_CONNECT_FAIL;

    GS8604MIPI_Sensor_Init();
	sensorid=read_GS8604MIPI_gain();
	spin_lock(&GS8604_drv_lock);
	GS8604MIPI_sensor.pv_mode=KAL_TRUE;
    GS8604MIPI_sensor_gain_base = sensorid;
	spin_unlock(&GS8604_drv_lock);
	SENSORDB("[GS8604MIPI_2LANE]exit GS8604MIPIOpen function\n");
    return ERROR_NONE;
}

/*************************************************************************
* FUNCTION
*   GS8604MIPIGetSensorID
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
static UINT32 GS8604MIPIGetSensorID(UINT32 *sensorID)
{
    int  retry = 3;
	SENSORDB("[GS8604MIPI_2LANE]enter GS8604MIPIGetSensorID function\n");

    do {
		*sensorID =(kal_uint16)(((GS8604MIPI_read_cmos_sensor(0x0000)&0x0f)<<8) | GS8604MIPI_read_cmos_sensor(0x0001));
        if (*sensorID == GS8604MIPI_SENSOR_ID)  //modified by tyd tianyaping avoid main and front cam id is same 20140214
            break;
    } while (--retry > 0);

	SENSORDB("[GS8604MIPI_2LANE]GS8604MIPIGetSensorID Sensor ID = 0x%04x\n",*sensorID);
    if (*sensorID != GS8604MIPI_SENSOR_ID) {
        *sensorID = 0xFFFFFFFF;
        return ERROR_SENSOR_CONNECT_FAIL;
    }
	*sensorID == GS8604MIPI_SENSOR_ID;
	//	hw_module_info_add(&hw_info);  //add by tyd tianyaping to add hardware info 20140218
	SENSORDB("[GS8604MIPI_2LANE]exit GS8604MIPIGetSensorID function\n");
#ifdef GS8604_2lane_OTP
//		OnReadOtpGS8604();		
	if(OnReadOtpCheckSensor())
		m_mir_flag = true;		//mir
	else
		m_mir_flag = false;		//

	GS8604MIPI_write_cmos_sensor(0x0190,0x0);
#endif
    return ERROR_NONE;
}


/*************************************************************************
* FUNCTION
*   GS8604MIPI_SetShutter
*
* DESCRIPTION
*   This function set e-shutter of GS8604MIPI to change exposure time.
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
static void GS8604MIPI_SetShutter(kal_uint16 iShutter)
{
	//xb.pang need check
	SENSORDB("[GS8604MIPI_2LANE]%s():shutter=%d\n",__FUNCTION__,iShutter);
    if (iShutter < 1)
        iShutter = 1;
	else if(iShutter > 0xffff)
		iShutter = 0xffff;
	unsigned long flags;
	spin_lock_irqsave(&GS8604_drv_lock,flags);
    GS8604MIPI_sensor.pv_shutter = iShutter;
	spin_unlock_irqrestore(&GS8604_drv_lock,flags);
	//xb.pang for zsd ae
    GS8604MIPI_write_shutter(iShutter);
	SENSORDB("[GS8604MIPI_2LANE]exit GS8604MIPI_SetShutter \n");
}   /*  GS8604MIPI_SetShutter   */



/*************************************************************************
* FUNCTION
*   GS8604MIPI_read_shutter
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
static UINT16 GS8604MIPI_read_shutter(void)
{
	//xb.pang need check
    return (UINT16)( (GS8604MIPI_read_cmos_sensor(0x015a)<<8) | GS8604MIPI_read_cmos_sensor(0x015b) );
}

/*************************************************************************
* FUNCTION
*   GS8604MIPI_night_mode
*
* DESCRIPTION
*   This function night mode of GS8604MIPI.
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
static void GS8604MIPI_NightMode(kal_bool bEnable)
{
	SENSORDB("[GS8604MIPI_2LANE]GS8604MIPI_NightMode\n");
}/*	GS8604MIPI_NightMode */



/*************************************************************************
* FUNCTION
*   GS8604MIPIClose
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
static UINT32 GS8604MIPIClose_2lane(void)
{
	//xb.pang need check
    GS8604MIPI_write_cmos_sensor(0x0100,0x00);
    return ERROR_NONE;
}	/* GS8604MIPIClose() */

static void GS8604MIPISetFlipMirror(kal_int32 imgMirror)
{
	//xb.pang need check
    kal_uint8  iTemp;
	SENSORDB("[GS8604MIPI_2LANE]enter GS8604MIPISetFlipMirror function\n");
    iTemp = GS8604MIPI_read_cmos_sensor(0x0172) & 0x03;	//Clear the mirror and flip bits.
    switch (imgMirror)
    {
        case IMAGE_NORMAL:
            GS8604MIPI_write_cmos_sensor(0x0172, 0x03);	//Set normal
            break;
        case IMAGE_V_MIRROR:
            GS8604MIPI_write_cmos_sensor(0x0172, iTemp | 0x01);	//Set flip
            break;
        case IMAGE_H_MIRROR:
            GS8604MIPI_write_cmos_sensor(0x0172, iTemp | 0x02);	//Set mirror
            break;
        case IMAGE_HV_MIRROR:
            GS8604MIPI_write_cmos_sensor(0x0172, 0x00);	//Set mirror and flip
            break;
    }
	SENSORDB("[GS8604MIPI_2LANE]exit GS8604MIPISetFlipMirror function\n");
}

/*
*	FOR TEST PATTERN--test pattern need change sensor out size
*	first_testpattern=false----set sensor out size: preview size
*   first_testpattern=true----set sensor out size: capture size
*/
//bool first_testpattern = false;
static UINT32 GS8604MIPISetTestPatternMode(kal_bool bEnable)
{
    SENSORDB("[GS8604MIPISetTestPatternMode] Test pattern enable:%d\n", bEnable);
    if(GS8604MIPI_sensor.capture_mode == KAL_FALSE)
    {
	    if(bEnable)
		{
			//1640 x 1232
			// enable color bar
			test_pattern_flag=TRUE;
			GS8604MIPI_write_cmos_sensor(0x0600, 0x00);
			GS8604MIPI_write_cmos_sensor(0x0601, 0x02);
			GS8604MIPI_write_cmos_sensor(0x0624, 0x06); //W:3280---h
			GS8604MIPI_write_cmos_sensor(0x0625, 0x68); //		   l
			GS8604MIPI_write_cmos_sensor(0x0626, 0x04); //H:2464   h
			GS8604MIPI_write_cmos_sensor(0x0627, 0xd0); //		   l
			GS8604MIPI_write_cmos_sensor(0x6128, 0x00);
			GS8604MIPI_write_cmos_sensor(0x6129, 0x02);
			GS8604MIPI_write_cmos_sensor(0x613C, 0x06); //W         h
			GS8604MIPI_write_cmos_sensor(0x613D, 0x68); //		    l
			GS8604MIPI_write_cmos_sensor(0x613E, 0x04); //H         h
			GS8604MIPI_write_cmos_sensor(0x613F, 0xd0); // 		    l
			GS8604MIPI_write_cmos_sensor(0x6506, 0x00);
			GS8604MIPI_write_cmos_sensor(0x6507, 0x00);

	    }
		else
		{
			//1640 x 1232
			test_pattern_flag=FALSE;
			GS8604MIPI_write_cmos_sensor(0x0600, 0x00);
			GS8604MIPI_write_cmos_sensor(0x0601, 0x00);
			GS8604MIPI_write_cmos_sensor(0x0624, 0x06); //W:3280---h
			GS8604MIPI_write_cmos_sensor(0x0625, 0x68); //		   l
			GS8604MIPI_write_cmos_sensor(0x0626, 0x04); //H:2464   h
			GS8604MIPI_write_cmos_sensor(0x0627, 0xd0); //		   l
			GS8604MIPI_write_cmos_sensor(0x6128, 0x00);
			GS8604MIPI_write_cmos_sensor(0x6129, 0x02);
			GS8604MIPI_write_cmos_sensor(0x613C, 0x06); //W         h
			GS8604MIPI_write_cmos_sensor(0x613D, 0x68); //		    l
			GS8604MIPI_write_cmos_sensor(0x613E, 0x04); //H         h
			GS8604MIPI_write_cmos_sensor(0x613F, 0xd0); // 		    l
			GS8604MIPI_write_cmos_sensor(0x6506, 0x00);
			GS8604MIPI_write_cmos_sensor(0x6507, 0x00);

	    }
    }
	else
	{
        if(bEnable)
		{
			//3280 x 2464
			// enable color bar
			test_pattern_flag=TRUE;
			GS8604MIPI_write_cmos_sensor(0x0600, 0x00);
			GS8604MIPI_write_cmos_sensor(0x0601, 0x02);
			GS8604MIPI_write_cmos_sensor(0x0624, 0x0C); //W:3280---h
			GS8604MIPI_write_cmos_sensor(0x0625, 0xD0); //		   l
			GS8604MIPI_write_cmos_sensor(0x0626, 0x09); //H:2464   h
			GS8604MIPI_write_cmos_sensor(0x0627, 0xA0); //		   l
			GS8604MIPI_write_cmos_sensor(0x6128, 0x00);
			GS8604MIPI_write_cmos_sensor(0x6129, 0x02);
			GS8604MIPI_write_cmos_sensor(0x613C, 0x0C); //W         h
			GS8604MIPI_write_cmos_sensor(0x613D, 0xD0); //		    l
			GS8604MIPI_write_cmos_sensor(0x613E, 0x09); //H         h
			GS8604MIPI_write_cmos_sensor(0x613F, 0xA0); // 		    l
			GS8604MIPI_write_cmos_sensor(0x6506, 0x00);
			GS8604MIPI_write_cmos_sensor(0x6507, 0x00);

	    }
		else
		{
			test_pattern_flag=FALSE;
			GS8604MIPI_write_cmos_sensor(0x0600, 0x00);
			GS8604MIPI_write_cmos_sensor(0x0601, 0x02);
			GS8604MIPI_write_cmos_sensor(0x0624, 0x0C); //W:3280---h
			GS8604MIPI_write_cmos_sensor(0x0625, 0xD0); //		   l
			GS8604MIPI_write_cmos_sensor(0x0626, 0x09); //H:2464   h
			GS8604MIPI_write_cmos_sensor(0x0627, 0xA0); //		   l
			GS8604MIPI_write_cmos_sensor(0x6128, 0x00);
			GS8604MIPI_write_cmos_sensor(0x6129, 0x02);
			GS8604MIPI_write_cmos_sensor(0x613C, 0x0C); //W         h
			GS8604MIPI_write_cmos_sensor(0x613D, 0xD0); //		    l
			GS8604MIPI_write_cmos_sensor(0x613E, 0x09); //H         h
			GS8604MIPI_write_cmos_sensor(0x613F, 0xA0); // 		    l
			GS8604MIPI_write_cmos_sensor(0x6506, 0x00);
			GS8604MIPI_write_cmos_sensor(0x6507, 0x00);


	    }
	}

    return ERROR_NONE;
}

/*************************************************************************
* FUNCTION
*   GS8604MIPIPreview
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
static UINT32 GS8604MIPIPreview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                                                MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	kal_uint16 iStartX = 0, iStartY = 0;
	SENSORDB("[GS8604MIPI_2LANE]enter GS8604MIPIPreview function\n");
	spin_lock(&GS8604_drv_lock);
	GS8604MIPI_MPEG4_encode_mode = KAL_FALSE;
	GS8604MIPI_sensor.video_mode=KAL_FALSE;
	GS8604MIPI_sensor.pv_mode=KAL_TRUE;
	GS8604MIPI_sensor.capture_mode=KAL_FALSE;
	spin_unlock(&GS8604_drv_lock);
	PreviewSetting();
	iStartX += GS8604MIPI_IMAGE_SENSOR_PV_STARTX;
	iStartY += GS8604MIPI_IMAGE_SENSOR_PV_STARTY;
	spin_lock(&GS8604_drv_lock);
	GS8604MIPI_sensor.cp_dummy_pixels = 0;
	GS8604MIPI_sensor.cp_dummy_lines = 0;
	GS8604MIPI_sensor.pv_dummy_pixels = 0;
	GS8604MIPI_sensor.pv_dummy_lines = 0;
	GS8604MIPI_sensor.video_dummy_pixels = 0;
	GS8604MIPI_sensor.video_dummy_lines = 0;
	GS8604MIPI_sensor.pv_line_length = GS8604MIPI_PV_LINE_LENGTH_PIXELS+GS8604MIPI_sensor.pv_dummy_pixels;
	GS8604MIPI_sensor.pv_frame_length = GS8604MIPI_PV_FRAME_LENGTH_LINES+GS8604MIPI_sensor.pv_dummy_lines;
	spin_unlock(&GS8604_drv_lock);

	GS8604MIPI_SetDummy(GS8604MIPI_sensor.pv_dummy_pixels,GS8604MIPI_sensor.pv_dummy_lines);
	GS8604MIPI_SetShutter(GS8604MIPI_sensor.pv_shutter);
	spin_lock(&GS8604_drv_lock);
	memcpy(&GS8604MIPISensorConfigData, sensor_config_data, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
	spin_unlock(&GS8604_drv_lock);
	image_window->GrabStartX= iStartX;
	image_window->GrabStartY= iStartY;
	image_window->ExposureWindowWidth= GS8604MIPI_REAL_PV_WIDTH ;
	image_window->ExposureWindowHeight= GS8604MIPI_REAL_PV_HEIGHT ;
	SENSORDB("[GS8604MIPI_2LANE]eXIT GS8604MIPIPreview function\n");
	return ERROR_NONE;
	}	/* GS8604MIPIPreview() */

/*************************************************************************
* FUNCTION
* RETURNS
*   None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static UINT32 GS8604MIPIVideo(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                                                MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	kal_uint16 iStartX = 0, iStartY = 0;
	SENSORDB("[GS8604MIPI_2LANE]enter GS8604MIPIVideo function\n");
	spin_lock(&GS8604_drv_lock);
    GS8604MIPI_MPEG4_encode_mode = KAL_TRUE;
	GS8604MIPI_sensor.video_mode=KAL_TRUE;
	GS8604MIPI_sensor.pv_mode=KAL_FALSE;
	GS8604MIPI_sensor.capture_mode=KAL_FALSE;
	spin_unlock(&GS8604_drv_lock);
	VideoFullSizeSetting();
	iStartX += GS8604MIPI_IMAGE_SENSOR_VIDEO_STARTX;
	iStartY += GS8604MIPI_IMAGE_SENSOR_VIDEO_STARTY;
	spin_lock(&GS8604_drv_lock);
	GS8604MIPI_sensor.cp_dummy_pixels = 0;
	GS8604MIPI_sensor.cp_dummy_lines = 0;
	GS8604MIPI_sensor.pv_dummy_pixels = 0;
	GS8604MIPI_sensor.pv_dummy_lines = 0;
	GS8604MIPI_sensor.video_dummy_pixels = 0;
	GS8604MIPI_sensor.video_dummy_lines = 0;
	GS8604MIPI_sensor.video_line_length = GS8604MIPI_VIDEO_LINE_LENGTH_PIXELS+GS8604MIPI_sensor.video_dummy_pixels;
	GS8604MIPI_sensor.video_frame_length = GS8604MIPI_VIDEO_FRAME_LENGTH_LINES+GS8604MIPI_sensor.video_dummy_lines;
	spin_unlock(&GS8604_drv_lock);

	GS8604MIPI_SetDummy(GS8604MIPI_sensor.video_dummy_pixels,GS8604MIPI_sensor.video_dummy_lines);
	GS8604MIPI_SetShutter(GS8604MIPI_sensor.video_shutter);
	spin_lock(&GS8604_drv_lock);
	memcpy(&GS8604MIPISensorConfigData, sensor_config_data, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
	spin_unlock(&GS8604_drv_lock);
	image_window->GrabStartX= iStartX;
	image_window->GrabStartY= iStartY;
    SENSORDB("[GS8604MIPI_2LANE]eXIT GS8604MIPIVideo function\n");
	return ERROR_NONE;
}	/* GS8604MIPIPreview() */

static UINT32 GS8604MIPICapture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                                                MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	kal_uint16 iStartX = 0, iStartY = 0;
	SENSORDB("[GS8604MIPI_2LANE]enter GS8604MIPICapture function\n");
	if (GS8604MIPI_sensor.pv_mode == KAL_TRUE||GS8604MIPI_sensor.video_mode == KAL_TRUE)
	{
		spin_lock(&GS8604_drv_lock);
		GS8604MIPI_sensor.video_mode=KAL_FALSE;
		GS8604MIPI_sensor.pv_mode=KAL_FALSE;
		GS8604MIPI_sensor.capture_mode=KAL_TRUE;
		GS8604MIPI_MPEG4_encode_mode = KAL_FALSE;
		//GS8604MIPI_Auto_Flicker_mode = KAL_FALSE;
		GS8604MIPI_sensor.cp_dummy_pixels = 0;
		GS8604MIPI_sensor.cp_dummy_lines = 0;
		spin_unlock(&GS8604_drv_lock);
		GS8604MIPI_set_8M();
		//GS8604MIPISetFlipMirror(sensor_config_data->SensorImageMirror);
		spin_lock(&GS8604_drv_lock);
		GS8604MIPI_sensor.cp_line_length=GS8604MIPI_FULL_LINE_LENGTH_PIXELS+GS8604MIPI_sensor.cp_dummy_pixels;
		GS8604MIPI_sensor.cp_frame_length=GS8604MIPI_FULL_FRAME_LENGTH_LINES+GS8604MIPI_sensor.cp_dummy_lines;
		spin_unlock(&GS8604_drv_lock);
		iStartX = GS8604MIPI_IMAGE_SENSOR_CAP_STARTX;
		iStartY = GS8604MIPI_IMAGE_SENSOR_CAP_STARTY;
		image_window->GrabStartX=iStartX;
		image_window->GrabStartY=iStartY;
		image_window->ExposureWindowWidth=GS8604MIPI_REAL_CAP_WIDTH ;
		image_window->ExposureWindowHeight=GS8604MIPI_REAL_CAP_HEIGHT;
		GS8604MIPI_SetDummy(GS8604MIPI_sensor.cp_dummy_pixels, GS8604MIPI_sensor.cp_dummy_lines);
		spin_lock(&GS8604_drv_lock);
		memcpy(&GS8604MIPISensorConfigData, sensor_config_data, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
		spin_unlock(&GS8604_drv_lock);
		if(test_pattern_flag)
		{
			GS8604MIPISetTestPatternMode(TRUE);
			test_pattern_flag=FALSE;
		}
	}
	SENSORDB("[GS8604MIPI_2LANE]exit GS8604MIPICapture function\n");
	return ERROR_NONE;
}	/* GS8604MIPICapture() */

UINT32 GS8604MIPIGetResolution_2lane(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *pSensorResolution)
{
    SENSORDB("[GS8604MIPI_2LANE]eXIT GS8604MIPIGetResolution function\n");
    pSensorResolution->SensorPreviewWidth	= GS8604MIPI_REAL_PV_WIDTH;
    pSensorResolution->SensorPreviewHeight	= GS8604MIPI_REAL_PV_HEIGHT;
    pSensorResolution->SensorFullWidth		= GS8604MIPI_REAL_CAP_WIDTH;
    pSensorResolution->SensorFullHeight		= GS8604MIPI_REAL_CAP_HEIGHT;
    pSensorResolution->SensorVideoWidth		= GS8604MIPI_REAL_VIDEO_WIDTH;
    pSensorResolution->SensorVideoHeight    = GS8604MIPI_REAL_VIDEO_HEIGHT;
    SENSORDB("GS8604MIPIGetResolution :8-14");
    return ERROR_NONE;
}   /* GS8604MIPIGetResolution() */

UINT32 GS8604MIPIGetInfo_2lane(MSDK_SCENARIO_ID_ENUM ScenarioId,
                                                MSDK_SENSOR_INFO_STRUCT *pSensorInfo,
                                                MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
	SENSORDB("[GS8604MIPI_2LANE]enter GS8604MIPIGetInfo function\n");
	switch(ScenarioId){
			case MSDK_SCENARIO_ID_CAMERA_ZSD:
			case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG://hhl 2-28
				pSensorInfo->SensorFullResolutionX=GS8604MIPI_REAL_CAP_WIDTH;
				pSensorInfo->SensorFullResolutionY=GS8604MIPI_REAL_CAP_HEIGHT;
				//pSensorInfo->SensorStillCaptureFrameRate=20;
				pSensorInfo->SensorStillCaptureFrameRate=GS8604MIPI_CAPTURE_MAX_FRAMERATE;

			break;//hhl 2-28
			case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
				pSensorInfo->SensorPreviewResolutionX=GS8604MIPI_REAL_VIDEO_WIDTH;
				pSensorInfo->SensorPreviewResolutionY=GS8604MIPI_REAL_VIDEO_HEIGHT;
				pSensorInfo->SensorCameraPreviewFrameRate=GS8604MIPI_PREVIEW_MAX_FRAMERATE;
			break;
		default:
        pSensorInfo->SensorPreviewResolutionX=GS8604MIPI_REAL_PV_WIDTH;
        pSensorInfo->SensorPreviewResolutionY=GS8604MIPI_REAL_PV_HEIGHT;
				pSensorInfo->SensorCameraPreviewFrameRate=GS8604MIPI_PREVIEW_MAX_FRAMERATE;
			break;
	}
    pSensorInfo->SensorVideoFrameRate=GS8604MIPI_VIDEO_MAX_FRAMERATE;
    //pSensorInfo->SensorStillCaptureFrameRate=24;
    pSensorInfo->SensorStillCaptureFrameRate=GS8604MIPI_CAPTURE_MAX_FRAMERATE;
    //pSensorInfo->SensorWebCamCaptureFrameRate=24;
    pSensorInfo->SensorWebCamCaptureFrameRate=GS8604MIPI_WEB_CAM_CAPTURE_MAX_FRAMERATE;
    pSensorInfo->SensorResetActiveHigh=FALSE;
    pSensorInfo->SensorResetDelayCount=5;

	if(m_mir_flag)
    	pSensorInfo->SensorOutputDataFormat=SENSOR_OUTPUT_FORMAT_RAW_B;
	else
		pSensorInfo->SensorOutputDataFormat=SENSOR_OUTPUT_FORMAT_RAW_R;
	
	pSensorInfo->SensorClockPolarity=SENSOR_CLOCK_POLARITY_LOW; /*??? */
    pSensorInfo->SensorClockFallingPolarity=SENSOR_CLOCK_POLARITY_LOW;
    pSensorInfo->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
    pSensorInfo->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
    pSensorInfo->SensorInterruptDelayLines = 1;
    pSensorInfo->SensroInterfaceType=SENSOR_INTERFACE_TYPE_MIPI;
    pSensorInfo->MIPIsensorType=MIPI_OPHY_CSI2;

    pSensorInfo->CaptureDelayFrame = 2;
    pSensorInfo->PreviewDelayFrame = 2;
    pSensorInfo->VideoDelayFrame = 2;
    pSensorInfo->SensorMasterClockSwitch = 0;
    pSensorInfo->SensorDrivingCurrent = ISP_DRIVING_8MA;
    pSensorInfo->AEShutDelayFrame = 0;//0		    /* The frame of setting shutter default 0 for TG int */
    pSensorInfo->AESensorGainDelayFrame = 1;  //0   /* The frame of setting sensor gain */
    pSensorInfo->AEISPGainDelayFrame = 2;

    switch (ScenarioId)
    {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
            pSensorInfo->SensorClockFreq=24;
            pSensorInfo->SensorClockDividCount=	5;
            pSensorInfo->SensorClockRisingCount= 0;
            pSensorInfo->SensorClockFallingCount= 2;
            pSensorInfo->SensorPixelClockCount= 3;
            pSensorInfo->SensorDataLatchCount= 2;
            pSensorInfo->SensorGrabStartX = GS8604MIPI_IMAGE_SENSOR_PV_STARTX;
            pSensorInfo->SensorGrabStartY = GS8604MIPI_IMAGE_SENSOR_PV_STARTY;
            pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_2_LANE;
            pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
	     	pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 32;//14;
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
			   pSensorInfo->SensorGrabStartX = GS8604MIPI_IMAGE_SENSOR_VIDEO_STARTX;
			   pSensorInfo->SensorGrabStartY = GS8604MIPI_IMAGE_SENSOR_VIDEO_STARTY;
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
			//pSensorInfo->SensorClockFreq=12;
            pSensorInfo->SensorClockDividCount=	5;
            pSensorInfo->SensorClockRisingCount= 0;
            pSensorInfo->SensorClockFallingCount= 2;
            pSensorInfo->SensorPixelClockCount= 3;
            pSensorInfo->SensorDataLatchCount= 2;
            pSensorInfo->SensorGrabStartX = GS8604MIPI_IMAGE_SENSOR_CAP_STARTX;	//2*GS8604MIPI_IMAGE_SENSOR_PV_STARTX;
            pSensorInfo->SensorGrabStartY = GS8604MIPI_IMAGE_SENSOR_CAP_STARTY;	//2*GS8604MIPI_IMAGE_SENSOR_PV_STARTY;
            pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_2_LANE;
            pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
            pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 32;//14;
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
			 pSensorInfo->SensorGrabStartX = GS8604MIPI_IMAGE_SENSOR_PV_STARTX;
			 pSensorInfo->SensorGrabStartY = GS8604MIPI_IMAGE_SENSOR_PV_STARTY;
			 pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_2_LANE;
			 pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
		     pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 32;//14;
		  	 pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
			 pSensorInfo->SensorWidthSampling = 0;	// 0 is default 1x
			 pSensorInfo->SensorHightSampling = 0;	 // 0 is default 1x
			 pSensorInfo->SensorPacketECCOrder = 1;

            break;
    }
	spin_lock(&GS8604_drv_lock);
    GS8604MIPIPixelClockDivider=pSensorInfo->SensorPixelClockCount;
    memcpy(pSensorConfigData, &GS8604MIPISensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
	spin_unlock(&GS8604_drv_lock);
    SENSORDB("[GS8604MIPI_2LANE]exit GS8604MIPIGetInfo function\n");
    return ERROR_NONE;
}   /* GS8604MIPIGetInfo() */


UINT32 GS8604MIPIControl_2lane(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *pImageWindow,
                                                MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
		spin_lock(&GS8604_drv_lock);
		CurrentScenarioId = ScenarioId;
		spin_unlock(&GS8604_drv_lock);
		SENSORDB("[GS8604MIPI_2LANE]enter GS8604MIPIControl function\n");
    switch (ScenarioId)
    {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
            GS8604MIPIPreview(pImageWindow, pSensorConfigData);
            break;
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			GS8604MIPIVideo(pImageWindow, pSensorConfigData);
            break;
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
	    case MSDK_SCENARIO_ID_CAMERA_ZSD:
            GS8604MIPICapture(pImageWindow, pSensorConfigData);//hhl 2-28
            break;
        default:
            return ERROR_INVALID_SCENARIO_ID;
    }
	SENSORDB("[GS8604MIPI_2LANE]exit GS8604MIPIControl function\n");
    return ERROR_NONE;
} /* GS8604MIPIControl() */


//for auto-flicker fps
//int flag_framerate = 0;
static UINT32 GS8604MIPISetVideoMode(UINT16 u2FrameRate)
{
    SENSORDB("[GS8604MIPISetVideoMode] frame rate = %d\n", u2FrameRate);
	kal_uint16 GS8604MIPI_Video_Max_Expourse_Time = 0;
	SENSORDB("[GS8604MIPI_2LANE]%s():fix_frame_rate=%d\n",__FUNCTION__,u2FrameRate);
	spin_lock(&GS8604_drv_lock);
	GS8604MIPI_sensor.fix_video_fps = KAL_TRUE;
	spin_unlock(&GS8604_drv_lock);
	GS8604VIDEO_MODE_TARGET_FPS = u2FrameRate;
	u2FrameRate=u2FrameRate*10;//10*FPS
	SENSORDB("[GS8604MIPI_2LANE][Enter Fix_fps func] GS8604MIPI_Fix_Video_Frame_Rate = %d\n", u2FrameRate/10);
	GS8604MIPI_Video_Max_Expourse_Time = (kal_uint16)((GS8604MIPI_sensor.video_pclk*10/u2FrameRate)/GS8604MIPI_sensor.video_line_length);

	if (GS8604MIPI_Video_Max_Expourse_Time > GS8604MIPI_VIDEO_FRAME_LENGTH_LINES/*GS8604MIPI_sensor.pv_frame_length*/)
	{
		spin_lock(&GS8604_drv_lock);
		GS8604MIPI_sensor.video_frame_length = GS8604MIPI_Video_Max_Expourse_Time;
		GS8604MIPI_sensor.video_dummy_lines = GS8604MIPI_sensor.video_frame_length-GS8604MIPI_VIDEO_FRAME_LENGTH_LINES;
		spin_unlock(&GS8604_drv_lock);
		SENSORDB("[GS8604MIPI_2LANE]%s():frame_length=%d,dummy_lines=%d\n",__FUNCTION__,GS8604MIPI_sensor.video_frame_length,GS8604MIPI_sensor.video_dummy_lines);
		GS8604MIPI_SetDummy(GS8604MIPI_sensor.video_dummy_pixels,GS8604MIPI_sensor.video_dummy_lines);
	}
	spin_lock(&GS8604_drv_lock);
	GS8604MIPI_MPEG4_encode_mode = KAL_TRUE;
	spin_unlock(&GS8604_drv_lock);
	SENSORDB("[GS8604MIPI_2LANE]exit GS8604MIPISetVideoMode function\n");
	return ERROR_NONE;
}

static UINT32 GS8604MIPISetAutoFlickerMode(kal_bool bEnable, UINT16 u2FrameRate)
{

    return ERROR_NONE;
}
static UINT32 GS8604MIPISetMaxFramerateByScenario(MSDK_SCENARIO_ID_ENUM scenarioId, MUINT32 frameRate) {
	kal_uint32 pclk;
	kal_int16 dummyLine;
	kal_uint16 lineLength,frameHeight;
	SENSORDB("GS8604MIPISetMaxFramerateByScenario: scenarioId = %d, frame rate = %d\n",scenarioId,frameRate);
	switch (scenarioId) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			pclk = GS8604MIPI_PREVIEW_CLK;
			lineLength = GS8604MIPI_PV_LINE_LENGTH_PIXELS;
			frameHeight = (10 * pclk)/frameRate/lineLength;
			dummyLine = frameHeight - GS8604MIPI_PV_FRAME_LENGTH_LINES;


			if(dummyLine<0)
				dummyLine = 0;
			spin_lock(&GS8604_drv_lock);
			GS8604MIPI_sensor.pv_mode=TRUE;
			spin_unlock(&GS8604_drv_lock);
			GS8604MIPI_SetDummy(0, dummyLine);
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			pclk = GS8604MIPI_VIDEO_CLK;
			lineLength = GS8604MIPI_VIDEO_LINE_LENGTH_PIXELS;
			frameHeight = (10 * pclk)/frameRate/lineLength;
			dummyLine = frameHeight - GS8604MIPI_VIDEO_FRAME_LENGTH_LINES;
			if(dummyLine<0)
				dummyLine = 0;
			spin_lock(&GS8604_drv_lock);
			GS8604MIPI_sensor.pv_mode=TRUE;
			spin_unlock(&GS8604_drv_lock);
			GS8604MIPI_SetDummy(0, dummyLine);
			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
			pclk = GS8604MIPI_CAPTURE_CLK;
			lineLength = GS8604MIPI_FULL_LINE_LENGTH_PIXELS;
			frameHeight = (10 * pclk)/frameRate/lineLength;
			dummyLine = frameHeight - GS8604MIPI_FULL_FRAME_LENGTH_LINES;
			if(dummyLine<0)
				dummyLine = 0;

			spin_lock(&GS8604_drv_lock);
			GS8604MIPI_sensor.pv_mode=FALSE;
			spin_unlock(&GS8604_drv_lock);
			GS8604MIPI_SetDummy(0, dummyLine);
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
	SENSORDB("[GS8604MIPI_2LANE]exit GS8604MIPISetMaxFramerateByScenario function\n");
	return ERROR_NONE;
}
static UINT32 GS8604MIPIGetDefaultFramerateByScenario(MSDK_SCENARIO_ID_ENUM scenarioId, MUINT32 *pframeRate)
{

	switch (scenarioId) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			 *pframeRate = GS8604MIPI_PREVIEW_MAX_FRAMERATE * 10;
			 break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
			 *pframeRate = GS8604MIPI_CAPTURE_MAX_FRAMERATE * 10;
			break;		//hhl 2-28
        case MSDK_SCENARIO_ID_CAMERA_3D_PREVIEW: //added
        case MSDK_SCENARIO_ID_CAMERA_3D_VIDEO:
        case MSDK_SCENARIO_ID_CAMERA_3D_CAPTURE: //added
			 *pframeRate = GS8604MIPI_3D_PREVIEW_MAX_FRAMERATE * 10;
			break;
		default:
			break;
	}

	return ERROR_NONE;
}
UINT32 GS8604MIPIFeatureControl_2lane(MSDK_SENSOR_FEATURE_ENUM FeatureId,
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
            *pFeatureReturnPara16++=GS8604MIPI_REAL_CAP_WIDTH;
            *pFeatureReturnPara16=GS8604MIPI_REAL_CAP_HEIGHT;
            *pFeatureParaLen=4;
            break;
        case SENSOR_FEATURE_GET_PERIOD:
        		switch(CurrentScenarioId)
        		{
        			case MSDK_SCENARIO_ID_CAMERA_ZSD:
        		    case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
 		            *pFeatureReturnPara16++=GS8604MIPI_sensor.cp_line_length;
 		            *pFeatureReturnPara16=GS8604MIPI_sensor.cp_frame_length;
		            SENSORDB("Sensor period:%d %d\n",GS8604MIPI_sensor.cp_line_length, GS8604MIPI_sensor.cp_frame_length);
		            *pFeatureParaLen=4;
        				break;
        			case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
					*pFeatureReturnPara16++=GS8604MIPI_sensor.video_line_length;
					*pFeatureReturnPara16=GS8604MIPI_sensor.video_frame_length;
					 SENSORDB("Sensor period:%d %d\n", GS8604MIPI_sensor.video_line_length, GS8604MIPI_sensor.video_frame_length);
					 *pFeatureParaLen=4;
						break;
        			default:
					*pFeatureReturnPara16++=GS8604MIPI_sensor.pv_line_length;
					*pFeatureReturnPara16=GS8604MIPI_sensor.pv_frame_length;
		            SENSORDB("Sensor period:%d %d\n", GS8604MIPI_sensor.pv_line_length, GS8604MIPI_sensor.pv_frame_length);
		            *pFeatureParaLen=4;
	            break;
          	}
          	break;
        case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
        		switch(CurrentScenarioId)
        		{
        			case MSDK_SCENARIO_ID_CAMERA_ZSD:
        			case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		            *pFeatureReturnPara32 = GS8604MIPI_sensor.cp_pclk;
		            *pFeatureParaLen=4;

		            SENSORDB("Sensor CPCLK:%dn",GS8604MIPI_sensor.cp_pclk);
		         		break; //hhl 2-28
					case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
						*pFeatureReturnPara32 = GS8604MIPI_sensor.video_pclk;
						*pFeatureParaLen=4;
						SENSORDB("Sensor videoCLK:%d\n",GS8604MIPI_sensor.video_pclk);
						break;
		         		default:
		            *pFeatureReturnPara32 = GS8604MIPI_sensor.pv_pclk;
		            *pFeatureParaLen=4;
					SENSORDB("Sensor pvclk:%d\n",GS8604MIPI_sensor.pv_pclk);
		            break;
		         }
		         break;
        case SENSOR_FEATURE_SET_ESHUTTER:
            GS8604MIPI_SetShutter(*pFeatureData16);
            break;
		case SENSOR_FEATURE_SET_SENSOR_SYNC:
			break;
        case SENSOR_FEATURE_SET_NIGHTMODE:
            GS8604MIPI_NightMode((BOOL) *pFeatureData16);
            break;
        case SENSOR_FEATURE_SET_GAIN:
           GS8604MIPI_SetGain((UINT16) *pFeatureData16);
            break;
        case SENSOR_FEATURE_SET_FLASHLIGHT:
            break;
        case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
			spin_lock(&GS8604_drv_lock);
            GS8604MIPI_isp_master_clock=*pFeatureData32;
			spin_unlock(&GS8604_drv_lock);
            break;
        case SENSOR_FEATURE_SET_REGISTER:
			//GS8604MIPI_REAL_CAP_WIDTH =				pSensorRegData->RegAddr;//3280
            //GS8604MIPI_REAL_CAP_HEIGHT	=			pSensorRegData->RegData;//2464
            //SENSORDB("xb.pang---w:%d, h:%d\n",GS8604MIPI_REAL_CAP_WIDTH, GS8604MIPI_REAL_CAP_HEIGHT);
            printk("[gs8604_mipiraw_2lane_sensor.c][GS8604MIPIFeatureControl_2lane]SENSOR_FEATURE_SET_REGISTER:pSensorRegData->RegAddr = 0x%4x, pSensorRegData->RegData = 0x%.4x\r\n",
            	pSensorRegData->RegAddr, pSensorRegData->RegData);
			GS8604MIPI_write_cmos_sensor(pSensorRegData->RegAddr, pSensorRegData->RegData);

			break;
        case SENSOR_FEATURE_GET_REGISTER:
            pSensorRegData->RegData = GS8604MIPI_read_cmos_sensor(pSensorRegData->RegAddr);
			printk("[gs8604_mipiraw_2lane_sensor.c][GS8604MIPIFeatureControl_2lane]SENSOR_FEATURE_GET_REGISTER:pSensorRegData->RegAddr = 0x%4x, pSensorRegData->RegData = 0x%.4x\r\n",
            	pSensorRegData->RegAddr, pSensorRegData->RegData);
            break;
        case SENSOR_FEATURE_SET_CCT_REGISTER:
            SensorRegNumber=FACTORY_END_ADDR;
            for (i=0;i<SensorRegNumber;i++)
            {
            	spin_lock(&GS8604_drv_lock);
                GS8604MIPISensorCCT[i].Addr=*pFeatureData32++;
                GS8604MIPISensorCCT[i].Para=*pFeatureData32++;
				spin_unlock(&GS8604_drv_lock);
            }
            break;
        case SENSOR_FEATURE_GET_CCT_REGISTER:
            SensorRegNumber=FACTORY_END_ADDR;
            if (*pFeatureParaLen<(SensorRegNumber*sizeof(SENSOR_REG_STRUCT)+4))
                return FALSE;
            *pFeatureData32++=SensorRegNumber;
            for (i=0;i<SensorRegNumber;i++)
            {
                *pFeatureData32++=GS8604MIPISensorCCT[i].Addr;
                *pFeatureData32++=GS8604MIPISensorCCT[i].Para;
            }
            break;
        case SENSOR_FEATURE_SET_ENG_REGISTER:
            SensorRegNumber=ENGINEER_END;
            for (i=0;i<SensorRegNumber;i++)
            {	spin_lock(&GS8604_drv_lock);
                GS8604MIPISensorReg[i].Addr=*pFeatureData32++;
                GS8604MIPISensorReg[i].Para=*pFeatureData32++;
				spin_unlock(&GS8604_drv_lock);
            }
            break;
        case SENSOR_FEATURE_GET_ENG_REGISTER:
            SensorRegNumber=ENGINEER_END;
            if (*pFeatureParaLen<(SensorRegNumber*sizeof(SENSOR_REG_STRUCT)+4))
                return FALSE;
            *pFeatureData32++=SensorRegNumber;
            for (i=0;i<SensorRegNumber;i++)
            {
                *pFeatureData32++=GS8604MIPISensorReg[i].Addr;
                *pFeatureData32++=GS8604MIPISensorReg[i].Para;
            }
            break;
        case SENSOR_FEATURE_GET_REGISTER_DEFAULT:
            if (*pFeatureParaLen>=sizeof(NVRAM_SENSOR_DATA_STRUCT))
            {
                pSensorDefaultData->Version=NVRAM_CAMERA_SENSOR_FILE_VERSION;
                pSensorDefaultData->SensorId=GS8604MIPI_SENSOR_ID;
                memcpy(pSensorDefaultData->SensorEngReg, GS8604MIPISensorReg, sizeof(SENSOR_REG_STRUCT)*ENGINEER_END);
                memcpy(pSensorDefaultData->SensorCCTReg, GS8604MIPISensorCCT, sizeof(SENSOR_REG_STRUCT)*FACTORY_END_ADDR);
            }
            else
                return FALSE;
            *pFeatureParaLen=sizeof(NVRAM_SENSOR_DATA_STRUCT);
            break;
        case SENSOR_FEATURE_GET_CONFIG_PARA:
            memcpy(pSensorConfigData, &GS8604MIPISensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
            *pFeatureParaLen=sizeof(MSDK_SENSOR_CONFIG_STRUCT);
            break;
        case SENSOR_FEATURE_CAMERA_PARA_TO_SENSOR:
            GS8604MIPI_camera_para_to_sensor();
            break;

        case SENSOR_FEATURE_SENSOR_TO_CAMERA_PARA:
            GS8604MIPI_sensor_to_camera_para();
            break;
        case SENSOR_FEATURE_GET_GROUP_COUNT:
            *pFeatureReturnPara32++=GS8604MIPI_get_sensor_group_count();
            *pFeatureParaLen=4;
            break;
        case SENSOR_FEATURE_GET_GROUP_INFO:
            GS8604MIPI_get_sensor_group_info(pSensorGroupInfo->GroupIdx, pSensorGroupInfo->GroupNamePtr, &pSensorGroupInfo->ItemCount);
            *pFeatureParaLen=sizeof(MSDK_SENSOR_GROUP_INFO_STRUCT);
            break;
        case SENSOR_FEATURE_GET_ITEM_INFO:
            GS8604MIPI_get_sensor_item_info(pSensorItemInfo->GroupIdx,pSensorItemInfo->ItemIdx, pSensorItemInfo);
            *pFeatureParaLen=sizeof(MSDK_SENSOR_ITEM_INFO_STRUCT);
            break;

        case SENSOR_FEATURE_SET_ITEM_INFO:
            GS8604MIPI_set_sensor_item_info(pSensorItemInfo->GroupIdx, pSensorItemInfo->ItemIdx, pSensorItemInfo->ItemValue);
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
            GS8604MIPISetVideoMode(*pFeatureData16);
            break;
        case SENSOR_FEATURE_CHECK_SENSOR_ID:
            GS8604MIPIGetSensorID(pFeatureReturnPara32);
            break;
        case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE:
            GS8604MIPISetAutoFlickerMode((BOOL)*pFeatureData16, *(pFeatureData16+1));
	        break;
        case SENSOR_FEATURE_SET_TEST_PATTERN:
            GS8604MIPISetTestPatternMode((BOOL)*pFeatureData16);
            break;
		case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE:
			*pFeatureReturnPara32 = GS8604_TEST_PATTERN_CHECKSUM;
			*pFeatureParaLen=4;
			break;
		case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
			GS8604MIPISetMaxFramerateByScenario((MSDK_SCENARIO_ID_ENUM)*pFeatureData32, *(pFeatureData32+1));
			break;
		case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
			GS8604MIPIGetDefaultFramerateByScenario((MSDK_SCENARIO_ID_ENUM)*pFeatureData32, (MUINT32 *)(*(pFeatureData32+1)));
			break;
        default:
            break;
    }
    return ERROR_NONE;
}	/* GS8604MIPIFeatureControl() */


SENSOR_FUNCTION_STRUCT	SensorFuncGS8604MIPI_2lane=
{
    GS8604MIPIOpen_2lane,
    GS8604MIPIGetInfo_2lane,
    GS8604MIPIGetResolution_2lane,
    GS8604MIPIFeatureControl_2lane,
    GS8604MIPIControl_2lane,
    GS8604MIPIClose_2lane
};

UINT32 GS8604_MIPI_RAW_2lane_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
    /* To Do : Check Sensor status here */
    if (pfFunc!=NULL)
        *pfFunc=&SensorFuncGS8604MIPI_2lane;
    return ERROR_NONE;
}   /* SensorInit() */
