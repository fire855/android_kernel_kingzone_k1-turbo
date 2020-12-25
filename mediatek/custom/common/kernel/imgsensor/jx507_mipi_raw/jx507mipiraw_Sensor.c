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
#include "cust_gpio_usage.h"

#include "jx507mipiraw_Sensor.h"
#include "jx507mipiraw_Camera_Sensor_para.h"
#include "jx507mipiraw_CameraCustomized.h"


#define		_SW_VER						"V1.28.00.00"
#define		_SW_CODE					"20140822r1"
#define		_SW_HINT					"JX507_0822r1"


#define		_ADJUST_FRAMERATE			//min@20131223


///#define	_SENSOR_INITIAL_FLIP		//min@20131224
#define		_VAVG_ON					//min@20140301 //min@20140227 comment out	//min@20140212
#define		_DRIVE_OPT					//min@20140331
#define		_VIDEO_FAST_INIT			//min@20140409
//#define	_BLC_8						//min@20140812

//#define	JX507MIPI_DEBUG


#ifdef JX507MIPI_DEBUG
	#define	PRINT_K(fmt, arg...)		xlog_printk(ANDROID_LOG_DEBUG, "[507]", fmt, ##arg)
#else
	#define	PRINT_K(x,...)
#endif
///#define	PRINT_K						printk

#ifdef JX507MIPI_DEBUG
	#define	SENSORDB(fmt, arg...)		xlog_printk(ANDROID_LOG_DEBUG, "[JX507MIPI]", fmt, ##arg)
#else
	#define	SENSORDB(x,...)
#endif

#ifdef JX507MIPI_DEBUG_SOFIA
	#define	SENSORDBSOFIA(fmt, arg...)	xlog_printk(ANDROID_LOG_DEBUG, "[JX507MIPI]", fmt, ##arg)
#else
	#define	SENSORDBSOFIA(x,...)
#endif

#define		mDELAY(ms)		mdelay(ms)
//#define	ACDK

static DEFINE_SPINLOCK(jx507mipi_drv_lock);

kal_uint32	JX507MIPI_FeatureControl_PERIOD_PixelNum	= JX507MIPI_PV_PERIOD_PIXEL_NUMS;
kal_uint32	JX507MIPI_FeatureControl_PERIOD_LineNum		= JX507MIPI_PV_PERIOD_LINE_NUMS;

static UINT16	VIDEO_MODE_TARGET_FPS	= 30;				//min@20131223 static
static BOOL		ReEnteyCamera			= KAL_FALSE;


MSDK_SENSOR_CONFIG_STRUCT	JX507SensorConfigData;

kal_uint32	JX507MIPI_FAC_SENSOR_REG;

MSDK_SCENARIO_ID_ENUM	JX507MIPICurrentScenarioId		= MSDK_SCENARIO_ID_CAMERA_PREVIEW;
static kal_bool			JX507MIPIAutoFlicKerMode	= KAL_FALSE;

/* FIXME: old factors and DIDNOT use now. s*/
SENSOR_REG_STRUCT	JX507SensorCCT[]						= JX507MIPI_CAMERA_SENSOR_CCT_DEFAULT_VALUE;
SENSOR_REG_STRUCT	JX507SensorReg[JX507MIPI_ENGINEER_END]	= JX507MIPI_CAMERA_SENSOR_REG_DEFAULT_VALUE;
/* FIXME: old factors and DIDNOT use now. e*/

static JX507MIPI_PARA_STRUCT	jx507;

#ifdef _DUAL_I2C_ID
static volatile UINT32	gSensor_WRITE_ID	= JX507MIPI_WRITE_ID;	//min@20140630
#endif

//min@20140805
#ifdef _SENSOR_INITIAL_FLIP
static UINT8	gMirrorFlip = 0x03;
#else
static UINT8	gMirrorFlip = 0x00;
#endif

////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////

extern int iReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 * a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
extern int iWriteRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u16 i2cId);

kal_uint16 JX507MIPI_write_cmos_sensor(kal_uint32 addr, kal_uint32 para)
{
	char puSendCmd[3] = {0};
	
	puSendCmd[0] = (char)(addr & 0xFF);
	puSendCmd[1] = (char)(para & 0xFF);
	puSendCmd[2] = 0x00;
	#ifdef _DUAL_I2C_ID
	iWriteRegI2C( (u8 *)puSendCmd, 2, gSensor_WRITE_ID );	//min@20140630
	#else
	iWriteRegI2C( (u8 *)puSendCmd, 2, JX507MIPI_WRITE_ID );
	#endif
	return TRUE;
}

kal_uint16 JX507MIPI_read_cmos_sensor(kal_uint32 addr)
{
	u8		get_byte = 0;
	char	puSendCmd[2] = {0};
	
	puSendCmd[0] = (char)(addr & 0xFF);
	puSendCmd[1] = 0x00;
	#ifdef _DUAL_I2C_ID
	iReadRegI2C( (u8 *)puSendCmd, 1, (u8 *)&get_byte, 1, gSensor_WRITE_ID );	//min@20140630
	#else
	iReadRegI2C( (u8 *)puSendCmd, 1, (u8 *)&get_byte, 1, JX507MIPI_WRITE_ID );
	#endif
	return (kal_uint16) get_byte;
}

#define		Sleep(ms)	mdelay(ms)

////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////

static void JX507MIPI_Write_Shutter(kal_uint32 iShutter)
{
	kal_uint32	min_framelength	= JX507MIPI_PV_PERIOD_PIXEL_NUMS;
	kal_uint32	max_shutter		= 0;
	kal_uint32	extra_lines		= 0;
	kal_uint32	line_length		= 0;
	kal_uint32	frame_length	= 0;
	unsigned long	flags;
	
	PRINT_K( ">> JX507MIPI_Write_Shutter(): iShutter=0x%X, %d\n", iShutter, iShutter );
	SENSORDBSOFIA( "!!iShutter=%d!!!!!\n", iShutter );
	
	if( jx507.AutoFlickerMode == KAL_TRUE )
	{
		if( SENSOR_MODE_PREVIEW == jx507.sensorMode ) { //(g_iJX507MIPI_Mode == JX507MIPI_MODE_PREVIEW)	//SXGA size output
			PRINT_K( "@PREVIEW_AUTO_FLICKER\n" );
			line_length = JX507MIPI_PV_PERIOD_PIXEL_NUMS;
			max_shutter = JX507MIPI_PV_PERIOD_LINE_NUMS  + jx507.DummyLines;
		}
		else if( SENSOR_MODE_VIDEO == jx507.sensorMode ) {
			PRINT_K( "@VIDEO_AUTO_FLICKER\n" );
			line_length = JX507MIPI_VIDEO_PERIOD_PIXEL_NUMS;
			max_shutter = JX507MIPI_VIDEO_PERIOD_LINE_NUMS  + jx507.DummyLines;
		}
		else {
			PRINT_K( "@CAPTURE_AUTO_FLICKER\n" );
			line_length = JX507MIPI_FULL_PERIOD_PIXEL_NUMS;
			max_shutter = JX507MIPI_FULL_PERIOD_LINE_NUMS  + jx507.DummyLines;
		}
		
		switch( JX507MIPICurrentScenarioId )
		{
			case MSDK_SCENARIO_ID_CAMERA_ZSD:
			case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
				SENSORDBSOFIA( "AutoFlickerMode!!! MSDK_SCENARIO_ID_CAMERA_ZSD  0!!\n" );
			#if defined(ZSD15FPS)
				min_framelength = (jx507.capPclk*10000) /(JX507MIPI_FULL_PERIOD_PIXEL_NUMS + jx507.DummyPixels)/148;	//min@20140408
			#else
				min_framelength = (jx507.capPclk*10000) /(JX507MIPI_FULL_PERIOD_PIXEL_NUMS + jx507.DummyPixels)/130;	//min@20140408
			#endif
				break;
			case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
				if( VIDEO_MODE_TARGET_FPS==30 ) {
					min_framelength = (jx507.videoPclk*10000) /(JX507MIPI_VIDEO_PERIOD_PIXEL_NUMS + jx507.DummyPixels)/306;	//min@20140408
				}
				else if( VIDEO_MODE_TARGET_FPS==15 ) {
					min_framelength = (jx507.videoPclk*10000) /(JX507MIPI_VIDEO_PERIOD_PIXEL_NUMS + jx507.DummyPixels)/148;	//min@20140408
				}
				else {
					min_framelength = max_shutter;
				}
				break;
			default:
				min_framelength = (jx507.pvPclk*10000) /(JX507MIPI_PV_PERIOD_PIXEL_NUMS + jx507.DummyPixels)/296;	//min@20140408
				break;
		}
		
		SENSORDBSOFIA( "AutoFlickerMode!!! min_framelength for AutoFlickerMode = %d (0x%x)\n", min_framelength, min_framelength );
		SENSORDBSOFIA( "max framerate(10 base) autofilker = %d\n", (jx507.pvPclk*10000)*10 /line_length/min_framelength );
		
		if( iShutter < 3 )
			iShutter = 3;
		
		if( iShutter > max_shutter-4 )
			extra_lines = iShutter - max_shutter + 4;
		else
			extra_lines = 0;
		
		if( SENSOR_MODE_PREVIEW == jx507.sensorMode ) {
			frame_length = JX507MIPI_PV_PERIOD_LINE_NUMS + jx507.DummyLines + extra_lines;
		}
		else if( SENSOR_MODE_VIDEO == jx507.sensorMode ) {
			frame_length = JX507MIPI_VIDEO_PERIOD_LINE_NUMS + jx507.DummyLines + extra_lines;
		}
		else {
			frame_length = JX507MIPI_FULL_PERIOD_LINE_NUMS + jx507.DummyLines + extra_lines;
		}
		SENSORDBSOFIA( "frame_length 0= %d\n", frame_length );
		
		if( frame_length < min_framelength )
		{
			//iShutter = min_framelength - 4;
			switch( JX507MIPICurrentScenarioId )
			{
				case MSDK_SCENARIO_ID_CAMERA_ZSD:
				case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
					extra_lines = min_framelength - (JX507MIPI_FULL_PERIOD_LINE_NUMS  + jx507.DummyLines);
					break;
				case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
					extra_lines = min_framelength - (JX507MIPI_VIDEO_PERIOD_LINE_NUMS + jx507.DummyLines);
					break;
				default:
					extra_lines = min_framelength - (JX507MIPI_PV_PERIOD_LINE_NUMS    + jx507.DummyLines);
					break;
			}
			frame_length = min_framelength;
		}
		
		SENSORDBSOFIA( "frame_length 1= %d\n", frame_length );
		
		ASSERT( line_length  < JX507MIPI_MAX_LINE_LENGTH  );	//0xCCCC
		ASSERT( frame_length < JX507MIPI_MAX_FRAME_LENGTH ); 	//0xFFFF
		
		#ifdef _ADJUST_FRAMERATE //min@20131223
		// Set total frame length.
		JX507MIPI_write_cmos_sensor( 0x23, (frame_length >> 8) & 0xff );
		JX507MIPI_write_cmos_sensor( 0x22, (frame_length     ) & 0xff );
		PRINT_K( "[JX507 I2C] FrameH=0x%X (%d)\n", frame_length, frame_length );
		#endif
		
		spin_lock_irqsave( &jx507mipi_drv_lock, flags );
		jx507.maxExposureLines = frame_length;
		JX507MIPI_FeatureControl_PERIOD_PixelNum = line_length;
		JX507MIPI_FeatureControl_PERIOD_LineNum	 = frame_length;
		spin_unlock_irqrestore( &jx507mipi_drv_lock, flags );
		
		// Set shutter. (Coarse integration time, uint: lines.)
		JX507MIPI_write_cmos_sensor( 0x02, (iShutter >> 8) & 0xff );
		JX507MIPI_write_cmos_sensor( 0x01, (iShutter     ) & 0xff );
		PRINT_K( "[JX507 I2C] Exp=0x%X (%d)\n", iShutter, iShutter );
		SENSORDBSOFIA( "frame_length 2= %d\n", frame_length );
		//SENSORDB( "framerate(10 base) = %d\n", (jx507.pvPclk*10000)*10 /line_length/frame_length );
		SENSORDB( "iShutter=%d, extra_lines=%d, line_length=%d, frame_length=%d\n", iShutter, extra_lines, line_length, frame_length );
	}
	else //AUTO FLICKER OFF
	{
		if( SENSOR_MODE_PREVIEW == jx507.sensorMode ) { //(g_iJX507MIPI_Mode == JX507MIPI_MODE_PREVIEW)	//SXGA size output
			PRINT_K( "@PREVIEW_FLICKER_OFF\n" );
			max_shutter = JX507MIPI_PV_PERIOD_LINE_NUMS + jx507.DummyLines;
		}
		else if( SENSOR_MODE_VIDEO == jx507.sensorMode ) { //add for video_6M setting
			PRINT_K( "@VIDEO_FLICKER_OFF\n" );
			max_shutter = JX507MIPI_VIDEO_PERIOD_LINE_NUMS + jx507.DummyLines;
		}
		else {
			PRINT_K( "@CAPTURE_FLICKER_OFF\n" );
			max_shutter = JX507MIPI_FULL_PERIOD_LINE_NUMS + jx507.DummyLines;
		}
		
		if( iShutter < 3 )
			iShutter = 3;
		
		if( iShutter > max_shutter-4 )
			extra_lines = iShutter - max_shutter + 4;
		else
			extra_lines = 0;
		
		if( SENSOR_MODE_PREVIEW == jx507.sensorMode ) { //SXGA size output
			line_length	 = JX507MIPI_PV_PERIOD_PIXEL_NUMS;
			frame_length = JX507MIPI_PV_PERIOD_LINE_NUMS  + jx507.DummyLines + extra_lines;
		}
		else if( SENSOR_MODE_VIDEO == jx507.sensorMode ) {
			line_length	 = JX507MIPI_VIDEO_PERIOD_PIXEL_NUMS;
			frame_length = JX507MIPI_VIDEO_PERIOD_LINE_NUMS  + jx507.DummyLines + extra_lines;
		}
		else {
			line_length	 = JX507MIPI_FULL_PERIOD_PIXEL_NUMS;
			frame_length = JX507MIPI_FULL_PERIOD_LINE_NUMS  + jx507.DummyLines + extra_lines;
		}
		PRINT_K( "AutoFlicker OFF: extra_lines=%d (0x%X)\n", extra_lines, extra_lines );
		PRINT_K( "AutoFlicker OFF: line_length=%d (0x%X)\n", line_length, line_length );
		PRINT_K( "AutoFlicker OFF: frame_length=%d (0x%X)\n", frame_length, frame_length );
		
		ASSERT( line_length  < JX507MIPI_MAX_LINE_LENGTH  );	//0xCCCC
		ASSERT( frame_length < JX507MIPI_MAX_FRAME_LENGTH ); 	//0xFFFF
		
		// Set total frame length
		#ifdef _ADJUST_FRAMERATE //min@20131223
		JX507MIPI_write_cmos_sensor( 0x23, (frame_length >> 8) & 0xff );
		JX507MIPI_write_cmos_sensor( 0x22, (frame_length     ) & 0xff );
		PRINT_K( "[JX507 I2C] FrameH=0x%X (%d)\n", frame_length, frame_length );
		#endif
		
		spin_lock_irqsave( &jx507mipi_drv_lock, flags );
		jx507.maxExposureLines = frame_length - 4;
		JX507MIPI_FeatureControl_PERIOD_PixelNum = line_length;
		JX507MIPI_FeatureControl_PERIOD_LineNum	 = frame_length;
		spin_unlock_irqrestore( &jx507mipi_drv_lock, flags );
		
		// Set shutter. (Coarse integration time, uint: lines.)
		JX507MIPI_write_cmos_sensor( 0x02, (iShutter >> 8) & 0xff );
		JX507MIPI_write_cmos_sensor( 0x01, (iShutter     ) & 0xff );
		PRINT_K( "[JX507 I2C] Exp=0x%X (%d)\n", iShutter, iShutter );
		
		//SENSORDB( "framerate(10 base) = %d\n", (jx507.pvPclk*10000)*10 /line_length/frame_length );
		SENSORDB( "iShutter=%d, extra_lines=%d, line_length=%d, frame_length=%d\n", iShutter, extra_lines, line_length, frame_length );
	}
	
	PRINT_K( "<< JX507MIPI_Write_Shutter()\n" );
}

static void JX507MIPI_Set_Dummy(const kal_uint32 iPixels, const kal_uint32 iLines)
{
#ifdef _ADJUST_FRAMERATE //min@20131223
	kal_uint32	line_length	 = 0;
	kal_uint32	frame_length = 0;
	
	PRINT_K( ">> JX507MIPI_Set_Dummy(): iPixels=%d, iLines=%d\n", iPixels, iLines );
	
	if( SENSOR_MODE_PREVIEW == jx507.sensorMode ) {	//SXGA size output
		PRINT_K( "@PREVIEW_Dummy\n" );
		line_length	 = JX507MIPI_PV_PERIOD_PIXEL_NUMS;
		frame_length = JX507MIPI_PV_PERIOD_LINE_NUMS  + iLines;
	}
	else if( SENSOR_MODE_VIDEO == jx507.sensorMode ) {
		PRINT_K( "@VIDEO_Dummy\n" );
		line_length	 = JX507MIPI_VIDEO_PERIOD_PIXEL_NUMS;
		frame_length = JX507MIPI_VIDEO_PERIOD_LINE_NUMS  + iLines;
	}
	else {
		PRINT_K( "@CAPTURE_Dummy\n" );
		line_length	 = JX507MIPI_FULL_PERIOD_PIXEL_NUMS;
		frame_length = JX507MIPI_FULL_PERIOD_LINE_NUMS  + iLines;
	}
	
	//if( jx507.maxExposureLines > frame_length - 4 )
	//	return;
	
	//ASSERT( line_length  < JX507MIPI_MAX_LINE_LENGTH  );	//0xCCCC
	//ASSERT( frame_length < JX507MIPI_MAX_FRAME_LENGTH );	//0xFFFF
	
	// Set total frame length
	#ifdef _ADJUST_FRAMERATE //min@20131223
		JX507MIPI_write_cmos_sensor( 0x23, (frame_length >> 8) & 0xff );
		JX507MIPI_write_cmos_sensor( 0x22, (frame_length     ) & 0xff );
		PRINT_K( "[JX507 I2C] FrameH=0x%X (%d)\n", frame_length, frame_length );
	///	jx507.frame_height = frame_length;					//min@20140324
	#endif
	
	spin_lock( &jx507mipi_drv_lock );
	jx507.maxExposureLines = frame_length - 4;
	JX507MIPI_FeatureControl_PERIOD_PixelNum = line_length;
	JX507MIPI_FeatureControl_PERIOD_LineNum	 = frame_length;
	spin_unlock( &jx507mipi_drv_lock );
	
	// Set total line length
	///	JX507MIPI_write_cmos_sensor( 0x21, (line_length >> 8) & 0xff );
	///	JX507MIPI_write_cmos_sensor( 0x20, (line_length     ) & 0xff );
	
	PRINT_K( "<< JX507MIPI_Set_Dummy()\n" );
#endif
}

/*************************************************************************
* FUNCTION
*   JX507MIPI_SetShutter
*
* DESCRIPTION
*   This function set e-shutter of JX507MIPI to change exposure time.
*
* PARAMETERS
*   iShutter : exposured lines
*
* RETURNS
*   None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static void JX507MIPI_SetShutter(kal_uint32 iShutter)
{
	PRINT_K( ">> JX507MIPI_SetShutter(): iShutter=0x%X, %d\n", iShutter, iShutter );
	
	if( MSDK_SCENARIO_ID_CAMERA_ZSD == JX507MIPICurrentScenarioId ) {
		//SENSORDB( "always UPDATE SHUTTER when jx507.sensorMode == SENSOR_MODE_CAPTURE\n" );
	}
	else {
		if( jx507.sensorMode == SENSOR_MODE_CAPTURE ) {
			//SENSORDB( "capture!!DONT UPDATE SHUTTER!!\n" );
			//return;
		}
	}
	
	if( jx507.shutter == iShutter )
		return;
	spin_lock( &jx507mipi_drv_lock );
	jx507.shutter = iShutter;
	spin_unlock( &jx507mipi_drv_lock );
	
	JX507MIPI_Write_Shutter( iShutter );
	
	PRINT_K( "<< JX507MIPI_SetShutter()\n" );
}

/*************************************************************************
* FUNCTION
*   JX507MIPI_read_shutter
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
UINT32 JX507MIPI_read_shutter(void)
{
	kal_uint16	temp_reg1, temp_reg2;
	UINT32		shutter = 0;
	
	temp_reg1 = JX507MIPI_read_cmos_sensor( 0x02 );
	temp_reg2 = JX507MIPI_read_cmos_sensor( 0x01 );
	shutter	  = (temp_reg1 << 8) | temp_reg2;
	return shutter;
}

/*******************************************************************************
*
********************************************************************************/
static kal_uint16 JX507MIPIReg2Gain(const kal_uint16 iReg)
{
	kal_uint8	iI;
	kal_uint16	iGain = jx507.ispBaseGain;		// 1x-gain base
	
	// Range: 1x to 32x
	// Gain = (GAIN[7] + 1) * (GAIN[6] + 1) * (GAIN[5] + 1) * (GAIN[4] + 1) * (1 + GAIN[3:0] / 16)
	for( iI = 8; iI >= 4; iI-- ) {
		iGain *= (((iReg >> iI) & 0x01) + 1);
	}
	SENSORDBSOFIA( "[JX507MIPIReg2Gain]real gain= %d\n", (iGain +  (iGain * (iReg & 0x0F)) / 16) );
	return iGain +  (iGain * (iReg & 0x0F)) / 16; //jx507.realGain
}

/*******************************************************************************
*
********************************************************************************/
static kal_uint16 JX507MIPIGain2Reg(const kal_uint16 Gain)
{
	kal_uint16 iReg	 = 0x0000;
	kal_uint16 iGain = Gain;
	
	if( iGain <  jx507.ispBaseGain ) {
		iReg =0;
	} else if( iGain < 2 * jx507.ispBaseGain ) {
		iReg = 16 * iGain / jx507.ispBaseGain - 16;
	} else if( iGain < 4 * jx507.ispBaseGain ) {
		iReg |= 0x10;
		iReg |= (8 *iGain / jx507.ispBaseGain - 16);
	} else if( iGain < 8 * jx507.ispBaseGain ) {
		iReg |= 0x30;
		iReg |= (4 * iGain / jx507.ispBaseGain - 16);
	} else if( iGain < 16 * jx507.ispBaseGain ) {
		iReg |= 0x70;
		iReg |= (2 * iGain /jx507.ispBaseGain - 16);
	} else if( iGain < 32 * jx507.ispBaseGain ) {
		iReg |= 0xF0;
		iReg |= (iGain /jx507.ispBaseGain - 16);
	} else if( iGain <= 62 * jx507.ispBaseGain ) {
		iReg |= 0x1F0;
		iReg |= (iGain /jx507.ispBaseGain/2 - 16);
	} else {
		SENSORDB( "out of range!\n" );
	}
	SENSORDBSOFIA( "[JX507MIPIGain2Reg]: isp gain:%d,sensor gain:0x%x\n", iGain, iReg );
	
	return iReg;						//jx507. sensorGlobalGain
}

void write_JX507MIPI_gain(kal_uint16 gain)
{
	JX507MIPI_write_cmos_sensor( 0x00, gain & 0xff );
	PRINT_K( "[JX507 I2C] Gain=0x%X (%d)\n", gain, gain );
}

/*************************************************************************
* FUNCTION
*	JX507MIPI_SetGain
*
* DESCRIPTION
*	This function is to set global gain to sensor.
*
* PARAMETERS
*	gain : sensor global gain(base: 0x40)
*
* RETURNS
*	the actually gain set to sensor.
*
* GLOBALS AFFECTED
*
*************************************************************************/
void JX507MIPI_SetGain(UINT16 iGain)
{
	unsigned long flags;
	
	PRINT_K( ">> JX507MIPI_SetGain(): iGain=0x%X\n", iGain );
	
	spin_lock_irqsave( &jx507mipi_drv_lock, flags );
	jx507.realGain = iGain;
	jx507.sensorGlobalGain = JX507MIPIGain2Reg( iGain );
	spin_unlock_irqrestore( &jx507mipi_drv_lock, flags );
	
	write_JX507MIPI_gain( jx507.sensorGlobalGain );
	SENSORDB( "jx507.sensorGlobalGain=0x%X, jx507.realGain=%d\n", jx507.sensorGlobalGain, jx507.realGain );
	
	PRINT_K( "<< JX507MIPI_SetGain()\n" );
}

/*************************************************************************
* FUNCTION
*	read_JX507_gain
*
* DESCRIPTION
*	This function is to set global gain to sensor.
*
* PARAMETERS
*	None
*
* RETURNS
*	gain : sensor global gain
*
* GLOBALS AFFECTED
*
*************************************************************************/
kal_uint16 read_JX507_gain(void)
{
	kal_uint16 read_gain = 0;
	
	read_gain = JX507MIPI_read_cmos_sensor( 0x00 );
	
	spin_lock( &jx507mipi_drv_lock );
	jx507.sensorGlobalGain = read_gain;
	jx507.realGain = JX507MIPIReg2Gain( jx507.sensorGlobalGain );
	spin_unlock( &jx507mipi_drv_lock );
	
	SENSORDB( "jx507.sensorGlobalGain=0x%X, jx507.realGain=%d\n", jx507.sensorGlobalGain, jx507.realGain );
	
	return jx507.sensorGlobalGain;
}

/*************************************************************************
* FUNCTION
*   JX507MIPI_NightMode
*
* DESCRIPTION
*   This function night mode of JX507.
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
void JX507MIPI_NightMode(kal_bool bEnable)
{
}

void JX507MIPI_camera_para_to_sensor(void)
{
//	kal_uint32	i;
//	
//	for( i = 0; 0xFFFFFFFF != JX507SensorReg[i].Addr; i++ )
//	{
//		JX507MIPI_write_cmos_sensor( JX507SensorReg[i].Addr, JX507SensorReg[i].Para );
//	}
//	for( i = JX507MIPI_ENGINEER_START_ADDR; 0xFFFFFFFF != JX507SensorReg[i].Addr; i++ )
//	{
//		JX507MIPI_write_cmos_sensor( JX507SensorReg[i].Addr, JX507SensorReg[i].Para );
//	}
//	for( i = JX507MIPI_FACTORY_START_ADDR; i < JX507MIPI_FACTORY_END_ADDR; i++ )
//	{
//		JX507MIPI_write_cmos_sensor( JX507SensorCCT[i].Addr, JX507SensorCCT[i].Para );
//	}
}

/*************************************************************************
* FUNCTION
*	JX507MIPI_sensor_to_camera_para
*
* DESCRIPTION
*	// update camera_para from sensor register
*
* PARAMETERS
*	None
*
* RETURNS
*	gain : sensor global gain(base: 0x40)
*
* GLOBALS AFFECTED
*
*************************************************************************/
void JX507MIPI_sensor_to_camera_para(void)
{
//	kal_uint32	i, temp_data;
//	
//	for( i = 0; 0xFFFFFFFF != JX507SensorReg[i].Addr; i++ )
//	{
//		temp_data = JX507MIPI_read_cmos_sensor( JX507SensorReg[i].Addr );
//		spin_lock( &jx507mipi_drv_lock );
//		JX507SensorReg[i].Para = temp_data;
//		spin_unlock( &jx507mipi_drv_lock );
//	}
//	for( i = JX507MIPI_ENGINEER_START_ADDR; 0xFFFFFFFF != JX507SensorReg[i].Addr; i++ )
//	{
//		temp_data = JX507MIPI_read_cmos_sensor( JX507SensorReg[i].Addr );
//		spin_lock( &jx507mipi_drv_lock );
//		JX507SensorReg[i].Para = temp_data;
//		spin_unlock( &jx507mipi_drv_lock );
//	}
}

/*************************************************************************
* FUNCTION
*	JX507MIPI_get_sensor_group_count
*
* DESCRIPTION
*	//
*
* PARAMETERS
*	None
*
* RETURNS
*	gain : sensor global gain(base: 0x40)
*
* GLOBALS AFFECTED
*
*************************************************************************/
kal_int32  JX507MIPI_get_sensor_group_count(void)
{
	return JX507MIPI_GROUP_TOTAL_NUMS;
}

void JX507MIPI_get_sensor_group_info(kal_uint16 group_idx, kal_int8* group_name_ptr, kal_int32* item_count_ptr)
{
	switch( group_idx )
	{
		case JX507MIPI_PRE_GAIN:
			sprintf( (char *)group_name_ptr, "CCT" );
			*item_count_ptr = 2;
			break;
		case JX507MIPI_CMMCLK_CURRENT:
			sprintf( (char *)group_name_ptr, "CMMCLK Current" );
			*item_count_ptr = 1;
			break;
		case JX507MIPI_FRAME_RATE_LIMITATION:
			sprintf( (char *)group_name_ptr, "Frame Rate Limitation" );
			*item_count_ptr = 2;
			break;
		case JX507MIPI_REGISTER_EDITOR:
			sprintf( (char *)group_name_ptr, "Register Editor" );
			*item_count_ptr = 2;
			break;
		default:
			ASSERT( 0 );
	}
}

void JX507MIPI_get_sensor_item_info(kal_uint16 group_idx,kal_uint16 item_idx, MSDK_SENSOR_ITEM_INFO_STRUCT* info_ptr)
{
	kal_int16	temp_reg=0;
	kal_uint16	temp_gain=0, temp_addr=0, temp_para=0;
	
	switch( group_idx )
	{
		case JX507MIPI_PRE_GAIN:
			switch( item_idx )
			{
				case 0:
					sprintf( (char *)info_ptr->ItemNamePtr, "Pregain-R" );
					temp_addr = JX507MIPI_PRE_GAIN_R_INDEX;
					break;
				case 1:
					sprintf( (char *)info_ptr->ItemNamePtr, "Pregain-Gr" );
					temp_addr = JX507MIPI_PRE_GAIN_Gr_INDEX;
					break;
				case 2:
					sprintf( (char *)info_ptr->ItemNamePtr, "Pregain-Gb" );
					temp_addr = JX507MIPI_PRE_GAIN_Gb_INDEX;
					break;
				case 3:
					sprintf( (char *)info_ptr->ItemNamePtr, "Pregain-B" );
					temp_addr = JX507MIPI_PRE_GAIN_B_INDEX;
					break;
				case 4:
					sprintf( (char *)info_ptr->ItemNamePtr, "SENSOR_BASEGAIN" );
					temp_addr = JX507MIPI_SENSOR_BASEGAIN;
					break;
				default:
					ASSERT( 0 );
			}
			
			temp_para = JX507SensorCCT[temp_addr].Para;
			//temp_gain = (temp_para/jx507.sensorBaseGain) * 1000;
			
			info_ptr->ItemValue		= temp_gain;
			info_ptr->IsTrueFalse	= KAL_FALSE;
			info_ptr->IsReadOnly	= KAL_FALSE;
			info_ptr->IsNeedRestart	= KAL_FALSE;
			info_ptr->Min			= JX507MIPI_MIN_ANALOG_GAIN * 1000;
			info_ptr->Max			= JX507MIPI_MAX_ANALOG_GAIN * 1000;
			break;
		case JX507MIPI_CMMCLK_CURRENT:
			switch( item_idx )
			{
				case 0:
					sprintf( (char *)info_ptr->ItemNamePtr, "Drv Cur[2,4,6,8]mA" );
					
					//temp_reg=MT9P017SensorReg[CMMCLK_CURRENT_INDEX].Para;
					temp_reg = ISP_DRIVING_2MA;
					if( temp_reg==ISP_DRIVING_2MA )
					{
						info_ptr->ItemValue = 2;
					}
					else if( temp_reg==ISP_DRIVING_4MA )
					{
						info_ptr->ItemValue = 4;
					}
					else if( temp_reg==ISP_DRIVING_6MA )
					{
						info_ptr->ItemValue = 6;
					}
					else if( temp_reg==ISP_DRIVING_8MA )
					{
						info_ptr->ItemValue = 8;
					}
					
					info_ptr->IsTrueFalse	= KAL_FALSE;
					info_ptr->IsReadOnly	= KAL_FALSE;
					info_ptr->IsNeedRestart	= KAL_TRUE;
					info_ptr->Min			= 2;
					info_ptr->Max			= 8;
					break;
				default:
					ASSERT( 0 );
			}
			break;
		case JX507MIPI_FRAME_RATE_LIMITATION:
			switch( item_idx )
			{
				case 0:
					sprintf( (char *)info_ptr->ItemNamePtr, "Max Exposure Lines" );
					info_ptr->ItemValue		= 111;  //MT9P017_MAX_EXPOSURE_LINES;
					info_ptr->IsTrueFalse	= KAL_FALSE;
					info_ptr->IsReadOnly	= KAL_TRUE;
					info_ptr->IsNeedRestart	= KAL_FALSE;
					info_ptr->Min			= 0;
					info_ptr->Max			= 0;
					break;
				case 1:
					sprintf( (char *)info_ptr->ItemNamePtr, "Min Frame Rate" );
					info_ptr->ItemValue		= 12;
					info_ptr->IsTrueFalse	= KAL_FALSE;
					info_ptr->IsReadOnly	= KAL_TRUE;
					info_ptr->IsNeedRestart	= KAL_FALSE;
					info_ptr->Min			= 0;
					info_ptr->Max			= 0;
					break;
				default:
					ASSERT( 0 );
			}
			break;
		case JX507MIPI_REGISTER_EDITOR:
			switch( item_idx )
			{
				case 0:
					sprintf( (char *)info_ptr->ItemNamePtr, "REG Addr." );
					info_ptr->ItemValue		= 0;
					info_ptr->IsTrueFalse	= KAL_FALSE;
					info_ptr->IsReadOnly	= KAL_FALSE;
					info_ptr->IsNeedRestart	= KAL_FALSE;
					info_ptr->Min			= 0;
					info_ptr->Max			= 0xFFFF;
					break;
				case 1:
					sprintf( (char *)info_ptr->ItemNamePtr, "REG Value" );
					info_ptr->ItemValue		= 0;
					info_ptr->IsTrueFalse	= KAL_FALSE;
					info_ptr->IsReadOnly	= KAL_FALSE;
					info_ptr->IsNeedRestart	= KAL_FALSE;
					info_ptr->Min			= 0;
					info_ptr->Max			= 0xFFFF;
					break;
				default:
				ASSERT( 0 );
			}
			break;
		default:
			ASSERT( 0 );
	}
}

kal_bool JX507MIPI_set_sensor_item_info(kal_uint16 group_idx, kal_uint16 item_idx, kal_int32 ItemValue)
{
//	kal_int16	temp_reg;
	kal_uint16	temp_gain=0, temp_addr=0, temp_para=0;
	
	switch( group_idx )
	{
		case JX507MIPI_PRE_GAIN:
			switch( item_idx )
			{
				case 0:
					temp_addr = JX507MIPI_PRE_GAIN_R_INDEX;
					break;
				case 1:
					temp_addr = JX507MIPI_PRE_GAIN_Gr_INDEX;
					break;
				case 2:
					temp_addr = JX507MIPI_PRE_GAIN_Gb_INDEX;
					break;
				case 3:
					temp_addr = JX507MIPI_PRE_GAIN_B_INDEX;
					break;
				case 4:
					temp_addr = JX507MIPI_SENSOR_BASEGAIN;
					break;
				default:
					ASSERT( 0 );
			}
			
			temp_gain = ((ItemValue * BASEGAIN + 500) / 1000);	//+500:get closed integer value
			
			if( temp_gain >= 1*BASEGAIN && temp_gain <= 16*BASEGAIN )
			{
//				temp_para = (temp_gain * jx507.sensorBaseGain + BASEGAIN/2) / BASEGAIN;
			}
			else
				ASSERT( 0 );
			
			SENSORDBSOFIA( "JX507????????????????????? :\n" );
			spin_lock( &jx507mipi_drv_lock );
			JX507SensorCCT[temp_addr].Para = temp_para;
			spin_unlock( &jx507mipi_drv_lock );
			JX507MIPI_write_cmos_sensor( JX507SensorCCT[temp_addr].Addr, temp_para );
			break;
		case JX507MIPI_CMMCLK_CURRENT:
			switch( item_idx )
			{
				case 0:
					//no need to apply this item for driving current
					break;
				default:
					ASSERT( 0 );
			}
			break;
		case JX507MIPI_FRAME_RATE_LIMITATION:
			ASSERT( 0 );
			break;
		case JX507MIPI_REGISTER_EDITOR:
			switch( item_idx )
			{
				case 0:
					spin_lock( &jx507mipi_drv_lock );
					JX507MIPI_FAC_SENSOR_REG = ItemValue;
					spin_unlock( &jx507mipi_drv_lock );
					break;
				case 1:
					JX507MIPI_write_cmos_sensor( JX507MIPI_FAC_SENSOR_REG, ItemValue );
					break;
				default:
					ASSERT( 0 );
			}
			break;
		default:
			ASSERT( 0 );
	}
	return KAL_TRUE;
}

void JX507MIPISetFlipMirror(kal_int32 imgMirror)
{
	#if 0
	kal_int8 tmp = 0;
	
	tmp = (kal_int8)JX507MIPI_read_cmos_sensor( 0x12 );
	switch( imgMirror )
	{
		case IMAGE_NORMAL:
			#ifdef _SENSOR_INITIAL_FLIP //min@20131224
			tmp = (tmp & 0xCF) | 0x10;
			#else
			tmp = tmp & 0xCF;
			#endif
			JX507MIPI_write_cmos_sensor( 0x12, (kal_int32)tmp );	// Set normal mode.
			gMirrorFlip = 0x0;
			break;
		case IMAGE_H_MIRROR:
			#ifdef _SENSOR_INITIAL_FLIP //min@20131224
			tmp = tmp | 0x30;
			#else
			tmp = (tmp & 0xCF) | 0x20;
			#endif
			JX507MIPI_write_cmos_sensor( 0x12, (kal_int32)tmp );	// Set mirror mode.
			gMirrorFlip = 0x2;
			break;
		case IMAGE_V_MIRROR:
			#ifdef _SENSOR_INITIAL_FLIP //min@20131224
			tmp = tmp & 0xCF;
			#else
			tmp = (tmp & 0xCF) | 0x10;
			#endif
			JX507MIPI_write_cmos_sensor( 0x12, (kal_int32)tmp );	// Set flip mode.
			gMirrorFlip = 0x1;
			break;
		case IMAGE_HV_MIRROR:
			#ifdef _SENSOR_INITIAL_FLIP //min@20131224
			tmp = (tmp & 0xCF) | 0x20;
			#else
			tmp = tmp | 0x30;
			#endif
			JX507MIPI_write_cmos_sensor( 0x12, (kal_int32)tmp );	// Set mirror & flip mode.
			gMirrorFlip = 0x3;
			break;
	}
	#endif
}

////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////

static void JX507MIPI_Sensor_Init(void)
{
	SENSORDB( ">> JX507MIPI_Sensor_Init()\n" );
	
	SENSORDB( "Reset sensor...\n" );
	JX507MIPI_write_cmos_sensor( 0x12, 0x80 );
	//(5ms)
	//kal_sleep_task(2);
	mdelay( 6 );
	
	#ifdef _JX507_INI_20140820 //Terra20140820-01
		#ifdef _VAVG_ON
		#ifdef _SENSOR_INITIAL_FLIP
		JX507MIPI_write_cmos_sensor( 0x12, 0x73 );
		#else
		JX507MIPI_write_cmos_sensor( 0x12, 0x43 );
		#endif
		#else //_VAVG_OFF
		JX507MIPI_write_cmos_sensor( 0x12, 0x41 );
		#endif
		JX507MIPI_write_cmos_sensor( 0x1D, 0x00 );
		JX507MIPI_write_cmos_sensor( 0x1E, 0x00 );
		JX507MIPI_write_cmos_sensor( 0x6C, 0x40 );
		JX507MIPI_write_cmos_sensor( 0x73, 0x33 );
		JX507MIPI_write_cmos_sensor( 0x70, 0x69 );
		JX507MIPI_write_cmos_sensor( 0x76, 0x40 );
		JX507MIPI_write_cmos_sensor( 0x77, 0x06 );
		JX507MIPI_write_cmos_sensor( 0x74, 0x02 );
		JX507MIPI_write_cmos_sensor( 0x78, 0x2C );
		JX507MIPI_write_cmos_sensor( 0x0D, 0x50 );
		JX507MIPI_write_cmos_sensor( 0x0E, 0x10 );
		JX507MIPI_write_cmos_sensor( 0x0F, 0x04 );
		JX507MIPI_write_cmos_sensor( 0x10, 0x0D );
		JX507MIPI_write_cmos_sensor( 0x11, 0x80 );
		JX507MIPI_write_cmos_sensor( 0x20, 0x8A );
		JX507MIPI_write_cmos_sensor( 0x21, 0x07 );
		JX507MIPI_write_cmos_sensor( 0x22, 0x36 );
		JX507MIPI_write_cmos_sensor( 0x23, 0x04 );
		JX507MIPI_write_cmos_sensor( 0x24, 0x00 );
		JX507MIPI_write_cmos_sensor( 0x25, 0xC0 );
		JX507MIPI_write_cmos_sensor( 0x26, 0x35 );
		JX507MIPI_write_cmos_sensor( 0x27, 0x97 );
		JX507MIPI_write_cmos_sensor( 0x28, 0x07 );
		JX507MIPI_write_cmos_sensor( 0x29, 0x02 );
		JX507MIPI_write_cmos_sensor( 0x2A, 0x81 );
		JX507MIPI_write_cmos_sensor( 0x2B, 0x2A );
		JX507MIPI_write_cmos_sensor( 0x2C, 0x06 );
		JX507MIPI_write_cmos_sensor( 0x2D, 0x03 );
		JX507MIPI_write_cmos_sensor( 0x2E, 0xE8 );
		JX507MIPI_write_cmos_sensor( 0x2F, 0x04 );
		JX507MIPI_write_cmos_sensor( 0x30, 0x93 );
		JX507MIPI_write_cmos_sensor( 0x31, 0x0E );
		JX507MIPI_write_cmos_sensor( 0x32, 0xBE );
		JX507MIPI_write_cmos_sensor( 0x33, 0x10 );
		JX507MIPI_write_cmos_sensor( 0x34, 0x23 );
		JX507MIPI_write_cmos_sensor( 0x35, 0xCA );
		JX507MIPI_write_cmos_sensor( 0x5F, 0x43 );
		JX507MIPI_write_cmos_sensor( 0x60, 0x27 );
		JX507MIPI_write_cmos_sensor( 0x61, 0xF8 );
		JX507MIPI_write_cmos_sensor( 0x62, 0x00 );
		JX507MIPI_write_cmos_sensor( 0x63, 0xC0 );
		JX507MIPI_write_cmos_sensor( 0x64, 0x07 );
		JX507MIPI_write_cmos_sensor( 0x67, 0x79 );
		JX507MIPI_write_cmos_sensor( 0x6A, 0x3A );
		JX507MIPI_write_cmos_sensor( 0x69, 0x74 );
		JX507MIPI_write_cmos_sensor( 0x13, 0x81 );
		JX507MIPI_write_cmos_sensor( 0x14, 0x80 );
		JX507MIPI_write_cmos_sensor( 0x16, 0xC0 );
		JX507MIPI_write_cmos_sensor( 0x17, 0x40 );
		JX507MIPI_write_cmos_sensor( 0x18, 0x43 );
		JX507MIPI_write_cmos_sensor( 0x19, 0x29 );
		JX507MIPI_write_cmos_sensor( 0x38, 0xE9 );
		JX507MIPI_write_cmos_sensor( 0x39, 0x20 );
		JX507MIPI_write_cmos_sensor( 0x4a, 0x03 );
		JX507MIPI_write_cmos_sensor( 0x49, 0x10 );
		JX507MIPI_write_cmos_sensor( 0x50, 0x10 );
		JX507MIPI_write_cmos_sensor( 0x6D, 0x0A );
		JX507MIPI_write_cmos_sensor( 0x71, 0x6A );
		#ifdef _VAVG_ON
		#ifdef _SENSOR_INITIAL_FLIP
		JX507MIPI_write_cmos_sensor( 0x12, 0x33 );
		#else
		JX507MIPI_write_cmos_sensor( 0x12, 0x03 );
		#endif
		#else //_VAVG_OFF
		JX507MIPI_write_cmos_sensor( 0x12, 0x01 );
		#endif
	#endif //_JX507_INI_20140820
	////////////////////////////////////////////////////////////////////////////
	#ifdef _JX507_INI_20140211
		#ifdef _VAVG_ON
		#ifdef _SENSOR_INITIAL_FLIP
		JX507MIPI_write_cmos_sensor( 0x12, 0x73 );
		#else
		JX507MIPI_write_cmos_sensor( 0x12, 0x43 );
		#endif
		#else //_VAVG_OFF
		JX507MIPI_write_cmos_sensor( 0x12, 0x41 );
		#endif
		JX507MIPI_write_cmos_sensor( 0x0D, 0x00 );
		JX507MIPI_write_cmos_sensor( 0x0E, 0x10 );
		JX507MIPI_write_cmos_sensor( 0x0F, 0x04 );
		JX507MIPI_write_cmos_sensor( 0x10, 0x13 );
		JX507MIPI_write_cmos_sensor( 0x11, 0x80 );
		JX507MIPI_write_cmos_sensor( 0x20, 0x88 );
		JX507MIPI_write_cmos_sensor( 0x21, 0x07 );
		JX507MIPI_write_cmos_sensor( 0x22, 0x2A );
		JX507MIPI_write_cmos_sensor( 0x23, 0x06 );
		JX507MIPI_write_cmos_sensor( 0x24, 0x00 );
		JX507MIPI_write_cmos_sensor( 0x25, 0xC0 );
		JX507MIPI_write_cmos_sensor( 0x26, 0x35 );
		JX507MIPI_write_cmos_sensor( 0x27, 0x96 );
		JX507MIPI_write_cmos_sensor( 0x28, 0x07 );
		JX507MIPI_write_cmos_sensor( 0x29, 0x02 );
		JX507MIPI_write_cmos_sensor( 0x2A, 0x81 );
		JX507MIPI_write_cmos_sensor( 0x2B, 0x2A );
		JX507MIPI_write_cmos_sensor( 0x2C, 0x06 );
///		JX507MIPI_write_cmos_sensor( 0x2C, 0x02 );
		JX507MIPI_write_cmos_sensor( 0x2D, 0x03 );
		JX507MIPI_write_cmos_sensor( 0x2E, 0xE8 );
		JX507MIPI_write_cmos_sensor( 0x2F, 0x04 );
		JX507MIPI_write_cmos_sensor( 0x31, 0x14 );
		JX507MIPI_write_cmos_sensor( 0x32, 0xBE );
		JX507MIPI_write_cmos_sensor( 0x33, 0x18 );
		JX507MIPI_write_cmos_sensor( 0x34, 0x3E );
		JX507MIPI_write_cmos_sensor( 0x35, 0xEC );
		#ifdef _DRIVE_OPT
		JX507MIPI_write_cmos_sensor( 0x5F, 0x43 );
		#else
		JX507MIPI_write_cmos_sensor( 0x5F, 0x03 );
		#endif
		JX507MIPI_write_cmos_sensor( 0x60, 0x27 );
		JX507MIPI_write_cmos_sensor( 0x61, 0xFC );
		JX507MIPI_write_cmos_sensor( 0x62, 0x00 );
		JX507MIPI_write_cmos_sensor( 0x63, 0xC0 );
		JX507MIPI_write_cmos_sensor( 0x64, 0x07 );
		JX507MIPI_write_cmos_sensor( 0x67, 0x79 );
		JX507MIPI_write_cmos_sensor( 0x69, 0x74 );
		JX507MIPI_write_cmos_sensor( 0x6A, 0x3A );
		JX507MIPI_write_cmos_sensor( 0x13, 0x81 );
		JX507MIPI_write_cmos_sensor( 0x14, 0x80 );
		JX507MIPI_write_cmos_sensor( 0x16, 0xC0 );
		JX507MIPI_write_cmos_sensor( 0x17, 0x40 );
		JX507MIPI_write_cmos_sensor( 0x18, 0xD9 );
		JX507MIPI_write_cmos_sensor( 0x19, 0x29 );
		JX507MIPI_write_cmos_sensor( 0x38, 0xE9 );
		JX507MIPI_write_cmos_sensor( 0x39, 0x20 );
		JX507MIPI_write_cmos_sensor( 0x4a, 0x03 );
		JX507MIPI_write_cmos_sensor( 0x49, 0x10 );
		JX507MIPI_write_cmos_sensor( 0x50, 0x10 );
		JX507MIPI_write_cmos_sensor( 0x1D, 0x00 );
		JX507MIPI_write_cmos_sensor( 0x1E, 0x00 );
		JX507MIPI_write_cmos_sensor( 0x68, 0x00 );
		JX507MIPI_write_cmos_sensor( 0x6C, 0x00 );
		JX507MIPI_write_cmos_sensor( 0x73, 0x33 );
		JX507MIPI_write_cmos_sensor( 0x70, 0x69 );
		JX507MIPI_write_cmos_sensor( 0x76, 0x40 );
		JX507MIPI_write_cmos_sensor( 0x77, 0x06 );
		JX507MIPI_write_cmos_sensor( 0x6D, 0x09 );
		JX507MIPI_write_cmos_sensor( 0x74, 0x02 );
		#ifdef _VAVG_ON //min@20140212
		#ifdef _SENSOR_INITIAL_FLIP
		JX507MIPI_write_cmos_sensor( 0x12, 0x33 );
		#else
		JX507MIPI_write_cmos_sensor( 0x12, 0x03 );
		#endif
		#else //_VAVG_OFF
		JX507MIPI_write_cmos_sensor( 0x12, 0x01 );
		#endif
	#endif //_JX507_INI_20140211
	
	SENSORDB( "<< JX507MIPI_Sensor_Init()\n" );
}

static void JX507MIPI_Sensor_1080P(void) 
{
	SENSORDB( ">> JX507MIPI_Sensor_1080P()\n" );
	
	if( ReEnteyCamera == KAL_TRUE ) {
		
	}
	else {
		
	}
	
	#ifdef _JX507_INI_20140820 //Terra20140820-01
		#ifdef _SENSOR_INITIAL_FLIP
		JX507MIPI_write_cmos_sensor( 0x12, 0x70 );
		#else
		JX507MIPI_write_cmos_sensor( 0x12, 0x40 );
		#endif
		JX507MIPI_write_cmos_sensor( 0x1D, 0x00 );
		JX507MIPI_write_cmos_sensor( 0x1E, 0x00 );
		JX507MIPI_write_cmos_sensor( 0x6C, 0x40 );
		JX507MIPI_write_cmos_sensor( 0x73, 0x33 );
		JX507MIPI_write_cmos_sensor( 0x70, 0x68 );
		JX507MIPI_write_cmos_sensor( 0x76, 0x60 );
		JX507MIPI_write_cmos_sensor( 0x77, 0x09 );
		JX507MIPI_write_cmos_sensor( 0x74, 0x02 );
		JX507MIPI_write_cmos_sensor( 0x78, 0x16 );
		JX507MIPI_write_cmos_sensor( 0x0D, 0x50 );
		JX507MIPI_write_cmos_sensor( 0x0E, 0x10 );
		JX507MIPI_write_cmos_sensor( 0x0F, 0x04 );
		JX507MIPI_write_cmos_sensor( 0x10, 0x13 );
		JX507MIPI_write_cmos_sensor( 0x11, 0x80 );
		JX507MIPI_write_cmos_sensor( 0x20, 0x96 );
		JX507MIPI_write_cmos_sensor( 0x21, 0x04 );
		JX507MIPI_write_cmos_sensor( 0x22, 0x0A );
		JX507MIPI_write_cmos_sensor( 0x23, 0x05 );
		JX507MIPI_write_cmos_sensor( 0x24, 0xC0 );
		JX507MIPI_write_cmos_sensor( 0x25, 0x38 );
		JX507MIPI_write_cmos_sensor( 0x26, 0x43 );
		JX507MIPI_write_cmos_sensor( 0x27, 0xE4 );
		JX507MIPI_write_cmos_sensor( 0x28, 0x0D );
		JX507MIPI_write_cmos_sensor( 0x29, 0x00 );
		JX507MIPI_write_cmos_sensor( 0x2A, 0xD9 );
		JX507MIPI_write_cmos_sensor( 0x2B, 0x28 );
		JX507MIPI_write_cmos_sensor( 0x2C, 0x56 );
		JX507MIPI_write_cmos_sensor( 0x2D, 0x6C );
		JX507MIPI_write_cmos_sensor( 0x2E, 0x7F );
		JX507MIPI_write_cmos_sensor( 0x2F, 0x04 );
		JX507MIPI_write_cmos_sensor( 0x30, 0x8E );
		JX507MIPI_write_cmos_sensor( 0x31, 0x0A );
		JX507MIPI_write_cmos_sensor( 0x32, 0x9F );
		JX507MIPI_write_cmos_sensor( 0x33, 0x0C );
		JX507MIPI_write_cmos_sensor( 0x34, 0x1F );
		JX507MIPI_write_cmos_sensor( 0x35, 0xB6 );
		JX507MIPI_write_cmos_sensor( 0x5F, 0x43 );
		JX507MIPI_write_cmos_sensor( 0x60, 0x27 );
		JX507MIPI_write_cmos_sensor( 0x61, 0xF8 );
		JX507MIPI_write_cmos_sensor( 0x62, 0x00 );
		JX507MIPI_write_cmos_sensor( 0x63, 0xC0 );
		JX507MIPI_write_cmos_sensor( 0x64, 0x07 );
		JX507MIPI_write_cmos_sensor( 0x67, 0x79 );
		JX507MIPI_write_cmos_sensor( 0x6A, 0x3A );
		JX507MIPI_write_cmos_sensor( 0x69, 0x74 );
		JX507MIPI_write_cmos_sensor( 0x13, 0x81 );
		JX507MIPI_write_cmos_sensor( 0x14, 0x80 );
		JX507MIPI_write_cmos_sensor( 0x16, 0xC0 );
		JX507MIPI_write_cmos_sensor( 0x17, 0x40 );
		JX507MIPI_write_cmos_sensor( 0x18, 0x82 );
		JX507MIPI_write_cmos_sensor( 0x19, 0x21 );
		JX507MIPI_write_cmos_sensor( 0x38, 0x0F );
		JX507MIPI_write_cmos_sensor( 0x39, 0xFF );
		JX507MIPI_write_cmos_sensor( 0x4a, 0x03 );
		JX507MIPI_write_cmos_sensor( 0x49, 0x10 );
		JX507MIPI_write_cmos_sensor( 0x50, 0x10 );
		JX507MIPI_write_cmos_sensor( 0x6D, 0x0A );
		JX507MIPI_write_cmos_sensor( 0x71, 0x6A );
		#ifdef _SENSOR_INITIAL_FLIP
		JX507MIPI_write_cmos_sensor( 0x12, 0x30 );
		#else
		JX507MIPI_write_cmos_sensor( 0x12, 0x00 );
		#endif
	#endif //_JX507_INI_20140820
	////////////////////////////////////////////////////////////////////////////
	#ifdef _JX507_INI_20140211
		#ifdef _SENSOR_INITIAL_FLIP
		JX507MIPI_write_cmos_sensor( 0x12, 0x70 );
		#else
		JX507MIPI_write_cmos_sensor( 0x12, 0x40 );
		#endif
		JX507MIPI_write_cmos_sensor( 0x0D, 0x00 );
		JX507MIPI_write_cmos_sensor( 0x0E, 0x10 );
		JX507MIPI_write_cmos_sensor( 0x0F, 0x04 );
		JX507MIPI_write_cmos_sensor( 0x10, 0x13 );
		JX507MIPI_write_cmos_sensor( 0x11, 0x80 );
		JX507MIPI_write_cmos_sensor( 0x20, 0x08 );
		JX507MIPI_write_cmos_sensor( 0x21, 0x09 );
		JX507MIPI_write_cmos_sensor( 0x22, 0x24 );
		JX507MIPI_write_cmos_sensor( 0x23, 0x05 );
		JX507MIPI_write_cmos_sensor( 0x24, 0x80 );
		JX507MIPI_write_cmos_sensor( 0x25, 0x38 );
		JX507MIPI_write_cmos_sensor( 0x26, 0x47 );
		JX507MIPI_write_cmos_sensor( 0x27, 0x99 );
		JX507MIPI_write_cmos_sensor( 0x28, 0x0D );
		JX507MIPI_write_cmos_sensor( 0x29, 0x01 );
		JX507MIPI_write_cmos_sensor( 0x2A, 0x80 );
		JX507MIPI_write_cmos_sensor( 0x2B, 0x29 );
		JX507MIPI_write_cmos_sensor( 0x2C, 0x55 );
		JX507MIPI_write_cmos_sensor( 0x2D, 0x6C );
		JX507MIPI_write_cmos_sensor( 0x2E, 0x7E );
		JX507MIPI_write_cmos_sensor( 0x2F, 0x04 );
		JX507MIPI_write_cmos_sensor( 0x31, 0x14 );
		JX507MIPI_write_cmos_sensor( 0x32, 0xBE );
		JX507MIPI_write_cmos_sensor( 0x33, 0x18 );
		JX507MIPI_write_cmos_sensor( 0x34, 0x3E );
		JX507MIPI_write_cmos_sensor( 0x35, 0xEC );
		#ifdef _DRIVE_OPT //min@20140331
		JX507MIPI_write_cmos_sensor( 0x5F, 0x43 );
		#else
		JX507MIPI_write_cmos_sensor( 0x5F, 0x03 );
		#endif
		JX507MIPI_write_cmos_sensor( 0x60, 0x27 );
		JX507MIPI_write_cmos_sensor( 0x61, 0xFC );
		JX507MIPI_write_cmos_sensor( 0x62, 0x00 );
		JX507MIPI_write_cmos_sensor( 0x63, 0xC0 );
		JX507MIPI_write_cmos_sensor( 0x64, 0x07 );
		JX507MIPI_write_cmos_sensor( 0x67, 0x79 );
		JX507MIPI_write_cmos_sensor( 0x69, 0x74 );
		JX507MIPI_write_cmos_sensor( 0x6A, 0x3A );
		JX507MIPI_write_cmos_sensor( 0x13, 0x81 );
		JX507MIPI_write_cmos_sensor( 0x14, 0x80 );
		JX507MIPI_write_cmos_sensor( 0x16, 0xC0 );
		JX507MIPI_write_cmos_sensor( 0x17, 0x40 );
		JX507MIPI_write_cmos_sensor( 0x18, 0x8A );
		JX507MIPI_write_cmos_sensor( 0x19, 0x29 );
		JX507MIPI_write_cmos_sensor( 0x38, 0x0F );
		JX507MIPI_write_cmos_sensor( 0x39, 0xFF );
		JX507MIPI_write_cmos_sensor( 0x4a, 0x03 );
		JX507MIPI_write_cmos_sensor( 0x49, 0x10 );
		JX507MIPI_write_cmos_sensor( 0x50, 0x10 );
		JX507MIPI_write_cmos_sensor( 0x1D, 0x00 );
		JX507MIPI_write_cmos_sensor( 0x1E, 0x00 );
		JX507MIPI_write_cmos_sensor( 0x68, 0x00 );
		JX507MIPI_write_cmos_sensor( 0x6C, 0x00 );
		JX507MIPI_write_cmos_sensor( 0x73, 0x33 );
		JX507MIPI_write_cmos_sensor( 0x70, 0x69 );
		JX507MIPI_write_cmos_sensor( 0x76, 0x60 );
		JX507MIPI_write_cmos_sensor( 0x77, 0x09 );
		JX507MIPI_write_cmos_sensor( 0x6D, 0x09 );
		JX507MIPI_write_cmos_sensor( 0x74, 0x02 );
		#ifdef _SENSOR_INITIAL_FLIP
		JX507MIPI_write_cmos_sensor( 0x12, 0x30 );
		#else
		JX507MIPI_write_cmos_sensor( 0x12, 0x00 );
		#endif
	#endif //_JX507_INI_20140211
	
	if( ReEnteyCamera == KAL_TRUE ) {
		
	}
	else {
		
	}
	ReEnteyCamera = KAL_FALSE;
	
	SENSORDB( "<< JX507MIPI_Sensor_1080P()\n" );
}

static void JX507MIPI_Sensor_1M(void)
{
	SENSORDB( ">> JX507MIPI_Sensor_1M()\n" );
	
	if( ReEnteyCamera == KAL_TRUE ) {
		
	}
	else {
		
	}	
	
	#ifdef _JX507_INI_20140820 //Terra20140820-01
		#ifdef _VAVG_ON
		#ifdef _SENSOR_INITIAL_FLIP
		JX507MIPI_write_cmos_sensor( 0x12, 0x73 );
		#else
		JX507MIPI_write_cmos_sensor( 0x12, 0x43 );
		#endif
		#else //_VAVG_OFF
		JX507MIPI_write_cmos_sensor( 0x12, 0x41 );
		#endif
		JX507MIPI_write_cmos_sensor( 0x1D, 0x00 );
		JX507MIPI_write_cmos_sensor( 0x1E, 0x00 );
		JX507MIPI_write_cmos_sensor( 0x6C, 0x40 );
		JX507MIPI_write_cmos_sensor( 0x73, 0x33 );
		JX507MIPI_write_cmos_sensor( 0x70, 0x69 );
		JX507MIPI_write_cmos_sensor( 0x76, 0x40 );
		JX507MIPI_write_cmos_sensor( 0x77, 0x06 );
		JX507MIPI_write_cmos_sensor( 0x74, 0x02 );
		JX507MIPI_write_cmos_sensor( 0x78, 0x2C );
		JX507MIPI_write_cmos_sensor( 0x0D, 0x50 );
		JX507MIPI_write_cmos_sensor( 0x0E, 0x10 );
		JX507MIPI_write_cmos_sensor( 0x0F, 0x04 );
		JX507MIPI_write_cmos_sensor( 0x10, 0x0D );
		JX507MIPI_write_cmos_sensor( 0x11, 0x80 );
		JX507MIPI_write_cmos_sensor( 0x20, 0x8A );
		JX507MIPI_write_cmos_sensor( 0x21, 0x07 );
		JX507MIPI_write_cmos_sensor( 0x22, 0x36 );
		JX507MIPI_write_cmos_sensor( 0x23, 0x04 );
		JX507MIPI_write_cmos_sensor( 0x24, 0x00 );
		JX507MIPI_write_cmos_sensor( 0x25, 0xC0 );
		JX507MIPI_write_cmos_sensor( 0x26, 0x35 );
		JX507MIPI_write_cmos_sensor( 0x27, 0x97 );
		JX507MIPI_write_cmos_sensor( 0x28, 0x07 );
		JX507MIPI_write_cmos_sensor( 0x29, 0x02 );
		JX507MIPI_write_cmos_sensor( 0x2A, 0x81 );
		JX507MIPI_write_cmos_sensor( 0x2B, 0x2A );
		JX507MIPI_write_cmos_sensor( 0x2C, 0x06 );
		JX507MIPI_write_cmos_sensor( 0x2D, 0x03 );
		JX507MIPI_write_cmos_sensor( 0x2E, 0xE8 );
		JX507MIPI_write_cmos_sensor( 0x2F, 0x04 );
		JX507MIPI_write_cmos_sensor( 0x30, 0x93 );
		JX507MIPI_write_cmos_sensor( 0x31, 0x0E );
		JX507MIPI_write_cmos_sensor( 0x32, 0xBE );
		JX507MIPI_write_cmos_sensor( 0x33, 0x10 );
		JX507MIPI_write_cmos_sensor( 0x34, 0x23 );
		JX507MIPI_write_cmos_sensor( 0x35, 0xCA );
		JX507MIPI_write_cmos_sensor( 0x5F, 0x43 );
		JX507MIPI_write_cmos_sensor( 0x60, 0x27 );
		JX507MIPI_write_cmos_sensor( 0x61, 0xF8 );
		JX507MIPI_write_cmos_sensor( 0x62, 0x00 );
		JX507MIPI_write_cmos_sensor( 0x63, 0xC0 );
		JX507MIPI_write_cmos_sensor( 0x64, 0x07 );
		JX507MIPI_write_cmos_sensor( 0x67, 0x79 );
		JX507MIPI_write_cmos_sensor( 0x6A, 0x3A );
		JX507MIPI_write_cmos_sensor( 0x69, 0x74 );
		JX507MIPI_write_cmos_sensor( 0x13, 0x81 );
		JX507MIPI_write_cmos_sensor( 0x14, 0x80 );
		JX507MIPI_write_cmos_sensor( 0x16, 0xC0 );
		JX507MIPI_write_cmos_sensor( 0x17, 0x40 );
		JX507MIPI_write_cmos_sensor( 0x18, 0x43 );
		JX507MIPI_write_cmos_sensor( 0x19, 0x29 );
		JX507MIPI_write_cmos_sensor( 0x38, 0xE9 );
		JX507MIPI_write_cmos_sensor( 0x39, 0x20 );
		JX507MIPI_write_cmos_sensor( 0x4a, 0x03 );
		JX507MIPI_write_cmos_sensor( 0x49, 0x10 );
		JX507MIPI_write_cmos_sensor( 0x50, 0x10 );
		JX507MIPI_write_cmos_sensor( 0x6D, 0x0A );
		JX507MIPI_write_cmos_sensor( 0x71, 0x6A );
		#ifdef _VAVG_ON
		#ifdef _SENSOR_INITIAL_FLIP
		JX507MIPI_write_cmos_sensor( 0x12, 0x33 );
		#else
		JX507MIPI_write_cmos_sensor( 0x12, 0x03 );
		#endif
		#else //_VAVG_OFF
		JX507MIPI_write_cmos_sensor( 0x12, 0x01 );
		#endif
	#endif //_JX507_INI_20140820
	////////////////////////////////////////////////////////////////////////////
	#ifdef _JX507_INI_20140211
		#ifdef _VAVG_ON
		#ifdef _SENSOR_INITIAL_FLIP
		JX507MIPI_write_cmos_sensor( 0x12, 0x73 );
		#else
		JX507MIPI_write_cmos_sensor( 0x12, 0x43 );
		#endif
		#else //_VAVG_OFF
		JX507MIPI_write_cmos_sensor( 0x12, 0x41 );
		#endif
		JX507MIPI_write_cmos_sensor( 0x0D, 0x00 );
		JX507MIPI_write_cmos_sensor( 0x0E, 0x10 );
		JX507MIPI_write_cmos_sensor( 0x0F, 0x04 );
		JX507MIPI_write_cmos_sensor( 0x10, 0x13 );
		JX507MIPI_write_cmos_sensor( 0x11, 0x80 );
		JX507MIPI_write_cmos_sensor( 0x20, 0x88 );
		JX507MIPI_write_cmos_sensor( 0x21, 0x07 );
		JX507MIPI_write_cmos_sensor( 0x22, 0x2A );
		JX507MIPI_write_cmos_sensor( 0x23, 0x06 );
		JX507MIPI_write_cmos_sensor( 0x24, 0x00 );
		JX507MIPI_write_cmos_sensor( 0x25, 0xC0 );
		JX507MIPI_write_cmos_sensor( 0x26, 0x35 );
		JX507MIPI_write_cmos_sensor( 0x27, 0x96 );
		JX507MIPI_write_cmos_sensor( 0x28, 0x07 );
		JX507MIPI_write_cmos_sensor( 0x29, 0x02 );
		JX507MIPI_write_cmos_sensor( 0x2A, 0x81 );
		JX507MIPI_write_cmos_sensor( 0x2B, 0x2A );
		JX507MIPI_write_cmos_sensor( 0x2C, 0x06 );
///		JX507MIPI_write_cmos_sensor( 0x2C, 0x02 );
		JX507MIPI_write_cmos_sensor( 0x2D, 0x03 );
		JX507MIPI_write_cmos_sensor( 0x2E, 0xE8 );
		JX507MIPI_write_cmos_sensor( 0x2F, 0x04 );
		JX507MIPI_write_cmos_sensor( 0x31, 0x14 );
		JX507MIPI_write_cmos_sensor( 0x32, 0xBE );
		JX507MIPI_write_cmos_sensor( 0x33, 0x18 );
		JX507MIPI_write_cmos_sensor( 0x34, 0x3E );
		JX507MIPI_write_cmos_sensor( 0x35, 0xEC );
		#ifdef _DRIVE_OPT
		JX507MIPI_write_cmos_sensor( 0x5F, 0x43 );
		#else
		JX507MIPI_write_cmos_sensor( 0x5F, 0x03 );
		#endif
		JX507MIPI_write_cmos_sensor( 0x60, 0x27 );
		JX507MIPI_write_cmos_sensor( 0x61, 0xFC );
		JX507MIPI_write_cmos_sensor( 0x62, 0x00 );
		JX507MIPI_write_cmos_sensor( 0x63, 0xC0 );
		JX507MIPI_write_cmos_sensor( 0x64, 0x07 );
	 	JX507MIPI_write_cmos_sensor( 0x67, 0x79 );
		JX507MIPI_write_cmos_sensor( 0x69, 0x74 );
		JX507MIPI_write_cmos_sensor( 0x6A, 0x3A );
		JX507MIPI_write_cmos_sensor( 0x13, 0x81 );
		JX507MIPI_write_cmos_sensor( 0x14, 0x80 );
		JX507MIPI_write_cmos_sensor( 0x16, 0xC0 );
		JX507MIPI_write_cmos_sensor( 0x17, 0x40 );
		JX507MIPI_write_cmos_sensor( 0x18, 0xD9 );
		JX507MIPI_write_cmos_sensor( 0x19, 0x29 );
		JX507MIPI_write_cmos_sensor( 0x38, 0xE9 );
		JX507MIPI_write_cmos_sensor( 0x39, 0x20 );
		JX507MIPI_write_cmos_sensor( 0x4a, 0x03 );
		JX507MIPI_write_cmos_sensor( 0x49, 0x10 );
		JX507MIPI_write_cmos_sensor( 0x50, 0x10 );
		JX507MIPI_write_cmos_sensor( 0x1D, 0x00 );
		JX507MIPI_write_cmos_sensor( 0x1E, 0x00 );
		JX507MIPI_write_cmos_sensor( 0x68, 0x00 );
		JX507MIPI_write_cmos_sensor( 0x6C, 0x00 );
		JX507MIPI_write_cmos_sensor( 0x73, 0x33 );
		JX507MIPI_write_cmos_sensor( 0x70, 0x69 );
		JX507MIPI_write_cmos_sensor( 0x76, 0x40 );
		JX507MIPI_write_cmos_sensor( 0x77, 0x06 );
		JX507MIPI_write_cmos_sensor( 0x6D, 0x09 );
		JX507MIPI_write_cmos_sensor( 0x74, 0x02 );
		#ifdef _VAVG_ON //min@20140212
		#ifdef _SENSOR_INITIAL_FLIP
		JX507MIPI_write_cmos_sensor( 0x12, 0x33 );
		#else
		JX507MIPI_write_cmos_sensor( 0x12, 0x03 );
		#endif
		#else //_VAVG_OFF
		JX507MIPI_write_cmos_sensor( 0x12, 0x01 );
		#endif
	#endif //_JX507_INI_20140211
	
	if( ReEnteyCamera == KAL_TRUE ) {
		
	}
	else {
		
	}
	ReEnteyCamera = KAL_FALSE;
	
	SENSORDB( "<< JX507MIPI_Sensor_1M()\n" );
}

static void JX507MIPI_Sensor_5M(void)
{
	SENSORDB( ">> JX507MIPI_Sensor_5M()\n" );
	
	if( ReEnteyCamera == KAL_TRUE ) {
		
	}
	else {
		
	}
	
	#ifdef _JX507_INI_20140820 //Terra20140820-01
		#ifdef _SENSOR_INITIAL_FLIP
		JX507MIPI_write_cmos_sensor( 0x12, 0x70 );
		#else
		JX507MIPI_write_cmos_sensor( 0x12, 0x40 );
		#endif
		JX507MIPI_write_cmos_sensor( 0x1D, 0x00 );
		JX507MIPI_write_cmos_sensor( 0x1E, 0x00 );
		JX507MIPI_write_cmos_sensor( 0x6C, 0x40 );
		JX507MIPI_write_cmos_sensor( 0x73, 0x33 );
		JX507MIPI_write_cmos_sensor( 0x70, 0x68 );
		JX507MIPI_write_cmos_sensor( 0x76, 0xA8 );
		JX507MIPI_write_cmos_sensor( 0x77, 0x0C );
		JX507MIPI_write_cmos_sensor( 0x74, 0x02 );
		JX507MIPI_write_cmos_sensor( 0x78, 0x16 );
		JX507MIPI_write_cmos_sensor( 0x0D, 0x50 );
		JX507MIPI_write_cmos_sensor( 0x0E, 0x10 );
		JX507MIPI_write_cmos_sensor( 0x0F, 0x04 );
		JX507MIPI_write_cmos_sensor( 0x10, 0x13 );
		JX507MIPI_write_cmos_sensor( 0x11, 0x80 );
		JX507MIPI_write_cmos_sensor( 0x20, 0xE6 );
		JX507MIPI_write_cmos_sensor( 0x21, 0x05 );
		JX507MIPI_write_cmos_sensor( 0x22, 0xD6 );
		JX507MIPI_write_cmos_sensor( 0x23, 0x07 );
		JX507MIPI_write_cmos_sensor( 0x24, 0x10 );
		JX507MIPI_write_cmos_sensor( 0x25, 0x98 );
		JX507MIPI_write_cmos_sensor( 0x26, 0x75 );
		JX507MIPI_write_cmos_sensor( 0x27, 0xE4 );
		JX507MIPI_write_cmos_sensor( 0x28, 0x0D );
		JX507MIPI_write_cmos_sensor( 0x29, 0x00 );
		JX507MIPI_write_cmos_sensor( 0x2A, 0xD9 );
		JX507MIPI_write_cmos_sensor( 0x2B, 0x28 );
		JX507MIPI_write_cmos_sensor( 0x2C, 0x02 );
		JX507MIPI_write_cmos_sensor( 0x2D, 0x00 );
		JX507MIPI_write_cmos_sensor( 0x2E, 0xEB );
		JX507MIPI_write_cmos_sensor( 0x2F, 0x04 );
		JX507MIPI_write_cmos_sensor( 0x30, 0x8E );
		JX507MIPI_write_cmos_sensor( 0x31, 0x0A );
		JX507MIPI_write_cmos_sensor( 0x32, 0x9F );
		JX507MIPI_write_cmos_sensor( 0x33, 0x0C );
		JX507MIPI_write_cmos_sensor( 0x34, 0x1F );
		JX507MIPI_write_cmos_sensor( 0x35, 0xB6 );
		JX507MIPI_write_cmos_sensor( 0x5F, 0x43 );
		JX507MIPI_write_cmos_sensor( 0x60, 0x27 );
		JX507MIPI_write_cmos_sensor( 0x61, 0xF8 );
		JX507MIPI_write_cmos_sensor( 0x62, 0x00 );
		JX507MIPI_write_cmos_sensor( 0x63, 0xC0 );
		JX507MIPI_write_cmos_sensor( 0x64, 0x07 );
		JX507MIPI_write_cmos_sensor( 0x67, 0x79 );
		JX507MIPI_write_cmos_sensor( 0x6A, 0x3A );
		JX507MIPI_write_cmos_sensor( 0x69, 0x74 );
		JX507MIPI_write_cmos_sensor( 0x13, 0x81 );
		JX507MIPI_write_cmos_sensor( 0x14, 0x80 );
		JX507MIPI_write_cmos_sensor( 0x16, 0xC0 );
		JX507MIPI_write_cmos_sensor( 0x17, 0x40 );
		JX507MIPI_write_cmos_sensor( 0x18, 0x2C );
		JX507MIPI_write_cmos_sensor( 0x19, 0x21 );
		JX507MIPI_write_cmos_sensor( 0x38, 0xE7 );
		JX507MIPI_write_cmos_sensor( 0x39, 0x34 );
		JX507MIPI_write_cmos_sensor( 0x4a, 0x03 );
		JX507MIPI_write_cmos_sensor( 0x49, 0x10 );
		JX507MIPI_write_cmos_sensor( 0x50, 0x10 );
		JX507MIPI_write_cmos_sensor( 0x6D, 0x0A );
		JX507MIPI_write_cmos_sensor( 0x71, 0x6A );
		#ifdef _SENSOR_INITIAL_FLIP
		JX507MIPI_write_cmos_sensor( 0x12, 0x30 );
		#else
		JX507MIPI_write_cmos_sensor( 0x12, 0x00 );
		#endif
	#endif //_JX507_INI_20140820
	////////////////////////////////////////////////////////////////////////////
	#ifdef _JX507_INI_20140211
		#ifdef _SENSOR_INITIAL_FLIP
		JX507MIPI_write_cmos_sensor( 0x12, 0x70 );
		#else
		JX507MIPI_write_cmos_sensor( 0x12, 0x40 );
		#endif
		JX507MIPI_write_cmos_sensor( 0x0D, 0x00 );
		JX507MIPI_write_cmos_sensor( 0x0E, 0x10 );
		JX507MIPI_write_cmos_sensor( 0x0F, 0x04 );
		JX507MIPI_write_cmos_sensor( 0x10, 0x13 );
		JX507MIPI_write_cmos_sensor( 0x11, 0x80 );
		JX507MIPI_write_cmos_sensor( 0x20, 0xA8 );
		JX507MIPI_write_cmos_sensor( 0x21, 0x0B );
		JX507MIPI_write_cmos_sensor( 0x22, 0xF6 );
		JX507MIPI_write_cmos_sensor( 0x23, 0x07 );
		JX507MIPI_write_cmos_sensor( 0x24, 0x20 );
		JX507MIPI_write_cmos_sensor( 0x25, 0x98 );
		JX507MIPI_write_cmos_sensor( 0x26, 0x7A );
		JX507MIPI_write_cmos_sensor( 0x27, 0x9D );
///		JX507MIPI_write_cmos_sensor( 0x27, 0x99 );
		JX507MIPI_write_cmos_sensor( 0x28, 0x19 );
///		JX507MIPI_write_cmos_sensor( 0x28, 0x0D );
		JX507MIPI_write_cmos_sensor( 0x29, 0x01 );
		JX507MIPI_write_cmos_sensor( 0x2A, 0x80 );
		JX507MIPI_write_cmos_sensor( 0x2B, 0x29 );
		JX507MIPI_write_cmos_sensor( 0x2C, 0x01 );
///		JX507MIPI_write_cmos_sensor( 0x2C, 0x00 );
		JX507MIPI_write_cmos_sensor( 0x2D, 0x00 );
		JX507MIPI_write_cmos_sensor( 0x2E, 0xEA );
		JX507MIPI_write_cmos_sensor( 0x2F, 0x04 );
		JX507MIPI_write_cmos_sensor( 0x31, 0x14 );
		JX507MIPI_write_cmos_sensor( 0x32, 0xBE );
		JX507MIPI_write_cmos_sensor( 0x33, 0x18 );
		JX507MIPI_write_cmos_sensor( 0x34, 0x3E );
		JX507MIPI_write_cmos_sensor( 0x35, 0xEC );
		#ifdef _DRIVE_OPT //min@20140331
		JX507MIPI_write_cmos_sensor( 0x5F, 0x43 );
		#else
		JX507MIPI_write_cmos_sensor( 0x5F, 0x03 );
		#endif
		JX507MIPI_write_cmos_sensor( 0x60, 0x27 );
		JX507MIPI_write_cmos_sensor( 0x61, 0xFC );
		JX507MIPI_write_cmos_sensor( 0x62, 0x00 );
		JX507MIPI_write_cmos_sensor( 0x63, 0xC0 );
		JX507MIPI_write_cmos_sensor( 0x64, 0x07 );
		JX507MIPI_write_cmos_sensor( 0x67, 0x79 );
		JX507MIPI_write_cmos_sensor( 0x69, 0x74 );
		JX507MIPI_write_cmos_sensor( 0x6A, 0x3A );
		JX507MIPI_write_cmos_sensor( 0x13, 0x81 );
		JX507MIPI_write_cmos_sensor( 0x14, 0x80 );
		JX507MIPI_write_cmos_sensor( 0x16, 0xC0 );
		JX507MIPI_write_cmos_sensor( 0x17, 0x40 );
		JX507MIPI_write_cmos_sensor( 0x18, 0x31 );
		JX507MIPI_write_cmos_sensor( 0x19, 0x29 );
		JX507MIPI_write_cmos_sensor( 0x38, 0xE7 );
		JX507MIPI_write_cmos_sensor( 0x39, 0x34 );
		JX507MIPI_write_cmos_sensor( 0x4a, 0x03 );
		JX507MIPI_write_cmos_sensor( 0x49, 0x10 );
		JX507MIPI_write_cmos_sensor( 0x50, 0x10 );
		JX507MIPI_write_cmos_sensor( 0x1D, 0x00 );
		JX507MIPI_write_cmos_sensor( 0x1E, 0x00 );
		JX507MIPI_write_cmos_sensor( 0x68, 0x00 );
		JX507MIPI_write_cmos_sensor( 0x6C, 0x00 );
		JX507MIPI_write_cmos_sensor( 0x73, 0x33 );
		JX507MIPI_write_cmos_sensor( 0x70, 0x69 );
		JX507MIPI_write_cmos_sensor( 0x76, 0xA8 );
		JX507MIPI_write_cmos_sensor( 0x77, 0x0C );
		JX507MIPI_write_cmos_sensor( 0x6D, 0x09 );
		JX507MIPI_write_cmos_sensor( 0x74, 0x02 );
		#ifdef _SENSOR_INITIAL_FLIP
		JX507MIPI_write_cmos_sensor( 0x12, 0x30 );
		#else
		JX507MIPI_write_cmos_sensor( 0x12, 0x00 );
		#endif
	#endif //_JX507_INI_20140211
	
	if( ReEnteyCamera == KAL_TRUE ) {
		
	}
	else {
		
	}
	ReEnteyCamera = KAL_FALSE;
	
	SENSORDB( "<< JX507MIPI_Sensor_5M()\n" );
}

////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////

/*************************************************************************
* FUNCTION
*   JX507MIPIOpen
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
UINT32 JX507MIPIOpen(void)
{
	volatile signed int i = 0;
	kal_uint16 sensor_id = 0;
	
	SENSORDB( ">> JX507MIPIOpen()\n" );
	
///	mt_set_gpio_mode( GPIO_CAMERA_CMPDN1_PIN, GPIO_MODE_00 );
///	mt_set_gpio_dir(  GPIO_CAMERA_CMPDN1_PIN, GPIO_DIR_OUT );
///	mt_set_gpio_out(  GPIO_CAMERA_CMPDN1_PIN, GPIO_OUT_ONE );
	
///	JX507MIPI_write_cmos_sensor( 0x12, 0x80 );				// Reset sensor
///	mDELAY( 2 );
	
	// check if sensor ID correct
	#ifdef _DUAL_I2C_ID //min@20140630
	gSensor_WRITE_ID = JX507MIPI_WRITE_ID;					//min@20140630
	sensor_id = ((UINT32)JX507MIPI_read_cmos_sensor( 0x0A ) << 8) | ((UINT32)JX507MIPI_read_cmos_sensor( 0x0B ));
	SENSORDB( "sensor_id=0x%04X\n", sensor_id );
	if( sensor_id != JX507MIPI_SENSOR_ID )
	#endif
	{
		#ifdef _DUAL_I2C_ID //min@20140630
		gSensor_WRITE_ID = JX507MIPI_WRITE_ID2;
		mdelay( 5 );
		#endif
		sensor_id = ((UINT32)JX507MIPI_read_cmos_sensor( 0x0A ) << 8) | ((UINT32)JX507MIPI_read_cmos_sensor( 0x0B ));
		SENSORDB( "sensor_id=0x%04X\n", sensor_id );
		if( sensor_id != JX507MIPI_SENSOR_ID )
		{
			return ERROR_SENSOR_CONNECT_FAIL;
		}
	}
	
	spin_lock( &jx507mipi_drv_lock );
	jx507.sensorMode		= SENSOR_MODE_PREVIEW;			//min@20140302
///	jx507.sensorMode		= SENSOR_MODE_INIT;
	jx507.AutoFlickerMode	= KAL_FALSE;
	jx507.VideoMode			= KAL_FALSE;
	spin_unlock( &jx507mipi_drv_lock );
	
	JX507MIPI_Sensor_Init();
	
	spin_lock( &jx507mipi_drv_lock );
	jx507.DummyLines  = 0;
	jx507.DummyPixels = 0;
	
	#if defined(_JX507_INI_20140820)
	jx507.pvPclk	= 62400;
	jx507.videoPclk	= 62400;
	#endif
	#if defined(_JX507_INI_20140211)
	jx507.pvPclk	= 91200;
	jx507.videoPclk	= 91200;
	#endif
	
	spin_unlock( &jx507mipi_drv_lock );
	
	switch( JX507MIPICurrentScenarioId )
	{
	case MSDK_SCENARIO_ID_CAMERA_ZSD:
		#if defined(ZSD15FPS)
		spin_lock( &jx507mipi_drv_lock );
		jx507.capPclk = 91200;
		spin_unlock( &jx507mipi_drv_lock );
		#else
		spin_lock( &jx507mipi_drv_lock );
		jx507.capPclk = 91200;
		spin_unlock( &jx507mipi_drv_lock );
		#endif
		break;
	default:
		spin_lock( &jx507mipi_drv_lock );
		jx507.capPclk = 91200;
		spin_unlock( &jx507mipi_drv_lock );
		break;
	}
	
	spin_lock( &jx507mipi_drv_lock );
	jx507.shutter			= 0x4EA;
	jx507.pvShutter			= 0x4EA;
	jx507.maxExposureLines	= JX507MIPI_PV_PERIOD_LINE_NUMS - 4;
	jx507.ispBaseGain		= BASEGAIN;						//0x40
	jx507.sensorGlobalGain	= 0x00;	///0x1f;				//sensor gain
	jx507.pvGain			= 0x00;	///0x1f;
	jx507.realGain			= JX507MIPIReg2Gain( 0x00 );
	///jx507.realGain		= JX507MIPIReg2Gain( 0x1f );	//ispBaseGain as 1x
	spin_unlock( &jx507mipi_drv_lock );
	
	printk( "Ver : %s\n", _SW_VER  );
	printk( "Code: %s\n", _SW_CODE );
	printk( "Hint: %s\n", _SW_HINT );
	
	SENSORDB( "<< JX507MIPIOpen()\n" );
	return ERROR_NONE;
}

/*************************************************************************
* FUNCTION
*   JX507MIPIGetSensorID
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
extern bool camera_pdn_reverse;
UINT32 JX507MIPIGetSensorID(UINT32 *sensorID)
{
	int  retry = 1;

	SENSORDB( ">> JX507MIPIGetSensorID()\n" );	
	mt_set_gpio_mode(GPIO_CAMERA_CMPDN_PIN,GPIO_CAMERA_CMPDN_PIN_M_GPIO);
	   mt_set_gpio_dir(GPIO_CAMERA_CMPDN_PIN,GPIO_DIR_OUT);
	   mt_set_gpio_out(GPIO_CAMERA_CMPDN_PIN,GPIO_OUT_ZERO);
	   msleep(10);
	   
	///JX507MIPI_write_cmos_sensor( 0x12, 0x80 );			// Reset sensor
	///mDELAY(10);
	
	// check if sensor ID correct
	do {
		*sensorID = ((JX507MIPI_read_cmos_sensor( 0x0A ) << 8) | JX507MIPI_read_cmos_sensor( 0x0B ));
		#ifdef JX507MIPI_DRIVER_TRACE
		SENSORDB( "Sensor ID=0x%X\n", *sensorID );
		#endif
		if( *sensorID == JX507MIPI_SENSOR_ID ) {
			SENSORDB( "JX507MIPI: Sensor ID=0x%04X\n", *sensorID );
			break;
		}
		SENSORDB( "Read Sensor ID Fail: 0x%04X\n", *sensorID );
		retry--;
	} while( retry > 0 );
	
	if( *sensorID != JX507MIPI_SENSOR_ID ) {
		#ifdef _DUAL_I2C_ID //min@20140630
		gSensor_WRITE_ID = JX507MIPI_WRITE_ID2;
		mdelay( 5 );
		do {
			*sensorID = ((JX507MIPI_read_cmos_sensor( 0x0A ) << 8) | JX507MIPI_read_cmos_sensor( 0x0B ));
			#ifdef JX507MIPI_DRIVER_TRACE
			SENSORDB( "Sensor ID=0x%X\n", *sensorID );
			#endif
			if( *sensorID == JX507MIPI_SENSOR_ID ) {
				SENSORDB( "JX507MIPI: Sensor ID=0x%04X\n", *sensorID );
				break;
			}
			SENSORDB( "Read Sensor ID Fail: 0x%04X\n", *sensorID );
			retry--;
		} while( retry > 0 );
		
		if( *sensorID != JX507MIPI_SENSOR_ID ) {
			*sensorID = 0xFFFFFFFF;
		    SENSORDB( "<< JX507MIPIGetSensorID(): ERR\n" );
			return ERROR_SENSOR_CONNECT_FAIL;
		}
		#else
		*sensorID = 0xFFFFFFFF;
	    SENSORDB( "<< JX507MIPIGetSensorID(): ERR\n" );
		return ERROR_SENSOR_CONNECT_FAIL;
		#endif
	}
	SENSORDB( "<< JX507MIPIGetSensorID(): OK\n" );
	
	camera_pdn_reverse = 1;
	return ERROR_NONE;
}

/*************************************************************************
* FUNCTION
*   JX507MIPIClose
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
UINT32 JX507MIPIClose(void)
{
#ifdef JX507MIPI_DRIVER_TRACE
	SENSORDB( "JX507MIPIClose()\n" );
#endif
	//CISModulePowerOn( FALSE );
	//DRV_I2CClose( JX507MIPIhDrvI2C );
	JX507MIPI_write_cmos_sensor( 0x12, 0x40 );
    JX507MIPI_write_cmos_sensor( 0x6c, 0x80 );

	ReEnteyCamera = KAL_FALSE;
	return ERROR_NONE;
}

/*************************************************************************
* FUNCTION
*   JX507MIPIPreview
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
#if 0
UINT32 JX507MIPIPreview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window, MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	SENSORDB( ">> JX507MIPIPreview()\n" );
	
	// preview size
	if( jx507.sensorMode == SENSOR_MODE_PREVIEW ) {
		// do nothing
		// FOR CCT PREVIEW
		JX507MIPI_Sensor_1M();
	}
	else {
		JX507MIPI_Sensor_1M();
	}
	spin_lock( &jx507mipi_drv_lock );
	jx507.sensorMode  = SENSOR_MODE_PREVIEW;	// Need set preview setting after capture mode
	jx507.DummyPixels = 0;						//define dummy pixels and lines
	jx507.DummyLines  = 0;
	JX507MIPI_FeatureControl_PERIOD_PixelNum = JX507MIPI_PV_PERIOD_PIXEL_NUMS + jx507.DummyPixels;
	JX507MIPI_FeatureControl_PERIOD_LineNum	 = JX507MIPI_PV_PERIOD_LINE_NUMS  + jx507.DummyLines;
	spin_unlock( &jx507mipi_drv_lock );
	
	//JX507MIPI_Write_Shutter( jx507.shutter );
	//write_JX507_gain( jx507.pvGain );
	
	// set mirror & flip
	//SENSORDB( "[JX507MIPIPreview] mirror&flip: %d \n", sensor_config_data->SensorImageMirror );
	spin_lock( &jx507mipi_drv_lock );
	jx507.imgMirror = sensor_config_data->SensorImageMirror;
	spin_unlock( &jx507mipi_drv_lock );
	
	JX507MIPISetFlipMirror( IMAGE_NORMAL );					//min@20131224 fix
	//JX507MIPISetFlipMirror( IMAGE_HV_MIRROR );
	////JX507MIPISetFlipMirror( sensor_config_data->SensorImageMirror );
	mDELAY( 40 );
	
	SENSORDB( "<< JX507MIPIPreview()\n" );
	return ERROR_NONE;
}
#else
UINT32 JX507MIPIPreview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window, MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	kal_uint16 dummy_line = 0;
	
	SENSORDB( ">> JX507MIPIPreview()\n" );
	
	JX507MIPI_Sensor_1M();
	
	spin_lock( &jx507mipi_drv_lock );
	//min@20140302 begin: add
	jx507.sensorMode  = SENSOR_MODE_PREVIEW;	// Need set preview setting after capture mode
	jx507.DummyPixels = 0;
	jx507.DummyLines  = 0;
	//min@20140302 end
	jx507.pv_mode	= KAL_TRUE;
	jx507.video_mode = KAL_FALSE;
	spin_unlock( &jx507mipi_drv_lock );
	//JX507MIPIZsdCameraPreview = KAL_FALSE;
	
	//JX507MIPI_set_mirror( sensor_config_data->SensorImageMirror );
	spin_lock( &jx507mipi_drv_lock );
	jx507.line_length	= JX507MIPI_PV_PERIOD_PIXEL_NUMS;
	jx507.frame_height	= JX507MIPI_PV_PERIOD_LINE_NUMS + dummy_line;
	jx507.pclk			= JX507MIPI_PREVIEW_CLK;
	spin_unlock( &jx507mipi_drv_lock );
	
	JX507MIPI_Set_Dummy( 0, dummy_line );					/* modify dummy_pixel must gen AE table again */
	JX507MIPI_Write_Shutter( jx507.shutter );				//min@20140302
	msleep( 40 );
	
	//PRINT_K( "[soso][JX507MIPIPreview]shutter=%x,shutter=%d\n", jx507.shutter, jx507.shutter );
	
	SENSORDB( "<< JX507MIPIPreview()\n" );
	return ERROR_NONE;
}
#endif

#if 0
UINT32 JX507MIPIVideo(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window, MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	SENSORDB( ">> JX507MIPIVideo()\n" );
	
	if( jx507.sensorMode == SENSOR_MODE_VIDEO ) {
		// do nothing
	}
	else {
		#ifdef _JX507_VIDEO_720P //min@20140226
			JX507MIPI_Sensor_1M();
		#else //1080P
			JX507MIPI_Sensor_1080P();
		#endif
	}
	
	spin_lock( &jx507mipi_drv_lock );
	jx507.sensorMode = SENSOR_MODE_VIDEO;
	JX507MIPI_FeatureControl_PERIOD_PixelNum = JX507MIPI_VIDEO_PERIOD_PIXEL_NUMS + jx507.DummyPixels;
	JX507MIPI_FeatureControl_PERIOD_LineNum	 = JX507MIPI_VIDEO_PERIOD_LINE_NUMS  + jx507.DummyLines;
	spin_unlock( &jx507mipi_drv_lock );
	
	spin_lock( &jx507mipi_drv_lock );
	jx507.imgMirror = sensor_config_data->SensorImageMirror;
	spin_unlock( &jx507mipi_drv_lock );
	
	JX507MIPISetFlipMirror( IMAGE_NORMAL );					//min@20131224 fix
	//JX507MIPISetFlipMirror( IMAGE_HV_MIRROR );
	////JX507MIPISetFlipMirror( sensor_config_data->SensorImageMirror );
	mDELAY( 10 );	///mDELAY( 40 );
	
	SENSORDB( "<< JX507MIPIVideo()\n" );
	return ERROR_NONE;
}
#else
UINT32 JX507MIPIVideo(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window, MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	kal_uint16 dummy_line = 0;
	
	SENSORDB( ">> JX507MIPIVideo()\n" );
	
	#ifdef _JX507_VIDEO_720P //min@20140226
		#ifdef _VIDEO_FAST_INIT //min@20140409
		// Do nothing...
		#else
		JX507MIPI_Sensor_1M();
		#endif
	#else //1080P
		JX507MIPI_Sensor_1080P();
	#endif
	
	//min@20140302 begin: add
	spin_lock( &jx507mipi_drv_lock );
	jx507.sensorMode  = SENSOR_MODE_VIDEO;
	#ifndef _VIDEO_FAST_INIT //min@20140409
	jx507.DummyPixels = 0;
	jx507.DummyLines  = 0;
	#endif
	spin_unlock( &jx507mipi_drv_lock );
	//min@20140302 end
	
	jx507.video_mode	= KAL_TRUE;
	jx507.pv_mode		= KAL_FALSE;
	#ifndef _VIDEO_FAST_INIT //min@20140409
	jx507.line_length	= JX507MIPI_VIDEO_PERIOD_PIXEL_NUMS;
	jx507.frame_height	= JX507MIPI_VIDEO_PERIOD_LINE_NUMS + dummy_line;
	#endif
	jx507.pclk			= JX507MIPI_VIDEO_CLK;
	
	#ifndef _VIDEO_FAST_INIT //min@20140409
	JX507MIPI_Set_Dummy( 0, dummy_line );					/* modify dummy_pixel must gen AE table again */
	JX507MIPI_Write_Shutter( jx507.shutter );				//min@20140302
	msleep( 10 );		///msleep( 40 );					//min@20140324 test
	#endif
	
	//PRINT_K( "[soso][JX507MIPIPreview]shutter=%x,shutter=%d\n", jx507.shutter, jx507.shutter );
	
	SENSORDB( "<< JX507MIPIVideo()\n" );
	return ERROR_NONE;
}
#endif

#if 0
UINT32 JX507MIPICapture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window, MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
 	kal_uint32 shutter = jx507.shutter;
	kal_uint32 temp_data;
	//kal_uint32 pv_line_length, cap_line_length,
	
	SENSORDB( ">> JX507MIPICapture()\n" );
	
	if( SENSOR_MODE_CAPTURE == jx507.sensorMode ) {
		SENSORDB( "JX507MIPICapture BusrtShot!!!\n" );
	}
	else {
		// Record Preview shutter & gain
		shutter		= JX507MIPI_read_shutter();
		temp_data	= read_JX507_gain();
		spin_lock( &jx507mipi_drv_lock );
		jx507.pvShutter			= shutter;
		jx507.sensorGlobalGain	= temp_data;
		jx507.pvGain			= jx507.sensorGlobalGain;
		jx507.sensorMode		= SENSOR_MODE_CAPTURE;
		spin_unlock( &jx507mipi_drv_lock );
		
		SENSORDB( "jx507.shutter=%d, read_pv_shutter=%d, read_pv_gain = 0x%x\n", jx507.shutter, shutter,jx507.sensorGlobalGain );
		
		// Full size setting
		JX507MIPI_Sensor_5M();
		
		msleep( 10 );	///msleep( 40 );						//min@20140324 test
		
		// rewrite pixel number to Register ,for mt6589 line start/end;
		JX507MIPI_Set_Dummy( jx507.DummyPixels, jx507.DummyLines );
		
		spin_lock( &jx507mipi_drv_lock );
		jx507.imgMirror		= sensor_config_data->SensorImageMirror;
		jx507.DummyPixels	= 0;//define dummy pixels and lines
		jx507.DummyLines	= 0;
		JX507MIPI_FeatureControl_PERIOD_PixelNum = JX507MIPI_FULL_PERIOD_PIXEL_NUMS + jx507.DummyPixels;
		JX507MIPI_FeatureControl_PERIOD_LineNum	 = JX507MIPI_FULL_PERIOD_LINE_NUMS  + jx507.DummyLines;
		spin_unlock( &jx507mipi_drv_lock );
		
		//SENSORDB("[JX507MIPICapture] mirror&flip: %d\n",sensor_config_data->SensorImageMirror);
		JX507MIPISetFlipMirror( IMAGE_NORMAL );				//min@20131224 fix
		//JX507MIPISetFlipMirror( IMAGE_HV_MIRROR );
		////JX507MIPISetFlipMirror( sensor_config_data->SensorImageMirror );
		
		//#if defined(MT6575)||defined(MT6577)
		if( JX507MIPICurrentScenarioId == MSDK_SCENARIO_ID_CAMERA_ZSD )
		{
			SENSORDB( "JX507MIPICapture exit ZSD!!\n" );
			return ERROR_NONE;
		}
		//#endif
		
		#if 0 //no need to calculate shutter from mt6589
		// calculate shutter
		pv_line_length  = JX507MIPI_PV_PERIOD_PIXEL_NUMS   + jx507.DummyPixels;
		cap_line_length = JX507MIPI_FULL_PERIOD_PIXEL_NUMS + jx507.DummyPixels;
		SENSORDB( "[JX507MIPICapture]pv_line_length =%d,cap_line_length =%d\n", pv_line_length, cap_line_length );
		SENSORDB( "[JX507MIPICapture]pv_shutter =%d\n",shutter );
		
		shutter = shutter * pv_line_length / cap_line_length;
		shutter = shutter * jx507.capPclk  / jx507.pvPclk;
		shutter *= 2; //preview bining///////////////////////////////////////
		
		if( shutter < 3 )
			shutter = 3;
		JX507MIPI_Write_Shutter( shutter );
		gain = read_JX507_gain();
		
		SENSORDB( "cap_shutter=%d, cap_read gain=0x%X\n", shutter, read_JX507_gain() );
		//write_JX507_gain( jx507.sensorGlobalGain );
		#endif
	}
	
	SENSORDB( "<< JX507MIPICapture()\n" );
	return ERROR_NONE;
}
#else
UINT32 JX507MIPICapture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window, MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	kal_uint16 cap_fps;
	
	SENSORDB( ">> JX507MIPICapture()\n" );
	
	spin_lock( &jx507mipi_drv_lock );
	//min@20140302 begin: add
	jx507.sensorMode  = SENSOR_MODE_CAPTURE;
	jx507.DummyPixels = 0;
	jx507.DummyLines  = 0;
	//min@20140302 end
	jx507.video_mode		 = KAL_FALSE;
	JX507MIPIAutoFlicKerMode = KAL_FALSE;
	spin_unlock( &jx507mipi_drv_lock );
	
	JX507MIPI_Sensor_5M();
	
	spin_lock( &jx507mipi_drv_lock );
	jx507.pv_mode	= KAL_FALSE;
	jx507.pclk		= JX507MIPI_CAPTURE_CLK;
	spin_unlock( &jx507mipi_drv_lock );
	
	cap_fps = JX507MIPI_FPS( 15 );
	
	JX507MIPI_Set_Dummy( 0, 0 );
	spin_lock( &jx507mipi_drv_lock );
	jx507.line_length  = JX507MIPI_FULL_PERIOD_PIXEL_NUMS;
	jx507.frame_height = JX507MIPI_FULL_PERIOD_LINE_NUMS;
	spin_unlock( &jx507mipi_drv_lock );
	
///	mdelay( 10 );	///mdelay( 40 );	//min@20140326 comment out //min@20140324 test
	
	SENSORDB( "<< JX507MIPICapture()\n" );
	return ERROR_NONE;
}
#endif

UINT32 JX507MIPIGetResolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *pSensorResolution)
{
	SENSORDB("JX507MIPIGetResolution!!\n");
	
	pSensorResolution->SensorPreviewWidth	= JX507MIPI_IMAGE_SENSOR_PV_WIDTH;
	pSensorResolution->SensorPreviewHeight	= JX507MIPI_IMAGE_SENSOR_PV_HEIGHT;
	
	pSensorResolution->SensorFullWidth		= JX507MIPI_IMAGE_SENSOR_FULL_WIDTH;
	pSensorResolution->SensorFullHeight		= JX507MIPI_IMAGE_SENSOR_FULL_HEIGHT;
	
	pSensorResolution->SensorVideoWidth		= JX507MIPI_IMAGE_SENSOR_VIDEO_WIDTH;
	pSensorResolution->SensorVideoHeight	= JX507MIPI_IMAGE_SENSOR_VIDEO_HEIGHT;
//	SENSORDB( "SensorPreviewWidth:  %d.\n", pSensorResolution->SensorPreviewWidth  );
//	SENSORDB( "SensorPreviewHeight: %d.\n", pSensorResolution->SensorPreviewHeight );
//	SENSORDB( "SensorFullWidth:     %d.\n", pSensorResolution->SensorFullWidth     );
//	SENSORDB( "SensorFullHeight:    %d.\n", pSensorResolution->SensorFullHeight    );
	return ERROR_NONE;
}   /* JX507MIPIGetResolution() */

UINT32 JX507MIPIGetInfo(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_INFO_STRUCT *pSensorInfo, MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
	pSensorInfo->SensorPreviewResolutionX = JX507MIPI_IMAGE_SENSOR_PV_WIDTH;
	pSensorInfo->SensorPreviewResolutionY = JX507MIPI_IMAGE_SENSOR_PV_HEIGHT;
	
	pSensorInfo->SensorFullResolutionX = JX507MIPI_IMAGE_SENSOR_FULL_WIDTH;
	pSensorInfo->SensorFullResolutionY = JX507MIPI_IMAGE_SENSOR_FULL_HEIGHT;
	
	spin_lock( &jx507mipi_drv_lock );
	jx507.imgMirror = pSensorConfigData->SensorImageMirror;
	spin_unlock( &jx507mipi_drv_lock );
	
	#if defined(_JX507_INI_20140820)
		#ifdef _SENSOR_INITIAL_FLIP
		pSensorInfo->SensorOutputDataFormat	= SENSOR_OUTPUT_FORMAT_RAW_Gr;
		#else
		pSensorInfo->SensorOutputDataFormat	= SENSOR_OUTPUT_FORMAT_RAW_Gb;
		#endif
	#else
		#ifdef _SENSOR_INITIAL_FLIP
		pSensorInfo->SensorOutputDataFormat	= SENSOR_OUTPUT_FORMAT_RAW_R;
		#else
		pSensorInfo->SensorOutputDataFormat	= SENSOR_OUTPUT_FORMAT_RAW_B;
		#endif
	#endif
	pSensorInfo->SensorClockPolarity		= SENSOR_CLOCK_POLARITY_LOW;
	pSensorInfo->SensorClockFallingPolarity	= SENSOR_CLOCK_POLARITY_LOW;
	pSensorInfo->SensorHsyncPolarity		= SENSOR_CLOCK_POLARITY_LOW;
	pSensorInfo->SensorVsyncPolarity		= SENSOR_CLOCK_POLARITY_LOW;
	
	pSensorInfo->SensroInterfaceType		= SENSOR_INTERFACE_TYPE_MIPI;
	
	//min@20140114 modify ->1
	//min@20140324 modify ->2
	pSensorInfo->CaptureDelayFrame	= 2;	//1;
	pSensorInfo->PreviewDelayFrame	= 2;	//1;
	pSensorInfo->VideoDelayFrame	= 2;
	
	pSensorInfo->SensorDrivingCurrent	= ISP_DRIVING_8MA;///ISP_DRIVING_2MA;
	//min@20140114 modify
	pSensorInfo->AEShutDelayFrame		= 0;				/* The frame of setting shutter default 0 for TG int */
	pSensorInfo->AESensorGainDelayFrame	= 1;				/* The frame of setting sensor gain */
	pSensorInfo->AEISPGainDelayFrame	= 2;
	
	switch( ScenarioId )
	{
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			pSensorInfo->SensorClockFreq		= 24;	///26;
			pSensorInfo->SensorClockRisingCount	= 0;
			
			pSensorInfo->SensorGrabStartX = JX507MIPI_PV_X_START;
			pSensorInfo->SensorGrabStartY = JX507MIPI_PV_Y_START;
			
			pSensorInfo->SensorMIPILaneNumber					  = SENSOR_MIPI_2_LANE;
			pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount	  = 0;
			pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 14;
			pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount	  = 0;
			pSensorInfo->SensorPacketECCOrder					  = 1;
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			pSensorInfo->SensorClockFreq		= 24;	///26;
			pSensorInfo->SensorClockRisingCount	= 0;
			
			pSensorInfo->SensorGrabStartX = JX507MIPI_VIDEO_X_START;
			pSensorInfo->SensorGrabStartY = JX507MIPI_VIDEO_Y_START;
			
			pSensorInfo->SensorMIPILaneNumber					  = SENSOR_MIPI_2_LANE;
			pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount	  = 0;
		 	pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 14;
			pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount	  = 0;
			pSensorInfo->SensorPacketECCOrder					  = 1;
			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
			pSensorInfo->SensorClockFreq		= 24;	///26;
			pSensorInfo->SensorClockRisingCount	= 0;
			
			pSensorInfo->SensorGrabStartX = JX507MIPI_FULL_X_START;	//2*JX507MIPI_IMAGE_SENSOR_PV_STARTX;
			pSensorInfo->SensorGrabStartY = JX507MIPI_FULL_Y_START;	//2*JX507MIPI_IMAGE_SENSOR_PV_STARTY;
			
			pSensorInfo->SensorMIPILaneNumber					  = SENSOR_MIPI_2_LANE;
			pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount	  = 0;
			pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 14;
			pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount	  = 0;
			pSensorInfo->SensorPacketECCOrder					  = 1;
			break;
		default:
			pSensorInfo->SensorClockFreq		= 24;	///26;
			pSensorInfo->SensorClockRisingCount	= 0;
			
			pSensorInfo->SensorGrabStartX = JX507MIPI_PV_X_START;
			pSensorInfo->SensorGrabStartY = JX507MIPI_PV_Y_START;
			
			pSensorInfo->SensorMIPILaneNumber					  = SENSOR_MIPI_2_LANE;
			pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount	  = 0;
		 	pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 14;
			pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount	  = 0;
			pSensorInfo->SensorPacketECCOrder					  = 1;
			break;
	}
	
	memcpy( pSensorConfigData, &JX507SensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT) );
	
	return ERROR_NONE;
}

UINT32 JX507MIPIControl(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *pImageWindow, MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
	spin_lock( &jx507mipi_drv_lock );
	JX507MIPICurrentScenarioId = ScenarioId;
	spin_unlock( &jx507mipi_drv_lock );
	
	//SENSORDB( "ScenarioId=%d\n", ScenarioId );
	SENSORDB( "JX507CurrentScenarioId=%d\n", JX507MIPICurrentScenarioId );
	
	switch( ScenarioId )
	{
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			JX507MIPIPreview( pImageWindow, pSensorConfigData );
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			JX507MIPIVideo( pImageWindow, pSensorConfigData );
			//JX507MIPIPreview( pImageWindow, pSensorConfigData );
			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
			JX507MIPICapture( pImageWindow, pSensorConfigData );
			break;
		default:
			return ERROR_INVALID_SCENARIO_ID;
	}
	return ERROR_NONE;
}

UINT32 JX507MIPISetVideoMode(UINT16 u2FrameRate)
{
	kal_uint32 MIN_Frame_length = 0, frameRate = 0, extralines = 0;
	
	SENSORDB( ">> JX507MIPISetVideoMode(): frame rate=%d\n", u2FrameRate );
	
	spin_lock( &jx507mipi_drv_lock );
	VIDEO_MODE_TARGET_FPS = u2FrameRate;
	spin_unlock( &jx507mipi_drv_lock );
	
	if( u2FrameRate == 0 )
	{
		SENSORDB("Disable Video Mode or dynimac fps\n");
		return KAL_TRUE;
	}
	if( u2FrameRate > 30 || u2FrameRate < 5 )
		SENSORDB( "error frame rate setting\n" );
	
	if( SENSOR_MODE_VIDEO == jx507.sensorMode ) { //min@20140411
		jx507.AutoFlickerMode = KAL_FALSE;
		return KAL_TRUE;
	}
	
	if( jx507.sensorMode == SENSOR_MODE_VIDEO ) //video ScenarioId recording
	{
		if( jx507.AutoFlickerMode == KAL_TRUE )
		{
			if( u2FrameRate == 30 )
				frameRate = 306;
			else if( u2FrameRate == 15 )
				frameRate = 148;
			else
				frameRate = u2FrameRate*10;
			
			MIN_Frame_length = (jx507.videoPclk*10000) / (JX507MIPI_VIDEO_PERIOD_PIXEL_NUMS) / frameRate;			//min@20140408
		}
		else {
			MIN_Frame_length = (jx507.videoPclk*10000) / (JX507MIPI_VIDEO_PERIOD_PIXEL_NUMS) / u2FrameRate / 10;	//min@20140408
		}
		PRINT_K( "videoPclk=%d, PIXEL_NUMS=%d, u2FrameRate=%d\n", jx507.videoPclk, JX507MIPI_VIDEO_PERIOD_PIXEL_NUMS, u2FrameRate );
		PRINT_K( "[JX507] MIN_Frame_length=%d\n", MIN_Frame_length );
		
		if( (MIN_Frame_length <= JX507MIPI_VIDEO_PERIOD_LINE_NUMS) )
		{
			MIN_Frame_length = JX507MIPI_VIDEO_PERIOD_LINE_NUMS;
		}
		PRINT_K( "[JX507] max MIN_Frame_length=%d\n", MIN_Frame_length );
	
		extralines = MIN_Frame_length - JX507MIPI_VIDEO_PERIOD_LINE_NUMS;	//min@20140408
		
		spin_lock( &jx507mipi_drv_lock );
		jx507.DummyPixels = 0;
		jx507.DummyLines  = extralines;					//min@20140408
		spin_unlock( &jx507mipi_drv_lock );
		
		JX507MIPI_Set_Dummy( 0, extralines );			//min@20140408
	}
	else if( jx507.sensorMode == SENSOR_MODE_CAPTURE )
	{
		SENSORDB( "-------[JX507MIPISetVideoMode]ZSD???---------\n" );
		
		if( jx507.AutoFlickerMode == KAL_TRUE ) {
			#if defined(ZSD15FPS)
			if( u2FrameRate == 15 )
				frameRate = 148;
			#else
			if( u2FrameRate == 13 )
				frameRate = 130;
			#endif
			else
				frameRate = u2FrameRate*10;
			
			MIN_Frame_length = (jx507.capPclk*10000) / (JX507MIPI_FULL_PERIOD_PIXEL_NUMS + jx507.DummyPixels) / frameRate;			//min@20140408
		}
		else {
			MIN_Frame_length = (jx507.capPclk*10000) / (JX507MIPI_FULL_PERIOD_PIXEL_NUMS + jx507.DummyPixels) / u2FrameRate / 10;	//min@20140408
		}
		PRINT_K( "capPclk=%d, PIXEL_NUMS=%d, u2FrameRate=%d\n", jx507.videoPclk, JX507MIPI_VIDEO_PERIOD_PIXEL_NUMS, u2FrameRate );
		PRINT_K( "[JX507] MIN_Frame_length=%d\n", MIN_Frame_length );
		
		if( (MIN_Frame_length <= JX507MIPI_FULL_PERIOD_LINE_NUMS) )
		{
			MIN_Frame_length = JX507MIPI_FULL_PERIOD_LINE_NUMS;
			SENSORDB( "current fps = %d\n", (jx507.capPclk * 10000) / (JX507MIPI_FULL_PERIOD_PIXEL_NUMS) / JX507MIPI_FULL_PERIOD_LINE_NUMS );
		}
		PRINT_K( "[JX507] max MIN_Frame_length=%d\n", MIN_Frame_length );
		
		SENSORDB( "current fps (10 base)= %d\n", (jx507.pvPclk*10000)*10 / (JX507MIPI_FULL_PERIOD_PIXEL_NUMS + jx507.DummyPixels) / MIN_Frame_length );
		
		extralines = MIN_Frame_length - JX507MIPI_FULL_PERIOD_LINE_NUMS;	//min@20140408
		
		spin_lock( &jx507mipi_drv_lock );
		jx507.DummyPixels = 0;
		jx507.DummyLines  = extralines;						//min@20140408
		spin_unlock( &jx507mipi_drv_lock );
		
		JX507MIPI_Set_Dummy( 0, extralines );				//min@20140408
	}
	SENSORDB( "MIN_Frame_length=%d,jx507.DummyLines=%d\n", MIN_Frame_length, jx507.DummyLines );
	
	SENSORDB( "<< JX507MIPISetVideoMode()\n" );
	return KAL_TRUE;
}

UINT32 JX507SetAutoFlickerMode(kal_bool bEnable, UINT16 u2FrameRate)
{
	SENSORDB( ">> JX507SetAutoFlickerMode(): frame rate(10base)= %d, %d\n", bEnable, u2FrameRate );
	
	if( bEnable ) { // enable auto flicker
		spin_lock( &jx507mipi_drv_lock );
		jx507.AutoFlickerMode = KAL_TRUE;
		spin_unlock( &jx507mipi_drv_lock );
		SENSORDB( "Enable Auto flicker\n" );
	}
	else {
		spin_lock( &jx507mipi_drv_lock );
		jx507.AutoFlickerMode = KAL_FALSE;
		spin_unlock( &jx507mipi_drv_lock );
		SENSORDB( "Disable Auto flicker\n" );
	}
	
	SENSORDB( "<< JX507SetAutoFlickerMode()\n" );
	
	///return TRUE;						//min@20140408
	return ERROR_NONE;
}

UINT32 JX507SetTestPatternMode(kal_bool bEnable)
{
	SENSORDB("[JX507SetTestPatternMode] Test pattern enable:%d\n", bEnable);
	
	return TRUE;
}

UINT32 JX507MIPISetMaxFramerateByScenario(MSDK_SCENARIO_ID_ENUM scenarioId, MUINT32 frameRate) 
{
	kal_uint32	pclk;
	kal_int16	dummyLine = 0;
	kal_uint16	lineLength, frameHeight;
	
	SENSORDB( "JX507MIPISetMaxFramerateByScenario(): scenarioId=%d, frame rate=%d\n", scenarioId, frameRate );
	
	switch( scenarioId )
	{
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			#if defined(_JX507_INI_20140820)
			pclk = 62400000;
			#endif
			#if defined(_JX507_INI_20140211)
			pclk = 91200000;
			#endif
			lineLength	= JX507MIPI_PV_PERIOD_PIXEL_NUMS;
			frameHeight	= (10 * pclk)/frameRate/lineLength;
			///frameHeight	= pclk / frameRate / lineLength;	//min@20140307
			PRINT_K( "[JX507] lineLength=%d, frameHeight=%d\n", lineLength, frameHeight );
			
			spin_lock( &jx507mipi_drv_lock );
			jx507.sensorMode = SENSOR_MODE_PREVIEW;
			spin_unlock( &jx507mipi_drv_lock );			
			
			JX507MIPI_Set_Dummy( 0, frameHeight );	//min@20140408 //min@20140307
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			PRINT_K( "@MSDK_SCENARIO_ID_VIDEO_PREVIEW\n" );
			#if defined(_JX507_INI_20140820)
			pclk = 62400000;
			#endif
			#if defined(_JX507_INI_20140211)
			pclk = 91200000;
			#endif
			lineLength	= JX507MIPI_VIDEO_PERIOD_PIXEL_NUMS;
			frameHeight	= (10 * pclk)/frameRate/lineLength;
			///frameHeight	= pclk / frameRate / lineLength;	//min@20140307
			PRINT_K( "[JX507] lineLength=%d, frameHeight=%d\n", lineLength, frameHeight );
			
			spin_lock( &jx507mipi_drv_lock );
			jx507.sensorMode = SENSOR_MODE_VIDEO;
			spin_unlock( &jx507mipi_drv_lock );
			
			JX507MIPI_Set_Dummy( 0, frameHeight );	//min@20140408 //min@20140307
			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
			pclk = 91200000;
			lineLength	= JX507MIPI_FULL_PERIOD_PIXEL_NUMS;
			frameHeight	= (10 * pclk)/frameRate/lineLength;
			///frameHeight	= pclk / frameRate / lineLength;	//min@20140307
			PRINT_K( "[JX507] lineLength=%d, frameHeight=%d\n", lineLength, frameHeight );
			
			spin_lock( &jx507mipi_drv_lock );
			jx507.sensorMode = SENSOR_MODE_CAPTURE;
			spin_unlock( &jx507mipi_drv_lock );
			
			JX507MIPI_Set_Dummy( 0, frameHeight );	//min@20140408 //min@20140307
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

UINT32 JX507MIPIGetDefaultFramerateByScenario(MSDK_SCENARIO_ID_ENUM scenarioId, MUINT32 *pframeRate) 
{
	switch( scenarioId )
	{
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			*pframeRate = 300;
			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
			*pframeRate = 150;
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

UINT32 JX507MIPIFeatureControl(MSDK_SENSOR_FEATURE_ENUM FeatureId, UINT8 *pFeaturePara,UINT32 *pFeatureParaLen)
{
	UINT16 *pFeatureReturnPara16	= (UINT16 *) pFeaturePara;
	UINT16 *pFeatureData16			= (UINT16 *) pFeaturePara;
	UINT32 *pFeatureReturnPara32	= (UINT32 *) pFeaturePara;
	UINT32 *pFeatureData32			= (UINT32 *) pFeaturePara;
	UINT32 SensorRegNumber;
	UINT32 i;
	PNVRAM_SENSOR_DATA_STRUCT	  pSensorDefaultData = (PNVRAM_SENSOR_DATA_STRUCT) pFeaturePara;
	MSDK_SENSOR_CONFIG_STRUCT	  *pSensorConfigData = (MSDK_SENSOR_CONFIG_STRUCT *) pFeaturePara;
	MSDK_SENSOR_REG_INFO_STRUCT	  *pSensorRegData	 = (MSDK_SENSOR_REG_INFO_STRUCT *) pFeaturePara;
	MSDK_SENSOR_GROUP_INFO_STRUCT *pSensorGroupInfo	 = (MSDK_SENSOR_GROUP_INFO_STRUCT *) pFeaturePara;
	MSDK_SENSOR_ITEM_INFO_STRUCT  *pSensorItemInfo	 = (MSDK_SENSOR_ITEM_INFO_STRUCT *) pFeaturePara;
	MSDK_SENSOR_ENG_INFO_STRUCT   *pSensorEngInfo	 = (MSDK_SENSOR_ENG_INFO_STRUCT *) pFeaturePara;
	
	switch( FeatureId )
	{
		case SENSOR_FEATURE_GET_RESOLUTION:
			*pFeatureReturnPara16++= JX507MIPI_IMAGE_SENSOR_FULL_WIDTH;
			*pFeatureReturnPara16 = JX507MIPI_IMAGE_SENSOR_FULL_HEIGHT;
			*pFeatureParaLen = 4;
			break;
		case SENSOR_FEATURE_GET_PERIOD:
			*pFeatureReturnPara16++= JX507MIPI_FeatureControl_PERIOD_PixelNum;
			*pFeatureReturnPara16 = JX507MIPI_FeatureControl_PERIOD_LineNum;
			*pFeatureParaLen = 4;
			break;
		case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
			switch( JX507MIPICurrentScenarioId )
			{
				case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
					*pFeatureReturnPara32 = JX507MIPI_PREVIEW_CLK;
					*pFeatureParaLen = 4;
					break;
				case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
					*pFeatureReturnPara32 = JX507MIPI_VIDEO_CLK;
					*pFeatureParaLen = 4;
					break;
				case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
				case MSDK_SCENARIO_ID_CAMERA_ZSD:
					*pFeatureReturnPara32 = JX507MIPI_CAPTURE_CLK;
					*pFeatureParaLen = 4;
					break;
				default:
					*pFeatureReturnPara32 = JX507MIPI_PREVIEW_CLK;
					*pFeatureParaLen = 4;
					break;
			}
			break;
		case SENSOR_FEATURE_SET_ESHUTTER:
			PRINT_K( "@SENSOR_FEATURE_SET_ESHUTTER: Exp=0x%X (%d)\n", *pFeatureData16, *pFeatureData16 );
			JX507MIPI_SetShutter( *pFeatureData16 );
			break;
		case SENSOR_FEATURE_SET_NIGHTMODE:
			JX507MIPI_NightMode( (BOOL) *pFeatureData16 );
			break;
		case SENSOR_FEATURE_SET_GAIN:
			PRINT_K( "@SENSOR_FEATURE_SET_GAIN: Gain=0x%X (%d)\n", *pFeatureData16, *pFeatureData16 );
			JX507MIPI_SetGain( (UINT16) *pFeatureData16 );
			break;
		case SENSOR_FEATURE_SET_FLASHLIGHT:
			break;
		case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
			//JX507MIPI_isp_master_clock = *pFeatureData32;
			break;
		case SENSOR_FEATURE_SET_REGISTER:
		///	PRINT_K( "@SENSOR_FEATURE_SET_REGISTER: Reg[0x%X]=0x%X\n", pSensorRegData->RegAddr, pSensorRegData->RegData );
			JX507MIPI_write_cmos_sensor( pSensorRegData->RegAddr, pSensorRegData->RegData );
			break;
		case SENSOR_FEATURE_GET_REGISTER:
		///	PRINT_K( "@SENSOR_FEATURE_GET_REGISTER: Reg[0x%X]\n", pSensorRegData->RegAddr );
			pSensorRegData->RegData = JX507MIPI_read_cmos_sensor( pSensorRegData->RegAddr );
		///	PRINT_K( " SENSOR_FEATURE_GET_REGISTER: Reg[0x%X]=0x%X\n", pSensorRegData->RegAddr, pSensorRegData->RegData );
			break;
		case SENSOR_FEATURE_SET_CCT_REGISTER:
			SensorRegNumber = JX507MIPI_FACTORY_END_ADDR;
			for( i=0; i<SensorRegNumber; i++ )
			{
				spin_lock( &jx507mipi_drv_lock );
				JX507SensorCCT[i].Addr=*pFeatureData32++;
				JX507SensorCCT[i].Para=*pFeatureData32++;
				spin_unlock( &jx507mipi_drv_lock );
			}
			break;
		case SENSOR_FEATURE_GET_CCT_REGISTER:
			SensorRegNumber = JX507MIPI_FACTORY_END_ADDR;
			if( *pFeatureParaLen < (SensorRegNumber*sizeof(SENSOR_REG_STRUCT)+4) )
				return FALSE;
			*pFeatureData32++=SensorRegNumber;
			for( i=0; i<SensorRegNumber; i++ )
			{
				*pFeatureData32++=JX507SensorCCT[i].Addr;
				*pFeatureData32++=JX507SensorCCT[i].Para;
			}
			break;
		case SENSOR_FEATURE_SET_ENG_REGISTER:
			SensorRegNumber = JX507MIPI_ENGINEER_END;
			for( i=0; i<SensorRegNumber; i++ )
			{
				spin_lock( &jx507mipi_drv_lock );
				JX507SensorReg[i].Addr=*pFeatureData32++;
				JX507SensorReg[i].Para=*pFeatureData32++;
				spin_unlock( &jx507mipi_drv_lock );
			}
			break;
		case SENSOR_FEATURE_GET_ENG_REGISTER:
			SensorRegNumber = JX507MIPI_ENGINEER_END;
			if( *pFeatureParaLen < (SensorRegNumber*sizeof(SENSOR_REG_STRUCT)+4) )
				return FALSE;
			*pFeatureData32++=SensorRegNumber;
			for( i=0; i<SensorRegNumber; i++ )
			{
				*pFeatureData32++=JX507SensorReg[i].Addr;
				*pFeatureData32++=JX507SensorReg[i].Para;
			}
			break;
		case SENSOR_FEATURE_GET_REGISTER_DEFAULT:
			if( *pFeatureParaLen >= sizeof(NVRAM_SENSOR_DATA_STRUCT) )
			{
				pSensorDefaultData->Version	 = NVRAM_CAMERA_SENSOR_FILE_VERSION;
				pSensorDefaultData->SensorId = JX507MIPI_SENSOR_ID;
				memcpy( pSensorDefaultData->SensorEngReg, JX507SensorReg, sizeof(SENSOR_REG_STRUCT) * JX507MIPI_ENGINEER_END );
				memcpy( pSensorDefaultData->SensorCCTReg, JX507SensorCCT, sizeof(SENSOR_REG_STRUCT) * JX507MIPI_FACTORY_END_ADDR );
			}
			else
				return FALSE;
			*pFeatureParaLen = sizeof(NVRAM_SENSOR_DATA_STRUCT);
			break;
		case SENSOR_FEATURE_GET_CONFIG_PARA:
			memcpy( pSensorConfigData, &JX507SensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT) );
			*pFeatureParaLen = sizeof(MSDK_SENSOR_CONFIG_STRUCT);
			break;
		case SENSOR_FEATURE_CAMERA_PARA_TO_SENSOR:
			JX507MIPI_camera_para_to_sensor();
			break;
		case SENSOR_FEATURE_SENSOR_TO_CAMERA_PARA:
			JX507MIPI_sensor_to_camera_para();
			break;
		case SENSOR_FEATURE_GET_GROUP_COUNT:
			*pFeatureReturnPara32++=JX507MIPI_get_sensor_group_count();
			*pFeatureParaLen = 4;
			break;
		case SENSOR_FEATURE_GET_GROUP_INFO:
			JX507MIPI_get_sensor_group_info( pSensorGroupInfo->GroupIdx, pSensorGroupInfo->GroupNamePtr, &pSensorGroupInfo->ItemCount );
			*pFeatureParaLen = sizeof(MSDK_SENSOR_GROUP_INFO_STRUCT);
			break;
		case SENSOR_FEATURE_GET_ITEM_INFO:
			JX507MIPI_get_sensor_item_info( pSensorItemInfo->GroupIdx,pSensorItemInfo->ItemIdx, pSensorItemInfo );
			*pFeatureParaLen = sizeof(MSDK_SENSOR_ITEM_INFO_STRUCT);
			break;
		case SENSOR_FEATURE_SET_ITEM_INFO:
			JX507MIPI_set_sensor_item_info( pSensorItemInfo->GroupIdx, pSensorItemInfo->ItemIdx, pSensorItemInfo->ItemValue );
			*pFeatureParaLen = sizeof(MSDK_SENSOR_ITEM_INFO_STRUCT);
			break;
		case SENSOR_FEATURE_GET_ENG_INFO:
			pSensorEngInfo->SensorId			   = 129;
			pSensorEngInfo->SensorType			   = CMOS_SENSOR;
			#if defined(_JX507_INI_20140820)
				#ifdef _SENSOR_INITIAL_FLIP
				pSensorEngInfo->SensorOutputDataFormat = SENSOR_OUTPUT_FORMAT_RAW_Gr;
				#else
				pSensorEngInfo->SensorOutputDataFormat = SENSOR_OUTPUT_FORMAT_RAW_Gb;
				#endif
			#else
				#ifdef _SENSOR_INITIAL_FLIP
				pSensorEngInfo->SensorOutputDataFormat = SENSOR_OUTPUT_FORMAT_RAW_R;
				#else
				pSensorEngInfo->SensorOutputDataFormat = SENSOR_OUTPUT_FORMAT_RAW_B;
				#endif
			#endif
			*pFeatureParaLen = sizeof(MSDK_SENSOR_ENG_INFO_STRUCT);
			break;
		case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
			// get the lens driver ID from EEPROM or just return LENS_DRIVER_ID_DO_NOT_CARE
			// if EEPROM does not exist in camera module.
			*pFeatureReturnPara32 = LENS_DRIVER_ID_DO_NOT_CARE;
			*pFeatureParaLen = 4;
			break;
		case SENSOR_FEATURE_INITIALIZE_AF:
			break;
		case SENSOR_FEATURE_CONSTANT_AF:
			break;
		case SENSOR_FEATURE_MOVE_FOCUS_LENS:
			break;
		case SENSOR_FEATURE_SET_VIDEO_MODE:
			JX507MIPISetVideoMode( *pFeatureData16 );
			break;
		case SENSOR_FEATURE_CHECK_SENSOR_ID:
			JX507MIPIGetSensorID( pFeatureReturnPara32 );
			break;
		case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE:
			JX507SetAutoFlickerMode( (BOOL)*pFeatureData16, *(pFeatureData16+1) );
			break;
		case SENSOR_FEATURE_SET_TEST_PATTERN:
			JX507SetTestPatternMode( (BOOL)*pFeatureData16 );
			break;
		case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
			JX507MIPISetMaxFramerateByScenario( (MSDK_SCENARIO_ID_ENUM)*pFeatureData32, *(pFeatureData32+1) );
			break;
		case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
			JX507MIPIGetDefaultFramerateByScenario( (MSDK_SCENARIO_ID_ENUM)*pFeatureData32, (MUINT32 *)(*(pFeatureData32+1)) );
			break;
		default:
			break;
	}
	return ERROR_NONE;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////

SENSOR_FUNCTION_STRUCT	SensorFuncJX507MIPI = {
	JX507MIPIOpen,
	JX507MIPIGetInfo,
	JX507MIPIGetResolution,
	JX507MIPIFeatureControl,
	JX507MIPIControl,
	JX507MIPIClose
};

UINT32 JX507MIPISensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
	/* To Do : Check Sensor status here */
	if( pfFunc != NULL )
		*pfFunc = &SensorFuncJX507MIPI;
	
	return ERROR_NONE;
}   /* SensorInit() */
