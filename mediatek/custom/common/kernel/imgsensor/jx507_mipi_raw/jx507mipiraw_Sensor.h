/*******************************************************************************************/


/*******************************************************************************************/

/* SENSOR FULL SIZE */
#ifndef _JX507MIPI_SENSOR_H
#define _JX507MIPI_SENSOR_H


#define		SW_VER		"JX507_MTK6582_MIPI2L_A38_base_tsf0717a_6x_6fps_PV62M4_HalfClk_20140822r1"


#define		_JX507_INI_20140820								// Preview @62.4MHz, Capture @HalfHclk //min@20140821 Terra20140820-01
//#define	_JX507_INI_20140211								// Preview @91.2MHz


#define		_DUAL_I2C_ID									//min@20140630


#define		_JX507_VIDEO_720P								//min@20140226


#define		JX507MIPI_FACTORY_START_ADDR		0
#define		JX507MIPI_ENGINEER_START_ADDR		10
#define		FACTORY_START_ADDR					0
#define		ENGINEER_START_ADDR					10


#define		ZSD15FPS
//#define	MIPI_INTERFACE

typedef enum JX507MIPI_group_enum {
	JX507MIPI_PRE_GAIN = 0,
	JX507MIPI_CMMCLK_CURRENT,
	JX507MIPI_FRAME_RATE_LIMITATION,
	JX507MIPI_REGISTER_EDITOR,
	JX507MIPI_GROUP_TOTAL_NUMS
} JX507MIPI_FACTORY_GROUP_ENUM;

typedef enum JX507MIPI_engineer_index {
	JX507MIPI_CMMCLK_CURRENT_INDEX	= JX507MIPI_ENGINEER_START_ADDR,
	JX507MIPI_ENGINEER_END
} JX507MIPI_FACTORY_ENGINEER_INDEX;

typedef enum JX507MIPI_register_index {
	JX507MIPI_SENSOR_BASEGAIN		= JX507MIPI_FACTORY_START_ADDR,
	JX507MIPI_PRE_GAIN_R_INDEX,
	JX507MIPI_PRE_GAIN_Gr_INDEX,
	JX507MIPI_PRE_GAIN_Gb_INDEX,
	JX507MIPI_PRE_GAIN_B_INDEX,
	JX507MIPI_FACTORY_END_ADDR
} JX507MIPI_FACTORY_REGISTER_INDEX;

typedef struct _sensor_data_struct {
	SENSOR_REG_STRUCT	Reg[JX507MIPI_ENGINEER_END];
	SENSOR_REG_STRUCT	CCT[JX507MIPI_FACTORY_END_ADDR];
} SENSOR_DATA_STRUCT, *PSENSOR_DATA_STRUCT;

typedef enum {
	SENSOR_MODE_INIT = 0,
	SENSOR_MODE_PREVIEW,
	SENSOR_MODE_VIDEO,
	SENSOR_MODE_CAPTURE
} JX507MIPI_SENSOR_MODE;

typedef struct {
	kal_uint32	DummyPixels;
	kal_uint32	DummyLines;
	
	kal_uint32	pvShutter;
	kal_uint32	pvGain;
	
	kal_uint32	pvPclk;					// x10 480 for 48MHZ
	kal_uint32	videoPclk;
	kal_uint32	capPclk;				// x10
	
	kal_uint32	shutter;
	kal_uint32	maxExposureLines;
	
	kal_uint16	sensorGlobalGain;		//sensor gain read from 0x350a 0x350b;
	kal_uint16	ispBaseGain;			//64
	kal_uint16	realGain;				//ispBaseGain as 1x
	
	kal_int16	imgMirror;
	
	JX507MIPI_SENSOR_MODE	sensorMode;
	
	kal_bool	AutoFlickerMode;
	kal_bool	VideoMode;
#if 1
	MSDK_SENSOR_CONFIG_STRUCT	cfg_data;
	SENSOR_DATA_STRUCT			eng;			/* engineer mode */
	MSDK_SENSOR_ENG_INFO_STRUCT	eng_info;
	kal_uint8					mirror;
	kal_bool					pv_mode;
	kal_bool					video_mode;
	kal_bool					NightMode;
	kal_uint16					normal_fps;		/* video normal mode max fps */
	kal_uint16					night_fps;		/* video night mode max fps */
	kal_uint16					FixedFps;
///	kal_uint16					shutter;
	kal_uint16					gain;
	kal_uint32					pclk;
	kal_uint16					frame_height;
	kal_uint16					frame_height_BackUp;
	kal_uint16					line_length; 
#endif
} JX507MIPI_PARA_STRUCT, *PJX507MIPI_PARA_STRUCT;


////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////

/* SENSOR PREVIEW/CAPTURE VT CLOCK */
#if defined(_JX507_INI_20140820)
	#define	JX507MIPI_PREVIEW_CLK				62400000
	#define	JX507MIPI_CAPTURE_CLK				91200000
	#ifdef _JX507_VIDEO_720P
	#define	JX507MIPI_VIDEO_CLK					62400000
	#else //1080P
	#define	JX507MIPI_VIDEO_CLK					91200000
	#endif
	
	#define	JX507MIPI_COLOR_FORMAT				SENSOR_OUTPUT_FORMAT_RAW_Gb
#endif
#if defined(_JX507_INI_20140211)
	#define	JX507MIPI_PREVIEW_CLK				91200000
	#define	JX507MIPI_CAPTURE_CLK				91200000
	#define	JX507MIPI_VIDEO_CLK					91200000
	
	#define	JX507MIPI_COLOR_FORMAT				SENSOR_OUTPUT_FORMAT_RAW_B
#endif

#define	JX507MIPI_MIN_ANALOG_GAIN				1
#define	JX507MIPI_MAX_ANALOG_GAIN				8		///16
#define	JX507MIPI_ANALOG_GAIN_1X				0x00	///(0x0020)

//#define	JX507MIPI_MAX_DIGITAL_GAIN			8
//#define	JX507MIPI_MIN_DIGITAL_GAIN			1
//#define	JX507MIPI_DIGITAL_GAIN_1X			0x00	///(0x0100)


/* FRAME RATE UNIT */
#define JX507MIPI_FPS(x)						(10 * (x))


/* SENSOR SCALER FACTOR */
#define	JX507MIPI_PV_SCALER_FACTOR				3
#define	JX507MIPI_FULL_SCALER_FACTOR			1


////////////////////////////////////////////////////////////////////////////////////////////////////
#if defined(_JX507_INI_20140820)
/* SENSOR PIXEL/LINE NUMBERS IN ONE PERIOD */
// CAPTURE
#define JX507MIPI_FULL_PERIOD_PIXEL_NUMS		3032	//2984		/* 15 fps */
#if defined(ZSD15FPS)
#define	JX507MIPI_FULL_PERIOD_LINE_NUMS			2006	//2038
#else
#define	JX507MIPI_FULL_PERIOD_LINE_NUMS			2006	//2038
#endif
// PREVIEW
#define JX507MIPI_PV_PERIOD_PIXEL_NUMS			1930	//1928		/* 30 fps */
#define JX507MIPI_PV_PERIOD_LINE_NUMS			1078	//1578
// VIDEO
#ifdef _JX507_VIDEO_720P
#define JX507MIPI_VIDEO_PERIOD_PIXEL_NUMS		1930	//1928		/* 30 fps */
#define JX507MIPI_VIDEO_PERIOD_LINE_NUMS		1078	//1578
#else //1080P
#define JX507MIPI_VIDEO_PERIOD_PIXEL_NUMS 		2360	//2312		/* 30 fps */
#define JX507MIPI_VIDEO_PERIOD_LINE_NUMS		1290	//1316
#endif

/* SENSOR START/END POSITION */
// CAPTURE
#define	JX507MIPI_FULL_X_START					4				///(2)
#define	JX507MIPI_FULL_Y_START					4				///(2)
#define	JX507MIPI_FULL_X_END					(2560 - 32)		///(3264+200)
#define	JX507MIPI_FULL_Y_END					(1920 - 24)		///(2448) 
#define	JX507MIPI_IMAGE_SENSOR_FULL_WIDTH		(2560 - 32)						///(3264-64)
#define	JX507MIPI_IMAGE_SENSOR_FULL_HEIGHT		(1920 - 24)						///(2448-48)
// PREVIEW
#define	JX507MIPI_PV_X_START					2				///(2)
#define	JX507MIPI_PV_Y_START					2				///(2)
#define	JX507MIPI_PV_X_END						(1280 - 16)		///(1632) 
#define	JX507MIPI_PV_Y_END						( 960 - 12)		///(1224) 
#define JX507MIPI_IMAGE_SENSOR_PV_WIDTH			(1280 - 16)						///(1632-32)
#define JX507MIPI_IMAGE_SENSOR_PV_HEIGHT		( 960 - 12)						///(1224-24)
// VIDEO
#ifdef _JX507_VIDEO_720P
#define	JX507MIPI_VIDEO_X_START					2				///(2)
#define	JX507MIPI_VIDEO_Y_START					2				///(2)
#define	JX507MIPI_VIDEO_X_END					(1280 - 16)		///(1632) 
#define	JX507MIPI_VIDEO_Y_END					( 960 - 12)		///(1224) 
#define JX507MIPI_IMAGE_SENSOR_VIDEO_WIDTH		(1280 - 16)						///(1632-32)
#define JX507MIPI_IMAGE_SENSOR_VIDEO_HEIGHT		( 960 - 12)						///(1224-24)
#else //1080P
#define	JX507MIPI_VIDEO_X_START					2				///(2)
#define	JX507MIPI_VIDEO_Y_START					2				///(2)
#define	JX507MIPI_VIDEO_X_END 					(1920 - 16)		///(2160)
#define	JX507MIPI_VIDEO_Y_END 					(1080 - 12)		///(1620)
#define JX507MIPI_IMAGE_SENSOR_VIDEO_WIDTH		(1920 - 16)						///(2160-40)
#define JX507MIPI_IMAGE_SENSOR_VIDEO_HEIGHT		(1080 - 12)						///(1620-30)
#endif

#define	JX507MIPI_MIN_LINE_LENGTH				3032	///2984
#define	JX507MIPI_MIN_FRAME_LENGTH				2006	///2038
#endif //_JX507_INI_20140820

////////////////////////////////////////////////////////////////////////////////////////////////////
#if defined(_JX507_INI_20140211)
/* SENSOR PIXEL/LINE NUMBERS IN ONE PERIOD */
// CAPTURE
#define JX507MIPI_FULL_PERIOD_PIXEL_NUMS		2984		/* 15 fps */	//mt6589 add dummy pixel 200 for line_start/end
#if defined(ZSD15FPS)
#define	JX507MIPI_FULL_PERIOD_LINE_NUMS			2038
#else
#define	JX507MIPI_FULL_PERIOD_LINE_NUMS			2038
#endif
// PREVIEW
#define JX507MIPI_PV_PERIOD_PIXEL_NUMS			1928		/* 30 fps */
#define JX507MIPI_PV_PERIOD_LINE_NUMS			1578
// VIDEO
#ifdef _JX507_VIDEO_720P //min@20140306 fix
#define JX507MIPI_VIDEO_PERIOD_PIXEL_NUMS		1928		/* 30 fps */
#define JX507MIPI_VIDEO_PERIOD_LINE_NUMS		1578
#else //1080P
#define JX507MIPI_VIDEO_PERIOD_PIXEL_NUMS 		2312		/* 30 fps */
#define JX507MIPI_VIDEO_PERIOD_LINE_NUMS		1316
#endif

/* SENSOR START/END POSITION */
// CAPTURE
#define	JX507MIPI_FULL_X_START					4				///(2)
#define	JX507MIPI_FULL_Y_START					4				///(2)
#define	JX507MIPI_FULL_X_END					(2560 - 32)		///(3264+200)
#define	JX507MIPI_FULL_Y_END					(1920 - 24)		///(2448) 
#define	JX507MIPI_IMAGE_SENSOR_FULL_WIDTH		(2560 - 32)		///(3264-64)
#define	JX507MIPI_IMAGE_SENSOR_FULL_HEIGHT		(1920 - 24)		///(2448-48)
// PREVIEW
#define	JX507MIPI_PV_X_START					2				///(2)
#define	JX507MIPI_PV_Y_START					2				///(2)
#define	JX507MIPI_PV_X_END						(1280 - 16)		///(1632) 
#define	JX507MIPI_PV_Y_END						( 960 - 12)		///(1224) 
#define JX507MIPI_IMAGE_SENSOR_PV_WIDTH			(1280 - 16)		///(1632-32)
#define JX507MIPI_IMAGE_SENSOR_PV_HEIGHT		( 960 - 12)		///(1224-24)
// VIDEO
#ifdef _JX507_VIDEO_720P //min@20140226
#define	JX507MIPI_VIDEO_X_START					2				///(2)
#define	JX507MIPI_VIDEO_Y_START					2				///(2)
#define	JX507MIPI_VIDEO_X_END					(1280 - 16)		///(1632) 
#define	JX507MIPI_VIDEO_Y_END					( 960 - 12)		///(1224) 
#define JX507MIPI_IMAGE_SENSOR_VIDEO_WIDTH		(1280 - 16)		///(1632-32)
#define JX507MIPI_IMAGE_SENSOR_VIDEO_HEIGHT		( 960 - 12)		///(1224-24)
#else //1080P
#define	JX507MIPI_VIDEO_X_START					2				///(2)
#define	JX507MIPI_VIDEO_Y_START					2				///(2)
#define	JX507MIPI_VIDEO_X_END 					(1920 - 16)		///(2160)
#define	JX507MIPI_VIDEO_Y_END 					(1080 - 12)		///(1620)
#define JX507MIPI_IMAGE_SENSOR_VIDEO_WIDTH		(1920 - 16)		///(2160-40)
#define JX507MIPI_IMAGE_SENSOR_VIDEO_HEIGHT		(1080 - 12)		///(1620-30)
#endif

#define	JX507MIPI_MIN_LINE_LENGTH				2984
#define	JX507MIPI_MIN_FRAME_LENGTH				2038
#endif //_JX507_INI_20140211


////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////

#define	JX507MIPI_MAX_LINE_LENGTH				0xCCCC
#define	JX507MIPI_MAX_FRAME_LENGTH				0xFFFF

/* DUMMY NEEDS TO BE INSERTED */
/* SETUP TIME NEED TO BE INSERTED */
#define	JX507MIPI_IMAGE_SENSOR_PV_INSERTED_PIXELS			2
#define	JX507MIPI_IMAGE_SENSOR_PV_INSERTED_LINES			2

#define	JX507MIPI_IMAGE_SENSOR_FULL_INSERTED_PIXELS			4
#define	JX507MIPI_IMAGE_SENSOR_FULL_INSERTED_LINES			4


////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////

/* SENSOR READ/WRITE ID */
#ifdef _DUAL_I2C_ID
// for dual camera
#define		JX507MIPI_WRITE_ID					(0x68)
#define		JX507MIPI_READ_ID					(0x69)
#define		JX507MIPI_WRITE_ID2					(0x60)
#define		JX507MIPI_READ_ID2					(0x61)
#else
//min@20131120
#define		JX507MIPI_WRITE_ID					(0x60)
#define		JX507MIPI_READ_ID					(0x61)
#endif

/* SENSOR ID */
//#define	JX507MIPI_SENSOR_ID					(0xA507)
//#define	JX507MIPI_SENSOR_ID					JX507_SENSOR_ID
#define		JX507MIPI_PAGE_SETTING_REG			(0xFF)

#if 0
/* SENSOR PRIVATE STRUCT */
typedef struct JX507MIPI_sensor_STRUCT {
	MSDK_SENSOR_CONFIG_STRUCT	cfg_data;
	SENSOR_DATA_STRUCT			eng;			/* engineer mode */
	MSDK_SENSOR_ENG_INFO_STRUCT	eng_info;
	kal_uint8					mirror;
	kal_bool					pv_mode;
	kal_bool					video_mode;
	kal_bool					NightMode;
	kal_uint16					normal_fps;		/* video normal mode max fps */
	kal_uint16					night_fps;		/* video night mode max fps */
	kal_uint16					FixedFps;
	kal_uint16					shutter;
	kal_uint16					gain;
	kal_uint32					pclk;
	kal_uint16					frame_height;
	kal_uint16					frame_height_BackUp;
	kal_uint16					line_length;  
} JX507MIPI_sensor_struct;
#endif

////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////

//export functions
UINT32 JX507MIPIOpen(void);
UINT32 JX507MIPIControl(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *pImageWindow, MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData);
UINT32 JX507MIPIFeatureControl(MSDK_SENSOR_FEATURE_ENUM FeatureId, UINT8 *pFeaturePara,UINT32 *pFeatureParaLen);
UINT32 JX507MIPIGetInfo(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_INFO_STRUCT *pSensorInfo, MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData);
UINT32 JX507MIPIGetResolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *pSensorResolution);
UINT32 JX507MIPIClose(void);

//#define	Sleep(ms)	mdelay(ms)
//#define	RETAILMSG(x,...)
//#define	TEXT


#endif /* _JX507MIPI_SENSOR_H_ */
