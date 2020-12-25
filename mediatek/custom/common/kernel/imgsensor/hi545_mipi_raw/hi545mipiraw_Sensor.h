/*******************************************************************************************/


/*******************************************************************************************/

/* SENSOR FULL SIZE */
#ifndef __SENSOR_H
#define __SENSOR_H   


typedef enum group_enum {
    PRE_GAIN=0,
    CMMCLK_CURRENT,
    FRAME_RATE_LIMITATION,
    REGISTER_EDITOR,
    GROUP_TOTAL_NUMS
} FACTORY_GROUP_ENUM;


#define ENGINEER_START_ADDR 10
#define FACTORY_START_ADDR 0

typedef enum engineer_index
{
    CMMCLK_CURRENT_INDEX=ENGINEER_START_ADDR,
    ENGINEER_END
} FACTORY_ENGINEER_INDEX;

typedef enum register_index
{
	SENSOR_BASEGAIN=FACTORY_START_ADDR,
	PRE_GAIN_R_INDEX,
	PRE_GAIN_Gr_INDEX,
	PRE_GAIN_Gb_INDEX,
	PRE_GAIN_B_INDEX,
	FACTORY_END_ADDR
} FACTORY_REGISTER_INDEX;

typedef struct
{
    SENSOR_REG_STRUCT	Reg[ENGINEER_END];
    SENSOR_REG_STRUCT	CCT[FACTORY_END_ADDR];
} SENSOR_DATA_STRUCT, *PSENSOR_DATA_STRUCT;

typedef enum {
    SENSOR_MODE_INIT = 0,
    SENSOR_MODE_PREVIEW,
    SENSOR_MODE_VIDEO,
    SENSOR_MODE_CAPTURE
} HI545_SENSOR_MODE;


typedef struct
{
	kal_uint32 DummyPixels;
	kal_uint32 DummyLines;
	
	kal_uint32 pvShutter;
	kal_uint32 pvGain;
	
	kal_uint32 pvPclk;  // x10 480 for 48MHZ
	kal_uint32 videoPclk;
	kal_uint32 capPclk; // x10
	
	kal_uint32 shutter;
	kal_uint32 maxExposureLines;

	kal_uint8 sensorGlobalGain;//sensor gain read from 0x350a 0x350b;
	kal_uint16 ispBaseGain;//64
	kal_uint16 realGain;//ispBaseGain as 1x

	kal_int16 imgMirror;

	HI545_SENSOR_MODE sensorMode;

	kal_bool HI545AutoFlickerMode;
	kal_bool HI545VideoMode;
	
}HI545_PARA_STRUCT,*PHI545_PARA_STRUCT;


	#define MIPI_DELAY_COUNT								14
	#define MAXANOLOGBASEGAIN									8

	#define HI545_IMAGE_SENSOR_FULL_WIDTH					(2560)	
	#define HI545_IMAGE_SENSOR_FULL_HEIGHT					(1920)

	/* SENSOR PV SIZE */
	#define HI545_IMAGE_SENSOR_PV_WIDTH						(1280)
	#define HI545_IMAGE_SENSOR_PV_HEIGHT					(960)

	#define HI545_IMAGE_SENSOR_VIDEO_WIDTH					(1280)
	#define HI545_IMAGE_SENSOR_VIDEO_HEIGHT					(960)
	#define HI545_IMAGE_SENSOR_VIDEO720P_WIDTH					(1280)
	#define HI545_IMAGE_SENSOR_VIDEO720P_HEIGHT 				(720)
	

	/* SENSOR SCALER FACTOR */
	#define HI545_PV_SCALER_FACTOR					    	3
	#define HI545_FULL_SCALER_FACTOR					    1
	                                        	
	/* SENSOR START/EDE POSITION */         	
	#define HI545_FULL_X_START						    		(0)
	#define HI545_FULL_Y_START						    		(0)//(8)
	#define HI545_PV_X_START						    		(0)
	#define HI545_PV_Y_START						    		(0)
	
	#define HI545_VIDEO_X_START								(0)
	#define HI545_VIDEO_Y_START								(0)

	#define HI545_MAX_ANALOG_GAIN					(16)
	#define HI545_MIN_ANALOG_GAIN					(1)
	#define HI545_ANALOG_GAIN_1X						(0x0020)

	//#define HI545_MAX_DIGITAL_GAIN					(8)
	//#define HI545_MIN_DIGITAL_GAIN					(1)
	//#define HI545_DIGITAL_GAIN_1X					(0x0100)

	/* SENSOR PIXEL/LINE NUMBERS IN ONE PERIOD */
	#define HI545_FULL_PERIOD_PIXEL_NUMS					2880  //
	#define HI545_FULL_PERIOD_LINE_NUMS					2017	//
	
	#define HI545_PV_PERIOD_PIXEL_NUMS					2880  //
	#define HI545_PV_PERIOD_LINE_NUMS					2017	//

	#define HI545_VIDEO_PERIOD_PIXEL_NUMS 				2880	//
	#define HI545_VIDEO_PERIOD_LINE_NUMS				2017	//
	

	#define HI545_MIN_LINE_LENGTH						0x0AA4  //
	#define HI545_MIN_FRAME_LENGTH						0x0214  //
	
	#define HI545_MAX_LINE_LENGTH						0xCCCC
	#define HI545_MAX_FRAME_LENGTH						0xFFFF

	/* DUMMY NEEDS TO BE INSERTED */
	/* SETUP TIME NEED TO BE INSERTED */
	#define HI545_IMAGE_SENSOR_PV_INSERTED_PIXELS			2
	#define HI545_IMAGE_SENSOR_PV_INSERTED_LINES			2

	#define HI545_IMAGE_SENSOR_FULL_INSERTED_PIXELS		4
	#define HI545_IMAGE_SENSOR_FULL_INSERTED_LINES		4

#define HI545MIPI_WRITE_ID 	(0x40)
#define HI545MIPI_READ_ID	(0x41)

#define HI545_PV_CLK		176000000
#define HI545_VIDEO_CLK		176000000
#define HI545_CAP_CLK		176000000


// SENSOR CHIP VERSION

#define HI545MIPI_PAGE_SETTING_REG    (0xFF)

//s_add for porting
//s_add for porting
//s_add for porting

//export functions
UINT32 HI545MIPIOpen(void);
UINT32 HI545MIPIGetResolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *pSensorResolution);
UINT32 HI545MIPIGetInfo(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_INFO_STRUCT *pSensorInfo, MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData);
UINT32 HI545MIPIControl(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *pImageWindow, MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData);
UINT32 HI545MIPIFeatureControl(MSDK_SENSOR_FEATURE_ENUM FeatureId, UINT8 *pFeaturePara,UINT32 *pFeatureParaLen);
UINT32 HI545MIPIClose(void);

//#define Sleep(ms) mdelay(ms)
//#define RETAILMSG(x,...)
//#define TEXT

//e_add for porting
//e_add for porting
//e_add for porting

#endif /* __SENSOR_H */

