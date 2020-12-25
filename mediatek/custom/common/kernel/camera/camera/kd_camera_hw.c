#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
#include <linux/xlog.h>
#include <linux/kernel.h>


#include "kd_camera_hw.h"

#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_camera_feature.h"

/******************************************************************************
 * Debug configuration
 ******************************************************************************/
#define PFX "[kd_camera_hw]"
#define PK_DBG_NONE(fmt, arg...)    do {} while (0)
#define PK_DBG_FUNC(fmt, arg...)    xlog_printk(ANDROID_LOG_INFO, PFX , fmt, ##arg)

#define DEBUG_CAMERA_HW_K
#ifdef DEBUG_CAMERA_HW_K
#define PK_DBG PK_DBG_FUNC
#define PK_ERR(fmt, arg...)         xlog_printk(ANDROID_LOG_ERR, PFX , fmt, ##arg)
#define PK_XLOG_INFO(fmt, args...) \
	do {    \
		xlog_printk(ANDROID_LOG_INFO, PFX , fmt, ##arg); \
	} while(0)
#else
#define PK_DBG(a,...)
#define PK_ERR(a,...)
#define PK_XLOG_INFO(fmt, args...)
#endif

unsigned char sensor_index;

bool camera_pdn_reverse = FALSE;
bool camera_pdn1_reverse = FALSE;

kal_bool searchMainSensor = KAL_TRUE;
MT65XX_POWER_VOLTAGE vanzoVcamD2Volt;
MT65XX_POWER_VOLTAGE vanzoVcamDVolt;
MT65XX_POWER_VOLTAGE vanzoVcamSubDVolt;
int kdCISModulePowerOn(CAMERA_DUAL_CAMERA_SENSOR_ENUM SensorIdx, char *currSensorName, BOOL On, char* mode_name)
{
	u32 pinSetIdx = 0;//default main sensor

#define IDX_PS_CMRST 0
#define IDX_PS_CMPDN 4

#define IDX_PS_MODE 1
#define IDX_PS_ON   2
#define IDX_PS_OFF  3

	u32 pinSet[2][8] = {
		//for main sensor
		{GPIO_CAMERA_CMRST_PIN,
			GPIO_CAMERA_CMRST_PIN_M_GPIO,   /* mode */
			GPIO_OUT_ONE,                   /* ON state */
			GPIO_OUT_ZERO,                  /* OFF state */
			GPIO_CAMERA_CMPDN_PIN,
			GPIO_CAMERA_CMPDN_PIN_M_GPIO,
			GPIO_OUT_ONE,
			GPIO_OUT_ZERO,
		},
		//for sub sensor
		{GPIO_CAMERA_CMRST1_PIN,
			GPIO_CAMERA_CMRST1_PIN_M_GPIO,
			GPIO_OUT_ONE,
			GPIO_OUT_ZERO,
			GPIO_CAMERA_CMPDN1_PIN,
			GPIO_CAMERA_CMPDN1_PIN_M_GPIO,
			GPIO_OUT_ONE,			
			GPIO_OUT_ZERO,
		},
	};
	if (DUAL_CAMERA_MAIN_SENSOR == SensorIdx){
		pinSetIdx = 0;
		searchMainSensor = KAL_TRUE;
	}
	else if (DUAL_CAMERA_SUB_SENSOR == SensorIdx) {
		pinSetIdx = 1;
		searchMainSensor = KAL_FALSE;
	}

	sensor_index =  pinSetIdx;

	if (currSensorName && ((0 == strcmp(SENSOR_DRVNAME_SP0A20_MIPI_YUV, currSensorName))
						   ))
		camera_pdn1_reverse = TRUE;
	/* else */
	/* 	camera_pdn1_reverse = FALSE; */


	if(camera_pdn_reverse)
	{
		pinSet[0][IDX_PS_CMPDN+IDX_PS_ON] = GPIO_OUT_ZERO;
		pinSet[0][IDX_PS_CMPDN+IDX_PS_OFF] = GPIO_OUT_ONE;
	}
	if(camera_pdn1_reverse)
	{
		pinSet[1][IDX_PS_CMPDN+IDX_PS_ON] = GPIO_OUT_ZERO;
		pinSet[1][IDX_PS_CMPDN+IDX_PS_OFF] = GPIO_OUT_ONE;
	}

	//power ON
	if (On) {
#ifdef GPIO_CAMERA_LDO_EN_PIN //TODO: depends on HW layout. Should be notified by SA.
		PK_DBG("Set CAMERA_POWER_PULL_PIN for power \n");
		if (mt_set_gpio_pull_enable(GPIO_CAMERA_LDO_EN_PIN, GPIO_PULL_DISABLE)) {PK_DBG("[[CAMERA SENSOR] Set CAMERA_POWER_PULL_PIN DISABLE ! \n"); }
		if(mt_set_gpio_mode(GPIO_CAMERA_LDO_EN_PIN, GPIO_CAMERA_LDO_EN_PIN_M_GPIO)){PK_DBG("[[CAMERA SENSOR] set CAMERA_POWER_PULL_PIN mode failed!! \n");}
		if(mt_set_gpio_dir(GPIO_CAMERA_LDO_EN_PIN,GPIO_DIR_OUT)){PK_DBG("[[CAMERA SENSOR] set CAMERA_POWER_PULL_PIN dir failed!! \n");}
		if(mt_set_gpio_out(GPIO_CAMERA_LDO_EN_PIN,GPIO_OUT_ONE)){PK_DBG("[[CAMERA SENSOR] set CAMERA_POWER_PULL_PIN failed!! \n");}
#endif
#ifdef GPIO_CAMERA_AF_EN_PIN
		PK_DBG("af enable pin \n");
		if (mt_set_gpio_pull_enable(GPIO_CAMERA_AF_EN_PIN, GPIO_PULL_DISABLE)) {PK_DBG("[[CAMERA SENSOR] Set GPIO_CAMERA_AF_EN_PIN DISABLE ! \n"); }
		if(mt_set_gpio_mode(GPIO_CAMERA_AF_EN_PIN, GPIO_CAMERA_AF_EN_PIN_M_GPIO)){PK_DBG("[[CAMERA SENSOR] set GPIO_CAMERA_AF_EN_PIN mode failed!! \n");}
		if(mt_set_gpio_dir(GPIO_CAMERA_AF_EN_PIN,GPIO_DIR_OUT)){PK_DBG("[[CAMERA SENSOR] set GPIO_CAMERA_AF_EN_PIN dir failed!! \n");}
		//if(mt_set_gpio_out(GPIO_CAMERA_AF_EN_PIN,GPIO_OUT_ONE)){PK_DBG("[[CAMERA SENSOR] set CAMERA_POWER_PULL_PIN failed!! \n");}
#endif

		PK_DBG("kdCISModulePowerOn -on:currSensorName=%s\n",currSensorName);
		PK_DBG("kdCISModulePowerOn -on:pinSetIdx=%d\n",pinSetIdx);

		vanzoVcamD2Volt=VOL_1800;
		if (currSensorName && ((0 == strcmp(SENSOR_DRVNAME_OV13850_MIPI_RAW,currSensorName)) || (0 == strcmp(SENSOR_DRVNAME_OV13850_TRULY_MIPI_RAW,currSensorName)) || (0 == strcmp(SENSOR_DRVNAME_OV12830_MIPI_RAW,currSensorName))||/* (0 == strcmp(SENSOR_DRVNAME_OV8865_MIPI_RAW,currSensorName)) || */(0 == strcmp(SENSOR_DRVNAME_MN34130_MIPI_RAW,currSensorName))|| (0 == strcmp(SENSOR_DRVNAME_IMX111_MIPI_RAW,currSensorName))||(0 == strcmp(SENSOR_DRVNAME_OV5693_MIPI_RAW,currSensorName))||(0 == strcmp(SENSOR_DRVNAME_C2580_YUV,currSensorName))||(0 == strcmp(SENSOR_DRVNAME_S5K5E2YA_MIPI_RAW,currSensorName)) ||(0 == strcmp(SENSOR_DRVNAME_S5K3H5XA_MIPI_RAW,currSensorName)) || (0 == strcmp(SENSOR_DRVNAME_S5K3M2_MIPI_RAW, currSensorName)) ||(0 == strcmp(SENSOR_DRVNAME_IMX219_MIPI_RAW,currSensorName)) || (0 == strcmp(SENSOR_DRVNAME_GS8604_MIPI_RAW,currSensorName)) ||(0 == strcmp(SENSOR_DRVNAME_HI545_MIPI_RAW,currSensorName)) || (0 == strcmp(SENSOR_DRVNAME_HI551_MIPI_RAW,currSensorName)) || (0 == strcmp(SENSOR_DRVNAME_IMX175_MIPI_RAW_2LANE,currSensorName)) || (0 == strcmp(SENSOR_DRVNAME_OV8858_MIPI_RAW,currSensorName)) ||(0 == strcmp(SENSOR_DRVNAME_OV5670_MIPI_RAW,currSensorName))|| (0 == strcmp(SENSOR_DRVNAME_S5K4H5YX_MIPI_RAW,currSensorName))))
		{
			vanzoVcamDVolt=VOL_1200;
		}
		else if (currSensorName && (0 == strcmp(SENSOR_DRVNAME_IMX214_MIPI_RAW,currSensorName)))
		{
			vanzoVcamDVolt=VOL_1000;
		}
		else if (currSensorName && (0 == strcmp(SENSOR_DRVNAME_OV8865_MIPI_RAW,currSensorName)))
		{
			vanzoVcamDVolt=VOL_1300;
		}
        else if (currSensorName && ((0 == strcmp(SENSOR_DRVNAME_GC2235_RAW,currSensorName)) || (0 == strcmp(SENSOR_DRVNAME_GC2235_MIPI_RAW,currSensorName)) || (0 == strcmp(SENSOR_DRVNAME_GC2355_MIPI_RAW,currSensorName)) || (0 == strcmp(SENSOR_DRVNAME_GC2355_RAW,currSensorName)) || (0 == strcmp(SENSOR_DRVNAME_GC2355SUB_MIPI_RAW, currSensorName)) || (0 == strcmp(SENSOR_DRVNAME_GC2035_YUV, currSensorName)) || (0 == strcmp(SENSOR_DRVNAME_GC2035_MIPI_YUV, currSensorName)) || (0 == strcmp(SENSOR_DRVNAME_SP5408_MIPI_RAW,currSensorName)) || (0 == strcmp(SENSOR_DRVNAME_SP5409_MIPI_RAW,currSensorName)) || (0 == strcmp(SENSOR_DRVNAME_SP0838_YUV,currSensorName)) || (0 == strcmp(SENSOR_DRVNAME_HM5040_MIPI_RAW,currSensorName)) || (0 == strcmp(SENSOR_DRVNAME_GC2755_MIPI_RAW,currSensorName)) || (0 == strcmp(SENSOR_DRVNAME_GC0409MIPI_RAW,currSensorName))))
		{
			vanzoVcamDVolt=VOL_1800;
		}
		else
			vanzoVcamDVolt=VOL_1500;
		if (currSensorName && ((0 == strcmp(SENSOR_DRVNAME_HM8131_MIPI_RAW,currSensorName)) || (0 == strcmp(SENSOR_DRVNAME_OV5693_MIPI_RAW,currSensorName)) || (0 == strcmp(SENSOR_DRVNAME_OV5645_MIPI_YUV,currSensorName))))
		{
			vanzoVcamSubDVolt=VOL_1200;
		}
		else if (currSensorName && ((0 == strcmp(SENSOR_DRVNAME_GC2235_RAW,currSensorName)) || (0 == strcmp(SENSOR_DRVNAME_HI704_YUV,currSensorName)) || (0 == strcmp(SENSOR_DRVNAME_GC2235_MIPI_RAW,currSensorName)) || (0 == strcmp(SENSOR_DRVNAME_GC2355_MIPI_RAW,currSensorName)) || (0 == strcmp(SENSOR_DRVNAME_GC2355SUB_MIPI_RAW, currSensorName)) || (0 == strcmp(SENSOR_DRVNAME_GC2145_MIPI_YUV,currSensorName)) || (0 == strcmp(SENSOR_DRVNAME_GC2155_YUV,currSensorName)) ||(0 == strcmp(SENSOR_DRVNAME_GC2155SUB_MIPI_YUV,currSensorName))||(0 == strcmp(SENSOR_DRVNAME_GC2155_MIPI_YUV,currSensorName)) || (0 == strcmp(SENSOR_DRVNAME_GC2035_YUV, currSensorName)) || (0 == strcmp(SENSOR_DRVNAME_GC2035_MIPI_YUV, currSensorName)) || (0 == strcmp(SENSOR_DRVNAME_SP2508_MIPI_RAW,currSensorName)) || (0 == strcmp(SENSOR_DRVNAME_GC0409MIPI_RAW,currSensorName))))
		{
			vanzoVcamSubDVolt=VOL_1800;
		}
		else
		{
			vanzoVcamSubDVolt=VOL_1500;
		}
		//PK_DBG("kdCISModulePowerOn get in---  other \n");
		//enable active sensor
		//RST pin
		if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {
			if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
			if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
			if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
			mdelay(10);

			//PDN pin
			if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
			if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
			if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_OFF])){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
		}
#ifdef GPIO_CAMERA_AF_EN_PIN
		if(mt_set_gpio_out(GPIO_CAMERA_AF_EN_PIN,GPIO_OUT_ONE)){PK_DBG("[[CAMERA SENSOR]  enable af en out 1 failed!! \n");}
#endif
		//AVDD
		if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A, VOL_2800,mode_name))
		{
			PK_DBG("[CAMERA SENSOR] Fail to enable analog power\n");
			//return -EIO;
			//goto _kdCISModulePowerOn_exit_;
		}
		mdelay(10);
		
		//DVDD
		if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D, vanzoVcamDVolt,mode_name))
		{
			PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
			goto _kdCISModulePowerOn_exit_;
		}
		mdelay(10);		
		
		//DOVDD
		// PK_DBG("[ON_general 1.8V]sensorIdx:%d \n",SensorIdx);
		if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D2, vanzoVcamD2Volt,mode_name))
		{
			PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
			//return -EIO;
			//goto _kdCISModulePowerOn_exit_;
		}
		mdelay(10);
		
#if (defined(MT6592) || defined(DCT_K8) || defined(DCT_A8) || defined(DCT_A805) || defined(DCT_A5) || defined(DCT_A9) || defined(DCT_A807) || defined(DCT_A562))
		//sub DVDD
		if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D_SUB, vanzoVcamSubDVolt,mode_name))
		{
			PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
			goto _kdCISModulePowerOn_exit_;
		}
		mdelay(10);
#endif
#if (defined(DCT_A20) || defined(DCT_A60) || defined(DCT_A6) || defined(DCT_A29) || defined DCT_A318)
		//sub DVDD
		if(TRUE != hwPowerOn(MT6323_POWER_LDO_VGP2, vanzoVcamSubDVolt,mode_name))
		{
			PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
			goto _kdCISModulePowerOn_exit_;
		}
		mdelay(10);
#endif

		//AF_VCC
		if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A2, VOL_2800,mode_name))
		{
			PK_DBG("[CAMERA SENSOR] Fail to enable analog power\n");
			//return -EIO;
			goto _kdCISModulePowerOn_exit_;
		}
#if 1
		//enable active sensor
		if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {
			//PDN pin
			if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
			if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
			if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_ON])){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
			mdelay(10);
			if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_OFF])){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
			mdelay(15);
			if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_ON])){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
			mdelay(10);

			if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
			if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
			if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_ON])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
			mdelay(10);
			if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
			mdelay(15);
			if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_ON])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
			mdelay(10);
		}
#endif
		//disable inactive sensor
		if(pinSetIdx == 0 || pinSetIdx == 2) {//disable sub
			if (GPIO_CAMERA_INVALID != pinSet[1][IDX_PS_CMRST]) {
				if(mt_set_gpio_mode(pinSet[1][IDX_PS_CMRST],pinSet[1][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
				if(mt_set_gpio_mode(pinSet[1][IDX_PS_CMPDN],pinSet[1][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
				if(mt_set_gpio_dir(pinSet[1][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
				if(mt_set_gpio_dir(pinSet[1][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
				if(mt_set_gpio_out(pinSet[1][IDX_PS_CMRST],pinSet[1][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");} //low == reset sensor
				if(mt_set_gpio_out(pinSet[1][IDX_PS_CMPDN],pinSet[1][IDX_PS_CMPDN+IDX_PS_OFF])){PK_DBG("[CAMERA LENS] set gpio failed!! \n");} //high == power down lens module
			}
		}
		else {
			if (GPIO_CAMERA_INVALID != pinSet[0][IDX_PS_CMRST]) {
				if(mt_set_gpio_mode(pinSet[0][IDX_PS_CMRST],pinSet[0][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
				if(mt_set_gpio_mode(pinSet[0][IDX_PS_CMPDN],pinSet[0][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
				if(mt_set_gpio_dir(pinSet[0][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
				if(mt_set_gpio_dir(pinSet[0][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
				if(mt_set_gpio_out(pinSet[0][IDX_PS_CMRST],pinSet[0][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");} //low == reset sensor
				if(mt_set_gpio_out(pinSet[0][IDX_PS_CMPDN],pinSet[0][IDX_PS_CMPDN+IDX_PS_OFF])){PK_DBG("[CAMERA LENS] set gpio failed!! \n");} //high == power down lens module
			}
			if (GPIO_CAMERA_INVALID != pinSet[2][IDX_PS_CMRST]) {
				/*if(mt_set_gpio_mode(pinSet[2][IDX_PS_CMRST],pinSet[2][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
				  if(mt_set_gpio_mode(pinSet[2][IDX_PS_CMPDN],pinSet[2][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
				  if(mt_set_gpio_dir(pinSet[2][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
				  if(mt_set_gpio_dir(pinSet[2][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
				  if(mt_set_gpio_out(pinSet[2][IDX_PS_CMRST],pinSet[2][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");} //low == reset sensor
				  if(mt_set_gpio_out(pinSet[2][IDX_PS_CMPDN],pinSet[2][IDX_PS_CMPDN+IDX_PS_OFF])){PK_DBG("[CAMERA LENS] set gpio failed!! \n");} //high == power down lens module
				  */
			}
		}
	}
	else {//power OFF

		PK_DBG("kdCISModulePowerOn -off:currSensorName=%s\n",currSensorName);
#ifdef GPIO_CAMERA_LDO_EN_PIN//TODO: depends on HW layout. Should be notified by SA.
		if(mt_set_gpio_out(GPIO_CAMERA_LDO_EN_PIN,GPIO_OUT_ZERO)){PK_DBG("[[CAMERA SENSOR] set CAMERA_POWER_PULL_PIN failed!! \n");}
#endif

#ifdef GPIO_CAMERA_AF_EN_PIN
		if(mt_set_gpio_out(GPIO_CAMERA_AF_EN_PIN,GPIO_OUT_ZERO)){PK_DBG("[[CAMERA SENSOR]  enable af en out 1 failed!! \n");}
#endif
		//PK_DBG("kdCISModulePower--off get in---other \n");
		//PK_DBG("[OFF]sensorIdx:%d \n",SensorIdx);
		if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {
			if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
			if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
			if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
			if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
			if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_OFF])){PK_DBG("[CAMERA LENS] set gpio failed!! \n");} //high == power down lens module
			if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");} //low == reset sensor
		}

		if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_A,mode_name)) {
			PK_DBG("[CAMERA SENSOR] Fail to OFF analog power\n");
			//return -EIO;
			goto _kdCISModulePowerOn_exit_;
		}
		if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_A2,mode_name))
		{
			PK_DBG("[CAMERA SENSOR] Fail to enable analog power\n");
			//return -EIO;
			goto _kdCISModulePowerOn_exit_;
		}
		if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_D, mode_name)) {
			PK_DBG("[CAMERA SENSOR] Fail to OFF digital power\n");
			//return -EIO;
			goto _kdCISModulePowerOn_exit_;
		}
#if (defined(MT6592) || defined(DCT_K8) || defined(DCT_A8) || defined(DCT_A805) || defined(DCT_A5) || defined(DCT_A9) || defined(DCT_A807) || defined(DCT_A562))
		if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_D_SUB, mode_name)) {
			PK_DBG("[CAMERA SENSOR] Fail to OFF digital power\n");
			//return -EIO;
			goto _kdCISModulePowerOn_exit_;
		}
#endif
#if (defined(DCT_A20) || defined(DCT_A60) || defined(DCT_A6) || defined(DCT_A29) || defined DCT_A318 || defined DCT_S1)
		if(TRUE != hwPowerDown(MT6323_POWER_LDO_VGP2, mode_name)) {
			PK_DBG("[CAMERA SENSOR] Fail to OFF digital power\n");
			//return -EIO;
			goto _kdCISModulePowerOn_exit_;
		}
#endif

		if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_D2,mode_name))
		{
			PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
			//return -EIO;
			goto _kdCISModulePowerOn_exit_;
		}

        if (currSensorName && ((0 == strcmp(SENSOR_DRVNAME_SP0A20_MIPI_YUV, currSensorName))
                               ))
            camera_pdn1_reverse = FALSE;
	}
	return 0;

_kdCISModulePowerOn_exit_:
	return -EIO;
}

EXPORT_SYMBOL(kdCISModulePowerOn);
