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

  //power ON
  if (On) {
    PK_DBG("kdCISModulePowerOn -on:currSensorName=%s\n",currSensorName);
    PK_DBG("kdCISModulePowerOn -on:pinSetIdx=%d\n",pinSetIdx);

    if (currSensorName && (0 == strcmp(currSensorName,"imx135mipiraw")))
    {
      if(mt_set_gpio_mode(GPIO64, 0)){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
      if(mt_set_gpio_dir(GPIO64, GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
      if(mt_set_gpio_out(GPIO64, GPIO_OUT_ONE)){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}

      mdelay(5);

      if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D2, 2800, mode_name)) // 18
      {
        PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
        goto _kdCISModulePowerOn_exit_;
      }

      if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A2, 2800, mode_name)) // 12
      {
        PK_DBG("[CAMERA SENSOR] Fail to enable analog power\n");
        goto _kdCISModulePowerOn_exit_;
      }
      mdelay(5);		

      // disable sub camera
      if (GPIO_CAMERA_INVALID != pinSet[1][IDX_PS_CMRST])
      {
        if(mt_set_gpio_mode(pinSet[1][IDX_PS_CMRST],pinSet[1][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
        if(mt_set_gpio_dir(pinSet[1][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
        if(mt_set_gpio_out(pinSet[1][IDX_PS_CMRST],pinSet[1][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}

        if(mt_set_gpio_mode(pinSet[1][IDX_PS_CMPDN],pinSet[1][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
        if(mt_set_gpio_dir(pinSet[1][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
        if(mt_set_gpio_out(pinSet[1][IDX_PS_CMPDN],pinSet[1][IDX_PS_CMPDN+IDX_PS_OFF])){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
      }

      if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST])
      {
        if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
        if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
        if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_OFF])){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
        mdelay(10);
        if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_ON])){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
        mdelay(1);

        if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
        if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
        if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
        mdelay(10);
        if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_ON])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
        mdelay(1);
      }
    }
    else if (currSensorName && (0 == strcmp(currSensorName,"imx111mipiraw")))
    {
      if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D, VOL_1200, mode_name)) // 17
      {
        PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
        goto _kdCISModulePowerOn_exit_;
      }
      mdelay(1);		

      if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D2, VOL_1800, mode_name)) // 18
      {
        PK_DBG("[CAMERA SENSOR] Fail to enable analog power\n");
        goto _kdCISModulePowerOn_exit_;
      }
      mdelay(1);

      if(mt_set_gpio_mode(GPIO64, 0)){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
      if(mt_set_gpio_dir(GPIO64, GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
      if(mt_set_gpio_out(GPIO64, GPIO_OUT_ONE)){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}

      mdelay(5);

      if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A, VOL_2800, mode_name)) // 21
      {
        PK_DBG("[CAMERA SENSOR] Fail to enable analog power\n");
        goto _kdCISModulePowerOn_exit_;
      }

      // disable main camera
      if (GPIO_CAMERA_INVALID != pinSet[0][IDX_PS_CMRST])
      {
        if(mt_set_gpio_mode(pinSet[0][IDX_PS_CMRST],pinSet[0][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
        if(mt_set_gpio_dir(pinSet[0][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
        if(mt_set_gpio_out(pinSet[0][IDX_PS_CMRST],pinSet[0][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}

        if(mt_set_gpio_mode(pinSet[0][IDX_PS_CMPDN],pinSet[0][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
        if(mt_set_gpio_dir(pinSet[0][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
        if(mt_set_gpio_out(pinSet[0][IDX_PS_CMPDN],pinSet[0][IDX_PS_CMPDN+IDX_PS_OFF])){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
      }

      if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST])
      {
        if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
        if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
        if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_OFF])){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
        mdelay(10);
        if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_ON])){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
        mdelay(1);

        if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
        if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
        if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
        mdelay(10);
        if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_ON])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
        mdelay(1);
      }
    }
  }
  else {//power OFF
    PK_DBG("kdCISModulePowerOn -off:currSensorName=%s\n",currSensorName);
    PK_DBG("kdCISModulePowerOn -off:pinSetIdx=%d\n",pinSetIdx);

    if (currSensorName && (0 == strcmp(currSensorName,"imx135mipiraw")))
    {
      if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
      if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
      if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_OFF])){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}

      if(mt_set_gpio_mode(GPIO64, 0)){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
      if(mt_set_gpio_dir(GPIO64, GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
      if(mt_set_gpio_out(GPIO64, GPIO_OUT_ZERO)){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}

      if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_D2, mode_name))
      {
        PK_DBG("[CAMERA SENSOR] Fail to OFF analog power\n");
        goto _kdCISModulePowerOn_exit_;
      }

      if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_A2, mode_name))
      {
        PK_DBG("[CAMERA SENSOR] Fail to OFF digital power\n");
        goto _kdCISModulePowerOn_exit_;
      }

      if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
      if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
      if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
    }
    else if (currSensorName && (0 == strcmp(currSensorName,"imx111mipiraw")))
    {
      if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
      if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
      if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_OFF])){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}

      if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_A, mode_name)) // 21
      {
        PK_DBG("[CAMERA SENSOR] Fail to OFF analog power\n");
        goto _kdCISModulePowerOn_exit_;
      }

      if(mt_set_gpio_mode(GPIO64, 0)){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
      if(mt_set_gpio_dir(GPIO64, GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
      if(mt_set_gpio_out(GPIO64, GPIO_OUT_ZERO)){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}

      if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_D2, mode_name)) // 18
      {
        PK_DBG("[CAMERA SENSOR] Fail to OFF digital power\n");
        goto _kdCISModulePowerOn_exit_;
      }

      if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_D, mode_name)) // 17
      {
        PK_DBG("[CAMERA SENSOR] Fail to OFF digital power\n");
        goto _kdCISModulePowerOn_exit_;
      }

      if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
      if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
      if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
    }
	}
	return 0;

_kdCISModulePowerOn_exit_:
	return -EIO;
}

EXPORT_SYMBOL(kdCISModulePowerOn);
