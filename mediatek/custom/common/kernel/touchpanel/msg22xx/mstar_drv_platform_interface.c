////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2006-2012 MStar Semiconductor, Inc.
// All rights reserved.
//
// Unless otherwise stipulated in writing, any and all information contained
// herein regardless in any format shall remain the sole proprietary of
// MStar Semiconductor Inc. and be kept in strict confidence
// (??MStar Confidential Information??) by the recipient.
// Any unauthorized act including without limitation unauthorized disclosure,
// copying, use, reproduction, sale, distribution, modification, disassembling,
// reverse engineering and compiling of the contents of MStar Confidential
// Information is unlawful and strictly prohibited. MStar hereby reserves the
// rights to any and all damages, losses, costs and expenses resulting therefrom.
//
////////////////////////////////////////////////////////////////////////////////

/**
 *
 * @file    mstar_drv_platform_interface.c
 *
 * @brief   This file defines the interface of touch screen
 *
 * @version v2.0.0.0
 *
 */

/*=============================================================*/
// INCLUDE FILE
/*=============================================================*/

#include "mstar_drv_platform_interface.h"
#include "mstar_drv_main.h"
#include "mstar_drv_ic_fw_porting_layer.h"
#include "mstar_drv_platform_porting_layer.h"

/*=============================================================*/
// EXTERN VARIABLE DECLARATION
/*=============================================================*/

#ifdef CONFIG_ENABLE_GESTURE_WAKEUP
extern u16 g_GestureWakeupMode;
extern u8 g_GestureWakeupFlag;
#endif //CONFIG_ENABLE_GESTURE_WAKEUP

/*=============================================================*/
// GLOBAL VARIABLE DEFINITION
/*=============================================================*/

extern struct input_dev *g_InputDevice;
#ifdef CONFIG_TOUCH_DRIVER_RUN_ON_SPRD_PLATFORM 
extern struct regulator *g_ReguVdd;
#endif //CONFIG_TOUCH_DRIVER_RUN_ON_SPRD_PLATFORM


/*=============================================================*/
// GLOBAL FUNCTION DEFINITION
/*=============================================================*/

void MsDrvInterfaceTouchDeviceSuspend(struct early_suspend *pSuspend)
{
    DBG("*** %s() ***\n", __func__);

#ifdef CONFIG_ENABLE_GESTURE_WAKEUP
   g_GestureWakeupMode = 0x3FFF; // Enable all gesture wakeup mode for testing

    if (g_GestureWakeupMode != 0x0000)
    {
	//memset(msg_coordinate_x,0,150);
	//memset(msg_coordinate_y,0,150);
	memset(msg_original_coordinate,0,200);
        if(DrvIcFwLyrOpenGestureWakeup(g_GestureWakeupMode) == 1)
        	return;
    }
#endif //CONFIG_ENABLE_GESTURE_WAKEUP

    DrvPlatformLyrFingerTouchReleased(0, 0); // Send touch end for clearing point touch
    input_sync(g_InputDevice);

    DrvPlatformLyrDisableFingerTouchReport();
    DrvPlatformLyrTouchDevicePowerOff(); 
}

void MsDrvInterfaceTouchDeviceResume(struct early_suspend *pSuspend)
{
    DBG("*** %s() ***\n", __func__);
#ifdef CONFIG_ENABLE_GESTURE_WAKEUP
    if (g_GestureWakeupFlag == 1)
    {
        DrvIcFwLyrCloseGestureWakeup();
    }
    else
    {
        DrvPlatformLyrEnableFingerTouchReport(); 
    }
#endif //CONFIG_ENABLE_GESTURE_WAKEUP
    
    DrvPlatformLyrTouchDevicePowerOn();
/*
    DrvPlatformLyrFingerTouchReleased(0, 0);
    input_sync(g_InputDevice);
*/    
#ifdef CONFIG_ENABLE_FIRMWARE_DATA_LOG
    DrvIcFwLyrRestoreFirmwareModeToLogDataMode();
#endif //CONFIG_ENABLE_FIRMWARE_DATA_LOG

#ifndef CONFIG_ENABLE_GESTURE_WAKEUP
    DrvPlatformLyrEnableFingerTouchReport(); 
#endif //CONFIG_ENABLE_GESTURE_WAKEUP

}

/* probe function is used for matching and initializing input device */
s32 /*__devinit*/ MsDrvInterfaceTouchDeviceProbe(struct i2c_client *pClient, const struct i2c_device_id *pDeviceId)
{
    s32 nRetVal = 0;

    DBG("*** %s() ***\n", __func__);
  
    DrvPlatformLyrInputDeviceInitialize(pClient);
  
    DrvPlatformLyrTouchDeviceRequestGPIO();

#ifdef CONFIG_TOUCH_DRIVER_RUN_ON_SPRD_PLATFORM 
    DrvPlatformLyrTouchDeviceRegulatorPowerOn(g_ReguVdd);
#endif //CONFIG_TOUCH_DRIVER_RUN_ON_SPRD_PLATFORM

    DrvPlatformLyrTouchDevicePowerOn();

    nRetVal = DrvMainTouchDeviceInitialize();

    DrvPlatformLyrTouchDeviceRegisterFingerTouchInterruptHandler();

    DrvPlatformLyrTouchDeviceRegisterEarlySuspend();

    DBG("*** MStar touch driver registered ***\n");
    
    return nRetVal;
}

/* remove function is triggered when the input device is removed from input sub-system */
s32 /*__devexit*/ MsDrvInterfaceTouchDeviceRemove(struct i2c_client *pClient)
{
    DBG("*** %s() ***\n", __func__);

    return DrvPlatformLyrTouchDeviceRemove(pClient);
}

void MsDrvInterfaceTouchDeviceSetIicDataRate(struct i2c_client *pClient, u32 nIicDataRate)
{
    DBG("*** %s() ***\n", __func__);

    DrvPlatformLyrSetIicDataRate(pClient, nIicDataRate);
}    
