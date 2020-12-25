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
 * @file    mstar_drv_ic_fw_porting_layer.c
 *
 * @brief   This file defines the interface of touch screen
 *
 * @version v2.0.0.0
 *
 */
 
/*=============================================================*/
// INCLUDE FILE
/*=============================================================*/

#include "mstar_drv_ic_fw_porting_layer.h"
#include <asm/uaccess.h>
#include <linux/kdev_t.h>
#include <linux/fs.h>
#include "tpd.h"
#include <cust_alsps.h>



/*=============================================================*/
// EXTERN VARIABLE DECLARATION
/*=============================================================*/

#ifdef CONFIG_ENABLE_GESTURE_WAKEUP
extern u16 g_GestureWakeupMode;
extern u8 g_GestureWakeupFlag;
#endif //CONFIG_ENABLE_GESTURE_WAKEUP


/*=============================================================*/
// GLOBAL FUNCTION DEFINITION
/*=============================================================*/

u8 DrvIcFwLyrGetChipType(void)
{
//    DBG("*** %s() ***\n", __func__);

    return DrvFwCtrlGetChipType();
}

void DrvIcFwLyrGetCustomerFirmwareVersion(u16 *pMajor, u16 *pMinor, u8 **ppVersion)
{
//    DBG("*** %s() ***\n", __func__);

    DrvFwCtrlGetCustomerFirmwareVersion(pMajor, pMinor, ppVersion);
}

void DrvIcFwLyrGetPlatformFirmwareVersion(u8 **ppVersion)
{
//    DBG("*** %s() ***\n", __func__);

    DrvFwCtrlGetPlatformFirmwareVersion(ppVersion);
}

s32 DrvIcFwLyrUpdateFirmware(u8 szFwData[][1024], EmemType_e eEmemType)
{
//    DBG("*** %s() ***\n", __func__);

    return DrvFwCtrlUpdateFirmware(szFwData, eEmemType);
}	

u32 DrvIcFwLyrIsRegisterFingerTouchInterruptHandler(void)
{
    DBG("*** %s() ***\n", __func__);

    return 1; // Indicate that it is necessary to register interrupt handler with GPIO INT pin when firmware is running on IC
}

void DrvIcFwLyrHandleFingerTouch(u8 *pPacket, u16 nLength)
{
//    DBG("*** %s() ***\n", __func__);

    DrvFwCtrlHandleFingerTouch();
}

//------------------------------------------------------------------------------//

#ifdef CONFIG_ENABLE_GESTURE_WAKEUP

u8 DrvIcFwLyrOpenGestureWakeup(u16 nWakeupMode)
{
//    DBG("*** %s() ***\n", __func__);

   return  DrvFwCtrlOpenGestureWakeup(nWakeupMode);
}	

void DrvIcFwLyrCloseGestureWakeup(void)
{
//    DBG("*** %s() ***\n", __func__);

    DrvFwCtrlCloseGestureWakeup();
}	

#endif //CONFIG_ENABLE_GESTURE_WAKEUP

//------------------------------------------------------------------------------//

#ifdef CONFIG_ENABLE_FIRMWARE_DATA_LOG

#ifdef CONFIG_ENABLE_TOUCH_DRIVER_FOR_MUTUAL_IC
u16 DrvIcFwLyrGetFirmwareMode(void)
{
//    DBG("*** %s() ***\n", __func__);

    return DrvFwCtrlGetFirmwareMode();
}
#endif //CONFIG_ENABLE_TOUCH_DRIVER_FOR_MUTUAL_IC

u16 DrvIcFwLyrChangeFirmwareMode(u16 nMode)
{
//    DBG("*** %s() ***\n", __func__);

    return DrvFwCtrlChangeFirmwareMode(nMode); 
}

void DrvIcFwLyrGetFirmwareInfo(FirmwareInfo_t *pInfo)
{
//    DBG("*** %s() ***\n", __func__);

    DrvFwCtrlGetFirmwareInfo(pInfo);
}

void DrvIcFwLyrRestoreFirmwareModeToLogDataMode(void)
{
//    DBG("*** %s() ***\n", __func__);

    DrvFwCtrlRestoreFirmwareModeToLogDataMode();
}	

#endif //CONFIG_ENABLE_FIRMWARE_DATA_LOG

//------------------------------------------------------------------------------//

#ifdef CONFIG_UPDATE_FIRMWARE_BY_SW_ID
void DrvIcFwLyrCheckFirmwareUpdateBySwId(void)
{
//    DBG("*** %s() ***\n", __func__);

    DrvFwCtrlCheckFirmwareUpdateBySwId();
}
#endif //CONFIG_UPDATE_FIRMWARE_BY_SW_ID

//------------------------------------------------------------------------------//

#ifdef CONFIG_ENABLE_ITO_MP_TEST

void DrvIcFwLyrCreateMpTestWorkQueue(void)
{
//    DBG("*** %s() ***\n", __func__);
	
    DrvMpTestCreateMpTestWorkQueue();
}

void DrvIcFwLyrScheduleMpTestWork(ItoTestMode_e eItoTestMode)
{
//    DBG("*** %s() ***\n", __func__);
	
    DrvMpTestScheduleMpTestWork(eItoTestMode);
}

s32 DrvIcFwLyrGetMpTestResult(void)
{
//    DBG("*** %s() ***\n", __func__);
	
    return DrvMpTestGetTestResult();
}

void DrvIcFwLyrGetMpTestFailChannel(ItoTestMode_e eItoTestMode, u8 *pFailChannel, u32 *pFailChannelCount)
{
//    DBG("*** %s() ***\n", __func__);
	
    return DrvMpTestGetTestFailChannel(eItoTestMode, pFailChannel, pFailChannelCount);
}

void DrvIcFwLyrGetMpTestDataLog(ItoTestMode_e eItoTestMode, u8 *pDataLog, u32 *pLength)
{
//    DBG("*** %s() ***\n", __func__);

    return DrvMpTestGetTestDataLog(eItoTestMode, pDataLog, pLength);
}
#endif //CONFIG_ENABLE_ITO_MP_TEST		


#define MSG_GESTURE_PROC_FILE    	"gesture_enable" 
#define MSG_GESTURE_POINT_PROC_FILE    	"ft5x0x-debug"///"gesture_point_read" 
static unsigned char msg_proc_operate_mode = 0;
/////just for read point
#define MSG_PROC_READ_GESTRUEX		8
#define MSG_PROC_READ_GESTRUEY		10///9
static int msg_gesture_read_proc(char *page, char **start, off_t off, int count, int *eof, void *data);
static int msg_gesture_write_proc(struct file *file, const char *buffer, unsigned long count, void *data);
static int msg_gesture_point_write(struct file *filp, 
   const char __user *buff, unsigned long len, void *data);
static int msg_gesture_point_read( char *page, char **start,
	off_t off, int count, int *eof, void *data );

extern short msg_pointnum;
extern struct tpd_device *tpd;

void DrvIcFwCreateProcNode(void)
{
    static struct proc_dir_entry *Msg_gesture_proc = NULL;
    static struct proc_dir_entry *Msg_gesture_point_proc = NULL;

    Msg_gesture_proc = create_proc_entry(MSG_GESTURE_PROC_FILE, 0666, NULL);
    if (Msg_gesture_proc == NULL)
    {
	DBG("create Msg_gesture_proc fail\n");
    }
    else
    {
    	Msg_gesture_proc->read_proc = msg_gesture_read_proc;
    	Msg_gesture_proc->write_proc = msg_gesture_write_proc;
	DBG("create Msg_gesture_proc success\n");
    }
	
    Msg_gesture_point_proc = create_proc_entry(MSG_GESTURE_POINT_PROC_FILE, 0777, NULL);
    if (NULL == Msg_gesture_point_proc)
    {
    	DBG("create Msg_gesture_point_proc fail\n");
    } 
    else 
    {
    	Msg_gesture_point_proc->write_proc = msg_gesture_point_write;
    	Msg_gesture_point_proc->read_proc = msg_gesture_point_read;
    	DBG("create Msg_gesture_point_proc success\n");
    }
}






static int msg_gesture_read_proc(char *page, char **start, off_t off, int count, int *eof, void *data)
{
return 0;

}

static int msg_gesture_write_proc(struct file *file, const char *buffer, unsigned long count, void *data)
{
return 0;

}



 static int msg_gesture_point_write(struct file *filp, 
	const char __user *buff, unsigned long len, void *data)
{
	return 0;
}

/*interface of read proc*/
static int msg_gesture_point_read( char *page, char **start,
	off_t off, int count, int *eof, void *data )
{


	return 0;
}


