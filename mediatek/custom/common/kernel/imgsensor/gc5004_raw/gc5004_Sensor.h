/* Copyright Statement:
 *
 * This software/firmware and related documentation ("MediaTek Software") are
 * protected under relevant copyright laws. The information contained herein
 * is confidential and proprietary to MediaTek Inc. and/or its licensors.
 * Without the prior written permission of MediaTek inc. and/or its licensors,
 * any reproduction, modification, use or disclosure of MediaTek Software,
 * and information contained herein, in whole or in part, shall be strictly prohibited.  
 */
/* MediaTek Inc. (C) 2010. All rights reserved.
 *
 * BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 * THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
 * RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO RECEIVER ON
 * AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
 * NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
 * SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
 * SUPPLIED WITH THE MEDIATEK SOFTWARE, AND RECEIVER AGREES TO LOOK ONLY TO SUCH
 * THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. RECEIVER EXPRESSLY ACKNOWLEDGES
 * THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES
 * CONTAINED IN MEDIATEK SOFTWARE. MEDIATEK SHALL ALSO NOT BE RESPONSIBLE FOR ANY MEDIATEK
 * SOFTWARE RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR
 * STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND
 * CUMULATIVE LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
 * AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
 * OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY RECEIVER TO
 * MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
 */

/*****************************************************************************
*  Copyright Statement:
*  --------------------
*  This software is protected by Copyright and the information contained
*  herein is confidential. The software may not be copied and the information
*  contained herein may not be used or disclosed except with the written
*  permission of MediaTek Inc. (C) 2005
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
 *
 * Filename:
 * ---------
 *	 sensor.c
 *
 * Project:
 * --------
 *	 RAW
 *
 * Description:
 * ------------
 *	 Source code of Sensor driver
 *
 *
 * Author:
 * -------
 *   Leo Lee
 *
 *============================================================================
 *             HISTORY
 *------------------------------------------------------------------------------
 * $Revision:$
 * $Modtime:$
 * $Log:$
 *
 * 03 01 2013
 * First release GC5004 driver Version 1.0
 *
 *------------------------------------------------------------------------------
 *============================================================================
 ****************************************************************************/

#ifndef _GC5004_SENSOR_H
#define _GC5004_SENSOR_H

#define GC5004_DEBUG
#define GC5004_DRIVER_TRACE
//#define GC5004_TEST_PATTEM

#define GC5004_FACTORY_START_ADDR 0
#define GC5004_ENGINEER_START_ADDR 10
 
typedef enum GC5004_group_enum
{
  GC5004_PRE_GAIN = 0,
  GC5004_CMMCLK_CURRENT,
  GC5004_FRAME_RATE_LIMITATION,
  GC5004_REGISTER_EDITOR,
  GC5004_GROUP_TOTAL_NUMS
} GC5004_FACTORY_GROUP_ENUM;

typedef enum GC5004_register_index
{
  GC5004_SENSOR_BASEGAIN = GC5004_FACTORY_START_ADDR,
  GC5004_PRE_GAIN_R_INDEX,
  GC5004_PRE_GAIN_Gr_INDEX,
  GC5004_PRE_GAIN_Gb_INDEX,
  GC5004_PRE_GAIN_B_INDEX,
  GC5004_FACTORY_END_ADDR
} GC5004_FACTORY_REGISTER_INDEX;

typedef enum GC5004_engineer_index
{
  GC5004_CMMCLK_CURRENT_INDEX = GC5004_ENGINEER_START_ADDR,
  GC5004_ENGINEER_END
} GC5004_FACTORY_ENGINEER_INDEX;

typedef struct _sensor_data_struct
{
  SENSOR_REG_STRUCT reg[GC5004_ENGINEER_END];
  SENSOR_REG_STRUCT cct[GC5004_FACTORY_END_ADDR];
} sensor_data_struct;

/* SENSOR PREVIEW/CAPTURE VT CLOCK */
#define GC5004_PREVIEW_CLK                   240000000
#define GC5004_CAPTURE_CLK                    96000000

#define GC5004_COLOR_FORMAT                    SENSOR_OUTPUT_FORMAT_RAW_B //SENSOR_OUTPUT_FORMAT_RAW_Gb //SENSOR_OUTPUT_FORMAT_RAW_R

#define GC5004_MIN_ANALOG_GAIN				1	/* 1x */
#define GC5004_MAX_ANALOG_GAIN				6	/* 6x */


/* FRAME RATE UNIT */
#define GC5004_FPS(x)                          (10 * (x))

/* SENSOR PIXEL/LINE NUMBERS IN ONE PERIOD */
#define GC5004_FULL_PERIOD_PIXEL_NUMS          3840//1974 /* 8 fps */
#define GC5004_FULL_PERIOD_LINE_NUMS           1988

#define GC5004_VIDEO_PERIOD_PIXEL_NUMS          4800//1974 /* 8 fps */
#define GC5004_VIDEO_PERIOD_LINE_NUMS           1988

#define GC5004_PV_PERIOD_PIXEL_NUMS            4800//1974 /* 30 fps */
#define GC5004_PV_PERIOD_LINE_NUMS             1988

/* SENSOR START/END POSITION */
#define GC5004_FULL_X_START                    2
#define GC5004_FULL_Y_START                    2
#define GC5004_IMAGE_SENSOR_FULL_WIDTH         (2592 - 8)
#define GC5004_IMAGE_SENSOR_FULL_HEIGHT        (1944 - 6)

#define GC5004_VIDEO_X_START                      2
#define GC5004_VIDEO_Y_START                      2
#define GC5004_IMAGE_SENSOR_VIDEO_WIDTH           (1296 - 8)
#define GC5004_IMAGE_SENSOR_VIDEO_HEIGHT          (976  - 6)

#define GC5004_PV_X_START                      2
#define GC5004_PV_Y_START                      2
#define GC5004_IMAGE_SENSOR_PV_WIDTH           (1296 - 8)
#define GC5004_IMAGE_SENSOR_PV_HEIGHT          (976  - 6)

/* SENSOR READ/WRITE ID */
#define GC5004_WRITE_ID (0x6c)
#define GC5004_READ_ID  (0x6d)

/* SENSOR ID */
//#define GC5004_SENSOR_ID						(0x5004)

/* SENSOR PRIVATE STRUCT */
typedef enum {
    SENSOR_MODE_INIT = 0,
    SENSOR_MODE_PREVIEW,
    SENSOR_MODE_VIDEO,
    SENSOR_MODE_CAPTURE
} GC5004_SENSOR_MODE;

typedef enum{
	GC5004_IMAGE_NORMAL = 0,
	GC5004_IMAGE_H_MIRROR,
	GC5004_IMAGE_V_MIRROR,
	GC5004_IMAGE_HV_MIRROR
}GC5004_IMAGE_MIRROR;

typedef struct GC5004_sensor_STRUCT
{
	MSDK_SENSOR_CONFIG_STRUCT cfg_data;
	sensor_data_struct eng; /* engineer mode */
	MSDK_SENSOR_ENG_INFO_STRUCT eng_info;
	GC5004_SENSOR_MODE sensorMode;
	GC5004_IMAGE_MIRROR Mirror;
	kal_bool pv_mode;
	kal_bool cap_mode;
	kal_bool video_mode;
	kal_bool NightMode;
	kal_bool LowLightMode;	
	kal_uint16 normal_fps; /* video normal mode max fps */
	kal_uint16 night_fps; /* video night mode max fps */
	kal_uint16 FixedFps;
	kal_uint16 shutter;
	kal_uint16 gain;
	kal_uint32 pclk;
	kal_uint16 frame_height;
	kal_uint16 frame_height_BackUp;
	kal_uint16 line_length;  
	kal_uint16 Prv_line_length;
} GC5004_sensor_struct;

typedef enum GC5004_GainMode_Index
{
	GC5004_Analogic_Gain = 0,
	GC5004_Digital_Gain
}GC5004_GainMode_Index;
//export functions
UINT32 GC5004Open(void);
UINT32 GC5004Control(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *pImageWindow, MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData);
UINT32 GC5004FeatureControl(MSDK_SENSOR_FEATURE_ENUM FeatureId, UINT8 *pFeaturePara,UINT32 *pFeatureParaLen);
UINT32 GC5004GetInfo(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_INFO_STRUCT *pSensorInfo, MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData);
UINT32 GC5004GetResolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *pSensorResolution);
UINT32 GC5004Close(void);

#define Sleep(ms) mdelay(ms)

#endif 
