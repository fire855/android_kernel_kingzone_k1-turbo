/* Copyright Statement:
 *
 * This software/firmware and related documentation ("MediaTek Software") are
 * protected under relevant copyright laws. The information contained herein
 * is confidential and proprietary to MediaTek Inc. and/or its licensors.
 * Without the prior written permission of MediaTek inc. and/or its licensors,
 * any reproduction, modification, use or disclosure of MediaTek Software,
 * and information contained herein, in whole or in part, shall be strictly prohibited.
 *
 * MediaTek Inc. (C) 2010. All rights reserved.
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
 *
 * The following software/firmware and/or related documentation ("MediaTek Software")
 * have been modified by MediaTek Inc. All revisions are subject to any receiver's
 * applicable license agreements with MediaTek Inc.
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
 *   Sensor.c
 *
 * Project:
 * --------
 *   DUMA
 *
 * Description:
 * ------------
 *   Image sensor driver function
 *
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by PVCS VM. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/



/*#####################################################


superpix    sensor   30m  SP0A20 .   sensorID = 0X0A       SLAVE ADDR= 0X42 



#####################################################*/

 
#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
#include <asm/io.h>

#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"
#include "kd_camera_feature.h"

#include "sp0a20yuv_Sensor.h"
#include "sp0a20yuv_Camera_Sensor_para.h"
#include "sp0a20yuv_CameraCustomized.h"

static MSDK_SENSOR_CONFIG_STRUCT SP0A20SensorConfigData;
static struct SP0A20_Sensor_Struct SP0A20_Sensor_Driver;




#define SP0A20YUV_DEBUG
#ifdef SP0A20YUV_DEBUG
#define SENSORDB printk
#else
#define SENSORDB(x,...)
#endif


#define __SENSOR_CONTROL__
#ifdef __SENSOR_CONTROL__
#define CAMERA_CONTROL_FLOW(para1,para2) printk("[%s:%d]::para1=0x%x,para1=0x%x\n\n",__FUNCTION__,__LINE__,para1,para2)
#else
#define CAMERA_CONTROL_FLOW(para1, para2)
#endif

kal_uint8 isBanding = 0; // 0: 50hz  1:60hz

#define SP0A20_NORMAL_Y0ffset  0x20
#define SP0A20_LOWLIGHT_Y0ffset  0x25

extern int iReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 * a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
extern int iWriteRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u16 i2cId);
static void SP0A20_write_cmos_sensor(kal_uint8 addr, kal_uint8 para);
  
static void SP0A20_HVMirror(ACDK_SENSOR_IMAGE_MIRROR_ENUM SensorImageMirror);
#define DEBUG_SENSOR_SP0A20
#ifdef DEBUG_SENSOR_SP0A20
	#define SP0A20_OP_CODE_INI		0x00		/* Initial value. */
	#define SP0A20_OP_CODE_REG		0x01		/* Register */
	#define SP0A20_OP_CODE_DLY		0x02		/* Delay */
	#define SP0A20_OP_CODE_END		0x03		/* End of initial setting. */
	kal_uint16 fromsd;
    
		typedef struct
	{
		u16 init_reg;
		u16 init_val;	/* Save the register value and delay tick */
		u8 op_code;		/* 0 - Initial value, 1 - Register, 2 - Delay, 3 - End of setting. */
	} SP0A20_initial_set_struct;

	SP0A20_initial_set_struct SP0A20_Init_Reg[1000];
	
 u32 strtol(const char *nptr, u8 base)
{
	u8 ret;
	if(!nptr || (base!=16 && base!=10 && base!=8))
	{
		printk("%s(): NULL pointer input\n", __FUNCTION__);
		return -1;
	}
	for(ret=0; *nptr; nptr++)
	{
		if((base==16 && *nptr>='A' && *nptr<='F') || 
			(base==16 && *nptr>='a' && *nptr<='f') || 
			(base>=10 && *nptr>='0' && *nptr<='9') ||
			(base>=8 && *nptr>='0' && *nptr<='7') )
		{
			ret *= base;
			if(base==16 && *nptr>='A' && *nptr<='F')
				ret += *nptr-'A'+10;
			else if(base==16 && *nptr>='a' && *nptr<='f')
				ret += *nptr-'a'+10;
			else if(base>=10 && *nptr>='0' && *nptr<='9')
				ret += *nptr-'0';
			else if(base>=8 && *nptr>='0' && *nptr<='7')
				ret += *nptr-'0';
		}
		else
			return ret;
	}
	return ret;
}

 u8 SP0A20_Initialize_from_T_Flash()
{
	//FS_HANDLE fp = -1;				/* Default, no file opened. */
	//u8 *data_buff = NULL;
	u8 *curr_ptr = NULL;
	u32 file_size = 0;
	//u32 bytes_read = 0;
	u32 i = 0, j = 0;
	u8 func_ind[4] = {0};	/* REG or DLY */

	
	struct file *fp; 
    mm_segment_t fs; 
    loff_t pos = 0; 
	static u8 data_buff[10*1024] ;
 
    fp = filp_open("/mnt/sdcard/sp0a20_sd", O_RDONLY , 0); 
    if (IS_ERR(fp)) { 
        printk("create file error\n");  
        return -1; 
    }   
    fs = get_fs(); 
    set_fs(KERNEL_DS); 

	file_size = vfs_llseek(fp, 0, SEEK_END);
    vfs_read(fp, data_buff, file_size, &pos); 
    //printk("%s %d %d\n", buf,iFileLen,pos); 
	filp_close(fp, NULL); 
    set_fs(fs);
	
	



	/* Start parse the setting witch read from t-flash. */
	curr_ptr = data_buff;
	while (curr_ptr < (data_buff + file_size))
	{
		while ((*curr_ptr == ' ') || (*curr_ptr == '\t'))/* Skip the Space & TAB */
			curr_ptr++;				

		if (((*curr_ptr) == '/') && ((*(curr_ptr + 1)) == '*'))
		{
			while (!(((*curr_ptr) == '*') && ((*(curr_ptr + 1)) == '/')))
			{
				curr_ptr++;		/* Skip block comment code. */
			}

			while (!((*curr_ptr == 0x0D) && (*(curr_ptr+1) == 0x0A)))
			{
				curr_ptr++;
			}

			curr_ptr += 2;						/* Skip the enter line */
			
			continue ;
		}
		
		if (((*curr_ptr) == '/') || ((*curr_ptr) == '{') || ((*curr_ptr) == '}'))		/* Comment line, skip it. */
		{
			while (!((*curr_ptr == 0x0D) && (*(curr_ptr+1) == 0x0A)))
			{
				curr_ptr++;
			}

			curr_ptr += 2;						/* Skip the enter line */

			continue ;
		}
		/* This just content one enter line. */
		if (((*curr_ptr) == 0x0D) && ((*(curr_ptr + 1)) == 0x0A))
		{
			curr_ptr += 2;
			continue ;
		}
		//printk(" curr_ptr1 = %s\n",curr_ptr);
		memcpy(func_ind, curr_ptr, 3);
	
						
		if (strcmp((const char *)func_ind, "REG") == 0)		/* REG */
		{
			curr_ptr += 6;				/* Skip "REG(0x" or "DLY(" */
			SP0A20_Init_Reg[i].op_code = SP0A20_OP_CODE_REG;
			
			SP0A20_Init_Reg[i].init_reg = strtol((const char *)curr_ptr, 16);
			curr_ptr += 5;	/* Skip "00, 0x" */
		
			SP0A20_Init_Reg[i].init_val = strtol((const char *)curr_ptr, 16);
			curr_ptr += 4;	/* Skip "00);" */
		
		}
		else									/* DLY */
		{
			/* Need add delay for this setting. */
			curr_ptr += 4;	
			SP0A20_Init_Reg[i].op_code = SP0A20_OP_CODE_DLY;
			
			SP0A20_Init_Reg[i].init_reg = 0xFF;
			SP0A20_Init_Reg[i].init_val = strtol((const char *)curr_ptr,  10);	/* Get the delay ticks, the delay should less then 50 */
		}
		i++;
		

		/* Skip to next line directly. */
		while (!((*curr_ptr == 0x0D) && (*(curr_ptr+1) == 0x0A)))
		{
			curr_ptr++;
		}
		curr_ptr += 2;
	}

	/* (0xFFFF, 0xFFFF) means the end of initial setting. */
	SP0A20_Init_Reg[i].op_code = SP0A20_OP_CODE_END;
	SP0A20_Init_Reg[i].init_reg = 0xFF;
	SP0A20_Init_Reg[i].init_val = 0xFF;
	i++;
	//for (j=0; j<i; j++)
		//printk(" %x  ==  %x\n",SP0A20_Init_Reg[j].init_reg, SP0A20_Init_Reg[j].init_val);

	/* Start apply the initial setting to sensor. */
	#if 1
	for (j=0; j<i; j++)
	{
		if (SP0A20_Init_Reg[j].op_code == SP0A20_OP_CODE_END)	/* End of the setting. */
		{
			break ;
		}
		else if (SP0A20_Init_Reg[j].op_code == SP0A20_OP_CODE_DLY)
		{
			msleep(SP0A20_Init_Reg[j].init_val);		/* Delay */
		}
		else if (SP0A20_Init_Reg[j].op_code == SP0A20_OP_CODE_REG)
		{
		
			SP0A20_write_cmos_sensor(SP0A20_Init_Reg[j].init_reg, SP0A20_Init_Reg[j].init_val);
		}
		else
		{
			printk("REG ERROR!\n");
		}
	}
#endif


	return 1;	
}

#endif


/*************************************************************************
* FUNCTION
*    SP0A20_write_cmos_sensor
*
* DESCRIPTION
*    This function wirte data to CMOS sensor through I2C
*
* PARAMETERS
*    addr: the 16bit address of register
*    para: the 8bit value of register
*
* RETURNS
*    None
*
* LOCAL AFFECTED
*
*************************************************************************/
static void SP0A20_write_cmos_sensor(kal_uint8 addr, kal_uint8 para)
{
kal_uint8 out_buff[2];

    out_buff[0] = addr;
    out_buff[1] = para;

    iWriteRegI2C((u8*)out_buff , (u16)sizeof(out_buff), SP0A20_WRITE_ID); 

#if (defined(__SP0A20_DEBUG_TRACE__))
  if (sizeof(out_buff) != rt) printk("I2C write %x, %x error\n", addr, para);
#endif
}

/*************************************************************************
* FUNCTION
*    SP0A20_read_cmos_sensor
*
* DESCRIPTION
*    This function read data from CMOS sensor through I2C.
*
* PARAMETERS
*    addr: the 16bit address of register
*
* RETURNS
*    8bit data read through I2C
*
* LOCAL AFFECTED
*
*************************************************************************/
static kal_uint8 SP0A20_read_cmos_sensor(kal_uint8 addr)
{
  kal_uint8 in_buff[1] = {0xFF};
  kal_uint8 out_buff[1];
  
  out_buff[0] = addr;
   
    if (0 != iReadRegI2C((u8*)out_buff , (u16) sizeof(out_buff), (u8*)in_buff, (u16) sizeof(in_buff), SP0A20_WRITE_ID)) {
        SENSORDB("ERROR: SP0A20_read_cmos_sensor \n");
    }

#if (defined(__SP0A20_DEBUG_TRACE__))
  if (size != rt) printk("I2C read %x error\n", addr);
#endif

  return in_buff[0];
}



 static void SP0A20_Set_Dummy(const kal_uint16 iPixels, const kal_uint16 iLines)
{
}   /*  SP0A20_Set_Dummy    */


/*************************************************************************
* FUNCTION
*	SP0A20_write_reg
*
* DESCRIPTION
*	This function set the register of SP0A20.
*
* PARAMETERS
*	addr : the register index of OV76X0
*  para : setting parameter of the specified register of OV76X0
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/

//static void SP0A20_write_reg(kal_uint32 addr, kal_uint32 para)
//{
//	SP0A20_write_cmos_sensor(addr,para);
//}	/* SP0A20_write_reg() */

/*************************************************************************
* FUNCTION
*	ov7670_read_cmos_sensor
*
* DESCRIPTION
*	This function read parameter of specified register from OV76X0.
*
* PARAMETERS
*	addr : the register index of OV76X0
*
* RETURNS
*	the data that read from OV76X0
*
* GLOBALS AFFECTED
*
*************************************************************************/
//static kal_uint32 SP0A20_read_reg(kal_uint32 addr)
//{
//	return (SP0A20_read_cmos_sensor(addr));
//}	/* OV7670_read_reg() */


/*************************************************************************
* FUNCTION
*	SP0A20_NightMode
*
* DESCRIPTION
*	This function night mode of SP0A20.
*
* PARAMETERS
*	bEnable: KAL_TRUE -> enable night mode, otherwise, disable night mode
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static void SP0A20_night_mode(kal_bool bEnable)
{
// kal_uint8 temp = SP0A20_read_cmos_sensor(0x3B);


  if (!SP0A20_Sensor_Driver.MODE_CAPTURE) { 
	if(bEnable)//night mode
	{ 
		   SP0A20_Sensor_Driver.bNight_mode = KAL_TRUE;
		SP0A20_write_cmos_sensor(0xfd,0x01);
		SP0A20_write_cmos_sensor(0xcd,SP0A20_LOWLIGHT_Y0ffset);
		SP0A20_write_cmos_sensor(0xce,0x1f);
	   if(SP0A20_Sensor_Driver.MPEG4_encode_mode == KAL_TRUE)
		{
				if(isBanding== 0)
				{
				printk("video 50Hz night\n");					                     
				 /////video 50Hz 24M 12-12fps         night
				SP0A20_write_cmos_sensor(0xfd,0x00); 
				SP0A20_write_cmos_sensor(0x03,0x01); 
				SP0A20_write_cmos_sensor(0x04,0x68); 
				SP0A20_write_cmos_sensor(0x05,0x00); 
				SP0A20_write_cmos_sensor(0x06,0x00); 
				SP0A20_write_cmos_sensor(0x07,0x00); 
				SP0A20_write_cmos_sensor(0x08,0x00); 
				SP0A20_write_cmos_sensor(0x09,0x04); 
				SP0A20_write_cmos_sensor(0x0a,0x84); 
				SP0A20_write_cmos_sensor(0xfd,0x01); 
				SP0A20_write_cmos_sensor(0xf0,0x00); 
				SP0A20_write_cmos_sensor(0xf7,0x3c); 
				SP0A20_write_cmos_sensor(0x02,0x08); 
				SP0A20_write_cmos_sensor(0x03,0x01); 
				SP0A20_write_cmos_sensor(0x06,0x3c); 
				SP0A20_write_cmos_sensor(0x07,0x00); 
				SP0A20_write_cmos_sensor(0x08,0x01); 
				SP0A20_write_cmos_sensor(0x09,0x00); 
				SP0A20_write_cmos_sensor(0xfd,0x02); 
				SP0A20_write_cmos_sensor(0xbe,0xe0); 
				SP0A20_write_cmos_sensor(0xbf,0x01); 
				SP0A20_write_cmos_sensor(0xd0,0xe0); 
				SP0A20_write_cmos_sensor(0xd1,0x01); 
				//dbg_print(" video 50Hz night\r\n");
				}
				else if(isBanding == 1)
				{
				//video 60Hz 24M 12-12fps      night     
				SP0A20_write_cmos_sensor(0xfd,0x00);
				SP0A20_write_cmos_sensor(0x03,0x01);
				SP0A20_write_cmos_sensor(0x04,0x2c);
				SP0A20_write_cmos_sensor(0x05,0x00);
				SP0A20_write_cmos_sensor(0x06,0x00);
				SP0A20_write_cmos_sensor(0x07,0x00);
				SP0A20_write_cmos_sensor(0x08,0x00);
				SP0A20_write_cmos_sensor(0x09,0x04);
				SP0A20_write_cmos_sensor(0x0a,0x84);
				SP0A20_write_cmos_sensor(0xfd,0x01);
				SP0A20_write_cmos_sensor(0xf0,0x00);
				SP0A20_write_cmos_sensor(0xf7,0x32);
				SP0A20_write_cmos_sensor(0x02,0x0a);
				SP0A20_write_cmos_sensor(0x03,0x01);
				SP0A20_write_cmos_sensor(0x06,0x32);
				SP0A20_write_cmos_sensor(0x07,0x00);
				SP0A20_write_cmos_sensor(0x08,0x01);
				SP0A20_write_cmos_sensor(0x09,0x00);
				SP0A20_write_cmos_sensor(0xfd,0x02);
				SP0A20_write_cmos_sensor(0xbe,0xf4);
				SP0A20_write_cmos_sensor(0xbf,0x01);
				SP0A20_write_cmos_sensor(0xd0,0xf4);
				SP0A20_write_cmos_sensor(0xd1,0x01);
				printk(" video 60Hz night\r\n");
				}
   		  	}	
	    else 
	   {
			//	dbg_print(" SP0A20_banding=%x\r\n",SP0A20_banding);
			       if(isBanding== 0)
				{
				///capture 50Hz 24M 7-12fps      night    
				SP0A20_write_cmos_sensor(0xfd,0x00); 
				SP0A20_write_cmos_sensor(0x03,0x01); 
				SP0A20_write_cmos_sensor(0x04,0x68); 
				SP0A20_write_cmos_sensor(0x05,0x00); 
				SP0A20_write_cmos_sensor(0x06,0x00); 
				SP0A20_write_cmos_sensor(0x07,0x00); 
				SP0A20_write_cmos_sensor(0x08,0x00); 
				SP0A20_write_cmos_sensor(0x09,0x04); 
				SP0A20_write_cmos_sensor(0x0a,0x84); 
				SP0A20_write_cmos_sensor(0xfd,0x01); 
				SP0A20_write_cmos_sensor(0xf0,0x00); 
				SP0A20_write_cmos_sensor(0xf7,0x3c); 
				SP0A20_write_cmos_sensor(0x02,0x0e); 
				SP0A20_write_cmos_sensor(0x03,0x01); 
				SP0A20_write_cmos_sensor(0x06,0x3c); 
				SP0A20_write_cmos_sensor(0x07,0x00); 
				SP0A20_write_cmos_sensor(0x08,0x01); 
				SP0A20_write_cmos_sensor(0x09,0x00); 
				SP0A20_write_cmos_sensor(0xfd,0x02); 
				SP0A20_write_cmos_sensor(0xbe,0x48); 
				SP0A20_write_cmos_sensor(0xbf,0x03); 
				SP0A20_write_cmos_sensor(0xd0,0x48); 
				SP0A20_write_cmos_sensor(0xd1,0x03); 
				printk(" priview 50Hz night\r\n");	
				}  
				else if(isBanding== 1)
				{
				//capture 60Hz 24M 7-12fps           
				SP0A20_write_cmos_sensor(0xfd,0x00); 
				SP0A20_write_cmos_sensor(0x03,0x01); 
				SP0A20_write_cmos_sensor(0x04,0x2c); 
				SP0A20_write_cmos_sensor(0x05,0x00); 
				SP0A20_write_cmos_sensor(0x06,0x00); 
				SP0A20_write_cmos_sensor(0x07,0x00); 
				SP0A20_write_cmos_sensor(0x08,0x00); 
				SP0A20_write_cmos_sensor(0x09,0x04); 
				SP0A20_write_cmos_sensor(0x0a,0x84); 
				SP0A20_write_cmos_sensor(0xfd,0x01); 
				SP0A20_write_cmos_sensor(0xf0,0x00); 
				SP0A20_write_cmos_sensor(0xf7,0x32); 
				SP0A20_write_cmos_sensor(0x02,0x11); 
				SP0A20_write_cmos_sensor(0x03,0x01); 
				SP0A20_write_cmos_sensor(0x06,0x32); 
				SP0A20_write_cmos_sensor(0x07,0x00); 
				SP0A20_write_cmos_sensor(0x08,0x01); 
				SP0A20_write_cmos_sensor(0x09,0x00); 
				SP0A20_write_cmos_sensor(0xfd,0x02); 
				SP0A20_write_cmos_sensor(0xbe,0x52); 
				SP0A20_write_cmos_sensor(0xbf,0x03); 
				SP0A20_write_cmos_sensor(0xd0,0x52); 
				SP0A20_write_cmos_sensor(0xd1,0x03); 
				printk(" priview 60Hz night\r\n");	
				}
			       } 		
	}
	else    // daylight mode
	{
		SP0A20_Sensor_Driver.bNight_mode = KAL_FALSE;
		SP0A20_write_cmos_sensor(0xfd,0x01);
		SP0A20_write_cmos_sensor(0xcd,SP0A20_NORMAL_Y0ffset);
		SP0A20_write_cmos_sensor(0xce,0x1f);
	    if(SP0A20_Sensor_Driver.MPEG4_encode_mode == KAL_TRUE)
	    {
				//dbg_print(" SP0A20_banding=%x\r\n",SP0A20_banding);
				if(isBanding== 0)
				{
				/////video 50Hz 24M 12-12fps        daylight 
				SP0A20_write_cmos_sensor(0xfd,0x00); 
				SP0A20_write_cmos_sensor(0x03,0x01); 
				SP0A20_write_cmos_sensor(0x04,0x68); 
				SP0A20_write_cmos_sensor(0x05,0x00); 
				SP0A20_write_cmos_sensor(0x06,0x00); 
				SP0A20_write_cmos_sensor(0x07,0x00); 
				SP0A20_write_cmos_sensor(0x08,0x00); 
				SP0A20_write_cmos_sensor(0x09,0x04); 
				SP0A20_write_cmos_sensor(0x0a,0x84); 
				SP0A20_write_cmos_sensor(0xfd,0x01); 
				SP0A20_write_cmos_sensor(0xf0,0x00); 
				SP0A20_write_cmos_sensor(0xf7,0x3c); 
				SP0A20_write_cmos_sensor(0x02,0x08); 
				SP0A20_write_cmos_sensor(0x03,0x01); 
				SP0A20_write_cmos_sensor(0x06,0x3c); 
				SP0A20_write_cmos_sensor(0x07,0x00); 
				SP0A20_write_cmos_sensor(0x08,0x01); 
				SP0A20_write_cmos_sensor(0x09,0x00); 
				SP0A20_write_cmos_sensor(0xfd,0x02); 
				SP0A20_write_cmos_sensor(0xbe,0xe0); 
				SP0A20_write_cmos_sensor(0xbf,0x01); 
				SP0A20_write_cmos_sensor(0xd0,0xe0); 
				SP0A20_write_cmos_sensor(0xd1,0x01); 
				printk(" video 50Hz normal\r\n");				
				}
				else if(isBanding == 1)
				{
				//video 60Hz 24M 12-12fps           
				SP0A20_write_cmos_sensor(0xfd,0x00);
				SP0A20_write_cmos_sensor(0x03,0x01);
				SP0A20_write_cmos_sensor(0x04,0x2c);
				SP0A20_write_cmos_sensor(0x05,0x00);
				SP0A20_write_cmos_sensor(0x06,0x00);
				SP0A20_write_cmos_sensor(0x07,0x00);
				SP0A20_write_cmos_sensor(0x08,0x00);
				SP0A20_write_cmos_sensor(0x09,0x04);
				SP0A20_write_cmos_sensor(0x0a,0x84);
				SP0A20_write_cmos_sensor(0xfd,0x01);
				SP0A20_write_cmos_sensor(0xf0,0x00);
				SP0A20_write_cmos_sensor(0xf7,0x32);
				SP0A20_write_cmos_sensor(0x02,0x0a);
				SP0A20_write_cmos_sensor(0x03,0x01);
				SP0A20_write_cmos_sensor(0x06,0x32);
				SP0A20_write_cmos_sensor(0x07,0x00);
				SP0A20_write_cmos_sensor(0x08,0x01);
				SP0A20_write_cmos_sensor(0x09,0x00);
				SP0A20_write_cmos_sensor(0xfd,0x02);
				SP0A20_write_cmos_sensor(0xbe,0xf4);
				SP0A20_write_cmos_sensor(0xbf,0x01);
				SP0A20_write_cmos_sensor(0xd0,0xf4);
				SP0A20_write_cmos_sensor(0xd1,0x01);
				printk(" video 60Hz normal\r\n");	
				}
			   }
		else 
			{
			//	dbg_print(" SP0A20_banding=%x\r\n",SP0A20_banding);
			       if(isBanding== 0)
				{
				///capture 50Hz 24M 7-12fps          
				SP0A20_write_cmos_sensor(0xfd,0x00); 
				SP0A20_write_cmos_sensor(0x03,0x01); 
				SP0A20_write_cmos_sensor(0x04,0x68); 
				SP0A20_write_cmos_sensor(0x05,0x00); 
				SP0A20_write_cmos_sensor(0x06,0x00); 
				SP0A20_write_cmos_sensor(0x07,0x00); 
				SP0A20_write_cmos_sensor(0x08,0x00); 
				SP0A20_write_cmos_sensor(0x09,0x04); 
				SP0A20_write_cmos_sensor(0x0a,0x84); 
				SP0A20_write_cmos_sensor(0xfd,0x01); 
				SP0A20_write_cmos_sensor(0xf0,0x00); 
				SP0A20_write_cmos_sensor(0xf7,0x3c); 
				SP0A20_write_cmos_sensor(0x02,0x0e); 
				SP0A20_write_cmos_sensor(0x03,0x01); 
				SP0A20_write_cmos_sensor(0x06,0x3c); 
				SP0A20_write_cmos_sensor(0x07,0x00); 
				SP0A20_write_cmos_sensor(0x08,0x01); 
				SP0A20_write_cmos_sensor(0x09,0x00); 
				SP0A20_write_cmos_sensor(0xfd,0x02); 
				SP0A20_write_cmos_sensor(0xbe,0x48); 
				SP0A20_write_cmos_sensor(0xbf,0x03); 
				SP0A20_write_cmos_sensor(0xd0,0x48); 
				SP0A20_write_cmos_sensor(0xd1,0x03); 
				printk(" priview 50Hz normal\r\n");
				}
				else if(isBanding== 1)
				{
				//capture 60Hz 24M 7-12fps           
				SP0A20_write_cmos_sensor(0xfd,0x00); 
				SP0A20_write_cmos_sensor(0x03,0x01); 
				SP0A20_write_cmos_sensor(0x04,0x2c); 
				SP0A20_write_cmos_sensor(0x05,0x00); 
				SP0A20_write_cmos_sensor(0x06,0x00); 
				SP0A20_write_cmos_sensor(0x07,0x00); 
				SP0A20_write_cmos_sensor(0x08,0x00); 
				SP0A20_write_cmos_sensor(0x09,0x04); 
				SP0A20_write_cmos_sensor(0x0a,0x84); 
				SP0A20_write_cmos_sensor(0xfd,0x01); 
				SP0A20_write_cmos_sensor(0xf0,0x00); 
				SP0A20_write_cmos_sensor(0xf7,0x32); 
				SP0A20_write_cmos_sensor(0x02,0x11); 
				SP0A20_write_cmos_sensor(0x03,0x01); 
				SP0A20_write_cmos_sensor(0x06,0x32); 
				SP0A20_write_cmos_sensor(0x07,0x00); 
				SP0A20_write_cmos_sensor(0x08,0x01); 
				SP0A20_write_cmos_sensor(0x09,0x00); 
				SP0A20_write_cmos_sensor(0xfd,0x02); 
				SP0A20_write_cmos_sensor(0xbe,0x52); 
				SP0A20_write_cmos_sensor(0xbf,0x03); 
				SP0A20_write_cmos_sensor(0xd0,0x52); 
				SP0A20_write_cmos_sensor(0xd1,0x03); 
				printk(" priview 60Hz normal\r\n");
				}
			       }
	   
	}  
	}
}	/*	SP0A20_NightMode	*/

/*
static void SP0A20_set_isp_driving_current(kal_uint8 current)
{
    //#define CONFIG_BASE      	(0xF0001000)     
//  iowrite32((0xE << 12)|(0 << 28)|0x8880888, 0xF0001500);
}
*/

static void SP0A20_Sensor_Driver_Init(void)
{
  SP0A20_write_cmos_sensor(0xfd,0x00);
  SP0A20_write_cmos_sensor(0x0c,0x01);
  SP0A20_write_cmos_sensor(0x1b,0x27);
  SP0A20_write_cmos_sensor(0x12,0x02);
  SP0A20_write_cmos_sensor(0x13,0x2f);
  SP0A20_write_cmos_sensor(0x6d,0x32);
  SP0A20_write_cmos_sensor(0x6c,0x32);
  SP0A20_write_cmos_sensor(0x6f,0x33);
  SP0A20_write_cmos_sensor(0x6e,0x34);
  SP0A20_write_cmos_sensor(0x16,0x38);
  SP0A20_write_cmos_sensor(0x17,0x38);
  SP0A20_write_cmos_sensor(0x70,0x3a);
  SP0A20_write_cmos_sensor(0x14,0x02);
  SP0A20_write_cmos_sensor(0x15,0x20);
  SP0A20_write_cmos_sensor(0x71,0x23);
  SP0A20_write_cmos_sensor(0x69,0x25);
  SP0A20_write_cmos_sensor(0x6a,0x1a);
  SP0A20_write_cmos_sensor(0x72,0x1c);
  SP0A20_write_cmos_sensor(0x75,0x1e);
  SP0A20_write_cmos_sensor(0x73,0x3c);
  SP0A20_write_cmos_sensor(0x74,0x21);
  SP0A20_write_cmos_sensor(0x79,0x00);
  SP0A20_write_cmos_sensor(0x77,0x10);
  SP0A20_write_cmos_sensor(0x1a,0x4d);
  SP0A20_write_cmos_sensor(0x1c,0x07);
  SP0A20_write_cmos_sensor(0x1e,0x15);
  SP0A20_write_cmos_sensor(0x21,0x0e);
  SP0A20_write_cmos_sensor(0x22,0x28);
  SP0A20_write_cmos_sensor(0x26,0x66);
  SP0A20_write_cmos_sensor(0x28,0x0b);
  SP0A20_write_cmos_sensor(0x37,0x5a);
//pre_gain
  SP0A20_write_cmos_sensor(0xfd,0x02);
  SP0A20_write_cmos_sensor(0x01,0x80);
  SP0A20_write_cmos_sensor(0x52,0x10);
  SP0A20_write_cmos_sensor(0x54,0x00);
//blacklevel
  SP0A20_write_cmos_sensor(0xfd,0x01);//blacklevel
  SP0A20_write_cmos_sensor(0x41,0x00);
  SP0A20_write_cmos_sensor(0x42,0x00);
  SP0A20_write_cmos_sensor(0x43,0x00);
  SP0A20_write_cmos_sensor(0x44,0x00);
///capture 50Hz 24M 7-12fps          
SP0A20_write_cmos_sensor(0xfd,0x00); 
SP0A20_write_cmos_sensor(0x03,0x01); 
SP0A20_write_cmos_sensor(0x04,0xc2);
SP0A20_write_cmos_sensor(0x05,0x00); 
SP0A20_write_cmos_sensor(0x06,0x00); 
SP0A20_write_cmos_sensor(0x07,0x00); 
SP0A20_write_cmos_sensor(0x08,0x00); 
SP0A20_write_cmos_sensor(0x09,0x02);
SP0A20_write_cmos_sensor(0x0a,0xf4);
SP0A20_write_cmos_sensor(0xfd,0x01); 
SP0A20_write_cmos_sensor(0xf0,0x00); 
SP0A20_write_cmos_sensor(0xf7,0x4b);
SP0A20_write_cmos_sensor(0x02,0x0e); 
SP0A20_write_cmos_sensor(0x03,0x01); 
SP0A20_write_cmos_sensor(0x06,0x4b);
SP0A20_write_cmos_sensor(0x07,0x00); 
SP0A20_write_cmos_sensor(0x08,0x01); 
SP0A20_write_cmos_sensor(0x09,0x00); 
SP0A20_write_cmos_sensor(0xfd,0x02); 
SP0A20_write_cmos_sensor(0xbe,0x1a);
SP0A20_write_cmos_sensor(0xbf,0x04);
SP0A20_write_cmos_sensor(0xd0,0x1a);
SP0A20_write_cmos_sensor(0xd1,0x04);


SP0A20_write_cmos_sensor(0xfd,0x01);
SP0A20_write_cmos_sensor(0x5a,0x40);
SP0A20_write_cmos_sensor(0xfd,0x02);
SP0A20_write_cmos_sensor(0xbc,0x70);
SP0A20_write_cmos_sensor(0xbd,0x50);
SP0A20_write_cmos_sensor(0xb8,0x66);
SP0A20_write_cmos_sensor(0xb9,0x8f);
SP0A20_write_cmos_sensor(0xba,0x30);
SP0A20_write_cmos_sensor(0xbb,0x45);
  
  //rpc
  SP0A20_write_cmos_sensor(0xfd,0x01);//rpc                   
  SP0A20_write_cmos_sensor(0xe0,0x44);//SP0A20_write_cmos_sensor(0x4c//rpc_1base_max
  SP0A20_write_cmos_sensor(0xe1,0x36);//SP0A20_write_cmos_sensor(0x3c//rpc_2base_max
  SP0A20_write_cmos_sensor(0xe2,0x30);//SP0A20_write_cmos_sensor(0x34//rpc_3base_max
  SP0A20_write_cmos_sensor(0xe3,0x2a);//SP0A20_write_cmos_sensor(0x2e//rpc_4base_max
  SP0A20_write_cmos_sensor(0xe4,0x2a);//SP0A20_write_cmos_sensor(0x2e//rpc_5base_max
  SP0A20_write_cmos_sensor(0xe5,0x28);//SP0A20_write_cmos_sensor(0x2c//rpc_6base_max
  SP0A20_write_cmos_sensor(0xe6,0x28);//SP0A20_write_cmos_sensor(0x2c//rpc_7base_max
  SP0A20_write_cmos_sensor(0xe7,0x26);//SP0A20_write_cmos_sensor(0x2a//rpc_8base_max
  SP0A20_write_cmos_sensor(0xe8,0x26);//SP0A20_write_cmos_sensor(0x2a//rpc_9base_max
  SP0A20_write_cmos_sensor(0xe9,0x26);//SP0A20_write_cmos_sensor(0x2a//rpc_10base_max
  SP0A20_write_cmos_sensor(0xea,0x24);//SP0A20_write_cmos_sensor(0x28//rpc_11base_max
  SP0A20_write_cmos_sensor(0xf3,0x24);//SP0A20_write_cmos_sensor(0x28//rpc_12base_max
  SP0A20_write_cmos_sensor(0xf4,0x24);//SP0A20_write_cmos_sensor(0x28//rpc_13base_max
//ae min gain  
  SP0A20_write_cmos_sensor(0xfd,0x01);
  SP0A20_write_cmos_sensor(0x04,0xa0);//rpc_max_indr
  SP0A20_write_cmos_sensor(0x05,0x24);//rpc_min_indr 
  SP0A20_write_cmos_sensor(0x0a,0xa0);//rpc_max_outdr
  SP0A20_write_cmos_sensor(0x0b,0x24);//rpc_min_outdr
  
//target
  SP0A20_write_cmos_sensor(0xfd,0x01);
  SP0A20_write_cmos_sensor(0xeb,0x78);//target indr
  SP0A20_write_cmos_sensor(0xec,0x78);//target outdr
  SP0A20_write_cmos_sensor(0xed,0x05);//lock range
  SP0A20_write_cmos_sensor(0xee,0x0c);//hold range

  
//??
  SP0A20_write_cmos_sensor(0xfd,0x01);//??
  SP0A20_write_cmos_sensor(0xf2,0x4d);//
  SP0A20_write_cmos_sensor(0xfd,0x02);//
  SP0A20_write_cmos_sensor(0x5b,0x05);//dp status
  SP0A20_write_cmos_sensor(0x5c,0xa0);//
  
//lens shading
  SP0A20_write_cmos_sensor(0xfd,0x01);
  SP0A20_write_cmos_sensor(0x26,0x80);//
  SP0A20_write_cmos_sensor(0x27,0x4f);//
  SP0A20_write_cmos_sensor(0x28,0x00);//
  SP0A20_write_cmos_sensor(0x29,0x20);//
  SP0A20_write_cmos_sensor(0x2a,0x00);//
  SP0A20_write_cmos_sensor(0x2b,0x03);//
  SP0A20_write_cmos_sensor(0x2c,0x00);//
  SP0A20_write_cmos_sensor(0x2d,0x20);//
  SP0A20_write_cmos_sensor(0x30,0x00);//
  SP0A20_write_cmos_sensor(0x31,0x00);//
 
//lsc 1
  SP0A20_write_cmos_sensor(0xfd,0x01);//lsc 1
  SP0A20_write_cmos_sensor(0xa1,0x31);//2d//2c//r
  SP0A20_write_cmos_sensor(0xa2,0x33);//2f//2c//
  SP0A20_write_cmos_sensor(0xa3,0x2f);//2d//2c//
  SP0A20_write_cmos_sensor(0xa4,0x2f);//2d//2c//
  SP0A20_write_cmos_sensor(0xa5,0x26);//23//2c//g
  SP0A20_write_cmos_sensor(0xa6,0x25);//21//2c//
  SP0A20_write_cmos_sensor(0xa7,0x28);//26//2c//
  SP0A20_write_cmos_sensor(0xa8,0x28);//26//2c//
  SP0A20_write_cmos_sensor(0xa9,0x23);//21//2c//b
  SP0A20_write_cmos_sensor(0xaa,0x23);//21//2c//
  SP0A20_write_cmos_sensor(0xab,0x24);//26//2c//
  SP0A20_write_cmos_sensor(0xac,0x24);//26//2c//
  SP0A20_write_cmos_sensor(0xad,0x0e);//10//12//r
  SP0A20_write_cmos_sensor(0xae,0x0e);//10//12//
  SP0A20_write_cmos_sensor(0xaf,0x0e);//0e//12//
  SP0A20_write_cmos_sensor(0xb0,0x0e);//0e//12//
  SP0A20_write_cmos_sensor(0xb1,0x00);//00//00//g
  SP0A20_write_cmos_sensor(0xb2,0x00);//00//00//
  SP0A20_write_cmos_sensor(0xb3,0x00);//00//00//
  SP0A20_write_cmos_sensor(0xb4,0x00);//00//00//
  SP0A20_write_cmos_sensor(0xb5,0x00);//00//00//b
  SP0A20_write_cmos_sensor(0xb6,0x00);//00//00//
  SP0A20_write_cmos_sensor(0xb7,0x00);//00//00//
  SP0A20_write_cmos_sensor(0xb8,0x00);//00//00//
//skin detect
  SP0A20_write_cmos_sensor(0xfd,0x02);//skin detect
  SP0A20_write_cmos_sensor(0x08,0x00);//
  SP0A20_write_cmos_sensor(0x09,0x06);//
  SP0A20_write_cmos_sensor(0x1d,0x03);//
  SP0A20_write_cmos_sensor(0x1f,0x05);//
//awb
  SP0A20_write_cmos_sensor(0xfd,0x01);//awb
  SP0A20_write_cmos_sensor(0x32,0x00);//
  SP0A20_write_cmos_sensor(0xfd,0x02);//
  SP0A20_write_cmos_sensor(0x26,0xb8);//
  SP0A20_write_cmos_sensor(0x27,0xa3);//
  SP0A20_write_cmos_sensor(0xfd,0x00);
  SP0A20_write_cmos_sensor(0xe7,0x03);
  SP0A20_write_cmos_sensor(0xe7,0x00);
  SP0A20_write_cmos_sensor(0xfd,0x02);
  SP0A20_write_cmos_sensor(0x10,0x00);//
  SP0A20_write_cmos_sensor(0x11,0x00);//
  SP0A20_write_cmos_sensor(0x1b,0x80);//
  SP0A20_write_cmos_sensor(0x1a,0x80);//
  SP0A20_write_cmos_sensor(0x18,0x27);//
  SP0A20_write_cmos_sensor(0x19,0x26);//
  SP0A20_write_cmos_sensor(0x2a,0x00);//
  SP0A20_write_cmos_sensor(0x2b,0x00);//
  SP0A20_write_cmos_sensor(0x28,0xf8);//
  SP0A20_write_cmos_sensor(0x29,0x08);//

//d65 10
  SP0A20_write_cmos_sensor(0x66,0x43);// 4e    d65 10
  SP0A20_write_cmos_sensor(0x67,0x5e);// 6e
  SP0A20_write_cmos_sensor(0x68,0xd2);// d2
  SP0A20_write_cmos_sensor(0x69,0xee);// ee
  SP0A20_write_cmos_sensor(0x6a,0xa5);//

//indoor 11
  SP0A20_write_cmos_sensor(0x7c,0x34);
  SP0A20_write_cmos_sensor(0x7d,0x58);//
  SP0A20_write_cmos_sensor(0x7e,0xfb);//
  SP0A20_write_cmos_sensor(0x7f,0x1d);//
  SP0A20_write_cmos_sensor(0x80,0xa6);//

//cwf   12
  SP0A20_write_cmos_sensor(0x70,0x20);
  SP0A20_write_cmos_sensor(0x71,0x41);//
  SP0A20_write_cmos_sensor(0x72,0x28);//
  SP0A20_write_cmos_sensor(0x73,0x47);//
  SP0A20_write_cmos_sensor(0x74,0xaa);//
 
//tl84  13
  SP0A20_write_cmos_sensor(0x6b,0x00);//tl84  13
  SP0A20_write_cmos_sensor(0x6c,0x20);//
  SP0A20_write_cmos_sensor(0x6d,0x30);//
  SP0A20_write_cmos_sensor(0x6e,0x50);//
  SP0A20_write_cmos_sensor(0x6f,0xaa);//
 
//f    14
  SP0A20_write_cmos_sensor(0x61,0xf2);//f    14
  SP0A20_write_cmos_sensor(0x62,0x13);//
  SP0A20_write_cmos_sensor(0x63,0x50);//
  SP0A20_write_cmos_sensor(0x64,0x70);//
  SP0A20_write_cmos_sensor(0x65,0x6a);//
 
  SP0A20_write_cmos_sensor(0x75,0x80);//
  SP0A20_write_cmos_sensor(0x76,0x09);//
  SP0A20_write_cmos_sensor(0x77,0x02);//
  SP0A20_write_cmos_sensor(0x24,0x25);//
  SP0A20_write_cmos_sensor(0x0e,0x16);//
  SP0A20_write_cmos_sensor(0x3b,0x09);//

// sharp

  SP0A20_write_cmos_sensor(0xfd,0x02);//sharp
  SP0A20_write_cmos_sensor(0xde,0x0f);//
  SP0A20_write_cmos_sensor(0xd7,0x08);// //sharp_flat_thr 轮廓判断
  SP0A20_write_cmos_sensor(0xd8,0x08);//
  SP0A20_write_cmos_sensor(0xd9,0x10);//
  SP0A20_write_cmos_sensor(0xda,0x14);//
  SP0A20_write_cmos_sensor(0xe8,0x20);//sharp_fac_pos 轮廓强度
  SP0A20_write_cmos_sensor(0xe9,0x20);//
  SP0A20_write_cmos_sensor(0xea,0x20);//
  SP0A20_write_cmos_sensor(0xeb,0x20);//
  SP0A20_write_cmos_sensor(0xec,0x20);//sharp_fac_neg
  SP0A20_write_cmos_sensor(0xed,0x20);//
  SP0A20_write_cmos_sensor(0xee,0x20);//
  SP0A20_write_cmos_sensor(0xef,0x20);//
  
  SP0A20_write_cmos_sensor(0xd3,0x20);// sharp_ofst_pos
  SP0A20_write_cmos_sensor(0xd4,0x48);// sharp_ofst_neg
  SP0A20_write_cmos_sensor(0xd5,0x20);// sharp_ofst_min
  SP0A20_write_cmos_sensor(0xd6,0x08);// sharp_k_val
  

  
//skin sharpen        
  SP0A20_write_cmos_sensor(0xfd,0x01);
  SP0A20_write_cmos_sensor(0xd1,0x20);//skin_sharp_delta
  SP0A20_write_cmos_sensor(0xfd,0x02);//
  SP0A20_write_cmos_sensor(0xdc,0x05);//肤色降锐化 skin_sharp_sel
  SP0A20_write_cmos_sensor(0x05,0x20);//排除肤色降`锐化对分辨率卡引起的干扰
    
//BPC
  SP0A20_write_cmos_sensor(0xfd,0x02);
  SP0A20_write_cmos_sensor(0x81,0x00);//bpc_ratio_vt
  SP0A20_write_cmos_sensor(0xfd,0x01);//
  SP0A20_write_cmos_sensor(0xfc,0x00);//bpc_median_en
  SP0A20_write_cmos_sensor(0x7d,0x05);//bpc_med_thr
  SP0A20_write_cmos_sensor(0x7e,0x05);//
  SP0A20_write_cmos_sensor(0x7f,0x09);//
  SP0A20_write_cmos_sensor(0x80,0x08);//
  
//dns
  SP0A20_write_cmos_sensor(0xfd,0x02); 
  SP0A20_write_cmos_sensor(0xdd,0x0f);//enable
  SP0A20_write_cmos_sensor(0xfd,0x01);//沿方向边缘平滑阈值，越小越弱
  
  SP0A20_write_cmos_sensor(0x6d,0x08);//dns_flat_dif 强平滑（平坦）区域平滑阈值  SP0A20_write_cmos_sensor(0x81↓
  SP0A20_write_cmos_sensor(0x6e,0x08);//
  SP0A20_write_cmos_sensor(0x6f,0x10);//
  SP0A20_write_cmos_sensor(0x70,0x18);//
  SP0A20_write_cmos_sensor(0x86,0x18);//dark
  SP0A20_write_cmos_sensor(0x71,0x0a);//dns_edge_dif 弱轮廓（非平坦）区域平滑阈值	 SP0A20_write_cmos_sensor(0x81↑
  SP0A20_write_cmos_sensor(0x72,0x0a);//
  SP0A20_write_cmos_sensor(0x73,0x14);//
  SP0A20_write_cmos_sensor(0x74,0x14);//
  
  SP0A20_write_cmos_sensor(0x75,0x08);//dns_edge_gdif
  SP0A20_write_cmos_sensor(0x76,0x0a);//
  SP0A20_write_cmos_sensor(0x77,0x06);//
  SP0A20_write_cmos_sensor(0x78,0x06);//
  SP0A20_write_cmos_sensor(0x79,0x25);//{raw_flat_fac, raw_edge_fac}  
  SP0A20_write_cmos_sensor(0x7a,0x23);//
  SP0A20_write_cmos_sensor(0x7b,0x22);//
  SP0A20_write_cmos_sensor(0x7c,0x00);//
    
  SP0A20_write_cmos_sensor(0x81,0x0d);//2x//dns_flat_thr 根据增益判定区域阈值
  SP0A20_write_cmos_sensor(0x82,0x18);//4x
  SP0A20_write_cmos_sensor(0x83,0x20);//8x
  SP0A20_write_cmos_sensor(0x84,0x24);//16x
//dem
  SP0A20_write_cmos_sensor(0xfd,0x02);//dem  
  SP0A20_write_cmos_sensor(0x83,0x12);//{dem_morie_thr, dem_hfmax_thr}
  SP0A20_write_cmos_sensor(0x84,0x14);//dem_grad_thr 
  SP0A20_write_cmos_sensor(0x86,0x04);//dem_grad_dif
//pf
  SP0A20_write_cmos_sensor(0xfd,0x01);//pf
  SP0A20_write_cmos_sensor(0x61,0x60);//
  SP0A20_write_cmos_sensor(0x62,0x28);//
  SP0A20_write_cmos_sensor(0x8a,0x10);//

//gamma  
  SP0A20_write_cmos_sensor(0xfd,0x01);//gamma  
  SP0A20_write_cmos_sensor(0x8b,0x00);//00  00
  SP0A20_write_cmos_sensor(0x8c,0x09);//0e  09
  SP0A20_write_cmos_sensor(0x8d,0x17);//1a  13
  SP0A20_write_cmos_sensor(0x8e,0x22);//2b  1b
  SP0A20_write_cmos_sensor(0x8f,0x2e);//38  23
  SP0A20_write_cmos_sensor(0x90,0x42);//4e  32
  SP0A20_write_cmos_sensor(0x91,0x53);//5f  41
  SP0A20_write_cmos_sensor(0x92,0x5f);//6e  4d
  SP0A20_write_cmos_sensor(0x93,0x6d);//7a  5c
  SP0A20_write_cmos_sensor(0x94,0x84);//8e  70
  SP0A20_write_cmos_sensor(0x95,0x95);//9d  84
  SP0A20_write_cmos_sensor(0x96,0xa5);//aa  94
  SP0A20_write_cmos_sensor(0x97,0xb3);//b6  a3
  SP0A20_write_cmos_sensor(0x98,0xc0);//c0  b5
  SP0A20_write_cmos_sensor(0x99,0xcc);//c9  c2
  SP0A20_write_cmos_sensor(0x9a,0xd6);//d3  cf
  SP0A20_write_cmos_sensor(0x9b,0xdf);//dc  d9
  SP0A20_write_cmos_sensor(0x9c,0xe7);//e4  e3
  SP0A20_write_cmos_sensor(0x9d,0xee);//eb  ec
  SP0A20_write_cmos_sensor(0x9e,0xf4);//f3  f4
  SP0A20_write_cmos_sensor(0x9f,0xfa);//fa  fa
  SP0A20_write_cmos_sensor(0xa0,0xff);//ff  ff

//CCM
  SP0A20_write_cmos_sensor(0xfd,0x02);//CCM
  SP0A20_write_cmos_sensor(0x15,0xc0);//d4//b>th
  SP0A20_write_cmos_sensor(0x16,0x8c);//a1//r<th  
  //!F
  SP0A20_write_cmos_sensor(0xa0,0x86);//a6//(红色接近，肤色不理想)   //!F
  SP0A20_write_cmos_sensor(0xa1,0xfa);//f4//
  SP0A20_write_cmos_sensor(0xa2,0x00);//e6//
  SP0A20_write_cmos_sensor(0xa3,0xdb);//f4//
  SP0A20_write_cmos_sensor(0xa4,0xc0);//ac//
  SP0A20_write_cmos_sensor(0xa5,0xe6);//e0//
  SP0A20_write_cmos_sensor(0xa6,0xed);//ed//
  SP0A20_write_cmos_sensor(0xa7,0xda);//cd//
  SP0A20_write_cmos_sensor(0xa8,0xb9);//c6//
  SP0A20_write_cmos_sensor(0xa9,0x0c);//3c//
  SP0A20_write_cmos_sensor(0xaa,0x33);//33//
  SP0A20_write_cmos_sensor(0xab,0x0f);//0f//
           
  SP0A20_write_cmos_sensor(0xac,0x93);//a6//F
  SP0A20_write_cmos_sensor(0xad,0xe7);//da
  SP0A20_write_cmos_sensor(0xae,0x06);//00
  SP0A20_write_cmos_sensor(0xaf,0xda);//c7
  SP0A20_write_cmos_sensor(0xb0,0xcc);//cc
  SP0A20_write_cmos_sensor(0xb1,0xda);//ed
  SP0A20_write_cmos_sensor(0xb2,0xda);//c7
  SP0A20_write_cmos_sensor(0xb3,0xda);//d4
  SP0A20_write_cmos_sensor(0xb4,0xcc);//e6
  SP0A20_write_cmos_sensor(0xb5,0x0c);//0c
  SP0A20_write_cmos_sensor(0xb6,0x33);//33
  SP0A20_write_cmos_sensor(0xb7,0x0f);//0f         
//sat u 
  SP0A20_write_cmos_sensor(0xfd,0x01);//sat u 
  SP0A20_write_cmos_sensor(0xd3,0x66);//
  SP0A20_write_cmos_sensor(0xd4,0x6a);//
  SP0A20_write_cmos_sensor(0xd5,0x56);//
  SP0A20_write_cmos_sensor(0xd6,0x44);//
//sat v                             
  SP0A20_write_cmos_sensor(0xd7,0x66);//     
  SP0A20_write_cmos_sensor(0xd8,0x6a);//
  SP0A20_write_cmos_sensor(0xd9,0x56);//
  SP0A20_write_cmos_sensor(0xda,0x44);//
//auto_sat                          
  SP0A20_write_cmos_sensor(0xfd,0x01);//auto_sat
  SP0A20_write_cmos_sensor(0xdd,0x30);//
  SP0A20_write_cmos_sensor(0xde,0x10);//
  SP0A20_write_cmos_sensor(0xdf,0xff);//a0//y_mean_th
  SP0A20_write_cmos_sensor(0x00,0x00);//status
                                    
//uv_th                             
  SP0A20_write_cmos_sensor(0xfd,0x01);//白色物体表面有彩色噪声降低此值//uv_th
  SP0A20_write_cmos_sensor(0xc2,0xaa);//
  SP0A20_write_cmos_sensor(0xc3,0x88);//
  SP0A20_write_cmos_sensor(0xc4,0x77);//
  SP0A20_write_cmos_sensor(0xc5,0x66);//

//low_lum_offset
  SP0A20_write_cmos_sensor(0xfd,0x01);
  SP0A20_write_cmos_sensor(0xcd,0x10);//
  SP0A20_write_cmos_sensor(0xce,0x1f);//
  SP0A20_write_cmos_sensor(0xcf,0x30);//
  SP0A20_write_cmos_sensor(0xd0,0x45);//

//gw
  SP0A20_write_cmos_sensor(0xfd,0x02);//low_lum_offset
  SP0A20_write_cmos_sensor(0x31,0x60);//
  SP0A20_write_cmos_sensor(0x32,0x60);//
  SP0A20_write_cmos_sensor(0x33,0xc0);//
  SP0A20_write_cmos_sensor(0x35,0x60);//
  SP0A20_write_cmos_sensor(0x37,0x13);//

//heq
  SP0A20_write_cmos_sensor(0xfd,0x01);//heq                         
  SP0A20_write_cmos_sensor(0x0e,0x80);//      
  SP0A20_write_cmos_sensor(0x0f,0x20);//k_max
  SP0A20_write_cmos_sensor(0x10,0x80);//ku_outdoor
  SP0A20_write_cmos_sensor(0x11,0x80);//ku_nr
  SP0A20_write_cmos_sensor(0x12,0x80);//ku_dummy
  SP0A20_write_cmos_sensor(0x13,0x80);//ku_low
  SP0A20_write_cmos_sensor(0x14,0x88);//kl_outdoor 
  SP0A20_write_cmos_sensor(0x15,0x88);//kl_nr      
  SP0A20_write_cmos_sensor(0x16,0x88);//kl_dummy    
  SP0A20_write_cmos_sensor(0x17,0x88);//kl_low        

  SP0A20_write_cmos_sensor(0xfd,0x00);//
  
	if(0 == strncmp(VANZO_SUB_CAM_ROTATION, "180", 3))
	{
		SP0A20_write_cmos_sensor(0x31,0x06);
	} else {
		SP0A20_write_cmos_sensor(0x31,0x00);
	}
  //SP0A20_write_cmos_sensor(0x31,0x06);//

//auto 

  SP0A20_write_cmos_sensor(0xfd,0x01);//auto   
  SP0A20_write_cmos_sensor(0x32,0x15);//
  SP0A20_write_cmos_sensor(0x33,0xef);//
  SP0A20_write_cmos_sensor(0x34,0x07);//
  SP0A20_write_cmos_sensor(0xd2,0x01);//{contrast sat}
  SP0A20_write_cmos_sensor(0xfb,0x25);//
  SP0A20_write_cmos_sensor(0xf2,0x49);//
  SP0A20_write_cmos_sensor(0x35,0x00);//
  SP0A20_write_cmos_sensor(0x5d,0x11);// 
  SP0A20_write_cmos_sensor(0xfd,0x00);//out en

//out en
  SP0A20_write_cmos_sensor(0xfd,0x00);
  SP0A20_write_cmos_sensor(0x1b,0x20);     
}

//shaohui add for DEVINFO CMM
#ifdef SLT_DEVINFO_CMM 
#include  <linux/dev_info.h>
static struct devinfo_struct *s_DEVINFO_ccm;   //suppose 10 max lcm device 

#endif
/*****************************************************************************/
/* Windows Mobile Sensor Interface */
/*****************************************************************************/
/*************************************************************************
* FUNCTION
*	SP0A20Open
*
* DESCRIPTION
*	This function initialize the registers of CMOS sensor
*
* PARAMETERS
*	None
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 SP0A20Open(void)

{
	kal_uint16 sensor_id=0,sensor_id_1=0,sensor_id_2=0;
	int i=0;

    SENSORDB("SP0A20Open_start \n");

//	SP0A20_Sensor_Driver.i2c_clit.addr=SP0A20_WRITE_ID;
//	SP0A20_Sensor_Driver.i2c_clit = i2c_clit;
//    SP0A20_Sensor_Driver.i2c_clit->addr = SP0A20_WRITE_ID;

#if 0 
	SP0A20_write_cmos_sensor(0x12, 0x80);
	mDELAY(10);
#endif 

	// check if sensor ID correct
	 for( ; i<3 ; i++)
	{
        sensor_id_1  =SP0A20_read_cmos_sensor(0x01);
        sensor_id_2=SP0A20_read_cmos_sensor(0x02);
        sensor_id = (sensor_id_1<<8)+sensor_id_2;
		 
        if (sensor_id == SP0A20_SENSOR_ID)
        {
            break;
        }
	}
	
	if (sensor_id != SP0A20_SENSOR_ID) {
	    return ERROR_SENSOR_CONNECT_FAIL;
	}



  memset(&SP0A20_Sensor_Driver, 0, sizeof(struct SP0A20_Sensor_Struct)); 
	SP0A20_Sensor_Driver.MPEG4_encode_mode=KAL_FALSE;
	SP0A20_Sensor_Driver.dummy_pixels=0;
	SP0A20_Sensor_Driver.dummy_lines=0;
	SP0A20_Sensor_Driver.extra_exposure_lines=0;
	SP0A20_Sensor_Driver.exposure_lines=0;
	SP0A20_Sensor_Driver.MODE_CAPTURE=KAL_FALSE;
		
	SP0A20_Sensor_Driver.bNight_mode =KAL_FALSE; // to distinguish night mode or auto mode, default: auto mode setting
	SP0A20_Sensor_Driver.bBanding_value = AE_FLICKER_MODE_50HZ; // to distinguish between 50HZ and 60HZ.
		
	SP0A20_Sensor_Driver.fPV_PCLK = 24; //26;
	SP0A20_Sensor_Driver.iPV_Pixels_Per_Line = 0;

//	SP0A20_set_isp_driving_current(1);
	// initail sequence write in
//    SP0A20_write_cmos_sensor(0x12, 0x80);
    mDELAY(10);
#ifdef DEBUG_SENSOR_SP0A20  //gepeiwei   120903
	//判断手机对应目录下是否有名为sp2528_sd 的文件,没有默认参数

	//介于各种原因，本版本初始化参数在_s_fmt中。
	struct file *fp; 
	mm_segment_t fs; 
	loff_t pos = 0; 
	static char buf[10*1024] ;

	fp = filp_open("/mnt/sdcard/sp0a20_sd", O_RDONLY , 0); 
	if (IS_ERR(fp)) { 
		fromsd = 0;     
		printk("open file error\n");

	} 
	else 
	{
		fromsd = 1;
		printk("open file ok\n");

		//SP0A20_Initialize_from_T_Flash();


		filp_close(fp, NULL); 
		set_fs(fs);
	}

	if(fromsd == 1)//是否从SD读取//gepeiwei   120903
	{
		printk("________________from t!\n");
		SP0A20_Initialize_from_T_Flash();//从SD卡读取的主要函数
	}
	else
	{
		SP0A20_Sensor_Driver_Init();
		//SP0A20_Write_More_Registers();//added for FAE to debut
	}
#else  
	//RETAILMSG(1, (TEXT("Sensor Read ID OK \r\n")));
	// initail sequence write in
	SP0A20_Sensor_Driver_Init();
	//SP0A20_Write_More_Registers();//added for FAE to debut
#endif

	//SP0A20_HVMirror(IMAGE_NORMAL);
   // SP0A20_Sensor_Driver_Init();		
    SENSORDB("SP0A20Open_end \n");
    
    return ERROR_NONE;
}   /* SP0A20Open  */


 
/*************************************************************************
* FUNCTION
*	SP0A20_GetSensorID
*
* DESCRIPTION
*	This function get the sensor ID
*
* PARAMETERS
*	None
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
extern bool camera_pdn1_reverse;
static kal_uint32 SP0A20_GetSensorID(kal_uint32 *sensorID)

{
	kal_uint16 sensor_id=0,sensor_id_1=0,sensor_id_2=0;
	int i=0;
	
    // check if sensor ID correct
    mt_set_gpio_mode(GPIO_CAMERA_CMPDN1_PIN,GPIO_CAMERA_CMPDN1_PIN_M_GPIO);
    mt_set_gpio_dir(GPIO_CAMERA_CMPDN1_PIN,GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_CAMERA_CMPDN1_PIN,GPIO_OUT_ZERO);
    mDELAY(10);
    
    for( ; i<3 ; i++)
	{
        sensor_id_1  =SP0A20_read_cmos_sensor(0x01);
        sensor_id_2=SP0A20_read_cmos_sensor(0x02);
        sensor_id = (sensor_id_1<<8)+sensor_id_2;
        SENSORDB("Sensor Read SP0A20 ID 0x%x OK\n", (unsigned int)sensor_id);
		 
        if (sensor_id == SP0A20_SENSOR_ID)
        {
            *sensorID=sensor_id;
            break;
        }
	}

	if (sensor_id != SP0A20_SENSOR_ID)
	{
	    *sensorID=0xFFFFFFFF;

		return ERROR_SENSOR_CONNECT_FAIL;
	}
   camera_pdn1_reverse = 1;
    return ERROR_NONE;    
}  


/*************************************************************************
* FUNCTION
*	SP0A20Close
*
* DESCRIPTION
*	This function is to turn off sensor module power.
*
* PARAMETERS
*	None
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 SP0A20Close(void)
{
	kal_uint8 tmp1;
   // tmp1 = closed;
	//CAMERA_CONTROL_FLOW(tmp1,closed++);
   SENSORDB("SP0A20Close\n");
	return ERROR_NONE;
}   /* SP0A20Close */


static void SP0A20_HVMirror(ACDK_SENSOR_IMAGE_MIRROR_ENUM SensorImageMirror)
{
}
/*************************************************************************
* FUNCTION
* SP0A20_Preview
*
* DESCRIPTION
*	This function start the sensor preview.
*
* PARAMETERS
*	*image_window : address pointer of pixel numbers in one period of HSYNC
*  *sensor_config_data : address pointer of line numbers in one period of VSYNC
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 SP0A20_Preview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)

{
	SP0A20_Sensor_Driver.fPV_PCLK=24000000;//26000000
	SP0A20_Sensor_Driver.MODE_CAPTURE=KAL_FALSE;

	if(sensor_config_data->SensorOperationMode==MSDK_SENSOR_OPERATION_MODE_VIDEO){
		SP0A20_Sensor_Driver.MPEG4_encode_mode = KAL_TRUE;  // MPEG4 Encode Mode
	}else{
		SP0A20_Sensor_Driver.MPEG4_encode_mode = KAL_FALSE;  
	}


	// SP0A20_HVMirror(sensor_config_data->SensorImageMirror);

//	SP0A20_Sensor_Driver.dummy_pixels = 0;
//	SP0A20_Sensor_Driver.dummy_lines = 42;
//	SP0A20_Sensor_Driver.iPV_Pixels_Per_Line =VGA_PERIOD_PIXEL_NUMS+SP0A20_Sensor_Driver.dummy_pixels;  
//	SP0A20_Set_Dummy(SP0A20_Sensor_Driver.dummy_pixels, SP0A20_Sensor_Driver.dummy_lines);

	
	image_window->GrabStartX= IMAGE_SENSOR_VGA_INSERTED_PIXELS;
	image_window->GrabStartY= IMAGE_SENSOR_VGA_INSERTED_LINES;
	image_window->ExposureWindowWidth = IMAGE_SENSOR_PV_WIDTH;
	image_window->ExposureWindowHeight =IMAGE_SENSOR_PV_HEIGHT;

	if(KAL_TRUE == SP0A20_Sensor_Driver.bNight_mode) // for nd 128 noise,decrease color matrix
	{
	}

	// copy sensor_config_data
	memcpy(&SP0A20SensorConfigData, sensor_config_data, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
	return ERROR_NONE;

}   /*  SP0A20_Preview   */

/*************************************************************************
* FUNCTION
*	SP0A20_Capture
*
* DESCRIPTION
*	This function setup the CMOS sensor in capture MY_OUTPUT mode
*
* PARAMETERS
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
#if 0
static kal_uint32 SP0A20_Capture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
						  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)

{
}   /* OV7576_Capture() */
#endif

static kal_uint32 SP0A20GetResolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *pSensorResolution)
{
kal_uint8 tmp1;
//    tmp1 = res;
//	CAMERA_CONTROL_FLOW(tmp1,res++);

	pSensorResolution->SensorFullWidth=IMAGE_SENSOR_FULL_WIDTH;
	pSensorResolution->SensorFullHeight=IMAGE_SENSOR_FULL_HEIGHT;
    pSensorResolution->SensorPreviewWidth=IMAGE_SENSOR_PV_WIDTH;
	pSensorResolution->SensorPreviewHeight=IMAGE_SENSOR_PV_HEIGHT;
	pSensorResolution->SensorVideoWidth=IMAGE_SENSOR_FULL_WIDTH;
	pSensorResolution->SensorVideoHeight=IMAGE_SENSOR_FULL_HEIGHT;

	return ERROR_NONE;
}	/* SP0A20GetResolution() */
   
static kal_uint32 SP0A20GetInfo(MSDK_SCENARIO_ID_ENUM ScenarioId,
					  MSDK_SENSOR_INFO_STRUCT *pSensorInfo,
					  MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
	
		SENSORDB("SP0A20GetInfo \n");
		pSensorInfo->SensorPreviewResolutionX = IMAGE_SENSOR_PV_WIDTH;
		pSensorInfo->SensorPreviewResolutionY = IMAGE_SENSOR_PV_HEIGHT;
		pSensorInfo->SensorFullResolutionX = IMAGE_SENSOR_PV_WIDTH;
		pSensorInfo->SensorFullResolutionY = IMAGE_SENSOR_PV_HEIGHT;
	
		pSensorInfo->SensorCameraPreviewFrameRate=30;
		pSensorInfo->SensorVideoFrameRate=30;
		pSensorInfo->SensorStillCaptureFrameRate=30;
		pSensorInfo->SensorWebCamCaptureFrameRate=30;
		pSensorInfo->SensorResetActiveHigh=FALSE;
		pSensorInfo->SensorResetDelayCount=1;
		pSensorInfo->SensorDrivingCurrent = ISP_DRIVING_8MA;//ISP_DRIVING_8MA
	
		pSensorInfo->SensorOutputDataFormat=SENSOR_OUTPUT_FORMAT_YVYU;//SENSOR_OUTPUT_FORMAT_YUYV;
		pSensorInfo->SensorClockPolarity=SENSOR_CLOCK_POLARITY_LOW;
		pSensorInfo->SensorClockFallingPolarity=SENSOR_CLOCK_POLARITY_LOW;
		pSensorInfo->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
		pSensorInfo->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_HIGH;
		 //  pSensorInfo->SensorDriver3D = 0;   // the sensor driver is 2D
		pSensorInfo->SensorInterruptDelayLines = 1;
		pSensorInfo->SensroInterfaceType=SENSOR_INTERFACE_TYPE_PARALLEL;//SENSOR_INTERFACE_TYPE_MIPI;
		
		pSensorInfo->SensorMasterClockSwitch = 0; 
	
	
		switch (ScenarioId)
		{
			case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			//case MSDK_SCENARIO_ID_CAMERA_PREVIEW://MSDK_SCENARIO_ID_VIDEO_CAPTURE_MPEG4:
				 
			case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
				
			//case MSDK_SCENARIO_ID_CAMERA_CAPTURE_MEM:
			default:			
				pSensorInfo->SensorClockFreq=24;//26;
				pSensorInfo->SensorClockDividCount= 3;
				pSensorInfo->SensorClockRisingCount= 0;
				pSensorInfo->SensorClockFallingCount= 2;
				pSensorInfo->SensorPixelClockCount= 3;
				pSensorInfo->SensorDataLatchCount= 2;
				pSensorInfo->SensorGrabStartX = 1; 
				pSensorInfo->SensorGrabStartY = 1;		   
				break;
		}

	memcpy(pSensorConfigData, &SP0A20SensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
	
	return ERROR_NONE;
}	/* SP0A20GetInfo() */


static kal_uint32 SP0A20Control(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *pImageWindow,
					  MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
	CAMERA_CONTROL_FLOW(ScenarioId,ScenarioId);

	switch (ScenarioId)
	{
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		//case MSDK_SCENARIO_ID_VIDEO_CAPTURE_MPEG4:
			SP0A20_Preview(pImageWindow, pSensorConfigData);
		break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		//case MSDK_SCENARIO_ID_CAMERA_CAPTURE_MEM:
			//SP0A20_Capture(pImageWindow, pSensorConfigData);
			SP0A20_Preview(pImageWindow, pSensorConfigData);
		break;
		default:
			return ERROR_INVALID_SCENARIO_ID;
	}
	return TRUE;
}	/* MT9P012Control() */



static BOOL SP0A20_set_param_wb(UINT16 para)
{
	kal_uint8  temp_reg;

	if(SP0A20_Sensor_Driver.u8Wb_value==para)
		return FALSE;

	
	SP0A20_Sensor_Driver.u8Wb_value = para;

	switch (para)
	 {
		 case AWB_MODE_OFF:
		 //SP0A20_write_cmos_sensor(0xfd,0x00);				   
		 //SP0A20_write_cmos_sensor(0x32,0x05);	   
		 break;
			 
		 case AWB_MODE_AUTO:
		 SP0A20_write_cmos_sensor(0xfd,0x02);															
		 SP0A20_write_cmos_sensor(0x26,0xbf);																
		 SP0A20_write_cmos_sensor(0x27,0xa3);
		 SP0A20_write_cmos_sensor(0xfd,0x01);	// AUTO 3000K~7000K 	   
		 SP0A20_write_cmos_sensor(0x32,0x15);		
			 break;
	
		 case AWB_MODE_CLOUDY_DAYLIGHT: //cloudy
		 SP0A20_write_cmos_sensor(0xfd,0x01);	 //7000K									 
		 SP0A20_write_cmos_sensor(0x32,0x05);															
		 SP0A20_write_cmos_sensor(0xfd,0x02);															
		 SP0A20_write_cmos_sensor(0x26,0xe2);																
		 SP0A20_write_cmos_sensor(0x27,0x82);																
		 SP0A20_write_cmos_sensor(0xfd,0x00);											   
			 break;
	
		 case AWB_MODE_DAYLIGHT: //sunny
		 // SP0A20_reg_WB_auto	
		 SP0A20_write_cmos_sensor(0xfd,0x01);	//6500K 									
		 SP0A20_write_cmos_sensor(0x32,0x05);															
		 SP0A20_write_cmos_sensor(0xfd,0x02);															
		 SP0A20_write_cmos_sensor(0x26,0xc1);																
		 SP0A20_write_cmos_sensor(0x27,0x88);																
		 SP0A20_write_cmos_sensor(0xfd,0x00);														   
			 break;
	
		 case AWB_MODE_INCANDESCENT: //office
			 // SP0A20_reg_WB_auto 
		 SP0A20_write_cmos_sensor(0xfd,0x01);	//2800K~3000K									  
		 SP0A20_write_cmos_sensor(0x32,0x05);															
		 SP0A20_write_cmos_sensor(0xfd,0x02);															
		 SP0A20_write_cmos_sensor(0x26,0x7b);																
		 SP0A20_write_cmos_sensor(0x27,0xd3);																
		 SP0A20_write_cmos_sensor(0xfd,0x00);															
			 break;
	
		 case AWB_MODE_TUNGSTEN: //home
		 // SP0A20_reg_WB_auto 
		 SP0A20_write_cmos_sensor(0xfd,0x01);	//4000K 								  
		 SP0A20_write_cmos_sensor(0x32,0x05);															
		 SP0A20_write_cmos_sensor(0xfd,0x02);															
		 SP0A20_write_cmos_sensor(0x26,0xb4);																
		 SP0A20_write_cmos_sensor(0x27,0xc4);																
		 SP0A20_write_cmos_sensor(0xfd,0x00);														   
			 break;
			 
		 case AWB_MODE_FLUORESCENT:
		 // SP0A20_reg_WB_auto 
		 SP0A20_write_cmos_sensor(0xfd,0x01);	//4000K 								  
		 SP0A20_write_cmos_sensor(0x32,0x05);															
		 SP0A20_write_cmos_sensor(0xfd,0x02);															
		 SP0A20_write_cmos_sensor(0x26,0xb4);																
		 SP0A20_write_cmos_sensor(0x27,0xc4);																
		 SP0A20_write_cmos_sensor(0xfd,0x00);														   
			 break;
	
		 default:
			 return FALSE;
	 }


	return TRUE;
} /* SP0A20_set_param_wb */


static BOOL SP0A20_set_param_effect(UINT16 para)
{
	kal_uint32 ret = KAL_TRUE;

	if(para==SP0A20_Sensor_Driver.u8Effect_value)
		return FALSE;

	
	SP0A20_Sensor_Driver.u8Effect_value = para;
	//SP0A20_write_cmos_sensor(0xfd, 0x01);  
	//SP0A20_write_cmos_sensor(0x36, 0x02);  

    switch (para)
    {
        case MEFFECT_OFF:  
		SP0A20_write_cmos_sensor(0xfd, 0x01);  
		SP0A20_write_cmos_sensor(0x66, 0x00);
		SP0A20_write_cmos_sensor(0x67, 0x80);
		SP0A20_write_cmos_sensor(0x68, 0x80);
            break;

        case MEFFECT_SEPIA:  
		SP0A20_write_cmos_sensor(0xfd, 0x01);
		SP0A20_write_cmos_sensor(0x66, 0x10);
		SP0A20_write_cmos_sensor(0x67, 0xc0);
		SP0A20_write_cmos_sensor(0x68, 0x20);

            break;

        case MEFFECT_NEGATIVE: 
		SP0A20_write_cmos_sensor(0xfd, 0x01);
		SP0A20_write_cmos_sensor(0x66, 0x08);
		SP0A20_write_cmos_sensor(0x67, 0x80);
		SP0A20_write_cmos_sensor(0x68, 0x80);
            break;

        case MEFFECT_SEPIAGREEN:
		SP0A20_write_cmos_sensor(0xfd, 0x01);
		SP0A20_write_cmos_sensor(0x66, 0x10);
		SP0A20_write_cmos_sensor(0x67, 0x60);  //20
		SP0A20_write_cmos_sensor(0x68, 0x60);  //20
            break;

        case MEFFECT_SEPIABLUE:
		SP0A20_write_cmos_sensor(0xfd, 0x01);
		SP0A20_write_cmos_sensor(0x66, 0x10);
		SP0A20_write_cmos_sensor(0x67, 0x20);
		SP0A20_write_cmos_sensor(0x68, 0xd0);  //f0

            break;
			
		case MEFFECT_MONO: //B&W
		SP0A20_write_cmos_sensor(0xfd, 0x01);
		SP0A20_write_cmos_sensor(0x66, 0x20);  //08
		SP0A20_write_cmos_sensor(0x67, 0x80);
		SP0A20_write_cmos_sensor(0x68, 0x80);

			break;
        default:
            return FALSE;
    }
	SP0A20_write_cmos_sensor(0xfd, 0x00);  
	SP0A20_write_cmos_sensor(0xe7, 0x03);  
	SP0A20_write_cmos_sensor(0xe7, 0x00);
	//SP0A20_write_cmos_sensor(0xfd, 0x01);
	//SP0A20_write_cmos_sensor(0x36, 0x00);  

	return ret;

} /* SP0A20_set_param_effect */

static void SP0A20_set_banding_for_50Hz(void)
{
printk("SP0A20_set_banding_for_50Hz\n");

}


static void SP0A20_set_banding_for_60Hz(void)
{
printk("SP0A20_set_banding_for_60Hz\n");

}

static BOOL SP0A20_set_param_banding(UINT16 para)
{
	//if(SP0A20_Sensor_Driver.bBanding_value == para)
	//	return TRUE;
	
	SP0A20_Sensor_Driver.bBanding_value = para;
	
	switch (para)
	{
		case AE_FLICKER_MODE_50HZ:
			isBanding = 0;
			printk("SP0A20_set_param_banding_50hz\n");
			//SP0A20_set_banding_for_50Hz();
			break;
		case AE_FLICKER_MODE_60HZ:
			isBanding = 1;
			printk("SP0A20_set_param_banding_60hz\n");
			//SP0A20_set_banding_for_60Hz();
			break;
		default:
			return FALSE;
	}

	return TRUE;
} /* SP0A20_set_param_banding */
static BOOL SP0A20_set_param_exposure(UINT16 para)
{
	if(para == SP0A20_Sensor_Driver.u8Ev_value)
		return FALSE;

	SP0A20_Sensor_Driver.u8Ev_value = para;

    switch (para)
    {
        case AE_EV_COMP_n13:
		SP0A20_write_cmos_sensor(0xfd, 0x01);
		SP0A20_write_cmos_sensor(0xdb, 0xc0);
            break;

        case AE_EV_COMP_n10:
		SP0A20_write_cmos_sensor(0xfd, 0x01);
		SP0A20_write_cmos_sensor(0xdb, 0xd0);
            break;

        case AE_EV_COMP_n07:
		SP0A20_write_cmos_sensor(0xfd, 0x01);
		SP0A20_write_cmos_sensor(0xdb, 0xe0);
            break;

        case AE_EV_COMP_n03:
		SP0A20_write_cmos_sensor(0xfd, 0x01);
		SP0A20_write_cmos_sensor(0xdb, 0xf0);
            break;

        case AE_EV_COMP_00:
		SP0A20_write_cmos_sensor(0xfd, 0x01);
		SP0A20_write_cmos_sensor(0xdb, 0x00);//0xfa before
            break;

        case AE_EV_COMP_03:
		SP0A20_write_cmos_sensor(0xfd, 0x01);
		SP0A20_write_cmos_sensor(0xdb, 0x10);
            break;

        case AE_EV_COMP_07:
		SP0A20_write_cmos_sensor(0xfd, 0x01);
		SP0A20_write_cmos_sensor(0xdb, 0x20);
            break;

        case AE_EV_COMP_10:
		SP0A20_write_cmos_sensor(0xfd, 0x01);
		SP0A20_write_cmos_sensor(0xdb, 0x30);
            break;

        case AE_EV_COMP_13:
		SP0A20_write_cmos_sensor(0xfd, 0x01);
		SP0A20_write_cmos_sensor(0xdb, 0x40);
            break;

        default:
            return FALSE;
    }


	return TRUE;
} /* SP0A20_set_param_exposure */

static kal_uint32 SP0A20_YUVSensorSetting(FEATURE_ID iCmd, UINT16 iPara)
{
#ifdef DEBUG_SENSOR_SP0A20
	return TRUE;
#endif
	switch (iCmd) {
		case FID_SCENE_MODE:
		    if (iPara == SCENE_MODE_OFF){
		        SP0A20_night_mode(FALSE); 
		    }else if (iPara == SCENE_MODE_NIGHTSCENE){
               SP0A20_night_mode(TRUE); 
		    }	    
		  
		    break; 
		case FID_AWB_MODE:
			SP0A20_set_param_wb(iPara);
		break;
		case FID_COLOR_EFFECT:
			SP0A20_set_param_effect(iPara);
		break;
		case FID_AE_EV:	
			SP0A20_set_param_exposure(iPara);
		break;
		case FID_AE_FLICKER:
			SP0A20_set_param_banding(iPara);
			//whl120717 test
			 if (SP0A20_Sensor_Driver.bNight_mode == KAL_FALSE){
		        SP0A20_night_mode(FALSE); 
		    }else if (SP0A20_Sensor_Driver.bNight_mode == KAL_TRUE){
               	SP0A20_night_mode(TRUE); 
        
		    }	
		      
		break;
		default:
		break;
	}
	
	return TRUE;
}   /* SP0A20_YUVSensorSetting */

static kal_uint32 SP0A20_YUVSetVideoMode(UINT16 u2FrameRate)
{
    kal_uint8 temp ;//= SP0A20_read_cmos_sensor(0x3B);
    SP0A20_Sensor_Driver.MPEG4_encode_mode = KAL_TRUE; 

    if (u2FrameRate == 30)
    {
    }
    else if (u2FrameRate == 15)       
    {
    }
    else 
    {
        printk("Wrong frame rate setting \n");
    }   
    
	printk("\n SP0A20_YUVSetVideoMode:u2FrameRate=%d\n\n",u2FrameRate);
    return TRUE;
}

UINT32 SP0A20SetSoftwarePWDNMode(kal_bool bEnable)
{
    return TRUE;
}

/*************************************************************************
* FUNCTION
*    SP0A20_get_size
*
* DESCRIPTION
*    This function return the image width and height of image sensor.
*
* PARAMETERS
*    *sensor_width: address pointer of horizontal effect pixels of image sensor
*    *sensor_height: address pointer of vertical effect pixels of image sensor
*
* RETURNS
*    None
*
* LOCAL AFFECTED
*
*************************************************************************/
static void SP0A20_get_size(kal_uint16 *sensor_width, kal_uint16 *sensor_height)
{
  *sensor_width = IMAGE_SENSOR_FULL_WIDTH; /* must be 4:3 */
  *sensor_height = IMAGE_SENSOR_FULL_HEIGHT;
}

/*************************************************************************
* FUNCTION
*    SP0A20_get_period
*
* DESCRIPTION
*    This function return the image width and height of image sensor.
*
* PARAMETERS
*    *pixel_number: address pointer of pixel numbers in one period of HSYNC
*    *line_number: address pointer of line numbers in one period of VSYNC
*
* RETURNS
*    None
*
* LOCAL AFFECTED
*
*************************************************************************/
static void SP0A20_get_period(kal_uint16 *pixel_number, kal_uint16 *line_number)
{
  *pixel_number = VGA_PERIOD_PIXEL_NUMS+SP0A20_Sensor_Driver.dummy_pixels;
  *line_number = VGA_PERIOD_LINE_NUMS+SP0A20_Sensor_Driver.dummy_lines;
}

/*************************************************************************
* FUNCTION
*    SP0A20_feature_control
*
* DESCRIPTION
*    This function control sensor mode
*
* PARAMETERS
*    id: scenario id
*    image_window: image grab window
*    cfg_data: config data
*
* RETURNS
*    error code
*
* LOCAL AFFECTED
*
*************************************************************************/
static kal_uint32 SP0A20FeatureControl(MSDK_SENSOR_FEATURE_ENUM id, kal_uint8 *para, kal_uint32 *len)
{
	UINT32 *pFeatureData32=(UINT32 *) para;

	switch (id)
	{
		case SENSOR_FEATURE_GET_RESOLUTION: /* no use */
			SP0A20_get_size((kal_uint16 *)para, (kal_uint16 *)(para + sizeof(kal_uint16)));
			*len = sizeof(kal_uint32);
			break;
		case SENSOR_FEATURE_GET_PERIOD:
			SP0A20_get_period((kal_uint16 *)para, (kal_uint16 *)(para + sizeof(kal_uint16)));
			*len = sizeof(kal_uint32);
			break;
		case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
			*(kal_uint32 *)para = SP0A20_Sensor_Driver.fPV_PCLK;
			*len = sizeof(kal_uint32);
			break;
		case SENSOR_FEATURE_SET_ESHUTTER:
			break;
		case SENSOR_FEATURE_SET_NIGHTMODE: 
#ifndef DEBUG_SENSOR_SP0A20      
			SP0A20_night_mode((kal_bool)*(kal_uint16 *)para);
#endif
			break;
		case SENSOR_FEATURE_SET_GAIN:
		case SENSOR_FEATURE_SET_FLASHLIGHT:
		case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
			break;
		case SENSOR_FEATURE_SET_REGISTER:
			SP0A20_write_cmos_sensor(((MSDK_SENSOR_REG_INFO_STRUCT *)para)->RegAddr, ((MSDK_SENSOR_REG_INFO_STRUCT *)para)->RegData);
			break;
		case SENSOR_FEATURE_GET_REGISTER: /* 10 */
			((MSDK_SENSOR_REG_INFO_STRUCT *)para)->RegData = SP0A20_read_cmos_sensor(((MSDK_SENSOR_REG_INFO_STRUCT *)para)->RegAddr);
			break;
		case SENSOR_FEATURE_SET_CCT_REGISTER:
			memcpy(&SP0A20_Sensor_Driver.eng.CCT, para, sizeof(SP0A20_Sensor_Driver.eng.CCT));
			break;
		case SENSOR_FEATURE_GET_CCT_REGISTER:
		case SENSOR_FEATURE_SET_ENG_REGISTER:
		case SENSOR_FEATURE_GET_ENG_REGISTER:
		case SENSOR_FEATURE_GET_REGISTER_DEFAULT:
		case SENSOR_FEATURE_GET_CONFIG_PARA: /* no use */
			break;
		case SENSOR_FEATURE_CAMERA_PARA_TO_SENSOR:
			break;
		case SENSOR_FEATURE_SENSOR_TO_CAMERA_PARA:
			break;
		case SENSOR_FEATURE_GET_GROUP_COUNT:
		case SENSOR_FEATURE_GET_GROUP_INFO: /* 20 */
		case SENSOR_FEATURE_GET_ITEM_INFO:
		case SENSOR_FEATURE_SET_ITEM_INFO:
		case SENSOR_FEATURE_GET_ENG_INFO:
			break;
		case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
		/*
		* get the lens driver ID from EEPROM or just return LENS_DRIVER_ID_DO_NOT_CARE
		* if EEPROM does not exist in camera module.
		*/
			*(kal_uint32 *)para = LENS_DRIVER_ID_DO_NOT_CARE;
			*len = sizeof(kal_uint32);
			break;
		case SENSOR_FEATURE_SET_YUV_CMD:
	//		SP0A20_YUVSensorSetting((FEATURE_ID)(UINT32 *)para, (UINT32 *)(para+1));
			
			SP0A20_YUVSensorSetting((FEATURE_ID)*pFeatureData32, *(pFeatureData32+1));
			break;
#if 0		    		
		case SENSOR_FEATURE_QUERY:
			SP0A20_Query(pSensorFeatureInfo);
			*pFeatureParaLen = sizeof(MSDK_FEATURE_INFO_STRUCT);
			break;		
		case SENSOR_FEATURE_SET_YUV_CAPTURE_RAW_SUPPORT:
			/* update yuv capture raw support flag by *pFeatureData16 */
			break;		
#endif 			
		case SENSOR_FEATURE_SET_VIDEO_MODE:
			SP0A20_YUVSetVideoMode(*para);
			break;
              case SENSOR_FEATURE_CHECK_SENSOR_ID:
                     SP0A20_GetSensorID(pFeatureData32); 
                     break; 	
              case SENSOR_FEATURE_SET_SOFTWARE_PWDN:
                     SP0A20SetSoftwarePWDNMode((BOOL)*pFeatureData32);        	        	
                     break;
		default:
			break;
	}
	return ERROR_NONE;
}




#if 0
image_sensor_func_struct image_sensor_driver_SP0A20=
{
	SP0A20Open,
	SP0A20Close,
	SP0A20GetResolution,
	SP0A20GetInfo,
	SP0A20Control,
	SP0A20FeatureControl
};
void image_sensor_func_config(void)
{
	extern image_sensor_func_struct *image_sensor_driver;

	image_sensor_driver = &image_sensor_driver_SP0A20;
}

#endif

SENSOR_FUNCTION_STRUCT	SensorFuncSP0A20=
{
	SP0A20Open,
	SP0A20GetInfo,
	SP0A20GetResolution,
	SP0A20FeatureControl,
	SP0A20Control,
	SP0A20Close
};

UINT32 SP0A20_YUV_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{

	/* To Do : Check Sensor status here */
	if (pfFunc!=NULL)
		*pfFunc=&SensorFuncSP0A20;

	return ERROR_NONE;
}	/* SensorInit() */

