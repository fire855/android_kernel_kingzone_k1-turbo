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

#include "sp5409mipiraw_Sensor.h"
#include "sp5409mipiraw_Camera_Sensor_para.h"
#include "sp5409mipiraw_CameraCustomized.h"

kal_bool  SP5409MIPI_MPEG4_encode_mode = KAL_FALSE;
kal_bool SP5409MIPI_Auto_Flicker_mode = KAL_FALSE;
static kal_bool ONLINE_DEBUG_BZW = KAL_TRUE;


kal_uint8 SP5409MIPI_sensor_write_I2C_address = SP5409MIPI_WRITE_ID;
kal_uint8 SP5409MIPI_sensor_read_I2C_address = SP5409MIPI_READ_ID;

//#define ONLINE_DEBUG //在线adb调试
//#define DEBUG_SENSOR //T卡调试
#ifdef DEBUG_SENSOR
	#define SP5409MIPI_OP_CODE_INI		0x00		/* Initial value. */
	#define SP5409MIPI_OP_CODE_REG		0x01		/* Register */
	#define SP5409MIPI_OP_CODE_DLY		0x02		/* Delay */
	#define SP5409MIPI_OP_CODE_END		0x03		/* End of initial setting. */
	

		typedef struct
	{
		u16 init_reg;
		u16 init_val;	/* Save the register value and delay tick */
		u8 op_code;		/* 0 - Initial value, 1 - Register, 2 - Delay, 3 - End of setting. */
	} SP5409MIPI_initial_set_struct;

	SP5409MIPI_initial_set_struct SP5409MIPI_Init_Reg[1000];
	UINT32 fromsd;//gpwdebug
	
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

 u8 SP5409MIPI_Initialize_from_T_Flash()
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
 
	fp = filp_open("/mnt/sdcard/sp5409_sd", O_RDONLY , 0); 
	if (IS_ERR(fp)) 
	{ 
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
			SP5409MIPI_Init_Reg[i].op_code = SP5409MIPI_OP_CODE_REG;
			
			SP5409MIPI_Init_Reg[i].init_reg = strtol((const char *)curr_ptr, 16);
			curr_ptr += 5;	/* Skip "00, 0x" */
		
			SP5409MIPI_Init_Reg[i].init_val = strtol((const char *)curr_ptr, 16);
			curr_ptr += 4;	/* Skip "00);" */
		
		}
		else									/* DLY */
		{
			/* Need add delay for this setting. */
			curr_ptr += 4;	
			SP5409MIPI_Init_Reg[i].op_code = SP5409MIPI_OP_CODE_DLY;
			
			SP5409MIPI_Init_Reg[i].init_reg = 0xFF;
			SP5409MIPI_Init_Reg[i].init_val = strtol((const char *)curr_ptr,  10);	/* Get the delay ticks, the delay should less then 50 */
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
	SP5409MIPI_Init_Reg[i].op_code = SP5409MIPI_OP_CODE_END;
	SP5409MIPI_Init_Reg[i].init_reg = 0xFF;
	SP5409MIPI_Init_Reg[i].init_val = 0xFF;
	i++;
	//for (j=0; j<i; j++)
		//printk(" %x  ==  %x\n",SP5409MIPI_Init_Reg[j].init_reg, SP5409MIPI_Init_Reg[j].init_val);

	/* Start apply the initial setting to sensor. */
	#if 1
	for (j=0; j<i; j++)
	{
		if (SP5409MIPI_Init_Reg[j].op_code == SP5409MIPI_OP_CODE_END)	/* End of the setting. */
		{
			break ;
		}
		else if (SP5409MIPI_Init_Reg[j].op_code == SP5409MIPI_OP_CODE_DLY)
		{
			msleep(SP5409MIPI_Init_Reg[j].init_val);		/* Delay */
		}
		else if (SP5409MIPI_Init_Reg[j].op_code == SP5409MIPI_OP_CODE_REG)
		{
		
			SP5409MIPI_write_cmos_sensor(SP5409MIPI_Init_Reg[j].init_reg, SP5409MIPI_Init_Reg[j].init_val);
			printk("%x = %x\n",SP5409MIPI_Init_Reg[j].init_reg,SP5409MIPI_Init_Reg[j].init_val);
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

#ifdef ONLINE_DEBUG//for debug online
static u32 cur_reg=0;
static u8 cur_val;
static u32 debug_id = 0;

static ssize_t fcam_show(struct device *dev, struct device_attribute *attr, char *_buf)
{
	return sprintf(_buf, "0x%02x=0x%02x id = %x\n", cur_reg, cur_val,debug_id);
}
#ifndef DEBUG_SENSOR
static u32 strtol(const char *nptr, int base)
{
	u32 ret;
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
#endif

static ssize_t fcam_store(struct device *dev,
					struct device_attribute *attr,
					const char *_buf, size_t _count)
{
	const char * p=_buf;
	uint32_t reg;
	uint16_t val;
	uint16_t tmp;

	if(!strncmp(_buf, "get", strlen("get")))
	{
		p+=strlen("get");
		cur_reg=(u32)strtol(p, 16);
		
		//msm_camera_i2c_read(ov7695_s_ctrl.sensor_i2c_client, cur_reg, &val,MSM_CAMERA_I2C_BYTE_DATA);
		val=SP5409MIPI_read_cmos_sensor(cur_reg);//hanlei
		
		printk("%s(): get 0x%02x=0x%02x\n", __FUNCTION__, cur_reg, val);
		cur_val=val;
	}
	else if(!strncmp(_buf, "put", strlen("put")))
	{
		p+=strlen("put");
		reg=strtol(p, 16);
	
		p=strchr(_buf, '=');
		if(p)
		{
			++ p;
			val=strtol(p, 16);
			
			//msm_camera_i2c_write(ov7695_s_ctrl.sensor_i2c_client, reg, val,MSM_CAMERA_I2C_BYTE_DATA);
			SP5409MIPI_write_cmos_sensor(reg,val);//hanlei
			//msm_camera_i2c_read(ov7695_s_ctrl.sensor_i2c_client, reg, &tmp,MSM_CAMERA_I2C_BYTE_DATA);
			tmp=SP5409MIPI_read_cmos_sensor(reg);
			
			printk("%s(): set 0x%02x=0x%02x==(0x%02x)\n", __FUNCTION__, reg, val,tmp);
		}
		else
			printk("%s(): Bad string format input!\n", __FUNCTION__);
	}
	else
		printk("%s(): Bad string format input!\n", __FUNCTION__);
	
	return _count;
} 

static ssize_t currreg_show(struct device *dev, struct device_attribute *attr, char *_buf)
{
    strcpy(_buf, "SP5409");
    return 4;
}

static struct device *fcam_dev = NULL;
static struct class *  fcam_class = NULL;
static DEVICE_ATTR(fcam, 0666, fcam_show, fcam_store);
static DEVICE_ATTR(currreg, 0666, currreg_show, NULL);
#endif



//#define SP5409MIPI_USE_OTP

#ifdef SP5409MIPI_USE_OTP
static uint16_t used_otp = 0;
static uint16_t ret = -1;
extern int sp5409_update_otp_wb(void);
extern int sp5409_update_awb_gain(void);
extern int sp5409_check_mid(uint mid);
#endif
	
static struct SP5409MIPI_sensor_STRUCT SP5409MIPI_sensor={SP5409MIPI_WRITE_ID,SP5409MIPI_READ_ID,KAL_TRUE,KAL_FALSE,KAL_TRUE,KAL_FALSE,
KAL_FALSE,KAL_FALSE,KAL_FALSE,32000000,32000000,32000000,1,1,1,16,16,16,SP5409MIPI_PV_LINE_LENGTH_PIXELS,SP5409MIPI_PV_FRAME_LENGTH_LINES,
SP5409MIPI_VIDEO_LINE_LENGTH_PIXELS,SP5409MIPI_VIDEO_FRAME_LENGTH_LINES,SP5409MIPI_FULL_LINE_LENGTH_PIXELS,SP5409MIPI_FULL_FRAME_LENGTH_LINES,0,16,0,16,0,16,18};
static MSDK_SCENARIO_ID_ENUM CurrentScenarioId = MSDK_SCENARIO_ID_CAMERA_PREVIEW;	
kal_uint16	SP5409MIPI_sensor_gain_base=0x10;//0x00//hanlei
/* MAX/MIN Explosure Lines Used By AE Algorithm */
kal_uint16 SP5409MIPI_MAX_EXPOSURE_LINES = SP5409MIPI_PV_FRAME_LENGTH_LINES-5;//3;//650;//5//hanlei
kal_uint8  SP5409MIPI_MIN_EXPOSURE_LINES = 2;//1//2//hanlei
kal_uint32 SP5409MIPI_isp_master_clock;
static DEFINE_SPINLOCK(sp5409_drv_lock);

#define SENSORDB(fmt, arg...) printk( "[SP5409MIPIRaw] "  fmt, ##arg)
#define RETAILMSG(x,...)
#define TEXT
UINT8 SP5409MIPIPixelClockDivider=0;
kal_uint16 SP5409MIPI_sensor_id=0;
MSDK_SENSOR_CONFIG_STRUCT SP5409MIPISensorConfigData;
kal_uint32 SP5409MIPI_FAC_SENSOR_REG;
kal_uint16 SP5409MIPI_sensor_flip_value; 
/* FIXME: old factors and DIDNOT use now. s*/
SENSOR_REG_STRUCT SP5409MIPISensorCCT[]=CAMERA_SENSOR_CCT_DEFAULT_VALUE;
SENSOR_REG_STRUCT SP5409MIPISensorReg[ENGINEER_END]=CAMERA_SENSOR_REG_DEFAULT_VALUE;
/* FIXME: old factors and DIDNOT use now. e*/
extern int iReadReg(u16 a_u2Addr , u8 * a_puBuff , u16 i2cId);
extern int iWriteReg(u16 a_u2Addr , u32 a_u4Data , u32 a_u4Bytes , u16 i2cId);
//#define SP5409MIPI_write_cmos_sensor(addr, para) iWriteReg((u16) addr , (u32) para , 1, SP5409MIPI_WRITE_ID)

extern int iReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 * a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
extern int iWriteRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u16 i2cId);




/*kal_uint16 SP5409MIPI_read_cmos_sensor(kal_uint32 addr)
{
    kal_uint16 get_byte=0;
    iReadReg((u16) addr ,(u8*)&get_byte,SP5409MIPI_WRITE_ID);
    return get_byte;
}*/

//for test 5409
kal_uint16 SP5409MIPI_write_cmos_sensor(kal_uint32 addr, kal_uint32 para)
{
	char puSendCmd[2] = { (char)(addr & 0xFF) ,(char)(para & 0xFF)};

	//SENSORDB("HCY IIC 8 8");  
	
	iWriteRegI2C(puSendCmd , 2,0x78);
	return TRUE;

}

kal_uint16 SP5409MIPI_read_cmos_sensor(kal_uint32 addr)
{
	kal_uint16 get_byte=0;
	char puSendCmd[1] = { (char)(addr & 0xFF) };
	iReadRegI2C(puSendCmd , 1, (u8*)&get_byte,1,0x78);
	return get_byte;
}
//end for test 5409

void SP5409MIPI_write_shutter(kal_uint16 shutter)
{
	SENSORDB("[SP5409MIPI]enter SP5409MIPI_write_shutter function\n"); 
	kal_uint32 frame_length = 0,line_length=0,shutter1=0;
    	kal_uint32 extra_lines = 0;
	kal_uint32 max_exp_shutter = 0;
	unsigned long flags;	
#if 0
SP5409MIPI_write_cmos_sensor(0xfd, 0x01);
SP5409MIPI_write_cmos_sensor(0x03, (2098 >> 8) & 0xFF);
SP5409MIPI_write_cmos_sensor(0x04, 2098  & 0xFF);

SP5409MIPI_write_cmos_sensor(0x24,0xa0);

SP5409MIPI_write_cmos_sensor(0x01, 0x01); 

#endif

	

	 //shutter=0x5300;



	 SP5409MIPI_write_cmos_sensor(0xfd, 0x01);
	 SP5409MIPI_write_cmos_sensor(0x03, (shutter >> 8) & 0xFF);
	 SP5409MIPI_write_cmos_sensor(0x04, shutter  & 0xFF);
	 SP5409MIPI_write_cmos_sensor(0x01, 0x01); 
		
	 
	
	 SENSORDB("hcy  SP5409MIPI_write_shutter  =%d \n" , shutter );
}   /* write_SP5409MIPI_shutter */

static kal_uint16 SP5409MIPIReg2Gain(const kal_uint8 iReg)
{
	SENSORDB("[SP5409MIPI]enter SP5409MIPIReg2Gain function\n");

	SENSORDB("[SP5409MIPI]exit SP5409MIPIReg2Gain function\n");
	
       return ;
}
static kal_uint8 SP5409MIPIGain2Reg(const kal_uint16 iGain)
{
	kal_uint8 iI;
    	SENSORDB("[SP5409MIPI]enter SP5409MIPIGain2Reg function\n");
	SENSORDB("[SP5409MIPI]exit SP5409MIPIGain2Reg function\n");
	
       return  ;
}

/*************************************************************************
* FUNCTION
*    SP5409MIPI_SetGain
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

//test gain


void SP5409MIPI_SetGain(UINT16 iGain)
{   
   	 SENSORDB("hcy  SP5409MIPI_SetGain function=%d\n",iGain);

   	 kal_uint8 iReg;
	 


	 if(iGain >= BASEGAIN && iGain <= 15*BASEGAIN)
   	 {   
   	    	 iReg = 0x10 * iGain/BASEGAIN ;        //change mtk gain base to aptina gain base

   	    	 if(iReg<=0x10)
   	    	 {
   	    	    	 SP5409MIPI_write_cmos_sensor(0xfd, 0x01);
   	    	    	 SP5409MIPI_write_cmos_sensor(0x24, 0x10);//0x23
   	    	    	 SP5409MIPI_write_cmos_sensor(0x01, 0x01);
   	    	    	 SENSORDB("SP5409 reg_gain =%d, SP5409MIPI igain = %d", iReg, iGain);
   	    	 }
   	    	 else if(iReg>= 0xa0)
   	    	 {
   	    	    	 SP5409MIPI_write_cmos_sensor(0xfd, 0x01);
   	    	    	 SP5409MIPI_write_cmos_sensor(0x24,0xa0);
   	    	    	 SP5409MIPI_write_cmos_sensor(0x01, 0x01);
   	    	    	 SENSORDB("SP5409MIPI reg_gain =%d, SP5409MIPI igain = %d", iReg, iGain);
	        }        	
   	    	 else
   	    	 {
   	    	    	 SP5409MIPI_write_cmos_sensor(0xfd, 0x01);
   	    	    	 SP5409MIPI_write_cmos_sensor(0x24, (kal_uint8)iReg);
   	    	    	 SP5409MIPI_write_cmos_sensor(0x01, 0x01);
	       }	
   	 }	
   	 else
   	    	 SENSORDB("error gain setting");

}   /*  SP5409MIPI_SetGain_SetGain  */


/*************************************************************************
* FUNCTION
*    read_SP5409MIPI_gain
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
kal_uint16 read_SP5409MIPI_gain(void)
{  
   	 SENSORDB("[SP5409MIPI]enter read_SP5409MIPI_gain function\n");
   	 SP5409MIPI_write_cmos_sensor(0xfd, 0x01);
   	 return (kal_uint16)(SP5409MIPI_read_cmos_sensor(0x24)) ;	//  N*4 //hanlei
   	 
}  /* read_SP5409MIPI_gain */

void write_SP5409MIPI_gain(kal_uint16 gain)
{
   	 SP5409MIPI_SetGain(gain);
}


void SP5409MIPI_camera_para_to_sensor(void)
{
   	 kal_uint32    i;
   	 SENSORDB("[SP5409MIPI]enter SP5409MIPI_camera_para_to_sensor function\n");
   	 for(i=0; 0xFFFFFFFF!=SP5409MIPISensorReg[i].Addr; i++)
   	 {
   	    	 SP5409MIPI_write_cmos_sensor(SP5409MIPISensorReg[i].Addr, SP5409MIPISensorReg[i].Para);
   	 }
   	 for(i=ENGINEER_START_ADDR; 0xFFFFFFFF!=SP5409MIPISensorReg[i].Addr; i++)
   	 {
   	    	 SP5409MIPI_write_cmos_sensor(SP5409MIPISensorReg[i].Addr, SP5409MIPISensorReg[i].Para);
   	 }
   	 for(i=FACTORY_START_ADDR; i<FACTORY_END_ADDR; i++)
   	 {
   	    	 SP5409MIPI_write_cmos_sensor(SP5409MIPISensorCCT[i].Addr, SP5409MIPISensorCCT[i].Para);
   	 }
	 
   	 SENSORDB("[SP5409MIPI]exit SP5409MIPI_camera_para_to_sensor function\n");
}


/*************************************************************************
* FUNCTION
*    SP5409MIPI_sensor_to_camera_para
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
void SP5409MIPI_sensor_to_camera_para(void)
{
   	 SENSORDB("[SP5409MIPI]enter SP5409MIPI_sensor_to_camera_para function\n");
	 
   	 kal_uint32    i,temp_data;
	
   	 for(i=0; 0xFFFFFFFF!=SP5409MIPISensorReg[i].Addr; i++)
   	 {
   	    	 temp_data=SP5409MIPI_read_cmos_sensor(SP5409MIPISensorReg[i].Addr);
   	    	 spin_lock(&sp5409_drv_lock);
   	    	 SP5409MIPISensorReg[i].Para = temp_data;
   	    	 spin_unlock(&sp5409_drv_lock);
   	 }
   	 for(i=ENGINEER_START_ADDR; 0xFFFFFFFF!=SP5409MIPISensorReg[i].Addr; i++)
   	 {
   	    	 temp_data=SP5409MIPI_read_cmos_sensor(SP5409MIPISensorReg[i].Addr);
   	    	 spin_lock(&sp5409_drv_lock);
   	    	 SP5409MIPISensorReg[i].Para = temp_data;
   	    	 spin_unlock(&sp5409_drv_lock);
   	 }
	 
   	 SENSORDB("[SP5409MIPI]exit SP5409MIPI_sensor_to_camera_para function\n");
}

/*************************************************************************
* FUNCTION
*    SP5409MIPI_get_sensor_group_count
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
kal_int32  SP5409MIPI_get_sensor_group_count(void)
{
   	 return GROUP_TOTAL_NUMS;
}

void SP5409MIPI_get_sensor_group_info(kal_uint16 group_idx, kal_int8* group_name_ptr, kal_int32* item_count_ptr)
{
   	 switch (group_idx)
   	 {
   	    	 case PRE_GAIN:
   	    	    	 sprintf((char *)group_name_ptr, "CCT");
   	    	    	 *item_count_ptr = 2;
   	    	    	 break;
   	    	 case CMMCLK_CURRENT:
   	    	    	 sprintf((char *)group_name_ptr, "CMMCLK Current");
   	    	    	 *item_count_ptr = 1;
   	    	    	 break;
   	    	 case FRAME_RATE_LIMITATION:
   	    	    	 sprintf((char *)group_name_ptr, "Frame Rate Limitation");
   	    	    	 *item_count_ptr = 2;
   	    	    	 break;
   	    	 case REGISTER_EDITOR:
   	    	    	 sprintf((char *)group_name_ptr, "Register Editor");
   	    	    	 *item_count_ptr = 2;
   	    	    	 break;
   	    	 default:
   	    	    	 ASSERT(0);
   	 }
}

void SP5409MIPI_get_sensor_item_info(kal_uint16 group_idx,kal_uint16 item_idx, MSDK_SENSOR_ITEM_INFO_STRUCT* info_ptr)
{
   	 kal_int16 temp_reg=0;
   	 kal_uint16 temp_gain=0, temp_addr=0, temp_para=0;
    
   	 switch (group_idx)
   	 {
   	     case PRE_GAIN:
   	    	   switch (item_idx)
   	    	   {
                 	  case 0:
                   	      sprintf((char *)info_ptr->ItemNamePtr,"Pregain-R");
                           temp_addr = PRE_GAIN_R_INDEX;
                 	    	break;
                 	  case 1:
                   	      sprintf((char *)info_ptr->ItemNamePtr,"Pregain-Gr");
                   	      temp_addr = PRE_GAIN_Gr_INDEX;
                   	      break;
                 	  case 2:
                   	      sprintf((char *)info_ptr->ItemNamePtr,"Pregain-Gb");
                   	      temp_addr = PRE_GAIN_Gb_INDEX;
                   	      break;
                 	  case 3:
                   	      sprintf((char *)info_ptr->ItemNamePtr,"Pregain-B");
                   	      temp_addr = PRE_GAIN_B_INDEX;
                   	      break;
                 	  case 4:
                   	      sprintf((char *)info_ptr->ItemNamePtr,"SENSOR_BASEGAIN");
                   	      temp_addr = SENSOR_BASEGAIN;
                   	      break;
                 	  default:
                   	      SENSORDB("[IMX105MIPI][Error]get_sensor_item_info error!!!\n");
   	    	   }
			   
   	    	   spin_lock(&sp5409_drv_lock);    
   	    	   temp_para=SP5409MIPISensorCCT[temp_addr].Para;	
   	    	   spin_unlock(&sp5409_drv_lock);
   	    	   temp_gain = SP5409MIPIReg2Gain(temp_para);
   	    	   temp_gain=(temp_gain*1000)/BASEGAIN; //hanlei?
   	    	   info_ptr->ItemValue=temp_gain;
   	    	   info_ptr->IsTrueFalse=KAL_FALSE;
   	    	   info_ptr->IsReadOnly=KAL_FALSE;
   	    	   info_ptr->IsNeedRestart=KAL_FALSE;
   	    	   info_ptr->Min=1000;//hanlei ?
   	    	   info_ptr->Max=15875;
   	    	   break;
   	     case CMMCLK_CURRENT:
   	    	   switch (item_idx)
   	    	   {
                 	  case 0:
                   	      sprintf((char *)info_ptr->ItemNamePtr,"Drv Cur[2,4,6,8]mA");
                
                   	      //temp_reg=SP5409MIPISensorReg[CMMCLK_CURRENT_INDEX].Para;
                   	      temp_reg = ISP_DRIVING_2MA;
                   	      if(temp_reg==ISP_DRIVING_2MA)
                   	      {
                   	      		info_ptr->ItemValue=2;
                   	      }
                   	      else if(temp_reg==ISP_DRIVING_4MA)
                   	      {
                   	      		info_ptr->ItemValue=4;
                   	      }
                   	      else if(temp_reg==ISP_DRIVING_6MA)
                   	      {
                   	      		info_ptr->ItemValue=6;
                   	      }
                   	      else if(temp_reg==ISP_DRIVING_8MA)
                   	      {
                   	      		info_ptr->ItemValue=8;
                   	      }
                
                   	      info_ptr->IsTrueFalse=KAL_FALSE;
                   	      info_ptr->IsReadOnly=KAL_FALSE;
                   	      info_ptr->IsNeedRestart=KAL_TRUE;
                   	      info_ptr->Min=2;
                   	      info_ptr->Max=8;
                   	      break;
                 	  default:
                   	      ASSERT(0);
   	    	   }
   	    	   break;
        case FRAME_RATE_LIMITATION:
            switch (item_idx)
            {
                case 0:
                    sprintf((char *)info_ptr->ItemNamePtr,"Max Exposure Lines");
                    info_ptr->ItemValue=SP5409MIPI_MAX_EXPOSURE_LINES;
                    info_ptr->IsTrueFalse=KAL_FALSE;
                    info_ptr->IsReadOnly=KAL_TRUE;
                    info_ptr->IsNeedRestart=KAL_FALSE;
                    info_ptr->Min=0;
                    info_ptr->Max=0;
                    break;
                case 1:
                    sprintf((char *)info_ptr->ItemNamePtr,"Min Frame Rate");
                    info_ptr->ItemValue=12;
                    info_ptr->IsTrueFalse=KAL_FALSE;
                    info_ptr->IsReadOnly=KAL_TRUE;
                    info_ptr->IsNeedRestart=KAL_FALSE;
                    info_ptr->Min=0;
                    info_ptr->Max=0;
                    break;
                default:
                    ASSERT(0);
            }
            break;
        case REGISTER_EDITOR:
            switch (item_idx)
            {
                case 0:
                    sprintf((char *)info_ptr->ItemNamePtr,"REG Addr.");
                    info_ptr->ItemValue=0;
                    info_ptr->IsTrueFalse=KAL_FALSE;
                    info_ptr->IsReadOnly=KAL_FALSE;
                    info_ptr->IsNeedRestart=KAL_FALSE;
                    info_ptr->Min=0;
                    info_ptr->Max=0xFFFF;
                    break;
                case 1:
                    sprintf((char *)info_ptr->ItemNamePtr,"REG Value");
                    info_ptr->ItemValue=0;
                    info_ptr->IsTrueFalse=KAL_FALSE;
                    info_ptr->IsReadOnly=KAL_FALSE;
                    info_ptr->IsNeedRestart=KAL_FALSE;
                    info_ptr->Min=0;
                    info_ptr->Max=0xFFFF;
                    break;
                default:
                ASSERT(0);
            }
            break;
        default:
            ASSERT(0);
    }
}
kal_bool SP5409MIPI_set_sensor_item_info(kal_uint16 group_idx, kal_uint16 item_idx, kal_int32 ItemValue)
{
   kal_uint16 temp_addr=0, temp_para=0;

   switch (group_idx)
    {
        case PRE_GAIN:
            switch (item_idx)
            {
              case 0:
                temp_addr = PRE_GAIN_R_INDEX;
              break;
              case 1:
                temp_addr = PRE_GAIN_Gr_INDEX;
              break;
              case 2:
                temp_addr = PRE_GAIN_Gb_INDEX;
              break;
              case 3:
                temp_addr = PRE_GAIN_B_INDEX;
              break;
              case 4:
                temp_addr = SENSOR_BASEGAIN;
              break;
              default:
                 SENSORDB("[IMX105MIPI][Error]set_sensor_item_info error!!!\n");
          }
            temp_para = SP5409MIPIGain2Reg(ItemValue);
            spin_lock(&sp5409_drv_lock);    
            SP5409MIPISensorCCT[temp_addr].Para = temp_para;
			spin_unlock(&sp5409_drv_lock);
		

		SP5409MIPI_write_cmos_sensor(0xfd, 0x01);

		SP5409MIPI_write_cmos_sensor(SP5409MIPISensorCCT[temp_addr].Addr,temp_para);//hanlei?
		
			temp_para=read_SP5409MIPI_gain();	
            spin_lock(&sp5409_drv_lock);    
            SP5409MIPI_sensor_gain_base=temp_para;
			spin_unlock(&sp5409_drv_lock);

            break;
        case CMMCLK_CURRENT:
            switch (item_idx)
            {
                case 0:
                    if(ItemValue==2)
                    {			
                        spin_lock(&sp5409_drv_lock);    
                        SP5409MIPISensorReg[CMMCLK_CURRENT_INDEX].Para = ISP_DRIVING_2MA;
                        spin_unlock(&sp5409_drv_lock);
                        //SP5409MIPI_set_isp_driving_current(ISP_DRIVING_2MA);
                    }
                    else if(ItemValue==3 || ItemValue==4)
                    {
                        spin_lock(&sp5409_drv_lock);    
                        SP5409MIPISensorReg[CMMCLK_CURRENT_INDEX].Para = ISP_DRIVING_4MA;
                        spin_unlock(&sp5409_drv_lock);
                        //SP5409MIPI_set_isp_driving_current(ISP_DRIVING_4MA);
                    }
                    else if(ItemValue==5 || ItemValue==6)
                    {
                        spin_lock(&sp5409_drv_lock);    
                        SP5409MIPISensorReg[CMMCLK_CURRENT_INDEX].Para = ISP_DRIVING_6MA;
                        spin_unlock(&sp5409_drv_lock);
                        //SP5409MIPI_set_isp_driving_current(ISP_DRIVING_6MA);
                    }
                    else
                    {
                    	    spin_lock(&sp5409_drv_lock);    
                        SP5409MIPISensorReg[CMMCLK_CURRENT_INDEX].Para = ISP_DRIVING_8MA;
                    	    spin_unlock(&sp5409_drv_lock);
                        //SP5409MIPI_set_isp_driving_current(ISP_DRIVING_8MA);
                    }
                    break;
                default:
                    ASSERT(0);
            }
            break;
        case FRAME_RATE_LIMITATION:
            ASSERT(0);
            break;
        case REGISTER_EDITOR:
            switch (item_idx)
            {
                case 0:
					spin_lock(&sp5409_drv_lock);    
                    SP5409MIPI_FAC_SENSOR_REG=ItemValue;
					spin_unlock(&sp5409_drv_lock);
                    break;
                case 1:
                    SP5409MIPI_write_cmos_sensor(SP5409MIPI_FAC_SENSOR_REG,ItemValue);
                    break;
                default:
                    ASSERT(0);
            }
            break;
        default:
            ASSERT(0);
    }
    return KAL_TRUE;
}

static void SP5409MIPI_SetDummy(const kal_uint16 iPixels, const kal_uint16 iLines)
{
    SENSORDB("[SP5409MIPI]enter SP5409MIPI_SetDummy function\n");
    
  

}   /*  SP5409MIPI_SetDummy */

static void SP5409MIPI_Sensor_Init(void)
{
   #ifdef DEBUG_SENSOR
	if(fromsd == 1)//是否从SD读取//gepeiwei   120903
		SP5409MIPI_Initialize_from_T_Flash();//从SD卡读取的主要函数
       else
   #endif

   //init setting
   #if 1
   	{
   		#if 0
	SP5409MIPI_write_cmos_sensor(0xfd,0x00);
	SP5409MIPI_write_cmos_sensor(0x1b,0x00);
	SP5409MIPI_write_cmos_sensor(0x1c,0x00);
	SP5409MIPI_write_cmos_sensor(0xfd,0x01);
	SP5409MIPI_write_cmos_sensor(0x0d,0x01);  //test_en
	SP5409MIPI_write_cmos_sensor(0xfb,0x21);  //abl
	SP5409MIPI_write_cmos_sensor(0x01,0x01);
     #endif 


	 SP5409MIPI_write_cmos_sensor(0xfd,0x00);
    
SP5409MIPI_write_cmos_sensor(0xfd,0x00);
SP5409MIPI_write_cmos_sensor(0x1b,0x00);
SP5409MIPI_write_cmos_sensor(0x2f,0x14);
SP5409MIPI_write_cmos_sensor(0x34,0x00);
SP5409MIPI_write_cmos_sensor(0xfd,0x01);
SP5409MIPI_write_cmos_sensor(0x06,0x10);
SP5409MIPI_write_cmos_sensor(0x03,0x01);
SP5409MIPI_write_cmos_sensor(0x04,0x0f);
SP5409MIPI_write_cmos_sensor(0x0a,0x80);
SP5409MIPI_write_cmos_sensor(0x24,0x50);
SP5409MIPI_write_cmos_sensor(0x01,0x01);
SP5409MIPI_write_cmos_sensor(0x21,0x37);
SP5409MIPI_write_cmos_sensor(0x25,0xf4);
SP5409MIPI_write_cmos_sensor(0x26,0x0c);
SP5409MIPI_write_cmos_sensor(0x2c,0xb0);
SP5409MIPI_write_cmos_sensor(0x8a,0x55);
SP5409MIPI_write_cmos_sensor(0x8b,0x55);
SP5409MIPI_write_cmos_sensor(0x58,0x3a);
SP5409MIPI_write_cmos_sensor(0x75,0x4a);
SP5409MIPI_write_cmos_sensor(0x77,0x6f);
SP5409MIPI_write_cmos_sensor(0x78,0xef);
SP5409MIPI_write_cmos_sensor(0x84,0x0f);
SP5409MIPI_write_cmos_sensor(0x85,0x02);
SP5409MIPI_write_cmos_sensor(0x11,0x30);
SP5409MIPI_write_cmos_sensor(0x51,0x4a);
SP5409MIPI_write_cmos_sensor(0x52,0x4a);
SP5409MIPI_write_cmos_sensor(0x5d,0x00);
SP5409MIPI_write_cmos_sensor(0x5e,0x00);
SP5409MIPI_write_cmos_sensor(0x68,0x52);
SP5409MIPI_write_cmos_sensor(0x72,0x54);
SP5409MIPI_write_cmos_sensor(0xfb,0x25);
SP5409MIPI_write_cmos_sensor(0xf0,0x04);
SP5409MIPI_write_cmos_sensor(0xf1,0x04);
SP5409MIPI_write_cmos_sensor(0xf2,0x04);
SP5409MIPI_write_cmos_sensor(0xf3,0x04);
SP5409MIPI_write_cmos_sensor(0xfd,0x01);
//SP5409MIPI_write_cmos_sensor(0x3f,0x01);//mirror
//SP5409MIPI_write_cmos_sensor(0x01,0x01);//mirror
SP5409MIPI_write_cmos_sensor(0xb3,0x00);
SP5409MIPI_write_cmos_sensor(0xb1,0x01);
SP5409MIPI_write_cmos_sensor(0xa4,0x6d);
SP5409MIPI_write_cmos_sensor(0xb6,0xc0);
SP5409MIPI_write_cmos_sensor(0x9d,0x25);
SP5409MIPI_write_cmos_sensor(0x97,0x08);
SP5409MIPI_write_cmos_sensor(0xc4,0x28);
SP5409MIPI_write_cmos_sensor(0xc1,0x09);
SP5409MIPI_write_cmos_sensor(0xfd,0x02);
SP5409MIPI_write_cmos_sensor(0x5e,0x07);



#if 0	
SP5409MIPI_write_cmos_sensor(0xfd,0x00);
SP5409MIPI_write_cmos_sensor(0x1b,0x00);
SP5409MIPI_write_cmos_sensor(0x2f,0x14);//192M
SP5409MIPI_write_cmos_sensor(0x34,0x00);  
SP5409MIPI_write_cmos_sensor(0xfd,0x01);
SP5409MIPI_write_cmos_sensor(0x06,0x10); //vblank
SP5409MIPI_write_cmos_sensor(0x03,0x03); //exp time 3base
SP5409MIPI_write_cmos_sensor(0x04,0x81);
SP5409MIPI_write_cmos_sensor(0x24,0xf0); //pga gain
SP5409MIPI_write_cmos_sensor(0x01,0x01);
SP5409MIPI_write_cmos_sensor(0x25,0xf4); //reg dac, bl_en
SP5409MIPI_write_cmos_sensor(0x26,0x0c); //vref2 1.3v, disable rd
SP5409MIPI_write_cmos_sensor(0x2c,0xb0);
SP5409MIPI_write_cmos_sensor(0x8a,0x55);
SP5409MIPI_write_cmos_sensor(0x8b,0x55);
SP5409MIPI_write_cmos_sensor(0x77,0x6f);
SP5409MIPI_write_cmos_sensor(0x78,0xef);
SP5409MIPI_write_cmos_sensor(0x84,0x0f);
SP5409MIPI_write_cmos_sensor(0x85,0x02);
SP5409MIPI_write_cmos_sensor(0x11,0x30); //rst_num
SP5409MIPI_write_cmos_sensor(0x51,0x20); //p1
SP5409MIPI_write_cmos_sensor(0x52,0x20); //p2
SP5409MIPI_write_cmos_sensor(0x5d,0x00);
SP5409MIPI_write_cmos_sensor(0x5e,0x00);
SP5409MIPI_write_cmos_sensor(0x68,0x52); //p22
SP5409MIPI_write_cmos_sensor(0x72,0x54); //p22
SP5409MIPI_write_cmos_sensor(0xf0,0x20); //offset
SP5409MIPI_write_cmos_sensor(0xf1,0x20);
SP5409MIPI_write_cmos_sensor(0xf2,0x20);
SP5409MIPI_write_cmos_sensor(0xf3,0x20);
SP5409MIPI_write_cmos_sensor(0xfd,0x01); //mipi
SP5409MIPI_write_cmos_sensor(0xb3,0x00);
SP5409MIPI_write_cmos_sensor(0xb1,0x01);
SP5409MIPI_write_cmos_sensor(0xa4,0x6d);
SP5409MIPI_write_cmos_sensor(0xb6,0xc0);  //80:1lane,c0:2lane
SP5409MIPI_write_cmos_sensor(0x9d,0x25);
SP5409MIPI_write_cmos_sensor(0x97,0x04);
SP5409MIPI_write_cmos_sensor(0xc4,0x48);
SP5409MIPI_write_cmos_sensor(0xc1,0x04);
#endif


   }
   #endif
   #if 0
   {
   //from spreadtrum
	SP5409MIPI_write_cmos_sensor(0xfd,0x00);
	SP5409MIPI_write_cmos_sensor(0x10,0x04);//0x04  2倍频
	SP5409MIPI_write_cmos_sensor(0x11,0x10);
	SP5409MIPI_write_cmos_sensor(0x12,0x04);
	SP5409MIPI_write_cmos_sensor(0x13,0x01);
	SP5409MIPI_write_cmos_sensor(0x15,0x00);
	SP5409MIPI_write_cmos_sensor(0x21,0x03);
	SP5409MIPI_write_cmos_sensor(0x20,0x3f);
	SP5409MIPI_write_cmos_sensor(0x0e,0x10);//20140613 dis pixel ldo 0x10使用外部电压，0x00使用平台电压
	SP5409MIPI_write_cmos_sensor(0x26,0x00);

	SP5409MIPI_write_cmos_sensor(0xfd,0x05);	
	SP5409MIPI_write_cmos_sensor(0x00,0xc1);
	SP5409MIPI_write_cmos_sensor(0x02,0x10);
	SP5409MIPI_write_cmos_sensor(0x03,0x05);
	SP5409MIPI_write_cmos_sensor(0x04,0xcc);
	SP5409MIPI_write_cmos_sensor(0x05,0x03);
	
	SP5409MIPI_write_cmos_sensor(0xfd,0x06);
	SP5409MIPI_write_cmos_sensor(0xfb,0x00);//95/*2014/5/26*/
	SP5409MIPI_write_cmos_sensor(0xc6,0x80);
	SP5409MIPI_write_cmos_sensor(0xc7,0x80);
	SP5409MIPI_write_cmos_sensor(0xc8,0x80);
	SP5409MIPI_write_cmos_sensor(0xc9,0x80);
	
	SP5409MIPI_write_cmos_sensor(0xfd,0x01);
	SP5409MIPI_write_cmos_sensor(0x09,0x00);//14/04/10
	SP5409MIPI_write_cmos_sensor(0x0a,0x83);//14/04/10
	SP5409MIPI_write_cmos_sensor(0x0b,0x02);//14/04/10
	SP5409MIPI_write_cmos_sensor(0x0c,0x55);//33 /*2014/5/26*/
	SP5409MIPI_write_cmos_sensor(0x0d,0x55);//33/*2014/5/26*/
	SP5409MIPI_write_cmos_sensor(0x21,0x1f);
	SP5409MIPI_write_cmos_sensor(0x29,0x03);
	SP5409MIPI_write_cmos_sensor(0x2a,0xff);
	
	SP5409MIPI_write_cmos_sensor(0xfd,0x01);
	SP5409MIPI_write_cmos_sensor(0x0f,0x3f);//7f  lujunge
	SP5409MIPI_write_cmos_sensor(0x10,0x3e);//7e lujunge
	SP5409MIPI_write_cmos_sensor(0x11,0x00);
	SP5409MIPI_write_cmos_sensor(0x12,0x00);
	SP5409MIPI_write_cmos_sensor(0x13,0x2f);
	SP5409MIPI_write_cmos_sensor(0x16,0x00);
	SP5409MIPI_write_cmos_sensor(0x17,0x2f);
	SP5409MIPI_write_cmos_sensor(0x18,0x01);//20140613 dele purple照太阳去紫
	SP5409MIPI_write_cmos_sensor(0x14,0x00);
	SP5409MIPI_write_cmos_sensor(0x15,0x38);
	SP5409MIPI_write_cmos_sensor(0x6b,0x32);
	SP5409MIPI_write_cmos_sensor(0x6c,0x32);
	SP5409MIPI_write_cmos_sensor(0x6f,0x32);
	SP5409MIPI_write_cmos_sensor(0x70,0x40);
	SP5409MIPI_write_cmos_sensor(0x71,0x40);
	SP5409MIPI_write_cmos_sensor(0x72,0x40);
	SP5409MIPI_write_cmos_sensor(0x73,0x33);
	SP5409MIPI_write_cmos_sensor(0x74,0x38);
	SP5409MIPI_write_cmos_sensor(0x75,0x38);
	SP5409MIPI_write_cmos_sensor(0x76,0x44);
	SP5409MIPI_write_cmos_sensor(0x77,0x38);
	SP5409MIPI_write_cmos_sensor(0x7a,0x40);
	SP5409MIPI_write_cmos_sensor(0x7e,0x40);
	SP5409MIPI_write_cmos_sensor(0x84,0x40);
	SP5409MIPI_write_cmos_sensor(0x7d,0x0f);

	SP5409MIPI_write_cmos_sensor(0xfd,0x01);
	SP5409MIPI_write_cmos_sensor(0x19,0x03);//0x13
	SP5409MIPI_write_cmos_sensor(0x31,0x04);
	SP5409MIPI_write_cmos_sensor(0x30,0x1e);//20140613 enable sunspot 使能太阳黑子
	SP5409MIPI_write_cmos_sensor(0x24,0x80);
	SP5409MIPI_write_cmos_sensor(0x03,0x0e);
	SP5409MIPI_write_cmos_sensor(0x04,0xec);
	SP5409MIPI_write_cmos_sensor(0x22,0x18);/*0x1a*/
	SP5409MIPI_write_cmos_sensor(0x2e,0x08);
	SP5409MIPI_write_cmos_sensor(0x1e,0x01);
	SP5409MIPI_write_cmos_sensor(0x1d,0x04);
	SP5409MIPI_write_cmos_sensor(0x20,0xdf);
	SP5409MIPI_write_cmos_sensor(0x25,0xad);//0x8d  20140613 pixel vcm
	SP5409MIPI_write_cmos_sensor(0x27,0xb5);//0xb9   /*0xa1//14/5/19*/a5
	SP5409MIPI_write_cmos_sensor(0x1a,0x43);
	SP5409MIPI_write_cmos_sensor(0x28,0x00);
	SP5409MIPI_write_cmos_sensor(0x40,0x05);
	SP5409MIPI_write_cmos_sensor(0x3f,0x00);
	SP5409MIPI_write_cmos_sensor(0x01,0x01);
	
	SP5409MIPI_write_cmos_sensor(0xfd,0x02);
	SP5409MIPI_write_cmos_sensor(0x37,0x00);
	SP5409MIPI_write_cmos_sensor(0x38,0x04);
	SP5409MIPI_write_cmos_sensor(0x3b,0x00);
	SP5409MIPI_write_cmos_sensor(0x3c,0x02);
	SP5409MIPI_write_cmos_sensor(0x39,0x03);
	SP5409MIPI_write_cmos_sensor(0x3a,0xcc);
	SP5409MIPI_write_cmos_sensor(0x3d,0x02);
	SP5409MIPI_write_cmos_sensor(0x3e,0x88);
	SP5409MIPI_write_cmos_sensor(0x33,0x0f);
	SP5409MIPI_write_cmos_sensor(0x3f,0x01);
	
	SP5409MIPI_write_cmos_sensor(0xfd,0x03);
	SP5409MIPI_write_cmos_sensor(0xaa,0x46);
	SP5409MIPI_write_cmos_sensor(0xab,0x01);
	SP5409MIPI_write_cmos_sensor(0xac,0xf5);
	SP5409MIPI_write_cmos_sensor(0xad,0x00);
	//SP5409MIPI_write_cmos_sensor(0xff,0xff);

   }
	{	//77 capture setting
	SENSORDB("[SP5409MIPI]exit SP5409MIPI_set_8M function\n"); 
	//capture setting
 
	//2592x1944
	SP5409MIPI_write_cmos_sensor(0xfd,0x00);
	SP5409MIPI_write_cmos_sensor(0x10,0x04);//hanlei
	SP5409MIPI_write_cmos_sensor(0x11,0x00);

	//SP5409MIPI_write_cmos_sensor(0xfd,0x01);
	//SP5409MIPI_write_cmos_sensor(0x09,0x01);
	//SP5409MIPI_write_cmos_sensor(0x0a,0x43);//50us max9.99fps

	SP5409MIPI_write_cmos_sensor(0xfd,0x01);
	SP5409MIPI_write_cmos_sensor(0x19,0x00);
	SP5409MIPI_write_cmos_sensor(0x31,0x00);
	SP5409MIPI_write_cmos_sensor(0x01,0x01);

	SP5409MIPI_write_cmos_sensor(0xfd,0x02);
	SP5409MIPI_write_cmos_sensor(0x37,0x00);
	SP5409MIPI_write_cmos_sensor(0x38,0x08);
	SP5409MIPI_write_cmos_sensor(0x3b,0x00);
	SP5409MIPI_write_cmos_sensor(0x3c,0x04);
	SP5409MIPI_write_cmos_sensor(0x39,0x07);
	SP5409MIPI_write_cmos_sensor(0x3a,0x98);
	SP5409MIPI_write_cmos_sensor(0x3d,0x05);
	SP5409MIPI_write_cmos_sensor(0x3e,0x10);

	SP5409MIPI_write_cmos_sensor(0xfd,0x05);
	SP5409MIPI_write_cmos_sensor(0x02,0x20);
	SP5409MIPI_write_cmos_sensor(0x03,0x0a);
	SP5409MIPI_write_cmos_sensor(0x04,0x98);
	SP5409MIPI_write_cmos_sensor(0x05,0x07);

}
	#endif
   // The register only need to enable 1 time.    
   spin_lock(&sp5409_drv_lock);  
   SP5409MIPI_Auto_Flicker_mode = KAL_FALSE;	  // reset the flicker status	 
   spin_unlock(&sp5409_drv_lock);
   SENSORDB("[SP5409MIPI]exit SP5409MIPI_Sensor_Init function\n");

}   /*  SP5409MIPI_Sensor_Init  */

static VideoFullSizeSetting(void)//16:9   6M
{
	SENSORDB("[SP5409MIPI]enter VideoFullSizeSetting function\n");

       #ifdef SP5409MIPI_USE_OTP
       if(ret == 0)
       {
	    SENSORDB("[SP5409MIPI_USE_OTP]VideoFullSizeSetting function,sp5409_update_awb_gain\n");
	    sp5409_update_awb_gain();
       }
       #endif
       SENSORDB("[SP5409MIPI]exit VideoFullSizeSetting function\n");
}
static PreviewSetting(void)
{
	//preview setting 

	//1296x972

	/*SP5409MIPI_write_cmos_sensor(0xfd,0x06);
	SP5409MIPI_write_cmos_sensor(0x40,0x06);
	SP5409MIPI_write_cmos_sensor(0x42,0x06);//b
	SP5409MIPI_write_cmos_sensor(0x46,0x00);
	SP5409MIPI_write_cmos_sensor(0x48,0x00);//r
	SP5409MIPI_write_cmos_sensor(0x4c,0x06);
	SP5409MIPI_write_cmos_sensor(0x4e,0x06);//gb
	SP5409MIPI_write_cmos_sensor(0x52,0x04);
	SP5409MIPI_write_cmos_sensor(0x54,0x04);//gr*/
	
	
}

void SP5409MIPI_set_5M(void)
{	//77 capture setting
	SENSORDB("[SP5409MIPI]exit SP5409MIPI_set_8M function\n"); 
	//capture setting
 
	//2592x1944


}
/*****************************************************************************/
/* Windows Mobile Sensor Interface */
/*****************************************************************************/
/*************************************************************************
* FUNCTION
*   SP5409MIPIOpen
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

UINT32 SP5409MIPIOpen(void)
{
    SENSORDB("[hcy SP5409MIPI]enter SP5409MIPIOpen function\n");
	
    kal_uint32 rc = 0;
	
    #ifdef SP5409MIPI_USE_OTP
    if(0 == used_otp){
	printk("[SP5409MIPI_USE_OTP] before update otp wb...........................................\n");
	printk("[SP5409MIPI_USE_OTP] before update otp wb...........................................\n");
	printk("[SP5409MIPI_USE_OTP] before update otp wb...........................................\n");
	ret = sp5409_update_otp_wb();

	used_otp =1;
	printk("[SP5409MIPI_USE_OTP] after update otp wb............................................\n");
	printk("[SP5409MIPI_USE_OTP] after update otp wb............................................\n");
	printk("[SP5409MIPI_USE_OTP] after update otp wb............................................\n");
    }
    #endif
	
    int  retry = 0; 
    kal_uint16 sensorid;
    kal_uint16 sensorIDH;
    kal_uint16 sensorIDL;
    kal_uint16 sensorID;
    // check if sensor ID correct
    retry = 3; 

    do {

	  SP5409MIPI_write_cmos_sensor(0xfd, 0x00);
	   sensorIDH = (SP5409MIPI_read_cmos_sensor(0x02)<<8)&0xFF00;
	  
	   sensorIDL = SP5409MIPI_read_cmos_sensor(0x03) ;
	   sensorid = sensorIDH | sensorIDL;

	   SENSORDB("hcy sensorIDL =  0x%04x\n", sensorid);

	   spin_lock(&sp5409_drv_lock);    
	   SP5409MIPI_sensor_id =sensorid;
	   spin_unlock(&sp5409_drv_lock);
		if (SP5409MIPI_sensor_id == SP5409MIPI_SENSOR_ID)
			break; 
		retry--; 

		
	    }
	while (retry > 0);
    SENSORDB("Read Sensor ID = 0x%04x\n", SP5409MIPI_sensor_id);
    if (SP5409MIPI_sensor_id != SP5409MIPI_SENSOR_ID)
        return ERROR_SENSOR_CONNECT_FAIL;

#ifdef DEBUG_SENSOR  //gepeiwei   120903
					//判断手机对应目录下是否有名为sp5409_sd 的文件,没有默认参数

					//介于各种原因，本版本初始化参数在_s_fmt中。
 struct file *fp; 
    mm_segment_t fs; 
    loff_t pos = 0; 
	static char buf[10*1024] ;
 
    fp = filp_open("/mnt/sdcard/sp5409_sd", O_RDONLY , 0); 
    if (IS_ERR(fp)) { 
		fromsd = 0;   
		printk("gpw open file error\n");
		//return 0;
    } 
	
	else 
		{
		fromsd = 1;
	//SP5409MIPI_Initialize_from_T_Flash();
	printk("gpw read ok!\n");

	filp_close(fp, NULL); 
    set_fs(fs);
		}
#endif




	
    SP5409MIPI_Sensor_Init();
	sensorid=read_SP5409MIPI_gain();
	spin_lock(&sp5409_drv_lock);	
    SP5409MIPI_sensor_gain_base = sensorid;
	spin_unlock(&sp5409_drv_lock);
	SENSORDB("[SP5409MIPI]exit SP5409MIPIOpen function\n");
    return ERROR_NONE;
}

/*************************************************************************
* FUNCTION
*   SP5409MIPIGetSensorID
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
UINT32 SP5409MIPIGetSensorID(UINT32 *sensorID) 
{
    int  retry = 3; 
	SENSORDB("[SP5409MIPI]enter SP5409MIPIGetSensorID function\n");

	mt_set_gpio_mode(GPIO_CAMERA_CMPDN_PIN,GPIO_CAMERA_CMPDN_PIN_M_GPIO);
	mt_set_gpio_dir(GPIO_CAMERA_CMPDN_PIN,GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_CAMERA_CMPDN_PIN,GPIO_OUT_ZERO);
	msleep(10);

//test 5409 sensor id for iic
    do {
	//int  retry = 3; 
	kal_uint16 sensorIDH = 0;
	kal_uint16 sensorIDL = 0;
	    // check if sensor ID correct
	    SENSORDB("hcy 5409MIPIGetSensorID\n"); 
	  SP5409MIPI_write_cmos_sensor(0xfd, 0x00);
        SENSORDB("hcy Read Sensor ID tets = 0x%04x  reg0x04=%d\n", *sensorID,SP5409MIPI_read_cmos_sensor(0x04)); 

	  // *sensorID =(kal_uint16)((IMX135MIPI_read_cmos_sensor_truly(0x0000)<<8) | IMX135MIPI_read_cmos_sensor_truly(0x0001)); 
	   sensorIDH = (SP5409MIPI_read_cmos_sensor(0x02)<<8)&0xFF00;
	  SENSORDB("hcy 5409 sensorIDH =  0x%04x\n", sensorIDH);
	  
	   sensorIDL = SP5409MIPI_read_cmos_sensor(0x03) ;
	   SENSORDB("hcy sensorIDL =  0x%04x\n", sensorIDL);
	   *sensorID = sensorIDH | sensorIDL;
        SENSORDB("hcy Read Sensor ID Fail = 0x%04x  reg0x04=%d\n", *sensorID,SP5409MIPI_read_cmos_sensor(0x03)); 
	   
        if (*sensorID == SP5409MIPI_SENSOR_ID)
        {
            camera_pdn_reverse = 1;
            break;
        }
        SENSORDB("hcy Read Sensor ID Fail = 0x%04x  reg0x04=%d\n", *sensorID,SP5409MIPI_read_cmos_sensor(0x03)); 
        retry--; 
		
    } while (retry > 0);

//end test
#ifdef ONLINE_DEBUG
debug_id = *sensorID;
#endif
    if (*sensorID != SP5409MIPI_SENSOR_ID) {
        *sensorID = 0xFFFFFFFF; 
        return ERROR_SENSOR_CONNECT_FAIL;
    }
        SENSORDB("hcy Read Sensor ID   = 0x%04x\n", *sensorID); 
    return ERROR_NONE;
}


/*************************************************************************
* FUNCTION
*   SP5409MIPI_SetShutter
*
* DESCRIPTION
*   This function set e-shutter of SP5409MIPI to change exposure time.
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
void SP5409MIPI_SetShutter(kal_uint16 iShutter)
{
	SENSORDB("GuoJinHui[SP5409MIPI_SetShutter]%s():shutter=%d\n",__FUNCTION__,iShutter);
       if (iShutter < 1)
          iShutter = 1; 
	else if(iShutter > 0xffff)
	   iShutter = 0xffff;
	unsigned long flags;
	spin_lock_irqsave(&sp5409_drv_lock,flags);
       SP5409MIPI_sensor.pv_shutter = iShutter;	
	spin_unlock_irqrestore(&sp5409_drv_lock,flags);
       SP5409MIPI_write_shutter(iShutter);
	SENSORDB("[SP5409MIPI]exit SP5409MIPIGetSensorID function\n");
}   /*  SP5409MIPI_SetShutter   */



/*************************************************************************
* FUNCTION
*   SP5409MIPI_read_shutter
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
UINT16 SP5409MIPI_read_shutter(void)
{

    SP5409MIPI_write_cmos_sensor(0xfd, 0x01);

    return (UINT16)( (SP5409MIPI_read_cmos_sensor(0x03)<<8) | SP5409MIPI_read_cmos_sensor(0x04) );
}

/*************************************************************************
* FUNCTION
*   SP5409MIPI_night_mode
*
* DESCRIPTION
*   This function night mode of SP5409MIPI.
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
void SP5409MIPI_NightMode(kal_bool bEnable)
{
	SENSORDB("[SP5409MIPI]enter SP5409MIPI_NightMode function\n");
#if 0
    /************************************************************************/
    /*                      Auto Mode: 30fps                                                                                          */
    /*                      Night Mode:15fps                                                                                          */
    /************************************************************************/
    if(bEnable)
    {
        if(OV5642_MPEG4_encode_mode==KAL_TRUE)
        {
            OV5642_MAX_EXPOSURE_LINES = (kal_uint16)((OV5642_sensor_pclk/15)/(OV5642_PV_PERIOD_PIXEL_NUMS+OV5642_PV_dummy_pixels));
            OV5642_write_cmos_sensor(0x350C, (OV5642_MAX_EXPOSURE_LINES >> 8) & 0xFF);
            OV5642_write_cmos_sensor(0x350D, OV5642_MAX_EXPOSURE_LINES & 0xFF);
            OV5642_CURRENT_FRAME_LINES = OV5642_MAX_EXPOSURE_LINES;
            OV5642_MAX_EXPOSURE_LINES = OV5642_CURRENT_FRAME_LINES - OV5642_SHUTTER_LINES_GAP;
        }
    }
    else// Fix video framerate 30 fps
    {
        if(OV5642_MPEG4_encode_mode==KAL_TRUE)
        {
            OV5642_MAX_EXPOSURE_LINES = (kal_uint16)((OV5642_sensor_pclk/30)/(OV5642_PV_PERIOD_PIXEL_NUMS+OV5642_PV_dummy_pixels));
            if(OV5642_pv_exposure_lines < (OV5642_MAX_EXPOSURE_LINES - OV5642_SHUTTER_LINES_GAP)) // for avoid the shutter > frame_lines,move the frame lines setting to shutter function
            {
                OV5642_write_cmos_sensor(0x350C, (OV5642_MAX_EXPOSURE_LINES >> 8) & 0xFF);
                OV5642_write_cmos_sensor(0x350D, OV5642_MAX_EXPOSURE_LINES & 0xFF);
                OV5642_CURRENT_FRAME_LINES = OV5642_MAX_EXPOSURE_LINES;
            }
            OV5642_MAX_EXPOSURE_LINES = OV5642_MAX_EXPOSURE_LINES - OV5642_SHUTTER_LINES_GAP;
        }
    }
	
#endif	
	SENSORDB("[SP5409MIPI]exit SP5409MIPI_NightMode function\n");
}/*	SP5409MIPI_NightMode */



/*************************************************************************
* FUNCTION
*   SP5409MIPIClose
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
UINT32 SP5409MIPIClose(void)
{
    //SP5409MIPI_write_cmos_sensor(0x0100,0x00);
    return ERROR_NONE;
}	/* SP5409MIPIClose() */

void SP5409MIPISetFlipMirror(kal_int32 imgMirror)
{
    kal_uint8  iTemp; 
	
	SENSORDB("[SP5409MIPI]enter SP5409MIPISetFlipMirror function\n");

	    SP5409MIPI_write_cmos_sensor(0xfd, 0x01);


    iTemp = SP5409MIPI_read_cmos_sensor(0x3f) & 0x03;	//Clear the mirror and flip bits.
    switch (imgMirror)
    {
        case IMAGE_HV_MIRROR:
            SP5409MIPI_write_cmos_sensor(0x3f, 0x03);	//Set normal
            break;
        case IMAGE_V_MIRROR:
            SP5409MIPI_write_cmos_sensor(0x3f, iTemp | 0x01);	//Set flip
            break;
        case IMAGE_H_MIRROR:
            SP5409MIPI_write_cmos_sensor(0x3f, iTemp | 0x02);	//Set mirror
            break;
        case IMAGE_NORMAL:
            SP5409MIPI_write_cmos_sensor(0x3f, 0x00);	//Set mirror and flip
            break;
    }
	SENSORDB("[SP5409MIPI]exit SP5409MIPISetFlipMirror function\n");
}


/*************************************************************************
* FUNCTION
*   SP5409MIPIPreview
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
UINT32 SP5409MIPIPreview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                                                MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	kal_uint16 iStartX = 0, iStartY = 0;	
	SENSORDB("[SP5409MIPI]enter SP5409MIPIPreview function\n");
	spin_lock(&sp5409_drv_lock);    
	SP5409MIPI_MPEG4_encode_mode = KAL_FALSE;
	SP5409MIPI_sensor.video_mode=KAL_FALSE;
	SP5409MIPI_sensor.pv_mode=KAL_TRUE;
	SP5409MIPI_sensor.capture_mode=KAL_FALSE;
	spin_unlock(&sp5409_drv_lock);

	PreviewSetting();
       SP5409MIPISetFlipMirror(IMAGE_NORMAL);//hanlei

	iStartX += SP5409MIPI_IMAGE_SENSOR_PV_STARTX;
	iStartY += SP5409MIPI_IMAGE_SENSOR_PV_STARTY;
	spin_lock(&sp5409_drv_lock);

	//SP5409MIPI_sensor.pv_line_length = SP5409MIPI_PV_LINE_LENGTH_PIXELS+SP5409MIPI_sensor.pv_dummy_pixels; 
	//SP5409MIPI_sensor.pv_frame_length = SP5409MIPI_PV_FRAME_LENGTH_LINES+SP5409MIPI_sensor.pv_dummy_lines;
	spin_unlock(&sp5409_drv_lock);
	   
	//SP5409MIPI_SetDummy(SP5409MIPI_sensor.pv_dummy_pixels,SP5409MIPI_sensor.pv_dummy_lines);
	SP5409MIPI_SetShutter(SP5409MIPI_sensor.pv_shutter);
	spin_lock(&sp5409_drv_lock);	
	memcpy(&SP5409MIPISensorConfigData, sensor_config_data, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
	spin_unlock(&sp5409_drv_lock);
	
	image_window->GrabStartX= iStartX;
	image_window->GrabStartY= iStartY;
	image_window->ExposureWindowWidth= SP5409MIPI_REAL_PV_WIDTH ;
	image_window->ExposureWindowHeight= SP5409MIPI_REAL_PV_HEIGHT ; 
	SENSORDB("hcy [SP5409MIPI]eXIT SP5409MIPIPreview function\n"); 
	
	return ERROR_NONE;
}	/* SP5409MIPIPreview() */

/*************************************************************************
* FUNCTION
* RETURNS
*   None
*
* GLOBALS AFFECTED
*
*************************************************************************/
UINT32 SP5409MIPIVideo(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                                                MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	SENSORDB("[SP5409MIPI]enter SP5409MIPIVideo function\n"); 
	
	kal_uint16 iStartX = 0, iStartY = 0;

	spin_lock(&sp5409_drv_lock);    
       SP5409MIPI_MPEG4_encode_mode = KAL_TRUE;  
	SP5409MIPI_sensor.video_mode=KAL_TRUE;
	SP5409MIPI_sensor.pv_mode=KAL_FALSE;
	SP5409MIPI_sensor.capture_mode=KAL_FALSE;
	spin_unlock(&sp5409_drv_lock);
	VideoFullSizeSetting();

	SP5409MIPISetFlipMirror(IMAGE_NORMAL);	//add by lishengli 20130614 hanlei
	iStartX += SP5409MIPI_IMAGE_SENSOR_VIDEO_STARTX;
	iStartY += SP5409MIPI_IMAGE_SENSOR_VIDEO_STARTY;
	spin_lock(&sp5409_drv_lock);

	//SP5409MIPI_sensor.video_line_length = SP5409MIPI_VIDEO_LINE_LENGTH_PIXELS+SP5409MIPI_sensor.video_dummy_pixels; 
	//SP5409MIPI_sensor.video_frame_length = SP5409MIPI_VIDEO_FRAME_LENGTH_LINES+SP5409MIPI_sensor.video_dummy_lines;
	spin_unlock(&sp5409_drv_lock);

	//SP5409MIPI_SetDummy(SP5409MIPI_sensor.video_dummy_pixels,SP5409MIPI_sensor.video_dummy_lines);
	SP5409MIPI_SetShutter(SP5409MIPI_sensor.video_shutter);
	spin_lock(&sp5409_drv_lock);	
	memcpy(&SP5409MIPISensorConfigData, sensor_config_data, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
	spin_unlock(&sp5409_drv_lock);
	image_window->GrabStartX= iStartX;
	image_window->GrabStartY= iStartY;    
    SENSORDB("[SP5409MIPI]eXIT SP5409MIPIVideo function\n"); 
	return ERROR_NONE;
}	/* SP5409MIPIPreview() */

UINT32 SP5409MIPICapture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                                                MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
       SENSORDB("[SP5409MIPI]enter SP5409MIPICapture function\n");
	  
	kal_uint16 iStartX = 0, iStartY = 0;

	spin_lock(&sp5409_drv_lock);	
	SP5409MIPI_sensor.video_mode=KAL_FALSE;
	SP5409MIPI_sensor.pv_mode=KAL_FALSE;
	SP5409MIPI_sensor.capture_mode=KAL_TRUE;
	SP5409MIPI_MPEG4_encode_mode = KAL_FALSE; 
	SP5409MIPI_Auto_Flicker_mode = KAL_FALSE;    
	spin_unlock(&sp5409_drv_lock);
    
		SP5409MIPI_set_5M();
		SP5409MIPISetFlipMirror(IMAGE_NORMAL);
		spin_lock(&sp5409_drv_lock);

		//SP5409MIPI_sensor.cp_line_length=SP5409MIPI_FULL_LINE_LENGTH_PIXELS+SP5409MIPI_sensor.cp_dummy_pixels;
		//SP5409MIPI_sensor.cp_frame_length=SP5409MIPI_FULL_FRAME_LENGTH_LINES+SP5409MIPI_sensor.cp_dummy_lines;
		spin_unlock(&sp5409_drv_lock);
		iStartX = SP5409MIPI_IMAGE_SENSOR_CAP_STARTX;
		iStartY = SP5409MIPI_IMAGE_SENSOR_CAP_STARTY;
		image_window->GrabStartX=iStartX;
		image_window->GrabStartY=iStartY;
		image_window->ExposureWindowWidth=SP5409MIPI_REAL_CAP_WIDTH ;
		image_window->ExposureWindowHeight=SP5409MIPI_REAL_CAP_HEIGHT;
		//SP5409MIPI_SetDummy(SP5409MIPI_sensor.cp_dummy_pixels, SP5409MIPI_sensor.cp_dummy_lines);   
	
	
	spin_lock(&sp5409_drv_lock);	
	memcpy(&SP5409MIPISensorConfigData, sensor_config_data, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
	spin_unlock(&sp5409_drv_lock);
	
	SENSORDB("[SP5409MIPI]exit SP5409MIPICapture function\n");
	return ERROR_NONE;
}	/* SP5409MIPICapture() */

UINT32 SP5409MIPIGetResolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *pSensorResolution)
{
    SENSORDB("[SP5409MIPI]eXIT SP5409MIPIGetResolution function\n");
    pSensorResolution->SensorPreviewWidth	= SP5409MIPI_REAL_PV_WIDTH;
    pSensorResolution->SensorPreviewHeight	= SP5409MIPI_REAL_PV_HEIGHT;
    pSensorResolution->SensorFullWidth		= SP5409MIPI_REAL_CAP_WIDTH;
    pSensorResolution->SensorFullHeight		= SP5409MIPI_REAL_CAP_HEIGHT;
    pSensorResolution->SensorVideoWidth		= SP5409MIPI_REAL_VIDEO_WIDTH;
    pSensorResolution->SensorVideoHeight    = SP5409MIPI_REAL_VIDEO_HEIGHT;
    SENSORDB("SP5409MIPIGetResolution :8-14");    

    return ERROR_NONE;
}   /* SP5409MIPIGetResolution() */

UINT32 SP5409MIPIGetInfo(MSDK_SCENARIO_ID_ENUM ScenarioId,
                                                MSDK_SENSOR_INFO_STRUCT *pSensorInfo,
                                                MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{ 
	SENSORDB("[SP5409MIPI]enter SP5409MIPIGetInfo function\n");
	
      pSensorInfo->SensorOutputDataFormat=SENSOR_OUTPUT_FORMAT_RAW_B;//SENSOR_OUTPUT_FORMAT_RAW_B;hanlei
      pSensorInfo->SensorClockPolarity=SENSOR_CLOCK_POLARITY_LOW; 
      
      pSensorInfo->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
      pSensorInfo->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_HIGH;

      pSensorInfo->SensroInterfaceType=SENSOR_INTERFACE_TYPE_MIPI;//hanlei//SENSOR_INTERFACE_TYPE_PARALLEL;
			
    pSensorInfo->CaptureDelayFrame = 1; //2//hanlei140401
      pSensorInfo->PreviewDelayFrame =  1; //old 2
      pSensorInfo->VideoDelayFrame = 2; 

      pSensorInfo->SensorDrivingCurrent = ISP_DRIVING_8MA; //hanlei dri     
      pSensorInfo->AEShutDelayFrame = 0;	//  2   //0//hanlei	    /* The frame of setting shutter default 0 for TG int */
      pSensorInfo->AESensorGainDelayFrame =  0;  // 2  //0//hanlei   /* The frame of setting sensor gain */
      pSensorInfo->AEISPGainDelayFrame =  1;
	   
      switch (ScenarioId)
      {
          case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
               pSensorInfo->SensorClockFreq=24;

               pSensorInfo->SensorClockRisingCount= 0;

               pSensorInfo->SensorGrabStartX = SP5409MIPI_IMAGE_SENSOR_PV_STARTX; 
               pSensorInfo->SensorGrabStartY = SP5409MIPI_IMAGE_SENSOR_PV_STARTY;           		 
               pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_2_LANE;			
               pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0; 
	        pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 14; 
	        pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
               pSensorInfo->SensorWidthSampling = 0;  // 0 is default 1x
               pSensorInfo->SensorHightSampling = 0;   // 0 is default 1x 
               pSensorInfo->SensorPacketECCOrder = 1;
			
               break;	
          case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
               pSensorInfo->SensorClockFreq=24;
               pSensorInfo->SensorClockRisingCount= 0;

               pSensorInfo->SensorGrabStartX = SP5409MIPI_IMAGE_SENSOR_VIDEO_STARTX; 
               pSensorInfo->SensorGrabStartY = SP5409MIPI_IMAGE_SENSOR_VIDEO_STARTY;				   
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
               pSensorInfo->SensorClockRisingCount= 0;
               
               pSensorInfo->SensorGrabStartX = SP5409MIPI_IMAGE_SENSOR_CAP_STARTX;//	//2*SP5409MIPI_IMAGE_SENSOR_PV_STARTX; 
               pSensorInfo->SensorGrabStartY = SP5409MIPI_IMAGE_SENSOR_CAP_STARTY;//	//2*SP5409MIPI_IMAGE_SENSOR_PV_STARTY;          			
               pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_2_LANE;				
               pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0; 
               pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 14; 
               pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;            
               pSensorInfo->SensorWidthSampling = 0;  // 0 is default 1x
               pSensorInfo->SensorHightSampling = 0;   // 0 is default 1x
               pSensorInfo->SensorPacketECCOrder = 1;

               break;
          default:
               pSensorInfo->SensorClockFreq=24;
               pSensorInfo->SensorClockRisingCount= 0;
               pSensorInfo->SensorGrabStartX = SP5409MIPI_IMAGE_SENSOR_PV_STARTX; 
               pSensorInfo->SensorGrabStartY = SP5409MIPI_IMAGE_SENSOR_PV_STARTY; 				 
               pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_2_LANE;		 
               pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0; 
               pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 14; 
               pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;		  	 
               pSensorInfo->SensorWidthSampling = 0;	// 0 is default 1x
               pSensorInfo->SensorHightSampling = 0;	 // 0 is default 1x 
               pSensorInfo->SensorPacketECCOrder = 1;
			
               break;
      }
	
      spin_lock(&sp5409_drv_lock);	
      SP5409MIPIPixelClockDivider=pSensorInfo->SensorPixelClockCount;
      memcpy(pSensorConfigData, &SP5409MIPISensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
      spin_unlock(&sp5409_drv_lock);
      SENSORDB("[hcy SP5409MIPI]exit SP5409MIPIGetInfo function\n");
      return ERROR_NONE;
}   /* SP5409MIPIGetInfo() */


UINT32 SP5409MIPIControl(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *pImageWindow,
                                                MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{    
		spin_lock(&sp5409_drv_lock);	
		CurrentScenarioId = ScenarioId;
		spin_unlock(&sp5409_drv_lock);
		SENSORDB("[SP5409MIPI]enter SP5409MIPIControl function\n");
    switch (ScenarioId)
    {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
            SP5409MIPIPreview(pImageWindow, pSensorConfigData);
            break;
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			SP5409MIPIVideo(pImageWindow, pSensorConfigData);
            break;
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
	      case MSDK_SCENARIO_ID_CAMERA_ZSD:
            SP5409MIPICapture(pImageWindow, pSensorConfigData);//hhl 2-28
            break;
        default:
            return ERROR_INVALID_SCENARIO_ID;
    }
	SENSORDB("[SP5409MIPI]exit SP5409MIPIControl function\n");
    return ERROR_NONE;
} /* SP5409MIPIControl() */

UINT32 SP5409MIPISetVideoMode(UINT16 u2FrameRate)
{
    SENSORDB("[SP5409MIPISetVideoMode] frame rate = %d\n", u2FrameRate);
	kal_uint16 SP5409MIPI_Video_Max_Expourse_Time = 0;
	SENSORDB("[SP5409MIPI]%s():fix_frame_rate=%d\n",__FUNCTION__,u2FrameRate);
	spin_lock(&sp5409_drv_lock);
	SP5409MIPI_sensor.fix_video_fps = KAL_TRUE;
	spin_unlock(&sp5409_drv_lock);
	u2FrameRate=u2FrameRate*10;//10*FPS
	SENSORDB("[SP5409MIPI][Enter Fix_fps func] SP5409MIPI_Fix_Video_Frame_Rate = %d\n", u2FrameRate/10);
	//SP5409MIPI_Video_Max_Expourse_Time = (kal_uint16)((SP5409MIPI_sensor.video_pclk*10/u2FrameRate)/SP5409MIPI_sensor.video_line_length);
	#if 0
	if (SP5409MIPI_Video_Max_Expourse_Time > SP5409MIPI_VIDEO_FRAME_LENGTH_LINES/*SP5409MIPI_sensor.pv_frame_length*/) 
	{
		spin_lock(&sp5409_drv_lock);    
		SP5409MIPI_sensor.video_frame_length = SP5409MIPI_Video_Max_Expourse_Time;
		SP5409MIPI_sensor.video_dummy_lines = SP5409MIPI_sensor.video_frame_length-SP5409MIPI_VIDEO_FRAME_LENGTH_LINES;
		spin_unlock(&sp5409_drv_lock);
		
		SP5409MIPI_SetDummy(SP5409MIPI_sensor.video_dummy_pixels,SP5409MIPI_sensor.video_dummy_lines);
	}
	#endif
	spin_lock(&sp5409_drv_lock);    
	SP5409MIPI_MPEG4_encode_mode = KAL_TRUE; 
	spin_unlock(&sp5409_drv_lock);
	SENSORDB("[SP5409MIPI]exit SP5409MIPISetVideoMode function\n");


	
	return ERROR_NONE;
}

UINT32 SP5409MIPISetAutoFlickerMode(kal_bool bEnable, UINT16 u2FrameRate)
{
	kal_uint32 pv_max_frame_rate_lines=0;

	if(SP5409MIPI_sensor.pv_mode==TRUE)
	pv_max_frame_rate_lines=SP5409MIPI_PV_FRAME_LENGTH_LINES;
	else
    pv_max_frame_rate_lines=SP5409MIPI_VIDEO_FRAME_LENGTH_LINES	;
    SENSORDB("[SP5409MIPISetAutoFlickerMode] frame rate(10base) = %d %d\n", bEnable, u2FrameRate);
    if(bEnable) 
	{   // enable auto flicker   
    	spin_lock(&sp5409_drv_lock);    
        SP5409MIPI_Auto_Flicker_mode = KAL_TRUE; 
		spin_unlock(&sp5409_drv_lock);
        if(SP5409MIPI_MPEG4_encode_mode == KAL_TRUE) 
		{ // in the video mode, reset the frame rate
            pv_max_frame_rate_lines = SP5409MIPI_MAX_EXPOSURE_LINES + (SP5409MIPI_MAX_EXPOSURE_LINES>>7);            
           // SP5409MIPI_write_cmos_sensor(0x0104, 1);        
            //SP5409MIPI_write_cmos_sensor(0x0340, (pv_max_frame_rate_lines >>8) & 0xFF);
            //SP5409MIPI_write_cmos_sensor(0x0341, pv_max_frame_rate_lines & 0xFF);	
            //SP5409MIPI_write_cmos_sensor(0x0104, 0);        	
        }
    } 
	else 
	{
    	spin_lock(&sp5409_drv_lock);    
        SP5409MIPI_Auto_Flicker_mode = KAL_FALSE; 
		spin_unlock(&sp5409_drv_lock);
        if(SP5409MIPI_MPEG4_encode_mode == KAL_TRUE) 
		{    // in the video mode, restore the frame rate
            //SP5409MIPI_write_cmos_sensor(0x0104, 1);        
            //SP5409MIPI_write_cmos_sensor(0x0340, (SP5409MIPI_MAX_EXPOSURE_LINES >>8) & 0xFF);
            //SP5409MIPI_write_cmos_sensor(0x0341, SP5409MIPI_MAX_EXPOSURE_LINES & 0xFF);	
           // SP5409MIPI_write_cmos_sensor(0x0104, 0);        	
        }
        printk("Disable Auto flicker\n");    
    }
    return ERROR_NONE;
}
UINT32 SP5409MIPISetMaxFramerateByScenario(MSDK_SCENARIO_ID_ENUM scenarioId, MUINT32 frameRate) {
	kal_uint32 pclk;
	kal_int16 dummyLine;
	kal_uint16 lineLength,frameHeight;	
	SENSORDB("SP5409MIPISetMaxFramerateByScenario: scenarioId = %d, frame rate = %d\n",scenarioId,frameRate);
	switch (scenarioId) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			pclk =SP5409MIPI_sensor.pv_pclk;//24000000;//hanlei
			//lineLength = SP5409MIPI_PV_LINE_LENGTH_PIXELS;
			//lineLength = SP5409MIPI_PV_LINE_LENGTH_PIXELS+SP5409MIPI_sensor.pv_dummy_pixels;
			//frameHeight = (10 * pclk)/frameRate/lineLength;
			//dummyLine = frameHeight - SP5409MIPI_PV_FRAME_LENGTH_LINES;

			
			if(dummyLine<0)
				dummyLine = 0;
			
			spin_lock(&sp5409_drv_lock);	
			SP5409MIPI_sensor.pv_mode=TRUE;
			spin_unlock(&sp5409_drv_lock);
			//SP5409MIPI_SetDummy(0, dummyLine);			
			break;			
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			pclk = SP5409MIPI_sensor.video_pclk;//24000000;//hanlei
			//lineLength = SP5409MIPI_VIDEO_LINE_LENGTH_PIXELS;
			//lineLength = SP5409MIPI_VIDEO_LINE_LENGTH_PIXELS+SP5409MIPI_sensor.video_dummy_pixels;//copy hailei
			//frameHeight = (10 * pclk)/frameRate/lineLength;
			//dummyLine = frameHeight - SP5409MIPI_VIDEO_FRAME_LENGTH_LINES;
			if(dummyLine<0)
				dummyLine = 0;
			spin_lock(&sp5409_drv_lock);	
			SP5409MIPI_sensor.pv_mode=TRUE;
			spin_unlock(&sp5409_drv_lock);
			//SP5409MIPI_SetDummy(0, dummyLine);			
			break;			
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			//hanlei add
			////////////////////////////////////////////////////
			pclk = SP5409MIPI_sensor.cp_pclk;//18000000;
			//lineLength = SP5409MIPI_FULL_LINE_LENGTH_PIXELS+SP5409MIPI_sensor.cp_dummy_pixels;//copy hanlei
			//frameHeight = (10 * pclk)/frameRate/lineLength;
			//dummyLine = frameHeight - SP5409MIPI_FULL_FRAME_LENGTH_LINES;
			if(dummyLine<0)
				dummyLine = 0;
			
			spin_lock(&sp5409_drv_lock);	
			SP5409MIPI_sensor.pv_mode=FALSE;
			spin_unlock(&sp5409_drv_lock);
			//SP5409MIPI_SetDummy(0, dummyLine);	
			////////////////////////////////////////////////////
			break;
		case MSDK_SCENARIO_ID_CAMERA_ZSD:			
			pclk = SP5409MIPI_sensor.cp_pclk;//12000000;//hanlei
			//lineLength = SP5409MIPI_FULL_LINE_LENGTH_PIXELS+SP5409MIPI_sensor.cp_dummy_pixels;//copy hanlei
			//frameHeight = (10 * pclk)/frameRate/lineLength;
			//dummyLine = frameHeight - SP5409MIPI_FULL_FRAME_LENGTH_LINES;
			if(dummyLine<0)
				dummyLine = 0;
			
			spin_lock(&sp5409_drv_lock);	
			SP5409MIPI_sensor.pv_mode=FALSE;
			spin_unlock(&sp5409_drv_lock);
			//SP5409MIPI_SetDummy(0, dummyLine);			
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
	SENSORDB("[SP5409MIPI]exit SP5409MIPISetMaxFramerateByScenario function\n");
	return ERROR_NONE;
}
UINT32 SP5409MIPIGetDefaultFramerateByScenario(MSDK_SCENARIO_ID_ENUM scenarioId, MUINT32 *pframeRate) 
{

	switch (scenarioId) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			 *pframeRate = 180;//120; //hanlei add
			 break;			
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			 *pframeRate = 180;//150; //hanlei
			 break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
			 *pframeRate = 60;//80 ;//hanlei
			break;		//hhl 2-28
        case MSDK_SCENARIO_ID_CAMERA_3D_PREVIEW: //added
        case MSDK_SCENARIO_ID_CAMERA_3D_VIDEO:
        case MSDK_SCENARIO_ID_CAMERA_3D_CAPTURE: //added   
			 *pframeRate = 180;//150 ;//hanlei
			break;		
		default:
			break;
	}

	return ERROR_NONE;
}
UINT32 SP5409MIPISetTestPatternMode(kal_bool bEnable)
{
    SENSORDB("[SP5409MIPISetTestPatternMode] Test pattern enable:%d\n", bEnable);
    
    if(bEnable) {   // enable color bar   
        //SP5409MIPI_write_cmos_sensor(0x30D8, 0x10);  // color bar test pattern
        //SP5409MIPI_write_cmos_sensor(0x0600, 0x00);  // color bar test pattern
        //SP5409MIPI_write_cmos_sensor(0x0601, 0x02);  // color bar test pattern 
    } else {
        //SP5409MIPI_write_cmos_sensor(0x30D8, 0x00);  // disable color bar test pattern
    }
    return ERROR_NONE;
}

UINT32 SP5409MIPIFeatureControl(MSDK_SENSOR_FEATURE_ENUM FeatureId,
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
            *pFeatureReturnPara16++=SP5409MIPI_REAL_CAP_WIDTH;
            *pFeatureReturnPara16=SP5409MIPI_REAL_CAP_HEIGHT;
            *pFeatureParaLen=4;
            break;
        case SENSOR_FEATURE_GET_PERIOD:
        		switch(CurrentScenarioId)
        		{
        			case MSDK_SCENARIO_ID_CAMERA_ZSD:
        		    case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
 		            *pFeatureReturnPara16++=SP5409MIPI_sensor.cp_line_length;  
 		            *pFeatureReturnPara16=SP5409MIPI_sensor.cp_frame_length;
		            SENSORDB("Sensor period:%d %d\n",SP5409MIPI_sensor.cp_line_length, SP5409MIPI_sensor.cp_frame_length); 
		            *pFeatureParaLen=4;        				
        				break;
        			case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
					*pFeatureReturnPara16++=SP5409MIPI_sensor.video_line_length;  
					*pFeatureReturnPara16=SP5409MIPI_sensor.video_frame_length;
					 SENSORDB("Sensor period:%d %d\n", SP5409MIPI_sensor.video_line_length, SP5409MIPI_sensor.video_frame_length); 
					 *pFeatureParaLen=4;
						break;
        			default:	
					*pFeatureReturnPara16++=SP5409MIPI_sensor.pv_line_length;  
					*pFeatureReturnPara16=SP5409MIPI_sensor.pv_frame_length;
		            SENSORDB("Sensor period:%d %d\n", SP5409MIPI_sensor.pv_line_length, SP5409MIPI_sensor.pv_frame_length); 
		            *pFeatureParaLen=4;
	            break;
          	}
          	break;
        case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
        		switch(CurrentScenarioId)
        		{
        			case MSDK_SCENARIO_ID_CAMERA_ZSD:
        			case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		            *pFeatureReturnPara32 = SP5409MIPI_sensor.cp_pclk; 
		            *pFeatureParaLen=4;		         	
					
		            SENSORDB("Sensor CPCLK:%dn",SP5409MIPI_sensor.cp_pclk); 
		         		break; //hhl 2-28
					case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
						*pFeatureReturnPara32 = SP5409MIPI_sensor.video_pclk;
						*pFeatureParaLen=4;
						SENSORDB("Sensor videoCLK:%d\n",SP5409MIPI_sensor.video_pclk); 
						break;
		         		default:
		            *pFeatureReturnPara32 = SP5409MIPI_sensor.pv_pclk;
		            *pFeatureParaLen=4;
					SENSORDB("Sensor pvclk:%d\n",SP5409MIPI_sensor.pv_pclk); 
		            break;
		         }
		         break;
        case SENSOR_FEATURE_SET_ESHUTTER:
            SP5409MIPI_SetShutter(*pFeatureData16); 
            break;
		case SENSOR_FEATURE_SET_SENSOR_SYNC: 
			break;
        case SENSOR_FEATURE_SET_NIGHTMODE:
            SP5409MIPI_NightMode((BOOL) *pFeatureData16);
            break;
        case SENSOR_FEATURE_SET_GAIN:
           SP5409MIPI_SetGain((UINT16) *pFeatureData16); 
            break;
        case SENSOR_FEATURE_SET_FLASHLIGHT:
            break;
        case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
			spin_lock(&sp5409_drv_lock);    
            SP5409MIPI_isp_master_clock=*pFeatureData32;
			spin_unlock(&sp5409_drv_lock);
            break;
        case SENSOR_FEATURE_SET_REGISTER:
			SP5409MIPI_write_cmos_sensor(pSensorRegData->RegAddr, pSensorRegData->RegData);
            break;
        case SENSOR_FEATURE_GET_REGISTER:
            pSensorRegData->RegData = SP5409MIPI_read_cmos_sensor(pSensorRegData->RegAddr);
            break;
        case SENSOR_FEATURE_SET_CCT_REGISTER:
            SensorRegNumber=FACTORY_END_ADDR;
            for (i=0;i<SensorRegNumber;i++)
            {
            	spin_lock(&sp5409_drv_lock);    
                SP5409MIPISensorCCT[i].Addr=*pFeatureData32++;
                SP5409MIPISensorCCT[i].Para=*pFeatureData32++; 
				spin_unlock(&sp5409_drv_lock);
            }
            break;
        case SENSOR_FEATURE_GET_CCT_REGISTER:
            SensorRegNumber=FACTORY_END_ADDR;
            if (*pFeatureParaLen<(SensorRegNumber*sizeof(SENSOR_REG_STRUCT)+4))
                return FALSE;
            *pFeatureData32++=SensorRegNumber;
            for (i=0;i<SensorRegNumber;i++)
            {
                *pFeatureData32++=SP5409MIPISensorCCT[i].Addr;
                *pFeatureData32++=SP5409MIPISensorCCT[i].Para; 
            }
            break;
        case SENSOR_FEATURE_SET_ENG_REGISTER:
            SensorRegNumber=ENGINEER_END;
            for (i=0;i<SensorRegNumber;i++)
            {	spin_lock(&sp5409_drv_lock);    
                SP5409MIPISensorReg[i].Addr=*pFeatureData32++;
                SP5409MIPISensorReg[i].Para=*pFeatureData32++;
				spin_unlock(&sp5409_drv_lock);
            }
            break;
        case SENSOR_FEATURE_GET_ENG_REGISTER:
            SensorRegNumber=ENGINEER_END;
            if (*pFeatureParaLen<(SensorRegNumber*sizeof(SENSOR_REG_STRUCT)+4))
                return FALSE;
            *pFeatureData32++=SensorRegNumber;
            for (i=0;i<SensorRegNumber;i++)
            {
                *pFeatureData32++=SP5409MIPISensorReg[i].Addr;
                *pFeatureData32++=SP5409MIPISensorReg[i].Para;
            }
            break;
        case SENSOR_FEATURE_GET_REGISTER_DEFAULT:
            if (*pFeatureParaLen>=sizeof(NVRAM_SENSOR_DATA_STRUCT))
            {
                pSensorDefaultData->Version=NVRAM_CAMERA_SENSOR_FILE_VERSION;
                pSensorDefaultData->SensorId=SP5409MIPI_SENSOR_ID;
                memcpy(pSensorDefaultData->SensorEngReg, SP5409MIPISensorReg, sizeof(SENSOR_REG_STRUCT)*ENGINEER_END);
                memcpy(pSensorDefaultData->SensorCCTReg, SP5409MIPISensorCCT, sizeof(SENSOR_REG_STRUCT)*FACTORY_END_ADDR);
            }
            else
                return FALSE;
            *pFeatureParaLen=sizeof(NVRAM_SENSOR_DATA_STRUCT);
            break;
        case SENSOR_FEATURE_GET_CONFIG_PARA:
            memcpy(pSensorConfigData, &SP5409MIPISensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
            *pFeatureParaLen=sizeof(MSDK_SENSOR_CONFIG_STRUCT);
            break;
        case SENSOR_FEATURE_CAMERA_PARA_TO_SENSOR:
            SP5409MIPI_camera_para_to_sensor();
            break;

        case SENSOR_FEATURE_SENSOR_TO_CAMERA_PARA:
            SP5409MIPI_sensor_to_camera_para();
            break;
        case SENSOR_FEATURE_GET_GROUP_COUNT:
            *pFeatureReturnPara32++=SP5409MIPI_get_sensor_group_count();
            *pFeatureParaLen=4;
            break;
        case SENSOR_FEATURE_GET_GROUP_INFO:
            SP5409MIPI_get_sensor_group_info(pSensorGroupInfo->GroupIdx, pSensorGroupInfo->GroupNamePtr, &pSensorGroupInfo->ItemCount);
            *pFeatureParaLen=sizeof(MSDK_SENSOR_GROUP_INFO_STRUCT);
            break;
        case SENSOR_FEATURE_GET_ITEM_INFO:
            SP5409MIPI_get_sensor_item_info(pSensorItemInfo->GroupIdx,pSensorItemInfo->ItemIdx, pSensorItemInfo);
            *pFeatureParaLen=sizeof(MSDK_SENSOR_ITEM_INFO_STRUCT);
            break;

        case SENSOR_FEATURE_SET_ITEM_INFO:
            SP5409MIPI_set_sensor_item_info(pSensorItemInfo->GroupIdx, pSensorItemInfo->ItemIdx, pSensorItemInfo->ItemValue);
            *pFeatureParaLen=sizeof(MSDK_SENSOR_ITEM_INFO_STRUCT);
            break;

        case SENSOR_FEATURE_GET_ENG_INFO:
            pSensorEngInfo->SensorId = 129;
            pSensorEngInfo->SensorType = CMOS_SENSOR;
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
            SP5409MIPISetVideoMode(*pFeatureData16);
            break;
        case SENSOR_FEATURE_CHECK_SENSOR_ID:
            SP5409MIPIGetSensorID(pFeatureReturnPara32); 
            break;             
        case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE:
            SP5409MIPISetAutoFlickerMode((BOOL)*pFeatureData16, *(pFeatureData16+1));            
	        break;
        case SENSOR_FEATURE_SET_TEST_PATTERN:
            SP5409MIPISetTestPatternMode((BOOL)*pFeatureData16);        	
            break;
	 case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
			SP5409MIPISetMaxFramerateByScenario((MSDK_SCENARIO_ID_ENUM)*pFeatureData32, *(pFeatureData32+1));
			break;
	 case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
			SP5409MIPIGetDefaultFramerateByScenario((MSDK_SCENARIO_ID_ENUM)*pFeatureData32, (MUINT32 *)(*(pFeatureData32+1)));
			break;
        default:
            break;
    }
    return ERROR_NONE;
}	/* SP5409MIPIFeatureControl() */


SENSOR_FUNCTION_STRUCT	SensorFuncSP5409MIPI=
{
    SP5409MIPIOpen,
    SP5409MIPIGetInfo,
    SP5409MIPIGetResolution,
    SP5409MIPIFeatureControl,
    SP5409MIPIControl,
    SP5409MIPIClose
};

UINT32 SP5409_MIPI_RAW_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
    /* To Do : Check Sensor status here */


#ifdef  ONLINE_DEBUG//for online debug
      if(ONLINE_DEBUG_BZW==KAL_TRUE)
      	{
	  fcam_class = class_create(THIS_MODULE, "fcam");
		if (IS_ERR(fcam_class)) 
		{
			printk("Create class fcam.\n");
			return -ENOMEM;
		}
		fcam_dev = device_create(fcam_class, NULL, MKDEV(0, 1), NULL, "dev");
		 device_create_file(fcam_dev, &dev_attr_fcam);
		 device_create_file(fcam_dev, &dev_attr_currreg);
	       ONLINE_DEBUG_BZW=KAL_FALSE;
       }

#endif


	
    if (pfFunc!=NULL)
        *pfFunc=&SensorFuncSP5409MIPI;
    return ERROR_NONE;
}   /* SensorInit() */

