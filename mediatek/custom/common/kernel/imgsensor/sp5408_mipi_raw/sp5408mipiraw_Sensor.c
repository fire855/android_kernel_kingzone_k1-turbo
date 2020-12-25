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

#include "sp5408mipiraw_Sensor.h"
#include "sp5408mipiraw_Camera_Sensor_para.h"
#include "sp5408mipiraw_CameraCustomized.h"

kal_bool  SP5408MIPI_MPEG4_encode_mode = KAL_FALSE;
kal_bool SP5408MIPI_Auto_Flicker_mode = KAL_FALSE;
kal_bool ONLINE_DEBUG_BZW = KAL_TRUE;


kal_uint8 SP5408MIPI_sensor_write_I2C_address = SP5408MIPI_WRITE_ID;
kal_uint8 SP5408MIPI_sensor_read_I2C_address = SP5408MIPI_READ_ID;

//#define ONLINE_DEBUG
#define DEBUG_SENSOR
#ifdef DEBUG_SENSOR
	#define SP5408MIPI_OP_CODE_INI		0x00		/* Initial value. */
	#define SP5408MIPI_OP_CODE_REG		0x01		/* Register */
	#define SP5408MIPI_OP_CODE_DLY		0x02		/* Delay */
	#define SP5408MIPI_OP_CODE_END		0x03		/* End of initial setting. */
	

		typedef struct
	{
		u16 init_reg;
		u16 init_val;	/* Save the register value and delay tick */
		u8 op_code;		/* 0 - Initial value, 1 - Register, 2 - Delay, 3 - End of setting. */
	} SP5408MIPI_initial_set_struct;

	SP5408MIPI_initial_set_struct SP5408MIPI_Init_Reg[1000];
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

 u8 SP5408MIPI_Initialize_from_T_Flash()
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
 
    fp = filp_open("/mnt/sdcard/sp5408_sd", O_RDONLY , 0); 
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
			SP5408MIPI_Init_Reg[i].op_code = SP5408MIPI_OP_CODE_REG;
			
			SP5408MIPI_Init_Reg[i].init_reg = strtol((const char *)curr_ptr, 16);
			curr_ptr += 5;	/* Skip "00, 0x" */
		
			SP5408MIPI_Init_Reg[i].init_val = strtol((const char *)curr_ptr, 16);
			curr_ptr += 4;	/* Skip "00);" */
		
		}
		else									/* DLY */
		{
			/* Need add delay for this setting. */
			curr_ptr += 4;	
			SP5408MIPI_Init_Reg[i].op_code = SP5408MIPI_OP_CODE_DLY;
			
			SP5408MIPI_Init_Reg[i].init_reg = 0xFF;
			SP5408MIPI_Init_Reg[i].init_val = strtol((const char *)curr_ptr,  10);	/* Get the delay ticks, the delay should less then 50 */
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
	SP5408MIPI_Init_Reg[i].op_code = SP5408MIPI_OP_CODE_END;
	SP5408MIPI_Init_Reg[i].init_reg = 0xFF;
	SP5408MIPI_Init_Reg[i].init_val = 0xFF;
	i++;
	//for (j=0; j<i; j++)
		//printk(" %x  ==  %x\n",SP5408MIPI_Init_Reg[j].init_reg, SP5408MIPI_Init_Reg[j].init_val);

	/* Start apply the initial setting to sensor. */
	#if 1
	for (j=0; j<i; j++)
	{
		if (SP5408MIPI_Init_Reg[j].op_code == SP5408MIPI_OP_CODE_END)	/* End of the setting. */
		{
			break ;
		}
		else if (SP5408MIPI_Init_Reg[j].op_code == SP5408MIPI_OP_CODE_DLY)
		{
			msleep(SP5408MIPI_Init_Reg[j].init_val);		/* Delay */
		}
		else if (SP5408MIPI_Init_Reg[j].op_code == SP5408MIPI_OP_CODE_REG)
		{
		
			SP5408MIPI_write_cmos_sensor(SP5408MIPI_Init_Reg[j].init_reg, SP5408MIPI_Init_Reg[j].init_val);
			printk("%x = %x\n",SP5408MIPI_Init_Reg[j].init_reg,SP5408MIPI_Init_Reg[j].init_val);
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

static ssize_t fcam_show(struct device *dev, struct device_attribute *attr, char *_buf)
{
	return sprintf(_buf, "0x%02x=0x%02x\n", cur_reg, cur_val);
}

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
		val=SP5408MIPI_read_cmos_sensor(cur_reg);//hanlei
		
		printk("%s(): sp5408r get 0x%02x=0x%02x\n", __FUNCTION__, cur_reg, val);
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
			SP5408MIPI_write_cmos_sensor(reg,val);//hanlei
			//msm_camera_i2c_read(ov7695_s_ctrl.sensor_i2c_client, reg, &tmp,MSM_CAMERA_I2C_BYTE_DATA);
			tmp=SP5408MIPI_read_cmos_sensor(reg);
			
			printk("%s(): sp5408w set 0x%02x=0x%02x==(0x%02x)\n", __FUNCTION__, reg, val,tmp);
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
    strcpy(_buf, "SP5408");
    return 4;
}

static struct device *fcam_dev = NULL;
static struct class *  fcam_class = NULL;
static DEVICE_ATTR(fcam, 0666, fcam_show, fcam_store);
static DEVICE_ATTR(currreg, 0666, currreg_show, NULL);
#endif



//#define SP5408MIPI_USE_OTP

#ifdef SP5408MIPI_USE_OTP
static uint16_t used_otp = 0;
static uint16_t ret = -1;
extern int sp5408_update_otp_wb(void);
extern int sp5408_update_awb_gain(void);
extern int sp5408_check_mid(uint mid);
#endif
	
static struct SP5408MIPI_sensor_STRUCT SP5408MIPI_sensor={SP5408MIPI_WRITE_ID,SP5408MIPI_READ_ID,KAL_TRUE,KAL_FALSE,KAL_TRUE,KAL_FALSE,
//KAL_FALSE,KAL_FALSE,KAL_FALSE,480000000,480000000,480000000,0,0,0,64,64,64,SP5408MIPI_PV_LINE_LENGTH_PIXELS,SP5408MIPI_PV_FRAME_LENGTH_LINES,
KAL_FALSE,KAL_FALSE,KAL_FALSE,24000000,24000000,12000000,1,1,1,16,16,16,SP5408MIPI_PV_LINE_LENGTH_PIXELS,SP5408MIPI_PV_FRAME_LENGTH_LINES,
//SP5408MIPI_VIDEO_LINE_LENGTH_PIXELS,SP5408MIPI_VIDEO_FRAME_LENGTH_LINES,SP5408MIPI_FULL_LINE_LENGTH_PIXELS,SP5408MIPI_FULL_FRAME_LENGTH_LINES,0,0,0,0,0,0,30};
SP5408MIPI_VIDEO_LINE_LENGTH_PIXELS,SP5408MIPI_VIDEO_FRAME_LENGTH_LINES,SP5408MIPI_FULL_LINE_LENGTH_PIXELS,SP5408MIPI_FULL_FRAME_LENGTH_LINES,131,0,131,0,131,0,18};
MSDK_SCENARIO_ID_ENUM CurrentScenarioId = MSDK_SCENARIO_ID_CAMERA_PREVIEW;	
kal_uint16	SP5408MIPI_sensor_gain_base=0x10;//0x00//hanlei
/* MAX/MIN Explosure Lines Used By AE Algorithm */
kal_uint16 SP5408MIPI_MAX_EXPOSURE_LINES = SP5408MIPI_PV_FRAME_LENGTH_LINES-5;//650;//5//hanlei
kal_uint8  SP5408MIPI_MIN_EXPOSURE_LINES = 4;//2//hanlei
kal_uint32 SP5408MIPI_isp_master_clock;
static DEFINE_SPINLOCK(sp5408_drv_lock);

#define SENSORDB(fmt, arg...) printk( "[SP5408MIPIRaw] "  fmt, ##arg)
#define RETAILMSG(x,...)
#define TEXT
UINT8 SP5408MIPIPixelClockDivider=0;
kal_uint16 SP5408MIPI_sensor_id=0;
MSDK_SENSOR_CONFIG_STRUCT SP5408MIPISensorConfigData;
kal_uint32 SP5408MIPI_FAC_SENSOR_REG;
kal_uint16 SP5408MIPI_sensor_flip_value; 
#define SP5408MIPI_MaxGainIndex (97)
kal_uint16 SP5408MIPI_sensorGainMapping[SP5408MIPI_MaxGainIndex][2] ={

{ 683,232}
};
/* FIXME: old factors and DIDNOT use now. s*/
SENSOR_REG_STRUCT SP5408MIPISensorCCT[]=CAMERA_SENSOR_CCT_DEFAULT_VALUE;
SENSOR_REG_STRUCT SP5408MIPISensorReg[ENGINEER_END]=CAMERA_SENSOR_REG_DEFAULT_VALUE;
/* FIXME: old factors and DIDNOT use now. e*/
extern int iReadReg(u16 a_u2Addr , u8 * a_puBuff , u16 i2cId);
extern int iWriteReg(u16 a_u2Addr , u32 a_u4Data , u32 a_u4Bytes , u16 i2cId);
//#define SP5408MIPI_write_cmos_sensor(addr, para) iWriteReg((u16) addr , (u32) para , 1, SP5408MIPI_WRITE_ID)

extern int iReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 * a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
extern int iWriteRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u16 i2cId);




/*kal_uint16 SP5408MIPI_read_cmos_sensor(kal_uint32 addr)
{
	kal_uint16 get_byte=0;
    iReadReg((u16) addr ,(u8*)&get_byte,SP5408MIPI_WRITE_ID);
    return get_byte;
}*/

//for test 5408


kal_uint16 SP5408MIPI_write_cmos_sensor(kal_uint32 addr, kal_uint32 para)
{
    char puSendCmd[2] = { (char)(addr & 0xFF) ,(char)(para & 0xFF)};

	//SENSORDB("HCY IIC 8 8");  
	
	iWriteRegI2C(puSendCmd , 2,0x6c);
	return TRUE;

}

kal_uint16 SP5408MIPI_read_cmos_sensor(kal_uint32 addr)
{
	kal_uint16 get_byte=0;
    char puSendCmd[1] = { (char)(addr & 0xFF) };
	iReadRegI2C(puSendCmd , 1, (u8*)&get_byte,1,0x6c);
    return get_byte;
}



//end for test 5408



void SP5408MIPI_write_shutter(kal_uint16 shutter)
{
	kal_uint32 frame_length = 0,line_length=0,shutter1=0;
    	kal_uint32 extra_lines = 0;
	kal_uint32 max_exp_shutter = 0;
	unsigned long flags;	
	SENSORDB("[SP5408MIPI]enter SP5408MIPI_write_shutter function\n"); 
    if (SP5408MIPI_sensor.pv_mode == KAL_TRUE) 
	 {
	   	//shutter=shutter/42;
	   
	   max_exp_shutter = SP5408MIPI_PV_FRAME_LENGTH_LINES + SP5408MIPI_sensor.pv_dummy_lines-5;//5//hanlei
     }
     else if (SP5408MIPI_sensor.video_mode== KAL_TRUE) 
     {
     		//shutter=shutter/42;
       max_exp_shutter = SP5408MIPI_VIDEO_FRAME_LENGTH_LINES + SP5408MIPI_sensor.video_dummy_lines-5;//5//hanlei
	 }	 
     else if (SP5408MIPI_sensor.capture_mode== KAL_TRUE) 
     {

      //shutter=shutter*32/61;//hanlei140401
       max_exp_shutter = SP5408MIPI_FULL_FRAME_LENGTH_LINES + SP5408MIPI_sensor.cp_dummy_lines-5;//5//hanlei
	 }	 
	 else
	 	{
	 	
		SENSORDB("sensor mode error\n");
	 	}
	 
	 if(shutter > max_exp_shutter)
	   extra_lines = shutter - max_exp_shutter;
	 else 
	   extra_lines = 0;
	 if (SP5408MIPI_sensor.pv_mode == KAL_TRUE) 
	 {
       frame_length =SP5408MIPI_PV_FRAME_LENGTH_LINES+ SP5408MIPI_sensor.pv_dummy_lines + extra_lines;
	   line_length = SP5408MIPI_PV_LINE_LENGTH_PIXELS+ SP5408MIPI_sensor.pv_dummy_pixels;
	   spin_lock_irqsave(&sp5408_drv_lock,flags);
	   SP5408MIPI_sensor.pv_line_length = line_length;
	   SP5408MIPI_sensor.pv_frame_length = frame_length;
	   spin_unlock_irqrestore(&sp5408_drv_lock,flags);
	 }
	 else if (SP5408MIPI_sensor.video_mode== KAL_TRUE) 
     {
	    frame_length = SP5408MIPI_VIDEO_FRAME_LENGTH_LINES+ SP5408MIPI_sensor.video_dummy_lines + extra_lines;
		line_length =SP5408MIPI_VIDEO_LINE_LENGTH_PIXELS + SP5408MIPI_sensor.video_dummy_pixels;
		spin_lock_irqsave(&sp5408_drv_lock,flags);
		SP5408MIPI_sensor.video_line_length = line_length;
	    SP5408MIPI_sensor.video_frame_length = frame_length;
		spin_unlock_irqrestore(&sp5408_drv_lock,flags);
	 } 
	 else if(SP5408MIPI_sensor.capture_mode== KAL_TRUE)
	 	{
	    frame_length = SP5408MIPI_FULL_FRAME_LENGTH_LINES+ SP5408MIPI_sensor.cp_dummy_lines + extra_lines;
		line_length =SP5408MIPI_FULL_LINE_LENGTH_PIXELS + SP5408MIPI_sensor.cp_dummy_pixels;
		spin_lock_irqsave(&sp5408_drv_lock,flags);
		SP5408MIPI_sensor.cp_line_length = line_length;
	    SP5408MIPI_sensor.cp_frame_length = frame_length;
		spin_unlock_irqrestore(&sp5408_drv_lock,flags);
	 }
	 else
	 	{
	 	
		SENSORDB("sensor mode error\n");
	 	}
	 
#if 0
	 
	//SP5408MIPI_write_cmos_sensor(0x0100,0x00);// STREAM STop
	SP5408MIPI_write_cmos_sensor(0x0104, 1);        
	SP5408MIPI_write_cmos_sensor(0x0340, (frame_length >>8) & 0xFF);
    SP5408MIPI_write_cmos_sensor(0x0341, frame_length & 0xFF);	  
    SP5408MIPI_write_cmos_sensor(0x0202, (shutter >> 8) & 0xFF);
    SP5408MIPI_write_cmos_sensor(0x0203, shutter  & 0xFF);
    SP5408MIPI_write_cmos_sensor(0x0104, 0);    
#endif 
    //shutter=0x5300;
    SP5408MIPI_write_cmos_sensor(0xfd, 0x01);
    SP5408MIPI_write_cmos_sensor(0x03, (shutter >> 8) & 0xFF);
    SP5408MIPI_write_cmos_sensor(0x04, shutter  & 0xFF);
    SP5408MIPI_write_cmos_sensor(0x01, 0x01);    

	
    SENSORDB("hcy  SP5408MIPI_write_shutter  =%d \n" , shutter );
}   /* write_SP5408MIPI_shutter */

static kal_uint16 SP5408MIPIReg2Gain(const kal_uint8 iReg)
{
	SENSORDB("[SP5408MIPI]enter SP5408MIPIReg2Gain function\n");

	SENSORDB("[SP5408MIPI]exit SP5408MIPIReg2Gain function\n");
    return ;
}
static kal_uint8 SP5408MIPIGain2Reg(const kal_uint16 iGain)
{
	kal_uint8 iI;
    	SENSORDB("[SP5408MIPI]enter SP5408MIPIGain2Reg function\n");
	SENSORDB("[SP5408MIPI]exit SP5408MIPIGain2Reg function\n");
    return  ;
	//return NONE;
}

/*************************************************************************
* FUNCTION
*    SP5408MIPI_SetGain
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


void SP5408MIPI_SetGain(UINT16 iGain)
{   
    kal_uint8 iReg;
	SENSORDB("hcy  SP5408MIPI_SetGain function=%d\n",iGain);
	

#if 0
    iReg = SP5408MIPIGain2Reg(iGain);
	SP5408MIPI_write_cmos_sensor(0x0104, 1);
    SP5408MIPI_write_cmos_sensor(0x0205, (kal_uint8)iReg);
    SP5408MIPI_write_cmos_sensor(0x0104, 0);
    SENSORDB("[SP5408MIPI]exit SP5408MIPI_SetGain function\n");
#endif 

	/*iGain=iGain/4;
	if(iGain>255)
		iGain=255;
	if(iGain<1)
		iGain=0x30; //10//hanlei */

  if(iGain >= BASEGAIN && iGain <= 15*BASEGAIN)
    {   
        iReg = 0x10 * iGain/BASEGAIN;        //change mtk gain base to aptina gain base

	if(iReg<=0x30)
		{
	 SP5408MIPI_write_cmos_sensor(0xfd, 0x01);
        SP5408MIPI_write_cmos_sensor(0x24, 0x30);//0x23
        SP5408MIPI_write_cmos_sensor(0x01, 0x01);
         SENSORDB("SP5408 reg_gain =%d, SP5408MIPI igain = %d", iReg, iGain);
	       }
	else if(iReg>= 0xc0)
		{
	 SP5408MIPI_write_cmos_sensor(0xfd, 0x01);
        SP5408MIPI_write_cmos_sensor(0x24,0xc0);
        SP5408MIPI_write_cmos_sensor(0x01, 0x01);
        SENSORDB("SP5408MIPI reg_gain =%d, SP5408MIPI igain = %d", iReg, iGain);
	        }        	
      else
		{
	 SP5408MIPI_write_cmos_sensor(0xfd, 0x01);
        SP5408MIPI_write_cmos_sensor(0x24, (kal_uint8)iReg);
        SP5408MIPI_write_cmos_sensor(0x01, 0x01);}	
   }	
  else
        SENSORDB("error gain setting");

	/*
	SP5408MIPI_write_cmos_sensor(0xfd, 0x01);
        SP5408MIPI_write_cmos_sensor(0x24, (kal_uint8)iGain);
        SP5408MIPI_write_cmos_sensor(0x01, 0x01);*/

}   /*  SP5408MIPI_SetGain_SetGain  */


/*************************************************************************
* FUNCTION
*    read_SP5408MIPI_gain
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
kal_uint16 read_SP5408MIPI_gain(void)
{  
	SENSORDB("[SP5408MIPI]enter read_SP5408MIPI_gain function\n");
		SP5408MIPI_write_cmos_sensor(0xfd, 0x01);

    return (kal_uint16)(SP5408MIPI_read_cmos_sensor(0x24)) ;	//  N*4 //hanlei
}  /* read_SP5408MIPI_gain */

void write_SP5408MIPI_gain(kal_uint16 gain)
{
    SP5408MIPI_SetGain(gain);
}


void SP5408MIPI_camera_para_to_sensor(void)
{

	kal_uint32    i;
	SENSORDB("[SP5408MIPI]enter SP5408MIPI_camera_para_to_sensor function\n");
    for(i=0; 0xFFFFFFFF!=SP5408MIPISensorReg[i].Addr; i++)
    {
        SP5408MIPI_write_cmos_sensor(SP5408MIPISensorReg[i].Addr, SP5408MIPISensorReg[i].Para);
    }
    for(i=ENGINEER_START_ADDR; 0xFFFFFFFF!=SP5408MIPISensorReg[i].Addr; i++)
    {
        SP5408MIPI_write_cmos_sensor(SP5408MIPISensorReg[i].Addr, SP5408MIPISensorReg[i].Para);
    }
    for(i=FACTORY_START_ADDR; i<FACTORY_END_ADDR; i++)
    {
        SP5408MIPI_write_cmos_sensor(SP5408MIPISensorCCT[i].Addr, SP5408MIPISensorCCT[i].Para);
    }
	SENSORDB("[SP5408MIPI]exit SP5408MIPI_camera_para_to_sensor function\n");

}


/*************************************************************************
* FUNCTION
*    SP5408MIPI_sensor_to_camera_para
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
void SP5408MIPI_sensor_to_camera_para(void)
{

	kal_uint32    i,temp_data;
	SENSORDB("[SP5408MIPI]enter SP5408MIPI_sensor_to_camera_para function\n");
    for(i=0; 0xFFFFFFFF!=SP5408MIPISensorReg[i].Addr; i++)
    {
		temp_data=SP5408MIPI_read_cmos_sensor(SP5408MIPISensorReg[i].Addr);
		spin_lock(&sp5408_drv_lock);
		SP5408MIPISensorReg[i].Para = temp_data;
		spin_unlock(&sp5408_drv_lock);
    }
    for(i=ENGINEER_START_ADDR; 0xFFFFFFFF!=SP5408MIPISensorReg[i].Addr; i++)
    {
    	temp_data=SP5408MIPI_read_cmos_sensor(SP5408MIPISensorReg[i].Addr);
         spin_lock(&sp5408_drv_lock);
        SP5408MIPISensorReg[i].Para = temp_data;
		spin_unlock(&sp5408_drv_lock);
    }
	SENSORDB("[SP5408MIPI]exit SP5408MIPI_sensor_to_camera_para function\n");

}

/*************************************************************************
* FUNCTION
*    SP5408MIPI_get_sensor_group_count
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
kal_int32  SP5408MIPI_get_sensor_group_count(void)
{
    return GROUP_TOTAL_NUMS;
}

void SP5408MIPI_get_sensor_group_info(kal_uint16 group_idx, kal_int8* group_name_ptr, kal_int32* item_count_ptr)
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

void SP5408MIPI_get_sensor_item_info(kal_uint16 group_idx,kal_uint16 item_idx, MSDK_SENSOR_ITEM_INFO_STRUCT* info_ptr)
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
           	spin_lock(&sp5408_drv_lock);    
            temp_para=SP5408MIPISensorCCT[temp_addr].Para;	
			spin_unlock(&sp5408_drv_lock);
            temp_gain = SP5408MIPIReg2Gain(temp_para);
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
                
                    //temp_reg=SP5408MIPISensorReg[CMMCLK_CURRENT_INDEX].Para;
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
                    info_ptr->ItemValue=SP5408MIPI_MAX_EXPOSURE_LINES;
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
kal_bool SP5408MIPI_set_sensor_item_info(kal_uint16 group_idx, kal_uint16 item_idx, kal_int32 ItemValue)
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
            temp_para = SP5408MIPIGain2Reg(ItemValue);
            spin_lock(&sp5408_drv_lock);    
            SP5408MIPISensorCCT[temp_addr].Para = temp_para;
			spin_unlock(&sp5408_drv_lock);
		

		SP5408MIPI_write_cmos_sensor(0xfd, 0x01);

		SP5408MIPI_write_cmos_sensor(SP5408MIPISensorCCT[temp_addr].Addr,temp_para);//hanlei?
		
			temp_para=read_SP5408MIPI_gain();	
            spin_lock(&sp5408_drv_lock);    
            SP5408MIPI_sensor_gain_base=temp_para;
			spin_unlock(&sp5408_drv_lock);

            break;
        case CMMCLK_CURRENT:
            switch (item_idx)
            {
                case 0:
                    if(ItemValue==2)
                    {			
                    spin_lock(&sp5408_drv_lock);    
                        SP5408MIPISensorReg[CMMCLK_CURRENT_INDEX].Para = ISP_DRIVING_2MA;
					spin_unlock(&sp5408_drv_lock);
                        //SP5408MIPI_set_isp_driving_current(ISP_DRIVING_2MA);
                    }
                    else if(ItemValue==3 || ItemValue==4)
                    {
                    	spin_lock(&sp5408_drv_lock);    
                        SP5408MIPISensorReg[CMMCLK_CURRENT_INDEX].Para = ISP_DRIVING_4MA;
						spin_unlock(&sp5408_drv_lock);
                        //SP5408MIPI_set_isp_driving_current(ISP_DRIVING_4MA);
                    }
                    else if(ItemValue==5 || ItemValue==6)
                    {
                    	spin_lock(&sp5408_drv_lock);    
                        SP5408MIPISensorReg[CMMCLK_CURRENT_INDEX].Para = ISP_DRIVING_6MA;
						spin_unlock(&sp5408_drv_lock);
                        //SP5408MIPI_set_isp_driving_current(ISP_DRIVING_6MA);
                    }
                    else
                    {
                    	spin_lock(&sp5408_drv_lock);    
                        SP5408MIPISensorReg[CMMCLK_CURRENT_INDEX].Para = ISP_DRIVING_8MA;
						spin_unlock(&sp5408_drv_lock);
                        //SP5408MIPI_set_isp_driving_current(ISP_DRIVING_8MA);
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
					spin_lock(&sp5408_drv_lock);    
                    SP5408MIPI_FAC_SENSOR_REG=ItemValue;
					spin_unlock(&sp5408_drv_lock);
                    break;
                case 1:
                    SP5408MIPI_write_cmos_sensor(SP5408MIPI_FAC_SENSOR_REG,ItemValue);
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
static void SP5408MIPI_SetDummy(const kal_uint16 iPixels, const kal_uint16 iLines)
{
	kal_uint32 frame_length = 0, line_length = 0;
    if(SP5408MIPI_sensor.pv_mode == KAL_TRUE)
   	{
   	 spin_lock(&sp5408_drv_lock);    
   	 SP5408MIPI_sensor.pv_dummy_pixels = iPixels;
	 SP5408MIPI_sensor.pv_dummy_lines = iLines;
   	 SP5408MIPI_sensor.pv_line_length = SP5408MIPI_PV_LINE_LENGTH_PIXELS + iPixels;
	 SP5408MIPI_sensor.pv_frame_length = SP5408MIPI_PV_FRAME_LENGTH_LINES + iLines;
	 spin_unlock(&sp5408_drv_lock);
	 line_length = SP5408MIPI_sensor.pv_line_length;
	 frame_length = SP5408MIPI_sensor.pv_frame_length;	 	
	//hanlei
	SP5408MIPI_write_cmos_sensor(0xfd, 0x01);
	SP5408MIPI_write_cmos_sensor(0x05, (SP5408MIPI_sensor.pv_dummy_lines>>8) & 0xFF);
	SP5408MIPI_write_cmos_sensor(0x06, SP5408MIPI_sensor.pv_dummy_lines & 0xFF);	
	SP5408MIPI_write_cmos_sensor(0x07, (SP5408MIPI_sensor.pv_dummy_lines >>8) & 0xFF);
	SP5408MIPI_write_cmos_sensor(0x08, SP5408MIPI_sensor.pv_dummy_lines & 0xFF);	
	SP5408MIPI_write_cmos_sensor(0x09, (SP5408MIPI_sensor.pv_dummy_pixels >>8) & 0x0F);
	SP5408MIPI_write_cmos_sensor(0x0A, SP5408MIPI_sensor.pv_dummy_pixels & 0xFF);	
	SP5408MIPI_write_cmos_sensor(0x01, 0x01);	
   	}
   else if(SP5408MIPI_sensor.video_mode== KAL_TRUE)
   	{
   	 spin_lock(&sp5408_drv_lock);    
   	 SP5408MIPI_sensor.video_dummy_pixels = iPixels;
	 SP5408MIPI_sensor.video_dummy_lines = iLines;
   	 SP5408MIPI_sensor.video_line_length = SP5408MIPI_VIDEO_LINE_LENGTH_PIXELS + iPixels;
	 SP5408MIPI_sensor.video_frame_length = SP5408MIPI_VIDEO_FRAME_LENGTH_LINES + iLines;
	 spin_unlock(&sp5408_drv_lock);
	 line_length = SP5408MIPI_sensor.video_line_length;
	 frame_length = SP5408MIPI_sensor.video_frame_length;
	//hanlei
	SP5408MIPI_write_cmos_sensor(0xfd, 0x01);
	SP5408MIPI_write_cmos_sensor(0x05, (SP5408MIPI_sensor.video_dummy_lines>>8) & 0xFF);
	SP5408MIPI_write_cmos_sensor(0x06, SP5408MIPI_sensor.video_dummy_lines & 0xFF);	
	SP5408MIPI_write_cmos_sensor(0x07, (SP5408MIPI_sensor.video_dummy_lines >>8) & 0xFF);
	SP5408MIPI_write_cmos_sensor(0x08, SP5408MIPI_sensor.video_dummy_lines & 0xFF);	
	SP5408MIPI_write_cmos_sensor(0x09, (SP5408MIPI_sensor.video_dummy_pixels >>8) & 0x0F);
	SP5408MIPI_write_cmos_sensor(0x0A, SP5408MIPI_sensor.video_dummy_pixels & 0xFF);	 
	SP5408MIPI_write_cmos_sensor(0x01, 0x01);		
   	}
	else if(SP5408MIPI_sensor.capture_mode== KAL_TRUE) 
		{
	  spin_lock(&sp5408_drv_lock);	
   	  SP5408MIPI_sensor.cp_dummy_pixels = iPixels;
	  SP5408MIPI_sensor.cp_dummy_lines = iLines;
	  SP5408MIPI_sensor.cp_line_length = SP5408MIPI_FULL_LINE_LENGTH_PIXELS + iPixels;
	  SP5408MIPI_sensor.cp_frame_length = SP5408MIPI_FULL_FRAME_LENGTH_LINES + iLines;
	   spin_unlock(&sp5408_drv_lock);
	  line_length = SP5408MIPI_sensor.cp_line_length;
	  frame_length = SP5408MIPI_sensor.cp_frame_length;
	//hanlei
	SP5408MIPI_write_cmos_sensor(0xfd, 0x01);
	SP5408MIPI_write_cmos_sensor(0x05, (SP5408MIPI_sensor.cp_dummy_lines>>8) & 0xFF);
	SP5408MIPI_write_cmos_sensor(0x06, SP5408MIPI_sensor.cp_dummy_lines & 0xFF);	
	SP5408MIPI_write_cmos_sensor(0x07, (SP5408MIPI_sensor.cp_dummy_lines >>8) & 0xFF);
	SP5408MIPI_write_cmos_sensor(0x08, SP5408MIPI_sensor.cp_dummy_lines & 0xFF);	
	SP5408MIPI_write_cmos_sensor(0x09, (SP5408MIPI_sensor.cp_dummy_pixels >>8) & 0x0F);
	SP5408MIPI_write_cmos_sensor(0x0A, SP5408MIPI_sensor.cp_dummy_pixels & 0xFF);	
	SP5408MIPI_write_cmos_sensor(0x01, 0x01);		
    }
	else
	{
	 SENSORDB("[SP5408MIPI]%s(),sensor mode error",__FUNCTION__);
	}

	  line_length = iPixels;
	  frame_length = iLines;

	/*
	//hanlei
	SP5408MIPI_write_cmos_sensor(0xfd, 0x01);
	SP5408MIPI_write_cmos_sensor(0x05, (frame_length >>8) & 0xFF);	//v blank  indoor
	SP5408MIPI_write_cmos_sensor(0x06, frame_length & 0xFF);	
	SP5408MIPI_write_cmos_sensor(0x07, (frame_length >>8) & 0xFF);	//delay outdoor 
	SP5408MIPI_write_cmos_sensor(0x08, frame_length & 0xFF);	
	
	SP5408MIPI_write_cmos_sensor(0x09, (line_length >>8) & 0x0F);
	SP5408MIPI_write_cmos_sensor(0x0A, line_length & 0xFF);
    */

}   /*  SP5408MIPI_SetDummy */
static void SP5408MIPI_Sensor_Init(void)
{
#ifdef DEBUG_SENSOR
		if(fromsd == 1)//是否从SD读取//gepeiwei   120903
		SP5408MIPI_Initialize_from_T_Flash();//从SD卡读取的主要函数

		else
#endif

//init setting

{
#if 0 //from mtk
//1296x972
SP5408MIPI_write_cmos_sensor(0xfd,0x00);
SP5408MIPI_write_cmos_sensor(0x10,0x04);
SP5408MIPI_write_cmos_sensor(0x11,0x10);
SP5408MIPI_write_cmos_sensor(0x12,0x04);
SP5408MIPI_write_cmos_sensor(0x21,0x00);// 3 //hanlei
SP5408MIPI_write_cmos_sensor(0x20,0xcf);// 00//hanlei
SP5408MIPI_write_cmos_sensor(0x0e,0x10);
SP5408MIPI_write_cmos_sensor(0x13,0x21);//hanlei
SP5408MIPI_write_cmos_sensor(0x26,0x00);//55//hanlei

SP5408MIPI_write_cmos_sensor(0xfd,0x05);
SP5408MIPI_write_cmos_sensor(0x00,0x00);//c1//hanlei
SP5408MIPI_write_cmos_sensor(0x02,0x10);
SP5408MIPI_write_cmos_sensor(0x03,0x05);
SP5408MIPI_write_cmos_sensor(0x04,0xcc);
SP5408MIPI_write_cmos_sensor(0x05,0x03);

SP5408MIPI_write_cmos_sensor(0xfd,0x06);
SP5408MIPI_write_cmos_sensor(0xfb,0x00);//95//disable OB 13.9.25//hanlei
SP5408MIPI_write_cmos_sensor(0xc6,0x80); 
SP5408MIPI_write_cmos_sensor(0xc7,0x80); 
SP5408MIPI_write_cmos_sensor(0xc8,0x80); 
SP5408MIPI_write_cmos_sensor(0xc9,0x80);  

SP5408MIPI_write_cmos_sensor(0xfd,0x01);
SP5408MIPI_write_cmos_sensor(0x09,0x02);
SP5408MIPI_write_cmos_sensor(0x0a,0x26);
SP5408MIPI_write_cmos_sensor(0x0b,0x45);
SP5408MIPI_write_cmos_sensor(0x0c,0x33);//add 13.9.25
SP5408MIPI_write_cmos_sensor(0x0d,0x33);//add 13.9.25

SP5408MIPI_write_cmos_sensor(0x12,0x02);
SP5408MIPI_write_cmos_sensor(0x13,0x0f);
SP5408MIPI_write_cmos_sensor(0x15,0x18);
SP5408MIPI_write_cmos_sensor(0x6b,0x12);
SP5408MIPI_write_cmos_sensor(0x6f,0x12);
SP5408MIPI_write_cmos_sensor(0x7a,0x12);
SP5408MIPI_write_cmos_sensor(0x73,0x13);
SP5408MIPI_write_cmos_sensor(0x70,0x12);
SP5408MIPI_write_cmos_sensor(0x71,0x1b);
SP5408MIPI_write_cmos_sensor(0x72,0x14);
SP5408MIPI_write_cmos_sensor(0x74,0x10);
SP5408MIPI_write_cmos_sensor(0x75,0x10);
SP5408MIPI_write_cmos_sensor(0x7f,0x10);
SP5408MIPI_write_cmos_sensor(0x76,0x1e);
SP5408MIPI_write_cmos_sensor(0x77,0x10);
SP5408MIPI_write_cmos_sensor(0x7d,0x04);
SP5408MIPI_write_cmos_sensor(0x6c,0x12);
SP5408MIPI_write_cmos_sensor(0x7e,0x1b);
SP5408MIPI_write_cmos_sensor(0x84,0x14);

SP5408MIPI_write_cmos_sensor(0xfd,0x01);
SP5408MIPI_write_cmos_sensor(0x19,0x03);
SP5408MIPI_write_cmos_sensor(0x31,0x04);
SP5408MIPI_write_cmos_sensor(0x24,0x80);
SP5408MIPI_write_cmos_sensor(0x03,0x07);
SP5408MIPI_write_cmos_sensor(0x04,0x80);
SP5408MIPI_write_cmos_sensor(0x22,0x1e);
SP5408MIPI_write_cmos_sensor(0x01,0x01);
SP5408MIPI_write_cmos_sensor(0x1d,0x04);
SP5408MIPI_write_cmos_sensor(0x20,0xdf);
SP5408MIPI_write_cmos_sensor(0x25,0x8d);
SP5408MIPI_write_cmos_sensor(0x1a,0x43);
SP5408MIPI_write_cmos_sensor(0x1c,0x00);

SP5408MIPI_write_cmos_sensor(0xfd,0x02);
SP5408MIPI_write_cmos_sensor(0x37,0x00);
SP5408MIPI_write_cmos_sensor(0x38,0x04);
SP5408MIPI_write_cmos_sensor(0x3b,0x00);
SP5408MIPI_write_cmos_sensor(0x3c,0x02);
SP5408MIPI_write_cmos_sensor(0x39,0x03);
SP5408MIPI_write_cmos_sensor(0x3a,0xcc);
SP5408MIPI_write_cmos_sensor(0x3d,0x02);
SP5408MIPI_write_cmos_sensor(0x3e,0x88);
SP5408MIPI_write_cmos_sensor(0x33,0x00);//ff//hanlei 
SP5408MIPI_write_cmos_sensor(0x3f,0x01);

SP5408MIPI_write_cmos_sensor(0xfd,0x03);
SP5408MIPI_write_cmos_sensor(0x04,0x11);
SP5408MIPI_write_cmos_sensor(0xaa,0x46);
SP5408MIPI_write_cmos_sensor(0xab,0x01);
SP5408MIPI_write_cmos_sensor(0xac,0xf5);
SP5408MIPI_write_cmos_sensor(0xad,0x00);
#endif

#if 0
//1280x960  2.25x max 10fps //from debug
SP5408MIPI_write_cmos_sensor(0xfd,0x00);
SP5408MIPI_write_cmos_sensor(0x10,0x04);
SP5408MIPI_write_cmos_sensor(0x11,0x10);
SP5408MIPI_write_cmos_sensor(0x12,0x04);
SP5408MIPI_write_cmos_sensor(0x21,0x00);
SP5408MIPI_write_cmos_sensor(0x20,0xcf);//data driver
SP5408MIPI_write_cmos_sensor(0x0e,0x10);
SP5408MIPI_write_cmos_sensor(0x26,0x00);//pclk v h driver 
SP5408MIPI_write_cmos_sensor(0x13,0x21);//pclk 0x21
//SP5408MIPI_write_cmos_sensor(0x15,0x07);//pclk delay

SP5408MIPI_write_cmos_sensor(0xfd,0x06);
SP5408MIPI_write_cmos_sensor(0xfb,0x95);//hanlei
SP5408MIPI_write_cmos_sensor(0xc6,0x80);
SP5408MIPI_write_cmos_sensor(0xc7,0x80);
SP5408MIPI_write_cmos_sensor(0xc8,0x80);
SP5408MIPI_write_cmos_sensor(0xc9,0x80);

SP5408MIPI_write_cmos_sensor(0xfd,0x01);
SP5408MIPI_write_cmos_sensor(0x09,0x02);//02
SP5408MIPI_write_cmos_sensor(0x0a,0x16);//6c 
SP5408MIPI_write_cmos_sensor(0x0b,0x02);//0x45 hanlei14.3.13
SP5408MIPI_write_cmos_sensor(0x0c,0x33);
SP5408MIPI_write_cmos_sensor(0x0d,0x33);
SP5408MIPI_write_cmos_sensor(0x1c,0x00);//0x01

SP5408MIPI_write_cmos_sensor(0x0f,0x7f);
SP5408MIPI_write_cmos_sensor(0x10,0x7e);
SP5408MIPI_write_cmos_sensor(0x11,0x00);
SP5408MIPI_write_cmos_sensor(0x12,0x00);
SP5408MIPI_write_cmos_sensor(0x13,0x2f);
SP5408MIPI_write_cmos_sensor(0x16,0x00);
SP5408MIPI_write_cmos_sensor(0x17,0x2f);
SP5408MIPI_write_cmos_sensor(0x15,0x38);//18
SP5408MIPI_write_cmos_sensor(0x6b,0x32);
SP5408MIPI_write_cmos_sensor(0x6c,0x32);//60
SP5408MIPI_write_cmos_sensor(0x14,0x00);//24
SP5408MIPI_write_cmos_sensor(0x6f,0x32);//62
SP5408MIPI_write_cmos_sensor(0x70,0x40);
SP5408MIPI_write_cmos_sensor(0x71,0x40);//20
SP5408MIPI_write_cmos_sensor(0x72,0x40);
SP5408MIPI_write_cmos_sensor(0x73,0x33);//00
SP5408MIPI_write_cmos_sensor(0x74,0x38);//a0
SP5408MIPI_write_cmos_sensor(0x75,0x38);//80
SP5408MIPI_write_cmos_sensor(0x76,0x44);//38
SP5408MIPI_write_cmos_sensor(0x77,0x38);//40
SP5408MIPI_write_cmos_sensor(0x7a,0x40);//42
SP5408MIPI_write_cmos_sensor(0x7e,0x40);//20
SP5408MIPI_write_cmos_sensor(0x84,0x40);

SP5408MIPI_write_cmos_sensor(0xfd,0x01);
SP5408MIPI_write_cmos_sensor(0x19,0x03);
SP5408MIPI_write_cmos_sensor(0x31,0x04);
SP5408MIPI_write_cmos_sensor(0x24,0x80);
SP5408MIPI_write_cmos_sensor(0x03,0x05);
SP5408MIPI_write_cmos_sensor(0x04,0x78);
SP5408MIPI_write_cmos_sensor(0x22,0x19);
SP5408MIPI_write_cmos_sensor(0x1d,0x04);
SP5408MIPI_write_cmos_sensor(0x20,0xdf);
SP5408MIPI_write_cmos_sensor(0x25,0x8d);
SP5408MIPI_write_cmos_sensor(0x27,0xa9);
SP5408MIPI_write_cmos_sensor(0x1a,0x43);
SP5408MIPI_write_cmos_sensor(0x28,0x00);
SP5408MIPI_write_cmos_sensor(0x40,0x05);
SP5408MIPI_write_cmos_sensor(0x3f,0x00);//00 mirror
SP5408MIPI_write_cmos_sensor(0x01,0x01);
	//error
SP5408MIPI_write_cmos_sensor(0xfd,0x02);
SP5408MIPI_write_cmos_sensor(0x37,0x00);
SP5408MIPI_write_cmos_sensor(0x38,0x04);
SP5408MIPI_write_cmos_sensor(0x3b,0x00);
SP5408MIPI_write_cmos_sensor(0x3c,0x02);
SP5408MIPI_write_cmos_sensor(0x39,0x03);
SP5408MIPI_write_cmos_sensor(0x3a,0xcc);
SP5408MIPI_write_cmos_sensor(0x3d,0x02);
SP5408MIPI_write_cmos_sensor(0x3e,0x88);
SP5408MIPI_write_cmos_sensor(0x33,0x0f);//hanlei
SP5408MIPI_write_cmos_sensor(0x3f,0x01);
SP5408MIPI_write_cmos_sensor(0xfd,0x03);
SP5408MIPI_write_cmos_sensor(0x04,0x11);
SP5408MIPI_write_cmos_sensor(0xaa,0x46);
SP5408MIPI_write_cmos_sensor(0xab,0x01);
SP5408MIPI_write_cmos_sensor(0xac,0xf5);
SP5408MIPI_write_cmos_sensor(0xad,0x00);
#endif

#if 0
//1296x972 //from qc
SP5408MIPI_write_cmos_sensor(0xfd,0x00);
SP5408MIPI_write_cmos_sensor(0x10,0x04);
SP5408MIPI_write_cmos_sensor(0x11,0x10);
SP5408MIPI_write_cmos_sensor(0x12,0x04);
SP5408MIPI_write_cmos_sensor(0x13,0x21);//hanlei
SP5408MIPI_write_cmos_sensor(0x21,0x00);//3//hanlei
SP5408MIPI_write_cmos_sensor(0x20,0x4f);//3f//hanlei
SP5408MIPI_write_cmos_sensor(0x0e,0x10);//0x00
SP5408MIPI_write_cmos_sensor(0x26,0x55);//55//hanlei

SP5408MIPI_write_cmos_sensor(0xfd,0x05);
SP5408MIPI_write_cmos_sensor(0x00,0x00);//hanlei
SP5408MIPI_write_cmos_sensor(0x02,0x10);
SP5408MIPI_write_cmos_sensor(0x03,0x05);
SP5408MIPI_write_cmos_sensor(0x04,0xcc);
SP5408MIPI_write_cmos_sensor(0x05,0x03);

SP5408MIPI_write_cmos_sensor(0xfd,0x06);
SP5408MIPI_write_cmos_sensor(0xfb,0x00);//95:en OB 00:dis OB//hanlei
SP5408MIPI_write_cmos_sensor(0xc6,0x80); 
SP5408MIPI_write_cmos_sensor(0xc7,0x80); 
SP5408MIPI_write_cmos_sensor(0xc8,0x80); 
SP5408MIPI_write_cmos_sensor(0xc9,0x80);  

SP5408MIPI_write_cmos_sensor(0xfd,0x01);
SP5408MIPI_write_cmos_sensor(0x09,0x02);//
SP5408MIPI_write_cmos_sensor(0x0a,0x26);//
SP5408MIPI_write_cmos_sensor(0x0b,0x45);//

SP5408MIPI_write_cmos_sensor(0x1e,0x00);//low power
SP5408MIPI_write_cmos_sensor(0x2e,0x08);
SP5408MIPI_write_cmos_sensor(0x0c,0x33);//add 13.9.25
SP5408MIPI_write_cmos_sensor(0x0d,0x33);//add 13.9.25
SP5408MIPI_write_cmos_sensor(0x40,0x00);//05:en digital vfpn//00:dis digital vfpn//halei
SP5408MIPI_write_cmos_sensor(0x19,0x03);
SP5408MIPI_write_cmos_sensor(0x31,0x04);
SP5408MIPI_write_cmos_sensor(0x24,0x80);
SP5408MIPI_write_cmos_sensor(0x03,0x07);
SP5408MIPI_write_cmos_sensor(0x04,0x80);
SP5408MIPI_write_cmos_sensor(0x22,0x1e);
SP5408MIPI_write_cmos_sensor(0x01,0x01);
SP5408MIPI_write_cmos_sensor(0x1d,0x04);
SP5408MIPI_write_cmos_sensor(0x20,0x5f);//df:en anolog vfpn//5f:dis anolog vfpn//hanlei
SP5408MIPI_write_cmos_sensor(0x25,0x8d);
SP5408MIPI_write_cmos_sensor(0x27,0xa9);
SP5408MIPI_write_cmos_sensor(0x28,0x00);

SP5408MIPI_write_cmos_sensor(0x1c,0x00);
SP5408MIPI_write_cmos_sensor(0x3f,0x00);//0x03 mirror

//pixel timing    
SP5408MIPI_write_cmos_sensor(0x1a,0x43);
SP5408MIPI_write_cmos_sensor(0x0f,0x7f);
SP5408MIPI_write_cmos_sensor(0x10,0x7e);
SP5408MIPI_write_cmos_sensor(0x11,0x00);
SP5408MIPI_write_cmos_sensor(0x12,0x00);
SP5408MIPI_write_cmos_sensor(0x13,0x2f);
SP5408MIPI_write_cmos_sensor(0x16,0x00);
SP5408MIPI_write_cmos_sensor(0x17,0x2f);
SP5408MIPI_write_cmos_sensor(0x14,0x00);
SP5408MIPI_write_cmos_sensor(0x15,0x38);
SP5408MIPI_write_cmos_sensor(0x6b,0x32);
SP5408MIPI_write_cmos_sensor(0x6c,0x32);    
SP5408MIPI_write_cmos_sensor(0x6f,0x32);
SP5408MIPI_write_cmos_sensor(0x70,0x40);
SP5408MIPI_write_cmos_sensor(0x71,0x40);
SP5408MIPI_write_cmos_sensor(0x72,0x40);
SP5408MIPI_write_cmos_sensor(0x73,0x33);
SP5408MIPI_write_cmos_sensor(0x74,0x38);
SP5408MIPI_write_cmos_sensor(0x75,0x38);
SP5408MIPI_write_cmos_sensor(0x76,0x44);
SP5408MIPI_write_cmos_sensor(0x77,0x38);
SP5408MIPI_write_cmos_sensor(0x7a,0x40);
SP5408MIPI_write_cmos_sensor(0x7e,0x40);
SP5408MIPI_write_cmos_sensor(0x84,0x40);
SP5408MIPI_write_cmos_sensor(0x7d,0x08);//f//hanlei14.3.20

SP5408MIPI_write_cmos_sensor(0xfd,0x02);
SP5408MIPI_write_cmos_sensor(0x37,0x00);
SP5408MIPI_write_cmos_sensor(0x38,0x04);
SP5408MIPI_write_cmos_sensor(0x3b,0x00);
SP5408MIPI_write_cmos_sensor(0x3c,0x02);
SP5408MIPI_write_cmos_sensor(0x39,0x03);
SP5408MIPI_write_cmos_sensor(0x3a,0xcc);
SP5408MIPI_write_cmos_sensor(0x3d,0x02);
SP5408MIPI_write_cmos_sensor(0x3e,0x88);
SP5408MIPI_write_cmos_sensor(0x33,0x0f);//close shading oxff//0f//hanlei
SP5408MIPI_write_cmos_sensor(0x3f,0x01);

SP5408MIPI_write_cmos_sensor(0xfd,0x03);
SP5408MIPI_write_cmos_sensor(0x04,0x11);
SP5408MIPI_write_cmos_sensor(0xaa,0x46);
SP5408MIPI_write_cmos_sensor(0xab,0x01);
SP5408MIPI_write_cmos_sensor(0xac,0xf5);
SP5408MIPI_write_cmos_sensor(0xad,0x00);
#endif

#if 1 
//from spreadtrum
/*
SP5408MIPI_write_cmos_sensor(0xfd,0x00);
SP5408MIPI_write_cmos_sensor(0x10,0x04);//1b//04
SP5408MIPI_write_cmos_sensor(0x11,0x10);
SP5408MIPI_write_cmos_sensor(0x12,0x04);// 14
SP5408MIPI_write_cmos_sensor(0x13,0x01);//hanlei
SP5408MIPI_write_cmos_sensor(0x15,0x00);//+
SP5408MIPI_write_cmos_sensor(0x21,0x03);//00//hanlei
SP5408MIPI_write_cmos_sensor(0x20,0x7f);//00//4f//hanlei
SP5408MIPI_write_cmos_sensor(0x0e,0x00);//50//hanlei
SP5408MIPI_write_cmos_sensor(0x26,0x55);//55//hanlei

SP5408MIPI_write_cmos_sensor(0xfd,0x05);
SP5408MIPI_write_cmos_sensor(0x00,0xc1);//00//hanlei
SP5408MIPI_write_cmos_sensor(0x02,0x10);
SP5408MIPI_write_cmos_sensor(0x03,0x05);
SP5408MIPI_write_cmos_sensor(0x04,0xcc);
SP5408MIPI_write_cmos_sensor(0x05,0x03);

SP5408MIPI_write_cmos_sensor(0xfd,0x06);
SP5408MIPI_write_cmos_sensor(0xfb,0x00);//95//hanlei
SP5408MIPI_write_cmos_sensor(0xc6,0x80);
SP5408MIPI_write_cmos_sensor(0xc7,0x80);
SP5408MIPI_write_cmos_sensor(0xc8,0x80);
SP5408MIPI_write_cmos_sensor(0xc9,0x80);
SP5408MIPI_write_cmos_sensor(0xfd,0x01);
SP5408MIPI_write_cmos_sensor(0x09,0x01);//2//hanlei
SP5408MIPI_write_cmos_sensor(0x0a,0x3e);//26//hanlei//25us max19.99fps
SP5408MIPI_write_cmos_sensor(0x0b,0x45);//
SP5408MIPI_write_cmos_sensor(0x0c,0x33);//33
SP5408MIPI_write_cmos_sensor(0x0d,0x33);//33//[4:3] en sunspot
SP5408MIPI_write_cmos_sensor(0x21,0x1f);//[6:0]//1f//sunspot range hanlei
SP5408MIPI_write_cmos_sensor(0x29,0x03);//sunspot replace
SP5408MIPI_write_cmos_sensor(0x2a,0xff);//sunspot replace



SP5408MIPI_write_cmos_sensor(0x0f,0x7f);
SP5408MIPI_write_cmos_sensor(0x10,0x7e);
SP5408MIPI_write_cmos_sensor(0x11,0x00);
SP5408MIPI_write_cmos_sensor(0x12,0x00);
SP5408MIPI_write_cmos_sensor(0x13,0x2f);
SP5408MIPI_write_cmos_sensor(0x16,0x00);
SP5408MIPI_write_cmos_sensor(0x17,0x2f);
SP5408MIPI_write_cmos_sensor(0x15,0x18);
SP5408MIPI_write_cmos_sensor(0x6b,0x32);
SP5408MIPI_write_cmos_sensor(0x6c,0x60);
SP5408MIPI_write_cmos_sensor(0x14,0x24);
SP5408MIPI_write_cmos_sensor(0x6f,0x62);
SP5408MIPI_write_cmos_sensor(0x70,0x40);
SP5408MIPI_write_cmos_sensor(0x71,0x20);
SP5408MIPI_write_cmos_sensor(0x72,0x40);
SP5408MIPI_write_cmos_sensor(0x73,0x00);
SP5408MIPI_write_cmos_sensor(0x74,0xa0);
SP5408MIPI_write_cmos_sensor(0x75,0x80);
SP5408MIPI_write_cmos_sensor(0x76,0x38);
SP5408MIPI_write_cmos_sensor(0x77,0x40);
SP5408MIPI_write_cmos_sensor(0x7a,0x42);
SP5408MIPI_write_cmos_sensor(0x7e,0x20);
SP5408MIPI_write_cmos_sensor(0x84,0x40);

//pixel timing 
SP5408MIPI_write_cmos_sensor(0xfd,0x01);

SP5408MIPI_write_cmos_sensor(0x0f,0x7f);
SP5408MIPI_write_cmos_sensor(0x10,0x7e);
SP5408MIPI_write_cmos_sensor(0x11,0x00);
SP5408MIPI_write_cmos_sensor(0x12,0x00);
SP5408MIPI_write_cmos_sensor(0x13,0x2f);
SP5408MIPI_write_cmos_sensor(0x16,0x00);
SP5408MIPI_write_cmos_sensor(0x17,0x2f);
SP5408MIPI_write_cmos_sensor(0x14,0x00);
SP5408MIPI_write_cmos_sensor(0x15,0x38);
SP5408MIPI_write_cmos_sensor(0x6b,0x32);
SP5408MIPI_write_cmos_sensor(0x6c,0x32);//52 //32  //58 //60
SP5408MIPI_write_cmos_sensor(0x6f,0x32);
SP5408MIPI_write_cmos_sensor(0x70,0x40);
SP5408MIPI_write_cmos_sensor(0x71,0x40);
SP5408MIPI_write_cmos_sensor(0x72,0x40);
SP5408MIPI_write_cmos_sensor(0x73,0x33);
SP5408MIPI_write_cmos_sensor(0x74,0x38);
SP5408MIPI_write_cmos_sensor(0x75,0x38);//
SP5408MIPI_write_cmos_sensor(0x76,0x44);
SP5408MIPI_write_cmos_sensor(0x77,0x30);//38//hanlei
SP5408MIPI_write_cmos_sensor(0x7a,0x40);//20//40
SP5408MIPI_write_cmos_sensor(0x7e,0x40);//60//40//70
SP5408MIPI_write_cmos_sensor(0x84,0x30);//20//40
SP5408MIPI_write_cmos_sensor(0x7d,0x0f);//f//8hanlei14.3.20

SP5408MIPI_write_cmos_sensor(0xfd,0x01);
SP5408MIPI_write_cmos_sensor(0x19,0x03);//
SP5408MIPI_write_cmos_sensor(0x31,0x04);//
SP5408MIPI_write_cmos_sensor(0x24,0x80);
SP5408MIPI_write_cmos_sensor(0x03,0x0e);
SP5408MIPI_write_cmos_sensor(0x04,0xec);
SP5408MIPI_write_cmos_sensor(0x22,0x18);//1e//hanlei//09vfpn
SP5408MIPI_write_cmos_sensor(0x2e,0x08);//+
SP5408MIPI_write_cmos_sensor(0x1e,0x01);//+//3//[0]digital sunspot en
SP5408MIPI_write_cmos_sensor(0x1d,0x04);
SP5408MIPI_write_cmos_sensor(0x20,0xdf);//df//5f//hanlei
SP5408MIPI_write_cmos_sensor(0x25,0x8d);
SP5408MIPI_write_cmos_sensor(0x27,0xa9);//f9//a9
SP5408MIPI_write_cmos_sensor(0x1a,0x43);//41//hanlei
SP5408MIPI_write_cmos_sensor(0x28,0x00);
SP5408MIPI_write_cmos_sensor(0x40,0x05);//05//00hanlei
SP5408MIPI_write_cmos_sensor(0x3f,0x00);
SP5408MIPI_write_cmos_sensor(0x01,0x01);
SP5408MIPI_write_cmos_sensor(0xfd,0x02);
SP5408MIPI_write_cmos_sensor(0x37,0x00);
SP5408MIPI_write_cmos_sensor(0x38,0x04);
SP5408MIPI_write_cmos_sensor(0x3b,0x00);
SP5408MIPI_write_cmos_sensor(0x3c,0x02);
SP5408MIPI_write_cmos_sensor(0x39,0x03);
SP5408MIPI_write_cmos_sensor(0x3a,0xcc);
SP5408MIPI_write_cmos_sensor(0x3d,0x02);
SP5408MIPI_write_cmos_sensor(0x3e,0x88);
SP5408MIPI_write_cmos_sensor(0x33,0x0f);
SP5408MIPI_write_cmos_sensor(0x3f,0x01);
SP5408MIPI_write_cmos_sensor(0xfd,0x03);
SP5408MIPI_write_cmos_sensor(0x04,0x11);
SP5408MIPI_write_cmos_sensor(0xaa,0x46);
SP5408MIPI_write_cmos_sensor(0xab,0x01);
SP5408MIPI_write_cmos_sensor(0xac,0xf5);
SP5408MIPI_write_cmos_sensor(0xad,0x00);*/
SP5408MIPI_write_cmos_sensor(0xfd,0x00);

SP5408MIPI_write_cmos_sensor(0x10,0x04);
SP5408MIPI_write_cmos_sensor(0x11,0x10);
SP5408MIPI_write_cmos_sensor(0x12,0x04);
SP5408MIPI_write_cmos_sensor(0x13,0x01);
SP5408MIPI_write_cmos_sensor(0x15,0x00);
SP5408MIPI_write_cmos_sensor(0x21,0x03);
SP5408MIPI_write_cmos_sensor(0x20,0x3f);
SP5408MIPI_write_cmos_sensor(0x0e,0x10);
SP5408MIPI_write_cmos_sensor(0x26,0x55);

SP5408MIPI_write_cmos_sensor(0xfd,0x05);
SP5408MIPI_write_cmos_sensor(0x00,0xc1);
SP5408MIPI_write_cmos_sensor(0x02,0x10);
SP5408MIPI_write_cmos_sensor(0x03,0x05);
SP5408MIPI_write_cmos_sensor(0x04,0xcc);
SP5408MIPI_write_cmos_sensor(0x05,0x03);
SP5408MIPI_write_cmos_sensor(0xfd,0x06);
SP5408MIPI_write_cmos_sensor(0xfb,0x00);
SP5408MIPI_write_cmos_sensor(0xc6,0x80);
SP5408MIPI_write_cmos_sensor(0xc7,0x80);
SP5408MIPI_write_cmos_sensor(0xc8,0x80);
SP5408MIPI_write_cmos_sensor(0xc9,0x80);

SP5408MIPI_write_cmos_sensor(0xfd,0x01);
SP5408MIPI_write_cmos_sensor(0x09,0x00);
SP5408MIPI_write_cmos_sensor(0x0a,0x83);
SP5408MIPI_write_cmos_sensor(0x0b,0x02);
SP5408MIPI_write_cmos_sensor(0x0c,0x33);
SP5408MIPI_write_cmos_sensor(0x0d,0x33);
SP5408MIPI_write_cmos_sensor(0x21,0x1f);
SP5408MIPI_write_cmos_sensor(0x29,0x03);
SP5408MIPI_write_cmos_sensor(0x2a,0xff);
SP5408MIPI_write_cmos_sensor(0xfd,0x01);
SP5408MIPI_write_cmos_sensor(0x0f,0x7f);
SP5408MIPI_write_cmos_sensor(0x10,0x7e);
SP5408MIPI_write_cmos_sensor(0x11,0x00);
SP5408MIPI_write_cmos_sensor(0x12,0x00);
SP5408MIPI_write_cmos_sensor(0x13,0x2f);
SP5408MIPI_write_cmos_sensor(0x16,0x00);
SP5408MIPI_write_cmos_sensor(0x17,0x2f);
SP5408MIPI_write_cmos_sensor(0x14,0x00);
SP5408MIPI_write_cmos_sensor(0x15,0x38);
SP5408MIPI_write_cmos_sensor(0x6b,0x32);
SP5408MIPI_write_cmos_sensor(0x6c,0x32);
SP5408MIPI_write_cmos_sensor(0x6f,0x32);
SP5408MIPI_write_cmos_sensor(0x70,0x40);
SP5408MIPI_write_cmos_sensor(0x71,0x40);
SP5408MIPI_write_cmos_sensor(0x72,0x40);
SP5408MIPI_write_cmos_sensor(0x73,0x33);
SP5408MIPI_write_cmos_sensor(0x74,0x38);
SP5408MIPI_write_cmos_sensor(0x75,0x38);
SP5408MIPI_write_cmos_sensor(0x76,0x44);
SP5408MIPI_write_cmos_sensor(0x77,0x38);
SP5408MIPI_write_cmos_sensor(0x7a,0x40);
SP5408MIPI_write_cmos_sensor(0x7e,0x40);
SP5408MIPI_write_cmos_sensor(0x84,0x40);
SP5408MIPI_write_cmos_sensor(0x7d,0x0f);

SP5408MIPI_write_cmos_sensor(0xfd,0x01);
SP5408MIPI_write_cmos_sensor(0x19,0x03);
SP5408MIPI_write_cmos_sensor(0x31,0x04);
SP5408MIPI_write_cmos_sensor(0x24,0x80);
SP5408MIPI_write_cmos_sensor(0x03,0x0e);
SP5408MIPI_write_cmos_sensor(0x04,0xec);
SP5408MIPI_write_cmos_sensor(0x22,0x1a);
SP5408MIPI_write_cmos_sensor(0x2e,0x08);
SP5408MIPI_write_cmos_sensor(0x1e,0x01);
SP5408MIPI_write_cmos_sensor(0x1d,0x04);
SP5408MIPI_write_cmos_sensor(0x20,0xdf);
SP5408MIPI_write_cmos_sensor(0x25,0x8d);
SP5408MIPI_write_cmos_sensor(0x27,0xa9);
SP5408MIPI_write_cmos_sensor(0x1a,0x43);
SP5408MIPI_write_cmos_sensor(0x28,0x00);
SP5408MIPI_write_cmos_sensor(0x40,0x05);
SP5408MIPI_write_cmos_sensor(0x3f,0x00);//02
SP5408MIPI_write_cmos_sensor(0x01,0x01);

SP5408MIPI_write_cmos_sensor(0xfd,0x02);
SP5408MIPI_write_cmos_sensor(0x37,0x00);
SP5408MIPI_write_cmos_sensor(0x38,0x04);
SP5408MIPI_write_cmos_sensor(0x3b,0x00);
SP5408MIPI_write_cmos_sensor(0x3c,0x02);
SP5408MIPI_write_cmos_sensor(0x39,0x03);
SP5408MIPI_write_cmos_sensor(0x3a,0xcc);
SP5408MIPI_write_cmos_sensor(0x3d,0x02);
SP5408MIPI_write_cmos_sensor(0x3e,0x88);
SP5408MIPI_write_cmos_sensor(0x33,0x0f);
SP5408MIPI_write_cmos_sensor(0x3f,0x01);
SP5408MIPI_write_cmos_sensor(0xfd,0x03);
//SP5408MIPI_write_cmos_sensor(0x04,0x11);
SP5408MIPI_write_cmos_sensor(0xaa,0x46);
SP5408MIPI_write_cmos_sensor(0xab,0x01);
SP5408MIPI_write_cmos_sensor(0xac,0xf5);
SP5408MIPI_write_cmos_sensor(0xad,0x00);
//SP5408MIPI_write_cmos_sensor(0xff,0xff);
#endif
}



	
	// The register only need to enable 1 time.    
	spin_lock(&sp5408_drv_lock);  
	SP5408MIPI_Auto_Flicker_mode = KAL_FALSE;	  // reset the flicker status	 
	spin_unlock(&sp5408_drv_lock);
	SENSORDB("[SP5408MIPI]exit SP5408MIPI_Sensor_Init function\n");


}   /*  SP5408MIPI_Sensor_Init  */
void VideoFullSizeSetting(void)//16:9   6M
{
	SENSORDB("[SP5408MIPI]enter VideoFullSizeSetting function\n");

#ifdef SP5408MIPI_USE_OTP
        if(ret == 0)
        {
	    SENSORDB("[SP5408MIPI_USE_OTP]VideoFullSizeSetting function,sp5408_update_awb_gain\n");
	    sp5408_update_awb_gain();
        }
#endif
	SENSORDB("[SP5408MIPI]exit VideoFullSizeSetting function\n");
}
void PreviewSetting(void)
{
//preview setting 

//preview
{
/*SP5408MIPI_write_cmos_sensor(0xfd,0x01);
SP5408MIPI_write_cmos_sensor(0x0f,0x7f);
SP5408MIPI_write_cmos_sensor(0x10,0x7e);
SP5408MIPI_write_cmos_sensor(0x11,0x00);
SP5408MIPI_write_cmos_sensor(0x12,0x00);
SP5408MIPI_write_cmos_sensor(0x13,0x2f);
SP5408MIPI_write_cmos_sensor(0x16,0x00);
SP5408MIPI_write_cmos_sensor(0x17,0x2f);
SP5408MIPI_write_cmos_sensor(0x15,0x18);
SP5408MIPI_write_cmos_sensor(0x6b,0x32);
SP5408MIPI_write_cmos_sensor(0x6c,0x60);
SP5408MIPI_write_cmos_sensor(0x14,0x24);
SP5408MIPI_write_cmos_sensor(0x6f,0x62);
SP5408MIPI_write_cmos_sensor(0x70,0x40);
SP5408MIPI_write_cmos_sensor(0x71,0x20);
SP5408MIPI_write_cmos_sensor(0x72,0x40);
SP5408MIPI_write_cmos_sensor(0x73,0x00);
SP5408MIPI_write_cmos_sensor(0x74,0xa0);
SP5408MIPI_write_cmos_sensor(0x75,0x80);
SP5408MIPI_write_cmos_sensor(0x76,0x38);
SP5408MIPI_write_cmos_sensor(0x77,0x40);
SP5408MIPI_write_cmos_sensor(0x7a,0x42);
SP5408MIPI_write_cmos_sensor(0x7e,0x20);
SP5408MIPI_write_cmos_sensor(0x84,0x40);*/

//1296x972
SP5408MIPI_write_cmos_sensor(0xfd,0x00);
SP5408MIPI_write_cmos_sensor(0x10,0x04);//48MHz Pclk//max 11.9820fps//one  line time:41.6666us
SP5408MIPI_write_cmos_sensor(0x11,0x10);

SP5408MIPI_write_cmos_sensor(0xfd,0x01);
SP5408MIPI_write_cmos_sensor(0x19,0x03);
SP5408MIPI_write_cmos_sensor(0x31,0x04);
SP5408MIPI_write_cmos_sensor(0x01,0x01);

SP5408MIPI_write_cmos_sensor(0xfd,0x02);
SP5408MIPI_write_cmos_sensor(0x37,0x00);
SP5408MIPI_write_cmos_sensor(0x38,0x04);
SP5408MIPI_write_cmos_sensor(0x3b,0x00);
SP5408MIPI_write_cmos_sensor(0x3c,0x02);
SP5408MIPI_write_cmos_sensor(0x39,0x03);
SP5408MIPI_write_cmos_sensor(0x3a,0xcc);
SP5408MIPI_write_cmos_sensor(0x3d,0x02);
SP5408MIPI_write_cmos_sensor(0x3e,0x88);

SP5408MIPI_write_cmos_sensor(0xfd,0x05);
SP5408MIPI_write_cmos_sensor(0x02,0x10);
SP5408MIPI_write_cmos_sensor(0x03,0x05);
SP5408MIPI_write_cmos_sensor(0x04,0xcc);
SP5408MIPI_write_cmos_sensor(0x05,0x03);
}
	
}

void SP5408MIPI_set_5M(void)
{	//77 capture setting
	SENSORDB("[SP5408MIPI]exit SP5408MIPI_set_8M function\n"); 
//capture setting
{
//pixel timing 
/*SP5408MIPI_write_cmos_sensor(0xfd,0x01);

SP5408MIPI_write_cmos_sensor(0x0f,0x7f);
SP5408MIPI_write_cmos_sensor(0x10,0x7e);
SP5408MIPI_write_cmos_sensor(0x11,0x00);
SP5408MIPI_write_cmos_sensor(0x12,0x00);
SP5408MIPI_write_cmos_sensor(0x13,0x2f);
SP5408MIPI_write_cmos_sensor(0x16,0x00);
SP5408MIPI_write_cmos_sensor(0x17,0x2f);
SP5408MIPI_write_cmos_sensor(0x14,0x00);
SP5408MIPI_write_cmos_sensor(0x15,0x38);
SP5408MIPI_write_cmos_sensor(0x6b,0x32);
SP5408MIPI_write_cmos_sensor(0x6c,0x32);    
SP5408MIPI_write_cmos_sensor(0x6f,0x32);
SP5408MIPI_write_cmos_sensor(0x70,0x40);
SP5408MIPI_write_cmos_sensor(0x71,0x40);
SP5408MIPI_write_cmos_sensor(0x72,0x40);
SP5408MIPI_write_cmos_sensor(0x73,0x33);
SP5408MIPI_write_cmos_sensor(0x74,0x38);
SP5408MIPI_write_cmos_sensor(0x75,0x38);
SP5408MIPI_write_cmos_sensor(0x76,0x44);
SP5408MIPI_write_cmos_sensor(0x77,0x38);
SP5408MIPI_write_cmos_sensor(0x7a,0x40);
SP5408MIPI_write_cmos_sensor(0x7e,0x40);
SP5408MIPI_write_cmos_sensor(0x84,0x40);
SP5408MIPI_write_cmos_sensor(0x7d,0x08);//f//hanlei14.3.20*/

//2592x1944
SP5408MIPI_write_cmos_sensor(0xfd,0x00);
SP5408MIPI_write_cmos_sensor(0x10,0x04);//hanlei
SP5408MIPI_write_cmos_sensor(0x11,0x00);

SP5408MIPI_write_cmos_sensor(0xfd,0x01);
SP5408MIPI_write_cmos_sensor(0x19,0x00);
SP5408MIPI_write_cmos_sensor(0x31,0x00);
SP5408MIPI_write_cmos_sensor(0x01,0x01);

SP5408MIPI_write_cmos_sensor(0xfd,0x02);
SP5408MIPI_write_cmos_sensor(0x37,0x00);
SP5408MIPI_write_cmos_sensor(0x38,0x08);
SP5408MIPI_write_cmos_sensor(0x3b,0x00);
SP5408MIPI_write_cmos_sensor(0x3c,0x04);
SP5408MIPI_write_cmos_sensor(0x39,0x07);
SP5408MIPI_write_cmos_sensor(0x3a,0x98);
SP5408MIPI_write_cmos_sensor(0x3d,0x05);
SP5408MIPI_write_cmos_sensor(0x3e,0x10);

SP5408MIPI_write_cmos_sensor(0xfd,0x05);
SP5408MIPI_write_cmos_sensor(0x02,0x20);
SP5408MIPI_write_cmos_sensor(0x03,0x0a);
SP5408MIPI_write_cmos_sensor(0x04,0x98);
SP5408MIPI_write_cmos_sensor(0x05,0x07);
}
}
/*****************************************************************************/
/* Windows Mobile Sensor Interface */
/*****************************************************************************/
/*************************************************************************
* FUNCTION
*   SP5408MIPIOpen
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

UINT32 SP5408MIPIOpen(void)
{
	kal_uint32 rc = 0;
#ifdef SP5408MIPI_USE_OTP
    if(0 == used_otp){
	printk("[SP5408MIPI_USE_OTP] before update otp wb...........................................\n");
	printk("[SP5408MIPI_USE_OTP] before update otp wb...........................................\n");
	printk("[SP5408MIPI_USE_OTP] before update otp wb...........................................\n");
	ret = sp5408_update_otp_wb();

	used_otp =1;
	printk("[SP5408MIPI_USE_OTP] after update otp wb............................................\n");
	printk("[SP5408MIPI_USE_OTP] after update otp wb............................................\n");
	printk("[SP5408MIPI_USE_OTP] after update otp wb............................................\n");
    }
#endif
    int  retry = 0; 
	kal_uint16 sensorid;
	kal_uint16 sensorIDH;
	kal_uint16 sensorIDL;
	kal_uint16 sensorID;
    // check if sensor ID correct
    retry = 3; 
	SENSORDB("[hcy SP5408MIPI]enter SP5408MIPIOpen function\n");
    do {

	  SP5408MIPI_write_cmos_sensor(0xfd, 0x00);
	   sensorIDH = (SP5408MIPI_read_cmos_sensor(0x02)<<8)&0xFF00;
	  
	   sensorIDL = SP5408MIPI_read_cmos_sensor(0x03) ;
	   sensorid = sensorIDH | sensorIDL;

	   SENSORDB("hcy sensorIDL =  0x%04x\n", sensorid);

	   spin_lock(&sp5408_drv_lock);    
	   SP5408MIPI_sensor_id =sensorid;
	   spin_unlock(&sp5408_drv_lock);
		if (SP5408MIPI_sensor_id == SP5408MIPI_SENSOR_ID)
			break; 
		retry--; 

		
	    }
	while (retry > 0);
    SENSORDB("Read Sensor ID = 0x%04x\n", SP5408MIPI_sensor_id);
    if (SP5408MIPI_sensor_id != SP5408MIPI_SENSOR_ID)
        return ERROR_SENSOR_CONNECT_FAIL;

#ifdef DEBUG_SENSOR  //gepeiwei   120903
					//判断手机对应目录下是否有名为sp5408_sd 的文件,没有默认参数

					//介于各种原因，本版本初始化参数在_s_fmt中。
 struct file *fp; 
    mm_segment_t fs; 
    loff_t pos = 0; 
	static char buf[10*1024] ;
 
    fp = filp_open("/mnt/sdcard/sp5408_sd", O_RDONLY , 0); 
    if (IS_ERR(fp)) { 
		fromsd = 0;   
		printk("gpw open file error\n");
	//	return 0;
    } 
	
	else 
		{
		fromsd = 1;
	//SP5408MIPI_Initialize_from_T_Flash();
	printk("gpw read ok!\n");

	filp_close(fp, NULL); 
    set_fs(fs);
		}
#endif

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
		rc = device_create_file(fcam_dev, &dev_attr_fcam);
		rc = device_create_file(fcam_dev, &dev_attr_currreg);
	       ONLINE_DEBUG_BZW=KAL_FALSE;
       }

#endif


	
    SP5408MIPI_Sensor_Init();
	sensorid=read_SP5408MIPI_gain();
	spin_lock(&sp5408_drv_lock);	
    SP5408MIPI_sensor_gain_base = sensorid;
	spin_unlock(&sp5408_drv_lock);
	SENSORDB("[SP5408MIPI]exit SP5408MIPIOpen function\n");
    return ERROR_NONE;
}

/*************************************************************************
* FUNCTION
*   SP5408MIPIGetSensorID
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
UINT32 SP5408MIPIGetSensorID(UINT32 *sensorID) 
{
    int  retry = 3; 
	SENSORDB("[SP5408MIPI]enter SP5408MIPIGetSensorID function\n");


//test 5408 sensor id for iic
    do {
	//int  retry = 3; 
	kal_uint16 sensorIDH = 0;
	kal_uint16 sensorIDL = 0;
	    // check if sensor ID correct
	    SENSORDB("hcy 5408MIPIGetSensorID\n"); 
	  SP5408MIPI_write_cmos_sensor(0xfd, 0x00);
        SENSORDB("hcy Read Sensor ID tets = 0x%04x  reg0x04=%d\n", *sensorID,SP5408MIPI_read_cmos_sensor(0x04)); 

	  // *sensorID =(kal_uint16)((IMX135MIPI_read_cmos_sensor_truly(0x0000)<<8) | IMX135MIPI_read_cmos_sensor_truly(0x0001)); 
	   sensorIDH = (SP5408MIPI_read_cmos_sensor(0x02)<<8)&0xFF00;
	  SENSORDB("hcy 5408 sensorIDH =  0x%04x\n", sensorIDH);
	  
	   sensorIDL = SP5408MIPI_read_cmos_sensor(0x03) ;
	   SENSORDB("hcy sensorIDL =  0x%04x\n", sensorIDL);
	   *sensorID = sensorIDH | sensorIDL;
        SENSORDB("hcy Read Sensor ID Fail = 0x%04x  reg0x04=%d\n", *sensorID,SP5408MIPI_read_cmos_sensor(0x03)); 
	   
        if (*sensorID == 0x5408)
            break;
        SENSORDB("hcy Read Sensor ID Fail = 0x%04x  reg0x04=%d\n", *sensorID,SP5408MIPI_read_cmos_sensor(0x03)); 
        retry--; 
		
    } while (retry > 0);

//end test


#if 0
    // check if sensor ID correct
    do {		
	   *sensorID =(kal_uint16)(((SP5408MIPI_read_cmos_sensor(0x0002)&&0x0f)<<8) | SP5408MIPI_read_cmos_sensor(0x0003)); 
        if (*sensorID == SP5408MIPI_SENSOR_ID)
            break;
        retry--; 
    } while (retry > 0);
	
#endif

    if (*sensorID != SP5408MIPI_SENSOR_ID) {
        *sensorID = 0xFFFFFFFF; 
        return ERROR_SENSOR_CONNECT_FAIL;
    }
        SENSORDB("hcy Read Sensor ID   = 0x%04x\n", *sensorID); 
    return ERROR_NONE;
}


/*************************************************************************
* FUNCTION
*   SP5408MIPI_SetShutter
*
* DESCRIPTION
*   This function set e-shutter of SP5408MIPI to change exposure time.
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
void SP5408MIPI_SetShutter(kal_uint16 iShutter)
{

	SENSORDB("[SP5408MIPI]%s():shutter=%d\n",__FUNCTION__,iShutter);
    if (iShutter < 1)
        iShutter = 1; 
	else if(iShutter > 0xffff)
		iShutter = 0xffff;
		//iShutter = 0x3906;//hanlei
	unsigned long flags;
	spin_lock_irqsave(&sp5408_drv_lock,flags);
    SP5408MIPI_sensor.pv_shutter = iShutter;	
		
	spin_unlock_irqrestore(&sp5408_drv_lock,flags);
    SP5408MIPI_write_shutter(iShutter);
	SENSORDB("[SP5408MIPI]exit SP5408MIPIGetSensorID function\n");
}   /*  SP5408MIPI_SetShutter   */



/*************************************************************************
* FUNCTION
*   SP5408MIPI_read_shutter
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
UINT16 SP5408MIPI_read_shutter(void)
{

    SP5408MIPI_write_cmos_sensor(0xfd, 0x01);

    return (UINT16)( (SP5408MIPI_read_cmos_sensor(0x03)<<8) | SP5408MIPI_read_cmos_sensor(0x04) );
}

/*************************************************************************
* FUNCTION
*   SP5408MIPI_night_mode
*
* DESCRIPTION
*   This function night mode of SP5408MIPI.
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
void SP5408MIPI_NightMode(kal_bool bEnable)
{
	SENSORDB("[SP5408MIPI]enter SP5408MIPI_NightMode function\n");
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
	SENSORDB("[SP5408MIPI]exit SP5408MIPI_NightMode function\n");
}/*	SP5408MIPI_NightMode */



/*************************************************************************
* FUNCTION
*   SP5408MIPIClose
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
UINT32 SP5408MIPIClose(void)
{
    //SP5408MIPI_write_cmos_sensor(0x0100,0x00);
    return ERROR_NONE;
}	/* SP5408MIPIClose() */
/*
void SP5408MIPISetFlipMirror(kal_int32 imgMirror)
{
    kal_uint8  iTemp; 
	SENSORDB("[SP5408MIPI]enter SP5408MIPISetFlipMirror function\n");

	    SP5408MIPI_write_cmos_sensor(0xfd, 0x01);


    iTemp = SP5408MIPI_read_cmos_sensor(0x3f) & 0x03;	//Clear the mirror and flip bits.
    switch (imgMirror)
    {
        case IMAGE_NORMAL:
            SP5408MIPI_write_cmos_sensor(0x3f, 0x03);	//Set normal
            break;
        case IMAGE_V_MIRROR:
            SP5408MIPI_write_cmos_sensor(0x3f, iTemp | 0x01);	//Set flip
            break;
        case IMAGE_H_MIRROR:
            SP5408MIPI_write_cmos_sensor(0x3f, iTemp | 0x02);	//Set mirror
            break;
        case IMAGE_HV_MIRROR:
            SP5408MIPI_write_cmos_sensor(0x3f, 0x00);	//Set mirror and flip
            break;
    }
	SENSORDB("[SP5408MIPI]exit SP5408MIPISetFlipMirror function\n");
}
*/

/*************************************************************************
* FUNCTION
*   SP5408MIPIPreview
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
UINT32 SP5408MIPIPreview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                                                MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	kal_uint16 iStartX = 0, iStartY = 0;	
	SENSORDB("[SP5408MIPI]enter SP5408MIPIPreview function\n");
	spin_lock(&sp5408_drv_lock);    
	SP5408MIPI_MPEG4_encode_mode = KAL_FALSE;
	SP5408MIPI_sensor.video_mode=KAL_FALSE;
	SP5408MIPI_sensor.pv_mode=KAL_TRUE;
	SP5408MIPI_sensor.capture_mode=KAL_FALSE;
	spin_unlock(&sp5408_drv_lock);

	PreviewSetting();
     //  SP5408MIPISetFlipMirror(IMAGE_NORMAL);//hanlei

	iStartX += SP5408MIPI_IMAGE_SENSOR_PV_STARTX;
	iStartY += SP5408MIPI_IMAGE_SENSOR_PV_STARTY;
	spin_lock(&sp5408_drv_lock);	
/*
	SP5408MIPI_sensor.cp_dummy_pixels = 0;
	SP5408MIPI_sensor.cp_dummy_lines = 0;
	SP5408MIPI_sensor.pv_dummy_pixels = 0;
	SP5408MIPI_sensor.pv_dummy_lines = 0;
	SP5408MIPI_sensor.video_dummy_pixels = 0;
	SP5408MIPI_sensor.video_dummy_lines = 0;
	*/
	SP5408MIPI_sensor.pv_line_length = SP5408MIPI_PV_LINE_LENGTH_PIXELS+SP5408MIPI_sensor.pv_dummy_pixels; 
	SP5408MIPI_sensor.pv_frame_length = SP5408MIPI_PV_FRAME_LENGTH_LINES+SP5408MIPI_sensor.pv_dummy_lines;
	spin_unlock(&sp5408_drv_lock);

	SP5408MIPI_SetDummy(SP5408MIPI_sensor.pv_dummy_pixels,SP5408MIPI_sensor.pv_dummy_lines);
	SP5408MIPI_SetShutter(SP5408MIPI_sensor.pv_shutter);
	spin_lock(&sp5408_drv_lock);	
	memcpy(&SP5408MIPISensorConfigData, sensor_config_data, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
	spin_unlock(&sp5408_drv_lock);
	image_window->GrabStartX= iStartX;
	image_window->GrabStartY= iStartY;
	image_window->ExposureWindowWidth= SP5408MIPI_REAL_PV_WIDTH ;
	image_window->ExposureWindowHeight= SP5408MIPI_REAL_PV_HEIGHT ; 
	SENSORDB("hcy [SP5408MIPI]eXIT SP5408MIPIPreview function\n"); 
	return ERROR_NONE;
	}	/* SP5408MIPIPreview() */

/*************************************************************************
* FUNCTION
* RETURNS
*   None
*
* GLOBALS AFFECTED
*
*************************************************************************/
UINT32 SP5408MIPIVideo(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                                                MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	kal_uint16 iStartX = 0, iStartY = 0;
	SENSORDB("[SP5408MIPI]enter SP5408MIPIVideo function\n"); 
	spin_lock(&sp5408_drv_lock);    
    SP5408MIPI_MPEG4_encode_mode = KAL_TRUE;  
	SP5408MIPI_sensor.video_mode=KAL_TRUE;
	SP5408MIPI_sensor.pv_mode=KAL_FALSE;
	SP5408MIPI_sensor.capture_mode=KAL_FALSE;
	spin_unlock(&sp5408_drv_lock);
	VideoFullSizeSetting();

	//SP5408MIPISetFlipMirror(IMAGE_NORMAL);	//add by lishengli 20130614 hanlei
	iStartX += SP5408MIPI_IMAGE_SENSOR_VIDEO_STARTX;
	iStartY += SP5408MIPI_IMAGE_SENSOR_VIDEO_STARTY;
	spin_lock(&sp5408_drv_lock);	
/*
	SP5408MIPI_sensor.cp_dummy_pixels = 0;
	SP5408MIPI_sensor.cp_dummy_lines = 0;
	SP5408MIPI_sensor.pv_dummy_pixels = 0;
	SP5408MIPI_sensor.pv_dummy_lines = 0;
	SP5408MIPI_sensor.video_dummy_pixels = 0;
	SP5408MIPI_sensor.video_dummy_lines = 0;
	*/
	SP5408MIPI_sensor.video_line_length = SP5408MIPI_VIDEO_LINE_LENGTH_PIXELS+SP5408MIPI_sensor.video_dummy_pixels; 
	SP5408MIPI_sensor.video_frame_length = SP5408MIPI_VIDEO_FRAME_LENGTH_LINES+SP5408MIPI_sensor.video_dummy_lines;
	spin_unlock(&sp5408_drv_lock);
	
	SP5408MIPI_SetDummy(SP5408MIPI_sensor.video_dummy_pixels,SP5408MIPI_sensor.video_dummy_lines);
	SP5408MIPI_SetShutter(SP5408MIPI_sensor.video_shutter);
	spin_lock(&sp5408_drv_lock);	
	memcpy(&SP5408MIPISensorConfigData, sensor_config_data, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
	spin_unlock(&sp5408_drv_lock);
	image_window->GrabStartX= iStartX;
	image_window->GrabStartY= iStartY;    
    SENSORDB("[SP5408MIPI]eXIT SP5408MIPIVideo function\n"); 
	return ERROR_NONE;
}	/* SP5408MIPIPreview() */

UINT32 SP5408MIPICapture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                                                MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	kal_uint16 iStartX = 0, iStartY = 0;
	SENSORDB("[SP5408MIPI]enter SP5408MIPICapture function\n");
	spin_lock(&sp5408_drv_lock);	
	SP5408MIPI_sensor.video_mode=KAL_FALSE;
	SP5408MIPI_sensor.pv_mode=KAL_FALSE;
	SP5408MIPI_sensor.capture_mode=KAL_TRUE;
	SP5408MIPI_MPEG4_encode_mode = KAL_FALSE; 
	SP5408MIPI_Auto_Flicker_mode = KAL_FALSE;       
	//SP5408MIPI_sensor.cp_dummy_pixels = 0;
	//SP5408MIPI_sensor.cp_dummy_lines = 0;
	spin_unlock(&sp5408_drv_lock);
	if((image_window->ExposureWindowWidth<=SP5408MIPI_REAL_PV_WIDTH)&&(image_window->ExposureWindowHeight<=SP5408MIPI_REAL_PV_HEIGHT))
		{
	PreviewSetting();
//	SP5408MIPISetFlipMirror(IMAGE_NORMAL);	//add by lishengli 20130614
	spin_lock(&sp5408_drv_lock);    
	SP5408MIPI_sensor.pv_line_length=SP5408MIPI_PV_LINE_LENGTH_PIXELS+SP5408MIPI_sensor.pv_dummy_pixels;
	SP5408MIPI_sensor.pv_frame_length=SP5408MIPI_PV_FRAME_LENGTH_LINES+SP5408MIPI_sensor.pv_dummy_lines;
	spin_unlock(&sp5408_drv_lock);
	iStartX = SP5408MIPI_IMAGE_SENSOR_PV_STARTX;
	iStartY = SP5408MIPI_IMAGE_SENSOR_PV_STARTY;
	image_window->GrabStartX=iStartX;
	image_window->GrabStartY=iStartY;
	image_window->ExposureWindowWidth=SP5408MIPI_REAL_PV_WIDTH ;
	image_window->ExposureWindowHeight=SP5408MIPI_REAL_PV_HEIGHT;
	SP5408MIPI_SetDummy(SP5408MIPI_sensor.pv_dummy_pixels, SP5408MIPI_sensor.pv_dummy_lines);   }
	else
		{
	SP5408MIPI_set_5M();
//	SP5408MIPISetFlipMirror(IMAGE_NORMAL);	//add by lishengli 20130614
	spin_lock(&sp5408_drv_lock);    
	SP5408MIPI_sensor.cp_line_length=SP5408MIPI_FULL_LINE_LENGTH_PIXELS+SP5408MIPI_sensor.cp_dummy_pixels;
	SP5408MIPI_sensor.cp_frame_length=SP5408MIPI_FULL_FRAME_LENGTH_LINES+SP5408MIPI_sensor.cp_dummy_lines;
	spin_unlock(&sp5408_drv_lock);
	iStartX = SP5408MIPI_IMAGE_SENSOR_CAP_STARTX;
	iStartY = SP5408MIPI_IMAGE_SENSOR_CAP_STARTY;
	image_window->GrabStartX=iStartX;
	image_window->GrabStartY=iStartY;
	image_window->ExposureWindowWidth=SP5408MIPI_REAL_CAP_WIDTH ;
	image_window->ExposureWindowHeight=SP5408MIPI_REAL_CAP_HEIGHT;
	SP5408MIPI_SetDummy(SP5408MIPI_sensor.cp_dummy_pixels, SP5408MIPI_sensor.cp_dummy_lines);   
		}
	spin_lock(&sp5408_drv_lock);	
	memcpy(&SP5408MIPISensorConfigData, sensor_config_data, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
	spin_unlock(&sp5408_drv_lock);
	SENSORDB("[SP5408MIPI]exit SP5408MIPICapture function\n");
	return ERROR_NONE;
}	/* SP5408MIPICapture() */

UINT32 SP5408MIPIGetResolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *pSensorResolution)
{
    SENSORDB("[SP5408MIPI]eXIT SP5408MIPIGetResolution function\n");
    pSensorResolution->SensorPreviewWidth	= SP5408MIPI_REAL_PV_WIDTH;
    pSensorResolution->SensorPreviewHeight	= SP5408MIPI_REAL_PV_HEIGHT;
    pSensorResolution->SensorFullWidth		= SP5408MIPI_REAL_CAP_WIDTH;
    pSensorResolution->SensorFullHeight		= SP5408MIPI_REAL_CAP_HEIGHT;
    pSensorResolution->SensorVideoWidth		= SP5408MIPI_REAL_VIDEO_WIDTH;
    pSensorResolution->SensorVideoHeight    = SP5408MIPI_REAL_VIDEO_HEIGHT;
    SENSORDB("SP5408MIPIGetResolution :8-14");    

    return ERROR_NONE;
}   /* SP5408MIPIGetResolution() */

UINT32 SP5408MIPIGetInfo(MSDK_SCENARIO_ID_ENUM ScenarioId,
                                                MSDK_SENSOR_INFO_STRUCT *pSensorInfo,
                                                MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{ 
	SENSORDB("[SP5408MIPI]enter SP5408MIPIGetInfo function\n");
	switch(ScenarioId){
			case MSDK_SCENARIO_ID_CAMERA_ZSD:
			case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG://hhl 2-28
				pSensorInfo->SensorFullResolutionX=SP5408MIPI_REAL_CAP_WIDTH;
				pSensorInfo->SensorFullResolutionY=SP5408MIPI_REAL_CAP_HEIGHT;
				pSensorInfo->SensorStillCaptureFrameRate=6;//22//hanlei

			break;//hhl 2-28
			case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
				pSensorInfo->SensorPreviewResolutionX=SP5408MIPI_REAL_VIDEO_WIDTH;
				pSensorInfo->SensorPreviewResolutionY=SP5408MIPI_REAL_VIDEO_HEIGHT;
				pSensorInfo->SensorVideoFrameRate=18;//30//hanlei
			break;
		default:
        pSensorInfo->SensorPreviewResolutionX=SP5408MIPI_REAL_PV_WIDTH;
        pSensorInfo->SensorPreviewResolutionY=SP5408MIPI_REAL_PV_HEIGHT;
				pSensorInfo->SensorCameraPreviewFrameRate=18;//30//hanlei
			break;
	}
    pSensorInfo->SensorVideoFrameRate=18;	//30//hanlei
    pSensorInfo->SensorStillCaptureFrameRate=6;//24//hanlei
    pSensorInfo->SensorWebCamCaptureFrameRate=18;//24//hanlei
    pSensorInfo->SensorResetActiveHigh=FALSE;
    pSensorInfo->SensorResetDelayCount=5;
    pSensorInfo->SensorOutputDataFormat=SENSOR_OUTPUT_FORMAT_RAW_B;//hanlei
    pSensorInfo->SensorClockPolarity=SENSOR_CLOCK_POLARITY_LOW; /*??? */
    pSensorInfo->SensorClockFallingPolarity=SENSOR_CLOCK_POLARITY_LOW;
    pSensorInfo->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
    pSensorInfo->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_HIGH;//hanlei
    pSensorInfo->SensorInterruptDelayLines = 1;
    pSensorInfo->SensroInterfaceType=SENSOR_INTERFACE_TYPE_MIPI;//hanlei//SENSOR_INTERFACE_TYPE_PARALLEL;

    pSensorInfo->CaptureDelayFrame = 3; 
    pSensorInfo->PreviewDelayFrame = 2; 
    pSensorInfo->VideoDelayFrame = 2; 
    pSensorInfo->SensorMasterClockSwitch = 0; 
    pSensorInfo->SensorDrivingCurrent = ISP_DRIVING_8MA; //hanlei dri     
    pSensorInfo->AEShutDelayFrame = 2;	//0//hanlei	    /* The frame of setting shutter default 0 for TG int */
    pSensorInfo->AESensorGainDelayFrame = 2;  //0//hanlei   /* The frame of setting sensor gain */
    pSensorInfo->AEISPGainDelayFrame = 2;	
	   
    switch (ScenarioId)
    {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
            pSensorInfo->SensorClockFreq=24;
            pSensorInfo->SensorClockDividCount=	5;
            pSensorInfo->SensorClockRisingCount= 0;
            pSensorInfo->SensorClockFallingCount= 2;
            pSensorInfo->SensorPixelClockCount= 3;
            pSensorInfo->SensorDataLatchCount= 2;
            pSensorInfo->SensorGrabStartX = SP5408MIPI_IMAGE_SENSOR_PV_STARTX; 
            pSensorInfo->SensorGrabStartY = SP5408MIPI_IMAGE_SENSOR_PV_STARTY;           		
            /*pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_2_LANE;			
            pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0; 
	     	pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 14; 
	    	pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;*/
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
			   pSensorInfo->SensorClockDividCount= 5;
			   pSensorInfo->SensorClockRisingCount= 0;
			   pSensorInfo->SensorClockFallingCount= 2;
			   pSensorInfo->SensorPixelClockCount= 3;
			   pSensorInfo->SensorDataLatchCount= 2;
			   pSensorInfo->SensorGrabStartX = SP5408MIPI_IMAGE_SENSOR_VIDEO_STARTX; 
			   pSensorInfo->SensorGrabStartY = SP5408MIPI_IMAGE_SENSOR_VIDEO_STARTY;				   
			   /*pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_2_LANE;		   
			   pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0; 
			pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 14; 
			pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;*/
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
            pSensorInfo->SensorClockDividCount=	5;
            pSensorInfo->SensorClockRisingCount= 0;
            pSensorInfo->SensorClockFallingCount= 2;
            pSensorInfo->SensorPixelClockCount= 3;
            pSensorInfo->SensorDataLatchCount= 2;
            pSensorInfo->SensorGrabStartX = SP5408MIPI_IMAGE_SENSOR_CAP_STARTX;//	//2*SP5408MIPI_IMAGE_SENSOR_PV_STARTX; 
            pSensorInfo->SensorGrabStartY = SP5408MIPI_IMAGE_SENSOR_CAP_STARTY;//	//2*SP5408MIPI_IMAGE_SENSOR_PV_STARTY;          			
            /*pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_2_LANE;			
            pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0; 
            pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 14; 
            pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0; */
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
			 pSensorInfo->SensorClockDividCount= 5;
			 pSensorInfo->SensorClockRisingCount= 0;
			 pSensorInfo->SensorClockFallingCount= 2;
			 pSensorInfo->SensorPixelClockCount= 3;
			 pSensorInfo->SensorDataLatchCount= 2;
			 pSensorInfo->SensorGrabStartX = SP5408MIPI_IMAGE_SENSOR_PV_STARTX; 
			 pSensorInfo->SensorGrabStartY = SP5408MIPI_IMAGE_SENSOR_PV_STARTY; 				 
			/* pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_2_LANE;		 
			 pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0; 
		     pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 14; 
		  	 pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;*/
                      pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_2_LANE;		 
			 pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0; 
		        pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 14; 
		  	 pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;		  	 
			 pSensorInfo->SensorWidthSampling = 0;	// 0 is default 1x
			 pSensorInfo->SensorHightSampling = 0;	 // 0 is default 1x 
			 pSensorInfo->SensorPacketECCOrder = 1;

            break;
    }
	spin_lock(&sp5408_drv_lock);	
    SP5408MIPIPixelClockDivider=pSensorInfo->SensorPixelClockCount;
    memcpy(pSensorConfigData, &SP5408MIPISensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
	spin_unlock(&sp5408_drv_lock);
    SENSORDB("[hcy SP5408MIPI]exit SP5408MIPIGetInfo function\n");
    return ERROR_NONE;
}   /* SP5408MIPIGetInfo() */


UINT32 SP5408MIPIControl(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *pImageWindow,
                                                MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{    
		spin_lock(&sp5408_drv_lock);	
		CurrentScenarioId = ScenarioId;
		spin_unlock(&sp5408_drv_lock);
		SENSORDB("[SP5408MIPI]enter SP5408MIPIControl function\n");
    switch (ScenarioId)
    {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
            SP5408MIPIPreview(pImageWindow, pSensorConfigData);
            break;
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			SP5408MIPIVideo(pImageWindow, pSensorConfigData);
            break;
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
	    case MSDK_SCENARIO_ID_CAMERA_ZSD:
            SP5408MIPICapture(pImageWindow, pSensorConfigData);//hhl 2-28
            break;
        default:
            return ERROR_INVALID_SCENARIO_ID;
    }
	SENSORDB("[SP5408MIPI]exit SP5408MIPIControl function\n");
    return ERROR_NONE;
} /* SP5408MIPIControl() */

UINT32 SP5408MIPISetVideoMode(UINT16 u2FrameRate)
{
    SENSORDB("[SP5408MIPISetVideoMode] frame rate = %d\n", u2FrameRate);
	kal_uint16 SP5408MIPI_Video_Max_Expourse_Time = 0;
	SENSORDB("[SP5408MIPI]%s():fix_frame_rate=%d\n",__FUNCTION__,u2FrameRate);
	spin_lock(&sp5408_drv_lock);
	SP5408MIPI_sensor.fix_video_fps = KAL_TRUE;
	spin_unlock(&sp5408_drv_lock);
	u2FrameRate=u2FrameRate*10;//10*FPS
	SENSORDB("[SP5408MIPI][Enter Fix_fps func] SP5408MIPI_Fix_Video_Frame_Rate = %d\n", u2FrameRate/10);
	SP5408MIPI_Video_Max_Expourse_Time = (kal_uint16)((SP5408MIPI_sensor.video_pclk*10/u2FrameRate)/SP5408MIPI_sensor.video_line_length);
	
	if (SP5408MIPI_Video_Max_Expourse_Time > SP5408MIPI_VIDEO_FRAME_LENGTH_LINES/*SP5408MIPI_sensor.pv_frame_length*/) 
	{
		spin_lock(&sp5408_drv_lock);    
		SP5408MIPI_sensor.video_frame_length = SP5408MIPI_Video_Max_Expourse_Time;
		SP5408MIPI_sensor.video_dummy_lines = SP5408MIPI_sensor.video_frame_length-SP5408MIPI_VIDEO_FRAME_LENGTH_LINES;
		spin_unlock(&sp5408_drv_lock);
		SENSORDB("[SP5408MIPI]%s():frame_length=%d,dummy_lines=%d\n",__FUNCTION__,SP5408MIPI_sensor.video_frame_length,SP5408MIPI_sensor.video_dummy_lines);
		SP5408MIPI_SetDummy(SP5408MIPI_sensor.video_dummy_pixels,SP5408MIPI_sensor.video_dummy_lines);
	}
	spin_lock(&sp5408_drv_lock);    
	SP5408MIPI_MPEG4_encode_mode = KAL_TRUE; 
	spin_unlock(&sp5408_drv_lock);
	SENSORDB("[SP5408MIPI]exit SP5408MIPISetVideoMode function\n");
	return ERROR_NONE;
}

UINT32 SP5408MIPISetAutoFlickerMode(kal_bool bEnable, UINT16 u2FrameRate)
{
	kal_uint32 pv_max_frame_rate_lines=0;

	if(SP5408MIPI_sensor.pv_mode==TRUE)
	pv_max_frame_rate_lines=SP5408MIPI_PV_FRAME_LENGTH_LINES;
	else
    pv_max_frame_rate_lines=SP5408MIPI_VIDEO_FRAME_LENGTH_LINES	;
    SENSORDB("[SP5408MIPISetAutoFlickerMode] frame rate(10base) = %d %d\n", bEnable, u2FrameRate);
    if(bEnable) 
	{   // enable auto flicker   
    	spin_lock(&sp5408_drv_lock);    
        SP5408MIPI_Auto_Flicker_mode = KAL_TRUE; 
		spin_unlock(&sp5408_drv_lock);
        if(SP5408MIPI_MPEG4_encode_mode == KAL_TRUE) 
		{ // in the video mode, reset the frame rate
            pv_max_frame_rate_lines = SP5408MIPI_MAX_EXPOSURE_LINES + (SP5408MIPI_MAX_EXPOSURE_LINES>>7);            
           // SP5408MIPI_write_cmos_sensor(0x0104, 1);        
            //SP5408MIPI_write_cmos_sensor(0x0340, (pv_max_frame_rate_lines >>8) & 0xFF);
            //SP5408MIPI_write_cmos_sensor(0x0341, pv_max_frame_rate_lines & 0xFF);	
            //SP5408MIPI_write_cmos_sensor(0x0104, 0);        	
        }
    } 
	else 
	{
    	spin_lock(&sp5408_drv_lock);    
        SP5408MIPI_Auto_Flicker_mode = KAL_FALSE; 
		spin_unlock(&sp5408_drv_lock);
        if(SP5408MIPI_MPEG4_encode_mode == KAL_TRUE) 
		{    // in the video mode, restore the frame rate
            //SP5408MIPI_write_cmos_sensor(0x0104, 1);        
            //SP5408MIPI_write_cmos_sensor(0x0340, (SP5408MIPI_MAX_EXPOSURE_LINES >>8) & 0xFF);
            //SP5408MIPI_write_cmos_sensor(0x0341, SP5408MIPI_MAX_EXPOSURE_LINES & 0xFF);	
           // SP5408MIPI_write_cmos_sensor(0x0104, 0);        	
        }
        printk("Disable Auto flicker\n");    
    }
    return ERROR_NONE;
}
UINT32 SP5408MIPISetMaxFramerateByScenario(MSDK_SCENARIO_ID_ENUM scenarioId, MUINT32 frameRate) {
	kal_uint32 pclk;
	kal_int16 dummyLine;
	kal_uint16 lineLength,frameHeight;	
	SENSORDB("SP5408MIPISetMaxFramerateByScenario: scenarioId = %d, frame rate = %d\n",scenarioId,frameRate);
	switch (scenarioId) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			pclk =SP5408MIPI_sensor.pv_pclk;//hanlei
			//lineLength = SP5408MIPI_PV_LINE_LENGTH_PIXELS;
			lineLength = SP5408MIPI_PV_LINE_LENGTH_PIXELS+SP5408MIPI_sensor.pv_dummy_pixels;//hanlei140409
			frameHeight = (10 * pclk)/frameRate/lineLength;
			dummyLine = frameHeight - SP5408MIPI_PV_FRAME_LENGTH_LINES;

			
			if(dummyLine<0)
				SP5408MIPI_sensor.pv_dummy_lines = 0;
			spin_lock(&sp5408_drv_lock);	
			SP5408MIPI_sensor.pv_mode=TRUE;
			spin_unlock(&sp5408_drv_lock);
			SP5408MIPI_sensor.pv_dummy_lines = dummyLine;		
			break;			
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			pclk = SP5408MIPI_sensor.video_pclk;//hanlei
			lineLength = SP5408MIPI_VIDEO_LINE_LENGTH_PIXELS+SP5408MIPI_sensor.video_dummy_pixels;//hanlei140409
			frameHeight = (10 * pclk)/frameRate/lineLength;
			dummyLine = frameHeight - SP5408MIPI_VIDEO_FRAME_LENGTH_LINES;
			if(dummyLine<0)
				SP5408MIPI_sensor.video_dummy_lines = 0;
			spin_lock(&sp5408_drv_lock);	
			SP5408MIPI_sensor.pv_mode=TRUE;
			spin_unlock(&sp5408_drv_lock);
			SP5408MIPI_sensor.video_dummy_lines = dummyLine;	
			break;			
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			pclk = SP5408MIPI_sensor.cp_pclk;//hanlei
			lineLength = SP5408MIPI_FULL_LINE_LENGTH_PIXELS+SP5408MIPI_sensor.cp_dummy_pixels;//hanlei140409
			frameHeight = (10 * pclk)/frameRate/lineLength;
			dummyLine = frameHeight - SP5408MIPI_FULL_FRAME_LENGTH_LINES;
			if(dummyLine<0)
				SP5408MIPI_sensor.cp_dummy_lines = 0;
			
			spin_lock(&sp5408_drv_lock);	
			SP5408MIPI_sensor.pv_mode=FALSE;
			spin_unlock(&sp5408_drv_lock);
			SP5408MIPI_sensor.cp_dummy_lines = dummyLine;	
			break;		
		case MSDK_SCENARIO_ID_CAMERA_ZSD:			
			pclk = SP5408MIPI_sensor.cp_pclk;//hanlei
			lineLength = SP5408MIPI_FULL_LINE_LENGTH_PIXELS+SP5408MIPI_sensor.cp_dummy_pixels;//hanlei140409
			frameHeight = (10 * pclk)/frameRate/lineLength;
			dummyLine = frameHeight - SP5408MIPI_FULL_FRAME_LENGTH_LINES;
			if(dummyLine<0)
				SP5408MIPI_sensor.cp_dummy_lines = 0;
			
			spin_lock(&sp5408_drv_lock);	
			SP5408MIPI_sensor.pv_mode=FALSE;
			spin_unlock(&sp5408_drv_lock);
			SP5408MIPI_sensor.cp_dummy_lines = dummyLine;	
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
	SENSORDB("[SP5408MIPI]exit SP5408MIPISetMaxFramerateByScenario function\n");
	return ERROR_NONE;
}
UINT32 SP5408MIPIGetDefaultFramerateByScenario(MSDK_SCENARIO_ID_ENUM scenarioId, MUINT32 *pframeRate) 
{

	switch (scenarioId) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			 *pframeRate = 180; //hanlei add
			 break;			
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			 *pframeRate = 180; //hanlei
			 break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
			 *pframeRate = 60 ;//hanlei
			break;		//hhl 2-28
        case MSDK_SCENARIO_ID_CAMERA_3D_PREVIEW: //added
        case MSDK_SCENARIO_ID_CAMERA_3D_VIDEO:
        case MSDK_SCENARIO_ID_CAMERA_3D_CAPTURE: //added   
			 *pframeRate = 180 ;//hanlei
			break;		
		default:
			break;
	}

	return ERROR_NONE;
}
UINT32 SP5408MIPISetTestPatternMode(kal_bool bEnable)
{
    SENSORDB("[SP5408MIPISetTestPatternMode] Test pattern enable:%d\n", bEnable);
    
    if(bEnable) {   // enable color bar   
        //SP5408MIPI_write_cmos_sensor(0x30D8, 0x10);  // color bar test pattern
        //SP5408MIPI_write_cmos_sensor(0x0600, 0x00);  // color bar test pattern
        //SP5408MIPI_write_cmos_sensor(0x0601, 0x02);  // color bar test pattern 
    } else {
        //SP5408MIPI_write_cmos_sensor(0x30D8, 0x00);  // disable color bar test pattern
    }
    return ERROR_NONE;
}

UINT32 SP5408MIPIFeatureControl(MSDK_SENSOR_FEATURE_ENUM FeatureId,
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
            *pFeatureReturnPara16++=SP5408MIPI_REAL_CAP_WIDTH;
            *pFeatureReturnPara16=SP5408MIPI_REAL_CAP_HEIGHT;
            *pFeatureParaLen=4;
            break;
        case SENSOR_FEATURE_GET_PERIOD:
        		switch(CurrentScenarioId)
        		{
        			case MSDK_SCENARIO_ID_CAMERA_ZSD:
        		    case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
 		            *pFeatureReturnPara16++=SP5408MIPI_sensor.cp_line_length;  
 		            *pFeatureReturnPara16=SP5408MIPI_sensor.cp_frame_length;
		            SENSORDB("Sensor period:%d %d\n",SP5408MIPI_sensor.cp_line_length, SP5408MIPI_sensor.cp_frame_length); 
		            *pFeatureParaLen=4;        				
        				break;
        			case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
					*pFeatureReturnPara16++=SP5408MIPI_sensor.video_line_length;  
					*pFeatureReturnPara16=SP5408MIPI_sensor.video_frame_length;
					 SENSORDB("Sensor period:%d %d\n", SP5408MIPI_sensor.video_line_length, SP5408MIPI_sensor.video_frame_length); 
					 *pFeatureParaLen=4;
						break;
        			default:	
					*pFeatureReturnPara16++=SP5408MIPI_sensor.pv_line_length;  
					*pFeatureReturnPara16=SP5408MIPI_sensor.pv_frame_length;
		            SENSORDB("Sensor period:%d %d\n", SP5408MIPI_sensor.pv_line_length, SP5408MIPI_sensor.pv_frame_length); 
		            *pFeatureParaLen=4;
	            break;
          	}
          	break;
        case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
        		switch(CurrentScenarioId)
        		{
        			case MSDK_SCENARIO_ID_CAMERA_ZSD:
        			case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		            *pFeatureReturnPara32 = SP5408MIPI_sensor.cp_pclk; 
		            *pFeatureParaLen=4;		         	
					
		            SENSORDB("Sensor CPCLK:%dn",SP5408MIPI_sensor.cp_pclk); 
		         		break; //hhl 2-28
					case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
						*pFeatureReturnPara32 = SP5408MIPI_sensor.video_pclk;
						*pFeatureParaLen=4;
						SENSORDB("Sensor videoCLK:%d\n",SP5408MIPI_sensor.video_pclk); 
						break;
		         		default:
		            *pFeatureReturnPara32 = SP5408MIPI_sensor.pv_pclk;
		            *pFeatureParaLen=4;
					SENSORDB("Sensor pvclk:%d\n",SP5408MIPI_sensor.pv_pclk); 
		            break;
		         }
		         break;
        case SENSOR_FEATURE_SET_ESHUTTER:
            SP5408MIPI_SetShutter(*pFeatureData16); 
            break;
		case SENSOR_FEATURE_SET_SENSOR_SYNC: 
			break;
        case SENSOR_FEATURE_SET_NIGHTMODE:
            SP5408MIPI_NightMode((BOOL) *pFeatureData16);
            break;
        case SENSOR_FEATURE_SET_GAIN:
           SP5408MIPI_SetGain((UINT16) *pFeatureData16); 
            break;
        case SENSOR_FEATURE_SET_FLASHLIGHT:
            break;
        case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
			spin_lock(&sp5408_drv_lock);    
            SP5408MIPI_isp_master_clock=*pFeatureData32;
			spin_unlock(&sp5408_drv_lock);
            break;
        case SENSOR_FEATURE_SET_REGISTER:
			SP5408MIPI_write_cmos_sensor(pSensorRegData->RegAddr, pSensorRegData->RegData);
            break;
        case SENSOR_FEATURE_GET_REGISTER:
            pSensorRegData->RegData = SP5408MIPI_read_cmos_sensor(pSensorRegData->RegAddr);
            break;
        case SENSOR_FEATURE_SET_CCT_REGISTER:
            SensorRegNumber=FACTORY_END_ADDR;
            for (i=0;i<SensorRegNumber;i++)
            {
            	spin_lock(&sp5408_drv_lock);    
                SP5408MIPISensorCCT[i].Addr=*pFeatureData32++;
                SP5408MIPISensorCCT[i].Para=*pFeatureData32++; 
				spin_unlock(&sp5408_drv_lock);
            }
            break;
        case SENSOR_FEATURE_GET_CCT_REGISTER:
            SensorRegNumber=FACTORY_END_ADDR;
            if (*pFeatureParaLen<(SensorRegNumber*sizeof(SENSOR_REG_STRUCT)+4))
                return FALSE;
            *pFeatureData32++=SensorRegNumber;
            for (i=0;i<SensorRegNumber;i++)
            {
                *pFeatureData32++=SP5408MIPISensorCCT[i].Addr;
                *pFeatureData32++=SP5408MIPISensorCCT[i].Para; 
            }
            break;
        case SENSOR_FEATURE_SET_ENG_REGISTER:
            SensorRegNumber=ENGINEER_END;
            for (i=0;i<SensorRegNumber;i++)
            {	spin_lock(&sp5408_drv_lock);    
                SP5408MIPISensorReg[i].Addr=*pFeatureData32++;
                SP5408MIPISensorReg[i].Para=*pFeatureData32++;
				spin_unlock(&sp5408_drv_lock);
            }
            break;
        case SENSOR_FEATURE_GET_ENG_REGISTER:
            SensorRegNumber=ENGINEER_END;
            if (*pFeatureParaLen<(SensorRegNumber*sizeof(SENSOR_REG_STRUCT)+4))
                return FALSE;
            *pFeatureData32++=SensorRegNumber;
            for (i=0;i<SensorRegNumber;i++)
            {
                *pFeatureData32++=SP5408MIPISensorReg[i].Addr;
                *pFeatureData32++=SP5408MIPISensorReg[i].Para;
            }
            break;
        case SENSOR_FEATURE_GET_REGISTER_DEFAULT:
            if (*pFeatureParaLen>=sizeof(NVRAM_SENSOR_DATA_STRUCT))
            {
                pSensorDefaultData->Version=NVRAM_CAMERA_SENSOR_FILE_VERSION;
                pSensorDefaultData->SensorId=SP5408MIPI_SENSOR_ID;
                memcpy(pSensorDefaultData->SensorEngReg, SP5408MIPISensorReg, sizeof(SENSOR_REG_STRUCT)*ENGINEER_END);
                memcpy(pSensorDefaultData->SensorCCTReg, SP5408MIPISensorCCT, sizeof(SENSOR_REG_STRUCT)*FACTORY_END_ADDR);
            }
            else
                return FALSE;
            *pFeatureParaLen=sizeof(NVRAM_SENSOR_DATA_STRUCT);
            break;
        case SENSOR_FEATURE_GET_CONFIG_PARA:
            memcpy(pSensorConfigData, &SP5408MIPISensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
            *pFeatureParaLen=sizeof(MSDK_SENSOR_CONFIG_STRUCT);
            break;
        case SENSOR_FEATURE_CAMERA_PARA_TO_SENSOR:
            SP5408MIPI_camera_para_to_sensor();
            break;

        case SENSOR_FEATURE_SENSOR_TO_CAMERA_PARA:
            SP5408MIPI_sensor_to_camera_para();
            break;
        case SENSOR_FEATURE_GET_GROUP_COUNT:
            *pFeatureReturnPara32++=SP5408MIPI_get_sensor_group_count();
            *pFeatureParaLen=4;
            break;
        case SENSOR_FEATURE_GET_GROUP_INFO:
            SP5408MIPI_get_sensor_group_info(pSensorGroupInfo->GroupIdx, pSensorGroupInfo->GroupNamePtr, &pSensorGroupInfo->ItemCount);
            *pFeatureParaLen=sizeof(MSDK_SENSOR_GROUP_INFO_STRUCT);
            break;
        case SENSOR_FEATURE_GET_ITEM_INFO:
            SP5408MIPI_get_sensor_item_info(pSensorItemInfo->GroupIdx,pSensorItemInfo->ItemIdx, pSensorItemInfo);
            *pFeatureParaLen=sizeof(MSDK_SENSOR_ITEM_INFO_STRUCT);
            break;

        case SENSOR_FEATURE_SET_ITEM_INFO:
            SP5408MIPI_set_sensor_item_info(pSensorItemInfo->GroupIdx, pSensorItemInfo->ItemIdx, pSensorItemInfo->ItemValue);
            *pFeatureParaLen=sizeof(MSDK_SENSOR_ITEM_INFO_STRUCT);
            break;

        case SENSOR_FEATURE_GET_ENG_INFO:
            pSensorEngInfo->SensorId = 129;
            pSensorEngInfo->SensorType = CMOS_SENSOR;
            pSensorEngInfo->SensorOutputDataFormat=SENSOR_OUTPUT_FORMAT_RAW_R;//hanlei
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
            SP5408MIPISetVideoMode(*pFeatureData16);
            break;
        case SENSOR_FEATURE_CHECK_SENSOR_ID:
            SP5408MIPIGetSensorID(pFeatureReturnPara32); 
            break;             
        case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE:
            SP5408MIPISetAutoFlickerMode((BOOL)*pFeatureData16, *(pFeatureData16+1));            
	        break;
        case SENSOR_FEATURE_SET_TEST_PATTERN:
            SP5408MIPISetTestPatternMode((BOOL)*pFeatureData16);        	
            break;
		case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
			SP5408MIPISetMaxFramerateByScenario((MSDK_SCENARIO_ID_ENUM)*pFeatureData32, *(pFeatureData32+1));
			break;
		case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
			SP5408MIPIGetDefaultFramerateByScenario((MSDK_SCENARIO_ID_ENUM)*pFeatureData32, (MUINT32 *)(*(pFeatureData32+1)));
			break;
        default:
            break;
    }
    return ERROR_NONE;
}	/* SP5408MIPIFeatureControl() */


SENSOR_FUNCTION_STRUCT	SensorFuncSP5408MIPI=
{
    SP5408MIPIOpen,
    SP5408MIPIGetInfo,
    SP5408MIPIGetResolution,
    SP5408MIPIFeatureControl,
    SP5408MIPIControl,
    SP5408MIPIClose
};

UINT32 SP5408_MIPI_RAW_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
    /* To Do : Check Sensor status here */
    if (pfFunc!=NULL)
        *pfFunc=&SensorFuncSP5408MIPI;
    return ERROR_NONE;
}   /* SensorInit() */

