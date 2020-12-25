/*****************************************************************************
 *
 * Filename:
 * ---------
 *   sensor.c
 *
 * Project:
 * --------

 *
 * Description:
 * ------------
 *   Source code of Sensor driver
 *
 *
 * Author:
 * -------

 *
 *============================================================================
 *             HISTORY
 * Below this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *------------------------------------------------------------------------------
 * $Revision:$
 * $Modtime:$
 * $Log:$
 *
 *
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/
#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/xlog.h>

#include <asm/atomic.h>
#include <asm/io.h>
#include <asm/system.h>

#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"
#include "kd_camera_feature.h"
   
#include "sp0a20yuv_Sensor.h"
#include "sp0a20yuv_Camera_Sensor_para.h"
#include "sp0a20yuv_CameraCustomized.h"

#define SP0A20MIPI_DEBUG
#ifdef SP0A20MIPI_DEBUG
#define SENSORDB(fmt, arg...) xlog_printk(ANDROID_LOG_DEBUG, "[SP0A20MIPI]", fmt, ##arg)

#else
#define SENSORDB(x,...)
#endif

typedef struct
{
  UINT16  iSensorVersion;
  UINT16  iNightMode;
  UINT16  iWB;
  UINT16  iEffect;
  UINT16  iEV;
  UINT16  iBanding;
  UINT16  iMirror;
  UINT16  iFrameRate;
} SP0A20MIPIStatus;

SP0A20MIPIStatus SP0A20MIPICurrentStatus;
static DEFINE_SPINLOCK(sp0A20mipi_yuv_drv_lock);
extern int iReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 * a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
extern int iWriteRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u16 i2cId);
//static int sensor_id_fail = 0; 
static kal_uint32 zoom_factor = 0; 


/*kal_uint8 SP0A20MIPI_read_cmos_sensor(kal_uint32 addr)
{
	kal_uint8 get_byte=0;
	char puSendCmd[2] = {(char)(addr >> 8) , (char)(addr & 0xFF) };
	iReadRegI2C(puSendCmd , 2, (u8*)&get_byte,1,SP0A20MIPI_WRITE_ID);

	 SENSORDB("SP0A20_read_cmos_sensor, addr:%x;get_byte:%x \n",addr,get_byte);
	
	return get_byte;
}*/

 /*inline void SP0A20_write_cmos_sensor(u16 addr, u32 para)
{
   char puSendCmd[3] = {(char)(addr >> 8) , (char)(addr & 0xFF) ,(char)(para & 0xFF)};
   iWriteRegI2C(puSendCmd , 3,SP0A20MIPI_WRITE_ID);
    //SENSORDB("SP0A20_write_cmos_sensor, addr:%x; para:%x \n",addr,para);
} */

static kal_uint8 SP0A20MIPI_read_cmos_sensor(kal_uint8 addr)
{
  kal_uint8 in_buff[1] = {0xFF};
  kal_uint8 out_buff[1];
  
  out_buff[0] = addr;

    if (0 != iReadRegI2C((u8*)out_buff , (u16) sizeof(out_buff), (u8*)in_buff, (u16) sizeof(in_buff), SP0A20MIPI_READ_ID)) {
        SENSORDB("ERROR: SP2529MIPI_read_cmos_sensor \n");
    }
  return in_buff[0];

}	

static void SP0A20_write_cmos_sensor(kal_uint8 addr, kal_uint8 para)
{
kal_uint8 out_buff[2];

    out_buff[0] = addr;
    out_buff[1] = para;

    iWriteRegI2C((u8*)out_buff , (u16)sizeof(out_buff), SP0A20MIPI_WRITE_ID); 
}	
	

/*******************************************************************************
* // Adapter for Winmo typedef 
********************************************************************************/
#define WINMO_USE 0
#define Sleep(ms) mdelay(ms)
#define RETAILMSG(x,...)
#define TEXT
/*******************************************************************************
* // End Adapter for Winmo typedef 
********************************************************************************/
/* Global Valuable */

kal_bool SP0A20MIPI_MPEG4_encode_mode = KAL_FALSE, SP0A20MIPI_MJPEG_encode_mode = KAL_FALSE;
static kal_bool   SP0A20MIPI_VEDIO_encode_mode = KAL_FALSE; 
static kal_bool   SP0A20MIPI_sensor_cap_state = KAL_FALSE; 
static kal_uint32 SP0A20MIPI_sensor_pclk=960;//720;//hanlei
static kal_bool   SP0A20MIPI_AE_ENABLE = KAL_TRUE; 
MSDK_SENSOR_CONFIG_STRUCT SP0A20MIPISensorConfigData;

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
 
    fp = filp_open("/mnt/sdcard/sp0A20_sd", O_RDONLY , 0); 
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
*	SP0A20MIPIInitialPara
*
* DESCRIPTION
*	This function initialize the global status of  MT9V114
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
static void SP0A20MIPIInitialPara(void)
{
  spin_lock(&sp0A20mipi_yuv_drv_lock);
  SP0A20MIPICurrentStatus.iNightMode = 0;
  SP0A20MIPICurrentStatus.iWB = AWB_MODE_AUTO;
  SP0A20MIPICurrentStatus.iEffect = MEFFECT_OFF;
  SP0A20MIPICurrentStatus.iBanding = AE_FLICKER_MODE_50HZ;
  SP0A20MIPICurrentStatus.iEV = AE_EV_COMP_n03;
  SP0A20MIPICurrentStatus.iMirror = IMAGE_NORMAL;
  SP0A20MIPICurrentStatus.iFrameRate = 0;
  spin_unlock(&sp0A20mipi_yuv_drv_lock);
}


void SP0A20MIPI_set_mirror(kal_uint8 image_mirror)
{

		if(SP0A20MIPICurrentStatus.iMirror == image_mirror)
		  return;
		//rotation 180 case
		switch (image_mirror)  
		{
			case IMAGE_NORMAL:
				//SP0A20_write_cmos_sensor(0x0101, 0x03); 
				break;
			case IMAGE_H_MIRROR:
				//SP0A20_write_cmos_sensor(0x0101, 0x02);  //mirror : bit1	
				break;
			case IMAGE_V_MIRROR:
				//SP0A20_write_cmos_sensor(0x0101, 0x01);  //mirror : bit 0 
				break;
			case IMAGE_HV_MIRROR:
				//SP0A20_write_cmos_sensor(0x0101, 0x00);
				break;
			default:
				ASSERT(0);
				break;
		} 

		/* //general case.
		switch (image_mirror)  
		{
			case IMAGE_NORMAL:
				SP0A20_write_cmos_sensor(0x0101, 0x00);
				break;
			case IMAGE_H_MIRROR:
				SP0A20_write_cmos_sensor(0x0101, 0x01);  //mirror : bit 0 
				break;
			case IMAGE_V_MIRROR:
				SP0A20_write_cmos_sensor(0x0101, 0x02);  //mirror : bit1	
				break;
			case IMAGE_HV_MIRROR:
				SP0A20_write_cmos_sensor(0x0101, 0x03); 	
				break;
			default:
				ASSERT(0);
				break;
		} */
		spin_lock(&sp0A20mipi_yuv_drv_lock);
		SP0A20MIPICurrentStatus.iMirror = image_mirror;
		spin_unlock(&sp0A20mipi_yuv_drv_lock);

}





/*****************************************************************************
 * FUNCTION
 *  SP0A20MIPI_set_dummy
 * DESCRIPTION
 *
 * PARAMETERS
 *  pixels      [IN]
 *  lines       [IN]
 * RETURNS
 *  void
 *****************************************************************************/
void SP0A20MIPI_set_dummy(kal_uint16 dummy_pixels, kal_uint16 dummy_lines)
{
		/****************************************************
		  * Adjust the extra H-Blanking & V-Blanking.
		  *****************************************************/
		//SP0A20_write_cmos_sensor(0x0340, (dummy_lines>>8)  ); 	// Extra V-Blanking
		//SP0A20_write_cmos_sensor(0x0341, (dummy_lines&0xFF)); 	// Extra V-Blanking
		//SP0A20_write_cmos_sensor(0x0342, (dummy_pixels>>8)	);	// Extra HBlanking
		//SP0A20_write_cmos_sensor(0x0343, (dummy_pixels&0xFF));	// Extra H-Blanking


}   /* SP0A20MIPI_set_dummy */

/*****************************************************************************
 * FUNCTION
 *  SP0A20MIPI_Initialize_Setting
 * DESCRIPTION
 *    SP0A20 DVP 1280x720 Full at 30FPS in YUV format
 *    24Mhz input, system clock 76Mhz
 * PARAMETERS
 *  void
 * RETURNS
 *  void
 *****************************************************************************/
void SP0A20MIPI_Initialize_Setting(void)
{
  SP0A20_write_cmos_sensor(0xfd , 0x00);
  SP0A20_write_cmos_sensor(0x0c , 0x00);
  SP0A20_write_cmos_sensor(0x12 , 0x02);
  SP0A20_write_cmos_sensor(0x13 , 0x2f);
  SP0A20_write_cmos_sensor(0x6d , 0x32);
  SP0A20_write_cmos_sensor(0x6c , 0x32);
  SP0A20_write_cmos_sensor(0x6f , 0x33);
  SP0A20_write_cmos_sensor(0x6e , 0x34);
  SP0A20_write_cmos_sensor(0x92 , 0x01);//11
  SP0A20_write_cmos_sensor(0x99 , 0x05);
  SP0A20_write_cmos_sensor(0x16 , 0x38);
  SP0A20_write_cmos_sensor(0x17 , 0x38);
  SP0A20_write_cmos_sensor(0x70 , 0x3a);
  SP0A20_write_cmos_sensor(0x14 , 0x02);
  SP0A20_write_cmos_sensor(0x15 , 0x20);
  SP0A20_write_cmos_sensor(0x71 , 0x23);
  SP0A20_write_cmos_sensor(0x69 , 0x25);
  SP0A20_write_cmos_sensor(0x6a , 0x1a);
  SP0A20_write_cmos_sensor(0x72 , 0x1c);
  SP0A20_write_cmos_sensor(0x75 , 0x1e);
  SP0A20_write_cmos_sensor(0x73 , 0x3c);
  SP0A20_write_cmos_sensor(0x74 , 0x21);
  SP0A20_write_cmos_sensor(0x79 , 0x00);
  SP0A20_write_cmos_sensor(0x77 , 0x10);
  SP0A20_write_cmos_sensor(0x1a , 0x4d);
  SP0A20_write_cmos_sensor(0x1b , 0x27);
  SP0A20_write_cmos_sensor(0x1c , 0x07);
  SP0A20_write_cmos_sensor(0x1e , 0x01);
  SP0A20_write_cmos_sensor(0x21 , 0x10);
  SP0A20_write_cmos_sensor(0x22 , 0x28);
  SP0A20_write_cmos_sensor(0x26 , 0x66);
  SP0A20_write_cmos_sensor(0x28 , 0x0b);
  SP0A20_write_cmos_sensor(0x37 , 0x5a);
  
//pre_gain
  SP0A20_write_cmos_sensor(0xfd , 0x02);
  SP0A20_write_cmos_sensor(0x01 , 0x80);
  
//blacklevel
  SP0A20_write_cmos_sensor(0xfd , 0x01);
  SP0A20_write_cmos_sensor(0x41 , 0x00);
  SP0A20_write_cmos_sensor(0x42 , 0x00);
  SP0A20_write_cmos_sensor(0x43 , 0x00);
  SP0A20_write_cmos_sensor(0x44 , 0x00);
/*
//ae setting 8-24fps
  SP0A20_write_cmos_sensor(0xfd , 0x00);
  SP0A20_write_cmos_sensor(0x03 , 0x02);
  SP0A20_write_cmos_sensor(0x04 , 0xd0);
  SP0A20_write_cmos_sensor(0x05 , 0x00);
  SP0A20_write_cmos_sensor(0x06 , 0x00);
  SP0A20_write_cmos_sensor(0x07 , 0x00);
  SP0A20_write_cmos_sensor(0x08 , 0x00);
  SP0A20_write_cmos_sensor(0x09 , 0x00);
  SP0A20_write_cmos_sensor(0x0a , 0x9c);
  SP0A20_write_cmos_sensor(0xfd , 0x01);
  SP0A20_write_cmos_sensor(0xf0 , 0x00);
  SP0A20_write_cmos_sensor(0xf7 , 0x78);
  SP0A20_write_cmos_sensor(0x02 , 0x0c);
  SP0A20_write_cmos_sensor(0x03 , 0x01);
  SP0A20_write_cmos_sensor(0x06 , 0x78);
  SP0A20_write_cmos_sensor(0x07 , 0x00);
  SP0A20_write_cmos_sensor(0x08 , 0x01);
  SP0A20_write_cmos_sensor(0x09 , 0x00);
  SP0A20_write_cmos_sensor(0xfd , 0x02);
  SP0A20_write_cmos_sensor(0xbe , 0xa0);
  SP0A20_write_cmos_sensor(0xbf , 0x05);
  SP0A20_write_cmos_sensor(0xd0 , 0xa0);
  SP0A20_write_cmos_sensor(0xd1 , 0x05);
*/
/*
	//sp0A20 24M 50Hz 1分频 15.0301-8fps AE_Parameters_20140226095728.txt

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
	SP0A20_write_cmos_sensor(0x02,0x0c);
	SP0A20_write_cmos_sensor(0x03,0x01);
	SP0A20_write_cmos_sensor(0x06,0x4b);
	SP0A20_write_cmos_sensor(0x07,0x00);
	SP0A20_write_cmos_sensor(0x08,0x01);
	SP0A20_write_cmos_sensor(0x09,0x00);
	SP0A20_write_cmos_sensor(0xfd,0x02);
	SP0A20_write_cmos_sensor(0xbe,0x84);
	SP0A20_write_cmos_sensor(0xbf,0x03);
	SP0A20_write_cmos_sensor(0xd0,0x84);
	SP0A20_write_cmos_sensor(0xd1,0x03);
	//sp0A20 24M 50Hz 1分频 13-8fps AE_Parameters
	SP0A20_write_cmos_sensor(0xfd,0x00);
	SP0A20_write_cmos_sensor(0x03,0x01);
	SP0A20_write_cmos_sensor(0x04,0x86);
	SP0A20_write_cmos_sensor(0x05,0x00);
	SP0A20_write_cmos_sensor(0x06,0x00);
	SP0A20_write_cmos_sensor(0x07,0x00);
	SP0A20_write_cmos_sensor(0x08,0x00);
	SP0A20_write_cmos_sensor(0x09,0x03);
	SP0A20_write_cmos_sensor(0x0a,0xea);
	SP0A20_write_cmos_sensor(0xfd,0x01);
	SP0A20_write_cmos_sensor(0xf0,0x00);
	SP0A20_write_cmos_sensor(0xf7,0x41);
	SP0A20_write_cmos_sensor(0x02,0x0c);
	SP0A20_write_cmos_sensor(0x03,0x01);
	SP0A20_write_cmos_sensor(0x06,0x41);
	SP0A20_write_cmos_sensor(0x07,0x00);
	SP0A20_write_cmos_sensor(0x08,0x01);
	SP0A20_write_cmos_sensor(0x09,0x00);
	SP0A20_write_cmos_sensor(0xfd,0x02);
	SP0A20_write_cmos_sensor(0xbe,0x0c);
	SP0A20_write_cmos_sensor(0xbf,0x03);
	SP0A20_write_cmos_sensor(0xd0,0x0c);
	SP0A20_write_cmos_sensor(0xd1,0x03);
*/
//sp0A20 24M 50Hz 1分频 10-8fps AE_Parameters
SP0A20_write_cmos_sensor(0xfd,0x00);
SP0A20_write_cmos_sensor(0x03,0x01);
SP0A20_write_cmos_sensor(0x04,0x2c);
SP0A20_write_cmos_sensor(0x05,0x00);
SP0A20_write_cmos_sensor(0x06,0x00);
SP0A20_write_cmos_sensor(0x07,0x00);
SP0A20_write_cmos_sensor(0x08,0x00);
SP0A20_write_cmos_sensor(0x09,0x01);
SP0A20_write_cmos_sensor(0x0a,0x64);
SP0A20_write_cmos_sensor(0xfd,0x01);
SP0A20_write_cmos_sensor(0xf0,0x00);
SP0A20_write_cmos_sensor(0xf7,0x32);
SP0A20_write_cmos_sensor(0x02,0x10);
SP0A20_write_cmos_sensor(0x03,0x01);
SP0A20_write_cmos_sensor(0x06,0x32);
SP0A20_write_cmos_sensor(0x07,0x00);
SP0A20_write_cmos_sensor(0x08,0x01);
SP0A20_write_cmos_sensor(0x09,0x00);
SP0A20_write_cmos_sensor(0xfd,0x02);
SP0A20_write_cmos_sensor(0xbe,0x20);
SP0A20_write_cmos_sensor(0xbf,0x03);
SP0A20_write_cmos_sensor(0xd0,0x20);
SP0A20_write_cmos_sensor(0xd1,0x03);

//ae gain &status
  SP0A20_write_cmos_sensor(0xfd , 0x01);
  SP0A20_write_cmos_sensor(0x5a , 0x40);//dp rpc   
  SP0A20_write_cmos_sensor(0xfd , 0x02); 
  SP0A20_write_cmos_sensor(0xbc , 0x70);//rpc_heq_low
  SP0A20_write_cmos_sensor(0xbd , 0x50);//rpc_heq_dummy
  SP0A20_write_cmos_sensor(0xb8 , 0x66);//mean_normal_dummy
  SP0A20_write_cmos_sensor(0xb9 , 0x8f);//mean_dummy_normal
  SP0A20_write_cmos_sensor(0xba , 0x30);//mean_dummy_low
  SP0A20_write_cmos_sensor(0xbb , 0x45);//mean low_dummy
  
  //rpc
  SP0A20_write_cmos_sensor(0xfd , 0x01);                
  SP0A20_write_cmos_sensor(0xe0 , 0x44);//0x4c//rpc_1base_max
  SP0A20_write_cmos_sensor(0xe1 , 0x36);//0x3c//rpc_2base_max
  SP0A20_write_cmos_sensor(0xe2 , 0x30);//0x34//rpc_3base_max
  SP0A20_write_cmos_sensor(0xe3 , 0x2a);//0x2e//rpc_4base_max
  SP0A20_write_cmos_sensor(0xe4 , 0x2a);//0x2e//rpc_5base_max
  SP0A20_write_cmos_sensor(0xe5 , 0x28);//0x2c//rpc_6base_max
  SP0A20_write_cmos_sensor(0xe6 , 0x28);//0x2c//rpc_7base_max
  SP0A20_write_cmos_sensor(0xe7 , 0x26);//0x2a//rpc_8base_max
  SP0A20_write_cmos_sensor(0xe8 , 0x26);//0x2a//rpc_9base_max
  SP0A20_write_cmos_sensor(0xe9 , 0x26);//0x2a//rpc_10base_max
  SP0A20_write_cmos_sensor(0xea , 0x24);//0x28//rpc_11base_max
  SP0A20_write_cmos_sensor(0xf3 , 0x24);//0x28//rpc_12base_max
  SP0A20_write_cmos_sensor(0xf4 , 0x24);//0x28//rpc_13base_max
//ae min gain  
  SP0A20_write_cmos_sensor(0xfd , 0x01);
  SP0A20_write_cmos_sensor(0x04 , 0xa0);//rpc_max_indr
  SP0A20_write_cmos_sensor(0x05 , 0x24);//rpc_min_indr 
  SP0A20_write_cmos_sensor(0x0a , 0xa0);//rpc_max_outdr
  SP0A20_write_cmos_sensor(0x0b , 0x24);//rpc_min_outdr
  
//target
  SP0A20_write_cmos_sensor(0xfd , 0x01);
  SP0A20_write_cmos_sensor(0xeb , 0x70);//78//target indr
  SP0A20_write_cmos_sensor(0xec , 0x70);//78//target outdr
  SP0A20_write_cmos_sensor(0xed , 0x05);//lock range
  SP0A20_write_cmos_sensor(0xee , 0x0a);//hold range

  
//??
  SP0A20_write_cmos_sensor(0xfd , 0x01);
  SP0A20_write_cmos_sensor(0xf2 , 0x4d);
  SP0A20_write_cmos_sensor(0xfd , 0x02);
  SP0A20_write_cmos_sensor(0x54 , 0x0a);
  SP0A20_write_cmos_sensor(0x5b , 0x05);//dp status
  SP0A20_write_cmos_sensor(0x5c , 0xa0);
  
//lens shading
  SP0A20_write_cmos_sensor(0xfd , 0x01);
  SP0A20_write_cmos_sensor(0x26 , 0x80);
  SP0A20_write_cmos_sensor(0x26 , 0x4f);
  SP0A20_write_cmos_sensor(0x28 , 0x00);
  SP0A20_write_cmos_sensor(0x29 , 0x20);
  SP0A20_write_cmos_sensor(0x2a , 0x00);
  SP0A20_write_cmos_sensor(0x2b , 0x03);
  SP0A20_write_cmos_sensor(0x2c , 0x00);
  SP0A20_write_cmos_sensor(0x2d , 0x20);
  SP0A20_write_cmos_sensor(0x30 , 0x00);
  SP0A20_write_cmos_sensor(0x31 , 0x00);
 
//lsc 1
  SP0A20_write_cmos_sensor(0xfd , 0x01);
  SP0A20_write_cmos_sensor(0xa1 , 0x12);//15//r
  SP0A20_write_cmos_sensor(0xa2 , 0x1a);//17
  SP0A20_write_cmos_sensor(0xa3 , 0x11);//11
  SP0A20_write_cmos_sensor(0xa4 , 0x0d);//10
  SP0A20_write_cmos_sensor(0xa5 , 0x0f);//14//g
  SP0A20_write_cmos_sensor(0xa6 , 0x15);//14
  SP0A20_write_cmos_sensor(0xa7 , 0x11);//11
  SP0A20_write_cmos_sensor(0xa8 , 0x0c);//10
  SP0A20_write_cmos_sensor(0xa9 , 0x0e);//13//b
  SP0A20_write_cmos_sensor(0xaa , 0x14);//13
  SP0A20_write_cmos_sensor(0xab , 0x0f);//0f
  SP0A20_write_cmos_sensor(0xac , 0x0c);//10
  SP0A20_write_cmos_sensor(0xad , 0x0c);//00//r
  SP0A20_write_cmos_sensor(0xae , 0x06);//06
  SP0A20_write_cmos_sensor(0xaf , 0x08);//06
  SP0A20_write_cmos_sensor(0xb0 , 0x06);//06
  SP0A20_write_cmos_sensor(0xb1 , 0x04);//06//g
  SP0A20_write_cmos_sensor(0xb2 , 0x00);//00
  SP0A20_write_cmos_sensor(0xb3 , 0x03);//03
  SP0A20_write_cmos_sensor(0xb4 , 0x00);//00
  SP0A20_write_cmos_sensor(0xb5 , 0x06);//08//b
  SP0A20_write_cmos_sensor(0xb6 , 0x00);//00
  SP0A20_write_cmos_sensor(0xb7 , 0x00);//00
  SP0A20_write_cmos_sensor(0xb8 , 0x00);//00
  SP0A20_write_cmos_sensor(0xfd , 0x00);
  
//skin detect
  SP0A20_write_cmos_sensor(0xfd , 0x02);
  SP0A20_write_cmos_sensor(0x09 , 0x09);
  SP0A20_write_cmos_sensor(0x0d , 0x1a);
  SP0A20_write_cmos_sensor(0x1d , 0x03);
  SP0A20_write_cmos_sensor(0x1f , 0x04);  
//awb
  SP0A20_write_cmos_sensor(0xfd , 0x01);
  SP0A20_write_cmos_sensor(0x32 , 0x00);
  SP0A20_write_cmos_sensor(0xfd , 0x02);
  SP0A20_write_cmos_sensor(0x26 , 0xbf);
  SP0A20_write_cmos_sensor(0x27 , 0xa3);
  SP0A20_write_cmos_sensor(0x10 , 0x00);
  SP0A20_write_cmos_sensor(0x11 , 0x00);
  SP0A20_write_cmos_sensor(0x1b , 0x80);
  SP0A20_write_cmos_sensor(0x1a , 0x80);
  SP0A20_write_cmos_sensor(0x18 , 0x27);
  SP0A20_write_cmos_sensor(0x19 , 0x26);
  SP0A20_write_cmos_sensor(0x2a , 0x01);
  SP0A20_write_cmos_sensor(0x2b , 0x10);
  SP0A20_write_cmos_sensor(0x28 , 0xf8);
  SP0A20_write_cmos_sensor(0x29 , 0x08);

//d65 10
  SP0A20_write_cmos_sensor(0x66 , 0x4b);//59
  SP0A20_write_cmos_sensor(0x67 , 0x6b);//81
  SP0A20_write_cmos_sensor(0x68 , 0xcd);
  SP0A20_write_cmos_sensor(0x69 , 0xe7);//ee
  SP0A20_write_cmos_sensor(0x6a , 0xa5);

//indoor 11
  SP0A20_write_cmos_sensor(0x7c , 0x38);//47
  SP0A20_write_cmos_sensor(0x7d , 0x60);//64
  SP0A20_write_cmos_sensor(0x7e , 0xf0);//f9
  SP0A20_write_cmos_sensor(0x7f , 0x10);//12
  SP0A20_write_cmos_sensor(0x80 , 0xa6);

//cwf   12
  SP0A20_write_cmos_sensor(0x70 , 0x2d);//38
  SP0A20_write_cmos_sensor(0x71 , 0x4f);//61
  SP0A20_write_cmos_sensor(0x72 , 0x1d);//23
  SP0A20_write_cmos_sensor(0x73 , 0x37);//41
  SP0A20_write_cmos_sensor(0x74 , 0xaa);
 
//tl84  13
  SP0A20_write_cmos_sensor(0x6b , 0x11);//10
  SP0A20_write_cmos_sensor(0x6c , 0x2d);//38
  SP0A20_write_cmos_sensor(0x6d , 0x22);//2a
  SP0A20_write_cmos_sensor(0x6e , 0x43);//46
  SP0A20_write_cmos_sensor(0x6f , 0xaa);
 
//f    14
  SP0A20_write_cmos_sensor(0x61 , 0x04);//11
  SP0A20_write_cmos_sensor(0x62 , 0x20);//3a
  SP0A20_write_cmos_sensor(0x63 , 0x38);//48
  SP0A20_write_cmos_sensor(0x64 , 0x60);//5f
  SP0A20_write_cmos_sensor(0x65 , 0xaa);
 
  SP0A20_write_cmos_sensor(0x75 , 0x80);
  SP0A20_write_cmos_sensor(0x76 , 0x09);
  SP0A20_write_cmos_sensor(0x77 , 0x02);
  SP0A20_write_cmos_sensor(0x24 , 0x25);
  SP0A20_write_cmos_sensor(0x0e , 0x16);
  SP0A20_write_cmos_sensor(0x3b , 0x09);
  SP0A20_write_cmos_sensor(0xfd , 0x02);
  SP0A20_write_cmos_sensor(0x08 , 0x00);
  SP0A20_write_cmos_sensor(0x09 , 0x06);

// sharp

  SP0A20_write_cmos_sensor(0xfd , 0x02);
  SP0A20_write_cmos_sensor(0xde , 0x0f);
  
  SP0A20_write_cmos_sensor(0xd7 , 0x06); //sharp_flat_thr 轮廓判断
  SP0A20_write_cmos_sensor(0xd8 , 0x08);
  SP0A20_write_cmos_sensor(0xd9 , 0x0b);
  SP0A20_write_cmos_sensor(0xda , 0x0f);
  SP0A20_write_cmos_sensor(0xe8 , 0x38); //sharp_fac_pos 轮廓强度
  SP0A20_write_cmos_sensor(0xe9 , 0x38);//36
  SP0A20_write_cmos_sensor(0xea , 0x30);//24
  SP0A20_write_cmos_sensor(0xeb , 0x24);
  SP0A20_write_cmos_sensor(0xec , 0x38);//18 //sharp_fac_neg
  SP0A20_write_cmos_sensor(0xed , 0x38);//36//18
  SP0A20_write_cmos_sensor(0xee , 0x30);//24
  SP0A20_write_cmos_sensor(0xef , 0x24);
  
  SP0A20_write_cmos_sensor(0xd3 , 0x24);// sharp_ofst_pos
  SP0A20_write_cmos_sensor(0xd4 , 0x48);// sharp_ofst_neg
  SP0A20_write_cmos_sensor(0xd5 , 0x20);// sharp_ofst_min
  SP0A20_write_cmos_sensor(0xd6 , 0x08);// sharp_k_val
  

  
//skin sharpen        
  SP0A20_write_cmos_sensor(0xfd , 0x01);
  SP0A20_write_cmos_sensor(0xd1 , 0x20); //skin_sharp_delta
  SP0A20_write_cmos_sensor(0xfd , 0x02);
  SP0A20_write_cmos_sensor(0xdc , 0x05); //肤色降锐化 skin_sharp_sel
  SP0A20_write_cmos_sensor(0x05 , 0x20); //排除肤色降锐化对分辨率卡引起的干扰
//  0x09 , 0x10 //肤色排除白点区域
    
//BPC
  SP0A20_write_cmos_sensor(0xfd , 0x02);
  SP0A20_write_cmos_sensor(0x81 , 0x00); // bpc_ratio_vt
  SP0A20_write_cmos_sensor(0xfd , 0x01);
  SP0A20_write_cmos_sensor(0xfc , 0x00); // bpc_median_en
  SP0A20_write_cmos_sensor(0x7d , 0x05); // bpc_med_thr
  SP0A20_write_cmos_sensor(0x7e , 0x05);
  SP0A20_write_cmos_sensor(0x7f , 0x09);
  SP0A20_write_cmos_sensor(0x80 , 0x08);
  
//dns
  SP0A20_write_cmos_sensor(0xfd , 0x02); 
  SP0A20_write_cmos_sensor(0xdd , 0x0f); //enable
  SP0A20_write_cmos_sensor(0xfd , 0x01);  
  SP0A20_write_cmos_sensor(0x86 , 0x20); //沿方向边缘平滑阈值，越小越弱
  
  SP0A20_write_cmos_sensor(0x6d , 0x0d); //0d//dns_flat_dif 强平滑（平坦）区域平滑阈值
  SP0A20_write_cmos_sensor(0x6e , 0x26); //16
  SP0A20_write_cmos_sensor(0x6f , 0x2a); //1a
  SP0A20_write_cmos_sensor(0x70 , 0x28); //20
  SP0A20_write_cmos_sensor(0x86 , 0x20);//dark
  SP0A20_write_cmos_sensor(0x71 , 0x08); //dns_edge_dif 弱轮廓（非平坦）区域平滑阈值	
  SP0A20_write_cmos_sensor(0x72 , 0x0b);
  SP0A20_write_cmos_sensor(0x73 , 0x0e);
  SP0A20_write_cmos_sensor(0x74 , 0x12);
  
  SP0A20_write_cmos_sensor(0x75 , 0x08); //dns_edge_gdif
  SP0A20_write_cmos_sensor(0x76 , 0x0b); 
  SP0A20_write_cmos_sensor(0x77 , 0x0e); 
  SP0A20_write_cmos_sensor(0x78 , 0x12); 
  SP0A20_write_cmos_sensor(0x79 , 0x24); //45//{raw_flat_fac, raw_edge_fac}  
  SP0A20_write_cmos_sensor(0x7a , 0x04); //45
  SP0A20_write_cmos_sensor(0x7b , 0x03); //34
  SP0A20_write_cmos_sensor(0x7c , 0x03); //14
    
  SP0A20_write_cmos_sensor(0x81 , 0x0a); //2x//dns_flat_thr 根据增益判定区域阈值
  SP0A20_write_cmos_sensor(0x82 , 0x0f); //4x
  SP0A20_write_cmos_sensor(0x83 , 0x1d); //8x
  SP0A20_write_cmos_sensor(0x84 , 0x1d); //16x
//dem
  SP0A20_write_cmos_sensor(0xfd , 0x02);  
  SP0A20_write_cmos_sensor(0x83 , 0x14); //{dem_morie_thr, dem_hfmax_thr}
  SP0A20_write_cmos_sensor(0x84 , 0x18); //dem_grad_thr 
  SP0A20_write_cmos_sensor(0x86 , 0x04); //dem_grad_dif
//pf
  SP0A20_write_cmos_sensor(0xfd , 0x01);
  SP0A20_write_cmos_sensor(0x61 , 0x60);
  SP0A20_write_cmos_sensor(0x62 , 0x28);
  SP0A20_write_cmos_sensor(0x8a , 0x10);

//gamma  
  SP0A20_write_cmos_sensor(0xfd , 0x01);//01
  SP0A20_write_cmos_sensor(0x8b , 0x00);//00//00//0a19
  SP0A20_write_cmos_sensor(0x8c , 0x0a);//0a//0a//
  SP0A20_write_cmos_sensor(0x8d , 0x17);//17//19//
  SP0A20_write_cmos_sensor(0x8e , 0x23);//23//23//
  SP0A20_write_cmos_sensor(0x8f , 0x31);//2f//2d//
  SP0A20_write_cmos_sensor(0x90 , 0x47);//44//3e//
  SP0A20_write_cmos_sensor(0x91 , 0x58);//55//4f//
  SP0A20_write_cmos_sensor(0x92 , 0x65);//63//5c//
  SP0A20_write_cmos_sensor(0x93 , 0x72);//71//6a//
  SP0A20_write_cmos_sensor(0x94 , 0x86);//87//86//
  SP0A20_write_cmos_sensor(0x95 , 0x96);//96//9b//
  SP0A20_write_cmos_sensor(0x96 , 0xa4);//a4//ab//
  SP0A20_write_cmos_sensor(0x97 , 0xb1);//b0//b6//
  SP0A20_write_cmos_sensor(0x98 , 0xbb);//bb//be//
  SP0A20_write_cmos_sensor(0x99 , 0xc5);//c5//c5//
  SP0A20_write_cmos_sensor(0x9a , 0xce);//cf//cc//
  SP0A20_write_cmos_sensor(0x9b , 0xd7);//d8//D4//
  SP0A20_write_cmos_sensor(0x9c , 0xe1);//e0//db//
  SP0A20_write_cmos_sensor(0x9d , 0xe9);//e9//E3//
  SP0A20_write_cmos_sensor(0x9e , 0xf0);//f1//Eb//
  SP0A20_write_cmos_sensor(0x9f , 0xf8);//f8//F5//
  SP0A20_write_cmos_sensor(0xa0 , 0xff);//ff//FF//

//CCM
  SP0A20_write_cmos_sensor(0xfd , 0x02);
  SP0A20_write_cmos_sensor(0x15 , 0xc6);//d4//b>th
  SP0A20_write_cmos_sensor(0x16 , 0x92);//a1//r<th  
  //!F
  SP0A20_write_cmos_sensor(0xa0 , 0xa6);//SP0A20_write_cmos_sensor(红色接近，肤色不理想)
  SP0A20_write_cmos_sensor(0xa1 , 0xf4);//
  SP0A20_write_cmos_sensor(0xa2 , 0xe6);//
  SP0A20_write_cmos_sensor(0xa3 , 0xf4);//
  SP0A20_write_cmos_sensor(0xa4 , 0xb3);//
  SP0A20_write_cmos_sensor(0xa5 , 0xd9);//
  SP0A20_write_cmos_sensor(0xa6 , 0x06);//
  SP0A20_write_cmos_sensor(0xa7 , 0xcd);//
  SP0A20_write_cmos_sensor(0xa8 , 0xac);//
  SP0A20_write_cmos_sensor(0xa9 , 0x3c);//
  SP0A20_write_cmos_sensor(0xaa , 0x33);//
  SP0A20_write_cmos_sensor(0xab , 0x0c);//
           
  SP0A20_write_cmos_sensor(0xac , 0x8c);//
  SP0A20_write_cmos_sensor(0xad , 0x06);//
  SP0A20_write_cmos_sensor(0xae , 0xee);//
  SP0A20_write_cmos_sensor(0xaf , 0xe7);//
  SP0A20_write_cmos_sensor(0xb0 , 0xc0);//
  SP0A20_write_cmos_sensor(0xb1 , 0xda);//
  SP0A20_write_cmos_sensor(0xb2 , 0xe7);//
  SP0A20_write_cmos_sensor(0xb3 , 0xe1);//
  SP0A20_write_cmos_sensor(0xb4 , 0xb8);//
  SP0A20_write_cmos_sensor(0xb5 , 0x30);//
  SP0A20_write_cmos_sensor(0xb6 , 0x33);//
  SP0A20_write_cmos_sensor(0xb7 , 0x0f);//
  
         
//sat u 
  SP0A20_write_cmos_sensor(0xfd , 0x01);
  SP0A20_write_cmos_sensor(0xd3 , 0x78);//82
  SP0A20_write_cmos_sensor(0xd4 , 0x70);//7c
  SP0A20_write_cmos_sensor(0xd5 , 0x64);//68
  SP0A20_write_cmos_sensor(0xd6 , 0x50);//50
//sat v     
  SP0A20_write_cmos_sensor(0xd7 , 0x78);//82
  SP0A20_write_cmos_sensor(0xd8 , 0x70);//7c
  SP0A20_write_cmos_sensor(0xd9 , 0x64);//68
  SP0A20_write_cmos_sensor(0xda , 0x50);//50
//auto_sat
  SP0A20_write_cmos_sensor(0xfd , 0x01);
  SP0A20_write_cmos_sensor(0xdd , 0x30);
  SP0A20_write_cmos_sensor(0xde , 0x10);  
  SP0A20_write_cmos_sensor(0xdf , 0xff); //a0//y_mean_th
  SP0A20_write_cmos_sensor(0x00 , 0x00); //status

//uv_th
  SP0A20_write_cmos_sensor(0xfd , 0x01); //白色物体表面有彩色噪声降低此值
  SP0A20_write_cmos_sensor(0xc2 , 0xaa);
  SP0A20_write_cmos_sensor(0xc3 , 0x77);//aa
  SP0A20_write_cmos_sensor(0xc4 , 0x66);//66
  SP0A20_write_cmos_sensor(0xc5 , 0x44);//66

//low_lum_offset
  SP0A20_write_cmos_sensor(0xfd , 0x01);
  SP0A20_write_cmos_sensor(0xcd , 0x10);
  SP0A20_write_cmos_sensor(0xce , 0x1f);
  SP0A20_write_cmos_sensor(0xcf , 0x30);
  SP0A20_write_cmos_sensor(0xd0 , 0x45);

//gw
  SP0A20_write_cmos_sensor(0xfd , 0x02);
  SP0A20_write_cmos_sensor(0x31 , 0x60);
  SP0A20_write_cmos_sensor(0x32 , 0x60);
  SP0A20_write_cmos_sensor(0x33 , 0xc0);
  SP0A20_write_cmos_sensor(0x35 , 0x60);
  SP0A20_write_cmos_sensor(0x37 , 0x13);

//heq
  SP0A20_write_cmos_sensor(0xfd , 0x01);                         
  SP0A20_write_cmos_sensor(0x0e , 0x80);           
  SP0A20_write_cmos_sensor(0x0f , 0x20); //k_max
  SP0A20_write_cmos_sensor(0x10 , 0x68); //78//ku_outdoor
  SP0A20_write_cmos_sensor(0x11 , 0x68); //78//ku_nr
  SP0A20_write_cmos_sensor(0x12 , 0x68); //78//ku_dummy
  SP0A20_write_cmos_sensor(0x13 , 0x68); //78//ku_low
  SP0A20_write_cmos_sensor(0x14 , 0xa0);//84//c4 //kl_outdoor 
  SP0A20_write_cmos_sensor(0x15 , 0xa0);//84//c4 //kl_nr      
  SP0A20_write_cmos_sensor(0x16 , 0xa0);//84//c4 //kl_dummy    
  SP0A20_write_cmos_sensor(0x17 , 0xa0);//84//c4 //kl_low        

  SP0A20_write_cmos_sensor(0xfd , 0x00);
  SP0A20_write_cmos_sensor(0x31 , 0x00);

//auto 

  SP0A20_write_cmos_sensor(0xfd , 0x01);  
  SP0A20_write_cmos_sensor(0x32 , 0x15);
  SP0A20_write_cmos_sensor(0x33 , 0xef);
  SP0A20_write_cmos_sensor(0x34 , 0x07);
  SP0A20_write_cmos_sensor(0xd2 , 0x01); //{contrast sat}
  SP0A20_write_cmos_sensor(0xfb , 0x25); 
  SP0A20_write_cmos_sensor(0xf2 , 0x49);
  SP0A20_write_cmos_sensor(0x35 , 0x00);
  SP0A20_write_cmos_sensor(0x5d , 0x11);    

}

/*****************************************************************************
 * FUNCTION
 *  SP0A20MIPI_PV_Mode
 * DESCRIPTION
 *
 * PARAMETERS
 *  void
 * RETURNS
 *  void
 *****************************************************************************/
void SP0A20MIPI_PV_Mode(void)
{
//
}

/*****************************************************************************
 * FUNCTION
 *  SP0A20MIPI_CAP_Mode
 * DESCRIPTION
 *
 * PARAMETERS
 *  void
 * RETURNS
 *  void
 *****************************************************************************/

void SP0A20MIPI_CAP_Mode(void)
{
//
}

static void SP0A20MIPI_set_AE_mode(kal_bool AE_enable)
{
     if(AE_enable==KAL_TRUE)
     {
     	//SP0A20_write_cmos_sensor(0x3503, 0x00);  
     }
     else
     {
     	//SP0A20_write_cmos_sensor(0x3503, 0x03);  
     }
}


/*************************************************************************
* FUNCTION
*	SP0A20MIPI_night_mode
*
* DESCRIPTION
*	This function night mode of SP0A20MIPI.
*
* PARAMETERS
*	none
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
void SP0A20MIPI_night_mode(kal_bool enable)
{
    //if(SP0A20MIPICurrentStatus.iNightMode == enable)
    //    return;

	if (SP0A20MIPI_sensor_cap_state == KAL_TRUE)
		return ;	

	if (enable)
	{
		if (SP0A20MIPI_MPEG4_encode_mode == KAL_TRUE)
		{
			/*SP0A20_write_cmos_sensor(0x0303,0x2); 
			SP0A20_write_cmos_sensor(0x3A00,0x78); 
			SP0A20_write_cmos_sensor(0x3a14,0x02);//15);	// 50Hz Max exposure
			SP0A20_write_cmos_sensor(0x3a15,0xf0);//c6);	// 50Hz Max exposure
			SP0A20_write_cmos_sensor(0x3a02,0x02);//18);	// 60Hz Max exposure
			SP0A20_write_cmos_sensor(0x3a03,0xf0);//20);	// 60Hz Max exposure*/

		}
		else
		{	
			/*SP0A20_write_cmos_sensor(0x0303,0x1); 
			SP0A20_write_cmos_sensor(0x3A00,0x7C); 
			SP0A20_write_cmos_sensor(0x3a14,0x04);  // 50Hz Max exposure
			SP0A20_write_cmos_sensor(0x3a15,0xB0);  // 50Hz Max exposure
			SP0A20_write_cmos_sensor(0x3a02,0x4);  // 60Hz Max exposure
			SP0A20_write_cmos_sensor(0x3a03,0xB0);  // 60Hz Max exposure*/
		}	
	}
	else
	{
		if (SP0A20MIPI_MPEG4_encode_mode == KAL_TRUE)
		{
			/*SP0A20_write_cmos_sensor(0x0303,0x01); 
			SP0A20_write_cmos_sensor(0x3A00,0x78); 
			SP0A20_write_cmos_sensor(0x3a14,0x02);	// 50Hz Max exposure
			SP0A20_write_cmos_sensor(0x3a15,0xf0);	// 50Hz Max exposure
			SP0A20_write_cmos_sensor(0x3a02,0x02);	// 60Hz Max exposure
			SP0A20_write_cmos_sensor(0x3a03,0xf0);	// 60Hz Max exposure*/
		}
		else
		{
			/*SP0A20_write_cmos_sensor(0x0303,0x01); 
			SP0A20_write_cmos_sensor(0x3A00,0x78); 
			SP0A20_write_cmos_sensor(0x3a14,0x02);	// 50Hz Max exposure
			SP0A20_write_cmos_sensor(0x3a15,0xf0);	// 50Hz Max exposure
			SP0A20_write_cmos_sensor(0x3a02,0x02);	// 60Hz Max exposure
			SP0A20_write_cmos_sensor(0x3a03,0xf0);	// 60Hz Max exposure*/
		}	
	}

	spin_lock(&sp0A20mipi_yuv_drv_lock);
    SP0A20MIPICurrentStatus.iNightMode = enable;
	spin_unlock(&sp0A20mipi_yuv_drv_lock);
}	/* SP0A20MIPI_night_mode */

/*************************************************************************
* FUNCTION
*	SP0A20MIPI_GetSensorID
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
static kal_uint32 SP0A20MIPI_GetSensorID(kal_uint32 *sensorID)
{
    //volatile signed char i;
	kal_uint16 sensor_id=0,sensor_id_2=0;
	int i=0;
	
    // check if sensor ID correct
    for( ; i<3 ; i++)
	{
        sensor_id  =SP0A20MIPI_read_cmos_sensor(0x01);
        sensor_id_2=SP0A20MIPI_read_cmos_sensor(0x02);
        sensor_id = (sensor_id<<8)+sensor_id_2;
        SENSORDB("Sensor Read SP0A20MIPI ID 0x%x\n", (unsigned int)sensor_id);
		 
        if (sensor_id == SP0A20MIPI_SENSOR_ID)
        {
            *sensorID=sensor_id;
            break;
        }
	}

	if (sensor_id != SP0A20MIPI_SENSOR_ID)
	{
		*sensorID=0xFFFFFFFF;
	    SENSORDB("Sensor Read ByeBye \r\n");
		return ERROR_SENSOR_CONNECT_FAIL;
	}

    return ERROR_NONE;    
}  


/*****************************************************************************/
/* Windows Mobile Sensor Interface */
/*****************************************************************************/
/*************************************************************************
* FUNCTION
*	SP0A20MIPIOpen
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
UINT32 SP0A20MIPIOpen(void)
{
	//volatile signed char i;
	kal_uint16 sensor_id=0,sensor_id_2=0;
	
	int i;
	for(i=0 ; i<3 ; i++)
		{
			sensor_id  =SP0A20MIPI_read_cmos_sensor(0x01);
			sensor_id_2=SP0A20MIPI_read_cmos_sensor(0x02);
			sensor_id = (sensor_id<<8)+sensor_id_2;
			SENSORDB("Sensor Read SP0A20MIPI ID %x \r\n",(unsigned int)sensor_id);
			
			if (sensor_id == SP0A20MIPI_SENSOR_ID)
			{
				break;
			}
		}
	if (sensor_id != SP0A20MIPI_SENSOR_ID)
	{
	    SENSORDB("Sensor Read ByeBye \r\n");
		return ERROR_SENSOR_CONNECT_FAIL;
	}
#ifdef DEBUG_SENSOR_SP0A20  //gepeiwei   120903
							//判断手机对应目录下是否有名为sp2528_sd 的文件,没有默认参数
		
							//介于各种原因，本版本初始化参数在_s_fmt中。
		 struct file *fp; 
			mm_segment_t fs; 
			loff_t pos = 0; 
			static char buf[10*1024] ;
		 
			fp = filp_open("/mnt/sdcard/sp0A20_sd", O_RDONLY , 0); 
			if (IS_ERR(fp)) { 
				fromsd = 0;   
				printk("open file error\n");
				
				SP0A20MIPIInitialPara(); 
				SP0A20MIPI_Initialize_Setting();
			} 
			else 
			{
				fromsd = 1;
				printk("open file ok\n");
					
				SP0A20_Initialize_from_T_Flash();
			
				filp_close(fp, NULL); 
				set_fs(fs);
			}
#else  
	SP0A20MIPIInitialPara(); 
	SP0A20MIPI_Initialize_Setting();
#endif
	return ERROR_NONE;
}

/*************************************************************************
* FUNCTION
*	SP0A20MIPIClose
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
UINT32 SP0A20MIPIClose(void)
{
	return ERROR_NONE;
}	/* SP0A20MIPIClose() */

/*************************************************************************
* FUNCTION
*	SP0A20MIPIPreview
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
UINT32 SP0A20MIPIPreview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{

	spin_lock(&sp0A20mipi_yuv_drv_lock);
	SP0A20MIPI_sensor_cap_state = KAL_FALSE;
	
	if(sensor_config_data->SensorOperationMode==MSDK_SENSOR_OPERATION_MODE_VIDEO)		// MPEG4 Encode Mode
	{
		SP0A20MIPI_MPEG4_encode_mode = KAL_TRUE;		
		SP0A20MIPI_MJPEG_encode_mode = KAL_FALSE;
	}
	else
	{
		SP0A20MIPI_MPEG4_encode_mode = KAL_FALSE;		
		SP0A20MIPI_MJPEG_encode_mode = KAL_FALSE;
		
	}
	spin_unlock(&sp0A20mipi_yuv_drv_lock);
	
	SP0A20MIPI_PV_Mode();
	SP0A20MIPI_night_mode(SP0A20MIPICurrentStatus.iNightMode);
	SP0A20MIPI_set_mirror(sensor_config_data->SensorImageMirror);
	
	//just add for porting,please delete this when release
	//SP0A20MIPI_set_mirror(IMAGE_HV_MIRROR);

    image_window->ExposureWindowWidth = SP0A20MIPI_IMAGE_SENSOR_PV_WIDTH;
    image_window->ExposureWindowHeight = SP0A20MIPI_IMAGE_SENSOR_PV_HEIGHT;
	
	// copy sensor_config_data
	memcpy(&SP0A20MIPISensorConfigData, sensor_config_data, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
  	return ERROR_NONE;
}	/* SP0A20MIPIPreview() */

UINT32 SP0A20MIPICapture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                                                 MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
     //kal_uint32 pv_integration_time = 0;       // Uinit - us
     //kal_uint32 cap_integration_time = 0;
     //kal_uint16 PV_line_len = 0;
     //kal_uint16 CAP_line_len = 0;

	spin_lock(&sp0A20mipi_yuv_drv_lock);
	SP0A20MIPI_sensor_cap_state = KAL_TRUE;
	spin_unlock(&sp0A20mipi_yuv_drv_lock);
               
    SP0A20MIPI_CAP_Mode();         

    image_window->GrabStartX = SP0A20MIPI_IMAGE_SENSOR_FULL_INSERTED_PIXELS;
    image_window->GrabStartY = SP0A20MIPI_IMAGE_SENSOR_FULL_INSERTED_LINES;
    image_window->ExposureWindowWidth= SP0A20MIPI_IMAGE_SENSOR_FULL_WIDTH;
    image_window->ExposureWindowHeight = SP0A20MIPI_IMAGE_SENSOR_FULL_HEIGHT;

     // copy sensor_config_data
     memcpy(&SP0A20MIPISensorConfigData, sensor_config_data, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
     return ERROR_NONE;
}        /* SP0A20MIPICapture() */

UINT32 SP0A20MIPIGetResolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *pSensorResolution)
{
	pSensorResolution->SensorFullWidth=SP0A20MIPI_IMAGE_SENSOR_FULL_WIDTH;  //modify by yanxu
	pSensorResolution->SensorFullHeight=SP0A20MIPI_IMAGE_SENSOR_FULL_HEIGHT;
	pSensorResolution->SensorPreviewWidth=SP0A20MIPI_IMAGE_SENSOR_PV_WIDTH; 
	pSensorResolution->SensorPreviewHeight=SP0A20MIPI_IMAGE_SENSOR_PV_HEIGHT;
	pSensorResolution->SensorVideoWidth=SP0A20MIPI_IMAGE_SENSOR_PV_WIDTH; 
	pSensorResolution->SensorVideoHeight=SP0A20MIPI_IMAGE_SENSOR_PV_HEIGHT;
	return ERROR_NONE;
}	/* SP0A20MIPIGetResolution() */

UINT32 SP0A20MIPIGetInfo(MSDK_SCENARIO_ID_ENUM ScenarioId,
					  MSDK_SENSOR_INFO_STRUCT *pSensorInfo,
					  MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
	#if 0
    switch(ScenarioId)
    {
        
    	case MSDK_SCENARIO_ID_CAMERA_ZSD:
			 pSensorInfo->SensorPreviewResolutionX=SP0A20MIPI_IMAGE_SENSOR_FULL_WIDTH;
	         pSensorInfo->SensorPreviewResolutionY=SP0A20MIPI_IMAGE_SENSOR_FULL_HEIGHT;
			 pSensorInfo->SensorCameraPreviewFrameRate=15;
			 break;
		
		default:
	#else
	{
	#endif
	
			 pSensorInfo->SensorPreviewResolutionX=SP0A20MIPI_IMAGE_SENSOR_PV_WIDTH;
	         pSensorInfo->SensorPreviewResolutionY=SP0A20MIPI_IMAGE_SENSOR_PV_HEIGHT;
			 pSensorInfo->SensorCameraPreviewFrameRate=30;
    }
	pSensorInfo->SensorFullResolutionX=SP0A20MIPI_IMAGE_SENSOR_FULL_WIDTH;
	pSensorInfo->SensorFullResolutionY=SP0A20MIPI_IMAGE_SENSOR_FULL_HEIGHT;
	pSensorInfo->SensorCameraPreviewFrameRate=30;
	pSensorInfo->SensorVideoFrameRate=30;
	pSensorInfo->SensorStillCaptureFrameRate=10;
	pSensorInfo->SensorWebCamCaptureFrameRate=15;
	pSensorInfo->SensorResetActiveHigh=FALSE;
	pSensorInfo->SensorResetDelayCount=1;
	
	pSensorInfo->SensorOutputDataFormat=SENSOR_OUTPUT_FORMAT_VYUY;//SENSOR_OUTPUT_FORMAT_YUYV;
	
	pSensorInfo->SensorClockPolarity=SENSOR_CLOCK_POLARITY_LOW;	
	pSensorInfo->SensorClockFallingPolarity=SENSOR_CLOCK_POLARITY_LOW;

	pSensorInfo->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	pSensorInfo->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	pSensorInfo->SensorInterruptDelayLines = 1;
	
	#ifdef MIPI_INTERFACE
   		pSensorInfo->SensroInterfaceType        = SENSOR_INTERFACE_TYPE_MIPI;
   	#else
   		pSensorInfo->SensroInterfaceType		= SENSOR_INTERFACE_TYPE_PARALLEL;
   	#endif
	
	pSensorInfo->SensorMasterClockSwitch = 0; 
    pSensorInfo->SensorDrivingCurrent = ISP_DRIVING_8MA;//ISP_DRIVING_4MA;   	
	pSensorInfo->CaptureDelayFrame = 3; 
	pSensorInfo->PreviewDelayFrame = 3; 
	pSensorInfo->VideoDelayFrame = 4; 
	
	switch (ScenarioId) 
	{
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			pSensorInfo->SensorClockFreq=24;
			
			pSensorInfo->SensorClockDividCount=	3;
			pSensorInfo->SensorClockRisingCount= 0;
			pSensorInfo->SensorClockFallingCount= 2;
			pSensorInfo->SensorPixelClockCount= 3;
			
			pSensorInfo->SensorDataLatchCount= 2;
			
			pSensorInfo->SensorGrabStartX = 2; 
			pSensorInfo->SensorGrabStartY = 2; 
#ifdef MIPI_INTERFACE
            pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_1_LANE;//SENSOR_MIPI_2_LANE;			
            pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0; 
	        pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 14; 
	        pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
            pSensorInfo->SensorWidthSampling = 0;  // 0 is default 1x
            pSensorInfo->SensorHightSampling = 0;   // 0 is default 1x 
            pSensorInfo->SensorPacketECCOrder = 1;
#endif
		break;

		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			pSensorInfo->SensorClockFreq=24;
			pSensorInfo->SensorClockDividCount=	3;
			pSensorInfo->SensorClockRisingCount= 0;
			pSensorInfo->SensorClockFallingCount= 2;
			pSensorInfo->SensorPixelClockCount= 3;
			pSensorInfo->SensorDataLatchCount= 2;
			pSensorInfo->SensorGrabStartX = 2; 
			pSensorInfo->SensorGrabStartY = 2; 					
			#ifdef MIPI_INTERFACE
	            pSensorInfo->SensorMIPILaneNumber =SENSOR_MIPI_1_LANE; //SENSOR_MIPI_2_LANE;			
	            pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0; 
		        pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 14; 
		        pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
	            pSensorInfo->SensorWidthSampling = 0;  // 0 is default 1x
	            pSensorInfo->SensorHightSampling = 0;   // 0 is default 1x 
	            pSensorInfo->SensorPacketECCOrder = 1;
	        	#endif			
		break;
				
		default:
			pSensorInfo->SensorClockFreq=24;
			pSensorInfo->SensorClockDividCount=3;
			pSensorInfo->SensorClockRisingCount=0;
			pSensorInfo->SensorClockFallingCount=2;
			pSensorInfo->SensorPixelClockCount=3;
			pSensorInfo->SensorDataLatchCount=2;
			pSensorInfo->SensorGrabStartX = 2; 
			pSensorInfo->SensorGrabStartY = 2; 					
		break;
	}
	memcpy(pSensorConfigData, &SP0A20MIPISensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
	return ERROR_NONE;
}	/* SP0A20MIPIGetInfo() */


UINT32 SP0A20MIPIControl(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *pImageWindow,
					  MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
	switch (ScenarioId)
	{
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			SP0A20MIPIPreview(pImageWindow, pSensorConfigData);
		break;
		
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			SP0A20MIPICapture(pImageWindow, pSensorConfigData);
		break;
		
		#if 0
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
			   SP0A20MIPICapture(pImageWindow, pSensorConfigData);
			break;
		#endif
		
        default:
            return ERROR_INVALID_SCENARIO_ID;
	}
	return ERROR_NONE;
}	/* SP0A20MIPIControl() */

/* [TC] YUV sensor */	
#if WINMO_USE
void SP0A20MIPIQuery(PMSDK_FEATURE_INFO_STRUCT pSensorFeatureInfo)
{
	MSDK_FEATURE_TYPE_RANGE_STRUCT *pFeatureRange;
	MSDK_FEATURE_TYPE_MULTI_SELECTION_STRUCT *pFeatureMultiSelection;
	switch (pSensorFeatureInfo->FeatureId)
	{
		case ISP_FEATURE_DSC_MODE:
			pSensorFeatureInfo->FeatureType = MSDK_FEATURE_TYPE_MULTI_SELECTION;
			pSensorFeatureInfo->FeatureSupported = (UINT8)(MSDK_SET_GET_FEATURE_SUPPORTED|MSDK_QUERY_CAMERA_SUPPORTED);
			pFeatureMultiSelection = (PMSDK_FEATURE_TYPE_MULTI_SELECTION_STRUCT)(&pSensorFeatureInfo->FeatureInformation.FeatureMultiSelection);
			pFeatureMultiSelection->TotalSelection = CAM_NO_OF_SCENE_MODE_MAX-2;
			pFeatureMultiSelection->DefaultSelection = CAM_AUTO_DSC_MODE;
			pFeatureMultiSelection->SupportedSelection = 
				(CAMERA_FEATURE_SUPPORT(CAM_AUTO_DSC_MODE)|
				CAMERA_FEATURE_SUPPORT(CAM_NIGHTSCENE_MODE));			
		break;
		case ISP_FEATURE_WHITEBALANCE:
			pSensorFeatureInfo->FeatureType = MSDK_FEATURE_TYPE_MULTI_SELECTION;
			pSensorFeatureInfo->FeatureSupported = (UINT8)(MSDK_SET_GET_FEATURE_SUPPORTED|MSDK_QUERY_CAMERA_VIDEO_SUPPORTED);
			pFeatureMultiSelection = (PMSDK_FEATURE_TYPE_MULTI_SELECTION_STRUCT)(&pSensorFeatureInfo->FeatureInformation.FeatureMultiSelection);
			pFeatureMultiSelection->TotalSelection = CAM_NO_OF_WB;
			pFeatureMultiSelection->DefaultSelection = CAM_WB_AUTO;
			pFeatureMultiSelection->SupportedSelection = 
				(CAMERA_FEATURE_SUPPORT(CAM_WB_AUTO)|
				CAMERA_FEATURE_SUPPORT(CAM_WB_CLOUD)|
				CAMERA_FEATURE_SUPPORT(CAM_WB_DAYLIGHT)|
				CAMERA_FEATURE_SUPPORT(CAM_WB_INCANDESCENCE)|
				CAMERA_FEATURE_SUPPORT(CAM_WB_FLUORESCENT));
		break;
		case ISP_FEATURE_IMAGE_EFFECT:
			pSensorFeatureInfo->FeatureType = MSDK_FEATURE_TYPE_MULTI_SELECTION;
			pSensorFeatureInfo->FeatureSupported = (UINT8)(MSDK_SET_GET_FEATURE_SUPPORTED|MSDK_QUERY_CAMERA_VIDEO_SUPPORTED);
			pFeatureMultiSelection = (PMSDK_FEATURE_TYPE_MULTI_SELECTION_STRUCT)(&pSensorFeatureInfo->FeatureInformation.FeatureMultiSelection);
			pFeatureMultiSelection->TotalSelection = CAM_NO_OF_EFFECT_ENC;
			pFeatureMultiSelection->DefaultSelection = CAM_EFFECT_ENC_NORMAL;
			pFeatureMultiSelection->SupportedSelection = 
				(CAMERA_FEATURE_SUPPORT(CAM_EFFECT_ENC_NORMAL)|
				CAMERA_FEATURE_SUPPORT(CAM_EFFECT_ENC_GRAYSCALE)|
				CAMERA_FEATURE_SUPPORT(CAM_EFFECT_ENC_COLORINV)|
				CAMERA_FEATURE_SUPPORT(CAM_EFFECT_ENC_GRAYINV)|
				CAMERA_FEATURE_SUPPORT(CAM_EFFECT_ENC_SEPIABLUE)|
				CAMERA_FEATURE_SUPPORT(CAM_EFFECT_ENC_SKETCH)|
				CAMERA_FEATURE_SUPPORT(CAM_EFFECT_ENC_EMBOSSMENT)|
				CAMERA_FEATURE_SUPPORT(CAM_EFFECT_ENC_SEPIA));	
		break;
		case ISP_FEATURE_AE_METERING_MODE:
			pSensorFeatureInfo->FeatureSupported = MSDK_FEATURE_NOT_SUPPORTED;
		break;
		case ISP_FEATURE_BRIGHTNESS:
			pSensorFeatureInfo->FeatureType = MSDK_FEATURE_TYPE_RANGE;
			pSensorFeatureInfo->FeatureSupported = (UINT8)(MSDK_SET_GET_FEATURE_SUPPORTED|MSDK_QUERY_CAMERA_VIDEO_SUPPORTED);
			pFeatureRange = (PMSDK_FEATURE_TYPE_RANGE_STRUCT)(&pSensorFeatureInfo->FeatureInformation.FeatureRange);
			pFeatureRange->MinValue = CAM_EV_NEG_4_3;
			pFeatureRange->MaxValue = CAM_EV_POS_4_3;
			pFeatureRange->StepValue = CAMERA_FEATURE_ID_EV_STEP;
			pFeatureRange->DefaultValue = CAM_EV_ZERO;
		break;
		case ISP_FEATURE_BANDING_FREQ:
			pSensorFeatureInfo->FeatureType = MSDK_FEATURE_TYPE_MULTI_SELECTION;
			pSensorFeatureInfo->FeatureSupported = (UINT8)(MSDK_SET_GET_FEATURE_SUPPORTED|MSDK_QUERY_CAMERA_VIDEO_SUPPORTED);
			pFeatureMultiSelection = (PMSDK_FEATURE_TYPE_MULTI_SELECTION_STRUCT)(&pSensorFeatureInfo->FeatureInformation.FeatureMultiSelection);
			pFeatureMultiSelection->TotalSelection = CAM_NO_OF_BANDING;
			pFeatureMultiSelection->DefaultSelection = CAM_BANDING_50HZ;
			pFeatureMultiSelection->SupportedSelection = 
				(CAMERA_FEATURE_SUPPORT(CAM_BANDING_50HZ)|
				CAMERA_FEATURE_SUPPORT(CAM_BANDING_60HZ));
		break;
		case ISP_FEATURE_AF_OPERATION:
			pSensorFeatureInfo->FeatureSupported = MSDK_FEATURE_NOT_SUPPORTED;
		break;
		case ISP_FEATURE_AF_RANGE_CONTROL:
			pSensorFeatureInfo->FeatureSupported = MSDK_FEATURE_NOT_SUPPORTED;
		break;
		case ISP_FEATURE_FLASH:
			pSensorFeatureInfo->FeatureSupported = MSDK_FEATURE_NOT_SUPPORTED;			
		break;
		case ISP_FEATURE_VIDEO_SCENE_MODE:
			pSensorFeatureInfo->FeatureType = MSDK_FEATURE_TYPE_MULTI_SELECTION;
			pSensorFeatureInfo->FeatureSupported = (UINT8)(MSDK_SET_GET_FEATURE_SUPPORTED|MSDK_QUERY_VIDEO_SUPPORTED);
			pFeatureMultiSelection = (PMSDK_FEATURE_TYPE_MULTI_SELECTION_STRUCT)(&pSensorFeatureInfo->FeatureInformation.FeatureMultiSelection);
			pFeatureMultiSelection->TotalSelection = 2;
			pFeatureMultiSelection->DefaultSelection = CAM_VIDEO_AUTO_MODE;
			pFeatureMultiSelection->SupportedSelection = 
				(CAMERA_FEATURE_SUPPORT(CAM_VIDEO_AUTO_MODE)|
				CAMERA_FEATURE_SUPPORT(CAM_VIDEO_NIGHT_MODE));
		break;
		case ISP_FEATURE_ISO:
			pSensorFeatureInfo->FeatureSupported = MSDK_FEATURE_NOT_SUPPORTED;			
		break;
		default:
			pSensorFeatureInfo->FeatureSupported = MSDK_FEATURE_NOT_SUPPORTED;			
		break;
	}
}
#endif 

BOOL SP0A20MIPI_set_param_wb(UINT16 para)
{

	if(SP0A20MIPICurrentStatus.iWB == para)
		return TRUE;
	SENSORDB("[Enter]SP0A20MIPI set_param_wb func:para = %d\n",para);

	switch (para)
	{
		case AWB_MODE_AUTO:
			//SP0A20_write_cmos_sensor(0x3406, 0x00); //bit[0]:AWB Auto:0 menual:1
			break;

		case AWB_MODE_CLOUDY_DAYLIGHT:	
			//======================================================================
			//	MWB : Cloudy_D65										 
			//======================================================================
			/*SP0A20_write_cmos_sensor(0x3406, 0x01); //bit[0]:AWB Auto:0 menual:1
			SP0A20_write_cmos_sensor(0x3400, 0x05);
			SP0A20_write_cmos_sensor(0x3401, 0xC0);
			SP0A20_write_cmos_sensor(0x3402, 0x04);
			SP0A20_write_cmos_sensor(0x3403, 0x00);	
			SP0A20_write_cmos_sensor(0x3404, 0x05);			
			SP0A20_write_cmos_sensor(0x3405, 0x60);*/
			break;

		case AWB_MODE_DAYLIGHT:
			//==============================================
			//	MWB : sun&daylight_D50						
			//==============================================
			/*SP0A20_write_cmos_sensor(0x3406, 0x01); //bit[0]:AWB Auto:0 menual:1
			SP0A20_write_cmos_sensor(0x3400, 0x06);
			SP0A20_write_cmos_sensor(0x3401, 0x00);
			SP0A20_write_cmos_sensor(0x3402, 0x04);
			SP0A20_write_cmos_sensor(0x3403, 0x00);	
			SP0A20_write_cmos_sensor(0x3404, 0x05);			
			SP0A20_write_cmos_sensor(0x3405, 0x80);*/

			break;

		case AWB_MODE_INCANDESCENT:
			//==============================================									   
			//	MWB : Incand_Tungsten						
			//==============================================
			/*SP0A20_write_cmos_sensor(0x3406, 0x01); //bit[0]:AWB Auto:0 menual:1
			SP0A20_write_cmos_sensor(0x3400, 0x04);
			SP0A20_write_cmos_sensor(0x3401, 0x00);
			SP0A20_write_cmos_sensor(0x3402, 0x04);
			SP0A20_write_cmos_sensor(0x3403, 0x00);	
			SP0A20_write_cmos_sensor(0x3404, 0x09);			
			SP0A20_write_cmos_sensor(0x3405, 0xa0);*/	
			break;

		case AWB_MODE_FLUORESCENT:
			//==================================================================
			//	MWB : Florescent_TL84							  
			//==================================================================
			/*SP0A20_write_cmos_sensor(0x3406, 0x01); //bit[0]:AWB Auto:0 menual:1
			SP0A20_write_cmos_sensor(0x3400, 0x04);
			SP0A20_write_cmos_sensor(0x3401, 0x20);
			SP0A20_write_cmos_sensor(0x3402, 0x04);
			SP0A20_write_cmos_sensor(0x3403, 0x00);	
			SP0A20_write_cmos_sensor(0x3404, 0x08);			
			SP0A20_write_cmos_sensor(0x3405, 0xB0);*/

			break;

		case AWB_MODE_TUNGSTEN:	
			/*SP0A20_write_cmos_sensor(0x3406, 0x01); //bit[0]:AWB Auto:0 menual:1
			SP0A20_write_cmos_sensor(0x3400, 0x04);
			SP0A20_write_cmos_sensor(0x3401, 0x00);
			SP0A20_write_cmos_sensor(0x3402, 0x04);
			SP0A20_write_cmos_sensor(0x3403, 0x00);	
			SP0A20_write_cmos_sensor(0x3404, 0x0B);			
			SP0A20_write_cmos_sensor(0x3405, 0x80);*/

			break;
		default:
			return KAL_FALSE;	

		}
		spin_lock(&sp0A20mipi_yuv_drv_lock);
	    SP0A20MIPICurrentStatus.iWB = para;
		spin_unlock(&sp0A20mipi_yuv_drv_lock);
return TRUE;
}

BOOL SP0A20MIPI_set_param_effect(UINT16 para)
{
	/*----------------------------------------------------------------*/
	   /* Local Variables												 */
	   /*----------------------------------------------------------------*/
	   kal_uint32 ret = KAL_TRUE;
	   kal_uint8  SDE_1, SDE_2;
	   /*----------------------------------------------------------------*/
	   /* Code Body 													 */
	   /*----------------------------------------------------------------*/
	   if(SP0A20MIPICurrentStatus.iEffect == para)
		  return TRUE;
	   SENSORDB("[Enter]sp0A20mipi_yuv set_param_effect func:para = %d\n",para);

	  /* SDE_1=SP0A20MIPI_read_cmos_sensor(0x5001);
	   SDE_2=SP0A20MIPI_read_cmos_sensor(0x5580);
	   SDE_1&=0x7F;
	   SDE_2&=0x87;
	   SP0A20_write_cmos_sensor(0x5001,SDE_1);  // Normal,	 SDE off
	   SP0A20_write_cmos_sensor(0x5580,SDE_2);  
	   SP0A20_write_cmos_sensor(0x5583,0x40); // SEPIA			   
	   SP0A20_write_cmos_sensor(0x5584,0x40); // SEPIA*/

	   switch (para)
	   {
		   case MEFFECT_OFF:
			   break;
		   case MEFFECT_MONO:
			   /*SP0A20_write_cmos_sensor(0x5001,(SDE_1|0x80));  
			   SP0A20_write_cmos_sensor(0x5580,(SDE_2|0x20));  // Monochrome (Black & White)*/
			   break;
		   case MEFFECT_SEPIA:
			  /* SP0A20_write_cmos_sensor(0x5001,(SDE_1|0x80)); // SDE, UV adj en,
			   SP0A20_write_cmos_sensor(0x5580,(SDE_2|0x18)); // SDE manual UV en
			   SP0A20_write_cmos_sensor(0x5583,0xA0); // SEPIA			   
			   SP0A20_write_cmos_sensor(0x5584,0x40); // SEPIA*/
			   break;
		   case MEFFECT_SEPIABLUE:
			   /*SP0A20_write_cmos_sensor(0x5001,(SDE_1|0x80)); // SDE, UV adj en,
			   SP0A20_write_cmos_sensor(0x5580,(SDE_2|0x18)); // SDE manual UV en
			   SP0A20_write_cmos_sensor(0x5583,0x40); // SEPIA			   
			   SP0A20_write_cmos_sensor(0x5584,0xA0); // SEPIA*/
			   break;
		   case MEFFECT_NEGATIVE:
			   /*SP0A20_write_cmos_sensor(0x5001,(SDE_1|0x80));  
			   SP0A20_write_cmos_sensor(0x5580,(SDE_2|0x40)); // Negative Mono*/
			   break;
		   default:
			   ret = KAL_FALSE;
			   break;
	   }
	   
	   spin_lock(&sp0A20mipi_yuv_drv_lock);
	   SP0A20MIPICurrentStatus.iEffect = para;
	   spin_unlock(&sp0A20mipi_yuv_drv_lock);
	   return ret;
} 


BOOL SP0A20MIPI_set_param_banding(UINT16 para)
{
	kal_uint8 m_banding_auto;
	kal_uint8 m_banding_sel;  
	kal_uint8 m_banding_sel_set;  

//	if(SP0A20MIPICurrentStatus.iBanding == para)
//		return TRUE;
	
	SENSORDB("SP0A20MIPI_set_param_banding %d  \r\n", para);
	SENSORDB("50hz %d  \r\n", AE_FLICKER_MODE_50HZ);
	SENSORDB("60hz %d  \r\n", AE_FLICKER_MODE_60HZ);

	/*m_banding_auto  =SP0A20MIPI_read_cmos_sensor(0x3C01);
	//m_banding_sel   =SP0A20MIPI_read_cmos_sensor(0x3C0C);//read only
	m_banding_sel_set = SP0A20MIPI_read_cmos_sensor(0x3C00);
	

	m_banding_auto = m_banding_auto & 0x7F;
	//m_banding_sel = m_banding_sel   & 0xFA;
	m_banding_sel_set &= 0xfb;*/

    switch (para)
	{
		case AE_FLICKER_MODE_50HZ:
			/*SP0A20_write_cmos_sensor(0x3C00, (m_banding_sel_set |   4) );
			SP0A20_write_cmos_sensor(0x3C01, (m_banding_auto|0x80) );
		//	SP0A20_write_cmos_sensor(0x3C0C, (m_banding_sel |   1) );*/
			break;
		case AE_FLICKER_MODE_60HZ:
			/*SP0A20_write_cmos_sensor(0x3C00, (m_banding_sel_set)    );
			SP0A20_write_cmos_sensor(0x3C01, (m_banding_auto|0x80) );
		//	SP0A20_write_cmos_sensor(0x3C0C,  m_banding_sel        );*/
			break;
		default:

			/*SP0A20_write_cmos_sensor(0x3C01,  m_banding_auto     );
		//	SP0A20_write_cmos_sensor(0x3C0C, (m_banding_sel | 4) );*/
			return FALSE;
	}
	spin_lock(&sp0A20mipi_yuv_drv_lock);
    SP0A20MIPICurrentStatus.iBanding = para;
	spin_unlock(&sp0A20mipi_yuv_drv_lock);
	return TRUE;
} /* SP0A20MIPI_set_param_banding */

BOOL SP0A20MIPI_set_param_exposure(UINT16 para)
{

	if(SP0A20MIPICurrentStatus.iEV == para)
		return TRUE;
	
	SENSORDB("[Enter]sp0A20mipi_yuv set_param_exposure func:para = %d\n",para);
	  
	//SP0A20_write_cmos_sensor(0x0028, 0x7000);	
    switch (para)
	{	
		case AE_EV_COMP_n10://EV -1
			/*SP0A20_write_cmos_sensor(0x3a11, 0x90-0x12); // High threshold
			SP0A20_write_cmos_sensor(0x3a1b, 0x4a-0x12); // WPT 2 
			SP0A20_write_cmos_sensor(0x3a0f, 0x48-0x12); // WPT  
			SP0A20_write_cmos_sensor(0x3a10, 0x44-0x12); // BPT  
			SP0A20_write_cmos_sensor(0x3a1e, 0x42-0x12); // BPT 2 
			SP0A20_write_cmos_sensor(0x3a1f, 0x22-0x12); // Low threshold */
			break;
		case AE_EV_COMP_00:		  //EV 0		   
			/*SP0A20_write_cmos_sensor(0x3a11, 0x90);	// High threshold
			SP0A20_write_cmos_sensor(0x3a1b, 0x4a);	// WPT 2 
			SP0A20_write_cmos_sensor(0x3a0f, 0x48);	// WPT	
			SP0A20_write_cmos_sensor(0x3a10, 0x44);	// BPT	
			SP0A20_write_cmos_sensor(0x3a1e, 0x42);	// BPT 2 
			SP0A20_write_cmos_sensor(0x3a1f, 0x22);	// Low threshold */
			break;
		case AE_EV_COMP_10:			 //EV +1		
			/*SP0A20_write_cmos_sensor(0x3a11, 0x90+0x12); // High threshold
			SP0A20_write_cmos_sensor(0x3a1b, 0x4a+0x12); // WPT 2 
			SP0A20_write_cmos_sensor(0x3a0f, 0x48+0x12); // WPT  
			SP0A20_write_cmos_sensor(0x3a10, 0x44+0x12); // BPT  
			SP0A20_write_cmos_sensor(0x3a1e, 0x42+0x12); // BPT 2 
			SP0A20_write_cmos_sensor(0x3a1f, 0x22+0x12); // Low threshold */
			break;
			
		case AE_EV_COMP_n13:					
		case AE_EV_COMP_n07:				   
		case AE_EV_COMP_n03:				   
		case AE_EV_COMP_03: 			
		case AE_EV_COMP_07: 
		case AE_EV_COMP_13:
			break;
			
		default:			
			return FALSE;
	}
	spin_lock(&sp0A20mipi_yuv_drv_lock);
	SP0A20MIPICurrentStatus.iEV = para;
	spin_unlock(&sp0A20mipi_yuv_drv_lock);
	return TRUE;

}/* SP0A20MIPI_set_param_exposure */


UINT32 SP0A20MIPIYUVSensorSetting(FEATURE_ID iCmd, UINT16 iPara)
{
#ifdef DEBUG_SENSOR_SP0A20
	return 0;
#endif

	switch (iCmd) {
	case FID_SCENE_MODE:
	    if (iPara == SCENE_MODE_OFF)
	    {
	        SP0A20MIPI_night_mode(0); 
	    }

         else if (iPara == SCENE_MODE_NIGHTSCENE)			
	    {
            SP0A20MIPI_night_mode(1); 
	    }	    
	    break; 	    
	case FID_AWB_MODE:
         SP0A20MIPI_set_param_wb(iPara);
	break;
	case FID_COLOR_EFFECT:
         SP0A20MIPI_set_param_effect(iPara);
	break;
	case FID_AE_EV:  	    
         SP0A20MIPI_set_param_exposure(iPara);
	break;
	case FID_AE_FLICKER:
         SP0A20MIPI_set_param_banding(iPara);
	break;
    case FID_AE_SCENE_MODE:  
		spin_lock(&sp0A20mipi_yuv_drv_lock);
      if (iPara == AE_MODE_OFF) {
          SP0A20MIPI_AE_ENABLE = KAL_FALSE; 
      }
      else {
          SP0A20MIPI_AE_ENABLE = KAL_TRUE; 
      }
	  spin_unlock(&sp0A20mipi_yuv_drv_lock);
      SP0A20MIPI_set_AE_mode(SP0A20MIPI_AE_ENABLE);
    break; 
	case FID_ZOOM_FACTOR:
		spin_lock(&sp0A20mipi_yuv_drv_lock);
	    zoom_factor = iPara; 
		spin_unlock(&sp0A20mipi_yuv_drv_lock);
	break; 
	default:
	break;
	}
	return ERROR_NONE;
}   /* SP0A20MIPIYUVSensorSetting */


UINT32 SP0A20MIPIYUVSetVideoMode(UINT16 u2FrameRate)
{

    if(SP0A20MIPICurrentStatus.iFrameRate == u2FrameRate)
      return ERROR_NONE;

	spin_lock(&sp0A20mipi_yuv_drv_lock);
    SP0A20MIPI_VEDIO_encode_mode = KAL_TRUE; 
	SP0A20MIPI_MPEG4_encode_mode = KAL_TRUE;
	spin_unlock(&sp0A20mipi_yuv_drv_lock);
	

	/*SP0A20_write_cmos_sensor(0x3A00,0x78); 
	SP0A20_write_cmos_sensor(0x3a14,0x02);//15);	// 50Hz Max exposure
	SP0A20_write_cmos_sensor(0x3a15,0xf0);//c6);	// 50Hz Max exposure
	SP0A20_write_cmos_sensor(0x3a02,0x02);//18);	// 60Hz Max exposure
	SP0A20_write_cmos_sensor(0x3a03,0xf0);//20);	// 60Hz Max exposure*/

    if(20<=u2FrameRate && u2FrameRate<=30) //fix 30
    {		
		//SP0A20_write_cmos_sensor(0x0303,0x01);	// PLL control
    }
    else if(5<=u2FrameRate && u2FrameRate<20 )// fix 15
    {
		//SP0A20_write_cmos_sensor(0x0303,0x02);	// PLL control
    }
    else 
    {
        printk("Wrong Frame Rate \n"); 
    }
    return ERROR_NONE;
}


kal_uint16 SP0A20MIPIReadShutter(void)
{
   kal_uint16 temp_msb=0x0000,temp_lsb=0x0000;

  /* temp_msb=SP0A20MIPI_read_cmos_sensor(0x0202);
   temp_msb = (temp_msb<<8)&0xff00;
   temp_lsb=SP0A20MIPI_read_cmos_sensor(0x0203);
   temp_lsb = temp_lsb&0x00ff;
   
   return (temp_msb|temp_lsb);
}
kal_uint16 SP0A20MIPIReadGain(void)
{
	kal_uint16 temp_msb=0x0000,temp_lsb=0x0000;
	
	temp_msb=SP0A20MIPI_read_cmos_sensor(0x0204);
	temp_msb = (temp_msb<<8)&0xff00;
	temp_lsb=SP0A20MIPI_read_cmos_sensor(0x0205);
	temp_lsb = temp_lsb&0x00ff;
	
	return (temp_msb|temp_lsb);

}
kal_uint16 SP0A20MIPIReadAwbRGain(void)
{
	kal_uint16 temp_msb=0x0000,temp_lsb=0x0000;
	temp_msb = SP0A20MIPI_read_cmos_sensor(0x3400);
	temp_msb = (temp_msb<<8)&0xff00;
	temp_lsb = SP0A20MIPI_read_cmos_sensor(0x3401);
	temp_lsb = temp_lsb&0x00ff;*/

	return (temp_msb|temp_lsb);

}
kal_uint16 SP0A20MIPIReadAwbBGain(void)
{
	kal_uint16 temp_msb=0x0000,temp_lsb=0x0000;
	/*temp_msb = SP0A20MIPI_read_cmos_sensor(0x3404);
	temp_msb = (temp_msb<<8)&0xff00;
	temp_lsb = SP0A20MIPI_read_cmos_sensor(0x3405);
	temp_lsb = temp_lsb&0x00ff;*/
	return (temp_msb|temp_lsb);

}
#if 0

/*************************************************************************
* FUNCTION
*    SP0A20MIPIGetEvAwbRef
*
* DESCRIPTION
* RETURNS
*    None
*
* LOCAL AFFECTED
*
*************************************************************************/
static void SP0A20MIPIGetEvAwbRef(UINT32 pSensorAEAWBRefStruct/*PSENSOR_AE_AWB_REF_STRUCT Ref*/)
{
    PSENSOR_AE_AWB_REF_STRUCT Ref = (PSENSOR_AE_AWB_REF_STRUCT)pSensorAEAWBRefStruct;
    SENSORDB("SP0A20MIPIGetEvAwbRef ref = 0x%x \n", Ref);
    	
	Ref->SensorAERef.AeRefLV05Shutter = 5422;
    Ref->SensorAERef.AeRefLV05Gain = 478; /* 128 base */
    Ref->SensorAERef.AeRefLV13Shutter = 80;
    Ref->SensorAERef.AeRefLV13Gain = 128; /*  128 base */
    Ref->SensorAwbGainRef.AwbRefD65Rgain = 186; /* 128 base */
    Ref->SensorAwbGainRef.AwbRefD65Bgain = 158; /* 128 base */
    Ref->SensorAwbGainRef.AwbRefCWFRgain = 196; /* 1.25x, 128 base */
    Ref->SensorAwbGainRef.AwbRefCWFBgain = 278; /* 1.28125x, 128 base */
}
/*************************************************************************
* FUNCTION
*    SP0A20MIPIGetCurAeAwbInfo
*
* DESCRIPTION
* RETURNS
*    None
*
* LOCAL AFFECTED
*
*************************************************************************/
static void SP0A20MIPIGetCurAeAwbInfo(UINT32 pSensorAEAWBCurStruct/*PSENSOR_AE_AWB_CUR_STRUCT Info*/)
{
    PSENSOR_AE_AWB_CUR_STRUCT Info = (PSENSOR_AE_AWB_CUR_STRUCT)pSensorAEAWBCurStruct;
    SENSORDB("SP0A20MIPIGetCurAeAwbInfo Info = 0x%x \n", Info);

    Info->SensorAECur.AeCurShutter = SP0A20MIPIReadShutter();
    Info->SensorAECur.AeCurGain = SP0A20MIPIReadGain() * 2; /* 128 base */
    
    Info->SensorAwbGainCur.AwbCurRgain = SP0A20MIPIReadAwbRGain()<< 1; /* 128 base */
    
    Info->SensorAwbGainCur.AwbCurBgain = SP0A20MIPIReadAwbBGain()<< 1; /* 128 base */
}

#endif

void SP0A20MIPIGetAFMaxNumFocusAreas(UINT32 *pFeatureReturnPara32)
{	
    *pFeatureReturnPara32 = 0;    
    SENSORDB("SP0A20MIPIGetAFMaxNumFocusAreas, *pFeatureReturnPara32 = %d\n",*pFeatureReturnPara32);

}

void SP0A20MIPIGetAFMaxNumMeteringAreas(UINT32 *pFeatureReturnPara32)
{	
    *pFeatureReturnPara32 = 0;    
    SENSORDB("SP0A20MIPIGetAFMaxNumMeteringAreas,*pFeatureReturnPara32 = %d\n",*pFeatureReturnPara32);

}

void SP0A20MIPIGetAEAWBLock(UINT32 *pAElockRet32,UINT32 *pAWBlockRet32)
{
    *pAElockRet32 = 1;
	*pAWBlockRet32 = 1;
    SENSORDB("SP0A20MIPIGetAEAWBLock,AE=%d ,AWB=%d\n,",*pAElockRet32,*pAWBlockRet32);
}


void SP0A20MIPIGetExifInfo(UINT32 exifAddr)
{
    SENSOR_EXIF_INFO_STRUCT* pExifInfo = (SENSOR_EXIF_INFO_STRUCT*)exifAddr;
    pExifInfo->FNumber = 28;
    pExifInfo->AEISOSpeed = AE_ISO_100;
    pExifInfo->AWBMode = SP0A20MIPICurrentStatus.iWB;
    pExifInfo->CapExposureTime = 0;
    pExifInfo->FlashLightTimeus = 0;
    pExifInfo->RealISOValue = AE_ISO_100;
}

UINT32 SP0A20MIPISetMaxFramerateByScenario(MSDK_SCENARIO_ID_ENUM scenarioId, MUINT32 frameRate) {
	kal_uint32 pclk;
	kal_int16 dummyLine;
	kal_uint16 lineLength,frameHeight;
		
	SENSORDB("SP0A20MIPISetMaxFramerateByScenario: scenarioId = %d, frame rate = %d\n",scenarioId,frameRate);
	switch (scenarioId) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			pclk = SP0A20MIPI_sensor_pclk/10;
			lineLength = SP0A20MIPI_SXGA_PERIOD_PIXEL_NUMS;
			frameHeight = (10 * pclk)/frameRate/lineLength;
			dummyLine = frameHeight - SP0A20MIPI_SXGA_PERIOD_LINE_NUMS;
			SP0A20MIPI_set_dummy(0, dummyLine);			
			break;			
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			pclk = SP0A20MIPI_sensor_pclk/10;
			lineLength = SP0A20MIPI_SXGA_PERIOD_PIXEL_NUMS;
			frameHeight = (10 * pclk)/frameRate/lineLength;
			dummyLine = frameHeight - SP0A20MIPI_SXGA_PERIOD_LINE_NUMS;
			SP0A20MIPI_set_dummy(0, dummyLine);			
			break;			
			 break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_CAMERA_ZSD:			
			pclk = SP0A20MIPI_sensor_pclk/10;//hanlei
			lineLength = SP0A20MIPI_SXGA_PERIOD_PIXEL_NUMS;
			frameHeight = (10 * pclk)/frameRate/lineLength;
			dummyLine = frameHeight - SP0A20MIPI_SXGA_PERIOD_LINE_NUMS;
			SP0A20MIPI_set_dummy(0, dummyLine);			
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
	return ERROR_NONE;
}


UINT32 SP0A20MIPIGetDefaultFramerateByScenario(MSDK_SCENARIO_ID_ENUM scenarioId, MUINT32 *pframeRate) 
{

	switch (scenarioId) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			 *pframeRate = 300;
			 break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
			 *pframeRate = 300;
			break;		
        case MSDK_SCENARIO_ID_CAMERA_3D_PREVIEW: //added
        case MSDK_SCENARIO_ID_CAMERA_3D_VIDEO:
        case MSDK_SCENARIO_ID_CAMERA_3D_CAPTURE: //added   
			 *pframeRate = 300;
			break;		
		default:
			break;
	}

	return ERROR_NONE;
}

UINT32 SP0A20MIPIFeatureControl(MSDK_SENSOR_FEATURE_ENUM FeatureId,
							 UINT8 *pFeaturePara,UINT32 *pFeatureParaLen)
{
	UINT16 *pFeatureReturnPara16=(UINT16 *) pFeaturePara;
	UINT16 *pFeatureData16=(UINT16 *) pFeaturePara;
	UINT32 *pFeatureReturnPara32=(UINT32 *) pFeaturePara;
	UINT32 *pFeatureData32=(UINT32 *) pFeaturePara;
	//PNVRAM_SENSOR_DATA_STRUCT pSensorDefaultData=(PNVRAM_SENSOR_DATA_STRUCT) pFeaturePara;
	MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData=(MSDK_SENSOR_CONFIG_STRUCT *) pFeaturePara;
	MSDK_SENSOR_REG_INFO_STRUCT *pSensorRegData=(MSDK_SENSOR_REG_INFO_STRUCT *) pFeaturePara;
	//MSDK_SENSOR_GROUP_INFO_STRUCT *pSensorGroupInfo=(MSDK_SENSOR_GROUP_INFO_STRUCT *) pFeaturePara;
	//MSDK_SENSOR_ITEM_INFO_STRUCT *pSensorItemInfo=(MSDK_SENSOR_ITEM_INFO_STRUCT *) pFeaturePara;
	//MSDK_SENSOR_ENG_INFO_STRUCT	*pSensorEngInfo=(MSDK_SENSOR_ENG_INFO_STRUCT *) pFeaturePara;
#if WINMO_USE	
	PMSDK_FEATURE_INFO_STRUCT pSensorFeatureInfo=(PMSDK_FEATURE_INFO_STRUCT) pFeaturePara;
#endif 


	switch (FeatureId)
	{
		case SENSOR_FEATURE_GET_RESOLUTION:
			*pFeatureReturnPara16++=SP0A20MIPI_IMAGE_SENSOR_FULL_WIDTH;
			*pFeatureReturnPara16=SP0A20MIPI_IMAGE_SENSOR_FULL_HEIGHT;
			*pFeatureParaLen=4;
		break;
		case SENSOR_FEATURE_GET_PERIOD:
		break;
		case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
			*pFeatureReturnPara32 = SP0A20MIPI_sensor_pclk/10;
			*pFeatureParaLen=4;
		break;
		case SENSOR_FEATURE_SET_ESHUTTER:
		break;
		case SENSOR_FEATURE_SET_NIGHTMODE:
			
	#ifndef DEBUG_SENSOR_SP0A20      
			SP0A20MIPI_night_mode((BOOL) *pFeatureData16);
	#endif
		break;
		case SENSOR_FEATURE_SET_GAIN:
		case SENSOR_FEATURE_SET_FLASHLIGHT:
		break;
		case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
		break;
		case SENSOR_FEATURE_SET_REGISTER:
			SP0A20_write_cmos_sensor(pSensorRegData->RegAddr, pSensorRegData->RegData);
		break;
		case SENSOR_FEATURE_GET_REGISTER:
			pSensorRegData->RegData = SP0A20MIPI_read_cmos_sensor(pSensorRegData->RegAddr);
		break;
		case SENSOR_FEATURE_GET_CONFIG_PARA:
			memcpy(pSensorConfigData, &SP0A20MIPISensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
			*pFeatureParaLen=sizeof(MSDK_SENSOR_CONFIG_STRUCT);
		break;
		case SENSOR_FEATURE_SET_CCT_REGISTER:
		case SENSOR_FEATURE_GET_CCT_REGISTER:
		case SENSOR_FEATURE_SET_ENG_REGISTER:
		case SENSOR_FEATURE_GET_ENG_REGISTER:
		case SENSOR_FEATURE_GET_REGISTER_DEFAULT:

		case SENSOR_FEATURE_CAMERA_PARA_TO_SENSOR:
		case SENSOR_FEATURE_SENSOR_TO_CAMERA_PARA:
		
		case SENSOR_FEATURE_GET_GROUP_INFO:
		case SENSOR_FEATURE_GET_ITEM_INFO:
		case SENSOR_FEATURE_SET_ITEM_INFO:
		case SENSOR_FEATURE_GET_ENG_INFO:
					break;
		case SENSOR_FEATURE_GET_GROUP_COUNT:
                        *pFeatureReturnPara32++=0;
                        *pFeatureParaLen=4;	    

		break;
		case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
			// get the lens driver ID from EEPROM or just return LENS_DRIVER_ID_DO_NOT_CARE
			// if EEPROM does not exist in camera module.
			*pFeatureReturnPara32=LENS_DRIVER_ID_DO_NOT_CARE;
			*pFeatureParaLen=4;
		break;
		case SENSOR_FEATURE_SET_YUV_CMD:
			//SP0A20MIPIYUVSensorSetting((MSDK_ISP_FEATURE_ENUM)*pFeatureData16, *(pFeatureData16+1));
			SP0A20MIPIYUVSensorSetting((FEATURE_ID)*pFeatureData32, *(pFeatureData32+1));
		break;
		break;
#if WINMO_USE		
		case SENSOR_FEATURE_QUERY:
			SP0A20MIPIQuery(pSensorFeatureInfo);
			*pFeatureParaLen = sizeof(MSDK_FEATURE_INFO_STRUCT);
		break;		
		case SENSOR_FEATURE_SET_YUV_CAPTURE_RAW_SUPPORT:
			/* update yuv capture raw support flag by *pFeatureData16 */
		break;
#endif 				
		case SENSOR_FEATURE_SET_VIDEO_MODE:
		    SP0A20MIPIYUVSetVideoMode(*pFeatureData16);
		    break; 
		#if 0
		case SENSOR_FEATURE_GET_EV_AWB_REF:
			 SP0A20MIPIGetEvAwbRef(*pFeatureData32);
				break;
  		case SENSOR_FEATURE_GET_SHUTTER_GAIN_AWB_GAIN:
			   SP0A20MIPIGetCurAeAwbInfo(*pFeatureData32);	
			break;
		#endif
		case SENSOR_FEATURE_CHECK_SENSOR_ID:
            SP0A20MIPI_GetSensorID(pFeatureData32); 
            break; 	

		case SENSOR_FEATURE_GET_AF_MAX_NUM_FOCUS_AREAS:
			SP0A20MIPIGetAFMaxNumFocusAreas(pFeatureReturnPara32); 		   
			*pFeatureParaLen=4;
			break;	   
		case SENSOR_FEATURE_GET_AE_MAX_NUM_METERING_AREAS:
			SP0A20MIPIGetAFMaxNumMeteringAreas(pFeatureReturnPara32);			  
			*pFeatureParaLen=4;
			break;
		case SENSOR_FEATURE_GET_EXIF_INFO:
			SENSORDB("SENSOR_FEATURE_GET_EXIF_INFO\n");
			SENSORDB("EXIF addr = 0x%x\n",*pFeatureData32); 		 
			SP0A20MIPIGetExifInfo(*pFeatureData32);
			break;
		case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
			SP0A20MIPISetMaxFramerateByScenario((MSDK_SCENARIO_ID_ENUM)*pFeatureData32, *(pFeatureData32+1));
			break;
		case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
			SP0A20MIPIGetDefaultFramerateByScenario((MSDK_SCENARIO_ID_ENUM)*pFeatureData32, (MUINT32 *)(*(pFeatureData32+1)));
			break;
		case SENSOR_FEATURE_GET_AE_AWB_LOCK_INFO:
			SP0A20MIPIGetAEAWBLock((*pFeatureData32),*(pFeatureData32+1));
			break;			
		default:
			break;			
	}
	return ERROR_NONE;
}

SENSOR_FUNCTION_STRUCT	SensorFuncSP0A20MIPI=
{
	SP0A20MIPIOpen,
	SP0A20MIPIGetInfo,
	SP0A20MIPIGetResolution,
	SP0A20MIPIFeatureControl,
	SP0A20MIPIControl,
	SP0A20MIPIClose
};

UINT32 SP0A20_MIPI_YUV_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
	/* To Do : Check Sensor status here */
	if (pfFunc!=NULL)
		*pfFunc=&SensorFuncSP0A20MIPI;
	return ERROR_NONE;
}	/* SensorInit() */
