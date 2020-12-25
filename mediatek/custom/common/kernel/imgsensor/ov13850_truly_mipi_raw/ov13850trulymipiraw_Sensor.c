/*******************************************************************************************/


/*******************************************************************************************/

#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
#include <linux/xlog.h>
#include <asm/system.h>

#include <linux/proc_fs.h> 


#include <linux/dma-mapping.h>

#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"

#include "ov13850trulymipiraw_Sensor.h"
#include "ov13850trulymipiraw_Camera_Sensor_para.h"
#include "ov13850trulymipiraw_CameraCustomized.h"
static DEFINE_SPINLOCK(ov13850trulymipiraw_drv_lock);

//for shutter,gain,framelength use muliwrite
//#define SHUTTER_MWRITE 1
#ifdef SHUTTER_MWRITE
kal_uint8* pSBuf=NULL;
dma_addr_t sdmaHandle;
 kal_uint8 ov13850truly_shutter[] = {
 0x35, 0x00, 0x00,
 0x35, 0x01, 0x00,
 0x35, 0x02, 0x00,
};
static kal_uint8 ov13850truly_shutter_len = 9;
static kal_uint8 ov13850truly_gain[] = {
	0x35, 0x0a, 0x00,
	0x35, 0x0b, 0x00,

};
static kal_uint8  ov13850truly_gain_len=6;

static kal_uint8 ov13850truly_framelength[] = {
	0x38, 0x0e, 0x00,
	0x38, 0x0f, 0x00,
};
static kal_uint8  ov13850truly_framelength_len=6;


#endif

#define OV13850TRULY_DEBUG  //xb.pang
//#define OV13850TRULY_DEBUG_SOFIA
#define OV13850TRULY_TEST_PATTERN_CHECKSUM (0xf86cfdf4) //V_MIRROR
#define OV13850TRULY_DEBUG_SOFIA  //xb.pang

#ifdef OV13850TRULY_DEBUG
	#define OV13850TRULYDB(fmt, arg...) xlog_printk(ANDROID_LOG_DEBUG, "[OV13850TRULYRaw] ",  fmt, ##arg)
#else
	#define OV13850TRULYDB(fmt, arg...)
#endif

#ifdef OV13850TRULY_DEBUG_SOFIA
	#define OV13850TRULYDBSOFIA(fmt, arg...) xlog_printk(ANDROID_LOG_DEBUG, "[OV13850TRULYRaw] ",  fmt, ##arg)
#else
	#define OV13850TRULYDBSOFIA(fmt, arg...)
#endif

//xb.pang for test
//kal_uint8 prv_flag = 0;

#define mDELAY(ms)  mdelay(ms)
kal_uint8 OV13850TRULY_WRITE_ID = OV13850TRULYMIPI_WRITE_ID;
kal_uint32 OV13850TRULY_FeatureControl_PERIOD_PixelNum=OV13850TRULY_PV_PERIOD_PIXEL_NUMS;
kal_uint32 OV13850TRULY_FeatureControl_PERIOD_LineNum=OV13850TRULY_PV_PERIOD_LINE_NUMS;
UINT16  ov13850trulyVIDEO_MODE_TARGET_FPS = 30;
MSDK_SENSOR_CONFIG_STRUCT OV13850TRULYSensorConfigData;
MSDK_SCENARIO_ID_ENUM OV13850TRULYCurrentScenarioId = MSDK_SCENARIO_ID_CAMERA_PREVIEW;

/* FIXME: old factors and DIDNOT use now. s*/
SENSOR_REG_STRUCT OV13850TRULYSensorCCT[]=CAMERA_SENSOR_CCT_DEFAULT_VALUE;
SENSOR_REG_STRUCT OV13850TRULYSensorReg[ENGINEER_END]=CAMERA_SENSOR_REG_DEFAULT_VALUE;
/* FIXME: old factors and DIDNOT use now. e*/

static OV13850TRULY_PARA_STRUCT ov13850truly;

extern int iReadReg(u16 a_u2Addr , u8 * a_puBuff , u16 i2cId);
extern int iWriteReg(u16 a_u2Addr , u32 a_u4Data , u32 a_u4Bytes , u16 i2cId);
extern int iMultiWriteReg(u8 *pData, u16 lens, u16 i2cId);

kal_uint16 OV13850TRULY_read_cmos_sensor(kal_uint32 addr);
#define OV13850TRULY_write_cmos_sensor(addr, para) iWriteReg((u16) addr , (u32) para , 1, OV13850TRULY_WRITE_ID)

#define OV13850TRULY_multi_write_cmos_sensor(pData, lens) iMultiWriteReg((u8*) pData, (u16) lens, OV13850TRULY_WRITE_ID)

#define OV13850TRULY_ORIENTATION IMAGE_H_MIRROR//IMAGE_NORMAL //IMAGE_V_MIRROR

#define OTP_CALIBRATION
#ifdef OTP_CALIBRATION
#define RG_Ratio_Typical 0x146//0x17d
#define BG_Ratio_Typical 0x141//0x168

#define OTP_DRV_START_ADDR 0x7220
#define OTP_DRV_INFO_GROUP_COUNT 3
#define OTP_DRV_INFO_SIZE 5
#define OTP_DRV_AWB_GROUP_COUNT 3
#define OTP_DRV_AWB_SIZE 5
#define OTP_DRV_VCM_GROUP_COUNT 3
#define OTP_DRV_VCM_SIZE 3
#define OTP_DRV_LSC_GROUP_COUNT 3
#define OTP_DRV_LSC_SIZE 62
#define OTP_DRV_LSC_REG_ADDR 0x5200

struct otp_struct {
int module_integrator_id;
int lens_id;
int production_year;
int production_month;
int production_day;
int rg_ratio;
int bg_ratio;
int light_rg;
int light_bg;
int lenc[OTP_DRV_LSC_SIZE ];
int VCM_start;
int VCM_end;
int VCM_dir;
};

#ifndef ORIGINAL_VERSION	
static struct otp_struct s_otp_wb;
static struct otp_struct s_otp_lenc;
static struct otp_struct s_otp_module;
static bool isNeedReadOtp = false;
#endif
// index: index of otp group. (1, 2, 3)
// return: 0, group index is empty
// 1, group index has invalid data
// 2, group index has valid data
static int check_otp_info(int index)
{
	int flag;
	int nFlagAddress = OTP_DRV_START_ADDR;
	OV13850TRULY_write_cmos_sensor(0x3d84, 0xC0);
	//partial mode OTP write start address
	OV13850TRULY_write_cmos_sensor(0x3d88, (nFlagAddress>>8) & 0xff);
	OV13850TRULY_write_cmos_sensor(0x3d89, nFlagAddress & 0xff);
	// partial mode OTP write end address
	OV13850TRULY_write_cmos_sensor(0x3d8A, (nFlagAddress>>8) & 0xff );
	OV13850TRULY_write_cmos_sensor(0x3d8B, nFlagAddress & 0xff );
	// read otp into buffer
	OV13850TRULY_write_cmos_sensor(0x3d81, 0x01);
	mDELAY(5);
	flag = OV13850TRULY_read_cmos_sensor(nFlagAddress);
	//select group
	if (index == 1)
	{
	flag = (flag>>6) & 0x03;
	}
	else if (index == 2)
	{
	flag = (flag>>4) & 0x03;
	}
	else if (index ==3)
	{
	flag = (flag>>2) & 0x03;
	}
	// clear otp buffer
	OV13850TRULY_write_cmos_sensor(nFlagAddress, 0x00);
	if (flag == 0x00) {
	return 0;
	}
	else if (flag & 0x02) {
	return 1;
	}
	else {
	return 2;
	}
}

static int check_otp_wb(int index)
{
	int flag;
	int nFlagAddress = OTP_DRV_START_ADDR+1+OTP_DRV_INFO_GROUP_COUNT*OTP_DRV_INFO_SIZE;
	OV13850TRULY_write_cmos_sensor(0x3d84, 0xC0);
	//partial mode OTP write start address
	OV13850TRULY_write_cmos_sensor(0x3d88, (nFlagAddress>>8) & 0xff );
	OV13850TRULY_write_cmos_sensor(0x3d89, nFlagAddress & 0xff);
	// partial mode OTP write end address
	OV13850TRULY_write_cmos_sensor(0x3d8A, (nFlagAddress>>8) & 0xff );
	OV13850TRULY_write_cmos_sensor(0x3d8B, nFlagAddress & 0xff);
	// read otp into buffer
	OV13850TRULY_write_cmos_sensor(0x3d81, 0x01);
	mDELAY(5);
	//select group
	flag = OV13850TRULY_read_cmos_sensor(nFlagAddress);
	if (index == 1)
	{
	flag = (flag>>6) & 0x03;
	}
	else if (index == 2)
	{
	flag = (flag>>4) & 0x03;
	}
	else if (index == 3)
	{
	flag = (flag>>2) & 0x03;
	}
	// clear otp buffer
	OV13850TRULY_write_cmos_sensor(nFlagAddress, 0x00);
	if (flag == 0x00) {
	return 0;
	}
	else if (flag & 0x02) {
	return 1;
	}
	else {
	return 2;
	}
}
static int check_otp_lenc(int index)
{
	int flag;
	int nFlagAddress = OTP_DRV_START_ADDR+1+OTP_DRV_INFO_GROUP_COUNT*OTP_DRV_INFO_SIZE
	+1+OTP_DRV_AWB_GROUP_COUNT*OTP_DRV_AWB_SIZE
	+1+OTP_DRV_VCM_GROUP_COUNT*OTP_DRV_VCM_SIZE ;
	OV13850TRULY_write_cmos_sensor(0x3d84, 0xC0);
	//partial mode OTP write start address
	OV13850TRULY_write_cmos_sensor(0x3d88, (nFlagAddress>>8) & 0xff );
	OV13850TRULY_write_cmos_sensor(0x3d89, nFlagAddress & 0xff);
	// partial mode OTP write end address
	OV13850TRULY_write_cmos_sensor(0x3d8A, (nFlagAddress>>8) & 0xff );
	OV13850TRULY_write_cmos_sensor(0x3d8B, nFlagAddress & 0xff);
	// read otp into buffer
	OV13850TRULY_write_cmos_sensor(0x3d81, 0x01);
	mDELAY(5);
	flag = OV13850TRULY_read_cmos_sensor(nFlagAddress);
	if (index == 1)
	{
	flag = (flag>>6) & 0x03;
	}
	else if (index == 2)
	{
	flag = (flag>>4) & 0x03;
	}
	else if (index == 3)
	{
	flag = (flag>> 2)& 0x03;
	}
	// clear otp buffer
	OV13850TRULY_write_cmos_sensor(nFlagAddress, 0x00);
	if (flag == 0x00) {
	return 0;
	}
	else if (flag & 0x02) {
	return 1;
	}
	else {
	return 2;
	}
}

static int check_otp_VCM(int index, int code)
{
	int flag;
	int nFlagAddress= OTP_DRV_START_ADDR+1+OTP_DRV_INFO_GROUP_COUNT*OTP_DRV_INFO_SIZE
	+1+OTP_DRV_AWB_GROUP_COUNT*OTP_DRV_AWB_SIZE;
	OV13850TRULY_write_cmos_sensor(0x3d84, 0xC0);
	//partial mode OTP write start address
	OV13850TRULY_write_cmos_sensor(0x3d88, (nFlagAddress>>8) & 0xff );
	OV13850TRULY_write_cmos_sensor(0x3d89, nFlagAddress & 0xff);
	// partial mode OTP write end address
	OV13850TRULY_write_cmos_sensor(0x3d8A, (nFlagAddress>>8) & 0xff );
	OV13850TRULY_write_cmos_sensor(0x3d8B, nFlagAddress & 0xff);
	// read otp into buffer
	OV13850TRULY_write_cmos_sensor(0x3d81, 0x01);
	mDELAY(5);
	//select group
	flag = OV13850TRULY_read_cmos_sensor(nFlagAddress);
	if (index == 1)
	{
	flag = (flag>>6) & 0x03;
	}
	else if (index == 2)
	{
	flag = (flag>>4) & 0x03;
	}
	else if (index == 3)
	{
	flag = (flag>>2) & 0x03;
	}
	// clear otp buffer
	OV13850TRULY_write_cmos_sensor(nFlagAddress, 0x00);
	if (flag == 0x00) {
	return 0;
	}
	else if (flag & 0x02) {
	return 1;
	}
	else {
	return 2;
	}
}
static int read_otp_info(int index, struct otp_struct *otp_ptr)
{
	int i;
	int nFlagAddress = OTP_DRV_START_ADDR;
	int start_addr, end_addr;
	start_addr = nFlagAddress+1+(index-1)*OTP_DRV_INFO_SIZE;
	end_addr = start_addr+OTP_DRV_INFO_SIZE-1;
	OV13850TRULY_write_cmos_sensor(0x3d84, 0xC0);
	//partial mode OTP write start address
	OV13850TRULY_write_cmos_sensor(0x3d88, (start_addr >> 8) & 0xff);
	OV13850TRULY_write_cmos_sensor(0x3d89, start_addr & 0xff);
	// partial mode OTP write end address
	OV13850TRULY_write_cmos_sensor(0x3d8A, (end_addr >> 8) & 0xff);
	OV13850TRULY_write_cmos_sensor(0x3d8B, end_addr & 0xff);
	// read otp into buffer
	OV13850TRULY_write_cmos_sensor(0x3d81, 0x01);
	mDELAY(5);
	(*otp_ptr).module_integrator_id = OV13850TRULY_read_cmos_sensor(start_addr);
	(*otp_ptr).lens_id = OV13850TRULY_read_cmos_sensor(start_addr + 1);
	(*otp_ptr).production_year = OV13850TRULY_read_cmos_sensor(start_addr + 2);
	(*otp_ptr).production_month = OV13850TRULY_read_cmos_sensor(start_addr + 3);
	(*otp_ptr).production_day = OV13850TRULY_read_cmos_sensor(start_addr + 4);
	// clear otp buffer
	for (i=start_addr; i<=end_addr; i++) {
	OV13850TRULY_write_cmos_sensor(i, 0x00);
}
return 0;
}
static int read_otp_wb(int index, struct otp_struct * otp_ptr)
{
	int i;
	int temp;
	int start_addr, end_addr;
	int nFlagAddress = OTP_DRV_START_ADDR+1+OTP_DRV_INFO_GROUP_COUNT*OTP_DRV_INFO_SIZE;
	start_addr = nFlagAddress+1+(index-1)* OTP_DRV_AWB_SIZE;
	end_addr = start_addr+OTP_DRV_AWB_SIZE;
	OV13850TRULY_write_cmos_sensor(0x3d84, 0xC0);
	//partial mode OTP write start address
	OV13850TRULY_write_cmos_sensor(0x3d88, (start_addr >> 8) & 0xff);
	OV13850TRULY_write_cmos_sensor(0x3d89, start_addr & 0xff);
	// partial mode OTP write end address
	OV13850TRULY_write_cmos_sensor(0x3d8A, (end_addr >> 8) & 0xff);
	OV13850TRULY_write_cmos_sensor(0x3d8B, end_addr & 0xff);
	// read otp into buffer
	OV13850TRULY_write_cmos_sensor(0x3d81, 0x01);
	mDELAY(5);
	temp = OV13850TRULY_read_cmos_sensor(start_addr + 4);
	(*otp_ptr).rg_ratio = (OV13850TRULY_read_cmos_sensor(start_addr)<<2) + ((temp>>6) & 0x03);
	(*otp_ptr).bg_ratio = (OV13850TRULY_read_cmos_sensor(start_addr + 1)<<2) + ((temp>>4) & 0x03);
	(*otp_ptr).light_rg = (OV13850TRULY_read_cmos_sensor(start_addr + 2) <<2) + ((temp>>2) & 0x03);
	(*otp_ptr).light_bg = (OV13850TRULY_read_cmos_sensor(start_addr + 3)<<2) + (temp & 0x03);
	// clear otp buffer
	for (i=start_addr; i<=end_addr; i++) {
	OV13850TRULY_write_cmos_sensor(i, 0x00);
	}

return 0;
}

static int read_otp_VCM(int index, struct otp_struct * otp_ptr)
{
	int i;
	int temp;
	int start_addr, end_addr;
	int nFlagAddress = OTP_DRV_START_ADDR+1+OTP_DRV_INFO_GROUP_COUNT*OTP_DRV_INFO_SIZE
	+1+OTP_DRV_AWB_GROUP_COUNT*OTP_DRV_AWB_SIZE;
	start_addr = nFlagAddress+1+(index-1)*OTP_DRV_VCM_SIZE;
	end_addr = start_addr+OTP_DRV_VCM_SIZE-1;
	OV13850TRULY_write_cmos_sensor(0x3d84, 0xC0);
	//partial mode OTP write start address
	OV13850TRULY_write_cmos_sensor(0x3d88, (start_addr >> 8) & 0xff);
	OV13850TRULY_write_cmos_sensor(0x3d89, start_addr & 0xff);
	// partial mode OTP write end address
	OV13850TRULY_write_cmos_sensor(0x3d8A, (end_addr >> 8) & 0xff);
	OV13850TRULY_write_cmos_sensor(0x3d8B, end_addr & 0xff);
	// read otp into buffer
	OV13850TRULY_write_cmos_sensor(0x3d81, 0x01);
	mDELAY(5);
	//flag and lsb of VCM start code
	temp = OV13850TRULY_read_cmos_sensor(start_addr+2);
	(* otp_ptr).VCM_start = (OV13850TRULY_read_cmos_sensor(start_addr)<<2) | ((temp>>6) & 0x03);
	(* otp_ptr).VCM_end = (OV13850TRULY_read_cmos_sensor(start_addr + 1) << 2) | ((temp>>4) & 0x03);
	(* otp_ptr).VCM_dir = (temp>>2) & 0x03;
	// clear otp buffer
	for (i=start_addr; i<=end_addr; i++) {
	OV13850TRULY_write_cmos_sensor(i, 0x00);
	}
return 0;
}
static int read_otp_lenc(int index, struct otp_struct * otp_ptr)
{
	int i;
	int start_addr, end_addr;
	int nFlagAddress= OTP_DRV_START_ADDR+1+OTP_DRV_INFO_GROUP_COUNT* OTP_DRV_INFO_SIZE
	+1+OTP_DRV_AWB_GROUP_COUNT* OTP_DRV_AWB_SIZE
	+1+OTP_DRV_VCM_GROUP_COUNT* OTP_DRV_VCM_SIZE ;
	start_addr = nFlagAddress+1+(index-1)*OTP_DRV_LSC_SIZE ;
	end_addr = start_addr+OTP_DRV_LSC_SIZE-1;
	OV13850TRULY_write_cmos_sensor(0x3d84, 0xC0);
	//partial mode OTP write start address
	OV13850TRULY_write_cmos_sensor(0x3d88, (start_addr >> 8) & 0xff);
	OV13850TRULY_write_cmos_sensor(0x3d89, start_addr & 0xff);
	// partial mode OTP write end address
	OV13850TRULY_write_cmos_sensor(0x3d8A, (end_addr >> 8) & 0xff);
	OV13850TRULY_write_cmos_sensor(0x3d8B, end_addr & 0xff);
	// read otp into buffer
	OV13850TRULY_write_cmos_sensor(0x3d81, 0x01);
	mDELAY(10);
	for(i=0; i<OTP_DRV_LSC_SIZE; i++) {
	(* otp_ptr).lenc[i] = OV13850TRULY_read_cmos_sensor(start_addr + i);
	
	OV13850TRULYDB("i is : %d, lenc is : 0x%x \n ",i,(* otp_ptr).lenc[i]);

	}
	// clear otp buffer
	for (i=start_addr; i<=end_addr; i++) {
	OV13850TRULY_write_cmos_sensor(i, 0x00);
	}
return 0;
}
// R_gain, sensor red gain of AWB, 0x400 =1
// G_gain, sensor green gain of AWB, 0x400 =1
// B_gain, sensor blue gain of AWB, 0x400 =1
// return 0;
static int update_awb_gain(int R_gain, int G_gain, int B_gain)
{
	if (R_gain>0x400) {
	OV13850TRULY_write_cmos_sensor(0x5056, R_gain>>8);
	OV13850TRULY_write_cmos_sensor(0x5057, R_gain & 0x00ff);
	}
	if (G_gain>0x400) {
	OV13850TRULY_write_cmos_sensor(0x5058, G_gain>>8);
	OV13850TRULY_write_cmos_sensor(0x5059, G_gain & 0x00ff);
	}
	if (B_gain>0x400) {
	OV13850TRULY_write_cmos_sensor(0x505A, B_gain>>8);
	OV13850TRULY_write_cmos_sensor(0x505B, B_gain & 0x00ff);
	}
	return 0;
	// otp_ptr: pointer of otp_struct
}

static int update_lenc(struct otp_struct * otp_ptr)
{
	int i, temp;
	temp = OV13850TRULY_read_cmos_sensor(0x5000);
	temp = 0x01 | temp;
	OV13850TRULY_write_cmos_sensor(0x5000, temp);
	for(i=0;i<OTP_DRV_LSC_SIZE ;i++) {
	OV13850TRULY_write_cmos_sensor(OTP_DRV_LSC_REG_ADDR + i, (*otp_ptr).lenc[i]);
	}
	return 0;
}

// call this function after OV13850TRULY initialization
// return value: 0 update success
// 1, no OTP
static int update_otp_wb()
{
	struct otp_struct current_otp;
	int i;
	int otp_index;
	int temp;
	int rg,bg;
	int R_gain, G_gain, B_gain;
	// R/G and B/G of current camera module is read out from sensor OTP
	// check first OTP with valid data
	for(i=1;i<=OTP_DRV_AWB_GROUP_COUNT;i++) {
	temp = check_otp_wb(i);
	if (temp == 2) {
	otp_index = i;
	break;
	}
	}
	if (i>OTP_DRV_AWB_GROUP_COUNT) {
	// no valid wb OTP data
	return 1;
	}
	read_otp_wb(otp_index, &current_otp);
	if(current_otp.light_rg==0) {
	// no light source information in OTP, light factor = 1
	rg = current_otp.rg_ratio;
	}
	else {
	rg = current_otp.rg_ratio * (current_otp.light_rg +512) / 1024;
	}
	if(current_otp.light_bg==0) {
	// not light source information in OTP, light factor = 1
	bg = current_otp.bg_ratio;
	}
	else {
	bg = current_otp.bg_ratio * (current_otp.light_bg +512) / 1024;
	}

	OV13850TRULYDB("rg=0x%x\n ",rg);
	OV13850TRULYDB("bg=0x%x\n ",bg);

	//calculate G gain
	int nR_G_gain, nB_G_gain, nG_G_gain;
	int nBase_gain;
	nR_G_gain = (RG_Ratio_Typical*1000) / rg;
	nB_G_gain = (BG_Ratio_Typical*1000) / bg;
	nG_G_gain = 1000;
	if (nR_G_gain < 1000 || nB_G_gain < 1000)
	{
	if (nR_G_gain < nB_G_gain)
	nBase_gain = nR_G_gain;
	else
	nBase_gain = nB_G_gain;
	}
	else
	{
	nBase_gain = nG_G_gain;
	}
	R_gain = 0x400 * nR_G_gain / (nBase_gain);
	B_gain = 0x400 * nB_G_gain / (nBase_gain);
	G_gain = 0x400 * nG_G_gain / (nBase_gain);

	OV13850TRULYDB("R_gain=0x%x\n ",R_gain);
	OV13850TRULYDB("B_gain=0x%x\n ",B_gain);
	OV13850TRULYDB("G_gain=0x%x\n ",G_gain);

	update_awb_gain(R_gain, G_gain, B_gain);
	return 0;

}

// call this function after OV13850TRULY initialization
// return value: 0 update success
// 1, no OTP
static int update_otp_lenc()
{
	struct otp_struct current_otp;
	int i;
	int otp_index;
	int temp;
	// check first lens correction OTP with valid data
	for(i=1;i<=OTP_DRV_LSC_GROUP_COUNT;i++) {
	temp = check_otp_lenc(i);	
	OV13850TRULYDB("temp is : %d, i is :%d \n ",temp, i);
	if (temp == 2) {
	otp_index = i;
	break;}
	}
	if (i>OTP_DRV_LSC_GROUP_COUNT) {
	// no valid WB OTP data
	return 1;
	}
	read_otp_lenc(otp_index, &current_otp);
	update_lenc(&current_otp);
	// success
	return 0;
}

#endif


kal_uint16 OV13850TRULY_read_cmos_sensor(kal_uint32 addr)
{
kal_uint16 get_byte=0;
    iReadReg((u16) addr ,(u8*)&get_byte,OV13850TRULY_WRITE_ID);
    return get_byte;
}

#define Sleep(ms) mdelay(ms)

void OV13850TRULY_write_shutter(kal_uint32 shutter)
{

#if 1
	kal_uint32 min_framelength = OV13850TRULY_PV_PERIOD_PIXEL_NUMS, max_shutter=0;
	kal_uint32 extra_lines = 0;
	kal_uint32 line_length = 0;
	kal_uint32 frame_length = 0;
	unsigned long flags;

	OV13850TRULYDBSOFIA("!!shutter=%d!!!!!\n", shutter);

	if(ov13850truly.OV13850TRULYAutoFlickerMode == KAL_TRUE)
	{
		if ( SENSOR_MODE_PREVIEW == ov13850truly.sensorMode )  //(g_iOV13850TRULY_Mode == OV13850TRULY_MODE_PREVIEW)	//SXGA size output
		{
			line_length = OV13850TRULY_PV_PERIOD_PIXEL_NUMS + ov13850truly.DummyPixels;
			max_shutter = OV13850TRULY_PV_PERIOD_LINE_NUMS + ov13850truly.DummyLines ;
		}
		else if( SENSOR_MODE_VIDEO == ov13850truly.sensorMode ) //add for video_6M setting
		{
			line_length = OV13850TRULY_VIDEO_PERIOD_PIXEL_NUMS + ov13850truly.DummyPixels;
			max_shutter = OV13850TRULY_VIDEO_PERIOD_LINE_NUMS + ov13850truly.DummyLines ;
		}
		else
		{
			line_length = OV13850TRULY_FULL_PERIOD_PIXEL_NUMS + ov13850truly.DummyPixels;
			max_shutter = OV13850TRULY_FULL_PERIOD_LINE_NUMS + ov13850truly.DummyLines ;
		}
		//OV13850TRULYDBSOFIA("linelength %d, max_shutter %d!!\n",line_length,max_shutter);

		switch(OV13850TRULYCurrentScenarioId)
		{
        	case MSDK_SCENARIO_ID_CAMERA_ZSD:
			case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
				//OV13850TRULYDBSOFIA("AutoFlickerMode!!! MSDK_SCENARIO_ID_CAMERA_ZSD  0!!\n");
				min_framelength = max_shutter;
				break;
			case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
				
				//OV13850TRULYDBSOFIA("AutoFlickerMode!!! MSDK_SCENARIO_ID_VIDEO_PREVIEW  0!!\n");
				if( ov13850trulyVIDEO_MODE_TARGET_FPS==30)
				{
					min_framelength = (OV13850TRULYMIPI_VIDEO_CLK) /(OV13850TRULY_VIDEO_PERIOD_PIXEL_NUMS + ov13850truly.DummyPixels)/306*10 ;
				}
				else if( ov13850trulyVIDEO_MODE_TARGET_FPS==15)
				{
					min_framelength = (OV13850TRULYMIPI_VIDEO_CLK) /(OV13850TRULY_VIDEO_PERIOD_PIXEL_NUMS + ov13850truly.DummyPixels)/148*10 ;
				}
				else
				{
					min_framelength = max_shutter;
				}
				break;
			default:
				//min_framelength = (OV13850TRULYMIPI_PREVIEW_CLK) /(OV13850TRULY_PV_PERIOD_PIXEL_NUMS + ov13850truly.DummyPixels)/294*10 ;
				min_framelength = (OV13850TRULYMIPI_PREVIEW_CLK) /(OV13850TRULY_PV_PERIOD_PIXEL_NUMS + ov13850truly.DummyPixels)/303*10 ;
    			break;
		}

		//OV13850TRULYDBSOFIA("AutoFlickerMode!!! min_framelength for AutoFlickerMode = %d (0x%x)\n",min_framelength,min_framelength);
		//OV13850TRULYDBSOFIA("max framerate(10 base) autofilker = %d\n",(OV13850TRULYMIPI_PREVIEW_CLK)*10 /line_length/min_framelength);

		if (shutter < 3)
			shutter = 3;

		if (shutter > (max_shutter-8))
			extra_lines = shutter -( max_shutter - 8);
		else
			extra_lines = 0;
		//OV13850TRULYDBSOFIA("extra_lines 0=%d!!\n",extra_lines);

		if ( SENSOR_MODE_PREVIEW == ov13850truly.sensorMode )	//SXGA size output
		{
			frame_length = OV13850TRULY_PV_PERIOD_LINE_NUMS+ ov13850truly.DummyLines + extra_lines ;
		}
		else if(SENSOR_MODE_VIDEO == ov13850truly.sensorMode)
		{
			frame_length = OV13850TRULY_VIDEO_PERIOD_LINE_NUMS+ ov13850truly.DummyLines + extra_lines ;
		}
		else				//QSXGA size output
		{
			frame_length = OV13850TRULY_FULL_PERIOD_LINE_NUMS + ov13850truly.DummyLines + extra_lines ;
		}
		//OV13850TRULYDBSOFIA("frame_length 0= %d\n",frame_length);

		if (frame_length < min_framelength)
		{
			//shutter = min_framelength - 4;

			switch(OV13850TRULYCurrentScenarioId)
			{
        	case MSDK_SCENARIO_ID_CAMERA_ZSD:
			case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
				extra_lines = min_framelength- (OV13850TRULY_FULL_PERIOD_LINE_NUMS+ ov13850truly.DummyLines);
				break;
			case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
				extra_lines = min_framelength- (OV13850TRULY_VIDEO_PERIOD_LINE_NUMS+ ov13850truly.DummyLines);
				break;//need check
			default:
				extra_lines = min_framelength- (OV13850TRULY_PV_PERIOD_LINE_NUMS+ ov13850truly.DummyLines);
    			break;
			}
			frame_length = min_framelength;
		}
		//Set total frame length
		if (frame_length >= 0x8000)
			frame_length = 0x7fff;
		#ifdef SHUTTER_MWRITE
		ov13850truly_framelength[2]=(frame_length >> 8) & 0x7F;
		ov13850truly_framelength[5]=frame_length & 0xFF;
		memset(pSBuf,0, 1024);
		memcpy(pSBuf, ov13850truly_framelength, ov13850truly_framelength_len);
		OV13850TRULY_multi_write_cmos_sensor(sdmaHandle, ov13850truly_framelength_len); 
		#else
		OV13850TRULY_write_cmos_sensor(0x380e, (frame_length >> 8) & 0x7F);
		OV13850TRULY_write_cmos_sensor(0x380f, frame_length & 0xFF);
		#endif
		spin_lock_irqsave(&ov13850trulymipiraw_drv_lock,flags);
		ov13850truly.maxExposureLines = frame_length-8;
		OV13850TRULY_FeatureControl_PERIOD_PixelNum = line_length;
		OV13850TRULY_FeatureControl_PERIOD_LineNum = frame_length;
		spin_unlock_irqrestore(&ov13850trulymipiraw_drv_lock,flags);

		//Set shutter (Coarse integration time, uint: lines.)
		if (shutter > 0x7ff8)
			shutter = 0x7ff8;
		#ifdef SHUTTER_MWRITE
		ov13850truly_shutter[2]=((shutter>>12) & 0x0F);
		ov13850truly_shutter[5]=((shutter>>4) & 0xFF);
		ov13850truly_shutter[8]=((shutter<<4) & 0xF0);
		memset(pSBuf,0, 1024);
		memcpy(pSBuf, ov13850truly_shutter, ov13850truly_shutter_len);
		OV13850TRULY_multi_write_cmos_sensor(sdmaHandle, ov13850truly_shutter_len); 
		#else
		OV13850TRULY_write_cmos_sensor(0x3500, (shutter>>12) & 0x0F);
		OV13850TRULY_write_cmos_sensor(0x3501, (shutter>>4) & 0xFF);
		OV13850TRULY_write_cmos_sensor(0x3502, (shutter<<4) & 0xF0);	/* Don't use the fraction part. */
		#endif
	//OV13850TRULYDBSOFIA("frame_length %d,shutter %d!!\n",frame_length,shutter);
		//OV13850TRULYDB("framerate(10 base) = %d\n",(OV13850TRULYMIPI_PREVIEW_CLK)*10 /line_length/frame_length);
		//OV13850TRULYDB("shutter=%d, extra_lines=%d, line_length=%d, frame_length=%d\n", shutter, extra_lines, line_length, frame_length);

	}
	else
	{
		if ( SENSOR_MODE_PREVIEW == ov13850truly.sensorMode )  //(g_iOV13850TRULY_Mode == OV13850TRULY_MODE_PREVIEW)	//SXGA size output
		{
			max_shutter = OV13850TRULY_PV_PERIOD_LINE_NUMS + ov13850truly.DummyLines ;
		}
		else if( SENSOR_MODE_VIDEO == ov13850truly.sensorMode ) //add for video_6M setting
		{
			max_shutter = OV13850TRULY_VIDEO_PERIOD_LINE_NUMS + ov13850truly.DummyLines ;
		}
		else
		{
			max_shutter = OV13850TRULY_FULL_PERIOD_LINE_NUMS + ov13850truly.DummyLines ;
		}
		//OV13850TRULYDBSOFIA(" max_shutter %d!!\n",max_shutter);

		if (shutter < 3)
			shutter = 3;

		if (shutter > (max_shutter-8))
			extra_lines = shutter - (max_shutter -8);
		else
			extra_lines = 0;
		//OV13850TRULYDBSOFIA("extra_lines 0=%d!!\n",extra_lines);

		if ( SENSOR_MODE_PREVIEW == ov13850truly.sensorMode )	//SXGA size output
		{
			line_length = OV13850TRULY_PV_PERIOD_PIXEL_NUMS + ov13850truly.DummyPixels;
			frame_length = OV13850TRULY_PV_PERIOD_LINE_NUMS+ ov13850truly.DummyLines + extra_lines ;
		}
		else if( SENSOR_MODE_VIDEO == ov13850truly.sensorMode )
		{
			line_length = OV13850TRULY_VIDEO_PERIOD_PIXEL_NUMS + ov13850truly.DummyPixels;
			frame_length = OV13850TRULY_VIDEO_PERIOD_LINE_NUMS + ov13850truly.DummyLines + extra_lines ;
		}
		else				//QSXGA size output
		{
			line_length = OV13850TRULY_FULL_PERIOD_PIXEL_NUMS + ov13850truly.DummyPixels;
			frame_length = OV13850TRULY_FULL_PERIOD_LINE_NUMS + ov13850truly.DummyLines + extra_lines ;
		}

		ASSERT(line_length < OV13850TRULY_MAX_LINE_LENGTH);		//0xCCCC
		ASSERT(frame_length < OV13850TRULY_MAX_FRAME_LENGTH); 	//0xFFFF
		
		//Set total frame length
		if (frame_length >= 0x8000)
			frame_length = 0x7fff;
		#ifdef SHUTTER_MWRITE
		ov13850truly_framelength[2]=(frame_length >> 8) & 0x7F;
		ov13850truly_framelength[5]=frame_length & 0xFF;
		memset(pSBuf,0, 1024);
		memcpy(pSBuf, ov13850truly_framelength, ov13850truly_framelength_len);
		OV13850TRULY_multi_write_cmos_sensor(sdmaHandle, ov13850truly_framelength_len); 
		#else
		OV13850TRULY_write_cmos_sensor(0x380e, (frame_length >> 8) & 0x7F);
		OV13850TRULY_write_cmos_sensor(0x380f, frame_length & 0xFF);
		#endif
		spin_lock_irqsave(&ov13850trulymipiraw_drv_lock,flags);
		ov13850truly.maxExposureLines = frame_length -8;
		OV13850TRULY_FeatureControl_PERIOD_PixelNum = line_length;
		OV13850TRULY_FeatureControl_PERIOD_LineNum = frame_length;
		spin_unlock_irqrestore(&ov13850trulymipiraw_drv_lock,flags);

		//Set shutter (Coarse integration time, uint: lines.)
		if (shutter > 0x7ff8)
			shutter = 0x7ff8;
		#ifdef SHUTTER_MWRITE
		ov13850truly_shutter[2]=((shutter>>12) & 0x0F);
		ov13850truly_shutter[5]=((shutter>>4) & 0xFF);
		ov13850truly_shutter[8]=((shutter<<4) & 0xF0);
		memset(pSBuf,0, 1024);
		memcpy(pSBuf, ov13850truly_shutter, ov13850truly_shutter_len);
		OV13850TRULY_multi_write_cmos_sensor(sdmaHandle, ov13850truly_shutter_len); 
		#else
		OV13850TRULY_write_cmos_sensor(0x3500, (shutter>>12) & 0x0F);
		OV13850TRULY_write_cmos_sensor(0x3501, (shutter>>4) & 0xFF);
		OV13850TRULY_write_cmos_sensor(0x3502, (shutter<<4) & 0xF0);	/* Don't use the fraction part. */
		#endif
		//OV13850TRULYDBSOFIA("frame_length %d,shutter %d!!\n",frame_length,shutter);

		//OV13850TRULYDB("framerate(10 base) = %d\n",(OV13850TRULYMIPI_PREVIEW_CLK)*10 /line_length/frame_length);

		//OV13850TRULYDB("shutter=%d, extra_lines=%d, line_length=%d, frame_length=%d\n", shutter, extra_lines, line_length, frame_length);
	}
#endif
}   /* write_OV13850TRULY_shutter */

static kal_uint16 OV13850TRULYReg2Gain(const kal_uint16 iReg)
{
    kal_uint8 iI;
    kal_uint16 iGain = ov13850truly.ispBaseGain;    // 1x-gain base

    // Range: 1x to 32x
    // Gain = (GAIN[9] + 1) *(GAIN[8] + 1) *(GAIN[7] + 1) * (GAIN[6] + 1) * (GAIN[5] + 1) * (GAIN[4] + 1) * (1 + GAIN[3:0] / 16)
    //for (iI = 8; iI >= 4; iI--) {
    //    iGain *= (((iReg >> iI) & 0x01) + 1);
    //}
    iGain = iReg * ov13850truly.ispBaseGain / 32;
    return iGain; //ov13850truly.realGain
}

static kal_uint16 OV13850TRULYGain2Reg(const kal_uint16 Gain)
{
    kal_uint16 iReg = 0x0000;
	kal_uint16 iGain=Gain;
	//if(iGain <  ov13850truly.ispBaseGain) 
	//{
		iReg = Gain*32/ov13850truly.ispBaseGain;
		if(iReg < 0x20)
		{
			iReg = 0x20;
		}
		if(iReg > 0xfc)
		{
			iReg = 0xfc;
		}
	//}
	//else
	//{
	//	OV13850TRULYDB("out of range!\n");
	//}
	OV13850TRULYDBSOFIA("[OV13850TRULYGain2Reg]: isp gain:%d,sensor gain:0x%x\n",iGain,iReg);

    return iReg;//ov13850truly. sensorGlobalGain

}

void write_OV13850TRULY_gain(kal_uint16 gain)
{

#ifdef SHUTTER_MWRITE
	ov13850truly_gain[2]=(gain>>8);
	ov13850truly_gain[5]=(gain&0xff);
	memset(pSBuf,0, 1024);
	memcpy(pSBuf, ov13850truly_gain, ov13850truly_gain_len);
	OV13850TRULY_multi_write_cmos_sensor(sdmaHandle, ov13850truly_gain_len); 
#else
	OV13850TRULY_write_cmos_sensor(0x350a,(gain>>8));
	OV13850TRULY_write_cmos_sensor(0x350b,(gain&0xff));
#endif
	return;
}
void OV13850TRULY_SetGain(UINT16 iGain)
{
	unsigned long flags;
	spin_lock_irqsave(&ov13850trulymipiraw_drv_lock,flags);
	ov13850truly.realGain = iGain;
	ov13850truly.sensorGlobalGain = OV13850TRULYGain2Reg(iGain);
	spin_unlock_irqrestore(&ov13850trulymipiraw_drv_lock,flags);
	write_OV13850TRULY_gain(ov13850truly.sensorGlobalGain);
	OV13850TRULYDB("[OV13850TRULY_SetGain]ov13850truly.sensorGlobalGain=0x%x,ov13850truly.realGain=%d\n",ov13850truly.sensorGlobalGain,ov13850truly.realGain);

}   /*  OV13850TRULY_SetGain_SetGain  */

kal_uint16 read_OV13850TRULY_gain(void)
{
	kal_uint16 read_gain=0;
	read_gain=(((OV13850TRULY_read_cmos_sensor(0x350a)&0x01) << 8) | OV13850TRULY_read_cmos_sensor(0x350b));
	spin_lock(&ov13850trulymipiraw_drv_lock);
	ov13850truly.sensorGlobalGain = read_gain;
	ov13850truly.realGain = OV13850TRULYReg2Gain(ov13850truly.sensorGlobalGain);
	spin_unlock(&ov13850trulymipiraw_drv_lock);
	OV13850TRULYDB("ov13850truly.sensorGlobalGain=0x%x,ov13850truly.realGain=%d\n",ov13850truly.sensorGlobalGain,ov13850truly.realGain);
	return ov13850truly.sensorGlobalGain;
}  /* read_OV13850TRULY_gain */


void OV13850TRULY_camera_para_to_sensor(void)
{}

void OV13850TRULY_sensor_to_camera_para(void)
{}

kal_int32  OV13850TRULY_get_sensor_group_count(void)
{
    return GROUP_TOTAL_NUMS;
}

void OV13850TRULY_get_sensor_group_info(kal_uint16 group_idx, kal_int8* group_name_ptr, kal_int32* item_count_ptr)
{}
void OV13850TRULY_get_sensor_item_info(kal_uint16 group_idx,kal_uint16 item_idx, MSDK_SENSOR_ITEM_INFO_STRUCT* info_ptr)
{}
kal_bool OV13850TRULY_set_sensor_item_info(kal_uint16 group_idx, kal_uint16 item_idx, kal_int32 ItemValue)
{    return KAL_TRUE;}

static void OV13850TRULY_SetDummy( const kal_uint32 iPixels, const kal_uint32 iLines )
{
 	kal_uint32 line_length = 0;
	kal_uint32 frame_length = 0;

	if ( SENSOR_MODE_PREVIEW == ov13850truly.sensorMode )	//SXGA size output
	{
		line_length = OV13850TRULY_PV_PERIOD_PIXEL_NUMS + iPixels;
		frame_length = OV13850TRULY_PV_PERIOD_LINE_NUMS + iLines;
	}
	else if( SENSOR_MODE_VIDEO== ov13850truly.sensorMode )
	{
		line_length = OV13850TRULY_VIDEO_PERIOD_PIXEL_NUMS + iPixels;
		frame_length = OV13850TRULY_VIDEO_PERIOD_LINE_NUMS + iLines;
	}
	else//QSXGA size output
	{
		line_length = OV13850TRULY_FULL_PERIOD_PIXEL_NUMS + iPixels;
		frame_length = OV13850TRULY_FULL_PERIOD_LINE_NUMS + iLines;
	}

	//if(ov13850truly.maxExposureLines > frame_length -4 )
	//	return;

	//ASSERT(line_length < OV13850TRULY_MAX_LINE_LENGTH);		//0xCCCC
	//ASSERT(frame_length < OV13850TRULY_MAX_FRAME_LENGTH);	//0xFFFF

	//Set total frame length
	if (frame_length >= 0x8000)
			frame_length = 0x7fff;
	OV13850TRULY_write_cmos_sensor(0x380e, (frame_length >> 8) & 0x7F);
	OV13850TRULY_write_cmos_sensor(0x380f, frame_length & 0xFF);

	spin_lock(&ov13850trulymipiraw_drv_lock);
	ov13850truly.maxExposureLines = frame_length -8;
	OV13850TRULY_FeatureControl_PERIOD_PixelNum = line_length;
	OV13850TRULY_FeatureControl_PERIOD_LineNum = frame_length;
	spin_unlock(&ov13850trulymipiraw_drv_lock);

	//Set total line length
	OV13850TRULY_write_cmos_sensor(0x380c, (line_length >> 8) & 0x7F);
	OV13850TRULY_write_cmos_sensor(0x380d, line_length & 0xFF);

	
	OV13850TRULYDB("OV13850TRULY_SetDummy linelength %d,  frame_length= %d \n",line_length,frame_length);

}   /*  OV13850TRULY_SetDummy */



static kal_uint8 ov13850truly_init[] = {
	//for PCLK :384M or fast:pclk:240M
	/*
	
	@@ RES_4208x3120 24fps
	;24Mhz Xclk
	;SCLK 96Mhz, Pclk 384MHz
	;4Lane, MIPI datarate 960Mbps/Lane
	;24fps
	;pixels per line=4800(0x12c0) 
	;lines per frame=3328(0xD00)
	
	;102 2630 960
	;88 e7 3f
	
	;100 99 4208 3120
	;100 98 1 0
	;102 81 0
	;102 3601 964
	
	;102 910 31
	;102 84 1
	
	;c8 0300 62
	*/
	//0x01, 0x03, 0x01,//reset , need delay
	0x03, 0x03, 0x00,
	0x03, 0x02, 0x28, 
	0x03, 0x0a, 0x00, 
	0x30, 0x0f, 0x11, 
	0x30, 0x10, 0x01, 
	0x30, 0x11, 0x76, 
	0x30, 0x12, 0x41, 
	0x30, 0x13, 0x12, 
	0x30, 0x14, 0x11, 
	0x30, 0x1f, 0x03, 
	0x31, 0x06, 0x00, 
	0x32, 0x10, 0x47, 
	0x35, 0x00, 0x00, 
	0x35, 0x01, 0xc0, 
	0x35, 0x02, 0x00, 
	0x35, 0x06, 0x00, 
	0x35, 0x07, 0x02, 
	0x35, 0x08, 0x00, 
	0x35, 0x0a, 0x00, 
	0x35, 0x0b, 0x80, 
	0x35, 0x0e, 0x00, 
	0x35, 0x0f, 0x10, 
	0x36, 0x00, 0x40, 
	0x36, 0x01, 0xfc, 
	0x36, 0x02, 0x02, 
	0x36, 0x03, 0x48, 
	0x36, 0x04, 0xa5, 
	0x36, 0x05, 0x9f, 
	0x36, 0x07, 0x00, 
	0x36, 0x0a, 0x40, 
	0x36, 0x0b, 0x91, 
	0x36, 0x0c, 0x49, 
	0x36, 0x0f, 0x8a, 
	0x36, 0x11, 0x10, 
	0x36, 0x12, 0x33, 
	0x36, 0x13, 0x33, 
	0x36, 0x15, 0x08, 
	0x36, 0x41, 0x02, 
	0x36, 0x60, 0x82, 
	0x36, 0x68, 0x54, 
	0x36, 0x69, 0x40, 
	0x36, 0x67, 0xa0, 
	0x37, 0x02, 0x40, 
	0x37, 0x03, 0x44, 
	0x37, 0x04, 0x2c, 
	0x37, 0x05, 0x24, 
	0x37, 0x06, 0x50, 
	0x37, 0x07, 0x44, 
	0x37, 0x08, 0x3c, 
	0x37, 0x09, 0x1f, 
	0x37, 0x0a, 0x24, 
	0x37, 0x0b, 0x3c, 
	0x37, 0x20, 0x66, 
	0x37, 0x22, 0x84, 
	0x37, 0x28, 0x40, 
	0x37, 0x2a, 0x04, 
	0x37, 0x2f, 0xa0, 
	0x37, 0x10, 0x28, 
	0x37, 0x16, 0x03, 
	0x37, 0x18, 0x10, 
	0x37, 0x19, 0x08, 
	0x37, 0x1c, 0xfc, 
	0x37, 0x60, 0x13, 
	0x37, 0x61, 0x34, 
	0x37, 0x67, 0x24, 
	0x37, 0x68, 0x06, 
	0x37, 0x69, 0x45, 
	0x37, 0x6c, 0x23, 
	0x3d, 0x84, 0x00, 
	0x3d, 0x85, 0x17, 
	0x3d, 0x8c, 0x73, 
	0x3d, 0x8d, 0xbf, 
	0x38, 0x00, 0x00, 
	0x38, 0x01, 0x14, 
	0x38, 0x02, 0x00, 
	0x38, 0x03, 0x0c, 
	0x38, 0x04, 0x10, 
	0x38, 0x05, 0x8b, 
	0x38, 0x06, 0x0c, 
	0x38, 0x07, 0x43, 
	0x38, 0x08, 0x10, 
	0x38, 0x09, 0x70, 
	0x38, 0x0a, 0x0c, 
	0x38, 0x0b, 0x30, 
	0x38, 0x0c, 0x12, 
	0x38, 0x0d, 0xc0, 
	0x38, 0x0e, 0x0d, 
	0x38, 0x0f, 0x00, 
	0x38, 0x10, 0x00, 
	0x38, 0x11, 0x04, 
	0x38, 0x12, 0x00, 
	0x38, 0x13, 0x04, 
	0x38, 0x14, 0x11, 
	0x38, 0x15, 0x11, 
	0x38, 0x20, 0x00, 
	0x38, 0x21, 0x04, 
	0x38, 0x34, 0x00, 
	0x38, 0x35, 0x1c, 
	0x38, 0x36, 0x04, 
	0x38, 0x37, 0x01, 
	0x40, 0x00, 0xf1, 
	0x40, 0x01, 0x00, 
	0x40, 0x06, 0x04,//31fps--->4fps---check blc
	0x40, 0x07, 0x04,//31fps--->4fps---check blc
	0x40, 0x0b, 0x0c, 
	0x40, 0x11, 0x00, 
	0x40, 0x1a, 0x00, 
	0x40, 0x1b, 0x00, 
	0x40, 0x1c, 0x00, 
	0x40, 0x1d, 0x00, 
	0x40, 0x20, 0x04, 
	0x40, 0x21, 0x90, 
	0x40, 0x22, 0x0b, 
	0x40, 0x23, 0xef, 
	0x40, 0x24, 0x0d, 
	0x40, 0x25, 0xc0, 
	0x40, 0x26, 0x0d, 
	0x40, 0x27, 0xc3, 
	0x40, 0x28, 0x00, 
	0x40, 0x29, 0x02, 
	0x40, 0x2a, 0x04, 
	0x40, 0x2b, 0x08, 
	0x40, 0x2c, 0x02, 
	0x40, 0x2d, 0x02, 
	0x40, 0x2e, 0x0c, 
	0x40, 0x2f, 0x08, 
	0x40, 0x3d, 0x2c, 
	0x40, 0x3f, 0x7f, 
	0x45, 0x00, 0x82, 
	0x45, 0x01, 0x38, 
	0x46, 0x01, 0x04, 
	0x46, 0x02, 0x22, ///12 18 add new for prev capture 
	0x46, 0x03, 0x00, 
	0x48, 0x37, 0x10,
	0x4d, 0x00, 0x04, 
	0x4d, 0x01, 0x42, 
	0x4d, 0x02, 0xd1, 
	0x4d, 0x03, 0x90, 
	0x4d, 0x04, 0x66, 
	0x4d, 0x05, 0x65, 
	0x50, 0x00, 0x0e, 
	0x50, 0x01, 0x01, 
	0x50, 0x02, 0x07, 
	0x50, 0x13, 0x40, 
	0x50, 0x1c, 0x00, 
	0x50, 0x1d, 0x10, 
	0x52, 0x42, 0x00, 
	0x52, 0x43, 0xb8, 
	0x52, 0x44, 0x00, 
	0x52, 0x45, 0xf9, 
	0x52, 0x46, 0x00, 
	0x52, 0x47, 0xf6, 
	0x52, 0x48, 0x00, 
	0x52, 0x49, 0xa6, 
	0x53, 0x00, 0xfc, 
	0x53, 0x01, 0xdf, 
	0x53, 0x02, 0x3f, 
	0x53, 0x03, 0x08, 
	0x53, 0x04, 0x0c, 
	0x53, 0x05, 0x10, 
	0x53, 0x06, 0x20, 
	0x53, 0x07, 0x40, 
	0x53, 0x08, 0x08, 
	0x53, 0x09, 0x08, 
	0x53, 0x0a, 0x02, 
	0x53, 0x0b, 0x01, 
	0x53, 0x0c, 0x01, 
	0x53, 0x0d, 0x0c, 
	0x53, 0x0e, 0x02, 
	0x53, 0x0f, 0x01, 
	0x53, 0x10, 0x01, 
	0x54, 0x00, 0x00, 
	0x54, 0x01, 0x71, 
	0x54, 0x02, 0x00, 
	0x54, 0x03, 0x00, 
	0x54, 0x04, 0x00, 
	0x54, 0x05, 0x80, 
	0x54, 0x0c, 0x05, 
	0x5b, 0x00, 0x00, 
	0x5b, 0x01, 0x00, 
	0x5b, 0x02, 0x01, 
	0x5b, 0x03, 0xff, 
	//0x5b, 0x04, 0xa2, 
	0x5b, 0x04, 0x02,
	0x5b, 0x05, 0x6c, 
	0x5b, 0x09, 0x02, 
	0x5e, 0x00, 0x00, 
	//0x5e, 0x10, 0x0c, 
	0x5e, 0x10, 0x1c,
	0x01, 0x00, 0x01, 

};

void OV13850TRULYPreviewSetting(void)
{
	OV13850TRULYDB("OV13850TRULYPreviewSetting \n");
    /*
	@@ 0 20 RES_2112x1568 30fps_key setting
	;24Mhz Xclk
	;SCLK 60Mhz, Pclk 384MHz
	;4Lane, MIPI datarate 960Mbps/Lane
	;30fps
	;pixels per line=4800 
	;lines per frame=2664

	;100 99 2112 1568
	;102 3601 BBD
	*/
	
	OV13850TRULY_write_cmos_sensor(0x0100, 0x00);
	OV13850TRULY_write_cmos_sensor(0x0300, 0x00);
	OV13850TRULY_write_cmos_sensor(0x0301, 0x00);
	OV13850TRULY_write_cmos_sensor(0x0302, 0x28);
	OV13850TRULY_write_cmos_sensor(0x0303, 0x00);
	OV13850TRULY_write_cmos_sensor(0x3612, 0x23);
	OV13850TRULY_write_cmos_sensor(0x3614, 0x28);
	OV13850TRULY_write_cmos_sensor(0x370a, 0x26);
	OV13850TRULY_write_cmos_sensor(0x372a, 0x00);
	OV13850TRULY_write_cmos_sensor(0x372f, 0xa0);
	OV13850TRULY_write_cmos_sensor(0x3718, 0x1c);
	OV13850TRULY_write_cmos_sensor(0x3767, 0x24);
	OV13850TRULY_write_cmos_sensor(0x3800, 0x00);
	OV13850TRULY_write_cmos_sensor(0x3801, 0x00);
	OV13850TRULY_write_cmos_sensor(0x3802, 0x00);
	OV13850TRULY_write_cmos_sensor(0x3803, 0x04);
	OV13850TRULY_write_cmos_sensor(0x3804, 0x10);
	OV13850TRULY_write_cmos_sensor(0x3805, 0x9f);
	OV13850TRULY_write_cmos_sensor(0x3806, 0x0C);
	OV13850TRULY_write_cmos_sensor(0x3807, 0x4B);
	OV13850TRULY_write_cmos_sensor(0x3808, 0x08);
	OV13850TRULY_write_cmos_sensor(0x3809, 0x40);
	OV13850TRULY_write_cmos_sensor(0x380A, 0x06);
	OV13850TRULY_write_cmos_sensor(0x380B, 0x20);
	OV13850TRULY_write_cmos_sensor(0x380C, 0x12);
	OV13850TRULY_write_cmos_sensor(0x380D, 0xC0);
	OV13850TRULY_write_cmos_sensor(0x380E, 0x0d);
	OV13850TRULY_write_cmos_sensor(0x380F, 0x00);
	OV13850TRULY_write_cmos_sensor(0x3810, 0x00);
	OV13850TRULY_write_cmos_sensor(0x3811, 0x08);
	OV13850TRULY_write_cmos_sensor(0x3812, 0x00);
	OV13850TRULY_write_cmos_sensor(0x3813, 0x02);
	OV13850TRULY_write_cmos_sensor(0x3814, 0x31);
	OV13850TRULY_write_cmos_sensor(0x3815, 0x31);
	OV13850TRULY_write_cmos_sensor(0x3820, 0x01);
	OV13850TRULY_write_cmos_sensor(0x3821, 0x05);
	OV13850TRULY_write_cmos_sensor(0x3836, 0x08);
	OV13850TRULY_write_cmos_sensor(0x3837, 0x02);
	OV13850TRULY_write_cmos_sensor(0x4020, 0x02);
	OV13850TRULY_write_cmos_sensor(0x4021, 0x40);
	OV13850TRULY_write_cmos_sensor(0x4022, 0x03);
	OV13850TRULY_write_cmos_sensor(0x4023, 0x3f);
	OV13850TRULY_write_cmos_sensor(0x4024, 0x06);
	OV13850TRULY_write_cmos_sensor(0x4025, 0xf8);
	OV13850TRULY_write_cmos_sensor(0x4026, 0x07);
	OV13850TRULY_write_cmos_sensor(0x4027, 0xf7);
	OV13850TRULY_write_cmos_sensor(0x4601, 0x04);
	OV13850TRULY_write_cmos_sensor(0x4602, 0x22);
	OV13850TRULY_write_cmos_sensor(0x4603, 0x01);
	OV13850TRULY_write_cmos_sensor(0x4837, 0x11);
	OV13850TRULY_write_cmos_sensor(0x5401, 0x61);
	OV13850TRULY_write_cmos_sensor(0x5405, 0x40);
	OV13850TRULY_write_cmos_sensor(0x350b, 0x20);
	OV13850TRULY_write_cmos_sensor(0x3500, 0x00);
	OV13850TRULY_write_cmos_sensor(0x3501, 0xbb);
	OV13850TRULY_write_cmos_sensor(0x3502, 0x20);
	OV13850TRULY_write_cmos_sensor(0x0100, 0x01);
	mdelay(10);	
	
}


void OV13850TRULYVideoSetting(void)
{
	OV13850TRULYDB("OV13850TRULYVideoSetting \n");
	 /*
	@@ 0 20 RES_2112x1568 30fps_key setting
	;24Mhz Xclk
	;SCLK 60Mhz, Pclk 480MHz
	;4Lane, MIPI datarate 640Mbps/Lane
	;30fps
	;pixels per line=4800
	;lines per frame=3328

	;100 99 2112 1568
	;102 3601 BBD
	*/
   
	OV13850TRULY_write_cmos_sensor(0x0100, 0x00);
	OV13850TRULY_write_cmos_sensor(0x0300, 0x00);
	OV13850TRULY_write_cmos_sensor(0x0301, 0x00);
	OV13850TRULY_write_cmos_sensor(0x0302, 0x28);
	OV13850TRULY_write_cmos_sensor(0x0303, 0x00);
	OV13850TRULY_write_cmos_sensor(0x3612, 0x23);
	OV13850TRULY_write_cmos_sensor(0x3614, 0x28);
	OV13850TRULY_write_cmos_sensor(0x370a, 0x26);
	OV13850TRULY_write_cmos_sensor(0x372a, 0x00);
	OV13850TRULY_write_cmos_sensor(0x372f, 0xa0);
	OV13850TRULY_write_cmos_sensor(0x3718, 0x1c);
	OV13850TRULY_write_cmos_sensor(0x3767, 0x24);
	OV13850TRULY_write_cmos_sensor(0x3800, 0x00);
	OV13850TRULY_write_cmos_sensor(0x3801, 0x00);
	OV13850TRULY_write_cmos_sensor(0x3802, 0x00);
	OV13850TRULY_write_cmos_sensor(0x3803, 0x04);
	OV13850TRULY_write_cmos_sensor(0x3804, 0x10);
	OV13850TRULY_write_cmos_sensor(0x3805, 0x9f);
	OV13850TRULY_write_cmos_sensor(0x3806, 0x0C);
	OV13850TRULY_write_cmos_sensor(0x3807, 0x4B);
	OV13850TRULY_write_cmos_sensor(0x3808, 0x08);
	OV13850TRULY_write_cmos_sensor(0x3809, 0x40);
	OV13850TRULY_write_cmos_sensor(0x380A, 0x06);
	OV13850TRULY_write_cmos_sensor(0x380B, 0x20);
	OV13850TRULY_write_cmos_sensor(0x380C, 0x12);
	OV13850TRULY_write_cmos_sensor(0x380D, 0xC0);
	OV13850TRULY_write_cmos_sensor(0x380E, 0x0d);
	OV13850TRULY_write_cmos_sensor(0x380F, 0x00);
	OV13850TRULY_write_cmos_sensor(0x3810, 0x00);
	OV13850TRULY_write_cmos_sensor(0x3811, 0x08);
	OV13850TRULY_write_cmos_sensor(0x3812, 0x00);
	OV13850TRULY_write_cmos_sensor(0x3813, 0x02);
	OV13850TRULY_write_cmos_sensor(0x3814, 0x31);
	OV13850TRULY_write_cmos_sensor(0x3815, 0x31);
	OV13850TRULY_write_cmos_sensor(0x3820, 0x01);
	OV13850TRULY_write_cmos_sensor(0x3821, 0x05);
	OV13850TRULY_write_cmos_sensor(0x3836, 0x08);
	OV13850TRULY_write_cmos_sensor(0x3837, 0x02);
	OV13850TRULY_write_cmos_sensor(0x4020, 0x02);
	OV13850TRULY_write_cmos_sensor(0x4021, 0x40);
	OV13850TRULY_write_cmos_sensor(0x4022, 0x03);
	OV13850TRULY_write_cmos_sensor(0x4023, 0x3f);
	OV13850TRULY_write_cmos_sensor(0x4024, 0x06);
	OV13850TRULY_write_cmos_sensor(0x4025, 0xf8);
	OV13850TRULY_write_cmos_sensor(0x4026, 0x07);
	OV13850TRULY_write_cmos_sensor(0x4027, 0xf7);
	OV13850TRULY_write_cmos_sensor(0x4601, 0x04);
	OV13850TRULY_write_cmos_sensor(0x4602, 0x22);
	OV13850TRULY_write_cmos_sensor(0x4603, 0x01);
	OV13850TRULY_write_cmos_sensor(0x4837, 0x11);
	OV13850TRULY_write_cmos_sensor(0x5401, 0x61);
	OV13850TRULY_write_cmos_sensor(0x5405, 0x40);
	OV13850TRULY_write_cmos_sensor(0x350b, 0x20);
	OV13850TRULY_write_cmos_sensor(0x3500, 0x00);
	OV13850TRULY_write_cmos_sensor(0x3501, 0xbb);
	OV13850TRULY_write_cmos_sensor(0x3502, 0x20);
	OV13850TRULY_write_cmos_sensor(0x0100, 0x01);
	mdelay(10);

}




void OV13850TRULYCaptureSetting(void)
{
	/*
	@@ RES_4208x3120 24fps-Key setting
	;24Mhz Xclk
	;SCLK 96Mhz, Pclk 384MHz
	;4Lane, MIPI datarate 960Mbps/Lane
	;24fps
	;pixels per line=4800(0x12c0) 
	;lines per frame=3328(0xD00)

	;100 99 4208 3120
	;102 3601 964
	*/
	#if 1
	//for PCLK :384M
	OV13850TRULY_write_cmos_sensor(0x0100, 0x00);  
	OV13850TRULY_write_cmos_sensor(0x0300, 0x00);  
	OV13850TRULY_write_cmos_sensor(0x0301, 0x00);  
	OV13850TRULY_write_cmos_sensor(0x0302, 0x28);  
	OV13850TRULY_write_cmos_sensor(0x0303, 0x00);  
	OV13850TRULY_write_cmos_sensor(0x3612, 0x33);  
	OV13850TRULY_write_cmos_sensor(0x3614, 0x28);  
	OV13850TRULY_write_cmos_sensor(0x370a, 0x24);  
	OV13850TRULY_write_cmos_sensor(0x372a, 0x04);  
	OV13850TRULY_write_cmos_sensor(0x372f, 0xa0);  
	OV13850TRULY_write_cmos_sensor(0x3718, 0x10);  
	OV13850TRULY_write_cmos_sensor(0x3767, 0x24);  
	//;Windowing							  
	OV13850TRULY_write_cmos_sensor(0x3800, 0x00);  
	OV13850TRULY_write_cmos_sensor(0x3801, 0x14);  
	OV13850TRULY_write_cmos_sensor(0x3802, 0x00);  
	OV13850TRULY_write_cmos_sensor(0x3803, 0x0C);  
	OV13850TRULY_write_cmos_sensor(0x3804, 0x10);  
	OV13850TRULY_write_cmos_sensor(0x3805, 0x8B);  
	OV13850TRULY_write_cmos_sensor(0x3806, 0x0C);  
	OV13850TRULY_write_cmos_sensor(0x3807, 0x43);  
	OV13850TRULY_write_cmos_sensor(0x3808, 0x10);  
	OV13850TRULY_write_cmos_sensor(0x3809, 0x70);  
	OV13850TRULY_write_cmos_sensor(0x380A, 0x0C);  
	OV13850TRULY_write_cmos_sensor(0x380B, 0x30);  
	OV13850TRULY_write_cmos_sensor(0x380C, 0x12);  
	OV13850TRULY_write_cmos_sensor(0x380D, 0xC0);  
	OV13850TRULY_write_cmos_sensor(0x380E, 0x0d);  
	OV13850TRULY_write_cmos_sensor(0x380F, 0x00);  
	OV13850TRULY_write_cmos_sensor(0x3810, 0x00);  
	OV13850TRULY_write_cmos_sensor(0x3811, 0x04);  
	OV13850TRULY_write_cmos_sensor(0x3812, 0x00);  
	OV13850TRULY_write_cmos_sensor(0x3813, 0x04);  
	OV13850TRULY_write_cmos_sensor(0x3814, 0x11);  
	OV13850TRULY_write_cmos_sensor(0x3815, 0x11);  
	OV13850TRULY_write_cmos_sensor(0x3820, 0x00);  
	OV13850TRULY_write_cmos_sensor(0x3821, 0x04);  
	OV13850TRULY_write_cmos_sensor(0x3836, 0x04);  
	OV13850TRULY_write_cmos_sensor(0x3837, 0x01);  
	OV13850TRULY_write_cmos_sensor(0x4020, 0x2 );  
	OV13850TRULY_write_cmos_sensor(0x4021, 0x3C);  
	OV13850TRULY_write_cmos_sensor(0x4022, 0xE );  
	OV13850TRULY_write_cmos_sensor(0x4023, 0x37);  
	OV13850TRULY_write_cmos_sensor(0x4024, 0xF );  
	OV13850TRULY_write_cmos_sensor(0x4025, 0x1C);  
	OV13850TRULY_write_cmos_sensor(0x4026, 0xF );  
	OV13850TRULY_write_cmos_sensor(0x4027, 0x1F);  
	OV13850TRULY_write_cmos_sensor(0x4601, 0x00);  
	OV13850TRULY_write_cmos_sensor(0x4603, 0x00);  
	OV13850TRULY_write_cmos_sensor(0x4837, 0x11);  
	OV13850TRULY_write_cmos_sensor(0x5401, 0x71);  
	OV13850TRULY_write_cmos_sensor(0x5405, 0x80);  
	OV13850TRULY_write_cmos_sensor(0x350b, 0x80);  
	OV13850TRULY_write_cmos_sensor(0x3500, 0x00);  
	OV13850TRULY_write_cmos_sensor(0x3501, 0xCF);  
	OV13850TRULY_write_cmos_sensor(0x3502, 0x80);  
	OV13850TRULY_write_cmos_sensor(0x0100, 0x01);  
    #else
	////for PCLK :240M
	//OV13850TRULY_write_cmos_sensor(0x0100, 0x00);
	OV13850TRULY_write_cmos_sensor(0x3208 , 0xa2);
	//OV13850TRULY_write_cmos_sensor(0x0100, 0x01);
	#endif

}



static void OV13850TRULY_Sensor_Init(void)
{


    int totalCnt = 0, len = 0;
	int transfer_len, transac_len=3;
	kal_uint8* pBuf=NULL;
	dma_addr_t dmaHandle;
	pBuf = (kal_uint8*)kmalloc(1024, GFP_KERNEL);
	

    totalCnt = ARRAY_SIZE(ov13850truly_init);
	transfer_len = totalCnt / transac_len;
	len = (transfer_len<<8)|transac_len;    
	OV13850TRULYDB("Total Count = %d, Len = 0x%x\n", totalCnt,len);    
	memcpy(pBuf, &ov13850truly_init, totalCnt );   
	dmaHandle = dma_map_single(NULL, pBuf, 1024, DMA_TO_DEVICE);	
	OV13850TRULY_multi_write_cmos_sensor(dmaHandle, len); 

	dma_unmap_single(NULL, dmaHandle, 1024, DMA_TO_DEVICE);
	kfree(pBuf);	

	
	#ifdef SHUTTER_MWRITE
	pSBuf = (kal_uint8*)kmalloc(1024, GFP_KERNEL);
	sdmaHandle = dma_map_single(NULL, pSBuf, 1024, DMA_TO_DEVICE);	
	
	#endif
	
}   /*  OV13850TRULY_Sensor_Init  */

UINT32 OV13850TRULYOpen(void)
{
	volatile signed int i;
	int  retry = 1;
	kal_uint16 sensor_id = 0;
	OV13850TRULYDB("OV13850TRULYOpen enter :\n ");
	OV13850TRULY_WRITE_ID = OV13850TRULYMIPI_WRITE_ID_1;
	OV13850TRULY_write_cmos_sensor(0x0103,0x01);// Reset sensor
    mDELAY(10);

    // check if sensor ID correct
    do {
        sensor_id = (OV13850TRULY_read_cmos_sensor(0x300A)<<8)|OV13850TRULY_read_cmos_sensor(0x300B);
        if (sensor_id == OV13850_SENSOR_ID)
        	{
        		OV13850TRULYDB("write id=%x, Sensor ID = 0x%04x\n", OV13850TRULY_WRITE_ID,sensor_id);
            	break;
        	}
        OV13850TRULYDB("Read Sensor ID Fail = 0x%04x\n", sensor_id);
        retry--;
    } while (retry > 0);

    if (sensor_id != OV13850_SENSOR_ID) {
		OV13850TRULY_WRITE_ID=OV13850TRULYMIPI_WRITE_ID;
		OV13850TRULY_write_cmos_sensor(0x0103,0x01);// Reset sensor
	    mDELAY(10);
        retry = 1;
	    // check if sensor ID correct
	    do {
	        sensor_id = (OV13850TRULY_read_cmos_sensor(0x300A)<<8)|OV13850TRULY_read_cmos_sensor(0x300B);
	        if (sensor_id == OV13850_SENSOR_ID)
	        	{
	        		OV13850TRULYDB("write id=%x,Sensor ID = 0x%04x\n",OV13850TRULY_WRITE_ID, sensor_id);
	            	break;
	        	}
	        OV13850TRULYDB("Read Sensor ID Fail = 0x%04x\n", sensor_id);
	        retry--;
	    } while (retry > 0);
		 if (sensor_id != OV13850_SENSOR_ID) 
		 {
           return ERROR_SENSOR_CONNECT_FAIL;
		 	}
    }
	spin_lock(&ov13850trulymipiraw_drv_lock);
	ov13850truly.sensorMode = SENSOR_MODE_INIT;
	ov13850truly.OV13850TRULYAutoFlickerMode = KAL_FALSE;
	ov13850truly.OV13850TRULYVideoMode = KAL_FALSE;
	spin_unlock(&ov13850trulymipiraw_drv_lock);
	OV13850TRULY_Sensor_Init();
  //mDELAY(10);
  /************OTP function************/
        update_otp_wb();
        update_otp_lenc();
  /***********************************/
	spin_lock(&ov13850trulymipiraw_drv_lock);
	ov13850truly.DummyLines= 0;
	ov13850truly.DummyPixels= 0;
	ov13850truly.shutter = 0x4EA;
	ov13850truly.pvShutter = 0x4EA;
	ov13850truly.maxExposureLines =OV13850TRULY_PV_PERIOD_LINE_NUMS -4;
	ov13850truly.ispBaseGain = BASEGAIN;//0x40
	ov13850truly.sensorGlobalGain = 0x1f;//sensor gain read from 0x350a 0x350b; 0x1f as 3.875x
	ov13850truly.pvGain = 0x1f;
	ov13850truly.realGain = OV13850TRULYReg2Gain(0x1f);//ispBaseGain as 1x
	spin_unlock(&ov13850trulymipiraw_drv_lock);

	OV13850TRULYDB("OV13850TRULYOpen exit :\n ");

    return ERROR_NONE;
}

UINT32 OV13850TRULYGetSensorID(UINT32 *sensorID)
{
    int  retry = 1;

	OV13850TRULYDB("OV13850TRULYGetSensorID enter :\n ");
	OV13850TRULY_WRITE_ID = OV13850TRULYMIPI_WRITE_ID;
	OV13850TRULY_write_cmos_sensor(0x0103,0x01);// Reset sensor
    mDELAY(10);

    // check if sensor ID correct
    do {
        *sensorID = (OV13850TRULY_read_cmos_sensor(0x300A)<<8)|OV13850TRULY_read_cmos_sensor(0x300B);
        if (*sensorID == OV13850_SENSOR_ID)
        	{
        		OV13850TRULYDB("write id=%x, Sensor ID = 0x%04x\n", OV13850TRULY_WRITE_ID,*sensorID);
            	break;
        	}
        OV13850TRULYDB("Read Sensor ID Fail = 0x%04x\n", *sensorID);
        retry--;
    } while (retry > 0);

    if (*sensorID != OV13850_SENSOR_ID) {
		OV13850TRULY_WRITE_ID=OV13850TRULYMIPI_WRITE_ID_1;
		OV13850TRULY_write_cmos_sensor(0x0103,0x01);// Reset sensor
	    mDELAY(10);
        retry = 1;
	    // check if sensor ID correct
	    do {
	        *sensorID = (OV13850TRULY_read_cmos_sensor(0x300A)<<8)|OV13850TRULY_read_cmos_sensor(0x300B);
	        if (*sensorID == OV13850_SENSOR_ID)
	        	{
	        		OV13850TRULYDB("write id=%x,Sensor ID = 0x%04x\n",OV13850TRULY_WRITE_ID, *sensorID);
	            	break;
	        	}
	        OV13850TRULYDB("Read Sensor ID Fail = 0x%04x\n", *sensorID);
	        retry--;
	    } while (retry > 0);
		 if (*sensorID != OV13850_SENSOR_ID) 
		 {
		 
        *sensorID = 0xFFFFFFFF;
        return ERROR_SENSOR_CONNECT_FAIL;
		 	}
    }
    OV13850TRULY_write_cmos_sensor(0x0100, 0x01);
    mDELAY(50);
    read_otp_info(1,&s_otp_module);
    //printk("the module info modules id is %x ,lens_id is %x,production_date is: %d year %d month %d day \n",s_otp_module.module_integrator_id,s_otp_module.lens_id,s_otp_module.production_year,s_otp_module.production_month,s_otp_module.production_day);

    if(s_otp_module.module_integrator_id==0x02)
    {
        *sensorID = OV13850_TRULY_SENSOR_ID;
        return ERROR_NONE;
    }
    else
    {
        *sensorID = 0xFFFFFFFF;
        return ERROR_SENSOR_CONNECT_FAIL;
    }
}


void OV13850TRULY_SetShutter(kal_uint32 iShutter)
{
	if(MSDK_SCENARIO_ID_CAMERA_ZSD == OV13850TRULYCurrentScenarioId )
	{
		//OV13850TRULYDB("always UPDATE SHUTTER when ov13850truly.sensorMode == SENSOR_MODE_CAPTURE\n");
	}
	else{
		if(ov13850truly.sensorMode == SENSOR_MODE_CAPTURE)
		{
			//OV13850TRULYDB("capture!!DONT UPDATE SHUTTER!!\n");
			//return;
		}
	}
	//if(ov13850truly.shutter == iShutter)
		//return;
   spin_lock(&ov13850trulymipiraw_drv_lock);
   ov13850truly.shutter= iShutter;
   spin_unlock(&ov13850trulymipiraw_drv_lock);
   OV13850TRULY_write_shutter(iShutter);
   return;
}   /*  OV13850TRULY_SetShutter   */

UINT32 OV13850TRULY_read_shutter(void)
{

	kal_uint16 temp_reg1, temp_reg2 ,temp_reg3;
	UINT32 shutter =0;
	temp_reg1 = OV13850TRULY_read_cmos_sensor(0x3500);    // AEC[b19~b16]
	temp_reg2 = OV13850TRULY_read_cmos_sensor(0x3501);    // AEC[b15~b8]
	temp_reg3 = OV13850TRULY_read_cmos_sensor(0x3502);    // AEC[b7~b0]
	shutter  = (temp_reg1 <<12)| (temp_reg2<<4)|(temp_reg3>>4);

	return shutter;
}

void OV13850TRULY_NightMode(kal_bool bEnable)
{}


UINT32 OV13850TRULYClose(void)
{    
#ifdef SHUTTER_MWRITE
	dma_unmap_single(NULL, sdmaHandle, 1024, DMA_TO_DEVICE);
	kfree(pSBuf);	
#endif

return ERROR_NONE;}

void OV13850TRULYSetFlipMirror(kal_int32 imgMirror)
{
	kal_int16 mirror=0,flip=0;
	if(0 == strncmp(VANZO_MAIN_CAM_ROTATION, "180", 3))
		imgMirror= IMAGE_V_MIRROR;
    flip = OV13850TRULY_read_cmos_sensor(0x3820);
	mirror   = OV13850TRULY_read_cmos_sensor(0x3821);
    switch (imgMirror)
    {
        case IMAGE_NORMAL://IMAGE_NORMAL:
            OV13850TRULY_write_cmos_sensor(0x3820, (flip & (0xFB)));//Set normal 0xBD--->0xbc  for capture size
            OV13850TRULY_write_cmos_sensor(0x3821, (mirror  & (0xFB)));	//Set normal  0xf9-->0xf8  for capture size
            break;
        case IMAGE_H_MIRROR://IMAGE_H_MIRROR:
            OV13850TRULY_write_cmos_sensor(0x3820, (flip & (0xFB)));//Set normal  0xbd--->0xbc  for capture size
            OV13850TRULY_write_cmos_sensor(0x3821, (mirror  | (0x04)));	//Set mirror
            break;
        case IMAGE_V_MIRROR://IMAGE_V_MIRROR:
            OV13850TRULY_write_cmos_sensor(0x3820, (flip |(0x04)));	//Set flip
            OV13850TRULY_write_cmos_sensor(0x3821, (mirror  & (0xFB)));	//Set normal //0xf9-->0xf8  for capture size
            break;
        case IMAGE_HV_MIRROR://IMAGE_HV_MIRROR:
            OV13850TRULY_write_cmos_sensor(0x3820, (flip |(0x4)));	//Set flip
            OV13850TRULY_write_cmos_sensor(0x3821, (mirror  |(0x04)));	//Set mirror
            break;
    }
}

UINT32 OV13850TRULYPreview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                                                MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{

	OV13850TRULYDB("OV13850TRULYPreview enter:");

	// preview size
	if(ov13850truly.sensorMode == SENSOR_MODE_PREVIEW)
	{
		// do nothing
		// FOR CCT PREVIEW
	}
	else
	{
		OV13850TRULYDB("OV13850TRULYPreview setting!!\n");
		OV13850TRULYPreviewSetting();
		//mdelay(30);
	}
	spin_lock(&ov13850trulymipiraw_drv_lock);
	ov13850truly.sensorMode = SENSOR_MODE_PREVIEW; // Need set preview setting after capture mode
	ov13850truly.DummyPixels = 0;//define dummy pixels and lines
	ov13850truly.DummyLines = 0 ;
	OV13850TRULY_FeatureControl_PERIOD_PixelNum=OV13850TRULY_PV_PERIOD_PIXEL_NUMS+ ov13850truly.DummyPixels;
	OV13850TRULY_FeatureControl_PERIOD_LineNum=OV13850TRULY_PV_PERIOD_LINE_NUMS+ov13850truly.DummyLines;
	spin_unlock(&ov13850trulymipiraw_drv_lock);

	//OV13850TRULY_write_shutter(ov13850truly.shutter);
	//write_OV13850TRULY_gain(ov13850truly.pvGain);
	//set mirror & flip
	//OV13850TRULYDB("[OV13850TRULYPreview] mirror&flip: %d \n",sensor_config_data->SensorImageMirror);
	spin_lock(&ov13850trulymipiraw_drv_lock);
	ov13850truly.imgMirror = sensor_config_data->SensorImageMirror;
	spin_unlock(&ov13850trulymipiraw_drv_lock);
	OV13850TRULYSetFlipMirror(OV13850TRULY_ORIENTATION);
	OV13850TRULYDB("OV13850TRULYPreview exit: \n");
    return ERROR_NONE;
}	/* OV13850TRULYPreview() */


UINT32 OV13850TRULYVideo(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                                                MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	OV13850TRULYDB("OV13850TRULYVideo enter:");
	if(ov13850truly.sensorMode == SENSOR_MODE_VIDEO)
	{
		// do nothing
	}
	else
		OV13850TRULYVideoSetting();
	
	spin_lock(&ov13850trulymipiraw_drv_lock);
	ov13850truly.sensorMode = SENSOR_MODE_VIDEO;
	OV13850TRULY_FeatureControl_PERIOD_PixelNum=OV13850TRULY_VIDEO_PERIOD_PIXEL_NUMS+ ov13850truly.DummyPixels;
	OV13850TRULY_FeatureControl_PERIOD_LineNum=OV13850TRULY_VIDEO_PERIOD_LINE_NUMS+ov13850truly.DummyLines;
	spin_unlock(&ov13850trulymipiraw_drv_lock);

	//OV13850TRULY_write_shutter(ov13850truly.shutter);
	//write_OV13850TRULY_gain(ov13850truly.pvGain);

	spin_lock(&ov13850trulymipiraw_drv_lock);
	ov13850truly.imgMirror = sensor_config_data->SensorImageMirror;
	spin_unlock(&ov13850trulymipiraw_drv_lock);
	OV13850TRULYSetFlipMirror(OV13850TRULY_ORIENTATION);

	OV13850TRULYDBSOFIA("[OV13850TRULYVideo]frame_len=%x\n", ((OV13850TRULY_read_cmos_sensor(0x380e)<<8)+OV13850TRULY_read_cmos_sensor(0x380f)));

	OV13850TRULYDB("OV13850TRULYVideo exit:\n");
    return ERROR_NONE;
}


UINT32 OV13850TRULYCapture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                                                MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
 	kal_uint32 shutter = ov13850truly.shutter;
	kal_uint32 temp_data;
	if( SENSOR_MODE_CAPTURE== ov13850truly.sensorMode)
	{
		OV13850TRULYDB("OV13850TRULYCapture BusrtShot!!!\n");
	}else{
		OV13850TRULYDB("OV13850TRULYCapture enter:\n");
		//Record Preview shutter & gain
		shutter=OV13850TRULY_read_shutter();
		temp_data =  read_OV13850TRULY_gain();
		spin_lock(&ov13850trulymipiraw_drv_lock);
		ov13850truly.pvShutter =shutter;
		ov13850truly.sensorGlobalGain = temp_data;
		ov13850truly.pvGain =ov13850truly.sensorGlobalGain;
		spin_unlock(&ov13850trulymipiraw_drv_lock);

		OV13850TRULYDB("[OV13850TRULYCapture]ov13850truly.shutter=%d, read_pv_shutter=%d, read_pv_gain = 0x%x\n",ov13850truly.shutter, shutter,ov13850truly.sensorGlobalGain);

		// Full size setting
		OV13850TRULYCaptureSetting();
		spin_lock(&ov13850trulymipiraw_drv_lock);
		ov13850truly.sensorMode = SENSOR_MODE_CAPTURE;
		ov13850truly.imgMirror = sensor_config_data->SensorImageMirror;
		ov13850truly.DummyPixels = 0;//define dummy pixels and lines                                                                                                         
		ov13850truly.DummyLines = 0 ;    
		OV13850TRULY_FeatureControl_PERIOD_PixelNum = OV13850TRULY_FULL_PERIOD_PIXEL_NUMS + ov13850truly.DummyPixels;
		OV13850TRULY_FeatureControl_PERIOD_LineNum = OV13850TRULY_FULL_PERIOD_LINE_NUMS + ov13850truly.DummyLines;
		spin_unlock(&ov13850trulymipiraw_drv_lock);

		//OV13850TRULYDB("[OV13850TRULYCapture] mirror&flip: %d\n",sensor_config_data->SensorImageMirror);
		OV13850TRULYSetFlipMirror(OV13850TRULY_ORIENTATION);

	    if(OV13850TRULYCurrentScenarioId==MSDK_SCENARIO_ID_CAMERA_ZSD)
	    {
			OV13850TRULYDB("OV13850TRULYCapture exit ZSD!!\n");
			return ERROR_NONE;
	    }
		OV13850TRULYDB("OV13850TRULYCapture exit:\n");
	}

    return ERROR_NONE;
}	/* OV13850TRULYCapture() */

UINT32 OV13850TRULYGetResolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *pSensorResolution)
{

    OV13850TRULYDB("OV13850TRULYGetResolution!!\n");
	pSensorResolution->SensorPreviewWidth	= OV13850TRULY_IMAGE_SENSOR_PV_WIDTH;
    pSensorResolution->SensorPreviewHeight	= OV13850TRULY_IMAGE_SENSOR_PV_HEIGHT;
    pSensorResolution->SensorFullWidth		= OV13850TRULY_IMAGE_SENSOR_FULL_WIDTH;
    pSensorResolution->SensorFullHeight		= OV13850TRULY_IMAGE_SENSOR_FULL_HEIGHT;
    pSensorResolution->SensorVideoWidth		= OV13850TRULY_IMAGE_SENSOR_VIDEO_WIDTH;
    pSensorResolution->SensorVideoHeight    = OV13850TRULY_IMAGE_SENSOR_VIDEO_HEIGHT;
    return ERROR_NONE;
}   /* OV13850TRULYGetResolution() */

UINT32 OV13850TRULYGetInfo(MSDK_SCENARIO_ID_ENUM ScenarioId,
                                                MSDK_SENSOR_INFO_STRUCT *pSensorInfo,
                                                MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{

	pSensorInfo->SensorPreviewResolutionX= OV13850TRULY_IMAGE_SENSOR_PV_WIDTH;
	pSensorInfo->SensorPreviewResolutionY= OV13850TRULY_IMAGE_SENSOR_PV_HEIGHT;
	pSensorInfo->SensorFullResolutionX= OV13850TRULY_IMAGE_SENSOR_FULL_WIDTH;
    pSensorInfo->SensorFullResolutionY= OV13850TRULY_IMAGE_SENSOR_FULL_HEIGHT;

	spin_lock(&ov13850trulymipiraw_drv_lock);
	ov13850truly.imgMirror = pSensorConfigData->SensorImageMirror ;
	spin_unlock(&ov13850trulymipiraw_drv_lock);

   	pSensorInfo->SensorOutputDataFormat= SENSOR_OUTPUT_FORMAT_RAW_B;
    pSensorInfo->SensorClockPolarity =SENSOR_CLOCK_POLARITY_LOW;
    pSensorInfo->SensorClockFallingPolarity=SENSOR_CLOCK_POLARITY_LOW;
    pSensorInfo->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
    pSensorInfo->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;

    pSensorInfo->SensroInterfaceType=SENSOR_INTERFACE_TYPE_MIPI;
    //pSensorInfo->MIPIsensorType = MIPI_OPHY_CSI2;
    pSensorInfo->CaptureDelayFrame = 3;
    pSensorInfo->PreviewDelayFrame = 2;//2;
    pSensorInfo->VideoDelayFrame = 2;

    pSensorInfo->SensorDrivingCurrent = ISP_DRIVING_8MA;
    pSensorInfo->AEShutDelayFrame = 0;//0;		    /* The frame of setting shutter default 0 for TG int */
    pSensorInfo->AESensorGainDelayFrame = 0 ;//0;     /* The frame of setting sensor gain */
    pSensorInfo->AEISPGainDelayFrame = 2;

    switch (ScenarioId)
    {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
            pSensorInfo->SensorClockFreq=24;
            pSensorInfo->SensorClockRisingCount= 0;

            pSensorInfo->SensorGrabStartX = OV13850TRULY_PV_X_START;
            pSensorInfo->SensorGrabStartY = OV13850TRULY_PV_Y_START;

            pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_4_LANE;
            pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
	     	pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 23;
	    	pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
            pSensorInfo->SensorPacketECCOrder = 1;
            break;
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
            pSensorInfo->SensorClockFreq=24;
            pSensorInfo->SensorClockRisingCount= 0;

            pSensorInfo->SensorGrabStartX = OV13850TRULY_VIDEO_X_START;
            pSensorInfo->SensorGrabStartY = OV13850TRULY_VIDEO_Y_START;

            pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_4_LANE;
            pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
	     	pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 23;
	    	pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
            pSensorInfo->SensorPacketECCOrder = 1;
            break;
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
            pSensorInfo->SensorClockFreq=24;
            pSensorInfo->SensorClockRisingCount= 0;

            pSensorInfo->SensorGrabStartX = OV13850TRULY_FULL_X_START;	//2*OV13850TRULY_IMAGE_SENSOR_PV_STARTX;
            pSensorInfo->SensorGrabStartY = OV13850TRULY_FULL_Y_START;	//2*OV13850TRULY_IMAGE_SENSOR_PV_STARTY;

            pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_4_LANE;
            pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
            pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 23;
            pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
            pSensorInfo->SensorPacketECCOrder = 1;
            break;
        default:
			pSensorInfo->SensorClockFreq=24;
            pSensorInfo->SensorClockRisingCount= 0;

            pSensorInfo->SensorGrabStartX = OV13850TRULY_PV_X_START;
            pSensorInfo->SensorGrabStartY = OV13850TRULY_PV_Y_START;

            pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_4_LANE;
            pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
	     	pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 23;
	    	pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
            pSensorInfo->SensorPacketECCOrder = 1;
            break;
    }

    memcpy(pSensorConfigData, &OV13850TRULYSensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));

    return ERROR_NONE;
}   /* OV13850TRULYGetInfo() */


UINT32 OV13850TRULYControl(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *pImageWindow,
                                                MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
		spin_lock(&ov13850trulymipiraw_drv_lock);
		OV13850TRULYCurrentScenarioId = ScenarioId;
		spin_unlock(&ov13850trulymipiraw_drv_lock);
		OV13850TRULYDB("OV13850TRULYCurrentScenarioId=%d\n",OV13850TRULYCurrentScenarioId);
    switch (ScenarioId)
    {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
            OV13850TRULYPreview(pImageWindow, pSensorConfigData);
            break;
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			OV13850TRULYVideo(pImageWindow, pSensorConfigData);
			break;
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
            OV13850TRULYCapture(pImageWindow, pSensorConfigData);
            break;
        default:
            return ERROR_INVALID_SCENARIO_ID;
    }
    return ERROR_NONE;
} /* OV13850TRULYControl() */


UINT32 OV13850TRULYSetVideoMode(UINT16 u2FrameRate)
{

    kal_uint32 MIN_Frame_length =0,frameRate=0,extralines=0;
    OV13850TRULYDB("[OV13850TRULYSetVideoMode] frame rate = %d\n", u2FrameRate);

	spin_lock(&ov13850trulymipiraw_drv_lock);
	 ov13850trulyVIDEO_MODE_TARGET_FPS=u2FrameRate;
	spin_unlock(&ov13850trulymipiraw_drv_lock);

	if(u2FrameRate==0)
	{
		OV13850TRULYDB("Disable Video Mode or dynimac fps\n");
		return KAL_TRUE;
	}
	if(u2FrameRate >30 || u2FrameRate <5)
	    OV13850TRULYDB("error frame rate seting\n");

    if(ov13850truly.sensorMode == SENSOR_MODE_VIDEO)//video ScenarioId recording
    {
    	if(ov13850truly.OV13850TRULYAutoFlickerMode == KAL_TRUE)
    	{
    		if (u2FrameRate==30)
				frameRate= 306;
			else if(u2FrameRate==15)
				frameRate= 148;
			else
				frameRate=u2FrameRate*10;

			MIN_Frame_length = (OV13850TRULYMIPI_VIDEO_CLK)/(OV13850TRULY_VIDEO_PERIOD_PIXEL_NUMS + ov13850truly.DummyPixels)/frameRate*10;
    	}
		else
			MIN_Frame_length = (OV13850TRULYMIPI_VIDEO_CLK) /(OV13850TRULY_VIDEO_PERIOD_PIXEL_NUMS + ov13850truly.DummyPixels)/u2FrameRate;

		if((MIN_Frame_length <=OV13850TRULY_VIDEO_PERIOD_LINE_NUMS))
		{
			MIN_Frame_length = OV13850TRULY_VIDEO_PERIOD_LINE_NUMS;
			OV13850TRULYDB("[OV13850TRULYSetVideoMode]current fps = %d\n", (OV13850TRULYMIPI_PREVIEW_CLK)  /(OV13850TRULY_PV_PERIOD_PIXEL_NUMS)/OV13850TRULY_PV_PERIOD_LINE_NUMS);
		}
		//OV13850TRULYDB("[OV13850TRULYSetVideoMode]current fps (10 base)= %d\n", (OV13850TRULYMIPI_PREVIEW_CLK)*10/(OV13850TRULY_PV_PERIOD_PIXEL_NUMS + ov13850truly.DummyPixels)/MIN_Frame_length);
		if(ov13850truly.shutter+4 > MIN_Frame_length) 
			MIN_Frame_length = ov13850truly.shutter + 4;
		extralines = MIN_Frame_length - OV13850TRULY_VIDEO_PERIOD_LINE_NUMS;

		spin_lock(&ov13850trulymipiraw_drv_lock);
		ov13850truly.DummyPixels = 0;//define dummy pixels and lines
		ov13850truly.DummyLines = extralines ;
		spin_unlock(&ov13850trulymipiraw_drv_lock);
		
		OV13850TRULY_SetDummy(ov13850truly.DummyPixels,extralines);
    }
	else if(ov13850truly.sensorMode == SENSOR_MODE_CAPTURE)
	{
		OV13850TRULYDB("-------[OV13850TRULYSetVideoMode]ZSD???---------\n");
		if(ov13850truly.OV13850TRULYAutoFlickerMode == KAL_TRUE)
    	{
    		if (u2FrameRate==15)
			    frameRate= 148;
			else
				frameRate=u2FrameRate*10;

			MIN_Frame_length = (OV13850TRULYMIPI_CAPTURE_CLK) /(OV13850TRULY_FULL_PERIOD_PIXEL_NUMS + ov13850truly.DummyPixels)/frameRate*10;
    	}
		else
			MIN_Frame_length = (OV13850TRULYMIPI_CAPTURE_CLK) /(OV13850TRULY_FULL_PERIOD_PIXEL_NUMS + ov13850truly.DummyPixels)/u2FrameRate;

		if((MIN_Frame_length <=OV13850TRULY_FULL_PERIOD_LINE_NUMS))
		{
			MIN_Frame_length = OV13850TRULY_FULL_PERIOD_LINE_NUMS;
			//OV13850TRULYDB("[OV13850TRULYSetVideoMode]current fps = %d\n", (OV13850TRULYMIPI_CAPTURE_CLK) /(OV13850TRULY_FULL_PERIOD_PIXEL_NUMS)/OV13850TRULY_FULL_PERIOD_LINE_NUMS);

		}
		//OV13850TRULYDB("[OV13850TRULYSetVideoMode]current fps (10 base)= %d\n", (OV13850TRULYMIPI_CAPTURE_CLK)*10/(OV13850TRULY_FULL_PERIOD_PIXEL_NUMS + ov13850truly.DummyPixels)/MIN_Frame_length);
		if(ov13850truly.shutter+4 > MIN_Frame_length) 
			MIN_Frame_length = ov13850truly.shutter + 4;
		extralines = MIN_Frame_length - OV13850TRULY_FULL_PERIOD_LINE_NUMS;

		spin_lock(&ov13850trulymipiraw_drv_lock);
		ov13850truly.DummyPixels = 0;//define dummy pixels and lines
		ov13850truly.DummyLines = extralines ;
		spin_unlock(&ov13850trulymipiraw_drv_lock);

		OV13850TRULY_SetDummy(ov13850truly.DummyPixels,extralines);
	}
	OV13850TRULYDB("[OV13850TRULYSetVideoMode]MIN_Frame_length=%d,ov13850truly.DummyLines=%d\n",MIN_Frame_length,ov13850truly.DummyLines);

    return KAL_TRUE;
}

UINT32 OV13850TRULYSetAutoFlickerMode(kal_bool bEnable, UINT16 u2FrameRate)
{
	//return ERROR_NONE;
    //OV13850TRULYDB("[OV13850TRULYSetAutoFlickerMode] frame rate(10base) = %d %d\n", bEnable, u2FrameRate);
	if(bEnable) {   // enable auto flicker
		spin_lock(&ov13850trulymipiraw_drv_lock);
		ov13850truly.OV13850TRULYAutoFlickerMode = KAL_TRUE;
		spin_unlock(&ov13850trulymipiraw_drv_lock);
    } else {
    	spin_lock(&ov13850trulymipiraw_drv_lock);
        ov13850truly.OV13850TRULYAutoFlickerMode = KAL_FALSE;
		spin_unlock(&ov13850trulymipiraw_drv_lock);
        OV13850TRULYDB("Disable Auto flicker\n");
    }

    return ERROR_NONE;
}

UINT32 OV13850TRULYSetTestPatternMode(kal_bool bEnable)
{
    OV13850TRULYDB("[OV13850TRULYSetTestPatternMode] Test pattern enable:%d\n", bEnable);
	if(bEnable)
		{
		   OV13850TRULY_write_cmos_sensor(0x5E00, 0x80);
		}
		else
		{
		
			OV13850TRULY_write_cmos_sensor(0x5E00, 0x00);
		}

    return ERROR_NONE;
}


UINT32 OV13850TRULYMIPISetMaxFramerateByScenario(MSDK_SCENARIO_ID_ENUM scenarioId, MUINT32 frameRate) {
	kal_uint32 pclk;
	kal_int16 dummyLine;
	kal_uint16 lineLength,frameHeight;
		
	OV13850TRULYDB("OV13850TRULYMIPISetMaxFramerateByScenario: scenarioId = %d, frame rate = %d\n",scenarioId,frameRate);
	switch (scenarioId) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			pclk = OV13850TRULYMIPI_PREVIEW_CLK;
			lineLength = OV13850TRULY_PV_PERIOD_PIXEL_NUMS;
			frameHeight = (10 * pclk)/frameRate/lineLength;
			dummyLine = frameHeight - OV13850TRULY_PV_PERIOD_LINE_NUMS;
			ov13850truly.sensorMode = SENSOR_MODE_PREVIEW;
			OV13850TRULY_SetDummy(0, dummyLine);			
			break;			
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			pclk = OV13850TRULYMIPI_VIDEO_CLK; 
			lineLength = OV13850TRULY_VIDEO_PERIOD_PIXEL_NUMS;
			frameHeight = (10 * pclk)/frameRate/lineLength;
			dummyLine = frameHeight - OV13850TRULY_VIDEO_PERIOD_LINE_NUMS;
			ov13850truly.sensorMode = SENSOR_MODE_VIDEO;
			OV13850TRULY_SetDummy(0, dummyLine);			
			break;			
			 break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_CAMERA_ZSD:			
			pclk = OV13850TRULYMIPI_CAPTURE_CLK;
			lineLength = OV13850TRULY_FULL_PERIOD_PIXEL_NUMS;
			frameHeight = (10 * pclk)/frameRate/lineLength;
			dummyLine = frameHeight - OV13850TRULY_FULL_PERIOD_LINE_NUMS;
			ov13850truly.sensorMode = SENSOR_MODE_CAPTURE;
			OV13850TRULY_SetDummy(0, dummyLine);			
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


UINT32 OV13850TRULYMIPIGetDefaultFramerateByScenario(MSDK_SCENARIO_ID_ENUM scenarioId, MUINT32 *pframeRate) 
{

	switch (scenarioId) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			 *pframeRate = 300;
			 break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
			 *pframeRate = 240;
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


UINT32 OV13850TRULYFeatureControl(MSDK_SENSOR_FEATURE_ENUM FeatureId,
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
	OV13850TRULYDB(" OV13850TRULYFeatureControl is %d \n", FeatureId);

    switch (FeatureId)
    {
        case SENSOR_FEATURE_GET_RESOLUTION:
            *pFeatureReturnPara16++= OV13850TRULY_IMAGE_SENSOR_FULL_WIDTH;
            *pFeatureReturnPara16= OV13850TRULY_IMAGE_SENSOR_FULL_HEIGHT;
            *pFeatureParaLen=4;
            break;
        case SENSOR_FEATURE_GET_PERIOD:
				*pFeatureReturnPara16++= OV13850TRULY_FeatureControl_PERIOD_PixelNum;
				*pFeatureReturnPara16= OV13850TRULY_FeatureControl_PERIOD_LineNum;
				*pFeatureParaLen=4;
				break;
        case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
			switch(OV13850TRULYCurrentScenarioId)
			{
				case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
					*pFeatureReturnPara32 = OV13850TRULYMIPI_PREVIEW_CLK;
					*pFeatureParaLen=4;
					break;
				case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
					*pFeatureReturnPara32 = OV13850TRULYMIPI_VIDEO_CLK;
					*pFeatureParaLen=4;
					break;
				case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
				case MSDK_SCENARIO_ID_CAMERA_ZSD:
					*pFeatureReturnPara32 = OV13850TRULYMIPI_CAPTURE_CLK;
					*pFeatureParaLen=4;
					break;
				default:
					*pFeatureReturnPara32 = OV13850TRULYMIPI_CAPTURE_CLK;
					*pFeatureParaLen=4;
					break;
			}
		      break;

        case SENSOR_FEATURE_SET_ESHUTTER:
            OV13850TRULY_SetShutter(*pFeatureData16);
            break;
        case SENSOR_FEATURE_SET_NIGHTMODE:
            OV13850TRULY_NightMode((BOOL) *pFeatureData16);
            break;
        case SENSOR_FEATURE_SET_GAIN:
            OV13850TRULY_SetGain((UINT16) *pFeatureData16);
            break;
        case SENSOR_FEATURE_SET_FLASHLIGHT:
            break;
        case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
            //OV13850TRULY_isp_master_clock=*pFeatureData32;
            break;
        case SENSOR_FEATURE_SET_REGISTER:
            OV13850TRULY_write_cmos_sensor(pSensorRegData->RegAddr, pSensorRegData->RegData);
            break;
        case SENSOR_FEATURE_GET_REGISTER:
            pSensorRegData->RegData = OV13850TRULY_read_cmos_sensor(pSensorRegData->RegAddr);
            break;
        case SENSOR_FEATURE_SET_CCT_REGISTER:
            SensorRegNumber=FACTORY_END_ADDR;
            for (i=0;i<SensorRegNumber;i++)
            {
            	spin_lock(&ov13850trulymipiraw_drv_lock);
                OV13850TRULYSensorCCT[i].Addr=*pFeatureData32++;
                OV13850TRULYSensorCCT[i].Para=*pFeatureData32++;
				spin_unlock(&ov13850trulymipiraw_drv_lock);
            }
            break;
        case SENSOR_FEATURE_GET_CCT_REGISTER:
            SensorRegNumber=FACTORY_END_ADDR;
            if (*pFeatureParaLen<(SensorRegNumber*sizeof(SENSOR_REG_STRUCT)+4))
                return FALSE;
            *pFeatureData32++=SensorRegNumber;
            for (i=0;i<SensorRegNumber;i++)
            {
                *pFeatureData32++=OV13850TRULYSensorCCT[i].Addr;
                *pFeatureData32++=OV13850TRULYSensorCCT[i].Para;
            }
            break;
        case SENSOR_FEATURE_SET_ENG_REGISTER:
            SensorRegNumber=ENGINEER_END;
            for (i=0;i<SensorRegNumber;i++)
            {
            	spin_lock(&ov13850trulymipiraw_drv_lock);
                OV13850TRULYSensorReg[i].Addr=*pFeatureData32++;
                OV13850TRULYSensorReg[i].Para=*pFeatureData32++;
				spin_unlock(&ov13850trulymipiraw_drv_lock);
            }
            break;
        case SENSOR_FEATURE_GET_ENG_REGISTER:
            SensorRegNumber=ENGINEER_END;
            if (*pFeatureParaLen<(SensorRegNumber*sizeof(SENSOR_REG_STRUCT)+4))
                return FALSE;
            *pFeatureData32++=SensorRegNumber;
            for (i=0;i<SensorRegNumber;i++)
            {
                *pFeatureData32++=OV13850TRULYSensorReg[i].Addr;
                *pFeatureData32++=OV13850TRULYSensorReg[i].Para;
            }
            break;
        case SENSOR_FEATURE_GET_REGISTER_DEFAULT:
            if (*pFeatureParaLen>=sizeof(NVRAM_SENSOR_DATA_STRUCT))
            {
                pSensorDefaultData->Version=NVRAM_CAMERA_SENSOR_FILE_VERSION;
                pSensorDefaultData->SensorId=OV13850_TRULY_SENSOR_ID;
                memcpy(pSensorDefaultData->SensorEngReg, OV13850TRULYSensorReg, sizeof(SENSOR_REG_STRUCT)*ENGINEER_END);
                memcpy(pSensorDefaultData->SensorCCTReg, OV13850TRULYSensorCCT, sizeof(SENSOR_REG_STRUCT)*FACTORY_END_ADDR);
            }
            else
                return FALSE;
            *pFeatureParaLen=sizeof(NVRAM_SENSOR_DATA_STRUCT);
            break;
        case SENSOR_FEATURE_GET_CONFIG_PARA:
            memcpy(pSensorConfigData, &OV13850TRULYSensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
            *pFeatureParaLen=sizeof(MSDK_SENSOR_CONFIG_STRUCT);
            break;
        case SENSOR_FEATURE_CAMERA_PARA_TO_SENSOR:
            OV13850TRULY_camera_para_to_sensor();
            break;

        case SENSOR_FEATURE_SENSOR_TO_CAMERA_PARA:
            OV13850TRULY_sensor_to_camera_para();
            break;
        case SENSOR_FEATURE_GET_GROUP_COUNT:
            *pFeatureReturnPara32++=OV13850TRULY_get_sensor_group_count();
            *pFeatureParaLen=4;
            break;
        case SENSOR_FEATURE_GET_GROUP_INFO:
            OV13850TRULY_get_sensor_group_info(pSensorGroupInfo->GroupIdx, pSensorGroupInfo->GroupNamePtr, &pSensorGroupInfo->ItemCount);
            *pFeatureParaLen=sizeof(MSDK_SENSOR_GROUP_INFO_STRUCT);
            break;
        case SENSOR_FEATURE_GET_ITEM_INFO:
            OV13850TRULY_get_sensor_item_info(pSensorItemInfo->GroupIdx,pSensorItemInfo->ItemIdx, pSensorItemInfo);
            *pFeatureParaLen=sizeof(MSDK_SENSOR_ITEM_INFO_STRUCT);
            break;

        case SENSOR_FEATURE_SET_ITEM_INFO:
            OV13850TRULY_set_sensor_item_info(pSensorItemInfo->GroupIdx, pSensorItemInfo->ItemIdx, pSensorItemInfo->ItemValue);
            *pFeatureParaLen=sizeof(MSDK_SENSOR_ITEM_INFO_STRUCT);
            break;

        case SENSOR_FEATURE_GET_ENG_INFO:
            pSensorEngInfo->SensorId = 129;
            pSensorEngInfo->SensorType = CMOS_SENSOR;
            pSensorEngInfo->SensorOutputDataFormat=SENSOR_OUTPUT_FORMAT_RAW_B;
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
            OV13850TRULYSetVideoMode(*pFeatureData16);
            break;
        case SENSOR_FEATURE_CHECK_SENSOR_ID:
            OV13850TRULYGetSensorID(pFeatureReturnPara32);
            break;
        case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE:
            OV13850TRULYSetAutoFlickerMode((BOOL)*pFeatureData16, *(pFeatureData16+1));
	        break;
        case SENSOR_FEATURE_SET_TEST_PATTERN:
            OV13850TRULYSetTestPatternMode((BOOL)*pFeatureData16);
            break;
		case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE:
			*pFeatureReturnPara32=OV13850TRULY_TEST_PATTERN_CHECKSUM;
			*pFeatureParaLen=4;
			break;
		case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
			OV13850TRULYMIPISetMaxFramerateByScenario((MSDK_SCENARIO_ID_ENUM)*pFeatureData32, *(pFeatureData32+1));
			break;
		case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
			OV13850TRULYMIPIGetDefaultFramerateByScenario((MSDK_SCENARIO_ID_ENUM)*pFeatureData32, (MUINT32 *)(*(pFeatureData32+1)));
			break;
        default:
            break;
    }
    return ERROR_NONE;
}	/* OV13850TRULYFeatureControl() */


SENSOR_FUNCTION_STRUCT	SensorFuncOV13850TRULY=
{
    OV13850TRULYOpen,
    OV13850TRULYGetInfo,
    OV13850TRULYGetResolution,
    OV13850TRULYFeatureControl,
    OV13850TRULYControl,
    OV13850TRULYClose
};

UINT32 OV13850TRULY_MIPI_RAW_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
    /* To Do : Check Sensor status here */
    if (pfFunc!=NULL)
        *pfFunc=&SensorFuncOV13850TRULY;

    return ERROR_NONE;
}   /* SensorInit() */
