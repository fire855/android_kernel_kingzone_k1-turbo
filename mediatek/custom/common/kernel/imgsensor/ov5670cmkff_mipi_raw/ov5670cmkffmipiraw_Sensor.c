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

#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"

#include "ov5670cmkffmipiraw_Sensor.h"
#include "ov5670cmkffmipiraw_Camera_Sensor_para.h"
#include "ov5670cmkffmipiraw_CameraCustomized.h"
static DEFINE_SPINLOCK(ov5670cmkffmipiraw_drv_lock);

#define OV5670CMKFF_DEBUG
#ifdef OV5670CMKFF_DEBUG
	#define OV5670CMKFFDB(fmt, arg...) xlog_printk(ANDROID_LOG_DEBUG, "[OV5670CMKFFRaw] ",  fmt, ##arg)
#else
	#define OV5670CMKFFDB(fmt, arg...)
#endif


kal_uint32 OV5670CMKFF_FeatureControl_PERIOD_PixelNum=OV5670CMKFF_PV_PERIOD_PIXEL_NUMS;
kal_uint32 OV5670CMKFF_FeatureControl_PERIOD_LineNum=OV5670CMKFF_PV_PERIOD_LINE_NUMS;

UINT16 OV5670CMKFF_VIDEO_MODE_TARGET_FPS = 30;

MSDK_SCENARIO_ID_ENUM OV5670CMKFFCurrentScenarioId = MSDK_SCENARIO_ID_CAMERA_PREVIEW;
MSDK_SENSOR_CONFIG_STRUCT OV5670CMKFFSensorConfigData;
static OV5670CMKFF_PARA_STRUCT OV5670CMKFF;
kal_uint32 OV5670CMKFF_FAC_SENSOR_REG;


SENSOR_REG_STRUCT OV5670CMKFFSensorCCT[]=CAMERA_SENSOR_CCT_DEFAULT_VALUE;
SENSOR_REG_STRUCT OV5670CMKFFSensorReg[ENGINEER_END]=CAMERA_SENSOR_REG_DEFAULT_VALUE;


#define OV5670CMKFF_TEST_PATTERN_CHECKSUM 0xca3667da //0x5d8082f0 //0x75bef806 //0xa2230d9f    //0xf5e2f1ce
kal_bool OV5670CMKFF_During_testpattern = KAL_FALSE;

extern int iReadReg(u16 a_u2Addr , u8 * a_puBuff , u16 i2cId);
extern int iWriteReg(u16 a_u2Addr , u32 a_u4Data , u32 a_u4Bytes , u16 i2cId);

#define OV5670CMKFF_write_cmos_sensor(addr, para) iWriteReg((u16) addr , (u32) para , 1, OV5670CMKFFMIPI_WRITE_ID)

kal_uint16 OV5670CMKFF_read_cmos_sensor(kal_uint32 addr)
{
kal_uint16 get_byte=0;
    iReadReg((u16) addr ,(u8*)&get_byte,OV5670CMKFFMIPI_WRITE_ID);
    return get_byte;
}
#define OV5670_OTP
#ifdef OV5670_OTP

struct otp_struct {
	UINT32 module_integrator_id;
	UINT32 lens_id;
	UINT32 production_year;
	UINT32 production_month;
	UINT32 production_day;
	UINT32 rg_ratio;
	UINT32 bg_ratio;
	UINT32 r_golden;
	UINT32 b_golden;
	UINT32 gr_golden;
	UINT32 gb_golden;
};
static int OV5670MIPI_check_awb_gain(){

	UINT8  	R_Gain_H;
	UINT8 	R_Gain_L;
	UINT8  	G_Gain_H;
	UINT8 	G_Gain_L;
	UINT8  	B_Gain_H;
	UINT8 	B_Gain_L;


	R_Gain_H=(UINT8)OV5670CMKFF_read_cmos_sensor(0x5032);
	R_Gain_L=(UINT8)OV5670CMKFF_read_cmos_sensor(0x5033);
	G_Gain_H=(UINT8)OV5670CMKFF_read_cmos_sensor(0x5034);
	G_Gain_L=(UINT8)OV5670CMKFF_read_cmos_sensor(0x5035);
	B_Gain_H=(UINT8)OV5670CMKFF_read_cmos_sensor(0x5036);
	B_Gain_L=(UINT8)OV5670CMKFF_read_cmos_sensor(0x5037);

	OV5670CMKFFDB("suny test R_Gain_H=0x%02x R_Gain_L=0x%02x G_Gain_H=0x%02x G_Gain_L=0x%02x B_Gain_H=0x%02x B_Gain_L=0x%02x\r\n",
					R_Gain_H,R_Gain_L,G_Gain_H,G_Gain_L,B_Gain_H,B_Gain_L);
}
static void OV5670_OTP_read_begin(void)
{
	// set reg 0x5002[3] to '0'
	int temp1;
	temp1 = OV5670CMKFF_read_cmos_sensor(0x5002);
	OV5670CMKFF_write_cmos_sensor(0x5002, (temp1 &(~0x08)));
}

static void OV5670_OTP_read_end(void)
{
	// set reg 0x5002[3] to '1'
	int temp1;
	temp1 = OV5670CMKFF_read_cmos_sensor(0x5002);
	OV5670CMKFF_write_cmos_sensor(0x5002, (0x08 | temp1));
}


// index: index of otp group. (1, 2, 3)
// return: 0, group index is empty
// 1, group index has invalid data
// 2, group index has valid data
static int check_otp_info(int index)
{
	int flag, i;
	int address_start = 0x7010;
	int address_end = 0x7010;
	OV5670_OTP_read_begin();
	// read otp into buffer
	OV5670CMKFF_write_cmos_sensor(0x3d84, 0xc0); // program disable, manual mode
	//partial mode OTP write start address
	OV5670CMKFF_write_cmos_sensor(0x3d88, (address_start>>8));
	OV5670CMKFF_write_cmos_sensor(0x3d89, (address_start & 0xff));
	// partial mode OTP write end address
	OV5670CMKFF_write_cmos_sensor(0x3d8A, (address_end>>8));

	OV5670CMKFF_write_cmos_sensor(0x3d8B, (address_end & 0xff));
	OV5670CMKFF_write_cmos_sensor(0x3d81, 0x01); // read otp
	mdelay(5);
	flag = OV5670CMKFF_read_cmos_sensor(0x7010);
	OV5670CMKFFDB("%s line %d flag %d",__func__,__LINE__,flag);
	//select group
	if(index==1)
	{
		flag = (flag>>6) & 0x03;
	}
	else if(index==2)
	{
		flag = (flag>>4) & 0x03;
	}
	else
	{
		flag = (flag>>2) & 0x03;
	}
	// clear otp buffer
	for (i=address_start;i<=address_end;i++) {
		OV5670CMKFF_write_cmos_sensor(i, 0x00);
	}
	OV5670_OTP_read_end();
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
// index: index of otp group. (1, 2, 3)
// return: 0, group index is empty
// 1, group index has invalid data
// 2, group index has valid data

static int check_otp_wb(int index)
{
	int flag, i;
	int address_start = 0x7041;
	int address_end = 0x7041;
	OV5670_OTP_read_begin();
	// read otp into buffer
	OV5670CMKFF_write_cmos_sensor(0x3d84, 0xc0); // program disable, manual mode
	//partial mode OTP write start address
	OV5670CMKFF_write_cmos_sensor(0x3d88, (address_start>>8));
	OV5670CMKFF_write_cmos_sensor(0x3d89, (address_start & 0xff));
	// partial mode OTP write end address
	OV5670CMKFF_write_cmos_sensor(0x3d8A, (address_end>>8));
	OV5670CMKFF_write_cmos_sensor(0x3d8B, (address_end & 0xff));
	OV5670CMKFF_write_cmos_sensor(0x3d81, 0x01); // read OTP
	mdelay(5);

	//select group
	flag = OV5670CMKFF_read_cmos_sensor(0x7041);
	OV5670CMKFFDB("AWB otp flag = %d \n",flag);

	if(index==1)
	{
		flag = (flag>>6) & 0x03;
	}
	else if(index==2)
	{
		flag = (flag>>4) & 0x03;
	}
	else
	{
		flag = (flag>>2) & 0x03;
	}
	// clear otp buffer
	for (i=address_start;i<=address_end;i++) {
		OV5670CMKFF_write_cmos_sensor(i, 0x00);
	}
	OV5670_OTP_read_end();
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

// index: index of otp group. (1, 2, 3)
// otp_ptr: pointer of otp_struct
// return: 0,
static int read_otp_info(int index, struct otp_struct *otp_ptr)
{
	int i;
	int address_start;
	int address_end;
	OV5670_OTP_read_begin();
	// read otp into buffer
	OV5670CMKFF_write_cmos_sensor(0x3d84, 0xc0); // program disable, manual mode
	//select group
	if(index==1)
	{
		address_start = 0x7011;
		address_end = 0x7015;
	}
	else if(index==2)
	{
		address_start = 0x7016;
		address_end = 0x701a;
	}
	else
	{
		address_start = 0x701b;
		address_end = 0x701f;
	}
	//partial mode OTP write start address
	OV5670CMKFF_write_cmos_sensor(0x3d88, (address_start>>8));
	OV5670CMKFF_write_cmos_sensor(0x3d89, (address_start & 0xff));
	// partial mode OTP write end address
	OV5670CMKFF_write_cmos_sensor(0x3d8A, (address_end>>8));
	OV5670CMKFF_write_cmos_sensor(0x3d8B, (address_end & 0xff));
	OV5670CMKFF_write_cmos_sensor(0x3d81, 0x01); // load otp into buffer

	mdelay(5);
	(*otp_ptr).module_integrator_id = OV5670CMKFF_read_cmos_sensor(address_start);
	(*otp_ptr).lens_id = OV5670CMKFF_read_cmos_sensor(address_start + 1);
	(*otp_ptr).production_year = OV5670CMKFF_read_cmos_sensor(address_start + 2);
	(*otp_ptr).production_month = OV5670CMKFF_read_cmos_sensor(address_start + 3);
	(*otp_ptr).production_day = OV5670CMKFF_read_cmos_sensor(address_start + 4);
	// clear otp buffer
	for (i=address_start;i<=address_end;i++) {
		OV5670CMKFF_write_cmos_sensor(i, 0x00);
	}
	OV5670_OTP_read_end();
	return 0;
}

// index: index of otp group. (1, 2, 3)
// otp_ptr: pointer of otp_struct
// return: 0,
static int read_otp_wb(UINT32 index, struct otp_struct *otp_ptr)
{
	UINT32 i, temp;
	UINT32 address_start;
	UINT32 address_end;
	//select group
	if(index==1){
		address_start = 0x7042;
		address_end = 0x705e;
	}
	else if(index==2){
		address_start = 0x705f;
		address_end = 0x707b;
	}
	else if(index==3){
		address_start = 0x707c;
		address_end = 0x7098;
	}
	OV5670_OTP_read_begin();
	// read otp into buffer
	OV5670CMKFF_write_cmos_sensor(0x3d84, 0xc0); // program disable, manual mode
	//partial mode OTP write start address
	OV5670CMKFF_write_cmos_sensor(0x3d88, (address_start>>8));
	OV5670CMKFF_write_cmos_sensor(0x3d89, (address_start & 0xff));
	// partial mode OTP write end address
	OV5670CMKFF_write_cmos_sensor(0x3d8A, (address_end>>8));
	OV5670CMKFF_write_cmos_sensor(0x3d8B, (address_end & 0xff));
	OV5670CMKFF_write_cmos_sensor(0x3d81, 0x01); // load otp into buffer
	mdelay(5);
	temp = OV5670CMKFF_read_cmos_sensor(address_start+1);
	otp_ptr->rg_ratio = (OV5670CMKFF_read_cmos_sensor(address_start)<<2) + ((temp>>6) & 0x03);
	temp = OV5670CMKFF_read_cmos_sensor(address_start+3);
	otp_ptr->bg_ratio = (OV5670CMKFF_read_cmos_sensor(address_start + 2)<<2) + ((temp>>6) & 0x03);
	temp = OV5670CMKFF_read_cmos_sensor(address_start+21);
	otp_ptr->r_golden = (OV5670CMKFF_read_cmos_sensor(address_start + 20)<<2) + ((temp>>6) & 0x03);
	temp = OV5670CMKFF_read_cmos_sensor(address_start+23);
	otp_ptr->b_golden = (OV5670CMKFF_read_cmos_sensor(address_start + 22)<<2) + ((temp>>6) & 0x03);
	temp = OV5670CMKFF_read_cmos_sensor(address_start+25);
	otp_ptr->gr_golden = (OV5670CMKFF_read_cmos_sensor(address_start + 24)<<2) + ((temp>>6) & 0x03);
	temp = OV5670CMKFF_read_cmos_sensor(address_start+27);
	otp_ptr->gb_golden = (OV5670CMKFF_read_cmos_sensor(address_start + 26)<<2) + ((temp>>6) & 0x03);
	// clear otp buffer

	for (i=address_start;i<=address_end;i++) {
		OV5670CMKFF_write_cmos_sensor(i, 0x00);
	}
	OV5670_OTP_read_end();

	OV5670CMKFFDB("AWB otp: r_golden = %d \n",otp_ptr->r_golden);
	OV5670CMKFFDB("AWB otp: b_golden = %d \n",otp_ptr->b_golden);
	OV5670CMKFFDB("AWB otp: gr_golden = %d \n",otp_ptr->gr_golden);
	OV5670CMKFFDB("AWB otp: gb_golden = %d \n",otp_ptr->gb_golden);

	
	return 0;
}
// R_gain, sensor red gain of AWB, 0x400 =1
// G_gain, sensor green gain of AWB, 0x400 =1
// B_gain, sensor blue gain of AWB, 0x400 =1
// return 0;
static int update_awb_gain(UINT32 R_gain, UINT32 G_gain, UINT32 B_gain)
{
	UINT32	uR_gain,uG_gain,uB_gain;

	UINT8  	R_Gain_H;
	UINT8 	R_Gain_L;
	UINT8  	G_Gain_H;
	UINT8 	G_Gain_L;
	UINT8  	B_Gain_H;
	UINT8 	B_Gain_L;

	R_Gain_H=(UINT8)(R_gain>>8);
	R_Gain_L=(UINT8)(R_gain&0x00FF);
	G_Gain_H=(UINT8)(G_gain>>8);
	G_Gain_L=(UINT8)(G_gain&0x00FF);
	B_Gain_H=(UINT8)(B_gain>>8);
	B_Gain_L=(UINT8)(B_gain&0x00FF);
	if (R_gain>=0x400) {
		OV5670CMKFF_write_cmos_sensor(0x5032, R_Gain_H);
		OV5670CMKFF_write_cmos_sensor(0x5033, R_Gain_L);
	}
	if (G_gain>=0x400) {
		OV5670CMKFF_write_cmos_sensor(0x5034, G_Gain_H);
		OV5670CMKFF_write_cmos_sensor(0x5035, G_Gain_L);
	}
	if (B_gain>=0x400) {
		OV5670CMKFF_write_cmos_sensor(0x5036, B_Gain_H);
		OV5670CMKFF_write_cmos_sensor(0x5037, B_Gain_L);
	}
	return 0;
}

// call this function after OV5670 initialization
// return value: 0 update success
// 1, no OTP
static int update_otp_wb()
{
	struct otp_struct current_otp;
	UINT32 i;
	UINT32 otp_index;
	UINT32 temp,BG_Ratio_Typical,RG_Ratio_Typical;
	UINT32 R_gain, G_gain, B_gain, G_gain_R, G_gain_B;
	UINT32 rg,bg,r_golden,b_golden,gr_golden,gb_golden;

// R/G and B/G of current camera module is read out from sensor OTP
// check first OTP with valid data
	for(i=1;i<=3;i++) {
		temp = check_otp_wb(i);
		if (temp == 2) 
		{
			otp_index = i;
			break;
		}
	}
	printk("suny test OTP index----%d---\r\n",otp_index);
	if (i>3) {
	// no valid wb OTP data
		printk("suny test no wb OTP-------\r\n");
		return 1;
	}
	read_otp_wb(otp_index, &current_otp);
	rg = current_otp.rg_ratio;
	bg = current_otp.bg_ratio;
	
	r_golden = current_otp.r_golden;
	b_golden = current_otp.b_golden;
	gr_golden = current_otp.gr_golden;
	gb_golden = current_otp.gb_golden;
	//BG_Ratio_Typical = b_golden / ((gr_golden+gb_golden)>>1);
	//RG_Ratio_Typical = r_golden / ((gr_golden+gb_golden)>>1);
	BG_Ratio_Typical=0x12E;
	RG_Ratio_Typical=0x139;
	OV5670CMKFFDB("suny test ration rg bg [0x%x,0x%x]\r\n",rg,bg);
	OV5670CMKFFDB("suny test r b gr gb[0x%x,0x%x,0x%x,0x%x]\r\n",r_golden,b_golden,gr_golden,gb_golden);
	OV5670CMKFFDB("suny test BG_Ratio_Typical=0x%04x  RG_Ratio_Typical=0x%04x\r\n",BG_Ratio_Typical,RG_Ratio_Typical);
	//calculate G gain
	//0x400 = 1x gain
	if(bg < BG_Ratio_Typical) {
		if (rg< RG_Ratio_Typical) {
			// current_otp.bg_ratio < BG_Ratio_typical &&
			// current_otp.rg_ratio < RG_Ratio_typical
			G_gain = 0x400;
			B_gain = 0x400 * BG_Ratio_Typical / bg;
			R_gain = 0x400 * RG_Ratio_Typical / rg;
		}
		else {
			// current_otp.bg_ratio < BG_Ratio_typical &&
			// current_otp.rg_ratio >= RG_Ratio_typical
			R_gain = 0x400;
			G_gain = 0x400 * rg / RG_Ratio_Typical;
			B_gain = G_gain * BG_Ratio_Typical /bg;
		}
	}
	else {
		if (rg < RG_Ratio_Typical) {
			// current_otp.bg_ratio >= BG_Ratio_typical &&
			// current_otp.rg_ratio < RG_Ratio_typical
			B_gain = 0x400;
			G_gain = 0x400 * bg / BG_Ratio_Typical;
			R_gain = G_gain * RG_Ratio_Typical / rg;
		}
		else {
			// current_otp.bg_ratio >= BG_Ratio_typical &&
			// current_otp.rg_ratio >= RG_Ratio_typical
			G_gain_B = 0x400 * bg / BG_Ratio_Typical;
			G_gain_R = 0x400 * rg / RG_Ratio_Typical;
			if(G_gain_B > G_gain_R ) {
				B_gain = 0x400;
				G_gain = G_gain_B;
				R_gain = G_gain * RG_Ratio_Typical /rg;
			}
			else {
				R_gain = 0x400;
				G_gain = G_gain_R;
				B_gain = G_gain * BG_Ratio_Typical / bg;
			}
		}
	}
	OV5670MIPI_check_awb_gain();
	OV5670CMKFFDB("suny test uR_gain=0x%04x,uG_gain=0x%04x,uB_gain=0x%04x\r\n",R_gain,G_gain,B_gain);
	update_awb_gain(R_gain, G_gain, B_gain);
	OV5670MIPI_check_awb_gain();
	return 0;
}

static int read_module_integrator_id()
{
	struct otp_struct current_otp;
	UINT32 otp_index;
	int i, temp;

    OV5670CMKFF_write_cmos_sensor(0x0100, 0x01);
// R/G and B/G of current camera module is read out from sensor OTP
// check first OTP with valid data
	for(i=1;i<=3;i++) {
		temp = check_otp_info(i);
		if (temp == 2) 
		{
			otp_index = i;
			break;
		}
	}
	
	if (i>3) {
	// no valid wb OTP data
		printk("suny test no wb OTP-------\r\n");
		return 0;
	}
	
	read_otp_info(otp_index, &current_otp);
	
	return (current_otp.module_integrator_id << 8 | current_otp.lens_id);
}
#endif
void OV5670CMKFF_Init_Para(void)
{

	spin_lock(&ov5670cmkffmipiraw_drv_lock);
	OV5670CMKFF.sensorMode = SENSOR_MODE_INIT;
	OV5670CMKFF.OV5670CMKFFAutoFlickerMode = KAL_FALSE;
	OV5670CMKFF.OV5670CMKFFVideoMode = KAL_FALSE;
	OV5670CMKFF.DummyLines= 0;
	OV5670CMKFF.DummyPixels= 0;
	OV5670CMKFF.pvPclk =  (10285); 
	OV5670CMKFF.videoPclk = (10285);
	OV5670CMKFF.capPclk = (10285);

	OV5670CMKFF.shutter = 0x4C00;
	OV5670CMKFF.ispBaseGain = BASEGAIN;
	OV5670CMKFF.sensorGlobalGain = 0x0200;
	spin_unlock(&ov5670cmkffmipiraw_drv_lock);
}

kal_uint32 GetOV5670CMKFFLineLength(void)
{
	kal_uint32 OV5670CMKFF_line_length = 0;
	if ( SENSOR_MODE_PREVIEW == OV5670CMKFF.sensorMode )  
	{
		OV5670CMKFF_line_length = OV5670CMKFF_PV_PERIOD_PIXEL_NUMS + OV5670CMKFF.DummyPixels;
	}
	else if( SENSOR_MODE_VIDEO == OV5670CMKFF.sensorMode ) 
	{
		OV5670CMKFF_line_length = OV5670CMKFF_VIDEO_PERIOD_PIXEL_NUMS + OV5670CMKFF.DummyPixels;
	}
	else
	{
		OV5670CMKFF_line_length = OV5670CMKFF_FULL_PERIOD_PIXEL_NUMS + OV5670CMKFF.DummyPixels;
	}

    return OV5670CMKFF_line_length;

}


kal_uint32 GetOV5670CMKFFFrameLength(void)
{
	kal_uint32 OV5670CMKFF_frame_length = 0;

	if ( SENSOR_MODE_PREVIEW == OV5670CMKFF.sensorMode )  
	{
		OV5670CMKFF_frame_length = OV5670CMKFF_PV_PERIOD_LINE_NUMS + OV5670CMKFF.DummyLines ;
	}
	else if( SENSOR_MODE_VIDEO == OV5670CMKFF.sensorMode ) 
	{
		OV5670CMKFF_frame_length = OV5670CMKFF_VIDEO_PERIOD_LINE_NUMS + OV5670CMKFF.DummyLines ;
	}
	else
	{
		OV5670CMKFF_frame_length = OV5670CMKFF_FULL_PERIOD_LINE_NUMS + OV5670CMKFF.DummyLines ;
	}

	return OV5670CMKFF_frame_length;
}


kal_uint32 OV5670CMKFF_CalcExtra_For_ShutterMargin(kal_uint32 shutter_value,kal_uint32 shutterLimitation)
{
    kal_uint32 extra_lines = 0;

	
	if (shutter_value <4 ){
		shutter_value = 4;
	}

	
	if (shutter_value > shutterLimitation)
	{
		extra_lines = shutter_value - shutterLimitation;
    }
	else
		extra_lines = 0;

    return extra_lines;

}


kal_uint32 OV5670CMKFF_CalcFrameLength_For_AutoFlicker(void)
{

    kal_uint32 AutoFlicker_min_framelength = 0;

	switch(OV5670CMKFFCurrentScenarioId)
	{
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			AutoFlicker_min_framelength = (OV5670CMKFF.capPclk*10000) /(OV5670CMKFF_FULL_PERIOD_PIXEL_NUMS + OV5670CMKFF.DummyPixels)/OV5670CMKFF_AUTOFLICKER_OFFSET_25*10 ;
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			if(OV5670CMKFF_VIDEO_MODE_TARGET_FPS==30)
			{
				AutoFlicker_min_framelength = (OV5670CMKFF.videoPclk*10000) /(OV5670CMKFF_VIDEO_PERIOD_PIXEL_NUMS + OV5670CMKFF.DummyPixels)/OV5670CMKFF_AUTOFLICKER_OFFSET_30*10 ;
			}
			else if(OV5670CMKFF_VIDEO_MODE_TARGET_FPS==15)
			{
				AutoFlicker_min_framelength = (OV5670CMKFF.videoPclk*10000) /(OV5670CMKFF_VIDEO_PERIOD_PIXEL_NUMS + OV5670CMKFF.DummyPixels)/OV5670CMKFF_AUTOFLICKER_OFFSET_15*10 ;
			}
			else
			{
				AutoFlicker_min_framelength = OV5670CMKFF_VIDEO_PERIOD_LINE_NUMS + OV5670CMKFF.DummyLines;
			}
			break;
			
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			AutoFlicker_min_framelength = (OV5670CMKFF.pvPclk*10000) /(OV5670CMKFF_PV_PERIOD_PIXEL_NUMS + OV5670CMKFF.DummyPixels)/OV5670CMKFF_AUTOFLICKER_OFFSET_30*10 ;
			break;
	}

	OV5670CMKFFDB("AutoFlicker_min_framelength =%d,OV5670CMKFFCurrentScenarioId =%d\n", AutoFlicker_min_framelength,OV5670CMKFFCurrentScenarioId);

	return AutoFlicker_min_framelength;

}


void OV5670CMKFF_write_shutter(kal_uint32 shutter)
{
	kal_uint32 min_framelength = OV5670CMKFF_PV_PERIOD_PIXEL_NUMS, max_shutter=0;
	kal_uint32 line_length = 0;
	kal_uint32 frame_length = 0;
	unsigned long flags;

	//for test
	//shutter = 0x7fc;  //issue 
	
	
    line_length  = GetOV5670CMKFFLineLength();
	frame_length = GetOV5670CMKFFFrameLength();
	
	max_shutter  = frame_length-OV5670CMKFF_SHUTTER_MARGIN;

    frame_length = frame_length + OV5670CMKFF_CalcExtra_For_ShutterMargin(shutter,max_shutter);
	


	if(OV5670CMKFF.OV5670CMKFFAutoFlickerMode == KAL_TRUE)
	{
        min_framelength = OV5670CMKFF_CalcFrameLength_For_AutoFlicker();

        if(frame_length < min_framelength)
			frame_length = min_framelength;
	}
	

	spin_lock_irqsave(&ov5670cmkffmipiraw_drv_lock,flags);
	OV5670CMKFF_FeatureControl_PERIOD_PixelNum = line_length;
	OV5670CMKFF_FeatureControl_PERIOD_LineNum = frame_length;
	spin_unlock_irqrestore(&ov5670cmkffmipiraw_drv_lock,flags);

	//Set total frame length  //VTS
	OV5670CMKFF_write_cmos_sensor(0x380e, (frame_length >> 8) & 0xFF);
	OV5670CMKFF_write_cmos_sensor(0x380f, frame_length & 0xFF);

	//Set shutter 
	OV5670CMKFF_write_cmos_sensor(0x3500, (shutter>>12) & 0x0F);
	OV5670CMKFF_write_cmos_sensor(0x3501, (shutter>>4) & 0xFF);
	OV5670CMKFF_write_cmos_sensor(0x3502, (shutter<<4) & 0xF0);	  /* Don't use the fraction part. */

	OV5670CMKFFDB("ov5670 write shutter=%x, line_length=%x, frame_length=%x\n", shutter, line_length, frame_length);

}


static kal_uint16 OV5670CMKFFReg2Gain(const kal_uint16 iReg)
{
    kal_uint16 iGain =0; 

	iGain = iReg*BASEGAIN/OV5670CMKFF_GAIN_BASE;
	return iGain;
	
}

static kal_uint16 OV5670CMKFFGain2Reg(const kal_uint16 Gain)
{
    kal_uint32 iReg = 0x0000;
	kal_uint32 TempGain = BASEGAIN;


	TempGain = Gain;
	if(TempGain < BASEGAIN){
		TempGain = BASEGAIN;
		//OV5670CMKFFDB("###ov5670 write gain underflow### Gain =%x\n", Gain);
	}
	if(TempGain > 16*BASEGAIN){
		TempGain = 16*BASEGAIN;
		//OV5670CMKFFDB("###ov5670 write gain overflow### Gain =%x\n", Gain);
	}

	iReg = (TempGain*OV5670CMKFF_GAIN_BASE)/BASEGAIN;

	//iReg = ((TempGain /BASEGAIN)<<7)+((TempGain % BASEGAIN)<<7/BASEGAIN);
	iReg = iReg & 0xFFFF;

	//OV5670CMKFFDB("###ov5670 write Reg ### iReg =%x\n", iReg);

    return iReg;

}

void write_OV5670CMKFF_gain(kal_uint16 gain)
{
	kal_uint16 iGain =1;
	kal_uint8 ChangeFlag=0x01;

	kal_uint16 read_gain;
	
	iGain=(gain / OV5670CMKFF_GAIN_BASE);

	if(iGain<2){
		ChangeFlag= 0x00;
	}
	else if(iGain<4){
		ChangeFlag= 0x01;
	}
	else if(iGain<8){
		ChangeFlag= 0x03;
	}
	else{
		ChangeFlag= 0x07;
	}

	//ChangeFlag= 0x07;
	
	OV5670CMKFF_write_cmos_sensor(0x301d, 0xf0);
	OV5670CMKFF_write_cmos_sensor(0x3209, 0x00);
	OV5670CMKFF_write_cmos_sensor(0x320a, 0x01);
	
	//group write  hold
	//group 0:delay 0x366a for one frame,then active with gain
	OV5670CMKFF_write_cmos_sensor(0x3208, 0x00);
	OV5670CMKFF_write_cmos_sensor(0x366a, ChangeFlag);
	OV5670CMKFF_write_cmos_sensor(0x3208, 0x10);

	//group 1:all other registers( gain)
	OV5670CMKFF_write_cmos_sensor(0x3208, 0x01);
	OV5670CMKFF_write_cmos_sensor(0x3508,(gain>>8));
    OV5670CMKFF_write_cmos_sensor(0x3509,(gain&0xff));
	
	OV5670CMKFF_write_cmos_sensor(0x3208, 0x11);

	//group lanch
	OV5670CMKFF_write_cmos_sensor(0x320B, 0x15);
	OV5670CMKFF_write_cmos_sensor(0x3208, 0xA1);

	//read_gain=(((OV5670CMKFF_read_cmos_sensor(0x3508)&0x1F) << 8) | OV5670CMKFF_read_cmos_sensor(0x3509));
	//OV5670CMKFFDB("[OV5670CMKFF_SetGain]0x3508|0x3509=0x%x \n",read_gain);
	//OV5670CMKFFDB("[OV5670CMKFF_SetGain]0x366a=%d \n",(OV5670CMKFF_read_cmos_sensor(0x366a)));

	return;

}

void OV5670CMKFF_SetGain(UINT16 iGain)
{
	unsigned long flags;
	spin_lock_irqsave(&ov5670cmkffmipiraw_drv_lock,flags);

	OV5670CMKFFDB("OV5670CMKFF_SetGain iGain = %d :\n ",iGain);
	
	OV5670CMKFF.realGain = iGain;
	OV5670CMKFF.sensorGlobalGain = OV5670CMKFFGain2Reg(iGain);
	spin_unlock_irqrestore(&ov5670cmkffmipiraw_drv_lock,flags);

	write_OV5670CMKFF_gain(OV5670CMKFF.sensorGlobalGain);
	OV5670CMKFFDB(" [OV5670CMKFF_SetGain]OV5670CMKFF.sensorGlobalGain=0x%x,OV5670CMKFF.realGain =%d",OV5670CMKFF.sensorGlobalGain,
		OV5670CMKFF.realGain); 

	//temperature test
	//OV5670CMKFF_write_cmos_sensor(0x4d12,0x01);
	//OV5670CMKFFDB("Temperature read_reg  0x4d13  =%x \n",OV5670CMKFF_read_cmos_sensor(0x4d13));
}   

kal_uint16 read_OV5670CMKFF_gain(void)
{
	kal_uint16 read_gain=0;

	read_gain=(((OV5670CMKFF_read_cmos_sensor(0x3508)&0x1F) << 8) | OV5670CMKFF_read_cmos_sensor(0x3509));

	spin_lock(&ov5670cmkffmipiraw_drv_lock);
	OV5670CMKFF.sensorGlobalGain = read_gain;
	OV5670CMKFF.realGain = OV5670CMKFFReg2Gain(OV5670CMKFF.sensorGlobalGain);
	spin_unlock(&ov5670cmkffmipiraw_drv_lock);

	OV5670CMKFFDB("OV5670CMKFF.sensorGlobalGain=0x%x,OV5670CMKFF.realGain=%d\n",OV5670CMKFF.sensorGlobalGain,OV5670CMKFF.realGain);

	return OV5670CMKFF.sensorGlobalGain;
}  


#if 1
void OV5670CMKFF_camera_para_to_sensor(void)
{
    kal_uint32    i;
    for(i=0; 0xFFFFFFFF!=OV5670CMKFFSensorReg[i].Addr; i++)
    {
        OV5670CMKFF_write_cmos_sensor(OV5670CMKFFSensorReg[i].Addr, OV5670CMKFFSensorReg[i].Para);
    }
    for(i=ENGINEER_START_ADDR; 0xFFFFFFFF!=OV5670CMKFFSensorReg[i].Addr; i++)
    {
        OV5670CMKFF_write_cmos_sensor(OV5670CMKFFSensorReg[i].Addr, OV5670CMKFFSensorReg[i].Para);
    }
    for(i=FACTORY_START_ADDR; i<FACTORY_END_ADDR; i++)
    {
        OV5670CMKFF_write_cmos_sensor(OV5670CMKFFSensorCCT[i].Addr, OV5670CMKFFSensorCCT[i].Para);
    }
}

void OV5670CMKFF_sensor_to_camera_para(void)
{
    kal_uint32    i, temp_data;
    for(i=0; 0xFFFFFFFF!=OV5670CMKFFSensorReg[i].Addr; i++)
    {
         temp_data = OV5670CMKFF_read_cmos_sensor(OV5670CMKFFSensorReg[i].Addr);
		 spin_lock(&ov5670cmkffmipiraw_drv_lock);
		 OV5670CMKFFSensorReg[i].Para =temp_data;
		 spin_unlock(&ov5670cmkffmipiraw_drv_lock);
    }
    for(i=ENGINEER_START_ADDR; 0xFFFFFFFF!=OV5670CMKFFSensorReg[i].Addr; i++)
    {
        temp_data = OV5670CMKFF_read_cmos_sensor(OV5670CMKFFSensorReg[i].Addr);
		spin_lock(&ov5670cmkffmipiraw_drv_lock);
		OV5670CMKFFSensorReg[i].Para = temp_data;
		spin_unlock(&ov5670cmkffmipiraw_drv_lock);
    }
}

kal_int32  OV5670CMKFF_get_sensor_group_count(void)
{
    return GROUP_TOTAL_NUMS;
}

void OV5670CMKFF_get_sensor_group_info(kal_uint16 group_idx, kal_int8* group_name_ptr, kal_int32* item_count_ptr)
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

void OV5670CMKFF_get_sensor_item_info(kal_uint16 group_idx,kal_uint16 item_idx, MSDK_SENSOR_ITEM_INFO_STRUCT* info_ptr)
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
                 ASSERT(0);
          }

            temp_para= OV5670CMKFFSensorCCT[temp_addr].Para;
			//temp_gain= (temp_para/OV5670CMKFF.sensorBaseGain) * 1000;

            info_ptr->ItemValue=temp_gain;
            info_ptr->IsTrueFalse=KAL_FALSE;
            info_ptr->IsReadOnly=KAL_FALSE;
            info_ptr->IsNeedRestart=KAL_FALSE;
            info_ptr->Min= OV5670CMKFF_MIN_ANALOG_GAIN * 1000;
            info_ptr->Max= OV5670CMKFF_MAX_ANALOG_GAIN * 1000;
            break;
        case CMMCLK_CURRENT:
            switch (item_idx)
            {
                case 0:
                    sprintf((char *)info_ptr->ItemNamePtr,"Drv Cur[2,4,6,8]mA");

                    //temp_reg=MT9P017SensorReg[CMMCLK_CURRENT_INDEX].Para;
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
                    info_ptr->ItemValue=    111;  
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



kal_bool OV5670CMKFF_set_sensor_item_info(kal_uint16 group_idx, kal_uint16 item_idx, kal_int32 ItemValue)
{
   kal_uint16  temp_gain=0,temp_addr=0, temp_para=0;

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
                 ASSERT(0);
          }

		 temp_gain=((ItemValue*BASEGAIN+500)/1000);			//+500:get closed integer value

		  if(temp_gain>=1*BASEGAIN && temp_gain<=16*BASEGAIN)
          {
//             temp_para=(temp_gain * OV5670CMKFF.sensorBaseGain + BASEGAIN/2)/BASEGAIN;
          }
          else
			  ASSERT(0);

		  spin_lock(&ov5670cmkffmipiraw_drv_lock);
          OV5670CMKFFSensorCCT[temp_addr].Para = temp_para;
		  spin_unlock(&ov5670cmkffmipiraw_drv_lock);
          OV5670CMKFF_write_cmos_sensor(OV5670CMKFFSensorCCT[temp_addr].Addr,temp_para);

            break;
        case CMMCLK_CURRENT:
            switch (item_idx)
            {
                case 0:
                    //no need to apply this item for driving current
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
					spin_lock(&ov5670cmkffmipiraw_drv_lock);
                    OV5670CMKFF_FAC_SENSOR_REG=ItemValue;
					spin_unlock(&ov5670cmkffmipiraw_drv_lock);
                    break;
                case 1:
                    OV5670CMKFF_write_cmos_sensor(OV5670CMKFF_FAC_SENSOR_REG,ItemValue);
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
#endif


static void OV5670CMKFF_SetDummy( const kal_uint32 iPixels, const kal_uint32 iLines )
{
	kal_uint32 line_length = 0;
	kal_uint32 frame_length = 0;

	if ( SENSOR_MODE_PREVIEW == OV5670CMKFF.sensorMode )
	{
		line_length = OV5670CMKFF_PV_PERIOD_PIXEL_NUMS + iPixels;
		frame_length = OV5670CMKFF_PV_PERIOD_LINE_NUMS + iLines;
	}
	else if( SENSOR_MODE_VIDEO== OV5670CMKFF.sensorMode )
	{
		line_length = OV5670CMKFF_VIDEO_PERIOD_PIXEL_NUMS + iPixels;
		frame_length = OV5670CMKFF_VIDEO_PERIOD_LINE_NUMS + iLines;
	}
	else
	{
		line_length = OV5670CMKFF_FULL_PERIOD_PIXEL_NUMS + iPixels;
		frame_length = OV5670CMKFF_FULL_PERIOD_LINE_NUMS + iLines;
	}

	spin_lock(&ov5670cmkffmipiraw_drv_lock);
	OV5670CMKFF_FeatureControl_PERIOD_PixelNum = line_length;
	OV5670CMKFF_FeatureControl_PERIOD_LineNum = frame_length;
	spin_unlock(&ov5670cmkffmipiraw_drv_lock);

	//Set total frame length
	OV5670CMKFF_write_cmos_sensor(0x380e, (frame_length >> 8) & 0xFF);
	OV5670CMKFF_write_cmos_sensor(0x380f, frame_length & 0xFF);
	//Set total line length
	OV5670CMKFF_write_cmos_sensor(0x380c, (line_length >> 8) & 0xFF);
	OV5670CMKFF_write_cmos_sensor(0x380d, line_length & 0xFF);

}   


void OV5670CMKFFPreviewSetting(void)
{
	OV5670CMKFFDB(" OV5670CMKFFPreviewSetting_2lane enter\n");

	/*  //
	
	//@@PV_Quarter_size_30fps_800Mbps/lane				  
	//99 1296 960 										  
	//;;102 3601	157c									  
	//;;PCLK=HTS*VTS*fps=0x68c*0x7fd*30=1676*2045*30=102.85M

		OV5670CMKFF_write_cmos_sensor(0x0100, 0x00);  //
	
	OV5670CMKFF_write_cmos_sensor(0x3501, 0x3d);  //
	OV5670CMKFF_write_cmos_sensor(0x366e, 0x08);  //
	OV5670CMKFF_write_cmos_sensor(0x370b, 0x1b);  //
	OV5670CMKFF_write_cmos_sensor(0x3808, 0x05);  //
	OV5670CMKFF_write_cmos_sensor(0x3809, 0x10);  //
	OV5670CMKFF_write_cmos_sensor(0x380a, 0x03);  //
	OV5670CMKFF_write_cmos_sensor(0x380b, 0xc0);  //
	OV5670CMKFF_write_cmos_sensor(0x380c, 0x06);  //
	OV5670CMKFF_write_cmos_sensor(0x380d, 0x8c);  //
	OV5670CMKFF_write_cmos_sensor(0x380e, 0x07);  //;03
	OV5670CMKFF_write_cmos_sensor(0x380f, 0xfd);  //;e0
	OV5670CMKFF_write_cmos_sensor(0x3814, 0x03);  //
	OV5670CMKFF_write_cmos_sensor(0x3820, 0x90);  //
	OV5670CMKFF_write_cmos_sensor(0x3821, 0x47);  //
	OV5670CMKFF_write_cmos_sensor(0x382a, 0x03);  //
	OV5670CMKFF_write_cmos_sensor(0x4009, 0x05);  //
	OV5670CMKFF_write_cmos_sensor(0x4502, 0x48);  //
	OV5670CMKFF_write_cmos_sensor(0x4508, 0x55);  //
	OV5670CMKFF_write_cmos_sensor(0x4509, 0x55);  //
	OV5670CMKFF_write_cmos_sensor(0x4600, 0x00);  //
	OV5670CMKFF_write_cmos_sensor(0x4601, 0x81);  //
	OV5670CMKFF_write_cmos_sensor(0x4017, 0x10);  //; threshold = 4LSB for Binning sum format.
	OV5670CMKFF_write_cmos_sensor(0x400a, 0x02);  //;
	OV5670CMKFF_write_cmos_sensor(0x400b, 0x00);  //; 
	
	OV5670CMKFF_write_cmos_sensor(0x0100, 0x01);  //
*/

	//@@PV_Quarter_size_30fps_800Mbps/lane_1296x972							  
	//99 1296 972 															  
	//;;102 3601	157c														  
	//;;PCLK=HTS*VTS*fps=0x68c*0x7fd*30=1676*2045*30=102.85M					  
																			  
	OV5670CMKFF_write_cmos_sensor(0x0100, 0x00);  // 	
	
	OV5670CMKFF_write_cmos_sensor(0x3501, 0x73);  // 							  
	OV5670CMKFF_write_cmos_sensor(0x3502, 0x00);  // 							  
	OV5670CMKFF_write_cmos_sensor(0x3508, 0x01);  // 							  
	OV5670CMKFF_write_cmos_sensor(0x3509, 0x80);  // 							  
	OV5670CMKFF_write_cmos_sensor(0x366e, 0x08);  // 							  
	OV5670CMKFF_write_cmos_sensor(0x370b, 0x1b);  // 							  
	OV5670CMKFF_write_cmos_sensor(0x3808, 0x05);  // 							  
	OV5670CMKFF_write_cmos_sensor(0x3809, 0x10);  // 							  
	OV5670CMKFF_write_cmos_sensor(0x380a, 0x03);  // 							  
	OV5670CMKFF_write_cmos_sensor(0x380b, 0xcc);  //;c0							  
	OV5670CMKFF_write_cmos_sensor(0x380c, 0x06);  // 							  
	OV5670CMKFF_write_cmos_sensor(0x380d, 0x8c);  // 							  
	OV5670CMKFF_write_cmos_sensor(0x380e, 0x07);  //;03							  
	OV5670CMKFF_write_cmos_sensor(0x380f, 0xfd);  //;e0							  
	OV5670CMKFF_write_cmos_sensor(0x3814, 0x03);  // 							  
	OV5670CMKFF_write_cmos_sensor(0x3820, 0x90);  // 							  
	OV5670CMKFF_write_cmos_sensor(0x3821, 0x47);  // 							  
	OV5670CMKFF_write_cmos_sensor(0x382a, 0x03);  // 							  
	OV5670CMKFF_write_cmos_sensor(0x3845, 0x02);  // 							  
	OV5670CMKFF_write_cmos_sensor(0x4009, 0x05);  // 							  
	OV5670CMKFF_write_cmos_sensor(0x4502, 0x48);  // 							  
	OV5670CMKFF_write_cmos_sensor(0x4508, 0x55);  // 							  
	OV5670CMKFF_write_cmos_sensor(0x4509, 0x55);  // 							  
	OV5670CMKFF_write_cmos_sensor(0x4600, 0x00);  // 							  
	OV5670CMKFF_write_cmos_sensor(0x4601, 0x81);  // 							  
	OV5670CMKFF_write_cmos_sensor(0x4017, 0x10);  //; threshold = 4LSB for Binning 
	OV5670CMKFF_write_cmos_sensor(0x400a, 0x02);  //;							  
	OV5670CMKFF_write_cmos_sensor(0x400b, 0x00);  //;	
	
	OV5670CMKFF_write_cmos_sensor(0x0100, 0x01);  // 	
}


void OV5670CMKFFVideoSetting(void)
{
	OV5670CMKFFDB(" OV5670CMKFFvideoSetting_2lane enter:video/preview sync\n");

	OV5670CMKFFPreviewSetting();
}



void OV5670CMKFFCaptureSetting(void)
{
	OV5670CMKFFDB("OV5670CMKFFCaptureSetting_2lane enter\n");

	OV5670CMKFF_write_cmos_sensor(0x0100, 0x00); 
	
	OV5670CMKFF_write_cmos_sensor(0x3501, 0x5f); //long exposure
	OV5670CMKFF_write_cmos_sensor(0x3502, 0xd0);  //long exposure
	
	OV5670CMKFF_write_cmos_sensor(0x3508, 0x03);  //gain
	OV5670CMKFF_write_cmos_sensor(0x3509, 0x00);  //gain
	
	OV5670CMKFF_write_cmos_sensor(0x366e, 0x10); 
	OV5670CMKFF_write_cmos_sensor(0x370b, 0x1b); 
	OV5670CMKFF_write_cmos_sensor(0x3808, 0x0a); 
	OV5670CMKFF_write_cmos_sensor(0x3809, 0x20); 
	OV5670CMKFF_write_cmos_sensor(0x380a, 0x07); 
	OV5670CMKFF_write_cmos_sensor(0x380b, 0x98); 
	OV5670CMKFF_write_cmos_sensor(0x380c, 0x07); //;06
	OV5670CMKFF_write_cmos_sensor(0x380d, 0xdc); //;8c
	OV5670CMKFF_write_cmos_sensor(0x380e, 0x07); 
	OV5670CMKFF_write_cmos_sensor(0x380f, 0xfd); 
	OV5670CMKFF_write_cmos_sensor(0x3814, 0x01); 
	OV5670CMKFF_write_cmos_sensor(0x3820, 0x80); 
	OV5670CMKFF_write_cmos_sensor(0x3821, 0x46); 
	OV5670CMKFF_write_cmos_sensor(0x382a, 0x01);
	
	OV5670CMKFF_write_cmos_sensor(0x3845, 0x00);  //v_offset for auto size mode
	
	OV5670CMKFF_write_cmos_sensor(0x4009, 0x0d); 
	OV5670CMKFF_write_cmos_sensor(0x4502, 0x40); 
	OV5670CMKFF_write_cmos_sensor(0x4508, 0xaa); 
	OV5670CMKFF_write_cmos_sensor(0x4509, 0xaa); 
	OV5670CMKFF_write_cmos_sensor(0x4600, 0x01); 
	OV5670CMKFF_write_cmos_sensor(0x4601, 0x03); 
	OV5670CMKFF_write_cmos_sensor(0x4017, 0x08); //threshold= 2LSB for full size
	OV5670CMKFF_write_cmos_sensor(0x400a, 0x02); //
	OV5670CMKFF_write_cmos_sensor(0x400b, 0x00); //
	
	OV5670CMKFF_write_cmos_sensor(0x0100, 0x01); 
	
}


static void OV5670CMKFF_Sensor_Init(void)
{
	OV5670CMKFFDB("OV5670CMKFF_Sensor_Init_2lane enter\n");
	
	OV5670CMKFF_write_cmos_sensor(0x0103,0x01);// ; software reset
	mdelay(10);
	OV5670CMKFF_write_cmos_sensor(0x0100, 0x00);// ; software standby
	OV5670CMKFF_write_cmos_sensor(0x0100, 0x00); 
	OV5670CMKFF_write_cmos_sensor(0x0300, 0x04); 
	OV5670CMKFF_write_cmos_sensor(0x0301, 0x00); 
	OV5670CMKFF_write_cmos_sensor(0x0302, 0x64); //;78
	OV5670CMKFF_write_cmos_sensor(0x0303, 0x00); 
	OV5670CMKFF_write_cmos_sensor(0x0304, 0x03); 
	OV5670CMKFF_write_cmos_sensor(0x0305, 0x01); 
	OV5670CMKFF_write_cmos_sensor(0x0306, 0x01); 
	OV5670CMKFF_write_cmos_sensor(0x030a, 0x00); 
	OV5670CMKFF_write_cmos_sensor(0x030b, 0x00); 
	OV5670CMKFF_write_cmos_sensor(0x030c, 0x00); 
	OV5670CMKFF_write_cmos_sensor(0x030d, 0x1e); 
	OV5670CMKFF_write_cmos_sensor(0x030e, 0x00); 
	OV5670CMKFF_write_cmos_sensor(0x030f, 0x06); 
	OV5670CMKFF_write_cmos_sensor(0x0312, 0x01); 
	OV5670CMKFF_write_cmos_sensor(0x3000, 0x00); 
	OV5670CMKFF_write_cmos_sensor(0x3002, 0x21); 
	OV5670CMKFF_write_cmos_sensor(0x3005, 0xf0); 
	OV5670CMKFF_write_cmos_sensor(0x3007, 0x00); 
	OV5670CMKFF_write_cmos_sensor(0x3015, 0x0f); 
	OV5670CMKFF_write_cmos_sensor(0x3018, 0x32); 
	OV5670CMKFF_write_cmos_sensor(0x301a, 0xf0); 
	OV5670CMKFF_write_cmos_sensor(0x301b, 0xf0); 
	OV5670CMKFF_write_cmos_sensor(0x301c, 0xf0); 
	OV5670CMKFF_write_cmos_sensor(0x301d, 0xf0); 
	OV5670CMKFF_write_cmos_sensor(0x301e, 0xf0); 
	OV5670CMKFF_write_cmos_sensor(0x3030, 0x00); 
	OV5670CMKFF_write_cmos_sensor(0x3031, 0x0a); 
	OV5670CMKFF_write_cmos_sensor(0x303c, 0xff); 
	OV5670CMKFF_write_cmos_sensor(0x303e, 0xff); 
	OV5670CMKFF_write_cmos_sensor(0x3040, 0xf0); 
	OV5670CMKFF_write_cmos_sensor(0x3041, 0x00); 
	OV5670CMKFF_write_cmos_sensor(0x3042, 0xf0); 
	OV5670CMKFF_write_cmos_sensor(0x3106, 0x11); 
	OV5670CMKFF_write_cmos_sensor(0x3500, 0x00); 
	OV5670CMKFF_write_cmos_sensor(0x3501, 0x7b); 
	OV5670CMKFF_write_cmos_sensor(0x3502, 0x00); 
	OV5670CMKFF_write_cmos_sensor(0x3503, 0x04); 
	OV5670CMKFF_write_cmos_sensor(0x3504, 0x03); 
	OV5670CMKFF_write_cmos_sensor(0x3505, 0x83); 
	OV5670CMKFF_write_cmos_sensor(0x3508, 0x07); 
	OV5670CMKFF_write_cmos_sensor(0x3509, 0x80); 
	OV5670CMKFF_write_cmos_sensor(0x350e, 0x04); 
	OV5670CMKFF_write_cmos_sensor(0x350f, 0x00); 
	OV5670CMKFF_write_cmos_sensor(0x3510, 0x00); 
	OV5670CMKFF_write_cmos_sensor(0x3511, 0x02); 
	OV5670CMKFF_write_cmos_sensor(0x3512, 0x00); 
	OV5670CMKFF_write_cmos_sensor(0x3601, 0xc8); 
	OV5670CMKFF_write_cmos_sensor(0x3610, 0x88); 
	OV5670CMKFF_write_cmos_sensor(0x3612, 0x48); 
	OV5670CMKFF_write_cmos_sensor(0x3614, 0x5b); 
	OV5670CMKFF_write_cmos_sensor(0x3615, 0x96); 
	OV5670CMKFF_write_cmos_sensor(0x3621, 0xd0); 
	OV5670CMKFF_write_cmos_sensor(0x3622, 0x00); 
	OV5670CMKFF_write_cmos_sensor(0x3623, 0x00); 
	OV5670CMKFF_write_cmos_sensor(0x3633, 0x13); 
	OV5670CMKFF_write_cmos_sensor(0x3634, 0x13); 
	OV5670CMKFF_write_cmos_sensor(0x3635, 0x13); 
	OV5670CMKFF_write_cmos_sensor(0x3636, 0x13); 
	OV5670CMKFF_write_cmos_sensor(0x3645, 0x13); 
	OV5670CMKFF_write_cmos_sensor(0x3646, 0x82); 
	OV5670CMKFF_write_cmos_sensor(0x3650, 0x00); 
	OV5670CMKFF_write_cmos_sensor(0x3652, 0xff); 
	OV5670CMKFF_write_cmos_sensor(0x3655, 0x20); 
	OV5670CMKFF_write_cmos_sensor(0x3656, 0xff); 
	OV5670CMKFF_write_cmos_sensor(0x365a, 0xff); 
	OV5670CMKFF_write_cmos_sensor(0x365e, 0xff); 
	OV5670CMKFF_write_cmos_sensor(0x3668, 0x00); 
	OV5670CMKFF_write_cmos_sensor(0x366a, 0x07); 
	OV5670CMKFF_write_cmos_sensor(0x366e, 0x10); 
	OV5670CMKFF_write_cmos_sensor(0x366d, 0x00); 
	OV5670CMKFF_write_cmos_sensor(0x366f, 0x80); 
	OV5670CMKFF_write_cmos_sensor(0x3700, 0x28); 
	OV5670CMKFF_write_cmos_sensor(0x3701, 0x10); 
	OV5670CMKFF_write_cmos_sensor(0x3702, 0x3a); 
	OV5670CMKFF_write_cmos_sensor(0x3703, 0x19); 
	OV5670CMKFF_write_cmos_sensor(0x3704, 0x10);
	OV5670CMKFF_write_cmos_sensor(0x3705, 0x00); 
	OV5670CMKFF_write_cmos_sensor(0x3706, 0x66); 
	OV5670CMKFF_write_cmos_sensor(0x3707, 0x08); 
	OV5670CMKFF_write_cmos_sensor(0x3708, 0x34); 
	OV5670CMKFF_write_cmos_sensor(0x3709, 0x40); 
	OV5670CMKFF_write_cmos_sensor(0x370a, 0x01); 
	OV5670CMKFF_write_cmos_sensor(0x370b, 0x1b); 
	OV5670CMKFF_write_cmos_sensor(0x3714, 0x24); 
	OV5670CMKFF_write_cmos_sensor(0x371a, 0x3e); 
	OV5670CMKFF_write_cmos_sensor(0x3733, 0x00); 
	OV5670CMKFF_write_cmos_sensor(0x3734, 0x00); 
	OV5670CMKFF_write_cmos_sensor(0x373a, 0x05); 
	OV5670CMKFF_write_cmos_sensor(0x373b, 0x06); 
	OV5670CMKFF_write_cmos_sensor(0x373c, 0x0a); 
	OV5670CMKFF_write_cmos_sensor(0x373f, 0xa0); 
	OV5670CMKFF_write_cmos_sensor(0x3755, 0x00); 
	OV5670CMKFF_write_cmos_sensor(0x3758, 0x00); 
	OV5670CMKFF_write_cmos_sensor(0x375b, 0x0e);
	OV5670CMKFF_write_cmos_sensor(0x3766, 0x5f); 
	OV5670CMKFF_write_cmos_sensor(0x3768, 0x00); 
	OV5670CMKFF_write_cmos_sensor(0x3769, 0x22); 
	OV5670CMKFF_write_cmos_sensor(0x3773, 0x08); 
	OV5670CMKFF_write_cmos_sensor(0x3774, 0x1f); 
	OV5670CMKFF_write_cmos_sensor(0x3776, 0x06); 
	OV5670CMKFF_write_cmos_sensor(0x37a0, 0x88); 
	OV5670CMKFF_write_cmos_sensor(0x37a1, 0x5c); 
	OV5670CMKFF_write_cmos_sensor(0x37a7, 0x88); 
	OV5670CMKFF_write_cmos_sensor(0x37a8, 0x70); 
	OV5670CMKFF_write_cmos_sensor(0x37aa, 0x88); 
	OV5670CMKFF_write_cmos_sensor(0x37ab, 0x48); 
	OV5670CMKFF_write_cmos_sensor(0x37b3, 0x66); 
	OV5670CMKFF_write_cmos_sensor(0x37c2, 0x04); 
	OV5670CMKFF_write_cmos_sensor(0x37c5, 0x00); 
	OV5670CMKFF_write_cmos_sensor(0x37c8, 0x00); 
	OV5670CMKFF_write_cmos_sensor(0x3800, 0x00); 
	OV5670CMKFF_write_cmos_sensor(0x3801, 0x0c); 
	OV5670CMKFF_write_cmos_sensor(0x3802, 0x00); 
	OV5670CMKFF_write_cmos_sensor(0x3803, 0x04); 
	OV5670CMKFF_write_cmos_sensor(0x3804, 0x0a); 
	OV5670CMKFF_write_cmos_sensor(0x3805, 0x33); 
	OV5670CMKFF_write_cmos_sensor(0x3806, 0x07); 
	OV5670CMKFF_write_cmos_sensor(0x3807, 0xa3); 
	OV5670CMKFF_write_cmos_sensor(0x3808, 0x0a); 
	OV5670CMKFF_write_cmos_sensor(0x3809, 0x20); 
	OV5670CMKFF_write_cmos_sensor(0x380a, 0x07); 
	OV5670CMKFF_write_cmos_sensor(0x380b, 0x98); 
	OV5670CMKFF_write_cmos_sensor(0x380c, 0x07); // ;06
	OV5670CMKFF_write_cmos_sensor(0x380d, 0xdc); // ;8c
	OV5670CMKFF_write_cmos_sensor(0x380e, 0x07); 
	OV5670CMKFF_write_cmos_sensor(0x380f, 0xb8); 
	OV5670CMKFF_write_cmos_sensor(0x3811, 0x04); 
	OV5670CMKFF_write_cmos_sensor(0x3813, 0x02); 
	OV5670CMKFF_write_cmos_sensor(0x3814, 0x01); 
	OV5670CMKFF_write_cmos_sensor(0x3815, 0x01); 
	OV5670CMKFF_write_cmos_sensor(0x3816, 0x00); 
	OV5670CMKFF_write_cmos_sensor(0x3817, 0x00); 
	OV5670CMKFF_write_cmos_sensor(0x3818, 0x00); 
	OV5670CMKFF_write_cmos_sensor(0x3819, 0x00); 
	OV5670CMKFF_write_cmos_sensor(0x3820, 0x80); 
	OV5670CMKFF_write_cmos_sensor(0x3821, 0x46); 
	OV5670CMKFF_write_cmos_sensor(0x3822, 0x48); 
	OV5670CMKFF_write_cmos_sensor(0x3826, 0x00); 
	OV5670CMKFF_write_cmos_sensor(0x3827, 0x08); 
	OV5670CMKFF_write_cmos_sensor(0x382a, 0x01); 
	OV5670CMKFF_write_cmos_sensor(0x382b, 0x01); 
	OV5670CMKFF_write_cmos_sensor(0x3830, 0x08); 
	OV5670CMKFF_write_cmos_sensor(0x3836, 0x02); 
	OV5670CMKFF_write_cmos_sensor(0x3837, 0x00); 
	OV5670CMKFF_write_cmos_sensor(0x3838, 0x10); 
	OV5670CMKFF_write_cmos_sensor(0x3841, 0xff); 
	OV5670CMKFF_write_cmos_sensor(0x3846, 0x48); 
	OV5670CMKFF_write_cmos_sensor(0x3861, 0x00); 
	OV5670CMKFF_write_cmos_sensor(0x3862, 0x04);//x00); 
	OV5670CMKFF_write_cmos_sensor(0x3863, 0x06);//0x18); 
	OV5670CMKFF_write_cmos_sensor(0x3a11, 0x01); 
	OV5670CMKFF_write_cmos_sensor(0x3a12, 0x78); 
	OV5670CMKFF_write_cmos_sensor(0x3b00, 0x00); 
	OV5670CMKFF_write_cmos_sensor(0x3b02, 0x00); 
	OV5670CMKFF_write_cmos_sensor(0x3b03, 0x00); 
	OV5670CMKFF_write_cmos_sensor(0x3b04, 0x00); 
	OV5670CMKFF_write_cmos_sensor(0x3b05, 0x00); 
	OV5670CMKFF_write_cmos_sensor(0x3c00, 0x89); 
	OV5670CMKFF_write_cmos_sensor(0x3c01, 0xab); 
	OV5670CMKFF_write_cmos_sensor(0x3c02, 0x01); 
	OV5670CMKFF_write_cmos_sensor(0x3c03, 0x00); 
	OV5670CMKFF_write_cmos_sensor(0x3c04, 0x00); 
	OV5670CMKFF_write_cmos_sensor(0x3c05, 0x03); 
	OV5670CMKFF_write_cmos_sensor(0x3c06, 0x00); 
	OV5670CMKFF_write_cmos_sensor(0x3c07, 0x05); 
	OV5670CMKFF_write_cmos_sensor(0x3c0c, 0x00); 
	OV5670CMKFF_write_cmos_sensor(0x3c0d, 0x00); 
	OV5670CMKFF_write_cmos_sensor(0x3c0e, 0x00); 
	OV5670CMKFF_write_cmos_sensor(0x3c0f, 0x00); 
	OV5670CMKFF_write_cmos_sensor(0x3c40, 0x00); 
	OV5670CMKFF_write_cmos_sensor(0x3c41, 0xa3); 
	OV5670CMKFF_write_cmos_sensor(0x3c43, 0x7d); 
	OV5670CMKFF_write_cmos_sensor(0x3c45, 0xd7); 
	OV5670CMKFF_write_cmos_sensor(0x3c47, 0xfc); 
	OV5670CMKFF_write_cmos_sensor(0x3c50, 0x05); 
	OV5670CMKFF_write_cmos_sensor(0x3c52, 0xaa); 
	OV5670CMKFF_write_cmos_sensor(0x3c54, 0x71); 
	OV5670CMKFF_write_cmos_sensor(0x3c56, 0x80); 
	OV5670CMKFF_write_cmos_sensor(0x3d85, 0x17); 
	OV5670CMKFF_write_cmos_sensor(0x3f03, 0x00); 
	OV5670CMKFF_write_cmos_sensor(0x3f0a, 0x00); 
	OV5670CMKFF_write_cmos_sensor(0x3f0b, 0x00); 
	OV5670CMKFF_write_cmos_sensor(0x4001, 0x60); 
	OV5670CMKFF_write_cmos_sensor(0x4009, 0x0d); 
	OV5670CMKFF_write_cmos_sensor(0x4020, 0x00); 
	OV5670CMKFF_write_cmos_sensor(0x4021, 0x00); 
	OV5670CMKFF_write_cmos_sensor(0x4022, 0x00); 
	OV5670CMKFF_write_cmos_sensor(0x4023, 0x00); 
	OV5670CMKFF_write_cmos_sensor(0x4024, 0x00); 
	OV5670CMKFF_write_cmos_sensor(0x4025, 0x00); 
	OV5670CMKFF_write_cmos_sensor(0x4026, 0x00); 
	OV5670CMKFF_write_cmos_sensor(0x4027, 0x00); 
	OV5670CMKFF_write_cmos_sensor(0x4028, 0x00); 
	OV5670CMKFF_write_cmos_sensor(0x4029, 0x00); 
	OV5670CMKFF_write_cmos_sensor(0x402a, 0x00); 
	OV5670CMKFF_write_cmos_sensor(0x402b, 0x00); 
	OV5670CMKFF_write_cmos_sensor(0x402c, 0x00); 
	OV5670CMKFF_write_cmos_sensor(0x402d, 0x00); 
	OV5670CMKFF_write_cmos_sensor(0x402e, 0x00); 
	OV5670CMKFF_write_cmos_sensor(0x402f, 0x00); 
	OV5670CMKFF_write_cmos_sensor(0x4040, 0x00); 
	OV5670CMKFF_write_cmos_sensor(0x4041, 0x03);//0x00); 
	OV5670CMKFF_write_cmos_sensor(0x4042, 0x00); 
	OV5670CMKFF_write_cmos_sensor(0x4043, 0x7a);//0x80); 
	OV5670CMKFF_write_cmos_sensor(0x4044, 0x00); 
	OV5670CMKFF_write_cmos_sensor(0x4045, 0x7a);//0x80); 
	OV5670CMKFF_write_cmos_sensor(0x4046, 0x00); 
	OV5670CMKFF_write_cmos_sensor(0x4047, 0x7a);//0x80); 
	OV5670CMKFF_write_cmos_sensor(0x4048, 0x00); 
	OV5670CMKFF_write_cmos_sensor(0x4049, 0x7a);//0x80); 
	OV5670CMKFF_write_cmos_sensor(0x4303, 0x00); 
	OV5670CMKFF_write_cmos_sensor(0x4307, 0x30); 
	OV5670CMKFF_write_cmos_sensor(0x4500, 0x58); 
	OV5670CMKFF_write_cmos_sensor(0x4501, 0x04); 
	OV5670CMKFF_write_cmos_sensor(0x4502, 0x40); 
	OV5670CMKFF_write_cmos_sensor(0x4503, 0x10); 
	OV5670CMKFF_write_cmos_sensor(0x4508, 0xaa); 
	OV5670CMKFF_write_cmos_sensor(0x4509, 0xaa); 
	OV5670CMKFF_write_cmos_sensor(0x450a, 0x00); 
	OV5670CMKFF_write_cmos_sensor(0x450b, 0x00); 
	OV5670CMKFF_write_cmos_sensor(0x4600, 0x01); 
	OV5670CMKFF_write_cmos_sensor(0x4601, 0x03); 
	OV5670CMKFF_write_cmos_sensor(0x4700, 0xa4); 
	OV5670CMKFF_write_cmos_sensor(0x4800, 0x4c); 
	OV5670CMKFF_write_cmos_sensor(0x4816, 0x53); 
	OV5670CMKFF_write_cmos_sensor(0x481f, 0x40); 
	OV5670CMKFF_write_cmos_sensor(0x4837, 0x14); // ;11
	OV5670CMKFF_write_cmos_sensor(0x5000, 0x56);//0x16); 
	OV5670CMKFF_write_cmos_sensor(0x5001, 0x01); 
	OV5670CMKFF_write_cmos_sensor(0x5002, 0x28);//0xa8); 
	OV5670CMKFF_write_cmos_sensor(0x5004, 0x0c); 
	OV5670CMKFF_write_cmos_sensor(0x5006, 0x0c); 
	OV5670CMKFF_write_cmos_sensor(0x5007, 0xe0); 
	OV5670CMKFF_write_cmos_sensor(0x5008, 0x01); 
	OV5670CMKFF_write_cmos_sensor(0x5009, 0xb0); 
	OV5670CMKFF_write_cmos_sensor(0x5901, 0x00); 
	OV5670CMKFF_write_cmos_sensor(0x5a01, 0x00); 
	OV5670CMKFF_write_cmos_sensor(0x5a03, 0x00); 
	OV5670CMKFF_write_cmos_sensor(0x5a04, 0x0c); 
	OV5670CMKFF_write_cmos_sensor(0x5a05, 0xe0); 
	OV5670CMKFF_write_cmos_sensor(0x5a06, 0x09); 
	OV5670CMKFF_write_cmos_sensor(0x5a07, 0xb0); 
	OV5670CMKFF_write_cmos_sensor(0x5a08, 0x06); 
	OV5670CMKFF_write_cmos_sensor(0x5e00, 0x00); 
	//for BLC
	OV5670CMKFF_write_cmos_sensor(0x3734, 0x40); 
	OV5670CMKFF_write_cmos_sensor(0x5b00, 0x01); 
	OV5670CMKFF_write_cmos_sensor(0x5b01, 0x10); 
	OV5670CMKFF_write_cmos_sensor(0x5b02, 0x01); 
	OV5670CMKFF_write_cmos_sensor(0x5b03, 0xdb); 
	OV5670CMKFF_write_cmos_sensor(0x3d8c, 0x71);
	OV5670CMKFF_write_cmos_sensor(0x3d8d, 0xea);
	OV5670CMKFF_write_cmos_sensor(0x4017, 0x08);
	
	OV5670CMKFF_write_cmos_sensor(0x3618, 0x2a); 
								   
	//;Ally031414					  
	OV5670CMKFF_write_cmos_sensor(0x3734, 0x40); //	;; Improve HFPN
	OV5670CMKFF_write_cmos_sensor(0x5b00, 0x01);  // ;; [2:0] otp start addr[10:8]
	OV5670CMKFF_write_cmos_sensor(0x5b01, 0x10);  // ;; [7:0] otp start addr[7:0]
	OV5670CMKFF_write_cmos_sensor(0x5b02, 0x01);  // ;; [2:0] otp end addr[10:8]
	OV5670CMKFF_write_cmos_sensor(0x5b03, 0xDB);  // ;; [7:0] otp end addr[7:0]
	OV5670CMKFF_write_cmos_sensor(0x3d8c, 0x71); //; Header address high byte
	OV5670CMKFF_write_cmos_sensor(0x3d8d, 0xEA); //; Header address low byte
	OV5670CMKFF_write_cmos_sensor(0x4017, 0x08); // ; threshold= 2LSB for full size
								  
	//;Strong DPC1.53				 
	OV5670CMKFF_write_cmos_sensor(0x5780, 0x3e); 
	OV5670CMKFF_write_cmos_sensor(0x5781, 0x0f); 
	OV5670CMKFF_write_cmos_sensor(0x5782, 0x44); 
	OV5670CMKFF_write_cmos_sensor(0x5783, 0x02); 
	OV5670CMKFF_write_cmos_sensor(0x5784, 0x01); 
	OV5670CMKFF_write_cmos_sensor(0x5785, 0x00); 
	OV5670CMKFF_write_cmos_sensor(0x5786, 0x00); 
	OV5670CMKFF_write_cmos_sensor(0x5787, 0x04); 
	OV5670CMKFF_write_cmos_sensor(0x5788, 0x02); 
	OV5670CMKFF_write_cmos_sensor(0x5789, 0x0f); 
	OV5670CMKFF_write_cmos_sensor(0x578a, 0xfd); 
	OV5670CMKFF_write_cmos_sensor(0x578b, 0xf5); 
	OV5670CMKFF_write_cmos_sensor(0x578c, 0xf5); 
	OV5670CMKFF_write_cmos_sensor(0x578d, 0x03); 
	OV5670CMKFF_write_cmos_sensor(0x578e, 0x08); 
	OV5670CMKFF_write_cmos_sensor(0x578f, 0x0c); 
	OV5670CMKFF_write_cmos_sensor(0x5790, 0x08); 
	OV5670CMKFF_write_cmos_sensor(0x5791, 0x04); 
	OV5670CMKFF_write_cmos_sensor(0x5792, 0x00); 
	OV5670CMKFF_write_cmos_sensor(0x5793, 0x52); 
	OV5670CMKFF_write_cmos_sensor(0x5794, 0xa3); 
	//;Ping 					  
	OV5670CMKFF_write_cmos_sensor(0x380e, 0x07); //; fps fine adjustment
	OV5670CMKFF_write_cmos_sensor(0x380f, 0xfd); //; fps fine adjustment
	OV5670CMKFF_write_cmos_sensor(0x3503, 0x00); //; real gain [2]   gain no delay, shutter no delay
	//;added					 
	OV5670CMKFF_write_cmos_sensor(0x3d85, 0x17); 
	OV5670CMKFF_write_cmos_sensor(0x3655, 0x20); 
								   
	//OV5670CMKFF_write_cmos_sensor(0x0100, 0x00); //;01


}


UINT32 OV5670CMKFFOpen(void)
{

	volatile signed int i;
	kal_uint16 sensor_id = 0;
	int module_id;

	OV5670CMKFFDB("OV5670CMKFF Open enter :\n ");
	OV5670CMKFF_write_cmos_sensor(0x0103,0x01);// Reset sensor
    mdelay(2);

	for(i=0;i<2;i++)
	{
		sensor_id = (OV5670CMKFF_read_cmos_sensor(0x300B)<<8)|OV5670CMKFF_read_cmos_sensor(0x300C);
		OV5670CMKFFDB("OV5670CMKFF READ ID :%x",sensor_id);
		if(sensor_id != 0x5670/*OV5670CMKFF_SENSOR_ID*/)
		{
			return ERROR_SENSOR_CONNECT_FAIL;
		}else
			break;
	}
	
	OV5670CMKFF_Sensor_Init();
    OV5670CMKFF_Init_Para();
	
	OV5670CMKFFDB("OV5670CMKFFOpen exit :\n ");

    return ERROR_NONE;
}


UINT32 OV5670CMKFFGetSensorID(UINT32 *sensorID)
{
    int  retry = 2;
	int  module_id;
	OV5670CMKFFDB("OV5670CMKFFGetSensorID enter :\n ");
    mdelay(5);

    do {
        *sensorID = (OV5670CMKFF_read_cmos_sensor(0x300B)<<8)|OV5670CMKFF_read_cmos_sensor(0x300C);
        if (*sensorID == 0x5670/*OV5670CMKFF_SENSOR_ID*/)
        	{
        		OV5670CMKFFDB("Sensor ID = 0x%04x\n", *sensorID);
				
				module_id = read_module_integrator_id();
				OV5670CMKFFDB("Module ID = 0x%04x\n", module_id);
				
				if(module_id == 0x0dc8)
					*sensorID = OV5670CMKFF_SENSOR_ID;
            	break;
        	}
        OV5670CMKFFDB("Read Sensor ID Fail = 0x%04x\n", *sensorID);
        retry--;
    } while (retry > 0);

    if (*sensorID != OV5670CMKFF_SENSOR_ID) {
		OV5670CMKFFDB("Read Sensor ID Fail = 0x%04x\n", *sensorID);
		
		/*
        *sensorID = OV5670CMKFF_SENSOR_ID;
        return ERROR_SENSOR_CONNECT_FAIL;
	*/
	 *sensorID = 0xFFFFFFFF;
     return ERROR_SENSOR_CONNECT_FAIL;
    }
    return ERROR_NONE;
}


void OV5670CMKFF_SetShutter(kal_uint32 iShutter)
{

   spin_lock(&ov5670cmkffmipiraw_drv_lock);
   OV5670CMKFF.shutter= iShutter;
   spin_unlock(&ov5670cmkffmipiraw_drv_lock);

   OV5670CMKFF_write_shutter(iShutter);
   return;
}



UINT32 OV5670CMKFF_read_shutter(void)
{

	kal_uint16 temp_reg1, temp_reg2 ,temp_reg3;
	UINT32 shutter =0;
	temp_reg1 = OV5670CMKFF_read_cmos_sensor(0x3500);    // AEC[b19~b16]
	temp_reg2 = OV5670CMKFF_read_cmos_sensor(0x3501);    // AEC[b15~b8]
	temp_reg3 = OV5670CMKFF_read_cmos_sensor(0x3502);    // AEC[b7~b0]
	
	shutter  = (temp_reg1 <<12)| (temp_reg2<<4)|(temp_reg3>>4);

	return shutter;
}

void OV5670CMKFF_NightMode(kal_bool bEnable)
{

}

UINT32 OV5670CMKFFClose(void)
{

    return ERROR_NONE;
}

#if 0
void OV5670CMKFFSetFlipMirror(kal_int32 imgMirror)
{
	kal_int16 mirror=0,flip=0;
	mirror= OV5670CMKFF_read_cmos_sensor(0x3820);
	flip  = OV5670CMKFF_read_cmos_sensor(0x3821);

    switch (imgMirror)
    {
        case IMAGE_H_MIRROR://IMAGE_NORMAL:
            OV5670CMKFF_write_cmos_sensor(0x3820, (mirror & (0xF9)));//Set normal
            OV5670CMKFF_write_cmos_sensor(0x3821, (flip & (0xF9)));	//Set normal
            break;
        case IMAGE_NORMAL://IMAGE_V_MIRROR:
            OV5670CMKFF_write_cmos_sensor(0x3820, (mirror & (0xF9)));//Set flip
            OV5670CMKFF_write_cmos_sensor(0x3821, (flip | (0x06)));	//Set flip
            break;
        case IMAGE_HV_MIRROR://IMAGE_H_MIRROR:
            OV5670CMKFF_write_cmos_sensor(0x3820, (mirror |(0x06)));	//Set mirror
            OV5670CMKFF_write_cmos_sensor(0x3821, (flip & (0xF9)));	//Set mirror
            break;
        case IMAGE_V_MIRROR://IMAGE_HV_MIRROR:
            OV5670CMKFF_write_cmos_sensor(0x3820, (mirror |(0x06)));	//Set mirror & flip
            OV5670CMKFF_write_cmos_sensor(0x3821, (flip |(0x06)));	//Set mirror & flip
            break;
    }
}
#endif


UINT32 OV5670CMKFFPreview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                                                MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{

	OV5670CMKFFDB("OV5670CMKFFPreview enter:");

	OV5670CMKFFPreviewSetting();

	spin_lock(&ov5670cmkffmipiraw_drv_lock);
	OV5670CMKFF.sensorMode = SENSOR_MODE_PREVIEW; 
	OV5670CMKFF.DummyPixels = 0;
	OV5670CMKFF.DummyLines = 0 ;
	OV5670CMKFF_FeatureControl_PERIOD_PixelNum=OV5670CMKFF_PV_PERIOD_PIXEL_NUMS+ OV5670CMKFF.DummyPixels;
	OV5670CMKFF_FeatureControl_PERIOD_LineNum=OV5670CMKFF_PV_PERIOD_LINE_NUMS+OV5670CMKFF.DummyLines;
	OV5670CMKFF.imgMirror = sensor_config_data->SensorImageMirror;
	spin_unlock(&ov5670cmkffmipiraw_drv_lock);
	
	//OV5670CMKFFSetFlipMirror(sensor_config_data->SensorImageMirror);

    mdelay(40);//THIS DELAY SHOULD BE NEED BY CTS OR MONKEY
	OV5670CMKFFDB("OV5670CMKFFPreview exit:\n");

	  
    return ERROR_NONE;
}


UINT32 OV5670CMKFFVideo(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                                                MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{

	OV5670CMKFFDB("OV5670CMKFFVideo enter:");

	OV5670CMKFFVideoSetting();

	spin_lock(&ov5670cmkffmipiraw_drv_lock);
	OV5670CMKFF.sensorMode = SENSOR_MODE_VIDEO;
	OV5670CMKFF_FeatureControl_PERIOD_PixelNum=OV5670CMKFF_VIDEO_PERIOD_PIXEL_NUMS+ OV5670CMKFF.DummyPixels;
	OV5670CMKFF_FeatureControl_PERIOD_LineNum=OV5670CMKFF_VIDEO_PERIOD_LINE_NUMS+OV5670CMKFF.DummyLines;
	OV5670CMKFF.imgMirror = sensor_config_data->SensorImageMirror;
	spin_unlock(&ov5670cmkffmipiraw_drv_lock);
	
	//OV5670CMKFFSetFlipMirror(sensor_config_data->SensorImageMirror);

    mdelay(40);//THIS DELAY SHOULD BE NEED BY CTS OR MONKEY
	OV5670CMKFFDB("OV5670CMKFFVideo exit:\n");
    return ERROR_NONE;
}


UINT32 OV5670CMKFFCapture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                                                MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{

 	//kal_uint32 shutter = OV5670CMKFF.shutter;

	if( SENSOR_MODE_CAPTURE== OV5670CMKFF.sensorMode)
	{
		OV5670CMKFFDB("OV5670CMKFFCapture BusrtShot / ZSD!!!\n");
	}
	else
	{
		OV5670CMKFFDB("OV5670CMKFFCapture enter:\n");

		OV5670CMKFFCaptureSetting();
	    mdelay(40);//THIS DELAY SHOULD BE NEED BY CTS OR MONKEY

		spin_lock(&ov5670cmkffmipiraw_drv_lock);
		OV5670CMKFF.sensorMode = SENSOR_MODE_CAPTURE;
		OV5670CMKFF.imgMirror = sensor_config_data->SensorImageMirror;
		OV5670CMKFF.DummyPixels = 0;
		OV5670CMKFF.DummyLines = 0 ;
		OV5670CMKFF_FeatureControl_PERIOD_PixelNum = OV5670CMKFF_FULL_PERIOD_PIXEL_NUMS + OV5670CMKFF.DummyPixels;
		OV5670CMKFF_FeatureControl_PERIOD_LineNum = OV5670CMKFF_FULL_PERIOD_LINE_NUMS + OV5670CMKFF.DummyLines;
		spin_unlock(&ov5670cmkffmipiraw_drv_lock);

		//OV5670CMKFFSetFlipMirror(sensor_config_data->SensorImageMirror);

		OV5670CMKFFDB("OV5670CMKFFCapture exit:\n");
	}

	if(OV5670CMKFF_During_testpattern == KAL_TRUE)
	{
		OV5670CMKFF_write_cmos_sensor(0x4303,0x80);
	}

    return ERROR_NONE;
}	



UINT32 OV5670CMKFFGetResolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *pSensorResolution)
{

    OV5670CMKFFDB("OV5670CMKFFGetResolution!!\n");

	pSensorResolution->SensorPreviewWidth	= OV5670CMKFF_IMAGE_SENSOR_PV_WIDTH;
    pSensorResolution->SensorPreviewHeight	= OV5670CMKFF_IMAGE_SENSOR_PV_HEIGHT;
	
    pSensorResolution->SensorFullWidth		= OV5670CMKFF_IMAGE_SENSOR_FULL_WIDTH;
    pSensorResolution->SensorFullHeight		= OV5670CMKFF_IMAGE_SENSOR_FULL_HEIGHT;
	
    pSensorResolution->SensorVideoWidth		= OV5670CMKFF_IMAGE_SENSOR_VIDEO_WIDTH;
    pSensorResolution->SensorVideoHeight    = OV5670CMKFF_IMAGE_SENSOR_VIDEO_HEIGHT;
    return ERROR_NONE;
}   

UINT32 OV5670CMKFFGetInfo(MSDK_SCENARIO_ID_ENUM ScenarioId,
                                                MSDK_SENSOR_INFO_STRUCT *pSensorInfo,
                                                MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{

	spin_lock(&ov5670cmkffmipiraw_drv_lock);
	OV5670CMKFF.imgMirror = pSensorConfigData->SensorImageMirror ;
	spin_unlock(&ov5670cmkffmipiraw_drv_lock);

    pSensorInfo->SensorOutputDataFormat= SENSOR_OUTPUT_FORMAT_RAW_B;
   
    pSensorInfo->SensorClockPolarity =SENSOR_CLOCK_POLARITY_LOW;
    pSensorInfo->SensorClockFallingPolarity=SENSOR_CLOCK_POLARITY_LOW;
    pSensorInfo->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
    pSensorInfo->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;

    pSensorInfo->SensroInterfaceType=SENSOR_INTERFACE_TYPE_MIPI;

    pSensorInfo->CaptureDelayFrame = 2;
    pSensorInfo->PreviewDelayFrame = 2;
    pSensorInfo->VideoDelayFrame = 2;

    pSensorInfo->SensorDrivingCurrent = ISP_DRIVING_8MA;
    pSensorInfo->AEShutDelayFrame = 0;	    
    pSensorInfo->AESensorGainDelayFrame = 0;
    pSensorInfo->AEISPGainDelayFrame = 2;

    switch (ScenarioId)
    {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
            pSensorInfo->SensorClockFreq=24;
            pSensorInfo->SensorClockRisingCount= 0;

            pSensorInfo->SensorGrabStartX = OV5670CMKFF_PV_X_START;
            pSensorInfo->SensorGrabStartY = OV5670CMKFF_PV_Y_START;

            pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_2_LANE;
			
            pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
	     	pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 14;
	    	pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
            pSensorInfo->SensorPacketECCOrder = 1;
            break;
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
            pSensorInfo->SensorClockFreq=24;
            pSensorInfo->SensorClockRisingCount= 0;

            pSensorInfo->SensorGrabStartX = OV5670CMKFF_VIDEO_X_START;
            pSensorInfo->SensorGrabStartY = OV5670CMKFF_VIDEO_Y_START;

            pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_2_LANE;
			
            pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
	     	pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 14;
	    	pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
            pSensorInfo->SensorPacketECCOrder = 1;
            break;
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
            pSensorInfo->SensorClockFreq=24;
            pSensorInfo->SensorClockRisingCount= 0;

            pSensorInfo->SensorGrabStartX = OV5670CMKFF_FULL_X_START;	
            pSensorInfo->SensorGrabStartY = OV5670CMKFF_FULL_Y_START;	

            pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_2_LANE;
			
            pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
            pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 14;
            pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
            pSensorInfo->SensorPacketECCOrder = 1;
            break;
        default:
			pSensorInfo->SensorClockFreq=24;
            pSensorInfo->SensorClockRisingCount= 0;

            pSensorInfo->SensorGrabStartX = OV5670CMKFF_PV_X_START;
            pSensorInfo->SensorGrabStartY = OV5670CMKFF_PV_Y_START;

            pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_2_LANE;
			
            pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
	     	pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 14;
	    	pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
            pSensorInfo->SensorPacketECCOrder = 1;
            break;
    }

    memcpy(pSensorConfigData, &OV5670CMKFFSensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));

    return ERROR_NONE;
}   /* OV5670CMKFFGetInfo() */



UINT32 OV5670CMKFFControl(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *pImageWindow,
                                                MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
		spin_lock(&ov5670cmkffmipiraw_drv_lock);
		OV5670CMKFFCurrentScenarioId = ScenarioId;
		spin_unlock(&ov5670cmkffmipiraw_drv_lock);
		
		OV5670CMKFFDB("OV5670CMKFFCurrentScenarioId=%d\n",OV5670CMKFFCurrentScenarioId);

	switch (ScenarioId)
    {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
            OV5670CMKFFPreview(pImageWindow, pSensorConfigData);
            break;
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			OV5670CMKFFDB("OV5670CMKFF video_preiew sync\n");
			OV5670CMKFFVideo(pImageWindow, pSensorConfigData);
			break;
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
            OV5670CMKFFCapture(pImageWindow, pSensorConfigData);
            break;

        default:
            return ERROR_INVALID_SCENARIO_ID;

    }
    return ERROR_NONE;
} /* OV5670CMKFFControl() */



kal_uint32 OV5670CMKFF_SET_FrameLength_ByVideoMode(UINT16 Video_TargetFps)
{
    UINT32 frameRate = 0;
	kal_uint32 MIN_FrameLength=0;
	
	if(OV5670CMKFF.OV5670CMKFFAutoFlickerMode == KAL_TRUE)
	{
		if (Video_TargetFps==30)
			frameRate= OV5670CMKFF_AUTOFLICKER_OFFSET_30;
		else if(Video_TargetFps==15)
			frameRate= OV5670CMKFF_AUTOFLICKER_OFFSET_15;
		else
			frameRate=Video_TargetFps*10;
	
		MIN_FrameLength = (OV5670CMKFF.videoPclk*10000)/(OV5670CMKFF_VIDEO_PERIOD_PIXEL_NUMS + OV5670CMKFF.DummyPixels)/frameRate*10;
	}
	else
		MIN_FrameLength = (OV5670CMKFF.videoPclk*10000) /(OV5670CMKFF_VIDEO_PERIOD_PIXEL_NUMS + OV5670CMKFF.DummyPixels)/Video_TargetFps;

     return MIN_FrameLength;

}



UINT32 OV5670CMKFFSetVideoMode(UINT16 u2FrameRate)
{

    kal_uint32 MIN_Frame_length =0,frameRate=0,extralines=0;
    OV5670CMKFFDB("[OV5670CMKFFSetVideoMode] frame rate = %d\n", u2FrameRate);

	spin_lock(&ov5670cmkffmipiraw_drv_lock);
	OV5670CMKFF_VIDEO_MODE_TARGET_FPS=u2FrameRate;
	spin_unlock(&ov5670cmkffmipiraw_drv_lock);

	if(u2FrameRate==0)
	{
		OV5670CMKFFDB("Disable Video Mode or dynimac fps\n");
		return KAL_TRUE;
	}
	if(u2FrameRate >30 || u2FrameRate <5)
	    OV5670CMKFFDB("abmornal frame rate seting,pay attention~\n");

    if(OV5670CMKFF.sensorMode == SENSOR_MODE_VIDEO)//video ScenarioId recording
    {

        MIN_Frame_length = OV5670CMKFF_SET_FrameLength_ByVideoMode(u2FrameRate);

		if((MIN_Frame_length <=OV5670CMKFF_VIDEO_PERIOD_LINE_NUMS))
		{
			MIN_Frame_length = OV5670CMKFF_VIDEO_PERIOD_LINE_NUMS;
			OV5670CMKFFDB("[OV5670CMKFFSetVideoMode]current fps = %d\n", (OV5670CMKFF.videoPclk*10000)  /(OV5670CMKFF_VIDEO_PERIOD_PIXEL_NUMS)/OV5670CMKFF_VIDEO_PERIOD_LINE_NUMS);
		}
		OV5670CMKFFDB("[OV5670CMKFFSetVideoMode]current fps (10 base)= %d\n", (OV5670CMKFF.videoPclk*10000)*10/(OV5670CMKFF_VIDEO_PERIOD_PIXEL_NUMS + OV5670CMKFF.DummyPixels)/MIN_Frame_length);
		extralines = MIN_Frame_length - OV5670CMKFF_VIDEO_PERIOD_LINE_NUMS;
		
		spin_lock(&ov5670cmkffmipiraw_drv_lock);
		OV5670CMKFF.DummyPixels = 0;//define dummy pixels and lines
		OV5670CMKFF.DummyLines = extralines ;
		spin_unlock(&ov5670cmkffmipiraw_drv_lock);
		
		OV5670CMKFF_SetDummy(OV5670CMKFF.DummyPixels,extralines);
    }
	
	OV5670CMKFFDB("[OV5670CMKFFSetVideoMode]MIN_Frame_length=%d,OV5670CMKFF.DummyLines=%d\n",MIN_Frame_length,OV5670CMKFF.DummyLines);

    return KAL_TRUE;
}


UINT32 OV5670CMKFFSetAutoFlickerMode(kal_bool bEnable, UINT16 u2FrameRate)
{

	if(bEnable) {   
		spin_lock(&ov5670cmkffmipiraw_drv_lock);
		OV5670CMKFF.OV5670CMKFFAutoFlickerMode = KAL_TRUE;
		spin_unlock(&ov5670cmkffmipiraw_drv_lock);
        OV5670CMKFFDB("OV5670CMKFF Enable Auto flicker\n");
    } else {
    	spin_lock(&ov5670cmkffmipiraw_drv_lock);
        OV5670CMKFF.OV5670CMKFFAutoFlickerMode = KAL_FALSE;
		spin_unlock(&ov5670cmkffmipiraw_drv_lock);
        OV5670CMKFFDB("OV5670CMKFF Disable Auto flicker\n");
    }

    return ERROR_NONE;
}


UINT32 OV5670CMKFFSetTestPatternMode(kal_bool bEnable)
{
    OV5670CMKFFDB("[OV5670CMKFFSetTestPatternMode] Test pattern enable:%d\n", bEnable);
    if(bEnable == KAL_TRUE)
    {
        OV5670CMKFF_During_testpattern = KAL_TRUE;

		//OV5670CMKFF_write_cmos_sensor(0x5000,0x16);// ; LENC off, MWB on, BPC on, WPC on
		
		OV5670CMKFF_write_cmos_sensor(0x4303,0x08);
    }
	else
	{
        OV5670CMKFF_During_testpattern = KAL_FALSE;
		//OV5670CMKFF_write_cmos_sensor(0x5000,0x96);// ; LENC on, MWB on, BPC on, WPC on
		OV5670CMKFF_write_cmos_sensor(0x4303,0x00);
	}

    return ERROR_NONE;
}


/*************************************************************************
*
* DESCRIPTION:
* INTERFACE FUNCTION, FOR USER TO SET MAX  FRAMERATE;
* 
*************************************************************************/
UINT32 OV5670CMKFFMIPISetMaxFramerateByScenario(MSDK_SCENARIO_ID_ENUM scenarioId, MUINT32 frameRate) {
	kal_uint32 pclk;
	kal_int16 dummyLine;
	kal_uint16 lineLength,frameHeight;
		
	OV5670CMKFFDB("OV5670CMKFFMIPISetMaxFramerateByScenario: scenarioId = %d, frame rate = %d\n",scenarioId,frameRate);
	switch (scenarioId) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			pclk = OV5670CMKFF_PREVIEW_PCLK;
			lineLength = OV5670CMKFF_PV_PERIOD_PIXEL_NUMS;
			frameHeight = (10 * pclk)/frameRate/lineLength;
			dummyLine = frameHeight - OV5670CMKFF_PV_PERIOD_LINE_NUMS;
			if(dummyLine<0)
				dummyLine = 0;
			spin_lock(&ov5670cmkffmipiraw_drv_lock);
			OV5670CMKFF.sensorMode = SENSOR_MODE_PREVIEW;
			spin_unlock(&ov5670cmkffmipiraw_drv_lock);
			OV5670CMKFF_SetDummy(0, dummyLine);			
			break;			
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			pclk = OV5670CMKFF_VIDEO_PCLK;
			lineLength = OV5670CMKFF_VIDEO_PERIOD_PIXEL_NUMS;
			frameHeight = (10 * pclk)/frameRate/lineLength;
			dummyLine = frameHeight - OV5670CMKFF_VIDEO_PERIOD_LINE_NUMS;
			if(dummyLine<0)
				dummyLine = 0;
			spin_lock(&ov5670cmkffmipiraw_drv_lock);
			OV5670CMKFF.sensorMode = SENSOR_MODE_VIDEO;
			spin_unlock(&ov5670cmkffmipiraw_drv_lock);
			OV5670CMKFF_SetDummy(0, dummyLine);			
			break;			
			 break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_CAMERA_ZSD:			
			pclk = OV5670CMKFF_CAPTURE_PCLK;
			lineLength = OV5670CMKFF_FULL_PERIOD_PIXEL_NUMS;
			frameHeight = (10 * pclk)/frameRate/lineLength;
			dummyLine = frameHeight - OV5670CMKFF_FULL_PERIOD_LINE_NUMS;
			if(dummyLine<0)
				dummyLine = 0;
			spin_lock(&ov5670cmkffmipiraw_drv_lock);
			OV5670CMKFF.sensorMode = SENSOR_MODE_CAPTURE;
			spin_unlock(&ov5670cmkffmipiraw_drv_lock);
			OV5670CMKFF_SetDummy(0, dummyLine);			
			break;		
        case MSDK_SCENARIO_ID_CAMERA_3D_PREVIEW:
            break;
        case MSDK_SCENARIO_ID_CAMERA_3D_VIDEO:
			break;
        case MSDK_SCENARIO_ID_CAMERA_3D_CAPTURE:   
			break;		
		default:
			break;
	}	
	return ERROR_NONE;
}


UINT32 OV5670CMKFFMIPIGetDefaultFramerateByScenario(MSDK_SCENARIO_ID_ENUM scenarioId, MUINT32 *pframeRate) 
{

	switch (scenarioId) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			 *pframeRate = OV5670CMKFF_MAX_FPS_PREVIEW;
			 break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
			 *pframeRate = OV5670CMKFF_MAX_FPS_CAPTURE;
			break;		
        case MSDK_SCENARIO_ID_CAMERA_3D_PREVIEW: //added
        case MSDK_SCENARIO_ID_CAMERA_3D_VIDEO:
        case MSDK_SCENARIO_ID_CAMERA_3D_CAPTURE: //added   
			 *pframeRate = OV5670CMKFF_MAX_FPS_CAPTURE;
			break;		
		default:
			break;
	}

	return ERROR_NONE;
}


UINT32 OV5670CMKFFFeatureControl(MSDK_SENSOR_FEATURE_ENUM FeatureId,
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
            *pFeatureReturnPara16++= OV5670CMKFF_IMAGE_SENSOR_FULL_WIDTH;
            *pFeatureReturnPara16= OV5670CMKFF_IMAGE_SENSOR_FULL_HEIGHT;
            *pFeatureParaLen=4;
            break;
        case SENSOR_FEATURE_GET_PERIOD:
				*pFeatureReturnPara16++= OV5670CMKFF_FeatureControl_PERIOD_PixelNum;
				*pFeatureReturnPara16= OV5670CMKFF_FeatureControl_PERIOD_LineNum;
				*pFeatureParaLen=4;
				break;
        case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
			switch(OV5670CMKFFCurrentScenarioId)
			{
				case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
					*pFeatureReturnPara32 = OV5670CMKFF_PREVIEW_PCLK;
					*pFeatureParaLen=4;
					break;
				case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
					*pFeatureReturnPara32 = OV5670CMKFF_VIDEO_PCLK;
					*pFeatureParaLen=4;
					break;
				case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
				case MSDK_SCENARIO_ID_CAMERA_ZSD:
					*pFeatureReturnPara32 = OV5670CMKFF_CAPTURE_PCLK;
					*pFeatureParaLen=4;
					break;
				default:
					*pFeatureReturnPara32 = OV5670CMKFF_PREVIEW_PCLK;
					*pFeatureParaLen=4;
					break;
			}
		    break;

        case SENSOR_FEATURE_SET_ESHUTTER:
            OV5670CMKFF_SetShutter(*pFeatureData16);
            break;
        case SENSOR_FEATURE_SET_NIGHTMODE:
            OV5670CMKFF_NightMode((BOOL) *pFeatureData16);
            break;
        case SENSOR_FEATURE_SET_GAIN:  
           OV5670CMKFF_SetGain((UINT16) *pFeatureData16);
            break;
        case SENSOR_FEATURE_SET_FLASHLIGHT:
            break;
        case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
            //OV5670CMKFF_isp_master_clock=*pFeatureData32;
            break;
        case SENSOR_FEATURE_SET_REGISTER:
            OV5670CMKFF_write_cmos_sensor(pSensorRegData->RegAddr, pSensorRegData->RegData);
            break;
        case SENSOR_FEATURE_GET_REGISTER:
            pSensorRegData->RegData = OV5670CMKFF_read_cmos_sensor(pSensorRegData->RegAddr);
            break;
        case SENSOR_FEATURE_SET_CCT_REGISTER:
            SensorRegNumber=FACTORY_END_ADDR;
            for (i=0;i<SensorRegNumber;i++)
            {
            	spin_lock(&ov5670cmkffmipiraw_drv_lock);
                OV5670CMKFFSensorCCT[i].Addr=*pFeatureData32++;
                OV5670CMKFFSensorCCT[i].Para=*pFeatureData32++;
				spin_unlock(&ov5670cmkffmipiraw_drv_lock);
            }
            break;
        case SENSOR_FEATURE_GET_CCT_REGISTER:
            SensorRegNumber=FACTORY_END_ADDR;
            if (*pFeatureParaLen<(SensorRegNumber*sizeof(SENSOR_REG_STRUCT)+4))
                return FALSE;
            *pFeatureData32++=SensorRegNumber;
            for (i=0;i<SensorRegNumber;i++)
            {
                *pFeatureData32++=OV5670CMKFFSensorCCT[i].Addr;
                *pFeatureData32++=OV5670CMKFFSensorCCT[i].Para;
            }
            break;
        case SENSOR_FEATURE_SET_ENG_REGISTER:
            SensorRegNumber=ENGINEER_END;
            for (i=0;i<SensorRegNumber;i++)
            {
            	spin_lock(&ov5670cmkffmipiraw_drv_lock);
                OV5670CMKFFSensorReg[i].Addr=*pFeatureData32++;
                OV5670CMKFFSensorReg[i].Para=*pFeatureData32++;
				spin_unlock(&ov5670cmkffmipiraw_drv_lock);
            }
            break;
        case SENSOR_FEATURE_GET_ENG_REGISTER:
            SensorRegNumber=ENGINEER_END;
            if (*pFeatureParaLen<(SensorRegNumber*sizeof(SENSOR_REG_STRUCT)+4))
                return FALSE;
            *pFeatureData32++=SensorRegNumber;
            for (i=0;i<SensorRegNumber;i++)
            {
                *pFeatureData32++=OV5670CMKFFSensorReg[i].Addr;
                *pFeatureData32++=OV5670CMKFFSensorReg[i].Para;
            }
            break;
        case SENSOR_FEATURE_GET_REGISTER_DEFAULT:
            if (*pFeatureParaLen>=sizeof(NVRAM_SENSOR_DATA_STRUCT))
            {
                pSensorDefaultData->Version=NVRAM_CAMERA_SENSOR_FILE_VERSION;
                pSensorDefaultData->SensorId=OV5670CMKFF_SENSOR_ID;
                memcpy(pSensorDefaultData->SensorEngReg, OV5670CMKFFSensorReg, sizeof(SENSOR_REG_STRUCT)*ENGINEER_END);
                memcpy(pSensorDefaultData->SensorCCTReg, OV5670CMKFFSensorCCT, sizeof(SENSOR_REG_STRUCT)*FACTORY_END_ADDR);
            }
            else
                return FALSE;
            *pFeatureParaLen=sizeof(NVRAM_SENSOR_DATA_STRUCT);
            break;
        case SENSOR_FEATURE_GET_CONFIG_PARA:
            memcpy(pSensorConfigData, &OV5670CMKFFSensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
            *pFeatureParaLen=sizeof(MSDK_SENSOR_CONFIG_STRUCT);
            break;
        case SENSOR_FEATURE_CAMERA_PARA_TO_SENSOR:
            OV5670CMKFF_camera_para_to_sensor();
            break;

        case SENSOR_FEATURE_SENSOR_TO_CAMERA_PARA:
            OV5670CMKFF_sensor_to_camera_para();
            break;
        case SENSOR_FEATURE_GET_GROUP_COUNT:
            *pFeatureReturnPara32++=OV5670CMKFF_get_sensor_group_count();
            *pFeatureParaLen=4;
            break;
        case SENSOR_FEATURE_GET_GROUP_INFO:
            OV5670CMKFF_get_sensor_group_info(pSensorGroupInfo->GroupIdx, pSensorGroupInfo->GroupNamePtr, &pSensorGroupInfo->ItemCount);
            *pFeatureParaLen=sizeof(MSDK_SENSOR_GROUP_INFO_STRUCT);
            break;
        case SENSOR_FEATURE_GET_ITEM_INFO:
            OV5670CMKFF_get_sensor_item_info(pSensorItemInfo->GroupIdx,pSensorItemInfo->ItemIdx, pSensorItemInfo);
            *pFeatureParaLen=sizeof(MSDK_SENSOR_ITEM_INFO_STRUCT);
            break;

        case SENSOR_FEATURE_SET_ITEM_INFO:
            OV5670CMKFF_set_sensor_item_info(pSensorItemInfo->GroupIdx, pSensorItemInfo->ItemIdx, pSensorItemInfo->ItemValue);
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
            OV5670CMKFFSetVideoMode(*pFeatureData16);
            break;
        case SENSOR_FEATURE_CHECK_SENSOR_ID:
            OV5670CMKFFGetSensorID(pFeatureReturnPara32);
            break;
        case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE:
            OV5670CMKFFSetAutoFlickerMode((BOOL)*pFeatureData16, *(pFeatureData16+1));
	        break;
		case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
			OV5670CMKFFMIPISetMaxFramerateByScenario((MSDK_SCENARIO_ID_ENUM)*pFeatureData32, *(pFeatureData32+1));
			break;
		case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
			OV5670CMKFFMIPIGetDefaultFramerateByScenario((MSDK_SCENARIO_ID_ENUM)*pFeatureData32, (MUINT32 *)(*(pFeatureData32+1)));
			break;
		case SENSOR_FEATURE_SET_TEST_PATTERN:
			OV5670CMKFFSetTestPatternMode((BOOL)*pFeatureData16);
			break;
		case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE://for factory mode auto testing 			
			*pFeatureReturnPara32=OV5670CMKFF_TEST_PATTERN_CHECKSUM; 		  
			*pFeatureParaLen=4; 							
		     break;
        default:
            break;
    }
    return ERROR_NONE;
}	


SENSOR_FUNCTION_STRUCT	SensorFuncOV5670CMKFF=
{
    OV5670CMKFFOpen,
    OV5670CMKFFGetInfo,
    OV5670CMKFFGetResolution,
    OV5670CMKFFFeatureControl,
    OV5670CMKFFControl,
    OV5670CMKFFClose
};

UINT32 OV5670CMKFF_MIPI_RAW_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
    if (pfFunc!=NULL)
        *pfFunc=&SensorFuncOV5670CMKFF;

    return ERROR_NONE;
}  

