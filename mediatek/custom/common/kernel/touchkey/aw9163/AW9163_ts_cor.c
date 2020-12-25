/**************************************************************************
 *  AW9163_ts_cor.c
 *
 *  Create Date :
 *
 *  Modify Date :
 *
 *  Create by   : AWINIC Technology CO., LTD
 *
 *  Version     : 1.0 , 2013/11/15
 *
 *
 **************************************************************************/
//////////////////////////////////////////////////////////////


#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/gpio.h>
#include <linux/earlysuspend.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/firmware.h>
#include <linux/platform_device.h>

#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <linux/init.h>
#include <linux/pci.h>
#include <linux/dma-mapping.h>
#include <linux/gameport.h>
#include <linux/moduleparam.h>
#include <linux/mutex.h>

#include <mach/mt_gpio.h>
#include "AW9163_reg.h"
#include "cust_gpio_usage.h"
#include <cust_eint.h>
#include <mach/mt_gpt.h>
#include <linux/wakelock.h>

#include <mach/mt_typedefs.h>
#include <mach/mt_pm_ldo.h>
#include <mach/mt_boot.h>
#include <mach/eint.h>
#include <linux/kpd.h>
#include <aw9163_cust.h>

static int suspended = 0;
extern bool GetHeadPhoneState(void);
extern int cust_get_touchkey_hw(void);
#define TS_DEBUG_MSG 			0
//////////////////////////////////////
// IO PIN DEFINE
//////////////////////////////////////
#if defined(DCT_S15)
#define AW9163_PDN          (GPIO6|0x80000000)
#define AW9163_EINT_NUM     5//15
#define AW9163_EINT_PIN     (GPIO5|0x80000000)//15
#elif   (defined DCT_S316)
#define AW9163_PDN          (GPIO55|0x80000000)
#define AW9163_EINT_NUM     106//15
#define AW9163_EINT_PIN     (GPIO106|0x80000000)//15
#else
#define AW9163_PDN          (GPIO83|0x80000000)
#define AW9163_EINT_NUM     82//15
#define AW9163_EINT_PIN     (GPIO82|0x80000000)//15
#endif
#define AW9163_ts_NAME	   	       	"AW9163_ts"
#define AW9163_ts_I2C_ADDR		     0x2C
//#define AW9163_ts_I2C_BUS		     2

#define AW9163
#define AW9163_EINT_SUPPORT   // use interrupt mode
//#define WATER_PROOF
//#define RD_RAW

#define ABS(x,y)         ((x>y)?(x-y):(y-x))


//////////////////////////////////////
//
// TOUCH KEY PARAMETER
//
//////////////////////////////////////
#define MV_TAIL_DBS

#define INTERPOLATION_X	     (AW9163_LCM_W/2)  // interpolation X-coordinate
#define INTERPOLATION_Y	     (AW9163_LCM_H/2)  // interpolation Y-coordinate

#define TOUCH_SET_TH	     80    // touch on judge
#define TOUCH_CLR_TH	     80    // touch off judge
#define TOUCH_HIGH_TH	     1000 // legal touch judge, high limit of sum 3 keys touch-delta
#define KEY_HIGH_TH				 500  //legal touch judge, each key touch-delta
#define BIG_TH						 TOUCH_SET_TH-15  //legal touch judge, key touch-delta max-min

#define PRESS_DEBOUNCE_TH    2   // press debounce count threshold
#define RELEASE_DEBOUNCE_TH  1   // release debounce count threshold
#define REV_DEBOUNCE_TH    5   // Reverse Release debounce count threshold

#define SAT_VAL		         150  // count saturation value
#define ON_EFF		         1   // low limit of single click on_cnt range
#define ON_MAX		         50  // high limit of single click on_cnt range
#define OFF_MAX		         20   // single click touch off count threshold
#define LONG_PRESS_TIME_TH   100  // long touch press count threshold

#define FRAME_RATE		     50  // polling touch-delta rate

#define UP_TRACE_STEP        1
#define DOWN_TRACE_STEP      1
#define UP_TRACE_TH					 5
#define DOWN_TRACE_TH        -5
#define NOISE_TH             5

#define MV_START_DIS         45
#define MV_STOP_DIS          45
#define MAX_MOVING           600

#define LONG_TOUCH_TIME		600 
#define ILLEGAL_TOUCH_CNT	8

#define KEY_DEBOUNCE_TH		2
#define KEY_SET_TH			50
#define KEY_CLR_TH			50

#define REL_DUR_TH          8
#define NOISTH              TOUCH_SET_TH/4

#define POS_NUM 			4
#define POS_DLY				1		// 0~1
//////////////////////////////////////////////////////
//
// Touch process variable
//
//////////////////////////////////////////////////////

static unsigned char suspend_flag = 0 ; //0: normal; 1: sleep

int WorkMode             = 1 ; //1: sleep, 2: normal

int Legal_touch          = 0 ; //0: illegal touch, 1: legal touch
      
int X_cur                = 0 ; // current frame coordinate-X                  
int X_pre                = 0 ; // pre-frame coordinate-X      
int X_pre1, X_pre2;
int X_rpt                = 0 ; // report coordinate-X

int Y_cur                = 0 ; // current frame coordinate-Y                 
int Y_pre                = 0 ; // pre-frame coordinate-Y           
int Y_rpt				 = 0 ; // report coordinate-Y
int Y_pre1, Y_pre2;
int X_rpt_valid, Y_rpt_valid;
int quere_num			 = 0;

int Touch_on             = 0 ; // Touch state, 0 : touch off, 1: touch on
int Press_flag           = 0 ; // Touch state, 0 : touch off, 1: touch on


int Key_Off_cnt              = 0 ; // Touch off count
int Key_On_cnt               = 0 ; // Touch on count

int Off_cnt              = 0 ; // Touch off count
int On_cnt               = 0 ; // Touch on count
int Press_debounce_cnt   = 0 ; // Press debounce count
int Release_debounce_cnt = 0 ; // Release debounce count
int Slide_debounce_cnt   = 0 ; // Slide debounce count


int Slide_flag           = 0 ; // slide flag, 0 : None slide, 1 : at slide
int Long_press_flag      = 0 ; // long press flag, 0 : None long press, 1 : at long press

int Click_times          = 0 ; // click times for double click 
int X_last[7], Y_last[7];

int delta[7];
int delta_key_sum = 0;

int deltaS[7];									// filted delta
int deltaS_raw[7];								// raw delta for tracing Base, Rawdata - baseline
int deltaS_app[7];								// composeted delta for touch detetion, delta_raw - delta_composate
int deltaS_composate[6] = {0, 0, 0, 0, 0, 0};   // composate deltaSx
int delta_buf[7][100];							// buffered delta 
int X_buff[16], Y_buff[16];

long Ini_sum[6];

unsigned int Initial_busy     = 1;   // 0: normal scan, 1: @ initial base
unsigned int Initial_cnt      = 0;   // initial base count

int Bl[7], Bl_trace_cnt[7];
int Nr = 0;
int first_slide = 0;
int	force_cnt = 0;
int Slide_st    = 0;
int Slide_over  = 0;

int sum=0;
int sum_pre;
int max=0;
int maxnum=0;
unsigned char weight_coor [3][5] = {
	{5,3,0,0,3},
	{8,5,3,0,4},
	{6,4,3,3,4}
};

unsigned int  long_touch_cnt = 0;
unsigned char key_clear_flag = 0;
unsigned char key_status = 0;
unsigned char key_clr_cnt = 0;
unsigned char key_set_cnt = 0;
unsigned char key_click_cnt = 0;

unsigned char illegal_touch_cnt = 0;

int deltaS_sum_pre;
int rel_flag;
int rev_rel_flag;
int rel_dur_cnt;
int rel_debounce_cnt;
int rel_start_sum;

struct pos_t{
	unsigned char flag;				// 0b 0000  00|press_flag|first_slider
	int x;
	int y;
};

struct pos_t pos[POS_NUM];

void De_jitter(void);
void delta_filter(void);
void delta_key_filter(void);
void Illegal_proc(void);

//////////////////////////////////////

static int debug_level=0;

#if TS_DEBUG_MSG
#define TS_DBG(format, ...)	\
    if(debug_level == 1)	\
printk(KERN_INFO "AW9163 " format "\n", ## __VA_ARGS__)
#else
#define TS_DBG(format, ...)
#endif

char	AW9163_UCF_FILENAME[50] = {0,};
unsigned int arraylen=59;

static unsigned int romcode[59] = 
{
    0x9f0a,0x800,0x900,0x3600,0x3,0x3700,0x190b,0x180a,0x1c00,0x1800,0x1008,0x520,0x1800,0x1004,0x513,0x1800,
    0x1010,0x52d,0x3,0xa4ff,0xa37f,0xa27f,0x3cff,0x3cff,0x3cff,0xc4ff,0xc37f,0xc27f,0x38ff,0x387f,0xbf00,0x3,
    0xa3ff,0xa47f,0xa27f,0x3cff,0x3cff,0x3cff,0xc3ff,0xc47f,0xc27f,0x38ff,0x387f,0xbf00,0x3,0xa2ff,0xa37f,0xa47f,
    0x3cff,0x3cff,0x3cff,0xc2ff,0xc37f,0xc47f,0x38ff,0x387f,0xbf00,0x3,0x3402,
};
//extern int camera_is_power_on;

struct AW9163_i2c_setup_data {
    unsigned i2c_bus;  //the same number as i2c->adap.nr in adapter probe function
    unsigned short i2c_address;
    int irq;
    char type[I2C_NAME_SIZE];
};

struct AW9163_ts_data {
    struct input_dev	*input_dev;
#ifdef AW9163_EINT_SUPPORT
    struct work_struct 	eint_work;
#else
    struct work_struct 	pen_event_work;
    struct workqueue_struct *ts_workqueue;
    struct timer_list touch_timer;
#endif
    struct early_suspend	early_suspend;
    struct timer_list touch_timer;
};

struct AW9163_ts_data *AW9163_ts;
struct wake_lock touchkey_wakelock;


//////////////////////////////////////////////////////

static struct i2c_client *this_client;
static struct AW9163_i2c_setup_data AW9163_ts_setup={AW9163_ts_I2C_BUS, AW9163_ts_I2C_ADDR, 0, AW9163_ts_NAME};

#if 0
extern void mt65xx_eint_unmask(unsigned int line);
extern void mt65xx_eint_mask(unsigned int line);
extern void mt65xx_eint_set_polarity(unsigned int eint_num, unsigned int pol);
extern void mt65xx_eint_set_hw_debounce(unsigned int eint_num, unsigned int ms);
extern unsigned int mt65xx_eint_set_sens(unsigned int eint_num, unsigned int sens);
extern void mt65xx_eint_registration(unsigned int eint_num, unsigned int is_deb_en, unsigned int pol, void (EINT_FUNC_PTR)(void), unsigned int is_auto_umask);
#endif
#ifdef MT6582
extern void mt_eint_mask(unsigned int eint_num);
extern void mt_eint_unmask(unsigned int eint_num);
extern void mt_eint_set_hw_debounce(unsigned int eint_num, unsigned int ms);
extern void mt_eint_set_polarity(unsigned int eint_num, unsigned int pol);
extern unsigned int mt_eint_set_sens(unsigned int eint_num, unsigned int sens);
extern void mt_eint_registration(unsigned int eint_num, unsigned int flow, void (EINT_FUNC_PTR)(void), unsigned int is_auto_umask);
extern void mt_eint_print_status(void);
#endif



unsigned int I2C_write_reg(unsigned char addr, unsigned int reg_data)
{

    int ret,i;
    u8 wdbuf[512] = {0};

    wdbuf[0] = addr;

    wdbuf[2] = (unsigned char)(reg_data & 0x00ff);
    wdbuf[1] = (unsigned char)((reg_data & 0xff00)>>8);

    struct i2c_msg msgs[] = {
        {
            .addr	= this_client->addr,
            .flags	= 0,
            .len	= 3,
            .buf	= wdbuf,
        },
    };

    ret = i2c_transfer(this_client->adapter, msgs, 1);
    if (ret < 0)
        pr_err("msg %s i2c read error: %d\n", __func__, ret);

    return ret;

}


unsigned int I2C_read_reg(unsigned char addr)
{

    int ret,i;
    u8 rdbuf[512] = {0};
    unsigned int getdata;

    rdbuf[0] = addr;
    struct i2c_msg msgs[] = {
        {
            .addr	= this_client->addr,
            .flags	= 0,
            .len	= 1,
            .buf	= rdbuf,
        },
        {
            .addr	= this_client->addr,
            .flags	= I2C_M_RD,
            .len	= 2,
            .buf	= rdbuf,
        },
    };

    ret = i2c_transfer(this_client->adapter, msgs, 2);
    if (ret < 0)
        pr_err("msg %s i2c read error: %d\n", __func__, ret);

    getdata=rdbuf[0] & 0x00ff;
    getdata<<= 8;
    getdata |=rdbuf[1];

    return getdata;

}

//jed add the power control here
static void AW9163_ts_pwron(void)
{
    mt_set_gpio_mode(AW9163_PDN,GPIO_MODE_00);
    mt_set_gpio_dir(AW9163_PDN,GPIO_DIR_OUT);
    mt_set_gpio_out(AW9163_PDN,GPIO_OUT_ONE);
    msleep(20);
}

static void AW9163_ts_pwroff(void)
{
    mt_set_gpio_out(AW9163_PDN,GPIO_OUT_ZERO);
}


static void AW9163_ts_config_pins(void)
{
    AW9163_ts_pwron();   //jed
    msleep(10); //wait for stable
}


/* adb file start*/
static ssize_t AW9163_show_debug(struct device* cd,struct device_attribute *attr, char* buf);
static ssize_t AW9163_store_debug(struct device* cd, struct device_attribute *attr,const char* buf, size_t len);
static ssize_t AW9163_show_writeoffset(struct device* cd,struct device_attribute *attr, char* buf);
static ssize_t AW9163_store_writeoffset(struct device* cd, struct device_attribute *attr,const char* buf, size_t len);
static ssize_t AW9163_get_reg(struct device* cd,struct device_attribute *attr, char* buf);
static ssize_t AW9163_write_reg(struct device* cd, struct device_attribute *attr,const char* buf, size_t len);
static ssize_t AW9163_get_adbBase(struct device* cd,struct device_attribute *attr, char* buf);
static ssize_t AW9163_get_rawdata(struct device* cd,struct device_attribute *attr, char* buf);
static ssize_t AW9163_get_delta(struct device* cd,struct device_attribute *attr, char* buf);
static ssize_t AW_get_ramled(struct device* cd,struct device_attribute *attr, char* buf);
static ssize_t AW_set_ramled(struct device* cd,struct device_attribute *attr, char* buf);
static ssize_t AW9163_get_irqstate(struct device* cd,struct device_attribute *attr, char* buf);
static ssize_t show_led_mode(struct device* cd,struct device_attribute *attr, char* buf);
static ssize_t store_led_mode(struct device* cd, struct device_attribute *attr,const char* buf, size_t len);


static DEVICE_ATTR(debug, S_IWUGO | S_IRUGO, AW9163_show_debug, AW9163_store_debug);
static DEVICE_ATTR(writeoffset, S_IWUGO | S_IRUGO, AW9163_show_writeoffset, AW9163_store_writeoffset);
static DEVICE_ATTR(getreg,  S_IWUGO | S_IRUGO, AW9163_get_reg,    AW9163_write_reg);
static DEVICE_ATTR(adbbase,  S_IWUGO | S_IRUGO, AW9163_get_adbBase,    NULL);
static DEVICE_ATTR(rawdata,  S_IWUGO | S_IRUGO, AW9163_get_rawdata,    NULL);
static DEVICE_ATTR(delta,  S_IWUGO | S_IRUGO, AW9163_get_delta,    NULL);
static DEVICE_ATTR(ramled,  S_IWUGO | S_IRUGO, AW_get_ramled,    AW_set_ramled);
static DEVICE_ATTR(getstate,  S_IWUGO | S_IRUGO, AW9163_get_irqstate,    NULL);
static DEVICE_ATTR(led_mode, S_IWUGO | S_IRUGO, show_led_mode, store_led_mode);

void AW9163_LED_Set(void);
static int led_mode = 0;
static ssize_t show_led_mode(struct device* cd,struct device_attribute *attr, char* buf)
{
    ssize_t ret = 0;

    sprintf(buf, "%d\n", led_mode);
    ret = strlen(buf) + 1;
    return ret;

}
static ssize_t store_led_mode(struct device* cd, struct device_attribute *attr,const char* buf, size_t len)
{
	sscanf(buf, "%d", &led_mode);
    TS_DBG("klutu %s: led_mode = %d\n", __FUNCTION__, led_mode);
    AW9163_LED_Set();
}

int enable_double_click_wakeup = 0;

static ssize_t store_double_click_wakeup_flag(struct device* cd, struct device_attribute *attr,
		       const char* buf, size_t len)
{
	sscanf(buf, "%d", &enable_double_click_wakeup);
	printk("enable_double_click_wakeup = %d\n", enable_double_click_wakeup);
	return len; 
}

static ssize_t show_double_click_wakeup_flag(struct device* cd,struct device_attribute *attr, char* buf)
{
	return sprintf(buf, "%d\n", enable_double_click_wakeup);
}

static DEVICE_ATTR(double_click_wakeup_flag, S_IWUGO | S_IRUGO, show_double_click_wakeup_flag, store_double_click_wakeup_flag);

int enable_backtouch_flag = 1; //default on
static ssize_t store_enable_backtouch(struct device* cd, struct device_attribute *attr,
		       const char* buf, size_t len)
{
	sscanf(buf, "%d", &enable_backtouch_flag);
	printk("enable_backtouch_flag = %d\n", enable_backtouch_flag);
	return len; 
}

static ssize_t show_enable_backtouch(struct device* cd,struct device_attribute *attr, char* buf)
{
	return sprintf(buf, "%d\n", enable_backtouch_flag);
}

static DEVICE_ATTR(enable_backtouch, S_IWUGO | S_IRUGO, show_enable_backtouch, store_enable_backtouch);
int AW_nvram_read(char *filename, char *buf, ssize_t len, int offset)
{
    struct file *fd;
    //ssize_t ret;
    int retLen = -1;

    mm_segment_t old_fs = get_fs();
    set_fs(KERNEL_DS);

    fd = filp_open(filename, O_RDONLY, 0);

    if(IS_ERR(fd)) {
        TS_DBG("[AW9163][nvram_read] : failed to open!!\n");
        return -1;
    }
    do{
        if ((fd->f_op == NULL) || (fd->f_op->read == NULL))
        {
            TS_DBG("[AW9163][nvram_read] : file can not be read!!\n");
            break;
        }

        if (fd->f_pos != offset) {
            if (fd->f_op->llseek) {
                if(fd->f_op->llseek(fd, offset, 0) != offset) {
                    TS_DBG("[AW9163][nvram_read] : failed to seek!!\n");
                    break;
                }
            } else {
                fd->f_pos = offset;
            }
        }

        retLen = fd->f_op->read(fd,
                buf,
                len,
                &fd->f_pos);

    }while(false);

    filp_close(fd, NULL);

    set_fs(old_fs);

    return retLen;
}

int AW_nvram_write(char *filename, char *buf, ssize_t len, int offset)
{
    struct file *fd;
    //ssize_t ret;
    int retLen = -1;

    mm_segment_t old_fs = get_fs();
    set_fs(KERNEL_DS);

    fd = filp_open(filename, O_WRONLY|O_CREAT, 0666);

    if(IS_ERR(fd)) {
        TS_DBG("[AW9163][nvram_write] : failed to open!!\n");
        return -1;
    }
    do{
        if ((fd->f_op == NULL) || (fd->f_op->write == NULL))
        {
            TS_DBG("[AW9163][nvram_write] : file can not be write!!\n");
            break;
        } /* End of if */

        if (fd->f_pos != offset) {
            if (fd->f_op->llseek) {
                if(fd->f_op->llseek(fd, offset, 0) != offset) {
                    TS_DBG("[AW9163][nvram_write] : failed to seek!!\n");
                    break;
                }
            } else {
                fd->f_pos = offset;
            }
        }

        retLen = fd->f_op->write(fd,
                buf,
                len,
                &fd->f_pos);

    }while(false);

    filp_close(fd, NULL);

    set_fs(old_fs);

    return retLen;
}



static ssize_t AW9163_show_debug(struct device* cd,struct device_attribute *attr, char* buf)
{
    ssize_t ret = 0;

    sprintf(buf, "AW9163 Debug %d\n",debug_level);

    ret = strlen(buf) + 1;

    return ret;
}

static ssize_t AW9163_store_debug(struct device* cd, struct device_attribute *attr,
        const char* buf, size_t len)
{
    unsigned long on_off = simple_strtoul(buf, NULL, 10);
    debug_level = on_off;

    TS_DBG("%s: debug_level=%d\n",__func__, debug_level);

    return len;
}


static ssize_t AW9163_show_writeoffset(struct device* cd,struct device_attribute *attr, char* buf)
{
    unsigned int offset1,offset2,offset3;
    ssize_t len = 0;
    mt_eint_mask(AW9163_EINT_NUM);
    len += snprintf(buf+len, PAGE_SIZE-len, "offset: \n");
    offset1=I2C_read_reg(0x13);
    offset2=I2C_read_reg(0x14);
    offset3=I2C_read_reg(0x15);
    len += snprintf(buf+len, PAGE_SIZE-len, "%x, ",offset1);
    len += snprintf(buf+len, PAGE_SIZE-len, "%x, ",offset2);
    len += snprintf(buf+len, PAGE_SIZE-len, "%x, ",offset3);
    len += snprintf(buf+len, PAGE_SIZE-len, "\n");
    mt_eint_unmask(AW9163_EINT_NUM);
    return len;
}
static ssize_t AW9163_store_writeoffset(struct device* cd, struct device_attribute *attr,
        const char* buf, size_t len)
{
    unsigned long on_off = simple_strtoul(buf, NULL, 10);
		if(on_off ==1)
		{
			mt_eint_mask(AW9163_EINT_NUM);
				I2C_write_reg(0x13,0x1b1b);
				I2C_write_reg(0x14,0x1b1b);
				I2C_write_reg(0x15,0x1b1b);
				on_off =0;
				len += snprintf(buf+len, PAGE_SIZE-len, "AW9163 write offset sucess!!! \n");
		}
		mt_eint_unmask(AW9163_EINT_NUM);
    return len;
}

static ssize_t AW9163_get_reg(struct device* cd,struct device_attribute *attr, char* buf)
{
    unsigned int reg_val[1];
    ssize_t len = 0;
    u8 i;
    mt_eint_mask(AW9163_EINT_NUM);
    for(i=0;i<0x7F;i++)
    {
        reg_val[0] = I2C_read_reg(i);
        len += snprintf(buf+len, PAGE_SIZE-len, "reg%2X = 0x%4X, ", i,reg_val[0]);
    }
    mt_eint_unmask(AW9163_EINT_NUM);
    return len;

}

static ssize_t AW9163_write_reg(struct device* cd, struct device_attribute *attr,
        const char* buf, size_t len)
{

    unsigned int databuf[2];
    mt_eint_mask(AW9163_EINT_NUM);
    if(2 == sscanf(buf,"%x %x",&databuf[0], &databuf[1]))
    {
        I2C_write_reg((u8)databuf[0],databuf[1]);
    }
    mt_eint_unmask(AW9163_EINT_NUM);
    return len;
}

static ssize_t AW9163_get_adbBase(struct device* cd,struct device_attribute *attr, char* buf)
{
    unsigned int dataS1,dataS2,dataS3,dataS4,dataS5,dataS6;
    ssize_t len = 0;
    mt_eint_mask(AW9163_EINT_NUM);
    len += snprintf(buf+len, PAGE_SIZE-len, "base: \n");
    I2C_write_reg(MCR,0x0003);

    dataS1=I2C_read_reg(0x36);
    dataS2=I2C_read_reg(0x37);
    dataS3=I2C_read_reg(0x38);
    dataS4=I2C_read_reg(0x39);
    dataS5=I2C_read_reg(0x3a);
    dataS6=I2C_read_reg(0x3b);
    len += snprintf(buf+len, PAGE_SIZE-len, "%d, ",dataS1);
    len += snprintf(buf+len, PAGE_SIZE-len, "%d, ",dataS2);
    len += snprintf(buf+len, PAGE_SIZE-len, "%d, ",dataS3);
    len += snprintf(buf+len, PAGE_SIZE-len, "%d, ",dataS4);
    len += snprintf(buf+len, PAGE_SIZE-len, "%d, ",dataS5);
    len += snprintf(buf+len, PAGE_SIZE-len, "%d, ",dataS6);

    len += snprintf(buf+len, PAGE_SIZE-len, "\n");
    mt_eint_unmask(AW9163_EINT_NUM);
    return len;
}

static ssize_t AW9163_get_rawdata(struct device* cd,struct device_attribute *attr, char* buf)
{
    unsigned int dataS1,dataS2,dataS3,dataS4,dataS5,dataS6;
    ssize_t len = 0;
    mt_eint_mask(AW9163_EINT_NUM);
    len += snprintf(buf+len, PAGE_SIZE-len, "base: \n");
    I2C_write_reg(MCR,0x0003);

    dataS1=I2C_read_reg(0x36);
    dataS2=I2C_read_reg(0x37);
    dataS3=I2C_read_reg(0x38);
    dataS4=I2C_read_reg(0x39);
    dataS5=I2C_read_reg(0x3a);
    dataS6=I2C_read_reg(0x3b);
    len += snprintf(buf+len, PAGE_SIZE-len, "%d, ",dataS1);
    len += snprintf(buf+len, PAGE_SIZE-len, "%d, ",dataS2);
    len += snprintf(buf+len, PAGE_SIZE-len, "%d, ",dataS3);
    len += snprintf(buf+len, PAGE_SIZE-len, "%d, ",dataS4);
    len += snprintf(buf+len, PAGE_SIZE-len, "%d, ",dataS5);
    len += snprintf(buf+len, PAGE_SIZE-len, "%d, ",dataS6);

    len += snprintf(buf+len, PAGE_SIZE-len, "\n");
    mt_eint_unmask(AW9163_EINT_NUM);
    return len;
}

static ssize_t AW9163_get_delta(struct device* cd,struct device_attribute *attr, char* buf)
{
    unsigned int deltaSS1,deltaSS2,deltaSS3,deltaSS4,deltaSS5,deltaSS6;
    ssize_t len = 0;
    mt_eint_mask(AW9163_EINT_NUM);
    len += snprintf(buf+len, PAGE_SIZE-len, "delta: \n");
    I2C_write_reg(MCR,0x0001);
		/*	
    deltaSS1=I2C_read_reg(0x36);if((deltaSS1 & 0x8000) == 0x8000) { deltaSS1 = 0; }
    deltaSS2=I2C_read_reg(0x37);if((deltaSS2 & 0x8000) == 0x8000) { deltaSS2 = 0; }
    deltaSS3=I2C_read_reg(0x38);if((deltaSS3 & 0x8000) == 0x8000) { deltaSS3 = 0; }
    deltaSS4=I2C_read_reg(0x39);if((deltaSS4 & 0x8000) == 0x8000) { deltaSS4 = 0; }
    deltaSS5=I2C_read_reg(0x3a);if((deltaSS5 & 0x8000) == 0x8000) { deltaSS5 = 0; }
    deltaSS6=I2C_read_reg(0x3b);if((deltaSS6 & 0x8000) == 0x8000) { deltaSS6 = 0; }
    len += snprintf(buf+len, PAGE_SIZE-len, "%d, ",deltaSS1);
    len += snprintf(buf+len, PAGE_SIZE-len, "%d, ",deltaSS2);
    len += snprintf(buf+len, PAGE_SIZE-len, "%d, ",deltaSS3);
    len += snprintf(buf+len, PAGE_SIZE-len, "%d, ",deltaSS4);
    len += snprintf(buf+len, PAGE_SIZE-len, "%d, ",deltaSS5);
    len += snprintf(buf+len, PAGE_SIZE-len, "%d, ",deltaSS6);
*/
			len += snprintf(buf+len, PAGE_SIZE-len, "%d, ",deltaS[0]);
			len += snprintf(buf+len, PAGE_SIZE-len, "%d, ",deltaS[1]);
			len += snprintf(buf+len, PAGE_SIZE-len, "%d, ",deltaS[2]);
			len += snprintf(buf+len, PAGE_SIZE-len, "%d, ",deltaS[3]);
			len += snprintf(buf+len, PAGE_SIZE-len, "%d, ",deltaS[4]);
			len += snprintf(buf+len, PAGE_SIZE-len, "%d, ",deltaS[5]);
    len += snprintf(buf+len, PAGE_SIZE-len, "\n");
    mt_eint_unmask(AW9163_EINT_NUM);
    return len;
}

static ssize_t AW9163_get_irqstate(struct device* cd,struct device_attribute *attr, char* buf)
{
    unsigned int keytouch,keyS1,keyS2,keyS3,keyS4,keyS5,keyS6;
    unsigned int gesture,slide1,slide2,slide3,slide4,doubleclick1,doubleclick2;
    ssize_t len = 0;
    mt_eint_mask(AW9163_EINT_NUM);
    len += snprintf(buf+len, PAGE_SIZE-len, "keytouch: \n");

    keytouch=I2C_read_reg(0x31);
    if((keytouch&0x1) == 0x1) keyS1=1;else keyS1=0;
    if((keytouch&0x2) == 0x2) keyS2=1;else keyS2=0;
    if((keytouch&0x4) == 0x4) keyS3=1;else keyS3=0;
    if((keytouch&0x8) == 0x8) keyS4=1;else keyS4=0;
    if((keytouch&0x10) == 0x10) keyS5=1;else keyS5=0;
    if((keytouch&0x20) == 0x20) keyS6=1;else keyS6=0;

    len += snprintf(buf+len, PAGE_SIZE-len, "%d, ",keyS1);
    len += snprintf(buf+len, PAGE_SIZE-len, "%d, ",keyS2);
    len += snprintf(buf+len, PAGE_SIZE-len, "%d, ",keyS3);
    len += snprintf(buf+len, PAGE_SIZE-len, "%d, ",keyS4);
    len += snprintf(buf+len, PAGE_SIZE-len, "%d, ",keyS5);
    len += snprintf(buf+len, PAGE_SIZE-len, "%d, ",keyS6);
    len += snprintf(buf+len, PAGE_SIZE-len, "\n");

    len += snprintf(buf+len, PAGE_SIZE-len, "gesture: \n");
    gesture=I2C_read_reg(0x2e);
    if(gesture == 0x1) slide1=1;else slide1=0;
    if(gesture == 0x2) slide2=1;else slide2=0;
    if(gesture == 0x4) slide3=1;else slide3=0;
    if(gesture == 0x8) slide4=1;else slide4=0;
    if(gesture == 0x10) doubleclick1=1;else doubleclick1=0;
    if(gesture == 0x200) doubleclick2=1;else doubleclick2=0;

    len += snprintf(buf+len, PAGE_SIZE-len, "%d, ",slide1);
    len += snprintf(buf+len, PAGE_SIZE-len, "%d, ",slide2);
    len += snprintf(buf+len, PAGE_SIZE-len, "%d, ",slide3);
    len += snprintf(buf+len, PAGE_SIZE-len, "%d, ",slide4);
    len += snprintf(buf+len, PAGE_SIZE-len, "%d, ",doubleclick1);
    len += snprintf(buf+len, PAGE_SIZE-len, "%d, ",doubleclick2);

    len += snprintf(buf+len, PAGE_SIZE-len, "\n");
    mt_eint_unmask(AW9163_EINT_NUM);
    return len;
}





char DataToInt(char s_char)
{
    if((s_char>47)&&(s_char<58))
    {
        s_char-=48;
    }
    else
    {
        if((s_char>64)&&(s_char<71))
        {
            s_char-=55;
        }
        else
        {
            if((s_char>96)&&(s_char<103))
            {
                s_char-=87;
            }
        }
    }
    return s_char;
}

char AW9163_Get_UCF(void)
{
	return 0;
/*
    int ret;
    int i=0;
    int j=0;
    unsigned int buf[255];
    unsigned char buff[1000];
	ret = AW_nvram_read(AW9163_UCF_FILENAME,&buff[0],1000,0);
    if(ret == -1)
    {
        TS_DBG("AW9163 NO UCF FILE,use default UCF\n");
        return 0;
    }
    else
    {

        buff[i]=DataToInt(buff[i]);
        if(buff[i+1]==13&&buff[i+2]==10)//一位数
        {
            buf[j]=buff[i];
            i++;
            j++;
        }
        else if(buff[i+2]==13&&buff[i+3]==10)//两位数
        {

            buff[i+1]=DataToInt(buff[i+1]);
            buf[j]=10*buff[i]+buff[i+1];
            i=i+2;
            j++;
        }
        else if(buff[i+3]==13&&buff[i+4]==10)//三位数
        {
            buff[i+1]=DataToInt(buff[i+1]);
            buff[i+2]=DataToInt(buff[i+2]);
            buf[j]=10*10*buff[i]+10*buff[i+1]+buff[i+2];
            i=i+3;
            j++;
        }
        else if(buff[i+4]==13&&buff[i+5]==10)//四位数
        {
            buff[i+1]=DataToInt(buff[i+1]);
            buff[i+2]=DataToInt(buff[i+2]);
            buff[i+3]=DataToInt(buff[i+3]);
            buf[j]=10*10*10*buff[i]+10*10*buff[i+1]+10*buff[i+2]+buff[i+3];
            i=i+4;
            j++;

        }
			  while(i<ret)
        {
            if(buff[i]==32||buff[i]==10||buff[i]==13||buff[i]==44)
            {
                i++;
                continue;
            }
            else
            {

                buff[i+2]=DataToInt(buff[i+2]);

                if(buff[i+3]==44)//一位数
                {
                    buff[i+2]=DataToInt(buff[i+2]);
                    buf[j]=buff[i+2];
                    i=i+3;
                    j++;
                }
                else
                {
                    if(buff[i+4]==44)//两位数
                    {
                        buff[i+3]=DataToInt(buff[i+3]);
                        buff[i+2]=DataToInt(buff[i+2]);
                        buf[j]=16*buff[i+2]+buff[i+3];
                        i=i+4;
                        j++;
                    }
                    else
                    {
                        if(buff[i+5]==44)//3 //三位数
                        {
                            buff[i+4]=DataToInt(buff[i+4]);
                            buff[i+3]=DataToInt(buff[i+3]);
                            buff[i+2]=DataToInt(buff[i+2]);
                            buf[j]=16*16*buff[i+2]+16*buff[i+3]+buff[i+4];
                            i=i+5;
                            j++;
                        }
                        else
                        {
                            if(buff[i+6]==44)//四位数
                            {
                                buff[i+5]=DataToInt(buff[i+5]);
                                buff[i+4]=DataToInt(buff[i+4]);
                                buff[i+3]=DataToInt(buff[i+3]);
                                buff[i+2]=DataToInt(buff[i+2]);
                                buf[j]=16*16*16*buff[i+2]+16*16*buff[i+3]+16*buff[i+4]+buff[i+5];
                                i=i+6;
                                j++;
                            }
                        }

                    }

                }
            }
        }


        arraylen=buf[0];
        for(i=0;i<arraylen;i++)
        {
            romcode[i]=buf[i+1];
        }


    }
    return 1;
*/	
}

static ssize_t AW_get_ramled(struct device* cd,struct device_attribute *attr, char* buf)
{
    unsigned char i,j;
    ssize_t len = 0;
    int ret;
    len += snprintf(buf+len, PAGE_SIZE-len, "romcode: \n");

    for(j=0;j<arraylen;j++)
    {
        len += snprintf(buf+len, PAGE_SIZE-len, "%x, ",romcode[j]);
    }
    len += snprintf(buf+len, PAGE_SIZE-len, "\n");
		
		return len;
}

static ssize_t AW_set_ramled(struct device* cd,struct device_attribute *attr, char* buf)
{
    unsigned char i,j;
    ssize_t len = 0;
    int ret;
    unsigned long on_off = simple_strtoul(buf, NULL, 10);

    if(on_off ==1)
    {
        mt_eint_mask(AW9163_EINT_NUM);
        ret =AW9163_Get_UCF();

        if(ret == 1)
        {
            I2C_write_reg(GCR,0x0003);    // LED enable and touch scan enable
            ///////////////////////////////////////
            // LED RAM program

            I2C_write_reg(INTVEC,0x5);
            I2C_write_reg(TIER,0x1c);
            I2C_write_reg(PMR,0x0000);
            I2C_write_reg(RMR,0x0000);
            I2C_write_reg(WADDR,0x0000);

            for(i=0;i<arraylen;i++)
            {
                I2C_write_reg(WDATA,romcode[i]);
            }

            I2C_write_reg(SADDR,0x0000);
            I2C_write_reg(PMR,0x0001);
            I2C_write_reg(RMR,0x0002);

			len += snprintf(buf+len, PAGE_SIZE-len, "AW9163 get ucf sucess!!! \n");
        }
        else
        {
            len += snprintf(buf+len, PAGE_SIZE-len, "AW9163 get ramled ucf fail!!! \n");
        }
        mt_eint_unmask(AW9163_EINT_NUM);
    }
	return len;
}

static int AW9163_create_sysfs(struct i2c_client *client)
{
    int err;
    struct device *dev = &(client->dev);

    TS_DBG("%s", __func__);

    err = device_create_file(dev, &dev_attr_debug);
	err = device_create_file(dev, &dev_attr_writeoffset);
    err = device_create_file(dev, &dev_attr_getreg);
    err = device_create_file(dev, &dev_attr_adbbase);
    err = device_create_file(dev, &dev_attr_rawdata);
    err = device_create_file(dev, &dev_attr_delta);
    err = device_create_file(dev, &dev_attr_ramled);
    err = device_create_file(dev, &dev_attr_getstate);
	err = device_create_file(dev, &dev_attr_double_click_wakeup_flag);
	err = device_create_file(dev, &dev_attr_enable_backtouch);
    err = device_create_file(dev, &dev_attr_led_mode);
    return err;
}
/*adb file end*/

/*AW9163 body start*/

//////////////////////////////////////////////////////////////////////
/* 	MAX(x1,x2,x3): max val in x1,x2,x3                              */
//////////////////////////////////////////////////////////////////////
unsigned int MAX(unsigned int x1, unsigned int x2, unsigned int x3)
{
	unsigned int max_val;
	
	if(x1<=x2) 
	{
		if(x2<=x3)
			max_val = x3;
		else
			max_val = x2;
	}
	else      
	{
		if(x1<=x3)
			max_val = x3;
		else
			max_val = x1;
	}

	return max_val;
}
//////////////////////////////////////////////////////////////////////
/* 	MIN(x1,x2,x3): min val in x1,x2,x3                              */
//////////////////////////////////////////////////////////////////////
unsigned int MIN(unsigned int x1, unsigned int x2, unsigned int x3)
{
    unsigned int min_val;

    if(x1>=x2)
    {
        if(x2>=x3)
            min_val = x3;
        else
            min_val = x2;
    }
    else
    {
        if(x1>=x3)
            min_val = x3;
        else
            min_val = x1;
    }

    return min_val;
}

//////////////////////////////////////////////////////////////////////
// AW9163 initial register @ mobile active
//////////////////////////////////////////////////////////////////////
void AW_NormalMode(void)
{
	unsigned int i;
    unsigned int reg;
    reg = I2C_read_reg(0x01);
    TS_DBG("klutu %s: GCR = 0x%x\n", __FUNCTION__, reg);
	I2C_write_reg(GCR, reg & 0xfffd);  // disable chip

        ///////////////////////////////////////
        // LED config
//	I2C_write_reg(LER1,0x00ff);
	//I2C_write_reg(LER1,0x0008);
	//I2C_write_reg(LER2,0x0000);

	//I2C_write_reg(CTRS1,0x0000);
	//I2C_write_reg(CTRS2,0x0000);
	//I2C_write_reg(IMAX1,0x3333);
	//I2C_write_reg(IMAX2,0x3333);
	//I2C_write_reg(IMAX3,0x0000);
	//I2C_write_reg(IMAX4,0x0000);
	//I2C_write_reg(IMAX5,0x0000);

	//I2C_write_reg(LCR,0x00a1);
	I2C_write_reg(IDLECR,0x1800);
        ///////////////////////////////////////
        // cap-touch config
	I2C_write_reg(SLPR,0x0000);   // touch key enable
	I2C_write_reg(SCFG1,0x0082);  // 
	I2C_write_reg(SCFG2,0x202);
	
	I2C_write_reg(OFR1,0x1718);   // offset
	I2C_write_reg(OFR2,0x1717);
	I2C_write_reg(OFR3,0x1818);

	I2C_write_reg(THR0, 0x191e);
	I2C_write_reg(THR1, 0x191e);
	I2C_write_reg(THR2, 0x191e);
	I2C_write_reg(THR3, 0x191e);
	I2C_write_reg(THR4, 0x191e);
	I2C_write_reg(THR5, 0x191e);

    I2C_write_reg(SETCNT,0x0202);  // debounce
    //I2C_write_reg(IDLECR,0x1805);
    I2C_write_reg(BLCTH,0x0402);   // base trace rate : increment 8 or decrement 16 in 1s

    I2C_write_reg(AKSR,0x0000);    // AKS

    I2C_write_reg(INTER,0x0080);   // interrupt

    I2C_write_reg(GDTR,0x0000);    // gesture
    I2C_write_reg(GDCFGR,0x000);
    I2C_write_reg(TAPR1,0x0000);
    I2C_write_reg(TAPR2,0x0000);
    //I2C_write_reg(TDTR,0x0a1f);
    I2C_write_reg(TDTR,0x0000);
    I2C_write_reg(GIER,0x0000);

    ///////////////////////////////////////
    I2C_write_reg(GCR, reg | 0x0002);    // LED enable and touch scan enable

    ///////////////////////////////////////
    // LED RAM program
    /*
       I2C_write_reg(INTVEC,0x5);
       I2C_write_reg(TIER,0x1c);
       I2C_write_reg(PMR,0x0000);
       I2C_write_reg(RMR,0x0000);
       I2C_write_reg(WADDR,0x0000);

       for(i=0;i<59;i++)
       {
       I2C_write_reg(WDATA,romcode[i]);
       }

       I2C_write_reg(SADDR,0x0000);
       I2C_write_reg(PMR,0x0001);
       I2C_write_reg(RMR,0x0002);
     */
    WorkMode = 2;
    TS_DBG("AW9163 enter Normal mode\n");

}


//////////////////////////////////////////////////////////////////////
// AW9163 initial register @ mobile sleep
//////////////////////////////////////////////////////////////////////
void AW_SleepMode(void)
{
    unsigned int i;
    unsigned int reg;
    reg = I2C_read_reg(0x01);
    TS_DBG("klutu %s: GCR = 0x%x\n", __FUNCTION__, reg);
	I2C_write_reg(GCR, reg & 0xfffd);  // disable chip

        ///////////////////////////////////////
        // LED config
	//I2C_write_reg(LER1,0x0000);
	//I2C_write_reg(LER1,0x0008);
	//I2C_write_reg(LER2,0x0000);
	//I2C_write_reg(CTRS1,0x0000);
	//I2C_write_reg(CTRS2,0x0000);

	//I2C_write_reg(LCR,0x00a1);
	I2C_write_reg(IDLECR,0x1800);

	//I2C_write_reg(IMAX1,0x3333);
	//I2C_write_reg(IMAX2,0x3333);
	//I2C_write_reg(IMAX3,0x0000);
	//I2C_write_reg(IMAX4,0x0000);
	//I2C_write_reg(IMAX5,0x0000);

        ///////////////////////////////////////
        // cap-touch config
	I2C_write_reg(SLPR,0x0000);   // touch key enable
	I2C_write_reg(SCFG1,0x0084);  // 
	I2C_write_reg(SCFG2,0x284);
	
	I2C_write_reg(OFR1,0x1617);   // offset
	I2C_write_reg(OFR2,0x1616);
	I2C_write_reg(OFR3,0x1717);

	I2C_write_reg(THR0, 0x0b0f);
	I2C_write_reg(THR1, 0x0b0f);
	I2C_write_reg(THR2, 0x0b0f);
	I2C_write_reg(THR3, 0x0b0f);
	I2C_write_reg(THR4, 0x0b0f);
	I2C_write_reg(THR5, 0x0b0f);
	
	I2C_write_reg(SETCNT,0x0202);
	//I2C_write_reg(IDLECR,0x1805);
	I2C_write_reg(BLCTH,0x0402);

	I2C_write_reg(AKSR,0x0000);

	I2C_write_reg(INTER,0x0000);


	//I2C_write_reg(GTIMR,0x0016);
	//I2C_write_reg(GDTR,0x0021);
	I2C_write_reg(GDCFGR,0x025f);
	I2C_write_reg(TAPR1,0x00FE);
	I2C_write_reg(TAPR2,0x0000);
	//I2C_write_reg(TDTR,0x0a1f);
	I2C_write_reg(TDTR,0x1932);
	I2C_write_reg(GIER,0x0010);
	
	//I2C_write_reg(MCR,0x0003);

        ///////////////////////////////////////
	I2C_write_reg(GCR, reg | 0x0002);   // enable chip

/*
	I2C_write_reg(INTVEC,0x5);	
	I2C_write_reg(TIER,0x1c);  
	I2C_write_reg(PMR,0x0000);
	I2C_write_reg(RMR,0x0000);
	I2C_write_reg(WADDR,0x0000);

	for(i=0;i<59;i++)
	{
		I2C_write_reg(WDATA,romcode[i]);
	}

	I2C_write_reg(SADDR,0x0000);
	I2C_write_reg(PMR,0x0001);
	I2C_write_reg(RMR,0x0002);
*/
	WorkMode = 1;
    TS_DBG("AW9163 enter Sleep mode\n");
}

enum {
    LED_OFF_ALL = 0,
    LED_CH1 = 1,
    LED_CH2,
    LED_CH3,
    LED_OFF_CH1 = 41,
    LED_OFF_CH2,
    LED_OFF_CH3,
};

void AW9163_LED_Set(void)
{
    unsigned int reg;
    reg = I2C_read_reg(0x01);
    printk("klutu %s: GCR = 0x%4x", __FUNCTION__, reg);

    switch (led_mode) {
    case LED_CH1:
        I2C_write_reg(GCR, reg | 0x0001);
        /* I2C_write_reg(GCR,0x0003); */
        I2C_write_reg(IMAX1,0x1111);
        I2C_write_reg(IMAX2,0x1111);
        I2C_write_reg(LER1,0x001c);
        I2C_write_reg(CTRS1,0x00fc);
        I2C_write_reg(CMDR,0xa2ff);
        /* I2C_write_reg(CMDR,0xa3ff); */
        /* I2C_write_reg(CMDR,0xa4ff); */
        break;
    case LED_CH2:
        I2C_write_reg(GCR, reg | 0x0001);
        /* I2C_write_reg(GCR,0x0003); */
        I2C_write_reg(IMAX1,0x1111);
        I2C_write_reg(IMAX2,0x1111);
        I2C_write_reg(LER1,0x001c);
        I2C_write_reg(CTRS1,0x00fc);
        /* I2C_write_reg(CMDR,0xa2ff); */
        I2C_write_reg(CMDR,0xa3ff);
        /* I2C_write_reg(CMDR,0xa4ff); */
        break;
    case LED_CH3:
        I2C_write_reg(GCR, reg | 0x0001);
        /* I2C_write_reg(GCR,0x0003); */
        I2C_write_reg(IMAX1,0x1111);
        I2C_write_reg(IMAX2,0x1111);
        I2C_write_reg(LER1,0x001c);
        I2C_write_reg(CTRS1,0x00fc);
        /* I2C_write_reg(CMDR,0xa2ff); */
        /* I2C_write_reg(CMDR,0xa3ff); */
        I2C_write_reg(CMDR,0xa4ff);
        break;

    case LED_OFF_ALL:
        I2C_write_reg(LER1,0x001c);
        I2C_write_reg(CTRS1,0x00fc);
        I2C_write_reg(CMDR,0xa200);
        I2C_write_reg(CMDR,0xa300);
        I2C_write_reg(CMDR,0xa400);
        I2C_write_reg(GCR, reg & 0xfffe);
        break;

    case LED_OFF_CH1:
        I2C_write_reg(LER1,0x001c);
        I2C_write_reg(CTRS1,0x00fc);
        I2C_write_reg(CMDR,0xa200);
        break;

    case LED_OFF_CH2:
        I2C_write_reg(LER1,0x001c);
        I2C_write_reg(CTRS1,0x00fc);
        I2C_write_reg(CMDR,0xa300);
        break;

    case LED_OFF_CH3:
        I2C_write_reg(LER1,0x001c);
        I2C_write_reg(CTRS1,0x00fc);
        I2C_write_reg(CMDR,0xa400);
        break;
    }
    printk("klutu done\n");
}

#if 0
void AW9163_LED_ON(void)
{
    I2C_write_reg(IMAX1,0x3333);
    I2C_write_reg(IMAX2,0x3333);
    I2C_write_reg(LER1,0x001c);
    I2C_write_reg(CTRS1,0x00fc);
    I2C_write_reg(CMDR,0xa2ff);
    I2C_write_reg(CMDR,0xa3ff);
    I2C_write_reg(CMDR,0xa4ff);
    I2C_write_reg(CMDR,0xa5ff);
    I2C_write_reg(GCR,0x0003);
}

void AW9163_LED_OFF(void)
{
    I2C_write_reg(IMAX1,0x3333);
    I2C_write_reg(IMAX2,0x3333);
    I2C_write_reg(LER1,0x001c);
    I2C_write_reg(CTRS1,0x00fc);
    I2C_write_reg(CMDR,0xa200);
	I2C_write_reg(CMDR,0xa300);
	I2C_write_reg(CMDR,0xa400);
	I2C_write_reg(CMDR,0xa500);
    I2C_write_reg(GCR,0x0003);
}
#endif




	int delta[7];
long rawdata_sum_pre;
void Delta_proc(void) 
{
	unsigned int bf0[7];

	int i, j;
	int  weight[7] = {1, 1, 1, 1, 1, 1, 1};	// {1, 2, 4, 8, 16, 32, 64};
	long delta_sum[7];
	long filter_min[7], filter_max[7];
	long min_delta;
	long tmp;
	long rawdata_sum = 0;

	I2C_write_reg(0x1e,0x3);

		bf0[0] = (int)I2C_read_reg(0x36);
		bf0[1] = (int)I2C_read_reg(0x37);
		bf0[2] = (int)I2C_read_reg(0x38);
		bf0[3] = (int)I2C_read_reg(0x39);
		bf0[4] = (int)I2C_read_reg(0x3a);
		bf0[5] = (int)I2C_read_reg(0x3b);
	
	if ( Initial_busy==1 ) 
	{
		Nr = 0;

		for(i=0;i<6;i++) 
		{
			delta_buf[i][Initial_cnt] = 0;
			delta[i] = 0;
			deltaS_composate[i] = 0;
			rawdata_sum = rawdata_sum + bf0[i];
		}

		for(i=0;i<6;i++) Ini_sum[i] = (Initial_cnt==0) ? (bf0[i]) : (Ini_sum[i] + bf0[i]);
		Initial_busy = (Initial_cnt==7) ? 0 : 1;
		for(i=0;i<6;i++) Bl[i] = (Initial_cnt==7) ? (Ini_sum[i]/8) : Bl[i];
		Initial_cnt  = (Initial_cnt==7) ? 7 : (Initial_cnt + 1);
		
		if (Initial_cnt>=1)
		{
			if ( ABS(rawdata_sum, rawdata_sum_pre) > TOUCH_SET_TH )
			{
				Initial_cnt = 0;
				Initial_busy = 1;
			}
		}
			
		rawdata_sum_pre = rawdata_sum;
	}
	else
	{
		Initial_cnt = 0;
		for(i=0;i<6;i++) delta[i] = bf0[i] - Bl[i] ;
	}
/*
	for(i=0;i<6;i++) deltaS_raw[i] = delta[i];

	for(i=0;i<6;i++) 
	{
		deltaS_app[i] = delta[i] - deltaS_composate[i];
	}
*/
	delta_filter();
	delta_key_filter();
	printk("AW9163 Rawdata: %d, %d, %d, %d, %d, %d\n",bf0[0],bf0[1],bf0[2],bf0[3],bf0[4],bf0[5]);
	printk("AW9163 deltaS: %d, %d, %d, %d, %d, %d\n",deltaS[0],deltaS[1],deltaS[2],deltaS[3],deltaS[4],deltaS[5]);
//	printk("AW9163 d0=	%d,d1=	%d,d2=	%d,d3=	%d,d4=	%d,d5=	%d\n",deltaS[0],deltaS[1],deltaS[2],deltaS[3],deltaS[4],deltaS[5]);
//	printk("AW9163 dc0=	%d,dc1=	%d,dc2=	%d,dc3=	%d,dc4=	%d,dc5=	%d\n",deltaS_composate[0],deltaS_composate[1],deltaS_composate[2],deltaS_composate[3],deltaS_composate[4],deltaS_composate[5]);
}

void delta_key_filter()
{
	int i,j;
	delta_key_sum = 0;
	for(i=0; i<6; i++)
	{
		//delta_key_sum += (delta[i] - deltaS_composate[i]);
		delta_key_sum += deltaS[i];
	}
}

#define FILT_WINDOW  5



void delta_filter()
{
	int i, j;
	int  weight[100] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};	// {1, 2, 4, 8, 16, 32, 64};
	long delta_sum[100];
	long filter_min[7], filter_max[7];
	long filter_min0[7], filter_max0[7];
	long min_delta;
	long tmp;
	long deltaS_max=0;
	long deltaS_min=0;
	int  key1_delta ;
	int  key2_delta ;
	int  key3_delta ;
	int  illegal_cnd1 ;
	int  illegal_cnd2 ;

	for(i=0;i<6;i++) 
	{
		for(j=(FILT_WINDOW-1);j>=0;j--) 
		{
			if ( j==0 ) 
				//delta_buf[i][0] = deltaS_app[i] ;
				delta_buf[i][0] = delta[i] ;
			else
				delta_buf[i][j] = delta_buf[i][j-1] ;
		}
	}

	for(i=0;i<6;i++) 
	{
		delta_sum[i] = 0; 
		filter_min[i] = delta_buf[i][0];
		filter_max[i] = delta_buf[i][0];

		filter_min0[i] = delta_buf[i][0];
		filter_max0[i] = delta_buf[i][0];		

		for(j=0;j<FILT_WINDOW;j++) 
		{
			if (filter_max[i] < delta_buf[i][j]) 
			{
				filter_max0[i]= filter_max[i];		
				filter_max[i] = delta_buf[i][j];
			}
			else if (delta_buf[i][j] > filter_max0[i])
			{
				filter_max0[i] = delta_buf[i][j];	
			}
			
			if (filter_min[i] > delta_buf[i][j]) 
			{
				filter_min0[i] = filter_min[i];
				filter_min[i]  = delta_buf[i][j];
			}
			else if (delta_buf[i][j] < filter_min0[i])
			{
				filter_min0[i] = delta_buf[i][j];
			}
			
			
			delta_sum[i] = delta_sum[i] + weight[j] * delta_buf[i][j] ;
		}

		//delta_sum[i] = delta_sum[i] - filter_min[i] - filter_max[i] - filter_min0[i] - filter_max0[i];
		delta_sum[i] = delta_sum[i] - filter_min[i] - filter_max[i];
	}

	// search the minimum delta within all keys
	for(i=0;i<6;i++) 
	{
		if ( i==0 )
			min_delta = delta_sum[0];
		else
		{
			if ( delta_sum[i] < min_delta) min_delta = delta_sum[i];
		}
	}

	min_delta = 0;
	if ( min_delta <= 0 ) {
		for(i=0;i<6;i++) deltaS_raw[i] = delta_sum[i]/(FILT_WINDOW-2);
	}
	else {
		for(i=0;i<6;i++) deltaS_raw[i] = (delta_sum[i] - min_delta)/(FILT_WINDOW-2);
	}

	for(i=0;i<6;i++) deltaS[i] = deltaS_raw[i] - deltaS_composate[i];

	sum =0;
	for(i=0;i<6;i++) sum +=deltaS[i];

	for(i=0;i<6;i++)
	{
		if(deltaS[i]>max)
		{
			max=deltaS[i];
			maxnum=i;
		}		
	}

	/* If the sum of delta below Base too large, Force initialize Baseline */
	if (sum < (0-TOUCH_SET_TH) ) // illegal touch 																											  
	{
		force_cnt++;
		if (force_cnt==8)
		{
			Initial_busy = 1;
			force_cnt = 0;;
		}
	}
	else { force_cnt = 0; } 

	sum = 0;
	for(i=0;i<6;i++) 
	{
		deltaS[i] = (deltaS[i]<0) ? 0 : deltaS[i];
		sum += deltaS[i];
	}
	
	key1_delta = deltaS[0] + deltaS[3];
	key2_delta = deltaS[2] + deltaS[4];
	key3_delta = deltaS[1] + deltaS[5];

	deltaS_max = MAX(key1_delta, key2_delta, key3_delta);
	deltaS_min = MIN(key1_delta, key2_delta, key3_delta);

	illegal_cnd1 = ( (deltaS_max-deltaS_min) < BIG_TH );
	illegal_cnd2 = (( deltaS[0] > KEY_HIGH_TH ) || 
                   ( deltaS[1] > KEY_HIGH_TH ) || 
                   ( deltaS[2] > KEY_HIGH_TH ) || 
                   ( deltaS[3] > KEY_HIGH_TH ) || 
                   ( deltaS[4] > KEY_HIGH_TH ) || 
                   ( deltaS[5] > KEY_HIGH_TH ) ); 

	//if ((sum > TOUCH_HIGH_TH ) || (illegal_cnd1==1) || (illegal_cnd2==1) )
	if ((sum > TOUCH_HIGH_TH ) || (illegal_cnd2==1) )
		Legal_touch = 0;
	else
		Legal_touch = 1;



}


//////////////////////////////////////////////////////////////////////
/* 	Key_Proc Function:												*/
/* 	Press and Release Debounce										*/
/*  Generate Stable one click & double click Flag					*/
//////////////////////////////////////////////////////////////////////
void Key_proc(void)
{
	if(Press_flag)
	{
		Key_Off_cnt = 0 ;
		
		if ( first_slide == 1) 
		{
			Key_On_cnt = 0;
		}
		else
		{
			if(Key_On_cnt >= SAT_VAL) 
				Key_On_cnt = SAT_VAL;
			else 
				Key_On_cnt++;
		}		
	}
	else
	{
		Key_On_cnt = 0 ;
		
		if(Key_Off_cnt >= SAT_VAL) 
			Key_Off_cnt = SAT_VAL;
		else 
			Key_Off_cnt++;
	}

	if(first_slide)
	{
		key_click_cnt = 0;
	}
	
}

//////////////////////////////////////////////////////////////////////
/* 	Touch_Proc Function:											*/
/* 	Press and Release Debounce										*/
/*  Generate Stable Touch_on Flag									*/
//////////////////////////////////////////////////////////////////////
void Touch_proc(void)
{
	long tmp = 0;
	int i;
	int key1_delta, key2_delta, key3_delta;
	int deltaS_min, deltaS_max;
	
	for (i=0; i<6; i++) tmp += deltaS[i];
	
	key1_delta = deltaS[0] + deltaS[3];
	key2_delta = deltaS[2] + deltaS[4];
	key3_delta = deltaS[1] + deltaS[5];

	deltaS_max = MAX(key1_delta, key2_delta, key3_delta);
	deltaS_min = MIN(key1_delta, key2_delta, key3_delta);

	if ( tmp>TOUCH_SET_TH && ( (deltaS_max-deltaS_min) > BIG_TH ) )
		Touch_on = 1;
	else
		Touch_on = 0;

	if ( Press_flag == 1 )		// ON state
	{
        if ( rel_flag==0)
        {
        	rev_rel_flag = 0;
        	rel_debounce_cnt = 0;
        	if ( (deltaS_sum_pre - tmp) > NOISTH )
        	{
        		rel_flag = 1;
        		rel_dur_cnt = 1;
        		rel_start_sum = deltaS_sum_pre;
        	}
        }
        else
        {
        	rel_dur_cnt++;
        	if ( (rel_start_sum - tmp) >  TOUCH_SET_TH )
        	{
        		rel_debounce_cnt++;
        	}
        	else
        	{
        		rel_debounce_cnt = 0;
        	}
        
        	if ( rel_debounce_cnt >= REV_DEBOUNCE_TH )
        	{
        		rev_rel_flag = 1;
        		rel_flag = 0;
        		rel_dur_cnt = 0;
        		rel_debounce_cnt = 0;
        	}
        	else if ( rel_dur_cnt >= REL_DUR_TH )
        	{
        		rel_flag = 0;
        		rel_dur_cnt = 0;
        	}
        }
	}
	else
	{
			rel_flag = 0;
			rel_dur_cnt = 0;
			rel_debounce_cnt = 0;
			rel_flag = 0;
			rel_dur_cnt = 0;
	
	}

	deltaS_sum_pre = tmp;
	printk("AW9163 TouchOn: %d, rev_rel_flag: %d\n",Touch_on, rev_rel_flag);
}

//////////////////////////////////////////////////////////////////////
/* 	Press_Proc Function:											*/
/* 	Count On_cnt and Clear Off_cnt 								*/
//////////////////////////////////////////////////////////////////////
void Press_proc(void)
{
	int i;
	int Touch_flag;

	Touch_flag = 0;
	for (i=0; i<=PRESS_DEBOUNCE_TH; i++)
	{
		Touch_flag += X_buff[i]%2;
	}

	if ( Press_flag == 0 )		// OFF state
	{
		On_cnt = 0;
		Slide_flag = 0;
		first_slide = 0;
		Nr = 0;

		//if ( (X_buff[2]%2)==1 && (X_buff[1]%2)==1 && (X_buff[0]%2)==1 )
		if ( Touch_flag==(PRESS_DEBOUNCE_TH+1) )
		{
			Press_flag = 1;
			Off_cnt    = 0;
		}
		else
		{
			Off_cnt = (Off_cnt>=SAT_VAL) ? SAT_VAL :( ++Off_cnt);
			for ( i=0; i < 6; i++ ) {
				if ( deltaS_composate[i]>deltaS_raw[i]) {
					deltaS_composate[i] = (deltaS_raw[i]<0) ? 0 : deltaS_raw[i];
				}
			}
		}
	}
	else						// ON state
	{
		Off_cnt = 0;
		//if ( (X_buff[2]%2)==0 && (X_buff[1]%2)==0 && (X_buff[0]%2)==0 )
		if ( Touch_flag==0 || rev_rel_flag==1 )
		{
			Press_flag = 0;
			key_click_cnt++;	//  finger leaving
			On_cnt     = 0;
			
			for ( i=0; i < 6; i++ ) 
			{
				if (deltaS_raw[i] > 0)
					deltaS_composate[i] = deltaS_raw[i];
				else
					deltaS_composate[i] = 0;
			}
		}
		else
		{
			On_cnt = (On_cnt>SAT_VAL) ? SAT_VAL : (++On_cnt);
			
		}
	}
}
//////////////////////////////////////////////////////////////////////
/* 	Release_Proc Function:											*/
/* 	Count Off_cnt and Clear On_cnt 									*/
/* 	Count Tap times counter 										*/
//////////////////////////////////////////////////////////////////////
void Release_proc(void) { }

void Bl_trace(void)
{
	int i;

	for ( i=0; i < 6; i++ )
	{
		if ( ABS(deltaS_raw[i], 0) > NOISE_TH )
		{
			if ( deltaS_raw[i] > 0 )	{ Bl_trace_cnt[i]++; }
			else { Bl_trace_cnt[i]--;	}

			if ( Bl_trace_cnt[i] > UP_TRACE_TH )
			{
				Bl[i] = Bl[i] + UP_TRACE_STEP;
				Bl_trace_cnt[i] = 0;
			}
			else if ( Bl_trace_cnt[i] < DOWN_TRACE_TH )
			{
				Bl[i] = Bl[i] - DOWN_TRACE_STEP;
				Bl_trace_cnt[i] = 0;
			}
		}
	}
}

//////////////////////////////////////////////////////////////////////
// 	Coordinate_proc Function:			
// 	Calculate Current Frame Touch coordinate 
//////////////////////////////////////////////////////////////////////
void Coordinate_proc(void)
{

	int delta_tmp[7];
	int i;
	long sum_tmp = 0;

    for (i=0; i<6; i++) {

		delta_tmp[i] = deltaS[i];
		sum_tmp += delta_tmp[i];
	}

    if ( Touch_on )
    {	
			X_cur =(int)(INTERPOLATION_X * (delta_tmp[2] + delta_tmp[4] +2*(delta_tmp[0] + delta_tmp[1]))/sum_tmp ); //chenqi
			Y_cur =(int)(INTERPOLATION_Y * (delta_tmp[0] + delta_tmp[4] +   delta_tmp[5])/sum_tmp);	
	 }
	else 
	{
		X_cur = X_cur;
		Y_cur = Y_cur;

	}

	for (i=15; i>=0; i--)
	{
		if (i==0) {
			X_buff[0] = X_cur*2 + Touch_on;
			Y_buff[0] = Y_cur*2 + Touch_on;
		}
		else {
			X_buff[i] = X_buff[i-1];
			Y_buff[i] = Y_buff[i-1];
		}
	}
	printk("AW9163 Comp Pos: %d, %d\n", X_cur, Y_cur);
}

void De_jitter(void)
{
	int X_last[16], Y_last[16];
	int i;
	int X_delta, Y_delta, XY_delta;
	int X_tmp, Y_tmp;
	int tmp;

	unsigned char w[3][5] = {
								{ 5,3,0,0,3 },			// 2
								{ 8,5,3,0,4 },			// 3
								{ 6,4,3,3,4 }			// 4
							};
	unsigned char w1[4][5] = {
								{ 5,3,0,0,3 },			// debounce time 1
								{ 8,5,3,0,4 },			// debounce time 2
								{ 6,4,3,3,4 },			// debounce time 3
								{ 6,4,3,3,4 }			// debounce time 4
							};


	if ( Nr < (PRESS_DEBOUNCE_TH+4) ) Nr++;

	if ( Nr < PRESS_DEBOUNCE_TH)
	{
		for(i=0; i<PRESS_DEBOUNCE_TH; i++)
		{
			X_buff[i+1] = X_buff[0];
			Y_buff[i+1] = Y_buff[0];
		}
	}



	for (i=0; i<15; i++)
	{
		X_last[i] = X_buff[i]/2;
		Y_last[i] = Y_buff[i]/2;
	}

	if ( Nr > 2 )	
	{
		XY_delta = ABS( (X_last[PRESS_DEBOUNCE_TH] + Y_last[PRESS_DEBOUNCE_TH]),(X_last[PRESS_DEBOUNCE_TH+1] + Y_last[PRESS_DEBOUNCE_TH+1]) );
		if ( XY_delta > MAX_MOVING ) {
			Nr = 0;
		}
	}

	if ( Nr==0)
	{
		X_rpt   = X_pre;
		Y_rpt   = Y_pre;
	}
	else if ( Nr < (PRESS_DEBOUNCE_TH+2))
	{
		X_pre   = X_last[PRESS_DEBOUNCE_TH];
		Y_pre   = Y_last[PRESS_DEBOUNCE_TH];

		X_rpt   = X_last[PRESS_DEBOUNCE_TH];
		Y_rpt   = Y_last[PRESS_DEBOUNCE_TH];
	}
	else
	{
		X_tmp = 0;
		Y_tmp = 0;
		for (i = 0; i < (Nr-PRESS_DEBOUNCE_TH); i++)
		{
			X_tmp = X_tmp + w[Nr-2-PRESS_DEBOUNCE_TH][i] * X_last[i+PRESS_DEBOUNCE_TH];
			Y_tmp = Y_tmp + w[Nr-2-PRESS_DEBOUNCE_TH][i] * Y_last[i+PRESS_DEBOUNCE_TH];
		}


		X_rpt = X_tmp >> w[Nr-2-PRESS_DEBOUNCE_TH][4];
		Y_rpt = Y_tmp >> w[Nr-2-PRESS_DEBOUNCE_TH][4];
	}

					
	printk("AW9163 Nr: %d, Filter Pos: %d, %d\n", Nr, X_rpt, Y_rpt);

	if ( Slide_st == 0 )		// first slide 
	{
		Slide_over = 0;

		if ( (ABS(X_rpt,X_pre)+ABS(Y_rpt,Y_pre)) > MV_START_DIS )
		{
			 first_slide = 1;
			 Slide_st    = 1;
		}
		else
		{
			X_rpt =X_pre;
			Y_rpt =Y_pre;
		}
	}
	else
	{
		if ( (ABS(X_rpt,X_pre)+ABS(Y_rpt,Y_pre)) < MV_STOP_DIS )
		{
			Slide_over++;

			if(Slide_over==2) 
			{ 
#ifdef MV_TAIL_DBS
				Slide_st = 0;
#endif

				X_rpt =X_pre; 
				Y_rpt =Y_pre;
			}
		}
	}
	
	if(Slide_st == 0)
	{
		long_touch_cnt ++;
		if(long_touch_cnt > LONG_TOUCH_TIME)
		{
			long_touch_cnt = 0;
			Initial_busy = 1; 
		}
	}
	else
	{
		long_touch_cnt  = 0;	
	}

	X_pre=X_rpt;
	Y_pre=Y_rpt;

	printk("AW9163 Rpt Pos: %d, %d\n", X_rpt, Y_rpt);


}

///////////////////////////////////////////////////
// Function : Points Filter
// store pos_x, pos_y and flags to buff  
///////////////////////////////////////////////////
void Points_filter(void)
{
	unsigned char i;
	for(i=0; i<POS_NUM-1; i++)
	{
		pos[i].flag = pos[i+1].flag;
		pos[i].x = pos[i+1].x;
		pos[i].y = pos[i+1].y;
	}
	pos[POS_NUM-1].flag = (unsigned char)((Press_flag<<1) | first_slide);
	pos[POS_NUM-1].x = X_rpt;
	pos[POS_NUM-1].y = Y_rpt;
}

////////////////////////////////////////////////////
//
// Function : long touch press report
//
////////////////////////////////////////////////////
void Long_touch_press_rpt(void)
{
	
	printk("AW9163 long touch press rpt \n");
	input_report_key(AW9163_ts->input_dev, KEY_F1, 1); 
	input_sync(AW9163_ts->input_dev);
	//input_report_key(AW9163_ts->input_dev, KEY_F1, 0); 
	//input_sync(AW9163_ts->input_dev);
	
}

////////////////////////////////////////////////////
//
// Function : long touch release report
//
////////////////////////////////////////////////////
void Long_touch_release_rpt(void)
{
	
	 printk("AW9163 long touch release rpt \n");
	 input_report_key(AW9163_ts->input_dev, KEY_F1, 0); 
	 input_sync(AW9163_ts->input_dev);
	 
}

////////////////////////////////////////////////////
//
// Function : single click report
//
////////////////////////////////////////////////////
void Single_click_rpt(void)
{

	printk("AW9163 single click rpt \n");
	input_report_key(AW9163_ts->input_dev, KEY_F2, 1); 
	//input_sync(AW9163_ts->input_dev);
	input_report_key(AW9163_ts->input_dev, KEY_F2, 0); 
	input_sync(AW9163_ts->input_dev);
	
}

////////////////////////////////////////////////////
//
// Function : double click report
//
////////////////////////////////////////////////////
void Double_click_rpt()
{

	printk("AW9163 double click rpt \n");
	input_report_key(AW9163_ts->input_dev, KEY_F3, 1); 
	//input_sync(AW9163_ts->input_dev);
	input_report_key(AW9163_ts->input_dev, KEY_F3, 0); 
	input_sync(AW9163_ts->input_dev);
	
}


////////////////////////////////////////////////////
//
// Function : report event
//             - slide
//             - long touch press
//             - long touch release
//             - single click
//             - double click
//
////////////////////////////////////////////////////
void Key_report(void)
{
	printk("AW9163, On_cnt = %d,   Off_cnt = %d , Click_tim = %d ,slide_flag=%d \n", On_cnt, Off_cnt, Click_times,Slide_flag);

	if( Key_On_cnt >= LONG_PRESS_TIME_TH )     // long_press press
		{
			Long_touch_press_rpt();

			key_click_cnt     = 0;
			Long_press_flag = 1;
		}
		else if((key_click_cnt==1) &&(Key_Off_cnt > OFF_MAX) && (!Long_press_flag))    // single click
		{
			Single_click_rpt();

			key_click_cnt  = 0;
		}
		else if (key_click_cnt==2)    // double click
		{
			//printk("AW9136 double click rpt \n");
			Double_click_rpt();
			key_click_cnt = 0;
		}
	  else if ( Key_Off_cnt > OFF_MAX )   // touch clear 
		{
			if(Long_press_flag==1)    // long_press release
			{
				Long_touch_release_rpt();

				Long_press_flag = 0;

			}

			key_click_cnt     = 0;
			
		 }

}

void Illegal_proc(void)
{
#ifdef ILLEGAL_CMP
	int i;
	for (i = 0; i < 6; i++) deltaS_composate[i] = deltaS_raw[i];
#else
	illegal_touch_cnt ++;
	if(illegal_touch_cnt > ILLEGAL_TOUCH_CNT)
	{
		illegal_touch_cnt = 0;
		Initial_busy = 1; 
	}	
	//printk("aw9163 debug illegal_touch_cnt= %d\n",illegal_touch_cnt);
#endif
}


////////////////////////////////////////////////////
//
// Function : slide distance report
//
////////////////////////////////////////////////////
void Slide_rpt(void)
{

	//printk("AW9163 X_rpt = %d , Y_rpt = %d \n",X_rpt,Y_rpt);
	//printk("AW9163touchon=%d,legal=%d,d1=%d,d2=%d,d3=%d,d4=%d,d5=%d,d6=%d, \n",Touch_on,Legal_touch,deltaS[0],deltaS[1],deltaS[2],deltaS[3],deltaS[4],deltaS[5]);
	printk("AW9163 report: flag=%d, x=%d, y=%d \n",pos[POS_NUM-1-POS_DLY].flag, pos[POS_NUM-1-POS_DLY].x, pos[POS_NUM-1-POS_DLY].y);

	if((pos[POS_NUM-1-POS_DLY].flag == 3) || ((pos[POS_NUM-1-POS_DLY].flag==2) && (pos[POS_NUM-1].flag==3)))
	{
		input_report_key(AW9163_ts->input_dev, BTN_TOUCH, 1);
		input_report_abs(AW9163_ts->input_dev, ABS_X, pos[POS_NUM-1-POS_DLY].x);
		input_report_abs(AW9163_ts->input_dev, ABS_Y, (pos[POS_NUM-1-POS_DLY].y + 200));
		input_sync(AW9163_ts->input_dev);

	}
	else if (pos[POS_NUM-1-POS_DLY].flag == 1)
	{
		input_report_key(AW9163_ts->input_dev, BTN_TOUCH, 0);
		input_sync(AW9163_ts->input_dev);
	}

    }

////////////////////////////////////////////////////
//
// Function : Sleep double click report
//
////////////////////////////////////////////////////
void Sleep_Double_Click_rpt()
{

	printk("AW9163 Sleep double click rpt \n");
	//input_report_key(AW9163_ts->input_dev, KEY_F4, 1); 
	//input_sync(AW9163_ts->input_dev);
	//input_report_key(AW9163_ts->input_dev, KEY_F4, 0); 
	//input_sync(AW9163_ts->input_dev);
	input_report_key(AW9163_ts->input_dev, KPD_PWRKEY_MAP, 1);
	input_report_key(AW9163_ts->input_dev, KPD_PWRKEY_MAP, 0);
	input_sync(AW9163_ts->input_dev);	
}


////////////////////////////////////////////////////
//
// Function : Cap-touch main program @ mobile sleep 
//            wake up after double-click
//
////////////////////////////////////////////////////
void AW_SleepMode_Proc(void)
{
	unsigned int buff1;
	
	buff1=I2C_read_reg(0x2e);
	if(buff1 == 0x10)
	{
		Sleep_Double_Click_rpt();
	}
}


// Function : Cap-touch main pragram @ mobile normal state
//            press/release/slide process
//
////////////////////////////////////////////////////
void AW_NormalMode_Proc(void)
{
		
		Delta_proc();
	
		if(Legal_touch) 
		{ 
			Touch_proc(); 
			Coordinate_proc();
			Press_proc(); 
	
			if(Press_flag)
			{ 
				De_jitter();
			} 
			else 
			{ 
				Bl_trace();
			}	 
			Points_filter();
			
			Key_proc();
			Slide_rpt();
			Key_report(); 

#ifndef ILLEGAL_CMP			
			illegal_touch_cnt = 0;
#endif		
		}
		else
		{
			Illegal_proc();
		}
}

/*AW9163 body end*/


#ifdef AW9163_EINT_SUPPORT

static int AW9163_ts_clear_intr(struct i2c_client *client)
{
    int res;

    res = I2C_read_reg(0x32);
    if(res <= 0)
    {
        goto EXIT_ERR;
    }
    else
    {
        res = 0;
    }

    return res;

EXIT_ERR:
    TS_DBG("AW9163_ts_clear_intr fail\n");
    return 1;
}

////////////////////////////////////////////////////
//
// Function : Interrupt sub-program
//            work in AW_SleepMode_Proc() or
//            AW_NormalMode_Proc()
//
////////////////////////////////////////////////////
static void AW9163_ts_eint_work(struct work_struct *work)
{
    //TS_DBG("AW9163 Eint work \n");
		if(enable_backtouch_flag)
		{
    switch(WorkMode)
    {
        case 1:
            AW_SleepMode_Proc();
            break;

        case 2:
            AW_NormalMode_Proc();
            break;

        default:
            break;
    }
}
/*
	if(suspend_flag == 1 && WorkMode != 1)
	{
		AW_SleepMode();
	}*/
    AW9163_ts_clear_intr(this_client);
    mt_eint_unmask(AW9163_EINT_NUM);
}


void AW9163_ts_eint_func(void)
{
    if(AW9163_ts == NULL)
    {
        return;
    }
  if(suspended == 1){
			suspended = 0;
  }
    schedule_work(&AW9163_ts->eint_work);

}

int AW9163_ts_setup_eint(void)
{

    mt_set_gpio_mode(AW9163_EINT_PIN, GPIO_MODE_00);//eint mode
    mt_set_gpio_dir(AW9163_EINT_PIN, GPIO_DIR_IN);
    mt_set_gpio_pull_enable(AW9163_EINT_PIN, 1);
    mt_set_gpio_pull_select(AW9163_EINT_PIN, GPIO_PULL_UP);

    /*mt65xx_eint_set_sens(AW9163_EINT_NUM, 1);//level
      mt65xx_eint_set_polarity(AW9163_EINT_NUM, CUST_EINT_POLARITY_LOW);
      mt65xx_eint_set_hw_debounce(AW9163_EINT_NUM, 0);
      mt65xx_eint_registration(AW9163_EINT_NUM, 0, CUST_EINT_POLARITY_LOW, AW9163_ts_eint_func, 0);
      mt65xx_eint_unmask(AW9163_EINT_NUM);*/


    mt_eint_set_hw_debounce(AW9163_EINT_NUM, 0);
    mt_eint_registration(AW9163_EINT_NUM, CUST_EINTF_TRIGGER_LOW, AW9163_ts_eint_func, 0);
    mt_eint_unmask(AW9163_EINT_NUM);

    return 0;
}
#else

static void AW9163_ts_work(struct work_struct *work)
{
    switch(WorkMode)
    {
        case 1:
            AW_SleepMode_Proc();
            break;

        case 2:
            AW_NormalMode_Proc();
            break;

        default:
            break;
    }

}

void AW9163_tpd_polling(unsigned long unuse)
{
    struct AW9163_ts_data *data = i2c_get_clientdata(this_client);

    if (!work_pending(&data->pen_event_work))
    {
        queue_work(data->ts_workqueue, &data->pen_event_work);
    }
    data->touch_timer.expires = jiffies + HZ/FRAME_RATE;
    add_timer(&data->touch_timer);
}
#endif

void AW9163_sleep_timer_func(void)
{
    //TS_DBG("AW9163 speaker en=%d,erji=%d \n",mt_get_gpio_out(GPIO_SPEAKER_EN_PIN),GetHeadPhoneState());

    if(!((mt_get_gpio_out(GPIO_SPEAKER_EN_PIN))||(GetHeadPhoneState())))
    {
        TS_DBG("AW9163 change to Sleep Mode \n");
        suspend_flag = 1;

    }

}


////////////////////////////////////////////////////
//
// Function : AW9163 initial @ mobile goto sleep mode
//            enter SleepMode
//
////////////////////////////////////////////////////
static void AW9163_ts_suspend(struct early_suspend *handler)
{


    struct AW9163_ts_data *data = i2c_get_clientdata(this_client);
	
	suspended = 1;
	
		if(WorkMode != 1)
			{
				AW_SleepMode();
				suspend_flag = 1;
			}
    TS_DBG("==AW9163_ts_suspend=\n");
#ifndef AW9163_EINT_SUPPORT+
    del_timer(&data->touch_timer);
#endif
    Initial_busy = 1;

	//	AW9163_ts->touch_timer.expires = jiffies + HZ*4;
	//	add_timer(&AW9163_ts->touch_timer);

}

////////////////////////////////////////////////////
//
// Function : AW9163 initial @ mobile wake up
//            enter NormalMode
//
////////////////////////////////////////////////////
static void AW9163_ts_resume(struct early_suspend *handler)
{
    struct AW9163_ts_data *data = i2c_get_clientdata(this_client);


    if(WorkMode != 2)
    {
        AW_NormalMode();
        suspend_flag = 0;
    }
    TS_DBG("AW9163 WAKE UP!!!");
#ifndef AW9163_EINT_SUPPORT
    data->touch_timer.expires = jiffies + HZ*5;
    add_timer(&data->touch_timer);
#endif
    del_timer(&data->touch_timer);

}

////////////////////////////////////////////////////
//
// Function : AW9163 initial @ mobile power on
//            enter NormalMode directly
//
////////////////////////////////////////////////////
static int AW9163_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{

    struct input_dev *input_dev;
    int err = 0;
    unsigned int reg_value,reg_value1;

    TS_DBG("==AW9163_ts_probe=\n");
    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
        err = -ENODEV;
        goto exit_check_functionality_failed;
    }

    TS_DBG("==kzalloc=");
    AW9163_ts = kzalloc(sizeof(*AW9163_ts), GFP_KERNEL);
    if (!AW9163_ts)	{
        err = -ENOMEM;
        goto exit_alloc_data_failed;
    }

    AW9163_ts_config_pins();

    client->addr = 0x2C;
    client->timing= 400;
    this_client = client;
    i2c_set_clientdata(client, AW9163_ts);
    //sc8810_i2c_set_clk(2,500000);

    TS_DBG("I2C addr=%x", client->addr);

    reg_value = I2C_read_reg(0x00);
    TS_DBG("AW9163 chip ID = 0x%4x", reg_value);

    if(reg_value != 0xb223)
    {
        err = -ENODEV;
        goto exit_create_singlethread;
    }


	memcpy(AW9163_UCF_FILENAME,"/data/AW9136ucf",15);
	memset(pos, 0, sizeof(pos));

#ifdef AW9163_EINT_SUPPORT
    INIT_WORK(&AW9163_ts->eint_work, AW9163_ts_eint_work);
#else
    INIT_WORK(&AW9163_ts->pen_event_work, AW9163_ts_work);

    AW9163_ts->ts_workqueue = create_singlethread_workqueue(dev_name(&client->dev));
    if (!AW9163_ts->ts_workqueue) {
        err = -ESRCH;
        goto exit_create_singlethread;
    }
#endif

    input_dev = input_allocate_device();
    if (!input_dev) {
        err = -ENOMEM;
        dev_err(&client->dev, "failed to allocate input device\n");
        goto exit_input_dev_alloc_failed;
    }

    AW9163_ts->input_dev = input_dev;


    set_bit(EV_ABS, input_dev->evbit);
    set_bit(EV_KEY, input_dev->evbit);
    set_bit(ABS_X, input_dev->absbit);
    set_bit(ABS_Y, input_dev->absbit);
    set_bit(ABS_PRESSURE, input_dev->absbit);
    set_bit(BTN_TOUCH, input_dev->keybit);
    set_bit(INPUT_PROP_DIRECT, input_dev->propbit);
    set_bit(KPD_PWRKEY_MAP, input_dev->keybit);
    	//set_bit(ABS_MT_TRACKING_ID, input_dev->absbit);
    	//set_bit(ABS_MT_TOUCH_MAJOR, input_dev->absbit);
    	//set_bit(ABS_MT_TOUCH_MINOR, input_dev->absbit);
    	//set_bit(ABS_MT_POSITION_X, input_dev->absbit);
    	//set_bit(ABS_MT_POSITION_Y, input_dev->absbit);
    	
    //input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0, 320, 0, 0);
		//input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0, 420, 0, 0);
		//input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, 100, 0, 0);
		//input_set_abs_params(input_dev, ABS_MT_TOUCH_MINOR, 0, 100, 0, 0); 
//    input_set_abs_params(input_dev, ABS_X, 0, 320, 0, 0);
//		input_set_abs_params(input_dev, ABS_Y, 0, 420, 0, 0);
    input_set_abs_params(input_dev, ABS_X, 0, AW9163_LCM_W, 0, 0);
	input_set_abs_params(input_dev, ABS_Y, 0, AW9163_LCM_H, 0, 0);
	__set_bit(EV_SYN, input_dev->evbit);
	
	__set_bit(KEY_F1, input_dev->keybit);
	__set_bit(KEY_F2, input_dev->keybit);
	__set_bit(KEY_F3, input_dev->keybit);
	__set_bit(KEY_F4, input_dev->keybit);
	
    input_dev->name		= AW9163_ts_NAME;		//dev_name(&client->dev)
    err = input_register_device(input_dev);
    if (err) {
        dev_err(&client->dev,
                "AW9163_ts_probe: failed to register input device: %s\n",
                dev_name(&client->dev));
        goto exit_input_register_device_failed;
    }

    TS_DBG("==register_early_suspend =");
    AW9163_ts->early_suspend.level = EARLY_SUSPEND_LEVEL_DISABLE_FB + 1;
    AW9163_ts->early_suspend.suspend = AW9163_ts_suspend;
    AW9163_ts->early_suspend.resume	= AW9163_ts_resume;
    register_early_suspend(&AW9163_ts->early_suspend);


    msleep(50);

    AW9163_create_sysfs(client);

    WorkMode = 2;
    AW_NormalMode();

    reg_value1 = I2C_read_reg(0x01);
    TS_DBG("AW9163 GCR = 0x%4x", reg_value1);

#ifdef AW9163_EINT_SUPPORT
    AW9163_ts_setup_eint();
#else
    AW9163_ts->touch_timer.function = AW9163_tpd_polling;
    AW9163_ts->touch_timer.data = 0;
    init_timer(&AW9163_ts->touch_timer);
    AW9163_ts->touch_timer.expires = jiffies + HZ*5;
    add_timer(&AW9163_ts->touch_timer);
#endif



    AW9163_ts->touch_timer.function = AW9163_sleep_timer_func;
    AW9163_ts->touch_timer.data = 0;
    init_timer(&AW9163_ts->touch_timer);


    TS_DBG("==probe over =\n");
    return 0;

exit_input_register_device_failed:
    input_free_device(input_dev);
exit_input_dev_alloc_failed:
    //free_irq(client->irq, AW9163_ts);
#ifdef AW9163_EINT_SUPPORT
    cancel_work_sync(&AW9163_ts->eint_work);
#else
    cancel_work_sync(&AW9163_ts->pen_event_work);
    destroy_workqueue(AW9163_ts->ts_workqueue);
#endif
exit_create_singlethread:
    TS_DBG("==singlethread error =\n");
    i2c_set_clientdata(client, NULL);
    kfree(AW9163_ts);
exit_alloc_data_failed:
exit_check_functionality_failed:
    //sprd_free_gpio_irq(AW9163_ts_setup.irq);
    return err;
}
/***********************************************************************************************
Name	:

Input	:


Output	:

function	:

 ***********************************************************************************************/
static int __devexit AW9163_ts_remove(struct i2c_client *client)
{

    struct AW9163_ts_data *AW9163_ts = i2c_get_clientdata(client);

    TS_DBG("==AW9163_ts_remove=\n");

    unregister_early_suspend(&AW9163_ts->early_suspend);
    input_unregister_device(AW9163_ts->input_dev);

    //cancel_work_sync(&AW9163_ts->pen_event_work);
    //destroy_workqueue(AW9163_ts->ts_workqueue);

    kfree(AW9163_ts);

    i2c_set_clientdata(client, NULL);
    return 0;
}

static const struct i2c_device_id AW9163_ts_id[] = {
    { AW9163_ts_NAME, 0 },{ }
};


MODULE_DEVICE_TABLE(i2c, AW9163_ts_id);

static struct i2c_board_info __initdata AW9163_i2c_led[] = {
    {
        I2C_BOARD_INFO(AW9163_ts_NAME, AW9163_ts_I2C_ADDR)	/* 0x2c */
    }
};

static struct i2c_driver AW9163_ts_driver = {
    .probe		= AW9163_ts_probe,
    .remove		= __devexit_p(AW9163_ts_remove),
    .id_table	= AW9163_ts_id,
    //.suspend    = AW9163_ts_suspend,
    //.resume     = AW9163_ts_resume,
    .driver	= {
        .name	= AW9163_ts_NAME,
        .owner	= THIS_MODULE,
    },
};

#if 0
//kaka_13_0703 add

static ssize_t show_asp_prog(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value = 1;
    ret_value = sprintf(buf, "Not support %s\n",__func__);
    return ret_value;
}

static ssize_t store_asp_prog(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    TS_DBG("[%s]: Not Support Write Function\n",__func__);
    return size;
}


static ssize_t show_asp_reg(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value = 1;
    ret_value = sprintf(buf, "Not support %s\n",__func__);
    return ret_value;
}

static ssize_t store_asp_reg(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    TS_DBG("[%s]: Not Support Write Function\n",__func__);
    return size;
}

static ssize_t show_reg(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value = 1;
    ret_value = sprintf(buf, "Not support %s\n",__func__);
    return ret_value;
}

static ssize_t store_reg(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    TS_DBG("[%s]: Not Support Write Function\n",__func__);
    return size;
}

static ssize_t show_start(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value = 1;
    ret_value = sprintf(buf, "Not support %s\n",__func__);
    return ret_value;
}

static ssize_t store_start(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    TS_DBG("[%s]: Not Support Write Function\n",__func__);
    return size;
}

static ssize_t show_stop(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value = 1;
    ret_value = sprintf(buf, "Not support %s\n",__func__);
    return ret_value;
}

static ssize_t store_stop(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    TS_DBG("[%s]: Not Support Write Function\n",__func__);
    return size;
}

static ssize_t show_scenario(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value = 1;
    ret_value = sprintf(buf, "Not support %s\n",__func__);
    return ret_value;
}

static ssize_t store_scenario(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    TS_DBG("[%s]: Not Support Write Function\n",__func__);
    return size;
}

static ssize_t show_led(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value = 1;
    ret_value = sprintf(buf, "Not support %s\n",__func__);
    return ret_value;
}

static ssize_t store_led(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    TS_DBG("[%s]: Not Support Write Function\n",__func__);
    return size;
}


static DEVICE_ATTR(asp_prog, 0644, show_asp_prog, store_asp_prog);
static DEVICE_ATTR(asp_reg, 0644, show_asp_reg, store_asp_reg);
static DEVICE_ATTR(reg, 0644, show_reg, store_reg);
static DEVICE_ATTR(start, 0644, show_start, store_start);
static DEVICE_ATTR(stop, 0644, show_stop, store_stop);
static DEVICE_ATTR(scenario, 0644, show_scenario, store_scenario);
static DEVICE_ATTR(led, 0644, show_led, store_led);



static int AW9163_driver_probe(struct platform_device *dev){
    int ret_device_file = 0;

    TS_DBG("** AW9163_driver_probe!! **\n" );
    if((ret_device_file = device_create_file(&(dev->dev), &dev_attr_asp_prog)) != 0) goto exit_error;
    if((ret_device_file = device_create_file(&(dev->dev), &dev_attr_asp_reg)) != 0) goto exit_error;
    if((ret_device_file = device_create_file(&(dev->dev), &dev_attr_reg)) != 0) goto exit_error;
    if((ret_device_file = device_create_file(&(dev->dev), &dev_attr_start)) != 0) goto exit_error;
    if((ret_device_file = device_create_file(&(dev->dev), &dev_attr_stop)) != 0) goto exit_error;
    if((ret_device_file = device_create_file(&(dev->dev), &dev_attr_scenario)) != 0) goto exit_error;
    if((ret_device_file = device_create_file(&(dev->dev), &dev_attr_led)) != 0) goto exit_error;


exit_error:
    return ret_device_file;
}

static int AW9163_driver_remove(struct platform_device *dev){
    TS_DBG("** AW9163_drvier_remove!! **");

    device_remove_file(&(dev->dev), &dev_attr_asp_prog);
    device_remove_file(&(dev->dev), &dev_attr_asp_reg);
    device_remove_file(&(dev->dev), &dev_attr_reg);
    device_remove_file(&(dev->dev), &dev_attr_start);
    device_remove_file(&(dev->dev), &dev_attr_stop);
    device_remove_file(&(dev->dev), &dev_attr_scenario);
    device_remove_file(&(dev->dev), &dev_attr_led);

    return 0;
}


static struct platform_driver AW9163_driver = {
    .probe		= AW9163_driver_probe,
    .remove     = AW9163_driver_remove,
    .driver     = {
        .name = "AW9163",
    },
};

static struct platform_device AW9163_device = {
    .name   = "AW9163",
    .id	    = -1,
};


static int init_AW9163_device(){

    int ret = 0;
    ret = platform_device_register(&AW9163_device);
    if (ret) {
        TS_DBG("**AW9163_mod_init  Unable to driver register(%d)\n", ret);
        goto  fail_2;
    }


    ret = platform_driver_register(&AW9163_driver);
    if (ret) {
        TS_DBG("**AW9163_mod_init  Unable to driver register(%d)\n", ret);
        goto  fail_1;
    }

    return ret;


fail_1:
    platform_driver_unregister(&AW9163_driver);
fail_2:
    platform_device_unregister(&AW9163_device);

    return ret;
}
//kaka_13_0703 end
#endif

/***********************************************************************************************
Name	:

Input	:


Output	:

function	:

 ***********************************************************************************************/
static int __init AW9163_ts_init(void)
{
    int ret;
    TS_DBG("==AW9163_ts_init==\n");
#if 0
    init_AW9163_device();
#endif
    int i2c_num = cust_get_touchkey_hw();
    i2c_register_board_info(i2c_num, AW9163_i2c_led, 1);

    msleep(50);

    ret = i2c_add_driver(&AW9163_ts_driver);
    return ret;
}

/***********************************************************************************************
Name	:

Input	:


Output	:

function	:

 ***********************************************************************************************/
static void __exit AW9163_ts_exit(void)
{
    TS_DBG("==AW9163_ts_exit==\n");
    i2c_del_driver(&AW9163_ts_driver);
}




module_init(AW9163_ts_init);
module_exit(AW9163_ts_exit);

MODULE_AUTHOR("<lijunjiea@AWINIC.com>");
MODULE_DESCRIPTION("AWINIC AW9163 Touch driver");
MODULE_LICENSE("GPL");
