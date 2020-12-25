/* 
 * Copyright 2014 Solomon Systech Ltd. All rights reserved.
 *
 * Author: binkazhang@solomon-systech.com
 * Date: 2014.04.14
 */

#include <linux/interrupt.h>
#include <cust_eint.h>
#include <linux/i2c.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/rtpm_prio.h>
#include <linux/wait.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/gpio.h>

#include <linux/proc_fs.h>
#include <asm/uaccess.h>

//misc
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>

#ifdef MT6575
#include <mach/mt6575_pm_ldo.h>
#include <mach/mt6575_typedefs.h>
#include <mach/mt6575_boot.h>
#endif
#ifdef MT6577
#include <mach/mt6577_pm_ldo.h>
#include <mach/mt6577_typedefs.h>
#include <mach/mt6577_boot.h>
#endif

#include <linux/uaccess.h>
#include <linux/input.h>
#include <linux/input/mt.h>

#include <linux/hrtimer.h>
#include <linux/string.h>
#include <linux/vmalloc.h>

#include <linux/init.h>

#include <linux/bitops.h>
#include <linux/kernel.h>

#include <linux/byteorder/generic.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#include <linux/jiffies.h>

#ifdef TPD_PROXIMITY
#include <linux/hwmsensor.h>
#include <linux/hwmsen_dev.h>
#include <linux/sensors_io.h>
#endif
#include <mach/mt_pm_ldo.h>

#include "tpd.h"
#include "cust_gpio_usage.h"
#include "tpd_custom_ssd60xx.h"
//###############################################

#define ENABLE_INT              1
#define FINGERNO                10
#define MicroTimeTInterupt      25000000// 100Hz - 10,000,000us

//###############################################

#define ADJUST_NEARBYPOINT
#undef  ADJUST_NEARBYPOINT

#define SCREEN_MAX_X    854//960
#define SCREEN_MAX_Y    480//540 

#define CUT_EDGE 
#undef  CUT_EDGE

#define TPD_HAVE_BUTTON 
//#undef  TPD_HAVE_BUTTON


#define KEY_W_H             80,50

#define KEY_0               40,880// 
#define KEY_1               240,880
#define KEY_2               430,880
//#define KEY_3               430,880

#define TPD_KEY_COUNT       3
#define TPD_KEYS            {KEY_MENU,KEY_HOMEPAGE,KEY_BACK}
#define TPD_KEYS_DIM        {{KEY_0,KEY_W_H},{KEY_1,KEY_W_H},{KEY_2,KEY_W_H}} 


extern struct tpd_device* tpd;

//struct i2c_client* ssl_priv.client = NULL;
struct task_struct* ssd60xx_ts_thread = NULL;

static DECLARE_WAIT_QUEUE_HEAD(waiter);

static void ssd60xx_ts_interrupt_handler(void);
static int ssd60xx_ts_event_handler(void* unused);
/*
extern void mt65xx_eint_unmask(unsigned int line);
extern void mt65xx_eint_mask(unsigned int line);
extern void mt65xx_eint_set_hw_debounce(kal_uint8 eintno, kal_uint32 ms);
extern unsigned int mt65xx_eint_set_sens(kal_uint8 eintno, kal_bool sens);
extern void mt65xx_eint_registration(kal_uint8 eintno, kal_bool Dbounce_En, kal_bool ACT_Polarity, void (EINT_FUNC_PTR) (void), kal_bool auto_umask);
*/

#ifdef TPD_HAVE_BUTTON 
static int tpd_keys_local[TPD_KEY_COUNT] = TPD_KEYS;
static int tpd_keys_dim_local[TPD_KEY_COUNT][4] = TPD_KEYS_DIM;
#endif

static int tpd_flag = 0;

#define DEVICE_ID_REG                   0x02
#define VERSION_ID_REG                  0x03
#define EVENT_STATUS                    0x79
#define FINGER01_REG                    0x7C
#define KEY_STATUS                      0xAF   //es3

#define SSD254X_I2C_NAME            "ssd60xx"
//#define SSD254X_I2C_ADDR            (0x4B<<1)
#define SSD254X_I2C_ADDR            (0x4B<<1)  //es6021

struct ssl_ts_priv
{
    int device_id;
    int version_id;
    
    struct i2c_client* client;
    
    int FingerX[FINGERNO];
    int FingerY[FINGERNO];
    int FingerP[FINGERNO];

    int prevFingerX[FINGERNO];
    int prevFingerY[FINGERNO];
    int prevFingerP[FINGERNO];

    int pFingerX[FINGERNO];
    int pFingerY[FINGERNO];
    
    int keys[4];
};

static struct ssl_ts_priv ssl_priv;

int ssd60xx_ts_read_regs(struct i2c_client* client, uint8_t reg, char*buf, int len);
int ssd60xx_ts_read_reg(struct i2c_client* client, uint8_t reg, int ByteNo);
void ssd60xx_ts_write_reg(struct i2c_client* client, uint8_t Reg, unsigned char Data1, unsigned char Data2, int ByteNo);
void ssd60xx_ts_checkfreq(struct i2c_client* client, int prev, int next, int count);

void ssd60xx_ts_device_patch(struct i2c_client* client);
void ssd60xx_ts_device_init(struct i2c_client* client); 
static int ssd60xx_ts_work(void);

int suspend_flag = 0;

//#define MT65XX_POWER_LDO_VGP     MT65XX_POWER_LDO_VGP1
#define TPD_POWER_SOURCE_CUSTOM MT6323_POWER_LDO_VGP1
#define TPD_POWER_SOURCE_1800 MT6323_POWER_LDO_VGP3

static int __devinit tpd_probe(struct i2c_client* client, const struct i2c_device_id* id);
static int tpd_detect(struct i2c_client* client, int kind, struct i2c_board_info* info);
static int __devexit tpd_remove(struct i2c_client* clien);

static const struct i2c_device_id ssd60xx_tpd_id[] ={{SSD254X_I2C_NAME,0}, {}};
static struct i2c_board_info __initdata ssd60xx_i2c_tpd = {I2C_BOARD_INFO(SSD254X_I2C_NAME,  SSD254X_I2C_ADDR>>1)};
static unsigned short force[] = {0,  SSD254X_I2C_ADDR, I2C_CLIENT_END, I2C_CLIENT_END};
static const unsigned short *const forces[] = { force, NULL };

#define SSL_AUTO_UPGRADE

#ifdef SSL_AUTO_UPGRADE
	typedef struct
	{
	 int patch_checksum;
	 int bin_len;	 
	} SSD_UPDAGE_INFO;
	SSD_UPDAGE_INFO ssd_update_info;

#endif


#if VANZO_TOUCHPANEL_GESTURES_SUPPORT
typedef enum
{
    SSD_GST_DISABLED = 0,
    SSD_GST_ENABLED = 1,
    SSD_GST_WAKEUP = 2,
}SSD_GST_T;

#define GST_NO_EVENT	0
#define GST_LEFT_EVENT	1  //
#define GST_RIGHT_EVENT	2  //
#define GST_UP_EVENT	3  //
#define GST_DOWN_EVENT	4  //
#define GST_DOUBLE_CLICK_EVENT	6 //

#define GST_A_EVENT		12
#define GST_B_EVENT		13  //
#define GST_C_EVENT		14
#define GST_D_EVENT		15
#define GST_E_EVENT		16   //
#define GST_F_EVENT		17 
#define GST_G_EVENT		18
#define GST_H_EVENT		19
#define GST_I_EVENT		20
#define GST_J_EVENT		21

#define GST_K_EVENT		22
#define GST_L_EVENT		23
#define GST_M_EVENT		24  //
#define GST_N_EVENT		25
#define GST_O_EVENT		26   //
#define GST_P_EVENT		27
#define GST_Q_EVENT		28
#define GST_R_EVENT		29
#define GST_S_EVENT		30
#define GST_T_EVENT		31

#define GST_U_EVENT		32
#define GST_V_EVENT		33
#define GST_W_EVENT		34 //
#define GST_X_EVENT		35 //
#define GST_Y_EVENT		36
#define GST_Z_EVENT		37

#define GST_0_EVENT		38
#define GST_1_EVENT		39
#define GST_2_EVENT		40
#define GST_3_EVENT		41
#define GST_4_EVENT		42
#define GST_5_EVENT		43
#define GST_6_EVENT		44 //
#define GST_7_EVENT		45 //
#define GST_8_EVENT		46
#define GST_9_EVENT		47
#define GST_a_EVENT     48


//#define GST_EVENT_STATUS              0xD2  //es2
#define GST_EVENT_STATUS              0x9E    //es3
//#define GST_CONTROL_REG				  0xD1   //es2
#define GST_CONTROL_REG				  0x9D   //es3

static SSD_GST_T ssd_gst_status = SSD_GST_DISABLED;

static int ssd_gesture_keys[] = {KEY_RIGHT,KEY_LEFT,KEY_UP,KEY_DOWN,KEY_U,KEY_O,KEY_W,KEY_M,KEY_E,KEY_C,KEY_Z,KEY_S,KEY_V};
#define GESTURE_KEY_CNT	(sizeof( ssd_gesture_keys )/sizeof( ssd_gesture_keys[0] ))

static int ssd_gst_test = 0;
static int ssd_gst_event_handler(void);

static int is_ssd_gst_mode = 0;

static ssize_t ssd_gst_show(struct device *dev,struct device_attribute *attr,char *buf)
{
	ssize_t ret = 0;

	ret = snprintf(buf, PAGE_SIZE, "%d\n", is_ssd_gst_mode);

	return ret;
}

static ssize_t ssd_gst_store(struct device *dev,struct device_attribute *attr,const char *buf, size_t count)
{
	if(count == 0)
		return;

	mt_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);

	if(buf[0] == '1' && is_ssd_gst_mode == 0){
		is_ssd_gst_mode = 1;
	}
	if(buf[0] == '0' && is_ssd_gst_mode == 1){
		is_ssd_gst_mode = 0;
	}
    mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
	printk("is_ssd_gst_mode = %d", is_ssd_gst_mode);
    return count;
}
#ifdef BUILD_CTS
static DEVICE_ATTR(gesture, 0664, ssd_gst_show, ssd_gst_store);
#else
static DEVICE_ATTR(gesture, 0777, ssd_gst_show, ssd_gst_store);
#endif
static struct attribute *ssdgst_attributes[] = {
	&dev_attr_gesture.attr, 
    NULL
};
static struct attribute_group ssdgst_attribute_group = {
    .attrs = ssdgst_attributes
};
#endif

static struct i2c_driver tpd_i2c_driver =
{
    .driver = 
    {
        .name = SSD254X_I2C_NAME,
    }, 
    .probe = tpd_probe, 
    .remove = __devexit_p(tpd_remove), 
    .id_table = ssd60xx_tpd_id, 
    .detect = tpd_detect,
    .address_list = (const unsigned short *) forces,
};

//###############################################################################
#define CONFIG_TOUCHSCREEN_SSL_DEBUG  

#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
static int debug_show = 1;
#define SSL_DEBUG(fmt,arg...) do { if(debug_show){ printk("ssd60xx: "fmt"\n", ##arg); } } while(0)
//#define SSL_DEBUG(fmt,arg...) printk("ssd60xx: "fmt"\n", ##arg)
#else
#define SSL_DEBUG(fmt,arg...) 
#endif
//###############################################################################

//===============================================================================
#define MISC_TP_CHR  //for debug don't remark

#include <linux/miscdevice.h>   


#ifdef MISC_TP_CHR
#define TP_CHR "tp_chr"

static ssize_t ssd_misc_read(struct file* file, char __user* buf, size_t count, loff_t* offset);
static ssize_t ssd_misc_write(struct file* file, const char __user* buf, size_t count, loff_t* offset);
#endif
//===============================================================================

//*******************************************************************************
#define TPD_PROC_DEBUG   //for debug don't remark
//#undef TPD_PROC_DEBUG

#ifdef TPD_PROC_DEBUG
#include <linux/proc_fs.h>
#include <asm/uaccess.h>

static struct proc_dir_entry *ssd_config_proc = NULL;
#define SSD_CONFIG_PROC_FILE "ssd_config"

static struct ChipSetting ssd_config_patch[5120]={{0}};
static struct ChipSetting ssd_config_table[1024]={{0}};
int ssd_config_patch_count=0;
int ssd_config_table_count=0;

static char ssd_receive_buf[25600]={0};
static char ssd_return_str[25600]={0};

#endif
//*******************************************************************************

//*******************************************************************************
#ifdef TPD_PROC_DEBUG
void ssd_config_run_patch(void)
{
    int i;

    mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
    mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
    msleep(10);
    mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);
    msleep(10);
    mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
    msleep(50);
    TPD_DMESG("ssd60xx reset\n");


    for (i = 0; i < ssd_config_patch_count; i++)
    {
        ssd60xx_ts_write_reg(ssl_priv.client, ssd_config_patch[i].Reg, ssd_config_patch[i].Data1, ssd_config_patch[i].Data2, ssd_config_patch[i].No);
    }
    printk("%s ok\n", __func__);
    //read patch checksum
    mdelay(50);
    ssd60xx_ts_write_reg(ssl_priv.client, 0x96, 0x00, 0x01, 2);
    mdelay(50);
    i = ssd60xx_ts_read_reg(ssl_priv.client, 0x97, 2);
}

void ssd_config_run_init(void)
{
    int i;

    for (i = 0; i < ssd_config_table_count; i++)
    {
        ssd60xx_ts_write_reg(ssl_priv.client, ssd_config_table[i].Reg, ssd_config_table[i].Data1, ssd_config_table[i].Data2, ssd_config_table[i].No);
    }
    printk("%s ok\n", __func__);

    //read patch checksum
    mdelay(50);
    ssd60xx_ts_write_reg(ssl_priv.client, 0x96, 0x00, 0x01, 2);
    mdelay(50);
    i = ssd60xx_ts_read_reg(ssl_priv.client, 0x97, 2);

    printk("patch checksum : 0x%04X\n", i);
}

static int ssd_config_read_proc(char *page, char **start, off_t off, int count, int *eof, void *data)
{
    char *ptr = page;

    ptr += sprintf(ptr,"%s\n", ssd_return_str);

    *eof = 1;
    return (ptr - page);
}

#define SSDTOUCH_WRITE              "write"
#define SSDTOUCH_READ               "read"
#define SSDTOUCH_READS              "reads"

#define SSDTOUCH_PATCHCLEAR         "patchclear"
#define SSDTOUCH_PATCHADDS          "patchadds"
#define SSDTOUCH_PATCHADD           "patchadd"
#define SSDTOUCH_PATCH              "patch"

#define SSDTOUCH_INITCLEAR          "initclear"
#define SSDTOUCH_INITADDS           "initadds"
#define SSDTOUCH_INITADD            "initadd"
#define SSDTOUCH_INIT               "init"

#define SSDTOUCH_DEBUGSHOW          "debugshow"
#define SSDTOUCH_GESTURE            "gstflag"
#define SSDTOUCH_GESTURE_MODE  			"gstmode"

static int ssd_config_write_proc(struct file *file, const char *buffer, unsigned long count, void *data)
{
    char *value=NULL;
    unsigned int Reg=0;
    unsigned int Data1=0;
    unsigned int Data2=0;
    unsigned int Count=0;
    unsigned int ret=0;
    int i=0;
    char tmp_str[256]={0};

    memset(ssd_receive_buf, 0x00, sizeof(ssd_receive_buf));
    if(copy_from_user(ssd_receive_buf, buffer, count))
    {
        printk("copy from user fail\n");
        sprintf(ssd_return_str, "%s error!\n","copy_from_user");
        return -EFAULT;
    }


    if ((value=strstr(ssd_receive_buf, SSDTOUCH_WRITE)) != NULL)//write register
    {
        value += strlen(SSDTOUCH_WRITE);
        if (strlen(value)<14)
        {
            sprintf(ssd_return_str, "%s error!\n",SSDTOUCH_WRITE);
            return -EFAULT;
        }
        value += strlen(" ");
        sscanf(value, "0x%x 0x%x 0x%x", &Reg, &Data1, &Data2);

        ssd60xx_ts_write_reg(ssl_priv.client, Reg, Data1, Data2, 2);

        memset(ssd_return_str, 0 ,sizeof(ssd_return_str));
        sprintf(ssd_return_str, "write [0x%02X]=0x%02X%02X\n", Reg, Data1, Data2);
    }
    else if ((value=strstr(ssd_receive_buf, SSDTOUCH_READS)) != NULL) //reads register
    {
        value += strlen(SSDTOUCH_READS);
        if (strlen(value)<4)
        {
            sprintf(ssd_return_str, "%s error!\n",SSDTOUCH_READS);
            return -EFAULT;
        }

        value += strlen(" ");
        sscanf(value, "0x%x", &Reg);

        value += 5;
        sscanf(value, "0x%x", &Count);

        memset(ssd_return_str, 0 ,sizeof(ssd_return_str));
        for (i = 0; i < Count; i++)
        {
            ret = ssd60xx_ts_read_reg(ssl_priv.client, Reg+i, 2);
            memset(tmp_str, 0 ,sizeof(tmp_str));
            sprintf(tmp_str, " 0x%04X", ret);
            strcat(ssd_return_str, tmp_str);
        }
        strcat(ssd_return_str, "\n");
    }
    else if ((value=strstr(ssd_receive_buf, SSDTOUCH_READ)) != NULL) //read register
    {
        value += strlen(SSDTOUCH_READ);
        if (strlen(value)<4)
        {
            sprintf(ssd_return_str, "%s error!\n",SSDTOUCH_READ);
            return -EFAULT;
        }

        value += strlen(" ");
        sscanf(value, "0x%x", &Reg);

        ret = ret = ssd60xx_ts_read_reg(ssl_priv.client, Reg, 2);

        memset(ssd_return_str, 0 ,sizeof(ssd_return_str));
        sprintf(ssd_return_str, "0x%04X\n", ret);
    }
    else if ((value=strstr(ssd_receive_buf, SSDTOUCH_PATCHCLEAR)) != NULL) //patch clear
    {
        ssd_config_patch_count = 0;

        memset(ssd_return_str, 0 ,sizeof(ssd_return_str));
        sprintf(ssd_return_str, "%s ok!\n",SSDTOUCH_PATCHCLEAR);
    }
    else if ((value=strstr(ssd_receive_buf, SSDTOUCH_PATCHADDS)) != NULL) //patch add
    {
        value += strlen(SSDTOUCH_PATCHADDS);
        if (strlen(value)<14)
        {
            sprintf(ssd_return_str, "%s error!\n",SSDTOUCH_PATCHADDS);
            return -EFAULT;
        }
        value += strlen(" ");
        sscanf(value, "0x%x", &Count);
        
        value +=5;
        if (strlen(value) != Count*15)
        {
            sprintf(ssd_return_str, "%s Count=%d strlen(value)=%d count error!\n",SSDTOUCH_PATCHADDS, Count, strlen(value));
            return -EFAULT;
        }
        
       memset(ssd_return_str, 0 ,sizeof(ssd_return_str));
       for (i = 0; i < Count; i++)
        {
            sscanf(value, " 0x%x 0x%x 0x%x", &Reg, &Data1, &Data2);
            value +=15;

            ssd_config_patch[ssd_config_patch_count].No = 2;
            ssd_config_patch[ssd_config_patch_count].Reg = Reg;
            ssd_config_patch[ssd_config_patch_count].Data1 = Data1;
            ssd_config_patch[ssd_config_patch_count].Data2 = Data2;
            ssd_config_patch_count++;
    
            memset(tmp_str, 0 ,sizeof(tmp_str));
            sprintf(tmp_str, "%s Patch[%04d]= {%d,0x%02X,0x%02X,0x%02X} ok!\n", SSDTOUCH_PATCHADDS, ssd_config_patch_count-1, 2, Reg, Data1, Data2);
            strcat(ssd_return_str, tmp_str);
        }
    }
    else if ((value=strstr(ssd_receive_buf, SSDTOUCH_PATCHADD)) != NULL) //patch add
    {
        value += strlen(SSDTOUCH_PATCHADD);
        if (strlen(value)<14)
        {
            sprintf(ssd_return_str, "%s error!\n",SSDTOUCH_PATCHADD);
            return -EFAULT;
        }
        value += strlen(" ");
        
        sscanf(value, "0x%x 0x%x 0x%x", &Reg, &Data1, &Data2);

        ssd_config_patch[ssd_config_patch_count].No = 2;
        ssd_config_patch[ssd_config_patch_count].Reg = Reg;
        ssd_config_patch[ssd_config_patch_count].Data1 = Data1;
        ssd_config_patch[ssd_config_patch_count].Data2 = Data2;
        ssd_config_patch_count++;

        memset(ssd_return_str, 0 ,sizeof(ssd_return_str));
        sprintf(ssd_return_str, "%s Patch[%04d]= {%d,0x%02X,0x%02X,0x%02X} ok!\n", SSDTOUCH_PATCHADD, ssd_config_patch_count-1, 2, Reg, Data1, Data2);
    }
    else if ((value=strstr(ssd_receive_buf, SSDTOUCH_PATCH)) != NULL)
    {
        ssd_config_run_patch();

        memset(ssd_return_str, 0 ,sizeof(ssd_return_str));
        sprintf(ssd_return_str, "%s ok!\n",SSDTOUCH_PATCH);
    }
    else if ((value=strstr(ssd_receive_buf, SSDTOUCH_INITCLEAR)) != NULL)
    {
        ssd_config_table_count = 0;

        memset(ssd_return_str, 0 ,sizeof(ssd_return_str));
        sprintf(ssd_return_str, "%s ok!\n",SSDTOUCH_INITCLEAR);
    }
    else if ((value=strstr(ssd_receive_buf, SSDTOUCH_INITADDS)) != NULL)
    {
        value += strlen(SSDTOUCH_INITADDS);
        if (strlen(value)<14)
        {
            sprintf(ssd_return_str, "%s error!\n",SSDTOUCH_INITADDS);
            return -EFAULT;
        }
        value += strlen(" ");
        sscanf(value, "0x%x", &Count);
        
        value +=5;
        if (strlen(value) != Count*15)
        {
            sprintf(ssd_return_str, "%s Count=%d strlen(value)=%d count error!\n",SSDTOUCH_PATCHADDS, Count, strlen(value));
            return -EFAULT;
        }
        
        memset(ssd_return_str, 0 ,sizeof(ssd_return_str));
        for (i = 0; i < Count; i++)
        {
            sscanf(value, " 0x%x 0x%x 0x%x", &Reg, &Data1, &Data2);
            value +=15;

            ssd_config_table[ssd_config_table_count].No = 2;
            ssd_config_table[ssd_config_table_count].Reg = Reg;
            ssd_config_table[ssd_config_table_count].Data1 = Data1;
            ssd_config_table[ssd_config_table_count].Data2 = Data2;
            ssd_config_table_count++;
    
            memset(tmp_str, 0 ,sizeof(tmp_str));
            sprintf(tmp_str, "%s Init[%04d]= {%d,0x%02X,0x%02X,0x%02X} ok!\n", SSDTOUCH_INITADDS, ssd_config_table_count-1, 2, Reg, Data1, Data2);
            strcat(ssd_return_str, tmp_str);

        }
    }
    else if ((value=strstr(ssd_receive_buf, SSDTOUCH_INITADD)) != NULL)
    {
        value += strlen(SSDTOUCH_INITADD);
        if (strlen(value)<14)
        {
            sprintf(ssd_return_str, "%s error!\n",SSDTOUCH_INITADD);
            return -EFAULT;
        }
        value += strlen(" ");
        sscanf(value, "0x%x 0x%x 0x%x", &Reg, &Data1, &Data2);

        ssd_config_table[ssd_config_table_count].No = 2;
        ssd_config_table[ssd_config_table_count].Reg = Reg;
        ssd_config_table[ssd_config_table_count].Data1 = Data1;
        ssd_config_table[ssd_config_table_count].Data2 = Data2;
        ssd_config_table_count++;

        memset(ssd_return_str, 0 ,sizeof(ssd_return_str));
        sprintf(ssd_return_str, "%s Init[%04d]= {%d,0x%02X,0x%02X,0x%02X} ok!\n", SSDTOUCH_INITADD, ssd_config_table_count-1, 2, Reg, Data1, Data2);
    }
    else if ((value=strstr(ssd_receive_buf, SSDTOUCH_INIT)) != NULL)
    {
        ssd_config_run_init();

        memset(ssd_return_str, 0 ,sizeof(ssd_return_str));
        sprintf(ssd_return_str, "%s ok!\n",SSDTOUCH_INIT);
    }
    else if ((value=strstr(ssd_receive_buf, SSDTOUCH_DEBUGSHOW)) != NULL)
    {
#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
        value += strlen(SSDTOUCH_DEBUGSHOW);
        if (strlen(value)<4)
        {
            sprintf(ssd_return_str, "%s error!\n",SSDTOUCH_DEBUGSHOW);
            return -EFAULT;
        }
        value += strlen(" ");
        sscanf(value, "0x%x", &Reg);
        debug_show = Reg;

        memset(ssd_return_str, 0 ,sizeof(ssd_return_str));
        sprintf(ssd_return_str, "%s debug_show=%d ok!\n",SSDTOUCH_DEBUGSHOW, debug_show);
#endif
    }
#ifdef VANZO_TOUCHPANEL_GESTURES_SUPPORT	
	else if ((value=strstr(ssd_receive_buf, SSDTOUCH_GESTURE)) != NULL)
    {
        value += strlen(SSDTOUCH_GESTURE);
        if (strlen(value)<4)
        {
            sprintf(ssd_return_str, "%s error!\n",SSDTOUCH_GESTURE);
            return -EFAULT;
        }
        value += strlen(" ");
        sscanf(value, "0x%x", &Reg);
        ssd_gst_test= Reg;
        
        memset(ssd_return_str, 0 ,sizeof(ssd_return_str));
        sprintf(ssd_return_str, "%s ssd_gst_enable_test=%d ok!\n",SSDTOUCH_GESTURE, ssd_gst_test);
    }else if ((value=strstr(ssd_receive_buf, SSDTOUCH_GESTURE_MODE)) != NULL)
    {
        value += strlen(SSDTOUCH_GESTURE_MODE);
        if (strlen(value)<4)
        {
            sprintf(ssd_return_str, "%s error!\n",SSDTOUCH_GESTURE_MODE);
            return -EFAULT;
        }
        value += strlen(" ");
        sscanf(value, "0x%x", &Reg);
        is_ssd_gst_mode= Reg;
        
        memset(ssd_return_str, 0 ,sizeof(ssd_return_str));
        sprintf(ssd_return_str, "%s ssd_gst_enable_test=%d ok!\n",SSDTOUCH_GESTURE_MODE, is_ssd_gst_mode);
    }
#endif
    return 1;
}
#endif
//*******************************************************************************

    
//===============================================================================
#ifdef MISC_TP_CHR
static const struct file_operations ssd_misc_fops =
{
    //.owner        = THIS_MODULE,
    .read = ssd_misc_read, 
    .write = ssd_misc_write, 
};

static struct miscdevice ssd_misc =
{
    .minor = MISC_DYNAMIC_MINOR, 
    .name = TP_CHR, 
    .fops = &ssd_misc_fops, 
};  

static ssize_t ssd_misc_read(struct file* file, char __user* buf, size_t count, loff_t* offset)
{
    char* kbuf;  
    uint8_t reg;
    int ByteNo;
    int readValue;
    int i;

    kbuf = kmalloc(count, GFP_KERNEL);  

    if (copy_from_user(kbuf, buf, 1))
    {
        printk("no enough memory!\n");  
        return -1;
    }  

    reg = (uint8_t) kbuf[0];
    ByteNo = count;


    readValue = ssd60xx_ts_read_reg(ssl_priv.client, reg, ByteNo);
    for (i = 0; i < ByteNo; i++)
    {
        kbuf[i] = (readValue >> (8 * i)) & 0xff;
    }

    if (copy_to_user(buf, kbuf, count))
    {
        printk("no enough memory!\n");  
        kfree(kbuf);
        return -1;
    }  

    kfree(kbuf);

    return count;
}

static ssize_t ssd_misc_write(struct file* file, const char __user* buf, size_t count, loff_t* offset)
{
    char* kbuf;   

    kbuf = kmalloc(count, GFP_KERNEL);  

    if (copy_from_user(kbuf, buf, count))
    {
        printk("no enough memory!\n");  
        return -1;
    }  

    ssd60xx_ts_write_reg(ssl_priv.client, kbuf[1], kbuf[2], kbuf[3], kbuf[0]);
 
    kfree(kbuf);

    return count;
}
#endif
//===============================================================================
int ssd60xx_ts_read_regs(struct i2c_client* client, uint8_t reg, char*buf, int len )
{
    int ret;

    client->addr = client->addr & I2C_MASK_FLAG | I2C_WR_FLAG | I2C_RS_FLAG;
    buf[0] = reg;
    ret = i2c_master_send(client, (const char *) buf, len << 8 | 1);
    client->addr = client->addr & I2C_MASK_FLAG;
    mdelay(1);

    //SSL_DEBUG("%s\n", (ret < 0)? "i2c_transfer Error !":"i2c_transfer OK !");
    return ret;
}

int ssd60xx_ts_read_reg(struct i2c_client* client, uint8_t reg, int ByteNo)
{
    unsigned char buf[4];
    int ret;

    client->addr = client->addr & I2C_MASK_FLAG | I2C_WR_FLAG | I2C_RS_FLAG;
    buf[0] = reg;
    ret = i2c_master_send(client, (const char *) buf, ByteNo << 8 | 1);
    client->addr = client->addr & I2C_MASK_FLAG;
    mdelay(1);

    //SSL_DEBUG("%s\n", (ret < 0)? "i2c_transfer Error !":"i2c_transfer OK !");

    if (ByteNo == 1)
    {
        return (int) ((unsigned int) buf[0] << 0);
    }
    if (ByteNo == 2)
    {
        return (int) ((unsigned int) buf[1] << 0) | ((unsigned int) buf[0] << 8);
    }
    if (ByteNo == 3)
    {
        return (int) ((unsigned int) buf[2] << 0) | ((unsigned int) buf[1] << 8) | ((unsigned int) buf[0] << 16);
    }
    if (ByteNo == 4)
    {
        return (int) ((unsigned int) buf[3] << 0) | ((unsigned int) buf[2] << 8) | ((unsigned int) buf[1] << 16) | (buf[0] << 24);
    }
    return 0;
}

void ssd60xx_ts_write_reg(struct i2c_client* client, uint8_t Reg, unsigned char Data1, unsigned char Data2, int ByteNo)
{
    unsigned char buf[4];
    int ret;

    if (Reg == 0xFF)
    {
        mdelay(Data1*256 + Data2);
        return;
    }

    buf[0] = Data1;
    buf[1] = Data2;
    buf[2] = 0;
    ret = i2c_smbus_write_i2c_block_data(client, Reg, ByteNo, buf);
    //if ( ret !=0 ) SSL_DEBUG("ssd60xx_ts_write_reg %d\n", ret);
    mdelay(1);

    //SSL_DEBUG("%s\n", (ret < 0)? "i2c_master_send Error!":"i2c_master_send OK!");
}
#ifdef VANZO_TOUCHPANEL_GESTURES_SUPPORT

static int ssd_gst_event_handler(void)
{   
	int gst_status;
	int keyValue;
    int isValuedKey;
    if(ssd_gst_status == SSD_GST_ENABLED)
	{
        /* input_report_key(tpd->dev, KEY_POWER, 1);
           input_sync(tpd->dev);
           input_report_key(tpd->dev, KEY_POWER, 0);
           input_sync(tpd->dev);*/
          
			gst_status = ssd60xx_ts_read_reg(ssl_priv.client, GST_EVENT_STATUS, 2);
		
            SSL_DEBUG("gst_status = %d", gst_status);

					if (gst_status > 0 || ssd_gst_test>0 )
					{				
						isValuedKey = 1;
						if(ssd_gst_test>0) 
							gst_status = ssd_gst_test;
						switch (gst_status){
			
							case GST_C_EVENT:
								 //9:
								keyValue = KEY_C;
								break;
							case GST_E_EVENT:
								keyValue = KEY_E;
								break;
							case GST_O_EVENT:
								 //15:
								 //16:
								 //17:
								keyValue = KEY_O;
								break;
							case GST_S_EVENT:
								keyValue = KEY_S;
								break;
							case GST_Z_EVENT:
								keyValue = KEY_Z;
								break;
							case GST_V_EVENT:
								keyValue = KEY_V;
								break;
							case GST_W_EVENT:
								keyValue = KEY_W;
								break;
							case GST_M_EVENT:
								 //12:
								keyValue = KEY_M;
								break;
							case GST_RIGHT_EVENT:
								keyValue = KEY_RIGHT;
								break;
							case GST_LEFT_EVENT:
								keyValue = KEY_LEFT;
								break;
							case GST_DOWN_EVENT:
								keyValue = KEY_DOWN;
								break;
							case GST_UP_EVENT:
								keyValue = KEY_UP;
								break;
							case GST_U_EVENT:
								keyValue = KEY_U;
								break;
							case GST_DOUBLE_CLICK_EVENT:
								keyValue = KEY_U;
								break;
							default:
								isValuedKey = 0;
								break;
						}
			
						if(isValuedKey){
							SSL_DEBUG("gst key event = %d", keyValue);
				
							input_report_key(tpd->dev, keyValue, 1);
							input_sync(tpd->dev);
							input_report_key(tpd->dev, keyValue, 0);
							input_sync(tpd->dev);
						}
						else{
							SSL_DEBUG("no gst key event, but reported event is  = %d",gst_status );
							ssd_gst_status = SSD_GST_ENABLED;
							ssd60xx_ts_write_reg(ssl_priv.client, GST_CONTROL_REG, 0x00, 0x01, 2);
						}
				}
    }
}
#endif

static int ssd60xx_ts_event_handler(void* unused)
{
    struct sched_param param =
    {
        .sched_priority = RTPM_PRIO_TPD
    };
    sched_setscheduler(current, SCHED_RR, &param);

    SSL_DEBUG("enter ssd60xx_ts_event_handler!\n");

    do
    {
        mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM); 
        set_current_state(TASK_INTERRUPTIBLE); 
        wait_event_interruptible(waiter, tpd_flag != 0);

        tpd_flag = 0;

        set_current_state(TASK_RUNNING);
 #ifdef VANZO_TOUCHPANEL_GESTURES_SUPPORT
		if(ssd_gst_test != 0 || is_ssd_gst_mode)
 		ssd_gst_event_handler();
 #endif 
        ssd60xx_ts_work();
    }
    while (!kthread_should_stop());

    return 0;
}

void ssd60xx_ts_device_patch(struct i2c_client* client)
{
    int i;

    for (i = 0; i < sizeof(ssd60xxcfgPatch) / sizeof(ssd60xxcfgPatch[0]); i++)
    {
        ssd60xx_ts_write_reg(client, ssd60xxcfgPatch[i].Reg, ssd60xxcfgPatch[i].Data1, ssd60xxcfgPatch[i].Data2, ssd60xxcfgPatch[i].No);
        mdelay(1);
    }

    mdelay(200); //binka add
}

void ssd60xx_ts_device_init(struct i2c_client* client)
{
    int i;

    for (i = 0; i < sizeof(ssd60xxcfgTable) / sizeof(ssd60xxcfgTable[0]); i++)
    {
        ssd60xx_ts_write_reg(client, ssd60xxcfgTable[i].Reg, ssd60xxcfgTable[i].Data1, ssd60xxcfgTable[i].Data2, ssd60xxcfgTable[i].No);
        mdelay(1);
    }

    mdelay(200); //binka add

    ssd60xx_ts_write_reg(ssl_priv.client, 0x96, 0x00, 0x01, 2);
    mdelay(50);
    
    i = ssd60xx_ts_read_reg(ssl_priv.client, 0x97, 2);
    mdelay(5);
    printk("ssd60xx Patch CheckSum : 0x%04X\n", i);
}

void ssd60xx_ts_device_reset(void)
{
    mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
    mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
    msleep(10);
    mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
    mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);
    msleep(10);
    mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
    mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
    msleep(50);
    TPD_DMESG("ssd60xx reset\n");
}

#ifdef SSL_AUTO_UPGRADE

#define FILE_READ_TRY	50
#define PATCH_HEADER_OFFSET 32
#define UPDATE_BUF_SIZE 0x5000

static char ssl_bin_file1[128] = {"/data/ssd_tp.bin"};
static char ssl_bin_file2[128] = {"/sdcard/ssd_tp.bin"};


void do_fw_upgrade(char *buffer)
{
   char *buf;
   int i;
   buf = buffer;

	ssd60xx_ts_device_reset();
 	for(i=PATCH_HEADER_OFFSET; i<ssd_update_info.bin_len; )  
 	{
        ssd60xx_ts_write_reg(ssl_priv.client, *(buf+i), *(buf+i+1),*(buf+i+2),2);
		if((i<PATCH_HEADER_OFFSET+10) || (i > ssd_update_info.bin_len-10))
		{
		printk("the count is %d ---- [%02x][%02x%02x]......\n", i, *(buf+i), *(buf+i+1), *(buf+i+2));
		}
		i=i+3;
 	}
}

int ssl_fw_update_proc(void *dir)
{
	struct file  *fp; 
	int i, retry; 
	mm_segment_t fs; 
	char *buf =NULL;

    while (retry++ < 5)
    {
        buf = (char *)kzalloc(UPDATE_BUF_SIZE, GFP_KERNEL);

        if (buf == NULL)
        {
            continue;
        }
        else
        {
            printk("upgrade buffer kzalloc OK\n");
            break;
        }
    }

    if (retry >= 5)
    {
        printk("upgrade buffer kzalloc error -exit upgrade thread\n");
        return ;
    }


	for (i=0; i < FILE_READ_TRY; i++)
	{
   		fs=get_fs();
   		set_fs(KERNEL_DS);
   
  		fp = filp_open(ssl_bin_file1, O_RDONLY, 0); 
  		if (IS_ERR(fp)) { 

			fp = filp_open(ssl_bin_file2, O_RDONLY, 0); 
			if (IS_ERR(fp)) { 
	  			printk("-------------------read fw file error---------------\n"); 
				msleep(3000);
			}
  		} 

		if(!IS_ERR(fp))
  		{ 
  			printk("---------------open fw file ok-----------------------\n"); 
			fp->f_op->llseek(fp, 0, SEEK_SET);
            ssd_update_info.bin_len = fp->f_op->llseek(fp, 0, SEEK_END);
			fp->f_op->llseek(fp, 0, SEEK_SET);
			fp->f_op->read(fp, buf, ssd_update_info.bin_len, &fp->f_pos );
			
  			printk("the upgrade file len is %d\n", ssd_update_info.bin_len); 
            do_fw_upgrade(buf);
  			filp_close(fp, NULL);
			i = FILE_READ_TRY;
  		}
  		set_fs(fs);
	 }
	kfree(buf);
	
  return; 
}

int auto_update_init(void)
{
		struct task_struct *thread = NULL;
		
		thread = kthread_run(ssl_fw_update_proc, (void *)NULL, "ssd_update");
		if (IS_ERR(thread))
		{
			printk("Failed to create update thread.\n");
			return -1;
		}
		return 0;
}

#endif



#ifdef ADJUST_NEARBYPOINT
static void ssd60xx_ts_adjust_nearbypoint(unsigned short i, unsigned short* x_pos, unsigned short* y_pos)
{
    u16 x_delta = 0;
    u16 y_delta = 0;
    int X, Y;
    unsigned short No;

    X = *x_pos;
    Y = *y_pos;

    if ((ssl_priv.FingerX[i] != 0xfff) && (ssl_priv.FingerY[i] = 0xfff))
    {
        x_delta = abs(ssl_priv.pFingerX[No] - X);
        y_delta = abs(ssl_priv.pFingerY[No] - Y);
        //if(!((x_delta <= 5) && (y_delta <= 2)) || ((x_delta <= 2) && (y_delta <= 5)))
        if ((x_delta > 5) || (y_delta > 5))
        {
            ssl_priv.pFingerX[No] = X;
            ssl_priv.pFingerY[No] = Y;
        }
    }
    else
    {
        ssl_priv.pFingerX[No] = X;
        ssl_priv.pFingerY[No] = Y;
    }

    *x_pos = ssl_priv.pFingerX[No];
    *y_pos = ssl_priv.pFingerY[No];
}
#endif

#ifdef CUT_EDGE
static void ssd60xx_ts_cut_edge(int *x, int *y)
{
    #define CUT_EDGE_DISTANCE 5
    int x_new, y_new;

    x_new = *x;
    x_new = x_new < CUT_EDGE_DISTANCE ? CUT_EDGE_DISTANCE : x_new;
    x_new = x_new > (SCREEN_MAX_X-CUT_EDGE_DISTANCE) ? (SCREEN_MAX_X-CUT_EDGE_DISTANCE) : x_new;

    y_new = *y;
    y_new = y_new < CUT_EDGE_DISTANCE ? CUT_EDGE_DISTANCE : y_new;
    y_new = y_new > (SCREEN_MAX_Y-CUT_EDGE_DISTANCE) ? (SCREEN_MAX_Y-CUT_EDGE_DISTANCE) : y_new;
    
    //printk("(%d %d)/(%d %d) -> (%d %d)\n", *x, *y, x_max, y_max, x_new, y_new);

    *x = x_new;
    *y = y_new;
}
#endif

static int ssd60xx_touch_down_up(int i, int xpos, int ypos, int width, int isdown)
{
    if (isdown)
    {
        if (i == 0) SSL_DEBUG("X = %d , Y = %d, W = %d\n", xpos, ypos, width);
        input_report_key(tpd->dev, BTN_TOUCH, 1);
        input_report_abs(tpd->dev, ABS_MT_TRACKING_ID, i);  
        input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, width);
        input_report_abs(tpd->dev, ABS_MT_PRESSURE, width);
        input_report_abs(tpd->dev, ABS_MT_POSITION_X, xpos);
        input_report_abs(tpd->dev, ABS_MT_POSITION_Y, ypos);
        input_mt_sync(tpd->dev);
    
#ifdef TPD_HAVE_BUTTON
#if (defined(MT6575) || defined(mt6577))
        if (FACTORY_BOOT == get_boot_mode() || RECOVERY_BOOT == get_boot_mode())
        {
            tpd_button(xpos, ypos, 1);
        }
#endif
#endif
    }
    else
    {
        input_report_key(tpd->dev, BTN_TOUCH, 0);
        input_mt_sync(tpd->dev);

#ifdef TPD_HAVE_BUTTON
#if (defined(MT6575) || defined(mt6577))
        if (FACTORY_BOOT == get_boot_mode() || RECOVERY_BOOT == get_boot_mode())
        {
            tpd_button(0, 0, 0);
        }
#endif
#endif
    }
}

static int ssd60xx_ts_work(void)
{
    int i;
    unsigned short xpos = 0, ypos = 0, width = 0;
    int FingerInfo;
    int EventStatus;
    int FingerX[FINGERNO];
    int FingerY[FINGERNO];
    int FingerP[FINGERNO];
    int clrFlag = 0;

    unsigned char key=0;
    unsigned char keys[4]={0};
    unsigned char istouchkey=0;

    SSL_DEBUG("enter ssd60xx_ts_work!\n");

    if (suspend_flag==1)
    {
        return ;
    }

#ifdef TPD_HAVE_BUTTON     
    key = ssd60xx_ts_read_reg(ssl_priv.client, KEY_STATUS, 2);
    SSL_DEBUG("KeyStatus=0x%x\n", key);

    keys[0] = (key >> 0) & 0x1;
    keys[1] = (key >> 1) & 0x1;
    keys[2] = (key >> 2) & 0x1;
    keys[3] = (key >> 3) & 0x1;
    
    if (keys[0])
    {
        ssd60xx_touch_down_up(0, KEY_0, 20, 1);
        istouchkey=1;
    }
    else if (ssl_priv.keys[0])
    {
        ssd60xx_touch_down_up(0, KEY_0, 0, 0);
        istouchkey=1;
    }
    
    if (keys[1])
    {
        ssd60xx_touch_down_up(0, KEY_1, 20, 1);
        istouchkey=1;
    }
    else if (ssl_priv.keys[1])
    {
        ssd60xx_touch_down_up(0, KEY_1, 0, 0);
        istouchkey=1;
    }

    if (keys[2])
    {
        ssd60xx_touch_down_up(0, KEY_2, 20, 1);
        istouchkey=1;
    }
    else if (ssl_priv.keys[2])
    {
        ssd60xx_touch_down_up(0, KEY_2, 0, 0);
        istouchkey=1;
    }
    
    ssl_priv.keys[0] = keys[0];
    ssl_priv.keys[1] = keys[1];
    ssl_priv.keys[2] = keys[2];
    ssl_priv.keys[3] = keys[3];

    if (istouchkey)
    {
        input_sync(tpd->dev);
       // return;
    }
#endif

    EventStatus = ssd60xx_ts_read_reg(ssl_priv.client, EVENT_STATUS, 2) >> 4;

    SSL_DEBUG("EventStatus=%.2X!\n",EventStatus);
    for (i = 0; i < FINGERNO; i++)
    {
        if ((EventStatus >> i) & 0x1)
        {
            FingerInfo = ssd60xx_ts_read_reg(ssl_priv.client, FINGER01_REG + i, 4);
            xpos = ((FingerInfo >> 4) & 0xF00) | ((FingerInfo >> 24) & 0xFF);
            ypos = ((FingerInfo >> 0) & 0xF00) | ((FingerInfo >> 16) & 0xFF);
            width = ((FingerInfo >> 0) & 0x0FF)*100/255;    

            if (xpos == 0xFFF)
            {
                // This part is to avoid asyn problem when the finger leaves
                //printk("		ssd60xx_ts_work: Correct %x\n",EventStatus);
                EventStatus = EventStatus & ~(1 << i);
                clrFlag = 1;
            }
        }
        else
        {
            xpos = ypos = 0xFFF;
            width = 0;
            clrFlag = 1;
        }

#ifdef CUT_EDGE
        ssd60xx_ts_cut_edge(&xpos, &ypos);
#endif

#ifdef ADJUST_NEARBYPOINT
        ssd60xx_ts_adjust_nearbypoint(i, &xpos, &ypos);
#endif
        FingerX[i] = xpos;
        FingerY[i] = ypos;
        FingerP[i] = width;

    }

    for (i = 0; i < FINGERNO; i++)
    {
        xpos = FingerX[i];
        ypos = FingerY[i];
        width = FingerP[i];
        if (xpos != 0xFFF)
        {
            ssd60xx_touch_down_up(i, xpos, ypos, width, 1);
        }
        else if (EventStatus == 0)
        {
            ssd60xx_touch_down_up(i, 0, 0, 0, 0);
        }
        ssl_priv.prevFingerX[i] = ssl_priv.FingerX[i];
        ssl_priv.prevFingerY[i] = ssl_priv.FingerY[i];
        ssl_priv.prevFingerP[i] = ssl_priv.FingerP[i];

        ssl_priv.FingerX[i] = xpos;
        ssl_priv.FingerY[i] = ypos;
        ssl_priv.FingerP[i] = width;
    }
    input_sync(tpd->dev);
}

static void ssd60xx_ts_interrupt_handler(void)
{
    SSL_DEBUG("enter ssd60xx_ts_interrupt_handler!\n");

    TPD_DEBUG_PRINT_INT;
    tpd_flag = 1;
    wake_up_interruptible(&waiter);
}

static int tpd_probe(struct i2c_client* client, const struct i2c_device_id* id)
{
    int retval;
    int i;

    SSL_DEBUG("enter tpd_probe!\n");

    ssl_priv.client = client;
#if VANZO_TOUCHPANEL_GESTURES_SUPPORT
		input_set_capability(tpd->dev, EV_KEY, KEY_POWER);
	
		for(i=0; i< GESTURE_KEY_CNT; i++)
		{
			input_set_capability(tpd->dev, EV_KEY, ssd_gesture_keys[i]);
		}
#endif

#ifdef MT6572
    //power on, need confirm with SA
    hwPowerOn(MT65XX_POWER_LDO_VGP2, VOL_2800, "TP");
    hwPowerOn(MT65XX_POWER_LDO_VGP, VOL_1800, "TP");
#endif

#ifdef MT6577
    //power on, need confirm with SA
    hwPowerOn(MT65XX_POWER_LDO_VGP2, VOL_2800, "TP");
    hwPowerOn(MT65XX_POWER_LDO_VGP, VOL_1800, "TP");
#endif

#ifdef GPIO_CTP_EN_PIN
    mt_set_gpio_mode(GPIO_CTP_EN_PIN, GPIO_CTP_EN_PIN_M_GPIO);
    mt_set_gpio_dir(GPIO_CTP_EN_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_CTP_EN_PIN, GPIO_OUT_ONE);
#endif

		#ifdef TPD_POWER_SOURCE_CUSTOM                           
        hwPowerOn(TPD_POWER_SOURCE_CUSTOM, VOL_2800, "TP");
    #endif
    #ifdef TPD_POWER_SOURCE_1800
        hwPowerOn(TPD_POWER_SOURCE_1800, VOL_1800, "TP");
    #endif

    for (i = 0; i < FINGERNO; i++)
    {
        ssl_priv.pFingerX[i] = 0xFFF;
        ssl_priv.pFingerY[i] = 0xFFF;
    }
//lichengcheng 
    mt_eint_set_hw_debounce(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_DEBOUNCE_CN);
		mt_eint_registration(CUST_EINT_TOUCH_PANEL_NUM, EINTF_TRIGGER_FALLING, ssd60xx_ts_interrupt_handler, 1);
    mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
//lichengcheng

    mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
    mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
    msleep(5);
    mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
    mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);
    msleep(2);
    mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
    mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
    msleep(5);
    TPD_DMESG("ssd60xx reset\n");

    ssl_priv.device_id = ssd60xx_ts_read_reg(ssl_priv.client, DEVICE_ID_REG, 2);
    ssl_priv.version_id = ssd60xx_ts_read_reg(ssl_priv.client, VERSION_ID_REG, 2);
    printk("ssd60xx Device ID  : 0x%04X\n", ssl_priv.device_id);
    printk("ssd60xx   Version ID : 0x%04X\n", ssl_priv.version_id);
	//lichengcheng
		if (ssl_priv.device_id==0)
		{
        printk("%s: could not get device id.\n", __func__);
        return -1;
		} 
//lichengcheng
    ssd60xx_ts_device_patch(ssl_priv.client);
    ssd60xx_ts_device_init(ssl_priv.client);

    ssd60xx_ts_thread = kthread_run(ssd60xx_ts_event_handler, 0, TPD_DEVICE);
    if (IS_ERR(ssd60xx_ts_thread))
    {
        retval = PTR_ERR(ssd60xx_ts_thread);
        TPD_DMESG(TPD_DEVICE "failed to create kernel thread: %d\n", retval);
    }       

    tpd_load_status = 1;
#ifdef SSL_AUTO_UPGRADE
	retval = auto_update_init();
	if (retval < 0)
  	{
	  printk("update thread error........");
  	}

#endif

#ifdef VANZO_TOUCHPANEL_GESTURES_SUPPORT
    if ((retval = sysfs_create_group(&client->dev.kobj, &ssdgst_attribute_group))) {
        printk("%s: could not register sysfs \n", __func__);
        return -1;
    }
#endif 
//===============================================================================
#ifdef MISC_TP_CHR
    retval = misc_register(&ssd_misc);  
    if (retval < 0)
    {
        printk("%s: could not register ssd_misc device\n", __func__);
        return -1;
    }
#endif
//===============================================================================

//*******************************************************************************
#ifdef TPD_PROC_DEBUG
    ssd_config_proc = create_proc_entry(SSD_CONFIG_PROC_FILE, 0666, NULL);
    printk("[%s] ssd_config_proc = %x \n",__func__, (int)ssd_config_proc);
    if (ssd_config_proc == NULL)
    {
        printk("create_proc_entry %s failed\n", SSD_CONFIG_PROC_FILE);
        return -2;
    }
    else
    {
        ssd_config_proc->read_proc = ssd_config_read_proc;
        ssd_config_proc->write_proc = ssd_config_write_proc;
    }
#endif
//*******************************************************************************
   SSL_DEBUG("exit tpd_probe, ok!\n");

    return 0;
}

static void tpd_resume(struct early_suspend* h)
{
    SSL_DEBUG("enter tpd_resume!\n");
    int i;

    mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);  

#ifdef VANZO_TOUCHPANEL_GESTURES_SUPPORT
		if(ssd_gst_test || is_ssd_gst_mode)
		{
			if (ssd_gst_status != SSD_GST_WAKEUP)
			{
				SSL_DEBUG("Wakeup by power key!");
			}
			else
			{
				SSL_DEBUG("Wakeup by gesture!");
			}
			ssd_gst_status = SSD_GST_DISABLED;
			for (i = 0; i < sizeof(GST_Resume) / sizeof(GST_Resume[0]); i++)
			{
				ssd60xx_ts_write_reg(ssl_priv.client, GST_Resume[i].Reg, GST_Resume[i].Data1, GST_Resume[i].Data2, GST_Resume[i].No);
			}
		}
		else
		{
#endif
    for (i = 0; i < sizeof(Resume) / sizeof(Resume[0]); i++)
    {
        ssd60xx_ts_write_reg(ssl_priv.client, Resume[i].Reg, Resume[i].Data1, Resume[i].Data2, Resume[i].No);
    }
    //ssd60xx_ts_device_init(ssl_priv.client);
#ifdef VANZO_TOUCHPANEL_GESTURES_SUPPORT
		}
#endif	

    suspend_flag = 0;
#ifdef TPD_HAVE_BUTTON
#if (defined(MT6575) || defined(mt6577))
    if (FACTORY_BOOT == get_boot_mode() || RECOVERY_BOOT == get_boot_mode())
    {
        tpd_button(0, 0, 0);
    }
#endif
#endif
}

static void tpd_suspend(struct early_suspend* h)
{
    SSL_DEBUG("enter tpd_suspend!\n");
    int i;
    suspend_flag = 1;
#ifdef VANZO_TOUCHPANEL_GESTURES_SUPPORT
	if(ssd_gst_test || is_ssd_gst_mode)
	{
		ssd_gst_status = SSD_GST_ENABLED;
	    for (i = 0; i < sizeof(GST_Suspend) / sizeof(GST_Suspend[0]); i++)
	    {
	        ssd60xx_ts_write_reg(ssl_priv.client, GST_Suspend[i].Reg, GST_Suspend[i].Data1, GST_Suspend[i].Data2, GST_Suspend[i].No);
	    }
	}
	else
	{
#endif 
    for (i = 0; i < sizeof(Suspend) / sizeof(Suspend[0]); i++)
    {
        ssd60xx_ts_write_reg(ssl_priv.client, Suspend[i].Reg, Suspend[i].Data1, Suspend[i].Data2, Suspend[i].No);
    }
#ifdef VANZO_TOUCHPANEL_GESTURES_SUPPORT
	}
#endif	

}

static int tpd_remove(struct i2c_client* client)
{
    SSL_DEBUG("enter tpd_remove!\n");

    TPD_DEBUG("TPD removed\n");

//*******************************************************************************
#ifdef TPD_PROC_DEBUG
    printk("%s: remove_proc_entry!\n", __func__);
    remove_proc_entry(SSD_CONFIG_PROC_FILE, NULL);
#endif
//*******************************************************************************

//===============================================================================
#ifdef MISC_TP_CHR
    printk("%s: misc_deregister!\n", __func__);
    misc_deregister(&ssd_misc);  
#endif
//===============================================================================

    return 0;
}

static int tpd_detect(struct i2c_client* client, int kind, struct i2c_board_info* info)
{
    strcpy(info->type, TPD_DEVICE); 
    return 0;
}

static int tpd_local_init(void)
{
    SSL_DEBUG("Solomon SSD254x I2C Touchscreen Driver (Built %s @ %s)\n", __DATE__, __TIME__);

    if (i2c_add_driver(&tpd_i2c_driver) != 0)
    {
        TPD_DMESG("ssd60xx unable to add i2c driver.\n");
        return -1;
    }

    if (tpd_load_status == 0)
    {
        TPD_DMESG("ssd60xx add error touch panel driver.\n");
        i2c_del_driver(&tpd_i2c_driver);
        return -1;
    }

#ifdef TPD_HAVE_BUTTON     
    tpd_button_setting(TPD_KEY_COUNT, tpd_keys_local, tpd_keys_dim_local);// initialize tpd button data
#endif   

    TPD_DMESG("end %s, %d\n", __FUNCTION__, __LINE__);  
    tpd_type_cap = 1;
    return 0;
}

static struct tpd_driver_t tpd_device_driver =
{
    .tpd_device_name = SSD254X_I2C_NAME, 
    .tpd_local_init = tpd_local_init, 
    .suspend = tpd_suspend, 
    .resume = tpd_resume,
    #ifdef TPD_HAVE_BUTTON
    .tpd_have_button = 1,
    #else
    .tpd_have_button = 0,
    #endif      

};

/* called when loaded into kernel */
static int __init tpd_driver_init(void)
{
    int ret;

    i2c_register_board_info(0, &ssd60xx_i2c_tpd, 1);
    if (tpd_driver_add(&tpd_device_driver) < 0)
    {
        TPD_DMESG("add ssd60xx driver failed\n");
    }
    printk("ssd60xx touch panel driver init\n");

    return 0;
}

/* should never be called */
static void __exit tpd_driver_exit(void)
{
    printk("ssd60xx touch panel driver exit\n");
    tpd_driver_remove(&tpd_device_driver);
}


module_init(tpd_driver_init);
module_exit(tpd_driver_exit);

MODULE_AUTHOR("Solomon Systech (ShenZhen) Limited - binkazhang@solomon-systech.com");
MODULE_DESCRIPTION("ssd60xx Touchscreen Driver 1.5");
MODULE_LICENSE("GPL v2");
