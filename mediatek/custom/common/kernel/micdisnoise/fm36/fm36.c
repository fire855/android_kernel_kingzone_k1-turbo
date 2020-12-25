#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/wait.h>
#include <linux/spinlock.h>
#include <linux/ctype.h>

#include <asm/uaccess.h>
#include <asm/io.h>
#include <linux/workqueue.h>
#include <linux/switch.h>
#include <linux/delay.h>

#include <linux/device.h>
#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <asm/uaccess.h>
#include <linux/kthread.h>
#include <linux/time.h>
#include <linux/sbsuspend.h>
#include <linux/earlysuspend.h>
#include <linux/string.h>

#include <mach/mt_typedefs.h>
#include <mach/mt_reg_base.h>
#include <mach/irqs.h>

#include "fm36.h"
#include "fm36_psf.h"
#include <mach/mt_boot.h>
#include <cust_eint.h>
#include <cust_gpio_usage.h>
#include <mach/mt_gpio.h>
#include <mach/mt_pwm.h>
#include <mach/mt_clkmgr.h>

/******************************************************************************
 * configuration
 *******************************************************************************/
/*----------------------------------------------------------------------------*/
#define FM36_DEV_NAME     "fm36"

#define FM36_TAG                  "[MIC/FM36] "
#define FM36_FUN(f)               printk(KERN_ERR FM36_TAG"%s\n", __FUNCTION__)
#define FM36_ERR(fmt, args...)    printk(KERN_ERR FM36_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define FM36_LOG(fmt, args...)    printk(KERN_ERR FM36_TAG fmt, ##args)
#define FM36_DBG(fmt, args...)    printk(KERN_ERR FM36_TAG fmt, ##args)

#define I2C_FLAG_WRITE	0
#define I2C_FLAG_READ	1

static struct i2c_client *fm36_i2c_client = NULL;
static const struct i2c_device_id fm36_i2c_id[] = {{FM36_DEV_NAME,0},{}};
static struct i2c_board_info __initdata i2c_fm36={ I2C_BOARD_INFO("fm36", FM36_I2C_ADDRESS)};

/*----------------------------------------------------------------------------*/
static int fm36_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int fm36_i2c_remove(struct i2c_client *client);
static int fm36_i2c_detect(struct i2c_client *client, struct i2c_board_info *info);
/*----------------------------------------------------------------------------*/
static int fm36_suspend(struct i2c_client *client, pm_message_t msg);
static int fm36_resume(struct i2c_client *client);

extern int return_call_status(void);

static DEFINE_MUTEX(fm36_mutex);
static int intr_flag_value = 0;
static int g_fm36_effect_mode = 0;
static int state = 0;
static int state_pre = 0;
static DEFINE_MUTEX(fm36_mutex_state);
/*----------------------------------------------------------------------------*/
static struct platform_driver fm36_driver;
#ifdef CONFIG_HAS_EARLYSUSPEND
static struct early_suspend fm36_early_suspend_handler =
{
	.level = EARLY_SUSPEND_LEVEL_DISABLE_FB-1,
	.suspend = fm36_suspend,
	.resume  = fm36_resume,
};
#endif
static struct i2c_driver fm36_i2c_driver = {
	.probe      = fm36_i2c_probe,
	.remove     = fm36_i2c_remove,
	.detect     = fm36_i2c_detect,
	.suspend    = fm36_suspend,
	.resume     = fm36_resume,
	.id_table   = fm36_i2c_id,
	.driver = {
		.name           = FM36_DEV_NAME,
	},
};

static struct platform_driver fm36_power_driver;

static int fm36_writel(void)
{
	int res = 0;
	uint8_t wbuf[6] = {0xFC,0xF3,0x6A,0x10,0x00,0x00};
	struct i2c_client *client = fm36_i2c_client;
	mutex_lock(&fm36_mutex);
	res = i2c_master_send(client, wbuf, 6);
	if(res < 0)
	{
		goto EXIT_ERR;
	}
	mutex_unlock(&fm36_mutex);
	return res;
EXIT_ERR:
	mutex_unlock(&fm36_mutex);
	FM36_ERR("fm36_i2c_transfer fail\n");
	return res;
}

static int fm36_i2c_master_write(int reg, int data)
{
	int res = 0;
	uint8_t wbuf[7] = {0xFC,0xF3,0x3B,0x00,0x00,0x00,0x00};
	struct i2c_client *client = fm36_i2c_client;

	mutex_lock(&fm36_mutex);
	wbuf[3]=reg >> 8;
	wbuf[4]=reg & 0xff;
	wbuf[5]=data >> 8;
	wbuf[6]=data & 0xff;
	res = i2c_master_send(client, wbuf, 7);
	if(res < 0)
	{
		goto EXIT_ERR;
	}
	mutex_unlock(&fm36_mutex);
	return res;
EXIT_ERR:
	mutex_unlock(&fm36_mutex);
	FM36_ERR("fm36_i2c_transfer fail\n");
	return res;
}
static int fm36_i2c_master_read(int reg)
{
	int res = 0;
	uint8_t wbuf[5];
	uint8_t rbuf[4];
	int dataH, dataL, dataA;
	struct i2c_client *client =fm36_i2c_client;

	mutex_lock(&fm36_mutex);
	wbuf[0]=0xFC;
	wbuf[1]=0xF3;
	wbuf[2]=0x37;
	wbuf[3]=reg >> 8;
	wbuf[4]=reg & 0xff;
	res = i2c_master_send(client, wbuf, 5);
	if(res < 0)
	{
		goto EXIT_ERR;
	}
	/* Get high byte */
	rbuf[0]=0xfc;
	rbuf[1]=0xf3;
	rbuf[2]=0x60;
	rbuf[3]=0x26;
	res = i2c_master_send(client, rbuf, 4);
	if(res < 0)
	{
		goto EXIT_ERR;
	}
	res = i2c_master_recv(client, rbuf, 1);
	if(res < 0)
	{
		goto EXIT_ERR;
	}
	dataH = rbuf[0];

	/* Get low byte */
	rbuf[0]=0xfc;
	rbuf[1]=0xf3;
	rbuf[2]=0x60;
	rbuf[3]=0x25;
	res = i2c_master_send(client, rbuf, 4);
	if(res < 0)
	{
		goto EXIT_ERR;
	}
	res = i2c_master_recv(client, rbuf, 1);
	if(res < 0)
	{
		goto EXIT_ERR;
	}
	dataL = rbuf[0];

	dataA = dataH;
	dataA = dataA << 8;
	dataA = dataA | dataL;
	mutex_unlock(&fm36_mutex);
	return dataA;
EXIT_ERR:
	mutex_unlock(&fm36_mutex);
	FM36_ERR("fm36_i2c_transfer fail\n");
	return res;


}

static int fm36_para_download(fm36_reg_struct *para,int length)
{
	int i = 0;
	for (i = 0; i < length; i++) {
		fm36_i2c_master_write(para[i].regaddr, para[i].regdata);
	}
}

static void fm36_reset(void)
{
	mt_set_gpio_mode(FM36_RST_PIN, GPIO_MODE_00);
	mt_set_gpio_dir(FM36_RST_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(FM36_RST_PIN, GPIO_OUT_ONE);
	mdelay(1);
	mt_set_gpio_out(FM36_RST_PIN, GPIO_OUT_ZERO);
	mdelay(10);
	mt_set_gpio_out(FM36_RST_PIN, GPIO_OUT_ONE);
	mdelay(15);

	printk("%s\n", __func__);
}
static void fm36_sleep(void)
{
	fm36_i2c_master_write(0x22F9, 0x0001);//PWD by I2C
	mdelay(10);
	printk("%s: OK\n", __func__);
}
/****************************************************************************
    fm36_STATE_EFFECT_8K_HSNS_ENABLE,               //handset mode
	fm36_STATE_EFFECT_8K_HFNS_ENABLE,               //handfree mode
	fm36_STATE_EFFECT_8K_HSWexin_ENABLE,            //QQ  weixin Record mode
	fm36_STATE_EFFECT_8K_HFFFP_ENABLE,              //Con-Call mode
	fm36_STATE_EFFECT_8K_HSRecord_ENABLE,           //FFP Record mode
	fm36_STATE_EFFECT_8K_Record_General_ENABLE,     //General Record mode
*****************************************************************************/
static int fm36_set_mode(FM36_EFFECT_MODETAG mode)
{
	int rc;

	printk("old CEM=%d new CEM=%d\n", g_fm36_effect_mode, mode);

	switch (mode)
	{
	case fm36_STATE_EFFECT_8K_HSNS_ENABLE:
		{
			fm36_para_download(FM36_8K_HSNSon_para,sizeof(FM36_8K_HSNSon_para)/sizeof(fm36_reg_struct));
		}
		break;
	case fm36_STATE_EFFECT_8K_HFNS_ENABLE:
		{
			fm36_para_download(FM36_8K_HFNSon_para,sizeof(FM36_8K_HFNSon_para)/sizeof(fm36_reg_struct));
		}
		break;
	case fm36_STATE_EFFECT_8K_HSWexin_ENABLE:
		{
			fm36_para_download(FM36_Wexin_HSNSon_para,sizeof(FM36_Wexin_HSNSon_para)/sizeof(fm36_reg_struct));
		}
		break;
	case fm36_STATE_EFFECT_8K_HFFFP_ENABLE:
		{
			fm36_para_download(FM36_Recoder_Con_call_para,sizeof(FM36_Recoder_Con_call_para)/sizeof(fm36_reg_struct));
		}
		break;
	case fm36_STATE_EFFECT_8K_HSRecord_ENABLE:
		{
			fm36_para_download(FM36_Record_FFP_para,sizeof(FM36_Record_FFP_para)/sizeof(fm36_reg_struct));
		}
		break;
	case fm36_STATE_EFFECT_8K_Record_General_ENABLE:
		{
			fm36_para_download(FM36_Record_General_para,sizeof(FM36_Record_General_para)/sizeof(fm36_reg_struct));
		}
		break;
	case fm36_STATE_SLEEP:
		{
			fm36_sleep();
		}
		break;
	default:
		printk("not support effect mode %d\n", mode);
		return -1;
	}
	if (mode != fm36_STATE_SLEEP)
	{
		mdelay(50);
		rc = fm36_i2c_master_read(0x22fb);
		if (rc != 0x5a5a) {
			printk("set mode %d failed, result=0x%x\n", mode, rc);
			return -2;
		}
	}
	g_fm36_effect_mode = mode;
	return 0;
}
static int fm36_init(FM36_EFFECT_MODETAG ef_mode)
{
	int err = 0;
	int rc = 0;
    int count = 0;
	CLK_Monitor(2,1,0);
	mt_set_gpio_mode(FM36_RST_PIN, GPIO_MODE_00);
	mt_set_gpio_dir(FM36_RST_PIN, GPIO_DIR_OUT);
__begin:
	mt_set_gpio_out(FM36_RST_PIN, GPIO_OUT_ONE);
	mdelay(1);
	mt_set_gpio_out(FM36_RST_PIN, GPIO_OUT_ZERO);
	mdelay(10);
	mt_set_gpio_out(FM36_RST_PIN, GPIO_OUT_ONE);
	mdelay(15);
	{
		fm36_i2c_master_write(0x2273, 0x0AAA);
		rc = fm36_i2c_master_read(0x2273);
		printk("rc = 0x%x\n", rc);
		if (rc == 0x0AAA)
		{
			printk("detect fm36 ok\n");
		}
		else
		{
			printk("detect fm36 failed\n");
            count++;
            if(count < 5){
                goto __begin;
            }
			return -1;
		}
		//fm36_writel();
		fm36_set_mode(ef_mode);
		//fm36_para_download(FM36_8K_HSNSon_para,sizeof(FM36_8K_HSNSon_para)/sizeof(fm36_reg_struct));
		printk("%s: OK\n", __func__);
		/*
		   while(1)
		   {
		   rc=fm36_i2c_master_read(0x2306);                                                                                                                                         \
		   printk("rc = 0x%x\n", rc);
		   }
		   */
	}
    return 0;

}

/*
int fm36_voice_processor_init(void)
{
	int rc;
	int retry = 2;

	do{
		rc = fm36_set_mode(fm36_STATE_8K_INIT);
		if (rc == 0) {
			break;
		}
	}while(retry--);

	return rc;
}
int fm36_voice_processor_8k_bths(void)
{
	int rc;
	int retry = 2;

	fm36_reset();
	do
	{
	//	rc = fm36_set_mode(fm36_STATE_EFFECT_8K_BLUETOOTH_HEADSET);
		if (rc == 0) {
			break;
		}
	} while(retry--);
	return rc;
}
int fm36_voice_processor_8k_bthf(void)
{
	int rc;
	int retry = 2;

	fm36_reset();
	do
	{
		rc = fm36_set_mode(fm36_STATE_EFFECT_8K_BLUETOOTH_HANDFREE);
		if (rc == 0) {
			break;
		}
	} while(retry--);
	return rc;
}
int fm36_voice_processor_8k_mm_record(void)
{
	int rc;
	int retry = 2;

	fm36_reset();

	do
	{
		rc = fm36_set_mode(fm36_STATE_EFFECT_8K_MEDIA_RECORD);
		if (rc == 0) {
			break;
		}
	}while (retry--);


	return rc;
}

int fm36_voice_processor_16k_hf_noise_suppression(int on)
{
	int rc;
	int retry = 2;

	fm36_reset();

	do
	{
		if (on)
		{
			rc = fm36_set_mode(fm36_STATE_EFFECT_16K_HFNS_ENABLE);
		}
		else
		{
			rc = fm36_set_mode(fm36_STATE_EFFECT_16K_HFNS_DISABLE);
		}

		if (rc == 0) {
			break;
		}
	} while(retry--);

	return rc;
}

int fm36_voice_processor_16k_hf_ffp(void)
{
	int rc;
	int retry = 2;

	fm36_reset();
	do
	{
		rc = fm36_set_mode(fm36_STATE_EFFECT_16K_HFFFP);
		if(rc=0){
			break;
		}
	}while(retry--);
	return rc;
}

int fm36_voice_processor_16k_hs_noise_suppression(int on)
{
	int rc;
	int retry = 2;

	fm36_reset();

	do
	{
		if (on)
		{
			rc = fm36_set_mode(fm36_STATE_EFFECT_16K_HSNS_ENABLE);
		}
		else
		{
			rc = fm36_set_mode(fm36_STATE_EFFECT_16K_HSNS_DISABLE);
		}

		if (rc == 0) {
			break;
		}
	} while(retry--);

	return rc;
}

int fm36_voice_processor_16k_mm_record(void)
{
	int rc;
	int retry = 2;

	fm36_reset();

	do
	{
		rc = fm36_set_mode(fm36_STATE_EFFECT_16K_MEDIA_RECORD);
		if (rc == 0) {
			break;
		}
	}while (retry--);


	return rc;
}

int fm36_voice_processor_16k_hfvr(void)
{
	int rc;
	int retry = 2;

	fm36_reset();

	do
	{
		rc = fm36_set_mode(fm36_STATE_EFFECT_16K_HFVR);
		if (rc == 0) {
			break;
		}
	}while (retry--);


	return rc;
}

int fm36_voice_processor_16k_carkit_vr(void)
{
	int rc;
	int retry = 2;

	fm36_reset();

	do
	{
		rc = fm36_set_mode(fm36_STATE_EFFECT_16K_CARKITVR);
		if (rc == 0) {
			break;
		}
	}while (retry--);


	return rc;
}

int fm36_voice_processor_48K_mm_record(void)
{
	int rc;
	int retry = 2;

	fm36_reset();

	do
	{
		rc = fm36_set_mode(fm36_STATE_EFFECT_48K_MEDIA_RECORD);
		if (rc == 0) {
			break;
		}
	}while (retry--);


	return rc;
}
int  fm36_voice_processor_mic0_test(void)
{
	int rc;
	int retry = 2;

	fm36_reset();

	do {
	//	rc = fm36_set_mode(fm36_STATE_FTM_8K_MIC0TEST);
		if (rc == 0) {
			break;
		}
	}while(retry--);

	return rc;
}

int  fm36_voice_processor_mic1_test(void)
{
	int rc;
	int retry = 2;

	fm36_reset();

	do{
		rc = fm36_set_mode(fm36_STATE_FTM_8K_MIC1_TEST);
		if (rc == 0) {
			break;
		}
	}while(retry--);

	return rc;
}

int  fm36_voice_processor_mic2_test(void)
{
	int rc;
	int retry = 2;

	fm36_reset();

	do{
		rc = fm36_set_mode(fm36_STATE_FTM_8K_MIC2_TEST);
		if (rc == 0) {
			break;
		}
	}while(retry--);

	return rc;
}

*/
int fm36_voice_processor_8k_hs_noise_suppression(void)
{
	int rc;
	int retry = 2;

	fm36_reset();

	do
	{
		rc = fm36_set_mode(fm36_STATE_EFFECT_8K_HSNS_ENABLE);
		if (rc == 0) {
			break;
		}
	} while(retry--);

	return rc;
}

int fm36_voice_processor_8k_hf_noise_suppression(void)
{
	int rc;
	int retry = 2;

	fm36_reset();

	do
	{
		rc = fm36_set_mode(fm36_STATE_EFFECT_8K_HFNS_ENABLE);
		if (rc == 0) {
			break;
		}
	} while(retry--);

	return rc;
}

int fm36_voice_processor_8k_hs_weixin(void)
{
	int rc;
	int retry = 2;

	fm36_reset();
	do
	{
		rc = fm36_set_mode(fm36_STATE_EFFECT_8K_HSWexin_ENABLE);
		if (rc == 0) {
			break;
		}
	} while(retry--);
	return rc;
}
int fm36_voice_processor_8k_hf_ffp(void)
{
	int rc;
	int retry = 2;

	fm36_reset();
	do
	{
		rc = fm36_set_mode(fm36_STATE_EFFECT_8K_HFFFP_ENABLE);
		if (rc == 0) {
			break;
		}
	} while(retry--);
	return rc;
}
int fm36_voice_processor_8k_hs_record(void)
{
	int rc;
	int retry = 2;

	fm36_reset();
	do
	{
		rc = fm36_set_mode(fm36_STATE_EFFECT_8K_HSRecord_ENABLE);
		if (rc == 0) {
			break;
		}
	} while(retry--);
	return rc;
}

int  fm36_voice_processor_sleep(void)
{
	int rc;

	rc = fm36_set_mode(fm36_STATE_SLEEP);

	return rc;
}
/*----------------------------------------------------------------------------*/
static int fm36_i2c_detect(struct i2c_client *client, struct i2c_board_info *info)
{
	strcpy(info->type, FM36_DEV_NAME);
	return 0;
}
static ssize_t fm36_dump_register(struct device_driver *ddri, char *buf)
{
	int i, len = 0;
	switch (state)
	{
	case 1:
		for(i=0;i<(sizeof(FM36_8K_HSNSon_para)/sizeof(fm36_reg_struct));i++)
		{
			len += snprintf(buf+len, PAGE_SIZE-len, "reg: 0x%x  value: 0x%x\n", FM36_8K_HSNSon_para[i].regaddr, fm36_i2c_master_read(FM36_8K_HSNSon_para[i].regaddr));
		}
		break;
	case 2:
		for(i=0;i<(sizeof(FM36_8K_HFNSon_para)/sizeof(fm36_reg_struct));i++)
		{
			len += snprintf(buf+len, PAGE_SIZE-len, "reg: 0x%x  value: 0x%x\n", FM36_8K_HFNSon_para[i].regaddr, fm36_i2c_master_read(FM36_8K_HFNSon_para[i].regaddr));
		}
		break;
	case 3:
		for(i=0;i<(sizeof(FM36_Wexin_HSNSon_para)/sizeof(fm36_reg_struct));i++)
		{
			len += snprintf(buf+len, PAGE_SIZE-len, "reg: 0x%x  value: 0x%x\n", FM36_Wexin_HSNSon_para[i].regaddr, fm36_i2c_master_read(FM36_Wexin_HSNSon_para[i].regaddr));
		}
		break;
	case 4:
		for(i=0;i<(sizeof(FM36_Recoder_Con_call_para)/sizeof(fm36_reg_struct));i++)
		{
			len += snprintf(buf+len, PAGE_SIZE-len, "reg: 0x%x  value: 0x%x\n", FM36_Recoder_Con_call_para[i].regaddr, fm36_i2c_master_read(FM36_Recoder_Con_call_para[i].regaddr));
		}
		break;
	case 5:
		for(i=0;i<(sizeof(FM36_Record_FFP_para)/sizeof(fm36_reg_struct));i++)
		{
			len += snprintf(buf+len, PAGE_SIZE-len, "reg: 0x%x  value: 0x%x\n", FM36_Record_FFP_para[i].regaddr, fm36_i2c_master_read(FM36_Record_FFP_para[i].regaddr));
		}
		break;
	default:
		for(i=0;i<(sizeof(FM36_8K_HSNSon_para)/sizeof(fm36_reg_struct));i++)
		{
			len += snprintf(buf+len, PAGE_SIZE-len, "reg: 0x%x  value: 0x%x\n", FM36_8K_HSNSon_para[i].regaddr, fm36_i2c_master_read(FM36_8K_HSNSon_para[i].regaddr));
		}
		break;
	}
	return len;
}
static ssize_t fm36_debug_write_register(struct device_driver *ddri, char *buf, size_t count)
{
	int addr,cmd;
	if(2 == sscanf(buf,"%x %x",&addr, &cmd))
	{
		printk("fm36_dbg [addr %x,cmd %x,count %d]\n", addr,cmd,count);
		fm36_i2c_master_write(addr,cmd);
		return count;
	}
	return 0;
}
static ssize_t fm36_debug_reset(struct device_driver *ddri, char *buf, size_t count)
{
	int cmd;
	if(1 != sscanf(buf,"%x",&cmd))
	{
		return -1;
	}
	if(cmd == 1)
		fm36_reset();
	return count;
}
static ssize_t fm36_show_state(struct device_driver *ddri, char *buf)
{
    int len = 0;
    len += snprintf(buf, PAGE_SIZE, "%d\n", state);
    return len;
}
static ssize_t fm36_store_state(struct device_driver *ddri, char *buf, size_t count)
{
    if(buf == NULL)
        return 0;
	mutex_lock(&fm36_mutex_state);
    if(buf[0] == '9'){
        switch(state_pre)
        {
            case 0:
                if(0 != return_call_status()){ // in call
                    fm36_init(fm36_STATE_EFFECT_8K_HSNS_ENABLE);
                    state = 2;
                    state_pre = 2;
                }else{//for qq weixin
                    fm36_init(fm36_STATE_EFFECT_8K_HSWexin_ENABLE);
                    state = 6;
                    state_pre = 6;
                }
                break;
            case 1:
                fm36_init(fm36_STATE_EFFECT_8K_HSNS_ENABLE);
                state = 1;
                break;
            case 2:
                fm36_init(fm36_STATE_EFFECT_8K_HFNS_ENABLE);
                state = 2;
                break;
            case 3:
                fm36_init(fm36_STATE_EFFECT_8K_Record_General_ENABLE);
                state = 3;
                break;
            case 4:
                fm36_init(fm36_STATE_EFFECT_8K_HFFFP_ENABLE);
                state = 4;
                break;
            case 5:
                fm36_init(fm36_STATE_EFFECT_8K_HSRecord_ENABLE);
                state = 5;
                break;
            case 6:
                fm36_init(fm36_STATE_EFFECT_8K_HSWexin_ENABLE);
                state = 6;
                break;

            case 12:
                fm36_sleep();
                state = 0;
                state_pre = 0;
                break;
            default:
                break;
        }
	    mutex_unlock(&fm36_mutex_state);
        return count;
    }else if(buf[0] == '0'){
        if(0 != return_call_status()){
            printk("MAMAMA not close !!!");
            mutex_unlock(&fm36_mutex_state);
            return count;
        }
        fm36_sleep();
        state = 0;
        state_pre = 0;
        mutex_unlock(&fm36_mutex_state);
        return count;
    }

    if(state_pre == 0){
        switch(buf[0]){
            case '1':
                state_pre = 1;
                break;
            case '2':
                state_pre = 2;
                break;
            case '3':
                state_pre = 3;
                break;
            case '4':
                state_pre = 4;
                break;
            case '5':
                state_pre = 5;
                break;
            case '6':
                state_pre = 6;
                break;
            default:
                state_pre = 0;
                break;
        }
    }else{

        if(buf[0] == '1'){
            if(state != 1){
                fm36_init(fm36_STATE_EFFECT_8K_HSNS_ENABLE);
                state = 1;
            }
        }else if(buf[0] == '2'){
            if(state != 2){
                fm36_init(fm36_STATE_EFFECT_8K_HFNS_ENABLE);
                state = 2;
            }
        }
    	else if(buf[0] == '3'){
            if(state != 3){
                fm36_init(fm36_STATE_EFFECT_8K_Record_General_ENABLE);
                state = 3;
            }
        }
    	else if(buf[0] == '4'){
            if(state != 4){
                fm36_init(fm36_STATE_EFFECT_8K_HFFFP_ENABLE);
                state = 4;
            }
        }
    	else if(buf[0] == '5'){
            if(state != 5){
                fm36_init(fm36_STATE_EFFECT_8K_HSRecord_ENABLE);
                state = 5;
            }
		}
    	else if(buf[0] == '6'){
            if(state != 6){
                fm36_init(fm36_STATE_EFFECT_8K_HSWexin_ENABLE);
                state = 6;
            }
        }
    	else if(buf[0] == '0'){
        printk("MAMAMA should not to be here !!!\n");
            if(state != 0){
                fm36_sleep();
                state = 0;
                state_pre = 0;
            }
        }

    }
    mutex_unlock(&fm36_mutex_state);
    return count;
}

static DRIVER_ATTR(fm36_reg,S_IWUSR | S_IRUGO,fm36_dump_register,fm36_debug_write_register);
static DRIVER_ATTR(fm36_rst,      S_IWUSR | S_IRUGO, 	NULL,    			   fm36_debug_reset);
static DRIVER_ATTR(fm36_state, 00777, fm36_show_state, fm36_store_state);
/*----------------------------------------------------------------------------*/
static struct device_attribute *fm36_attr_list[] = {
	&driver_attr_fm36_reg,
	&driver_attr_fm36_state,
	&driver_attr_fm36_rst,
};

static int fm36_create_attr(struct device_driver *driver)
{
	int idx, err = 0;
	int num = (int)(sizeof(fm36_attr_list)/sizeof(fm36_attr_list[0]));
	if (driver == NULL)
	{
		return -EINVAL;
	}

	for(idx = 0; idx < num; idx++)
	{
		if(err = driver_create_file(driver, fm36_attr_list[idx]))
		{
			printk("driver_create_file (%s) = %d\n", fm36_attr_list[idx]->attr.name, err);
			break;
		}
	}
	return err;
}

/*----------------------------------------------------------------------------*/
static int fm36_delete_attr(struct device_driver *driver)
{
	int idx ,err = 0;
	int num = (int)(sizeof(fm36_attr_list)/sizeof(fm36_attr_list[0]));

	if (!driver)
		return -EINVAL;

	for (idx = 0; idx < num; idx++)
	{
		driver_remove_file(driver, fm36_attr_list[idx]);
	}

	return err;
}

/*----------------------------------------------------------------------------*/
static int fm36_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int err = 0;
	fm36_i2c_client = client;
	fm36_init(fm36_STATE_EFFECT_8K_HSNS_ENABLE);
    fm36_sleep();
	if(err = fm36_create_attr(&fm36_driver.driver))
	{
		printk("create attribute err = %d\n", err);
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	register_early_suspend(&fm36_early_suspend_handler);
#endif
	printk("%s: OK\n", __func__);
	return 0;
}
/*----------------------------------------------------------------------------*/
static int fm36_i2c_remove(struct i2c_client *client)
{
	int err;
	err = fm36_delete_attr(&fm36_driver.driver);
	if(err)
	{
		printk("fm36 delete_attr fail: %d\n", err);
	}
	return 0;
}
/*----------------------------------------------------------------------------*/
static int fm36_probe(struct platform_device *pdev)
{
	if(i2c_add_driver(&fm36_i2c_driver))
	{
		printk("add driver error\n");
		return -1;
	}
	return 0;
}
/*----------------------------------------------------------------------------*/
static int fm36_remove(struct platform_device *pdev)
{
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&fm36_early_suspend_handler);
#endif
	i2c_del_driver(&fm36_i2c_driver);
	return 0;
}
static int fm36_suspend(struct i2c_client *client, pm_message_t msg)
{
	/*
	printk("fm36_i2c_suspend\n");
	int status=0;
	status=return_call_status();
	//printk("call status = %d\n",status);
	if(!status)
	fm36_sleep();
	*/
	return 0;
}
/*----------------------------------------------------------------------------*/
static int fm36_resume(struct i2c_client *client)
{
	/*
	printk("fm36_resume\n");
	int status=0;
	status=return_call_status();
	printk("call status = %d\n",status);
	if(!status)
	{
		CLK_Monitor(2,1,0);
		fm36_init();
	}
	*/
	return 0;
}

static struct platform_driver fm36_driver = {
	.probe = fm36_probe,
	.suspend = NULL,
	.resume = NULL,
	.remove = fm36_remove,
	.driver = {
		.name = "fm36_driver",
	},
};

static int fm36_mod_init(void)
{
	int ret = 0;
	i2c_register_board_info(1, &i2c_fm36, 1);
	if(platform_driver_register(&fm36_driver) != 0)
	{
		return -1;
	}

	return 0;
}

static void fm36_mod_exit(void)
{
	platform_driver_unregister(&fm36_driver);
}

module_init(fm36_mod_init);
module_exit(fm36_mod_exit);

MODULE_DESCRIPTION("Vanzo fm36 driver");
MODULE_AUTHOR("AL <zhangxinyu@vanzotec.com>");
MODULE_LICENSE("GPL");
