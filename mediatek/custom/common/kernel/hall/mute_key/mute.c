#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/interrupt.h>
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

#include <linux/string.h>

#include <mach/mt_typedefs.h>
#include <mach/mt_reg_base.h>
#include <mach/irqs.h>

#include "mute.h"
#include <mach/mt_boot.h>
#include <cust_eint.h>
#include <cust_gpio_usage.h>
#include <mach/mt_gpio.h>

//#define GPIO_MUTE_EINT_PIN GPIO116	//move to dct
//#define CUST_EINT_MUTE_NUM 11		//move to dct

int mute_cur_eint_state = MUTE_NO;

extern void mt_eint_mask(unsigned int eint_num);                                                                                                                         
extern void mt_eint_unmask(unsigned int eint_num);
extern void mt_eint_set_hw_debounce(unsigned int eint_num, unsigned int ms); 
extern void mt_eint_set_polarity(unsigned int eint_num, unsigned int pol);
extern unsigned int mt_eint_set_sens(unsigned int eint_num, unsigned int sens);
extern void mt_eint_registration(unsigned int eint_num, unsigned int flow, void (EINT_FUNC_PTR)(void), unsigned int is_auto_umask);
extern void mt_eint_print_status(void);

static struct workqueue_struct * mute_eint_workqueue = NULL;
static struct work_struct mute_eint_work;

static struct switch_dev mute_data;

void mute_eint_work_callback(struct work_struct *work)
{
	MUTE_FUNC();
    mt_eint_mask(CUST_EINT_MUTE_NUM);
	if(mute_cur_eint_state == MUTE_YES)
	{
		MUTE_DEBUG("MUTE_YES\n");
        switch_set_state((struct switch_dev *)&mute_data, MUTE_YES);
	}
	else
	{
		MUTE_DEBUG("MUTE_NO\n");
        switch_set_state((struct switch_dev *)&mute_data, MUTE_NO);
	}
    mt_eint_unmask(CUST_EINT_MUTE_NUM);
}

void mute_eint_func(void)
{
	int ret;
	
	MUTE_FUNC();
	if(mute_cur_eint_state ==  MUTE_NO ) 
	{
		mt_eint_set_polarity(CUST_EINT_MUTE_NUM, 1);
		mute_cur_eint_state = MUTE_YES;
	}
	else
	{
		mt_eint_set_polarity(CUST_EINT_MUTE_NUM, 0);
		mute_cur_eint_state = MUTE_NO;
	}

	ret = queue_work(mute_eint_workqueue, &mute_eint_work); 
}

static inline int mute_setup_eint(void)
{
	MUTE_FUNC();
	
	mt_set_gpio_mode(GPIO_MUTE_EINT_PIN, GPIO_MUTE_EINT_PIN_M_EINT);
	mt_set_gpio_dir(GPIO_MUTE_EINT_PIN, GPIO_DIR_IN);
	mt_set_gpio_pull_enable(GPIO_MUTE_EINT_PIN, GPIO_PULL_DISABLE);

    //mt65xx_eint_set_sens(CUST_EINT_MUTE_NUM, CUST_EINT_MUTE_SENSITIVE);
	mt_eint_set_hw_debounce(CUST_EINT_MUTE_NUM,CUST_EINT_MUTE_DEBOUNCE_CN);
	mt_eint_registration(CUST_EINT_MUTE_NUM, mute_cur_eint_state?CUST_EINTF_TRIGGER_LOW:CUST_EINTF_TRIGGER_HIGH,  mute_eint_func, 0);
	mt_eint_unmask(CUST_EINT_MUTE_NUM);  

	return 0;
}

static int mute_probe(struct platform_device *dev)
{
	int ret = 0;
	bool curr_state;
		
	MUTE_FUNC();

	mt_set_gpio_mode(GPIO_MUTE_EINT_PIN, 0);
	mt_set_gpio_dir(GPIO_MUTE_EINT_PIN, GPIO_DIR_IN);
	mt_set_gpio_pull_enable(GPIO_MUTE_EINT_PIN, GPIO_PULL_DISABLE);
	curr_state = mt_get_gpio_in(GPIO_MUTE_EINT_PIN);
	printk("%s line %d curr_state %d\n",__func__,__LINE__, curr_state);

	if(!curr_state){
		mute_data.name = "mute";
		mute_data.index = 0;
		mute_data.state = MUTE_NO;
		mute_cur_eint_state = MUTE_NO;
	}else{
		mute_data.name = "mute";
		mute_data.index = 0;
		mute_data.state = MUTE_YES;
		mute_cur_eint_state = MUTE_YES;
	}

	ret = switch_dev_register(&mute_data);
	if(ret)
	{
		MUTE_DEBUG("switch_dev_register return %d\n", ret);
	}

	mute_eint_workqueue = create_singlethread_workqueue("mute_eint");
	INIT_WORK(&mute_eint_work, mute_eint_work_callback);

	mute_setup_eint();

	return 0;
}

static int mute_remove(struct platform_device *dev)
{
	MUTE_FUNC();

	destroy_workqueue(mute_eint_workqueue);
	switch_dev_unregister(&mute_data);

	return 0;
}

static struct platform_driver mute_driver = {
	.probe = mute_probe,
	.suspend = NULL,
	.resume = NULL,
	.remove = mute_remove,
	.driver = {
		.name = "mute_driver",
	},
};

static int mute_mod_init(void)
{
	int ret = 0;

	MUTE_FUNC();	
	if(platform_driver_register(&mute_driver) != 0)
	{
		MUTE_DEBUG("unable to register mute driver\n");
		return -1;
	}
	return 0;
}

static void mute_mod_exit(void)
{
	MUTE_FUNC();
	platform_driver_unregister(&mute_driver);
}

module_init(mute_mod_init);
module_exit(mute_mod_exit);

MODULE_DESCRIPTION("Vanzo Mute driver");
MODULE_AUTHOR("AL <xuechuanzheng@vanzotec.com>");
MODULE_LICENSE("GPL");
