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

#include <mach/mt_boot.h>
#include <cust_eint.h>
#include <cust_gpio_usage.h>
#include <mach/mt_gpio.h>

enum cam_switch_report_state
{
	CAM_SWITCH_MAIN =0,
	CAM_SWITCH_SUB = 1,
};

int cam_switch_hall_enit_state = CAM_SWITCH_MAIN;

extern void mt_eint_mask(unsigned int eint_num);                                                                                                                         
extern void mt_eint_unmask(unsigned int eint_num);
extern void mt_eint_set_hw_debounce(unsigned int eint_num, unsigned int ms); 
extern void mt_eint_set_polarity(unsigned int eint_num, unsigned int pol);
extern unsigned int mt_eint_set_sens(unsigned int eint_num, unsigned int sens);
extern void mt_eint_registration(unsigned int eint_num, unsigned int flow, void (EINT_FUNC_PTR)(void), unsigned int is_auto_umask);
extern void mt_eint_print_status(void);

extern void Cam_HallSwitchFlipMirror();

static struct workqueue_struct * cam_switch_eint_workqueue = NULL;
static struct work_struct cam_switch_eint_work;

static struct switch_dev cam_switch_data;

void cam_switch_hall_eint_work_callback(struct work_struct *work)
{
    mt_eint_mask(CUST_EINT_HALL_1_NUM);
	if(cam_switch_hall_enit_state == CAM_SWITCH_SUB)
	{
        switch_set_state((struct switch_dev *)&cam_switch_data, CAM_SWITCH_SUB);
        Cam_HallSwitchFlipMirror();
	}
	else
	{
        switch_set_state((struct switch_dev *)&cam_switch_data, CAM_SWITCH_MAIN);
        Cam_HallSwitchFlipMirror();
	}
    mt_eint_unmask(CUST_EINT_HALL_1_NUM);
}

void cam_switch_hall_eint_func(void)
{
	int ret;
	
	if(cam_switch_hall_enit_state ==  CAM_SWITCH_MAIN ) 
	{
		mt_eint_set_polarity(CUST_EINT_HALL_1_NUM, 1);
		cam_switch_hall_enit_state = CAM_SWITCH_SUB;
	}
	else
	{
		mt_eint_set_polarity(CUST_EINT_HALL_1_NUM, 0);
		cam_switch_hall_enit_state = CAM_SWITCH_MAIN;
	}

	ret = queue_work(cam_switch_eint_workqueue, &cam_switch_eint_work); 
}

static inline int cam_switch_setup_eint(void)
{
	printk("%s line %d cam_switch_hall_enit_state %d\n",__func__,__LINE__, cam_switch_hall_enit_state);
	
	mt_set_gpio_mode(GPIO_HALL_1_PIN, GPIO_HALL_1_PIN_M_EINT);
	mt_set_gpio_dir(GPIO_HALL_1_PIN, GPIO_DIR_IN);
	mt_set_gpio_pull_enable(GPIO_HALL_1_PIN, GPIO_PULL_DISABLE);

    //mt65xx_eint_set_sens(CUST_EINT_HALL_1_NUM, CUST_EINT_HALL_SENSITIVE);
	mt_eint_set_hw_debounce(CUST_EINT_HALL_1_NUM,CUST_EINT_HALL_1_DEBOUNCE_EN);
	mt_eint_registration(CUST_EINT_HALL_1_NUM, cam_switch_hall_enit_state?CUST_EINTF_TRIGGER_HIGH:CUST_EINTF_TRIGGER_LOW,  cam_switch_hall_eint_func, 0);
	mt_eint_unmask(CUST_EINT_HALL_1_NUM);  

	return 0;
}

static int cam_switch_probe(struct platform_device *dev)
{
	int ret = 0;
	bool curr_state;
	
	mt_set_gpio_mode(GPIO_HALL_1_PIN, 0);
	mt_set_gpio_dir(GPIO_HALL_1_PIN, GPIO_DIR_IN);
	mt_set_gpio_pull_enable(GPIO_HALL_1_PIN, GPIO_PULL_DISABLE);
	curr_state = mt_get_gpio_in(GPIO_HALL_1_PIN);
	printk("%s line %d curr_state %d\n",__func__,__LINE__, curr_state);

	if(!curr_state){
		cam_switch_data.name = "cam_switch";
		cam_switch_data.index = 0;
		cam_switch_data.state = CAM_SWITCH_SUB;
		cam_switch_hall_enit_state = CAM_SWITCH_SUB;
	}else{
		cam_switch_data.name = "cam_switch";
		cam_switch_data.index = 0;
		cam_switch_data.state = CAM_SWITCH_MAIN;
		cam_switch_hall_enit_state = CAM_SWITCH_MAIN;
	}

	ret = switch_dev_register(&cam_switch_data);
	if(ret)
	{
		printk("switch_dev_register return %d\n", ret);
	}

	cam_switch_eint_workqueue = create_singlethread_workqueue("cam_switch_hall_eint");
	INIT_WORK(&cam_switch_eint_work, cam_switch_hall_eint_work_callback);

	cam_switch_setup_eint();
	
	return 0;
}

static int cam_switch_remove(struct platform_device *dev)
{
	destroy_workqueue(cam_switch_eint_workqueue);
	switch_dev_unregister(&cam_switch_data);

	return 0;
}

static struct platform_driver cam_switch_driver = {
	.probe = cam_switch_probe,
	.suspend = NULL,
	.resume = NULL,
	.remove = cam_switch_remove,
	.driver = {
		.name = "cam_switch",
	},
};

static int cam_switch_mod_init(void)
{
	int ret = 0;

	if(platform_driver_register(&cam_switch_driver) != 0)
	{
		printk("unable to register hall driver\n");
		return -1;
	}
	
	return 0;
}

static void cam_switch_mod_exit(void)
{
	platform_driver_unregister(&cam_switch_driver);
}

module_init(cam_switch_mod_init);
module_exit(cam_switch_mod_exit);
