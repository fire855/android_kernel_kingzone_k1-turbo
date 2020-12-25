#include <linux/module.h>               // For module specific items
#include <linux/moduleparam.h>          // For new moduleparam's
#include <linux/types.h>                // For standard types (like size_t)
#include <linux/errno.h>                // For the -ENODEV/... values
#include <linux/kernel.h>               // For dbmsg/panic/...
#include <linux/fs.h>                   // For file operations
#include <linux/ioport.h>               // For io-port access
#include <linux/platform_device.h>      // For platform_driver framework
#include <linux/init.h>                 // For __init/__exit/...
#include <linux/uaccess.h>              // For copy_to_user/put_user/...
#include <linux/io.h>                   // For inb/outb/...
#include <linux/gpio.h>  								// GPIO
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/slab.h>               	//kamlloc

#include <linux/string.h>
#include <linux/mutex.h>
#include <linux/kmod.h>
#include <linux/mm.h>
#include <asm/system.h>

#include <linux/hrtimer.h>
#include <linux/err.h>
#include <linux/spinlock.h>

#include <linux/jiffies.h>
#include <linux/timer.h>



#include <mach/mt_pwm.h>
#include <mach/mt_typedefs.h>
//#include <mach/mt_clock_manager.h>


#include <mach/mt_reg_base.h>
#include <mach/mt_boot.h>
#include <mtk_kpd.h>		/* custom file */
#include <mach/mt_gpio.h>


//#include <mach/irqs.h>
//#include <mach/eint.h>
//#include <mach/mt_gpio.h>
/**************gpio_config_start*********************/
//#include <linux/gpio.h>
#define BUFFERSIZE  1024
struct estar_lcm3d_t
{
	struct cdev cdev;
	struct class * estar_class;
	dev_t devno;
	int device_major;
	int flag_cam3d; //CAMERA_3D or CAMERA_2D
	unsigned char mem[BUFFERSIZE];
};
struct estar_lcm3d_t *estar_lcm3dp; /* 设备结构体指针*/

struct FaceOffsets
{
	int    confidence;
	int    midpointx;
	int    midpointy;
	int    eyedist;
	int    eulerx;
	int    eulery;
	int    eulerz;
};
typedef struct FaceOffsets FaceOffset;
static FaceOffset g_face;

#define DEBUG 1

#if DEBUG
#define dbmsg(fmt, args ...) printk(KERN_NOTICE "lcm3d: %s[%d]: "fmt"\n", __FUNCTION__, __LINE__,##args)
#else
#define dbmsg(fmt, args ...)
#endif
//配置管脚定义
/****************
PWM1=LCM_COM=GPIO48
PWM2=LCM_SEG=GPIO49
LCM_CS=GPIO43
dc_en GPIO55
*****************/
#define gpio_lcd_backlight_3d2_en (GPIO48 | 0x80000000)/*3d2 gpio			3D模式时，COM脚输出60HZ方波，时序与SIG脚相反，2D时拉低*/
#define gpio_lcd_backlight_3d1_en (GPIO49 | 0x80000000)/*3d1 gpio			3D模式时，SIG脚输出60HZ方波，2D时拉低		*/
#define gpio_lcd_backlight_dc_en  (GPIO55 | 0x80000000)/*dc control gpio   5V电源控制，2D状态时拉低，3D模式拉高使5V有效*/
//#define gpio_switch_3d        	  (MT_GPIO_BASE_MAX | 0x80000000)
//#define gpio_switch_lrcam         (MT_GPIO_BASE_MAX | 0x80000000)


static void gpio_3d_config_up(void)
{
	mt_set_gpio_out(gpio_lcd_backlight_3d1_en,1);//1////sunhouzan for test 2014-6-5
	mt_set_gpio_out(gpio_lcd_backlight_3d2_en,1);////0//sunhouzan for test 2014-6-5
	mt_set_gpio_out(gpio_lcd_backlight_dc_en,1);
}
/*
static void gpio_3d_config_down(void)
{
	mt_set_gpio_out(gpio_lcd_backlight_3d1_en,0);
	mt_set_gpio_out(gpio_lcd_backlight_3d2_en,1);
	mt_set_gpio_out(gpio_lcd_backlight_dc_en,0);
}
*/
static void gpio_3d_config_down(void)
{
	mt_set_gpio_out(gpio_lcd_backlight_3d1_en,1);//0//sunhouzan for test 2014-6-5
	mt_set_gpio_out(gpio_lcd_backlight_3d2_en,1);//0//sunhouzan for test 2014-6-5
	mt_set_gpio_out(gpio_lcd_backlight_dc_en,0);
}
/**************gpio_config_end*********************/

#include <linux/dma-mapping.h> //DO-->hrtimer包含以下三个头文件 /* DMA APIs             */
#include <linux/hrtimer.h>
#include <linux/time.h>           /* struct timespec    */

//#define KER_PRINT(fmt, ...) dbmsg("<ker-driver>"fmt, ##__VA_ARGS__);
static struct hrtimer vibe_timer;
static spinlock_t vibe_lock;
static int value = 8;   /*注：以毫秒ms为单位 Time out setting,0.016 seconds 产生60Hz方波*/


static unsigned long cam3d_open_flag=0x4803;
static int lcm3d_open_flag=0;//未打开上层软件时suspend和resume内部函数不生效
static enum hrtimer_restart vibrator_timer_func(struct hrtimer *timer)
{
	static int turn_flag;
	//if lcm3d is  off, the next cann't exec
	if(lcm3d_open_flag == 0)
		return -1;
	if(turn_flag == 0)
	{
		turn_flag = 1;
		mt_set_gpio_out(gpio_lcd_backlight_3d1_en,1);
		mt_set_gpio_out(gpio_lcd_backlight_3d2_en,0);
	}
	else if(turn_flag == 1)
	{
		turn_flag = 0;
		mt_set_gpio_out(gpio_lcd_backlight_3d1_en,0);
		mt_set_gpio_out(gpio_lcd_backlight_3d2_en,1);
	}

	//dbmsg("'vibe_work_func'-->work\n");
	// msleep(50); /* CPU sleep */
	if(lcm3d_open_flag==1){
		vibe_timer.function = vibrator_timer_func;
		hrtimer_start(&vibe_timer,  ktime_set(value / 1000, (value % 1000) * 1000000),HRTIMER_MODE_REL);
	}
	return HRTIMER_NORESTART;
}

static void ker_driver_init(void)                        //DO-->hrtimer高精度定时器初始化函数
{
	//struct timespec uptime;

	dbmsg("ker_driver_init");
	hrtimer_init(&vibe_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);  //DO-->hrtimer定时器初始化
	vibe_timer.function = vibrator_timer_func;                     //DO-->hrtimer定时器回调函数
	hrtimer_start(&vibe_timer,
			ktime_set(value / 1000, (value % 1000) * 1000000),HRTIMER_MODE_REL);  //DO-->hrtimer定时器时间初始化，其中ktime_set（秒，毫秒）
}
/**************platform-suspend********************/
static struct platform_device suspend_3d_device={
	.name   = "suspend_3d",//与driver的name统一
	.id     = -1,//driver对应的不同device
};

void lcm3d_off( void );
void lcm3d_on( void );
void camera_3d(void);
void camera_2d(void);
void lcam_on(void);
void rcam_on(void);
static int probe_3d_func(struct platform_device *lcm3d_device)//通过probe挂载到队伍里被搜索到
{
	mt_set_gpio_mode(gpio_lcd_backlight_3d1_en, 0);
	mt_set_gpio_mode(gpio_lcd_backlight_3d2_en, 0);
	mt_set_gpio_mode(gpio_lcd_backlight_dc_en, 0);
	mt_set_gpio_dir(gpio_lcd_backlight_3d1_en,1);
	mt_set_gpio_dir(gpio_lcd_backlight_3d2_en,1);//GPIO_DIR_OUT
	mt_set_gpio_dir(gpio_lcd_backlight_dc_en,1);

	//mt_set_gpio_mode(gpio_switch_3d,0);
	//mt_set_gpio_dir(gpio_switch_3d,GPIO_DIR_OUT);

	//mt_set_gpio_mode(gpio_switch_lrcam,0);
	//mt_set_gpio_dir(gpio_switch_lrcam,GPIO_DIR_OUT);
	//mt_set_gpio_mode(gpio_lcd_backlight_sw_en,0);

	//dbmsg("DO--------------->%s\n",__func__);
	return 0;
}
static int suspend_3d_func(struct platform_device *lcm3d, pm_message_t state)
{
	// dbmsg("DO--------------->%s\n",__func__);
	if(lcm3d_open_flag==1)//未打开上层软件时suspend和resume内部函数不生效
		gpio_3d_config_down();//3d管脚不输出电压
	return 0;
}
static int resume_3d_func(struct platform_device *lcm3d)
{
	// dbmsg("DO--------------->%s\n",__func__);
	if(lcm3d_open_flag==1)//未打开上层软件时suspend和resume内部函数不生效
		gpio_3d_config_up();//3d管脚输出电压
	return 0;
}
static struct platform_driver suspend_3d_driver={
	.probe      = probe_3d_func,
	.suspend    = suspend_3d_func,
	.resume     = resume_3d_func,
	.driver     = {
		.name   = "suspend_3d",
		.owner  =THIS_MODULE,
	}
};

/**************platform-end********************/

//#define gpio_lcd_backlight_sw_en  69; //sw control gpio
/******************************************************************************
 * Sysfs attributes
 *******************************************************************************/
static ssize_t lcm3d_show_control(struct device_driver *ddri, char *buf)
{
	ssize_t res;

	res = snprintf(buf, PAGE_SIZE, "%d\n",lcm3d_open_flag);
	return res;
}
/*----------------------------------------------------------------------------*/
static ssize_t lcm3d_store_control(struct device_driver *ddri, const char *buf, size_t count)
{
	int lcm3d_status;
	if(1 == sscanf(buf, "%d", &lcm3d_status))
	{
		if(lcm3d_status)
		{
			lcm3d_open_flag = lcm3d_status;
			lcm3d_on();
		}
		else
		{
			lcm3d_open_flag = lcm3d_status;
			lcm3d_off();
		}
	}
	else
	{
		dbmsg("invalid content: '%s', length = %d\n", buf, count);
	}
	return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t lcm3d_store_cam3d(struct device_driver *ddri, const char *buf, size_t count)
{
	int lcm3d_status;
	if(1 == sscanf(buf, "%d", &lcm3d_status))
	{
		if(lcm3d_status)
		{
			//mt_set_gpio_out(gpio_switch_3d,1);
			camera_3d();
			dbmsg("%s:switch to camera 3d\n",__func__);
		}
		else
		{
			//mt_set_gpio_out(gpio_switch_3d,0);
			camera_2d();
			dbmsg("%s:switch to camera 2d\n",__func__);
		}
	}
	else
	{
		dbmsg("invalid content: '%s', length = %d\n", buf, count);
	}
	return count;
}
static ssize_t cam3d_show_state(struct device_driver *ddri, char *buf)
{
	ssize_t res;

	res = snprintf(buf, PAGE_SIZE, "%ld\n",cam3d_open_flag);
	return res;
}
static ssize_t lcm3d_store_LRcam3d(struct device_driver *ddri, const char *buf, size_t count)
{
	int lcm3d_status;
	if(1 == sscanf(buf, "%d", &lcm3d_status))
	{
		if(lcm3d_status)
		{
			//mt_set_gpio_out(gpio_switch_lrcam,1);
			dbmsg("lcm3d: %s:switch to camera L\n",__func__);
		}
		else
		{
			//mt_set_gpio_out(gpio_switch_lrcam,0);
			dbmsg("lcm3d: %s:switch to camera R\n",__func__);
		}
	}
	else
	{
		dbmsg("invalid content: '%s', length = %d\n", buf, count);
	}
	return count;
}

static DRIVER_ATTR(control,  S_IWUSR | S_IRUGO, lcm3d_show_control,lcm3d_store_control);
static DRIVER_ATTR(cam3d,  S_IWUSR | S_IRUGO, cam3d_show_state,lcm3d_store_cam3d);
static DRIVER_ATTR(lrcam,  S_IWUSR | S_IRUGO, NULL,lcm3d_store_LRcam3d);

static struct driver_attribute *lcm3d_attr_list[] = {
	&driver_attr_control,
	&driver_attr_cam3d,
	&driver_attr_lrcam,

};
static int lcm3d_create_attr(struct device_driver *driver)
{
	int idx, err = 0;
	int num = (int)(sizeof(lcm3d_attr_list)/sizeof(lcm3d_attr_list[0]));
	if (driver == NULL)
	{
		return -EINVAL;
	}

	for(idx = 0; idx < num; idx++)
	{
		if((err = driver_create_file(driver, lcm3d_attr_list[idx])))
		{
			dbmsg("driver_create_file (%s) = %d\n", lcm3d_attr_list[idx]->attr.name, err);
			break;
		}
	}
	return err;
}
/*----------------------------------------------------------------------------*/
static int lcm3d_delete_attr(struct device_driver *driver)
{
	int idx ,err = 0;
	int num = (int)(sizeof(lcm3d_attr_list)/sizeof(lcm3d_attr_list[0]));

	if (!driver)
		return -EINVAL;

	for (idx = 0; idx < num; idx++)
	{
		driver_remove_file(driver, lcm3d_attr_list[idx]);
	}

	return err;
}

//static int lcm3d_major = 252;   // lcm3d_major = 0;
static int  lcm3d_major = 0;

#define LCM3D_ON  0x4800
#define LCM3D_OFF 0x4801
#define CAMERA_3D  0x4802
#define CAMERA_2D 0x4803
#define LRCAM_CLR   0x4804
#define LRCAM_SET   0x4805
#define CAMERA_GET_STATE  0x4806
#define LCM_SET_IO_STATE  0x4807
#define LCM_GET_IO_STATE  0x4808
#define CAMERA_SET_FACE_DATA  0x4810
#define CAMERA_GET_FACE_DATA  0x4811


void lcm3d_off( void )
{
	unsigned long	flags;

	spin_lock_irqsave(&vibe_lock, flags);

	while(hrtimer_cancel(&vibe_timer))//定时器取消
	{
		dbmsg("[lcm3d] lcm3d_off: try to cancel hrtimer \n");
	}
	gpio_3d_config_down();//3d管脚不输出电压
	spin_unlock_irqrestore(&vibe_lock, flags);

	dbmsg("DO_DEBUG-->set lcm3d off\n");
}
void lcm3d_on( void )
{
	dbmsg("DO_DEBUG-->set lcm3d on");
	ker_driver_init();
	gpio_3d_config_up();
}

void camera_3d(void)
{
	int ret = 0;
	unsigned long	flags;
	spin_lock_irqsave(&vibe_lock, flags);
	//mt_set_gpio_out(gpio_switch_3d,1);
	//ret = mt_get_gpio_out(gpio_switch_3d);
	//mt_set_gpio_out(gpio_switch_lrcam, 1); //1-->right
	cam3d_open_flag = CAMERA_3D;
	spin_unlock_irqrestore(&vibe_lock, flags);
	dbmsg("%s:switch to camera 3d, ret=%d\n",__func__, ret);
}

void camera_2d(void)
{
	unsigned long	flags;
	spin_lock_irqsave(&vibe_lock, flags);
	//mt_set_gpio_out(gpio_switch_3d,0);
	cam3d_open_flag = CAMERA_2D;
	//0-->left, set to left, because left camera can fouse
	//mt_set_gpio_out(gpio_switch_lrcam,0);
	spin_unlock_irqrestore(&vibe_lock, flags);
	dbmsg("%s:switch to camera 2d\n",__func__);
}
void lrcam_clr(void)
{
	//mt_set_gpio_out(gpio_switch_lrcam,0);
	dbmsg("%s:switch to camera R(now clr)\n",__func__);
}
void lrcam_set(void) //0-->left; 1-->right
{
	//mt_set_gpio_out(gpio_switch_lrcam,1);
	dbmsg("%s:switch to camera L(now set)\n",__func__);
}
void get_cam_state(void)
{
	dbmsg("%s:get_cam_state \n",__func__);
}


static long lcm3d_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	void __user *data;
	unsigned long	flags;
	int n_3d1_en,n_3d2_en,n_dc_en;
	unsigned long state;
	long err = 0;
	//	cmd = (unsigned int)file; //解决与2.3版本兼容问题
	dbmsg("cmd=0x%x, arg=0x%x", cmd, (unsigned int)arg);
	switch ( cmd ) {
		case LCM3D_ON:
			if(lcm3d_open_flag !=1)
			{
				lcm3d_open_flag=1;//打开上层软件时suspend内部函数生效
				lcm3d_on();
				dbmsg("%s:open 3d\n",__func__);
			}
			break;
		case LCM3D_OFF:
			if(lcm3d_open_flag !=0)
			{
				lcm3d_open_flag=0;//未打开上层软件时suspend内部函数不生效
				lcm3d_off();
				dbmsg("%s:close 3d\n",__func__);
			}
			break;

		case CAMERA_3D:
			{
				camera_3d();
				//lrcam_set();
				dbmsg("camera 3d\n");
				break;
			}
		case CAMERA_2D:
			{
				camera_2d();
				dbmsg("%s:camera 2d\n",__func__);
				break;
			}
		case LRCAM_SET:
			{
				lrcam_set();
				break;
			}
		case LRCAM_CLR:
			{
				lrcam_clr();
				break;
			}
		case CAMERA_GET_STATE:
			{
				spin_lock_irqsave(&vibe_lock, flags);
				data = (void __user *) arg;
				if(data == NULL)
				{
					err = -EINVAL;
					break;
				}
				if(copy_to_user((int __user*)data, &cam3d_open_flag, sizeof(cam3d_open_flag)))
				{
					err = -EFAULT;
					dbmsg(KERN_NOTICE "err copy_to_user\n");
					break;
				}
				spin_unlock_irqrestore(&vibe_lock, flags);
				dbmsg(KERN_NOTICE "cam3d_open_flag: data=0x%x\n", *(int*)data);
				break;
			}
			case LCM_SET_IO_STATE:
			{
				spin_lock_irqsave(&vibe_lock, flags);
				data = (void __user *) arg;
				if(data == NULL)
				{
					err = -EINVAL;
					break;
				}
				state = *(unsigned long*)arg;
				dbmsg("set state = %d", (unsigned int)state);
				spin_unlock_irqrestore(&vibe_lock, flags);
				n_3d1_en = state%10;
				n_3d2_en = state%100/10;
				n_dc_en = state/100;
				mt_set_gpio_out(gpio_lcd_backlight_3d1_en, n_3d1_en);
				mt_set_gpio_out(gpio_lcd_backlight_3d2_en, n_3d2_en);
				mt_set_gpio_out(gpio_lcd_backlight_dc_en, n_dc_en);
				break;
			}

		case LCM_GET_IO_STATE:
			{
				data = (void __user *) arg;
				if(data == NULL)
				{
					err = -EINVAL;
					break;
				}
				n_3d1_en = mt_get_gpio_out(gpio_lcd_backlight_3d1_en);
				n_3d2_en = mt_get_gpio_out(gpio_lcd_backlight_3d2_en);
				n_dc_en= mt_get_gpio_out(gpio_lcd_backlight_dc_en);
				state = n_dc_en*100 + n_3d2_en*10 + n_3d1_en;
				dbmsg("state = %d",(unsigned int)state);
				if(copy_to_user((int __user*)data, &state, sizeof(state)))
				{
					err = -EFAULT;
					dbmsg(KERN_NOTICE "err copy_to_user\n");
					break;
				}
				dbmsg("blacklight_3d1_en=%d", mt_get_gpio_out(gpio_lcd_backlight_3d1_en));
				dbmsg("gpio_lcd_backlight_3d2_en=%d",mt_get_gpio_out(gpio_lcd_backlight_3d2_en));
				dbmsg("gpio_lcd_backlight_dc_en=%d", mt_get_gpio_out(gpio_lcd_backlight_dc_en));
				break;
			}
		case CAMERA_SET_FACE_DATA:
			copy_from_user(&g_face, arg, sizeof(g_face));
			break;
		case CAMERA_GET_FACE_DATA:
			copy_to_user(arg, &g_face, sizeof(g_face));
			memset(&g_face,0,sizeof(g_face));
			break;
		default:
			{
				break;
			}
	}
	return 0;
}

int lcm3d_open (struct inode *inode, struct file *filp)
{
	//dbmsg("cam3d_open_flag: open");
	filp->private_data = estar_lcm3dp;

	mt_set_gpio_out(gpio_lcd_backlight_3d1_en,1);//sunhouzan for test 2014-6-5
	mt_set_gpio_out(gpio_lcd_backlight_3d2_en,1);//sunhouzan for test 2014-6-5

	return 0;
}
ssize_t lcm3d_read(struct file *filp, char __user *buff, size_t count, loff_t *ppos)
{
	int ret = 0;
	struct estar_lcm3d_t *dev = filp->private_data; /*通过文件私有数据指针得到设备结构体，和前面的open对应*/
	if ( count > BUFFERSIZE  )
		count = BUFFERSIZE;
	/*内核空间->用户空间*/
	if ( copy_to_user(buff, (void *)(dev->mem), count) )
	{
		ret = -EFAULT;
	}
	memset(dev->mem, 0, BUFFERSIZE);
	return count;
}
ssize_t lcm3d_write(struct file *filp, const char __user *buff, size_t count, loff_t *ppos)
{
	int ret = 0;
	struct estar_lcm3d_t *dev = filp->private_data; /*通过文件私有数据指针得到设备结构体，和前面?膐pen对应*/
	if (count > BUFFERSIZE)
		count = BUFFERSIZE;
	//dbmsg("write: copy_from_user: buf=%s", buff);
	if ( copy_from_user( dev->mem , buff, count))
	{
		ret = -EFAULT;
		return ret;
	}
	return count;
}

static int lcm3d_release(struct inode *node, struct file *file)
{
	return 0;
}
static struct file_operations lcm3d_remap_ops = {
	.owner   = THIS_MODULE,
	.open    = lcm3d_open,
	.release = lcm3d_release,
	.read    = lcm3d_read,
	.write   = lcm3d_write,
	.unlocked_ioctl   = lcm3d_ioctl,	//ioctl
};
static void lcm3d_setup_cdev(struct cdev *dev, int minor,
		struct file_operations *fops)
{
	int err;
	cdev_init(dev, fops);
	dev->owner = THIS_MODULE;
	dev->ops = fops;
	err = cdev_add (dev, estar_lcm3dp->devno, 1);
	if (err)
		dbmsg (KERN_NOTICE "Error %d adding lcm3d %d", err, minor);
}

static int __init lcm3d_init(void)
{
	int result;
	dbmsg("lcm3d init");
	estar_lcm3dp = 	kmalloc(sizeof(struct estar_lcm3d_t), GFP_KERNEL);
	if(NULL == estar_lcm3dp)
	{
		dbmsg("NO Memory!");
		return -1;
	}
	memset(estar_lcm3dp, 0, sizeof(struct estar_lcm3d_t));
	estar_lcm3dp->devno = MKDEV(lcm3d_major, 0);

	if (lcm3d_major)
		result = register_chrdev_region(estar_lcm3dp->devno, 1, "lcm3d");
	else {
		result = alloc_chrdev_region(&estar_lcm3dp->devno, 0, 1, "lcm3d");
		estar_lcm3dp->device_major= MAJOR(estar_lcm3dp->devno);
	}
	if (result < 0) {
		dbmsg(KERN_WARNING "lcm3d: unable to get major %d\n", estar_lcm3dp->device_major);
		return result;
	}
	if (estar_lcm3dp->device_major== 0)
		estar_lcm3dp->device_major= result;

	lcm3d_setup_cdev(&estar_lcm3dp->cdev, 0, &lcm3d_remap_ops);
	dbmsg("lcm3d device installed, with major %d\n", estar_lcm3dp->device_major);
	spin_lock_init(&vibe_lock);
	estar_lcm3dp->estar_class= class_create(THIS_MODULE, "lcm3d");
	device_create(estar_lcm3dp->estar_class, NULL, estar_lcm3dp->devno, NULL, "lcm3d");

	result = platform_device_register(&suspend_3d_device);
	if(result < 0)
	{
		dbmsg("DO--------------->%s,ret=%d\n",__func__,result);
		return -1;
	}
	result = platform_driver_register(&suspend_3d_driver);
	if(result < 0)
	{
		dbmsg("DO--------------->%s,ret=%d\n",__func__,result);
		return -1;
	}
	if((result = lcm3d_create_attr(&suspend_3d_driver.driver)))
	{
		dbmsg("create attribute err = %d\n", result);
		return -1;
	}
	return 0;
}
static void __exit lcm3d_cleanup(void)
{
	int err;
	if((err = lcm3d_delete_attr(&suspend_3d_driver.driver)))
	{
		dbmsg("lcm3d_delete_attr fail: %d\n", err);
	}

	device_destroy(estar_lcm3dp->estar_class,estar_lcm3dp->devno);
	class_destroy(estar_lcm3dp->estar_class);
	platform_driver_unregister(&suspend_3d_driver);
	platform_device_unregister(&suspend_3d_device);
	cdev_del(&estar_lcm3dp->cdev);
	unregister_chrdev_region(estar_lcm3dp->devno, 1);
	dbmsg("lcm3d device uninstalled\n");
}

module_init(lcm3d_init);
module_exit(lcm3d_cleanup);
module_param(lcm3d_major, int, 0);
MODULE_AUTHOR("www.estar.cn");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_VERSION("1.0");
