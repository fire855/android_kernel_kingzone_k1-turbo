#include <cust_leds.h>
#include <cust_leds_def.h>
#include <mach/mt_pwm.h>
#include <mach/mt_gpio.h>

#include <linux/kernel.h>
#include <linux/delay.h>
#include <asm/delay.h>
#include <mach/pmic_mt6329_hw_bank1.h>
#include <mach/pmic_mt6329_sw_bank1.h>
#include <mach/pmic_mt6329_hw.h>
#include <mach/pmic_mt6329_sw.h>
#include <mach/upmu_common_sw.h>
#include <mach/upmu_hw.h>

extern int disp_bls_set_backlight(unsigned int level);

// Only support 64 levels of backlight (when lcd-backlight = MT65XX_LED_MODE_PWM)
#define BACKLIGHT_LEVEL_PWM_64_FIFO_MODE_SUPPORT 64
// Support 256 levels of backlight (when lcd-backlight = MT65XX_LED_MODE_PWM)
#define BACKLIGHT_LEVEL_PWM_256_SUPPORT 256

// Configure the support type "BACKLIGHT_LEVEL_PWM_256_SUPPORT" or "BACKLIGHT_LEVEL_PWM_64_FIFO_MODE_SUPPORT" !!
#define BACKLIGHT_LEVEL_PWM_MODE_CONFIG BACKLIGHT_LEVEL_PWM_256_SUPPORT

unsigned int Cust_GetBacklightLevelSupport_byPWM(void)
{
	return BACKLIGHT_LEVEL_PWM_MODE_CONFIG;
}
static bool backlight_flag =0;
static unsigned int back_level = 64;
unsigned int disp_set_backlight(int level)
{
	int num ,now_level,pre_level;
	int i;
	if(!backlight_flag)
	{
		mt_set_gpio_out(GPIO_LCM_BACKLIGHT_EN_PIN, GPIO_OUT_ZERO);
		msleep(1);
		backlight_flag = 1;
	}
	if(level == 0) {
		mt_set_gpio_out(GPIO_LCM_BACKLIGHT_EN_PIN, GPIO_OUT_ZERO);
	} else {
		if(back_level == 0) {
			mt_set_gpio_out(GPIO_LCM_BACKLIGHT_EN_PIN, GPIO_OUT_ONE);
			msleep(2);
		}

		now_level = 64 -(level >> 4);
		pre_level = 64 - (back_level >> 4);
		if(now_level >= pre_level)
			num = now_level - pre_level;
		else if (now_level < pre_level)
			num = 64 + now_level - pre_level;
		//printk("ktd318 level back_level now_level pre_level num = %d %d %d %d %d\n" ,level,back_level,now_level,pre_level,num);
		for(i=0 ;i < num;i++)
		{
			mt_set_gpio_out(GPIO_LCM_BACKLIGHT_EN_PIN, GPIO_OUT_ZERO);
			udelay(2);
			mt_set_gpio_out(GPIO_LCM_BACKLIGHT_EN_PIN, GPIO_OUT_ONE);
			udelay(2);
		}
	}
	back_level = level;
	return 0;
}
unsigned int brightness_mapping(unsigned int level)
{
    unsigned int mapped_level;

    mapped_level = level;

	return mapped_level;
}
static struct cust_mt65xx_led cust_led_list[MT65XX_LED_TYPE_TOTAL] = {
/* Vanzo:songlixin on: Sat, 12 Apr 2014 15:48:50 +0800
	{"red",               MT65XX_LED_MODE_PMIC, MT65XX_LED_PMIC_NLED_ISINK0,{0}},
	{"green",             MT65XX_LED_MODE_PMIC, MT65XX_LED_PMIC_NLED_ISINK1,{0}},
	{"blue",              MT65XX_LED_MODE_NONE, -1,{0}},
	{"jogball-backlight", MT65XX_LED_MODE_NONE, -1,{0}},
	{"keyboard-backlight",MT65XX_LED_MODE_NONE, -1,{0}},
	{"button-backlight",  MT65XX_LED_MODE_NONE, -1,{0}},
	{"lcd-backlight",     MT65XX_LED_MODE_CUST_BLS_PWM, (int)disp_bls_set_backlight,{0}},
 */
	{"red",               MT65XX_LED_MODE_PMIC, MT65XX_LED_PMIC_NLED_ISINK1,{0}},
	{"green",             MT65XX_LED_MODE_PMIC, MT65XX_LED_PMIC_NLED_ISINK0,{0}},
	{"blue",              MT65XX_LED_MODE_PMIC, MT65XX_LED_PMIC_NLED_ISINK2,{0}},
	{"jogball-backlight", MT65XX_LED_MODE_NONE, -1,{0}},
	{"keyboard-backlight",MT65XX_LED_MODE_NONE, -1,{0}},
	{"button-backlight",  MT65XX_LED_MODE_PMIC, MT65XX_LED_PMIC_NLED_ISINK3,{0}},
	//{"lcd-backlight",     MT65XX_LED_MODE_CUST_BLS_PWM, (int)disp_bls_set_backlight,{0}},
	{"lcd-backlight",     MT65XX_LED_MODE_CUST_BLS_PWM, disp_set_backlight, {0}},
// End of Vanzo: songlixin
};

struct cust_mt65xx_led *get_cust_led_list(void)
{
	return cust_led_list;
}
