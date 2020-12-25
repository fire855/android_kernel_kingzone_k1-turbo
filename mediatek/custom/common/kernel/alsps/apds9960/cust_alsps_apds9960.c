#ifndef __APDS9960_CUST_ALSPS__
#define __APDS9960_CUST_ALSPS__

#include <linux/types.h>
#include <mach/mt_pm_ldo.h>
#include <cust_alsps.h>
#include <mach/upmu_common.h>

static struct alsps_hw cust_alsps_hw = {
    .i2c_num    = 2,
	.polling_mode_ps =0,
	.polling_mode_als =1,
    .power_id   = MT65XX_POWER_NONE,    /*LDO is not used*/
    .power_vol  = VOL_DEFAULT,          /*LDO is not used*/
    //.i2c_addr   = {0x0C, 0x48, 0x78, 0x00},
    .als_level  = {20, 40, 70, 100, 250, 650, 800,  1100, 1500, 2500,  4000,  6000,  7000, 8000,  10000},
    .als_value  = {40, 40, 80,  90, 120, 160, 225,  320,  640,  1280,  1280,  2600,  2600, 2600,  10240, 10240},
    .ps_threshold_high = 120,
    .ps_threshold_low = 100,
};
struct alsps_hw *get_cust_alsps_hw(void) {
    return &cust_alsps_hw;
}
#endif
