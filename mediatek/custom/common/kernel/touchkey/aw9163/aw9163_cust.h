#include <linux/types.h>
#include <mach/mt_pm_ldo.h>

#define AW9163_ts_I2C_BUS            2

#if (defined(WVGA) || defined(CU_WVGA) || defined(CMCC_WVGA))
#define AW9163_LCM_H	800
#define AW9163_LCM_W	480
#elif (defined(FWVGA) || defined(CU_FWVGA) || defined(CMCC_FWVGA))
#define AW9163_LCM_H	854
#define AW9163_LCM_W	480
#elif (defined(QHD) || defined(CU_QHD) || defined(CMCC_QHD))
#define AW9163_LCM_H	960
#define AW9163_LCM_W	540
#elif (defined(HD) || defined(HD720))
#define AW9163_LCM_H	1280
#define AW9163_LCM_W	720
#elif (defined(FHD))
#define AW9163_LCM_H	1920
#define AW9163_LCM_W	1080
#elif (defined(HVGA))
#define AW9163_LCM_H	480
#define AW9163_LCM_W	320
#else
#define AW9163_LCM_H	800
#define AW9163_LCM_W	480
#endif

int cust_get_touchkey_hw(void) {
    return AW9163_ts_I2C_BUS;
}

