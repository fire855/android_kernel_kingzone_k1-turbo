#ifdef BUILD_LK
#include <platform/mt_pmic.h>
#else
#include <linux/string.h>
#include <mach/mt_pm_ldo.h>
#endif

#include "lcm_drv.h"
#define udelay(us)  \
    do { \
        volatile int count = us * 10; \
        while (count--); \
    }while(0)

#define mdelay(ms) \
    do { \
        unsigned long i; \
        for (i = 0; i < ms; i++) \
        udelay(1000); \
    }while(0)

void lcm_power_on(void)
{
#if defined(MT6592)
#ifdef BUILD_LK
    upmu_set_rg_vgp2_en(0);
    mdelay(10);
    upmu_set_rg_vgp2_vosel(6);
    upmu_set_rg_vgp2_en(1);
#else
    hwPowerDown(MT6323_POWER_LDO_VGP2, "LCM");
    mdelay(10);
    hwPowerOn(MT6323_POWER_LDO_VGP2, VOL_3300, "LCM");
#endif
#elif defined(MT6582)
#if (defined(DCT_A8) || defined(DCT_A7))
#ifdef BUILD_LK
	upmu_set_rg_vgp2_vosel(6);
	upmu_set_rg_vgp2_en(1);
#else
	hwPowerOn(MT6323_POWER_LDO_VGP2, VOL_3300, "LCM");
#endif
#endif
#endif
}
void lcm_power_off(void)
{
#if defined(MT6592)
#ifdef BUILD_LK
	upmu_set_rg_vgp2_en(0);
#else
	hwPowerDown(MT6323_POWER_LDO_VGP2, "LCM");
#endif
#elif defined(MT6582)
#if (defined(DCT_A8) || defined(DCT_A7))
#ifdef BUILD_LK
	upmu_set_rg_vgp2_en(0);
#else
	hwPowerDown(MT6323_POWER_LDO_VGP2, "LCM");
#endif
#endif
#endif
}
