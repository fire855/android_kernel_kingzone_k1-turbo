#include <linux/types.h>
#include <cust_acc.h>
#include <mach/mt_pm_ldo.h>


/*---------------------------------------------------------------------------*/
#if defined(VANZO_ACC_BOTTOM)
static struct acc_hw cust_acc_hw = {
    .i2c_num = 2,
    .direction = 1,
    .power_id = MT65XX_POWER_NONE,  /*!< LDO is not used */
    .power_vol= VOL_DEFAULT,        /*!< LDO is not used */
    .firlen = 0, //old value 16                /*!< don't enable low pass fileter */
};
#else
static struct acc_hw cust_acc_hw = {
    .i2c_num = 2,
    .direction = 3,
    .power_id = MT65XX_POWER_NONE,  /*!< LDO is not used */
    .power_vol= VOL_DEFAULT,        /*!< LDO is not used */
    .firlen = 0, //old value 16                /*!< don't enable low pass fileter */
}; 
#endif
/*---------------------------------------------------------------------------*/
struct acc_hw* lsm6ds0_get_cust_acc_hw(void) 
{
    return &cust_acc_hw;
}
