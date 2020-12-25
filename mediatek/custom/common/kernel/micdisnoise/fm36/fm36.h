#ifndef __FM36_H__
#define __FM36_H__

#include <linux/ioctl.h>

/*i2c address*/
#define FM36_I2C_ADDRESS	(0xC0>>1)

#define FM36_RST_PIN 		(GPIO123 | 0x80000000)

#endif
