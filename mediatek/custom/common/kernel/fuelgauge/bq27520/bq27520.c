#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <linux/earlysuspend.h>
#include <linux/platform_device.h>
#include <asm/atomic.h>

#include <cust_acc.h>
#include <linux/hwmsensor.h>
#include <linux/hwmsen_dev.h>
#include <linux/sensors_io.h>
#include <linux/hwmsen_helper.h>

#include <mach/mt_typedefs.h>
#include <mach/mt_gpio.h>
#include <mach/mt_pm_ldo.h>
#include <linux/dma-mapping.h>
#include "bq27520.h"
#include "cust_battery_meter.h"
/**********************************************************
  *
  *   [I2C Slave Setting]
  *
  *********************************************************/
#define bq27520_SLAVE_ADDR_WRITE   0xAA
#define bq27520_SLAVE_ADDR_READ    0xAB
#define BQ27520_BUSNUM 1

static bool BQ27520_DEBUG = 0;
bool Bq27520_Exist=0;
int mtk_current=0;

static struct i2c_client *new_client = NULL;
static const struct i2c_device_id bq27520_i2c_id[] = {{"bq27520",0},{}};
static struct i2c_board_info __initdata i2c_bq27520 = { I2C_BOARD_INFO("bq27520", (0xaa>>1))};
static int bq27520_driver_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int bq27520_i2c_remove(struct i2c_client *client);
static int bq27520_probe(struct platform_device *pdev);
static int bq27520_remove(struct platform_device *pdev);

static struct platform_driver bq27520_dev_driver = {
	.driver	  = {
		.name   = "bq27520",
	},
	.probe	  = bq27520_probe,
	.remove	  = bq27520_remove,
};

static struct i2c_driver bq27520_driver = {
	.driver = {
		.name    = "bq27520",
	},
	.probe       = bq27520_driver_probe,
	.remove      = bq27520_i2c_remove,
	.id_table    = bq27520_i2c_id,
};

/**********************************************************
  *
  *   [Global Variable]
  *
  *********************************************************/
static DEFINE_MUTEX(bq27520_i2c_access);
/**********************************************************
  *
  *   [I2C Function For Read/Write bq27520]
  *
  *********************************************************/
#define MAX_CMD_LEN          254
static int bq27520_DMA_write_ROM(unsigned int addr, kal_uint8 cmd,char *writebuf, int writelen)
{
	int ret;
	int i = 0,j = 0;
	u8 *buf = NULL;
	u32 phyAddr = 0;
	u32 bytes = 0;
	if (writelen > MAX_CMD_LEN) {
		printk("zlog [bq27520_DMA_write_ROM] exceed the max write length \n");
		return 0;
	}
	mutex_lock(&bq27520_i2c_access);
	new_client->addr = addr>>1;
	new_client->addr = new_client->addr & I2C_MASK_FLAG;
	new_client->timing=400;
	bytes = writelen +1;

	buf = dma_alloc_coherent(NULL, bytes, &phyAddr, GFP_KERNEL);
	buf[0] = cmd;
	for(i = 0 ; i < writelen; i++)
	{
		j=i+1;
		buf[j] = writebuf[i];
	}
	new_client->ext_flag = new_client->ext_flag | I2C_DMA_FLAG | I2C_ENEXT_FLAG;
	if(BQ27520_DEBUG)
	{
		printk("bq27520_set_DMA_write_ROM addr=0x%x,cmd=0x%x,data1=0x%x,Data2=0x%x\n",new_client->addr,buf[0],buf[1],buf[2]);
	}
	ret = i2c_master_send(new_client, (u8*)phyAddr, bytes);
	if (ret != bytes) {
		printk("zlog sent I2C ret = %d\n", ret);
	}
	dma_free_coherent(NULL, bytes, buf, phyAddr);
	new_client->ext_flag=0;
	mutex_unlock(&bq27520_i2c_access);

	return 0;
}

static int bq27520_set_cmd_write_ROM(unsigned int addr,kal_uint8 cmd, int WriteData)
{
	char cmd_buf[2]={0x00, 0x00};
    int ret=0;

    mutex_lock(&bq27520_i2c_access);

    new_client->addr = addr>>1;
	new_client->timing=200;
    new_client->ext_flag=((new_client->ext_flag ) & I2C_MASK_FLAG ) | I2C_DIRECTION_FLAG;

    cmd_buf[0] = cmd;
	cmd_buf[1] = WriteData & 0xFF;

	if(BQ27520_DEBUG)
	{
		printk("bq27520_set_cmd_write_ROM addr=0x%x,cmd=0x%x,data1=0x%x\n",new_client->addr,cmd_buf[0],cmd_buf[1]);
	}
    ret = mt_i2c_master_send(new_client, cmd_buf, 2, new_client->ext_flag);
    if (ret < 0)
    {
        new_client->ext_flag=0;
		printk("bq27520_set_cmd_write_ROM failed addr =0x%x,cmd=0x%x,data1=0x%x\n",new_client->addr,cmd_buf[0],cmd_buf[1]);
        mutex_unlock(&bq27520_i2c_access);
        return 0;
    }

    new_client->ext_flag=0;

    mutex_unlock(&bq27520_i2c_access);
    return 1;
}
static int bq27520_set_cmd_write(kal_uint8 cmd, int WriteData)
{
	char cmd_buf[2]={0x00, 0x00};
	int ret=0;

	mutex_lock(&bq27520_i2c_access);

	new_client->addr = 0xaa>>1;
	new_client->timing=200;
	new_client->ext_flag=((new_client->ext_flag ) & I2C_MASK_FLAG ) | I2C_DIRECTION_FLAG;

	cmd_buf[0] = cmd;
	cmd_buf[1] = WriteData & 0xFF;

	if(BQ27520_DEBUG)
	{
		printk("bq27520_set_cmd_write---addr=0x%x cmd=0x%x,cmd_buf1=0x%x,cmd_buf2=0x%x\n\r",new_client->addr,cmd_buf[0],cmd_buf[1],cmd_buf[2]);
	}
	ret = mt_i2c_master_send(new_client, cmd_buf, 2, new_client->ext_flag);
	if (ret < 0)
	{
		new_client->ext_flag=0;
		printk("bq27520_set_cmd_write fail---addr=%x -cmd=%d\n\r",new_client->addr,cmd);
		mutex_unlock(&bq27520_i2c_access);
		return 0;
	}
	new_client->ext_flag=0;

	mutex_unlock(&bq27520_i2c_access);
	return 1;
}

static int bq27520_set_cmd_read_ROM(unsigned int addr,kal_uint8 cmd,int *returnData)
{
	char cmd_buf[2]={0x00, 0x00};
	int readData = 0;
	int ret=0;

	mutex_lock(&bq27520_i2c_access);

	new_client->addr = addr>>1;
	new_client->timing=100;
	new_client->ext_flag=((new_client->ext_flag ) & I2C_MASK_FLAG ) | I2C_WR_FLAG | I2C_DIRECTION_FLAG | I2C_RS_FLAG;

	cmd_buf[0] = cmd;
	//printk("bq27520_set_cmd_read_ROM new_client->addr=0x%x\n",new_client->addr);
	//printk("bq27520_set_cmd_read_ROM---buffer=0x%x\n\r",cmd_buf[0]);
	ret = mt_i2c_master_send(new_client, &cmd_buf[0], (1<<8 | 1), new_client->ext_flag);
	if (ret < 0)
	{
		new_client->ext_flag=0;
		printk("bq27520_set_cmd_read_ROM failed new_client->addr=0x%x,buffer=0x%x\n",new_client->addr,cmd_buf[0]);
		mutex_unlock(&bq27520_i2c_access);
		return 0;
	}

	//readData = (cmd_buf[1] << 8) | cmd_buf[0];
	readData = cmd_buf[0];
	*returnData = readData;
	//printk("bq27520_set_cmd_read_ROM---readData=%d\n\r",readData);
	new_client->ext_flag=0;

	mutex_unlock(&bq27520_i2c_access);
	return 1;
}
static int bq27520_set_cmd_read(kal_uint8 cmd, int *returnData)
{
	char     cmd_buf[2]={0x00, 0x00};
	int      readData = 0;
	int      ret=0;

	mutex_lock(&bq27520_i2c_access);

	new_client->addr = 0xaa>>1;
	new_client->timing=100;
	new_client->ext_flag=((new_client->ext_flag ) & I2C_MASK_FLAG ) | I2C_WR_FLAG | I2C_DIRECTION_FLAG | I2C_RS_FLAG;

	cmd_buf[0] = cmd;
	ret = mt_i2c_master_send(new_client, &cmd_buf[0], (2<<8 | 1), new_client->ext_flag);
	if (ret < 0)
	{
		new_client->ext_flag=0;
		printk("bq27520_set_cmd_read_fail---addr=%x -cmd=%d\n\r",new_client->addr,cmd);
		mutex_unlock(&bq27520_i2c_access);
		return 0;
	}

	readData = (cmd_buf[1] << 8) | cmd_buf[0];
	*returnData = readData;
	if(BQ27520_DEBUG)
	{
		printk("bq27520_set_cmd_read [cmd data] [%x %d]\n", cmd,readData);
	}
	new_client->ext_flag=0;

	mutex_unlock(&bq27520_i2c_access);
	return 1;
}

static int bq27520_set_cmd_read_current(int *returnData)
{
	char     cmd_buf[2]={0x00, 0x00};
	int      readData = 0;
	int      ret=0;

	mutex_lock(&bq27520_i2c_access);

	new_client->addr = 0xaa>>1;
	new_client->timing=100;

	cmd_buf[0] = 0x14;
	ret = i2c_master_send(new_client, &cmd_buf[0], 1);
	if (ret < 0)
	{
		mutex_unlock(&bq27520_i2c_access);
		return 0;
	}
	ret = i2c_master_recv(new_client, &cmd_buf[0], 2);
	if (ret < 0)
	{
		mutex_unlock(&bq27520_i2c_access);
		return 0;
	}
	if(cmd_buf[1]&0x80)
	{
		readData = ((cmd_buf[1] << 8) | cmd_buf[0])-65535;
	}
	else{
		readData = ((cmd_buf[1] << 8) | cmd_buf[0]);
	}
	*returnData = readData;
	mutex_unlock(&bq27520_i2c_access);
	return 1;
}

int bq27520_get_battery_percentage(void)
{
	int returndata=0;
	bq27520_set_cmd_read(BQ27520_CMD_StateOfCharge,&returndata);
	return returndata;
}
int bq27520_get_battery_vol(void)
{
	int returndata=0;
	bq27520_set_cmd_read(BQ27520_CMD_Voltage,&returndata);
	return returndata;
}
int bq27520_get_battery_current(void)
{
	int returndata=0;
	bq27520_set_cmd_read_current(&returndata);
	return returndata;
}

int bq27520_get_battery_temprature(void)
{
#ifdef FIXED_TBAT_25
    return 25;
#else    
	int returndata=0;
	bq27520_set_cmd_read(BQ27520_CMD_Temperature,&returndata);
	returndata = (returndata/10)-273;
	return returndata;
#endif 
}
int bq27520_get_debuginfo(void)
{
	int chemichalID=0;
	int control_status=0;
	int SOC=0;
	int curr=0;
	int Voltage=0;
	int Temperature=0;
	int flag=0;
	int operation_config=0;
	int remain_capacity=0;
	int fullcharge_capacity=0;

	bq27520_set_cmd_write(0x00,0x08);
	bq27520_set_cmd_write(0x01,0x00);
	bq27520_set_cmd_read(0x00,&chemichalID);
	bq27520_set_cmd_write(0x00,0x00);
	bq27520_set_cmd_write(0x01,0x00);
	bq27520_set_cmd_read(0x00,&control_status);
	bq27520_set_cmd_read(BQ27520_CMD_StateOfCharge,&SOC);
	bq27520_set_cmd_read_current(&curr);
	bq27520_set_cmd_read(BQ27520_CMD_Voltage,&Voltage);
	bq27520_set_cmd_read(BQ27520_CMD_Temperature,&Temperature);
	bq27520_set_cmd_read(0x0a,&flag);
	bq27520_set_cmd_read(0x3a,&operation_config);
	bq27520_set_cmd_read(0x10,&remain_capacity);
	bq27520_set_cmd_read(0x12,&fullcharge_capacity);
	printk("[chemichalID %d] [control_status %d] [SOC %d] [current %d] [Voltage %d] [Temperature %d] [flag %d] [operation_config %d] [remain_capacity %d] [fullcharge_capacity %d]\n",
			chemichalID,control_status,SOC,curr,Voltage,Temperature,flag,operation_config,remain_capacity,fullcharge_capacity);
}

static int bq27520_push_table(struct bq27520_setting_table *init_table,unsigned int count)
{
	unsigned int i,j;
	unsigned int flag,addr,cmd,offset,data;
	int returnData=0xFF;
	for(i=0;i<count;i++)
	{
		flag=init_table[i].flag;
		addr=init_table[i].addr;
		offset=init_table[i].offset;
		switch (flag) {
		case BQ27520_W:
			if (offset==1)
			{
				cmd=init_table[i].cmd;
				data=init_table[i].data[0];
				bq27520_set_cmd_write_ROM(addr,cmd,data);
			}
			else
			{
				/*	for (j=0;j<offset;j++)
					{
					cmd=init_table[i].cmd+j;
					data=init_table[i].data[j];
					bq27520_set_cmd_write_ROM(addr,cmd,data);
					}
					*/
				cmd=init_table[i].cmd;
				bq27520_DMA_write_ROM(addr,cmd,init_table[i].data,offset);
			}
			break;
		case BQ27520_X:
			mdelay(addr);
			break;
		case BQ27520_C:
			if (offset==1)
			{
				cmd=init_table[i].cmd;
				data=init_table[i].data[0];
				bq27520_set_cmd_read_ROM(addr,cmd,&returnData);
				if(returnData!=data)
				{
					printk("bq27520_init push table err cmd=0x%x\n\r",cmd);
					msleep(50);
					return 0;
				}
				else
				{
					//printk("bq27520_init push table cmd=0x%x\n\r",cmd);
				}
			}
			else
			{
				for (j=0;j<offset;j++)
				{
					cmd=init_table[i].cmd+j;
					data=init_table[i].data[j];
					bq27520_set_cmd_read_ROM(addr,cmd,&returnData);
					if(returnData!=data)
					{
						printk("bq27520_init push table err cmd=0x%x\n\r",cmd);
						msleep(50);
						return 0;
					}
					else
					{
						//printk("bq27520_init push table cmd=0x%x\n\r",cmd);
					}
				}
			}
			break;
		default:
			break;
		}
	}
	printk("zlog bq27520 push end\n");
	return 1;
#if 0
	bq27520_set_cmd_write_ROM(0x16,0x00,0x0f);
	bq27520_set_cmd_write_ROM(0x16,0x64,0x0F);
	bq27520_set_cmd_write_ROM(0x16,0x65,0x00);
	msleep(4000);
#endif
}
static ssize_t bq27520_show_control(struct device_driver *ddri, char *buf)
{
	int len = 0;
	len = snprintf(buf, PAGE_SIZE, "'1' IT enable, '0' outof ROM mode\n");
	return len;
}
static ssize_t bq27520_mode_control(struct device_driver *ddri, char *buf, size_t count)
{
	if(buf == NULL)
		return 0;
	if(buf[0] == '0'){
		bq27520_set_cmd_write_ROM(0x16,0x00,0x0f);
		bq27520_set_cmd_write_ROM(0x16,0x64,0x0F);
		bq27520_set_cmd_write_ROM(0x16,0x65,0x00);
		msleep(4000);

	}else if(buf[0] == '1'){
		bq27520_set_cmd_write_ROM(0xaa,0x00,0x41);
		bq27520_set_cmd_write_ROM(0xaa,0x01,0x00);
		mdelay(4000);
		bq27520_set_cmd_write_ROM(0xAA,0x00,0x21);
		bq27520_set_cmd_write_ROM(0xAA,0x01,0x00);
	}
	return count;
}
static ssize_t bq27520_show_capacity(struct device_driver *ddri, char *buf)
{
	int len = 0;
	int returndata=0;
	bq27520_set_cmd_read(BQ27520_DESIGN_CAPACITY,&returndata);
	len = snprintf(buf, PAGE_SIZE, "%d\n", returndata);
	return len;
}

static ssize_t bq27520_show_version(struct device_driver *ddri, char *buf)
{
	int len = 0;
	int returndata=0;
	bq27520_set_cmd_write(0x3e,0x39);
	bq27520_set_cmd_write(0x3f,0x00);
	mdelay(2);
	bq27520_set_cmd_read(0x40,&returndata);
	len += snprintf(buf+len, PAGE_SIZE-len, "firmware_version: 0x%x\n", returndata);
	return len;
}
static ssize_t bq27520_show_current(struct device_driver *ddri, char *buf)
{
	int len = 0;
	len += snprintf(buf+len, PAGE_SIZE-len, "MTK_CURRENT: 0x%x\n", mtk_current);
	return len;
}

static ssize_t bq27520_show_DebugInfo(struct device_driver *ddri, char *buf)
{
	int len = 0;
	int returndata=0;
	bq27520_set_cmd_write(0x3e,0x52);
	bq27520_set_cmd_write(0x3f,0x00);
	bq27520_set_cmd_read(0x40,&returndata);
	len += snprintf(buf+len, PAGE_SIZE-len, "Qmax: 0x%x\n", returndata);

	bq27520_set_cmd_write(0x00,0x08);
	bq27520_set_cmd_write(0x01,0x00);
	bq27520_set_cmd_read(0x00,&returndata);
	len += snprintf(buf+len, PAGE_SIZE-len, "chemichalID: 0x%x\n", returndata);

	bq27520_set_cmd_write(0x00,0x00);
	bq27520_set_cmd_write(0x01,0x00);
	bq27520_set_cmd_read(0x00,&returndata);
	len += snprintf(buf+len, PAGE_SIZE-len, "control status: 0x%x\n", returndata);

	bq27520_set_cmd_read(BQ27520_DESIGN_CAPACITY,&returndata);
	len += snprintf(buf+len, PAGE_SIZE-len, "capacity: 0x%x\n", returndata);

	bq27520_set_cmd_read(BQ27520_CMD_StateOfCharge,&returndata);
	len += snprintf(buf+len, PAGE_SIZE-len, "SOC: 0x%x\n", returndata);

	bq27520_set_cmd_read_current(&returndata);	
	len += snprintf(buf+len, PAGE_SIZE-len, "current: %d\n", returndata);

	bq27520_set_cmd_read(BQ27520_CMD_Voltage,&returndata);
	len += snprintf(buf+len, PAGE_SIZE-len, "Voltage: 0x%x\n", returndata);

	bq27520_set_cmd_read(BQ27520_CMD_Temperature,&returndata);
	len += snprintf(buf+len, PAGE_SIZE-len, "Temperature: 0x%x\n", returndata);

	bq27520_set_cmd_read(0x0a,&returndata);
	len += snprintf(buf+len, PAGE_SIZE-len, "flag: 0x%x\n", returndata);

	bq27520_set_cmd_read(0x3a,&returndata);
	len += snprintf(buf+len, PAGE_SIZE-len, "operation config: 0x%x\n", returndata);

	bq27520_set_cmd_read(0x10,&returndata);
	len += snprintf(buf+len, PAGE_SIZE-len, "remain capacity: 0x%x\n", returndata);

	bq27520_set_cmd_read(0x12,&returndata);
	len += snprintf(buf+len, PAGE_SIZE-len, "fullcharge capacity: 0x%x\n", returndata);
	return len;
}
static ssize_t bq27520_debug_open(struct device_driver *ddri, char *buf, size_t count)
{
	if(buf == NULL)
		return 0;
	 if(buf[0] == '1' && BQ27520_DEBUG == 0){
		 BQ27520_DEBUG = 1;
	}
	return count;
}
static ssize_t bq27520_show_upgrade_status(struct device_driver *ddri, char *buf)
{
	int len = 0;
	len = snprintf(buf, PAGE_SIZE, "%d\n", Bq27520_Exist);
	return len;
}

static ssize_t bq27520_write_init_para(struct device_driver *ddri, char *buf, size_t count)
{
	int size1,size2;
	int ret=0;
	int returndata = 0;
	int c_number=0;
	int capacity_Data=0;
	int firmware_version=0;
	if(buf == NULL)
		return 0;
	if(buf[0] == '1'){
		Bq27520_Exist = 0;
		size1=sizeof(bq27520_GotoRomMode_table)/sizeof(struct bq27520_setting_table);
		size2=sizeof(bq27520_init_table)/sizeof(struct bq27520_setting_table);
		/*ret = bq27520_set_cmd_read(BQ27520_DESIGN_CAPACITY,&capacity_Data);
		bq27520_set_cmd_write(0x3e,0x39);
		bq27520_set_cmd_write(0x3f,0x00);
		mdelay(2);
		ret = bq27520_set_cmd_read(0x40,&firmware_version);*/
		//printk("zlog bq27520 capacity =%d  firmware_version =%d\n", capacity_Data,firmware_version);
		/*if(capacity_Data == 2400 && firmware_version == 0)
		{
			Bq27520_Exist = 1;
			return count;
		}
		else*/
		{
			bq27520_push_table(bq27520_GotoRomMode_table,size1);
__begin:
			ret=bq27520_push_table(bq27520_init_table,size2);
			if(!ret)
			{
				c_number++;
				if(c_number < 5){
					goto __begin;
				}
				return -1;
			}
		}
		bq27520_set_cmd_write_ROM(0xaa,0x00,0x41);
		bq27520_set_cmd_write_ROM(0xaa,0x01,0x00);
		mdelay(4000);
		bq27520_set_cmd_write_ROM(0xaa,0x00,0x21);
		bq27520_set_cmd_write_ROM(0xaa,0x01,0x00);
		Bq27520_Exist = 1;
		}
	return count;
}
static ssize_t bq27520_upgrade_init_para(struct device_driver *ddri, char *buf, size_t count)
{
	int size1,size2;
	int ret=0;
	int returndata = 0;
	int c_number=0;
	int capacity_Data=0;
	int firmware_version=0;
	if(buf == NULL)
		return 0;
	if(buf[0] == '1'){
		size1=sizeof(bq27520_GotoRomMode_table)/sizeof(struct bq27520_setting_table);
		size2=sizeof(bq27520_init_table)/sizeof(struct bq27520_setting_table);
		ret = bq27520_set_cmd_read(BQ27520_DESIGN_CAPACITY,&capacity_Data);
		bq27520_set_cmd_write(0x3e,0x39);
		bq27520_set_cmd_write(0x3f,0x00);
		mdelay(2);
		ret = bq27520_set_cmd_read(0x40,&firmware_version);
		printk("zlog bq27520 capacity =%d  firmware_version =%d\n", capacity_Data,firmware_version);
		if((capacity_Data == 2400) && (firmware_version < 4))
		{
			Bq27520_Exist = 0;
			bq27520_push_table(bq27520_GotoRomMode_table,size1);
__begin:
			ret=bq27520_push_table(bq27520_init_table,size2);
			if(!ret)
			{
				c_number++;
				if(c_number < 5){
					goto __begin;
				}
				return -1;
			}
	bq27520_set_cmd_write_ROM(0xaa,0x00,0x41);
	bq27520_set_cmd_write_ROM(0xaa,0x01,0x00);
	mdelay(4000);
	bq27520_set_cmd_write_ROM(0xaa,0x00,0x21);
	bq27520_set_cmd_write_ROM(0xaa,0x01,0x00);
		Bq27520_Exist = 1;
		}
	}
	return count;
}

static DRIVER_ATTR(bq27520_debuginfo,S_IWUSR | S_IRUGO,bq27520_show_DebugInfo,bq27520_debug_open);
static DRIVER_ATTR(bq27520_capacity,S_IWUSR | S_IRUGO,bq27520_show_capacity,NULL);
static DRIVER_ATTR(bq27520_version,S_IWUSR | S_IRUGO,bq27520_show_version,NULL);
static DRIVER_ATTR(bq27520_current,S_IWUSR | S_IRUGO,bq27520_show_current,NULL);
static DRIVER_ATTR(bq27520_control,S_IWUSR | S_IRUGO,bq27520_show_control,bq27520_mode_control);
static DRIVER_ATTR(bq27520_upgrade_status,S_IWUSR | S_IRUGO,bq27520_show_upgrade_status,NULL);
static DRIVER_ATTR(bq27520_upgrade,S_IWUSR | S_IRUGO,NULL,bq27520_write_init_para);
static DRIVER_ATTR(bq27520_auto_upgrade,S_IWUSR | S_IRUGO,NULL,bq27520_upgrade_init_para);
/*----------------------------------------------------------------------------*/
static struct device_attribute *bq27520_attr_list[] = {
	&driver_attr_bq27520_debuginfo,
	&driver_attr_bq27520_capacity,
	&driver_attr_bq27520_version,
	&driver_attr_bq27520_current,
	&driver_attr_bq27520_control,
	&driver_attr_bq27520_upgrade,
	&driver_attr_bq27520_auto_upgrade,
	&driver_attr_bq27520_upgrade_status,
};

static int bq27520_create_attr(struct device_driver *driver)
{
	int idx, err = 0;
	int num = (int)(sizeof(bq27520_attr_list)/sizeof(bq27520_attr_list[0]));
	if (driver == NULL)
	{
		return -EINVAL;
	}

	for(idx = 0; idx < num; idx++)
	{
		if(err = driver_create_file(driver, bq27520_attr_list[idx]))
		{
			printk("driver_create_file (%s) = %d\n", bq27520_attr_list[idx]->attr.name, err);
			break;
		}
	}
	return err;
}
static int bq27520_delete_attr(struct device_driver *driver)
{
	int idx ,err = 0;
	int num = (int)(sizeof(bq27520_attr_list)/sizeof(bq27520_attr_list[0]));

	if (!driver)
		return -EINVAL;

	for (idx = 0; idx < num; idx++)
	{
		driver_remove_file(driver, bq27520_attr_list[idx]);
	}

	return err;
}
/*----------------------------------------------------------------------------*/

static int bq27520_driver_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int err=0;
	int size1,size2;
	int ret=0;
	int c_number=0;
	int firmware_version=0;
	int returndata;
	int capacity_Data=0;
    printk("[bq27520_driver_probe] \n");

    if (!(new_client = kmalloc(sizeof(struct i2c_client), GFP_KERNEL))) {
        err = -ENOMEM;
        goto exit;
    }
    memset(new_client, 0, sizeof(struct i2c_client));

    new_client = client;

#if 1
	//size=sizeof(bq27520_init_table)/sizeof(struct bq27520_setting_table);
	//out Rom mode
	/*bq27520_set_cmd_write_ROM(0x16,0x00,0x0f);
	bq27520_set_cmd_write_ROM(0x16,0x64,0x0F);
	bq27520_set_cmd_write_ROM(0x16,0x65,0x00);
	msleep(4000);*/
	//read firmware version
	bq27520_set_cmd_write(0x3e,0x39);
	bq27520_set_cmd_write(0x3f,0x00);
	mdelay(2);
	bq27520_set_cmd_read(0x40,&firmware_version);
	ret = bq27520_set_cmd_read(BQ27520_DESIGN_CAPACITY,&capacity_Data);
	if(!ret)
	{
		printk("zlog bq27520 read capacity err\n");
		return -1;
	}
/*	if(firmware_version == 0x00 && capacity_Data == 2400)
	{
		size1=sizeof(bq27520_GotoRomMode_table)/sizeof(struct bq27520_setting_table);
		size2=sizeof(bq27520_init_table)/sizeof(struct bq27520_setting_table);
		bq27520_push_table(bq27520_GotoRomMode_table,size1);
__begin:
		ret=bq27520_push_table(bq27520_init_table,size2);
		if(!ret)
		{
			c_number++;
			if(c_number < 5){
				goto __begin;
			}
			return -1;
		}
		mdelay(5);
		bq27520_set_cmd_write_ROM(0xaa,0x00,0x21);
		bq27520_set_cmd_write_ROM(0xaa,0x01,0x00);
	}*/
		Bq27520_Exist = 1;
#endif
	//bq27520_set_cmd_write_ROM(0xaa,0x00,0x21);
	//bq27520_set_cmd_write_ROM(0xaa,0x01,0x00);
	if(err = bq27520_create_attr(&bq27520_dev_driver.driver))
	{
		printk("create attribute err = %d\n", err);
	}

    return 0;
exit:
    return err;
}
static int bq27520_i2c_remove(struct i2c_client *client)
{
	int err;
	err = bq27520_delete_attr(&bq27520_dev_driver.driver);
	if(err)
	{
		printk("bq27520 delete_attr fail: %d\n", err);
	}
	return 0;
}

static int bq27520_probe(struct platform_device *pdev)
{
     if(i2c_add_driver(&bq27520_driver)!=0)
    {
        printk("[bq27520_init] failed to register bq27520 i2c driver.\n");
    }

	return 0;
}
static int bq27520_remove(struct platform_device *pdev)
{
	i2c_del_driver(&bq27520_driver);
	return 0;
}
static int __init bq27520_init(void)
{
	int retval = -1;
	i2c_register_board_info(BQ27520_BUSNUM, &i2c_bq27520, 1);
	if(platform_driver_register(&bq27520_dev_driver) < 0)
	{
		printk("add bq27520 driver failed\n");
		return -1;
	}
	return 0;
}

static void __exit bq27520_exit(void)
{
    i2c_del_driver(&bq27520_driver);
}
module_init(bq27520_init);
module_exit(bq27520_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("I2C bq27520 Driver");
MODULE_AUTHOR("zhangxinyu@vanzotec.com>");
