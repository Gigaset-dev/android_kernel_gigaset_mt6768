/******************************************************************************
 * cw_scannerctrl.c - MT6537 Android Linux hall Device Driver
 *
 * Copyright 2009-2010 MediaTek Co.,Ltd.
 *
 * DESCRIPTION:
 *     This file provid the other drivers hall relative functions
 *
 ******************************************************************************/
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/miscdevice.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/proc_fs.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/mm.h>
#include <linux/interrupt.h>
#include <linux/vmalloc.h>
#include <linux/wait.h>
#include <linux/spinlock.h>
#include <linux/ctype.h>
#include <linux/semaphore.h>
#include <linux/io.h>
#include <linux/workqueue.h>
#include <linux/kdev_t.h>
#include <linux/kthread.h>
#include <linux/input.h>
#include <linux/time.h>
#include <linux/string.h>
#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/atomic.h>
#include <linux/types.h>
#include <mt-plat/mtk_boot.h>
#include "../../mediatek/imgsensor/inc/kd_camera_typedef.h"
#ifdef CONFIG_COMPAT
#include <linux/fs.h>
#include <linux/compat.h>
#endif


struct i2c_data {
	unsigned int slave_addr;
	unsigned int reg;
	unsigned int data;
};

#define IOCTL_TYPE 0x62

#define CW_SCANNERCTRL_I2C_REG16_DATA8_WRITE		_IOW(IOCTL_TYPE, 1, struct i2c_data)
#define CW_SCANNERCTRL_I2C_REG16_DATA8_READ			_IOR(IOCTL_TYPE, 2, struct i2c_data)

#define CW_SCANNERCTRL_I2C_REG8_DATA8_WRITE			_IOW(IOCTL_TYPE, 3, struct i2c_data)
#define CW_SCANNERCTRL_I2C_REG8_DATA8_READ			_IOR(IOCTL_TYPE, 4, struct i2c_data)
#define CW_SCANNERCTRL_I2C_REG8_DATA16_READ			_IOR(IOCTL_TYPE, 5, struct i2c_data)

//drv add by lipengpeng 20230906 start
#if defined(CONFIG_PRIZE_CW9281_SCANNER_CTRL)
extern int scan_vcc_3v3_en_high;
extern int scan_vcc_3v3_en_low;
extern int scan_vled_3v3_high;
extern int scan_vled_3v3_low;
extern int scan_gpio_set(int i);
#endif
//drv add by lipengpeng 20230906 end 
struct miscdevice miscdev;


#define SENSORDB(fmt, arg...) printk("%s: " fmt "\n", __FUNCTION__ ,##arg)

//drv add by lipengpeng 20230906 start
#if defined(CONFIG_PRIZE_CW9281_SCANNER_CTRL)
static int cw_scannerctrl_setGpio(int high)
{
    int ret = 0;



    if (high)
    {
		scan_gpio_set(scan_vcc_3v3_en_high);
		mdelay(5);
		scan_gpio_set(scan_vled_3v3_high);
    }
    else
    {
		scan_gpio_set(scan_vcc_3v3_en_low);
		mdelay(5);
		scan_gpio_set(scan_vled_3v3_low);
    }
    return ret;
}
#endif
//drv add by lipengpeng 20230906 end 

extern int iReadReg(u16 a_u2Addr , u8 * a_puBuff , u16 i2cId);
extern int iWriteReg(u16 a_u2Addr , u32 a_u4Data , u32 a_u4Bytes , u16 i2cId);
extern int iReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 * a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
extern int iWriteRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u16 i2cId);

void i2c_write_reg16_data8(u16 slave_addr, u16 reg, u32 data)    //8bit slave_addr
{
	char pu_send_cmd[3] = { (char)(reg >> 8), (char)(reg & 0xFF), (char)(data & 0xFF) };

	iWriteRegI2C(pu_send_cmd, 3, slave_addr);
}

kal_uint16 i2c_read_reg16_data8(u16 slave_addr, kal_uint32 reg)   //8bit slave_addr
{
	kal_uint16 get_byte = 0;

	char pu_send_cmd[2] = { (char)(reg >> 8), (char)(reg & 0xFF) };

	iReadRegI2C(pu_send_cmd, 2, (u8 *) &get_byte, 1, slave_addr);

	return get_byte;
}


void i2c_write_reg8_data8(u16 slave_addr, u16 reg, u32 data)    //8bit slave_addr
{
	char puSendCmd[2] = {(char)(reg & 0xFF) ,((char)(data & 0xFF))};
		
	iWriteRegI2C(puSendCmd, 2, slave_addr);
}

kal_uint16 i2c_read_reg8_data8(u16 slave_addr, u16 reg)    //8bit slave_addr
{
	kal_uint16 get_byte = 0;
	
	char puSendCmd[1] = {(char)(reg & 0xFF)};
//	char purecData[3] = {0x00};
		
	iReadRegI2C(puSendCmd, 1, (u8 *) &get_byte, 1, slave_addr);
	
	return get_byte;
}

kal_uint16 i2c_read_reg8_data16(u16 slave_addr, u16 reg)    //8bit slave_addr
{
	kal_uint16 get_byte = 0;
	
	char puSendCmd[1] = {(char)(reg & 0xFF)};
	char purecData[3] = {0x00};
		
	iReadRegI2C(puSendCmd, 1, purecData, 2, slave_addr);
	get_byte = purecData[0]<<8 | purecData[1];
	
	return get_byte;
}


static long cw_scannerctrl_ioctl(struct file *fp, unsigned int cmd, unsigned long args)
{
    int ret = 0;  

    struct i2c_data *registerinf = (void*)args;
    struct i2c_data reginf;

    SENSORDB("%s cmd %d, args %d\n", __FUNCTION__, cmd, (unsigned int)args);
	
	if (copy_from_user(&reginf, registerinf, sizeof(struct i2c_data))){
		SENSORDB("%s Error copy argument\n", __FUNCTION__);
		return -EFAULT;
	}

    switch (cmd)
    {
		case CW_SCANNERCTRL_I2C_REG16_DATA8_WRITE:
					SENSORDB("%s CW_SCANNERCTRL_I2C_REG16_DATA8_WRITE\n", __FUNCTION__);
			
			i2c_write_reg16_data8(reginf.slave_addr, reginf.reg, reginf.data);
			
			break;
		case CW_SCANNERCTRL_I2C_REG16_DATA8_READ:
					SENSORDB("%s CW_SCANNERCTRL_SENSOR_IIC_READ\n", __FUNCTION__);
					reginf.data = i2c_read_reg16_data8(reginf.slave_addr, reginf.reg);
			ret = copy_to_user((void __user *)args, &reginf, sizeof(struct i2c_data));
			break;
		case CW_SCANNERCTRL_I2C_REG8_DATA8_WRITE:
					SENSORDB("%s CW_SCANNERCTRL_I2C_REG8_DATA8_WRITE\n", __FUNCTION__);
					
			i2c_write_reg8_data8(reginf.slave_addr, reginf.reg, reginf.data);
			
			break;
		case CW_SCANNERCTRL_I2C_REG8_DATA8_READ:
			reginf.data = i2c_read_reg8_data8(reginf.slave_addr, reginf.reg);
			ret = copy_to_user((void __user *)args, &reginf, sizeof(struct i2c_data));
			//pr_err("CW_SCANNERCTRL_I2C_REG8_DATA8_READ reg:0x%x, val: 0x%x\n", reginf.reg, reginf.data);
			break;
			
		case CW_SCANNERCTRL_I2C_REG8_DATA16_READ:
			reginf.data = i2c_read_reg8_data16(reginf.slave_addr, reginf.reg);
			ret = copy_to_user((void __user *)args, &reginf, sizeof(struct i2c_data));
			//pr_err("CW_SCANNERCTRL_I2C_REG8_DATA8_READ reg:0x%x, val: 0x%x\n", reginf.reg, reginf.data);
			break;
			
		default:
			break;
    }

    if ( ret == 0 )
        return 0;
    else
        return 1;
}

static int cw_scannerctrl_open(struct inode *inode, struct file *fp)
{
    printk("%s\n", __FUNCTION__);
//drv add by lipengpeng 20230906 start
#if defined(CONFIG_PRIZE_CW9281_SCANNER_CTRL)	
	cw_scannerctrl_setGpio(1);//power
#endif
//drv add by lipengpeng 20230906 end 
    return 0;
}

static int cw_scannerctrl_release(struct inode *inode, struct file *fp)
{
//drv add by lipengpeng 20230906 start
#if defined(CONFIG_PRIZE_CW9281_SCANNER_CTRL)	
	cw_scannerctrl_setGpio(0);//power
#endif
//drv add by lipengpeng 20230906 end 
    return 0;
}

static const struct file_operations cw_scannerctrl_fops = {
    .owner           = THIS_MODULE,
    .unlocked_ioctl  = cw_scannerctrl_ioctl,
    .open            = cw_scannerctrl_open,
    .release         = cw_scannerctrl_release,
};

static struct miscdevice cw_scannerctrl_misc_device = {
        .minor			= MISC_DYNAMIC_MINOR,
        .name			= "cw_scannerctrl",
        .fops			= &cw_scannerctrl_fops,
};

static int __init cw_scannerctrl_init(void)
{
    printk("%s\n", __FUNCTION__);
	misc_register(&cw_scannerctrl_misc_device);
	return 0;
}

static void __exit cw_scannerctrl_cleanup(void)
{
    misc_deregister(&cw_scannerctrl_misc_device);
}

module_init(cw_scannerctrl_init);
module_exit(cw_scannerctrl_cleanup);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("CW");
MODULE_DESCRIPTION("cw_scannerctrl driver");