#include <linux/init.h>		/* For init/exit macros */
#include <linux/module.h>	/* For MODULE_ marcros  */
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/workqueue.h>
#include <linux/fb.h>
#include <linux/notifier.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/i2c.h>
#include <linux/of_irq.h>
#include <linux/interrupt.h>
#include <linux/kthread.h>
#include <linux/mutex.h>
#include <linux/regulator/consumer.h>
#include <linux/kthread.h>

#include "gc9a01_i2c.h"
#include "focaltech_core.h"

#define DEVICE_NAME  "gc9a01_iic"

#ifndef GC9A01_SYS_TEST
#define GC9A01_SYS_TEST
#endif

struct gc9a01_data * g_gc9a01_data = NULL;
/* For abnormal check */
static struct task_struct *ft3311_touch_esd_check;
/*
static unsigned int gc9a01_tp_reset = 0;
static unsigned int gc9a01_tp_int = 0;
static int tp_irq = 0;
static int tpd_flag = 0;
static struct input_dev *gc9a01_dev;
struct i2c_client *g_client = NULL;
//static struct delayed_work delay_work;
static DECLARE_WAIT_QUEUE_HEAD(waiter);
static DEFINE_MUTEX(i2c_access);
*/

/*****************************************************************************
* Included header files
*****************************************************************************/
#include "focaltech_core.h"

/*****************************************************************************
* Private constant and macro definitions using #define
*****************************************************************************/
#define FTS_CMD_START_DELAY                 		12

/* register address */
#define FTS_REG_WORKMODE                    		0x00
#define FTS_REG_WORKMODE_FACTORY_VALUE      		0x40
#define FTS_REG_WORKMODE_SCAN_VALUE					0xC0
#define FTS_REG_FLOW_WORK_CNT               		0x91
#define FTS_REG_POWER_MODE                  		0xA5
#define FTS_REG_GESTURE_EN                  		0xD0
#define FTS_REG_GESTURE_ENABLE              		0x01
#define FTS_REG_GESTURE_OUTPUT_ADDRESS      		0xD3

/*Max point numbers of gesture trace*/
#define MAX_POINTS_GESTURE_TRACE                	6
/*Length of gesture information*/
#define MAX_LEN_GESTURE_INFO            			(MAX_POINTS_GESTURE_TRACE * 4 + 2)

/*Max point numbers of touch trace*/
#define MAX_POINTS_TOUCH_TRACE                  	2
/*Length of touch information*/
#define MAX_LEN_TOUCH_INFO            			    (MAX_POINTS_TOUCH_TRACE * 6 + 2)

/*Max touch points that touch controller supports*/
#define	FTS_MAX_POINTS_SUPPORT						10

/*****************************************************************************
* Private variables/functions
*****************************************************************************/
static struct fts_ts_data _fts_data = {
    .suspended = 0,
    .gesture_support = FTS_GESTURE_EN,
    .esd_support = 0,
};


/*****************************************************************************
* Global variable or extern global variabls/functions
*****************************************************************************/
struct fts_ts_data *fts_data = &_fts_data;

/*delay, unit: millisecond */
void fts_msleep(unsigned long msec)
{
    msleep(msec);
}

/*****************************************************************************
* reset chip
*****************************************************************************/
static int fts_hw_reset(uint8_t msec)
{

	if(!g_gc9a01_data){
		pr_err("gc_data was null,return...\n");
	}
    /*firsty. set reset_pin low, and then delay 10ms*/
    /*secondly. set reset_pin high, and then delay 200ms*/
	gpio_direction_output(g_gc9a01_data->gc9a01_tp_reset,0);
	msleep(10);
	gpio_direction_output(g_gc9a01_data->gc9a01_tp_reset,1);
	msleep(200);

    return 0;
}

/*****************************************************************************
* Initialize i2c
*****************************************************************************/
static int platform_i2c_init(void)
{
    /*Initialize I2C bus, you should implement it based on your platform*/
    return 0;
}

/*****************************************************************************
* Initialize reset pin
*****************************************************************************/
static int platform_reset_pin_cfg(void)
{
    /*Initialize reset_pin,  you should implement it based on your platform*/

    /*firstly,set the reset_pin to output mode*/
    /*secondly,set the reset_pin to low */
    return 0;
}

/*****************************************************************************
* Initialize gpio interrupt, and set trigger mode to falling edge.
*****************************************************************************/
static int platform_interrupt_gpio_init(void)
{
    /*Initialize gpio interrupt , and the corresponding interrupt function is fts_gpio_interrupt_handler,
    you should implement it based on your platform*/


    /*firstly,set int_pin to input mode with pull-up*/
    /*secondly,and set int_pin's trigger mode to falling edge trigger */
    return 0;
}

/*****************************************************************************
* TP power on
*****************************************************************************/
static void fts_power_on(void)
{
    /*refer to ic datasheet*/
}

/*****************************************************************************
* Initialize timer and set to interrupt mode which period is 1 second
*****************************************************************************/
static int platform_interrupt_timer_init(void)
{

    /*Initialize timer and set to interrupt mode which period is 1 second,
    and the corresponding interrupt function is fts_timer_interrupt_handler,
    you should implement it based on your platform*/
    return 0;
}

/*****************************************************************************
* Name: fts_write
* Brief:
*   Write function via I2C bus, you should implement it based on your platform.
*
*   The code below is only a sample code, a pseudo code. you must implement it
*  following standard I2C protocol based on your platform
*
*
* Input: @addr: the command or register address
*        @data: the data buffer, data buffer can be NULL for commands without data fields.
*        @datalen: length of data buffer
* Output:
* Return:
*   return 0 if success, otherwise error code.
*****************************************************************************/
int fts_write(uint8_t addr, uint8_t *data, uint16_t datalen)
{
    /*TODO based your platform*/
    int ret = 0;
    uint8_t txbuf[256] = { 0 };
    uint16_t txlen = 0;
    int i = 0;

    if ( datalen >= 256 ) {
        FTS_ERROR("txlen(%d) fails", datalen);
        return -1;
    }

    memcpy(&txbuf[0], &addr, 1);
    txlen = 1;
    if (data && datalen) {
        memcpy(&txbuf[txlen], data, datalen);
        txlen += datalen;
    }

    /*call platform_i2c_write function to transfer I2C package to TP controller
     *platform_i2c_write() is different for different platform, based on your platform.
     */
    for(i = 0; i < 3; i++) {
        //ret = i2c_transfer(g_gc9a01_data ->client->adapter, &msgs, 1);
		ret = i2c_smbus_write_i2c_block_data(g_gc9a01_data ->client, addr, txlen, txbuf);
        if (ret < 0) {
            FTS_ERROR("platform_i2c_write(%d) fails,ret:%d,retry:%d", addr, ret, i);
            continue;
        } else {
            ret = 0;
            break;
        }
    }

    return ret;

}

/*****************************************************************************
* Name: fts_read
* Brief:
*   Read function via I2C bus, you should implement it based on your platform.
*
*   The code below is only a sample code, a pseudo code. you must implement it
*  following standard I2C protocol based on your platform
*
*
* Input: @addr: the command or register data
*        @datalen: length of data buffer
* Output:
*        @data: the data buffer read from TP controller
* Return:
*   return 0 if success, otherwise error code.
*****************************************************************************/
int fts_read(uint8_t addr, uint8_t *data, uint16_t datalen)
{
    /*TODO based your platform*/
    int ret = 0;
    int i = 0;

    if (!data || !datalen) {
        FTS_ERROR("data is null, or datalen is 0");
        return -1;
    }

    for(i = 0; i < 3; i++) {
        /*call platform_i2c_write function to transfer I2C package to TP controller
         *platform_i2c_write() is different for different platform, based on your platform.
         */
        //ret = platform_i2c_write(&addr, 1);
        //if (ret < 0
            //FTS_ERROR("platform_i2c_write(%d) fails,ret:%d,retry:%d", addr, ret, i);
            //continue;
        //}


        /*call platform_i2c_read function to transfer I2C package to read data from TP controller
         *platform_i2c_read() is different for different platform, based on your platform.
         */

        //ret = platform_i2c_read(data, datalen);
		ret = i2c_smbus_read_i2c_block_data(g_gc9a01_data ->client, addr, datalen, data);
		printk("gc9a01 addr 0x%2x = 0x%2x\n", addr, data[0]);
        if (ret < 0) {
            FTS_ERROR("platform_i2c_read fails,addr:0x%2x ret:%d, retry:%d", addr, ret, i);
            continue;
        }

        ret = 0;
        break;
    }

    return ret;
}

/*****************************************************************************
* Name: fts_write_reg
* Brief:
*   write function via I2C bus, you should implement it based on your platform.
*
*   The code below is only a sample code, a pseudo code. you must implement it
*  following standard I2C protocol based on your platform
*
*
* Input: @addr: the command or register address
*        @val: the data write to TP controller
* Return:
*   return 0 if success, otherwise error code.
*****************************************************************************/
int fts_write_reg(uint8_t addr, uint8_t val)
{
    return fts_write(addr, &val, 1);
}

/*****************************************************************************
* Name: fts_read_reg
* Brief:
*   read function via I2C bus, you should implement it based on your platform.
*
*   The code below is only a sample code, a pseudo code. you must implement it
*  following standard I2C protocol based on your platform
*
*
* Input: @addr: the command or register address
* Output:
*        @val: the data read from TP controller
* Return:
*   return 0 if success, otherwise error code.
*****************************************************************************/
int fts_read_reg(uint8_t addr, uint8_t *val)
{
    return fts_read(addr, val, 1);
}

/*****************************************************************************
* Name: fts_check_id
* Brief:
*   The function is used to check id.
* Input:
* Output:
* Return:
*   return 0 if check id successfully, otherwise error code.
*****************************************************************************/
static int fts_check_id(void)
{
    int ret = 0;

    uint8_t chip_id[2] = { 0 };

	printk("%s----%d-----\n",__func__,__LINE__);
	//return 0;

    /*delay 200ms,wait fw*/
    fts_msleep(200);

    /*get chip id*/
    fts_read_reg(FTS_REG_CHIP_ID, &chip_id[0]);
    fts_read_reg(FTS_REG_CHIP_ID2, &chip_id[1]);
    if ((FTS_CHIP_IDH == chip_id[0]) && (FTS_CHIP_IDL == chip_id[1])) {
        FTS_INFO("get ic information, chip id = 0x%02x%02x",  chip_id[0], chip_id[1]);
        return 0;
    }

    /*get boot id*/
    FTS_INFO("fw is invalid, need read boot id");
    ret = fts_write_reg(0x55, 0xAA);
    if (ret < 0) {
        FTS_ERROR("start cmd write fail");
        return ret;
    }

    fts_msleep(FTS_CMD_START_DELAY);
    ret = fts_read(FTS_CMD_READ_ID, chip_id, 2);
    if((ret == 0) && ((FTS_CHIP_IDH == chip_id[0]) && (FTS_CHIP_IDL == chip_id[1]))) {
        FTS_INFO("get ic information, boot id = 0x%02x%02x", chip_id[0], chip_id[1]);
        ret = 0;
    } else {
        FTS_ERROR("read boot id fail,read:0x%02x%02x", chip_id[0], chip_id[1]);
        return 0;
    }

    return ret;
}

/*****************************************************************************
*  Name: fts_esdcheck_algorithm
*  Brief:
*    The function is use to esd check.It should be called in timer interrupt handler.
*  Input:
*  Output:
*  Return:
*****************************************************************************/
static void fts_esdcheck_process(void)
{
    uint8_t reg_value = 0;
    static uint8_t flow_work_hold_cnt = 0;
    static uint8_t flow_work_cnt_last = 0;

    /* 1. check power state, if suspend, no need check esd */
    if (fts_data->suspended == 1) {
        FTS_DEBUG("In suspend, not check esd");
        /* because in suspend state, when upgrade FW, will
         * active ESD check(active = 1); when resume, don't check ESD again
         */
        return ;
    }

    /* 2. In factory mode, can't check esd */
    fts_read_reg(FTS_REG_WORKMODE, &reg_value);
    if (( reg_value == FTS_REG_WORKMODE_FACTORY_VALUE) || ( reg_value == FTS_REG_WORKMODE_SCAN_VALUE)) {
        FTS_DEBUG("in factory mode(%x), no check esd", reg_value);
        return ;
    }

    /* 3. get Flow work cnt: 0x91 If no change for 5 times, then ESD and reset */
    fts_read_reg(FTS_REG_FLOW_WORK_CNT, &reg_value);
    if (flow_work_cnt_last == reg_value)
        flow_work_hold_cnt++;
    else
        flow_work_hold_cnt = 0;

    flow_work_cnt_last = reg_value;

    /* 4. If need hardware reset, then handle it here */
    if (flow_work_hold_cnt >= 5) {
        FTS_DEBUG("ESD, Hardware Reset");
        flow_work_hold_cnt = 0;
        fts_hw_reset(200);
    }
}


/*****************************************************************************
* Name: fts_gesture_process
* Brief:
*   The function is used to read and parse gesture information. It should be
*  called in gpio interrupt handler while system is in suspend state.
* Input:
* Output:
* Return:
*   return 0 if getting and parsing gesture successfully,
*   return 1 if gesture isn't enabled in FW,
*   otherwise error code.
*****************************************************************************/
static int fts_gesture_process(void)
{
    int ret = 0;
    //uint8_t i = 0;
    //uint8_t base = 0;
    uint8_t regaddr = 0;
    uint8_t value = 0xFF;
    uint8_t buf[MAX_LEN_GESTURE_INFO] = { 0 };
    uint8_t gesture_id = 0;

    /*Read a byte from register 0xD0 to confirm gesture function in FW is enabled*/
    ret = fts_read_reg(FTS_REG_GESTURE_EN, &value);
    if ((ret <0) || (value != FTS_REG_GESTURE_ENABLE)) {
        FTS_DEBUG("gesture isn't enable in fw, don't process gesture");
        return 1;
    }

    /*Read 26 bytes from register 0xD3 to get gesture information*/
    regaddr = FTS_REG_GESTURE_OUTPUT_ADDRESS;
    ret = fts_read(regaddr, buf, MAX_LEN_GESTURE_INFO);
    if (ret < 0) {
        FTS_DEBUG("read gesture information from reg0xD3 fails");
        return ret;
    }

    /*get gesture_id, and the gestrue_id table provided by our technicians */
    gesture_id = buf[0];

    /* Now you have parsed the gesture information, you can recognise the gesture type based on gesture id.
     * You can do anything you want to do, for example,
     *     gesture id 0x24, so the gesture type id "Double Tap", now you can inform system to wake up
     *     from gesture mode.
     */

    /*TODO...(report gesture to system)*/

    return 0;
}


/*****************************************************************************
* print touch buffer read from register address 0x01
*****************************************************************************/
static void log_touch_buf(uint8_t *buf, uint32_t buflen)
{
    int i = 0;
    int count = 0;
    char tmpbuf[512] = { 0 };

    for (i = 0; i < buflen; i++) {
        count += snprintf(tmpbuf + count, 1024 - count, "%02X,", buf[i]);
        if (count >= 1024)
            break;
    }
    FTS_DEBUG("point buffer:%s", tmpbuf);
}


/*****************************************************************************
* print touch points information of this interrupt
*****************************************************************************/
static void log_touch_info(struct fts_ts_event *events, uint8_t event_nums)
{
    uint8_t i = 0;

    for (i = 0; i < event_nums; i++) {
        FTS_DEBUG("[%d][%d][%d,%d,%d]%d", events[i].id, events[i].type, events[i].x,
                  events[i].y, events[i].p, events[i].area);
    }
}

/*****************************************************************************
* Name: fts_touch_process
* Brief:
*   The function is used to read and parse touch points information. It should be
*  called in gpio interrupt handler while system is in display(normal) state.
* Input:
* Output:
* Return:
*   return 0 if getting and parsing touch points successfully, otherwise error code.
*****************************************************************************/
static int fts_touch_process(void)
{
    int ret = 0;
    uint8_t i = 0;
    uint8_t base = 0;
    uint8_t regaddr = 0x01;
    uint8_t buf[MAX_LEN_TOUCH_INFO];/*A maximum of two points are supported*/
    uint8_t point_num = 0;
    uint8_t touch_event_nums = 0;
    uint8_t point_id = 0;
    struct fts_ts_event events[FTS_MAX_POINTS_SUPPORT];    /* multi-touch */


    /*read touch information from reg0x01*/
    ret = fts_read(regaddr, buf, MAX_LEN_TOUCH_INFO);
    if (ret < 0) {
        FTS_DEBUG("Read touch information from reg0x01 fails");
        return ret;
    }

    /*print touch buffer, for debug usage*/
    log_touch_buf(buf, MAX_LEN_TOUCH_INFO);

    if ((buf[1] == 0xFF) && (buf[2] == 0xFF) && (buf[3] == 0xFF)) {
        FTS_INFO("FW initialization, need recovery");
        if (fts_data->gesture_support && fts_data->suspended)
            fts_write_reg(FTS_REG_GESTURE_EN, FTS_REG_GESTURE_ENABLE);
    }

    /*parse touch information based on register map*/
    memset(events, 0xFF, sizeof(struct fts_ts_event) * FTS_MAX_POINTS_SUPPORT);
    point_num = buf[1] & 0x0F;
    if (point_num > FTS_MAX_POINTS_SUPPORT) {
        FTS_DEBUG("invalid point_num(%d)", point_num);
        return -1;
    }

    for (i = 0; i < MAX_POINTS_TOUCH_TRACE; i++) {
        base = 2 + i * 6;
        point_id = buf[base + 2] >> 4;
        if (point_id >= MAX_POINTS_TOUCH_TRACE) {
            break;
        }

        events[i].x = ((buf[base] & 0x0F) << 8) + buf[base + 1];
        events[i].y = ((buf[base + 2] & 0x0F) << 8) + buf[base + 3];
        events[i].id = point_id;
        events[i].type = (buf[base] >> 6) & 0x03;
        events[i].p = buf[base + 4];
        events[i].area = buf[base + 5];
        if (((events[i].type == 0) || (events[i].type == 2)) & (point_num == 0)) {
            FTS_DEBUG("abnormal touch data from fw");
            return -2;
        }

        touch_event_nums++;
    }

    if (touch_event_nums == 0) {
        FTS_DEBUG("no touch point information(%02x)", buf[1]);
        return -3;
    }

    /*print touch information*/
    log_touch_info(events, touch_event_nums);

    /*Now you have get the touch information, you can report anything(X/Y coordinates...) you want to system*/
    /*TODO...(report touch information to system)*/
    /*Below sample code is a pseudo code*/
    for (i = 0; i < touch_event_nums; i++) {
        if ((events[i].type == 0) || (events[i].type == 2)) {
            /* The event of point(point id is events[i].id) is down event, the finger of this id stands for is
             * pressing on the screen.*/
            /*TODO...(down event)*/
			//input_mt_report_slot_state(g_gc9a01_data->gc9a01_dev, MT_TOOL_FINGER, true);
			input_report_abs(g_gc9a01_data->gc9a01_dev, ABS_MT_POSITION_X, events[i].x);
			input_report_abs(g_gc9a01_data->gc9a01_dev, ABS_MT_POSITION_Y, events[i].y);
			input_report_key(g_gc9a01_data->gc9a01_dev, BTN_TOOL_FINGER, 1);
        } else {
            /*TODO...(up event)*/
			//input_mt_report_slot_state(g_gc9a01_data->gc9a01_dev, MT_TOOL_FINGER, false);
			input_report_key(g_gc9a01_data->gc9a01_dev, BTN_TOOL_FINGER, 0);
        }
    }

	input_sync(g_gc9a01_data->gc9a01_dev);

    return 0;
}


/*****************************************************************************
* An interrupt handler, will be called while the voltage of INT Port changes from high to low(falling-edge trigger mode).
* The program reads touch data or gesture data from TP controller, and then sends it into system.
*****************************************************************************/
static int fts_gpio_interrupt_handler(void)
{
    int ret = 0;

    if (fts_data->gesture_support && fts_data->suspended) {
        /*if gesture is enabled, interrupt handler should process gesture at first*/
        ret = fts_gesture_process();
        if (ret == 0) {
            FTS_DEBUG("success to process gesture.");
            return 1;
        }
    }

    /*if gesture isn't enabled, the handler should process touch points*/
    fts_touch_process();

    return 0;
}


/*****************************************************************************
* An interrupt handler, will be called while the timer interrupt trigger , the code based on your platform.
* The program use to check esd.
*****************************************************************************/
static int fts_timer_interrupt_handler(void *data)
{
    /* esd check */

	while (1) {
		printk("TP ESD %s----%d-----\n",__func__,__LINE__);
		msleep(2000); /* 2s */
		fts_esdcheck_process();
	}

	return 1;
}

/*****************************************************************************
*  Name: fts_suspend
*  Brief: System suspends and update the suspended state
*  Input:
*  Output:
*  Return:
*     return 0 if enter suspend successfully, otherwise error code
*****************************************************************************/
int fts_ts_suspend(void)
{
    int ret = 0;

    if (fts_data->suspended) {
        FTS_INFO("Already in suspend state");
        return 0;
    }

    if(fts_data->gesture_support) {
        /*Host writes 0x01 to register address 0xD0 to enable gesture function while system suspends.*/
        ret = fts_write_reg(FTS_REG_GESTURE_EN, FTS_REG_GESTURE_ENABLE);
        if (ret < 0) {
            FTS_ERROR("enable gesture fails.ret:%d",ret);
        } else {
            FTS_INFO("enable gesture success.");
        }
    } else {
        /*Host writes 0x03 to register address 0xA5 to enter into sleep mode.*/
        ret = fts_write_reg(FTS_REG_POWER_MODE, 0x03);
        if(ret < 0) {
            FTS_ERROR("system enter sleep mode fails.ret:%d",ret);
        } else {
            FTS_INFO("system enter sleep mode success.");
        }
    }

    fts_data->suspended = 1;
    return 0;
}

/*****************************************************************************
*  Name: fts_resume
*  Brief: System resume and update the suspended state
*  Input:
*  Output:
*  Return:
*    return 0 if enter resume successfully, otherwise error code
*****************************************************************************/
int fts_ts_resume(void)
{
    if (!fts_data->suspended) {
        FTS_INFO("Already in resume state");
        return 0;
    }

    fts_data->suspended = 0;
    fts_hw_reset(200);

    return 0;
}

/*****************************************************************************
* Name: fts_init
* Brief:
*   The function is used to i2c init��tp_rst pin init��interrupt_pin init��timer init.
* Input:
* Output:
* Return:
*   return 0 if success, otherwise error code.
*****************************************************************************/
int fts_ts_init(void)
{
    int ret = 0;

    /*Initialize I2C*/
    ret = platform_i2c_init();
    if( ret < 0) {
        FTS_ERROR("I2C init fails.ret:%d",ret);
        return ret;
    }

    /*reset pin cfg*/
    ret = platform_reset_pin_cfg();
    if( ret < 0) {
        FTS_ERROR("reset pin init fails.ret:%d",ret);
        return ret;
    }

    /*tp power on*/
    fts_power_on();

    /*check chip id*/
    ret = fts_check_id();
    if(ret < 0) {
        FTS_ERROR("get chip id fails.ret:%d",ret);
        return ret;
    }

    /*Register gpio interrupt handler,which for touch process or gestrue process*/
    ret = platform_interrupt_gpio_init();
    if(ret < 0) {
        FTS_ERROR("Register gpio interrupt handler fails.ret:%d",ret);
        return ret;
    }

    /*Initialize timer and set to interrupt mode with one second one period, which for esd check*/
    ret = platform_interrupt_timer_init();
    if(ret < 0) {
        FTS_ERROR("Initialize timer fails.ret:%d",ret);
        return ret;
    }

    return ret;
}

static const struct of_device_id gc9a01_match_table[] = {
    {.compatible = "mediatek,gc9a01_touch",},
    { },
};

/*Define gc9a01 iic read function*/
int gc9a01_read(struct i2c_client *client, unsigned char reg, unsigned char *data)
{
	int ret = 0;
	//msleep(10);
	printk("%s----0x%x-----\n",__func__,__LINE__);	
	ret = i2c_smbus_read_i2c_block_data(client, reg, 1, data);
	printk("gc9a01 id %2x = %2x\n", reg, data[0]);
	return ret;
}

int gc9a01_read_block(struct i2c_client *client, unsigned char reg, unsigned char *data,int len)
{
	int ret = 0;
	//msleep(10);	
	ret = i2c_smbus_read_i2c_block_data(client, reg, len, data);
	//cw_printk(0,"%2x = %2x\n", reg, buf[0]);
	return ret;
}

/*Define gc9a01 iic write function*/		
int gc9a01_write(struct i2c_client *client, unsigned char reg, unsigned char *buf)
{
	int ret = 0;
	//msleep(10);	
	ret = i2c_smbus_write_i2c_block_data(client, reg, 1, buf);
	//cw_printk(0,"%2x = %2x\n", reg, buf[0]);
	return ret;
}


void gc9a01_irq_disable(struct gc9a01_data * gc_data)
{
    unsigned long irqflags;

    spin_lock_irqsave(&gc_data->irq_lock, irqflags);
	
	pr_err("------%s--------irq_disabled = %d\n",__func__,gc_data->irq_disabled);

    if (!gc_data->irq_disabled) {
        disable_irq_nosync(gc_data->tp_irq);
        gc_data->irq_disabled = true;
    }

    spin_unlock_irqrestore(&gc_data->irq_lock, irqflags);
    
}

void gc9a01_irq_enable(struct gc9a01_data * gc_data)
{
    unsigned long irqflags = 0;
	pr_err("------%s--------irq_disabled = %d\n",__func__,gc_data->irq_disabled);
    spin_lock_irqsave(&gc_data->irq_lock, irqflags);

    if (gc_data->irq_disabled) {
        enable_irq(gc_data->tp_irq);
        gc_data->irq_disabled = false;
    }
    spin_unlock_irqrestore(&gc_data->irq_lock, irqflags);

}


static void gc9a01_reset(struct gc9a01_data * gc_data)
{
	/*
	if(value){
		gpio_direction_output(gc9a01_tp_reset,1);
	}
	else{
		gpio_direction_output(gc9a01_tp_reset,0);
	}
	*/
	if(!gc_data){
		pr_err("gc_data was null,return...\n");
	}
	gpio_direction_output(gc_data->gc9a01_tp_reset,0);
	msleep(10);
	gpio_direction_output(gc_data->gc9a01_tp_reset,1);
	msleep(50);	
}


 
static int gc9a01_get_gpio(struct device *dev,struct gc9a01_data * gc_data)
{
	int ret = 0;
	const struct of_device_id *match;
	
	if (dev->of_node){
		match = of_match_device(of_match_ptr(gc9a01_match_table), dev);
		if (!match) {
			printk("Error: No device match found\n");
			return -ENODEV;
		}
	}

	gc_data->gc9a01_tp_reset = of_get_named_gpio(dev->of_node, "reset_gpio", 0);
	
	printk("%s------[gezi] gc9a01_tp_reset =  %d\n", __func__, gc_data->gc9a01_tp_reset);
	if(gc_data->gc9a01_tp_reset != 0) {
		ret = gpio_request(gc_data->gc9a01_tp_reset, "gc9a01_tp_reset");
		if (ret) 
			printk("%s------[gezi] gpio request gc9a01_tp_reset = 0x%x fail with %d\n", __func__, gc_data->gc9a01_tp_reset, ret);
	}
	
	gc_data->gc9a01_tp_int = of_get_named_gpio(dev->of_node, "irq_gpio", 0);
	
	printk("%s------[gezi] gc9a01_tp_int =  %d\n", __func__, gc_data->gc9a01_tp_int);
	if(gc_data->gc9a01_tp_int != 0) {
		ret = gpio_request(gc_data->gc9a01_tp_int, "gc9a01_tp_int");
		if (ret) 
			printk("%s------[gezi] gpio request gc9a01_tp_int = 0x%x fail with %d\n", __func__,gc_data->gc9a01_tp_int, ret);
	}
/*	
	gpio_direction_output(gc9a01_tp_reset,0);
	msleep(10);
	gpio_direction_output(gc9a01_tp_reset,1);
	msleep(50);	
*/
	//gpio_direction_output(gc9a01_tp_reset,0);
	return ret;
}


static irqreturn_t cst3xx_ts_irq_handler(int irq, void *data)
{

    //printk("-------enter cst3xx_ts_irq_handler-------%d-----\n",__gpio_get_value(gc9a01_tp_int));
	struct gc9a01_data * gc_data = data;

	//disable_irq_nosync(gc_data->tp_irq);//use in interrupt,disable_irq will make dead lock.

	gc_data->tpd_flag = 1;
	wake_up_interruptible(&gc_data->waiter);

	return IRQ_HANDLED;
}

static int gc9a01_input_init(struct gc9a01_data * gc_data)
{
	int ret = 0;
	//struct input_dev *gc9a01_dev = gc_data->gc_dev;

	printk("%s----%d-----\n",__func__,__LINE__);
	
	gc_data->gc9a01_dev = input_allocate_device();
	if (gc_data->gc9a01_dev == NULL) {
		pr_err("Failed to allocate input device for gc9a01_dev\n");
		return -1;
	}
	
	gc_data->gc9a01_dev->name = "gc9a01_touch";
	//gc_data->gc9a01_dev->evbit[0] = BIT_MASK(EV_SYN) | BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	//gc_data->gc9a01_dev->keybit[BIT_WORD(BTN_TOOL_FINGER)] = BIT_MASK(BTN_TOOL_FINGER);
	
	__set_bit(EV_REL,gc_data->gc9a01_dev->evbit);
    __set_bit(REL_X, gc_data->gc9a01_dev->relbit);
    __set_bit(REL_Y, gc_data->gc9a01_dev->relbit);
    //__set_bit(REL_Z, gc_data->gc9a01_dev->relbit);
	
    __set_bit(EV_SYN, gc_data->gc9a01_dev->evbit);
    __set_bit(EV_ABS, gc_data->gc9a01_dev->evbit);
    __set_bit(EV_KEY, gc_data->gc9a01_dev->evbit);
    __set_bit(BTN_TOOL_FINGER,gc_data->gc9a01_dev->keybit);
    __set_bit(INPUT_PROP_BUTTONPAD,gc_data->gc9a01_dev->propbit);

	input_set_abs_params(gc_data->gc9a01_dev, ABS_MT_POSITION_X, 0, 228, 0, 0);
	input_set_abs_params(gc_data->gc9a01_dev, ABS_MT_POSITION_Y, 0, 460, 0, 0);
	
	//input_set_abs_params(gc_data->gc9a01_dev, ABS_MT_TOUCH_MAJOR, 0, 0xFF, 0, 0);
	
	//input_set_abs_params(gc_data->gc9a01_dev, ABS_MT_PRESSURE, 0, 255, 0, 0);
	//input_set_abs_params(gc_data->gc9a01_dev, ABS_MT_TRACKING_ID, 0, 0x0F, 0, 0);
/*
	input_set_abs_params(gc9a01_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(gc9a01_dev, ABS_MT_TRACKING_ID, 0, 255, 0, 0);
*/

	ret = input_register_device(gc_data->gc9a01_dev);
	if (ret) {
		pr_err("Register %s input device failed", gc_data->gc9a01_dev->name);
		return -1;
	}
	
	return 0;
}

static int gc9a01_eint_setup(struct gc9a01_data * gc_data)
{
	int ret;
	//struct device_node *node;

	//usb_eint_type = IRQ_TYPE_EDGE_RISING;
	printk("%s----%d-----\n",__func__,__LINE__);
	
	gpio_direction_input(gc_data->gc9a01_tp_int);	
	
	gc_data->tp_irq = gpio_to_irq(gc_data->gc9a01_tp_int);

	pr_err("tp_irq=%d", gc_data->tp_irq);
	
	ret = request_irq(gc_data->tp_irq, cst3xx_ts_irq_handler,IRQF_TRIGGER_FALLING, "gc9a01_tp_eint_func", gc_data);
	//ret = request_irq(charge_state_irq, charge_state_eint_func,IRQ_TYPE_LEVEL_LOW, "battery_exist_eint_default", NULL);	
	if (ret > 0){
		pr_err("usb EINT IRQ LINE NOT AVAILABLE\n");
	}
	else {
		pr_err("usb eint set EINT finished, usb_irq=%d\n",gc_data->tp_irq);
	}		
	
	enable_irq_wake(gc_data->tp_irq);	

	return ret;
}

static int gc9a01_get_chip_id(struct i2c_client *client)
{
	unsigned char chip_id = 0;

	printk("%s----%d-----\n",__func__,__LINE__);

	gc9a01_read(client,FTS_REG_CHIP_ID,&chip_id);
	
	printk("[Touch sensor(gc9a01_get_chip_id) get chip id succ] = %x\n", chip_id);
	
	if(chip_id == 0xB5) {
		return 0;
	}
	
	return -1;
	
}

static void gc9a01_report(unsigned short x,unsigned short y,unsigned char mode,unsigned char finger_num)
{
	static unsigned char pressed = 0;
	//static unsigned int tracking_id = 0;
	
	if(unlikely(!g_gc9a01_data)){
		return;
	}
	//if(finger_num){
	//	input_mt_report_slot_state(g_gc9a01_data->gc9a01_dev, MT_TOOL_FINGER, true);
	//}
	//input_report_abs(g_gc9a01_data->gc9a01_dev, ABS_MT_TOUCH_MAJOR, mode);
		//input_report_abs(g_gc9a01_data->gc9a01_dev, ABS_MT_PRESSURE,mode);
	input_report_abs(g_gc9a01_data->gc9a01_dev, ABS_MT_POSITION_X, x);
	input_report_abs(g_gc9a01_data->gc9a01_dev, ABS_MT_POSITION_Y, y);

	if((!pressed) && finger_num){
		//tracking_id++;
		//input_mt_report_slot_state(g_gc9a01_data->gc9a01_dev, MT_TOOL_FINGER, true);
		//input_report_abs(g_gc9a01_data->gc9a01_dev, ABS_MT_TOUCH_MAJOR, mode);
		//input_report_abs(g_gc9a01_data->gc9a01_dev, ABS_MT_TRACKING_ID, tracking_id);
		input_report_key(g_gc9a01_data->gc9a01_dev, BTN_TOOL_FINGER, 1);
		pressed = 1;
	}

	
	if(!finger_num){
		//input_mt_report_slot_state(g_gc9a01_data->gc9a01_dev, MT_TOOL_FINGER, false);
		//input_report_abs(g_gc9a01_data->gc9a01_dev, ABS_MT_TRACKING_ID, 0xffffffff);
		input_report_key(g_gc9a01_data->gc9a01_dev, BTN_TOOL_FINGER, 0);
		pressed = 0;
	}
	
    input_sync(g_gc9a01_data->gc9a01_dev);
	
}

static int touch_event_handler(void *arg)
{
	struct gc9a01_data *gc_data = arg;
	int ret = 0;
	unsigned short pdwSampleX, pdwSampleY;
	unsigned char tp_temp[10];
	unsigned char finger_num;
	do{
		wait_event_interruptible(gc_data->waiter, gc_data->tpd_flag != 0);
		gc_data->tpd_flag = 0;
		mutex_lock(&gc_data->i2c_access);

/*		
		//pr_err("gezi---------------%s----------%d\n",__func__,__LINE__);
		
		ret = gc9a01_read_block(gc_data->client, FTS_REG_START,tp_temp, 7);
		
		//for(i = 0; i < 7; i++){
		//	
		//}
		if(ret < 0){
			pr_err("gezi-------- err-------%s----------%d\n",__func__,__LINE__);
			goto exit_unlock;
		}
		finger_num = tp_temp[2]; //手指个数
		pdwSampleX = ((tp_temp[3] & 0x0F) << 8) + tp_temp[4];
		pdwSampleY = ((tp_temp[5] & 0x0F) << 8) + tp_temp[6];
		
	//	pr_err("reg[0x03]= %x,reg[0x05] = %x\n",tp_temp[3],tp_temp[5]);

		pr_err("mode = %d,finger_num = %d,x = %x,y = %x\n",tp_temp[1],finger_num,pdwSampleX,pdwSampleY);
		
		gc9a01_report(pdwSampleX,pdwSampleY,tp_temp[1],finger_num);
		
		//if(tp_temp[0] = 0x00) //扱点模式.要求FAE將 手勢碍清零.很多吋候手勢碍没有被清除
*/
		fts_touch_process();

exit_unlock:
		//enable_irq(gc_data->tp_irq);
		mutex_unlock(&gc_data->i2c_access);
		
	} while (!kthread_should_stop());

	return ret;
}

static void gc9a01_suspend(void)
{
	int ret = 0;
	unsigned char enterSleep = 0x03;

	if(unlikely(!g_gc9a01_data)){
		return;
	}
	
	gc9a01_reset(g_gc9a01_data);
    ret = gc9a01_write(g_gc9a01_data->client,FTS_REG_LOW_POWER,&enterSleep);
	if(ret < 0){
		pr_err("gc9a01 enter suspend failed\n");
	}
	gc9a01_irq_disable(g_gc9a01_data);
	pr_err("gc9a01 enter suspend \n");
	
}

static void gc9a01_resume(void)
{
	//int ret = 0;
	//unsigned char QuitSleep = 0x01;
	
	if(unlikely(!g_gc9a01_data)){
		return;
	}
	
	gc9a01_reset(g_gc9a01_data);
/*	
    ret = gc9a01_write(g_gc9a01_data->client,0xfe,&QuitSleep);
	if(ret < 0){
		pr_err("gc9a01 quit suspend failed\n");
	}
*/
	gc9a01_irq_enable(g_gc9a01_data);
	pr_err("gc9a01 enter resume \n");
}


void gc9a01_tp_power_ctrl(int value)
{
	if(value){
		//gc9a01_resume();
		fts_ts_resume();
	}
	else{
		//gc9a01_suspend();
		fts_ts_suspend();
	}
}
EXPORT_SYMBOL(gc9a01_tp_power_ctrl);

#ifdef GC9A01_SYS_TEST
static struct class * gc9a01_class;

static ssize_t gc9a01_test_store(struct class *class, struct class_attribute *attr,	const char *buf, size_t count)
{
	if(buf[0] == '0')
	{
		//gc9a01_suspend();
		fts_ts_suspend();
	}
	else if(buf[0] == '1')
	{
		//gc9a01_resume();
		fts_ts_resume();
	}
	return count;
}

static struct class_attribute gc9a01_class_attrs[] = {
	__ATTR(test, S_IRUGO | S_IWUSR, NULL, gc9a01_test_store),
	__ATTR_NULL,
};


static int gc9a01_sysfs_create(void)
{
	int i = 0,ret = 0;
	
	gc9a01_class = class_create(THIS_MODULE, "gc9a01_tp");
	if (IS_ERR(gc9a01_class))
		return PTR_ERR(gc9a01_class);
	for (i = 0; gc9a01_class_attrs[i].attr.name; i++) {
		ret = class_create_file(gc9a01_class,&gc9a01_class_attrs[i]);
		if (ret < 0)
		{
			pr_err("gc9a01_sysfs_create error !!\n");
			return ret;
		}
	}
	return ret;
	//gc9a01_class->dev_groups = rt5509_cal_groups;
}
#endif

static int gc9a01_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct task_struct *thread;
	struct gc9a01_data * gc_data = NULL;
	int err = 0;
	
	pr_err("gezi-------------%s-----------------%d\n",__func__,__LINE__);
	
	gc_data = devm_kzalloc(&client->dev, sizeof(*gc_data), GFP_KERNEL);
	if (!gc_data){
		pr_err("gezi------ENOMEM-------%s-----------------%d\n",__func__,__LINE__);
		return -ENOMEM;
	}
	
	gc9a01_get_gpio(&client->dev,gc_data);
	init_waitqueue_head(&gc_data->waiter);
	mutex_init(&gc_data->i2c_access);
	spin_lock_init(&gc_data->irq_lock);
	gc_data->client = client;
	//gc9a01_reset(gc_data);
	g_gc9a01_data = gc_data;
	fts_hw_reset(200);
/*
	gc_data->vdd = devm_regulator_get(&client->dev, "vdd");
	if (IS_ERR_OR_NULL(gc_data->vdd))
		dev_err(&client->dev, "get regulator fail %d\n",PTR_ERR(gc_data->vdd));
	else
		regulator_enable(gc_data->vdd);
*/
	msleep(150);
	
	err = fts_check_id();
	if(err < 0){
		pr_err("gezi---gc9a01 get sensor id failed--%s---%d\n",__func__,__LINE__);
		return -1;
	}
	
	gc9a01_input_init(gc_data);
	
	thread = kthread_run(touch_event_handler, gc_data, "gc9a01_thread");
	
	
	if (IS_ERR(thread)) {
		err = PTR_ERR(thread);
		pr_err(" failed to create kernel thread: %d\n",err);
	}
	
	gc9a01_eint_setup(gc_data);
#ifdef GC9A01_SYS_TEST	
	gc9a01_sysfs_create();
#endif	
	ft3311_touch_esd_check =kthread_create(fts_timer_interrupt_handler,NULL, "ft3311_esd_check");

	if ( fts_data->esd_support == 1) {
		printk("esd_support %s----%d-----\n",__func__,__LINE__);
		wake_up_process(ft3311_touch_esd_check);
	}

	return 0;
}

static int gc9a01_remove(struct i2c_client *client) 
{
	return 0;
}



static const struct i2c_device_id gc9a01_dev_id[] = {
    {"gc9a01_touch", 0},
    {},
};
MODULE_DEVICE_TABLE(i2c,gc9a01_dev_id);

static struct i2c_driver gc9a01_driver = {
    .driver   = {
        .name           = DEVICE_NAME,
        .owner          = THIS_MODULE,
        .of_match_table = gc9a01_match_table,
    },
    .probe    = gc9a01_probe,
    .remove   = gc9a01_remove,
    .id_table = gc9a01_dev_id,
};
//module_i2c_driver(gc9a01_driver);
static int __init gc9a01_touch_init(void)
{
	int ret = 0;
	pr_err("gezi--------%s---\n",__func__);
	
	ret = i2c_add_driver(&gc9a01_driver);
	
	return ret;
}
static void __exit gc9a01_touch_exit(void)
{
	i2c_del_driver(&gc9a01_driver);
}
late_initcall_sync(gc9a01_touch_init);
module_exit(gc9a01_touch_exit);

MODULE_AUTHOR("zhaopengge@cooseagroup.com");
MODULE_DESCRIPTION("GC9A01 TP DRIVER");
MODULE_LICENSE("GPL v2");

