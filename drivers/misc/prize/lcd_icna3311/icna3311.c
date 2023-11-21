#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/timer.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/slab.h>
//#include <mt-plat/mtk_pwm.h>
#include <linux/types.h>
#include <linux/spi/spi.h>
#include <linux/workqueue.h>
#include <linux/regulator/consumer.h>
#ifdef CONFIG_PM_SLEEP
#include <linux/pm_wakeup.h>
#endif

#include "icna3311.h"

#define LCD_WIDTH 		228
#define LCD_HEIGHT		460

#define DPS_DEV_NAME  "mediatek,gc9a01"

struct gc9a01_struct {
	struct delayed_work dwork;
	struct spi_device *spi;
	struct regulator *vdd;
	//struct wakeup_source pwm_wakelock;
};

struct gc9a01_struct gc9a01;

static int GC9A01_DEBUG_ENABLE = 1;
#define GC_DEBUG(format, args...) do { \
	if (GC9A01_DEBUG_ENABLE) \
	{\
		printk(KERN_WARNING format, ##args);\
	} \
} while (0)
	

static struct pinctrl *gc9a01_pinctrl;

static struct pinctrl_state *gc9a01_reset_active;
static struct pinctrl_state *gc9a01_reset_suspend;

static struct pinctrl_state *gc9a01_bl_active;
static struct pinctrl_state *gc9a01_bl_suspend;

static struct pinctrl_state *gc9a01_dc_active;
static struct pinctrl_state *gc9a01_dc_suspend;

static struct pinctrl_state *spi1_as_cs;
static struct pinctrl_state *spi1_as_ck;
static struct pinctrl_state *spi1_as_mi;
static struct pinctrl_state *spi1_as_mo;
static struct pinctrl_state *gc9a01_bl_pwm;

static struct pinctrl_state *gc9a01_vci_active;
static struct pinctrl_state *gc9a01_vci_suspend;

static struct class * gc9a01_class;

//static void gc9a01_set_pwm_duty(unsigned char duty);
static int gc9a01_write_cmd(struct spi_device *spi, uint8_t cmd);
static int gc9a01_write_data(struct spi_device *spi, uint8_t data);


static int gc9a01_sync_write(struct spi_device *spi, uint8_t *tx, uint32_t len){
	
	int ret = 0;
	struct spi_message m;
	struct spi_transfer t = {
		.tx_buf = tx,
		.len = len,
		//.speed_hz	= spi->speed_hz,
	};

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	ret = spi_sync(spi, &m);
	if (ret == 0)
		return m.actual_length;
	
	return ret;
}

static int icna3311_spi_write_multi_byte(struct spi_device *spi, u8 *value_data)
{

	struct spi_message msg;
	struct spi_transfer xfer = {
		.len		= 209760,
	};
	xfer.tx_buf = value_data;
	spi_message_init(&msg);
	spi_message_add_tail(&xfer, &msg);

	return spi_sync(spi, &msg);
}

static int gc9a01_sync_read(struct spi_device *spi,uint8_t command, uint8_t *data, uint32_t length){
	
	spi_write_then_read(spi, &command, 1, data, length);
	
	return 0;
}

static ssize_t manual_duty_show(struct class *class, struct class_attribute *attr,	char *buf)
{
	return sprintf(buf, "%d\n", 0);
}

static ssize_t manual_duty_store(struct class *class, struct class_attribute *attr, const char *buf, size_t count)
{
	int count_t = (int)count;
	
	int duty = 0;
	
	if(count_t == 2)// 0 < duty < 10
	{
		if(buf[0] != CHAR_LF)
		{
			duty = buf[0] - 0x30;
		}
	}
	else if(count_t == 3)// 10 <= duty < 100
	{
		if((buf[0] != CHAR_LF) && (buf[1] != CHAR_LF))
		{
			duty = (buf[0] - 0x30) * 10 + (buf[1] - 0x30);
		}
	}
	else if(count_t == 4)
	{
		duty = 100;
	}
	
	if(duty < 0)
	{
		duty = 0;
	}
	if(duty > 100)
	{
		duty = 100;
	}
	
	//fan_w->pwm_duty = duty;
	
	//gc9a01_set_pwm_duty(duty);
	
	pr_err("manual_duty_store = 0x%x,0x%x,count = %d\n",buf[0],buf[1],count_t);

	return count;
}

static struct class_attribute gc9a01_class_attrs[] = {
	 __ATTR(duty, S_IRUGO | S_IWUSR, manual_duty_show, manual_duty_store),
	 __ATTR_NULL
 };


static int gc9a01_sysfs_create(void)
{
	int i = 0,ret = 0;
	
	gc9a01_class = class_create(THIS_MODULE, "gc9a01_lcd");
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
	
static int gc9a01_pinctrl_init(struct device *dev)
{
	int ret = 0;

	printk("second lcd %s line = %d\n", __func__, __LINE__);

	/* get pinctrl */
	gc9a01_pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR(gc9a01_pinctrl)) {
		pr_err("Failed to get flashlight pinctrl ret = %d\n",IS_ERR(gc9a01_pinctrl));
		ret = PTR_ERR(gc9a01_pinctrl);
		return ret;
	}
	
	gc9a01_reset_active = pinctrl_lookup_state(gc9a01_pinctrl, "reset_active");
	if (IS_ERR(gc9a01_reset_active)) {
		pr_err("Failed to init gc9a01_reset_active\n");
		ret = PTR_ERR(gc9a01_reset_active);
	}

	gc9a01_reset_suspend = pinctrl_lookup_state(gc9a01_pinctrl, "reset_suspend");
	if (IS_ERR(gc9a01_reset_suspend)) {
		pr_err("Failed to init gc9a01_reset_suspend\n");
		ret = PTR_ERR(gc9a01_reset_suspend);
	}
	
	gc9a01_bl_active = pinctrl_lookup_state(gc9a01_pinctrl, "bl_active");
	if (IS_ERR(gc9a01_bl_active)) {
		pr_err("Failed to init gc9a01_bl_active\n");
		ret = PTR_ERR(gc9a01_bl_active);
	}
	gc9a01_bl_suspend = pinctrl_lookup_state(gc9a01_pinctrl, "bl_suspend");
	if (IS_ERR(gc9a01_bl_suspend)) {
		pr_err("Failed to init gc9a01_bl_suspend\n");
		ret = PTR_ERR(gc9a01_bl_suspend);
	}
	
	gc9a01_dc_active = pinctrl_lookup_state(gc9a01_pinctrl, "dc_active");
	if (IS_ERR(gc9a01_dc_active)) {
		pr_err("Failed to init gc9a01_dc_active\n");
		ret = PTR_ERR(gc9a01_dc_active);
	}
	gc9a01_dc_suspend = pinctrl_lookup_state(gc9a01_pinctrl, "dc_suspend");
	if (IS_ERR(gc9a01_dc_suspend)) {
		pr_err("Failed to init gc9a01_dc_suspend\n");
		ret = PTR_ERR(gc9a01_dc_suspend);
	}
	
	spi1_as_cs = pinctrl_lookup_state(gc9a01_pinctrl, "spi1_as_cs_t");
	if (IS_ERR(spi1_as_cs)) {
		pr_err("Failed to init spi1_as_cs\n");
		ret = PTR_ERR(spi1_as_cs);
	}
	
	spi1_as_ck = pinctrl_lookup_state(gc9a01_pinctrl, "spi1_as_ck_t");
	if (IS_ERR(spi1_as_ck)) {
		pr_err("Failed to init spi1_as_ck\n");
		ret = PTR_ERR(spi1_as_ck);
	}
	
	spi1_as_mi = pinctrl_lookup_state(gc9a01_pinctrl, "spi1_as_mi_t");
	if (IS_ERR(spi1_as_mi)) {
		pr_err("Failed to init spi1_as_mi\n");
		ret = PTR_ERR(spi1_as_mi);
	}
	
	spi1_as_mo = pinctrl_lookup_state(gc9a01_pinctrl, "spi1_as_mo_t");
	if (IS_ERR(spi1_as_mo)) {
		pr_err("Failed to init spi1_as_mo\n");
		ret = PTR_ERR(spi1_as_mo);
	}

	//gc9a01_bl_pwm = pinctrl_lookup_state(gc9a01_pinctrl, "bl_pwm");
	//if (IS_ERR(gc9a01_bl_pwm)) {
		//pr_err("Failed to init gc9a01_bl_pwm\n");
		//ret = PTR_ERR(gc9a01_bl_pwm);
	//}

	gc9a01_vci_active = pinctrl_lookup_state(gc9a01_pinctrl, "vci_active");
	if (IS_ERR(gc9a01_vci_active)) {
		pr_err("Failed to init gc9a01_vci_active\n");
		ret = PTR_ERR(gc9a01_vci_active);
	}

	gc9a01_vci_suspend = pinctrl_lookup_state(gc9a01_pinctrl, "vci_suspend");
	if (IS_ERR(gc9a01_vci_suspend)) {
		pr_err("Failed to init gc9a01_vci_suspend\n");
		ret = PTR_ERR(gc9a01_vci_suspend);
	}

	pinctrl_select_state(gc9a01_pinctrl, spi1_as_cs);
	pinctrl_select_state(gc9a01_pinctrl, spi1_as_ck);
	pinctrl_select_state(gc9a01_pinctrl, spi1_as_mi);
	pinctrl_select_state(gc9a01_pinctrl, spi1_as_mo);
	//pinctrl_select_state(gc9a01_pinctrl, gc9a01_bl_pwm);
	
	return ret;
}

void gc9a01_bl_ctl(int level)
{
	char data = 0xFF;
	GC_DEBUG("%s----%d-----\n",__func__,__LINE__);
/*	if(level < 0){
		level = 0;
	}
	else if(level > 100){
		level = 100;
	}

	if(level)
	{
		pinctrl_select_state(gc9a01_pinctrl, gc9a01_bl_active);
		//pinctrl_select_state(gc9a01_pinctrl, gc9a01_bl_pwm);
	}
	else
	{
		pinctrl_select_state(gc9a01_pinctrl, gc9a01_bl_suspend);
	}
	
	gc9a01_set_pwm_duty(level);
*/

	level = level * 255 / 100;
	data = level & 0xFF;

	GC_DEBUG("%s----backlight = %d-----\n",__func__,data);

	gc9a01_write_cmd(gc9a01.spi,0x51);
	gc9a01_write_data(gc9a01.spi,data);

}
EXPORT_SYMBOL(gc9a01_bl_ctl);

void gc9a01_reset_ctl(int on)
{
	GC_DEBUG("%s----%d-----\n",__func__,__LINE__);
	
	if(on)
	{
		pinctrl_select_state(gc9a01_pinctrl, gc9a01_reset_active);
	}
	else
	{
		pinctrl_select_state(gc9a01_pinctrl, gc9a01_reset_suspend);
	}
}
EXPORT_SYMBOL(gc9a01_reset_ctl);

void gc9a01_dc_ctl(int on)
{
	//GC_DEBUG("%s----%d-----\n",__func__,__LINE__);
	
	if(on)
	{
		pinctrl_select_state(gc9a01_pinctrl, gc9a01_dc_active);
	}
	else
	{
		pinctrl_select_state(gc9a01_pinctrl, gc9a01_dc_suspend);
	}
}
EXPORT_SYMBOL(gc9a01_dc_ctl);

/*
static void gc9a01_set_pwm_duty(unsigned char duty)
{
    struct pwm_spec_config pwm_setting;

	if(duty <= 0)
	{
		mt_pwm_disable(PWM1, 0);
		__pm_relax(&gc9a01.pwm_wakelock);
		return;
	}
	if (duty > 100){
		duty = 100;
	}
	
	__pm_stay_awake(&gc9a01.pwm_wakelock);
	memset(&pwm_setting, 0, sizeof(struct pwm_spec_config));
    pwm_setting.pwm_no = PWM2;
    pwm_setting.mode = PWM_MODE_OLD;
    pwm_setting.clk_src = PWM_CLK_OLD_MODE_BLOCK;  // PWM_CLK_OLD_MODE_32K PWM_CLK_OLD_MODE_BLOCK
    pwm_setting.clk_div = CLK_DIV32;//CLK_DIV1 = 1
    pwm_setting.PWM_MODE_OLD_REGS.THRESH = duty - 1;
	pwm_setting.PWM_MODE_OLD_REGS.DATA_WIDTH = 100-1;
	pwm_setting.PWM_MODE_OLD_REGS.IDLE_VALUE = 0;
	pwm_setting.PWM_MODE_OLD_REGS.GUARD_VALUE = 0;
	pwm_setting.PWM_MODE_OLD_REGS.GDURATION = 0;
	pwm_setting.PWM_MODE_OLD_REGS.WAVE_NUM = 0;
    pwm_set_spec_config(&pwm_setting);
}
*/

static int gc9a01_write_cmd(struct spi_device *spi, uint8_t cmd){
	uint8_t tx_buf[2] = {0};
	int ret = 0;
	
	gc9a01_dc_ctl(0);
	tx_buf[0] = cmd;
	ret = gc9a01_sync_write(spi, tx_buf, 1);
	gc9a01_dc_ctl(1);
	
	return ret;
}

static int gc9a01_write_data(struct spi_device *spi, uint8_t data){
	uint8_t tx_buf[2] = {0};
	int ret = 0;
	
	//gc9a01_dc_ctl(1);
	tx_buf[0] = data;
	ret = gc9a01_sync_write(spi, tx_buf, 1);
	//gc9a01_dc_ctl(0);
	return ret;
}

static void lcd_gc9a01_set_window(struct spi_device *spi, 
			uint16_t xStart, uint16_t yStart, uint16_t xEnd, uint16_t yEnd,
			uint16_t xOffset, uint16_t yOffset){
	yStart = yStart + yOffset;
	yEnd = yEnd + yOffset;
	xStart = xStart + xOffset;
	xEnd = xEnd + xOffset;

	gc9a01_write_cmd(spi, 0x2A);
	gc9a01_write_data(spi, xStart>>8);
	gc9a01_write_data(spi, xStart&0xff);
	gc9a01_write_data(spi, xEnd>>8);
	gc9a01_write_data(spi, xEnd&0xff);
	
	gc9a01_write_cmd(spi, 0x2B);
	gc9a01_write_data(spi, yStart>>8);
	gc9a01_write_data(spi, yStart&0xff);
	gc9a01_write_data(spi, yEnd>>8);
	gc9a01_write_data(spi, yEnd&0xff);

	gc9a01_write_cmd(spi, 0x2c);
}

int gc9a01_scr_on(struct spi_device *spi){
	
	schedule_delayed_work(&gc9a01.dwork, 2*HZ);
	
	return 0;
}
	
static int gc9a01_init(struct spi_device *spi){
	
	spi->mode = SPI_MODE_3;
	spi->bits_per_word = 8;
	spi->max_speed_hz = 18000000;
	spi_setup(spi);
	pinctrl_select_state(gc9a01_pinctrl, gc9a01_vci_active);
	msleep(15);
	gc9a01_reset_ctl(1);
	msleep(120);
	gc9a01_reset_ctl(0);
	msleep(100);
	gc9a01_reset_ctl(1);
	msleep(120);
	
	printk("%s----%d-----\n",__func__,__LINE__);	

/*
// ===  CMD2 password  ===
	gc9a01_write_cmd(spi,0xFE);
	gc9a01_write_data(spi,0x20);
	gc9a01_write_cmd(spi,0xF4);
	gc9a01_write_data(spi,0x5A);
	gc9a01_write_cmd(spi,0xF5);
	gc9a01_write_data(spi,0x59);


	gc9a01_write_cmd(spi,0xFE);
	gc9a01_write_data(spi,0x20);
	gc9a01_write_cmd(spi,0x19);//QSPI setting, MIPI remove
	gc9a01_write_data(spi,0x10);
	gc9a01_write_cmd(spi,0x1C);//QSPI setting, MIPI remove
	gc9a01_write_data(spi,0xA0);

*/
//=== bits mode
/*
	gc9a01_write_cmd(spi,0xFE);
 	gc9a01_write_data(spi,0x20);
	gc9a01_write_cmd(spi,0xF4);
 	gc9a01_write_data(spi,0x5A);
	gc9a01_write_cmd(spi,0xF5);
 	gc9a01_write_data(spi,0x59);
	gc9a01_write_cmd(spi,0xFE);
 	gc9a01_write_data(spi,0xD0);
	gc9a01_write_cmd(spi,0x4D);
 	gc9a01_write_data(spi,0x7F);
	gc9a01_write_cmd(spi,0x4E);
 	gc9a01_write_data(spi,0x80);
	gc9a01_write_cmd(spi,0xFE);
 	gc9a01_write_data(spi,0x40);
	gc9a01_write_cmd(spi,0x54);
 	gc9a01_write_data(spi,0xAF);
*/

//=== CMD1 setting ===
	gc9a01_write_cmd(spi,0xFE);
 	gc9a01_write_data(spi,0x00);
	gc9a01_write_cmd(spi,0xC4);
 	gc9a01_write_data(spi,0x80); //SPI setting, mipi remove
	gc9a01_write_cmd(spi,0x3A);
 	gc9a01_write_data(spi,0x55); //55 RGB565, 77 RGB888
	gc9a01_write_cmd(spi,0x35);
 	gc9a01_write_data(spi,0x00);
	gc9a01_write_cmd(spi,0x53);
 	gc9a01_write_data(spi,0x20);
	gc9a01_write_cmd(spi,0x51);
 	gc9a01_write_data(spi,0xFF);
	gc9a01_write_cmd(spi,0x63);
 	gc9a01_write_data(spi,0xFF);
	gc9a01_write_cmd(spi,0x2A);
 	gc9a01_write_data(spi,0x00);
 	gc9a01_write_data(spi,0x24);
 	gc9a01_write_data(spi,0x01);
 	gc9a01_write_data(spi,0x07);
	gc9a01_write_cmd(spi,0x2B);
 	gc9a01_write_data(spi,0x00);
 	gc9a01_write_data(spi,0x00);
 	gc9a01_write_data(spi,0x01);
 	gc9a01_write_data(spi,0xCB);
	gc9a01_write_cmd(spi,0x11);
	msleep(60);
	gc9a01_write_cmd(spi,0x29);

	lcd_gc9a01_set_window(spi, 0, 0, LCD_WIDTH - 1, LCD_HEIGHT - 1, 36, 0);
	return 0;
}

static void gc9a01_fill(struct spi_device *spi){
	int i = 0;
	int j = 0;
	uint8_t *tx_buf;
	int count = 209760;//(RGB565)
	//int count = 314640;//(RGB888)

	tx_buf = (uint8_t *)kzalloc((count), GFP_KERNEL | GFP_DMA);
	memset(tx_buf,0xff,count);
	gc9a01_sync_write(spi, tx_buf, count);


	kfree(tx_buf);
/*
	uint16_t r[LCD_WIDTH] = {0};
	uint16_t g[LCD_WIDTH] = {0};
	uint16_t b[LCD_WIDTH] = {0};
	//int ret = 0;
	int height = LCD_HEIGHT;
	int strip_w = 30;
	int i = 0;
	
	for(i=0;i<LCD_WIDTH;i++)
		*(r+i) = 0x00f8;
	for(i=0;i<LCD_WIDTH;i++)
		*(g+i) = 0xe007;
	for(i=0;i<LCD_WIDTH;i++)
		*(b+i) = 0x1f00;

#if 0
	printk("%s %x %x %x %x %d\n",__func__, *((uint8_t *)r),*((uint8_t *)r+1), *((uint8_t *)r+2),*((uint8_t *)r+3), sizeof(r));
	for(i=0;i<LCD_HEIGHT;i++){
		ret = gc9a01_sync_write(spi, (uint8_t *)r, sizeof(r));
		printk("%s %d %d\n",__func__,i, ret);
	}
#endif
	do {
		if (height < strip_w){
			strip_w = height;
			if (height <= 0)
				break;
		}
		for(i = 0; i < strip_w; i++){
			gc9a01_sync_write(spi, (uint8_t *)r, sizeof(r));
			height -= 1;
		}
		if (height < strip_w){
			strip_w = height;
			if (height <= 0)
				break;
		}
		for(i = 0; i < strip_w; i++){
			gc9a01_sync_write(spi, (uint8_t *)g, sizeof(r));
			height -= 1;
		}
		if (height < strip_w){
			strip_w = height;
			if (height <= 0)
				break;
		}
		for(i = 0; i < strip_w; i++){
			gc9a01_sync_write(spi, (uint8_t *)b, sizeof(r));
			height -= 1;
		}
	}while(height > 0);
*/
}

static void gc9a01_dwork(struct work_struct *work){

	printk("%s----%d-----\n",__func__,__LINE__);
	gc9a01_init(gc9a01.spi);
	//gc9a01_fill(gc9a01.spi);
	gc9a01_bl_ctl(85);
}

int gc9a01_probe(struct spi_device *spi)
{
	int err = 0;

	GC_DEBUG("%s----%d-----\n",__func__,__LINE__);
	printk("second lcd %s line = %d\n", __func__, __LINE__);
	
	/* init pinctrl */
	if (gc9a01_pinctrl_init(&spi->dev)) {
		printk("Failed to init pinctrl.\n");
		err = -EFAULT;
		goto error;
	}

	//gc9a01_sysfs_create();
	//class_register(&gc9a01_class);
	
	//gpio_probe(pdev);
	
	//gc9a01_bl_ctl(0);

	//gc9a01_reset_ctl(0);

	//gc9a01_power_on();
	//if(0){
	//	gc9a01_set_pwm_duty(95);
	//}
	
	gc9a01_sysfs_create();

	GC_DEBUG("%s----%d-----\n",__func__,__LINE__);
	
	gc9a01.spi = spi;
	spi->mode = SPI_MODE_3;
	INIT_DELAYED_WORK(&gc9a01.dwork, gc9a01_dwork);
	schedule_delayed_work(&gc9a01.dwork, 2*HZ);
	//wakeup_source_init(&gc9a01.pwm_wakelock, "miniscr wakelock");
/*
	gc9a01.vdd = devm_regulator_get(&spi->dev, "vdd");
	if (IS_ERR(gc9a01.vdd)){
		dev_err(&spi->dev, "get vdd fail %d\n", PTR_ERR(gc9a01.vdd));
		goto error;
	}
	err = regulator_enable(gc9a01.vdd);
	if (err)
		dev_err(&spi->dev, "enable vdd fail %d\n",err);
*/	


	return 0;
error:
	return err;
}
