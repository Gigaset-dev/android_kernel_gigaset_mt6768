/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

/*****************************************************************************
 *
 * Filename:
 * ---------
 *   ialengmipiraw_Sensor.c
 *
 * Project:
 * --------
 *     ALPS
 *
 * Description:
 * ------------
 *     Source code of Sensor driver
 *
 *
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/

#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/atomic.h>
#include <linux/types.h>
#include <mt-plat/mtk_boot.h>
#include "kd_camera_typedef.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"
#include "kd_camera_feature.h"

#include "cw9281_mipi_raw.h"


#define MIPI_SETTLE         30


#ifdef CONFIG_COMPAT

#include <linux/compat.h>
#endif

#define SENSORDB LOG_INF
/****************************Modify Following Strings for Debug****************************/
#define PFX "cw9281_camera_sensor"
#define LOG_INF(format, args...)    pr_err(PFX "[%s] " format, __func__, ##args)
/****************************   Modify end    *******************************************/


static DEFINE_SPINLOCK(imgsensor_drv_lock);




static struct imgsensor_info_struct imgsensor_info = {
    .sensor_id = 0x9282,        //record sensor id defined in Kd_imgsensor.h   CW9281_SENSOR_ID

	.checksum_value = 0xf7375923,

	.pre = {
		.pclk = 75000000,
		.linelength = 1488,
		.framelength = 827,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 1280,
		.grabwindow_height = 800,
		.mipi_data_lp2hs_settle_dc = MIPI_SETTLE,
		.max_framerate = 600,
	},
	.cap = {
		.pclk = 12000000,
		.linelength = 1488,
		.framelength = 827,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 1280,
		.grabwindow_height = 800,
		.mipi_data_lp2hs_settle_dc = MIPI_SETTLE,
		.max_framerate = 600,
	},
	.cap1 = {
		.pclk = 12000000,
		.linelength = 1488,
		.framelength = 827,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 1280,
		.grabwindow_height = 800,
		.mipi_data_lp2hs_settle_dc = MIPI_SETTLE,
		.max_framerate = 600,             /*less than 13M(include 13M)*/
	},
	.normal_video = {
		.pclk = 12000000,
		.linelength = 1488,
		.framelength = 827,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 1280,
		.grabwindow_height = 800,
		.mipi_data_lp2hs_settle_dc = MIPI_SETTLE,
		.max_framerate = 600,
	},
	.hs_video = {
		.pclk = 12000000,
		.linelength = 1488,
		.framelength = 827,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 1280,
		.grabwindow_height = 800,
		.mipi_data_lp2hs_settle_dc = MIPI_SETTLE,
		.max_framerate = 600,
	},
	.slim_video = {
		.pclk = 12000000,
		.linelength = 1488,
		.framelength = 827,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 1280,
		.grabwindow_height = 800,
		.mipi_data_lp2hs_settle_dc = MIPI_SETTLE,
		.max_framerate = 600,
	},
	.margin = 4,
	.min_shutter = 1,
	.max_frame_length = 0x7fff,
	.ae_shut_delay_frame = 0,
	.ae_sensor_gain_delay_frame = 0,
	.ae_ispGain_delay_frame = 2,
	.ihdr_support = 0,
	.ihdr_le_firstline = 0,
	.sensor_mode_num = 5,

	.cap_delay_frame = 0,
	.pre_delay_frame = 0,
	.video_delay_frame = 0,
	.hs_video_delay_frame = 0,
	.slim_video_delay_frame = 0,

	.isp_driving_current = ISP_DRIVING_6MA,	/* mclk driving current */
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,	/* sensor_interface_type */
	.mipi_sensor_type = MIPI_OPHY_NCSI2,	/* 0,MIPI_OPHY_NCSI2;  1,MIPI_OPHY_CSI2 */
	.mipi_settle_delay_mode = MIPI_SETTLEDELAY_AUTO,	/* 0,MIPI_SETTLEDELAY_AUTO;
								 * 1,MIPI_SETTLEDELAY_MANNUAL
								 */
	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW8_MONO, //SENSOR_OUTPUT_FORMAT_RAW_B,	/* sensor output first pixel color */
	.mclk = 24,		/* mclk value, suggest 24 or 26 for 24Mhz or 26Mhz */
	.mipi_lane_num = SENSOR_MIPI_1_LANE,	/* mipi lane num */
    .i2c_addr_table = {0xc0, 0x20, 0xff},//record sensor support all write id addr, only supprt 4must end with 0xff
						/* only supprt 4must end with 0xff
						 */
	.i2c_speed = 300,	/* i2c read/write speed */
};


static struct imgsensor_struct imgsensor = {
	.mirror = IMAGE_NORMAL,
	.sensor_mode = IMGSENSOR_MODE_INIT,
	.shutter = 0x3D0,
	.gain = 0x100,
	.dummy_pixel = 0,
	.dummy_line = 0,
	.current_fps = 600,
	.autoflicker_en = KAL_FALSE,
	.test_pattern = KAL_FALSE,
	.current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW,
	.ihdr_en = 0,
	.i2c_write_id = 0xc0,
};


/* Sensor output window information*/
static struct SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[5] = {
	{ 1280, 800,	 0,    0, 1280, 800, 1280,  800, 0000, 0000, 1280,  800, 	 0,    0, 1280,  800}, // Preview 
	{ 1280, 800,	 0,    0, 1280, 800, 1280,  800, 0000, 0000, 1280,  800, 	 0,    0, 1280,  800}, // capture 
	{ 1280, 800,	 0,    0, 1280, 800, 1280,  800, 0000, 0000, 1280,  800, 	 0,    0, 1280,  800}, // video 
	{ 1280, 800,	 0,    0, 1280, 800, 1280,  800, 0000, 0000, 1280,  800, 	 0,    0, 1280,  800}, //hight speed video 
	{ 1280, 800,	 0,    0, 1280, 800, 1280,  800, 0000, 0000, 1280,  800, 	 0,    0, 1280,  800}// slim video 
};



#define MULTI_WRITE 1

#if MULTI_WRITE
#define I2C_BUFFER_LEN 225
#else
#define I2C_BUFFER_LEN 3

#endif

extern void i2c_write_reg16_data8(kal_uint32 slave_addr, kal_uint32 reg, kal_uint32 data);
extern kal_uint16 i2c_read_reg16_data8(kal_uint32 slave_addr, kal_uint32 reg);
extern void i2c_write_reg8_data8(kal_uint16 slave_addr, kal_uint16 reg, kal_uint32 data);



static void cw9281_sensor_poweron(void)
{
	i2c_write_reg8_data8(GPIO_EXPANDER_WRITE_ID, 0x01, 0xf8);
	i2c_write_reg8_data8(GPIO_EXPANDER_WRITE_ID, 0x03, 0xf1);	
	msleep(10);
	i2c_write_reg8_data8(GPIO_EXPANDER_WRITE_ID, 0x01, 0xfc);
	msleep(20);
}

static void cw9281_sensor_powerdown(void)
{
	i2c_write_reg8_data8(GPIO_EXPANDER_WRITE_ID, 0x03, 0xff);
	i2c_write_reg8_data8(GPIO_EXPANDER_WRITE_ID, 0x01, 0xf0);
}

static void sensor_init(void)
{
	LOG_INF("cw9281 sensor_init E\n");
	//0x0103, 0x01,
	i2c_write_reg16_data8(imgsensor.i2c_write_id, 0x0103,0x01);// ; software reset
	//mdelay(30);
//	i2c_write_reg16_data8(imgsensor.i2c_write_id, 0x0100, 0x00);//
	i2c_write_reg16_data8(imgsensor.i2c_write_id, 0x0302, 0x32);//
	i2c_write_reg16_data8(imgsensor.i2c_write_id, 0x030d, 0x50);//
	i2c_write_reg16_data8(imgsensor.i2c_write_id, 0x030e, 0x02);//
	i2c_write_reg16_data8(imgsensor.i2c_write_id, 0x3001, 0x00);//
	i2c_write_reg16_data8(imgsensor.i2c_write_id, 0x3004, 0x00);//
	i2c_write_reg16_data8(imgsensor.i2c_write_id, 0x3005, 0x00);//
	i2c_write_reg16_data8(imgsensor.i2c_write_id, 0x3006, 0x00);//
	i2c_write_reg16_data8(imgsensor.i2c_write_id, 0x3011, 0x0a);//
	i2c_write_reg16_data8(imgsensor.i2c_write_id, 0x3013, 0x18);//
	i2c_write_reg16_data8(imgsensor.i2c_write_id, 0x3022, 0x51);//
	i2c_write_reg16_data8(imgsensor.i2c_write_id, 0x3030, 0x0c);//
	i2c_write_reg16_data8(imgsensor.i2c_write_id, 0x3039, 0x12);//
	i2c_write_reg16_data8(imgsensor.i2c_write_id, 0x303a, 0x00);//
	i2c_write_reg16_data8(imgsensor.i2c_write_id, 0x3500, 0x00);//
	i2c_write_reg16_data8(imgsensor.i2c_write_id, 0x3501, 0x09);//
	i2c_write_reg16_data8(imgsensor.i2c_write_id, 0x3502, 0x00);//
	i2c_write_reg16_data8(imgsensor.i2c_write_id, 0x3503, 0x08);//
	i2c_write_reg16_data8(imgsensor.i2c_write_id, 0x3505, 0x8c);//
	i2c_write_reg16_data8(imgsensor.i2c_write_id, 0x3507, 0x03);//
	i2c_write_reg16_data8(imgsensor.i2c_write_id, 0x3508, 0x00);//
	i2c_write_reg16_data8(imgsensor.i2c_write_id, 0x3509, 0xa0);//
	i2c_write_reg16_data8(imgsensor.i2c_write_id, 0x3610, 0x80);//
	i2c_write_reg16_data8(imgsensor.i2c_write_id, 0x3611, 0xa0);//
	i2c_write_reg16_data8(imgsensor.i2c_write_id, 0x3620, 0x6f);//
	i2c_write_reg16_data8(imgsensor.i2c_write_id, 0x3632, 0x56);//
	i2c_write_reg16_data8(imgsensor.i2c_write_id, 0x3633, 0x78);//
	i2c_write_reg16_data8(imgsensor.i2c_write_id, 0x3662, 0x03);//
	i2c_write_reg16_data8(imgsensor.i2c_write_id, 0x3666, 0x00);//
	i2c_write_reg16_data8(imgsensor.i2c_write_id, 0x366f, 0x5a);//
	i2c_write_reg16_data8(imgsensor.i2c_write_id, 0x3680, 0x84);//
	i2c_write_reg16_data8(imgsensor.i2c_write_id, 0x3712, 0x80);//,
	i2c_write_reg16_data8(imgsensor.i2c_write_id, 0x372d, 0x22);//,
	i2c_write_reg16_data8(imgsensor.i2c_write_id, 0x3731, 0x80);//,
	i2c_write_reg16_data8(imgsensor.i2c_write_id, 0x3732, 0x30);//,
	i2c_write_reg16_data8(imgsensor.i2c_write_id, 0x3778, 0x00);//,
	i2c_write_reg16_data8(imgsensor.i2c_write_id, 0x377d, 0x22);//,
	i2c_write_reg16_data8(imgsensor.i2c_write_id, 0x3788, 0x02);//,
	i2c_write_reg16_data8(imgsensor.i2c_write_id, 0x3789, 0xa4);//,
	i2c_write_reg16_data8(imgsensor.i2c_write_id, 0x378a, 0x00);//,
	i2c_write_reg16_data8(imgsensor.i2c_write_id, 0x378b, 0x4a);//,
	i2c_write_reg16_data8(imgsensor.i2c_write_id, 0x3799, 0x20);//,
	i2c_write_reg16_data8(imgsensor.i2c_write_id, 0x3800, 0x00);//,
	i2c_write_reg16_data8(imgsensor.i2c_write_id, 0x3801, 0x00);//,
	i2c_write_reg16_data8(imgsensor.i2c_write_id, 0x3802, 0x00);//,
	i2c_write_reg16_data8(imgsensor.i2c_write_id, 0x3803, 0x00);//,
	i2c_write_reg16_data8(imgsensor.i2c_write_id, 0x3804, 0x05);//,
	i2c_write_reg16_data8(imgsensor.i2c_write_id, 0x3805, 0x0f);//,
	i2c_write_reg16_data8(imgsensor.i2c_write_id, 0x3806, 0x03);//,
	i2c_write_reg16_data8(imgsensor.i2c_write_id, 0x3807, 0x2f);//,
	i2c_write_reg16_data8(imgsensor.i2c_write_id, 0x3808, 0x05);//,
	i2c_write_reg16_data8(imgsensor.i2c_write_id, 0x3809, 0x00);//,
	i2c_write_reg16_data8(imgsensor.i2c_write_id, 0x380a, 0x03);//,
	i2c_write_reg16_data8(imgsensor.i2c_write_id, 0x380b, 0x20);//,
	i2c_write_reg16_data8(imgsensor.i2c_write_id, 0x380c, 0x05);//,
	i2c_write_reg16_data8(imgsensor.i2c_write_id, 0x380d, 0xb0);//,
	i2c_write_reg16_data8(imgsensor.i2c_write_id, 0x380e, 0x03);//,
	i2c_write_reg16_data8(imgsensor.i2c_write_id, 0x380f, 0x8e);//,
	i2c_write_reg16_data8(imgsensor.i2c_write_id, 0x3810, 0x00);//,
	i2c_write_reg16_data8(imgsensor.i2c_write_id, 0x3811, 0x08);//,
	i2c_write_reg16_data8(imgsensor.i2c_write_id, 0x3812, 0x00);//,
	i2c_write_reg16_data8(imgsensor.i2c_write_id, 0x3813, 0x08);//,
	i2c_write_reg16_data8(imgsensor.i2c_write_id, 0x3814, 0x11);//,
	i2c_write_reg16_data8(imgsensor.i2c_write_id, 0x3815, 0x11);//,
	i2c_write_reg16_data8(imgsensor.i2c_write_id, 0x3820, 0x40);//,
	i2c_write_reg16_data8(imgsensor.i2c_write_id, 0x3821, 0x00);//
	i2c_write_reg16_data8(imgsensor.i2c_write_id, 0x3881, 0x42);//,
	i2c_write_reg16_data8(imgsensor.i2c_write_id, 0x38b1, 0x00);//,
	i2c_write_reg16_data8(imgsensor.i2c_write_id, 0x3920, 0xff);//,
	i2c_write_reg16_data8(imgsensor.i2c_write_id, 0x3921, 0x00);//,
	i2c_write_reg16_data8(imgsensor.i2c_write_id, 0x3922, 0x00);//,
	i2c_write_reg16_data8(imgsensor.i2c_write_id, 0x3923, 0x00);//,
	i2c_write_reg16_data8(imgsensor.i2c_write_id, 0x3924, 0x00);//,
	i2c_write_reg16_data8(imgsensor.i2c_write_id, 0x3925, 0x00);//,
	i2c_write_reg16_data8(imgsensor.i2c_write_id, 0x3926, 0x00);//,
	i2c_write_reg16_data8(imgsensor.i2c_write_id, 0x3927, 0x00);//,
	i2c_write_reg16_data8(imgsensor.i2c_write_id, 0x3928, 0x2c);//,0x39
	i2c_write_reg16_data8(imgsensor.i2c_write_id, 0x4003, 0x10);//,
	i2c_write_reg16_data8(imgsensor.i2c_write_id, 0x4008, 0x04);//,
	i2c_write_reg16_data8(imgsensor.i2c_write_id, 0x4009, 0x0b);//,
	i2c_write_reg16_data8(imgsensor.i2c_write_id, 0x400c, 0x00);//,
	i2c_write_reg16_data8(imgsensor.i2c_write_id, 0x400d, 0x07);//,
	i2c_write_reg16_data8(imgsensor.i2c_write_id, 0x4010, 0x40);//,
	i2c_write_reg16_data8(imgsensor.i2c_write_id, 0x4043, 0x40);//,
	i2c_write_reg16_data8(imgsensor.i2c_write_id, 0x4307, 0x30);//,
	i2c_write_reg16_data8(imgsensor.i2c_write_id, 0x4317, 0x00);//,
	i2c_write_reg16_data8(imgsensor.i2c_write_id, 0x4501, 0x00);//,
	i2c_write_reg16_data8(imgsensor.i2c_write_id, 0x4507, 0x00);//,
	i2c_write_reg16_data8(imgsensor.i2c_write_id, 0x4509, 0x00);//,
	i2c_write_reg16_data8(imgsensor.i2c_write_id, 0x450a, 0x08);//,
	i2c_write_reg16_data8(imgsensor.i2c_write_id, 0x4601, 0x04);//,
	i2c_write_reg16_data8(imgsensor.i2c_write_id, 0x470f, 0x00);//,
	i2c_write_reg16_data8(imgsensor.i2c_write_id, 0x4f07, 0x00);//,
	i2c_write_reg16_data8(imgsensor.i2c_write_id, 0x4800, 0x00);//,
	i2c_write_reg16_data8(imgsensor.i2c_write_id, 0x5000, 0x9f);//,
	i2c_write_reg16_data8(imgsensor.i2c_write_id, 0x5001, 0x00);//,
	i2c_write_reg16_data8(imgsensor.i2c_write_id, 0x5e00, 0x00);//,
	i2c_write_reg16_data8(imgsensor.i2c_write_id, 0x5d00, 0x07);//,
	i2c_write_reg16_data8(imgsensor.i2c_write_id, 0x5d01, 0x00);//
	
	//strobe
	i2c_write_reg16_data8(imgsensor.i2c_write_id, 0x3006, 0x08);
	i2c_write_reg16_data8(imgsensor.i2c_write_id, 0x3210, 0x10);
	i2c_write_reg16_data8(imgsensor.i2c_write_id, 0x3007, 0x02);
	i2c_write_reg16_data8(imgsensor.i2c_write_id, 0x301c, 0x22);
	i2c_write_reg16_data8(imgsensor.i2c_write_id, 0x3020, 0x20);
	i2c_write_reg16_data8(imgsensor.i2c_write_id, 0x3025, 0x02);
	i2c_write_reg16_data8(imgsensor.i2c_write_id, 0x382c, 0x0b);
	i2c_write_reg16_data8(imgsensor.i2c_write_id, 0x382d, 0x60);
	i2c_write_reg16_data8(imgsensor.i2c_write_id, 0x3920, 0xff);
	i2c_write_reg16_data8(imgsensor.i2c_write_id, 0x3923, 0x00);
	i2c_write_reg16_data8(imgsensor.i2c_write_id, 0x3924, 0x00);
	i2c_write_reg16_data8(imgsensor.i2c_write_id, 0x3925, 0x00);
	i2c_write_reg16_data8(imgsensor.i2c_write_id, 0x3926, 0x00);
	i2c_write_reg16_data8(imgsensor.i2c_write_id, 0x3927, 0x00);
	i2c_write_reg16_data8(imgsensor.i2c_write_id, 0x3928, 0x90);
	i2c_write_reg16_data8(imgsensor.i2c_write_id, 0x392b, 0x00);
	i2c_write_reg16_data8(imgsensor.i2c_write_id, 0x392c, 0x00);
	i2c_write_reg16_data8(imgsensor.i2c_write_id, 0x392d, 0x05);
	i2c_write_reg16_data8(imgsensor.i2c_write_id, 0x392e, 0xb0);
	i2c_write_reg16_data8(imgsensor.i2c_write_id, 0x392f, 0xcb);
	i2c_write_reg16_data8(imgsensor.i2c_write_id, 0x38b3, 0x07);
	i2c_write_reg16_data8(imgsensor.i2c_write_id, 0x3885, 0x07);
	i2c_write_reg16_data8(imgsensor.i2c_write_id, 0x382b, 0x5a);
	i2c_write_reg16_data8(imgsensor.i2c_write_id, 0x3670, 0x68);

	i2c_write_reg16_data8(imgsensor.i2c_write_id, 0x3208, 0x00);
	i2c_write_reg16_data8(imgsensor.i2c_write_id, 0x3929, 0x03);
	i2c_write_reg16_data8(imgsensor.i2c_write_id, 0x392a, 0x36);
	i2c_write_reg16_data8(imgsensor.i2c_write_id, 0x3208, 0x10);
	i2c_write_reg16_data8(imgsensor.i2c_write_id, 0x3208, 0xa0);
	
	

	LOG_INF("stream on s\n");
	i2c_write_reg16_data8(imgsensor.i2c_write_id, 0x0106, 0x00);//,
	//i2c_write_reg16_data8(imgsensor.i2c_write_id, 0x0100, 0x01);//,
    LOG_INF("stream on e\n");                            
                                

};

static void set_dummy(void)
{
	LOG_INF("set_dummy\n");
}

static void set_max_framerate(UINT16 framerate, kal_bool min_framelength_en)
{
	kal_uint32 frame_length = imgsensor.frame_length;
	/* unsigned long flags; */

	SENSORDB("framerate = %d, min framelength should enable? %d\n", framerate,
		 min_framelength_en);

	frame_length = imgsensor.pclk / framerate * 10 / imgsensor.line_length;
	spin_lock(&imgsensor_drv_lock);
	imgsensor.frame_length =
		(frame_length > imgsensor.min_frame_length) ? frame_length : imgsensor.min_frame_length;
	imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;
	/* dummy_line = frame_length - imgsensor.min_frame_length; */
	/* if (dummy_line < 0) */
	/* imgsensor.dummy_line = 0; */
	/* else */
	/* imgsensor.dummy_line = dummy_line; */
	/* imgsensor.frame_length = frame_length + imgsensor.dummy_line; */
	if (imgsensor.frame_length > imgsensor_info.max_frame_length) {
		imgsensor.frame_length = imgsensor_info.max_frame_length;
		imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;
	}
	if (min_framelength_en)
		imgsensor.min_frame_length = imgsensor.frame_length;
	spin_unlock(&imgsensor_drv_lock);
	//set_dummy();
}				/*      set_max_framerate  */



/*************************************************************************
 * FUNCTION
 *	set_shutter
 *
 * DESCRIPTION
 *	This function set e-shutter of sensor to change exposure time.
 *
 * PARAMETERS
 *	iShutter : exposured lines
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static void set_shutter(kal_uint16 shutter)
{
	unsigned long flags;
	kal_uint16 realtime_fps = 0;

	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.shutter = shutter;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

	/* write_shutter(shutter); */
	/* 0x3500, 0x3501, 0x3502 will increase VBLANK to get exposure larger than frame exposure */
	/* AE doesn't update sensor gain at capture mode, thus extra exposure lines must be updated here. */

	/* OV Recommend Solution */
	/* if shutter bigger than frame_length, should extend frame length first */
	spin_lock(&imgsensor_drv_lock);
	if (shutter > imgsensor.min_frame_length - imgsensor_info.margin)
		imgsensor.frame_length = shutter + imgsensor_info.margin;
	else
		imgsensor.frame_length = imgsensor.min_frame_length;
	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
		imgsensor.frame_length = imgsensor_info.max_frame_length;
	imgsensor.frame_length = (imgsensor.frame_length >> 1) << 1;
	spin_unlock(&imgsensor_drv_lock);
	shutter = (shutter < imgsensor_info.min_shutter) ? imgsensor_info.min_shutter : shutter;
	shutter = (shutter > (imgsensor_info.max_frame_length - imgsensor_info.margin))
		? (imgsensor_info.max_frame_length - imgsensor_info.margin) : shutter;
	shutter = (shutter >> 1) << 1;

	if (imgsensor.autoflicker_en) {
		realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length;
		if (realtime_fps >= 297 && realtime_fps <= 305)
			set_max_framerate(296, 0);
		else if (realtime_fps >= 147 && realtime_fps <= 150)
			set_max_framerate(146, 0);
	}
	LOG_INF("Exit! shutter =%d, framelength =%d, for flicker realtime_fps=%d\n", shutter,
		imgsensor.frame_length, realtime_fps);
		
	//	i2c_write_reg16_data8(0x20, 0x3501, 0x08);//
	//i2c_write_reg16_data8(0x20, 0x3502, 0x00);//
	
	//i2c_write_reg16_data8(0x20, 0x3509, 0xa0);//

}



static kal_uint32 streaming_control(kal_bool enable)
{
	LOG_INF("streaming_enable(0=Sw Standby,1=streaming): %d\n", enable);
	if (enable)
		i2c_write_reg16_data8(imgsensor.i2c_write_id, 0x0100, 0X01);
	else
		i2c_write_reg16_data8(imgsensor.i2c_write_id, 0x0100, 0x00);
	return ERROR_NONE;
}
static void preview_setting(void)
{
	LOG_INF(" E\n");

	/* 3.2 Raw 10bit 1632x1224 30fps 2lane 720M bps/lane */
	/* ;XVCLK=24Mhz, SCLK=72Mhz, MIPI 720Mbps, DACCLK=180Mhz, Tline = 8.925926us */
	/* mdelay(5); */


}				/*      preview_setting  */


static void capture_setting(kal_uint16 currefps)
{
	LOG_INF("E! currefps:%d\n", currefps);

}


static void normal_video_setting(kal_uint16 currefps)
{
	LOG_INF("E! currefps:%d\n", currefps);
	LOG_INF("E! video just has 30fps preview size setting ,NOT HAS 24FPS SETTING!\n");
	preview_setting();
}



static void slim_video_setting(void)
{
	LOG_INF("E\n");
	preview_setting();
}


/*************************************************************************
 * FUNCTION
 *	get_imgsensor_id
 *
 * DESCRIPTION
 *	This function get the sensor ID
 *
 * PARAMETERS
 *	*sensorID : return the sensor ID
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/

//extern sensorhsmID_flag;
//extern sensorse47xxID_flag;

static kal_uint32 get_imgsensor_id(UINT32 *sensor_id)
{
	kal_uint8 i = 0;
	kal_uint8 retry = 2;

    cw9281_sensor_poweron();
	mdelay(50);

	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			*sensor_id = (((i2c_read_reg16_data8(imgsensor.i2c_write_id+1, 0x300A) << 8) | i2c_read_reg16_data8(imgsensor.i2c_write_id+1, 0x300B))+1);
			if (*sensor_id == imgsensor_info.sensor_id) {
				printk("[cw9281_camera_sensor]i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id,*sensor_id);

				 return ERROR_NONE;

			}
            printk("[cw9281_camera_sensor]Read sensor id fail, write id: 0x%x, id: 0x%x  imgsensor_info.sensor_id:0x%x\n", imgsensor.i2c_write_id,*sensor_id, imgsensor_info.sensor_id);
            retry--;
		} while (retry > 0);
		i++;
		retry = 2;
	}
	if (*sensor_id != imgsensor_info.sensor_id) {
		/* if Sensor ID is not correct, Must set *sensor_id to 0xFFFFFFFF */
		*sensor_id = 0xFFFFFFFF;
		return ERROR_SENSOR_CONNECT_FAIL;
	}
	return ERROR_NONE;
}

/*************************************************************************
 * FUNCTION
 *	open
 *
 * DESCRIPTION
 *	This function initialize the registers of CMOS sensor
 *
 * PARAMETERS
 *	None
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 open(void)
{

	kal_uint32 sensor_id = 0;
	printk("cw9281 open\n");
    
	get_imgsensor_id(&sensor_id);

    if (imgsensor_info.sensor_id != sensor_id)
    {
        return ERROR_SENSOR_CONNECT_FAIL;
    }

	/* initail sequence write in  */
	sensor_init();
	spin_lock(&imgsensor_drv_lock);

	imgsensor.autoflicker_en = KAL_FALSE;
	imgsensor.sensor_mode = IMGSENSOR_MODE_INIT;
	imgsensor.shutter = 0x3D0;
	imgsensor.gain = 0x100;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	imgsensor.frame_length = imgsensor_info.pre.framelength;
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.dummy_pixel = 0;
	imgsensor.dummy_line = 0;
	imgsensor.ihdr_en = 0;
	imgsensor.test_pattern = KAL_FALSE;
	imgsensor.current_fps = imgsensor_info.pre.max_framerate;
	spin_unlock(&imgsensor_drv_lock);
	
	

#if defined(CONFIG_H205_LED)
	aeon_gpio_set(11);
#endif	

	return ERROR_NONE;
}				/*      open  */



/*************************************************************************
 * FUNCTION
 *	close
 *
 * DESCRIPTION
 *
 *
 * PARAMETERS
 *	None
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 close(void)
{
	LOG_INF("E\n");

    cw9281_sensor_powerdown();

	/*No Need to implement this function */
 #if defined(CONFIG_H205_LED)
	aeon_gpio_set(12);
#endif	
	return ERROR_NONE;
}				/*      close  */


/*************************************************************************
 * FUNCTION
 * preview
 *
 * DESCRIPTION
 *	This function start the sensor preview.
 *
 * PARAMETERS
 *	*image_window : address pointer of pixel numbers in one period of HSYNC
 *  *sensor_config_data : address pointer of line numbers in one period of VSYNC
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 preview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_PREVIEW;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	/* imgsensor.video_mode = KAL_FALSE; */
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.frame_length = imgsensor_info.pre.framelength;
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	preview_setting();
	return ERROR_NONE;
}				/*      preview   */

/*************************************************************************
 * FUNCTION
 *	capture
 *
 * DESCRIPTION
 *	This function setup the CMOS sensor in capture MY_OUTPUT mode
 *
 * PARAMETERS
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 capture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E! current %d fps\n", imgsensor.current_fps);
#if 1	
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CAPTURE;
	if (imgsensor.current_fps == imgsensor_info.cap1.max_framerate) {
		/* PIP capture: 24fps for less than 13M, 20fps for 16M,15fps for 20M */
		imgsensor.pclk = imgsensor_info.cap1.pclk;
		imgsensor.line_length = imgsensor_info.cap1.linelength;
		imgsensor.frame_length = imgsensor_info.cap1.framelength;
		imgsensor.min_frame_length = imgsensor_info.cap1.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	} else {
		if (imgsensor.current_fps != imgsensor_info.cap.max_framerate)
			LOG_INF("Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n",
				imgsensor_info.cap1.max_framerate / 10, imgsensor_info.cap.max_framerate);
		imgsensor.pclk = imgsensor_info.cap.pclk;
		imgsensor.line_length = imgsensor_info.cap.linelength;
		imgsensor.frame_length = imgsensor_info.cap.framelength;
		imgsensor.min_frame_length = imgsensor_info.cap.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
		imgsensor.current_fps = imgsensor_info.cap.max_framerate;
	}
	spin_unlock(&imgsensor_drv_lock);

	capture_setting(imgsensor.current_fps);
	mdelay(100);
#endif
	return ERROR_NONE;
}				/* capture() */

static kal_uint32 normal_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			       MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");
#if 1
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_VIDEO;
	imgsensor.pclk = imgsensor_info.normal_video.pclk;
	imgsensor.line_length = imgsensor_info.normal_video.linelength;
	imgsensor.frame_length = imgsensor_info.normal_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.normal_video.framelength;
	/* imgsensor.current_fps = 300; */
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	normal_video_setting(30);
	mdelay(10);
#endif

	return ERROR_NONE;
}				/*      normal_video   */

static kal_uint32 hs_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			   MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");
#if 1
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_HIGH_SPEED_VIDEO;
	imgsensor.pclk = imgsensor_info.hs_video.pclk;
	/* imgsensor.video_mode = KAL_TRUE; */
	imgsensor.line_length = imgsensor_info.hs_video.linelength;
	imgsensor.frame_length = imgsensor_info.hs_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.hs_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	/* imgsensor.current_fps = 300; */
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	preview_setting();
	mdelay(10);
#endif
	return ERROR_NONE;
}				/*      hs_video   */

static kal_uint32 slim_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			     MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");
#if 1
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_SLIM_VIDEO;
	imgsensor.pclk = imgsensor_info.slim_video.pclk;
	/* imgsensor.video_mode = KAL_TRUE; */
	imgsensor.line_length = imgsensor_info.slim_video.linelength;
	imgsensor.frame_length = imgsensor_info.slim_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.slim_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	/* imgsensor.current_fps = 300; */
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	slim_video_setting();
	mdelay(10);
#endif
	return ERROR_NONE;
}				/*      slim_video       */



static kal_uint32 get_resolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *sensor_resolution)
{
	LOG_INF("E\n");
	sensor_resolution->SensorFullWidth = imgsensor_info.cap.grabwindow_width;
	sensor_resolution->SensorFullHeight = imgsensor_info.cap.grabwindow_height;

	sensor_resolution->SensorPreviewWidth = imgsensor_info.pre.grabwindow_width;
	sensor_resolution->SensorPreviewHeight = imgsensor_info.pre.grabwindow_height;

	sensor_resolution->SensorVideoWidth = imgsensor_info.normal_video.grabwindow_width;
	sensor_resolution->SensorVideoHeight = imgsensor_info.normal_video.grabwindow_height;


	sensor_resolution->SensorHighSpeedVideoWidth = imgsensor_info.hs_video.grabwindow_width;
	sensor_resolution->SensorHighSpeedVideoHeight = imgsensor_info.hs_video.grabwindow_height;

	sensor_resolution->SensorSlimVideoWidth = imgsensor_info.slim_video.grabwindow_width;
	sensor_resolution->SensorSlimVideoHeight = imgsensor_info.slim_video.grabwindow_height;
	return ERROR_NONE;
}				/*      get_resolution  */

static kal_uint32 get_info(enum MSDK_SCENARIO_ID_ENUM scenario_id,
	MSDK_SENSOR_INFO_STRUCT *sensor_info,
	MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("scenario_id = %d\n", scenario_id);


	/* sensor_info->SensorVideoFrameRate = imgsensor_info.normal_video.max_framerate/10; */ /* not use */
	/* sensor_info->SensorStillCaptureFrameRate= imgsensor_info.cap.max_framerate/10; */ /* not use */
	/* imgsensor_info->SensorWebCamCaptureFrameRate= imgsensor_info.v.max_framerate; */ /* not use */

	sensor_info->SensorClockPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorClockFallingPolarity = SENSOR_CLOCK_POLARITY_LOW;	/* not use */
	sensor_info->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;	/* inverse with datasheet */
	sensor_info->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorInterruptDelayLines = 4;	/* not use */
	sensor_info->SensorResetActiveHigh = FALSE;	/* not use */
	sensor_info->SensorResetDelayCount = 5;	/* not use */

	sensor_info->SensroInterfaceType = imgsensor_info.sensor_interface_type;
	sensor_info->MIPIsensorType = imgsensor_info.mipi_sensor_type;
	sensor_info->SettleDelayMode = imgsensor_info.mipi_settle_delay_mode;
	sensor_info->SensorOutputDataFormat = imgsensor_info.sensor_output_dataformat;

	sensor_info->CaptureDelayFrame = imgsensor_info.cap_delay_frame;
	sensor_info->PreviewDelayFrame = imgsensor_info.pre_delay_frame;
	sensor_info->VideoDelayFrame = imgsensor_info.video_delay_frame;
	sensor_info->HighSpeedVideoDelayFrame = imgsensor_info.hs_video_delay_frame;
	sensor_info->SlimVideoDelayFrame = imgsensor_info.slim_video_delay_frame;

	sensor_info->SensorMasterClockSwitch = 0;	/* not use */
	sensor_info->SensorDrivingCurrent = imgsensor_info.isp_driving_current;

	sensor_info->AEShutDelayFrame = imgsensor_info.ae_shut_delay_frame;	/* The frame of setting
										 * shutter default 0 for TG int
										 */
	sensor_info->AESensorGainDelayFrame = imgsensor_info.ae_sensor_gain_delay_frame;	/* The frame of setting
												 * sensor gain
												 */
	sensor_info->AEISPGainDelayFrame = imgsensor_info.ae_ispGain_delay_frame;
	sensor_info->IHDR_Support = imgsensor_info.ihdr_support;
	sensor_info->IHDR_LE_FirstLine = imgsensor_info.ihdr_le_firstline;
	sensor_info->SensorModeNum = imgsensor_info.sensor_mode_num;

	sensor_info->SensorMIPILaneNumber = imgsensor_info.mipi_lane_num;
	sensor_info->SensorClockFreq = imgsensor_info.mclk;
	sensor_info->SensorClockDividCount = 3;	/* not use */
	sensor_info->SensorClockRisingCount = 0;
	sensor_info->SensorClockFallingCount = 2;	/* not use */
	sensor_info->SensorPixelClockCount = 3;	/* not use */
	sensor_info->SensorDataLatchCount = 2;	/* not use */

	sensor_info->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->SensorWidthSampling = 0;	/* 0 is default 1x */
	sensor_info->SensorHightSampling = 0;	/* 0 is default 1x */
	sensor_info->SensorPacketECCOrder = 1;

	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:  //0
		sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;
		//sensor_info->DPCM_INFO = imgsensor_dpcm_info_ov8858[0];
		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.pre.mipi_data_lp2hs_settle_dc;

		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG: //1
		sensor_info->SensorGrabStartX = imgsensor_info.cap.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.cap.starty;
		//sensor_info->DPCM_INFO = imgsensor_dpcm_info_ov8858[1];
		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.cap.mipi_data_lp2hs_settle_dc;

		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW: //2

		sensor_info->SensorGrabStartX = imgsensor_info.normal_video.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.normal_video.starty;
		//sensor_info->DPCM_INFO = imgsensor_dpcm_info_ov8858[2];
		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.normal_video.mipi_data_lp2hs_settle_dc;

		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO: //3
		sensor_info->SensorGrabStartX = imgsensor_info.hs_video.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.hs_video.starty;
		//sensor_info->DPCM_INFO = imgsensor_dpcm_info_ov8858[3];
		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.hs_video.mipi_data_lp2hs_settle_dc;

		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO: //4
		sensor_info->SensorGrabStartX = imgsensor_info.slim_video.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.slim_video.starty;
		//sensor_info->DPCM_INFO = imgsensor_dpcm_info_ov8858[4];
		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.slim_video.mipi_data_lp2hs_settle_dc;

		break;
	default:
		sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;
		//sensor_info->DPCM_INFO = imgsensor_dpcm_info_ov8858[0];
		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.pre.mipi_data_lp2hs_settle_dc;
		break;
	}

	return ERROR_NONE;
}				/*      get_info  */

static kal_uint32 control(enum MSDK_SCENARIO_ID_ENUM scenario_id,
	MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
	MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("scenario_id = %d\n", scenario_id);
	spin_lock(&imgsensor_drv_lock);
	imgsensor.current_scenario_id = scenario_id;
	spin_unlock(&imgsensor_drv_lock);
	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		preview(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		capture(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		normal_video(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		hs_video(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		slim_video(image_window, sensor_config_data);
		break;
	default:
		LOG_INF("Error ScenarioId setting");
		preview(image_window, sensor_config_data);
		return ERROR_INVALID_SCENARIO_ID;
	}
	return ERROR_NONE;
}				/* control() */

static kal_uint32 set_max_framerate_by_scenario(enum MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 framerate)
{
	kal_uint32 frame_length;

	LOG_INF("scenario_id = %d, framerate = %d\n", scenario_id, framerate);
	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		frame_length = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.pre.framelength) ?
			(frame_length - imgsensor_info.pre.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		set_dummy();
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		if (framerate == 0)
			return ERROR_NONE;
		frame_length = imgsensor_info.normal_video.pclk / framerate * 10 /
			imgsensor_info.normal_video.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.normal_video.framelength) ?
			(frame_length - imgsensor_info.normal_video.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.normal_video.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		set_dummy();
		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		if (imgsensor.current_fps == imgsensor_info.cap1.max_framerate) {
			frame_length = imgsensor_info.cap1.pclk / framerate * 10 / imgsensor_info.cap1.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.cap1.framelength) ?
				(frame_length - imgsensor_info.cap1.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.cap1.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
		} else {
			if (imgsensor.current_fps != imgsensor_info.cap.max_framerate)
				LOG_INF("Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n",
					framerate, imgsensor_info.cap.max_framerate / 10);
			frame_length = imgsensor_info.cap.pclk / framerate * 10 / imgsensor_info.cap.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.cap.framelength) ?
				(frame_length - imgsensor_info.cap.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.cap.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
		}
		set_dummy();
		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		frame_length = imgsensor_info.hs_video.pclk / framerate * 10 / imgsensor_info.hs_video.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.hs_video.framelength) ?
			(frame_length - imgsensor_info.hs_video.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.hs_video.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		set_dummy();
		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		frame_length = imgsensor_info.slim_video.pclk / framerate * 10 / imgsensor_info.slim_video.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.slim_video.framelength) ?
			(frame_length - imgsensor_info.slim_video.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.slim_video.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		set_dummy();
		break;
	default:  /*coding with  preview scenario by default*/
		frame_length = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.pre.framelength) ?
			(frame_length - imgsensor_info.pre.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		set_dummy();
		LOG_INF("error scenario_id = %d, we use preview scenario\n", scenario_id);
		break;
	}
	return ERROR_NONE;
}


static kal_uint32 get_default_framerate_by_scenario(enum MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 *framerate)
{
	LOG_INF("scenario_id = %d\n", scenario_id);

	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		*framerate = imgsensor_info.pre.max_framerate;
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		*framerate = imgsensor_info.normal_video.max_framerate;
		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		*framerate = imgsensor_info.cap.max_framerate;
		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		*framerate = imgsensor_info.hs_video.max_framerate;
		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		*framerate = imgsensor_info.slim_video.max_framerate;
		break;
	default:
		break;
	}

	return ERROR_NONE;
}

static kal_uint32 feature_control(MSDK_SENSOR_FEATURE_ENUM feature_id,
				  UINT8 *feature_para, UINT32 *feature_para_len)
{
	UINT16 *feature_return_para_16 = (UINT16 *)feature_para;
	//UINT16 *feature_data_16 = (UINT16 *)feature_para;
	UINT32 *feature_return_para_32 = (UINT32 *)feature_para;
	UINT32 *feature_data_32 = (UINT32 *)feature_para;
	unsigned long long *feature_data = (unsigned long long *) feature_para;
	/* unsigned long long *feature_return_para=(unsigned long long *) feature_para; */

	struct SENSOR_WINSIZE_INFO_STRUCT *wininfo;

	/* LOG_INF("feature_id = %d\n", feature_id); */
	switch (feature_id) {
	case SENSOR_FEATURE_GET_PERIOD:
		*feature_return_para_16++ = imgsensor.line_length;
		*feature_return_para_16 = imgsensor.frame_length;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
		*feature_return_para_32 = imgsensor.pclk;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_SET_ESHUTTER:
		//set_shutter(*feature_data);
		break;
	case SENSOR_FEATURE_SET_NIGHTMODE:
		//night_mode((BOOL)*feature_data);
		break;
	case SENSOR_FEATURE_SET_GAIN:
		//set_gain((UINT16)*feature_data);
		break;
	case SENSOR_FEATURE_SET_FLASHLIGHT:
		break;
	case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
		break;
	case SENSOR_FEATURE_SET_REGISTER:
	LOG_INF("cgq: SENSOR_FEATURE_SET_REGISTER\n");
		break;
	case SENSOR_FEATURE_GET_REGISTER:
	LOG_INF("cgq: SENSOR_FEATURE_GET_REGISTER\n");
		break;
	case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
		/* get the lens driver ID from EEPROM or just return LENS_DRIVER_ID_DO_NOT_CARE */
		/* if EEPROM does not exist in camera module. */
		*feature_return_para_32 = LENS_DRIVER_ID_DO_NOT_CARE;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_SET_VIDEO_MODE:
		//set_video_mode(*feature_data);
		break;
	case SENSOR_FEATURE_CHECK_SENSOR_ID:
		get_imgsensor_id(feature_return_para_32);
		break;
	case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE:
		//set_auto_flicker_mode((BOOL)*feature_data_16, *(feature_data_16 + 1));
		break;
	case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
		set_max_framerate_by_scenario((enum MSDK_SCENARIO_ID_ENUM)*feature_data, *(feature_data + 1));
		break;
	case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
		get_default_framerate_by_scenario((enum MSDK_SCENARIO_ID_ENUM)*(feature_data),
			(MUINT32 *)(uintptr_t)(*(feature_data + 1)));
		break;
	case SENSOR_FEATURE_SET_TEST_PATTERN:
		//set_test_pattern_mode((BOOL)*feature_data);
		break;
	case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE: /*for factory mode auto testing */
		*feature_return_para_32 = imgsensor_info.checksum_value;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_SET_FRAMERATE:
		LOG_INF("current fps :%d\n", (UINT32)*feature_data);
		spin_lock(&imgsensor_drv_lock);
		imgsensor.current_fps = *feature_data;
		spin_unlock(&imgsensor_drv_lock);
		break;
	case SENSOR_FEATURE_SET_HDR:
		LOG_INF("ihdr enable :%d\n", (BOOL)*feature_data);
		spin_lock(&imgsensor_drv_lock);
		imgsensor.ihdr_en = (BOOL)*feature_data;
		spin_unlock(&imgsensor_drv_lock);
		break;
	case SENSOR_FEATURE_GET_CROP_INFO:
		LOG_INF("SENSOR_FEATURE_GET_CROP_INFO scenarioId:%d\n", (UINT32)*feature_data);
		wininfo = (struct SENSOR_WINSIZE_INFO_STRUCT *)(uintptr_t)(*(feature_data + 1));
		switch (*feature_data_32) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[1], sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[2], sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[3], sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[4], sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[0], sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		}
		break;
	case SENSOR_FEATURE_SET_IHDR_SHUTTER_GAIN:
		//LOG_INF("SENSOR_SET_SENSOR_IHDR LE=%d, SE=%d, Gain=%d\n",
		//	(UINT16)*feature_data, (UINT16)*(feature_data + 1), (UINT16)*(feature_data + 2));
		//ihdr_write_shutter_gain((UINT16)*feature_data,
		//	(UINT16)*(feature_data + 1), (UINT16)*(feature_data + 2));
		break;
	case SENSOR_FEATURE_SET_STREAMING_SUSPEND:
		LOG_INF("SENSOR_FEATURE_SET_STREAMING_SUSPEND\n");
		streaming_control(KAL_FALSE);
		break;
	case SENSOR_FEATURE_SET_STREAMING_RESUME:
		LOG_INF("SENSOR_FEATURE_SET_STREAMING_RESUME, shutter:%llu\n", *feature_data);
		if (*feature_data != 0)
			set_shutter(*feature_data);
		streaming_control(KAL_TRUE);
		break;
		
	#if 1
	case SENSOR_FEATURE_GET_PIXEL_RATE:
	printk("\n [cm60_mipi_raw_sensor] SENSOR_FEATURE_GET_PIXEL_RATE \n");
	switch (*feature_data) {
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
	*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
	(imgsensor_info.cap.pclk /
	(imgsensor_info.cap.linelength - 80))*
	imgsensor_info.cap.grabwindow_width;
	break;

	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
	*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
	(imgsensor_info.normal_video.pclk /
	(imgsensor_info.normal_video.linelength - 80))*
	imgsensor_info.normal_video.grabwindow_width;
	break;

	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
	*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
	(imgsensor_info.hs_video.pclk /
	(imgsensor_info.hs_video.linelength - 80))*
	imgsensor_info.hs_video.grabwindow_width;
	break;

	case MSDK_SCENARIO_ID_SLIM_VIDEO:
	*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
	(imgsensor_info.slim_video.pclk /
	(imgsensor_info.slim_video.linelength - 80))*
	imgsensor_info.slim_video.grabwindow_width;
	break;

	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
	default:
	*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
	(imgsensor_info.pre.pclk /
	(imgsensor_info.pre.linelength - 80))*
	imgsensor_info.pre.grabwindow_width;
	break;
	}
	break;
		#endif
	default:
		break;
	}

	return ERROR_NONE;
}				/*    feature_control()  */

static struct SENSOR_FUNCTION_STRUCT sensor_func = {
	open,
	get_info,
	get_resolution,
	feature_control,
	control,
	close
};

UINT32 CW9281_MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc)
{
	/* To Do : Check Sensor status here */
	if (pfFunc != NULL)
		*pfFunc = &sensor_func;
	return ERROR_NONE;
}				

