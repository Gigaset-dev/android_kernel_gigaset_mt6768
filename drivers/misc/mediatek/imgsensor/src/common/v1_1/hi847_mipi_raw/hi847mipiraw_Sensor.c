/*
 * Copyright (C) 2016 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 */

/*****************************************************************************
 *
 * Filename:
 * ---------
 *	 HI847mipi_Sensor.c
 *
 * Project:
 * --------
 *	 SOP
 *
 * Description:
 * ------------
 *	 Source code of Sensor driver
 *
 *
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *============================================================================
 
 ****************************************************************************/
#define PFX "HI847_camera_sensor"
#define pr_fmt(fmt) PFX "[%s] " fmt, __func__


#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/atomic.h>
#include <linux/types.h>

#include "kd_camera_typedef.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"

#include "hi847mipiraw_Sensor.h"


//extern bool read_eeprom_hi847( kal_uint16 addr, BYTE *data, kal_uint32 size);

#undef VENDOR_EDIT

#define USE_BURST_MODE 1
#define ENABLE_PDAF 1

//#define I2C_BUFFER_LEN 255 /* trans# max is 255, each 3 bytes */

#if USE_BURST_MODE
static kal_uint16 hi847_table_write_cmos_sensor(
		kal_uint16 * para, kal_uint32 len);
#endif
static DEFINE_SPINLOCK(imgsensor_drv_lock);


static struct imgsensor_info_struct imgsensor_info = {
	.sensor_id = HI847_SENSOR_ID,

	.checksum_value = 0xe32ded8,
	.pre = { //3264X2448@30FPS_NO PD		
		.pclk = 288000000,
		.linelength = 828,
		.framelength = 2898, 
		.startx = 0,	
		.starty = 0,
		.grabwindow_width = 3264,
		.grabwindow_height = 2448,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 300,
		.mipi_pixel_rate = 320000000, 
	},
	.cap = {  //3264X2448@30FPS_NO PD		
		.pclk = 288000000,
		.linelength = 828,
		.framelength = 2898, 
		.startx = 0,	
		.starty = 0,
		.grabwindow_width = 3264,
		.grabwindow_height = 2448,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 300,
		.mipi_pixel_rate = 320000000, 
	},
	.normal_video = { //3264X2448@30FPS PD2		
		.pclk = 288000000,
		.linelength = 833, 	
		.framelength = 2881, 
		.startx = 0,	
		.starty = 0,
		.grabwindow_width = 3264,
		.grabwindow_height = 2448,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 300,
		.mipi_pixel_rate = 400000000, 
	},
	.hs_video = {
		.pclk = 288000000,
		.linelength = 830,
		.framelength = 963,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 1280 ,
		.grabwindow_height = 720 ,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 900,
		.mipi_pixel_rate = 200000000,
	},
	.slim_video = {
		.pclk = 288000000,
		.linelength = 831,
		.framelength = 1444,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 1920,
		.grabwindow_height = 1080,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 600,
		.mipi_pixel_rate = 400000000,
	},

	.margin = 4,		/* sensor framelength & shutter margin */
	.min_shutter = 4,	/* min shutter */
	
	.min_gain = 64,
	.max_gain = 1024,
	.min_gain_iso = 100,
	.exp_step = 2,
	.gain_step = 4,
	.gain_type = 3,
	
	.max_frame_length = 0xffffff,
	.ae_shut_delay_frame = 0,
	.ae_sensor_gain_delay_frame = 0,
	.ae_ispGain_delay_frame = 2,	/* isp gain delay frame for AE cycle */
	.ihdr_support = 0,	/* 1, support; 0,not support */
	.ihdr_le_firstline = 0,	/* 1,le first ; 0, se first */
	.temperature_support = 0,/* 1, support; 0,not support */
	.sensor_mode_num = 5,	/* support sensor mode num */

	.cap_delay_frame = 1,	/* enter capture delay frame num */
	.pre_delay_frame = 1,	/* enter preview delay frame num */
	.video_delay_frame = 1,	/* enter video delay frame num */
	.hs_video_delay_frame = 1,
	.slim_video_delay_frame = 1,	/* enter slim video delay frame num */
	.frame_time_delay_frame = 2,

	.isp_driving_current = ISP_DRIVING_6MA,
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
	.mipi_sensor_type = MIPI_OPHY_NCSI2,
	.mipi_settle_delay_mode = MIPI_SETTLEDELAY_AUTO, //0,MIPI_SETTLEDELAY_AUTO; 1,MIPI_SETTLEDELAY_MANNUAL
	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_Gr,
	.mclk = 24, 
	.mipi_lane_num = SENSOR_MIPI_4_LANE,
	.i2c_addr_table = {0x42, 0x40, 0xff},
	.i2c_speed = 400,
};

static struct imgsensor_struct imgsensor = {
	.mirror = IMAGE_NORMAL,	/* mirrorflip information */
	.sensor_mode = IMGSENSOR_MODE_INIT,
	/* IMGSENSOR_MODE enum value,record current sensor mode,such as:
	 * INIT, Preview, Capture, Video,High Speed Video, Slim Video
	 */
	.shutter = 0x100,	/* current shutter */
	.gain = 0xe0,		/* current gain */
	.dummy_pixel = 0,	/* current dummypixel */
	.dummy_line = 0,	/* current dummyline */
	.current_fps = 300,
	.autoflicker_en = KAL_FALSE,
	.test_pattern = KAL_FALSE,
	.current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW,
	.ihdr_mode = 0, /* sensor need support LE, SE with HDR feature */
	.i2c_write_id = 0x42, /* record current sensor's i2c write id */
};


/* Sensor output window information */
static struct SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[5] = {
	{ 3296, 2480,   8,  14, 3280, 2452,	 3280, 2452,    8, 2, 3264, 2448, 0, 0, 3264, 2448},	// preview (1632 x 1224)
	{ 3296, 2480,   8,  14, 3280, 2452,	 3280, 2452,    8, 2, 3264, 2448, 0, 0, 3264, 2448},	// capture (3264 x 2448)
	{ 3296, 2480,   8,  14, 3280, 2452,	 3280, 2452,    8, 2, 3264, 2448, 0, 0, 3264, 2448},	// video (3264 x 2448)
	{ 3296, 2480,   8, 516, 3280, 1448,  1640,  724,  180, 2, 1280,  720, 0, 0, 1280,  720},	// hight speed video (1280 x 720)
	{ 3296, 2480,   8, 698, 3280, 1084,  3280, 1084,  680, 2, 1920, 1080, 0, 0, 1920, 1080},	// slim video (1920 x 1080)	
};


#if ENABLE_PDAF

static struct SENSOR_VC_INFO_STRUCT SENSOR_VC_INFO[2] =
{
	/* Capture mode setting */
	 {0x02, //VC_Num
	  0x0A, //VC_PixelNum	
	  0x00, //ModeSelect	/* 0:auto 1:direct */
	  0x00, //EXPO_Ratio	/* 1/1, 1/2, 1/4, 1/8 */
	  0x00, //0DValue		/* 0D Value */
	  0x00, //RG_STATSMODE	/* STATS divistion mode 0:16x16  1:8x8  2:4x4  3:1x1 */
	  0x00, 0x2b, 0x0cc0, 0x0990, 	// VC0 Maybe image data?
	  0x00, 0x00, 0x0000, 0x0000,	// VC1 MVHDR
	  0x01, 0x30, 0x00FA, 0x0258,   // VC2 PDAF 0x01, 0x2b, 0x00C8, 0x0258,  0x01, 0x30, 0x00FA, 0x0258,
	  0x00, 0x00, 0x0000, 0x0000},	// VC3 ??
	/* Video mode setting */
	 {0x02, //VC_Num
	  0x0a, //VC_PixelNum	
	  0x00, //ModeSelect	/* 0:auto 1:direct */
	  0x00, //EXPO_Ratio	/* 1/1, 1/2, 1/4, 1/8 */
	  0x00, //0DValue		/* 0D Value */
	  0x00, //RG_STATSMODE	/* STATS divistion mode 0:16x16  1:8x8  2:4x4  3:1x1 */
	  0x00, 0x2b, 0x0cc0, 0x0990, 	// VC0 Maybe image data?
	  0x00, 0x00, 0x0000, 0x0000,	// VC1 MVHDR
	  0x01, 0x30, 0x00FA, 0x0258,   // VC2 PDAF 0x01, 0x2b, 0x00C8, 0x0258,    0x01, 0x30, 0x00FA, 0x0258,
	  0x00, 0x00, 0x0000, 0x0000},	// VC3 ??
};

static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info =
{
	.i4OffsetX = 32,
	.i4OffsetY = 24,
	.i4PitchX = 32,
	.i4PitchY = 32,
	.i4PairNum = 8,
	.i4SubBlkW = 16,
	.i4SubBlkH = 8,
	.i4BlockNumX = 100,
	.i4BlockNumY = 75,
	.iMirrorFlip = 0,
	.i4PosL = {
					{ 36, 29 }, { 52, 29 }, { 44, 33 }, { 60, 33 },
					{ 36, 45 }, { 52, 45 }, { 44, 49 }, { 60, 49 },
				},
	.i4PosR = {
					{ 36, 25 }, { 52, 25 }, { 44, 37 }, { 60, 37 },
					{ 36, 41 }, { 52, 41 }, { 44, 53 }, { 60, 53 },
				},

};
#endif

static void write_cmos_sensor(kal_uint16 addr, kal_uint16 para)
{
	char pusendcmd[4] = {(char)(addr >> 8), (char)(addr & 0xFF),
			     (char)(para >> 8), (char)(para & 0xFF)};

	/*kdSetI2CSpeed(imgsensor_info.i2c_speed);*/
	/* Add this func to set i2c speed by each sensor */
	iWriteRegI2C(pusendcmd, 4, imgsensor.i2c_write_id);
}

static kal_uint16 read_cmos_sensor_8(kal_uint16 addr)
{
	kal_uint16 get_byte = 0;
	char pusendcmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF) };

	iReadRegI2C(pusendcmd, 2, (u8 *)&get_byte, 1, imgsensor.i2c_write_id);
	return get_byte;
}

static void write_cmos_sensor_8(kal_uint16 addr, kal_uint8 para)
{
	char pusendcmd[3] = {(char)(addr >> 8), (char)(addr & 0xFF),
			(char)(para & 0xFF)};

	iWriteRegI2C(pusendcmd, 3, imgsensor.i2c_write_id);
}

static void set_dummy(void)
{
	pr_debug("dummyline = %d, dummypixels = %d\n",
		imgsensor.dummy_line, imgsensor.dummy_pixel);
	write_cmos_sensor(0x020e, imgsensor.frame_length & 0xFFFF);
	write_cmos_sensor(0x0206, imgsensor.line_length);

}	/*	set_dummy  */

static kal_uint32 return_sensor_id(void)
{
	return ((read_cmos_sensor_8(0x0716) << 8) | read_cmos_sensor_8(0x0717));

}

static void set_max_framerate(UINT16 framerate, kal_bool min_framelength_en)
{
	/*kal_int16 dummy_line;*/
	kal_uint32 frame_length = imgsensor.frame_length;

	pr_debug("framerate = %d, min framelength should enable %d\n", framerate, min_framelength_en);

	frame_length = imgsensor.pclk / framerate * 10 / imgsensor.line_length;
	spin_lock(&imgsensor_drv_lock);
	if (frame_length >= imgsensor.min_frame_length)
		imgsensor.frame_length = frame_length;
	else
		imgsensor.frame_length = imgsensor.min_frame_length;

	imgsensor.dummy_line =
			imgsensor.frame_length - imgsensor.min_frame_length;

	if (imgsensor.frame_length > imgsensor_info.max_frame_length) {
		imgsensor.frame_length = imgsensor_info.max_frame_length;
		imgsensor.dummy_line =
			imgsensor.frame_length - imgsensor.min_frame_length;
	}
	if (min_framelength_en)
		imgsensor.min_frame_length = imgsensor.frame_length;
	spin_unlock(&imgsensor_drv_lock);
	set_dummy();
}	/*	set_max_framerate  */

static void write_shutter(kal_uint32 shutter)
{
	kal_uint16 realtime_fps = 0;

	spin_lock(&imgsensor_drv_lock);
	if (shutter > imgsensor.min_frame_length - imgsensor_info.margin)
		imgsensor.frame_length = shutter + imgsensor_info.margin;
	else
		imgsensor.frame_length = imgsensor.min_frame_length;
	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
		imgsensor.frame_length = imgsensor_info.max_frame_length;
	spin_unlock(&imgsensor_drv_lock);
	if (shutter < imgsensor_info.min_shutter)
		shutter = imgsensor_info.min_shutter;

	if (imgsensor.autoflicker_en) {
		realtime_fps = imgsensor.pclk / imgsensor.line_length * 10
				/ imgsensor.frame_length;
		if (realtime_fps >= 297 && realtime_fps <= 305)
			set_max_framerate(296, 0);
		else if (realtime_fps >= 147 && realtime_fps <= 150)
			set_max_framerate(146, 0);
		else {
			/* Extend frame length */
			write_cmos_sensor_8(0x0211, (imgsensor.frame_length & 0xFF0000) >> 16);
			write_cmos_sensor(0x020e, imgsensor.frame_length);
		}
	} else {
		/* Extend frame length*/
		write_cmos_sensor_8(0x0211, (imgsensor.frame_length & 0xFF0000) >> 16);
		write_cmos_sensor(0x020e, imgsensor.frame_length);
	}

	/* Update Shutter */
	write_cmos_sensor_8(0x020D, (shutter & 0xFF0000) >> 16);
	write_cmos_sensor(0x020A, shutter);

	pr_debug("frame_length = %d , shutter = %d \n", imgsensor.frame_length, shutter);

}	/*	write_shutter  */

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
static void set_shutter(kal_uint32 shutter)
{
	unsigned long flags;

	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.shutter = shutter;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

	write_shutter(shutter);
} /* set_shutter */


/*************************************************************************
 * FUNCTION
 *	set_shutter_frame_length
 *
 * DESCRIPTION
 *	for frame & 3A sync
 *
 *************************************************************************/
static void set_shutter_frame_length(kal_uint16 shutter,
				     kal_uint16 frame_length,
				     kal_bool auto_extend_en)
{
	unsigned long flags;
	kal_uint16 realtime_fps = 0;
	kal_int32 dummy_line = 0;

	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.shutter = shutter;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

	spin_lock(&imgsensor_drv_lock);
	/* Change frame time */
	if (frame_length > 1)
		dummy_line = frame_length - imgsensor.frame_length;

	imgsensor.frame_length = imgsensor.frame_length + dummy_line;

	if (shutter > imgsensor.frame_length - imgsensor_info.margin)
		imgsensor.frame_length = shutter + imgsensor_info.margin;

	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
		imgsensor.frame_length = imgsensor_info.max_frame_length;
	spin_unlock(&imgsensor_drv_lock);
	shutter = (shutter < imgsensor_info.min_shutter)
			? imgsensor_info.min_shutter : shutter;
	shutter =
	(shutter > (imgsensor_info.max_frame_length - imgsensor_info.margin))
		? (imgsensor_info.max_frame_length - imgsensor_info.margin)
		: shutter;

	if (imgsensor.autoflicker_en) {
		realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 /
				imgsensor.frame_length;
		if (realtime_fps >= 297 && realtime_fps <= 305)
			set_max_framerate(296, 0);
		else if (realtime_fps >= 147 && realtime_fps <= 150)
			set_max_framerate(146, 0);
		else {
			/* Extend frame length */
			write_cmos_sensor_8(0x0211, (imgsensor.frame_length & 0xFF0000) >> 16 );
			write_cmos_sensor(0x020e, imgsensor.frame_length);
		}
	} else {
		/* Extend frame length */
			write_cmos_sensor_8(0x0211, (imgsensor.frame_length & 0xFF0000) >> 16 );
			write_cmos_sensor(0x020e, imgsensor.frame_length);
	}

	/* Update Shutter */
	write_cmos_sensor_8(0x020D, (shutter & 0xFF0000) >> 16 );
	write_cmos_sensor(0x020A, shutter);
	
    pr_debug("frame_length = %d , shutter = %d \n", imgsensor.frame_length, shutter);

}	/* set_shutter_frame_length */

static kal_uint16 gain2reg(const kal_uint16 gain)
{
    kal_uint16 reg_gain = 0x0000;
    reg_gain = gain / 4 - 16;

    return (kal_uint16)reg_gain;
}

/*************************************************************************
 * FUNCTION
 *	set_gain
 *
 * DESCRIPTION
 *	This function is to set global gain to sensor.
 *
 * PARAMETERS
 *	iGain : sensor global gain(base: 0x40)
 *
 * RETURNS
 *	the actually gain set to sensor.
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint16 set_gain(kal_uint16 gain)
{
	kal_uint16 reg_gain, max_gain = 16 * BASEGAIN;

	if (gain < BASEGAIN || gain > max_gain) {
		pr_debug("Error max gain setting: %d\n", max_gain);

		if (gain < BASEGAIN)
			gain = BASEGAIN;
		else if (gain > max_gain)
			gain = max_gain;
	}

	reg_gain = gain2reg(gain);
	spin_lock(&imgsensor_drv_lock);
	imgsensor.gain = reg_gain;
	spin_unlock(&imgsensor_drv_lock);
	pr_debug("gain = %d, reg_gain = 0x%x, max_gain:0x%x\n ",
		gain, reg_gain, max_gain);

	reg_gain = reg_gain & 0x00FF;
	write_cmos_sensor_8(0x0213, reg_gain);

	return gain;
} /* set_gain */

/*
static void set_mirror_flip(kal_uint8 image_mirror)
{
	pr_debug("image_mirror = %d", image_mirror);

	switch (image_mirror) {
	case IMAGE_NORMAL:
		write_cmos_sensor(0x0000, 0x0000);
		break;
	case IMAGE_H_MIRROR:
		write_cmos_sensor(0x0000, 0x0100);

		break;
	case IMAGE_V_MIRROR:
		write_cmos_sensor(0x0000, 0x0200);

		break;
	case IMAGE_HV_MIRROR:
		write_cmos_sensor(0x0000, 0x0300);

		break;
	default:
		pr_debug("Error image_mirror setting");
		break;
	}

}
*/
static kal_uint32 streaming_control(kal_bool enable)
{
	pr_debug("streaming_enable(0=Sw Standby,1=streaming): %d\n", enable);

	if (enable)
		write_cmos_sensor(0x0b00, 0x0100); // stream on
	else
		write_cmos_sensor(0x0b00, 0x0000); // stream off

	mdelay(10);
	return ERROR_NONE;
}

#if USE_BURST_MODE
#define I2C_BUFFER_LEN 1020 /* trans# max is 255, each 3 bytes */
static kal_uint16 hi847_table_write_cmos_sensor(kal_uint16 *para,
						 kal_uint32 len)
{
	char puSendCmd[I2C_BUFFER_LEN];
	kal_uint32 tosend, IDX;
	kal_uint16 addr = 0, addr_last = 0, data;
	int ret = 0;
	int retry_cnt = 0;

	tosend = 0;
	IDX = 0;

	while (len > IDX) {
		addr = para[IDX];

		{
			puSendCmd[tosend++] = (char)(addr >> 8);
			puSendCmd[tosend++] = (char)(addr & 0xFF);
			data = para[IDX + 1];
			puSendCmd[tosend++] = (char)(data >> 8);
			puSendCmd[tosend++] = (char)(data & 0xFF);
			IDX += 2;
			addr_last = addr;

		}
		/* Write when remain buffer size is less than 3 bytes
		 * or reach end of data
		 */
		if ((I2C_BUFFER_LEN - tosend) < 4
			|| IDX == len || addr != addr_last) {
			ret = iBurstWriteReg_multi(puSendCmd,
						tosend,
						imgsensor.i2c_write_id,
						4,
						imgsensor_info.i2c_speed);

			if( ret < 0 )
			{
				while(ret < 0 ){
					ret = iBurstWriteReg_multi(puSendCmd,
							tosend,
							imgsensor.i2c_write_id,
							4,
							imgsensor_info.i2c_speed);
					retry_cnt++;

					if( retry_cnt >3 )
					{
						retry_cnt =0 ;
						break;
					}
				}
			}

			tosend = 0;
		}
	}
	return 0;
}
#endif


static kal_uint16 hi847_init_setting[] = {
0x0790, 0x0100,	
0x2000, 0x0000,
0x2002, 0x0058,
0x2006, 0x40B2,
0x2008, 0xB05C,
0x200A, 0x8446,
0x200C, 0x40B2,
0x200E, 0xB082,
0x2010, 0x8450,
0x2012, 0x40B2,
0x2014, 0xB0AE,
0x2016, 0x84C6,
0x2018, 0x40B2,
0x201A, 0xB11A,
0x201C, 0x84BC,
0x201E, 0x40B2,
0x2020, 0xB34A,
0x2022, 0x84B4,
0x2024, 0x40B2,
0x2026, 0xB386,
0x2028, 0x84B0,
0x202A, 0x40B2,
0x202C, 0xB3B4,
0x202E, 0x84B8,
0x2030, 0x40B2,
0x2032, 0xB0F4,
0x2034, 0x8470,
0x2036, 0x40B2,
0x2038, 0xB3EA,
0x203A, 0x847C,
0x203C, 0x40B2,
0x203E, 0xB658,
0x2040, 0x8478,
0x2042, 0x40B2,
0x2044, 0xB67E,
0x2046, 0x847E,
0x2048, 0x40B2,
0x204A, 0xB78E,
0x204C, 0x843A,
0x204E, 0x40B2,
0x2050, 0xB980,
0x2052, 0x845C,
0x2054, 0x40B2,
0x2056, 0xB9B0,
0x2058, 0x845E,
0x205A, 0x4130,
0x205C, 0x1292,
0x205E, 0xD016,
0x2060, 0xB3D2,
0x2062, 0x0B00,
0x2064, 0x2002,
0x2066, 0xD2E2,
0x2068, 0x0381,
0x206A, 0x93C2,
0x206C, 0x0263,
0x206E, 0x2001,
0x2070, 0x4130,
0x2072, 0x422D,
0x2074, 0x403E,
0x2076, 0x888E,
0x2078, 0x403F,
0x207A, 0x192A,
0x207C, 0x1292,
0x207E, 0x843E,
0x2080, 0x3FF7,
0x2082, 0x422D,
0x2084, 0x403E,
0x2086, 0x192A,
0x2088, 0x403F,
0x208A, 0x888E,
0x208C, 0x1292,
0x208E, 0x843E,
0x2090, 0xB3D2,
0x2092, 0x0267,
0x2094, 0x2403,
0x2096, 0xD0F2,
0x2098, 0x0040,
0x209A, 0x0381,
0x209C, 0x90F2,
0x209E, 0x0010,
0x20A0, 0x0260,
0x20A2, 0x2002,
0x20A4, 0x1292,
0x20A6, 0x84BC,
0x20A8, 0x1292,
0x20AA, 0xD020,
0x20AC, 0x4130,
0x20AE, 0x1292,
0x20B0, 0x8470,
0x20B2, 0x1292,
0x20B4, 0x8452,
0x20B6, 0x0900,
0x20B8, 0x7118,
0x20BA, 0x1292,
0x20BC, 0x848E,
0x20BE, 0x0900,
0x20C0, 0x7112,
0x20C2, 0x0800,
0x20C4, 0x7A20,
0x20C6, 0x4292,
0x20C8, 0x87DE,
0x20CA, 0x7334,
0x20CC, 0x0F00,
0x20CE, 0x7304,
0x20D0, 0x421F,
0x20D2, 0x8718,
0x20D4, 0x1292,
0x20D6, 0x846E,
0x20D8, 0x1292,
0x20DA, 0x8488,
0x20DC, 0x0B00,
0x20DE, 0x7114,
0x20E0, 0x0002,
0x20E2, 0x1292,
0x20E4, 0x848C,
0x20E6, 0x1292,
0x20E8, 0x8454,
0x20EA, 0x43C2,
0x20EC, 0x86EE,
0x20EE, 0x1292,
0x20F0, 0x8444,
0x20F2, 0x4130,
0x20F4, 0x4392,
0x20F6, 0x7360,
0x20F8, 0xB3D2,
0x20FA, 0x0B00,
0x20FC, 0x2402,
0x20FE, 0xC2E2,
0x2100, 0x0381,
0x2102, 0x0900,
0x2104, 0x732C,
0x2106, 0x4382,
0x2108, 0x7360,
0x210A, 0x422D,
0x210C, 0x403E,
0x210E, 0x87F0,
0x2110, 0x403F,
0x2112, 0x87E8,
0x2114, 0x1292,
0x2116, 0x843E,
0x2118, 0x4130,
0x211A, 0x120B,
0x211C, 0x120A,
0x211E, 0x4392,
0x2120, 0x87FA,
0x2122, 0x4392,
0x2124, 0x760E,
0x2126, 0x0900,
0x2128, 0x760C,
0x212A, 0x421B,
0x212C, 0x760A,
0x212E, 0x903B,
0x2130, 0x0201,
0x2132, 0x2408,
0x2134, 0x903B,
0x2136, 0x0102,
0x2138, 0x2405,
0x213A, 0x4292,
0x213C, 0x030A,
0x213E, 0x87F8,
0x2140, 0x1292,
0x2142, 0x849A,
0x2144, 0x903B,
0x2146, 0x0020,
0x2148, 0x2010,
0x214A, 0x403B,
0x214C, 0x8498,
0x214E, 0x422F,
0x2150, 0x12AB,
0x2152, 0x403F,
0x2154, 0x0028,
0x2156, 0x12AB,
0x2158, 0x403B,
0x215A, 0x84C4,
0x215C, 0x407F,
0x215E, 0xFFAA,
0x2160, 0x12AB,
0x2162, 0x407F,
0x2164, 0x0055,
0x2166, 0x12AB,
0x2168, 0x3FDC,
0x216A, 0x903B,
0x216C, 0x0021,
0x216E, 0x2890,
0x2170, 0x903B,
0x2172, 0x0100,
0x2174, 0x200D,
0x2176, 0x403F,
0x2178, 0x0028,
0x217A, 0x1292,
0x217C, 0x8498,
0x217E, 0x425F,
0x2180, 0x0306,
0x2182, 0x1292,
0x2184, 0x84C4,
0x2186, 0x4FC2,
0x2188, 0x0318,
0x218A, 0x0261,
0x218C, 0x0000,
0x218E, 0x3FC9,
0x2190, 0x903B,
0x2192, 0x0101,
0x2194, 0x2858,
0x2196, 0x903B,
0x2198, 0x0200,
0x219A, 0x2450,
0x219C, 0x903B,
0x219E, 0x0201,
0x21A0, 0x2C47,
0x21A2, 0x903B,
0x21A4, 0x0102,
0x21A6, 0x2041,
0x21A8, 0x93E2,
0x21AA, 0x0262,
0x21AC, 0x240A,
0x21AE, 0x425F,
0x21B0, 0x0306,
0x21B2, 0x1292,
0x21B4, 0x84C4,
0x21B6, 0x4F4E,
0x21B8, 0x4EC2,
0x21BA, 0x0318,
0x21BC, 0x0260,
0x21BE, 0x0000,
0x21C0, 0x3FB0,
0x21C2, 0x403A,
0x21C4, 0x8030,
0x21C6, 0x4382,
0x21C8, 0x0326,
0x21CA, 0x4382,
0x21CC, 0x0328,
0x21CE, 0x421B,
0x21D0, 0x030C,
0x21D2, 0x930B,
0x21D4, 0x2420,
0x21D6, 0x4A5F,
0x21D8, 0x0001,
0x21DA, 0x1292,
0x21DC, 0x84C4,
0x21DE, 0x4F4E,
0x21E0, 0x4A5F,
0x21E2, 0x0001,
0x21E4, 0x9F0E,
0x21E6, 0x2402,
0x21E8, 0x5392,
0x21EA, 0x0326,
0x21EC, 0x4ECA,
0x21EE, 0x0001,
0x21F0, 0x533B,
0x21F2, 0x2411,
0x21F4, 0x4A6F,
0x21F6, 0x1292,
0x21F8, 0x84C4,
0x21FA, 0x4F4E,
0x21FC, 0x4A6F,
0x21FE, 0x9F0E,
0x2200, 0x2402,
0x2202, 0x5392,
0x2204, 0x0326,
0x2206, 0x4ECA,
0x2208, 0x0000,
0x220A, 0x533B,
0x220C, 0x532A,
0x220E, 0x0260,
0x2210, 0x0000,
0x2212, 0x930B,
0x2214, 0x23E0,
0x2216, 0x40B2,
0x2218, 0xAA55,
0x221A, 0x0328,
0x221C, 0xB0F2,
0x221E, 0x0040,
0x2220, 0x0381,
0x2222, 0x277F,
0x2224, 0xD3D2,
0x2226, 0x0267,
0x2228, 0x3F7C,
0x222A, 0x0261,
0x222C, 0x0000,
0x222E, 0x3F79,
0x2230, 0x903B,
0x2232, 0x0201,
0x2234, 0x23FA,
0x2236, 0x1292,
0x2238, 0x84C0,
0x223A, 0x3F73,
0x223C, 0x1292,
0x223E, 0x84C0,
0x2240, 0x0261,
0x2242, 0x0000,
0x2244, 0x3F6E,
0x2246, 0x903B,
0x2248, 0x0040,
0x224A, 0x2018,
0x224C, 0x422F,
0x224E, 0x1292,
0x2250, 0x8498,
0x2252, 0x12B0,
0x2254, 0xF0EA,
0x2256, 0x907F,
0x2258, 0xFFAA,
0x225A, 0x240D,
0x225C, 0x5392,
0x225E, 0x0312,
0x2260, 0x12B0,
0x2262, 0xF0EA,
0x2264, 0x907F,
0x2266, 0x0055,
0x2268, 0x2403,
0x226A, 0x5392,
0x226C, 0x0312,
0x226E, 0x3F59,
0x2270, 0x5392,
0x2272, 0x0310,
0x2274, 0x3F56,
0x2276, 0x5392,
0x2278, 0x0310,
0x227A, 0x3FF2,
0x227C, 0x903B,
0x227E, 0x0080,
0x2280, 0x23D4,
0x2282, 0x4382,
0x2284, 0x0312,
0x2286, 0x4382,
0x2288, 0x0310,
0x228A, 0x0261,
0x228C, 0x0000,
0x228E, 0x3F49,
0x2290, 0x932B,
0x2292, 0x2005,
0x2294, 0x403F,
0x2296, 0x0028,
0x2298, 0x1292,
0x229A, 0x8498,
0x229C, 0x3F42,
0x229E, 0x903B,
0x22A0, 0x0003,
0x22A2, 0x284B,
0x22A4, 0x923B,
0x22A6, 0x2015,
0x22A8, 0x403F,
0x22AA, 0x0023,
0x22AC, 0x1292,
0x22AE, 0x8498,
0x22B0, 0x421B,
0x22B2, 0x87F8,
0x22B4, 0x421F,
0x22B6, 0x030C,
0x22B8, 0x9F0B,
0x22BA, 0x2F33,
0x22BC, 0x1292,
0x22BE, 0x84BA,
0x22C0, 0x930F,
0x22C2, 0x2004,
0x22C4, 0x5392,
0x22C6, 0x0312,
0x22C8, 0x531B,
0x22CA, 0x3FF4,
0x22CC, 0x5392,
0x22CE, 0x0310,
0x22D0, 0x3FFB,
0x22D2, 0x903B,
0x22D4, 0x0009,
0x22D6, 0x2818,
0x22D8, 0x903B,
0x22DA, 0x0010,
0x22DC, 0x23A6,
0x22DE, 0x403F,
0x22E0, 0x0027,
0x22E2, 0x1292,
0x22E4, 0x8498,
0x22E6, 0x421B,
0x22E8, 0x87F8,
0x22EA, 0x421F,
0x22EC, 0x030C,
0x22EE, 0x9F0B,
0x22F0, 0x2F18,
0x22F2, 0x1292,
0x22F4, 0x84BA,
0x22F6, 0x930F,
0x22F8, 0x2004,
0x22FA, 0x5392,
0x22FC, 0x0312,
0x22FE, 0x531B,
0x2300, 0x3FF4,
0x2302, 0x5392,
0x2304, 0x0310,
0x2306, 0x3FFB,
0x2308, 0x922B,
0x230A, 0x238F,
0x230C, 0x421B,
0x230E, 0x87F8,
0x2310, 0x421F,
0x2312, 0x030C,
0x2314, 0x9F0B,
0x2316, 0x2C0B,
0x2318, 0x1292,
0x231A, 0x84C2,
0x231C, 0x934F,
0x231E, 0x240A,
0x2320, 0x5392,
0x2322, 0x0312,
0x2324, 0x531B,
0x2326, 0x421F,
0x2328, 0x030C,
0x232A, 0x9F0B,
0x232C, 0x2BF5,
0x232E, 0x0261,
0x2330, 0x0000,
0x2332, 0x3EF7,
0x2334, 0x5392,
0x2336, 0x0310,
0x2338, 0x3FF5,
0x233A, 0x930B,
0x233C, 0x277F,
0x233E, 0x931B,
0x2340, 0x277A,
0x2342, 0x3F73,
0x2344, 0x413A,
0x2346, 0x413B,
0x2348, 0x4130,
0x234A, 0x4F0C,
0x234C, 0x403F,
0x234E, 0x0267,
0x2350, 0xF0FF,
0x2352, 0xFFDF,
0x2354, 0x0000,
0x2356, 0xF0FF,
0x2358, 0xFFEF,
0x235A, 0x0000,
0x235C, 0x421D,
0x235E, 0x84B0,
0x2360, 0x403E,
0x2362, 0x06F9,
0x2364, 0x4C0F,
0x2366, 0x1292,
0x2368, 0x84AC,
0x236A, 0x4F4E,
0x236C, 0xB31E,
0x236E, 0x2403,
0x2370, 0xD0F2,
0x2372, 0x0020,
0x2374, 0x0267,
0x2376, 0xB32E,
0x2378, 0x2403,
0x237A, 0xD0F2,
0x237C, 0x0010,
0x237E, 0x0267,
0x2380, 0xC3E2,
0x2382, 0x0267,
0x2384, 0x4130,
0x2386, 0x120B,
0x2388, 0x120A,
0x238A, 0x403A,
0x238C, 0x1140,
0x238E, 0x1292,
0x2390, 0xD080,
0x2392, 0x430B,
0x2394, 0x4A0F,
0x2396, 0x532A,
0x2398, 0x1292,
0x239A, 0x84A4,
0x239C, 0x4F0E,
0x239E, 0x430F,
0x23A0, 0x5E82,
0x23A2, 0x87FC,
0x23A4, 0x6F82,
0x23A6, 0x87FE,
0x23A8, 0x531B,
0x23AA, 0x923B,
0x23AC, 0x2BF3,
0x23AE, 0x413A,
0x23B0, 0x413B,
0x23B2, 0x4130,
0x23B4, 0xF0F2,
0x23B6, 0x007F,
0x23B8, 0x0267,
0x23BA, 0x421D,
0x23BC, 0x84B6,
0x23BE, 0x403E,
0x23C0, 0x01F9,
0x23C2, 0x1292,
0x23C4, 0x84AC,
0x23C6, 0x4F4E,
0x23C8, 0xF35F,
0x23CA, 0x2403,
0x23CC, 0xD0F2,
0x23CE, 0xFF80,
0x23D0, 0x0267,
0x23D2, 0xB36E,
0x23D4, 0x2404,
0x23D6, 0xD0F2,
0x23D8, 0x0040,
0x23DA, 0x0267,
0x23DC, 0x3C03,
0x23DE, 0xF0F2,
0x23E0, 0xFFBF,
0x23E2, 0x0267,
0x23E4, 0xC2E2,
0x23E6, 0x0267,
0x23E8, 0x4130,
0x23EA, 0x120B,
0x23EC, 0x120A,
0x23EE, 0x8231,
0x23F0, 0x430B,
0x23F2, 0x93C2,
0x23F4, 0x0C0A,
0x23F6, 0x2404,
0x23F8, 0xB3D2,
0x23FA, 0x0B05,
0x23FC, 0x2401,
0x23FE, 0x431B,
0x2400, 0x422D,
0x2402, 0x403E,
0x2404, 0x192A,
0x2406, 0x403F,
0x2408, 0x888E,
0x240A, 0x1292,
0x240C, 0x843E,
0x240E, 0x930B,
0x2410, 0x20F4,
0x2412, 0x93E2,
0x2414, 0x0241,
0x2416, 0x24EB,
0x2418, 0x403A,
0x241A, 0x0292,
0x241C, 0x4AA2,
0x241E, 0x0A00,
0x2420, 0xB2E2,
0x2422, 0x0361,
0x2424, 0x2405,
0x2426, 0x4A2F,
0x2428, 0x1292,
0x242A, 0x8474,
0x242C, 0x4F82,
0x242E, 0x0A1C,
0x2430, 0x93C2,
0x2432, 0x0360,
0x2434, 0x34CD,
0x2436, 0x430C,
0x2438, 0x4C0F,
0x243A, 0x5F0F,
0x243C, 0x4F0D,
0x243E, 0x510D,
0x2440, 0x4F0E,
0x2442, 0x5A0E,
0x2444, 0x4E1E,
0x2446, 0x0002,
0x2448, 0x4F1F,
0x244A, 0x192A,
0x244C, 0x1202,
0x244E, 0xC232,
0x2450, 0x4303,
0x2452, 0x4E82,
0x2454, 0x0130,
0x2456, 0x4F82,
0x2458, 0x0138,
0x245A, 0x421E,
0x245C, 0x013A,
0x245E, 0x421F,
0x2460, 0x013C,
0x2462, 0x4132,
0x2464, 0x108E,
0x2466, 0x108F,
0x2468, 0xEF4E,
0x246A, 0xEF0E,
0x246C, 0xF37F,
0x246E, 0xC312,
0x2470, 0x100F,
0x2472, 0x100E,
0x2474, 0x4E8D,
0x2476, 0x0000,
0x2478, 0x531C,
0x247A, 0x922C,
0x247C, 0x2BDD,
0x247E, 0xB3D2,
0x2480, 0x1921,
0x2482, 0x2403,
0x2484, 0x410F,
0x2486, 0x1292,
0x2488, 0x847E,
0x248A, 0x403B,
0x248C, 0x843E,
0x248E, 0x422D,
0x2490, 0x410E,
0x2492, 0x403F,
0x2494, 0x1908,
0x2496, 0x12AB,
0x2498, 0x403D,
0x249A, 0x0005,
0x249C, 0x403E,
0x249E, 0x0292,
0x24A0, 0x403F,
0x24A2, 0x86E4,
0x24A4, 0x12AB,
0x24A6, 0x421F,
0x24A8, 0x060E,
0x24AA, 0x9F82,
0x24AC, 0x8720,
0x24AE, 0x288D,
0x24B0, 0x9382,
0x24B2, 0x060E,
0x24B4, 0x248A,
0x24B6, 0x90BA,
0x24B8, 0x0010,
0x24BA, 0x0000,
0x24BC, 0x2C0B,
0x24BE, 0x93C2,
0x24C0, 0x86EE,
0x24C2, 0x2008,
0x24C4, 0x403F,
0x24C6, 0x06A7,
0x24C8, 0xD0FF,
0x24CA, 0x0007,
0x24CC, 0x0000,
0x24CE, 0xF0FF,
0x24D0, 0xFFF8,
0x24D2, 0x0000,
0x24D4, 0x4392,
0x24D6, 0x8720,
0x24D8, 0x403F,
0x24DA, 0x06A7,
0x24DC, 0xD2EF,
0x24DE, 0x0000,
0x24E0, 0xC2EF,
0x24E2, 0x0000,
0x24E4, 0x93C2,
0x24E6, 0x87D3,
0x24E8, 0x2068,
0x24EA, 0xB0F2,
0x24EC, 0x0040,
0x24EE, 0x0B05,
0x24F0, 0x2461,
0x24F2, 0xD3D2,
0x24F4, 0x0410,
0x24F6, 0xB3E2,
0x24F8, 0x0381,
0x24FA, 0x2089,
0x24FC, 0x90B2,
0x24FE, 0x0030,
0x2500, 0x0A00,
0x2502, 0x2C52,
0x2504, 0x93C2,
0x2506, 0x86EE,
0x2508, 0x204F,
0x250A, 0x430E,
0x250C, 0x430C,
0x250E, 0x4C0F,
0x2510, 0x5F0F,
0x2512, 0x5F0F,
0x2514, 0x5F0F,
0x2516, 0x4F1F,
0x2518, 0x8668,
0x251A, 0xF03F,
0x251C, 0x07FF,
0x251E, 0x903F,
0x2520, 0x0400,
0x2522, 0x343E,
0x2524, 0x5F0E,
0x2526, 0x531C,
0x2528, 0x923C,
0x252A, 0x2BF1,
0x252C, 0x4E0F,
0x252E, 0x930E,
0x2530, 0x3834,
0x2532, 0x110F,
0x2534, 0x110F,
0x2536, 0x110F,
0x2538, 0x9382,
0x253A, 0x86EE,
0x253C, 0x2023,
0x253E, 0x5F82,
0x2540, 0x87D6,
0x2542, 0x403B,
0x2544, 0x87D6,
0x2546, 0x4B2F,
0x2548, 0x12B0,
0x254A, 0xB624,
0x254C, 0x4F8B,
0x254E, 0x0000,
0x2550, 0x430C,
0x2552, 0x4C0D,
0x2554, 0x5D0D,
0x2556, 0x5D0D,
0x2558, 0x5D0D,
0x255A, 0x403A,
0x255C, 0x87D8,
0x255E, 0x421B,
0x2560, 0x87D6,
0x2562, 0x4B0F,
0x2564, 0x8A2F,
0x2566, 0x4F0E,
0x2568, 0x4E0F,
0x256A, 0x5F0F,
0x256C, 0x7F0F,
0x256E, 0xE33F,
0x2570, 0x8E8D,
0x2572, 0x8668,
0x2574, 0x7F8D,
0x2576, 0x866A,
0x2578, 0x531C,
0x257A, 0x923C,
0x257C, 0x2BEA,
0x257E, 0x4B8A,
0x2580, 0x0000,
0x2582, 0x3C45,
0x2584, 0x9382,
0x2586, 0x86F0,
0x2588, 0x2005,
0x258A, 0x4382,
0x258C, 0x87D6,
0x258E, 0x4382,
0x2590, 0x87D8,
0x2592, 0x3FD7,
0x2594, 0x4F82,
0x2596, 0x87D6,
0x2598, 0x3FD4,
0x259A, 0x503F,
0x259C, 0x0007,
0x259E, 0x3FC9,
0x25A0, 0x5F0E,
0x25A2, 0x503E,
0x25A4, 0xF800,
0x25A6, 0x3FBF,
0x25A8, 0x430F,
0x25AA, 0x12B0,
0x25AC, 0xB624,
0x25AE, 0x4382,
0x25B0, 0x87D6,
0x25B2, 0x3C2D,
0x25B4, 0xC3D2,
0x25B6, 0x0410,
0x25B8, 0x3F9E,
0x25BA, 0x430D,
0x25BC, 0x403E,
0x25BE, 0x0050,
0x25C0, 0x403F,
0x25C2, 0x85C8,
0x25C4, 0x1292,
0x25C6, 0x844E,
0x25C8, 0x3F90,
0x25CA, 0x5392,
0x25CC, 0x8720,
0x25CE, 0x3F84,
0x25D0, 0x403B,
0x25D2, 0x843E,
0x25D4, 0x4A0F,
0x25D6, 0x532F,
0x25D8, 0x422D,
0x25DA, 0x4F0E,
0x25DC, 0x403F,
0x25DE, 0x0E08,
0x25E0, 0x12AB,
0x25E2, 0x422D,
0x25E4, 0x403E,
0x25E6, 0x192A,
0x25E8, 0x410F,
0x25EA, 0x12AB,
0x25EC, 0x3F48,
0x25EE, 0x93C2,
0x25F0, 0x86EE,
0x25F2, 0x2312,
0x25F4, 0x403A,
0x25F6, 0x86E4,
0x25F8, 0x3F11,
0x25FA, 0x403D,
0x25FC, 0x0200,
0x25FE, 0x422E,
0x2600, 0x403F,
0x2602, 0x192A,
0x2604, 0x1292,
0x2606, 0x844E,
0x2608, 0xC3D2,
0x260A, 0x1921,
0x260C, 0x3F02,
0x260E, 0x422D,
0x2610, 0x403E,
0x2612, 0x888E,
0x2614, 0x403F,
0x2616, 0x192A,
0x2618, 0x1292,
0x261A, 0x843E,
0x261C, 0x5231,
0x261E, 0x413A,
0x2620, 0x413B,
0x2622, 0x4130,
0x2624, 0x4382,
0x2626, 0x052C,
0x2628, 0x4F0D,
0x262A, 0x930D,
0x262C, 0x3402,
0x262E, 0xE33D,
0x2630, 0x531D,
0x2632, 0xF03D,
0x2634, 0x07F0,
0x2636, 0x4D0E,
0x2638, 0xC312,
0x263A, 0x100E,
0x263C, 0x110E,
0x263E, 0x110E,
0x2640, 0x110E,
0x2642, 0x930F,
0x2644, 0x3803,
0x2646, 0x4EC2,
0x2648, 0x052C,
0x264A, 0x3C04,
0x264C, 0x4EC2,
0x264E, 0x052D,
0x2650, 0xE33D,
0x2652, 0x531D,
0x2654, 0x4D0F,
0x2656, 0x4130,
0x2658, 0x1292,
0x265A, 0xD048,
0x265C, 0x93C2,
0x265E, 0x86EE,
0x2660, 0x200D,
0x2662, 0xB0F2,
0x2664, 0x0020,
0x2666, 0x0381,
0x2668, 0x2407,
0x266A, 0x9292,
0x266C, 0x8722,
0x266E, 0x0384,
0x2670, 0x2C03,
0x2672, 0xD3D2,
0x2674, 0x0649,
0x2676, 0x4130,
0x2678, 0xC3D2,
0x267A, 0x0649,
0x267C, 0x4130,
0x267E, 0x120B,
0x2680, 0x120A,
0x2682, 0x1209,
0x2684, 0x1208,
0x2686, 0x1207,
0x2688, 0x1206,
0x268A, 0x1205,
0x268C, 0x1204,
0x268E, 0x8231,
0x2690, 0x4F81,
0x2692, 0x0000,
0x2694, 0x4381,
0x2696, 0x0002,
0x2698, 0x4304,
0x269A, 0x411C,
0x269C, 0x0002,
0x269E, 0x5C0C,
0x26A0, 0x4C0F,
0x26A2, 0x5F0F,
0x26A4, 0x5F0F,
0x26A6, 0x5F0F,
0x26A8, 0x5F0F,
0x26AA, 0x5F0F,
0x26AC, 0x503F,
0x26AE, 0x1980,
0x26B0, 0x440D,
0x26B2, 0x5D0D,
0x26B4, 0x4D0E,
0x26B6, 0x5F0E,
0x26B8, 0x4E2E,
0x26BA, 0x4D05,
0x26BC, 0x5505,
0x26BE, 0x5F05,
0x26C0, 0x4516,
0x26C2, 0x0008,
0x26C4, 0x4517,
0x26C6, 0x000A,
0x26C8, 0x460A,
0x26CA, 0x470B,
0x26CC, 0xF30A,
0x26CE, 0xF32B,
0x26D0, 0x4A81,
0x26D2, 0x0004,
0x26D4, 0x4B81,
0x26D6, 0x0006,
0x26D8, 0xB03E,
0x26DA, 0x2000,
0x26DC, 0x2404,
0x26DE, 0xF03E,
0x26E0, 0x1FFF,
0x26E2, 0xE33E,
0x26E4, 0x531E,
0x26E6, 0xF317,
0x26E8, 0x503E,
0x26EA, 0x2000,
0x26EC, 0x4E0F,
0x26EE, 0x5F0F,
0x26F0, 0x7F0F,
0x26F2, 0xE33F,
0x26F4, 0x512C,
0x26F6, 0x4C28,
0x26F8, 0x4309,
0x26FA, 0x4E0A,
0x26FC, 0x4F0B,
0x26FE, 0x480C,
0x2700, 0x490D,
0x2702, 0x1202,
0x2704, 0xC232,
0x2706, 0x12B0,
0x2708, 0xFFC0,
0x270A, 0x4132,
0x270C, 0x108E,
0x270E, 0x108F,
0x2710, 0xEF4E,
0x2712, 0xEF0E,
0x2714, 0xF37F,
0x2716, 0xC312,
0x2718, 0x100F,
0x271A, 0x100E,
0x271C, 0x4E85,
0x271E, 0x0018,
0x2720, 0x4F85,
0x2722, 0x001A,
0x2724, 0x480A,
0x2726, 0x490B,
0x2728, 0x460C,
0x272A, 0x470D,
0x272C, 0x1202,
0x272E, 0xC232,
0x2730, 0x12B0,
0x2732, 0xFFC0,
0x2734, 0x4132,
0x2736, 0x4E0C,
0x2738, 0x4F0D,
0x273A, 0x108C,
0x273C, 0x108D,
0x273E, 0xED4C,
0x2740, 0xED0C,
0x2742, 0xF37D,
0x2744, 0xC312,
0x2746, 0x100D,
0x2748, 0x100C,
0x274A, 0x411E,
0x274C, 0x0004,
0x274E, 0x411F,
0x2750, 0x0006,
0x2752, 0x5E0E,
0x2754, 0x6F0F,
0x2756, 0x5E0E,
0x2758, 0x6F0F,
0x275A, 0x5E0E,
0x275C, 0x6F0F,
0x275E, 0xDE0C,
0x2760, 0xDF0D,
0x2762, 0x4C85,
0x2764, 0x002C,
0x2766, 0x4D85,
0x2768, 0x002E,
0x276A, 0x5314,
0x276C, 0x9224,
0x276E, 0x2B95,
0x2770, 0x5391,
0x2772, 0x0002,
0x2774, 0x92A1,
0x2776, 0x0002,
0x2778, 0x2B8F,
0x277A, 0x5231,
0x277C, 0x4134,
0x277E, 0x4135,
0x2780, 0x4136,
0x2782, 0x4137,
0x2784, 0x4138,
0x2786, 0x4139,
0x2788, 0x413A,
0x278A, 0x413B,
0x278C, 0x4130,
0x278E, 0x120B,
0x2790, 0x120A,
0x2792, 0x1209,
0x2794, 0x8031,
0x2796, 0x000C,
0x2798, 0x425F,
0x279A, 0x0205,
0x279C, 0xC312,
0x279E, 0x104F,
0x27A0, 0x114F,
0x27A2, 0x114F,
0x27A4, 0x114F,
0x27A6, 0x114F,
0x27A8, 0x114F,
0x27AA, 0xF37F,
0x27AC, 0x4F0B,
0x27AE, 0xF31B,
0x27B0, 0x5B0B,
0x27B2, 0x5B0B,
0x27B4, 0x5B0B,
0x27B6, 0x503B,
0x27B8, 0xD194,
0x27BA, 0x4219,
0x27BC, 0x0508,
0x27BE, 0xF039,
0x27C0, 0x2000,
0x27C2, 0x4F0A,
0x27C4, 0xC312,
0x27C6, 0x100A,
0x27C8, 0xE31A,
0x27CA, 0x421F,
0x27CC, 0x87DE,
0x27CE, 0x503F,
0x27D0, 0xFF60,
0x27D2, 0x903F,
0x27D4, 0x00C8,
0x27D6, 0x2C02,
0x27D8, 0x403F,
0x27DA, 0x00C8,
0x27DC, 0x4F82,
0x27DE, 0x7322,
0x27E0, 0xB3D2,
0x27E2, 0x0381,
0x27E4, 0x2009,
0x27E6, 0x421F,
0x27E8, 0x86F0,
0x27EA, 0xD21F,
0x27EC, 0x86EE,
0x27EE, 0x930F,
0x27F0, 0x24B9,
0x27F2, 0x40F2,
0x27F4, 0xFF80,
0x27F6, 0x0619,
0x27F8, 0x1292,
0x27FA, 0xD00A,
0x27FC, 0xB3D2,
0x27FE, 0x0385,
0x2800, 0x2405,
0x2802, 0x421F,
0x2804, 0x880A,
0x2806, 0x4F92,
0x2808, 0x0002,
0x280A, 0x8714,
0x280C, 0x430D,
0x280E, 0x93C2,
0x2810, 0x87D0,
0x2812, 0x2003,
0x2814, 0xB2F2,
0x2816, 0x0360,
0x2818, 0x2001,
0x281A, 0x431D,
0x281C, 0x425F,
0x281E, 0x87D3,
0x2820, 0xD25F,
0x2822, 0x87D2,
0x2824, 0xF37F,
0x2826, 0x5F0F,
0x2828, 0x425E,
0x282A, 0x87CD,
0x282C, 0xDE0F,
0x282E, 0x5F0F,
0x2830, 0x5B0F,
0x2832, 0x4FA2,
0x2834, 0x0402,
0x2836, 0x930D,
0x2838, 0x2007,
0x283A, 0x930A,
0x283C, 0x248E,
0x283E, 0x4F5F,
0x2840, 0x0001,
0x2842, 0xF37F,
0x2844, 0x4FC2,
0x2846, 0x0403,
0x2848, 0x93C2,
0x284A, 0x87CD,
0x284C, 0x2483,
0x284E, 0xC2F2,
0x2850, 0x0400,
0x2852, 0xB2E2,
0x2854, 0x0265,
0x2856, 0x2407,
0x2858, 0x421F,
0x285A, 0x0508,
0x285C, 0xF03F,
0x285E, 0xFFDF,
0x2860, 0xD90F,
0x2862, 0x4F82,
0x2864, 0x0508,
0x2866, 0xB3D2,
0x2868, 0x0383,
0x286A, 0x2484,
0x286C, 0x403F,
0x286E, 0x0508,
0x2870, 0x4FB1,
0x2872, 0x0000,
0x2874, 0x4FB1,
0x2876, 0x0002,
0x2878, 0x4FB1,
0x287A, 0x0004,
0x287C, 0x403F,
0x287E, 0x0500,
0x2880, 0x4FB1,
0x2882, 0x0006,
0x2884, 0x4FB1,
0x2886, 0x0008,
0x2888, 0x4FB1,
0x288A, 0x000A,
0x288C, 0xB3E2,
0x288E, 0x0383,
0x2890, 0x2412,
0x2892, 0xC2E1,
0x2894, 0x0002,
0x2896, 0xB2E2,
0x2898, 0x0383,
0x289A, 0x434F,
0x289C, 0x634F,
0x289E, 0xF37F,
0x28A0, 0x4F4E,
0x28A2, 0x114E,
0x28A4, 0x434E,
0x28A6, 0x104E,
0x28A8, 0x415F,
0x28AA, 0x0007,
0x28AC, 0xF07F,
0x28AE, 0x007F,
0x28B0, 0xDE4F,
0x28B2, 0x4FC1,
0x28B4, 0x0007,
0x28B6, 0xB2F2,
0x28B8, 0x0383,
0x28BA, 0x2415,
0x28BC, 0xF0F1,
0x28BE, 0xFFBF,
0x28C0, 0x0000,
0x28C2, 0xB0F2,
0x28C4, 0x0010,
0x28C6, 0x0383,
0x28C8, 0x434E,
0x28CA, 0x634E,
0x28CC, 0x5E4E,
0x28CE, 0x5E4E,
0x28D0, 0x5E4E,
0x28D2, 0x5E4E,
0x28D4, 0x5E4E,
0x28D6, 0x5E4E,
0x28D8, 0x415F,
0x28DA, 0x0006,
0x28DC, 0xF07F,
0x28DE, 0xFFBF,
0x28E0, 0xDE4F,
0x28E2, 0x4FC1,
0x28E4, 0x0006,
0x28E6, 0xB0F2,
0x28E8, 0x0020,
0x28EA, 0x0383,
0x28EC, 0x2410,
0x28EE, 0xF0F1,
0x28F0, 0xFFDF,
0x28F2, 0x0002,
0x28F4, 0xB0F2,
0x28F6, 0x0040,
0x28F8, 0x0383,
0x28FA, 0x434E,
0x28FC, 0x634E,
0x28FE, 0x5E4E,
0x2900, 0x5E4E,
0x2902, 0x415F,
0x2904, 0x0008,
0x2906, 0xC26F,
0x2908, 0xDE4F,
0x290A, 0x4FC1,
0x290C, 0x0008,
0x290E, 0x93C2,
0x2910, 0x0383,
0x2912, 0x3412,
0x2914, 0xF0F1,
0x2916, 0xFFDF,
0x2918, 0x0000,
0x291A, 0x425E,
0x291C, 0x0382,
0x291E, 0xF35E,
0x2920, 0x5E4E,
0x2922, 0x5E4E,
0x2924, 0x5E4E,
0x2926, 0x5E4E,
0x2928, 0x5E4E,
0x292A, 0x415F,
0x292C, 0x0006,
0x292E, 0xF07F,
0x2930, 0xFFDF,
0x2932, 0xDE4F,
0x2934, 0x4FC1,
0x2936, 0x0006,
0x2938, 0x410F,
0x293A, 0x4FB2,
0x293C, 0x0508,
0x293E, 0x4FB2,
0x2940, 0x050A,
0x2942, 0x4FB2,
0x2944, 0x050C,
0x2946, 0x4FB2,
0x2948, 0x0500,
0x294A, 0x4FB2,
0x294C, 0x0502,
0x294E, 0x4FB2,
0x2950, 0x0504,
0x2952, 0x3C10,
0x2954, 0xD2F2,
0x2956, 0x0400,
0x2958, 0x3F7C,
0x295A, 0x4F6F,
0x295C, 0xF37F,
0x295E, 0x4FC2,
0x2960, 0x0402,
0x2962, 0x3F72,
0x2964, 0x90F2,
0x2966, 0x0011,
0x2968, 0x0619,
0x296A, 0x2B46,
0x296C, 0x50F2,
0x296E, 0xFFF0,
0x2970, 0x0619,
0x2972, 0x3F42,
0x2974, 0x5031,
0x2976, 0x000C,
0x2978, 0x4139,
0x297A, 0x413A,
0x297C, 0x413B,
0x297E, 0x4130,
0x2980, 0x0900,
0x2982, 0x7312,
0x2984, 0x421F,
0x2986, 0x0A08,
0x2988, 0xF03F,
0x298A, 0xF7FF,
0x298C, 0x4F82,
0x298E, 0x0A88,
0x2990, 0x0900,
0x2992, 0x7312,
0x2994, 0x421F,
0x2996, 0x0A0E,
0x2998, 0xF03F,
0x299A, 0x7FFF,
0x299C, 0x4F82,
0x299E, 0x0A8E,
0x29A0, 0x0900,
0x29A2, 0x7312,
0x29A4, 0x421F,
0x29A6, 0x0A1E,
0x29A8, 0xC31F,
0x29AA, 0x4F82,
0x29AC, 0x0A9E,
0x29AE, 0x4130,
0x29B0, 0x4292,
0x29B2, 0x0A08,
0x29B4, 0x0A88,
0x29B6, 0x0900,
0x29B8, 0x7312,
0x29BA, 0x4292,
0x29BC, 0x0A0E,
0x29BE, 0x0A8E,
0x29C0, 0x0900,
0x29C2, 0x7312,
0x29C4, 0x4292,
0x29C6, 0x0A1E,
0x29C8, 0x0A9E,
0x29CA, 0x4130,
0x29CC, 0x7400,
0x29CE, 0x8058,
0x29D0, 0x1807,
0x29D2, 0x00E0,
0x29D4, 0x7002,
0x29D6, 0x17C7,
0x29D8, 0x0045,
0x29DA, 0x0006,
0x29DC, 0x17CC,
0x29DE, 0x0015,
0x29E0, 0x1512,
0x29E2, 0x216F,
0x29E4, 0x005B,
0x29E6, 0x005D,
0x29E8, 0x00DE,
0x29EA, 0x00DD,
0x29EC, 0x5023,
0x29EE, 0x00DE,
0x29F0, 0x005B,
0x29F2, 0x0410,
0x29F4, 0x0091,
0x29F6, 0x0015,
0x29F8, 0x0040,
0x29FA, 0x7023,
0x29FC, 0x1653,
0x29FE, 0x0156,
0x2A00, 0x0001,
0x2A02, 0x2081,
0x2A04, 0x7020,
0x2A06, 0x2F99,
0x2A08, 0x005C,
0x2A0A, 0x0000,
0x2A0C, 0x5040,
0x2A0E, 0x0045,
0x2A10, 0x213A,
0x2A12, 0x0303,
0x2A14, 0x0148,
0x2A16, 0x0049,
0x2A18, 0x0045,
0x2A1A, 0x0046,
0x2A1C, 0x05DD,
0x2A1E, 0x00DE,
0x2A20, 0x00DD,
0x2A22, 0x00DC,
0x2A24, 0x00DE,
0x2A26, 0x04D6,
0x2A28, 0x2014,
0x2A2A, 0x2081,
0x2A2C, 0x7087,
0x2A2E, 0x2F99,
0x2A30, 0x005C,
0x2A32, 0x0002,
0x2A34, 0x5060,
0x2A36, 0x31C0,
0x2A38, 0x2122,
0x2A3A, 0x7800,
0x2A3C, 0xC08C,
0x2A3E, 0x0001,
0x2A40, 0x9038,
0x2A42, 0x59F7,
0x2A44, 0x907A,
0x2A46, 0x03D8,
0x2A48, 0x8D90,
0x2A4A, 0x01C0,
0x2A4C, 0x7400,
0x2A4E, 0x8058,
0x2A50, 0x1807,
0x2A52, 0x00E0,
0x2A54, 0x7002,
0x2A56, 0x17C7,
0x2A58, 0x0045,
0x2A5A, 0x0006,
0x2A5C, 0x17CC,
0x2A5E, 0x0015,
0x2A60, 0x1512,
0x2A62, 0x216F,
0x2A64, 0x005B,
0x2A66, 0x005D,
0x2A68, 0x00DE,
0x2A6A, 0x00DD,
0x2A6C, 0x5023,
0x2A6E, 0x00DE,
0x2A70, 0x005B,
0x2A72, 0x0410,
0x2A74, 0x0091,
0x2A76, 0x0015,
0x2A78, 0x0040,
0x2A7A, 0x7023,
0x2A7C, 0x1653,
0x2A7E, 0x0156,
0x2A80, 0x0001,
0x2A82, 0x2081,
0x2A84, 0x7020,
0x2A86, 0x2F99,
0x2A88, 0x005C,
0x2A8A, 0x0000,
0x2A8C, 0x5040,
0x2A8E, 0x0045,
0x2A90, 0x213A,
0x2A92, 0x0303,
0x2A94, 0x0148,
0x2A96, 0x0049,
0x2A98, 0x0045,
0x2A9A, 0x0046,
0x2A9C, 0x05DD,
0x2A9E, 0x00DE,
0x2AA0, 0x00DD,
0x2AA2, 0x00DC,
0x2AA4, 0x00DE,
0x2AA6, 0x0296,
0x2AA8, 0x2014,
0x2AAA, 0x2081,
0x2AAC, 0x7087,
0x2AAE, 0x2F99,
0x2AB0, 0x005C,
0x2AB2, 0x0002,
0x2AB4, 0x5060,
0x2AB6, 0x31C0,
0x2AB8, 0x2122,
0x2ABA, 0x7800,
0x2ABC, 0xC08C,
0x2ABE, 0x0001,
0x2AC0, 0x9038,
0x2AC2, 0x59F7,
0x2AC4, 0x907A,
0x2AC6, 0x03D8,
0x2AC8, 0x8D90,
0x2ACA, 0x01C0,
0x2ACC, 0x7400,
0x2ACE, 0x2002,
0x2AD0, 0x70DF,
0x2AD2, 0x2F21,
0x2AD4, 0x04C1,
0x2AD6, 0x0D80,
0x2AD8, 0x7800,
0x2ADA, 0x0041,
0x2ADC, 0x7400,
0x2ADE, 0x2004,
0x2AE0, 0x70DF,
0x2AE2, 0x2F21,
0x2AE4, 0x04C2,
0x2AE6, 0x0D80,
0x2AE8, 0x7800,
0x2AEA, 0x7400,
0x2AEC, 0x2008,
0x2AEE, 0x70DF,
0x2AF0, 0x2F21,
0x2AF2, 0x04C3,
0x2AF4, 0x0D80,
0x2AF6, 0x7800,
0x2AF8, 0x7400,
0x2AFA, 0x0004,
0x2AFC, 0x70DF,
0x2AFE, 0x2F22,
0x2B00, 0x7008,
0x2B02, 0x2F1F,
0x2B04, 0x7021,
0x2B06, 0x2F01,
0x2B08, 0x7800,
0x2B0A, 0x7400,
0x2B0C, 0x0002,
0x2B0E, 0x70DF,
0x2B10, 0x3F5F,
0x2B12, 0x703A,
0x2B14, 0x2F01,
0x2B16, 0x7800,
0x2B18, 0x7400,
0x2B1A, 0x2010,
0x2B1C, 0x70DF,
0x2B1E, 0x3F40,
0x2B20, 0x700A,
0x2B22, 0x0FC0,
0x2B24, 0x7800,
0x2B26, 0x7400,
0x2B28, 0x2004,
0x2B2A, 0x70DF,
0x2B2C, 0x2F21,
0x2B2E, 0x04C2,
0x2B30, 0x0D80,
0x2B32, 0x7800,
0x2B34, 0x0041,
0x2B36, 0x7400,
0x2B38, 0x2002,
0x2B3A, 0x70DF,
0x2B3C, 0x2F22,
0x2B3E, 0x04C1,
0x2B40, 0x0D80,
0x2B42, 0x7800,
0x2B44, 0x7400,
0x2B46, 0x0001,
0x2B48, 0x70DF,
0x2B4A, 0x3F5F,
0x2B4C, 0x703A,
0x2B4E, 0x2F01,
0x2B50, 0x7800,
0x2B52, 0x7400,
0x2B54, 0x200A,
0x2B56, 0x70DF,
0x2B58, 0x3F40,
0x2B5A, 0x700A,
0x2B5C, 0x0FC0,
0x2B5E, 0x7800,
0x2B60, 0x7400,
0x2B62, 0x2015,
0x2B64, 0x70DF,
0x2B66, 0x3F5F,
0x2B68, 0x703A,
0x2B6A, 0x2F01,
0x2B6C, 0x7800,
0x2B6E, 0x7400,
0x2B70, 0x7800,
0x2B72, 0x007F,
0x2B74, 0x0000,
0x2B76, 0xB9CC,
0x2B78, 0x0000,
0x2B7A, 0xB9CC,
0x2B7C, 0xBA3C,
0x2B7E, 0x0002,
0x2B80, 0x0000,
0x2B82, 0xBA4C,
0x2B84, 0x0000,
0x2B86, 0xBA4C,
0x2B88, 0xBABC,
0x2B8A, 0x0002,
0x2B8C, 0x0063,
0x2B8E, 0xBB26,
0x2B90, 0x0063,
0x2B92, 0xBB36,
0x2B94, 0x0063,
0x2B96, 0xBAEA,
0x2B98, 0x0063,
0x2B9A, 0xBAF8,
0x2B9C, 0xBADA,
0x2B9E, 0x0004,
0x2BA0, 0x0063,
0x2BA2, 0xBAEA,
0x2BA4, 0x0063,
0x2BA6, 0xBB18,
0x2BA8, 0x0063,
0x2BAA, 0xBB26,
0x2BAC, 0x0063,
0x2BAE, 0xBB44,
0x2BB0, 0xBADA,
0x2BB2, 0x0004,
0x2BB4, 0x0063,
0x2BB6, 0xBACC,
0x2BB8, 0x0063,
0x2BBA, 0xBADC,
0x2BBC, 0x0063,
0x2BBE, 0xBAEA,
0x2BC0, 0x0063,
0x2BC2, 0xBAF8,
0x2BC4, 0xBADA,
0x2BC6, 0x0004,
0x2BC8, 0x0063,
0x2BCA, 0xBAEA,
0x2BCC, 0x0063,
0x2BCE, 0xBB18,
0x2BD0, 0x0063,
0x2BD2, 0xBACC,
0x2BD4, 0x0063,
0x2BD6, 0xBB0A,
0x2BD8, 0xBADA,
0x2BDA, 0x0004,
0x2BDC, 0x0063,
0x2BDE, 0xBACC,
0x2BE0, 0x0063,
0x2BE2, 0xBADC,
0x2BE4, 0x0063,
0x2BE6, 0xBAEA,
0x2BE8, 0x0063,
0x2BEA, 0xBB18,
0x2BEC, 0xBADA,
0x2BEE, 0x0004,
0x2BF0, 0xFFFF,
0x2BF2, 0xBB6E,
0x2BF4, 0x0000,
0x2BF6, 0x0000,
0x2BF8, 0x0000,
0x2BFA, 0x0000,
0x2BFC, 0x0000,
0x2BFE, 0x0000,
0x2C00, 0xBB72,
0x2C02, 0x0001,
0x2C04, 0x0063,
0x2C06, 0xBB52,
0x2C08, 0x0063,
0x2C0A, 0xBB60,
0x2C0C, 0x0000,
0x2C0E, 0x0000,
0x2C10, 0x0000,
0x2C12, 0x0000,
0x2C14, 0xBADA,
0x2C16, 0x0002,
0x2C18, 0x0066,
0x2C1A, 0x0067,
0x2C1C, 0x00AF,
0x2C1E, 0x01CF,
0x2C20, 0x0087,
0x2C22, 0x0083,
0x2C24, 0x011B,
0x2C26, 0x035A,
0x2C28, 0x00FA,
0x2C2A, 0x00F2,
0x2C2C, 0x00A6,
0x2C2E, 0x00A4,
0x2C30, 0xFFFF,
0x2C32, 0x002C,
0x2C34, 0x0058,
0x2C36, 0x0000,
0x2C38, 0x0000,
0x2C3A, 0xBC18,
0x2C3C, 0xBB74,
0x2C3E, 0xBB80,
0x2C40, 0xBC32,
0x2C42, 0xBB8C,
0x2C44, 0xBBA0,
0x2C46, 0xBB8C,
0x2C48, 0xBBA0,
0x2C4A, 0xBC04,
0x2C4C, 0xBC04,
0x2C4E, 0xBBF0,
0x2C50, 0xBBF0,
0x2C52, 0xBBB4,
0x2C54, 0xBBC8,
0x2C56, 0xBBB4,
0x2C58, 0xBBC8,
0x2C5A, 0xBC04,
0x2C5C, 0xBC04,
0x2C5E, 0xBBF0,
0x2C60, 0xBBF0,
0x2C62, 0xBB8C,
0x2C64, 0xBBA0,
0x2C66, 0xBB8C,
0x2C68, 0xBBA0,
0x2C6A, 0xBC04,
0x2C6C, 0xBC04,
0x2C6E, 0xBBF0,
0x2C70, 0xBBF0,
0x2C72, 0xBBB4,
0x2C74, 0xBBC8,
0x2C76, 0xBBB4,
0x2C78, 0xBBC8,
0x2C7A, 0xBC04,
0x2C7C, 0xBC04,
0x2C7E, 0xBBF0,
0x2C80, 0xBBF0,
0x3800, 0x880E,
0x3802, 0xBC62,
0x3804, 0xBC40,
0x3806, 0xD13E,
0x3808, 0xBC42,
0x380A, 0xBC3C,
0x380C, 0x0000,
0x380E, 0x0040,
0x3810, 0x0040,
0x3812, 0x0040,
0x3814, 0x0043,
0x3816, 0x0046,
0x3818, 0x004B,
0x381A, 0x004D,
0x381C, 0x0051,
0x381E, 0x0055,
0x3820, 0x005A,
0x3822, 0x005E,
0x3824, 0x0062,
0x3826, 0x0067,
0x3828, 0x006C,
0x382A, 0x0070,
0x382C, 0x0078,
0x382E, 0x0086,
0x3830, 0x0090,
0x3832, 0x0096,
0x3834, 0x009D,
0x3836, 0x00A5,
0x3838, 0x00AD,
0x383A, 0x00B4,
0x383C, 0x00B9,
0x383E, 0x00BE,
0x3840, 0x00C3,
0x3842, 0x00C8,
0x3844, 0x00CD,
0x3846, 0x00D2,
0x3848, 0x00D7,
0x384A, 0x00DC,
0x384C, 0x00DC,
0x384E, 0x0000,
0x3850, 0x0000,
0x3852, 0x0000,
0x3854, 0x0000,
0x3856, 0x0000,
0x3858, 0x0000,
0x385A, 0x0000,
0x385C, 0x0000,
0x385E, 0x0000,
0x3860, 0x0000,
0x3862, 0x0000,
0x3864, 0x0000,
0x3866, 0x0000,
0x3868, 0x0000,
0x386A, 0x0000,
0x386C, 0x0000,
0x386E, 0x0000,
0x3870, 0x0000,
0x3872, 0x0000,
0x3874, 0x0000,
0x3876, 0x0000,
0x3878, 0x0000,
0x387A, 0x0000,
0x387C, 0x0000,
0x387E, 0x0000,
0x3880, 0x0000,
0x3882, 0x0000,
0x3884, 0x0000,
0x3886, 0x0000,
0x3888, 0x0000,
0x388A, 0x0000,
0x388C, 0x0000,
0x026A, 0xFFFF,	
0x026C, 0x00FF,	
0x026E, 0x0000,	
0x0360, 0x1E8E,	
0x040E, 0x01EB,	
0x0600, 0x1130,	
0x0602, 0x3112,	
0x0604, 0x8048,	
0x0606, 0x00E9,	
0x067A, 0x0404,	
0x067C, 0x0404,	
0x06A8, 0x0240,	
0x06AA, 0x00CA,	
0x06AC, 0x0041,	
0x06B4, 0x3FFF,	
0x06DE, 0x0404,	
0x06E0, 0x0404,	
0x06E2, 0xFF00,	
0x06E4, 0x8333,	
0x06E6, 0x8333,	
0x06E8, 0x8333,	
0x06EA, 0x8333,	
0x052A, 0x0000,	
0x052C, 0x0000,	
0x0F06, 0x0002,	
0x0A04, 0xB4C5,	
0x0A06, 0xC400,	
0x0A08, 0x988A,	
0x0A0A, 0xA387,	
0x0A0E, 0xEEC0,	
0x0A12, 0x0000,	
0x0A18, 0x0010,	
0x0A1C, 0x0040,	
0x0A20, 0x0015,	
0x0C00, 0x0021,	
0x0C16, 0x0002,	
0x0708, 0x6FC0,	
0x070C, 0x0000,	
0x120C, 0x1428,	
0x121A, 0x0000,	
0x121C, 0x1896,	
0x121E, 0x0032,	
0x1220, 0x0000,	
0x1222, 0x96FF,	
0x1244, 0x0000,	
0x105C, 0x0F0B,	
0x1958, 0x0000,	
0x195A, 0x004C,	
0x195C, 0x0097,	
0x195E, 0x0221,	
0x1960, 0x03FE,	
0x1980, 0x00E0,	
0x1982, 0x0010,	
0x1984, 0x2018,	
0x1986, 0x0008,	
0x1988, 0x0000,	
0x198A, 0x0000,	
0x198C, 0x0880,	
0x198E, 0x0000,	
0x1990, 0x1A00,	
0x1992, 0x0000,	
0x1994, 0x2800,	
0x1996, 0x0002,	
0x1962, 0x0000,	
0x1964, 0x004C,	
0x1966, 0x0097,	
0x1968, 0x0221,	
0x196A, 0x03FE,	
0x19C0, 0x00E0,	
0x19C2, 0x0010,	
0x19C4, 0x2018,	
0x19C6, 0x0008,	
0x19C8, 0x0000,	
0x19CA, 0x0000,	
0x19CC, 0x0880,	
0x19CE, 0x0000,	
0x19D0, 0x1A00,	
0x19D2, 0x0000,	
0x19D4, 0x2800,	
0x19D6, 0x0002,	
0x196C, 0x0000,	
0x196E, 0x004C,	
0x1970, 0x0097,	
0x1972, 0x0221,	
0x1974, 0x03FE,	
0x1A00, 0x00E0,	
0x1A02, 0x0010,	
0x1A04, 0x2018,	
0x1A06, 0x0008,	
0x1A08, 0x0000,	
0x1A0A, 0x0000,	
0x1A0C, 0x0880,	
0x1A0E, 0x0000,	
0x1A10, 0x1A00,	
0x1A12, 0x0000,	
0x1A14, 0x2800,	
0x1A16, 0x0002,	
0x1976, 0x0000,	
0x1978, 0x004C,	
0x197A, 0x0097,	
0x197C, 0x0221,	
0x197E, 0x03FE,	
0x1A40, 0x00E0,	
0x1A42, 0x0010,	
0x1A44, 0x2018,	
0x1A46, 0x0008,	
0x1A48, 0x0000,	
0x1A4A, 0x0000,	
0x1A4C, 0x0880,	
0x1A4E, 0x0000,	
0x1A50, 0x1A00,	
0x1A52, 0x0000,	
0x1A54, 0x2800,	
0x1A56, 0x0002,	
0x192A, 0x0201,	
0x0384, 0x0001,	
0x027E, 0x0100,	
};

static void sensor_init(void)
{
	pr_debug("[%s] Hi-847 initial setting\n", __func__);
	hi847_table_write_cmos_sensor(hi847_init_setting, sizeof(hi847_init_setting)/sizeof(kal_uint16));
	pr_debug("[%s] Hi-847 initial setting end\n", __func__);

}	/*	  sensor_init  */


//3264X2448@30FPS_NO PD
static kal_uint16 hi847_preview_setting[] = {
//0x0B00, 0x0000,	
0x0204, 0x0000,	
0x0206, 0x0341,	
0x020A, 0x0B3D,	
0x020E, 0x0B41,	
0x0214, 0x0200,	
0x0216, 0x0200,	
0x0218, 0x0200,	
0x021A, 0x0200,	
0x0224, 0x002E,	
0x022A, 0x0017,	
0x022C, 0x0E1F,	
0x022E, 0x09C1,	
0x0234, 0x1111,	
0x0236, 0x1111,	
0x0238, 0x1111,	
0x023A, 0x1111,	
0x0250, 0x0000,	
0x0252, 0x0006,	
0x0254, 0x0000,	
0x0256, 0x0000,	
0x0258, 0x0000,	
0x025A, 0x0000,	
0x025C, 0x0000,	
0x025E, 0x0202,	
0x0268, 0x00CC,	
0x0440, 0x0020,	
0x0F00, 0x0000,	
0x0F04, 0x0008,	
0x0B02, 0x0100,	
0x0B04, 0x00DC,	
0x0B12, 0x0CC0,	
0x0B14, 0x0990,	
0x0B20, 0x0100,	
0x1100, 0x1100,	
0x1102, 0x0008,	
0x1108, 0x0202,	
0x1118, 0x0000,	
0x0A10, 0xB040,	
0x0C14, 0x0008,	
0x0C18, 0x0CC0,	
0x0C1A, 0x0990,	
0x0730, 0x0001,	
0x0732, 0x0000,	
0x0734, 0x0300,	
0x0736, 0x005A,	
0x0738, 0x0002,	
0x073C, 0x0900,	
0x0740, 0x0000,	
0x0742, 0x0000,	
0x0744, 0x0300,	
0x0746, 0x007D,	
0x0748, 0x0002,	
0x074A, 0x0900,	
0x074C, 0x0000,	
0x074E, 0x0100,	
0x0750, 0x0000,	
0x1200, 0x0946,	
0x1202, 0x1A00,	
0x120E, 0x6027,	
0x1210, 0x8027,	
0x1246, 0x0104,	
0x1000, 0x0300,	
0x1002, 0xC311,	
0x1004, 0x2BB0,	
0x1010, 0x0100,	
0x1012, 0x015E,	
0x1014, 0x0069,	
0x1016, 0x0069,	
0x101A, 0x0069,	
0x1020, 0xC108,	
0x1022, 0x0925,	
0x1024, 0x050A,	
0x1026, 0x0D0D,	
0x1028, 0x160A,	
0x102A, 0x0E0A,	
0x102C, 0x1800,	
0x1038, 0x1100,	
0x103E, 0x0001,	
0x1040, 0x0000,	
0x1042, 0x0108,	
0x1044, 0x00C8,	
0x1046, 0x0004,	
0x1048, 0x00C8,	
0x1066, 0x0100,	
0x1600, 0xE000,	
0x1608, 0x0028,	
0x160A, 0x0C80,	
0x160C, 0x001A,	
0x160E, 0x0960,	
0x0252, 0x0009,	
};

static void preview_setting(void)
{
	pr_debug("[%s] Hi-847 preview setting\n", __func__);

	hi847_table_write_cmos_sensor(hi847_preview_setting, sizeof(hi847_preview_setting)/sizeof(kal_uint16));

	pr_debug("[%s] Hi-847 preview setting end\n", __func__);

} /* preview_setting */

//3264X2448@30FPS_NO PD
static kal_uint16 hi847_capture_setting[] = {
//0x0B00, 0x0000,	
0x0204, 0x0000,	
0x0206, 0x0341,	
0x020A, 0x0B3D,	
0x020E, 0x0B41,	
0x0214, 0x0200,	
0x0216, 0x0200,	
0x0218, 0x0200,	
0x021A, 0x0200,	
0x0224, 0x002E,	
0x022A, 0x0017,	
0x022C, 0x0E1F,	
0x022E, 0x09C1,	
0x0234, 0x1111,	
0x0236, 0x1111,	
0x0238, 0x1111,	
0x023A, 0x1111,	
0x0250, 0x0000,	
0x0252, 0x0006,	
0x0254, 0x0000,	
0x0256, 0x0000,	
0x0258, 0x0000,	
0x025A, 0x0000,	
0x025C, 0x0000,	
0x025E, 0x0202,	
0x0268, 0x00CC,	
0x0440, 0x0020,	
0x0F00, 0x0000,	
0x0F04, 0x0008,	
0x0B02, 0x0100,	
0x0B04, 0x00DC,	
0x0B12, 0x0CC0,	
0x0B14, 0x0990,	
0x0B20, 0x0100,	
0x1100, 0x1100,	
0x1102, 0x0008,	
0x1108, 0x0202,	
0x1118, 0x0000,	
0x0A10, 0xB040,	
0x0C14, 0x0008,	
0x0C18, 0x0CC0,	
0x0C1A, 0x0990,	
0x0730, 0x0001,	
0x0732, 0x0000,	
0x0734, 0x0300,	
0x0736, 0x005A,	
0x0738, 0x0002,	
0x073C, 0x0900,	
0x0740, 0x0000,	
0x0742, 0x0000,	
0x0744, 0x0300,	
0x0746, 0x007D,	
0x0748, 0x0002,	
0x074A, 0x0900,	
0x074C, 0x0000,	
0x074E, 0x0100,	
0x0750, 0x0000,	
0x1200, 0x0946,	
0x1202, 0x1A00,	
0x120E, 0x6027,	
0x1210, 0x8027,	
0x1246, 0x0104,	
0x1000, 0x0300,	
0x1002, 0xC311,	
0x1004, 0x2BB0,	
0x1010, 0x0100,	
0x1012, 0x015E,	
0x1014, 0x0069,	
0x1016, 0x0069,	
0x101A, 0x0069,	
0x1020, 0xC108,	
0x1022, 0x0925,	
0x1024, 0x050A,	
0x1026, 0x0D0D,	
0x1028, 0x160A,	
0x102A, 0x0E0A,	
0x102C, 0x1800,	
0x1038, 0x1100,	
0x103E, 0x0001,	
0x1040, 0x0000,	
0x1042, 0x0108,	
0x1044, 0x00C8,	
0x1046, 0x0004,	
0x1048, 0x00C8,	
0x1066, 0x0100,	
0x1600, 0xE000,	
0x1608, 0x0028,	
0x160A, 0x0C80,	
0x160C, 0x001A,	
0x160E, 0x0960,	
0x0252, 0x0009,	

};
/*full size 30fps*/
static void capture_setting(kal_uint16 currefps)
{
	pr_debug("[%s] Hi-847 capture setting\n", __func__);
	
	hi847_table_write_cmos_sensor(hi847_capture_setting,
		sizeof(hi847_capture_setting)/sizeof(kal_uint16));

	pr_debug("[%s] Hi-847 capture setting end\n", __func__);
}


//3264X2448@30FPS_ PD type2 vc
static kal_uint16 hi847_normal_video_setting[] = {
//0x0B00, 0x0000,	
0x0204, 0x0000,	
0x0206, 0x0341,	
0x020A, 0x0B3D,	
0x020E, 0x0B41,	
0x0214, 0x0200,	
0x0216, 0x0200,	
0x0218, 0x0200,	
0x021A, 0x0200,	
0x0224, 0x002E,	
0x022A, 0x0017,	
0x022C, 0x0E1F,	
0x022E, 0x09C1,	
0x0234, 0x1111,	
0x0236, 0x1111,	
0x0238, 0x1111,	
0x023A, 0x1111,	
0x0250, 0x0000,	
0x0252, 0x0006,	
0x0254, 0x0000,	
0x0256, 0x0000,	
0x0258, 0x0000,	
0x025A, 0x0000,	
0x025C, 0x0000,	
0x025E, 0x0202,	
0x0268, 0x00CC,	
0x0440, 0x0020,	
0x0F00, 0x0000,	
0x0F04, 0x0008,	
0x0B02, 0x0100,	
0x0B04, 0x00DC,	
0x0B12, 0x0CC0,	
0x0B14, 0x0990,	
0x0B20, 0x0100,	
0x1100, 0x1100,	
0x1102, 0x0008,	
0x1108, 0x0202,	
0x1118, 0x0000,	
0x0A10, 0xB040,	
0x0C14, 0x0008,	
0x0C18, 0x0CC0,	
0x0C1A, 0x0990,	
0x0730, 0x0001,	
0x0732, 0x0000,	
0x0734, 0x0300,	
0x0736, 0x005A,	
0x0738, 0x0002,	
0x073C, 0x0900,	
0x0740, 0x0000,	
0x0742, 0x0000,	
0x0744, 0x0300,	
0x0746, 0x007D,	
0x0748, 0x0002,	
0x074A, 0x0900,	
0x074C, 0x0000,	
0x074E, 0x0100,	
0x0750, 0x0000,	
0x1200, 0x0946,	
0x1202, 0x1A00,	
0x120E, 0x6027,	
0x1210, 0x8027,	
0x1246, 0x0104,	
0x1000, 0x0300,	
0x1002, 0xC311,	
0x1004, 0x2BB0,	
0x1010, 0x0100,	
0x1012, 0x015E,	
0x1014, 0x0069,	
0x1016, 0x0069,	
0x101A, 0x0069,	
0x1020, 0xC108,	
0x1022, 0x0925,	
0x1024, 0x050A,	
0x1026, 0x0D0D,	
0x1028, 0x160A,	
0x102A, 0x0E0A,	
0x102C, 0x1800,	
0x1038, 0x1100,	
0x103E, 0x0001,	
0x1040, 0x0000,	
0x1042, 0x0108,	
0x1044, 0x00C8,	
0x1046, 0x0004,	
0x1048, 0x00C8,	
0x1066, 0x0100,	
0x1600, 0xE000,	
0x1608, 0x0028,	
0x160A, 0x0C80,	
0x160C, 0x001A,	
0x160E, 0x0960,	
0x0252, 0x0009,	
};

static void normal_video_setting(kal_uint16 currefps)
{
	pr_debug("[%s] Hi-847 normal_video setting\n", __func__);

	hi847_table_write_cmos_sensor(hi847_normal_video_setting,
		sizeof(hi847_normal_video_setting)/sizeof(kal_uint16));

	pr_debug("[%s] Hi-847 normal_video setting end\n", __func__);
}


//720p@90fps
static kal_uint16 hi847_hs_video_setting[] = {
//0x0B00, 0x0000,	
0x0204, 0x0000,	
0x0206, 0x033E,	
0x020A, 0x03BF,	
0x020E, 0x03C3,	
0x0214, 0x0200,	
0x0216, 0x0200,	
0x0218, 0x0200,	
0x021A, 0x0200,	
0x0224, 0x0224,	
0x022A, 0x0017,	
0x022C, 0x0E2D,	
0x022E, 0x07C9,	
0x0234, 0x1111,	
0x0236, 0x3311,	
0x0238, 0x3311,	
0x023A, 0x1122,	
0x0250, 0x0000,	
0x0252, 0x0006,	
0x0254, 0x0000,	
0x0256, 0x0000,	
0x0258, 0x0000,	
0x025A, 0x0000,	
0x025C, 0x0000,	
0x025E, 0x0202,	
0x0268, 0x00CD,	
0x0440, 0x0028,	
0x0F00, 0x0400,	
0x0F04, 0x00B4,	
0x0B02, 0x0100,	
0x0B04, 0x00FC,	
0x0B12, 0x0500,	
0x0B14, 0x02D0,	
0x0B20, 0x0200,	
0x1100, 0x1100,	
0x1102, 0x0008,	
0x1108, 0x0002,	
0x1118, 0x040C,	
0x0A10, 0xB040,	
0x0C14, 0x0168,	
0x0C18, 0x0A00,	
0x0C1A, 0x02D0,	
0x0730, 0x0001,	
0x0732, 0x0000,	
0x0734, 0x0300,	
0x0736, 0x005A,	
0x0738, 0x0002,	
0x073C, 0x0900,	
0x0740, 0x0000,	
0x0742, 0x0000,	
0x0744, 0x0300,	
0x0746, 0x007D,	
0x0748, 0x0002,	
0x074A, 0x0900,	
0x074C, 0x0100,	
0x074E, 0x0100,	
0x0750, 0x0000,	
0x1200, 0x0946,	
0x1202, 0x1A00,	
0x120E, 0x6027,	
0x1210, 0x8027,	
0x1246, 0x0105,	
0x1000, 0x0300,	
0x1002, 0xC311,	
0x1004, 0x2BB0,	
0x1010, 0x0543,	
0x1012, 0x010D,	
0x1014, 0x0020,	
0x1016, 0x0020,	
0x101A, 0x0020,	
0x1020, 0xC105,	
0x1022, 0x0412,	
0x1024, 0x0305,	
0x1026, 0x0708,	
0x1028, 0x1206,	
0x102A, 0x0705,	
0x102C, 0x0E00,	
0x1038, 0x0000,	
0x103E, 0x0101,	
0x1040, 0x0000,	
0x1042, 0x0008,	
0x1044, 0x0120,	
0x1046, 0x01B0,	
0x1048, 0x0090,	
0x1066, 0x0557,	
0x1600, 0x0400,	
0x1608, 0x0028,	
0x160A, 0x0C80,	
0x160C, 0x001A,	
0x160E, 0x0960,	
0x0252, 0x0009,	
};

static void hs_video_setting(void)
{
	pr_debug("[%s] Hi-847 hs_video setting\n", __func__);

	hi847_table_write_cmos_sensor(hi847_hs_video_setting,
		sizeof(hi847_hs_video_setting)/sizeof(kal_uint16));

	pr_debug("[%s] Hi-847 hs_video setting end\n", __func__);
}


//1080p@60fps
static kal_uint16 hi847_slim_video_setting[] = {
//0x0B00, 0x0000,	
0x0204, 0x0000,	
0x0206, 0x033F,	
0x020A, 0x05A0,	
0x020E, 0x05A4,	
0x0214, 0x0200,	
0x0216, 0x0200,	
0x0218, 0x0200,	
0x021A, 0x0200,	
0x0224, 0x02DA,	
0x022A, 0x0017,	
0x022C, 0x0E1F,	
0x022E, 0x0715,	
0x0234, 0x1111,	
0x0236, 0x1111,	
0x0238, 0x1111,	
0x023A, 0x1111,	
0x0250, 0x0000,	
0x0252, 0x0006,	
0x0254, 0x0000,	
0x0256, 0x0000,	
0x0258, 0x0000,	
0x025A, 0x0000,	
0x025C, 0x0000,	
0x025E, 0x0202,	
0x0268, 0x00CD,	
0x0440, 0x003F,	
0x0F00, 0x0000,	
0x0F04, 0x02A8,	
0x0B02, 0x0100,	
0x0B04, 0x00DC,	
0x0B12, 0x0780,	
0x0B14, 0x0438,	
0x0B20, 0x0100,	
0x1100, 0x1100,	
0x1102, 0x0008,	
0x1108, 0x0002,	
0x1118, 0x04C2,	
0x0A10, 0xB040,	
0x0C14, 0x02A8,	
0x0C18, 0x0780,	
0x0C1A, 0x0438,	
0x0730, 0x0001,	
0x0732, 0x0000,	
0x0734, 0x0300,	
0x0736, 0x005A,	
0x0738, 0x0002,	
0x073C, 0x0900,	
0x0740, 0x0000,	
0x0742, 0x0000,	
0x0744, 0x0300,	
0x0746, 0x007D,	
0x0748, 0x0002,	
0x074A, 0x0900,	
0x074C, 0x0000,	
0x074E, 0x0100,	
0x0750, 0x0000,	
0x1200, 0x0946,	
0x1202, 0x1A00,	
0x120E, 0x6027,	
0x1210, 0x8027,	
0x1246, 0x0104,	
0x1000, 0x0300,	
0x1002, 0xC311,	
0x1004, 0x2BB0,	
0x1010, 0x0AA6,	
0x1012, 0x02FE,	
0x1014, 0x0020,	
0x1016, 0x0020,	
0x101A, 0x0020,	
0x1020, 0xC108,	
0x1022, 0x0925,	
0x1024, 0x050A,	
0x1026, 0x0D0D,	
0x1028, 0x160A,	
0x102A, 0x0E0A,	
0x102C, 0x1800,	
0x1038, 0x0000,	
0x103E, 0x0001,	
0x1040, 0x0000,	
0x1042, 0x0008,	
0x1044, 0x0120,	
0x1046, 0x01B0,	
0x1048, 0x0090,	
0x1066, 0x0ACF,	
0x1600, 0x0000,	
0x1608, 0x0028,	
0x160A, 0x0C80,	
0x160C, 0x001A,	
0x160E, 0x0960,	
0x0252, 0x0009,	
};

static void slim_video_setting(void)
{
	pr_debug("[%s] Hi-847 slim_video setting\n", __func__);

	hi847_table_write_cmos_sensor(hi847_slim_video_setting,
		sizeof(hi847_slim_video_setting)/sizeof(kal_uint16));

	pr_debug("[%s] Hi-847 slim_video setting end\n", __func__);
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
 
  //prize add by lipengpeng 20200722 start
 #define OV50C40_MODULE_ID_OFFSET  0x0001
static kal_uint16 read_module_id_hi847(void)
{
	kal_uint16 get_byte = 0;
	char pusendcmd[2] = {(char)(OV50C40_MODULE_ID_OFFSET >> 8), (char)(OV50C40_MODULE_ID_OFFSET & 0xFF)};

	iReadRegI2C(pusendcmd, 2, (u8 *)&get_byte, 1, 0xA0);
	pr_err("the module id is %d\n", get_byte);
	return get_byte;
}
//prize add by lipengpeng 20200722 end

static kal_uint32 get_imgsensor_id(UINT32 *sensor_id)
{
	kal_uint8 i = 0;
	kal_uint8 retry = 2;
	/*sensor have two i2c address 0x34 & 0x20,
	 *we should detect the module used i2c address
	 */
	 
	printk("lpp---hi847,read_module_id_hi847=%x",read_module_id_hi847());	 
	
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			*sensor_id = return_sensor_id();
			if (*sensor_id == imgsensor_info.sensor_id) {
				pr_debug("i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id, *sensor_id);
				return ERROR_NONE;
			}

			pr_debug("Read sensor id fail,read:0x%x id: 0x%x\n", return_sensor_id(), imgsensor.i2c_write_id);
			retry--;
		} while (retry > 0);
		i++;
		retry = 2;
	}
	if (*sensor_id != imgsensor_info.sensor_id) {
		/*if Sensor ID is not correct,
		 *Must set *sensor_id to 0xFFFFFFFF
		 */
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
	kal_uint8 i = 0;
	kal_uint8 retry = 2;
	kal_uint16 sensor_id = 0;

	pr_debug("%s +\n", __func__);
	/*sensor have two i2c address 0x6c 0x6d & 0x21 0x20,
	 *we should detect the module used i2c address
	 */
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			sensor_id = return_sensor_id();
			if (sensor_id == imgsensor_info.sensor_id) {
				pr_debug("i2c write id: 0x%x, sensor id: 0x%x\n",
					imgsensor.i2c_write_id, sensor_id);
				break;
			}
			pr_debug("Read sensor id fail, id: 0x%x\n",
				imgsensor.i2c_write_id);
			retry--;
		} while (retry > 0);
		i++;
		if (sensor_id == imgsensor_info.sensor_id)
			break;
		retry = 2;
	}
	if (imgsensor_info.sensor_id != sensor_id)
		return ERROR_SENSOR_CONNECT_FAIL;

	/* initail sequence write in  */

	pr_debug("%s HI-847 initial setting start \n", __func__);
	sensor_init();
	pr_debug("%s HI-847 initial setting end \n", __func__);

	spin_lock(&imgsensor_drv_lock);

	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	spin_unlock(&imgsensor_drv_lock);
	pr_debug("%s -\n", __func__);

	return ERROR_NONE;
} /* open */

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
	pr_debug("E\n");
	/* No Need to implement this function */
	streaming_control(KAL_FALSE);
	return ERROR_NONE;
} /* close */


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
	pr_debug("%s E\n", __func__);

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_PREVIEW;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.frame_length = imgsensor_info.pre.framelength;
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);

	pr_debug("%s HI-847 preview setting start \n", __func__);
	preview_setting();
	pr_debug("%s HI-847 preview setting end \n", __func__);


	return ERROR_NONE;
} /* preview */

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
	pr_debug("E\n");
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CAPTURE;

	if (imgsensor.current_fps != imgsensor_info.cap.max_framerate)
		pr_debug(
			"Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n",
			imgsensor.current_fps,
			imgsensor_info.cap.max_framerate / 10);
	imgsensor.pclk = imgsensor_info.cap.pclk;
	imgsensor.line_length = imgsensor_info.cap.linelength;
	imgsensor.frame_length = imgsensor_info.cap.framelength;
	imgsensor.min_frame_length = imgsensor_info.cap.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;

	spin_unlock(&imgsensor_drv_lock);

	pr_debug("%s HI-847 capture setting start \n", __func__);
	capture_setting(imgsensor.current_fps);
	pr_debug("%s HI-847 capture setting end \n", __func__);

	return ERROR_NONE;
}	/* capture() */
static kal_uint32 normal_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
				MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	pr_debug("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_VIDEO;
	imgsensor.pclk = imgsensor_info.normal_video.pclk;
	imgsensor.line_length = imgsensor_info.normal_video.linelength;
	imgsensor.frame_length = imgsensor_info.normal_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.normal_video.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);

	pr_debug("%s HI-847 normal_video setting start \n", __func__);
	normal_video_setting(imgsensor.current_fps);
	pr_debug("%s HI-847 normal_video setting emd \n", __func__);

	return ERROR_NONE;
}	/*	normal_video   */

static kal_uint32 hs_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
				MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	pr_debug("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_HIGH_SPEED_VIDEO;
	imgsensor.pclk = imgsensor_info.hs_video.pclk;
	/*imgsensor.video_mode = KAL_TRUE;*/
	imgsensor.line_length = imgsensor_info.hs_video.linelength;
	imgsensor.frame_length = imgsensor_info.hs_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.hs_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	/*imgsensor.current_fps = 300;*/
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);

	pr_debug("%s HI-847 hs_video setting start \n", __func__);
	hs_video_setting();
	pr_debug("%s HI-847 hs_video setting end \n", __func__);

	return ERROR_NONE;
}	/*	hs_video   */

static kal_uint32 slim_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
				MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	pr_debug("%s. 720P@240FPS\n", __func__);

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_SLIM_VIDEO;
	imgsensor.pclk = imgsensor_info.slim_video.pclk;
	/*imgsensor.video_mode = KAL_TRUE;*/
	imgsensor.line_length = imgsensor_info.slim_video.linelength;
	imgsensor.frame_length = imgsensor_info.slim_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.slim_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	/*imgsensor.current_fps = 300;*/
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	
	pr_debug("%s HI-847 slim_video setting start \n", __func__);
	slim_video_setting();
	pr_debug("%s HI-847 slim_video setting end \n", __func__);

	return ERROR_NONE;
}	/* slim_video */


static kal_uint32
get_resolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *sensor_resolution)
{
	pr_debug("E\n");
	sensor_resolution->SensorFullWidth =
		imgsensor_info.cap.grabwindow_width;
	sensor_resolution->SensorFullHeight =
		imgsensor_info.cap.grabwindow_height;

	sensor_resolution->SensorPreviewWidth =
		imgsensor_info.pre.grabwindow_width;
	sensor_resolution->SensorPreviewHeight =
		imgsensor_info.pre.grabwindow_height;

	sensor_resolution->SensorVideoWidth =
		imgsensor_info.normal_video.grabwindow_width;
	sensor_resolution->SensorVideoHeight =
		imgsensor_info.normal_video.grabwindow_height;

	sensor_resolution->SensorHighSpeedVideoWidth =
		imgsensor_info.hs_video.grabwindow_width;
	sensor_resolution->SensorHighSpeedVideoHeight =
		imgsensor_info.hs_video.grabwindow_height;

	sensor_resolution->SensorSlimVideoWidth =
		imgsensor_info.slim_video.grabwindow_width;
	sensor_resolution->SensorSlimVideoHeight =
		imgsensor_info.slim_video.grabwindow_height;

	sensor_resolution->SensorCustom1Width =
		imgsensor_info.custom1.grabwindow_width;
	sensor_resolution->SensorCustom1Height =
		imgsensor_info.custom1.grabwindow_height;

	sensor_resolution->SensorCustom2Width =
		imgsensor_info.custom2.grabwindow_width;
	sensor_resolution->SensorCustom2Height =
		imgsensor_info.custom2.grabwindow_height;

	sensor_resolution->SensorCustom3Width =
		imgsensor_info.custom3.grabwindow_width;
	sensor_resolution->SensorCustom3Height =
		imgsensor_info.custom3.grabwindow_height;

	sensor_resolution->SensorCustom4Width =
		imgsensor_info.custom4.grabwindow_width;
	sensor_resolution->SensorCustom4Height =
		imgsensor_info.custom4.grabwindow_height;

	return ERROR_NONE;
} /* get_resolution */

static kal_uint32 get_info(enum MSDK_SCENARIO_ID_ENUM scenario_id,
			   MSDK_SENSOR_INFO_STRUCT *sensor_info,
			   MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	pr_debug("scenario_id = %d\n", scenario_id);

	sensor_info->SensorClockPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorClockFallingPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorInterruptDelayLines = 4; /* not use */
	sensor_info->SensorResetActiveHigh = FALSE; /* not use */
	sensor_info->SensorResetDelayCount = 5; /* not use */

	sensor_info->SensroInterfaceType = imgsensor_info.sensor_interface_type;
	sensor_info->MIPIsensorType = imgsensor_info.mipi_sensor_type;
	sensor_info->SettleDelayMode = imgsensor_info.mipi_settle_delay_mode;
	sensor_info->SensorOutputDataFormat =
		imgsensor_info.sensor_output_dataformat;

	sensor_info->CaptureDelayFrame = imgsensor_info.cap_delay_frame;
	sensor_info->PreviewDelayFrame = imgsensor_info.pre_delay_frame;
	sensor_info->VideoDelayFrame = imgsensor_info.video_delay_frame;
	sensor_info->HighSpeedVideoDelayFrame =
		imgsensor_info.hs_video_delay_frame;
	sensor_info->SlimVideoDelayFrame =
		imgsensor_info.slim_video_delay_frame;
	sensor_info->Custom1DelayFrame = imgsensor_info.custom1_delay_frame;
	sensor_info->Custom2DelayFrame = imgsensor_info.custom2_delay_frame;
	sensor_info->Custom3DelayFrame = imgsensor_info.custom3_delay_frame;
	sensor_info->Custom4DelayFrame = imgsensor_info.custom4_delay_frame;

	sensor_info->SensorMasterClockSwitch = 0; /* not use */
	sensor_info->SensorDrivingCurrent = imgsensor_info.isp_driving_current;

	sensor_info->AEShutDelayFrame = imgsensor_info.ae_shut_delay_frame;
	sensor_info->AESensorGainDelayFrame =
		imgsensor_info.ae_sensor_gain_delay_frame;
	sensor_info->AEISPGainDelayFrame =
		imgsensor_info.ae_ispGain_delay_frame;
	sensor_info->IHDR_Support = imgsensor_info.ihdr_support;
	sensor_info->IHDR_LE_FirstLine = imgsensor_info.ihdr_le_firstline;
	sensor_info->SensorModeNum = imgsensor_info.sensor_mode_num;
#if ENABLE_PDAF 
	sensor_info->PDAF_Support = PDAF_SUPPORT_CAMSV; //PDAF_TYPE2
#else
	sensor_info->PDAF_Support = 0;
#endif
	sensor_info->SensorMIPILaneNumber = imgsensor_info.mipi_lane_num;
	sensor_info->TEMPERATURE_SUPPORT = imgsensor_info.temperature_support;
	sensor_info->SensorClockFreq = imgsensor_info.mclk;
	sensor_info->SensorClockDividCount = 3; /* not use */
	sensor_info->SensorClockRisingCount = 0;
	sensor_info->SensorClockFallingCount = 2; /* not use */
	sensor_info->SensorPixelClockCount = 3; /* not use */
	sensor_info->SensorDataLatchCount = 2; /* not use */

	sensor_info->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->SensorWidthSampling = 0; /* 0 is default 1x */
	sensor_info->SensorHightSampling = 0; /* 0 is default 1x */
	sensor_info->SensorPacketECCOrder = 1;

	sensor_info->FrameTimeDelayFrame =
		imgsensor_info.frame_time_delay_frame;

	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.pre.mipi_data_lp2hs_settle_dc;

		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		sensor_info->SensorGrabStartX = imgsensor_info.cap.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.cap.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.cap.mipi_data_lp2hs_settle_dc;

		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:

		sensor_info->SensorGrabStartX =
			imgsensor_info.normal_video.startx;
		sensor_info->SensorGrabStartY =
			imgsensor_info.normal_video.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.normal_video.mipi_data_lp2hs_settle_dc;

		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		sensor_info->SensorGrabStartX = imgsensor_info.hs_video.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.hs_video.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.hs_video.mipi_data_lp2hs_settle_dc;

		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		sensor_info->SensorGrabStartX =
			imgsensor_info.slim_video.startx;
		sensor_info->SensorGrabStartY =
			imgsensor_info.slim_video.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.slim_video.mipi_data_lp2hs_settle_dc;

		break;

	default:
		sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.pre.mipi_data_lp2hs_settle_dc;
		break;
	}

	return ERROR_NONE;
}	/*	get_info  */


static kal_uint32 control(enum MSDK_SCENARIO_ID_ENUM scenario_id,
			MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	pr_debug("scenario_id = %d\n", scenario_id);
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
		pr_debug("Error ScenarioId setting");
		preview(image_window, sensor_config_data);
		return ERROR_INVALID_SCENARIO_ID;
	}

	return ERROR_NONE;
}	/* control() */



static kal_uint32 set_video_mode(UINT16 framerate)
{
	pr_debug("framerate = %d\n ", framerate);
	/* SetVideoMode Function should fix framerate */
	if (framerate == 0)
		/* Dynamic frame rate */
		return ERROR_NONE;
	spin_lock(&imgsensor_drv_lock);
	if ((framerate == 300) && (imgsensor.autoflicker_en == KAL_TRUE))
		imgsensor.current_fps = 296;
	else if ((framerate == 150) && (imgsensor.autoflicker_en == KAL_TRUE))
		imgsensor.current_fps = 146;
	else
		imgsensor.current_fps = framerate;
	spin_unlock(&imgsensor_drv_lock);
	set_max_framerate(imgsensor.current_fps, 1);

	return ERROR_NONE;
}

static kal_uint32 set_auto_flicker_mode(kal_bool enable, UINT16 framerate)
{
	spin_lock(&imgsensor_drv_lock);
	if (enable) /*enable auto flicker*/ {
		imgsensor.autoflicker_en = KAL_TRUE;
		pr_debug("enable! fps = %d", framerate);
	} else {
		 /*Cancel Auto flick*/
		imgsensor.autoflicker_en = KAL_FALSE;
	}
	spin_unlock(&imgsensor_drv_lock);

	return ERROR_NONE;
}


static kal_uint32 set_max_framerate_by_scenario(
		enum MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 framerate)
{
	kal_uint32 frame_length;

	pr_debug("scenario_id = %d, framerate = %d\n", scenario_id, framerate);

	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		frame_length = imgsensor_info.pre.pclk / framerate * 10
				/ imgsensor_info.pre.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line =
			(frame_length > imgsensor_info.pre.framelength)
		? (frame_length - imgsensor_info.pre.framelength) : 0;
		imgsensor.frame_length =
			imgsensor_info.pre.framelength
			+ imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		if (framerate == 0)
			return ERROR_NONE;
		frame_length = imgsensor_info.normal_video.pclk /
				framerate * 10 /
				imgsensor_info.normal_video.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line =
			(frame_length > imgsensor_info.normal_video.framelength)
		? (frame_length - imgsensor_info.normal_video.framelength)
		: 0;
		imgsensor.frame_length =
			imgsensor_info.normal_video.framelength
			+ imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
	if (imgsensor.current_fps != imgsensor_info.cap.max_framerate)
		pr_debug(
			"Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n"
			, framerate, imgsensor_info.cap.max_framerate/10);
		frame_length = imgsensor_info.cap.pclk / framerate * 10
				/ imgsensor_info.cap.linelength;
		spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line =
			(frame_length > imgsensor_info.cap.framelength)
			  ? (frame_length - imgsensor_info.cap.framelength) : 0;
			imgsensor.frame_length =
				imgsensor_info.cap.framelength
				+ imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);

		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		frame_length = imgsensor_info.hs_video.pclk / framerate * 10
				/ imgsensor_info.hs_video.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line =
			(frame_length > imgsensor_info.hs_video.framelength)
			  ? (frame_length - imgsensor_info.hs_video.framelength)
			  : 0;
		imgsensor.frame_length =
			imgsensor_info.hs_video.framelength
				+ imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		frame_length = imgsensor_info.slim_video.pclk / framerate * 10
			/ imgsensor_info.slim_video.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line =
			(frame_length > imgsensor_info.slim_video.framelength)
			? (frame_length - imgsensor_info.slim_video.framelength)
			: 0;
		imgsensor.frame_length =
			imgsensor_info.slim_video.framelength
			+ imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	default:  /*coding with  preview scenario by default*/
		frame_length = imgsensor_info.pre.pclk / framerate * 10
			/ imgsensor_info.pre.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line =
			(frame_length > imgsensor_info.pre.framelength)
			? (frame_length - imgsensor_info.pre.framelength) : 0;
		imgsensor.frame_length =
			imgsensor_info.pre.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		pr_debug("error scenario_id = %d, we use preview scenario\n",
			scenario_id);
		break;
	}
	return ERROR_NONE;
}


static kal_uint32 get_default_framerate_by_scenario(
		enum MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 *framerate)
{
	pr_debug("scenario_id = %d\n", scenario_id);

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

static kal_uint32 set_test_pattern_mode(kal_bool enable)
{
	pr_debug("set_test_pattern_mode enable: %d", enable);

	if (enable) {
		write_cmos_sensor(0x0b04, 0x00D9);
		write_cmos_sensor(0x0C0A, 0x0204);
	} else {
		write_cmos_sensor(0x0b04, 0x00DE);
		write_cmos_sensor(0x0C0A, 0x0000);
	}
	spin_lock(&imgsensor_drv_lock);
	imgsensor.test_pattern = enable;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}

static kal_uint32 feature_control(MSDK_SENSOR_FEATURE_ENUM feature_id,
				 UINT8 *feature_para, UINT32 *feature_para_len)
{
	UINT16 *feature_return_para_16 = (UINT16 *) feature_para;
	UINT16 *feature_data_16 = (UINT16 *) feature_para;
	UINT32 *feature_return_para_32 = (UINT32 *) feature_para;
	UINT32 *feature_data_32 = (UINT32 *) feature_para;
	unsigned long long *feature_data = (unsigned long long *) feature_para;
	/* unsigned long long *feature_return_para
	 *  = (unsigned long long *) feature_para;
	 */
	struct SENSOR_WINSIZE_INFO_STRUCT *wininfo;
#if ENABLE_PDAF
	struct SET_PD_BLOCK_INFO_T *PDAFinfo;
	struct SENSOR_VC_INFO_STRUCT *pvcinfo;
#endif
	/* SET_SENSOR_AWB_GAIN *pSetSensorAWB
	 *  = (SET_SENSOR_AWB_GAIN *)feature_para;
	 */
	MSDK_SENSOR_REG_INFO_STRUCT *sensor_reg_data
		= (MSDK_SENSOR_REG_INFO_STRUCT *) feature_para;

	/*pr_debug("feature_id = %d\n", feature_id);*/
	switch (feature_id) {
	case SENSOR_FEATURE_GET_GAIN_RANGE_BY_SCENARIO:
		*(feature_data + 1) = imgsensor_info.min_gain;
		*(feature_data + 2) = imgsensor_info.max_gain;
		break;
	case SENSOR_FEATURE_GET_BASE_GAIN_ISO_AND_STEP:
		*(feature_data + 0) = imgsensor_info.min_gain_iso;
		*(feature_data + 1) = imgsensor_info.gain_step;
		*(feature_data + 2) = imgsensor_info.gain_type;
		break;
	case SENSOR_FEATURE_GET_MIN_SHUTTER_BY_SCENARIO:
		*(feature_data + 1) = imgsensor_info.min_shutter;
		*(feature_data + 2) = imgsensor_info.exp_step;
		break;
	case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ_BY_SCENARIO:
		switch (*feature_data) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.cap.pclk;
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.normal_video.pclk;
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.hs_video.pclk;
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.slim_video.pclk;
			break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.pre.pclk;
			break;
		}
		break;
	case SENSOR_FEATURE_GET_PERIOD_BY_SCENARIO:
		switch (*feature_data) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.cap.framelength << 16)
				+ imgsensor_info.cap.linelength;
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.normal_video.framelength << 16)
				+ imgsensor_info.normal_video.linelength;
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.hs_video.framelength << 16)
				+ imgsensor_info.hs_video.linelength;
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.slim_video.framelength << 16)
				+ imgsensor_info.slim_video.linelength;
			break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.pre.framelength << 16)
				+ imgsensor_info.pre.linelength;
			break;
		}
		break;

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
		 set_shutter(*feature_data);
		break;
	case SENSOR_FEATURE_SET_NIGHTMODE:
		 /* night_mode((BOOL) *feature_data); */
		break;
	#ifdef VENDOR_EDIT
	case SENSOR_FEATURE_CHECK_MODULE_ID:
		*feature_return_para_32 = imgsensor_info.module_id;
		break;
	#endif
	case SENSOR_FEATURE_SET_GAIN:
		set_gain((UINT16) *feature_data);
		break;
	case SENSOR_FEATURE_SET_FLASHLIGHT:
		break;
	case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
		break;
	case SENSOR_FEATURE_SET_REGISTER:
		write_cmos_sensor_8(sensor_reg_data->RegAddr,
				    sensor_reg_data->RegData);
		break;
	case SENSOR_FEATURE_GET_REGISTER:
		sensor_reg_data->RegData =
			read_cmos_sensor_8(sensor_reg_data->RegAddr);
		break;
	case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
		/*get the lens driver ID from EEPROM
		 * or just return LENS_DRIVER_ID_DO_NOT_CARE
		 * if EEPROM does not exist in camera module.
		 */
		*feature_return_para_32 = LENS_DRIVER_ID_DO_NOT_CARE;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_SET_VIDEO_MODE:
		set_video_mode(*feature_data);
		break;
	case SENSOR_FEATURE_CHECK_SENSOR_ID:
		get_imgsensor_id(feature_return_para_32);
		break;
	case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE:
		set_auto_flicker_mode((BOOL)*feature_data_16,
				      *(feature_data_16+1));
		break;
	case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
		 set_max_framerate_by_scenario(
				(enum MSDK_SCENARIO_ID_ENUM)*feature_data,
				*(feature_data+1));
		break;
	case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
		 get_default_framerate_by_scenario(
				(enum MSDK_SCENARIO_ID_ENUM)*(feature_data),
				(MUINT32 *)(uintptr_t)(*(feature_data+1)));
		break;
	case SENSOR_FEATURE_GET_PDAF_DATA:
				printk("SENSOR_FEATURE_GET_PDAF_DATA success\n");
				break;

	case SENSOR_FEATURE_SET_TEST_PATTERN:
		set_test_pattern_mode((BOOL)*feature_data);
		break;
	case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE:
		/* for factory mode auto testing */
		*feature_return_para_32 = imgsensor_info.checksum_value;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_SET_FRAMERATE:
		pr_debug("current fps :%d\n", (UINT32)*feature_data_32);
		spin_lock(&imgsensor_drv_lock);
		imgsensor.current_fps = *feature_data_32;
		spin_unlock(&imgsensor_drv_lock);
		break;
	case SENSOR_FEATURE_SET_HDR:
		pr_debug("ihdr enable :%d\n", (BOOL)*feature_data_32);
		spin_lock(&imgsensor_drv_lock);
		imgsensor.ihdr_mode = *feature_data_32;
		spin_unlock(&imgsensor_drv_lock);
		break;
	case SENSOR_FEATURE_GET_CROP_INFO:
	#if 0
		pr_debug("SENSOR_FEATURE_GET_CROP_INFO scenarioId:%d\n",
			(UINT32)*feature_data);
	#endif
		wininfo =
	(struct SENSOR_WINSIZE_INFO_STRUCT *)(uintptr_t)(*(feature_data+1));

		switch (*feature_data_32) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			memcpy((void *)wininfo,
				(void *)&imgsensor_winsize_info[1],
				sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			memcpy((void *)wininfo,
				(void *)&imgsensor_winsize_info[2],
				sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			memcpy((void *)wininfo,
			(void *)&imgsensor_winsize_info[3],
			sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			memcpy((void *)wininfo,
			(void *)&imgsensor_winsize_info[4],
			sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			memcpy((void *)wininfo,
			(void *)&imgsensor_winsize_info[0],
			sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		}
		break;
	#if ENABLE_PDAF	
	case SENSOR_FEATURE_GET_PDAF_INFO:
		pr_debug("SENSOR_FEATURE_GET_PDAF_INFO scenarioId:%d\n",
			(UINT16) *feature_data);
		PDAFinfo =
		  (struct SET_PD_BLOCK_INFO_T *)(uintptr_t)(*(feature_data+1));
		switch (*feature_data)
		{
				case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
					memcpy((void *)PDAFinfo,(void *)&imgsensor_pd_info,sizeof(struct SET_PD_BLOCK_INFO_T));
					break;
				case MSDK_SCENARIO_ID_VIDEO_PREVIEW: 
					memcpy((void *)PDAFinfo,(void *)&imgsensor_pd_info,sizeof(struct SET_PD_BLOCK_INFO_T));
					break;
				case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
					memcpy((void *)PDAFinfo,(void *)&imgsensor_pd_info,sizeof(struct SET_PD_BLOCK_INFO_T));
					break;
				case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
				case MSDK_SCENARIO_ID_SLIM_VIDEO:
		default:
			break;
		}
		break;


	case SENSOR_FEATURE_GET_VC_INFO:
			pvcinfo = (struct SENSOR_VC_INFO_STRUCT *)(uintptr_t)(*(feature_data+1));	 
			switch (*feature_data_32)
			{
					case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
						memcpy((void *)pvcinfo,(void *)&SENSOR_VC_INFO[0],sizeof(struct SENSOR_VC_INFO_STRUCT));
						break;
					case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
						memcpy((void *)pvcinfo,(void *)&SENSOR_VC_INFO[1],sizeof(struct SENSOR_VC_INFO_STRUCT));
						break;
					case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
						memcpy((void *)pvcinfo,(void *)&SENSOR_VC_INFO[0],sizeof(struct SENSOR_VC_INFO_STRUCT));
						break;
					case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
					case MSDK_SCENARIO_ID_SLIM_VIDEO:
					default:
						pr_info("error: get wrong vc_INFO id = %d",
						*feature_data_32);
						//memcpy((void *)pvcinfo,(void *)&SENSOR_VC_INFO[0],sizeof(struct SENSOR_VC_INFO_STRUCT));
						break;
			}
		break;

	case SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY:
		pr_debug(
		"SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY scenarioId:%d\n",
			(UINT16) *feature_data);
		/*PDAF capacity enable or not, 2p8 only full size support PDAF*/
		switch (*feature_data) {
			case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
				*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1; // type2 - VC enable
				break;
			case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
				*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
				break;
			case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
				*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
				break;
			case MSDK_SCENARIO_ID_SLIM_VIDEO:
				*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
				break;
			case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
				*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
				break;
			default:
				*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
				break;
		}
		break;
		
	case SENSOR_FEATURE_GET_PDAF_REG_SETTING:
		pr_debug("SENSOR_FEATURE_GET_PDAF_REG_SETTING %d",
			(*feature_para_len));
		break;
	case SENSOR_FEATURE_SET_PDAF:
		pr_debug("PDAF mode :%d\n", *feature_data_16);
		imgsensor.pdaf_mode = *feature_data_16;
		break;
#endif


	case SENSOR_FEATURE_SET_AWB_GAIN:
		break;
	case SENSOR_FEATURE_SET_LSC_TBL:
		break;
	case SENSOR_FEATURE_GET_AWB_REQ_BY_SCENARIO:
		break;

	case SENSOR_FEATURE_SET_IHDR_SHUTTER_GAIN:
		pr_debug("SENSOR_SET_SENSOR_IHDR LE=%d, SE=%d, Gain=%d\n",
			(UINT16)*feature_data,
			(UINT16)*(feature_data+1),
			(UINT16)*(feature_data+2));
		break;
	case SENSOR_FEATURE_SET_SHUTTER_FRAME_TIME:
		set_shutter_frame_length((UINT16) (*feature_data),
					(UINT16) (*(feature_data + 1)),
					(BOOL) (*(feature_data + 2)));
		break;
	case SENSOR_FEATURE_GET_FRAME_CTRL_INFO_BY_SCENARIO:
		/*
		 * 1, if driver support new sw frame sync
		 * set_shutter_frame_length() support third para auto_extend_en
		 */
		*(feature_data + 1) = 1;
		/* margin info by scenario */
		*(feature_data + 2) = imgsensor_info.margin;
		break;
	case SENSOR_FEATURE_SET_HDR_SHUTTER:
		pr_debug("SENSOR_FEATURE_SET_HDR_SHUTTER LE=%d, SE=%d\n",
			(UINT16)*feature_data, (UINT16)*(feature_data+1));
		#if 0
		ihdr_write_shutter((UINT16)*feature_data,
				   (UINT16)*(feature_data+1));
		#endif
		break;
	case SENSOR_FEATURE_SET_STREAMING_SUSPEND:
		pr_debug("SENSOR_FEATURE_SET_STREAMING_SUSPEND\n");
		streaming_control(KAL_FALSE);
		break;
	case SENSOR_FEATURE_SET_STREAMING_RESUME:
		pr_debug("SENSOR_FEATURE_SET_STREAMING_RESUME, shutter:%llu\n",
			*feature_data);
		if (*feature_data != 0)
			set_shutter(*feature_data);
		streaming_control(KAL_TRUE);
		break;
	case SENSOR_FEATURE_GET_BINNING_TYPE:
		switch (*(feature_data + 1)) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			*feature_return_para_32 = 1; /*BINNING_AVERAGED*/
			break;
		}
		pr_debug("SENSOR_FEATURE_GET_BINNING_TYPE AE_binning_type:%d,\n",
			*feature_return_para_32);
		*feature_para_len = 4;

		break;
	case SENSOR_FEATURE_GET_PIXEL_RATE:
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

	case SENSOR_FEATURE_GET_MIPI_PIXEL_RATE:
	{
		switch (*feature_data) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.cap.mipi_pixel_rate;
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.normal_video.mipi_pixel_rate;
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.hs_video.mipi_pixel_rate;
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.slim_video.mipi_pixel_rate;
			break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.pre.mipi_pixel_rate;
			break;
		}
	}
	break;



	default:
		break;
	}
	return ERROR_NONE;
} /* feature_control() */

static struct SENSOR_FUNCTION_STRUCT sensor_func = {
	open,
	get_info,
	get_resolution,
	feature_control,
	control,
	close
};

UINT32 HI847_MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc)
{
	/* To Do : Check Sensor status here */
	if (pfFunc != NULL)
		*pfFunc = &sensor_func;
	return ERROR_NONE;
} /* HI847_MIPI_RAW_SensorInit */
