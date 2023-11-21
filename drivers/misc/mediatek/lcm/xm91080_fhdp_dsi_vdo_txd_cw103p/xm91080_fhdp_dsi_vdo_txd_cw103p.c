/* Copyright Statement:
*
* This software/firmware and related documentation ("MediaTek Software") are
* protected under relevant copyright laws. The information contained herein
* is confidential and proprietary to MediaTek Inc. and/or its licensors.
* Without the prior written permission of MediaTek inc. and/or its licensors,
* any reproduction, modification, use or disclosure of MediaTek Software,
* and information contained herein, in whole or in part, shall be strictly prohibited.
*/
/* MediaTek Inc. (C) 2015. All rights reserved.
*
* BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
* THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
* RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO RECEIVER ON
* AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
* NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
* SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
* SUPPLIED WITH THE MEDIATEK SOFTWARE, AND RECEIVER AGREES TO LOOK ONLY TO SUCH
* THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. RECEIVER EXPRESSLY ACKNOWLEDGES
* THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES
* CONTAINED IN MEDIATEK SOFTWARE. MEDIATEK SHALL ALSO NOT BE RESPONSIBLE FOR ANY MEDIATEK
* SOFTWARE RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR
* STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND
* CUMULATIVE LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
* AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
* OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY RECEIVER TO
* MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
*/

#define LOG_TAG "LCM"

#ifndef BUILD_LK
#include <linux/string.h>
#include <linux/kernel.h>
#endif

#include "lcm_drv.h"

#ifdef BUILD_LK
#include <platform/upmu_common.h>
#include <platform/mt_gpio.h>
#include <platform/mt_i2c.h>
#include <platform/mt_pmic.h>
#include <string.h>
#elif defined(BUILD_UBOOT)
#include <asm/arch/mt_gpio.h>
#else
#include "disp_dts_gpio.h"
#endif

#ifndef MACH_FPGA
#include <lcm_pmic.h>
#endif

#ifdef BUILD_LK
#define LCM_LOGI(string, args...)  dprintf(0, "[LK/"LOG_TAG"]"string, ##args)
#define LCM_LOGD(string, args...)  dprintf(1, "[LK/"LOG_TAG"]"string, ##args)
#else
#define LCM_LOGI(fmt, args...)  pr_debug("[KERNEL/"LOG_TAG"]"fmt, ##args)
#define LCM_LOGD(fmt, args...)  pr_debug("[KERNEL/"LOG_TAG"]"fmt, ##args)
#endif

#define LCM_ID_XM91080 (0x02)//raw is 0x40

static const unsigned int BL_MIN_LEVEL = 20;
static struct LCM_UTIL_FUNCS lcm_util;

#define SET_RESET_PIN(v)	(lcm_util.set_reset_pin((v)))
#define MDELAY(n)		(lcm_util.mdelay(n))
#define UDELAY(n)		(lcm_util.udelay(n))


#define dsi_set_cmdq_V22(cmdq, cmd, count, ppara, force_update) \
	lcm_util.dsi_set_cmdq_V22(cmdq, cmd, count, ppara, force_update)
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update) \
	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update) \
		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd) lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums) \
		lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd) \
	  lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size) \
		lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

#ifndef BUILD_LK
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/irq.h>
/* #include <linux/jiffies.h> */
/* #include <linux/delay.h> */
#include <linux/uaccess.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#endif

/* static unsigned char lcd_id_pins_value = 0xFF; */
static const unsigned char LCD_MODULE_ID = 0x01;
#define LCM_DSI_CMD_MODE									0
#define FRAME_WIDTH										(1080)
#define FRAME_HEIGHT									(2160)

#define REGFLAG_DELAY       0xFFFC
#define REGFLAG_UDELAY  0xFFFB
#define REGFLAG_END_OF_TABLE    0xFFFD
#define REGFLAG_RESET_LOW   0xFFFE
#define REGFLAG_RESET_HIGH  0xFFFF
static struct LCM_DSI_MODE_SWITCH_CMD lcm_switch_mode_cmd;

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

struct LCM_setting_table {
	unsigned int cmd;
	unsigned char count;
	unsigned char para_list[64];
};

static struct LCM_setting_table lcm_suspend_setting[] = {
    {0x28, 0, {} },
	{REGFLAG_DELAY, 30, {} },
	{0x10, 0, {} },
	{REGFLAG_DELAY, 120, {} },


    {REGFLAG_END_OF_TABLE, 0x00, {} }
};

static struct LCM_setting_table lcm_initialization_setting[] =
{
//==== Initial Code Start =======     

// Set XM Command Password 1
    {0x00,1,{0x00}}, 
    {0xFF,3,{0x10,0x80,0x01}},  

//Set XM Command Password 2
    {0x00,1,{0x80}},  
    {0xFF,2,{0x10,0x80}},  

//tcon setting ok
    {0x00,1,{0x81}},  
    {0xb2,5,{0xA0,0x00,0x14,0x00,0x14}},  

    {0x00,1,{0x86}},  
    {0xb2,6,{0x04,0x06,0x04,0x04,0x23,0x04}}, 

////ckv setting
    {0x00,1,{0x80}}, 
    {0xb4,10,{0x18,0x0B,0x08,0x77,0x0f,0x00,0x00,0x02,0x00,0x00}}, 
    {0x00,1,{0x90}}, 
    {0xb4,10,{0x18,0x0A,0x08,0x78,0x0f,0x00,0x00,0x02,0x00,0x00}}, 
    {0x00,1,{0xa0}}, 
    {0xb4,10,{0x18,0x09,0x08,0x79,0x0f,0x00,0x00,0x02,0x00,0x00}}, 
    {0x00,1,{0xb0}}, 
    {0xb4,10,{0x18,0x08,0x08,0x7A,0x0f,0x00,0x00,0x02,0x00,0x00}}, 

//vst
    {0x00,1,{0x80}}, 
    {0xb6,8,{0x83,0x02,0x00,0x00,0x82,0x02,0x00,0x00}},  

//GCH
    {0x00,1,{0xB0}}, 
    {0xb6,4,{0x08,0x2f,0x08,0x0c}}, 

    {0x00,1,{0xc1}},  
    {0xb2,1,{0x01}}, 

//u2d ok
    {0x00,1,{0x80}}, 
    {0xbc,16,{0x00,0x00,0x0E,0x26,0x25,0x02,0x1D,0x00,0x08,0x06,0x1F,0x20,0x21,0x00,0x00,0x00}}, 
    {0x00,1,{0x90}}, 
    {0xbc,16,{0x00,0x00,0x0D,0x26,0x25,0x02,0x1D,0x00,0x07,0x05,0x1F,0x20,0x21,0x00,0x00,0x00}}, 

//d2u ok
    {0x00,1,{0xa0}}, 
    {0xbc,16,{0x00,0x00,0x0D,0x25,0x26,0x02,0x1D,0x00,0x05,0x07,0x1F,0x20,0x21,0x00,0x00,0x00}}, 
    {0x00,1,{0xb0}}, 
    {0xbc,16,{0x00,0x00,0x0E,0x25,0x26,0x02,0x1D,0x00,0x06,0x08,0x1F,0x20,0x21,0x00,0x00,0x00}}, 

//enmode ok
    {0x00,1,{0xa0}}, 
    {0xb9,16,{0x00,0x00,0xE4,0xE4,0xE4,0xEB,0xE7,0xE3,0xE6,0xE6,0xE7,0xE7,0xE7,0x00,0x00,0x00}}, 
    {0x00,1,{0xb0}}, 
    {0xb9,16,{0x00,0x00,0xE4,0xE4,0xE4,0xEB,0xE7,0xE3,0xE6,0xE6,0xE7,0xE7,0xE7,0x00,0x00,0x00}}, 
    {0x00,1,{0x80}}, 
    {0xb9,16,{0x00,0x00,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0x00,0x00,0x00}}, 
    {0x00,1,{0x90}}, 
    {0xb9,16,{0x00,0x00,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0x00,0x00,0x00}}, 

//gip lvd
    {0x00,1,{0x80}}, 
    {0xba,8,{0x0A,0xAA,0xAA,0x80,0x0A,0xAA,0xAA,0x80}}, 

//gip ultra setting
    {0x00,1,{0xd0}}, 
    {0xb6,4,{0x81,0x00,0x02,0x02}},  
    {0x00,1,{0xe0}},  
    {0xb6,10,{0x00,0x0c,0x00,0x04,0x20,0x11,0x01,0x01,0x04,0x00}},  

// G-swap                                                                                 
    {0x00,1,{0xA5}},                                                                              
    {0xC0,1,{0x20}},    

// mirror X2                                                                                 
    {0x00,1,{0xA0}},                                                                              
    {0xA5,1,{0x20}}, 

//SW panel size 1080x2160
    {0x00,1,{0xA1}},   
    {0xC0,4,{0x04,0x38,0x08,0x70}},
 
    {0x00,1,{0xa6}}, 
    {0xc0,1,{0x10}},   

// VGH=VGHO/VGL=VGLO
    {0x00,1,{0xF0}},                                                                             
    {0xA4,1,{0x00}}, 

// VGL/VGH = -8 / 8
    {0x00,1,{0x90}},                                                                             
    {0xAB,2,{0xA8,0x94}}, 

//gamma2.2
    {0x00,1,{0x80}},                                                                                                                             //
    {0xD4,38,{0x00,0x03,0x0B,0x14,0x19,0x1E,0x21,0x25,0x28,0x33,0x3B,0x48,0x52,0x62,0x70,0x70,0x7F,0x92,0x9E,0xAF,0xBA,0xC9,0xCD,0xD2,0xD8,0xDE,0xE5,0xED,0xF8,0xFF,0x00,0x00,0x00,0x02,0x00,0x00,0x00,0x00}},
    {0x00,1,{0x80}},
    {0xD5,38,{0x00,0x03,0x0B,0x14,0x19,0x1E,0x21,0x25,0x28,0x33,0x3B,0x48,0x52,0x62,0x70,0x70,0x7F,0x92,0x9E,0xAF,0xBA,0xC9,0xCD,0xD2,0xD8,0xDE,0xE5,0xED,0xF8,0xFF,0x00,0x00,0x00,0x02,0x00,0x00,0x00,0x00}},
    {0x00,1,{0x80}},
    {0xD6,38,{0x00,0x03,0x0B,0x14,0x19,0x1E,0x21,0x25,0x28,0x33,0x3B,0x48,0x52,0x62,0x70,0x70,0x7F,0x92,0x9E,0xAF,0xBA,0xC9,0xCD,0xD2,0xD8,0xDE,0xE5,0xED,0xF8,0xFF,0x00,0x00,0x00,0x02,0x00,0x00,0x00,0x00}},
    {0x00,1,{0x80}},
    {0xD7,38,{0x00,0x03,0x0B,0x14,0x19,0x1E,0x21,0x25,0x28,0x33,0x3B,0x48,0x52,0x62,0x70,0x70,0x7F,0x92,0x9E,0xAF,0xBA,0xC9,0xCD,0xD2,0xD8,0xDE,0xE5,0xED,0xF8,0xFF,0x00,0x00,0x00,0x02,0x00,0x00,0x00,0x00}},
    {0x00,1,{0x80}},
    {0xD8,38,{0x00,0x03,0x0B,0x14,0x19,0x1E,0x21,0x25,0x28,0x33,0x3B,0x48,0x52,0x62,0x70,0x70,0x7F,0x92,0x9E,0xAF,0xBA,0xC9,0xCD,0xD2,0xD8,0xDE,0xE5,0xED,0xF8,0xFF,0x00,0x00,0x00,0x02,0x00,0x00,0x00,0x00}},
    {0x00,1,{0x80}},
    {0xD9,38,{0x00,0x03,0x0B,0x14,0x19,0x1E,0x21,0x25,0x28,0x33,0x3B,0x48,0x52,0x62,0x70,0x70,0x7F,0x92,0x9E,0xAF,0xBA,0xC9,0xCD,0xD2,0xD8,0xDE,0xE5,0xED,0xF8,0xFF,0x00,0x00,0x00,0x02,0x00,0x00,0x00,0x00}},

//VCOM
//    {0x00,1,{0xB1}},                                                                              
//    {0xA4,2,{0x9C,0x9C}}, 

//GVDD=5.2V
    {0x00,1,{0xA0}},                                                                              
    {0xA4,2,{0x2F,0x2F}}, 

// vglo_str
    {0x00,1,{0x9C}},                                                                              
    {0xA6,1,{0x90}},   

// enmode_sdpch
    {0x00,1,{0xC2}},                                                                              
    {0xA6,1,{0x08}},   

// sd_prc
    {0x00,1,{0x86}},                                                                              
    {0xA5,1,{0x19}}, 

// mipi skew
    {0x00,1,{0x90}},  
    {0xA3,6,{0x04,0x04,0x01,0x05,0x06,0x00}},   

    {0x11,1,{0x00}},
    {REGFLAG_DELAY,120, {}},
    {0x29,1,{0x00}},
	 //-------------  Display Initial Setting end ------------------------- 
																		  
	 {REGFLAG_DELAY,20,{}},   
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
	unsigned int i;
	unsigned int cmd;

	for (i = 0; i < count; i++) {
		cmd = table[i].cmd;

		switch (cmd) {

		case REGFLAG_DELAY:
			if (table[i].count <= 10)
				MDELAY(table[i].count);
			else
				MDELAY(table[i].count);
			break;

		case REGFLAG_UDELAY:
			UDELAY(table[i].count);
			break;

		case REGFLAG_END_OF_TABLE:
			break;

		default:
            dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
		}
	}
}


static void lcm_set_util_funcs(const struct LCM_UTIL_FUNCS *util)
{
	memcpy(&lcm_util, util, sizeof(struct LCM_UTIL_FUNCS));
}


static void lcm_get_params(struct LCM_PARAMS *params)
{
	memset(params, 0, sizeof(struct LCM_PARAMS));

	params->type = LCM_TYPE_DSI;

	params->width = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;
	#ifndef BUILD_LK
	params->physical_width               = 64;
	params->physical_height              = 115;
	params->physical_width_um            = 64800;
	params->physical_height_um           = 115200;
//	params->density                      = 320;
	#endif

	// enable tearing-free
	params->dbi.te_mode                  = LCM_DBI_TE_MODE_VSYNC_ONLY;
	params->dbi.te_edge_polarity         = LCM_POLARITY_RISING;
#if (LCM_DSI_CMD_MODE)
	params->dsi.mode = CMD_MODE;
	params->dsi.switch_mode = SYNC_PULSE_VDO_MODE;
#else
	params->dsi.mode = SYNC_PULSE_VDO_MODE;
	params->dsi.switch_mode = CMD_MODE;
#endif
	params->dsi.switch_mode_enable = 0;

	/* DSI */
	/* Command mode setting */
	params->dsi.LANE_NUM = LCM_FOUR_LANE;
	/* The following defined the fomat for data coming from LCD engine. */
	params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq = LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding = LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format = LCM_DSI_FORMAT_RGB888;

	/* Highly depends on LCD driver capability. */
	params->dsi.packet_size = 256;
	/* video mode timing */

	params->dsi.PS = LCM_PACKED_PS_24BIT_RGB888;

	params->dsi.vertical_sync_active = 2;
	params->dsi.vertical_backporch = 16;
	params->dsi.vertical_frontporch = 16;
	/* params->dsi.vertical_frontporch_for_low_power = 540; */
	params->dsi.vertical_active_line = FRAME_HEIGHT;

	params->dsi.horizontal_sync_active = 4;
	params->dsi.horizontal_backporch = 30;
	params->dsi.horizontal_frontporch = 30;
	params->dsi.horizontal_active_pixel = FRAME_WIDTH;
	/* params->dsi.ssc_disable = 1; */

	params->dsi.PLL_CLOCK = 488;//240;	/* this value must be in MTK suggested table */
//Data rate= (Height+VSA+VBP+VFP)*(Width+HSA+HBP+HFP)* total_bit_per_pixel*frame_per_second/total_lane_num
//clk=data_rate/2
//clk=(2160+2+16+16)*(1080+4+30+30)*24*60/4/2
//451.788480M
	params->dsi.esd_check_enable = 0;
	params->dsi.customization_esd_check_enable = 0;
	params->dsi.lcm_esd_check_table[0].cmd = 0x0A;
	params->dsi.lcm_esd_check_table[0].count = 1;
	params->dsi.lcm_esd_check_table[0].para_list[0] = 0x9C;

}

static void lcm_init(void)
{
	//SET_RESET_PIN(0);
	display_bias_enable();
//#ifndef MACH_FPGA
#if 0
	int ret = 0;
    /*config rt5081 register 0xB2[7:6]=0x3, that is set db_delay=4ms.*/
    ret = PMU_REG_MASK(0xB2, (0x3 << 6), (0x3 << 6));

    /* set AVDD 5.4v, (4v+28*0.05v) */
    /*ret = RT5081_write_byte(0xB3, (1 << 6) | 28);*/
	ret = PMU_REG_MASK(0xB3, 28, (0x3F << 0));
    if (ret < 0)
        LCM_LOGI("xm92160----tps6132----cmd=%0x--i2c write error----\n", 0xB3);
    else
        LCM_LOGI("xm92160----tps6132----cmd=%0x--i2c write success----\n", 0xB3);

    /* set AVEE */
    /*ret = RT5081_write_byte(0xB4, (1 << 6) | 28);*/
	ret = PMU_REG_MASK(0xB4, 28, (0x3F << 0));
    if (ret < 0)
        LCM_LOGI("xm92160----tps6132----cmd=%0x--i2c write error----\n", 0xB4);
    else
        LCM_LOGI("xm92160----tps6132----cmd=%0x--i2c write success----\n", 0xB4);

    /* enable AVDD & AVEE */
    /* 0x12--default value; bit3--Vneg; bit6--Vpos; */
    /*ret = RT5081_write_byte(0xB1, 0x12 | (1<<3) | (1<<6));*/
    ret = PMU_REG_MASK(0xB1, (1<<3) | (1<<6), (1<<3) | (1<<6));
    if (ret < 0)
        LCM_LOGI("xm92160----tps6132----cmd=%0x--i2c write error----\n", 0xB1);
    else
        LCM_LOGI("xm92160----tps6132----cmd=%0x--i2c write success----\n", 0xB1);

//    MDELAY(15);

#endif
	LCM_LOGI("%s\n", __func__);
	MDELAY(20);
	SET_RESET_PIN(1);
	MDELAY(10);
	SET_RESET_PIN(0);
	MDELAY(10);

	SET_RESET_PIN(1);
	MDELAY(50);

    push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
}

static void lcm_suspend(void)
{
	LCM_LOGI("%s\n", __func__);

	MDELAY(10);
    push_table(lcm_suspend_setting, sizeof(lcm_suspend_setting) / sizeof(struct LCM_setting_table), 1);

	SET_RESET_PIN(0);
    MDELAY(10);

	display_bias_disable();

	/* SET_RESET_PIN(0); */
}

static void lcm_resume(void)
{
	/*LCM_LOGD("lcm_resume\n");*/

	lcm_init();
}

static void lcm_update(unsigned int x, unsigned int y, unsigned int width, unsigned int height)
{
	unsigned int x0 = x;
	unsigned int y0 = y;
	unsigned int x1 = x0 + width - 1;
	unsigned int y1 = y0 + height - 1;

	unsigned char x0_MSB = ((x0 >> 8) & 0xFF);
	unsigned char x0_LSB = (x0 & 0xFF);
	unsigned char x1_MSB = ((x1 >> 8) & 0xFF);
	unsigned char x1_LSB = (x1 & 0xFF);
	unsigned char y0_MSB = ((y0 >> 8) & 0xFF);
	unsigned char y0_LSB = (y0 & 0xFF);
	unsigned char y1_MSB = ((y1 >> 8) & 0xFF);
	unsigned char y1_LSB = (y1 & 0xFF);

	unsigned int data_array[16];

	data_array[0] = 0x00053902;
	data_array[1] = (x1_MSB << 24) | (x0_LSB << 16) | (x0_MSB << 8) | 0x2a;
	data_array[2] = (x1_LSB);
	dsi_set_cmdq(data_array, 3, 1);

	data_array[0] = 0x00053902;
	data_array[1] = (y1_MSB << 24) | (y0_LSB << 16) | (y0_MSB << 8) | 0x2b;
	data_array[2] = (y1_LSB);
	dsi_set_cmdq(data_array, 3, 1);

	data_array[0] = 0x002c3909;
	dsi_set_cmdq(data_array, 1, 0);
}

static unsigned int lcm_compare_id(void)
{
//	return 1;

	unsigned int id = 0;//, version_id = 0;
	unsigned char buffer[2];
//	unsigned int array[16];

	display_bias_enable();
	MDELAY(20);
	SET_RESET_PIN(1);
	MDELAY(10);
	SET_RESET_PIN(0);
	MDELAY(10);

	SET_RESET_PIN(1);
	MDELAY(50);

//	array[0] = 0x00023700;	// read id return two byte,version and id 
//	dsi_set_cmdq(array, 1, 1);

//	read_reg_v2(0xF4, buffer, 2);
//	id = buffer[0];		// we only need ID 
//
//	read_reg_v2(0xDB, buffer, 1);
//	version_id = buffer[0];

	read_reg_v2(0xDA, buffer, 2);
	id = buffer[0];		// we only need ID 

	LCM_LOGI("%s,is 0x%02x ?? xm91080_id=0x%02x\n", __func__, LCM_ID_XM91080,id);

	SET_RESET_PIN(0);
    MDELAY(10);

	display_bias_disable();

	if (id == LCM_ID_XM91080)
		return 1;
	else
		return 0;

}


/* return TRUE: need recovery */
/* return FALSE: No need recovery */
static unsigned int lcm_esd_check(void)
{
#ifndef BUILD_LK
	char buffer[3];
	int array[4];

	array[0] = 0x00013700;
	dsi_set_cmdq(array, 1, 1);

	read_reg_v2(0x53, buffer, 1);

	if (buffer[0] != 0x24) {
		LCM_LOGI("[LCM ERROR] [0x53]=0x%02x\n", buffer[0]);
		return TRUE;
	} else {
		LCM_LOGI("[LCM NORMAL] [0x53]=0x%02x\n", buffer[0]);
		return FALSE;
	}
#else
	return FALSE;
#endif

}

static unsigned int lcm_ata_check(unsigned char *buffer)
{
#ifndef BUILD_LK
	unsigned int ret = 0;
	unsigned int x0 = FRAME_WIDTH / 4;
	unsigned int x1 = FRAME_WIDTH * 3 / 4;

	unsigned char x0_MSB = ((x0 >> 8) & 0xFF);
	unsigned char x0_LSB = (x0 & 0xFF);
	unsigned char x1_MSB = ((x1 >> 8) & 0xFF);
	unsigned char x1_LSB = (x1 & 0xFF);

	unsigned int data_array[3];
	unsigned char read_buf[4];

	pr_debug("[LCM]ATA check size = 0x%x,0x%x,0x%x,0x%x\n",
		x0_MSB, x0_LSB, x1_MSB, x1_LSB);
	data_array[0] = 0x0005390A;	/* HS packet */
	data_array[1] = (x1_MSB << 24) | (x0_LSB << 16) | (x0_MSB << 8) | 0x2a;
	data_array[2] = (x1_LSB);
	dsi_set_cmdq(data_array, 3, 1);

	/* read id return two byte,version and id */
	data_array[0] = 0x00043700;
	dsi_set_cmdq(data_array, 1, 1);

	read_reg_v2(0x2A, read_buf, 4);

	if ((read_buf[0] == x0_MSB) && (read_buf[1] == x0_LSB)
	    && (read_buf[2] == x1_MSB) && (read_buf[3] == x1_LSB))
		ret = 1;
	else
		ret = 0;

	x0 = 0;
	x1 = FRAME_WIDTH - 1;

	x0_MSB = ((x0 >> 8) & 0xFF);
	x0_LSB = (x0 & 0xFF);
	x1_MSB = ((x1 >> 8) & 0xFF);
	x1_LSB = (x1 & 0xFF);

	data_array[0] = 0x0005390A;	/* HS packet */
	data_array[1] = (x1_MSB << 24) | (x0_LSB << 16) | (x0_MSB << 8) | 0x2a;
	data_array[2] = (x1_LSB);
	dsi_set_cmdq(data_array, 3, 1);
	return ret;
#else
	return 0;
#endif
}

static void *lcm_switch_mode(int mode)
{
#ifndef BUILD_LK
/* customization: 1. V2C config 2 values, C2V config 1 value; 2. config mode control register */
	if (mode == 0) {	/* V2C */
		lcm_switch_mode_cmd.mode = CMD_MODE;
		lcm_switch_mode_cmd.addr = 0xBB;	/* mode control addr */
		lcm_switch_mode_cmd.val[0] = 0x13;	/* enabel GRAM firstly, ensure writing one frame to GRAM */
		lcm_switch_mode_cmd.val[1] = 0x10;	/* disable video mode secondly */
	} else {		/* C2V */
		lcm_switch_mode_cmd.mode = SYNC_PULSE_VDO_MODE;
		lcm_switch_mode_cmd.addr = 0xBB;
		lcm_switch_mode_cmd.val[0] = 0x03;	/* disable GRAM and enable video mode */
	}
	return (void *)(&lcm_switch_mode_cmd);
#else
	return NULL;
#endif
}


struct LCM_DRIVER xm91080_fhdp_dsi_vdo_txd_cw103p_lcm_drv = {
	.name = "xm91080_fhdp_dsi_vdo_txd_cw103p",
#if defined(CONFIG_PRIZE_HARDWARE_INFO) && !defined (BUILD_LK)
	.lcm_info = {
		.chip	= "xm91080_gms",
		.vendor	= "txd",
		.id		= "0x82",
		.more	= "1080*2160",
	},
#endif
	.set_util_funcs = lcm_set_util_funcs,
	.get_params = lcm_get_params,
	.init = lcm_init,
	.suspend = lcm_suspend,
	.resume = lcm_resume,
	.compare_id = lcm_compare_id,
	.esd_check = lcm_esd_check,
	.ata_check = lcm_ata_check,
	.update = lcm_update,
	.switch_mode = lcm_switch_mode,
};
