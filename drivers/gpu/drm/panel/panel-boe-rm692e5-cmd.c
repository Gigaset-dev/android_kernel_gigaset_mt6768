/*
 * Copyright (c) 2015 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/backlight.h>
#include <drm/drmP.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_panel.h>

#include <linux/gpio/consumer.h>
#include <linux/regulator/consumer.h>

#include <video/mipi_display.h>
#include <video/of_videomode.h>
#include <video/videomode.h>

#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/of_graph.h>
#include <linux/platform_device.h>

#define CONFIG_MTK_PANEL_EXT
#if defined(CONFIG_MTK_PANEL_EXT)
#include "../mediatek/mtk_panel_ext.h"
#include "../mediatek/mtk_log.h"
#include "../mediatek/mtk_drm_graphics_base.h"
#endif

#include <linux/i2c-dev.h>
#include <linux/i2c.h>

#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
#include "../mediatek/mtk_corner_pattern/mtk_data_hw_roundedpattern.h"
#endif
#include "include/panel-boe-rm692e5-cmd.h"
/* i2c control start */
#define LCM_I2C_ID_NAME "I2C_LCD_BIAS"
static struct i2c_client *_lcm_i2c_client;
/*PRIZE:Added by lvyuanchuan,X9-678,20221230 start*/
#define BLK_LEVEL_OFFSET			(0)
#define BLK_LEVEL_MAP3				(4095)
extern int mtk_drm_esd_check_status(void);
extern void mtk_drm_esd_set_status(int status);
/*PRIZE:Added by lvyuanchuan,X9-678,20221230 end*/
/*PRIZE:Added by lvyuanchuan,X9-534,20230103 start*/
static unsigned int Gamma_to_level[] = {
0     ,
1     ,
1     ,
1     ,
2     ,
2     ,
2     ,
3     ,
4     ,
5     ,
6     ,
7     ,
9     ,
10    ,
12    ,
13    ,
15    ,
17    ,
19    ,
21    ,
23    ,
26    ,
28    ,
31    ,
33    ,
36    ,
39    ,
42    ,
45    ,
48    ,
51    ,
55    ,
58    ,
62    ,
66    ,
70    ,
74    ,
78    ,
82    ,
86    ,
90    ,
95    ,
99    ,
104   ,
109   ,
114   ,
119   ,
124   ,
129   ,
135   ,
140   ,
146   ,
151   ,
157   ,
163   ,
169   ,
175   ,
181   ,
188   ,
194   ,
201   ,
207   ,
214   ,
221   ,
228   ,
235   ,
242   ,
249   ,
257   ,
264   ,
272   ,
280   ,
288   ,
296   ,
304   ,
312   ,
320   ,
328   ,
337   ,
345   ,
354   ,
363   ,
372   ,
381   ,
390   ,
399   ,
409   ,
418   ,
428   ,
437   ,
447   ,
457   ,
467   ,
477   ,
487   ,
498   ,
508   ,
519   ,
529   ,
540   ,
551   ,
562   ,
573   ,
584   ,
595   ,
607   ,
618   ,
630   ,
642   ,
654   ,
666   ,
678   ,
690   ,
702   ,
714   ,
727   ,
739   ,
752   ,
765   ,
778   ,
791   ,
804   ,
817   ,
831   ,
844   ,
858   ,
871   ,
885   ,
899   ,
913   ,
927   ,
941   ,
956   ,
970   ,
985   ,
999   ,
1014  ,
1024  ,
1044  ,
1059  ,
1074  ,
1089  ,
1105  ,
1120  ,
1136  ,
1152  ,
1168  ,
1184  ,
1200  ,
1216  ,
1232  ,
1248  ,
1265  ,
1281  ,
1298  ,
1315  ,
1332  ,
1349  ,
1366  ,
1383  ,
1401  ,
1418  ,
1436  ,
1453  ,
1471  ,
1489  ,
1507  ,
1525  ,
1543  ,
1562  ,
1580  ,
1599  ,
1617  ,
1636  ,
1655  ,
1674  ,
1693  ,
1712  ,
1732  ,
1751  ,
1770  ,
1790  ,
1810  ,
1830  ,
1850  ,
1870  ,
1890  ,
1910  ,
1930  ,
1951  ,
1972  ,
1992  ,
2013  ,
2034  ,
2055  ,
2076  ,
2097  ,
2119  ,
2140  ,
2162  ,
2183  ,
2205  ,
2227  ,
2249  ,
2271  ,
2293  ,
2316  ,
2338  ,
2361  ,
2383  ,
2406  ,
2429  ,
2452  ,
2475  ,
2498  ,
2522  ,
2545  ,
2568  ,
2592  ,
2616  ,
2640  ,
2664  ,
2688  ,
2712  ,
2736  ,
2760  ,
2785  ,
2810  ,
2834  ,
2859  ,
2884  ,
2909  ,
2934  ,
2959  ,
2985  ,
3010  ,
3036  ,
3061  ,
3087  ,
3113  ,
3139  ,
3165  ,
3191  ,
3218  ,
3244  ,
3271  ,
3297  ,
3324  ,
3351  ,
3378  ,
3405  ,
3432  ,
3460  ,
3487  ,
3515  ,
3515  ,
/* prize modified by gongtaitao for x9 lava hbm mode 20230421 start */
3765  ,
3895  ,
3997  ,
4095
/* prize modified by gongtaitao for x9 lava hbm mode 20230421 end */
};
/*PRIZE:Added by lvyuanchuan,X9-534,20230103 end*/
//prize add by wangfei for lcd hardware info 20210726 start
#if defined(CONFIG_PRIZE_HARDWARE_INFO)
#include "../../../misc/prize/hardware_info/hardware_info.h"
extern struct hardware_info current_lcm_info;
#endif
//prize add by wangfei for lcd hardware info 20210726 end


//prize add by wangfei for HBM 20210906 start
unsigned int bl_level;
//prize add by wangfei for HBM 20210906 end
struct lcm *g_ctx;
//prize add by majiangtao for frequency Select 20230103 start
int g_vrefresh = -1;
//prize add by majiangtao for frequency Select 20230103 end
/* prize modified by gongtaitao for x9 lava hbm mode 20230421 start */
#define NORMAL_MAX_LEVEL 255
#define HBM_MAX_LEVEL 259
#define CONTINUOUS_HBM_TIMES 600
#define CONTINUOUS_NORMAL_TIMES 200
#define NORMAL_TIMES 100
static struct timer_list check_level_timer;
static struct work_struct check_level_worker;
static unsigned int g_max_set_level = HBM_MAX_LEVEL;
static unsigned int g_current_level = 0;
extern int prize_mt_leds_set_max_brightness(char *name, int percent, bool enable);
/* prize modified by gongtaitao for x9 lava hbm mode 20230421 end */



struct lcm {
	struct device *dev;
	struct drm_panel panel;
	struct backlight_device *backlight;
	struct gpio_desc *reset_gpio;
	struct gpio_desc *bias_pos, *bias_neg;
	struct gpio_desc *pm_enable_gpio;

	bool prepared;
	bool enabled;
	bool oled_screen;/* drv-add oled sysfs-pengzhipeng-20230306-end */
	bool hbm_en;
	bool hbm_wait;
	bool hbm_stat;           //0Î´ÔÚHBM  1ÔÚHBM

	int error;
};

#define lcm_dcs_write_seq(ctx, seq...) \
({\
	const u8 d[] = { seq };\
	BUILD_BUG_ON_MSG(ARRAY_SIZE(d) > 64, "DCS sequence too big for stack");\
	lcm_dcs_write(ctx, d, ARRAY_SIZE(d));\
})

#define lcm_dcs_write_seq_static(ctx, seq...) \
({\
	static const u8 d[] = { seq };\
	lcm_dcs_write(ctx, d, ARRAY_SIZE(d));\
})

static inline struct lcm *panel_to_lcm(struct drm_panel *panel)
{
	return container_of(panel, struct lcm, panel);
}

static void lcm_dcs_write(struct lcm *ctx, const void *data, size_t len)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	ssize_t ret;
	char *addr;

	if (ctx->error < 0)
		return;

	addr = (char *)data;
	if ((int)*addr < 0xB0)
		ret = mipi_dsi_dcs_write_buffer(dsi, data, len);
	else
		ret = mipi_dsi_generic_write(dsi, data, len);
	if (ret < 0) {
		dev_err(ctx->dev, "error %zd writing seq: %ph\n", ret, data);
		ctx->error = ret;
	}
}

#ifdef PANEL_SUPPORT_READBACK
static int lcm_dcs_read(struct lcm *ctx, u8 cmd, void *data, size_t len)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	ssize_t ret;

	if (ctx->error < 0)
		return 0;

	ret = mipi_dsi_dcs_read(dsi, cmd, data, len);
	if (ret < 0) {
		dev_err(ctx->dev, "error %d reading dcs seq:(%#x)\n", ret, cmd);
		ctx->error = ret;
	}

	return ret;
}

static void lcm_panel_get_data(struct lcm *ctx)
{
	u8 buffer[3] = {0};
	static int ret;

	if (ret == 0) {
		ret = lcm_dcs_read(ctx,  0x0A, buffer, 1);
		dev_info(ctx->dev, "return %d data(0x%08x) to dsi engine\n",
			 ret, buffer[0] | (buffer[1] << 8));
	}
}
#endif


/*PRIZE:Added by lvyuanchuan,X9-678,20221230 start*/
static void lcm_pannel_reconfig_blk(struct lcm *ctx)
{
	char bl_tb0[] = {0x51,0x0F,0xFF};
	unsigned int reg_level = 125;
	pr_err("[%s][%d]bl_level:%d , esd:%d \n",__func__,__LINE__,bl_level ,mtk_drm_esd_check_status());
	if(mtk_drm_esd_check_status()){
		/*PRIZE:Added by lvyuanchuan,X9-534,20230103*/
		if(bl_level)
			reg_level = Gamma_to_level[bl_level] + BLK_LEVEL_OFFSET;
		else
			reg_level = 0;
		bl_tb0[1] = (reg_level>>8)&0xf;
		bl_tb0[2] = (reg_level)&0xff;
		lcm_dcs_write(ctx,bl_tb0,ARRAY_SIZE(bl_tb0));
		mtk_drm_esd_set_status(0);
	}
}
/*PRIZE:Added by lvyuanchuan,X9-678,20221230 end*/

static void lcm_panel_init(struct lcm *ctx)
{
	ctx->reset_gpio =
		devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio)) {
		dev_err(ctx->dev, "%s: cannot get reset_gpio %ld\n",
			__func__, PTR_ERR(ctx->reset_gpio));
		return;
	}

	pr_err("gezi----------%s----%d,bl_level %d\n",__func__,__LINE__,bl_level);
	gpiod_set_value(ctx->reset_gpio, 0);
	udelay(10 * 1000);
	gpiod_set_value(ctx->reset_gpio, 1);
	udelay(10 * 1000);
	gpiod_set_value(ctx->reset_gpio, 0);
	udelay(10 * 1000);
	gpiod_set_value(ctx->reset_gpio, 1);
	// udelay(250 * 1000);
	mdelay(50);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);

	lcm_dcs_write_seq_static(ctx,0xFE,0xF4);
	lcm_dcs_write_seq_static(ctx,0x00,0xFF);
	lcm_dcs_write_seq_static(ctx,0x01,0x63);
	lcm_dcs_write_seq_static(ctx,0x02,0x88);
	lcm_dcs_write_seq_static(ctx,0x03,0x9B);
	lcm_dcs_write_seq_static(ctx,0x04,0x4C);
	lcm_dcs_write_seq_static(ctx,0x05,0xF4);
	lcm_dcs_write_seq_static(ctx,0x06,0xE0);
	lcm_dcs_write_seq_static(ctx,0x07,0xF3);
	lcm_dcs_write_seq_static(ctx,0x08,0x0F);
	lcm_dcs_write_seq_static(ctx,0x09,0x40);
	lcm_dcs_write_seq_static(ctx,0x0A,0xCC);
	lcm_dcs_write_seq_static(ctx,0x0B,0xA4);
	lcm_dcs_write_seq_static(ctx,0x0C,0x82);
	lcm_dcs_write_seq_static(ctx,0x0D,0x08);
	lcm_dcs_write_seq_static(ctx,0xFE,0xF4);
	lcm_dcs_write_seq_static(ctx,0x1B,0xF0);
	lcm_dcs_write_seq_static(ctx,0x1C,0xFF);
	lcm_dcs_write_seq_static(ctx,0x1D,0xD2);
	lcm_dcs_write_seq_static(ctx,0x1E,0xD2);
	lcm_dcs_write_seq_static(ctx,0x1F,0xFE);
	lcm_dcs_write_seq_static(ctx,0x20,0x49);
	lcm_dcs_write_seq_static(ctx,0x21,0x0F);
	lcm_dcs_write_seq_static(ctx,0x22,0x3E);
	lcm_dcs_write_seq_static(ctx,0x23,0xFF);
	lcm_dcs_write_seq_static(ctx,0x24,0x00);
	lcm_dcs_write_seq_static(ctx,0x25,0xC4);
	lcm_dcs_write_seq_static(ctx,0x26,0x4C);
	lcm_dcs_write_seq_static(ctx,0x27,0x2A);
	lcm_dcs_write_seq_static(ctx,0x28,0x88);
	lcm_dcs_write_seq_static(ctx,0x29,0x00);
	lcm_dcs_write_seq_static(ctx,0xFE,0xF4);
	lcm_dcs_write_seq_static(ctx,0x0D,0xC0);
	lcm_dcs_write_seq_static(ctx,0x0E,0xFF);
	lcm_dcs_write_seq_static(ctx,0x0F,0xCB);
	lcm_dcs_write_seq_static(ctx,0x10,0x4B);
	lcm_dcs_write_seq_static(ctx,0x11,0xCD);
	lcm_dcs_write_seq_static(ctx,0x12,0x2C);
	lcm_dcs_write_seq_static(ctx,0x13,0x3D);
	lcm_dcs_write_seq_static(ctx,0x14,0xF8);
	lcm_dcs_write_seq_static(ctx,0x15,0xFC);
	lcm_dcs_write_seq_static(ctx,0x16,0x03);
	lcm_dcs_write_seq_static(ctx,0x17,0x10);
	lcm_dcs_write_seq_static(ctx,0x18,0x33);
	lcm_dcs_write_seq_static(ctx,0x19,0xA9);
	lcm_dcs_write_seq_static(ctx,0x1A,0x20);
	lcm_dcs_write_seq_static(ctx,0x1B,0x02);
	
//drv-Solving probable black screen problems-pengzhipeng-20230309-start
//ESD recover code
    lcm_dcs_write_seq_static(ctx,0xFE,0xD4);  //Page D4
    lcm_dcs_write_seq_static(ctx,0x40,0x03);  // lvd/mipi error flag enable
    lcm_dcs_write_seq_static(ctx,0xFE,0xFD); //Page FD
    lcm_dcs_write_seq_static(ctx,0x80,0x06);  //error report to 0A00 enable
    lcm_dcs_write_seq_static(ctx,0x83,0x00);  //command 1 OFF  00 Page
    lcm_dcs_write_seq_static(ctx,0xFE,0xA0);  //Page A0
    lcm_dcs_write_seq_static(ctx,0x06,0x36);  //lvd function VCI/VDDI/AVDD
    lcm_dcs_write_seq_static(ctx,0xFE,0xA1);  //Page A1
	lcm_dcs_write_seq_static(ctx,0xB3,0x7F);  //7f=hs cmd off 1f=hs cmd on
    lcm_dcs_write_seq_static(ctx,0x74,0x70);
    lcm_dcs_write_seq_static(ctx,0xC3,0xC3);
    lcm_dcs_write_seq_static(ctx,0xC4,0x9C);
    lcm_dcs_write_seq_static(ctx,0xC5,0x1E);
    lcm_dcs_write_seq_static(ctx,0xC6,0x23);
    lcm_dcs_write_seq_static(ctx,0xFE,0xD0);
    lcm_dcs_write_seq_static(ctx,0x11,0x75);
    lcm_dcs_write_seq_static(ctx,0x92,0x03);
//drv-Solving probable black screen problems-pengzhipeng-20230309-end
	lcm_dcs_write_seq_static(ctx,0xFE,0x40);
	lcm_dcs_write_seq_static(ctx,0xBD,0x00);
	lcm_dcs_write_seq_static(ctx,0xFE,0xD0);
	lcm_dcs_write_seq_static(ctx,0x86,0x14);
//drv-modify 10bit-pengzhipeng-20230429-start	
	lcm_dcs_write_seq_static(ctx,0xFE,0xD2);// switch to D2 page
	lcm_dcs_write_seq_static(ctx,0x50,0x11);// pps000
	lcm_dcs_write_seq_static(ctx,0x51,0xab);// pps003
	lcm_dcs_write_seq_static(ctx,0x52,0x30);// pps004
	lcm_dcs_write_seq_static(ctx,0x53,0x09);// pps006
	lcm_dcs_write_seq_static(ctx,0x54,0x6c);// pps007
	lcm_dcs_write_seq_static(ctx,0x55,0x04);// pps008
	lcm_dcs_write_seq_static(ctx,0x56,0x38);// pps009
	lcm_dcs_write_seq_static(ctx,0x58,0x00);// pps010
	lcm_dcs_write_seq_static(ctx,0x59,0x0c);// pps011
	lcm_dcs_write_seq_static(ctx,0x5a,0x02);// pps012
	lcm_dcs_write_seq_static(ctx,0x5b,0x1c);// pps013
	lcm_dcs_write_seq_static(ctx,0x5c,0x01);// pps016
	lcm_dcs_write_seq_static(ctx,0x5d,0x9a);// pps017
	lcm_dcs_write_seq_static(ctx,0x5e,0x19);// pps021
	lcm_dcs_write_seq_static(ctx,0x5f,0x01);// pps022
	lcm_dcs_write_seq_static(ctx,0x60,0x03);// pps023
	lcm_dcs_write_seq_static(ctx,0x61,0x00);// pps024
	lcm_dcs_write_seq_static(ctx,0x62,0x0a);// pps025
	lcm_dcs_write_seq_static(ctx,0x63,0x0c);// pps027
	lcm_dcs_write_seq_static(ctx,0x64,0x08);// pps028
	lcm_dcs_write_seq_static(ctx,0x65,0xbb);// pps029
	lcm_dcs_write_seq_static(ctx,0x66,0x0a);// pps030
	lcm_dcs_write_seq_static(ctx,0x67,0x5f);// pps031
	lcm_dcs_write_seq_static(ctx,0x68,0x16);// pps032
	lcm_dcs_write_seq_static(ctx,0x69,0x00);// pps033
	lcm_dcs_write_seq_static(ctx,0x6a,0x10);// pps034
	lcm_dcs_write_seq_static(ctx,0x6b,0xec);// pps035
	lcm_dcs_write_seq_static(ctx,0x6c,0x07);// pps036
	lcm_dcs_write_seq_static(ctx,0x6d,0x10);// pps037
	lcm_dcs_write_seq_static(ctx,0x6e,0x20);// pps038
	lcm_dcs_write_seq_static(ctx,0x6f,0x00);// pps039
	lcm_dcs_write_seq_static(ctx,0x70,0x06);// pps040
	lcm_dcs_write_seq_static(ctx,0x71,0x0f);// pps041
	lcm_dcs_write_seq_static(ctx,0x72,0x0f);// pps042
	lcm_dcs_write_seq_static(ctx,0x73,0x33);// pps043
	lcm_dcs_write_seq_static(ctx,0x74,0x0e);// pps044
	lcm_dcs_write_seq_static(ctx,0x75,0x1c);// pps045
	lcm_dcs_write_seq_static(ctx,0x76,0x2a);// pps046
	lcm_dcs_write_seq_static(ctx,0x77,0x38);// pps047
	lcm_dcs_write_seq_static(ctx,0x78,0x46);// pps048
	lcm_dcs_write_seq_static(ctx,0x79,0x54);// pps049
	lcm_dcs_write_seq_static(ctx,0x7a,0x62);// pps050
	lcm_dcs_write_seq_static(ctx,0x7b,0x69);// pps051
	lcm_dcs_write_seq_static(ctx,0x7c,0x70);// pps052
	lcm_dcs_write_seq_static(ctx,0x7d,0x77);// pps053
	lcm_dcs_write_seq_static(ctx,0x7e,0x79);// pps054
	lcm_dcs_write_seq_static(ctx,0x7f,0x7b);// pps055
	lcm_dcs_write_seq_static(ctx,0x80,0x7d);// pps056
	lcm_dcs_write_seq_static(ctx,0x81,0x7e);// pps057
	lcm_dcs_write_seq_static(ctx,0x82,0x01);// pps058
	lcm_dcs_write_seq_static(ctx,0x83,0xc2);// pps059
	lcm_dcs_write_seq_static(ctx,0x84,0x22);// pps060
	lcm_dcs_write_seq_static(ctx,0x85,0x00);// pps061
	lcm_dcs_write_seq_static(ctx,0x86,0x2a);// pps062
	lcm_dcs_write_seq_static(ctx,0x87,0x40);// pps063
	lcm_dcs_write_seq_static(ctx,0x88,0x32);// pps064
	lcm_dcs_write_seq_static(ctx,0x89,0xbe);// pps065
	lcm_dcs_write_seq_static(ctx,0x8a,0x3a);// pps066
	lcm_dcs_write_seq_static(ctx,0x8b,0xfc);// pps067
	lcm_dcs_write_seq_static(ctx,0x8c,0x3a);// pps068
	lcm_dcs_write_seq_static(ctx,0x8d,0xfa);// pps069
	lcm_dcs_write_seq_static(ctx,0x8e,0x3a);// pps070
	lcm_dcs_write_seq_static(ctx,0x8f,0xf8);// pps071
	lcm_dcs_write_seq_static(ctx,0x90,0x3b);// pps072
	lcm_dcs_write_seq_static(ctx,0x91,0x38);// pps073
	lcm_dcs_write_seq_static(ctx,0x92,0x3b);// pps074
	lcm_dcs_write_seq_static(ctx,0x93,0x78);// pps075
	lcm_dcs_write_seq_static(ctx,0x94,0x3b);// pps076
	lcm_dcs_write_seq_static(ctx,0x95,0x76);// pps077
	lcm_dcs_write_seq_static(ctx,0x96,0x4b);// pps078
	lcm_dcs_write_seq_static(ctx,0x97,0xb6);// pps079
	lcm_dcs_write_seq_static(ctx,0x98,0x4b);// pps080
	lcm_dcs_write_seq_static(ctx,0x99,0xf6);// pps081
	lcm_dcs_write_seq_static(ctx,0x9a,0x4c);// pps082
	lcm_dcs_write_seq_static(ctx,0x9b,0x34);// pps083
	lcm_dcs_write_seq_static(ctx,0x9c,0x5c);// pps084
	lcm_dcs_write_seq_static(ctx,0x9d,0x74);// pps085
	lcm_dcs_write_seq_static(ctx,0x9e,0x8c);// pps086
	lcm_dcs_write_seq_static(ctx,0x9f,0xf4);// pps087
	lcm_dcs_write_seq_static(ctx,0xa2,0x02);// pps014
	lcm_dcs_write_seq_static(ctx,0xa3,0xa3);// pps015
	lcm_dcs_write_seq_static(ctx,0xa4,0x00);// pps088
	lcm_dcs_write_seq_static(ctx,0xa5,0x00);// pps089
	lcm_dcs_write_seq_static(ctx,0xa6,0x00);// pps090
	lcm_dcs_write_seq_static(ctx,0xa7,0x00);// pps091
	lcm_dcs_write_seq_static(ctx,0xa9,0x00);// pps092
	lcm_dcs_write_seq_static(ctx,0xaa,0x00);// pps093
	lcm_dcs_write_seq_static(ctx,0xa0,0xa0);// pps005
//drv-modify 10bit-pengzhipeng-20230429-end
	lcm_dcs_write_seq_static(ctx,0xFE,0xa1);
	lcm_dcs_write_seq_static(ctx,0xCA,0x80);
	lcm_dcs_write_seq_static(ctx,0xCD,0x00);
	lcm_dcs_write_seq_static(ctx,0xCE,0x00);

	lcm_dcs_write_seq_static(ctx,0xFE,0x74);
	lcm_dcs_write_seq_static(ctx,0xEC,0x11); 
	lcm_dcs_write_seq_static(ctx,0xED,0x11); 
	lcm_dcs_write_seq_static(ctx,0xEF,0x98); 
	lcm_dcs_write_seq_static(ctx,0xF0,0x98); 
	lcm_dcs_write_seq_static(ctx,0xF2,0x98); 
	lcm_dcs_write_seq_static(ctx,0xF3,0x98);

	lcm_dcs_write_seq_static(ctx,0xFE,0x00);
	lcm_dcs_write_seq_static(ctx,0xFA,0x01); 
	lcm_dcs_write_seq_static(ctx,0xC2,0x08);
	lcm_dcs_write_seq_static(ctx,0x35,0x00);
	//lcm_dcs_write_seq_static(ctx,0x51,0x0D,0xBB);//drv-To solve the problem that wake up from sleep will light up instantly-pengzhipeng-2022300510-start
	

	/*PRIZE:Added by lvyuanchuan,X9-678,20221230*/
	lcm_pannel_reconfig_blk(ctx);
	lcm_dcs_write_seq_static(ctx,0x11,0x00);
	mdelay(120);
	lcm_dcs_write_seq_static(ctx,0x29,0x00);
	//prize add by majiangtao for frequency Select 20230103 start
	mdelay(10);
#if 0    //prize add by yinhanhan for hbm, 20230311 start
	if (g_vrefresh != -1)
	{
		if (g_vrefresh == MODE_0_FPS) /*60HZ*/
		{
			printk("[panel] %s mode_switch MODE_0_FPS 60HZ\n",__func__);
			lcm_dcs_write_seq_static(ctx, 0xFE, 0x40);
			lcm_dcs_write_seq_static(ctx, 0xBD, 0x00);
			lcm_dcs_write_seq_static(ctx, 0xFE, 0x00);
		}
		else if (g_vrefresh == MODE_1_FPS) /*90HZ*/
		{
			printk("[panel] %s mode_switch MODE_1_FPS 90HZ\n",__func__);
			lcm_dcs_write_seq_static(ctx, 0xFE, 0x40);
			lcm_dcs_write_seq_static(ctx, 0xBD, 0x06);
			lcm_dcs_write_seq_static(ctx, 0xFE, 0x00);
		}
		else if (g_vrefresh == MODE_2_FPS) /*120HZ*/
		{
			printk("[panel] %s mode_switch MODE_2_FPS 120HZ\n",__func__);
			lcm_dcs_write_seq_static(ctx, 0xFE, 0x40);
			lcm_dcs_write_seq_static(ctx, 0xBD, 0x05);
			lcm_dcs_write_seq_static(ctx, 0xFE, 0x00);
		}
		else
		{
			printk("[panel] %s invalid g_vrefresh:%d\n",__func__, g_vrefresh);
		}
	}
	//prize add by majiangtao for frequency Select 20230103 end
#endif
}

static int lcm_disable(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);

	if (!ctx->enabled)
		return 0;
	pr_err("gezi----exit------%s-----%d\n",__func__,__LINE__);
	if (ctx->backlight) {
		ctx->backlight->props.power = FB_BLANK_POWERDOWN;
		backlight_update_status(ctx->backlight);
	}

	ctx->enabled = false;

	return 0;
}

static int lcm_unprepare(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);

	pr_err("gezi----------%s-----%d\n",__func__,__LINE__);

	if (!ctx->prepared)
		return 0;

	lcm_dcs_write_seq_static(ctx, 0x28);
	lcm_dcs_write_seq_static(ctx, 0x10);
	msleep(120);

	ctx->error = 0;
	ctx->prepared = false;

	pr_err("gezi------exit----%s-----%d\n",__func__,__LINE__);
	//reset
	ctx->reset_gpio =
		devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio)) {
		dev_err(ctx->dev, "%s: cannot get reset_gpio %ld\n",
			__func__, PTR_ERR(ctx->reset_gpio));
		return PTR_ERR(ctx->reset_gpio);
	}
	gpiod_set_value(ctx->reset_gpio, 0);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);

	udelay(3000);
	// 138   --  2.8
	ctx->bias_neg = devm_gpiod_get_index(ctx->dev,
		"bias", 1, GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->bias_neg)) {
		dev_err(ctx->dev, "%s: cannot get bias_neg %ld\n",
			__func__, PTR_ERR(ctx->bias_neg));
		return PTR_ERR(ctx->bias_neg);
	}
	gpiod_set_value(ctx->bias_neg, 0);
	devm_gpiod_put(ctx->dev, ctx->bias_neg);

	udelay(3000);

	pr_err("gezi----------%s-----%d\n",__func__,__LINE__);

	//137  -  1.2
	ctx->bias_pos = devm_gpiod_get_index(ctx->dev,
		"bias", 0, GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->bias_pos)) {
		dev_err(ctx->dev, "%s: cannot get bias_pos %ld\n",
			__func__, PTR_ERR(ctx->bias_pos));
		return PTR_ERR(ctx->bias_pos);
	}
	gpiod_set_value(ctx->bias_pos, 0);
	devm_gpiod_put(ctx->dev, ctx->bias_pos);

	udelay(2000);


	//prize add by wangfei for ldo 1.8 20210709 start
	ctx->pm_enable_gpio = devm_gpiod_get_index(ctx->dev,
		"pm-enable", 0, GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->pm_enable_gpio)) {
		dev_err(ctx->dev, "%s: cannot get pm_enable_gpio %ld\n",
			__func__, PTR_ERR(ctx->pm_enable_gpio));
		return PTR_ERR(ctx->pm_enable_gpio);
	}
	gpiod_set_value(ctx->pm_enable_gpio, 0);
	devm_gpiod_put(ctx->dev, ctx->pm_enable_gpio);
	//prize add by wangfei for ldo 1.8 20210709 end

	ctx->hbm_en = false;
	/*przie update hbm_stat X9LAVA-953 20230329*/
	ctx->hbm_stat = false;
	return 0;
}

static int lcm_prepare(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	int ret;

	pr_info("%s\n", __func__);

	if (ctx->prepared)
		return 0;

	//prize add by wangfei for ldo 1.8 20210709 start
	ctx->pm_enable_gpio = devm_gpiod_get_index(ctx->dev,
		"pm-enable", 0, GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->pm_enable_gpio)) {
		dev_err(ctx->dev, "%s: cannot get pm_enable_gpio %ld\n",
			__func__, PTR_ERR(ctx->pm_enable_gpio));
		return PTR_ERR(ctx->pm_enable_gpio);
	}
	gpiod_set_value(ctx->pm_enable_gpio, 1);
	devm_gpiod_put(ctx->dev, ctx->pm_enable_gpio);
	//prize add by wangfei for ldo 1.8 20210709 end
	udelay(3000);


	ctx->bias_pos = devm_gpiod_get_index(ctx->dev,
		"bias", 0, GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->bias_pos)) {
		dev_err(ctx->dev, "%s: cannot get bias_pos %ld\n",
			__func__, PTR_ERR(ctx->bias_pos));
		return PTR_ERR(ctx->bias_pos);
	}
	gpiod_set_value(ctx->bias_pos, 1);
	devm_gpiod_put(ctx->dev, ctx->bias_pos);

	udelay(3000);

	ctx->bias_neg = devm_gpiod_get_index(ctx->dev,
		"bias", 1, GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->bias_neg)) {
		dev_err(ctx->dev, "%s: cannot get bias_neg %ld\n",
			__func__, PTR_ERR(ctx->bias_neg));
		return PTR_ERR(ctx->bias_neg);
	}
	gpiod_set_value(ctx->bias_neg, 1);
	devm_gpiod_put(ctx->dev, ctx->bias_neg);



	lcm_panel_init(ctx);

	ret = ctx->error;
	if (ret < 0)
		lcm_unprepare(panel);

	ctx->prepared = true;

#if defined(CONFIG_MTK_PANEL_EXT)
	//mtk_panel_tch_rst(panel);
	pr_err("gezi----------%s-----%d\n",__func__,__LINE__);
#endif
#ifdef PANEL_SUPPORT_READBACK
	lcm_panel_get_data(ctx);
#endif
	return ret;
}

static int lcm_enable(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);

	if (ctx->enabled)
		return 0;

	if (ctx->backlight) {
		ctx->backlight->props.power = FB_BLANK_UNBLANK;
		backlight_update_status(ctx->backlight);
	}
	ctx->enabled = true;

	return 0;
}

#define VAC (2412)
#define HAC (1080)
static u32 fake_heigh = 2412;
static u32 fake_width = 1080;
static bool need_fake_resolution;

static const struct drm_display_mode switch_mode_120 = {
	.clock = 327060,
	.hdisplay = FRAME_WIDTH,
	.hsync_start = FRAME_WIDTH + MODE_2_HFP,
	.hsync_end = FRAME_WIDTH + MODE_2_HFP + HSA,
	.htotal = FRAME_WIDTH + MODE_2_HFP + HSA + HBP,
	.vdisplay = FRAME_HEIGHT,
	.vsync_start = FRAME_HEIGHT + MODE_2_VFP,
	.vsync_end = FRAME_HEIGHT + MODE_2_VFP + VSA,
	.vtotal = FRAME_HEIGHT + MODE_2_VFP + VSA + VBP,
	.vrefresh = MODE_2_FPS,
};


static const struct drm_display_mode switch_mode_90 = {
	.clock = 266090,
	.hdisplay = FRAME_WIDTH,
	.hsync_start = FRAME_WIDTH + MODE_1_HFP,
	.hsync_end = FRAME_WIDTH + MODE_1_HFP + HSA,
	.htotal = FRAME_WIDTH + MODE_1_HFP + HSA + HBP,
	.vdisplay = FRAME_HEIGHT,
	.vsync_start = FRAME_HEIGHT + MODE_1_VFP,
	.vsync_end = FRAME_HEIGHT + MODE_1_VFP + VSA,
	.vtotal = FRAME_HEIGHT + MODE_1_VFP + VSA + VBP,
	.vrefresh = MODE_1_FPS,
};

static const struct drm_display_mode default_mode = {
	.clock = 227090,
	.hdisplay = FRAME_WIDTH,
	.hsync_start = FRAME_WIDTH + MODE_0_HFP,
	.hsync_end = FRAME_WIDTH + MODE_0_HFP + HSA,
	.htotal = FRAME_WIDTH + MODE_0_HFP + HSA + HBP,
	.vdisplay = FRAME_HEIGHT,
	.vsync_start = FRAME_HEIGHT + MODE_0_VFP,
	.vsync_end = FRAME_HEIGHT + MODE_0_VFP + VSA,
	.vtotal = FRAME_HEIGHT + MODE_0_VFP + VSA + VBP,
	.vrefresh = MODE_0_FPS,
};

#if defined(CONFIG_MTK_PANEL_EXT)
static int panel_ext_reset(struct drm_panel *panel, int on)
{
	struct lcm *ctx = panel_to_lcm(panel);

	ctx->reset_gpio =
		devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio)) {
		dev_err(ctx->dev, "%s: cannot get reset_gpio %ld\n",
			__func__, PTR_ERR(ctx->reset_gpio));
		return PTR_ERR(ctx->reset_gpio);
	}
	gpiod_set_value(ctx->reset_gpio, on);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);

	return 0;
}

static int panel_ata_check(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	struct gpio_desc *id2_gpio = NULL;
	//struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	//unsigned char data[3] = {0x00, 0x00, 0x00};
	//unsigned char id[3] = {0x0, 0x81, 0x0};
	ssize_t ret;
	pr_err("panel----exit------%s-----%d\n",__func__,__LINE__);

	// prize baibo for lcm ata test begin
#if 0
	ret = mipi_dsi_dcs_read(dsi, 0x4, data, 3);
	if (ret < 0) {
		pr_err("%s error\n", __func__);
		return 0;
	}

	DDPINFO("ATA read data %x %x %x\n", data[0], data[1], data[2]);

	if (data[0] == id[0] &&
			data[1] == id[1] &&
			data[2] == id[2])
		return 1;

	DDPINFO("ATA expect read data is %x %x %x\n",
			id[0], id[1], id[2]);
#endif

	id2_gpio = devm_gpiod_get(ctx->dev, "id2", GPIOD_IN);
	if (IS_ERR(id2_gpio)) {
		dev_err(ctx->dev, "%s: cannot get id2_gpio %ld\n",
			__func__, PTR_ERR(id2_gpio));
		return 0;
	}

	if (gpiod_get_value(id2_gpio)) {
		pr_err("%s %d id2 value 1\n",__func__,__LINE__);
		ret = 0;
	}else{
		pr_err("%s %d id2 value 0\n",__func__,__LINE__);
		ret = 1;
	}
	devm_gpiod_put(ctx->dev, id2_gpio);

	return ret;
	// prize baibo for lcm ata test end
}

extern void prize_common_node_show_register(char* name,bool(*hbm_set)(void));
bool get_hbmstate(void)
{
	printk("%s g_ctx->hbm_stat:%d",__func__, g_ctx->hbm_stat);
	return g_ctx->hbm_stat;
}

static int lcm_setbacklight_cmdq(void *dsi, dcs_write_gce cb,
	void *handle, unsigned int level)
{
	 char bl_tb0[] = {0x51,0x07,0xFF};
	 char hbm_tb[] = {0x51,0x0F,0xFF};
	 unsigned int level_normal = 125;
	 unsigned int reg_level = 125;
	 if(level > 259) /* prize modified by gongtaitao for x9 lava hbm mode 20230421 */
	 {
	 	if(level == 260)
	 	{
	 		printk("panel into HBM\n");
			if (!cb)
				return -1;
			g_ctx->hbm_stat = true;
			cb(dsi, handle, hbm_tb, ARRAY_SIZE(hbm_tb));
	 	}
	 	else if(level == 270)
	 	{
	 		/*PRIZE:Added by lvyuanchuan,X9-534,20230103*/
			//level_normal = bl_level * BLK_LEVEL_MAP3/255 + BLK_LEVEL_OFFSET;
			/*PRIZE:modify by durunshen,MT6877-98,20230530*/
			level_normal = Gamma_to_level[bl_level] + BLK_LEVEL_OFFSET;
			bl_tb0[1] = (level_normal>>8)&0xf;
			bl_tb0[2] = (level_normal)&0xff;
			if (!cb)
				return -1;
			printk("panel out HBM bl_level = %d\n",bl_level);
			g_ctx->hbm_stat = false;
			cb(dsi, handle, bl_tb0, ARRAY_SIZE(bl_tb0));
	 	}
	 }
	 else
	 {
	 	/*PRIZE:Added by lvyuanchuan,X9-534,20230103 */
	 	if(level){
			reg_level = Gamma_to_level[level] + BLK_LEVEL_OFFSET;
			bl_level = level; //PRIZE:modify by durunshen,X9-1080,20230301
		}
		else
			reg_level = 0;
        g_current_level = level;
		bl_tb0[1] = (reg_level>>8)&0xf;
		bl_tb0[2] = (reg_level)&0xff;
		pr_err("level{ %d - %d },bl_tb0[1] = %d,bl_tb0[2] = %d\n",level,reg_level,bl_tb0[1],bl_tb0[2]);
		if (!cb)
			return -1;
		if(g_ctx->hbm_stat == false || level == 0)//modify by zhangchao for X9LAVA-539
			cb(dsi, handle, bl_tb0, ARRAY_SIZE(bl_tb0));
		/*PRIZE:Added by lvyuanchuan,X9-678,20221230*/
	}
	 return 0;
}


//prize add by gongtaitao for sensorhub get backlight 20221028 start
unsigned short led_level_disp_get(char *name)
{
    int trans_level = 0;
	trans_level = Gamma_to_level[g_current_level];
	pr_err("[%s]: name: %s, level : %d",__func__, name, trans_level);
	return trans_level;
}
EXPORT_SYMBOL(led_level_disp_get);
//prize add by gongtaitao for sensorhub get backlight 20221028 end



static int panel_hbm_set_cmdq(struct drm_panel *panel, void *dsi,
			      dcs_write_gce cb, void *handle, bool en)
{
	unsigned int level_hbm = 255;
	unsigned int level_normal = 125;
	char normal_tb0[] = {0x51, 0x07,0xFF};
	char hbm_tb[] = {0x51,0x0F,0xFF};
	struct lcm *ctx = panel_to_lcm(panel);

	if (!cb)
		return -1;

	//if (ctx->hbm_en == en)
	//	goto done;

	if (en)
	{
		printk("[panel] %s : set HBM\n",__func__);
	#if 0
		lcm_dcs_write_seq_static(ctx,0xF0,0x55, 0xAA, 0x52, 0x08, 0x00);   //ELVSS
		udelay(100);
		lcm_dcs_write_seq_static(ctx,0xB5,0x80,0x80);
		lcm_dcs_write_seq_static(ctx,0x6F,0x07);
		lcm_dcs_write_seq_static(ctx,0xB5,0x1D);
	#endif
		/*PRIZE:Added by durunshen,X9-677,20230106 start*/
		g_ctx->hbm_stat = true;
		/*PRIZE:Added by durunshen,X9-677,20230106 end*/
		cb(dsi, handle, hbm_tb, ARRAY_SIZE(hbm_tb));
	}
	else
	{
		printk("[panel] %s : set normal = %d\n",__func__,bl_level);
		/*PRIZE:Added by lvyuanchuan,X9-534,20230103*/
		level_normal = bl_level * BLK_LEVEL_MAP3/259 + BLK_LEVEL_OFFSET; /* prize modified by gongtaitao for x9 lava hbm mode 20230421 */
		normal_tb0[1] = (level_normal>>8)&0xff;
		normal_tb0[2] = (level_normal)&0xff;
	#if 0
		lcm_dcs_write_seq_static(ctx,0xF0,0x55, 0xAA, 0x52, 0x08, 0x00);   //ELVSS
		udelay(100);
		lcm_dcs_write_seq_static(ctx,0xB5,0x80,0x80);
		lcm_dcs_write_seq_static(ctx,0x6F,0x07);
		lcm_dcs_write_seq_static(ctx,0xB5,0x23);
	#endif
		/*PRIZE:Added by durunshen,X9-677,20230106 start*/
		g_ctx->hbm_stat = false;
		/*PRIZE:Added by durunshen,X9-677,20230106 end*/
		cb(dsi, handle, normal_tb0, ARRAY_SIZE(normal_tb0));
	}

	ctx->hbm_en = en;
	ctx->hbm_wait = true;

 done:
	return 0;
}

static void panel_hbm_get_state(struct drm_panel *panel, bool *state)
{
	struct lcm *ctx = panel_to_lcm(panel);

	*state = ctx->hbm_en;
}

static int lcm_get_virtual_heigh(void)
{
	return VAC;
}

static int lcm_get_virtual_width(void)
{
	return HAC;
}

static struct mtk_panel_params ext_params_120 = {
	// .vfp_low_power = 743,
	.cust_esd_check = 1,
	.esd_check_enable = 1,
	.lcm_esd_check_table[0] = {
		.cmd = 0x0B,
		.count = 1,
		.para_list[0] = 0x00,
	},
	.lcm_esd_check_table[1] = {
		.cmd = 0x0A,
		.count = 1,
		.para_list[0] = 0x9c,
	},
	.lcm_esd_check_table[2] = {
		.cmd = 0xFA,
		.count = 1,
		.para_list[0] = 0x01,
	},
	/*drv-During the ESD call process, the MIPI may have an Error report callback action, 
		  which may result in an error during the first read. Here, read the 0x0B register first, 
		  but ignore this read back value. Do not use the 0x0B read back value as a reference for recovery, 
		  and use the subsequent 0x0A and 0xFA read back values as a reference for recovery-pengzhipeng-20230324-end*/
	.physical_width_um = PHYSICAL_WIDTH,
	.physical_height_um = PHYSICAL_HEIGHT,
	.dsc_params = {
			.enable = 1,
			.bdg_dsc_enable = 0,
			.ver                   =  DSC_VER,
			.slice_mode            =  DSC_SLICE_MODE,
			.rgb_swap              =  DSC_RGB_SWAP,
			.dsc_cfg               =  DSC_DSC_CFG,
			.rct_on                =  DSC_RCT_ON,
			.bit_per_channel       =  DSC_BIT_PER_CHANNEL,
			.dsc_line_buf_depth    =  DSC_DSC_LINE_BUF_DEPTH,
			.bp_enable             =  DSC_BP_ENABLE,
			.bit_per_pixel         =  DSC_BIT_PER_PIXEL,
			.pic_height            =  FRAME_HEIGHT,
			.pic_width             =  FRAME_WIDTH,
			.slice_height          =  DSC_SLICE_HEIGHT,
			.slice_width           =  DSC_SLICE_WIDTH,
			.chunk_size            =  DSC_CHUNK_SIZE,
			.xmit_delay            =  DSC_XMIT_DELAY,
			.dec_delay             =  DSC_DEC_DELAY,
			.scale_value           =  DSC_SCALE_VALUE,
			.increment_interval    =  DSC_INCREMENT_INTERVAL,
			.decrement_interval    =  DSC_DECREMENT_INTERVAL,
			.line_bpg_offset       =  DSC_LINE_BPG_OFFSET,
			.nfl_bpg_offset        =  DSC_NFL_BPG_OFFSET,
			.slice_bpg_offset      =  DSC_SLICE_BPG_OFFSET,
			.initial_offset        =  DSC_INITIAL_OFFSET,
			.final_offset          =  DSC_FINAL_OFFSET,
			.flatness_minqp        =  DSC_FLATNESS_MINQP,
			.flatness_maxqp        =  DSC_FLATNESS_MAXQP,
			.rc_model_size         =  DSC_RC_MODEL_SIZE,
			.rc_edge_factor        =  DSC_RC_EDGE_FACTOR,
			.rc_quant_incr_limit0  =  DSC_RC_QUANT_INCR_LIMIT0,
			.rc_quant_incr_limit1  =  DSC_RC_QUANT_INCR_LIMIT1,
			.rc_tgt_offset_hi      =  DSC_RC_TGT_OFFSET_HI,
			.rc_tgt_offset_lo      =  DSC_RC_TGT_OFFSET_LO,
		},

	.data_rate = MODE_2_DATA_RATE,

};

static struct mtk_panel_params ext_params_90 = {
	// .vfp_low_power = 743,
	.cust_esd_check = 1,
	.esd_check_enable = 1,
	/*drv-During the ESD call process, the MIPI may have an Error report callback action, 
		  which may result in an error during the first read. Here, read the 0x0B register first, 
		  but ignore this read back value. Do not use the 0x0B read back value as a reference for recovery, 
		  and use the subsequent 0x0A and 0xFA read back values as a reference for recovery-pengzhipeng-20230324-start*/
	.lcm_esd_check_table[0] = {
		.cmd = 0x0B,
		.count = 1,
		.para_list[0] = 0x00,
	},
	.lcm_esd_check_table[1] = {
		.cmd = 0x0A,
		.count = 1,
		.para_list[0] = 0x9c,
	},
	.lcm_esd_check_table[2] = {
		.cmd = 0xFA,
		.count = 1,
		.para_list[0] = 0x01,
	},
	/*drv-During the ESD call process, the MIPI may have an Error report callback action, 
		  which may result in an error during the first read. Here, read the 0x0B register first, 
		  but ignore this read back value. Do not use the 0x0B read back value as a reference for recovery, 
		  and use the subsequent 0x0A and 0xFA read back values as a reference for recovery-pengzhipeng-20230324-end*/
	.physical_width_um = PHYSICAL_WIDTH,
	.physical_height_um = PHYSICAL_HEIGHT,
	.dsc_params = {
			.enable = 1,
			.bdg_dsc_enable = 0,
			.ver                   =  DSC_VER,
			.slice_mode            =  DSC_SLICE_MODE,
			.rgb_swap              =  DSC_RGB_SWAP,
			.dsc_cfg               =  DSC_DSC_CFG,
			.rct_on                =  DSC_RCT_ON,
			.bit_per_channel       =  DSC_BIT_PER_CHANNEL,
			.dsc_line_buf_depth    =  DSC_DSC_LINE_BUF_DEPTH,
			.bp_enable             =  DSC_BP_ENABLE,
			.bit_per_pixel         =  DSC_BIT_PER_PIXEL,
			.pic_height            =  FRAME_HEIGHT,
			.pic_width             =  FRAME_WIDTH,
			.slice_height          =  DSC_SLICE_HEIGHT,
			.slice_width           =  DSC_SLICE_WIDTH,
			.chunk_size            =  DSC_CHUNK_SIZE,
			.xmit_delay            =  DSC_XMIT_DELAY,
			.dec_delay             =  DSC_DEC_DELAY,
			.scale_value           =  DSC_SCALE_VALUE,
			.increment_interval    =  DSC_INCREMENT_INTERVAL,
			.decrement_interval    =  DSC_DECREMENT_INTERVAL,
			.line_bpg_offset       =  DSC_LINE_BPG_OFFSET,
			.nfl_bpg_offset        =  DSC_NFL_BPG_OFFSET,
			.slice_bpg_offset      =  DSC_SLICE_BPG_OFFSET,
			.initial_offset        =  DSC_INITIAL_OFFSET,
			.final_offset          =  DSC_FINAL_OFFSET,
			.flatness_minqp        =  DSC_FLATNESS_MINQP,
			.flatness_maxqp        =  DSC_FLATNESS_MAXQP,
			.rc_model_size         =  DSC_RC_MODEL_SIZE,
			.rc_edge_factor        =  DSC_RC_EDGE_FACTOR,
			.rc_quant_incr_limit0  =  DSC_RC_QUANT_INCR_LIMIT0,
			.rc_quant_incr_limit1  =  DSC_RC_QUANT_INCR_LIMIT1,
			.rc_tgt_offset_hi      =  DSC_RC_TGT_OFFSET_HI,
			.rc_tgt_offset_lo      =  DSC_RC_TGT_OFFSET_LO,
		},
	.data_rate = MODE_2_DATA_RATE,

};


static struct mtk_panel_params ext_params = {
	// .pll_clk = 373,
	// .vfp_low_power = 743,
	.cust_esd_check = 1,
	.esd_check_enable = 1,
	/*drv-During the ESD call process, the MIPI may have an Error report callback action, 
		  which may result in an error during the first read. Here, read the 0x0B register first, 
		  but ignore this read back value. Do not use the 0x0B read back value as a reference for recovery, 
		  and use the subsequent 0x0A and 0xFA read back values as a reference for recovery-pengzhipeng-20230324-start*/
	.lcm_esd_check_table[0] = {
		.cmd = 0x0B,
		.count = 1,
		.para_list[0] = 0x00,
	},
	.lcm_esd_check_table[1] = {
		.cmd = 0x0A,
		.count = 1,
		.para_list[0] = 0x9c,
	},
	.lcm_esd_check_table[2] = {
		.cmd = 0xFA,
		.count = 1,
		.para_list[0] = 0x01,
	},
	/*drv-During the ESD call process, the MIPI may have an Error report callback action, 
		  which may result in an error during the first read. Here, read the 0x0B register first, 
		  but ignore this read back value. Do not use the 0x0B read back value as a reference for recovery, 
		  and use the subsequent 0x0A and 0xFA read back values as a reference for recovery-pengzhipeng-20230324-end*/
	.physical_width_um = PHYSICAL_WIDTH,
	.physical_height_um = PHYSICAL_HEIGHT,
	.dsc_params = {
			.enable = 1,
			.bdg_dsc_enable = 0,
			.ver                   =  DSC_VER,
			.slice_mode            =  DSC_SLICE_MODE,
			.rgb_swap              =  DSC_RGB_SWAP,
			.dsc_cfg               =  DSC_DSC_CFG,
			.rct_on                =  DSC_RCT_ON,
			.bit_per_channel       =  DSC_BIT_PER_CHANNEL,
			.dsc_line_buf_depth    =  DSC_DSC_LINE_BUF_DEPTH,
			.bp_enable             =  DSC_BP_ENABLE,
			.bit_per_pixel         =  DSC_BIT_PER_PIXEL,
			.pic_height            =  FRAME_HEIGHT,
			.pic_width             =  FRAME_WIDTH,
			.slice_height          =  DSC_SLICE_HEIGHT,
			.slice_width           =  DSC_SLICE_WIDTH,
			.chunk_size            =  DSC_CHUNK_SIZE,
			.xmit_delay            =  DSC_XMIT_DELAY,
			.dec_delay             =  DSC_DEC_DELAY,
			.scale_value           =  DSC_SCALE_VALUE,
			.increment_interval    =  DSC_INCREMENT_INTERVAL,
			.decrement_interval    =  DSC_DECREMENT_INTERVAL,
			.line_bpg_offset       =  DSC_LINE_BPG_OFFSET,
			.nfl_bpg_offset        =  DSC_NFL_BPG_OFFSET,
			.slice_bpg_offset      =  DSC_SLICE_BPG_OFFSET,
			.initial_offset        =  DSC_INITIAL_OFFSET,
			.final_offset          =  DSC_FINAL_OFFSET,
			.flatness_minqp        =  DSC_FLATNESS_MINQP,
			.flatness_maxqp        =  DSC_FLATNESS_MAXQP,
			.rc_model_size         =  DSC_RC_MODEL_SIZE,
			.rc_edge_factor        =  DSC_RC_EDGE_FACTOR,
			.rc_quant_incr_limit0  =  DSC_RC_QUANT_INCR_LIMIT0,
			.rc_quant_incr_limit1  =  DSC_RC_QUANT_INCR_LIMIT1,
			.rc_tgt_offset_hi      =  DSC_RC_TGT_OFFSET_HI,
			.rc_tgt_offset_lo      =  DSC_RC_TGT_OFFSET_LO,
		},
	.data_rate = MODE_2_DATA_RATE,

};


struct drm_display_mode *get_mode_by_id(struct drm_panel *panel,
	unsigned int mode)
{
	struct drm_display_mode *m;
	unsigned int i = 0;

	list_for_each_entry(m, &panel->connector->modes, head) {
		if (i == mode)
			return m;
		i++;
	}
	return NULL;
}

static int mtk_panel_ext_param_set(struct drm_panel *panel,
			 unsigned int mode)
{
	struct mtk_panel_ext *ext = find_panel_ext(panel);
	int ret = 0;
	struct drm_display_mode *m = get_mode_by_id(panel, mode);
	printk("[panel] %s,vrefresh = %d\n",__func__,m->vrefresh);
	if (m->vrefresh == MODE_0_FPS)
		ext->params = &ext_params;
	else if (m->vrefresh == MODE_1_FPS)
		ext->params = &ext_params_90;
	else if (m->vrefresh == MODE_2_FPS)
		ext->params = &ext_params_120;
	else
		ret = 1;

	return ret;
}

static void mode_switch_to_120(struct drm_panel *panel,
	enum MTK_PANEL_MODE_SWITCH_STAGE stage)
{
	struct lcm *ctx = panel_to_lcm(panel);

	if (stage == BEFORE_DSI_POWERDOWN) {
		printk("[panel] %s\n",__func__);
			lcm_dcs_write_seq_static(ctx, 0xFE, 0x40);
			lcm_dcs_write_seq_static(ctx, 0xBD, 0x05);
			lcm_dcs_write_seq_static(ctx, 0xFE, 0x00);
	}
}

static void mode_switch_to_90(struct drm_panel *panel,
	enum MTK_PANEL_MODE_SWITCH_STAGE stage)
{
	struct lcm *ctx = panel_to_lcm(panel);

	if (stage == BEFORE_DSI_POWERDOWN) {
		printk("[panel] %s\n",__func__);
			lcm_dcs_write_seq_static(ctx, 0xFE, 0x40);
			lcm_dcs_write_seq_static(ctx, 0xBD, 0x06);
			lcm_dcs_write_seq_static(ctx, 0xFE, 0x00);
	}
}

static void mode_switch_to_60(struct drm_panel *panel,
	enum MTK_PANEL_MODE_SWITCH_STAGE stage)
{
	struct lcm *ctx = panel_to_lcm(panel);

	if (stage == BEFORE_DSI_POWERDOWN) {
		printk("[panel] %s\n",__func__);
			lcm_dcs_write_seq_static(ctx, 0xFE, 0x40);
			lcm_dcs_write_seq_static(ctx, 0xBD, 0x00);
			lcm_dcs_write_seq_static(ctx, 0xFE, 0x00);
	}
}

static int mode_switch(struct drm_panel *panel, unsigned int cur_mode,
		unsigned int dst_mode, enum MTK_PANEL_MODE_SWITCH_STAGE stage)
{
	int ret = 0;
	struct drm_display_mode *m = get_mode_by_id(panel, dst_mode);
	printk("[panel] %s,cur_mode = %d,dst_mode = %d\n",__func__,cur_mode,dst_mode);
	if (cur_mode == dst_mode)
		return ret;

	if (m->vrefresh == MODE_0_FPS) { /*switch to 60 */
		mode_switch_to_60(panel, stage);
	} else if (m->vrefresh == MODE_1_FPS) { /*switch to 90 */
		mode_switch_to_90(panel, stage);
	} else if (m->vrefresh == MODE_2_FPS) { /*switch to 120 */
		mode_switch_to_120(panel, stage);
	} else
		ret = 1;
	//prize add by majiangtao for frequency Select 20230103 start
	g_vrefresh = m->vrefresh;
	//prize add by majiangtao for frequency Select 20230103 end
	return ret;
}
/*PRIZE:added by lvyuanchuan,x9-750,20230110 start*/
static int panel_doze_enable_start(struct drm_panel *panel,
	void *dsi, dcs_write_gce cb, void *handle)
{
	struct lcm *ctx = panel_to_lcm(panel);

	pr_info("panel %s\n", __func__);
	panel_ext_reset(panel, 0);
	usleep_range(10 * 1000, 15 * 1000);
	panel_ext_reset(panel, 1);

	lcm_dcs_write_seq_static(ctx, 0x28);
	lcm_dcs_write_seq_static(ctx, 0x10);
	msleep(120);
	return 0;
}

//drv-Solve the screen display and touch function-pengzhipeng-20230707-start
extern void prize_syna_dev_suspend(void);
//drv-Solve the screen display and touch function-pengzhipeng-20230707-end
static int panel_doze_enable(struct drm_panel *panel,
	void *dsi, dcs_write_gce cb, void *handle)
{
	struct lcm *ctx = panel_to_lcm(panel);

	pr_info("panel %s\n", __func__);
	lcm_dcs_write_seq_static(ctx, 0xFE, 0x40);
	/*Inter power on*/
	lcm_dcs_write_seq_static(ctx, 0xB3, 0x50);

	/*Page00*/
	lcm_dcs_write_seq_static(ctx, 0xFE, 0x00);
	/*Idle mode on*/
	lcm_dcs_write_seq_static(ctx, 0x39);
	/*Enter AOD 30nit*/
	lcm_dcs_write_seq_static(ctx, 0x51, 0x09, 0x56);
	//drv-Solve the screen display and touch function-pengzhipeng-20230707-start
	prize_syna_dev_suspend();
	//drv-Solve the screen display and touch function-pengzhipeng-20230707-end
	
	return 0;
}

static int panel_doze_disable(struct drm_panel *panel,
	void *dsi, dcs_write_gce cb, void *handle)
{
	struct lcm *ctx = panel_to_lcm(panel);

	pr_info("panel %s\n", __func__);
	lcm_dcs_write_seq_static(ctx, 0xFE, 0x40);
	/*Inter power on*/
	lcm_dcs_write_seq_static(ctx, 0xB3, 0x4F);
	/*Page00*/
	lcm_dcs_write_seq_static(ctx, 0xFE, 0x00);
	/*Idle mode off*/
	lcm_dcs_write_seq_static(ctx, 0x38);


	return 0;
}

static int panel_set_aod_light_mode(void *dsi,
	dcs_write_gce cb, void *handle, unsigned int mode)
{
	int i = 0;

	pr_info("panel %s\n", __func__);

	if (mode >= 1) {
		/*Enter AOD 50nit*/
		pr_info("panel %s Enter AOD 50nit\n", __func__);
		lcm_dcs_write_seq_static(g_ctx, 0x51, 0x09, 0x56);
	} else {
		/*Enter AOD 30nit*/
		pr_info("panel %s Enter AOD 30nit\n", __func__);
		lcm_dcs_write_seq_static(g_ctx, 0x51, 0x00, 0x05);
	}
	pr_info("%s : %d !\n", __func__, mode);

	return 0;
}
/*PRIZE:added by lvyuanchuan,x9-750,20230110 end*/

static struct mtk_panel_funcs ext_funcs = {
	.reset = panel_ext_reset,
	.set_backlight_cmdq = lcm_setbacklight_cmdq,
	.ata_check = panel_ata_check,
	.hbm_set_cmdq = panel_hbm_set_cmdq,
	.hbm_get_state = panel_hbm_get_state,
	.get_virtual_heigh = lcm_get_virtual_heigh,
	.get_virtual_width = lcm_get_virtual_width,
	.ext_param_set = mtk_panel_ext_param_set,
	.mode_switch = mode_switch,
	/*aod mode*/
	.doze_enable = panel_doze_enable,
	//.doze_enable_start = panel_doze_enable_start,
	.doze_disable = panel_doze_disable,
	.set_aod_light_mode = panel_set_aod_light_mode,
};
#endif

struct panel_desc {
	const struct drm_display_mode *modes;
	unsigned int num_modes;

	unsigned int bpc;

	struct {
		unsigned int width;
		unsigned int height;
	} size;

	struct {
		unsigned int prepare;
		unsigned int enable;
		unsigned int disable;
		unsigned int unprepare;
	} delay;
};
/*
static void change_drm_disp_mode_params(struct drm_display_mode *mode)
{
	if (fake_heigh > 0 && fake_heigh < VAC) {
		mode->vdisplay = fake_heigh;
		mode->vsync_start = fake_heigh + VFP;
		mode->vsync_end = fake_heigh + VFP + VSA;
		mode->vtotal = fake_heigh + VFP + VSA + VBP;
	}
	if (fake_width > 0 && fake_width < HAC) {
		mode->hdisplay = fake_width;
		mode->hsync_start = fake_width + HFP;
		mode->hsync_end = fake_width + HFP + HSA;
		mode->htotal = fake_width + HFP + HSA + HBP;
	}
}
*/
static int lcm_get_modes(struct drm_panel *panel)
{
	struct drm_display_mode *mode;
	struct drm_display_mode *mode_1;
	struct drm_display_mode *mode_2;

	//if (need_fake_resolution)
	//	change_drm_disp_mode_params(&default_mode);
	mode = drm_mode_duplicate(panel->drm, &default_mode);
	if (!mode) {
		dev_err(panel->drm->dev, "failed to add mode %ux%ux@%u\n",
			default_mode.hdisplay, default_mode.vdisplay,
			default_mode.vrefresh);
		return -ENOMEM;
	}
	drm_mode_set_name(mode);
	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	drm_mode_probed_add(panel->connector, mode);
	printk("[panel] %s,111\n",__func__);

	mode_1 = drm_mode_duplicate(panel->drm, &switch_mode_90);
	if (!mode_1) {
		dev_err(panel->drm->dev, "failed to add mode_1 %ux%ux@%u\n",
			switch_mode_90.hdisplay, switch_mode_90.vdisplay,
			switch_mode_90.vrefresh);
		return -ENOMEM;
	}
	drm_mode_set_name(mode_1);
	mode_1->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	drm_mode_probed_add(panel->connector, mode_1);
	printk("[panel] %s,222\n",__func__);

	mode_2 = drm_mode_duplicate(panel->drm, &switch_mode_120);
	if (!mode_2) {
		dev_err(panel->drm->dev, "failed to add mode_2 %ux%ux@%u\n",
			switch_mode_120.hdisplay, switch_mode_120.vdisplay,
			switch_mode_120.vrefresh);
		return -ENOMEM;
	}
	drm_mode_set_name(mode_2);
	mode_2->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	drm_mode_probed_add(panel->connector, mode_2);
	printk("[panel] %s,333\n",__func__);

	panel->connector->display_info.width_mm = 64;
	panel->connector->display_info.height_mm = 129;

	return 1;
}

static const struct drm_panel_funcs lcm_drm_funcs = {
	.disable = lcm_disable,
	.unprepare = lcm_unprepare,
	.prepare = lcm_prepare,
	.enable = lcm_enable,
	.get_modes = lcm_get_modes,
};

static void check_is_need_fake_resolution(struct device *dev)
{
	unsigned int ret = 0;

	ret = of_property_read_u32(dev->of_node, "fake_heigh", &fake_heigh);
	if (ret)
		need_fake_resolution = false;
	ret = of_property_read_u32(dev->of_node, "fake_width", &fake_width);
	if (ret)
		need_fake_resolution = false;
	if (fake_heigh > 0 && fake_heigh < VAC)
		need_fake_resolution = true;
	if (fake_width > 0 && fake_width < HAC)
		need_fake_resolution = true;
	pr_err("%s------need_fake_resolution = %d------%d\n", __func__,need_fake_resolution,__LINE__);
}
/* prize modified by gongtaitao for x9 lava hbm mode 20230421 start */
static void lcm_check_level_work(struct work_struct *work)
{
	static unsigned int hbm_times = 0;
	static bool limit_max_level_flags = false;
	static unsigned int limit_hbm_times = 0;
	static unsigned int abnormal_times = 0;
	static bool hbm_flags = false;

	pr_err("%s g_current_level:%u, limit_max_level_flags:%d, hbm_times:%u, limit_hbm_times:%u, g_max_set_level:%u, abnormal_times:%u, hbm_flags:%d\n", __func__,
		g_current_level, limit_max_level_flags, hbm_times, limit_hbm_times, g_max_set_level, abnormal_times, hbm_flags);

	if (!limit_max_level_flags) {
		if (g_current_level > NORMAL_MAX_LEVEL && g_current_level <= HBM_MAX_LEVEL) {
			hbm_flags = true;
			if (++hbm_times >= CONTINUOUS_HBM_TIMES) {
				g_max_set_level = NORMAL_MAX_LEVEL;
				hbm_times = 0;
				limit_max_level_flags = true;
				abnormal_times = 0;
				hbm_flags = false;
				pr_err("%s limit_max_level 255\n", __func__);
				prize_mt_leds_set_max_brightness("lcd-backlight", NORMAL_MAX_LEVEL, 1);
			}
		} else if (hbm_flags && g_current_level <= NORMAL_MAX_LEVEL) {
			if (++abnormal_times > NORMAL_TIMES) {
				hbm_times = 0;
				abnormal_times = 0;
				hbm_flags = false;
			}
		}
	} else {
		if (++limit_hbm_times >= CONTINUOUS_NORMAL_TIMES) {
			limit_max_level_flags = false;
			limit_hbm_times = 0;
			g_max_set_level = HBM_MAX_LEVEL;
			pr_err("%s limit_max_level 259\n", __func__);
			prize_mt_leds_set_max_brightness("lcd-backlight", HBM_MAX_LEVEL, 0);
		}
	}
}

static void lcm_check_level_timer(struct timer_list *list)
{
	schedule_work(&check_level_worker);

	mod_timer(&check_level_timer,
		jiffies +  msecs_to_jiffies(3000));
}
/* prize modified by gongtaitao for x9 lava hbm mode 20230421 end */
/* drv-add oled sysfs-pengzhipeng-20230306-start */
static ssize_t oled_show(struct device *dev,
                   struct device_attribute *attr, char *buf)
{
   int count = 0;
   
   count = snprintf(buf, PAGE_SIZE, "oled screens:%s\n",
					g_ctx->oled_screen ? "yes" : "no");  
   return count;
}
 
static ssize_t oled_store(struct device *dev,
           struct device_attribute *attr, const char *buf, size_t size)
{
    return size;
}
 
static DEVICE_ATTR(oled, 0664, oled_show, oled_store);

int oled_sysfs_add(struct platform_device *pdev)
{
	int err = 0;
    pr_err("Add gezi device attr groups,gezi_sysfs_add\n");
  	err = device_create_file(&pdev->dev, &dev_attr_oled);
	if (err) {
        pr_err("sys file creation failed\n");
        return -ENODEV;
	}
	return 0;
}

static const struct of_device_id oled_of_match[] = {
	{.compatible = "mediatek,oled",},
	{},
};
MODULE_DEVICE_TABLE(of, oled_of_match);


static int gesture_probe(struct platform_device *pdev)
{

	printk("%s\n",__func__);
	oled_sysfs_add(pdev);
	return 0;
}


static struct platform_driver oled_driver = {
	.probe = gesture_probe,
	.driver = {
		   .name = "oled",
		   .of_match_table = oled_of_match,
	},
};

int oled_init(void)
{
	printk("%s\n",__func__);
	return platform_driver_register(&oled_driver);
}
/* drv-add oled sysfs-pengzhipeng-20230306-end */

static int lcm_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct lcm *ctx;
	struct device_node *backlight;
	int ret;
	struct device_node *dsi_node, *remote_node = NULL, *endpoint = NULL;

	pr_err("gezi ---------%d-----\n",__LINE__);

	dsi_node = of_get_parent(dev->of_node);
	if (dsi_node) {
		endpoint = of_graph_get_next_endpoint(dsi_node, NULL);
		if (endpoint) {
			remote_node = of_graph_get_remote_port_parent(endpoint);
			if (!remote_node) {
				pr_err("No panel connected,skip probe lcm\n");
				return -ENODEV;
			}
			pr_err("device node name:%s\n", remote_node->name);
		}
	}
	if (remote_node != dev->of_node) {
		pr_err("gezi ---- %s+ skip probe due to not current lcm\n", __func__);
		return -ENODEV;
	}

	ctx = devm_kzalloc(dev, sizeof(struct lcm), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	mipi_dsi_set_drvdata(dsi, ctx);

	ctx->dev = dev;
	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	// dsi->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_SYNC_PULSE
	// 		 | MIPI_DSI_MODE_LPM | MIPI_DSI_MODE_EOT_PACKET
	// 		 | MIPI_DSI_CLOCK_NON_CONTINUOUS;
	dsi->mode_flags = MIPI_DSI_MODE_LPM | MIPI_DSI_MODE_EOT_PACKET | MIPI_DSI_CLOCK_NON_CONTINUOUS;  //prize add by yinhanhan for hbm 20230311
	// dsi->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_BURST;
	backlight = of_parse_phandle(dev->of_node, "backlight", 0);
	if (backlight) {
		ctx->backlight = of_find_backlight_by_node(backlight);
		of_node_put(backlight);

		if (!ctx->backlight)
			return -EPROBE_DEFER;
	}

	pr_err("gezi ---------%d-----\n",__LINE__);

	ctx->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio)) {
		dev_err(dev, "%s: cannot get reset-gpios %ld\n",
			__func__, PTR_ERR(ctx->reset_gpio));
		return PTR_ERR(ctx->reset_gpio);
	}
	devm_gpiod_put(dev, ctx->reset_gpio);

	ctx->bias_pos = devm_gpiod_get_index(dev, "bias", 0, GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->bias_pos)) {
		dev_err(dev, "%s: cannot get bias-pos 0 %ld\n",
			__func__, PTR_ERR(ctx->bias_pos));
		return PTR_ERR(ctx->bias_pos);
	}
	devm_gpiod_put(dev, ctx->bias_pos);

	ctx->bias_neg = devm_gpiod_get_index(dev, "bias", 1, GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->bias_neg)) {
		dev_err(dev, "%s: cannot get bias-neg 1 %ld\n",
			__func__, PTR_ERR(ctx->bias_neg));
		return PTR_ERR(ctx->bias_neg);
	}

	pr_err("gezi ---------%d-----\n",__LINE__);

	devm_gpiod_put(dev, ctx->bias_neg);

	ctx->prepared = true;
	ctx->enabled = true;

	drm_panel_init(&ctx->panel);
	ctx->panel.dev = dev;
	ctx->panel.funcs = &lcm_drm_funcs;

	ret = drm_panel_add(&ctx->panel);
	if (ret < 0)
		return ret;

	pr_err("gezi ---------%d-----\n",__LINE__);

	ret = mipi_dsi_attach(dsi);
	if (ret < 0)
		drm_panel_remove(&ctx->panel);

#if defined(CONFIG_MTK_PANEL_EXT)
	//mtk_panel_tch_handle_reg(&ctx->panel);
	ret = mtk_panel_ext_create(dev, &ext_params, &ext_funcs, &ctx->panel);
	if (ret < 0)
		return ret;
#endif
	check_is_need_fake_resolution(dev);
	pr_err("%s------------%d\n", __func__,__LINE__);

	//add by wangfei
	// lcm_panel_init(ctx);
	g_ctx = ctx;
	ctx->hbm_en = false;
	g_ctx->hbm_stat = false;
	oled_init();
	prize_common_node_show_register("HBMSTATE", &get_hbmstate);

/* prize modified by gongtaitao for x9 lava hbm mode 20230421 start */
	INIT_WORK(&check_level_worker, lcm_check_level_work);
	timer_setup(&check_level_timer, lcm_check_level_timer, 0);
	mod_timer(&check_level_timer, jiffies + msecs_to_jiffies(15000));
	/* prize modified by gongtaitao for x9 lava hbm mode 20230421 end */

#if defined(CONFIG_PRIZE_HARDWARE_INFO)
    strcpy(current_lcm_info.chip,"rm692e5.cmd");
    strcpy(current_lcm_info.vendor,"Raydium");
    sprintf(current_lcm_info.id,"0x%02x",0x81);
    strcpy(current_lcm_info.more,"1080*2412");
#endif
	return ret;
}



static int lcm_remove(struct mipi_dsi_device *dsi)
{
	struct lcm *ctx = mipi_dsi_get_drvdata(dsi);

	mipi_dsi_detach(dsi);
	drm_panel_remove(&ctx->panel);

	return 0;
}

static const struct of_device_id lcm_of_match[] = {
	{ .compatible = "Raydium,rm692e5,cmd", },
	{ }
};

MODULE_DEVICE_TABLE(of, lcm_of_match);

static struct mipi_dsi_driver lcm_driver = {
	.probe = lcm_probe,
	.remove = lcm_remove,
	.driver = {
		.name = "panel-rm692e5-cmd",
		.owner = THIS_MODULE,
		.of_match_table = lcm_of_match,
	},
};

module_mipi_dsi_driver(lcm_driver);

MODULE_AUTHOR("Yi-Lun Wang <Yi-Lun.Wang@mediatek.com>");
MODULE_DESCRIPTION("rm692e5 CMD LCD Panel Driver");
MODULE_LICENSE("GPL v2");

