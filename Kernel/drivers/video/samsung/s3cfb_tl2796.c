/*
 * s6e63m0 AMOLED Panel Driver for the Samsung Universal board
 *
 * Derived from drivers/video/omap/lcd-apollon.c
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 */

#include <linux/wait.h>
#include <linux/fb.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/spi/spi.h>
#include <linux/lcd.h>
#include <linux/backlight.h>
#if defined(CONFIG_MACH_S5PC110_LATONA)
#include <linux/nt35580.h>
#include "s3cfb_mdnie.h"
#else
#include <linux/tl2796.h>
#endif
#include <plat/gpio-cfg.h>
#include <plat/regs-fb.h>
#include <linux/earlysuspend.h>
#if defined(CONFIG_MACH_S5PC110_PRESTIGE)
#include <plat/map-base.h>
#include <mach/regs-gpio.h>
#endif

#define SLEEPMSEC		0x1000
#define ENDDEF			0x2000
#define DEFMASK		0xFF00
#define DIM_BL	20
#define MAX_BL	255
#if defined(CONFIG_MACH_S5PC110_LATONA)
#define MIN_BL	7 // 30
#else
#define MIN_BL	30
#endif

#if defined(CONFIG_MACH_S5PC110_PRESTIGE) //TEMP
#define S5PV210_GPH0_BASE		(S5P_VA_GPIO + 0xC00)
#define S5PV210_GPH0DAT			(S5PV210_GPH0_BASE + 0x04)
#define S5PV210_GPH1DAT			(S5PV210_GPH1_BASE + 0x04)
#define S5PV210_GPH2DAT			(S5PV210_GPH2_BASE + 0x04)
#define S5PV210_GPH3DAT			(S5PV210_GPH3_BASE + 0x04)
#endif

#if defined(CONFIG_MACH_S5PC110_PRESTIGE)
#define MAIN_LCD		0
#define SUB_LCD			1

#define MAX_GAMMA_VALUE	11	// we have 25 levels. -> 16 levels -> 24 levels
#else
#define MAX_GAMMA_VALUE	24
#endif

extern struct class *sec_class;

#if defined(CONFIG_MACH_S5PC110_LATONA)
#define MAX_BRIGHTNESS_LEVEL 255
#define LOW_BRIGHTNESS_LEVEL 7 // 30

#define MAX_BACKLIGHT_VALUE 255 // 192 // If you change this, You should change MAX_BACKLIGHT_VALUE at sesor/optical/gp2a.c
#define LOW_BACKLIGHT_VALUE_SONY 7 // 34 // 30
#define DIM_BACKLIGHT_VALUE_SONY 7 // 20 // 12

#define MAX_BRIGHTNESS_LEVEL_A1 221
#define MAX_BACKLIGHT_VALUE_A1 221

#define PWM_REG_OFFSET		1
#define PWM_REG_OFFSET_HITACHI	4

extern unsigned int HWREV;
extern Lcd_mDNIe_UI current_mDNIe_UI;

static unsigned short brightness_setting_table_hitachi[] = {
	0x0B0, 0x102,
	0x0B9, 0x100,
	0x16D, 			// PWM control
	0x102, 0x108, 
	0x0B0, 0x103,
	ENDDEF, 0x0000                                
};

static unsigned short brightness_setting_table_hitachi_a1[] = {
	0x0B0, 0x102,
	0x0B9, 0x100,
	0x156, 			// PWM control
	0x102, 0x108, 
	0x0B0, 0x103,
	ENDDEF, 0x0000                                
};

enum { // LCD type
	LCD_TYPE_MIN = 0,
	LCD_TYPE_SONY,
	LCD_TYPE_HYDIS,
	LCD_TYPE_HITACHI_1,
	LCD_TYPE_HITACHI_2,
	LCD_TYPE_SMD,
	LCD_TYPE_GP,
	LCD_TYPE_MAX
};

static int lcd_type = LCD_TYPE_SONY;
u8 lcd_id_arr[5] = {0};

// Hydis Panel
const unsigned short SEQ_PANEL_ON_HYDIS[] = { //sjlee_0212 (hydis panel tuning value)
		// User Set
		0x0FF, 0x1AA, // Test Commands
		0x155,
		0x125,
		0x101,
		0x0F3, 0x100, // Test Commands
		0x132,
		0x100,
		0x138,
		0x131,
		0x108,
		0x111,
		0x100,
		0x0F0, 0x155, // Manufacture Command Set Selection
		0x1AA,
		0x152,
		0x108,
		0x100,
		0x0B0, 0x104, // DE low enable
		0x10A,
		0x10E,
		0x109,
		0x104,
		0x0B1, 0x1CC, // Backward for Gate
		0x104,
		0x036, 0x102,
		0x0B3, 0x100,
		0x0B6, 0x103,
		0x0B7, 0x170,
		0x170,
		0x0B8, 0x100,
		0x106,
		0x106,
		0x106,
		0x0BC, 0x100,
		0x100,
		0x100,
		0x0BD, 0x101,
		0x184,
		0x106,
		0x150,
		0x100,
		0x0CC, 0x103,
		0x12A,
		0x106,

		// Power Set
		0x0F0, 0x155,
		0x1AA,
		0x152,
		0x108,
		0x101,
		0x0B0, 0x105,
		0x105,
		0x105,
		0x0B1, 0x105,
		0x105,
		0x105,
		0x0B2, 0x103,
		0x103,
		0x103,
		0x0B8, 0x124,
		0x124,
		0x124,
		0x0B3, 0x10A,
		0x10A,
		0x10A,
		0x0B9, 0x124,
		0x124,
		0x124,
		0x0BF, 0x101,
		0x0B5, 0X108,
		0x108,
		0x108,
		0x0B4, 0x12D,
		0x12D,
		0x12D,
		0x0BC, 0x100,
		0x150,
		0x100,
		0x0BD, 0x100,
		0x160,
		0x100,
		0x0BE, 0x100,
		0x13D,
		0x0CE, 0x100,
		0x100,
		0x100,
		0x100,
		0x100,
		0x100,
		0x100,

		//Gamma Control
		0x0D0, 0X109,
		0x113,
		0x104,
		0x10A,

		// POSI (RED)
		0x0D1, 0x100,
		0x137,
		0x100,
		0x171,
		0x100,
		0x1A2,
		0x100,
		0x1C4,
		0x100,
		0x1DF,
		0x101,
		0x116,
		0x101,
		0x151,
		0x101,
		0x18A,
		0x101,
		0x1A6,
		0x101,
		0x1D1,
		0x102,
		0x101,
		0x102,
		0x13D,
		0x102,
		0x177,
		0x102,
		0x179,
		0x102,
		0x1A5,
		0x102,
		0x1D1,
		0x102,
		0x1F9,
		0x103,
		0x125,
		0x103,
		0x143,
		0x103,
		0x16E,
		0x103,
		0x177,
		0x103,
		0x194,
		0x103,
		0x19E,
		0x103,
		0x1AC,
		0x103,
		0x1BD,
		0x103,
		0x1F1,

		// POSI (GREEN)
		0x0D2, 0x100,
		0x137,
		0x100,
		0x171,
		0x100,
		0x1A2,
		0x100,
		0x1C4,
		0x100,
		0x1DF,
		0x101,
		0x116,
		0x101,
		0x151,
		0x101,
		0x18A,
		0x101,
		0x1A6,
		0x101,
		0x1D1,
		0x102,
		0x101,
		0x102,
		0x13D,
		0x102,
		0x177,
		0x102,
		0x179,
		0x102,
		0x1A5,
		0x102,
		0x1D1,
		0x102,
		0x1F9,
		0x103,
		0x125,
		0x103,
		0x143,
		0x103,
		0x16E,
		0x103,
		0x177,
		0x103,
		0x194,
		0x103,
		0x19E,
		0x103,
		0x1AC,
		0x103,
		0x1BD,
		0x103,
		0x1F1,


		// POSI (BLUE)
		0x0D3, 0x100,
		0x137,
		0x100,
		0x171,
		0x100,
		0x1A2,
		0x100,
		0x1C4,
		0x100,
		0x1DF,
		0x101,
		0x116,
		0x101,
		0x151,
		0x101,
		0x18A,
		0x101,
		0x1A6,
		0x101,
		0x1D1,
		0x102,
		0x101,
		0x102,
		0x13D,
		0x102,
		0x177,
		0x102,
		0x179,
		0x102,
		0x1A5,
		0x102,
		0x1D1,
		0x102,
		0x1F9,
		0x103,
		0x125,
		0x103,
		0x143,
		0x103,
		0x16E,
		0x103,
		0x177,
		0x103,
		0x194,
		0x103,
		0x19E,
		0x103,
		0x1AC,
		0x103,
		0x1BD,
		0x103,
		0x1F1,


		// NEGA (RED)
		0x0D4, 0x100,
		0x137,
		0x100,
		0x146,
		0x100,
		0x17E,
		0x100,
		0x19E,
		0x100,
		0x1C7,
		0x101,
		0x11A,
		0x101,
		0x134,
		0x101,
		0x154,
		0x101,
		0x179,
		0x101,
		0x1D8,
		0x101,
		0x1DF,
		0x102,
		0x12F,
		0x102,
		0x168,
		0x102,
		0x16A,
		0x102,
		0x1A3,
		0x102,
		0x1E0,
		0x102,
		0x1F9,
		0x103,
		0x125,
		0x103,
		0x143,
		0x103,
		0x16E,
		0x103,
		0x177,
		0x103,
		0x194,
		0x103,
		0x19E,
		0x103,
		0x1AC,
		0x103,
		0x1BD,
		0x103,
		0x1F1,

		// NEGA (GREEN)
		0x0D5, 0x100,
		0x137,
		0x100,
		0x146,
		0x100,
		0x17E,
		0x100,
		0x19E,
		0x100,
		0x1C7,
		0x101,
		0x11A,
		0x101,
		0x134,
		0x101,
		0x154,
		0x101,
		0x179,
		0x101,
		0x1D8,
		0x101,
		0x1DF,
		0x102,
		0x12F,
		0x102,
		0x168,
		0x102,
		0x16A,
		0x102,
		0x1A3,
		0x102,
		0x1E0,
		0x102,
		0x1F9,
		0x103,
		0x125,
		0x103,
		0x143,
		0x103,
		0x16E,
		0x103,
		0x177,
		0x103,
		0x194,
		0x103,
		0x19E,
		0x103,
		0x1AC,
		0x103,
		0x1BD,
		0x103,
		0x1F1,

		// NEGA (BLUE)
		0x0D6, 0x100,
		0x137,
		0x100,
		0x146,
		0x100,
		0x17E,
		0x100,
		0x19E,
		0x100,
		0x1C7,
		0x101,
		0x11A,
		0x101,
		0x134,
		0x101,
		0x154,
		0x101,
		0x179,
		0x101,
		0x1D8,
		0x101,
		0x1DF,
		0x102,
		0x12F,
		0x102,
		0x168,
		0x102,
		0x16A,
		0x102,
		0x1A3,
		0x102,
		0x1E0,
		0x102,
		0x1F9,
		0x103,
		0x125,
		0x103,
		0x143,
		0x103,
		0x16E,
		0x103,
		0x177,
		0x103,
		0x194,
		0x103,
		0x19E,
		0x103,
		0x1AC,
		0x103,
		0x1BD,
		0x103,
		0x1F1,

		// SLPOUT
		0x011,

		SLEEPMSEC, 150,

		// PWM
		0x051, 0x16C,
		0x053, 0x12C,
		
		// DISPLAY ON
		0x029,
	
		ENDDEF, 0x0000	
};

const unsigned short SEQ_PANEL_ON_HITACHI_A1[] = {
	0x036, 0x100,
	0x03A, 0x177,

	0x011, // exit sleep mode
	SLEEPMSEC, 200, // min 7frame,

	0x0B0, 0x104,
	0x0B9, 0x100,
	0x156,
	0x102,
	0x108,	

	0x0C1, 0x142,
	0x131,
	0x104,
	0x0B0, 0x103,	
	
	0x029,	// set display on

	ENDDEF, 0x0000
};

#endif  // CONFIG_MACH_S5PC110_LATONA

/*********** for debug **********************************************************/
#if 0
#define gprintk(fmt, x... ) printk("%s(%d): " fmt, __FUNCTION__ , __LINE__ , ## x)
#else
#define gprintk(x...) do { } while (0)
#endif
/*******************************************************************************/

int current_gamma_value = -1;
static int ldi_enable = 0;

#ifdef CONFIG_FB_S3C_MDNIE
extern void init_mdnie_class(void);
#endif

#if defined(CONFIG_MACH_S5PC110_PRESTIGE)
extern unsigned int s3c_keygpio_get_flip_status(void);
extern const u16 s6e63m0_SEQ_DISPLAY_ON[];
extern const u16 s6e63m0_SEQ_DISPLAY_OFF[];
extern const u16 s6e63m0_SEQ_STANDBY_ON[];
extern const u16 s6e63m0_SEQ_STANDBY_OFF[];
extern const u16 s6e63m0_SEQ_DISPLAY_SETTING[];
extern const u16 s6e63m0_SEQ_ETC_SETTING[];
extern const unsigned short s6e63m0_MSEQ_SETTING[];
extern const unsigned short s6e63m0_SSEQ_SETTING[];
extern const u16 *p19Gamma_set[];
extern const u16 *p22Gamma_set[];

extern int is_keypad_backlight_night;

static int bd_brightness = 0;
static int on_19gamma = 0;

static struct s5p_lcd sLcd;

int working_lcd = 0;
EXPORT_SYMBOL(working_lcd);

static DEFINE_MUTEX(spi_use);
//static DEFINE_MUTEX(s3cfb_tl2796_use);

typedef enum {
	BACKLIGHT_LEVEL_OFF		= 0,
	BACKLIGHT_LEVEL_DIMMING	= 1,
	BACKLIGHT_LEVEL_NORMAL	= 6
} backlight_level_t;

backlight_level_t backlight_level = BACKLIGHT_LEVEL_OFF;

#elif defined(CONFIG_MACH_S5PC110_LATONA)  // CONFIG_MACH_S5PC110_PRESTIGE

typedef enum
{
	CABC_OFF_MODE=0,
	CABC_UI_MODE,
	CABC_IMAGE_MODE,
	CABC_VIDEO_MODE,
} CABC_MODE;

static CABC_MODE cur_cabc_mode = CABC_OFF_MODE;

#endif  // CONFIG_MACH_S5PC110_PRESTIGE

struct s5p_lcd {
	int ldi_enable;
	int bl;
	int acl_enable;
	int cur_acl;
	int on_19gamma;
	const struct tl2796_gamma_adj_points *gamma_adj_points;
	struct mutex	lock;
	struct device *dev;
	struct spi_device *g_spi;
#if defined(CONFIG_MACH_S5PC110_LATONA)
	struct s5p_tft_panel_data *data;
#else
	struct s5p_panel_data	*data;
#endif
	struct backlight_device *bl_dev;
	struct lcd_device *lcd_dev;
	struct class *acl_class;
	struct device *switch_aclset_dev;
	struct class *gammaset_class;
	struct device *switch_gammaset_dev;
	struct device *sec_lcdtype_dev;
	struct early_suspend    early_suspend;
};

int IsLDIEnabled(void)
{
	return ldi_enable;
}
EXPORT_SYMBOL(IsLDIEnabled);

#if defined(CONFIG_MACH_S5PC110_LATONA)
static inline int lcd_si_get_value(void)
{
	return gpio_get_value(GPIO_DISPLAY_SO);
}

static inline void lcd_si_value(int level)
{
	s3c_gpio_cfgpin(GPIO_DISPLAY_SI, S3C_GPIO_OUTPUT);
	gpio_set_value(GPIO_DISPLAY_SI, level);
}

static inline void lcd_cs_value(int level)
{
	s3c_gpio_cfgpin(GPIO_DISPLAY_CS, S3C_GPIO_OUTPUT);
	gpio_set_value(GPIO_DISPLAY_CS, level);
}

static inline void lcd_clk_value(int level)
{
	s3c_gpio_cfgpin(GPIO_DISPLAY_CLK, S3C_GPIO_OUTPUT);
	gpio_set_value(GPIO_DISPLAY_CLK, level);
}

static unsigned char c110_spi_read_byte()
{
	u8 i, lcdId=0;
	int pin_value;

	for (i = 0; i < 8; i++)
	{
		lcd_clk_value(0);
		udelay(1);

		lcd_clk_value(1);
		udelay(1);

		pin_value = lcd_si_get_value();
		lcdId = ((lcdId << 1) & ~0x0001) | (0x01 & pin_value);
	}

	return lcdId;
}

static void s6e63m0_c110_spi_read_byte()
{
	u8 lcd_id_dummy;
	int i=0;

	s3c_gpio_cfgpin(GPIO_DISPLAY_SO, S3C_GPIO_INPUT);

	lcd_id_dummy = c110_spi_read_byte();
	for( ; i<5 ; i++)
		lcd_id_arr[i] = c110_spi_read_byte();

	s3c_gpio_cfgpin(GPIO_DISPLAY_SO, S3C_GPIO_OUTPUT);
	gpio_set_value(GPIO_DISPLAY_SO, GPIO_LEVEL_HIGH);
}

static void s6e63m0_spi_read_driver(unsigned char address, unsigned char command) //sjlee_0214 (SPI read byte)
{
	int     j;
	unsigned short data;

	data = (address << 8) + command;

	lcd_cs_value(1);
	lcd_si_value(1);
	lcd_clk_value(1);
	udelay(1);

	lcd_cs_value(0);
	udelay(1);

	for (j = 8; j >= 0; j--)
	{
		lcd_clk_value(0);

		/* data high or low */
		if ((data >> j) & 0x0001)
			lcd_si_value(1);
		else
			lcd_si_value(0);

		udelay(1);

		lcd_clk_value(1);
		udelay(1);
	}

	s6e63m0_c110_spi_read_byte();

	lcd_cs_value(1);
	udelay(1);
}
#endif

static int s6e63m0_spi_write_driver(struct s5p_lcd *lcd, u16 reg)
{
	u16 buf[1];
	int ret;
	struct spi_message msg;

	struct spi_transfer xfer = {
		.len	= 2,
		.tx_buf	= buf,
	};

	buf[0] = reg;

	spi_message_init(&msg);
	spi_message_add_tail(&xfer, &msg);

	ret = spi_sync(lcd->g_spi, &msg);

	if (ret < 0)
		pr_err("%s error\n", __func__);

	return ret ;
}

static void s6e63m0_panel_send_sequence(struct s5p_lcd *lcd,
	const u16 *wbuf)
{
	int i = 0;

	while ((wbuf[i] & DEFMASK) != ENDDEF) {
		if ((wbuf[i] & DEFMASK) != SLEEPMSEC) {
			s6e63m0_spi_write_driver(lcd, wbuf[i]);
			i += 1;
		} else {
			msleep(wbuf[i+1]);
			i += 2;
		}
	}
}

#if defined(CONFIG_MACH_S5PC110_LATONA)
static int get_pwm_value_from_bl(int level)
{
	int tune_value;
	int max_level, max_value;

	max_level = MAX_BRIGHTNESS_LEVEL;
	max_value = MAX_BACKLIGHT_VALUE;

	// SMD LCD
	if(level > max_level)
		level = max_level;

	if(level >= LOW_BRIGHTNESS_LEVEL)
		tune_value = (level - LOW_BRIGHTNESS_LEVEL) * (max_value-LOW_BACKLIGHT_VALUE_SONY) / (max_level-LOW_BRIGHTNESS_LEVEL) + LOW_BACKLIGHT_VALUE_SONY;
	else if(level > 0)
		tune_value = DIM_BACKLIGHT_VALUE_SONY;
	else
		tune_value = level;
	
	if(tune_value > max_value)
		tune_value = max_value;			// led_val must be less than or equal to MAX_BACKLIGHT_VALUE

	if(level && !tune_value)
		tune_value = 1;

	return tune_value;
}
#else  // CONFIG_MACH_S5PC110_LATONA
static int get_gamma_value_from_bl(int bl)
{
	int gamma_value = 0;
	int gamma_val_x10 = 0;

	if (bl >= MIN_BL)		{
		gamma_val_x10 = 10*(MAX_GAMMA_VALUE-1)*bl/(MAX_BL-MIN_BL) + (10 - 10*(MAX_GAMMA_VALUE-1)*(MIN_BL)/(MAX_BL-MIN_BL)) ;
		gamma_value = (gamma_val_x10+5)/10;
	} else {
		gamma_value = 0;
	}

	return gamma_value;
}
#endif  // CONFIG_MACH_S5PC110_LATONA

#ifdef CONFIG_FB_S3C_TL2796_ACL
static void update_acl(struct s5p_lcd *lcd)
{
	struct s5p_panel_data *pdata = lcd->data;
	int gamma_value;

	gamma_value = get_gamma_value_from_bl(lcd->bl);

	if (lcd->acl_enable) {
		if ((lcd->cur_acl == 0) && (gamma_value != 1)) {
			s6e63m0_panel_send_sequence(lcd, pdata->acl_init);
			msleep(20);
		}

		switch (gamma_value) {
#if defined(CONFIG_MACH_S5PC110_PRESTIGE)		
		case 1:
			if (lcd->cur_acl != 0) {
				s6e63m0_panel_send_sequence(lcd, pdata->acl_table[0]);
				lcd->cur_acl = 0;
				gprintk(" ACL_cutoff_set Percentage : 0!!\n");
			}
			break;
		case 2:
			if (lcd->cur_acl != 12) {
				s6e63m0_panel_send_sequence(lcd, pdata->acl_table[1]);
				lcd->cur_acl = 12;
				gprintk(" ACL_cutoff_set Percentage : 12!!\n");
			}
			break;
		case 3:
			if (lcd->cur_acl != 22) {
				s6e63m0_panel_send_sequence(lcd, pdata->acl_table[2]);
				lcd->cur_acl = 22;
				gprintk(" ACL_cutoff_set Percentage : 22!!\n");
			}
			break;
		case 4:
			if (lcd->cur_acl != 30) {
				s6e63m0_panel_send_sequence(lcd, pdata->acl_table[3]);
				lcd->cur_acl = 30;
				gprintk(" ACL_cutoff_set Percentage : 30!!\n");
			}
			break;
		case 5:
			if (lcd->cur_acl != 35) {
				s6e63m0_panel_send_sequence(lcd, pdata->acl_table[4]);
				lcd->cur_acl = 35;
				gprintk(" ACL_cutoff_set Percentage : 35!!\n");
			}
			break;
		default:
			if (lcd->cur_acl != 40) {
				s6e63m0_panel_send_sequence(lcd, pdata->acl_table[5]);
				lcd->cur_acl = 40;
				gprintk(" ACL_cutoff_set Percentage : 40!!\n");
			}
#else
		case 1:
			if (lcd->cur_acl != 0) {
				s6e63m0_panel_send_sequence(lcd, pdata->acl_table[0]);
				lcd->cur_acl = 0;
				gprintk(" ACL_cutoff_set Percentage : 0!!\n");
			}
			break;
		case 2 ... 12:
			if (lcd->cur_acl != 40)	{
				s6e63m0_panel_send_sequence(lcd, pdata->acl_table[9]);
				lcd->cur_acl = 40;
				gprintk(" ACL_cutoff_set Percentage : 40!!\n");
			}
			break;
		case 13:
			if (lcd->cur_acl != 43)	{
				s6e63m0_panel_send_sequence(lcd, pdata->acl_table[16]);
				lcd->cur_acl = 43;
				gprintk(" ACL_cutoff_set Percentage : 43!!\n");
			}
			break;
		case 14:
			if (lcd->cur_acl != 45)	{
				s6e63m0_panel_send_sequence(lcd, pdata->acl_table[10]);
				lcd->cur_acl = 45;
				gprintk(" ACL_cutoff_set Percentage : 45!!\n");
			}
			break;
		case 15:
			if (lcd->cur_acl != 47)	{
				s6e63m0_panel_send_sequence(lcd, pdata->acl_table[11]);
				lcd->cur_acl = 47;
				gprintk(" ACL_cutoff_set Percentage : 47!!\n");
			}
			break;
		case 16:
			if (lcd->cur_acl != 48)	{
				s6e63m0_panel_send_sequence(lcd, pdata->acl_table[12]);
				lcd->cur_acl = 48;
				gprintk(" ACL_cutoff_set Percentage : 48!!\n");
			}
			break;
		default:
			if (lcd->cur_acl != 50)	{
				s6e63m0_panel_send_sequence(lcd, pdata->acl_table[13]);
				lcd->cur_acl = 50;
				gprintk(" ACL_cutoff_set Percentage : 50!!\n");
			}
#endif			
		}
	} else	{
		s6e63m0_panel_send_sequence(lcd, pdata->acl_table[0]);
		lcd->cur_acl  = 0;
		gprintk(" ACL_cutoff_set Percentage : 0!!\n");
	}

}
#endif

static void update_brightness(struct s5p_lcd *lcd)
{
#if defined(CONFIG_MACH_S5PC110_LATONA)
	struct s5p_tft_panel_data *pdata = lcd->data;
#else
	struct s5p_panel_data *pdata = lcd->data;
#endif
	int gamma_value;

#if defined(CONFIG_MACH_S5PC110_LATONA)
	gamma_value = get_pwm_value_from_bl(lcd->bl);
#else
	gamma_value = get_gamma_value_from_bl(lcd->bl);
#endif


#if defined(CONFIG_MACH_S5PC110_PRESTIGE)
	current_gamma_value = gamma_value;
#endif

	gprintk("Update status brightness[0~255]:(%d) gamma_value:(%d) on_19gamma(%d)\n", lcd->bl, gamma_value, lcd->on_19gamma);

#ifdef CONFIG_FB_S3C_TL2796_ACL
	update_acl(lcd);
#endif

#if defined(CONFIG_MACH_S5PC110_LATONA)
	pdata->brightness_set[PWM_REG_OFFSET] = 0x100 | (gamma_value & 0xff);
	s6e63m0_panel_send_sequence(lcd, pdata->brightness_set);
#else  // CONFIG_MACH_S5PC110_LATONA
	if (lcd->on_19gamma)
		s6e63m0_panel_send_sequence(lcd, pdata->gamma19_table[gamma_value]);
	else
		s6e63m0_panel_send_sequence(lcd, pdata->gamma22_table[gamma_value]);
	s6e63m0_panel_send_sequence(lcd, pdata->gamma_update);
#endif  // CONFIG_MACH_S5PC110_LATONA
}

#if defined(CONFIG_MACH_S5PC110_PRESTIGE)
const u16 s6e63m0_SSEQ_DISPLAY_SETTING[] = {
	SLEEPMSEC, 10,
	0x0F8,	/* Panel Condition Set Command*/
	0x101,	/* DOCT */
	0x127,	/* CLWEA */
	0x127,	/* CLWEB*/
	0x107,	/* CLTE */
	0x107,	/* SHE */
	0x154,	/* FLTE */
	0x19F,	/* FLWE */
	0x163,	/* SCTE */
	0x186,	/* SCWE */
	0x11A,	/* INTE */
	0x133,	/* INWE */
	0x10D,	/* EMPS */
	0x100,	/* E_INTE */
	0x100,	/* E_INWE */
	0x0F2,	/* Display Condition Set Command*/
	0x102,	/* Number of Line */
	0x103,	/* VBP */
	0x11C,	/* VFP */
	0x110,	/* HBP */
	0x110,	/* HFP */
	0x0F7,	/* Command */
	0x100,	/* GTCON */
	0x100,	/* Display Mode */
	0x100,	/* Vsync/Hsync, DOCCLK, RGB mode */
	ENDDEF, 0x0000
};

static void SetLDIEnabledFlag(int OnOff);
#endif

#if defined(CONFIG_MACH_S5PC110_LATONA)
void Lcd_cabc_on(struct s5p_lcd *lcd)
{
	CABC_MODE cabc_mode;
	
	if(lcd->acl_enable == 0)
		return;
		
	printk(KERN_DEBUG "mDNIe_MODE = %d\n", current_mDNIe_UI); 

	switch(current_mDNIe_UI)
	{
		case mDNIe_UI_MODE:		
			cabc_mode = CABC_UI_MODE;
			break;
		case mDNIe_VIDEO_MODE:
		case mDNIe_VIDEO_WARM_MODE:
		case mDNIe_VIDEO_COLD_MODE:
		case mDNIe_CAMERA_MODE:
			cabc_mode = CABC_VIDEO_MODE;
			break;
		case mDNIe_NAVI:
			cabc_mode = CABC_IMAGE_MODE;
			break;
		default:
			cabc_mode = CABC_UI_MODE;
			break;
	}
	
	if (!IsLDIEnabled())
	{
		printk(KERN_DEBUG "skip set cabc mode : LDI is not ready\n");
		return;
	}

	if (cur_cabc_mode != cabc_mode)
	{
		printk(KERN_DEBUG "update cabc mode = %d\n", cabc_mode); 		
		switch(cabc_mode)
		{
			case CABC_UI_MODE:
				s6e63m0_panel_send_sequence(lcd, lcd->data->cabc_ui);
				break;
			case CABC_VIDEO_MODE:
				s6e63m0_panel_send_sequence(lcd, lcd->data->cabc_video);
				break;
			case CABC_IMAGE_MODE:
				s6e63m0_panel_send_sequence(lcd, lcd->data->cabc_image);
				break;
			default:
				s6e63m0_panel_send_sequence(lcd, lcd->data->cabc_ui);
				break;
		}
		cur_cabc_mode = cabc_mode;
	}
}

static void Lcd_cabc_off(struct s5p_lcd *lcd)
{
	if (cur_cabc_mode != CABC_OFF_MODE)
	{
		printk(KERN_DEBUG "update cabc mode = %d\n", cur_cabc_mode); 
		s6e63m0_panel_send_sequence(lcd, lcd->data->cabc_off);
		cur_cabc_mode = CABC_OFF_MODE;
	}
}

static ssize_t cabc_mode_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
	gprintk("[%s] cabc_mode = %d\n",__func__, cur_cabc_mode);

	return sprintf(buf,"%u\n", cur_cabc_mode);
}
static ssize_t cabc_mode_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t size)
{
	struct s5p_lcd *lcd = dev_get_drvdata(dev);
	int value;
	
	sscanf(buf, "%d", &value);

	switch(value)
	{
		case 1:
			s6e63m0_panel_send_sequence(lcd, lcd->data->cabc_ui);
			cur_cabc_mode = CABC_UI_MODE;
			printk("[%s] set CABC_UI\n", __func__);
			break;
		case 2:
			s6e63m0_panel_send_sequence(lcd, lcd->data->cabc_image);
			cur_cabc_mode = CABC_IMAGE_MODE;
			printk("[%s] set CABC_IMAGE\n", __func__);
			break;
		case 3:
			s6e63m0_panel_send_sequence(lcd, lcd->data->cabc_video);
			cur_cabc_mode = CABC_VIDEO_MODE;
			printk("[%s] set CABC_VIDEO\n", __func__);
			break;
		default:
			s6e63m0_panel_send_sequence(lcd, lcd->data->cabc_off);
			cur_cabc_mode = CABC_OFF_MODE;
			printk("[%s] set CABC_OFF\n", __func__);
			break;
	}

	return size;
}

static DEVICE_ATTR(cabc_mode,0664, cabc_mode_show, cabc_mode_store);
#endif

static void tl2796_ldi_enable(struct s5p_lcd *lcd)
{
#if defined(CONFIG_MACH_S5PC110_LATONA)
	struct s5p_tft_panel_data *pdata = lcd->data;
#else
	struct s5p_panel_data *pdata = lcd->data;
#endif

	mutex_lock(&lcd->lock);

#if defined(CONFIG_MACH_S5PC110_LATONA) //sjlee_0212
	if(lcd_type == LCD_TYPE_HYDIS) {
		s6e63m0_panel_send_sequence(lcd, SEQ_PANEL_ON_HYDIS);
	} else if (lcd_type == LCD_TYPE_HITACHI_1) {
		s6e63m0_panel_send_sequence(lcd, SEQ_PANEL_ON_HITACHI_A1);
	} else {
		s6e63m0_panel_send_sequence(lcd, pdata->seq_set);
		s6e63m0_panel_send_sequence(lcd, pdata->display_on);
	}
	update_brightness(lcd);
#else  // CONFIG_MACH_S5PC110_LATONA

#if defined(CONFIG_MACH_S5PC110_PRESTIGE)
	if (working_lcd == SUB_LCD)
		s6e63m0_panel_send_sequence(lcd, &s6e63m0_SSEQ_DISPLAY_SETTING);
	else
#endif  // CONFIG_MACH_S5PC110_PRESTIGE
		s6e63m0_panel_send_sequence(lcd, pdata->seq_display_set);

	s6e63m0_panel_send_sequence(lcd, pdata->seq_etc_set);
	update_brightness(lcd);
#endif  // CONFIG_MACH_S5PC110_LATONA

	lcd->ldi_enable = 1;
#if defined(CONFIG_MACH_S5PC110_PRESTIGE)
	sLcd.ldi_enable = 1;
	SetLDIEnabledFlag(1);
#elif defined(CONFIG_MACH_S5PC110_LATONA)  // CONFIG_MACH_S5PC110_PRESTIGE
	Lcd_cabc_on(lcd);
#endif  // CONFIG_MACH_S5PC110_PRESTIGE

	mutex_unlock(&lcd->lock);
}

static void tl2796_ldi_disable(struct s5p_lcd *lcd)
{
#if defined(CONFIG_MACH_S5PC110_LATONA)
	struct s5p_tft_panel_data *pdata = lcd->data;
#else
	struct s5p_panel_data *pdata = lcd->data;
#endif

	mutex_lock(&lcd->lock);

	lcd->ldi_enable = 0;
#if defined(CONFIG_MACH_S5PC110_PRESTIGE)
	sLcd.ldi_enable = 0;
	SetLDIEnabledFlag(0);
#endif

#if defined(CONFIG_MACH_S5PC110_LATONA)
	s6e63m0_panel_send_sequence(lcd, pdata->sleep_in);
	s6e63m0_panel_send_sequence(lcd, pdata->display_off);
	cur_cabc_mode = CABC_OFF_MODE;
#else  // CONFIG_MACH_S5PC110_LATONA
	s6e63m0_panel_send_sequence(lcd, pdata->standby_on);
#endif  // CONFIG_MACH_S5PC110_LATONA

	mutex_unlock(&lcd->lock);
}

#if defined(CONFIG_MACH_S5PC110_PRESTIGE)
static void SetLDIEnabledFlag(int OnOff)
{
	ldi_enable = OnOff;
}

void tl2796_main_ldi_wakeup(void)
{
	static u8 once = 0;
	sLcd.g_spi->controller_data = (void *)(GPIO_DISPLAY_CS);
#if 1 // lcd blue problem, so it should be added
	if(once)  //sjlee_0125 (LCD flash problem at boot up)
	{
		if (current_gamma_value != -1)
		{
			s3c_gpio_setpin(GPIO_MLCD_RST, GPIO_LEVEL_LOW);
			mdelay(1);
			s3c_gpio_setpin(GPIO_MLCD_RST, GPIO_LEVEL_HIGH);
			mdelay(20);
		}
	}
	once = 1;
#endif
	printk("main ldi wakeup start : %d \n", ldi_enable);

#ifdef CONFIG_LUNCH_BOX_BOARD
	s6e63m0_panel_send_sequence((struct s5p_lcd *)&sLcd, s6e63m0_SSEQ_SETTING);
#else
	s6e63m0_panel_send_sequence((struct s5p_lcd *)&sLcd, s6e63m0_MSEQ_SETTING);
#endif
	s6e63m0_panel_send_sequence((struct s5p_lcd *)&sLcd, s6e63m0_SEQ_STANDBY_OFF);
	s6e63m0_panel_send_sequence((struct s5p_lcd *)&sLcd, s6e63m0_SEQ_DISPLAY_ON);

	if(is_keypad_backlight_night==0) // && is_debug_screen==0)
		s3c_gpio_setpin(GPIO_KEY_BL_EN, 1);

	dev_dbg(sLcd.lcd_dev,"%s::%d -> main ldi wakeup\n",__func__,__LINE__);	
	printk("main ldi wakeup end\n");
}

void tl2796_main_ldi_sleep(void)
{
	sLcd.g_spi->controller_data = (void *)(GPIO_DISPLAY_CS);

	printk("main ldi sleep start : %d \n", ldi_enable);

	s6e63m0_panel_send_sequence((struct s5p_lcd *)&sLcd, s6e63m0_SEQ_STANDBY_ON);
	s6e63m0_panel_send_sequence((struct s5p_lcd *)&sLcd, s6e63m0_SEQ_DISPLAY_OFF);

	s3c_gpio_setpin(GPIO_KEY_BL_EN, 0);

	dev_dbg(sLcd.lcd_dev,"%s::%d -> main ldi sleep\n",__func__,__LINE__);	
	printk("main ldi sleep end \n");
}

void tl2796_main_ldi_bl_on(void)
{
	struct s5p_panel_data *pdata = sLcd.data;
	sLcd.g_spi->controller_data = (void *)(GPIO_DISPLAY_CS);

	printk("main ldi bl on start : %d \n", ldi_enable);

	if(current_gamma_value != -1)	{
		if(on_19gamma)
			s6e63m0_panel_send_sequence((struct s5p_lcd *)&sLcd, p19Gamma_set[current_gamma_value]);
		else
			s6e63m0_panel_send_sequence((struct s5p_lcd *)&sLcd, p22Gamma_set[current_gamma_value]);
		s6e63m0_panel_send_sequence((struct s5p_lcd *)&sLcd, pdata->gamma_update);
	}
	printk("main ldi bl on end : %d \n", ldi_enable);
}

void tl2796_sub_ldi_wakeup(void)
{
	static u8 once = 0;
	sLcd.g_spi->controller_data = (void *)(GPIO_DISPLAY_SUB_CS);
#if 1 // lcd blue problem, so it should be added
	if(once)  //sjlee_0125 (LCD flash problem at boot up)
	{
		if (current_gamma_value != -1)
		{
			s3c_gpio_setpin(GPIO_SLCD_RST, GPIO_LEVEL_LOW);
			mdelay(1);
			s3c_gpio_setpin(GPIO_SLCD_RST, GPIO_LEVEL_HIGH);
			mdelay(20);
		}
	}
	once = 1;
#endif
	printk("sub ldi wakeup start : %d \n", ldi_enable);

#ifdef CONFIG_LUNCH_BOX_BOARD
	s6e63m0_panel_send_sequence((struct s5p_lcd *)&sLcd, s6e63m0_MSEQ_SETTING);
#else
	s6e63m0_panel_send_sequence((struct s5p_lcd *)&sLcd, s6e63m0_SSEQ_SETTING);
#endif
	s6e63m0_panel_send_sequence((struct s5p_lcd *)&sLcd, s6e63m0_SEQ_STANDBY_OFF);
	s6e63m0_panel_send_sequence((struct s5p_lcd *)&sLcd, s6e63m0_SEQ_DISPLAY_ON);

	dev_dbg(sLcd.lcd_dev,"%s::%d -> sub ldi wakeup\n",__func__,__LINE__);	
	printk("sub ldi wakeup end\n");
}

void tl2796_sub_ldi_sleep(void)
{
	printk("sub ldi sleep start : %d\n", ldi_enable);

	sLcd.g_spi->controller_data = (void *)(GPIO_DISPLAY_SUB_CS);

	s6e63m0_panel_send_sequence((struct s5p_lcd *)&sLcd, s6e63m0_SEQ_STANDBY_ON);
	s6e63m0_panel_send_sequence((struct s5p_lcd *)&sLcd, s6e63m0_SEQ_DISPLAY_OFF);

	dev_dbg(sLcd.lcd_dev,"%s::%d -> sub ldi sleep\n",__func__,__LINE__);	
	printk("sub ldi sleep end\n");
}

void tl2796_sub_ldi_bl_on(void)
{
	struct s5p_panel_data *pdata = sLcd.data;
	sLcd.g_spi->controller_data = (void *)(GPIO_DISPLAY_SUB_CS);

	printk("sub ldi bl on start : %d\n", ldi_enable);

	if(current_gamma_value != -1)	{
		if(on_19gamma)
			s6e63m0_panel_send_sequence((struct s5p_lcd *)&sLcd, p19Gamma_set[current_gamma_value]);
		else
			s6e63m0_panel_send_sequence((struct s5p_lcd *)&sLcd, p22Gamma_set[current_gamma_value]);
		s6e63m0_panel_send_sequence((struct s5p_lcd *)&sLcd, pdata->gamma_update);
	}
	printk("sub ldi bl on end\n");
}

void tl2796_switch_main2sub_ldi(void)
{
	mutex_lock(&sLcd.lock);

	tl2796_main_ldi_sleep();
	tl2796_sub_ldi_wakeup();
	tl2796_sub_ldi_bl_on();

	sLcd.g_spi->controller_data = (void *)(GPIO_DISPLAY_SUB_CS);
	working_lcd = SUB_LCD;
	SetLDIEnabledFlag(1);

	mutex_unlock(&sLcd.lock);
}

void tl2796_switch_sub2main_ldi(void)
{
	mutex_lock(&sLcd.lock);

	tl2796_sub_ldi_sleep();
	tl2796_main_ldi_wakeup();
	tl2796_main_ldi_bl_on();

	sLcd.g_spi->controller_data = (void *)(GPIO_DISPLAY_CS);
	working_lcd = MAIN_LCD;
	SetLDIEnabledFlag(1);

	mutex_unlock(&sLcd.lock);
}

void tl2796_set_working_ldi(int working_ldi)
{
	mutex_lock(&sLcd.lock);

	if(working_ldi)	{
		sLcd.g_spi->controller_data = (void *)(GPIO_DISPLAY_CS);
		working_lcd = MAIN_LCD;
	} else {
		sLcd.g_spi->controller_data = (void *)(GPIO_DISPLAY_SUB_CS);
		working_lcd = SUB_LCD;
	}

	mutex_unlock(&sLcd.lock);
}

void tl2796_set_working_ldi_in_isr(int working_ldi)
{
	if(working_ldi)	{
		sLcd.g_spi->controller_data = (void *)(GPIO_DISPLAY_CS);
		working_lcd = MAIN_LCD;
	} else {
		sLcd.g_spi->controller_data = (void *)(GPIO_DISPLAY_SUB_CS);
		working_lcd = SUB_LCD;
	}
}

EXPORT_SYMBOL(tl2796_switch_main2sub_ldi);
EXPORT_SYMBOL(tl2796_switch_sub2main_ldi);
EXPORT_SYMBOL(tl2796_set_working_ldi);
EXPORT_SYMBOL(tl2796_set_working_ldi_in_isr);
#endif


static int s5p_lcd_set_power(struct lcd_device *ld, int power)
{
	struct s5p_lcd *lcd = lcd_get_data(ld);
	
#if defined(CONFIG_MACH_S5PC110_LATONA)
	struct s5p_tft_panel_data *pdata = lcd->data;
#else
	struct s5p_panel_data *pdata = lcd->data;
#endif

	printk(KERN_DEBUG "s5p_lcd_set_power is called: %d", power);

	if (power)
		s6e63m0_panel_send_sequence(lcd, pdata->display_on);
	else
		s6e63m0_panel_send_sequence(lcd, pdata->display_off);

	return 0;
}

static int s5p_lcd_check_fb(struct lcd_device *lcddev, struct fb_info *fi)
{
	return 0;
}

struct lcd_ops s5p_lcd_ops = {
	.set_power = s5p_lcd_set_power,
	.check_fb = s5p_lcd_check_fb,
};

static ssize_t gammaset_file_cmd_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct s5p_lcd *lcd = dev_get_drvdata(dev);

	gprintk("called %s\n", __func__);

	return sprintf(buf, "%u\n", lcd->bl);
}
static ssize_t gammaset_file_cmd_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	struct s5p_lcd *lcd = dev_get_drvdata(dev);
	int value;

	sscanf(buf, "%d", &value);

	if ((lcd->ldi_enable) && ((value == 0) || (value == 1))) {
		printk("[gamma set] in gammaset_file_cmd_store, input value = %d\n", value);
		if (value != lcd->on_19gamma)	{
			lcd->on_19gamma = value;
			update_brightness(lcd);
		}
	}

	return size;
}

static DEVICE_ATTR(gammaset_file_cmd, 0664, gammaset_file_cmd_show, gammaset_file_cmd_store);

static ssize_t aclset_file_cmd_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct s5p_lcd *lcd = dev_get_drvdata(dev);
	gprintk("called %s\n", __func__);

	return sprintf(buf, "%u\n", lcd->acl_enable);
}
static ssize_t aclset_file_cmd_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct s5p_lcd *lcd = dev_get_drvdata(dev);
	int value;

	sscanf(buf, "%d", &value);

	if ((lcd->ldi_enable) && ((value == 0) || (value == 1))) {
		printk(KERN_INFO "[acl set] in aclset_file_cmd_store, input value = %d\n", value);
		if (lcd->acl_enable != value) {
			lcd->acl_enable = value;
#if defined(CONFIG_MACH_S5PC110_LATONA)
			Lcd_cabc_off(lcd);
#else  // CONFIG_MACH_S5PC110_LATONA

#ifdef CONFIG_FB_S3C_TL2796_ACL
			update_acl(lcd);
#endif  // CONFIG_FB_S3C_TL2796_ACL

#endif  // CONFIG_MACH_S5PC110_LATONA
		}
	}

	return size;
}

static DEVICE_ATTR(aclset_file_cmd, 0664, aclset_file_cmd_show, aclset_file_cmd_store);

static ssize_t lcdtype_file_cmd_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "SMD_AMS397GE03\n");
}

static ssize_t lcdtype_file_cmd_store(
        struct device *dev, struct device_attribute *attr,
        const char *buf, size_t size)
{
    return size;
}
static DEVICE_ATTR(lcdtype_file_cmd, 0664, lcdtype_file_cmd_show, lcdtype_file_cmd_store);

static int s5p_bl_update_status(struct backlight_device *bd)
{
	struct s5p_lcd *lcd = bl_get_data(bd);
	int bl = bd->props.brightness;

	pr_debug("\nupdate status brightness %d\n",
				bd->props.brightness);

	if (bl < 0 || bl > 255)
		return -EINVAL;

	mutex_lock(&lcd->lock);

	lcd->bl = bl;

	if (lcd->ldi_enable) {
		pr_debug("\n bl :%d\n", bl);
		update_brightness(lcd);
	}

	mutex_unlock(&lcd->lock);

	return 0;
}


static int s5p_bl_get_brightness(struct backlight_device *bd)
{
	struct s5p_lcd *lcd = bl_get_data(bd);

	printk(KERN_DEBUG "\n reading brightness\n");

	return lcd->bl;
}

const struct backlight_ops s5p_bl_ops = {
	.update_status = s5p_bl_update_status,
	.get_brightness = s5p_bl_get_brightness,
};

void tl2796_early_suspend(struct early_suspend *h)
{
	struct s5p_lcd *lcd = container_of(h, struct s5p_lcd, early_suspend);

	pr_info("%s called\n", __func__);

#if defined(CONFIG_MACH_S5PC110_PRESTIGE)
	s3c_gpio_setpin(GPIO_KEY_BL_EN, 0);
#endif

	tl2796_ldi_disable(lcd);

	current_gamma_value = -1;

	return ;
}
void tl2796_late_resume(struct early_suspend *h)
{
	struct s5p_lcd *lcd = container_of(h, struct s5p_lcd, early_suspend);

	pr_info("%s called\n", __func__);

	tl2796_ldi_enable(lcd);

#if defined(CONFIG_MACH_S5PC110_PRESTIGE)
	if (working_lcd == MAIN_LCD)	
	{
		if(is_keypad_backlight_night==0) //&& is_debug_screen==0)
			s3c_gpio_setpin(GPIO_KEY_BL_EN, 1);
	}
#endif

	return ;
}

#if defined(CONFIG_MACH_S5PC110_LATONA)
void tl2796_check_lcdtype(struct s5p_lcd *lcd) //sjlee_0212 (check LCD panel type)
{
	u16 lcd_id_check[3];
	int err;

	err = gpio_request(GPIO_LCD_ID, "LCD_ID");

	if (err) {
		  printk(KERN_ERR "failed to request LCD_ID1 \n");
		  return -1;
	}
	err = gpio_request(GPIO_LCD_ID2, "LCD_ID2");

	if (err) {
		  printk(KERN_ERR "failed to request LCD_ID2 \n");
		  return -1;
	}

	gpio_direction_input(GPIO_LCD_ID);
	gpio_direction_input(GPIO_LCD_ID2);

	if((gpio_get_value(GPIO_LCD_ID)==0) && ( gpio_get_value(GPIO_LCD_ID2) != 0))
		lcd_type = LCD_TYPE_SONY; // Sony Panel
	else
		lcd_type = LCD_TYPE_HITACHI_1; // hitachi Panel

	gpio_free(GPIO_LCD_ID);
	gpio_free(GPIO_LCD_ID2);

	printk("LCD_ID1=0x%x, LCD_ID2=0x%x, LCDTYPE=%s\n", gpio_get_value(GPIO_LCD_ID), gpio_get_value(GPIO_LCD_ID2), (lcd_type ? "SONY" : "Hitachi") ); 

	if(lcd_type == LCD_TYPE_SONY) {
		s6e63m0_spi_read_driver(0x00, 0xDA);
		lcd_id_check[0] = lcd_id_arr[4];
		msleep(1);

		s6e63m0_spi_read_driver(0x00, 0xDB);
		lcd_id_check[1] = lcd_id_arr[4];
		msleep(1);

		s6e63m0_spi_read_driver(0x00, 0xDC);
		lcd_id_check[2] = lcd_id_arr[4];
		msleep(1);

		printk("LCD_ID1=%x, LCD_ID2=%x, LCD_ID3=%x\n", lcd_id_check[0], lcd_id_check[1], lcd_id_check[2]); 

		if(lcd_id_check[0]==0x55 && lcd_id_check[1]==0x6C && lcd_id_check[2]==0xC0)
			lcd_type = LCD_TYPE_HYDIS; //HYDIS Module
		else if(lcd_id_check[0]==0x55 && lcd_id_check[1]==0x68 && lcd_id_check[2]==0xC0)
			lcd_type = LCD_TYPE_HYDIS; // HYDIS Panel
		else lcd_type = LCD_TYPE_SONY; //Sony Panel //sjlee_0222 (default panel is Sony)
	}
}
#endif

static int __devinit tl2796_probe(struct spi_device *spi)
{
#if !defined(CONFIG_MACH_S5PC110_PRESTIGE)
	struct s5p_lcd *lcd;
	int ret;

	lcd = kzalloc(sizeof(struct s5p_lcd), GFP_KERNEL);
	if (!lcd) {
		pr_err("failed to allocate for lcd\n");
		ret = -ENOMEM;
		goto err_alloc;
	}
	mutex_init(&lcd->lock);

	spi->bits_per_word = 9;
	if (spi_setup(spi)) {
		pr_err("failed to setup spi\n");
		ret = -EINVAL;
		goto err_setup;
	}

	lcd->g_spi = spi;
	lcd->dev = &spi->dev;
	lcd->bl = 255;

	if (!spi->dev.platform_data) {
		dev_err(lcd->dev, "failed to get platform data\n");
		ret = -EINVAL;
		goto err_setup;
	}
#if defined(CONFIG_MACH_S5PC110_LATONA)
	lcd->data = (struct s5p_tft_panel_data *)spi->dev.platform_data;
#else
	lcd->data = (struct s5p_panel_data *)spi->dev.platform_data;
#endif
	
#if defined(CONFIG_MACH_S5PC110_LATONA)
	if (!lcd->data->seq_set || !lcd->data->sleep_in ||
		!lcd->data->display_on || !lcd->data->display_off ||
		!lcd->data->brightness_set) {
		dev_err(lcd->dev, "Invalid platform data\n");
		ret = -EINVAL;
		goto err_setup;
	}
#else  // CONFIG_MACH_S5PC110_LATONA
	if (!lcd->data->gamma19_table || !lcd->data->gamma19_table ||
		!lcd->data->seq_display_set || !lcd->data->seq_etc_set ||
		!lcd->data->display_on || !lcd->data->display_off ||
		!lcd->data->standby_on || !lcd->data->standby_off ||
		!lcd->data->acl_init || !lcd->data->acl_table ||
		!lcd->data->gamma_update) {
		dev_err(lcd->dev, "Invalid platform data\n");
		ret = -EINVAL;
		goto err_setup;
	}
#endif  // CONFIG_MACH_S5PC110_LATONA

	lcd->bl_dev = backlight_device_register("s5p_bl",
			&spi->dev, lcd, &s5p_bl_ops, NULL);
	if (!lcd->bl_dev) {
		dev_err(lcd->dev, "failed to register backlight\n");
		ret = -EINVAL;
		goto err_setup;
	}

	lcd->bl_dev->props.max_brightness = 255;

	lcd->lcd_dev = lcd_device_register("s5p_lcd",
			&spi->dev, lcd, &s5p_lcd_ops);
	if (!lcd->lcd_dev) {
		dev_err(lcd->dev, "failed to register lcd\n");
		ret = -EINVAL;
		goto err_setup_lcd;
	}

	lcd->gammaset_class = class_create(THIS_MODULE, "gammaset");
	if (IS_ERR(lcd->gammaset_class))
		pr_err("Failed to create class(gammaset_class)!\n");

	lcd->switch_gammaset_dev = device_create(lcd->gammaset_class, &spi->dev, 0, lcd, "switch_gammaset");
	if (!lcd->switch_gammaset_dev) {
		dev_err(lcd->dev, "failed to register switch_gammaset_dev\n");
		ret = -EINVAL;
		goto err_setup_gammaset;
	}

	if (device_create_file(lcd->switch_gammaset_dev, &dev_attr_gammaset_file_cmd) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_gammaset_file_cmd.attr.name);

	lcd->acl_enable = 0;
	lcd->cur_acl = 0;

	lcd->acl_class = class_create(THIS_MODULE, "aclset");
	if (IS_ERR(lcd->acl_class))
		pr_err("Failed to create class(acl_class)!\n");

	lcd->switch_aclset_dev = device_create(lcd->acl_class, &spi->dev, 0, lcd, "switch_aclset");
	if (IS_ERR(lcd->switch_aclset_dev))
		pr_err("Failed to create device(switch_aclset_dev)!\n");

	if (device_create_file(lcd->switch_aclset_dev, &dev_attr_aclset_file_cmd) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_aclset_file_cmd.attr.name);

#if defined(CONFIG_MACH_S5PC110_LATONA)
	if (device_create_file(lcd->switch_aclset_dev, &dev_attr_cabc_mode) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_cabc_mode.attr.name);	
#endif

	 if (sec_class == NULL)
	 	sec_class = class_create(THIS_MODULE, "sec");
	 if (IS_ERR(sec_class))
                pr_err("Failed to create class(sec)!\n");

	 lcd->sec_lcdtype_dev = device_create(sec_class, NULL, 0, NULL, "sec_lcd");
	 if (IS_ERR(lcd->sec_lcdtype_dev))
	 	pr_err("Failed to create device(ts)!\n");

	 if (device_create_file(lcd->sec_lcdtype_dev, &dev_attr_lcdtype_file_cmd) < 0)
	 	pr_err("Failed to create device file(%s)!\n", dev_attr_lcdtype_file_cmd.attr.name);
	
#ifdef CONFIG_FB_S3C_MDNIE
	init_mdnie_class();  //set mDNIe UI mode, Outdoormode
#endif

	spi_set_drvdata(spi, lcd);

#if defined(CONFIG_MACH_S5PC110_LATONA) //sjlee_0212
	tl2796_check_lcdtype(lcd);
#endif

	tl2796_ldi_enable(lcd);
#ifdef CONFIG_HAS_EARLYSUSPEND
	lcd->early_suspend.suspend = tl2796_early_suspend;
	lcd->early_suspend.resume = tl2796_late_resume;
	lcd->early_suspend.level = EARLY_SUSPEND_LEVEL_DISABLE_FB - 1;
	register_early_suspend(&lcd->early_suspend);
#endif
	pr_info("tl2796_probe successfully proved\n");

	return 0;
err_setup_gammaset:
	lcd_device_unregister(lcd->lcd_dev);

err_setup_lcd:
	backlight_device_unregister(lcd->bl_dev);

err_setup:
	mutex_destroy(&lcd->lock);
	kfree(lcd);

err_alloc:
	return ret;

#else  // CONFIG_MACH_S5PC110_PRESTIGE

	int ret;

#if 0 //!defined(CONFIG_MACH_S5PC110_PRESTIGE)
	struct s5p_lcd *lcd;
#endif

#if 0 //!defined(CONFIG_MACH_S5PC110_PRESTIGE)
	sLcd = kzalloc(sizeof(struct s5p_lcd), GFP_KERNEL);
	if (!sLcd) {
		pr_err("failed to allocate for lcd\n");
		ret = -ENOMEM;
		goto err_alloc;
	}
#endif
	mutex_init(&sLcd.lock);

	spi->bits_per_word = 9;
	if (spi_setup(spi)) {
		pr_err("failed to setup spi\n");
		ret = -EINVAL;
		goto err_setup;
	}

	sLcd.g_spi = spi;
	sLcd.dev = &spi->dev;
	sLcd.bl = 118;

	if (!spi->dev.platform_data) {
		dev_err(sLcd.dev, "failed to get platform data\n");
		ret = -EINVAL;
		goto err_setup;
	}
	sLcd.data = (struct s5p_panel_data *)spi->dev.platform_data;

	if (!sLcd.data->gamma19_table || !sLcd.data->gamma19_table ||
		!sLcd.data->seq_display_set || !sLcd.data->seq_etc_set ||
		!sLcd.data->display_on || !sLcd.data->display_off ||
		!sLcd.data->standby_on || !sLcd.data->standby_off ||
		!sLcd.data->acl_init || !sLcd.data->acl_table ||
		!sLcd.data->gamma_update) {
		dev_err(sLcd.dev, "Invalid platform data\n");
		ret = -EINVAL;
		goto err_setup;
	}

	sLcd.bl_dev = backlight_device_register("s5p_bl",
			&spi->dev, &sLcd, &s5p_bl_ops, NULL);
	if (!sLcd.bl_dev) {
		dev_err(sLcd.dev, "failed to register backlight\n");
		ret = -EINVAL;
		goto err_setup;
	}

	sLcd.bl_dev->props.max_brightness = 255;

	sLcd.lcd_dev = lcd_device_register("s5p_lcd",
			&spi->dev, &sLcd, &s5p_lcd_ops);
	if (!sLcd.lcd_dev) {
		dev_err(sLcd.dev, "failed to register lcd\n");
		ret = -EINVAL;
		goto err_setup_lcd;
	}

	sLcd.gammaset_class = class_create(THIS_MODULE, "gammaset");
	if (IS_ERR(sLcd.gammaset_class))
		pr_err("Failed to create class(gammaset_class)!\n");

	sLcd.switch_gammaset_dev = device_create(sLcd.gammaset_class, &spi->dev, 0, &sLcd, "switch_gammaset");
	if (!sLcd.switch_gammaset_dev) {
		dev_err(sLcd.dev, "failed to register switch_gammaset_dev\n");
		ret = -EINVAL;
		goto err_setup_gammaset;
	}

	if (device_create_file(sLcd.switch_gammaset_dev, &dev_attr_gammaset_file_cmd) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_gammaset_file_cmd.attr.name);

	sLcd.acl_enable = 0;
	sLcd.cur_acl = 0;

	sLcd.acl_class = class_create(THIS_MODULE, "aclset");
	if (IS_ERR(sLcd.acl_class))
		pr_err("Failed to create class(acl_class)!\n");

	sLcd.switch_aclset_dev = device_create(sLcd.acl_class, &spi->dev, 0, &sLcd, "switch_aclset");
	if (IS_ERR(sLcd.switch_aclset_dev))
		pr_err("Failed to create device(switch_aclset_dev)!\n");

	if (device_create_file(sLcd.switch_aclset_dev, &dev_attr_aclset_file_cmd) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_aclset_file_cmd.attr.name);

	 if (sec_class == NULL)
	 	sec_class = class_create(THIS_MODULE, "sec");
	 if (IS_ERR(sec_class))
                pr_err("Failed to create class(sec)!\n");

	 sLcd.sec_lcdtype_dev = device_create(sec_class, NULL, 0, NULL, "sec_lcd");
	 if (IS_ERR(sLcd.sec_lcdtype_dev))
	 	pr_err("Failed to create device(ts)!\n");

	 if (device_create_file(sLcd.sec_lcdtype_dev, &dev_attr_lcdtype_file_cmd) < 0)
	 	pr_err("Failed to create device file(%s)!\n", dev_attr_lcdtype_file_cmd.attr.name);
	
#ifdef CONFIG_FB_S3C_MDNIE
	init_mdnie_class();  //set mDNIe UI mode, Outdoormode
#endif

	spi_set_drvdata(spi, &sLcd);
	SetLDIEnabledFlag(1);
	tl2796_ldi_enable(&sLcd);
	
#ifdef CONFIG_HAS_EARLYSUSPEND
	sLcd.early_suspend.suspend = tl2796_early_suspend;
	sLcd.early_suspend.resume = tl2796_late_resume;
	sLcd.early_suspend.level = EARLY_SUSPEND_LEVEL_DISABLE_FB - 1;
	register_early_suspend(&sLcd.early_suspend);
#endif
	pr_info("tl2796_probe successfully proved\n");

	s3c_gpio_cfgpin(GPIO_HALL_SW, S3C_GPIO_INPUT);
	s3c_gpio_setpull(GPIO_HALL_SW, S3C_GPIO_PULL_UP);

	if((readl(S5PV210_GPH0DAT)) & (1 << 6)){
		// FLIP OPEN
		working_lcd = MAIN_LCD;
		tl2796_sub_ldi_sleep();
		tl2796_main_ldi_bl_on();
	} else	{
		// FLIP CLOSE
		working_lcd = SUB_LCD;
		tl2796_main_ldi_sleep();
		tl2796_sub_ldi_bl_on();
	}

	return 0;
err_setup_gammaset:
	lcd_device_unregister(sLcd.lcd_dev);

err_setup_lcd:
//	backlight_device_unregister(sLcd.bl_dev);

err_setup:
	mutex_destroy(&sLcd.lock);
#if 0 //!defined(CONFIG_MACH_S5PC110_PRESTIGE)
	kfree(lcd);
#endif

err_alloc:
	return ret;
#endif  // CONFIG_MACH_S5PC110_PRESTIGE
}

static int __devexit tl2796_remove(struct spi_device *spi)
{
	struct s5p_lcd *lcd = spi_get_drvdata(spi);

	unregister_early_suspend(&lcd->early_suspend);

#if !defined(CONFIG_MACH_S5PC110_PRESTIGE)
	backlight_device_unregister(lcd->bl_dev);
#endif
	tl2796_ldi_disable(lcd);

	kfree(lcd);

	return 0;
}

static struct spi_driver tl2796_driver = {
	.driver = {
		.name	= "tl2796",
		.owner	= THIS_MODULE,
	},
	.probe		= tl2796_probe,
	.remove		= __devexit_p(tl2796_remove),
};

static int __init tl2796_init(void)
{
	return spi_register_driver(&tl2796_driver);
}
static void __exit tl2796_exit(void)
{
	spi_unregister_driver(&tl2796_driver);
}

module_init(tl2796_init);
module_exit(tl2796_exit);


MODULE_AUTHOR("SAMSUNG");
MODULE_DESCRIPTION("s6e63m0 LDI driver");
MODULE_LICENSE("GPL");
