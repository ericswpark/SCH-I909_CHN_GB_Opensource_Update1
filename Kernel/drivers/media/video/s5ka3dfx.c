/*
 * Driver for S5KA3DFX (UXGA camera) from Samsung Electronics
 * 
 * 1/4" 2.0Mp CMOS Image Sensor SoC with an Embedded Image Processor
 *
 * Copyright (C) 2009, Jinsung Yang <jsgood.yang@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/version.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-i2c-drv.h>
#include <media/s5ka3dfx_platform.h>

#ifdef CONFIG_VIDEO_SAMSUNG_V4L2
#include <linux/videodev2_samsung.h>
#endif

#include "s5ka3dfx.h"
#include "camsensor_regset_s5ka3dfx.h" //CTC_W899_seungjoo.yi
#include <mach/gpio.h>
#include <plat/gpio-cfg.h>
#include <mach/regs-gpio.h>
//#include <mach/max8998_function.h>

//#define VGA_CAM_DEBUG

#ifdef VGA_CAM_DEBUG
#define dev_dbg	dev_err
#endif

#define S5KA3DFX_DRIVER_NAME	"S5KA3DFX"

/* Default resolution & pixelformat. plz ref s5ka3dfx_platform.h */
#define DEFAULT_RES			WVGA				/* Index of resoultion */
#define DEFAUT_FPS_INDEX	S5KA3DFX_15FPS
#define DEFAULT_FMT			V4L2_PIX_FMT_UYVY	/* YUV422 */
#define POLL_TIME_MS		10

/*
 * Specification
 * Parallel : ITU-R. 656/601 YUV422, RGB565, RGB888 (Up to VGA), RAW10 
 * Serial : MIPI CSI2 (single lane) YUV422, RGB565, RGB888 (Up to VGA), RAW10
 * Resolution : 1280 (H) x 1024 (V)
 * Image control : Brightness, Contrast, Saturation, Sharpness, Glamour
 * Effect : Mono, Negative, Sepia, Aqua, Sketch
 * FPS : 15fps @full resolution, 30fps @VGA, 24fps @720p
 * Max. pixel clock frequency : 48MHz(upto)
 * Internal PLL (6MHz to 27MHz input frequency)
 */

static int s5ka3dfx_init(struct v4l2_subdev *sd, u32 val);		//for fixing build error	//s1_camera [ Defense process by ESD input ]

/* Camera functional setting values configured by user concept */
struct s5ka3dfx_userset {
	signed int exposure_bias;	/* V4L2_CID_EXPOSURE */
	unsigned int ae_lock;
	unsigned int awb_lock;
	unsigned int auto_wb;	/* V4L2_CID_AUTO_WHITE_BALANCE */
	unsigned int manual_wb;	/* V4L2_CID_WHITE_BALANCE_PRESET */
	unsigned int wb_temp;	/* V4L2_CID_WHITE_BALANCE_TEMPERATURE */
	unsigned int effect;	/* Color FX (AKA Color tone) */
	unsigned int contrast;	/* V4L2_CID_CONTRAST */
	unsigned int saturation;	/* V4L2_CID_SATURATION */
	unsigned int sharpness;		/* V4L2_CID_SHARPNESS */
	unsigned int glamour;
};

struct s5ka3dfx_state {
	struct s5ka3dfx_platform_data *pdata;
	struct v4l2_subdev sd;
	struct v4l2_pix_format pix;
	struct v4l2_fract timeperframe;
	struct s5ka3dfx_userset userset;
	int framesize_index;
	int freq;	/* MCLK in KHz */
	int is_mipi;
	int isize;
	int ver;
	int fps;
	int vt_mode; /*For VT camera*/
	int check_dataline;
	int check_previewdata;
};

enum {
	S5KA3DFX_PREVIEW_VGA,
} S5KA3DFX_FRAME_SIZE;

struct s5ka3dfx_enum_framesize {
	unsigned int index;
	unsigned int width;
	unsigned int height;	
};

struct s5ka3dfx_enum_framesize s5ka3dfx_framesize_list[] = {
	{S5KA3DFX_PREVIEW_VGA, 640, 480}
};

static inline struct s5ka3dfx_state *to_state(struct v4l2_subdev *sd)
{
	return container_of(sd, struct s5ka3dfx_state, sd);
}

//s1_camera [ Defense process by ESD input ] _[
//CTC_MAIN_seungjoo.yi

extern unsigned int jupiter_gpio_table[][8];
#if 0
static int s5ka3dfx_power_on(void)
{
	int err;
	u32 i, gpio;

	printk(KERN_DEBUG "s5ka3dfx_power_on_s5ka3dfx\n");

	for(i=0; i<50; i++){
		gpio = jupiter_gpio_table[i][0];

		if((gpio >= S5PV210_GPE0(0)) && (gpio <= S5PV210_GPE1(4))) {
			s3c_gpio_set_drvstrength(gpio, S3C_GPIO_DRVSTR_4X);
		}
	}

	/* CAM_VGA_nSTBY - GPC1(1)  */ 
	err = gpio_request(GPIO_CAM_VGA_nSTBY, "GPC1");

	if (err) {
		printk(KERN_ERR "failed to request GPC1 for camera control\n");

		return err;
	}

	/* CAM_VGA_nRST - GPE1(4) */
	err = gpio_request(GPIO_CAM_VGA_nRST, "GPE1");

	if (err) {
		printk(KERN_ERR "failed to request GPE1 for camera control\n");

		return err;
	}

	/* CAM_IO_EN - GPC0(0) */
	err = gpio_request(GPIO_CAM_IO_EN, "GPC0");

	if(err) {
		printk(KERN_ERR "failed to request GPC0 for camera control\n");

		return err;
	}

	// Turn CAM_SENSOR_A2.8V on
	gpio_direction_output(GPIO_CAM_IO_EN, 0);
	gpio_set_value(GPIO_CAM_IO_EN, 1);

	mdelay(1);

	// Turn CAM_ISP_RAM_1.8V on
	//Set_MAX8998_PM_OUTPUT_Voltage(LDO14, VCC_1p800);
	//Set_MAX8998_PM_REG(ELDO14, 1);

	mdelay(1);

	// Turn CAM_ISP_HOST_1.8V on
	//Set_MAX8998_PM_OUTPUT_Voltage(LDO15, VCC_1p700);
	//Set_MAX8998_PM_REG(ELDO15, 1);

	mdelay(1);

	gpio_free(GPIO_CAM_IO_EN);	
	mdelay(1);

	// CAM_VGA_nSTBY  HIGH		
	gpio_direction_output(GPIO_CAM_VGA_nSTBY, 0);
	gpio_set_value(GPIO_CAM_VGA_nSTBY, 1);

	mdelay(1);

	// Mclk enable
	s3c_gpio_cfgpin(GPIO_CAM_MCLK, S5PV210_GPE1_3_CAM_A_CLKOUT);

	mdelay(1);

	// CAM_VGA_nRST  HIGH		
	gpio_direction_output(GPIO_CAM_VGA_nRST, 0);
	gpio_set_value(GPIO_CAM_VGA_nRST, 1);		

	mdelay(4);

	gpio_free(GPIO_CAM_VGA_nSTBY);
	gpio_free(GPIO_CAM_VGA_nRST);	

	return 0;
}

//CTC_MAIN_seungjoo.yi
static int s5ka3dfx_power_off(void)
{
	int err;

	printk(KERN_DEBUG "s5ka3dfx_power_off\n");

	/* CAM_VGA_nSTBY - GPC1(1)  */
	err = gpio_request(GPIO_CAM_VGA_nSTBY, "GPC1");

	if (err) {
		printk(KERN_ERR "failed to request GPC1 for camera control\n");

		return err;
	}

	/* CAM_VGA_nRST - GPE1(4) */
	err = gpio_request(GPIO_CAM_VGA_nRST, "GPE1");

	if (err) {
		printk(KERN_ERR "failed to request GPE1 for camera control\n");

		return err;
	}


	// CAM_VGA_nRST  LOW		
	gpio_direction_output(GPIO_CAM_VGA_nRST, 1);
	gpio_set_value(GPIO_CAM_VGA_nRST, 0);

	mdelay(1);

	// Mclk disable
	s3c_gpio_cfgpin(GPIO_CAM_MCLK, 0);

	mdelay(1);

	// CAM_VGA_nSTBY  LOW		
	gpio_direction_output(GPIO_CAM_VGA_nSTBY, 1);
	gpio_set_value(GPIO_CAM_VGA_nSTBY, 0);

	mdelay(1);

	/* CAM_IO_EN - GPC0(0) */
	err = gpio_request(GPIO_CAM_IO_EN, "GPC0");

	if(err) {
		printk(KERN_ERR "failed to request GPC0 for camera control\n");

		return err;
	}

	// Turn CAM_SENSOR_A2.8V off
	gpio_direction_output(GPIO_CAM_IO_EN, 1);
	gpio_set_value(GPIO_CAM_IO_EN, 0);

	mdelay(1);

	// Turn CAM_ISP_RAM_1.8V off
	//Set_MAX8998_PM_REG(ELDO14, 0);

	// Turn CAM_ISP_HOST_1.8V off
	//Set_MAX8998_PM_REG(ELDO15, 0);
	
	gpio_free(GPIO_CAM_IO_EN);

	gpio_free(GPIO_CAM_VGA_nSTBY);
	gpio_free(GPIO_CAM_VGA_nRST);	

	return 0;
}


static int s5ka3dfx_power_en(int onoff)
{
	if(onoff){
		s5ka3dfx_power_on();
	} else {
		s5ka3dfx_power_off();
	}

	return 0;
}
#endif


static int s5ka3dfx_i2c_write_multi(struct i2c_client *client,
				    unsigned short addr, unsigned int w_data)
{
	int retry_count = 5;
	unsigned char buf[2];
	struct i2c_msg msg = { client->addr, 0, 2, buf };
	int ret;

	buf[0] = addr;
	buf[1] = w_data;

#ifdef VGA_CAM_DEBUG
	int i;
	for (i = 0; i < 2; i++) {
		dev_err(&client->dev, "buf[%d] = %x  ", i, buf[i]);
		if (i == 1)
			dev_err(&client->dev, "\n");
	}
#endif

	while (retry_count--) {
		ret = i2c_transfer(client->adapter, &msg, 1);
		if (likely(ret == 1))
			break;
		msleep(POLL_TIME_MS);
	}
	if (ret != 1)
		dev_err(&client->dev, "I2C is not working.\n");

	return (ret == 1) ? 0 : -EIO;
}


static int s5ka3dfx_reset(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
        struct s5ka3dfx_platform_data *pdata;

        pdata = client->dev.platform_data;

        if (pdata->cam_power) {
                pdata->cam_power(0);
                msleep(5);
                pdata->cam_power(1);
                msleep(5);
                s5ka3dfx_init(sd, 0);
        }

        return 0;
/*
	s5ka3dfx_power_en(0);
	mdelay(5);
	s5ka3dfx_power_en(1);
	mdelay(5);
	s5ka3dfx_init(sd, 0);
	return 0;
*/
}
// _]

/*
 * S5KA3DFX register structure : 2bytes address, 2bytes value
 * retry on write failure up-to 5 times
 */
static inline int s5ka3dfx_write(struct v4l2_subdev *sd, u8 addr, u8 val)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct i2c_msg msg[1];
	unsigned char reg[2];
	int err = 0;
	int retry = 0;


	if (!client->adapter)
		return -ENODEV;

again:
	msg->addr = client->addr;
	msg->flags = 0;
	msg->len = 2;
	msg->buf = reg;

	reg[0] = addr & 0xff;
	reg[1] = val & 0xff;

	err = i2c_transfer(client->adapter, msg, 1);
	if (err >= 0)
		return err;	/* Returns here on success */

	/* abnormal case: retry 5 times */
	if (retry < 5) {
		dev_err(&client->dev, "%s: address: 0x%02x%02x, " \
			"value: 0x%02x%02x\n", __func__, \
			reg[0], reg[1], reg[2], reg[3]);
		retry++;
		goto again;
	}

	return err;
}

static int s5ka3dfx_i2c_write(struct v4l2_subdev *sd, unsigned char i2c_data[],
				unsigned char length)
{
	int ret = -1;
	int retry_count = 1;
	
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	unsigned char buf[length], i;
	struct i2c_msg msg = {client->addr, 0, length, buf};

	for (i = 0; i < length; i++) {
		buf[i] = i2c_data[i];
	}
	
#ifdef VGA_CAM_DEBUG
	printk("i2c cmd Length : %d\n", length);
	for (i = 0; i < length; i++) {
		printk("buf[%d] = %x  ", i, buf[i]);
		if(i == length)
			printk("\n");
	}
#endif

	while(retry_count--){
		ret  = i2c_transfer(client->adapter, &msg, 1);
		if(ret == 1)
			break;
		msleep(10);
	}

	return (ret == 1) ? 0 : -EIO;
}
#if 0	/* unused functions */
static int s5ka3dfx_write_regs(struct v4l2_subdev *sd, unsigned char regs[], 
				int size)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int i, err;

	for (i = 0; i < size; i++) {
		err = s5ka3dfx_i2c_write(sd, &regs[i], sizeof(regs[i]));
		if (err < 0)
		{
			v4l_info(client, "%s: register set failed\n", \
			__func__);
			
			break;
		}
	}
	if(err < 0)
		return -EIO;	

	return 0;	/* FIXME */
}
#endif	/* unused functions */
//CTC_W899_seungjoo.yi
#if (IS_USE_VGA_TUNNING)

int	parsing_section;

static unsigned char my_util_hex_val(char hex)
{
	if ( hex >= 'a' && hex <= 'f' )
	{
		return (hex-'a'+10);
	}
	else if ( hex >= 'A' && hex <= 'F' )
	{
		return (hex - 'A' + 10 );
	}
	else if ( hex >= '0' && hex <= '9' )
	{
		return (hex - '0');
	}
	else
	{
		return 0;
	}
}

static unsigned char my_gets(char *buffer, char *line, int is_start)
{
	int          i;
	char*        _r_n_ptr;
	static char* buffer_ptr;

	memset(line, 0, 1024);

	if ( is_start )
		buffer_ptr = buffer;

	_r_n_ptr = strstr(buffer_ptr, "\r\n");

	//\n  
	if ( _r_n_ptr )
	{
		for ( i = 0 ; ; i++ )
		{
			if ( buffer_ptr+i == _r_n_ptr )
			{
				buffer_ptr = _r_n_ptr+1;
				break;
			}
			line[i] = buffer_ptr[i];
		}
		line[i] = '\0';

		return 1;
	}
//\n  
	else
	{
		if ( strlen(buffer_ptr) > 0 )
		{
			strcpy(line, buffer_ptr);
			return 0;
		}
		else
		{
			return 0;
		}
	}
}

static unsigned char my_atoi(char *str)
{
	unsigned int i,j=0;
	unsigned int val_len;
	unsigned int ret_val=0;

	if (str == NULL)
		return 0;

	val_len = strlen(str);

	//decimal
	if(val_len <= 2 || (strstr(str, "0x")==NULL && strstr(str, "0X")==NULL ))
	{
		for(i=0 ; i<val_len ; i++, str++ ) {
			switch( *str ) {
				case '0'...'9':
					ret_val= 10 * ret_val + ( *str - '0' ) ;
					break ;
					
				default:
					break ;
		}
	}

	return ret_val;
	}

	//hex ex:0xa0c
	for (i = val_len-1 ; i >= 2 ; i--)
	{
		ret_val = ret_val + (my_util_hex_val(str[i])<<(j*4));
		j++;
	}

	return ret_val;
}

static unsigned char res_util_trim(char* buff)
{
	int left_index;
	int right_index;
	int buff_len;
	int i;

	buff_len = strlen(buff);
	left_index = -1;
	right_index = -1;

	if (buff_len == 0)
		return 0;

	/* left index(시작에서 처음으로 white space가 없는 글자)를 찾는다. */
	for (i=0; i<buff_len; i++)
	{
		if (buff[i] != ' ' && buff[i] != '\t' && buff[i] != '\n' && buff[i] != '\r')
		{
			left_index = i;
			break;
		}
	}

	/* right index(끝에서 처음으로 white space가 없는 글자)를 찾는다. */
	for (i=buff_len-1; i>=0; i--)
	{
		if (buff[i] != ' ' && buff[i] != '\t' && buff[i] != '\n' && buff[i] != '\r')
		{
			right_index = i;
			buff[i+1] = '\0';
			break;
		}
	}

	if (left_index == -1 && right_index == -1)
		strcpy(buff, "");
	else if (left_index <= right_index)
		strcpy(buff, buff+left_index);
	else
		return -EINVAL;

	return 0;
}

static unsigned char insert_register_table(char* line)
{
  int  i;
  int  is_addr = 1;
  char reg_addr_str[5];
  char reg_val_str[5];
  int  reg_addr_str_idx = 0;
  int  reg_val_str_idx = 0;
  unsigned char reg_addr;
  unsigned char reg_val;

  //주석일 경우 건너 뛴다.
  res_util_trim(line);
  if (strlen(line) == 0 || (line[0] == '/' && line[1] == '/'))
    return 0;

  for (i=0; ; i++)
  {
    // address
    if (is_addr == 1)
    {
      if (line[i] == ' ' || line[i] == '\t')
        continue;
      if (line[i] == ',')
      //if (line[i] == ',')
      {
        is_addr = 0;
        continue;
      }
      reg_addr_str[reg_addr_str_idx++] = line[i];

    }
    // val
    else
    {
      if (line[i] == ' ' || line[i] == '\t' || line[i] == '/' || line[i] == '\0')
        break;
      reg_val_str[reg_val_str_idx++] = line[i];
    }
  }

  reg_addr_str[reg_addr_str_idx] = '\0';
  reg_val_str[reg_val_str_idx] = '\0';

  reg_addr = my_atoi(reg_addr_str);
  reg_val = my_atoi(reg_val_str);

  if (parsing_section == INIT_SECTION)
  {
    camsensor_init_table[camsensor_init_index][0] = reg_addr;
    camsensor_init_table[camsensor_init_index][1] = reg_val;
    camsensor_init_index++;
  }
  else if (parsing_section == INIT_VT_SECTION)
  {
    camsensor_init_vt_table[camsensor_init_vt_index][0] = reg_addr;
    camsensor_init_vt_table[camsensor_init_vt_index][1] = reg_val;
    camsensor_init_vt_index++;
  }
  else if (parsing_section == EV_M5_SECTION)
  {
    camsensor_ev_m5_table[camsensor_ev_m5_index][0] = reg_addr;
    camsensor_ev_m5_table[camsensor_ev_m5_index][1] = reg_val;
    camsensor_ev_m5_index++;
  }
  else if (parsing_section == EV_M4_SECTION)
  {
    camsensor_ev_m4_table[camsensor_ev_m4_index][0] = reg_addr;
    camsensor_ev_m4_table[camsensor_ev_m4_index][1] = reg_val;
    camsensor_ev_m4_index++;
  }
  else if (parsing_section == EV_M3_SECTION)
  {
    camsensor_ev_m3_table[camsensor_ev_m3_index][0] = reg_addr;
    camsensor_ev_m3_table[camsensor_ev_m3_index][1] = reg_val;
    camsensor_ev_m3_index++;
  }
  else if (parsing_section == EV_M2_SECTION)
  {
    camsensor_ev_m2_table[camsensor_ev_m2_index][0] = reg_addr;
    camsensor_ev_m2_table[camsensor_ev_m2_index][1] = reg_val;
    camsensor_ev_m2_index++;
  }
  else if (parsing_section == EV_M1_SECTION)
  {
    camsensor_ev_m1_table[camsensor_ev_m1_index][0] = reg_addr;
    camsensor_ev_m1_table[camsensor_ev_m1_index][1] = reg_val;
    camsensor_ev_m1_index++;
  }
  else if (parsing_section == EV_DEFAULT_SECTION)
  {
    camsensor_ev_default_table[camsensor_ev_default_index][0] = reg_addr;
    camsensor_ev_default_table[camsensor_ev_default_index][1] = reg_val;
    camsensor_ev_default_index++;
  }
  else if (parsing_section == EV_P1_SECTION)
  {
    camsensor_ev_p1_table[camsensor_ev_p1_index][0] = reg_addr;
    camsensor_ev_p1_table[camsensor_ev_p1_index][1] = reg_val;
    camsensor_ev_p1_index++;
  }
  else if (parsing_section == EV_P2_SECTION)
  {
    camsensor_ev_p2_table[camsensor_ev_p2_index][0] = reg_addr;
    camsensor_ev_p2_table[camsensor_ev_p2_index][1] = reg_val;
    camsensor_ev_p2_index++;
  }
  else if (parsing_section == EV_P3_SECTION)
  {
    camsensor_ev_p3_table[camsensor_ev_p3_index][0] = reg_addr;
    camsensor_ev_p3_table[camsensor_ev_p3_index][1] = reg_val;
    camsensor_ev_p3_index++;
  }
  else if (parsing_section == EV_P4_SECTION)
  {
    camsensor_ev_p4_table[camsensor_ev_p4_index][0] = reg_addr;
    camsensor_ev_p4_table[camsensor_ev_p4_index][1] = reg_val;
    camsensor_ev_p4_index++;
  }
  else if (parsing_section == EV_P5_SECTION)
  {
    camsensor_ev_p5_table[camsensor_ev_p5_index][0] = reg_addr;
    camsensor_ev_p5_table[camsensor_ev_p5_index][1] = reg_val;
    camsensor_ev_p5_index++;
  }
  else if (parsing_section == EV_VT_M5_SECTION)
  {
    camsensor_ev_vt_m5_table[camsensor_ev_vt_m5_index][0] = reg_addr;
    camsensor_ev_vt_m5_table[camsensor_ev_vt_m5_index][1] = reg_val;
    camsensor_ev_vt_m5_index++;
  }
  else if (parsing_section == EV_VT_M4_SECTION)
  {
    camsensor_ev_vt_m4_table[camsensor_ev_vt_m4_index][0] = reg_addr;
    camsensor_ev_vt_m4_table[camsensor_ev_vt_m4_index][1] = reg_val;
    camsensor_ev_vt_m4_index++;
  }
  else if (parsing_section == EV_VT_M3_SECTION)
  {
    camsensor_ev_vt_m3_table[camsensor_ev_vt_m3_index][0] = reg_addr;
    camsensor_ev_vt_m3_table[camsensor_ev_vt_m3_index][1] = reg_val;
    camsensor_ev_vt_m3_index++;
  }
  else if (parsing_section == EV_VT_M2_SECTION)
  {
    camsensor_ev_vt_m2_table[camsensor_ev_vt_m2_index][0] = reg_addr;
    camsensor_ev_vt_m2_table[camsensor_ev_vt_m2_index][1] = reg_val;
    camsensor_ev_vt_m2_index++;
  }
  else if (parsing_section == EV_VT_M1_SECTION)
  {
    camsensor_ev_vt_m1_table[camsensor_ev_vt_m1_index][0] = reg_addr;
    camsensor_ev_vt_m1_table[camsensor_ev_vt_m1_index][1] = reg_val;
    camsensor_ev_vt_m1_index++;
  }
  else if (parsing_section == EV_VT_DEFAULT_SECTION)
  {
    camsensor_ev_vt_default_table[camsensor_ev_vt_default_index][0] = reg_addr;
    camsensor_ev_vt_default_table[camsensor_ev_vt_default_index][1] = reg_val;
    camsensor_ev_vt_default_index++;
  }
  else if (parsing_section == EV_VT_P1_SECTION)
  {
    camsensor_ev_vt_p1_table[camsensor_ev_vt_p1_index][0] = reg_addr;
    camsensor_ev_vt_p1_table[camsensor_ev_vt_p1_index][1] = reg_val;
    camsensor_ev_vt_p1_index++;
  }
  else if (parsing_section == EV_VT_P2_SECTION)
  {
    camsensor_ev_vt_p2_table[camsensor_ev_vt_p2_index][0] = reg_addr;
    camsensor_ev_vt_p2_table[camsensor_ev_vt_p2_index][1] = reg_val;
    camsensor_ev_vt_p2_index++;
  }
  else if (parsing_section == EV_VT_P3_SECTION)
  {
    camsensor_ev_vt_p3_table[camsensor_ev_vt_p3_index][0] = reg_addr;
    camsensor_ev_vt_p3_table[camsensor_ev_vt_p3_index][1] = reg_val;
    camsensor_ev_vt_p3_index++;
  }
  else if (parsing_section == EV_VT_P4_SECTION)
  {
    camsensor_ev_vt_p4_table[camsensor_ev_vt_p4_index][0] = reg_addr;
    camsensor_ev_vt_p4_table[camsensor_ev_vt_p4_index][1] = reg_val;
    camsensor_ev_vt_p4_index++;
  }
  else if (parsing_section == EV_VT_P5_SECTION)
  {
    camsensor_ev_vt_p5_table[camsensor_ev_vt_p5_index][0] = reg_addr;
    camsensor_ev_vt_p5_table[camsensor_ev_vt_p5_index][1] = reg_val;
    camsensor_ev_vt_p5_index++;
  }
  else if (parsing_section == WB_AUTO_SECTION)
  {
    camsensor_wb_auto_table[camsensor_wb_auto_index][0] = reg_addr;
    camsensor_wb_auto_table[camsensor_wb_auto_index][1] = reg_val;
    camsensor_wb_auto_index++;
  }
  else if (parsing_section == WB_TUNGSTEN_SECTION)
  {
    camsensor_wb_tungsten_table[camsensor_wb_tungsten_index][0] = reg_addr;
    camsensor_wb_tungsten_table[camsensor_wb_tungsten_index][1] = reg_val;
    camsensor_wb_tungsten_index++;
  }
  else if (parsing_section == WB_FLUORESCENT_SECTION)
  {
    camsensor_wb_fluorescent_table[camsensor_wb_fluorescent_index][0] = reg_addr;
    camsensor_wb_fluorescent_table[camsensor_wb_fluorescent_index][1] = reg_val;
    camsensor_wb_fluorescent_index++;
  }
  else if (parsing_section == WB_SUNNY_SECTION)
  {
    camsensor_wb_sunny_table[camsensor_wb_sunny_index][0] = reg_addr;
    camsensor_wb_sunny_table[camsensor_wb_sunny_index][1] = reg_val;
    camsensor_wb_sunny_index++;
  }
  else if (parsing_section == WB_CLOUDY_SECTION)
  {
    camsensor_wb_cloudy_table[camsensor_wb_cloudy_index][0] = reg_addr;
    camsensor_wb_cloudy_table[camsensor_wb_cloudy_index][1] = reg_val;
    camsensor_wb_cloudy_index++;
  }
  else if (parsing_section == EFFECT_NONE_SECTION)
  {
    camsensor_effect_none_table[camsensor_effect_none_index][0] = reg_addr;
    camsensor_effect_none_table[camsensor_effect_none_index][1] = reg_val;
    camsensor_effect_none_index++;
  }
  else if (parsing_section == EFFECT_GRAY_SECTION)
  {
    camsensor_effect_gray_table[camsensor_effect_gray_index][0] = reg_addr;
    camsensor_effect_gray_table[camsensor_effect_gray_index][1] = reg_val;
    camsensor_effect_gray_index++;
  }
  else if (parsing_section == EFFECT_SEPIA_SECTION)
  {
    camsensor_effect_sepia_table[camsensor_effect_sepia_index][0] = reg_addr;
    camsensor_effect_sepia_table[camsensor_effect_sepia_index][1] = reg_val;
    camsensor_effect_sepia_index++;
  }
  else if (parsing_section == EFFECT_NEGATIVE_SECTION)
  {
    camsensor_effect_negative_table[camsensor_effect_negative_index][0] = reg_addr;
    camsensor_effect_negative_table[camsensor_effect_negative_index][1] = reg_val;
    camsensor_effect_negative_index++;
  }
  else if (parsing_section == EFFECT_AQUA_SECTION)
  {
    camsensor_effect_aqua_table[camsensor_effect_aqua_index][0] = reg_addr;
    camsensor_effect_aqua_table[camsensor_effect_aqua_index][1] = reg_val;
    camsensor_effect_aqua_index++;
  }
  else if (parsing_section == BLUR_NONE_SECTION)
  {
    camsensor_blur_none_table[camsensor_blur_none_index][0] = reg_addr;
    camsensor_blur_none_table[camsensor_blur_none_index][1] = reg_val;
    camsensor_blur_none_index++;
  }
  else if (parsing_section == BLUR_P1_SECTION)
  {
    camsensor_blur_p1_table[camsensor_blur_p1_index][0] = reg_addr;
    camsensor_blur_p1_table[camsensor_blur_p1_index][1] = reg_val;
    camsensor_blur_p1_index++;
  }
  else if (parsing_section == BLUR_P2_SECTION)
  {
    camsensor_blur_p2_table[camsensor_blur_p2_index][0] = reg_addr;
    camsensor_blur_p2_table[camsensor_blur_p2_index][1] = reg_val;
    camsensor_blur_p2_index++;
  }
  else if (parsing_section == BLUR_P3_SECTION)
  {
    camsensor_blur_p3_table[camsensor_blur_p3_index][0] = reg_addr;
    camsensor_blur_p3_table[camsensor_blur_p3_index][1] = reg_val;
    camsensor_blur_p3_index++;
  }
  else if (parsing_section == BLUR_VT_NONE_SECTION)
  {
    camsensor_blur_vt_none_table[camsensor_blur_vt_none_index][0] = reg_addr;
    camsensor_blur_vt_none_table[camsensor_blur_vt_none_index][1] = reg_val;
    camsensor_blur_vt_none_index++;
  }
  else if (parsing_section == BLUR_VT_P1_SECTION)
  {
    camsensor_blur_vt_p1_table[camsensor_blur_vt_p1_index][0] = reg_addr;
    camsensor_blur_vt_p1_table[camsensor_blur_vt_p1_index][1] = reg_val;
    camsensor_blur_vt_p1_index++;
  }
  else if (parsing_section == BLUR_VT_P2_SECTION)
  {
    camsensor_blur_vt_p2_table[camsensor_blur_vt_p2_index][0] = reg_addr;
    camsensor_blur_vt_p2_table[camsensor_blur_vt_p2_index][1] = reg_val;
    camsensor_blur_vt_p2_index++;
  }
  else if (parsing_section == BLUR_VT_P3_SECTION)
  {
    camsensor_blur_vt_p3_table[camsensor_blur_vt_p3_index][0] = reg_addr;
    camsensor_blur_vt_p3_table[camsensor_blur_vt_p3_index][1] = reg_val;
    camsensor_blur_vt_p3_index++;
  }
  else if (parsing_section == DATALINE_SECTION)
  {
    camsensor_dataline_table[camsensor_dataline_index][0] = reg_addr;
    camsensor_dataline_table[camsensor_dataline_index][1] = reg_val;
    camsensor_dataline_index++;
  }
  else if (parsing_section == DATALINE_STOP_SECTION)
  {
    camsensor_dataline_stop_table[camsensor_dataline_stop_index][0] = reg_addr;
    camsensor_dataline_stop_table[camsensor_dataline_stop_index][1] = reg_val;
    camsensor_dataline_stop_index++;
  }
  else if (parsing_section == FPS_7_SECTION)
  {
    camsensor_fps_7_table[camsensor_fps_7_index][0] = reg_addr;
    camsensor_fps_7_table[camsensor_fps_7_index][1] = reg_val;
    camsensor_fps_7_index++;
  }
  else if (parsing_section == FPS_10_SECTION)
  {
    camsensor_fps_10_table[camsensor_fps_10_index][0] = reg_addr;
    camsensor_fps_10_table[camsensor_fps_10_index][1] = reg_val;
    camsensor_fps_10_index++;
  }
  else if (parsing_section == FPS_15_SECTION)
  {
    camsensor_fps_15_table[camsensor_fps_15_index][0] = reg_addr;
    camsensor_fps_15_table[camsensor_fps_15_index][1] = reg_val;
    camsensor_fps_15_index++;
  }
  else if (parsing_section == VT_FPS_7_SECTION)
  {
    camsensor_vt_fps_7_table[camsensor_vt_fps_7_index][0] = reg_addr;
    camsensor_vt_fps_7_table[camsensor_vt_fps_7_index][1] = reg_val;
    camsensor_vt_fps_7_index++;
  }
  else if (parsing_section == VT_FPS_10_SECTION)
  {
    camsensor_vt_fps_10_table[camsensor_vt_fps_10_index][0] = reg_addr;
    camsensor_vt_fps_10_table[camsensor_vt_fps_10_index][1] = reg_val;
    camsensor_vt_fps_10_index++;
  }
  else if (parsing_section == VT_FPS_15_SECTION)
  {
    camsensor_vt_fps_15_table[camsensor_vt_fps_15_index][0] = reg_addr;
    camsensor_vt_fps_15_table[camsensor_vt_fps_15_index][1] = reg_val;
    camsensor_vt_fps_15_index++;
  }

  return 0;
}

static int check_parsing_section(char* line)
{
  if (strstr(line, CAMSENSOR_INIT) != NULL)
    parsing_section = INIT_SECTION;
  else if (strstr(line, CAMSENSOR_INIT_VT) != NULL)
    parsing_section = INIT_VT_SECTION;
  else if (strstr(line, CAMSENSOR_EV_M5) != NULL)
    parsing_section = EV_M5_SECTION;
  else if (strstr(line, CAMSENSOR_EV_M4) != NULL)
    parsing_section = EV_M4_SECTION;
  else if (strstr(line, CAMSENSOR_EV_M3) != NULL)
    parsing_section = EV_M3_SECTION;
  else if (strstr(line, CAMSENSOR_EV_M2) != NULL)
    parsing_section = EV_M2_SECTION;
  else if (strstr(line, CAMSENSOR_EV_M1) != NULL)
    parsing_section = EV_M1_SECTION;
  else if (strstr(line, CAMSENSOR_EV_DEFAULT) != NULL)
    parsing_section = EV_DEFAULT_SECTION;
  else if (strstr(line, CAMSENSOR_EV_P1) != NULL)
    parsing_section = EV_P1_SECTION;
  else if (strstr(line, CAMSENSOR_EV_P2) != NULL)
    parsing_section = EV_P2_SECTION;
  else if (strstr(line, CAMSENSOR_EV_P3) != NULL)
    parsing_section = EV_P3_SECTION;
  else if (strstr(line, CAMSENSOR_EV_P4) != NULL)
    parsing_section = EV_P4_SECTION;
  else if (strstr(line, CAMSENSOR_EV_P5) != NULL)
    parsing_section = EV_P5_SECTION;
  else if (strstr(line, CAMSENSOR_EV_VT_M5) != NULL)
    parsing_section = EV_VT_M5_SECTION;
  else if (strstr(line, CAMSENSOR_EV_VT_M4) != NULL)
    parsing_section = EV_VT_M4_SECTION;
  else if (strstr(line, CAMSENSOR_EV_VT_M3) != NULL)
    parsing_section = EV_VT_M3_SECTION;
  else if (strstr(line, CAMSENSOR_EV_VT_M2) != NULL)
    parsing_section = EV_VT_M2_SECTION;
  else if (strstr(line, CAMSENSOR_EV_VT_M1) != NULL)
    parsing_section = EV_VT_M1_SECTION;
  else if (strstr(line, CAMSENSOR_EV_VT_DEFAULT) != NULL)
    parsing_section = EV_VT_DEFAULT_SECTION;
  else if (strstr(line, CAMSENSOR_EV_VT_P1) != NULL)
    parsing_section = EV_VT_P1_SECTION;
  else if (strstr(line, CAMSENSOR_EV_VT_P2) != NULL)
    parsing_section = EV_VT_P2_SECTION;
  else if (strstr(line, CAMSENSOR_EV_VT_P3) != NULL)
    parsing_section = EV_VT_P3_SECTION;
  else if (strstr(line, CAMSENSOR_EV_VT_P4) != NULL)
    parsing_section = EV_VT_P4_SECTION;
  else if (strstr(line, CAMSENSOR_EV_VT_P5) != NULL)
    parsing_section = EV_VT_P5_SECTION;
  else if (strstr(line, CAMSENSOR_WB_AUTO) != NULL)
    parsing_section = WB_AUTO_SECTION;
  else if (strstr(line, CAMSENSOR_WB_TUNGSTEN) != NULL)
    parsing_section = WB_TUNGSTEN_SECTION;
  else if (strstr(line, CAMSENSOR_WB_FLUORESCENT) != NULL)
    parsing_section = WB_FLUORESCENT_SECTION;
  else if (strstr(line, CAMSENSOR_WB_SUNNY) != NULL)
    parsing_section = WB_SUNNY_SECTION;
  else if (strstr(line, CAMSENSOR_WB_CLOUDY) != NULL)
    parsing_section = WB_CLOUDY_SECTION;
  else if (strstr(line, CAMSENSOR_EFFECT_NONE) != NULL)
    parsing_section = EFFECT_NONE_SECTION;
  else if (strstr(line, CAMSENSOR_EFFECT_GRAY) != NULL)
    parsing_section = EFFECT_GRAY_SECTION;
  else if (strstr(line, CAMSENSOR_EFFECT_SEPIA) != NULL)
    parsing_section = EFFECT_SEPIA_SECTION;
  else if (strstr(line, CAMSENSOR_EFFECT_NEGATIVE) != NULL)
    parsing_section = EFFECT_NEGATIVE_SECTION;
  else if (strstr(line, CAMSENSOR_EFFECT_AQUA) != NULL)
    parsing_section = EFFECT_AQUA_SECTION;
  else if (strstr(line, CAMSENSOR_BLUR_NONE) != NULL)
    parsing_section = BLUR_NONE_SECTION;
  else if (strstr(line, CAMSENSOR_BLUR_P1) != NULL)
    parsing_section = BLUR_P1_SECTION;
  else if (strstr(line, CAMSENSOR_BLUR_P2) != NULL)
    parsing_section = BLUR_P2_SECTION;
  else if (strstr(line, CAMSENSOR_BLUR_P3) != NULL)
    parsing_section = BLUR_P3_SECTION;
  else if (strstr(line, CAMSENSOR_BLUR_VT_NONE) != NULL)
    parsing_section = BLUR_VT_NONE_SECTION;
  else if (strstr(line, CAMSENSOR_BLUR_VT_P1) != NULL)
    parsing_section = BLUR_VT_P1_SECTION;
  else if (strstr(line, CAMSENSOR_BLUR_VT_P2) != NULL)
    parsing_section = BLUR_VT_P2_SECTION;
  else if (strstr(line, CAMSENSOR_BLUR_VT_P3) != NULL)
    parsing_section = BLUR_VT_P3_SECTION;
  else if (strstr(line, CAMSENSOR_DATALINE) != NULL)
    parsing_section = DATALINE_SECTION;
  else if (strstr(line, CAMSENSOR_DATALINE_STOP) != NULL)
    parsing_section = DATALINE_STOP_SECTION;
  else if (strstr(line, CAMSENSOR_FPS_7) != NULL)
    parsing_section = FPS_7_SECTION;
  else if (strstr(line, CAMSENSOR_FPS_10) != NULL)
    parsing_section = FPS_10_SECTION;
  else if (strstr(line, CAMSENSOR_FPS_15) != NULL)
    parsing_section = FPS_15_SECTION;
  else if (strstr(line, CAMSENSOR_VT_FPS_7) != NULL)
    parsing_section = VT_FPS_7_SECTION;
  else if (strstr(line, CAMSENSOR_VT_FPS_10) != NULL)
    parsing_section = VT_FPS_10_SECTION;
  else if (strstr(line, CAMSENSOR_VT_FPS_15) != NULL)
    parsing_section = VT_FPS_15_SECTION;
  else
    return 0;

  return 1;
}

static int make_register_table(void)
{
	char			*buffer = NULL;
	char			line[1024];
	unsigned int		file_size;
	struct file		*pFile = NULL;
	mm_segment_t	old_fs;
	int			result = 0;

	printk("make_register_table is called...\n");

	pFile = filp_open(CAMERA_CONFIGURE_FILE, O_RDONLY, 0) ;

	if (pFile && (pFile!= 0xfffffffe))
	{
		old_fs = get_fs();
		set_fs(KERNEL_DS);
		
		file_size = pFile->f_op->llseek(pFile, 0, SEEK_END);
		pFile->f_op->llseek(pFile, 0, SEEK_SET);
		
		buffer = (char*)kmalloc(file_size+1, GFP_KERNEL);
		
		pFile->f_op->read(pFile, buffer, file_size, &pFile->f_pos);
		buffer[file_size] = '\0';
		
		filp_close(pFile, current->files);

		set_fs(old_fs);

		printk("File size : %d\n", file_size);
	}
	else
	{
		return -EINVAL;
	}

	parsing_section = 0;

	my_gets(buffer, line, 1);
	if (check_parsing_section(line)==0)
	{
		insert_register_table(line);
	}

	while (my_gets(buffer, line, 0))
	{
		if (check_parsing_section(line)==0)
		{
	  		insert_register_table(line);
		}
	}

	insert_register_table(line);

	kfree(buffer);
	buffer = NULL;

	return 0;
}
#endif //#if IS_USE_VGA_TUNNING

static int init_reg_table(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	v4l_info(client, "%s: i2c write multi register set\n", __func__);

#if !(IS_USE_VGA_TUNNING)

	camsensor_init_index = sizeof(camsensor_init_table)/sizeof(camsensor_init_table[0]);
	camsensor_init_vt_index = sizeof(camsensor_init_vt_table)/sizeof(camsensor_init_vt_table[0]);
	camsensor_ev_m5_index = sizeof(camsensor_ev_m5_table)/sizeof(camsensor_ev_m5_table[0]);
	camsensor_ev_m4_index = sizeof(camsensor_ev_m4_table)/sizeof(camsensor_ev_m4_table[0]);
	camsensor_ev_m3_index = sizeof(camsensor_ev_m3_table)/sizeof(camsensor_ev_m3_table[0]);
	camsensor_ev_m2_index = sizeof(camsensor_ev_m2_table)/sizeof(camsensor_ev_m2_table[0]);
	camsensor_ev_m1_index = sizeof(camsensor_ev_m1_table)/sizeof(camsensor_ev_m1_table[0]);
	camsensor_ev_default_index = sizeof(camsensor_ev_default_table)/sizeof(camsensor_ev_default_table[0]);
	camsensor_ev_p1_index = sizeof(camsensor_ev_p1_table)/sizeof(camsensor_ev_p1_table[0]);
	camsensor_ev_p2_index = sizeof(camsensor_ev_p2_table)/sizeof(camsensor_ev_p2_table[0]);
	camsensor_ev_p3_index = sizeof(camsensor_ev_p3_table)/sizeof(camsensor_ev_p3_table[0]);
	camsensor_ev_p4_index = sizeof(camsensor_ev_p4_table)/sizeof(camsensor_ev_p4_table[0]);
	camsensor_ev_p5_index = sizeof(camsensor_ev_p5_table)/sizeof(camsensor_ev_p5_table[0]);
	camsensor_ev_vt_m5_index = sizeof(camsensor_ev_vt_m5_table)/sizeof(camsensor_ev_vt_m5_table[0]);
	camsensor_ev_vt_m4_index = sizeof(camsensor_ev_vt_m4_table)/sizeof(camsensor_ev_vt_m4_table[0]);
	camsensor_ev_vt_m3_index = sizeof(camsensor_ev_vt_m3_table)/sizeof(camsensor_ev_vt_m3_table[0]);
	camsensor_ev_vt_m2_index = sizeof(camsensor_ev_vt_m2_table)/sizeof(camsensor_ev_vt_m2_table[0]);
	camsensor_ev_vt_m1_index = sizeof(camsensor_ev_vt_m1_table)/sizeof(camsensor_ev_vt_m1_table[0]);
	camsensor_ev_vt_default_index = sizeof(camsensor_ev_vt_default_table)/sizeof(camsensor_ev_vt_default_table[0]);
	camsensor_ev_vt_p1_index = sizeof(camsensor_ev_vt_p1_table)/sizeof(camsensor_ev_vt_p1_table[0]);
	camsensor_ev_vt_p2_index = sizeof(camsensor_ev_vt_p2_table)/sizeof(camsensor_ev_vt_p2_table[0]);
	camsensor_ev_vt_p3_index = sizeof(camsensor_ev_vt_p3_table)/sizeof(camsensor_ev_vt_p3_table[0]);
	camsensor_ev_vt_p4_index = sizeof(camsensor_ev_vt_p4_table)/sizeof(camsensor_ev_vt_p4_table[0]);
	camsensor_ev_vt_p5_index = sizeof(camsensor_ev_vt_p5_table)/sizeof(camsensor_ev_vt_p5_table[0]);
	camsensor_wb_auto_index = sizeof(camsensor_wb_auto_table)/sizeof(camsensor_wb_auto_table[0]);
	camsensor_wb_tungsten_index = sizeof(camsensor_wb_tungsten_table)/sizeof(camsensor_wb_tungsten_table[0]);
	camsensor_wb_fluorescent_index = sizeof(camsensor_wb_fluorescent_table)/sizeof(camsensor_wb_fluorescent_table[0]);
	camsensor_wb_sunny_index = sizeof(camsensor_wb_sunny_table)/sizeof(camsensor_wb_sunny_table[0]);
	camsensor_wb_cloudy_index = sizeof(camsensor_wb_cloudy_table)/sizeof(camsensor_wb_cloudy_table[0]);
	camsensor_effect_none_index = sizeof(camsensor_effect_none_table)/sizeof(camsensor_effect_none_table[0]);
	camsensor_effect_gray_index = sizeof(camsensor_effect_gray_table)/sizeof(camsensor_effect_gray_table[0]);
	camsensor_effect_sepia_index = sizeof(camsensor_effect_sepia_table)/sizeof(camsensor_effect_sepia_table[0]);
	camsensor_effect_negative_index = sizeof(camsensor_effect_negative_table)/sizeof(camsensor_effect_negative_table[0]);
	camsensor_effect_aqua_index = sizeof(camsensor_effect_aqua_table)/sizeof(camsensor_effect_aqua_table[0]);
	camsensor_blur_none_index = sizeof(camsensor_blur_none_table)/sizeof(camsensor_blur_none_table[0]);
	camsensor_blur_p1_index = sizeof(camsensor_blur_p1_table)/sizeof(camsensor_blur_p1_table[0]);
	camsensor_blur_p2_index = sizeof(camsensor_blur_p2_table)/sizeof(camsensor_blur_p2_table[0]);
	camsensor_blur_p3_index = sizeof(camsensor_blur_p3_table)/sizeof(camsensor_blur_p3_table[0]);
	camsensor_blur_vt_none_index = sizeof(camsensor_blur_vt_none_table)/sizeof(camsensor_blur_vt_none_table[0]);
	camsensor_blur_vt_p1_index = sizeof(camsensor_blur_vt_p1_table)/sizeof(camsensor_blur_vt_p1_table[0]);
	camsensor_blur_vt_p2_index = sizeof(camsensor_blur_vt_p2_table)/sizeof(camsensor_blur_vt_p2_table[0]);
	camsensor_blur_vt_p3_index = sizeof(camsensor_blur_vt_p3_table)/sizeof(camsensor_blur_vt_p3_table[0]);
	camsensor_dataline_index = sizeof(camsensor_dataline_table)/sizeof(camsensor_dataline_table[0]);
	camsensor_dataline_stop_index = sizeof(camsensor_dataline_stop_table)/sizeof(camsensor_dataline_stop_table[0]);
	camsensor_fps_7_index = sizeof(camsensor_fps_7_table)/sizeof(camsensor_fps_7_table[0]);
	camsensor_fps_10_index = sizeof(camsensor_fps_10_table)/sizeof(camsensor_fps_10_table[0]);
	camsensor_fps_15_index = sizeof(camsensor_fps_15_table)/sizeof(camsensor_fps_15_table[0]);
	camsensor_vt_fps_7_index = sizeof(camsensor_vt_fps_7_table)/sizeof(camsensor_vt_fps_7_table[0]);
	camsensor_vt_fps_10_index = sizeof(camsensor_vt_fps_10_table)/sizeof(camsensor_vt_fps_10_table[0]);
	camsensor_vt_fps_15_index = sizeof(camsensor_vt_fps_15_table)/sizeof(camsensor_vt_fps_15_table[0]);

#else //#if !(IS_USE_VGA_TUNNING)

	camsensor_init_index = 0;
	camsensor_init_vt_index = 0;
	camsensor_ev_m5_index = 0;
	camsensor_ev_m4_index = 0;
	camsensor_ev_m3_index = 0;
	camsensor_ev_m2_index = 0;
	camsensor_ev_m1_index = 0;
	camsensor_ev_default_index = 0;
	camsensor_ev_p1_index = 0;
	camsensor_ev_p2_index = 0;
	camsensor_ev_p3_index = 0;
	camsensor_ev_p4_index = 0;
	camsensor_ev_p5_index = 0;
	camsensor_ev_vt_m5_index = 0;
	camsensor_ev_vt_m4_index = 0;
	camsensor_ev_vt_m3_index = 0;
	camsensor_ev_vt_m2_index = 0;
	camsensor_ev_vt_m1_index = 0;
	camsensor_ev_vt_default_index = 0;
	camsensor_ev_vt_p1_index = 0;
	camsensor_ev_vt_p2_index = 0;
	camsensor_ev_vt_p3_index = 0;
	camsensor_ev_vt_p4_index = 0;
	camsensor_ev_vt_p5_index = 0;
	camsensor_wb_auto_index = 0;
	camsensor_wb_tungsten_index = 0;
	camsensor_wb_fluorescent_index = 0;
	camsensor_wb_sunny_index = 0;
	camsensor_wb_cloudy_index = 0;
	camsensor_effect_none_index = 0;
	camsensor_effect_gray_index = 0;
	camsensor_effect_sepia_index = 0;
	camsensor_effect_negative_index = 0;
	camsensor_effect_aqua_index = 0;
	camsensor_blur_none_index = 0;
	camsensor_blur_p1_index = 0;
	camsensor_blur_p2_index = 0;
	camsensor_blur_p3_index = 0;
	camsensor_blur_vt_none_index = 0;
	camsensor_blur_vt_p1_index = 0;
	camsensor_blur_vt_p2_index = 0;
	camsensor_blur_vt_p3_index = 0;
	camsensor_dataline_index = 0;
	camsensor_dataline_stop_index = 0;
	camsensor_fps_7_index = 0;
	camsensor_fps_10_index = 0;
	camsensor_fps_15_index = 0;
	camsensor_vt_fps_7_index = 0;
	camsensor_vt_fps_10_index = 0;
	camsensor_vt_fps_15_index = 0;

	memset(&camsensor_init_table, 0, sizeof(camsensor_init_table));
	memset(&camsensor_init_vt_table, 0, sizeof(camsensor_init_vt_table));
	memset(&camsensor_ev_m5_table, 0, sizeof(camsensor_ev_m5_table));
	memset(&camsensor_ev_m4_table, 0, sizeof(camsensor_ev_m4_table));
	memset(&camsensor_ev_m3_table, 0, sizeof(camsensor_ev_m3_table));
	memset(&camsensor_ev_m2_table, 0, sizeof(camsensor_ev_m2_table));
	memset(&camsensor_ev_m1_table, 0, sizeof(camsensor_ev_m1_table));
	memset(&camsensor_ev_default_table, 0, sizeof(camsensor_ev_default_table));
	memset(&camsensor_ev_p1_table, 0, sizeof(camsensor_ev_p1_table));
	memset(&camsensor_ev_p2_table, 0, sizeof(camsensor_ev_p2_table));
	memset(&camsensor_ev_p3_table, 0, sizeof(camsensor_ev_p3_table));
	memset(&camsensor_ev_p4_table, 0, sizeof(camsensor_ev_p4_table));
	memset(&camsensor_ev_p5_table, 0, sizeof(camsensor_ev_p5_table));
	memset(&camsensor_ev_vt_m5_table, 0, sizeof(camsensor_ev_vt_m5_table));
	memset(&camsensor_ev_vt_m4_table, 0, sizeof(camsensor_ev_vt_m4_table));
	memset(&camsensor_ev_vt_m3_table, 0, sizeof(camsensor_ev_vt_m3_table));
	memset(&camsensor_ev_vt_m2_table, 0, sizeof(camsensor_ev_vt_m2_table));
	memset(&camsensor_ev_vt_m1_table, 0, sizeof(camsensor_ev_vt_m1_table));
	memset(&camsensor_ev_vt_default_table, 0, sizeof(camsensor_ev_vt_default_table));
	memset(&camsensor_ev_vt_p1_table, 0, sizeof(camsensor_ev_vt_p1_table));
	memset(&camsensor_ev_vt_p2_table, 0, sizeof(camsensor_ev_vt_p2_table));
	memset(&camsensor_ev_vt_p3_table, 0, sizeof(camsensor_ev_vt_p3_table));
	memset(&camsensor_ev_vt_p4_table, 0, sizeof(camsensor_ev_vt_p4_table));
	memset(&camsensor_ev_vt_p5_table, 0, sizeof(camsensor_ev_vt_p5_table));
	memset(&camsensor_wb_auto_table, 0, sizeof(camsensor_wb_auto_table));
	memset(&camsensor_wb_tungsten_table, 0, sizeof(camsensor_wb_tungsten_table));
	memset(&camsensor_wb_fluorescent_table, 0, sizeof(camsensor_wb_fluorescent_table));
	memset(&camsensor_wb_sunny_table, 0, sizeof(camsensor_wb_sunny_table));
	memset(&camsensor_wb_cloudy_table, 0, sizeof(camsensor_wb_cloudy_table));
	memset(&camsensor_effect_none_table, 0, sizeof(camsensor_effect_none_table));
	memset(&camsensor_effect_gray_table, 0, sizeof(camsensor_effect_gray_table));
	memset(&camsensor_effect_sepia_table, 0, sizeof(camsensor_effect_sepia_table));
	memset(&camsensor_effect_negative_table, 0, sizeof(camsensor_effect_negative_table));
	memset(&camsensor_effect_aqua_table, 0, sizeof(camsensor_effect_aqua_table));
	memset(&camsensor_blur_none_table, 0, sizeof(camsensor_blur_none_table));
	memset(&camsensor_blur_p1_table, 0, sizeof(camsensor_blur_p1_table));
	memset(&camsensor_blur_p2_table, 0, sizeof(camsensor_blur_p2_table));
	memset(&camsensor_blur_p3_table, 0, sizeof(camsensor_blur_p3_table));
	memset(&camsensor_blur_vt_none_table, 0, sizeof(camsensor_blur_vt_none_table));
	memset(&camsensor_blur_vt_p1_table, 0, sizeof(camsensor_blur_vt_p1_table));
	memset(&camsensor_blur_vt_p2_table, 0, sizeof(camsensor_blur_vt_p2_table));
	memset(&camsensor_blur_vt_p3_table, 0, sizeof(camsensor_blur_vt_p3_table));
	memset(&camsensor_dataline_table, 0, sizeof(camsensor_dataline_table));
	memset(&camsensor_dataline_stop_table, 0, sizeof(camsensor_dataline_stop_table));
	memset(&camsensor_fps_7_table, 0, sizeof(camsensor_fps_7_table));
	memset(&camsensor_fps_10_table, 0, sizeof(camsensor_fps_10_table));
	memset(&camsensor_fps_15_table, 0, sizeof(camsensor_fps_15_table));
	memset(&camsensor_vt_fps_7_table, 0, sizeof(camsensor_vt_fps_7_table));
	memset(&camsensor_vt_fps_10_table, 0, sizeof(camsensor_vt_fps_10_table));
	memset(&camsensor_vt_fps_15_table, 0, sizeof(camsensor_vt_fps_15_table));

#endif
	return 0;
}

#if 0	// temporary delete
static const char *s5ka3dfx_querymenu_wb_preset[] = {
	"WB Tungsten", "WB Fluorescent", "WB sunny", "WB cloudy", NULL
};

static const char *s5ka3dfx_querymenu_effect_mode[] = {
	"Effect Sepia", "Effect Aqua", "Effect Monochrome",
	"Effect Negative", "Effect Sketch", NULL
};

static const char *s5ka3dfx_querymenu_ev_bias_mode[] = {
	"-3EV",	"-2,1/2EV", "-2EV", "-1,1/2EV",
	"-1EV", "-1/2EV", "0", "1/2EV",
	"1EV", "1,1/2EV", "2EV", "2,1/2EV",
	"3EV", NULL
};
#endif

static struct v4l2_queryctrl s5ka3dfx_controls[] = {
#if 0	// temporary delete
	{
		/*
		 * For now, we just support in preset type
		 * to be close to generic WB system,
		 * we define color temp range for each preset
		 */
		.id = V4L2_CID_WHITE_BALANCE_TEMPERATURE,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "White balance in kelvin",
		.minimum = 0,
		.maximum = 10000,
		.step = 1,
		.default_value = 0,	/* FIXME */
	},
	{
		.id = V4L2_CID_WHITE_BALANCE_PRESET,
		.type = V4L2_CTRL_TYPE_MENU,
		.name = "White balance preset",
		.minimum = 0,
		.maximum = ARRAY_SIZE(s5ka3dfx_querymenu_wb_preset) - 2,
		.step = 1,
		.default_value = 0,
	},
	{
		.id = V4L2_CID_AUTO_WHITE_BALANCE,
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.name = "Auto white balance",
		.minimum = 0,
		.maximum = 1,
		.step = 1,
		.default_value = 0,
	},
	{
		.id = V4L2_CID_EXPOSURE,
		.type = V4L2_CTRL_TYPE_MENU,
		.name = "Exposure bias",
		.minimum = 0,
		.maximum = ARRAY_SIZE(s5ka3dfx_querymenu_ev_bias_mode) - 2,
		.step = 1,
		.default_value = (ARRAY_SIZE(s5ka3dfx_querymenu_ev_bias_mode) - 2) / 2,	/* 0 EV */
	},
	{
		.id = V4L2_CID_COLORFX,
		.type = V4L2_CTRL_TYPE_MENU,
		.name = "Image Effect",
		.minimum = 0,
		.maximum = ARRAY_SIZE(s5ka3dfx_querymenu_effect_mode) - 2,
		.step = 1,
		.default_value = 0,
	},
	{
		.id = V4L2_CID_CONTRAST,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "Contrast",
		.minimum = 0,
		.maximum = 4,
		.step = 1,
		.default_value = 2,
	},
	{
		.id = V4L2_CID_SATURATION,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "Saturation",
		.minimum = 0,
		.maximum = 4,
		.step = 1,
		.default_value = 2,
	},
	{
		.id = V4L2_CID_SHARPNESS,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "Sharpness",
		.minimum = 0,
		.maximum = 4,
		.step = 1,
		.default_value = 2,
	},
#endif	
};

const char **s5ka3dfx_ctrl_get_menu(u32 id)
{
	printk(KERN_DEBUG "s5ka3dfx_ctrl_get_menu is called... id : %d \n", id);

	switch (id) {
#if 0	// temporary delete
	case V4L2_CID_WHITE_BALANCE_PRESET:
		return s5ka3dfx_querymenu_wb_preset;

	case V4L2_CID_COLORFX:
		return s5ka3dfx_querymenu_effect_mode;

	case V4L2_CID_EXPOSURE:
		return s5ka3dfx_querymenu_ev_bias_mode;
#endif
	default:
		return v4l2_ctrl_get_menu(id);
	}
}

static inline struct v4l2_queryctrl const *s5ka3dfx_find_qctrl(int id)
{
	int i;

	printk(KERN_DEBUG "s5ka3dfx_find_qctrl is called...  id : %d \n", id);

	for (i = 0; i < ARRAY_SIZE(s5ka3dfx_controls); i++)
		if (s5ka3dfx_controls[i].id == id)
			return &s5ka3dfx_controls[i];

	return NULL;
}

static int s5ka3dfx_queryctrl(struct v4l2_subdev *sd, struct v4l2_queryctrl *qc)
{
	int i;

	printk(KERN_DEBUG "s5ka3dfx_queryctrl is called... \n");

	for (i = 0; i < ARRAY_SIZE(s5ka3dfx_controls); i++) {
		if (s5ka3dfx_controls[i].id == qc->id) {
			memcpy(qc, &s5ka3dfx_controls[i], \
				sizeof(struct v4l2_queryctrl));
			return 0;
		}
	}

	return -EINVAL;
}

static int s5ka3dfx_querymenu(struct v4l2_subdev *sd, struct v4l2_querymenu *qm)
{
	struct v4l2_queryctrl qctrl;

	printk(KERN_DEBUG "s5ka3dfx_querymenu is called... \n");

	qctrl.id = qm->id;
	s5ka3dfx_queryctrl(sd, &qctrl);

	return v4l2_ctrl_query_menu(qm, &qctrl, s5ka3dfx_ctrl_get_menu(qm->id));
}

/*
 * Clock configuration
 * Configure expected MCLK from host and return EINVAL if not supported clock
 * frequency is expected
 * 	freq : in Hz
 * 	flag : not supported for now
 */
static int s5ka3dfx_s_crystal_freq(struct v4l2_subdev *sd, u32 freq, u32 flags)
{
	int err = -EINVAL;

	printk(KERN_DEBUG "s5ka3dfx_s_crystal_freq is called... \n");

	return err;
}

static int s5ka3dfx_g_fmt(struct v4l2_subdev *sd, struct v4l2_format *fmt)
{
	int err = 0;

	printk(KERN_DEBUG "s5ka3dfx_g_fmt is called... \n");

	return err;
}

static int s5ka3dfx_s_fmt(struct v4l2_subdev *sd, struct v4l2_format *fmt)
{
	int err = 0;

	printk(KERN_DEBUG "s5ka3dfx_s_fmt is called... \n");

	return err;
}
static int s5ka3dfx_enum_framesizes(struct v4l2_subdev *sd, \
					struct v4l2_frmsizeenum *fsize)
{
	struct  s5ka3dfx_state *state = to_state(sd);
	int num_entries = sizeof(s5ka3dfx_framesize_list)/sizeof(struct s5ka3dfx_enum_framesize);	
	struct s5ka3dfx_enum_framesize *elem;	
	int index = 0;
	int i = 0;

	printk(KERN_DEBUG "s5ka3dfx_enum_framesizes is called... \n");

	/* The camera interface should read this value, this is the resolution
 	 * at which the sensor would provide framedata to the camera i/f
 	 *
 	 * In case of image capture, this returns the default camera resolution (WVGA)
 	 */
	fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;

	index = state->framesize_index;

	for(i = 0; i < num_entries; i++){
		elem = &s5ka3dfx_framesize_list[i];
		if(elem->index == index){
			fsize->discrete.width = s5ka3dfx_framesize_list[index].width;
			fsize->discrete.height = s5ka3dfx_framesize_list[index].height;
			return 0;
		}
	}

	return -EINVAL;
}


static int s5ka3dfx_enum_frameintervals(struct v4l2_subdev *sd, 
					struct v4l2_frmivalenum *fival)
{
	int err = 0;

	printk(KERN_DEBUG "s5ka3dfx_enum_frameintervals is called... \n");
	
	return err;
}

static int s5ka3dfx_enum_fmt(struct v4l2_subdev *sd, struct v4l2_fmtdesc *fmtdesc)
{
	int err = 0;

	printk(KERN_DEBUG "s5ka3dfx_enum_fmt is called... \n");

	return err;
}

static int s5ka3dfx_try_fmt(struct v4l2_subdev *sd, struct v4l2_format *fmt)
{
	int err = 0;

	printk(KERN_DEBUG "s5ka3dfx_enum_fmt is called... \n");

	return err;
}

static int s5ka3dfx_g_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *param)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int err = 0;

	dev_dbg(&client->dev, "%s\n", __func__);

	return err;
}

static int s5ka3dfx_s_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *param)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int err = 0;

	dev_dbg(&client->dev, "%s: numerator %d, denominator: %d\n", \
		__func__, param->parm.capture.timeperframe.numerator, \
		param->parm.capture.timeperframe.denominator);

	return err;
}
#if 0	/* unused functions */
static int s5ka3dfx_get_framesize_index(struct v4l2_subdev *sd)
{
	int i = 0;
	struct s5ka3dfx_state *state = to_state(sd);
	struct s5ka3dfx_enum_framesize *frmsize;
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	/* Check for video/image mode */
	for(i = 0; i < (sizeof(s5ka3dfx_framesize_list)/sizeof(struct s5ka3dfx_enum_framesize)); i++)
	{
		frmsize = &s5ka3dfx_framesize_list[i];
		if(frmsize->width >= state->pix.width && frmsize->height >= state->pix.height){
			return frmsize->index;
		} 
	}
	
	v4l_info(client, "%s: s5ka3dfx_framesize_list[%d].index = %d\n", __func__, i - 1, s5ka3dfx_framesize_list[i].index);
	
	/* FIXME: If it fails, return the last index. */
	return s5ka3dfx_framesize_list[i-1].index;
}
#endif	/* unused functions */
#if 0	/* unused function */
/* This function is called from the s_ctrl api
 * Given the index, it checks if it is a valid index.
 * On success, it returns 0.
 * On Failure, it returns -EINVAL
 */
static int s5ka3dfx_set_framesize_index(struct v4l2_subdev *sd, unsigned int index)
{
	int i = 0;
	struct s5ka3dfx_state *state = to_state(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	v4l_info(client, "%s: index = %d\n", __func__, index);

	/* Check for video/image mode */
	for(i = 0; i < (sizeof(s5ka3dfx_framesize_list)/sizeof(struct s5ka3dfx_enum_framesize)); i++)
	{
		if(s5ka3dfx_framesize_list[i].index == index){
			state->framesize_index = index; 
			state->pix.width = s5ka3dfx_framesize_list[i].width;
			state->pix.height = s5ka3dfx_framesize_list[i].height;
			return 0;
		} 
	} 
	
	return -EINVAL;
}
#endif	/* unused functions */
/* set sensor register values for adjusting brightness */ //CTC_W899_seungjoo.yi
static int s5ka3dfx_set_brightness(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct s5ka3dfx_state *state = to_state(sd);

	int err = -EINVAL;
	int ev_value = 0;

	dev_dbg(&client->dev, "%s: value : %d state->vt_mode %d \n", __func__, ctrl->value, state->vt_mode);

	ev_value = ctrl->value;

	printk(KERN_DEBUG "state->vt_mode : %d \n", state->vt_mode);
	if(state->vt_mode == 1)
	{
		switch(ev_value)
		{	
			case EV_MINUS_4:
				err = s5ka3dfx_i2c_write(sd, camsensor_ev_vt_m4_table[0], sizeof(camsensor_ev_vt_m4_table[0]));
				err = s5ka3dfx_i2c_write(sd, camsensor_ev_vt_m4_table[1], sizeof(camsensor_ev_vt_m4_table[1]));
				err = s5ka3dfx_i2c_write(sd, camsensor_ev_vt_m4_table[2], sizeof(camsensor_ev_vt_m4_table[2]));
			break;

			case EV_MINUS_3:
				err = s5ka3dfx_i2c_write(sd, camsensor_ev_vt_m3_table[0], sizeof(camsensor_ev_vt_m3_table[0]));
				err = s5ka3dfx_i2c_write(sd, camsensor_ev_vt_m3_table[1], sizeof(camsensor_ev_vt_m3_table[1]));
				err = s5ka3dfx_i2c_write(sd, camsensor_ev_vt_m3_table[2], sizeof(camsensor_ev_vt_m3_table[2]));
			break;

			
			case EV_MINUS_2:
				err = s5ka3dfx_i2c_write(sd, camsensor_ev_vt_m2_table[0], sizeof(camsensor_ev_vt_m2_table[0]));
				err = s5ka3dfx_i2c_write(sd, camsensor_ev_vt_m2_table[1], sizeof(camsensor_ev_vt_m2_table[1]));
				err = s5ka3dfx_i2c_write(sd, camsensor_ev_vt_m2_table[2], sizeof(camsensor_ev_vt_m2_table[2]));
			break;
			
			case EV_MINUS_1:
				err = s5ka3dfx_i2c_write(sd, camsensor_ev_vt_m1_table[0], sizeof(camsensor_ev_vt_m1_table[0]));
				err = s5ka3dfx_i2c_write(sd, camsensor_ev_vt_m1_table[1], sizeof(camsensor_ev_vt_m1_table[1]));
				err = s5ka3dfx_i2c_write(sd, camsensor_ev_vt_m1_table[2], sizeof(camsensor_ev_vt_m1_table[2]));
			break;

			case EV_DEFAULT:
				err = s5ka3dfx_i2c_write(sd, camsensor_ev_vt_default_table[0], sizeof(camsensor_ev_vt_default_table[0]));
				err = s5ka3dfx_i2c_write(sd, camsensor_ev_vt_default_table[1], sizeof(camsensor_ev_vt_default_table[1]));
				err = s5ka3dfx_i2c_write(sd, camsensor_ev_vt_default_table[2], sizeof(camsensor_ev_vt_default_table[2]));
			break;

			case EV_PLUS_1:
				err = s5ka3dfx_i2c_write(sd, camsensor_ev_vt_p1_table[0], sizeof(camsensor_ev_vt_p1_table[0]));
				err = s5ka3dfx_i2c_write(sd, camsensor_ev_vt_p1_table[1], sizeof(camsensor_ev_vt_p1_table[1]));
				err = s5ka3dfx_i2c_write(sd, camsensor_ev_vt_p1_table[2], sizeof(camsensor_ev_vt_p1_table[2]));
			break;

			case EV_PLUS_2:
				err = s5ka3dfx_i2c_write(sd, camsensor_ev_vt_p2_table[0], sizeof(camsensor_ev_vt_p2_table[0]));
				err = s5ka3dfx_i2c_write(sd, camsensor_ev_vt_p2_table[1], sizeof(camsensor_ev_vt_p2_table[1]));
				err = s5ka3dfx_i2c_write(sd, camsensor_ev_vt_p2_table[2], sizeof(camsensor_ev_vt_p2_table[2]));
			break;

			case EV_PLUS_3:
				err = s5ka3dfx_i2c_write(sd, camsensor_ev_vt_p3_table[0], sizeof(camsensor_ev_vt_p3_table[0]));
				err = s5ka3dfx_i2c_write(sd, camsensor_ev_vt_p3_table[1], sizeof(camsensor_ev_vt_p3_table[1]));
				err = s5ka3dfx_i2c_write(sd, camsensor_ev_vt_p3_table[2], sizeof(camsensor_ev_vt_p3_table[2]));
			break;

			case EV_PLUS_4:
				err = s5ka3dfx_i2c_write(sd, camsensor_ev_vt_p4_table[0], sizeof(camsensor_ev_vt_p4_table[0]));
				err = s5ka3dfx_i2c_write(sd, camsensor_ev_vt_p4_table[1], sizeof(camsensor_ev_vt_p4_table[1]));
				err = s5ka3dfx_i2c_write(sd, camsensor_ev_vt_p4_table[2], sizeof(camsensor_ev_vt_p4_table[2]));
			break;	
			
			default:
				err = s5ka3dfx_i2c_write(sd, camsensor_ev_vt_default_table[0], sizeof(camsensor_ev_vt_default_table[0]));
				err = s5ka3dfx_i2c_write(sd, camsensor_ev_vt_default_table[1], sizeof(camsensor_ev_vt_default_table[1]));
				err = s5ka3dfx_i2c_write(sd, camsensor_ev_vt_default_table[2], sizeof(camsensor_ev_vt_default_table[2]));				
			break;
		}
	}
	else
	{
		switch(ev_value)
		{	
			case EV_MINUS_4:
				err = s5ka3dfx_i2c_write(sd, camsensor_ev_m4_table[0], sizeof(camsensor_ev_m4_table[0]));
				err = s5ka3dfx_i2c_write(sd, camsensor_ev_m4_table[1], sizeof(camsensor_ev_m4_table[1]));
				err = s5ka3dfx_i2c_write(sd, camsensor_ev_m4_table[2], sizeof(camsensor_ev_m4_table[2]));
			break;

			case EV_MINUS_3:
				err = s5ka3dfx_i2c_write(sd, camsensor_ev_m3_table[0], sizeof(camsensor_ev_m3_table[0]));
				err = s5ka3dfx_i2c_write(sd, camsensor_ev_m3_table[1], sizeof(camsensor_ev_m3_table[1]));
				err = s5ka3dfx_i2c_write(sd, camsensor_ev_m3_table[2], sizeof(camsensor_ev_m3_table[2]));
			break;

			
			case EV_MINUS_2:
				err = s5ka3dfx_i2c_write(sd, camsensor_ev_m2_table[0], sizeof(camsensor_ev_m2_table[0]));
				err = s5ka3dfx_i2c_write(sd, camsensor_ev_m2_table[1], sizeof(camsensor_ev_m2_table[1]));
				err = s5ka3dfx_i2c_write(sd, camsensor_ev_m2_table[2], sizeof(camsensor_ev_m2_table[2]));
			break;
			
			case EV_MINUS_1:
				err = s5ka3dfx_i2c_write(sd, camsensor_ev_m1_table[0], sizeof(camsensor_ev_m1_table[0]));
				err = s5ka3dfx_i2c_write(sd, camsensor_ev_m1_table[1], sizeof(camsensor_ev_m1_table[1]));
				err = s5ka3dfx_i2c_write(sd, camsensor_ev_m1_table[2], sizeof(camsensor_ev_m1_table[2]));
			break;

			case EV_DEFAULT:
				err = s5ka3dfx_i2c_write(sd, camsensor_ev_default_table[0], sizeof(camsensor_ev_default_table[0]));
				err = s5ka3dfx_i2c_write(sd, camsensor_ev_default_table[1], sizeof(camsensor_ev_default_table[1]));
				err = s5ka3dfx_i2c_write(sd, camsensor_ev_default_table[2], sizeof(camsensor_ev_default_table[2]));
			break;

			case EV_PLUS_1:
				err = s5ka3dfx_i2c_write(sd, camsensor_ev_p1_table[0], sizeof(camsensor_ev_p1_table[0]));
				err = s5ka3dfx_i2c_write(sd, camsensor_ev_p1_table[1], sizeof(camsensor_ev_p1_table[1]));
				err = s5ka3dfx_i2c_write(sd, camsensor_ev_p1_table[2], sizeof(camsensor_ev_p1_table[2]));
			break;

			case EV_PLUS_2:
				err = s5ka3dfx_i2c_write(sd, camsensor_ev_p2_table[0], sizeof(camsensor_ev_p2_table[0]));
				err = s5ka3dfx_i2c_write(sd, camsensor_ev_p2_table[1], sizeof(camsensor_ev_p2_table[1]));
				err = s5ka3dfx_i2c_write(sd, camsensor_ev_p2_table[2], sizeof(camsensor_ev_p2_table[2]));
			break;

			case EV_PLUS_3:
				err = s5ka3dfx_i2c_write(sd, camsensor_ev_p3_table[0], sizeof(camsensor_ev_p3_table[0]));
				err = s5ka3dfx_i2c_write(sd, camsensor_ev_p3_table[1], sizeof(camsensor_ev_p3_table[1]));
				err = s5ka3dfx_i2c_write(sd, camsensor_ev_p3_table[2], sizeof(camsensor_ev_p3_table[2]));
			break;

			case EV_PLUS_4:
				err = s5ka3dfx_i2c_write(sd, camsensor_ev_p4_table[0], sizeof(camsensor_ev_p4_table[0]));
				err = s5ka3dfx_i2c_write(sd, camsensor_ev_p4_table[1], sizeof(camsensor_ev_p4_table[1]));
				err = s5ka3dfx_i2c_write(sd, camsensor_ev_p4_table[2], sizeof(camsensor_ev_p4_table[2]));
			break;	
			
			default:
				err = s5ka3dfx_i2c_write(sd, camsensor_ev_default_table[0], sizeof(camsensor_ev_default_table[0]));
				err = s5ka3dfx_i2c_write(sd, camsensor_ev_default_table[1], sizeof(camsensor_ev_default_table[1]));
				err = s5ka3dfx_i2c_write(sd, camsensor_ev_default_table[2], sizeof(camsensor_ev_default_table[2]));				
			break;
		}
	}
	if (err < 0)
	{
		v4l_info(client, "%s: register set failed\n", __func__);
		return -EIO;
	}
	return err;
}

/* set sensor register values for adjusting whitebalance, both auto and manual */
static int s5ka3dfx_set_wb(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int err = -EINVAL;

	dev_dbg(&client->dev, "%s:  value : %d \n", __func__, ctrl->value);

	switch(ctrl->value)
	{
	case WHITE_BALANCE_AUTO:
		err = s5ka3dfx_i2c_write(sd, camsensor_wb_auto_table[0], sizeof(camsensor_wb_auto_table[0]));
		err = s5ka3dfx_i2c_write(sd, camsensor_wb_auto_table[1], sizeof(camsensor_wb_auto_table[1]));
		err = s5ka3dfx_i2c_write(sd, camsensor_wb_auto_table[2], sizeof(camsensor_wb_auto_table[2]));	
		err = s5ka3dfx_i2c_write(sd, camsensor_wb_auto_table[3], sizeof(camsensor_wb_auto_table[3]));
		err = s5ka3dfx_i2c_write(sd, camsensor_wb_auto_table[4], sizeof(camsensor_wb_auto_table[4]));
		err = s5ka3dfx_i2c_write(sd, camsensor_wb_auto_table[5], sizeof(camsensor_wb_auto_table[5]));			
		break;

	case WHITE_BALANCE_SUNNY:
		err = s5ka3dfx_i2c_write(sd, camsensor_wb_sunny_table[0], sizeof(camsensor_wb_sunny_table[0]));
		err = s5ka3dfx_i2c_write(sd, camsensor_wb_sunny_table[1], sizeof(camsensor_wb_sunny_table[1]));
		err = s5ka3dfx_i2c_write(sd, camsensor_wb_sunny_table[2], sizeof(camsensor_wb_sunny_table[2]));	
		err = s5ka3dfx_i2c_write(sd, camsensor_wb_sunny_table[3], sizeof(camsensor_wb_sunny_table[3]));
		err = s5ka3dfx_i2c_write(sd, camsensor_wb_sunny_table[4], sizeof(camsensor_wb_sunny_table[4]));
		err = s5ka3dfx_i2c_write(sd, camsensor_wb_sunny_table[5], sizeof(camsensor_wb_sunny_table[5]));		
		break;

	case WHITE_BALANCE_CLOUDY:
		err = s5ka3dfx_i2c_write(sd, camsensor_wb_cloudy_table[0], sizeof(camsensor_wb_cloudy_table[0]));
		err = s5ka3dfx_i2c_write(sd, camsensor_wb_cloudy_table[1], sizeof(camsensor_wb_cloudy_table[1]));
		err = s5ka3dfx_i2c_write(sd, camsensor_wb_cloudy_table[2], sizeof(camsensor_wb_cloudy_table[2]));	
		err = s5ka3dfx_i2c_write(sd, camsensor_wb_cloudy_table[3], sizeof(camsensor_wb_cloudy_table[3]));
		err = s5ka3dfx_i2c_write(sd, camsensor_wb_cloudy_table[4], sizeof(camsensor_wb_cloudy_table[4]));
		err = s5ka3dfx_i2c_write(sd, camsensor_wb_cloudy_table[5], sizeof(camsensor_wb_cloudy_table[5]));		
		break;

	case WHITE_BALANCE_TUNGSTEN:
		err = s5ka3dfx_i2c_write(sd, camsensor_wb_tungsten_table[0], sizeof(camsensor_wb_tungsten_table[0]));
		err = s5ka3dfx_i2c_write(sd, camsensor_wb_tungsten_table[1], sizeof(camsensor_wb_tungsten_table[1]));
		err = s5ka3dfx_i2c_write(sd, camsensor_wb_tungsten_table[2], sizeof(camsensor_wb_tungsten_table[2]));	
		err = s5ka3dfx_i2c_write(sd, camsensor_wb_tungsten_table[3], sizeof(camsensor_wb_tungsten_table[3]));
		err = s5ka3dfx_i2c_write(sd, camsensor_wb_tungsten_table[4], sizeof(camsensor_wb_tungsten_table[4]));
		err = s5ka3dfx_i2c_write(sd, camsensor_wb_tungsten_table[5], sizeof(camsensor_wb_tungsten_table[5]));		
		break;

	case WHITE_BALANCE_FLUORESCENT:
		err = s5ka3dfx_i2c_write(sd, camsensor_wb_fluorescent_table[0], sizeof(camsensor_wb_fluorescent_table[0]));
		err = s5ka3dfx_i2c_write(sd, camsensor_wb_fluorescent_table[1], sizeof(camsensor_wb_fluorescent_table[1]));
		err = s5ka3dfx_i2c_write(sd, camsensor_wb_fluorescent_table[2], sizeof(camsensor_wb_fluorescent_table[2]));	
		err = s5ka3dfx_i2c_write(sd, camsensor_wb_fluorescent_table[3], sizeof(camsensor_wb_fluorescent_table[3]));
		err = s5ka3dfx_i2c_write(sd, camsensor_wb_fluorescent_table[4], sizeof(camsensor_wb_fluorescent_table[4]));
		err = s5ka3dfx_i2c_write(sd, camsensor_wb_fluorescent_table[5], sizeof(camsensor_wb_fluorescent_table[5]));		
		break;

	default:
		dev_dbg(&client->dev, "%s: Not Support value \n", __func__);
		err = 0;
		break;

	}
	return err;
}

/* set sensor register values for adjusting color effect */
static int s5ka3dfx_set_effect(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int err = -EINVAL;

	dev_dbg(&client->dev, "%s: value : %d \n", __func__, ctrl->value);

	switch(ctrl->value)
	{
	case IMAGE_EFFECT_NONE:
		err = s5ka3dfx_i2c_write(sd, camsensor_effect_none_table[0], sizeof(camsensor_effect_none_table[0]));
		err = s5ka3dfx_i2c_write(sd, camsensor_effect_none_table[1], sizeof(camsensor_effect_none_table[1]));
		err = s5ka3dfx_i2c_write(sd, camsensor_effect_none_table[2], sizeof(camsensor_effect_none_table[2]));	
		err = s5ka3dfx_i2c_write(sd, camsensor_effect_none_table[3], sizeof(camsensor_effect_none_table[3]));
		err = s5ka3dfx_i2c_write(sd, camsensor_effect_none_table[4], sizeof(camsensor_effect_none_table[4]));		
		break;

	case IMAGE_EFFECT_BNW:		//Gray
		err = s5ka3dfx_i2c_write(sd, camsensor_effect_gray_table[0], sizeof(camsensor_effect_gray_table[0]));
		err = s5ka3dfx_i2c_write(sd, camsensor_effect_gray_table[1], sizeof(camsensor_effect_gray_table[1]));
		err = s5ka3dfx_i2c_write(sd, camsensor_effect_gray_table[2], sizeof(camsensor_effect_gray_table[2]));	
		err = s5ka3dfx_i2c_write(sd, camsensor_effect_gray_table[3], sizeof(camsensor_effect_gray_table[3]));
		err = s5ka3dfx_i2c_write(sd, camsensor_effect_gray_table[4], sizeof(camsensor_effect_gray_table[4]));
		break;

	case IMAGE_EFFECT_SEPIA:
		err = s5ka3dfx_i2c_write(sd, camsensor_effect_sepia_table[0], sizeof(camsensor_effect_sepia_table[0]));
		err = s5ka3dfx_i2c_write(sd, camsensor_effect_sepia_table[1], sizeof(camsensor_effect_sepia_table[1]));
		err = s5ka3dfx_i2c_write(sd, camsensor_effect_sepia_table[2], sizeof(camsensor_effect_sepia_table[2]));	
		err = s5ka3dfx_i2c_write(sd, camsensor_effect_sepia_table[3], sizeof(camsensor_effect_sepia_table[3]));
		err = s5ka3dfx_i2c_write(sd, camsensor_effect_sepia_table[4], sizeof(camsensor_effect_sepia_table[4]));
		break;

	case IMAGE_EFFECT_AQUA:
		err = s5ka3dfx_i2c_write(sd, camsensor_effect_aqua_table[0], sizeof(camsensor_effect_aqua_table[0]));
		err = s5ka3dfx_i2c_write(sd, camsensor_effect_aqua_table[1], sizeof(camsensor_effect_aqua_table[1]));
		err = s5ka3dfx_i2c_write(sd, camsensor_effect_aqua_table[2], sizeof(camsensor_effect_aqua_table[2]));	
		err = s5ka3dfx_i2c_write(sd, camsensor_effect_aqua_table[3], sizeof(camsensor_effect_aqua_table[3]));
		err = s5ka3dfx_i2c_write(sd, camsensor_effect_aqua_table[4], sizeof(camsensor_effect_aqua_table[4]));
		break;

	case IMAGE_EFFECT_NEGATIVE:
		err = s5ka3dfx_i2c_write(sd, camsensor_effect_negative_table[0], sizeof(camsensor_effect_negative_table[0]));
		err = s5ka3dfx_i2c_write(sd, camsensor_effect_negative_table[1], sizeof(camsensor_effect_negative_table[1]));
		err = s5ka3dfx_i2c_write(sd, camsensor_effect_negative_table[2], sizeof(camsensor_effect_negative_table[2]));	
		err = s5ka3dfx_i2c_write(sd, camsensor_effect_negative_table[3], sizeof(camsensor_effect_negative_table[3]));
		err = s5ka3dfx_i2c_write(sd, camsensor_effect_negative_table[4], sizeof(camsensor_effect_negative_table[4]));
		break;

	default:
		dev_dbg(&client->dev, "%s: Not Support value \n", __func__);
		err = 0;
		break;

	}
	
	return err;
}

/* set sensor register values for frame rate(fps) setting */
static int s5ka3dfx_set_frame_rate(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct s5ka3dfx_state *state = to_state(sd);

	int err = -EINVAL;
	int i = 0;

	dev_dbg(&client->dev, "%s: value : %d \n", __func__, ctrl->value);
	
	printk(KERN_DEBUG "state->vt_mode : %d \n", state->vt_mode);
	if(state->vt_mode == 1)
	{
		switch(ctrl->value)
		{
		case 7:
			for (i = 0; i < camsensor_vt_fps_7_index; i++) {
				err = s5ka3dfx_i2c_write(sd, camsensor_vt_fps_7_table[i], sizeof(camsensor_vt_fps_7_table[i]));
			}
			break;

		case 10:
			for (i = 0; i < camsensor_vt_fps_10_index; i++) {
				err = s5ka3dfx_i2c_write(sd, camsensor_vt_fps_10_table[i], sizeof(camsensor_vt_fps_10_table[i]));
			}
			break;
			
		case 15:
			for (i = 0; i < camsensor_vt_fps_15_index; i++) {
				err = s5ka3dfx_i2c_write(sd, camsensor_vt_fps_15_table[i], sizeof(camsensor_vt_fps_15_table[i]));
			}
			break;

		default:
			dev_dbg(&client->dev, "%s: Not Support value \n", __func__);
			err = 0;
			break;
		}
	}
	else
	{
		switch(ctrl->value)
		{
		case 7:
			for (i = 0; i < camsensor_fps_7_index; i++) {
				err = s5ka3dfx_i2c_write(sd, camsensor_fps_7_table[i], sizeof(camsensor_fps_7_table[i]));
			}
			break;

		case 10:
			for (i = 0; i < camsensor_fps_10_index; i++) {
				err = s5ka3dfx_i2c_write(sd, camsensor_fps_10_table[i], sizeof(camsensor_fps_10_table[i]));
			}
			break;
			
		case 15:
			for (i = 0; i < camsensor_fps_15_index; i++) {
				err = s5ka3dfx_i2c_write(sd, camsensor_fps_15_table[i], sizeof(camsensor_fps_15_table[i]));
			}
			break;

		default:
			dev_dbg(&client->dev, "%s: Not Support value \n", __func__);
			err = 0;
			break;
		}
	}
	return err;
}

/* set sensor register values for adjusting blur effect */
static int s5ka3dfx_set_blur(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct s5ka3dfx_state *state = to_state(sd);
	int err = -EINVAL;

	dev_dbg(&client->dev, "%s: value : %d \n", __func__, ctrl->value);
	
	printk(KERN_DEBUG "state->vt_mode : %d \n", state->vt_mode);
	if(state->vt_mode == 1)
	{
		switch(ctrl->value)
		{
			case BLUR_LEVEL_0:
				err = s5ka3dfx_i2c_write(sd, camsensor_blur_vt_none_table[0], sizeof(camsensor_blur_vt_none_table[0]));
				err = s5ka3dfx_i2c_write(sd, camsensor_blur_vt_none_table[1], sizeof(camsensor_blur_vt_none_table[1]));
				err = s5ka3dfx_i2c_write(sd, camsensor_blur_vt_none_table[2], sizeof(camsensor_blur_vt_none_table[2]));	
				err = s5ka3dfx_i2c_write(sd, camsensor_blur_vt_none_table[3], sizeof(camsensor_blur_vt_none_table[3]));
				break;

			case BLUR_LEVEL_1:	
				err = s5ka3dfx_i2c_write(sd, camsensor_blur_vt_p1_table[0], sizeof(camsensor_blur_vt_p1_table[0]));
				err = s5ka3dfx_i2c_write(sd, camsensor_blur_vt_p1_table[1], sizeof(camsensor_blur_vt_p1_table[1]));
				err = s5ka3dfx_i2c_write(sd, camsensor_blur_vt_p1_table[2], sizeof(camsensor_blur_vt_p1_table[2]));	
				err = s5ka3dfx_i2c_write(sd, camsensor_blur_vt_p1_table[3], sizeof(camsensor_blur_vt_p1_table[3]));
				break;

			case BLUR_LEVEL_2:
				err = s5ka3dfx_i2c_write(sd, camsensor_blur_vt_p2_table[0], sizeof(camsensor_blur_vt_p2_table[0]));
				err = s5ka3dfx_i2c_write(sd, camsensor_blur_vt_p2_table[1], sizeof(camsensor_blur_vt_p2_table[1]));
				err = s5ka3dfx_i2c_write(sd, camsensor_blur_vt_p2_table[2], sizeof(camsensor_blur_vt_p2_table[2]));	
				err = s5ka3dfx_i2c_write(sd, camsensor_blur_vt_p2_table[3], sizeof(camsensor_blur_vt_p2_table[3]));
				break;

			case BLUR_LEVEL_3:
				err = s5ka3dfx_i2c_write(sd, camsensor_blur_vt_p3_table[0], sizeof(camsensor_blur_vt_p3_table[0]));
				err = s5ka3dfx_i2c_write(sd, camsensor_blur_vt_p3_table[1], sizeof(camsensor_blur_vt_p3_table[1]));
				err = s5ka3dfx_i2c_write(sd, camsensor_blur_vt_p3_table[2], sizeof(camsensor_blur_vt_p3_table[2]));	
				err = s5ka3dfx_i2c_write(sd, camsensor_blur_vt_p3_table[3], sizeof(camsensor_blur_vt_p3_table[3]));
				break;

			default:
				dev_dbg(&client->dev, "%s: Not Support value \n", __func__);
				err = 0;
				break;

		}
	}
	else
	{
		switch(ctrl->value)
		{
			case BLUR_LEVEL_0:
				err = s5ka3dfx_i2c_write(sd, camsensor_blur_none_table[0], sizeof(camsensor_blur_none_table[0]));
				err = s5ka3dfx_i2c_write(sd, camsensor_blur_none_table[1], sizeof(camsensor_blur_none_table[1]));
				err = s5ka3dfx_i2c_write(sd, camsensor_blur_none_table[2], sizeof(camsensor_blur_none_table[2]));	
				err = s5ka3dfx_i2c_write(sd, camsensor_blur_none_table[3], sizeof(camsensor_blur_none_table[3]));
				break;

			case BLUR_LEVEL_1:	
				err = s5ka3dfx_i2c_write(sd, camsensor_blur_p1_table[0], sizeof(camsensor_blur_p1_table[0]));
				err = s5ka3dfx_i2c_write(sd, camsensor_blur_p1_table[1], sizeof(camsensor_blur_p1_table[1]));
				err = s5ka3dfx_i2c_write(sd, camsensor_blur_p1_table[2], sizeof(camsensor_blur_p1_table[2]));	
				err = s5ka3dfx_i2c_write(sd, camsensor_blur_p1_table[3], sizeof(camsensor_blur_p1_table[3]));
				break;

			case BLUR_LEVEL_2:
				err = s5ka3dfx_i2c_write(sd, camsensor_blur_p2_table[0], sizeof(camsensor_blur_p2_table[0]));
				err = s5ka3dfx_i2c_write(sd, camsensor_blur_p2_table[1], sizeof(camsensor_blur_p2_table[1]));
				err = s5ka3dfx_i2c_write(sd, camsensor_blur_p2_table[2], sizeof(camsensor_blur_p2_table[2]));	
				err = s5ka3dfx_i2c_write(sd, camsensor_blur_p2_table[3], sizeof(camsensor_blur_p2_table[3]));
				break;

			case BLUR_LEVEL_3:
				err = s5ka3dfx_i2c_write(sd, camsensor_blur_p3_table[0], sizeof(camsensor_blur_p3_table[0]));
				err = s5ka3dfx_i2c_write(sd, camsensor_blur_p3_table[1], sizeof(camsensor_blur_p3_table[1]));
				err = s5ka3dfx_i2c_write(sd, camsensor_blur_p3_table[2], sizeof(camsensor_blur_p3_table[2]));	
				err = s5ka3dfx_i2c_write(sd, camsensor_blur_p3_table[3], sizeof(camsensor_blur_p3_table[3]));
				break;

			default:
				dev_dbg(&client->dev, "%s: Not Support value \n", __func__);
				err = 0;
				break;
		}		
	}
	return err;
}

static int s5ka3dfx_check_dataline_stop(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct s5ka3dfx_state *state = to_state(sd);
	int err = -EINVAL, i;

	dev_dbg(&client->dev, "%s\n", __func__);

	for (i = 0; i < 2; i++) {
		err = s5ka3dfx_i2c_write(sd, camsensor_dataline_stop_table[i], \
					sizeof(camsensor_dataline_stop_table[i]));
		if (err < 0)
		{
			v4l_info(client, "%s: register set failed\n", __func__);
			return -EIO;
		}
	}
	state->check_dataline = 0;
	err = s5ka3dfx_reset(sd);
	if (err < 0)
	{
		v4l_info(client, "%s: register set failed\n", __func__);
		return -EIO;
	}
	return err;
}



/* returns the real iso currently used by sensor due to lighting
 * conditions, not the requested iso we sent using s_ctrl.
 */
static int s5ka3dfx_get_iso(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	int err;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	s32 read_value;
	int gain;

	err = s5ka3dfx_i2c_write_multi(client, 0xEF, 0x02);
	if (err < 0)
		return err;

	read_value = i2c_smbus_read_byte_data(client, 0x1D);
	if (read_value < 0)
		return read_value;

	read_value &= 0x7F;
	gain = (128 * 100) / (128 - read_value);

	if (gain > 280)
		ctrl->value = ISO_400;
	else if (gain > 230)
		ctrl->value = ISO_200;
	else if (gain > 190)
		ctrl->value = ISO_100;
	else if (gain > 100)
		ctrl->value = ISO_50;
	else
		ctrl->value = gain;

	dev_dbg(&client->dev, "%s: get iso == %d (0x%x)\n",__func__, ctrl->value, read_value);

	return err;
}



/* returns the shutterspeed currently used by sensor due to lighting conditions*/
static int s5ka3dfx_get_shutterspeed(struct v4l2_subdev *sd,struct v4l2_control *ctrl)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct s5ka3dfx_state *state = to_state(sd);
	s32 read_value;
	int cintr;
	int err;

	err = s5ka3dfx_i2c_write_multi(client, 0xEF, 0x02);
	if (err < 0)
		return err;

	read_value = i2c_smbus_read_byte_data(client, 0x0E);
	if (read_value < 0)
		return read_value;
	cintr = (read_value & 0x1F) << 8;

	read_value = i2c_smbus_read_byte_data(client, 0x0F);
	if (read_value < 0)
		return read_value;
	cintr |= read_value & 0xFF;

	/* A3D Shutter Speed (Sec.) = MCLK / (2 * (cintr - 1) * 814) */
	ctrl->value =  ((cintr - 1) * 1628) / (state->freq / 1000);

	dev_dbg(&client->dev,"%s: get shutterspeed == %d\n", __func__, ctrl->value);

	return err;
}




/* if you need, add below some functions below */

static int s5ka3dfx_g_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct s5ka3dfx_state *state = to_state(sd);
	struct s5ka3dfx_userset userset = state->userset;
	int err = -EINVAL;

	dev_dbg(&client->dev, "%s: id : 0x%08x \n", __func__, ctrl->id);

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		ctrl->value = userset.exposure_bias;
		err = 0;
		break;

	case V4L2_CID_AUTO_WHITE_BALANCE:
		ctrl->value = userset.auto_wb;
		err = 0;
		break;

	case V4L2_CID_WHITE_BALANCE_PRESET:
		ctrl->value = userset.manual_wb;
		err = 0;
		break;

	case V4L2_CID_COLORFX:
		ctrl->value = userset.effect;
		err = 0;
		break;

	case V4L2_CID_CONTRAST:
		ctrl->value = userset.contrast;
		err = 0;
		break;

	case V4L2_CID_SATURATION:
		ctrl->value = userset.saturation;
		err = 0;
		break;

	case V4L2_CID_SHARPNESS:
		ctrl->value = userset.saturation;
		err = 0;
		break;

	case V4L2_CID_CAMERA_GET_ISO:
		s5ka3dfx_get_iso(sd, ctrl);
		err = 0;
		break;

	case V4L2_CID_CAMERA_GET_SHT_TIME:
		s5ka3dfx_get_shutterspeed(sd, ctrl);
		err = 0;
		break;

#if 0
	case V4L2_CID_CAM_FRAMESIZE_INDEX:
		ctrl->value = s5ka3dfx_get_framesize_index(sd);
		err = 0;
		break;
#endif

	default:
		dev_dbg(&client->dev, "%s: no such ctrl\n", __func__);
		break;
	}
	
	return err;
}

static int s5ka3dfx_s_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
#ifdef S5KA3DFX_COMPLETE
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct s5ka3dfx_state *state = to_state(sd);

	int err = -EINVAL;

	printk(KERN_DEBUG "s5ka3dfx_s_ctrl() : ctrl->id 0x%08x, ctrl->value %d \n",ctrl->id, ctrl->value);

	switch (ctrl->id) {

#if 0		
	case V4L2_CID_EXPOSURE:
		dev_dbg(&client->dev, "%s: V4L2_CID_EXPOSURE\n", __func__);
		err = s5ka3dfx_write_regs(sd, \
			(unsigned char *) s5ka3dfx_regs_ev_bias[ctrl->value], \
			sizeof(s5ka3dfx_regs_ev_bias[ctrl->value]));
		break;

	case V4L2_CID_AUTO_WHITE_BALANCE:
		dev_dbg(&client->dev, "%s: V4L2_CID_AUTO_WHITE_BALANCE\n", \
			__func__);
		err = s5ka3dfx_write_regs(sd, \
			(unsigned char *) s5ka3dfx_regs_awb_enable[ctrl->value], \
			sizeof(s5ka3dfx_regs_awb_enable[ctrl->value]));
		break;

	case V4L2_CID_WHITE_BALANCE_PRESET:
		dev_dbg(&client->dev, "%s: V4L2_CID_WHITE_BALANCE_PRESET\n", \
			__func__);
		err = s5ka3dfx_write_regs(sd, \
			(unsigned char *) s5ka3dfx_regs_wb_preset[ctrl->value], \
			sizeof(s5ka3dfx_regs_wb_preset[ctrl->value]));
		break;

	case V4L2_CID_COLORFX:
		dev_dbg(&client->dev, "%s: V4L2_CID_COLORFX\n", __func__);
		err = s5ka3dfx_write_regs(sd, \
			(unsigned char *) s5ka3dfx_regs_color_effect[ctrl->value], \
			sizeof(s5ka3dfx_regs_color_effect[ctrl->value]));
		break;

	case V4L2_CID_CONTRAST:
		dev_dbg(&client->dev, "%s: V4L2_CID_CONTRAST\n", __func__);
		err = s5ka3dfx_write_regs(sd, \
			(unsigned char *) s5ka3dfx_regs_contrast_bias[ctrl->value], \
			sizeof(s5ka3dfx_regs_contrast_bias[ctrl->value]));
		break;

	case V4L2_CID_SATURATION:
		dev_dbg(&client->dev, "%s: V4L2_CID_SATURATION\n", __func__);
		err = s5ka3dfx_write_regs(sd, \
			(unsigned char *) s5ka3dfx_regs_saturation_bias[ctrl->value], \
			sizeof(s5ka3dfx_regs_saturation_bias[ctrl->value]));
		break;

	case V4L2_CID_SHARPNESS:
		dev_dbg(&client->dev, "%s: V4L2_CID_SHARPNESS\n", __func__);
		err = s5ka3dfx_write_regs(sd, \
			(unsigned char *) s5ka3dfx_regs_sharpness_bias[ctrl->value], \
			sizeof(s5ka3dfx_regs_sharpness_bias[ctrl->value]));
		break;	

	/* The camif supports only a few frame resolutions. 
 	 * Through this call, camif can set the camera resolution with given index.
 	 * Typically, camif gets the index through g_ctrl call with this ID.
 	 */

 	case V4L2_CID_CAM_FRAMESIZE_INDEX:
		err = s5ka3dfx_set_framesize_index(sd, ctrl->value);
        break;
#endif

	case V4L2_CID_CAMERA_BRIGHTNESS:	//V4L2_CID_EXPOSURE:
		dev_dbg(&client->dev, "%s: V4L2_CID_CAMERA_BRIGHTNESS\n", __func__);
		err = s5ka3dfx_set_brightness(sd, ctrl);
		break;

	case V4L2_CID_CAMERA_WHITE_BALANCE: //V4L2_CID_AUTO_WHITE_BALANCE:
		dev_dbg(&client->dev, "%s: V4L2_CID_AUTO_WHITE_BALANCE\n", __func__);
		err = s5ka3dfx_set_wb(sd, ctrl);
		break;

	case V4L2_CID_CAMERA_EFFECT:	//V4L2_CID_COLORFX:
		dev_dbg(&client->dev, "%s: V4L2_CID_CAMERA_EFFECT\n", __func__);
		err = s5ka3dfx_set_effect(sd, ctrl);
		break;

	case V4L2_CID_CAMERA_FRAME_RATE:
		dev_dbg(&client->dev, "%s: V4L2_CID_CAMERA_FRAME_RATE\n", __func__);
		err = s5ka3dfx_set_frame_rate(sd, ctrl);	
		break;
		
	case V4L2_CID_CAMERA_VGA_BLUR:
		dev_dbg(&client->dev, "%s: V4L2_CID_CAMERA_FRAME_RATE\n", __func__);
		err = s5ka3dfx_set_blur(sd, ctrl);	
		break;

	case V4L2_CID_CAMERA_VT_MODE:
		state->vt_mode = ctrl->value;
		dev_dbg(&client->dev, "%s: V4L2_CID_CAMERA_VT_MODE : state->vt_mode %d \n", __func__, state->vt_mode);
		err = 0;
		break;

	case V4L2_CID_CAMERA_CHECK_DATALINE:
		state->check_dataline = ctrl->value;
		err = 0;
		break;	

	case V4L2_CID_CAMERA_CHECK_DATALINE_STOP:
		err = s5ka3dfx_check_dataline_stop(sd);
		break;

	case V4L2_CID_CAM_PREVIEW_ONOFF:
		if(state->check_previewdata == 0)
		{
			err = 0;
		}
		else
		{
			err = -EIO;	
		}
		break;

	//s1_camera [ Defense process by ESD input ] _[
	case V4L2_CID_CAMERA_RESET:
		dev_dbg(&client->dev, "%s: V4L2_CID_CAMERA_RESET \n", __func__);
		err = s5ka3dfx_reset(sd);
		break;
	// _]

	default:
		dev_dbg(&client->dev, "%s: no support control in camera sensor, S5KA3DFX\n", __func__);
		//err = -ENOIOCTLCMD;
		err = 0;
		break;
	}

	if (err < 0)
		goto out;
	else
		return 0;

out:
	dev_dbg(&client->dev, "%s: vidioc_s_ctrl failed\n", __func__);
	return err;
#else
	return 0;
#endif
}

static int s5ka3dfx_init(struct v4l2_subdev *sd, u32 val)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct s5ka3dfx_state *state = to_state(sd);
	int err = -EINVAL, i;

	//CTC_W899_seungjoo.yi
	// init table
	init_reg_table(sd);
#if (IS_USE_VGA_TUNNING)
	make_register_table();
#endif

	//v4l_info(client, "%s: camera initialization start : state->vt_mode %d \n", __func__, state->vt_mode);
	printk(KERN_DEBUG "camera initialization start, state->vt_mode : %d \n", state->vt_mode); 
	printk(KERN_DEBUG "state->check_dataline : %d \n", state->check_dataline); 
	if(state->vt_mode == 0)
	{
		if(state->check_dataline)
		{	
			for (i = 0; i < camsensor_dataline_index; i++) {
				err = s5ka3dfx_i2c_write(sd, camsensor_dataline_table[i], \
							sizeof(camsensor_dataline_table[i]));
				if (err < 0)
				{
					v4l_info(client, "%s: register set failed\n", \
					__func__);
					
					break;
				}	
			}
		}
		else
		{
			for (i = 0; i < camsensor_init_index; i++) {
				err = s5ka3dfx_i2c_write(sd, camsensor_init_table[i], \
							sizeof(camsensor_init_table[i]));
				if (err < 0)
				{
					v4l_info(client, "%s: register set failed\n", \
					__func__);
					
					break;
				}	
			}
		}
	}
	else
	{
		for (i = 0; i < camsensor_init_vt_index; i++) {
			err = s5ka3dfx_i2c_write(sd, camsensor_init_vt_table[i], \
						sizeof(camsensor_init_vt_table[i]));
			if (err < 0)
			{
				v4l_info(client, "%s: register set failed\n", \
				__func__);
				
				break;
			}	
		}
	}

	if (err < 0) {
		//This is preview fail 
		state->check_previewdata = 100;
		v4l_err(client, "%s: camera initialization failed. err(%d)\n", \
			__func__, state->check_previewdata);
		return -EIO;	/* FIXME */	
	}

	//This is preview success
	state->check_previewdata = 0;
	return 0;
}

/*
 * s_config subdev ops
 * With camera device, we need to re-initialize every single opening time therefor,
 * it is not necessary to be initialized on probe time. except for version checking
 * NOTE: version checking is optional
 */
static int s5ka3dfx_s_config(struct v4l2_subdev *sd, int irq, void *platform_data)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct s5ka3dfx_state *state = to_state(sd);
	struct s5ka3dfx_platform_data *pdata;

	dev_dbg(&client->dev, "fetching platform data\n");

	pdata = client->dev.platform_data;

	if (!pdata) {
		dev_err(&client->dev, "%s: no platform data\n", __func__);
		return -ENODEV;
	}

	/*
	 * Assign default format and resolution
	 * Use configured default information in platform data
	 * or without them, use default information in driver
	 */
	if (!(pdata->default_width && pdata->default_height)) {
		/* TODO: assign driver default resolution */
	} else {
		state->pix.width = pdata->default_width;
		state->pix.height = pdata->default_height;
	}

	if (!pdata->pixelformat)
		state->pix.pixelformat = DEFAULT_FMT;
	else
		state->pix.pixelformat = pdata->pixelformat;

	if (!pdata->freq)
		state->freq = 24000000;	/* 24MHz default */
	else
		state->freq = pdata->freq;

	if (!pdata->is_mipi) {
		state->is_mipi = 0;
		dev_dbg(&client->dev, "parallel mode\n");
	} else
		state->is_mipi = pdata->is_mipi;

	return 0;
}

static const struct v4l2_subdev_core_ops s5ka3dfx_core_ops = {
	.init = s5ka3dfx_init,	/* initializing API */
	.s_config = s5ka3dfx_s_config,	/* Fetch platform data */
	.queryctrl = s5ka3dfx_queryctrl,
	.querymenu = s5ka3dfx_querymenu,
	.g_ctrl = s5ka3dfx_g_ctrl,
	.s_ctrl = s5ka3dfx_s_ctrl,
};

static const struct v4l2_subdev_video_ops s5ka3dfx_video_ops = {
	.s_crystal_freq = s5ka3dfx_s_crystal_freq,
	.g_fmt = s5ka3dfx_g_fmt,
	.s_fmt = s5ka3dfx_s_fmt,
	.enum_framesizes = s5ka3dfx_enum_framesizes,
	.enum_frameintervals = s5ka3dfx_enum_frameintervals,
	.enum_fmt = s5ka3dfx_enum_fmt,
	.try_fmt = s5ka3dfx_try_fmt,
	.g_parm = s5ka3dfx_g_parm,
	.s_parm = s5ka3dfx_s_parm,
};

static const struct v4l2_subdev_ops s5ka3dfx_ops = {
	.core = &s5ka3dfx_core_ops,
	.video = &s5ka3dfx_video_ops,
};

/*
 * s5ka3dfx_probe
 * Fetching platform data is being done with s_config subdev call.
 * In probe routine, we just register subdev device
 */
static int s5ka3dfx_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct s5ka3dfx_state *state;
	struct v4l2_subdev *sd;

	state = kzalloc(sizeof(struct s5ka3dfx_state), GFP_KERNEL);
	if (state == NULL)
		return -ENOMEM;

	sd = &state->sd;
	strcpy(sd->name, S5KA3DFX_DRIVER_NAME);

	/* Registering subdev */
	v4l2_i2c_subdev_init(sd, client, &s5ka3dfx_ops);

	dev_dbg(&client->dev, "s5ka3dfx has been probed\n");
	return 0;
}


static int s5ka3dfx_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);

	v4l2_device_unregister_subdev(sd);
	kfree(to_state(sd));
	return 0;
}

static const struct i2c_device_id s5ka3dfx_id[] = {
	{ S5KA3DFX_DRIVER_NAME, 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, s5ka3dfx_id);

static struct v4l2_i2c_driver_data v4l2_i2c_data = {
	.name = S5KA3DFX_DRIVER_NAME,
	.probe = s5ka3dfx_probe,
	.remove = s5ka3dfx_remove,
	.id_table = s5ka3dfx_id,
};

MODULE_DESCRIPTION("Samsung Electronics S5KA3DFX UXGA camera driver");
MODULE_AUTHOR("Jinsung Yang <jsgood.yang@samsung.com>");
MODULE_LICENSE("GPL");

