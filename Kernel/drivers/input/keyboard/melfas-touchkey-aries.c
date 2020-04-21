/*
 * Driver for keys on GPIO lines capable of generating interrupts.
 *
 * Copyright 2005 Phil Blundell
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>

#include <linux/init.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/sched.h>
#include <linux/pm.h>
#include <linux/sysctl.h>
#include <linux/proc_fs.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <mach/regs-gpio.h>
#include <plat/gpio-cfg.h>
#include <asm/gpio.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/earlysuspend.h>
#include <asm/io.h>


#if 1 //TEMP
#define S5PV210_GPH0DAT			(S5PV210_GPH0_BASE + 0x04)
#define S5PV210_GPH1DAT			(S5PV210_GPH1_BASE + 0x04)
#define S5PV210_GPH2DAT			(S5PV210_GPH2_BASE + 0x04)
#define S5PV210_GPH3DAT			(S5PV210_GPH3_BASE + 0x04)
#endif

/*
Melfas touchkey register
*/
#define KEYCODE_REG 0x00
#define FIRMWARE_VERSION 0x01
#define TOUCHKEY_MODULE_VERSION 0x02
#define TOUCHKEY_ADDRESS	0x20

#define UPDOWN_EVENT_BIT 0x08
#define KEYCODE_BIT 0x07
#define ESD_STATE_BIT 0x10

#define I2C_M_WR 0		/* for i2c */

#if defined (CONFIG_MACH_S5PC110_ARIESCT) || defined (CONFIG_MACH_S5PC110_PRESTIGE) || defined (CONFIG_MACH_S5PC110_GALAXY) || defined (CONFIG_MACH_S5PC110_LATONA) //TEMP for compile
#define IRQ_TOUCH_INT (IRQ_EINT_GROUP17_BASE + 2)
#else
#define IRQ_TOUCH_INT (IRQ_EINT_GROUP22_BASE + 1)
#endif

#define DEVICE_NAME "melfas-touchkey"

#define TOUCHKEY_KEYCODE_MENU 	KEY_BACK
#if defined (CONFIG_MACH_S5PC110_PRESTIGE)
#define TOUCHKEY_KEYCODE_HOME 	KEY_MENU
#endif
#define TOUCHKEY_KEYCODE_BACK 	KEY_ENTER
#if defined (CONFIG_MACH_S5PC110_PRESTIGE)
#define TOUCHKEY_KEYCODE_SEARCH 	KEY_END
#endif

#if defined (CONFIG_MACH_S5PC110_PRESTIGE)
static int touchkey_keycode[5] =
    { NULL, KEY_MENU, KEY_HOME, KEY_BACK, KEY_END };	//{ NULL, KEY_BACK, KEY_MENU, KEY_ENTER, KEY_END };	// NULL, MENU, HOME, BACK, SEARCH
#else
static int touchkey_keycode[3] =
    { NULL, KEY_MENU, KEY_BACK };
#endif
static struct input_dev *touchkey_dev;

struct i2c_touchkey_driver {
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct work_struct work;
	struct early_suspend early_suspend;
};
struct i2c_touchkey_driver *touchkey_driver = NULL;
struct workqueue_struct *touchkey_wq;

static const struct i2c_device_id melfas_touchkey_id[] = {
	{"melfas_touchkey", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, melfas_touchkey_id);

struct work_struct touch_update_work;
struct delayed_work touch_resume_work;

static void __iomem *gpio_pend_mask_mem;
#define 	INT_PEND_BASE	0xE0200A54

static DEFINE_MUTEX(melfas_touchkey_use);

static void init_hw(void);
static int i2c_touchkey_probe(struct i2c_client *client, const struct i2c_device_id *id);
extern int get_touchkey_firmware(char *version);
extern int get_touchkey_firmware6(char *version);

struct i2c_driver touchkey_i2c_driver = {
	.driver = {
		.name = "melfas_touchkey",
		   },
	.id_table = melfas_touchkey_id,
	.probe = i2c_touchkey_probe,
};

extern unsigned int HWREV;
extern int is_debug_screen;
extern unsigned int touch_state_val;

static u8 recommended_ver = 0;
static u8 menu_sensitivity = 0;
static u8 back_sensitivity = 0;
static u8 show_sensitivity = 0;
#if defined (CONFIG_MACH_S5PC110_PRESTIGE)
static u8 home_sensitivity = 0;
static u8 search_sensitivity = 0;
#endif

static int touchkey_enable = 0;
static int touch_version = 0;

int touchkey_debug_count = 0;
char touchkey_debug[104];

static int touchkey_press = 0;
static int old_keycode = 0;

static void set_touchkey_debug(char value)
{
	if (touchkey_debug_count == 100)
		touchkey_debug_count = 0;
	touchkey_debug[touchkey_debug_count] = value;
	touchkey_debug_count++;
}

static int i2c_touchkey_read(u8 reg, u8 * val, unsigned int len)
{
	int err;
	int retry = 10;
	struct i2c_msg msg[1];

	if ((touchkey_driver == NULL)) {
		return -ENODEV;
	}
	while (retry--) {
		msg->addr = touchkey_driver->client->addr;
		msg->flags = I2C_M_RD;
		msg->len = len;
		msg->buf = val;
		err = i2c_transfer(touchkey_driver->client->adapter, msg, 1);
		if (err >= 0) {
			return 0;
		}
		printk("%s %d i2c transfer error\n", __func__, __LINE__);	/* add by inter.park */
		mdelay(10);
	}
	return err;

}

static int i2c_touchkey_write(u8 * val, unsigned int len)
{
	int err;
	struct i2c_msg msg[1];
	unsigned char data[2];
	int retry = 1;

	if ((touchkey_driver == NULL) || !(touchkey_enable == 1)) {
		return -ENODEV;
	}

	while (retry--) {
		data[0] = *val;
		msg->addr = touchkey_driver->client->addr;
		msg->flags = I2C_M_WR;
		msg->len = len;
		msg->buf = data;
		err = i2c_transfer(touchkey_driver->client->adapter, msg, 1);
		if (err >= 0)
			return 0;
		printk("%s %d i2c transfer error\n", __func__, __LINE__);
		mdelay(10);
	}
	return err;
}


extern void TSP_forced_release(void);
void touchkey_work_func(struct work_struct *p)
{
	u8 data[3];
	u8 cmd=2;
	int ret;
	int retry = 10;
	int keycode;

#if defined(CONFIG_MACH_S5PC110_PRESTIGE)
	unsigned long isFlipOpen;
	isFlipOpen = (readl(S5PV210_GPH0DAT)) & (1 << 6);
#endif

	set_touchkey_debug('a');
	if(!gpio_get_value(_3_GPIO_TOUCH_INT))
	{
		ret = i2c_touchkey_read(KEYCODE_REG, data, 1);
		set_touchkey_debug(data[0]);
		if((data[0] & ESD_STATE_BIT) ||(ret !=0))
		{
			printk("ESD_STATE_BIT set or I2C fail: data: %d, retry: %d\n", data[0], retry);
			//releae key 
#if defined (CONFIG_MACH_S5PC110_PRESTIGE)
			input_report_key(touchkey_driver->input_dev, TOUCHKEY_KEYCODE_MENU, 0);
			input_report_key(touchkey_driver->input_dev, TOUCHKEY_KEYCODE_HOME, 0);
			input_report_key(touchkey_driver->input_dev, TOUCHKEY_KEYCODE_BACK, 0);
			input_report_key(touchkey_driver->input_dev, TOUCHKEY_KEYCODE_SEARCH, 0);
#else
			input_report_key(touchkey_driver->input_dev, touchkey_keycode[1], 0);
			input_report_key(touchkey_driver->input_dev, touchkey_keycode[2], 0);
#endif
			retry = 10;
			while(retry--)
			{
				gpio_direction_output(_3_GPIO_TOUCH_EN, 0);
				mdelay(300);
				init_hw();
				if(i2c_touchkey_read(KEYCODE_REG, data, 3) >= 0)
				{
					printk("%s touchkey init success\n", __func__);
					set_touchkey_debug('O');
					enable_irq(IRQ_TOUCH_INT);
					return;
				}
				printk("%s %d i2c transfer error retry = %d\n", __func__, __LINE__, retry);
			}
			//touchkey die , do not enable touchkey
			//enable_irq(IRQ_TOUCH_INT);
			//touchkey_enable = -1;
			gpio_direction_output(_3_GPIO_TOUCH_EN, 0);
			gpio_direction_output(_3_GPIO_TOUCH_CE, 0);
			gpio_direction_output(_3_TOUCH_SDA_28V, 0);
			gpio_direction_output(_3_TOUCH_SCL_28V, 0);
			printk("%s touchkey died\n", __func__);
			set_touchkey_debug('D');
			return;
		}

		keycode = touchkey_keycode[data[0] & KEYCODE_BIT];
	
		if(data[0] & UPDOWN_EVENT_BIT)
		{
#if defined(CONFIG_MACH_S5PC110_PRESTIGE)
			if (!isFlipOpen)	{
#endif
				input_report_key(touchkey_driver->input_dev,keycode, 0);
				input_sync(touchkey_driver->input_dev);
				touchkey_press=0;
				//printk(KERN_DEBUG "touchkey release keycode:%d data:%d\n", keycode,data[0]);
				printk(KERN_DEBUG "touchkey release data:%d\n", data[0]);
#if defined(CONFIG_MACH_S5PC110_PRESTIGE)
			}
#endif

		}
		else
		{
			if(touch_state_val == 1) 
			{
				printk(KERN_DEBUG "touchkey pressed but don't send event because touch is pressed. \n");
				i2c_touchkey_write(&cmd, 1); // to turn off LED when touch pressed
				set_touchkey_debug('P');
			}
			else
			{
				if(show_sensitivity == 0)
				{
					if(keycode==TOUCHKEY_KEYCODE_BACK)
					{
						TSP_forced_release();// if back key is pressed, release multitouch
					}
				}
				else if(show_sensitivity == 1)
				{
#if 0
					if(keycode==TOUCHKEY_KEYCODE_MENU)
					{
						menu_sensitivity = data[3];
					}
					else if(keycode==TOUCHKEY_KEYCODE_HOME)
					{
						home_sensitivity = data[4];
					}
					else if(keycode==TOUCHKEY_KEYCODE_BACK)
					{
						back_sensitivity = data[5];
					}
#endif
				}
#if defined(CONFIG_MACH_S5PC110_PRESTIGE)
				if (!isFlipOpen)	{
#endif
					//printk(KERN_DEBUG "sens=%d %d %d\n", data[3], data[4], data[5]);
#if defined(CONFIG_MACH_S5PC110_GALAXY)					
					if((touchkey_press==1)&&(old_keycode==158)){	//screen capture problem defence code
						input_report_key(touchkey_driver->input_dev, 158,0);
						input_sync(touchkey_driver->input_dev);
						touchkey_press=0;
						//printk(KERN_DEBUG "touchkey force release keycode :158 data:%d\n", keycode,data[0]);
						printk(KERN_DEBUG "touchkey force release data:%d\n", data[0]);
					}
					else{	
#endif
					input_report_key(touchkey_driver->input_dev, keycode,1);
					input_sync(touchkey_driver->input_dev);
						touchkey_press=1;
						old_keycode=keycode;
						//printk(KERN_DEBUG "touchkey press keycode :%d data:%d\n", keycode,data[0]);
						printk(KERN_DEBUG "touchkey press data:%d\n", data[0]);
#if defined(CONFIG_MACH_S5PC110_GALAXY)					
					}
#endif						
#if defined(CONFIG_MACH_S5PC110_PRESTIGE)
				}
#endif
			}
		}
	}

	//clear interrupt
	if(readl(gpio_pend_mask_mem)&(0x1<<1))
	{
		writel(readl(gpio_pend_mask_mem)|(0x1<<1), gpio_pend_mask_mem); 
	}

	set_touchkey_debug('A');
	enable_irq(IRQ_TOUCH_INT);
}


static irqreturn_t touchkey_interrupt(int irq, void *dummy)
{
	set_touchkey_debug('I');
	disable_irq_nosync(IRQ_TOUCH_INT);
	queue_work(touchkey_wq, &touchkey_driver->work);

	return IRQ_HANDLED;
}

#if defined(CONFIG_MACH_S5PC110_PRESTIGE)
extern unsigned int s3c_keygpio_get_flip_status(void);

void melfas_touchkey_suspend()
{
    set_touchkey_debug('S');
    printk(KERN_DEBUG "melfas_touchkey_suspend : %x : %x\n", touchkey_driver, touchkey_enable);

    if (touchkey_driver == NULL)
        return;

	if (touchkey_enable == 0)
		return;

	mutex_lock(&melfas_touchkey_use);

    if(touchkey_enable < 0)
	{		
        printk(KERN_DEBUG "---%s---touchkey_enable: %d\n",__FUNCTION__, touchkey_enable);
		mutex_unlock(&melfas_touchkey_use);
        return;
    }
	
    disable_irq(IRQ_TOUCH_INT);
    gpio_direction_output(_3_GPIO_TOUCH_EN, 0);
    gpio_direction_output(_3_GPIO_TOUCH_CE, 0);
    gpio_direction_output(_3_TOUCH_SDA_28V, 0);
    gpio_direction_output(_3_TOUCH_SCL_28V, 0);
    touchkey_enable = 0;

	mutex_unlock(&melfas_touchkey_use);
}

void melfas_touchkey_resume()
{
    unsigned char data;

    set_touchkey_debug('R');
    printk(KERN_DEBUG "melfas_touchkey_resume : %x : %x\n", touchkey_driver, touchkey_enable);

    if (touchkey_driver == NULL)
        return;

	if (touchkey_enable == 1)
		return;

	mutex_lock(&melfas_touchkey_use);

    if(touchkey_enable < 0)
    {
        printk(KERN_DEBUG "---%s---touchkey_enable: %d\n",__FUNCTION__, touchkey_enable);
		mutex_unlock(&melfas_touchkey_use);
        return;
    }
    gpio_direction_output(_3_GPIO_TOUCH_EN, 1);
    gpio_direction_output(_3_GPIO_TOUCH_CE, 1);
 
    msleep(50);

    //clear interrupt
    if(readl(gpio_pend_mask_mem)&(0x1<<1))
        writel(readl(gpio_pend_mask_mem)|(0x1<<1), gpio_pend_mask_mem); 

    enable_irq(IRQ_TOUCH_INT);
    touchkey_enable = 1;

	mutex_unlock(&melfas_touchkey_use);
}
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
static void melfas_touchkey_early_suspend(struct early_suspend *h)
{
    set_touchkey_debug('S');
    printk(KERN_DEBUG "melfas_touchkey_early_suspend : %x\n", touchkey_enable);

	if (touchkey_enable == 0)
		return;

	mutex_lock(&melfas_touchkey_use);

    if(touchkey_enable < 0)
    {		
        printk(KERN_DEBUG "---%s---touchkey_enable: %d\n",__FUNCTION__, touchkey_enable);
		mutex_unlock(&melfas_touchkey_use);
        return;
    }
    disable_irq(IRQ_TOUCH_INT);
    gpio_direction_output(_3_GPIO_TOUCH_EN, 0);
    gpio_direction_output(_3_GPIO_TOUCH_CE, 0);
    gpio_direction_output(_3_TOUCH_SDA_28V, 0);
    gpio_direction_output(_3_TOUCH_SCL_28V, 0);
    touchkey_enable = 0;	

    mutex_unlock(&melfas_touchkey_use);

}

static void melfas_touchkey_early_resume(struct early_suspend *h)
{
	set_touchkey_debug('R');
    printk(KERN_DEBUG "melfas_touchkey_early_resume : %x\n", touchkey_enable);

	if (touchkey_enable == 1)
		return;

	mutex_lock(&melfas_touchkey_use);

    if(touchkey_enable < 0)
    {
        printk(KERN_DEBUG "---%s---touchkey_enable: %d\n",__FUNCTION__, touchkey_enable);
		mutex_unlock(&melfas_touchkey_use);
        return;
    }
	gpio_direction_output(_3_GPIO_TOUCH_EN, 1);
	gpio_direction_output(_3_GPIO_TOUCH_CE, 1);

    msleep(50);

	//clear interrupt
	if (readl(gpio_pend_mask_mem) & (0x1 << 1))
        writel(readl(gpio_pend_mask_mem)|(0x1<<1), gpio_pend_mask_mem); 

	enable_irq(IRQ_TOUCH_INT);
	touchkey_enable = 1;

	mutex_unlock(&melfas_touchkey_use);

}
#endif				// End of CONFIG_HAS_EARLYSUSPEND

extern int mcsdl_download_binary_data(void);

static int i2c_touchkey_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct input_dev *input_dev;
	int err = 0;

#if defined(CONFIG_MACH_S5PC110_PRESTIGE)
	unsigned int flip_status;
#endif

	touchkey_driver =
	    kzalloc(sizeof(struct i2c_touchkey_driver), GFP_KERNEL);
	if (touchkey_driver == NULL) {
		dev_err(dev, "failed to create our state\n");
		return -ENOMEM;
	}

	touchkey_driver->client = client;

	touchkey_driver->client->irq = IRQ_TOUCH_INT;
	strlcpy(touchkey_driver->client->name, "melfas-touchkey", I2C_NAME_SIZE);

	input_dev = input_allocate_device();

	if (!input_dev)
		return -ENOMEM;

	touchkey_driver->input_dev = input_dev;

	input_dev->name = DEVICE_NAME;
	input_dev->phys = "melfas-touchkey/input0";
	input_dev->id.bustype = BUS_HOST;

	set_bit(EV_SYN, input_dev->evbit);
	set_bit(EV_KEY, input_dev->evbit);
	set_bit(touchkey_keycode[1], input_dev->keybit);
	set_bit(touchkey_keycode[2], input_dev->keybit);
#if defined(CONFIG_MACH_S5PC110_PRESTIGE)
	set_bit(touchkey_keycode[3], input_dev->keybit);
	set_bit(touchkey_keycode[4], input_dev->keybit);
#endif

	err = input_register_device(input_dev);
	if (err) {
		input_free_device(input_dev);
		return err;
	}

	gpio_pend_mask_mem = ioremap(INT_PEND_BASE, 0x10);
	touchkey_wq = create_singlethread_workqueue("melfas_touchkey_wq");
	if (!touchkey_wq)
		return -ENOMEM;

	INIT_WORK(&touchkey_driver->work, touchkey_work_func);
//       INIT_DELAYED_WORK(&touch_resume_work, touchkey_resume_func);

#ifdef CONFIG_HAS_EARLYSUSPEND
	touchkey_driver->early_suspend.suspend = melfas_touchkey_early_suspend;
	touchkey_driver->early_suspend.resume = melfas_touchkey_early_resume;
	register_early_suspend(&touchkey_driver->early_suspend);
#endif				/* CONFIG_HAS_EARLYSUSPEND */

	touchkey_enable = 1;

	if (request_irq(IRQ_TOUCH_INT, touchkey_interrupt, IRQF_DISABLED, DEVICE_NAME, touchkey_driver)) {
		printk(KERN_ERR "%s Can't allocate irq ..\n", __FUNCTION__);
		return -EBUSY;
	}

	set_touchkey_debug('K');

#if defined(CONFIG_MACH_S5PC110_PRESTIGE)
	if(s3c_keygpio_get_flip_status() == 0)	{	// FLIP OPEN
		touchkey_enable = 1;
		melfas_touchkey_suspend();
	} else	{									// FLIP CLOSE
		touchkey_enable = 0;
		melfas_touchkey_resume();
	}
#endif

	return 0;
}

static void init_hw(void)
{
	if (gpio_is_valid(_3_GPIO_TOUCH_EN)) {
		if (gpio_request(_3_GPIO_TOUCH_EN, "MP03"))
			printk(KERN_ERR "Filed to request _3_GPIO_TOUCH_EN!\n");
		gpio_direction_output(_3_GPIO_TOUCH_EN, GPIO_LEVEL_HIGH);
	}

	if (gpio_is_valid(_3_GPIO_TOUCH_CE)) {
		if (gpio_request(_3_GPIO_TOUCH_CE, "MP03"))
			printk(KERN_ERR "Filed to request _3_GPIO_TOUCH_CE!\n");
		gpio_direction_output(_3_GPIO_TOUCH_CE, GPIO_LEVEL_HIGH);
	}

	msleep(200);
	s3c_gpio_setpull(_3_GPIO_TOUCH_INT, S3C_GPIO_PULL_NONE);
#if defined (CONFIG_MACH_S5PC110_ARIESCT) || defined (CONFIG_MACH_S5PC110_PRESTIGE)
	s3c_gpio_cfgpin(S5PV210_GPG3(2), S3C_GPIO_SFN(0xf));
#else
	s3c_gpio_cfgpin(S5PV210_GPJ4(1), S3C_GPIO_SFN(0xf));
#endif
	set_irq_type(IRQ_TOUCH_INT, IRQ_TYPE_LEVEL_LOW);
}


int touchkey_update_open(struct inode *inode, struct file *filp)
{
	return 0;
}


ssize_t touchkey_update_read(struct file * filp, char *buf, size_t count, loff_t * f_pos)
{
	char data[3]={0,};

	get_touchkey_firmware(data);
	put_user(data[1], buf);

	return 1;
}

static ssize_t touchkey_show_sensitivity(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	u8 data;
	if(sscanf(buf, "%d\n", &data) == 1) 
	{
		show_sensitivity = data;
	}
	else
	{
		printk("touchkey_show_sensitivity Error\n");
	}

	return size;
}


static ssize_t touchkey_version_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	u8 version[3];
	//printk("called %s \n",__func__);
	get_touchkey_firmware(version);
	return sprintf(buf,"%02x\n",version[1]);
}

static ssize_t touchkey_recommend_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	u8 version[3];
	//printk("called %s \n",__func__);
	get_touchkey_firmware(version);

	recommended_ver = version[1];

	return sprintf(buf,"%02x\n",recommended_ver);
}

#if 0
extern int mcsdl_download_binary_file(unsigned char *pData, unsigned short nBinary_length);
ssize_t touchkey_update_write(struct file *filp, const char *buf, size_t count, loff_t * f_pos)
{
	unsigned char *pdata;

	disable_irq(IRQ_TOUCH_INT);
	printk("count = %d\n", count);
	pdata = kzalloc(count, GFP_KERNEL);
	if (pdata == NULL) {
		printk("memory allocate fail \n");
		return 0;
	}
	if (copy_from_user(pdata, buf, count)) {
		printk("copy fail \n");
		kfree(pdata);
		return 0;
	}

	mcsdl_download_binary_file((unsigned char *)pdata,
				   (unsigned short)count);
	kfree(pdata);

	init_hw();
	enable_irq(IRQ_TOUCH_INT);
	return count;
}
#endif
int touchkey_update_release(struct inode *inode, struct file *filp)
{
	return 0;
}

struct file_operations touchkey_update_fops = {
	.owner = THIS_MODULE,
	.read = touchkey_update_read,
//      .write   = touchkey_update_write,
	.open = touchkey_update_open,
	.release = touchkey_update_release,
};

static struct miscdevice touchkey_update_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "melfas_touchkey",
	.fops = &touchkey_update_fops,
};

static ssize_t touch_version_read(struct device *dev, struct device_attribute *attr, char *buf)
{
	char data[3]={0,};
	int count;

	init_hw();
	if(get_touchkey_firmware(data) !=0)
	{
		i2c_touchkey_read(KEYCODE_REG, data, 3);
	}
	count = sprintf(buf,"0x%x\n", data[1]);

	printk("touch_version_read 0x%x\n", data[1]);
	return count;
}

static ssize_t touch_version_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	//buf[size]=0;
	printk("input data --> %s\n", buf);

	return size;
}


//extern int mcsdl_download_binary_file(void);
static int touchkey_update_status = 0;

void touchkey_update_func(struct work_struct *p)
{
#if 0
	int retry = 10;
	touchkey_update_status = 1;
	printk("%s start\n", __FUNCTION__);
	while (retry--) {
		if (mcsdl_download_binary_file() == 1) {
			touchkey_update_status = 0;
			printk("touchkey_update successed\n");
			return;
		}
	}
	touchkey_update_status = -1;
	printk("touchkey_update failed\n");
	return;
#endif
}

static ssize_t touch_update_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	printk("touchkey firmware update \n");
	if(*buf == 'S')
	{
		disable_irq(IRQ_TOUCH_INT);
		INIT_WORK(&touch_update_work, touchkey_update_func);
		queue_work(touchkey_wq, &touch_update_work);
	}
	return size;
}

static ssize_t touch_update_read(struct device *dev, struct device_attribute *attr, char *buf)
{
	int count;

	printk("touch_update_read: touchkey_update_status %d\n", touchkey_update_status);
	
	if(touchkey_update_status == 0)
	{
		count = sprintf(buf,"PASS\n");
	}
	else if(touchkey_update_status == 1)
	{
		count = sprintf(buf,"Downloading\n");
	}
	else if(touchkey_update_status == -1)
	{
		count = sprintf(buf,"Fail\n");
	}	

	return count;
}

#if defined(CONFIG_MACH_S5PC110_PRESTIGE)
static ssize_t touchkey_home_show(struct device *dev, struct device_attribute *attr, char *buf)
{	
	//printk("called %s \n",__func__);
	return sprintf(buf,"%d\n",home_sensitivity);
}

static ssize_t touchkey_search_show(struct device *dev, struct device_attribute *attr, char *buf)
{	
	//printk("called %s \n",__func__);
	return sprintf(buf,"%d\n",search_sensitivity);
}
#endif

static ssize_t touchkey_menu_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	//printk("called %s \n",__func__);
	return sprintf(buf,"%d\n",menu_sensitivity);
}

static ssize_t touchkey_back_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	//printk("called %s \n",__func__);
	return sprintf(buf,"%d\n",back_sensitivity);
}

static ssize_t touch_led_control(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	unsigned char data;
	if(sscanf(buf, "%d\n", &data) == 1) 
	{
/*		
		if(is_debug_screen==1) // key led off at debug screen
		{
			data = 2;	
		}	
*/
		printk(KERN_DEBUG "touch_led_control: %d \n", data);
		i2c_touchkey_write(&data, 1);
	}
	else
	{
		printk("touch_led_control Error\n");
	}
	return size;
}

static ssize_t touchkey_enable_disable(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
    #if 0
    printk("touchkey_enable_disable %c \n", *buf);
    if(*buf == '0')
    {
        set_touchkey_debug('d');
        disable_irq(IRQ_TOUCH_INT);
        gpio_direction_output(_3_GPIO_TOUCH_EN, 0);
        gpio_direction_output(_3_GPIO_TOUCH_CE, 0);
        touchkey_enable = -2;
    }
    else if(*buf == '1')
    {
        if(touchkey_enable == -2)
        {
            set_touchkey_debug('e');
            gpio_direction_output(_3_GPIO_TOUCH_EN, 1);
            gpio_direction_output(_3_GPIO_TOUCH_CE, 1);
            touchkey_enable = 1;
            enable_irq(IRQ_TOUCH_INT);
        }
    }
    else
    {
        printk("touchkey_enable_disable: unknown command %c \n", *buf);
    }
    #endif
    return size;
}

int	s5pc110_upper_board_touchkey = 0; // Default : No Exist

int hw_upper_board_check_touchkey(void)
{
	return s5pc110_upper_board_touchkey;
}
EXPORT_SYMBOL(hw_upper_board_check_touchkey);

static DEVICE_ATTR(touch_version, 0664, touch_version_read, touch_version_write);
static DEVICE_ATTR(touch_update, 0664, touch_update_read, touch_update_write);
static DEVICE_ATTR(brightness, 0664, NULL, touch_led_control);
static DEVICE_ATTR(enable_disable, 0664, NULL, touchkey_enable_disable);
static DEVICE_ATTR(show_sensitivity, 0664, NULL, touchkey_show_sensitivity);
static DEVICE_ATTR(touchkey_version, 0664, touchkey_version_show, NULL);
static DEVICE_ATTR(touchkey_recommend, 0664, touchkey_recommend_show, NULL);
static DEVICE_ATTR(touchkey_menu, 0664, touchkey_menu_show, NULL);
static DEVICE_ATTR(touchkey_back, 0664, touchkey_back_show, NULL);
#if defined (CONFIG_MACH_S5PC110_PRESTIGE)
static DEVICE_ATTR(touchkey_home, 0664, touchkey_home_show, NULL);
static DEVICE_ATTR(touchkey_search, 0664, touchkey_search_show, NULL);
#endif

static int __init touchkey_init(void)
{
	int ret = 0;
	int retry = 10;
	char data[3]={0,};

#if defined(CONFIG_MACH_S5PC110_LATONA)
/* Latona does not use Melfas Touchkey, but used Cypress TSP + Touchkey */
	return 0;
#endif

	ret = misc_register(&touchkey_update_device);
	if (ret) {
		printk("%s misc_register fail\n", __FUNCTION__);
	}

	if (device_create_file(touchkey_update_device.this_device, &dev_attr_touch_version) < 0)
	{
		printk("%s device_create_file fail dev_attr_touch_version\n",__FUNCTION__);
		pr_err("Failed to create device file(%s)!\n", dev_attr_touch_version.attr.name);
	}

	if (device_create_file(touchkey_update_device.this_device, &dev_attr_touch_update) < 0)
	{
		printk("%s device_create_file fail dev_attr_touch_update\n",__FUNCTION__);
		pr_err("Failed to create device file(%s)!\n", dev_attr_touch_update.attr.name);
	}

	if (device_create_file(touchkey_update_device.this_device, &dev_attr_brightness) < 0)
	{
		printk("%s device_create_file fail dev_attr_touch_update\n",__FUNCTION__);
		pr_err("Failed to create device file(%s)!\n", dev_attr_brightness.attr.name);
	}

	if (device_create_file(touchkey_update_device.this_device, &dev_attr_enable_disable) < 0)
	{
		printk("%s device_create_file fail dev_attr_touch_update\n",__FUNCTION__);
		pr_err("Failed to create device file(%s)!\n", dev_attr_enable_disable.attr.name);
	}

	if (device_create_file(touchkey_update_device.this_device, &dev_attr_show_sensitivity) < 0)
	{
		//printk("%s device_create_file fail dev_attr_show_sensitivity\n",__FUNCTION__);
		pr_err("Failed to create device file(%s)!\n", dev_attr_show_sensitivity.attr.name);
	}
	
	if (device_create_file(touchkey_update_device.this_device, &dev_attr_touchkey_version) < 0)
	{
		//printk("%s device_create_file fail dev_attr_touchkey_version\n",__FUNCTION__);
		pr_err("Failed to create device file(%s)!\n", dev_attr_touchkey_version.attr.name);
	}

	if (device_create_file(touchkey_update_device.this_device, &dev_attr_touchkey_recommend) < 0)
	{
		//printk("%s device_create_file fail dev_attr_touchkey_recommend\n",__FUNCTION__);
		pr_err("Failed to create device file(%s)!\n", dev_attr_touchkey_recommend.attr.name);
	}

	if (device_create_file(touchkey_update_device.this_device, &dev_attr_touchkey_menu) < 0)
	{
		//printk("%s device_create_file fail dev_attr_touchkey_menu\n",__FUNCTION__);
		pr_err("Failed to create device file(%s)!\n", dev_attr_touchkey_menu.attr.name);
	}

	if (device_create_file(touchkey_update_device.this_device, &dev_attr_touchkey_back) < 0)
	{
		//printk("%s device_create_file fail dev_attr_touchkey_back\n",__FUNCTION__);
		pr_err("Failed to create device file(%s)!\n", dev_attr_touchkey_back.attr.name);
	}

#if defined (CONFIG_MACH_S5PC110_PRESTIGE)
	if (device_create_file(touchkey_update_device.this_device, &dev_attr_touchkey_home) < 0)
	{
		//printk("%s device_create_file fail dev_attr_touchkey_home\n",__FUNCTION__);
		pr_err("Failed to create device file(%s)!\n", dev_attr_touchkey_home.attr.name);
	}

	if (device_create_file(touchkey_update_device.this_device, &dev_attr_touchkey_search) < 0)
	{
		//printk("%s device_create_file fail dev_attr_touchkey_search\n",__FUNCTION__);
		pr_err("Failed to create device file(%s)!\n", dev_attr_touchkey_search.attr.name);
	}
#endif

	init_hw();
	while(retry--)
	{
		if(get_touchkey_firmware(data) == 0) //melfas need delay for multiple read
			break;
	}
	printk(KERN_DEBUG "%s F/W version: 0x%x, Module version:0x%x\n",__FUNCTION__, data[1], data[2]);
	touch_version = data[1];
	retry = 3;
	printk(KERN_DEBUG "HWREV is 0x%x\n",HWREV);

	if ((1 <= data[1]) && (data[1] <= 20))	{
		s5pc110_upper_board_touchkey = 1;
	}
	printk("Upper Board Touchkey is 0x%x\n", s5pc110_upper_board_touchkey);

#if 0
	if((data[1]==0x0) || (data[1]==0xff)) 
	{
		// do nothing
	}
	else if (data[1] < 0x08) 
	{
		set_touchkey_debug('U');
		while (retry--) 
		{
			if (mcsdl_download_binary_data() == 1) 
			{
				printk("touchkey_update successed\n");
				set_touchkey_debug('C');
				break;
			}
			printk("touchkey_update failed... retry...\n");
			set_touchkey_debug('f');
		}
		if(retry <= 0)
		{
			gpio_direction_output(_3_GPIO_TOUCH_EN, 0);
			gpio_direction_output(_3_GPIO_TOUCH_CE, 0);
			msleep(300);
		}
		init_hw(); //after update, re initalize.
	}
#endif
	ret = i2c_add_driver(&touchkey_i2c_driver);

	if (ret) {
		printk
		    ("melfas touch keypad registration failed, module not inserted.ret= %d\n",
		     ret);
	}
	return ret;
}

static void __exit touchkey_exit(void)
{
	printk("%s \n", __FUNCTION__);
	i2c_del_driver(&touchkey_i2c_driver);
	misc_deregister(&touchkey_update_device);
	if (touchkey_wq)
		destroy_workqueue(touchkey_wq);

	gpio_free(_3_GPIO_TOUCH_CE);
	gpio_free(_3_GPIO_TOUCH_EN);
//	gpio_free(_3_TOUCH_SDA_28V);
//	gpio_free(_3_TOUCH_SCL_28V);
}

module_init(touchkey_init);
module_exit(touchkey_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("@@@");
MODULE_DESCRIPTION("melfas touch keypad");
