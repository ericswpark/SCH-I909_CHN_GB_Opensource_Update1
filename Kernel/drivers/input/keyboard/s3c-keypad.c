/* drivers/input/keyboard/s3c-keypad.c
 *
 * Driver core for Samsung SoC onboard UARTs.
 *
 * Kim Kyoungil, Copyright (c) 2006-2009 Samsung Electronics
 *      http://www.samsungsemi.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/clk.h>
#include <linux/irq.h>
#include <linux/slab.h>
#include <linux/i2c.h>


#include "../../staging/android/timed_output.h"

#include <linux/io.h>
#include <mach/hardware.h>
#include <asm/delay.h>
#include <asm/irq.h>

#include <mach/regs-gpio.h>
#include <mach/gpio.h>
#include <mach/gpio-aries.h>

#include <plat/gpio-cfg.h>
#include <plat/regs-keypad.h>
#ifdef CONFIG_CPU_FREQ
#include <mach/cpu-freq-v210.h>
#endif
 
#include "s3c-keypad.h"

#if defined(CONFIG_MACH_S5PC110_PRESTIGE)
#include <linux/wakelock.h>
static struct wake_lock lcd_wake_lock;
extern void s3c_setup_keypad_cfg_gpio(int rows, int columns);
#endif

#if defined(CONFIG_MACH_S5PC110_PRESTIGE) || defined (CONFIG_MACH_S5PC110_GALAXY) || defined (CONFIG_MACH_S5PC110_LATONA) //TEMP for compile//TEMP
#define S5PV210_GPH0DAT			(S5PV210_GPH0_BASE + 0x04)
#define S5PV210_GPH1DAT			(S5PV210_GPH1_BASE + 0x04)
#define S5PV210_GPH2DAT			(S5PV210_GPH2_BASE + 0x04)
#define S5PV210_GPH3DAT			(S5PV210_GPH3_BASE + 0x04)
enum freq_level_states {
	LEV_1200MHZ,
	LEV_1000MHZ,
	LEV_800MHZ,
	LEV_400MHZ,
	LEV_200MHZ,
	LEV_100MHZ,
};

enum key_input_status {
	KEY_PRESS,
	KEY_RELEASE,
};
#endif


#define USE_PERF_LEVEL_KEYPAD 1 
#undef S3C_KEYPAD_DEBUG 

#ifdef S3C_KEYPAD_DEBUG
#define DPRINTK(x...) printk("S3C-Keypad " x)
#define INPUT_REPORT_KEY(a,b,c) do {				\
		printk(KERN_ERR "%s:%d input_report_key(%x, %d, %d)\n", \
		       __func__, __LINE__, a, b, c);			\
		input_report_key(a,b,c);				\
	} while (0)
#else
#define DPRINTK(x...)		/* !!!! */
#define INPUT_REPORT_KEY	input_report_key
#endif

#define DEVICE_NAME "s3c-keypad"

#define TRUE 1
#define FALSE 0
#define	SUBJECT	"s3c_keypad.c"
#define P(format,...)\
    printk ("[ "SUBJECT " (%s,%d) ] " format "\n", __func__, __LINE__, ## __VA_ARGS__);
#define FI \
    printk ("[ "SUBJECT " (%s,%d) ] " "%s - IN" "\n", __func__, __LINE__, __func__);
#define FO \
    printk ("[ "SUBJECT " (%s,%d) ] " "%s - OUT" "\n", __func__, __LINE__, __func__);

#define FLASH_MOVIE_MODE
#if defined(CONFIG_MACH_S5PC110_PRESTIGE) || defined (CONFIG_MACH_S5PC110_GALAXY) || defined (CONFIG_MACH_S5PC110_LATONA) //TEMP for compile
#undef FLASH_MOVIE_MODE
#endif

static struct timer_list keypad_timer;
static int is_timer_on = FALSE;
static struct clk *keypad_clock;


static u32 keymask[KEYPAD_COLUMNS];
static u32 prevmask[KEYPAD_COLUMNS];

#if defined(CONFIG_MACH_S5PC110_PRESTIGE) || defined (CONFIG_MACH_S5PC110_GALAXY) || defined (CONFIG_MACH_S5PC110_LATONA) //TEMP for compile
static int in_sleep = 0;

#include <linux/switch.h>

struct switch_dev switch_flip = {
		.name = "flip",
};

#if defined(CONFIG_MACH_S5PC110_PRESTIGE)

extern unsigned int HWREV;
extern unsigned int W899_HWREV;

static unsigned int current_flip_status;
static unsigned int wq_flip_status;

static struct workqueue_struct *keygpio_flip_wq;
static struct work_struct keygpio_flip_work;

static struct workqueue_struct *keygpio_tsp_wq;
static struct delayed_work keygpio_tsp_work;

#define FLIP_SCAN_INTERVAL	(50)	/* ms */
#define FLIP_STABLE_COUNT	(2)
static int flip_state;
static int flip_count;
#define TSP_WQ_DELAY		(500)	/* ms */

#endif

extern int IsLDIEnabled(void);
extern void tl2796_set_working_ldi(int working_ldi);
extern void tl2796_set_working_ldi_in_isr(int working_ldi);

extern void tl2796_switch_main2sub_ldi(void);
extern void tl2796_switch_sub2main_ldi(void);

extern void qt602240_set_working_tsp(int working_tsp);
extern void qt602240_switch_sub2main_tsp(void);
extern void qt602240_switch_main2sub_tsp(void);
extern void qt602240_switch_suspend(void);

extern void melfas_touchkey_suspend(void);
extern void melfas_touchkey_resume(void);
extern int gp2a_flip_notify(int onoff);

extern void set_dvfs_target_level_in_isr(enum freq_level_states freq_level);
#if defined(CONFIG_MACH_S5PC110_PRESTIGE) || defined (CONFIG_MACH_S5PC110_GALAXY) || defined (CONFIG_MACH_S5PC110_LATONA)
extern bool charging_mode_get(void);
#endif

static struct work_struct keygpio_ok_key_work;
static struct work_struct keygpio_key_timer_work;

struct s3c_keypad *s3c_keypad;

static int keypad_is_key_supported(int keycode)
{
	int	ret = 1;

#if defined(CONFIG_MACH_S5PC110_PRESTIGE)
	/*
	 * If Folder closed,
	 * Don't occur key events,
	 * !EXCEPT! specific keycodes.
	 */

	if (current_flip_status == 1)	{
		switch (keycode)	{
			case 115:	// VOL_UP
			case 114:	// VOL_DOWN
			case 116:	// HOLD
				ret = 1;
			break;

			default:
				ret = 0;
			break;
		}
	}
#endif

	return ret;
}


/*static*/ int keypad_scan(void)
{

	u32 col,cval,rval,gpio;

	DPRINTK("H3C %x H2C %x \n",readl(S5PV210_GPH3CON),readl(S5PV210_GPH2CON));
	DPRINTK("keypad_scan() is called\n");

	DPRINTK("row val = %x",readl(key_base + S3C_KEYIFROW));

	for (col=0; col < KEYPAD_COLUMNS; col++) {

		cval = KEYCOL_DMASK & ~((1 << col) | (1 << col+ 8)); // clear that column number and 

		writel(cval, key_base+S3C_KEYIFCOL);             // make that Normal output.
								 // others shuld be High-Z output.
		udelay(KEYPAD_DELAY);

#if defined(CONFIG_MACH_S5PC110_PRESTIGE)
		rval = ~(readl(key_base+S3C_KEYIFROW)) & ((1<<KEYPAD_ROWS+2)-1) ;
		if (rval & 0x80)
			rval = 0x20 + (rval & 0x1F);
		else
			rval &= 0x1F;
#else
		rval = ~(readl(key_base+S3C_KEYIFROW)) & ((1<<KEYPAD_ROWS)-1) ;
#endif
		keymask[col] = rval; 
	}

	writel(KEYIFCOL_CLEAR, key_base+S3C_KEYIFCOL);

	return 0;
}

static void keypad_timer_handler(unsigned long data)
{
	u32 press_mask;
	u32 release_mask;
	u32 restart_timer = 0;
	int i,col;
	struct s3c_keypad *pdata = (struct s3c_keypad *)data;
	struct input_dev *dev = pdata->dev;

	keypad_scan();


	for(col=0; col < KEYPAD_COLUMNS; col++) {
		press_mask = ((keymask[col] ^ prevmask[col]) & keymask[col]); 
		release_mask = ((keymask[col] ^ prevmask[col]) & prevmask[col]); 

#ifdef CONFIG_CPU_FREQ
#if USE_PERF_LEVEL_KEYPAD
#if defined(CONFIG_MACH_S5PC110_PRESTIGE)
		if (press_mask || release_mask)
			queue_work(keygpio_flip_wq, &keygpio_key_timer_work);
#endif
#endif
#endif
		i = col * KEYPAD_ROWS;

		while (press_mask) {
			if (press_mask & 1) {
				if (keypad_is_key_supported(pdata->keycodes[i])) {
					input_report_key(dev,pdata->keycodes[i],1);
					DPRINTK("\nkey Pressed  : key %d map %d\n",i, pdata->keycodes[i]);
				}
			}
			press_mask >>= 1;
			i++;
		}

		i = col * KEYPAD_ROWS;

		while (release_mask) {
			if (release_mask & 1) {
				if (keypad_is_key_supported(pdata->keycodes[i])) {
					input_report_key(dev,pdata->keycodes[i],0);
					DPRINTK("\nkey Released : %d  map %d\n",i,pdata->keycodes[i]);
				}
            }
			release_mask >>= 1;
			i++;
		}
		prevmask[col] = keymask[col];

		restart_timer |= keymask[col];
	}


	if (restart_timer) {
		mod_timer(&keypad_timer,jiffies + HZ/10);
	} else {
		writel(KEYIFCON_INIT, key_base+S3C_KEYIFCON);
		is_timer_on = FALSE;
	}

}


static irqreturn_t s3c_keypad_isr(int irq, void *dev_id)
{

	/* disable keypad interrupt and schedule for keypad timer handler */
	writel(readl(key_base+S3C_KEYIFCON) & ~(INT_F_EN|INT_R_EN), key_base+S3C_KEYIFCON);

	keypad_timer.expires = jiffies;
	if ( is_timer_on == FALSE) {
		add_timer(&keypad_timer);
		is_timer_on = TRUE;
	} else {
		mod_timer(&keypad_timer,keypad_timer.expires);
	}
	/*Clear the keypad interrupt status*/
	writel(KEYIFSTSCLR_CLEAR, key_base+S3C_KEYIFSTSCLR);
	
	return IRQ_HANDLED;
}



static irqreturn_t s3c_keygpio_isr(int irq, void *dev_id)
{
	unsigned int key_status;
	static unsigned int prev_key_status = (1 << 6);
	struct s3c_keypad *pdata = (struct s3c_keypad *)dev_id;
	struct input_dev *dev = pdata->dev;

	// Beware that we may not obtain exact key up/down status at
	// this point.
	key_status = (readl(S5PV210_GPH2DAT)) & (1 << 6);

	// If ISR is called and up/down status remains the same, we
	// must have lost one and should report that first with
	// upside/down.
	if(in_sleep)
	{
		if (key_status == prev_key_status)
		{
#if defined(CONFIG_MACH_S5PC110_PRESTIGE)
			if (keypad_is_key_supported(END_KEYCODE))
			INPUT_REPORT_KEY(dev, END_KEYCODE, key_status ? 1 : 0);
#else
			INPUT_REPORT_KEY(dev, 116, key_status ? 1 : 0);
#endif
		}
		in_sleep = 0;
	}

#if defined(CONFIG_MACH_S5PC110_PRESTIGE)
	if (keypad_is_key_supported(END_KEYCODE))	{
		if (!key_status)	{
			INPUT_REPORT_KEY(dev, END_APPS_KEYCODE, 1);
			INPUT_REPORT_KEY(dev, END_APPS_KEYCODE, 0);
		}
		INPUT_REPORT_KEY(dev, END_KEYCODE, key_status ? 0 : 1);
	}
#else
	INPUT_REPORT_KEY(dev, 116, key_status ? 0 : 1);
#endif

	prev_key_status = key_status;
	printk(KERN_DEBUG "s3c_keygpio_isr pwr key_status =%d\n", key_status);

	set_irq_type(IRQ_EINT(22), IRQ_TYPE_EDGE_BOTH);

	return IRQ_HANDLED;
}

static irqreturn_t s3c_keygpio_vol_up_isr(int irq, void *dev_id)
{
	unsigned int key_status;
	struct s3c_keypad *pdata = (struct s3c_keypad *)dev_id;
	struct input_dev *dev = pdata->dev;

	//we cannot check HWREV 0xb and 0xc, we check 2 hw key
	key_status = (readl(S5PV210_GPH3DAT)) & ((1 << 3));
	
	INPUT_REPORT_KEY(dev, 42, key_status ? 0 : 1);

       printk(KERN_DEBUG "s3c_keygpio_vol_up_isr key_status =%d,\n", key_status);
       
        return IRQ_HANDLED;
}

//EINT26
static irqreturn_t s3c_keygpio_vol_up26_isr(int irq, void *dev_id)
{
	unsigned int key_status;
	struct s3c_keypad *pdata = (struct s3c_keypad *)dev_id;
	struct input_dev *dev = pdata->dev;

	key_status = (readl(S5PV210_GPH3DAT)) & ((1 << 2));
	
	INPUT_REPORT_KEY(dev, 42, key_status ? 0 : 1);

       printk(KERN_DEBUG "s3c_keygpio_vol_up26_isr key_status =%d,\n", key_status);
       
        return IRQ_HANDLED;
}

static irqreturn_t s3c_keygpio_vol_down_isr(int irq, void *dev_id)
{
	unsigned int key_status;
	struct s3c_keypad *pdata = (struct s3c_keypad *)dev_id;
	struct input_dev *dev = pdata->dev;

	key_status = (readl(S5PV210_GPH3DAT)) & (1 << 1);
	
	INPUT_REPORT_KEY(dev, 58, key_status ? 0 : 1);

	printk(KERN_DEBUG "s3c_keygpio_vol_down_isr key_status =%d,\n", key_status);
	
        return IRQ_HANDLED;
}

#if defined(CONFIG_MACH_S5PC110_PRESTIGE) || defined (CONFIG_MACH_S5PC110_GALAXY) || defined (CONFIG_MACH_S5PC110_LATONA)
/* this is a temporary funtion, 
  * because lcd turns on only by key at lpm charging */
void lpm_report_ok_key(void)
{
#if defined(CONFIG_MACH_S5PC110_PRESTIGE)
	int keycode = OK_KEYCODE;
#else
	int keycode = 102;
#endif
	if(!charging_mode_get())
		return ;
	printk("%s\n", __func__);
	INPUT_REPORT_KEY(s3c_keypad->dev, keycode, KEY_PRESS);
	printk("%s this is only for delay\n", __func__);
	INPUT_REPORT_KEY(s3c_keypad->dev, keycode, KEY_RELEASE);
}
EXPORT_SYMBOL(lpm_report_ok_key);
#endif

extern void TSP_forced_release_forOKkey(void);
static irqreturn_t s3c_keygpio_ok_isr(int irq, void *dev_id)
{
	unsigned int key_status;
	static unsigned int prev_key_status = (1 << 5);
	struct s3c_keypad *pdata = (struct s3c_keypad *)dev_id;
	struct input_dev *dev = pdata->dev;

#if defined(CONFIG_MACH_S5PC110_PRESTIGE)
	queue_work(keygpio_flip_wq, &keygpio_ok_key_work);
#endif

	// Beware that we may not obtain exact key up/down status at
	// this point.
	key_status = (readl(S5PV210_GPH3DAT)) & ((1 << 5));

	// If ISR is called and up/down status remains the same, we
	// must have lost one and should report that first with
	// upside/down.
	if(in_sleep)
	{
		if (key_status == prev_key_status)
		{
#if defined(CONFIG_MACH_S5PC110_PRESTIGE)
			INPUT_REPORT_KEY(dev, OK_KEYCODE, key_status ? 1 : 0);
#else
			INPUT_REPORT_KEY(dev, 102, key_status ? 1 : 0);
#endif
		}
		in_sleep = 0;
	}

#if defined(CONFIG_MACH_S5PC110_PRESTIGE)
	INPUT_REPORT_KEY(dev, OK_KEYCODE, key_status ? 0 : 1);
#else
	INPUT_REPORT_KEY(dev, 102, key_status ? 0 : 1);
#endif

	if(key_status)
		TSP_forced_release_forOKkey();
	
	prev_key_status = key_status;
        printk(KERN_DEBUG "s3c_keygpio_ok_isr key_status =%d\n", key_status);
        
        return IRQ_HANDLED;
}

void s3c_keygpio_key_timer_handler(void)
{
#ifdef CONFIG_CPU_FREQ
#if USE_PERF_LEVEL_KEYPAD
//TEMP	set_dvfs_target_level(LEV_400MHZ);
#endif
#endif
}

void s3c_keygpio_ok_key_handler(void)
{
#ifdef CONFIG_CPU_FREQ
//TEMP	set_dvfs_target_level(LEV_800MHZ);
#endif
}

#if defined(CONFIG_MACH_S5PC110_PRESTIGE)
static irqreturn_t s3c_keygpio_flip_isr(int irq, void *dev_id)
{

#if defined(CONFIG_MACH_S5PC110_PRESTIGE)
	wake_lock_timeout(&lcd_wake_lock, 1 * HZ);
#endif

#if 1
	unsigned long state;
	state = (readl(S5PV210_GPH0DAT)) & (1 << 6);
#endif

	if (!timer_pending(&s3c_keypad->flip_timer))
		mod_timer(&s3c_keypad->flip_timer, jiffies + msecs_to_jiffies(FLIP_SCAN_INTERVAL));
	
#if 1
	if (IsLDIEnabled() == false)	{
		if(state)	{
			qt602240_set_working_tsp(1);
			tl2796_set_working_ldi_in_isr(true);
		} else	{
			qt602240_set_working_tsp(0);
			tl2796_set_working_ldi_in_isr(false);
		}
	}
#endif
	return IRQ_HANDLED;
}

void s3c_keygpio_flip_timer(unsigned long data)
{
	unsigned long state;

	state = (readl(S5PV210_GPH0DAT)) & (1 << 6);

	if (state != flip_state) {
		flip_count = 0;
		flip_state = state;
	} else if (flip_count < FLIP_STABLE_COUNT) {
		flip_count++;
	}

	if (flip_count >= FLIP_STABLE_COUNT) {
		if(state)	{		// OPEN
			if (current_flip_status == 0) return;
			current_flip_status = 0;		
		} else	{			// CLOSE
			if (current_flip_status == 1) return;
			current_flip_status = 1;
		}
		queue_work(keygpio_flip_wq, &keygpio_flip_work);
	} else {
		mod_timer(&s3c_keypad->flip_timer, jiffies + msecs_to_jiffies(FLIP_SCAN_INTERVAL));
	}

}


void s3c_keygpio_flip_handler(void)
{
	//spin_lock(&s3c_keypad->lock);

	gp2a_flip_notify(current_flip_status);
	switch_set_state(&switch_flip, current_flip_status);

	if(!current_flip_status)	{		// OPEN
		melfas_touchkey_suspend();

		printk(KERN_DEBUG "s3c_keygpio_flip_handler : SET MAIN : %d\n", current_flip_status);
		input_report_key(s3c_keypad->dev, FLIP_OPEN_KEYCODE, 1);
		input_report_key(s3c_keypad->dev, FLIP_OPEN_KEYCODE, 0);
		input_sync(s3c_keypad->dev);
	} else	{			// CLOSE
		printk(KERN_DEBUG "s3c_keygpio_flip_handler : SET SUB : %d\n", current_flip_status);
		input_report_key(s3c_keypad->dev, FLIP_CLOSE_KEYCODE, 1);
		input_report_key(s3c_keypad->dev, FLIP_CLOSE_KEYCODE, 0);
		input_sync(s3c_keypad->dev);

		melfas_touchkey_resume();
	}

	if (IsLDIEnabled() == false)	{
		if(!current_flip_status)	{
			qt602240_set_working_tsp(1);
			tl2796_set_working_ldi(true);
		} else	{
			qt602240_set_working_tsp(0);
			tl2796_set_working_ldi(false);
		}
	} else	{
		qt602240_switch_suspend();
		cancel_delayed_work_sync(&keygpio_tsp_work);
		queue_delayed_work(keygpio_flip_wq, &keygpio_tsp_work, msecs_to_jiffies(TSP_WQ_DELAY));

		if(!current_flip_status)	{
//			qt602240_switch_sub2main_tsp();
			tl2796_switch_sub2main_ldi();
		} else	{
//			qt602240_switch_main2sub_tsp();
			tl2796_switch_main2sub_ldi();
		}
	}

	//spin_unlock(&s3c_keypad->lock);
}

unsigned int s3c_keygpio_get_flip_status(void)
{
	return (current_flip_status);
}
#endif

void s3c_keygpio_tsp_handler(unsigned long data)
{
	//spin_lock(&s3c_keypad->lock);

#if defined(CONFIG_MACH_S5PC110_PRESTIGE)
	if(!current_flip_status)	{
		qt602240_switch_sub2main_tsp();
	} else	{
		qt602240_switch_main2sub_tsp();
	}
#endif
	//spin_unlock(&s3c_keypad->lock);
}
#endif /* CONFIG_MACH_S5PC110_ARIESCT */

static int s3c_keygpio_isr_setup(void *pdev)
{
	int ret;

#if defined(CONFIG_MACH_S5PC110_PRESTIGE)
	unsigned int key_status;

	//FLIP key
	INIT_WORK(&keygpio_flip_work, s3c_keygpio_flip_handler);
	keygpio_flip_wq = create_singlethread_workqueue("keygpio_flip_wq");

	INIT_DELAYED_WORK(&keygpio_tsp_work, s3c_keygpio_tsp_handler);
//	keygpio_tsp_wq = create_singlethread_workqueue("keygpio_tsp_wq");
	
//	INIT_WORK(&switch_flip_work, s3c_keygpio_switch_flip);
//	switch_flip_wq = create_singlethread_workqueue("switch_flip_wq");

	INIT_WORK(&keygpio_ok_key_work, s3c_keygpio_ok_key_handler);
	INIT_WORK(&keygpio_key_timer_work, s3c_keygpio_key_timer_handler);

	s3c_gpio_setpull(GPIO_OK_KEY, S3C_GPIO_PULL_UP);
	set_irq_type(IRQ_EINT(29), IRQ_TYPE_EDGE_BOTH);
	ret = request_irq(IRQ_EINT(29), s3c_keygpio_ok_isr, IRQF_SAMPLE_RANDOM,
				"key ok", (void *) pdev);
	if (ret) {
		printk("request_irq failed (IRQ_KEYPAD (key ok)) !!!\n");
		ret = -EIO;
		return ret;
	}

	s3c_gpio_setpull(GPIO_HALL_SW, S3C_GPIO_PULL_UP);
	set_irq_type(IRQ_EINT6, IRQ_TYPE_EDGE_BOTH);
	ret = request_irq(IRQ_EINT6, s3c_keygpio_flip_isr, IRQF_SAMPLE_RANDOM,
				"key gpio - flip", (void *) pdev);
	if (ret) {
		printk("request_irq failed (IRQ_KEYPAD (flip)) !!!\n");
		ret = -EIO;
	}

	key_status = (readl(S5PV210_GPH0DAT)) & (1 << 6);
	if(key_status)	{
		current_flip_status = 0;		
		qt602240_switch_sub2main_tsp();
		tl2796_switch_sub2main_ldi();
	} else	{
		current_flip_status = 1;
		qt602240_switch_main2sub_tsp();
		tl2796_switch_main2sub_ldi();
	}
	wq_flip_status = key_status;
	
	switch_set_state(&switch_flip, current_flip_status);
#else
#if 0
	//volume down
	s3c_gpio_setpull(S5PV210_GPH3(1), S3C_GPIO_PULL_NONE);
	set_irq_type(IRQ_EINT(25), IRQ_TYPE_EDGE_BOTH);
	ret = request_irq(IRQ_EINT(25), s3c_keygpio_vol_down_isr, IRQF_SAMPLE_RANDOM, "key vol down", (void *) pdev);
	if (ret) {
		printk("request_irq failed (IRQ_KEYPAD (key vol down)) !!!\n");
		ret = -EIO;
		  return ret;
	}

	//volume up
	s3c_gpio_setpull(S5PV210_GPH3(2), S3C_GPIO_PULL_NONE);
	set_irq_type(IRQ_EINT(26), IRQ_TYPE_EDGE_BOTH);
	ret = request_irq(IRQ_EINT(26), s3c_keygpio_vol_up26_isr, IRQF_SAMPLE_RANDOM, "key vol up(26)", (void *) pdev);
	if (ret) {
			printk("request_irq failed (IRQ_KEYPAD (key vol up(26))) !!!\n");
		ret = -EIO;
		  return ret;
	}
#endif
	//ok key
	s3c_gpio_setpull(S5PV210_GPH3(5), S3C_GPIO_PULL_NONE);
	set_irq_type(IRQ_EINT(29), IRQ_TYPE_EDGE_BOTH);
	ret = request_irq(IRQ_EINT(29), s3c_keygpio_ok_isr, IRQF_DISABLED | IRQF_SAMPLE_RANDOM, "key ok", (void *) pdev);
	if (ret) {
		printk("request_irq failed (IRQ_KEYPAD (key ok)) !!!\n");
		ret = -EIO;
		  return ret;
	}

		
	//ok key
	#if 0
	s3c_gpio_setpull(S5PC11X_GPH3(0), S3C_GPIO_PULL_UP);
	set_irq_type(IRQ_EINT(24), IRQ_TYPE_EDGE_BOTH);
	ret = request_irq(IRQ_EINT(24), s3c_keygpio_ok_isr, IRQF_SAMPLE_RANDOM, "key ok", (void *) pdev);
	if (ret) {
		printk("request_irq failed (IRQ_KEYPAD (key ok)) !!!\n");
		ret = -EIO;
		  return ret;
	}
	 #endif
#endif

	//PWR key
	s3c_gpio_setpull(S5PV210_GPH2(6), S3C_GPIO_PULL_UP);

	set_irq_type(IRQ_EINT(22), IRQ_TYPE_EDGE_BOTH);

	// stk.lim: Add IRQF_DISABLED to eliminate any possible race
	// regarding key status
	ret = request_irq(IRQ_EINT(22), s3c_keygpio_isr, IRQF_DISABLED
			  | IRQF_SAMPLE_RANDOM, "key gpio", (void *)pdev);

        if (ret) {
                printk("request_irq failed (IRQ_KEYPAD (gpio)) !!!\n");
                ret = -EIO;
        }

	return ret;

}


static ssize_t keyshort_test(struct device *dev, struct device_attribute *attr, char *buf)
{
	int count;
	
	if(!gpio_get_value(GPIO_KBR1) || !gpio_get_value(GPIO_KBR2) || !gpio_get_value(GPIO_nPOWER)  || !gpio_get_value(S5PV210_GPH3(5)))
	{
		count = sprintf(buf,"PRESS\n");
              printk("keyshort_test: PRESS\n");
	}
	else
	{
		count = sprintf(buf,"RELEASE\n");
              printk("keyshort_test: RELEASE\n");
	}	

	return count;
}

#if defined(CONFIG_MACH_S5PC110_PRESTIGE)
static ssize_t key_bl_on_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	s3c_gpio_setpin(GPIO_KEY_BL_EN, 1);

	return sprintf(buf, "%s\n", "KEY BL ON");
}
static ssize_t key_bl_off_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	s3c_gpio_setpin(GPIO_KEY_BL_EN, 0);

	return sprintf(buf, "%s\n", "KEY BL OFF");
}

int is_keypad_backlight_night = 0;
EXPORT_SYMBOL(is_keypad_backlight_night);
extern int working_lcd;
static ssize_t key_bl_night_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	printk("[KEY] %s Check night..\n", __FUNCTION__);
	
	if(sscanf(buf, "%d\n", &is_keypad_backlight_night) == 1)
	{
		printk("[KEY] is_keypad_backlight_night : %d \n", is_keypad_backlight_night);	
	}
	else
	{
		printk("[KEY] check_night Error\n");
	}

	return size;
}

// keypad backlight control 0:off, 1:on
static ssize_t key_bl_onoff_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	unsigned int key_onoff = 0;
	
#if !defined(CONFIG_MACH_S5PC110_PRESTIGE)	
	printk("[KEY] %s BL control..\n", __FUNCTION__);
#endif

	if(sscanf(buf, "%d\n", &key_onoff) == 1)
	{
		if (working_lcd == 1) // MAIN_LCD:0, SUB_LCD:1
			key_onoff=0;
		s3c_gpio_setpin(GPIO_KEY_BL_EN, key_onoff);
#if !defined(CONFIG_MACH_S5PC110_PRESTIGE)			
		printk("[KEY] %s key_onoff : %d, working_lcd : %d\n", __FUNCTION__, key_onoff, working_lcd);
#endif
	}
	else
	{
		printk("[KEY] key_bl_control Error\n");
	}

	return size;
}

static DEVICE_ATTR(key_bl_on, 0664, key_bl_on_show, NULL);
static DEVICE_ATTR(key_bl_off, 0664, key_bl_off_show, NULL);
static DEVICE_ATTR(key_bl_onoff, 0664, NULL, key_bl_onoff_store);
static DEVICE_ATTR(key_bl_night, 0664, NULL, key_bl_night_store);
#endif  // CONFIG_MACH_S5PC110_PRESTIGE

static DEVICE_ATTR(key_pressed, 0664, keyshort_test, NULL);

#ifdef FLASH_MOVIE_MODE

#define CAM_FLASH_EN                    S5PV210_GPJ1(2)
#define CAM_FLASH_EN_SET                S5PV210_GPJ1(0)
#define CAM_FLASH_EN_STR                "GPJ12"
#define CAM_FLASH_EN_SET_STR    	"GPJ10"


#define OFF             0
#define ON              1

#define MOVIE_MODE_CURRENT                      17
#define FLASH_SAFETY_TIMER                      18
#define MOVIE_MODE_CONFIG                       19
#define FLASH_TO_MOVIE_RATIO                    20

#define MOVIE_MODE_CURRENT_71                   4
#define MOVIE_MODE_CURRENT_63                   5
#define MOVIE_MODE_CURRENT_56                   6

#define OFFSET_CAM_FLASH                (0x1 << 9)
/*
 * timed_output for movie mode flash
 */
static struct hrtimer timer;

static int max_timeout = 5000;
static int flash_value = 0;

/*
 * IFLOUTA = IFLOUTB = 81K / 160K * A = 500mA
 * T = 7.98s / uF * Ct(uF) = 7.98s / uF * 0.1uF = 0.798s 
 */
//extern void s3c_bat_set_compensation_for_drv(int mode,int offset);

static void aat1271a_flash_write(int addr, int data)
{
        int i;

        for (i = 0; i < addr; i++) {
                gpio_set_value(CAM_FLASH_EN_SET, GPIO_LEVEL_LOW);
                udelay(10);
                gpio_set_value(CAM_FLASH_EN_SET, GPIO_LEVEL_HIGH);
                udelay(50);
        }

        udelay(500);

        for (i = 0; i < data; i++) {
                gpio_set_value(CAM_FLASH_EN_SET, GPIO_LEVEL_LOW);
                udelay(10);
                gpio_set_value(CAM_FLASH_EN_SET, GPIO_LEVEL_HIGH);
                udelay(50);
        }

        udelay(500);
}

void aat1271a_falsh_camera_control(int ctrl)
{
        if (ctrl) {
                /* Movie Mode Off */
                gpio_set_value(CAM_FLASH_EN_SET, GPIO_LEVEL_LOW);
                gpio_set_value(CAM_FLASH_EN, GPIO_LEVEL_LOW);
                udelay(10);
                /* Falsh Mode On */
                gpio_set_value(CAM_FLASH_EN, GPIO_LEVEL_HIGH);
                //s3c_bat_set_compensation_for_drv(1,OFFSET_CAM_FLASH);

        }
        else {
                /* Movie Mode Off */
                gpio_set_value(CAM_FLASH_EN_SET, GPIO_LEVEL_LOW);
                /* Falsh Mode Off */
                gpio_set_value(CAM_FLASH_EN, GPIO_LEVEL_LOW);
                //s3c_bat_set_compensation_for_drv(0,OFFSET_CAM_FLASH);
        }
}

/*
 * IMOVIEMODE = IFLOUTA / 7.3 = 500mA / 7.3 = 68mA
 * 45mA / 68mA = 0.66 = 0.7 = 70 %
 */

void aat1271a_falsh_movie_control(int ctrl)
{
        if (ctrl) {

                /* Falsh Mode Off */
                gpio_set_value(CAM_FLASH_EN, GPIO_LEVEL_LOW);
                /* Movie Mode Current Setting & On */
                aat1271a_flash_write(MOVIE_MODE_CURRENT, MOVIE_MODE_CURRENT_63);
                //s3c_bat_set_compensation_for_drv(1,OFFSET_CAM_FLASH);
        } 
        else {
                /* Falsh Mode Off */
                gpio_set_value(CAM_FLASH_EN, GPIO_LEVEL_LOW);
                /* Movie Mode Off */
                gpio_set_value(CAM_FLASH_EN_SET, GPIO_LEVEL_LOW);
                //s3c_bat_set_compensation_for_drv(0,OFFSET_CAM_FLASH);
        }
}

static enum hrtimer_restart aat1271_flash_timer_func(struct hrtimer *timer)
{
        /* Movie Mode Off */
        gpio_set_value(CAM_FLASH_EN_SET, GPIO_LEVEL_LOW);
        /* Falsh Mode Off */
        gpio_set_value(CAM_FLASH_EN, GPIO_LEVEL_LOW);

        return HRTIMER_NORESTART;
}

static int get_time_for_flash(struct timed_output_dev *dev)
{
        int remaining;

        if (hrtimer_active(&timer)) {
                ktime_t r = hrtimer_get_remaining(&timer);
                remaining = r.tv.sec * 1000 + r.tv.nsec / 1000000;
        } else
                remaining = 0;

        if (flash_value ==-1)
                remaining = -1;

        return remaining;

}

static void enable_flash_from_user(struct timed_output_dev *dev,int value)
{
	int err;

        if (value > 0) {
                if (value < 780) {       /* Flash Mode */
					gpio_direction_output(CAM_FLASH_EN, 0);
                    gpio_direction_output(CAM_FLASH_EN_SET, 0);

                    aat1271a_falsh_camera_control(ON);
                    printk("flash test: FLASH MODE\n");
               }
                else {   /* Movie Mode */
                	gpio_direction_output(CAM_FLASH_EN, 0);
                    gpio_direction_output(CAM_FLASH_EN_SET, 0);

                    aat1271a_falsh_movie_control(ON);
                    printk("flash test: MOVIE MODE\n");
               }
        } else if (value == 0) {
        	gpio_direction_output(CAM_FLASH_EN, 0);
            gpio_direction_output(CAM_FLASH_EN_SET, 0);

            aat1271a_falsh_camera_control(OFF);
            aat1271a_falsh_movie_control(OFF);
            printk("flash test: Flash OFF\n");
       }

}

static struct timed_output_dev timed_output_flash= {
        .name     = "flash",
        .get_time = get_time_for_flash,
        .enable   = enable_flash_from_user,
};

#endif

static int __init s3c_keypad_probe(struct platform_device *pdev)
{
	struct resource *res, *keypad_mem, *keypad_irq;
	struct input_dev *input_dev;
	int ret, size;
	int key, code;

#if defined(CONFIG_MACH_S5PC110_PRESTIGE)
	int *keypad_keycode_ptr = NULL;

	wake_lock_init(&lcd_wake_lock, WAKE_LOCK_SUSPEND, "lcd_present");
#endif

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		dev_err(&pdev->dev,"no memory resource specified\n");
		return -ENOENT;
	}

	size = (res->end - res->start) + 1;

	keypad_mem = request_mem_region(res->start, size, pdev->name);
	if (keypad_mem == NULL) {
		dev_err(&pdev->dev, "failed to get memory region\n");
		ret = -ENOENT;
		goto err_req;
	}

	key_base = ioremap(res->start, size);
	if (key_base == NULL) {
		printk(KERN_ERR "Failed to remap register block\n");
		ret = -ENOMEM;
		goto err_map;
	}

	keypad_clock = clk_get(&pdev->dev, "keypad");
	if (IS_ERR(keypad_clock)) {
		dev_err(&pdev->dev, "failed to find keypad clock source\n");
		ret = PTR_ERR(keypad_clock);
		goto err_clk;
	}

	clk_enable(keypad_clock);
	
	s3c_keypad = kzalloc(sizeof(struct s3c_keypad), GFP_KERNEL);
	input_dev = input_allocate_device();

	if (!s3c_keypad || !input_dev) {
		ret = -ENOMEM;
		goto err_alloc;
	}

	platform_set_drvdata(pdev, s3c_keypad);
	s3c_keypad->dev = input_dev;
	
	writel(KEYIFCON_INIT, key_base+S3C_KEYIFCON);
	writel(KEYIFFC_DIV, key_base+S3C_KEYIFFC);

	/* Set GPIO Port for keypad mode and pull-up disable*/
	s3c_setup_keypad_cfg_gpio(KEYPAD_ROWS, KEYPAD_COLUMNS);

	writel(KEYIFCOL_CLEAR, key_base+S3C_KEYIFCOL);

	/* create and register the input driver */
	set_bit(EV_KEY, input_dev->evbit);
	/*Commenting the generation of repeat events*/
	//set_bit(EV_REP, input_dev->evbit);
	s3c_keypad->nr_rows = KEYPAD_ROWS;
	s3c_keypad->no_cols = KEYPAD_COLUMNS;
	s3c_keypad->total_keys = MAX_KEYPAD_NR;

#if defined(CONFIG_MACH_S5PC110_PRESTIGE)
	//spin_lock_init(&s3c_keypad->lock);

	init_timer(&s3c_keypad->flip_timer);
	s3c_keypad->flip_timer.function = s3c_keygpio_flip_timer;
	s3c_keypad->flip_timer.data = (unsigned long)s3c_keypad;

	if(W899_HWREV >= 0x05)
		keypad_keycode_ptr = keypad_keycode_Rev05;
	else
	keypad_keycode_ptr = keypad_keycode;

	for(key = 0; key < s3c_keypad->total_keys; key++){
		code = s3c_keypad->keycodes[key] = keypad_keycode_ptr[key]; //W899_Rev05   keypad_keycode[key];
		if(code<=0)
			continue;
		set_bit(code & KEY_MAX, input_dev->keybit);
	}
#else
	for(key = 0; key < s3c_keypad->total_keys; key++){
	code = s3c_keypad->keycodes[key] = keypad_keycode[key];
		if(code<=0)
			continue;
		set_bit(code & KEY_MAX, input_dev->keybit);
	}
#endif


	//printk("%s, keypad row number is %d, column is %d",__FUNCTION__, s3c_keypad->nr_rows, s3c_keypad->no_cols);

#if defined(CONFIG_MACH_S5PC110_PRESTIGE)
	set_bit(FLIP_OPEN_KEYCODE & KEY_MAX, input_dev->keybit);
	set_bit(FLIP_CLOSE_KEYCODE & KEY_MAX, input_dev->keybit);	
	set_bit(OK_KEYCODE & KEY_MAX, input_dev->keybit);
	set_bit(END_KEYCODE & KEY_MAX, input_dev->keybit);
	set_bit(END_APPS_KEYCODE & KEY_MAX, input_dev->keybit);
#else
	set_bit(26 & KEY_MAX, input_dev->keybit);
#endif
	input_dev->name = DEVICE_NAME;
	input_dev->phys = "s3c-keypad/input0";
	
	input_dev->id.bustype = BUS_HOST;
	input_dev->id.vendor = 0x0001;
	input_dev->id.product = 0x0001;
	input_dev->id.version = 0x0001;

#if defined(CONFIG_MACH_S5PC110_PRESTIGE)
	input_dev->keycode = keypad_keycode_ptr; //W899 Rev05  keypad_keycode;
#else
input_dev->keycode = keypad_keycode;
#endif	

	ret = input_register_device(input_dev);
	if (ret) {
		printk("Unable to register s3c-keypad input device!!!\n");
		goto out;
	}

	/* Scan timer init */
	init_timer(&keypad_timer);
	keypad_timer.function = keypad_timer_handler;
	keypad_timer.data = (unsigned long)s3c_keypad;

	/* For IRQ_KEYPAD */
	keypad_irq = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (keypad_irq == NULL) {
		dev_err(&pdev->dev, "no irq resource specified\n");
		ret = -ENOENT;
		goto err_irq;
	}

	ret = switch_dev_register(&switch_flip);
	if (ret < 0) 
	{
		printk(KERN_ERR "FLIP: Failed to register switch device\n");
		goto err_flip;
	}
	s3c_keygpio_isr_setup((void *)s3c_keypad);
#if !defined(CONFIG_MACH_S5PC110_PRESTIGE)
	switch_set_state(&switch_flip, 1);	
#endif

    ret = request_irq(keypad_irq->start, s3c_keypad_isr, IRQF_SAMPLE_RANDOM, DEVICE_NAME, (void *) pdev);
	if (ret) {
		printk("request_irq failed (IRQ_KEYPAD) !!!\n");
		ret = -EIO;
		goto err_irq;
	}

	keypad_timer.expires = jiffies + (HZ/10);

	if (is_timer_on == FALSE) {
		add_timer(&keypad_timer);
		is_timer_on = TRUE;
	} 
	else {
			mod_timer(&keypad_timer,keypad_timer.expires);
	}
	
	printk( DEVICE_NAME " Initialized\n");

	if (device_create_file(&(pdev->dev), &dev_attr_key_pressed) < 0) {
		printk("%s s3c_keypad_probe\n",__FUNCTION__);
		pr_err("Failed to create device file(%s)!\n", dev_attr_key_pressed.attr.name);
	}

#if defined(CONFIG_MACH_S5PC110_PRESTIGE)
	if (device_create_file(&(pdev->dev), &dev_attr_key_bl_on) < 0) {
		printk("%s s3c_keypad_probe\n",__FUNCTION__);
		pr_err("Failed to create device file(%s)!\n", dev_attr_key_bl_on.attr.name);
	}

	if (device_create_file(&(pdev->dev), &dev_attr_key_bl_off) < 0) {
		printk("%s s3c_keypad_probe\n",__FUNCTION__);
		pr_err("Failed to create device file(%s)!\n", dev_attr_key_bl_off.attr.name);
	}
	
	if (device_create_file(&(pdev->dev), &dev_attr_key_bl_onoff) < 0) {
		printk("%s s3c_keypad_probe\n",__FUNCTION__);
		pr_err("Failed to create device file(%s)!\n", dev_attr_key_bl_onoff.attr.name);
	}

	if (device_create_file(&(pdev->dev), &dev_attr_key_bl_night) < 0) {
		printk("%s s3c_keypad_probe\n",__FUNCTION__);
		pr_err("Failed to create device file(%s)!\n", dev_attr_key_bl_night.attr.name);
	}
#endif  // CONFIG_MACH_S5PC110_PRESTIGE

#ifdef FLASH_MOVIE_MODE
        /* hrtimer settings */
        hrtimer_init(&timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
        timer.function = aat1271_flash_timer_func;

        timed_output_dev_register(&timed_output_flash);

#endif

	return 0;

err_flip:
	switch_dev_unregister(&switch_flip);

out:
	free_irq(keypad_irq->start, input_dev);
	free_irq(keypad_irq->end, input_dev);

err_irq:
	input_free_device(input_dev);
	kfree(s3c_keypad);
	
err_alloc:
	clk_disable(keypad_clock);
	clk_put(keypad_clock);

err_clk:
	iounmap(key_base);

err_map:
	release_resource(keypad_mem);
	kfree(keypad_mem);

err_req:

	return ret;
}

static int s3c_keypad_remove(struct platform_device *pdev)
{
	struct input_dev *input_dev = platform_get_drvdata(pdev);
	writel(KEYIFCON_CLEAR, key_base+S3C_KEYIFCON);

	if(keypad_clock) {
		clk_disable(keypad_clock);
		clk_put(keypad_clock);
		keypad_clock = NULL;
	}

#if defined(CONFIG_MACH_S5PC110_PRESTIGE)
	del_timer_sync(&s3c_keypad->flip_timer);
	switch_dev_unregister(&switch_flip);
#endif
	
	input_unregister_device(input_dev);
	iounmap(key_base);
	kfree(pdev->dev.platform_data);
	free_irq(IRQ_KEYPAD, (void *) pdev);

	del_timer(&keypad_timer);	
	printk(DEVICE_NAME " Removed.\n");
		return 0;
}


#if defined(CONFIG_PM)
#include <plat/pm.h>

static struct sleep_save s3c_keypad_save[] = {
	SAVE_ITEM(KEYPAD_ROW_GPIOCON),
	SAVE_ITEM(KEYPAD_COL_GPIOCON),
	SAVE_ITEM(KEYPAD_ROW_GPIOPUD),
	SAVE_ITEM(KEYPAD_COL_GPIOPUD),
};

static unsigned int keyifcon, keyiffc;
static int s3c_keypad_suspend(struct platform_device *dev, pm_message_t state)
{
	keyifcon = readl(key_base+S3C_KEYIFCON);
	keyiffc = readl(key_base+S3C_KEYIFFC);

	s3c_pm_do_save(s3c_keypad_save, ARRAY_SIZE(s3c_keypad_save));
	
	//writel(~(0xfffffff), KEYPAD_ROW_GPIOCON);
	//writel(~(0xfffffff), KEYPAD_COL_GPIOCON);

	disable_irq(IRQ_KEYPAD);

	clk_disable(keypad_clock);

	in_sleep = 1;

	return 0;
}


static int s3c_keypad_resume(struct platform_device *dev)
{
	struct s3c_keypad          *s3c_keypad = (struct s3c_keypad *) platform_get_drvdata(dev);
      struct input_dev           *iDev = s3c_keypad->dev;
	unsigned int key_temp_data=0;
	
	printk(KERN_DEBUG "++++ %s\n", __FUNCTION__ );

	clk_enable(keypad_clock);

	writel(KEYIFCON_INIT, key_base+S3C_KEYIFCON);
	writel(keyiffc, key_base+S3C_KEYIFFC);
	writel(KEYIFCOL_CLEAR, key_base+S3C_KEYIFCOL);

#if 0
	key_temp_data = readl(key_base+S3C_KEYIFROW) & 0x01;
	if (!key_temp_data){
		input_report_key(iDev, 50, 1);
		printk("key data is %d \n", key_temp_data);		
		input_report_key(iDev, 50, 0);
		}
	else {
		/*send some event to android to start the full resume*/
		input_report_key(iDev, KEYCODE_UNKNOWN, 1);//ENDCALL up event
		udelay(5);
		input_report_key(iDev, KEYCODE_UNKNOWN, 0);//ENDCALL down event
		}

	//printk("H3C %x H2C %x \n",readl(S5PC11X_GPH3CON),readl(S5PC11X_GPH2CON));
#endif
	s3c_pm_do_restore(s3c_keypad_save, ARRAY_SIZE(s3c_keypad_save));

	enable_irq(IRQ_KEYPAD);
	printk(KERN_DEBUG "---- %s\n", __FUNCTION__ );
	return 0;
}
#else
#define s3c_keypad_suspend NULL
#define s3c_keypad_resume  NULL
#endif

static struct platform_driver s3c_keypad_driver = {
	.probe		= s3c_keypad_probe,
	.remove		= s3c_keypad_remove,
	.suspend	= s3c_keypad_suspend,
	.resume		= s3c_keypad_resume,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= "s3c-keypad",
	},
};

static int __init s3c_keypad_init(void)
{
	int ret;
	
	ret = platform_driver_register(&s3c_keypad_driver);

	if(!ret)
	   printk(KERN_INFO "S3C Keypad Driver\n");

	return ret;
}

static void __exit s3c_keypad_exit(void)
{
	platform_driver_unregister(&s3c_keypad_driver);
}

module_init(s3c_keypad_init);
module_exit(s3c_keypad_exit);

MODULE_AUTHOR("Samsung");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("KeyPad interface for Samsung S3C");
