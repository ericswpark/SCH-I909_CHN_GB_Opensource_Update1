/* linux/arch/arm/plat-s3c/pm.c
 *
 * Copyright 2008 Openmoko, Inc.
 * Copyright 2004-2008 Simtec Electronics
 *	Ben Dooks <ben@simtec.co.uk>
 *	http://armlinux.simtec.co.uk/
 *
 * S3C common power management (suspend to ram) support.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/init.h>
#include <linux/suspend.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/serial_core.h>
#include <linux/io.h>

#include <asm/cacheflush.h>
#include <mach/hardware.h>
#include <mach/map.h>

#include <plat/regs-serial.h>
#include <mach/regs-clock.h>
#include <mach/regs-irq.h>
#include <asm/fiq_glue.h>
#include <asm/irq.h>

#include <plat/pm.h>
#include <plat/irq-eint-group.h>
#include <mach/pm-core.h>

#if 1 //from_FROYO
#include <plat/map-base.h>
#include <mach/regs-gpio.h>

//#define DBG(fmt...) printk(fmt)
#define DBG(fmt...)
#endif

/* for external use */

unsigned long s3c_pm_flags;

/* ---------------------------------------------- */
extern unsigned int pm_debug_scratchpad;
#include <linux/slab.h>
#include <linux/debugfs.h>
#include <linux/uaccess.h>
#include <linux/module.h>

#define PMSTATS_MAGIC "*PM*DEBUG*STATS*"

struct pmstats {
	char magic[16];
	unsigned sleep_count;
	unsigned wake_count;
	unsigned sleep_freq;
	unsigned wake_freq;
};

static struct pmstats *pmstats;
static struct pmstats *pmstats_last;

#if 1 //pending_int
unsigned int s5pc11x_pm_wakeup_eint0_pend = 0;
unsigned int s5pc11x_pm_wakeup_eint1_pend = 0;
unsigned int s5pc11x_pm_wakeup_eint2_pend = 0;
unsigned int s5pc11x_pm_wakeup_eint3_pend = 0;
#endif

static ssize_t pmstats_read(struct file *file, char __user *buf,
			    size_t len, loff_t *offset)
{
	if (*offset != 0)
		return 0;
	if (len > 4096)
		len = 4096;

	if (copy_to_user(buf, file->private_data, len))
		return -EFAULT;

	*offset += len;
	return len;
}

static int pmstats_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static const struct file_operations pmstats_ops = {
	.owner = THIS_MODULE,
	.read = pmstats_read,
	.open = pmstats_open,
};

void __init pmstats_init(void)
{
	pr_info("pmstats at %08x\n", pm_debug_scratchpad);
	if (pm_debug_scratchpad)
		pmstats = ioremap(pm_debug_scratchpad, 4096);
	else
		pmstats = kzalloc(4096, GFP_ATOMIC);

	if (!memcmp(pmstats->magic, PMSTATS_MAGIC, 16)) {
		pmstats_last = kzalloc(4096, GFP_ATOMIC);
		if (pmstats_last)
			memcpy(pmstats_last, pmstats, 4096);
	}

	memset(pmstats, 0, 4096);
	memcpy(pmstats->magic, PMSTATS_MAGIC, 16);

	debugfs_create_file("pmstats", 0444, NULL, pmstats, &pmstats_ops);
	if (pmstats_last)
		debugfs_create_file("pmstats_last", 0444, NULL, pmstats_last, &pmstats_ops);
}
/* ---------------------------------------------- */

/* Debug code:
 *
 * This code supports debug output to the low level UARTs for use on
 * resume before the console layer is available.
*/

#ifdef CONFIG_SAMSUNG_PM_DEBUG
extern void printascii(const char *);

void s3c_pm_dbg(const char *fmt, ...)
{
	va_list va;
	char buff[256];

	va_start(va, fmt);
	vsprintf(buff, fmt, va);
	va_end(va);

	printascii(buff);
}

static inline void s3c_pm_debug_init(void)
{
	/* restart uart clocks so we can use them to output */
	s3c_pm_debug_init_uart();
}

#else
#define s3c_pm_debug_init() do { } while(0)

#endif /* CONFIG_SAMSUNG_PM_DEBUG */

/* Save the UART configurations if we are configured for debug. */

unsigned char pm_uart_udivslot;

#define ENABLE_UART_SAVE_RESTORE
#ifdef ENABLE_UART_SAVE_RESTORE//CONFIG_SAMSUNG_PM_DEBUG

struct pm_uart_save uart_save[CONFIG_SERIAL_SAMSUNG_UARTS];

static void s3c_pm_save_uart(unsigned int uart, struct pm_uart_save *save)
{
	void __iomem *regs = S3C_VA_UARTx(uart);

	save->ulcon = __raw_readl(regs + S3C2410_ULCON);
	save->ucon = __raw_readl(regs + S3C2410_UCON);
	save->ufcon = __raw_readl(regs + S3C2410_UFCON);
	save->umcon = __raw_readl(regs + S3C2410_UMCON);
	save->ubrdiv = __raw_readl(regs + S3C2410_UBRDIV);

	if (pm_uart_udivslot)
		save->udivslot = __raw_readl(regs + S3C2443_DIVSLOT);

	S3C_PMDBG("UART[%d]: ULCON=%04x, UCON=%04x, UFCON=%04x, UBRDIV=%04x\n",
		  uart, save->ulcon, save->ucon, save->ufcon, save->ubrdiv);
}

static void s3c_pm_save_uarts(void)
{
	struct pm_uart_save *save = uart_save;
	unsigned int uart;

	for (uart = 0; uart < CONFIG_SERIAL_SAMSUNG_UARTS; uart++, save++)
		s3c_pm_save_uart(uart, save);
}

static void s3c_pm_restore_uart(unsigned int uart, struct pm_uart_save *save)
{
	void __iomem *regs = S3C_VA_UARTx(uart);

	s3c_pm_arch_update_uart(regs, save);

	__raw_writel(save->ulcon, regs + S3C2410_ULCON);
	__raw_writel(save->ucon,  regs + S3C2410_UCON);
	__raw_writel(save->ufcon, regs + S3C2410_UFCON);
	__raw_writel(save->umcon, regs + S3C2410_UMCON);
	__raw_writel(save->ubrdiv, regs + S3C2410_UBRDIV);

	if (pm_uart_udivslot)
		__raw_writel(save->udivslot, regs + S3C2443_DIVSLOT);
}

static void s3c_pm_restore_uarts(void)
{
	struct pm_uart_save *save = uart_save;
	unsigned int uart;

	for (uart = 0; uart < CONFIG_SERIAL_SAMSUNG_UARTS; uart++, save++)
		s3c_pm_restore_uart(uart, save);
}
#else
static void s3c_pm_save_uarts(void) { }
static void s3c_pm_restore_uarts(void) { }
#endif

/* The IRQ ext-int code goes here, it is too small to currently bother
 * with its own file. */

unsigned long s3c_irqwake_intmask	= 0xffffffffL;
unsigned long s3c_irqwake_eintmask	= 0xffffffffL;

int s3c_irqext_wake(unsigned int irqno, unsigned int state)
{
	unsigned long bit = 1L << IRQ_EINT_BIT(irqno);

	if (!(s3c_irqwake_eintallow & bit))
		return -ENOENT;

	printk(KERN_INFO "wake %s for irq %d\n",
	       state ? "enabled" : "disabled", irqno);

	if (!state)
		s3c_irqwake_eintmask |= bit;
	else
		s3c_irqwake_eintmask &= ~bit;

	return 0;
}

#if 1 //from_FROYO

#define eint_offset(irq)                (irq)
#define eint_irq_to_bit(irq)            (1 << (eint_offset(irq) & 0x7))
#define eint_conf_reg(irq)              ((eint_offset(irq)) >> 3)
#define eint_filt_reg(irq)              ((eint_offset(irq)) >> 2)
#define eint_mask_reg(irq)              ((eint_offset(irq)) >> 3)
#define eint_pend_reg(irq)              ((eint_offset(irq)) >> 3)

#define S5PV210_GPH0CON			(S5PV210_GPH0_BASE + 0x00)
#define S5PV210_GPH0DAT			(S5PV210_GPH0_BASE + 0x04)
#define S5PV210_GPH0PUD			(S5PV210_GPH0_BASE + 0x08)
#define S5PV210_GPH0DRV			(S5PV210_GPH0_BASE + 0x0c)
#define S5PV210_GPH0CONPDN		(S5PV210_GPH0_BASE + 0x10)
#define S5PV210_GPH0PUDPDN		(S5PV210_GPH0_BASE + 0x14)

#define S5PV210_GPH1CON			(S5PV210_GPH1_BASE + 0x00)
#define S5PV210_GPH1DAT			(S5PV210_GPH1_BASE + 0x04)
#define S5PV210_GPH1PUD			(S5PV210_GPH1_BASE + 0x08)
#define S5PV210_GPH1DRV			(S5PV210_GPH1_BASE + 0x0c)
#define S5PV210_GPH1CONPDN		(S5PV210_GPH1_BASE + 0x10)
#define S5PV210_GPH1PUDPDN		(S5PV210_GPH1_BASE + 0x14)

#define S5PV210_GPH2CON			(S5PV210_GPH2_BASE + 0x00)
#define S5PV210_GPH2DAT			(S5PV210_GPH2_BASE + 0x04)
#define S5PV210_GPH2PUD			(S5PV210_GPH2_BASE + 0x08)
#define S5PV210_GPH2DRV			(S5PV210_GPH2_BASE + 0x0c)
#define S5PV210_GPH2CONPDN		(S5PV210_GPH2_BASE + 0x10)
#define S5PV210_GPH2PUDPDN		(S5PV210_GPH2_BASE + 0x14)

#define S5PV210_GPH3CON			(S5PV210_GPH3_BASE + 0x00)
#define S5PV210_GPH3DAT			(S5PV210_GPH3_BASE + 0x04)
#define S5PV210_GPH3PUD			(S5PV210_GPH3_BASE + 0x08)
#define S5PV210_GPH3DRV			(S5PV210_GPH3_BASE + 0x0c)
#define S5PV210_GPH3CONPDN		(S5PV210_GPH3_BASE + 0x10)
#define S5PV210_GPH3PUDPDN		(S5PV210_GPH3_BASE + 0x14)

#define S5PV210_MP04CON			(S5PV210_MP04_BASE + 0x00)
#define S5PV210_MP04DAT			(S5PV210_MP04_BASE + 0x04)
#define S5PV210_MP04PUD			(S5PV210_MP04_BASE + 0x08)
#define S5PV210_MP04DRV			(S5PV210_MP04_BASE + 0x0c)
#define S5PV210_MP04CONPDN		(S5PV210_MP04_BASE + 0x10)
#define S5PV210_MP04PUDPDN		(S5PV210_MP04_BASE + 0x14)


#define S5PV210_GPIOREG(x)		(S5P_VA_GPIO + (x))

#define S5PV210_EINT0CON		S5PV210_GPIOREG(0xE00)		/* EINT0  ~ EINT7  */
#define S5PV210_EINT1CON		S5PV210_GPIOREG(0xE04)		/* EINT8  ~ EINT15 */
#define S5PV210_EINT2CON		S5PV210_GPIOREG(0xE08)		/* EINT16 ~ EINT23 */
#define S5PV210_EINT3CON		S5PV210_GPIOREG(0xE0C)		/* EINT24 ~ EINT31 */
#define S5PV210_EINTCON(x)		(S5PV210_EINT0CON+x*0x4)	/* EINT0  ~ EINT31  */

#define S5PV210_EINT0FLTCON0		S5PV210_GPIOREG(0xE80)		/* EINT0  ~ EINT3  */
#define S5PV210_EINT0FLTCON1		S5PV210_GPIOREG(0xE84)
#define S5PV210_EINT1FLTCON0		S5PV210_GPIOREG(0xE88)		/* EINT8 ~  EINT11 */
#define S5PV210_EINT1FLTCON1		S5PV210_GPIOREG(0xE8C)
#define S5PV210_EINT2FLTCON0		S5PV210_GPIOREG(0xE90)
#define S5PV210_EINT2FLTCON1		S5PV210_GPIOREG(0xE94)
#define S5PV210_EINT3FLTCON0		S5PV210_GPIOREG(0xE98)
#define S5PV210_EINT3FLTCON1		S5PV210_GPIOREG(0xE9C)
#define S5PV210_EINTFLTCON(x)		(S5PV210_EINT0FLTCON0+x*0x4)	/* EINT0  ~ EINT31 */

#define S5PV210_EINT0MASK		S5PV210_GPIOREG(0xF00)		/* EINT30[0] ~  EINT30[7]  */
#define S5PV210_EINT1MASK		S5PV210_GPIOREG(0xF04)		/* EINT31[0] ~  EINT31[7] */
#define S5PV210_EINT2MASK		S5PV210_GPIOREG(0xF08)		/* EINT32[0] ~  EINT32[7] */
#define S5PV210_EINT3MASK		S5PV210_GPIOREG(0xF0C)		/* EINT33[0] ~  EINT33[7] */
#define S5PV210_EINTMASK(x)		(S5PV210_EINT0MASK+x*0x4)	/* EINT0 ~  EINT31  */

#define S5PV210_EINT0PEND		S5PV210_GPIOREG(0xF40)		/* EINT30[0] ~  EINT30[7]  */
#define S5PV210_EINT1PEND		S5PV210_GPIOREG(0xF44)		/* EINT31[0] ~  EINT31[7] */
#define S5PV210_EINT2PEND		S5PV210_GPIOREG(0xF48)		/* EINT32[0] ~  EINT32[7] */
#define S5PV210_EINT3PEND		S5PV210_GPIOREG(0xF4C)		/* EINT33[0] ~  EINT33[7] */
#define S5PV210_EINTPEND(x)		(S5PV210_EINT0PEND+x*04)	/* EINT0 ~  EINT31  */

// intr_mode 0x2=>falling edge, 0x3=>rising dege, 0x4=>Both edge
static void s3c_pm_set_eint(unsigned int irq, unsigned int intr_mode)
{
	int offs = (irq);
	int shift;
	u32 ctrl, mask, tmp;
	//u32 newvalue = 0x2; // Falling edge

	shift = (offs & 0x7) * 4;
	if((0 <= offs) && (offs < 8)){
		tmp = readl(S5PV210_GPH0CON);
		tmp |= (0xf << shift);
		writel(tmp , S5PV210_GPH0CON);
		/*pull up disable*/
	}
	else if((8 <= offs) && (offs < 16)){
		tmp = readl(S5PV210_GPH1CON);
		tmp |= (0xf << shift);
		writel(tmp , S5PV210_GPH1CON);
	}
	else if((16 <= offs) && (offs < 24)){
		tmp = readl(S5PV210_GPH2CON);
		tmp |= (0xf << shift);
		writel(tmp , S5PV210_GPH2CON);
	}
	else if((24 <= offs) && (offs < 32)){
		tmp = readl(S5PV210_GPH3CON);
		tmp |= (0xf << shift);
		writel(tmp , S5PV210_GPH3CON);
	}
	else{
		printk(KERN_ERR "No such irq number %d", offs);
		return;
	}

	/*special handling for keypad eint*/
#if defined(CONFIG_MACH_S5PC110_PRESTIGE)
	if( (24 <= irq) && (irq <= 31))
#else
	if( (24 <= irq) && (irq <= 27))
#endif
	{// disable the pull up
		tmp = readl(S5PV210_GPH3PUD);
		tmp &= ~(0x3 << ((offs & 0x7) * 2));	
		writel(tmp, S5PV210_GPH3PUD);
		DBG("S5PV210_GPH3PUD = %x\n",readl(S5PV210_GPH3PUD));
	}
	

	/*Set irq type*/
	mask = 0x7 << shift;
	ctrl = readl(S5PV210_EINTCON(eint_conf_reg(irq)));
	ctrl &= ~mask;
	//ctrl |= newvalue << shift;
	ctrl |= intr_mode << shift;

	writel(ctrl, S5PV210_EINTCON(eint_conf_reg(irq)));
	/*clear mask*/
	mask = readl(S5PV210_EINTMASK(eint_mask_reg(irq)));
	mask &= ~(eint_irq_to_bit(irq));
	writel(mask, S5PV210_EINTMASK(eint_mask_reg(irq)));

	/*clear pending*/
	mask = readl(S5PV210_EINTPEND(eint_pend_reg(irq)));
	mask &= (eint_irq_to_bit(irq));
	writel(mask, S5PV210_EINTPEND(eint_pend_reg(irq)));
	
	/*Enable wake up mask*/
	tmp = readl(S5P_EINT_WAKEUP_MASK);
	tmp &= ~(1 << (irq));
	writel(tmp , S5P_EINT_WAKEUP_MASK);

	DBG("S5PV210_EINTCON = %x\n",readl(S5PV210_EINTCON(eint_conf_reg(irq))));
	DBG("S5PV210_EINTMASK = %x\n",readl(S5PV210_EINTMASK(eint_mask_reg(irq))));
	DBG("S5PV210_EINTPEND = %x\n",readl(S5PV210_EINTPEND(eint_pend_reg(irq))));
	
	return;
}

static void s3c_pm_clear_eint(unsigned int irq)
{
	u32 mask;
	/*clear pending*/
	mask = readl(S5PV210_EINTPEND(eint_pend_reg(irq)));
	mask &= (eint_irq_to_bit(irq));
	writel(mask, S5PV210_EINTPEND(eint_pend_reg(irq)));
}
#endif

/* helper functions to save and restore register state */

/**
 * s3c_pm_do_save() - save a set of registers for restoration on resume.
 * @ptr: Pointer to an array of registers.
 * @count: Size of the ptr array.
 *
 * Run through the list of registers given, saving their contents in the
 * array for later restoration when we wakeup.
 */
void s3c_pm_do_save(struct sleep_save *ptr, int count)
{
	for (; count > 0; count--, ptr++) {
		ptr->val = __raw_readl(ptr->reg);
		S3C_PMDBG("saved %p value %08lx\n", ptr->reg, ptr->val);
	}
}

/**
 * s3c_pm_do_restore() - restore register values from the save list.
 * @ptr: Pointer to an array of registers.
 * @count: Size of the ptr array.
 *
 * Restore the register values saved from s3c_pm_do_save().
 *
 * Note, we do not use S3C_PMDBG() in here, as the system may not have
 * restore the UARTs state yet
*/

void s3c_pm_do_restore(struct sleep_save *ptr, int count)
{
	for (; count > 0; count--, ptr++)
		__raw_writel(ptr->val, ptr->reg);
}

/**
 * s3c_pm_do_restore_core() - early restore register values from save list.
 *
 * This is similar to s3c_pm_do_restore() except we try and minimise the
 * side effects of the function in case registers that hardware might need
 * to work has been restored.
 *
 * WARNING: Do not put any debug in here that may effect memory or use
 * peripherals, as things may be changing!
*/

void s3c_pm_do_restore_core(struct sleep_save *ptr, int count)
{
	for (; count > 0; count--, ptr++)
		__raw_writel(ptr->val, ptr->reg);
}

/* s3c2410_pm_show_resume_irqs
 *
 * print any IRQs asserted at resume time (ie, we woke from)
*/
static void s3c_pm_show_resume_irqs(int start, unsigned long which,
				    unsigned long mask)
{
	int i;

	which &= ~mask;

	for (i = 0; i <= 31; i++) {
		if (which & (1L<<i)) {
			S3C_PMDBG("IRQ %d asserted at resume\n", start+i);
		}
	}
}

#if 1 //pending_int
bool s3c_pm_check_pending_interrupt(void)
{
	bool ret=true;

	s5pc11x_pm_wakeup_eint0_pend =__raw_readl(S5PV210_EINT0PEND);		/* EINT30[0] ~  EINT30[7]  */
	s5pc11x_pm_wakeup_eint1_pend =__raw_readl(S5PV210_EINT1PEND);		/* EINT31[0] ~  EINT31[7] */
	s5pc11x_pm_wakeup_eint2_pend =__raw_readl(S5PV210_EINT2PEND);		/* EINT32[0] ~  EINT32[7] */
	s5pc11x_pm_wakeup_eint3_pend =__raw_readl(S5PV210_EINT3PEND);		/* EINT33[0] ~  EINT33[7] */

#if defined(CONFIG_MACH_S5PC110_PRESTIGE)
	if(s5pc11x_pm_wakeup_eint0_pend) {	
		if(s5pc11x_pm_wakeup_eint0_pend & 0x40) {	// AP_HALL_SW : EINT_6 (1<<6)
			printk(KERN_ERR "[%s] foler flip interrupt pending.\n", __func__);
			ret=false;
		}
	}
#endif

	if(s5pc11x_pm_wakeup_eint1_pend) {	
		if(s5pc11x_pm_wakeup_eint1_pend & 0x08) {	// ONEDRAM_INT : EINT_11	(1<<3)
			printk(KERN_ERR "[%s] onedram interrupt pending.\n", __func__);
			ret=false;
		}
	}

	if(s5pc11x_pm_wakeup_eint1_pend) {
		if(s5pc11x_pm_wakeup_eint1_pend & 0x40) {	// DPRAM_INT : EINT_14 (1<<6)
			printk(KERN_ERR "[%s] idpram interrupt pending.\n", __func__);
			ret=false;
		}
	}
	
	if(s5pc11x_pm_wakeup_eint2_pend) {
		if(s5pc11x_pm_wakeup_eint2_pend & 0x40) {	// POWER_KEY : EINT_22 (1<<6)
			printk(KERN_ERR "[%s] power_key interrupt pending.\n", __func__);
			ret=false;
		}
	}

	if(s5pc11x_pm_wakeup_eint3_pend) {
		if(s5pc11x_pm_wakeup_eint3_pend & 0x20) {	// OK_KEY : EINT_29 (1<<5)
			printk(KERN_ERR "[%s] ok_key interrupt pending.\n", __func__);
			ret=false;
		}
	}
	return ret;
}
#endif


void (*pm_cpu_prep)(void);
void (*pm_cpu_sleep)(void);
void (*pm_cpu_restore)(void);

#define any_allowed(mask, allow) (((mask) & (allow)) != (allow))

#if defined(CONFIG_MACH_S5PC110_PRESTIGE)
extern unsigned int s3c_keygpio_get_flip_status(void);
#endif

/* s3c_pm_enter
 *
 * central control for sleep/resume process
*/

static int s3c_pm_enter(suspend_state_t state)
{
	static unsigned long regs_save[16];
#if 1 //pending_int
	unsigned int gpio;

	// 20110207 - check pending interrupt to wakeup device
	if(!s3c_pm_check_pending_interrupt())
	{
		printk(KERN_ERR "[%s] interrupt pending. wakeup!!(1)\n", __func__);	
		return -EINVAL;
	}
	
	// 20110113 - dukho.kim : control power of moviNAND at PM and add 400ms delay for stabilization of moviNAND. 
	gpio = readl(S5PV210_MP04DAT);
	writel(gpio & (~0x70), S5PV210_MP04DAT);
	mdelay(400);

	// H110419-4425 : Enable interrupt pending when delay to power off the iNand before enter sleep.
	if(!s3c_pm_check_pending_interrupt())
	{
		printk(KERN_ERR "[%s] interrupt pending. wakeup!!(2)\n", __func__);	
		return -EINVAL;
	}
#endif

	/* ensure the debug is initialised (if enabled) */

	s3c_pm_debug_init();

	S3C_PMDBG("%s(%d)\n", __func__, state);

	if (pm_cpu_prep == NULL || pm_cpu_sleep == NULL) {
		printk(KERN_ERR "%s: error: no cpu sleep function\n", __func__);
		return -EINVAL;
	}
#if 1 //pending_int
	s5pc11x_pm_wakeup_eint0_pend = 0;
	s5pc11x_pm_wakeup_eint1_pend = 0;
	s5pc11x_pm_wakeup_eint2_pend = 0;
	s5pc11x_pm_wakeup_eint3_pend = 0;
#endif
	/* check if we have anything to wake-up with... bad things seem
	 * to happen if you suspend with no wakeup (system will often
	 * require a full power-cycle)
	*/
#if defined(CONFIG_MACH_S5PC110_PRESTIGE)
	if (s3c_keygpio_get_flip_status() == 0)
	{
		s3c_irqwake_intmask = 0xFFDD; // key, rtc_alarm
	} else {
		s3c_irqwake_intmask = 0xFFFD; // rtc_alarm
	}
#else
	s3c_irqwake_intmask = 0xFFFD; // rtc_alarm
#endif

	if (!any_allowed(s3c_irqwake_intmask, s3c_irqwake_intallow) &&
	    !any_allowed(s3c_irqwake_eintmask, s3c_irqwake_eintallow)) {
		printk(KERN_ERR "%s: No wake-up sources!\n", __func__);
		printk(KERN_ERR "%s: Aborting sleep\n", __func__);
		return -EINVAL;
	}

	/* store the physical address of the register recovery block */

	s3c_sleep_save_phys = virt_to_phys(regs_save);

	S3C_PMDBG("s3c_sleep_save_phys=0x%08lx\n", s3c_sleep_save_phys);

	/* save all necessary core registers not covered by the drivers */

	s3c_pm_save_gpios();
	s3c_pm_save_uarts();
	s3c_pm_save_core();

	config_sleep_gpio();

	/* set the irq configuration for wake */

	s3c_pm_configure_extint();

	S3C_PMDBG("sleep: irq wakeup masks: %08lx,%08lx\n",
	    s3c_irqwake_intmask, s3c_irqwake_eintmask);

	s3c_pm_set_eint( 6, 0x4); // det_3.5
	s3c_pm_set_eint( 7, 0x2); // pmic
	s3c_pm_set_eint(11, 0x2); // onedram
	s3c_pm_set_eint(12, 0x4); //tflash
	s3c_pm_set_eint(13, 0x2); //low_batt
	s3c_pm_set_eint(14, 0x3); //qsc_int - rising edge
	s3c_pm_set_eint(15, 0x2); //wlan
	s3c_pm_set_eint(21, 0x4); // bt
	s3c_pm_set_eint(22, 0x2); // power key
	s3c_pm_set_eint(23, 0x2); // microusb

#if defined(CONFIG_MACH_S5PC110_PRESTIGE)
	if (s3c_keygpio_get_flip_status() == 0)
	{
		s3c_pm_set_eint(24, 0x4); // KBR(0)
		s3c_pm_set_eint(25, 0x4); // KBR(1)
		s3c_pm_set_eint(26, 0x4); // KBR(2)
		s3c_pm_set_eint(27, 0x4); // KBR(3)
		s3c_pm_set_eint(28, 0x4); // KBR(4)
		s3c_pm_set_eint(29, 0x4); //ok_key
		s3c_pm_set_eint(31, 0x4); // KBR(5)
	} else {
		s3c_pm_set_eint(31, 0x4); // KBR(5)
	}
#else
	s3c_pm_set_eint(25, 0x4); // volume down
	s3c_pm_set_eint(26, 0x4); // volume up
	s3c_pm_set_eint(29, 0x4); // ok key
#if 0 //TEMP
   	if(get_headset_status() & SEC_HEADSET_4_POLE_DEVICE)
	{
	    s3c_pm_set_eint(30, 0x4); //sendend
	}
	else
#endif
	{
	    s3c_pm_clear_eint(30);
	}
#endif

	//s3c_pm_arch_prepare_irqs();

	/* call cpu specific preparation */

	pm_cpu_prep();

	/* flush cache back to ram */

	flush_cache_all();

	s3c_pm_check_store();

	__raw_writel(s3c_irqwake_intmask, S5P_WAKEUP_MASK); //0xFFDD:key, RTC_ALARM

	/* clear wakeup_stat register for next wakeup reason */
	__raw_writel(__raw_readl(S5P_WAKEUP_STAT), S5P_WAKEUP_STAT);

	/* send the cpu to sleep... */

	s3c_pm_arch_stop_clocks();

	/* s3c_cpu_save will also act as our return point from when
	 * we resume as it saves its own register state and restores it
	 * during the resume.  */

	pmstats->sleep_count++;
	pmstats->sleep_freq = __raw_readl(S5P_CLK_DIV0);
	s3c_cpu_save(regs_save);
	pmstats->wake_count++;
	pmstats->wake_freq = __raw_readl(S5P_CLK_DIV0);

	/* restore the cpu state using the kernel's cpu init code. */

	cpu_init();

	fiq_glue_resume();
	local_fiq_enable();

	s3c_pm_restore_core();
	s3c_pm_restore_uarts();
	s3c_pm_restore_gpios();
	s5pv210_restore_eint_group();

	s3c_pm_debug_init();

        /* restore the system state */

	if (pm_cpu_restore)
		pm_cpu_restore();

	/* check what irq (if any) restored the system */

	s3c_pm_arch_show_resume_irqs();

	S3C_PMDBG("%s: post sleep, preparing to return\n", __func__);

	/* LEDs should now be 1110 */
	s3c_pm_debug_smdkled(1 << 1, 0);

	s3c_pm_check_restore();

	/* ok, let's return from sleep */

	S3C_PMDBG("S3C PM Resume (post-restore)\n");
	return 0;
}

/* callback from assembly code */
void s3c_pm_cb_flushcache(void)
{
	flush_cache_all();
}

static int s3c_pm_prepare(void)
{
	/* prepare check area if configured */

	s3c_pm_check_prepare();
	return 0;
}

static void s3c_pm_finish(void)
{
	s3c_pm_check_cleanup();
}

static struct platform_suspend_ops s3c_pm_ops = {
	.enter		= s3c_pm_enter,
	.prepare	= s3c_pm_prepare,
	.finish		= s3c_pm_finish,
	.valid		= suspend_valid_only_mem,
};

/* s3c_pm_init
 *
 * Attach the power management functions. This should be called
 * from the board specific initialisation if the board supports
 * it.
*/

int __init s3c_pm_init(void)
{
	printk("S3C Power Management, Copyright 2004 Simtec Electronics\n");
	pmstats_init();

	suspend_set_ops(&s3c_pm_ops);
	return 0;
}
