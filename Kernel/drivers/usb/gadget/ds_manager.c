#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <plat/s5pv210.h>
#include <plat/gpio-cfg.h>
#include <mach/regs-gpio.h>
#include <mach/param.h>

#include "ds_manager.h"

extern struct device *slot_switch_dev;

static ssize_t get_slot_switch(struct device *dev, struct device_attribute *attr, char *buf)
{
		int value;

//		printk("get_slot_switch !!\n");
#if defined(CONFIG_MACH_S5PC110_PRESTIGE)
		value = ((~ gpio_get_value(GPIO_UIM_SIM_SEL)) & 0x1);
#else
		value = (gpio_get_value(GPIO_UIM_SIM_SEL) & 0x1);
#endif
//		printk("Current Slot is %x\n", value);

		return sprintf(buf, "%d\n", value);
}

static ssize_t set_slot_switch(struct device *dev, struct device_attribute *attr,   const char *buf, size_t size)
{
		int value;

		printk("set_slot_switch !!\n");
		sscanf(buf, "%d", &value);

		switch(value) {
				case 0:
#if defined(CONFIG_MACH_S5PC110_PRESTIGE)
						gpio_set_value(GPIO_UIM_SIM_SEL, GPIO_LEVEL_HIGH);
#else
						gpio_set_value(GPIO_UIM_SIM_SEL, GPIO_LEVEL_LOW);
#endif
						cdma_slot_switch_handler(SIM_SLOT_0);
						gsm_slot_switch_handler(SIM_SLOT_0);
						printk("set slot switch to %x\n", gpio_get_value(GPIO_UIM_SIM_SEL));
						break;
				case 1:
#if defined(CONFIG_MACH_S5PC110_PRESTIGE)
						gpio_set_value(GPIO_UIM_SIM_SEL, GPIO_LEVEL_LOW);
#else
						gpio_set_value(GPIO_UIM_SIM_SEL, GPIO_LEVEL_HIGH);
#endif
						cdma_slot_switch_handler(SIM_SLOT_1);
						gsm_slot_switch_handler(SIM_SLOT_1);
						printk("set slot switch to %x\n", gpio_get_value(GPIO_UIM_SIM_SEL));
						break;
				default:
						printk("Enter 0 or 1!!\n");
		}

		return size;
}

static DEVICE_ATTR(slot_sel, S_IRUGO |S_IWUGO | S_IRUSR | S_IWUSR, get_slot_switch, set_slot_switch);

static int __init ds_manager_init(void)
{
		int ret = 0;

		printk("ds_manager_init !!\n");
		if(gpio_is_valid(GPIO_UIM_SIM_SEL)) {
				printk("GPIO_UIM_SIM_SEL valid!!\n");
				if(gpio_request(GPIO_UIM_SIM_SEL, "GPB6"))
						printk(KERN_ERR "Failed to request GPIO_UIM_SIM_SEL!\n");
		}
		s3c_gpio_setpull(GPIO_UIM_SIM_SEL, S3C_GPIO_PULL_NONE);

		if(device_create_file(slot_switch_dev, &dev_attr_slot_sel) < 0)
				pr_err("Failed to create device file(%s)!\n", dev_attr_slot_sel.attr.name);

		return ret;
}

static void __exit ds_manager_exit(void)
{
		gpio_free(GPIO_UIM_SIM_SEL);
}

module_init(ds_manager_init);
module_exit(ds_manager_exit);

MODULE_AUTHOR("SAMSUNG ELECTRONICS CO., LTD");
MODULE_DESCRIPTION("Slot Switch");
MODULE_LICENSE("GPL");
