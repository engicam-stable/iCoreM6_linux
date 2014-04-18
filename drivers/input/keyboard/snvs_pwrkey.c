/*
 * Copyright (C) 2011-2012 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/mm.h>
#include <linux/init.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <linux/kd.h>
#include <linux/fs.h>
#include <linux/ioctl.h>
#include <linux/input.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>

#define	SNVS_LPSR_REG	0x4C	/* LP Status Register */
#define	SNVS_LPCR_REG	0x38	/* LP Control Register */
#define SNVS_LPSR_SPO	(0x1 << 18)
#define SNVS_LPSR_EO	(0x1 << 17)
#define SNVS_LPSR_MASK	0x0f000000
#define SNVS_LPCR_TOP	(0x1 << 6)
#define SNVS_LPCR_DEP_EN	(0x1 << 5)
static struct wake_lock wakelock;

struct pwrkey_drv_data {
	void __iomem *ioaddr;
	unsigned long baseaddr;
	int irq;
	int keycode;
	bool keyup;
	bool keydown;
	struct work_struct work;
	struct input_dev *input;
};

static void pwr_keys_work_func(struct work_struct *work)
{
	struct pwrkey_drv_data *pdata =
		container_of(work, struct pwrkey_drv_data, work);
	struct input_dev *input = pdata->input;
	bool up = pdata->keyup;
	bool down = pdata->keydown;

	if (up){
		input_event(input, EV_KEY, pdata->keycode, down);
		input_sync(input);
		input_event(input, EV_KEY, pdata->keycode, !up);
		input_sync(input);
	} else{
		input_event(input, EV_KEY, pdata->keycode, !down);
		input_sync(input);
	}

	//enable_irq(pdata->irq);

}

/*!
 * This function is the SNVS power off request  interrupt service routine.
 *
 * @param  irq          RTC IRQ number
 * @param  dev_id       device ID which is not used
 *
 * @return IRQ_HANDLED as defined in the include/linux/interrupt.h file.
 */
static irqreturn_t snvs_pwrkey_interrupt(int irq, void *dev_id)
{
	struct platform_device *pdev = dev_id;
	struct pwrkey_drv_data *pdata = platform_get_drvdata(pdev);
	void __iomem *ioaddr = pdata->ioaddr;
	u32 lp_status, lp_cr;
	u32 events = 0;

	lp_status = __raw_readl(ioaddr + SNVS_LPSR_REG);
	lp_cr = __raw_readl(ioaddr + SNVS_LPCR_REG);
	printk(" =======%s()lpstatus 0x%x, lp_cr 0x%x \n",__func__, lp_status, lp_cr);
	printk("Unhandled key !!! \n");
#if 0	
	if (lp_cr & SNVS_LPCR_TOP)
		printk(" power off the pmic request trigger");
	if (lp_cr & SNVS_LPCR_DEP_EN)
		printk(" set snvs as DUMP PMIC MODE\n");
	else
		printk(" Not DUMP PMIC MODE, EO will be ingnored!\n");

	if (lp_status & SNVS_LPSR_SPO && !(lp_status & SNVS_LPSR_EO)) {
		pdata->keyup = true;
		pdata->keydown = true;
	} else {
		pdata->keyup = false;
		pdata->keydown = true;
	}	
#endif

	/* clear SPO/EO status */
	__raw_writel(lp_status, ioaddr + SNVS_LPSR_REG);

	//disable_irq(pdata->irq);
	schedule_work(&pdata->work);

	return IRQ_HANDLED;
}

static int snvs_pwrkey_probe(struct platform_device *pdev)
{
	struct pwrkey_drv_data *pdata = NULL;
	struct input_dev *input = NULL;
	void __iomem *ioaddr;
	u32 lp_cr;
	int ret = 0;
	struct resource *res;
	int retval;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENODEV;

	pdata = kzalloc(sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return -ENOMEM;

	pdata->baseaddr = res->start;
	pdata->ioaddr = ioremap(pdata->baseaddr, 0xC00);
	ioaddr = pdata->ioaddr;
	pdata->irq = platform_get_irq(pdev, 0);
	pdata->keycode = KEY_POWER;
	platform_set_drvdata(pdev, pdata);
	lp_cr = __raw_readl(ioaddr + SNVS_LPCR_REG);
	printk(" =======%s() lp_cr 0x%x \n",__func__, lp_cr);
	lp_cr |= SNVS_LPCR_DEP_EN,
	printk(" =======%s() lp_cr 0x%x \n",__func__, lp_cr);
	__raw_writel(lp_cr, ioaddr + SNVS_LPCR_REG);

	INIT_WORK(&pdata->work, pwr_keys_work_func);

	if (pdata->irq >= 0) {

		ret = request_irq(pdata->irq, snvs_pwrkey_interrupt, IRQF_TRIGGER_HIGH,
				pdev->name, pdev); 
		if(ret < 0) {
			dev_warn(&pdev->dev, "interrupt not available.\n");
			pdata->irq = -1;
		} else 
			disable_irq(pdata->irq);
	}

	input = input_allocate_device();
	if (!input) {
		dev_err(&pdev->dev, "no memory for input device\n");
		retval = -ENOMEM;
		goto err1;
	}

	input->name = "snvs_power_key";
	input->phys = "snvspwrkey/input0";
	input->id.bustype = BUS_HOST;
	input->evbit[0] = BIT_MASK(EV_KEY);

	input_set_capability(input, EV_KEY, KEY_POWER);

	retval = input_register_device(input);
	if (retval < 0) {
		dev_err(&pdev->dev, "failed to register input device\n");
		goto err2;
	}
	pdata->input = input;
	device_init_wakeup(&pdev->dev, 1);
	enable_irq(pdata->irq);
	printk(KERN_INFO "i.MX6 powerkey probe\n");

	return 0;

err2:
	input_free_device(input);
err1:
	platform_set_drvdata(pdev, NULL);
	cancel_work_sync(&pdata->work);
	return retval;
}

static int snvs_pwrkey_remove(struct platform_device *pdev)
{
	struct pwrkey_drv_data *pdata = platform_get_drvdata(pdev);
	input_unregister_device(pdata->input);
	input_free_device(pdata->input);
	cancel_work_sync(&pdata->work);

	return 0;
}

static int snvs_pwrkey_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct pwrkey_drv_data *pdata = platform_get_drvdata(pdev);

	if (device_may_wakeup(&pdev->dev))
		enable_irq_wake(pdata->irq);

	return 0;
}

static int snvs_pwrkey_resume(struct platform_device *pdev)
{
	struct pwrkey_drv_data *pdata = platform_get_drvdata(pdev);

	if (device_may_wakeup(&pdev->dev))
		disable_irq_wake(pdata->irq);

	return 0;
}

static struct platform_driver snvs_pwrkey_driver = {
	.driver = {
		.name = "snvs_pwrkey",
	},
	.probe = snvs_pwrkey_probe,
	.remove = snvs_pwrkey_remove,
	.suspend =  snvs_pwrkey_suspend,
	.resume =  snvs_pwrkey_resume,
};

static int __init snvs_pwrkey_init(void)
{
	return platform_driver_register(&snvs_pwrkey_driver);
}

static void __exit snvs_pwrkey_exit(void)
{
	platform_driver_unregister(&snvs_pwrkey_driver);
}

module_init(snvs_pwrkey_init);
module_exit(snvs_pwrkey_exit);


MODULE_AUTHOR("Freescale Semiconductor");
MODULE_DESCRIPTION("MXC snvs power key Driver");
MODULE_LICENSE("GPL");
