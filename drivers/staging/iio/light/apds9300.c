/*
 * apds9300.c - IIO driver for Avago APDS9300 ambient light sensor
 *
 * Copyright 2013 Oleksandr Kravchenko <o.v.kravchenko@globallogic.com>
 *
 * This file is subject to the terms and conditions of version 2 of
 * the GNU General Public License.  See the file COPYING in the main
 * directory of this archive for more details.
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/mutex.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include "../iio.h"
#include <linux/sysfs.h>
#include <config/proc/events.h>

#define ALS_DRV_NAME "apds9300"
#define ALS_IRQ_NAME "apds9300_event"

/* Command register bits */
#define ALS_CMD		BIT(7) /* Select command register. Must write as 1 */
#define ALS_WORD	BIT(5) /* I2C write/read: if 1 word, if 0 byte */
#define ALS_CLEAR	BIT(6) /* Interrupt clear. Clears pending interrupt */

/* Register set */
#define ALS_CONTROL		0x00 /* Control of basic functions */
#define ALS_THRESHLOWLOW	0x02 /* Low byte of low interrupt threshold */
#define ALS_THRESHHIGHLOW	0x04 /* Low byte of high interrupt threshold */
#define ALS_INTERRUPT		0x06 /* Interrupt control */
#define ALS_DATA0LOW		0x0c /* Low byte of ADC channel 0 */
#define ALS_DATA1LOW		0x0e /* Low byte of ADC channel 1 */

/* Power on/off value for ALS_CONTROL register */
#define ALS_POWER_ON		0x03
#define ALS_POWER_OFF		0x00

/* Interrupts */
#define ALS_INTR_ENABLE		0x10
/* Interrupt Persist Function: Any value outside of threshold range */
#define ALS_THRESH_INTR		0x01

#define ALS_THRESH_MAX		0xffff /* Max threshold value */

struct als_data {
	struct i2c_client *client;
	struct mutex mutex;
	int power_state;
	int thresh_low;
	int thresh_hi;
	int intr_en;
};

/* Lux calculation */

/* Calculated values 1000 * (CH1/CH0)^1.4 for CH1/CH0 from 0 to 0.52 */
static const u16 lux_ratio[] = {
	0, 2, 4, 7, 11, 15, 19, 24, 29, 34, 40, 45, 51, 57, 64, 70, 77, 84, 91,
	98, 105, 112, 120, 128, 136, 144, 152, 160, 168, 177, 185, 194, 203,
	212, 221, 230, 239, 249, 258, 268, 277, 287, 297, 307, 317, 327, 337,
	347, 358, 368, 379, 390, 400,
};

static unsigned long als_calculate_lux(u16 ch0, u16 ch1)
{
	unsigned long lux, tmp;
	u64 tmp64;

	/* avoid division by zero */
	if (ch0 == 0)
		return 0;

	tmp = ch1 * 100 / ch0;
	if (tmp <= 52) {
		/*
		 * Variable tmp64 need to avoid overflow of this part of lux
		 * calculation formula.
		 */
		tmp64 = ch0 * lux_ratio[tmp] * 5930 / 1000;
		lux = 3150 * ch0 - (unsigned long)tmp64;
	}
	else if (tmp <= 65)
		lux = 2290 * ch0 - 2910 * ch1;
	else if (tmp <= 80)
		lux = 1570 * ch0 - 1800 * ch1;
	else if (tmp <= 130)
		lux = 338 * ch0 - 260 * ch1;
	else
		lux = 0;

	return lux / 100000;
}

/* I2C I/O operations */

static int als_set_power_state(struct als_data *data, int state)
{
	int ret;
	u8 cmd;

	cmd = state ? ALS_POWER_ON : ALS_POWER_OFF;
	ret = i2c_smbus_write_byte_data(data->client,
			ALS_CONTROL | ALS_CMD, cmd);
	if (!ret)
		data->power_state = state;
	else
		dev_err(&data->client->dev,
				"failed to set power state %d\n", state);

	return ret;
}

static int als_get_adc_val(struct als_data *data, int adc_number)
{
	int ret;
	u8 flags = ALS_CMD | ALS_WORD;

	if (!data->power_state)
		return -EAGAIN;

	/* Select ADC0 or ADC1 data register */
	flags |= adc_number ? ALS_DATA1LOW : ALS_DATA0LOW;

	ret = i2c_smbus_read_word_data(data->client, flags);
	if (ret < 0)
		dev_err(&data->client->dev,
				"failed to read ADC%d value\n", adc_number);

	return ret;
}

static int als_set_thresh_low(struct als_data *data, int value)
{
	int ret;

	if (!data->power_state)
		return -EAGAIN;

	if (value > ALS_THRESH_MAX || value > data->thresh_hi)
		return -EINVAL;

	ret = i2c_smbus_write_word_data(data->client,
			ALS_THRESHLOWLOW | ALS_CMD | ALS_WORD, value);
	if (!ret)
		data->thresh_low = value;
	else
		dev_err(&data->client->dev,
				"failed to set thresh_low\n");

	return ret;
}

static int als_set_thresh_hi(struct als_data *data, int value)
{
	int ret;

	if (!data->power_state)
		return -EAGAIN;

	if (value > ALS_THRESH_MAX || value < data->thresh_low)
		return -EINVAL;

	ret = i2c_smbus_write_word_data(data->client,
			ALS_THRESHHIGHLOW | ALS_CMD | ALS_WORD, value);
	if (!ret)
		data->thresh_hi = value;
	else
		dev_err(&data->client->dev,
				"failed to set thresh_hi\n");

	return ret;
}

static int als_set_intr_state(struct als_data *data, int state)
{
	int ret;
	u8 cmd;

	if (!data->power_state)
		return -EAGAIN;

	cmd = state ? ALS_INTR_ENABLE | ALS_THRESH_INTR : 0x00;
	ret = i2c_smbus_write_byte_data(data->client,
			ALS_INTERRUPT | ALS_CMD, cmd);
	if (!ret)
		data->intr_en = state;
	else
		dev_err(&data->client->dev,
				"failed to set interrupt state %d\n", state);

	return ret;
}

static void als_clear_intr(struct als_data *data)
{
	int ret;

	ret = i2c_smbus_write_byte(data->client, ALS_CLEAR | ALS_CMD);
	if (ret < 0)
		dev_err(&data->client->dev,
				"failed to clear interrupt\n");
}

static int als_chip_init(struct als_data *data)
{
	int ret;

	/* Need to set power off to ensure that the chip is off */
	ret = als_set_power_state(data, 0);
	if (ret < 0)
		goto err;
	/*
	 * Probe the chip. To do so we try to power up the device and then to
	 * read back the 0x03 code
	 */
	ret = als_set_power_state(data, 1);
	if (ret < 0)
		goto err;
	ret = i2c_smbus_read_byte_data(data->client, ALS_CONTROL | ALS_CMD);
	if (ret != ALS_POWER_ON) {
		ret = -ENODEV;
		goto err;
	}
	/*
	 * Disable interrupt to ensure thai it is doesn't enable
	 * i.e. after device soft reset
	 */
	ret = als_set_intr_state(data, 0);
	if (ret < 0)
		goto err;

	return 0;

err:
	dev_err(&data->client->dev, "failed to init the chip\n");
	return ret;
}

/* Industrial I/O data and functions */

static int als_read_raw(struct iio_dev *indio_dev,
		struct iio_chan_spec const *chan, int *val, int *val2,
		long mask)
{
	int ch0, ch1, ret = -EINVAL;
	struct als_data *data = iio_priv(indio_dev);

	mutex_lock(&data->mutex);

	switch (chan->type) {
	case IIO_LIGHT:
		ch0 = als_get_adc_val(data, 0);
		if (ch0 < 0) {
			ret = ch0;
			break;
		}
		ch1 = als_get_adc_val(data, 1);
		if (ch1 < 0) {
			ret = ch1;
			break;
		}
		*val = als_calculate_lux(ch0, ch1);
		ret = IIO_VAL_INT;
		break;
	case IIO_INTENSITY:
		ret = als_get_adc_val(data, chan->channel);
		if (ret < 0)
			break;
		*val = ret;
		ret = IIO_VAL_INT;
		break;
	default:
		break;
	}
 
	mutex_unlock(&data->mutex);

	return ret;
}

static int als_read_thresh(struct iio_dev *indio_dev, int event_code, int *val)
{
	struct als_data *data = iio_priv(indio_dev);

	switch (IIO_EVENT_CODE_EXTRACT_DIR(event_code)) {
	case IIO_EV_DIR_RISING:
		*val = data->thresh_hi;
		break;
	case IIO_EV_DIR_FALLING:
		*val = data->thresh_low;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int als_write_thresh(struct iio_dev *indio_dev, int event_code, int val)
{
	struct als_data *data = iio_priv(indio_dev);
	int ret;

	mutex_lock(&data->mutex);
	if (IIO_EVENT_CODE_EXTRACT_DIR(event_code) == IIO_EV_DIR_RISING)
		ret = als_set_thresh_hi(data, val);
	else
		ret = als_set_thresh_low(data, val);
	mutex_unlock(&data->mutex);

	return ret;
}

static int als_read_interrupt_config(struct iio_dev *indio_dev,
		int event_code)
{
	struct als_data *data = iio_priv(indio_dev);
	return data->intr_en;
}

static int als_write_interrupt_config(struct iio_dev *indio_dev,
		int event_code, int state)
{
	struct als_data *data = iio_priv(indio_dev);
	int ret = 0;

	mutex_lock(&data->mutex);
	ret = als_set_intr_state(data, state);
	mutex_unlock(&data->mutex);

	return ret;
}

static const struct iio_info als_info_no_irq = {
	.driver_module	= THIS_MODULE,
	.read_raw	= &als_read_raw,
};

static const struct iio_info als_info = {
	.driver_module		= THIS_MODULE,
	.read_raw		= als_read_raw,
	.read_event_value	= &als_read_thresh,
	.write_event_value	= &als_write_thresh,
	.read_event_config	= &als_read_interrupt_config,
	.write_event_config	= &als_write_interrupt_config,
};

static const struct iio_chan_spec als_channels[] = {
		IIO_CHAN(IIO_LIGHT, 0, 1, 1, NULL, 0, 0, 0, 0, 0, {}, 0),
	IIO_CHAN(IIO_INTENSITY, 1, 1, 0, "both", 0,
		 (1 << IIO_CHAN_INFO_CALIBSCALE_SEPARATE), 0, 0, 0, {},
		 IIO_EV_BIT(IIO_EV_TYPE_THRESH, IIO_EV_DIR_RISING) |
		 IIO_EV_BIT(IIO_EV_TYPE_THRESH, IIO_EV_DIR_FALLING)),
	IIO_CHAN(IIO_INTENSITY, 1, 1, 0, "ir", 1,
		 (1 << IIO_CHAN_INFO_CALIBSCALE_SEPARATE), 0, 0, 0, {},
		 0),
};

static irqreturn_t als_interrupt_handler(int irq, void *private)
{
	struct iio_dev *dev_info = private;
	struct als_data *data = iio_priv(dev_info);

	iio_push_event(dev_info, 0,
		       IIO_UNMOD_EVENT_CODE(IIO_EV_CLASS_LIGHT,
					    0,
					    IIO_EV_TYPE_THRESH,
					    IIO_EV_DIR_EITHER),
		       iio_get_time_ns());

	als_clear_intr(data);

	return IRQ_HANDLED;
}

/* Probe/remove functions */

static int __devinit als_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct als_data *data;
	struct iio_dev *indio_dev;
	int ret;
	
	indio_dev = iio_allocate_device(sizeof(*data));
	if (!indio_dev)
		return -ENOMEM;
	
	data = iio_priv(indio_dev);
	i2c_set_clientdata(client, indio_dev);
	data->client = client;
	
	ret = als_chip_init(data);
	if (ret < 0)
		goto err;
	
	mutex_init(&data->mutex);
	
	indio_dev->dev.parent = &client->dev;
	indio_dev->channels = als_channels;
	indio_dev->num_channels = ARRAY_SIZE(als_channels);
	indio_dev->name = ALS_DRV_NAME;
	indio_dev->modes = INDIO_DIRECT_MODE;
	
	if (client->irq)
		indio_dev->info = &als_info;
	else
		indio_dev->info = &als_info_no_irq;

	if (client->irq) {
		ret = devm_request_threaded_irq(&client->dev, client->irq,
				NULL, als_interrupt_handler,
				IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
				ALS_IRQ_NAME, indio_dev);
		if (ret) {
			dev_err(&client->dev, "irq request error %d\n", -ret);
			goto err;
		}
	}
	
	ret = iio_device_register(indio_dev);
	if (ret < 0)
		goto err;

	dev_info(&client->dev, "ambient light sensor\n");

	return 0;

err:
	/* Ensure that power off in case of error */
	als_set_power_state(data, 0);
	iio_device_unregister(indio_dev);
	return ret;
}

static int als_remove(struct i2c_client *client)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(client);
	struct als_data *data = iio_priv(indio_dev);
	int ret;

	iio_device_unregister(indio_dev);

	/* Ensure that power off and interrupts are disabled */
	ret = als_set_intr_state(data, 0);
	if (!ret)
		ret = als_set_power_state(data, 0);

//	iio_device_free(indio_dev);

	return ret;
}

#ifdef CONFIG_PM_SLEEP
static int als_suspend(struct device *dev)
{
	struct als_data *data = i2c_get_clientdata(to_i2c_client(dev));
	int ret;

	mutex_lock(&data->mutex);
	ret = als_set_power_state(data, 0);
	mutex_unlock(&data->mutex);

	return ret;
}

static int als_resume(struct device *dev)
{
	struct als_data *data = i2c_get_clientdata(to_i2c_client(dev));
	int ret;

	mutex_lock(&data->mutex);
	ret = als_set_power_state(data, 1);
	mutex_unlock(&data->mutex);

	return ret;
}

static SIMPLE_DEV_PM_OPS(als_pm_ops, als_suspend, als_resume);
#define ALS_PM_OPS (&als_pm_ops)
#else
#define ALS_PM_OPS NULL
#endif

static struct i2c_device_id als_id[] = {
	{ ALS_DRV_NAME, 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, als_id);

static struct i2c_driver als_driver = {
	.driver = {
		.name	= ALS_DRV_NAME,
		.owner	= THIS_MODULE,
		.pm	= ALS_PM_OPS,
	},
	.probe		= als_probe,
	.remove		= als_remove,
	.id_table	= als_id,
};

static int __init apds9300_init(void)
{
	return i2c_add_driver(&als_driver);
}

static void __exit apds9300_exit(void)
{
	i2c_del_driver(&als_driver);
}


module_init(apds9300_init);
module_exit(apds9300_exit);

MODULE_AUTHOR("Kravchenko Oleksandr <o.v.kravchenko@globallogic.com>");
MODULE_AUTHOR("GlobalLogic inc.");
MODULE_DESCRIPTION("APDS9300 ambient light photo sensor driver");
MODULE_LICENSE("GPL");
