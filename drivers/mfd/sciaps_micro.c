#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/sysfs.h>
#include <linux/mod_devicetable.h>
#include <linux/of.h>
#include <linux/i2c.h>
#include <linux/types.h>

#include <linux/qpnp/power-on.h>

#include <linux/mfd/sciaps_micro.h>


struct sciaps_micro_data {
	struct	i2c_client	*client;
	u8 read_reg;
	u16 read_reg_value;
	bool			nvram_unlocked;
};

typedef struct {
	const char	*command;
	u8		buf[3];
} sciaps_micro_cmds;

static sciaps_micro_cmds sciaps_micro_known_cmds[] = {
	{ "reset_micro",	{0xA0, 0x7A, 0xA7} },
	{ "enter_bl",		{0xA1, 0x3C, 0xC3} },
	{ "unlock_nvram",	{0xA2, 0x55, 0xAA} },
};

static const int libs_cmd_num = sizeof(sciaps_micro_known_cmds)/sizeof(sciaps_micro_cmds);

#if 0
static int sciaps_micro_i2c_read(struct i2c_client *client, int count,
			u8 *buf)
{
	int ret;

	struct i2c_msg msgs[] = {
		{
			.addr	= client->addr,
			.flags	= I2C_M_RD,
			.len	= count,
			.buf	= buf,
		},
	};

	ret = i2c_transfer(client->adapter, msgs, 2);
	if (ret < 0)
		dev_err(&client->dev, "%s read error %d\n", __func__, ret);
	return ret;
}
#endif

static int sciaps_micro_i2c_write(struct i2c_client *client,
			u8 *buf, int count)
{
	int ret;

	struct i2c_msg msg[] = {
		{
			.addr	= client->addr,
			.flags	= 0,
			.len	= count,
			.buf	= buf,
		},
	};

	ret = i2c_transfer(client->adapter, msg, 1);
	if (ret < 0)
		dev_err(&client->dev, "%s write error %d\n", __func__, ret);
	return ret;
}

static int sciaps_micro_i2c_write_reg(struct i2c_client *client, u8 reg,
	u16 value)
{
	int ret;
	u8 value_buf[2] = { 0xFF & value, 0xFF & (value >> 8) };
	struct i2c_msg msgs[] = {
		{
			.addr	= client->addr,
			.flags	= 0,
			.len	= 1,
			.buf	= &reg,
		},
		{
			.addr	= client->addr,
			.flags	= 0,
			.len	= 2,
			.buf	= value_buf,
		},
	};

	ret = i2c_transfer(client->adapter, msgs, 2);
	if (ret < 0)
		dev_err(&client->dev, "%s read error %d\n", __func__, ret);
	return ret;
}

static int sciaps_micro_i2c_read_reg(struct i2c_client *client, u8 reg,
	u16 *value)
{
	int ret;
	u8 value_buf[2];
	struct i2c_msg msgs[] = {
		{
			.addr	= client->addr,
			.flags	= 0,
			.len	= 1,
			.buf	= &reg,
		},
		{
			.addr	= client->addr,
			.flags	= I2C_M_RD,
			.len	= 2,
			.buf	= value_buf,
		},
	};

	ret = i2c_transfer(client->adapter, msgs, 2);
	if (ret < 0) {
		dev_err(&client->dev, "%s read error %d\n", __func__, ret);
	} else {
		*value = (value_buf[1] << 8) | value_buf[0];
	}
	return ret;
}

static ssize_t sciaps_micro_print_reg(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct i2c_client* client = to_i2c_client(dev);
	struct sciaps_micro_data* drvdata = i2c_get_clientdata(client);

	return scnprintf(buf, PAGE_SIZE, "0x%02x 0x%04x\n",
		drvdata->read_reg, drvdata->read_reg_value);
}


static ssize_t sciaps_micro_set_read_reg(struct device *dev,
			struct device_attribute *attr, const char *buf,
			size_t count)
{
	struct i2c_client* client = to_i2c_client(dev);
	struct sciaps_micro_data* drvdata = i2c_get_clientdata(client);
	int err;

	err = kstrtou8(buf, 16, &drvdata->read_reg);
	if(err < 0){
		dev_err(dev, "Error parsing register num\n");
		return -EINVAL;
	}

	err = sciaps_micro_i2c_read_reg(drvdata->client,
		drvdata->read_reg, &drvdata->read_reg_value);
	if(err < 0) {
		dev_err(dev, "error reading from i2c bus: 0x%x", err);
		return err;
	}

	return err;
}

static ssize_t sciaps_micro_store_reg(struct device *dev,
			struct device_attribute *attr, const char *buf,
			size_t count)
{
	struct i2c_client* client = to_i2c_client(dev);
	int err;
	u8 reg;
	u16 value;

	err = sscanf(buf, "%hhu %hx", &reg, &value);
	if(err != 2) {
		dev_err(dev, "error parsing command\n");
		return -EINVAL;
	}

	err = sciaps_micro_i2c_write_reg(client, reg, value);
	if(err < 0) {
		dev_err(dev, "error writing i2c command: 0x%x\n", err);
	}

	return err;
}

static ssize_t sciaps_micro_write_data(struct device *dev,
			struct device_attribute *attr, const char *buf,
			size_t count)
{
	struct i2c_client* client = to_i2c_client(dev);
	int err;

	err = sciaps_micro_i2c_write(client, (u8*)buf, count);
	if(err < 0) {
		dev_err(dev, "error writing i2c data: 0x%x\n", err);
	}

	return err;
}

static ssize_t sciaps_trigger_status_show(struct device *child, struct device_attribute* attr, char* buf);

static DEVICE_ATTR(read_reg, 0666, sciaps_micro_print_reg, sciaps_micro_set_read_reg);
static DEVICE_ATTR(write_reg, 0222, NULL, sciaps_micro_store_reg);
static DEVICE_ATTR(write, 0222, NULL, sciaps_micro_write_data);
static DEVICE_ATTR(sciaps_trigger, 0444, sciaps_trigger_status_show, NULL);

static int sciaps_micro_setup_sysfs(struct i2c_client *client)
{
	device_create_file(&client->dev, &dev_attr_read_reg);
	device_create_file(&client->dev, &dev_attr_write_reg);
	device_create_file(&client->dev, &dev_attr_write);

	return 0;
}

static int sciaps_micro_delete_sysfs(struct i2c_client *client)
{
	device_remove_file(&client->dev, &dev_attr_read_reg);
	device_remove_file(&client->dev, &dev_attr_write_reg);
	device_remove_file(&client->dev, &dev_attr_write);

	return 0;
}

#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>

#define SCIAPS_TRIGGER_GPIO	1008
#define SCIAPS_TRIGGER_DEBOUNCE_TIME_MS_DEFAULT 10
#define SCIAPS_TRIGGER_STATE_DEFAULT	1

static int sciaps_trigger_gpio = -1;
static int sciaps_trigger_irq = -1;

static struct delayed_work sciaps_trigger_debounce_work;
static uint8_t sciaps_trigger_debounce_work_in_progress;
static int sciaps_trigger_state = SCIAPS_TRIGGER_STATE_DEFAULT;
static int sciaps_trigger_state_candidate;
static int sciaps_trigger_debounce_time_ms = SCIAPS_TRIGGER_DEBOUNCE_TIME_MS_DEFAULT;

static ssize_t sciaps_trigger_status_show(struct device *child, struct device_attribute* attr, char* buf)
{
	int value = 0;
	value = sciaps_trigger_state;
	return scnprintf(buf, PAGE_SIZE, "%x\n", value);
}

static void sciaps_notify_trigger(int value);

static void sciaps_trigger_debounce_work_func(struct work_struct *work)
{
	int value = gpio_get_value(sciaps_trigger_gpio);

	if (value == sciaps_trigger_state_candidate) {
		if (sciaps_trigger_state_candidate != sciaps_trigger_state) {
			sciaps_trigger_state = sciaps_trigger_state_candidate;
			printk(KERN_INFO "%s: Trigger State changed to  %d;\n",
				__func__,sciaps_trigger_state);
			sciaps_notify_trigger(sciaps_trigger_state);
		}
	}
	else {
		sciaps_trigger_state_candidate = value;
		schedule_delayed_work(&sciaps_trigger_debounce_work, msecs_to_jiffies(sciaps_trigger_debounce_time_ms));
		return;
	}
	sciaps_trigger_debounce_work_in_progress = 0;

}

static irqreturn_t sciaps_trigger_irq_handler(int irq, void *dev_id)
{
	int trigger_value = gpio_get_value(sciaps_trigger_gpio);

	if (sciaps_trigger_debounce_work_in_progress == 0
			&& trigger_value != sciaps_trigger_state) {
		sciaps_trigger_debounce_work_in_progress = 1;
		sciaps_trigger_state_candidate = trigger_value;
		schedule_delayed_work(&sciaps_trigger_debounce_work, msecs_to_jiffies(sciaps_trigger_debounce_time_ms));
	}

	return IRQ_NONE; //IRQ_HANDLED;
}

#define SCIAPS_TRIGGER_GPIO_CLEAN_BIT_REMOVE_FILES	0x01
#define SCIAPS_TRIGGER_GPIO_CLEAN_BIT_UNEXPORT		0x02
#define SCIAPS_TRIGGER_GPIO_CLEAN_ALL				0xff

static void sciaps_sysfs_clean(void);
static void trigger_gpio_clean(struct i2c_client *client, uint8_t mask)
{
	dev_info(&client->dev, "%s: Enter\n", __func__);

	sciaps_sysfs_clean();

	if (sciaps_trigger_irq != -1) {
		free_irq(sciaps_trigger_irq, 0);
		sciaps_trigger_irq = -1;
	}
	if (sciaps_trigger_gpio != -1) {
		if (SCIAPS_TRIGGER_GPIO_CLEAN_BIT_REMOVE_FILES == (mask & SCIAPS_TRIGGER_GPIO_CLEAN_BIT_REMOVE_FILES)) {
			struct gpio_chip* sciaps_trigger_gpio_chip = gpio_to_chip(sciaps_trigger_gpio);
			if (sciaps_trigger_gpio_chip) {
				device_remove_file(sciaps_trigger_gpio_chip->dev, &dev_attr_sciaps_trigger);
			}

		}
		if (SCIAPS_TRIGGER_GPIO_CLEAN_BIT_UNEXPORT == (mask & SCIAPS_TRIGGER_GPIO_CLEAN_BIT_UNEXPORT)) {
			gpio_unexport(sciaps_trigger_gpio);
		}
		gpio_free(sciaps_trigger_gpio);
		sciaps_trigger_gpio = -1;
	}
}

struct sciaps_attr {
    struct attribute attr;
    int value;
};

static struct sciaps_attr trigger_debounce_time_ms = {
    .attr.name="trigger-debounce-ms",
    .attr.mode = 0666,
    .value = SCIAPS_TRIGGER_DEBOUNCE_TIME_MS_DEFAULT,
};

static struct sciaps_attr trigger = {
    .attr.name="trigger",
    .attr.mode = 0444,
    .value = SCIAPS_TRIGGER_STATE_DEFAULT,
};

static struct attribute * sciaps_attrs[] = {
    &trigger.attr,
    &trigger_debounce_time_ms.attr,
    NULL
};

static ssize_t sciaps_attr_show(struct kobject *kobj, struct attribute *attr, char *buf)
{
    struct sciaps_attr *sa = container_of(attr, struct sciaps_attr, attr);

	if (sa == &trigger)
		sa->value = sciaps_trigger_state;
	else if (sa == &trigger_debounce_time_ms)
		sa->value = sciaps_trigger_debounce_time_ms;
	else
		sa->value = -1;
    pr_info( "%s: show called (%s). Value: %d;\n", __func__, sa->attr.name, sa->value);

	return scnprintf(buf, PAGE_SIZE, "%d\n", sa->value);
}

static struct kobject *mykobj;

static void sciaps_notify_trigger(int value)
{
    struct sciaps_attr *sa = &trigger;

	sa->value = value;

    sysfs_notify(mykobj, NULL, "trigger");
}

static ssize_t sciaps_attr_store(struct kobject *kobj, struct attribute *attr, const char *buf, size_t len)
{
    struct sciaps_attr *sa = container_of(attr, struct sciaps_attr, attr);
	int value = -1;

    sscanf(buf, "%d", &value);

    pr_info("%s: store called (%s). Value: %d/%d (old/new)\n", __func__, sa->attr.name, sa->value, value);

	if (sa == &trigger_debounce_time_ms) {
		trigger_debounce_time_ms.value = value;
		sciaps_trigger_debounce_time_ms = trigger_debounce_time_ms.value;

	}

    return sizeof(int);
}

static struct sysfs_ops sciaps_sysfs_ops = {
    .show = sciaps_attr_show,
    .store = sciaps_attr_store,
};

static struct kobj_type sciaps_kobj_type = {
    .sysfs_ops = &sciaps_sysfs_ops,
    .default_attrs = sciaps_attrs,
};

static int sciaps_sysfs_init(void)
{
    int err = -1;
    pr_info("%s: init\n", __func__);
    mykobj = kzalloc(sizeof(*mykobj), GFP_KERNEL);
    /* mykobj = kobject_create() is not exported */
    if (mykobj) {
        kobject_init(mykobj, &sciaps_kobj_type);
        if (kobject_add(mykobj, NULL, "%s", "sciaps")) {
             err = -1;
             pr_info("%s: kobject_add() failed\n", __func__);
             kobject_put(mykobj);
             mykobj = NULL;
        }
        err = 0;
    }
    return err;
}

static void sciaps_sysfs_clean(void)
{
    if (mykobj) {
        kobject_put(mykobj);
        kfree(mykobj);
    }
    pr_info("%s: exit\n", __func__);
}

static int trigger_gpio_init(struct i2c_client *client)
{
	int ret;
	uint8_t clean_mask = 0;

	dev_info(&client->dev, "%s: Enter\n", __func__);

	mykobj = 0;

	sciaps_trigger_debounce_work_in_progress = 0;
	sciaps_trigger_state = SCIAPS_TRIGGER_STATE_DEFAULT;
	sciaps_trigger_state_candidate = SCIAPS_TRIGGER_STATE_DEFAULT;
	ret = gpio_is_valid(SCIAPS_TRIGGER_GPIO);
	if (ret) {
		ret = sciaps_sysfs_init();
		if (ret) {
			dev_err(&client->dev, "%s: sciaps_sysfs_init failed with err: %d\n",
					__func__, ret);
			goto trigger_gpio_fail;
		}
		ret = gpio_request(SCIAPS_TRIGGER_GPIO, "sciaps_trigger_gpio");
		if (ret) {
			dev_err(&client->dev, "%s: gpio %d request failed with err: %d\n",
					__func__, SCIAPS_TRIGGER_GPIO, ret);
			goto trigger_gpio_fail;
		}
		else {
			sciaps_trigger_gpio = SCIAPS_TRIGGER_GPIO;
		}
		ret = gpio_direction_input(sciaps_trigger_gpio);
		if (ret) {
			dev_err(&client->dev, "%s: gpio %d gpio_direction_input failed with err: %d\n",
					__func__, sciaps_trigger_gpio, ret);
			goto trigger_gpio_fail;
		}
		ret = gpio_export(sciaps_trigger_gpio, false);
		if (ret) {
			dev_err(&client->dev, "%s: gpio %d gpio_export failed with err: %d\n",
					__func__, sciaps_trigger_gpio, ret);
			goto trigger_gpio_fail;
		}
		clean_mask |= SCIAPS_TRIGGER_GPIO_CLEAN_BIT_UNEXPORT;
		{
			struct gpio_chip* sciaps_trigger_gpio_chip = gpio_to_chip(sciaps_trigger_gpio);
			if (sciaps_trigger_gpio_chip) {
				ret = device_create_file(sciaps_trigger_gpio_chip->dev, &dev_attr_sciaps_trigger);
				if (ret) {
					dev_err(&client->dev, "%s: gpio %d device_create_file failed with err: %d\n",
							__func__, sciaps_trigger_gpio, ret);
					goto trigger_gpio_fail;
				}
				else {
					clean_mask |= SCIAPS_TRIGGER_GPIO_CLEAN_BIT_REMOVE_FILES;
				}
			}
			else {
				dev_err(&client->dev, "%s: gpio %d gpio_to_chip failed\n",
						__func__, sciaps_trigger_gpio);
				goto trigger_gpio_fail;
			}
		}
		ret = request_irq(gpio_to_irq(sciaps_trigger_gpio)
							, (irq_handler_t)sciaps_trigger_irq_handler
							, (IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING )
							, "sciaps_trigger_irq_handler"
							, NULL);
		if (ret) {
			dev_err(&client->dev, "%s: gpio %d request_irq %d failed with err: %d\n",
					__func__, sciaps_trigger_gpio, gpio_to_irq(sciaps_trigger_gpio), ret);
			goto trigger_gpio_fail;
		}
		else {
			sciaps_trigger_irq = gpio_to_irq(sciaps_trigger_gpio);
			INIT_DELAYED_WORK(&sciaps_trigger_debounce_work, sciaps_trigger_debounce_work_func);
		}

	}
	else {
		dev_err(&client->dev, "%s: Invalid gpio %d\n", __func__,
					sciaps_trigger_gpio);
		goto trigger_gpio_fail;
	}

	return 0;

trigger_gpio_fail:
	trigger_gpio_clean(client, clean_mask);

	return ret;
}

static struct i2c_client *sciaps_micro_i2c_client = NULL;

static int sciaps_micro_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct sciaps_micro_data *sciaps_micro;

	sciaps_micro = kzalloc(sizeof(struct sciaps_micro_data), GFP_KERNEL);
	if (!sciaps_micro) {
		dev_err(&client->dev, "cannot allocate memory\n");
		return -ENOMEM;
	}

	sciaps_micro->client = client;
	sciaps_micro->read_reg = 0;
	sciaps_micro->read_reg_value = 0;
	sciaps_micro->nvram_unlocked = 0;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "i2c not supported\n");
		return -EPFNOSUPPORT;
	}

	i2c_set_clientdata(client, sciaps_micro);

	sciaps_micro_i2c_client = client;

	sciaps_micro_setup_sysfs(client);

	trigger_gpio_init(client);

	dev_info(&client->dev, "device probed\n");

	return 0;
}
static int sciaps_micro_remove(struct i2c_client *client)
{
	struct sciaps_micro_data *sciaps_micro;

	sciaps_micro = i2c_get_clientdata(client);

	sciaps_micro_delete_sysfs(client);

	trigger_gpio_clean(client, SCIAPS_TRIGGER_GPIO_CLEAN_ALL);

	i2c_unregister_device(client);

	kfree(sciaps_micro);

	return 0;
}

static void do_sciaps_msm_poweroff(void);
static void sciaps_micro_shutdown(struct i2c_client *client)
{
	int err;
	dev_dbg(&client->dev, "%s:%d\n", __func__, __LINE__);

	pr_notice("%s: Shutting down... System state: %d\n", __func__, system_state);

	if (system_state == SYSTEM_POWER_OFF) {
		pr_notice("%s: Powering off the SoM via Sciaps Ctrl...\n", __func__);
		err = sciaps_micro_i2c_write(client, sciaps_micro_known_cmds[0].buf, 3);
		if (err < 0) {
			dev_err(&client->dev, "%s: could not send poweroff command to PIC\n", __func__);
		}
		else {
			pr_notice("%s: Sciaps ctrl has been notified!\n", __func__);
			pm_power_off = do_sciaps_msm_poweroff;
		}
	}

}

static void do_sciaps_msm_poweroff(void)
{
	pr_notice("%s: Powering off the SoC\n", __func__);

	qpnp_pon_system_pwr_off(PON_POWER_OFF_SHUTDOWN);

	mdelay(10000);
	pr_err("%s: Powering off has failed\n", __func__);
	return;
}

static struct i2c_device_id sciaps_micro_idtable[] = {
	{ "sciaps_micro", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, sciaps_micro_idtable);

static struct i2c_driver sciaps_micro_driver = {
	.driver = {
		.name   = "sciaps_micro",
	},

	.id_table   = sciaps_micro_idtable,
	.probe      = sciaps_micro_probe,
	.remove     = sciaps_micro_remove,
	.shutdown   = sciaps_micro_shutdown,
};

module_i2c_driver(sciaps_micro_driver);

int sciaps_micro_read_register(uint8_t reg, uint16_t *value_out)
{
	struct i2c_client* client = sciaps_micro_i2c_client;
	int err;
	uint16_t value;

	err = sciaps_micro_i2c_read_reg(client, reg, &value);
	if(err < 0) {
		pr_err("%s: error reading from i2c bus: 0x%x", __func__, err);
	}
	else {
		pr_info("%s: Register 0x%x is 0x%x\n", __func__, reg, value);
		if (value_out)
		   *value_out = value;
	}

	return err;
}
EXPORT_SYMBOL(sciaps_micro_read_register);

int sciaps_micro_check_battery_presence(void)
{
	uint16_t value;
	int rc;

	if ((rc = sciaps_micro_read_register(0x40, &value)) < 0) {
		// log if needed.
	}
	else {
		if((value&SCIAPS_MICRO_BATTERY_NOT_PRESENT) == SCIAPS_MICRO_BATTERY_NOT_PRESENT) {
			rc = SCIAPS_MICRO_BATTERY_NOT_PRESENT;
		}
		else {
			rc = SCIAPS_MICRO_BATTERY_PRESENT;
		}
	}
	return rc;
}
EXPORT_SYMBOL(sciaps_micro_check_battery_presence);

MODULE_AUTHOR("Paul Soucy <paul@dev-smart.com>");
MODULE_DESCRIPTION("Sciaps Power Board Micro I2C client driver");
MODULE_LICENSE("GPL");
