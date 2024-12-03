/*
 * LTC1760 (Charger)
 *
 * Based on bq27x00_battery.c:
 * Copyright (C) 2008 Rodolfo Giometti <giometti@linux.it>
 * Copyright (C) 2008 Eurotech S.p.A. <info@eurotech.it>
 * Copyright (C) 2010-2011 Lars-Peter Clausen <lars@metafoo.de>
 * Copyright (C) 2011 Pali Rohár <pali.rohar@gmail.com
 *
 * Based on ltc1760-charger.c:
 *		Steve Schfter <steve@scheftech.com>
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 */

/*
 * This driver reads battery information from the ND2054 batteries through
 * the LTC1760 charger.  Although the LTC1760 maintains information about
 * the batteries which can be read over the I2C bus, its information is
 * insufficient for reporting purposes.
 */

#include <linux/module.h>
#include <linux/param.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/power_supply.h>
#include <linux/idr.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/debugfs.h>
#include <asm/unaligned.h>
#include <linux/power/smb-ad-ltc1760-charger.h>

#define LTC1760_DEBUG 0

#define DRIVER_VERSION			"2.0.0"

#define LTC1760_REG_STATE			0x01 /* BatterySystemState */
#define LTC1760_REG_STATE_CONT		0x02 /* BatterySystemStateCont */

#define LTC1760_CHARGER_STATE_CONT_AC_PRESENT	0x0001

#define LTC1760_CHARGER_SELECT_BATTERY1			0x1000
#define LTC1760_CHARGER_SELECT_BATTERY2			0x2000


/*
 * For simplicity, a single ltc1760 device is supported.  If support for
 * multiple LTC1760 devices was required, this would have to be an array of
 * devices or a structure allocated in the ltc1760_charger_probe() routine.
 *
 * Note that there is an association between LTC1760 charger devices and
 * devices corresponding to the batteries.  In order for the battery driver to
 * send an I2C message to the battery, it must call ltc1760_select_battery()
 * to connect the host I2C bus to the appropriate battery.  If support for
 * multiple LTC1760 devices is added, a way of mapping batteries to the
 * charger that they are connected to will be needed.
 */

static enum power_supply_property smb_ad_ltc1760_properties[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
	//POWER_SUPPLY_PROP_HEALTH,
	//POWER_SUPPLY_PROP_TYPE,
	//POWER_SUPPLY_PROP_STATUS,
	//POWER_SUPPLY_PROP_CHARGING_ENABLED,
};

static char *battery_supplied_to[] = {
	SMB_AD_LTC1760_BATTERY_SUPPLIED_TO,
};

struct smb_ad_ltc1760_battery_reg_cache {
	bool present;
	s16 state;
	s16 state_cont;
};

struct smb_ad_ltc1760_device_info {
	struct device *dev;
	struct power_supply	power_supply;
	struct smb_ad_ltc1760_battery_reg_cache reg_cache;
	struct delayed_work	work;
	uint8_t		power_supply_registered;
	struct mutex lock;
	struct mutex i2clock;
} ltc_device_info;

static int i2c_read_errs = 0;

/*
 * Read a 16 bit value from an I2C register in the LTC1760.
 */
static int smb_ad_ltc1760_read(struct smb_ad_ltc1760_device_info *di, u8 reg)
{
	struct i2c_client *client = to_i2c_client(di->dev);
	struct i2c_msg msg[2];
	unsigned char data[2];
	int ret;

	if (!client->adapter)
		return -ENODEV;

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].buf = &reg;
	msg[0].len = sizeof(reg);
	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].buf = data;
	msg[1].len = 2;		/* all registers are 16 bits wide */

	mutex_lock(&di->i2clock);
	ret = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
	mutex_unlock(&di->i2clock);

	if (ret < 0) {
		dev_err(di->dev,
			"%s: %s: i2c_transfer failed: %d\n", __func__, client->name, ret);
		return ret;
	}

	ret = get_unaligned_le16(data);

	return ret;
}

/*
 * Write a 16 bit value into an I2C register in the LTC1760.
 */
static int smb_ad_ltc1760_write(struct smb_ad_ltc1760_device_info *di, u8 reg, u16 val)
{
	struct i2c_client *client = to_i2c_client(di->dev);
	struct i2c_msg msg[1];
	unsigned char data[3];
	int ret;

	if (!client->adapter)
		return -ENODEV;

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	data[0] = reg;
	data[1] = val & 0x00FF;
	data[2] = (val & 0xFF00) >> 8;
	msg[0].len = 3; /* 1 reg addr + 2 cmd bytes */
	msg[0].buf = data;

	mutex_lock(&di->i2clock);
	ret = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
	mutex_unlock(&di->i2clock);

	return ret;
}

static int smb_ad_ltc1760_get_property(struct power_supply *psy,
	enum power_supply_property psp,
	union power_supply_propval *val)
{
	int ret = 0;
	struct smb_ad_ltc1760_battery_reg_cache cache;
	struct smb_ad_ltc1760_device_info *di = container_of(psy,
				struct smb_ad_ltc1760_device_info, power_supply);

	struct i2c_client *client = to_i2c_client(di->dev);

	mutex_lock(&di->lock);
	cache = di->reg_cache;
	mutex_unlock(&di->lock);


	switch(psp) {

		case POWER_SUPPLY_PROP_PRESENT:
			val->intval = cache.present ? 1 : 0;
			break;

		case POWER_SUPPLY_PROP_ONLINE:
			val->intval = 0;
			if (cache.present) {
				if (cache.state_cont & LTC1760_CHARGER_STATE_CONT_AC_PRESENT)
					val->intval = 1;
			}
			break;

		//case POWER_SUPPLY_PROP_STATUS:
		//	val->intval = POWER_SUPPLY_STATUS_CHARGING;
		//	break;

		//case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		//	val->intval = 1;
		//	break;

		default:
			dev_err(&client->dev, "unknown psp: %d\n", psp);
			ret = -EINVAL;
	}


	if(ret >= 0) {
		dev_dbg(&client->dev,
				"%s: property = %d, value = 0x%x\n"
						, __func__
						, psp
						, val->intval);
	}

	return ret;
}

static void smb_ad_ltc1760_external_power_changed(struct power_supply *psy)
{
#if LTC1760_DEBUG
	struct smb_ad_ltc1760_device_info *di = container_of(psy,
				struct smb_ad_ltc1760_device_info, power_supply);
	struct i2c_client *client = to_i2c_client(di->dev);

	dev_info(&client->dev, "%s: --------------\n", __func__);
#endif
	i2c_read_errs = 0;
}

static void smb_ad_ltc1760_delayed_work(struct work_struct *work)
{
	// Roughly 60 seconds at 2s poll freq
	#define READ_ERR_LIMIT 30

	struct smb_ad_ltc1760_device_info *di = container_of(work,
				struct smb_ad_ltc1760_device_info, work.work);

	struct i2c_client *client = to_i2c_client(di->dev);
	bool changed = false;
	struct smb_ad_ltc1760_battery_reg_cache cache;

	mutex_lock(&di->lock);

	cache = di->reg_cache;

	mutex_unlock(&di->lock);

	cache.state  = smb_ad_ltc1760_read(&ltc_device_info, LTC1760_REG_STATE);

	if (cache.state < 0) {
		dev_err(&client->dev,
				"%s: %s: i2c_transfer for register %d failed: %d\n", __func__, client->name, LTC1760_REG_STATE, cache.state);
	}
#if LTC1760_DEBUG
	else {
		dev_info(&client->dev,
				"%s: %s: state: 0x%x\n", __func__, client->name, cache.state);
	}
#endif

	cache.state_cont = smb_ad_ltc1760_read(&ltc_device_info, LTC1760_REG_STATE_CONT);

	if (cache.state_cont < 0) {
		dev_err(&client->dev,
				"%s: %s: i2c_transfer for register %d failed: %d\n", __func__, client->name, LTC1760_REG_STATE_CONT, cache.state_cont);
	}
#if LTC1760_DEBUG
	else {
		dev_dbg(&client->dev,
				"%s: %s: BatterySystemStateCont: 0x%x\n", __func__, client->name, cache.state_cont);
	}
#endif
	cache.present = (cache.state_cont >= 0);
	if (cache.present) {
		i2c_read_errs = 0;
	}
	else {
		if (i2c_read_errs <= READ_ERR_LIMIT) {
			if (i2c_read_errs == READ_ERR_LIMIT)
				dev_err(&client->dev,
					"%s: Too many i2c errors.  Possibly entering slow poll mode\n", __func__);
			i2c_read_errs++;
		}
	}

	if (cache.state_cont >= 0 && cache.state >= 0) {
		mutex_lock(&di->lock);

		changed = memcmp(&di->reg_cache, &cache, sizeof(struct smb_ad_ltc1760_battery_reg_cache));
		if (changed) {
			di->reg_cache = cache;
		}

		mutex_unlock(&di->lock);
	}
	if (changed) {
		power_supply_changed(&di->power_supply);
	}

	if (i2c_read_errs > READ_ERR_LIMIT) {
		schedule_delayed_work(&di->work, 10*HZ);
	}
	else {
		schedule_delayed_work(&di->work, 1*HZ);
	}
}


int smb_ad_ltc1760_ac_present(void)
{
	struct smb_ad_ltc1760_device_info *di = &ltc_device_info;
	s16 present, state_cont;

	mutex_lock(&di->lock);
	present		= di->reg_cache.present;
	state_cont	= di->reg_cache.state_cont;
	mutex_unlock(&di->lock);

	return (present && (state_cont & LTC1760_CHARGER_STATE_CONT_AC_PRESENT));
}
EXPORT_SYMBOL_GPL(smb_ad_ltc1760_ac_present);


/*
 * Select one of the two batteries connected to the LTC1760.  This connects
 * the host I2C bus to either I2C B1 (battery 1 I2C bus) or I2C B2
 * (battery 2 I2C bus).  The host can then access the battery at the I2C
 * address defined in the Smart Battery specification.
 */
int smb_ad_ltc1760_select_battery(u8 battery_num)
{
	struct i2c_client *client = to_i2c_client(ltc_device_info.dev);
	u16 val;

	if (ltc_device_info.dev == NULL)	// not yet probed
		return -EINVAL;

	if (battery_num > 1)
		return -EINVAL;

	if (!client->adapter)
		return -ENODEV;

	val = battery_num ? LTC1760_CHARGER_SELECT_BATTERY2 : LTC1760_CHARGER_SELECT_BATTERY1;

#if LTC1760_DEBUG
	{
		int state =
				smb_ad_ltc1760_read(&ltc_device_info, LTC1760_REG_STATE);

		if (state < 0) {
			dev_err(&client->dev,
				"%s: %s: i2c_transfer for register %d failed: %d\n", __func__, client->name, LTC1760_REG_STATE, state);
		}
		else {
			dev_dbg(&client->dev,
				"%s: BatterySystemState: 0x%x\n", client->name, state);
		}

		state =
				smb_ad_ltc1760_read(&ltc_device_info, LTC1760_REG_STATE_CONT);

		if (state < 0) {
			dev_err(&client->dev,
				"%s: %s: i2c_transfer for register %d failed: %d\n", __func__, client->name, LTC1760_REG_STATE_CONT, state);
		}
		else {
			dev_dbg(&client->dev,
				"%s: BatterySystemStateCont: 0x%x\n", client->name, state);
		}
	}

	dev_dbg(&client->dev,
		"%s: %s: Switching to battery %d ...\n", __func__, client->name, battery_num);
#endif

	return smb_ad_ltc1760_write(&ltc_device_info, LTC1760_REG_STATE, val);
}
EXPORT_SYMBOL_GPL(smb_ad_ltc1760_select_battery);

static int smb_ad_ltc1760_charger_probe(struct i2c_client *client,
				 const struct i2c_device_id *id)
{
	int rc = 0;
	struct smb_ad_ltc1760_device_info *di = &ltc_device_info;

	//if (ltc_device_info.dev != NULL)
	//	return -EBUSY;

	di->dev = &client->dev;
	di->power_supply.dev = &client->dev;
	di->power_supply.name = SMB_AD_LTC1760_CHARGER_NAME;
	di->power_supply.type = POWER_SUPPLY_TYPE_MAINS;
	di->power_supply.properties = smb_ad_ltc1760_properties;
	di->power_supply.num_properties = ARRAY_SIZE(smb_ad_ltc1760_properties);
	di->power_supply.get_property = smb_ad_ltc1760_get_property;
	di->power_supply.external_power_changed = smb_ad_ltc1760_external_power_changed;
	di->power_supply.supplied_to = battery_supplied_to;
	di->power_supply.num_supplicants = ARRAY_SIZE(battery_supplied_to);

	di->reg_cache.present = false;

	mutex_init(&di->lock);
	mutex_init(&di->i2clock);

	i2c_set_clientdata(client, di);

	rc = power_supply_register(&client->dev, &di->power_supply);
	if (rc) {
		dev_err(&client->dev,
			"%s: Failed to register power supply\n", __func__);
	}
	else {
		INIT_DELAYED_WORK(&di->work, smb_ad_ltc1760_delayed_work);
		schedule_delayed_work(&di->work, 1*HZ);
		di->power_supply_registered = 1;
		dev_info(&client->dev,
			"%s: %s: LTC1760 charger device registered.\n", __func__, client->name);
	}

	return rc;
}

static int smb_ad_ltc1760_charger_remove(struct i2c_client *client)
{
	struct smb_ad_ltc1760_device_info *di = i2c_get_clientdata(client);

	if (di->power_supply_registered) {
		cancel_delayed_work_sync(&di->work);
		power_supply_unregister(&di->power_supply);
		di->power_supply_registered = 0;
	}
	di->dev = NULL;
	mutex_destroy(&di->lock);
	mutex_destroy(&di->i2clock);

	return 0;
}

static const struct i2c_device_id smb_ad_ltc1760_id[] = {
	{ "ltc1760", 0 },
	{},
};
MODULE_DEVICE_TABLE(i2c, smb_ad_ltc1760_id);

static struct i2c_driver smb_ad_ltc1760_charger_driver = {
	.driver = {
		.name = "smb-ad-ltc1760-charger",
	},
	.probe = smb_ad_ltc1760_charger_probe,
	.remove = smb_ad_ltc1760_charger_remove,
	.id_table = smb_ad_ltc1760_id,
};

static inline int smb_ad_ltc1760_charger_i2c_init(void)
{
	int ret = i2c_add_driver(&smb_ad_ltc1760_charger_driver);
	if (ret)
		printk(KERN_ERR "Unable to register LTC1760 i2c driver\n");

	return ret;
}

static inline void smb_ad_ltc1760_charger_i2c_exit(void)
{
	i2c_del_driver(&smb_ad_ltc1760_charger_driver);
}

static int __init smb_ad_ltc1760_charger_init(void)
{
	int ret;

	ret = smb_ad_ltc1760_charger_i2c_init();

	return ret;
}
module_init(smb_ad_ltc1760_charger_init);

static void __exit smb_ad_ltc1760_charger_exit(void)
{
	smb_ad_ltc1760_charger_i2c_exit();
}
module_exit(smb_ad_ltc1760_charger_exit);

MODULE_AUTHOR("Andre Doudkin <adoudkin@gmail.com>");
MODULE_DESCRIPTION("LTC1760 charger driver");
MODULE_LICENSE("GPL");
