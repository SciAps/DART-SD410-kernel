/*
 * Gas Gauge driver for SBS Compliant Batteries
 *
 * Copyright (c) 2016, Sciaps
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/power_supply.h>
#include <linux/i2c.h>
#include <linux/slab.h>

#if !defined(__devinit)
#define __devinit
#endif

#if !defined(__devexit)
#define __devexit
#endif

#if !defined(__devexit_p)
#define __devexit_p(x) (&(x))
#endif

#define DEV_DBG dev_dbg

#define LTC4100_CHARGER_STATUS			0x13 /* Charger Status*/
#define LTC4100_CHARGER_ALARM			0x16 /* Alarm Warning*/

/* bits in LTC4100_CHARGER_STATUS */
#define LTC4100_CHARGER_STATUS_AC_PRESENT	0x8000
/* bits in LTC4100_CHARGER_ALARM */
#define LTC4100_CHARGER_ALARM_OVER_CHARGED		0x8000
#define LTC4100_CHARGER_ALARM_TERMINATE_CHARGE	0x4000
#define LTC4100_CHARGER_ALARM_RESERVED			0x2000
#define LTC4100_CHARGER_ALARM_OVER_TEMP			0x1000

static enum power_supply_property smb_ltc4100_properties[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
	//POWER_SUPPLY_PROP_HEALTH,
	//POWER_SUPPLY_PROP_TYPE,
	//POWER_SUPPLY_PROP_STATUS,
	//POWER_SUPPLY_PROP_CHARGING_ENABLED,
};

static char *battery_supplied_to[] = {
	"main-battery-02",
};

struct smb_ltc4100_battery_reg_cache {
	bool present;
	s16 status;
};

struct smb_ltc4100_info {
	struct i2c_client	*client;
	struct power_supply	power_supply;
	struct smb_ltc4100_battery_reg_cache reg_cache;
	struct delayed_work	work;
	uint8_t		power_supply_registered;
	struct mutex lock;
};

static int i2c_read_errs = 0;

static int smb_ltc4100_read_word_data(struct i2c_client *client, u8 address, s16* out)
{
	s32 ret = i2c_smbus_read_word_data(client, address);

	if (ret < 0) {
		DEV_DBG(&client->dev,
			"%s: i2c read at address 0x%x failed: %d\n",
			__func__, address, ret);
	} else {
		*out = (s16)(0xFFFF & ret);
	}

	return ret;
}

static int smb_ltc4100_charger_online(struct smb_ltc4100_info *chip, union power_supply_propval *val)
{

	if (!chip->reg_cache.present) {
		val->intval = 0;
		return 0;
	}

	if (chip->reg_cache.status & LTC4100_CHARGER_STATUS_AC_PRESENT)
		val->intval = 1;
	else
		val->intval = 0;

	return 0;
}


static int smb_ltc4100_get_property(struct power_supply *psy,
	enum power_supply_property psp,
	union power_supply_propval *val)
{
	int ret = 0;
	struct smb_ltc4100_info *chip = container_of(psy,
				struct smb_ltc4100_info, power_supply);

  struct i2c_client *client = chip->client;

	mutex_lock(&chip->lock);

	switch(psp) {

		case POWER_SUPPLY_PROP_PRESENT:
			val->intval = chip->reg_cache.present ? 1 : 0;
			break;

		case POWER_SUPPLY_PROP_ONLINE:
			ret = smb_ltc4100_charger_online(chip, val);
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

	mutex_unlock(&chip->lock);

	if(ret >= 0) {
		DEV_DBG(&client->dev,
				"%s: property = %d, value = 0x%x\n"
						, __func__
						, psp
						, val->intval);
	}

	return ret;
}


static void smb_ltc4100_external_power_changed(struct power_supply *psy)
{
	struct smb_ltc4100_info *chip = container_of(psy, struct smb_ltc4100_info, power_supply);
	struct i2c_client *client = chip->client;

	dev_err(&client->dev, "%s", __func__);

	i2c_read_errs = 0;
}

#define UNKNOWN			0
#define ACTIVE			1
#define PASSIVE			2

static uint8_t g_active = UNKNOWN;
static struct smb_ltc4100_info* g_chip = NULL;

void deactivate_ltc4100(void)
{
	if (g_active != PASSIVE) {
		g_active = PASSIVE;
		if (g_chip && g_chip->power_supply_registered) {
			power_supply_unregister(&g_chip->power_supply);
			g_chip->power_supply_registered = 0;
		}
		cancel_delayed_work(&g_chip->work);
	}
}

void activate_ltc4100(void)
{
	int rc;
	if (g_active != ACTIVE && g_chip) {
		g_active = ACTIVE;
		if (g_chip->power_supply_registered) {
			power_supply_unregister(&g_chip->power_supply);
			g_chip->power_supply_registered = 0;
		}
		rc = power_supply_register(&g_chip->client->dev, &g_chip->power_supply);
		if (rc) {
			dev_err(&g_chip->client->dev,
				"%s: Failed to register power supply\n", __func__);
		}
		else {
			g_chip->power_supply_registered = 1;
			schedule_delayed_work(&g_chip->work, 2*HZ);
		}
	}
}

static void smb_ltc4100_delayed_work(struct work_struct *work)
{
	#define CHECK_READ_WORD(reg, location, min, max) \
	ret = smb_ltc4100_read_word_data(chip->client, reg, &ival); \
	if(ret >= 0) { \
		DEV_DBG(&chip->client->dev, "read " #reg " = 0x%x(%d)\n", ival, ival); \
		if((max == 0 && min == 0) || (ival >= min && ival < max)) { \
			location = ival; \
		}	\
	}

	// Roughly 60 seconds at 2s poll freq
	#define READ_ERR_LIMIT 30

	struct smb_ltc4100_info *chip;
	bool changed = false;
	struct smb_ltc4100_battery_reg_cache cache;
	s32 ret;
	s16 ival;

	chip = container_of(work, struct smb_ltc4100_info, work.work);

	mutex_lock(&chip->lock);

	cache = chip->reg_cache;

	mutex_unlock(&chip->lock);

	CHECK_READ_WORD(LTC4100_CHARGER_STATUS, cache.status, 0, 0)
	cache.present = ret >= 0;
	if (cache.present) {
		i2c_read_errs = 0;
	}
	else {
		if (i2c_read_errs <= READ_ERR_LIMIT) {
			if (i2c_read_errs == READ_ERR_LIMIT)
				dev_err(&chip->client->dev,
					"%s: Too many i2c errors.  Possibly entering slow poll mode\n", __func__);
			i2c_read_errs++;
		}
	}

	mutex_lock(&chip->lock);

	changed = memcmp(&chip->reg_cache, &cache, sizeof(struct smb_ltc4100_battery_reg_cache));
	if (changed) {
		chip->reg_cache = cache;
	}

	mutex_unlock(&chip->lock);

	if (changed) {
		power_supply_changed(&chip->power_supply);
	}

	if (i2c_read_errs > READ_ERR_LIMIT) {
		//schedule_delayed_work(&chip->work, 10*60*HZ);
		schedule_delayed_work(&chip->work, 2*HZ);
	}
	else {
		schedule_delayed_work(&chip->work, 2*HZ);
	}
}

static int __devinit smb_ltc4100_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	struct smb_ltc4100_info *chip;
	//int rc;

	chip = kzalloc(sizeof(struct smb_ltc4100_info), GFP_KERNEL);
	if (!chip) {
		return  -ENOMEM;
	}

	g_chip = chip;
	chip->client = client;
	chip->power_supply.name = "dc";//name; //"ie-smbus-battery"
	chip->power_supply.type = POWER_SUPPLY_TYPE_MAINS;
	chip->power_supply.properties = smb_ltc4100_properties;
	chip->power_supply.num_properties = ARRAY_SIZE(smb_ltc4100_properties);
	chip->power_supply.get_property = smb_ltc4100_get_property;
	chip->power_supply.external_power_changed = smb_ltc4100_external_power_changed;
	chip->power_supply.supplied_to = battery_supplied_to;
	chip->power_supply.num_supplicants = ARRAY_SIZE(battery_supplied_to);

	chip->reg_cache.present = false;

	mutex_init(&chip->lock);

	i2c_set_clientdata(client, chip);

	INIT_DELAYED_WORK(&chip->work, smb_ltc4100_delayed_work);

	//rc = power_supply_register(&client->dev, &chip->power_supply);
	//if (rc) {
	//	dev_err(&client->dev,
	//		"%s: Failed to register power supply\n", __func__);
	//	goto exit_psupply;
	//}

	dev_info(&client->dev,
		"%s: LTC4100 charger device registered\n", client->name);

	//schedule_delayed_work(&chip->work, HZ);

	return 0;
}

static int __devexit smb_ltc4100_remove(struct i2c_client *client)
{
	struct smb_ltc4100_info *chip = i2c_get_clientdata(client);

	g_chip = NULL;
	cancel_delayed_work_sync(&chip->work);
	if (chip->power_supply_registered) {
		power_supply_unregister(&chip->power_supply);
		chip->power_supply_registered = 0;
	}
	mutex_destroy(&chip->lock);
	//kfree(chip->power_supply.name);
	kfree(chip);
	chip = NULL;

	return 0;
}

static const struct i2c_device_id smb_ltc4100_id[] = {
	{ "smb-ltc4100-charger", 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, smb_ltc4100_id);

static struct i2c_driver smb_ltc4100_charger_driver = {
	.probe		= smb_ltc4100_probe,
	.remove		= __devexit_p(smb_ltc4100_remove),
	.id_table	= smb_ltc4100_id,
	.driver = {
		.name	= "smb-ltc4100-charger",
	},
};
module_i2c_driver(smb_ltc4100_charger_driver);

MODULE_AUTHOR("Andre Doudkin <adoudkin@sciaps.com>");
MODULE_DESCRIPTION("SMBus LTC4100 charger driver");
MODULE_LICENSE("GPL");
