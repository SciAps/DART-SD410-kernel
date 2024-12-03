/*
 * NH2054/NH3054 (Battery) driver
 *
 * Based on bq27x00_battery.c:
 * Copyright (C) 2008 Rodolfo Giometti <giometti@linux.it>
 * Copyright (C) 2008 Eurotech S.p.A. <info@eurotech.it>
 * Copyright (C) 2010-2011 Lars-Peter Clausen <lars@metafoo.de>
 * Copyright (C) 2011 Pali Rohár <pali.rohar@gmail.com>
 *
 * Based on nd2054_battery.c:
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
 * This driver reads battery information from the NH2054/NH3054 batteries through
 * the LTC1760 charger.  Although the LTC1760 maintains information about
 * the batteries which can be read over the I2C bus, its information is
 * insufficient for reporting purposes.
 *
 * This driver's involvement with the LTC1760 is limited to the use of
 * smb_ad_ltc1760_select_battery() to select which NH2054/NH3054 battery is connected
 * to the host's I2C bus.
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
#include <linux/qpnp/power-on.h>

#define NHx054_INDIVIDUAL_BAT_PS	0

#define DRIVER_VERSION			"2.0.0"

#define NHx054_REG_TTF			0x12 /* Average time to full */
#define NHx054_REG_TTE			0x13 /* Average time to empty */
#define NHx054_REG_TEMP			0x08 /* Temperature */
#define NHx054_REG_VOLT			0x09 /* Voltage */
#define NHx054_REG_ACURR		0x0b /* Average current */
#define NHx054_REG_RSOC			0x0D /* Relative State-of-Charge */
#define NHx054_REG_REMAIN_CHARGE	0x0F /* reamaining charge capacity */
#define NHx054_REG_FULL_CHARGE		0x10 /* full charge capacity  */
#define NHx054_REG_STATUS		0x16 /* Battery Status */
#define NHx054_REG_CYCT			0x17 /* Cycle count total */
#define NHx054_REG_DCAP			0x18 /* Design capacity */

/* bits in NHx054_REG_STATUS */

#define NHx054_STATUS_OVER_CHARGD	0x8000
#define NHx054_STATUS_OVER_TEMP		0x1000
#define NHx054_STATUS_INIT			0x80
#define NHx054_STATUS_DISCHARGING	0x40
#define NHx054_STATUS_FULL_CHRG		0x20
#define NHx054_STATUS_FULL_DISCHRG	0x10

#define SCIAPS_SUPPORT_SHUTDOWN_DEFAULT				1
#define SCIAPS_SHUTDOWN_TIMER_DEFAULT				10

#define BATTERY_DISCHARGING_THRES_CURR_uA			(-50000)
#define BATTERY_CAPACITY_LOW_THRES					5	// 0.5%
#define BATTERY_CAPACITY_LOW_THRES_MAX				200 // 20%
#define BATTERY_CAPACITY_LOW_THRES_mAh				0	// calculated based on BATTERY_CAPACITY_LOW_THRES
#define BATTERY_VOLTAGE_CRIT_LOW_THRES_mV			12000
#define BATTERY_VOLTAGE_LOW_THRES_mV				13500


#define DEV_DBG dev_dbg
#define DEV_INFO dev_dbg

// Roughly 60 seconds at 2s poll freq
#define READ_ERR_LIMIT 30

//static unsigned int poll_interval = 1;
static int i2c_read_errs = 0;

struct nh2054_device_info;

struct nh2054_reg_cache {
	int _temperature;
	int _time_to_empty;
	int _time_to_full;
	int _charge_full;
	int _charge_remain;
	int _charge_design_full;
	int _cycle_count;
	int _capacity;
	int _status;
};

struct nh2054_reg_cache_ex {
	int _volt;
	int _curr;
	int _capacity_low_thres_uAh;
	int _capacity_crit_low;
};

struct nh2054_device_info {
	struct device 		*dev;

	struct delayed_work work;

	bool battery_present[2];
	struct power_supply	bat_dual;
#if NHx054_INDIVIDUAL_BAT_PS
	struct power_supply	bat[2];
#endif
	struct nh2054_reg_cache _cache[2];
	struct nh2054_reg_cache_ex _cache_ex[2];

	uint8_t		bat_dual_ps_registered;
#if NHx054_INDIVIDUAL_BAT_PS
	uint8_t		bat_ps_registered[2];
#endif
	struct delayed_work shutdown_work;	/* Shutdown workcheduler */

	uint16_t	sciaps_support_shutdown;
	uint16_t	sciaps_shutdown_timer;

	int			capacity_low_thres_uAh;
	uint16_t	capacity_low_thres; // in 0.1 %
	int			voltage_crit_low_thres_uV;
	int			voltage_low_thres_uV;

	uint8_t		shutdown_work_running;

	struct	mutex lock;
	struct	mutex i2clock;
	/* reference to the cooling device */
	struct	thermal_dev *tdev;
};

static enum power_supply_property nh2054_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CAPACITY_LEVEL,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW,
	POWER_SUPPLY_PROP_TIME_TO_FULL_NOW,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
};

#if 0
module_param(poll_interval, uint, 0644);
MODULE_PARM_DESC(poll_interval, "battery poll interval in seconds - " \
				"0 disables polling");
#endif

/*
 * nh2054_read().  Read the indicated register from the battery over the
 * I2C bus.  It is assumed that the appropriate battery as been selected
 * in the LTC charger configuration.
 */
static int nh2054_read(struct nh2054_device_info *di, u8 reg, int *val)
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
		DEV_DBG(&client->dev,
				"%s: %s: i2c_transfer for register %d failed: %d\n", __func__, client->name, reg, ret);
		return ret;
	}

	if (val)
		*val = get_unaligned_le16(data);

	DEV_DBG(&client->dev,
			">>>> AAD >>>> %s:i2c read at address 0x%x success. Value: %d/0x%x\n",
			__func__, reg, ret, ret);

	return ret;
}

#if 0
The host does not currently write to any battery registers.
/*
 * nh2054_write().  Write to the indicated register from the battery over the
 * I2C bus.  It is assumed that the appropriate battery as been selected in
 * the LTC charger configuration.
 */
static int nh2054_write(struct nh2054_device_info *di, u8 reg, u16 val)
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

	ret = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));

	return ret;
}
#endif

/*
 * Return the battery Relative State-of-Charge
 * Or < 0 if something fails.
 */
static int nh2054_battery_read_rsoc(struct nh2054_device_info *di, int* val)
{
	int rsoc, rc;

	rc = nh2054_read(di, NHx054_REG_RSOC, &rsoc);

	if (rc < 0)
		DEV_DBG(di->dev, "error reading relative State-of-Charge\n");

	if (val)
		*val = rsoc;

	return rc;
}

/*
 * Return a battery charge value in uAh
 * Or < 0 if something fails.
 */
static int nh2054_battery_read_charge(struct nh2054_device_info *di, u8 reg, int* val)
{
	int charge, rc;

	rc = nh2054_read(di, reg, &charge);

	if (rc < 0) {
		DEV_DBG(di->dev, "error reading charge register %02x: %d\n",
			reg, rc);
	}
	else {
		charge *= 1000;
		if (val)
			*val = charge;
	}

	return rc;
}

/*
 * Return the battery remaining capaciy in uAh
 * Or < 0 if something fails.
 */
static inline int nh2054_battery_remaining_charge(struct nh2054_device_info *di, int* val)
{
	return nh2054_battery_read_charge(di, NHx054_REG_REMAIN_CHARGE, val);
}

/*
 * Return the battery full charge capacity in uAh
 * Or < 0 if something fails.
 */
static inline int nh2054_battery_full_charge(struct nh2054_device_info *di, int* val)
{
	return nh2054_battery_read_charge(di, NHx054_REG_FULL_CHARGE, val);
}

/*
 * Return the battery design capacity in uAh
 * Or < 0 if something fails.
 */
static int nh2054_battery_design_cap(struct nh2054_device_info *di, int* val)
{
	int dcap, rc;

	rc = nh2054_read(di, NHx054_REG_DCAP, &dcap);

	if (rc < 0) {
		dev_info(di->dev, "%s: error reading design capacity: %d\n", __func__, rc);
	}
	else {
		dcap *= 1000;
		if (val)
			*val = dcap;
	}

	return rc;
}

/*
 * Return the battery temperature in tenths of degree Celsius
 * Or < 0 if something fails.
 */
static int nh2054_battery_read_temperature(struct nh2054_device_info *di, int* val)
{
	int temp, rc;

	rc = nh2054_read(di, NHx054_REG_TEMP, &temp);

	if (rc < 0) {
		dev_err(di->dev, "%s: error reading temperature: %d\n", __func__, rc);
	}
	else {
		temp -= 2731;	// convert from .1 degrees K
		if (val)
			*val = temp;
	}

	return rc;
}

/*
 * Return the battery Voltage in uVolts
 * Or < 0 if something fails.
 */
static int nh2054_battery_read_voltage(struct nh2054_device_info *di, int* val)
{
	int volt, rc;

	rc = nh2054_read(di, NHx054_REG_VOLT, &volt);

	if (rc < 0) {
		dev_err(di->dev, "%s: error reading voltage: %d\n", __func__, rc);
	}
	else {
		volt *= 1000;
		if (val)
			*val = volt;
	}

	return rc;
}

/*
 * Return the battery average current in uA
 * Note that current can be negative signed as well
 * Or <0 if something fails.
 */
static int nh2054_battery_read_current(struct nh2054_device_info *di, int* val)
{
	int curr, rc;

	rc = nh2054_read(di, NHx054_REG_ACURR, &curr);

	if (rc < 0) {
		dev_err(di->dev, "%s: error reading average current: %d\n", __func__, rc);
	}
	else {
		curr = (int)((s16)curr) * 1000;
		if (val)
			*val = curr;
	}

	return rc;
}

/*
 * Return the battery Cycle count total
 * Or < 0 if something fails.
 */
static int nh2054_battery_read_cyct(struct nh2054_device_info *di, int* val)
{
	int cyct, rc;

	rc = nh2054_read(di, NHx054_REG_CYCT, &cyct);

	if (rc < 0) {
		dev_err(di->dev, "%s: error reading cycle count total: %d\n", __func__, rc);
	}
	else {
		if (val)
			*val = cyct;
	}

	return rc;
}

/*
 * Read a time register.
 * Return < 0 if something fails.
 */
static int nh2054_battery_read_time(struct nh2054_device_info *di, u8 reg, int* val)
{
	int tval, rc;

	rc = nh2054_read(di, reg, &tval);

	if (rc < 0) {
		dev_info(di->dev, "%s: error reading time register %02x: %d\n", __func__, reg, tval);
	}
	else {
		if (tval == 65535) {
			tval = -ENODATA;
		}
		else {
			tval *= 60;
		}
		if (val)
			*val = tval;
	}

	return rc;
}

/*
 * Update the battery information held by the driver.
 */
static bool nh2054_update(struct nh2054_device_info *di, int battery_num)
{
	struct nh2054_reg_cache cache;
	struct nh2054_reg_cache_ex cache_ex;
	bool notify_power_supply = false;
	int status = 0, rc;

	if (smb_ad_ltc1760_select_battery(battery_num) < 0) {
		if (di->battery_present[battery_num] == 1) {
			notify_power_supply = true;
			mutex_lock(&di->lock);
			di->battery_present[battery_num] = 0;
			mutex_unlock(&di->lock);
		}

		return notify_power_supply;
	}

	rc = nh2054_read(di, NHx054_REG_STATUS, &status);

	if (rc >= 0) {
		i2c_read_errs = 0;
		cache._status = status;
		if (!(cache._status & NHx054_STATUS_INIT)) {
			dev_info(di->dev, "battery is not initilized! Ignoring capacity values\n");
		} else {
			int charge_design_full, capacity_low_thres_uAh_curr, capacity_low_thres_uAh_new = 0;
			mutex_lock(&di->lock);
			memcpy(&cache,		&(di->_cache[battery_num]),		sizeof(cache));
			memcpy(&cache_ex,	&(di->_cache_ex[battery_num]),	sizeof(cache_ex));
			mutex_unlock(&di->lock);
			charge_design_full = cache._charge_design_full;
			capacity_low_thres_uAh_curr = cache_ex._capacity_low_thres_uAh;

			nh2054_battery_read_rsoc(di, &cache._capacity);
			nh2054_battery_read_time(di, NHx054_REG_TTE, &cache._time_to_empty);
			nh2054_battery_read_time(di, NHx054_REG_TTF, &cache._time_to_full);
			nh2054_battery_full_charge(di, &cache._charge_full);
			nh2054_battery_remaining_charge(di, &cache._charge_remain);
			nh2054_battery_read_temperature(di, &cache._temperature);
			nh2054_battery_read_cyct(di, &cache._cycle_count);
			nh2054_battery_read_voltage(di, &cache_ex._volt);
			nh2054_battery_read_current(di, &cache_ex._curr);

			/* We only have to read charge design full once */
			if (charge_design_full <= 0) {
				nh2054_battery_design_cap(di, &cache._charge_design_full);
			}

			if ( di->capacity_low_thres != 0
					&& (capacity_low_thres_uAh_curr == 0
						|| charge_design_full != cache._charge_design_full)) {
				capacity_low_thres_uAh_new = cache._charge_full;
				capacity_low_thres_uAh_new *= di->capacity_low_thres;
				capacity_low_thres_uAh_new /= 1000;
			}

			if (capacity_low_thres_uAh_new && cache_ex._capacity_low_thres_uAh != capacity_low_thres_uAh_new) {
				dev_info(di->dev,
							"--!!!--> Battery %d: Updating capacity-low-thres-uAh from %d uAh to %d uAh\n", battery_num, cache_ex._capacity_low_thres_uAh, capacity_low_thres_uAh_new);
				cache_ex._capacity_low_thres_uAh = capacity_low_thres_uAh_new;
			}

			cache_ex._capacity_crit_low = 0;
			if (cache_ex._volt <= di->voltage_crit_low_thres_uV
					|| (cache._charge_remain <= cache_ex._capacity_low_thres_uAh && cache_ex._volt < di->voltage_low_thres_uV)
			   ) {
				//if (cache_ex._curr <= BATTERY_DISCHARGING_THRES_CURR_uA) {
				if (!smb_ad_ltc1760_ac_present()) {
					dev_info(di->dev, "%s: --!!!--> Battery Low Alert!\n", __func__);
					dev_info(di->dev, "%s: --!!!--> capacity: %d uAh; voltage: %d uV; current: %d uA;\n"
							, __func__
							, cache._charge_remain
							, cache_ex._volt
							, cache_ex._curr);
					cache_ex._capacity_crit_low = 1;
				}
			}

			if (di->battery_present[battery_num] == 0) {
				notify_power_supply = true;
			}

			mutex_lock(&di->lock);
			di->battery_present[battery_num] = 1;
			if (memcmp(&(di->_cache[battery_num]),
					&cache, sizeof(cache)) != 0) {
				memcpy(&(di->_cache[battery_num]), &cache, sizeof(cache));
				notify_power_supply = true;
			}
			if (memcmp(&(di->_cache_ex[battery_num]),
					&cache_ex, sizeof(cache_ex)) != 0) {
				memcpy(&(di->_cache_ex[battery_num]), &cache_ex, sizeof(cache_ex));
			}
			mutex_unlock(&di->lock);

#if NHx054_INDIVIDUAL_BAT_PS
			if (di->bat_ps_registered[battery_num] == 0)  {
				int rc = 0;
				dev_info(di->dev, "%s: ---> Registering Power Supply for battery %d...\n", __func__, battery_num);
				if ((rc = power_supply_register(di->dev, &di->bat[battery_num]))) {
					dev_err(di->dev, "%s: fail to register battery %d: %d\n", __func__, battery_num, rc);
				}
				else {
					di->bat_ps_registered[battery_num] = 1;
					dev_info(di->dev, "%s: Power Supply for battery %d registered!\n", __func__, battery_num);
				}
			}
#endif
		}
	}
	else {
		if (i2c_read_errs <= READ_ERR_LIMIT) {
			if (i2c_read_errs == READ_ERR_LIMIT)
				dev_err(di->dev,
					"%s: Too many i2c errors.  Possibly entering slow poll mode\n", __func__);
			i2c_read_errs++;
		}
		if (di->battery_present[battery_num]) {
			notify_power_supply = true;
			mutex_lock(&di->lock);
			di->battery_present[battery_num] = 0;
			mutex_unlock(&di->lock);
		}

#if NHx054_INDIVIDUAL_BAT_PS
		if (di->bat_ps_registered[battery_num] == 1)  {
			dev_info(di->dev, "%s: ---> Unregistering Power Supply for battery %d...\n", __func__, battery_num);
			power_supply_unregister(&di->bat[battery_num]);
			di->bat_ps_registered[battery_num] = 0;
			dev_info(di->dev, "%s: Power Supply for battery %d unregistered!\n", __func__, battery_num);
		}
#endif
	}

	return notify_power_supply;
}

static void shutdown_work_func(struct work_struct *work)
{
	struct nh2054_device_info *di = container_of(work, struct nh2054_device_info, shutdown_work.work);

	//printk(KERN_INFO"%s: ----> Battery Low!!! Powering off...\n", __func__);
	dev_info(di->dev, "%s: ----> Battery Low!!! Powering off...\n", __func__);
	sciaps_device_power_off(SCIAPS_DEVICE_POWER_OFF_OPT_SRC_BatteryLow);
}

static void nh2054_battery_poll(struct work_struct *work)
{
	struct nh2054_device_info *di = container_of(work, struct nh2054_device_info, work.work);
	int bat_num = 0;
	bool notify_power_supply = false;

	/* first battery */
	bat_num = 0;
	if (nh2054_update(di, bat_num))
		notify_power_supply = true;

	DEV_INFO(di->dev, "%s: %d - %d - 0x%.2x - %d - %d - %d - %d/%d/%d - %d - %d - %d - %d\n", __func__, bat_num, di->battery_present[bat_num]
			, di->_cache[bat_num]._status
			, di->_cache[bat_num]._temperature
			, di->_cache[bat_num]._time_to_empty
			, di->_cache[bat_num]._time_to_full
			, di->_cache[bat_num]._charge_remain
			, di->_cache[bat_num]._charge_full
			, di->_cache_ex[bat_num]._capacity_low_thres_uAh
			//, di->_cache[bat_num]._charge_design_full
			, di->_cache[bat_num]._cycle_count
			, di->_cache[bat_num]._capacity
			, di->_cache_ex[bat_num]._volt
			, di->_cache_ex[bat_num]._curr
			);

	/* second battery */
	bat_num = 1;
	if (nh2054_update(di, bat_num))
		notify_power_supply = true;

	DEV_INFO(di->dev, "%s: %d - %d - 0x%.2x - %d - %d - %d - %d/%d/%d - %d - %d - %d - %d\n", __func__, bat_num, di->battery_present[bat_num]
			, di->_cache[bat_num]._status
			, di->_cache[bat_num]._temperature
			, di->_cache[bat_num]._time_to_empty
			, di->_cache[bat_num]._time_to_full
			, di->_cache[bat_num]._charge_remain
			, di->_cache[bat_num]._charge_full
			, di->_cache_ex[bat_num]._capacity_low_thres_uAh
			//, di->_cache[bat_num]._charge_design_full
			, di->_cache[bat_num]._cycle_count
			, di->_cache[bat_num]._capacity
			, di->_cache_ex[bat_num]._volt
			, di->_cache_ex[bat_num]._curr
			);

	{
		int p[2], ccl[2];
		mutex_lock(&di->lock);
		p[0] = di->battery_present[0];
		p[1] = di->battery_present[1];
		ccl[0] = di->_cache_ex[0]._capacity_crit_low;
		ccl[1] = di->_cache_ex[1]._capacity_crit_low;
		mutex_unlock(&di->lock);


		if (p[0] && p[1]) {
			ccl[0] = (ccl[0] && ccl[1]);
		}
		else if (p[0]) {
			//do nothing;
		}
		else if (p[1]) {
			p[0]	= p[1];
			ccl[0] = ccl[1];
		}
		else {
			ccl[0] = 0;
		}

		if (ccl[0]) {
			if (di->shutdown_work_running == 0) {
				dev_info(di->dev, "%s: ----> Battery Low Alert! Shutting down in %d seconds...\n", __func__, di->sciaps_shutdown_timer);
				schedule_delayed_work(&di->shutdown_work, di->sciaps_shutdown_timer*HZ);
				di->shutdown_work_running = 1;
			}
		}
		else {
			if (di->shutdown_work_running) {
				dev_info(di->dev, "%s: ----> Cancelling system shutdown...\n", __func__);
				di->shutdown_work_running = 0;
				cancel_delayed_work_sync(&di->shutdown_work);
			}
		}

	}

	if (notify_power_supply) {
		power_supply_changed(&di->bat_dual);
#if NHx054_INDIVIDUAL_BAT_PS
		{
			int i;
			for (i = 0; i < 2; i++) {
				if (di->bat_ps_registered[i]) {
					power_supply_changed(&di->bat[i]);
				}
			}
		}
#endif
	}

	if (i2c_read_errs > READ_ERR_LIMIT) {
		schedule_delayed_work(&di->work, 2*HZ);
	}
	else {
		schedule_delayed_work(&di->work, HZ);
	}
	//if (poll_interval > 0) {
	//	/* The timer does not have to be accurate. */
	//	set_timer_slack(&di->work.timer, poll_interval * HZ / 4);
	//	schedule_delayed_work(&di->work, poll_interval * HZ);
	//}
}

static int nh2054_battery_status(struct nh2054_device_info *di, int p[],  const struct nh2054_reg_cache c[], union power_supply_propval *val, int bat)
{
	int status = POWER_SUPPLY_STATUS_UNKNOWN;
	int p1, p2, s1, s2;

	p1 = p[0];
	p2 = p[1];
	s1 = c[0]._status;
	s2 = c[1]._status;

	if (bat == 0) {
		p2 = p1;
		s2 = s1;
	}
	else if (bat == 1) {
		p1 = p2;
		s1 = s2;
	}
	else if (p1 == 1 && p2 == 0)
		s2 = s1;
 	else if (p1 == 0 && p2 == 1)
	   s1 = s2;

	if (p1 || p2) {
		if ((s1 & NHx054_STATUS_FULL_CHRG) && (s2 & NHx054_STATUS_FULL_CHRG))
			status = POWER_SUPPLY_STATUS_FULL;
		else if ((s1 & NHx054_STATUS_DISCHARGING) && (s2 & NHx054_STATUS_DISCHARGING))
			status = POWER_SUPPLY_STATUS_DISCHARGING;
		else
			status = POWER_SUPPLY_STATUS_CHARGING;

	}

	val->intval = status;

	return 0;
}

static int nh2054_battery_health(struct nh2054_device_info *di, int p[], const struct nh2054_reg_cache c[], union power_supply_propval *val, int bat)
{
	int status = POWER_SUPPLY_HEALTH_UNKNOWN;
	int p1, p2, s1, s2;

	p1 = p[0];
	p2 = p[1];
	s1 = c[0]._status;
	s2 = c[1]._status;

	if (bat == 0) {
		p2 = p1;
		s2 = s1;
	}
	else if (bat == 1) {
		p1 = p2;
		s1 = s2;
	}
	else if (p1 == 1 && p2 == 0)
		s2 = s1;
 	else if (p1 == 0 && p2 == 1)
	   s1 = s2;

	if (p1 || p2) {
		if ((s1 & NHx054_STATUS_OVER_TEMP) || (s2 & NHx054_STATUS_OVER_TEMP))
			status = POWER_SUPPLY_HEALTH_OVERHEAT;
		else if ((s1 & NHx054_STATUS_OVER_CHARGD) || (s2 & NHx054_STATUS_OVER_CHARGD))
			status = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
		else
			status = POWER_SUPPLY_HEALTH_GOOD;

	}

	val->intval = status;

	return 0;
}

static int nh2054_battery_capacity_level(struct nh2054_device_info *di, int p[],  const struct nh2054_reg_cache c[], union power_supply_propval *val, int bat)
{
	int status = POWER_SUPPLY_CAPACITY_LEVEL_UNKNOWN;
	int p1, p2, s1, s2;

	p1 = p[0];
	p2 = p[1];
	s1 = c[0]._status;
	s2 = c[1]._status;

	if (bat == 0) {
		p2 = p1;
		s2 = s1;
	}
	else if (bat == 1) {
		p1 = p2;
		s1 = s2;
	}
	else if (p1 == 1 && p2 == 0)
		s2 = s1;
 	else if (p1 == 0 && p2 == 1)
	   s1 = s2;

	if (p1 || p2) {
		if ((s1 & NHx054_STATUS_FULL_CHRG) && (s2 & NHx054_STATUS_FULL_CHRG))
			status = POWER_SUPPLY_CAPACITY_LEVEL_FULL;
		else if ((s1 & NHx054_STATUS_FULL_DISCHRG) && (s2 & NHx054_STATUS_FULL_DISCHRG))
			status = POWER_SUPPLY_CAPACITY_LEVEL_CRITICAL;
		else
			status = POWER_SUPPLY_CAPACITY_LEVEL_NORMAL;

	}

	val->intval = status;

	return 0;
}

static int nh2054_simple_value(int value, union power_supply_propval *val)
{
	if (value < 0)
		return value;

	val->intval = value;

	return 0;
}

static int nh2054_battery_get_property(struct power_supply *psy,
					enum power_supply_property psp,
					union power_supply_propval *val)
{
	struct nh2054_device_info *di = dev_get_drvdata(psy->dev->parent);
	int ret = 0;
	int value, p[2];
	struct nh2054_reg_cache c[2];
	struct nh2054_reg_cache_ex ce[2];
	int bat_num = -1;

#if NHx054_INDIVIDUAL_BAT_PS
	if (psy == &(di->bat[0]))
		bat_num = 0;
	else if (psy == &(di->bat[1]))
		bat_num = 1;
	else
		bat_num = -1;
#endif

	mutex_lock(&di->lock);
	p[0] = di->battery_present[0];
	p[1] = di->battery_present[1];
	memcpy(&(c[0]), &(di->_cache[0]), sizeof(struct nh2054_reg_cache));
	memcpy(&(c[1]), &(di->_cache[1]), sizeof(struct nh2054_reg_cache));
	memcpy(&(ce[0]), &(di->_cache_ex[0]), sizeof(struct nh2054_reg_cache_ex));
	memcpy(&(ce[1]), &(di->_cache_ex[1]), sizeof(struct nh2054_reg_cache_ex));
	mutex_unlock(&di->lock);


#define prop_value_max(v1, v2) ((v1 >= v2) ? v1 : v2)
#define prop_value_abs_max(v1, v2) ((abs(v1) >= abs(v2)) ? v1 : v2)

#define di_prop_value_ex(pres, cache, prop) (pres[0] ? cache[0]._##prop : (pres[1] ? cache[1]._##prop : -ENODATA))
#define di_prop_value_max_ex(pres, cache, prop) ((pres[0] && pres[1]) ? prop_value_max(cache[0]._##prop, cache[1]._##prop) : (pres[0] ? cache[0]._##prop : (pres[1] ? cache[1]._##prop : -ENODATA)))
#define di_prop_value_abs_max_ex(pres, cache, prop) ((pres[0] && pres[1]) ? prop_value_abs_max(cache[0]._##prop, cache[1]._##prop) : (pres[0] ? cache[0]._##prop : (pres[1] ? cache[1]._##prop : -ENODATA)))

#define di_prop_value_single(pres, cache, prop, bat) (pres[bat] ? cache[bat]._##prop : (-ENODATA))

#define di_prop_value(pres, cache, prop, bat) ((bat < 0) ? di_prop_value_ex(pres, cache, prop) :  di_prop_value_single(pres, cache, prop, bat))
#define di_prop_value_max(pres, cache, prop, bat) ((bat < 0) ? di_prop_value_max_ex(pres, cache, prop) :  di_prop_value_single(pres, cache, prop, bat))
#define di_prop_value_abs_max(pres, cache, prop, bat) ((bat < 0) ? di_prop_value_abs_max_ex(pres, cache, prop) :  di_prop_value_single(pres, cache, prop, bat))

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		ret = nh2054_battery_status(di, p, c, val, bat_num);
		DEV_DBG(di->dev, "---> %s: --> Enter. psy-name: %s ===> %s(%d): %d\n", __func__, psy->name, "POWER_SUPPLY_PROP_STATUS", psp, val->intval);
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		value = 0;
		if (bat_num >= 0)
			value = p[bat_num];
		else if (p[0] || p[1])
			value = 1;
		val->intval = value;
		DEV_DBG(di->dev, "---> %s: --> Enter. psy-name: %s ===> %s(%d): %d\n", __func__, psy->name, "POWER_SUPPLY_PROP_ONLINE", psp, val->intval);
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		value = 0;
		if (bat_num >= 0)
			value = p[bat_num];
		else if (p[0] || p[1])
			value = 1;
		val->intval = value;
		DEV_DBG(di->dev, "---> %s: --> Enter. psy-name: %s ===> %s(%d): %d\n", __func__, psy->name, "POWER_SUPPLY_PROP_PRESENT", psp, val->intval);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		value = di_prop_value_max(p, ce, volt, bat_num);
		ret = nh2054_simple_value(value, val);
		DEV_DBG(di->dev, "---> %s: --> Enter. psy-name: %s ===> %s(%d): %d\n", __func__, psy->name, "POWER_SUPPLY_PROP_VOLTAGE_NOW", psp, val->intval);
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		value = di_prop_value_abs_max(p, ce, curr, bat_num);
		val->intval = value;
		DEV_DBG(di->dev, "---> %s: --> Enter. psy-name: %s ===> %s(%d): %d - 0: %d; 1: %d\n", __func__, psy->name, "POWER_SUPPLY_PROP_CURRENT_NOW", psp, val->intval, di->_cache_ex[0]._curr, di->_cache_ex[1]._curr);
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		ret = nh2054_battery_health(di, p, c, val, bat_num);
		DEV_DBG(di->dev, "---> %s: --> Enter. psy-name: %s ===> %s(%d): %d\n", __func__, psy->name, "POWER_SUPPLY_PROP_HEALTH", psp, val->intval);
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		value = di_prop_value_max(p, c, capacity, bat_num);
		ret = nh2054_simple_value(value, val);
		DEV_DBG(di->dev, "---> %s: --> Enter. psy-name: %s ===> %s(%d): %d - 0: %d; 1: %d\n", __func__, psy->name, "POWER_SUPPLY_PROP_CAPACITY", psp, val->intval, di->_cache[0]._capacity, di->_cache[1]._capacity);
		break;
	case POWER_SUPPLY_PROP_CAPACITY_LEVEL:
		ret = nh2054_battery_capacity_level(di, p, c, val, bat_num);
		DEV_DBG(di->dev, "---> %s: --> Enter. psy-name: %s ===> %s(%d): %d\n", __func__, psy->name, "POWER_SUPPLY_PROP_CAPACITY_LEVEL", psp, val->intval);
		break;
	case POWER_SUPPLY_PROP_TEMP:
		value = di_prop_value_max(p, c, temperature, bat_num);
		ret = nh2054_simple_value(value, val);
		DEV_DBG(di->dev, "---> %s: --> Enter. psy-name: %s ===> %s(%d): %d\n", __func__, psy->name, "POWER_SUPPLY_PROP_TEMP", psp, val->intval);
		break;
	case POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW:
		value = di_prop_value_max(p, c, time_to_empty, bat_num);
		ret = nh2054_simple_value(value, val);
		DEV_DBG(di->dev, "---> %s: --> Enter. psy-name: %s ===> %s(%d): %d\n", __func__, psy->name, "POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW", psp, val->intval);
		break;
	case POWER_SUPPLY_PROP_TIME_TO_FULL_NOW:
		value = di_prop_value_max(p, c, time_to_full, bat_num);
		ret = nh2054_simple_value(value, val);
		DEV_DBG(di->dev, "---> %s: --> Enter. psy-name: %s ===> %s(%d): %d\n", __func__, psy->name, "POWER_SUPPLY_PROP_TIME_TO_FULL_NOW", psp, val->intval);
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		ret = nh2054_simple_value(POWER_SUPPLY_TECHNOLOGY_LION, val);
		DEV_DBG(di->dev, "---> %s: --> Enter. psy-name: %s ===> %s(%d): %d\n", __func__, psy->name, "POWER_SUPPLY_PROP_TECHNOLOGY", psp, val->intval);
		break;
	case POWER_SUPPLY_PROP_CHARGE_NOW:
		value = di_prop_value_max(p, c, charge_remain, bat_num);
		ret = nh2054_simple_value(value, val);
		DEV_DBG(di->dev, "---> %s: --> Enter. psy-name: %s ===> %s(%d): %d\n", __func__, psy->name, "POWER_SUPPLY_PROP_CHARGE_NOW", psp, val->intval);
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		value = di_prop_value_max(p, c, charge_full, bat_num);
		ret = nh2054_simple_value(value, val);
		DEV_DBG(di->dev, "---> %s: --> Enter. psy-name: %s ===> %s(%d): %d\n", __func__, psy->name, "POWER_SUPPLY_PROP_CHARGE_FULL", psp, val->intval);
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		value = di_prop_value_max(p, c, charge_design_full, bat_num);
		ret = nh2054_simple_value(value, val);
		DEV_DBG(di->dev, "---> %s: --> Enter. psy-name: %s ===> %s(%d): %d\n", __func__, psy->name, "POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN", psp, val->intval);
		break;
	case POWER_SUPPLY_PROP_CYCLE_COUNT:
		value = di_prop_value_max(p, c, cycle_count, bat_num);
		ret = nh2054_simple_value(value, val);
		DEV_DBG(di->dev, "---> %s: --> Enter. psy-name: %s ===> %s(%d): %d\n", __func__, psy->name, "POWER_SUPPLY_PROP_CYCLE_COUNT", psp, val->intval);
		break;
	default:
		ret = -EINVAL;
	}

	if (ret < 0) {
		DEV_DBG(di->dev, "---> %s: --> Enter. psy-name: %s ===> Failed for Property %d! rc: %d\n", __func__, psy->name, psp, ret);
	}
	else {
		DEV_DBG(di->dev, "---> %s: --> Enter. psy-name: %s ===> Prop %d: %d\n", __func__, psy->name, psp, val->intval);
	}

	return ret;
}

static void nh2054_external_power_changed(struct power_supply *psy)
{
	struct nh2054_device_info *di = dev_get_drvdata(psy->dev->parent);

	DEV_DBG(di->dev, "%s: ----> psy->name: %s\n", __func__, psy->name);

	i2c_read_errs = 0;
}

static int nh2054_battery_probe(struct i2c_client *client,
				 const struct i2c_device_id *id)
{
	struct nh2054_device_info *di;
	int rc;
	struct power_supply *battery;
	struct device_node *np;
	u32 dt_value_u32;

	di = kzalloc(sizeof(*di), GFP_KERNEL);
	if (!di) {
		dev_err(&client->dev, "failed to allocate device info data\n");
		return -ENOMEM;
	}

	mutex_init(&di->lock);
	mutex_init(&di->i2clock);

	memset(di->battery_present, 0x00, sizeof(di->battery_present));
	memset(di->_cache,			0x00, sizeof(di->_cache));
	memset(di->_cache_ex,		0x00, sizeof(di->_cache_ex));
	di->bat_dual_ps_registered = 0;
#if NHx054_INDIVIDUAL_BAT_PS
	memset(di->bat_ps_registered, 0x00, sizeof(di->bat_ps_registered));

	battery = &(di->bat[0]);
	battery->dev = &client->dev;
	battery->name = SMB_AD_LTC1760_BATTERY_SUPPLIED_TO_01;
	battery->type = POWER_SUPPLY_TYPE_BATTERY;
	battery->properties = nh2054_battery_props;
	battery->num_properties = ARRAY_SIZE(nh2054_battery_props);
	battery->get_property = nh2054_battery_get_property;
	battery->external_power_changed = nh2054_external_power_changed;

	battery = &(di->bat[1]);
	battery->dev = &client->dev;
	battery->name = SMB_AD_LTC1760_BATTERY_SUPPLIED_TO_02;
	battery->type = POWER_SUPPLY_TYPE_BATTERY;
	battery->properties = nh2054_battery_props;
	battery->num_properties = ARRAY_SIZE(nh2054_battery_props);
	battery->get_property = nh2054_battery_get_property;
	battery->external_power_changed = nh2054_external_power_changed;
#endif

	battery = &(di->bat_dual);
	di->dev = &client->dev;
	battery->dev = &client->dev;
	battery->name = SMB_AD_LTC1760_BATTERY_SUPPLIED_TO_DUAL;
	battery->type = POWER_SUPPLY_TYPE_BATTERY;
	battery->properties = nh2054_battery_props;
	battery->num_properties = ARRAY_SIZE(nh2054_battery_props);
	battery->get_property = nh2054_battery_get_property;
	battery->external_power_changed = nh2054_external_power_changed;

	di->capacity_low_thres_uAh			= BATTERY_CAPACITY_LOW_THRES_mAh * 1000;
	di->voltage_crit_low_thres_uV		= BATTERY_VOLTAGE_CRIT_LOW_THRES_mV * 1000;
	di->voltage_low_thres_uV			= BATTERY_VOLTAGE_LOW_THRES_mV * 1000;

	np = of_node_get(client->dev.of_node);

	rc = of_property_read_u32(np, "sciaps,support-shutdown",
					&dt_value_u32);

	if (rc < 0) {
		di->sciaps_support_shutdown = SCIAPS_SUPPORT_SHUTDOWN_DEFAULT;
		dev_info(&client->dev,
			"Unable to read 'sciaps,support-shutdown'. Use default: %d\n", di->sciaps_support_shutdown);
	}
	else {
		di->sciaps_support_shutdown = (uint16_t)dt_value_u32;
		dev_info(&client->dev,
			"'sciaps,support-shutdown' == %d\n", di->sciaps_support_shutdown);
	}

	if (di->sciaps_support_shutdown) {
		rc = of_property_read_u32(np, "sciaps,shutdown-timer",
						&dt_value_u32);

		if (rc < 0) {
			di->sciaps_shutdown_timer = SCIAPS_SHUTDOWN_TIMER_DEFAULT;
			dev_info(&client->dev,
				"Unable to read 'sciaps,shutdown-timer'. Use default: %d s\n", di->sciaps_shutdown_timer);
		}
		else {
			di->sciaps_shutdown_timer = (uint16_t)dt_value_u32;
			dev_info(&client->dev,
				"'sciaps,shutdown-timer' is %d s\n", di->sciaps_shutdown_timer);
		}
	}

	rc = of_property_read_u32(np, "sciaps,capacity-low-thres",
				&dt_value_u32);

	if (rc < 0) {
		dev_warn(&client->dev,
			"%s: sciaps,capacity-low-thres is not in the devicetree. Using the default value: %d in 0.1 %%\n", __func__, BATTERY_CAPACITY_LOW_THRES);
	}
	else {
		if (dt_value_u32 > BATTERY_CAPACITY_LOW_THRES_MAX)
			dt_value_u32 = BATTERY_CAPACITY_LOW_THRES_MAX;
		di->capacity_low_thres = (uint16_t)dt_value_u32;
		dev_info(&client->dev,
				"'sciaps,capacity-low-thres' is %d in 0.1 %%\n", di->capacity_low_thres);
	}

	if (di->capacity_low_thres == 0) {
		rc = of_property_read_u32(np, "sciaps,capacity-low-thres-mAh",
					&dt_value_u32);

		if (rc < 0) {
			dev_warn(&client->dev,
				"%s: sciaps,capacity-low-thres-mAh not in devicetree. Using the default value: %d mAh\n", __func__, BATTERY_CAPACITY_LOW_THRES_mAh);
		}
		else {
			di->capacity_low_thres_uAh = dt_value_u32 * 1000;
			dev_info(&client->dev,
					"'sciaps,capacity-low-thres-mAh' is %d\n", dt_value_u32);
		}
	}
	else {
		di->capacity_low_thres_uAh = 0;
		dev_info(&client->dev,
					"'sciaps,capacity-low-thres-mAh' is %d. sciaps,capacity-low-thres is to be used to calculate it.\n", di->capacity_low_thres_uAh/1000);

	}

	rc = of_property_read_u32(np, "sciaps,voltage-crit-low-thres-mV",
				&dt_value_u32);

	if (rc < 0) {
		dev_warn(&client->dev,
			"%s: sciaps,voltage-crit-low-thres-mV not in devicetree. Using the default value: %d mV\n", __func__, BATTERY_VOLTAGE_CRIT_LOW_THRES_mV);
	}
	else {
		di->voltage_crit_low_thres_uV = dt_value_u32 * 1000;
		dev_info(&client->dev,
			"'sciaps,voltage-crit-low-thres-mV' is %d\n", dt_value_u32);
	}

	rc = of_property_read_u32(np, "sciaps,voltage-low-thres-mV",
				&dt_value_u32);

	if (rc < 0) {
		dev_warn(&client->dev,
			"%s: sciaps,voltage-low-thres-mV not in devicetree. Using the default value: %d mV\n", __func__, BATTERY_VOLTAGE_LOW_THRES_mV);
	}
	else {
		di->voltage_low_thres_uV = dt_value_u32 * 1000;
		dev_info(&client->dev,
			"'sciaps,voltage-low-thres-mV' is %d\n", dt_value_u32);
	}


	i2c_set_clientdata(client, di);

	dev_info(di->dev, "%s: Registering Power Supply for dual battery...\n", __func__);
	if ((rc = power_supply_register(di->dev, battery))) {
		dev_err(di->dev, "%s: fail to register dual battery: %d\n", __func__, rc);
	}
	else {
		INIT_DELAYED_WORK(&di->work, nh2054_battery_poll);
		di->shutdown_work_running = 0;
		INIT_DELAYED_WORK(&di->shutdown_work, shutdown_work_func);
		//if (poll_interval > 0) {
		//	/* The timer does not have to be accurate. */
		//	set_timer_slack(&di->work.timer, poll_interval * HZ / 4);
		//	schedule_delayed_work(&di->work, poll_interval * HZ);
		//}
		schedule_delayed_work(&di->work, HZ);
		di->bat_dual_ps_registered = 1;
		dev_info(di->dev, "%s: Power Supply for dual battery registered!\n", __func__);
	}

	return rc;
}

static int nh2054_battery_remove(struct i2c_client *client)
{
	struct nh2054_device_info *di = i2c_get_clientdata(client);

	//poll_interval = 0;

	di->battery_present[0] = 0;
	di->battery_present[1] = 0;


	if (di->bat_dual_ps_registered) {
		cancel_delayed_work_sync(&di->work);
		if (di->shutdown_work_running) {
			di->shutdown_work_running = 0;
			cancel_delayed_work_sync(&di->shutdown_work);

		}
		power_supply_unregister(&di->bat_dual);
		di->bat_dual_ps_registered = 0;
	}

#if NHx054_INDIVIDUAL_BAT_PS
	{
		int i;
		for (i = 0; i < 2; i++) {
			if (di->bat_ps_registered[i]) {
				power_supply_unregister(&di->bat[i]);
				di->bat_ps_registered[i] = 0;
			}
		}
	}
#endif

	mutex_destroy(&di->lock);
	mutex_destroy(&di->i2clock);
	kfree(di);

	return 0;
}

static const struct i2c_device_id nh2054_id[] = {
	{ "nh2054-dual", 0 },
	{ "nh3054-dual", 0 },
	{},
};
MODULE_DEVICE_TABLE(i2c, nh2054_id);

static struct i2c_driver nh2054_battery_driver = {
	.driver = {
		.name = "ie-nh2054-dual-battery",
	},
	.probe = nh2054_battery_probe,
	.remove = nh2054_battery_remove,
	.id_table = nh2054_id,
};

static inline int nh2054_battery_i2c_init(void)
{
	int ret = i2c_add_driver(&nh2054_battery_driver);
	if (ret)
		printk(KERN_ERR "Unable to register NHx054 i2c driver\n");

	return ret;
}

static inline void nh2054_battery_i2c_exit(void)
{
	i2c_del_driver(&nh2054_battery_driver);
}

static int __init nh2054_battery_init(void)
{
	int ret;

	ret = nh2054_battery_i2c_init();

	return ret;
}
module_init(nh2054_battery_init);

static void __exit nh2054_battery_exit(void)
{
	nh2054_battery_i2c_exit();
}
module_exit(nh2054_battery_exit);

MODULE_AUTHOR("Andre Doudkin <adoudkin@gmail.com>");
MODULE_DESCRIPTION("NHx054 battery driver");
MODULE_LICENSE("GPL");
