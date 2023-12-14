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

#include <linux/qpnp/power-on.h>
#include <linux/mfd/sciaps_micro.h>

#if !defined(__devinit)
#define __devinit
#endif

#if !defined(__devexit)
#define __devexit
#endif

#if !defined(__devexit_p)
#define __devexit_p(x) (&(x))
#endif

#define SMB_TI_DEBUG 0
#define DEV_DBG dev_dbg

#define SMBUS_BATTERY_REG_TEMP			0x08 /* Temperature */
#define SMBUS_BATTERY_REG_VOLTAGE		0x09 /* Voltage */
#define SMBUS_BATTERY_REG_CURRENT		0x0a /* Current */
#define SMBUS_BATTERY_REG_AVG_CURRENT	0x0b /* Average Current */
#define SMBUS_BATTERY_REG_RSOC			0x0d /* Relative State-of-Charge */
#define SMBUS_BATTERY_REG_ASOC			0x0e /* Absolute State-of-Charge */
#define SMBUS_BATTERY_REG_RemainingCapacity			0x0f /* Remaining Capacity*/
#define SMBUS_BATTERY_REG_FullChargeCapacity		0x10 /* Full Charge Capacity */
#define SMBUS_BATTERY_REG_STATUS		0x16 /* Battery Status */
#define SMBUS_BATTERY_REG_RTTE			0x11 /* Run Time-to-Empty */
#define SMBUS_BATTERY_REG_ATTE			0x12 /* Average Time-to-Empty */
#define SMBUS_BATTERY_REG_ATTF			0x13 /* Average Time-to-Full */

/* bits in SMBUS_BATTERY_REG_STATUS */
#define SMBUS_BATTERY_STATUS_OVER_CHARGED			0x8000
#define SMBUS_BATTERY_STATUS_TERMINATE_CHARGE		0x4000
												  //0x2000
#define SMBUS_BATTERY_STATUS_OVERTEMP				0x1000
#define SMBUS_BATTERY_STATUS_TERMINATE_DISCHARGE	0x0800
												  //0x0400
#define SMBUS_BATTERY_STATUS_REMAINING_CAPACITY		0x0200
#define SMBUS_BATTERY_STATUS_REMAINING_TIME			0x0100
#define SMBUS_BATTERY_STATUS_INIT					0x0080
#define SMBUS_BATTERY_STATUS_DISCHARGING			0x0040
#define SMBUS_BATTERY_STATUS_FULL_CHRG				0x0020
#define SMBUS_BATTERY_STATUS_FULL_DISCHRG			0x0010

#define SCIAPS_SUPPORT_SHUTDOWN_DEFAULT				0
#define SCIAPS_SHUTDOWN_TIMER_DEFAULT				20

#define BATTERY_DISCHARGING_THRES_CURR				(-50)
#define BATTERY_CAPACITY_LOW_THRES					0 // By the default the BATTERY_CAPACITY_LOW_THRES_mAh is to be used
#define BATTERY_CAPACITY_LOW_THRES_MAX				200 // 20%
#define BATTERY_CAPACITY_LOW_THRES_mAh				100
#define BATTERY_VOLTAGE_CRIT_LOW_THRES_mV			12000
#define BATTERY_VOLTAGE_LOW_THRES_mV				13500


static enum power_supply_property smb_ti_bq40z80_properties[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	//POWER_SUPPLY_PROP_TYPE,
	//POWER_SUPPLY_PROP_CYCLE_COUNT,
	POWER_SUPPLY_PROP_CHARGING_ENABLED,
	//POWER_SUPPLY_PROP_CHARGE_ENABLED,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CAPACITY_LEVEL,
	POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW,
	POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG,
	POWER_SUPPLY_PROP_TIME_TO_FULL_NOW,
	POWER_SUPPLY_PROP_TIME_TO_FULL_AVG,
	POWER_SUPPLY_PROP_TEMP,
	//POWER_SUPPLY_PROP_SERIAL_NUMBER,
	//POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	//POWER_SUPPLY_PROP_CHARGE_NOW,
	//POWER_SUPPLY_PROP_CHARGE_FULL,
	//POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
};

struct smb_ti_bq40z80_battery_reg_cache {
	bool present;
	s16 status;
	s16 temperature;
	s16 time_to_empty;
	s16 time_to_full;
	s16 charge_full;
	s16 charge_design_full;
	s16 cycle_count;
	s16 capacity;
	s16 capacity_abs;
	s16 capacity_level;
	s16 voltage_now;
	s16 current_now;
	s16 time_to_empty_now;
	s16 charging_enabled;
	s16 full_charge_capacity;
	s16 remaining_capacity;
	//s16 atte;
	//s16 attf;
	//s16 type;
};

struct smb_ti_bq40z80_info {
	struct i2c_client	*client;
	struct power_supply	power_supply;
	struct smb_ti_bq40z80_battery_reg_cache reg_cache;
	struct delayed_work	work;

	struct delayed_work shutdown_work;	/* Shutdown workcheduler */

	uint16_t	sciaps_support_shutdown;
	uint16_t	sciaps_shutdown_timer;

	uint16_t	capacity_low_thres_mAh;
	uint16_t	capacity_low_thres; // in 0.1 %
	uint16_t	voltage_crit_low_thres_mV;
	uint16_t	voltage_low_thres_mV;

	uint8_t		shutdown_work_running;

	uint8_t		power_supply_registered;
	struct mutex lock;
};

static int i2c_read_errs = 0;

static int smb_ti_bq40z80_read_word_data(struct i2c_client *client, u8 address, s16* out)
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

/*
static int smb_ti_bq40z80_write_word_data(struct i2c_client *client, u8 address,
	u16 value)
{
	s32 ret = 0;
	int retries = 2;

	while (retries > 0) {
		ret = i2c_smbus_write_word_data(client, address,
			le16_to_cpu(value));
		if (ret >= 0)
			break;
		retries--;
	}

	if (ret < 0) {
		dev_dbg(&client->dev,
			"%s: i2c write to address 0x%x failed\n",
			__func__, address);
		return ret;
	}

	return 0;
}
*/

static int smb_ti_bq40z80_battery_status(struct smb_ti_bq40z80_info *chip, union power_supply_propval *val)
{

	if (!chip->reg_cache.present) {
		val->intval = POWER_SUPPLY_STATUS_UNKNOWN;
		return 0;
	}

	if (chip->reg_cache.status & SMBUS_BATTERY_STATUS_FULL_CHRG)
		val->intval = POWER_SUPPLY_STATUS_FULL;
	else if (chip->reg_cache.status & SMBUS_BATTERY_STATUS_DISCHARGING) {
		val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
		chip->reg_cache.charging_enabled = 0;
	}
	else {
		val->intval = POWER_SUPPLY_STATUS_CHARGING;
		chip->reg_cache.charging_enabled = 1;
	}

	return 0;
}

static int smb_ti_bq40z80_battery_health(struct smb_ti_bq40z80_info *chip, union power_supply_propval *val)
{
	if(!chip->reg_cache.present) {
		val->intval = POWER_SUPPLY_HEALTH_UNKNOWN;
		return 0;
	}

	if (chip->reg_cache.status & SMBUS_BATTERY_STATUS_OVERTEMP) {
		val->intval = POWER_SUPPLY_HEALTH_OVERHEAT;
	} else if(chip->reg_cache.status & SMBUS_BATTERY_STATUS_OVER_CHARGED) {
		val->intval = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
	} else {
		val->intval = POWER_SUPPLY_HEALTH_GOOD;
	}

	return 0;
}

static int smb_ti_bq40z80_battery_get_capacity(struct smb_ti_bq40z80_info *chip,
	union power_supply_propval *val)
{
	if (!chip->reg_cache.present) {
		val->intval = 0;
		return 0;
	}


	if (chip->reg_cache.status & SMBUS_BATTERY_STATUS_FULL_CHRG)
		val->intval = 100;
	else
		val->intval = chip->reg_cache.capacity;

	if(val->intval < 1) {
		val->intval = 1;
	}

	if(val->intval > 100) {
		val->intval = 100;
	}

	return 0;
}

static int smb_ti_bq40z80_battery_get_capacity_level(struct smb_ti_bq40z80_info *chip,
	union power_supply_propval *val)
{
	s16 capacity;
	if (!chip->reg_cache.present) {
		val->intval = 0;
		return 0;
	}


	if (chip->reg_cache.status & SMBUS_BATTERY_STATUS_FULL_CHRG)
		capacity = 100;
	else
		capacity = chip->reg_cache.capacity;

	if(capacity < 1) {
		capacity = 1;
	}

	if(capacity > 100) {
		capacity = 100;
	}

	if (capacity < 10)
		val->intval = POWER_SUPPLY_CAPACITY_LEVEL_CRITICAL;
	else if (capacity < 20)
		val->intval = POWER_SUPPLY_CAPACITY_LEVEL_LOW;
	else if (capacity < 90)
		val->intval = POWER_SUPPLY_CAPACITY_LEVEL_NORMAL;
	else if (capacity < 100)
		val->intval = POWER_SUPPLY_CAPACITY_LEVEL_HIGH;
	else
		val->intval = POWER_SUPPLY_CAPACITY_LEVEL_FULL;

	return 0;
}

static int smb_ti_bq40z80_battery_get_temp(struct smb_ti_bq40z80_info *chip,
	union power_supply_propval *val)
{
	if (!chip->reg_cache.present) {
		return -ENODATA;
	}


	val->intval = chip->reg_cache.temperature;
	val->intval -= 2731;	// convert from .1 degrees K

	return 0;
}

static int smb_ti_bq40z80_battery_get_voltage_now(struct smb_ti_bq40z80_info *chip,
	union power_supply_propval *val)
{
	if (!chip->reg_cache.present) {
		return -ENODATA;
	}


	val->intval = chip->reg_cache.voltage_now;
	val->intval *= 1000; //mV to uV

	return 0;
}

static int smb_ti_bq40z80_battery_get_current_now(struct smb_ti_bq40z80_info *chip,
	union power_supply_propval *val)
{
	if (!chip->reg_cache.present) {
		return -ENODATA;
	}

	val->intval = chip->reg_cache.current_now;
	val->intval *= 1000; //mA to uA

	return 0;
}

static int smb_ti_bq40z80_battery_get_time_to_empty(struct smb_ti_bq40z80_info *chip,
	union power_supply_propval *val)
{
	if (!chip->reg_cache.present) {
		return -ENODATA;
	}

	val->intval = chip->reg_cache.time_to_empty;

	// For testing...
	//if (val->intval < 0)
	//	val->intval = 0;

	if (val->intval > 0)
		val->intval *= 60; // minutes to seconds

	return 0;
}

static int smb_ti_bq40z80_battery_get_time_to_empty_now(struct smb_ti_bq40z80_info *chip,
	union power_supply_propval *val)
{
	if (!chip->reg_cache.present) {
		return -ENODATA;
	}

	val->intval = chip->reg_cache.time_to_empty_now;

	// For testing...
	//if (val->intval < 0)
	//	val->intval = 0;

	if (val->intval > 0)
		val->intval *= 60; // minutes to seconds

	return 0;
}

static int smb_ti_bq40z80_battery_get_time_to_full(struct smb_ti_bq40z80_info *chip,
	union power_supply_propval *val)
{
	if (!chip->reg_cache.present) {
		return -ENODATA;
	}

	val->intval = chip->reg_cache.time_to_full;

	// For testing...
	//if (val->intval < 0)
	//	val->intval = 0;

	if (val->intval > 0)
		val->intval *= 60; // minutes to seconds

	return 0;
}

#if 0
static int smb_ti_bq40z80_battery_get_type(struct smb_ti_bq40z80_info *chip,
	union power_supply_propval *val)
{
	if (!chip->reg_cache.present) {
		return -ENODATA;
	}


	val->intval = chip->reg_cache.type;

	return 0;
}
#endif

static char smb_ti_bq40z80_power_supply_prop_string[100];
static char smb_ti_bq40z80_power_supply_prop_value_string[100];
static char smb_ti_bq40z80_power_supply_prop_status_value_string[200];
static char* smb_ti_bq40z80_power_supply_prop_to_string(int prop)
{
	smb_ti_bq40z80_power_supply_prop_string[0] = 0;

	switch (prop) {
		case POWER_SUPPLY_PROP_STATUS : strcpy(smb_ti_bq40z80_power_supply_prop_string,"POWER_SUPPLY_PROP_STATUS"); break;
		case POWER_SUPPLY_PROP_CHARGE_TYPE : strcpy(smb_ti_bq40z80_power_supply_prop_string,"POWER_SUPPLY_PROP_CHARGE_TYPE"); break;
		case POWER_SUPPLY_PROP_HEALTH : strcpy(smb_ti_bq40z80_power_supply_prop_string,"POWER_SUPPLY_PROP_HEALTH"); break;
		case POWER_SUPPLY_PROP_PRESENT : strcpy(smb_ti_bq40z80_power_supply_prop_string,"POWER_SUPPLY_PROP_PRESENT"); break;
		case POWER_SUPPLY_PROP_ONLINE : strcpy(smb_ti_bq40z80_power_supply_prop_string,"POWER_SUPPLY_PROP_ONLINE"); break;
		case POWER_SUPPLY_PROP_AUTHENTIC : strcpy(smb_ti_bq40z80_power_supply_prop_string,"POWER_SUPPLY_PROP_AUTHENTIC"); break;
		case POWER_SUPPLY_PROP_CHARGING_ENABLED : strcpy(smb_ti_bq40z80_power_supply_prop_string,"POWER_SUPPLY_PROP_CHARGING_ENABLED"); break;
		case POWER_SUPPLY_PROP_TECHNOLOGY : strcpy(smb_ti_bq40z80_power_supply_prop_string,"POWER_SUPPLY_PROP_TECHNOLOGY"); break;
		case POWER_SUPPLY_PROP_CYCLE_COUNT : strcpy(smb_ti_bq40z80_power_supply_prop_string,"POWER_SUPPLY_PROP_CYCLE_COUNT"); break;
		case POWER_SUPPLY_PROP_VOLTAGE_MAX : strcpy(smb_ti_bq40z80_power_supply_prop_string,"POWER_SUPPLY_PROP_VOLTAGE_MAX"); break;
		case POWER_SUPPLY_PROP_VOLTAGE_MIN : strcpy(smb_ti_bq40z80_power_supply_prop_string,"POWER_SUPPLY_PROP_VOLTAGE_MIN"); break;
		case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN : strcpy(smb_ti_bq40z80_power_supply_prop_string,"POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN"); break;
		case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN : strcpy(smb_ti_bq40z80_power_supply_prop_string,"POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN"); break;
		case POWER_SUPPLY_PROP_VOLTAGE_NOW : strcpy(smb_ti_bq40z80_power_supply_prop_string,"POWER_SUPPLY_PROP_VOLTAGE_NOW"); break;
		case POWER_SUPPLY_PROP_VOLTAGE_AVG : strcpy(smb_ti_bq40z80_power_supply_prop_string,"POWER_SUPPLY_PROP_VOLTAGE_AVG"); break;
		case POWER_SUPPLY_PROP_VOLTAGE_OCV : strcpy(smb_ti_bq40z80_power_supply_prop_string,"POWER_SUPPLY_PROP_VOLTAGE_OCV"); break;
		case POWER_SUPPLY_PROP_INPUT_VOLTAGE_REGULATION : strcpy(smb_ti_bq40z80_power_supply_prop_string,"POWER_SUPPLY_PROP_INPUT_VOLTAGE_REGULATION"); break;
		case POWER_SUPPLY_PROP_CURRENT_MAX : strcpy(smb_ti_bq40z80_power_supply_prop_string,"POWER_SUPPLY_PROP_CURRENT_MAX"); break;
		case POWER_SUPPLY_PROP_INPUT_CURRENT_MAX : strcpy(smb_ti_bq40z80_power_supply_prop_string,"POWER_SUPPLY_PROP_INPUT_CURRENT_MAX"); break;
		case POWER_SUPPLY_PROP_INPUT_CURRENT_TRIM : strcpy(smb_ti_bq40z80_power_supply_prop_string,"POWER_SUPPLY_PROP_INPUT_CURRENT_TRIM"); break;
		case POWER_SUPPLY_PROP_INPUT_CURRENT_SETTLED : strcpy(smb_ti_bq40z80_power_supply_prop_string,"POWER_SUPPLY_PROP_INPUT_CURRENT_SETTLED"); break;
		case POWER_SUPPLY_PROP_VCHG_LOOP_DBC_BYPASS : strcpy(smb_ti_bq40z80_power_supply_prop_string,"POWER_SUPPLY_PROP_VCHG_LOOP_DBC_BYPASS"); break;
		case POWER_SUPPLY_PROP_CURRENT_NOW : strcpy(smb_ti_bq40z80_power_supply_prop_string,"POWER_SUPPLY_PROP_CURRENT_NOW"); break;
		case POWER_SUPPLY_PROP_CURRENT_AVG : strcpy(smb_ti_bq40z80_power_supply_prop_string,"POWER_SUPPLY_PROP_CURRENT_AVG"); break;
		case POWER_SUPPLY_PROP_POWER_NOW : strcpy(smb_ti_bq40z80_power_supply_prop_string,"POWER_SUPPLY_PROP_POWER_NOW"); break;
		case POWER_SUPPLY_PROP_POWER_AVG : strcpy(smb_ti_bq40z80_power_supply_prop_string,"POWER_SUPPLY_PROP_POWER_AVG"); break;
		case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN : strcpy(smb_ti_bq40z80_power_supply_prop_string,"POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN"); break;
		case POWER_SUPPLY_PROP_CHARGE_EMPTY_DESIGN : strcpy(smb_ti_bq40z80_power_supply_prop_string,"POWER_SUPPLY_PROP_CHARGE_EMPTY_DESIGN"); break;
		case POWER_SUPPLY_PROP_CHARGE_FULL : strcpy(smb_ti_bq40z80_power_supply_prop_string,"POWER_SUPPLY_PROP_CHARGE_FULL"); break;
		case POWER_SUPPLY_PROP_CHARGE_EMPTY : strcpy(smb_ti_bq40z80_power_supply_prop_string,"POWER_SUPPLY_PROP_CHARGE_EMPTY"); break;
		case POWER_SUPPLY_PROP_CHARGE_NOW : strcpy(smb_ti_bq40z80_power_supply_prop_string,"POWER_SUPPLY_PROP_CHARGE_NOW"); break;
		case POWER_SUPPLY_PROP_CHARGE_AVG : strcpy(smb_ti_bq40z80_power_supply_prop_string,"POWER_SUPPLY_PROP_CHARGE_AVG"); break;
		case POWER_SUPPLY_PROP_CHARGE_COUNTER : strcpy(smb_ti_bq40z80_power_supply_prop_string,"POWER_SUPPLY_PROP_CHARGE_COUNTER"); break;
		case POWER_SUPPLY_PROP_CHARGE_COUNTER_SHADOW : strcpy(smb_ti_bq40z80_power_supply_prop_string,"POWER_SUPPLY_PROP_CHARGE_COUNTER_SHADOW"); break;
		case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT : strcpy(smb_ti_bq40z80_power_supply_prop_string,"POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT"); break;
		case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX : strcpy(smb_ti_bq40z80_power_supply_prop_string,"POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX"); break;
		case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE : strcpy(smb_ti_bq40z80_power_supply_prop_string,"POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE"); break;
		case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX : strcpy(smb_ti_bq40z80_power_supply_prop_string,"POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX"); break;
		case POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT : strcpy(smb_ti_bq40z80_power_supply_prop_string,"POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT"); break;
		case POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT_MAX : strcpy(smb_ti_bq40z80_power_supply_prop_string,"POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT_MAX"); break;
		case POWER_SUPPLY_PROP_ENERGY_FULL_DESIGN : strcpy(smb_ti_bq40z80_power_supply_prop_string,"POWER_SUPPLY_PROP_ENERGY_FULL_DESIGN"); break;
		case POWER_SUPPLY_PROP_ENERGY_EMPTY_DESIGN : strcpy(smb_ti_bq40z80_power_supply_prop_string,"POWER_SUPPLY_PROP_ENERGY_EMPTY_DESIGN"); break;
		case POWER_SUPPLY_PROP_ENERGY_FULL : strcpy(smb_ti_bq40z80_power_supply_prop_string,"POWER_SUPPLY_PROP_ENERGY_FULL"); break;
		case POWER_SUPPLY_PROP_ENERGY_EMPTY : strcpy(smb_ti_bq40z80_power_supply_prop_string,"POWER_SUPPLY_PROP_ENERGY_EMPTY"); break;
		case POWER_SUPPLY_PROP_ENERGY_NOW : strcpy(smb_ti_bq40z80_power_supply_prop_string,"POWER_SUPPLY_PROP_ENERGY_NOW"); break;
		case POWER_SUPPLY_PROP_ENERGY_AVG : strcpy(smb_ti_bq40z80_power_supply_prop_string,"POWER_SUPPLY_PROP_ENERGY_AVG"); break;
		case POWER_SUPPLY_PROP_HI_POWER : strcpy(smb_ti_bq40z80_power_supply_prop_string,"POWER_SUPPLY_PROP_HI_POWER"); break;
		case POWER_SUPPLY_PROP_LOW_POWER : strcpy(smb_ti_bq40z80_power_supply_prop_string,"POWER_SUPPLY_PROP_LOW_POWER"); break;
		case POWER_SUPPLY_PROP_CAPACITY : strcpy(smb_ti_bq40z80_power_supply_prop_string,"POWER_SUPPLY_PROP_CAPACITY"); break;
		case POWER_SUPPLY_PROP_CAPACITY_ALERT_MIN : strcpy(smb_ti_bq40z80_power_supply_prop_string,"POWER_SUPPLY_PROP_CAPACITY_ALERT_MIN"); break;
		case POWER_SUPPLY_PROP_CAPACITY_ALERT_MAX : strcpy(smb_ti_bq40z80_power_supply_prop_string,"POWER_SUPPLY_PROP_CAPACITY_ALERT_MAX"); break;
		case POWER_SUPPLY_PROP_CAPACITY_LEVEL : strcpy(smb_ti_bq40z80_power_supply_prop_string,"POWER_SUPPLY_PROP_CAPACITY_LEVEL"); break;
		case POWER_SUPPLY_PROP_TEMP : strcpy(smb_ti_bq40z80_power_supply_prop_string,"POWER_SUPPLY_PROP_CAPACITY_LEVEL"); break;
		case POWER_SUPPLY_PROP_TEMP_ALERT_MIN : strcpy(smb_ti_bq40z80_power_supply_prop_string,"POWER_SUPPLY_PROP_TEMP"); break;
		case POWER_SUPPLY_PROP_TEMP_ALERT_MAX : strcpy(smb_ti_bq40z80_power_supply_prop_string,"POWER_SUPPLY_PROP_TEMP_ALERT_MIN"); break;
		case POWER_SUPPLY_PROP_COOL_TEMP : strcpy(smb_ti_bq40z80_power_supply_prop_string,"POWER_SUPPLY_PROP_TEMP_ALERT_MAX"); break;
		case POWER_SUPPLY_PROP_WARM_TEMP : strcpy(smb_ti_bq40z80_power_supply_prop_string,"POWER_SUPPLY_PROP_COOL_TEMP"); break;
		case POWER_SUPPLY_PROP_TEMP_AMBIENT : strcpy(smb_ti_bq40z80_power_supply_prop_string,"POWER_SUPPLY_PROP_WARM_TEMP"); break;
		case POWER_SUPPLY_PROP_TEMP_AMBIENT_ALERT_MIN : strcpy(smb_ti_bq40z80_power_supply_prop_string,"POWER_SUPPLY_PROP_TEMP_AMBIENT_ALERT_MIN"); break;
		case POWER_SUPPLY_PROP_TEMP_AMBIENT_ALERT_MAX : strcpy(smb_ti_bq40z80_power_supply_prop_string,"POWER_SUPPLY_PROP_TEMP_AMBIENT_ALERT_MAX"); break;
		case POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW : strcpy(smb_ti_bq40z80_power_supply_prop_string,"POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW"); break;
		case POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG : strcpy(smb_ti_bq40z80_power_supply_prop_string,"POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG"); break;
		case POWER_SUPPLY_PROP_TIME_TO_FULL_NOW : strcpy(smb_ti_bq40z80_power_supply_prop_string,"POWER_SUPPLY_PROP_TIME_TO_FULL_NOW"); break;
		case POWER_SUPPLY_PROP_TIME_TO_FULL_AVG : strcpy(smb_ti_bq40z80_power_supply_prop_string,"POWER_SUPPLY_PROP_TIME_TO_FULL_AVG"); break;
		case POWER_SUPPLY_PROP_TYPE : strcpy(smb_ti_bq40z80_power_supply_prop_string,"POWER_SUPPLY_PROP_TYPE"); break;
		case POWER_SUPPLY_PROP_SCOPE : strcpy(smb_ti_bq40z80_power_supply_prop_string,"POWER_SUPPLY_PROP_SCOPE"); break;
		case POWER_SUPPLY_PROP_SYSTEM_TEMP_LEVEL : strcpy(smb_ti_bq40z80_power_supply_prop_string,"POWER_SUPPLY_PROP_SYSTEM_TEMP_LEVEL"); break;
		case POWER_SUPPLY_PROP_RESISTANCE : strcpy(smb_ti_bq40z80_power_supply_prop_string,"POWER_SUPPLY_PROP_RESISTANCE"); break;
		case POWER_SUPPLY_PROP_RESISTANCE_CAPACITIVE : strcpy(smb_ti_bq40z80_power_supply_prop_string,"POWER_SUPPLY_PROP_RESISTANCE_CAPACITIVE"); break;
		case POWER_SUPPLY_PROP_RESISTANCE_ID : strcpy(smb_ti_bq40z80_power_supply_prop_string,"POWER_SUPPLY_PROP_RESISTANCE_ID"); break;
		case POWER_SUPPLY_PROP_RESISTANCE_NOW : strcpy(smb_ti_bq40z80_power_supply_prop_string,"POWER_SUPPLY_PROP_RESISTANCE_NOW"); break;
		case POWER_SUPPLY_PROP_USB_HC : strcpy(smb_ti_bq40z80_power_supply_prop_string,"POWER_SUPPLY_PROP_USB_HC"); break;
		case POWER_SUPPLY_PROP_USB_OTG : strcpy(smb_ti_bq40z80_power_supply_prop_string,"POWER_SUPPLY_PROP_USB_OTG"); break;
		case POWER_SUPPLY_PROP_CHARGE_ENABLED : strcpy(smb_ti_bq40z80_power_supply_prop_string,"POWER_SUPPLY_PROP_CHARGE_ENABLED"); break;
		case POWER_SUPPLY_PROP_FLASH_CURRENT_MAX : strcpy(smb_ti_bq40z80_power_supply_prop_string,"POWER_SUPPLY_PROP_FLASH_CURRENT_MAX"); break;
		case POWER_SUPPLY_PROP_CHARGE_COUNTER_EXT : strcpy(smb_ti_bq40z80_power_supply_prop_string,"POWER_SUPPLY_PROP_CHARGE_COUNTER_EXT"); break;
		case POWER_SUPPLY_PROP_MODEL_NAME : strcpy(smb_ti_bq40z80_power_supply_prop_string,"POWER_SUPPLY_PROP_MODEL_NAME"); break;
		case POWER_SUPPLY_PROP_MANUFACTURER : strcpy(smb_ti_bq40z80_power_supply_prop_string,"POWER_SUPPLY_PROP_MANUFACTURER"); break;
		case POWER_SUPPLY_PROP_SERIAL_NUMBER : strcpy(smb_ti_bq40z80_power_supply_prop_string,"POWER_SUPPLY_PROP_SERIAL_NUMBER"); break;
		case POWER_SUPPLY_PROP_BATTERY_TYPE : strcpy(smb_ti_bq40z80_power_supply_prop_string,"POWER_SUPPLY_PROP_BATTERY_TYPE"); break;
		default : strcpy(smb_ti_bq40z80_power_supply_prop_string,"POWER_SUPPLY_PROP_UNKNOWN"); break;
	}
	return smb_ti_bq40z80_power_supply_prop_string;
}

static char* smb_ti_bq40z80_power_supply_prop_value_to_string(int prop, int value)
{
	char* rv = smb_ti_bq40z80_power_supply_prop_value_string;

	switch (prop) {
		case POWER_SUPPLY_PROP_STATUS :
			switch (value) {
				case POWER_SUPPLY_STATUS_UNKNOWN : strcpy(smb_ti_bq40z80_power_supply_prop_value_string,"POWER_SUPPLY_STATUS_UNKNOWN"); break;
				case POWER_SUPPLY_STATUS_CHARGING : strcpy(smb_ti_bq40z80_power_supply_prop_value_string,"POWER_SUPPLY_STATUS_CHARGING"); break;
				case POWER_SUPPLY_STATUS_DISCHARGING : strcpy(smb_ti_bq40z80_power_supply_prop_value_string,"POWER_SUPPLY_STATUS_DISCHARGING"); break;
				case POWER_SUPPLY_STATUS_NOT_CHARGING : strcpy(smb_ti_bq40z80_power_supply_prop_value_string,"POWER_SUPPLY_STATUS_NOT_CHARGING"); break;
				case POWER_SUPPLY_STATUS_FULL : strcpy(smb_ti_bq40z80_power_supply_prop_value_string,"POWER_SUPPLY_STATUS_FULL"); break;
			}
			break;
		case POWER_SUPPLY_PROP_TYPE :
			switch (value) {
				case POWER_SUPPLY_CHARGE_TYPE_UNKNOWN : strcpy(smb_ti_bq40z80_power_supply_prop_value_string,"POWER_SUPPLY_CHARGE_TYPE_UNKNOWN"); break;
				case POWER_SUPPLY_CHARGE_TYPE_NONE : strcpy(smb_ti_bq40z80_power_supply_prop_value_string,"POWER_SUPPLY_CHARGE_TYPE_NONE"); break;
				case POWER_SUPPLY_CHARGE_TYPE_TRICKLE : strcpy(smb_ti_bq40z80_power_supply_prop_value_string,"POWER_SUPPLY_CHARGE_TYPE_TRICKLE"); break;
				case POWER_SUPPLY_CHARGE_TYPE_FAST : strcpy(smb_ti_bq40z80_power_supply_prop_value_string,"POWER_SUPPLY_CHARGE_TYPE_FAST"); break;
				case POWER_SUPPLY_CHARGE_TYPE_TAPER : strcpy(smb_ti_bq40z80_power_supply_prop_value_string,"POWER_SUPPLY_CHARGE_TYPE_TAPER"); break;
			}
			break;
		case POWER_SUPPLY_PROP_HEALTH :
			switch (value) {
				case POWER_SUPPLY_HEALTH_UNKNOWN : strcpy(smb_ti_bq40z80_power_supply_prop_value_string,"POWER_SUPPLY_HEALTH_UNKNOWN"); break;
				case POWER_SUPPLY_HEALTH_GOOD : strcpy(smb_ti_bq40z80_power_supply_prop_value_string,"POWER_SUPPLY_HEALTH_GOOD"); break;
				case POWER_SUPPLY_HEALTH_OVERHEAT : strcpy(smb_ti_bq40z80_power_supply_prop_value_string,"POWER_SUPPLY_HEALTH_OVERHEAT"); break;
				case POWER_SUPPLY_HEALTH_WARM : strcpy(smb_ti_bq40z80_power_supply_prop_value_string,"POWER_SUPPLY_HEALTH_WARM"); break;
				case POWER_SUPPLY_HEALTH_DEAD : strcpy(smb_ti_bq40z80_power_supply_prop_value_string,"POWER_SUPPLY_HEALTH_DEAD"); break;
				case POWER_SUPPLY_HEALTH_OVERVOLTAGE : strcpy(smb_ti_bq40z80_power_supply_prop_value_string,"POWER_SUPPLY_HEALTH_OVERVOLTAGE"); break;
				case POWER_SUPPLY_HEALTH_UNSPEC_FAILURE : strcpy(smb_ti_bq40z80_power_supply_prop_value_string,"POWER_SUPPLY_HEALTH_UNSPEC_FAILURE"); break;
				case POWER_SUPPLY_HEALTH_COLD : strcpy(smb_ti_bq40z80_power_supply_prop_value_string,"POWER_SUPPLY_HEALTH_COLD"); break;
				case POWER_SUPPLY_HEALTH_COOL : strcpy(smb_ti_bq40z80_power_supply_prop_value_string,"POWER_SUPPLY_HEALTH_COOL"); break;
				case POWER_SUPPLY_HEALTH_WATCHDOG_TIMER_EXPIRE : strcpy(smb_ti_bq40z80_power_supply_prop_value_string,"POWER_SUPPLY_HEALTH_WATCHDOG_TIMER_EXPIRE"); break;
				case POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE : strcpy(smb_ti_bq40z80_power_supply_prop_value_string,"POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE"); break;
			}
			break;
		case POWER_SUPPLY_PROP_TECHNOLOGY :
			switch (value) {
				case POWER_SUPPLY_TECHNOLOGY_UNKNOWN : strcpy(smb_ti_bq40z80_power_supply_prop_value_string,"POWER_SUPPLY_TECHNOLOGY_UNKNOWN"); break;
				case POWER_SUPPLY_TECHNOLOGY_NiMH : strcpy(smb_ti_bq40z80_power_supply_prop_value_string,"POWER_SUPPLY_TECHNOLOGY_NiMH"); break;
				case POWER_SUPPLY_TECHNOLOGY_LION : strcpy(smb_ti_bq40z80_power_supply_prop_value_string,"POWER_SUPPLY_TECHNOLOGY_LION"); break;
				case POWER_SUPPLY_TECHNOLOGY_LIPO : strcpy(smb_ti_bq40z80_power_supply_prop_value_string,"POWER_SUPPLY_TECHNOLOGY_LIPO"); break;
				case POWER_SUPPLY_TECHNOLOGY_LiFe : strcpy(smb_ti_bq40z80_power_supply_prop_value_string,"POWER_SUPPLY_TECHNOLOGY_LiFe"); break;
				case POWER_SUPPLY_TECHNOLOGY_NiCd : strcpy(smb_ti_bq40z80_power_supply_prop_value_string,"POWER_SUPPLY_TECHNOLOGY_NiCd"); break;
				case POWER_SUPPLY_TECHNOLOGY_LiMn : strcpy(smb_ti_bq40z80_power_supply_prop_value_string,"POWER_SUPPLY_TECHNOLOGY_LiMn"); break;
			}
			break;
		case POWER_SUPPLY_PROP_CAPACITY_LEVEL :
			switch (value) {
				case POWER_SUPPLY_CAPACITY_LEVEL_UNKNOWN : strcpy(smb_ti_bq40z80_power_supply_prop_value_string,"POWER_SUPPLY_CAPACITY_LEVEL_UNKNOWN"); break;
				case POWER_SUPPLY_CAPACITY_LEVEL_CRITICAL : strcpy(smb_ti_bq40z80_power_supply_prop_value_string,"POWER_SUPPLY_CAPACITY_LEVEL_CRITICAL"); break;
				case POWER_SUPPLY_CAPACITY_LEVEL_LOW : strcpy(smb_ti_bq40z80_power_supply_prop_value_string,"POWER_SUPPLY_CAPACITY_LEVEL_LOW"); break;
				case POWER_SUPPLY_CAPACITY_LEVEL_NORMAL : strcpy(smb_ti_bq40z80_power_supply_prop_value_string,"POWER_SUPPLY_CAPACITY_LEVEL_NORMAL"); break;
				case POWER_SUPPLY_CAPACITY_LEVEL_HIGH : strcpy(smb_ti_bq40z80_power_supply_prop_value_string,"POWER_SUPPLY_CAPACITY_LEVEL_HIGH"); break;
				case POWER_SUPPLY_CAPACITY_LEVEL_FULL : strcpy(smb_ti_bq40z80_power_supply_prop_value_string,"POWER_SUPPLY_CAPACITY_LEVEL_FULL"); break;
			}
			break;
		default:
			rv = NULL;
			break;
		//case POWER_SUPPLY_PROP_TYPE :
		//	switch (value) {
		//	}
		//	break;
	}
	return rv;
}

static char* smb_ti_bq40z80_power_supply_prop_status_value_to_string(int value)
{
	char* str = smb_ti_bq40z80_power_supply_prop_status_value_string;

	str[0] = 0;

	if (value & SMBUS_BATTERY_STATUS_OVER_CHARGED)
		strcat(str, "over-charged,");
	if (value & SMBUS_BATTERY_STATUS_TERMINATE_CHARGE)
		strcat(str, "term-charge,");
	if (value & SMBUS_BATTERY_STATUS_OVERTEMP)
		strcat(str, "over-temp,");
	if (value & SMBUS_BATTERY_STATUS_TERMINATE_DISCHARGE)
		strcat(str, "term-discharge,");
	if (value & SMBUS_BATTERY_STATUS_REMAINING_CAPACITY)
		strcat(str, "remaining-cap-alarm,");
	if (value & SMBUS_BATTERY_STATUS_REMAINING_TIME)
		strcat(str, "remaining-time-alarm,");
	if (value & SMBUS_BATTERY_STATUS_INIT)
		strcat(str, "init,");
	if (value & SMBUS_BATTERY_STATUS_DISCHARGING)
		strcat(str, "discharging,");
	if (value & SMBUS_BATTERY_STATUS_FULL_CHRG)
		strcat(str, "full-charge,");
	if (value & SMBUS_BATTERY_STATUS_FULL_DISCHRG)
		strcat(str, "full-discharged,");
	{
		char int_str[11];
		snprintf(int_str, 10, "%d", (value&0xf));
		strcat(str, int_str);
	}

	return str;
}

static int smb_ti_bq40z80_property_is_writeable(struct power_supply *psy,
						enum power_supply_property psp)
{
	struct smb_ti_bq40z80_info *chip = container_of(psy,
				struct smb_ti_bq40z80_info, power_supply);

	struct i2c_client *client = chip->client;

	dev_info(&client->dev,"%s", __func__);

	switch (psp) {
		case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		//case POWER_SUPPLY_PROP_CURRENT_MAX:
			return 1;
		default:
			break;
	}

	return 0;
}

static int smb_ti_bq40z80_set_property(struct power_supply *psy,
				  enum power_supply_property psp,
				  const union power_supply_propval *val)
{

	int ret = 0;
	struct smb_ti_bq40z80_info *chip = container_of(psy,
				struct smb_ti_bq40z80_info, power_supply);

	struct i2c_client *client = chip->client;

	dev_info(&client->dev,"%s", __func__);

	mutex_lock(&chip->lock);

	switch (psp) {
		case POWER_SUPPLY_PROP_CHARGING_ENABLED:
			if (val->intval)
				chip->reg_cache.charging_enabled = 1;
			else
				chip->reg_cache.charging_enabled = 0;
			break;
		//case POWER_SUPPLY_PROP_CURRENT_MAX:
		//	smb137c_set_charge_current_limit(chip, val->intval);
		//	break;
	default:
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}

	mutex_unlock(&chip->lock);

	power_supply_changed(&chip->power_supply);

	if(ret >= 0) {
		char* prop_str = smb_ti_bq40z80_power_supply_prop_to_string(psp);
		char* value_str = smb_ti_bq40z80_power_supply_prop_value_to_string(psp, val->intval);
		if (value_str) {
			dev_info(&client->dev,
				"%s: property SET = %d(%s), value = 0x%x(%s)\n"
						, __func__
						, psp
						, prop_str
						, val->intval
						, value_str);
		}
		else {
			dev_info(&client->dev,
				"%s: property SET = %d(%s), value = 0x%x(%d)\n"
						, __func__
						, psp
						, prop_str
						, val->intval
						, val->intval);
		}
	}
	return 0;
}

static int smb_ti_bq40z80_get_property(struct power_supply *psy,
	enum power_supply_property psp,
	union power_supply_propval *val)
{
	int ret = 0;
	struct smb_ti_bq40z80_info *chip = container_of(psy,
				struct smb_ti_bq40z80_info, power_supply);

  struct i2c_client *client = chip->client;

	mutex_lock(&chip->lock);

	switch(psp) {

		case POWER_SUPPLY_PROP_ONLINE:
		case POWER_SUPPLY_PROP_PRESENT:
			val->intval = chip->reg_cache.present ? 1 : 0;
			break;

		case POWER_SUPPLY_PROP_STATUS:
			ret = smb_ti_bq40z80_battery_status(chip, val);
			break;

		case POWER_SUPPLY_PROP_HEALTH:
			ret = smb_ti_bq40z80_battery_health(chip, val);
			break;

		case POWER_SUPPLY_PROP_CAPACITY:
			ret = smb_ti_bq40z80_battery_get_capacity(chip, val);
			break;

		case POWER_SUPPLY_PROP_TEMP:
			ret = smb_ti_bq40z80_battery_get_temp(chip, val);
			break;

		case POWER_SUPPLY_PROP_VOLTAGE_NOW:
			ret = smb_ti_bq40z80_battery_get_voltage_now(chip, val);
			break;

		case POWER_SUPPLY_PROP_CURRENT_NOW:
			ret = smb_ti_bq40z80_battery_get_current_now(chip, val);
			break;

		case POWER_SUPPLY_PROP_TECHNOLOGY:
			val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
			break;

		//case POWER_SUPPLY_PROP_TYPE:
		//	ret = smb_ti_bq40z80_battery_get_type(chip, val);
		//	break;

		case POWER_SUPPLY_PROP_CAPACITY_LEVEL:
			ret = smb_ti_bq40z80_battery_get_capacity_level(chip, val);
			break;

		case POWER_SUPPLY_PROP_CAPACITY_ALERT_MIN:
			val->intval = 20; // 20%
			break;

		case POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW:
			ret = smb_ti_bq40z80_battery_get_time_to_empty_now(chip, val);
			break;

		case POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG:
			ret = smb_ti_bq40z80_battery_get_time_to_empty(chip, val);
			break;

		case POWER_SUPPLY_PROP_TIME_TO_FULL_AVG:
			ret = smb_ti_bq40z80_battery_get_time_to_full(chip, val);
			break;

		case POWER_SUPPLY_PROP_TIME_TO_FULL_NOW:
			ret = smb_ti_bq40z80_battery_get_time_to_full(chip, val);
			break;

		case POWER_SUPPLY_PROP_CHARGING_ENABLED:
			val->intval = chip->reg_cache.charging_enabled;
			break;

		default:
			dev_err(&client->dev, "unknown psp: %d\n", psp);
			ret = -EINVAL;
	}

	mutex_unlock(&chip->lock);
#if SMB_TI_DEBUG
	if(ret >= 0) {
		char* prop_str = smb_ti_bq40z80_power_supply_prop_to_string(psp);
		char* value_str = smb_ti_bq40z80_power_supply_prop_value_to_string(psp, val->intval);
		if (value_str) {
			dev_info(&client->dev,
				"%s: property = %d(%s), value = 0x%x(%s)\n"
						, __func__
						, psp
						, prop_str
						, val->intval
						, value_str);
		}
		else {
			dev_info(&client->dev,
				"%s: property = %d(%s), value = 0x%x(%d)\n"
						, __func__
						, psp
						, prop_str
						, val->intval
						, val->intval);
		}
	}
#endif
	return ret;
}


static void smb_ti_bq40z80_external_power_changed(struct power_supply *psy)
{
	struct smb_ti_bq40z80_info *chip = container_of(psy, struct smb_ti_bq40z80_info, power_supply);
	struct i2c_client *client = chip->client;

	dev_err(&client->dev, "%s: external_power_changed", __func__);

	mutex_lock(&chip->lock);

	if ( chip->reg_cache.charging_enabled)
		chip->reg_cache.charging_enabled = 0;
	else
		chip->reg_cache.charging_enabled = 1;

	mutex_unlock(&chip->lock);

	power_supply_changed(&chip->power_supply);


	i2c_read_errs = 0;
}

static void shutdown_work_func(struct work_struct *work)
{
	sciaps_device_power_off(SCIAPS_DEVICE_POWER_OFF_OPT_SRC_BatteryRemoved);
}

#define UNKNOWN			0
#define ACTIVE			1
#define PASSIVE			2

static uint8_t g_active = UNKNOWN;
static struct smb_ti_bq40z80_info* g_chip = NULL;

//int8_t is_ltc294x_active() {
//	return g_active;
//}
//extern is_ti_bq40z80_active(void);
extern void deactivate_ltc294x(void);
extern void deactivate_ltc4100(void);
extern void activate_ltc4100(void);

void deactivate_ti_bq40z80(void)
{
	if (g_active != PASSIVE) {
		g_active = PASSIVE;
		deactivate_ltc4100();
		if (g_chip && g_chip->shutdown_work_running) {
			dev_info(&g_chip->client->dev, "%s: sciaps -- cancelling shutdown_work work!\n", __func__);
			cancel_delayed_work(&g_chip->shutdown_work);
			g_chip->shutdown_work_running = 0;
		}
		if (g_chip && g_chip->power_supply_registered) {
			dev_info(&g_chip->client->dev, "%s: sciaps -- unregistering power_suppply!\n", __func__);
			power_supply_unregister(&g_chip->power_supply);
			g_chip->power_supply_registered = 0;
		}

	}
}


static void smb_ti_bq40z80_delayed_work(struct work_struct *work)
{
	#define CHECK_READ_WORD(reg, location, min, max) \
	ret = smb_ti_bq40z80_read_word_data(chip->client, reg, &ival); \
	if(ret >= 0) { \
		DEV_DBG(&chip->client->dev, "read " #reg " = 0x%x(%d)\n", ival, ival); \
		if((max == 0 && min == 0) || (ival >= min && ival < max)) { \
			location = ival; \
		}	\
		else { \
			dev_info(&chip->client->dev, "---out of range value--> read " #reg " = 0x%x(%d)\n", ival, ival); \
		} \
	}

	// Roughly 60 seconds at 2s poll freq
	#define READ_ERR_LIMIT 30

	struct smb_ti_bq40z80_info *chip;
	bool changed = false;
	struct smb_ti_bq40z80_battery_reg_cache cache;
	s32 ret;
	s16 ival;
	bool no_battery = false;
	uint32_t capacity_low_thres_mAh = 0;

	chip = container_of(work, struct smb_ti_bq40z80_info, work.work);

	mutex_lock(&chip->lock);

	cache = chip->reg_cache;

	mutex_unlock(&chip->lock);

	CHECK_READ_WORD(SMBUS_BATTERY_REG_STATUS, cache.status, 0, 0)
	cache.present = ret >= 0;
	if (cache.present) {
		deactivate_ltc294x();
		if (g_active != ACTIVE) {
			g_active = ACTIVE;
			activate_ltc4100();
			if (chip->power_supply_registered) {
				power_supply_unregister(&chip->power_supply);
				chip->power_supply_registered = 0;
			}
			ret = power_supply_register(&chip->client->dev, &chip->power_supply);
			if (ret < 0) {
				dev_err(&chip->client->dev,
								"%s: Failed to register power supply\n", __func__);
				ret = 0;
			}
			else {
				chip->power_supply_registered = 1;
			}
		}
		usleep_range(100, 10000);
		CHECK_READ_WORD(SMBUS_BATTERY_REG_RSOC, cache.capacity, 0, 101)
		//usleep_range(100, 10000);
		CHECK_READ_WORD(SMBUS_BATTERY_REG_ASOC, cache.capacity_abs, 0, 101)
		//usleep_range(100, 10000);
		CHECK_READ_WORD(SMBUS_BATTERY_REG_TEMP, cache.temperature, 0, 0)
		//usleep_range(100, 10000);
		CHECK_READ_WORD(SMBUS_BATTERY_REG_VOLTAGE, cache.voltage_now, 0, 24000)
		//usleep_range(100, 10000);
		CHECK_READ_WORD(SMBUS_BATTERY_REG_CURRENT, cache.current_now, -10000, +10000)
		//usleep_range(100, 10000);
		CHECK_READ_WORD(SMBUS_BATTERY_REG_RTTE, cache.time_to_empty_now, 0, 0)
		//usleep_range(100, 10000);
		CHECK_READ_WORD(SMBUS_BATTERY_REG_ATTE, cache.time_to_empty, 0, 0)
		//usleep_range(100, 10000);
		CHECK_READ_WORD(SMBUS_BATTERY_REG_ATTF, cache.time_to_full, 0, 0)
		CHECK_READ_WORD(SMBUS_BATTERY_REG_FullChargeCapacity, cache.full_charge_capacity, 0, 25000)
		CHECK_READ_WORD(SMBUS_BATTERY_REG_RemainingCapacity, cache.remaining_capacity, 0, 25000)


		//usleep_range(100, 10000);
		//CHECK_READ_WORD(SMBUS_BATTERY_REG_TYPE, cache.type, 0, 65535)
		//usleep_range(100, 10000);
		//CHECK_READ_WORD(SMBUS_BATTERY_REG_CAPACITY_LEVEL, cache.capacity_level, 0, 65535)
		i2c_read_errs = 0;

		if ( chip->capacity_low_thres != 0
				&& (chip->capacity_low_thres_mAh == 0
						|| cache.full_charge_capacity != chip->reg_cache.full_charge_capacity)) {
			capacity_low_thres_mAh = cache.full_charge_capacity;
			capacity_low_thres_mAh *= chip->capacity_low_thres;
			capacity_low_thres_mAh /= 1000;
		}
	}
	else {
		//bool other_battery_present = true; // Check if other driver active

		//if (other_battery_present) {
			// Do nothing other battery has the control....
		if (g_active != ACTIVE) {
			if (i2c_read_errs <= READ_ERR_LIMIT) {
				if (i2c_read_errs == READ_ERR_LIMIT)
					dev_err(&chip->client->dev,
						"%s: Too many i2c errors.  Entering slow poll mode\n", __func__);
				i2c_read_errs++;
			}
		}
		else {
			no_battery = true;
		}
	}

	if (g_active == ACTIVE) {
		mutex_lock(&chip->lock);
		if (i2c_read_errs == 0) {
			if (capacity_low_thres_mAh && chip->capacity_low_thres_mAh != (uint16_t)capacity_low_thres_mAh) {
				dev_info(&chip->client->dev,
							"--!!!--> Updating capacity-low-thres-mAh from %d mAh to %d mAh\n", chip->capacity_low_thres_mAh, (uint16_t)capacity_low_thres_mAh);
				chip->capacity_low_thres_mAh = (uint16_t)capacity_low_thres_mAh;
			}

			DEV_DBG(&chip->client->dev,
						"%s: ---> %s - %d mV - %d mA - %d %% - %d/%d mAh - %d/%d min - %d min\n", __func__, smb_ti_bq40z80_power_supply_prop_status_value_to_string(cache.status)
							, cache.voltage_now
							, cache.current_now
							, cache.capacity
							, cache.remaining_capacity, cache.full_charge_capacity
							, cache.time_to_empty_now
							, cache.time_to_empty
							, cache.time_to_full);
		}

		if (chip->sciaps_support_shutdown
				&& chip->sciaps_shutdown_timer) {
			if (no_battery) {
				if (!chip->shutdown_work_running) {
					dev_info(&chip->client->dev, "sciaps -- starting shutdown_work work!\n");
					schedule_delayed_work(&chip->shutdown_work, chip->sciaps_shutdown_timer * HZ);
					chip->shutdown_work_running = 1;
				}
			}
			else {
				if (chip->shutdown_work_running) {
					dev_info(&chip->client->dev, "sciaps -- cancelling shutdown_work work!\n");
					cancel_delayed_work(&chip->shutdown_work);
				}
				chip->shutdown_work_running = 0;
			}
		}


		changed = memcmp(&chip->reg_cache, &cache, sizeof(struct smb_ti_bq40z80_battery_reg_cache));


		if (changed) {
			if (cache.voltage_now <= chip->voltage_crit_low_thres_mV
								|| (cache.remaining_capacity <= chip->capacity_low_thres_mAh && cache.voltage_now < chip->voltage_low_thres_mV)
							) {
				if (cache.current_now <= BATTERY_DISCHARGING_THRES_CURR) {
					dev_info(&chip->client->dev, "%s: ----> Battery Low Alert! Powering off...\n", __func__);
					dev_info(&chip->client->dev, "%s --> capacity: %d mAh; voltage: %d mV; current: %d mA;\n"
							, __func__
							, cache.remaining_capacity
							, cache.voltage_now
							, cache.current_now);
					sciaps_device_power_off(SCIAPS_DEVICE_POWER_OFF_OPT_SRC_BatteryLow);
				}
			}
			chip->reg_cache = cache;
		}

		mutex_unlock(&chip->lock);

		if (changed) {
			power_supply_changed(&chip->power_supply);
		}
	}
	if (i2c_read_errs > READ_ERR_LIMIT)
		schedule_delayed_work(&chip->work, 2*2*HZ); else
		schedule_delayed_work(&chip->work, 2*HZ);
}

static int __devinit smb_ti_bq40z80_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	struct smb_ti_bq40z80_info *chip;
	int rc;
	struct device_node *np;
	u32 dt_value_u32;

	chip = kzalloc(sizeof(struct smb_ti_bq40z80_info), GFP_KERNEL);
	if (!chip) {
		return -ENOMEM;
	}
	g_chip = chip;
	chip->client = client;
	np = of_node_get(client->dev.of_node);

	chip->power_supply_registered = 0;
	chip->power_supply.name = "main-battery";//name; //"ie-smbus-battery"
	chip->power_supply.type = POWER_SUPPLY_TYPE_BATTERY;
	chip->power_supply.properties = smb_ti_bq40z80_properties;
	chip->power_supply.num_properties = ARRAY_SIZE(smb_ti_bq40z80_properties);
	chip->power_supply.get_property = smb_ti_bq40z80_get_property;
	chip->power_supply.set_property = smb_ti_bq40z80_set_property;
	chip->power_supply.property_is_writeable = smb_ti_bq40z80_property_is_writeable;
	chip->power_supply.external_power_changed = smb_ti_bq40z80_external_power_changed;

	chip->capacity_low_thres_mAh			= BATTERY_CAPACITY_LOW_THRES_mAh;
	chip->voltage_crit_low_thres_mV			= BATTERY_VOLTAGE_CRIT_LOW_THRES_mV;
	chip->voltage_low_thres_mV				= BATTERY_VOLTAGE_LOW_THRES_mV;


	rc = of_property_read_u32(np, "sciaps,support-shutdown",
					&dt_value_u32);

	if (rc < 0) {
		chip->sciaps_support_shutdown = SCIAPS_SUPPORT_SHUTDOWN_DEFAULT;
		dev_info(&client->dev,
			"Unable to read 'sciaps,support-shutdown'. Use default: %d\n", chip->sciaps_support_shutdown);
	}
	else {
		chip->sciaps_support_shutdown = (uint16_t)dt_value_u32;
		dev_info(&client->dev,
			"'sciaps,support-shutdown' == %d\n", chip->sciaps_support_shutdown);
	}

	if (chip->sciaps_support_shutdown) {
		rc = of_property_read_u32(np, "sciaps,shutdown-timer",
						&dt_value_u32);

		if (rc < 0) {
			chip->sciaps_shutdown_timer = SCIAPS_SHUTDOWN_TIMER_DEFAULT;
			dev_info(&client->dev,
				"Unable to read 'sciaps,shutdown-timer'. Use default: %d s\n", chip->sciaps_shutdown_timer);
		}
		else {
			chip->sciaps_shutdown_timer = (uint16_t)dt_value_u32;
			dev_info(&client->dev,
				"'sciaps,shutdown-timer' is %d s\n", chip->sciaps_shutdown_timer);
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
		chip->capacity_low_thres = (uint16_t)dt_value_u32;
		dev_info(&client->dev,
				"'sciaps,capacity-low-thres' is %d in 0.1 %%\n", chip->capacity_low_thres);
	}

	if (chip->capacity_low_thres == 0) {
		rc = of_property_read_u32(np, "sciaps,capacity-low-thres-mAh",
					&dt_value_u32);

		if (rc < 0) {
			dev_warn(&client->dev,
				"%s: sciaps,capacity-low-thres-mAh not in devicetree. Using the default value: %d mAh\n", __func__, BATTERY_CAPACITY_LOW_THRES_mAh);
		}
		else {
			chip->capacity_low_thres_mAh = (uint16_t)dt_value_u32;
			dev_info(&client->dev,
					"'sciaps,capacity-low-thres-mAh' is %d\n", chip->capacity_low_thres_mAh);
		}
	}
	else {
		chip->capacity_low_thres_mAh = 0;
		dev_info(&client->dev,
					"'sciaps,capacity-low-thres-mAh' is %d. sciaps,capacity-low-thres is to be used to calculate it.\n", chip->capacity_low_thres_mAh);

	}

	rc = of_property_read_u32(np, "sciaps,voltage-crit-low-thres-mV",
				&dt_value_u32);

	if (rc < 0) {
		dev_warn(&client->dev,
			"%s: sciaps,voltage-crit-low-thres-mV not in devicetree. Using the default value: %d mV\n", __func__, BATTERY_VOLTAGE_CRIT_LOW_THRES_mV);
	}
	else {
		chip->voltage_crit_low_thres_mV = (uint16_t)dt_value_u32;
		dev_info(&client->dev,
			"'sciaps,voltage-crit-low-thres-mV' is %d\n", chip->voltage_crit_low_thres_mV);
	}

	rc = of_property_read_u32(np, "sciaps,voltage-low-thres-mV",
				&dt_value_u32);

	if (rc < 0) {
		dev_warn(&client->dev,
			"%s: sciaps,voltage-low-thres-mV not in devicetree. Using the default value: %d mV\n", __func__, BATTERY_VOLTAGE_LOW_THRES_mV);
	}
	else {
		chip->voltage_low_thres_mV = (uint16_t)dt_value_u32;
		dev_info(&client->dev,
			"'sciaps,voltage-low-thres-mV' is %d\n", chip->voltage_low_thres_mV);
	}

	chip->reg_cache.present = false;
	chip->reg_cache.capacity = 100;

	mutex_init(&chip->lock);

	i2c_set_clientdata(client, chip);

	INIT_DELAYED_WORK(&chip->work, smb_ti_bq40z80_delayed_work);
	chip->shutdown_work_running = 0;
	INIT_DELAYED_WORK(&chip->shutdown_work, shutdown_work_func);

	rc = power_supply_register(&chip->client->dev, &chip->power_supply);

	if (rc < 0) {
		dev_err(&chip->client->dev,
						"%s: Failed to register power supply\n", __func__);
		rc = 0;
	}
	else {
		chip->power_supply_registered = 1;
		g_active = ACTIVE;
		activate_ltc4100();
	}

	dev_info(&client->dev,
		"%s: battery gas gauge device registered\n", client->name);

	schedule_delayed_work(&chip->work, HZ);

	return 0;
}

static int __devexit smb_ti_bq40z80_remove(struct i2c_client *client)
{
	struct smb_ti_bq40z80_info *chip = i2c_get_clientdata(client);

	cancel_delayed_work_sync(&chip->work);
	cancel_delayed_work(&chip->shutdown_work);
	if (chip->power_supply_registered) {
		power_supply_unregister(&chip->power_supply);
		chip->power_supply_registered = 0;
	}
	g_chip = NULL;
	mutex_destroy(&chip->lock);
	//kfree(chip->power_supply.name);
	kfree(chip);
	chip = NULL;

	return 0;
}

static const struct i2c_device_id smb_ti_bq40z80_id[] = {
	{ "smb-ti-bq40z80", 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, smb_ti_bq40z80_id);

static struct i2c_driver smb_ti_bq40z80_battery_driver = {
	.probe		= smb_ti_bq40z80_probe,
	.remove		= __devexit_p(smb_ti_bq40z80_remove),
	.id_table	= smb_ti_bq40z80_id,
	.driver = {
		.name	= "smb-ti-bq40z80",
	},
};
module_i2c_driver(smb_ti_bq40z80_battery_driver);

MODULE_AUTHOR("Andre Doudkin <adoudkin@sciaps.com>");
MODULE_DESCRIPTION("SMB TI BQ40Z80 battery monitor driver");
MODULE_LICENSE("GPL");
