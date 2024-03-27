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
#include <asm/uaccess.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>

#if !defined(__devinit)
#define __devinit
#endif

#if !defined(__devexit)
#define __devexit
#endif

#if !defined(__devexit_p)
#define __devexit_p(x) (&(x))
#endif

#define UBLOX_SAM_M10Q_DEBUG_LEVEL_NONE 0
#define UBLOX_SAM_M10Q_DEBUG_LEVEL_1	1
#define UBLOX_SAM_M10Q_DEBUG_LEVEL_2	2
#define UBLOX_SAM_M10Q_DEBUG UBLOX_SAM_M10Q_DEBUG_LEVEL_NONE
#define DEV_DBG dev_dbg

const uint32_t UBLOX_SAM_M10Q_hAccRejectHardThreshold_Default		= 35000;	// mm
const uint16_t UBLOX_SAM_M10Q_pDOPRejectHardThreshold_Default		= 3000;		// 0.01
const uint16_t UBLOX_SAM_M10Q_pDOPRejectThreshold_Default			= 1000;		// 0.01
const uint32_t UBLOX_SAM_M10Q_hAccAcceptThreshold_Default			= 15000;	// mm
const uint32_t UBLOX_SAM_M10Q_nSatsAcceptAlwaysThreshold_Default	= 5;		// # of sats


#define UBX_NAV_PVT_FIX_TYPE_2D		2
#define UBX_NAV_PVT_FIX_TYPE_3D		3
#define UBX_NAV_PVT_FIX_OK(fixType)	(fixType == UBX_NAV_PVT_FIX_TYPE_2D || fixType == UBX_NAV_PVT_FIX_TYPE_3D)

#define UBX_NAV_PVT_FLAGS_FIX_VALID	0x01

#if 0
static int num_ttys = 4;	    /* # of std ttys to create per fw_card    */
				    /* - doubles as loopback port index       */
static bool auto_connect = true;    /* try to VIRT_CABLE to every peer        */
static bool create_loop_dev = true; /* create a loopback device for each card */

module_param_named(ttys, num_ttys, int, S_IRUGO | S_IWUSR);
module_param_named(auto, auto_connect, bool, S_IRUGO | S_IWUSR);
module_param_named(loop, create_loop_dev, bool, S_IRUGO | S_IWUSR);
#endif

static int s_cmdline_param_enable = 0;

module_param_named(enable, s_cmdline_param_enable, int, S_IRUSR);


// Power Mode
// ----------
// Attribute:
//		/sys/kernel/sciaps/gnss/power-mode
//
// Values:
//		#define UBX_PM_CONTINUOUS	0
//		#define UBX_PM_SW_STANDBY	3
//
// Ecamples:
//		echo 0 > /sys/kernel/sciaps/gnss/power-mode  # Enables Normal/Continuous Mode
//		echo 3 > /sys/kernel/sciaps/gnss/power-mode  # Enbales Standby Mode
//
//	Format:
//		"v"
//			where
//				- 'v' is the operation mode:
//						- 0 = Normal/Continuous Mode
//						- 3 = SW Standby Mode

// Reset
// -----
// Attribute:
//		/sys/kernel/sciaps/gnss/reset
// Ecamples:
//		echo 0000:01 > /sys/kernel/sciaps/gnss/reset  # Controlled software reset/Hot start
//		echo FFFF:00 > /sys/kernel/sciaps/gnss/reset  # Hardware reset (watchdog) immediately/Cold start
//		echo FFFF:FF > /sys/kernel/sciaps/gnss/reset  # U-Blox RST pin reset
//	Format:
//		"mmmm:rr"
//			where
//				- 'rr' is the reset type in hex:
//						- 00 = Hardware reset (watchdog) immediately
//						- 01 = Controlled software reset
//						- 02 = Controlled software reset (GNSS only)
//						- 04 = Hardware reset (watchdog) after shutdown
//						- 08 = Controlled GNSS stop
//						- 09 = Controlled GNSS start
//						- FF = ---> U-Blox RST pin reset Use it only in critical situations to recover the receiver. It resets the receiver and
//											clears the BBR content including receiver configuration, real-time clock (RTC), and GNSS orbit
//											data, triggering a cold start
//				- 'mmmm' is the reset mask in hex:
//						- 0000 - Hot start;
//						- 0001 - Warm start;
//						- FFFF - Cold start;
//						Details:
//							.... .... .... ...x - eph		- Ephemeris
//							.... .... .... ..x. - alm		- Almanac
//							.... .... .... .x.. - health	- Health
//							.... .... .... x... - klob		- Klobuchar parameters
//							.... .... ...x .... - pos		- Position
//							.... .... ..x. .... - clkd		- Clock drift
//							.... .... .x.. .... - osc		- Oscillator parameter
//							.... .... x... .... - utc		- UTC correction + GPS leap seconds parameters
//							.... ...x .... .... - rtc		- RTC
//							.... ..x. .... .... - reserved
//							.... .x.. .... .... - reserved
//							.... x... .... .... - sfdr		- SFDR Parameters (N/A)
//							...x .... .... .... - vmon		- SFDR Vehicle Monitoring Parameter (N/A)
//							..x. .... .... .... - tct		- TCT Parameters (N/A)
//							.x.. .... .... .... - reserved
//							x... .... .... .... - aop		- Autonomous orbit parameters


struct ubx_nav_pvt_time {
	uint32_t	itow;
	uint16_t	year;			// y        Year (UTC)
	uint8_t		month;			// month    Month, range 1..12 (UTC)
	uint8_t		day;			// d		Day of month, range 1..31 (UTC)
	uint8_t		hour;			// h		Hour of day, range 0..23 (UTC)
	uint8_t		min;			// min		Minute of hour, range 0..59 (UTC)
	uint8_t		sec;			// s		Seconds of minute, range 0..60 (UTC)
#define UBX_NAV_PVT_VALID_MASK	0x0f
	uint8_t		valid;			// Flags:
								//				.... ...x - validDate;
								//				.... ..x. - validTime;
								//				.... .x.. - fullyResolved;
								//				.... x... - validMag;
	uint32_t	tAcc;			// ns		Time accuracy estimate (UTC)
	int32_t		nano;			// ns		Fraction of second, range -1e9 .. 1e9 (UTC)
} __attribute__ ((packed));

struct ubx_nav_pvt {
	uint32_t	itow;
	uint16_t	year;			// y        Year (UTC)
	uint8_t		month;			// month    Month, range 1..12 (UTC)
	uint8_t		day;			// d		Day of month, range 1..31 (UTC)
	uint8_t		hour;			// h		Hour of day, range 0..23 (UTC)
	uint8_t		min;			// min		Minute of hour, range 0..59 (UTC)
	uint8_t		sec;			// s		Seconds of minute, range 0..60 (UTC)
#define UBX_NAV_PVT_VALID_MASK	0x0f
	uint8_t		valid;			// Flags:
								//				.... ...x - validDate;
								//				.... ..x. - validTime;
								//				.... .x.. - fullyResolved;
								//				.... x... - validMag;
	uint32_t	tAcc;			// ns		Time accuracy estimate (UTC)
	int32_t		nano;			// ns		Fraction of second, range -1e9 .. 1e9 (UTC)
	uint8_t		fixType;		// Fix Type:	0 = no fix;
								//				1 = dead reckoning only;
								//				2 = 2D fix;
								//				3 = 3D fix;
								//				4 = GNSS + dead reckoning combined
								//				5 = time only fix
#define UBX_NAV_PVT_FLAGS_MASK	0xff
	uint8_t		flags;			// Flags:
								//				.... ...x = valid fix (i.e. within DOP & accuracy mask)
								//				.... ..x. = differential corrections were applied
								//				...x xx.. = Power save mode state:
								//										0 = PSM is not active
								//										1 = Enabled (an intermediate state before Acquisition state)
								//										2 = Acquisition
								//										3 = Tracking
								//										4 = Power Optimized Tracking
								//										5 = Inactive
								//				..x. .... = headVehValid - heading of vehicle is valid, only set if the receiver is in sensor fusion mode
								//				xx.. .... = carrSoln - Carrier phase range solution status:
								//										0 = no carrier phase range solution
								//										1 = carrier phase range solution with floating ambiguities
								//										2 = carrier phase range solution with fixed ambiguities (not supported for protocol versions less than 20.00)


#define UBX_NAV_PVT_FLAGS2_MASK	0xe0
	uint8_t		flags2;			// Flags 2:
								//				..x. .... = Information about UTC Date and Time of Day validity confirmationn is available
								//				.x.. .... = UTC Date validity could be confirmed
								//				x... .... = UTC Time of Day could be confirmed
	uint8_t		numSV;			//			Number of satellites used in Nav Solution
	int32_t		lon;			// 1e-7 deg	Longitude
	int32_t		lat;			// 1e-7 deg	Latitude
	int32_t		height;			// mm		Height above ellipsoid
	int32_t		hMSL;			// mm		Height above mean sea level
	uint32_t	hAcc;			// mm		Horizontal accuracy estimate
	uint32_t	vAcc;			// mm		Vertical accuracy estimate
	int32_t		velN;			// mm/s		NED north velocity
	int32_t		velE;			// mm/s		NED east velocity
	int32_t		velD;			// mm/s		NED down velocity
	int32_t		gSpeed;			// mm/s		Ground Speed (2-D)
	int32_t		headMot;		// 1e-5 deg Heading of motion (2-D)
	uint32_t	sAcc;			// mm/s		Speed accuracy estimate
	uint32_t	headAcc;		// 1e-5 deg	Heading accuracy estimate (both motion and vehicle)
	uint16_t	pDOP;			// 0.01		Position DOP
	uint16_t	flags3;			// Flags 3:
								//				.... .... .... ...x = Invalid lon, lat, height and hMSL
								//				.... .... ...x xxx. = Age of the most recently received differential correction:
								//										0 = Not available
								//										1 = Age between 0 and 1 second
								//										2 = Age between 1 (inclusive) and 2 seconds
								//										3 = Age between 2 (inclusive) and 5 seconds
								//										4 = Age between 5 (inclusive) and 10 seconds
								//										5 = Age between 10 (inclusive) and 15 seconds
								//										6 = Age between 15 (inclusive) and 20 seconds
								//										7 = Age between 20 (inclusive) and 30 seconds
								//										8 = Age between 30 (inclusive) and 45 seconds
								//										9 = Age between 45 (inclusive) and 60 seconds
								//										10 = Age between 60 (inclusive) and 90 seconds
								//										11 = Age between 90 (inclusive) and 120 seconds
								//										>=12 = Age greater or equal than 120 seconds
								//				.x.. .... .... .... = Flag that indicates if the output time has been validated against an external trusted time source:
								//										0 = Time is not authenticated
								//										1 = Time is authenticated
#if 0 // Not supported in Standard precision GNSS product
	uint8_t		reserved0[4];	//			Reserved
	int32_t		headVeh;		// 1e-5 deg	Heading of vehicle (2-D), this is only valid when
								//			headVehValid is set, otherwise the output is set to the
								//			heading of motion
	int16_t		magDec;			// 1e-2	deg	Magnetic declination. Only supported in ADR 4.10 and later.
	uint16_t	magAcc;			// 1e-2	deg	Magnetic declination accuracy. Only supported in ADR 4.10 and later.
#endif
} __attribute__ ((packed));


static ssize_t ublox_sam_m10q_bin_attr_curr_pvt_read(struct file *filp, struct kobject *kobj, struct bin_attribute *bin_attr, char *buf, loff_t off, size_t count);



struct sciaps_attribute {
	struct attribute attr;
	struct ublox_sam_m10q_info_t* chip;
};

struct ublox_sam_m10q_info_t {
	struct i2c_client		*client;
	struct ubx_nav_pvt		curr_pvt;
	struct ubx_nav_pvt		last_valid_pvt;
	struct delayed_work		work;

	struct delayed_work shutdown_work;	/* Shutdown workcheduler */

	uint16_t	sciaps_support_shutdown;
	uint16_t	sciaps_shutdown_timer;
	uint8_t		shutdown_work_running;

	int			gpio_rst_pin;
	int			gpio_extint_pin;

	uint32_t	hAccRejectHardThreshold;	// mm
	uint16_t	pDOPRejectHardThreshold;	// 0.01
	uint16_t	pDOPRejectThreshold;		// 0.01
	uint32_t	hAccAcceptThreshold;		// mm
	uint16_t	nSatsAcceptAlwaysThreshold;	// # of sats


#define UBLOX_SAM_M10Q_STATUS_RESET						0x00
#define UBLOX_SAM_M10Q_STATUS_BIT_FIX_VALID				0x01
#define UBLOX_SAM_M10Q_STATUS_BIT_TIME_VALID			0x02
#define UBLOX_SAM_M10Q_STATUS_BIT_ASSIST_UTC_APPLIED	0x04
#define UBLOX_SAM_M10Q_STATUS_BIT_ASSIST_DATA_APPLIED	0x08

	uint8_t		status;
#define UBX_PM_CONTINUOUS	0
#define UBX_PM_SW_STANDBY	3
	uint8_t		power_mode;
	uint16_t	year;			// y        Year (UTC)
	uint8_t		month;			// month    Month, range 1..12 (UTC)
	uint8_t		day;			// d		Day of month, range 1..31 (UTC)
	uint8_t		hour;			// h		Hour of day, range 0..23 (UTC)
	uint8_t		min;			// min		Minute of hour, range 0..59 (UTC)
	uint8_t		sec;			// s		Seconds of minute, range 0..60 (UTC)

	uint32_t
				last_time_assistance;

	struct mutex lock;
	struct kobject		*gnss_kobj;
	struct bin_attribute	bin_attr_curr_pvt;
	uint8_t					curr_pvt_sysfs_ready;
	struct bin_attribute	bin_attr_last_valid_pvt;
	uint8_t					last_valid_pvt_sysfs_ready;
	struct bin_attribute	bin_attr_mga_ano;
	uint8_t					mga_ano_sysfs_ready;
	struct bin_attribute	bin_attr_mga_any;
	uint8_t					mga_any_sysfs_ready;
	struct sciaps_attribute	attrs[5];
};

//static void ublox_sam_m10q_bin_attr_curr_pvt_notify(struct ublox_sam_m10q_info_t *chip);

static int i2c_read_errs = 0;

#define UBX_SYNC_1			0xb5
#define UBX_SYNC_2			0x62
#define UBX_HEADER_LENGTH	6
#define UBX_CHECKSUM_LENGTH	2

#define UBX_MSG_LENGTH_MIN 8

#define UBX_MSG_LENGTH_MAX 300
static uint8_t s_ubx_msg[UBX_MSG_LENGTH_MAX];
static uint8_t s_ubx_data_rcvd[4096];
static uint16_t s_ubx_data_rcvd_length;

static void ublox_sam_m10q_calc_checksum(uint8_t* data, uint16_t length, uint8_t *CK_A_out, uint8_t *CK_B_out) {
	int i;
	if (!data || !CK_A_out || !CK_B_out)
		return;

	{
		uint8_t CK_A = 0, CK_B = 0;
		for (i = 2; i < length; i++) {
			CK_A = CK_A + data[i];
			CK_B = CK_B + CK_A;
		}
		*CK_A_out = CK_A;
		*CK_B_out = CK_B;
	}
}

static uint8_t* ublox_sam_m10q_make_ubx_msg(uint16_t class_mid, uint8_t* payload, uint16_t payload_len, uint8_t* ubx_msg, uint16_t* ubx_msg_length)
{
	uint16_t length_processed = 0;
	int i;

	if (ubx_msg && ((payload_len && payload) || payload_len == 0) && payload_len <= (UBX_MSG_LENGTH_MAX - (UBX_HEADER_LENGTH + UBX_CHECKSUM_LENGTH))) {
		ubx_msg[length_processed++] = UBX_SYNC_1;
		ubx_msg[length_processed++] = UBX_SYNC_2;
		ubx_msg[length_processed++] = (uint8_t)(class_mid>>8);
		ubx_msg[length_processed++] = (uint8_t)class_mid;
		ubx_msg[length_processed++] = (uint8_t)(payload_len);
		ubx_msg[length_processed++] = (uint8_t)(payload_len>>8);
		for(i = 0; i < payload_len; i++)
			ubx_msg[length_processed++] = payload[i];

		{
			uint8_t CK_A = 0, CK_B = 0;
			ublox_sam_m10q_calc_checksum(ubx_msg, length_processed, &CK_A, &CK_B);
			ubx_msg[length_processed++] = CK_A;
			ubx_msg[length_processed++] = CK_B;
		}

		if (ubx_msg_length)
			*ubx_msg_length = length_processed;
		return ubx_msg;
	}

	return NULL;
}

#if 1
static struct ublox_sam_m10q_info_t* ublox_sam_m10q_check_and_get_info(struct i2c_client *i2c)
{
	struct ublox_sam_m10q_info_t* info;
	//uint8_t device_id = SCIAPS_DRIVER_ID_SCIAPS_CNTLR_I2C;
	if (!i2c) {
		return NULL;
	}
	info = i2c_get_clientdata(i2c);
	//if (!info) {
	//	return NULL;
	//}
	//if (info->_device_id != device_id) {
	//	return NULL;
	//}
	return info;
}

static int ublox_sam_m10q_i2c_write_buffer(struct i2c_client *i2c, uint8_t* data, uint16_t data_len, bool lock_it)
{
	struct ublox_sam_m10q_info_t* info = ublox_sam_m10q_check_and_get_info(i2c);
	int ret;

	if (!info || !data || !data_len) {
		return -EINVAL;
	}

	ret = 0;
	if (lock_it)
		mutex_lock(&info->lock);
	{
		//uint8_t buf[SCIAPS_CNTLR_REG_Size + 1] = {reg, (uint8_t)(value), (uint8_t)(value>>8)};

		struct i2c_msg msgs[] = {
			{
				.addr   = i2c->addr,
				.flags  = 0,
				.len    = data_len,
				.buf    = data,
			},
		};

		if ((ret = i2c_transfer(i2c->adapter, msgs, 1)) < 0) {
			dev_err(&i2c->dev, "%s: error: %d\n", __func__, ret);
		}

	}
	if (lock_it)
		mutex_unlock(&info->lock);

	return ret;
}

static int ublox_sam_m10q_read_buffer(struct i2c_client *i2c, uint8_t reg, uint8_t* data, uint16_t data_len, bool lock_it)
{
	struct ublox_sam_m10q_info_t* info = ublox_sam_m10q_check_and_get_info(i2c);
	int ret;

	if (!info) {
		return -EINVAL;
	}

	ret = 0;
	if (lock_it)
		mutex_lock(&info->lock);
	{
		uint8_t addr[1] = {reg};

		struct i2c_msg msgs[] = {
			{
				.addr   = i2c->addr,
				.flags  = 0,
				.len    = 1,
				.buf    = addr,
			},
			{
				.addr   = i2c->addr,
				.flags  = I2C_M_RD,
				.len    = data_len,
				.buf    = data,
			}
		};

		if ((ret = i2c_transfer(i2c->adapter, msgs, 2)) < 0) {
			dev_err(&i2c->dev, "%s: error: %d\n", __func__, ret);
		}
	}
	if (lock_it)
		mutex_unlock(&info->lock);

	return ret;
}
#else
static int ublox_sam_m10q_read_byte_data(struct i2c_client *client, u8 address, uint8_t* out)
{
	s32 ret = i2c_smbus_read_byte_data(client, address);

	if (ret < 0) {
		dev_err(&client->dev,
			"%s: i2c read at address 0x%x failed: 0x%x(%d)\n",
			__func__, address, ret, ret);
	} else {
		*out = (uint8_t)(0xFFFF & ret);
	}

	return ret;
}
#endif
#if 0
static int ublox_sam_m10q_read_word_data(struct i2c_client *client, u8 address, s16* out)
{
	s32 ret = i2c_smbus_read_word_data(client, address);

	if (ret < 0) {
		dev_err(&client->dev,
			"%s: i2c read at address 0x%x failed: 0x%x\n",
			__func__, address, ret);
	} else {
		*out = (s16)(0xFFFF & ret);
	}

	return ret;
}
#endif
/*
static int ublox_sam_m10q_write_word_data(struct i2c_client *client, u8 address,
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

#define UBX_CFG_RATE_MEAS_VALUE					250	// ms
#define UBX_CFG_RATE_NAV_VALUE					4
#define UBX_CFG_NAVSPG_DYNMODEL_VALUE_PORT		0
#define UBX_CFG_NAVSPG_FIXMODE_VALUE_AUTO		3

#define UBX_CFG_TXREADY_PIN_VALUE_TXD			1		// TXD
#define UBX_CFG_TXREADY_THRES_VALUE				1		// in 8-byte chunks

#define UBX_CFG_VALSET			0x068a
#define UBX_CFG_VALGET			0x068b
#define UBX_MON_GNSS			0x0a28
#define UBX_MON_VER				0x0a04
#define UBX_CFG_RST				0x0604
#define UBX_RXM_PMREQ			0x0241
#define UBX_MGA_INI_TIME_UTC	0x1340

static uint8_t s_default_config[] = {
					//header
					0x00 // set with no transactions
						, 0x03 //  Store in RAM and BBR
						, 0x00, 0x00 // reserved . must be 0x00, 0x00
					//Data
					, 0x06, 0x00, 0x92, 0x20, 0x00	// 0x20920006
					, 0x07, 0x00, 0x92, 0x20, 0x00	// 0x20920007
					, 0x0a, 0x00, 0x92, 0x20, 0x00	// 0x2092000a
					, 0x01, 0x00, 0x92, 0x20, 0x07  // 0x20920001 (ERROR | WARNING | NOTICE)
					, 0x22, 0x00, 0x31, 0x10, 0x01	// 0x10310022 // CFG-SIGNAL-BDS_ENA
					, 0x25, 0x00, 0x31, 0x10, 0x01	// 0x10310025
					, 0x03, 0x00, 0x51, 0x10, 0x01							// CFG-I2C-ENABLED
					, 0x01, 0x00, 0x71, 0x10, 0x01							// CFG-I2CINPROT-UBX
					, 0x02, 0x00, 0x71, 0x10, 0x00							// CFG-I2CINPROT-NMEA
					, 0x01, 0x00, 0x72, 0x10, 0x01							// CFG-I2COUTPROT-UBX
					, 0x02, 0x00, 0x72, 0x10, 0x00							// CFG-I2COUTPROT-NMEA
				//	, 0x21, 0x00, 0x11, 0x20, UBX_CFG_NAVSPG_DYNMODEL_VALUE_PORT	// CFG-NAVSPG-DYNMODEL - key id: 0x20110021 type: E1 scale: - unit: -
				//	, 0x11, 0x00, 0x11, 0x20, UBX_CFG_NAVSPG_FIXMODE_VALUE_AUTO	// CFG-NAVSPG-FIXMODE - key id: 0x20110011 type: E1 scale: - unit: -
					, 0x01, 0x00, 0x21, 0x30, (uint8_t)(UBX_CFG_RATE_MEAS_VALUE), (uint8_t)(UBX_CFG_RATE_MEAS_VALUE>>8) // CFG-RATE-MEAS - key id: 0x30210001 type: U2 scale: 0.001 unit: s
					, 0x02, 0x00, 0x21, 0x30, (uint8_t)(UBX_CFG_RATE_NAV_VALUE), (uint8_t)(UBX_CFG_RATE_NAV_VALUE>>8)		// CFG-RATE-NAV - key id: 0x30210002 type: U2 scale: - unit: -
					, 0x06, 0x00, 0x91, 0x20, 0x01								// CFG-MSGOUT-UBX_NAV_PVT_I2C - key id: 0x20910006 type: U1 scale: - unit: -
					, 0x05, 0x00, 0x52, 0x10, 0x00								// CFG-UART1-ENABLED - key id: 0x10520005 type: L scale: - unit: -
					, 0x01, 0x00, 0xa2, 0x10, 0x01								// CFG-TXREADY-ENABLED 0x10a20001 type: L scale: - unit: -
					, 0x02, 0x00, 0xa2, 0x10, 0x00								// CFG-TXREADY-POLARITY 0x10a20002 type: L scale: - unit: - desc: The polarity of the TX ready pin: false:high-active, true:low-active. AD : Actually 0x00 is low -active!
					, 0x03, 0x00, 0xa2, 0x20, UBX_CFG_TXREADY_PIN_VALUE_TXD		// CFG-TXREADY-PIN 0x20a20003 type: U1 scale: - unit: - desc: Pin number to use for the TX ready functionality
					, 0x05, 0x00, 0xa2, 0x20, 0x00								// CFG-TXREADY-INTERFACE 0x20a20005 type: E1 scale: - unit: - desc: Interface where the TX ready feature should be linked to. Set to oxoo which is I2C
					, 0x04, 0x00, 0xa2, 0x30, (uint8_t)(UBX_CFG_TXREADY_THRES_VALUE), (uint8_t)(UBX_CFG_TXREADY_THRES_VALUE>>8)			// CFG-TXREADY-THRESHOLD 0x30a20004 type: U2 scale: - unit: -
					, 0x25, 0x00, 0x11, 0x10, 0x01		// CFG-NAVSPG-ACKAIDING 0x10110025 L--Acknowledge assistance input messages
				};

static uint8_t s_read_config[512];
#if 0
 = {
					//header
					0x00
						, 0x07 // Deefault layer
						, 0x00, 0x00 // Nothing to skip
					//Data
					, 0xb1, 0x00, 0x11, 0x30									//CFG-NAVSPG-OUTFIL_PDOP	0x301100b1	U2	0.1	-	Output ﬁlter position DOP mask (threshold)
					, 0xb3, 0x00, 0x11, 0x30									//CFG-NAVSPG-OUTFIL_PACC	0x301100b3	U2	-	m	Output ﬁlter position accuracy mask (threshold)
					, 0x21, 0x00, 0x11, 0x20 //, UBX_CFG_NAVSPG_DYNMODEL_VALUE_PORT	// CFG-NAVSPG-DYNMODEL - key id: 0x20110021 type: E1 scale: - unit: -
					, 0x11, 0x00, 0x11, 0x20 //, UBX_CFG_NAVSPG_FIXMODE_VALUE_AUTO	// CFG-NAVSPG-FIXMODE - key id: 0x20110011 type: E1 scale: - unit: -
					, 0x01, 0x00, 0x21, 0x30 //, (uint8_t)(UBX_CFG_RATE_MEAS_VALUE), (uint8_t)(UBX_CFG_RATE_MEAS_VALUE>>8) // CFG-RATE-MEAS - key id: 0x30210001 type: U2 scale: 0.001 unit: s
					, 0x02, 0x00, 0x21, 0x30 //, (uint8_t)(UBX_CFG_RATE_NAV_VALUE), (uint8_t)(UBX_CFG_RATE_NAV_VALUE>>8)		// CFG-RATE-NAV - key id: 0x30210002 type: U2 scale: - unit: -
#define 0x1031001f
0x10310001
0x10310020
0x10310005
0x10310021
0x10310007
0x10310022
0x1031000d
0x1031000f
0x10310024
0x10310012
0x10310014
0x10310025
0x10310018








				};
#endif

#define CFG_NAVSPG_OUTFIL_PDOP		0x301100b1			// U2	0.1	-	Output ﬁlter position DOP mask (threshold)
#define CFG_NAVSPG_OUTFIL_PACC		0x301100b3			// U2	-	m	Output ﬁlter position accuracy mask (threshold)
#define CFG_SIGNAL_GPS_ENA			0x1031001f			// L    -   -   GPS enable
#define CFG_SIGNAL_GPS_L1CA_ENA		0x10310001			// L	-	-	GPS L1C/A
#define CFG_SIGNAL_SBAS_ENA			0x10310020			// L	-	-	SBAS enable
#define CFG_SIGNAL_SBAS_L1CA_ENA	0x10310005			// L	-	-	SBAS L1C/A
#define CFG_SIGNAL_GAL_ENA			0x10310021			// L	-	-	Galileo enable
#define CFG_SIGNAL_GAL_E1_ENA		0x10310007			// L	-	-	Galileo E1
#define CFG_SIGNAL_BDS_ENA			0x10310022			// L	-	-	BeiDou Enable
#define CFG_SIGNAL_BDS_B1_ENA		0x1031000d			// L	-	-	BeiDou B1I
#define CFG_SIGNAL_BDS_B1C_ENA		0x1031000f			// L	-	-	BeiDou B1C
#define CFG_SIGNAL_QZSS_ENA			0x10310024			// L	-	-	QZSS enable
#define CFG_SIGNAL_QZSS_L1CA_ENA	0x10310012			// L	-	-	QZSS L1C/A
#define CFG_SIGNAL_QZSS_L1S_ENA		0x10310014			// L	-	-	QZSS L1S
#define CFG_SIGNAL_GLO_ENA			0x10310025			// L	-	-	GLONASS enable
#define CFG_SIGNAL_GLO_L1_ENA		0x10310018			// L	-	-	GLONASS L1

static uint8_t s_default_config_status;

#define UBLOX_CONFIG_STATUS_INIT		0
#define UBLOX_CONFIG_STATUS_APPLYING	1
#define UBLOX_CONFIG_STATUS_READY		2
#define UBLOX_CONFIG_STATUS_VERIFYING	3
#define UBLOX_CONFIG_STATUS_VERIFIED	4

static void dev_info_data(struct ublox_sam_m10q_info_t* chip, char* prefix, uint8_t* data, uint16_t data_len)
{
	static char str[16*3], str_byte[10];
	int i;

	str[0] = 0;
	for(i = 0; i < data_len; i++) {
		sprintf(str_byte, " %.2x", data[i]);
		strcat(str, str_byte);
		if ((i % 15) == 0 && i > 0) {
			dev_info(&chip->client->dev, "%s: %s\n", prefix, str);
			str[0] = 0;
		}
	}
	if (str[0]) {
		dev_info(&chip->client->dev, "%s: %s\n", prefix, str);
		str[0] = 0;
	}

}

static void dev_dbg_data(struct ublox_sam_m10q_info_t* chip, char* prefix, uint8_t* data, uint16_t data_len)
{
	static char str[16*3], str_byte[10];
	int i;

	str[0] = 0;
	for(i = 0; i < data_len; i++) {
		sprintf(str_byte, " %.2x", data[i]);
		strcat(str, str_byte);
		if ((i % 15) == 0 && i > 0) {
			DEV_DBG(&chip->client->dev, "%s: %s\n", prefix, str);
			str[0] = 0;
		}
	}
	if (str[0]) {
		DEV_DBG(&chip->client->dev, "%s: %s\n", prefix, str);
		str[0] = 0;
	}

}

static bool	verify_ubx_nav_pvt(struct ublox_sam_m10q_info_t* chip, struct ubx_nav_pvt* ubx_msg)
{
	bool good_one_maybe = false;


	if (UBX_NAV_PVT_FIX_OK(ubx_msg->fixType)) {
		if (ubx_msg->numSV >= chip->nSatsAcceptAlwaysThreshold) {
			// Accept - at least for now
			good_one_maybe = true;
		}
		else if (ubx_msg->hAcc >= chip->hAccRejectHardThreshold) {
			// Reject
		}
		else if (ubx_msg->pDOP >= chip->pDOPRejectHardThreshold) {
			// Reject
		}
		else if (ubx_msg->hAcc < chip->hAccAcceptThreshold) {
			// Accept
			good_one_maybe = true;
		}
		else if (ubx_msg->pDOP >= chip->pDOPRejectThreshold) {
			// Reject
		}
		else {
			// Accept
			good_one_maybe = true;
		}
	}

	if (good_one_maybe) {
		ubx_msg->flags |=  UBX_NAV_PVT_FLAGS_FIX_VALID;
	}
	else {
		ubx_msg->flags &=  ~UBX_NAV_PVT_FLAGS_FIX_VALID;
	}
	return good_one_maybe;
}

#define	UBX_INF_STR_MAX_LENGTH 256
static char s_ubx_inf_str[UBX_INF_STR_MAX_LENGTH + 1];

static bool	parse_ubx_msgs(struct ublox_sam_m10q_info_t* chip)
{
	uint8_t* data = s_ubx_data_rcvd;
	uint16_t length = s_ubx_data_rcvd_length;
	uint16_t payload_len;
	uint8_t class, id;
	bool got_it;
	bool rv = false;

	if (length >= UBX_MSG_LENGTH_MIN) {
		int i;
keep_processing:
		// find start of ubx packet.
		got_it = false;
		for (i = 0; i < length - 1; i++) {
			if (data[i] == UBX_SYNC_1 && data[i + 1] == UBX_SYNC_2) {
				data += i;
				length -= i;
				got_it = true;
				break;
			}
		}
		if (!got_it) {
			//nothing to do
			s_ubx_data_rcvd_length = 0;
		}
		else if  (length >= UBX_MSG_LENGTH_MIN) {
			class = data[2];
			id = data[3];
			payload_len = data[5]; payload_len <<= 8; payload_len |= data[4];
			if (UBX_MSG_LENGTH_MIN + payload_len <= length) {
				uint8_t CK_A, CK_B;

				ublox_sam_m10q_calc_checksum(data, payload_len + 6, &CK_A, &CK_B);

				if(data[6 + payload_len] == CK_A && data[6 + payload_len + 1] == CK_B) {
					//checksum is good. Process it!!!!
					DEV_DBG(&chip->client->dev, "UBX message recvd: 0x%.2x:0x%.2x - %d \n", class, id, payload_len);
					rv = true;
					if (class == 0x05 && id == 0x01) {
						//UBX-ACK-ACK
						dev_info_data(chip, "UBX-ACK Rcvd", data, UBX_MSG_LENGTH_MIN + payload_len);
						// we just reading two specific values for now: but we are to make it generic
						if (payload_len == 2)  {
							if(data[6] == (uint8_t)(UBX_CFG_VALSET >> 8) && data[7] == (uint8_t)(UBX_CFG_VALSET)) {
								dev_info(&chip->client->dev, "%s: UBX-ACK for UBX_CFG_VALSET recvd", __func__);
								s_default_config_status = UBLOX_CONFIG_STATUS_READY;
							}
							else if(data[6] == (uint8_t)(UBX_CFG_VALGET >> 8) && data[7] == (uint8_t)(UBX_CFG_VALGET)) {
								dev_info(&chip->client->dev, "%s: UBX-ACK for UBX_CFG_VALGET recvd", __func__);
								s_default_config_status = UBLOX_CONFIG_STATUS_VERIFIED;
								//s_default_config_status = UBLOX_CONFIG_STATUS_READY;
							}
							else if(data[6] == (uint8_t)(UBX_MON_GNSS >> 8) && data[7] == (uint8_t)(UBX_MON_GNSS)) {
								dev_info(&chip->client->dev, "%s: UBX-ACK for UBX_MON_GNSS recvd", __func__);
								//s_default_config_status = UBLOX_CONFIG_STATUS_READY;
							}
							else if(data[6] == (uint8_t)(UBX_MON_VER >> 8) && data[7] == (uint8_t)(UBX_MON_VER)) {
								dev_info(&chip->client->dev, "%s: UBX-ACK for UBX_MON_VER recvd", __func__);
								//s_default_config_status = UBLOX_CONFIG_STATUS_READY;
							}
							else if(data[6] == (uint8_t)(UBX_MGA_INI_TIME_UTC >> 8) && data[7] == (uint8_t)(UBX_MGA_INI_TIME_UTC)) {
								dev_info(&chip->client->dev, "%s: UBX-ACK for UBX_MGA_INI_TIME_UTC recvd", __func__);
								//s_default_config_status = UBLOX_CONFIG_STATUS_READY;
							}
						}
					}
					else if (class == 0x13 && id == 0x60) { // UBX-MGA-ACK (0x13 0x60)
						dev_info_data(chip, "UBX-MGA-ACK (0x13 0x60) Rcvd", data, UBX_MSG_LENGTH_MIN + payload_len);
					}
					else if (class == 0x0a && id == 0x28) {
						dev_info_data(chip, "UBX-MON-GNSS Rcvd", data, UBX_MSG_LENGTH_MIN + payload_len);
					}
					else if (class == 0x0a && id == 0x04) {
						uint16_t str_len, o = 0;
						dev_dbg_data(chip, "UBX-MON-VER Rcvd", data, UBX_MSG_LENGTH_MIN + payload_len);

						str_len  = 30;
						if (o + str_len <= payload_len) {
							memcpy(s_ubx_inf_str, (data + 6 + o), str_len);
							s_ubx_inf_str[str_len] = 0;
							dev_info(&chip->client->dev, "--->> UBX-MON-VER swVersion: %s", s_ubx_inf_str);
							o += str_len;
						}

						str_len  = 10;
						if (o + str_len <= payload_len) {
							memcpy(s_ubx_inf_str, (data + 6 + o), str_len);
							s_ubx_inf_str[str_len] = 0;
							dev_info(&chip->client->dev, "--->> UBX-MON-VER hwVersion: %s", s_ubx_inf_str);
							o += str_len;
						}
						str_len = 30;
						for (;o + str_len <= payload_len; o += str_len) {
							memcpy(s_ubx_inf_str, (data + 6 + o), str_len);
							s_ubx_inf_str[str_len] = 0;
							dev_info(&chip->client->dev, "--->> UBX-MON-VER extension: %s", s_ubx_inf_str);
							o += str_len;
						}

					}
					else if (class == 0x04) {
						uint16_t str_len = UBX_INF_STR_MAX_LENGTH;
						dev_dbg_data(chip, "============> UBX-INF-xxx Rcvd", data, UBX_MSG_LENGTH_MIN + payload_len);
						if (payload_len < UBX_INF_STR_MAX_LENGTH)
							str_len = payload_len;
						memcpy(s_ubx_inf_str, (data + 6), str_len);
						s_ubx_inf_str[str_len] = 0;
						dev_info(&chip->client->dev, "============> UBX-INF-xxx Rcvd(class: 0x%02x, id: 0x%02x): %s", class, id, s_ubx_inf_str);
					}
					else if (class == 0x06 && id == 0x8b) {
						dev_dbg_data(chip, "UBX-CFG-VALGET Rcvd", data, UBX_MSG_LENGTH_MIN + payload_len);
						// we just reading two specific values for now: but we are to make it generic
						if (payload_len >= 0x10 && data[6] == 0x01)  {
							uint16_t pdop_thres = 0;
							uint16_t pacc_thres = 0;
							int o = 10;
							if ( *(uint32_t*)(data + o) == CFG_NAVSPG_OUTFIL_PDOP)
									pdop_thres = *(uint16_t*)(data + o + 4);
							else if ( *(uint32_t*)(data + o) == CFG_NAVSPG_OUTFIL_PACC)
									pdop_thres = *(uint16_t*)(data + o + 4);
							o += 6;

							if ( *(uint32_t*)(data + o) == CFG_NAVSPG_OUTFIL_PDOP)
									pacc_thres = *(uint16_t*)(data + o + 4);
							else if ( *(uint32_t*)(data + o) == CFG_NAVSPG_OUTFIL_PACC)
									pacc_thres = *(uint16_t*)(data + o + 4);
							o += 6;

							DEV_DBG(&chip->client->dev, "UBX-CFG-VALGET Rcvd ===> PDOP Threshold: %d; pAcc Threshold: %d m"
																	,pdop_thres, pacc_thres);
						}

					}
					else if (class == 0x01 && id == 0x07) {
						struct ubx_nav_pvt* ubx_msg = (struct ubx_nav_pvt*)(data+6);
						uint8_t status  = 0, prev_status;
#if UBLOX_SAM_M10Q_DEBUG
						struct timeval	time;
#endif

						verify_ubx_nav_pvt(chip, ubx_msg);

						if((ubx_msg->flags & UBX_NAV_PVT_FLAGS_FIX_VALID) == UBX_NAV_PVT_FLAGS_FIX_VALID || UBX_NAV_PVT_FIX_OK(ubx_msg->fixType)) {
							status |= UBLOX_SAM_M10Q_STATUS_BIT_FIX_VALID;
						}

						if((ubx_msg->valid & 0x03) == 0x03) {
							status |= UBLOX_SAM_M10Q_STATUS_BIT_TIME_VALID;
						}

						mutex_lock(&chip->lock);
						prev_status = chip->status;
						chip->status |= status;
						status = chip->status;

						if((ubx_msg->valid & 0x03) == 0x03) {
							chip->year	= ubx_msg->year;
							chip->month = ubx_msg->month;
							chip->day	= ubx_msg->day;
							chip->hour	= ubx_msg->hour;
							chip->min	= ubx_msg->min;
							chip->sec	= ubx_msg->sec;
						}

						memcpy(&chip->curr_pvt, ubx_msg, sizeof (struct ubx_nav_pvt));
						if((ubx_msg->flags & UBX_NAV_PVT_FLAGS_FIX_VALID) == UBX_NAV_PVT_FLAGS_FIX_VALID || (chip->status & UBLOX_SAM_M10Q_STATUS_BIT_FIX_VALID) == 0) {
							memcpy(&chip->last_valid_pvt, ubx_msg, sizeof (struct ubx_nav_pvt));
						}

						mutex_unlock(&chip->lock);

						if (prev_status != status) {
							// notify!
						}

#if (UBLOX_SAM_M10Q_DEBUG >= UBLOX_SAM_M10Q_DEBUG_LEVEL_1)
						do_gettimeofday(&time);

						dev_info(&chip->client->dev, "UBX-NAV-PVT ===> itow: %d - %d/%d/%d %d:%d:%d(%d ns - %.2x - %d ns) - kernel time: %ld-%ld"
																	,ubx_msg->itow
																	,ubx_msg->year
																	,ubx_msg->month
																	,ubx_msg->day
																	,ubx_msg->hour
																	,ubx_msg->min
																	,ubx_msg->sec
																	,ubx_msg->nano
																	,(ubx_msg->valid & UBX_NAV_PVT_VALID_MASK)
																	,ubx_msg->tAcc
																	,time.tv_sec,time.tv_usec
																					);
						dev_info(&chip->client->dev, "UBX-NAV-PVT ===> itow: %d - fix: %d (%.2x:%.2x) - numSV: %d - lon: %d - lat: %d - hMSL: %d mm - hAcc: %d mm - vAcc: %d mm - pDOP: %d"
																	,ubx_msg->itow
																	,ubx_msg->fixType
																	,(ubx_msg->flags & UBX_NAV_PVT_FLAGS_MASK)
																	,(ubx_msg->flags2 & UBX_NAV_PVT_FLAGS2_MASK)
																	,ubx_msg->numSV
																	,ubx_msg->lon
																	,ubx_msg->lat
																	//	,ubx_msg->height
																	,ubx_msg->hMSL
																	,ubx_msg->hAcc
																	,ubx_msg->vAcc
																	,ubx_msg->pDOP
																					);
#endif

#if (UBLOX_SAM_M10Q_DEBUG >= UBLOX_SAM_M10Q_DEBUG_LEVEL_2)
						dev_info(&chip->client->dev, "UBX-NAV-PVT ===> itow: %d - gspeed: %d mm/s - head: %d 1e-5 deg - sAcc %d mm/s - headAcc: %d 1e-5 deg - flags3: %.2x"
																	,ubx_msg->itow
																	//,ubx_msg->velN
																	//,ubx_msg->velEx
																	//,ubx_msg->velD
																	,ubx_msg->gSpeed
																	,ubx_msg->headMot
																	,ubx_msg->sAcc
																	,ubx_msg->headAcc
																	,ubx_msg->flags3
																					);
#endif

#if 0 // Not supported in Standard precision GNSS product
						DEV_DBG(&chip->client->dev, "UBX-NAV-PVT ===> itow: %d - headVeh: %d 1e-5deg - magDec: %d 1e-2 deg - magAcc: %d 1e-2 deg",
																	ubx_msg->itow,
																	ubx_msg->headVeh,
																	ubx_msg->magDec,
																	ubx_msg->magAcc
																					);
#endif
					}
					data += (6 + payload_len + 2);
					length -= (6 + payload_len + 2);
					if (length >= UBX_MSG_LENGTH_MIN) {
						goto keep_processing;
					}
					else {
						s_ubx_data_rcvd_length = length;
						if (s_ubx_data_rcvd_length)
							memmove(s_ubx_data_rcvd, data, s_ubx_data_rcvd_length);
					}
				}
				else {
					// Just find the next start of ubx packet
					data += 2;
					length  -= 2;
					goto keep_processing;
				}

			}
			else {
				//still incomplete
				if (data !=  s_ubx_data_rcvd) {
					s_ubx_data_rcvd_length = length;
					if (s_ubx_data_rcvd_length)
						memmove(s_ubx_data_rcvd, data, s_ubx_data_rcvd_length);
				}
				else {
					// not sure if it is needed
					s_ubx_data_rcvd_length = length;
				}

			}
		}
		else {
			//incomplete
			if (data !=  s_ubx_data_rcvd) {
				s_ubx_data_rcvd_length = length;
				if (s_ubx_data_rcvd_length)
					memmove(s_ubx_data_rcvd, data, s_ubx_data_rcvd_length);
			}
			else {
				// not sure if it is needed
				s_ubx_data_rcvd_length = length;
			}
		}
	}
	return rv;
}

static void ublox_sam_m10q_read_config (struct ublox_sam_m10q_info_t *chip)
{
	int ret;
	if (s_default_config_status >= UBLOX_CONFIG_STATUS_READY && s_default_config_status < UBLOX_CONFIG_STATUS_VERIFIED) {
		uint16_t data_len, config_len = 0;
		uint8_t* data;

		s_default_config_status = UBLOX_CONFIG_STATUS_VERIFYING;

		{
			data = ublox_sam_m10q_make_ubx_msg(UBX_MON_GNSS, NULL, 0, s_ubx_msg, &data_len);
			if (data) {
				dev_info_data(chip, "Sending UBX_MON_GNSS", data, data_len);
				ret = ublox_sam_m10q_i2c_write_buffer(chip->client, data,  data_len, false);
				if (ret < 0) {
					//----s_default_config_status = UBLOX_CONFIG_STATUS_INIT;
				}
				else {
					//check on ACK
					//---s_default_config_status = UBLOX_CONFIG_STATUS_APPLYING;
				}
			}
			config_len = 0;
		}

		{
			data = ublox_sam_m10q_make_ubx_msg(UBX_MON_VER, NULL, 0, s_ubx_msg, &data_len);
			if (data) {
				dev_info_data(chip, "Sending UBX_MON_VER", data, data_len);
				ret = ublox_sam_m10q_i2c_write_buffer(chip->client, data,  data_len, false);
				if (ret < 0) {
					//----s_default_config_status = UBLOX_CONFIG_STATUS_INIT;
				}
				else {
					//check on ACK
					//---s_default_config_status = UBLOX_CONFIG_STATUS_APPLYING;
				}
			}
			config_len = 0;
		}

		//header
		s_read_config[config_len++] = 0x00;
		s_read_config[config_len++] = 0x07; // Deefault layer
		s_read_config[config_len++] = 0x00; // Nothing to skip
		s_read_config[config_len++] = 0x00;
		//Data
		*(uint32_t*)(s_read_config + config_len) = CFG_NAVSPG_OUTFIL_PDOP;
		config_len += 4;
		*(uint32_t*)(s_read_config + config_len) = CFG_NAVSPG_OUTFIL_PACC;
		config_len += 4;
		*(uint32_t*)(s_read_config + config_len) = 0x20110021;
		config_len += 4;
		*(uint32_t*)(s_read_config + config_len) = 0x20110011;
		config_len += 4;
		*(uint32_t*)(s_read_config + config_len) = 0x30210001;
		config_len += 4;
		*(uint32_t*)(s_read_config + config_len) = 0x30210002;
		config_len += 4;
       *(uint32_t*)(s_read_config + config_len) = CFG_SIGNAL_GPS_ENA;
		config_len += 4;
       *(uint32_t*)(s_read_config + config_len) = CFG_SIGNAL_GPS_L1CA_ENA;
		config_len += 4;
       *(uint32_t*)(s_read_config + config_len) = CFG_SIGNAL_SBAS_ENA;
		config_len += 4;
       *(uint32_t*)(s_read_config + config_len) = CFG_SIGNAL_SBAS_L1CA_ENA;
		config_len += 4;
       *(uint32_t*)(s_read_config + config_len) = CFG_SIGNAL_GAL_ENA;
		config_len += 4;
       *(uint32_t*)(s_read_config + config_len) = CFG_SIGNAL_GAL_E1_ENA;
		config_len += 4;
       *(uint32_t*)(s_read_config + config_len) = CFG_SIGNAL_BDS_ENA;
		config_len += 4;
       *(uint32_t*)(s_read_config + config_len) = CFG_SIGNAL_BDS_B1_ENA;
		config_len += 4;
       *(uint32_t*)(s_read_config + config_len) = CFG_SIGNAL_BDS_B1C_ENA;
		config_len += 4;
       *(uint32_t*)(s_read_config + config_len) = CFG_SIGNAL_QZSS_ENA;
		config_len += 4;
       *(uint32_t*)(s_read_config + config_len) = CFG_SIGNAL_QZSS_L1CA_ENA;
		config_len += 4;
       *(uint32_t*)(s_read_config + config_len) = CFG_SIGNAL_QZSS_L1S_ENA;
		config_len += 4;
       *(uint32_t*)(s_read_config + config_len) = CFG_SIGNAL_GLO_ENA;
		config_len += 4;
       *(uint32_t*)(s_read_config + config_len) = CFG_SIGNAL_GLO_L1_ENA;
		config_len += 4;

		data = ublox_sam_m10q_make_ubx_msg(UBX_CFG_VALGET, s_read_config, config_len, s_ubx_msg, &data_len);
		if (data) {
			dev_dbg_data(chip, "Sending BX_CFG_VALGET", data, data_len);
			ret = ublox_sam_m10q_i2c_write_buffer(chip->client, data,  data_len, false);
			if (ret < 0) {
				//----s_default_config_status = UBLOX_CONFIG_STATUS_INIT;
			}
			else {
				//check on ACK
				//---s_default_config_status = UBLOX_CONFIG_STATUS_APPLYING;
			}
		}
	}
}

static void ublox_sam_m10q_apply_default_config (struct ublox_sam_m10q_info_t *chip)
{
	int ret;
	if (s_default_config_status < UBLOX_CONFIG_STATUS_READY) {
		uint16_t data_len;
		uint8_t* data;

		data = ublox_sam_m10q_make_ubx_msg(UBX_CFG_VALSET, s_default_config, sizeof (s_default_config) / sizeof (*s_default_config), s_ubx_msg, &data_len);
		if (data) {
			dev_dbg_data(chip, "ubx_defualt_config", data, data_len);
			ret = ublox_sam_m10q_i2c_write_buffer(chip->client, data,  data_len, false);
			if (ret < 0) {
				s_default_config_status = UBLOX_CONFIG_STATUS_INIT;
			}
			else {
				//check on ACK
				s_default_config_status = UBLOX_CONFIG_STATUS_APPLYING;
			}
		}
	}
}

static void ublox_sam_m10q_use_utc_from_system_rtc (struct ublox_sam_m10q_info_t *chip, bool set, bool apply)
{
	if ((chip->status & UBLOX_SAM_M10Q_STATUS_BIT_TIME_VALID) == 0) {
		struct timeval	time;
		struct tm tm_time;
		const unsigned long abs_threshold		= 1707346020;
		const unsigned long resend_threshold	= 15;

		do_gettimeofday(&time);

		dev_dbg(&chip->client->dev,
				"%s: time.tv_sec: %ld; time.tv_usec: %ld; threshold: %ld;\n", __func__, time.tv_sec, time.tv_usec, abs_threshold);

		if (time.tv_sec <= abs_threshold) {
			return;
		}

		if ((chip->last_time_assistance + resend_threshold) > time.tv_sec) {
			return;
		}

		time_to_tm(time.tv_sec, 0, &tm_time);

		{
			int ret;
			uint8_t msg[26];
			uint16_t data_len;
			uint8_t* data;
			uint16_t	year = 1900 + tm_time.tm_year;
			uint8_t		month = (tm_time.tm_mon + 1), day = tm_time.tm_mday, hour = tm_time.tm_hour, min = tm_time.tm_min, sec = tm_time.tm_sec;
			uint16_t tAccS = 30;

			if (set) {
				mutex_lock(&chip->lock);
				chip->status |= UBLOX_SAM_M10Q_STATUS_BIT_ASSIST_UTC_APPLIED;
				chip->year	= year;
				chip->month = month;
				chip->day	= day;
				chip->hour	= hour;
				chip->min	= min;
				chip->sec	= sec;
				mutex_unlock(&chip->lock);
			}

			if (apply) {
				uint8_t  payload[] = { 0x10						// message type. must be 0x10
										, 0x00					// version. must be 0x00
										, 0x00					// reference: none
										, 0x80					// leapSecs
										, (uint8_t)year
										, (uint8_t)(year>>8)
										, month
										, day
										, hour
										, min
										, sec
										, 0x01					// trustedSource
										, 0x00, 0x00, 0x00, 0x00
										, (uint8_t)tAccS, (uint8_t)(tAccS>>8)
									};

				data_len = sizeof (msg) / sizeof (*msg);
				data = ublox_sam_m10q_make_ubx_msg(UBX_MGA_INI_TIME_UTC, payload, sizeof (payload) / sizeof (*payload), msg, &data_len);
				if (data) {
					dev_info_data(chip, "UBX-MGA-INI-TIME_UTC", data, data_len);
					ret = ublox_sam_m10q_i2c_write_buffer(chip->client, data,  data_len, false);
					if (ret < 0) {
						dev_info(&chip->client->dev, "%s: UBX-MGA-INI-TIME_UTC failed to sent! ret: %d\n", __func__, ret);
					}
					else {
						chip->last_time_assistance = time.tv_sec;
						dev_info(&chip->client->dev, "%s: UBX-MGA-INI-TIME_UTC Sent!!!\n", __func__);
					}
				}
			}
		}
	}
}

static void shutdown_work_func(struct work_struct *work)
{
	//sciaps_device_power_off(SCIAPS_DEVICE_POWER_OFF_OPT_SRC_BatteryRemoved);
}

static void ublox_sam_m10q_delayed_work(struct work_struct *work)
{
	#define CHECK_READ_BYTE(reg, location, min, max) \
	ret = ublox_sam_m10q_read_byte_data(chip->client, reg, &ival, false); \
	if(ret >= 0) { \
		DEV_DBG(&chip->client->dev, "read " #reg " = 0x%x(%d)\n", ival, ival); \
		if((max == 0 && min == 0) || (ival >= min && ival < max)) { \
			location = ival; \
		}	\
	}
	#define UBX_CFG_SET_DATA_U8(data, key_id, u8_value) \
		data[0] = (uint8_t)(key_id >> 24); data[1] = (uint8_t)(key_id >> 16); data[2] = (uint8_t)(key_id >> 8); data[3] = (uint8_t)(key_id); data[4] = (uint8_t)u8_value; cfg_msg_len = 5;
	// Roughly 60 seconds at 2s poll freq
	#define READ_ERR_LIMIT 30

	struct ublox_sam_m10q_info_t *chip;
	//bool changed = false;
	int ret;
	//uint8_t ival;
	//bool no_battery = false;
	//uint32_t capacity_low_thres_mAh = 0;

	chip = container_of(work, struct ublox_sam_m10q_info_t, work.work);

	//CHECK_READ_BYTE(0xfd, cache.data, 0, 0)

	{
		uint8_t reg = 0xfd;

		//if (ret >=0)
		if (chip->power_mode == UBX_PM_CONTINUOUS) {
			uint16_t	data_len_avail = 0;
			uint8_t		data_len_avail_buffer[2];

			reg = 0xfd;
			ret = ublox_sam_m10q_read_buffer(chip->client, reg,  data_len_avail_buffer, 2, false);

			if (ret >= 0 ) {
				//dev_dbg_data(chip, "read: ", data_len_avail_buffer, 2);
				data_len_avail = data_len_avail_buffer[0];
				data_len_avail <<= 8;
				data_len_avail |= data_len_avail_buffer[1];

				i2c_read_errs = 0;
				//DEV_DBG(&chip->client->dev, "data avail: %d\n", data_len_avail);

				if (data_len_avail) {
					uint16_t curr = 0;
					uint16_t block_size = 128;
					reg = 0xff;
					for(curr = 0; curr < data_len_avail; curr += block_size) {
						int length_to_read = block_size;
						if (curr + length_to_read > data_len_avail)
							length_to_read = (data_len_avail - curr);
						ret = ublox_sam_m10q_read_buffer(chip->client, reg,  s_ubx_data_rcvd + s_ubx_data_rcvd_length, length_to_read, false);
						DEV_DBG(&chip->client->dev, "Reading off data -> offset %d length: %d - ret: %d\n", curr, length_to_read, ret);
						if (ret >= 0) {
							s_ubx_data_rcvd_length += length_to_read;
							if (false == parse_ubx_msgs(chip)) {
								dev_dbg_data(chip, "failed on parse_ubx_msgs.  data: ", s_ubx_data_rcvd, length_to_read);
								if (s_default_config_status == UBLOX_CONFIG_STATUS_READY)
									s_default_config_status = UBLOX_CONFIG_STATUS_INIT;
							}
							else {
								//sleep_range(10, 100);
								//ublox_sam_m10q_bin_attr_curr_pvt_notify(chip);
							}
						}
						else {
							break;
						}
					}
				}
				if(s_default_config_status < UBLOX_CONFIG_STATUS_READY)
					ublox_sam_m10q_apply_default_config(chip);
				else if (s_default_config_status < UBLOX_CONFIG_STATUS_VERIFIED)
					ublox_sam_m10q_read_config(chip);
				else {
					if ((chip->status & UBLOX_SAM_M10Q_STATUS_BIT_TIME_VALID) == 0 && (chip->status & UBLOX_SAM_M10Q_STATUS_BIT_ASSIST_DATA_APPLIED) == 0)
						ublox_sam_m10q_use_utc_from_system_rtc (chip, true, false);
				}
			}
			else {
				if (i2c_read_errs <= READ_ERR_LIMIT) {
					if (i2c_read_errs == READ_ERR_LIMIT)
						dev_err(&chip->client->dev,
							"%s: Too many i2c errors.  Entering slow poll mode\n", __func__);
					i2c_read_errs++;
				}
			}
		}
		else {
			if (i2c_read_errs <= READ_ERR_LIMIT) {
				i2c_read_errs = READ_ERR_LIMIT;
				if (i2c_read_errs == READ_ERR_LIMIT)
					dev_err(&chip->client->dev,
						"%s: UBlox should be in standby.  Entering slow check mode\n", __func__);
				i2c_read_errs++;
			}

		}
	}

	if (i2c_read_errs > READ_ERR_LIMIT)
		schedule_delayed_work(&chip->work, 10*HZ); else
		schedule_delayed_work(&chip->work, 1*HZ);
}


struct kobject* sciaps_kobj;

//static void ublox_sam_m10q_bin_attr_curr_pvt_notify(struct ublox_sam_m10q_info_t *chip)
//{
//
//	if (chip->gnss_kobj)
//		sysfs_notify(chip->gnss_kobj, NULL, "current");
//}

static ssize_t ublox_sam_m10q_bin_attr_pvt_read(struct ublox_sam_m10q_info_t* chip,
				struct ubx_nav_pvt* pvt,
				char *buf, loff_t off, size_t count)
{
	uint8_t buffer[sizeof (struct ubx_nav_pvt)];
	int i, processed;
	char* p = buf;

	count = min_t(loff_t, count, sizeof (struct ubx_nav_pvt) - off);

	mutex_lock(&chip->lock);
	memcpy(buffer, ((uint8_t*)pvt) + off, count);
	mutex_unlock(&chip->lock);


	//dev_dbg_data(chip, "attr_pvt_read", buffer, count);
	dev_dbg(&chip->client->dev, "%s: reading %d out of %d\n", __func__, (int)count, (int)sizeof (struct ubx_nav_pvt));

	processed = 0;
	for (i = off; count > 0; ++i, ++p, --count) {
        if (__put_user(buffer[i], p)) {
            dev_err(&chip->client->dev, "%s: __put_user FAILED!!!!", __func__);
             return -EFAULT;
        }
        ++processed;
    }

	return processed;
}

static ssize_t ublox_sam_m10q_bin_attr_curr_pvt_read(struct file *filp,
				struct kobject *kobj,
				struct bin_attribute *bin_attr,
				char *buf, loff_t off, size_t count)
{
	struct ublox_sam_m10q_info_t *chip = container_of(bin_attr, struct ublox_sam_m10q_info_t, bin_attr_curr_pvt);
	struct ubx_nav_pvt *pvt = &chip->curr_pvt;

	dev_dbg(&chip->client->dev, "%s: Enter\n", __func__);
	return ublox_sam_m10q_bin_attr_pvt_read(chip, pvt, buf, off, count);
}

static ssize_t ublox_sam_m10q_bin_attr_last_valid_pvt_read(struct file *filp,
				struct kobject *kobj,
				struct bin_attribute *bin_attr,
				char *buf, loff_t off, size_t count)
{
	struct ublox_sam_m10q_info_t *chip = container_of(bin_attr, struct ublox_sam_m10q_info_t, bin_attr_last_valid_pvt);
	struct ubx_nav_pvt *pvt = &chip->last_valid_pvt;

	dev_dbg(&chip->client->dev, "%s: Enter\n", __func__);
	return ublox_sam_m10q_bin_attr_pvt_read(chip, pvt, buf, off, count);
}

static ssize_t ublox_sam_m10q_bin_attr_mga_ano_write(struct file *filp,
				struct kobject *kobj,
				struct bin_attribute *bin_attr,
				char *buf, loff_t off, size_t count)
{
	struct ublox_sam_m10q_info_t *chip = container_of(bin_attr, struct ublox_sam_m10q_info_t, bin_attr_mga_ano);

#define UBX_MGA_ANO_FULL_MSG_LENGTH 84
	uint8_t buffer[UBX_MGA_ANO_FULL_MSG_LENGTH];
	int i, m, processed;

	dev_info(&chip->client->dev, "%s: Enter\n", __func__);

	if (off) {
		dev_info(&chip->client->dev, "%s: Offset must be zero! loff_t = %d, count = %d", __func__, (int)off, (int)count);
		return 0;
	}

	if ((chip->status & UBLOX_SAM_M10Q_STATUS_BIT_TIME_VALID) == 0 && (chip->status & UBLOX_SAM_M10Q_STATUS_BIT_ASSIST_DATA_APPLIED) == 0)
		ublox_sam_m10q_use_utc_from_system_rtc (chip, false, true);

	processed = 0;
	for (m = 0; m + UBX_MGA_ANO_FULL_MSG_LENGTH <= count; m += UBX_MGA_ANO_FULL_MSG_LENGTH) {
		for(i = 0; i < UBX_MGA_ANO_FULL_MSG_LENGTH; i++) {
			if (__get_user(buffer[i], &buf[m+i])) {
				dev_err(&chip->client->dev, "%s: __get_user FAILED!!!!", __func__);
				return -EFAULT;
			}
		}
		// process the message here!
		dev_dbg_data(chip, "UBX-MGA-ANO", buffer, UBX_MGA_ANO_FULL_MSG_LENGTH);
		ublox_sam_m10q_i2c_write_buffer(chip->client, buffer,  UBX_MGA_ANO_FULL_MSG_LENGTH, false);
        processed += UBX_MGA_ANO_FULL_MSG_LENGTH;
    }

	mutex_lock(&chip->lock);
	chip->status |= UBLOX_SAM_M10Q_STATUS_BIT_ASSIST_DATA_APPLIED;
	//memcpy(buffer, ((uint8_t*)pvt) + off, count);
	mutex_unlock(&chip->lock);

	return processed;
}

static ssize_t ublox_sam_m10q_bin_attr_mga_any_write(struct file *filp,
				struct kobject *kobj,
				struct bin_attribute *bin_attr,
				char *buf, loff_t off, size_t count)
{
	struct ublox_sam_m10q_info_t *chip = container_of(bin_attr, struct ublox_sam_m10q_info_t, bin_attr_mga_any);

#define UBX_MGA_ANY_FULL_MSG_LENGTH_MAX 172 //UBX-MGA-DBD, need to ignore UBX-MGA-FLASH-DATA
	uint8_t buffer[UBX_MGA_ANY_FULL_MSG_LENGTH_MAX];
	int i, m, processed;
	uint16_t payload_length;

	dev_info(&chip->client->dev, "%s: Enter\n", __func__);

	if (off) {
		dev_info(&chip->client->dev, "%s: Offset must be zero! loff_t = %d, count = %d", __func__, (int)off, (int)count);
		return 0;
	}

	processed = 0;
	m = 0;
	while ((m + UBX_HEADER_LENGTH + UBX_CHECKSUM_LENGTH) <= count) {
		i = 0;
		for(; i < UBX_HEADER_LENGTH; i++) {
			if (__get_user(buffer[i], &buf[m+i])) {
				dev_err(&chip->client->dev, "%s: __get_user FAILED!!!!", __func__);
				return -EFAULT;
			}
		}
		payload_length = buffer[i-1];
		payload_length <<= 8;
		payload_length |= buffer[i-2];
		if ((m + UBX_HEADER_LENGTH + UBX_CHECKSUM_LENGTH + payload_length) <= count
				&&  (UBX_HEADER_LENGTH + UBX_CHECKSUM_LENGTH + payload_length) <= UBX_MGA_ANY_FULL_MSG_LENGTH_MAX) {
			for(; i < (UBX_HEADER_LENGTH + UBX_CHECKSUM_LENGTH + payload_length); i++) {
				if (__get_user(buffer[i], &buf[m+i])) {
					dev_err(&chip->client->dev, "%s: __get_user FAILED!!!!", __func__);
					return -EFAULT;
				}
			}
		}
		else {
			dev_err(&chip->client->dev, "%s: Invalid UBX-MGA data!!!!", __func__);
			return -EFAULT;
		}

		// process the message here!
		dev_info_data(chip, "UBX-MGA-ANY", buffer, i);
		ublox_sam_m10q_i2c_write_buffer(chip->client, buffer,  i, false);
		m += i;
        processed += i;
    }

	//mutex_lock(&chip->lock);
	//chip->status |= UBLOX_SAM_M10Q_STATUS_BIT_ASSIST_DATA_APPLIED;
	////memcpy(buffer, ((uint8_t*)pvt) + off, count);
	//mutex_unlock(&chip->lock);

	return processed;
}


static ssize_t ublox_sam_m10q_attr_show(struct kobject *kobj, struct attribute *attr, char *buf)
{
	struct sciaps_attribute *sa = container_of(attr, struct sciaps_attribute, attr);
	struct ublox_sam_m10q_info_t *chip = sa->chip;
	int value = -1;
	ssize_t rv;

	if (sa == &chip->attrs[1]) {
		mutex_lock(&chip->lock);
		value = chip->status;
		mutex_unlock(&chip->lock);
		rv = scnprintf(buf, PAGE_SIZE, "%d\n", value);
	}
	else if (sa == &chip->attrs[2]) {
		int year, month, day;
		mutex_lock(&chip->lock);
		year = chip->year;
		month = chip->month;
		day = chip->day;
		mutex_unlock(&chip->lock);
		rv = scnprintf(buf, PAGE_SIZE, "%02d/%02d/%4d\n", month, day, year);
	}
	else if (sa == &chip->attrs[3]) {
		int hour, min, sec;
		mutex_lock(&chip->lock);
		hour = chip->hour;
		min = chip->min;
		sec = chip->sec;
		mutex_unlock(&chip->lock);
		rv = scnprintf(buf, PAGE_SIZE, "%02d:%02d:%02d\n", hour, min, sec);
	}
	else if (sa == &chip->attrs[4]) {
		mutex_lock(&chip->lock);
		value = chip->power_mode;
		mutex_unlock(&chip->lock);
		rv = scnprintf(buf, PAGE_SIZE, "%d\n", value);
	}
	else {
		rv = scnprintf(buf, PAGE_SIZE, "%d\n", value);
	}

   // dev_info(&chip->client->dev, "%s: show called (%s). Value: %d;\n", __func__, sa->attr.name, value);

	return rv;
}

static ssize_t ublox_sam_m10q_attr_store(struct kobject *kobj, struct attribute *attr, const char *buf, size_t len)
{
	struct sciaps_attribute *sa = container_of(attr, struct sciaps_attribute, attr);
	struct ublox_sam_m10q_info_t *chip = sa->chip;

	dev_info(&chip->client->dev, "%s: Enter\n", __func__);

	if (sa == &chip->attrs[0]) {
		uint32_t mask = 0, mode = 0;
		int ret;
		uint8_t msg[16];
		uint16_t data_len;
		uint8_t* data;
		uint8_t  reset_payload[] = { (uint8_t)mask, (uint8_t)(mask>>8), (uint8_t)mode, 0x00 };

		sscanf(buf, "%04x:%02x\n", &mask, &mode);

		dev_info(&chip->client->dev, "%s: UBX-CFG-RST requested ==> navBbrMask: 0x%04x; resetMode: %d;\n", __func__, mask, mode);


		if (mode == 0xFF) {
			int rc;
			dev_info(&chip->client->dev, "%s: Reseting u-blox via RST pin : %d\n", __func__, chip->gpio_rst_pin);

			gpio_set_value(chip->gpio_rst_pin, 0);

			rc = gpio_direction_output(chip->gpio_rst_pin, 0);
			if (rc) {
				dev_err(&chip->client->dev, "%s: gpio %d gpio_direction_output failed with err: %d\n",
						__func__, chip->gpio_rst_pin, rc);
			}

			// Must be at least 1ms
			udelay(1010);

			rc = gpio_direction_input(chip->gpio_rst_pin);
			if (rc) {
				dev_err(&chip->client->dev, "%s: gpio %d gpio_direction_input failed with err: %d\n",
						__func__, chip->gpio_rst_pin, rc);
			}

			gpio_set_value(chip->gpio_rst_pin, 1);


			chip->status = UBLOX_SAM_M10Q_STATUS_RESET;
			s_default_config_status = UBLOX_CONFIG_STATUS_INIT;

			dev_info(&chip->client->dev, "%s: Reseting u-blox via RST pin comleted!", __func__);
		}
		else {
			data_len = sizeof (msg) / sizeof (*msg);
			data = ublox_sam_m10q_make_ubx_msg(UBX_CFG_RST, reset_payload, sizeof (reset_payload) / sizeof (*reset_payload), msg, &data_len);
			if (data) {
				dev_info_data(chip, "UBX-CFG-RST", data, data_len);
				ret = ublox_sam_m10q_i2c_write_buffer(chip->client, data,  data_len, false);
				if (ret < 0) {
					dev_info(&chip->client->dev, "%s: UBX-CFG-RST failed to sent! ret: %d\n", __func__, ret);
				}
				else {
					chip->status = UBLOX_SAM_M10Q_STATUS_RESET;
					s_default_config_status = UBLOX_CONFIG_STATUS_INIT;
					//s_ubx_data_rcvd_length = 0;
					dev_info(&chip->client->dev, "%s: UBX-CFG-RST Sent!!!\n", __func__);
				}
			}
		}

	}
	else if (sa == &chip->attrs[4]) {
		uint32_t wakeup_sources = (0x40 | 0x20); /* 0x80 - spics, 0x40 - extint1, 0x20 - extint0, 0x08 - iuartrx*/
		uint32_t flags = (0x04 | 0x02); /*0x02 - backup; 0x04 - force*/
		uint32_t pm;
		int ret;
		uint8_t msg[24];
		uint16_t data_len;
		uint8_t* data;
		uint8_t  payload[] = { 0x00							// version. must be 0x00
									, 0x00, 0x00, 0x00			// Reserved
									, 0x00, 0x00, 0x00, 0x00	// Duration of the requested task. The maximum
																// supported value is 12 days. Set to 0 to wait for a
																// wakeup signal on a pin
									,(uint8_t)flags, (uint8_t)(flags>>8), (uint8_t)(flags>>16), (uint8_t)(flags>>24)
									,(uint8_t)wakeup_sources, (uint8_t)(wakeup_sources>>8), (uint8_t)(wakeup_sources>>16), (uint8_t)(wakeup_sources>>24)
								};

		sscanf(buf, "%d\n", &pm);

		dev_info(&chip->client->dev, "%s: PM mode change requested ==> current mode: %d; requested mode: %d;\n", __func__, chip->power_mode, pm);

		if (pm == UBX_PM_CONTINUOUS) {
			//exit PM mode if needed
			if (chip->power_mode != pm) {
				// Exit power save mode!!!!
				chip->power_mode = pm;

				dev_info(&chip->client->dev, "%s: Waking up u-blox via EXTINT pin : %d\n", __func__, chip->gpio_extint_pin);

				gpio_set_value(chip->gpio_extint_pin, 0);
				ndelay(1000);
				gpio_set_value(chip->gpio_extint_pin, 1);


			}
		}
		else if (pm == UBX_PM_SW_STANDBY) {

			if (chip->power_mode != pm) {
				chip->power_mode = pm;
				data_len = sizeof (msg) / sizeof (*msg);
				data = ublox_sam_m10q_make_ubx_msg(UBX_RXM_PMREQ, payload, sizeof (payload) / sizeof (*payload), msg, &data_len);
				if (data) {
					dev_info_data(chip, "UBX-RXM-PMREQ", data, data_len);
					ret = ublox_sam_m10q_i2c_write_buffer(chip->client, data,  data_len, false);
					if (ret < 0) {
						dev_info(&chip->client->dev, "%s: UBX-RXM-PMREQ failed to sent! ret: %d\n", __func__, ret);
					}
					else {
						//chip->status = 0;
						//s_default_config_status = UBLOX_CONFIG_STATUS_INIT;
						//s_ubx_data_rcvd_length = 0;
						dev_info(&chip->client->dev, "%s: UBX-RXM-PMREQ Sent!!!\n", __func__);
					}
				}
			}
		}
	}
/*
    struct sciaps_attr *sa = container_of(attr, struct sciaps_attr, attr);
	int value = -1;

    sscanf(buf, "%d", &value);

    pr_info("%s: store called (%s). Value: %d/%d (old/new)\n", __func__, sa->attr.name, sa->value, value);

	if (sa == &trigger_debounce_time_ms) {
		trigger_debounce_time_ms.value = value;
		sciaps_trigger_debounce_time_ms = trigger_debounce_time_ms.value;

	}
*/
    return 8;
}

static struct sysfs_ops ublox_sam_m10q_gnss_sysfs_ops = {
    .show = ublox_sam_m10q_attr_show,
    .store = ublox_sam_m10q_attr_store,
};

static struct kobj_type ublox_sam_m10q_gnss_kobj_type = {
    .sysfs_ops = &ublox_sam_m10q_gnss_sysfs_ops,
    //.default_attrs = sciaps_attrs,
};

#define MAX_SCIAPS_ATTRIBUTES 10
static struct attribute* sciaps_attrs[MAX_SCIAPS_ATTRIBUTES];

static int ublox_sam_m10q_setup_sysfs(struct i2c_client *client)
{
	int ret = -1;
	struct ublox_sam_m10q_info_t *chip = i2c_get_clientdata(client);

    dev_info(&client->dev, "%s: init\n", __func__);

	chip->curr_pvt_sysfs_ready = 0;
	chip->last_valid_pvt_sysfs_ready = 0;

	sciaps_kobj = kobject_create_and_add("sciaps", kernel_kobj);
	if (sciaps_kobj) {

		//chip->gnss_kobj = kobject_create_and_add("gnss", sciaps_kobj);
		chip->gnss_kobj = kzalloc(sizeof(*chip->gnss_kobj), GFP_KERNEL);

		if (chip->gnss_kobj) {
			int pos = 0;
			ublox_sam_m10q_gnss_kobj_type.default_attrs = sciaps_attrs;
			chip->attrs[pos].attr.name = "reset";
			chip->attrs[pos].attr.mode = S_IWUGO;
			chip->attrs[pos].chip = chip;
			sciaps_attrs[pos] = &chip->attrs[pos].attr;
			pos++;

			chip->attrs[pos].attr.name = "status";
			chip->attrs[pos].attr.mode = S_IRUGO;
			chip->attrs[pos].chip = chip;
			sciaps_attrs[pos] = &chip->attrs[pos].attr;
			pos++;

			chip->attrs[pos].attr.name = "date";
			chip->attrs[pos].attr.mode = S_IRUGO;
			chip->attrs[pos].chip = chip;
			sciaps_attrs[pos] = &chip->attrs[pos].attr;
			pos++;

			chip->attrs[pos].attr.name = "time";
			chip->attrs[pos].attr.mode = S_IRUGO;
			chip->attrs[pos].chip = chip;
			sciaps_attrs[pos] = &chip->attrs[pos].attr;
			pos++;

			chip->attrs[pos].attr.name = "power-mode";
			chip->attrs[pos].attr.mode = S_IRUGO|S_IWUGO;
			chip->attrs[pos].chip = chip;
			sciaps_attrs[pos] = &chip->attrs[pos].attr;
			pos++;


			sciaps_attrs[pos] = NULL;

			kobject_init(chip->gnss_kobj, &ublox_sam_m10q_gnss_kobj_type);
			if (kobject_add(chip->gnss_kobj, sciaps_kobj, "%s", "gnss")) {
				dev_info(&client->dev,
								"%s: kobject_add for gnss failed", __func__);
				kobject_put(chip->gnss_kobj);
				kfree(chip->gnss_kobj);
				chip->gnss_kobj = NULL;
			}
		}
		else {
			dev_info(&client->dev,
								"%s: kzalloc for chip->gnss_kobj failed!", __func__);
		}


		if (chip->gnss_kobj) {
			chip->bin_attr_curr_pvt.attr.name = "current-pvt";
			chip->bin_attr_curr_pvt.attr.mode = S_IRUGO;
			chip->bin_attr_curr_pvt.size  = sizeof(struct ubx_nav_pvt);
			chip->bin_attr_curr_pvt.read = ublox_sam_m10q_bin_attr_curr_pvt_read;
			ret = sysfs_create_bin_file(chip->gnss_kobj, &chip->bin_attr_curr_pvt);

			if (ret) {
				dev_info(&client->dev,
								"failed to create chip->bin_attr_curr_pvt");
			}
			else {
				chip->curr_pvt_sysfs_ready = 1;
			}
			chip->bin_attr_last_valid_pvt.attr.name = "last-valid-pvt";
			chip->bin_attr_last_valid_pvt.attr.mode = S_IRUGO;
			chip->bin_attr_last_valid_pvt.size  = sizeof(struct ubx_nav_pvt);
			chip->bin_attr_last_valid_pvt.read = ublox_sam_m10q_bin_attr_last_valid_pvt_read;
			ret = sysfs_create_bin_file(chip->gnss_kobj, &chip->bin_attr_last_valid_pvt);

			if (ret) {
				dev_info(&client->dev,
								"failed to create chip->bin_attr_last_valid_pvt");
			}
			else {
				chip->last_valid_pvt_sysfs_ready = 1;
			}
			chip->bin_attr_mga_ano.attr.name = "mga-ano";
			chip->bin_attr_mga_ano.attr.mode = S_IWUGO;
			chip->bin_attr_mga_ano.size  = 0;
			chip->bin_attr_mga_ano.write = ublox_sam_m10q_bin_attr_mga_ano_write;
			ret = sysfs_create_bin_file(chip->gnss_kobj, &chip->bin_attr_mga_ano);

			if (ret) {
				dev_info(&client->dev,
								"failed to create chip->bin_attr_mga_ano");
			}
			else {
				chip->mga_ano_sysfs_ready = 1;
			}
			chip->bin_attr_mga_any.attr.name = "mga-any";
			chip->bin_attr_mga_any.attr.mode = S_IWUGO;
			chip->bin_attr_mga_any.size  = 0;
			chip->bin_attr_mga_any.write = ublox_sam_m10q_bin_attr_mga_any_write;
			ret = sysfs_create_bin_file(chip->gnss_kobj, &chip->bin_attr_mga_any);

			if (ret) {
				dev_info(&client->dev,
								"failed to create chip->bin_attr_mga_any");
			}
			else {
				chip->mga_any_sysfs_ready = 1;
			}
		}
		else {
			pr_err("%s: kobject_create_and_add() for 'gnss' failed\n", __func__);
			kobject_put(sciaps_kobj);
			kfree(sciaps_kobj);
			sciaps_kobj = NULL;
			chip->gnss_kobj = NULL;
		}

	}
	else {
		pr_err("%s: kobject_create_and_add() for 'sciaps' failed\n", __func__);
		sciaps_kobj = NULL;
		chip->gnss_kobj = NULL;
	}


	return ret;
}

static int ublox_sam_m10q_delete_sysfs(struct i2c_client *client)
{
	struct ublox_sam_m10q_info_t *chip = i2c_get_clientdata(client);

	if (chip->gnss_kobj) {
		if (chip->curr_pvt_sysfs_ready) {
			sysfs_remove_bin_file(chip->gnss_kobj, &chip->bin_attr_curr_pvt);
			chip->curr_pvt_sysfs_ready = 0;
		}
		if (chip->last_valid_pvt_sysfs_ready) {
			sysfs_remove_bin_file(chip->gnss_kobj, &chip->bin_attr_last_valid_pvt);
			chip->last_valid_pvt_sysfs_ready = 0;
		}
		if (chip->mga_ano_sysfs_ready) {
			sysfs_remove_bin_file(chip->gnss_kobj, &chip->bin_attr_mga_ano);
			chip->mga_ano_sysfs_ready = 0;
		}
		if (chip->mga_any_sysfs_ready) {
			sysfs_remove_bin_file(chip->gnss_kobj, &chip->bin_attr_mga_any);
			chip->mga_any_sysfs_ready = 0;
		}
		kobject_put(chip->gnss_kobj);
		kfree(chip->gnss_kobj);
		chip->gnss_kobj = NULL;
	}
	if (sciaps_kobj) {
		kobject_put(sciaps_kobj);
		kfree(sciaps_kobj);
		sciaps_kobj = NULL;
	}

	return 0;
}


#define UBLOX_RST_PIN_GPIO			(120 + 902)
#define UBLOX_EXTINT_PIN_GPIO		(21 + 902)

static int __devinit ublox_sam_m10q_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	struct ublox_sam_m10q_info_t *chip;
	int rc = 0;
	struct device_node *np;
	uint32_t dt_value_u32;


	dev_info(&client->dev,
			"%s : ---> s_cmdline_param_enable: %d\n", __func__, s_cmdline_param_enable);

	if (!s_cmdline_param_enable) {
		dev_info(&client->dev,"%s : Must be enabled using 'ulox-sam-m10q-i2c.enable=1'. Goodbye!!!\n", __func__);
		return -ENODEV;
	}

	s_default_config_status = UBLOX_CONFIG_STATUS_INIT;
	s_ubx_data_rcvd_length = 0;

	chip = kzalloc(sizeof(struct ublox_sam_m10q_info_t), GFP_KERNEL);
	if (!chip) {
		rc = -ENOMEM;
		goto exit_free_name;
	}

	chip->client = client;
	np = of_node_get(client->dev.of_node);


	chip->status					= UBLOX_SAM_M10Q_STATUS_RESET;
	chip->power_mode				= UBX_PM_CONTINUOUS;
	//chip->numSVAcceptHardThreshold	= UBLOX_SAM_M10Q_numSVAcceptHardThreshold_Default;
	chip->hAccRejectHardThreshold		= UBLOX_SAM_M10Q_hAccRejectHardThreshold_Default;
	chip->pDOPRejectHardThreshold		= UBLOX_SAM_M10Q_pDOPRejectHardThreshold_Default;
	chip->pDOPRejectThreshold			= UBLOX_SAM_M10Q_pDOPRejectThreshold_Default;
	chip->hAccAcceptThreshold			= UBLOX_SAM_M10Q_hAccAcceptThreshold_Default;
	chip->nSatsAcceptAlwaysThreshold	= UBLOX_SAM_M10Q_nSatsAcceptAlwaysThreshold_Default;
	chip->gpio_rst_pin				= UBLOX_RST_PIN_GPIO;
	chip->gpio_extint_pin			= UBLOX_EXTINT_PIN_GPIO;
	chip->last_time_assistance		= 0;
	chip->year	= 0;
	chip->month = 0;
	chip->day	= 0;
	chip->hour	= 0;
	chip->min	= 0;
	chip->sec	= 0;

#if 0
	rc = gpio_request(UBLOX_RST_PIN_GPIO, "ublox-rst-pin");
	if (rc) {
		dev_err(&client->dev, "%s: gpio %d request failed with err: %d\n",
					__func__, UBLOX_RST_PIN_GPIO, rc);
	}
	else {
		gpio_set_value(UBLOX_RST_PIN_GPIO, 1);
		rc = gpio_direction_output(UBLOX_RST_PIN_GPIO, 1);
		if (rc) {
			dev_err(&client->dev, "%s: gpio %d gpio_direction_output failed with err: %d\n",
					__func__, UBLOX_RST_PIN_GPIO, rc);
		}
		rc = gpio_export(UBLOX_RST_PIN_GPIO, false);
		if (rc) {
			dev_err(&client->dev, "%s: gpio %d gpio_export failed with err: %d\n",
					__func__, UBLOX_RST_PIN_GPIO, rc);
		}
	}
	rc = gpio_request(UBLOX_EXTINT_PIN_GPIO, "ublox-extint-pin");
	if (rc) {
		dev_err(&client->dev, "%s: gpio %d request failed with err: %d\n",
					__func__, UBLOX_EXTINT_PIN_GPIO, rc);
	}
	else {
		gpio_set_value(UBLOX_EXTINT_PIN_GPIO, 1);
		rc = gpio_direction_output(UBLOX_EXTINT_PIN_GPIO, 1);
		if (rc) {
			dev_err(&client->dev, "%s: gpio %d gpio_direction_output failed with err: %d\n",
					__func__, UBLOX_EXTINT_PIN_GPIO, rc);
		}
		rc = gpio_export(UBLOX_EXTINT_PIN_GPIO, false);
		if (rc) {
			dev_err(&client->dev, "%s: gpio %d gpio_export failed with err: %d\n",
					__func__, UBLOX_EXTINT_PIN_GPIO, rc);
		}
	}

#endif

	/*
	 * Returns 0 on success, -EINVAL if the property does not exist,
	 * -ENODATA if property does not have a value, and -EOVERFLOW if the
	 * property data isn't large enough.
	 *
	 * The out_value is modified only if a valid u32 value can be decoded.
	 */
	if (1 == of_gpio_named_count(np, "ublox,rst-pin-gpio")) {
		chip->gpio_rst_pin  = of_get_named_gpio(np, "ublox,rst-pin-gpio", 0);

		dev_info(&client->dev,
			"%s : ublox,rst-pin-gpio is %d\n", __func__, chip->gpio_rst_pin);
		if (gpio_is_valid(chip->gpio_rst_pin)) {
			dev_info(&client->dev,
				"%s : ublox,rst-pin-gpio gpio is valid\n", __func__);
		}
		else {
			chip->gpio_rst_pin = UBLOX_RST_PIN_GPIO;
			dev_warn(&client->dev,
				"%s : ublox,rst-pin-gpio gpio is NOT valid. Using default: %d\n", __func__, chip->gpio_rst_pin);
		}
	}
	else {
		chip->gpio_rst_pin = UBLOX_RST_PIN_GPIO;
		dev_warn(&client->dev,
			"%s : Could not find ublox,rst-pin-gpio in the devicetree. Using default: %d\n", __func__, chip->gpio_rst_pin);
	}

	rc = gpio_request(chip->gpio_rst_pin, "ublox-rst-pin");
	if (rc) {
		dev_err(&client->dev, "%s: gpio %d request failed with err: %d\n",
					__func__, chip->gpio_rst_pin, rc);
	}
	else {
		gpio_set_value(chip->gpio_rst_pin, 1);
		rc = gpio_direction_input(chip->gpio_rst_pin);
		if (rc) {
			dev_err(&client->dev, "%s: gpio %d gpio_direction_input failed with err: %d\n",
					__func__, chip->gpio_rst_pin, rc);
		}
	}

	if (1 == of_gpio_named_count(np, "ublox,extint-pin-gpio")) {
		chip->gpio_extint_pin  = of_get_named_gpio(np, "ublox,extint-pin-gpio", 0);

		dev_info(&client->dev,
			"%s : ublox,extint-pin-gpio is %d\n", __func__, chip->gpio_extint_pin);
		if (gpio_is_valid(chip->gpio_extint_pin)) {
			dev_info(&client->dev,
				"%s : ublox,extint-pin-gpio gpio is valid\n", __func__);
		}
		else {
			chip->gpio_extint_pin = UBLOX_EXTINT_PIN_GPIO;
			dev_warn(&client->dev,
				"%s : ublox,extint-pin-gpio gpio is NOT valid. Using default: %d\n", __func__, chip->gpio_extint_pin);
		}
	}
	else {
		chip->gpio_extint_pin = UBLOX_EXTINT_PIN_GPIO;
		dev_warn(&client->dev,
			"%s : Could not find ublox,rst-pin-gpio in the devicetree. Using default: %d\n", __func__, chip->gpio_extint_pin);
	}

	rc = gpio_request(chip->gpio_extint_pin, "ublox-extint-pin");
	if (rc) {
		dev_err(&client->dev, "%s: gpio %d request failed with err: %d\n",
					__func__, chip->gpio_extint_pin, rc);
	}
	else {
		gpio_set_value(chip->gpio_extint_pin, 1);
		rc = gpio_direction_output(chip->gpio_extint_pin, 1);
		if (rc) {
			dev_err(&client->dev, "%s: gpio %d gpio_direction_output failed with err: %d\n",
					__func__, chip->gpio_extint_pin, rc);
		}
	}


	rc = of_property_read_u32(np, "ubx,hAccRejectHardThreshold",
					&dt_value_u32);
	if (rc < 0) {
		chip->hAccRejectHardThreshold = UBLOX_SAM_M10Q_hAccRejectHardThreshold_Default;
		dev_info(&client->dev,
			"Unable to read 'hAccRejectHardThreshold'. Use default: %d mm\n", chip->hAccRejectHardThreshold);
	}
	else {
		chip->hAccRejectHardThreshold = dt_value_u32;
		dev_info(&client->dev,
			"'hAccRejectHardThreshold' == %d mm\n", chip->hAccRejectHardThreshold);
	}

	rc = of_property_read_u32(np, "ubx,pDOPRejectHardThreshold",
					&dt_value_u32);
	if (rc < 0) {
		chip->pDOPRejectHardThreshold = UBLOX_SAM_M10Q_pDOPRejectHardThreshold_Default;
		dev_info(&client->dev,
			"Unable to read 'pDOPRejectHardThreshold'. Use default: %d (0.01)\n", chip->pDOPRejectHardThreshold);
	}
	else {
		chip->pDOPRejectHardThreshold = (uint16_t)dt_value_u32;
		dev_info(&client->dev,
			"'pDOPRejectHardThreshold' == %d (0.01)\n", chip->pDOPRejectHardThreshold);
	}

	rc = of_property_read_u32(np, "ubx,pDOPRejectThreshold",
					&dt_value_u32);
	if (rc < 0) {
		chip->pDOPRejectThreshold = UBLOX_SAM_M10Q_pDOPRejectThreshold_Default;
		dev_info(&client->dev,
			"Unable to read 'pDOPRejectThreshold'. Use default: %d (0.01)\n", chip->pDOPRejectThreshold);
	}
	else {
		chip->pDOPRejectThreshold = (uint16_t)dt_value_u32;
		dev_info(&client->dev,
			"'pDOPRejectThreshold' == %d (0.01)\n", chip->pDOPRejectThreshold);
	}

	rc = of_property_read_u32(np, "ubx,hAccAcceptThreshold",
					&dt_value_u32);
	if (rc < 0) {
		chip->hAccAcceptThreshold = UBLOX_SAM_M10Q_hAccAcceptThreshold_Default;
		dev_info(&client->dev,
			"Unable to read 'hAccAcceptThreshold'. Use default: %d mm\n", chip->hAccAcceptThreshold);
	}
	else {
		chip->hAccAcceptThreshold = dt_value_u32;
		dev_info(&client->dev,
			"'hAccAcceptThreshold' == %d mm\n", chip->hAccAcceptThreshold);
	}


	rc = of_property_read_u32(np, "ubx,nSatsAcceptAlwaysThreshold",
					&dt_value_u32);
	if (rc < 0) {
		chip->nSatsAcceptAlwaysThreshold = UBLOX_SAM_M10Q_nSatsAcceptAlwaysThreshold_Default;
		dev_info(&client->dev,
			"Unable to read 'nSatsAcceptAlwaysThreshold'. Use default: %d # of sats\n", chip->nSatsAcceptAlwaysThreshold);
	}
	else {
		chip->nSatsAcceptAlwaysThreshold = (uint16_t)dt_value_u32;
		dev_info(&client->dev,
			"'nSatsAcceptAlwaysThreshold' == %d # of sats\n", chip->nSatsAcceptAlwaysThreshold);
	}
#if 0

	if (chip->sciaps_support_shutdown) {
		rc = of_property_read_u32(np, "sciaps,shutdown-timer",
						&dt_value_u32);

		if (rc && rc != -EINVAL) {
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
#endif

	mutex_init(&chip->lock);

	i2c_set_clientdata(client, chip);

	ublox_sam_m10q_apply_default_config(chip);

	ublox_sam_m10q_setup_sysfs(client);

	INIT_DELAYED_WORK(&chip->work, ublox_sam_m10q_delayed_work);
	chip->shutdown_work_running = 0;
	INIT_DELAYED_WORK(&chip->shutdown_work, shutdown_work_func);

	//rc = power_supply_register(&client->dev, &chip->power_supply);
	//if (rc) {
	//	dev_err(&client->dev,
	//		"%s: Failed to register power supply\n", __func__);
	//	goto exit_psupply;
	//}

	{
		struct timeval	time;
		//unsigned long	gps_time;

		do_gettimeofday(&time);

		dev_info(&client->dev,
				"%s: ublox SAm-M10Q i2c device registered. time.tv_sec: %ld; time.tv_usec: %ld;\n", client->name, time.tv_sec, time.tv_usec);
	}
	schedule_delayed_work(&chip->work, HZ);


	return 0;

//exit_psupply:
//	kfree(chip);
exit_free_name:
	// no names to free

	return rc;
}

static int __devexit ublox_sam_m10q_remove(struct i2c_client *client)
{
	struct ublox_sam_m10q_info_t *chip = i2c_get_clientdata(client);

	cancel_delayed_work_sync(&chip->work);
	cancel_delayed_work(&chip->shutdown_work);
	ublox_sam_m10q_delete_sysfs(client);
	mutex_destroy(&chip->lock);
	kfree(chip);
	chip = NULL;

	return 0;
}

static const struct i2c_device_id ublox_sam_m10q_id[] = {
	{ "ublox-sam-m10q-i2c", 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, ublox_sam_m10q_id);

static struct i2c_driver ublox_sam_m10q_driver = {
	.probe		= ublox_sam_m10q_probe,
	.remove		= __devexit_p(ublox_sam_m10q_remove),
	.id_table	= ublox_sam_m10q_id,
	.driver = {
		.name	= "ublox-sam-m10q-i2c",
	},
};
module_i2c_driver(ublox_sam_m10q_driver);

MODULE_AUTHOR("Andre Doudkin <adoudkin@sciaps.com>");
MODULE_DESCRIPTION("U-Blox SAM-M10Q driver");
MODULE_LICENSE("GPL");
