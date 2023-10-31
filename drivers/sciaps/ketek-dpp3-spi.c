/*
 * Copyright 2019 SciAps
 *
 *
 *
 *
 *
 */

#include <linux/slab.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/interrupt.h>
#include <linux/mfd/core.h>
#include <linux/mfd/mc13xxx.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/err.h>
#include <linux/spi/spi.h>
#include <linux/delay.h>

#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>

#include "sciaps-driver.h"

#define SCIAPS_DEBUG	0
#define DEV_DBG			dev_dbg

#define SCIAPS_KETEK_DPP3_STATUS_PENDING			0x00
#define SCIAPS_KETEK_DPP3_STATUS_READY				0x01
#define SCIAPS_KETEK_DPP3_STATUS_CHECK_RETRIES		100
#define SCIAPS_KETEK_DPP3_STATUS_CHECK_TIMEOUT_US	100


#define SCIAPS_KETEK_DPP3_CMD_STATUS				0x01
#define SCIAPS_KETEK_DPP3_CMD_TRANSFER				0x02
#define SCIAPS_KETEK_DPP3_CMD_READ					0x03


#define SCIAPS_KETEK_DPP3_PID_RuntimeStatisticsRead	18
#define SCIAPS_KETEK_DPP3_PID_RuntimeStatisticsRead_RespLength	(13*4)
#define SCIAPS_KETEK_DPP3_PID_MCARead				19
#define SCIAPS_KETEK_DPP3_PID_MCARead_Extra_RespLength		(512)	// Sciaps Dpp: To allow reading registers with MCAData
#define SCIAPS_KETEK_DPP3_PID_MCARead_Max_RespLength		(32768 + SCIAPS_KETEK_DPP3_PID_MCARead_Extra_RespLength)	// Sciaps Dpp: (32768) --> 2^13 * 4 ; Ketek Dpp3: (24576) --> 2^13 * 3
#define SCIAPS_KETEK_DPP3_PID_MCARead_Default_RespLength	(12288) // 2^12 * 3
#define SCIAPS_KETEK_DPP3_PID_MCARead_Additional_Alloc_Buffer_Size	(16)
#define SCIAPS_KETEK_DPP3_PID_MCANumberOfBins		20
#define SCIAPS_KETEK_DPP3_PID_MCANumberOfBins_Min	9
#define SCIAPS_KETEK_DPP3_PID_MCANumberOfBins_Max	13
#define SCIAPS_KETEK_DPP3_PID_MCABytesPerBin		21
#define SCIAPS_KETEK_DPP3_PID_MCABytesPerBin_Min	1
#define SCIAPS_KETEK_DPP3_PID_MCABytesPerBin_Max	4 //3

#define SCIAPS_KETEK_DPP3_PID_MCUPassthrough		71

#define SCIAPS_KETEK_DPP3_PID_ReadAllParameters		79
#define SCIAPS_KETEK_DPP3_PID_ReadAllParameters_RespLength		(256*4)


#define SCIAPS_KETEK_DPP3_PID_Standard_RespLength				(4)


#define SCIAPS_KETEK_DPP3_PID_SciapsPassthrough				173
#define SCIAPS_KETEK_DPP3_PID_SciapsPassthrough_Base_RespLength			(3)
#define SCIAPS_KETEK_DPP3_PID_SciapsPassthrough_Data_Default_RespLength	(4)

#define SCIAPS_KETEK_DPP3_MCANumberOfBins_Default	12
#define SCIAPS_KETEK_DPP3_MCABytesPerBin_Default	3
#define SCIAPS_KETEK_DPP3_MCADataLength(bins,bytes)			((((uint16_t)0x0001) << bins) * bytes)
#define SCIAPS_KETEK_DPP3_MCADataLength_Default		(SCIAPS_KETEK_DPP3_MCADataLength(SCIAPS_KETEK_DPP3_MCANumberOfBins_Default, SCIAPS_KETEK_DPP3_MCABytesPerBin_Default))

// MCU Passtrhough Datagrams
#define MCUPassthrough_CL_Response_Base						6
#define MCUPassthrough_CIX_ReqIndex							1

// Update
#define MCUPassthrough_CIX_SW_PKG_GET_ACTIVE 				0   // 6 + 1
#define MCUPassthrough_CIX_SW_PKG_GET_ACTIVE_CL_RespEx		1
#define MCUPassthrough_CIX_FLASH_READ 						1	// 6 + req[9]
#define MCUPassthrough_CIX_FLASH_READ_LEN_ReqIndex			9
#define MCUPassthrough_CIX_FLASH_READ_LEN_Max				249

#define MCUPassthrough_CIX_FLASH_WRITE_SESSION_START 		2	// 6
#define MCUPassthrough_CIX_FLASH_WRITE_SESSION_EXIT 		3	// 6
#define MCUPassthrough_CIX_FLASH_WRITE_SESSION_RESET 		4	// 6
#define MCUPassthrough_CIX_FLASH_WRITE_SESSION_DATA 		5	// 6
#define MCUPassthrough_CIX_BL_GET_SESSION 					10	// 6 + 1
#define MCUPassthrough_CIX_BL_GET_SESSION_CL_RespEx			1
#define MCUPassthrough_CIX_BL_GET_REASON					11	// 6 + 1
#define MCUPassthrough_CIX_BL_GET_REASON_CL_RespEx			1
#define MCUPassthrough_CIX_SW_PKG_START_APPLICATION 		14	// 6
#define MCUPassthrough_CIX_SW_PKG_START_BOOTLOADER 			15	// 6
// Info
#define MCUPassthrough_CIX_LIVEINFO1_VICO 					16	// 6 + 15
#define MCUPassthrough_CIX_LIVEINFO1_VICO_CL_RespEx			15
#define MCUPassthrough_CIX_LIVEINFO1_VIAMP					21	// 6 + 19
#define MCUPassthrough_CIX_LIVEINFO1_VIAMP_CL_RespEx		19
#define MCUPassthrough_CIX_DEVINFO1_BOOTLOADER				25	// 6 + 5
#define MCUPassthrough_CIX_DEVINFO1_BOOTLOADER_CL_RespEx	5
#define MCUPassthrough_CIX_DEVINFO1_VICO					26	// 6 + 5
#define MCUPassthrough_CIX_DEVINFO1_VICO_CL_RespEx			5
#define MCUPassthrough_CIX_DEVINFO2_VICO					27	// 6 + 84
#define MCUPassthrough_CIX_DEVINFO2_VICO_CL_RespEx			84
#define MCUPassthrough_CIX_DEVINFO1_VIAMP					29	// 6 + 88
#define MCUPassthrough_CIX_DEVINFO1_VIAMP_CL_RespEx			88
// Operating Mode
#define MCUPassthrough_CIX_SETMODE							32	// 6
// Temperature
#define MCUPassthrough_CIX_GETTEMP							48	// 6 + 10
#define MCUPassthrough_CIX_GETTEMP_CL_RespEx				10
#define MCUPassthrough_CIX_SETTEMP							49	// 6
#define MCUPassthrough_CIX_SETRDY							54	// 6
#define MCUPassthrough_CIX_SETARDY							55	// 6
#define MCUPassthrough_CIX_GETRDY							56	// 6 + 8
#define MCUPassthrough_CIX_GETRDY_CL_RespEx					8
#define MCUPassthrough_CIX_GETARDY							57	// 6 + 8
#define MCUPassthrough_CIX_GETARDY_CL_RespEx				8


struct sciaps_data_t {
	uint8_t				_device_id;
	struct mutex		_lock;
	struct spi_device*	_spi;
	union device_data_t {
		struct ketek_dpp3_t {
			uint16_t			_numberOfBins;
			uint16_t			_bytesPerBin;
			uint16_t			_mcaDataLength;
			uint16_t			_sciapsExpectedRespDataLength;
		} _ketek_dpp3;
	} _data;
};

static int ketek_dpp3_spi_transfer(struct spi_device *spi, uint8_t *data, uint16_t length);
static int ketek_dpp3_check_status(struct spi_device *spi, uint16_t  attempts, uint32_t delay_usec);
static int ketek_dpp3_spi_read(struct spi_device *spi, uint8_t *data, uint16_t length, uint16_t *length_read);

struct sciaps_data_t* ketek_dpp3_spi_data;

#define KETEK_DPP3_BUFFER_SIZE(length)	((((uint16_t)length > SCIAPS_KETEK_DPP3_PID_MCARead_Max_RespLength) ? (SCIAPS_KETEK_DPP3_PID_MCARead_Max_RespLength + SCIAPS_KETEK_DPP3_PID_MCARead_Additional_Alloc_Buffer_Size) : ((uint16_t)length) + SCIAPS_KETEK_DPP3_PID_MCARead_Additional_Alloc_Buffer_Size))

#ifdef USE_DYNAMIC
static uint8_t* ketek_dpp3_command_buffer;
#else
static uint8_t ketek_dpp3_command_buffer[SCIAPS_KETEK_DPP3_PID_MCARead_Max_RespLength + SCIAPS_KETEK_DPP3_PID_MCARead_Additional_Alloc_Buffer_Size ];
#endif


static uint8_t GetMCUPassthroughRespCL(uint8_t* data) {
	uint8_t cl = MCUPassthrough_CL_Response_Base;
	uint8_t flash_read_length;

	printk(KERN_INFO"%s: GetMCUPassthroughRespCL data ==> %.2x:%.2x:%.2x:%.2x:%.2x\n", __func__
					, data[0], data[1], data[2], data[3], data[4]
					);

	if (data) {
		switch (data[MCUPassthrough_CIX_ReqIndex]) {
			case MCUPassthrough_CIX_SW_PKG_GET_ACTIVE :
				cl += MCUPassthrough_CIX_SW_PKG_GET_ACTIVE_CL_RespEx;
				break;
			case MCUPassthrough_CIX_FLASH_READ :
				flash_read_length = data[MCUPassthrough_CIX_FLASH_READ_LEN_ReqIndex];
				if (flash_read_length > MCUPassthrough_CIX_FLASH_READ_LEN_Max)
					flash_read_length = MCUPassthrough_CIX_FLASH_READ_LEN_Max;
				cl += flash_read_length;
				break;
			case MCUPassthrough_CIX_FLASH_WRITE_SESSION_START :
			case MCUPassthrough_CIX_FLASH_WRITE_SESSION_EXIT :
			case MCUPassthrough_CIX_FLASH_WRITE_SESSION_RESET :
			case MCUPassthrough_CIX_FLASH_WRITE_SESSION_DATA :
				break;
			case MCUPassthrough_CIX_BL_GET_SESSION :
				cl += MCUPassthrough_CIX_BL_GET_SESSION_CL_RespEx;
				break;
			case MCUPassthrough_CIX_BL_GET_REASON :
				cl+= MCUPassthrough_CIX_BL_GET_REASON_CL_RespEx;
				break;
			case MCUPassthrough_CIX_SW_PKG_START_APPLICATION :
			case MCUPassthrough_CIX_SW_PKG_START_BOOTLOADER :
				break;
			case MCUPassthrough_CIX_LIVEINFO1_VICO :
				cl += MCUPassthrough_CIX_LIVEINFO1_VICO_CL_RespEx;
				break;
			case MCUPassthrough_CIX_LIVEINFO1_VIAMP :
				cl += MCUPassthrough_CIX_LIVEINFO1_VIAMP_CL_RespEx;
				break;
			case MCUPassthrough_CIX_DEVINFO1_BOOTLOADER :
				cl += MCUPassthrough_CIX_DEVINFO1_BOOTLOADER_CL_RespEx;
				break;
			case MCUPassthrough_CIX_DEVINFO1_VICO :
				cl += MCUPassthrough_CIX_DEVINFO1_VICO_CL_RespEx;
				break;
			case MCUPassthrough_CIX_DEVINFO2_VICO :
				cl += MCUPassthrough_CIX_DEVINFO2_VICO_CL_RespEx;
				break;
			case MCUPassthrough_CIX_DEVINFO1_VIAMP :
				cl += MCUPassthrough_CIX_DEVINFO1_VIAMP_CL_RespEx;
				break;
			case MCUPassthrough_CIX_SETMODE :
				break;
			case MCUPassthrough_CIX_GETTEMP :
				cl += MCUPassthrough_CIX_GETTEMP_CL_RespEx;
				break;
			case MCUPassthrough_CIX_SETTEMP :
			case MCUPassthrough_CIX_SETRDY :
			case MCUPassthrough_CIX_SETARDY :
				break;
			case MCUPassthrough_CIX_GETRDY :
				cl += MCUPassthrough_CIX_GETRDY_CL_RespEx;
				break;
			case MCUPassthrough_CIX_GETARDY	:
				cl += MCUPassthrough_CIX_GETARDY_CL_RespEx;
				break;
			default :
				break;
		}


	}

	return cl;
}

static ssize_t ketek_dpp3_char_dev_read(struct file *file, char __user *buf,
			  size_t count, loff_t *ppos)
{
	unsigned int i;
	char __user *p = buf;
	ssize_t ret;
	struct spi_device *spi;
	size_t size, processed;

	if (!access_ok(VERIFY_WRITE, buf, count))
		return -EFAULT;

#ifdef USE_DYNAMIC
	if (!ketek_dpp3_command_buffer)
		return -ENOMEM;
#endif
			//---->   || count > SCIAPS_KETEK_DPP3_PID_MCARead_Max_RespLength)

	if (ketek_dpp3_spi_data == 0)
		return -ENODEV;

	if (count == 0 || ketek_dpp3_command_buffer[0] == 0x00) {
		*ppos = 0;
		return 0;
	}

	spi = ketek_dpp3_spi_data->_spi;

	switch (ketek_dpp3_command_buffer[1]) {
		case SCIAPS_KETEK_DPP3_PID_ReadAllParameters :
			size = SCIAPS_KETEK_DPP3_PID_ReadAllParameters_RespLength;
			break;
		case SCIAPS_KETEK_DPP3_PID_RuntimeStatisticsRead :
			size = SCIAPS_KETEK_DPP3_PID_RuntimeStatisticsRead_RespLength;
			break;
		case SCIAPS_KETEK_DPP3_PID_MCARead :
			size = ketek_dpp3_spi_data->_data._ketek_dpp3._mcaDataLength;
			if (size > SCIAPS_KETEK_DPP3_PID_MCARead_Max_RespLength)
				size = SCIAPS_KETEK_DPP3_PID_MCARead_Max_RespLength;
			break;
		case SCIAPS_KETEK_DPP3_PID_MCUPassthrough :
			size = GetMCUPassthroughRespCL(ketek_dpp3_command_buffer + 3) + 2;
			break;
		case SCIAPS_KETEK_DPP3_PID_SciapsPassthrough :
			size = SCIAPS_KETEK_DPP3_PID_SciapsPassthrough_Base_RespLength + ketek_dpp3_spi_data->_data._ketek_dpp3._sciapsExpectedRespDataLength;
			break;
		default :
			size = SCIAPS_KETEK_DPP3_PID_Standard_RespLength;
			break;
	}

	if (ketek_dpp3_command_buffer[0] == 0xad) {
		uint8_t pid = ketek_dpp3_command_buffer[1];
		uint16_t prev_length = 0;
		uint16_t length_read = 0;

		//if (size == 0) {
		//	//size = 18;
		//	//ret = ketek_dpp3_spi_read_step_2(spi, ketek_dpp3_command_buffer + 1, size, true);
		//	ret = ketek_dpp3_spi_read_in_steps(spi, ketek_dpp3_command_buffer + 1, KETEK_DPP3_CMD_SIZE(ketek_dpp3_spi_data->_data._ketek_dpp3._mcaDataLength), &length_read);
		//	size = length_read;
		//}
		//else {
			ret = ketek_dpp3_spi_read(spi, ketek_dpp3_command_buffer + 1, size + 2, &length_read);
		//}
		ketek_dpp3_command_buffer[0] = 0xda;
		if (ret < 0) {
			dev_err(&spi->dev, "%s: ketek_dpp3_spi_read  failed with error code: %d", __func__, (int)ret);
			return ret;
		}
		else {
			//uint16_t length_read = ketek_dpp3_command_buffer[1];
			//length_read <<= 8;
			//length_read |= ketek_dpp3_command_buffer[2];
			if (size != length_read) {
				dev_err(&spi->dev, "%s: ketek_dpp3_spi_read completed with lengths mismatch(!!!). size = %d; length_read = %d ", __func__, (int)size, length_read);
				ketek_dpp3_command_buffer[0] = 0x00;
				return -EFAULT;
			}
			else {
				dev_info(&spi->dev, "%s: ketek_dpp3_spi_read is good. size = %d; length_read = %d ", __func__, (int)size, length_read);
			}

#if SCIAPS_DEBUG
			if (length_read > 128) {
				dev_info(&spi->dev,"%s: SciapsPassthrough data (length: %d, expected length:%d) ==> %.2x:%.2x:%.2x:%.2x:%.2x:%.2x:%.2x:%.2x:%.2x --- %.2x:%.2x:%.2x:%.2x:%.2x:%.2x:%.2x:%.2x", __func__
						, length_read
						, (int)size
						, ketek_dpp3_command_buffer[1]
						, ketek_dpp3_command_buffer[2]
						, ketek_dpp3_command_buffer[3]
						, ketek_dpp3_command_buffer[4]
						, ketek_dpp3_command_buffer[5]
						, ketek_dpp3_command_buffer[6]
						, ketek_dpp3_command_buffer[7]
						, ketek_dpp3_command_buffer[8]
						, ketek_dpp3_command_buffer[9]

						, ketek_dpp3_command_buffer[length_read + 3 -8]
						, ketek_dpp3_command_buffer[length_read + 3 -7]
						, ketek_dpp3_command_buffer[length_read + 3 -6]
						, ketek_dpp3_command_buffer[length_read + 3 -5]
						, ketek_dpp3_command_buffer[length_read + 3 -4]
						, ketek_dpp3_command_buffer[length_read + 3 -3]
						, ketek_dpp3_command_buffer[length_read + 3 -2]
						, ketek_dpp3_command_buffer[length_read + 3 -1]
						);
			}
			else if (length_read >= 7) {
				dev_info(&spi->dev,"%s: SciapsPassthrough data (length: %d, expected length:%d) ==> %.2x:%.2x:%.2x:%.2x:%.2x:%.2x:%.2x:%.2x:%.2x:%.2x", __func__
						, length_read
						, (int)size
						, ketek_dpp3_command_buffer[1]
						, ketek_dpp3_command_buffer[2]
						, ketek_dpp3_command_buffer[3]
						, ketek_dpp3_command_buffer[4]
						, ketek_dpp3_command_buffer[5]
						, ketek_dpp3_command_buffer[6]
						, ketek_dpp3_command_buffer[7]
						, ketek_dpp3_command_buffer[8]
						, ketek_dpp3_command_buffer[9]
						, ketek_dpp3_command_buffer[10]
						);

			}
#endif
			ketek_dpp3_command_buffer[1] = pid;
		}
		if (ketek_dpp3_command_buffer[3] == SCIAPS_KETEK_DPP3_PID_MCANumberOfBins) {
			if (ketek_dpp3_command_buffer[6] != ketek_dpp3_spi_data->_data._ketek_dpp3._numberOfBins
						&& ketek_dpp3_command_buffer[6] >= SCIAPS_KETEK_DPP3_PID_MCANumberOfBins_Min
						&& ketek_dpp3_command_buffer[6] <= SCIAPS_KETEK_DPP3_PID_MCANumberOfBins_Max) {
				prev_length = ketek_dpp3_spi_data->_data._ketek_dpp3._mcaDataLength;
				ketek_dpp3_spi_data->_data._ketek_dpp3._numberOfBins = ketek_dpp3_command_buffer[6];
				ketek_dpp3_spi_data->_data._ketek_dpp3._mcaDataLength = SCIAPS_KETEK_DPP3_MCADataLength(ketek_dpp3_spi_data->_data._ketek_dpp3._numberOfBins, ketek_dpp3_spi_data->_data._ketek_dpp3._bytesPerBin);
				dev_info(&spi->dev, "%s: Updated MCA Data Length: %d", __func__, ketek_dpp3_spi_data->_data._ketek_dpp3._mcaDataLength);
			}
		}

		if (ketek_dpp3_command_buffer[3] == SCIAPS_KETEK_DPP3_PID_MCABytesPerBin) {
			if (ketek_dpp3_command_buffer[6] != ketek_dpp3_spi_data->_data._ketek_dpp3._bytesPerBin
						&& ketek_dpp3_command_buffer[6] >= SCIAPS_KETEK_DPP3_PID_MCABytesPerBin_Min
						&& ketek_dpp3_command_buffer[6] <= SCIAPS_KETEK_DPP3_PID_MCABytesPerBin_Max) {
				prev_length = ketek_dpp3_spi_data->_data._ketek_dpp3._mcaDataLength;
				ketek_dpp3_spi_data->_data._ketek_dpp3._bytesPerBin = ketek_dpp3_command_buffer[6];
				ketek_dpp3_spi_data->_data._ketek_dpp3._mcaDataLength = SCIAPS_KETEK_DPP3_MCADataLength(ketek_dpp3_spi_data->_data._ketek_dpp3._numberOfBins, ketek_dpp3_spi_data->_data._ketek_dpp3._bytesPerBin);
				dev_info(&spi->dev, "%s: Updated MCA Data Length: %d", __func__, ketek_dpp3_spi_data->_data._ketek_dpp3._mcaDataLength);
			}
		}
		if (prev_length) {
			if (prev_length < ketek_dpp3_spi_data->_data._ketek_dpp3._mcaDataLength) {
#ifdef USE_DYNAMIC
				ketek_dpp3_command_buffer = krealloc(ketek_dpp3_command_buffer, KETEK_DPP3_BUFFER_SIZE(ketek_dpp3_spi_data->_data._ketek_dpp3._mcaDataLength), GFP_KERNEL);

				if (!ketek_dpp3_command_buffer) {
					dev_err(&spi->dev, "%s: unable to allocate command buffer of %d bytes", __func__, KETEK_DPP3_BUFFER_SIZE(ketek_dpp3_spi_data->_data._ketek_dpp3._mcaDataLength));
					return -ENOMEM;
				}
#else
				//nothing to do
#endif

			}
		}

	}
	{
		processed = 0;
		DEV_DBG(&spi->dev, "%s: *ppos = %d, count = %d, size = %d", __func__, (int)*ppos, (int)count, (int)size);

		if (*ppos > size) {
			// Nothing to do
			ketek_dpp3_command_buffer[0] = 0x00;
			*ppos = 0;
			DEV_DBG(&spi->dev, "%s: --nothing todo--> *ppos = %d, count = %d, size = %d", __func__, (int)*ppos, (int)count, (int)size);
			return 0;
		}
		else if (*ppos + count > size) {
			count = size - *ppos;
			DEV_DBG(&spi->dev, "%s: --updated--> *ppos = %d, count = %d, size = %d", __func__, (int)*ppos, (int)count, (int)size);
		}
		else {
			DEV_DBG(&spi->dev, "%s: --continue--> *ppos = %d, count = %d, size = %d", __func__, (int)*ppos, (int)count, (int)size);
		}


		for (i = *ppos; count > 0; ++i, ++p, --count) {
			if (__put_user(ketek_dpp3_command_buffer[i+3], p)) {
				dev_err(&spi->dev, "%s: __put_user FAILED!!!!", __func__);
				return -EFAULT;
			}
			++processed;
		}

		if ( i >= size) {
			ketek_dpp3_command_buffer[0] = 0x00;
			DEV_DBG(&spi->dev, "%s: Completed!", __func__);
			*ppos = 0;
		}
		else {
			DEV_DBG(&spi->dev, "%s: More data available!", __func__);
			*ppos = i;
		}
		//*ppos = i;
	}
	return processed;
}

static ssize_t ketek_dpp3_char_dev_write(struct file *file, const char __user *buf,
			   size_t count, loff_t *ppos)
{
	unsigned int i;
	const char __user *p = buf;
	char c;
	ssize_t ret;

	*ppos = 0;

	if (!access_ok(VERIFY_READ, buf, count))
		return -EFAULT;

	if (ketek_dpp3_command_buffer == 0)
		return -ENOMEM;

	if (ketek_dpp3_spi_data == 0)
		return -ENODEV;

	for (i = 0; count > 0; ++i, ++p, --count) {
		if (__get_user(c, p))
			return -EFAULT;
		ketek_dpp3_command_buffer[i+1] = c;
	}

	ret = i;

	if (i) {
		struct spi_device *spi = ketek_dpp3_spi_data->_spi;
		if (ketek_dpp3_command_buffer[1] == SCIAPS_KETEK_DPP3_PID_SciapsPassthrough) {
			if (i >= 6
				&& (ketek_dpp3_command_buffer[2] == 0 /*read*/|| ketek_dpp3_command_buffer[2] == 2/*exec*/)) {
				uint16_t length = ketek_dpp3_command_buffer[5];
				length <<= 8;
				length |= ketek_dpp3_command_buffer[6];

				if (length > SCIAPS_KETEK_DPP3_PID_MCARead_Max_RespLength) {
					dev_err(&spi->dev,"%s: SciapsPassthrough data requested length(%d) exceeds max length(%d)!!! Using max length instead."
							, __func__
							, length
							, SCIAPS_KETEK_DPP3_PID_MCARead_Max_RespLength);
					length = SCIAPS_KETEK_DPP3_PID_MCARead_Max_RespLength;
				}

				ketek_dpp3_spi_data->_data._ketek_dpp3._sciapsExpectedRespDataLength = length;
			}
			else {
				ketek_dpp3_spi_data->_data._ketek_dpp3._sciapsExpectedRespDataLength = 0;
			}

#if SCIAPS_DEBUG
			dev_info(&spi->dev,"%s: SciapsPassthrough data (length: %d, expected length: %d) ==> %.2x:%.2x:%.2x:%.2x:%.2x:%.2x:%.2x --- %.2x:%.2x:%.2x:%.2x:%.2x:%.2x:%.2x", __func__
					, i
					, ketek_dpp3_spi_data->_data._ketek_dpp3._sciapsExpectedRespDataLength
					, ketek_dpp3_command_buffer[1]
					, ketek_dpp3_command_buffer[2]
					, ketek_dpp3_command_buffer[3]
					, ketek_dpp3_command_buffer[4]
					, ketek_dpp3_command_buffer[5]
					, ketek_dpp3_command_buffer[6]
					, ketek_dpp3_command_buffer[7]

					, i>=6?ketek_dpp3_command_buffer[i-6]:0
					, i>=5?ketek_dpp3_command_buffer[i-5]:0
					, i>=4?ketek_dpp3_command_buffer[i-4]:0
					, i>=3?ketek_dpp3_command_buffer[i-3]:0
					, i>=2?ketek_dpp3_command_buffer[i-2]:0
					, i>=1?ketek_dpp3_command_buffer[i-1]:0
					, ketek_dpp3_command_buffer[i]
					);
#else
			dev_info(&spi->dev,"%s: SciapsPassthrough data (length: %d, expected length: %d)", __func__
					, i
					, ketek_dpp3_spi_data->_data._ketek_dpp3._sciapsExpectedRespDataLength);
#endif
			// AD: if _sciapsExpectedRespDataLength is larger than _mcaDataLength Sciaps Dpp's MCAData   is being read from DPP.
			// updating _mcaDataLength  and reallocating the buffer!
			if (ketek_dpp3_spi_data->_data._ketek_dpp3._mcaDataLength < ketek_dpp3_spi_data->_data._ketek_dpp3._sciapsExpectedRespDataLength) {

				ketek_dpp3_spi_data->_data._ketek_dpp3._mcaDataLength = ketek_dpp3_spi_data->_data._ketek_dpp3._sciapsExpectedRespDataLength;

				dev_info(&spi->dev, "%s: Updated MCA Data Length to accommodate Sciaps DPP expected response length: %d", __func__, ketek_dpp3_spi_data->_data._ketek_dpp3._mcaDataLength);

#ifdef USE_DYNAMIC
				ketek_dpp3_command_buffer = krealloc(ketek_dpp3_command_buffer, KETEK_DPP3_BUFFER_SIZE(ketek_dpp3_spi_data->_data._ketek_dpp3._mcaDataLength), GFP_KERNEL);

				if (!ketek_dpp3_command_buffer) {
					dev_err(&spi->dev, "%s: unable to allocate command buffer of %d bytes", __func__, KETEK_DPP3_BUFFER_SIZE(ketek_dpp3_spi_data->_data._ketek_dpp3._mcaDataLength));
					return -ENOMEM;
				}
#else
				//nothing to do
#endif

			}

		}

		dev_info(&spi->dev, "%s: Before ketek_dpp3_spi_transfer. length: %d", __func__, i);

		ret = ketek_dpp3_spi_transfer(spi, ketek_dpp3_command_buffer + 1, i);
		dev_info(&spi->dev, "%s: After ketek_dpp3_spi_transfer. ret = %d", __func__, (int)ret);

		if (ret >= 0) {
			//if (ketek_dpp3_command_buffer[1] != SCIAPS_KETEK_DPP3_PID_MCUPassthrough) {
				dev_info(&spi->dev, "%s: Before ketek_dpp3_check_status", __func__);
				ret = ketek_dpp3_check_status(spi, 2*SCIAPS_KETEK_DPP3_STATUS_CHECK_RETRIES, 2*SCIAPS_KETEK_DPP3_STATUS_CHECK_TIMEOUT_US);
				dev_info(&spi->dev, "%s: After ketek_dpp3_check_status. ret = %d", __func__, (int)ret);
				if (ret >= 0) {
					ret = i;
					ketek_dpp3_command_buffer[0] = 0xad;
				}
			//}
			//else {
			//	ret = i;
			//	ketek_dpp3_command_buffer[0] = 0xad;
			//}
		}
	}

	return ret;
}

const struct file_operations ketek_dpp3_char_dev_fops = {
	.owner		= THIS_MODULE,
	.read		= ketek_dpp3_char_dev_read,
	.write		= ketek_dpp3_char_dev_write,
};

#define KETEK_DPP3_CHAR_DEV_MINOR 1
static struct miscdevice ketek_dpp3_char_dev = {
	KETEK_DPP3_CHAR_DEV_MINOR,
	"ketek-dpp3",
	&ketek_dpp3_char_dev_fops
};


static const struct spi_device_id sciaps_ketek_dpp3_spi_device_id_table[] = {
	{ .name = "ketek-dpp3",	.driver_data = (kernel_ulong_t)SCIAPS_DRIVER_ID_KETEK_DPP3_SPI,	},
	{}
};
MODULE_DEVICE_TABLE(spi, sciaps_ketek_dpp3_spi_device_id_table);

static const struct of_device_id sciaps_ketek_dpp3_spi_of_match_table[] = {
	{ .compatible = "ketek,dpp3",	.data = &sciaps_ketek_dpp3_spi_device_id_table[0]	},
	{}
};
MODULE_DEVICE_TABLE(of, sciaps_ketek_dpp3_spi_of_match_table);


//#define SCIAPS_KETEK_DPP3_SPI_SPEED_HZ		SCIAPS_SPI_SPEED_HZ_Default
#define SCIAPS_KETEK_DPP3_SPI_SPEED_HZ		10000000

static struct sciaps_data_t* ketek_dpp3_spi_check_and_get_data(struct spi_device *spi)
{
	struct sciaps_data_t* data;
	uint8_t device_id = SCIAPS_DRIVER_ID_KETEK_DPP3_SPI;
	if (!spi) {
		return NULL;
	}
	data = spi_get_drvdata(spi);
	if (!data) {
		return NULL;
	}
	if (data->_device_id != device_id) {
		return NULL;
	}
	return data;
}

static int ketek_dpp3_spi_transfer(struct spi_device *spi, uint8_t *data, uint16_t length) {
	int ret;

	dev_info(&spi->dev, "%s: Enter", __func__);

	if (!spi || !data || !length) {
		return -EINVAL;
	}


	{
		struct spi_message		msg;
		struct spi_transfer		xfer[2];
		uint8_t	cmd_data[3];

		memset(&msg,0x00,sizeof (msg));
		memset(&xfer,0x00,sizeof (xfer));
		cmd_data[0] = SCIAPS_KETEK_DPP3_CMD_TRANSFER;
		cmd_data[1] = (uint8_t)(length >> 8);
		cmd_data[2] = (uint8_t)length;

		xfer[0].tx_buf		= cmd_data;
		xfer[0].rx_buf		= NULL;
		xfer[0].speed_hz	= SCIAPS_KETEK_DPP3_SPI_SPEED_HZ;
		xfer[0].len			= sizeof (cmd_data) / sizeof (*cmd_data);

		xfer[1].tx_buf		= data;
		xfer[1].rx_buf		= NULL;
		xfer[1].speed_hz	= SCIAPS_KETEK_DPP3_SPI_SPEED_HZ;
		xfer[1].len			= length;
		xfer[1].cs_change	= 1;
		xfer[1].delay_usecs	= 1;


		spi_message_init(&msg);
		spi_message_add_tail(&xfer[0], &msg);
		spi_message_add_tail(&xfer[1], &msg);

		ret = spi_sync(spi, &msg);

		if (ret < 0) {
			dev_info(&spi->dev, "%s: spi_sync failed with errno = %d", __func__,ret);
		}

	}

	dev_info(&spi->dev, "%s: Exit. ret = %d", __func__, ret);

	return ret;

}


static int ketek_dpp3_spi_read_step_1(struct spi_device *spi, uint8_t *data, uint16_t length, uint16_t *length_read, bool final) {
	int ret;

	if (!spi || !data || !length) {
		return -EINVAL;
	}


	{
		struct spi_message		msg;
		struct spi_transfer		xfer[2];
		uint8_t	cmd_data[1];

		memset(&msg,0x00,sizeof (msg));
		memset(&xfer,0x00,sizeof (xfer));
		cmd_data[0] = SCIAPS_KETEK_DPP3_CMD_READ;

		xfer[0].tx_buf		= cmd_data;
		xfer[0].rx_buf		= NULL;
		xfer[0].speed_hz	= SCIAPS_KETEK_DPP3_SPI_SPEED_HZ;
		xfer[0].len			= sizeof (cmd_data) / sizeof (*cmd_data);

		xfer[1].tx_buf		= NULL;
		xfer[1].rx_buf		= data;
		xfer[1].speed_hz	= SCIAPS_KETEK_DPP3_SPI_SPEED_HZ;
		xfer[1].len			= length;

		if (final) {
			xfer[1].cs_change	= 1;
			xfer[1].delay_usecs	= 1;
		}

		spi_message_init(&msg);
		spi_message_add_tail(&xfer[0], &msg);
		spi_message_add_tail(&xfer[1], &msg);
		ret = spi_sync(spi, &msg);

		if (ret < 0) {
			dev_info(&spi->dev, "%s: spi_sync failed with errno = %d", __func__,ret);
		}
		else {
			uint16_t read = data[0];
			read <<= 8;
			read |= data[1];
			if (length_read)
				*length_read = read;
			dev_info(&spi->dev, "%s: spi_sync is OK: length = %d; length_read = %d;", __func__, length, read);
			//dev_info(&spi->dev, "%s: spi_sync is OK: length = %d; length_read = %d ==> %.2x:%.2x:%.2x:%.2x:%.2x:%.2x:%.2x:%.2x:%.2x:%.2x:%.2x:%.2x", __func__, length, read
			//		, data[0], data[1], data[2], data[3], data[4]
			//		, data[5], data[6], data[7], data[8], data[9], data[10], data[11]
			//		);
		}

	}

	return ret;

}

#if 0
static int ketek_dpp3_spi_read_step_2(struct spi_device *spi, uint8_t *data, uint16_t length, bool final) {
	int ret;

	if (!spi || !data || !length) {
		return -EINVAL;
	}


	{
		struct spi_message		msg;
		struct spi_transfer		xfer[1];
		//uint8_t	cmd_data[1];

		memset(&msg,0x00,sizeof (msg));
		memset(&xfer,0x00,sizeof (xfer));
		//cmd_data[0] = SCIAPS_KETEK_DPP3_CMD_READ;

		//xfer[0].tx_buf		= cmd_data;
		//xfer[0].rx_buf		= data;
		//xfer[0].speed_hz	= SCIAPS_KETEK_DPP3_SPI_SPEED_HZ;
		//xfer[0].len			= 1;
		//if (length > 1) {
			xfer[0].tx_buf		= NULL;
			xfer[0].rx_buf		= data;
			xfer[0].speed_hz	= SCIAPS_KETEK_DPP3_SPI_SPEED_HZ;
			xfer[0].len			= length;
		//}
		if (final) {
			xfer[0].cs_change	= 1;
			xfer[0].delay_usecs	= 1;
		}

		spi_message_init(&msg);
		spi_message_add_tail(&xfer[0], &msg);
		//if (length > 1)
		//	spi_message_add_tail(&xfer[1], &msg);
		ret = spi_sync(spi, &msg);

		if (ret < 0) {
			dev_info(&spi->dev, "%s: spi_sync failed with errno = %d", __func__,ret);
		}
		else {
			dev_info(&spi->dev, "%s: spi_sync is OK: length = %d ==> %.2x:%.2x:%.2x:%.2x:%.2x:%.2x:%.2x:%.2x:%.2x:%.2x", __func__, length
					, data[0], data[1], data[2], data[3], data[4]
					, data[5], data[6], data[7], data[8], data[9]
					);
		}

	}

	return ret;
}
#endif
static int ketek_dpp3_spi_read(struct spi_device *spi, uint8_t *data, uint16_t length, uint16_t *length_read) {
	return ketek_dpp3_spi_read_step_1(spi, data, length, length_read, true);
}
#if 0
static int ketek_dpp3_spi_read_in_steps(struct spi_device *spi, uint8_t *data, uint16_t length, uint16_t *size) {
	int ret;
	uint16_t length_read;

	if (!spi || !data || !size || length < 2) {
		return -EINVAL;
	}

	ret = ketek_dpp3_spi_read_step_1(spi, data, 2, &length_read, false);

	if (ret < 0) {
		dev_err(&spi->dev, "%s: ketek_dpp3_spi_read_step_1 failed with errno = %d", __func__, ret);
		return ret;
	}

	if ((uint32_t)(length) < (uint32_t)(length_read) + 2) {
		dev_err(&spi->dev, "%s: Buffer is too short. length: %d.", __func__, length);
		return -EINVAL;
	}

	ret = ketek_dpp3_spi_read_step_2(spi, data + 2, length_read, true);

	if (ret < 0) {
		dev_err(&spi->dev, "%s: ketek_dpp3_spi_read_step_2 failed with errno = %d", __func__, ret);
	}
	else {
		*size = length_read + 2;
	}
	return ret;
}
#endif
static int ketek_dpp3_spi_status(struct spi_device *spi, uint8_t* data) {
	int ret;

	if (!spi) {
		return -EINVAL;
	}


	{
		struct spi_message		msg;
		struct spi_transfer		xfer[2];
		uint8_t	cmd_data[1];
		uint8_t	status[1];

		memset(&msg,0x00,sizeof (msg));
		memset(&xfer,0x00,sizeof (xfer));
		memset(status, 0x00, sizeof (status) / sizeof (*status));

		cmd_data[0] = SCIAPS_KETEK_DPP3_CMD_STATUS;

		xfer[0].tx_buf		= cmd_data;
		xfer[0].rx_buf		= NULL;
		xfer[0].speed_hz	= SCIAPS_KETEK_DPP3_SPI_SPEED_HZ;
		xfer[0].len			= sizeof (cmd_data) / sizeof (*cmd_data);

		xfer[1].tx_buf		= NULL;
		xfer[1].rx_buf		= status;
		xfer[1].speed_hz	= SCIAPS_KETEK_DPP3_SPI_SPEED_HZ;
		xfer[1].len			= sizeof (status) / sizeof (*status);
		xfer[1].cs_change	= 1;
		xfer[1].delay_usecs	= 1;


		spi_message_init(&msg);
		spi_message_add_tail(&xfer[0], &msg);
		spi_message_add_tail(&xfer[1], &msg);
		ret = spi_sync(spi, &msg);

		if (ret < 0) {
			dev_info(&spi->dev, "%s: spi_sync failed with errno = %d", __func__,ret);
		}
		else {
			dev_info(&spi->dev, "%s: Success. Status: 0x%.2x", __func__, status[0]);
			if (data) {
				*data = status[0];
			}
		}

	}

	return ret;

}

static int ketek_dpp3_check_status(struct spi_device *spi, uint16_t  attempts, uint32_t delay_usec) {
	int ret = -EAGAIN;
	uint16_t i;
	uint8_t status;

	for (i = 0; i < attempts; i++) {
		status = 0x00;
		if ((ret = ketek_dpp3_spi_status(spi, &status)) < 0) {
			break;
		}
		else {
			if (status == SCIAPS_KETEK_DPP3_STATUS_PENDING) {
				udelay(delay_usec);
				ret = -EAGAIN;
				continue;
			}
			else {
				if (status == SCIAPS_KETEK_DPP3_STATUS_READY) {
					ret = 0;
				}
				else {
					ret = -ENOSYS;
				}
				break;
			}
		}
	}

	return ret;
}

static int ketek_dpp3_read_parameter(struct spi_device *spi, uint8_t pid, uint16_t *value) {
	int ret;
	uint8_t req_data[4];
	uint8_t resp_data[6];
	uint16_t req_data_length = (sizeof (req_data) / sizeof (*req_data));
	uint16_t resp_data_length = (sizeof (resp_data) / sizeof (*resp_data));
	uint16_t length_read;

	if (!spi) {
		return -EINVAL;
	}

	memset(req_data, 0x00, req_data_length);
	memset(resp_data, 0x00, resp_data_length);

	req_data[0] = pid;

	ret = ketek_dpp3_spi_transfer(spi, req_data, req_data_length);

	if (ret >= 0) {
		ret = ketek_dpp3_check_status(spi, SCIAPS_KETEK_DPP3_STATUS_CHECK_RETRIES, SCIAPS_KETEK_DPP3_STATUS_CHECK_TIMEOUT_US);
	}

	if (ret >= 0) {
		ret = ketek_dpp3_spi_read(spi, resp_data, resp_data_length, &length_read);
		if (ret >= 0) {
			//uint16_t length_read = resp_data[0];
			//length_read <<= 8;
			//length_read |= resp_data[1];

			dev_info(&spi->dev, "%s: ketek_dpp3_spi_read is OK: resp_data_length = %d; length_read = %d. data recvd: 0x%.2x:0x%.2x:0x%.2x:0x%.2x", __func__
										, resp_data_length
										, length_read
										, resp_data[2], resp_data[3], resp_data[4], resp_data[5]);

			if (length_read == req_data_length && req_data[0] == resp_data[2]) {
				uint16_t tmp = resp_data[4];
				tmp <<= 8;
				tmp |= resp_data[5];
				dev_info(&spi->dev, "%s: Value for pid %d is 0x%.4x", __func__, pid, tmp);
				if (value) {
					*value = tmp;
				}
			}
		}
	}

	return ret;
}

static int ketek_dpp3_get_board_temperature(struct spi_device *spi, uint16_t *temperature) {

	return ketek_dpp3_read_parameter(spi, 0x49, temperature);
#if 0
	int ret;
	uint8_t temp_req_data[4] = {0x49, 0x00, 0x00, 0x00};
	uint8_t temp_resp_data[6];
	uint16_t temp_req_data_length = (sizeof (temp_req_data) / sizeof (*temp_req_data));
	uint16_t temp_resp_data_length = (sizeof (temp_resp_data) / sizeof (*temp_resp_data));

	dev_info(&spi->dev, "%s: Enter", __func__);

	if (!spi) {
		return -EINVAL;
	}

	memset(temp_resp_data, 0x00, temp_resp_data_length);

	dev_info(&spi->dev, "%s: Before ketek_dpp3_spi_transfer", __func__);
	ret = ketek_dpp3_spi_transfer(spi, temp_req_data, temp_req_data_length);
	dev_info(&spi->dev, "%s: After ketek_dpp3_spi_transfer. ret = %d", __func__, ret);

	if (ret >= 0) {
		dev_info(&spi->dev, "%s: Before ketek_dpp3_check_status", __func__);
		ret = ketek_dpp3_check_status(spi, SCIAPS_KETEK_DPP3_STATUS_CHECK_RETRIES, SCIAPS_KETEK_DPP3_STATUS_CHECK_TIMEOUT_US);
		dev_info(&spi->dev, "%s: After ketek_dpp3_check_status. ret = %d", __func__, ret);
	}

	if (ret >= 0) {
		ret = ketek_dpp3_spi_read(spi, temp_resp_data, temp_resp_data_length);
		if (ret >= 0) {
			uint16_t length_read = temp_resp_data[0];
			length_read <<= 8;
			length_read |= temp_resp_data[1];

			dev_info(&spi->dev, "%s: ketek_dpp3_spi_read is OK: temp_resp_data_length = %d; length_read = %d. data recvd: 0x%.2x:0x%.2x:0x%.2x:0x%.2x", __func__
										, temp_resp_data_length
										, length_read
										, temp_resp_data[2], temp_resp_data[3], temp_resp_data[4], temp_resp_data[5]);

			if (length_read == temp_req_data_length && temp_req_data[0] == temp_resp_data[2]) {
				uint16_t temp = temp_resp_data[4];
				temp <<= 8;
				temp |= temp_resp_data[5];
				dev_info(&spi->dev, "%s: Board Temperature is 0x%.4x", __func__, temp);
				if (temperature) {
					*temperature = temp;
				}
			}
		}
	}

	dev_info(&spi->dev, "%s: Exit. ret = %d", __func__, ret);

	return ret;
#endif
}

static ssize_t ketek_dpp3_board_temp_show(struct device* child, struct device_attribute* attr, char* buf)
{
	struct spi_device* spi = to_spi_device(child);
	struct sciaps_data_t* data = ketek_dpp3_spi_check_and_get_data(spi);
	uint16_t value = 0;
	int ret;

	if (!data) {
		return -EINVAL;
	}

	mutex_lock(&data->_lock);
	{
		ret = ketek_dpp3_get_board_temperature(spi, &value);
	}
	mutex_unlock(&data->_lock);

	if (ret < 0) {
		value = 0xffff;
	}
	return scnprintf(buf, PAGE_SIZE, "%x\n", value);
}

static DEVICE_ATTR(board_temp,		0444,	ketek_dpp3_board_temp_show,		NULL);

static int ketek_dpp3_create_files(struct spi_device *spi)
{
	return device_create_file(&spi->dev, &dev_attr_board_temp);
}

static void ketek_dpp3_remove_files(struct spi_device *spi)
{
	device_remove_file(&spi->dev, &dev_attr_board_temp);
}



static int sciaps_ketek_dpp3_spi_probe(struct spi_device *spi)
{

	int ret = 0;
	struct sciaps_data_t* data;

	ketek_dpp3_spi_data = 0;
#ifdef USE_DYNAMIC
	ketek_dpp3_command_buffer = 0;
#endif
	if (!spi) {
		dev_err(&spi->dev, "%s: Invalid params", __func__);
		return -EINVAL;
	}

	dev_info(&spi->dev, "%s: spi->max_speed_hz = %d;",		__func__, spi->max_speed_hz);
	dev_info(&spi->dev, "%s: spi->chip_select = %d;",		__func__, spi->chip_select);
	dev_info(&spi->dev, "%s: spi->mode = %d;",				__func__, spi->mode);
	dev_info(&spi->dev, "%s: spi->bits_per_word = %d;",		__func__, spi->bits_per_word);
	dev_info(&spi->dev, "%s: spi->modalias = %s;",			__func__, spi->modalias);
	dev_info(&spi->dev, "%s: spi->cs_gpio = %d;",			__func__, spi->cs_gpio);

	dev_info(&spi->dev, "%s: PAGE_SIZE = %lu;",			__func__, PAGE_SIZE);


	{
		uint8_t device_id = 0;
		const struct of_device_id *of_id =  of_match_device(sciaps_ketek_dpp3_spi_of_match_table, &spi->dev);

		if (!of_id) {
			ret = -ENODEV;
			dev_err(&spi->dev, "%s: there is NO match for %s;", __func__, spi->modalias);
			return ret;
		}
		if (!of_id->data) {
			ret = -ENODEV;
			dev_err(&spi->dev, "%s: there is NO .data for %s;", __func__, spi->modalias);
			return ret;
		}
		device_id = (uint8_t)((const struct spi_device_id*)(of_id->data))->driver_data;
		dev_info(&spi->dev, "%s: there is a match for spi->%s of device %d;", __func__, spi->modalias, device_id);
		if (device_id != SCIAPS_DRIVER_ID_KETEK_DPP3_SPI) {
			ret = -ENODEV;
			dev_err(&spi->dev, "%s: Invalid device ID for %s;", __func__, spi->modalias);
			return ret;
		}

		{
			uint8_t spi_cpol = 0;	/*SPI_CPOL: clock polarity: 0 - a clock idles at 0; 1 - a clock idles at 1; */
			uint8_t spi_cpha = 0;	/*SPI_CPHA: clock phase: 1 - 'out' changes on the leading edge; 0 - 'out' changes on the trailing edge*/

			dev_info(&spi->dev, "%s: processing for SCIAPS_DRIVER_ID_KETEK_DPP3_SPI...", __func__);
			spi->bits_per_word = 8;
			dev_info(&spi->dev, "%s: spi->bits_per_word = %d now;", __func__, spi->bits_per_word);
			dev_info(&spi->dev, "%s: Clock idles at %d; Leading edge change: %d", __func__, spi_cpha, spi_cpol);

			spi->mode = (spi_cpha|spi_cpol);
			dev_info(&spi->dev, "%s: spi->mode = %d now;", __func__, spi->mode);

			data = devm_kzalloc(&spi->dev, sizeof(struct sciaps_data_t), GFP_KERNEL);


			if (!data) {
				ret = -ENOMEM;
				dev_err(&spi->dev, "%s: unable to allocate buffer for struct sciaps_ketek_dpp3 for %s", __func__, spi->modalias);
				return ret;
			}


			data->_device_id					= device_id;
			data->_spi							= spi;


			data->_data._ketek_dpp3._numberOfBins	= SCIAPS_KETEK_DPP3_MCANumberOfBins_Default;
			data->_data._ketek_dpp3._bytesPerBin	= SCIAPS_KETEK_DPP3_MCABytesPerBin_Default;
			data->_data._ketek_dpp3._mcaDataLength	= SCIAPS_KETEK_DPP3_MCADataLength_Default;
			data->_data._ketek_dpp3._sciapsExpectedRespDataLength = 0;

			dev_info(&spi->dev, "%s: Default MCA Data Length: %d", __func__, data->_data._ketek_dpp3._mcaDataLength);
#ifdef USE_DYNAMIC
			ketek_dpp3_command_buffer = kmalloc(KETEK_DPP3_BUFFER_SIZE(data->_data._ketek_dpp3._mcaDataLength), GFP_KERNEL);

			if (!ketek_dpp3_command_buffer) {
				ret = -ENOMEM;
				dev_err(&spi->dev, "%s: unable to allocate command buffer of %d bytes", __func__, KETEK_DPP3_BUFFER_SIZE(data->_data._ketek_dpp3._mcaDataLength));
				return ret;
			}
#else
			//nothing to do
#endif

			if (ketek_dpp3_create_files(spi) != 0 ) {
				ret = -ENODEV;
				dev_err(&spi->dev, "%s: unable to create device files for for sciaps_lt_ltc1658", __func__);
				goto sciaps_ketek_dpp3_spi_probe_error_free_mem;
			}
			mutex_init(&data->_lock);
			spi_set_drvdata(spi, data);
			dev_info(&spi->dev, "%s: drvdata set for %s;", __func__, spi->modalias);

		}




		ret = spi_setup(spi);
		if (ret < 0) {
			dev_err(&spi->dev, "%s: spi_setup failed  for %s with errno: %d;", __func__, spi->modalias, ret);
			goto sciaps_ketek_dpp3_spi_probe_error;
		}
		else {
			dev_info(&spi->dev, "%s: spi_setup success for %s;", __func__, spi->modalias);

			{
				uint16_t value;
				int rv;
				bool updated  = false;
				rv = ketek_dpp3_read_parameter(spi, SCIAPS_KETEK_DPP3_PID_MCANumberOfBins, &value);
				if (rv >= 0
						&& value >= SCIAPS_KETEK_DPP3_PID_MCANumberOfBins_Min
						&& value <= SCIAPS_KETEK_DPP3_PID_MCANumberOfBins_Max
						&& value != data->_data._ketek_dpp3._numberOfBins) {
					data->_data._ketek_dpp3._numberOfBins = value;
					updated = true;
				}
				rv = ketek_dpp3_read_parameter(spi, SCIAPS_KETEK_DPP3_PID_MCABytesPerBin, &value);
				if (rv >= 0
						&& value >= SCIAPS_KETEK_DPP3_PID_MCABytesPerBin_Min
						&& value <= SCIAPS_KETEK_DPP3_PID_MCABytesPerBin_Max
						&& value != data->_data._ketek_dpp3._bytesPerBin) {
					data->_data._ketek_dpp3._bytesPerBin = value;
					updated = true;
				}
				if (updated) {
					uint16_t length = data->_data._ketek_dpp3._mcaDataLength;
					data->_data._ketek_dpp3._mcaDataLength	= SCIAPS_KETEK_DPP3_MCADataLength(data->_data._ketek_dpp3._numberOfBins, data->_data._ketek_dpp3._bytesPerBin);
					dev_info(&spi->dev, "%s: Updated MCA Data Length: %d", __func__, data->_data._ketek_dpp3._mcaDataLength);
					if (length < data->_data._ketek_dpp3._mcaDataLength) {
#ifdef USE_DYNAMIC

						ketek_dpp3_command_buffer = krealloc(ketek_dpp3_command_buffer, KETEK_DPP3_BUFFER_SIZE(data->_data._ketek_dpp3._mcaDataLength), GFP_KERNEL);

						if (!ketek_dpp3_command_buffer) {
							ret = -ENOMEM;
							dev_err(&spi->dev, "%s: unable to reallocate command buffer to %d bytes", __func__, KETEK_DPP3_BUFFER_SIZE(data->_data._ketek_dpp3._mcaDataLength));
							goto sciaps_ketek_dpp3_spi_probe_error;
						}
#else
						//nothing to do
#endif

					}

				}
			}

			{
				ret = misc_register(&ketek_dpp3_char_dev);
				if (ret >= 0) {
					ketek_dpp3_spi_data = data;
					ketek_dpp3_command_buffer[0] = 0;
				}
				else {
#ifdef USE_DYNAMIC
					if (ketek_dpp3_command_buffer) {
						kfree(ketek_dpp3_command_buffer);
						ketek_dpp3_command_buffer = 0;
					}
					ketek_dpp3_spi_data = 0;
#endif
				}
				dev_info(&spi->dev, "%s: misc_reigster for %s ret %d", __func__, spi->modalias, ret);
			}
		}
	}
	return 0;

sciaps_ketek_dpp3_spi_probe_error:
	ketek_dpp3_remove_files(spi);
	mutex_destroy(&data->_lock);

sciaps_ketek_dpp3_spi_probe_error_free_mem:
#ifdef USE_DYNAMIC
	if (ketek_dpp3_command_buffer) {
		kfree(ketek_dpp3_command_buffer);
		ketek_dpp3_command_buffer = 0;
	}
#endif
	return ret;
}

static int sciaps_ketek_dpp3_spi_remove(struct spi_device *spi)
{
	if (spi) {
		struct sciaps_data_t* data = spi_get_drvdata(spi);

		if (data) {
			mutex_destroy(&data->_lock);
			ketek_dpp3_remove_files(spi);

		}
		if (ketek_dpp3_spi_data) {
			misc_deregister(&ketek_dpp3_char_dev);
			ketek_dpp3_spi_data = 0;
		}
#ifdef USE_DYNAMIC
		if (ketek_dpp3_command_buffer) {
			kfree(ketek_dpp3_command_buffer);
			ketek_dpp3_command_buffer = 0;
		}
#endif
	}
	dev_err(&spi->dev, "%s: spi->modalias = %s;", __func__, spi->modalias);
	return 0;
}

static struct spi_driver sciaps_ketek_dpp3_spi_driver = {
	.id_table = sciaps_ketek_dpp3_spi_device_id_table,
	.driver = {
		.name = "sciaps-ketek-dpp3-spi",
		.owner = THIS_MODULE,
		.of_match_table = sciaps_ketek_dpp3_spi_of_match_table,
	},
	.probe = sciaps_ketek_dpp3_spi_probe,
	.remove = sciaps_ketek_dpp3_spi_remove,
};

static int __init sciaps_ketek_dpp3_spi_init(void)
{
	return spi_register_driver(&sciaps_ketek_dpp3_spi_driver);
}
subsys_initcall(sciaps_ketek_dpp3_spi_init);

static void __exit sciaps_ketek_dpp3_spi_exit(void)
{
	spi_unregister_driver(&sciaps_ketek_dpp3_spi_driver);
}
module_exit(sciaps_ketek_dpp3_spi_exit);

MODULE_DESCRIPTION("SciAps Ketek DPP3 SPI Driver");
MODULE_AUTHOR("Andre Doudkin");
MODULE_LICENSE("GPL v2");
