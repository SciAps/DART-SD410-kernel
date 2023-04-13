#ifndef _LINUX_MFD_SCIAPS_MICRO_H_
#define _LINUX_MFD_SCIAPS_MICRO_H_

#define SCIAPS_MICRO_BATTERY_PRESENT		0x0000
#define SCIAPS_MICRO_BATTERY_NOT_PRESENT	0x0400

extern int sciaps_micro_check_battery_presence(void);
extern int sciaps_micro_read_register(uint8_t reg, uint16_t *value_out);

#endif // _LINUX_MFD_SCIAPS_MICRO_H_
