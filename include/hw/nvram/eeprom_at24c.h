#ifndef EEPROM_AT24C_H
#define EEPROM_AT24C_H

#include "qemu/typedefs.h"

void at24c_eeprom_write(DeviceState *dev,
                           uint32_t start, uint32_t count,
                           const char *buf);

#endif /* EEPROM_AT24C_H */
