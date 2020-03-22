#ifndef FLASH_H
#define FLASH_H

#include <stdint.h>

#define FLASH_START_ADDRESS			0x0800FC00UL
#define FLASH_SECTOR_SIZE			0x400UL

void flash_store_data(void *data, uint32_t length);

#endif
