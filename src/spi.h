#ifndef SPI_H
#define SPI_H

#include <stdint.h>
#include "FreeRTOS.h"

void spi2_init(void);
uint8_t spi2_send_read_byte(uint8_t byte);

#endif
