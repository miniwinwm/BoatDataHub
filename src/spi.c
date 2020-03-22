#include <stdint.h>
#include "stm32f10x.h"
#include "spi.h"

void spi2_init(void)
{
	SPI_InitTypeDef SPI_InitStructure = {0};	// sets SPI_Direction = SPI_Direction_2Lines_FullDuplex, SPI_DataSize = SPI_DataSize_8b, SPI_CPOL = SPI_CPOL_Low, SPI_CPHA = SPI_CPHA_1Edge, SPI_FirstBit = SPI_FirstBit_MSB

	SPI_Cmd(SPI2, DISABLE);
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;
	SPI_Init(SPI2, &SPI_InitStructure);
	SPI_Cmd(SPI2, ENABLE);
}

uint8_t spi2_send_read_byte(uint8_t byte)
{
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET)
	{
	}

	SPI_I2S_SendData(SPI2, byte);

	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET)
	{
	}

	return SPI_I2S_ReceiveData(SPI2);
}
