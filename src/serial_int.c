/*
MIT License

Copyright (c) John Blaiklock 2020 Boat Data Hub

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include <stdint.h>
#include <stdbool.h>
#include "stm32f10x.h"
#include "serial.h"
#include "main.h"

static volatile uint8_t receive_buffer_1[RECEIVE_BUFFER_SIZE_1];
static volatile uint16_t receive_next_write_position_1;
static volatile uint16_t receive_next_read_position_1;
static volatile uint16_t bytes_available_in_receive_buffer_1;
static volatile uint8_t transmit_buffer_1[TRANSMIT_BUFFER_SIZE_1];
static volatile uint16_t transmit_next_write_position_1;
static volatile uint16_t transmit_next_read_position_1;
static volatile uint16_t bytes_free_in_transmit_buffer_1 = TRANSMIT_BUFFER_SIZE_1;

static volatile uint8_t receive_buffer_2[RECEIVE_BUFFER_SIZE_2];
static volatile uint16_t receive_next_write_position_2;
static volatile uint16_t receive_next_read_position_2;
static volatile uint16_t bytes_available_in_receive_buffer_2;

static volatile uint8_t receive_buffer_3[RECEIVE_BUFFER_SIZE_3];
static volatile uint16_t receive_next_write_position_3;
static volatile uint16_t receive_next_read_position_3;
static volatile uint16_t bytes_available_in_receive_buffer_3;
static volatile uint8_t transmit_buffer_3[TRANSMIT_BUFFER_SIZE_3];
static volatile uint16_t transmit_next_write_position_3;
static volatile uint16_t transmit_next_read_position_3;
static volatile uint16_t bytes_free_in_transmit_buffer_3 = TRANSMIT_BUFFER_SIZE_3;

void serial_init(uint32_t baud_rate_1, uint32_t baud_rate_2, uint32_t baud_rate_3)
{
	USART_InitTypeDef USART_InitStructure = {0};	// sets .USART_WordLength = USART_WordLength_8b, .USART_StopBits = USART_StopBits_1, .USART_Parity = USART_Parity_No, .USART_HardwareFlowControl = USART_HardwareFlowControl_None
	NVIC_InitTypeDef NVIC_InitStructure = {0};		// sets .NVIC_IRQChannelSubPriority to 0

	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x05U;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_Init(&NVIC_InitStructure);

	USART_InitStructure.USART_BaudRate = baud_rate_1;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &USART_InitStructure);

	USART_InitStructure.USART_BaudRate = baud_rate_3;
	USART_Init(USART3, &USART_InitStructure);

	USART_InitStructure.USART_BaudRate = baud_rate_2;
	USART_InitStructure.USART_Mode = USART_Mode_Rx;
	USART_Init(USART2, &USART_InitStructure);

	USART_Cmd(USART1, ENABLE);
	USART_Cmd(USART2, ENABLE);
	USART_Cmd(USART3, ENABLE);

	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
}

uint16_t serial_1_send_data(uint16_t length, const uint8_t *data)
{
	uint16_t i;
	uint16_t bytes_to_send;

    USART_ITConfig(USART1, USART_IT_TXE, DISABLE);

	if (length > bytes_free_in_transmit_buffer_1)
	{
		bytes_to_send = bytes_free_in_transmit_buffer_1;
	}
	else
	{
	    bytes_to_send = length;
	}

	for (i = 0U; i < bytes_to_send; i++)
	{
		transmit_buffer_1[transmit_next_write_position_1] = data[i];
		transmit_next_write_position_1++;
		if (transmit_next_write_position_1 == TRANSMIT_BUFFER_SIZE_1)
		{
			transmit_next_write_position_1 = 0U;
		}
	}

	bytes_free_in_transmit_buffer_1 -= bytes_to_send;

	if (bytes_free_in_transmit_buffer_1 < TRANSMIT_BUFFER_SIZE_1)
	{
	    USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
	}

	return bytes_to_send;
}

uint16_t serial_1_read_data(uint16_t buffer_length, uint8_t *data)
{
	uint16_t bytes_to_read;
	uint16_t i;

	USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

	if (bytes_available_in_receive_buffer_1 < buffer_length)
	{
		bytes_to_read = bytes_available_in_receive_buffer_1;
	}
	else
	{
		bytes_to_read = buffer_length;
	}

	bytes_available_in_receive_buffer_1 -= bytes_to_read;

	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

	for (i = 0U; i < bytes_to_read; i++)
	{
		data[i] = receive_buffer_1[receive_next_read_position_1];
		receive_next_read_position_1++;
		if (receive_next_read_position_1 == RECEIVE_BUFFER_SIZE_1)
		{
			receive_next_read_position_1 = 0U;
		}
	}

	return bytes_to_read;
}

void USART1_IRQHandler(void)
{
    if (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) != RESET)
	{
        uint8_t readByte = USART_ReceiveData(USART1);

        if (bytes_available_in_receive_buffer_1 < RECEIVE_BUFFER_SIZE_1)
        {
            receive_buffer_1[receive_next_write_position_1] = readByte;
            receive_next_write_position_1++;
            if (receive_next_write_position_1 == RECEIVE_BUFFER_SIZE_1)
            {
                receive_next_write_position_1 = 0U;
            }
            bytes_available_in_receive_buffer_1++;
        }
	}

    if (USART_GetITStatus(USART1, USART_IT_TXE) != RESET)
    {
        if (bytes_free_in_transmit_buffer_1 < TRANSMIT_BUFFER_SIZE_1)
    	{
    		USART_SendData(USART1, transmit_buffer_1[transmit_next_read_position_1]);
    		transmit_next_read_position_1++;
    		if (transmit_next_read_position_1 == TRANSMIT_BUFFER_SIZE_1)
    		{
    			transmit_next_read_position_1 = 0U;
    		}
    		bytes_free_in_transmit_buffer_1++;
    	}
    	else
    	{
    		USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
    	}
    }
}

uint16_t serial_2_read_data(uint16_t buffer_length, uint8_t *data)
{
    uint16_t bytes_to_read;
    uint16_t i;

	USART_ITConfig(USART2, USART_IT_RXNE, DISABLE);

    if (bytes_available_in_receive_buffer_2 < buffer_length)
    {
        bytes_to_read = bytes_available_in_receive_buffer_2;
    }
    else
    {
        bytes_to_read = buffer_length;
    }

    bytes_available_in_receive_buffer_2 -= bytes_to_read;

	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

    for (i = 0U; i < bytes_to_read; i++)
    {
        data[i] = receive_buffer_2[receive_next_read_position_2];
        receive_next_read_position_2++;
        if (receive_next_read_position_2 == RECEIVE_BUFFER_SIZE_2)
        {
            receive_next_read_position_2 = 0U;
        }
    }

    return bytes_to_read;
}

void USART2_IRQHandler(void)
{
    if (USART_GetFlagStatus(USART2, USART_FLAG_RXNE) != RESET)
    {
        uint8_t readByte = USART_ReceiveData(USART2);

        if (bytes_available_in_receive_buffer_2 < RECEIVE_BUFFER_SIZE_2)
        {
            receive_buffer_2[receive_next_write_position_2] = readByte;
            receive_next_write_position_2++;
            if (receive_next_write_position_2 == RECEIVE_BUFFER_SIZE_2)
            {
                receive_next_write_position_2 = 0U;
            }
            bytes_available_in_receive_buffer_2++;
        }
    }
}

uint16_t serial_3_send_data(uint16_t length, const uint8_t *pData)
{
    uint16_t i;
    uint16_t bytes_to_send;

    USART_ITConfig(USART3, USART_IT_TXE, DISABLE);

    if (length > bytes_free_in_transmit_buffer_3)
    {
        bytes_to_send = bytes_free_in_transmit_buffer_3;
    }
    else
    {
        bytes_to_send = length;
    }

    for (i = 0U; i < bytes_to_send; i++)
    {
        transmit_buffer_3[transmit_next_write_position_3] = pData[i];
        transmit_next_write_position_3++;
        if (transmit_next_write_position_3 == TRANSMIT_BUFFER_SIZE_3)
        {
            transmit_next_write_position_3 = 0U;
        }
    }

    bytes_free_in_transmit_buffer_3 -= bytes_to_send;

    if (bytes_free_in_transmit_buffer_3 < TRANSMIT_BUFFER_SIZE_3)
    {
        USART_ITConfig(USART3, USART_IT_TXE, ENABLE);
    }

    return bytes_to_send;
}

uint16_t serial_3_read_data(uint16_t buffer_length, uint8_t *pData)
{
    uint16_t bytes_to_read;
    uint16_t i;

	USART_ITConfig(USART3, USART_IT_RXNE, DISABLE);

    if (bytes_available_in_receive_buffer_3 < buffer_length)
    {
        bytes_to_read = bytes_available_in_receive_buffer_3;
    }
    else
    {
        bytes_to_read = buffer_length;
    }

    bytes_available_in_receive_buffer_3 -= bytes_to_read;

	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);

    for (i = 0U; i < bytes_to_read; i++)
    {
        pData[i] = receive_buffer_3[receive_next_read_position_3];
        receive_next_read_position_3++;
        if (receive_next_read_position_3 == RECEIVE_BUFFER_SIZE_3)
        {
            receive_next_read_position_3 = 0U;
        }
    }

    return bytes_to_read;
}

void USART3_IRQHandler(void)
{
    if (USART_GetFlagStatus(USART3, USART_FLAG_RXNE) != RESET)
    {
        uint8_t readByte = USART_ReceiveData(USART3);

        if (bytes_available_in_receive_buffer_3 < RECEIVE_BUFFER_SIZE_3)
        {
            receive_buffer_3[receive_next_write_position_3] = readByte;
            receive_next_write_position_3++;
            if (receive_next_write_position_3 == RECEIVE_BUFFER_SIZE_3)
            {
                receive_next_write_position_3 = 0U;
            }
            bytes_available_in_receive_buffer_3++;
        }
    }

    if (USART_GetITStatus(USART3, USART_IT_TXE) != RESET)
    {
        if (bytes_free_in_transmit_buffer_3 < TRANSMIT_BUFFER_SIZE_3)
        {
            USART_SendData(USART3, transmit_buffer_3[transmit_next_read_position_3]);
            transmit_next_read_position_3++;
            if (transmit_next_read_position_3 == TRANSMIT_BUFFER_SIZE_3)
            {
                transmit_next_read_position_3 = 0U;
            }
            bytes_free_in_transmit_buffer_3++;
        }
        else
        {
            USART_ITConfig(USART3, USART_IT_TXE, DISABLE);
        }
    }
}
