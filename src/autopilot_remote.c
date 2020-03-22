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
#include <string.h>
#include "delay.h"
#include "nrf24l01.h"
#include "stm32f10x.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "buzzer.h"
#include "autopilot_remote.h"

static QueueHandle_t autopilot_remote_queue_handle;
static TaskHandle_t this_task_handle;

void autopilot_remote_init(void)
{
	NVIC_InitTypeDef NVIC_InitStructure = {0};			// sets .NVIC_IRQChannelSubPriority to 0
	EXTI_InitTypeDef EXTI_InitStructure;

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource8);
	EXTI_InitStructure.EXTI_Line = EXTI_Line8;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0fU;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
	NVIC_Init(&NVIC_InitStructure);

	// init radio module and set as receiver
	if (RCC_GetFlagStatus(RCC_FLAG_PORRST) == SET)
	{
		// only fully init the radio module after a board power up
		nrf24l01_initialize(nrf24l01_CONFIG_DEFAULT_VAL | nrf24l01_CONFIG_PWR_UP | nrf24l01_CONFIG_PRIM_RX,
							true,
							nrf24l01_EN_AA_ENAA_NONE,
							nrf24l01_EN_RXADDR_DEFAULT_VAL,
							nrf24l01_SETUP_AW_DEFAULT_VAL,
							nrf24l01_SETUP_RETR_DEFAULT_VAL,
							nrf24l01_RF_CH_DEFAULT_VAL,
							nrf24l01_RF_SETUP_DEFAULT_VAL,
							NULL,
							NULL,
							nrf24l01_RX_ADDR_P2_DEFAULT_VAL,
							nrf24l01_RX_ADDR_P3_DEFAULT_VAL,
							nrf24l01_RX_ADDR_P4_DEFAULT_VAL,
							nrf24l01_RX_ADDR_P5_DEFAULT_VAL,
							NULL,
							1U,
							nrf24l01_RX_PW_P1_DEFAULT_VAL,
							nrf24l01_RX_PW_P2_DEFAULT_VAL,
							nrf24l01_RX_PW_P3_DEFAULT_VAL,
							nrf24l01_RX_PW_P4_DEFAULT_VAL,
							nrf24l01_RX_PW_P5_DEFAULT_VAL);
	}
	else
	{
		// just power cycle it
		nrf24l01_power_down_param(nrf24l01_CONFIG_DEFAULT_VAL | nrf24l01_CONFIG_PRIM_RX);
		delay_ms(20U);
		nrf24l01_power_up_param(true, nrf24l01_CONFIG_DEFAULT_VAL | nrf24l01_CONFIG_PRIM_RX);
	}
}

void autopilot_remote_task(void *parameters)
{
	uint8_t autopilot_remote_data;
	autopilot_remote_command_t autopilot_remote_command;

	(void)memcpy(&autopilot_remote_queue_handle, parameters, sizeof(QueueHandle_t));

	this_task_handle = xTaskGetCurrentTaskHandle();

	// signal main task that this task has started
	(void)xTaskNotifyGive(xTaskGetHandle("MAIN"));

    while (true)
    {
    	// wait for an interrupt from radio board
    	(void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    	// this delay is needed from time interrupt line is asserted until data can be read
    	// so make a buzzer click on base station when autopilot remote button is pressed
    	buzzer_on();
    	vTaskDelay((TickType_t)5);
    	buzzer_off();

    	// read the data from the radio chip
    	(void)nrf24l01_read_rx_payload((uint8_t *)&autopilot_remote_data, 1U);

		// acknowledge the interrupt on the radio chip
		nrf24l01_irq_clear_all();

		// send data read from radio chip to queue
		autopilot_remote_command = (autopilot_remote_command_t)autopilot_remote_data;
		(void)xQueueSend(autopilot_remote_queue_handle, (const void *)&autopilot_remote_command, (TickType_t)100);
    }
}
void EXTI9_5_IRQHandler(void)
{
	if (EXTI_GetITStatus(EXTI_Line8) == SET)
	{
		BaseType_t higher_priority_task_woken = pdFALSE;

		// clear the EXTI line pending bit
		EXTI_ClearITPendingBit(EXTI_Line8);

		// inform task that an interrupt has occurred from radio board
		vTaskNotifyGiveFromISR(this_task_handle, &higher_priority_task_woken);

		portYIELD_FROM_ISR(higher_priority_task_woken);
	}
}
