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
#include "buttons.h"
#include "buzzer.h"

/**** SET UP FOR NORMALLY CLOSED BUTTONS ****/

static volatile uint8_t buttons_value = 0U;

void buttons_init(void)
{
	EXTI_InitTypeDef EXTI_InitStructure = {0};				// sets .EXTI_Mode to EXTI_Mode_Interrupt
	NVIC_InitTypeDef NVIC_InitStructure = {0};				// sets .NVIC_IRQChannelSubPriority to 0
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure = {0};	// sets .TIM_ClockDivision to TIM_CKD_DIV1, sets .TIM_CounterMode to TIM_CounterMode_Up

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource12);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource15);
	EXTI_InitStructure.EXTI_Line = EXTI_Line12 | EXTI_Line15;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0aU;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
	NVIC_Init(&NVIC_InitStructure);

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    TIM_Cmd(TIM4, DISABLE);
	TIM_TimeBaseStructure.TIM_Period = 0x1000U;
	TIM_TimeBaseStructure.TIM_Prescaler = 24U;
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
	TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
    NVIC_Init(&NVIC_InitStructure);
}

uint8_t buttons_get_value(void)
{
	uint8_t return_value = buttons_value;
	buttons_value = 0U;

	return return_value;
}

void TIM4_IRQHandler(void)
{
	static uint8_t button_event;
	static uint32_t button_time_counter;

    TIM_ClearITPendingBit(TIM4, TIM_IT_Update);

	if (button_time_counter == 10UL)
	{
		if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_12) == 1U)
		{
			button_event = TOP_BUTTON;
			button_time_counter = 24UL;
			buzzer_on();
		}
		else if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_15) == 1U)
		{
			button_event = BOTTOM_BUTTON;
			button_time_counter = 24UL;
			buzzer_on();
		}
		else
		{
			// kill timer and switch exti interrupts back on
		    TIM_Cmd(TIM4, DISABLE);
			EXTI->IMR |= (EXTI_Line12 | EXTI_Line15);
			button_time_counter = 0UL;
		}
	}
	else if (button_time_counter >= 28UL && button_time_counter < 124UL)
	{
		buzzer_off();
		if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_12) == 0U && GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_15) == 0U)
		{
			buttons_value = button_event;
			button_time_counter = 200UL;
		}
	}
	else if (button_time_counter == 124UL)
	{
		buttons_value = button_event + BUTTON_LONG_PRESS_OFFSET;
		button_time_counter = 200UL;
	}
	else if (button_time_counter >= 210UL)
	{
		if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_12) == 0U && GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_15) == 0U)
		{
			// kill timer and switch exti interrupts back on
			TIM_Cmd(TIM4, DISABLE);
			EXTI->IMR |= (EXTI_Line12 | EXTI_Line15);
			button_time_counter = 0UL;
		}
	}

    button_time_counter++;
}

void EXTI15_10_IRQHandler(void)
{
	bool start_timer = false;

	if (EXTI_GetITStatus(EXTI_Line12) == SET)
	{
		// switch off exti interrupts on line 12
		EXTI_ClearITPendingBit(EXTI_Line12);
		EXTI->IMR &= ~EXTI_Line12;
		start_timer = true;
	}
	else if (EXTI_GetITStatus(EXTI_Line15) == SET)
	{
		// switch off exti interrupts on line 15
		EXTI_ClearITPendingBit(EXTI_Line15);
		EXTI->IMR &= ~EXTI_Line15;
		start_timer = true;
	}

	if (start_timer)
	{
		TIM_SetCounter(TIM4, 0U);
	    TIM_Cmd(TIM4, ENABLE);
	}
}
