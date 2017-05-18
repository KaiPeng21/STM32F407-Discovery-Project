/*
 * Demo 2 - Interrupt and Pushbutton
 *
 * GPIO (General Purpose Input Output)
 *
 * Toggle On-Chip Red LED  (Port D Pin 14)
 * when On-Chip pushbutton is pressed (Port A Pin 1)
 *
 */


#include "stm32f4xx.h"

// initialize setting functions
void init_LED_settings(void);
void init_PushButton_settings(void);
void init_EXTI_settings(void);
void init_NVIC_settings(void);

// misc functions
void Delay(uint32_t nCount);

// user define variables
char press = 0;

int main(void)
{
	// LED and Pushbuttons Configurations
	init_LED_settings();
	init_PushButton_settings();

	// interrupt initializations
	init_EXTI_settings();
	init_NVIC_settings();

	while (1)
	{
		if (press == 1)
		{
			press = 0;

			if (GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_14) == 1)
			{
				GPIO_WriteBit(GPIOD, GPIO_Pin_14, Bit_RESET);
			}
			else
			{
				GPIO_WriteBit(GPIOD, GPIO_Pin_14, Bit_SET);
			}
		}
	}
}

void EXTI0_IRQHandler(void)
{
	if (EXTI_GetITStatus(EXTI_Line0))
	{
		press = 1;

		EXTI_ClearITPendingBit(EXTI_Line0);
	}
}

void init_LED_settings(void)
{
	// Enable AHB1 Clock on Port D
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	// Port D Pin 12 - On-Chip Green LED
	// Port D¡@Pin 14 - On-Chip Red LED
	GPIO_InitTypeDef GPIO_Instruct;
	GPIO_Instruct.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_14;
	GPIO_Instruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_Instruct.GPIO_OType = GPIO_OType_PP;
	GPIO_Instruct.GPIO_Speed = GPIO_Speed_50MHz;

	// Save GPIO Setting to Port D Register
	GPIO_Init(GPIOD, &GPIO_Instruct);
}

void init_PushButton_settings(void)
{
	// Enable AHB1 Clock on Port A
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	// Port A Pin 0 - On-Chip Pushbutton
	GPIO_InitTypeDef GPIO_Instruct;
	GPIO_Instruct.GPIO_Pin = GPIO_Pin_0;
	GPIO_Instruct.GPIO_Mode = GPIO_Mode_IN;
	GPIO_Instruct.GPIO_OType = GPIO_OType_PP;
	GPIO_Instruct.GPIO_Speed = GPIO_Speed_50MHz;

	// Save GPIO Setting to Port A Register
	GPIO_Init(GPIOA, &GPIO_Instruct);
}

void init_EXTI_settings(void)
{
	// Enable APB2 Clock for External Interrupt
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	// External Interrupt Setting
	EXTI_InitTypeDef EXTI_Instruct;
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource0);
	EXTI_Instruct.EXTI_Line = EXTI_Line0;
	EXTI_Instruct.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_Instruct.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_Instruct.EXTI_LineCmd = ENABLE;

	// Write EXTI Setting to register
	EXTI_Init(&EXTI_Instruct);
}

void init_NVIC_settings(void)
{
	// Nested Vector Interrupt Controller
	NVIC_InitTypeDef NVIC_Instruct;
	NVIC_Instruct.NVIC_IRQChannel = EXTI0_IRQn;
	NVIC_Instruct.NVIC_IRQChannelPreemptionPriority = 0x00;
	NVIC_Instruct.NVIC_IRQChannelSubPriority = 0x00;
	NVIC_Instruct.NVIC_IRQChannelCmd = ENABLE;

	// Write NVIC Setting to register
	NVIC_Init(&NVIC_Instruct);
}

void Delay(uint32_t nCount)
{
	while (nCount > 0)
	{
		nCount--;
	}
}
