/*
 * Demo 3 - PWM
 *
 * Changing On-Chip Blue LED Intensity (Port D Pin 15)
 * Toggle the On-Chip Red LED  (Port D Pin 14) when On-Chip pushbutton is pressed (Port A Pin 1)
 * Oscillate the On-Chip Green LED (Port D Pin 12) at a rate of 2 second using TIM2 interrupt
 *
 */


#include "stm32f4xx.h"

// initialize setting functions
void init_LED_settings(void);
void init_PushButton_settings(void);
void init_EXTI_settings(void);
void init_NVIC_settings(void);
void init_TIM_settings(void);
void init_PWM_settings(void);

// misc functions
void Delay(uint32_t nCount);

// PWM Timer OC
TIM_OCInitTypeDef TIM_OC;

// user define variables
char press = 0;
char onems = 0;
char tenms = 0;
char onesec = 0;
int tenms_counter = 0;
int onesec_counter = 0;
char duty_state = 0;
int pulse = 0;

// user define constant
// PWM Duty Cycle Constant
int pulse_dut_25 = 2999;
int pulse_dut_50 = 4199;
int pulse_dut_75 = 6299;
int pulse_dut_100 = 8399;

int main(void)
{
	// LED and Pushbuttons Configurations
	init_LED_settings();
	init_PushButton_settings();

	// interrupt initializations
	init_EXTI_settings();
	init_NVIC_settings();

	// timer initializations
	init_TIM_settings();

	// PWM initializations
	init_PWM_settings();

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

			/* Toggle Duty Cycle using pushbuttons
			duty_state = (duty_state + 1) % 4;
			if (duty_state == 0)
			{
				TIM_OC.TIM_Pulse = pulse_dut_25;
			}
			else if (duty_state == 1)
			{
				TIM_OC.TIM_Pulse = pulse_dut_50;
			}
			else if (duty_state == 2)
			{
				TIM_OC.TIM_Pulse = pulse_dut_75;
			}
			else if (duty_state == 3)
			{
				TIM_OC.TIM_Pulse = pulse_dut_100;
			}
			TIM_OC4Init(TIM4, &TIM_OC);
			TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);
			*/
		}

		if (onems == 1)
		{
			onems = 0;
			pulse = pulse + 10;
			TIM_OC.TIM_Pulse = pulse % pulse_dut_100;
			TIM_OC4Init(TIM4, &TIM_OC);
			TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);
		}

		if (onesec == 1)
		{
			onesec = 0;

			GPIO_ToggleBits(GPIOD, GPIO_Pin_12);
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

void TIM2_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) == SET)
    {
    	onems = 1;

    	tenms_counter++;
    	if (tenms_counter >= 10)
    	{
    		tenms = 1;
    		tenms_counter = 0;
    	}

    	onesec_counter++;
    	if (onesec_counter >= 1000)
    	{
    		onesec = 1;
    		onesec_counter = 0;
    	}

        TIM_ClearITPendingBit(TIM2, TIM_FLAG_Update);
    }
}

void init_LED_settings(void)
{
	// Enable AHB1 Clock on Port D
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	// Port D Pin 12 - On-Chip Green LED
	// Port D�@Pin 14 - On-Chip Red LED
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

	// NVIC Setting for EXIT0
	NVIC_Instruct.NVIC_IRQChannel = EXTI0_IRQn;
	NVIC_Instruct.NVIC_IRQChannelPreemptionPriority = 0x00;
	NVIC_Instruct.NVIC_IRQChannelSubPriority = 0x00;
	NVIC_Instruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_Instruct);

	// NVIC Setting for TIM2
	NVIC_Instruct.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_Instruct.NVIC_IRQChannelPreemptionPriority = 0x00;
	NVIC_Instruct.NVIC_IRQChannelSubPriority = 0x01;
	NVIC_Instruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_Instruct);

}

void init_TIM_settings(void)
{
/*
 *  APB1 bus has a 42MHz clock. Internal PLL double this frequency for timer up to 84 MHz.
 *  APB2 bus has a 84MHz clock. Internal PLL double this frequency for timer up to 168 MHz.
 *
 *  TIM4 is Connected to APB1 with a maximum frequency up to 84 MHz
 *
 *  timer_tick_frequency = timer_default_frequency / (prescale_factor + 1)
 *  frequency = timer_tick_frequency / (TIM_Period + 1)
 *
 *  Note: The maximum TIM_Period is 65535
 *
 */

	// Enable APB1 clock for TIM2
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	// TIM2 Setting
	// timer_tick_frequency = 84MHz / 168 = 500KHz
	// frequency = 500KHz / 500 = 1KHz
	// Trigger TIM2 Interrupt every 1 millisecond

	TIM_TimeBaseInitTypeDef TIM_Base;
	TIM_Base.TIM_Prescaler = 168 - 1;
    TIM_Base.TIM_Period = 500 - 1;
    TIM_Base.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_Base.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_Base.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM2, &TIM_Base);

    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
    TIM_Cmd(TIM2, ENABLE); // Start Counting
}

void init_PWM_settings(void)
{
	// Enable APB1 clock for TIM4
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	// Enable AHB1 Clock on Port D
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	// Port D Pin 15 as PWM Output
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource15, GPIO_AF_TIM4);
	GPIO_InitTypeDef GPIO_Instruct;
	GPIO_Instruct.GPIO_Pin = GPIO_Pin_15;
	GPIO_Instruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_Instruct.GPIO_OType = GPIO_OType_PP;
	GPIO_Instruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Instruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOD, &GPIO_Instruct);

	// TIM4 Setting
	// timer_tick_frequency = 84MHz / 1 = 84MHz
	// PWM frequency = 84MHz / 8400 = 10KHz
	TIM_TimeBaseInitTypeDef TIM_Base;
	TIM_Base.TIM_Prescaler = 1 - 1;
	TIM_Base.TIM_Period = 8400 - 1;
	TIM_Base.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_Base.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_Base.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM4, &TIM_Base);
	TIM_Cmd(TIM4, ENABLE); // start Timer

	// PWM Setting
	// PWM mode 1 - clear on compare match
	// PWM mode 2 - set on compare match
	TIM_OC.TIM_OCMode = TIM_OCMode_PWM2;
	TIM_OC.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OC.TIM_OCPolarity = TIM_OCPolarity_Low;

	// Duty Cycle Calculations:
	// pulse_length = ((TIM_Period + 1) * Duty_Cycle) / 100 - 1
	// 25 % Duty Cycle: pulse_length = ((8399 + 1) * 25) / 100 - 1 = 2099
	// 50 % Duty Cycle: pulse_length = ((8399 + 1) * 50) / 100 - 1 = 4199
	// 75 % Duty Cycle: pulse_length = ((8399 + 1) * 75) / 100 - 1 = 6299
	// 100% Duty Cycle: pulse_length = ((8399 + 1) *100) / 100 - 1 = 8399

	TIM_OC.TIM_Pulse = pulse_dut_25;
	TIM_OC4Init(TIM4, &TIM_OC);
	TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);
}

void Delay(uint32_t nCount)
{
	while (nCount > 0)
	{
		nCount--;
	}
}
