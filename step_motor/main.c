#include "stm32f0xx_ll_crs.h"
#include "stm32f0xx_ll_rcc.h"
#include "stm32f0xx_ll_bus.h"
#include "stm32f0xx_ll_system.h"
#include "stm32f0xx_ll_exti.h"
#include "stm32f0xx_ll_cortex.h"
#include "stm32f0xx_ll_utils.h"
#include "stm32f0xx_ll_pwr.h"
#include "stm32f0xx_ll_dma.h"
#include "stm32f0xx_ll_gpio.h"
#include "stm32f0xx_ll_tim.h"

void system_clock_init(void)
{
	/* power on */
	LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_SYSCFG);
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

	/* setup flash */
	LL_FLASH_SetLatency(LL_FLASH_LATENCY_1);
	while(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_1);

	/* set system clock by 48Mhz with HSI source and PLL circuit */

	/* enable HSI and wait untill ready */
	while(LL_RCC_HSI_IsReady() != 1);
	LL_RCC_HSI_SetCalibTrimming(16);
	LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI_DIV_2, LL_RCC_PLL_MUL_12);
	LL_RCC_PLL_Enable();

	/* enable PLL and wait untill ready */
	LL_RCC_PLL_Enable();
	while(LL_RCC_PLL_IsReady() != 1);

	/* enable system clock and wait utill ready */
	LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
	LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
	LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
	while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL);
	LL_SetSystemCoreClock(48000000); //HCLK = 48Mhz
}

void delay(volatile uint32_t count)
{
	while(count--);
}

void gpio_init(void)
{
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);

	/* gpio initialization */
	LL_GPIO_InitTypeDef gpio_init_struct = {
	 	.Pin = LL_GPIO_PIN_13,
	  	.Mode = LL_GPIO_MODE_OUTPUT,
		.Speed = LL_GPIO_SPEED_FREQ_HIGH,
		.OutputType = LL_GPIO_OUTPUT_PUSHPULL,
		.Pull = LL_GPIO_PULL_NO
	};

	LL_GPIO_Init(GPIOB, &gpio_init_struct);

	LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_13);
}

void pwm_timer1_init(void)
{
	/* pa8  -> tim1 ch1 (pwm) */

	/* rcc initialization */
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
	LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_TIM1);

	/* gpio initialization */
	LL_GPIO_InitTypeDef timer1_init_struct = {
	 	.Pin = LL_GPIO_PIN_8,
	  	.Mode = LL_GPIO_MODE_ALTERNATE,
		.Speed = LL_GPIO_SPEED_FREQ_HIGH,
		.OutputType = LL_GPIO_OUTPUT_PUSHPULL,
		.Pull = LL_GPIO_PULL_NO,
		.Alternate = LL_GPIO_AF_2
	};
	LL_GPIO_Init(GPIOA, &timer1_init_struct);

	/* timer initialization */
	//48MHz / (1 * 1600) = 30KHz (pwm frequency)
	LL_TIM_InitTypeDef timer_init_struct = {
		.Prescaler = 1 - 1,
		.CounterMode = LL_TIM_COUNTERMODE_CENTER_UP,
		.Autoreload = 1600 - 1,
		.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1
	};
	LL_TIM_Init(TIM1, &timer_init_struct);

	LL_TIM_DisableARRPreload(TIM1);

	/* set timer 1 as master of the timer 2 */
	LL_TIM_SetTriggerOutput(TIM1, LL_TIM_TRGO_OC1REF);
	LL_TIM_EnableMasterSlaveMode(TIM1);

	/* timer oc initialization */
	LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH1);

	LL_TIM_OC_InitTypeDef timer_oc_init_struct = {
		.OCMode = LL_TIM_OCMODE_PWM1,
		.OCState = LL_TIM_OCSTATE_DISABLE,
		.OCNState = LL_TIM_OCSTATE_DISABLE,
		.CompareValue = 800,
		.OCPolarity = LL_TIM_OCPOLARITY_HIGH
	};
	LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH1, &timer_oc_init_struct);

	LL_TIM_OC_DisableFast(TIM1, LL_TIM_CHANNEL_CH1);
	LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1);

	/* enable timer counter */
	LL_TIM_EnableCounter(TIM1);

	/* advanced timers (e.g. TIM1) requires the user to enable the output manually */
	LL_TIM_EnableAllOutputs(TIM1);
}

void timer2_init(void)
{
	/*====================*
	 * rcc initialization *
	 *====================*/
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);

	/*=====================*
	 * nvic initialization *
	 *=====================*/
	NVIC_SetPriority(TIM2_IRQn, 0);
	NVIC_EnableIRQ(TIM2_IRQn);

	/*======================*
	 * timer initialization *
	 *======================*/

	LL_TIM_InitTypeDef timer_init_struct = {
		.Prescaler = 0,
		.Autoreload = 1,
		.CounterMode = LL_TIM_COUNTERMODE_UP,
		.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1,
		.RepetitionCounter = 0
	};
	LL_TIM_Init(TIM2, &timer_init_struct);

	LL_TIM_DisableARRPreload(TIM2);

	/* enable slave mode and set input trigger from timer1 */
	LL_TIM_SetTriggerInput(TIM2, LL_TIM_TS_ITR0);
	LL_TIM_SetSlaveMode(TIM2, LL_TIM_SLAVEMODE_RESET);
	LL_TIM_DisableIT_TRIG(TIM2);
	LL_TIM_DisableDMAReq_TRIG(TIM2);
	LL_TIM_DisableMasterSlaveMode(TIM2);
	LL_TIM_SetTriggerOutput(TIM2, LL_TIM_TRGO_UPDATE);
	LL_TIM_SetClockSource(TIM2, LL_TIM_CLOCKSOURCE_EXT_MODE1);

	/*======================*
	 * activate the timer 2 *
	 *======================*/
	LL_TIM_EnableIT_UPDATE(TIM2);
	LL_TIM_EnableCounter(TIM2);
}

void TIM2_IRQHandler(void)
{
	if(LL_TIM_IsActiveFlag_UPDATE(TIM2) == 1) {
		LL_TIM_DisableCounter(TIM1);
		LL_TIM_DisableCounter(TIM2);
		LL_TIM_DisableIT_UPDATE(TIM2);

		LL_TIM_ClearFlag_UPDATE(TIM2);
	}
}

void pulse_output(uint32_t pulse_number)
{
	LL_TIM_SetAutoReload(TIM2, pulse_number);
	LL_TIM_ClearFlag_UPDATE(TIM2);
	LL_TIM_EnableIT_UPDATE(TIM2);
	LL_TIM_EnableCounter(TIM2);

	LL_TIM_SetAutoReload(TIM1, 1600);
	LL_TIM_OC_SetCompareCH1(TIM1, 800);
	LL_TIM_EnableCounter(TIM1);
}

int main(void)
{
	system_clock_init();
	pwm_timer1_init(); //connect pa8 to the step motor
	timer2_init();

	int pulse_number = 3; //desired pulse number for controlling the step motor

	while(1) {
		pulse_output(pulse_number);
		delay(10000);
	}
}
