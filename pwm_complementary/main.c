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

void pwm_init(void)
{
	/* pa8  -> tim1 ch1
	 * pb13 -> tim1 ch1n */

	/* rcc initialization */
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
	LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_TIM1);

	/* gpio initialization */
	LL_GPIO_InitTypeDef pwm_init_struct = {
	 	.Pin = LL_GPIO_PIN_8,
	  	.Mode = LL_GPIO_MODE_ALTERNATE,
		.Speed = LL_GPIO_SPEED_FREQ_HIGH,
		.OutputType = LL_GPIO_OUTPUT_PUSHPULL,
		.Pull = LL_GPIO_PULL_NO,
		.Alternate = LL_GPIO_AF_2
	};
	LL_GPIO_Init(GPIOA, &pwm_init_struct);

	pwm_init_struct.Pin = LL_GPIO_PIN_13;
	LL_GPIO_Init(GPIOB, &pwm_init_struct);

	/* timer initialization */
	//48MHz / (1 * 1600) = 30KHz (pwm frequency)
	LL_TIM_InitTypeDef timer_init_struct = {
		.Prescaler = 1 - 1,
		.CounterMode = LL_TIM_COUNTERMODE_UP,
		.Autoreload = 1600 - 1,
		.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1
	};
	LL_TIM_Init(TIM1, &timer_init_struct);

	LL_TIM_DisableARRPreload(TIM1);
	LL_TIM_DisableMasterSlaveMode(TIM1);

	LL_TIM_SetTriggerOutput(TIM1, LL_TIM_TRGO_RESET);

	/* timer oc initialization */
	LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH1);

	LL_TIM_OC_InitTypeDef timer_oc_init_struct = {
		.OCMode = LL_TIM_OCMODE_PWM1,
		.OCState = LL_TIM_OCSTATE_DISABLE,
		.OCNState = LL_TIM_OCSTATE_DISABLE,
		.CompareValue = 1200, //duty = 1200/1600 = 75%
		.OCPolarity = LL_TIM_OCPOLARITY_HIGH
	};
	LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH1, &timer_oc_init_struct);
	LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH1N, &timer_oc_init_struct);

	LL_TIM_OC_DisableFast(TIM1, LL_TIM_CHANNEL_CH1);
	LL_TIM_OC_DisableFast(TIM1, LL_TIM_CHANNEL_CH1N);

	LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1);
	LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1N);

	/* enable timer counter */
	LL_TIM_EnableCounter(TIM1);

	/* advanced timers (e.g. TIM1 and TIM8) requires the user to enable the
         * output manually */
	LL_TIM_EnableAllOutputs(TIM1);
}

int main(void)
{
	system_clock_init();
	pwm_init();

	while(1);
}
