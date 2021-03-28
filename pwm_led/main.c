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
	/* pc8 -> timer3 channel3
	 * pc9 -> timer3 channel4 */

	/* rcc initialization */
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM3);

	/* gpio initialization */
	LL_GPIO_InitTypeDef pwm_init_struct = {
	 	.Pin = LL_GPIO_PIN_8 | LL_GPIO_PIN_9,
	  	.Mode = LL_GPIO_MODE_ALTERNATE,
		.Speed = LL_GPIO_SPEED_FREQ_HIGH,
		.OutputType = LL_GPIO_OUTPUT_PUSHPULL,
		.Pull = LL_GPIO_PULL_NO,
		.Alternate = LL_GPIO_AF_1
	};

	LL_GPIO_Init(GPIOC, &pwm_init_struct);

	/* pwm initialization */

	/* timer initialization */
	//48MHz / (500 * 960) = 100Hz (pwm frequency)
	LL_TIM_InitTypeDef timer_init_struct = {
		.Prescaler = 500 - 1,
		.CounterMode = LL_TIM_COUNTERMODE_UP,
		.Autoreload = 960,
		.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1
	};
	LL_TIM_Init(TIM3, &timer_init_struct);
	LL_TIM_DisableARRPreload(TIM3);

	/* output channel initialization */
	LL_TIM_OC_EnablePreload(TIM3, LL_TIM_CHANNEL_CH3);
	LL_TIM_OC_EnablePreload(TIM3, LL_TIM_CHANNEL_CH4);
	LL_TIM_OC_InitTypeDef timer_oc_init_struct = {
		.OCMode = LL_TIM_OCMODE_PWM1,
		.OCState = LL_TIM_OCSTATE_DISABLE,
		.OCNState = LL_TIM_OCSTATE_DISABLE,
		.CompareValue = 0,
		.OCPolarity = LL_TIM_OCPOLARITY_HIGH
	};
	LL_TIM_OC_Init(TIM3, LL_TIM_CHANNEL_CH3, &timer_oc_init_struct);
	LL_TIM_OC_Init(TIM3, LL_TIM_CHANNEL_CH4, &timer_oc_init_struct);

	LL_TIM_OC_DisableFast(TIM3, LL_TIM_CHANNEL_CH3);
	LL_TIM_OC_DisableFast(TIM3, LL_TIM_CHANNEL_CH4);
	LL_TIM_SetTriggerOutput(TIM3, LL_TIM_TRGO_RESET);
	LL_TIM_DisableMasterSlaveMode(TIM3);

	LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH3);
	LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH4);
	LL_TIM_EnableCounter(TIM3);
}

#define PWM_INC 1
#define PWM_DEC 0

int main(void)
{
	system_clock_init();
	pwm_init();

	int pwm_state = PWM_INC;
	int pwm_load = 0;

	while(1) {
		/* pwm value increment */
		if(pwm_state == PWM_INC) {
			pwm_load += 5;

			//change pwm state to decrement
			if(pwm_load == 960) {
				pwm_state = PWM_DEC;
			}
		/* pwm value decrement */
		} else {
			pwm_load -= 5;

			//change pwm state to increment
			if(pwm_load == 0) {
				pwm_state = PWM_INC;
			}
		}

		LL_TIM_OC_SetCompareCH3(TIM3, pwm_load);
		LL_TIM_OC_SetCompareCH4(TIM3, pwm_load);

		delay(50000L);
	}
}
