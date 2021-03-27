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
	LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
	LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
	LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

	/* enable system clock and wait utill ready */
	while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL);
	LL_Init1msTick(48000000);
	LL_SetSystemCoreClock(48000000);
}

void delay(volatile uint32_t count)
{
	while(count--);
}

void gpio_init(void)
{
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);

	LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_8 | LL_GPIO_PIN_9);

	LL_GPIO_InitTypeDef gpio_init_struct = {
	 	.Pin = LL_GPIO_PIN_8 | LL_GPIO_PIN_9,
	  	.Mode = LL_GPIO_MODE_OUTPUT,
		.Speed = LL_GPIO_SPEED_FREQ_HIGH,
		.OutputType = LL_GPIO_OUTPUT_PUSHPULL,
		.Pull = LL_GPIO_PULL_NO
	};

	LL_GPIO_Init(GPIOC, &gpio_init_struct);
}

int main(void)
{
	system_clock_init();
	gpio_init();

	int led_on = 0;

	while(1) {
		if(led_on) {
			LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_8 | LL_GPIO_PIN_9);
		} else {
			LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_8 |LL_GPIO_PIN_9);
		}

		delay(1000000L);

		led_on = (led_on + 1) % 2;
	}
}
