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

void led_init(void)
{
	/* rcc initialization */
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);

	LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_8 | LL_GPIO_PIN_9);

	/* gpio initialization */
	LL_GPIO_InitTypeDef gpio_init_struct = {
	 	.Pin = LL_GPIO_PIN_8 | LL_GPIO_PIN_9,
	  	.Mode = LL_GPIO_MODE_OUTPUT,
		.Speed = LL_GPIO_SPEED_FREQ_HIGH,
		.OutputType = LL_GPIO_OUTPUT_PUSHPULL,
		.Pull = LL_GPIO_PULL_NO
	};

	LL_GPIO_Init(GPIOC, &gpio_init_struct);
}

void button_init(void)
{
	/* rcc initialization */
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);

	/* gpio initialization */
	LL_GPIO_InitTypeDef gpio_init_struct = {
		.Pin = LL_GPIO_PIN_0,
		.Mode = LL_GPIO_MODE_INPUT,
		.Pull = LL_GPIO_PULL_NO
	};

	LL_GPIO_Init(GPIOA, &gpio_init_struct);

	/* exti initialization */
	LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTA, LL_SYSCFG_EXTI_LINE0);

	LL_EXTI_InitTypeDef exti_init_struct = {
		.Line_0_31 = LL_EXTI_LINE_0,
		.LineCommand = ENABLE,
		.Mode = LL_EXTI_MODE_IT,
		.Trigger = LL_EXTI_TRIGGER_RISING_FALLING
	};
	LL_EXTI_Init(&exti_init_struct);

	NVIC_SetPriority(EXTI0_1_IRQn, 0);
	NVIC_EnableIRQ(EXTI0_1_IRQn);
}

void EXTI0_1_IRQHandler(void)
{
	/* pa0 */
	if(LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_0) == 1) {
		if(LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_0)) {
			LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_8 | LL_GPIO_PIN_9);
		} else {
			LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_8 | LL_GPIO_PIN_9);
		}
		LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_0);
	}
}

int main(void)
{
	system_clock_init();
	led_init();
	button_init();

	while(1);
}
