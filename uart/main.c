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
#include "stm32f0xx_ll_usart.h"

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

void uart_init(void)
{
	/* uart1: tx: pa9, rx: pa10 */

	LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_USART1);
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

	LL_GPIO_InitTypeDef gpio_init_struct = {
		.Pin = LL_GPIO_PIN_9,
		.Mode = LL_GPIO_MODE_ALTERNATE,
		.Speed = LL_GPIO_SPEED_FREQ_HIGH,
		.OutputType = LL_GPIO_OUTPUT_PUSHPULL,
		.Pull = LL_GPIO_PULL_NO,
		.Alternate = LL_GPIO_AF_1
	};
	LL_GPIO_Init(GPIOA, &gpio_init_struct);

	LL_USART_InitTypeDef uart_init_struct = { 
		.BaudRate = 9600,
		.DataWidth = LL_USART_DATAWIDTH_8B,
		.StopBits = LL_USART_STOPBITS_1,
		.Parity = LL_USART_PARITY_NONE,
		.TransferDirection = LL_USART_DIRECTION_TX | LL_USART_DIRECTION_RX,
		.HardwareFlowControl = LL_USART_HWCONTROL_NONE,
		/*.OverSampling = LL_USART_OVERSAMPLING_16*/
	};
	LL_USART_Init(USART1, &uart_init_struct);

	LL_USART_DisableIT_CTS(USART1);
	LL_USART_ConfigAsyncMode(USART1);
	LL_USART_Enable(USART1);
}

void uart_putc(char data)
{
	while(LL_USART_IsActiveFlag_TXE(USART1) == 0);
	LL_USART_TransmitData8(USART1, (uint8_t)data);
	while(LL_USART_IsActiveFlag_TC(USART1) == 0);
}

void uart_puts(char *string)
{
	for(; *string != '\0'; string++) {
		uart_putc(*string);
	}
}


int main(void)
{
	system_clock_init();
	gpio_init();
	uart_init();

	int led_on = 0;

	uart_puts("hello world!\n\r");\

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
