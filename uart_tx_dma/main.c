#include <string.h>
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

void uart_init(void)
{
	/* uart1: tx: pa9, rx: pa10 */

	/* rcc initialization */
	LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_USART1);
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

	/* gpio initialization */
	LL_GPIO_InitTypeDef gpio_init_struct = {
		.Pin = LL_GPIO_PIN_9 | LL_GPIO_PIN_10,
		.Mode = LL_GPIO_MODE_ALTERNATE,
		.Speed = LL_GPIO_SPEED_FREQ_HIGH,
		.OutputType = LL_GPIO_OUTPUT_PUSHPULL,
		.Pull = LL_GPIO_PULL_NO,
		.Alternate = LL_GPIO_AF_1
	};
	LL_GPIO_Init(GPIOA, &gpio_init_struct);

	/* dma initialization */
	LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_2, LL_DMA_MDATAALIGN_BYTE);
	LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_2, LL_DMA_MEMORY_INCREMENT);
	LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_2, (uint32_t)&USART1->TDR);
	LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_2, LL_DMA_PDATAALIGN_BYTE);
	LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_2, LL_DMA_PERIPH_NOINCREMENT);
	LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_2, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
	LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_2, LL_DMA_PRIORITY_HIGH);
	LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_2, LL_DMA_MODE_NORMAL);
	LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_2);

	/* uart initialization */
	LL_USART_InitTypeDef uart_init_struct = { 
		.BaudRate = 9600,
		.DataWidth = LL_USART_DATAWIDTH_8B,
		.StopBits = LL_USART_STOPBITS_1,
		.Parity = LL_USART_PARITY_NONE,
		.TransferDirection = LL_USART_DIRECTION_TX | LL_USART_DIRECTION_RX,
		.HardwareFlowControl = LL_USART_HWCONTROL_NONE,
	};
	LL_USART_Init(USART1, &uart_init_struct);

	LL_USART_EnableDMAReq_TX(USART1);
	LL_USART_DisableIT_CTS(USART1);
	LL_USART_ConfigAsyncMode(USART1);
	LL_USART_Enable(USART1);
}

void uart_dma_puts(char *s)
{
	/* setup address and size of data to send */
	LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_2, strlen(s));
	LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_2, (uint32_t)s);

	/* clear all flags */
	LL_DMA_ClearFlag_GI2(DMA1);
	LL_DMA_ClearFlag_TC2(DMA1);
	LL_DMA_ClearFlag_HT2(DMA1);
	LL_DMA_ClearFlag_TE2(DMA1);

	/* enable dma to move data from memory to uart data register */
        LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_2);

	/* wait until transfer complete flag is set */
	while(!LL_DMA_IsActiveFlag_TC2(DMA1));

	/* important, diable dma when data finished transfering */
	LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_2);
}

int main(void)
{
	system_clock_init();
	led_init();
	uart_init();

	while(1) {
		uart_dma_puts("hello world!\n\r");
		LL_GPIO_TogglePin(GPIOC, LL_GPIO_PIN_8 | LL_GPIO_PIN_9);
		delay(1000000L);
	}
}
