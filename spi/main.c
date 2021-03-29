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
#include "stm32f0xx_ll_spi.h"

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

void uart_init(void)
{
	/* uart1: tx: pa9, rx: pa10 */

	/* rcc initialization */
	LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_USART1);
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);

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

	LL_USART_DisableIT_CTS(USART1);
	LL_USART_ConfigAsyncMode(USART1);
	LL_USART_Enable(USART1);
}

char uart_getc(void)
{
	while(LL_USART_IsActiveFlag_RXNE(USART1) == 0);
	return LL_USART_ReceiveData8(USART1);
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

void spi1_init(void)
{
	/* pa5 -> spi1_sck
	 * pa6 -> spi1_miso
	 * pa7 -> spi1_mosi */

	/* rcc initialization */
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
	LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_SPI1);

	/* gpio initialization */
	LL_GPIO_InitTypeDef gpio_init_struct = {
		.Pin = LL_GPIO_PIN_5 | LL_GPIO_PIN_6 | LL_GPIO_PIN_7,
		.Mode = LL_GPIO_MODE_ALTERNATE,
		.Speed = LL_GPIO_SPEED_FREQ_HIGH,
		.OutputType = LL_GPIO_OUTPUT_PUSHPULL,
		.Pull = LL_GPIO_PULL_NO,
		.Alternate = LL_GPIO_AF_0
	};
	LL_GPIO_Init(GPIOA, &gpio_init_struct);

	/* spi initialization */
	LL_SPI_InitTypeDef spi_init_struct = {
		.TransferDirection = LL_SPI_FULL_DUPLEX,
		.Mode = LL_SPI_MODE_MASTER,
		.DataWidth = LL_SPI_DATAWIDTH_4BIT,
		.ClockPolarity = LL_SPI_POLARITY_LOW,
		.ClockPhase = LL_SPI_PHASE_1EDGE,
		.NSS = LL_SPI_NSS_SOFT,
		.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV4, //12Mbits/s
		.BitOrder = LL_SPI_MSB_FIRST,
		.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE,
		.CRCPoly = 7
	};
	LL_SPI_Init(SPI1, &spi_init_struct);
	LL_SPI_SetStandard(SPI1, LL_SPI_PROTOCOL_MOTOROLA);
	LL_SPI_EnableNSSPulseMgt(SPI1);
}

void spi2_init(void)
{
	/* pb13 -> spi2_sck
         * pb14 -> spi2_miso
         * pb15 -> spi2_mosi */

	/* rcc initialization */
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_SPI2);

	/* gpio initialization */
	LL_GPIO_InitTypeDef gpio_init_struct = {
		.Pin = LL_GPIO_PIN_13 | LL_GPIO_PIN_14 | LL_GPIO_PIN_15,
		.Mode = LL_GPIO_MODE_ALTERNATE,
		.Speed = LL_GPIO_SPEED_FREQ_HIGH,
		.OutputType = LL_GPIO_OUTPUT_PUSHPULL,
		.Pull = LL_GPIO_PULL_NO,
		.Alternate = LL_GPIO_AF_0
	};
	LL_GPIO_Init(GPIOB, &gpio_init_struct);

	LL_SPI_InitTypeDef spi_init_struct = {
		.TransferDirection = LL_SPI_FULL_DUPLEX,
		.Mode = LL_SPI_MODE_MASTER,
		.DataWidth = LL_SPI_DATAWIDTH_4BIT,
		.ClockPolarity = LL_SPI_POLARITY_LOW,
		.ClockPhase = LL_SPI_PHASE_1EDGE,
		.NSS = LL_SPI_NSS_SOFT,
		.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV4, //12Mbits/s
		.BitOrder = LL_SPI_MSB_FIRST,
		.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE,
		.CRCPoly = 7
	};
	LL_SPI_Init(SPI2, &spi_init_struct);
	LL_SPI_SetStandard(SPI2, LL_SPI_PROTOCOL_MOTOROLA);
	LL_SPI_EnableNSSPulseMgt(SPI2);
}

uint8_t spi_read_write(SPI_TypeDef *spi_channel, uint8_t data)
{
	while(LL_SPI_IsActiveFlag_TXE(spi_channel) == 0);
	LL_SPI_TransmitData8(spi_channel, data);

	while(LL_SPI_IsActiveFlag_RXNE(spi_channel) == 0);
	return LL_SPI_ReceiveData8(spi_channel);
}

int main(void)
{
	system_clock_init();
	uart_init();

	spi1_init();
	spi2_init();

	uart_puts("hello world!\n\r");\

	/* connect spi1 to spi2 as following:
	 * pa5 <-> pb13
	 * pa6 <-> pb15
	 * pa7 <-> pb14 */

	/* observe the spi communication with uart1 */

	char *s = "hello spi\n\r";
	int s_size = strlen(s);
	int send_index = 0;

	while(1) {
		/* spi1 send string */
		spi_read_write(SPI1, s[send_index]);
		if(send_index == (s_size - 1)) {
			send_index = 0;
		} else {
			send_index++;
		}
		delay(50000L);

#if 0
		/* spi2 receive string */
		uint8_t recvd_data = spi_read_write(SPI2, 0x00);

		/* if spi2 received non-zero char then print it via uart1 */
		if(recvd_data != 0x00) {
			uart_putc(recvd_data);
		}
#endif
	}
}
