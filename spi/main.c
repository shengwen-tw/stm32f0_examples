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

volatile uint8_t recvd_spi_data;

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
		.DataWidth = LL_SPI_DATAWIDTH_8BIT,
		.ClockPolarity = LL_SPI_POLARITY_LOW,
		.ClockPhase = LL_SPI_PHASE_1EDGE,
		.NSS = LL_SPI_NSS_SOFT,
		.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV64, //0.75Mbits/s
		.BitOrder = LL_SPI_MSB_FIRST,
		.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE,
		.CRCPoly = 7
	};
	LL_SPI_Init(SPI1, &spi_init_struct);
	LL_SPI_SetStandard(SPI1, LL_SPI_PROTOCOL_MOTOROLA);
	LL_SPI_EnableNSSPulseMgt(SPI1);
	LL_SPI_Enable(SPI1);
}

/* pooling mode spi write */
void spi_write(SPI_TypeDef *spi_channel, uint8_t data)
{
	while(LL_SPI_IsActiveFlag_TXE(spi_channel) == 0);
	LL_SPI_TransmitData8(spi_channel, data);
}

/* pooling mode spi read */
uint8_t spi_read(SPI_TypeDef *spi_channel)
{
	while(LL_SPI_IsActiveFlag_RXNE(spi_channel) == 0);
	return LL_SPI_ReceiveData8(spi_channel);
}

int main(void)
{
	system_clock_init();
	spi1_init();

	/* observe spi1 using a logic analyzer with the following settings:
	 * sample frequency: > 2Mhz 
	 * CS (chip selection): none
	 * CPOL = 0 
	 * CPHA = 0
	 * 8bits mode
	 * MSB first */

	char *s = "hello\n\r";
	int s_size = strlen(s);
	int send_index = 0;

	while(1) {
		/* spi1 send string */
		spi_write(SPI1, s[send_index]);
		if(send_index == (s_size - 1)) {
			send_index = 0;
		} else {
			send_index++;
		}

		delay(50000L);
	}
}
