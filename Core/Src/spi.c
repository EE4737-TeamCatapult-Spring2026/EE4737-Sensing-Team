#include "spi.h"
#include "stm32f3xx_hal_spi.h"
#include "stm32f3xx_hal_spi_ex.h"

/* Declare the handle at file scope so HTS221_Init() can reach it */
SPI_HandleTypeDef hspi1;

/**
 * HAL calls this automatically from inside HAL_SPI_Init().
 * It handles clocks, JTAG release, and GPIO alternate functions.
 */
void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance != SPI1) return;

    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_SPI1_CLK_ENABLE();

    /* PB3 (SCK) and PB5 (MOSI/SDA) — assigning AF5 releases them
       from their JTAG reset-state function automatically on the F3 */
    gpio.Pin       = GPIO_PIN_3 | GPIO_PIN_5;
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_NOPULL;
    gpio.Speed     = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOB, &gpio);

    /* PA4 — CS — plain output, idle high */
    gpio.Pin       = GPIO_PIN_4;
    gpio.Mode      = GPIO_MODE_OUTPUT_PP;
    gpio.Pull      = GPIO_NOPULL;
    gpio.Speed     = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = 0;
    HAL_GPIO_Init(GPIOA, &gpio);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
}

void MX_SPI1_Init(void)
{
    hspi1.Instance               = SPI1;
    hspi1.Init.Mode              = SPI_MODE_MASTER;
    hspi1.Init.Direction         = SPI_DIRECTION_1LINE;  /* 3-wire half-duplex */
    hspi1.Init.DataSize          = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity       = SPI_POLARITY_HIGH;     /* CPOL = 1           */
    hspi1.Init.CLKPhase          = SPI_PHASE_2EDGE;      /* CPHA = 1  → Mode 3 */
    hspi1.Init.NSS               = SPI_NSS_SOFT;         /* CS driven manually */
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8; /* 72/8 = 9 MHz    */
    hspi1.Init.FirstBit          = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode            = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.CRCPolynomial     = 7;

    if (HAL_SPI_Init(&hspi1) != HAL_OK) {
    	Error_Handler(); // Gives warning, but should know this method. If a problem, change to while(1)
    }
}
