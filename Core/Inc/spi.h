#ifndef SPI_H
#define SPI_H

#include "stm32f3xx_hal.h"
#include "stm32f3xx_hal_def.h"
#include "stm32f3xx_hal_spi.h"
#include "stm32f3xx_hal_spi_ex.h"

extern SPI_HandleTypeDef hspi1;

/**
 * HAL calls this automatically from inside HAL_SPI_Init().
 * It handles clocks, JTAG release, and GPIO alternate functions.
 */
void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi);
void MX_SPI1_Init(void);

#endif /* SPI_H */
