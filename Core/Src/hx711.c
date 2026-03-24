#include "hx711.h"

// ~1 µs delay for clock timing (adjust if your HCLK differs from 80 MHz)
static inline void _delay_us(uint32_t us) {
    uint32_t cycles = (SystemCoreClock / 1000000) * us / 4;
    while (cycles--) __NOP();
}

void hx711_init(HX711 *hx,
                GPIO_TypeDef *clk_port, uint16_t clk_pin,
                GPIO_TypeDef *dat_port, uint16_t dat_pin,
                HX711_Gain gain)
{
    hx->clk_port = clk_port;
    hx->clk_pin  = clk_pin;
    hx->dat_port = dat_port;
    hx->dat_pin  = dat_pin;
    hx->gain     = gain;
    hx->offset   = 0;
    hx->scale    = 1.0f;

    // CLK starts low
    HAL_GPIO_WritePin(hx->clk_port, hx->clk_pin, GPIO_PIN_RESET);

    // Run one dummy read to set the gain register on the chip
    hx711_read_raw(hx);
}

uint8_t hx711_is_ready(HX711 *hx) {
    // DOUT low means data is ready
    return HAL_GPIO_ReadPin(hx->dat_port, hx->dat_pin) == GPIO_PIN_RESET;
}

int32_t hx711_read_raw(HX711 *hx) {
    // Wait until DOUT goes low (data ready), with a simple timeout
    uint32_t timeout = 1000000;
    while (!hx711_is_ready(hx)) {
        if (--timeout == 0) return 0;  // return 0 on timeout
    }

    int32_t data = 0;

    // Clock in 24 bits, MSB first
    for (int i = 0; i < 24; i++) {
        HAL_GPIO_WritePin(hx->clk_port, hx->clk_pin, GPIO_PIN_SET);
        _delay_us(1);
        data = (data << 1) | (HAL_GPIO_ReadPin(hx->dat_port, hx->dat_pin) == GPIO_PIN_SET ? 1 : 0);
        HAL_GPIO_WritePin(hx->clk_port, hx->clk_pin, GPIO_PIN_RESET);
        _delay_us(1);
    }

    // Extra pulses to set gain/channel for NEXT read
    for (int i = 0; i < (int)hx->gain; i++) {
        HAL_GPIO_WritePin(hx->clk_port, hx->clk_pin, GPIO_PIN_SET);
        _delay_us(1);
        HAL_GPIO_WritePin(hx->clk_port, hx->clk_pin, GPIO_PIN_RESET);
        _delay_us(1);
    }

    // HX711 output is 2's complement; sign-extend from 24-bit to 32-bit
    if (data & 0x800000) {
        data |= 0xFF000000;
    }

    return data;
}

int32_t hx711_read_average(HX711 *hx, uint8_t times) {
    int64_t sum = 0;
    for (uint8_t i = 0; i < times; i++) {
        sum += hx711_read_raw(hx);
    }
    return (int32_t)(sum / times);
}

void hx711_tare(HX711 *hx, uint8_t times) {
    hx->offset = hx711_read_average(hx, times);
}

float hx711_get_units(HX711 *hx, uint8_t times) {
    int32_t raw = hx711_read_average(hx, times);
    return (float)(raw - hx->offset) / hx->scale;
}

void hx711_set_scale(HX711 *hx, float scale) {
    hx->scale = scale;
}

void hx711_power_down(HX711 *hx) {
    HAL_GPIO_WritePin(hx->clk_port, hx->clk_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(hx->clk_port, hx->clk_pin, GPIO_PIN_SET);
    HAL_Delay(1);  // Hold CLK high >60 µs to power down
}

void hx711_power_up(HX711 *hx) {
    HAL_GPIO_WritePin(hx->clk_port, hx->clk_pin, GPIO_PIN_RESET);
    HAL_Delay(1);  // Allow chip to reset (~400 ms for first reading)
}
