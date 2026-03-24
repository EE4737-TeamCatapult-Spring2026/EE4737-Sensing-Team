#ifndef HX711_H
#define HX711_H

#include "stm32f3xx_hal.h"
#include <stdint.h>

// Channel & gain selection (number of extra clock pulses after 24 data bits)
typedef enum {
    HX711_GAIN_A_128 = 1,  // Channel A, Gain 128 (most common for strain gauges)
    HX711_GAIN_B_32  = 2,  // Channel B, Gain 32
    HX711_GAIN_A_64  = 3,  // Channel A, Gain 64
} HX711_Gain;

typedef struct {
    GPIO_TypeDef *clk_port;
    uint16_t      clk_pin;
    GPIO_TypeDef *dat_port;
    uint16_t      dat_pin;
    HX711_Gain    gain;
    int32_t       offset;   // tare offset
    float         scale;    // units per raw count
} HX711;

void    hx711_init(HX711 *hx, GPIO_TypeDef *clk_port, uint16_t clk_pin,
                               GPIO_TypeDef *dat_port, uint16_t dat_pin,
                               HX711_Gain gain);
uint8_t hx711_is_ready(HX711 *hx);
int32_t hx711_read_raw(HX711 *hx);
int32_t hx711_read_average(HX711 *hx, uint8_t times);
void    hx711_tare(HX711 *hx, uint8_t times);
float   hx711_get_units(HX711 *hx, uint8_t times);
void    hx711_set_scale(HX711 *hx, float scale);
void    hx711_power_down(HX711 *hx);
void    hx711_power_up(HX711 *hx);

#endif // HX711_H
