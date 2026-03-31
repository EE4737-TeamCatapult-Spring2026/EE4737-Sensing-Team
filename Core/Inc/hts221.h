/*!
 * @file hts221.h
 *
 * C driver for the HTS221 Humidity and Temperature Sensor
 * Targeting STM32 Nucleo F303K8 via STM32 HAL — 3-wire (half-duplex) SPI
 *
 * Adapted from Adafruit_HTS221 by Bryan Siepert / Adafruit Industries.
 * BSD license.
 *
 * ── 3-wire SPI wiring ────────────────────────────────────────────────────────
 *
 *  HTS221 pin │ Nucleo F303K8 pin  │ Notes
 *  ───────────┼───────────────────┼────────────────────────────────────────────
 *  VDD        │ 3.3 V             │
 *  GND        │ GND               │
 *  CS         │ any GPIO (output) │ Define HTS221_CS_PORT / HTS221_CS_PIN below
 *  SCL/SCK    │ SPI SCK           │ e.g. PA5 on SPI1
 *  SDA/SDO    │ SPI MOSI          │ Single bidirectional data line — connect
 *             │                   │ to MOSI only; leave MISO unconnected.
 *             │                   │ The STM32 SPI peripheral must be configured
 *             │                   │ for 1-line (half-duplex) mode in CubeMX:
 *             │                   │   Direction = "1-line bidirectional"
 *
 * ── CubeMX SPI settings ──────────────────────────────────────────────────────
 *  Mode                 : Half-Duplex Master
 *  Hardware NSS         : Disabled  (CS is driven manually as a GPIO)
 *  Data Size            : 8 bits
 *  First Bit            : MSB first
 *  CPOL                 : Low   (CPOL = 0)
 *  CPHA                 : 1 Edge (CPHA = 0)  →  SPI Mode 0
 *  Baud rate prescaler  : choose so clock ≤ 10 MHz
 *
 * ── SPI frame format (HTS221 datasheet §6.2) ─────────────────────────────────
 *  The 8-bit address byte sent at the start of every transaction carries:
 *    Bit 7  RW  — 1 = read, 0 = write
 *    Bit 6  MS  — 1 = auto-increment register address (required for
 *                    multi-byte transfers; uses 0x40, NOT 0x80 as in I2C)
 *    Bits 5-0   — register address
 */

#ifndef HTS221_H
#define HTS221_H

#include "stm32f3xx_hal.h"
#include "spi.h"
#include <stdint.h>
#include <stdbool.h>

/* ── CS pin — edit to match your PCB ──────────────────────────────────────── */
#ifndef HTS221_CS_PORT
  #define HTS221_CS_PORT   GPIOA
#endif
#ifndef HTS221_CS_PIN
  #define HTS221_CS_PIN    GPIO_PIN_4
#endif

/* ── SPI timeout ──────────────────────────────────────────────────────────── */
#define HTS221_SPI_TIMEOUT_MS  100U

/* ── Register addresses ───────────────────────────────────────────────────── */
#define HTS221_CHIP_ID           0xBC   /**< Expected WHO_AM_I value          */

#define HTS221_REG_WHOAMI        0x0F
#define HTS221_REG_CTRL1         0x20   /**< PD | BDU | ODR[1:0]             */
#define HTS221_REG_CTRL2         0x21   /**< BOOT | Heater | ONE_SHOT         */
#define HTS221_REG_CTRL3         0x22   /**< DRDY_H_L | PP_OD | DRDY_EN      */
#define HTS221_REG_HUMIDITY_OUT  0x28   /**< Humidity output LSB              */
#define HTS221_REG_TEMP_OUT_L    0x2A   /**< Temperature output LSB           */
#define HTS221_REG_H0_RH_X2      0x30
#define HTS221_REG_H1_RH_X2      0x31
#define HTS221_REG_T0_DEGC_X8    0x32
#define HTS221_REG_T1_T0_MSB     0x35
#define HTS221_REG_H0_T0_OUT     0x36   /**< 16-bit signed, LSB first         */
#define HTS221_REG_H1_T0_OUT     0x3A   /**< 16-bit signed, LSB first         */
#define HTS221_REG_T0_OUT        0x3C   /**< 16-bit signed, LSB first         */
#define HTS221_REG_T1_OUT        0x3E   /**< 16-bit signed, LSB first         */

/*
 * Address byte flag bits for SPI transactions:
 *
 *   HTS221_SPI_READ   Set bit 7 → tells the sensor this is a read cycle.
 *   HTS221_SPI_MULTI  Set bit 6 → enables register auto-increment so that
 *                     consecutive bytes in one transaction map to consecutive
 *                     registers.  On SPI this is bit 6 (0x40); I2C uses 0x80.
 */
#define HTS221_SPI_READ    0x80u
#define HTS221_SPI_MULTI   0x40u

/* ── Output data rate ─────────────────────────────────────────────────────── */
typedef enum {
    HTS221_RATE_ONE_SHOT = 0,
    HTS221_RATE_1_HZ     = 1,
    HTS221_RATE_7_HZ     = 2,
    HTS221_RATE_12_5_HZ  = 3,
} hts221_rate_t;

/* ── Driver state ─────────────────────────────────────────────────────────── */
/**
 * @brief  All state required by the driver.
 *
 * Declare one instance (typically as a global) and pass its address to every
 * API function.  Set hspi before calling HTS221_Init().
 */
typedef struct {
    SPI_HandleTypeDef *hspi;  /**< HAL SPI handle — must be 1-line half-duplex */

    /* Calibration constants loaded from OTP during HTS221_Init()             */
    uint16_t T0, T1;          /**< Temperature cal points in °C (after >>3)   */
    int16_t  T0_OUT, T1_OUT;  /**< Raw ADC counts at T0 and T1                */
    uint8_t  H0, H1;          /**< Humidity cal points ×2 (divide by 2 to use)*/
    int16_t  H0_T0_OUT;       /**< Raw ADC counts at H0                       */
    int16_t  H1_T0_OUT;       /**< Raw ADC counts at H1                       */

    /* Most recent sensor readings — updated by every call to HTS221_Read()   */
    float temperature;        /**< Temperature in degrees Celsius              */
    float humidity;           /**< Relative humidity in percent                */
} HTS221_HandleTypeDef;

/* ── Public API ───────────────────────────────────────────────────────────── */

/**
 * @brief  Initialise the HTS221 over 3-wire half-duplex SPI.
 *
 * Verifies WHO_AM_I, issues a BOOT cycle, enables continuous measurement at
 * 12.5 Hz, and reads all OTP calibration coefficients into @p dev.
 *
 * @param  dev   Caller-allocated driver state struct.
 * @param  hspi  HAL SPI handle configured for 1-line half-duplex master mode.
 * @retval true  Sensor identified and ready.
 * @retval false WHO_AM_I mismatch or SPI communication failure.
 */
bool HTS221_Init(HTS221_HandleTypeDef *dev, SPI_HandleTypeDef *hspi);

/**
 * @brief  Reload all trimming values from internal flash (sets BOOT bit).
 * @param  dev  Driver handle.
 */
void HTS221_Boot(HTS221_HandleTypeDef *dev);

/**
 * @brief  Power the sensor on or off via the PD bit in CTRL_REG1.
 * @param  dev     Driver handle.
 * @param  active  true = active mode, false = power-down.
 */
void HTS221_SetActive(HTS221_HandleTypeDef *dev, bool active);

/**
 * @brief  Set the measurement output data rate.
 * @param  dev   Driver handle.
 * @param  rate  Desired rate (see hts221_rate_t).
 */
void HTS221_SetDataRate(HTS221_HandleTypeDef *dev, hts221_rate_t rate);

/**
 * @brief  Return the current output data rate as read from the sensor.
 * @param  dev  Driver handle.
 * @return Current hts221_rate_t value.
 */
hts221_rate_t HTS221_GetDataRate(HTS221_HandleTypeDef *dev);

/**
 * @brief  Set the polarity of the DRDY output pin.
 * @param  dev        Driver handle.
 * @param  active_low true = pin asserts low when data is ready.
 */
void HTS221_DrdyActiveLow(HTS221_HandleTypeDef *dev, bool active_low);

/**
 * @brief  Enable or disable the data-ready interrupt on the DRDY pin.
 * @param  dev      Driver handle.
 * @param  enabled  true = DRDY pin is driven when new data is available.
 */
void HTS221_DrdyIntEnabled(HTS221_HandleTypeDef *dev, bool enabled);

/**
 * @brief  Perform a blocking read of temperature and humidity.
 *
 * On success, dev->temperature (°C) and dev->humidity (%RH) are updated.
 *
 * @param  dev  Driver handle.
 * @retval true  Data read and converted successfully.
 * @retval false SPI error.
 */
bool HTS221_Read(HTS221_HandleTypeDef *dev);

#endif /* HTS221_H */
