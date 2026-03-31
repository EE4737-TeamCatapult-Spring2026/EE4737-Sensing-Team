/*!
 * @file hts221.c
 *
 * C driver for the HTS221 Humidity and Temperature Sensor
 * Targeting STM32 Nucleo F303K8 via STM32 HAL — 3-wire (half-duplex) SPI
 *
 * Adapted from Adafruit_HTS221 by Bryan Siepert / Adafruit Industries.
 * BSD license.
 *
 * ── How 3-wire SPI works on this sensor ─────────────────────────────────────
 *
 * The HTS221 shares one physical wire for both MOSI and MISO (its SDA/SDO
 * pin).  Every transaction looks like this:
 *
 *   Write:  CS↓ → send addr byte (RW=0) → send data byte(s) → CS↑
 *   Read:   CS↓ → send addr byte (RW=1) → receive data byte(s) → CS↑
 *
 * Because the data direction flips between the address phase and the data
 * phase of a read, we must tell the STM32 SPI peripheral to switch its
 * bidirectional line from TX to RX between the two phases.  The HAL provides
 * two macros for this:
 *
 *   __HAL_SPI_ENABLE_IT(hspi, SPI_IT_RXNE)  — not what we need here
 *   SPI_1LINE_TX(hspi)  / SPI_1LINE_RX(hspi)
 *
 * The sequence used in spi_read_regs() below is therefore:
 *   1. Assert CS low.
 *   2. Switch line to TX, transmit the address byte.
 *   3. Switch line to RX, call HAL_SPI_Receive() for N data bytes.
 *   4. Deassert CS high.
 *
 * For writes, the line stays in TX throughout.
 *
 * ── Multi-byte auto-increment ────────────────────────────────────────────────
 *
 * Setting bit 6 of the address byte (HTS221_SPI_MULTI = 0x40) instructs the
 * HTS221 to auto-increment its internal register pointer after each byte,
 * which lets us burst-read e.g. two 8-bit output registers in one CS pulse.
 * This is the same concept as I2C multi-byte but uses bit 6, not bit 7.
 */

#include "hts221.h"
#include <string.h>   /* memset */

/* ═══════════════════════════════════════════════════════════════════════════
 * CS GPIO helpers
 * ═══════════════════════════════════════════════════════════════════════════ */

static inline void cs_assert(void)
{
    HAL_GPIO_WritePin(HTS221_CS_PORT, HTS221_CS_PIN, GPIO_PIN_RESET);
}

static inline void cs_deassert(void)
{
    HAL_GPIO_WritePin(HTS221_CS_PORT, HTS221_CS_PIN, GPIO_PIN_SET);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Low-level SPI helpers
 * ═══════════════════════════════════════════════════════════════════════════ */

/**
 * @brief  Write one or more consecutive registers over 3-wire SPI.
 *
 * Address byte layout:
 *   bit 7 = 0   (write cycle)
 *   bit 6 = MS  (1 if len > 1 to enable auto-increment)
 *   bits 5-0 = register address
 *
 * @param  dev   Driver handle.
 * @param  reg   Starting register address (raw, without flag bits).
 * @param  data  Pointer to bytes to write.
 * @param  len   Number of bytes to write.
 * @retval true  on success.
 */
static bool spi_write_regs(HTS221_HandleTypeDef *dev,
                            uint8_t reg, const uint8_t *data, uint8_t len)
{
    /* Build address byte: RW=0 (write), set MS bit if multi-byte */
    uint8_t addr = reg & ~HTS221_SPI_READ;   /* ensure RW bit is clear */
    if (len > 1) {
        addr |= HTS221_SPI_MULTI;
    }

    /*
     * The bidirectional line starts in TX after the previous transaction.
     * Explicitly switch to TX here so the state is always known.
     */
    SPI_1LINE_TX(dev->hspi);

    cs_assert();

    if (HAL_SPI_Transmit(dev->hspi, &addr, 1,
                         HTS221_SPI_TIMEOUT_MS) != HAL_OK) {
        cs_deassert();
        return false;
    }

    if (HAL_SPI_Transmit(dev->hspi, (uint8_t *)data, len,
                         HTS221_SPI_TIMEOUT_MS) != HAL_OK) {
        cs_deassert();
        return false;
    }

    cs_deassert();
    return true;
}

/**
 * @brief  Read one or more consecutive registers over 3-wire SPI.
 *
 * Address byte layout:
 *   bit 7 = 1   (read cycle)
 *   bit 6 = MS  (1 if len > 1 to enable auto-increment)
 *   bits 5-0 = register address
 *
 * After transmitting the address byte the bidirectional line is switched from
 * TX to RX before clocking in the data bytes.
 *
 * @param  dev  Driver handle.
 * @param  reg  Starting register address (raw, without flag bits).
 * @param  buf  Destination buffer.
 * @param  len  Number of bytes to read.
 * @retval true on success.
 */
static bool spi_read_regs(HTS221_HandleTypeDef *dev,
                           uint8_t reg, uint8_t *buf, uint8_t len)
{
    /* Build address byte: RW=1 (read), set MS bit if multi-byte */
    uint8_t addr = reg | HTS221_SPI_READ;
    if (len > 1) {
        addr |= HTS221_SPI_MULTI;
    }

    /* --- Phase 1: transmit address byte ----------------------------------- */
    SPI_1LINE_TX(dev->hspi);

    cs_assert();

    if (HAL_SPI_Transmit(dev->hspi, &addr, 1,
                         HTS221_SPI_TIMEOUT_MS) != HAL_OK) {
        cs_deassert();
        return false;
    }

    /* --- Phase 2: switch line to RX and clock in data bytes --------------- */
    /*
     * SPI_1LINE_RX() clears the BIDIOE bit in CR1, turning the shared data
     * line from an output into an input.  This must happen while CS is still
     * low and before the first data clock edge.
     */
    SPI_1LINE_RX(dev->hspi);

    if (HAL_SPI_Receive(dev->hspi, buf, len,
                        HTS221_SPI_TIMEOUT_MS) != HAL_OK) {
        cs_deassert();
        return false;
    }

    cs_deassert();

    /* Restore line to TX so the peripheral is ready for the next write */
    SPI_1LINE_TX(dev->hspi);

    return true;
}

/**
 * @brief  Read a single register byte.
 */
static uint8_t spi_read_reg(HTS221_HandleTypeDef *dev, uint8_t reg)
{
    uint8_t val = 0;
    spi_read_regs(dev, reg, &val, 1);
    return val;
}

/**
 * @brief  Read–modify–write a register: change only the bits covered by mask.
 * @param  dev    Driver handle.
 * @param  reg    Register address.
 * @param  mask   Bitmask of bits to modify (1 = affected).
 * @param  value  New values for masked bits, already in position.
 */
static void spi_modify_reg(HTS221_HandleTypeDef *dev,
                            uint8_t reg, uint8_t mask, uint8_t value)
{
    uint8_t current = spi_read_reg(dev, reg);
    current = (current & ~mask) | (value & mask);
    spi_write_regs(dev, reg, &current, 1);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * OTP calibration reads
 * ═══════════════════════════════════════════════════════════════════════════ */

static void fetch_temp_calibration(HTS221_HandleTypeDef *dev)
{
    uint8_t buf[2];
    HAL_StatusTypeDef status;

    /*
     * T1_T0_MSB (0x35) holds the upper 2 bits of both T0 and T1:
     *   bits [1:0] → T0 bits [9:8]
     *   bits [3:2] → T1 bits [9:8]
     * T0_DEGC_X8 (0x32) and T1_DEGC_X8 (0x33) hold the lower 8 bits.
     * All values are in units of °C × 8; shift right 3 to get °C.
     */
    uint8_t msb = spi_read_reg(dev, HTS221_REG_T1_T0_MSB);

    dev->T0 = (uint16_t)((msb & 0x03u) << 8);
    dev->T1 = (uint16_t)((msb & 0x0Cu) << 6);

    /* Burst-read T0_DEGC_X8 (0x32) and T1_DEGC_X8 (0x33) in one transaction */
    status = spi_read_regs(dev, HTS221_REG_T0_DEGC_X8, buf, 2) ? HAL_OK : HAL_ERROR;
    dev->T0 = (dev->T0 | buf[0]) >> 3;
    dev->T1 = (dev->T1 | buf[1]) >> 3;

    /* T0_OUT (0x3C–0x3D) and T1_OUT (0x3E–0x3F): 16-bit signed, LSB first */
    spi_read_regs(dev, HTS221_REG_T0_OUT, buf, 2);
    dev->T0_OUT = (int16_t)((uint16_t)buf[1] << 8 | buf[0]);

    spi_read_regs(dev, HTS221_REG_T1_OUT, buf, 2);
    dev->T1_OUT = (int16_t)((uint16_t)buf[1] << 8 | buf[0]);
}

static void fetch_humidity_calibration(HTS221_HandleTypeDef *dev)
{
    uint8_t buf[2];

    /* H0 and H1 are stored as (%RH × 2); divide by 2.0 when applying */
    dev->H0 = spi_read_reg(dev, HTS221_REG_H0_RH_X2);
    dev->H1 = spi_read_reg(dev, HTS221_REG_H1_RH_X2);

    /* H0_T0_OUT (0x36–0x37): 16-bit signed, LSB first */
    spi_read_regs(dev, HTS221_REG_H0_T0_OUT, buf, 2);
    dev->H0_T0_OUT = (int16_t)((uint16_t)buf[1] << 8 | buf[0]);

    /* H1_T0_OUT (0x3A–0x3B): 16-bit signed, LSB first */
    spi_read_regs(dev, HTS221_REG_H1_T0_OUT, buf, 2);
    dev->H1_T0_OUT = (int16_t)((uint16_t)buf[1] << 8 | buf[0]);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Calibration correction math
 * ═══════════════════════════════════════════════════════════════════════════ */

/**
 * @brief  Convert a raw temperature ADC count to degrees Celsius.
 *
 * Linear interpolation between the two factory calibration points:
 *   T = (raw - T0_OUT) × (T1 - T0) / (T1_OUT - T0_OUT) + T0
 *
 * Reference: HTS221 datasheet §8.2 / ST AN4672.
 */
static float apply_temp_correction(const HTS221_HandleTypeDef *dev,
                                   int16_t raw)
{
	 int16_t denom = dev->T1_OUT - dev->T0_OUT;
	 if (denom == 0) return -999.0f;   // sentinel — means cal read failed

    return (float)(raw - dev->T0_OUT)
           * (float)((int16_t)dev->T1 - (int16_t)dev->T0)
           / (float)(dev->T1_OUT - dev->T0_OUT)
           + (float)(int16_t)dev->T0;
}

/**
 * @brief  Convert a raw humidity ADC count to percent relative humidity.
 *
 * H0 and H1 are stored ×2 in OTP; divide by 2 to get real %RH cal points.
 *   H = (raw - H0_T0_OUT) × (H1/2 - H0/2) / (H1_T0_OUT - H0_T0_OUT) + H0/2
 */
static float apply_humidity_correction(const HTS221_HandleTypeDef *dev,
                                       int16_t raw)
{
	int16_t denom = dev->H1_T0_OUT - dev->H0_T0_OUT;
	if (denom == 0) return -999.0f;   // sentinel

    float h_span = ((float)(int16_t)dev->H1 - (float)(int16_t)dev->H0) / 2.0f;
    float h_temp = (float)(raw - dev->H0_T0_OUT)
                   * h_span
                   / (float)(dev->H1_T0_OUT - dev->H0_T0_OUT);
    return (float)(int16_t)dev->H0 / 2.0f + h_temp;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Public API
 * ═══════════════════════════════════════════════════════════════════════════ */

bool HTS221_Init(HTS221_HandleTypeDef *dev, SPI_HandleTypeDef *hspi)
{
    memset(dev, 0, sizeof(*dev));
    dev->hspi = hspi;

    /* CS idles high */
    cs_deassert();

    /* Line starts in TX */
    SPI_1LINE_TX(dev->hspi);

    /* Verify chip identity */
    if (spi_read_reg(dev, HTS221_REG_WHOAMI) != HTS221_CHIP_ID) {
        return false;
    }

    HTS221_Boot(dev);
    HTS221_SetActive(dev, true);
    HTS221_SetDataRate(dev, HTS221_RATE_12_5_HZ);

    fetch_temp_calibration(dev);
    fetch_humidity_calibration(dev);

    return true;
}

void HTS221_Boot(HTS221_HandleTypeDef *dev)
{
    /* Set BOOT bit (bit 7) in CTRL_REG2; hardware clears it when done */
    spi_modify_reg(dev, HTS221_REG_CTRL2, 0x80u, 0x80u);

    uint8_t ctrl2;
    do {
        HAL_Delay(1);
        ctrl2 = spi_read_reg(dev, HTS221_REG_CTRL2);
    } while (ctrl2 & 0x80u);
}

void HTS221_SetActive(HTS221_HandleTypeDef *dev, bool active)
{
    /* PD = bit 7 of CTRL_REG1: 1 = active, 0 = power-down */
    spi_modify_reg(dev, HTS221_REG_CTRL1, 0x80u, active ? 0x80u : 0x00u);
}

void HTS221_SetDataRate(HTS221_HandleTypeDef *dev, hts221_rate_t rate)
{
    /* ODR = bits [1:0] of CTRL_REG1 */
    spi_modify_reg(dev, HTS221_REG_CTRL1, 0x03u, (uint8_t)rate & 0x03u);
}

hts221_rate_t HTS221_GetDataRate(HTS221_HandleTypeDef *dev)
{
    return (hts221_rate_t)(spi_read_reg(dev, HTS221_REG_CTRL1) & 0x03u);
}

void HTS221_DrdyActiveLow(HTS221_HandleTypeDef *dev, bool active_low)
{
    /* DRDY_H_L = bit 7 of CTRL_REG3 */
    spi_modify_reg(dev, HTS221_REG_CTRL3, 0x80u, active_low ? 0x80u : 0x00u);
}

void HTS221_DrdyIntEnabled(HTS221_HandleTypeDef *dev, bool enabled)
{
    /* DRDY = bit 2 of CTRL_REG3 */
    spi_modify_reg(dev, HTS221_REG_CTRL3, 0x04u, enabled ? 0x04u : 0x00u);
}

bool HTS221_Read(HTS221_HandleTypeDef *dev)
{
    uint8_t buf[2];

    /* ── Humidity: two bytes starting at 0x28 ───────────────────────────── */
    if (!spi_read_regs(dev, HTS221_REG_HUMIDITY_OUT, buf, 2)) {
        return false;
    }
    int16_t raw_hum = (int16_t)((uint16_t)buf[1] << 8 | buf[0]);

    /* ── Temperature: two bytes starting at 0x2A ────────────────────────── */
    if (!spi_read_regs(dev, HTS221_REG_TEMP_OUT_L, buf, 2)) {
        return false;
    }
    int16_t raw_temp = (int16_t)((uint16_t)buf[1] << 8 | buf[0]);

    /* ── Apply OTP calibration ───────────────────────────────────────────── */
    dev->temperature = apply_temp_correction(dev, raw_temp);
    dev->humidity    = apply_humidity_correction(dev, raw_hum);

    return true;
}
