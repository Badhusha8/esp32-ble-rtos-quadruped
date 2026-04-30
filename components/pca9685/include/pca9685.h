#pragma once

#include "driver/i2c_master.h"
#include "esp_err.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Default I2C address of the PCA9685 (A0-A5 all low). */
#define PCA9685_ADDR_DEFAULT  0x40

/** PCA9685 register map */
#define PCA9685_REG_MODE1     0x00
#define PCA9685_REG_PRESCALE  0xFE
#define PCA9685_REG_LED0_ON_L 0x06   /* base; each channel occupies 4 bytes */

typedef struct {
    i2c_master_dev_handle_t dev_handle;
} pca9685_handle_t;

/**
 * @brief  Initialise the PCA9685 and configure it for the given PWM frequency.
 *
 * @param  bus_handle   Existing I2C master bus handle.
 * @param  dev_addr     7-bit I2C address (usually PCA9685_ADDR_DEFAULT).
 * @param  pwm_freq_hz  Desired PWM frequency in Hz (e.g. 50 for servos).
 * @param  out_handle   Output handle to pass to the other functions.
 * @return ESP_OK on success.
 */
esp_err_t pca9685_init(i2c_master_bus_handle_t bus_handle,
                        uint8_t dev_addr,
                        float   pwm_freq_hz,
                        pca9685_handle_t *out_handle);

/**
 * @brief  Set a single PWM channel.
 *
 * @param  handle  Handle returned by pca9685_init().
 * @param  channel Channel number 0-15.
 * @param  on      Tick count where the pulse goes high (0-4095).
 * @param  off     Tick count where the pulse goes low  (0-4095).
 * @return ESP_OK on success.
 */
esp_err_t pca9685_set_pwm(pca9685_handle_t *handle,
                           uint8_t channel,
                           uint16_t on,
                           uint16_t off);

/**
 * @brief  Convenience: set a channel to a specific pulse-width value (0-4095).
 *         on is always 0; off = value.
 */
esp_err_t pca9685_set_pwm_value(pca9685_handle_t *handle,
                                 uint8_t channel,
                                 uint16_t value);

#ifdef __cplusplus
}
#endif
