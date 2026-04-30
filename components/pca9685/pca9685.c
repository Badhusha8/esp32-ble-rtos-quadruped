#include "pca9685.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>

static const char *TAG = "pca9685";

/* ------------------------------------------------------------------ */
/*  Low-level helpers                                                   */
/* ------------------------------------------------------------------ */

static esp_err_t write_reg(pca9685_handle_t *h, uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = {reg, val};
    return i2c_master_transmit(h->dev_handle, buf, sizeof(buf), 100);
}

static esp_err_t read_reg(pca9685_handle_t *h, uint8_t reg, uint8_t *val)
{
    esp_err_t ret = i2c_master_transmit(h->dev_handle, &reg, 1, 100);
    if (ret != ESP_OK) return ret;
    return i2c_master_receive(h->dev_handle, val, 1, 100);
}

/* ------------------------------------------------------------------ */
/*  Public API                                                          */
/* ------------------------------------------------------------------ */

esp_err_t pca9685_init(i2c_master_bus_handle_t bus_handle,
                        uint8_t dev_addr,
                        float   pwm_freq_hz,
                        pca9685_handle_t *out_handle)
{
    esp_err_t ret;

    /* Add device to the bus */
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = dev_addr,
        .scl_speed_hz    = 400000,
    };
    ret = i2c_master_bus_add_device(bus_handle, &dev_cfg,
                                    &out_handle->dev_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add device: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Software reset */
    ret = write_reg(out_handle, PCA9685_REG_MODE1, 0x80);
    if (ret != ESP_OK) return ret;
    vTaskDelay(pdMS_TO_TICKS(10));

    /* Sleep mode required to change prescaler */
    ret = write_reg(out_handle, PCA9685_REG_MODE1, 0x10);
    if (ret != ESP_OK) return ret;

    /* prescale = round(osc_clock / (4096 * freq)) - 1
       PCA9685 internal oscillator = 25 MHz */
    uint8_t prescale = (uint8_t)(roundf(25000000.0f / (4096.0f * pwm_freq_hz)) - 1);
    ESP_LOGI(TAG, "PWM freq %.1f Hz → prescale %u", pwm_freq_hz, prescale);

    ret = write_reg(out_handle, PCA9685_REG_PRESCALE, prescale);
    if (ret != ESP_OK) return ret;

    /* Wake, enable auto-increment */
    ret = write_reg(out_handle, PCA9685_REG_MODE1, 0x20);
    if (ret != ESP_OK) return ret;
    vTaskDelay(pdMS_TO_TICKS(5));

    ESP_LOGI(TAG, "Initialised at 0x%02X", dev_addr);
    return ESP_OK;
}

esp_err_t pca9685_set_pwm(pca9685_handle_t *handle,
                            uint8_t channel,
                            uint16_t on,
                            uint16_t off)
{
    uint8_t buf[5];
    buf[0] = PCA9685_REG_LED0_ON_L + 4 * channel;
    buf[1] = on  & 0xFF;
    buf[2] = (on  >> 8) & 0x0F;
    buf[3] = off & 0xFF;
    buf[4] = (off >> 8) & 0x0F;
    return i2c_master_transmit(handle->dev_handle, buf, sizeof(buf), 100);
}

esp_err_t pca9685_set_pwm_value(pca9685_handle_t *handle,
                                  uint8_t channel,
                                  uint16_t value)
{
    return pca9685_set_pwm(handle, channel, 0, value);
}
