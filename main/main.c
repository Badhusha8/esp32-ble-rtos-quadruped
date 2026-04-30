#include <stdio.h>
#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "driver/uart.h"
#include "esp_log.h"

#include "pca9685.h"

/* ------------------------------------------------------------------ */
/*  Configuration                                                       */
/* ------------------------------------------------------------------ */

#define I2C_MASTER_SDA   21
#define I2C_MASTER_SCL   22
#define I2C_PORT         I2C_NUM_0

#define UART_PORT        UART_NUM_0
#define UART_BAUD        115200
#define UART_BUF_SIZE    256

/* Servo pulse-width limits (PCA9685 ticks at 50 Hz, 4096 steps)
   150 ≈ 0.73 ms  (≈ 0°)
   600 ≈ 2.93 ms  (≈ 180°)                                           */
#define SERVO_MIN  150
#define SERVO_MAX  600

#define NUM_SERVOS 12

static const char *TAG = "robodog";

/* ------------------------------------------------------------------ */
/*  Poses  (identical values to the original Arduino sketch)           */
/* ------------------------------------------------------------------ */

/* Indices:  0  1  2   FL
             3  4  5   FR
             6  7  8   BL
             9 10 11   BR  */

static const int sitPos[NUM_SERVOS]   = {   0, 150,  90,   170,  25, 90,    0, 160, 90,   170, 30, 90 };
static const int midPos[NUM_SERVOS]   = {  10, 150,  90,   160,  25, 90,   40, 165, 90,   130, 30, 90 };
static const int standPos[NUM_SERVOS] = { 110, 110,  90,    70,  70, 90,  110, 120, 90,    65, 70, 90 };
static const int waveA[NUM_SERVOS]    = { 110, 110,  90,    70,  70, 90,   50, 140, 90,   110, 70, 90 };
static const int waveB[NUM_SERVOS]    = { 110, 110,  90,   130, 180, 90,   50, 140, 90,   110, 70, 90 };

/* ------------------------------------------------------------------ */
/*  Runtime state                                                       */
/* ------------------------------------------------------------------ */

static pca9685_handle_t pca;
static int currentAngle[NUM_SERVOS];

/* ------------------------------------------------------------------ */
/*  Servo helpers                                                       */
/* ------------------------------------------------------------------ */

static uint16_t angle_to_pulse(int angle)
{
    /* map(angle, 0, 180, SERVO_MIN, SERVO_MAX) */
    return (uint16_t)(SERVO_MIN + (int64_t)(angle) * (SERVO_MAX - SERVO_MIN) / 180);
}

static void set_servo(int ch, int angle)
{
    pca9685_set_pwm_value(&pca, (uint8_t)ch, angle_to_pulse(angle));
}

/**
 * @brief  Smoothly interpolate all servos from currentAngle[] to target[].
 * @param  target    Destination angles for all NUM_SERVOS channels.
 * @param  duration_ms  Total move time in milliseconds.
 */
static void move_smooth(const int target[], int duration_ms)
{
    const int steps        = 60;
    const int delay_per_step = duration_ms / steps;   /* ms */

    int start[NUM_SERVOS];
    for (int i = 0; i < NUM_SERVOS; i++) start[i] = currentAngle[i];

    for (int s = 1; s <= steps; s++) {
        for (int i = 0; i < NUM_SERVOS; i++) {
            int angle = start[i] + (target[i] - start[i]) * s / steps;
            set_servo(i, angle);
        }
        vTaskDelay(pdMS_TO_TICKS(delay_per_step > 0 ? delay_per_step : 1));
    }

    for (int i = 0; i < NUM_SERVOS; i++) currentAngle[i] = target[i];
}

/* ------------------------------------------------------------------ */
/*  Command handler                                                     */
/* ------------------------------------------------------------------ */

static void handle_command(const char *cmd)
{
    if (strcmp(cmd, "stand") == 0) {
        ESP_LOGI(TAG, "Standing...");
        move_smooth(midPos,   1500);
        vTaskDelay(pdMS_TO_TICKS(200));
        move_smooth(standPos, 1500);
        ESP_LOGI(TAG, "Done.");
        printf("Done.\r\n");

    } else if (strcmp(cmd, "sit") == 0) {
        ESP_LOGI(TAG, "Sitting...");
        move_smooth(midPos, 1500);
        vTaskDelay(pdMS_TO_TICKS(200));
        move_smooth(sitPos, 1500);
        ESP_LOGI(TAG, "Done.");
        printf("Done.\r\n");

    } else if (strcmp(cmd, "wave") == 0) {
        ESP_LOGI(TAG, "Waving...");
        move_smooth(waveA,    1000);
        vTaskDelay(pdMS_TO_TICKS(200));
        move_smooth(waveB,     800);
        vTaskDelay(pdMS_TO_TICKS(200));
        move_smooth(waveA,     800);
        vTaskDelay(pdMS_TO_TICKS(200));
        move_smooth(waveB,     800);
        vTaskDelay(pdMS_TO_TICKS(200));
        move_smooth(standPos, 1000);
        ESP_LOGI(TAG, "Done.");
        printf("Done.\r\n");

    } else {
        printf("Unknown command. Available: stand | sit | wave\r\n");
    }
}

/* ------------------------------------------------------------------ */
/*  UART command reader task                                            */
/* ------------------------------------------------------------------ */

static void uart_task(void *arg)
{
    uint8_t buf[UART_BUF_SIZE];
    char    line[UART_BUF_SIZE];
    int     line_len = 0;

    while (1) {
        int len = uart_read_bytes(UART_PORT, buf, sizeof(buf) - 1,
                                  pdMS_TO_TICKS(20));
        for (int i = 0; i < len; i++) {
            char c = (char)buf[i];

            /* Echo back so the user can see what they type */
            uart_write_bytes(UART_PORT, &c, 1);

            if (c == '\n' || c == '\r') {
                if (line_len > 0) {
                    line[line_len] = '\0';

                    /* Trim trailing whitespace */
                    while (line_len > 0 &&
                           (line[line_len-1] == ' ' || line[line_len-1] == '\t'))
                        line[--line_len] = '\0';

                    /* To lowercase */
                    for (int j = 0; j < line_len; j++)
                        if (line[j] >= 'A' && line[j] <= 'Z')
                            line[j] += 32;

                    printf("\r\n");
                    handle_command(line);
                    printf("> ");
                    line_len = 0;
                }
            } else if (line_len < UART_BUF_SIZE - 1) {
                line[line_len++] = c;
            }
        }
    }
}

/* ------------------------------------------------------------------ */
/*  app_main                                                            */
/* ------------------------------------------------------------------ */

void app_main(void)
{
    /* ---- I2C master bus ---- */
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port        = I2C_PORT,
        .sda_io_num      = I2C_MASTER_SDA,
        .scl_io_num      = I2C_MASTER_SCL,
        .clk_source      = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t bus_handle;
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &bus_handle));

    /* ---- PCA9685 @ 50 Hz ---- */
    ESP_ERROR_CHECK(pca9685_init(bus_handle, PCA9685_ADDR_DEFAULT, 50.0f, &pca));

    /* ---- Boot into sit pose ---- */
    for (int i = 0; i < NUM_SERVOS; i++) {
        currentAngle[i] = sitPos[i];
        set_servo(i, sitPos[i]);
    }
    vTaskDelay(pdMS_TO_TICKS(500));

    /* ---- UART ---- */
    uart_config_t uart_cfg = {
        .baud_rate  = UART_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT, UART_BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT, &uart_cfg));

    printf("\r\nRoboDog ready.\r\n");
    printf("Commands: stand | sit | wave\r\n> ");

    xTaskCreate(uart_task, "uart_task", 4096, NULL, 5, NULL);
}
