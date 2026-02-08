/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_event.h"
#include "esp_timer.h"
#include "driver/i2c_slave.h"
#include "driver/gpio.h"

static const char *TAG = "LED Driver";

#define I2C_SLAVE_SCL_IO CONFIG_I2C_SLAVE_SCL               /*!< gpio number for i2c slave clock */
#define I2C_SLAVE_SDA_IO CONFIG_I2C_SLAVE_SDA               /*!< gpio number for i2c slave data */
#define I2C_SLAVE_NUM    0
#define ESP_SLAVE_ADDR   CONFIG_I2C_SLAVE_ADDRESS           /*!< ESP slave address, you can set any 7bit value */

// Command Lists
#define LED_ON_COMMAND  (0x10)
#define LED_OFF_COMMAND  (0x20)
#define LED_SET_BLINK_PERIOD_COMMAND (0x30)

// LED
#define BLINK_GPIO 5

typedef struct {
    uint8_t tmp_buffer[sizeof(int)];
    QueueHandle_t event_queue;
    uint8_t command_data [2];
    esp_timer_handle_t timer;
    i2c_slave_dev_handle_t handle;
} i2c_slave_context_t;

typedef enum {
    I2C_SLAVE_EVT_RX,
    I2C_SLAVE_EVT_TX
} i2c_slave_event_t;

static uint8_t s_led_state = 0;
static uint32_t led_blink_period_u_sec = 1000000;
static void periodic_timer_callback(void* arg);

static void configure_led(void)
{
    gpio_reset_pin(BLINK_GPIO);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
}

static void blink_led(void)
{
    /* Set the GPIO level according to the state (LOW or HIGH)*/
    gpio_set_level(BLINK_GPIO, s_led_state);
}

static bool i2c_slave_request_cb(i2c_slave_dev_handle_t i2c_slave, const i2c_slave_request_event_data_t *evt_data, void *arg)
{
    i2c_slave_context_t *context = (i2c_slave_context_t *)arg;
    i2c_slave_event_t evt = I2C_SLAVE_EVT_TX;
    BaseType_t xTaskWoken = 0;
    xQueueSendFromISR(context->event_queue, &evt, &xTaskWoken);
    return xTaskWoken;
}

static bool i2c_slave_receive_cb(i2c_slave_dev_handle_t i2c_slave, const i2c_slave_rx_done_event_data_t *evt_data, void *arg)
{
    i2c_slave_context_t *context = (i2c_slave_context_t *)arg;
    i2c_slave_event_t evt = I2C_SLAVE_EVT_RX;
    BaseType_t xTaskWoken = 0;
    context->command_data[0] = evt_data->buffer[0];
    context->command_data[1] = evt_data->buffer[1];
    xQueueSendFromISR(context->event_queue, &evt, &xTaskWoken);
    return xTaskWoken;
}

static void i2c_slave_task(void *arg)
{
    i2c_slave_context_t *context = (i2c_slave_context_t *)arg;
    esp_timer_handle_t timer = (esp_timer_handle_t)context->timer;
    while (true) {
        i2c_slave_event_t evt;
        if (xQueueReceive(context->event_queue, &evt, 10) == pdTRUE) {
            if (evt == I2C_SLAVE_EVT_RX) {
                switch (context->command_data[0]) {
                case LED_ON_COMMAND:
                    ESP_LOGI(TAG, "LED_ON_COMMAND");
                    if (esp_timer_is_active(timer) == true)
                    {
                    ESP_LOGI(TAG, "LED_TIMER_ALREADY_RUNNING");
                    }
                    else
                    {
                    ESP_ERROR_CHECK(esp_timer_start_periodic(timer, led_blink_period_u_sec));
                    }
                    break;
                case LED_OFF_COMMAND:
                    ESP_LOGI(TAG, "LED_OFF_COMMAND");
                    if (esp_timer_is_active(timer) == true)
                    {
                    ESP_ERROR_CHECK(esp_timer_stop(timer));
                    s_led_state = 0;
                    blink_led();
                    }
                    else
                    {
                    ESP_LOGI(TAG, "LED_TIMER_ALREADY_STOPPED");
                    }
                    break;
                case LED_SET_BLINK_PERIOD_COMMAND:
                    ESP_LOGI(TAG, "LED_SET_BLINK_PERIOD_COMMAND with period of: %d seconds", context->command_data[1]);
                    led_blink_period_u_sec = 1000000 * context->command_data[1];
                    if (esp_timer_is_active(timer) == true)
                    {
                        ESP_LOGI(TAG, "Restarting LED Timer");
                        ESP_ERROR_CHECK(esp_timer_stop(timer));
                        ESP_ERROR_CHECK(esp_timer_start_periodic(timer, led_blink_period_u_sec));
                    }
                break;
                    default:
                    ESP_LOGE(TAG, "Invalid command");
                    break;
                }
            }
        }
    }
    vTaskDelete(NULL);
}

void app_main(void)
{
    /* Configure the peripheral according to the LED type */
    configure_led();

    /* Create one timer: a periodic timer  */
    const esp_timer_create_args_t periodic_timer_args = {
            .callback = &periodic_timer_callback,
            /* name is optional, but may help identify the timer when debugging */
            .name = "periodic"
    };

    /* The timer has been created but is not running yet */
    esp_timer_handle_t periodic_timer;
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));

    /* Create Event loop for I2C */
    static i2c_slave_context_t context = {0};
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    context.event_queue = xQueueCreate(16, sizeof(i2c_slave_event_t));
    if (!context.event_queue) {
        ESP_LOGE(TAG, "Creating queue failed");
        return;
    }

    context.timer = periodic_timer;

    ESP_LOGI(TAG, "Init I2C LED Driver");

    i2c_slave_config_t i2c_slv_config = {
        .i2c_port = I2C_SLAVE_NUM,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .scl_io_num = I2C_SLAVE_SCL_IO,
        .sda_io_num = I2C_SLAVE_SDA_IO,
        .slave_addr = ESP_SLAVE_ADDR,
        .send_buf_depth = 100,
        .receive_buf_depth = 100,
    };

    ESP_ERROR_CHECK(i2c_new_slave_device(&i2c_slv_config, &context.handle));
    i2c_slave_event_callbacks_t cbs = {
        .on_receive = i2c_slave_receive_cb,
        .on_request = i2c_slave_request_cb,
    };
    ESP_ERROR_CHECK(i2c_slave_register_event_callbacks(context.handle, &cbs, &context));

    xTaskCreate(i2c_slave_task, "i2c_slave_task", 1024 * 4, &context, 10, NULL);
}

static void periodic_timer_callback(void* arg)
{
    blink_led();
    /* Toggle the LED state */
    s_led_state = !s_led_state;
    int64_t time_since_boot = esp_timer_get_time();
    ESP_LOGI(TAG, "Periodic timer called, time since boot: %lld us", time_since_boot);
}