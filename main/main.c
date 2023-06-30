/* RMT example -- Morse Code

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/rmt.h"
#include "driver/gpio.h"
#include "hal/rmt_ll.h"
#include "hal/gpio_ll.h"

void logic_analyzer_ws_server(void);

static const char *TAG = "example";

#define RMT_TX_CHANNEL RMT_CHANNEL_0

#define R1M                \
    {                      \
        {                  \
            {              \
                5, 1, 5, 0 \
            }              \
        }                  \
    }
#define R1F                  \
    {                        \
        {                    \
            {                \
                10, 1, 10, 0 \
            }                \
        }                    \
    }
#define R1EOF              \
    {                      \
        {                  \
            {              \
                0, 1, 0, 0 \
            }              \
        }                  \
    }
#define TX_THRES sizeof(sample) / sizeof(sample[0])
static const rmt_item32_t sample[] =
    {
    /*    
        R1M,R1M,R1M,R1M,R1M,R1M,R1M,R1M,R1M,R1M,R1M,R1M,R1M,R1M,R1M,R1M,
        R1M,R1M,R1M,R1M,R1M,R1M,R1M,R1M,R1M,R1M,R1M,R1M,R1M,R1M,R1M,R1M,
        R1M,R1M,R1M,R1M,R1M,R1M,R1M,R1M,R1M,R1M,R1M,R1M,R1M,R1M,R1M,R1M,
        R1M,R1M,R1M,R1M,R1M,R1M,R1M,R1M,R1M,R1M,R1M,R1M,R1M,R1M,R1M,R1M,
        R1M,R1M,R1M,R1M,R1M,R1M,R1M,R1M,R1M,R1M,R1M,R1M,R1M,R1M,R1M,R1M,
        R1M,R1M,R1M,R1M,R1M,R1M,R1M,R1M,R1M,R1M,R1M,R1M,R1M,R1M,R1M,R1M,
        R1M,R1M,R1M,R1M,R1M,R1M,R1M,R1M,R1M,R1M,R1M,R1M,R1M,R1M,R1M,R1M,
        R1M,R1M,R1M,R1M,R1M,R1M,R1M,R1M,R1M,R1M,R1M,R1M,R1M,R1M,R1M,R1M,
        R1M,R1M,R1M,R1M,R1M,R1M,R1M,R1M,R1M,R1M,R1M,R1M,R1M,R1M,R1M,R1M,
        R1M,R1M,R1M,R1M,R1M,R1M,R1M,R1M,R1M,R1M,R1M,R1M,R1M,R1M,R1M,R1M,
        R1M,R1M,R1M,R1M,R1M,R1M,R1M,R1M,R1M,R1M,R1M,R1M,R1M,R1M,R1M,R1M,
        R1M,R1M,R1M,R1M,R1M,R1M,R1M,R1M,R1M,R1M,R1M,R1M,R1M,R1M,R1M,R1M,
        R1M,R1M,R1M,R1M,R1M,R1M,R1M,R1M,R1M,R1M,R1M,R1M,R1M,R1M,R1M,R1M,
        R1M,R1M,R1M,R1M,R1M,R1M,R1M,R1M,R1M,R1M,R1M,R1M,R1M,R1M,R1M,R1M,
    */    
        R1M, R1M, R1M, R1M, R1M, R1M, R1M, R1M, R1M, R1M, R1M, R1M, R1M, R1M, R1M, R1M,
        R1M, R1M, R1M, R1M, R1M, R1M, R1M, R1M, R1M, R1M, R1M, R1M, R1M, R1M, R1M, R1M,
        R1F, R1EOF};


void IRAM_ATTR fn_tx_isr(void *arg)
{
    static uint32_t cnt = 0;
    uint32_t status = rmt_ll_tx_get_interrupt_status(&RMT, RMT_TX_CHANNEL);
    if (status & RMT_LL_EVENT_TX_THRES(RMT_TX_CHANNEL))
    {
        if (cnt++ > 10)
        {
            rmt_ll_tx_enable_loop(&RMT, RMT_TX_CHANNEL, false);
            rmt_ll_enable_interrupt(&RMT, RMT_LL_EVENT_TX_THRES(RMT_TX_CHANNEL), false);

            rmt_ll_clear_interrupt_status(&RMT, RMT_LL_EVENT_TX_DONE(RMT_TX_CHANNEL));
            rmt_ll_enable_interrupt(&RMT, RMT_LL_EVENT_TX_DONE(RMT_TX_CHANNEL), true);
            cnt = 0;
        }
        rmt_ll_clear_interrupt_status(&RMT, RMT_LL_EVENT_TX_THRES(RMT_TX_CHANNEL));

        gpio_ll_set_level(&GPIO, 19, 1);
        gpio_ll_set_level(&GPIO, 19, 0);

    }
    if (status & RMT_LL_EVENT_TX_DONE(RMT_TX_CHANNEL))
    {
        rmt_ll_enable_interrupt(&RMT, RMT_LL_EVENT_TX_DONE(RMT_TX_CHANNEL), false);
        rmt_ll_clear_interrupt_status(&RMT, RMT_LL_EVENT_TX_DONE(RMT_TX_CHANNEL));

        gpio_ll_set_level(&GPIO, 19, 1);
        gpio_ll_set_level(&GPIO, 19, 0);
        gpio_ll_set_level(&GPIO, 19, 1);
        gpio_ll_set_level(&GPIO, 19, 0);
    }
}
/*
 * Initialize the RMT Tx channel
 */
static void rmt_tx_init(void)
{
    rmt_config_t config = RMT_DEFAULT_CONFIG_TX(CONFIG_EXAMPLE_RMT_TX_GPIO, RMT_TX_CHANNEL);
    config.tx_config.loop_en = true;
    config.clk_div = 8;
    config.mem_block_num = 5;

    ESP_ERROR_CHECK(rmt_config(&config));

    rmt_isr_register(fn_tx_isr, NULL,/* ESP_INTR_FLAG_EDGE |*/ ESP_INTR_FLAG_IRAM | ESP_INTR_FLAG_LOWMED, NULL);
}

void app_main(void)
{
    logic_analyzer_ws_server();
    ESP_LOGI(TAG, "Configuring transmitter");
    gpio_reset_pin(19);
    gpio_set_direction(19, GPIO_MODE_OUTPUT);
    gpio_reset_pin(26);
    gpio_set_direction(26, GPIO_MODE_OUTPUT);

    rmt_tx_init();

    rmt_fill_tx_items(RMT_TX_CHANNEL, sample, sizeof(sample) / sizeof(sample[0]), 0);

    while (1)
    {
        gpio_ll_set_level(&GPIO, 26, 1);
        gpio_ll_set_level(&GPIO, 26, 0);

        rmt_ll_tx_set_limit(&RMT, RMT_TX_CHANNEL, TX_THRES);
        rmt_ll_clear_interrupt_status(&RMT, RMT_LL_EVENT_TX_THRES(RMT_TX_CHANNEL));
        rmt_ll_enable_interrupt(&RMT, RMT_LL_EVENT_TX_THRES(RMT_TX_CHANNEL), true);

        rmt_ll_tx_enable_loop(&RMT, RMT_TX_CHANNEL, true);
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}