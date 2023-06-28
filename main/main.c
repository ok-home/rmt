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

void logic_analyzer_ws_server(void);

static const char *TAG = "example";

#define RMT_TX_CHANNEL RMT_CHANNEL_0

/*
 * Prepare a raw table with a message in the Morse code
 *
 * The message is "ESP" : . ... .--.
 *
 * The table structure represents the RMT item structure:
 * {duration, level, duration, level}
 *
 */
/*
static const rmt_item32_t morse_esp[] = {
    // E : dot
    {{{ 32767, 1, 32767, 0 }}}, // dot
    {{{ 32767, 0, 32767, 0 }}}, // SPACE
    // S : dot, dot, dot
    {{{ 32767, 1, 32767, 0 }}}, // dot
    {{{ 32767, 1, 32767, 0 }}}, // dot
    {{{ 32767, 1, 32767, 0 }}}, // dot
    {{{ 32767, 0, 32767, 0 }}}, // SPACE
    // P : dot, dash, dash, dot
    {{{ 32767, 1, 32767, 0 }}}, // dot
    {{{ 32767, 1, 32767, 1 }}},
    {{{ 32767, 1, 32767, 0 }}}, // dash
    {{{ 32767, 1, 32767, 1 }}},
    {{{ 32767, 1, 32767, 0 }}}, // dash
    {{{ 32767, 1, 32767, 0 }}}, // dot
    // RMT end marker
    {{{ 0, 1, 0, 0 }}}
};*/
#define R1M {{{ 2, 1, 2, 0 }}}
#define R1F {{{ 10, 1, 10, 0 }}}
static const rmt_item32_t morse_esp[]=
{
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
R1M,R1M,R1M,R1M,R1M,R1M,R1M,R1M,R1M,R1M,R1M,R1M,R1M,R1M,R1M,R1M,
R1M,R1M,R1M,R1M,R1M,R1M,R1M,R1M,R1M,R1M,R1M,R1M,R1M,R1M,R1M,R1M,
R1F
};
void fn_tx_end(rmt_channel_t channel, void *arg)
{
    //ESP_LOGI("CB","CH %d",channel);
    gpio_set_level(33,1);
    gpio_set_level(33,0);
    return;
}
/*
 * Initialize the RMT Tx channel
 */
static void rmt_tx_init(void)
{
    rmt_config_t config = RMT_DEFAULT_CONFIG_TX(CONFIG_EXAMPLE_RMT_TX_GPIO, RMT_TX_CHANNEL);
    // enable the carrier to be able to hear the Morse sound
    // if the RMT_TX_GPIO is connected to a speaker
    config.tx_config.carrier_en = false;
    config.tx_config.loop_en = true;

    config.tx_config.carrier_duty_percent = 50;
    // set audible career frequency of 611 Hz
    // actually 611 Hz is the minimum, that can be set
    // with current implementation of the RMT API
    config.tx_config.carrier_freq_hz = 611;
    // set the maximum clock divider to be able to output
    // RMT pulses in range of about one hundred milliseconds
    config.clk_div = 20;
    config.mem_block_num = 8;

    ESP_ERROR_CHECK(rmt_config(&config));
    ESP_ERROR_CHECK(rmt_driver_install(config.channel, 0, 0));
    rmt_register_tx_end_callback(fn_tx_end,NULL);
    rmt_set_tx_thr_intr_en(0,true,50);
}

void app_main(void)
{
    logic_analyzer_ws_server();
    ESP_LOGI(TAG, "Configuring transmitter");
    gpio_reset_pin(33);
    gpio_set_direction(33,GPIO_MODE_OUTPUT);


    rmt_tx_init();

    while (1) {
        ESP_ERROR_CHECK(rmt_write_items(RMT_TX_CHANNEL, morse_esp, sizeof(morse_esp) / sizeof(morse_esp[0]), true));
        ESP_LOGI(TAG, "Transmission complete");
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}