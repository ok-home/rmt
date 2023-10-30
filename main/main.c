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
#include "soc/rmt_periph.h"

#include "driver/pulse_cnt.h"

void logic_analyzer_ws_server(void);

static const char *TAG = "RMT 257*32";

// pin & channel define
#define RMT_TX_GPIO_PIXEL 6
#define RMT_TX_PIXEL_CHANNEL 0
#define RMT_RX_GPIO_HSYNC 6
#define RMT_RX_HSYNC_CHANNEL 4

TaskHandle_t rmt_task_handle = NULL;
/*
debug pin
*/
int x_lvl = 0; // debug
#define IRQ_DBG_GPIO 10
#define START_DBG_GPIO 11

#define TX_PIX_THRES sizeof(pix_sample) / sizeof(pix_sample[0])
#define HSYNC_CNT 32

#define TX_EOF_S 0
// rmt div - clk=10 mHz
#define CLK_80_DIV 8
// pixels 0,5+0.5 mks
#define PIX_HIGHT 5
#define PIX_LOW 5
// every 257 pixel 1+1 mks
#define PIX_H_HIGHT 10
#define PIX_H_LOW 10
// define pixel samples
#define PIX_S                            \
    {                                    \
        {                                \
            {                            \
                PIX_HIGHT, 1, PIX_LOW, 0 \
            }                            \
        }                                \
    }
#define PIX_H                                \
    {                                        \
        {                                    \
            {                                \
                PIX_H_HIGHT, 1, PIX_H_LOW, 0 \
            }                                \
        }                                    \
    }
#define TX_EOF                           \
    {                                    \
        {                                \
            {                            \
                TX_EOF_S, 1, TX_EOF_S, 0 \
            }                            \
        }                                \
    }
// define pixel samples array
static const rmt_item32_t pix_sample[] =
    {
        PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_H, PIX_H, TX_EOF, TX_EOF

};
int dir = 0;
// main irq handler
void IRAM_ATTR fn_tx_isr(void *arg)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    static int lvl = 0; // debug

    uint32_t status = rmt_ll_tx_get_interrupt_status(&RMT, RMT_TX_PIXEL_CHANNEL);
    if (status & RMT_LL_EVENT_TX_DONE(RMT_TX_PIXEL_CHANNEL)) // end of frame
    {
        gpio_ll_set_level(&GPIO, IRQ_DBG_GPIO, 1 & lvl++);
        rmt_ll_enable_interrupt(&RMT, RMT_LL_EVENT_TX_DONE(RMT_TX_PIXEL_CHANNEL), false);
        rmt_ll_clear_interrupt_status(&RMT, RMT_LL_EVENT_TX_DONE(RMT_TX_PIXEL_CHANNEL));
        vTaskNotifyGiveFromISR(rmt_task_handle, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        gpio_ll_set_level(&GPIO, IRQ_DBG_GPIO, 1 & lvl++);
        // output enable flip/flop
        if (dir & 1)
            REG_WRITE(GPIO_ENABLE_W1TS_REG, 1ULL << RMT_TX_GPIO_PIXEL); // enable output
        else
            REG_WRITE(GPIO_ENABLE_W1TC_REG, 1ULL << RMT_TX_GPIO_PIXEL); // disable output
        dir++;
    }

    //    uint32_t  status1 = RMT.int_st.val;
    //    RMT.int_clr.val = status1;
}

static void rmt_rx_init()
{
    rmt_config_t rmt_rx_config = RMT_DEFAULT_CONFIG_RX(RMT_RX_GPIO_HSYNC, RMT_RX_HSYNC_CHANNEL);
    rmt_rx_config.clk_div = CLK_80_DIV;
    rmt_rx_config.rx_config.filter_en = false;
    rmt_rx_config.mem_block_num = 1;
    rmt_rx_config.rx_config.filter_ticks_thresh = 2;
    rmt_config(&rmt_rx_config);
}
/*
 * Initialize the RMT Tx channel
 */
static void rmt_tx_init()
{
    rmt_config_t config_s = RMT_DEFAULT_CONFIG_TX(RMT_TX_GPIO_PIXEL, RMT_TX_PIXEL_CHANNEL);
    config_s.tx_config.loop_en = false;
    config_s.clk_div = CLK_80_DIV;
    config_s.mem_block_num = 1;
    config_s.tx_config.idle_level = RMT_IDLE_LEVEL_HIGH;
    rmt_config(&config_s);
}
static void rmt_init(void *p)
{
    rmt_tx_init();
    rmt_rx_init();
    // restore out sig after rx_init
    esp_rom_gpio_connect_out_signal(RMT_TX_GPIO_PIXEL, rmt_periph_signals.groups[0].channels[RMT_TX_PIXEL_CHANNEL].tx_sig, false, false);
    // switch gpio control -> control GPIO_ENABLE_W1TS_REG/GPIO_ENABLE_W1TC_REG for GPIO 0-31
    uint32_t tmp = REG_READ(GPIO_FUNC0_OUT_SEL_CFG_REG + RMT_TX_GPIO_PIXEL * 4);
    REG_WRITE(GPIO_FUNC0_OUT_SEL_CFG_REG + RMT_TX_GPIO_PIXEL * 4, tmp | 1 << 10);
    REG_WRITE(GPIO_ENABLE_W1TS_REG, 1ULL << RMT_TX_GPIO_PIXEL); // enable output

    rmt_fill_tx_items(RMT_TX_PIXEL_CHANNEL, pix_sample, sizeof(pix_sample) / sizeof(pix_sample[0]), 0);
    rmt_tx_start(RMT_TX_PIXEL_CHANNEL, true);
    // register rmt irq
    rmt_isr_register(fn_tx_isr, NULL, ESP_INTR_FLAG_SHARED | ESP_INTR_FLAG_IRAM | ESP_INTR_FLAG_LOWMED, NULL);

    while (1)
    {
        gpio_ll_set_level(&GPIO, START_DBG_GPIO, 1 & x_lvl++); // debug
        rmt_ll_enable_interrupt(&RMT, RMT_LL_EVENT_TX_DONE(RMT_TX_PIXEL_CHANNEL), false);
        rmt_ll_clear_interrupt_status(&RMT, RMT_LL_EVENT_TX_DONE(RMT_TX_PIXEL_CHANNEL));
        rmt_ll_enable_interrupt(&RMT, RMT_LL_EVENT_TX_DONE(RMT_TX_PIXEL_CHANNEL), true);
        rmt_tx_start(RMT_TX_PIXEL_CHANNEL, true);

        vTaskDelay(1);
    }
}
void rmt_task(void *p)
{
    xTaskCreatePinnedToCore(rmt_init, "rmt init", 4000, NULL, 5, NULL, 1);
    vTaskDelay(10); // wait init done

    while (1)
    {
        // TX interrupt
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);               // irq eof
        gpio_ll_set_level(&GPIO, START_DBG_GPIO, 1 & x_lvl++); // debug
    }
}

void app_main(void)
{
    logic_analyzer_ws_server(); // internal Logic Analyzer // remove

    ESP_LOGI(TAG, "Configuring transmitter");
    // debug pin
    gpio_reset_pin(IRQ_DBG_GPIO);
    gpio_set_direction(IRQ_DBG_GPIO, GPIO_MODE_OUTPUT);
    gpio_reset_pin(START_DBG_GPIO);
    gpio_set_direction(START_DBG_GPIO, GPIO_MODE_OUTPUT);

    xTaskCreatePinnedToCore(rmt_task, "rmt task", 4000, NULL, 5, &rmt_task_handle, tskNO_AFFINITY);
}