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

static const char *TAG = "RMT 257*32";

// pin & channel define
#define RMT_TX_GPIO_PIXEL 18
#define RMT_TX_PIXEL_CHANNEL 0
#define RMT_TX_GPIO_HSYNC 19
#define RMT_TX_HSYNC_CHANNEL 5
#define RMT_TX_GPIO_VSYNC 21
#define RMT_TX_VSYNC_CHANNEL 6
#define RMT_TX_GPIO_LOOP_RESET 25
#define RMT_TX_LOOP_RESET_CHANNEL 7


/*
debug pin
*/
#define IRQ_DBG_GPIO 26
#define START_DBG_GPIO 27

#define TX_PIX_THRES sizeof(pix_sample) / sizeof(pix_sample[0])
#define HSYNC_CNT 32

#define TX_EOF_S 0
// rmt div - clk=2 mHz
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

        PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S,
        PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S,
        PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S,
        PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S,
        PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S,
        PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S,
        PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S,
        PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S,
        PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S,
        PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S,
        PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S,
        PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S,
        PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S,
        PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S,

        PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S,
        PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S, PIX_S,
        PIX_H, TX_EOF};
// define h-string pulse ( every 257 pixel)
#define HS_HIGHT (((PIX_HIGHT + PIX_LOW) * (TX_PIX_THRES - 2))) + 1
#define HS_LOW (PIX_H_HIGHT + PIX_H_LOW)
#define HS_S                           \
    {                                  \
        {                              \
            {                          \
                HS_HIGHT, 1, HS_LOW, 0 \
            }                          \
        }                              \
    }
#define HS_V                           \
    {                                  \
        {                              \
            {                          \
                HS_HIGHT, 1, HS_LOW, 0 \
            }                          \
        }                              \
    }
// define h-string sample array ( 32 string )
static const rmt_item32_t h_sync_sample[] =
    {

        HS_S, HS_S, HS_S, HS_S, HS_S, HS_S, HS_S, HS_S,
        HS_S, HS_S, HS_S, HS_S, HS_S, HS_S, HS_S, HS_S,
        HS_S, HS_S, HS_S, HS_S, HS_S, HS_S, HS_S, HS_S,
        HS_S, HS_S, HS_S, HS_S, HS_S, HS_S, HS_S,
        HS_V, TX_EOF};
// define v pulse - every 32 h-string
#define VS_HIGHT (((HS_HIGHT + HS_LOW) * (HSYNC_CNT)) - HS_LOW )
#define VS_LOW (HS_LOW)
#define VS_DEL_16                       \
    {                                  \
        {                              \
            {                          \
                VS_HIGHT/16, 1, VS_HIGHT/16, 1 \
            }                          \
        }                              \
    }

#define VS_S                           \
    {                                  \
        {                              \
            {                          \
                VS_HIGHT-(VS_HIGHT/16)*14, 1, VS_LOW, 0 \
            }                          \
        }                              \
    }
// define v-string sample array (pulse every 32 string )
static const rmt_item32_t v_sync_sample[] =
    {
       VS_DEL_16,VS_DEL_16,VS_DEL_16,VS_DEL_16,VS_DEL_16,VS_DEL_16,VS_DEL_16, VS_S, TX_EOF};



// define loop reset - every 31 h-string
#define LR_HIGHT (((HS_HIGHT + HS_LOW) * (HSYNC_CNT-1))  )
#define LR_LOW (HS_LOW)
#define LR_DEL_16                       \
    {                                  \
        {                              \
            {                          \
                LR_HIGHT/16, 1, LR_HIGHT/16, 1 \
            }                          \
        }                              \
    }

#define LR_S                           \
    {                                  \
        {                              \
            {                          \
                LR_HIGHT-(LR_HIGHT/16)*14, 1, LR_LOW, 0 \
            }                          \
        }                              \
    }
// define v-string sample array (pulse every 32 string )
static const rmt_item32_t loop_reset_sample[] =
    {
       LR_DEL_16,LR_DEL_16,LR_DEL_16,LR_DEL_16,LR_DEL_16,LR_DEL_16,LR_DEL_16, LR_S, TX_EOF};







// start frame 257*32
static void IRAM_ATTR start_loop()
{
    rmt_ll_tx_enable_loop(&RMT, RMT_TX_PIXEL_CHANNEL, true);
    rmt_ll_tx_enable_loop(&RMT, RMT_TX_HSYNC_CHANNEL, true);
    rmt_ll_tx_enable_loop(&RMT, RMT_TX_VSYNC_CHANNEL, true);
    rmt_ll_tx_enable_loop(&RMT, RMT_TX_LOOP_RESET_CHANNEL, true);
};
// stop frame ( stop loop - process last string )
static void IRAM_ATTR stop_loop()
{
    rmt_ll_tx_enable_loop(&RMT, RMT_TX_PIXEL_CHANNEL, false);
    rmt_ll_tx_enable_loop(&RMT, RMT_TX_HSYNC_CHANNEL, false);
    rmt_ll_tx_enable_loop(&RMT, RMT_TX_VSYNC_CHANNEL, false);
    rmt_ll_tx_enable_loop(&RMT, RMT_TX_LOOP_RESET_CHANNEL, false);
};
// main irq handler
void IRAM_ATTR fn_tx_isr(void *arg)
{
    static uint32_t cnt = 0;
    static int lvl = 0; // debug
    uint32_t status = rmt_ll_tx_get_interrupt_status(&RMT, RMT_TX_PIXEL_CHANNEL);
    if (status & RMT_LL_EVENT_TX_THRES(RMT_TX_PIXEL_CHANNEL)) // loop 257 pixel event
    {
        if (cnt++ >= (HSYNC_CNT - 2)) // 31 string
        {
            stop_loop(); // process last string
            rmt_ll_enable_interrupt(&RMT, RMT_LL_EVENT_TX_THRES(RMT_TX_PIXEL_CHANNEL), false); // disable thres int
            // enable end frame int
            rmt_ll_clear_interrupt_status(&RMT, RMT_LL_EVENT_TX_DONE(RMT_TX_PIXEL_CHANNEL));
            rmt_ll_enable_interrupt(&RMT, RMT_LL_EVENT_TX_DONE(RMT_TX_PIXEL_CHANNEL), true);
        }
        rmt_ll_clear_interrupt_status(&RMT, RMT_LL_EVENT_TX_THRES(RMT_TX_PIXEL_CHANNEL));

        gpio_ll_set_level(&GPIO, IRQ_DBG_GPIO, 1 & lvl++);// debug
    }
    if (status & RMT_LL_EVENT_TX_DONE(RMT_TX_PIXEL_CHANNEL)) // end of frame
    {
        cnt = 0;
        // disable int
        rmt_ll_enable_interrupt(&RMT, RMT_LL_EVENT_TX_DONE(RMT_TX_PIXEL_CHANNEL), false);
        rmt_ll_clear_interrupt_status(&RMT, RMT_LL_EVENT_TX_DONE(RMT_TX_PIXEL_CHANNEL));
        // debug
        gpio_ll_set_level(&GPIO, IRQ_DBG_GPIO, 1 & lvl++);
        gpio_ll_set_level(&GPIO, IRQ_DBG_GPIO, 1 & lvl++);
        gpio_ll_set_level(&GPIO, IRQ_DBG_GPIO, 1 & lvl++);
        //gpio_ll_set_level(&GPIO, IRQ_DBG_GPIO, 1 & lvl++);
    }
}
/*
 * Initialize the RMT Tx channel
 */
static void rmt_tx_init(void *p)
{
    rmt_config_t config_s = RMT_DEFAULT_CONFIG_TX(RMT_TX_GPIO_PIXEL, RMT_TX_PIXEL_CHANNEL);
    config_s.tx_config.loop_en = false;
    config_s.clk_div = CLK_80_DIV;
    config_s.mem_block_num = 5;
    config_s.tx_config.idle_level = RMT_IDLE_LEVEL_HIGH;
    rmt_config(&config_s);
    rmt_fill_tx_items(RMT_TX_PIXEL_CHANNEL, pix_sample, sizeof(pix_sample) / sizeof(pix_sample[0]), 0);

    rmt_config_t config_h = RMT_DEFAULT_CONFIG_TX(RMT_TX_GPIO_HSYNC, RMT_TX_HSYNC_CHANNEL);
    config_h.tx_config.loop_en = false;
    config_h.clk_div = CLK_80_DIV;
    config_h.mem_block_num = 1;
    config_h.tx_config.idle_level = RMT_IDLE_LEVEL_HIGH;
    rmt_config(&config_h);
    rmt_fill_tx_items(RMT_TX_HSYNC_CHANNEL, h_sync_sample, sizeof(h_sync_sample) / sizeof(h_sync_sample[0]), 0);

    rmt_config_t config_v = RMT_DEFAULT_CONFIG_TX(RMT_TX_GPIO_VSYNC, RMT_TX_VSYNC_CHANNEL);
    config_v.tx_config.loop_en = false;
    config_v.clk_div = CLK_80_DIV;
    config_v.mem_block_num = 1;
    config_v.tx_config.idle_level = RMT_IDLE_LEVEL_HIGH;
    rmt_config(&config_v);
    rmt_fill_tx_items(RMT_TX_VSYNC_CHANNEL, v_sync_sample, sizeof(v_sync_sample) / sizeof(v_sync_sample[0]), 0);

    rmt_config_t config_lr = RMT_DEFAULT_CONFIG_TX(RMT_TX_GPIO_LOOP_RESET, RMT_TX_LOOP_RESET_CHANNEL);
    config_lr.tx_config.loop_en = false;
    config_lr.clk_div = CLK_80_DIV;
    config_lr.mem_block_num = 1;
    config_lr.tx_config.idle_level = RMT_IDLE_LEVEL_HIGH;
    rmt_config(&config_lr);
    rmt_fill_tx_items(RMT_TX_LOOP_RESET_CHANNEL, loop_reset_sample, sizeof(loop_reset_sample) / sizeof(loop_reset_sample[0]), 0);


    // register rmt irq
    rmt_isr_register(fn_tx_isr, NULL, /* ESP_INTR_FLAG_SHARED |*/ ESP_INTR_FLAG_IRAM | ESP_INTR_FLAG_LOWMED, NULL);

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

    rmt_tx_init(NULL);

    int lvl = 0; // debug
    while (1)
    {
        gpio_ll_set_level(&GPIO, START_DBG_GPIO, 1 & lvl++); //debug
        // enable 257 pix irq

        rmt_ll_tx_set_limit(&RMT, RMT_TX_PIXEL_CHANNEL, TX_PIX_THRES);
        rmt_ll_clear_interrupt_status(&RMT, RMT_LL_EVENT_TX_THRES(RMT_TX_PIXEL_CHANNEL));
        rmt_ll_enable_interrupt(&RMT, RMT_LL_EVENT_TX_THRES(RMT_TX_PIXEL_CHANNEL), true);



        start_loop(); //start frame
        vTaskDelay(30 / portTICK_PERIOD_MS); 
    }
}