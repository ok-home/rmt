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

#include "driver/pulse_cnt.h"

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

portMUX_TYPE rmt_mux = portMUX_INITIALIZER_UNLOCKED;
TaskHandle_t rmt_task_handle = NULL;
/*
debug pin
*/
#define IRQ_DBG_GPIO 26
#define START_DBG_GPIO 27

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
#define VS_HIGHT (((HS_HIGHT + HS_LOW) * (HSYNC_CNT)) - HS_LOW)
#define VS_LOW (HS_LOW)
#define VS_DEL_16                                  \
    {                                              \
        {                                          \
            {                                      \
                VS_HIGHT / 16, 1, VS_HIGHT / 16, 1 \
            }                                      \
        }                                          \
    }

#define VS_S                                                  \
    {                                                         \
        {                                                     \
            {                                                 \
                VS_HIGHT - (VS_HIGHT / 16) * 14, 1, VS_LOW, 0 \
            }                                                 \
        }                                                     \
    }
// define v-string sample array (pulse every 32 string )
static const rmt_item32_t v_sync_sample[] =
    {
        VS_DEL_16, VS_DEL_16, VS_DEL_16, VS_DEL_16, VS_DEL_16, VS_DEL_16, VS_DEL_16, VS_S, TX_EOF};

// define loop reset - every 31 h-string
#define LR_HIGHT (((HS_HIGHT + HS_LOW) * (HSYNC_CNT - 1)))
#define LR_LOW (HS_LOW)
#define LR_DEL_16                                  \
    {                                              \
        {                                          \
            {                                      \
                LR_HIGHT / 16, 1, LR_HIGHT / 16, 1 \
            }                                      \
        }                                          \
    }

#define LR_S                                                  \
    {                                                         \
        {                                                     \
            {                                                 \
                LR_HIGHT - (LR_HIGHT / 16) * 14, 1, LR_LOW, 0 \
            }                                                 \
        }                                                     \
    }
// define v-string sample array (pulse every 32 string )
static const rmt_item32_t loop_reset_sample[] =
    {
        LR_DEL_16, LR_DEL_16, LR_DEL_16, LR_DEL_16, LR_DEL_16, LR_DEL_16, LR_DEL_16, LR_S, TX_EOF};

// start frame 257*32
static void IRAM_ATTR start_loop()
{
    portENTER_CRITICAL(&rmt_mux);
    rmt_ll_tx_enable_loop(&RMT, RMT_TX_PIXEL_CHANNEL, true);
    rmt_ll_tx_enable_loop(&RMT, RMT_TX_HSYNC_CHANNEL, true);
    rmt_ll_tx_enable_loop(&RMT, RMT_TX_VSYNC_CHANNEL, true);
    rmt_ll_tx_enable_loop(&RMT, RMT_TX_LOOP_RESET_CHANNEL, true);

    rmt_ll_tx_enable_loop(&RMT, RMT_TX_VSYNC_CHANNEL, false);
    rmt_ll_tx_enable_loop(&RMT, RMT_TX_LOOP_RESET_CHANNEL, false);
    portEXIT_CRITICAL(&rmt_mux);
};
// stop frame ( stop loop - process last string )
static void IRAM_ATTR stop_loop()
{
        //portENTER_CRITICAL(&rmt_mux);
    rmt_ll_tx_enable_loop(&RMT, RMT_TX_PIXEL_CHANNEL, false);
    rmt_ll_tx_enable_loop(&RMT, RMT_TX_HSYNC_CHANNEL, false);
    rmt_ll_tx_enable_loop(&RMT, RMT_TX_VSYNC_CHANNEL, false);
    rmt_ll_tx_enable_loop(&RMT, RMT_TX_LOOP_RESET_CHANNEL, false);
        //portEXIT_CRITICAL(&rmt_mux);
};
// main irq handler
void IRAM_ATTR fn_tx_isr(void *arg)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    static int lvl = 0; // debug
    uint32_t status = rmt_ll_tx_get_interrupt_status(&RMT, RMT_TX_LOOP_RESET_CHANNEL);
    if (status & RMT_LL_EVENT_TX_DONE(RMT_TX_LOOP_RESET_CHANNEL))
    {
        stop_loop(); // process last string
        rmt_ll_enable_interrupt(&RMT, RMT_LL_EVENT_TX_DONE(RMT_TX_LOOP_RESET_CHANNEL), false);
        rmt_ll_clear_interrupt_status(&RMT, RMT_LL_EVENT_TX_DONE(RMT_TX_LOOP_RESET_CHANNEL));

        rmt_ll_clear_interrupt_status(&RMT, RMT_LL_EVENT_TX_DONE(RMT_TX_PIXEL_CHANNEL));
        rmt_ll_enable_interrupt(&RMT, RMT_LL_EVENT_TX_DONE(RMT_TX_PIXEL_CHANNEL), true);

        gpio_ll_set_level(&GPIO, IRQ_DBG_GPIO, 1 & lvl++); // debug
    }
    status = rmt_ll_tx_get_interrupt_status(&RMT, RMT_TX_PIXEL_CHANNEL);
    if (status & RMT_LL_EVENT_TX_DONE(RMT_TX_PIXEL_CHANNEL)) // end of frame
    {
        rmt_ll_enable_interrupt(&RMT, RMT_LL_EVENT_TX_DONE(RMT_TX_PIXEL_CHANNEL), false);
        rmt_ll_clear_interrupt_status(&RMT, RMT_LL_EVENT_TX_DONE(RMT_TX_PIXEL_CHANNEL));

        gpio_ll_set_level(&GPIO, IRQ_DBG_GPIO, 1 & lvl++);

        vTaskNotifyGiveFromISR(rmt_task_handle,&xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR( xHigherPriorityTaskWoken );

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
    rmt_isr_register(fn_tx_isr, NULL,  ESP_INTR_FLAG_SHARED | ESP_INTR_FLAG_IRAM | ESP_INTR_FLAG_LOWMED, NULL);
    while(1){
    vTaskDelay(10);
    }
}

pcnt_unit_handle_t pcnt_unit = NULL;
pcnt_channel_handle_t pcnt_chan = NULL;
// samples counter - debug count ( gpio short 18-22)
void pcnt_init(void)
{
    pcnt_unit_config_t unit_config = {
        .high_limit = 16000,
        .low_limit = -16000,
    };
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_unit));
    pcnt_chan_config_t chan_config = {
        .edge_gpio_num = 22,
        .level_gpio_num = -1,
    };
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_config, &pcnt_chan));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan, PCNT_CHANNEL_EDGE_ACTION_HOLD, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));
}
void pcnt_chk(void)
{
    static int chk_cnt = 0;     // debug
    static int chk_cnt_pre = 0; // debug

    ESP_ERROR_CHECK(pcnt_unit_stop(pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_get_count(pcnt_unit, &chk_cnt));
    if (chk_cnt != chk_cnt_pre)
    {
        ESP_LOGI("CNT", "cnt = %d precnt=%d delta=%d", chk_cnt, chk_cnt_pre, chk_cnt - chk_cnt_pre);
        chk_cnt_pre = chk_cnt;
    }
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit));
}

void rmt_task( void *p)
{
    xTaskCreatePinnedToCore(rmt_tx_init,"rmt init",4000,NULL,5,NULL,1);
    vTaskDelay(10); //wait init done

    pcnt_init(); //debug

    int lvl = 0; // debug

    while (1)
    {
        gpio_ll_set_level(&GPIO, START_DBG_GPIO, 1 & lvl++); // debug
        pcnt_chk(); //debug

        rmt_ll_clear_interrupt_status(&RMT, RMT_LL_EVENT_TX_DONE(RMT_TX_LOOP_RESET_CHANNEL));
        rmt_ll_enable_interrupt(&RMT, RMT_LL_EVENT_TX_DONE(RMT_TX_LOOP_RESET_CHANNEL), true);

        start_loop(); // start frame
        ulTaskNotifyTake(pdTRUE,portMAX_DELAY ); // irq eof

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

    xTaskCreatePinnedToCore(rmt_task,"rmt task",4000,NULL,5,&rmt_task_handle,tskNO_AFFINITY);
}