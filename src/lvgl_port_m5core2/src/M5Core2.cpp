#include "../include/M5Core2.hpp"
#include "config.h"
#include "disp_spi.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "ft6x36.h"
#include "helpers.h"
#include "ili9341.h"
#include <mutex>

#include <lv_conf.h>

#define LV_TICK_PERIOD_MS 1
#define TAG "M5Core2"

std::mutex gui_mutex;

_AXP192 pmu;

#if 0
static void lv_tick_task(void *arg) {
  (void)arg;

  lv_tick_inc(LV_TICK_PERIOD_MS);
}
static void task_handler_task(void *pvParameter) {
  while (1) {
    vTaskDelay(pdMS_TO_TICKS(10));

    std::lock_guard<std::mutex> lk(gui_mutex);
    lv_task_handler();
  }
}

#else

void vTimerCallback_LvglTick(TimerHandle_t xTimer) {
  (void)xTimer;

  lv_tick_inc(10);
}


void lv_tick_init()
{
    // 申请定时器， 配置
  TimerHandle_t xTimerLvgl = xTimerCreate("LvglTick",
  /*定时溢出周期， 单位是任务节拍数*/
  10, 
  /*是否自动重载， 此处设置周期性执行*/
  pdTRUE,
  /*记录定时器溢出次数， 初始化零, 用户自己设置*/
  ( void * ) 0,
  /*回调函数*/
  vTimerCallback_LvglTick);

    if( xTimerLvgl != NULL ) {
        // 启动定时器， 0 表示不阻塞
        xTimerStart( xTimerLvgl, 0 );
    }
}


#endif

void m5core2_init() {
  // i2c and spi setup
  lvgl_i2c_driver_init(I2C_NUM_0, CONFIG_I2C_SDA, CONFIG_I2C_SCL,
                       TOUCH_I2C_SPEED_HZ);

  pmu.begin();
  lv_init();

  lvgl_spi_driver_init(HSPI_HOST, DISP_SPI_MISO, DISP_SPI_MOSI, DISP_SPI_CLK,
                       SPI_BUS_MAX_TRANSFER_SZ, 1, -1, -1);

  disp_spi_add_device(HSPI_HOST);
  ili9341_init();

  // create bufffers
  lv_color_t *buf1 = (lv_color_t *)heap_caps_malloc(
      DISP_BUF_SIZE * sizeof(lv_color_t), MALLOC_CAP_DMA);
  assert(buf1 != NULL);
  lv_color_t *buf2 = (lv_color_t *)heap_caps_malloc(
      DISP_BUF_SIZE * sizeof(lv_color_t), MALLOC_CAP_DMA);
  assert(buf2 != NULL);

  static lv_disp_buf_t disp_buf;
  uint32_t size_in_px = DISP_BUF_SIZE;
  lv_disp_buf_init(&disp_buf, buf1, buf2, size_in_px);

  // display driver setup
  lv_disp_drv_t disp_drv;
  lv_disp_drv_init(&disp_drv);
  disp_drv.hor_res = 320;
  disp_drv.ver_res = 240;
  disp_drv.flush_cb = ili9341_flush;
  disp_drv.buffer = &disp_buf;
  lv_disp_drv_register(&disp_drv);

  // touch setup
  ft6x06_init(FT6236_I2C_SLAVE_ADDR);
  lv_indev_drv_t indev_drv;
  lv_indev_drv_init(&indev_drv);
  indev_drv.read_cb = ft6x36_read;
  indev_drv.type = LV_INDEV_TYPE_POINTER;
  lv_indev_drv_register(&indev_drv);

#if 0
  // tick timer
  // TODO: replace with callback?
  const esp_timer_create_args_t periodic_timer_args = {
      .callback = &lv_tick_task,
      .arg = NULL,
      .dispatch_method = ESP_TIMER_TASK,
      .name = "periodic_gui",
  };
  esp_timer_handle_t periodic_timer;
  ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
  ESP_ERROR_CHECK(
      esp_timer_start_periodic(periodic_timer, LV_TICK_PERIOD_MS * 1000));

  // task handler task handler task
  xTaskCreatePinnedToCore(task_handler_task, "task_handler", 4096 * 2, NULL, 0,
                          NULL, 1);
#else

lv_tick_init();

#endif                          
}
