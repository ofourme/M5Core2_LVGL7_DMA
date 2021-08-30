#include <FreeRTOS.h>
#include <driver/i2s.h>
#include <lvgl.h>
#include "lvgl_port_m5core2/include/M5Core2.hpp"
//#include <AXP192.hpp>
#include "FastLED.h"

static void btn_event_cb(lv_obj_t * btn, lv_event_t event)
{
	if(event == LV_EVENT_CLICKED)
	{
		static uint8_t cnt = 0;
		cnt++;
		/*对按键的子类标签进行文字编辑*/
		lv_obj_t * label = lv_obj_get_child(btn, NULL);
		lv_label_set_text_fmt(label, "Button: %d", cnt);
	}
}


void lv_ex_get_started_1(void)
{
  lv_obj_t * btn = lv_btn_create(lv_scr_act(), NULL);
  lv_obj_set_pos(btn, 10, 10);
  lv_obj_set_size(btn, 120, 50);
  lv_obj_set_event_cb(btn, btn_event_cb);
  lv_obj_t * label = lv_label_create(btn, NULL);
  lv_label_set_text(label, "Button");
}

static lv_obj_t* kb;
static lv_obj_t* ta;
static void kb_event_cb(lv_obj_t* keyboard, lv_event_t e)
{
    lv_keyboard_def_event_cb(kb, e);
    if (e == LV_EVENT_CANCEL) {
        lv_keyboard_set_textarea(kb, NULL);
        lv_obj_del(kb);
        kb = NULL;
    }
}

static void kb_create(void)
{
    kb = lv_keyboard_create(lv_scr_act(), NULL);
    lv_keyboard_set_cursor_manage(kb, true);
    lv_obj_set_event_cb(kb, kb_event_cb);
    lv_keyboard_set_textarea(kb, ta);

}

static void ta_event_cb(lv_obj_t* ta_local, lv_event_t e)
{
    if (e == LV_EVENT_CLICKED && kb == NULL) {
        kb_create();
    }
}

void lvgl_keyboard_test(void)
{

    /*Create a text area. The keyboard will write here*/
    ta = lv_textarea_create(lv_scr_act(), NULL);
    lv_obj_align(ta, NULL, LV_ALIGN_IN_TOP_MID, 0, LV_DPI / 16);
    lv_obj_set_event_cb(ta, ta_event_cb);
    lv_textarea_set_text(ta, "");
    lv_coord_t max_h = LV_VER_RES / 2 - LV_DPI / 8;
    if (lv_obj_get_height(ta) > max_h) lv_obj_set_height(ta, max_h);

    kb_create();
}

void setup() 
{
  m5core2_init();
  lv_ex_get_started_1();
  lvgl_keyboard_test();
}

void loop()
{
#if 1

  static unsigned long Sms,  Ems, Cnt;
  Sms = Ems = millis();
  Cnt = 0;
  while(Ems - Sms < 1000)
  {
    Cnt ++;
    lv_task_handler();
    Ems = millis();
//    delay(10);
  }
  printf("LVGL DMA FPS:%d\n", (int)Cnt);

#endif
}
