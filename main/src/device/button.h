#ifndef BUTTON_H
#define BUTTON_H

#include <Arduino.h>
#include "../shared_val.h"
#define BTN_LONG_PUSH_T 1000
#define BUTTON_PIN 24
#define BUTTON_TRIG_LEVEL LOW


void func_button_pushed(){
  // 否则切换拍照模式
    flag_in_photo_mode = !flag_in_photo_mode;
}


void func_button_long_pushed(){
    use_upsample = !use_upsample;
}

void func_bootsel_pushed(){
    // Serial.printf("button pushed\n");
    // _ui_screen_change(&ui_Screen2, LV_SCR_LOAD_ANIM_MOVE_BOTTOM, 500, 0, &ui_Screen2_screen_init);
    flag_show_cursor = !flag_show_cursor;
    if (flag_in_photo_mode==true) {flag_clear_cursor = true;} // 拍照模式下按下这个键就直接重新刷新画面
}

void func_bootsel_long_pushed(){
    // Serial.printf("button Long pushed\n");
    // _ui_screen_change(&ui_Screen1, LV_SCR_LOAD_ANIM_MOVE_TOP, 500, 0, &ui_Screen1_screen_init);
}

void button_init(){
    pinMode(BUTTON_PIN, INPUT_PULLUP);
}

void button_loop(){
    static unsigned long btn_pushed_start_time =  0;
    static bool btn_pushed = false;
    static bool btn_long_pushed = false;
    if (digitalRead(BUTTON_PIN) == BUTTON_TRIG_LEVEL){  // 按钮1触发 
        if (millis() - btn_pushed_start_time >= BTN_LONG_PUSH_T){
            if (!btn_long_pushed){
            func_button_long_pushed();
            btn_long_pushed = true;
            }
        }
        vTaskDelay(5);
        if (digitalRead(BUTTON_PIN) == BUTTON_TRIG_LEVEL){btn_pushed=true;}
    }else{
        btn_pushed_start_time = millis();
        if (btn_pushed) {
            if (!btn_long_pushed){func_button_pushed();}
        }
        btn_pushed=false;
        btn_long_pushed = false;
    }
}

void bootsel_loop(){
    static unsigned long bootsel_pushed_start_time =  0;
    static bool bootsel_pushed = false;
    static bool bootsel_long_pushed = false;
    if (BOOTSEL){  // 长按
         if (millis() - bootsel_pushed_start_time >= BTN_LONG_PUSH_T){
             if (!bootsel_long_pushed){
              func_bootsel_long_pushed();
              bootsel_long_pushed = true;
            }
         } 
         vTaskDelay(5);
         if (BOOTSEL){bootsel_pushed=true;}
      }else{
         bootsel_pushed_start_time = millis();
         if (bootsel_pushed) {  // 短按
          if (!bootsel_long_pushed){func_bootsel_pushed();}
         }
         bootsel_pushed=false;
         bootsel_long_pushed = false;
      }

}

#endif // BUTTON_H