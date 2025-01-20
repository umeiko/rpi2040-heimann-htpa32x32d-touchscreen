#ifndef BAT_H
#define BAT_H

#include <Arduino.h>
#include "../shared_val.h"
#define BAT_ADC  26
float bat_v = 0.;

void bat_init(){
    pinMode(BAT_ADC, INPUT);
}

// 将任意范围的浮点数映射到指定范围的整数
inline int map_float_to_int(float value, float in_min, float in_max, int out_min, int out_max) {
    return (int)((value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
}

// 将3.55到4.2的浮点数映射到0-100的整形
inline int map_voltage_to_percentage(float voltage) {
    return map_float_to_int(voltage, 3.55, 4.2, 0, 100);
}

void v_to_percent(float voltage){
    if(voltage < 3.6){
        vbat_percent = 10;
    }else if(voltage < 3.7){
        vbat_percent = 30;
    }else if(voltage < 3.8){
        vbat_percent = 60;
    }else if(voltage < 3.9){
        vbat_percent = 80;
    }else if(voltage < 3.9){
        vbat_percent = 90;
    }else{
        vbat_percent = 100;
    }
}


void bat_loop(){
    static int adc_value = analogRead(BAT_ADC);
    static unsigned long lastMillis = 0;
    static const long xWait = 5000;

    static float r1 = 300.;
    static float r2 = 680.;
    static float coef = (r1+r2) / r2;
    static char c_batv[6];  // 电池电压字符串
    
    if(millis() - lastMillis >= xWait){
      adc_value = analogRead(BAT_ADC);
      bat_v = (float)adc_value / 1024. * 3.3 * coef;
      sprintf(c_batv, "%.2fv", bat_v);
      v_to_percent(bat_v);
      lastMillis = millis();
    }
}


#endif