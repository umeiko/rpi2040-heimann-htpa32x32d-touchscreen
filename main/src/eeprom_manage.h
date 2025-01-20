#ifndef EEPROM_MAN_H
#define EEPROM_MAN_H

#include <Arduino.h>
#include <EEPROM.h>
#include "shared_val.h"
#include "colormap.h"


#define FLAG_CONF_INITIALED 28  // 用来判断是否已经初始化过配置

// 将配置保存到不掉电存储器中
void eeprom_save_config(){
	EEPROM.write(0, FLAG_CONF_INITIALED);
	// EEPROM.write(1, (uint8_t)brightness);
	// EEPROM.write(2, flag_use_kalman);
	// EEPROM.write(3, use_upsample);
	// EEPROM.write(4, flag_trace_max);
	EEPROM.write(5, cmap_now_choose);
	EEPROM.commit();
}

void eeprom_initialize(){
	// brightness = 100;
	// flag_use_kalman = true;
	// use_upsample = true;
	// flag_trace_max = true;
	cmap_now_choose = COLORMAP_TURBO;
	eeprom_save_config();
}

// 从掉电存储器中读取配置参数，并设置屏幕ui
void load_settings(){
	EEPROM.begin(128);
	delay(5);
	if (EEPROM.read(0) != FLAG_CONF_INITIALED) { // 这表示eeprom首次初始化
       eeprom_initialize();
    }else{
		// brightness = EEPROM.read(1);
		// flag_use_kalman = EEPROM.read(2) != 0; // 非零值转换为 true，零值转换为 false
        // use_upsample = EEPROM.read(3) != 0; // 非零值转换为 true，零值转换为 false
        // flag_trace_max = EEPROM.read(4) != 0; // 非零值转换为 true，零值转换为 false
		cmap_now_choose = EEPROM.read(5);  // 如果读取的值不在 0 到 5 之间，设置为默认值
		if (cmap_now_choose>5){cmap_now_choose = COLORMAP_TURBO;}
	}
	load_colormap(cmap_now_choose);
}

#endif