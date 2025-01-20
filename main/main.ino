
#include <Arduino.h>
// #include <Wire.h>
#include <FreeRTOS.h>
#include <EEPROM.h>
#include "src/wire.h"
#include "src/device/screen.h"
#include "src/device/touch.h"
#include "src/device/button.h"
#include "src/device/bat.h"
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "src/probe/heimann_driver.hpp"
#include "src/software_timer.h"
#include "src/draw.h"
#include "src/colormap.h"
#include "src/eeprom_manage.h"


void setup() {
  Serial.begin(115200);
  load_settings();
  sensor_power_on();
  delay(1000);
  sensor_init();

  for(int i=0;i<55;i++){
    sensor_loop();
    delay(10);
  }
  prob_status = PROB_READY;
  flag_sensor_ok = true;
}


void loop() {
  sensor_loop();
}

void setup1() {
  screen_setup();
  while (flag_sensor_ok==false) {preparing_loop();delay(5);}
  refresh_status();
  touch_setup();
  smooth_off();
  tft.fillScreen(TFT_BLACK);
  screen_loop();
  smooth_on();
}

void loop1() {
  screen_loop();
  button_loop();
  bootsel_loop();
  touch_loop();
  bat_loop();
  delay(8);
}