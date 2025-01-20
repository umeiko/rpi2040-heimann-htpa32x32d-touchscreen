#ifndef SCREEN_H
#define SCREEN_H


#include "../logo_jpg.h"
#include <Arduino.h>
#include <TFT_eSPI.h> 
#include <TJpg_Decoder.h>
#include "../shared_val.h"

TFT_eSPI tft = TFT_eSPI();  


#define SCREEN_BL_PIN 4
#define SCREEN_VDD 5
#define SCREEN_ROTATION 1
#define ROTATE 1

// 平滑的亮起
void smooth_on(){
   analogWrite(SCREEN_BL_PIN, 0);
   analogWriteFreq(10000);
   for(int i=0; i<brightness; i++){
      analogWrite(SCREEN_BL_PIN, i);
      delay(2);
   }
}

// 平滑的熄灭
void smooth_off(){
   analogWrite(SCREEN_BL_PIN, brightness);
   analogWriteFreq(10000);
   for(int i=brightness; i>3; i--){
      analogWrite(SCREEN_BL_PIN, i);
      delay(2);
   }
}

// 背光调节,会限制输入亮度在正确范围内
void set_brightness(int _brightness){
   if (_brightness < 255 && _brightness > 5){
      analogWriteFreq(10000);
      analogWrite(SCREEN_BL_PIN, _brightness);
      brightness = _brightness;
   }else if(_brightness >= 255){analogWrite(SCREEN_BL_PIN, 255); brightness=255;
   }else if(_brightness <= 5)   {analogWrite(SCREEN_BL_PIN, 5); brightness=5;
   }
}

uint16_t* logoBuffer1 = nullptr; // Toggle buffer for 16*16 MCU block
uint16_t* logoBuffer2 = nullptr; // Toggle buffer for 16*16 MCU block
uint16_t* logoBufferPtr = logoBuffer1;
bool logoBufferSel = 0;
bool tft_output(int16_t x, int16_t y, uint16_t w, uint16_t h, uint16_t* bitmap){
   // Stop further decoding as image is running off bottom of screen
  if ( y >= tft.height() ) return 0;
  if (logoBufferSel) logoBufferPtr = logoBuffer2;
  else logoBufferPtr = logoBuffer1;
  logoBufferSel = !logoBufferSel; // Toggle buffer selection
  //  pushImageDMA() will clip the image block at screen boundaries before initiating DMA
  tft.pushImageDMA(x, y, w, h, bitmap, logoBufferPtr); // Initiate DMA - blocking only if last DMA is not complete
  // The DMA transfer of image block to the TFT is now in progress...
  // Return 1 to decode next block.
  return 1;
}


void render_logo(){
   // 动态分配内存
    logoBuffer1 = (uint16_t*)malloc(16 * 16 * sizeof(uint16_t));
    logoBuffer2 = (uint16_t*)malloc(16 * 16 * sizeof(uint16_t));

    if (logoBuffer1 == nullptr || logoBuffer2 == nullptr) {
        // 内存分配失败，处理错误
        if (logoBuffer1) free(logoBuffer1);
        if (logoBuffer2) free(logoBuffer2);
        return;
    }
   tft.fillScreen(TFT_BLACK);
   TJpgDec.setJpgScale(1);
   TJpgDec.setCallback(tft_output);
   uint16_t w = 0, h = 0;
   TJpgDec.getJpgSize(&w, &h, logo, sizeof(logo));
   tft.startWrite();
   TJpgDec.drawJpg(0, 0, logo, sizeof(logo));
   tft.endWrite();
   // 释放内存
   free(logoBuffer1);
   free(logoBuffer2);
   logoBuffer1 = nullptr;
   logoBuffer2 = nullptr;
}

// 开启屏幕
void screen_setup(){
   pinMode(SCREEN_BL_PIN, OUTPUT);
   digitalWrite(SCREEN_BL_PIN, LOW);
   pinMode(SCREEN_VDD, OUTPUT);
   digitalWrite(SCREEN_VDD, LOW);
   tft.init();
   tft.setRotation( ROTATE ); /* Landscape orientation, flipped */
   tft.setSwapBytes(true);
   tft.initDMA();
   render_logo();
   delay(300);
   smooth_on();
   delay(500);
   // tft.fillScreen(TFT_BLACK);
}

#endif // SCREEN_H