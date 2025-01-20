#ifndef DWAR_H
#define DWAR_H

#include <Arduino.h>
#include <FreeRTOS.h>
#include "device/screen.h"
#include "shared_val.h"
#include "probe/heimann_driver.hpp"
#include "BilinearInterpolation.h"
#include "colormap.h"


#define PROB_SCALE 8
unsigned short draw_pixel[PIXEL_PER_COLUMN][PIXEL_PER_ROW] = {0};
int R_colour, G_colour, B_colour; 
float ft_point; // 屏幕光标点的温度值
const int biox = 30;
const int bioy = 28;
const int lines = 3;  // 一次渲染多少行的像素
uint16_t  lineBuffer[biox * PROB_SCALE * lines]; // Toggle buffer for lines
uint16_t  dmaBuffer1[biox * PROB_SCALE * lines]; // Toggle buffer for lines
uint16_t  dmaBuffer2[biox * PROB_SCALE * lines]; // Toggle buffer for lines
uint16_t* dmaBufferPtr = dmaBuffer1;
bool dmaBufferSel = 0;

// void getColour(int j)
//    {
//     if (j >= 0 && j < 30)
//        {
//         R_colour = 0;
//         G_colour = 0;
//         B_colour = 20 + 4 * j;
//        }
    
//     if (j >= 30 && j < 60)
//        {
//         R_colour = 4 * (j - 30);
//         G_colour = 0;
//         B_colour = 140 - 2 * (j - 30);
//        }

//     if (j >= 60 && j < 90)
//        {
//         R_colour = 120 + 4 * (j - 60);
//         G_colour = 0;
//         B_colour = 80 - 2 * (j - 60);
//        }

//     if (j >= 90 && j < 120)
//        {
//         R_colour = 255;
//         G_colour = 0 + 2 * (j - 90);
//         B_colour = 10 - (j - 90) / 3;
//        }

//     if (j >= 120 && j < 150)
//        {
//         R_colour = 255;
//         G_colour = 60 + 175 * (j - 120) / 30;
//         B_colour = 0;
//        }

//     if (j >= 150 && j <= 180)
//        {
//         R_colour = 255;
//         G_colour = 235 + (j - 150) * 20 / 30;
//         B_colour = 0 + 85 * (j - 150) / 10;
//        }
// }

// 绘制十字
inline void draw_cross(int x, int y, int len){
   tft.drawLine(x - len/2, y, x + len/2, y, tft.color565(255, 255, 255));
   tft.drawLine(x, y-len/2, x, y+len/2,  tft.color565(255, 255, 255));

   tft.drawLine(x - len/4, y, x + len/4, y, tft.color565(0, 0, 0));
   tft.drawLine(x, y-len/4, x, y+len/4,  tft.color565(0, 0, 0));
}

// 点测温功能
inline void show_local_temp(float num, int x, int y, int cursor_size){
   tft.setRotation(1);
   draw_cross(x, y, 8);
   static short temp_xy;
   static int shift_x, shift_y;
   if (x<140){shift_x=10;} else {shift_x=-60;}
   if (y<120){shift_y=10;} else {shift_y=-20;}
   tft.setTextSize(cursor_size);
   tft.setCursor(x+shift_x, y+shift_y);
   tft.printf("%.2f", num);
}  

// 点测温功能
inline void show_local_temp(float num, int x, int y){
   show_local_temp(num, x, y, 2);
}  

// 判断应该在什么时候渲染光标（光标位置在当前渲染行数-40行的时候）
// 这么做是为了让光标别闪
inline void insert_temp_cursor(int y){
   static int trig_line;
   tft.setRotation(1);
   trig_line = test_point[0] + 80;
   if (trig_line>215){trig_line = 215;}
   if (y==trig_line){
      if (flag_show_cursor==true) {show_local_temp(ft_point, test_point[0], test_point[1]);}
   }
   tft.setRotation(0);
}


inline void draw_float_num(int x, int y, float num){
      // 定义两个字符串来存储小数点前后的部分
  char integerPart[5];
  char fractionalPart[5];
    // 使用 sprintf 格式化字符串
  sprintf(integerPart, "%d", (int)num); // 获取整数部分
  sprintf(fractionalPart, "%02d", (int)((num - (int)num) * 100)); // 获取小数部分
    tft.setCursor(x, y);
    tft.printf("%s.", integerPart);
    tft.setCursor(x+5, y+10);
    tft.printf("%s", fractionalPart);
}

inline void draw_percent_num(int x, int y, uint8_t num){
      // 定义两个字符串来存储小数点前后的部分
    tft.setCursor(x, y);
    tft.printf("%d%%", num);

}


void draw(){
      static int value;
      static int now_y = 0;

      if(use_upsample){
      tft.setRotation(0);
      tft.startWrite();
      for(int y=0; y<32 * PROB_SCALE; y++){ 
         for(int x=0; x<30 * PROB_SCALE; x++){
            value = bio_linear_interpolation(x, y, draw_pixel);
            lineBuffer[x + now_y * biox * PROB_SCALE] = colormap[value];
         }
         now_y ++;
         if(now_y==lines){
            if (dmaBufferSel) dmaBufferPtr = dmaBuffer2;
            else dmaBufferPtr = dmaBuffer1;
            dmaBufferSel = !dmaBufferSel; // Toggle buffer selection
            tft.pushImageDMA(0, y-now_y, biox*PROB_SCALE, lines, lineBuffer, dmaBufferPtr);
            now_y = 0;
         }
         insert_temp_cursor(y);
      }
      if(now_y!=0){
         if (dmaBufferSel) dmaBufferPtr = dmaBuffer2;
         else dmaBufferPtr = dmaBuffer1;
         dmaBufferSel = !dmaBufferSel; // Toggle buffer selection
         tft.pushImageDMA(0, 32*PROB_SCALE-1-now_y, biox*PROB_SCALE, now_y, lineBuffer, dmaBufferPtr);
         now_y = 0;
      }
      tft.endWrite(); 
   }else{
    static uint16_t c565;
    tft.setRotation(0);
    for (int i = 0 ; i < 30 ; i++){
    for (int j = 0; j < 32; j++){
         c565 = colormap[(int)draw_pixel[i][j]];
         // getColour((int)draw_pixel[i][j]);
         tft.fillRect(i*PROB_SCALE, j*PROB_SCALE, PROB_SCALE, PROB_SCALE, c565);  
    }
    }
   }
}

// 用来处理画面被暂停时的热成像图层的渲染工作
void freeze_handeler(){
   // 仅拍照模式下，位于第一屏时会启用这个功能
   if (flag_clear_cursor) {draw(); flag_clear_cursor=false;} // 通过重新渲染一张画面来清除光标
   if (flag_show_cursor) {
      show_local_temp(ft_point, test_point[0], test_point[1]);
   } // 每次点击都渲染光标位置
}


// 探头准备期间的渲染管线
void preparing_loop(){
   tft.setRotation(1);
   if (prob_status == PROB_CONNECTING){
      tft.setCursor(20, 200);
      tft.setTextColor(TFT_WHITE, TFT_BLACK, true);
      tft.setTextSize(1);
      tft.fillRect(20, 200, 240, 30, TFT_BLACK); 
      tft.printf("Triying to connect to HTPAd");
      tft.setCursor(20, 210);
      tft.printf("address: %d\n", SENSOR_ADDRESS);
      delay(10);
   }else if(prob_status == PROB_INITIALIZING){
      tft.setCursor(20, 200);
      tft.setTextColor(TFT_WHITE, TFT_BLACK, true);
      tft.setTextSize(1);
      tft.fillRect(20, 200, 240, 30, TFT_BLACK); 
      tft.printf("HTPAd is ready, initializing...\n");
      delay(10);
   }else if(prob_status == PROB_PREPARING){
      tft.setCursor(20, 200);
      tft.setTextColor(TFT_WHITE, TFT_BLACK, true);
      tft.setTextSize(1);
      tft.fillRect(20, 200, 240, 30, TFT_BLACK); 
      tft.printf("HTPAd initializing... \n"); 
      delay(10);
   }
}

// 探头准备期间的渲染管线
void refresh_status(){
   tft.setCursor(20, 200);
   tft.setTextColor(TFT_WHITE, TFT_BLACK, true);
   tft.setTextSize(1);
   tft.fillRect(20, 200, 240, 30, TFT_BLACK); 
   tft.printf("prepering touch panel...");
   delay(10);
}

// 处理tft_espi渲染管线
void screen_loop(){
   if(!flag_in_photo_mode){
      float ft_max = float(T_max) / 10 - 273.15;
      float ft_min = float(T_min) / 10 - 273.15;
      unsigned short value;
      while (prob_lock == true) {delay(5);}
      pix_cp_lock = true;
      ft_point = (float)(data_pixel[test_point[0] / PROB_SCALE][(test_point[1] / PROB_SCALE)+2] / 10) - 273.15;
      for (int i = 0; i < 32; i++) {
      for (int j = 0; j < 32; j++) {
         // 拷贝温度信息, 并提前映射到色彩空间中
         value = (180 * (data_pixel[i][j] - T_min) / (T_max - T_min));
         if (value < 180) {
         draw_pixel[i][j] = value;
         }
      }
      }
      pix_cp_lock = false;
      while (cmap_loading_lock == true) {delay(1);} // 拷贝温度信息, 并提前映射到色彩空间中
      draw();
      tft.setRotation(1);
      tft.setTextSize(1);
      tft.setCursor(258, 32);
      tft.setTextColor(TFT_RED);
      tft.printf("max");

      tft.setTextColor(TFT_BLUE);
      tft.setCursor(258, 110);
      tft.printf("min");
      
      tft.setTextColor(TFT_GREEN);
      tft.setCursor(258, 195);
      tft.printf("Bat");
      tft.setTextColor(TFT_WHITE, TFT_BLACK, true);
      draw_float_num(258, 47, ft_max);
      draw_float_num(258, 125, ft_min);
      draw_percent_num(258, 210, vbat_percent);
      if (flag_show_cursor==true) {show_local_temp(ft_point, test_point[0], test_point[1]);}
   }else{
      freeze_handeler();
   }

}



#endif