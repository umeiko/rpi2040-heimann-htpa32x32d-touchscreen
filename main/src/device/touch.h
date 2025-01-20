#ifndef _CST816T_H
#define _CST816T_H

#include <stdint.h>
#include <stdbool.h>
#include <FreeRTOS.h>
#include <task.h>
#include <Arduino.h>
#include "../shared_val.h"
#include "../colormap.h"
#include "../eeprom_manage.h"
#include "screen.h"

#define TOUCH_SDA 2
#define TOUCH_SCL 3

#define PinNotUsed 254
#define TOUCH_RST PinNotUsed
#define ROTATE 1
#define TOUCH_LONG_PUSH_T 200


#define TouchWidth 250
#define TouchHeight 300

#define I2C_ADDR_CST816T 0x15

// 触摸旋转方向
typedef enum {
    Rotation_0 = 0, 
    Rotation_1, 
    Rotation_2,
    Rotation_3, 
} TouchRotation; 

typedef enum :uint8_t {
    None = 0x00,
    SlideDown = 0x01,
    SlideUp = 0x02,
    SlideLeft = 0x03,
    SlideRight = 0x04,
    SingleTap = 0x05,
    DoubleTap = 0x0B,
    LongPress = 0x0C
}Gestures;

typedef struct{
    uint16_t x = 0;
    uint16_t y = 0;
    Gestures gesture = Gestures::None;
    bool touching = false;
    bool isValid = false;
}TouchInfos;

struct data_struct {
    uint16_t x = 0;
    uint16_t y = 0;
    Gestures gesture = Gestures::None;
    bool touching = false;
    bool isValid = false;
};


/**************************************************************************/
/*!
    @brief  CST816T I2C CTP controller driver
*/
/**************************************************************************/
class CST816T 
{
public:
    CST816T();
    CST816T(uint8_t rst_n, uint8_t int_n);
    CST816T(uint8_t sda, uint8_t scl, uint8_t rst_n, uint8_t int_n); 
    data_struct tp;
    virtual ~CST816T(); 

    void begin(void);
    void setRotation(TouchRotation Rotation);
    int update();

    // Scan Function
    TouchInfos GetTouchInfo(void);

private: 
    int sda = -1; 
    int scl = -1; 
    uint8_t int_n = -1; 
    uint8_t rst_n = -1; 
    uint8_t touch_rotation = Rotation_0;

    // Unused/Unavailable commented out
    static constexpr uint8_t gestureIndex = 1;
    static constexpr uint8_t touchPointNumIndex = 2;
    //static constexpr uint8_t touchEventIndex = 3;
    static constexpr uint8_t touchXHighIndex = 3;
    static constexpr uint8_t touchXLowIndex = 4;
    //static constexpr uint8_t touchIdIndex = 5;
    static constexpr uint8_t touchYHighIndex = 5;
    static constexpr uint8_t touchYLowIndex = 6;
    //static constexpr uint8_t touchStep = 6;
    //static constexpr uint8_t touchXYIndex = 7;
    //static constexpr uint8_t touchMiscIndex = 8;

    uint8_t readByte(uint8_t addr); 
    void writeByte(uint8_t addr, uint8_t data); 
}; 
CST816T::CST816T()
{
}

CST816T::CST816T(uint8_t rst_n_pin, uint8_t int_n_pin)
{
    rst_n = rst_n_pin;
    int_n = int_n_pin;
}

CST816T::CST816T(uint8_t sda_pin, uint8_t scl_pin, uint8_t rst_n_pin, uint8_t int_n_pin)
{
    sda = sda_pin;
    scl = scl_pin;
    rst_n = rst_n_pin;
    int_n = int_n_pin;
}

CST816T::~CST816T() {
}

void CST816T::begin(void) {
    // Initialize I2C
    if(sda != -1 && scl != -1) {
        Wire1.setSDA(sda);
        Wire1.setSCL(scl);
        Wire1.begin(); 
    }
    else {
        Wire1.begin(); 
        Serial.println("I2C initialized (SDA, SCL)1");
    }

    // Int Pin Configuration
    if(int_n != -1) {
        pinMode(int_n, INPUT); 
    }
    // Reset Pin Configuration
    if(rst_n != -1) {
        pinMode(rst_n, OUTPUT); 
        digitalWrite(rst_n, LOW); 
        delay(10); 
        digitalWrite(rst_n, HIGH); 
        delay(500); 
    }
    readByte(0x15);
    delay(1);
    readByte(0xa7);
    delay(1);
    writeByte(0xEC, 0b00000101);
    delay(1);
    writeByte(0xFA, 0b01110000);
    delay(1);
}

/**
 * @设置旋转方向,默认为Rotation_0
 * @Rotation：方向 0~3
*/
void CST816T::setRotation(TouchRotation Rotation)
{
    switch (Rotation) {
    case Rotation_0:
      touch_rotation = Rotation_0;
      break;
    case Rotation_1:
      touch_rotation = Rotation_1;
      break;
    case Rotation_2:
      touch_rotation = Rotation_2;
      break;
    case Rotation_3:
      touch_rotation = Rotation_3;
      break;
    }
}

//coordinate diagram（FPC downwards）
TouchInfos CST816T::GetTouchInfo(void){
    // Serial.println("GetTouchInfo");
    byte error;
    TouchInfos info;
    uint8_t touchData[7];
    uint8_t rdDataCount;
    // 唤醒
    // readByte(0x15);
    // readByte(0xa7);
    uint8_t i = 0;
    long startTime = millis();
    do {
        Wire1.beginTransmission(I2C_ADDR_CST816T); 
        Wire1.write(0); 
        error = Wire1.endTransmission(false); // Restart
        if (error != 0) {
            info.isValid = false;
            return info;
        }
        rdDataCount = Wire1.requestFrom(I2C_ADDR_CST816T, sizeof(touchData)); 
        // Serial.printf("读取中%d  %d\n", i, rdDataCount);
        if(millis() - startTime > 1) {
            info.isValid = false;
            return info;
        }
    } while(rdDataCount == 0); 
    i = 0;
    while(Wire1.available()) {
        touchData[i] = Wire1.read();
        // Serial.printf("%d %02X \n", i, touchData[i]);
        i++;
        if(i >= sizeof(touchData)) {
            break;
        }
    }
    // Serial.println("GetTouchInfo end");

    // This can only be 0 or 1
    uint8_t nbTouchPoints = touchData[touchPointNumIndex] & 0x0f;
    uint8_t xHigh = touchData[touchXHighIndex] & 0x0f;
    uint8_t xLow = touchData[touchXLowIndex];
    uint16_t x = (xHigh << 8) | xLow;
    uint8_t yHigh = touchData[touchYHighIndex] & 0x0f;
    uint8_t yLow = touchData[touchYLowIndex];
    uint16_t y = (yHigh << 8) | yLow;
    Gestures gesture = static_cast<Gestures>(touchData[gestureIndex]);

    // Validity check
    if(x >= TouchWidth || y >= TouchHeight ||
        (gesture != Gestures::None &&
        gesture != Gestures::SlideDown &&
        gesture != Gestures::SlideUp &&
        gesture != Gestures::SlideLeft &&
        gesture != Gestures::SlideRight &&
        gesture != Gestures::SingleTap &&
        gesture != Gestures::DoubleTap &&
        gesture != Gestures::LongPress)) {
        info.isValid = false;
        return info;
    }

    info.x = x;
    info.y = y;
    info.touching = (nbTouchPoints > 0);
    info.gesture = gesture;
    info.isValid = true;
    return info;
}

//coordinate diagram（FPC downwards）
int CST816T::update(void){
    static TouchInfos data = GetTouchInfo();
    data = GetTouchInfo();
    if (data.isValid){
        tp.x = data.x;
        tp.y = data.y;
        tp.touching = data.touching;
        tp.gesture = data.gesture;
    }
    return 0;
}

// Private Function
uint8_t CST816T::readByte(uint8_t addr) {
    uint8_t rdData; 
    uint8_t rdDataCount;
    uint8_t i;
    do {
        Wire1.beginTransmission(I2C_ADDR_CST816T); 
        Wire1.write(addr); 
        Wire1.endTransmission(false); // Restart
        rdDataCount = Wire1.requestFrom(I2C_ADDR_CST816T, 1);
        i ++; 
    } while(rdDataCount == 0 && i<250); 
    
    while(Wire1.available()) {
        rdData = Wire1.read(); 
    }
    return rdData; 
}

void CST816T::writeByte(uint8_t addr, uint8_t data) {
    Wire1.beginTransmission(I2C_ADDR_CST816T); 
    Wire1.write(addr); 
    Wire1.write(data); 
    Wire1.endTransmission(); 
}

CST816T touch(TOUCH_SDA, TOUCH_SCL, TOUCH_RST, PinNotUsed);



void gesture_handler(uint8_t gesture){
    static uint8_t last_gesture = Gestures::None;
    if (last_gesture != gesture){
        if (gesture == Gestures::SlideUp){
            priv_cmap();
        }else if (gesture == Gestures::SlideDown){
            next_cmap();
        }
        eeprom_save_config();
    }
    last_gesture = gesture;
}


void touch_setup(void){
    touch.begin();
}


void touch_loop(void){
   static uint16_t x, y;
   static uint16_t start_x, start_y;
   static int diffx, diffy;
   static bool long_pushed = false;
   static unsigned long touch_pushed_start_time =  0;
   static bool touched = false;
   static int start_br = 100;
   
   touch.update();
    if( touch.tp.touching )
      {
         x= touch.tp.y;
         y = 240 - touch.tp.x;
         gesture_handler(touch.tp.gesture);
        //  Serial.println(touch.tp.gesture);
         if (touch_pushed_start_time == 0){touch_pushed_start_time = millis();}
         if (touched==false){start_x = x;  start_y = y; diffy=0; diffx=0;}  // 下降沿
         if (millis() - touch_pushed_start_time >= TOUCH_LONG_PUSH_T){
            long_pushed = true;
            diffx= start_x-x;
            diffy= start_y-y;
            set_brightness(start_br+diffy*5);
            }else{ // 短按的中间
               
            }
      }else{
         touch_pushed_start_time = millis();
         if (touched==true){  // 上升沿
            if (start_br == brightness){
               if (x < 224){flag_show_cursor=true; test_point[0] = x; test_point[1] = y;}
            }
            if (long_pushed==false){  // 短按时
               if (x < 224){flag_show_cursor=true; test_point[0] = x; test_point[1] = y;}
            }
            start_br = brightness;
            long_pushed = false;  // 上升沿将长按检测标识符进行复位
         }  
      }
      touched = touch.tp.touching;
      delay(5);
}


void touch_task(void *ptr){
    for(;;){
        touch_loop();
    }
}


void touch_task_startup(){
  xTaskCreate(touch_task, "touch_task", 2048, NULL, 1, NULL);
}


#endif