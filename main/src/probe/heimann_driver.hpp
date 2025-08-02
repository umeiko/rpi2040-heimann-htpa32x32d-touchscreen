/*** PROGRAMM INFO***************************************************************************************
  source code for HTPAd RP2040 driver
  name:           RP2040_HTPAd_32x32
  version/date:   2.2-rp2040branch / 01 Jan 2025
  programmer:     Umeko (umeko@stu.xmu.edu.cn)
*********************************************************************************************************/
#ifndef HEIMANN_DRIVER_HPP
#define HEIMANN_DRIVER_HPP
/*** PROGRAMM INFO***************************************************************************************
  source code for ESP32 and HTPAd Application Shield
  name:           ESP32_HTPAd_32x32.ino
  version/date:   2.2 / 20 Dec 2022
  programmer:     Heimann Sensor GmbH / written by Dennis Pauer (pauer@heimannsensor.com)
*********************************************************************************************************/



/*** MODES **********************************************************************************************
  The source code includes three ways to interact with the sensor:
  - via WIFI you can stream thermal images in our GUI (#define WIFIMODE)
  - via the SERIAL monitor you can observe the sensor data as text output (#define SERIALMODE)
  - via ACCESSPOINT the ESP32 creates the wifi network. You have to connect your computer to this network
    to stream thermal images in the GUI (#define ACCESSPOINT)
  Both modes are contain in the same code and you can activate one or both by activate the matching define.
*********************************************************************************************************/
//#define WIFIMODE

// #define ACCESSPOINT // automatically diablead if WIFIMODE is active
#include <Arduino.h>

/*** NETWORK INFORMATION ********************************************************************************
  If you want to use the WIFI function, you have to change ssid and pass.
*********************************************************************************************************/
// #include "../my_wire/Wire.h"
// #define USE_SDK

#ifdef USE_SDK
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#else
// #include <Wire.h>
#include "../wire.h"
#endif

#include "def.h"
#include "../software_timer.h"
#include "../shared_val.h"

#define I2C_BUFFER_LENGTH 300
#define MLX_VDD  11
#define MLX_SDA  12
#define MLX_SCL  13
//-----------------------------------------
// WIFI
// int status = WL_IDLE_STATUS;
// int keyIndex = 0;
// unsigned int localPort = 30444;
// char packetBuffer[256];
// char  ReplyBuffer[] = "acknowledged";
// uint8_t mac[6];
// uint8_t ip_partner[4];
// uint8_t device_bind;
// signed short WIFIstrength;
// WiFiUDP udp;
// unsigned char wifi_on = 0;
//-----------------------------------------

// STRUCT WITH ALL SENSOR CHARACTERISTICS
struct characteristics {
  unsigned short NumberOfPixel;
  unsigned char NumberOfBlocks;
  unsigned char RowPerBlock;
  unsigned short PixelPerBlock;
  unsigned short PixelPerColumn;
  unsigned short PixelPerRow;
  unsigned char AllowedDeadPix;
  unsigned short TableNumber;
  unsigned short TableOffset;
  unsigned char PTATPos;
  unsigned char VDDPos;
  unsigned char PTATVDDSwitch;
  unsigned char CyclopsActive;
  unsigned char CyclopsPos;
  unsigned char DataPos;
};

characteristics DevConst = {
  NUMBER_OF_PIXEL,
  NUMBER_OF_BLOCKS,
  ROW_PER_BLOCK,
  PIXEL_PER_BLOCK,
  PIXEL_PER_COLUMN,
  PIXEL_PER_ROW,
  ALLOWED_DEADPIX,
  TABLENUMBER,
  TABLEOFFSET,
  PTAT_POS,
  VDD_POS,
  PTAT_VDD_SWITCH,
  ATC_ACTIVE,
  ATC_POS,
  DATA_POS,
};

//-----------------------------------------
// EEPROM DATA
unsigned char mbit_calib, bias_calib, clk_calib, bpa_calib, pu_calib, mbit_user, bias_user, clk_user, bpa_user, pu_user;
unsigned char nrofdefpix, gradscale, vddscgrad, vddscoff, epsilon, lastepsilon, arraytype;
unsigned char deadpixmask[ALLOWED_DEADPIX];
signed char globaloff;
signed short thgrad[PIXEL_PER_COLUMN][PIXEL_PER_ROW];
unsigned short tablenumber, vddth1, vddth2, ptatth1, ptatth2, ptatgr, globalgain;
unsigned short deadpixadr[ALLOWED_DEADPIX * 2];
signed short thoffset[PIXEL_PER_COLUMN][PIXEL_PER_ROW];
signed short vddcompgrad[ROW_PER_BLOCK * 2][PIXEL_PER_ROW];
signed short vddcompoff[ROW_PER_BLOCK * 2][PIXEL_PER_ROW];
unsigned long id, ptatoff;
float ptatgr_float, ptatoff_float, pixcmin, pixcmax, bw;
unsigned long *pixc2_0; // start address of the allocated heap memory
unsigned long *pixc2; // increasing address pointer

//-----------------------------------------
// SENSOR DATA
unsigned short data_pixel[PIXEL_PER_COLUMN][PIXEL_PER_ROW];
unsigned short* data_pixel_flatten = (unsigned short*)data_pixel;
unsigned char RAMoutput[2 * NUMBER_OF_BLOCKS + 2][BLOCK_LENGTH];
bool flag_min_max_initaled = false;  // 需要重新计算极值的标志位
uint8_t x_max, y_max, x_min, y_min;
/*
  RAMoutput is the place where the raw values are saved

  example, order for 80x64:
  RAMoutput[0][]... data from block 0 top
  RAMoutput[1][]... data from block 1 top
  RAMoutput[2][]... data from block 2 top
  RAMoutput[3][]... data from block 3 top
  RAMutput[4][]... electrical offset top
  RAMoutput[5][]... electrical offset bottom
  RAMoutput[6][]... data from block 3 bottom
  RAMoutput[7][]... data from block 2 bottom
  RAMoutput[8][]... data from block 1 bottom
  RAMoutput[9][]... data from block 0 bottom

*/
unsigned short eloffset[ROW_PER_BLOCK * 2][PIXEL_PER_ROW];
unsigned char statusreg;
unsigned short Ta, ptat_av_uint16, vdd_av_uint16;


// BUFFER for PTAT,VDD and elOffsets
// PTAT:
uint16_t ptat_buffer[PTAT_BUFFER_SIZE];
uint16_t ptat_buffer_average;
uint8_t use_ptat_buffer = 0;
uint8_t ptat_i = 0;
uint8_t PTATok = 0;
// VDD:
uint16_t vdd_buffer[VDD_BUFFER_SIZE];
uint16_t vdd_buffer_average;
uint8_t use_vdd_buffer = 0;
uint8_t vdd_i = 0;
// electrical offsets:
uint8_t use_eloffsets_buffer = 0;
uint8_t eloffsets_i = 0;
uint8_t new_offsets = 1;

// PROGRAMM CONTROL
bool switch_ptat_vdd = 0;
unsigned char adr_offset = 0x00;
unsigned char send_data = 0;
unsigned short picnum = 0;
unsigned char state = 0;
unsigned char read_block_num = START_WITH_BLOCK; // start with electrical offset
unsigned char read_eloffset_next_pic = 0;
unsigned char gui_mode = 0;
unsigned char wait_pic = 0;
bool ReadingRoutineEnable = 1;

// OTHER
uint32_t gradscale_div;
uint32_t vddscgrad_div;
uint32_t vddscoff_div;
int vddcompgrad_n;
int vddcompoff_n;
unsigned long t1;
unsigned char print_state = 0;


unsigned NewDataAvailable = 1;

unsigned short timert;
char serial_input = 'm';


// read new sensor data
void ISR(void){
  if (ReadingRoutineEnable) {
    /*
       HINT:
       this interrupt service routine set a flag called NedDataAvailable.
       This flag will be checked in the main loop. If this flag is set, the main loop will call
       the function to read the new sensor data and reset this flag and the timer. I go that way
       because the ESP32 cannot read I2C data directly in the ISR. If your µC can handle I2C in
       an interrupt,please read the new sensor volatges direclty in the ISR.
    */
    NewDataAvailable = 1;
  }
  // Serial.println("ISR triggered");
}


/********************************************************************
 ********************************************************************
    - - - PART 2: HTPAd FUNCTIONS - - -
    calcPixC()
    calculate_pixel_temp()
    pixel_masking()
    readblockinterrupt()
    read_eeprom()
    read_EEPROM_byte( uint8_t addr)
    read_sensor_register()
    sort_data()
    write_calibration_settings_to_sensor()
    write_sensor_byte( unsigned char addr, unsigned char input)
 ********************************************************************
 ********************************************************************/

/********************************************************************
   Function:        void read_EEPROM_byte(unsigned int eeaddress )
   Description:     read eeprom register as 8
   Dependencies:    register address (address)
 *******************************************************************/
byte read_EEPROM_byte(unsigned int address ) {
  byte rdata = 0xFF;
  Wire.beginTransmission(EEPROM_ADDRESS);
  Wire.write((int)(address >> 8));   // MSB
  Wire.write((int)(address & 0xFF)); // LSB
  Wire.endTransmission();
  Wire.requestFrom(EEPROM_ADDRESS, 1);
  if (Wire.available())
    rdata = Wire.read();
  return rdata;
}


/********************************************************************
   Function:        void write_sensor_byte( unsigned short addr)
   Description:     write to sensor register
   Dependencies:    register address (addr),
                    number of bytes (n)
 *******************************************************************/
byte write_sensor_byte(uint8_t deviceaddress, uint8_t registeraddress, uint8_t input) {
  // Serial.printf("write_sensor_byte: deviceaddress: %d, registeraddress: %d, input: %d\n", deviceaddress, registeraddress, input);
  Wire.beginTransmission(deviceaddress);
  Wire.write(registeraddress);
  Wire.write(input);
  byte result = Wire.endTransmission();
  if (result != 0) {
      Serial.printf("write_sensor_byte failed with error code: %d\n", result);
  }
  return 0;
}

/********************************************************************
   Function:        void read_sensor_register( uint16_t addr, uint8_t *dest, uint16_t n)
   Description:     read sensor register
 *******************************************************************/

void read_sensor_register(uint16_t addr, uint8_t *dest, uint16_t n)
{
  static int nBytes;
  //Setup a series of chunked I2C_BUFFER_LENGTH byte reads
  Wire.beginTransmission(SENSOR_ADDRESS);
  Wire.write((uint8_t)addr); //MSB
  if (Wire.endTransmission(false) != 0) //Do not release bus
  {
    Serial.println("No ack read");
    return; //Sensor did not ACK
  }
    nBytes = Wire.requestFrom((uint16_t)SENSOR_ADDRESS, n, true);
    if (nBytes != n)
    {
      Serial.printf("need %d bytes, but got %d bytes\n", n, nBytes);
    }
    while (Wire.available()) {
      *dest++  = Wire.read();
    }
  return; //Success
}

/********************************************************************
   Function:        void pixel_masking()
   Description:     repair dead pixel by using the average of the neighbors
 *******************************************************************/
void pixel_masking() {


  uint8_t number_neighbours[ALLOWED_DEADPIX];
  uint32_t temp_defpix[ALLOWED_DEADPIX];
  for (int i = 0; i < nrofdefpix; i++) {
    number_neighbours[i] = 0;
    temp_defpix[i] = 0;

    // top half
    if (deadpixadr[i] < (unsigned short)(NUMBER_OF_PIXEL / 2)) {

      if ( (deadpixmask[i] & 1 )  == 1) {
        number_neighbours[i]++;
        temp_defpix[i] = temp_defpix[i] + data_pixel[(deadpixadr[i] / PIXEL_PER_ROW) - 1][(deadpixadr[i] % PIXEL_PER_ROW)];
      }

      if ( (deadpixmask[i] & 2 )  == 2 ) {
        number_neighbours[i]++;
        temp_defpix[i] = temp_defpix[i] + data_pixel[(deadpixadr[i] / PIXEL_PER_ROW) - 1][(deadpixadr[i] % PIXEL_PER_ROW) + 1];
      }

      if ( (deadpixmask[i] & 4 )  == 4 ) {
        number_neighbours[i]++;
        temp_defpix[i] = temp_defpix[i] + data_pixel[(deadpixadr[i] / PIXEL_PER_ROW)][(deadpixadr[i] % PIXEL_PER_ROW) + 1];
      }

      if ( (deadpixmask[i] & 8 )  == 8 ) {
        number_neighbours[i]++;
        temp_defpix[i] = temp_defpix[i] + data_pixel[(deadpixadr[i] / PIXEL_PER_ROW) + 1][(deadpixadr[i] % PIXEL_PER_ROW) + 1];
      }

      if ( (deadpixmask[i] & 16 )  == 16 ) {
        number_neighbours[i]++;
        temp_defpix[i] = temp_defpix[i] + data_pixel[(deadpixadr[i] / PIXEL_PER_ROW) + 1][(deadpixadr[i] % PIXEL_PER_ROW)];
      }

      if ( (deadpixmask[i] & 32 )  == 32 ) {
        number_neighbours[i]++;
        temp_defpix[i] = temp_defpix[i] + data_pixel[(deadpixadr[i] / PIXEL_PER_ROW) + 1][(deadpixadr[i] % PIXEL_PER_ROW) - 1];
      }

      if ( (deadpixmask[i] & 64 )  == 64 ) {
        number_neighbours[i]++;
        temp_defpix[i] = temp_defpix[i] + data_pixel[(deadpixadr[i] / PIXEL_PER_ROW)][(deadpixadr[i] % PIXEL_PER_ROW) - 1];
      }

      if ( (deadpixmask[i] & 128 )  == 128 ) {
        number_neighbours[i]++;
        temp_defpix[i] = temp_defpix[i] + data_pixel[(deadpixadr[i] / PIXEL_PER_ROW) - 1][(deadpixadr[i] % PIXEL_PER_ROW) - 1];
      }

    }

    // bottom half
    else {

      if ( (deadpixmask[i] & 1 )  == 1 ) {
        number_neighbours[i]++;
        temp_defpix[i] = temp_defpix[i] + data_pixel[(deadpixadr[i] / PIXEL_PER_ROW) + 1][(deadpixadr[i] % PIXEL_PER_ROW)];
      }

      if ( (deadpixmask[i] & 2 )  == 2 ) {
        number_neighbours[i]++;
        temp_defpix[i] = temp_defpix[i] + data_pixel[(deadpixadr[i] / PIXEL_PER_ROW) + 1][(deadpixadr[i] % PIXEL_PER_ROW) + 1];
      }

      if ( (deadpixmask[i] & 4 )  == 4 ) {
        number_neighbours[i]++;
        temp_defpix[i] = temp_defpix[i] + data_pixel[(deadpixadr[i] / PIXEL_PER_ROW)][(deadpixadr[i] % PIXEL_PER_ROW) + 1];
      }

      if ( (deadpixmask[i] & 8 )  == 8 ) {
        number_neighbours[i]++;
        temp_defpix[i] = temp_defpix[i] + data_pixel[(deadpixadr[i] / PIXEL_PER_ROW) - 1][(deadpixadr[i] % PIXEL_PER_ROW) + 1];
      }

      if ( (deadpixmask[i] & 16 )  == 16 ) {
        number_neighbours[i]++;
        temp_defpix[i] = temp_defpix[i] + data_pixel[(deadpixadr[i] / PIXEL_PER_ROW) - 1][(deadpixadr[i] % PIXEL_PER_ROW)];
      }

      if ( (deadpixmask[i] & 32 )  == 32 ) {
        number_neighbours[i]++;
        temp_defpix[i] = temp_defpix[i] + data_pixel[(deadpixadr[i] / PIXEL_PER_ROW) - 1][(deadpixadr[i] % PIXEL_PER_ROW) - 1];
      }

      if ( (deadpixmask[i] & 64 )  == 64 ) {
        number_neighbours[i]++;
        temp_defpix[i] = temp_defpix[i] + data_pixel[(deadpixadr[i] / PIXEL_PER_ROW)][(deadpixadr[i] % PIXEL_PER_ROW) - 1];
      }

      if ( (deadpixmask[i] & 128 )  == 128 ) {
        number_neighbours[i]++;
        temp_defpix[i] = temp_defpix[i] + data_pixel[(deadpixadr[i] / PIXEL_PER_ROW) + 1][(deadpixadr[i] % PIXEL_PER_ROW) - 1];
      }
    }

    temp_defpix[i] = temp_defpix[i] / number_neighbours[i];
    data_pixel[deadpixadr[i] / PIXEL_PER_ROW][deadpixadr[i] % PIXEL_PER_ROW] = temp_defpix[i];

  }
}


/********************************************************************
   Function:      calcPixC
   Description:   calculates the pixel constants with the unscaled
                  values from EEPROM
 *******************************************************************/
void calcPixC() {

  /* uses the formula from datasheet:

                     PixC_uns[m][n]*(PixCmax-PixCmin)               epsilon   GlobalGain
      PixC[m][n] = ( -------------------------------- + PixCmin ) * ------- * ----------
                                  65535                               100        1000
  */

  double pixcij;
  pixc2 = pixc2_0; // set pointer to start address of the allocated heap

  for (int m = 0; m < DevConst.PixelPerColumn; m++) {
    for (int n = 0; n < DevConst.PixelPerRow; n++) {

      pixcij = (double)pixcmax;
      pixcij -= (double)pixcmin;
      pixcij /= (double)65535.0;
      pixcij *= (double) * pixc2;
      pixcij += (double)pixcmin;
      pixcij /= (double)100.0;
      pixcij *= (double)epsilon;
      pixcij /= (double)10000.0;
      pixcij *= (double)globalgain;
      pixcij += 0.5;

      *pixc2 = (unsigned long)pixcij;
      pixc2++;

    }
  }

  lastepsilon = epsilon;

}


/********************************************************************
   Function:        calculate_pixel_temp()
   Description:     compensate thermal, electrical offset and vdd and multiply sensitivity coeff
                    look for the correct temp in lookup table
 *******************************************************************/
void calculate_pixel_temp() {

  int64_t vij_pixc_and_pcscaleval;
  int64_t pixcij;
  int64_t vdd_calc_steps;
  uint16_t table_row, table_col;
  int32_t vx, vy, ydist, dta;
  signed long pixel;
  pixc2 = pixc2_0; // set pointer to start address of the allocated heap


  /******************************************************************************************************************
    step 0: find column of lookup table
  ******************************************************************************************************************/
  for (int i = 0; i < NROFTAELEMENTS; i++) {
    if (Ta > XTATemps[i]) {
      table_col = i;
    }
  }
  dta = Ta - XTATemps[table_col];
  ydist = (int32_t)ADEQUIDISTANCE;


  flag_min_max_initaled = false;
  for (int m = 0; m < DevConst.PixelPerColumn; m++) {
    for (int n = 0; n < DevConst.PixelPerRow; n++) {

      /******************************************************************************************************************
         step 1: use a variable with bigger data format for the compensation steps
       ******************************************************************************************************************/
      prob_lock = true;
      pixel = (signed long) data_pixel[m][n];
      prob_lock = false;
      /******************************************************************************************************************
         step 2: compensate thermal drifts (see datasheet, chapter: Thermal Offset)
       ******************************************************************************************************************/
      pixel -= (int32_t)(((int32_t)thgrad[m][n] * (int32_t)ptat_av_uint16) / (int32_t)gradscale_div);
      pixel -= (int32_t)thoffset[m][n];

      /******************************************************************************************************************
         step 3: compensate electrical offset (see datasheet, chapter: Electrical Offset)
       ******************************************************************************************************************/
      if (m < DevConst.PixelPerColumn / 2) { // top half
        pixel -= eloffset[m % DevConst.RowPerBlock][n];
      }
      else { // bottom half
        pixel -= eloffset[m % DevConst.RowPerBlock + DevConst.RowPerBlock][n];
      }

      /******************************************************************************************************************
         step 4: compensate vdd (see datasheet, chapter: Vdd Compensation)
       ******************************************************************************************************************/
      // first select VddCompGrad and VddCompOff for pixel m,n:
      if (m < DevConst.PixelPerColumn / 2) {      // top half
        vddcompgrad_n = vddcompgrad[m % DevConst.RowPerBlock][n];
        vddcompoff_n = vddcompoff[m % DevConst.RowPerBlock][n];
      }
      else {       // bottom half
        vddcompgrad_n = vddcompgrad[m % DevConst.RowPerBlock + DevConst.RowPerBlock][n];
        vddcompoff_n = vddcompoff[m % DevConst.RowPerBlock + DevConst.RowPerBlock][n];
      }
      // now do the vdd calculation
      vdd_calc_steps = vddcompgrad_n * ptat_av_uint16;
      vdd_calc_steps = vdd_calc_steps / vddscgrad_div;
      vdd_calc_steps = vdd_calc_steps + vddcompoff_n;
      vdd_calc_steps = vdd_calc_steps * ( vdd_av_uint16 - vddth1 - ((vddth2 - vddth1) / (ptatth2 - ptatth1)) * (ptat_av_uint16  - ptatth1));
      vdd_calc_steps = vdd_calc_steps / vddscoff_div;
      pixel -= vdd_calc_steps;

      /******************************************************************************************************************
         step 5: multiply sensitivity coeff for each pixel (see datasheet, chapter: Object Temperature)
       ******************************************************************************************************************/
      vij_pixc_and_pcscaleval = pixel * (int64_t)PCSCALEVAL;
      pixel =  (int32_t)(vij_pixc_and_pcscaleval / *pixc2);
      pixc2++;
      /******************************************************************************************************************
         step 6: find correct temp for this sensor in lookup table and do a bilinear interpolation (see datasheet, chapter:  Look-up table)
       ******************************************************************************************************************/
      table_row = pixel + TABLEOFFSET;
      table_row = table_row >> ADEXPBITS;
      // bilinear interpolation
      vx = ((((int32_t)TempTable[table_row][table_col + 1] - (int32_t)TempTable[table_row][table_col]) * (int32_t)dta) / (int32_t)TAEQUIDISTANCE) + (int32_t)TempTable[table_row][table_col];
      vy = ((((int32_t)TempTable[table_row + 1][table_col + 1] - (int32_t)TempTable[table_row + 1][table_col]) * (int32_t)dta) / (int32_t)TAEQUIDISTANCE) + (int32_t)TempTable[table_row + 1][table_col];
      pixel = (uint32_t)((vy - vx) * ((int32_t)(pixel + TABLEOFFSET) - (int32_t)YADValues[table_row]) / ydist + (int32_t)vx);

      /******************************************************************************************************************
         step 7: add GlobalOffset (stored as signed char)
       ******************************************************************************************************************/
      pixel += globaloff;

      /******************************************************************************************************************
        step 8: overwrite the uncompensate pixel with the new calculated compensated value
      ******************************************************************************************************************/
      data_pixel[m][n] = (unsigned short)pixel;

      /******************************************************************************************************************
        step 9: find min and max value
      ******************************************************************************************************************/
      
      if (flag_min_max_initaled == false) {
        T_max = data_pixel[m][n];
        T_min = data_pixel[m][n];
        T_avg = data_pixel[m][n];
        flag_min_max_initaled = true;
      }
      if (n<30 && m<30 && n > 2 && m > 2) {
        if(data_pixel[m][n] < T_min) {
          T_min = data_pixel[m][n];
            x_min = n;
            y_min = m;
          }

        if(data_pixel[m][n] > T_max) {
          T_max = data_pixel[m][n];
            x_max = n;
            y_max = m;
          }
        T_avg = T_avg + data_pixel[m][n];
      }
    }
  }
  /******************************************************************************************************************
    step 8: overwrite the uncompensate pixel with the new calculated compensated value
  ******************************************************************************************************************/
  T_avg = T_avg / 729;
  pixel_masking();
}


/********************************************************************
   Function:      calc_timert(uint8_t clk, uint8_t mbit)
   Description:   calculate the duration of the timer which reads the sensor blocks
 *******************************************************************/
word calc_timert(uint8_t clk, uint8_t mbit) {

  float a;
  uint16_t calculated_timer_duration;

  float Fclk_float = 12000000.0 / 63.0 * (float)clk + 1000000.0;    // calc clk in Hz
  a = 32.0 * ((float)pow(2, (unsigned char)(mbit & 0b00001111)) + 4.0) / Fclk_float;

  calculated_timer_duration = (unsigned short)(0.98 * a * 1000000); // c in s | timer_duration in µs
  return calculated_timer_duration;
}



/********************************************************************
   Function:        void readblockinterrupt()
   Description:     read one sensor block and change configuration register to next block
                    (also read electrical offset when read_eloffset_next_pic is set)
 *******************************************************************/
void readblockinterrupt() {
  unsigned char bottomblock;
  ReadingRoutineEnable = 0;
  TimerLib.clearTimer();
  // delay(15);
  // wait for end of conversion bit (~27ms)
  // check EOC bit
  read_sensor_register( STATUS_REGISTER, (uint8_t*)&statusreg, 1);
  // Serial.printf("statusreg: %d\n", statusreg);
  while (bitRead(statusreg, 0) == 0) {
    read_sensor_register( STATUS_REGISTER, (uint8_t*)&statusreg, 1);
    delay(10);
  }
  // get data of top half:
  read_sensor_register( TOP_HALF, (uint8_t*)&RAMoutput[read_block_num], BLOCK_LENGTH);
  bottomblock = (unsigned char)((unsigned char)(NUMBER_OF_BLOCKS + 1) * 2 - read_block_num - 1);
  read_sensor_register( BOTTOM_HALF, (uint8_t*)&RAMoutput[bottomblock], BLOCK_LENGTH);

  read_block_num++;
  // Serial.printf("block %d sampled\n", read_block_num);
  if (read_block_num < NUMBER_OF_BLOCKS) {
    // to start sensor set configuration register to 0x09
    // |    RFU    |   Block   | Start | VDD_MEAS | BLIND | WAKEUP |
    // |  0  |  0  |  x  |  x  |   1   |    0     |   0   |    1   |
    write_sensor_byte(SENSOR_ADDRESS, CONFIGURATION_REGISTER, (unsigned char)(0x09 + (0x10 * read_block_num) + (0x04 * switch_ptat_vdd)));
  }
  else {
    //*******************************************************************
    // all blocks for the current image are sampled, now check if its time
    // to get new electrical offsets and/or for switching PTAT and VDD
    //*******************************************************************
    // Serial.println("all blocks for the current image are sampled");
    if (read_eloffset_next_pic) {
      read_eloffset_next_pic = 0;
      // |    RFU    |   Block   | Start | VDD_MEAS | BLIND | WAKEUP |
      // |  0  |  0  |  0  |  0  |   1   |    0     |   1   |    1   |
      write_sensor_byte(SENSOR_ADDRESS, CONFIGURATION_REGISTER, (unsigned char)(0x0B + (0x04 * switch_ptat_vdd)));
      new_offsets = 1;
    }
    else {
      if (picnum > 1)
        state = 1; // state = 1 means that all required blocks are sampled
      picnum++; // increase the picture counter
      // check if the next sample routine should include electrical offsets
      if ((unsigned char)(picnum % READ_ELOFFSET_EVERYX) == 0)
        read_eloffset_next_pic = 1;
      if (DevConst.PTATVDDSwitch)
        switch_ptat_vdd ^= 1;
      read_block_num = 0;
      // |    RFU    |   Block   | Start | VDD_MEAS | BLIND | WAKEUP |
      // |  0  |  0  |  0  |  0  |   1   |    0     |   0   |    1   |
      write_sensor_byte(SENSOR_ADDRESS, CONFIGURATION_REGISTER, (unsigned char)(0x09 + (0x04 * switch_ptat_vdd)));
    }
  }
  TimerLib.setInterval_us(ISR, timert);
  ReadingRoutineEnable = 1;
}


/********************************************************************
   Function:        void read_eeprom()
   Description:     read all values from eeprom
 *******************************************************************/
void read_eeprom() {
  int m = 0;
  int n = 0;
  byte b[4];
  id = read_EEPROM_byte(E_ID4) << 24 | read_EEPROM_byte(E_ID3) << 16 | read_EEPROM_byte(E_ID2) << 8 | read_EEPROM_byte(E_ID1);
  mbit_calib = read_EEPROM_byte(E_MBIT_CALIB);
  bias_calib = read_EEPROM_byte(E_BIAS_CALIB);
  clk_calib = read_EEPROM_byte(E_CLK_CALIB);
  bpa_calib = read_EEPROM_byte(E_BPA_CALIB);
  pu_calib = read_EEPROM_byte(E_PU_CALIB);
  mbit_user = read_EEPROM_byte(E_MBIT_USER);
  bias_user = read_EEPROM_byte(E_BIAS_USER);
  clk_user = read_EEPROM_byte(E_CLK_USER);
  bpa_user = read_EEPROM_byte(E_BPA_USER);
  pu_user = read_EEPROM_byte(E_PU_USER);
  vddth1 = read_EEPROM_byte(E_VDDTH1_2) << 8 | read_EEPROM_byte(E_VDDTH1_1);
  vddth2 = read_EEPROM_byte(E_VDDTH2_2) << 8 | read_EEPROM_byte(E_VDDTH2_1);
  vddscgrad = read_EEPROM_byte(E_VDDSCGRAD);
  vddscoff = read_EEPROM_byte(E_VDDSCOFF);
  ptatth1 = read_EEPROM_byte(E_PTATTH1_2) << 8 | read_EEPROM_byte(E_PTATTH1_1);
  ptatth2 = read_EEPROM_byte(E_PTATTH2_2) << 8 | read_EEPROM_byte(E_PTATTH2_1);
  nrofdefpix = read_EEPROM_byte(E_NROFDEFPIX);
  gradscale = read_EEPROM_byte(E_GRADSCALE);
  tablenumber = read_EEPROM_byte(E_TABLENUMBER2) << 8 | read_EEPROM_byte(E_TABLENUMBER1);
  arraytype = read_EEPROM_byte(E_ARRAYTYPE);
  b[0] = read_EEPROM_byte(E_PTATGR_1);
  b[1] = read_EEPROM_byte(E_PTATGR_2);
  b[2] = read_EEPROM_byte(E_PTATGR_3);
  b[3] = read_EEPROM_byte(E_PTATGR_4);
  ptatgr_float = *(float*)b;
  b[0] = read_EEPROM_byte(E_PTATOFF_1);
  b[1] = read_EEPROM_byte(E_PTATOFF_2);
  b[2] = read_EEPROM_byte(E_PTATOFF_3);
  b[3] = read_EEPROM_byte(E_PTATOFF_4);
  ptatoff_float = *(float*)b;
  b[0] = read_EEPROM_byte(E_PIXCMIN_1);
  b[1] = read_EEPROM_byte(E_PIXCMIN_2);
  b[2] = read_EEPROM_byte(E_PIXCMIN_3);
  b[3] = read_EEPROM_byte(E_PIXCMIN_4);
  pixcmin = *(float*)b;
  b[0] = read_EEPROM_byte(E_PIXCMAX_1);
  b[1] = read_EEPROM_byte(E_PIXCMAX_2);
  b[2] = read_EEPROM_byte(E_PIXCMAX_3);
  b[3] = read_EEPROM_byte(E_PIXCMAX_4);
  pixcmax = *(float*)b;
  epsilon = read_EEPROM_byte(E_EPSILON);
  globaloff = read_EEPROM_byte(E_GLOBALOFF);
  globalgain = read_EEPROM_byte(E_GLOBALGAIN_2) << 8 | read_EEPROM_byte(E_GLOBALGAIN_1);


  // for (int m = 0; m < DevConst.PixelPerColumn; m++) {
  //   for (int n = 0; n < DevConst.PixelPerRow; n++) {

  // --- DeadPixAdr ---
  for (int i = 0; i < nrofdefpix; i++) {
    deadpixadr[i] = read_EEPROM_byte(E_DEADPIXADR + 2 * i + 1 ) << 8 | read_EEPROM_byte(E_DEADPIXADR + 2 * i);
    if (deadpixadr[i] > (unsigned short)(DevConst.NumberOfPixel / 2)) {  // adaptedAdr:
      deadpixadr[i] = (unsigned short)(DevConst.NumberOfPixel) + (unsigned short)(DevConst.NumberOfPixel / 2) - deadpixadr[i] + 2 * (unsigned short)(deadpixadr[i] % DevConst.PixelPerRow ) - DevConst.PixelPerRow;
    }
  }


  // --- DeadPixMask ---
  for (int i = 0; i < nrofdefpix; i++) {
    deadpixmask[i] = read_EEPROM_byte(E_DEADPIXMASK + i);
  }


  // --- Thgrad_ij, ThOffset_ij and P_ij ---
  m = 0;
  n = 0;
  pixc2 = pixc2_0; // set pointer to start address of the allocated heap // reset pointer to initial address
  // top half
  for (int i = 0; i < (unsigned short)(DevConst.NumberOfPixel / 2); i++) {
    thgrad[m][n] = read_EEPROM_byte(E_THGRAD + 2 * i + 1) << 8 | read_EEPROM_byte(E_THGRAD + 2 * i);
    thoffset[m][n] = read_EEPROM_byte(E_THOFFSET + 2 * i + 1) << 8 | read_EEPROM_byte(E_THOFFSET + 2 * i);
    *(pixc2 + m * DevConst.PixelPerRow + n) = read_EEPROM_byte(E_PIJ + 2 * i + 1) << 8 | read_EEPROM_byte(E_PIJ + 2 * i);
    n++;
    if (n ==  DevConst.PixelPerRow) {
      n = 0;
      m++;  // !!!! forwards !!!!
    }
  }
  // bottom half
  m = (unsigned char)(DevConst.PixelPerColumn - 1);
  n = 0;
  for (int i = (unsigned short)(DevConst.NumberOfPixel / 2); i < (unsigned short)(DevConst.NumberOfPixel); i++) {
    thgrad[m][n] = read_EEPROM_byte(E_THGRAD + 2 * i + 1) << 8 | read_EEPROM_byte(E_THGRAD + 2 * i);
    thoffset[m][n] = read_EEPROM_byte(E_THOFFSET + 2 * i + 1) << 8 | read_EEPROM_byte(E_THOFFSET + 2 * i);
    *(pixc2 + m * DevConst.PixelPerRow + n) = read_EEPROM_byte(E_PIJ + 2 * i + 1) << 8 | read_EEPROM_byte(E_PIJ + 2 * i);
    n++;

    if (n ==  DevConst.PixelPerRow) {
      n = 0;
      m--;      // !!!! backwards !!!!
    }
  }

  //---VddCompGrad and VddCompOff---
  // top half
  m = 0;
  n = 0;
  // top half
  for (int i = 0; i < (unsigned short)(DevConst.PixelPerBlock); i++) {
    vddcompgrad[m][n] = read_EEPROM_byte(E_VDDCOMPGRAD + 2 * i + 1) << 8 | read_EEPROM_byte(E_VDDCOMPGRAD + 2 * i);
    vddcompoff[m][n] = read_EEPROM_byte(E_VDDCOMPOFF + 2 * i + 1) << 8 | read_EEPROM_byte(E_VDDCOMPOFF + 2 * i);
    n++;
    if (n ==  DevConst.PixelPerRow) {
      n = 0;
      m++;  // !!!! forwards !!!!
    }
  }
  // bottom half
  m = (unsigned char)(DevConst.RowPerBlock * 2 - 1);
  n = 0;
  for (int i = (unsigned short)(DevConst.PixelPerBlock); i < (unsigned short)(DevConst.PixelPerBlock * 2); i++) {
    vddcompgrad[m][n] = read_EEPROM_byte(E_VDDCOMPGRAD + 2 * i + 1) << 8 | read_EEPROM_byte(E_VDDCOMPGRAD + 2 * i);
    vddcompoff[m][n] = read_EEPROM_byte(E_VDDCOMPOFF + 2 * i + 1) << 8 | read_EEPROM_byte(E_VDDCOMPOFF + 2 * i);
    n++;
    if (n ==  DevConst.PixelPerRow) {
      n = 0;
      m--;      // !!!! backwards !!!!
    }
  }

}


/********************************************************************
   Function:        void sort_data()
   Description:     sort the raw data blocks in 2d array and calculate ambient temperature, ptat and vdd
 *******************************************************************/
void sort_data() {

  unsigned long sum = 0;
  unsigned short pos = 0;

  for (int m = 0; m < DevConst.RowPerBlock; m++) {
    for (int n = 0; n < DevConst.PixelPerRow; n++) {

      /*
         for example: a normal line of RAMoutput for HTPAd80x64 looks like:
         RAMoutput[0][] = [ PTAT(MSB), PTAT(LSB), DATA0[MSB], DATA0[LSB], DATA1[MSB], DATA1[LSB], ... , DATA640[MSB], DATA640LSB];
                                                      |
                                                      |-- DATA_Pos = 2 (first data byte)
      */
      pos = (unsigned short)(2 * n + DevConst.DataPos + m * 2 * DevConst.PixelPerRow);



      /******************************************************************************************************************
        new PIXEL values
      ******************************************************************************************************************/
      for (int i = 0; i < DevConst.NumberOfBlocks; i++) {
        // top half
        data_pixel[m + i * DevConst.RowPerBlock][n] =
          (unsigned short)(RAMoutput[i][pos] << 8 | RAMoutput[i][pos + 1]);
        // bottom half
        data_pixel[DevConst.PixelPerColumn - 1 - m - i * DevConst.RowPerBlock][n] =
          (unsigned short)(RAMoutput[2 * DevConst.NumberOfBlocks + 2 - i - 1][pos] << 8 | RAMoutput[2 * DevConst.NumberOfBlocks + 2 - i - 1][pos + 1]);
      }


      /******************************************************************************************************************
        new electrical offset values (store them in electrical offset buffer and calculate the average for pixel compensation
      ******************************************************************************************************************/
      if (picnum % ELOFFSETS_BUFFER_SIZE == 1) {
        if ((!eloffset[m][n]) || (picnum < ELOFFSETS_FILTER_START_DELAY)) {
          // top half
          eloffset[m][n] = (unsigned short)(RAMoutput[DevConst.NumberOfBlocks][pos] << 8 | RAMoutput[DevConst.NumberOfBlocks][pos + 1]);
          // bottom half
          eloffset[2 * DevConst.RowPerBlock - 1 - m][n] = (unsigned short)(RAMoutput[DevConst.NumberOfBlocks + 1][pos] << 8 | RAMoutput[DevConst.NumberOfBlocks + 1][pos + 1]);
          use_eloffsets_buffer = 1;

        }
        else {
          // use a moving average filter
          // top half
          sum = (unsigned long)eloffset[m][n] * (unsigned long)(ELOFFSETS_BUFFER_SIZE - 1);
          sum += (unsigned long)(RAMoutput[DevConst.NumberOfBlocks][pos] << 8 | RAMoutput[DevConst.NumberOfBlocks][pos + 1]);
          eloffset[m][n] = (unsigned short)((float)sum / ELOFFSETS_BUFFER_SIZE + 0.5);
          // bottom half
          sum = (unsigned long)eloffset[2 * DevConst.RowPerBlock - 1 - m][n] * (unsigned long)(ELOFFSETS_BUFFER_SIZE - 1);
          sum += (unsigned long)(RAMoutput[DevConst.NumberOfBlocks + 1][pos] << 8 | RAMoutput[DevConst.NumberOfBlocks + 1][pos + 1]);
          eloffset[2 * DevConst.RowPerBlock - 1 - m][n] = (unsigned short)((float)sum / ELOFFSETS_BUFFER_SIZE + 0.5);
        }
      }

    }

  }



  /******************************************************************************************************************
    new PTAT values (store them in PTAT buffer and calculate the average for pixel compensation
  ******************************************************************************************************************/
  if (switch_ptat_vdd == 1) {
    sum = 0;
    // calculate ptat average (datasheet, chapter: 11.1 Ambient Temperature )
    for (int i = 0; i < DevConst.NumberOfBlocks; i++) {
      // block top half
      sum += (unsigned short)(RAMoutput[i][DevConst.PTATPos] << 8 | RAMoutput[i][DevConst.PTATPos + 1]);
      // block bottom half
      sum += (unsigned short)(RAMoutput[2 * DevConst.NumberOfBlocks - i + 1][DevConst.PTATPos] << 8 | RAMoutput[2 * DevConst.NumberOfBlocks - i + 1][DevConst.PTATPos + 1]);
    }
    ptat_av_uint16 = (unsigned short)((float)sum / (float)(2.0 * DevConst.NumberOfBlocks));
    Ta = (unsigned short)((unsigned short)ptat_av_uint16 * (float)ptatgr_float + (float)ptatoff_float);


    ptat_buffer[ptat_i] = ptat_av_uint16;
    ptat_i++;
    if (ptat_i == PTAT_BUFFER_SIZE) {
      if (use_ptat_buffer == 0) {
        //Serial.print(" | PTAT buffer complete");
        use_ptat_buffer = 1;
      }
      ptat_i = 0;
    }

    if (use_ptat_buffer) {
      // now overwrite the old ptat average
      sum = 0;
      for (int i = 0; i < PTAT_BUFFER_SIZE; i++) {
        sum += ptat_buffer[i];
      }
      ptat_av_uint16 = (uint16_t)((float)sum / PTAT_BUFFER_SIZE);
    }


  }


  /******************************************************************************************************************
    new VDD values (store them in VDD buffer and calculate the average for pixel compensation
  ******************************************************************************************************************/
  if (switch_ptat_vdd == 0) {
    sum = 0;
    // calculate vdd average (datasheet, chapter: 11.4 Vdd Compensation )
    for (int i = 0; i < DevConst.NumberOfBlocks; i++) {
      // block top half
      sum += (unsigned short)(RAMoutput[i][DevConst.VDDPos] << 8 | RAMoutput[i][DevConst.VDDPos + 1]);
      // block bottom half
      sum += (unsigned short)(RAMoutput[2 * DevConst.NumberOfBlocks - i + 1][DevConst.VDDPos] << 8 | RAMoutput[2 * DevConst.NumberOfBlocks - i + 1][DevConst.VDDPos + 1]);
    }
    vdd_av_uint16 = (unsigned short)((float)sum / (float)(2.0 * DevConst.NumberOfBlocks));


    // write into vdd buffer
    vdd_buffer[vdd_i] = vdd_av_uint16;
    vdd_i++;
    if (vdd_i == VDD_BUFFER_SIZE) {
      if (use_vdd_buffer == 0) {
        //Serial.print(" | VDD buffer complete");
        use_vdd_buffer = 1;
      }
      vdd_i = 0;
    }
    if (use_vdd_buffer) {
      sum = 0;
      for (int i = 0; i < VDD_BUFFER_SIZE; i++) {
        sum += vdd_buffer[i];
      }
      // now overwrite the old vdd average
      vdd_av_uint16 = (uint16_t)((float)sum / VDD_BUFFER_SIZE);
    }

  }

}


/********************************************************************
   Function:        void write_calibration_settings_to_sensor()
   Description:     write calibration data (from eeprom) to trim registers (sensor)
 *******************************************************************/
void write_calibration_settings_to_sensor() {

  write_sensor_byte(SENSOR_ADDRESS, TRIM_REGISTER1, mbit_calib);
  delay(5);
  write_sensor_byte(SENSOR_ADDRESS, TRIM_REGISTER2, bias_calib);
  delay(5);
  write_sensor_byte(SENSOR_ADDRESS, TRIM_REGISTER3, bias_calib);
  delay(5);
  write_sensor_byte(SENSOR_ADDRESS, TRIM_REGISTER4, clk_calib);
  delay(5);
  write_sensor_byte(SENSOR_ADDRESS, TRIM_REGISTER5, bpa_calib);
  delay(5);
  write_sensor_byte(SENSOR_ADDRESS, TRIM_REGISTER6, bpa_calib);
  delay(5);
  write_sensor_byte(SENSOR_ADDRESS, TRIM_REGISTER7, pu_calib);
  delay(5);
}


/********************************************************************
   Function:        void read_EEPROM_byte(unsigned int eeaddress )
   Description:     read eeprom register as 8
   Dependencies:    register address (address)
 *******************************************************************/
byte write_EEPROM_byte(unsigned short address, unsigned char content ) {

  Wire.beginTransmission(EEPROM_ADDRESS);
  Wire.write((int)(address >> 8));   // MSB
  Wire.write((int)(address & 0xFF)); // LSB
  Wire.write(content);
  Wire.endTransmission();
  return 0;

}




/********************************************************************
   Function:        void write_user_settings_to_sensor()

   Description:     write calibration data (from eeprom) to trim registers (sensor)

   Dependencies:
 *******************************************************************/
void write_user_settings_to_sensor() {
  write_sensor_byte(SENSOR_ADDRESS, TRIM_REGISTER1, mbit_user);
  delay(5);
  write_sensor_byte(SENSOR_ADDRESS, TRIM_REGISTER2, bias_user);
  delay(5);
  write_sensor_byte(SENSOR_ADDRESS, TRIM_REGISTER3, bias_user);
  delay(5);
  write_sensor_byte(SENSOR_ADDRESS, TRIM_REGISTER4, clk_user);
  delay(5);
  write_sensor_byte(SENSOR_ADDRESS, TRIM_REGISTER5, bpa_user);
  delay(5);
  write_sensor_byte(SENSOR_ADDRESS, TRIM_REGISTER6, bpa_user);
  delay(5);
  write_sensor_byte(SENSOR_ADDRESS, TRIM_REGISTER7, pu_user);
  delay(5);
}


/********************************************************************
 ********************************************************************
    - - - PART 4: SERIAL FUNCTIONS - - -
  checkSerial()
  print_eeprom_header()
  print_eeprom_hex()
  print_menu()
 ********************************************************************
 ********************************************************************/
/********************************************************************
   Function:        print_eeprom_header()
   Description:
 *******************************************************************/
void print_eeprom_header() {
  Serial.print("data\t\tregister\ttype\t\tvalue\n");
  Serial.println("------------------------------------------------------------");
  Serial.print("PixCmin\t\t0x00-0x03\tfloat\t\t");
  Serial.println(pixcmin, 0);
  Serial.print("PixCmax\t\t0x04-0x07\tfloat\t\t");
  Serial.println(pixcmax, 0);
  Serial.print("gradScale\t0x08\t\tunsigned char\t");
  Serial.println(gradscale);
  Serial.print("TN\t\t0x0B-0x0C\tunsigned short\t");
  Serial.println(tablenumber);
  Serial.print("epsilon\t\t0x0D\t\tunsigned char\t");
  Serial.println(epsilon);
  Serial.print("MBIT(calib)\t0x1A\t\tunsigned char\t");
  Serial.println(mbit_calib);
  Serial.print("BIAS(calib)\t0x1B\t\tunsigned char\t");
  Serial.println(bias_calib);
  Serial.print("CLK(calib)\t0x1C\t\tunsigned char\t");
  Serial.println(clk_calib);
  Serial.print("BPA(calib)\t0x1D\t\tunsigned char\t");
  Serial.println(bpa_calib);
  Serial.print("PU(calib)\t0x1E\t\tunsigned char\t");
  Serial.println(pu_calib);
  Serial.print("Arraytype\t0x22\t\tunsigned char\t");
  Serial.println(arraytype);
  Serial.print("VDDTH1\t\t0x26-0x27\tunsigned short\t");
  Serial.println(vddth1);
  Serial.print("VDDTH2\t\t0x28-0x29\tunsigned short\t");
  Serial.println(vddth2);
  Serial.print("PTAT-gradient\t0x34-0x37\tfloat\t\t");
  Serial.println(ptatgr_float, 4);
  Serial.print("PTAT-offset\t0x38-0x3B\tfloat\t\t");
  Serial.println(ptatoff_float, 4);
  Serial.print("PTAT(Th1)\t0x3C-0x3D\tunsigned short\t");
  Serial.println(ptatth1);
  Serial.print("PTAT(Th2)\t0x3E-0x3F\tunsigned short\t");
  Serial.println(ptatth2);
  Serial.print("VddScGrad\t0x4E\t\tunsigned char\t");
  Serial.println(vddscgrad);
  Serial.print("VddScOff\t0x4F\t\tunsigned char\t");
  Serial.println(vddscoff);
  Serial.print("GlobalOff\t0x54\t\tsigned char\t");
  Serial.println(globaloff);
  Serial.print("GlobalGain\t0x55-0x56\tunsigned short\t");
  Serial.println(globalgain);
  Serial.print("SensorID\t0x74-0x77\tunsigned long\t");
  Serial.println(id);
}

/********************************************************************
   Function:        print_eeprom_hex()
   Description:     print eeprom contint as hex values
 *******************************************************************/
void print_eeprom_hex() {

  Serial.print("\n\n\n---PRINT EEPROM (HEX)---\n");
  Serial.print("\n\n\t\t\t0x00\t0x01\t0x02\t0x03\t0x04\t0x05\t0x06\t0x07\t0x08\t0x09\t0x0A\t0x0B\t0x0C\t0x0D\t0x0E\t0x0F\n");

  // line
  for (int i = 0; i < 75; i++) {
    Serial.print("- ");
  }

  for (int i = 0; i < EEPROM_SIZE; i++) {


    if (i % 16 == 0) {
      Serial.print("\n");

      if (i < E_DEADPIXADR) {
        Serial.print("HEADER\t0x");
        Serial.print(i, HEX);
        Serial.print("\t|\t");
      }
      else if (i < 0x00D0) {
        Serial.print("DEADPIX\t0x");
        Serial.print(i, HEX);
        Serial.print("\t|\t");
      }
      else if (i < E_VDDCOMPGRAD) {
        Serial.print("FREE\t0x");
        Serial.print(i, HEX);
        Serial.print("\t|\t");
      }
      else if (i < E_VDDCOMPOFF) {
        Serial.print("VDDGRAD\t0x");
        Serial.print(i, HEX);
        Serial.print("\t|\t");
      }
      else if (i < E_THGRAD) {
        Serial.print("VDDOFF\t0x");
        Serial.print(i, HEX);
        Serial.print("\t|\t");
      }
      else if (i < E_THOFFSET) {
        Serial.print("THGRAD\t0x");
        Serial.print(i, HEX);
        Serial.print("\t|\t");
      }
      else if (i < E_PIJ) {
        Serial.print("THOFF\t0x");
        Serial.print(i, HEX);
        Serial.print("\t|\t");
      }
      else if (i < (E_PIJ + 2*NUMBER_OF_PIXEL)) {
        Serial.print("PIXC\t0x");
        Serial.print(i, HEX);
        Serial.print("\t|\t");
      }
      else {
        Serial.print("FREE\t0x");
        Serial.print(i, HEX);
        Serial.print("\t|\t");
      }
    }
    else {
      Serial.print("\t");
    }

    Serial.print("0x");
    if (read_EEPROM_byte(i) < 0x10) {
      Serial.print("0");
    }
    Serial.print(read_EEPROM_byte(i), HEX);

  }

  Serial.print("\n\n\n\ndone (m... back to menu)\n\n\n");
}



/********************************************************************
   Function:      print_menu()
   Description:
 *******************************************************************/
void print_menu() {
  Serial.println("\n\n\n***************************************************");
  Serial.println("Application Shield                      /_/eimann");
  Serial.println("for ESP32-DevkitC                      / /   Sensor");

  Serial.println("\nYou can choose one of these options by sending the \ncharacter\n ");
  Serial.println("read SENSOR values:");
  Serial.println("  a... final array temperatures (in deci Kelvin)");
  Serial.println("  b... show all raw values (in digits)");
  Serial.println("  c... show all calculation steps");
  Serial.println("read EEPROM values:");
  Serial.println("  d... whole eeprom content (in hexadecimal)");
  Serial.println("  e... Header values");
  Serial.println("  f... VddCompGrad");
  Serial.println("  g... VddCompOff");
  Serial.println("  h... ThGrad");
  Serial.println("  i... ThOff");
  Serial.println("  j... PixC (scaled)");
  Serial.println("write/change EEPROM values:");
  Serial.println("  k... increase emissivity by 1");
  Serial.println("  l... decrease emissivity by 1");
  Serial.println("\t\t\t\t\tver2.2 (dp)");
  Serial.println("***************************************************\n\n\n");
}


/********************************************************************
   Function:        print_final_array()
   Description:
 *******************************************************************/
void print_final_array(void) {
  Serial.println("\n\n---pixel data ---");
  for (int m = 0; m < DevConst.PixelPerColumn; m++) {
    for (int n = 0; n < DevConst.PixelPerRow; n++) {
      Serial.print(data_pixel[m][n]);
      Serial.print("\t");
    }
    Serial.println("");
  }
}

/********************************************************************
   Function:        print_RAM_array()
   Description:
 *******************************************************************/
void print_RAM_array(void) {
  Serial.print("\n\n\n---pixel data ---\n");
  for (int m = 0; m < (2 * NUMBER_OF_BLOCKS + 2); m++) {
    for (int n = 0; n < BLOCK_LENGTH; n++) {
      Serial.print(RAMoutput[m][n], HEX);
      Serial.print("\t");
    }
    Serial.print("\n");
  }
  Serial.print("\n\n\n");
}

/********************************************************************
   Function:        checkSerial()
   Description:
 *******************************************************************/
void checkSerial() {
  serial_input = Serial.read();

  switch (serial_input) {
    case 0xFF:
      //nothing
      break;

    case 'a':
      if (send_data)
        Serial.println("stop data stream in GUI before");
      else
        print_state = 1;
      break;

    case 'b':
      if (send_data)
        Serial.println("stop data stream in GUI before");
      else
        print_state = 2;
      break;

    case 'c':
      if (send_data)
        Serial.println("stop data stream in GUI before");
      else
        print_state = 3;
      break;


    case 'm':
      while (state);
      ReadingRoutineEnable = 0;
      print_menu();
      ReadingRoutineEnable = 1;
      break;

    case 'd':
      while (state);
      ReadingRoutineEnable = 0;
      print_eeprom_hex();
      ReadingRoutineEnable = 1;
      break;

    case 'e':
      while (state);
      ReadingRoutineEnable = 0;
      print_eeprom_header();
      ReadingRoutineEnable = 1;
      break;

    case 'f':
      while (state);
      ReadingRoutineEnable = 0;
      Serial.print("\n\n\n---VddCompGrad---\n");
      for (int m = 0; m < (DevConst.RowPerBlock * 2); m++) {
        for (int n = 0; n < DevConst.PixelPerRow; n++) {
          Serial.print(vddcompgrad[m][n]);
          Serial.print("\t");
        }
        Serial.print("\n");
      }
      Serial.print("\n\n\n");
      ReadingRoutineEnable = 1;
      break;

    case 'g':
      while (state);
      ReadingRoutineEnable = 0;
      Serial.print("\n\n\n---VddCompOff---\n");
      for (int m = 0; m < (DevConst.RowPerBlock * 2); m++) {
        for (int n = 0; n < DevConst.PixelPerRow; n++) {
          Serial.print(vddcompoff[m][n]);
          Serial.print("\t");
        }
        Serial.print("\n");
      }
      Serial.print("\n\n\n");
      ReadingRoutineEnable = 1;
      break;

    case 'h':
      while (state);
      ReadingRoutineEnable = 0;
      Serial.print("\n\n\n---ThGrad---\n");
      for (int m = 0; m < DevConst.PixelPerColumn; m++) {
        for (int n = 0; n < DevConst.PixelPerRow; n++) {
          Serial.print(thgrad[m][n]);
          Serial.print("\t");
        }
        Serial.print("\n");
      }
      Serial.print("\n\n\n");
      ReadingRoutineEnable = 1;
      break;



    case 'i':
      while (state);
      ReadingRoutineEnable = 0;
      // print ThOffset in serial monitor
      Serial.print("\n\n\n---ThOffset---\n");
      for (int m = 0; m < DevConst.PixelPerColumn; m++) {
        for (int n = 0; n < DevConst.PixelPerRow; n++) {
          Serial.print(thoffset[m][n]);
          Serial.print("\t");
        }
        Serial.print("\n");
      }
      Serial.print("\n\n\n");
      ReadingRoutineEnable = 1;
      break;

    case 'j':
      while (state);
      ReadingRoutineEnable = 0;
      // print PixC in serial monitor
      Serial.print("\n\n\n---PixC---\n");
      pixc2 = pixc2_0; // set pointer to start address of the allocated heap
      for (int m = 0; m < DevConst.PixelPerColumn; m++) {
        for (int n = 0; n < DevConst.PixelPerRow; n++) {
          Serial.print(*(pixc2 + m * DevConst.PixelPerRow + n));
          Serial.print("\t");
        }
        Serial.print("\n");
      }
      Serial.print("\n\n\n");
      ReadingRoutineEnable = 1;
      break;

    case 'k':
      ReadingRoutineEnable = 0;
      TimerLib.clearTimer();
      Serial.println("\n\n\n---Increase emissivity---");
      Serial.print("old emissivity: \t");
      Serial.println(epsilon);
      Serial.print("new emissivity: \t");
      if (epsilon < 100) {
        epsilon++;
        Wire.setClock(CLOCK_EEPROM); // I2C clock frequency 400kHz (for eeprom communication)
        write_EEPROM_byte(E_EPSILON, epsilon);
        Wire.setClock(CLOCK_SENSOR);

        // calculate pixcij with new epsilon
        pixc2 = pixc2_0; // set pointer to start address of the allocated heap
        double d = (double)epsilon / (double)lastepsilon;
        for (int m = 0; m < DevConst.PixelPerColumn; m++) {
          for (int n = 0; n < DevConst.PixelPerRow; n++) {
            *(pixc2 + m * DevConst.PixelPerRow + n) = (unsigned long)((double) * (pixc2 + m * DevConst.PixelPerRow + n) * (double)d);
          }
        }
        lastepsilon = epsilon;
        Serial.print(epsilon);
        Serial.println(" (new emissivity is stored in the EEPROM now)");
      }
      else {
        Serial.print(epsilon);
        Serial.println(" (you cannot set the emissivity higher than 100%)");
      }
      delay(1000);
      TimerLib.setInterval_us(ISR, timert);
      ReadingRoutineEnable = 1;
      break;

    case 'l':
      ReadingRoutineEnable = 0;
      TimerLib.clearTimer();
      Serial.print("\n\n\n---Decrease emissivity---");
      Serial.print("\nold emissivity: \t");
      Serial.print(epsilon);
      Serial.print("\nnew emissivity: \t");
      if (epsilon > 0) {
        epsilon--;
        Wire.setClock(CLOCK_EEPROM); // I2C clock frequency 400kHz (for eeprom communication)
        write_EEPROM_byte(E_EPSILON, epsilon);
        Wire.setClock(CLOCK_SENSOR);
        // calculate pixcij with new epsilon
        pixc2 = pixc2_0; // set pointer to start address of the allocated heap
        double d = (double)epsilon / (double)lastepsilon;
        for (int m = 0; m < DevConst.PixelPerColumn; m++) {
          for (int n = 0; n < DevConst.PixelPerRow; n++) {
            *(pixc2 + m * DevConst.PixelPerRow + n) = (unsigned long)((double) * (pixc2 + m * DevConst.PixelPerRow + n) * (double)d);
          }
        }
        lastepsilon = epsilon;
        Serial.print(epsilon);
        Serial.print(" (new emissivity is stored in the EEPROM now)");
      }
      else {
        Serial.print(epsilon);
        Serial.print(" (you cannot set the emissivity lower as 0%)");
      }
      delay(1000);
      TimerLib.setInterval_us(ISR, timert);
      ReadingRoutineEnable = 1;
      break;


  }


}


/********************************************************************
   Function:        print_calc_steps()
   Description:     print every needed step for temperature calculation + pixel masking
 *******************************************************************/
void print_calc_steps2() {
  int64_t vij_pixc_and_pcscaleval;
  int64_t pixcij;
  int64_t vdd_calc_steps;
  uint16_t table_row, table_col;
  int32_t vx, vy, ydist, dta;
  signed long pixel;
  pixc2 = pixc2_0;

  Serial.println("\n\ncalculate the average of VDD and PTAT buffer");

  Serial.print("PTATbuf[");
  Serial.print(PTAT_BUFFER_SIZE);
  Serial.print("] = { ");
  for (int i = 0; i < PTAT_BUFFER_SIZE; i++) {
    Serial.print(ptat_buffer[i]);
    if (i < (PTAT_BUFFER_SIZE - 1))
      Serial.print(" , ");
    else
      Serial.print(" }");
  }
  Serial.print("\nPTAT_average = ");
  Serial.print(ptat_av_uint16);

  Serial.print("\nVDDbuf[");
  Serial.print(VDD_BUFFER_SIZE);
  Serial.print("] = { ");
  for (int i = 0; i < VDD_BUFFER_SIZE; i++) {
    Serial.print(vdd_buffer[i]);
    if (i < (VDD_BUFFER_SIZE - 1))
      Serial.print(" , ");
    else
      Serial.print(" }");
  }
  Serial.print("\nVDD_average = ");
  Serial.print(vdd_av_uint16);

  Serial.println("\n\ncalculate ambient temperatur (Ta)");
  Serial.print("Ta = ");
  Serial.print(ptat_av_uint16);
  Serial.print(" * ");
  Serial.print(ptatgr_float, 5);
  Serial.print(" + ");
  Serial.print(ptatoff_float, 5);
  Serial.print(" = ");
  Serial.print(Ta);
  Serial.print(" (Value is given in dK)");


  /******************************************************************************************************************
    step 0: find column of lookup table
  ******************************************************************************************************************/
  for (int i = 0; i < NROFTAELEMENTS; i++) {
    if (Ta > XTATemps[i]) {
      table_col = i;
    }
  }
  dta = Ta - XTATemps[table_col];
  ydist = (int32_t)ADEQUIDISTANCE;

  Serial.println("\n\nprint all calculation steps for each pixel");
  Serial.println("table columns:");
  Serial.println("No\tpixel number");
  Serial.println("i\trepresents the row of the pixel");
  Serial.println("j\trepresents the column of the pixel");
  Serial.println("Vij\tis row pixel voltages (digital); readout from the RAM");
  Serial.println("I\tis the thermal offset compensated voltage");
  Serial.println("II\tis the thermal and electrical offset compensated voltage");
  Serial.println("III\tis the Vdd compensated voltage");
  Serial.println("IV\tis the sensivity compensated IR voltage");
  Serial.println("T[dK]\tis final pixel temperature in dK (deci Kelvin)");
  Serial.println("T[°C]\tis final pixel temperature in °C");

  Serial.println("\n\nNo\ti\tj\tVij\tI\tII\tIII\tIV\tT[dK]\tT[°C]");
  Serial.println("-----------------------------------------------------------------------------");


  for (int m = 0; m < DevConst.PixelPerColumn; m++) {
    for (int n = 0; n < DevConst.PixelPerRow; n++) {

      Serial.print(m * DevConst.PixelPerRow + n);
      Serial.print("\t");
      Serial.print(m);
      Serial.print("\t");
      Serial.print(n);
      /******************************************************************************************************************
         step 1: use a variable with bigger data format for the compensation steps
       ******************************************************************************************************************/
      pixel = (signed long) data_pixel[m][n];
      Serial.print("\t"); Serial.print(pixel);
      /******************************************************************************************************************
         step 2: compensate thermal drifts (see datasheet, chapter: Thermal Offset)
       ******************************************************************************************************************/
      pixel -= (int32_t)(((int32_t)thgrad[m][n] * (int32_t)ptat_av_uint16) / (int32_t)gradscale_div);
      pixel -= (int32_t)thoffset[m][n];
      Serial.print("\t"); Serial.print(pixel);
      /******************************************************************************************************************
         step 3: compensate electrical offset (see datasheet, chapter: Electrical Offset)
       ******************************************************************************************************************/
      if (m < DevConst.PixelPerColumn / 2) { // top half
        pixel -= eloffset[m % DevConst.RowPerBlock][n];
      }
      else { // bottom half
        pixel -= eloffset[m % DevConst.RowPerBlock + DevConst.RowPerBlock][n];
      }
      Serial.print("\t"); Serial.print(pixel);
      /******************************************************************************************************************
         step 4: compensate vdd (see datasheet, chapter: Vdd Compensation)
       ******************************************************************************************************************/
      // first select VddCompGrad and VddCompOff for pixel m,n:
      if (m < DevConst.PixelPerColumn / 2) {      // top half
        vddcompgrad_n = vddcompgrad[m % DevConst.RowPerBlock][n];
        vddcompoff_n = vddcompoff[m % DevConst.RowPerBlock][n];
      }
      else {       // bottom half
        vddcompgrad_n = vddcompgrad[m % DevConst.RowPerBlock + DevConst.RowPerBlock][n];
        vddcompoff_n = vddcompoff[m % DevConst.RowPerBlock + DevConst.RowPerBlock][n];
      }
      // now do the vdd calculation
      vdd_calc_steps = vddcompgrad_n * ptat_av_uint16;
      vdd_calc_steps = vdd_calc_steps / vddscgrad_div;
      vdd_calc_steps = vdd_calc_steps + vddcompoff_n;
      vdd_calc_steps = vdd_calc_steps * ( vdd_av_uint16 - vddth1 - ((vddth2 - vddth1) / (ptatth2 - ptatth1)) * (ptat_av_uint16  - ptatth1));
      vdd_calc_steps = vdd_calc_steps / vddscoff_div;
      pixel -= vdd_calc_steps;
      Serial.print("\t"); Serial.print(pixel);
      /******************************************************************************************************************
         step 5: multiply sensitivity coeff for each pixel (see datasheet, chapter: Object Temperature)
       ******************************************************************************************************************/
      vij_pixc_and_pcscaleval = pixel * (int64_t)PCSCALEVAL;
      pixel =  (int32_t)(vij_pixc_and_pcscaleval / *pixc2);
      pixc2++;
      Serial.print("\t"); Serial.print(pixel);
      /******************************************************************************************************************
         step 6: find correct temp for this sensor in lookup table and do a bilinear interpolation (see datasheet, chapter:  Look-up table)
       ******************************************************************************************************************/
      table_row = pixel + TABLEOFFSET;
      table_row = table_row >> ADEXPBITS;
      // bilinear interpolation
      vx = ((((int32_t)TempTable[table_row][table_col + 1] - (int32_t)TempTable[table_row][table_col]) * (int32_t)dta) / (int32_t)TAEQUIDISTANCE) + (int32_t)TempTable[table_row][table_col];
      vy = ((((int32_t)TempTable[table_row + 1][table_col + 1] - (int32_t)TempTable[table_row + 1][table_col]) * (int32_t)dta) / (int32_t)TAEQUIDISTANCE) + (int32_t)TempTable[table_row + 1][table_col];
      pixel = (uint32_t)((vy - vx) * ((int32_t)(pixel + TABLEOFFSET) - (int32_t)YADValues[table_row]) / ydist + (int32_t)vx);

      /******************************************************************************************************************
         step 7: add GlobalOffset (stored as signed char)
       ******************************************************************************************************************/
      pixel += globaloff;
      Serial.print("\t"); Serial.print(pixel);
      Serial.print("\t"); Serial.print((float)((pixel - 2732) / 10.0));
      Serial.print("\n");
      /******************************************************************************************************************
        step 8: overwrite the uncompensate pixel with the new calculated compensated value
      ******************************************************************************************************************/
      data_pixel[m][n] = (unsigned short)pixel;

    }
  }

  /******************************************************************************************************************
    step 8: overwrite the uncompensate pixel with the new calculated compensated value
  ******************************************************************************************************************/
  pixel_masking();
}


void sensor_init(){
  pixc2_0 = (unsigned long *)malloc(NUMBER_OF_PIXEL * 4);
  if (pixc2_0 == NULL)
  {
    Serial.println("heap_caps_malloc failed");
  }
  else
  {
    Serial.println("heap_caps_malloc succeeded");
    pixc2 = pixc2_0; // set pointer to start address of the allocated heap
  }
  #ifndef USE_SDK
  Wire.setClock(1000000);
//*******************************************************************
// searching for sensor; if connected: read the whole EEPROM
//*******************************************************************
  uint8_t error = 1;
  while (error != 0) {
    Wire.setSDA(MLX_SDA);
    Wire.setSCL(MLX_SCL);
    Serial.printf("Triying to connect to HTPAd at address: %d\n", SENSOR_ADDRESS);
    delay(2000);
    Wire.begin();
    Wire.beginTransmission(SENSOR_ADDRESS);
    error = Wire.endTransmission();
    Serial.printf("HTPAd is not ready, retrying..., error code: %d\n", error);
    }
    prob_status = PROB_INITIALIZING;
    Wire.setClock(CLOCK_EEPROM); // I2C clock frequency 400kHz (for eeprom communication)
    read_eeprom();
    // I2C clock frequency (for sensor communication)
    Wire.setClock(CLOCK_SENSOR);
    #else
    ... TODO
    #endif
    //*******************************************************************
    // wake up and start the sensor
    //*******************************************************************
    // to wake up sensor set configuration register to 0x01
    // |    RFU    |   Block   | Start | VDD_MEAS | BLIND | WAKEUP |
    // |  0  |  0  |  0  |  0  |   0   |    0     |   0   |    1   |
    write_sensor_byte(SENSOR_ADDRESS, CONFIGURATION_REGISTER, 0x01);
    // write the calibration settings into the trim registers
    write_calibration_settings_to_sensor();
    // to start sensor set configuration register to 0x09
    // |    RFU    |   Block   | Start | VDD_MEAS | BLIND | WAKEUP |
    // |  0  |  0  |  0  |  0  |   1   |    0     |   0   |    1   |
    write_sensor_byte(SENSOR_ADDRESS, CONFIGURATION_REGISTER, 0x09);
    Serial.println("HTPAd is ready");
    prob_status = PROB_PREPARING;
    //*******************************************************************
    // do bigger calculation here before you jump into the loop() function
    //*******************************************************************
    gradscale_div = pow(2, gradscale);
    vddscgrad_div = pow(2, vddscgrad);
    vddscoff_div = pow(2, vddscoff);
    calcPixC(); // calculate the pixel constants
    //*******************************************************************
    // timer initialization
    //*******************************************************************
    timert = calc_timert(clk_calib, mbit_calib);
    TimerLib.setInterval_us(ISR, timert);
    Serial.printf("calc_timert: %d\n", timert);
}

void sensor_power_on(){
  pinMode(MLX_VDD, OUTPUT);
  digitalWrite(MLX_VDD, LOW);
  prob_status = PROB_CONNECTING;
}


// 更新传感器画面
void sensor_loop(){
  // unsigned long t0 = millis();
  // TimerLib.timerLoop();
  
  NewDataAvailable = true;
  if (NewDataAvailable) {
    readblockinterrupt();
    NewDataAvailable = 0;
    delay(1);
    // Serial.printf("read_block cost :%d\n", millis() - t0);
  }

  if (state) { // state is 1 when all raw sensor voltages are read for this picture
    while (pix_cp_lock == true) {delay(2);}
    prob_lock = true;
    sort_data();
    state = 0;
    calculate_pixel_temp();
    prob_lock = false;
    // Serial.printf("max, min, avg: %d, %d, %d\n", T_max, T_min, T_avg);
  }
}

#endif
