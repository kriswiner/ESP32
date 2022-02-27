/* 06/16/2017 Copyright Tlera Corporation
 *  
 *  Created by Kris Winer
 *  
 This sketch uses SDA/SCL on pins 42/43 (back pads), respectively, and it uses the Dragonfly STM32L476RE Breakout Board.
 The BME280 is a simple but high resolution pressure/humidity/temperature sensor, which can be used in its high resolution
 mode but with power consumption of 20 microAmp, or in a lower resolution mode with power consumption of
 only 1 microAmp. The choice will depend on the application.
 
 Library may be used freely and without limit with attribution.
 
*/
 
#include "BME280.h"
#include "I2Cdev.h"

#define USBSerial Serial

BME280::BME280(I2Cdev* i2c_bus)
{
  _i2c_bus = i2c_bus;
}


uint8_t BME280::getChipID()
{
  uint8_t c = _i2c_bus->readByte(BME280_ADDRESS, BME280_ID);
  return c;
}

void BME280::resetBME280()
{
  _i2c_bus->writeByte(BME280_ADDRESS, BME280_RESET, 0xB6);
}


int32_t BME280::readBME280Temperature()
{
  uint8_t rawData[3];  // 20-bit temperature register data stored here
  _i2c_bus->readBytes(BME280_ADDRESS, BME280_TEMP_MSB, 3, &rawData[0]);  
  return (uint32_t) (((uint32_t) rawData[0] << 16 | (uint32_t) rawData[1] << 8 | rawData[2]) >> 4);
}


int32_t BME280::readBME280Pressure()
{
  uint8_t rawData[3];  // 20-bit pressure register data stored here
  _i2c_bus->readBytes(BME280_ADDRESS, BME280_PRESS_MSB, 3, &rawData[0]);  
  return (uint32_t) (((uint32_t) rawData[0] << 16 | (uint32_t) rawData[1] << 8 | rawData[2]) >> 4);
}


int32_t BME280::BME280::readBME280Humidity()
{
  uint8_t rawData[2];  // 16-bit humiditye register data stored here
  _i2c_bus->readBytes(BME280_ADDRESS, BME280_HUM_MSB, 2, &rawData[0]);  
  return (uint32_t) (((uint32_t) rawData[0] << 24 | (uint32_t) rawData[1] << 16) ) >> 16;
}


void BME280::BME280forced()
{
    uint8_t temp = _i2c_bus->readByte(BME280_ADDRESS, BME280_CTRL_MEAS);
    _i2c_bus->writeByte(BME280_ADDRESS, BME280_CTRL_MEAS, temp | Forced);
 
    while( (_i2c_bus->readByte(BME280_ADDRESS, BME280_STATUS)) & 0x10) { } // wait for conversion byte to clear
}


void BME280::BME280Init(uint8_t Posr, uint8_t Hosr, uint8_t Tosr, uint8_t Mode, uint8_t IIRFilter, uint8_t SBy)
{
  // Configure the BME280
  // Set H oversampling rate
  _i2c_bus->writeByte(BME280_ADDRESS, BME280_CTRL_HUM, 0x07 & Hosr);
  // Set T and P oversampling rates and sensor mode
  _i2c_bus->writeByte(BME280_ADDRESS, BME280_CTRL_MEAS, Tosr << 5 | Posr << 2 | Mode);
  // Set standby time interval in normal mode and bandwidth
  _i2c_bus->writeByte(BME280_ADDRESS, BME280_CONFIG, SBy << 5 | IIRFilter << 2);
  
  uint8_t calib[26];
  _i2c_bus->readBytes(BME280_ADDRESS, BME280_CALIB00, 26, &calib[0]);
  _dig_T1 = (uint16_t)(((uint16_t) calib[1] << 8) | calib[0]);
  _dig_T2 = ( int16_t)((( int16_t) calib[3] << 8) | calib[2]);
  _dig_T3 = ( int16_t)((( int16_t) calib[5] << 8) | calib[4]);
  _dig_P1 = (uint16_t)(((uint16_t) calib[7] << 8) | calib[6]);
  _dig_P2 = ( int16_t)((( int16_t) calib[9] << 8) | calib[8]);
  _dig_P3 = ( int16_t)((( int16_t) calib[11] << 8) | calib[10]);
  _dig_P4 = ( int16_t)((( int16_t) calib[13] << 8) | calib[12]);
  _dig_P5 = ( int16_t)((( int16_t) calib[15] << 8) | calib[14]);
  _dig_P6 = ( int16_t)((( int16_t) calib[17] << 8) | calib[16]);
  _dig_P7 = ( int16_t)((( int16_t) calib[19] << 8) | calib[18]);
  _dig_P8 = ( int16_t)((( int16_t) calib[21] << 8) | calib[20]);
  _dig_P9 = ( int16_t)((( int16_t) calib[23] << 8) | calib[22]);
  _dig_H1 = calib[25];
  _i2c_bus->readBytes(BME280_ADDRESS, BME280_CALIB26, 7, &calib[0]);
  _dig_H2 = ( int16_t)((( int16_t) calib[1] << 8) | calib[0]);
  _dig_H3 = calib[2];
  _dig_H4 = ( int16_t)(((( int16_t) calib[3] << 8) | (0x0F & calib[4]) << 4) >> 4);
  _dig_H5 = ( int16_t)(((( int16_t) calib[5] << 8) | (0xF0 & calib[4]) ) >> 4 );
  _dig_H6 = calib[6];
  Serial.println("Calibration coeficients:");
  Serial.print("_dig_T1 ="); 
  Serial.println(_dig_T1);
  Serial.print("_dig_T2 ="); 
  Serial.println(_dig_T2);
  Serial.print("_dig_T3 ="); 
  Serial.println(_dig_T3);
  Serial.print("_dig_P1 ="); 
  Serial.println(_dig_P1);
  Serial.print("_dig_P2 ="); 
  Serial.println(_dig_P2);
  Serial.print("_dig_P3 ="); 
  Serial.println(_dig_P3);
  Serial.print("_dig_P4 ="); 
  Serial.println(_dig_P4);
  Serial.print("_dig_P5 ="); 
  Serial.println(_dig_P5);
  Serial.print("_dig_P6 ="); 
  Serial.println(_dig_P6);
  Serial.print("_dig_P7 ="); 
  Serial.println(_dig_P7);
  Serial.print("_dig_P8 ="); 
  Serial.println(_dig_P8);
  Serial.print("_dig_P9 ="); 
  Serial.println(_dig_P9);
  Serial.print("_dig_H1 ="); 
  Serial.println(_dig_H1);
  Serial.print("_dig_H2 ="); 
  Serial.println(_dig_H2);
  Serial.print("_dig_H3 ="); 
  Serial.println(_dig_H3);
  Serial.print("_dig_H4 ="); 
  Serial.println(_dig_H4);
  Serial.print("_dig_H5 ="); 
  Serial.println(_dig_H5);
  Serial.print("_dig_H6 ="); 
  Serial.println(_dig_H6);
}


// Returns temperature in DegC, resolution is 0.01 DegC. Output value of
// “5123” equals 51.23 DegC.
int32_t BME280::BME280_compensate_T(int32_t adc_T)
{
  int32_t var1, var2, T;
  var1 = ((((adc_T >> 3) - ((int32_t)_dig_T1 << 1))) * ((int32_t)_dig_T2)) >> 11;
  var2 = (((((adc_T >> 4) - ((int32_t)_dig_T1)) * ((adc_T >> 4) - ((int32_t)_dig_T1))) >> 12) * ((int32_t)_dig_T3)) >> 14;
  _t_fine = var1 + var2;
  T = (_t_fine * 5 + 128) >> 8;
  return T;
}


// Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8
//fractional bits).
//Output value of “24674867” represents 24674867/256 = 96386.2 Pa = 963.862 hPa
uint32_t BME280::BME280_compensate_P(int32_t adc_P)
{
  long long var1, var2, p;
  var1 = ((long long)_t_fine) - 128000;
  var2 = var1 * var1 * (long long)_dig_P6;
  var2 = var2 + ((var1*(long long)_dig_P5)<<17);
  var2 = var2 + (((long long)_dig_P4)<<35);
  var1 = ((var1 * var1 * (long long)_dig_P3)>>8) + ((var1 * (long long)_dig_P2)<<12);
  var1 = (((((long long)1)<<47)+var1))*((long long)_dig_P1)>>33;
  if(var1 == 0)
  {
    return 0;
    // avoid exception caused by division by zero
  }
  p = 1048576 - adc_P;
  p = (((p<<31) - var2)*3125)/var1;
  var1 = (((long long)_dig_P9) * (p>>13) * (p>>13)) >> 25;
  var2 = (((long long)_dig_P8) * p)>> 19;
  p = ((p + var1 + var2) >> 8) + (((long long)_dig_P7)<<4);
  return (uint32_t)p;
}


// Returns humidity in %RH as unsigned 32 bit integer in Q22.10 format (22integer and 10fractional bits).
// Output value of “47445”represents 47445/1024= 46.333%RH
uint32_t BME280::BME280_compensate_H(int32_t adc_H)
{
int32_t var;

var = (_t_fine - ((int32_t)76800));
var = (((((adc_H << 14) - (((int32_t)_dig_H4) << 20) - (((int32_t)_dig_H5) * var)) +
((int32_t)16384)) >> 15) * (((((((var * ((int32_t)_dig_H6)) >> 10) * (((var *
((int32_t)_dig_H3)) >> 11) + ((int32_t)32768))) >> 10) + ((int32_t)2097152)) * ((int32_t)_dig_H2) + 8192) >> 14));
var = (var - (((((var >> 15) * (var >> 15)) >> 7) * ((int32_t)_dig_H1)) >> 4));
var = (var < 0 ? 0 : var); 
var = (var > 419430400 ? 419430400 : var);
return(uint32_t)(var >> 12);
}
