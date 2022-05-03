/*
 * Copyright (c) 2021 Tlera Corp.  All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to
 * deal with the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 *  1. Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimers.
 *  2. Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimers in the
 *     documentation and/or other materials provided with the distribution.
 *  3. Neither the name of Tlera Corp, nor the names of its contributors
 *     may be used to endorse or promote products derived from this Software
 *     without specific prior written permission.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * CONTRIBUTORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * WITH THE SOFTWARE.
 */

#include "HDC2010.h"

  HDC2010::HDC2010(I2Cdev* i2c_bus){
  _i2c_bus = i2c_bus;
  }


  void HDC2010::reset(uint8_t HDC2010_ADDRESS) {
  _i2c_bus->writeByte(HDC2010_ADDRESS, HDC2010_INT_STATUS, 0x80);  // soft reset device
  delay(1);
  }


  uint16_t HDC2010::getDevID(uint8_t HDC2010_ADDRESS)
  {
  uint8_t rawData[2] = {0, 0};
  rawData[0] = _i2c_bus->readByte(HDC2010_ADDRESS, HDC2010_DEV_ID_L); // read Dev ID LSByte
  rawData[1] = _i2c_bus->readByte(HDC2010_ADDRESS, HDC2010_DEV_ID_H); // read Dev ID HSByte
  uint16_t devID = ( (uint16_t) rawData[1] << 8) | rawData[0];
  return devID;
  }


  uint16_t HDC2010::getManuID(uint8_t HDC2010_ADDRESS)
  {
  uint8_t rawData[2] = {0, 0};
  rawData[0] = _i2c_bus->readByte(HDC2010_ADDRESS, HDC2010_MANU_ID_L); // read Dev ID LSByte
  rawData[1] = _i2c_bus->readByte(HDC2010_ADDRESS, HDC2010_MANU_ID_H); // read Dev ID HSByte
  uint16_t manuID = ( (uint16_t) rawData[1] << 8) | rawData[0];
  return manuID;
  }


  uint8_t HDC2010::getIntStatus(uint8_t HDC2010_ADDRESS)
  {
  uint8_t c  = _i2c_bus->readByte(HDC2010_ADDRESS, HDC2010_INT_STATUS); // read int status register
  return c;
  }
  

  void HDC2010::init(uint8_t HDC2010_ADDRESS, uint8_t hres, uint8_t tres, uint8_t freq)
  {
  // set sample frequency (bits 6 - 4), enable interrupt (bit 2 = 1), active HIGH (bit 1 = 1)
  _i2c_bus->writeByte(HDC2010_ADDRESS, HDC2010_CONFIG1, freq << 4 | 0x04 | 0x02);  
  // set temperature resolution (bits 7:6), set humidity resolution (bits 5:4), measure both H and T
  // start measurements by writing 1 to bit 0
  _i2c_bus->writeByte(HDC2010_ADDRESS, HDC2010_CONFIG2, tres << 6 | hres << 4 | 0x01);  
  // enable data ready interrupt (bit 7), can enable interrupt on thresholds here too
  _i2c_bus->writeByte(HDC2010_ADDRESS, HDC2010_INT_EN, 0x80); 
  }


  float HDC2010::getTemperature(uint8_t HDC2010_ADDRESS)
  {
  uint8_t rawData[2] = {0, 0};
  rawData[0] = _i2c_bus->readByte(HDC2010_ADDRESS, HDC2010_TEMP_L); // read Temp LSByte
  rawData[1] = _i2c_bus->readByte(HDC2010_ADDRESS, HDC2010_TEMP_H); // read Temp HSByte
  uint16_t temp = (uint16_t) ( ((uint16_t) rawData[1] << 8 ) | rawData[0]);
  float out = ((float) temp) * (165.0f/65536.0f) - 40.0f;
  return out;
  }


  float HDC2010::getHumidity(uint8_t HDC2010_ADDRESS)
  {
  uint8_t rawData[2] = {0, 0};
  rawData[0] = _i2c_bus->readByte(HDC2010_ADDRESS, HDC2010_HUM_L); // read Temp LSByte
  rawData[1] = _i2c_bus->readByte(HDC2010_ADDRESS, HDC2010_HUM_H); // read Temp HSByte
  uint16_t temp = (uint16_t) ( ((uint16_t) rawData[1] << 8 ) | rawData[0]);
  float out = ((float) temp) * (100.0f/65536.0f);
  return out;
  }

  uint16_t HDC2010::getRawTemperature(uint8_t HDC2010_ADDRESS)
  {
  uint8_t rawData[2] = {0, 0};
  rawData[0] = _i2c_bus->readByte(HDC2010_ADDRESS, HDC2010_TEMP_L); // read Temp LSByte
  rawData[1] = _i2c_bus->readByte(HDC2010_ADDRESS, HDC2010_TEMP_H); // read Temp HSByte
  uint16_t temp = (uint16_t) ( ((uint16_t) rawData[1] << 8 ) | rawData[0]);
  return temp;
  }


  uint16_t HDC2010::getRawHumidity(uint8_t HDC2010_ADDRESS)
  {
  uint8_t rawData[2] = {0, 0};
  rawData[0] = _i2c_bus->readByte(HDC2010_ADDRESS, HDC2010_HUM_L); // read Temp LSByte
  rawData[1] = _i2c_bus->readByte(HDC2010_ADDRESS, HDC2010_HUM_H); // read Temp HSByte
  uint16_t temp = (uint16_t) ( ((uint16_t) rawData[1] << 8 ) | rawData[0]);
  return temp;
  }


  void HDC2010::forcedMode(uint8_t HDC2010_ADDRESS)
  {
  uint8_t temp = _i2c_bus->readByte(HDC2010_ADDRESS, HDC2010_CONFIG2);
  _i2c_bus->writeByte(HDC2010_ADDRESS, HDC2010_CONFIG2, temp | 0x01);   //start measurement, self clearing when done
  }
