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
  
#ifndef HDC2010_h
#define HDC2010_h

#include "Arduino.h"
#include <Wire.h>
#include "I2Cdev.h"

// http://www.ti.com/lit/ds/symlink/hdc2010.pdf
// HDC2010 Registers
#define HDC2010_TEMP_L              0x00
#define HDC2010_TEMP_H              0x01 
#define HDC2010_HUM_L               0x02
#define HDC2010_HUM_H               0x03
#define HDC2010_INT_STATUS          0x04
#define HDC2010_TEMP_MAX            0x05
#define HDC2010_HUM_MAX             0x06
#define HDC2010_INT_EN              0x07
#define HDC2010_TEMP_OFFSET         0x08
#define HDC2010_HUM_OFFSET          0x09
#define HDC2010_TEMP_THR_L          0x0A
#define HDC2010_TEMP_THR_H          0x0B
#define HDC2010_RH_THR_L            0x0C
#define HDC2010_RH_THR_H            0x0D
#define HDC2010_CONFIG1             0x0E
#define HDC2010_CONFIG2             0x0F
#define HDC2010_MANU_ID_L           0xFC
#define HDC2010_MANU_ID_H           0xFD
#define HDC2010_DEV_ID_L            0xFE
#define HDC2010_DEV_ID_H            0xFF

// I2C address  
#define HDC2010_0_ADDRESS 0x40 // SA0 = 0
#define HDC2010_1_ADDRESS 0x41 // SA0 = 1

// Measurement Frequency (inverse sample rate)
#define ForceMode    0x00
#define Freq_120s    0x01
#define Freq_60s     0x02
#define Freq_10s     0x03
#define Freq_5s      0x04
#define Freq_1s      0x05 // 1 Hz
#define Freq_0_5s    0x06 // 2 Hz
#define Freq_0_2s    0x07 // 5 Hz

// Temperature resolution
#define TRES_14bit    0x00
#define TRES_11bit    0x01
#define TRES_9bit     0x02

// Humidity resolution
#define HRES_14bit    0x00
#define HRES_11bit    0x01
#define HRES_9bit     0x02
 


class HDC2010
{
  public: 
  HDC2010(I2Cdev* i2c_bus);
  void reset(uint8_t HDC2010_ADDRESS);
  uint16_t getDevID(uint8_t HDC2010_ADDRESS);
  uint16_t getManuID(uint8_t HDC2010_ADDRESS);
  void init(uint8_t HDC2010_ADDRESS, uint8_t hres, uint8_t tres, uint8_t freq);  
  uint8_t getIntStatus(uint8_t HDC2010_ADDRESS);
  void heaterOn(uint8_t HDC2010_ADDRESS);
  void heaterOff(uint8_t HDC2010_ADDRESS);
  float getTemperature(uint8_t HDC2010_ADDRESS);
  float getHumidity(uint8_t HDC2010_ADDRESS);
  uint16_t getRawTemperature(uint8_t HDC2010_ADDRESS);
  uint16_t getRawHumidity(uint8_t HDC2010_ADDRESS);
  void forcedMode(uint8_t HDC2010_ADDRESS);
  
  private:
  // Register read variables
  I2Cdev* _i2c_bus;
  };

#endif
