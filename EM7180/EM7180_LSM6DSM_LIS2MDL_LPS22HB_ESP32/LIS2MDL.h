/* 09/23/2017 Copyright Tlera Corporation

    Created by Kris Winer

  This sketch uses SDA/SCL on pins 21/20 (Butterfly default), respectively, and it uses the Butterfly STM32L433CU Breakout Board.
  The LIS2MDL is a low power magnetometer, here used as 3 DoF in a 9 DoF absolute orientation solution.

  Library may be used freely and without limit with attribution.

*/

#ifndef LIS2MDL_h
#define LIS2MDL_h

#include "Arduino.h"
#include <Wire.h>
#include "I2Cdev.h"

//Register map for LIS2MDL'
// http://www.st.com/content/ccc/resource/technical/document/datasheet/group3/29/13/d1/e0/9a/4d/4f/30/DM00395193/files/DM00395193.pdf/jcr:content/translations/en.DM00395193.pdf
#define LIS2MDL_OFFSET_X_REG_L        0x45
#define LIS2MDL_OFFSET_X_REG_H        0x46
#define LIS2MDL_OFFSET_Y_REG_L        0x47
#define LIS2MDL_OFFSET_Y_REG_H        0x48
#define LIS2MDL_OFFSET_Z_REG_L        0x49
#define LIS2MDL_OFFSET_Z_REG_H        0x4A
#define LIS2MDL_WHO_AM_I              0x4F
#define LIS2MDL_CFG_REG_A             0x60
#define LIS2MDL_CFG_REG_B             0x61
#define LIS2MDL_CFG_REG_C             0x62
#define LIS2MDL_INT_CTRL_REG          0x63
#define LIS2MDL_INT_SOURCE_REG        0x64
#define LIS2MDL_INT_THS_L_REG         0x65
#define LIS2MDL_INT_THS_H_REG         0x66
#define LIS2MDL_STATUS_REG            0x67
#define LIS2MDL_OUTX_L_REG            0x68
#define LIS2MDL_OUTX_H_REG            0x69
#define LIS2MDL_OUTY_L_REG            0x6A
#define LIS2MDL_OUTY_H_REG            0x6B
#define LIS2MDL_OUTZ_L_REG            0x6C
#define LIS2MDL_OUTZ_H_REG            0x6D
#define LIS2MDL_TEMP_OUT_L_REG        0x6E
#define LIS2MDL_TEMP_OUT_H_REG        0x6F

#define LIS2MDL_ADDRESS               0x1E

#define MODR_10Hz   0x00
#define MODR_20Hz   0x01
#define MODR_50Hz   0x02
#define MODR_100Hz  0x03


class LIS2MDL
{
  public:
  LIS2MDL(uint8_t intPin,   I2Cdev* i2c_bus);
  uint8_t getChipID();
  void init(uint8_t MODR);
  void offsetBias(float * dest1, float * dest2);
  void reset();
  void selfTest();
  uint8_t status();
  void readData(int16_t * destination);
  int16_t readTemperature();
  void I2Cscan();
  void writeByte(uint8_t address, uint8_t subAddress, uint8_t data);
  uint8_t readByte(uint8_t address, uint8_t subAddress);
  void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest);
  private:
  uint8_t _intPin;
  float _mRes;
  I2Cdev* _i2c_bus;
};

#endif
