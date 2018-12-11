/* 09/23/2017 Copyright Tlera Corporation

    Created by Kris Winer

  This sketch uses SDA/SCL on pins 21/20 (Butterfly default), respectively, and it uses the Butterfly STM32L433CU Breakout Board.
  The LPS22HB is a low power barometerr.

  Library may be used freely and without limit with attribution.

*/

#include "LPS22HB.h"
#include "Wire.h"

LPS22H::LPS22H(uint8_t intPin, I2Cdev* i2c_bus)
{
  _intPin = intPin;
  _i2c_bus = i2c_bus; 
}

uint8_t LPS22H::getChipID()
{
  // Read the WHO_AM_I register of the altimeter this is a good test of communication
  uint8_t temp = _i2c_bus->readByte(LPS22H_ADDRESS, LPS22H_WHOAMI);  // Read WHO_AM_I register for LPS22H
  return temp;
}

uint8_t LPS22H::status()
{
  // Read the status register of the altimeter  
  uint8_t temp = _i2c_bus->readByte(LPS22H_ADDRESS, LPS22H_STATUS);   
  return temp;
}

int32_t LPS22H::readAltimeterPressure()
{
    uint8_t rawData[3];  // 24-bit pressure register data stored here
    _i2c_bus->readBytes(LPS22H_ADDRESS, (LPS22H_PRESS_OUT_XL | 0x80), 3, &rawData[0]); // bit 7 must be one to read multiple bytes
    return (int32_t) ((int32_t) rawData[2] << 16 | (int32_t) rawData[1] << 8 | rawData[0]);
}

int16_t LPS22H::readAltimeterTemperature()
{
    uint8_t rawData[2];  // 16-bit pressure register data stored here
    _i2c_bus->readBytes(LPS22H_ADDRESS, (LPS22H_TEMP_OUT_L | 0x80), 2, &rawData[0]); // bit 7 must be one to read multiple bytes
    return (int16_t)((int16_t) rawData[1] << 8 | rawData[0]);
}


void LPS22H::Init(uint8_t PODR)
{
  // set sample rate by setting bits 6:4 
  // enable low-pass filter by setting bit 3 to one
  // bit 2 == 0 means bandwidth is odr/9, bit 2 == 1 means bandwidth is odr/20
  // make sure data not updated during read by setting block data udate (bit 1) to 1
    _i2c_bus->writeByte(LPS22H_ADDRESS, LPS22H_CTRL_REG1, PODR << 4 | 0x08 | 0x02);  
    _i2c_bus->writeByte(LPS22H_ADDRESS, LPS22H_CTRL_REG3, 0x04);  // enable data ready as interrupt source
}


