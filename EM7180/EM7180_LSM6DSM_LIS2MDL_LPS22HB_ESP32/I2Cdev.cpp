/*
 * Copyright (c) 2018 Tlera Corp.  All rights reserved.
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

#include "Arduino.h"
#include "I2Cdev.h"

I2Cdev::I2Cdev(TwoWire* i2c_bus)                                                                                                             // Class constructor
{
  _i2c_bus = i2c_bus;
}

I2Cdev::~I2Cdev()                                                                                                                            // Class destructor
{
}

/**
* @fn: readByte(uint8_t address, uint8_t subAddress)
*
* @brief: Read one byte from an I2C device
* 
* @params: I2C slave device address, Register subAddress
* @returns: unsigned short read
*/
uint8_t I2Cdev::readByte(uint8_t address, uint8_t subAddress)
{
  uint8_t data = 0;                             // `data` will store the register data   
  _i2c_bus->beginTransmission(address);         // Initialize the Tx buffer
  _i2c_bus->write(subAddress);                  // Put slave register address in Tx buffer
  _i2c_bus->endTransmission(false);             // Send the Tx buffer, but send a restart to keep connection alive
  _i2c_bus->requestFrom(address, 1);            // Read one byte from slave register address  
  data = _i2c_bus->read();                      // Fill Rx buffer with result
  return data;                                  // Return data read from slave register
  
}


/**
* @fn: readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest)
*
* @brief: Read multiple bytes from an I2C device
* 
* @params: I2C slave device address, Register subAddress, number of btes to be read, aray to store the read data
* @returns: void
*/
void I2Cdev::readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest)
{  
  _i2c_bus->beginTransmission(address);   // Initialize the Tx buffer
  _i2c_bus->write(subAddress);            // Put slave register address in Tx buffer
  _i2c_bus->endTransmission(false);       // Send the Tx buffer, but send a restart to keep connection alive
  uint8_t i = 0;
  _i2c_bus->requestFrom(address, count);  // Read bytes from slave register address 
  while (_i2c_bus->available()) {
        dest[i++] = _i2c_bus->read(); }   // Put read results in the Rx buffer
}


/**
* @fn: writeByte(uint8_t devAddr, uint8_t regAddr, uint8_t data)
*
* @brief: Write one byte to an I2C device
* 
* @params: I2C slave device address, Register subAddress, data to be written
* @returns: void
*/
void I2Cdev::writeByte(uint8_t devAddr, uint8_t regAddr, uint8_t data)
{
  _i2c_bus->beginTransmission(devAddr);  // Initialize the Tx buffer
  _i2c_bus->write(regAddr);           // Put slave register address in Tx buffer
  _i2c_bus->write(data);                 // Put data in Tx buffer
  _i2c_bus->endTransmission();           // Send the Tx buffer
}


/**
* @fn: writeBytes(uint8_t devAddr, uint8_t regAddr, uint8_t data)
*
* @brief: Write multiple bytes to an I2C device
* 
* @params: I2C slave device address, Register subAddress, byte count, data array to be written
* @returns: void
*/
void I2Cdev::writeBytes(uint8_t devAddr, uint8_t regAddr, uint8_t count, uint8_t *dest)
{
  uint8_t temp[1 + count];
  
  temp[0] = regAddr;
  for (uint8_t ii = 0; ii < count; ii++)
  { 
    temp[ii + 1] = dest[ii];
  }
  
  _i2c_bus->beginTransmission(devAddr);  // Initialize the Tx buffer
  
  for (uint8_t jj = 0; jj < count + 1; jj++)
  {
  _i2c_bus->write(temp[jj]);            // Put data in Tx buffer
  }
  
  _i2c_bus->endTransmission();           // Send the Tx buffer
}



/**
* @fn:I2Cscan()
* @brief: Scan the I2C bus for active I2C slave devices
* 
* @params: void
* @returns: void
*/
void I2Cdev::I2Cscan() 
{
  // Scan for i2c devices
  byte error, address;
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for(address = 1; address < 127; address++ ) 
  {
    // The i2c_scanner uses the return value of the Wire.endTransmisstion to see if a device did acknowledge to the address.
    _i2c_bus->beginTransmission(address);
    error = _i2c_bus->endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16) 
      Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");
      nDevices++;
    }
    else if (error==4) 
    {
      Serial.print("Unknown error at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("I2C scan complete\n");
}

