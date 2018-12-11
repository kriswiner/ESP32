/* 09/23/2017 Copyright Tlera Corporation

    Created by Kris Winer

  This sketch uses SDA/SCL on pins 21/20 (Butterfly default), respectively, and it uses the Butterfly STM32L433CU Breakout Board.
  The LIS2MDL is a low power magnetometer, here used as 3 DoF in a 9 DoF absolute orientation solution.

  Library may be used freely and without limit with attribution.

*/

#include "LIS2MDL.h"

LIS2MDL::LIS2MDL(uint8_t intPin, I2Cdev* i2c_bus)
{
  _intPin = intPin;
  _i2c_bus = i2c_bus;
}


uint8_t LIS2MDL::getChipID()
{
  uint8_t c = _i2c_bus->readByte(LIS2MDL_ADDRESS, LIS2MDL_WHO_AM_I);
  return c;
}


void LIS2MDL::reset()
{
  // reset device
  uint8_t temp = _i2c_bus->readByte(LIS2MDL_ADDRESS, LIS2MDL_CFG_REG_A);
  _i2c_bus->writeByte(LIS2MDL_ADDRESS, LIS2MDL_CFG_REG_A, temp | 0x20); // Set bit 5 to 1 to reset LIS2MDL
  delay(1);
  _i2c_bus->writeByte(LIS2MDL_ADDRESS, LIS2MDL_CFG_REG_A, temp | 0x40); // Set bit 6 to 1 to boot LIS2MDL
  delay(100); // Wait for all registers to reset 
}

void LIS2MDL::init(uint8_t MODR)
{
 // enable temperature compensation (bit 7 == 1), continuous mode (bits 0:1 == 00)
 _i2c_bus->writeByte(LIS2MDL_ADDRESS, LIS2MDL_CFG_REG_A, 0x80 | MODR<<2);  

 // enable low pass filter (bit 0 == 1), set to ODR/4
 _i2c_bus->writeByte(LIS2MDL_ADDRESS, LIS2MDL_CFG_REG_B, 0x01);  

 // enable data ready on interrupt pin (bit 0 == 1), enable block data read (bit 4 == 1)
 _i2c_bus->writeByte(LIS2MDL_ADDRESS, LIS2MDL_CFG_REG_C, 0x01 | 0x10);  
}


uint8_t LIS2MDL::status()
{
  // Read the status register of the altimeter  
  uint8_t temp = _i2c_bus->readByte(LIS2MDL_ADDRESS, LIS2MDL_STATUS_REG);   
  return temp;
}


void LIS2MDL::readData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z mag register data stored here
  _i2c_bus->readBytes(LIS2MDL_ADDRESS, (0x80 | LIS2MDL_OUTX_L_REG), 8, &rawData[0]);  // Read the 6 raw data registers into data array

  destination[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;       // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;  
  destination[2] = ((int16_t)rawData[5] << 8) | rawData[4] ; 
}


int16_t LIS2MDL::readTemperature()
{
  uint8_t rawData[2];  // x/y/z mag register data stored here
  _i2c_bus->readBytes(LIS2MDL_ADDRESS, (0x80 | LIS2MDL_TEMP_OUT_L_REG), 2, &rawData[0]);  // Read the 8 raw data registers into data array
  int16_t temp = ((int16_t)rawData[1] << 8) | rawData[0] ;       // Turn the MSB and LSB into a signed 16-bit value
  return temp;
}


void LIS2MDL::offsetBias(float * dest1, float * dest2)
{
  int32_t mag_bias[3] = {0, 0, 0}, mag_scale[3] = {0, 0, 0};
  int16_t mag_max[3] = {-32767, -32767, -32767}, mag_min[3] = {32767, 32767, 32767}, mag_temp[3] = {0, 0, 0};
  float _mRes = 0.0015f;
  
  Serial.println("Calculate mag offset bias: move all around to sample the complete response surface!");
  delay(4000);

  for (int ii = 0; ii < 4000; ii++)
  {
    readData(mag_temp);
       for (int jj = 0; jj < 3; jj++) {
      if(mag_temp[jj] > mag_max[jj]) mag_max[jj] = mag_temp[jj];
      if(mag_temp[jj] < mag_min[jj]) mag_min[jj] = mag_temp[jj];
    }
    delay(12);
  }

  _mRes = 0.0015f; // fixed sensitivity
    // Get hard iron correction
    mag_bias[0]  = (mag_max[0] + mag_min[0])/2;  // get average x mag bias in counts
    mag_bias[1]  = (mag_max[1] + mag_min[1])/2;  // get average y mag bias in counts
    mag_bias[2]  = (mag_max[2] + mag_min[2])/2;  // get average z mag bias in counts
    
    dest1[0] = (float) mag_bias[0] * _mRes;  // save mag biases in G for main program
    dest1[1] = (float) mag_bias[1] * _mRes;   
    dest1[2] = (float) mag_bias[2] * _mRes;  
       
    // Get soft iron correction estimate
    mag_scale[0]  = (mag_max[0] - mag_min[0])/2;  // get average x axis max chord length in counts
    mag_scale[1]  = (mag_max[1] - mag_min[1])/2;  // get average y axis max chord length in counts
    mag_scale[2]  = (mag_max[2] - mag_min[2])/2;  // get average z axis max chord length in counts

    float avg_rad = mag_scale[0] + mag_scale[1] + mag_scale[2];
    avg_rad /= 3.0f;

    dest2[0] = avg_rad/((float)mag_scale[0]);
    dest2[1] = avg_rad/((float)mag_scale[1]);
    dest2[2] = avg_rad/((float)mag_scale[2]);
  
   Serial.println("Mag Calibration done!");
}

void LIS2MDL::selfTest()
{
  int16_t temp[3] = {0, 0, 0};
  float magTest[3] = {0., 0., 0.};
  float magNom[3] = {0., 0., 0.};
  int32_t sum[3] = {0, 0, 0};
  float _mRes = 0.0015f;
    
  // first, get average response with self test disabled
  for (int ii = 0; ii < 50; ii++)
  {
    readData(temp);
    sum[0] += temp[0];
    sum[1] += temp[1];
    sum[2] += temp[2];
    delay(50);
  }
  
  magNom[0] = (float) sum[0] / 50.0f;
  magNom[1] = (float) sum[1] / 50.0f;
  magNom[2] = (float) sum[2] / 50.0f;
  
  uint8_t c = _i2c_bus->readByte(LIS2MDL_ADDRESS, LIS2MDL_CFG_REG_C);
  _i2c_bus->writeByte(LIS2MDL_ADDRESS, LIS2MDL_CFG_REG_C, c | 0x02); // enable self test
  delay(100); // let mag respond
  
  sum[0] = 0;
  sum[1] = 0;
  sum[2] = 0;
  for (int ii = 0; ii < 50; ii++)
  {
    readData(temp);
    sum[0] += temp[0];
    sum[1] += temp[1];
    sum[2] += temp[2];
    delay(50);
  }
  
  magTest[0] = (float) sum[0] / 50.0f;
  magTest[1] = (float) sum[1] / 50.0f;
  magTest[2] = (float) sum[2] / 50.0f;
  
  _i2c_bus->writeByte(LIS2MDL_ADDRESS, LIS2MDL_CFG_REG_C, c); // return to previous settings/normal mode
  delay(100); // let mag respond

  Serial.println("Mag Self Test:");
  Serial.print("Mx results:"); Serial.print(  (magTest[0] - magNom[0]) * _mRes * 1000.0); Serial.println(" mG");
  Serial.print("My results:"); Serial.println((magTest[0] - magNom[0]) * _mRes * 1000.0);
  Serial.print("Mz results:"); Serial.println((magTest[1] - magNom[1]) * _mRes * 1000.0);
  Serial.println("Should be between 15 and 500 mG");
  delay(2000);  // give some time to read the screen
}

