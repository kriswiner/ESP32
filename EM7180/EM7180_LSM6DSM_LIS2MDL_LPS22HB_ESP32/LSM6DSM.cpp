/* 09/23/2017 Copyright Tlera Corporation

    Created by Kris Winer

  This sketch uses SDA/SCL on pins 21/20 (Butterfly default), respectively, and it uses the Butterfly STM32L433CU Breakout Board.
  The LSM6DSM is a sensor hub with embedded accel and gyro, here used as 6 DoF in a 9 DoF absolute orientation solution.

  Library may be used freely and without limit with attribution.

*/

#include "LSM6DSM.h"

LSM6DSM::LSM6DSM(uint8_t intPin1, uint8_t intPin2, I2Cdev* i2c_bus)
{
  _intPin1 = intPin1;
  _intPin2 = intPin2;  
  _i2c_bus = i2c_bus; 
}


uint8_t LSM6DSM::getChipID()
{
  uint8_t c = _i2c_bus->readByte(LSM6DSM_ADDRESS, LSM6DSM_WHO_AM_I);
  return c;
}

float LSM6DSM::getAres(uint8_t Ascale) {
  switch (Ascale)
  {
  // Possible accelerometer scales (and their register bit settings) are:
  // 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11). 
        // Here's a bit of an algorithm to calculate DPS/(ADC tick) based on that 2-bit value:
    case AFS_2G:
         _aRes = 2.0f/32768.0f;
         return _aRes;
         break;
    case AFS_4G:
         _aRes = 4.0f/32768.0f;
         return _aRes;
         break;
    case AFS_8G:
         _aRes = 8.0f/32768.0f;
         return _aRes;
         break;
    case AFS_16G:
         _aRes = 16.0f/32768.0f;
         return _aRes;
         break;
  }
}

float LSM6DSM::getGres(uint8_t Gscale) {
  switch (Gscale)
  {
  // Possible gyro scales (and their register bit settings) are:
  // 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11). 
    case GFS_245DPS:
          _gRes = 245.0f/32768.0f;
          return _gRes;
          break;
    case GFS_500DPS:
          _gRes = 500.0f/32768.0f;
          return _gRes;
          break;
    case GFS_1000DPS:
         _gRes = 1000.0f/32768.0f;
         return _gRes;
         break;
    case GFS_2000DPS:
          _gRes = 2000.0f/32768.0f;
         return _gRes;
         break;
  }
}


void LSM6DSM::reset()
{
  // reset device
  uint8_t temp = _i2c_bus->readByte(LSM6DSM_ADDRESS, LSM6DSM_CTRL3_C);
  _i2c_bus->writeByte(LSM6DSM_ADDRESS, LSM6DSM_CTRL3_C, temp | 0x01); // Set bit 0 to 1 to reset LSM6DSM
  delay(100); // Wait for all registers to reset 
}


void LSM6DSM::init(uint8_t Ascale, uint8_t Gscale, uint8_t AODR, uint8_t GODR)
{
  _i2c_bus->writeByte(LSM6DSM_ADDRESS, LSM6DSM_CTRL1_XL, AODR << 4 | Ascale << 2);
  
  _i2c_bus->writeByte(LSM6DSM_ADDRESS, LSM6DSM_CTRL2_G, GODR << 4 | Gscale << 2);
 
  uint8_t temp = _i2c_bus->readByte(LSM6DSM_ADDRESS, LSM6DSM_CTRL3_C);
  // enable block update (bit 6 = 1), auto-increment registers (bit 2 = 1)
  _i2c_bus->writeByte(LSM6DSM_ADDRESS, LSM6DSM_CTRL3_C, temp | 0x40 | 0x04); 
  // by default, interrupts active HIGH, push pull, little endian data 
  // (can be changed by writing to bits 5, 4, and 1, resp to above register)

   // enable accel LP2 (bit 7 = 1), set LP2 tp ODR/9 (bit 6 = 1), enable input_composite (bit 3) for low noise
   _i2c_bus->writeByte(LSM6DSM_ADDRESS, LSM6DSM_CTRL8_XL, 0x80 | 0x40 | 0x08 );

   // interrupt handling
    _i2c_bus->writeByte(LSM6DSM_ADDRESS, LSM6DSM_DRDY_PULSE_CFG, 0x80); // latch interrupt until data read
    _i2c_bus->writeByte(LSM6DSM_ADDRESS, LSM6DSM_INT1_CTRL, 0x40);      // enable significant motion interrupts on INT1
    _i2c_bus->writeByte(LSM6DSM_ADDRESS, LSM6DSM_INT2_CTRL, 0x03);      // enable accel/gyro data ready interrupts on INT2  
}


void LSM6DSM::selfTest()
{
  int16_t temp[7] = {0, 0, 0, 0, 0, 0, 0};
  int16_t accelPTest[3] = {0, 0, 0}, accelNTest[3] = {0, 0, 0}, gyroPTest[3] = {0, 0, 0}, gyroNTest[3] = {0, 0, 0};
  int16_t accelNom[3] = {0, 0, 0}, gyroNom[3] = {0, 0, 0};

  readData(temp);
  accelNom[0] = temp[4];
  accelNom[1] = temp[5];
  accelNom[2] = temp[6];
  gyroNom[0]  = temp[1];
  gyroNom[1]  = temp[2];
  gyroNom[2]  = temp[3];
  
  _i2c_bus->writeByte(LSM6DSM_ADDRESS, LSM6DSM_CTRL5_C, 0x01); // positive accel self test
  delay(100); // let accel respond
  readData(temp);
  accelPTest[0] = temp[4];
  accelPTest[1] = temp[5];
  accelPTest[2] = temp[6];

  _i2c_bus->writeByte(LSM6DSM_ADDRESS, LSM6DSM_CTRL5_C, 0x03); // negative accel self test
  delay(100); // let accel respond
  readData(temp);
  accelNTest[0] = temp[4];
  accelNTest[1] = temp[5];
  accelNTest[2] = temp[6];

  _i2c_bus->writeByte(LSM6DSM_ADDRESS, LSM6DSM_CTRL5_C, 0x04); // positive gyro self test
  delay(100); // let gyro respond
  readData(temp);
  gyroPTest[0] = temp[1];
  gyroPTest[1] = temp[2];
  gyroPTest[2] = temp[3];

  _i2c_bus->writeByte(LSM6DSM_ADDRESS, LSM6DSM_CTRL5_C, 0x0C); // negative gyro self test
  delay(100); // let gyro respond
  readData(temp);
  gyroNTest[0] = temp[1];
  gyroNTest[1] = temp[2];
  gyroNTest[2] = temp[3];

  _i2c_bus->writeByte(LSM6DSM_ADDRESS, LSM6DSM_CTRL5_C, 0x00); // normal mode
  delay(100); // let accel and gyro respond

  Serial.println("Accel Self Test:");
  Serial.print("+Ax results:"); Serial.print(  (accelPTest[0] - accelNom[0]) * _aRes * 1000.0); Serial.println(" mg");
  Serial.print("-Ax results:"); Serial.println((accelNTest[0] - accelNom[0]) * _aRes * 1000.0);
  Serial.print("+Ay results:"); Serial.println((accelPTest[1] - accelNom[1]) * _aRes * 1000.0);
  Serial.print("-Ay results:"); Serial.println((accelNTest[1] - accelNom[1]) * _aRes * 1000.0);
  Serial.print("+Az results:"); Serial.println((accelPTest[2] - accelNom[2]) * _aRes * 1000.0);
  Serial.print("-Az results:"); Serial.println((accelNTest[2] - accelNom[2]) * _aRes * 1000.0);
  Serial.println("Should be between 90 and 1700 mg");

  Serial.println("Gyro Self Test:");
  Serial.print("+Gx results:"); Serial.print((gyroPTest[0] - gyroNom[0]) * _gRes); Serial.println(" dps");
  Serial.print("-Gx results:"); Serial.println((gyroNTest[0] - gyroNom[0]) * _gRes);
  Serial.print("+Gy results:"); Serial.println((gyroPTest[1] - gyroNom[1]) * _gRes);
  Serial.print("-Gy results:"); Serial.println((gyroNTest[1] - gyroNom[1]) * _gRes);
  Serial.print("+Gz results:"); Serial.println((gyroPTest[2] - gyroNom[2]) * _gRes);
  Serial.print("-Gz results:"); Serial.println((gyroNTest[2] - gyroNom[2]) * _gRes);
  Serial.println("Should be between 20 and 80 dps");
  delay(2000);

 
}


void LSM6DSM::offsetBias(float * dest1, float * dest2)
{
  int16_t temp[7] = {0, 0, 0, 0, 0, 0, 0};
  int32_t sum[7] = {0, 0, 0, 0, 0, 0, 0};
    
  Serial.println("Calculate accel and gyro offset biases: keep sensor flat and motionless!");
  delay(4000);

  for (int ii = 0; ii < 128; ii++)
  {
    readData(temp);
    sum[1] += temp[1];
    sum[2] += temp[2];
    sum[3] += temp[3];
    sum[4] += temp[4];
    sum[5] += temp[5];
    sum[6] += temp[6];
    delay(50);
  }

  dest1[0] = sum[1]*_gRes/128.0f;
  dest1[1] = sum[2]*_gRes/128.0f;
  dest1[2] = sum[3]*_gRes/128.0f;
  dest2[0] = sum[4]*_aRes/128.0f;
  dest2[1] = sum[5]*_aRes/128.0f;
  dest2[2] = sum[6]*_aRes/128.0f;

  if(dest2[0] > 0.8f)  {dest2[0] -= 1.0f;}  // Remove gravity from the x-axis accelerometer bias calculation
  if(dest2[0] < -0.8f) {dest2[0] += 1.0f;}  // Remove gravity from the x-axis accelerometer bias calculation
  if(dest2[1] > 0.8f)  {dest2[1] -= 1.0f;}  // Remove gravity from the y-axis accelerometer bias calculation
  if(dest2[1] < -0.8f) {dest2[1] += 1.0f;}  // Remove gravity from the y-axis accelerometer bias calculation
  if(dest2[2] > 0.8f)  {dest2[2] -= 1.0f;}  // Remove gravity from the z-axis accelerometer bias calculation
  if(dest2[2] < -0.8f) {dest2[2] += 1.0f;}  // Remove gravity from the z-axis accelerometer bias calculation

}


void LSM6DSM::readData(int16_t * destination)
{
  uint8_t rawData[14];  // x/y/z accel register data stored here
  _i2c_bus->readBytes(LSM6DSM_ADDRESS, LSM6DSM_OUT_TEMP_L, 14, &rawData[0]);  // Read the 14 raw data registers into data array
  destination[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;  
  destination[2] = ((int16_t)rawData[5] << 8) | rawData[4] ; 
  destination[3] = ((int16_t)rawData[7] << 8) | rawData[6] ;   
  destination[4] = ((int16_t)rawData[9] << 8) | rawData[8] ;  
  destination[5] = ((int16_t)rawData[11] << 8) | rawData[10] ;  
  destination[6] = ((int16_t)rawData[13] << 8) | rawData[12] ; 
}
