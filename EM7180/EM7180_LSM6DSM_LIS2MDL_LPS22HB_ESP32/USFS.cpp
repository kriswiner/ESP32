/* 06/29/2017 Copyright Tlera Corporation
 *  
 *  Created by Kris Winer
 *  
 *  
 *  Library may be used freely and without limit with attribution.
 *  
 */
#include "USFS.h"

USFS::USFS(uint8_t intPin, bool passThru, I2Cdev* i2c_bus)
{
  _intPin = intPin;
  _passThru = passThru;
  _i2c_bus = i2c_bus;    
}

void USFS::getChipID()
{
  // Read SENtral device information
  uint16_t ROM1 = _i2c_bus->readByte(EM7180_ADDRESS, EM7180_ROMVersion1);
  uint16_t ROM2 = _i2c_bus->readByte(EM7180_ADDRESS, EM7180_ROMVersion2);
  Serial.print("EM7180 ROM Version: 0x"); Serial.print(ROM1, HEX); Serial.println(ROM2, HEX); Serial.println("Should be: 0xE609");
  uint16_t RAM1 = _i2c_bus->readByte(EM7180_ADDRESS, EM7180_RAMVersion1);
  uint16_t RAM2 = _i2c_bus->readByte(EM7180_ADDRESS, EM7180_RAMVersion2);
  Serial.print("EM7180 RAM Version: 0x"); Serial.print(RAM1); Serial.println(RAM2);
  uint8_t PID = _i2c_bus->readByte(EM7180_ADDRESS, EM7180_ProductID);
  Serial.print("EM7180 ProductID: 0x"); Serial.print(PID, HEX); Serial.println(" Should be: 0x80");
  uint8_t RID = _i2c_bus->readByte(EM7180_ADDRESS, EM7180_RevisionID);
  Serial.print("EM7180 RevisionID: 0x"); Serial.print(RID, HEX); Serial.println(" Should be: 0x02");
}

 void USFS::loadfwfromEEPROM()
 {
  // Check which sensors can be detected by the EM7180
  uint8_t featureflag = _i2c_bus->readByte(EM7180_ADDRESS, EM7180_FeatureFlags);
    if(featureflag & 0x01)  Serial.println("A barometer is installed");
    if(featureflag & 0x02)  Serial.println("A humidity sensor is installed");
    if(featureflag & 0x04)  Serial.println("A temperature sensor is installed");
    if(featureflag & 0x08)  Serial.println("A custom sensor is installed");
    if(featureflag & 0x10)  Serial.println("A second custom sensor is installed");
    if(featureflag & 0x20)  Serial.println("A third custom sensor is installed");
  
  delay(1000); // give some time to read the screen

  // Check SENtral status, make sure EEPROM upload of firmware was accomplished
  byte STAT = (_i2c_bus->readByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x01);
    if(_i2c_bus->readByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x01)  Serial.println("EEPROM detected on the sensor bus!");
    if(_i2c_bus->readByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x02)  Serial.println("EEPROM uploaded config file!");
    if(_i2c_bus->readByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x04)  Serial.println("EEPROM CRC incorrect!");
    if(_i2c_bus->readByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x08)  Serial.println("EM7180 in initialized state!");
    if(_i2c_bus->readByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x10)  Serial.println("No EEPROM detected!");
  int count = 0;
  while(!STAT) {
    _i2c_bus->writeByte(EM7180_ADDRESS, EM7180_ResetRequest, 0x01);
    delay(500);  
    count++;  
    STAT = (_i2c_bus->readByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x01);
    if(_i2c_bus->readByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x01)  Serial.println("EEPROM detected on the sensor bus!");
    if(_i2c_bus->readByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x02)  Serial.println("EEPROM uploaded config file!");
    if(_i2c_bus->readByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x04)  Serial.println("EEPROM CRC incorrect!");
    if(_i2c_bus->readByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x08)  Serial.println("EM7180 in initialized state!");
    if(_i2c_bus->readByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x10)  Serial.println("No EEPROM detected!");
    if(count > 10) break;
  }
  
   if(!(_i2c_bus->readByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x04))  Serial.println("EEPROM upload successful!");
 }

  uint8_t USFS::checkEM7180Status(){
  // Check event status register, way to check data ready by polling rather than interrupt
  uint8_t c = _i2c_bus->readByte(EM7180_ADDRESS, EM7180_EventStatus); // reading clears the register and interrupt
  return c;
  }

  uint8_t USFS::checkEM7180Errors(){
  uint8_t c = _i2c_bus->readByte(EM7180_ADDRESS, EM7180_ErrorRegister); // check error register
  return c;
  }
  
 void  USFS::initEM7180(uint8_t accBW, uint8_t gyroBW, uint16_t accFS, uint16_t gyroFS, uint16_t magFS, uint8_t QRtDiv, uint8_t magRt, uint8_t accRt, uint8_t gyroRt, uint8_t baroRt)
 {
  uint16_t EM7180_mag_fs, EM7180_acc_fs, EM7180_gyro_fs; // EM7180 sensor full scale ranges
  uint8_t param[4];      

  // Enter EM7180 initialized state
  _i2c_bus->writeByte(EM7180_ADDRESS, EM7180_HostControl, 0x00); // set SENtral in initialized state to configure registers
  _i2c_bus->writeByte(EM7180_ADDRESS, EM7180_PassThruControl, 0x00); // make sure pass through mode is off
  _i2c_bus->writeByte(EM7180_ADDRESS, EM7180_HostControl, 0x01); // Force initialize
  _i2c_bus->writeByte(EM7180_ADDRESS, EM7180_HostControl, 0x00); // set SENtral in initialized state to configure registers
  
  //Setup LPF bandwidth (BEFORE setting ODR's)
  _i2c_bus->writeByte(EM7180_ADDRESS, EM7180_ACC_LPF_BW, accBW);   // accBW = 3 = 41Hz
  _i2c_bus->writeByte(EM7180_ADDRESS, EM7180_GYRO_LPF_BW, gyroBW); // gyroBW = 3 = 41Hz
 // Set accel/gyro/mag desired ODR rates
  _i2c_bus->writeByte(EM7180_ADDRESS, EM7180_QRateDivisor, QRtDiv); // quat rate = gyroRt/(1 QRTDiv)
  _i2c_bus->writeByte(EM7180_ADDRESS, EM7180_MagRate, magRt); // 0x64 = 100 Hz
  _i2c_bus->writeByte(EM7180_ADDRESS, EM7180_AccelRate, accRt); // 200/10 Hz, 0x14 = 200 Hz
  _i2c_bus->writeByte(EM7180_ADDRESS, EM7180_GyroRate, gyroRt); // 200/10 Hz, 0x14 = 200 Hz
  _i2c_bus->writeByte(EM7180_ADDRESS, EM7180_BaroRate, 0x80 | baroRt);  // set enable bit and set Baro rate to 25 Hz, rate = baroRt/2, 0x32 = 25 Hz

  // Configure operating mode
  _i2c_bus->writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x00); // read scale sensor data
  // Enable interrupt to host upon certain events
  // choose host interrupts when any sensor updated (0x40), new gyro data (0x20), new accel data (0x10),
  // new mag data (0x08), quaternions updated (0x04), an error occurs (0x02), or the SENtral needs to be reset(0x01)
  _i2c_bus-> writeByte(EM7180_ADDRESS, EM7180_EnableEvents, 0x07);
  // Enable EM7180 run mode
  _i2c_bus->writeByte(EM7180_ADDRESS, EM7180_HostControl, 0x01); // set SENtral in normal run mode
  delay(100);

 // EM7180 parameter adjustments
  Serial.println("Beginning Parameter Adjustments");
  
  // Read sensor default FS values from parameter space
  _i2c_bus->writeByte(EM7180_ADDRESS, EM7180_ParamRequest, 0x4A); // Request to read parameter 74
  _i2c_bus->writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x80); // Request parameter transfer process
  byte param_xfer = _i2c_bus->readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
  while(!(param_xfer==0x4A)) {
    param_xfer = _i2c_bus->readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
  }
  param[0] = _i2c_bus->readByte(EM7180_ADDRESS, EM7180_SavedParamByte0);
  param[1] = _i2c_bus->readByte(EM7180_ADDRESS, EM7180_SavedParamByte1);
  param[2] = _i2c_bus->readByte(EM7180_ADDRESS, EM7180_SavedParamByte2);
  param[3] = _i2c_bus->readByte(EM7180_ADDRESS, EM7180_SavedParamByte3);
  EM7180_mag_fs = ((int16_t)(param[1]<<8) | param[0]);
  EM7180_acc_fs = ((int16_t)(param[3]<<8) | param[2]);
  Serial.print("Magnetometer Default Full Scale Range: +/-"); Serial.print(EM7180_mag_fs); Serial.println("uT");
  Serial.print("Accelerometer Default Full Scale Range: +/-"); Serial.print(EM7180_acc_fs); Serial.println("g");
  _i2c_bus->writeByte(EM7180_ADDRESS, EM7180_ParamRequest, 0x4B); // Request to read  parameter 75
  param_xfer = _i2c_bus->readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
  while(!(param_xfer==0x4B)) {
    param_xfer = _i2c_bus->readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
  }
  param[0] = _i2c_bus->readByte(EM7180_ADDRESS, EM7180_SavedParamByte0);
  param[1] = _i2c_bus->readByte(EM7180_ADDRESS, EM7180_SavedParamByte1);
  param[2] = _i2c_bus->readByte(EM7180_ADDRESS, EM7180_SavedParamByte2);
  param[3] = _i2c_bus->readByte(EM7180_ADDRESS, EM7180_SavedParamByte3);
  EM7180_gyro_fs = ((int16_t)(param[1]<<8) | param[0]);
  Serial.print("Gyroscope Default Full Scale Range: +/-"); Serial.print(EM7180_gyro_fs); Serial.println("dps");
  _i2c_bus->writeByte(EM7180_ADDRESS, EM7180_ParamRequest, 0x00); //End parameter transfer
  _i2c_bus->writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x00); // re-enable algorithm
  
  //Disable stillness mode for balancing robot application
  EM7180_set_integer_param (0x49, 0x00);
  
  //Write desired sensor full scale ranges to the EM7180
  EM7180_set_mag_acc_FS (magFS, accFS); // 1000 uT == 0x3E8, 8 g == 0x08
  EM7180_set_gyro_FS (gyroFS); // 2000 dps == 0x7D0
  
  // Read sensor new FS values from parameter space
  _i2c_bus->writeByte(EM7180_ADDRESS, EM7180_ParamRequest, 0x4A); // Request to read  parameter 74
  _i2c_bus->writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x80); // Request parameter transfer process
  param_xfer = _i2c_bus->readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
  while(!(param_xfer==0x4A)) {
    param_xfer = _i2c_bus->readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
  }
  param[0] = _i2c_bus->readByte(EM7180_ADDRESS, EM7180_SavedParamByte0);
  param[1] = _i2c_bus->readByte(EM7180_ADDRESS, EM7180_SavedParamByte1);
  param[2] = _i2c_bus->readByte(EM7180_ADDRESS, EM7180_SavedParamByte2);
  param[3] = _i2c_bus->readByte(EM7180_ADDRESS, EM7180_SavedParamByte3);
  EM7180_mag_fs = ((int16_t)(param[1]<<8) | param[0]);
  EM7180_acc_fs = ((int16_t)(param[3]<<8) | param[2]);
  Serial.print("Magnetometer New Full Scale Range: +/-"); Serial.print(EM7180_mag_fs); Serial.println("uT");
  Serial.print("Accelerometer New Full Scale Range: +/-"); Serial.print(EM7180_acc_fs); Serial.println("g");
  _i2c_bus->writeByte(EM7180_ADDRESS, EM7180_ParamRequest, 0x4B); // Request to read  parameter 75
  param_xfer = _i2c_bus->readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
  while(!(param_xfer==0x4B)) {
    param_xfer = _i2c_bus->readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
  }
  param[0] = _i2c_bus->readByte(EM7180_ADDRESS, EM7180_SavedParamByte0);
  param[1] = _i2c_bus->readByte(EM7180_ADDRESS, EM7180_SavedParamByte1);
  param[2] = _i2c_bus->readByte(EM7180_ADDRESS, EM7180_SavedParamByte2);
  param[3] = _i2c_bus->readByte(EM7180_ADDRESS, EM7180_SavedParamByte3);
  EM7180_gyro_fs = ((int16_t)(param[1]<<8) | param[0]);
  Serial.print("Gyroscope New Full Scale Range: +/-"); Serial.print(EM7180_gyro_fs); Serial.println("dps");
  _i2c_bus->writeByte(EM7180_ADDRESS, EM7180_ParamRequest, 0x00); //End parameter transfer
  _i2c_bus->writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x00); // re-enable algorithm

  
// Read EM7180 status
uint8_t runStatus = _i2c_bus->readByte(EM7180_ADDRESS, EM7180_RunStatus);
if(runStatus & 0x01) Serial.println(" EM7180 run status = normal mode");
uint8_t algoStatus = _i2c_bus->readByte(EM7180_ADDRESS, EM7180_AlgorithmStatus);
if(algoStatus & 0x01) Serial.println(" EM7180 standby status");
if(algoStatus & 0x02) Serial.println(" EM7180 algorithm slow");
if(algoStatus & 0x04) Serial.println(" EM7180 in stillness mode");
if(algoStatus & 0x08) Serial.println(" EM7180 mag calibration completed");
if(algoStatus & 0x10) Serial.println(" EM7180 magnetic anomaly detected");
if(algoStatus & 0x20) Serial.println(" EM7180 unreliable sensor data");
uint8_t passthruStatus = _i2c_bus->readByte(EM7180_ADDRESS, EM7180_PassThruStatus);
if(passthruStatus & 0x01) Serial.print(" EM7180 in passthru mode!");
uint8_t eventStatus = _i2c_bus->readByte(EM7180_ADDRESS, EM7180_EventStatus);
if(eventStatus & 0x01) Serial.println(" EM7180 CPU reset");
if(eventStatus & 0x02) Serial.println(" EM7180 Error");
if(eventStatus & 0x04) Serial.println(" EM7180 new quaternion result");
if(eventStatus & 0x08) Serial.println(" EM7180 new mag result");
if(eventStatus & 0x10) Serial.println(" EM7180 new accel result");
if(eventStatus & 0x20) Serial.println(" EM7180 new gyro result"); 
  
  delay(1000); // give some time to read the screen
  
  // Check sensor status
  uint8_t sensorStatus = _i2c_bus->readByte(EM7180_ADDRESS, EM7180_SensorStatus);
  Serial.print(" EM7180 sensor status = "); Serial.println(sensorStatus);
  if(sensorStatus & 0x01) Serial.print("Magnetometer not acknowledging!");
  if(sensorStatus & 0x02) Serial.print("Accelerometer not acknowledging!");
  if(sensorStatus & 0x04) Serial.print("Gyro not acknowledging!");
  if(sensorStatus & 0x10) Serial.print("Magnetometer ID not recognized!");
  if(sensorStatus & 0x20) Serial.print("Accelerometer ID not recognized!");
  if(sensorStatus & 0x40) Serial.print("Gyro ID not recognized!");
  
  Serial.print("Actual MagRate = "); Serial.print(_i2c_bus->readByte(EM7180_ADDRESS, EM7180_ActualMagRate)); Serial.println(" Hz"); 
  Serial.print("Actual AccelRate = "); Serial.print(10*(_i2c_bus->readByte(EM7180_ADDRESS, EM7180_ActualAccelRate))); Serial.println(" Hz"); 
  Serial.print("Actual GyroRate = "); Serial.print(10*(_i2c_bus->readByte(EM7180_ADDRESS, EM7180_ActualGyroRate))); Serial.println(" Hz"); 
  Serial.print("Actual BaroRate = "); Serial.print(_i2c_bus->readByte(EM7180_ADDRESS, EM7180_ActualBaroRate)); Serial.println(" Hz"); 
}

float USFS::uint32_reg_to_float (uint8_t *buf)
{
  union {
    uint32_t ui32;
    float f;
  } u;

  u.ui32 =     (((uint32_t)buf[0]) +
               (((uint32_t)buf[1]) <<  8) +
               (((uint32_t)buf[2]) << 16) +
               (((uint32_t)buf[3]) << 24));
  return u.f;
}

float USFS::int32_reg_to_float (uint8_t *buf)
{
  union {
    int32_t i32;
    float f;
  } u;

  u.i32 =      (((int32_t)buf[0]) +
               (((int32_t)buf[1]) <<  8) +
               (((int32_t)buf[2]) << 16) +
               (((int32_t)buf[3]) << 24));
  return u.f;
}

void USFS::float_to_bytes (float param_val, uint8_t *buf) {
  union {
    float f;
    uint8_t comp[sizeof(float)];
  } u;
  u.f = param_val;
  for (uint8_t i=0; i < sizeof(float); i++) {
    buf[i] = u.comp[i];
  }
  //Convert to LITTLE ENDIAN
  for (uint8_t i=0; i < sizeof(float); i++) {
    buf[i] = buf[(sizeof(float)-1) - i];
  }
}

void USFS::EM7180_set_gyro_FS (uint16_t gyro_fs) {
  uint8_t bytes[4], STAT;
  bytes[0] = gyro_fs & (0xFF);
  bytes[1] = (gyro_fs >> 8) & (0xFF);
  bytes[2] = 0x00;
  bytes[3] = 0x00;
  _i2c_bus->writeByte(EM7180_ADDRESS, EM7180_LoadParamByte0, bytes[0]); //Gyro LSB
  _i2c_bus->writeByte(EM7180_ADDRESS, EM7180_LoadParamByte1, bytes[1]); //Gyro MSB
  _i2c_bus->writeByte(EM7180_ADDRESS, EM7180_LoadParamByte2, bytes[2]); //Unused
  _i2c_bus->writeByte(EM7180_ADDRESS, EM7180_LoadParamByte3, bytes[3]); //Unused
  _i2c_bus->writeByte(EM7180_ADDRESS, EM7180_ParamRequest, 0xCB); //Parameter 75; 0xCB is 75 decimal with the MSB set high to indicate a paramter write processs
  _i2c_bus->writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x80); //Request parameter transfer procedure
  STAT = _i2c_bus->readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge); //Check the parameter acknowledge register and loop until the result matches parameter request byte
  while(!(STAT==0xCB)) {
  STAT = _i2c_bus->readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
  }
  _i2c_bus->writeByte(EM7180_ADDRESS, EM7180_ParamRequest, 0x00); //Parameter request = 0 to end parameter transfer process
  _i2c_bus->writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x00); // Re-start algorithm
}

void USFS::EM7180_set_mag_acc_FS (uint16_t mag_fs, uint16_t acc_fs) {
  uint8_t bytes[4], STAT;
  bytes[0] = mag_fs & (0xFF);
  bytes[1] = (mag_fs >> 8) & (0xFF);
  bytes[2] = acc_fs & (0xFF);
  bytes[3] = (acc_fs >> 8) & (0xFF);
  _i2c_bus->writeByte(EM7180_ADDRESS, EM7180_LoadParamByte0, bytes[0]); //Mag LSB
  _i2c_bus->writeByte(EM7180_ADDRESS, EM7180_LoadParamByte1, bytes[1]); //Mag MSB
  _i2c_bus->writeByte(EM7180_ADDRESS, EM7180_LoadParamByte2, bytes[2]); //Acc LSB
  _i2c_bus->writeByte(EM7180_ADDRESS, EM7180_LoadParamByte3, bytes[3]); //Acc MSB
  _i2c_bus->writeByte(EM7180_ADDRESS, EM7180_ParamRequest, 0xCA); //Parameter 74; 0xCA is 74 decimal with the MSB set high to indicate a paramter write processs
  _i2c_bus->writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x80); //Request parameter transfer procedure
  STAT = _i2c_bus->readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge); //Check the parameter acknowledge register and loop until the result matches parameter request byte
  while(!(STAT==0xCA)) {
    STAT = _i2c_bus->readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
  }
  _i2c_bus->writeByte(EM7180_ADDRESS, EM7180_ParamRequest, 0x00); //Parameter request = 0 to end parameter transfer process
  _i2c_bus->writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x00); // Re-start algorithm
}

void USFS::EM7180_set_integer_param (uint8_t param, uint32_t param_val) {
  uint8_t bytes[4], STAT;
  bytes[0] = param_val & (0xFF);
  bytes[1] = (param_val >> 8) & (0xFF);
  bytes[2] = (param_val >> 16) & (0xFF);
  bytes[3] = (param_val >> 24) & (0xFF);
  param = param | 0x80; //Parameter is the decimal value with the MSB set high to indicate a paramter write processs
  _i2c_bus->writeByte(EM7180_ADDRESS, EM7180_LoadParamByte0, bytes[0]); //Param LSB
  _i2c_bus->writeByte(EM7180_ADDRESS, EM7180_LoadParamByte1, bytes[1]);
  _i2c_bus->writeByte(EM7180_ADDRESS, EM7180_LoadParamByte2, bytes[2]);
  _i2c_bus->writeByte(EM7180_ADDRESS, EM7180_LoadParamByte3, bytes[3]); //Param MSB
  _i2c_bus->writeByte(EM7180_ADDRESS, EM7180_ParamRequest, param);
  _i2c_bus->writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x80); //Request parameter transfer procedure
  STAT = _i2c_bus->readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge); //Check the parameter acknowledge register and loop until the result matches parameter request byte
  while(!(STAT==param)) {
    STAT = _i2c_bus->readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
  }
  _i2c_bus->writeByte(EM7180_ADDRESS, EM7180_ParamRequest, 0x00); //Parameter request = 0 to end parameter transfer process
  _i2c_bus->writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x00); // Re-start algorithm
}

void USFS::EM7180_set_float_param (uint8_t param, float param_val) {
  uint8_t bytes[4], STAT;
  float_to_bytes (param_val, &bytes[0]);
  param = param | 0x80; //Parameter is the decimal value with the MSB set high to indicate a paramter write processs
  _i2c_bus->writeByte(EM7180_ADDRESS, EM7180_LoadParamByte0, bytes[0]); //Param LSB
  _i2c_bus->writeByte(EM7180_ADDRESS, EM7180_LoadParamByte1, bytes[1]);
  _i2c_bus->writeByte(EM7180_ADDRESS, EM7180_LoadParamByte2, bytes[2]);
  _i2c_bus->writeByte(EM7180_ADDRESS, EM7180_LoadParamByte3, bytes[3]); //Param MSB
  _i2c_bus->writeByte(EM7180_ADDRESS, EM7180_ParamRequest, param);
  _i2c_bus->writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x80); //Request parameter transfer procedure
  STAT = _i2c_bus->readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge); //Check the parameter acknowledge register and loop until the result matches parameter request byte
  while(!(STAT==param)) {
    STAT = _i2c_bus->readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
  }
  _i2c_bus->writeByte(EM7180_ADDRESS, EM7180_ParamRequest, 0x00); //Parameter request = 0 to end parameter transfer process
  _i2c_bus->writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x00); // Re-start algorithm
}


void USFS::readSENtralQuatData(float * destination)
{
  uint8_t rawData[16];  // x/y/z quaternion register data stored here
  _i2c_bus->readBytes(EM7180_ADDRESS, EM7180_QX, 16, &rawData[0]); // Read the sixteen raw data registers into data array
  destination[1] = uint32_reg_to_float (&rawData[0]);
  destination[2] = uint32_reg_to_float (&rawData[4]);
  destination[3] = uint32_reg_to_float (&rawData[8]);
  destination[0] = uint32_reg_to_float (&rawData[12]);   // SENtral stores quats as qx, qy, qz, q0!

}

void USFS::readSENtralAccelData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z accel register data stored here
  _i2c_bus->readBytes(EM7180_ADDRESS, EM7180_AX, 6, &rawData[0]);       // Read the six raw data registers into data array
  destination[0] = (int16_t) (((int16_t)rawData[1] << 8) | rawData[0]);  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = (int16_t) (((int16_t)rawData[3] << 8) | rawData[2]);  
  destination[2] = (int16_t) (((int16_t)rawData[5] << 8) | rawData[4]); 
}

void USFS::readSENtralGyroData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z gyro register data stored here
  _i2c_bus->readBytes(EM7180_ADDRESS, EM7180_GX, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
  destination[0] = (int16_t) (((int16_t)rawData[1] << 8) | rawData[0]);   // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = (int16_t) (((int16_t)rawData[3] << 8) | rawData[2]);  
  destination[2] = (int16_t) (((int16_t)rawData[5] << 8) | rawData[4]); 
}

void USFS::readSENtralMagData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z gyro register data stored here
  _i2c_bus->readBytes(EM7180_ADDRESS, EM7180_MX, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
  destination[0] = (int16_t) (((int16_t)rawData[1] << 8) | rawData[0]);   // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = (int16_t) (((int16_t)rawData[3] << 8) | rawData[2]);  
  destination[2] = (int16_t) (((int16_t)rawData[5] << 8) | rawData[4]); 
}

int16_t USFS::readSENtralBaroData()
{
  uint8_t rawData[2];  // x/y/z gyro register data stored here
  _i2c_bus->readBytes(EM7180_ADDRESS, EM7180_Baro, 2, &rawData[0]);  // Read the two raw data registers sequentially into data array
  return  (int16_t) (((int16_t)rawData[1] << 8) | rawData[0]);   // Turn the MSB and LSB into a signed 16-bit value
}

int16_t USFS::readSENtralTempData()
{
  uint8_t rawData[2];  // x/y/z gyro register data stored here
  _i2c_bus->readBytes(EM7180_ADDRESS, EM7180_Temp, 2, &rawData[0]);  // Read the two raw data registers sequentially into data array
  return  (int16_t) (((int16_t)rawData[1] << 8) | rawData[0]);   // Turn the MSB and LSB into a signed 16-bit value
}


void USFS::SENtralPassThroughMode()
{
  // First put SENtral in standby mode
  uint8_t c = _i2c_bus->readByte(EM7180_ADDRESS, EM7180_AlgorithmControl);
  _i2c_bus->writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, c | 0x01);
//  c = readByte(EM7180_ADDRESS, EM7180_AlgorithmStatus);
//  Serial.print("c = "); Serial.println(c);
// Verify standby status
// if(readByte(EM7180_ADDRESS, EM7180_AlgorithmStatus) & 0x01) {
   Serial.println("SENtral in standby mode"); 
  // Place SENtral in pass-through mode
  _i2c_bus->writeByte(EM7180_ADDRESS, EM7180_PassThruControl, 0x01); 
  if(_i2c_bus->readByte(EM7180_ADDRESS, EM7180_PassThruStatus) & 0x01) {
    Serial.println("SENtral in pass-through mode");
  }
  else {
    Serial.println("ERROR! SENtral not in pass-through mode!");
  }
 }
 
 

// I2C communication with the M24512DFM EEPROM is a little different from I2C communication with the usual motion sensor
// since the address is defined by two bytes

       void USFS::M24512DFMwriteByte(uint8_t device_address, uint8_t data_address1, uint8_t data_address2, uint8_t  data)
       {
       Wire.beginTransmission(device_address);   // Initialize the Tx buffer
       Wire.write(data_address1);                // Put slave register address in Tx buffer
       Wire.write(data_address2);                // Put slave register address in Tx buffer
       Wire.write(data);                         // Put data in Tx buffer
       Wire.endTransmission();                   // Send the Tx buffer
       }


       void USFS::M24512DFMwriteBytes(uint8_t device_address, uint8_t data_address1, uint8_t data_address2, uint8_t count, uint8_t * dest)
       {
       if(count > 128) 
       {
        count = 128;
        Serial.print("Page count cannot be more than 128 bytes!");
        }       
        Wire.beginTransmission(device_address);   // Initialize the Tx buffer
        Wire.write(data_address1);                // Put slave register address in Tx buffer
        Wire.write(data_address2);                // Put slave register address in Tx buffer
        for(uint8_t i=0; i < count; i++) {
           Wire.write(dest[i]);                      // Put data in Tx buffer
           }
        Wire.endTransmission();                   // Send the Tx buffer
        }


        uint8_t USFS::M24512DFMreadByte(uint8_t device_address, uint8_t data_address1, uint8_t data_address2)
        {
        uint8_t data; // `data` will store the register data   
        Wire.beginTransmission(device_address);         // Initialize the Tx buffer
        Wire.write(data_address1);                // Put slave register address in Tx buffer
        Wire.write(data_address2);                // Put slave register address in Tx buffer
        Wire.endTransmission(false);             // Send the Tx buffer, but send a restart to keep connection alive
        Wire.requestFrom(device_address, 1);  // Read one byte from slave register address 
        data = Wire.read();                      // Fill Rx buffer with result
        return data;                             // Return data read from slave register
        }

        void USFS::M24512DFMreadBytes(uint8_t device_address, uint8_t data_address1, uint8_t data_address2, uint8_t count, uint8_t * dest)
        {  
        uint8_t temp[2] = {data_address1, data_address2};
        Wire.beginTransmission(device_address);   // Initialize the Tx buffer
        Wire.write(data_address1);                     // Put slave register address in Tx buffer
        Wire.write(data_address2);                     // Put slave register address in Tx buffer
        Wire.endTransmission(false);              // Send the Tx buffer, but send a restart to keep connection alive
        uint8_t i = 0;
        Wire.requestFrom(device_address, count);       // Read bytes from slave register address 
        while (Wire.available()) {
        dest[i++] = Wire.read(); }                // Put read results in the Rx buffer
        }






  // Implementation of Sebastian Madgwick's "...efficient orientation filter for... inertial/magnetic sensor arrays"
// (see http://www.x-io.co.uk/category/open-source/ for examples and more details)
// which fuses acceleration, rotation rate, and magnetic moments to produce a quaternion-based estimate of absolute
// device orientation -- which can be converted to yaw, pitch, and roll. Useful for stabilizing quadcopters, etc.
// The performance of the orientation filter is at least as good as conventional Kalman-based filtering algorithms
// but is much less computationally intensive---it can be performed on a 3.3 V Pro Mini operating at 8 MHz!
void USFS::MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
        {
            float q1 = _q[0], q2 = _q[1], q3 = _q[2], q4 = _q[3];   // short name local variable for readability
            float norm;
            float hx, hy, _2bx, _2bz;
            float s1, s2, s3, s4;
            float qDot1, qDot2, qDot3, qDot4;

            // Auxiliary variables to avoid repeated arithmetic
            float _2q1mx;
            float _2q1my;
            float _2q1mz;
            float _2q2mx;
            float _4bx;
            float _4bz;
            float _2q1 = 2.0f * q1;
            float _2q2 = 2.0f * q2;
            float _2q3 = 2.0f * q3;
            float _2q4 = 2.0f * q4;
            float _2q1q3 = 2.0f * q1 * q3;
            float _2q3q4 = 2.0f * q3 * q4;
            float q1q1 = q1 * q1;
            float q1q2 = q1 * q2;
            float q1q3 = q1 * q3;
            float q1q4 = q1 * q4;
            float q2q2 = q2 * q2;
            float q2q3 = q2 * q3;
            float q2q4 = q2 * q4;
            float q3q3 = q3 * q3;
            float q3q4 = q3 * q4;
            float q4q4 = q4 * q4;

            // Normalise accelerometer measurement
            norm = sqrt(ax * ax + ay * ay + az * az);
            if (norm == 0.0f) return; // handle NaN
            norm = 1.0f/norm;
            ax *= norm;
            ay *= norm;
            az *= norm;

            // Normalise magnetometer measurement
            norm = sqrt(mx * mx + my * my + mz * mz);
            if (norm == 0.0f) return; // handle NaN
            norm = 1.0f/norm;
            mx *= norm;
            my *= norm;
            mz *= norm;

            // Reference direction of Earth's magnetic field
            _2q1mx = 2.0f * q1 * mx;
            _2q1my = 2.0f * q1 * my;
            _2q1mz = 2.0f * q1 * mz;
            _2q2mx = 2.0f * q2 * mx;
            hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
            hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
            _2bx = sqrt(hx * hx + hy * hy);
            _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
            _4bx = 2.0f * _2bx;
            _4bz = 2.0f * _2bz;

            // Gradient decent algorithm corrective step
            s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax) + _2q2 * (2.0f * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
            s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + _2q1 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
            s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax) + _2q4 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
            s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
            norm = sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
            norm = 1.0f/norm;
            s1 *= norm;
            s2 *= norm;
            s3 *= norm;
            s4 *= norm;

            // Compute rate of change of quaternion
            qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - _beta * s1;
            qDot2 = 0.5f * ( q1 * gx + q3 * gz - q4 * gy) - _beta * s2;
            qDot3 = 0.5f * ( q1 * gy - q2 * gz + q4 * gx) - _beta * s3;
            qDot4 = 0.5f * ( q1 * gz + q2 * gy - q3 * gx) - _beta * s4;

            // Integrate to yield quaternion
            q1 += qDot1 * _deltat;
            q2 += qDot2 * _deltat;
            q3 += qDot3 * _deltat;
            q4 += qDot4 * _deltat;
            norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
            norm = 1.0f/norm;
            _q[0] = q1 * norm;
            _q[1] = q2 * norm;
            _q[2] = q3 * norm;
            _q[3] = q4 * norm;

        }
  
  
  
 // Similar to Madgwick scheme but uses proportional and integral filtering on the error between estimated reference vectors and
 // measured ones. 
            void USFS::MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
        {
            float q1 = _q[0], q2 = _q[1], q3 = _q[2], q4 = _q[3];   // short name local variable for readability
            float eInt[3] = {0.0f, 0.0f, 0.0f};       // vector to hold integral error for Mahony method
            float norm;
            float hx, hy, bx, bz;
            float vx, vy, vz, wx, wy, wz;
            float ex, ey, ez;
            float pa, pb, pc;

            // Auxiliary variables to avoid repeated arithmetic
            float q1q1 = q1 * q1;
            float q1q2 = q1 * q2;
            float q1q3 = q1 * q3;
            float q1q4 = q1 * q4;
            float q2q2 = q2 * q2;
            float q2q3 = q2 * q3;
            float q2q4 = q2 * q4;
            float q3q3 = q3 * q3;
            float q3q4 = q3 * q4;
            float q4q4 = q4 * q4;   

            // Normalise accelerometer measurement
            norm = sqrt(ax * ax + ay * ay + az * az);
            if (norm == 0.0f) return; // handle NaN
            norm = 1.0f / norm;        // use reciprocal for division
            ax *= norm;
            ay *= norm;
            az *= norm;

            // Normalise magnetometer measurement
            norm = sqrt(mx * mx + my * my + mz * mz);
            if (norm == 0.0f) return; // handle NaN
            norm = 1.0f / norm;        // use reciprocal for division
            mx *= norm;
            my *= norm;
            mz *= norm;

            // Reference direction of Earth's magnetic field
            hx = 2.0f * mx * (0.5f - q3q3 - q4q4) + 2.0f * my * (q2q3 - q1q4) + 2.0f * mz * (q2q4 + q1q3);
            hy = 2.0f * mx * (q2q3 + q1q4) + 2.0f * my * (0.5f - q2q2 - q4q4) + 2.0f * mz * (q3q4 - q1q2);
            bx = sqrt((hx * hx) + (hy * hy));
            bz = 2.0f * mx * (q2q4 - q1q3) + 2.0f * my * (q3q4 + q1q2) + 2.0f * mz * (0.5f - q2q2 - q3q3);

            // Estimated direction of gravity and magnetic field
            vx = 2.0f * (q2q4 - q1q3);
            vy = 2.0f * (q1q2 + q3q4);
            vz = q1q1 - q2q2 - q3q3 + q4q4;
            wx = 2.0f * bx * (0.5f - q3q3 - q4q4) + 2.0f * bz * (q2q4 - q1q3);
            wy = 2.0f * bx * (q2q3 - q1q4) + 2.0f * bz * (q1q2 + q3q4);
            wz = 2.0f * bx * (q1q3 + q2q4) + 2.0f * bz * (0.5f - q2q2 - q3q3);  

            // Error is cross product between estimated direction and measured direction of gravity
            ex = (ay * vz - az * vy) + (my * wz - mz * wy);
            ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
            ez = (ax * vy - ay * vx) + (mx * wy - my * wx);
            if (_Ki > 0.0f)
            {
                eInt[0] += ex;      // accumulate integral error
                eInt[1] += ey;
                eInt[2] += ez;
            }
            else
            {
                eInt[0] = 0.0f;     // prevent integral wind up
                eInt[1] = 0.0f;
                eInt[2] = 0.0f;
            }

            // Apply feedback terms
            gx = gx + _Kp * ex + _Ki * eInt[0];
            gy = gy + _Kp * ey + _Ki * eInt[1];
            gz = gz + _Kp * ez + _Ki * eInt[2];

            // Integrate rate of change of quaternion
            pa = q2;
            pb = q3;
            pc = q4;
            q1 = q1 + (-q2 * gx - q3 * gy - q4 * gz) * (0.5f * _deltat);
            q2 = pa + (q1 * gx + pb * gz - pc * gy) *  (0.5f * _deltat);
            q3 = pb + (q1 * gy - pa * gz + pc * gx) *  (0.5f * _deltat);
            q4 = pc + (q1 * gz + pa * gy - pb * gx) *  (0.5f * _deltat);

            // Normalise quaternion
            norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
            norm = 1.0f / norm;
            _q[0] = q1 * norm;
            _q[1] = q2 * norm;
            _q[2] = q3 * norm;
            _q[3] = q4 * norm;
 
        }
