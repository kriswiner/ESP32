#ifndef USFS_h
#define USFSh_h

#include "Arduino.h"
#include "Wire.h"
#include "I2Cdev.h"   

// EM7180 SENtral register map
// see http://www.emdeveloper.com/downloads/7180/EMSentral_EM7180_Register_Map_v1_3.pdf
//
#define EM7180_QX                 0x00  // this is a 32-bit normalized floating point number read from registers 0x00-03
#define EM7180_QY                 0x04  // this is a 32-bit normalized floating point number read from registers 0x04-07
#define EM7180_QZ                 0x08  // this is a 32-bit normalized floating point number read from registers 0x08-0B
#define EM7180_QW                 0x0C  // this is a 32-bit normalized floating point number read from registers 0x0C-0F
#define EM7180_QTIME              0x10  // this is a 16-bit unsigned integer read from registers 0x10-11
#define EM7180_MX                 0x12  // int16_t from registers 0x12-13
#define EM7180_MY                 0x14  // int16_t from registers 0x14-15
#define EM7180_MZ                 0x16  // int16_t from registers 0x16-17
#define EM7180_MTIME              0x18  // uint16_t from registers 0x18-19
#define EM7180_AX                 0x1A  // int16_t from registers 0x1A-1B
#define EM7180_AY                 0x1C  // int16_t from registers 0x1C-1D
#define EM7180_AZ                 0x1E  // int16_t from registers 0x1E-1F
#define EM7180_ATIME              0x20  // uint16_t from registers 0x20-21
#define EM7180_GX                 0x22  // int16_t from registers 0x22-23
#define EM7180_GY                 0x24  // int16_t from registers 0x24-25
#define EM7180_GZ                 0x26  // int16_t from registers 0x26-27
#define EM7180_GTIME              0x28  // uint16_t from registers 0x28-29
#define EM7180_Baro               0x2A  // start of two-byte MS5637 pressure data, 16-bit signed interger
#define EM7180_BaroTIME           0x2C  // start of two-byte MS5637 pressure timestamp, 16-bit unsigned
#define EM7180_Temp               0x2E  // start of two-byte MS5637 temperature data, 16-bit signed interger
#define EM7180_TempTIME           0x30  // start of two-byte MS5637 temperature timestamp, 16-bit unsigned
#define EM7180_QRateDivisor       0x32  // uint8_t 
#define EM7180_EnableEvents       0x33
#define EM7180_HostControl        0x34
#define EM7180_EventStatus        0x35
#define EM7180_SensorStatus       0x36
#define EM7180_SentralStatus      0x37
#define EM7180_AlgorithmStatus    0x38
#define EM7180_FeatureFlags       0x39
#define EM7180_ParamAcknowledge   0x3A
#define EM7180_SavedParamByte0    0x3B
#define EM7180_SavedParamByte1    0x3C
#define EM7180_SavedParamByte2    0x3D
#define EM7180_SavedParamByte3    0x3E
#define EM7180_ActualMagRate      0x45
#define EM7180_ActualAccelRate    0x46
#define EM7180_ActualGyroRate     0x47
#define EM7180_ActualBaroRate     0x48
#define EM7180_ActualTempRate     0x49
#define EM7180_ErrorRegister      0x50
#define EM7180_AlgorithmControl   0x54
#define EM7180_MagRate            0x55
#define EM7180_AccelRate          0x56
#define EM7180_GyroRate           0x57
#define EM7180_BaroRate           0x58
#define EM7180_TempRate           0x59
#define EM7180_LoadParamByte0     0x60
#define EM7180_LoadParamByte1     0x61
#define EM7180_LoadParamByte2     0x62
#define EM7180_LoadParamByte3     0x63
#define EM7180_ParamRequest       0x64
#define EM7180_ROMVersion1        0x70
#define EM7180_ROMVersion2        0x71
#define EM7180_RAMVersion1        0x72
#define EM7180_RAMVersion2        0x73
#define EM7180_ProductID          0x90
#define EM7180_RevisionID         0x91
#define EM7180_RunStatus          0x92
#define EM7180_UploadAddress      0x94 // uint16_t registers 0x94 (MSB)-5(LSB)
#define EM7180_UploadData         0x96  
#define EM7180_CRCHost            0x97  // uint32_t from registers 0x97-9A
#define EM7180_ResetRequest       0x9B   
#define EM7180_PassThruStatus     0x9E   
#define EM7180_PassThruControl    0xA0
#define EM7180_ACC_LPF_BW         0x5B  //Register GP36
#define EM7180_GYRO_LPF_BW        0x5C  //Register GP37
#define EM7180_BARO_LPF_BW        0x5D  //Register GP38

#define EM7180_ADDRESS           0x28   // Address of the EM7180 SENtral sensor hub
#define M24512DFM_DATA_ADDRESS   0x50   // Address of the 500 page M24512DFM EEPROM data buffer, 1024 bits (128 8-bit bytes) per page
#define M24512DFM_IDPAGE_ADDRESS 0x58   // Address of the single M24512DFM lockable EEPROM ID page
#define MPU9250_ADDRESS          0x68   // Device address when ADO = 0
#define AK8963_ADDRESS           0x0C   //  Address of magnetometer
#define MS5637_ADDRESS           0x76   // Address of altimeter

class USFS
{
  public: 
  USFS(uint8_t intPin, bool passThru, I2Cdev* i2c_bus);
  float uint32_reg_to_float (uint8_t *buf);
  float int32_reg_to_float (uint8_t *buf);
  void float_to_bytes (float param_val, uint8_t *buf);
  void EM7180_set_gyro_FS (uint16_t gyro_fs);
  void EM7180_set_mag_acc_FS (uint16_t mag_fs, uint16_t acc_fs);
  void EM7180_set_integer_param (uint8_t param, uint32_t param_val);
  void EM7180_set_float_param (uint8_t param, float param_val);
  void readSENtralQuatData(float * destination);
  void readSENtralAccelData(int16_t * destination);
  void readSENtralGyroData(int16_t * destination);
  void readSENtralMagData(int16_t * destination);
  void initEM7180(uint8_t accBW, uint8_t gyroBW, uint16_t accFS, uint16_t gyroFS, uint16_t magFS, uint8_t QRtDiv, uint8_t magRt, uint8_t accRt, uint8_t gyroRt, uint8_t baroRt);
  int16_t readSENtralBaroData();
  int16_t readSENtralTempData();
  void SENtralPassThroughMode();
  void M24512DFMwriteByte(uint8_t device_address, uint8_t data_address1, uint8_t data_address2, uint8_t  data);
  void M24512DFMwriteBytes(uint8_t device_address, uint8_t data_address1, uint8_t data_address2, uint8_t count, uint8_t * dest);
  uint8_t M24512DFMreadByte(uint8_t device_address, uint8_t data_address1, uint8_t data_address2);
  void M24512DFMreadBytes(uint8_t device_address, uint8_t data_address1, uint8_t data_address2, uint8_t count, uint8_t * dest);
  void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);
  void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);
  void getChipID();
  void loadfwfromEEPROM();
  uint8_t checkEM7180Status();
  uint8_t checkEM7180Errors();
  private:
  uint8_t _intPin;
  bool _passThru;
  float _q[4];
  float _beta;
  float _deltat;
  float _Kp;
  float _Ki;
  I2Cdev* _i2c_bus;
};

#endif
