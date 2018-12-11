#include "LSM6DSM.h"
#include "LIS2MDL.h"
#include "LPS22HB.h"
#include "USFS.h"

#define I2C_BUS          Wire                           // Define the I2C bus (Wire instance) you wish to use

I2Cdev                   i2c_0(&I2C_BUS);               // Instantiate the I2Cdev object and point to the desired I2C bus

bool SerialDebug = true;  // set to true to get Serial output for debugging
bool passThru  = false;

#define myLed 5
#define pinGND 12
#define pin3V3 13

void EM7180intHandler();
void myinthandler1();
void myinthandler2();
void myinthandler3();

// global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
float pi = 3.141592653589793238462643383279502884f;
float GyroMeasError = pi * (40.0f / 180.0f);   // gyroscope measurement error in rads/s (start at 40 deg/s)
float GyroMeasDrift = pi * (0.0f  / 180.0f);   // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
float beta = sqrtf(3.0f / 4.0f) * GyroMeasError;   // compute beta
float zeta = sqrtf(3.0f / 4.0f) * GyroMeasDrift;   // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
uint32_t delt_t = 0;                      // used to control display output rate
uint32_t sumCount = 0;                    // used to control display output rate
float pitch, yaw, roll, Yaw, Pitch, Roll;
float a12, a22, a31, a32, a33;            // rotation matrix coefficients for Euler angles and gravity components
float A112, A22, A31, A32, A33;            // rotation matrix coefficients for Hardware Euler angles and gravity components
float deltat = 0.0f, sum = 0.0f;          // integration interval for both filter schemes
uint32_t lastUpdate = 0, firstUpdate = 0; // used to calculate integration interval
uint32_t Now = 0;                         // used to calculate integration interval
float lin_ax, lin_ay, lin_az;             // linear acceleration (acceleration with gravity component subtracted)
float lin_Ax, lin_Ay, lin_Az;             // Hardware linear acceleration (acceleration with gravity component subtracted)
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // vector to hold quaternion
float Q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // hardware quaternion data register
float eInt[3] = {0.0f, 0.0f, 0.0f};       // vector to hold integral error for Mahony method

//LSM6DSM definitions
#define LSM6DSM_intPin1 10  // interrupt1 pin definitions, significant motion
#define LSM6DSM_intPin2 9   // interrupt2 pin definitions, data ready

/* Specify sensor parameters (sample rate is twice the bandwidth)
 * choices are:
      AFS_2G, AFS_4G, AFS_8G, AFS_16G  
      GFS_245DPS, GFS_500DPS, GFS_1000DPS, GFS_2000DPS 
      AODR_12_5Hz, AODR_26Hz, AODR_52Hz, AODR_104Hz, AODR_208Hz, AODR_416Hz, AODR_833Hz, AODR_1660Hz, AODR_3330Hz, AODR_6660Hz
      GODR_12_5Hz, GODR_26Hz, GODR_52Hz, GODR_104Hz, GODR_208Hz, GODR_416Hz, GODR_833Hz, GODR_1660Hz, GODR_3330Hz, GODR_6660Hz
*/ 
uint8_t Ascale = AFS_2G, Gscale = GFS_245DPS, AODR = AODR_208Hz, GODR = GODR_416Hz;

float aRes, gRes;              // scale resolutions per LSB for the accel and gyro sensor2
float accelBias[3] = {-0.00499, 0.01540, 0.02902}, gyroBias[3] = {-0.50, 0.14, 0.28}; // offset biases for the accel and gyro
int16_t LSM6DSMData[7];        // Stores the 16-bit signed sensor output
float   Gtemperature;           // Stores the real internal gyro temperature in degrees Celsius
float ax, ay, az, gx, gy, gz;  // variables to hold latest accel/gyro data values 

bool newLSM6DSMData = false;
bool newLSM6DSMTap  = false;

LSM6DSM LSM6DSM(LSM6DSM_intPin1, LSM6DSM_intPin2, &i2c_0); // instantiate LSM6DSM class


//LIS2MDL definitions
#define LIS2MDL_intPin  8 // interrupt for magnetometer data ready

/* Specify sensor parameters (sample rate is twice the bandwidth)
 * choices are: MODR_10Hz, MOIDR_20Hz, MODR_50 Hz and MODR_100Hz
*/ 
uint8_t MODR = MODR_100Hz;

float mRes = 0.0015f;            // mag sensitivity
float magBias[3] = {0,0,0}, magScale[3]  = {0,0,0}; // Bias corrections for magnetometer
int16_t LIS2MDLData[4];          // Stores the 16-bit signed sensor output
float Mtemperature;              // Stores the real internal chip temperature in degrees Celsius
float mx, my, mz;                // variables to hold latest mag data values 
uint8_t LIS2MDLstatus;

bool newLIS2MDLData = false;

LIS2MDL LIS2MDL(LIS2MDL_intPin, &i2c_0); // instantiate LIS2MDL class


// LPS22H definitions
uint8_t LPS22H_intPin = 5;

/* Specify sensor parameters (sample rate is twice the bandwidth) 
   Choices are P_1Hz, P_10Hz P_25 Hz, P_50Hz, and P_75Hz
 */
uint8_t PODR = P_25Hz;     // set pressure amd temperature output data rate
uint8_t LPS22Hstatus;
float temperature, pressure, altitude;

bool newLPS22HData = false;

LPS22H LPS22H(LPS22H_intPin, &i2c_0);


const uint8_t USFS_intPin = 27;
bool newEM7180Data = false;
int16_t accelCount[3];  // Stores the 16-bit signed accelerometer sensor output
int16_t gyroCount[3];   // Stores the 16-bit signed gyro sensor output
int16_t magCount[3];    // Stores the 16-bit signed magnetometer sensor output
int16_t tempCount, rawPressure, rawTemperature;            // temperature raw count output
float   Temperature, Pressure, Altitude; //  temperature in degrees Celsius, pressure in mbar
float Ax, Ay, Az, Gx, Gy, Gz, Mx, My, Mz; // variables to hold latest sensor data values


/* Choose EM7180, LSM6DSM, LIS2MDL sample rates and bandwidths
   Choices are:
   accBW, gyroBW 0x00 = 250 Hz, 0x01 = 184 Hz, 0x02 = 92 Hz, 0x03 = 41 Hz, 0x04 = 20 Hz, 0x05 = 10 Hz, 0x06 = 5 Hz, 0x07 = no filter (3600 Hz)
   QRtDiv 0x00, 0x01, 0x02, etc quat rate = gyroRt/(1 + QRtDiv)
   magRt 8 Hz = 0x08 or 100 Hz 0x64
   accRt, gyroRt 1000, 500, 250, 200, 125, 100, 50 Hz enter by choosing desired rate
   and dividing by 10, so 200 Hz would be 200/10 = 20 = 0x14
   sample rate of barometer is baroRt/2 so for 25 Hz enter 50 = 0x32
   LSM6DSM accel/gyro rates 0f 833 Hz set Rt variables to 0x53
*/
uint8_t accBW = 0x03, gyroBW = 0x03, QRtDiv = 0x03, magRt = 0x64, accRt = 0x53, gyroRt = 0x53, baroRt = 0x32;
/*
   Choose sensor full ranges
   Choices are 2, 4, 8, 16 g for accFS, 250, 500, 1000, and 2000 dps for gyro FS and 1000 uT for magFS expressed as HEX values
*/
uint16_t accFS = 0x02, gyroFS = 0x7D0, magFS = 0x3E8;

USFS USFS(USFS_intPin, passThru, &i2c_0);


void setup() {  
  Serial.begin(115200);
  delay(4000);

  // Configure led
  pinMode(myLed, OUTPUT);
  digitalWrite(myLed, HIGH); // start with led on

   pinMode(pinGND, OUTPUT);
  digitalWrite(pinGND, LOW);

  pinMode(pin3V3, OUTPUT);
  digitalWrite(pin3V3, HIGH);

  pinMode(USFS_intPin, INPUT);
  
  Wire.begin(16, 15, 400000); //(SDA, SCL) (21,22) are default on ESP32, 400 kHz I2C clock
  delay(1000);
 
  i2c_0.I2Cscan(); // which I2C device are on the bus?

  if(!passThru)
  {
  // Initialize the USFS
  USFS.getChipID();        // check ROM/RAM version of EM7180
  USFS.loadfwfromEEPROM(); // load EM7180 firmware from EEPROM
  USFS.initEM7180(accBW, gyroBW, accFS, gyroFS, magFS, QRtDiv, magRt, accRt, gyroRt, baroRt); // set MPU and MS5637 sensor parameters
  } // end of "if(!passThru)" handling

  if(passThru)
  {
  // Read the LSM6DSM Chip ID register, this is a good test of communication
  Serial.println("LSM6DSM accel/gyro...");
  byte c = LSM6DSM.getChipID();  // Read CHIP_ID register for LSM6DSM
  Serial.print("LSM6DSM "); Serial.print("I AM "); Serial.print(c, HEX); Serial.print(" I should be "); Serial.println(0x6A, HEX);
  Serial.println(" ");
  delay(1000); 

  // Read the LIS2MDL Chip ID register, this is a good test of communication
  Serial.println("LIS2MDL mag...");
  byte d = LIS2MDL.getChipID();  // Read CHIP_ID register for LSM6DSM
  Serial.print("LIS2MDL "); Serial.print("I AM "); Serial.print(d, HEX); Serial.print(" I should be "); Serial.println(0x40, HEX);
  Serial.println(" ");
  delay(1000); 

  Serial.println("LPS22HB barometer...");
  uint8_t e = LPS22H.getChipID();
  Serial.print("LPS25H "); Serial.print("I AM "); Serial.print(e, HEX); Serial.print(" I should be "); Serial.println(0xB1, HEX);
  delay(1000); 
  

  if(c == 0x6A && d == 0x40 && e == 0xB1) // check if all I2C sensors have acknowledged
  {
   Serial.println("LSM6DSM and LIS2MDL and LPS22HB are online..."); Serial.println(" ");
   
   digitalWrite(myLed, LOW);

   LSM6DSM.reset();  // software reset LSM6DSM to default registers

   // get sensor resolutions, only need to do this once
   aRes = LSM6DSM.getAres(Ascale);
   gRes = LSM6DSM.getGres(Gscale);

   LSM6DSM.init(Ascale, Gscale, AODR, GODR);

   LSM6DSM.selfTest();

   LSM6DSM.offsetBias(gyroBias, accelBias);
   Serial.println("accel biases (mg)"); Serial.println(1000.0f * accelBias[0]); Serial.println(1000.0f * accelBias[1]); Serial.println(1000.0f * accelBias[2]);
   Serial.println("gyro biases (dps)"); Serial.println(gyroBias[0]); Serial.println(gyroBias[1]); Serial.println(gyroBias[2]);
   delay(1000); 

   LIS2MDL.reset(); // software reset LIS2MDL to default registers

   mRes = 0.0015f;  // fixed sensitivity and full scale (+/- 49.152 Gauss); 
   
   LIS2MDL.init(MODR);

   LIS2MDL.selfTest();

   LIS2MDL.offsetBias(magBias, magScale);
   Serial.println("mag biases (mG)"); Serial.println(1000.0f * magBias[0]); Serial.println(1000.0f * magBias[1]); Serial.println(1000.0f * magBias[2]); 
   Serial.println("mag scale (mG)"); Serial.println(magScale[0]); Serial.println(magScale[1]); Serial.println(magScale[2]); 
   delay(2000); // add delay to see results before serial spew of data

   LPS22H.Init(PODR);  // Initialize LPS22H altimeter
   delay(1000);

   digitalWrite(myLed, HIGH);
   
  }
  else 
  {
  if(c != 0x6A) Serial.println(" LSM6DSM not functioning!");
  if(d != 0x40) Serial.println(" LIS2MDL not functioning!");    
  if(e != 0xB1) Serial.println(" LPS22HB not functioning!");   

  while(1){};
  }
  }  // end of "if(passThru)" handling


  if(!passThru)
  {
    attachInterrupt(USFS_intPin, EM7180intHandler, RISING);  // define interrupt for INT pin output of EM7180
 
    USFS.checkEM7180Status();
  }

  if(passThru)
  {
  attachInterrupt(LSM6DSM_intPin2, myinthandler1, RISING);  // define interrupt for intPin2 output of LSM6DSM
  attachInterrupt(LIS2MDL_intPin , myinthandler2, RISING);  // define interrupt for intPin  output of LIS2MDL
  attachInterrupt(LPS22H_intPin  , myinthandler3, RISING);  // define interrupt for intPin  output of LPS22HB

  LIS2MDLstatus = LIS2MDL.status();  // read status register to clear interrupt before main loop
  }

  digitalWrite(myLed, LOW); // turn led off when successfully through setup

}

/* End of setup */

void loop() {

   if(passThru)
   {
   // If intPin goes high, either all data registers have new data
   if(newLSM6DSMData == true) {   // On interrupt, read data
      newLSM6DSMData = false;     // reset newData flag

     LSM6DSM.readData(LSM6DSMData); // INT2 cleared on any read
   
   // Now we'll calculate the accleration value into actual g's
     ax = (float)LSM6DSMData[4]*aRes - accelBias[0];  // get actual g value, this depends on scale being set
     ay = (float)LSM6DSMData[5]*aRes - accelBias[1];   
     az = (float)LSM6DSMData[6]*aRes - accelBias[2];  

   // Calculate the gyro value into actual degrees per second
     gx = (float)LSM6DSMData[1]*gRes - gyroBias[0];  // get actual gyro value, this depends on scale being set
     gy = (float)LSM6DSMData[2]*gRes - gyroBias[1];  
     gz = (float)LSM6DSMData[3]*gRes - gyroBias[2]; 

    for(uint8_t i = 0; i < 10; i++) { // iterate a fixed number of times per data read cycle
    Now = micros();
    deltat = ((Now - lastUpdate)/1000000.0f); // set integration time by time elapsed since last filter update
    lastUpdate = Now;

    sum += deltat; // sum for averaging filter update rate
    sumCount++;

    USFS.MadgwickQuaternionUpdate(-ax, ay, az, gx*pi/180.0f, -gy*pi/180.0f, -gz*pi/180.0f,  mx,  my, -mz);
    }
    
   }

    // If intPin goes high, either all data registers have new data
   if(newLIS2MDLData == true) {   // On interrupt, read data
      newLIS2MDLData = false;     // reset newData flag

     LIS2MDLstatus = LIS2MDL.status();
     
     if(LIS2MDLstatus & 0x08) // if all axes have new data ready
     {
      LIS2MDL.readData(LIS2MDLData);  
   
   // Now we'll calculate the accleration value into actual G's
     mx = (float)LIS2MDLData[0]*mRes - magBias[0];  // get actual G value 
     my = (float)LIS2MDLData[1]*mRes - magBias[1];   
     mz = (float)LIS2MDLData[2]*mRes - magBias[2]; 
     mx *= magScale[0];
     my *= magScale[1];
     mz *= magScale[2];  
     }
   }
   }  // end of "if(passThru)" handling

   if(!passThru)
   {
      /*EM7180*/
  // If intpin goes high, all data registers have new data
  if (newEM7180Data == true) { // On interrupt, read data
    newEM7180Data = false;  // reset newData flag

    // Check event status register, way to chech data ready by polling rather than interrupt
    uint8_t eventStatus = USFS.checkEM7180Status(); // reading clears the register

    // Check for errors
    if (eventStatus & 0x02) { // error detected, what is it?

      uint8_t errorStatus = USFS.checkEM7180Errors();
      if (errorStatus != 0x00) { // is there an error?
        Serial.print(" EM7180 sensor status = "); Serial.println(errorStatus);
        if (errorStatus == 0x11) Serial.print("Magnetometer failure!");
        if (errorStatus == 0x12) Serial.print("Accelerometer failure!");
        if (errorStatus == 0x14) Serial.print("Gyro failure!");
        if (errorStatus == 0x21) Serial.print("Magnetometer initialization failure!");
        if (errorStatus == 0x22) Serial.print("Accelerometer initialization failure!");
        if (errorStatus == 0x24) Serial.print("Gyro initialization failure!");
        if (errorStatus == 0x30) Serial.print("Math error!");
        if (errorStatus == 0x80) Serial.print("Invalid sample rate!");
      }

      // Handle errors ToDo

    }

    // if no errors, see if new data is ready
    if (eventStatus & 0x10) { // new acceleration data available
      USFS.readSENtralAccelData(accelCount);

      // Now we'll calculate the accleration value into actual g's
      Ax = (float)accelCount[0] * 0.000488f; // get actual g value
      Ay = (float)accelCount[1] * 0.000488f;
      Az = (float)accelCount[2] * 0.000488f;
    }

    if (eventStatus & 0x20) { // new gyro data available
      USFS.readSENtralGyroData(gyroCount);

      // Now we'll calculate the gyro value into actual dps's
      Gx = (float)gyroCount[0] * 0.153f; // get actual dps value
      Gy = (float)gyroCount[1] * 0.153f;
      Gz = (float)gyroCount[2] * 0.153f;
    }

    if (eventStatus & 0x08) { // new mag data available
      USFS.readSENtralMagData(magCount);

      // Now we'll calculate the mag value into actual G's
      Mx = (float)magCount[0] * 0.305176f; // get actual G value
      My = (float)magCount[1] * 0.305176f;
      Mz = (float)magCount[2] * 0.305176f;
    }

    if (eventStatus & 0x04) { // new quaternion data available
      USFS.readSENtralQuatData(Q);
    }

    // get MS5637 pressure
    if (eventStatus & 0x40) { // new baro data available
      rawPressure = USFS.readSENtralBaroData();
      Pressure = (float)rawPressure * 0.01f + 1013.25f; // pressure in mBar

      // get MS5637 temperature
      rawTemperature = USFS.readSENtralTempData();
      Temperature = (float) rawTemperature * 0.01f; // temperature in degrees C
    }
  } 
   } // end of "if(!passThru)" handling
   
   // end sensor interrupt handling
    
   if(passThru)
   {
    if(SerialDebug) {
    Serial.print("ax = "); Serial.print((int)1000*ax);  
    Serial.print(" ay = "); Serial.print((int)1000*ay); 
    Serial.print(" az = "); Serial.print((int)1000*az); Serial.println(" mg");
    Serial.print("gx = "); Serial.print( gx, 2); 
    Serial.print(" gy = "); Serial.print( gy, 2); 
    Serial.print(" gz = "); Serial.print( gz, 2); Serial.println(" deg/s");
    Serial.print("mx = "); Serial.print((int)1000*mx);  
    Serial.print(" my = "); Serial.print((int)1000*my); 
    Serial.print(" mz = "); Serial.print((int)1000*mz); Serial.println(" mG");
    
    Serial.print("q0 = "); Serial.print(q[0]);
    Serial.print(" qx = "); Serial.print(q[1]); 
    Serial.print(" qy = "); Serial.print(q[2]); 
    Serial.print(" qz = "); Serial.println(q[3]); 
    }

    // get pressure and temperature from the LPS22HB
    LPS22Hstatus = LPS22H.status();

    if(LPS22Hstatus & 0x01) { // if new pressure data available
    pressure = (float) LPS22H.readAltimeterPressure()/4096.0f;
    temperature = (float) LPS22H.readAltimeterTemperature()/100.0f; 
    
    altitude = 145366.45f*(1.0f - pow((pressure/1013.25f), 0.190284f)); 

      if(SerialDebug) {
      Serial.print("Altimeter temperature = "); Serial.print( temperature, 2); Serial.println(" C"); // temperature in degrees Celsius  
      Serial.print("Altimeter temperature = "); Serial.print(9.0f*temperature/5.0f + 32.0f, 2); Serial.println(" F"); // temperature in degrees Fahrenheit
      Serial.print("Altimeter pressure = "); Serial.print(pressure, 2);  Serial.println(" mbar");// pressure in millibar
      Serial.print("Altitude = "); Serial.print(altitude, 2); Serial.println(" feet");
      }
    }

    Gtemperature = ((float) LSM6DSMData[0]) / 256.0f + 25.0f; // Gyro chip temperature in degrees Centigrade
    // Print temperature in degrees Centigrade      
    if(SerialDebug) {
      Serial.print("Gyro temperature is ");  Serial.print(Gtemperature, 1);  Serial.println(" degrees C"); // Print T values to tenths of s degree C
    }

    LIS2MDLData[3] = LIS2MDL.readTemperature();
    Mtemperature = ((float) LIS2MDLData[3]) / 8.0f + 25.0f; // Mag chip temperature in degrees Centigrade
    // Print temperature in degrees Centigrade      
    if(SerialDebug) {
      Serial.print("Mag temperature is ");  Serial.print(Mtemperature, 1);  Serial.println(" degrees C"); // Print T values to tenths of s degree C
    }

    a12 =   2.0f * (q[1] * q[2] + q[0] * q[3]);
    a22 =   q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3];
    a31 =   2.0f * (q[0] * q[1] + q[2] * q[3]);
    a32 =   2.0f * (q[1] * q[3] - q[0] * q[2]);
    a33 =   q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];
    pitch = -asinf(a32);
    roll  = atan2f(a31, a33);
    yaw   = atan2f(a12, a22);
    pitch *= 180.0f / pi;
    yaw   *= 180.0f / pi; 
    yaw   += 13.8f; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
    if(yaw < 0) yaw   += 360.0f; // Ensure yaw stays between 0 and 360
    roll  *= 180.0f / pi;
    lin_ax = ax + a31;
    lin_ay = ay + a32;
    lin_az = az - a33;

    if(SerialDebug) {
    Serial.print("Yaw, Pitch, Roll: ");
    Serial.print(yaw, 2);
    Serial.print(", ");
    Serial.print(pitch, 2);
    Serial.print(", ");
    Serial.println(roll, 2);

    Serial.print("Grav_x, Grav_y, Grav_z: ");
    Serial.print(-a31*1000.0f, 2);
    Serial.print(", ");
    Serial.print(-a32*1000.0f, 2);
    Serial.print(", ");
    Serial.print(a33*1000.0f, 2);  Serial.println(" mg");
    Serial.print("Lin_ax, Lin_ay, Lin_az: ");
    Serial.print(lin_ax*1000.0f, 2);
    Serial.print(", ");
    Serial.print(lin_ay*1000.0f, 2);
    Serial.print(", ");
    Serial.print(lin_az*1000.0f, 2);  Serial.println(" mg");
    
    Serial.print("rate = "); Serial.print((float)sumCount/sum, 2); Serial.println(" Hz");
    }

//     Serial.print(millis()/1000);Serial.print(",");
//     Serial.print(yaw, 2); Serial.print(","); Serial.print(pitch, 2); Serial.print(","); Serial.print(roll, 2); Serial.print(","); Serial.println(Pressure, 2);

    sumCount = 0;
    sum = 0;      

    }  // end of "if(passThru)" handling

    if(!passThru)
    {

      if (SerialDebug) {
      Serial.print("Ax = "); Serial.print((int)1000 * Ax);
      Serial.print(" Ay = "); Serial.print((int)1000 * Ay);
      Serial.print(" Az = "); Serial.print((int)1000 * Az); Serial.println(" mg");
      Serial.print("Gx = "); Serial.print( Gx, 2);
      Serial.print(" Gy = "); Serial.print( Gy, 2);
      Serial.print(" Gz = "); Serial.print( Gz, 2); Serial.println(" deg/s");
      Serial.print("Mx = "); Serial.print( (int)Mx);
      Serial.print(" My = "); Serial.print( (int)My);
      Serial.print(" Mz = "); Serial.print( (int)Mz); Serial.println(" mG");

      Serial.println("Hardware quaternions:");
      Serial.print("Q0 = "); Serial.print(Q[0]);
      Serial.print(" Qx = "); Serial.print(Q[1]);
      Serial.print(" Qy = "); Serial.print(Q[2]);
      Serial.print(" Qz = "); Serial.println(Q[3]);
    }

    //Hardware AHRS:
    A112 =   2.0f * (Q[1] * Q[2] + Q[0] * Q[3]);
    A22 =   Q[0] * Q[0] + Q[1] * Q[1] - Q[2] * Q[2] - Q[3] * Q[3];
    A31 =   2.0f * (Q[0] * Q[1] + Q[2] * Q[3]);
    A32 =   2.0f * (Q[1] * Q[3] - Q[0] * Q[2]);
    A33 =   Q[0] * Q[0] - Q[1] * Q[1] - Q[2] * Q[2] + Q[3] * Q[3];
    Pitch = -asinf(A32);
    Roll  = atan2f(A31, A33);
    Yaw   = atan2f(A112, A22);
    Pitch *= 180.0f / pi;
    Yaw   *= 180.0f / pi;
    Yaw   += 13.8f; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
    if (Yaw < 0) Yaw   += 360.0f ; // Ensure yaw stays between 0 and 360
    Roll  *= 180.0f / pi;
    lin_Ax = Ax + A31;
    lin_Ay = Ay + A32;
    lin_Az = Az - A33;

    if (SerialDebug) {
      Serial.print("Hardware Yaw, pitch, Roll: ");
      Serial.print(Yaw, 2);
      Serial.print(", ");
      Serial.print(Pitch, 2);
      Serial.print(", ");
      Serial.println(Roll, 2);

      Serial.print("Hardware Grav_x, Grav_y, Grav_z: ");
      Serial.print(-A31 * 1000, 2);
      Serial.print(", ");
      Serial.print(-A32 * 1000, 2);
      Serial.print(", ");
      Serial.print(A33 * 1000, 2);  Serial.println(" mg");
      Serial.print("Hardware Lin_ax, Lin_ay, Lin_az: ");
      Serial.print(lin_Ax * 1000, 2);
      Serial.print(", ");
      Serial.print(lin_Ay * 1000, 2);
      Serial.print(", ");
      Serial.print(lin_Az * 1000, 2);  Serial.println(" mg");

      Serial.println("MS5637:");
      Serial.print("Altimeter temperature = ");
      Serial.print(Temperature, 2);
      Serial.println(" C"); // temperature in degrees Celsius
      Serial.print("Altimeter temperature = ");
      Serial.print(9.0f * Temperature / 5.0f + 32.0f, 2);
      Serial.println(" F"); // temperature in degrees Fahrenheit
      Serial.print("Altimeter pressure = ");
      Serial.print(Pressure, 2);
      Serial.println(" mbar");// pressure in millibar
      Altitude = 145366.45f * (1.0f - powf(((Pressure) / 1013.25f), 0.190284f));
      Serial.print("Altitude = ");
      Serial.print(Altitude, 2);
      Serial.println(" feet");
      Serial.println(" ");
    }
    
    } // end of "if(!passThru)" handling
   
      digitalWrite(myLed, HIGH); delay(1); digitalWrite(myLed, LOW);  // flash led for 10 milliseconds
      delay(500);
 
}  //end of loop

/*  End of main loop */


void myinthandler1()
{
  newLSM6DSMData = true;
}

void myinthandler2()
{
  newLIS2MDLData = true;
}

void myinthandler3()
{
  newLPS22HData = true;
}

void EM7180intHandler()
{
  newEM7180Data = true;
}




