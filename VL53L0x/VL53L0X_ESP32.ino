/* This example shows how to use continuous mode to take
range measurements with the VL53L0X. It is based on
vl53l0x_ContinuousRanging_Example.c from the VL53L0X API.

Basic sketch from Pololu version of ST Microelectronic's API
see: https://github.com/pololu/vl53l0x-arduino

Further modified by Kris Winer for the ESP32 and interrupt data ready, etc.

The range readings are in units of mm. */

#include <Wire.h>
#include <VL53L0X.h>

VL53L0X sensor;

// Uncomment this line to use long range mode. This
// increases the sensitivity of the sensor and extends its
// potential range, but increases the likelihood of getting
// an inaccurate reading because of reflections from objects
// other than the intended target. It works best in dark
// conditions.

//#define LONG_RANGE


// Uncomment ONE of these two lines to get
// - higher speed at the cost of lower accuracy OR
// - higher accuracy at the cost of lower speed

//#define HIGH_SPEED
#define HIGH_ACCURACY

// Pin definitions
int myLed = 5;
int intPin = 14;

bool newData = false;

uint32_t delt_t = 0, count = 0, sumCount = 0;  // used to control display output rate
float deltat = 0.0f, sum = 0.0f;          // integration interval for both filter schemes
uint32_t lastUpdate = 0, firstUpdate = 0; // used to calculate integration interval
uint32_t Now = 0;                         // used to calculate integration interval

void setup()
{
  Serial.begin(115200);
  delay(4000);
  
 Wire.begin(21, 22, 400000); // SDA (21), SCL (22) on ESP32, 400 kHz rate
 
// Set up the led indicator
  pinMode(myLed, OUTPUT);
  digitalWrite(myLed, LOW);
  pinMode(intPin, INPUT);

  I2Cscan();
  
  delay(1000);
  
  sensor.init();
  sensor.setTimeout(500);

  #if defined LONG_RANGE
  // lower the return signal rate limit (default is 0.25 MCPS)
  sensor.setSignalRateLimit(0.1);
  // increase laser pulse periods (defaults are 14 and 10 PCLKs)
  sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
  sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
  #endif

  #if defined HIGH_SPEED
  // reduce timing budget to 20 ms (default is about 33 ms)
  sensor.setMeasurementTimingBudget(20000);  // minimum timing budget 20 ms
  #elif defined HIGH_ACCURACY
  // increase timing budget to 200 ms
  sensor.setMeasurementTimingBudget(200000);
  #endif

  // Start continuous back-to-back mode (take readings as
  // fast as possible).  To use continuous timed mode
  // instead, provide a desired inter-measurement period in
  // ms (e.g. sensor.startContinuous(100)).
  sensor.startContinuous();

  attachInterrupt(intPin, myinthandler, FALLING);  // define interrupt for GPI01 pin output of VL53L0X

}

void loop()
{  
  if (newData) // wait for data ready interrupt
  {
     newData = false; // reset data ready flag
     Now = micros(); // capture interrupt time
     // calculate time between last interrupt and current one, convert to sample data rate, and print to serial monitor
     Serial.print("data rate = "); Serial.print(1000000./(Now - lastUpdate)); Serial.println(" Hz");

     Serial.print(sensor.readRangeContinuousMillimeters()); // prit range in mm to serial monitor
     if (sensor.timeoutOccurred()) { Serial.print(" TIMEOUT"); }

     Serial.println();
  }
  lastUpdate = Now;
}

/****************************************************************************************************/
/* Useful functions*/
/****************************************************************************************************/
void myinthandler()
{
  newData = true; // set the new data ready flag to true on interrupt
}

// I2C scan function
void I2Cscan()
{
// scan for i2c devices
  byte error, address;
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for(address = 1; address < 127; address++ ) 
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

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
    Serial.println("done\n");
    
}
