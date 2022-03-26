/* 06/16/2017 Copyright Tlera Corporation
 *  
 *  Created by Kris Winer
 *  
 *  The AMS CCS811 is an air quality sensor that provides equivalent CO2 and volatile organic measurements from direct
 *  I2C register reads as well as current and voltage (effective resistance of the sensing element). Gas sensors, including 
 *  this MEMs gas sensor in the CCS811 measure resistance of a substrate that changes when exposed to inert gasses and 
 *  volatile organic compounds. Changed in concentration vary exponentially with the changes in resistance. The CCS811
 *  has an embedded ASIC calibrated against most common indoor pollutants that returns a good estimate of
 *  equivalent CO2 concentration in parts per million (400 - 8192 range) and volatile organic componds in parts per billion (0 - 1187).
 *  The sensor is quite sensitive to breath and other human emissions.
 *  
  *  The BME280 is a simple but high resolution pressure/humidity/temperature sensor, which can be used in its high resolution
 *  mode but with power consumption of 20 microAmp, or in a lower resolution mode with power consumption of
 *  only 1 microAmp. The choice will depend on the application.
 
    Library may be used freely and without limit with attribution.
 
  */
#include <Arduino.h>
#include <driver/adc.h>
#include "BME280.h"
#include "CCS811.h"
#include "I2Cdev.h"

#define uS_TO_S_FACTOR 1000000ULL  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  600        /* Time ESP32 will go to sleep (in seconds) */

RTC_DATA_ATTR int bootCount = 0;

void print_wakeup_reason(){
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch(wakeup_reason)
  {
//      case ESP_SLEEP_WAKEUP_EXT0 : USBSerial.println("Wakeup caused by external signal using RTC_IO"); break;
//      case ESP_SLEEP_WAKEUP_EXT1 : USBSerial.println("Wakeup caused by external signal using RTC_CNTL"); break;
//      case ESP_SLEEP_WAKEUP_TIMER : USBSerial.println("Wakeup caused by timer"); break;
//      case ESP_SLEEP_WAKEUP_TOUCHPAD : USBSerial.println("Wakeup caused by touchpad"); break;
//      case ESP_SLEEP_WAKEUP_ULP : USBSerial.println("Wakeup caused by ULP program"); break;
//      default : USBSerial.printf("Wakeup was not caused by deep sleep: %d\n",wakeup_reason); break;
  }
}

#define I2C_BUS          Wire               // Define the I2C bus (Wire instance) you wish to use

I2Cdev                   i2c_0(&I2C_BUS);   // Instantiate the I2Cdev object and point to the desired I2C bus

#define SerialDebug false  // set to true to get Serial output for debugging
const uint8_t myLed =  2;  // green led
const uint8_t myBat =  1;  // battery monitor

float VBat = 0.0f;
uint32_t chipId = 0;

// BME280 definitions
/* Specify BME280 configuration
 *  Choices are:
 P_OSR_01, P_OSR_02, P_OSR_04, P_OSR_08, P_OSR_16 // pressure oversampling
 H_OSR_01, H_OSR_02, H_OSR_04, H_OSR_08, H_OSR_16 // humidity oversampling
 T_OSR_01, T_OSR_02, T_OSR_04, T_OSR_08, T_OSR_16 // temperature oversampling
 full, BW0_223ODR,BW0_092ODR, BW0_042ODR, BW0_021ODR // bandwidth at 0.021 x sample rate
 BME280Sleep, forced,, forced2, normal //operation modes
 t_00_5ms = 0, t_62_5ms, t_125ms, t_250ms, t_500ms, t_1000ms, t_10ms, t_20ms // determines sample rate
 */
uint8_t Posr = P_OSR_01, Hosr = H_OSR_01, Tosr = T_OSR_01, Mode = BME280Sleep, IIRFilter = full, SBy = t_1000ms;     // set pressure amd temperature output data rate

float Temperature, Pressure, Humidity;              // stores BME280 pressures sensor pressure and temperature
int32_t rawPress, rawTemp, rawHumidity, compTemp;   // pressure and temperature raw count output for BME280
uint32_t compHumidity, compPress;                   // variables to hold raw BME280 humidity value

float temperature_C, temperature_F, pressure, humidity, altitude; // Scaled output of the BME280

BME280 BME280(&i2c_0); // instantiate BME280 class


// CCS811 definitions
#define CCS811_intPin  4
#define CCS811_wakePin 7

/* Specify CCS811 sensor parameters
 *  Choices are   dt_idle , dt_1sec, dt_10sec, dt_60sec
 */
//uint8_t AQRate = dt_60sec;  // set the sample rate
uint8_t AQRate = dt_idle;  // set the sample rate
uint8_t rawData[8] = {0, 0, 0, 0, 0, 0, 0, 0};  // array to hold the raw data
uint16_t eCO2 = 0, TVOC = 0;
uint8_t Current = 0;
float Voltage = 0.0f;

volatile bool newCCS811Data  = true; // boolean flag for interrupt

CCS811 CCS811(&i2c_0); // instantiate CCS811 class


void setup()
{
  if(SerialDebug) USBSerial.begin(115200);
  if(SerialDebug) USBSerial.println("Serial enabled!");

  pinMode(myLed, OUTPUT);
  digitalWrite(myLed, HIGH); // start with led off, active LOW
  pinMode(myBat, INPUT); // battery voltage monitor
  pinMode(CCS811_intPin, INPUT); // active LOW

  //Increment boot number and print it every reboot
  ++bootCount;
   if(SerialDebug) USBSerial.println("Boot number: " + String(bootCount));

  //Print the wakeup reason for ESP32
  print_wakeup_reason();

  if(bootCount == 1) { /*... Only need to do this part once on startup! */
    for(int i=0; i<17; i=i+8) {
      chipId |= ((ESP.getEfuseMac() >> (40 - i)) & 0xFF) << i;
    }

   if(SerialDebug) USBSerial.printf("ESP32 Chip model = %s Rev %d\n", ESP.getChipModel(), ESP.getChipRevision());
   if(SerialDebug) USBSerial.printf("This chip has %d cores\n", ESP.getChipCores());
   if(SerialDebug){USBSerial.print("Chip ID: "); USBSerial.println(chipId);}
  } /*... Only need to do this part once on startup! */

  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(ADC1_CHANNEL_1, ADC_ATTEN_DB_11);

  I2C_BUS.begin(0, 3);               // Set master mode, default on SDA on pin 0/SCL  on pin 3
  I2C_BUS.setClock(400000);          // I2C frequency at 400 kHz

  pinMode(CCS811_wakePin, OUTPUT);
  digitalWrite(CCS811_wakePin, HIGH); // set HIGH to disable the CCS811 air quality sensor

  if(bootCount == 1) { /*... Only need to do this part once on startup! */
  //Enable the CCS811 for I2C scan
  digitalWrite(CCS811_wakePin, LOW); // set LOW to enable the CCS811 air quality sensor
  
  if(SerialDebug) USBSerial.println("Scan for I2C devices:");
  i2c_0.I2Cscan();                   // should detect BME280 at 0x76 and CCS811 at 0x5A

  //Disable the CCS811 for I2C scan
  digitalWrite(CCS811_wakePin, HIGH); // set HIGH to disable the CCS811 air quality sensor
  delay(1);

  // Read the WHO_AM_I register of the BME280 this is a good test of communication
  byte BME280ChipID = BME280.getChipID();  // Read WHO_AM_I register for BME280
  if(SerialDebug){ 
    USBSerial.print("BME280 "); USBSerial.print("I AM "); USBSerial.print(BME280ChipID, HEX); USBSerial.print(" I should be "); USBSerial.println(0x60, HEX);
    USBSerial.println(" ");
  }

  // Read the WHO_AM_I register of the CCS811 this is a good test of communication
  digitalWrite(CCS811_wakePin, LOW); // set LOW to enable the CCS811 air quality sensor
  byte CCS811ChipID = CCS811.getChipID();
  digitalWrite(CCS811_wakePin, HIGH); // set HIGH to disable the CCS811 air quality sensor
  if(SerialDebug) {
    USBSerial.print("CCS811 "); USBSerial.print("I AM "); USBSerial.print(CCS811ChipID, HEX); USBSerial.print(" I should be "); USBSerial.println(0x81, HEX);
    USBSerial.println(" ");
  }
  
  if(BME280ChipID == 0x60 && CCS811ChipID == 0x81 ) {

   if(SerialDebug) {USBSerial.println("BME280+CCS811 are online..."); USBSerial.println(" ");}
   digitalWrite(myLed, LOW);

   BME280.resetBME280();                                                        // reset BME280 before initilization
   delay(10);
   BME280.BME280Init(Posr, Hosr, Tosr, Mode, IIRFilter, SBy);                   // Initialize BME280 altimeter
   BME280.BME280forced();                                                       // get initial data sample, then go back to sleep

   // initialize CCS811 and check version and status
   digitalWrite(CCS811_wakePin, LOW); // set LOW to enable the CCS811 air quality sensor
   CCS811.CCS811init(AQRate);
   digitalWrite(CCS811_wakePin, HIGH); // set HIGH to disable the CCS811 air quality sensor

   }
   else {
   if(BME280ChipID != 0x60 && SerialDebug) USBSerial.println(" BME280 not functioning!");    
   if(CCS811ChipID != 0x81 && SerialDebug) USBSerial.println(" CCS811 not functioning!");  
   while(1) { }; // no point proceeding, so wait here forever...
   }
  } /*... Only need to do this part once on startup! */
  

    /*   Put here anything you want to do before going to sleep */
    digitalWrite(CCS811_wakePin, LOW); // set LOW to enable the CCS811 air quality sensor
    CCS811.readCCS811Data(rawData);
    CCS811.compensateCCS811(compHumidity, compTemp); // compensate CCS811 using BME280 humidity and temperature
    digitalWrite(CCS811_wakePin, HIGH); // set HIGH to disable the CCS811 air quality sensor 

    eCO2 = (uint16_t) ((uint16_t) rawData[0] << 8 | rawData[1]);
    TVOC = (uint16_t) ((uint16_t) rawData[2] << 8 | rawData[3]);
    Current = (rawData[6] & 0xFC) >> 2;
    Voltage = (float) ((uint16_t) ((((uint16_t)rawData[6] & 0x02) << 8) | rawData[7])) * (1.65f/1023.0f); 
    
     if(SerialDebug){
      USBSerial.println("CCS811:");
      USBSerial.print("Eq CO2 in ppm = "); USBSerial.println(eCO2);
      USBSerial.print("TVOC in ppb = "); USBSerial.println(TVOC);
      USBSerial.print("Sensor current (uA) = "); USBSerial.println(Current);
      USBSerial.print("Sensor voltage (V) = "); USBSerial.println(Voltage, 2);  
      USBSerial.println(" ");
     }

    /* BME280 sensor data */
    BME280.BME280Init(Posr, Hosr, Tosr, Mode, IIRFilter, SBy);                   // Initialize BME280 altimeter
    BME280.BME280forced();  // get one data sample, then go back to sleep

    rawTemp =  BME280.readBME280Temperature();
    compTemp = BME280.BME280_compensate_T(rawTemp);
    temperature_C = (float) compTemp/100.0f;
    temperature_F = 9.0f*temperature_C/5.0f + 32.0f;
     
    rawPress =  BME280.readBME280Pressure();
    pressure = (float) BME280.BME280_compensate_P(rawPress)/25600.f; // Pressure in mbar
    altitude = 145366.45f*(1.0f - powf((pressure/1013.25f), 0.190284f));   
   
    rawHumidity =  BME280.readBME280Humidity();
    compHumidity = BME280.BME280_compensate_H(rawHumidity);
    humidity = (float)compHumidity/1024.0f; // Humidity in %RH

    if(SerialDebug) {
    USBSerial.println("BME280:");
    USBSerial.print("Altimeter temperature = "); 
    USBSerial.print( temperature_C, 2); 
    USBSerial.println(" C"); // temperature in degrees Celsius
    USBSerial.print("Altimeter temperature = "); 
    USBSerial.print(temperature_F, 2); 
    USBSerial.println(" F"); // temperature in degrees Fahrenheit
    USBSerial.print("Altimeter pressure = "); 
    USBSerial.print(pressure, 2);  
    USBSerial.println(" mbar");// pressure in millibar
    USBSerial.print("Altitude = "); 
    USBSerial.print(altitude, 2); 
    USBSerial.println(" feet");
    USBSerial.print("Altimeter humidity = "); 
    USBSerial.print(humidity, 1);  
    USBSerial.println(" %RH");// pressure in millibar
    USBSerial.println(" ");
    }
    /* ADC at attenuation 11 should read from 0 to 2.6 V nominally. The resistor divider
     *  is 1/2, and a calibration factor of 1.15 is applied to bring measurements into 
     *  agreement with multimeter */
    VBat = 2.0f * 2.60f * 1.15f * ((float) adc1_get_raw((adc1_channel_t)1)) / 4095.0f;
    if(SerialDebug) {USBSerial.print("Battery voltage = "); USBSerial.print(VBat, 2); USBSerial.println(" V");}
      
    digitalWrite(myLed, LOW); delay(1); digitalWrite(myLed, HIGH); // blink led at end of loop

    esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
    if(SerialDebug) USBSerial.println("Setup ESP32 to sleep for every " + String(TIME_TO_SLEEP) + " Seconds");
  
    //Go to sleep now
    if(SerialDebug) USBSerial.println("Going to sleep now");
    if(SerialDebug) USBSerial.flush(); 
    esp_deep_sleep_start();
    if(SerialDebug) USBSerial.println("This will never be printed");
}

void loop()
{
}
