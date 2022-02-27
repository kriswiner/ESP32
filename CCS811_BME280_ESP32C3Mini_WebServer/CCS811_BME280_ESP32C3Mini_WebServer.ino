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
 *  This sketch uses default SDA/SCL pins on the Ladybug development board.
 *  The BME280 is a simple but high resolution pressure/humidity/temperature sensor, which can be used in its high resolution
 *  mode but with power consumption of 20 microAmp, or in a lower resolution mode with power consumption of
 *  only 1 microAmp. The choice will depend on the application.
 
    Library may be used freely and without limit with attribution.
 
  */
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <driver/adc.h>
#include "USB.h"
#include "BME280.h"
#include "CCS811.h"
#include "I2Cdev.h"

#define Serial USBSerial

const char* ssid = "XXXXXXXXX";
const char* password = "XXXXXXXXXXXX";

WebServer server(80);

String webString="";     // String to display

void handleRoot() {
  server.send(200, "text/plain", "Hello from the environmental monitroing station ESP32!");
  delay(100);
}

  String createHTML(float var1, float var2, float var3, float var4, float var5, float var6, float var7, float var8) {
   webString = "<html><head><meta http-equiv=\"Refresh\" content=\"5\"></head><body><UL>"

  +String("<LI>Temperature = ")+String(var1)+String(" C</LI>") 
  +String("<LI>Temperature = ")+String(var2)+String(" F</LI>") 
  +String("<LI>Pressure = ")+String(var3)+String(" milliBar</LI>") 
  +String("<LI>Altitude = ")+String(var4)+String(" feet</LI>") 
  +String("<LI>Humidity = ")+String(var5)+String(" %RH</LI>") 
  +String("<LI>eCO2 = ")+String(var6)+String(" ppm</LI>") 
  +String("<LI>TVOC = ")+String(var7)+String(" ppb</LI>") 
  +String("<LI>Battery Voltage = ")+String(var8)+String(" V</LI>") 
     
  +"</UL></body></html>" ;
   return webString;
  }
  
void handleNotFound() {
  String message = "File Not Found\n\n";
  message += "URI: ";
  message += server.uri();
  message += "\nMethod: ";
  message += (server.method() == HTTP_GET) ? "GET" : "POST";
  message += "\nArguments: ";
  message += server.args();
  message += "\n";
  for (uint8_t i = 0; i < server.args(); i++) {
    message += " " + server.argName(i) + ": " + server.arg(i) + "\n";
  }
  server.send(404, "text/plain", message);
}

/* create an ESP32 hardware timer */
hw_timer_t * timer = NULL;
volatile bool alarmFlag = false;

void IRAM_ATTR onTimer(){
  alarmFlag = true;
}

#define I2C_BUS          Wire               // Define the I2C bus (Wire instance) you wish to use

I2Cdev                   i2c_0(&I2C_BUS);   // Instantiate the I2Cdev object and point to the desired I2C bus

#define SerialDebug true  // set to true to get Serial output for debugging
#define       myLed    2 // green led
const uint8_t myBat =  1;

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
uint8_t AQRate = dt_10sec;  // set the sample rate
uint8_t rawData[8] = {0, 0, 0, 0, 0, 0, 0, 0};  // array to hold the raw data
uint16_t eCO2 = 0, TVOC = 0;
uint8_t Current = 0;
float Voltage = 0.0f;

volatile bool newCCS811Data  = true; // boolean flag for interrupt

CCS811 CCS811(&i2c_0); // instantiate CCS811 class


void setup()
{
  Serial.begin(115200);
  delay(4000);
  Serial.println("Serial enabled!");

  pinMode(myLed, OUTPUT);
  digitalWrite(myLed, LOW); // start with led on, active LOW

  for(int i=0; i<17; i=i+8) {
    chipId |= ((ESP.getEfuseMac() >> (40 - i)) & 0xFF) << i;
  }

  Serial.printf("ESP32 Chip model = %s Rev %d\n", ESP.getChipModel(), ESP.getChipRevision());
  Serial.printf("This chip has %d cores\n", ESP.getChipCores());
  Serial.print("Chip ID: "); Serial.println(chipId);

  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(ADC1_CHANNEL_1, ADC_ATTEN_DB_11);

  pinMode(myBat, INPUT); // battery voltage monitor

  pinMode(CCS811_intPin, INPUT_PULLUP); // active LOW

  I2C_BUS.begin(0, 3);               // Set master mode, default on SDA on pin 0/SCL  on pin 3
  I2C_BUS.setClock(400000);          // I2C frequency at 400 kHz
  delay(1000);

  pinMode(CCS811_wakePin, OUTPUT);
  //Enable the CCS811 for I2C scan
  digitalWrite(CCS811_wakePin, LOW); // set LOW to enable the CCS811 air quality sensor
  
  Serial.println("Scan for I2C devices:");
  i2c_0.I2Cscan();                   // should detect BME280 at 0x76 and CCS811 at 0x5A
  delay(1000);

  //Disable the CCS811 for I2C scan
  digitalWrite(CCS811_wakePin, HIGH); // set LOW to enable the CCS811 air quality sensor
  delay(1000);

  // Read the WHO_AM_I register of the BME280 this is a good test of communication
  byte BME280ChipID = BME280.getChipID();  // Read WHO_AM_I register for BME280
  Serial.print("BME280 "); Serial.print("I AM "); Serial.print(BME280ChipID, HEX); Serial.print(" I should be "); Serial.println(0x60, HEX);
  Serial.println(" ");
  delay(1000); 

  // Read the WHO_AM_I register of the CCS811 this is a good test of communication
  digitalWrite(CCS811_wakePin, LOW); // set LOW to enable the CCS811 air quality sensor
  byte CCS811ChipID = CCS811.getChipID();
  digitalWrite(CCS811_wakePin, HIGH); // set HIGH to disable the CCS811 air quality sensor
  Serial.print("CCS811 "); Serial.print("I AM "); Serial.print(CCS811ChipID, HEX); Serial.print(" I should be "); Serial.println(0x81, HEX);
  Serial.println(" ");
  delay(1000); 
  
  if(BME280ChipID == 0x60 && CCS811ChipID == 0x81 ) {

   Serial.println("BME280+CCS811 are online..."); Serial.println(" ");
   digitalWrite(myLed, LOW);

   BME280.resetBME280();                                                        // reset BME280 before initilization
   delay(100);
   BME280.BME280Init(Posr, Hosr, Tosr, Mode, IIRFilter, SBy);                   // Initialize BME280 altimeter
   BME280.BME280forced();                                                       // get initial data sample, then go back to sleep

   // initialize CCS811 and check version and status
   digitalWrite(CCS811_wakePin, LOW); // set LOW to enable the CCS811 air quality sensor
   CCS811.CCS811init(AQRate);
   digitalWrite(CCS811_wakePin, HIGH); // set HIGH to disable the CCS811 air quality sensor

   }
   else {
   if(BME280ChipID != 0x60) Serial.println(" BME280 not functioning!");    
   if(CCS811ChipID != 0x81) Serial.println(" CCS811 not functioning!");  
   while(1) { }; // no point proceeding, so wait here forever...
   }

  // Set up ESP32 timer
  /* Use 1st timer of 4 */
  /* 1 tick take 1/(80MHZ/80) = 1us so we set divider 80 and count up */
  timer = timerBegin(0, 80, true);

  /* Attach onTimer function to our timer */
  timerAttachInterrupt(timer, &onTimer, true);

  /* Set alarm to call onTimer function every second 1 tick is 1us
  => 1 second is 1000000us */
  /* Repeat the alarm (third parameter) */
  timerAlarmWrite(timer, 10000000, true); // update every ten seconds

  /* Start an alarm */
  timerAlarmEnable(timer);
  Serial.println("start timer");

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.println("");

  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  if (MDNS.begin("esp32")) {
    Serial.println("MDNS responder started");
  }

  server.on("/", handleRoot);

  server.onNotFound(handleNotFound);

  server.on("/ESP32Data", [](){ // define web server data

    // BME280 Data
    rawTemp =   BME280.readBME280Temperature();
    temperature_C = (float) BME280.BME280_compensate_T(rawTemp)/100.0f; // temperature in Centigrade
    temperature_F = 9.0f*temperature_C/5.0f + 32.0f;
    rawPress =  BME280.readBME280Pressure();
    pressure = (float) BME280.BME280_compensate_P(rawPress)/25600.0f; // Pressure in millibar
    altitude = 145366.45f*(1.0f - powf((pressure/1013.25f), 0.190284f));
    rawHumidity =  BME280.readBME280Humidity();
    humidity = (float) BME280.BME280_compensate_H(rawHumidity)/1024.0f; // Humidity in %RH

    // CCS811 Data
    digitalWrite(CCS811_wakePin, LOW); // set LOW to enable the CCS811 air quality sensor
    CCS811.readCCS811Data(rawData);
    CCS811.compensateCCS811(compHumidity, compTemp); // compensate CCS811 using BME280 humidity and temperature
    digitalWrite(CCS811_wakePin, HIGH); // set LOW to enable the CCS811 air quality sensor 

    eCO2 = (uint16_t) ((uint16_t) rawData[0] << 8 | rawData[1]);
    TVOC = (uint16_t) ((uint16_t) rawData[2] << 8 | rawData[3]);
    Current = (rawData[6] & 0xFC) >> 2;
    Voltage = (float) ((uint16_t) ((((uint16_t)rawData[6] & 0x02) << 8) | rawData[7])) * (1.65f/1023.0f); 

    // Battery Voltage
    VBat = 2.0f * 2.60f * 1.15f * ((float) adc1_get_raw((adc1_channel_t)1)) / 4095.0f;
    
    createHTML(temperature_C, temperature_F, pressure, altitude, humidity, eCO2, TVOC, VBat);
    server.send(200, "text/html", webString);               // send to someone's browser when asked
  });

  server.begin();
  Serial.println("HTTP server started");
  
  attachInterrupt(CCS811_intPin,  myinthandler, FALLING); // enable CCS811 interrupt 
}

void loop()
{  
    // CCS811 data
    // If intPin goes LOW, all data registers have new data
    if(newCCS811Data == true) {  // On interrupt, read data
    newCCS811Data = false;  // reset newData flag
     
    digitalWrite(CCS811_wakePin, LOW); // set LOW to enable the CCS811 air quality sensor
    CCS811.readCCS811Data(rawData);
    CCS811.compensateCCS811(compHumidity, compTemp); // compensate CCS811 using BME280 humidity and temperature
    digitalWrite(CCS811_wakePin, HIGH); // set LOW to enable the CCS811 air quality sensor 

    eCO2 = (uint16_t) ((uint16_t) rawData[0] << 8 | rawData[1]);
    TVOC = (uint16_t) ((uint16_t) rawData[2] << 8 | rawData[3]);
    Current = (rawData[6] & 0xFC) >> 2;
    Voltage = (float) ((uint16_t) ((((uint16_t)rawData[6] & 0x02) << 8) | rawData[7])) * (1.65f/1023.0f); 
    
    Serial.println("CCS811:");
    Serial.print("Eq CO2 in ppm = "); Serial.println(eCO2);
    Serial.print("TVOC in ppb = "); Serial.println(TVOC);
    Serial.print("Sensor current (uA) = "); Serial.println(Current);
    Serial.print("Sensor voltage (V) = "); Serial.println(Voltage, 2);  
    Serial.println(" ");
  }
            
   /*RTC Timer*/
   if (alarmFlag) { // update serial output whenever there is a timer alarm
      alarmFlag = false;

    /* BME280 sensor data */
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
 
    Serial.println("BME280:");
    Serial.print("Altimeter temperature = "); 
    Serial.print( temperature_C, 2); 
    Serial.println(" C"); // temperature in degrees Celsius
    Serial.print("Altimeter temperature = "); 
    Serial.print(temperature_F, 2); 
    Serial.println(" F"); // temperature in degrees Fahrenheit
    Serial.print("Altimeter pressure = "); 
    Serial.print(pressure, 2);  
    Serial.println(" mbar");// pressure in millibar
    Serial.print("Altitude = "); 
    Serial.print(altitude, 2); 
    Serial.println(" feet");
    Serial.print("Altimeter humidity = "); 
    Serial.print(humidity, 1);  
    Serial.println(" %RH");// pressure in millibar
    Serial.println(" ");

    uint16_t Counts = adc1_get_raw((adc1_channel_t)1);
    /* ADC at attenuation 11 should rad from 0 to 2.6 V nominally. The resistor divider
     *  is 1/2, and a calibration factor of 1.15 is applied to bring measurements into 
     *  agreement with multimeter */
    VBat = 2.0f * 2.60f * 1.15f * ((float) Counts) / 4095.0f;
    Serial.print("Battery voltage = "); Serial.print(VBat, 2); Serial.println(" V");

    server.handleClient(); // update web server page
      
    digitalWrite(myLed, LOW); delay(10); digitalWrite(myLed, HIGH); // blink led at end of loop
    }  
    
 yield(); //allow the cpu to switch to other tasks
//     ESP32.sleep();    // time out in deep sleep mode to save power
}

//===================================================================================================================
//====== Set of useful functions
//===================================================================================================================

void myinthandler()
{
  newCCS811Data = true;
}


void alarmMatch()
{
  alarmFlag = true;
}
