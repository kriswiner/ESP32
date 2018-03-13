/* BME280 and VEML6040 Basic Example Code using ESP32
 by: Kris Winer
 date: December 14, 2016
 license: Beerware - Use this code however you'd like. If you 
 find it useful you can buy me a beer some time.
 
 This sketch uses SDA/SCL on pins 0/2, respectively, and it uses the ESP32.
 
 The BME280 is a simple but high resolution pressure/humidity/temperature sensor, which can be used in its high resolution
 mode but with power consumption of 20 microAmp, or in a lower resolution mode with power consumption of
 only 1 microAmp. The choice will depend on the application.

 VEML6040 color sensor senses red, green, blue, and white light and incorporates photodiodes, amplifiers, 
 and analog / digital circuits into a single chip using CMOS process. With the   color   sensor   applied,   
 the   brightness,   and   color temperature of backlight can be adjusted base on ambient light  source  
 that  makes  panel  looks  more  comfortable  for  end   user’s   eyes.   VEML6040’s   adoption   of   FiltronTM
 technology  achieves  the  closest  ambient  light  spectral  sensitivity to real human eye responses.

 VEML6040  provides  excellent  temperature  compensation  capability  for  keeping  the  output  stable  
 under  changing  temperature.   VEML6040’s   function   are   easily   operated   via the simple command format 
 of I2C (SMBus compatible) interface  protocol.  VEML6040’s  operating  voltage  ranges  from   2.5   V   to   
 3.6   V.   VEML6040   is   packaged   in   a   lead  (Pb)-free  4  pin  OPLGA  package  which  offers  the  best market-proven reliability.

 
 SDA and SCL should have 4K7 pull-up resistors (to 3.3V).
 
 Hardware setup:
 SDA ----------------------- 21
 SCL ----------------------- 22
 
  */
#include "Wire.h"   
#include <SPI.h>
#include <WiFi.h>
#include <WiFiClient.h>
//#include <WebServer.h>
//#include <ESP32mDNS.h>
//#include <ArduinoOTA.h>

/*#include "gpio.h"
extern "C" {
#include "user_interface.h"
  bool wifi_set_sleep_type(sleep_type_t);
  sleep_type_t wifi_get_sleep_type(void);
}
*/
// Must immediately declare functions to avoid "Not declared in this scope" errors
void      I2Cscan();
void      writeByte(uint8_t address, uint8_t subAddress, uint8_t data);
uint8_t   readByte(uint8_t address, uint8_t subAddress);
void      readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest);
int32_t  readBME280Temperature();
int32_t  readBME280Pressure();
int32_t  readBME280Humidity();
void      BME280Init();
uint32_t  BME280_compensate_P(int32_t adc_P);
int32_t   BME280_compensate_T(int32_t adc_T);
uint32_t  BME280_compensate_H(int32_t adc_H);
uint16_t  getRGBWdata(uint16_t * destination);
void      enableVEML6040();
//void      initWifi();

//ADC_MODE(ADC_VCC); // to use getVcc, don't use if using battery voltage monotor

// BME280 registers
#define BME280_HUM_LSB    0xFE
#define BME280_HUM_MSB    0xFD
#define BME280_TEMP_XLSB  0xFC
#define BME280_TEMP_LSB   0xFB
#define BME280_TEMP_MSB   0xFA
#define BME280_PRESS_XLSB 0xF9
#define BME280_PRESS_LSB  0xF8
#define BME280_PRESS_MSB  0xF7
#define BME280_CONFIG     0xF5
#define BME280_CTRL_MEAS  0xF4
#define BME280_STATUS     0xF3
#define BME280_CTRL_HUM   0xF2
#define BME280_RESET      0xE0
#define BME280_ID         0xD0  // should be 0x60
#define BME280_CALIB00    0x88
#define BME280_CALIB26    0xE1

////////////////////////////
// VEML6040 Command Codes //
////////////////////////////
#define  VEML6040_CONF            0x00 // command codes
#define  VEML6040_R_DATA          0x08  
#define  VEML6040_G_DATA          0x09 
#define  VEML6040_B_DATA          0x0A
#define  VEML6040_W_DATA          0x0B

#define VEML6040_ADDRESS         0x10
#define BME280_ADDRESS           0x76   // Address of BMP280 altimeter when ADO = 0

#define SerialDebug true  // set to true to get Serial output for debugging
#define myLed 5

enum Posr {
  P_OSR_00 = 0,  // no op
  P_OSR_01,
  P_OSR_02,
  P_OSR_04,
  P_OSR_08,
  P_OSR_16
};

enum Hosr {
  H_OSR_00 = 0,  // no op
  H_OSR_01,
  H_OSR_02,
  H_OSR_04,
  H_OSR_08,
  H_OSR_16
};

enum Tosr {
  T_OSR_00 = 0,  // no op
  T_OSR_01,
  T_OSR_02,
  T_OSR_04,
  T_OSR_08,
  T_OSR_16
};

enum IIRFilter {
  full = 0,  // bandwidth at full sample rate
  BW0_223ODR,
  BW0_092ODR,
  BW0_042ODR,
  BW0_021ODR // bandwidth at 0.021 x sample rate
};

enum Mode {
  BME280Sleep = 0,
  forced,
  forced2,
  normal
};

enum SBy {
  t_00_5ms = 0,
  t_62_5ms,
  t_125ms,
  t_250ms,
  t_500ms,
  t_1000ms,
  t_10ms,
  t_20ms,
};

enum IT {
  IT_40 = 0, //   40 ms
  IT_80,     //   80 ms
  IT_160,    //  160 ms
  IT_320,    //  320 ms
  IT_640,    //  640 ms
  IT_1280   // 1280 ms
};

// Specify BME280 configuration
uint8_t Posr = P_OSR_16, Hosr = H_OSR_16, Tosr = T_OSR_02, Mode = normal, IIRFilter = BW0_042ODR, SBy = t_62_5ms;     // set pressure amd temperature output data rate
// t_fine carries fine temperature as global value for BME280
int32_t t_fine;

// Specify VEML6070 Integration time
uint8_t IT = IT_160;
uint8_t ITime = 160;  // milliseconds
uint16_t RGBWData[4] = {0, 0, 0, 0};
float GSensitivity = 0.25168/((float) (IT + 1)); // ambient light sensitivity increases with integration time
float redLight, greenLight, blueLight, ambientLight;

float Temperature, Pressure, Humidity; // stores BME280 pressures sensor pressure and temperature
float VBAT;  // battery voltage from ESP8285 ADC read
int32_t rawPress, rawTemp, rawHumidity;   // pressure and temperature raw count output for BME280

// BME280 compensation parameters
uint8_t  dig_H1, dig_H3, dig_H6;
uint16_t dig_T1, dig_P1, dig_H4, dig_H5;
int16_t  dig_T2, dig_T3, dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9, dig_H2;

float   temperature_C, temperature_F, pressure, humidity, altitude; // Scaled output of the BME280

uint32_t delt_t = 0, count = 0, sumCount = 0;  // used to control display output rate

/*ESP8266WebServer server(80);
 
String webString="";     // String to display
 
void handle_root() {
  server.send(200, "text/plain", "Hello from the environmental monitoring station ESP8285!");
  delay(100);
}

  String createHTML(float var1, float var2, float var3, float var4, float var5, float var6, float var7, float var8, float var9, float var10) {
   webString = "<html><head><meta http-equiv=\"Refresh\" content=\"10\"></head><body><UL>"

  +String("<LI>Temperature = ")+String(var1)+String(" C</LI>") 
  +String("<LI>Temperature = ")+String(var2)+String(" F</LI>") 
  +String("<LI>Pressure = ")+String(var3)+String(" milliBar</LI>") 
  +String("<LI>Altitude = ")+String(var4)+String(" feet</LI>") 
  +String("<LI>Humidity = ")+String(var5)+String(" %RH</LI>") 
  +String("<LI>Red Light = ")+String(var6)+String(" microWatts/sq. cm</LI>") 
  +String("<LI>Green Light = ")+String(var7)+String(" microWatts/sq. cm</LI>") 
  +String("<LI>Blue Light = ")+String(var8)+String(" microWatts/sq. cm</LI>") 
  +String("<LI>Ambient Light = ")+String(var9)+String(" lux</LI>") 
  +String("<LI>Battery Voltage = ")+String(var10)+String(" V</LI>") 
     
  +"</UL></body></html>" ;
   return webString;
  }
*/

void setup()
{

  Serial.begin(115200);
  delay(4000);

  pinMode(myLed, OUTPUT);
  digitalWrite(myLed, HIGH);
  
  Wire.begin(21, 22, 400000); // SDA (21), SCL (22) on ESP32, 400 kHz rate

  I2Cscan(); // should detect BME280 at 0x76
  
  // Read the WHO_AM_I register of the BME280 this is a good test of communication
  byte f = readByte(BME280_ADDRESS, BME280_ID);  // Read WHO_AM_I register for BME280
  Serial.print("BME280 "); 
  Serial.print("I AM "); 
  Serial.print(f, HEX); 
  Serial.print(" I should be "); 
  Serial.println(0x60, HEX);
  Serial.println(" ");
  
 // delay(1000); 

  if(f == 0x60) {
   
  writeByte(BME280_ADDRESS, BME280_RESET, 0xB6); // reset BME280 before initilization
  delay(100);

  BME280Init(); // Initialize BME280 altimeter
  Serial.println("Calibration coeficients:");
  Serial.print("dig_T1 ="); 
  Serial.println(dig_T1);
  Serial.print("dig_T2 ="); 
  Serial.println(dig_T2);
  Serial.print("dig_T3 ="); 
  Serial.println(dig_T3);
  Serial.print("dig_P1 ="); 
  Serial.println(dig_P1);
  Serial.print("dig_P2 ="); 
  Serial.println(dig_P2);
  Serial.print("dig_P3 ="); 
  Serial.println(dig_P3);
  Serial.print("dig_P4 ="); 
  Serial.println(dig_P4);
  Serial.print("dig_P5 ="); 
  Serial.println(dig_P5);
  Serial.print("dig_P6 ="); 
  Serial.println(dig_P6);
  Serial.print("dig_P7 ="); 
  Serial.println(dig_P7);
  Serial.print("dig_P8 ="); 
  Serial.println(dig_P8);
  Serial.print("dig_P9 ="); 
  Serial.println(dig_P9);
  Serial.print("dig_H1 ="); 
  Serial.println(dig_H1);
  Serial.print("dig_H2 ="); 
  Serial.println(dig_H2);
  Serial.print("dig_H3 ="); 
  Serial.println(dig_H3);
  Serial.print("dig_H4 ="); 
  Serial.println(dig_H4);
  Serial.print("dig_H5 ="); 
  Serial.println(dig_H5);
  Serial.print("dig_H6 ="); 
  Serial.println(dig_H6);
 }
  else Serial.println(" BME280 not functioning!");

  enableVEML6040(); // initalize sensor
  delay(150);  

  // Get some information abut the ESP8285

  uint32_t freeheap = ESP.getFreeHeap();
  Serial.print("Free Heap Size = "); Serial.println(freeheap);
 // uint32_t chipID = ESP.getChipId();
 // Serial.print("ESP32 chip ID = "); Serial.println(chipID);
//  uint32_t flashChipID = ESP.getFlashChipId();
// Serial.print("ESP8285 flash chip ID = "); Serial.println(flashChipID);
  uint32_t flashChipSize = ESP.getFlashChipSize();
  Serial.print("ESP32 flash chip size = "); Serial.print(flashChipSize); Serial.println(" bytes");
  uint32_t flashChipSpeed = ESP.getFlashChipSpeed();
  Serial.print("ESP32 flash chip speed = "); Serial.print(flashChipSpeed); Serial.println(" Hz");
//  uint32_t getVcc = ESP.getVcc();
//  Serial.print("ESP8285 supply voltage = "); Serial.print(getVcc); Serial.println(" volts");

 /*    initWifi();

     Serial.println("Light sleep enabled");
     wifi_set_sleep_type(LIGHT_SLEEP_T); // Enable light sleep mode to save power
  
     server.on("/ESP8285Data", [](){

    // BME280 Data
    rawTemp =   readBME280Temperature();
    temperature_C = (float) BME280_compensate_T(rawTemp)/100.0; // temperature in Centigrade
    temperature_F = 9.*temperature_C/5. + 32.;
    rawPress =  readBME280Pressure();
    pressure = (float) BME280_compensate_P(rawPress)/25600.0; // Pressure in millibar
    altitude = 145366.45f*(1.0f - powf((pressure/1013.25f), 0.190284f));
    rawHumidity =  readBME280Humidity();
    humidity = (float) BME280_compensate_H(rawHumidity)/1024.0; // Humidity in %rH

    // VEML6040 Data
    getRGBWdata(RGBWData);
    redLight = (float)RGBWData[0]/96.0f;
    greenLight = (float)RGBWData[1]/74.0f;
    blueLight = (float)RGBWData[2]/56.0f;
    ambientLight = (float)RGBWData[1]*GSensitivity;

    // Battery Voltage
    VBAT = (1300.0/300.0) * (float)(analogRead(A0)) / 1024.0; // LiPo battery voltage in volts
 //   VBAT = (1100.0/100.0) * (float)(analogRead(A0)) / 1024.0; // 9V battery voltage in volts
    
    createHTML(temperature_C, temperature_F, pressure, altitude, humidity, redLight, greenLight, blueLight, ambientLight, VBAT);
    server.send(200, "text/html", webString);               // send to someone's browser when asked
    });
  
  server.begin();
  Serial.println("HTTP server started");

  Serial.println("LABEL,Timestamp,Pressure(mbar),Temperature(F),Humidity(rH%)");
  */
}
 

void loop()
{  
    rawTemp =   readBME280Temperature();
    temperature_C = (float) BME280_compensate_T(rawTemp)/100.0f;
    rawPress =  readBME280Pressure();
    pressure = (float) BME280_compensate_P(rawPress)/25600.0f; // Pressure in mbar
    rawHumidity =   readBME280Humidity();
    humidity = (float) BME280_compensate_H(rawHumidity)/1024.0f;
 
      Serial.println("BME280:");
      Serial.print("Altimeter temperature = "); 
      Serial.print( temperature_C, 2); 
      Serial.println(" C"); // temperature in degrees Celsius
      Serial.print("Altimeter temperature = "); 
      Serial.print(9.0f*temperature_C/5.0f + 32.0f, 2); 
      Serial.println(" F"); // temperature in degrees Fahrenheit
      Serial.print("Altimeter pressure = "); 
      Serial.print(pressure, 2); Serial.println(" mbar");// pressure in millibar

      Serial.print("DATA,");Serial.print("TIMER,"); 
      Serial.print(pressure); Serial.print(",");
      Serial.print(9.0f*temperature_C/5.0f + 32.0f, 2); Serial.print(",");
      Serial.print(humidity); Serial.println();

      altitude = 145366.45f*(1.0f - pow((pressure/1013.25f), 0.190284f));
      Serial.print("Altitude = "); 
      Serial.print(altitude, 2); 
      Serial.println(" feet");
      Serial.print("Altimeter humidity = "); 
      Serial.print(humidity, 1);  
      Serial.println(" %RH");// pressure in millibar
      Serial.println(" ");

      getRGBWdata(RGBWData);
      Serial.print("Red raw counts = ");   Serial.println(RGBWData[0]);
      Serial.print("Green raw counts = "); Serial.println(RGBWData[1]);
      Serial.print("Blue raw counts = ");  Serial.println(RGBWData[2]);
      Serial.print("White raw counts = "); Serial.println(RGBWData[3]);
      Serial.print("Inferred IR raw counts = "); Serial.println(RGBWData[3] - RGBWData[0] - RGBWData[1] - RGBWData[2]);
      Serial.println("  ");
 
      Serial.print("Red   light power density = "); Serial.print((float)RGBWData[0]/96.0f, 2); Serial.println(" microWatt/cm^2");
      Serial.print("Green light power density = "); Serial.print((float)RGBWData[1]/74.0f, 2); Serial.println(" microWatt/cm^2");
      Serial.print("Blue  light power density = "); Serial.print((float)RGBWData[2]/56.0f, 2); Serial.println(" microWatt/cm^2");
      Serial.println("  ");

      Serial.print("Ambient light intensity = "); Serial.print((float)RGBWData[1]*GSensitivity, 2); Serial.println(" lux");
      Serial.println("  ");

  // Empirical estimation of the correlated color temperature CCT:
  // see https://www.vishay.com/docs/84331/designingveml6040.pdf
      float temp = ( (float) (RGBWData[0] - RGBWData[2])/(float) RGBWData[1] );
      float CCT = 4278.6f*pow(temp, -1.2455f) + 0.5f;

      Serial.print("Correlated Color Temperature = "); Serial.print(CCT, 2); Serial.println(" Kelvin");
      Serial.println("  ");

      float VBAT = (100.0f/120.0f) * float(analogRead(34)) / 1024.0f;  // LiPo battery
      Serial.print("Battery Voltage = "); Serial.print(VBAT, 2); Serial.println(" V");    
  
      digitalWrite(myLed, HIGH); // blink blue led
      delay(100);
      digitalWrite(myLed, LOW);   

//      server.handleClient(); // serve data to web server site 
 
      delay(1000); // wait 1 seconds before refreshing data
}

//===================================================================================================================
//====== Set of useful function to access acceleration, gyroscope, magnetometer, and temperature data
//===================================================================================================================
/*void initWifi() {
  const char* ssid     = "NETGEAR16";
  const char* password = "braveroad553";
 // Connect to WiFi network
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("\n\r \n\rWorking to connect");

 // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("ESP8285 Environmental Data Server");
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
  }
*/
int32_t readBME280Temperature()
{
  uint8_t rawData[3];  // 20-bit pressure register data stored here
  readBytes(BME280_ADDRESS, BME280_TEMP_MSB, 3, &rawData[0]);  
  return (uint32_t) (((uint32_t) rawData[0] << 16 | (uint32_t) rawData[1] << 8 | rawData[2]) >> 4);
}

int32_t readBME280Pressure()
{
  uint8_t rawData[3];  // 20-bit pressure register data stored here
  readBytes(BME280_ADDRESS, BME280_PRESS_MSB, 3, &rawData[0]);  
  return (uint32_t) (((uint32_t) rawData[0] << 16 | (uint32_t) rawData[1] << 8 | rawData[2]) >> 4);
}

int32_t readBME280Humidity()
{
  uint8_t rawData[2];  // 16-bit humidity register data stored here
  readBytes(BME280_ADDRESS, BME280_HUM_MSB, 2, &rawData[0]);  
   return (uint32_t) (((uint32_t) rawData[0] << 24 | (uint32_t) rawData[1] << 16) ) >> 16;
}


void BME280Init()
{
  // Configure the BME280
  // Set H oversampling rate
  writeByte(BME280_ADDRESS, BME280_CTRL_HUM, 0x07 & Hosr);
  // Set T and P oversampling rates and sensor mode
  writeByte(BME280_ADDRESS, BME280_CTRL_MEAS, Tosr << 5 | Posr << 2 | Mode);
  // Set standby time interval in normal mode and bandwidth
  writeByte(BME280_ADDRESS, BME280_CONFIG, SBy << 5 | IIRFilter << 2);
  // Read and store calibration data
  uint8_t calib[26];
  readBytes(BME280_ADDRESS, BME280_CALIB00, 26, &calib[0]);
  dig_T1 = (uint16_t)(((uint16_t) calib[1] << 8) | calib[0]);
  dig_T2 = ( int16_t)((( int16_t) calib[3] << 8) | calib[2]);
  dig_T3 = ( int16_t)((( int16_t) calib[5] << 8) | calib[4]);
  dig_P1 = (uint16_t)(((uint16_t) calib[7] << 8) | calib[6]);
  dig_P2 = ( int16_t)((( int16_t) calib[9] << 8) | calib[8]);
  dig_P3 = ( int16_t)((( int16_t) calib[11] << 8) | calib[10]);
  dig_P4 = ( int16_t)((( int16_t) calib[13] << 8) | calib[12]);
  dig_P5 = ( int16_t)((( int16_t) calib[15] << 8) | calib[14]);
  dig_P6 = ( int16_t)((( int16_t) calib[17] << 8) | calib[16]);
  dig_P7 = ( int16_t)((( int16_t) calib[19] << 8) | calib[18]);
  dig_P8 = ( int16_t)((( int16_t) calib[21] << 8) | calib[20]);
  dig_P9 = ( int16_t)((( int16_t) calib[23] << 8) | calib[22]);
  dig_H1 = calib[25];
  readBytes(BME280_ADDRESS, BME280_CALIB26, 7, &calib[0]);
  dig_H2 = ( int16_t)((( int16_t) calib[1] << 8) | calib[0]);
  dig_H3 = calib[2];
  dig_H4 = ( int16_t)(((( int16_t) calib[3] << 8) | (0x0F & calib[4]) << 4) >> 4);
  dig_H5 = ( int16_t)(((( int16_t) calib[5] << 8) | (0xF0 & calib[4]) ) >> 4 );
  dig_H6 = calib[6];
}

// Returns temperature in DegC, resolution is 0.01 DegC. Output value of
// “5123” equals 51.23 DegC.
int32_t BME280_compensate_T(int32_t adc_T)
{
  int32_t var1, var2, T;
  var1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
  var2 = (((((adc_T >> 4) - ((int32_t)dig_T1)) * ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;
  t_fine = var1 + var2;
  T = (t_fine * 5 + 128) >> 8;
  return T;
}

// Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8
//fractional bits).
//Output value of “24674867” represents 24674867/256 = 96386.2 Pa = 963.862 hPa
uint32_t BME280_compensate_P(int32_t adc_P)
{
  long long var1, var2, p;
  var1 = ((long long)t_fine) - 128000;
  var2 = var1 * var1 * (long long)dig_P6;
  var2 = var2 + ((var1*(long long)dig_P5)<<17);
  var2 = var2 + (((long long)dig_P4)<<35);
  var1 = ((var1 * var1 * (long long)dig_P3)>>8) + ((var1 * (long long)dig_P2)<<12);
  var1 = (((((long long)1)<<47)+var1))*((long long)dig_P1)>>33;
  if(var1 == 0)
  {
    return 0;
    // avoid exception caused by division by zero
  }
  p = 1048576 - adc_P;
  p = (((p<<31) - var2)*3125)/var1;
  var1 = (((long long)dig_P9) * (p>>13) * (p>>13)) >> 25;
  var2 = (((long long)dig_P8) * p)>> 19;
  p = ((p + var1 + var2) >> 8) + (((long long)dig_P7)<<4);
  return (uint32_t)p;
}

// Returns humidity in %RH as unsigned 32 bit integer in Q22.10 format (22integer and 10fractional bits).
// Output value of “47445”represents 47445/1024= 46.333%RH
uint32_t BME280_compensate_H(int32_t adc_H)
{
int32_t var;

var = (t_fine - ((int32_t)76800));
var = (((((adc_H << 14) - (((int32_t)dig_H4) << 20) - (((int32_t)dig_H5) * var)) +
((int32_t)16384)) >> 15) * (((((((var * ((int32_t)dig_H6)) >> 10) * (((var *
((int32_t)dig_H3)) >> 11) + ((int32_t)32768))) >> 10) + ((int32_t)2097152)) * ((int32_t)dig_H2) + 8192) >> 14));
var = (var - (((((var >> 15) * (var >> 15)) >> 7) * ((int32_t)dig_H1)) >> 4));
var = (var < 0 ? 0 : var); 
var = (var > 419430400 ? 419430400 : var);
return(uint32_t)(var >> 12);
}

uint16_t getRGBWdata(uint16_t * destination)
{
    for (int j = 0; j < 4; j++)
    {
    uint8_t rawData[2] = {0, 0};
    Wire.beginTransmission(VEML6040_ADDRESS);
    Wire.write(VEML6040_R_DATA + j);        // Command code for reading rgbw data channels in sequence
    Wire.endTransmission(false);  // Send the Tx buffer, but send a restart to keep connection alive

    Wire.requestFrom(VEML6040_ADDRESS, 2);  // Read two bytes from slave register address 
    uint8_t i = 0;
    while (Wire.available()) 
    {
        rawData[i++] = Wire.read();       // Put read results in the Rx buffer
    }     
    Wire.endTransmission();
    destination[j] = ((uint16_t) rawData[1] << 8) | rawData[0];
    }
 
}

void enableVEML6040()
{
  Wire.beginTransmission(VEML6040_ADDRESS);
  Wire.write(VEML6040_CONF); // Command code for configuration register
  Wire.write(IT << 4); // Bit 3 must be 0, bit 0 is 0 for run and 1 for shutdown, LS Byte
  Wire.write(0x00); // MS Byte
  Wire.endTransmission();
}
 
// simple function to scan for I2C devices on the bus
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


// I2C read/write functions for the BMP280 sensors

  void writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
	Wire.beginTransmission(address);  // Initialize the Tx buffer
	Wire.write(subAddress);           // Put slave register address in Tx buffer
	Wire.write(data);                 // Put data in Tx buffer
	Wire.endTransmission();           // Send the Tx buffer
}

  uint8_t readByte(uint8_t address, uint8_t subAddress)
{
	uint8_t data; // `data` will store the register data	 
	Wire.beginTransmission(address);         // Initialize the Tx buffer
	Wire.write(subAddress);	                 // Put slave register address in Tx buffer
	Wire.endTransmission(false);             // Send the Tx buffer, but send a restart to keep connection alive
	Wire.requestFrom(address, 1);  // Read one byte from slave register address 
	data = Wire.read();                      // Fill Rx buffer with result
	return data;                             // Return data read from slave register
}

  void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest)
{  
	Wire.beginTransmission(address);   // Initialize the Tx buffer
	Wire.write(subAddress);            // Put slave register address in Tx buffer
	Wire.endTransmission(false);       // Send the Tx buffer, but send a restart to keep connection alive
	uint8_t i = 0;
  Wire.requestFrom(address, count);  // Read bytes from slave register address 
	while (Wire.available()) {
        dest[i++] = Wire.read(); }         // Put read results in the Rx buffer
}
