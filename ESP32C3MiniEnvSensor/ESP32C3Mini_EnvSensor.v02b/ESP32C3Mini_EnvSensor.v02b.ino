/* ESP32 C3 Mini Environmental Sensor
 
   This example code is in the public domain.
*/
#include "Arduino.h"
#include "LPS22HB.h"
#include "HDC2010.h"
#include "APDS9253.h"
#include "SPIFlash.h"
#include "WiFi.h"
#include "time.h"
#include <driver/adc.h> // for battery voltage reading

#define Serial USBSerial
#define SerialDebug true  // set to true to get Serial output for debugging, set to false to run on battery only

extern "C" void phy_bbpll_en_usb(bool en);

const char* ssid       = "YourNetworkName";
const char* password   = "YourNetworkPassword";

const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = -8*3600; // PST for California
const int   daylightOffset_sec = 3600;

struct tm timeinfo; // local time variables

/* create an ESP32 hardware timer */
hw_timer_t * timer = NULL;
volatile bool alarmFlag = false;

void IRAM_ATTR onTimer(){
  alarmFlag = true;
}

#define I2C_BUS    Wire              // Define the I2C bus (Wire instance) you wish to use

I2Cdev             i2c_0(&I2C_BUS);   // Instantiate the I2Cdev object and point to the desired I2C bus

// ESP32 C3 Mini pin configuration
const uint8_t myLed   =    2;  // blue led active LOW
// define battery voltage monitor pins
const uint8_t myBat   =    1;
const uint8_t myBatEn =   10;


// Battery voltage monitor definitions
float VBat = 0.0f;
uint32_t chipId = 0;
uint16_t ADCCounts = 0;

// Configure LPS22HB barometer
/* Specify sensor parameters (sample rate is twice the bandwidth) 
   Choices are P_1Hz, P_10Hz P_25 Hz, P_50Hz, and P_75Hz
 */
uint8_t PODR = P_1Hz;     // set pressure amd temperature output data rate
int32_t rawPressure = 0;
float LPSTemperature, LPSPressure, LPSAltitude;
uint8_t LPS22HBstatus, LPSintSource;

LPS22HB LPS22HB(&i2c_0);  // Instantiate LPS22HB barometer


// Configure HCD2010 humidity/temperature sensor
// Choices are:
// freq = ForceMode, Freq_120s, Freq_60s, Freq_10s, Freq_5s, Freq_1s, Freq_0_5s, Freq_0_2s
// tres = TRES_14bit, TRES_11bit, TRES_9bit
// hres = HRES_14bit, HRES_11bit, HRES_9bit
uint8_t freq = ForceMode, tres = TRES_14bit, hres = HRES_14bit;

float HDCTemperature = 0.0f, HDCHumidity = 0.0f;
uint16_t rawHDCTemperature = 0, rawHDCHumidity = 0;
uint8_t HDCStatus;


HDC2010 HDC2010(&i2c_0); // Instantiate HDC2010 Humidity sensor


// APDS9253 Configuration
uint8_t RGB_mode = RGBiR; // Choice is ALSandIR (green and IR channels only) or RGBiR for all four channels
//rate has to be slower than ADC settle time defines by resolution
// Bright sunlight is maximum ~25 klux so choose gain of 6x and minimum resolution (16 bits)
// that allows ~24 klux maximum to be measured;  a 1 Hz rate costs ~114 uA * 25/1000 ~ 3 uA
uint8_t LS_res = res16bit; // Choices are res20bit (400 ms), res19bit, res18bit (100 ms, default), res17bit, res16bit (25 ms).
uint8_t LS_rate = rate0_5Hz; // Choices are rate40Hz (25 ms), rate20Hz, rate10Hz (100 ms, default), rate5Hz, rate2_5Hz (400 ms), rate1Hz, rate0_5Hz
uint8_t LS_gain = gain6; // Choices are gain1, gain3 (default), gain6, gain9, gain18
uint32_t RGBiRData[4] = {0, 0, 0, 0}; // red, green, blue, ir counts
float ambientLight = 0; // ambient (green) light intensity in lux

float ALSluxTable[25]={         // lux per count for ALS depends on gain and resolution chosen
0.136, 0.273, 0.548, 1.099, 2.193,
0.045, 0.090, 0.180, 0.359, 0.722,
0.022, 0.045, 0.090, 0.179, 0.360,
0.015, 0.030, 0.059, 0.119, 0.239,
0.007, 0.015, 0.029, 0.059, 0.117
};

// Assume all channels have the same lux per LSB scaling
float luxScale = ALSluxTable[LS_gain * 5 + LS_res];

APDS9253 APDS9253(&i2c_0);  // Instantiate APDS9253 light sensor


// SPI flash configuration
const uint8_t CSPIN  = 4;
uint8_t flash_id[3] = {0, 0, 0};
uint16_t page_number = 0;     // set the page number for flash page write
uint8_t  sector_number = 0;   // set the sector number for sector write
uint8_t  flashPage[256];      // array to hold the data for flash page write

SPIFlash SPIFlash(CSPIN); // instantiate SPI flash class


void setup() 
{ 
  if(SerialDebug) phy_bbpll_en_usb(true); //this brings the USB serial-jtag back to life. 

  if(SerialDebug) Serial.begin(115200);
  if(SerialDebug) delay(4000);

  //connect to WiFi
  if(SerialDebug) Serial.printf("Connecting to %s ", ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      if(SerialDebug) Serial.print(".");
  }
  if(SerialDebug) Serial.println(" CONNECTED");
  
  //init and get the time
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  getLocalTime(&timeinfo);
  if(SerialDebug)Serial.print("The current date/time in Danville, California is ");
  if(SerialDebug) Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");

  //disconnect WiFi as it's no longer needed
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  
  // Configure led pin
  pinMode(myLed, OUTPUT);
  digitalWrite(myLed, LOW);  // start with led on since active LOW

  // Check ESP32 ID
  for(int i=0; i<17; i=i+8) {
    chipId |= ((ESP.getEfuseMac() >> (40 - i)) & 0xFF) << i;
  }

  if(SerialDebug) { 
  Serial.printf("ESP32 Chip model = %s Rev %d\n", ESP.getChipModel(), ESP.getChipRevision());
  Serial.printf("This chip has %d cores\n", ESP.getChipCores());
  Serial.print("Chip ID: "); Serial.println(chipId); Serial.println(" ");
  }

  // Configure ADC for battery voltage monitor
  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(ADC1_CHANNEL_1, ADC_ATTEN_DB_11); // ADC 1/Channel 1 is pin 1 -- myBat
  pinMode(myBat, INPUT);
  pinMode(myBatEn, OUTPUT);
  digitalWrite(myBatEn, HIGH);   
  ADCCounts = adc1_get_raw((adc1_channel_t)1);
  digitalWrite(myBatEn, LOW);   
  /* ADC at attenuation 11 should rad from 0 to 2.6 V nominally. The resistor divider
   *  is 1/2, and a calibration factor of 1.14 is applied to bring measurements into 
   *  agreement with multimeter */
  VBat = 2.0f * 2.60f * 1.14f * ((float) ADCCounts) / 4095.0f;
  if(SerialDebug) { 
    Serial.print("Battery voltage = "); Serial.print(VBat, 2); Serial.println(" V");
    Serial.print("Battery ADC Counts = "); Serial.print(ADCCounts); Serial.println(" Counts"); Serial.println(" ");
  }

   /* initialize wire bus */
  I2C_BUS.begin(0, 3);                // Set master mode, I2C_BUS.begin(SDA, SCL);
  I2C_BUS.setClock(400000);           // I2C frequency at 400 kHz
  delay(100);

  I2CscanDevices();                    // should detect all I2C devices on the bus

  if(SerialDebug) Serial.println("LPS22HB barometer...");
  uint8_t LPS22HB_chipID = LPS22HB.getChipID();
  if(SerialDebug) {
      Serial.print("LPS22HB "); Serial.print("I AM "); Serial.print(LPS22HB_chipID, HEX); Serial.print(" I should be "); Serial.println(0xB1, HEX);
      Serial.println(" ");
  }
  delay(100); 

  if(SerialDebug) Serial.println("HDC2010 humidity sensor...");
  uint16_t HDC2010_devID = HDC2010.getDevID(HDC2010_0_ADDRESS);
  if(SerialDebug) {
    Serial.print("DeviceID = 0x0"); Serial.print(HDC2010_devID, HEX); Serial.println(". Should be 0x07D0"); 
    Serial.println(" ");
  }

  uint16_t HDC2010_manuID = HDC2010.getManuID(HDC2010_0_ADDRESS);
  if(SerialDebug){ 
    Serial.print("Manufacturer's ID = 0x"); Serial.print(HDC2010_manuID, HEX); Serial.println(". Should be 0x5449");
    Serial.println(" ");
  }
  delay(100);

  // Read the APDS9253 Part ID register, this is a good test of communication
  if(SerialDebug)Serial.println("APDS9253 RGBiR Light Sensor...");
  byte APDS9253_ID = APDS9253.getChipID();  // Read PART_ID register for APDS9253
  if(SerialDebug) {
      Serial.print("APDS9253 "); Serial.print("chipID = 0x"); Serial.print(APDS9253_ID, HEX); Serial.print(", Should be 0x"); Serial.println(0xC2, HEX);
      Serial.println(" ");
  }
  delay(100);   

  if(LPS22HB_chipID == 0xB1 && HDC2010_devID == 0x07D0 && APDS9253_ID == 0xC2) // check if all I2C sensors with WHO_AM_I have acknowledged
  {
   if(SerialDebug) {Serial.println("LPS22HB, HDC2010, and APDS9253 are all online..."); Serial.println(" ");}

   LPS22HB.reset();
   LPS22HB.Init(PODR);  // Initialize LPS22HB barometer
   delay(100);
  
   HDC2010.reset(HDC2010_0_ADDRESS);

   // Configure HCD2010 for auto measurement mode if freq not ForceMode
   // else measurement performed only once each time init is called
   HDC2010.init(HDC2010_0_ADDRESS, hres, tres, freq); 

   if(SerialDebug) {Serial.print("Lux per count = "); Serial.println(luxScale, 3);}

   APDS9253.reset();
   delay(10);
   APDS9253.init(RGB_mode, LS_res, LS_rate, LS_gain);
   APDS9253.disable();

   digitalWrite(myLed, HIGH);  // when sensors successfully configured, turn off led
  }
   else 
  {
   if(LPS22HB_chipID != 0xB1 && SerialDebug)  Serial.println(" LPS22HB2 not functioning!");   
   if(HDC2010_devID != 0x07D0 && SerialDebug) Serial.println(" HDC20102 not functioning!");
   if(APDS9253_ID != 0xC2 && SerialDebug)     Serial.println(" APDS9253 not functioning!");
  }

  // configure SPI flash
  pinMode(CSPIN, OUTPUT);
  digitalWrite(CSPIN, HIGH);

 // check SPI Flash ID
  SPIFlash.init(6, 7, 5, 4);               // SPI.begin(SCK, MISO, MOSI, SS)
  SPIFlash.powerUp();                      // MX25R6435FZAI defaults to power down state
  
  SPIFlash.getChipID(flash_id);                    // Verify SPI flash communication
  if(flash_id[0] == 0xC2 && flash_id[1] == 0x28 && flash_id[2] == 0x17 && SerialDebug) {
  Serial.println(" ");  
  Serial.println("Found Macronix MX25R6435FZAI with Chip ID = 0xC2, 0x28, 0x17!");
  Serial.println(" ");  
  }
  else {
  if(SerialDebug) {
    Serial.println(" ");  
    Serial.println("no or unknown SPI flash!");
    Serial.println(" "); 
  } 
  }

  SPIFlash.powerDown();                    // power down SPI flash

  // Set up ESP32 timer
  /* Use 1st timer of 4 */
  /* 1 tick take 1/(80MHZ/80) = 1 us so we set divider 80 and count up */
  timer = timerBegin(0, 80, true);

  /* Attach onTimer function to our timer */
  timerAttachInterrupt(timer, &onTimer, false);

  /* Set alarm to call onTimer function every second 1 tick is 1us
  => 1 second is 1000000us */
  /* Repeat the alarm (third parameter) */
  timerAlarmWrite(timer, 300000000, true); // set time interval to five minute

  /* Start an alarm */
  timerAlarmEnable(timer);
  if(SerialDebug) Serial.println("start timer");
  
} // end of setup


void loop() 
{
  /*ESP32 timer*/
  if (alarmFlag) { // update RTC output at the alarm
      alarmFlag = false;

   /* APDS9253 Data Handling */
   APDS9253.enable(); // enable APDS9253 sensor
   while( !(APDS9253.getStatus() & 0x08) ) {}; // wait for data ready
   APDS9253.getRGBiRdata(RGBiRData); // read light sensor data
   APDS9253.disable(); // disable APDS9253 sensor
 
   if(SerialDebug) {
    Serial.print("Red raw counts = ");   Serial.println(RGBiRData[0]);
    Serial.print("Green raw counts = "); Serial.println(RGBiRData[1]);
    Serial.print("Blue raw counts = ");  Serial.println(RGBiRData[2]);
    Serial.print("IR raw counts = ");    Serial.println(RGBiRData[3]);
    Serial.println("  ");

    ambientLight = ((float) RGBiRData[1])*luxScale;
    Serial.print("Red intensity = ");   Serial.print(((float) RGBiRData[0])*luxScale); Serial.println(" lux");
    Serial.print("Green intensity = "); Serial.print(((float) RGBiRData[1])*luxScale); Serial.println(" lux");
    Serial.print("Blue intensity = ");  Serial.print(((float) RGBiRData[2])*luxScale); Serial.println(" lux");
    Serial.print("IR intensity = ");    Serial.print(((float) RGBiRData[3])*luxScale); Serial.println(" lux");
    Serial.println("  ");
   }
   
   
   /* HDC2010 data handling */
   HDC2010.forcedMode(HDC2010_0_ADDRESS);
   while( !(HDC2010.getIntStatus(HDC2010_0_ADDRESS) & 0x80) ) {}; // wait for HDC2010 data ready bit set
    
   rawHDCTemperature = HDC2010.getRawTemperature(HDC2010_0_ADDRESS); 
   HDCTemperature = ((float) rawHDCTemperature) * (165.0f/65536.0f) - 40.0f; // float degrees C, absolute accuracy +/- 0.2 C typical
   rawHDCHumidity    = HDC2010.getRawHumidity(HDC2010_0_ADDRESS);          
   HDCHumidity    = ((float) rawHDCHumidity) * (100.0f/65536.0f);   // float %rel humidity
 
   if(SerialDebug) {
     Serial.print("HDC2010 Temperature is "); Serial.print(HDCTemperature, 2); Serial.println(" degrees C");
     Serial.print("HDC2010 Humidity is "); Serial.print(HDCHumidity, 2); Serial.println(" %RH");
     Serial.println(" "); 
   }   /* end of HDC2010 data handling*/


   /* LPS22HB data handling */
//   LPS22HB.oneShot(); // baro data pretty jittery in one-shot mode, so run at 1 Hz instead...
   while( !(LPS22HB.status() & 0x03) ) {}; // wait for pressure and temperature data ready

   rawPressure = LPS22HB.readAltimeterPressure();
   LPSPressure = (float) rawPressure/4096.0f;
   LPSTemperature = (float) LPS22HB.readAltimeterTemperature()/100.0f; // absolute accuracy +/- 1.5 C
   LPSAltitude = 145366.45f*(1.0f - pow((LPSPressure/1013.25f), 0.190284f)); 

   if(SerialDebug) {
     Serial.print("Baro temperature = "); Serial.print(LPSTemperature, 2); Serial.print(" C"); // temperature in degrees Celsius  
     Serial.println(" ");
     Serial.print("Baro pressure = "); Serial.print(LPSPressure, 2);  Serial.print(" mbar");// pressure in millibar
     Serial.println(" ");     
     Serial.print("Altitude = "); Serial.print(LPSAltitude, 2); Serial.println(" ft");
     Serial.println(" ");
   }  /* end of LPS22HB interrupt handling */

     
   // get battery voltage
   digitalWrite(myBatEn, HIGH);   
   ADCCounts = adc1_get_raw((adc1_channel_t)1);
   digitalWrite(myBatEn, LOW);   
   /* ADC at attenuation 11 should rad from 0 to 2.6 V nominally. The resistor divider
    *  is 1/2, and a calibration factor of 1.14 is applied to bring measurements into 
    *  agreement with multimeter */
   VBat = 2.0f * 2.60f * 1.14f * ((float) ADCCounts) / 4095.0f;
   if(SerialDebug) { 
    Serial.print("Battery voltage = "); Serial.print(VBat, 2); Serial.println(" V");
    Serial.print("Battery ADC Counts = "); Serial.print(ADCCounts); Serial.println(" Counts"); Serial.println(" ");
   }


   getLocalTime(&timeinfo);
   if(SerialDebug) Serial.print("The current date/time in Danville, California is ");
   if(SerialDebug) Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S"); Serial.println(" ");

   // Capture time variables for logging https://www.cplusplus.com/reference/ctime/strftime/
   char timeYear[3];
   strftime(timeYear,3, "%y", &timeinfo);
   uint8_t Year = (uint8_t)atoi(timeYear);
   char timeMonth[3];
   strftime(timeMonth,3, "%m", &timeinfo);
   uint8_t Month = (uint8_t)atoi(timeMonth);
   char timeDay[3];
   strftime(timeDay,3, "%d", &timeinfo);
   uint8_t Day = (uint8_t)atoi(timeDay);
   char timeHour[3];
   strftime(timeHour,3, "%H", &timeinfo);
   uint8_t Hour = (uint8_t)atoi(timeHour); 
   char timeMinute[3];
   strftime(timeMinute,3, "%M", &timeinfo);
   uint8_t Minute = (uint8_t)atoi(timeMinute); 
   char timeSecond[3];
   strftime(timeSecond,3, "%S", &timeinfo);
   uint8_t Second = (uint8_t)atoi(timeSecond);


    /* Log some data to the QSPI flash */
    // Highest page number is 0x7FFF = 32767 for 64 Mbit flash
    // store some data to the SPI flash
    uint8_t bps = 23; // bytes per sector such that 256 bytes per page= sectors per page x bps = 11 x 23 <= 256
      if(sector_number < 11 && page_number < 0x7FFF) {
      
      flashPage[sector_number*bps + 0]  = (rawHDCTemperature & 0xFF00) >> 8;   // raw HDC2010 Temperature
      flashPage[sector_number*bps + 1]  = (rawHDCTemperature & 0x00FF); 
      flashPage[sector_number*bps + 2]  = (rawHDCHumidity & 0xFF00) >> 8;      // raw HDC2010 Humidity
      flashPage[sector_number*bps + 3]  = (rawHDCHumidity & 0x00FF);
      
      flashPage[sector_number*bps + 4]  = (rawPressure & 0x00FF0000) >> 16;    // LPS22HB raw Pressure
      flashPage[sector_number*bps + 5]  = (rawPressure & 0x0000FF00) >> 8; 
      flashPage[sector_number*bps + 6]  = (rawPressure & 0x000000FF);
      
      flashPage[sector_number*bps + 7]  = (RGBiRData[0] & 0xFF00) >> 8;        // APDS9253 RGBiR data, 16 bit or less 
      flashPage[sector_number*bps + 8]  = (RGBiRData[0] & 0x00FF);
      flashPage[sector_number*bps + 9]  = (RGBiRData[1] & 0xFF00) >> 8;
      flashPage[sector_number*bps + 10] = (RGBiRData[1] & 0x00FF);
      flashPage[sector_number*bps + 11] = (RGBiRData[2] & 0xFF00) >> 8;
      flashPage[sector_number*bps + 12] = (RGBiRData[2] & 0x00FF);
      flashPage[sector_number*bps + 13] = (RGBiRData[3] & 0xFF00) >> 8;
      flashPage[sector_number*bps + 14] = (RGBiRData[3] & 0x00FF);
      
      flashPage[sector_number*bps + 15] = (ADCCounts & 0xFF00) >> 8;           // raw VBAT from ADC
      flashPage[sector_number*bps + 16] =  ADCCounts & 0x00FF;

      flashPage[sector_number*bps + 17] =  Second;                            // Time and date
      flashPage[sector_number*bps + 18] =  Minute;
      flashPage[sector_number*bps + 19] =  Hour;
      flashPage[sector_number*bps + 20] =  Day;
      flashPage[sector_number*bps + 21] =  Month;
      flashPage[sector_number*bps + 22] =  Year;

      sector_number++;
      }
       
      if(sector_number == 11 && page_number < 0x7FFF)
      {
          SPIFlash.powerUp();
          SPIFlash.flash_page_program(flashPage, page_number);
          if(SerialDebug) {Serial.print("***Wrote flash page: "); Serial.println(page_number);}
          digitalWrite(myLed, LOW); delay(1); digitalWrite(myLed, HIGH); // indicate when flash page is written
          sector_number = 0;
          page_number++;
          SPIFlash.powerDown();  // Put SPI flash into power down mode
      }  
      else if(page_number >= 0x7FFF) 
      {
       if(SerialDebug){Serial.println("Reached last page of SPI flash!"); Serial.println("Data logging stopped!");}
      }
  
     digitalWrite(myLed, LOW); delay(1); digitalWrite(myLed, HIGH);
   } // end of time alarm section

     delay(1);    // wait for alarm
     
} // end of main loop


/* Useful functions */

void alarmMatch()
{
  alarmFlag = true;
}


void  I2CscanDevices() 
{
  // Scan for i2c devices
  uint8_t error, address, nDevices;

  Serial.println("Scanning for I2C Devices ...");

  nDevices = 0;
  for(address = 1; address < 127; address++ ) 
  {
    // The i2c_scanner uses the return value of the Wire.endTransmisstion to see if a device did acknowledge to the address.
    error = i2c_0.pollAddress(address);

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
