/* readSPIFlash_ESP32C3Mini_EnvSensor.v02b
 *  
2021 Copyright Tlera Corporation 

April 30, 2022

Sketch to read the QSPI Flash on the ESP32C3Mini Environmental Sensor v.02b, reconstruct the sensor data, and output CMS-compatible data for plotting.
 */

#include "SPIFlash.h"
#include "SPI.h"

#define Serial USBSerial

float HDCTemperature = 0.0f, HDCHumidity = 0.0f;
uint16_t rawHDCTemperature = 0, rawHDCHumidity = 0;
int32_t rawPress = 0;
float pressure, altitude; // Scaled output of the LPS22HB
uint8_t Second, Minute, Hour, Day, Month, Year;
float VBat = 0.0f;
uint16_t ADCCounts = 0;

uint8_t LS_gain = 0x02;  
uint8_t LS_res  = 0x04;  
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

// SPI flash configuration
//  Highest page number is 0x7FFF = 32767 for 64 Mbit flash
uint16_t max_page_number = 0x7FFF;
const uint8_t CSPIN  = 4;
uint8_t flash_id[3] = {0, 0, 0};
uint16_t page_number = 0;     // set the page number for flash page write
uint8_t  sector_number = 0;   // set the sector number for sector write
uint8_t  flashPage[256];      // array to hold the data for flash page write
uint8_t  bps = 23;            // bytes per sector such that 256 bytes per page= sectors per page x bps = 11 x 23

SPIFlash SPIFlash(CSPIN); // instantiate SPI flash class
 

void setup(void)
{ 
  Serial.begin(115200);
  while (!Serial) { }
  Serial.println("Serial enabled!");

  // configure SPI flash
  pinMode(CSPIN, OUTPUT);
  digitalWrite(CSPIN, HIGH);

 // check SPI Flash ID
  SPIFlash.init(6, 7, 5, 4);               // SPI.begin(SCK, MISO, MOSI, SS)
  SPIFlash.powerUp();                      // MX25R6435FZAI defaults to power down state
  
  SPIFlash.getChipID(flash_id);                    // Verify SPI flash communication
  if(flash_id[0] == 0xC2 && flash_id[1] == 0x28 && flash_id[2] == 0x17) {
  Serial.println(" ");  
  Serial.println("Found Macronix MX25R6435FZAI with Chip ID = 0xC2, 0x28, 0x17!");
  Serial.println(" ");  
  }
  else {
  Serial.println(" ");  
  Serial.println("no or unknown SPI flash!");
  Serial.println(" ");  
  }
  delay(4000); // give some time to read the screen
  
  // read the SPI flash
  for(page_number = 0; page_number < 10; page_number++)  { // change the page number limit to correspond to number of pages logged

//  Serial.print("Read Page 0x"); Serial.println(page_number, HEX);
   SPIFlash.flash_read_pages(flashPage, page_number, 1);
      
   for(sector_number = 0; sector_number < 11; sector_number++) {
    
    rawHDCTemperature =  ((uint16_t) flashPage[sector_number*bps + 0] << 8) | flashPage[sector_number*bps + 1];
    rawHDCHumidity =     ((uint16_t) flashPage[sector_number*bps + 2] << 8) | flashPage[sector_number*bps + 3];
    rawPress =           ((int32_t)  flashPage[sector_number*bps + 4] << 16) |  ((int32_t)flashPage[sector_number*bps + 5] << 8) | flashPage[sector_number*bps + 6];

    Second =  flashPage[sector_number*bps + 17];
    Minute =  flashPage[sector_number*bps + 18];
    Hour =    flashPage[sector_number*bps + 19];
    Day =     flashPage[sector_number*bps + 20];
    Month =   flashPage[sector_number*bps + 21];
    Year =    flashPage[sector_number*bps + 22];

    HDCTemperature = ((float) rawHDCTemperature) * (165.0f/65536.0f) - 40.0f; // float degrees C, absolute accuracy +/- 0.2 C typical
    HDCHumidity    = ((float) rawHDCHumidity) * (100.0f/65536.0f);   // float %rel humidity
      
    pressure = (float) rawPress/4096.0f; // Pressure in mbar
    altitude = 145366.45f*(1.0f - powf((pressure/1013.25f), 0.190284f));   
   

    RGBiRData[0] = ((uint16_t) flashPage[sector_number*bps +  7] << 8) |  flashPage[sector_number*bps + 8];
    RGBiRData[1] = ((uint16_t) flashPage[sector_number*bps +  9] << 8) |  flashPage[sector_number*bps + 10];
    RGBiRData[2] = ((uint16_t) flashPage[sector_number*bps + 11] << 8) |  flashPage[sector_number*bps + 12];
    RGBiRData[3] = ((uint16_t) flashPage[sector_number*bps + 13] << 8) |  flashPage[sector_number*bps + 14];

    ADCCounts = ((uint16_t) flashPage[sector_number*bps + 15] << 8) |  flashPage[sector_number*bps + 16];
    VBat = 2.0f * 2.60f * 1.14f * ((float) ADCCounts) / 4095.0f;

    // Output for spreadsheet analysis
    if(Month < 10) {Serial.print("0"); Serial.print(Month);} else Serial.print(Month);
    Serial.print("/");Serial.print(Day); Serial.print("/");Serial.print(2000 + Year); Serial.print(" ");
    if(Hour < 10) {Serial.print("0"); Serial.print(Hour);} else Serial.print(Hour);
    Serial.print(":"); 
    if(Minute < 10) {Serial.print("0"); Serial.print(Minute);} else Serial.print(Minute); 
    Serial.print(":"); 
    if(Second < 10) {Serial.print("0"); Serial.print(Second);} else Serial.print(Second); Serial.print(",");      
    Serial.print(pressure, 2); Serial.print(","); Serial.print(HDCTemperature, 2); Serial.print(",");  
    Serial.print(HDCHumidity, 1); Serial.print(","); 
    Serial.print(((float)RGBiRData[0])*luxScale, 2); Serial.print(","); // R
    Serial.print(((float)RGBiRData[1])*luxScale, 2); Serial.print(","); // G
    Serial.print(((float)RGBiRData[2])*luxScale, 2); Serial.print(","); // B
    Serial.print(((float)RGBiRData[3])*luxScale, 2); Serial.print(","); // IR
    Serial.println(VBat, 2);  // Bat voltage  
    }
  
  }


}

void loop(void)
{
}
