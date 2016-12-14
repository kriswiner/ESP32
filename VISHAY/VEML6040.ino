/* VEML6040 Basic Example Code
 by: Kris Winer
 date: December 14, 2016
 license: Beerware - Use this code however you'd like. If you 
 find it useful you can buy me a beer some time.
 
 Demonstrate basic VEML6040 functionality including parameterizing the register addresses, initializing the sensor, 
 getting properly scaled rgbw intensity data out. Sketch runs on the 3.3 V ESP32.
 
 From the data sheet: (https://www.vishay.com/docs/84276/veml6040.pdf)
 
 VEML6040 color sensor senses red, green, blue, and white light and incorporates photodiodes, amplifiers, 
 and analog / digital circuits into a single chip using CMOS process. With the   color   sensor   applied,   
 the   brightness,   and   color temperature of backlight can be adjusted base on ambient light  source  
 that  makes  panel  looks  more  comfortable  for  end   user’s   eyes.   VEML6040’s   adoption   of   FiltronTM
 technology  achieves  the  closest  ambient  light  spectral  sensitivity to real human eye responses.

 VEML6040  provides  excellent  temperature  compensation  capability  for  keeping  the  output  stable  
 under  changing  temperature.   VEML6040’s   function   are   easily   operated   via the simple command format 
 of I2C (SMBus compatible) interface  protocol.  VEML6040’s  operating  voltage  ranges  from   2.5   V   to   
 3.6   V.   VEML6040   is   packaged   in   a   lead  (Pb)-free  4  pin  OPLGA  package  which  offers  the  best market-proven reliability.

 SDA and SCL  have external pull-up resistors (to 3.3V).
 2K2 resistors are on the VEML6040 breakout board.
 
 Hardware setup:
 VEML6040 Breakout ------ ESP32
 VDD ---------------------- 3.3V or any digital pin i.e., digitalWrite(HIGH)
 SDA ----------------------- pin 21
 SCL ----------------------- pin 22
 GND ---------------------- GND or any digital pin i.e., digitalWrite(LOW)
 
*/
  
#include <Wire.h>

////////////////////////////
// VEML6040 Command Codes //
////////////////////////////
#define  VEML6040_CONF	          0x00 // command codes
#define  VEML6040_R_DATA   		    0x08  
#define  VEML6040_G_DATA		      0x09 
#define  VEML6040_B_DATA		      0x0A
#define  VEML6040_W_DATA          0x0B

#define VEML6040_ADDRESS          0x10

#define SerialDebug true  // set to true to get Serial output for debugging

// Pin definitions
int myLed  = 5;                      
uint16_t count = 0;

enum IT {
  IT_40 = 0, //   40 ms
  IT_80,     //   80 ms
  IT_160,    //  160 ms
  IT_320,    //  320 ms
  IT_640,    //  640 ms
  IT_1280   // 1280 ms
};

// Specify VEML6040 Integration time
uint8_t IT = IT_160;
uint8_t ITime = 160;  // milliseconds
uint16_t RGBWData[4] = {0, 0, 0, 0};
float GSensitivity = 0.25168/((float) (IT + 1)); // ambient light sensitivity increases with integration time


void setup()
{
  Wire.begin(21, 22, 400000); // (SDA, SCL) (21, 22) are default on ESP32, 400 kHz I2C bus speed
  delay(4000);
  Serial.begin(115200);
  
  // Set up the interrupt pin, its set as active high, push-pull
  pinMode(myLed, OUTPUT);
  digitalWrite(myLed, HIGH);

  I2Cscan();
  
  enableVEML6040(); // initalize sensor
 
  delay(150);
  
  digitalWrite(myLed, LOW);

}

void loop()
{  
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
  float CCT = 4278.6f*pow(temp, -1.2455) + 0.5f;

  Serial.print("Correlated Color Temperature = "); Serial.print(CCT, 2); Serial.println(" Kelvin");
  Serial.println("  ");


  digitalWrite(myLed, !digitalRead(myLed));
  delay(ITime+100);
}

//===================================================================================================================
//====== Set of useful function to access UV data
//===================================================================================================================

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
