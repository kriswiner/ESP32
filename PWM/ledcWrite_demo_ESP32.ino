/*
  ledcWrite_demo_ESP32.ino
  Runs through the full 255 color spectrum for an rgb led 
  Demonstrate ledcWrite functionality for driving leds with PWM on ESP32
 
  This example code is in the public domain.
 */
 
// Set up the rgb led names
uint8_t ledR = 12;
uint8_t ledG = 14; // internally pulled up
uint8_t ledB = 13; 

uint8_t myLed = 5;  // on-board blue led (also internally pulled up)

uint8_t ledArray[3] = {1, 2, 3}; // three led channels

const boolean invert = false; // set true if common anode, false if common cathode

uint8_t color = 0;          // a value from 0 to 255 representing the hue
uint32_t R, G, B;           // the Red Green and Blue color components
uint8_t brightness = 255;  // 255 is maximum brightness, but can be changed

// the setup routine runs once when you press reset:
void setup() 
{             
  pinMode(myLed, OUTPUT); 
  digitalWrite(myLed5, LOW);// Turn off on-board blue led
  
  ledcAttachPin(ledR, 1); // assign RGB led pins to channels
  ledcAttachPin(ledG, 2);
  ledcAttachPin(ledB, 3);
  
  // Initialize channels 
  // channels 0-15, resolution 1-16 bits, freq limits depend on resolution
  // ledcSetup(uint8_t channel, uint32_t freq, uint8_t resolution_bits);
  ledcSetup(1, 12000, 8); // 12 kHz PWM, 8-bit resolution
  ledcSetup(2, 12000, 8);
  ledcSetup(3, 12000, 8);

 for(uint8_t i=0; i < 3; i++) {
  // ledcWrite(channel, dutycycle)
  // For 8-bit resolution duty cycle is 0 - 255
  ledcWrite(ledArray[i], 255);  // test high output of all leds in sequence
  delay(1000);
  ledcWrite(ledArray[i], 0);
 }
 
}

// void loop runs over and over again
void loop() 
{
 for (color = 0; color < 255; color++) { // Slew through the color spectrum

  hueToRGB(color, brightness);  // call function to convert hue to RGB

  // write the RGB values to the pins
  ledcWrite(1, R); // write red component to channel 1, etc.
  ledcWrite(2, G);   
  ledcWrite(3, B); 
 
  delay(100); // full cycle of rgb over 256 colors takes 26 seconds
 }
 
 digitalWrite(myLed, HIGH); delay(10); digitalWrite(myLed, LOW); // indicate end of cycle
}


// Courtesy http://www.instructables.com/id/How-to-Use-an-RGB-LED/?ALLSTEPS
// function to convert a color to its Red, Green, and Blue components.

void hueToRGB(uint8_t hue, uint8_t brightness)
{
    uint16_t scaledHue = (hue * 6);
    uint8_t segment = scaledHue / 256; // segment 0 to 5 around the
                                            // color wheel
    uint16_t segmentOffset =
      scaledHue - (segment * 256); // position within the segment

    uint8_t complement = 0;
    uint16_t prev = (brightness * ( 255 -  segmentOffset)) / 256;
    uint16_t next = (brightness *  segmentOffset) / 256;

    if(invert)
    {
      brightness = 255 - brightness;
      complement = 255;
      prev = 255 - prev;
      next = 255 - next;
    }

    switch(segment ) {
    case 0:      // red
        R = brightness;
        G = next;
        B = complement;
    break;
    case 1:     // yellow
        R = prev;
        G = brightness;
        B = complement;
    break;
    case 2:     // green
        R = complement;
        G = brightness;
        B = next;
    break;
    case 3:    // cyan
        R = complement;
        G = prev;
        B = brightness;
    break;
    case 4:    // blue
        R = next;
        G = complement;
        B = brightness;
    break;
   case 5:      // magenta
    default:
        R = brightness;
        G = complement;
        B = prev;
    break;
    }
}
