Description of ESP32C3Mini-hosted environmental data logger

The idea is to use the remarkably [cheap](https://www.digikey.com/en/products/detail/espressif-systems/ESP32-C3-MINI-1-N4/138775740) ESP32C3Mini module (ESP32-C3-Mini-1-N4) as host for an environmental data logger that can last for months on a small battery. 

The set of I2C sensors:

APDS9253 RGBiR light sensor
HDC2010 humidity and temperature sensor
LPS22HB barometer

Rather than use the internal ~3 MByte SPIFFS, which uses a lot of power, I selected an external, ultra-low-power (100 nA in deep power down mode) 8 MByte MX25R6435FZAI NOR flash memory to log the data. 

In addition to four-color lux, humidity, temperature, pressure measurements, I added a resistor divider and dual FET to sample the battery voltage via an ESP32C3 ADC, user/boot and reset buttons, and an led for indication. 

The board main power rail is 3V3 from an MCP1812 LDO whose Iq is just 300 nA. I took great pains to minimize the power usage which is dominated by the ESP32Mini module itself, being 5.6 uA in deep sleep mode. Everything else adds a small amount to this (TBD).

I am using Wifi to connect to the NTP server to initally sync the time peripheral then disconnecting Wifi. The idea is to log human readable time (sec, min, hour, day, month, year) along with the sensor data so over long periods of logging the sensor data can be correlated to real time. One could drop year and second to reduce the memory burden especially over short logging sessions since logging at duty cycles where sensor variations meaningfully reflect environmental changes, say every ten minutes, means the seconds are always the same. Shot sessions means the year is unlikely to change.

So far I am logging 23 bytes of data for each logging event so I can log 11 events (253 bytes) before I write a 256-byte page to flash. This is efficient enough but YMMV.

So far I have the basic sketch working to configure the sensors and flash, and then peridically read sensor data, store it in an arrat and then write a full page to external flash. In each case but the baro, the sensors/flash is kept in its lowest power state until needed. In the case of the sensors, this means once every five or ten minutes. For the flash, this means once wvery 55 or 110 minutes. It took a while to get this all working properly because of some of the quirks of the ESP32C3Mini. One big issue was the USB serial (I am using the native USB not a USB-to-Serial transceiver). Turns out with WiFi this is automaticlly disconnected and  a boolean flag has to be set to make sure this is turned on again.

![ESP32C3Mini top](https://user-images.githubusercontent.com/6698410/166591280-3111662b-efe1-49bb-904c-abd950bf572f.jpg)
![ESP32C3Mini_bottom](https://user-images.githubusercontent.com/6698410/166591298-9c89f85a-87d2-4b78-b5d7-5e32c969c563.jpg)

Project pcb EAGLE CAD design files available in the OSH Park shared space [here.](https://oshpark.com/shared_projects/6YSyYfg9)
