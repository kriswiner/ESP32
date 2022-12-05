**ESP32C3Mini-hosted environmental data logger**

The idea is to use the remarkably [cheap](https://www.digikey.com/en/products/detail/espressif-systems/ESP32-C3-MINI-1-N4/13877574) ESP32C3Mini module (ESP32-C3-Mini-1-N4) as host for a small (25.6 x 20.5 mm) environmental data logger that can run for months on a small battery. 

The set of I2C sensors includes the APDS9253 RGBiR light sensor, HDC2010 humidity and temperature sensor, and LPS22HB barometer.

Rather than use the internal ~3 MByte SPIFFS, which uses a lot of power, I selected an external, ultra-low-power (100 nA in deep power down mode) 8 MByte MX25R6435FZAI SPI NOR flash memory to log the data. 

In addition to four-color lux, humidity, temperature, and pressure measurements, I added a resistor divider and dual FET to sample the battery voltage via an ESP32C3 ADC. The board also has user/boot and reset buttons, and an led for indication. 

The board main power rail is 3V3 from an MCP1812 LDO whose Iq is just 300 nA. I took pains to minimize the power usage which is dominated by the ESP32Mini module itself, being 5.6 uA in deep sleep mode. Everything else adds a small amount (~few uA, TBD) to this.

I am using Wifi to connect to the NTP server to initially sync the time peripheral then disconnecting Wifi. The idea is to log human readable time (sec, min, hour, day, month, year) along with the sensor data so over long periods of logging the sensor data can be correlated to real time. One could drop year and second to reduce the memory burden especially over short logging sessions since logging at duty cycles where sensor variations meaningfully reflect environmental changes, say every ten minutes, means the seconds are always the same. Short sessions means the year is unlikely to change.

The ESP32C3Mini cannot be woken from deep sleep by GPIO, i.e., by sensor interrupt, for example, which is a serious deficiency. So everything has to be based on timers and the only low power tool for the module is timed deep sleep. If the module could be woken by GPIO then recording the time (including seconds) of an event would make sense. For timed deep sleep only, minute resolution is probably good enough.

So far I am logging 23 bytes of data for each logging event. So I can log 11 events (253 bytes) before I write a 256-byte page to flash. This is efficient enough for me but YMMV.

I have two utility sketches in addition to the main logging sketch. SPIFlash tests the flash and ends up erasing it. This is useful for initial assembly tests as well as erasing the flash for the next logging session. The readSPIFlash sketch reads the  bytes stored on the flash during a data logging session and reconstructs the properly-scaled and formatted data and prints it to the serial monitor as comma-delimited lines (one data log per line) for subsequent plotting in a spreadsheet like Excel or OpenOffice (see below for an example).

So far I have the basic sketch working to configure the sensors and flash, and then peridically read sensor data, store it in an page array and then write a full page to external flash. In each case but the baro, the sensors/flash is kept in its lowest power state until needed. In the case of the sensors, this means once every five or ten minutes. For the flash, this means once every 55 or 110 minutes. I am running the baro continuously at 1 Hz since my attempts to use the forced mode have resulted in poor data quality from the sensor. I think it has to run a few cycles to settle so forced or one-shot mode for this sensor isn't a good option. I might replace it with the more accurate [ILPS22QS](https://github.com/kriswiner/ILPS22QS) baro which does work in one-shot mode in a subsequent redesign.

It took a while to get this all working properly because of some of the quirks of the ESP32C3Mini. One big issue was the USB serial (I am using the native USB not a USB-to-Serial transceiver). Turns out with WiFi this is automatically disconnected and a boolean flag has to be set to make sure this is turned on again. The other difficulty I had was selecting the SPI Flash clock speed. I settled on 10 MHz, which produced the most reliable results. It seemed to work at 20 and 40 MHz but in testing I had intermittent failures to record some or all of the data, and once the data was recorded but the time/date was mangled. The data never wrote to the flash at 80 MHz, the speed at which the internal flash usually operates. So the MX25R6435 external SPI flash or the ESP32C3 SPI peripheral or both might be a little flaky. Could also be pilot error. However I have been using the MX25R6435FZAI in STM32L0-hosted asset tracking applications running at 50 MHz SPI clock speed with no trouble for years. I will continue testing to assess logging reliability in real-world logging applications.

I don't expect perfection in a $2 module, but so far the ESP32C3Mini has worked well enough to keep me interested in continuing with the project.

Lastly, some data from an overnight logging test run:

![ESP32C3MiniEnvLogTest3](https://user-images.githubusercontent.com/6698410/166608157-96e9a205-15b8-46f6-a29f-296c916ab96c.jpg)

You can see the temperature spike from the initial Wifi time sync drop rapidly at the beginning. You can also see sunset at ~7:55 pm. The 105 mAH freshly-charged LiPo only lasted 7.3 hours before conking out, so an average current usage of 14 mA at 80 MHz clock speed and no deep sleep. Implementing the latter is the next task...

![ESP32C3Mini top](https://user-images.githubusercontent.com/6698410/166591280-3111662b-efe1-49bb-904c-abd950bf572f.jpg)
![ESP32C3Mini_bottom](https://user-images.githubusercontent.com/6698410/166591298-9c89f85a-87d2-4b78-b5d7-5e32c969c563.jpg)

Project pcb EAGLE CAD design files available in the OSH Park shared space [here.](https://oshpark.com/shared_projects/6YSyYfg9)
