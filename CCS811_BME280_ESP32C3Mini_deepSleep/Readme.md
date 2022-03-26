Sketch showing how to use the ESP32C3Mini1 as an environmental data logger. 

I am using a custom ESP32C3Mini1 development board with native USB for programming (no CP2102).

The CCS811 air quality sensor and BME280 humidity/pressure/temperature sensors are on a breakout board and are configured in setup once, 
their data is read and then the ESP32 goes into deep sleep for some time (10 minutes in the sketch). Upon wakeup, the sensor configuration is skipped if the bootcount variable is greater than 1 and the data is read and the ESP32C3 goes into deep sleep, etc. The cycle repeats.

![image](https://user-images.githubusercontent.com/6698410/155865582-daab5d08-0a00-4984-9684-b989d95954c0.jpg)

The idea is to log the data on the native SPIFFS (up to 3 MByte available) and send data updates via either BLE or ESPNow. These bits will be added when I assemble the next version which has the sensors (in this case APDS9253 ambient light sensor, HDC2010 humidity sensor, and LPS22HB barometer) directly on the ESP32C3Mini1 development board.

I measured deep sleep current with no sensors of 8.9 uA. However, this is with a simple 1 MOhm+1 MOhm voltage divider, so at least 2 uA come from this. In the new verson I added a dual FET to eliminate this current draw. Another 300 nA comes from the MCP1812 LDO. The data sheet specs 5 uA as typical, so 8.9 - ~2 - 0.3 is ~6.6 uA is in the ball park. The average current when the ESP32C3Mini is awake is ~18 mA for ~0.25 seconds; this is without BLE or ESPNow. So I expect running at a 10 minute data update duty cycle will use ~15 uA. Probably a bit more with BLE or ESPNow. In any case, I expect to be able to run this kind of environmental data logger for at least six months on a small (~100 mAH) LiPo battery.  

One oddity is that if I enable USBSerial output by setting SerialDebug = true in every file where USBSerial is called then the sketch works well, sending serial output to the serial monitor. However, upon wake from deep sleep the program stalls unless the serial monitor is closed then opened again. This happens even when running from battery with no USB connected. So in order to run this sketch as a data logger, one has to set SerialDebug to false everywhere then it works just fine.
