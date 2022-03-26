Sketch showing how to use the ESP32C3Mini1 as an environmental logger. 
The CCS811 air quality sensor and BME280 humidity/pressure/temperature sensors on a breakout board are configured in setup once, 
their data is read and then the ESP32 goes into deep sleep for some time (10 minutes in the sketch). Upon wakeup, the sensor configuration is skipped if the bootcount variable is
greater than 1 and the data is read an the ESP32C3 goes into deep sleep, etc. The cycle repeats.

The idea is to log the data on the native SPIFFS (up to 3 MByte available) and send data updates via either BLE or ESPNow. These bits will be added when I assemble the next version which
has the sensors (in this case (APDS9253 ambient light sensor, HDC2010 humidity sensor, and LPS22HN barometer) directly on the ESP32C3Mini1 develoment board.

I measured deep sleep current with no sensors of 8.9 uA. However, this is with a simple 1 MOhm+1 MOhm voltage divider, so at least 2 uA come from this. Another 300 nA comes from the MCP1812 LDO.
The data sheet specs 5 uA as typical, so 8.9 - ~2 - 0.3 is ~6.6 uA is in the ball park. The average current when the ESP32C3Mini is awake is ~18 mA for ~0.25 seconds; this is without
BLE or ESPNow. So I expect running at a 10 minute data update duty cycle will use ~15 uA. Probably a bit more with BLE or ESPNow. In any case, I expect to be able to run this kind of
environmental data logger for at least six months on a small (~100 mAH) LiPo battery.  
