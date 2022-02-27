Environmental Monitoring sketch

Demonstration of a simple webserver pushing data from BME280 (pressure, temperature, humidity) and CCS811 (eCO2 and TVOC) sensors to a local web server via wifi.

The ESP32 is an ESP32 C3 Mini module on a custom development board using the native USB for programming with the Arduino IDE and USB serial output.

![image](https://user-images.githubusercontent.com/6698410/155865582-daab5d08-0a00-4984-9684-b989d95954c0.jpg)

![ESP32Data](https://user-images.githubusercontent.com/6698410/155866112-22a74cbd-d34c-4b75-ac39-dd21a73026d8.jpg)

The web server update is based on a simple timer (here every 10 seconds). I heven't gotten deep sleep to work yet since the ExternalWakeUp example won't compile. But the idea is to manage the power usage of the ESP32 by keeping it in deep sleep mode either on a timer or using the CCS811 (with 60 second sample period) to wake up the ESP32 on data ready interrupt.

The [ESP32 C3 Mini](https://www.digikey.com/en/products/detail/espressif-systems/ESP32-C3-MINI-1-H4/14548936?utm_adgroup=&utm_source=bing&utm_medium=cpc&utm_campaign=Shopping_DK%2BSupplier_Other&utm_term=&utm_content=&utm_id=bi_cmp-384720322_adg-1301822093609990_ad-81363949567673_pla-4584963495352066_dev-c_ext-_prd-14548936) is an inexpensive, fairly small, easy-to-use module that embeds wifi and BLE, and has just enough GPIOs available to be useful for environmental end nodes. Being able to program it with native USB (no CP2102 or FTDI tranceiver!) is a real plus.

The development board [design](https://oshpark.com/shared_projects/wibSiWQn) is open source and available at the OSH Park shared space.
