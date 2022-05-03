/******************************************************************************
 *
 * Copyright (c) 2021 Tlera Corporation  All rights reserved.
 *
 * This library is open-source and freely available for all to use with attribution.
 * 
 * All rights reserved.
 *****************************************************************************
 */

#ifndef APDS9253_h
#define APDS9253_h

#include "Arduino.h"
#include <Wire.h>
#include "I2Cdev.h"

// Register Map
#define APDS9253_MAIN_CTRL       (0x00)
#define APDS9253_LS_MEAS_RATE    (0x04)
#define APDS9253_LS_GAIN         (0x05)
#define APDS9253_PART_ID         (0x06)
#define APDS9253_MAIN_STATUS     (0x07)
#define APDS9253_LS_DATA_IR_0    (0x0A)
#define APDS9253_LS_DATA_IR_1    (0x0B)
#define APDS9253_LS_DATA_IR_2    (0x0C)
#define APDS9253_LS_DATA_GREEN_0 (0x0D)
#define APDS9253_LS_DATA_GREEN_1 (0x0E)
#define APDS9253_LS_DATA_GREEN_2 (0x0F)
#define APDS9253_LS_DATA_BLUE_0  (0x10)
#define APDS9253_LS_DATA_BLUE_1  (0x11)
#define APDS9253_LS_DATA_BLUE_2  (0x12)
#define APDS9253_LS_DATA_RED_0   (0x13)
#define APDS9253_LS_DATA_RED_1   (0x14)
#define APDS9253_LS_DATA_RED_2   (0x15)
#define APDS9253_INT_CFG         (0x19)
#define APDS9253_INT_PST         (0x1A)
#define APDS9253_LS_THRES_UP_0   (0x21)
#define APDS9253_LS_THRES_UP_1   (0x22)
#define APDS9253_LS_THRES_UP_2   (0x23)
#define APDS9253_LS_THRES_LOW_0  (0x24)
#define APDS9253_LS_THRES_LOW_1  (0x25)
#define APDS9253_LS_THRES_LOW_2  (0x26)
#define APDS9253_LS_THRES_VAR    (0x27)
#define APDS9253_DK_CNT_STOR     (0x29)

#define APDS9253_ADDR      0x52  // I2C address

#define ALSandIR 0x00
#define RGBiR    0x01

//LS_res
#define res20bit 0x00 // 400 ms
#define res19bit 0x01 // 200 ms
#define res18bit 0x02 // 100 ms default
#define res17bit 0x03 // 50 ms
#define res16bit 0x04 // 25 ms
//#define res13bit 0x05 // 3.125 ms

//LS_rate
#define rate40Hz 0x00 // 25 ms
#define rate20Hz 0x01
#define rate10Hz 0x02 // 100 ms default
#define rate5Hz 0x03
#define rate2_5Hz 0x04
#define rate1Hz 0x05
#define rate0_5Hz 0x06 // 2000 ms

// LS_gain
#define gain1 0x00
#define gain3 0x01
#define gain6 0x02
#define gain9 0x03
#define gain18 0x04


/* general accel methods */
class APDS9253{
 public:
    APDS9253(I2Cdev* i2c_bus);
    void reset();
    uint8_t getStatus();
    uint8_t getChipID();
    void init(uint8_t RGBmode, uint8_t LS_res, uint8_t LS_rate, uint8_t LS_gain);
    void enable();
    void disable();
    void getRGBiRdata(uint32_t * destination);
 private:
    I2Cdev* _i2c_bus;
};
#endif
