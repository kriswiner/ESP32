/******************************************************************************
 *
 * Copyright (c) 2021 Tlera Corporation  All rights reserved.
 *
 * This library is open-source and freely available for all to use with attribution.
 * 
 * All rights reserved.
 *****************************************************************************
 */
#include "APDS9253.h"
#include "I2Cdev.h"


APDS9253::APDS9253(I2Cdev* i2c_bus)
{
  _i2c_bus = i2c_bus;
}

// set bit 4 to 1 for software reset
void APDS9253::reset()
{
   uint8_t temp = _i2c_bus->readByte(APDS9253_ADDR, APDS9253_MAIN_CTRL);
   _i2c_bus->writeByte(APDS9253_ADDR, APDS9253_MAIN_CTRL, temp | 0x10); 
}

 
uint8_t APDS9253::getChipID()
{
   uint8_t temp = _i2c_bus->readByte(APDS9253_ADDR, APDS9253_PART_ID);
   return temp;
}


void APDS9253::init(uint8_t RGBmode, uint8_t LS_res, uint8_t LS_rate, uint8_t LS_gain)
{
   _i2c_bus->writeByte(APDS9253_ADDR, APDS9253_MAIN_CTRL, RGBmode << 2); 
   _i2c_bus->writeByte(APDS9253_ADDR, APDS9253_LS_MEAS_RATE, (LS_res << 4) | LS_rate); 
   _i2c_bus->writeByte(APDS9253_ADDR, APDS9253_LS_GAIN, LS_gain); 

   // configure the interrupt
   // enable interrupt on ALS (green) threshold
   _i2c_bus->writeByte(APDS9253_ADDR, APDS9253_INT_CFG, 0x14); 
   // set persistence to 2 consecutive ALS values below threshold to trigger
   _i2c_bus->writeByte(APDS9253_ADDR, APDS9253_INT_PST, 0x10); 

   // set lower threshold
   _i2c_bus->writeByte(APDS9253_ADDR, APDS9253_LS_THRES_LOW_0, 0x0A); // low threshold 10 counts
   _i2c_bus->writeByte(APDS9253_ADDR, APDS9253_LS_THRES_LOW_1, 0x00); 
   _i2c_bus->writeByte(APDS9253_ADDR, APDS9253_LS_THRES_LOW_2, 0x00); 
}


void APDS9253::enable()
{
   uint8_t temp = _i2c_bus->readByte(APDS9253_ADDR, APDS9253_MAIN_CTRL);
   _i2c_bus->writeByte(APDS9253_ADDR, APDS9253_MAIN_CTRL, temp & ~(0x02) ); // clear LS_EN bit
   _i2c_bus->writeByte(APDS9253_ADDR, APDS9253_MAIN_CTRL, temp | 0x02 );    // set LS_EN bit
}


void APDS9253::disable()
{
   uint8_t temp = _i2c_bus->readByte(APDS9253_ADDR, APDS9253_MAIN_CTRL);
   _i2c_bus->writeByte(APDS9253_ADDR, APDS9253_MAIN_CTRL, temp & ~(0x02) ); // clear LS_EN bit
}


void APDS9253::getRGBiRdata(uint32_t * destination)
{
  uint8_t rawData[3] = {0, 0, 0};
     _i2c_bus->readBytes(APDS9253_ADDR, APDS9253_LS_DATA_IR_0, 3, &rawData[0]);  
     destination[3] = (((uint32_t) (rawData[3] & 0x0F)) << 16)  |  (((uint32_t) rawData[1]) << 8)  | ((uint32_t) rawData[0]);    // ir
     _i2c_bus->readBytes(APDS9253_ADDR, APDS9253_LS_DATA_BLUE_0, 3, &rawData[0]);  
     destination[2] = (((uint32_t) (rawData[3] & 0x0F)) << 16)  |  (((uint32_t) rawData[1]) << 8)  | ((uint32_t) rawData[0]);    // blue
     _i2c_bus->readBytes(APDS9253_ADDR, APDS9253_LS_DATA_GREEN_0, 3, &rawData[0]);  
     destination[1] = (((uint32_t) (rawData[3] & 0x0F)) << 16)  |  (((uint32_t) rawData[1]) << 8)  | ((uint32_t) rawData[0]);    // green
     _i2c_bus->readBytes(APDS9253_ADDR, APDS9253_LS_DATA_RED_0, 3, &rawData[0]);  
     destination[0] = (((uint32_t) (rawData[3] & 0x0F)) << 16)  |  (((uint32_t) rawData[1]) << 8)  | ((uint32_t) rawData[0]);    // red
}


uint8_t APDS9253::getStatus()
{
    uint8_t Status = _i2c_bus->readByte(APDS9253_ADDR, APDS9253_MAIN_STATUS);
    return Status;
}
