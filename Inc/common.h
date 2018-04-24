#ifndef __COMMON_H
#define __COMMON_H

#include "stm32f7xx_hal.h"

#define TYPE_MPU6050        0x00
#define TYPE_ADXL345        0x01
#define TYPE_H3LIS100DL     0x02

typedef struct
{
    uint8_t id;     // type:4 + id:4
    uint8_t addr;
    char* name;
    uint8_t enabled;
    uint8_t err;
    int16_t rawData[3];
}SensorType;


#endif
