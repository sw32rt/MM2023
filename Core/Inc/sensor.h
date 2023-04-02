#ifndef SENSOR_H_
#define SENSOR_H_
#include "stdint.h"

typedef struct sensor
{
    int16_t acc_x;
    int16_t acc_y;
    int16_t acc_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
    uint16_t enc_r;
    uint16_t enc_l;
    uint16_t range_f;
    uint16_t range_r;
    uint16_t range_l;
}SensorValues;

extern SensorValues g_sensor;


#endif /* SENSOR_H_ */
