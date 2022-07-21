//
// Created by 58286 on 2022/7/19.
//

#ifndef WINDMILL_FUNCTION_SENSOR_H_
#define WINDMILL_FUNCTION_SENSOR_H_
#include "main.h"
#include "user_config.h"

#define SENSOR_DELAY_TIME 20

typedef struct {
  union{
    struct {
      uint8_t sensor1;
      uint8_t sensor2;
      uint8_t sensor3;
      uint8_t sensor4;
      uint8_t sensor5;
    };
    uint8_t sensors[5];
  };
}SensorStruct;

void SensorInit(SensorStruct* _sensor);
void SensorKeyScan(void);

#endif // WINDMILL_FUNCTION_SENSOR_H_
