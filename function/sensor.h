//
// Created by 58286 on 2022/7/19.
//

#ifndef WINDMILL_FUNCTION_SENSOR_H_
#define WINDMILL_FUNCTION_SENSOR_H_
#include "main.h"

#define SENSOR1_GPIO_PIN GPIO_PIN_10
#define SENSOR2_GPIO_PIN GPIO_PIN_12
#define SENSOR3_GPIO_PIN GPIO_PIN_4
#define SENSOR4_GPIO_PIN GPIO_PIN_7
#define SENSOR5_GPIO_PIN GPIO_PIN_11

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

#endif // WINDMILL_FUNCTION_SENSOR_H_
