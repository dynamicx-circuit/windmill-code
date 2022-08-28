//
// Created by 58286 on 2022/7/19.
//
#include "sensor.h"

//static uint8_t *sensor;
//
//void SensorInit(uint8_t * _sensor)
//{
//  *_sensor = 0;
//  sensor = _sensor;
//}
//
//void SensorKeyScan(void)
//{
//  static uint16_t sensor_counter[5] = {0,0,0,0,0};
//
//  if(HAL_GPIO_ReadPin(Sensor1_GPIO_Port,Sensor1_Pin) == GPIO_PIN_SET){
//    sensor_counter[0]++;
//    if(sensor_counter[0] == SENSOR_DELAY_TIME){
//      *sensor|=1;
//      sensor_counter[0] = 0;
//    }
//  }else{
//    sensor_counter[0] = 0;
//  }
//
//  if(HAL_GPIO_ReadPin(Sensor2_GPIO_Port,Sensor2_Pin) == GPIO_PIN_SET){
//    sensor_counter[1]++;
//    if(sensor_counter[1] == SENSOR_DELAY_TIME){
//      *sensor|=2;
//      sensor_counter[1] = 0;
//    }
//  }else {
//    sensor_counter[1] = 0;
//  }
//
//  if(HAL_GPIO_ReadPin(Sensor3_GPIO_Port,Sensor3_Pin) == GPIO_PIN_SET){
//    sensor_counter[2]++;
//    if(sensor_counter[2] == SENSOR_DELAY_TIME){
//      *sensor|=4;
//      sensor_counter[2] = 0;
//    }
//  }else {
//    sensor_counter[2] = 0;
//  }
//
//  if(HAL_GPIO_ReadPin(Sensor4_GPIO_Port,Sensor4_Pin) == GPIO_PIN_SET){
//    sensor_counter[3]++;
//    if(sensor_counter[3] == SENSOR_DELAY_TIME){
//      *sensor|=8;
//      sensor_counter[3] = 0;
//    }
//  }else {
//    sensor_counter[3] = 0;
//  }
//
//  if(HAL_GPIO_ReadPin(Sensor5_GPIO_Port,Sensor5_Pin) == GPIO_PIN_SET){
//    sensor_counter[4]++;
//    if(sensor_counter[4] == SENSOR_DELAY_TIME){
//      *sensor|=16;
//      sensor_counter[4] = 0;
//    }
//  }else {
//    sensor_counter[4] = 0;
//  }
//}