//
// Created by 58286 on 2022/7/19.
//
#include "sensor.h"

SensorStruct *sensor;

void SensorInit(SensorStruct* _sensor)
{
  for(uint8_t i=0;i<5;i++){
    _sensor->sensors[i] = 0;
  }
  sensor = _sensor;
}

void SensorKeyScan(void)
{
  static uint16_t sensor_counter[5] = {0,0,0,0,0}; 
  
  if(HAL_GPIO_ReadPin(Sensor1_GPIO_Port,Sensor1_Pin) == GPIO_PIN_RESET){
    sensor_counter[0]++;
    if(sensor_counter[0] == SENSOR_DELAY_TIME){
      sensor->sensors[0] = 1;
      sensor_counter[0] = 0;
    }
  }else{
    sensor_counter[0] = 0;
  }

  if(HAL_GPIO_ReadPin(Sensor2_GPIO_Port,Sensor2_Pin) == GPIO_PIN_RESET){
    sensor_counter[1]++;
    if(sensor_counter[1] == SENSOR_DELAY_TIME){
      sensor->sensors[1] = 1;
      sensor_counter[1] = 0;
    }
  }else {
    sensor_counter[1] = 0;
  }

  if(HAL_GPIO_ReadPin(Sensor3_GPIO_Port,Sensor3_Pin) == GPIO_PIN_RESET){
    sensor_counter[2]++;
    if(sensor_counter[2] == SENSOR_DELAY_TIME){
      sensor->sensors[2] = 1;
      sensor_counter[2] = 0;
    }
  }else {
    sensor_counter[2] = 0;
  }

  if(HAL_GPIO_ReadPin(Sensor4_GPIO_Port,Sensor4_Pin) == GPIO_PIN_RESET){
    sensor_counter[3]++;
    if(sensor_counter[3] == SENSOR_DELAY_TIME){
      sensor->sensors[3] = 1;
      sensor_counter[3] = 0;
    }
  }else {
    sensor_counter[3] = 0;
  }

  if(HAL_GPIO_ReadPin(Sensor5_GPIO_Port,Sensor5_Pin) == GPIO_PIN_RESET){
    HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,GPIO_PIN_RESET);
    sensor_counter[4]++;
    if(sensor_counter[4] == SENSOR_DELAY_TIME){
      sensor->sensors[4] = 1;
      sensor_counter[4] = 0;
    }
  }else {
    sensor_counter[4] = 0;
  }
}