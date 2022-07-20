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
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == SENSOR1_GPIO_PIN){

  }else if(GPIO_Pin == SENSOR2_GPIO_PIN){
//    HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
  }
}