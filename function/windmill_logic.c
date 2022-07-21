//
// Created by 58286 on 2022/7/20.
//
#include "windmill_logic.h"

extern ADC_HandleTypeDef hadc1;

#define GetRand(max_lim) (rand()%max_lim)

typedef struct{
  uint16_t random_seed;
  SensorStruct sensor;
  MotorState motor_state;
}WindmillStruct;

static WindmillStruct windmill;

inline void SetFanBladeState(uint8_t number,FanBladeState state)
{
  WS2812_SetState(number,(DisplayType)state);
  WS2812_SetState(number+5,(DisplayType)state);
}

void WindmillLogic(void)
{

}

void WindmillInit(void)
{
  MotorInit();
  WS2812_Init();
  SensorInit(&windmill.sensor);

  HAL_ADC_Start(&hadc1);
  HAL_Delay(1);
  windmill.random_seed = HAL_ADC_GetValue(&hadc1);
  HAL_ADC_Stop(&hadc1);
  srand(windmill.random_seed);

  windmill.motor_state = MOTOR_STOP;
  memset(windmill.sensor.sensors,0,5);
}

void WindmillUpdate(void)
{
  static uint8_t flow_counter = 0;
  MotorRun(windmill.motor_state,UPDATE_FREQ);

  flow_counter++;
  if(flow_counter == FLOW_PRESCALE){
    flow_counter = 0;
    WS2812_FlowUpdate();
  }
  SensorKeyScan();
  WindmillLogic();
}