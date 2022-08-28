//
// Created by 58286 on 2022/7/20.
//
#include "windmill_logic.h"

extern ADC_HandleTypeDef hadc1;
uint16_t windmill_counter = 0;

#define GetRand(max_lim) (rand()%max_lim)
extern uint8_t sensor;

typedef enum{
  USER_STATE = 0,
  //Small energy mechanism
  SEM_STATE = 1,
  //Macro energy mechanism
  MEM_STATE = 2
}WindmillState;

typedef struct{
  uint16_t random_seed;
//  SensorStruct sensor;
  uint8_t sensors;
  MotorState motor_state;
  WindmillState windmill_state;
  uint8_t impact_state;
  uint8_t last_impact_state;
  uint16_t impact_counter;
  uint8_t impact_complete_counter;
  uint8_t impact_order[5];
  uint8_t flow_start[5];
  uint16_t counter;
}WindmillStruct;

static WindmillStruct windmill;

void SetFanBladeState(uint8_t number,FanBladeState state)
{
  WS2812_SetState(number,(DisplayType)state);
  WS2812_SetState(number+5,(DisplayType)state);
  if((DisplayType)state == IMPACTED){
    windmill.flow_start[number] = 1;
  }else{
    windmill.flow_start[number] = 0;
  }
}

void WindmillInit(void)
{
  MotorInit();
  WS2812_Init();

  windmill.motor_state = MOTOR_STOP;
  windmill.windmill_state = SEM_STATE;
  windmill.impact_state = 0;
  windmill.last_impact_state = 1;
  windmill.impact_counter = 0;
  windmill.impact_complete_counter = 0;

  windmill.sensors = 0;
  memset(windmill.flow_start,0,5);

  for(uint8_t i=0;i<5;i++){
    SetFanBladeState(i,ALL_OFF);
  }
}

void WindmillUpdate(void)
{
  static uint8_t flow_counter = 0;
  MotorRun(windmill.motor_state,UPDATE_FREQ);
  flow_counter++;
  if(flow_counter == FLOW_PRESCALE){
    flow_counter = 0;
    WS2812_FlowUpdate();
    for(uint8_t i=0;i<5;i++){
      if(windmill.flow_start[i]){
        WS2812_SetState(i+5,(DisplayType)IMPACTED);
      }
    }
  }
//  SensorKeyScan();
//  WindmillLogic();
  if(windmill_counter) windmill_counter--;
  windmill.sensors = 0;
}