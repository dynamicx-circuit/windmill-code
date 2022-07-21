//
// Created by 58286 on 2022/7/20.
//
#include "windmill_logic.h"

extern ADC_HandleTypeDef hadc1;

#define GetRand(max_lim) (rand()%max_lim)

typedef enum{
  USER_STATE = 0,
  //Small energy mechanism
  SEM_STATE = 1,
  //Macro energy mechanism
  MEM_STATE = 2
}WindmillState;

typedef struct{
  uint16_t random_seed;
  SensorStruct sensor;
  MotorState motor_state;
  WindmillState windmill_state;
  uint16_t impact_counter;
  uint8_t impact_complete_counter;
  uint8_t impact_order[5];
}WindmillStruct;

static WindmillStruct windmill;

inline void SetFanBladeState(uint8_t number,FanBladeState state)
{
  WS2812_SetState(number,(DisplayType)state);
  WS2812_SetState(number+5,(DisplayType)state);
}

void WindmillLogic(void)
{
  windmill.motor_state = (MotorState)windmill.windmill_state;
  if(windmill.windmill_state == SEM_STATE || windmill.windmill_state == MEM_STATE){
    if(windmill.impact_complete_counter == 5) {
      static uint8_t shining_counter = 0;
      static uint16_t counter = 0;
      counter++;
      if(counter == 400){
        for(uint8_t i=0;i<5;i++){
          SetFanBladeState(i,ALL_ON);
        }
      }else if(counter == 800){
        for(uint8_t i=0;i<5;i++){
          SetFanBladeState(i,ALL_OFF);
        }
        counter = 0;
        shining_counter++;
      }
      if(shining_counter == 3){
        counter = 0;
        shining_counter = 0;
        windmill.windmill_state = USER_STATE;
        windmill.impact_complete_counter = 0;
        windmill.impact_counter = 0;
      }
    }else {
      if(windmill.impact_counter == 0){
        SetFanBladeState(windmill.impact_order[windmill.impact_complete_counter],IMPACTED);
      }
      windmill.impact_counter++;
      if(windmill.impact_counter == (uint16_t)(2.5f*UPDATE_FREQ)){
        windmill.impact_counter=0;
        for(uint8_t i=0;i<=windmill.impact_complete_counter;i++){
          SetFanBladeState(windmill.impact_order[i],ALL_OFF);
        }
        windmill.windmill_state = USER_STATE;
      }
      for(uint8_t i=0;i<5;i++){
        if(windmill.sensor.sensors[i]){
          if(i==windmill.impact_order[windmill.impact_complete_counter])
            windmill.impact_counter=0;
          windmill.impact_complete_counter++;
          SetFanBladeState(windmill.impact_order[i],ALL_ON);
          break;
        }else{
          windmill.impact_counter=0;
          for(uint8_t j=0;j<=windmill.impact_complete_counter;j++){
            SetFanBladeState(windmill.impact_order[j],ALL_OFF);
          }
          windmill.windmill_state = USER_STATE;
        }
      }
    }
  }else if(windmill.windmill_state == USER_STATE){
    uint8_t i=0;
    uint8_t seleted = 0;
    while(i<5){
      uint8_t temp = GetRand(5);
      if((seleted|(1<<temp)) != seleted){
        seleted |= (1<<temp);
        windmill.impact_order[i] = temp;
        i++;
      }
    }
    windmill.windmill_state = SEM_STATE;
    windmill.impact_complete_counter = 0;
    windmill.impact_counter = 0;
  }
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
  windmill.windmill_state = SEM_STATE;
  windmill.impact_counter = 0;
  windmill.impact_complete_counter = 0;
  memset(windmill.sensor.sensors,0,5);

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
  }
  SensorKeyScan();
  WindmillLogic();
  memset(windmill.sensor.sensors,0,5);
}