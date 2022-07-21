//
// Created by 58286 on 2022/7/20.
//

#ifndef WINDMILL_FUNCTION_WINDMILL_LOGIC_H_
#define WINDMILL_FUNCTION_WINDMILL_LOGIC_H_
#include "main.h"
#include "user_config.h"
#include "motor.h"
#include "sensor.h"
#include "WS2812.h"
#include "stdlib.h"

#define FLOW_PRESCALE 160

typedef enum{
  ALL_ON=1,
  ALL_OFF=2,
  IMPACTED=3
}FanBladeState;

void WindmillInit(void);
void WindmillUpdate(void);

#endif // WINDMILL_FUNCTION_WINDMILL_LOGIC_H_
