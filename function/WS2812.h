//
// Created by 58286 on 2022/7/17.
//

#ifndef WINDMILL_FUNCTION_WS2812_H_
#define WINDMILL_FUNCTION_WS2812_H_
#include "main.h"
#include "tim.h"

#define STRIP_LENGTH 110
#define BOT_LENGTH 63
#define TOP_LENGTH STRIP_LENGTH-BOT_LENGTH
#define BOARD_LENGTH 40*8

#define STRIP1_TIM &htim1
#define STRIP2_TIM &htim2
#define STRIP3_TIM &htim2
#define STRIP4_TIM &htim1
#define STRIP5_TIM &htim2

#define STRIP1_CHANNEL TIM_CHANNEL_3
#define STRIP2_CHANNEL TIM_CHANNEL_1
#define STRIP3_CHANNEL TIM_CHANNEL_2
#define STRIP4_CHANNEL TIM_CHANNEL_1
#define STRIP5_CHANNEL TIM_CHANNEL_3

#define BOARD1_TIM &htim5
#define BOARD2_TIM &htim5
#define BOARD3_TIM &htim5
#define BOARD4_TIM &htim5
#define BOARD5_TIM &htim8

#define BOARD1_CHANNEL TIM_CHANNEL_3
#define BOARD2_CHANNEL TIM_CHANNEL_2
#define BOARD3_CHANNEL TIM_CHANNEL_1
#define BOARD4_CHANNEL TIM_CHANNEL_4
#define BOARD5_CHANNEL TIM_CHANNEL_1

#define WS2812_H 52
#define WS2812_L 21

#define WS2812_OFF 0
#define WS2812_RED 50<<8
#define WS2812_BLUE 50<<16

#define WINDMILL_INIT_COLOR WS2812_RED

typedef enum{
  WAIT_FOR_DISPLAY = 0,
  DISPLAY_ON = 1,
  DISPLAY_OFF = 2,
  DISPLAY_FLOW = 3,
  DISPLAY_TOP = 3,
}DisplayType;

typedef struct {
  union {
    struct{
      uint8_t strip1;
      uint8_t strip2;
      uint8_t strip3;
      uint8_t strip4;
      uint8_t strip5;
    };
    uint8_t strips[5];
  };
  union{
    struct {
      uint8_t board1;
      uint8_t board2;
      uint8_t board3;
      uint8_t board4;
      uint8_t board5;
    };
    uint8_t boards[5];
  };
}WindmillStruct;

extern WindmillStruct windmill;

void WS2812_Init(void);
void WS2812_Update(void);
void WS2812_FlowUpdate(void);
void WS2812_StopDMA(TIM_HandleTypeDef *htim);

#endif // WINDMILL_FUNCTION_WS2812_H_
