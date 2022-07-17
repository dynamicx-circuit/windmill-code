//
// Created by 58286 on 2022/7/17.
//

#ifndef WINDMILL_FUNCTION_WS2812_H_
#define WINDMILL_FUNCTION_WS2812_H_
#include "main.h"
#include "tim.h"

#define LED_LENGTH 2150
#define LED_ColorCategory 3
#define LED_NUM 10

#define WS2812_H 1
#define WS2812_L 0

#define WS2812_OFF 0
#define WS2812_RED 50<<8
#define WS2812_BLUE 50

typedef enum{
  LED_OFF = 0,
  LED_RED = 1,
  LED_BLUE = 2
}UserDefineColor;

void WS2812_Init(void);
void SetGolbalLED_Color(uint8_t color_index,uint32_t color);

#endif // WINDMILL_FUNCTION_WS2812_H_
