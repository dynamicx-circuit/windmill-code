//
// Created by 58286 on 2022/7/17.
//
#include "WS2812.h"

typedef struct{
  TIM_HandleTypeDef *htim;
  uint32_t tim_channel;
  uint16_t length;
  uint8_t *leds;
}LED_TypeStruct;

LED_TypeStruct led_struct[LED_NUM] ={
        {&htim1,TIM_CHANNEL_3,STRIP_LENGTH,0},
        {&htim2,TIM_CHANNEL_1,STRIP_LENGTH,0},
        {&htim2,TIM_CHANNEL_2,STRIP_LENGTH,0},
        {&htim1,TIM_CHANNEL_1,STRIP_LENGTH,0},
        {&htim2,TIM_CHANNEL_3,STRIP_LENGTH,0},
        {&htim5,TIM_CHANNEL_3,BOARD_LENGTH,0},
        {&htim5,TIM_CHANNEL_2,BOARD_LENGTH,0},
        {&htim5,TIM_CHANNEL_1,BOARD_LENGTH,0},
        {&htim5,TIM_CHANNEL_4,BOARD_LENGTH,0},
        {&htim8,TIM_CHANNEL_1,BOARD_LENGTH,0}
    };
uint32_t led_color_init[LED_ColorCategory] = {
    WS2812_OFF,
    WS2812_RED,
    WS2812_BLUE
};

uint8_t led_used_length = 0;
uint8_t led_buffer[LED_LENGTH];
uint8_t led_color[LED_ColorCategory][24];

void WS2812_Init(void){
  for(uint8_t i=0;i<LED_NUM;i++){
    if(led_struct->length + led_used_length > LED_LENGTH){
      break;
    }
    led_struct->leds = led_buffer+led_used_length;
    led_used_length += led_struct->length;
  }
  for(uint8_t i=0;i<LED_ColorCategory;i++){
    uint32_t temp = led_color_init[i];
    for(uint8_t j=0;j<24;j++){
      if((temp >> j) != 0)
        led_color[i][j] = WS2812_H;
      else
        led_color[i][j] = WS2812_L;
    }
  }
}

