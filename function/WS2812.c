//
// Created by 58286 on 2022/7/17.
//
#include "WS2812.h"

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
  uint8_t dma1_state;
  uint8_t dma2_state;
}WindmillStruct;

static WindmillStruct windmill_state;

typedef struct {
  TIM_HandleTypeDef *htim;
  uint32_t tim_channel;
}WindmillTIM_ChannelStruct;

WindmillTIM_ChannelStruct windmill_tim_channel[10];

uint8_t led_strip_on[STRIP_LENGTH+1][24];
//uint8_t led_strip_on[STRIP_LENGTH][24];
uint8_t led_strip_off[STRIP_LENGTH+1][24];
uint8_t led_strip_top[STRIP_LENGTH+1][24];

uint8_t led_board_on[BOARD_LENGTH+1][24];
uint8_t led_board_off[BOARD_LENGTH+1][24];

uint8_t led_flow[BOARD_LENGTH+1][24];

uint8_t flow[48] = {
    1,0,0,0,0,0,0,1,
    1,1,0,0,0,0,1,1,
    1,1,1,0,0,1,1,1,
    0,1,1,1,1,1,1,0,
    0,0,1,1,1,1,0,0,
    0,0,0,1,1,0,0,0
};

inline void SetSingleColor(uint8_t led[24],uint32_t color)
{
  for(uint8_t i=0;i<24;i++){
    if((color>>i)&1) led[i] = WS2812_H;
    else led[i] = WS2812_L;
  }
}

void SetColor(uint32_t color,uint8_t leds[][24],uint32_t length)
{
  for(uint32_t i=0;i<length;i++)
    SetSingleColor(leds[i],color);
}

void SetFlowColor(uint32_t color,uint8_t leds[BOARD_LENGTH][24])
{
  uint8_t j=0;
  for(uint32_t i=0;i<BOARD_LENGTH/8;i++){
    for(uint8_t n=0;n<8;n++){
      if(flow[8*j+n]) SetSingleColor(leds[8*i+n],WINDMILL_INIT_COLOR);
      else SetSingleColor(leds[8*i+n],WS2812_OFF);
    }
    j++;
    if(j==6) j=0;
  }
}

void WS2812_FlowUpdate(void)
{
  uint32_t flow_buffer[40];
  memcpy(flow_buffer,flow,40);
  memcpy(flow,flow+40,8);
  memcpy(flow+8,flow_buffer,40);
  SetFlowColor(WINDMILL_INIT_COLOR,led_flow);
}

void WS2812_Init(void)
{
  memset(led_strip_on,0,(STRIP_LENGTH+1)*24);
  memset(led_strip_off,0,(STRIP_LENGTH+1)*24);
  memset(led_strip_top,0,(STRIP_LENGTH+1)*24);
  memset(led_board_on,0,(STRIP_LENGTH+1)*24);
  memset(led_board_off,0,(STRIP_LENGTH+1)*24);
  memset(led_flow,0,(STRIP_LENGTH+1)*24);

  SetColor(WINDMILL_INIT_COLOR, led_strip_on,STRIP_LENGTH);
  SetColor(WS2812_OFF, led_strip_off,STRIP_LENGTH);

  SetColor(WS2812_OFF, led_strip_top,BOT_LENGTH);
  SetColor(WINDMILL_INIT_COLOR, &led_strip_top[BOT_LENGTH],TOP_LENGTH);

  SetColor(WINDMILL_INIT_COLOR,led_board_on,BOARD_LENGTH);
  SetColor(WS2812_OFF,led_board_off,BOARD_LENGTH);

  SetFlowColor(WINDMILL_INIT_COLOR,led_flow);

  windmill_tim_channel[0].htim = STRIP1_TIM;
  windmill_tim_channel[0].tim_channel = STRIP1_CHANNEL;

  windmill_tim_channel[1].htim = STRIP2_TIM;
  windmill_tim_channel[1].tim_channel = STRIP2_CHANNEL;

  windmill_tim_channel[2].htim = STRIP3_TIM;
  windmill_tim_channel[2].tim_channel = STRIP3_CHANNEL;

  windmill_tim_channel[3].htim = STRIP4_TIM;
  windmill_tim_channel[3].tim_channel = STRIP4_CHANNEL;

  windmill_tim_channel[4].htim = STRIP5_TIM;
  windmill_tim_channel[4].tim_channel = STRIP5_CHANNEL;

  windmill_tim_channel[5].htim = BOARD1_TIM;
  windmill_tim_channel[5].tim_channel = BOARD1_CHANNEL;

  windmill_tim_channel[6].htim = BOARD2_TIM;
  windmill_tim_channel[6].tim_channel = BOARD2_CHANNEL;

  windmill_tim_channel[7].htim = BOARD3_TIM;
  windmill_tim_channel[7].tim_channel = BOARD3_CHANNEL;

  windmill_tim_channel[8].htim = BOARD4_TIM;
  windmill_tim_channel[8].tim_channel = BOARD4_CHANNEL;

  windmill_tim_channel[9].htim = BOARD5_TIM;
  windmill_tim_channel[9].tim_channel = BOARD5_CHANNEL;
}

void WS2812_Update(void)
{
  static uint8_t strip_index = 0;
  static uint8_t board_index = 0;
  if(strip_index == 5) strip_index=0;
  if(board_index == 5) board_index=0;
  if(windmill_state.dma1_state == 0){
    for(;strip_index<5;strip_index++) {
      if (windmill_state.strips[strip_index]) {
        switch (windmill_state.strips[strip_index]) {
        case DISPLAY_ON:
          HAL_TIM_PWM_Start_DMA(windmill_tim_channel[strip_index].htim,
                                windmill_tim_channel[strip_index].tim_channel,
                                (uint32_t *)led_strip_on,
                                (STRIP_LENGTH + 1) * 24);
          break;
        case DISPLAY_OFF:
          HAL_TIM_PWM_Start_DMA(windmill_tim_channel[strip_index].htim,
                                windmill_tim_channel[strip_index].tim_channel,
                                (uint32_t *)led_strip_off,
                                (STRIP_LENGTH + 1) * 24);
          break;
        case DISPLAY_TOP:
          HAL_TIM_PWM_Start_DMA(windmill_tim_channel[strip_index].htim,
                                windmill_tim_channel[strip_index].tim_channel,
                                (uint32_t *)led_strip_top,
                                (STRIP_LENGTH + 1) * 24);
          break;
        default:
          break;
        }
        windmill_state.dma1_state = 1;
        windmill_state.strips[strip_index] = WAIT_FOR_DISPLAY;
        strip_index++;
        break;
      }
    }
  }
  if(windmill_state.dma2_state == 0 ){
    for(;board_index<5;board_index++) {
      if (windmill_state.boards[board_index]) {
        switch (windmill_state.boards[board_index]) {
        case DISPLAY_ON:
          HAL_TIM_PWM_Start_DMA(
              windmill_tim_channel[board_index + 5].htim,
              windmill_tim_channel[board_index + 5].tim_channel,
              (uint32_t *)led_board_on, (BOARD_LENGTH + 1) * 24);
          break;
        case DISPLAY_OFF:
          HAL_TIM_PWM_Start_DMA(
              windmill_tim_channel[board_index + 5].htim,
              windmill_tim_channel[board_index + 5].tim_channel,
              (uint32_t *)led_board_off, (BOARD_LENGTH + 1) * 24);
          break;
        case DISPLAY_FLOW:
          HAL_TIM_PWM_Start_DMA(
              windmill_tim_channel[board_index + 5].htim,
              windmill_tim_channel[board_index + 5].tim_channel,
              (uint32_t *)led_flow, (BOARD_LENGTH + 1) * 24);
          break;
        default:
          break;
        }
        windmill_state.dma2_state = 1;
        windmill_state.strips[board_index + 5] = WAIT_FOR_DISPLAY;
        board_index++;
        break;
      }
    }
  }
}

void WS2812_StopDMA(TIM_HandleTypeDef *htim)
{
  if(htim->Instance == TIM1 || htim->Instance == TIM2){
    for(uint8_t i=0;i<5;i++){
      HAL_TIM_PWM_Stop_DMA(windmill_tim_channel[i].htim,
                           windmill_tim_channel[i].tim_channel);
    }
    windmill_state.dma1_state = 0;
  }else if(htim->Instance == TIM5 || htim->Instance == TIM8){
    for(uint8_t i=5;i<10;i++){
      HAL_TIM_PWM_Stop_DMA(windmill_tim_channel[i].htim,
                           windmill_tim_channel[i].tim_channel);
    }
    windmill_state.dma2_state = 0;
  }
}

void WS2812_SetState(uint8_t number,DisplayType state)
{
  if(state!=0 && state<4){
    if(number<5) windmill_state.strips[number] = state;
    else if(number<10) windmill_state.boards[number-5] = state;
  }
}