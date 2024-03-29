/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#include "WS2812.h"
//#include "sensor.h"
#include "windmill_logic.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
extern DMA_HandleTypeDef hdma_tim2_ch1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t flag = 0;
//uint8_t counter = 0;
extern uint16_t windmill_counter;
uint8_t sensor = 0;

uint8_t led_on[24];
uint8_t led_off[24];

uint8_t strip[5][24];

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
  WS2812_StopDMA(htim);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim->Instance == TIM3){
    WS2812_Update();
  }else if(htim->Instance == TIM4){
    WindmillUpdate();
  }
}

void KeyScan(void)
{
  static uint16_t sensor_counter[5] = {0,0,0,0,0};

  if(HAL_GPIO_ReadPin(Sensor1_GPIO_Port,Sensor1_Pin) == GPIO_PIN_SET){
    HAL_Delay(10);
    if(HAL_GPIO_ReadPin(Sensor1_GPIO_Port,Sensor1_Pin) == GPIO_PIN_SET){
      sensor|=1;
    }
  }

  if(HAL_GPIO_ReadPin(Sensor2_GPIO_Port,Sensor2_Pin) == GPIO_PIN_SET) {
    HAL_Delay(10);
    if (HAL_GPIO_ReadPin(Sensor2_GPIO_Port, Sensor2_Pin) == GPIO_PIN_SET) {
      sensor |= 2;
    }
  }

  if(HAL_GPIO_ReadPin(Sensor3_GPIO_Port,Sensor3_Pin) == GPIO_PIN_SET) {
    HAL_Delay(10);
    if (HAL_GPIO_ReadPin(Sensor3_GPIO_Port,Sensor3_Pin) == GPIO_PIN_SET) {
      sensor |= 4;
    }
  }

  if(HAL_GPIO_ReadPin(Sensor4_GPIO_Port,Sensor4_Pin) == GPIO_PIN_SET){
    if(HAL_GPIO_ReadPin(Sensor4_GPIO_Port,Sensor4_Pin) == GPIO_PIN_SET){
      sensor|=8;
    }
  }

  if(HAL_GPIO_ReadPin(Sensor5_GPIO_Port,Sensor5_Pin) == GPIO_PIN_SET){
    HAL_Delay(10);
    if(HAL_GPIO_ReadPin(Sensor5_GPIO_Port,Sensor5_Pin) == GPIO_PIN_SET){
      sensor|=16;
    }
  }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_CAN_Init();
  MX_TIM5_Init();
  MX_TIM8_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_TIM6_Init();
  MX_TIM4_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  WindmillInit();
//  SensorInit(&sensor);
  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_Base_Start_IT(&htim4);
  HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,GPIO_PIN_SET);

  HAL_ADC_Start(&hadc1);
  HAL_Delay(1);
  srand(HAL_ADC_GetValue(&hadc1));
  HAL_ADC_Stop(&hadc1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint8_t state=0;
  uint8_t random_num = 0x1F;
  uint8_t order[5];
  uint8_t order_counter = 0;
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    switch (state) {
    case 0:
      HAL_Delay(200);
      for(uint8_t i = 0;i<5;i++) SetFanBladeState(i,ALL_OFF);
      order_counter = 0;
      for(uint8_t i=0;i<5;i++) order[i] = i;
      for(uint8_t i=0;i<5;i++){
        uint8_t ran = rand()%5;
        uint8_t temp = order[ran];
        order[ran] = order[0];
        order[0] = temp;
      }
      state++;
      HAL_Delay(200);
      break;
    case 1:
      if(order_counter<5){
//        uint8_t temp;
        SetFanBladeState(order[order_counter],IMPACTED);
        while(1){
          KeyScan();
          if((sensor>>order[order_counter])&1) {
            SetFanBladeState(order[order_counter], ALL_ON);
            order_counter++;
            sensor = 0;
            break;
          }
//          }else if(sensor&(~(1<<order[order_counter]))){
//            state = 0;
//            sensor=0;
//            break;
//          }
//          if(sensor) HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,GPIO_PIN_RESET);
          sensor=0;
          HAL_Delay(10);
        }
      }else state++;
      break;
    case 2:
      for(uint8_t k=0;k<3;k++){
        HAL_Delay(200);
        for(uint8_t i=0;i<5;i++) SetFanBladeState(i,ALL_OFF);
        HAL_Delay(200);
        for(uint8_t i=0;i<5;i++) SetFanBladeState(i,ALL_ON);
      }
      state = 0;
      break;
    default:break;
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
