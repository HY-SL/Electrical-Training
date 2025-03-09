/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "i2c.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "oled.h"
#include "BMP.h"

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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */


void Press_Down(void);
void Release(void);   
void DoubleClick(void);
void Long_Press(void);
void SingleClick(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/*************************** ↓↓ 1.Software_Delay ↓↓ ***************************/
uint16_t Count1 = 0;
void Key_Scan(void)
{
    if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3) == 0)
    {
        HAL_Delay(15);
        while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3) == 0);
        Count1 ++;
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    }
}
/*************************** ↑↑ 1.Software_Delay ↑↑ ***************************/


/*************************** ↓↓ 2.External_Interrupt ↑↑ ***************************/
//uint16_t Count1 = 0;
//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
//{
//    if(GPIO_Pin == GPIO_PIN_0)
//    {
//        if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0)==0)
//        {
//            Count1 ++;
//            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
//        }
//    }
//}
/*************************** ↑↑ 2.External_Interrupt ↑↑ ***************************/


/*************************** ↓↓ 3.Timer_Timing ***************************/
//#define Double_Press_Time 400   // 400ms
//#define Long_Press_Time   10000  // 1s

//uint8_t Key_State, Key_Last_State = 1;
//uint8_t Double_Flag = 0;
//uint32_t Tick;

//uint16_t Count1 = 0;
//uint16_t Count2 = 0;
//uint16_t Count3 = 0;
//uint16_t Count4 = 0;

//void Key_Scan(void)
//{
//    Key_State = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3);//获取按键状态
//    if(Key_State == 0 && Key_Last_State == 1)       //判断按下，（下降沿）
//    {
//        TIM1->CNT=0;
//        Count1++;
//        if(Double_Flag == 0)
//        {
//            Tick = HAL_GetTick();
//            Double_Flag = 1;
//        }
//        else if(Double_Flag == 1)
//        {
//            if(HAL_GetTick() - Tick <= Double_Press_Time)
//            {
//                Double_Flag = 2;
//                Count4 ++;
//            }
//        }
//    }
//    if(Key_State == 0 && Key_Last_State == 0)       //判断按键是否仍被按下
//    {
//        if(TIM1->CNT >= Long_Press_Time)
//        {
//            Count2 ++;
//        }
//    }
//    
//    if(Double_Flag == 1)
//    {
//        if(HAL_GetTick() - Tick > Double_Press_Time)
//        {
//            Double_Flag = 0;
//        }
//    }    
//    
//    if(Key_State == 1 && Key_Last_State == 0)       //判断松开，（上升沿）
//    {
//        Count3 ++;
//        if(TIM1->CNT < Long_Press_Time)
//        {
//            if(Double_Flag == 2)
//            {
//                Double_Flag = 0;
//            }
//        }
//    }
//    Key_Last_State = Key_State;
//}

//void Press_Down(void)
//{
//    Count1 ++;
//}

//void Release(void) 
//{
//    Count2 ++;
//}

//void DoubleClick(void)
//{
//    Count3 ++;
//}
//void Long_Press(void)
//{
//    Count4 ++;
//}
//void SingleClick(void)
//{
//}
/*************************** ↑↑ 3.Timer_Timing ↑↑ ***************************/


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
  MX_I2C1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
    HAL_TIM_Base_Start(&htim1);
    OLED_Init();
    OLED_Clear();
    
    OLED_ShowString(12,0,"Don1:",16);
    OLED_ShowString(12,2,"UP 2:",16);
    OLED_ShowString(12,4,"Dou3:",16);
    OLED_ShowString(12,6,"Lon4:",16);
    
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
    
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1)
    {
/************ ↓↓ 1 ↓↓ ************/
        Key_Scan();
        OLED_ShowNum(55,0,Count1,3,16);


/************ ↓↓ 2 ↓↓ ************/
//        OLED_ShowNum(55,0,Count1,3,16);


/************ ↓↓ 3 ↓↓ ************/
//        Key_Scan();
//        OLED_ShowNum(55,0,Count1,3,16);
//        OLED_ShowNum(55,2,Count2,3,16);
//        OLED_ShowNum(55,4,Count3,3,16);
//        OLED_ShowNum(55,6,Count4,3,16);
        
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
