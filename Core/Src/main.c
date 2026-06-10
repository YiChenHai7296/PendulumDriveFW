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
#include "hrtim.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* ======================== 1. 头文件引用（本文件额外） ======================== */
#include <stdio.h>            /* Error_Handler 内 printf */
#include "bsp.h"              /* Bsp_Init / Bsp_Ms_Delay（并透传 config.h 的 TEST_DRV/TEST_ADC） */
#include "adc.h"              /* TEST_DRV+TEST_ADC 自测时调用 Drv_ADC_TEST */
#include "State_Machine.h"    /* App_StateMachine_MainLoop */

/* ======================== 2. 私有宏定义 ======================== */
/*
 * 驱动自测开关 TEST_DRV / TEST_ADC 已集中至 `User/Config/Inc/config.h`（调试开关分区），
 * 经上方 bsp.h 透传 config.h 可见，此处仅保留组合合法性校验。
 * - TEST_DRV=0：正常应用，调用 App_StateMachine_MainLoop()（永不返回）。
 * - TEST_DRV=1：进入下方自测死循环；IWDG 仍依赖 TIM2 更新中断内 HAL_IWDG_Refresh。
 * - TEST_ADC 仅在 TEST_DRV=1 时生效：1 周期调用 Drv_ADC_TEST()；0 仅 Bsp_Ms_Delay(...)。
 */
#if (TEST_ADC != 0) && (TEST_DRV == 0)
#error "TEST_ADC=1 时必须同时 TEST_DRV=1，否则主循环不会调用 Drv_ADC_TEST"
#endif

/* ======================== 3. 私有类型定义 ======================== */
/* 无 */

/* ======================== 4. 对外变量定义 ======================== */
/* 无 */

/* ======================== 5. 私有变量 ======================== */
/* 无 */

/* ======================== 6. 私有函数声明 ======================== */
/* 无 */

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
  /* USER CODE BEGIN 2 */
  /* 外设 MX 初始化与 HRTIM 启动（HAL_Init / SystemClock_Config 已由上方 Cube 模板完成） */
  Bsp_Init();
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  BSP_LOG_PRINTF_FORCE("\n\nSoftscalePendulumDriveBoard V%s  ;  控制对象: %s  ;  编译日期: %s %s.\n", VERSION_NUM_STR, APP_CONTROL_OBJECT_NAME_STR, __DATE__, __TIME__);


#if TEST_DRV
  /* 驱动自测：不调用应用状态机；IWDG 仍依赖 TIM2 更新中断内 HAL_IWDG_Refresh */
  while (1)
  {
#if TEST_ADC
    Drv_ADC_TEST();
    Bsp_Ms_Delay(500);
#endif
  }
#else
  App_StateMachine_MainLoop();
#endif

  while (1)
  {
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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV6;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* ======================== 7. 接口函数实现 ======================== */
/* 无 */

/* ======================== 8. 私有函数实现 ======================== */
/* 无 */

/* ======================== 9. HAL 回调函数实现 ======================== */
/* 无 */

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
    printf("err\n");
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
