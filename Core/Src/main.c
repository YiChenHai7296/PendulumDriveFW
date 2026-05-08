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
#include "adc.h"
#include "crc.h"
#include "dma.h"
#include "hrtim.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "simulink_protocol.h"
#include "State_Machine.h"
#include "motor_service.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define Debug_Menu_WORKING    0x31
#define Debug_Menu_ADC_TEST   0x32
//#define Debug_Menu_USART_TSET 0x32
#define Debug_Menu_PWM_TEST   0x33
#define Debug_Menu_CRC_TEST   0x34
/* 1: 主循环打印 ADC3_IN1；0: 正常状态机流程 */
#define MAIN_LOOP_ADC3_DEBUG_PRINT 0

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t u8DebugMenuChoose = 0;

uint8_t pbuff[10] = {0};

int i = 0;
int User_uart_status = 0;



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

unsigned int u32CRC_TEST();


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */




unsigned int u32CRC_TEST()
{
  unsigned short crc = 0;
  uint8_t au8CrcData[12] = {1,2,3,4,5,6,7,8,9,0,1,2};

  crc = HAL_CRC_Accumulate(&hcrc,(unsigned int *)au8CrcData,12);
  printf("CRC:0x%x \n",crc);

  return crc;
}




HAL_StatusTypeDef StatusU5 = 0;





void Working_Task()
{
  



  return;
}







/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  //uint8_t tcp_demo_sendbuf[80]="G4 Uart Test \n";
  int i = 655356;
  uint32_t crc = 0;


  uint32_t crc_Data[6] = {0x00010203,0x09080607,0x0002,0x0003,0x0004,0x0005};
  int j = 0;


  uint32_t TEST = 0x12345678;
  uint32_t *p_TEST = &TEST;

    
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  //__disable_irq();

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CRC_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_HRTIM1_Init();
  MX_TIM1_Init();
  MX_ADC3_Init();
  /* USER CODE BEGIN 2 */


  /* HRTIM启动 */
  HAL_HRTIM_WaveformOutputStart(&hhrtim1,HRTIM_OUTPUT_TA1);
  HAL_HRTIM_WaveformOutputStart(&hhrtim1,HRTIM_OUTPUT_TB1);
  HAL_HRTIM_WaveformCountStart_IT(&hhrtim1,HRTIM_TIMERID_TIMER_A);
  HAL_HRTIM_WaveformCountStart_IT(&hhrtim1,HRTIM_TIMERID_TIMER_B);
  HAL_HRTIM_WaveformCountStart(&hhrtim1,HRTIM_TIMERID_MASTER);


  /* 串口启动 */
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  MotorService_InitMotor();

  printf("\n\n  ========== DCM_G474_V1.0 Menu ========== \n" );
  printf("                          编译日期:%s\n",__DATE__);


  StateMachine_MainLoop();



#if 0 
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


    #if 0
		Protocol_FeedbackFrame_t pFeedbackFrame_test;
    uint8_t pbufftest[32]= 0;

    pFeedbackFrame_test.motor_current = 0x12;
    pFeedbackFrame_test.motor_position = 0x1234;
    pFeedbackFrame_test.motor_speed = 0x1234;
    pFeedbackFrame_test.axis_position = 0x1234;
    pFeedbackFrame_test.axis_speed = 0x123;
    pFeedbackFrame_test.pendulum_position = 0x5678;
    pFeedbackFrame_test.pendulum_speed = 0x123;



    Protocol_PackFeedbackFrame(&pFeedbackFrame_test, pbufftest);
#endif




    //HAL_UART_Transmit(&huart1, tcp_demo_sendbuf, 15, 10000);

    printf("\n\n  ========== DCM_G474_V1.0 Menu ========== \n" );
    printf("                          编译日期:%s\n",__DATE__);
    printf("  1. ADC 采集精度测试  \n" );
    printf("  2. PWM占空比修改测�??  \n" );
    printf("  3. CRC校验计算测试...  \n" );
    printf("  ==================================== \n" );

    while(HAL_OK != HAL_UART_Receive(DEBUG_UART,g_au8DebugRxBuff,1,200));

    u8DebugMenuChoose = g_au8DebugRxBuff[0];
    
    switch(u8DebugMenuChoose)
    {
        case Debug_Menu_WORKING:
        {
            printf("  进入正常工作模式!  \n");
            Working_Task();
            break;
        }
    
        case Debug_Menu_ADC_TEST:
        {
            printf("  进入ADC精度测试  \n" );
            ADC_TEST();
            break;
        }

        case Debug_Menu_PWM_TEST:
        {
            printf("  进入PWM波发生测试！  \n" );
            PWM_TEST();
            break;
        }

        case Debug_Menu_CRC_TEST:
        {
            printf("  进入CRC校验计算测试  \n" );
            u32CRC_TEST();
            break;
        }

        default:
        {
            printf("  键入有误，请重试  \n" );
        }
    }

    u8DebugMenuChoose = 0;

    HAL_Delay(500);


#if 0

    crc = HAL_CRC_Accumulate(&hcrc,crc_Data,24);
		//crc = HAL_CRC_Accumulate(&hcrc,crc_Data,24);
    //HAL_UART_Transmit_IT(&huart1,(const uint8_t *)"\n CRC:",7);
  StatusU5 = HAL_UART_Transmit_IT(&huart1,(const uint8_t *)&crc,4);

crc = HAL_CRC_Accumulate(&hcrc,crc_Data,24);

#endif



  }
	#endif
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
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
