/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_custom_hid_if.h"
#include "dwt.h"
#include "read_joy.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

//#define _DEBUG
volatile uint8_t m_gamepad_updated = 0;
volatile uint8_t m_usb_report_need_update = 0;

Joy_Control m_joys;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
TIM_HandleTypeDef htim2_gamepad;

extern USBD_HandleTypeDef hUsbDeviceFS;

volatile uint8_t byte1 = 0;
volatile uint8_t byte2 = 0;
volatile uint8_t rep[16] = {0};

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == TIM2) {
        m_gamepad_updated = 1;
    }
}

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_USB_DEVICE_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start_IT(&htim2_gamepad);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

    if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST)) {
            HAL_UART_Transmit(&huart2, (uint8_t *)"reset\r\n", 7, 50);
    } else {
            HAL_UART_Transmit(&huart2, (uint8_t *)"init\r\n", 6, 50);
    }

    // keyboard codes 0-9
//    uint8_t button_code[10] = {
//        0x27,         // 0
//        0x1E,         // 1
//        0x1F,         // 2
//        0x20,         // 3
//        0x21,         // 4
//        0x22,         // 5
//        0x23,         // 6
//        0x24,         // 7
//        0x25,         // 8
//        0x26          // 9
//    };
//    uint8_t button_code[10] = {
//        0x10,         // 0
//        0x11,         // 1
//        0x12,         // 2
//        0x13,         // 3
//        0x14,         // 4
//        0x15,         // 5
//        0x16,         // 6
//        0x17,         // 7
//        0x18,         // 8
//        0x19          // 9
//    };

  InitControl_Joysticks(&m_joys);

  while (1)
  {

#ifdef _DEBUG
        static uint8_t one = 1;
        if (byte1 != 0xFF || byte2 != 0xFF) {
            char tmp[64];
            if (one) {
                one = 0;
                sprintf(tmp, "first gamepad data: %02X:%s:%s:%s:%s:%s:%s:%s:%s\r\n", byte1
                , !((byte1 >> 7) & 1) ? "RIGHT" : ""
                , !((byte1 >> 6) & 1) ? "LEFT" : ""
                , !((byte1 >> 5) & 1) ? "DOWN" : ""
                , !((byte1 >> 4) & 1) ? "UP" : ""
                , !((byte1 >> 3) & 1) ? "START" : ""
                , !((byte1 >> 2) & 1) ? "SELECT" : ""
                , !((byte1 >> 1) & 1) ? "B" : ""
                , !((byte1 >> 0) & 1) ? "A" : "");
                HAL_UART_Transmit(&huart2, (uint8_t *)tmp, strlen(tmp), 50);
                sprintf(tmp, "second gamepad data: %02X:%s:%s:%s:%s:%s:%s:%s:%s\r\n", byte2
                , !((byte2 >> 7) & 1) ? "RIGHT" : ""
                , !((byte2 >> 6) & 1) ? "LEFT" : ""
                , !((byte2 >> 5) & 1) ? "DOWN" : ""
                , !((byte2 >> 4) & 1) ? "UP" : ""
                , !((byte2 >> 3) & 1) ? "START" : ""
                , !((byte2 >> 2) & 1) ? "SELECT" : ""
                , !((byte2 >> 1) & 1) ? "B" : ""
                , !((byte2 >> 0) & 1) ? "A" : "");
                HAL_UART_Transmit(&huart2, (uint8_t *)tmp, strlen(tmp), 50);
            }
        } else {
            one = 1;
        }
#endif

    if (m_gamepad_updated) {
        m_gamepad_updated = 0;
			  m_joys.init();
        m_joys.read_joys();
        m_joys.send_report();
//        readTwoGamepads(byte1, byte2)
//        memset((void *)rep, 0x00, sizeof(rep));
//        rep[3] = (~byte1 & 0x0F);
//        rep[7] = (~byte2 & 0x0F);

//        // Y
//        if (!((byte1 >> 4) & 1)) {
//            rep[2] = (uint8_t)(-127);
//        } else if (!((byte1 >> 5) & 1)) {
//            rep[2] = (uint8_t)(127);
//        } else {
//            rep[2] = 0x00;
//        }

//        // X
//        if (!((byte1 >> 6) & 1)) {
//            rep[1] = (uint8_t)(-127);
//        } else if (!((byte1 >> 7) & 1)) {
//            rep[1] = (uint8_t)(127);
//        } else {
//            rep[1] = 0x00;
//        }
//        
//        // Y
//        if (!((byte2 >> 4) & 1)) {
//            rep[6] = (uint8_t)(-127);
//        } else if (!((byte2 >> 5) & 1)) {
//            rep[6] = (uint8_t)(127);
//        } else {
//            rep[6] = 0x00;
//        }

//        // X
//        if (!((byte2 >> 6) & 1)) {
//            rep[5] = (uint8_t)(-127);
//        } else if (!((byte2 >> 7) & 1)) {
//            rep[5] = (uint8_t)(127);
//        } else {
//            rep[5] = 0x00;
//        }

//        rep[0] = 1;
//        rep[4] = 2;
//        rep[8] = 3;
//        rep[12] = 4;
//        USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, (uint8_t *)rep, sizeof(rep));
    }
    //HAL_Delay(16);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
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
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV8;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2_gamepad.Instance = TIM2;
  htim2_gamepad.Init.Prescaler = 35999 - 2;
  htim2_gamepad.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2_gamepad.Init.Period = 16 * 2 - 1;
  htim2_gamepad.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2_gamepad.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2_gamepad) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2_gamepad, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2_gamepad, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA5 PA10*/
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
