
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"
#include "usbd_customhid.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "config.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

//#define _DEBUG
volatile uint8_t m_gamepad_updated = 0;
volatile uint8_t m_usb_report_need_update = 0;

static void delay_us()
{
    for(volatile uint16_t i = 0; i < 100; ++i);
}

#ifdef NES
#define readTwoGamepads(byte1, port1, latch1, clk1, data1       \
                        , byte2, port2, latch2, clk2, data2)    \
        HAL_GPIO_WritePin(port1, latch1, GPIO_PIN_SET);         \
        HAL_GPIO_WritePin(port2, latch2, GPIO_PIN_SET);         \
        delay_us();\
        HAL_GPIO_WritePin(port1, latch1, GPIO_PIN_RESET);       \
        HAL_GPIO_WritePin(port2, latch2, GPIO_PIN_RESET);       \
                                                                \
        byte1 = 0;                                              \
        byte2 = 0;                                              \
        for(int i = 0; i < 8; ++i) {                            \
            {                                                   \
            delay_us();\
            HAL_GPIO_WritePin(port1, clk1, GPIO_PIN_RESET);       \
            HAL_GPIO_WritePin(port2, clk2, GPIO_PIN_RESET);       \
            const uint8_t bit = HAL_GPIO_ReadPin(port1, data1); \
            byte1 |= ((bit & 0x1) << i);                        \
                if (bit) { HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, bit ? GPIO_PIN_SET : GPIO_PIN_RESET); \
            } \
            }                                                   \
            {                                                   \
            const uint8_t bit = HAL_GPIO_ReadPin(port2, data2); \
            byte2 |= ((bit & 0x1) << i);                        \
            if (bit) { HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, bit ? GPIO_PIN_SET : GPIO_PIN_RESET); \
            } \
            }                                                   \
            HAL_GPIO_WritePin(port1, clk1, GPIO_PIN_SET);     \
            HAL_GPIO_WritePin(port2, clk2, GPIO_PIN_SET);     \
        }

#elif defined(SEGA)
#define readTwoGamepadsAtSEGA(byte1, port1, sel, up_z, down_y, left_x \
                        , byte2, port2, right_mode, A_B, C_start)     \
        HAL_GPIO_WritePin(port1, sel, GPIO_PIN_SET);                  \
        delayUS_DWT(20);                                              \
                                                                      \
        byte1 = 0;                                                    \
        byte2 = 0;                                                    \
        const uint8_t up = HAL_GPIO_ReadPin(port1, up_z);             \
        const uint8_t down = HAL_GPIO_ReadPin(port1, down_y);         \
        const uint8_t left = HAL_GPIO_ReadPin(port1, left_x);         \
        const uint8_t right = HAL_GPIO_ReadPin(port2, right_mode);    \
        const uint8_t B = HAL_GPIO_ReadPin(port2, A_B);               \
        const uint8_t C = HAL_GPIO_ReadPin(port2, C_start);           \
        byte1 |= (up << 0);                                           \
        byte1 |= (down << 1);                                         \
        byte1 |= (left << 2);                                         \
        byte1 |= (right << 3);                                        \
        byte2 |= (C << 2);                                            \
        byte2 |= (B << 1);                                      \
                                                                \
        HAL_GPIO_WritePin(port1, sel, GPIO_PIN_RESET);          \
        delayUS_DWT(20);                                        \
                                                                \
        const uint8_t A = HAL_GPIO_ReadPin(port2, A_B);         \
        const uint8_t START = HAL_GPIO_ReadPin(port2, C_start); \
        byte2 |= (A << 0);/*A*/                                 \
        byte2 |= (START << 3);/*START*/                         \
                                                                \
        HAL_GPIO_WritePin(port1, sel, GPIO_PIN_SET);            \
        delayUS_DWT(20);                                        \
        HAL_GPIO_WritePin(port1, sel, GPIO_PIN_RESET);          \
        delayUS_DWT(20);                                        \
        HAL_GPIO_WritePin(port1, sel, GPIO_PIN_SET);            \
        delayUS_DWT(20);                                        \
        HAL_GPIO_WritePin(port1, sel, GPIO_PIN_RESET);          \
        delayUS_DWT(20);                                        \
        HAL_GPIO_WritePin(port1, sel, GPIO_PIN_SET);            \
        delayUS_DWT(20);                                        \
                                                                \
        const uint8_t Z = HAL_GPIO_ReadPin(port1, up_z);        \
        const uint8_t Y = HAL_GPIO_ReadPin(port1, down_y);      \
        const uint8_t X = HAL_GPIO_ReadPin(port1, left_x);      \
        const uint8_t MODE = HAL_GPIO_ReadPin(port2, right_mode); \
        byte2 |= (X << 4); /*X*/                                \
        byte2 |= (Y << 5); /*Y*/                                \
        byte2 |= (Z << 6); /*Z*/                                \
        byte2 |= (MODE << 7); /*MODE*/                          \
                                                                \
        HAL_GPIO_WritePin(port1, sel, GPIO_PIN_RESET);          \
        delayUS_DWT(20);                                        \
        HAL_GPIO_WritePin(port1, sel, GPIO_PIN_SET);            \
        delayUS_DWT(20);

#endif

#if !defined(SEGA) && !defined(NES)
    error: must be defined NES or SEGA
#endif

#define readGamepad(byte, port, latch, clk, data)               \
        HAL_GPIO_WritePin(port, latch, GPIO_PIN_SET);           \
        HAL_GPIO_WritePin(port, latch, GPIO_PIN_RESET);         \
                                                                \
        byte = 0;                                               \
        for(int i = 0; i < 8; ++i) {                            \
            const uint8_t bit = HAL_GPIO_ReadPin(port, data);   \
            byte |= ((bit & 0x1) << i);                         \
            HAL_GPIO_WritePin(port, clk, GPIO_PIN_RESET);       \
            HAL_GPIO_WritePin(port, clk, GPIO_PIN_SET);         \
        }                                                       \

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim2_gamepad;
        
TIM_HandleTypeDef htim4_usb;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
extern USBD_HandleTypeDef hUsbDeviceFS;

volatile uint8_t byte1 = 0;
volatile uint8_t byte2 = 0;
volatile uint8_t rep[8] = {0};

#define    DWT_CYCCNT    *(volatile unsigned long *)0xE0001004
#define    DWT_CONTROL   *(volatile unsigned long *)0xE0001000
#define    SCB_DEMCR     *(volatile unsigned long *)0xE000EDFC

void DWT_Init(void)
{
     SCB_DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
     DWT_CYCCNT  = 0;
     DWT_CONTROL|= DWT_CTRL_CYCCNTENA_Msk; 
}

void delayUS_DWT(uint32_t us) {
    volatile uint32_t cycles = (SystemCoreClock/1000000L) * us;
    volatile uint32_t start = DWT->CYCCNT;
    do  {
    } while((DWT->CYCCNT - start) < cycles);
}


void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

  /* USER CODE END TIM2_IRQn 0 */
    HAL_TIM_IRQHandler(&htim2_gamepad);
  /* USER CODE BEGIN TIM2_IRQn 1 */
    m_gamepad_updated = 1;
  /* USER CODE END TIM2_IRQn 1 */
}

void TIM4_IRQHandler(void)
{
  /* USER CODE BEGIN TIM4_IRQn 0 */

  /* USER CODE END TIM4_IRQn 0 */
    HAL_TIM_IRQHandler(&htim4_usb);
  /* USER CODE BEGIN TIM4_IRQn 1 */
    m_usb_report_need_update = 1;
    
     #ifdef NES
        rep[3] = (~byte1 & 0x0F);
        rep[7] = (~byte2 & 0x0F);

        // Y
        if (!((byte1 >> 4) & 1)) {
            rep[2] = (uint8_t)(-127);
        } else if (!((byte1 >> 5) & 1)) {
            rep[2] = (uint8_t)(127);
        } else {
            rep[2] = 0x00;
        }

        // X
        if (!((byte1 >> 6) & 1)) {
            rep[1] = (uint8_t)(-127);
        } else if (!((byte1 >> 7) & 1)) {
            rep[1] = (uint8_t)(127);
        } else {
            rep[1] = 0x00;
        }
        
        // Y
        if (!((byte2 >> 4) & 1)) {
            rep[6] = (uint8_t)(-127);
        } else if (!((byte2 >> 5) & 1)) {
            rep[6] = (uint8_t)(127);
        } else {
            rep[6] = 0x00;
        }

        // X
        if (!((byte2 >> 6) & 1)) {
            rep[5] = (uint8_t)(-127);
        } else if (!((byte2 >> 7) & 1)) {
            rep[5] = (uint8_t)(127);
        } else {
            rep[5] = 0x00;
        }

        rep[0] = 1;
        rep[4] = 2;
    #elif defined(SEGA)
        rep[3] = (~byte2);
        rep[3] = 0x00;

        // Y
        if (!((byte1 >> 0) & 1)) {
            rep[2] = (uint8_t)(-127);
        } else if (!((byte1 >> 1) & 1)) {
            rep[2] = (uint8_t)(127);
        } else {
            rep[2] = 0x00;
        }

        // X
        if (!((byte1 >> 2) & 1)) {
            rep[1] = (uint8_t)(-127);
        } else if (!((byte1 >> 3) & 1)) {
            rep[1] = (uint8_t)(127);
        } else {
            rep[1] = 0x00;
        }

        rep[0] = 1;
        rep[4] = 2;
        memcpy(&rep[5], rep, 3);
    #endif
  /* USER CODE END TIM4_IRQn 1 */
}

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_gamepad_Init(void);
static void MX_TIM4_usb_Init(void);
static void MX_SPI2_Init(void);
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

  DWT_Init();

  MX_USB_DEVICE_Init();
  MX_USART2_UART_Init();
  MX_TIM2_gamepad_Init();
  MX_TIM4_usb_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim2_gamepad);
  HAL_TIM_Base_Start_IT(&htim2_gamepad);
  HAL_TIM_Base_Start(&htim4_usb);
  HAL_TIM_Base_Start_IT(&htim4_usb);
   
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

    if (1 || m_gamepad_updated) {
        //__disable_irq();
        m_gamepad_updated = 0;
        #if !TWO_GAMEPAD
            readGamepad(byte1, GPIOA, GPIO_PIN_4, GPIO_PIN_5, GPIO_PIN_6)
        #else
            #ifdef NES
                readTwoGamepads(byte1, GPIOA, GPIO_PIN_4, GPIO_PIN_5, GPIO_PIN_6
                                , byte2, GPIOB, GPIO_PIN_0, GPIO_PIN_1, GPIO_PIN_10)
            #elif defined(SEGA)
                static uint32_t polling_st = 0;
                if (!polling_st)
                    polling_st = DWT->CYCCNT;
                while((DWT->CYCCNT - polling_st) > ((SystemCoreClock/1000000L) * 1100)) {
                    readTwoGamepadsAtSEGA(byte1, GPIOA, GPIO_PIN_4, GPIO_PIN_5, GPIO_PIN_6, GPIO_PIN_7
                                , byte2, GPIOB, GPIO_PIN_0, GPIO_PIN_1, GPIO_PIN_10)
                    polling_st = 0;
                }
            #endif //SEGA
        #endif //!TWO_GAMEPAD
       //__enable_irq();
    }
    
    rep[3] = (~byte1 & 0x0F);
        rep[7] = (~byte2 & 0x0F);

        // Y
        if (!((byte1 >> 4) & 1)) {
            rep[2] = (uint8_t)(-127);
        } else if (!((byte1 >> 5) & 1)) {
            rep[2] = (uint8_t)(127);
        } else {
            rep[2] = 0x00;
        }

        // X
        if (!((byte1 >> 6) & 1)) {
            rep[1] = (uint8_t)(-127);
        } else if (!((byte1 >> 7) & 1)) {
            rep[1] = (uint8_t)(127);
        } else {
            rep[1] = 0x00;
        }
        
        // Y
        if (!((byte2 >> 4) & 1)) {
            rep[6] = (uint8_t)(-127);
        } else if (!((byte2 >> 5) & 1)) {
            rep[6] = (uint8_t)(127);
        } else {
            rep[6] = 0x00;
        }

        // X
        if (!((byte2 >> 6) & 1)) {
            rep[5] = (uint8_t)(-127);
        } else if (!((byte2 >> 7) & 1)) {
            rep[5] = (uint8_t)(127);
        } else {
            rep[5] = 0x00;
        }

        rep[0] = 1;
        rep[4] = 2;
        memset((uint8_t *)&rep[5], 0xFF, 3);
        USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, (uint8_t *)rep, sizeof(rep));
    HAL_Delay(16);
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

  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

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
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_gamepad_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2_gamepad.Instance = TIM2;
  htim2_gamepad.Init.Prescaler = 71999;
  htim2_gamepad.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2_gamepad.Init.Period = 100;
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

static void MX_TIM4_usb_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4_usb.Instance = TIM4;
  htim4_usb.Init.Prescaler = 71999;
  htim4_usb.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4_usb.Init.Period = 1;
  htim4_usb.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4_usb.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4_usb) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4_usb, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4_usb, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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

    /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

#ifdef NES
  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA4 PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
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
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
#elif defined(SEGA)
 /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10|GPIO_PIN_3, GPIO_PIN_RESET);

  // A4 - sel
  /*Configure GPIO pins : FIRST_JOY: PA5 PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : FIRST_JOY: PB0 PB1 PB10 SECOND_JOY: PB9 PB8 PB7 PB6 PB5 PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_8|GPIO_PIN_7|GPIO_PIN_6|GPIO_PIN_5|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  
  /*Configure GPIO pin : SECOND_JOY: PB3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  
  /*Configure GPIO pin : FIRST_JOY: PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
#endif
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
