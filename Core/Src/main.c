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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef enum {
  STATE_STOP,
  STATE_FORWARD,
  STATE_BACKWARD,
  STATE_LEFT,
  STATE_RIGHT
} MotorState;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//                         BOARD PIN   || MOTOR DRIVER PIN
// 왼쪽 모터 (Motor B)
#define MOTOR_L_F_Port      GPIOB       // B-IA가 연결된 포트
#define MOTOR_L_F_Pin       GPIO_PIN_0  // B-IA가 연결된 핀 (왼쪽 정방향)
#define MOTOR_L_B_Port      GPIOA       // B-IB가 연결된 포트
#define MOTOR_L_B_Pin       GPIO_PIN_4  // B-IB가 연결된 핀 (왼쪽 역방향)

// 오른쪽 모터 (Motor A)
#define MOTOR_R_F_Port      GPIOA       // A-IB가 연결된 포트
#define MOTOR_R_F_Pin       GPIO_PIN_0  // A-IB가 연결된 핀 (오른쪽 정방향)
#define MOTOR_R_B_Port      GPIOA       // A-IA가 연결된 포트
#define MOTOR_R_B_Pin       GPIO_PIN_1  // A-IA가 연결된 핀 (오른쪽 역방향)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

//osThreadId defaultTaskHandle;

TaskHandle_t rxTaskHandle;
TaskHandle_t cmdTaskHandle;
TaskHandle_t ctrlTaskHandle;

QueueHandle_t cmdQueueHandle;


/* USER CODE BEGIN PV */

volatile MotorState currentMotorState = STATE_STOP;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);

void RxTask(void * argument);
void CmdTask(void * argument);
void CtrlTask(void * argument);

/* USER CODE BEGIN PFP */
void go_forward(void);
void go_backward(void);
void turn_left(void);
void turn_right(void);
void stop(void);
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
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  cmdQueueHandle = xQueueCreate(8, sizeof(uint8_t));
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  xTaskCreate(RxTask,   "rx",   128, NULL, tskIDLE_PRIORITY + 2, &rxTaskHandle);
  xTaskCreate(CmdTask,  "cmd",  128, NULL, tskIDLE_PRIORITY + 2, &cmdTaskHandle);
  xTaskCreate(CtrlTask, "ctrl", 128, NULL, tskIDLE_PRIORITY + 2, &ctrlTaskHandle);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  vTaskStartScheduler();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, MOTOR_R_F_Pin|MOTOR_R_B_Pin|MOTOR_L_B_Pin|LED2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(MOTOR_L_F_GPIO_Port, MOTOR_L_F_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : MOTOR_R_F_Pin MOTOR_R_B_Pin MOTOR_L_B_Pin LED2_Pin */
  GPIO_InitStruct.Pin = MOTOR_R_F_Pin|MOTOR_R_B_Pin|MOTOR_L_B_Pin|LED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : MOTOR_L_F_Pin */
  GPIO_InitStruct.Pin = MOTOR_L_F_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MOTOR_L_F_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// --- 모터 제어 함수들 ---
void go_forward() {
    HAL_GPIO_WritePin(MOTOR_L_F_Port, MOTOR_L_F_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR_L_B_Port, MOTOR_L_B_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_R_F_Port, MOTOR_R_F_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR_R_B_Port, MOTOR_R_B_Pin, GPIO_PIN_RESET);
}

void go_backward() {
    HAL_GPIO_WritePin(MOTOR_L_F_Port, MOTOR_L_F_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_L_B_Port, MOTOR_L_B_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR_R_F_Port, MOTOR_R_F_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_R_B_Port, MOTOR_R_B_Pin, GPIO_PIN_SET);
}

void turn_left() {
    HAL_GPIO_WritePin(MOTOR_L_F_Port, MOTOR_L_F_Pin, GPIO_PIN_RESET); // 왼쪽 후진
    HAL_GPIO_WritePin(MOTOR_L_B_Port, MOTOR_L_B_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR_R_F_Port, MOTOR_R_F_Pin, GPIO_PIN_SET);   // 오른쪽 전진
    HAL_GPIO_WritePin(MOTOR_R_B_Port, MOTOR_R_B_Pin, GPIO_PIN_RESET);
}

void turn_right() {
    HAL_GPIO_WritePin(MOTOR_L_F_Port, MOTOR_L_F_Pin, GPIO_PIN_SET);   // 왼쪽 전진
    HAL_GPIO_WritePin(MOTOR_L_B_Port, MOTOR_L_B_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_R_F_Port, MOTOR_R_F_Pin, GPIO_PIN_RESET); // 오른쪽 후진
    HAL_GPIO_WritePin(MOTOR_R_B_Port, MOTOR_R_B_Pin, GPIO_PIN_SET);
}

void stop() {
    HAL_GPIO_WritePin(MOTOR_L_F_Port, MOTOR_L_F_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_L_B_Port, MOTOR_L_B_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_R_F_Port, MOTOR_R_F_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_R_B_Port, MOTOR_R_B_Pin, GPIO_PIN_RESET);
}
// ------------

void RxTask(void * argument)
{
    uint8_t received_char;

    for (;;)
    {
        if (HAL_UART_Receive(&huart3, &received_char, 1, HAL_MAX_DELAY) == HAL_OK)
        {
            if (received_char != '\n' && received_char != '\r')
            {
                // 메시지 큐에 0 tick 기다리지 않고 보냄 (즉시 실패 시 버림)
                xQueueSend(cmdQueueHandle, &received_char, 0);
            }
        }
    }
}

void CmdTask(void * argument)
{
    uint8_t command;

    for (;;)
    {
        // 큐에서 명령어 받기 (무한 대기)
        if (xQueueReceive(cmdQueueHandle, &command, portMAX_DELAY) == pdPASS)
        {
            switch (command)
            {
                case 'F': currentMotorState = STATE_FORWARD;  break;
                case 'B': currentMotorState = STATE_BACKWARD; break;
                case 'L': currentMotorState = STATE_LEFT;     break;
                case 'R': currentMotorState = STATE_RIGHT;    break;
                case 'S': currentMotorState = STATE_STOP;     break;
                default :  /* 무시 */                          break;
            }
        }
    }
}

void CtrlTask(void * argument)
{
    MotorState lastState = -1;  // 마지막으로 처리한 상태

    for (;;)
    {
        if (lastState != currentMotorState)
        {
            lastState = currentMotorState;

            switch (currentMotorState)
            {
                case STATE_FORWARD:  go_forward();  break;
                case STATE_BACKWARD: go_backward(); break;
                case STATE_LEFT:     turn_left();   break;
                case STATE_RIGHT:    turn_right();  break;
                case STATE_STOP:     stop();        break;
                default:             stop();        break;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(20));  // 20ms 지연
    }
}


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

#ifdef USE_FULL_ASSERT
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
