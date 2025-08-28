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
#include "stm32f1xx_hal.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct {
    float linear_v;  // 선형 속도 v
    float angular_w; // 각속도 w
} MotorCommand_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//                         BOARD PIN   || MOTOR DRIVER PIN
// 왼쪽 모터 (Motor B)
#define MOTOR_L_F_Port      GPIOB       // B-IA가 연결된 포트
#define MOTOR_L_F_Pin       GPIO_PIN_11  // B-IA가 연결된 핀 (왼쪽 정방향)
#define MOTOR_L_B_Port      GPIOB       // B-IB가 연결된 포트
#define MOTOR_L_B_Pin       GPIO_PIN_10  // B-IB가 연결된 핀 (왼쪽 역방향)

/* ===========================
 *  PWM / Timer 설정
 * =========================== */
// PWM 주기 계산식: PWM_freq = Timer_clk / ((PSC+1) × (ARR+1))
#define PWM_TIMER_PERIOD      (899)   // ARR 값
#define MOTOR_PWM_DEADZONE    (670)   // 모터가 돌기 시작하는 최소 Pulse 값
#define PWM_CLIP              (PWM_TIMER_PERIOD)

/* ===========================
 * 바퀴 파라미터
 * =========================== */
#define WHEEL_RADIUS_M        (0.033f)   // 바퀴 반지름 [m]
#define WHEEL_BASE_M          (0.12f)    // 좌/우 바퀴 간 거리 [m]

/* ===========================
 *  모터 파라미터
 * =========================== */
#define MOTOR_MAX_RPM         (200.0f)   // 모터 최대 RPM (HC02-48 모터로 추정됨)
#define MOTOR_DEADZONE_RPM    (10.0f)

/* ===========================
 *  제어 파라미터 (튜닝 필요)
 * =========================== */
#define KP_ANG                (0.3f)   // [rad -> rad/s] 각속도 비례 게인
#define BETA_CURVE            (0.03f)    // 조향각 클수록 감속 (rad 기준)
#define V_TARGET              (0.1f)    // 기본 목표 선속도 [m/s]

/* 속도 제한/보정 */
#define V_MIN_CLAMP           (0.005f)   // 최소 선속도 제한 [m/s]
#define V_MAX_CLAMP           (0.05f)    // 최대 선속도 제한 [m/s]

/* ===========================
 *  필터링 / 제어 주기
 * =========================== */
#define ALPHA_V               (0.2f)     // 속도 스무딩 계수 (0~1)
#define ALPHA_W               (0.3f)     // 각속도 스무딩 계수
#define CONTROL_PERIOD_MS     (20)       // 모터 제어 주기 [ms] (50Hz)

// FreeRTOS
#define UART_LINE_MAX         (50)
#define MOTOR_QUEUE_DEPTH     (4)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

osThreadId defaultTaskHandle;
/* USER CODE BEGIN PV */
QueueHandle_t motorCommandQueue;

TaskHandle_t uartReceiveTaskHandle;
TaskHandle_t motorControlTaskHandle;

/* UART 인터럽트 수신용 */
static uint8_t  uart_rx_byte;
static char     uart_line_buf[UART_LINE_MAX];
static uint8_t  uart_line_idx = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM2_Init(void);
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */
void uartReceiveTask(void * argument);
void motorControlTask(void * argument);
void executeMotorControl(float v, float w); 
void Motor_SetSpeed(int motor_id, float rpm);

int _write(int file, char *ptr, int len)
{
  HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, HAL_MAX_DELAY);
  return len;
}

static inline float clampf(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}
static inline float deg2rad(float d) { return d * (float)M_PI / 180.0f; }

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
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); // L_IA
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2); // L_IB
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3); // R_IA
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4); // R_IB
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
  motorCommandQueue = xQueueCreate(5, sizeof(MotorCommand_t));
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */

  // 태스크 생성
  xTaskCreate(uartReceiveTask, "UART_Rx_Task", 256, NULL, 1, NULL);
  xTaskCreate(motorControlTask, "Motor_Ctrl_Task", 256, NULL, 2, NULL); 
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 3;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 899;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 250;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 0;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* UART 수신 초기화 & 라인 파서 태스크
   - 인터럽트에서 바로 큐에 MotorCommand_t를 넣으므로
     이 태스크는 모니터링/기타 유지보수용(옵션)로 비워둘 수도 있음 */
void uartReceiveTask(void *argument) {
    // 인터럽트 수신 시작(한 바이트씩)
    HAL_UART_Receive_IT(&huart3, &uart_rx_byte, 1);

    for(;;) {
        // 필요 시 상태 출력/워치독/통계 등
        osDelay(1000);
    }
}

/* 메인 모터 제어 태스크 */
void motorControlTask(void *argument) {
	// 디버깅 버퍼
	static char dbg_buf[128];
    MotorCommand_t receivedCmd;
    float target_v = 0.0f; // 명령 목표
    float target_w = 0.0f;

    float filt_v = 0.0f;   // 필터된 목표
    float filt_w = 0.0f;

    for(;;) {
        // 1) 새 명령 수신 (있으면 즉시 업데이트)
        while (xQueueReceive(motorCommandQueue, &receivedCmd, 0) == pdPASS) {
            target_v = receivedCmd.linear_v;
            target_w = receivedCmd.angular_w;
        }

        //----------------//
        // === [테스트 입력: 항상 90도] ===
//	   int ai_angle = 0;
//
//	   // 변환 (90도면 steering_deg=0 → steering_rad=0)
//	   float steering_deg = (float)ai_angle - 90.0f;
//	   float steering_rad = deg2rad(steering_deg);
//
//	   target_w = KP_ANG * steering_rad;   // 항상 0
//	   float v_cmd = V_TARGET / (1.0f + BETA_CURVE * fabsf(steering_rad));
//	   target_v = clampf(v_cmd, V_MIN_CLAMP, V_MAX_CLAMP);
	   //------------------//

	   // 2) 스무딩 (저역통과)
        filt_v = (1.0f - ALPHA_V) * filt_v + ALPHA_V * target_v;
        filt_w = (1.0f - ALPHA_W) * filt_w + ALPHA_W * target_w;

        // 3) 좌/우 바퀴의 선속도 계산 (차동구동)
        float v_left  = filt_v - (filt_w * WHEEL_BASE_M) * 0.5f;
        float v_right = filt_v + (filt_w * WHEEL_BASE_M) * 0.5f;

        // 4) 선속도 -> RPM 변환
        //    wheel angular speed [rad/s] = v / R
        //    RPM = rad/s * 60 / (2*pi)
        float rpm_left  = (v_left  / WHEEL_RADIUS_M) * 60.0f / (2.0f * (float)M_PI);
        float rpm_right = (v_right / WHEEL_RADIUS_M) * 60.0f / (2.0f * (float)M_PI);

        // --- [디버깅 코드] 주요 제어 값들을 UART2로 출력 ---
		    int len = sprintf(dbg_buf, "T_v:%.2f, T_w:%.2f | L_rpm:%.1f, R_rpm:%.1f\r\n",
						  target_v, target_w, rpm_left, rpm_right);
		  HAL_UART_Transmit(&huart2, (uint8_t*)dbg_buf, len, 100);
		// ------------------------------------------------

        // 5) 모터 구동
        Motor_SetSpeed(0, rpm_left);
        Motor_SetSpeed(1, rpm_right);

        osDelay(20);
    }
}

/* ===========================
 *  모터 PWM 출력
 *  - 데드존 처리 & 방향 제어
 * =========================== */
void Motor_SetSpeed(int motor_id, float rpm) {
	// 디버깅 버퍼
	static char dbg_buf[128];
    // 1) 아주 작은 속도는 정지 처리(데드존)
    float abs_rpm = fabsf(rpm);
//    if (abs_rpm < MOTOR_DEADZONE_RPM) {
//        if (motor_id == 0) { // 왼쪽
//            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 620);
//            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 620);
//        } else {             // 오른쪽
//            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 620);
//            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 620);
//        }
//        return;
//    }

    // 2) 유효 구간 선형 매핑: [DEADZONE_RPM..MAX] -> [MOTOR_PWM_DEADZONE..PWM_TIMER_PERIOD]
    float span_rpm = MOTOR_MAX_RPM - MOTOR_DEADZONE_RPM;
    if (span_rpm < 1.0f) span_rpm = 1.0f; // 0으로 나눔 방지

    float norm = (abs_rpm - MOTOR_DEADZONE_RPM) / span_rpm; // 0~1
    if (norm < 0.0f) norm = 0.0f;
    if (norm > 1.0f) norm = 1.0f;

    uint32_t pwm_range = PWM_TIMER_PERIOD - MOTOR_PWM_DEADZONE;
    uint32_t pulse = MOTOR_PWM_DEADZONE + (uint32_t)(norm * pwm_range);

    if (pulse > PWM_TIMER_PERIOD) pulse = PWM_TIMER_PERIOD;

    // --- [디버깅 코드] 최종 RPM과 PWM Pulse 값을 UART2로 출력 ---
	int len = sprintf(dbg_buf, "  -> M%d: RPM=%.1f, Pulse=%u\r\n", motor_id, rpm, (unsigned int)pulse);
	HAL_UART_Transmit(&huart2, (uint8_t*)dbg_buf, len, 100);
	// -------------------------------------------------------

    // 3) 방향 출력 (배선에 따라 부호 반전 필요할 수 있음)
    if (motor_id == 0) { // 왼쪽
        if (rpm > 0) {   // 정방향
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, pulse);
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
        } else {         // 역방향
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, pulse);
        }
    } else if (motor_id == 1) { // 오른쪽
        if (rpm > 0) {
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pulse);
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
        } else {
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pulse);
        }
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == &huart3) {
        char c = (char)uart_rx_byte;

        if (c != '\n' && c != '\r') {
            if (uart_line_idx < UART_LINE_MAX - 1) {
                uart_line_buf[uart_line_idx++] = c;
            } else {
                // 오버플로우: 리셋
                uart_line_idx = 0;
            }
        } else {
            uart_line_buf[uart_line_idx] = '\0';

            // 입력이 "정수\n" 형식임이 보장됨
            int ai_angle = atoi(uart_line_buf);

            // 0~180 범위 보정
            if (ai_angle < 0) ai_angle = 0;
            if (ai_angle > 180) ai_angle = 180;

            float steering_deg = (float)ai_angle - 90.0f;      // -90 ~ +90 [deg]
            float steering_rad = deg2rad(steering_deg);        // [rad]

            MotorCommand_t cmd;
            cmd.angular_w = KP_ANG * steering_rad;             // [rad/s]
            float v_cmd = V_TARGET / (1.0f + BETA_CURVE * fabsf(steering_rad));
            cmd.linear_v = clampf(v_cmd, V_MIN_CLAMP, V_MAX_CLAMP);

            BaseType_t hpTaskWoken = pdFALSE;
            xQueueSendFromISR(motorCommandQueue, &cmd, &hpTaskWoken);
            portYIELD_FROM_ISR(hpTaskWoken);

            uart_line_idx = 0;
        }

        // 다음 바이트 수신 재개
        HAL_UART_Receive_IT(&huart3, &uart_rx_byte, 1);
    }
}


/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

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
