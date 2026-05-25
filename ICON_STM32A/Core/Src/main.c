/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : ICON STM32-A (U1) — 모터/엔코더/CAN 송신
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"

/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
CAN_HandleTypeDef hcan;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

// ── 엔코더 카운터 (현재값 / 이전값) ──────────────────────────
int32_t enc_fl = 0,  enc_fl_prev = 0;
int32_t enc_fr = 0,  enc_fr_prev = 0;

// ── RPM (실제 2개 + 가상 2개) ────────────────────────────────
float rpm_fl = 0.0f;   // 전륜좌 (실제)
float rpm_fr = 0.0f;   // 전륜우 (실제)
float rpm_rl = 0.0f;   // 후륜좌 (가상 = FL 복사)
float rpm_rr = 0.0f;   // 후륜우 (가상 = FR 복사)

// ── CAN 송신 관련 ─────────────────────────────────────────────
CAN_TxHeaderTypeDef can_tx_header;
uint8_t can_tx_data[8];
uint32_t can_tx_mailbox;

/* USER CODE END PV */

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_CAN_Init(void);
static void MX_USART3_UART_Init(void);

/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

// RPM 계산 함수 (오버플로우 처리 포함)
float Calc_RPM(int32_t current, int32_t *prev)
{
    int32_t delta = current - *prev;

    // 오버플로우 처리 (카운터 0~65535 범위)
    if (delta > 32767)  delta -= 65536;
    if (delta < -32768) delta += 65536;

    *prev = current;

    // RPM = (delta / 1496펄스) × (60초 / 0.1초)
    return ((float)delta / 1496.0f) * 600.0f;
}

// CAN 송신 함수
// 4개 RPM을 8바이트에 압축해서 ID 0x100으로 송신
void CAN_Send_RPM(void)
{
    // RPM을 int16_t로 변환 (2바이트씩, 소수점 제거)
    int16_t fl = (int16_t)rpm_fl;
    int16_t fr = (int16_t)rpm_fr;
    int16_t rl = (int16_t)rpm_rl;
    int16_t rr = (int16_t)rpm_rr;

    // 8바이트 데이터 패킹
    // [0~1]: FL RPM
    // [2~3]: FR RPM
    // [4~5]: RL RPM
    // [6~7]: RR RPM
    can_tx_data[0] = (fl >> 8) & 0xFF;  // FL 상위 바이트
    can_tx_data[1] =  fl       & 0xFF;  // FL 하위 바이트
    can_tx_data[2] = (fr >> 8) & 0xFF;
    can_tx_data[3] =  fr       & 0xFF;
    can_tx_data[4] = (rl >> 8) & 0xFF;
    can_tx_data[5] =  rl       & 0xFF;
    can_tx_data[6] = (rr >> 8) & 0xFF;
    can_tx_data[7] =  rr       & 0xFF;

    // 송신
    HAL_CAN_AddTxMessage(&hcan, &can_tx_header,
                          can_tx_data, &can_tx_mailbox);
}

/* USER CODE END 0 */

int main(void)
{
    /* USER CODE BEGIN 1 */
    /* USER CODE END 1 */

    HAL_Init();

    /* USER CODE BEGIN Init */
    /* USER CODE END Init */

    SystemClock_Config();

    /* USER CODE BEGIN SysInit */
    __HAL_AFIO_REMAP_SWJ_NOJTAG();
    __HAL_AFIO_REMAP_TIM2_PARTIAL_1();
    __HAL_AFIO_REMAP_TIM3_PARTIAL();
    /* USER CODE END SysInit */

    MX_GPIO_Init();
    MX_TIM1_Init();
    MX_TIM2_Init();
    MX_TIM3_Init();
    MX_TIM4_Init();
    MX_CAN_Init();
    MX_USART3_UART_Init();

    /* USER CODE BEGIN 2 */

    // 엔코더 타이머 시작
    HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

    // PWM 시작
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

    // FL 모터 앞으로 50%
    HAL_GPIO_WritePin(GPIOA, FL_IN1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, FL_IN2_Pin, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 3600);

    // FR 모터 앞으로 50%
    HAL_GPIO_WritePin(GPIOA, FR_IN3_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, FR_IN4_Pin, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 3600);

    // CAN 헤더 설정 (한 번만 설정)
    can_tx_header.StdId = 0x100;        // CAN ID
    can_tx_header.ExtId = 0;
    can_tx_header.RTR = CAN_RTR_DATA;   // 데이터 프레임
    can_tx_header.IDE = CAN_ID_STD;     // 표준 ID
    can_tx_header.DLC = 8;              // 데이터 길이 8바이트
    can_tx_header.TransmitGlobalTime = DISABLE;

    // CAN 시작
    HAL_CAN_Start(&hcan);

    /* USER CODE END 2 */

    /* USER CODE BEGIN WHILE */
    while (1)
    {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

        // 엔코더 읽기
        enc_fl = (int32_t)__HAL_TIM_GET_COUNTER(&htim1);
        enc_fr = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);

        // 실제 RPM 계산
        rpm_fl = Calc_RPM(enc_fl, &enc_fl_prev);
        rpm_fr = Calc_RPM(enc_fr, &enc_fr_prev);

        // 가상 RPM (실제 하드웨어 없으므로 FL/FR 복사)
        rpm_rl = rpm_fl;
        rpm_rr = rpm_fr;

        // CAN 송신 (4개 RPM → 0x100)
        CAN_Send_RPM();

        // 디버그 출력
        printf("FL:%.1f FR:%.1f RL:%.1f RR:%.1f\r\n",
                rpm_fl, rpm_fr, rpm_rl, rpm_rr);

        HAL_Delay(100);

    /* USER CODE END 3 */
    }
}

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { Error_Handler(); }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                 | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) { Error_Handler(); }
}

static void MX_CAN_Init(void)
{
    /* USER CODE BEGIN CAN_Init 0 */
    /* USER CODE END CAN_Init 0 */

    /* USER CODE BEGIN CAN_Init 1 */
    /* USER CODE END CAN_Init 1 */

    hcan.Instance = CAN1;
    hcan.Init.Prescaler = 4;
    hcan.Init.Mode = CAN_MODE_NORMAL;
    hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
    hcan.Init.TimeSeg1 = CAN_BS1_15TQ;
    hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
    hcan.Init.TimeTriggeredMode = DISABLE;
    hcan.Init.AutoBusOff = DISABLE;
    hcan.Init.AutoWakeUp = DISABLE;
    hcan.Init.AutoRetransmission = ENABLE;
    hcan.Init.ReceiveFifoLocked = DISABLE;
    hcan.Init.TransmitFifoPriority = DISABLE;
    if (HAL_CAN_Init(&hcan) != HAL_OK) { Error_Handler(); }

    /* USER CODE BEGIN CAN_Init 2 */
    /* USER CODE END CAN_Init 2 */
}

static void MX_TIM1_Init(void)
{
    /* USER CODE BEGIN TIM1_Init 0 */
    /* USER CODE END TIM1_Init 0 */

    TIM_Encoder_InitTypeDef sConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    /* USER CODE BEGIN TIM1_Init 1 */
    /* USER CODE END TIM1_Init 1 */

    htim1.Instance = TIM1;
    htim1.Init.Prescaler = 0;
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim1.Init.Period = 65535;
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
    sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
    sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC1Filter = 0;
    sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
    sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC2Filter = 0;
    if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK) { Error_Handler(); }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK) { Error_Handler(); }

    /* USER CODE BEGIN TIM1_Init 2 */
    /* USER CODE END TIM1_Init 2 */
}

static void MX_TIM2_Init(void)
{
    /* USER CODE BEGIN TIM2_Init 0 */
    /* USER CODE END TIM2_Init 0 */

    TIM_Encoder_InitTypeDef sConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    /* USER CODE BEGIN TIM2_Init 1 */
    /* USER CODE END TIM2_Init 1 */

    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 0;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 65535;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
    sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
    sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC1Filter = 0;
    sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
    sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC2Filter = 0;
    if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK) { Error_Handler(); }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) { Error_Handler(); }

    /* USER CODE BEGIN TIM2_Init 2 */
    /* USER CODE END TIM2_Init 2 */
}

static void MX_TIM3_Init(void)
{
    /* USER CODE BEGIN TIM3_Init 0 */
    /* USER CODE END TIM3_Init 0 */

    TIM_Encoder_InitTypeDef sConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    /* USER CODE BEGIN TIM3_Init 1 */
    /* USER CODE END TIM3_Init 1 */

    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 0;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 65535;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
    sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
    sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC1Filter = 0;
    sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
    sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC2Filter = 0;
    if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK) { Error_Handler(); }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) { Error_Handler(); }

    /* USER CODE BEGIN TIM3_Init 2 */
    /* USER CODE END TIM3_Init 2 */
}

static void MX_TIM4_Init(void)
{
    /* USER CODE BEGIN TIM4_Init 0 */
    /* USER CODE END TIM4_Init 0 */

    TIM_Encoder_InitTypeDef sConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};

    /* USER CODE BEGIN TIM4_Init 1 */
    /* USER CODE END TIM4_Init 1 */

    htim4.Instance = TIM4;
    htim4.Init.Prescaler = 0;
    htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim4.Init.Period = 3599;
    htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
    sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
    sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC1Filter = 0;
    sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
    sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC2Filter = 0;
    if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK) { Error_Handler(); }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK) { Error_Handler(); }
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK) { Error_Handler(); }
    if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK) { Error_Handler(); }

    /* USER CODE BEGIN TIM4_Init 2 */
    /* USER CODE END TIM4_Init 2 */
}

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
    if (HAL_UART_Init(&huart3) != HAL_OK) { Error_Handler(); }

    /* USER CODE BEGIN USART3_Init 2 */
    /* USER CODE END USART3_Init 2 */
}

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* USER CODE BEGIN MX_GPIO_Init_1 */
    /* USER CODE END MX_GPIO_Init_1 */

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    HAL_GPIO_WritePin(GPIOA, FL_IN1_Pin | FL_IN2_Pin | FR_IN3_Pin | FR_IN4_Pin
                            | RL_IN1_Pin | RL_IN2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, RR_IN3_Pin | RR_IN4_Pin, GPIO_PIN_RESET);

    GPIO_InitStruct.Pin = GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = FL_IN1_Pin | FL_IN2_Pin | FR_IN3_Pin | FR_IN4_Pin
                         | RL_IN1_Pin | RL_IN2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = RR_IN3_Pin | RR_IN4_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* USER CODE BEGIN MX_GPIO_Init_2 */
    /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
int __io_putchar(int ch)
{
    HAL_UART_Transmit(&huart3, (uint8_t*)&ch, 1, HAL_MAX_DELAY);
    return ch;
}
/* USER CODE END 4 */

void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    __disable_irq();
    while (1) {}
    /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* USER CODE END 6 */
}
#endif
