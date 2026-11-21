/* USER CODE BEGIN Header */
/**
 * 3-Phase Induction Motor Predictive Maintenance & Safety Contactor
 * Target: STM32F103C6Tx (Proteus 8 Co-Simulation)
 *
 * Acquisition: TIM3 (600 Hz) + Single-Channel ADC1 DMA Interleaved Capture
 *   - Steps PA0 (Ia) -> PA1 (Ib) -> PA2 (Ic) at 200 Hz/channel (exact 90 deg @ 50 Hz)
 *   - Keeps ADC1 powered ON (ADON=1) between conversions so SQR3 switches cleanly
 *
 * Output: Compact JSON lines (< 75 chars) @ 9600 baud (1 line per update)
 * Control: Send 'r' or 'R' in Virtual Terminal to reset tripped contactor
 */
/* USER CODE END Header */

#include "main.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdint.h>

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;
TIM_HandleTypeDef htim3;
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
#define CONTACTOR_PORT      GPIOB
#define CONTACTOR_PIN       GPIO_PIN_0    /* LOW = Contactor Closed (Run), HIGH = Open (Trip) */
#define LED_PORT            GPIOC
#define LED_PIN             GPIO_PIN_13   /* Blue Pill onboard LED (Active LOW) */

/* Sensor Calibration: ACS712ELCTR-30A-T + 10k/20k voltage divider */
#define VREF                3.3f
#define ADC_MAX             4095.0f
#define ACS712_SENSITIVITY  0.0294f       /* V_rms per A_rms at ADC pin */

/* 20 samples/ch @ 200 Hz/ch (600 Hz TIM3) = 100.0 ms exact (5 cycles @ 50 Hz) */
#define NUM_CHANNELS        3             /* 0: PA0 (Ia), 1: PA1 (Ib), 2: PA2 (Ic) */
#define SAMPLES_PER_WINDOW  20
#define OBSERVE_COUNT_TRIP  3             /* 3 x 500 ms = 1.5 s observation before trip */

static const uint32_t adc_channels[NUM_CHANNELS] = {
  ADC_CHANNEL_0,
  ADC_CHANNEL_1,
  ADC_CHANNEL_2
};

static uint16_t dma_sample = 0;
static volatile float wave_buf[NUM_CHANNELS][SAMPLES_PER_WINDOW];
static volatile uint8_t cur_ch = 0;
static volatile uint16_t sample_idx = 0;
static volatile uint8_t window_ready = 0;

/* Interrupt-driven UART RX Ring Buffer */
#define RX_BUF_SIZE         16
static volatile uint8_t rxBuf[RX_BUF_SIZE];
static volatile uint8_t rxHead = 0;
static volatile uint8_t rxTail = 0;
static volatile uint8_t rxByte = 0;

/* Fault & Protection States */
typedef enum {
  FAULT_NONE = 0,
  FAULT_NO_SUPPLY,
  FAULT_SINGLE_PHASING,
  FAULT_OVERLOAD,
  FAULT_BEARING_ECCENTRICITY,
  FAULT_BROKEN_ROTOR_BAR
} FaultType_t;

typedef enum {
  SYS_HEALTHY = 0,
  SYS_OBSERVING,
  SYS_TRIPPED
} SystemState_t;

static SystemState_t sysState = SYS_HEALTHY;
static FaultType_t activeFault = FAULT_NONE;
static FaultType_t pendingFault = FAULT_NONE;
static uint8_t observeCounter = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);

/* USER CODE BEGIN PFP */
static void SetContactorTrip(uint8_t trip);
static void ComputeAllRMS(float *ia, float *ib, float *ic);
static FaultType_t ClassifyFromSensors(float ia, float ib, float ic);
static const char* FaultName(FaultType_t f);
static const char* StateName(SystemState_t s);
static void PublishTelemetry(float ia, float ib, float ic, FaultType_t fault);
static void UART_Print(const char *msg);
/* USER CODE END PFP */

int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();

  /* USER CODE BEGIN 2 */
  SetContactorTrip(0);

  UART_Print("\r\n{\"event\":\"BOOT\",\"msg\":\"Monitor Ready | Send r to RESET\"}\r\n");

  HAL_UART_Receive_IT(&huart1, (uint8_t *)&rxByte, 1);
  HAL_TIM_Base_Start_IT(&htim3);
  /* USER CODE END 2 */

  uint8_t printDivider = 0;

  while (1)
  {
    /* USER CODE BEGIN WHILE */
    /* 1. Handle UART reset commands */
    while (rxTail != rxHead)
    {
      uint8_t cmd = rxBuf[rxTail];
      rxTail = (rxTail + 1) % RX_BUF_SIZE;

      if (cmd == 'r' || cmd == 'R')
      {
        SetContactorTrip(0);
        sysState = SYS_HEALTHY;
        activeFault = FAULT_NONE;
        pendingFault = FAULT_NONE;
        observeCounter = 0;
        UART_Print("{\"event\":\"RESET\",\"c\":\"CLOSED\",\"s\":\"HEALTHY\"}\r\n");
      }
    }

    /* 2. Process completed 100 ms (5-cycle) 3-phase window */
    if (window_ready)
    {
      float ia, ib, ic;
      ComputeAllRMS(&ia, &ib, &ic);

      cur_ch = 0;
      sample_idx = 0;
      window_ready = 0;

      printDivider++;

      /* Evaluate protection & publish 1 clean line every 500 ms */
      if (printDivider >= 5)
      {
        printDivider = 0;

        if (sysState == SYS_TRIPPED)
        {
          PublishTelemetry(ia, ib, ic, activeFault);
        }
        else
        {
          FaultType_t detected = ClassifyFromSensors(ia, ib, ic);

          if (detected == FAULT_NONE || detected == FAULT_NO_SUPPLY)
          {
            sysState = SYS_HEALTHY;
            pendingFault = FAULT_NONE;
            observeCounter = 0;
            PublishTelemetry(ia, ib, ic, detected);
          }
          else
          {
            if (detected == pendingFault)
            {
              observeCounter++;
            }
            else
            {
              pendingFault = detected;
              observeCounter = 1;
            }

            sysState = SYS_OBSERVING;
            PublishTelemetry(ia, ib, ic, pendingFault);

            if (observeCounter >= OBSERVE_COUNT_TRIP)
            {
              activeFault = pendingFault;
              sysState = SYS_TRIPPED;
              SetContactorTrip(1); /* OPEN 3-PHASE CONTACTOR */

              char alert[80];
              snprintf(alert, sizeof(alert),
                       "{\"event\":\"TRIP\",\"f\":\"%s\",\"c\":\"OPEN\"}\r\n",
                       FaultName(activeFault));
              UART_Print(alert);
            }
          }
        }
      }
    }
    /* USER CODE END WHILE */
  }
}

/* USER CODE BEGIN 4 */
static void SetContactorTrip(uint8_t trip)
{
  HAL_GPIO_WritePin(CONTACTOR_PORT, CONTACTOR_PIN, trip ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_PORT, LED_PIN, trip ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

static void ComputeAllRMS(float *ia, float *ib, float *ic)
{
  float rms_volts[NUM_CHANNELS];

  for (int ch = 0; ch < NUM_CHANNELS; ch++)
  {
    double sum = 0.0;
    for (int i = 0; i < SAMPLES_PER_WINDOW; i++)
    {
      sum += (double)wave_buf[ch][i];
    }
    float mean = (float)(sum / SAMPLES_PER_WINDOW);

    double sumSq = 0.0;
    for (int i = 0; i < SAMPLES_PER_WINDOW; i++)
    {
      float dev = wave_buf[ch][i] - mean;
      sumSq += (double)dev * dev;
    }
    rms_volts[ch] = sqrtf((float)(sumSq / SAMPLES_PER_WINDOW));
  }

  *ia = rms_volts[0] / ACS712_SENSITIVITY;
  *ib = rms_volts[1] / ACS712_SENSITIVITY;
  *ic = rms_volts[2] / ACS712_SENSITIVITY;
}

static FaultType_t ClassifyFromSensors(float ia, float ib, float ic)
{
  float i_ref = 0.5f * (ib + ic);

  /* 0. Contactor open or main supply off */
  if (i_ref < 0.50f && ia < 0.50f)
  {
    return FAULT_NO_SUPPLY;
  }

  /* Fallback if Phase B/C supplies are unpowered while Phase A runs */
  if (i_ref < 0.50f)
  {
    if (ia > 20.5f) return FAULT_OVERLOAD;
    if (ia > 11.0f) return FAULT_BEARING_ECCENTRICITY;
    if (ia < 5.75f) return FAULT_BROKEN_ROTOR_BAR;
    return FAULT_NONE;
  }

  float ratio_a = ia / i_ref;

  /* 1. Single-Phasing: Phase A drops below 20% of healthy phases */
  if (ratio_a < 0.20f && i_ref > 2.00f)
  {
    return FAULT_SINGLE_PHASING;
  }

  /* 2. 3-Phase Overload: all 3 phases jump together above 15 A (balanced) */
  if (i_ref > 15.0f && ratio_a > 0.85f && ratio_a < 1.15f)
  {
    return FAULT_OVERLOAD;
  }

  /* 3. Bearing Air-Gap Eccentricity: Phase A spikes more than 35% above B & C */
  if (ratio_a > 1.35f)
  {
    return FAULT_BEARING_ECCENTRICITY;
  }

  /* 4. Broken Rotor Bar: Phase A drops below 91% of B & C (e.g. 5.30 A vs 6.30 A) */
  if (ratio_a >= 0.20f && ratio_a < 0.91f)
  {
    return FAULT_BROKEN_ROTOR_BAR;
  }

  return FAULT_NONE;
}

static void PublishTelemetry(float ia, float ib, float ic, FaultType_t fault)
{
  int ia_mA = (int)(ia * 1000.0f + 0.5f);
  int ib_mA = (int)(ib * 1000.0f + 0.5f);
  int ic_mA = (int)(ic * 1000.0f + 0.5f);

  char json[84];
  snprintf(json, sizeof(json),
           "{\"ia\":%d.%02d,\"ib\":%d.%02d,\"ic\":%d.%02d,\"s\":\"%s\",\"f\":\"%s\",\"c\":\"%s\"}\r\n",
           ia_mA / 1000, (ia_mA % 1000) / 10,
           ib_mA / 1000, (ib_mA % 1000) / 10,
           ic_mA / 1000, (ic_mA % 1000) / 10,
           StateName(sysState),
           FaultName(fault),
           (sysState == SYS_TRIPPED) ? "OPEN" : "CLOSED");
  UART_Print(json);
}

static const char* FaultName(FaultType_t f)
{
  switch (f)
  {
    case FAULT_NO_SUPPLY:            return "NO_SUPPLY";
    case FAULT_SINGLE_PHASING:       return "SINGLE_PHASING";
    case FAULT_OVERLOAD:             return "OVERLOAD_3PH";
    case FAULT_BEARING_ECCENTRICITY: return "ECCENTRICITY";
    case FAULT_BROKEN_ROTOR_BAR:     return "ROTOR_BAR";
    default:                         return "NONE";
  }
}

static const char* StateName(SystemState_t s)
{
  switch (s)
  {
    case SYS_OBSERVING: return "OBSERVING";
    case SYS_TRIPPED:   return "TRIPPED";
    default:            return "HEALTHY";
  }
}

static void UART_Print(const char *msg)
{
  HAL_UART_Transmit(&huart1, (uint8_t *)msg, strlen(msg), 100);
}

/* TIM3 triggers at 600 Hz (1.67 ms) -> switches channel while ADON stays ON */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM3 && !window_ready)
  {
    uint32_t ch = adc_channels[cur_ch];
    hadc1.State = HAL_ADC_STATE_READY;
    MODIFY_REG(hadc1.Instance->SQR3, ADC_SQR3_SQ1, ch);
    MODIFY_REG(hadc1.Instance->SMPR2, ADC_SMPR2_SMP0 << (3U * ch),
               ADC_SAMPLETIME_55CYCLES_5 << (3U * ch));

    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)&dma_sample, 1);
  }
}

/* DMA finishes single-channel transfer -> resets DMA without turning ADC off */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == ADC1)
  {
    CLEAR_BIT(hadc1.Instance->CR2, ADC_CR2_DMA);
    HAL_DMA_Abort(&hdma_adc1);
    hadc1.State = HAL_ADC_STATE_READY;

    if (!window_ready)
    {
      uint8_t ch = cur_ch;
      uint16_t idx = sample_idx;

      if (idx < SAMPLES_PER_WINDOW)
      {
    	  uint8_t target_ch = (ch + 2U) % NUM_CHANNELS;
    	  wave_buf[target_ch][idx] = ((float)dma_sample / ADC_MAX) * VREF;
      }

      ch++;
      if (ch >= NUM_CHANNELS)
      {
        ch = 0;
        idx++;
        if (idx >= SAMPLES_PER_WINDOW)
        {
          window_ready = 1;
        }
      }

      cur_ch = ch;
      sample_idx = idx;
    }
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    uint8_t next = (rxHead + 1) % RX_BUF_SIZE;
    if (next != rxTail)
    {
      rxBuf[rxHead] = rxByte;
      rxHead = next;
    }
    HAL_UART_Receive_IT(&huart1, (uint8_t *)&rxByte, 1);
  }
}
/* USER CODE END 4 */

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL8;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                              | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) Error_Handler();

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) Error_Handler();
}

static void MX_ADC1_Init(void)
{
  __HAL_RCC_ADC1_CLK_ENABLE();

  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK) Error_Handler();
}

static void MX_TIM3_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  __HAL_RCC_TIM3_CLK_ENABLE();

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 63;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1665;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK) Error_Handler();

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) Error_Handler();

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) Error_Handler();

  HAL_NVIC_SetPriority(TIM3_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(TIM3_IRQn);
}

static void MX_DMA_Init(void)
{
  __HAL_RCC_DMA1_CLK_ENABLE();

  hdma_adc1.Instance = DMA1_Channel1;
  hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
  hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_adc1.Init.MemInc = DMA_MINC_DISABLE;
  hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
  hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
  hdma_adc1.Init.Mode = DMA_NORMAL;
  hdma_adc1.Init.Priority = DMA_PRIORITY_HIGH;
  if (HAL_DMA_Init(&hdma_adc1) != HAL_OK) Error_Handler();

  __HAL_LINKDMA(&hadc1, DMA_Handle, hdma_adc1);

  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
}

static void MX_USART1_UART_Init(void)
{
  __HAL_RCC_USART1_CLK_ENABLE();

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK) Error_Handler();

  HAL_NVIC_SetPriority(USART1_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_AFIO_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  HAL_GPIO_WritePin(CONTACTOR_PORT, CONTACTOR_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);

  GPIO_InitStruct.Pin = CONTACTOR_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CONTACTOR_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LED_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void Error_Handler(void)
{
  __disable_irq();
  while (1) {}
}
