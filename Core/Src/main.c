/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "HCSR04.h"
#include "defines.h"
#include "lteiot9.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define HCSR04_SENSOR1 0
#define HCSR04_SENSOR2 1

#define NUM_SAMPLES 8

#define COUNTER_PERIOD 65535

// #define NUM_SAMPLES 3
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim15;
TIM_HandleTypeDef htim16;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

uint16_t TRIG_Ticks = 0;

float US_Dist_1 = 0.0;
float US_Dist_2 = 0.0;

int readings1[NUM_SAMPLES];
int readings2[NUM_SAMPLES];
int index1 = 0;
int index2 = 0;

lteiot9_t lteiot9;

char app_buf[PROCESS_BUFFER_SIZE] = {0};
char gnss_buf[PROCESS_BUFFER_SIZE] = {0};
char response[PROCESS_BUFFER_SIZE] = {0};

uint32_t gnss_buf_len = 0;
uint32_t gnss_buf_cnt = 0;

uint32_t app_buf_len = 0;
uint32_t app_buf_cnt = 0;

char latitude_data[30] = {0};
char longitude_data[30] = {0};
char altitude_data[30] = {0};

int app_error_flag;
int last_error_flag;

char uid_str[25];
char json_data[128];
char last_json_data[128];
char json_data_flag[64];

uint8_t lteiot9_status = 0;
uint8_t init_status = 0;
uint8_t net_config_status = 0;
uint8_t net_conn_status = 0;
uint8_t setup_http_flags_status = 0;
uint8_t post_http_flags_status = 0;
uint8_t gnss_config_status = 0;
uint8_t gnss_data_status = 0;
uint8_t setup_http_gnss_status = 0;
uint8_t post_http_gnss_status = 0;

uint8_t gps_status = 0;
uint8_t gps_status_old = 0;
uint8_t ping_status = 0;
uint32_t wait_value = DEFAULT_WAIT;
int http_fail_counter = 0;

int dns_idx = 0;
char primary_dns[3][8] = {"0.0.0.0", "8.8.8.8", "1.1.1.1"};
char secondary_dns[3][8] = {"0.0.0.0", "8.8.4.4", "1.0.0.1"};
int netConfigCounter = 0;

uint32_t ping_timer_counter = 0;

int gnssDataFunctionCounter = 0;
int gnssDataFunctionCounter_MaxVal = FIRST_TIME_GNSS;
int gnssPostFlag = -1;

uint8_t location_send_flag = 0;

int gnss_parse_flag = -1;
int gnss_post_flag = -1;
uint32_t initFunctionCounter = 0;
uint32_t initFunctionCounter_MaxVal = 20;
uint32_t netConfigFunctionCounter = 0;
uint32_t netConfigFunctionCounter_MaxVal = 3;

uint32_t netConnFunctionCounter = 0;
uint32_t netConnFunctionCounter_MaxVal = 3;
uint32_t setupHttpFlagsFunctionCounter = 0;
uint32_t setupHttpFlagsFunctionCounter_MaxVal = 5;
uint32_t postHttpFlagsFunctionCounter = 0;
uint32_t postHttpFlagsFunctionCounter_MaxVal = 5;
uint32_t setupHttpGnssFunctionCounter = 0;
uint32_t setupHttpGnssFunctionCounter_MaxVal = 5;
uint32_t postHttpGnssFunctionCounter = 0;
uint32_t postHttpGnssFunctionCounter_MaxVal = 5;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM15_Init(void);
static void MX_TIM16_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
  HCSR04_TMR_IC_ISR(htim);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  HCSR04_TMR_OVF_ISR(htim);
}

void SysTick_CallBack(void) {
  TRIG_Ticks++;

  // Every 15msec
  if (TRIG_Ticks >= 15) {
    HCSR04_Trigger(HCSR04_SENSOR1);
    HCSR04_Trigger(HCSR04_SENSOR2);
    TRIG_Ticks = 0;
  }
}

void set_vibration_1(float distance);

void set_vibration_2(float distance);

void ping(uint32_t time, uint32_t delay);

void ping_start();

void ping_routine();

void reset_status();

void lteiot9_clear_app_buf();

void lteiot9_clear_gnss_buf();

size_t filledCharacters(char buff[]);

uint32_t lteiot9_process_gnss();

uint32_t lteiot9_process();

uint32_t lteiot9_rsp_check();

void parse_flags_json_response();

float parse_coordinate(char *data);

void format_coordinate_json_data(float latitude, float longitude, char *json_data);

int32_t lteiot9_element_parser(char *cmd, uint8_t element, char *element_data);

void lteiot9_error_check(uint32_t error_flag);

int lteiot9_gnss_data_parser();

int lteiot9_gnss_coordinates();

void gnssDataFunction();

void initFunction();

void netConfigFunction();

void netConnFunction();

void setupHttpFlagsFunction();

void postHttpFlagsFunction();

void gnssConfigFunction();

void setupHttpGnssFunction();

void postHttpGnssFunction();

void lteiot9_task() {
  switch (lteiot9_status) {
    case LTEIOT9_INIT: {
      initFunction();
      break;
    }
    case LTEIOT9_NET_CONFIG: {
      netConfigFunction();
      break;
    }
    case LTEIOT9_NET_CONN: {
      netConnFunction();
      break;
    }
    case LTEIOT9_HTTP_FLAGS_SETUP: {
      setupHttpFlagsFunction();
      break;
    }
    case LTEIOT9_HTTP_FLAGS_POST: {
      postHttpFlagsFunction();
      break;
    }
    case LTEIOT9_GNSS_CONFIG: {
      gnssConfigFunction();
      break;
    }
    case LTEIOT9_GNSS_DATA: {
      gnssDataFunction();
      break;
    }
    case LTEIOT9_HTTP_GNSS_SETUP: {
      setupHttpGnssFunction();
      break;
    }
    case LTEIOT9_HTTP_GNSS_POST: {
      postHttpGnssFunction();
      break;
    }
    default: {
      // Handle unknown status
      break;
    }
  }

  if (http_fail_counter > 2) {
    dns_idx = (dns_idx + 1) % 3;
    lteiot9_status = LTEIOT9_NET_CONFIG;
    http_fail_counter = 0;
    reset_status();
  }
}

// Function to update rolling average for sensor 1
int UpdateRollingAverage1(int newReading) {
  readings1[index1] = newReading;
  index1 = (index1 + 1) % NUM_SAMPLES;

  int total = 0;
  for (int i = 0; i < NUM_SAMPLES; i++) {
    total += readings1[i];
  }
  return total / NUM_SAMPLES;
}

// Function to update rolling average for sensor 2
int UpdateRollingAverage2(int newReading) {
  readings2[index2] = newReading;
  index2 = (index2 + 1) % NUM_SAMPLES;

  int total = 0;
  for (int i = 0; i < NUM_SAMPLES; i++) {
    total += readings2[i];
  }
  return total / NUM_SAMPLES;
}

void us_sensor_task() {
  US_Dist_1 = HCSR04_Read(HCSR04_SENSOR1);
  US_Dist_2 = HCSR04_Read(HCSR04_SENSOR2);

  US_Dist_1 = UpdateRollingAverage1(US_Dist_1);
  US_Dist_2 = UpdateRollingAverage2(US_Dist_2);

  set_vibration_1(US_Dist_1);
  set_vibration_2(US_Dist_2);
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick.
   */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM15_Init();
  MX_TIM16_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
  HCSR04_Init(HCSR04_SENSOR1, &htim15);
  HCSR04_Init(HCSR04_SENSOR2, &htim15);

  HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);
  HAL_GPIO_WritePin(LTEIOT9_CS_GPIO_Port, LTEIOT9_CS_Pin, GPIO_PIN_SET);

  uint32_t uid[3];

  uid[0] = HAL_GetUIDw0();
  uid[1] = HAL_GetUIDw1();
  uid[2] = HAL_GetUIDw2();

  // Convert each uint32_t to its string representation
  snprintf(uid_str, sizeof(uid_str), "%08lX%08lX%08lX", uid[0], uid[1], uid[2]);
  lteiot9_init(&lteiot9, &huart2);

  ping_start();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    uint32_t tick = HAL_GetTick();

    while (HAL_GetTick() - tick < wait_value) {
      us_sensor_task();
    }

    wait_value = DEFAULT_WAIT;

    lteiot9_task();

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
   */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
    Error_Handler();
  }
}

/**
 * @brief TIM15 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM15_Init(void) {
  /* USER CODE BEGIN TIM15_Init 0 */

  /* USER CODE END TIM15_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM15_Init 1 */

  /* USER CODE END TIM15_Init 1 */
  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 72 - 1;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 0xffff - 1;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim15) != HAL_OK) {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim15, &sClockSourceConfig) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim15) != HAL_OK) {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) !=
      HAL_OK) {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim15, &sConfigIC, TIM_CHANNEL_1) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim15, &sConfigIC, TIM_CHANNEL_2) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM15_Init 2 */

  /* USER CODE END TIM15_Init 2 */
}

/**
 * @brief TIM16 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM16_Init(void) {
  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 0;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 65535;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim16) != HAL_OK) {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim16, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim16, &sBreakDeadTimeConfig) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */
  HAL_TIM_MspPostInit(&htim16);
}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {
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
  huart2.Init.HwFlowCtl = UART_HWCONTROL_RTS_CTS;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK) {
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
static void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6,
                    GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(
      GPIOB, GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_5 | GPIO_PIN_7 | GPIO_PIN_9,
      GPIO_PIN_RESET);

  /*Configure GPIO pins : PA4 PA5 PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB13 PB7 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_7 | GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void set_vibration_1(float distance) {
  if (distance < 100) {
    for (int i = 0; i < 50; i++) {
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
      HAL_Delay(2);
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
      HAL_Delay(1);
    }
  } else {
    HAL_GPIO_WritePin(VIBRATION_2_Port, VIBRATION_2_Pin, GPIO_PIN_RESET);
  }
}

void set_vibration_2(float distance) {
  if (distance > 0 && distance < 140) {
    TIM16->CCR1 = 58981;
    HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);
  } else if (distance >= 140 && distance < 240) {
    TIM16->CCR1 = 26214;
    HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);
  } else {
    HAL_TIM_PWM_Stop(&htim16, TIM_CHANNEL_1);
  }
}

void ping(uint32_t time, uint32_t delay) {
  for (int i = 0; i < time; i++) {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
    HAL_Delay(delay);  // wait for 1ms
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
    HAL_Delay(delay);  // wait for 1ms
  }
}

void ping_start() {
  ping(10, 1);
  HAL_Delay(50);
}

void ping_routine() {
  for (int i = 0; i < 3; i++) {
    ping(100, 1);
    HAL_Delay(100);
    ping(100, 1);
    HAL_Delay(100);
    ping(100, 1);
    HAL_Delay(1000);
  }
}

void reset_status() {
  init_status = 0;
  net_config_status = 0;
  net_conn_status = 0;
  setup_http_flags_status = 0;
  post_http_flags_status = 0;
  gnss_config_status = 0;
  gnss_data_status = 0;
  setup_http_gnss_status = 0;
  post_http_gnss_status = 0;
}

void lteiot9_clear_app_buf() {
  memset(app_buf, 0, PROCESS_BUFFER_SIZE);
  app_buf_len = 0;
  app_buf_cnt = 0;
}

void lteiot9_clear_gnss_buf() {
  memset(gnss_buf, 0, PROCESS_BUFFER_SIZE);
  gnss_buf_len = 0;
  gnss_buf_cnt = 0;
}

size_t filledCharacters(char buff[]) {
  size_t count = 0;

  for (size_t i = 0; i < RX_BUFFER_SIZE; i++) {
    if (buff[i] != '\0') {
      count++;
    }
  }

  if (count > 0) {
    size_t leadingNulls = 0;
    while (buff[leadingNulls] == '\0') {
      leadingNulls++;
    }

    // Move non-null characters to the beginning of the buffer
    for (size_t i = 0; i + leadingNulls < RX_BUFFER_SIZE; i++) {
      buff[i] = buff[i + leadingNulls];
    }

    // Fill the remaining part with null characters
    for (size_t i = RX_BUFFER_SIZE - leadingNulls; i < RX_BUFFER_SIZE; i++) {
      buff[i] = '\0';
    }
  }

  return count;
}

uint32_t lteiot9_process_gnss() {
  uint32_t return_flag = LTEIOT9_ERROR;
  uint32_t rx_size = 0;
  char rx_buff[RX_BUFFER_SIZE] = {0};

  HAL_UART_Receive(lteiot9.uart, (uint8_t *)rx_buff, RX_BUFFER_SIZE, 70);

  rx_size = filledCharacters(rx_buff);

  if (rx_size > 0) {
    uint32_t buf_cnt = 0;
    return_flag = LTEIOT9_OK;

    if (gnss_buf_len + rx_size >= RX_BUFFER_SIZE) {
      lteiot9_clear_gnss_buf();
      return_flag = LTEIOT9_ERROR_OVERFLOW;
    } else {
      buf_cnt = gnss_buf_len;

      gnss_buf_len += rx_size;
    }

    for (uint32_t rx_cnt = 0; rx_cnt < rx_size; rx_cnt++) {
      if (rx_buff[rx_cnt] != 0) {
        gnss_buf[(buf_cnt + rx_cnt)] = rx_buff[rx_cnt];
      } else {
        gnss_buf_len--;
      }
    }
  }

  return return_flag;
}

uint32_t lteiot9_process() {
  uint32_t return_flag = LTEIOT9_ERROR;
  uint32_t rx_size = 0;
  char rx_buff[RX_BUFFER_SIZE] = {0};

  HAL_UART_Receive(lteiot9.uart, (uint8_t *)rx_buff, RX_BUFFER_SIZE, 70);

  rx_size = filledCharacters(rx_buff);

  if (rx_size > 0) {
    uint32_t buf_cnt = 0;
    return_flag = LTEIOT9_OK;

    if (app_buf_len + rx_size >= RX_BUFFER_SIZE) {
      lteiot9_clear_app_buf();
      return_flag = LTEIOT9_ERROR_OVERFLOW;
    } else {
      buf_cnt = app_buf_len;

      app_buf_len += rx_size;
    }

    for (uint32_t rx_cnt = 0; rx_cnt < rx_size; rx_cnt++) {
      if (rx_buff[rx_cnt] != 0) {
        app_buf[(buf_cnt + rx_cnt)] = rx_buff[rx_cnt];
      } else {
        app_buf_len--;
      }
    }
  }

  return return_flag;
}

uint32_t lteiot9_rsp_check() {
  uint32_t timeout_cnt = 0;
  uint32_t timeout = 1;

  while (timeout_cnt < timeout) {
    lteiot9_clear_app_buf();
    lteiot9_process();
    if (strstr(app_buf, LTEIOT9_RSP_OK) != NULL) {
      return LTEIOT9_OK;
    }
    timeout_cnt++;
    HAL_Delay(1);
  }

  return LTEIOT9_ERROR;
}

void parse_flags_json_response() {
  char *ptr = response;

  // Find gps_status
  ptr = strstr(ptr, "\"gps_status\":");
  if (ptr != NULL) {
    ptr += strlen("\"gps_status\":");
    if (*ptr == 't') {
      gps_status = 1;
    } else {
      gps_status = 0;
    }
  } else {
    gps_status = 0;
  }

  // Find ping_status
  ptr = strstr(ptr, "\"ping_status\":");
  if (ptr != NULL) {
    ptr += strlen("\"ping_status\":");
    if (*ptr == 't') {
      ping_status = 1;
    } else {
      ping_status = 0;
    }
  } else {
    ping_status = 0;
  }
}

float parse_coordinate(char *data) {
  char *first_half;
  char *second_half;
  char last_two[3];  // To store the last two characters

  // Splitting the string at the decimal point
  first_half = strtok(data, ".");
  second_half = strtok(NULL, ".");

  // Get the last two characters from the first half
  int length = strlen(first_half);
  if (length >= 2) {
    strncpy(last_two, first_half + length - 2, 2);
    last_two[2] = '\0';  // Null-terminate the string
  } else {
    return 1;
  }

  // Create a new second half combining last_two and the original second_half
  char new_second_half[30];
  sprintf(new_second_half, "%s.%s", last_two, second_half);

  // Create a new first half without the last two characters
  char new_first_half[30];
  strncpy(new_first_half, first_half, length - 2);
  new_first_half[length - 2] = '\0';  // Null-terminate the string

  // Check if the new first half starts with '0', then replace it with '-'

  int factor = 1;
  if (new_first_half[0] == '0') {
    factor = -1;
  }
  double final_first_half = atof(new_first_half);
  double final_second_half = atof(new_second_half);

  double decimal = factor * (final_first_half + (final_second_half / 60.0));

  return decimal;
}

void format_coordinate_json_data(float latitude, float longitude,
                                 char *json_data) {
  char lat_str[20];
  char lon_str[20];

  // Convert floats to strings
  sprintf(lat_str, "%.6f", latitude);
  sprintf(lon_str, "%.6f", longitude);

  // Create JSON string
  sprintf(json_data,
          "{\\\"id\\\":\\\"%s\\\",\\\"latitude\\\":\\\"%s\\\","
          "\\\"longitude\\\":\\\"%s\\\"}",
          uid_str, lat_str, lon_str);

  strcpy(last_json_data, json_data);
}

int32_t lteiot9_element_parser(char *cmd, uint8_t element, char *element_data) {
  int32_t ret_flag = 0;

  if (strstr(gnss_buf, cmd) != NULL) {
    uint8_t element_cnt = 0;
    char data_buf[30] = {0};
    uint8_t data_cnt = 0;
    char *gngga_ptr;

    gngga_ptr = strstr(gnss_buf, cmd);

    while (strchr(gngga_ptr, LTEIOT9_GNSS_START) == NULL) {
      lteiot9_process_gnss();
    }

    for (;;) {
      if (*gngga_ptr == LTEIOT9_GNSS_START) {
        ret_flag = -2;
        break;
      }

      if (*gngga_ptr == LTEIOT9_GNSS_SEPARATOR) {
        if (element_cnt == element) {
          if (data_cnt == 0) {
            ret_flag = -2;
          }
          strcpy(element_data, data_buf);
          break;
        }
        element_cnt++;
      }

      if ((element == element_cnt) && (*gngga_ptr != LTEIOT9_GNSS_SEPARATOR)) {
        data_buf[data_cnt] = *gngga_ptr;
        data_cnt++;

        if (data_cnt >= 30) {
          ret_flag = -3;
          break;
        }
      }

      gngga_ptr++;
    }
  } else {
    ret_flag = -1;
  }

  return ret_flag;
}

void lteiot9_error_check(uint32_t error_flag) {
  if ((error_flag != 0) && (error_flag != -1)) {
    switch (error_flag) {
      case -2:
        // overflow
        break;
      case -3:
        // timeout
        break;
      default:
        break;
    }
  }
}

int lteiot9_gnss_data_parser() {
  int retval = -1;

  uint32_t error_flag = lteiot9_element_parser(
      LTEIOT9_GNSS_GPGGA, LTEIOT9_GPGGA_LATITUDE, latitude_data);

  if (error_flag == 0) {
    error_flag |= lteiot9_element_parser(
        LTEIOT9_GNSS_GPGGA, LTEIOT9_GPGGA_LONGITUDE, longitude_data);
  }

  if (error_flag == 0) {
    if (last_error_flag != 0) {
    }

    if (strlen(latitude_data) < 7 || strlen(longitude_data) < 7) {
      return retval;
    }

    if (strcmp(latitude_data, longitude_data) == 0) {
      return retval;
    }

    retval = 0;

  } else if (error_flag < -1) {
    if (last_error_flag == 0) {
    }
  }

  if (error_flag != -1) {
    last_error_flag = error_flag;
    lteiot9_clear_app_buf();
  }

  return retval;
}

int lteiot9_gnss_coordinates() {
  float latitude_decimal = parse_coordinate(latitude_data);
  float longitude_decimal = parse_coordinate(longitude_data);

  if (latitude_decimal == -999 || longitude_decimal == -999) {
    return -1;
  }

  format_coordinate_json_data(latitude_decimal, longitude_decimal, json_data);
  location_send_flag = 1;

  return 0;
}

void gnssDataFunction() {
  switch (gnss_data_status) {
    case GNSS_DATA_POWER_UP:
      lteiot9_send_cmd(&lteiot9, GNSS_POWER_UP);
      gnss_data_status = GNSS_DATA_POWER_UP_CHECK;
      wait_value = 1000;
      break;
    case GNSS_DATA_POWER_UP_CHECK:
      app_error_flag = lteiot9_rsp_check();
      if (strstr(app_buf, "^SGPSC: \"Engine\",\"0\",\"8\"") != NULL) {
        gnss_data_status = GNSS_DATA_POWER_UP;
        lteiot9_status = LTEIOT9_GNSS_CONFIG;
      }

      if (app_error_flag == 0) {
        gnss_data_status = GNSS_DATA_START_OUTPUT;
        gnssDataFunctionCounter = 0;
      } else {
        gnss_data_status = GNSS_DATA_POWER_UP;
      }
      break;
    case GNSS_DATA_START_OUTPUT:
      lteiot9_send_cmd(&lteiot9, GNSS_START_OUT);
      app_error_flag = lteiot9_rsp_check();

      if (app_error_flag == 0) {
        gnss_data_status = GNSS_DATA_PROCESS;
        gnssDataFunctionCounter = 0;
      }

      break;
    case GNSS_DATA_PROCESS:
      gnssDataFunctionCounter++;
      lteiot9_process_gnss();
      gnss_data_status = GNSS_DATA_PARSE;
      break;
    case GNSS_DATA_PARSE:
      gnssPostFlag = lteiot9_gnss_data_parser();
      if (gnssPostFlag == 0) {
        gnss_data_status = GNSS_DATA_PARSE_COORDINATES;
        gnssDataFunctionCounter = 0;
      } else {
        gnss_data_status = GNSS_DATA_PROCESS;
      }

      if (gnssDataFunctionCounter > gnssDataFunctionCounter_MaxVal) {
        gnssDataFunctionCounter = 0;
        gnss_data_status = GNSS_DATA_STOP_OUTPUT;
      }

      break;
    case GNSS_DATA_PARSE_COORDINATES:
      lteiot9_gnss_coordinates();
      gnss_data_status = GNSS_DATA_STOP_OUTPUT;
      break;
    case GNSS_DATA_STOP_OUTPUT:
      lteiot9_send_cmd(&lteiot9, GNSS_STOP_OUT);
      app_error_flag = lteiot9_rsp_check();
      if (app_error_flag == 0) {
        gnss_data_status = GNSS_DATA_POWER_DOWN;
      }
      break;
    case GNSS_DATA_POWER_DOWN:
      lteiot9_send_cmd(&lteiot9, GNSS_POWER_DOWN);
      app_error_flag = lteiot9_rsp_check();
      if (app_error_flag == 0) {
        gnss_data_status = GNSS_DATA_POWER_UP;
        gnssDataFunctionCounter_MaxVal = 15;

        if (gnssPostFlag == 0) {
          lteiot9_status = LTEIOT9_HTTP_GNSS_SETUP;
          gnssDataFunctionCounter_MaxVal = 10;
        } else {
          lteiot9_status = LTEIOT9_HTTP_FLAGS_SETUP;
        }
      } else {
        lteiot9_send_cmd(&lteiot9, GNSS_POWER_DOWN);
      }
      break;
    default:
      break;
  }
}

void initFunction() {
  switch (init_status) {
    case INIT_ON:
      lteiot9_set_on_pin(&lteiot9, 1);
      lteiot9_process();
      lteiot9_clear_app_buf();
      lteiot9_send_cmd_with_parameter(&lteiot9, LTEIOT9_CMD_CFUN, "1,1");
      init_status = INIT_SYSSTART;
      wait_value = 3000;
      break;
    case INIT_SYSSTART:
      initFunctionCounter++;
      lteiot9_process();
      if (strstr(app_buf, LTEIOT9_SYSSTART) != NULL) {
        init_status = INIT_CHECK;
        lteiot9_clear_app_buf();
        ping_start();
      }
      break;
    case INIT_CHECK:
      lteiot9_send_cmd(&lteiot9, LTEIOT9_CMD_AT);
      app_error_flag = lteiot9_rsp_check();

      lteiot9_send_cmd(&lteiot9, LTEIOT9_CMD_ATI);
      app_error_flag = lteiot9_rsp_check();

      lteiot9_send_cmd(&lteiot9, "AT+CEMODE=2");
      app_error_flag = lteiot9_rsp_check();

      init_status = INIT_ON;
      lteiot9_status = LTEIOT9_NET_CONFIG;
      break;
    default:
      break;
  }

  if (initFunctionCounter > initFunctionCounter_MaxVal) {
    lteiot9_set_on_pin(&lteiot9, 0);
    initFunctionCounter = 0;
    init_status = INIT_ON;
    wait_value = 1000;
  }
}

void netConfigFunction() {
  switch (net_config_status) {
    case NET_CONFIG_CIMI:
      netConfigFunctionCounter++;
      lteiot9_send_cmd(&lteiot9, LTEIOT9_CMD_CIMI);
      app_error_flag = lteiot9_rsp_check();
      if (app_error_flag == 0) {
        net_config_status = NET_CONFIG_SIM;
        netConfigFunctionCounter = 0;
        lteiot9_clear_app_buf();
      }

      break;
    case NET_CONFIG_SIM:
      netConfigFunctionCounter++;
      lteiot9_set_sim_apn(&lteiot9, SIM_APN);
      app_error_flag = lteiot9_rsp_check();
      if (app_error_flag == 0) {
        net_config_status = NET_CONFIG_CREG;
        netConfigFunctionCounter = 0;
        lteiot9_clear_app_buf();
      }
      break;
    case NET_CONFIG_CREG:
      netConfigFunctionCounter++;
      lteiot9_send_cmd_with_parameter(&lteiot9, LTEIOT9_CMD_CREG, "2");
      app_error_flag = lteiot9_rsp_check();
      if (app_error_flag == 0) {
        net_config_status = NET_CONFIG_CIMI;
        lteiot9_status = LTEIOT9_NET_CONN;
        netConfigFunctionCounter = 0;
        lteiot9_clear_app_buf();
        netConfigCounter++;

        if (netConfigCounter > 3) {
          netConfigCounter = 0;
          lteiot9_status = LTEIOT9_INIT;
          lteiot9_set_on_pin(&lteiot9, 0);
        }
      }

      break;
    default:
      // Handle unexpected mode
      break;
  }

  if (netConfigFunctionCounter > netConfigFunctionCounter_MaxVal) {
    netConfigFunctionCounter = 0;
    net_config_status = NET_CONFIG_CIMI;
  }
}

void netConnFunction() {
  switch (net_conn_status) {
    case NET_CONN_CGATT:
      wait_value = 500;
      netConnFunctionCounter++;
      lteiot9_send_cmd_check(&lteiot9, LTEIOT9_CMD_CGATT);
      app_error_flag = lteiot9_rsp_check();
      if (app_error_flag == 0) {
        net_conn_status = NET_CONN_CEREG;
        netConnFunctionCounter = 0;
        lteiot9_clear_app_buf();
      }

      break;
    case NET_CONN_CEREG:
      wait_value = 500;
      netConnFunctionCounter++;
      lteiot9_send_cmd_check(&lteiot9, LTEIOT9_CMD_CEREG);
      app_error_flag = lteiot9_rsp_check();
      if (app_error_flag == 0) {
        net_conn_status = NET_CONN_CSQ;
        netConnFunctionCounter = 0;
        lteiot9_clear_app_buf();
      }
      break;
    case NET_CONN_CSQ:
      wait_value = 500;
      netConnFunctionCounter++;
      lteiot9_send_cmd(&lteiot9, LTEIOT9_CMD_CSQ);
      app_error_flag = lteiot9_rsp_check();
      if (app_error_flag == 0) {
        net_conn_status = NET_CONN_CGATT;
        lteiot9_status = LTEIOT9_HTTP_FLAGS_SETUP;
        netConnFunctionCounter = 0;
        lteiot9_clear_app_buf();
      }

      break;
    default:
      // Handle unexpected mode
      break;
  }

  if (netConnFunctionCounter > netConnFunctionCounter_MaxVal) {
    netConnFunctionCounter = 0;
    net_config_status = NET_CONN_CGATT;
    lteiot9_status = LTEIOT9_NET_CONFIG;
  }
}

void setupHttpFlagsFunction() {
  switch (setup_http_flags_status) {
    case HTTP_FLAGS_SETUP_SICA_1:
      setup_http_flags_status = HTTP_FLAGS_SETUP_SICA_0;  /////
      setupHttpFlagsFunctionCounter = 0;
      break;
    case HTTP_FLAGS_SETUP_SICA:
      setup_http_flags_status = HTTP_FLAGS_SETUP_CGPADDR;
      setupHttpFlagsFunctionCounter = 0;
      break;
    case HTTP_FLAGS_SETUP_CGPADDR:
      setup_http_flags_status = HTTP_FLAGS_SETUP_SICA_0;
      setupHttpFlagsFunctionCounter = 0;
      break;
    case HTTP_FLAGS_SETUP_SICA_0:
      setupHttpFlagsFunctionCounter++;
      lteiot9_send_cmd(&lteiot9, "AT^SICA=0,1");  // Ensure no active connection
      app_error_flag = lteiot9_rsp_check();
      if (app_error_flag == 0) {
        setup_http_flags_status = HTTP_FLAGS_SETUP_SSECUA;
        setupHttpFlagsFunctionCounter = 0;
      } else if (strstr(app_buf, "GPRS Error") != NULL) {
        lteiot9_status = LTEIOT9_INIT;
        lteiot9_set_on_pin(&lteiot9, 0);
      }
      break;
    case HTTP_FLAGS_SETUP_SSECUA:
      lteiot9_send_cmd(&lteiot9,
                       "AT^SSECUA=\"CertStore/TLS/PreconfigureCerts\"");
      setup_http_flags_status = HTTP_FLAGS_SETUP_DNS;
      break;
    case HTTP_FLAGS_SETUP_DNS:
      setupHttpFlagsFunctionCounter++;
      char cmd_dns[100];
      sprintf(cmd_dns, "AT^SICS=1,\"dns1\",\"%s\"", primary_dns[dns_idx]);
      lteiot9_send_cmd(&lteiot9, cmd_dns);
      if (app_error_flag == 0) {
        setup_http_flags_status = HTTP_FLAGS_SETUP_SICA_ON;
        setupHttpFlagsFunctionCounter = 0;
      }
      sprintf(cmd_dns, "AT^SICS=1,\"dns2\",\"%s\"", secondary_dns[dns_idx]);
      lteiot9_send_cmd(&lteiot9, cmd_dns);
      if (app_error_flag == 0) {
        setup_http_flags_status = HTTP_FLAGS_SETUP_SICA_ON;
        setupHttpFlagsFunctionCounter = 0;
      }
      break;
    case HTTP_FLAGS_SETUP_SICA_ON:
      setupHttpFlagsFunctionCounter++;
      lteiot9_send_cmd(&lteiot9, "AT^SICA=1,1");
      app_error_flag = lteiot9_rsp_check();
      if (app_error_flag == 0) {
        setup_http_flags_status = HTTP_FLAGS_SETUP_HTTP;
        setupHttpFlagsFunctionCounter = 0;
      }
      break;
    case HTTP_FLAGS_SETUP_HTTP:
      setupHttpFlagsFunctionCounter++;
      lteiot9_send_cmd(&lteiot9, "AT^SISS=4,srvType,\"Http\"");
      app_error_flag = lteiot9_rsp_check();
      if (app_error_flag == 0) {
        setup_http_flags_status = HTTP_FLAGS_SETUP_CON;
        setupHttpFlagsFunctionCounter = 0;
      }
      break;
    case HTTP_FLAGS_SETUP_CON:
      setupHttpFlagsFunctionCounter++;
      lteiot9_send_cmd(&lteiot9, "AT^SISS=4,conid,\"1\"");
      app_error_flag = lteiot9_rsp_check();
      if (app_error_flag == 0) {
        setup_http_flags_status = HTTP_FLAGS_SETUP_ADDR;
        setupHttpFlagsFunctionCounter = 0;
      }
      break;
    case HTTP_FLAGS_SETUP_ADDR:
      setupHttpFlagsFunctionCounter++;
      lteiot9_send_cmd(&lteiot9,
                       "AT^SISS=4,\"address\",\"" SERVER_URL_FLAGS "\"");
      app_error_flag = lteiot9_rsp_check();
      if (app_error_flag == 0) {
        setup_http_flags_status = HTTP_FLAGS_SETUP_HCPROP;
        setupHttpFlagsFunctionCounter = 0;
      }
      break;
    case HTTP_FLAGS_SETUP_HCPROP:
      setupHttpFlagsFunctionCounter++;

      char cmd[100];
      sprintf(cmd, "AT^SISS=4,hcprop,\"device_id: %s\"", uid_str);
      lteiot9_send_cmd(&lteiot9, cmd);
      app_error_flag = lteiot9_rsp_check();
      if (app_error_flag == 0) {
        setup_http_flags_status = HTTP_FLAGS_SETUP_SICA_ON;
        lteiot9_status = LTEIOT9_HTTP_FLAGS_POST;
        setupHttpFlagsFunctionCounter = 0;
      }
      break;
    default:
      // Handle unexpected mode
      break;
  }

  if (setupHttpFlagsFunctionCounter > setupHttpFlagsFunctionCounter_MaxVal) {
    if (setup_http_flags_status == HTTP_FLAGS_SETUP_SICA_1 ||
        setup_http_flags_status == HTTP_FLAGS_SETUP_SICA_0) {
      lteiot9_status = LTEIOT9_NET_CONFIG;
      http_fail_counter = 0;
    }

    setup_http_flags_status = HTTP_FLAGS_SETUP_SICA_1;
    setupHttpFlagsFunctionCounter = 0;
  }
}

void postHttpFlagsFunction() {
  switch (post_http_flags_status) {
    case HTTP_FLAGS_POST_POST:
      postHttpFlagsFunctionCounter++;
      lteiot9_send_cmd(&lteiot9, "AT^SISS=4,cmd,\"get\"");
      app_error_flag = lteiot9_rsp_check();
      if (app_error_flag == 0) {
        post_http_flags_status = HTTP_FLAGS_POST_SISO;
        postHttpFlagsFunctionCounter = 0;
      }
      break;
    case HTTP_FLAGS_POST_CONTENT:
      postHttpFlagsFunctionCounter++;
      char cmd[100];
      sprintf(cmd, "AT^SISS=4,\"hccontent\",\"{\\\"id\\\":\\\"%s\\\"}\"",
              uid_str);
      lteiot9_send_cmd(&lteiot9, cmd);
      app_error_flag = lteiot9_rsp_check();
      if (app_error_flag == 0) {
        post_http_flags_status = HTTP_FLAGS_POST_LEN;
        postHttpFlagsFunctionCounter = 0;
      }
      break;
    case HTTP_FLAGS_POST_LEN:
      postHttpFlagsFunctionCounter++;
      lteiot9_send_cmd(&lteiot9, "AT^SISS=4,\"hccontlen\",\"0\"");
      app_error_flag = lteiot9_rsp_check();
      if (app_error_flag == 0) {
        post_http_flags_status = HTTP_FLAGS_POST_SISO;
        postHttpFlagsFunctionCounter = 0;
      }
      break;
    case HTTP_FLAGS_POST_SISO:
      postHttpFlagsFunctionCounter++;
      lteiot9_send_cmd(&lteiot9, "AT^SISO=4");  // Open Internet service
      app_error_flag = lteiot9_rsp_check();
      if (app_error_flag == 0) {
        post_http_flags_status = HTTP_FLAGS_POST_RESPONSE;
        postHttpFlagsFunctionCounter = 0;
        wait_value = 1000;
      }
      break;
    case HTTP_FLAGS_POST_RESPONSE:
      postHttpFlagsFunctionCounter++;
      lteiot9_clear_app_buf();
      lteiot9_process();
      if (strstr(app_buf, "^SISR: 4,1") != NULL) {
        lteiot9_send_cmd(&lteiot9, "AT^SISR=4,100");
        app_error_flag = lteiot9_rsp_check();
        if (app_error_flag == 0) {
          strcpy(response, app_buf);
          parse_flags_json_response();
          if (http_fail_counter >= 0) {
            http_fail_counter--;
          }
          postHttpFlagsFunctionCounter = 0;
        }
      }

      if (strstr(app_buf, "^SIS: 4,0,24") != NULL ||
          strstr(app_buf, "^SIS: 4,0,62") != NULL) {
        http_fail_counter++;
        lteiot9_send_cmd(&lteiot9, "AT^SISC=4");
        app_error_flag = lteiot9_rsp_check();
        if (app_error_flag == 0) {
          post_http_flags_status = HTTP_FLAGS_POST_SISC;
          postHttpFlagsFunctionCounter = 0;
          lteiot9_status = LTEIOT9_HTTP_FLAGS_SETUP;
        }
      }

      if (strstr(app_buf, "^SISR: 4,2") != NULL ||
          strstr(app_buf, "^SISR: 4,-2") != NULL) {
        lteiot9_send_cmd(&lteiot9, "AT^SISC=4");
        app_error_flag = lteiot9_rsp_check();
        if (app_error_flag == 0) {
          post_http_flags_status = HTTP_FLAGS_POST_SISC;
          postHttpFlagsFunctionCounter = 0;
        }
      }

      break;
    case HTTP_FLAGS_POST_SISC:
      lteiot9_send_cmd(&lteiot9, "AT^SISC=4");
      app_error_flag = lteiot9_rsp_check();
      if (app_error_flag == 0) {
        post_http_flags_status = HTTP_FLAGS_POST_POST;
        lteiot9_status = LTEIOT9_HTTP_FLAGS_SETUP;

        if (ping_status == 1) {
          ping_routine();
          lteiot9_status = LTEIOT9_HTTP_FLAGS_POST;
        } else {
          if (gps_status == 1) {
            if (gps_status_old == 1) {
              if (location_send_flag == 1) {
                lteiot9_status = LTEIOT9_HTTP_GNSS_SETUP;
              } else {
                lteiot9_status = LTEIOT9_GNSS_DATA;
              }
            } else {
              lteiot9_status = LTEIOT9_GNSS_CONFIG;
              gps_status_old = 1;
            }
          }
        }
      }
      break;
    default:
      // Handle unexpected mode
      break;
  }

  if (post_http_flags_status == HTTP_FLAGS_POST_RESPONSE) {
    if (postHttpFlagsFunctionCounter > 25) {
      postHttpFlagsFunctionCounter = 0;
      post_http_flags_status = HTTP_FLAGS_POST_SISC;
    }
  } else {
    if (postHttpFlagsFunctionCounter > postHttpFlagsFunctionCounter_MaxVal) {
      lteiot9_send_cmd(&lteiot9, "AT^SISC=4");
      app_error_flag = lteiot9_rsp_check();
      postHttpFlagsFunctionCounter = 0;
      post_http_flags_status = HTTP_FLAGS_POST_POST;
      lteiot9_status = LTEIOT9_HTTP_FLAGS_SETUP;
    }
  }
}

void gnssConfigFunction() {
  switch (gnss_config_status) {
    case GNSS_CONFIG_GPS:
      lteiot9_send_cmd(&lteiot9, GNNS_START_GPS);
      app_error_flag = lteiot9_rsp_check();
      if (app_error_flag == 0) {
        gnss_config_status = GNSS_CONFIG_NMEA_ON;
      }
      break;
    case GNSS_CONFIG_NMEA_ON:
      lteiot9_send_cmd(&lteiot9, GNSS_START_OUT);
      app_error_flag = lteiot9_rsp_check();
      if (app_error_flag == 0) {
        gnss_config_status = GNSS_CONFIG_START_MODE;
      }
      break;
    case GNSS_CONFIG_START_MODE:
      lteiot9_send_cmd(&lteiot9, GNNS_START_MODE_EN);
      app_error_flag = lteiot9_rsp_check();
      if (app_error_flag == 0) {
        gnss_config_status = GNSS_WAIT_REBOOT;
        lteiot9_send_cmd_with_parameter(&lteiot9, LTEIOT9_CMD_CFUN, "1,1");
      }
      break;
    case GNSS_WAIT_REBOOT:
      lteiot9_process();
      if (strstr(app_buf, LTEIOT9_SYSSTART) != NULL) {
        gnss_config_status = GNSS_CONFIG_GPS;
        lteiot9_status = LTEIOT9_GNSS_DATA;
      }
      break;
    default:
      // Handle unexpected mode
      break;
  }
}

void setupHttpGnssFunction() {
  switch (setup_http_gnss_status) {
    case HTTP_GNSS_SETUP_SICA_1:
      setup_http_gnss_status = HTTP_GNSS_SETUP_SICA_0;
      setupHttpGnssFunctionCounter = 0;
      break;
    case HTTP_GNSS_SETUP_SICA:
      setup_http_gnss_status = HTTP_GNSS_SETUP_CGPADDR;
      setupHttpGnssFunctionCounter = 0;
      break;
    case HTTP_GNSS_SETUP_CGPADDR:
      setup_http_gnss_status = HTTP_GNSS_SETUP_SICA_0;
      setupHttpGnssFunctionCounter = 0;
      break;
    case HTTP_GNSS_SETUP_SICA_0:
      setupHttpGnssFunctionCounter++;
      lteiot9_send_cmd(&lteiot9, "AT^SICA=0,1");  // Ensure no active connection
      app_error_flag = lteiot9_rsp_check();
      if (app_error_flag == 0) {
        setup_http_gnss_status = HTTP_GNSS_SETUP_SSECUA;
        setupHttpGnssFunctionCounter = 0;
      } else if (strstr(app_buf, "GPRS Error") != NULL) {
        lteiot9_status = LTEIOT9_INIT;
        lteiot9_set_on_pin(&lteiot9, 0);
      }
      break;
    case HTTP_GNSS_SETUP_SSECUA:
      lteiot9_send_cmd(&lteiot9,
                       "AT^SSECUA=\"CertStore/TLS/PreconfigureCerts\"");
      setup_http_gnss_status = HTTP_GNSS_SETUP_DNS;
      break;
    case HTTP_GNSS_SETUP_DNS:
      setupHttpGnssFunctionCounter++;
      char cmd[100];
      sprintf(cmd, "AT^SICS=1,\"dns1\",\"%s\"", primary_dns[dns_idx]);
      lteiot9_send_cmd(&lteiot9, cmd);
      if (app_error_flag == 0) {
        setup_http_gnss_status = HTTP_GNSS_SETUP_SICA_ON;
        setupHttpGnssFunctionCounter = 0;
      }
      sprintf(cmd, "AT^SICS=1,\"dns2\",\"%s\"", secondary_dns[dns_idx]);
      lteiot9_send_cmd(&lteiot9, cmd);
      if (app_error_flag == 0) {
        setup_http_gnss_status = HTTP_GNSS_SETUP_SICA_ON;
        setupHttpGnssFunctionCounter = 0;
      }
      break;
    case HTTP_GNSS_SETUP_SICA_ON:
      setupHttpGnssFunctionCounter++;
      lteiot9_send_cmd(&lteiot9, "AT^SICA=1,1");
      app_error_flag = lteiot9_rsp_check();
      if (app_error_flag == 0) {
        setup_http_gnss_status = HTTP_GNSS_SETUP_HTTP;
        setupHttpGnssFunctionCounter = 0;
      }
      break;
    case HTTP_GNSS_SETUP_HTTP:
      setupHttpGnssFunctionCounter++;
      lteiot9_send_cmd(&lteiot9, "AT^SISS=6,srvType,\"Http\"");
      app_error_flag = lteiot9_rsp_check();
      if (app_error_flag == 0) {
        setup_http_gnss_status = HTTP_GNSS_SETUP_CON;
        setupHttpGnssFunctionCounter = 0;
      }
      break;
    case HTTP_GNSS_SETUP_CON:
      setupHttpGnssFunctionCounter++;
      lteiot9_send_cmd(&lteiot9, "AT^SISS=6,conid,\"1\"");
      app_error_flag = lteiot9_rsp_check();
      if (app_error_flag == 0) {
        setup_http_gnss_status = HTTP_GNSS_SETUP_ADDR;
        setupHttpGnssFunctionCounter = 0;
      }
      break;
    case HTTP_GNSS_SETUP_ADDR:
      setupHttpGnssFunctionCounter++;
      lteiot9_send_cmd(&lteiot9,
                       "AT^SISS=6,\"address\",\"" SERVER_URL_LOCATION "\"");
      app_error_flag = lteiot9_rsp_check();
      if (app_error_flag == 0) {
        setup_http_gnss_status = HTTP_GNSS_SETUP_HCPROP;
        setupHttpGnssFunctionCounter = 0;
      }
      break;
    case HTTP_GNSS_SETUP_HCPROP:
      setupHttpGnssFunctionCounter++;
      lteiot9_send_cmd(&lteiot9,
                       "AT^SISS=6,hcprop,\"Content-Type: application/json\"");
      app_error_flag = lteiot9_rsp_check();
      if (app_error_flag == 0) {
        setup_http_gnss_status = HTTP_GNSS_SETUP_SICA_ON;
        setupHttpGnssFunctionCounter = 0;
        lteiot9_status = LTEIOT9_HTTP_GNSS_POST;
      }
      break;
    default:
      // Handle unexpected mode
      break;
  }

  if (setupHttpGnssFunctionCounter > setupHttpGnssFunctionCounter_MaxVal) {
    if (setup_http_gnss_status == HTTP_GNSS_SETUP_SICA_1) {
      lteiot9_status = LTEIOT9_NET_CONFIG;
      http_fail_counter = 0;
    }

    lteiot9_status = LTEIOT9_HTTP_FLAGS_SETUP;

    setup_http_gnss_status = HTTP_GNSS_SETUP_SICA_1;
    setupHttpGnssFunctionCounter = 0;
  }
}

void postHttpGnssFunction() {
  switch (post_http_gnss_status) {
    case HTTP_GNSS_POST_POST:
      postHttpGnssFunctionCounter++;
      lteiot9_send_cmd(&lteiot9, "AT^SISS=6,cmd,\"post\"");
      app_error_flag = lteiot9_rsp_check();
      if (app_error_flag == 0) {
        post_http_gnss_status = HTTP_GNSS_POST_CONTENT;
        postHttpGnssFunctionCounter = 0;
      }
      break;
    case HTTP_GNSS_POST_CONTENT:
      postHttpGnssFunctionCounter++;
      char cmd[200];
      sprintf(cmd, "AT^SISS=6,\"hccontent\",\"%s\"", json_data);
      lteiot9_send_cmd(&lteiot9, cmd);  // Set target URL
      app_error_flag = lteiot9_rsp_check();
      if (app_error_flag == 0) {
        post_http_gnss_status = HTTP_GNSS_POST_LEN;
        postHttpGnssFunctionCounter = 0;
      }
      break;
    case HTTP_GNSS_POST_LEN:
      postHttpGnssFunctionCounter++;
      lteiot9_send_cmd(&lteiot9, "AT^SISS=6,\"hccontlen\",\"0\"");
      app_error_flag = lteiot9_rsp_check();
      if (app_error_flag == 0) {
        post_http_gnss_status = HTTP_GNSS_POST_SISO;
        postHttpGnssFunctionCounter = 0;
      }
      break;
    case HTTP_GNSS_POST_SISO:
      postHttpGnssFunctionCounter++;
      lteiot9_send_cmd(&lteiot9, "AT^SISO=6");  // Open Internet service
      app_error_flag = lteiot9_rsp_check();
      if (app_error_flag == 0) {
        post_http_gnss_status = HTTP_GNSS_POST_RESPONSE;
        postHttpGnssFunctionCounter = 0;
      }
      break;
    case HTTP_GNSS_POST_RESPONSE:
      postHttpGnssFunctionCounter++;
      lteiot9_clear_app_buf();
      lteiot9_process();
      if (strstr(app_buf, "^SISR: 6,1") != NULL) {
        postHttpGnssFunctionCounter = 0;
        lteiot9_send_cmd(&lteiot9, "AT^SISR=6,100");
        lteiot9_process();
        http_fail_counter--;
        location_send_flag = 0;
      }

      if (strstr(app_buf, "^SIS: 6,0,24") != NULL ||
          strstr(app_buf, "^SIS: 6,0,62") != NULL) {
        http_fail_counter++;
        lteiot9_send_cmd(&lteiot9, "AT^SISC=6");
        if (app_error_flag == 0) {
          post_http_gnss_status = HTTP_GNSS_POST_SISC;
          postHttpGnssFunctionCounter = 0;
        }
      }

      if (strstr(app_buf, "^SISR: 6,2") != NULL ||
          strstr(app_buf, "^SISR: 6,-2") != NULL) {
        lteiot9_send_cmd(&lteiot9, "AT^SISC=6");
        if (app_error_flag == 0) {
          post_http_gnss_status = HTTP_GNSS_POST_SISC;
          postHttpGnssFunctionCounter = 0;
        }
      }

      break;
    case HTTP_GNSS_POST_SISC:
      lteiot9_send_cmd(&lteiot9, "AT^SISC=6");
      app_error_flag = lteiot9_rsp_check();
      if (app_error_flag == 0) {
        post_http_gnss_status = HTTP_GNSS_POST_POST;
        lteiot9_status = LTEIOT9_HTTP_FLAGS_SETUP;
      }
      break;
    default:
      // Handle unexpected mode
      break;
  }

  if (post_http_gnss_status == HTTP_GNSS_POST_RESPONSE) {
    if (postHttpGnssFunctionCounter > 30) {
      postHttpGnssFunctionCounter = 0;
      post_http_gnss_status = HTTP_GNSS_POST_SISC;
    }
  } else {
    if (postHttpGnssFunctionCounter > postHttpGnssFunctionCounter_MaxVal) {
      lteiot9_send_cmd(&lteiot9, "AT^SISC=6");
      app_error_flag = lteiot9_rsp_check();
      if (post_http_gnss_status == HTTP_GNSS_POST_POST) {
        lteiot9_status = LTEIOT9_HTTP_GNSS_SETUP;
      }
      postHttpGnssFunctionCounter = 0;
      post_http_gnss_status = LTEIOT9_HTTP_GNSS_SETUP;
    }
  }
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1) {
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
void assert_failed(uint8_t *file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
