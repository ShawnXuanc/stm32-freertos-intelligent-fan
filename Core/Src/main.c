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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "ssd1306.h"
#include "ssd1306_tests.h"
#include "ssd1306_fonts.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "state.h"
#include "semphr.h"
#include "string.h"

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
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define SPEED_LOW 800
#define SPEED_MID 1450
#define SPEED_HIGH 1999

#define DHT11_PORT GPIOB
// PB9
#define DHT11_PIN GPIO_PIN_9
#define PERIOD_MS 2000u

#define Q_SIZE 128

typedef struct SystemState {
	float temperature;
	uint8_t ir_state;
	uint16_t fan_pwm;
	uint8_t fan_enable;
	enum Mode mode;
	uint32_t last_update_ms;

} SystemState_t;

SystemState_t sys_state = {
    .temperature = 0,
	.ir_state = 0,
	.fan_pwm = SPEED_LOW,
	.fan_enable = 0,
	.mode = MODE_AUTO,
	.last_update_ms = 0
};

typedef struct {
	SystemState_t *sys;
	int set_pwm;
	int set_temp;
} bt_cmd_context_t ;

typedef void (*bt_cmd_handler_t)(bt_cmd_context_t *bct);

typedef struct {
	char *cmd;
	bt_cmd_handler_t bt_handler;
} bt_cmd_entry_t;


// include priority inheritance
SemaphoreHandle_t state_mutex;

#define BT_CMD_TABLE_SIZE (sizeof(bt_cmd_table) / sizeof(bt_cmd_table[0]))

QueueHandle_t btQueue;

// IR sensor
volatile uint32_t *irReg  = (uint32_t *)0x40020010;
// DHT11
uint32_t pMillis, cMillis;
// HC05
uint8_t rxData;

void bt_set_fan_state(SystemState_t *sys, uint8_t enable, uint16_t pwm, enum Mode mode)
{
	sys->fan_enable = enable;
	sys->fan_pwm    = pwm;
	sys->mode       = mode;
}

#define BT_CMD_MODE(name, enable, pwm, mode) \
	void bt_handler_##name(bt_cmd_context_t *bct) { \
	    bt_set_fan_state(bct->sys, enable, pwm, mode); \
	}

BT_CMD_MODE(on, 1, SPEED_LOW, MODE_MANUAL)
BT_CMD_MODE(off, 0, SPEED_LOW, MODE_AUTO)
BT_CMD_MODE(low, 1, SPEED_LOW, MODE_MANUAL)
BT_CMD_MODE(mid, 1, SPEED_MID, MODE_MANUAL)
BT_CMD_MODE(high, 1, SPEED_HIGH, MODE_MANUAL)

void bt_handler_set_pwm(bt_cmd_context_t *bct) {
	int pwm = bct->set_pwm;
	if (pwm < 0 || pwm > 2000)
	    pwm = 0;
	bt_set_fan_state(bct->sys, 1, pwm, MODE_MANUAL);
}

static const bt_cmd_entry_t bt_cmd_table[] = {
	{.cmd = "on", .bt_handler = bt_handler_on},
	{.cmd = "off", .bt_handler = bt_handler_off},
	{.cmd = "low", .bt_handler = bt_handler_low},
	{.cmd = "mid", .bt_handler = bt_handler_mid},
	{.cmd = "high", .bt_handler = bt_handler_high},
	{.cmd = "pwm", .bt_handler = bt_handler_set_pwm}
};


void microDelay(uint16_t delay) {
  __HAL_TIM_SET_COUNTER(&htim1, 0);
  while (__HAL_TIM_GET_COUNTER(&htim1) < delay)
    ;
}

uint8_t DHT11_Start(void) {
  uint8_t Response = 0;
  GPIO_InitTypeDef GPIO_InitStructPrivate = {0};
  GPIO_InitStructPrivate.Pin = DHT11_PIN;
  GPIO_InitStructPrivate.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStructPrivate.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStructPrivate.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DHT11_PORT, &GPIO_InitStructPrivate); // set the pin as output
  HAL_GPIO_WritePin(DHT11_PORT, DHT11_PIN, 0);        // pull the pin low
  HAL_Delay(20);                                      // wait for 20ms
  HAL_GPIO_WritePin(DHT11_PORT, DHT11_PIN, 1);        // pull the pin high
  microDelay(30);                                     // wait for 30us
  GPIO_InitStructPrivate.Mode = GPIO_MODE_INPUT;
  GPIO_InitStructPrivate.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(DHT11_PORT, &GPIO_InitStructPrivate); // set the pin as input
  microDelay(40);
  if (!(HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN))) {
	  microDelay(80);
	  if ((HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN)))
      Response = 1;
  }
  pMillis = HAL_GetTick();
  cMillis = HAL_GetTick();
  while ((HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN)) && pMillis + 2 > cMillis) {
    cMillis = HAL_GetTick();
  }
  return Response;
}

uint8_t DHT11_Read(void) {
  uint8_t a = 0, b = 0;
  for (a = 0; a < 8; a++) {
    pMillis = HAL_GetTick();
    cMillis = HAL_GetTick();

    while (!(HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN)) &&
           pMillis + 2 > cMillis) {
      cMillis = HAL_GetTick();
    }

    microDelay(40);                                 // wait for 40 us
    if (!(HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN))) // if the pin is low
      b &= ~(1 << (7 - a));
    else
      b |= (1 << (7 - a));
    pMillis = HAL_GetTick();
    cMillis = HAL_GetTick();
    while ((HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN)) &&
           pMillis + 2 > cMillis) { // wait for the pin to go low
      cMillis = HAL_GetTick();
    }
  }
  return b;
}

int check_dht11(float *tCel) {
	if (DHT11_Start()) {
	      uint8_t RHI, RHD, TCI, TCD, SUM;
	      RHI = DHT11_Read(); // Relative humidity integral
	      RHD = DHT11_Read(); // Relative humidity decimal
	      TCI = DHT11_Read(); // Celsius integral
	      TCD = DHT11_Read(); // Celsius decimal
	      SUM = DHT11_Read(); // Check sum
	      if (RHI + RHD + TCI + TCD != SUM) {
	    	  return 0;
	      }
	      // Can use RHI and TCI for any purposes if whole number only needed
		  *tCel = (float)TCI + (float)(TCD / 10.0);
		  return 1;
	 }
	 return 0;
}

void test(void *argument)
{
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15, GPIO_PIN_RESET);
		for(;;){

			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET );
			vTaskDelay(500);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET );
			vTaskDelay(500);
		}
}


void OLED_task(void *pvParameters) {
    SystemState_t *sys = &sys_state;

    float prev_temp = 0.0f;
	uint16_t prev_pwm = SPEED_LOW;
	enum Mode prev_mode = MODE_AUTO;

	for (;;) {
		//  Non-blocking take. If mutex is busy, keep using previous cached values.
		if (xSemaphoreTake(state_mutex, 0) == pdTRUE) {
			prev_temp = sys->temperature;
			prev_pwm = sys->fan_pwm;
			prev_mode = sys->mode;
			xSemaphoreGive(state_mutex);
		}
		char temp_str[20];
		sprintf(temp_str, "%.2f", prev_temp);
		ssd1306_print(prev_pwm, temp_str, prev_mode);
		vTaskDelay(100);
	}
}


void DHT11_task(void *pvParameters) {
	SystemState_t *sys = &sys_state;

	HAL_TIM_Base_Start(&htim1);

	TickType_t lastWakeTime = xTaskGetTickCount();
	const TickType_t period = pdMS_TO_TICKS(PERIOD_MS);

	for (;;) {

		float temp;
		if (check_dht11(&temp)) {
			if (xSemaphoreTake(state_mutex, portMAX_DELAY) == pdTRUE) {
				sys->temperature = temp;
				sys->last_update_ms = HAL_GetTick();
				xSemaphoreGive(state_mutex);
			}
		}
		// period wake up
		vTaskDelayUntil(&lastWakeTime, period);
	}
}

void turn_on_PWM(uint16_t pwm) {
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 0);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 1);
	__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3, pwm);
}

void turn_off_PWM() {
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 0);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 0);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
}


void IR_Init() {
	uint32_t *pClkCtrlReg     = (uint32_t *)0x40023830;
	uint32_t *pPortAModeReg   = (uint32_t *)0x40020000;

	//1. Enable the clock RCC AHB1_ENR FOR GPIOA
	*pClkCtrlReg |=  (1);
	//2. We need to set the GPIO A Mode to Input
	*pPortAModeReg &= ~(3);

}

uint8_t ir_read_reg() {
	return (*irReg & 1) == 0 ? 1 : 0;
}


void IR_task(void *pvParameters) {
	SystemState_t *sys = &sys_state;

	for (;;) {
		uint8_t ir_ = ir_read_reg();
		if (xSemaphoreTake(state_mutex, portMAX_DELAY) == pdTRUE){
			sys->ir_state = ir_;
			xSemaphoreGive(state_mutex);
		}
		vTaskDelay(100);
	}

}

static uint16_t cal_pwm_by_temp(float temp)
{
    if (temp >= 30)  return SPEED_HIGH;
    if (temp >= 28)  return SPEED_MID;
    return SPEED_LOW;
}

void change_fan_state_auto(uint8_t ir_, float temp, enum Mode mode, uint8_t *fan_enable, uint16_t *pwm) {
	if (mode != MODE_AUTO)
		return ;

	*fan_enable = ir_ ? 1 : 0;

	if (!*fan_enable)
		return ;

	*pwm = cal_pwm_by_temp(temp);
}

void FanControl_task(void *pvParameters) {
	SystemState_t *sys = &sys_state;

	for (;;) {
		uint8_t fan_enable;
		uint16_t pwm;
		uint8_t ir_;
		float temp;
		enum Mode mode;
		if (xSemaphoreTake(state_mutex, portMAX_DELAY) == pdTRUE) {
			ir_ = sys->ir_state;
			temp = sys->temperature;
			fan_enable = sys->fan_enable;
			pwm = sys->fan_pwm;
			mode = sys->mode;
			xSemaphoreGive(state_mutex);
		}

		change_fan_state_auto(ir_, temp, mode, &fan_enable, &pwm);

		if (xSemaphoreTake(state_mutex, portMAX_DELAY) == pdTRUE) {
			// Apply auto result only if mode is still AUTO,
			// ensuring MANUAL updates take priority
			if (sys->mode == MODE_AUTO) {
				sys->fan_enable = fan_enable;
				sys->fan_pwm = pwm;
			}

			fan_enable = sys->fan_enable;
			pwm = sys->fan_pwm;
			xSemaphoreGive(state_mutex);
		}


		if (fan_enable) {
			turn_on_PWM(pwm);
		} else {
			turn_off_PWM();
		}

		vTaskDelay(100);
	}
}

int s_to_int(char *str) {
	int num = 0;
	while (*str) {
		if (*str < '0' || *str > '9')
			return -1;
		num = num * 10  + (*str - '0');
		str++;
	}
	return num;
}

void bt_process_string(SystemState_t *sys, char *str) {
	char *cur = str;
	while (*cur) {
		if (*cur >= 'A' && *cur <= 'Z') {
			*cur = (*cur - 'A' + 'a');
		}
		cur++;
	}

	char *num = NULL;
	cur = str;
	while (*cur && *cur != ' ') {
		cur++;
	}

	if (*cur  == ' ') {
		*cur = '\0';
		cur++;
		while (*cur == ' ') {
			cur++;
		}
		if (*cur != '\0')
			num = cur;
	}

	int pwm_ = 0;
	if (num != NULL) {
		int tmp = s_to_int(num);
		if (tmp > 0)
			pwm_ = tmp;
	}


	 bt_cmd_context_t bct = {
	     .sys      = sys,
		 .set_pwm  = pwm_,
		 .set_temp = 0,
	  };

	for (int i = 0; i < BT_CMD_TABLE_SIZE; ++i) {
		if (!strcmp(str, bt_cmd_table[i].cmd)) {
			if (xSemaphoreTake(state_mutex, portMAX_DELAY) == pdTRUE) {
				bt_cmd_table[i].bt_handler(&bct);
				xSemaphoreGive(state_mutex);
			}
			break;
		}
	}
}

void BtRecieve_task(void *pvParameters) {
	SystemState_t *sys = &sys_state;
	char str[20];
	int cur = 0;
	for (;;) {
		// Block until the data is received in the　queue
		char cmd;
		if (xQueueReceive(btQueue, &cmd, portMAX_DELAY) == pdPASS) {
			if (cmd == '\r')
				continue;
			if (cmd == '\n') {
				str[cur] = '\0';
				bt_process_string(sys, str);
				cur = 0;
			} else {
				if (cur >= sizeof(str) - 1) {
					cur = 0;
					continue;
				}
				str[cur++] = cmd;
			}
		}
	}
}

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
  MX_TIM2_Init();
  MX_TIM1_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  ssd1306_Init();
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3); //PB2 TIM2 CH3
  IR_Init();
  btQueue = xQueueCreate(Q_SIZE, sizeof(uint8_t));
  state_mutex = xSemaphoreCreateMutex();
  HAL_UART_Receive_IT(&huart3,&rxData,1);
  xTaskCreate(
 		      test,
 		      "test",
 			  128,
 			  NULL,
 			  1,
 			  NULL);

  xTaskCreate(
			  FanControl_task,
			  "fan",
			  256,
			  NULL,
			  1,
			  NULL);

  xTaskCreate(
			  DHT11_task,
			  "dht11",
			  256,
			  NULL,
			  1,
			  NULL);

  xTaskCreate(
		  	  OLED_task,
  			  "oled",
  			  256,
  			  NULL,
  			  1,
  			  NULL);

  xTaskCreate(
		  	  BtRecieve_task,
  			  "bt",
  			  128,
  			  NULL,
  			  2,
  			  NULL);

  xTaskCreate(
		  	  IR_task,
			  "ir",
			  128,
			  NULL,
			  1,
			  NULL);

  vTaskStartScheduler();
  /* USER CODE END 2 */

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 167;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  htim2.Init.Prescaler = 89;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1999;
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
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  huart3.Init.BaudRate = 9600;
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|GPIO_PIN_5|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, led_green_Pin|led_orange_Pin|led_red_Pin|led_blue_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin : PE4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : btn_blue_Pin */
  GPIO_InitStruct.Pin = btn_blue_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(btn_blue_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA5 PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : led_green_Pin led_orange_Pin led_red_Pin led_blue_Pin */
  GPIO_InitStruct.Pin = led_green_Pin|led_orange_Pin|led_red_Pin|led_blue_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PB4 PB5 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART3) {
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		uint8_t c = rxData;
		xQueueSendFromISR(btQueue, &c, &xHigherPriorityTaskWoken);

		// Restart the UART receive interrupt
		HAL_UART_Receive_IT(&huart3, &rxData, 1);

		// If a higher priority task was unblocked by xQueueSendFromISR
		// request an immediate context switch before exiting the ISR
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

	}
}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM7 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM7) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
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
