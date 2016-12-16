/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */
// Common
#include <math.h>
#include <string.h>
#define SPI_TIMEOUT 20
#define I2C_TIMEOUT 50
#define UART_TIMEOUT 1
#define I2S_TIMEOUT 1000

// LIS302DL: Accelerometer
#define ACC_CS_GPIO_TYPE GPIOE
#define ACC_CS_GPIO_PIN_NUMBER GPIO_PIN_3

// MP45DT02: Microphone
#define PDM_BUFFER_SIZE 20
#define PCM_BUFFER_SIZE 2500
#define LEAKY_KEEP_RATE 0.95
#define PDM_STREAM_BLOCK_SIZE_BIT 8
#define LOUD_THRESHOLD 55
#define LOUD_THRESHOLD_ACCURATE 800

// CS43L22: Speaker
#define SPEAKER_RESET_GPIO_TYPE GPIOD
#define SPEAKER_RESET_GPIO_PIN_NUMBER GPIO_PIN_4
#define DELAY_DURATION 500
#define BEEP_VOLUME 0x1A
#define BEAT 100

// Control
#define AREA_WIDTH 75
#define AREA_HEIGHT 55
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

I2S_HandleTypeDef hi2s2;
I2S_HandleTypeDef hi2s3;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
// LIS302DL: Accelerometer
uint8_t acc_write_nonincremented_address_header = 0b00;
uint8_t acc_read_nonincremented_address_header = 0b10;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2S3_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2S2_Init(void);
static void MX_TIM1_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
// Common
void send_data_via_uart(char *data);
void send_data_via_uart_with_size(char *data, uint16_t size);

// LIS302DL: Accelerometer
void set_cs_before_communicating_with_acc();
void set_cs_after_communicating_with_acc();
uint8_t read_from_acc_reg(uint8_t address);
void write_to_acc_reg(uint8_t address, uint8_t data);
uint8_t acc_who_am_i();
void acc_init();
int8_t acc_read_x();
int8_t acc_read_y();
int8_t acc_read_z();
void show_acc_led_indicator(int8_t x_acc, int8_t y_acc, int8_t z_acc);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

// MP45DT02: Microphone
uint16_t read_pdm();
int calculate_volume(uint16_t data);
float abs_float(float in);
uint8_t is_loud();
uint8_t is_loud_accurate();

// CS43L22: Speaker
void play_musical_note(int index);
void play_musical_note_with_beat(int index, float beat);
void write_to_speaker_reg(uint8_t address, uint8_t data);
void speaker_init();
int character_note_index_mapping(uint8_t character);

// Control
void print_area();
void flush_screen();
void update_main_character(int8_t x_acc, int8_t y_acc, int8_t z_acc);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
// Common
char uart_buffer[100];

// MP45DT02: Microphone
uint16_t pdm_buffer[PDM_BUFFER_SIZE];
uint16_t pdm_value = 0;
int8_t pcm_value = 0;
uint16_t pcm_count = 0;

float leaky_pcm_buffer = 0.0; // For PDM moving average
float leaky_amp_buffer = 0.0; // For |PCM| moving average

double pcm_square = 0;
float max_amp = 0;

// CS43L22: Speaker
uint8_t musical_note[] = { 0x01, 0x11, 0x21, 0x31, 0x41, 0x51, 0x61, 0x71, 0x81,
		0x91, 0xA1, 0xB1, 0xC1, 0xD1, 0xE1, 0xF1 };
char musical_note_name[][2] = { "C4", "C5", "D5", "E5", "F5", "G5", "A5", "B5", "C6",
		"D6", "E6", "F6", "G6", "A6", "B6", "C7" };
int is_playing = 0;

// Control
int area[AREA_HEIGHT][AREA_WIDTH];
int main_character_i = 27;
int main_character_j = 37;
int beat_duration = 550;
int beat_duration_rewind_mode = 200;
int rewind_mode = 0;
int pause = 0;
int range_speed_up = 300;
int base_beat_duration = 600;
/* USER CODE END 0 */

int main(void)
{
  /* USER CODE BEGIN 1 */
	// Common
	int i;
	int state = 0;

	int number_of_note = 16;
	int note[] = { 6, -1, 6, 8, 9, 9, 8, 6, 9, 5, 9, 8, 9, 5, -1, -1 };
	float beat[] = { 0.5,
			0.5, 0.5, 0.5, 0.5, 0.25, 0.25, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 1.0, 0.5 };
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_I2S3_Init();
  MX_SPI1_Init();
  MX_TIM3_Init();
  MX_I2S2_Init();
  MX_TIM1_Init();

  /* USER CODE BEGIN 2 */
  // Initialize
  acc_init();
  speaker_init();
  i = 0;
  // Start timer3 for accelerometer
  HAL_TIM_Base_Start_IT(&htim3);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
		if(state == 0) {
			if(is_loud_accurate()) {
				state = 1;
			} else {
				state = 0;
			}
		} else if(state == 1) {
			if(!is_playing && !pause) {
				is_playing = 1;
				play_musical_note_with_beat(note[i], beat[i]);
				if(rewind_mode) {
					if(i == 0) i = number_of_note - 1;
					else i--;
				} else {
					i = (i + 1) % number_of_note;
				}
			}
			state = 1;
		}
  	// is_loud();
  	// HAL_Delay(500);

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 50;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S;
  PeriphClkInitStruct.PLLI2S.PLLI2SN = 88;
  PeriphClkInitStruct.PLLI2S.PLLI2SR = 4;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 50000;
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

}

/* I2S2 init function */
static void MX_I2S2_Init(void)
{

  hi2s2.Instance = SPI2;
  hi2s2.Init.Mode = I2S_MODE_MASTER_RX;
  hi2s2.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s2.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s2.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
  hi2s2.Init.AudioFreq = I2S_AUDIOFREQ_192K;
  hi2s2.Init.CPOL = I2S_CPOL_LOW;
  hi2s2.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s2.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s2) != HAL_OK)
  {
    Error_Handler();
  }

}

/* I2S3 init function */
static void MX_I2S3_Init(void)
{

  hi2s3.Instance = SPI3;
  hi2s3.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s3.Init.Standard = I2S_STANDARD_MSB;
  hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_44K;
  hi2s3.Init.CPOL = I2S_CPOL_LOW;
  hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s3) != HAL_OK)
  {
    Error_Handler();
  }

}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }

}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 0;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
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

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 249;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

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

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
     PA9   ------> USB_OTG_FS_VBUS
     PA10   ------> USB_OTG_FS_ID
     PA11   ------> USB_OTG_FS_DM
     PA12   ------> USB_OTG_FS_DP
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin 
                          |Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CS_I2C_SPI_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin 
                           Audio_RST_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin 
                          |Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : VBUS_FS_Pin */
  GPIO_InitStruct.Pin = VBUS_FS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(VBUS_FS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OTG_FS_ID_Pin OTG_FS_DM_Pin OTG_FS_DP_Pin */
  GPIO_InitStruct.Pin = OTG_FS_ID_Pin|OTG_FS_DM_Pin|OTG_FS_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MEMS_INT1_Pin */
  GPIO_InitStruct.Pin = MEMS_INT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEMS_INT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
// Common
/**
 * @brief  Send data via UART.
 * @param  data: data to be sent
 * @retval None
 */
void send_data_via_uart(char *data) {
	int i = 0;
	while (data[i] != '\0') {
		HAL_UART_Transmit(&huart2, (uint8_t *) &data[i++], 1, UART_TIMEOUT);
	}
}

/**
 * @brief  Send data via UART.
 * @param  data: data to be sent
 * @retval None
 */
void send_data_via_uart_with_size(char *data, uint16_t size) {
	HAL_UART_Transmit(&huart2, (uint8_t *) data, size, UART_TIMEOUT);
}

// LIS302DL: Accelerometer
/**
 * @brief  Set CS before communicating with an accelerometer.
 * @param  None
 * @retval None
 */
void set_cs_before_communicating_with_acc() {
	// Set to low
	HAL_GPIO_WritePin(ACC_CS_GPIO_TYPE, ACC_CS_GPIO_PIN_NUMBER, GPIO_PIN_RESET);
}

/**
 * @brief  Set CS after communicating with an accelerometer.
 * @param  None
 * @retval None
 */
void set_cs_after_communicating_with_acc() {
	// Set to high
	HAL_GPIO_WritePin(ACC_CS_GPIO_TYPE, ACC_CS_GPIO_PIN_NUMBER, GPIO_PIN_SET);
}

/**
 * @brief  Read data from an specific accelerometer register.
 * @param  address: register address to be read
 * @retval a byte of read data
 */
uint8_t read_from_acc_reg(uint8_t address) {
	set_cs_before_communicating_with_acc();
	uint8_t acc_address_data = (acc_read_nonincremented_address_header << 6) | address;
	HAL_SPI_Transmit(&hspi1, &acc_address_data, 1, SPI_TIMEOUT);

	uint8_t acc_do_data;
	HAL_SPI_Receive(&hspi1, &acc_do_data, 1, SPI_TIMEOUT);
	set_cs_after_communicating_with_acc();
	return acc_do_data;
}

/**
 * @brief  Read data from an specific accelerometer register.
 * @param  address: register address to be read
 * @retval a byte of read data
 */
int8_t read_from_acc_reg_with_sign(uint8_t address) {
	set_cs_before_communicating_with_acc();
	uint8_t acc_address_data = (acc_read_nonincremented_address_header << 6) | address;
	HAL_SPI_Transmit(&hspi1, &acc_address_data, 1, SPI_TIMEOUT);

	int8_t acc_do_data;
	HAL_SPI_Receive(&hspi1, &acc_do_data, 1, SPI_TIMEOUT);
	set_cs_after_communicating_with_acc();
	return acc_do_data;
}

/**
 * @brief  Write data to an specific accelerometer register.
 * @param  address: register address to be written
 * @param  data: data to be written
 * @retval None
 */
void write_to_acc_reg(uint8_t address, uint8_t data) {
	set_cs_before_communicating_with_acc();
	uint8_t acc_address_data = (acc_write_nonincremented_address_header << 6) | address;
	HAL_SPI_Transmit(&hspi1, &acc_address_data, 1, SPI_TIMEOUT);

	uint8_t acc_di_data = data;
	HAL_SPI_Receive(&hspi1, &acc_di_data, 1, SPI_TIMEOUT);
	set_cs_after_communicating_with_acc();
}

/**
 * @brief  Fetch data from WHO_AM_I register to identify an accelerometer device.
 * @param  None
 * @retval None
 */
uint8_t acc_who_am_i() {
	uint8_t who_am_i_reg_addr = 0x0F;
	return read_from_acc_reg(who_am_i_reg_addr);
}

/**
 * @brief  Initialize accelerometer.
 * @param  None
 * @retval None
 */
void acc_init() {
	uint8_t ctrl_reg1_reg_addr = 0x20;
	uint8_t ctrl_reg1_reg_data = 0b01000111;
	write_to_acc_reg(ctrl_reg1_reg_addr, ctrl_reg1_reg_data);
}

/**
 * @brief  Read x-axis acceleration.
 * @param  None
 * @retval x-axis acceleration
 */
int8_t acc_read_x() {
	uint8_t out_x_reg_addr = 0x29;
	return read_from_acc_reg_with_sign(out_x_reg_addr);
}

/**
 * @brief  Read y-axis acceleration.
 * @param  None
 * @retval y-axis acceleration
 */
int8_t acc_read_y() {
	uint8_t out_y_reg_addr = 0x2B;
	return read_from_acc_reg_with_sign(out_y_reg_addr);
}

/**
 * @brief  Read z-axis acceleration.
 * @param  None
 * @retval z-axis acceleration
 */
int8_t acc_read_z() {
	uint8_t out_z_reg_addr = 0x2D;
	return read_from_acc_reg_with_sign(out_z_reg_addr);
}

/**
 * @brief  Read accerelation data and send them via UART.
 * @param  None
 * @retval None
 */
void read_acceleration_and_send() {
	int8_t x_acc, y_acc, z_acc;
	// Read acceleration data
	x_acc = acc_read_x();
	y_acc = acc_read_y();
	z_acc = acc_read_z();
	// sprintf(uart_buffer, "x-axis: %d, y-axis: %d, z-axis: %d\n", x_acc, y_acc, z_acc);
	// send_data_via_uart(uart_buffer);
	show_acc_led_indicator(x_acc, y_acc, z_acc);
	update_main_character(x_acc, y_acc, z_acc);
}

void show_acc_led_indicator(int8_t x_acc, int8_t y_acc, int8_t z_acc) {
	// Left
	if(x_acc < 0) {
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET); // Green LED
	} else {
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET); // Green LED
	}
	// Right
	if(x_acc > 0) {
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET); // Red LED
	} else {
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET); // Red LED
	}
	// Bottom
	if(y_acc < 0) {
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET); // Blue LED
	} else {
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET); // Blue LED
	}
	// Top
	if(y_acc > 0) {
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET); // Orange LED
	} else {
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET); // Orange LED
	}
}

void update_main_character(int8_t x_acc, int8_t y_acc, int8_t z_acc) {
	if(x_acc < -30) {
		// Left
		beat_duration = (int) (base_beat_duration + (((x_acc - 30) * range_speed_up) / 98.0));
	} else if(x_acc > 30) {
		// Right
		beat_duration = (int) (base_beat_duration - (((-x_acc - 30) * range_speed_up) / 98.0));
	} else {
		beat_duration = base_beat_duration;
	}
	// Top
	if(y_acc < -30) {
		rewind_mode = 1;
	} else if(y_acc > 30) {

	} else {
		rewind_mode = 0;
	}
}

/**
 * @brief  (Override) Period elapsed callback in non blocking mode
 * @param  htim: pointer to a TIM_HandleTypeDef structure that contains
 *               the configuration information for TIM module.
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM3) {
		read_acceleration_and_send();
	}
}

// MP45DT02: Microphone
float abs_float(float in) {
	return in < 0 ? -in : in;
}

/**
 * @brief  Calculate volume from PDM stream.
 * @param  data: PDM stream
 * @retval Volume (no unit)
 */
int calculate_volume(uint16_t data) {
	int i, volume = 0;
	for (i = 0; i < 16; i++) {
		volume += data % 2;
		data /= 2;
	}
	return volume;
}

/**
 * @brief  Read PDM stream from microphone module.
 * @param  None
 * @retval 16-bit PDM
 */
uint16_t read_pdm() {
	uint16_t pdm_stream;
	HAL_I2S_Receive(&hi2s2, &pdm_stream, 1, I2S_TIMEOUT);
	return pdm_stream;
}

uint8_t is_loud() {
	int i = 0;
	int volume_accumulate = 0;
	for(; i < 8; i++) {
		uint16_t pdm_stream = read_pdm();
		volume_accumulate += calculate_volume(pdm_stream);
	}
	sprintf(uart_buffer, "Loudness: %d\n\r", volume_accumulate);
	send_data_via_uart_with_size(uart_buffer, strlen(uart_buffer));
	return volume_accumulate > LOUD_THRESHOLD;
}

uint8_t is_loud_accurate() {
	uint8_t i;
	while(pcm_count < 2500) {
		HAL_I2S_Receive(&hi2s2, pdm_buffer, PDM_BUFFER_SIZE, I2S_TIMEOUT);
		for(i = 0; i < PDM_BUFFER_SIZE; i++) {
			pcm_value = -PDM_STREAM_BLOCK_SIZE_BIT / 2;
			pdm_value = pdm_buffer[i];
			while(pdm_value != 0) {
				pcm_value++;
				pdm_value ^= pdm_value & -pdm_value;
			}
			leaky_pcm_buffer += pcm_value;
			leaky_pcm_buffer *= LEAKY_KEEP_RATE;
			leaky_amp_buffer += abs_float(leaky_pcm_buffer);
			leaky_amp_buffer *= LEAKY_KEEP_RATE;
		}
		pcm_count++;
		if(max_amp < leaky_amp_buffer)
			max_amp = leaky_amp_buffer;
		pcm_square += (leaky_amp_buffer / 2500) * leaky_amp_buffer;
	}
	sprintf(uart_buffer, "Loudness: %d\r\n", (int) max_amp);
	send_data_via_uart_with_size(uart_buffer, strlen(uart_buffer));
	pcm_count = 0;
	pcm_square = 0;
	if((int) max_amp > LOUD_THRESHOLD_ACCURATE) {
		max_amp = 0;
		return 1;
	}
	max_amp = 0;
	return 0;
}

// CS43L22: Speaker
int character_note_index_mapping(uint8_t character) {
	if (character == 'a')
		return 1;
	else if (character == 's')
		return 2;
	else if (character == 'd')
		return 3;
	else if (character == 'f')
		return 4;
	else if (character == 'g')
		return 5;
	else if (character == 'h')
		return 6;
	else if (character == 'j')
		return 7;
	else if (character == 'k')
		return 8;
	else if (character == 'l')
		return 9;
	else if (character == 'z')
		return 10;
	else if (character == 'x')
		return 11;
	else if (character == 'c')
		return 12;
	else if (character == 'v')
		return 13;
	else if (character == 'b')
		return 14;
	else if (character == 'n')
		return 15;
	else if (character == 'm')
		return 0;
	else
		return -1;
}

void play_musical_note(int index) {
	if (index < 0 || index > 15) {
		is_playing = 0;
		return;
	}

	HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15);

	// Beep Mix Disable: disable
	write_to_speaker_reg(0x1E, 0x20);
	write_to_speaker_reg(0x1C, musical_note[index]);
	// Beep Configuration: continuous
	write_to_speaker_reg(0x1E, 0xE0);

	int i;
	for (i = 0; i < BEAT; i++) {
		HAL_I2S_Transmit(&hi2s3, (uint16_t *) "a", 100, I2C_TIMEOUT);
	}
	is_playing = 0;
}

void play_musical_note_with_beat(int index, float beat) {
	if (index < -1 || index > 15) {
		is_playing = 0;
		return;
	}

	int _beat_duration = rewind_mode ? beat_duration_rewind_mode : beat_duration;

	int cycles_compromise = (int) (beat * _beat_duration);
	uint8_t note = index >= 0 ? musical_note[index] : 0;

	HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15);

	// Beep Mix Disable: disable
	write_to_speaker_reg(0x1E, 0x20);
	write_to_speaker_reg(0x1C, note);
	// Beep Configuration: continuous
	write_to_speaker_reg(0x1E, 0xE0);

	int i;
	// Beats
	for (i = 0; i < cycles_compromise; i++) {
		if (index != -1) {
			HAL_I2S_Transmit(&hi2s3, (uint16_t *) "a", 100, I2C_TIMEOUT);
		}
	}
	is_playing = 0;
}

void write_to_speaker_reg(uint8_t address, uint8_t data) {
	// Chip address: 100101 followed by the setting of the AD0 pin (0) and R/W bit
	uint8_t chip_address = 0b10010100;
	uint8_t data_concat[2] = { address, data };
	HAL_I2C_Master_Transmit(&hi2c1, chip_address, data_concat, 2, I2C_TIMEOUT);
}

void speaker_init() {
	// Reset
	HAL_GPIO_WritePin(SPEAKER_RESET_GPIO_TYPE, SPEAKER_RESET_GPIO_PIN_NUMBER,
			GPIO_PIN_RESET);
	HAL_Delay(DELAY_DURATION);
	HAL_GPIO_WritePin(SPEAKER_RESET_GPIO_TYPE, SPEAKER_RESET_GPIO_PIN_NUMBER,
			GPIO_PIN_SET);
	HAL_Delay(DELAY_DURATION);

	// Load the required initialization settings according to section 4.11 in data sheet
	write_to_speaker_reg(0x00, 0x99);
	write_to_speaker_reg(0x47, 0x80);
	write_to_speaker_reg(0x32, 0b10111011);
	write_to_speaker_reg(0x32, 0b00111011);
	write_to_speaker_reg(0x00, 0x00);

	// Beep & Tone Configuration: set BEEP[1:0] to 11 (continuous)
	write_to_speaker_reg(0x1E, 0xC0);

	// Change beep volume
	write_to_speaker_reg(0x1D, BEEP_VOLUME);

	// Set the “Power Ctl 1” register (0x02) to 0x9E
	write_to_speaker_reg(0x02, 0x9E);
}

// Control
void print_area() {
	uint8_t i, j;
	for(i = 0; i < AREA_HEIGHT; i++) {
		for(j = 0; j < AREA_WIDTH; j++) {
			if(i == main_character_i && j == main_character_j)
				send_data_via_uart_with_size("#", 1);
			else {
				send_data_via_uart_with_size(" ", 1);
			}
		}
		send_data_via_uart_with_size("\r\n", 2);
	}
}

void flush_screen() {
	uint8_t i;
	for(i = 0; i < 2 * AREA_HEIGHT + 20; i++) {
		send_data_via_uart_with_size("\r\n", 2);
	}
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
