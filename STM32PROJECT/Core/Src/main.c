/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include <stdio.h>
#include "software_timer.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX_BUFFER_SIZE 200
#define RTS 1
#define OK 2
#define THREE_SECOND 3
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t temp = 0;  // Biến để lưu dữ liệu nhận từ UART
uint8_t buffer[MAX_BUFFER_SIZE];  // Bộ đệm để lưu dữ liệu nhận từ UART
uint8_t index_buffer = 0;  // Chỉ số hiện tại của bộ đệm
uint8_t buffer_flag = 0;  // C�? để đánh dấu dữ liệu đã được nhận hoàn tất
int command_state = THREE_SECOND;
uint8_t cmd_data[MAX_BUFFER_SIZE];
static uint32_t last_adc_value = 0;
static uint32_t adc_value = 1234;
char adc_response[20];
int transmit_counter=0;
static enum {
		PARSER_IDLE,
		PARSER_RECEIVING,
		PARSER_DONE
	} parser_state = PARSER_IDLE;
   static enum {
		   UART_IDLE,
		   UART_SEND_ADC,
		   UART_WAIT_OK,
		   UART_RETRANSMIT
	   } uart_state = UART_IDLE;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)  // Kiểm tra nếu là UART2
    {
    //	HAL_UART_Transmit(&huart2, &temp, 1, 50);
        buffer[index_buffer++] = temp; // Lưu dữ liệu nhận được vào buffer tại vị trí index_buffer
        // Nếu bộ đệm đầy, reset chỉ số v�? 0
        if (index_buffer == MAX_BUFFER_SIZE)
            index_buffer = 0;
    }

    //�?ặt c�? buffer_flag thành 1 để đánh dấu dữ liệu đã được nhận
    buffer_flag = 1;

    // Tiếp tục nhận dữ liệu từ UART (bật chế độ ngắt nhận UART)
    HAL_UART_Receive_IT(&huart2, &temp, 1);
}

void command_parser_fsm() {
    static uint8_t cmd_idx = 0;
    char characters = buffer[index_buffer - 1];  // Lấy ký tự mới nhất từ buffer
    switch (parser_state) {
        case PARSER_IDLE:
            if (characters == '!') {  // Bắt đầu chuỗi lệnh
                cmd_idx = 0;
                cmd_data[cmd_idx++] = characters;
                parser_state = PARSER_RECEIVING;
            }
            break;

        case PARSER_RECEIVING:
            cmd_data[cmd_idx++] = characters;  // Lưu ký tự
            if (characters == '#') {  // Kết thúc chuỗi lệnh
                parser_state = PARSER_DONE;
                cmd_data[cmd_idx] = '\0';  // Kết thúc chuỗi
            }
            break;

        case PARSER_DONE:
            // Kiểm tra chuỗi lệnh
            if (strcmp((char *)cmd_data, "!RTS#") == 0) {
                command_state = RTS;  // Lệnh yêu cầu gửi giá trị ADC
            } else if (strcmp((char *)cmd_data, "!OK#") == 0) {
                command_state = OK;  // �?ánh dấu kết thúc giao tiếp
            }
            parser_state = PARSER_IDLE;  // Quay lại trạng thái ban đầu
            break;

        default:
            parser_state = PARSER_IDLE;
            break;
    }

    // Reset index_buffer khi đạt giới hạn
    if (index_buffer >= MAX_BUFFER_SIZE) {
        index_buffer = 0;
    }
}


void uart_communication_fsm() {

    switch (uart_state) {
        case UART_IDLE:
        //	transmit_counter=0;
            if (command_state == RTS) {
            	HAL_GPIO_WritePin(LED_YELLOW_GPIO_Port, LED_YELLOW_Pin, RESET);
              //  adc_value = HAL_ADC_GetValue(&hadc1);  // �?�?c giá trị ADC
                snprintf(adc_response, sizeof(adc_response), "!ADC=%lu#\r\n", (uint32_t)adc_value);  // �?ịnh dạng chuỗi lệnh
           //     HAL_UART_Transmit(&huart2, (uint8_t *)adc_response, strlen(adc_response), 100);  // Gửi qua UART
                last_adc_value = adc_value;  // Lưu giá trị ADC
                uart_state = UART_RETRANSMIT;  // Chuyển sang trạng thái ch�? phản hồi
              //  setTimer(0, 3000);  // Bắt đầu đếm th�?i gian 3 giây
            }
            break;

        case UART_WAIT_OK:
            if (isTimerExpired(0)) {  // Hết th�?i gian ch�?
                uart_state = UART_RETRANSMIT;  // Chuyển sang trạng thái gửi lại
            }
            if(transmit_counter==5&&(command_state != OK)){
            	uart_state = UART_RETRANSMIT;
            }
            if (command_state == OK) {  // Nhận phản hồi !OK#
            	HAL_GPIO_WritePin(LED_YELLOW_GPIO_Port, LED_YELLOW_Pin, SET);
            	transmit_counter=0;
            	command_state=THREE_SECOND;
                uart_state = UART_IDLE;
            }
            break;

        case UART_RETRANSMIT:
        	if(transmit_counter<5){
				snprintf(adc_response, sizeof(adc_response), "!ADC=%lu#\r\n", (uint32_t)last_adc_value);  // �?ịnh dạng chuỗi ADC
				HAL_UART_Transmit(&huart2, (uint8_t *)adc_response, strlen(adc_response), 100);  // Gửi lại qua UART
				if(transmit_counter<4) setTimer(0, 3000);
				  // Reset lại th�?i gian ch�?
//				if(transmit_counter==4) clear_timer_flag(0);
				transmit_counter++;
				uart_state = UART_WAIT_OK;  // Chuyển v�? trạng thái ch�? phản hồi
        	}
        	else {
        	//	uart_state = UART_IDLE;
        	//	command_state=THREE_SECOND;
        	//	transmit_counter=0;
        		uart_state = UART_WAIT_OK;
        	}
            break;

        default:
            uart_state = UART_IDLE;
            break;
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
  MX_ADC1_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart2, &temp, 1);
  HAL_TIM_Base_Start_IT(&htim2);
//  HAL_UART_RxCpltCallback(&huart2);
 // HAL_ADC_Start(&hadc1);
     //HAL_ADC_GetValue(&hadc1);
     command_state = THREE_SECOND;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  if (buffer_flag == 1) {
	  	 	          command_parser_fsm();  // Phân tích lệnh
	  	 	          buffer_flag = 0;       // Reset c�? nhận buffer
	  	 	      }
	  	 	      uart_communication_fsm();
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 7999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 9;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
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
  huart2.Init.BaudRate = 9600;
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
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_YELLOW_GPIO_Port, LED_YELLOW_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_YELLOW_Pin */
  GPIO_InitStruct.Pin = LED_YELLOW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_YELLOW_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	timerRun();
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
