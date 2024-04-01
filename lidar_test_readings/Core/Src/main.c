/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usart_ring.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RESPONSE_BYTES_COUNT 7
#define SCAN_BYTES_COUNT	 5
#define DENSE_MODE
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t buf[10];
float angle, dist_mm;
usart_ring_t lidar_rx;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void start_scan(void) {
	HAL_UART_StateTypeDef status;
	uint8_t start_cmd[] = {0xA5, 0x20};
	__HAL_UART_DISABLE_IT(&huart1, UART_IT_RXNE);
	HAL_UART_Transmit_IT(&huart1, start_cmd, sizeof(start_cmd));
	do {
		status = HAL_UART_GetState(&huart1);
	} while (status != HAL_UART_STATE_ERROR || status != HAL_UART_STATE_BUSY_TX);
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
}

void stop_scan(void) {
	HAL_UART_StateTypeDef status;
	uint8_t stop_cmd[] = {0xA5, 0x25};
	__HAL_UART_DISABLE_IT(&huart1, UART_IT_RXNE);
	HAL_UART_Transmit_IT(&huart1, stop_cmd, sizeof(stop_cmd));
	do {
		status = HAL_UART_GetState(&huart1);
	} while (status != HAL_UART_STATE_ERROR || status != HAL_UART_STATE_BUSY_TX);
	HAL_Delay(10);
	usart_ring_clear(&lidar_rx);
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
}

void reset_lidar(void) {
	HAL_UART_StateTypeDef status;
	uint8_t reset_cmd[] = {0xA5, 0x40};
	__HAL_UART_DISABLE_IT(&huart1, UART_IT_RXNE);
	HAL_UART_Transmit_IT(&huart1, reset_cmd, sizeof(reset_cmd));
	do {
		status = HAL_UART_GetState(&huart1);
	} while (status != HAL_UART_STATE_ERROR || status != HAL_UART_STATE_BUSY_TX);
	usart_ring_clear(&lidar_rx);
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
}

void get_lidar_values(void) {
	// TODO: нужно взаимодействие с кольцевым буфером
	volatile uint8_t s, not_s;

	while (usart_ring_available(&lidar_rx)) {
		buf[0] = usart_ring_read(&lidar_rx);
		s = buf[0] & 0x01;
		not_s = (buf[0] & 0x02) >> 1;
		if (s != not_s) {
			buf[1] = usart_ring_read(&lidar_rx);
			if (buf[1] & 0x01) { // "c" bit should be equal to 1
				if (buf[0] == 0x0A) {
					for (uint8_t i = 0; i < 3; i++) usart_ring_read(&lidar_rx);
				} else if (buf[0] == 0xBE) {
					for (uint8_t i = 2; i < SCAN_BYTES_COUNT; i++) buf[i] = usart_ring_read(&lidar_rx);
					angle = ((buf[1] >> 1) | (buf[2] << 7)) / 64.0f;
#ifdef DENSE_MODE
					dist_mm = (buf[3] | (buf[4] << 8));
#else
					dist_mm = (buf[3] | (buf[4] << 8)) / 4.0f;
#endif
				}
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
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_TXE);
  usart_ring_init(&lidar_rx, &huart1, 500);
  start_scan();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
	  get_lidar_values();
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

