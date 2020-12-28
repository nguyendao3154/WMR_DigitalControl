/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "app_uart.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "nrf24.h"
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ENCODER_PPR 20
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t position_buffer[4];
uint8_t receive_buffer;
uint8_t temp_buffer[6] = {0xbd, 0x00, 0x00, 0x00, 0x00, 0xed};
extern uint8_t nRF24_payload[6];
bool uart_receive_command = false;
uint8_t buffer_index = 0;
/*
 * Byte 0: x decimal
 * 		1: x fraction
 * 		2: y decimal
 * 		3: y fraction
 */

float x_pos, y_pos;                                   //feedback position from RF
uint16_t right_wheel_count = 0, left_wheel_count = 0; //use to count encoder of 2 wheels
float right_speed, left_speed;
uint16_t count_tick = 0;

bool timer2_flag = false;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void PWM_ChangeDuty(TIM_HandleTypeDef *htim, uint8_t duty); //only used for TIM3 and TIM4 to control motor
void task_100ms(void);
void packing_packet(void);
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
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart2, (uint8_t *)&receive_buffer, sizeof(temp_buffer) / 6);
  HAL_TIM_Base_Start_IT(&htim2);
  runRadio();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    //uint8_t data = 'a';
    task_100ms();
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
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL5;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == huart2.Instance)
  {
    // HAL_UART_Receive_IT(&huart2, receive_buffer, sizeof(receive_buffer));
    // UART_SendStr("in UART");
    HAL_UART_Receive_IT(&huart2, (uint8_t *)&receive_buffer, sizeof(temp_buffer) / 6);
    if (*(&receive_buffer) == START_BYTE)
    {
      buffer_index = 0;
      uart_receive_command = true;
      UART_SendStr("received BD\n");
			temp_buffer[5] = 0x00;
    }
    if (uart_receive_command)
    {
      temp_buffer[buffer_index] = receive_buffer;
      UART_SendStr("uart_receive_command = 1 \n");
      buffer_index++;
    }
    if ((buffer_index == 6) && (temp_buffer[5] == STOP_BYTE))
    {
      temp_buffer[buffer_index] = *(&receive_buffer);
      UART_SendStr("received ED \n");
      buffer_index = 0;
      uart_receive_command = false;
    }
    //  UART_SendBufHex((char*)receive_buffer, sizeof(receive_buffer));
  }
}

void task_100ms(void)
{
  if (timer2_flag)
  {
    if (count_tick >= 10)
    {
      //UART_SendStr("in timer ");
      packing_packet();
      transmitRF();
      //UART_SendBufHex((char *)&temp_buffer, sizeof(temp_buffer));
      //UART_SendStr("\n");
      //UART_SendHex8(buffer_index);
      //UART_SendStr("\n");
      count_tick = 0;
    }
  }
  timer2_flag = false;
}

void packing_packet(void)
{
  if (UART_CheckPackage(temp_buffer))
  {
    for (int i = 0; i < MAX_BYTE; i++)
    {
      nRF24_payload[i] = temp_buffer[i];
    }
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{ //finish reading data from RX FIFO NRF
}

void PWM_ChangeDuty(TIM_HandleTypeDef *htim, uint8_t duty)
{
  __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, duty);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{ //measure pulse of encoder timeout
  if (htim->Instance == TIM2)
  {

    timer2_flag = true;
    //htim2.Instance->CNT = 0x00;					//set counter = 0 to restart
    count_tick++;
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
