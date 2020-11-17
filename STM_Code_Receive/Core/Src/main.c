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
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "nrf24.h"
#include <stdbool.h>
#include "pid.h"
#include "systick.h"
//#include "mpu6050.h"
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

float x_pos, y_pos;											//feedback position from RF
float right_speed, left_speed;
bool htim2_check = 0;

uint32_t g_systick= 0;

bool new_measure = 0;
bool new_pos_measure = 0;

uint16_t right_wheel_count = 0, left_wheel_count = 0;		//use to count encoder of 2 wheels
uint16_t count_tick = 0;

bool timer2_flag = false;

extern uint8_t nRF24_payload[4];
/*
 * Byte 0: x decimal
 * 			1: x fraction
 * 			2: y decimal
 * 			3: y fraction
 */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void PWM_ChangeDuty(TIM_HandleTypeDef* htim, uint8_t duty);			//only used for TIM3 and TIM4 to control motor

void doc_encoder(void);
void WheelRotationCalculate(void);
void task_100ms(void);
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
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_TIM_Base_Start_IT(&htim1);

	//feedback position variables
		float x_fb_old;
		float y_fb_old;
		float x_fb;
		float y_fb;
		float phi_fb;
	
	//Set parameter for control motor
		float v_ref, w_ref;									//output of kinematic control
		float v_measure;
		float w_measure;
		
		
		float left_torque, right_torque;		
		float v_error = 0, w_error = 0;
		
		float delta_r, delta_l;								//output of compensato
		
		uint8_t left_duty, right_duty;
		
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
	
	HAL_Delay(1000);
	
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, LEFT_2_Pin |RIGHT_2_Pin, GPIO_PIN_RESET);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	runRadio();
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		//check position, calculate feedback value
		task_100ms();
		//if receive new position fb
		if(new_pos_measure == 1) {
			//handle data
			x_fb_old = x_fb;
			y_fb_old = y_fb;
			
			//getting new feedback position from nrf_payload
			x_fb = (float)nRF24_payload[0] + nRF24_payload[1]*0.01;
			y_fb = (float)nRF24_payload[2] + nRF24_payload[3]*0.01;
			//calculate phi_fb
			
			phi_fb = atan2(y_fb - y_fb_old, x_fb - x_fb_old);
			
			//after finishing handling data
			new_pos_measure = 0;
		}
		
		//Set parameter for control motor
		v_measure = PID_MeasureVelocity(left_speed, right_speed);
		w_measure = PID_MeasureRotation(left_speed, right_speed);
		
		v_error = 0;
		w_error = 0;			//used for Compensator	
		//motor control:
		PID_KinematicControl(x_fb, y_fb, phi_fb, &v_ref, &w_ref);
		PID_DynamicInverse(v_ref, w_ref, v_measure, w_measure, &left_torque, &right_torque);
		for(uint8_t i = 0; i < 10; i++) {
			while(new_measure == 0);
			new_measure = 0;
			WheelRotationCalculate();
			v_measure = PID_MeasureVelocity(left_speed, right_speed);
			w_measure = PID_MeasureRotation(left_speed, right_speed);
			PID_Compensator(v_ref, w_ref, v_measure, w_measure, &v_error, &w_error, &delta_r, &delta_l);
			PID_OutputDynamicControl(right_torque, left_torque, delta_r, delta_l, &right_torque, &left_torque);
			PID_DynamicModel(right_torque, left_torque, v_measure, w_measure, &left_duty, &right_duty);
			PWM_ChangeDuty(&htim3, left_duty);
			PWM_ChangeDuty(&htim4, right_duty);
		}
		
		
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
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
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
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if(GPIO_Pin == GPIO_PIN_3) {
		right_wheel_count++;
		return;
	}
	if(GPIO_Pin == GPIO_PIN_4) {
		left_wheel_count++;
		return;
	}
}

void PWM_ChangeDuty(TIM_HandleTypeDef* htim, uint8_t duty) {
	__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1,duty);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {		//measure pulse of encoder timeout
	
	if(htim->Instance == TIM2) 
		{
		
		timer2_flag = true;
		//htim2.Instance->CNT = 0x00;					//set counter = 0 to restart
		//measure in 10ms
		htim2_check = 1;
		new_measure = 1;
		count_tick++;
	}
	
	if(htim->Instance == TIM1) {
		g_systick++;
	}
}

void WheelRotationCalculate(void) {
		if(htim2_check == 1) {
			right_speed = (float)right_wheel_count * 5 * PI;	//(count/ (time * PPR)) *2 PI(rad/s) = (count / (0.01*20)) *2PI
			left_speed = (float)left_wheel_count * 5 * PI;
			right_wheel_count = 0;
			left_wheel_count = 0;
			htim2_check = 0;
		}
}
void task_100ms(void)
{
	if(timer2_flag)
	{
		if(count_tick >= 10)
		{
			//UART_SendStr("in timer ");
			radio_receive();
			new_pos_measure = 1;
			count_tick = 0;
		}
	}
	timer2_flag = false;
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
