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
#include "app_uart.h"

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
#define PID_INNER_COUNT 10
#define TIMER_LEFT &htim4
#define TIMER_RIGHT	&htim3
#define EXTI_INT_ON 0x18
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

float x_pos, y_pos; //feedback position from RF
float right_speed, left_speed;
bool htim2_check = 0;

uint32_t g_systick = 0;

bool new_measure = 0;
bool new_pos_measure = 0;

uint16_t right_wheel_count = 0, left_wheel_count = 0; //use to count encoder of 2 wheels
uint16_t tick_10ms = 0; 	

bool timer2_flag = false;

extern uint8_t nRF24_payload[6];
/*
 * Byte 0: x decimal
 * 			1: x fraction
 * 			2: y decimal
 * 			3: y fraction
 */

//feedback position variables
float x_fb_old;
float y_fb_old;
float x_fb;
float y_fb;
float phi_fb;

//Set parameter for control motor
float v_ref, w_ref; //output of kinematic control
float v_measure;
float w_measure;

float left_torque, right_torque;
float v_error = 0, w_error = 0;

float delta_r, delta_l; //output of compensato

uint8_t left_duty, right_duty;

uint8_t inner_loop_count = 0;

/*
	Variables for testing purpose
*/

uint8_t uart_send[13];
uint8_t temp;
uint8_t count = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void PWM_ChangeDuty(TIM_HandleTypeDef *htim, uint8_t duty); //only used for TIM3 and TIM4 to control motor

//void doc_encoder(void);
void WheelRotationCalculate(void);
void task_100ms(void);
void PID_InnerLoopTask(void);
void PID_OutterLoopTask(void);	
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
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);	

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

	HAL_Delay(1000);

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, LEFT_2_Pin | RIGHT_2_Pin, GPIO_PIN_RESET);
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_TIM_Base_Start_IT(&htim1);
	HAL_Delay(1000);
	runRadio();
	
	/*
	//test
	uart_send[4] = '\r';
	uart_send[5] = '\n';	
	uart_send[10] = '\r';
	uart_send[11] = '\n';
	uart_send[12] = 'e';
	*/
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
		/***************** Main Program ********************/
		
		//check position, calculate feedback value
		task_100ms();
		PID_OutterLoopTask();
		PID_InnerLoopTask();
		PWM_ChangeDuty(TIMER_RIGHT, right_duty);
		PWM_ChangeDuty(TIMER_LEFT, left_duty);
		
		//Note: changing duty to 100ms
		//Note2: duty cycle = 30% => run 150rpm
		
		//Note3: duty = 99% => run 240 - 339rpm
		/*
		PWM_ChangeDuty(&htim3, 99);
		PWM_ChangeDuty(&htim4, 99);
		WheelRotationCalculate();
		*/
		/***************** End Main Prg ********************/
		
		
		/***************** Test Program ********************/
		/*
		// Measure speed of motor
		if(htim2_check == 1) {
			WheelRotationCalculate();
			uart_send[0] = (uint8_t)right_speed/10 + 48;
			uart_send[1] = (uint8_t)right_speed%10 + 48;
			temp = (uint8_t)((right_speed - (uint8_t)right_speed)*100);
			uart_send[2] = temp/10 + 48;
			uart_send[3] = temp%10 + 48;
		
			uart_send[6] = (uint8_t)left_speed/10 + 48;
			uart_send[7] = (uint8_t)left_speed%10 + 48;
			temp = (uint8_t)((left_speed - (uint8_t)left_speed)*100);
			uart_send[8] = temp/10 + 48;
			uart_send[9] = temp%10 + 48;
			
			count++;
			if(count == 10) {
				count = 0;
				HAL_UART_Transmit(&huart2, (uint8_t*)uart_send, 13, HAL_MAX_DELAY);
			}
			
		}
		PWM_ChangeDuty(&htim3, 30);
		PWM_ChangeDuty(&htim4, 30); 
		*/ 
		/***************** End Test Prg ********************/
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

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == GPIO_PIN_3)
	{
		right_wheel_count++;
		return;
	}
	if (GPIO_Pin == GPIO_PIN_4)
	{
		left_wheel_count++;
		return;
	}
}

void PWM_ChangeDuty(TIM_HandleTypeDef *htim, uint8_t duty)
{
	__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, duty);
}

void PID_OutterLoopTask(void)
{
	if (inner_loop_count >= PID_INNER_COUNT)
	{
		//Set parameter for control motor
		v_measure = PID_MeasureVelocity(left_speed, right_speed);
		w_measure = PID_MeasureRotation(left_speed, right_speed);

		//v_error = 0;
		//w_error = 0; //used for Compensator
		//motor control:
		PID_KinematicControl(x_fb, y_fb, phi_fb, &v_ref, &w_ref);
		PID_DynamicInverse(v_ref, w_ref, v_measure, w_measure, &left_torque, &right_torque);
		inner_loop_count = 0;
		//PID_InnerLoopTask();
	}
}

void PID_ConvertPositionRF(void)
{
	//handle data
	x_fb_old = x_fb;
	y_fb_old = y_fb;

	//getting new feedback position from nrf_payload
	x_fb = (float)nRF24_payload[1];
	y_fb = (float)nRF24_payload[2];
	//calculate phi_fb

	phi_fb = atan2(y_fb - y_fb_old, x_fb - x_fb_old);

	//after finishing handling data
	new_pos_measure = 0;
}
void PID_InnerLoopTask(void)
{
	inner_loop_count++;
	WheelRotationCalculate();
	v_measure = PID_MeasureVelocity(left_speed, right_speed);
	w_measure = PID_MeasureRotation(left_speed, right_speed);
	PID_Compensator(v_ref, w_ref, v_measure, w_measure, &v_error, &w_error, &delta_r, &delta_l);
	PID_OutputDynamicControl(right_torque, left_torque, delta_r, delta_l, &right_torque, &left_torque);
	PID_DynamicModel(right_torque, left_torque, v_measure, w_measure, &left_duty, &right_duty);
	//PWM_ChangeDuty(&htim3, left_duty);
	//PWM_ChangeDuty(&htim4, right_duty);
	//PWM_ChangeDuty(&htim3, 50);
	
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{ //measure pulse of encoder timeout

	if (htim->Instance == TIM2)
	{

		timer2_flag = true;
		//htim2.Instance->CNT = 0x00;					//set counter = 0 to restart
		//measure in 10ms
		htim2_check = 1;
		//new_measure = 1;
		tick_10ms++;
	}

	//if (htim->Instance == TIM1)
	//{
	//	g_systick++;
	//}
}

void WheelRotationCalculate(void)
{
	if (htim2_check == 1)
	{
		//Turn off interrupt
		HAL_TIM_Base_Stop_IT(&htim2);
		//HAL_NVIC_DisableIRQ(EXTI3_IRQn);
		//HAL_NVIC_DisableIRQ(EXTI4_IRQn);
		//_disable_irq();
		EXTI->IMR  &= ~(EXTI_INT_ON);		
		/*
		uint8_t dat[5];
		dat[0] = right_wheel_count >> 8;
		dat[1] = right_wheel_count & 0xFF;
		
		dat[2] = left_wheel_count >> 8;
		dat[3] = left_wheel_count &  0xFF;
		dat[4] = 0xAA;
		*/
		//HAL_UART_Transmit(&huart2, (uint8_t*)&dat, 5, HAL_MAX_DELAY);
		//HAL_UART_Transmit(&huart2, (uint8_t*)&right_temp, 2, HAL_MAX_DELAY);
		//HAL_UART_Transmit(&huart2, (uint8_t*)&left_temp, 2, HAL_MAX_DELAY);
		//HAL_UART_Transmit(&huart2, (uint8_t*) 0xAA, 1, HAL_MAX_DELAY);
		
		right_speed = (float)right_wheel_count * PI; //(count/ (time * PPR)) *2 PI(rad/s) 
		left_speed = (float)left_wheel_count * PI;
		
		right_wheel_count = 0;
		left_wheel_count = 0;
		
		htim2_check = 0;
		
		//enable interrupt
		//HAL_NVIC_EnableIRQ(EXTI3_IRQn);
		//HAL_NVIC_EnableIRQ(EXTI4_IRQn);
		HAL_TIM_Base_Start_IT(&htim2);
		EXTI->IMR |= EXTI_INT_ON;
	}
}
void task_100ms(void)
{
	if (timer2_flag)
	{
		if (tick_10ms >= 10)
		{
			//UART_SendStr("in timer ");
			radio_receive();
			//PID_InnerLoopTask();
			PID_ConvertPositionRF();
			tick_10ms = 0;
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
