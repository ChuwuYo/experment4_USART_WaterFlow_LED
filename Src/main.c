/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

/* USER CODE BEGIN PV */
#include "string.h"
uint8_t dataBuf[128]={0};
const char stringMode1[8]="mode_1#";
const char stringMode2[8]="mode_2#";
const char stringStop[8]="stop#";
int8_t ledMode=-1;
uint8_t LED_value=0xFD;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t cror(uint8_t numbers,uint8_t bits)
{
	uint8_t right=numbers>>bits;
	uint8_t left=numbers<<(8-bits);
	uint8_t temp=left|right;
	return temp;
}

uint8_t crol(uint8_t numbers,uint8_t bits)
{
	uint8_t right=numbers<<bits;
	uint8_t left=numbers>>(8-bits);
	uint8_t temp=left|right;
	return temp;
}
void LED_State_Set(uint8_t numbers)
{
	HAL_GPIO_WritePin(LED0_GPIO_Port,LED0_Pin,((numbers&0x01)>>0)?GPIO_PIN_SET:GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,((numbers&0x02)>>1)?GPIO_PIN_SET:GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,((numbers&0x04)>>2)?GPIO_PIN_SET:GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED3_GPIO_Port,LED3_Pin,((numbers&0x08)>>3)?GPIO_PIN_SET:GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED4_GPIO_Port,LED4_Pin,((numbers&0x10)>>4)?GPIO_PIN_SET:GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED5_GPIO_Port,LED5_Pin,((numbers&0x20)>>5)?GPIO_PIN_SET:GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED6_GPIO_Port,LED6_Pin,((numbers&0x40)>>6)?GPIO_PIN_SET:GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED7_GPIO_Port,LED7_Pin,((numbers&0x80)>>7)?GPIO_PIN_SET:GPIO_PIN_RESET);
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
	__HAL_UART_ENABLE_IT(&huart1,UART_IT_RXNE);
	printf("hello world.\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		//printf("hello world.\r\n");
    /* USER CODE END WHILE */
		if(UART_RX_STATE_DEAL==uart1RxState)
		{
			if(strstr((const char*)uart1RxBuf,stringMode1)!=NULL)
			{
				printf("I'm in mode_1!\r\n");
				ledMode=1;
			}
			else if(strstr((const char*)uart1RxBuf,stringMode2)!=NULL)
			{
				printf("I'm in mode_2!\r\n");
				ledMode=2;
			}
			else if(strstr((const char*)uart1RxBuf,stringStop)!=NULL)
			{
				printf("I'm stop!\r\n");
				ledMode=0;
			}
			__HAL_UART_ENABLE_IT(&huart1,UART_IT_RXNE);
			
			uart1RxState=UART_RX_STATE_READY;
			uart1RxCounter=0;
			memset(uart1RxBuf,0,UART1_RCV_MAX);			
		}
		switch(ledMode)
		{
			case 1:				
				LED_State_Set(LED_value);
				LED_value=cror(LED_value,1);
				HAL_Delay(1000);
				break;
			case 2:				
				LED_State_Set(LED_value);
				LED_value=crol(LED_value,1);
				HAL_Delay(1000);
				break;
			case 0:
				LED_value=0xFF;
				LED_State_Set(LED_value);	
				LED_value=0xFD;			
				break;
			default:
				break;
		}

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

  /** Initializes the CPU, AHB and APB busses clocks 
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
  /** Initializes the CPU, AHB and APB busses clocks 
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
void USER_UART_IRQHandler(UART_HandleTypeDef *huart)
{
	if((__HAL_UART_GET_FLAG(huart,UART_FLAG_RXNE)!=RESET))
	{
		if(UART_RX_STATE_READY==uart1RxState)
		{
			uart1RxState=UART_RX_STATE_START;
			__HAL_UART_ENABLE_IT(huart,UART_IT_IDLE);
			uart1RxCounter=0;
			uart1RxBuf[uart1RxCounter]=(uint8_t)((*huart).Instance->DR&(uint8_t)0x00FF);
			uart1RxCounter++;
		}
		else if (UART_RX_STATE_START==uart1RxState)
		{
		uart1RxBuf[uart1RxCounter]=(uint8_t)((*huart).Instance->DR&(uint8_t)0x00FF);
		uart1RxCounter++;
		}
		__HAL_UART_CLEAR_FLAG(huart,UART_FLAG_RXNE);
	}
	
	if(__HAL_UART_GET_FLAG(huart,UART_FLAG_IDLE)!=RESET)
	{
		__HAL_UART_DISABLE_IT(huart,UART_IT_IDLE);
		__HAL_UART_DISABLE_IT(huart,UART_IT_RXNE);		
		uart1RxState=UART_RX_STATE_DEAL;
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
