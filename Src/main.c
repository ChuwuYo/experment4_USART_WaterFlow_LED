/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body with Modbus RTU protocol
  *
  * Modbus RTU命令说明:
  * ===================
  *
  * 设备地址: 0x01
  * 寄存器映射:
  * - 0x0000: LED模式控制 (0=停止, 1=右移, 2=左移)
  * - 0x0001: LED值控制
  *
  * 命令示例（使用Python生成器自动生成CRC）:
  *
  * 1. 启动右移模式 (mode_1#):
  *    python3 modbus_command_generator.py 1 mode1
  *    命令: echo -ne '\x01\x06\x00\x00\x00\x01\x48\x0A' > /dev/ttyUSB0
  *
  * 2. 启动左移模式 (mode_2#):
  *    python3 modbus_command_generator.py 1 mode2
  *    命令: echo -ne '\x01\x06\x00\x00\x00\x02\x08\x0B' > /dev/ttyUSB0
  *
  * 3. 停止模式 (stop#):
  *    python3 modbus_command_generator.py 1 stop
  *    命令: echo -ne '\x01\x06\x00\x00\x00\x00\xC9\x0A' > /dev/ttyUSB0
  *
  * 4. 读取当前状态:
  *    python3 modbus_command_generator.py 1 read
  *    命令: echo -ne '\x01\x03\x00\x00\x00\x01\x84\x0A' > /dev/ttyUSB0
  *
  * 或者使用Python生成器:
  * python3 modbus_command_generator.py 1 mode1    # 右移模式
  * python3 modbus_command_generator.py 1 mode2    # 左移模式
  * python3 modbus_command_generator.py 1 stop     # 停止模式
  * python3 modbus_command_generator.py 1 read     # 读取状态
  *
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
#include "stdint.h"

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

// Modbus相关定义
#define MODBUS_DEVICE_ADDRESS  0x01  // 从设备地址
#define MODBUS_BUFFER_SIZE     256   // Modbus缓冲区大小

// Modbus功能码
#define MODBUS_READ_HOLDING_REGISTERS  0x03
#define MODBUS_WRITE_SINGLE_REGISTER   0x06
#define MODBUS_WRITE_MULTIPLE_REGISTERS 0x10

// Modbus寄存器地址
#define LED_MODE_REGISTER      0x0000  // LED模式控制寄存器
#define LED_VALUE_REGISTER     0x0001  // LED值寄存器

// Modbus数据缓冲区
uint8_t modbusRxBuffer[MODBUS_BUFFER_SIZE];
uint8_t modbusTxBuffer[MODBUS_BUFFER_SIZE];
uint16_t modbusRxCount = 0;
uint8_t modbusRxComplete = 0;

// Modbus保持寄存器
uint16_t holdingRegisters[10] = {0};  // 保持10个寄存器

// LED控制变量
int8_t ledMode = -1;
uint8_t LED_value = 0xFD;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

// Modbus函数声明
uint16_t modbus_crc16(uint8_t *data, uint16_t length);
void modbus_process_frame(uint8_t *rxBuffer, uint16_t length);
void modbus_send_response(uint8_t function_code, uint16_t register_address, uint16_t register_value, uint8_t is_single);
void modbus_send_read_response(uint16_t register_address, uint16_t register_count);
void modbus_send_exception(uint8_t function_code, uint8_t exception_code);

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

/**
 * @brief 计算Modbus CRC16校验码
 * @param data 数据缓冲区
 * @param length 数据长度
 * @return CRC16校验码
 */
uint16_t modbus_crc16(uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFF;
    uint16_t i, j;

    for (i = 0; i < length; i++) {
        crc ^= data[i];
        for (j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

/**
 * @brief 处理接收到的Modbus数据帧
 * @param rxBuffer 接收缓冲区
 * @param length 数据长度
 */
void modbus_process_frame(uint8_t *rxBuffer, uint16_t length)
{
    uint16_t crc_received, crc_calculated;
    uint8_t device_address, function_code;
    uint16_t register_address, register_value, register_count;

    // 验证数据长度（最小帧长度：地址+功能码+CRC = 4字节）
    if (length < 4) {
        return;
    }

    // 验证CRC校验
    crc_received = (rxBuffer[length-1] << 8) | rxBuffer[length-2];
    crc_calculated = modbus_crc16(rxBuffer, length - 2);
    if (crc_received != crc_calculated) {
        return;  // CRC校验失败
    }

    // 解析Modbus帧
    device_address = rxBuffer[0];
    function_code = rxBuffer[1];

    // 检查设备地址
    if (device_address != MODBUS_DEVICE_ADDRESS) {
        return;  // 不是本设备
    }

    switch (function_code) {
        case MODBUS_WRITE_SINGLE_REGISTER:
            // 写单个寄存器：地址(2) + 功能码(1) + 寄存器地址(2) + 寄存器值(2) + CRC(2)
            if (length == 8) {
                register_address = (rxBuffer[2] << 8) | rxBuffer[3];
                register_value = (rxBuffer[4] << 8) | rxBuffer[5];

                if (register_address < 10) {  // 确保寄存器地址有效
                    holdingRegisters[register_address] = register_value;

                    // 更新LED控制变量
                    if (register_address == LED_MODE_REGISTER) {
                        ledMode = (int8_t)register_value;
                    } else if (register_address == LED_VALUE_REGISTER) {
                        LED_value = (uint8_t)register_value;
                    }

                    // 发送确认回复
                    modbus_send_response(function_code, register_address, register_value, 1);
                }
            }
            break;

        case MODBUS_WRITE_MULTIPLE_REGISTERS:
            // 写多个寄存器：地址(1) + 功能码(1) + 起始地址(2) + 寄存器数量(2) + 字节数(1) + 数据(n) + CRC(2)
            if (length >= 9) {
                register_address = (rxBuffer[2] << 8) | rxBuffer[3];
                register_count = (rxBuffer[4] << 8) | rxBuffer[5];
                uint8_t byte_count = rxBuffer[6];

                if (length == (9 + byte_count) && (byte_count == register_count * 2)) {
                    // 写入寄存器值
                    for (uint16_t i = 0; i < register_count && (register_address + i) < 10; i++) {
                        holdingRegisters[register_address + i] =
                            (rxBuffer[7 + i * 2] << 8) | rxBuffer[8 + i * 2];
                    }

                    // 更新LED控制变量
                    if (register_address <= LED_MODE_REGISTER &&
                        (register_address + register_count) > LED_MODE_REGISTER) {
                        ledMode = (int8_t)holdingRegisters[LED_MODE_REGISTER];
                    }
                    if (register_address <= LED_VALUE_REGISTER &&
                        (register_address + register_count) > LED_VALUE_REGISTER) {
                        LED_value = (uint8_t)holdingRegisters[LED_VALUE_REGISTER];
                    }

                    // 发送确认回复
                    modbus_send_response(function_code, register_address, register_count, 0);
                }
            }
            break;

        case MODBUS_READ_HOLDING_REGISTERS:
            // 读保持寄存器：地址(1) + 功能码(1) + 起始地址(2) + 寄存器数量(2) + CRC(2)
            if (length == 8) {
                register_address = (rxBuffer[2] << 8) | rxBuffer[3];
                register_count = (rxBuffer[4] << 8) | rxBuffer[5];

                if (register_address < 10 && (register_address + register_count) <= 10) {
                    modbus_send_read_response(register_address, register_count);
                }
            }
            break;

        default:
            // 未知功能码，返回异常响应
            modbus_send_exception(function_code, 0x01);  // 非法功能码
            break;
    }
}

/**
 * @brief 发送Modbus响应
 * @param function_code 功能码
 * @param register_address 寄存器地址
 * @param register_value 寄存器值或数量
 * @param is_single 是否为单个寄存器写操作
 */
void modbus_send_response(uint8_t function_code, uint16_t register_address, uint16_t register_value, uint8_t is_single)
{
    uint16_t crc;
    uint8_t response_length = 0;

    modbusTxBuffer[0] = MODBUS_DEVICE_ADDRESS;
    modbusTxBuffer[1] = function_code;
    response_length = 2;

    switch (function_code) {
        case MODBUS_WRITE_SINGLE_REGISTER:
            modbusTxBuffer[2] = (register_address >> 8) & 0xFF;
            modbusTxBuffer[3] = register_address & 0xFF;
            modbusTxBuffer[4] = (register_value >> 8) & 0xFF;
            modbusTxBuffer[5] = register_value & 0xFF;
            response_length = 8;
            break;

        case MODBUS_WRITE_MULTIPLE_REGISTERS:
            modbusTxBuffer[2] = (register_address >> 8) & 0xFF;
            modbusTxBuffer[3] = register_address & 0xFF;
            modbusTxBuffer[4] = (register_value >> 8) & 0xFF;
            modbusTxBuffer[5] = register_value & 0xFF;
            response_length = 8;
            break;
    }

    // 计算并添加CRC
    crc = modbus_crc16(modbusTxBuffer, response_length);
    modbusTxBuffer[response_length] = crc & 0xFF;
    modbusTxBuffer[response_length + 1] = (crc >> 8) & 0xFF;

    // 通过串口发送响应
    HAL_UART_Transmit(&huart1, modbusTxBuffer, response_length + 2, 100);
}

/**
 * @brief 发送读寄存器响应
 * @param register_address 起始寄存器地址
 * @param register_count 寄存器数量
 */
void modbus_send_read_response(uint16_t register_address, uint16_t register_count)
{
    uint16_t crc;
    uint8_t byte_count = register_count * 2;

    modbusTxBuffer[0] = MODBUS_DEVICE_ADDRESS;
    modbusTxBuffer[1] = MODBUS_READ_HOLDING_REGISTERS;
    modbusTxBuffer[2] = byte_count;

    // 复制寄存器数据
    for (uint16_t i = 0; i < register_count; i++) {
        modbusTxBuffer[3 + i * 2] = (holdingRegisters[register_address + i] >> 8) & 0xFF;
        modbusTxBuffer[4 + i * 2] = holdingRegisters[register_address + i] & 0xFF;
    }

    // 计算并添加CRC
    crc = modbus_crc16(modbusTxBuffer, 3 + byte_count);
    modbusTxBuffer[3 + byte_count] = crc & 0xFF;
    modbusTxBuffer[4 + byte_count] = (crc >> 8) & 0xFF;

    // 通过串口发送响应
    HAL_UART_Transmit(&huart1, modbusTxBuffer, 5 + byte_count, 100);
}

/**
 * @brief 发送Modbus异常响应
 * @param function_code 功能码
 * @param exception_code 异常码
 */
void modbus_send_exception(uint8_t function_code, uint8_t exception_code)
{
    uint16_t crc;

    modbusTxBuffer[0] = MODBUS_DEVICE_ADDRESS;
    modbusTxBuffer[1] = function_code | 0x80;  // 异常响应功能码
    modbusTxBuffer[2] = exception_code;

    // 计算并添加CRC
    crc = modbus_crc16(modbusTxBuffer, 3);
    modbusTxBuffer[3] = crc & 0xFF;
    modbusTxBuffer[4] = (crc >> 8) & 0xFF;

    // 通过串口发送异常响应
    HAL_UART_Transmit(&huart1, modbusTxBuffer, 5, 100);
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
  	// 处理Modbus数据帧
  	modbus_process_frame(uart1RxBuf, uart1RxCounter);

  	__HAL_UART_ENABLE_IT(&huart1,UART_IT_RXNE);

  	uart1RxState=UART_RX_STATE_READY;
  	uart1RxCounter=0;
  	memset(uart1RxBuf,0,UART1_RCV_MAX);
  }

  // LED控制逻辑保持不变
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
