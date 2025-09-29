/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body with Modbus RTU protocol implementation
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
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

// Modbus功能码定义
#define MODBUS_FC_READ_COILS           0x01
#define MODBUS_FC_WRITE_SINGLE_COIL    0x05
#define MODBUS_FC_WRITE_MULTIPLE_COILS 0x0F

// Modbus异常码定义
#define MODBUS_EXCEPTION_ILLEGAL_FUNCTION    0x01
#define MODBUS_EXCEPTION_ILLEGAL_DATA_ADDR   0x02
#define MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE  0x03

// Modbus帧结构
typedef struct {
    uint8_t address;        // 设备地址
    uint8_t function;       // 功能码
    uint8_t data[252];      // 数据域（最大252字节）
    uint16_t crc;           // CRC校验
    uint8_t length;         // 数据长度
} ModbusFrame_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define MODBUS_DEVICE_ADDRESS  0x01    // 设备地址
#define MAX_COILS             8        // 最大线圈数量（对应8个LED）
#define UART_BUFFER_SIZE      256      // UART缓冲区大小
#define MODBUS_TIMEOUT        100      // Modbus超时时间(ms)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

// UART接收缓冲区
uint8_t uart_rx_buffer[UART_BUFFER_SIZE];
uint8_t uart_tx_buffer[UART_BUFFER_SIZE];
volatile uint8_t uart_rx_index = 0;
volatile uint8_t modbus_frame_ready = 0;
volatile uint32_t last_rx_time = 0;

// LED状态存储（线圈状态）
uint8_t coil_status = 0x00;  // 8位对应8个LED状态

// 水流传感器相关
volatile uint32_t pulse_count = 0;
volatile uint32_t last_pulse_time = 0;
float flow_rate = 0.0f;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);

/* USER CODE BEGIN PFP */

// Modbus协议相关函数
uint16_t modbus_crc16(uint8_t *data, uint16_t length);
void modbus_process_frame(uint8_t *frame, uint8_t length);
void modbus_send_response(uint8_t *data, uint8_t length);
void modbus_send_exception(uint8_t function_code, uint8_t exception_code);

// LED控制函数
void update_led_status(void);
void set_single_coil(uint16_t address, uint8_t value);
void set_multiple_coils(uint16_t start_address, uint16_t quantity, uint8_t *values);

// 水流传感器函数
void water_flow_pulse_handler(void);
void calculate_flow_rate(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * @brief  计算Modbus CRC16校验
 * @param  data: 数据指针
 * @param  length: 数据长度
 * @retval CRC16值
 */
uint16_t modbus_crc16(uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFF;
    uint16_t i, j;
    
    for (i = 0; i < length; i++) {
        crc ^= data[i];
        for (j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc = crc >> 1;
            }
        }
    }
    return crc;
}

/**
 * @brief  发送Modbus响应
 * @param  data: 响应数据
 * @param  length: 数据长度
 */
void modbus_send_response(uint8_t *data, uint8_t length)
{
    uint16_t crc = modbus_crc16(data, length);
    
    // 添加CRC校验
    data[length] = crc & 0xFF;
    data[length + 1] = (crc >> 8) & 0xFF;
    
    // 发送响应
    HAL_UART_Transmit(&huart1, data, length + 2, 100);
}

/**
 * @brief  发送Modbus异常响应
 * @param  function_code: 功能码
 * @param  exception_code: 异常码
 */
void modbus_send_exception(uint8_t function_code, uint8_t exception_code)
{
    uint8_t response[3];
    
    response[0] = MODBUS_DEVICE_ADDRESS;
    response[1] = function_code | 0x80;  // 异常响应标志
    response[2] = exception_code;
    
    modbus_send_response(response, 3);
}

/**
 * @brief  设置单个线圈状态
 * @param  address: 线圈地址
 * @param  value: 线圈值（0或1）
 */
void set_single_coil(uint16_t address, uint8_t value)
{
    if (address < MAX_COILS) {
        if (value) {
            coil_status |= (1 << address);
        } else {
            coil_status &= ~(1 << address);
        }
        update_led_status();
    }
}

/**
 * @brief  设置多个线圈状态
 * @param  start_address: 起始地址
 * @param  quantity: 线圈数量
 * @param  values: 线圈值数组
 */
void set_multiple_coils(uint16_t start_address, uint16_t quantity, uint8_t *values)
{
    uint16_t i;
    uint8_t byte_index, bit_index;
    
    for (i = 0; i < quantity; i++) {
        if ((start_address + i) < MAX_COILS) {
            byte_index = i / 8;
            bit_index = i % 8;
            
            if (values[byte_index] & (1 << bit_index)) {
                coil_status |= (1 << (start_address + i));
            } else {
                coil_status &= ~(1 << (start_address + i));
            }
        }
    }
    update_led_status();
}

/**
 * @brief  更新LED状态
 */
void update_led_status(void)
{
    // 根据coil_status更新LED状态
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, (coil_status & 0x01) ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, (coil_status & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, (coil_status & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, (coil_status & 0x08) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, (coil_status & 0x10) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, (coil_status & 0x20) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, (coil_status & 0x40) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, (coil_status & 0x80) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/**
 * @brief  处理Modbus帧
 * @param  frame: 接收到的帧数据
 * @param  length: 帧长度
 */
void modbus_process_frame(uint8_t *frame, uint8_t length)
{
    uint16_t received_crc, calculated_crc;
    uint8_t response[UART_BUFFER_SIZE];
    uint8_t response_length = 0;
    
    // 检查最小帧长度
    if (length < 4) return;
    
    // 检查设备地址
    if (frame[0] != MODBUS_DEVICE_ADDRESS) return;
    
    // 验证CRC
    received_crc = (frame[length-1] << 8) | frame[length-2];
    calculated_crc = modbus_crc16(frame, length - 2);
    
    if (received_crc != calculated_crc) return;
    
    // 构建响应帧头
    response[0] = MODBUS_DEVICE_ADDRESS;
    response[1] = frame[1];  // 功能码
    
    switch (frame[1]) {
        case MODBUS_FC_READ_COILS: {
            uint16_t start_address = (frame[2] << 8) | frame[3];
            uint16_t quantity = (frame[4] << 8) | frame[5];
            
            // 检查地址和数量有效性
            if (start_address >= MAX_COILS || (start_address + quantity) > MAX_COILS || quantity == 0) {
                modbus_send_exception(frame[1], MODBUS_EXCEPTION_ILLEGAL_DATA_ADDR);
                return;
            }
            
            response[2] = 1;  // 字节数
            response[3] = (coil_status >> start_address) & ((1 << quantity) - 1);
            response_length = 4;
            break;
        }
        
        case MODBUS_FC_WRITE_SINGLE_COIL: {
            uint16_t address = (frame[2] << 8) | frame[3];
            uint16_t value = (frame[4] << 8) | frame[5];
            
            // 检查地址有效性
            if (address >= MAX_COILS) {
                modbus_send_exception(frame[1], MODBUS_EXCEPTION_ILLEGAL_DATA_ADDR);
                return;
            }
            
            // 检查值有效性（0x0000或0xFF00）
            if (value != 0x0000 && value != 0xFF00) {
                modbus_send_exception(frame[1], MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE);
                return;
            }
            
            // 设置线圈状态
            set_single_coil(address, (value == 0xFF00) ? 1 : 0);
            
            // 回显请求
            memcpy(&response[2], &frame[2], 4);
            response_length = 6;
            break;
        }
        
        case MODBUS_FC_WRITE_MULTIPLE_COILS: {
            uint16_t start_address = (frame[2] << 8) | frame[3];
            uint16_t quantity = (frame[4] << 8) | frame[5];
            uint8_t byte_count = frame[6];
            
            // 检查地址和数量有效性
            if (start_address >= MAX_COILS || (start_address + quantity) > MAX_COILS || 
                quantity == 0 || byte_count != ((quantity + 7) / 8)) {
                modbus_send_exception(frame[1], MODBUS_EXCEPTION_ILLEGAL_DATA_ADDR);
                return;
            }
            
            // 设置多个线圈
            set_multiple_coils(start_address, quantity, &frame[7]);
            
            // 构建响应
            memcpy(&response[2], &frame[2], 4);
            response_length = 6;
            break;
        }
        
        default:
            modbus_send_exception(frame[1], MODBUS_EXCEPTION_ILLEGAL_FUNCTION);
            return;
    }
    
    // 发送正常响应
    modbus_send_response(response, response_length);
}

/**
 * @brief  水流传感器脉冲中断处理
 */
void water_flow_pulse_handler(void)
{
    uint32_t current_time = HAL_GetTick();
    
    // 防抖处理
    if (current_time - last_pulse_time > 10) {
        pulse_count++;
        last_pulse_time = current_time;
    }
}

/**
 * @brief  计算流量
 */
void calculate_flow_rate(void)
{
    static uint32_t last_calc_time = 0;
    static uint32_t last_pulse_count = 0;
    
    uint32_t current_time = HAL_GetTick();
    
    if (current_time - last_calc_time >= 1000) {  // 每秒计算一次
        uint32_t pulse_diff = pulse_count - last_pulse_count;
        
        // 假设传感器系数为7.5脉冲/升
        flow_rate = (float)pulse_diff / 7.5f * 60.0f;  // L/min
        
        last_pulse_count = pulse_count;
        last_calc_time = current_time;
    }
}

/**
 * @brief  UART接收完成回调
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        last_rx_time = HAL_GetTick();
        uart_rx_index++;
        
        // 继续接收下一个字节
        if (uart_rx_index < UART_BUFFER_SIZE) {
            HAL_UART_Receive_IT(&huart1, &uart_rx_buffer[uart_rx_index], 1);
        }
    }
}

/**
 * @brief  GPIO外部中断回调
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_7) {  // 水流传感器引脚
        water_flow_pulse_handler();
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
  
  // 初始化LED状态
  update_led_status();
  
  // 启动UART接收中断
  HAL_UART_Receive_IT(&huart1, &uart_rx_buffer[0], 1);
  
  printf("Modbus RTU Device Ready - Address: 0x%02X\r\n", MODBUS_DEVICE_ADDRESS);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    
    // 检查Modbus帧接收超时
    if (uart_rx_index > 0 && (HAL_GetTick() - last_rx_time) > 10) {
        // 处理接收到的Modbus帧
        modbus_process_frame(uart_rx_buffer, uart_rx_index);
        
        // 重置接收缓冲区
        uart_rx_index = 0;
        memset(uart_rx_buffer, 0, UART_BUFFER_SIZE);
        
        // 重新启动接收
        HAL_UART_Receive_IT(&huart1, &uart_rx_buffer[0], 1);
    }
    
    // 计算水流量
    calculate_flow_rate();
    
    // 简单的心跳指示
    static uint32_t heartbeat_time = 0;
    if (HAL_GetTick() - heartbeat_time > 1000) {
        heartbeat_time = HAL_GetTick();
        // 可以在这里添加状态指示
    }
    
    HAL_Delay(1);
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

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3 
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 PA2 PA3 
                           PA4 PA5 PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3 
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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