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
  *
  * 保持寄存器映射:
  * - 0x0000: LED模式控制 (0=停止, 1=右移, 2=左移)
  * - 0x0001: LED值控制
  *
  * 线圈映射 (05功能码控制):
  * - 0x0010-0x0017: 单独LED控制 (0x0000=熄灭, 0xFF00=点亮)
  *   * 0x0010: LED0控制
  *   * 0x0011: LED1控制
  *   * 0x0012: LED2控制
  *   * 0x0013: LED3控制
  *   * 0x0014: LED4控制
  *   * 0x0015: LED5控制
  *   * 0x0016: LED6控制
  *   * 0x0017: LED7控制
  *
  * 命令示例:
  *
  * 1. 启动右移模式:
  *    命令: 01 06 00 00 00 01 48 0A
  *
  * 2. 启动左移模式:
  *    命令: 01 06 00 00 00 02 08 0B
  *
  * 3. 停止模式:
  *    命令: 01 06 00 00 00 00 C9 0A
  *
  * 4. 单独控制LED0点亮 (05功能码):
  *    命令: 01 05 00 10 FF 00 8C 3A
  *
  * 5. 单独控制LED0熄灭 (05功能码):
  *    命令: 01 05 00 10 00 00 CD CA
  *
  * 6. 单独控制LED7点亮 (05功能码):
  *    命令: 01 05 00 17 FF 00 5C 3A
  *
  * 7. 读取LED0状态:
  *    命令: 01 03 00 10 00 01 84 0A
  *
  * 8. 读取所有LED状态:
  *    命令: 01 03 00 10 00 08 C5 CE
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
uint8_t dataBuf[128] = {0};

/* Modbus RTU协议配置 */
#define MODBUS_DEVICE_ADDRESS    0x01    // 从设备地址
#define MODBUS_BUFFER_SIZE       256     // 接收/发送缓冲区大小

/* Modbus功能码定义 */
#define MODBUS_READ_COILS                0x01  // 读线圈状态
#define MODBUS_READ_HOLDING_REGISTERS    0x03  // 读保持寄存器
#define MODBUS_WRITE_SINGLE_COIL         0x05  // 写单个线圈
#define MODBUS_WRITE_SINGLE_REGISTER     0x06  // 写单个寄存器
#define MODBUS_WRITE_MULTIPLE_REGISTERS   0x10  // 写多个寄存器

/* Modbus寄存器地址映射 */
#define LED_MODE_REGISTER   0x0000  // LED模式控制寄存器
#define LED_VALUE_REGISTER  0x0001  // LED值控制寄存器

/* 单独LED控制寄存器映射 */
#define LED0_REGISTER       0x0010  // LED0控制寄存器 (0=熄灭, 1=点亮)
#define LED1_REGISTER       0x0011  // LED1控制寄存器 (0=熄灭, 1=点亮)
#define LED2_REGISTER       0x0012  // LED2控制寄存器 (0=熄灭, 1=点亮)
#define LED3_REGISTER       0x0013  // LED3控制寄存器 (0=熄灭, 1=点亮)
#define LED4_REGISTER       0x0014  // LED4控制寄存器 (0=熄灭, 1=点亮)
#define LED5_REGISTER       0x0015  // LED5控制寄存器 (0=熄灭, 1=点亮)
#define LED6_REGISTER       0x0016  // LED6控制寄存器 (0=熄灭, 1=点亮)
#define LED7_REGISTER       0x0017  // LED7控制寄存器 (0=熄灭, 1=点亮)

/* Modbus通信缓冲区 */
uint8_t modbusRxBuffer[MODBUS_BUFFER_SIZE];  // 接收缓冲区
uint8_t modbusTxBuffer[MODBUS_BUFFER_SIZE];  // 发送缓冲区
uint16_t modbusRxCount = 0;                  // 接收数据计数
uint8_t modbusRxComplete = 0;                // 接收完成标志

/* Modbus保持寄存器数组 */
uint16_t holdingRegisters[10] = {0};  // 保持10个寄存器供Modbus访问

/* LED状态寄存器数组（用于读取操作） */
uint16_t ledStatusRegisters[8] = {0};  // LED0-LED7状态寄存器

/* LED控制变量 */
int8_t ledMode = -1;      // LED模式：-1=未初始化，0=停止，1=右移，2=左移
uint8_t LED_value = 0xFD; // LED当前显示值

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* Modbus协议处理函数声明 */
uint16_t modbus_crc16(uint8_t *data, uint16_t length);
void modbus_process_frame(uint8_t *rxBuffer, uint16_t length);
void modbus_send_response(uint8_t function_code, uint16_t register_address, uint16_t register_value, uint8_t is_single);
void modbus_send_coil_response(uint8_t function_code, uint16_t coil_address, uint16_t coil_value);
void modbus_send_read_response(uint16_t register_address, uint16_t register_count);
void modbus_send_led_read_response(uint16_t register_address, uint16_t start_index, uint16_t register_count);
void modbus_send_exception(uint8_t function_code, uint8_t exception_code);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * @brief 循环右移函数
 * @param numbers 8位数据
 * @param bits 右移位数
 * @return 右移后的结果
 */
uint8_t cror(uint8_t numbers, uint8_t bits)
{
	uint8_t right = numbers >> bits;
	uint8_t left = numbers << (8 - bits);
	uint8_t temp = left | right;
	return temp;
}

/**
 * @brief 循环左移函数
 * @param numbers 8位数据
 * @param bits 左移位数
 * @return 左移后的结果
 */
uint8_t crol(uint8_t numbers, uint8_t bits)
{
	uint8_t right = numbers << bits;
	uint8_t left = numbers >> (8 - bits);
	uint8_t temp = left | right;
	return temp;
}

/**
 * @brief 设置8个LED灯的状态
 * @param numbers 8位数据，每位控制一个LED
 * @note LED0~LED7分别对应bit0~bit7，1=点亮，0=熄灭
 */
void LED_State_Set(uint8_t numbers)
{
	HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, ((numbers & 0x01) >> 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, ((numbers & 0x02) >> 1) ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, ((numbers & 0x04) >> 2) ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, ((numbers & 0x08) >> 3) ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, ((numbers & 0x10) >> 4) ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, ((numbers & 0x20) >> 5) ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED6_GPIO_Port, LED6_Pin, ((numbers & 0x40) >> 6) ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED7_GPIO_Port, LED7_Pin, ((numbers & 0x80) >> 7) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/**
 * @brief 计算Modbus CRC16校验码
 * @param data 数据缓冲区指针
 * @param length 数据长度（字节数）
 * @return CRC16校验码（低字节在前，高字节在后）
 * @note 采用Modbus标准CRC16多项式：0xA001
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
                crc ^= 0xA001;  /* Modbus标准CRC16多项式 */
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
 * @note 解析Modbus RTU数据帧，支持写寄存器和读寄存器操作
 */
void modbus_process_frame(uint8_t *rxBuffer, uint16_t length)
{
    uint16_t crc_received, crc_calculated;
    uint8_t device_address, function_code;
    uint16_t register_address, register_value, register_count;

    /* 验证数据长度（最小帧长度：地址+功能码+CRC = 4字节） */
    if (length < 4) {
        return;
    }

    /* 验证CRC校验 */
    crc_received = (rxBuffer[length-1] << 8) | rxBuffer[length-2];
    crc_calculated = modbus_crc16(rxBuffer, length - 2);
    if (crc_received != crc_calculated) {
        return;  /* CRC校验失败 */
    }

    /* 解析Modbus帧 */
    device_address = rxBuffer[0];
    function_code = rxBuffer[1];

    /* 检查设备地址 */
    if (device_address != MODBUS_DEVICE_ADDRESS) {
        return;  /* 不是本设备 */
    }

    switch (function_code) {
        case MODBUS_WRITE_SINGLE_REGISTER:
            /* 写单个寄存器：地址(2) + 功能码(1) + 寄存器地址(2) + 寄存器值(2) + CRC(2) */
            if (length == 8) {
                register_address = (rxBuffer[2] << 8) | rxBuffer[3];
                register_value = (rxBuffer[4] << 8) | rxBuffer[5];

                if (register_address < 10) {  /* 原有的保持寄存器 */
                    holdingRegisters[register_address] = register_value;

                    /* 更新LED控制变量 */
                    if (register_address == LED_MODE_REGISTER) {
                        ledMode = (int8_t)register_value;
                    } else if (register_address == LED_VALUE_REGISTER) {
                        LED_value = (uint8_t)register_value;
                    }

                    /* 发送确认回复 */
                    modbus_send_response(function_code, register_address, register_value, 1);
                }
                else if (register_address >= LED0_REGISTER && register_address <= LED7_REGISTER) {
                    /* 单独控制LED灯 */
                    uint8_t led_index = register_address - LED0_REGISTER;
                    GPIO_PinState pin_state = (register_value == 1) ? GPIO_PIN_SET : GPIO_PIN_RESET;

                    switch (led_index) {
                        case 0:
                            HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, pin_state);
                            ledStatusRegisters[0] = register_value;
                            break;
                        case 1:
                            HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, pin_state);
                            ledStatusRegisters[1] = register_value;
                            break;
                        case 2:
                            HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, pin_state);
                            ledStatusRegisters[2] = register_value;
                            break;
                        case 3:
                            HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, pin_state);
                            ledStatusRegisters[3] = register_value;
                            break;
                        case 4:
                            HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, pin_state);
                            ledStatusRegisters[4] = register_value;
                            break;
                        case 5:
                            HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, pin_state);
                            ledStatusRegisters[5] = register_value;
                            break;
                        case 6:
                            HAL_GPIO_WritePin(LED6_GPIO_Port, LED6_Pin, pin_state);
                            ledStatusRegisters[6] = register_value;
                            break;
                        case 7:
                            HAL_GPIO_WritePin(LED7_GPIO_Port, LED7_Pin, pin_state);
                            ledStatusRegisters[7] = register_value;
                            break;
                    }

                    /* 发送确认回复 */
                    modbus_send_response(function_code, register_address, register_value, 1);
                }
            }
            break;

        case MODBUS_WRITE_MULTIPLE_REGISTERS:
            /* 写多个寄存器：地址(1) + 功能码(1) + 起始地址(2) + 寄存器数量(2) + 字节数(1) + 数据(n) + CRC(2) */
            if (length >= 9) {
                register_address = (rxBuffer[2] << 8) | rxBuffer[3];
                register_count = (rxBuffer[4] << 8) | rxBuffer[5];
                uint8_t byte_count = rxBuffer[6];

                if (length == (9 + byte_count) && (byte_count == register_count * 2)) {
                    /* 写入寄存器值 */
                    for (uint16_t i = 0; i < register_count && (register_address + i) < 10; i++) {
                        holdingRegisters[register_address + i] =
                            (rxBuffer[7 + i * 2] << 8) | rxBuffer[8 + i * 2];
                    }

                    /* 更新LED控制变量 */
                    if (register_address <= LED_MODE_REGISTER &&
                        (register_address + register_count) > LED_MODE_REGISTER) {
                        ledMode = (int8_t)holdingRegisters[LED_MODE_REGISTER];
                    }
                    if (register_address <= LED_VALUE_REGISTER &&
                        (register_address + register_count) > LED_VALUE_REGISTER) {
                        LED_value = (uint8_t)holdingRegisters[LED_VALUE_REGISTER];
                    }

                    /* 发送确认回复 */
                    modbus_send_response(function_code, register_address, register_count, 0);
                }
            }
            break;

        case MODBUS_READ_HOLDING_REGISTERS:
            /* 读保持寄存器：地址(1) + 功能码(1) + 起始地址(2) + 寄存器数量(2) + CRC(2) */
            if (length == 8) {
                register_address = (rxBuffer[2] << 8) | rxBuffer[3];
                register_count = (rxBuffer[4] << 8) | rxBuffer[5];

                if (register_address < 10 && (register_address + register_count) <= 10) {
                    /* 读取原有的保持寄存器 */
                    modbus_send_read_response(register_address, register_count);
                }
                else if (register_address >= LED0_REGISTER && register_address <= LED7_REGISTER) {
                    /* 读取LED状态寄存器 */
                    uint16_t start_index = register_address - LED0_REGISTER;
                    uint16_t actual_count = (start_index + register_count > 8) ? (8 - start_index) : register_count;
                    modbus_send_led_read_response(register_address, start_index, actual_count);
                }
            }
            break;

        default:
            /* 未知功能码，返回异常响应 */
            modbus_send_exception(function_code, 0x01);  /* 非法功能码 */
            break;
    }
}

/**
 * @brief 发送Modbus写操作响应
 * @param function_code 功能码
 * @param register_address 寄存器地址
 * @param register_value 寄存器值或数量
 * @param is_single 是否为单个寄存器写操作
 * @note 响应格式与请求格式相同，用于确认写操作成功
 */
void modbus_send_response(uint8_t function_code, uint16_t register_address, uint16_t register_value, uint8_t is_single)
{
    uint16_t crc;
    uint8_t response_length = 0;

    /* 构建响应帧头 */
    modbusTxBuffer[0] = MODBUS_DEVICE_ADDRESS;
    modbusTxBuffer[1] = function_code;
    response_length = 2;

    switch (function_code) {
        case MODBUS_WRITE_SINGLE_COIL:
            /* 写单个线圈：地址(1) + 功能码(1) + 线圈地址(2) + 线圈值(2) + CRC(2) */
            if (length == 8) {
                register_address = (rxBuffer[2] << 8) | rxBuffer[3];
                register_value = (rxBuffer[4] << 8) | rxBuffer[5];

                if (register_address >= LED0_REGISTER && register_address <= LED7_REGISTER) {
                    /* 单独控制LED灯 */
                    uint8_t led_index = register_address - LED0_REGISTER;
                    GPIO_PinState pin_state = (register_value == 0xFF00) ? GPIO_PIN_SET : GPIO_PIN_RESET;

                    switch (led_index) {
                        case 0:
                            HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, pin_state);
                            ledStatusRegisters[0] = (register_value == 0xFF00) ? 1 : 0;
                            break;
                        case 1:
                            HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, pin_state);
                            ledStatusRegisters[1] = (register_value == 0xFF00) ? 1 : 0;
                            break;
                        case 2:
                            HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, pin_state);
                            ledStatusRegisters[2] = (register_value == 0xFF00) ? 1 : 0;
                            break;
                        case 3:
                            HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, pin_state);
                            ledStatusRegisters[3] = (register_value == 0xFF00) ? 1 : 0;
                            break;
                        case 4:
                            HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, pin_state);
                            ledStatusRegisters[4] = (register_value == 0xFF00) ? 1 : 0;
                            break;
                        case 5:
                            HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, pin_state);
                            ledStatusRegisters[5] = (register_value == 0xFF00) ? 1 : 0;
                            break;
                        case 6:
                            HAL_GPIO_WritePin(LED6_GPIO_Port, LED6_Pin, pin_state);
                            ledStatusRegisters[6] = (register_value == 0xFF00) ? 1 : 0;
                            break;
                        case 7:
                            HAL_GPIO_WritePin(LED7_GPIO_Port, LED7_Pin, pin_state);
                            ledStatusRegisters[7] = (register_value == 0xFF00) ? 1 : 0;
                            break;
                    }

                    /* 发送确认回复 */
                    modbus_send_coil_response(function_code, register_address, register_value);
                }
            }
            break;

        case MODBUS_WRITE_SINGLE_REGISTER:
            /* 单个寄存器写响应：回显地址和值 */
            modbusTxBuffer[2] = (register_address >> 8) & 0xFF;
            modbusTxBuffer[3] = register_address & 0xFF;
            modbusTxBuffer[4] = (register_value >> 8) & 0xFF;
            modbusTxBuffer[5] = register_value & 0xFF;
            response_length = 8;
            break;

        case MODBUS_WRITE_MULTIPLE_REGISTERS:
            /* 多个寄存器写响应：返回地址和数量 */
            modbusTxBuffer[2] = (register_address >> 8) & 0xFF;
            modbusTxBuffer[3] = register_address & 0xFF;
            modbusTxBuffer[4] = (register_value >> 8) & 0xFF;
            modbusTxBuffer[5] = register_value & 0xFF;
            response_length = 8;
            break;
    }

    /* 计算并添加CRC */
    crc = modbus_crc16(modbusTxBuffer, response_length);
    modbusTxBuffer[response_length] = crc & 0xFF;
    modbusTxBuffer[response_length + 1] = (crc >> 8) & 0xFF;

    /* 通过串口发送响应 */
    HAL_UART_Transmit(&huart1, modbusTxBuffer, response_length + 2, 100);
}

/**
 * @brief 发送读寄存器响应
 * @param register_address 起始寄存器地址
 * @param register_count 寄存器数量
 * @note 响应格式：地址(1) + 功能码(1) + 字节数(1) + 数据(n) + CRC(2)
 */
void modbus_send_read_response(uint16_t register_address, uint16_t register_count)
{
    uint16_t crc;
    uint8_t byte_count = register_count * 2;

    /* 构建响应帧头 */
    modbusTxBuffer[0] = MODBUS_DEVICE_ADDRESS;
    modbusTxBuffer[1] = MODBUS_READ_HOLDING_REGISTERS;
    modbusTxBuffer[2] = byte_count;

    /* 复制寄存器数据（大端序） */
    for (uint16_t i = 0; i < register_count; i++) {
        modbusTxBuffer[3 + i * 2] = (holdingRegisters[register_address + i] >> 8) & 0xFF;
        modbusTxBuffer[4 + i * 2] = holdingRegisters[register_address + i] & 0xFF;
    }

    /* 计算并添加CRC */
    crc = modbus_crc16(modbusTxBuffer, 3 + byte_count);
    modbusTxBuffer[3 + byte_count] = crc & 0xFF;
    modbusTxBuffer[4 + byte_count] = (crc >> 8) & 0xFF;

    /* 通过串口发送响应 */
    HAL_UART_Transmit(&huart1, modbusTxBuffer, 5 + byte_count, 100);
}

/**
  * @brief 发送LED状态寄存器读取响应
  * @param register_address Modbus寄存器地址
  * @param start_index LED状态数组起始索引
  * @param register_count 寄存器数量
  * @note 响应格式：地址(1) + 功能码(1) + 字节数(1) + 数据(n) + CRC(2)
  */
void modbus_send_led_read_response(uint16_t register_address, uint16_t start_index, uint16_t register_count)
{
    uint16_t crc;
    uint8_t byte_count = register_count * 2;

    /* 构建响应帧头 */
    modbusTxBuffer[0] = MODBUS_DEVICE_ADDRESS;
    modbusTxBuffer[1] = MODBUS_READ_HOLDING_REGISTERS;
    modbusTxBuffer[2] = byte_count;

    /* 复制LED状态数据（大端序） */
    for (uint16_t i = 0; i < register_count; i++) {
        modbusTxBuffer[3 + i * 2] = (ledStatusRegisters[start_index + i] >> 8) & 0xFF;
        modbusTxBuffer[4 + i * 2] = ledStatusRegisters[start_index + i] & 0xFF;
    }

    /* 计算并添加CRC */
    crc = modbus_crc16(modbusTxBuffer, 3 + byte_count);
    modbusTxBuffer[3 + byte_count] = crc & 0xFF;
    modbusTxBuffer[4 + byte_count] = (crc >> 8) & 0xFF;

    /* 通过串口发送响应 */
    HAL_UART_Transmit(&huart1, modbusTxBuffer, 5 + byte_count, 100);
}

/**
  * @brief 发送Modbus写线圈响应
  * @param function_code 功能码
  * @param coil_address 线圈地址
  * @param coil_value 线圈值
  * @note 响应格式与请求格式相同，用于确认写操作成功
  */
void modbus_send_coil_response(uint8_t function_code, uint16_t coil_address, uint16_t coil_value)
{
    uint16_t crc;

    /* 构建响应帧头 */
    modbusTxBuffer[0] = MODBUS_DEVICE_ADDRESS;
    modbusTxBuffer[1] = function_code;
    modbusTxBuffer[2] = (coil_address >> 8) & 0xFF;
    modbusTxBuffer[3] = coil_address & 0xFF;
    modbusTxBuffer[4] = (coil_value >> 8) & 0xFF;
    modbusTxBuffer[5] = coil_value & 0xFF;

    /* 计算并添加CRC */
    crc = modbus_crc16(modbusTxBuffer, 6);
    modbusTxBuffer[6] = crc & 0xFF;
    modbusTxBuffer[7] = (crc >> 8) & 0xFF;

    /* 通过串口发送响应 */
    HAL_UART_Transmit(&huart1, modbusTxBuffer, 8, 100);
}

/**
  * @brief 发送Modbus异常响应
  * @param function_code 功能码
  * @param exception_code 异常码
  * @note 异常响应格式：地址(1) + (功能码|0x80)(1) + 异常码(1) + CRC(2)
  */
void modbus_send_exception(uint8_t function_code, uint8_t exception_code)
{
    uint16_t crc;

    /* 构建异常响应帧 */
    modbusTxBuffer[0] = MODBUS_DEVICE_ADDRESS;
    modbusTxBuffer[1] = function_code | 0x80;  /* 异常响应功能码 */
    modbusTxBuffer[2] = exception_code;

    /* 计算并添加CRC */
    crc = modbus_crc16(modbusTxBuffer, 3);
    modbusTxBuffer[3] = crc & 0xFF;
    modbusTxBuffer[4] = (crc >> 8) & 0xFF;

    /* 通过串口发送异常响应 */
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
    /* USER CODE END WHILE */

    /* 处理接收到的Modbus数据帧 */
    if (UART_RX_STATE_DEAL == uart1RxState)
    {
      modbus_process_frame(uart1RxBuf, uart1RxCounter);

      __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);

      uart1RxState = UART_RX_STATE_READY;
      uart1RxCounter = 0;
      memset(uart1RxBuf, 0, UART1_RCV_MAX);
    }

    /* LED控制逻辑 */
    switch (ledMode)
    {
      case 1:  /* 右移模式 */
        LED_State_Set(LED_value);
        LED_value = cror(LED_value, 1);
        HAL_Delay(1000);
        break;

      case 2:  /* 左移模式 */
        LED_State_Set(LED_value);
        LED_value = crol(LED_value, 1);
        HAL_Delay(1000);
        break;

      case 0:  /* 停止模式 */
        LED_value = 0xFF;
        LED_State_Set(LED_value);
        LED_value = 0xFD;
        break;

      default:  /* 未初始化或无效模式 */
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

/**
 * @brief UART接收中断处理函数
 * @param huart UART句柄
 * @note 实现Modbus RTU数据帧的接收，使用空闲中断+RX中断组合
 */
void USER_UART_IRQHandler(UART_HandleTypeDef *huart)
{
  /* 处理数据接收中断 */
  if ((__HAL_UART_GET_FLAG(huart, UART_FLAG_RXNE) != RESET))
  {
    if (UART_RX_STATE_READY == uart1RxState)
    {
      /* 首次接收，切换到接收状态，启用空闲中断 */
      uart1RxState = UART_RX_STATE_START;
      __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
      uart1RxCounter = 0;
      uart1RxBuf[uart1RxCounter] = (uint8_t)((*huart).Instance->DR & (uint8_t)0x00FF);
      uart1RxCounter++;
    }
    else if (UART_RX_STATE_START == uart1RxState)
    {
      /* 连续接收数据 */
      uart1RxBuf[uart1RxCounter] = (uint8_t)((*huart).Instance->DR & (uint8_t)0x00FF);
      uart1RxCounter++;
    }
    __HAL_UART_CLEAR_FLAG(huart, UART_FLAG_RXNE);
  }

  /* 处理空闲中断，标志数据帧接收完成 */
  if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) != RESET)
  {
    __HAL_UART_DISABLE_IT(huart, UART_IT_IDLE);
    __HAL_UART_DISABLE_IT(huart, UART_IT_RXNE);
    uart1RxState = UART_RX_STATE_DEAL;
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
