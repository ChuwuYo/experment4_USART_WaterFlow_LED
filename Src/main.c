/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body with Modbus RTU protocol
  *
  * Modbus RTU命令说明（注释由AI生成）:
  * ===================
  *
  * 设备地址: 0x01
  *
  * 保持寄存器映射:
  * - 0x0000: LED模式控制 (0=停止, 1=右移, 2=左移)
  * - 0x0001: LED值控制
  *
  * 线圈映射 (05/15功能码控制):
  * - 0x0001-0x0008: 单独LED控制 (0x0000=熄灭, 0xFF00=点亮 for FC05; 0=熄灭, 1=点亮 for FC15)
  *   * 0x0001: LED1控制 (GPIO0)
  *   * ...
  *   * 0x0008: LED8控制 (GPIO7)
  *
  * 支持的功能码:
  * - 01 (0x01): 读线圈状态
  * - 03 (0x03): 读保持寄存器
  * - 05 (0x05): 写单个线圈
  * - 06 (0x06): 写单个寄存器
  * - 15 (0x0F): 写多个线圈
  *
  * 命令示例:
  *
  * 1. 启动右移模式 (FC06 - 写单个寄存器):
  *    命令格式: 设备地址(01) + 功能码(06) + 寄存器地址高字节(00) + 寄存器地址低字节(00) + 数据高字节(00) + 数据低字节(01) + CRC低字节(48) + CRC高字节(0A)
  *    命令: 01 06 00 00 00 01 48 0A
  *
  * 2. 启动左移模式 (FC06):
  *    命令: 01 06 00 00 00 02 08 0B
  *
  * 3. 停止模式 (FC06):
  *    命令: 01 06 00 00 00 00 C9 0A
  *
  * 4. 单独控制LED1点亮 (FC05 - 写单个线圈):
  *    命令格式: 设备地址(01) + 功能码(05) + 线圈地址高字节(00) + 线圈地址低字节(01) + 状态高字节(FF) + 状态低字节(00) + CRC低字节(9C) + CRC高字节(3A)
  *    命令: 01 05 00 01 FF 00 9C 3A
  *    说明: FF00表示点亮，0000表示熄灭
  *
  * 5. 单独控制LED1熄灭 (FC05):
  *    命令: 01 05 00 01 00 00 CD CA
  *
  * 6. 同时控制8个LED (FC15 - 写多个线圈, 亮-灭-亮-灭...):
  *    命令格式: 设备地址(01) + 功能码(0F) + 起始地址高字节(00) + 起始地址低字节(01) + 线圈数量高字节(00) + 线圈数量低字节(08) + 字节数(01) + 数据(55) + CRC低字节(7B) + CRC高字节(5D)
  *    命令: 01 0F 00 01 00 08 01 55 7B 5D
  *    说明: 数据0x55(二进制01010101) -> LED1,3,5,7亮, 其余灭
  *
  * 7. 同时点亮所有LED (FC15):
  *    命令: 01 0F 00 01 00 08 01 FF 7A 15
  *    说明: 数据0xFF(二进制11111111) -> 全部点亮
  *
  * 8. 查询单个LED状态 (FC03 - 读保持寄存器, 以LED1为例):
  *    命令格式: 设备地址(01) + 功能码(03) + 起始地址高字节(00) + 起始地址低字节(01) + 寄存器数量高字节(00) + 寄存器数量低字节(01) + CRC低字节(90) + CRC高字节(0A)
  *    命令: 01 03 00 01 00 01 90 0A
  *    响应格式: 设备地址(01) + 功能码(03) + 字节数(02) + 寄存器值高字节(00) + 寄存器值低字节(00/01) + CRC低字节 + CRC高字节
  *
  * 9. 查询所有8个LED的状态 (FC03):
  *    命令: 01 03 00 01 00 08 45 CE
  *
  * 10. 读取单个线圈状态 (FC01 - 读线圈状态, 以LED1为例):
  *     命令格式: 设备地址(01) + 功能码(01) + 起始地址高字节(00) + 起始地址低字节(01) + 线圈数量高字节(00) + 线圈数量低字节(01) + CRC低字节(90) + CRC高字节(0C)
  *     命令: 01 01 00 01 00 01 90 0C
  *     响应格式: 设备地址(01) + 功能码(01) + 字节数(01) + 状态值(00/01) + CRC低字节 + CRC高字节
  *     读取单个线圈状态 (FC01, 以LED8为例):
  *     命令: 01 01 00 08 00 01 3D CF
  * 
  * 
  * 11. 读取多个线圈状态 (FC01, 读取LED1-LED8):
  *     命令: 01 01 00 01 00 08 BD CC
  *     响应格式: 设备地址(01) + 功能码(01) + 字节数(01) + 状态值(8位打包) + CRC低字节 + CRC高字节
  *     状态值示例: 0x55(二进制01010101) 表示LED1,3,5,7亮，LED2,4,6,8灭
  *
  * 12. 读取部分线圈状态 (FC01, 读取LED1-LED4):
  *     命令: 01 01 00 01 00 04 BD CD
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
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* Modbus RTU协议配置 */
#define MODBUS_DEVICE_ADDRESS    0x01    // 从设备地址
#define MODBUS_BUFFER_SIZE       256     // 接收/发送缓冲区大小

/* Modbus功能码定义 */
#define MODBUS_READ_COILS                0x01  // 读线圈状态
#define MODBUS_READ_HOLDING_REGISTERS    0x03  // 读保持寄存器
#define MODBUS_WRITE_SINGLE_COIL         0x05  // 写单个线圈
#define MODBUS_WRITE_SINGLE_REGISTER     0x06  // 写单个寄存器
#define MODBUS_WRITE_MULTIPLE_COILS      0x0F  // 写多个线圈
#define MODBUS_WRITE_MULTIPLE_REGISTERS  0x10  // 写多个寄存器

/* Modbus寄存器地址映射 */
#define LED_MODE_REGISTER   0x0000  // LED模式控制寄存器
#define LED_VALUE_REGISTER  0x0001  // LED值控制寄存器

/* 单独LED控制线圈地址映射 (LED1-LED8) */
#define LED1_COIL_ADDRESS       0x0001
#define LED8_COIL_ADDRESS       0x0008
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Modbus通信缓冲区 - 用于接收和发送Modbus RTU数据帧 */
uint8_t modbusRxBuffer[MODBUS_BUFFER_SIZE];  // 接收缓冲区，存储从串口接收的Modbus数据
uint8_t modbusTxBuffer[MODBUS_BUFFER_SIZE];  // 发送缓冲区，存储准备发送的Modbus响应数据

/* Modbus保持寄存器数组 - 存储设备状态和配置信息 */
uint16_t holdingRegisters[10] = {0};         // 通用保持寄存器数组，支持10个寄存器地址(0x0000-0x0009)
uint16_t ledStatusRegisters[8] = {0};        // LED状态寄存器数组，记录8个LED的当前状态(0=灭,1=亮)

/* LED控制变量 - 控制LED灯的显示模式和移位效果 */
int8_t ledMode = 0;                         // LED显示模式: 0=停止,1=右移,2=左移
uint8_t LED_value = 0x01;                   // LED移位显示的初始值，8位表示8个LED的亮灭状态
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/**
 * @brief 计算Modbus RTU数据帧的CRC16校验码
 * @param data 指向数据缓冲区的指针
 * @param length 数据长度（不包括CRC校验码本身）
 * @return 计算得到的16位CRC校验码
 * @note 采用Modbus RTU标准CRC16多项式: 0xA001
 */
uint16_t modbus_crc16(uint8_t *data, uint16_t length);

/**
 * @brief 处理接收到的Modbus RTU数据帧
 * @param rxBuffer 接收数据缓冲区指针
 * @param length 接收数据长度
 * @note 支持的功能码：读保持寄存器(0x03)、写单个线圈(0x05)、写单个寄存器(0x06)、写多个线圈(0x0F)
 * @note 自动验证CRC校验、设备地址和功能码，处理完成后发送相应响应
 */
void modbus_process_frame(uint8_t *rxBuffer, uint16_t length);

/**
 * @brief 发送Modbus写操作响应（写单个线圈、写单个寄存器、写多个线圈）
 * @param function_code 功能码
 * @param address 寄存器/线圈起始地址
 * @param value 写入的值或线圈数量
 * @note 响应格式：设备地址 + 功能码 + 地址高字节 + 地址低字节 + 值高字节 + 值低字节 + CRC低字节 + CRC高字节
 */
void modbus_send_write_response(uint8_t function_code, uint16_t address, uint16_t value);

/**
 * @brief 发送Modbus读保持寄存器响应
 * @param register_address 寄存器起始地址
 * @param register_count 读取寄存器数量
 * @note 响应格式：设备地址 + 功能码(0x03) + 字节数 + 寄存器数据 + CRC低字节 + CRC高字节
 */
void modbus_send_read_response(uint16_t register_address, uint16_t register_count);

/**
 * @brief 发送Modbus读线圈状态响应
 * @param coil_address 线圈起始地址
 * @param coil_count 读取线圈数量
 * @note 响应格式：设备地址 + 功能码(0x01) + 字节数 + 线圈状态数据（按位打包） + CRC低字节 + CRC高字节
 */
void modbus_send_read_coils_response(uint16_t coil_address, uint16_t coil_count);

/**
 * @brief 发送Modbus异常响应
 * @param function_code 异常的功能码（最高位设置为1）
 * @param exception_code 异常代码（0x01=不支持的功能码，0x02=非法地址，0x03=非法数据值）
 * @note 异常响应格式：设备地址 + 异常功能码 + 异常代码 + CRC低字节 + CRC高字节
 */
void modbus_send_exception(uint8_t function_code, uint8_t exception_code);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// --- 全局GPIO查找表 ---
// 使用 static const 使其成为本文件私有的、只读的常量，提高效率和安全性
// LED GPIO端口查找表 - 按照LED编号顺序排列对应的GPIO端口
// 索引0-7对应LED0-LED7，方便通过索引快速访问对应的GPIO端口
static const GPIO_TypeDef* LED_GPIO_PORTS[] = {
    LED0_GPIO_Port,  // LED0对应的GPIO端口
    LED1_GPIO_Port,  // LED1对应的GPIO端口
    LED2_GPIO_Port,  // LED2对应的GPIO端口
    LED3_GPIO_Port,  // LED3对应的GPIO端口
    LED4_GPIO_Port,  // LED4对应的GPIO端口
    LED5_GPIO_Port,  // LED5对应的GPIO端口
    LED6_GPIO_Port,  // LED6对应的GPIO端口
    LED7_GPIO_Port   // LED7对应的GPIO端口
};

// LED GPIO引脚查找表 - 按照LED编号顺序排列对应的GPIO引脚
// 索引0-7对应LED0-LED7，配合端口查找表使用，实现LED的快速控制
static const uint16_t LED_GPIO_PINS[] = {
    LED0_Pin,  // LED0对应的GPIO引脚
    LED1_Pin,  // LED1对应的GPIO引脚
    LED2_Pin,  // LED2对应的GPIO引脚
    LED3_Pin,  // LED3对应的GPIO引脚
    LED4_Pin,  // LED4对应的GPIO引脚
    LED5_Pin,  // LED5对应的GPIO引脚
    LED6_Pin,  // LED6对应的GPIO引脚
    LED7_Pin   // LED7对应的GPIO引脚
};
// --- 查找表结束 ---

/**
 * @brief 8位无符号整数循环右移操作
 * @param numbers 要进行移位操作的8位无符号整数
 * @param bits 右移位数（0-7位）
 * @return 循环右移后的结果
 * @note 用于LED右移显示效果，例如：0b00000001 -> 0b10000000
 * @example cror(0b00000001, 1) 返回 0b10000000
 */
uint8_t cror(uint8_t numbers, uint8_t bits)
{
    return (numbers >> bits) | (numbers << (8 - bits));
}

/**
 * @brief 8位无符号整数循环左移操作
 * @param numbers 要进行移位操作的8位无符号整数
 * @param bits 左移位数（0-7位）
 * @return 循环左移后的结果
 * @note 用于LED左移显示效果，例如：0b10000000 -> 0b00000001
 * @example crol(0b10000000, 1) 返回 0b00000001
 */
uint8_t crol(uint8_t numbers, uint8_t bits)
{
    return (numbers << bits) | (numbers >> (8 - bits));
}

/**
 * @brief 根据8位数值设置8个LED的亮灭状态
 * @param numbers 8位数值，每位代表一个LED的状态（0=灭，1=亮）
 * @note 位0（LSB）对应LED0，位7（MSB）对应LED7
 * @note GPIO_PIN_RESET=LED点亮，GPIO_PIN_SET=LED熄灭（根据硬件连接决定）
 * @example LED_State_Set(0b10101010) 将点亮LED0,2,4,6，熄灭LED1,3,5,7
 */
void LED_State_Set(uint8_t numbers)
{
    for (int i = 0; i < 8; i++) {
        // 检查第i位是否为1，决定LED的亮灭状态
        // GPIO_PIN_RESET=点亮LED，GPIO_PIN_SET=熄灭LED
        GPIO_PinState state = ((numbers >> i) & 0x01) ? GPIO_PIN_RESET : GPIO_PIN_SET;
        HAL_GPIO_WritePin((GPIO_TypeDef*)LED_GPIO_PORTS[i], LED_GPIO_PINS[i], state);
    }
}

/**
 * @brief 计算Modbus RTU数据帧的CRC16校验码
 * @param data 指向数据缓冲区的指针，包含要计算CRC的数据
 * @param length 数据长度（字节数），CRC校验码本身不包含在计算范围内
 * @return 计算得到的16位CRC校验码，低字节在前高字节在后
 * @note 实现Modbus RTU标准CRC16算法：
 *       - 初始CRC值为0xFFFF
 *       - 多项式为0xA001（CRC16-IBM/Modbus标准）
 *       - 按位计算，每字节处理8位
 *       - 最终返回时高低字节已按Modbus RTU格式排列
 * @warning 输入数据长度应至少为1字节，长度为0将返回初始值0xFFFF
 */
uint16_t modbus_crc16(uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFF;  // CRC初始值，Modbus RTU标准

    // 遍历数据缓冲区的每个字节
    for (uint16_t i = 0; i < length; i++) {
        crc ^= (uint16_t)data[i];  // 将当前字节与CRC值进行XOR运算

        // 对当前字节的每一位进行处理
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                // 如果最低位为1，则右移一位并与多项式0xA001进行XOR
                crc = (crc >> 1) ^ 0xA001;
            } else {
                // 如果最低位为0，则只右移一位
                crc >>= 1;
            }
        }
    }
    return crc;  // 返回计算得到的CRC16校验码
}

/**
 * @brief 处理接收到的Modbus RTU数据帧
 * @param rxBuffer 接收数据缓冲区指针，包含完整的Modbus RTU数据帧
 * @param length 接收数据长度（字节数）
 * @note 支持的功能码：读线圈状态(0x01)、读保持寄存器(0x03)、写单个线圈(0x05)、写单个寄存器(0x06)、写多个线圈(0x0F)
 * @note 自动验证CRC校验、设备地址和功能码，处理完成后发送相应响应
 * @warning 输入数据长度至少应为4字节（设备地址+功能码+数据+CRC），否则直接返回
 */
void modbus_process_frame(uint8_t *rxBuffer, uint16_t length)
{
    if (length < 4) return;  // 数据长度不足，忽略该帧

    // 提取接收到的CRC校验码（低字节在前，高字节在后）
    uint16_t crc_received = (rxBuffer[length - 1] << 8) | rxBuffer[length - 2];
    // 计算CRC校验码（不包括CRC字段本身）
    uint16_t crc_calculated = modbus_crc16(rxBuffer, length - 2);

    if (crc_received != crc_calculated) return;  // CRC校验失败，忽略该帧
    if (rxBuffer[0] != MODBUS_DEVICE_ADDRESS) return;  // 设备地址不匹配，忽略该帧

    // 解析功能码和起始地址
    uint8_t function_code = rxBuffer[1];
    uint16_t address = (rxBuffer[2] << 8) | rxBuffer[3];

    switch (function_code) {
        case MODBUS_READ_COILS: {  // 读线圈状态 (0x01)
            if (length != 8) { modbus_send_exception(function_code, 0x03); return; }  // 数据长度错误
            uint16_t count = (rxBuffer[4] << 8) | rxBuffer[5];  // 读取线圈数量
            // 验证线圈地址范围（LED1-LED8: 0x0001-0x0008）
            if (address >= LED1_COIL_ADDRESS && address <= LED8_COIL_ADDRESS &&
                (address + count) <= (LED8_COIL_ADDRESS + 1) && count > 0) {
                modbus_send_read_coils_response(address, count);  // 发送读线圈响应
            } else {
                modbus_send_exception(function_code, 0x02);  // 非法地址异常
            }
            break;
        }

        case MODBUS_WRITE_SINGLE_COIL: {  // 写单个线圈 (0x05)
            if (length != 8) { modbus_send_exception(function_code, 0x03); return; }  // 数据长度错误
            uint16_t value = (rxBuffer[4] << 8) | rxBuffer[5];  // 线圈状态值
            if (address >= LED1_COIL_ADDRESS && address <= LED8_COIL_ADDRESS) {  // 地址范围检查
                uint8_t led_index = address - LED1_COIL_ADDRESS;  // 计算LED索引
                uint16_t led_state = (value == 0xFF00) ? 1 : 0;  // 转换线圈状态（FF00=亮，0000=灭）

                // 1. 控制物理LED硬件
                GPIO_PinState pin_state = (led_state == 1) ? GPIO_PIN_RESET : GPIO_PIN_SET;
                HAL_GPIO_WritePin((GPIO_TypeDef*)LED_GPIO_PORTS[led_index], LED_GPIO_PINS[led_index], pin_state);

                // 2. 更新用于快速读取的状态数组
                ledStatusRegisters[led_index] = led_state;

                // --- FIX: 新增的关键代码 ---
                // 3. 同步更新 holdingRegisters 数组，以便FC03可以查询
                holdingRegisters[address] = led_state;
                // --- 修正结束 ---

                modbus_send_write_response(function_code, address, value);  // 发送写响应
            } else {
                modbus_send_exception(function_code, 0x02);  // 非法地址异常
            }
            break;
        }

        case MODBUS_WRITE_MULTIPLE_COILS: {  // 写多个线圈 (0x0F)
            uint16_t coil_count = (rxBuffer[4] << 8) | rxBuffer[5];  // 线圈数量
            uint8_t byte_count = rxBuffer[6];  // 数据字节数

            // 验证帧长度和字节数正确性
            if (length != (9 + byte_count) || byte_count != (coil_count + 7) / 8) {
                modbus_send_exception(function_code, 0x03); return;  // 非法数据值异常
            }
            // 验证地址范围
            if (address < LED1_COIL_ADDRESS || (address + coil_count) > (LED8_COIL_ADDRESS + 1)) {
                modbus_send_exception(function_code, 0x02); return;  // 非法地址异常
            }

            uint8_t* data_bytes = &rxBuffer[7];  // 数据字节指针
            for (uint16_t i = 0; i < coil_count; ++i) {  // 遍历每个线圈
                uint16_t current_coil_addr = address + i;  // 当前线圈地址
                uint8_t led_index = current_coil_addr - LED1_COIL_ADDRESS;  // LED索引
                // 从数据字节中提取对应位的状态
                uint8_t coil_state_bit = (data_bytes[i / 8] >> (i % 8)) & 0x01;

                // 控制物理LED硬件
                GPIO_PinState pin_state = (coil_state_bit == 1) ? GPIO_PIN_RESET : GPIO_PIN_SET;
                HAL_GPIO_WritePin((GPIO_TypeDef*)LED_GPIO_PORTS[led_index], LED_GPIO_PINS[led_index], pin_state);
                // 更新状态数组
                ledStatusRegisters[led_index] = coil_state_bit;

                // --- FIX: 新增的关键代码 ---
                // 同步更新 holdingRegisters 数组
                holdingRegisters[current_coil_addr] = coil_state_bit;
                // --- 修正结束 ---
            }

            modbus_send_write_response(function_code, address, coil_count);  // 发送写响应
            break;
        }

        case MODBUS_WRITE_SINGLE_REGISTER: {  // 写单个寄存器 (0x06)
            if (length != 8) { modbus_send_exception(function_code, 0x03); return; }  // 数据长度错误
            uint16_t value = (rxBuffer[4] << 8) | rxBuffer[5];  // 寄存器值
            if (address == LED_MODE_REGISTER) {  // LED模式寄存器
                ledMode = (int8_t)value;  // 更新LED模式
                holdingRegisters[LED_MODE_REGISTER] = value;  // 同步到保持寄存器
                modbus_send_write_response(function_code, address, value);  // 发送写响应
            } else if (address == LED_VALUE_REGISTER) {  // LED值寄存器
                LED_value = (uint8_t)value;  // 更新LED值
                holdingRegisters[LED_VALUE_REGISTER] = value;  // 同步到保持寄存器
                modbus_send_write_response(function_code, address, value);  // 发送写响应
            } else {
                modbus_send_exception(function_code, 0x02);  // 非法地址异常
            }
            break;
        }

        case MODBUS_READ_HOLDING_REGISTERS: {  // 读保持寄存器 (0x03)
            if (length != 8) { modbus_send_exception(function_code, 0x03); return; }  // 数据长度错误
            uint16_t count = (rxBuffer[4] << 8) | rxBuffer[5];  // 读取寄存器数量
            // 注意：现在我们直接从 holdingRegisters 读取，地址范围可以更广
            if (address < 10 && (address + count) <= 10) {  // 地址范围检查
                 modbus_send_read_response(address, count);  // 发送读寄存器响应
            } else {
                 modbus_send_exception(function_code, 0x02);  // 非法地址异常
            }
            break;
        }

        default:  // 不支持的功能码
            modbus_send_exception(function_code, 0x01);  // 不支持的功能码异常
            break;
    }
}

/**
 * @brief 发送Modbus写操作响应（写单个线圈、写单个寄存器、写多个线圈）
 * @param function_code 功能码（原功能码，用于响应）
 * @param address 寄存器/线圈起始地址
 * @param value 写入的值或线圈数量
 * @note 响应格式：设备地址(1) + 功能码(1) + 地址高字节(1) + 地址低字节(1) + 值高字节(1) + 值低字节(1) + CRC低字节(1) + CRC高字节(1)
 * @note 总长度：8字节，超时时间：100ms
 */
void modbus_send_write_response(uint8_t function_code, uint16_t address, uint16_t value)
{
    // 构建Modbus RTU响应帧
    modbusTxBuffer[0] = MODBUS_DEVICE_ADDRESS;                    // 设备地址
    modbusTxBuffer[1] = function_code;                           // 功能码（回显）
    modbusTxBuffer[2] = (address >> 8) & 0xFF;                  // 地址高字节
    modbusTxBuffer[3] = address & 0xFF;                         // 地址低字节
    modbusTxBuffer[4] = (value >> 8) & 0xFF;                    // 值高字节
    modbusTxBuffer[5] = value & 0xFF;                           // 值低字节

    // 计算CRC校验码（对前6字节计算）
    uint16_t crc = modbus_crc16(modbusTxBuffer, 6);
    modbusTxBuffer[6] = crc & 0xFF;                             // CRC低字节
    modbusTxBuffer[7] = (crc >> 8) & 0xFF;                     // CRC高字节

    // 发送响应帧（8字节，100ms超时）
    HAL_UART_Transmit(&huart1, modbusTxBuffer, 8, 100);
}

/**
 * @brief 发送Modbus读保持寄存器响应
 * @param register_address 寄存器起始地址
 * @param register_count 读取寄存器数量
 * @note 响应格式：设备地址(1) + 功能码(1) + 字节数(1) + 寄存器数据(2×数量) + CRC低字节(1) + CRC高字节(1)
 * @note 总长度：5 + 2×寄存器数量 字节，超时时间：100ms
 */
void modbus_send_read_response(uint16_t register_address, uint16_t register_count)
{
    // 计算响应数据长度（每个寄存器2字节）
    uint8_t byte_count = register_count * 2;

    // 构建响应帧头部
    modbusTxBuffer[0] = MODBUS_DEVICE_ADDRESS;                    // 设备地址
    modbusTxBuffer[1] = MODBUS_READ_HOLDING_REGISTERS;           // 功能码(0x03)
    modbusTxBuffer[2] = byte_count;                              // 数据字节数

    // 填充寄存器数据（高字节在前，低字节在后）
    for (uint16_t i = 0; i < register_count; i++) {
        uint16_t reg_val = holdingRegisters[register_address + i];  // 获取寄存器值
        modbusTxBuffer[3 + i * 2] = (reg_val >> 8) & 0xFF;        // 寄存器值高字节
        modbusTxBuffer[4 + i * 2] = reg_val & 0xFF;               // 寄存器值低字节
    }

    // 计算CRC校验码（对前3字节 + 数据字节计算）
    uint16_t crc = modbus_crc16(modbusTxBuffer, 3 + byte_count);
    modbusTxBuffer[3 + byte_count] = crc & 0xFF;                // CRC低字节
    modbusTxBuffer[4 + byte_count] = (crc >> 8) & 0xFF;        // CRC高字节

    // 发送响应帧（5 + 数据字节数，100ms超时）
    HAL_UART_Transmit(&huart1, modbusTxBuffer, 5 + byte_count, 100);
}

/**
 * @brief 发送Modbus读线圈状态响应
 * @param coil_address 线圈起始地址
 * @param coil_count 读取线圈数量
 * @note 响应格式：设备地址(1) + 功能码(1) + 字节数(1) + 线圈状态数据(按位打包) + CRC低字节(1) + CRC高字节(1)
 * @note 线圈状态按位打包：每8个线圈占用1字节，bit0对应第一个线圈
 * @note 总长度：5 + 字节数 字节，超时时间：100ms
 */
void modbus_send_read_coils_response(uint16_t coil_address, uint16_t coil_count)
{
    // 计算需要的字节数（每8个线圈占1字节，向上取整）
    uint8_t byte_count = (coil_count + 7) / 8;

    // 构建响应帧头部
    modbusTxBuffer[0] = MODBUS_DEVICE_ADDRESS;                    // 设备地址
    modbusTxBuffer[1] = MODBUS_READ_COILS;                       // 功能码(0x01)
    modbusTxBuffer[2] = byte_count;                              // 数据字节数

    // 清空数据区域
    memset(&modbusTxBuffer[3], 0, byte_count);

    // 填充线圈状态数据（按位打包）
    for (uint16_t i = 0; i < coil_count; i++) {
        uint16_t current_coil = coil_address + i;
        // 从ledStatusRegisters数组读取线圈状态
        if (current_coil >= LED1_COIL_ADDRESS && current_coil <= LED8_COIL_ADDRESS) {
            uint8_t led_index = current_coil - LED1_COIL_ADDRESS;
            uint8_t coil_state = ledStatusRegisters[led_index];
            
            // 将线圈状态按位打包到字节中
            if (coil_state) {
                modbusTxBuffer[3 + (i / 8)] |= (1 << (i % 8));
            }
        }
    }

    // 计算CRC校验码（对前3字节 + 数据字节计算）
    uint16_t crc = modbus_crc16(modbusTxBuffer, 3 + byte_count);
    modbusTxBuffer[3 + byte_count] = crc & 0xFF;                // CRC低字节
    modbusTxBuffer[4 + byte_count] = (crc >> 8) & 0xFF;        // CRC高字节

    // 发送响应帧（5 + 数据字节数，100ms超时）
    HAL_UART_Transmit(&huart1, modbusTxBuffer, 5 + byte_count, 100);
}

/**
 * @brief 发送Modbus异常响应
 * @param function_code 异常的功能码（最高位设置为1）
 * @param exception_code 异常代码（0x01=不支持的功能码，0x02=非法地址，0x03=非法数据值）
 * @note 异常响应格式：设备地址(1) + 异常功能码(1) + 异常代码(1) + CRC低字节(1) + CRC高字节(1)
 * @note 总长度：5字节，超时时间：100ms
 */
void modbus_send_exception(uint8_t function_code, uint8_t exception_code)
{
    // 构建异常响应帧
    modbusTxBuffer[0] = MODBUS_DEVICE_ADDRESS;                    // 设备地址
    modbusTxBuffer[1] = function_code | 0x80;                    // 异常功能码（最高位为1）
    modbusTxBuffer[2] = exception_code;                          // 异常代码

    // 计算CRC校验码（对前3字节计算）
    uint16_t crc = modbus_crc16(modbusTxBuffer, 3);
    modbusTxBuffer[3] = crc & 0xFF;                             // CRC低字节
    modbusTxBuffer[4] = (crc >> 8) & 0xFF;                     // CRC高字节

    // 发送异常响应帧（5字节，100ms超时）
    HAL_UART_Transmit(&huart1, modbusTxBuffer, 5, 100);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
/**
 * @brief 应用程序主函数 - Modbus RTU从机程序入口
 * @retval int 程序返回值（实际不会返回）
 * @note 实现基于STM32的Modbus RTU从机功能，支持LED控制和状态查询
 * @note 支持的功能码：0x01(读线圈状态)、0x03(读保持寄存器)、0x05(写单个线圈)、0x06(写单个寄存器)、0x0F(写多个线圈)
 */
int main(void)
{
  /* USER CODE BEGIN 1 */
  // 用户自定义初始化代码段（预留）
  /* USER CODE END 1 */

  /* MCU配置初始化 ---------------------------------------------------------*/

  /* 复位所有外设，初始化Flash接口和SysTick */
  HAL_Init();

  /* USER CODE BEGIN Init */
  // 用户自定义初始化代码段（预留）
  /* USER CODE END Init */

  /* 配置系统时钟 */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  // 用户自定义系统初始化代码段（预留）
  /* USER CODE END SysInit */

  /* 初始化所有配置的外设 */
  MX_GPIO_Init();      // 初始化GPIO外设
  MX_USART1_UART_Init();  // 初始化USART1串口

  /* USER CODE BEGIN 2 */
  // 启用串口接收中断，准备接收Modbus RTU数据
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
  /* USER CODE END 2 */

  /* 主循环 */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // 检查是否收到完整的Modbus数据帧
    if (uart1RxState == UART_RX_STATE_DEAL)
    {
      // 处理接收到的Modbus RTU数据帧
      modbus_process_frame(uart1RxBuf, uart1RxCounter);

      // 清理接收缓冲区，准备下次接收
      memset(uart1RxBuf, 0, UART1_RCV_MAX);
      uart1RxCounter = 0;
      uart1RxState = UART_RX_STATE_READY;

      // 重新启用串口接收中断
      __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
    }

    // 根据LED模式执行不同的显示效果
    switch (ledMode)
    {
      case 1:  // 右移模式
        LED_State_Set(LED_value);        // 设置LED显示状态
        LED_value = cror(LED_value, 1);  // 循环右移一位
        HAL_Delay(500);                  // 延时500ms
        break;
      case 2:  // 左移模式
        LED_State_Set(LED_value);        // 设置LED显示状态
        LED_value = crol(LED_value, 1);  // 循环左移一位
        HAL_Delay(500);                  // 延时500ms
        break;
      case 0:  // 停止模式
        break;   // 什么也不做，保持当前状态
      default: // 其他模式
        break;   // 无效模式，保持停止状态
    }
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
 * @brief UART串口中断处理函数 - 处理Modbus RTU数据接收
 * @param huart 指向UART外设句柄的指针
 * @note 实现串口空闲中断+接收中断的Modbus RTU帧接收逻辑
 * @note 接收状态机：READY -> START -> 接收数据 -> IDLE触发 -> DEAL
 * @warning 必须在main函数中启用串口接收中断：__HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE)
 */
void USER_UART_IRQHandler(UART_HandleTypeDef *huart)
{
  // 仅处理USART1的中断
  if (huart->Instance == USART1)
  {
    // 处理接收数据寄存器非空中断（有新数据到达）
    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_RXNE) != RESET)
    {
        // 首次接收到数据时，切换到开始状态并启用空闲中断
        if (uart1RxState == UART_RX_STATE_READY) {
            uart1RxState = UART_RX_STATE_START;
            __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
        }

        // 检查缓冲区是否未满，将接收到的数据存入缓冲区
        if (uart1RxCounter < UART1_RCV_MAX) {
            // 读取串口数据寄存器（自动清除RXNE标志）
            uart1RxBuf[uart1RxCounter++] = (uint8_t)(huart->Instance->DR & 0xFF);
        }
        // 注意：如果缓冲区满了，数据将被丢弃（实际应用中应考虑缓冲区溢出处理）
    }
    // 处理串口空闲中断（数据帧接收完成）
    else if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) != RESET)
    {
        // 清除串口空闲标志
        __HAL_UART_CLEAR_IDLEFLAG(huart);

        // 禁用空闲中断和接收中断，防止在处理数据时被中断
        __HAL_UART_DISABLE_IT(huart, UART_IT_IDLE);
        __HAL_UART_DISABLE_IT(huart, UART_IT_RXNE);

        // 切换到数据处理状态，主循环会检测到此状态并处理Modbus帧
        uart1RxState = UART_RX_STATE_DEAL;
    }
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
  while(1)
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */