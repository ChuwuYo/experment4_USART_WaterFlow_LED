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
  * 线圈映射 (05/15功能码控制):
  * - 0x0001-0x0008: 单独LED控制 (0x0000=熄灭, 0xFF00=点亮 for FC05; 0=熄灭, 1=点亮 for FC15)
  *   * 0x0001: LED1控制 (GPIO0)
  *   * ...
  *   * 0x0008: LED8控制 (GPIO7)
  *
  * 支持的功能码:
  * - 03 (0x03): 读保持寄存器
  * - 05 (0x05): 写单个线圈
  * - 06 (0x06): 写单个寄存器
  * - 15 (0x0F): 写多个线圈
  *
  * 命令示例:
  *
  * 1. 启动右移模式 (FC06):
  *    命令: 01 06 00 00 00 01 48 0A
  *
  * 2. 启动左移模式 (FC06):
  *    命令: 01 06 00 00 00 02 08 0B
  *
  * 3. 停止模式 (FC06):
  *    命令: 01 06 00 00 00 00 C9 0A
  *
  * 4. 单独控制LED1点亮 (FC05):
  *    命令: 01 05 00 01 FF 00 9C 3A
  *
  * 5. 单独控制LED1熄灭 (FC05):
  *    命令: 01 05 00 01 00 00 CD CA
  *
  * 6. 同时控制8个LED (FC15, 亮-灭-亮-灭...):
  *    数据 0x55 (二进制 01010101) -> LED1,3,5,7亮, 其余灭
  *    命令: 01 0F 00 01 00 08 01 55 7B 5D
  *
  * 7. 同时点亮所有LED (FC15):
  *    数据 0xFF (二进制 11111111) -> 全部点亮
  *    命令: 01 0F 00 01 00 08 01 FF 7A 15
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
/* Modbus通信缓冲区 */
uint8_t modbusRxBuffer[MODBUS_BUFFER_SIZE];
uint8_t modbusTxBuffer[MODBUS_BUFFER_SIZE];

/* Modbus保持寄存器数组 */
uint16_t holdingRegisters[10] = {0};
uint16_t ledStatusRegisters[8] = {0};

/* LED控制变量 */
int8_t ledMode = 0;
uint8_t LED_value = 0x01;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
uint16_t modbus_crc16(uint8_t *data, uint16_t length);
void modbus_process_frame(uint8_t *rxBuffer, uint16_t length);
void modbus_send_write_response(uint8_t function_code, uint16_t address, uint16_t value);
void modbus_send_read_response(uint16_t register_address, uint16_t register_count);
void modbus_send_exception(uint8_t function_code, uint8_t exception_code);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// --- 全局GPIO查找表 ---
// 使用 static const 使其成为本文件私有的、只读的常量，提高效率和安全性
static const GPIO_TypeDef* LED_GPIO_PORTS[] = {
    LED0_GPIO_Port, LED1_GPIO_Port, LED2_GPIO_Port, LED3_GPIO_Port,
    LED4_GPIO_Port, LED5_GPIO_Port, LED6_GPIO_Port, LED7_GPIO_Port
};

static const uint16_t LED_GPIO_PINS[] = {
    LED0_Pin, LED1_Pin, LED2_Pin, LED3_Pin,
    LED4_Pin, LED5_Pin, LED6_Pin, LED7_Pin
};
// --- 查找表结束 ---

uint8_t cror(uint8_t numbers, uint8_t bits)
{
    return (numbers >> bits) | (numbers << (8 - bits));
}

uint8_t crol(uint8_t numbers, uint8_t bits)
{
    return (numbers << bits) | (numbers >> (8 - bits));
}

void LED_State_Set(uint8_t numbers)
{
    for (int i = 0; i < 8; i++) {
        GPIO_PinState state = ((numbers >> i) & 0x01) ? GPIO_PIN_RESET : GPIO_PIN_SET;
        HAL_GPIO_WritePin((GPIO_TypeDef*)LED_GPIO_PORTS[i], LED_GPIO_PINS[i], state);
    }
}

uint16_t modbus_crc16(uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < length; i++) {
        crc ^= (uint16_t)data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

void modbus_process_frame(uint8_t *rxBuffer, uint16_t length)
{
    if (length < 4) return;

    uint16_t crc_received = (rxBuffer[length - 1] << 8) | rxBuffer[length - 2];
    uint16_t crc_calculated = modbus_crc16(rxBuffer, length - 2);

    if (crc_received != crc_calculated) return;
    if (rxBuffer[0] != MODBUS_DEVICE_ADDRESS) return;

    uint8_t function_code = rxBuffer[1];
    uint16_t address = (rxBuffer[2] << 8) | rxBuffer[3];

    switch (function_code) {
        case MODBUS_WRITE_SINGLE_COIL: {
            if (length != 8) { modbus_send_exception(function_code, 0x03); return; }
            uint16_t value = (rxBuffer[4] << 8) | rxBuffer[5];
            if (address >= LED1_COIL_ADDRESS && address <= LED8_COIL_ADDRESS) {
                uint8_t led_index = address - LED1_COIL_ADDRESS;
                GPIO_PinState pin_state = (value == 0xFF00) ? GPIO_PIN_RESET : GPIO_PIN_SET;
                HAL_GPIO_WritePin((GPIO_TypeDef*)LED_GPIO_PORTS[led_index], LED_GPIO_PINS[led_index], pin_state);
                ledStatusRegisters[led_index] = (value == 0xFF00) ? 1 : 0;
                modbus_send_write_response(function_code, address, value);
            } else {
                modbus_send_exception(function_code, 0x02);
            }
            break;
        }

        case MODBUS_WRITE_MULTIPLE_COILS: {
            uint16_t coil_count = (rxBuffer[4] << 8) | rxBuffer[5];
            uint8_t byte_count = rxBuffer[6];
            
            if (length != (9 + byte_count) || byte_count != (coil_count + 7) / 8) {
                modbus_send_exception(function_code, 0x03); return;
            }
            if (address < LED1_COIL_ADDRESS || (address + coil_count) > (LED8_COIL_ADDRESS + 1)) {
                modbus_send_exception(function_code, 0x02); return;
            }
            
            uint8_t* data_bytes = &rxBuffer[7];
            for (uint16_t i = 0; i < coil_count; ++i) {
                uint16_t current_coil_addr = address + i;
                uint8_t led_index = current_coil_addr - LED1_COIL_ADDRESS;
                uint8_t coil_state_bit = (data_bytes[i / 8] >> (i % 8)) & 0x01;
                GPIO_PinState pin_state = (coil_state_bit == 1) ? GPIO_PIN_RESET : GPIO_PIN_SET;
                
                HAL_GPIO_WritePin((GPIO_TypeDef*)LED_GPIO_PORTS[led_index], LED_GPIO_PINS[led_index], pin_state);
                ledStatusRegisters[led_index] = coil_state_bit;
            }

            modbus_send_write_response(function_code, address, coil_count);
            break;
        }

        case MODBUS_WRITE_SINGLE_REGISTER: {
            if (length != 8) { modbus_send_exception(function_code, 0x03); return; }
            uint16_t value = (rxBuffer[4] << 8) | rxBuffer[5];
            if (address == LED_MODE_REGISTER) {
                ledMode = (int8_t)value;
                holdingRegisters[LED_MODE_REGISTER] = value;
                modbus_send_write_response(function_code, address, value);
            } else if (address == LED_VALUE_REGISTER) {
                LED_value = (uint8_t)value;
                holdingRegisters[LED_VALUE_REGISTER] = value;
                modbus_send_write_response(function_code, address, value);
            } else {
                modbus_send_exception(function_code, 0x02);
            }
            break;
        }

        case MODBUS_READ_HOLDING_REGISTERS: {
            if (length != 8) { modbus_send_exception(function_code, 0x03); return; }
            uint16_t count = (rxBuffer[4] << 8) | rxBuffer[5];
            if (address < 10 && (address + count) <= 10) {
                 modbus_send_read_response(address, count);
            } else {
                 modbus_send_exception(function_code, 0x02);
            }
            break;
        }

        default:
            modbus_send_exception(function_code, 0x01);
            break;
    }
}

void modbus_send_write_response(uint8_t function_code, uint16_t address, uint16_t value)
{
    modbusTxBuffer[0] = MODBUS_DEVICE_ADDRESS;
    modbusTxBuffer[1] = function_code;
    modbusTxBuffer[2] = (address >> 8) & 0xFF;
    modbusTxBuffer[3] = address & 0xFF;
    modbusTxBuffer[4] = (value >> 8) & 0xFF;
    modbusTxBuffer[5] = value & 0xFF;
    uint16_t crc = modbus_crc16(modbusTxBuffer, 6);
    modbusTxBuffer[6] = crc & 0xFF;
    modbusTxBuffer[7] = (crc >> 8) & 0xFF;
    HAL_UART_Transmit(&huart1, modbusTxBuffer, 8, 100);
}

void modbus_send_read_response(uint16_t register_address, uint16_t register_count)
{
    uint8_t byte_count = register_count * 2;
    modbusTxBuffer[0] = MODBUS_DEVICE_ADDRESS;
    modbusTxBuffer[1] = MODBUS_READ_HOLDING_REGISTERS;
    modbusTxBuffer[2] = byte_count;
    for (uint16_t i = 0; i < register_count; i++) {
        uint16_t reg_val = holdingRegisters[register_address + i];
        modbusTxBuffer[3 + i * 2] = (reg_val >> 8) & 0xFF;
        modbusTxBuffer[4 + i * 2] = reg_val & 0xFF;
    }
    uint16_t crc = modbus_crc16(modbusTxBuffer, 3 + byte_count);
    modbusTxBuffer[3 + byte_count] = crc & 0xFF;
    modbusTxBuffer[4 + byte_count] = (crc >> 8) & 0xFF;
    HAL_UART_Transmit(&huart1, modbusTxBuffer, 5 + byte_count, 100);
}

void modbus_send_exception(uint8_t function_code, uint8_t exception_code)
{
    modbusTxBuffer[0] = MODBUS_DEVICE_ADDRESS;
    modbusTxBuffer[1] = function_code | 0x80;
    modbusTxBuffer[2] = exception_code;
    uint16_t crc = modbus_crc16(modbusTxBuffer, 3);
    modbusTxBuffer[3] = crc & 0xFF;
    modbusTxBuffer[4] = (crc >> 8) & 0xFF;
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
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if (uart1RxState == UART_RX_STATE_DEAL)
    {
      modbus_process_frame(uart1RxBuf, uart1RxCounter);
      memset(uart1RxBuf, 0, UART1_RCV_MAX);
      uart1RxCounter = 0;
      uart1RxState = UART_RX_STATE_READY;
      __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
    }

    switch (ledMode)
    {
      case 1:
        LED_State_Set(LED_value);
        LED_value = cror(LED_value, 1);
        HAL_Delay(500);
        break;
      case 2:
        LED_State_Set(LED_value);
        LED_value = crol(LED_value, 1);
        HAL_Delay(500);
        break;
      case 0:
        break;
      default:
        break;
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
void USER_UART_IRQHandler(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_RXNE) != RESET)
    {
        if (uart1RxState == UART_RX_STATE_READY) {
            uart1RxState = UART_RX_STATE_START;
            __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
        }
        if (uart1RxCounter < UART1_RCV_MAX) {
            uart1RxBuf[uart1RxCounter++] = (uint8_t)(huart->Instance->DR & 0xFF);
        }
    }
    else if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) != RESET)
    {
        __HAL_UART_CLEAR_IDLEFLAG(huart);
        __HAL_UART_DISABLE_IT(huart, UART_IT_IDLE);
        __HAL_UART_DISABLE_IT(huart, UART_IT_RXNE);
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