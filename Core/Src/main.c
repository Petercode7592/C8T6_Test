/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include <stdio.h>
#include "./../../YYFDrive/Mdrv.h"
#include "string.h"
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
float V[4] = {0.0f, 0.0f, 0.0f, 0.0f};
int32_t MAX_SPEED;

uint8_t RxBuff[1]; //进入中断接收数据的数组
uint8_t DataBuff[500]; //保存接收到的数据的数组
int RxLine = 0; //接收到的数据长度

float mall[4];
int mtep[4];
float mspd[4];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


// /** @brief  MCU 发送单个字节
//   * @param mode 1代表 to PC 2代表 to 驱动板
//   * @param pData 数据指针
//   * @retval HAL_StatusTypeDef 返回状态
//   */
// HAL_StatusTypeDef UART_TransmitOnebit(const uint8_t mode, uint8_t *pData) {
//     // 1 代表 to PC 2代表 to 驱动板
//     return HAL_UART_Transmit((mode == 1) ? &PCUART_Cfg : &UMUART_Cfg, pData, 1, 10);
// }
//
// /** @brief  MCU 发送数据
//   * @param mode 1代表 to PC 2代表 to 驱动板
//   * @param pData 数据指针
//   * @param size 数据长度
//   * @retval HAL_StatusTypeDef 返回状态
//   */
// HAL_StatusTypeDef UART_Transmit(const uint8_t mode, const uint8_t *pData, uint16_t size) {
//     // 1 代表 to PC 2代表 to 驱动板
//     return HAL_UART_Transmit((mode == 1) ? &PCUART_Cfg : &UMUART_Cfg, pData, size, Uart_MaxWait);
// }
//
// /**
//   * @brief  MCU 发送字符串
//   * @param mode 1代表 to PC 2代表 to 驱动板
//   * @param String 字符串指针
//   * @retval HAL_StatusTypeDef 返回状态
//   */
// HAL_StatusTypeDef UART_TransmitString(const uint8_t mode, const char *String) {
//     return HAL_UART_Transmit((mode == 1) ? &PCUART_Cfg : &UMUART_Cfg, (uint8_t *) String, strlen(String), Uart_MaxWait);
// }
//
// /**
//   * @brief  MCU 接收单个字节
//   * @param mode 1代表 to PC 2代表 to 驱动板
//   * @param pData 数据指针
//   * @retval HAL_StatusTypeDef 返回状态
//   */
// HAL_StatusTypeDef UART_ReceiveOnebit(const uint8_t mode, uint8_t *pData) {
//     // 1 代表 to PC 2代表 to 驱动板
//     return HAL_UART_Receive((mode == 1) ? &PCUART_Cfg : &UMUART_Cfg, pData, 1, 10);
// }


// 可以使用DMA完成
char *MDrv_ReadFlash(char *charArray) {
    // // 发送指令
    // HAL_StatusTypeDef status = HAL_UART_Transmit(&UMUART_Cfg, (uint8_t*)"$read_flash#", 12, Uart_MaxWait);
    // if (status != HAL_OK)
    //   return NULL;
    //
    // uint32_t index = 0;
    // char ch = 0;
    //
    // while (index < 63)  // 预留1字节给 '\0'
    // {
    //   status = UART_ReceiveOnebit(1, (uint8_t*)&ch);
    //   if (status != HAL_OK)
    //     return NULL;  // 接收失败或超时，返回NULL
    //
    //   charArray[index++] = ch;
    //
    //   if (ch == '#')  // 遇到结束符，结束接收
    //     break;
    // }
    //
    // charArray[index] = '\0';  // 字符串结束符
    // return charArray;
}





void MDrv_ProcessMALL(char *data) {
    // 处理 $MALL 数据 代表轮子转动的总编码器数据
    // 例如解析数据并存储到相应的变量中
    // 假设数据格式为 "$MAll:23427,23801,24543,24453#"
    sscanf(data, "$MAll:%f,%f,%f,%f", &mall[0], &mall[1], &mall[2], &mall[3]);
}

void MDrv_ProcessMTEP(char *data) {
    // 处理 $MTEP 数据 代表轮子转动的实时转动编码器数据 10ms
    // 例如解析数据并存储到相应的变量中
    // 假设数据格式为 "$MTEP:63,64,66,66#"·
    sscanf(data, "$MTEP:%d,%d,%d,%d", &mtep[0], &mtep[1], &mtep[2], &mtep[3]);
}

void MDrv_ProcessMSPD(char *data) {
    // 处理 $MSPD 数据 代表轮子转动的实时速度数据
    // 例如解析数据并存储到相应的变量中
    // 假设数据格式为 "$MSPD:608.99,618.65,637.98,637.98#"
    // sscanf(data, "$MSPD:%f,%f,%f,%f", &mspd[0], &mspd[1], &mspd[2], &mspd[3]);
    // if (mspd[0] - V[0] > 0.1f || mspd[0] - V[0] < -0.1f) {
    //   MDrv_SetPWM((mspd[0] - V[0] > 0.1f || mspd[0] - V[0] < -0.1f) ? : ); // 设置电机1的PWM值
    // }
    //
    //  MDrv_SetPWM(mtep) : (void)0;
    // if (mspd[1] - V[1] > 0.1f || mspd[1] - V[1] < -0.1f) {
    //
    // }
    // if (mspd[2] - V[2] > 0.1f || mspd[2] - V[2] < -0.1f) {
    //
    // }
    // if (mspd[3] - V[3] > 0.1f || mspd[3] - V[3] < -0.1f) {
    //
    // }
    //
    //   // 如果速度变化超过阈值，更新V数组
    //   memcpy(V, mspd, sizeof(V));
}


void MDrv_JudgeDataType(char *Udata) {
    if (strncmp(Udata, "$MALL", 5) == 0) {
        MDrv_ProcessMALL(Udata + 6);
    } else if (strncmp(Udata, "$MTEP", 5) == 0) {
        MDrv_ProcessMTEP(Udata + 6);
    } else if (strncmp(Udata, "$MSPD", 5) == 0) {
        MDrv_ProcessMSPD(Udata + 6);
    } else if (strncmp(Udata, "$Battery", 8) == 0) {
        // Battery 电池电压数据
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == huart2.Instance) {
        uint8_t received_char = RxBuff[0];

        // 检查是否是帧尾字符
        if (received_char == '#') {
            // 确保我们之前已经收到了一些数据
            if (RxLine > 0) {
                DataBuff[RxLine] = '\0'; // 添加字符串结束符
                MDrv_JudgeDataType((char *)DataBuff); // 处理完整的数据
            }
            // 不论之前有无数据，收到帧尾都意味着一帧的结束，重置计数器
            RxLine = 0;
        }
        // 如果不是帧尾，则为数据字节
        else {
            // 忽略回车和换行符，它们通常没有用
            if (received_char != '\r' && received_char != '\n') {
                // 检查缓冲区是否会溢出
                if (RxLine < (500 - 1)) { // 留一个位置给 '\0'
                    DataBuff[RxLine++] = received_char;
                } else {
                    // 缓冲区溢出，这是一个严重错误。丢弃当前所有数据并重置。
                    // 最好在这里加一个错误处理，比如点亮一个LED。
                    RxLine = 0;
                }
            }
        }

        // 每次处理完一个字节后，必须重新使能中断以接收下一个字节！
        // 这是保证连续接收的关键。
        HAL_UART_Receive_IT(&huart2, (uint8_t *)RxBuff, 1);
    }
}

// void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle) {
//     if (UartHandle == &UMUART_Cfg) {
//         RxLine++; //每接收到一个数据，进入回调数据长度加1
//         DataBuff[RxLine - 1] = RxBuff[0]; //把每次接收到的数据保存到缓存数组
//         if (RxBuff[0] == 0x24) //接收开始标志位 $
//         {
//             // RxLine = 1; // 重置接收长度
//             HAL_UART_Receive_IT(&UMUART_Cfg, (uint8_t *) RxBuff, 1); // 继续接收下一个字节
//             return;
//             // memset(DataBuff, 0, sizeof(DataBuff)); //清空缓存数组
//         }else if (RxBuff[0] == 0x23) // 接收结束标志位 #
//         {
//             DataBuff[RxLine] = '\0'; // 添加字符串结束符
//             MDrv_JudgeDataType((char *) DataBuff); // 处理接收到的数据
//             // memset(DataBuff, 0, sizeof(DataBuff)); // 清空缓存数组
//             RxLine = 0; // 清空接收长度
//         }else if (RxBuff[0] == 0x0A || RxBuff[0] == 0x0D) {
//             // 换行符或回车符，忽略
//             DataBuff[RxLine - 1] = '\0'; // 覆盖换行符
//             RxLine = 0; // 重置接收长度
//             MDrv_JudgeDataType(DataBuff); // 处理接收到的数据
//             HAL_UART_Receive_IT(&UMUART_Cfg, (uint8_t *) RxBuff, 1); // 继续接收下一个字节
//             return;
//         }
//         // other case
//         // 继续接收下一个字节
//         RxBuff[0] = 0;
//         HAL_UART_Receive_IT(&UMUART_Cfg, (uint8_t *) RxBuff, 1);
//         //每接收一个数据，就打开一次串口中断接收，否则只会接收一个数据就停止接收
//     }
// }
//

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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
    MDrv_Init();
    HAL_UART_Transmit(&UMUART_Cfg, (uint8_t *) "$upload:0,0,1#", 14, Uart_MaxWait); // 开启上传数据
    HAL_UART_Receive_IT(&UMUART_Cfg, (uint8_t *) RxBuff, 1); //打开串口中断接收
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    MDrv_GoSpeed(1, 100);
    while (1) {

    }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

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
    while (1) {
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
