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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <sys/types.h>

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
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// UART
#define PCUART_Cfg huart1           // PC 通信串口配置
#define UMUART_Cfg huart2           // MCU-MCU 通信串口配置
#define Uart_MaxWait (uint32_t)25  // 传输时间 = 字节数 × 10位 / 波特率 (10位=1+8+0+1)  30ms 建议100B内

/** @brief  MCU 发送单个字节
  * @param mode 1代表 to PC 2代表 to 驱动板
  * @param pData 数据指针
  * @retval HAL_StatusTypeDef 返回状态
  */
HAL_StatusTypeDef UART_TransmitOnebit(const uint8_t mode, uint8_t *pData)
{
  // 1 代表 to PC 2代表 to 驱动板
  return HAL_UART_Transmit((mode == 1) ? &PCUART_Cfg : &UMUART_Cfg, pData, 1, 10);
}

/** @brief  MCU 发送数据
  * @param mode 1代表 to PC 2代表 to 驱动板
  * @param pData 数据指针
  * @param size 数据长度
  * @retval HAL_StatusTypeDef 返回状态
  */
HAL_StatusTypeDef UART_Transmit(const uint8_t mode, const uint8_t *pData,uint16_t size)
{
  // 1 代表 to PC 2代表 to 驱动板
  return HAL_UART_Transmit((mode == 1) ? &PCUART_Cfg : &UMUART_Cfg, pData, size, Uart_MaxWait);
}

/**
  * @brief  MCU 发送字符串
  * @param mode 1代表 to PC 2代表 to 驱动板
  * @param String 字符串指针
  * @retval HAL_StatusTypeDef 返回状态
  */
HAL_StatusTypeDef UART_TransmitString(const uint8_t mode, const char *String)
{
  return HAL_UART_Transmit((mode == 1) ? &PCUART_Cfg : &UMUART_Cfg, (uint8_t*) String, strlen(String), Uart_MaxWait);
}

/**
  * @brief  MCU 接收单个字节
  * @param mode 1代表 to PC 2代表 to 驱动板
  * @param pData 数据指针
  * @retval HAL_StatusTypeDef 返回状态
  */
HAL_StatusTypeDef UART_ReceiveOnebit(const uint8_t mode, uint8_t *pData)
{
  // 1 代表 to PC 2代表 to 驱动板
  return HAL_UART_Receive((mode == 1) ? &PCUART_Cfg : &UMUART_Cfg, pData, 1, 10);
}


// 可以使用DMA完成
char* MDrv_ReadFlash(char* charArray)
{

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

char* MDrv_ReadVol(char* charArray)
{
  // 发送指令
  HAL_StatusTypeDef status = HAL_UART_Transmit(&UMUART_Cfg, (uint8_t *) "$read_vol#", 10, Uart_MaxWait);
  if (status != HAL_OK)
    return NULL;

  uint32_t index = 0;
  char ch = 0;

  while (index < 63)  // 预留1字节给 '\0'
  {
    status = UART_ReceiveOnebit(1, (uint8_t*) &ch);
    if (status != HAL_OK)
      return NULL;    // 接收失败或超时，返回NULL或已收到内容
    charArray[index++] = ch;

    if (ch == '#')  // 遇到结束符，结束接收
      break;
  }

  charArray[index] = '\0';  // 字符串结束符
  return charArray;
}

/** @brief  (Checked)MCU 驱动板复位
  * @retval HAL_StatusTypeDef 返回状态
  */
HAL_StatusTypeDef MDrv_Reset()
{
  return HAL_UART_Transmit(&UMUART_Cfg, (uint8_t*)"$flash_reset#", 13, Uart_MaxWait);
}

/** @brief 配置电机类型
  * @param mtype 电机类型，1: 520电机 2：310电机 3: TT电机(带有编码器的) 4: TT电机(不带编码器)
  * @note 固件默认值为1 (520电机)
  * @retval HAL_StatusTypeDef 返回状态
  */
HAL_StatusTypeDef MDrv_SetMtype(uint8_t mtype)
{
  char cmd[25];
  snprintf(cmd, sizeof(cmd), "$mtype:%d#", mtype);
  return HAL_UART_Transmit(&UMUART_Cfg, (uint8_t*)cmd, strlen(cmd), Uart_MaxWait);
}

/**
  * @brief 配置电机的相位线
  * @param mline 是霍尔编码器转一圈的相位，此值需要查商家电机的参数表可得
  * @note 带编码器电机来说:此值对于控制速度起主要作用，此值需要正确.配置电机霍尔编码器的相位为13线
  * @retval HAL_StatusTypeDef 返回状态
  */
HAL_StatusTypeDef Mdrv_SetMline(uint32_t mline)
{
  char cmd[25];
  snprintf(cmd, sizeof(cmd), "$mline:%d#", mline);  // 发送指令
  return HAL_UART_Transmit(&UMUART_Cfg, (uint8_t*)cmd, strlen(cmd), Uart_MaxWait);
}

/**
  * @brief 配置电机的减速比
  * @param mphase 减速比，固件默认。此值需要查商家电机的参数表可得
  * @note 带编码器电机来说:此值对于控制速度起主要作用，此值需要正确.固件默认值30
  * @retval HAL_StatusTypeDef 返回状态
  */
HAL_StatusTypeDef Mdrv_SetMphase(uint8_t mphase)
{
  char cmd[25];
  snprintf(cmd, sizeof(cmd), "$mphase:%d#", mphase);  // 发送指令
  return HAL_UART_Transmit(&UMUART_Cfg, (uint8_t*)cmd, strlen(cmd), Uart_MaxWait);
}

/**
  * @brief 配置电机的pwm脉冲死区
  * @param deadzone 死区值，范围0-3600。这个需要测量得出,可通过改变此值，消除电机最小震荡。死区值的范围(0-3600)
  * @note 固件默认值1600.这样控制pwm时会默认加上死区的值，使得电机不会出现震荡区域
  * @retval HAL_StatusTypeDef 返回状态
  */
HAL_StatusTypeDef MDrv_SetDeadzone(uint32_t deadzone)
{
  char cmd[25];
  if (deadzone >= 0 && deadzone <= 3600) {
    // 确保死区值在0到3600之间
    snprintf(cmd, sizeof(cmd), "$deadzone:%d#", deadzone);  // 发送指令
    return HAL_UART_Transmit(&UMUART_Cfg, (uint8_t*)cmd, strlen(cmd), Uart_MaxWait);
  }
  return HAL_ERROR;  // 错误处理
}

/**
  * @brief (可选)配置车轮直径
  * @param wdiameter 车轮直径，单位为毫米
  * @note 带编码器电机来说：此值对于控制速度起主要作用，此值单位为毫米（mm）；如果此值不正确，只会影响速度数据不准确，不会影响编码器的数据
  * @retval HAL_StatusTypeDef 返回状态
  */
HAL_StatusTypeDef MDrv_SetWdiameter(uint8_t wdiameter)
{
  char cmd[25];
  snprintf(cmd, sizeof(cmd), "$wdiameter:%d#", wdiameter);  // 发送指令
  return HAL_UART_Transmit(&UMUART_Cfg, (uint8_t*)cmd, strlen(cmd), Uart_MaxWait);
}

/** @brief 设置PID参数, 设置后芯片会重启，会把正在运动的电机停止
  * @param p 比例系数
  * @param i 积分系数
  * @param d 微分系数
  * @retval HAL_StatusTypeDef 返回状态
  */
HAL_StatusTypeDef MDrv_SetPID(double p, double i, double d)
{
  // 发送指令
  char cmd[50];
  snprintf(cmd, sizeof(cmd), "$MPID:%ld,%ld,%ld#", p, i, d);
  return HAL_UART_Transmit(&UMUART_Cfg, (uint8_t*)cmd, strlen(cmd), Uart_MaxWait);
}

/** * @brief 设置各个电机速度
  * @param Vm1 电机1速度 (-1000到1000之间)
  * @param Vm2 电机2速度 (-1000到1000之间)
  * @param Vm3 电机3速度 (-1000到1000之间)
  * @param Vm4 电机4速度 (-1000到1000之间)
  * @retval HAL_StatusTypeDef 返回状态
  */
HAL_StatusTypeDef MDrv_SetSpeed(const int32_t Vm1, const int32_t Vm2, const int32_t Vm3, const int32_t Vm4)
{
  char cmd[50];
  if (Vm1 < -1000 || Vm1 > 1000 || Vm2 < -1000 || Vm2 > 1000 || Vm3 < -1000 || Vm3 > 1000 || Vm4 < -1000 || Vm4 > 1000) {
    return HAL_ERROR;  // 错误处理，速度值超出范围
  }
  snprintf(cmd, sizeof(cmd), "$spd:%d,%d,%d,%d#", Vm1, Vm2, Vm3, Vm4);
  return HAL_UART_Transmit(&UMUART_Cfg, (uint8_t*)cmd, strlen(cmd), Uart_MaxWait);
}

/**
  * @brief 设置电机的PWM值
  * @param pwm 电机的PWM值数组，长度为4，范围为-3600到3600
  * @note 此函数会设置所有电机的PWM值，pwm[0]对应电机1，pwm[1]对应电机2，pwm[2]对应电机3，pwm[3]对应电机4
  */
HAL_StatusTypeDef MDrv_SetPWM(int16_t* pwm) {
  // 设置电机的PWM值 -3600 到 3600 之间
  char cmd[50];
  snprintf(cmd, sizeof(cmd), "$pwm:%d,%d,%d,%d#", pwm[0], pwm[1], pwm[2], pwm[3]);
  return HAL_UART_Transmit(&UMUART_Cfg, (uint8_t*)cmd, strlen(cmd), Uart_MaxWait);
}

/**
  * @brief 设置电机速度
  * @param mode 1: 正向速度, -1: 反向速度, 0: 停止
  * @param Vm 电机速度 (-1000到1000之间)
  * @note 此函数会设置所有电机的速度，Vm为正时表示正向转动，负时表示反向转动，0时表示停止
  */
void MDrv_GoSpeed(uint8_t mode, int32_t Vm) {
  if (mode == 1)
    MDrv_SetSpeed(Vm, Vm, Vm, Vm); // 设置所有电机速度
  else if (mode == -1)
    MDrv_SetSpeed(-Vm, -Vm, -Vm, -Vm); // 转置所有电机速度
  else if (mode == 0)
    MDrv_SetSpeed(0, 0, 0, 0); // 停止所有电机
}


/**
  * @brief (bug)初始化驱动板
  * @retval HAL_StatusTypeDef 返回状态
  * @note 该函数会设置电机类型、相位线、车轮直径和速度等参数。为确保设置生效各命令延时15ms,收到HAL_OK只代表发出去不代表已经MD收到
  * @bug 1. 设置电机速度无论延迟多少都无法正确设置，可能要收到单片机回传的正确时间
  * @bug 2. 死区值需要测量得出，不能随意设置
  */
void MDrv_Init() {
  // MDrv_SetPID(0.8, 0.06, 0.5);
  MDrv_SetSpeed(0,0,0,0);   // 设置电机速度
  HAL_Delay(90);
  MDrv_SetMtype(2);         // 设置为310电机
  HAL_Delay(90);
  Mdrv_SetMline(13);        // 设置电机的相位线
  HAL_Delay(90);
  Mdrv_SetMphase(20);       // 设置电机的减速比
  HAL_Delay(90);
  MDrv_SetWdiameter(48);    // 设置车轮直径
  HAL_Delay(90);
  MDrv_SetDeadzone(1650);   // 设置电机的pwm脉冲死区,需要测量得出
  HAL_Delay(90);
}
/* USER CODE END 0 */

/*
 * 该部分的函数主要是为了处理四路驱动板上接收到的数据】
 * 以下将给出部分数据
 * $MTEP:63,64,66,66#
 * $MSPD:608.99,618.65,637.98,637.98#
 * $MAll:23427,23801,24543,24453#
 * $MTEP:63,64,66,65#
 * $MAll:23490,23865,24609,24518#
 * $MTEP:63,64,66,66#
 * $MAll:23553,23929,24675,24584#
 * $MTEP:63,64,66,66#
 * $MAll:23616,23993,24741,24650#
 * $MTEP:63,64,66,66#
 * $MAll:23679,24057,24807,24715#
 * $MTEP:63,64,66,65#
 * $MSPD:608.99,618.65,637.98,637.98#
  */


void MDrv_ProcessMALL(char *mall) {
  // 处理 $MALL 数据 代表轮子转动的总编码器数据
  // 例如解析数据并存储到相应的变量中
  // 假设数据格式为 "$MAll:23427,23801,24543,24453#"

}

void MDrv_ProcessMTEP(char *mtep) {
  // 处理 $MTEP 数据 代表轮子转动的实时转动编码器数据 10ms
  // 例如解析数据并存储到相应的变量中
  // 假设数据格式为 "$MTEP:63,64,66,66#"·
}

void MDrv_ProcessMSPD(char *mspd) {
  // 处理 $MSPD 数据 代表轮子转动的实时速度数据
  // 例如解析数据并存储到相应的变量中
  // 假设数据格式为 "$MSPD:608.99,618.65,637.98,637.98#"
}


void MDrv_JudgeDataType(char *Udata) {
  if (strncmp(Udata, "$MALL", 5) == 0) {
    MDrv_ProcessMALL(Udata + 6);
  }
  else if (strncmp(Udata, "$MTEP", 5) == 0) {
    MDrv_ProcessMTEP(Udata + 6);
  }
  else if (strncmp(Udata, "$MSPD", 5) == 0) {
    MDrv_ProcessMSPD(Udata + 6);
  }
}
















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

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  MDrv_Init();
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
  huart1.Init.BaudRate = 115200;
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
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

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
