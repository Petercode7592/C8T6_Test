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
#include "dma.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Mdrv.h"
#include "string.h"
#include "MPU6050.h" // 包含MPU6050模块
#include "oled.h"
#include "PID.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
MPU6050_Data_t MPU6050_Data; // 定义一个MPU6050数据结构体实例
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// volatile 代表变量可能会被中断或其他线程修改，防止编译器优化
volatile float V[4] = {0.0f, 0.0f, 0.0f, 0.0f};
volatile int32_t Vstd[4] = {0, 0, 0, 0}; // 标准速度
// volatile PID_Controller motor_pids[4];

uint8_t RxBuff[1]; //进入中断接收数据的数组
uint8_t DataBuff[200]; //保存接收到的数据的数组
int RxLine = 0; //接收到的数据长度

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void Uart_JudgeDataType(char *Udata) {
  if (strncmp(Udata, "$MALL:", 6) == 0) {
    MDrv_ProcessMALL(Udata + 6);
  } else if (strncmp(Udata, "$MTEP:", 6) == 0) {
    MDrv_ProcessMTEP(Udata + 6);
  } else if (strncmp(Udata, "$MSPD:", 6) == 0) {
    MDrv_ProcessMSPD(Udata + 6);
  } else if (strncmp(Udata, "$Battery:", 9) == 0) {
    // Battery 电池电压数据
  }
}


// 空闲中断+DMA 这里不要听信代码建议加 const
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
  // 这个回调函数会在接收到数据时被调用
  if (huart->Instance == UMUART_Cfg.Instance) {
    DataBuff[Size] = '\0'; // 确保字符串结束
    Uart_JudgeDataType(DataBuff); // [Debug 删除调速函数]处理接收到的数据
    // HAL_UART_Transmit(&UMUART_Cfg, DataBuff, Size, Uart_MaxWait); // Debug【回传】
    HAL_UARTEx_ReceiveToIdle_DMA(&UMUART_Cfg, DataBuff, 200); // 继续接收下一个字节
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
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
    MDrv_Init();
    Mdrv_UploadData(0,0,1); // 上传MSPD到MCU
    HAL_UARTEx_ReceiveToIdle_DMA(&UMUART_Cfg, DataBuff, 200); // 启动DMA接收，接收单个字节
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  // HAL_Delay(3000);
  // MDrv_GoSpeed(1, 200);
  OLED_Init(&hi2c2);
  MPU6050_Init();
  OLED_ShowString(1, 1, "MPU6050 Data", 8);
  while (1) {
    MPU6050_Read_All(&MPU6050_Data);

    OLED_ShowFloat(1,2,MPU6050_Data.Ax,4,4,8);
    OLED_ShowFloat(1,3,MPU6050_Data.Ay,4,4,8);
    OLED_ShowFloat(1,4,MPU6050_Data.Az,4,4,8);
    OLED_ShowFloat(1,5,MPU6050_Data.Gx,4,4,8);
    OLED_ShowFloat(1,6,MPU6050_Data.Gy,4,4,8);
    OLED_ShowFloat(1,7,MPU6050_Data.Gz,4,4,8);

    // MPU6050_Read_Accel(&MPU6050_Data);
    // sprintf(num,sizeof(num), "A:%f:%f:%f", &MPU6050_Data.Accel_X_RAW, &MPU6050_Data.Accel_Y_RAW, &MPU6050_Data.Accel_Z_RAW);

    // OLED_ShowString(1,1,num[0], 8);
    // OLED_ShowString(2,1,num[1], 8);
    // OLED_ShowString(3,1,num[2], 8);

    // HAL_UART_Transmit(&PCUART_Cfg, (uint8_t *) num,strlen(num), Uart_MaxWait);

    // OLED_ShowNum(1,1,MPU6050_Data.Accel_X_RAW, 8,8);
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
