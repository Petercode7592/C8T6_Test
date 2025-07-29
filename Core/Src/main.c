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
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "./Display/OLED/Oled.h"
#include "Eight_LineWalking.h"
#include "Four_LineWalking.h"
#include "Ganv_Grayscale_Sensor_Config.h"
#include "MDrv.h"
#include "MPU6050.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
volatile uint8_t mpu6050_data_ready_flag = 0; // 6050 数据就绪标志(中断)
volatile uint8_t tracking_data_ready_flag = 1; // =1 让其先触发一次
unsigned char UsrtDataBuff[500]; // 用于接收串口的数据
unsigned char rx_buff[256] = {0}; // 发送缓冲区
No_MCU_Sensor track_sensor; // 创建一个传感器对象
MPU6050_Data_t MPU6050_Data; // 定义一个MPU6050数据结构体实例
int16_t tim2_tick = 0; // 定时器2滴答计数器
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define Protection_Switch 0 // 1:防止SweetBaby		0:关闭 Protection 模块

#define Debug_Switch 0 // 1:开启 Debug 输出到串口 0:关闭 Debug 输出到串口 (Development Only)

#define Tracking4_Switch 0 // 1:开启 四路巡线 模块	0:关闭 LineWalking 模块
#define Tracking8_Switch 0 // 1:开启 八路巡线 模块	0:关闭 Tracking8 模块
#define MPU6050_Switch 1 // 1:开启 MPU6050 模块	0:关闭 MPU6050 模块
#define OLED_Switch 1 // 1:开启 OLED 模块		0:关闭 OLED 模块

#define Tracking4_Pin (GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// volatile 代表变量可能会被中断或其他线程修改，防止编译器优化


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Uart_JudgeDataType(char *Udata);
void K230_ProcessData(char *Udata);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * @brief GPIO中断回调函数
 * @param GPIO_Pin: 触发中断的GPIO引脚
 * @retval None
 * @note 需要注意的是引脚对应的可能相同，!应该尽量避免中断引脚Pin相同!
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (MPU6050_Switch && GPIO_Pin == MPU6050_EXTI_Pin) // PB9 – MPU INT
		mpu6050_data_ready_flag = 1;
	if (Tracking4_Switch && (GPIO_Pin & Tracking4_Pin))
		tracking_data_ready_flag = 1;
}

/**
 * @brief 定时器回调函数
 * @param htim: 定时器句柄
 * @retval None
 * @note 这里使用 TIM2 定时器作为系统滴答定时器 TIM2 定时器每 1ms 触发一次中断
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM2) {
		// 假设速度 200mm/s = 20cm/s 经过黑线 2cm 0.1s通过 可触发10次中断
		tim2_tick++; // 增加定时器滴答计数
		if (tim2_tick == 10) {
			// 每10ms触发一次
			Task_tick(&track_sensor);
			tim2_tick = 0; // 重置滴答计数
			tracking_data_ready_flag = 1; // 设置巡线数据就绪标志
		}
	}
}

/**
 * @brief UART接收完成回调函数
 * @param huart: UART句柄
 * @retval None
 * @note 空闲中断+DMA 这里不要听信代码建议加 const
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
	// 这个回调函数会在接收到数据时被调用
	if (huart->Instance == PCUMUART_Cfg.Instance) {
		UsrtDataBuff[Size - 1] = '\0'; // 确保字符串结束
		Uart_JudgeDataType(UsrtDataBuff); // [Debug 删除调速函数]处理接收到的数据
		HAL_UARTEx_ReceiveToIdle_DMA(&PCUMUART_Cfg, UsrtDataBuff, 500); // 继续接收字节
	}
}

void Uart_JudgeDataType(char *Udata) {
	if (Debug_Switch)
		HAL_UART_Transmit(&Bluetooth_Cfg, (uint8_t *) Udata, strlen(Udata), Uart_MaxWait);
	if (strncmp(Udata, "$", 1) == 0) {
		if (strncmp(Udata, "$K230:", 6) == 0) {
			K230_ProcessData(Udata + 6);
		}else if (strncmp(Udata, "$MAll:", 6) == 0) {
			MDrv_ProcessMALL(Udata + 6);
		} else if (strncmp(Udata, "$MTEP:", 6) == 0) {
			MDrv_ProcessMTEP(Udata + 6);
		} else if (strncmp(Udata, "$MSPD:", 6) == 0) {
			MDrv_ProcessMSPD(Udata + 6);
		} else if (strncmp(Udata, "$Battery:", 9) == 0) {
			MDrv_ProcessVol(Udata + 9);
		}
	}
}

void K230_ProcessData(char *Udata) {

}

void OLED_Debug(char mode, uint8_t showtype) {
	if (showtype == 0) {
		if (MPU6050_Switch)
			OLED_ShowString(1, 1, "6050", 8);
		OLED_ShowString(45, 1, "SPD", 8);
		OLED_ShowString(115, 7, "V", 8);
		if (Tracking8_Switch)
			OLED_ShowString(1, 6, "LineWalk", 8);
	} else if (showtype == 1) {
		if (MPU6050_Switch)
			OLED_ShowFloat(1, 2, MPU6050_Data.YawAngle, 3, 2, 8);
		OLED_ShowFloat(45, 2, Mdrv_Readspeed(0), 3, 1, 8);
		OLED_ShowFloat(45, 3, Mdrv_Readspeed(1), 3, 1, 8);
		OLED_ShowFloat(45, 4, Mdrv_Readspeed(2), 3, 1, 8);
		OLED_ShowFloat(45, 5, Mdrv_Readspeed(3), 3, 1, 8);
		OLED_ShowFloat(80, 7, Voltage, 1, 2, 8);
		// 显示 LineWalking 数据
		if (Tracking8_Switch) {
			OLED_ShowNum(1, 7, (track_sensor.Digtal >> 0) & 0x01, 1, 8);
			OLED_ShowNum(11, 7, (track_sensor.Digtal >> 1) & 0x01, 1, 8);
			OLED_ShowNum(21, 7, (track_sensor.Digtal >> 2) & 0x01, 1, 8);
			OLED_ShowNum(31, 7, (track_sensor.Digtal >> 3) & 0x01, 1, 8);
			OLED_ShowNum(41, 7, (track_sensor.Digtal >> 4) & 0x01, 1, 8);
			OLED_ShowNum(51, 7, (track_sensor.Digtal >> 5) & 0x01, 1, 8);
			OLED_ShowNum(61, 7, (track_sensor.Digtal >> 6) & 0x01, 1, 8);
			OLED_ShowNum(71, 7, (track_sensor.Digtal >> 7) & 0x01, 1, 8);
		}
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
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
	if (Protection_Switch)
		while (1) {
		}


	No_MCU_Ganv_Sensor_Init(&track_sensor); // 初始化八路循迹传感器
	HAL_TIM_Base_Start_IT(&htim2); // 启动定时器2中断

	MDrv_Init(0); // 初始化驱动板

	HAL_UARTEx_ReceiveToIdle_DMA(&PCUMUART_Cfg, UsrtDataBuff, 500); // 启动DMA接收，若有串口数据传送需在该函数后

	HAL_Delay(100);
	MDrv_RequestVol(); // 发送获取电池电压指令
	HAL_Delay(100);
	MDrv_UploadData(0, 0, 1); // 获取电机编码器数据

	LineWalking_Init(); // 初始化 PID 控制器

	if (OLED_Switch)
		OLED_Init(&hi2c2);

	if (MPU6050_Switch && !MPU6050_Init(&hi2c1)) {
		// 初始化MPU6050
		HAL_Delay(750); // 延时等待稳定
		OLED_ShowString(1, 1, "MPU6050 Init....", 8); // OLED 显示 MPU6050 初始化信息
		MPU6050_Calibrate_Gyro(); // 校准陀螺仪偏置 不可转动
	} else {
		OLED_ShowString(1, 2, "MPU6050 Err or OFF!", 8); // 6050错误信息提示
		HAL_Delay(1500);
	}

	if (OLED_Switch) {
		OLED_Clear();
		OLED_Debug('N', 0); // OLED 显示 LineWalking 模块标题[Debug]
	}

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
		OLED_Debug('N', 1); // OLED 显示 LineWalking 数据
		if (MPU6050_Switch) {
			if (mpu6050_data_ready_flag) {
				mpu6050_data_ready_flag = 0;
				MPU6050_Get_Angle_yaw(&MPU6050_Data); // 获取陀螺仪数据
			}
		}

		if (tracking_data_ready_flag) {
			tracking_data_ready_flag = 0;
			if (Tracking4_Switch) // 如果打开四路循迹
				LineWalking(150);
			if (Tracking8_Switch) {
				// 如果打开八路循迹
				No_Mcu_Ganv_Sensor_Task_With_tick(&track_sensor); // 传感器任务处理
				LineWalking8_Track(track_sensor.Digtal, 200);
			}
		}
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
