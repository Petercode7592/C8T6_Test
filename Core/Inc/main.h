/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h" // 重定向 Printf
#include "MPU6050.h" // 包含MPU6050模块
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define UUSART_TX_Pin GPIO_PIN_2
#define UUSART_TX_GPIO_Port GPIOA
#define UUSART_RX_Pin GPIO_PIN_3
#define UUSART_RX_GPIO_Port GPIOA
#define VL53L1X_EXTI_Pin GPIO_PIN_4
#define VL53L1X_EXTI_GPIO_Port GPIOA
#define VL53L1X_EXTI_EXTI_IRQn EXTI4_IRQn
#define Track8_AD2_Pin GPIO_PIN_7
#define Track8_AD2_GPIO_Port GPIOA
#define Track8_AD0_Pin GPIO_PIN_0
#define Track8_AD0_GPIO_Port GPIOB
#define Track8_AD1_Pin GPIO_PIN_1
#define Track8_AD1_GPIO_Port GPIOB
#define OLED_SCL_Pin GPIO_PIN_10
#define OLED_SCL_GPIO_Port GPIOB
#define OLED_SDA_Pin GPIO_PIN_11
#define OLED_SDA_GPIO_Port GPIOB
#define PCUSART_TX_Pin GPIO_PIN_9
#define PCUSART_TX_GPIO_Port GPIOA
#define PCUSART_RX_Pin GPIO_PIN_10
#define PCUSART_RX_GPIO_Port GPIOA
#define MPU6050_EXTI_Pin GPIO_PIN_9
#define MPU6050_EXTI_GPIO_Port GPIOB
#define MPU6050_EXTI_EXTI_IRQn EXTI9_5_IRQn

/* USER CODE BEGIN Private defines */
/* 这里定义的4个宏定义是为了方便使用四路循迹传感器的引脚 现在已经废弃 需要再在 cubemx 中配置
*#define Track_L2X1_Pin GPIO_PIN_12
#define Track_L2X1_GPIO_Port GPIOB
#define Track_L2X1_EXTI_IRQn EXTI15_10_IRQn
#define Track_L1X2_Pin GPIO_PIN_13
#define Track_L1X2_GPIO_Port GPIOB
#define Track_L1X2_EXTI_IRQn EXTI15_10_IRQn
#define Track_R1X3_Pin GPIO_PIN_14
#define Track_R1X3_GPIO_Port GPIOB
#define Track_R1X3_EXTI_IRQn EXTI15_10_IRQn
#define Track_R2X4_Pin GPIO_PIN_15
#define Track_R2X4_GPIO_Port GPIOB
#define Track_R2X4_EXTI_IRQn EXTI15_10_IRQn
*/
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
