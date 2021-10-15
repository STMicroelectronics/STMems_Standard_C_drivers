/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
#define DIR_DEN_CS_A_Pin GPIO_PIN_13
#define DIR_DEN_CS_A_GPIO_Port GPIOC
#define TEST_Adapter_Connected_Pin GPIO_PIN_14
#define TEST_Adapter_Connected_GPIO_Port GPIOC
#define DIR_INT3_INT4_Pin GPIO_PIN_15
#define DIR_INT3_INT4_GPIO_Port GPIOC
#define TEST_5_Pin GPIO_PIN_1
#define TEST_5_GPIO_Port GPIOB
#define LED2_Pin GPIO_PIN_7
#define LED2_GPIO_Port GPIOE
#define LED3_Pin GPIO_PIN_8
#define LED3_GPIO_Port GPIOE
#define LED1_Pin GPIO_PIN_9
#define LED1_GPIO_Port GPIOE
#define SW2_Pin GPIO_PIN_10
#define SW2_GPIO_Port GPIOB
#define CS_up_Pin GPIO_PIN_12
#define CS_up_GPIO_Port GPIOB
#define TEST_6_Pin GPIO_PIN_8
#define TEST_6_GPIO_Port GPIOD
#define DIR_GP_Pin GPIO_PIN_9
#define DIR_GP_GPIO_Port GPIOD
#define INT3_up_Pin GPIO_PIN_13
#define INT3_up_GPIO_Port GPIOD
#define INT4_up_Pin GPIO_PIN_14
#define INT4_up_GPIO_Port GPIOD
#define CTR_EN_Pin GPIO_PIN_15
#define CTR_EN_GPIO_Port GPIOD
#define USB_disc_Pin GPIO_PIN_9
#define USB_disc_GPIO_Port GPIOA
#define SW1_Pin GPIO_PIN_10
#define SW1_GPIO_Port GPIOA
#define Range_100x_A_Pin GPIO_PIN_0
#define Range_100x_A_GPIO_Port GPIOD
#define Range_100x_B_Pin GPIO_PIN_1
#define Range_100x_B_GPIO_Port GPIOD
#define CTR_EN_I2C_SD_Pin GPIO_PIN_2
#define CTR_EN_I2C_SD_GPIO_Port GPIOD
#define INT2_up_Pin GPIO_PIN_5
#define INT2_up_GPIO_Port GPIOB
#define INT2_up_EXTI_IRQn EXTI9_5_IRQn
#define INT1_up_Pin GPIO_PIN_8
#define INT1_up_GPIO_Port GPIOB
#define INT1_up_EXTI_IRQn EXTI9_5_IRQn
#define CS_A_up_Pin GPIO_PIN_9
#define CS_A_up_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
