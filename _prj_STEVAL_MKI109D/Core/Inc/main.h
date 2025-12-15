/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h5xx_hal.h"

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
#define TEST_6_Pin GPIO_PIN_13
#define TEST_6_GPIO_Port GPIOC
#define I2C_pullups_Pin GPIO_PIN_7
#define I2C_pullups_GPIO_Port GPIOF
#define EEPROM_WC_Pin GPIO_PIN_2
#define EEPROM_WC_GPIO_Port GPIOB
#define SAI_MCLK_FS_OE_Pin GPIO_PIN_0
#define SAI_MCLK_FS_OE_GPIO_Port GPIOG
#define SAI_MCLK_DIR_Pin GPIO_PIN_1
#define SAI_MCLK_DIR_GPIO_Port GPIOG
#define OE_INT_Pin GPIO_PIN_12
#define OE_INT_GPIO_Port GPIOE
#define LED2_Pin GPIO_PIN_13
#define LED2_GPIO_Port GPIOE
#define LED1_Pin GPIO_PIN_14
#define LED1_GPIO_Port GPIOE
#define LED3_Pin GPIO_PIN_15
#define LED3_GPIO_Port GPIOE
#define DIR_INT3_INT4_Pin GPIO_PIN_8
#define DIR_INT3_INT4_GPIO_Port GPIOD
#define DIR_INT1_INT2_Pin GPIO_PIN_9
#define DIR_INT1_INT2_GPIO_Port GPIOD
#define SW1_Pin GPIO_PIN_11
#define SW1_GPIO_Port GPIOD
#define INT1_uP_Pin GPIO_PIN_12
#define INT1_uP_GPIO_Port GPIOD
#define INT1_uP_EXTI_IRQn EXTI12_IRQn
#define INT2_uP_Pin GPIO_PIN_13
#define INT2_uP_GPIO_Port GPIOD
#define INT2_uP_EXTI_IRQn EXTI13_IRQn
#define INT3_uP_Pin GPIO_PIN_14
#define INT3_uP_GPIO_Port GPIOD
#define INT3_uP_EXTI_IRQn EXTI14_IRQn
#define INT4_uP_Pin GPIO_PIN_15
#define INT4_uP_GPIO_Port GPIOD
#define INT4_uP_EXTI_IRQn EXTI15_IRQn
#define SAI_SCK_SD_OE_Pin GPIO_PIN_2
#define SAI_SCK_SD_OE_GPIO_Port GPIOG
#define CTR_EN_Pin GPIO_PIN_3
#define CTR_EN_GPIO_Port GPIOG
#define DIR_DEN_CS_A_Pin GPIO_PIN_4
#define DIR_DEN_CS_A_GPIO_Port GPIOG
#define DIR_GP_Pin GPIO_PIN_5
#define DIR_GP_GPIO_Port GPIOG
#define CS_A_uP_Pin GPIO_PIN_6
#define CS_A_uP_GPIO_Port GPIOG
#define DEN_uP_Pin GPIO_PIN_8
#define DEN_uP_GPIO_Port GPIOG
#define MUX_Power_ON_Pin GPIO_PIN_5
#define MUX_Power_ON_GPIO_Port GPIOD
#define DIL24_SPIx6_CS_Pin GPIO_PIN_6
#define DIL24_SPIx6_CS_GPIO_Port GPIOD
#define SPI1_NSS_Pin GPIO_PIN_10
#define SPI1_NSS_GPIO_Port GPIOG
#define GP_up_Pin GPIO_PIN_15
#define GP_up_GPIO_Port GPIOG
#define SW2_Pin GPIO_PIN_0
#define SW2_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
