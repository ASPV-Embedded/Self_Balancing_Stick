/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "stm32f4xx_hal.h"
#include "stdint.h"
#include "stdlib.h"
#include "math.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define CW_CCW_X_Pin GPIO_PIN_14
#define CW_CCW_X_GPIO_Port GPIOC
#define CW_CCW_Y_Pin GPIO_PIN_15
#define CW_CCW_Y_GPIO_Port GPIOC
#define BRAKE_XY_Pin GPIO_PIN_0
#define BRAKE_XY_GPIO_Port GPIOH
#define PWM_DCM_X_Pin GPIO_PIN_0
#define PWM_DCM_X_GPIO_Port GPIOA
#define PWM_DCM_Y_Pin GPIO_PIN_1
#define PWM_DCM_Y_GPIO_Port GPIOA
#define ENC_X_A_Pin GPIO_PIN_6
#define ENC_X_A_GPIO_Port GPIOA
#define ENC_X_B_Pin GPIO_PIN_7
#define ENC_X_B_GPIO_Port GPIOA
#define BAT_STATUS_Pin GPIO_PIN_4
#define BAT_STATUS_GPIO_Port GPIOC
#define IMU_SCL_Pin GPIO_PIN_10
#define IMU_SCL_GPIO_Port GPIOB
#define IMU_SDA_Pin GPIO_PIN_12
#define IMU_SDA_GPIO_Port GPIOC
#define ENC_Y_A_Pin GPIO_PIN_6
#define ENC_Y_A_GPIO_Port GPIOB
#define ENC_Y_B_Pin GPIO_PIN_7
#define ENC_Y_B_GPIO_Port GPIOB
#define BUZ_PWM_Pin GPIO_PIN_8
#define BUZ_PWM_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
