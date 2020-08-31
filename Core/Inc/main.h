/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#define DIO1_Pin GPIO_PIN_0
#define DIO1_GPIO_Port GPIOC
#define DIO2_Pin GPIO_PIN_1
#define DIO2_GPIO_Port GPIOC
#define DIO3_Pin GPIO_PIN_2
#define DIO3_GPIO_Port GPIOC
#define DIO4_Pin GPIO_PIN_3
#define DIO4_GPIO_Port GPIOC
#define DC_Pin GPIO_PIN_0
#define DC_GPIO_Port GPIOA
#define SC_Pin GPIO_PIN_1
#define SC_GPIO_Port GPIOA
#define PE_Pin GPIO_PIN_4
#define PE_GPIO_Port GPIOA
#define TE_Pin GPIO_PIN_5
#define TE_GPIO_Port GPIOA
#define SRQ_Pin GPIO_PIN_6
#define SRQ_GPIO_Port GPIOA
#define ATN_Pin GPIO_PIN_7
#define ATN_GPIO_Port GPIOA
#define DIO5_Pin GPIO_PIN_4
#define DIO5_GPIO_Port GPIOC
#define DIO6_Pin GPIO_PIN_5
#define DIO6_GPIO_Port GPIOC
#define EOI_Pin GPIO_PIN_0
#define EOI_GPIO_Port GPIOB
#define DAV_Pin GPIO_PIN_1
#define DAV_GPIO_Port GPIOB
#define BOOT1_Pin GPIO_PIN_2
#define BOOT1_GPIO_Port GPIOB
#define NRFD_Pin GPIO_PIN_12
#define NRFD_GPIO_Port GPIOB
#define NDAC_Pin GPIO_PIN_13
#define NDAC_GPIO_Port GPIOB
#define IFC_Pin GPIO_PIN_14
#define IFC_GPIO_Port GPIOB
#define REN_Pin GPIO_PIN_15
#define REN_GPIO_Port GPIOB
#define DIO7_Pin GPIO_PIN_6
#define DIO7_GPIO_Port GPIOC
#define DIO8_Pin GPIO_PIN_7
#define DIO8_GPIO_Port GPIOC
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
