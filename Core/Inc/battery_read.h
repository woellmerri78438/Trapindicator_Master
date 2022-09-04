/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    battery_read.h
  * @author  Richard Woellmer
  * @brief   Header for ADC Battery-Read functions
  ******************************************************************************
  * @attention
  *
  * Leon Thürauf, Richard Woellmer, TH Nuernberg
  *
  *
  ******************************************************************************
  */
/* USER CODE END Header */


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BATTERY_READ_H__
#define __BATTERY_READ_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdint.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
/* USER CODE END EC */

/* External variables --------------------------------------------------------*/
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/

/* USER CODE BEGIN EFP */
float Berechnung_Akkustand(uint32_t R1, uint32_t R2);
int Ausgabe_Akkustand(float Vakku);
/* USER CODE END EFP */

#ifdef __cplusplus
}
#endif

#endif /* __BATTERY_READ_H__ */
