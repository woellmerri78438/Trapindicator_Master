/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    flashparam.h
  * @author  Richard Woellmer
  * @brief   Header for Flash Code
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
#ifndef __FLASHPARAM_H__
#define __FLASHPARAM_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/*Typedefs*/
typedef struct {
	char Handynummer[15];
	char ID[8];
	uint64_t Wakeup;
	char Handynummer1[8];
	char Handynummer2[8];
} Parametersatztype;


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
void Eraseflash_func(void);
void Writeflash_func(uint64_t Handynr1, uint64_t Handynr2, uint64_t wakeupmins);
Parametersatztype Readflash_func(void);

/* USER CODE BEGIN EFP */


/* USER CODE END EFP */

#ifdef __cplusplus
}
#endif

#endif /* __BATTERY_READ_H__ */
