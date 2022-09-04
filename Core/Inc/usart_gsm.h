/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart_gsm.h
  * @author  Richard Woellmer
  * @brief   Header for USART - GSM SIM800L - Module Function File
  ******************************************************************************
  * @attention
  *
  * Richard Woellmer, TH Nuernberg
  *
  *
  ******************************************************************************
  */
/* USER CODE END Header */


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USART_GSM_H__
#define __USART_GSM_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdint.h"
#include "usart.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
#define SMS_MSG_LEN		160
/* USER CODE END EC */

/* External variables --------------------------------------------------------*/
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/

/* USER CODE BEGIN EFP */

/*Special Callback for GSM-Uart. called by General UART Callback Event*/

void GSM_UART_RXCallback(UART_HandleTypeDef *huart, uint16_t Size);


/*
 * @brief Sends a simple "AT" command to Module and evaluates answer. Module should respond with OK.
 *  ! Also necessary for auto bauding function of the module.
 * @retval 1: Ok // 101: unexpected wrong data received (?) //  102: Timeout, no response
 *
 */
uint8_t GSMSimpleAT(void);


/*
 * @brief Sends special AT command to Module and evaluates the answer to get Simcard insertion status
 * @retval 1: Sim Inserted // 0: Sim Not inserted // 101: unexpected wrong data received (?) // 102: Timeout, no response
 *
 */
uint8_t GSMCheckSimInsert(void);

/*
 * @brief Sends special AT command to Module and evaluates the answer to get Network registration status
 * @retval 1: Registered, home network // 0: Not registered, MT is not currently searching //
 *  // 2: Not registered, but MT is currently searching // 3: Registration denied // 4: unknown
 *  // 5: Registered with Roaming
 *  // 101: unexpected wrong data received (?) // 102: Timeout, no response
 *
 */
uint8_t GSMGetRegState(void);

/*
 * @brief Sends special AT command to Module and evaluates the answer to get GSM Signal strength
 * @retval [RSSI] 0: -115 dBm or less // 1:  -111 dBm // 2...30:  -110... -54 dBm //
 * 31: -52 dBm or greater // 99:  not known or not detectable //
 *  // 101: unexpected wrong data received (?) // 102: Timeout, no response
 *
 */
uint8_t GSMGetSignalStrength(void);

/*
 * @brief Sends special AT command to Module and evaluates the answer to get Battery Charge and voltage on Module
 * @retval
 *
 */
uint8_t GSMGetBatteryCharge(uint16_t * voltagepntr);

/*
 * @brief: Lets the GSM Module send a simple SMS
 * @param:  const char * targetnr - mobile nr to send sms to (example +ZZxxxxxxxxxx)
 * 			const char * message - Textmessage to send as sms - limited to  100 Chars (99, because of \0 termination)
 * @retval  1: OK, SMS Sent // 101: unexpected wrong data received (Probably Error) //
 * 			102: Timeout, no response - not done//
 * 			103: Message too long
 *
 */
uint8_t GSMSendSMS(const char * targetnr, const char * message);

/*
 * @brief Sends special AT command to Module and evaluates the answer to get Prepaid Balance over USSD code
 * @param: uint16_t * balanceptr -> balance in cents gets written there if valid value from gsm module
 * @retval  //1: Ok, probalby a valid value in referenced string // 101: unexpected wrong data received (?) // 102: Timeout, no response
 */
uint8_t GSMGetBalance(uint16_t * balanceptr);

/* USER CODE END EFP */

#ifdef __cplusplus
}
#endif

#endif /* __USART_GSM_H__ */
