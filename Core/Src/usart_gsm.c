/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart_gsm.c
  * @brief   This file provides code for UART-Communication to GSM Module Sim800L
  ******************************************************************************
  * @attention
  *
  * Richard Woellmer, TH Nuernberg
  *
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN 0 */
#include "usart_gsm.h"
#include <stdio.h>
#include "string.h"
#include "stm32_timer.h"
#include "stdlib.h"

//test only
#include "sys_app.h"

/*external variables*/
/*Set the UART Handle you're using for this communication from uart.h" */
extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart2_rx;


UART_HandleTypeDef * huart_gsm = &huart2;
DMA_HandleTypeDef * hdma_uart_rx_gsm = &hdma_usart2_rx;

/*Timeout*/
#define Receive_TIMEOUT_VALUE              1500
static UTIL_TIMER_Object_t timerTimeout;
static void OnRxTimeout(void *context);


/*Flags */
static uint8_t flag_timeout = 0;
static uint8_t flag_RX_Done = 0;

static void SetUpTXRXCycle(void);

/*Buffer*/
#define RxBuf_SIZE		100
#define MainBuf_SIZE	200

static uint8_t RxBuf[RxBuf_SIZE];
static uint8_t MainBuf[MainBuf_SIZE];

static uint16_t oldPos = 0;
static uint16_t newPos = 0;
static uint16_t rxSize = 0;
static uint8_t offst = 0;

void GSM_UART_RXCallback(UART_HandleTypeDef *huart, uint16_t Size){
	oldPos = newPos;

	if (oldPos+Size > MainBuf_SIZE) {
		uint16_t datatocopy = MainBuf_SIZE-oldPos;
		memcpy ((uint8_t *)MainBuf+oldPos, RxBuf, datatocopy);
		oldPos = 0;
		memcpy ((uint8_t *)MainBuf, (uint8_t *)RxBuf+datatocopy, (Size-datatocopy));
		newPos = (Size-datatocopy);
		}
	else {
		memcpy ((uint8_t *)MainBuf+oldPos, RxBuf, Size);
		newPos = Size+oldPos;
	}

	rxSize = Size;
	flag_RX_Done = 1;
	UTIL_TIMER_Stop(&timerTimeout);
	HAL_UART_AbortReceive(huart_gsm);


	/*If more Data is expected: */
	//HAL_UARTEx_ReceiveToIdle_DMA(huart_gsm, RxBuf, RxBuf_SIZE);
	//__HAL_DMA_DISABLE_IT(hdma_uart_rx_gsm, DMA_IT_HT);

}



/*
 * @brief Sends a simple "AT" command to Module, Module should respond with OK. Also necessary for auto bauding function of the module.
 * @retval 1: Ok // 102: Timeout, no response  // 101: unexpected wrong data received (?)
 *
 */
uint8_t GSMSimpleAT(void){

	SetUpTXRXCycle();

	/*Transmit AT Command in Blocking Mode*/
	HAL_UART_Transmit(huart_gsm, (uint8_t *)"AT\r", sizeof("AT\r"), 300);

	/*Wait until Timeout flag or Data Received flag */
	while (flag_timeout == 0 && flag_RX_Done == 0) {}

	if (flag_timeout) {
		return 102;
	}
	else{
		/* check if expected Data
		 * Module response: (echo of command) + \0 + \r + \n + (RESPONSE STRING acc. to AT-Command-Set) + \r +\n
		 * */
		offst = sizeof("AT\r") + 2; //get offset index

		if (strncmp((char*)MainBuf+offst, "OK", 2) == 0){
			return 1;
		}
		else {
			return 101;
		}
	}
}

/*
 * @brief Sends special AT command to Module and evaluates the answer to get Simcard insertion status
 * @retval 1: Sim Inserted // 0: Sim Not inserted // 101: unexpected wrong data received (?) // 102: Timeout, no response
 *
 */
uint8_t GSMCheckSimInsert(void){

	SetUpTXRXCycle();

	/*Transmit AT Command in Blocking Mode*/
	HAL_UART_Transmit(huart_gsm, (uint8_t *)"AT+CSMINS?\r", sizeof("AT+CSMINS?\r"), 300);

	/*Wait until Timeout flag or Data Received flag */
	while (flag_timeout == 0 && flag_RX_Done == 0) {}

	if (flag_timeout) {
			return 102;
	}
	else{
		/* check if expected Data
		 * Module response: (echo of command) + \0 + \r + \n + (RESPONSE STRING acc. to AT-Command-Set) + \r +\n
		 * */
		offst = sizeof("AT+CSMINS?\r") + 2; //get offset index

		if (strncmp((char*)MainBuf+offst, "+CSMINS", 7) == 0){

			offst = offst + 11; //go to sim inserted value

			return (uint8_t)MainBuf[offst] - 48; //convert ASCII 0 or 1 to uint 0 or 1
		}
		else {
			return 101;
		}
	}
}

/*
 * @brief Sends special AT command to Module and evaluates the answer to get Network registration status
 * @retval 1: Registered, home network // 0: Not registered, MT is not currently searching //
 *  // 2: Not registered, but MT is currently searching // 3: Registration denied // 4: unknown
 *  // 5: Registered with Roaming
 *  // 101: unexpected wrong data received (?) // 102: Timeout, no response
 *
 */
uint8_t GSMGetRegState(void){

	SetUpTXRXCycle();

	/*Transmit AT Command in Blocking Mode*/
	HAL_UART_Transmit(huart_gsm, (uint8_t *)"AT+CREG?\r", sizeof("AT+CREG?\r"), 300);

	/*Wait until Timeout flag or Data Received flag */
	while (flag_timeout == 0 && flag_RX_Done == 0) {}

	if (flag_timeout) {
			return 102;
	}
	else{
		/* check if expected Data
		 * Module response: (echo of command) + \0 + \r + \n + (RESPONSE STRING acc. to AT-Command-Set) + \r +\n
		 * */
		offst = sizeof("AT+CREG?\r") + 2; //get offset index

		if (strncmp((char*)MainBuf+offst, "+CREG", 5) == 0){

			offst = offst + 9; //go to net reg value

			return (uint8_t)MainBuf[offst] - 48; //convert ASCII to uint
		}
		else {
			return 101;
		}
	}
}

/*
 * @brief Sends special AT command to Module and evaluates the answer to get GSM Signal strength
 * @retval [RSSI] 0: -115 dBm or less // 1:  -111 dBm // 2...30:  -110... -54 dBm //
 * 31: -52 dBm or greater // 99:  not known or not detectable //
 *  // 101: unexpected wrong data received (?) // 102: Timeout, no response
 *
 */
uint8_t GSMGetSignalStrength(void){

	SetUpTXRXCycle();

	/*Transmit AT Command in Blocking Mode*/
	HAL_UART_Transmit(huart_gsm, (uint8_t *)"AT+CSQ\r", sizeof("AT+CSQ\r"), 300);

	/*Wait until Timeout flag or Data Received flag */
	while (flag_timeout == 0 && flag_RX_Done == 0) {}

	if (flag_timeout) {
			return 102;
	}
	else{
		/* check if expected Data
		 * Module response: (echo of command) + \0 + \r + \n + (RESPONSE STRING acc. to AT-Command-Set) + \r +\n
		 * */
		offst = sizeof("AT+CSQ\r") + 2; //get offset index

		if (strncmp((char*)MainBuf+offst, "+CSQ", 4) == 0){

			offst = offst + 6; //go to RSSI value
			return (uint8_t)strtol((char*)MainBuf+offst, NULL, 10);

		}
		else {
			return 101;
		}
	}
}


/*
 * @brief Sends special AT command to Module and evaluates the answer to get Battery Charge and voltage on Module
 * @param: uint16_t * voltagepntr - Module can write its current supply voltage to this address,
 * 									Format: 4 digit usint, for example 3758 means 3,758 Volts supply
 * @retval 1 - 100: Battery Charge in Percent // 101: unexpected wrong data received (?) // 102: Timeout, no response
 *
 */
uint8_t GSMGetBatteryCharge(uint16_t * voltagepntr){

	SetUpTXRXCycle();

	/*Transmit AT Command in Blocking Mode*/
	HAL_UART_Transmit(huart_gsm, (uint8_t *)"AT+CBC\r", sizeof("AT+CBC\r"), 300);

	/*Wait until Timeout flag or Data Received flag */
	while (flag_timeout == 0 && flag_RX_Done == 0) {}

	if (flag_timeout) {
			return 102;
	}
	else{
		/* check if expected Data
		 * Module response: (echo of command) + \0 + \r + \n + (RESPONSE STRING acc. to AT-Command-Set) + \r +\n
		 * */
		offst = sizeof("AT+CBC\r") + 2; //get offset index

		if (strncmp((char*)MainBuf+offst, "+CBC", 4) == 0){
			offst = offst + 8; //go to percent value
			uint8_t percentage = (uint8_t)strtol((char*)MainBuf+offst, NULL, 10);

			if (voltagepntr != NULL) {
				(percentage/10 >= 1) ? (offst += 3) : (offst += 2);
				*voltagepntr = (uint16_t)strtol((char*)MainBuf+offst, NULL, 10);
			}
			return percentage;
		}
		else {
			return 101;
		}
	}
}

/*
 * @brief: Lets the GSM Module send a simple SMS
 * @param:  const char * targetnr - mobile nr to send sms to (example +ZZxxxxxxxxxx)
 * 			const char * message - Textmessage to send as sms - limited to  160 Chars (159, because of \0 termination)
 * @retval  1: OK, SMS Sent // 101: unexpected wrong data received (Probably Error) //
 * 			102: Timeout, no response - not done//
 * 			103: Message too long
 *
 */
uint8_t GSMSendSMS(const char * targetnr, const char * message){

	char sendtext[SMS_MSG_LEN];
	static uint8_t endmsg_nr = 26;

	if (strlen(message)+1 > SMS_MSG_LEN){ // strlen skips the \0 terminator, but its necessary
		return 103;
	}



	/*1 .Transmit AT Command for SMS Text Mode in Blocking Mode*/
	SetUpTXRXCycle();
	HAL_UART_Transmit(huart_gsm, (uint8_t *)"AT+CMGF=1\r", sizeof("AT+CMGF=1\r"), 300);

	/*1.1 Wait until Timeout flag or Data Received flag and verify Answer */
	while (flag_timeout == 0 && flag_RX_Done == 0) {}

	if (flag_timeout) {
			return 102;
	}
	offst = sizeof("AT+CMGF=1\r") + 2; //get offset index
	if (strncmp((char*)MainBuf+offst, "OK", 2) != 0  && strncmp((char*)MainBuf+offst+1, "OK", 2) != 0 ){
		return 101; //Error, probably no SIM Card or not registered to GSM Network
	}

	/*2. Transmit AT Switch to Textmode Command for SMS */
	HAL_Delay(300);
	SetUpTXRXCycle();

	strcpy(sendtext, "AT+CMGS=\"");
	strcat(sendtext, targetnr);
	strcat(sendtext, "\"\r");

	HAL_UART_Transmit(huart_gsm, (uint8_t *)sendtext, strlen(sendtext), 300);
	while (flag_timeout == 0 && flag_RX_Done == 0) {}

	/*2.1 After Echo is received send text to send */
	HAL_Delay(300);
	SetUpTXRXCycle();

	memset(sendtext, 0, SMS_MSG_LEN);
	strcpy(sendtext, message);

	HAL_UART_Transmit(huart_gsm, (uint8_t *)sendtext, strlen(sendtext), 300);
	while (flag_timeout == 0 && flag_RX_Done == 0) {}


	/*2.2 After Echo is received send END Character to terminate SMS */
	HAL_Delay(300);

	/*SetUp Custom TX RX Cycle because of higher Timeout*/
	memset(RxBuf, 0, RxBuf_SIZE);
	memset(MainBuf, 0, MainBuf_SIZE);
	oldPos = 0;
	newPos = 0;
	flag_RX_Done = 0;
	rxSize = 0;
	flag_timeout = 0;
	/*Start DMA to be prepared for RX*/
	HAL_UARTEx_ReceiveToIdle_DMA(huart_gsm, RxBuf, RxBuf_SIZE);
	__HAL_DMA_DISABLE_IT(hdma_uart_rx_gsm, DMA_IT_HT);
	/*Create and Start Timer for Timeout Watch*/
	UTIL_TIMER_Create(&timerTimeout, 6000, UTIL_TIMER_ONESHOT, OnRxTimeout, NULL);
	UTIL_TIMER_Start(&timerTimeout);
	/*Custom Setup TXRX Cycle End*/

	HAL_UART_Transmit(huart_gsm, &endmsg_nr, 1 , 300);

	/*2.3. Module should Acknowledge the Sending,
	 * Wait until Timeout flag or Data Received flag and verify Answer*/
	while (flag_timeout == 0 && flag_RX_Done == 0) {}

	if (flag_timeout) {
			return 102;
	}
	if (strstr((char*)MainBuf, "OK") == NULL){
			return 101; //Error, Unexpected Answer, no acknoledge from board
	}

	return 1; //OK

}

/*
 * @brief Sends special AT command to Module and evaluates the answer to get Prepaid Balance over USSD code
 * @param: uint16_t * balanceptr -> balance in cents gets written there if valid value from gsm module
 * @retval  //1: Ok, valid value in referenced argument // 101: unexpected wrong data received (?) // 102: Timeout, no response
 */
uint8_t GSMGetBalance(uint16_t * balanceptr){
	char * targetptr = NULL;
	char * endptr = NULL;
	uint16_t euro = 0;
	uint16_t cent = 0;
	uint8_t abstnd = 0;
	char digitstring[10] = {0};

	SetUpTXRXCycle();

	/*Transmit AT Command in Blocking Mode*/
	HAL_UART_Transmit(huart_gsm, (uint8_t *)"AT+CUSD=1, \"*100#\"\r", sizeof("AT+CUSD=1, \"*100#\"\r"), 300);

	/*Wait until Timeout flag or Data Received flag */
	while (flag_timeout == 0 && flag_RX_Done == 0) {}

	if (flag_timeout) {
			return 102;
	}

	/*Start Custom RX with longer timeout because USSD Code takes some time*/
	memset(RxBuf, 0, RxBuf_SIZE);
	memset(MainBuf, 0, MainBuf_SIZE);
	oldPos = 0;
	newPos = 0;
	flag_RX_Done = 0;
	rxSize = 0;
	flag_timeout = 0;

	/*Start DMA to be prepared for RX*/
	HAL_UARTEx_ReceiveToIdle_DMA(huart_gsm, RxBuf, RxBuf_SIZE);
	__HAL_DMA_DISABLE_IT(hdma_uart_rx_gsm, DMA_IT_HT);

	/*Create and Start Timer for Timeout Watch*/
	UTIL_TIMER_Create(&timerTimeout, 6000, UTIL_TIMER_ONESHOT, OnRxTimeout, NULL);
	UTIL_TIMER_Start(&timerTimeout);

	/*Wait until Timeout flag or Data Received flag */
	while (flag_timeout == 0 && flag_RX_Done == 0) {}

	if (flag_timeout) {
			return 102;
	}

	/* check if expected Data
	 * There should be the Substring "Aktuelles Prepaid Guthaben: " followed by the balance */
	targetptr = strstr((char*)MainBuf, "Aktuelles Prepaid Guthaben: ");
	if (targetptr != NULL){
		offst = sizeof("Aktuelles Prepaid Guthaben: ")-1;
		/*EUR*/
		targetptr = targetptr + offst;//go to Balance value
		endptr = strstr(targetptr, ",");
		if (endptr == NULL){
			return 101;
		}
		abstnd = endptr - targetptr;
		memcpy(digitstring, targetptr, abstnd);
		digitstring[abstnd+1] = '\0';
		euro = (uint16_t)strtol(digitstring, NULL, 10);
		/*Cent*/
		targetptr = endptr+1;
		memcpy(digitstring, targetptr, 2);
		digitstring[3] = '\0';
		cent = (uint16_t)strtol(digitstring, NULL, 10);

		if (balanceptr != NULL)
		{
			*balanceptr = euro*100 + cent;
		}

	}
	else {
		return 101;
	}

	SetUpTXRXCycle();
	/*Transmit AT Command in Blocking Mode*/
	HAL_UART_Transmit(huart_gsm, (uint8_t *)"AT+CUSD=0\r", sizeof("AT+CUSD=0\r"), 300);
	while (flag_timeout == 0 && flag_RX_Done == 0) {}
	return 1;
}

static void OnRxTimeout(void *context) {
	flag_timeout = 1;
	UTIL_TIMER_Stop(&timerTimeout);
	HAL_UART_AbortReceive(huart_gsm);
}

static void SetUpTXRXCycle(void){
	/*clear Buffer and flags*/
	memset(RxBuf, 0, RxBuf_SIZE);
	memset(MainBuf, 0, MainBuf_SIZE);
	oldPos = 0;
	newPos = 0;
	flag_RX_Done = 0;
	rxSize = 0;
	flag_timeout = 0;

	/*Start DMA to be prepared for RX*/
	HAL_UARTEx_ReceiveToIdle_DMA(huart_gsm, RxBuf, RxBuf_SIZE);
	__HAL_DMA_DISABLE_IT(hdma_uart_rx_gsm, DMA_IT_HT);

	/*Create and Start Timer for Timeout Watch*/
	UTIL_TIMER_Create(&timerTimeout, Receive_TIMEOUT_VALUE, UTIL_TIMER_ONESHOT, OnRxTimeout, NULL);
	UTIL_TIMER_Start(&timerTimeout);
}
/* USER CODE END 0 */
