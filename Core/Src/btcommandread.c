/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    btcommandread.c
  * @brief   This file provides code for reading the UART commands from BT-Module, sent by the app
  ******************************************************************************
  * @attention
  *
  * Richard Woellmer, TH Nuernberg
  *
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* USER CODE BEGIN 0 */
/* Includes ------------------------------------------------------------------*/
#include "stm32_timer.h"
#include "main.h"
#include "btcommandread.h"
#include "flashparam.h"
#include "math.h"

extern char handynr_set;
extern uint64_t wakeup_set;
char * mobilenr_set  = &handynr_set;

/*external variables*/
extern uint8_t commandflag;
/*external variables*/
/*Set the UART Handle you're using for this communication from uart.h" */
extern UART_HandleTypeDef hlpuart1;
extern DMA_HandleTypeDef hdma_lpuart1_rx;

/*Flags */
static uint8_t flag_timeout = 0;
static uint8_t flag_RX_Done = 0;



/*Buffer*/
#define RxBuf_SIZE		100
#define MainBuf_SIZE	200

static uint8_t RxBuf[RxBuf_SIZE];
static uint8_t MainBuf[MainBuf_SIZE];

static uint16_t oldPos = 0;
static uint16_t newPos = 0;
static uint16_t rxSize = 0;

/*FPT*/
static void memcpy_char_dword(uint64_t * targetword, uint8_t * chararray);

/*FDEF*/


void BT_UART_RXCallback(UART_HandleTypeDef *huart, uint16_t Size){
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

	/*Abort reception while received data is evaluated*/
	HAL_UART_AbortReceive(&hlpuart1);


	/*Evaluate received data*/
	if (strncmp((char*)MainBuf, "GSMTest", 7) == 0){
		commandflag = 1;
	}
	else if (strncmp((char*)MainBuf, "Batterytest", 11) == 0){
		commandflag = 2;
	}
	else if (strncmp((char*)MainBuf, "LoRaDevices", 11) == 0){
		commandflag = 3;
	}
	else if (strncmp((char*)MainBuf, "Startsleep", 10) == 0){
		commandflag = 4;
	}
	else if (strncmp((char*)MainBuf, "NewNr: ", 6) == 0){
		commandflag = 5;
		memcpy(mobilenr_set, MainBuf+sizeof("NewNr: ")-1, 14);
		strncat(mobilenr_set, "\0", 1);
	}
	else if (strncmp((char*)MainBuf, "NewWakeup: ", 10) == 0){
		commandflag = 6;
		memcpy_char_dword(&wakeup_set, MainBuf+sizeof("NewWakeup: ")-1);
	}
	else {
		SetUpTXRXCycle_BTUART();
	}

	//if none of that is detected, dont change commandflag -> no action


	/*If more Data is expected: */
	//HAL_UARTEx_ReceiveToIdle_DMA(huart_gsm, RxBuf, RxBuf_SIZE);
	//__HAL_DMA_DISABLE_IT(hdma_uart_rx_gsm, DMA_IT_HT);


}

void SetUpTXRXCycle_BTUART(void){
	/*clear Buffer and flags*/
	memset(RxBuf, 0, RxBuf_SIZE);
	memset(MainBuf, 0, MainBuf_SIZE);
	oldPos = 0;
	newPos = 0;
	flag_RX_Done = 0;
	rxSize = 0;
	flag_timeout = 0;

	/*Start DMA to be prepared for RX*/
	HAL_UARTEx_ReceiveToIdle_DMA(&hlpuart1, RxBuf, RxBuf_SIZE);
	__HAL_DMA_DISABLE_IT(&hdma_lpuart1_rx, DMA_IT_HT);
}


/*
 * Memcopy for char array of 8 to doubleword -> for example '0''0''0''1''2''5''5''5'' => 12555
 */
static void memcpy_char_dword(uint64_t * targetword, uint8_t * chararray){
	*targetword = 0;
	for (int i = 7; i >= 0; i--){
		*targetword += (chararray[i]-48) * (uint64_t)pow(10, (double)(8-(i+1)));
	}
}

/* USER CODE END 0 */
