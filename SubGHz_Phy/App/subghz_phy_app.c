/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    subghz_phy_app.c
  * @author  MCD Application Team
  * @brief   Application of the SubGHz_Phy Middleware
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
  * Changed by R.Woellmer with specific LoRa Application
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "platform.h"
#include "sys_app.h"
#include "subghz_phy_app.h"
#include "radio.h"

/* USER CODE BEGIN Includes */
#include "usart_gsm.h"
#include "stm32_seq.h"
#include "utilities_def.h"
#include "stm32_timer.h"
#include "usart.h"
#include "stdio.h"
#include "string.h"
#include "battery_read.h"
#include "btcommandread.h"
#include "flashparam.h"
#include <ctype.h>
#include <math.h>

/* USER CODE END Includes */

/* External variables ---------------------------------------------------------*/
/* USER CODE BEGIN EV */
/*Set the UART Handle you're using for this communication from uart.h" */
extern UART_HandleTypeDef hlpuart1;
extern DMA_HandleTypeDef hdma_lpuart1_rx;

/* USER CODE END EV */

/* Private typedef -----------------------------------------------------------*/

/* USER CODE BEGIN PTD */
typedef enum
{
  NONE,
  RX,
  RX_TIMEOUT,
  RX_ERROR,
  TX,
  TX_TIMEOUT,
} States_t;

typedef enum
{
  NON,
  ACK,
  ERROR_BEFORE_SLEEP,
  NOT_REACHABLE,
  BAD_CONDITION,
} Slavestate;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/*Buffer for TX and RX is selected LoRa PAYLOAD Length in Bytes*/
#define MAX_APP_BUFFER_SIZE		PAYLOAD_LEN

#define RXBUF_LEN				20

#define _BT_PRINT(TXT)	HAL_UART_Transmit(&hlpuart1, (uint8_t *)TXT, strlen(TXT), 300);

#define MAXSLAVES 				5

/* LED blink Period Config Idle*/
#define LEDConfIdle_PERIOD_MS                 500 //1Hz
/* GSM Startup time */
#define GSM_Startuptime_MS					15000 //15s for registration


/* Battery Readout with voltage divider & adc */
#define R8 45800 //Spannungsteiler unten gg Ground
#define R9 15000 //Spannungsteiler oben gg V3.3

/*LoRa*/
#define TX_TIMEOUT_VALUE              3000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* Radio events function pointer */
static RadioEvents_t RadioEvents;
/* USER CODE BEGIN PV */

Parametersatztype Parameter = { "00000000000000", {0}, 1, {0}, {0}  };
uint64_t speicherhandynr1 = 0;
uint64_t speicherhandynr2 = 0;
char handynr_set[15] = {0};
uint64_t wakeup_set = 0;

/* Timers objects*/
static UTIL_TIMER_Object_t timerLedIdleConf;
static UTIL_TIMER_Object_t timerGSMStartConf;
static UTIL_TIMER_Object_t timerSleepWakeup;
static UTIL_TIMER_Object_t timerLoraSync;
static UTIL_TIMER_Object_t timerLedFeedback;

/*CommandFlag used to determine what to do in config mode*/
uint8_t commandflag = 0;
uint8_t buttonpressdone = 0;


/*GSM Startup time in config mode over flag*/
uint8_t gsm_configmode_timeover = 0;
/*GSM Test OK*/
uint8_t gsmtest_OK = 0;

/*Battery Readout*/
float Akkuspannung;
int Akku_Prozent;
uint8_t battery_OK = 0;

/*Fallenstatus*/
uint8_t traptriggered = 0;

/*For bluetooth print*/
char printstr[100] = {0};

/*Time for cyclic wakeup in mins. */
uint64_t mins_passed = 0;

/*LoraNumber of Slaves registered*/
uint8_t slaves_reg = 0;
/*Array of registered Slave addresses*/
uint8_t slaveaddrss[MAXSLAVES] = {0};
/*Array of status of registered Slaves*/
Slavestate slavestatus[MAXSLAVES] = {0};
/*Array of battery state of registered Slaves*/
uint8_t slavebatterystate[MAXSLAVES] = {0};
/*Array of triggerstate of registered Slaves*/
uint8_t slavetriggerstate[MAXSLAVES] = {0};

/*To save, if certain event has already occured
 * -> no new SMS has to be sent, if already done during last wakeup
 */
uint8_t slavebatterybad_before[MAXSLAVES];
uint8_t slavetriggered_before[MAXSLAVES];
uint8_t slaveunreachable_before[MAXSLAVES];
uint8_t mastertriggered_before;
uint8_t masterbatterybad_before;

/*LoRaSynctime over flag*/
uint8_t syncover = 0;

/*LoRa Variables*/
/*Actual State */
static States_t State = NONE;
/* App Tx Buffer*/
static uint8_t BufferTx[MAX_APP_BUFFER_SIZE];
/* Last  Received Buffer Size*/
uint16_t RxBufferSize = 0;
/* Last  Received packer Rssi*/
int8_t RssiValue = 0;
/* Last  Received packer SNR (in Lora modulation)*/
int8_t SnrValue = 0;
/*Receive Buffer*/
uint8_t RxBuffer[RXBUF_LEN] = {0};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/*!
 * @brief Function to be executed on Radio Tx Done event
 */
static void OnTxDone(void);

/**
  * @brief Function to be executed on Radio Rx Done event
  * @param  payload ptr of buffer received
  * @param  size buffer size
  * @param  rssi
  * @param  LoraSnr_FskCfo
  */
static void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t LoraSnr_FskCfo);

/**
  * @brief Function executed on Radio Tx Timeout event
  */
static void OnTxTimeout(void);

/**
  * @brief Function executed on Radio Rx Timeout event
  */
static void OnRxTimeout(void);

/**
  * @brief Function executed on Radio Rx Error event
  */
static void OnRxError(void);

/* USER CODE BEGIN PFP */
/*Timer callbacks*/
static void OnledConfIdleEvent(void *context);
static void OnledFeedbackEventGood(void *context);
static void OnGSMConfTimeEvent(void *context);
static void timerSleepWakeupEvent(void *context);
static void OnSyncOverEvent(void *context);

/*Main Processes for Sequencer*/
static void ConfigModeIdle(void);
static void SyncedWakeup(void);

/*Other Subfunctions*/
static void GSMTest(void);
static void BatteryReadout(void);
static void LoRa_FindSync(void);
static uint8_t LoRa_StartSleep(void);

/*Helper Functions*/
static void indicateLED_return(uint8_t good_bad);
static uint8_t alreadyRegistered(uint8_t slaveid);
int digits_only(const char *s);
/* USER CODE END PFP */

/* Exported functions ---------------------------------------------------------*/
void SubghzApp_Init(void)
{
  /* USER CODE BEGIN SubghzApp_Init_1 */

	//Read Params from Flash
  Parameter = Readflash_func();

  /* USER CODE END SubghzApp_Init_1 */

  /* Radio initialization */
  RadioEvents.TxDone = OnTxDone;
  RadioEvents.RxDone = OnRxDone;
  RadioEvents.TxTimeout = OnTxTimeout;
  RadioEvents.RxTimeout = OnRxTimeout;
  RadioEvents.RxError = OnRxError;

  Radio.Init(&RadioEvents);

  /* USER CODE BEGIN SubghzApp_Init_2 */

  /* Radio Set frequency */
   Radio.SetChannel(RF_FREQUENCY);

   /* Radio configuration */
 #if ((USE_MODEM_LORA == 1) && (USE_MODEM_FSK == 0))
   APP_LOG(TS_OFF, VLEVEL_M, "---------------\n\r");
   APP_LOG(TS_OFF, VLEVEL_M, "LORA_MODULATION\n\r");
   APP_LOG(TS_OFF, VLEVEL_M, "LORA_BW=%d kHz\n\r", (1 << LORA_BANDWIDTH) * 125);
   APP_LOG(TS_OFF, VLEVEL_M, "LORA_SF=%d\n\r", LORA_SPREADING_FACTOR);

   Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                     LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                     LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                     true, 0, 0, LORA_IQ_INVERSION_ON, TX_TIMEOUT_VALUE);

   Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                     LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                     LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                     0, true, 0, 0, LORA_IQ_INVERSION_ON, true);

   Radio.SetMaxPayloadLength(MODEM_LORA, MAX_APP_BUFFER_SIZE);

 #elif ((USE_MODEM_LORA == 0) && (USE_MODEM_FSK == 1))
   APP_LOG(TS_OFF, VLEVEL_M, "---------------\n\r");
   APP_LOG(TS_OFF, VLEVEL_M, "FSK_MODULATION\n\r");
   APP_LOG(TS_OFF, VLEVEL_M, "FSK_BW=%d Hz\n\r", FSK_BANDWIDTH);
   APP_LOG(TS_OFF, VLEVEL_M, "FSK_DR=%d bits/s\n\r", FSK_DATARATE);

   Radio.SetTxConfig(MODEM_FSK, TX_OUTPUT_POWER, FSK_FDEV, 0,
                     FSK_DATARATE, 0,
                     FSK_PREAMBLE_LENGTH, FSK_FIX_LENGTH_PAYLOAD_ON,
                     true, 0, 0, 0, TX_TIMEOUT_VALUE);

   Radio.SetRxConfig(MODEM_FSK, FSK_BANDWIDTH, FSK_DATARATE,
                     0, FSK_AFC_BANDWIDTH, FSK_PREAMBLE_LENGTH,
                     0, FSK_FIX_LENGTH_PAYLOAD_ON, 0, true,
                     0, 0, false, true);

   Radio.SetMaxPayloadLength(MODEM_FSK, MAX_APP_BUFFER_SIZE);

 #else
 #error "Please define a modulation in the subghz_phy_app.h file."
 #endif /* USE_MODEM_LORA | USE_MODEM_FSK */

   //send Radio to sleep in the beginning
   Radio.Sleep();

  /*UART task registration*/
  UTIL_SEQ_RegTask((1 << CFG_SEQ_Task_ConfigMode_Idle), UTIL_SEQ_RFU, ConfigModeIdle);
  UTIL_SEQ_RegTask((1 << CFG_SEQ_Task_SyncedWakeup), UTIL_SEQ_RFU, SyncedWakeup);
  UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_ConfigMode_Idle), CFG_SEQ_Prio_0);
  /* USER CODE END SubghzApp_Init_2 */
}

/* USER CODE BEGIN EF */

/* USER CODE END EF */

/* Private functions ---------------------------------------------------------*/
static void OnTxDone(void)
{
  /* USER CODE BEGIN OnTxDone */
	State = TX;
  /* USER CODE END OnTxDone */
}

static void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t LoraSnr_FskCfo)
{
  /* USER CODE BEGIN OnRxDone */
	//_BT_PRINT("MSG Received\r");
	/*Only Messages with the correct identifier are relevant*/
	if(payload[0] != 0xFA)
		return;

	/*If size is not correct, its also not a correct message for master*/
	if (size != PAYLOAD_LEN)
		return;

	/*Eventuell noch MAster ID einführen, dass nachrichten an anderen Master auch verworfen werden können!!!!*/
	State = RX;
	RxBufferSize = size;
	/*Only Copy if Received Data is less than Buffer availible*/
	if (RxBufferSize < RXBUF_LEN){
		memcpy(RxBuffer, payload, size);
	}
	RssiValue = rssi;
	SnrValue = LoraSnr_FskCfo;
  /* USER CODE END OnRxDone */
}

static void OnTxTimeout(void)
{
  /* USER CODE BEGIN OnTxTimeout */
	State = TX_TIMEOUT;
  /* USER CODE END OnTxTimeout */
}

static void OnRxTimeout(void)
{
  /* USER CODE BEGIN OnRxTimeout */
	State = RX_TIMEOUT;
  /* USER CODE END OnRxTimeout */
}

static void OnRxError(void)
{
  /* USER CODE BEGIN OnRxError */
	State = RX_ERROR;
  /* USER CODE END OnRxError */
}

/* USER CODE BEGIN PrFD */

static void ConfigModeIdle(void){


	//Disable Reed Contact Interrupt in Config Mode (in System-Init all IRQs are enabled)
	HAL_NVIC_DisableIRQ(EXTI3_IRQn);
	//LED Red Blink 1 Hz start
	UTIL_TIMER_Create(&timerLedIdleConf, LEDConfIdle_PERIOD_MS, UTIL_TIMER_ONESHOT, OnledConfIdleEvent, NULL);
	UTIL_TIMER_Start(&timerLedIdleConf);
	_BT_PRINT("Master Trap in Config Mode...\r");
	//In Config Mode the GSM and Bluetooth module - power transistor is enabled
	HAL_GPIO_WritePin(GSM_Switch_GPIO_Port, GSM_Switch_Pin, 1);
	//Timer to give GSM Module time to register
	UTIL_TIMER_Create(&timerGSMStartConf, GSM_Startuptime_MS, UTIL_TIMER_ONESHOT, OnGSMConfTimeEvent, NULL);
	UTIL_TIMER_Start(&timerGSMStartConf);

	//Start RX of BT
	HAL_UART_AbortReceive(&hlpuart1);
	SetUpTXRXCycle_BTUART();


	//Wait for commandflag (either buttonpress or BT uart from App)
	//variable "commandflag" as int - defines which command should be initiated and is written by command interrupt
	while (1) {
		//button only pressed quickly -> Perform GSM Test
		if (commandflag == 1){
			HAL_UART_AbortReceive(&hlpuart1);
			commandflag = 0;
			/*Stop Idle LED Blink*/
			UTIL_TIMER_Stop(&timerLedIdleConf);
			HAL_GPIO_WritePin(LED_Red_GPIO_Port, LED_Red_Pin, 0);
			/*Test GSM Functions*/
			GSMTest();
			buttonpressdone = 0;
			/*Clear Pending Interrupts in case button was pressed while GSM Process and enable IRQs again*/
			__HAL_GPIO_EXTI_CLEAR_IT(BUT_BOOT_Pin);
			HAL_NVIC_ClearPendingIRQ(EXTI15_10_IRQn);
			HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
			SetUpTXRXCycle_BTUART();
		}
		//button pressed between 2 and 5 seconds -> Read out Battery voltage
		if (commandflag == 2){
			HAL_UART_AbortReceive(&hlpuart1);
			commandflag = 0;
			/*Stop Idle LED Blink*/
			UTIL_TIMER_Stop(&timerLedIdleConf);
			HAL_GPIO_WritePin(LED_Red_GPIO_Port, LED_Red_Pin, 0);
			/*Battery Readout*/
			BatteryReadout();
			/*Clear Pending Interrupts in case button was pressed while GSM Process and enable IRQs again*/
			__HAL_GPIO_EXTI_CLEAR_IT(BUT_BOOT_Pin);
			HAL_NVIC_ClearPendingIRQ(EXTI15_10_IRQn);
			HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
			SetUpTXRXCycle_BTUART();
		}
		//button over 5s pressed: search and sync slave devices over LoRa
		if (commandflag == 3){
			HAL_UART_AbortReceive(&hlpuart1);
			commandflag = 0;
			/*Stop Idle LED Blink*/
			UTIL_TIMER_Stop(&timerLedIdleConf);
			HAL_GPIO_WritePin(LED_Red_GPIO_Port, LED_Red_Pin, 0);
			/*Find, sync and register LoRa Slaves with their ID*/
			LoRa_FindSync();
			/*Clear Pending Interrupts in case button was pressed while GSM Process and enable IRQs again*/
			__HAL_GPIO_EXTI_CLEAR_IT(BUT_BOOT_Pin);
			HAL_NVIC_ClearPendingIRQ(EXTI15_10_IRQn);
			HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
			SetUpTXRXCycle_BTUART();

		}
		//button over 10s pressed: start LoRa
		if (commandflag == 4){
			HAL_UART_AbortReceive(&hlpuart1);
			uint8_t readyforsleep_lora = 0;
			uint8_t validnr = 0;
			commandflag = 0;
			/*Stop Idle LED Blink*/
			UTIL_TIMER_Stop(&timerLedIdleConf);
			HAL_GPIO_WritePin(LED_Red_GPIO_Port, LED_Red_Pin, 0);
			_BT_PRINT("Initializing Sleep for trap mode...\r");

			/*Check if Valid nr*/
			if (strncmp(Parameter.Handynummer, "+", 1) == 0 && digits_only(Parameter.Handynummer+1) == 1){
				validnr = 1;
			}
			else {
				validnr = 0;
			}

			/*Sleep Mode activation over LoRa, if slaves are used*/
			if (slaves_reg != 0){
				readyforsleep_lora = LoRa_StartSleep();
			}
			else {
				readyforsleep_lora = 1;
			}


			if (readyforsleep_lora == 0){
				sprintf(printstr, "Error! Following slave devices did not acknowledge sleep mode: ");
				for (uint8_t i = 0; i < slaves_reg; i++){
					if (slavestatus[i] == ERROR_BEFORE_SLEEP){	// 8=missing ack, 1 = ok,
						char slaveid_str[5] = {0};
						sprintf(slaveid_str, "%u", slaveaddrss[i]);
						strcat(printstr, slaveid_str);
					}
				}
				strcat(printstr, "\r");
				_BT_PRINT(printstr);
				_BT_PRINT("Initiation of sleep mode canceled.\r");
				indicateLED_return(0);
				__HAL_GPIO_EXTI_CLEAR_IT(BUT_BOOT_Pin);
				HAL_NVIC_ClearPendingIRQ(EXTI15_10_IRQn);
				HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
				SetUpTXRXCycle_BTUART();
			}
			else if (validnr == 0){
				_BT_PRINT("No valid Mobile-Nr to send SMS to was set. Please specifiy a Number starting with +49..\r");
				_BT_PRINT("Initiation of sleep mode canceled.\r");
				indicateLED_return(0);
				__HAL_GPIO_EXTI_CLEAR_IT(BUT_BOOT_Pin);
				HAL_NVIC_ClearPendingIRQ(EXTI15_10_IRQn);
				HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
				SetUpTXRXCycle_BTUART();
			}
			else{
				if (slaves_reg != 0){
					_BT_PRINT("All slaves acknowledged, starting power-down trap mode now... Bluetooth connection will break\r");
				}
				else {
					_BT_PRINT("Starting power-down trap mode with no slaves now...Bluetooth connection will break\r");
				}
				HAL_Delay(200);

				/*Stop UART BT Reception*/
				HAL_UART_AbortReceive(&hlpuart1);

				/*Shut down GSM and Bluetooth module*/
				HAL_GPIO_WritePin(GSM_Switch_GPIO_Port, GSM_Switch_Pin, 0);
				UTIL_TIMER_Create(&timerSleepWakeup, 60000, UTIL_TIMER_ONESHOT, timerSleepWakeupEvent, NULL); //every min, mins++
				UTIL_TIMER_Start(&timerSleepWakeup);
				return;
			}

		}

		if (commandflag == 5){
			HAL_UART_AbortReceive(&hlpuart1);
			commandflag = 0;
			/*Stop Idle LED Blink*/
			UTIL_TIMER_Stop(&timerLedIdleConf);
			HAL_GPIO_WritePin(LED_Red_GPIO_Port, LED_Red_Pin, 0);
			_BT_PRINT("Storing new number...\r");
			//Check if valid nr
			if (strncmp(handynr_set, "+", 1) == 0 && digits_only(handynr_set+1) == 1){
				memcpy(Parameter.Handynummer, handynr_set, 15);

				for (int i = 0; i <= 15; i++) {
					if (i <= 8)
						Parameter.Handynummer1[i] = Parameter.Handynummer[i];
					else {
						for (int f = (i - 8); (i - f) == 8; f++) {

							Parameter.Handynummer2[f] = Parameter.Handynummer[i];
						}
					}
				}

				memcpy(&speicherhandynr1, Parameter.Handynummer1, 8);
				memcpy(&speicherhandynr2, Parameter.Handynummer2, 8);

				Eraseflash_func();
				Writeflash_func(speicherhandynr1, speicherhandynr2, Parameter.Wakeup);
				_BT_PRINT("Successfully stored ")
				_BT_PRINT(handynr_set);
				_BT_PRINT(" as the number to send SMS to...\r");
				indicateLED_return(1);
			}
			else {
				_BT_PRINT("No valid Mobile-Nr to send SMS to was set. Please specifiy a Number starting with +49..\r");
				indicateLED_return(0);
			}

			/*Clear Pending Interrupts in case button was pressed while Process and enable IRQs again*/
			__HAL_GPIO_EXTI_CLEAR_IT(BUT_BOOT_Pin);
			HAL_NVIC_ClearPendingIRQ(EXTI15_10_IRQn);
			HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
			SetUpTXRXCycle_BTUART();

		}
		if (commandflag == 6){
			HAL_UART_AbortReceive(&hlpuart1);
			commandflag = 0;
			/*Stop Idle LED Blink*/
			UTIL_TIMER_Stop(&timerLedIdleConf);
			HAL_GPIO_WritePin(LED_Red_GPIO_Port, LED_Red_Pin, 0);
			_BT_PRINT("Storing new wakeup time interval...\r");
			//Check if valid nr
			if (wakeup_set > 0){
				Parameter.Wakeup = wakeup_set;
				memcpy(&speicherhandynr1, Parameter.Handynummer1, 8);
				memcpy(&speicherhandynr2, Parameter.Handynummer2, 8);

				Eraseflash_func();
				Writeflash_func(speicherhandynr1, speicherhandynr2, Parameter.Wakeup);
				_BT_PRINT("Successfully stored ");
				sprintf(printstr, "%lu" , (uint32_t)wakeup_set);
				_BT_PRINT(printstr);
				_BT_PRINT(" minutes as wakeup time interval...\r");
				indicateLED_return(1);
			}
			else {
				_BT_PRINT("No valid wakeup time interval in minutes was set, please specify a number over 0..\r");
				indicateLED_return(0);
			}

			/*Clear Pending Interrupts in case button was pressed while Process and enable IRQs again*/
			__HAL_GPIO_EXTI_CLEAR_IT(BUT_BOOT_Pin);
			HAL_NVIC_ClearPendingIRQ(EXTI15_10_IRQn);
			HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
			SetUpTXRXCycle_BTUART();
		}

	}
}

static void SyncedWakeup(void){
	HAL_GPIO_WritePin(LED_Board_GPIO_Port, LED_Board_Pin, 0);
	uint8_t i = 0;
	uint8_t message_neccessary = 0;

	/*Get Slaves status*/
	/*fills tx buffer*/
	memset(BufferTx, 0x0, PAYLOAD_LEN);
	BufferTx[0] = 0xFA; //Identifier, Message belongs to Trap devices
	BufferTx[1] = 0x84; //Function -> Message from Master, Poll Slaves state in trapmode
	/*Data*/
	BufferTx[3] = 0x00; //
	BufferTx[4] = 0x00;
	BufferTx[5] = 0x00;
	BufferTx[6] = 0x00;
	BufferTx[7] = 0x00;

	for (i = 0; i<slaves_reg; i++){
		/*Send to slaves*/
		BufferTx[2] = slaveaddrss[i]<<4; //SlaveID
		State = NONE;
		slavestatus[i] = NON;
		Radio.Send(BufferTx, PAYLOAD_LEN);
		/*Wait until LoRa module has done something*/
		while (State != TX && State != TX_TIMEOUT);
		if (State == TX_TIMEOUT){
			 Radio.Sleep();
			 slavestatus[i] = NOT_REACHABLE;
		}
		/*Start Reception with timeout*/
		State = NONE;
		Radio.Rx(6000);
		/*Wait until something happens*/
		while(State != RX && State != RX_ERROR && State != RX_TIMEOUT);
		if (State == RX_ERROR || State == RX_TIMEOUT){
			slavestatus[i] = NOT_REACHABLE;
		}
		else if (State == RX){
			/* check if slave with current address answered*/
			uint8_t currentslaveid = RxBuffer[2] >> 4;
			if (currentslaveid == slaveaddrss[i]){
				/* if its the correct slave, check data (battery, trap trigger state) */
				slavebatterystate[i] = RxBuffer[3];
				slavetriggerstate[i] = RxBuffer[4];
				if (slavebatterystate[i] == 255 || slavebatterystate[i] <= 20 || slavetriggerstate[i] == 1){
					slavestatus[i] = BAD_CONDITION;
				} else {
					slavestatus[i] = ACK; //fine in trap mode
				}
			}else {
				slavestatus[i] = NOT_REACHABLE;
			}
		}
	}
	State = NONE;
	Radio.Sleep();
	/*Get Slave Status end*/

	/*Get own status*/
	traptriggered = HAL_GPIO_ReadPin(REEDPin_GPIO_Port, REEDPin_Pin);
	Akku_Prozent = Ausgabe_Akkustand(Berechnung_Akkustand(R9, R8));

	/*Send SMS if:
	 * -one or more of the slaves or master has triggered
	 * -one or more of the slaves or master has low power or faulty battery reading
	 * -one or more of the slaves is not reachable
	 * (Slave conditions are already checked in code above)
	 * BUT ONLY SEND, IF THOSE EVENTS OCCUR THE FIST TIME!
	 */
	for(i = 0; i< slaves_reg; i++)
	{
		if(slavestatus[i] == NOT_REACHABLE && slaveunreachable_before[i] == 0){
			slaveunreachable_before[i] = 1;
			message_neccessary = 1;
		} else if (slavestatus[i] != NOT_REACHABLE && slavestatus[i] != NON) {
			slaveunreachable_before[i] = 0;  //if slave somehow manages to participate with correct time sync again, it can be evaluated in the next wakeup again
		}
		if((slavebatterystate[i] == 255 || slavebatterystate[i] <= 20) && slavebatterybad_before[i] == 0){
			slavebatterybad_before[i] = 1;
			message_neccessary = 1;
		} else if ((slavebatterystate[i] != 255 && slavebatterystate[i] > 20)){
			slavebatterybad_before[i] = 0;  //if slave somehow manages to participate with correct time sync and enough battery again, it can be evaluated in the next wakeup again
		}
		if(slavetriggerstate[i] == 1 && slavetriggered_before[i] == 0){
			slavetriggered_before[i] = 1;
			message_neccessary = 1;
		} else if (slavetriggerstate[i] == 0){
			slavetriggered_before[i] = 0;   //if slave somehow gets its trigger reset and is still on time sync, it can be evaulated in the next wakeup again
		}
	}


	if ((Akku_Prozent <= 20 || Akku_Prozent == 255) && masterbatterybad_before == 0){
		masterbatterybad_before = 1;
		message_neccessary = 1;
	} else if ((Akku_Prozent > 20 && Akku_Prozent != 255)) {
		masterbatterybad_before = 0;
	}
	if (traptriggered && mastertriggered_before == 0){
		mastertriggered_before = 1;
		message_neccessary = 1;
	} else if (traptriggered == 0) {
		mastertriggered_before = 0;  // if trigger gets reset while trap is still on time sync, it can be evaluated again
	}

	if (message_neccessary){
		//Code for sending an SMS
		HAL_GPIO_WritePin(GSM_Switch_GPIO_Port, GSM_Switch_Pin, 1);

		/*!!!! Eventuell UART GSM Neuinit nötig nach Wakeup!*/

		/*String bauen für SMS*/
		char message[320] = {0}; //one SMS can only transmit 160 chars, max 2 sms allowed
		char strtemp[32];
		sprintf(message, "Event on Trap(s):\nMaster: ");
		sprintf(strtemp, "Bat: %d%%, Trig: %u\n", Akku_Prozent, traptriggered);
		strcat(message, strtemp);
		for (i = 0; i<slaves_reg; i++){
			if(slavestatus[i] == NOT_REACHABLE){
				sprintf(strtemp, "SlaveID: %u: not reachable\n", slaveaddrss[i]);
			}else if (slavebatterystate[i] == 255){
				sprintf(strtemp, "SlaveID: %u: Bat: <0%%, Trig: %u\n", slaveaddrss[i], slavetriggerstate[i]);
			}else{
				sprintf(strtemp, "SlaveID: %u: Bat: <%d%%, Trig: %u\n", slaveaddrss[i], slavebatterystate[i], slavetriggerstate[i]);
			}
			strcat(message, strtemp);
		}

		/*Send SMS*/
		/*Workaround, UART2 may not wake up after Stopmode2, if not reinitialized with expicit call of the MspInit part */
		HAL_UART_MspInit(&huart2);
		MX_USART2_UART_Init();
		HAL_Delay(20000); //Give GSM Module time to wake up

		strcat(message, "\0");
		int statussms = GSMSendSMS(Parameter.Handynummer, message);  //GSMSendSMS("+4916094870875", "Hallo123TestKot");

		if (statussms == 101){
			HAL_GPIO_WritePin(LED_Red_GPIO_Port, LED_Red_Pin, 1);
			HAL_Delay(1000);
			HAL_GPIO_WritePin(LED_Red_GPIO_Port, LED_Red_Pin, 0);
		}
		if (statussms == 102){
					HAL_GPIO_WritePin(LED_Green_GPIO_Port, LED_Green_Pin, 1);
					HAL_Delay(1000);
					HAL_GPIO_WritePin(LED_Green_GPIO_Port, LED_Green_Pin, 0);
				}

		/*Simulation: BT-Message statt SMS*/
		//strcat(message, "\r");
		//_BT_PRINT(message);


		/*Power down again*/
		HAL_GPIO_WritePin(GSM_Switch_GPIO_Port, GSM_Switch_Pin, 0);
	}
	HAL_GPIO_WritePin(LED_Board_GPIO_Port, LED_Board_Pin, 1);

}

static void BatteryReadout(void){
	/*Read out Voltage and Percent*/
	_BT_PRINT("Reading Battery Voltage...\r");
	Akkuspannung = Berechnung_Akkustand(R9, R8);
	Akku_Prozent = Ausgabe_Akkustand(Akkuspannung);
	sprintf(printstr, "Battery-Voltage: %.4fV\r", Akkuspannung);
	_BT_PRINT(printstr);
	HAL_Delay(300);
	if (Akku_Prozent == 255){
		_BT_PRINT("Extreme low battery\r");
		battery_OK = 0;
		indicateLED_return(0);
	}
	else if (Akku_Prozent == 20) {
		sprintf(printstr, "That's less than %d %%, please charge\r", Akku_Prozent);
		_BT_PRINT(printstr);
		battery_OK = 0;
		indicateLED_return(0);
	}
	else {
		sprintf(printstr, "That's around %d %%\n\r", Akku_Prozent);
		_BT_PRINT(printstr);
		battery_OK = 1;
		indicateLED_return(1);
	}
}

static void GSMTest(void){

	/*Variables*/
	uint8_t statusAT = 255;
	uint8_t statusSIMCard = 255;
	uint8_t statusReg = 255;
	uint8_t signalRSSI = 255;
	uint8_t statusGSMBattery = 255;
	uint16_t statusGSMBatteryVOLT = 255;
	//static uint8_t statusGSMSendSMS = 255;

	/*Workaround, UART2 may not wake up after Stopmode2, if not reinitialized with expicit call of the MspInit part */
	HAL_UART_MspInit(&huart2);
	MX_USART2_UART_Init();
	//if (buttonpressdone)
		//_BT_PRINT("GSMButtonpress!\r");
	_BT_PRINT("GSM-Test started. Please wait...\r");
	gsmtest_OK = 0;

	if (strncmp(Parameter.Handynummer, "+", 1) == 0 && digits_only(Parameter.Handynummer+1) == 1){
		_BT_PRINT("Stored Mobile-Nr to send SMS to: ");
		_BT_PRINT(Parameter.Handynummer);
		_BT_PRINT("\r");
	}
	else{
		_BT_PRINT("No valid Mobile-Nr to send SMS to was set. Please specifiy a Number starting with +49..\r");
	}

	/*GSM Module must be turned on for at least 15s, because it needs time to register to cellular network*/
	while (gsm_configmode_timeover == 0);
	GSMSimpleAT(); //first command is for autobauding of sim module, response is often bullshit
	HAL_Delay(500);
	GSMSimpleAT();
	HAL_Delay(500);
	/*Test simple AT answer*/
	statusAT = GSMSimpleAT();
	if (statusAT != 1){
		_BT_PRINT("Error: No valid answer from GSM Module. GSM-Test failed...\r");
		gsmtest_OK = 0;
		indicateLED_return(0);
		return;
	}
	HAL_Delay(500);

	/*Check SIM-Card Insertion*/
	statusSIMCard = GSMCheckSimInsert();
	if (statusSIMCard == 0){
		_BT_PRINT("Error: No Sim Card inserted. GSM-Test failed...\r");
		gsmtest_OK = 0;
		indicateLED_return(0);
		return;
	}
	else if (statusSIMCard != 1){
		_BT_PRINT("Error: No valid answer from GSM Module. GSM-Test failed...\r");
		gsmtest_OK = 0;
		indicateLED_return(0);
		return;
	}
	HAL_Delay(500);

	/*Check Signal Strength*/
	signalRSSI = GSMGetSignalStrength();
	if (signalRSSI >= 99){
			_BT_PRINT("Error: No valid answer from GSM Module. GSM-Test failed...\r");
			gsmtest_OK = 0;
			indicateLED_return(0);
			return;
	}
	else if (signalRSSI == 0){
			_BT_PRINT("GSM signal strength: very bad - consider another location\r");
	}
	else if (signalRSSI == 1){
			_BT_PRINT("GSM signal strength: low but enough\r");
	}
	else if (signalRSSI >= 2 && signalRSSI < 31){
			_BT_PRINT("GSM signal strength: OK\r");
	}
	else if (signalRSSI == 31){
			_BT_PRINT("GSM signal strength: Very Good\r");
	}
	HAL_Delay(500);

	/*Check Network Registration*/
	statusReg = GSMGetRegState();
	if (statusReg == 0 || statusReg == 2 || statusReg == 3 || statusReg == 4){
		_BT_PRINT("Error: GSM Module not registered in cellular network. GSM-Test failed...\r");
		gsmtest_OK = 0;
		indicateLED_return(0);
		return;
	}
	else if (statusReg != 1){
		_BT_PRINT("Error: No valid answer from GSM Module. GSM-Test failed...\r");
		gsmtest_OK = 0;
		indicateLED_return(0);
		return;
	}
	else if (statusReg == 1){
		_BT_PRINT("GSM Module successfully registered to GSM Network\r");
	}
	HAL_Delay(500);


	//
	uint16_t balance_cent = 0;
	if (GSMGetBalance(&balance_cent) < 101){
		sprintf(printstr, "Balance: %u,%u EUR\r", balance_cent / 100, balance_cent % 100);
		_BT_PRINT(printstr);
	}
	else{
		_BT_PRINT("Prepaid balance could not be requested\r");
	}


	/*Short Battery-Test from GSM Module*/
	statusGSMBattery = GSMGetBatteryCharge(&statusGSMBatteryVOLT);
	if (statusGSMBattery < 101){
		sprintf(printstr, "Battery measurement of GSM-Module: %u%%\r", statusGSMBattery);
		_BT_PRINT(printstr);
	}

	/*If code proceeds until here without return, GSM Check was successful*/
	gsmtest_OK = 1;
	_BT_PRINT("-> GSM-Test successful\r");
	/*Indicate on Green LED and then start Config mode idle LED again*/
	indicateLED_return(1);

}

static void LoRa_FindSync(void){

	  /*Delete already registered slaves*/
	  slaves_reg = 0;
	  memset(slaveaddrss, 0x0, MAXSLAVES);

	  _BT_PRINT("Searching LoRa Slave devices, please wait...\r")
	  /*fills tx buffer*/
	  memset(BufferTx, 0x0, PAYLOAD_LEN);
	  BufferTx[0] = 0xFA; //Identifier, Message belongs to Trap devices
	  BufferTx[1] = 0x81; //Function -> Message from Master, Try to sync and register slaves
	  BufferTx[2] = 0x00; //Slave IDs unknown so far
	  /*Data*/

	  uint32_t tempWakeup = (uint32_t)Parameter.Wakeup; //cast is ok because nr is limited by app
	  memcpy(BufferTx+3, &tempWakeup, 4);
	  BufferTx[7] = 0x00;

	  /*Send Broadcast*/
	  State = NONE;
	  Radio.Send(BufferTx, PAYLOAD_LEN);

	  /*Wait until LoRa module has done someting*/
	  while (State != TX && State != TX_TIMEOUT);

	  if (State == TX_TIMEOUT){
		  _BT_PRINT("LoRa TX Error, syncing process stopped\r");
		  Radio.Sleep();
		  indicateLED_return(0);
		  return;
	  }

	  /*Start Reception*/
	  Radio.Rx(0);
	  /*Start timer for possible answers.
	   * Slaves answer multiple times with a random generated delay each,
	   * so each active slave in range has a chance to get registered by the master
	   * and none gets ignored accidently, if another one answers at the same time.
	   * On Air time with SF=12, BW=125KHz, 8 Bit Payload is ca. 1s -> Reception window should be active several seconds.
	   */
	  UTIL_TIMER_Create(&timerLoraSync, 20000, UTIL_TIMER_ONESHOT, OnSyncOverEvent, NULL);
	  UTIL_TIMER_Start(&timerLoraSync);
	  syncover = false;
	  /*Wait for Slaves to answer*/
	  State = NONE;
	  while(!syncover) {
		  if (State == RX_ERROR || State == RX_TIMEOUT){
			  _BT_PRINT("LoRa message reception error. Continuing to receive... \r");
			  /*Reset State for next receive*/
			  State = NONE;
			  Radio.Rx(0);
		  }
		  if (State == RX){
			  /* save new slave address(=ID)*/
			  uint8_t currentslaveid = RxBuffer[2] >> 4;
			  if (alreadyRegistered(currentslaveid) == 0 && slaves_reg<= MAXSLAVES){
				  slaveaddrss[slaves_reg++] = currentslaveid;
				  sprintf(printstr, "New slave device registered with ID: %u, RSSI: %ddBm, SNR: %ddB and Battery: %u%%\r", currentslaveid, RssiValue, SnrValue, RxBuffer[3]);
				  _BT_PRINT(printstr);
			  }
			  /*Reset State for next receive*/
			  State = NONE;
			  Radio.Rx(0);
		  }
	  }
	  Radio.Sleep();

	  if (slaves_reg == 0){
		  _BT_PRINT("Registration process completed: No slave devices found.\r");
	  }
	  else{
		  sprintf(printstr, "Registration process completed: Successfully registered %u slave devices.\r", slaves_reg);
		  _BT_PRINT(printstr);
	  }
	  indicateLED_return(1);

}

static uint8_t LoRa_StartSleep(void){
	 /*fills tx buffer*/
	  memset(BufferTx, 0x0, PAYLOAD_LEN);
	  BufferTx[0] = 0xFA; //Identifier, Message belongs to Trap devices
	  BufferTx[1] = 0x88; //Function -> Message from Master, Send Slaves to sleep
	  /*Data*/
	  uint32_t tempWakeup = (uint32_t)Parameter.Wakeup; //cast is ok because nr is limited by app
	  memcpy(BufferTx+3, &tempWakeup, 4);
	  BufferTx[7] = 0x00;

	  for (uint8_t i = 0; i<slaves_reg; i++){
		  /*Send to slaves*/
		  BufferTx[2] = slaveaddrss[i]<<4; //SlaveID
		  State = NONE;
		  slavestatus[i] = NON;
		  Radio.Send(BufferTx, PAYLOAD_LEN);
		  /*Wait until LoRa module has done something*/
		  while (State != TX && State != TX_TIMEOUT);
		  if (State == TX_TIMEOUT){
			  _BT_PRINT("LoRa TX Error\r");
			  Radio.Sleep();
			  slavestatus[i] = ERROR_BEFORE_SLEEP;
			  return 0;
		  }
		  /*Start Reception with timeout*/
		  State = NONE;
		  Radio.Rx(6000);
		  /*Wait until something happens*/
		  while(State != RX && State != RX_ERROR && State != RX_TIMEOUT);
		  if (State == RX_ERROR || State == RX_TIMEOUT){
			  slavestatus[i] = ERROR_BEFORE_SLEEP;
		  }
		  if (State == RX){
			  /* check if slave with current address answered with acknowledge*/
			  uint8_t currentslaveid = RxBuffer[2] >> 4;
			  uint8_t ackcommand = RxBuffer[1] & 0x0F;
			  if (currentslaveid == slaveaddrss[i] && ackcommand == 0x02){
				  slavestatus[i] = ACK;
			  }else {
				  slavestatus[i] = ERROR_BEFORE_SLEEP;
			  }
		  }
	  }
	  State = NONE;
	  Radio.Sleep();

	  for (uint8_t i = 0; i < slaves_reg; i++){
		  if (slavestatus[i] == ERROR_BEFORE_SLEEP)
			 return 0;
	  }
	  return 1;
}

static uint8_t alreadyRegistered(uint8_t slaveid){
	uint8_t i;

	for (i = 0; i<MAXSLAVES; i++ ){
		if (slaveaddrss[i] == slaveid)
			return 1;
	}
	return 0;
}

/* param: 1: good, 0: bad*/
static void indicateLED_return(uint8_t good_bad){
	if (good_bad == 1) {
		HAL_GPIO_WritePin(LED_Green_GPIO_Port, LED_Green_Pin, 1);
		UTIL_TIMER_Create(&timerLedFeedback, 500, UTIL_TIMER_ONESHOT, OnledFeedbackEventGood, NULL);
		UTIL_TIMER_Start(&timerLedFeedback);

	}
	/*Start config mode LED Indication again*/
	HAL_GPIO_TogglePin(LED_Red_GPIO_Port, LED_Red_Pin); /* LED_Red*/
	UTIL_TIMER_Start(&timerLedIdleConf);
}

/*Timer callbacks*/
static void OnSyncOverEvent(void *context){
  syncover = true;
}

static void OnledConfIdleEvent(void *context){
  HAL_GPIO_TogglePin(LED_Red_GPIO_Port, LED_Red_Pin); /* LED_Red*/
  UTIL_TIMER_Start(&timerLedIdleConf);
}

static void OnGSMConfTimeEvent(void *context){
  gsm_configmode_timeover = true;
}

static void OnledFeedbackEventGood(void *context){
	HAL_GPIO_WritePin(LED_Green_GPIO_Port, LED_Green_Pin, 0);
}


static void timerSleepWakeupEvent(void *context){
	mins_passed++;
	/*Start timer right again, same as in slaves.
	*because this way wakeup time interval will stay
	*the same most likely, independent from different amout of code
	*done during waked up process  */
	UTIL_TIMER_Start(&timerSleepWakeup);
	if (mins_passed >= Parameter.Wakeup){
		mins_passed = 0;
		UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_SyncedWakeup), CFG_SEQ_Prio_0);
	}
}


/*Helper*/
int digits_only(const char *s)
{
    while (*s) {
        if (isdigit((unsigned char)*s++) == 0) return 0;
    }

    return 1;
}


/* USER CODE END PrFD */
