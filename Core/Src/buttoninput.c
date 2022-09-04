/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    button_reed_input.c
  * @brief   This file provides code for different button input logic using interrupt and timer
  * including trigger detection - Wake-Up in stopmode2 (low power mode without system clock)
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

/*defines*/
#define BUTTONREAD_TIME_MS	200


/*external variables*/
extern uint8_t commandflag;
extern uint8_t buttonpressdone;


/*private variables*/
/* Timers objects*/
static UTIL_TIMER_Object_t timerButton;
uint16_t buttontimesctr = 0;

/*private function prototypes*/
static void onButtonTimmerEvent(void *context);

//Overwrite weak callback
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  /* When user button Interrupt came */
  if (GPIO_Pin == BUT_BOOT_Pin) {
	  commandflag = 0;
	  //Create and start timer of 200ms
	  UTIL_TIMER_Create(&timerButton, BUTTONREAD_TIME_MS, UTIL_TIMER_ONESHOT, onButtonTimmerEvent, NULL);

	  UTIL_TIMER_Start(&timerButton);
  }
}


static void onButtonTimmerEvent(void *context) {
	buttontimesctr++;
	if (HAL_GPIO_ReadPin(BUT_BOOT_GPIO_Port, BUT_BOOT_Pin) == 0){
		//if button still pressed then start timer again
		UTIL_TIMER_Start(&timerButton);

	}
	else {
		//if button not pressed anymore, evaluate how often 200ms have passed and return corresponding command
		if (buttontimesctr < 10) {
			//if shorter than 2seconds - commandflag is 1
			commandflag = 1;
			buttonpressdone = 1;
			buttontimesctr = 0;
		}
		else if (buttontimesctr >= 10 && buttontimesctr < 25 ) {
			//if button pressed between 2 and 5 seconds, commandflag is 2
			commandflag = 2;
			buttontimesctr = 0;
		}
		else if (buttontimesctr >= 25 && buttontimesctr < 50){
			//if button is pressed longer than 5 seconds, commandflag is 3
			commandflag = 3;
			buttontimesctr = 0;
		}
		else if (buttontimesctr >= 50){
			//if button is pressed longer than 10 seconds, commandflag is 4
			 commandflag = 4;
			 buttontimesctr = 0;
		}
	}
}

/* USER CODE END 0 */
