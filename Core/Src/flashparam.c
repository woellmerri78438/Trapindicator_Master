/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    flashparam.c
  * @brief   This file provides code for storing and reading data in the flash
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
#include "flashparam.h"


/*defines*/
#define FLASH_USER_START_ADDR   ADDR_FLASH_PAGE_96   /* Start @ of user Flash area */
#define FLASH_USER_END_ADDR     (ADDR_FLASH_PAGE_127 + FLASH_PAGE_SIZE - 1)   /* End @ of user Flash area */

/*private variables*/
uint32_t FirstPage = 0, NbOfPages = 0;
uint32_t Address = 0, PageError = 0;
__IO uint32_t MemoryProgramStatus = 0;

/*Variable used for Erase procedure*/
static FLASH_EraseInitTypeDef EraseInitStruct;

/*exported FP*/
static uint32_t GetPage(uint32_t Address);



/*PFDs*/
void Eraseflash_func(void) {
	/* Erase the user Flash area (area defined by FLASH_USER_START_ADDR and FLASH_USER_END_ADDR) ***********/
	HAL_FLASH_Unlock();
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR);
	FirstPage = GetPage(FLASH_USER_START_ADDR);

	/* Get the number of pages to erase from 1st page */
	NbOfPages = GetPage(FLASH_USER_END_ADDR) - FirstPage + 1;

	EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
	EraseInitStruct.Page = FirstPage;
	EraseInitStruct.NbPages = NbOfPages;

	if (HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) != HAL_OK) {
		while (1) {}
		HAL_FLASH_Lock();
	}
}


/* Die Übergebene Struktur "Parameter" wird in den Speicher geschrieben, dafür muss die größe der char Arrays beachtet werden, da der
 * Handynummer Array größer als ein doubleword ist, wird dieses auf zwei Arrays aufgeteilt. Die typkonvertierung findet mit memcpy statt.*/

void Writeflash_func(uint64_t Handynr1, uint64_t Handynr2, uint64_t wakeupmins) {

	HAL_FLASH_Unlock();
	Address = FLASH_USER_START_ADDR;

	int err = 0;
	err = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, Address, Handynr1);
	if (err == HAL_OK) {
		Address = Address + 8; /* increment to next double word*/
	} else {
		while (1) {}
	}

	if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, Address, Handynr2) == HAL_OK) {
		Address = Address + 8; /* increment to next double word*/
	} else {
		while (1) {}
	}

	if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, Address, wakeupmins) == HAL_OK) {
		Address = Address + 8; /* increment to next double word*/
	} else {
		while (1) {}
	}

	/* Lock the Flash to disable the flash control register access (recommended
	 to protect the FLASH memory against possible unwanted operation) *********/
	HAL_FLASH_Lock();

}


/* Die Funktion schreibt die im Flash abgelegten Daten in eine Struktur "Auslesen" und gibt diese zurück. Da beim schreiben des Flashs
 * die char Arrays als doubleword geschrieben werden, muss noch eine Typkonvertierung zurück in char stattfinden. Diese wird mittels memcpy erledigt.
 * Das Array der Handynummer wird am Ende aus den zwei gelesenen doublewords zusammengefügt.*/

Parametersatztype Readflash_func(void) {

	Parametersatztype Auslesen = {0};

	Address = FLASH_USER_START_ADDR;
	MemoryProgramStatus = 0x0;
	uint64_t speicher;

	if (Address < FLASH_USER_END_ADDR) {
		speicher = *(__IO uint64_t*) Address;
		Address = Address + 8;
	}
	memcpy(Auslesen.Handynummer1, &speicher, sizeof(char) * 8);

	if (Address < FLASH_USER_END_ADDR) {
		speicher = *(__IO uint64_t*) Address;
		Address = Address + 8;
	}
	memcpy(Auslesen.Handynummer2, &speicher, sizeof(char) * 8);

	if (Address < FLASH_USER_END_ADDR) {
		speicher = *(__IO uint64_t*) Address;
		Address = Address + 8;
	}
	memcpy(&Auslesen.Wakeup, &speicher, sizeof(uint64_t));

	for (int i = 0; i <= 15; i++) {
		if (i <= 7)
			Auslesen.Handynummer[i] = Auslesen.Handynummer1[i];
		else {
			for (int f = (i - 8); (i - f) == 8; f++) {
				Auslesen.Handynummer[i] = Auslesen.Handynummer2[f];

			}

		}
	}

	return Auslesen;

}



/**
 * @brief  Gets the page of a given address
 * @param  Addr: Address of the FLASH Memory
 * @retval The page of a given address
 */
static uint32_t GetPage(uint32_t Addr) {
	return (Addr - FLASH_BASE) / FLASH_PAGE_SIZE;;
}



/* USER CODE END 0 */
