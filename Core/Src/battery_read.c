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
#include "battery_read.h"
#include "adc.h"


/*defines*/

#define VMAX 3.3
#define BITMAX 4095

/*external variables*/
/*ADC */
extern ADC_HandleTypeDef hadc;

/*Private Variables*/


/*Function Prototypes*/
/* Berechnung des Akkustands
 * Einstellen der Parameter (R1,R2)
 * Übergabe: /
 * Rückgabe: berechneter Akkustand
 */
float Berechnung_Akkustand(uint32_t R1, uint32_t R2) {

	//Wegen Wakeup neuinit nötig!
	MX_ADC_Init();
	uint32_t adc0;
	float Vin;
	float Vakku;

	HAL_ADC_Start(&hadc);

	if (HAL_ADC_PollForConversion(&hadc, 1000) == HAL_OK) //ADC Conversion True
			{
		adc0 = HAL_ADC_GetValue(&hadc); ///Wert der Conversion in adc0 speichern
		Vin = (((float)adc0 / BITMAX) * VMAX);
		Vakku = Vin * (((float)R2 + (float)R1) / (float)R2); //Vin Berechnen (Spg.teiler mit einbezogen)
	}

	return Vakku;
}

/* Ausgabe des Prozentwerts und der Spannung über UART
 * Übergabe: berechneter Akkustand
 * Rückgabe: berechneter Prozentwert
 */
int Ausgabe_Akkustand(float Vakku) {
	int Prozentwert;

	if (Vakku >= 4.1) {
		Prozentwert = 100; //100%
	} else if ((Vakku >= 4.0) && (Vakku <= 4.1)) {
		Prozentwert = 90;		//90
	} else if ((Vakku >= 3.9) && (Vakku <= 4.0)) {
		Prozentwert = 80;		//80%
	} else if ((Vakku >= 3.8) && (Vakku <= 3.9)) {
		Prozentwert = 60;		//60%
	} else if ((Vakku >= 3.7) && (Vakku <= 3.8)) {
		Prozentwert = 40;		//40%
	} else if ((Vakku >= 3.6) && (Vakku <= 3.7)) {
		Prozentwert = 20;		//20%
	} else if ((Vakku <= 3.6) && (Vakku > 3)) {
		Prozentwert = 0;		//0%
	} else
		Prozentwert = 255;
	return (Prozentwert);
}

/* USER CODE END 0 */
