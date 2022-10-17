/*
 *  Buzzer.c
 *
 *  Created on: Sep 16, 2022
 *      Author: Emanuele Somma
 */

#include "Buzzer_Int.h"

/*
 * @brief Set Brake (On or Off)
 * @param Pointer to Motor Brake structure
 * @param Brake state to apply
 */
void Buzzer_PlaySong(TIM_HandleTypeDef *htim, uint32_t uint32_TimChannel)
{
	HAL_TIM_PWM_Start(htim, uint32_TimChannel);

	uint32_t bpm = 136;
	int i;
	uint32_t duration;
	int len;
	int note;
	uint32_t Clk = 21000000;

	len = sizeof(_values)/sizeof(uint32_t);

	for (i=0; i<len; i++) {
		htim->Instance->CCR1=0;
		HAL_Delay(3);
		note = (Clk/(_notes[i]))-1;
		if (note == 0)
		{
			htim->Instance->CCR1=note/2;
			htim->Instance->ARR=note;
		}
		else
		{
			htim->Instance->CCR1=note/2;
			htim->Instance->ARR=note;
		}
		duration = ((64/_values[i])*(60000/bpm));

		HAL_Delay(duration/16);
	}
}
