/*
 * Display_Ext.h
 *
 *  Created on: Oct 11, 2022
 *      Author: emanu
 */

#ifndef INC_DISPLAY_EXT_H_
#define INC_DISPLAY_EXT_H_

//typedef enum
//{
//	Display_PidCoeff_Kp = 0,
//	Display_PidCoeff_Ki = 1,
//	Display_PidCoeff_Kd = 2,
//	Display_PidCoeff_Ks = 3,
//	Display_PidCoeff_Invalid
//}Display_PidCoeff_te;
//
//typedef enum
//{
//	Display_ControllerX = 0,
//	Display_ControllerY = 1,
//	Display_ControllerInvalid
//}Display_Controller_te;

typedef enum
{
	Display_Element_FirstControllerValue = 0,
	Display_Element_1 = Display_Element_FirstControllerValue,
	Display_Element_2 = 1,
	Display_Element_LastControllerValue = Display_Element_2,

	Display_Element_FirstPidCoeffValue = 2,
	Display_Element_Kp = Display_Element_FirstPidCoeffValue,
	Display_Element_Ki = 3,
	Display_Element_Kd = 4,
	Display_Element_Ks = 5,
	Display_Element_LastPidCoeffValue = Display_Element_Ks,

	Display_Element_Invalid
}Display_Element_te;

void Display_UpdateScreen(/*Controller_t *pControllerX, Controller_t *pControllerY*/);
void Display_SelectCoeff(Display_Element_te Enum_PidCoeff);
void Display_SelectController(Display_Element_te Enum_Controller);
Display_Element_te Display_GetSelectedController();
Display_Element_te Display_GetSelectedCoeff();

#endif /* INC_DISPLAY_EXT_H_ */
