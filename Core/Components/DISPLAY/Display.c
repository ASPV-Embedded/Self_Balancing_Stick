/*
 * Display.c
 *
 *  Created on: Oct 11, 2022
 *      Author: emanu
 */

#include "Display_int.h"

void Display_DrawOutline()
{
	  ssd1306_Fill(Black);

	  ssd1306_Line(0, 17, 127, 17, White);
	  ssd1306_Line(0, 63, 127, 63, White);
	  ssd1306_Line(0, 40, 127, 40, White);

	  ssd1306_Line(0, 17, 0, 63, White);
	  ssd1306_Line(127, 17, 127, 63, White);
	  ssd1306_Line(63, 17, 63, 63, White);
//	  ssd1306_Line(64, 17, 64, 63, White);

	  ssd1306_SetCursor(2, 2);
	  ssd1306_WriteString("Pid Gains", Font_7x10, White);
	  ssd1306_SetCursor(103, 2);
	  ssd1306_WriteString("1 2", Font_7x10, White);

	  ssd1306_SetCursor(3, 24);
	  ssd1306_WriteString("Kp=", Font_7x10, White);

	  ssd1306_SetCursor(3, 48);
	  ssd1306_WriteString("Kd=", Font_7x10, White);

	  ssd1306_SetCursor(66, 24);
	  ssd1306_WriteString("Ki=", Font_7x10, White);

	  ssd1306_SetCursor(66, 48);
	  ssd1306_WriteString("Ks=", Font_7x10, White);
}

void Display_WritePidGains(float float_Kp, float float_Ki, float float_Kd, float float_Ks)
{
	char str[5] = {0};
	sprintf(str, "%.1f", float_Kp);
	ssd1306_SetCursor(24, 24);
	ssd1306_WriteString(str, Font_7x10, White);

	memset(str, 0x0, sizeof(str));
	sprintf(str, "%.1f", float_Ki);
	ssd1306_SetCursor(87, 24);
	ssd1306_WriteString(str, Font_7x10, White);

	memset(str, 0x0, sizeof(str));
	sprintf(str, "%.1f", float_Kd);
	ssd1306_SetCursor(24, 48);
	ssd1306_WriteString(str, Font_7x10, White);

	memset(str, 0x0, sizeof(str));
	sprintf(str, "%.1f", float_Ks);
	ssd1306_SetCursor(87, 48);
	ssd1306_WriteString(str, Font_7x10, White);

}

void Display_SelectCoeff(Display_Element_te Enum_PidCoeff)
{
	if (Enum_PidCoeff >= Display_Element_FirstPidCoeffValue &&
		Enum_PidCoeff <= Display_Element_LastPidCoeffValue)
	{
		_Enum_SelectedCoeff = Enum_PidCoeff;
	}
}

void Display_SelectController(Display_Element_te Enum_Controller)
{
	if (Enum_Controller >= Display_Element_FirstControllerValue &&
		Enum_Controller <= Display_Element_LastControllerValue)
	{
		_Enum_SelectedController = Enum_Controller;
	}
}

void Display_UnderlineElement(Display_Element_te Enum_ElementToUnderline, SSD1306_COLOR Enum_Color)
{
	switch(Enum_ElementToUnderline)
	{
	case Display_Element_1:
		ssd1306_Line(103, 13, 108, 13, Enum_Color);
		break;

	case Display_Element_2:
		ssd1306_Line(117, 13, 123, 13, Enum_Color);
		break;

	case Display_Element_Kp:
		ssd1306_Line(3, 35, 16, 35, Enum_Color);
		break;

	case Display_Element_Ki:
		ssd1306_Line(66, 34, 78, 34, Enum_Color);
		break;

	case Display_Element_Kd:
		ssd1306_Line(3, 58, 16, 58, Enum_Color);
		break;

	case Display_Element_Ks:
		ssd1306_Line(66, 58, 79, 58, Enum_Color);
		break;

	default:
		break;
	}
}

void Display_UpdateScreen(/*Controller_t *pControllerX, Controller_t *pControllerY*/)
{
	Display_DrawOutline();
	Display_UnderlineElement(_Enum_SelectedCoeff, White);
	Display_UnderlineElement(_Enum_SelectedController, White);

	switch(_Enum_SelectedController)
	{
	case Display_Element_1:
//		Display_WritePidGains(pControllerX->float_Kp, pControllerX->float_Ki,
//							  pControllerX->float_Kd, pControllerX->float_Ks);
		Display_WritePidGains(100.1, 200.2, 300.3, 400.4);
		break;

	case Display_Element_2:
//		Display_WritePidGains(pControllerY->float_Kp, pControllerY->float_Ki,
//							  pControllerY->float_Kd, pControllerY->float_Ks);
		Display_WritePidGains(500.5, 600.6, 700.7, 800.8);
		break;

	default:
		break;
	}

	ssd1306_UpdateScreen();
}

Display_Element_te Display_GetSelectedController()
{
	return _Enum_SelectedController;
}

Display_Element_te Display_GetSelectedCoeff()
{
	return _Enum_SelectedCoeff;
}
