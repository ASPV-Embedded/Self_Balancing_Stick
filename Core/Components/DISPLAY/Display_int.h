/*
 * Display_int.h
 *
 *  Created on: Oct 11, 2022
 *      Author: emanu
 */

#ifndef INC_DISPLAY_INT_H_
#define INC_DISPLAY_INT_H_

#include "Display_ext.h"
#include "ssd1306.h"
#include <stdio.h>
#include <string.h>

Display_Element_te _Enum_SelectedCoeff = Display_Element_Kp;
Display_Element_te _Enum_SelectedController = Display_Element_1;

void Display_DrawOutline();
void Display_WritePidGains(float float_Kp, float float_Ki, float float_Kd, float float_Ks);
void Display_UnderlineElement(Display_Element_te Enum_ElementToUnderline, SSD1306_COLOR Enum_Color);

#endif /* INC_DISPLAY_INT_H_ */
