/*
 * EncoderInt.h
 *
 *  Created on: 11 lug 2022
 *      Author: Federico Altieri
 */

#ifndef __ENCODERINT_H_
#define __ENCODERINT_H_

#include "EncoderExt.h"
#include "tim.h"

#define PI 3.14159265358979323846

#define CPR 100	/* Count Per Revolution Nidec 24H */
#define RESOLUTION 2	/* Previously configured in MX Tool TIM_ENCODERMODE_TI1 */
#define SPEED_FILTER 0.85	/*  */

Encoder_Handle_TypeDef _sEncoderX;
Encoder_Handle_TypeDef _sEncoderY;

float float_AngSpeedUnfiltered = 0;
float float_AngSpeedFiltered = 0;
float float_ThetaK = 0;
float float_ThetaK_1 = 0;
float float_PrevSpeed = 0;
uint32_t xCurrentTick = 0;
uint32_t xLastTick = 0;

#endif /* __ENCODERINT_H_ */
