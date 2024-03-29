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

#endif /* __ENCODERINT_H_ */
