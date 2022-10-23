/*
 * MotorInt.h
 *
 *  Created on: Jul 9, 2022
 *      Author: Federico Altieri
 */

#ifndef __MOTORINT_H_
#define __MOTORINT_H_

#include "MotorExt.h"

#define PWM_FREQUENCY_KHZ 		   25			/* f_in = [20, 30] KHz */

#define ARR_VALUE 	     (uint16_t)1000			/* Value of the ARR Register for source clock frequency 84MHz*/
#define PSC_VALUE 				   1			/* Value of the PSC Register */

#endif /* __MOTORINT_H_ */
