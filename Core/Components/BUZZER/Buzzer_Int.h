/*
 * Buzzer_Int.h
 *
 *  Created on: Sep 16, 2022
 *      Author: Emanuele Somma
 */

#ifndef __BUZZER_INT_H_
#define __BUZZER_INT_H_

#include "Buzzer_Ext.h"
#include "Notes_Frequencies.h"

int _notes[] = {PA, Fad4, Fad4, Re5, Dod5, PA, PA, PA, Si4, Si4, La4, Si4, PA, PA, PA, Fad4, Fad4, Re5, Dod5, PA, PA, PA, Si4, Si4, La4, Si4, Si4, La4, Si4, La4, Si4, La4, Fad4, PA, Fad4, Fad4, Re5, Dod5, PA, Dod5, Dod5, Re5, Si4, PA, Si4, Si4, La4, Si4, Si4, Si4, La4, Si4, La4, Fad4, PA, Fad4, Fad4, Re5, Dod5, PA, Dod5, Dod5, Re5, Si4, PA, Si4, Si4, La4, Si4, Si4, Si4, La4, Si4, La4, Fad4, PA, Fad4, Fad4, Re5, Dod5, PA, Dod5, Dod5, Re5, Si4, PA, Si4, Si4, La4, Si4, Si4, Si4, La4, Si4, La4, Fad4, PA, Fad4, Fad4, Re5, Dod5, PA, Dod5, Dod5, Re5, Si4, PA, Si4, Si4, La4, Si4, Si4, Si4, La4, Si4, La4, Fad4, PA, Fad4, Fad4, Re5, Dod5, PA, Dod5, Dod5, Re5, Si4, PA, Si4, Si4, La4, Si4, Si4, Si4, La4, Si4, La4, Fad4, PA, Fad4, Fad4, Re5, Dod5, PA, Dod5, Dod5, Re5, Si4, PA, Si4, Si4, La4, Si4, Si4, Si4, La4, Si4, La4, Fad4, PA, Fad4, Fad4, Re5, Dod5, PA, Dod5, Dod5, Re5, Si4, PA, Si4, Si4, La4,D4, D4, E4, E4, Fd4, Cd4, Cd4, D4, D4, E4, D4, D4, D4, D4, D4, G4, G4, Fd4, Fd4, G4, A4, A4, G4, G4,
				Fd4, E4, E4, Fd4, Fd4, A4, Fd4, Fd4, Fd4, Fd4, Fd4, G4, G4, Fd4, Fd4, E4};
uint32_t _values[] = {2, 4, 8, 8, 4, 4, 2, 2, 4, 8, 8, 4, 4, 2, 2, 4, 8, 8, 4, 4, 2, 2, 4, 8, 8, 4, 8, 8, 8, 8, 8, 8, 4, 4, 4, 8, 8, 4, 4, 4, 8, 8, 4, 4, 4, 8, 8, 4, 4, 8, 8, 8, 8, 4, 4, 4, 8, 8, 4, 4, 4, 8, 8, 4, 4, 4, 8, 8, 4, 4, 8, 8, 8, 8, 4, 4, 4, 8, 8, 4, 4, 4, 8, 8, 4, 4, 4, 8, 8, 4, 4, 8, 8, 8, 8, 4, 4, 4, 8, 8, 4, 4, 4, 8, 8, 4, 4, 4, 8, 8, 4, 4, 8, 8, 8, 8, 4, 4, 4, 8, 8, 4, 4, 4, 8, 8, 4, 4, 4, 8, 8, 4, 4, 8, 8, 8, 8, 4, 4, 4, 8, 8, 4, 4, 4, 8, 8, 4, 4, 4, 8, 8, 4, 4, 8, 8, 8, 8, 4, 4, 4, 8, 8, 4, 4, 4, 8, 8, 4, 4, 4, 8, 8,4,8,4,8,4,4,8,4,8,4,4,8,4,8,4,4,8,4,8,4,4,8,4,8,4,4,8,4,8,4,4,8,4,8,4,4,8,4,8,4};

#endif /* __BUZZER_INT_H_ */
