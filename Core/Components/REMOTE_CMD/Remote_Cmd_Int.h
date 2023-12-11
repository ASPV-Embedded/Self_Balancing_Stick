/*
 * Remote_Cmd_Int.h
 *
 *  Created on: 19 ott 2022
 *      Author: emanu
 */

#ifndef COMPONENTS_REMOTE_CMD_REMOTE_CMD_INT_H_
#define COMPONENTS_REMOTE_CMD_REMOTE_CMD_INT_H_

#include "Remote_Cmd_Ext.h"
#include "NEC_Decode.h"
#include "tim.h"

typedef struct
{
	NEC sNecParams;
	Motor_Brake_t *psMotorBrake;
	Bool_t *pBool_IsControlLoopEnabled;
	float* vEntityItemPointer[2][3];

}Remote_Cmd_Context_ts;

Remote_Cmd_Context_ts _sRemote_Cmd_Context = {0};

float value = 350.1;

void Remote_Cmd_OnCmdReceived(uint8_t uint8_Cmd);

void Remote_Cmd_NecDecodedCallback(uint16_t uint16_Address, uint8_t uint8_Cmd);
void Remote_Cmd_NecErrorCallback();
void Remote_Cmd_NecRepeatCallback();

#endif /* COMPONENTS_REMOTE_CMD_REMOTE_CMD_INT_H_ */
