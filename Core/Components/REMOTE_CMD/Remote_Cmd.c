/*
 * Remote_Cmd.c
 *
 *  Created on: 19 ott 2022
 *      Author: emanu
 */

#include "Remote_Cmd_Int.h"
#include "Display_Ext.h"

void Remote_Cmd_Init(Motor_Brake_t *psMotorBrake, Bool_t *pBool_IsControlLoopEnabled, float* vEntityItemPointer[][3])
{
	_sRemote_Cmd_Context.sNecParams.timerHandle = &htim8;

	_sRemote_Cmd_Context.sNecParams.timerChannel = TIM_CHANNEL_1;
	_sRemote_Cmd_Context.sNecParams.timerChannelActive = HAL_TIM_ACTIVE_CHANNEL_1;

	_sRemote_Cmd_Context.sNecParams.timingBitBoundary = 1680;
	_sRemote_Cmd_Context.sNecParams.timingAgcBoundary = 12500;
	_sRemote_Cmd_Context.sNecParams.type = NEC_NOT_EXTENDED;

	_sRemote_Cmd_Context.sNecParams.NEC_DecodedCallback = Remote_Cmd_NecDecodedCallback;
	_sRemote_Cmd_Context.sNecParams.NEC_ErrorCallback = Remote_Cmd_NecErrorCallback;
	_sRemote_Cmd_Context.sNecParams.NEC_RepeatCallback = Remote_Cmd_NecRepeatCallback;

	_sRemote_Cmd_Context.psMotorBrake = psMotorBrake;
	_sRemote_Cmd_Context.pBool_IsControlLoopEnabled = pBool_IsControlLoopEnabled;

	for (uint8_t i = 0; i <= Display_Element_LastControllerValue; i++)
	{
		for (uint8_t j = 0; j <= Display_Element_LastPidCoeffValue - Display_Element_FirstPidCoeffValue; j++)
		{
			_sRemote_Cmd_Context.vEntityItemPointer[i][j] = vEntityItemPointer[i][j];
		}
	}

	NEC_Read(&_sRemote_Cmd_Context.sNecParams);
}

void Remote_Cmd_OnCmdReceived(uint8_t uint8_Cmd)
{
	Display_Element_te Enum_SelectedController = Display_Element_Invalid;
	Display_Element_te Enum_SelectedCoeff = Display_Element_Invalid;

	Enum_SelectedController = Display_GetSelectedController();
	Enum_SelectedCoeff = Display_GetSelectedCoeff();

	switch(uint8_Cmd)
	{
	case 0x45:
		/* "POWER" */
		if (_sRemote_Cmd_Context.psMotorBrake->Enum_BrakeState == BRAKE_OFF)
		{
			Motor_SetBrake(_sRemote_Cmd_Context.psMotorBrake, BRAKE_ON);
		}
		else if (_sRemote_Cmd_Context.psMotorBrake->Enum_BrakeState == BRAKE_ON)
		{
			Motor_SetBrake(_sRemote_Cmd_Context.psMotorBrake, BRAKE_OFF);
		}
		break;

	case 0x47:
		/* "FUNC/STOP" */
		break;

	case 0x46:
		/* "VOL+" */
		if(Enum_SelectedCoeff == Display_Element_Kd)
		{
			Display_SelectCoeff(Display_Element_Kp);
		}
		break;

	case 0x44:
		/* "FAST BACK" */
		if(Enum_SelectedCoeff == Display_Element_Ki)
		{
			Display_SelectCoeff(Display_Element_Kp);
		}
		break;

	case 0x40:
		/* "PAUSE" */
		if(Enum_SelectedController == Display_Element_1)
		{
			Display_SelectController(Display_Element_2);
		}
		else if(Enum_SelectedController == Display_Element_2)
		{
			Display_SelectController(Display_Element_1);
		}
		break;

	case 0x43:
		/* "FAST FORWARD" */
		if(Enum_SelectedCoeff == Display_Element_Kp)
		{
			Display_SelectCoeff(Display_Element_Ki);
		}
		break;

	case 0x07:
		/* "DOWN" */
		break;

	case 0x15:
		/* "VOL-" */
		if(Enum_SelectedCoeff == Display_Element_Kp)
		{
			Display_SelectCoeff(Display_Element_Kd);
		}
		break;

	case 0x09:
		/* "UP" */
		break;

	case 0x19:
		/* "EQ" */
		*_sRemote_Cmd_Context.vEntityItemPointer[Enum_SelectedController][Enum_SelectedCoeff - Display_Element_FirstPidCoeffValue] += 0.1;
		break;

	case 0x0d:
		/* "ST/REPT" */
		break;

	case 0x16:
		/* "0" */
		*_sRemote_Cmd_Context.vEntityItemPointer[Enum_SelectedController][Enum_SelectedCoeff - Display_Element_FirstPidCoeffValue] -= 0.1;
		break;

	case 0x0c:
		/* "1" */
		*_sRemote_Cmd_Context.vEntityItemPointer[Enum_SelectedController][Enum_SelectedCoeff - Display_Element_FirstPidCoeffValue] -= 1;
		break;

	case 0x18:
		/* "2" */
		*_sRemote_Cmd_Context.vEntityItemPointer[Enum_SelectedController][Enum_SelectedCoeff - Display_Element_FirstPidCoeffValue] += 1;
		break;

	case 0x5e:
		/* "3" */
		break;

	case 0x08:
		/* "4" */
		*_sRemote_Cmd_Context.vEntityItemPointer[Enum_SelectedController][Enum_SelectedCoeff - Display_Element_FirstPidCoeffValue] -= 10;
		break;

	case 0x1c:
		/* "5" */
		*_sRemote_Cmd_Context.vEntityItemPointer[Enum_SelectedController][Enum_SelectedCoeff - Display_Element_FirstPidCoeffValue] += 10;
		break;

	case 0x5a:
		/* "6" */
		break;

	case 0x42:
		/* "7" */
		*_sRemote_Cmd_Context.vEntityItemPointer[Enum_SelectedController][Enum_SelectedCoeff - Display_Element_FirstPidCoeffValue] -= 100;
		break;

	case 0x52:
		/* "8" */
		*_sRemote_Cmd_Context.vEntityItemPointer[Enum_SelectedController][Enum_SelectedCoeff - Display_Element_FirstPidCoeffValue] += 100;
		break;

	case 0x4a:
		/* "9" */
		*_sRemote_Cmd_Context.pBool_IsControlLoopEnabled = !(*_sRemote_Cmd_Context.pBool_IsControlLoopEnabled);
		break;
	}

	for (uint8_t i = 0; i <= Display_Element_LastControllerValue; i++)
	{
		for (uint8_t j = 0; j <= Display_Element_LastPidCoeffValue - Display_Element_FirstPidCoeffValue; j++)
		{
			if (*_sRemote_Cmd_Context.vEntityItemPointer[i][j] <= 0)
				*_sRemote_Cmd_Context.vEntityItemPointer[i][j] = 0;
		}
	}
}

void Remote_Cmd_NecDecodedCallback(uint16_t uint16_Address, uint8_t uint8_Cmd)
{
	Remote_Cmd_OnCmdReceived(uint8_Cmd);
	NEC_Read(&_sRemote_Cmd_Context.sNecParams);
}

void Remote_Cmd_NecErrorCallback()
{
	NEC_Read(&_sRemote_Cmd_Context.sNecParams);
}

void Remote_Cmd_NecRepeatCallback()
{
	NEC_Read(&_sRemote_Cmd_Context.sNecParams);
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if (htim == &htim8)
	{
		NEC_TIM_IC_CaptureCallback(&_sRemote_Cmd_Context.sNecParams);
	}
}
