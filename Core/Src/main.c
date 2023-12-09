/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mpu6050_Ext.h"
#include "controller_Ext.h"
#include "MotorExt.h"
#include "EncoderExt.h"
#include "Buzzer_Ext.h"
#include "Remote_Cmd_Ext.h"
#include "Display_Ext.h"
#include "ssd1306.h"
#include "eeprom.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* Controller section*/
Controller_t _sControllerX;
Controller_t _sControllerY;

/* Motor X section*/
Motor_Handle_t _sMotorHandleX;
float _float_VoltageValueX = 0;
float _float_DutyCycleX = 0;

/* Motor Y section */
Motor_Handle_t _sMotorHandleY;
float _float_VoltageValueY = 0;
float _float_DutyCycleY = 0;

/* Tick variables */
uint32_t _xRoutineStartTick = 0;
uint32_t _xRoutineEndTick = 0;
uint32_t _xRoutineElapsedTime = 0;
uint32_t _xRoutineNow = 0;

Motor_Brake_t _sMotorBrake;

Bool_t _Bool_IsControlLoopEnabled = FALSE;

MPU6050_Angles_t _sMain_Angles = {0};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void Main_ControlLoop();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  Error_t Error = E_OK;
  Motor_BrakeState_te Enum_BrakeState = BRAKE_ON;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  memset(&hi2c2, 0x0, sizeof(hi2c2));
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM10_Init();
  MX_I2C2_Init();
  MX_ADC1_Init();
  MX_I2C3_Init();
  MX_TIM8_Init();
  MX_TIM11_Init();
  MX_TIM12_Init();
  /* USER CODE BEGIN 2 */

  /* Start encoder timer and clear any interrupt */
  __HAL_TIM_CLEAR_IT(&htim3, TIM_IT_UPDATE);
  __HAL_TIM_CLEAR_IT(&htim4, TIM_IT_UPDATE);
  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_Base_Start_IT(&htim4);

  /* IMU initialization */
  while (MPU6050_Init(&hi2c2) == 1)
  {
	  HAL_Delay (100);
  }

  /* Start IMU sample loop timer */
  HAL_TIM_Base_Start_IT(&htim11);

  /* Brake structure initialization */
  _sMotorBrake.Enum_BrakeState = BRAKE_OFF;
  _sMotorBrake.GPIO_Pin_Brake = BRAKE_XY_Pin;
  _sMotorBrake.GPIO_Port_Brake = BRAKE_XY_GPIO_Port;

  /* Motor initialization */
  memset(&_sMotorHandleX , 0, sizeof(_sMotorHandleX));
  _sMotorHandleX.htim = &htim2;
  _sMotorHandleX.uint32_TimChannel = TIM_CHANNEL_2;
  _sMotorHandleX.GPIO_Pin_SpinDirection = CW_CCW_Y_Pin;
  _sMotorHandleX.GPIO_Port_SpinDirection = CW_CCW_Y_GPIO_Port;
  Motor_Init(&_sMotorHandleX, &_sMotorBrake);

  memset(&_sMotorHandleY , 0, sizeof(_sMotorHandleY));
  _sMotorHandleY.htim = &htim2;
  _sMotorHandleY.uint32_TimChannel = TIM_CHANNEL_1;
  _sMotorHandleY.GPIO_Pin_SpinDirection = CW_CCW_X_Pin;
  _sMotorHandleY.GPIO_Port_SpinDirection = CW_CCW_X_GPIO_Port;
  Motor_Init(&_sMotorHandleY, &_sMotorBrake);

  /* Encoder initialization */
  Error = Encoder_Init(htim3.Instance, htim4.Instance);

  /* Controller initialization*/
  Error_Check(Error, Controller_Init(&_sControllerX,	//controller instance
		  	  	  	  	  	  	  	 CONTROLLER_AXIS_X, // direction
									 0,					// set point
									 200,				// Kp
									 2,                	// Ki
									 0));  				// Kd


  Error_Check(Error, Controller_Init(&_sControllerY,	//controller instance
		  	  	  	  	  	  	  	 CONTROLLER_AXIS_Y,	// direction
									 0,					// set point
									 200,				// Kp
									 2,					// Ki
									 0));  				// Kd

  /* OLED display initialization */
  Display_Init(&_sControllerX, &_sControllerY);

  /* Remote command module initialization */
  Remote_Cmd_Init(&_sMotorBrake, &_Bool_IsControlLoopEnabled);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  /* Wait till the stick is in vertical position */
  while (FALSE == MPU6050_IsVertical())
  {
	  HAL_Delay (100);
  }
  /* Toggle brake ON */
  Motor_SetBrake(&_sMotorBrake, Enum_BrakeState);

  /* Start Control loop timer */
  HAL_TIM_Base_Start_IT(&htim12);

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  asm("NOP");
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV8;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void Main_ControlLoop()
{
	_xRoutineStartTick = HAL_GetTick();

	if (_Bool_IsControlLoopEnabled == TRUE)
	{
		/* IMU Utility function, uncomment if necessary */
		/* Calculation of Accelerometer and Gyroscope offsets, need to be hard-coded manually after calibration */
		//	  MPU6050_Calibrate();

		/* Calculation of set point, need to be hard-coded manually after calibration */
		//	  MPU6050_CalculateSetPoint();

		/* Periodic update of OLED display to show updated info */
		//	  Display_UpdateScreen();

		/* Controller_X routine */
		Controller_GetPIDVoltageValue(_xRoutineStartTick, _sMain_Angles, &_sControllerX, &_float_VoltageValueX);
		Controller_CalculateDutyCycle(&_sControllerX, _float_VoltageValueX, &_float_DutyCycleX);
		Motor_SetDutyCycle(&_sMotorHandleX, _float_DutyCycleX);

		_xRoutineNow = HAL_GetTick();

		/* Controller_Y routine */
		Controller_GetPIDVoltageValue(_xRoutineNow, _sMain_Angles, &_sControllerY, &_float_VoltageValueY);
		Controller_CalculateDutyCycle(&_sControllerY, _float_VoltageValueY, &_float_DutyCycleY);
		Motor_SetDutyCycle(&_sMotorHandleY, -(_float_DutyCycleY));

		/* Sampling Time */
		_xRoutineEndTick = HAL_GetTick();
		_xRoutineElapsedTime = _xRoutineEndTick - _xRoutineStartTick;
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if ((htim == &htim3) || (htim == &htim4))
	{
		Encoder_InterruptCallback(htim);
	}
	/* TIM11 - IMU sample loop */
	else if (htim == &htim11)
	{
		MPU6050_Get_Angles(&_sMain_Angles);
	}
	/* TIM12 - Control loop */
	else if (htim == &htim12)
	{
		Main_ControlLoop();
	}
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
