/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "vl53l1_api.h"
#include "vl53l1_platform.h"
#include "serialplot.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
VL53L1_Dev_t      dev;
VL53L1_DEV        Dev = &dev;
VL53L1_CalibrationData_t Calibration_Data;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**********************************************
// IIC Write Command
**********************************************/
void Write_IIC_Command(unsigned char IIC_Command)
{
	HAL_I2C_Mem_Write(&hi2c1 ,0x78,0x00,I2C_MEMADD_SIZE_8BIT,&IIC_Command,1,0x100);
}
/**********************************************
// IIC Write Data
**********************************************/
void Write_IIC_Data(unsigned char IIC_Data)
{
	HAL_I2C_Mem_Write(&hi2c1 ,0x78,0x40,I2C_MEMADD_SIZE_8BIT,&IIC_Data,1,0x100);
}

void OLED_WR_Byte(unsigned char dat,unsigned cmd)
{
	if(cmd)
	{
		HAL_I2C_Mem_Write(&hi2c1 ,0x78,0x40,I2C_MEMADD_SIZE_8BIT,&dat,1,0x100);
	}
	else
	{
		HAL_I2C_Mem_Write(&hi2c1 ,0x78,0x00,I2C_MEMADD_SIZE_8BIT,&dat,1,0x100);
	}

}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	static VL53L1_RangingMeasurementData_t RangingData;
	VL53L1_UserRoi_t roiConfig;

	uint8_t byteData;
	uint8_t status;
	int16_t distance=0;
	uint16_t print_count = 0;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_TIM7_Init();
  MX_TIM6_Init();
  MX_USART3_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim6);
  HAL_TIM_Base_Start_IT(&htim7);

	printf("VL53L1X Examples...\n");
	Dev->I2cHandle = &hi2c1;
	Dev->I2cDevAddr = 0x52;
	Dev->comms_type = 1;
	Dev->comms_speed_khz = 400;

	status = VL53L1_RdByte(Dev, 0x010F, &byteData);
	printf("VL53L1X Model_ID: %x\r\n", byteData);
	HAL_Delay(500);
	status = VL53L1_RdByte(Dev, 0x0110, &byteData);
	printf("VL53L1X Module_Type: %x\r\n", byteData);
	HAL_Delay(500);


	printf("Autonomous Ranging Test\n");
	HAL_Delay(500);
	status = VL53L1_WaitDeviceBooted(Dev);
	status = VL53L1_DataInit(Dev);
	status = VL53L1_StaticInit(Dev);
	roiConfig.TopLeftX = 0;
	roiConfig.TopLeftY = 15;
	roiConfig.BotRightX = 15;
	roiConfig.BotRightY = 0;
	status = VL53L1_SetUserROI(Dev,&roiConfig);

	status = VL53L1_GetCalibrationData(Dev, &Calibration_Data);
	Calibration_Data.customer.algo__crosstalk_compensation_plane_offset_kcps = 2000;
	Calibration_Data.customer.algo__part_to_part_range_offset_mm = 156;
	Calibration_Data.customer.global_config__spad_enables_ref_0 = 255;
	Calibration_Data.customer.global_config__spad_enables_ref_1 = 255;
	Calibration_Data.customer.global_config__spad_enables_ref_2 = 255;
	Calibration_Data.customer.global_config__spad_enables_ref_3 = 255;
	Calibration_Data.customer.global_config__spad_enables_ref_4 = 255;
	Calibration_Data.customer.global_config__spad_enables_ref_5 = 255;
	Calibration_Data.customer.ref_spad_man__num_requested_ref_spads = 8;
	Calibration_Data.customer.ref_spad_man__ref_location = 1;
	status = VL53L1_SetCalibrationData(Dev, &Calibration_Data);



	status = VL53L1_SetDistanceMode(Dev, VL53L1_DISTANCEMODE_LONG);
	status = VL53L1_SetMeasurementTimingBudgetMicroSeconds(Dev, 50000);
	status = VL53L1_SetInterMeasurementPeriodMilliSeconds(Dev, 10);
	status = VL53L1_StartMeasurement(Dev);


	if(status){
		printf("VL53L1_StartMeasurement failed \n");
		while(1);
	}
	do // polling mode
		{
		  status = VL53L1_WaitMeasurementDataReady(Dev);
			if(!status)
			{
				status = VL53L1_GetRangingMeasurementData(Dev, &RangingData);
				if(status==0){
					SerialPlotFramePlotWord(RangingData.RangeMilliMeter,RangingData.RangeMilliMeter);
					distance = distance * 0.8f + RangingData.RangeMilliMeter * 0.2f;
					if(print_count++ >= 10)
					{
						print_count = 0;
//						printf("distance is: %d mm\r\n",distance);
//						SerialPlotFramePlotWord(distance,distance);
					}

//					printf("%d\r\n", RangingData.RangeMilliMeter);
				}
				status = VL53L1_ClearInterruptAndStartMeasurement(Dev);
			}
		}
		while (1);
/* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//	  uint8_t _iic_send = 0x0f;
//	  uint8_t _iic_read ;
//	  HAL_I2C_Mem_Write(&hi2c1 ,0x78,0x00,I2C_MEMADD_SIZE_8BIT,&_iic_send,1,0xFFFF);	//write datas
//	  HAL_I2C_Mem_Write(&hi2c1 ,0x78,0x40,I2C_MEMADD_SIZE_8BIT,&_iic_send,1,0xFFFF);	//write command
//	  HAL_I2C_Mem_Read(&hi2c1,0x78,0x00,I2C_MEMADD_SIZE_8BIT,&_iic_read,1,0xFFFF);		//read from sensor
	  HAL_Delay(500);
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
