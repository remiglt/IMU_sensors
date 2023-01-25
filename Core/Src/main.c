/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#include "../../Utilities/Fonts/fonts.h"
//#include "../../Drivers/BSP/inc/stm32746g_discovery.h"
//#include "../../Drivers/BSP/inc/stm32746g_discovery_sdram.h"
//#include "../../Drivers/BSP/inc/stm32746g_discovery_ts.h"
//#include "../../Drivers/BSP/inc/stm32746g_discovery_lcd.h"
//#include "../../Drivers/BSP/inc/stm32746g_discovery_lcd.h"
#include "stdio.h"
#include <math.h>
#include "Const.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//HAL_I2C_Mem_Read(&hi2c1,MPU_ADDRESS,ID_ADDRESS,1,&MPU_identity,1,10
#define ID_ADDRESS 0x75
#define MPU_ADDRESS 0xD0
#define ID_VALUE_MPU 0x
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

char mess[30];
int devAdd[256];
uint8_t MPU_identity;
double TempReceived;
double AccelReceived[3];
double SpeedReceived[3];
double MagReceived[3];

double Accel;
double Speed;
double Mag;
double offset;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int __io_putchar(int ch)
{
	HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, HAL_MAX_DELAY);

	return ch;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

	//===================================================================================================
	//Allumage/Exctinction de la LED
	HAL_GPIO_WritePin(GPIO_LED_GPIO_Port,GPIO_LED_Pin,GPIO_PIN_SET);
	HAL_Delay(1000);
	HAL_GPIO_WritePin(GPIO_LED_GPIO_Port,GPIO_LED_Pin,GPIO_PIN_RESET);
	//===================================================================================================

	//Initialisation du capteur

	Init(&hi2c1);

	//===================================================================================================
	//Affichage des adresses des péiphériques connectés à l'I2C
	int c=0;
	int i=0;
	printf("\r\nList of devices:\r\n");
	for(i=0; i<256; i++)
	{
		if(HAL_I2C_IsDeviceReady(&hi2c1,i,10,20)==HAL_OK)
		{
			devAdd[c] = i;
			printf("%d\r\n",devAdd[c]);
			c++;
		}
	}

	printf("\r\n");
	//===================================================================================================


	//===================================================================================================


	//Identité du capteur

	printf("Who Am I?\r\n");
	if(HAL_I2C_Mem_Read(&hi2c1,MPU_ADDRESS,ID_ADDRESS,1,&MPU_identity,1,10)!=HAL_OK)
	{
		printf("Error : I don't know who I am...\r\n");
		Error_Handler();
	}
	printf("%d\r\n",MPU_identity);

	if(MPU_identity == 0x71)
	{
		printf("I am the MPU-9250, nice to meet you\r\n");
	}
	else
	{
		printf("I am not the MPU-9250\r\n");
	}


	//GetAccelOffset(&hi2c1);

	Measure_T(&hi2c1, &TempReceived);
	Measure_A(&hi2c1, &AccelReceived);
	Measure_S(&hi2c1, &SpeedReceived);
	Measure_M(&hi2c1, &MagReceived);

	Speed = sqrt(SpeedReceived[0]*SpeedReceived[0] + SpeedReceived[1]*SpeedReceived[1] + SpeedReceived[2]*SpeedReceived[2]);
	printf("Total speed : %f\r\n\r\n",Speed);

	Accel = sqrt(AccelReceived[0]*AccelReceived[0] + AccelReceived[1]*AccelReceived[1] + AccelReceived[2]*AccelReceived[2]);
	printf("Total accel : %f\r\n\r\n",Accel);

	Mag = sqrt(MagReceived[0]*MagReceived[0] + MagReceived[1]*MagReceived[1] + MagReceived[2]*MagReceived[2]);
	printf("Total Mag : %f\r\n\r\n",Mag);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	/*
	Measure_T(&hi2c1,&TempReceived);
	Measure_A(&hi2c1,&AccelReceived);
	Measure_S(&hi2c1,&SpeedReceived);
	*/

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


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
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIO_LED_GPIO_Port, GPIO_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : GPIO_LED_Pin */
  GPIO_InitStruct.Pin = GPIO_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIO_LED_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
/**
 * @brief Toggle LED
 * @Note Cette fonction permet de faire clignoter la led verte pendant 3 secondes, afin d'indiquer une erreur
 * @param None
 * @retval None
 */
void ToggleLed()
{
	for(int i=0; i<6; i++)
	{
		HAL_GPIO_WritePin(GPIO_LED_GPIO_Port,GPIO_LED_Pin,GPIO_PIN_SET);
		HAL_Delay(500);
		HAL_GPIO_WritePin(GPIO_LED_GPIO_Port,GPIO_LED_Pin,GPIO_PIN_RESET);
		HAL_Delay(500);
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
	//Allumage de la LED
	ToggleLed();
	//HAL_GPIO_WritePin(GPIO_LED_GPIO_Port,GPIO_LED_Pin,GPIO_PIN_SET);
	printf("\n\rERROR\n\r");
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
