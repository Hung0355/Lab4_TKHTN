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
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "stm32f429i_discovery_lcd.h"
#include "time.h"
#include <unistd.h>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define DT 0.01         // [s/loop] loop period. 20ms
#define G_GAIN 0.070     // [deg/s/LSB]
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi5;

/* USER CODE BEGIN PV */
float val[3];
char LCD_send[30];
float rate_gyr_y = 0.0;   // [deg/s]
float rate_gyr_x = 0.0;   // [deg/s]
float rate_gyr_z = 0.0;   // [deg/s]
float gyroXangle = 0.0;
float gyroYangle = 0.0;
float gyroZangle = 0.0;
float tmpX = 0.0;
float tmpY = 0.0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI5_Init(void);
/* USER CODE BEGIN PFP */

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
  MX_SPI5_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  BSP_LCD_Init();//init LCD
  BSP_GYRO_Init();
  //set the layer buffer address into SDRAM
  BSP_LCD_LayerDefaultInit(1, SDRAM_DEVICE_ADDR);
  BSP_LCD_SelectLayer(1);//select on which layer we write
  BSP_LCD_DisplayOn();//turn on LCD
  BSP_LCD_Clear(LCD_COLOR_BLUE);//clear the LCD on blue color
  BSP_LCD_SetBackColor(LCD_COLOR_BLUE);//set text background color
  BSP_LCD_SetTextColor(LCD_COLOR_WHITE);//set text color
  //write text
  /* USER CODE END 2 */
  //int startInt  = mymillis();
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  /*BSP_LCD_FillTriangle(120,95,145,10,50,50); //phia tren
	  BSP_LCD_FillTriangle(5,45,45,160,135,185); //ben trai
	  BSP_LCD_FillTriangle(235,195,195,160,135,185); //ben phai
	  BSP_LCD_FillTriangle(120,95,145,310,270,270); //phia duoi*/
	  //startInt = mymillis();
	  BSP_GYRO_GetXYZ(val);

	  rate_gyr_x = val[0]  * G_GAIN;
	  rate_gyr_y = val[1]  * G_GAIN;
	  rate_gyr_z = val[2]  * G_GAIN;

	  gyroXangle+=rate_gyr_x*DT;
	  gyroYangle+=rate_gyr_y*DT;
	  gyroZangle+=rate_gyr_z*DT;

	  BSP_LCD_Clear(LCD_COLOR_BLUE);
	  BSP_LCD_SetBackColor(LCD_COLOR_BLUE);

	  if ((tmpX - gyroXangle)>10){
		  if ((tmpY - gyroYangle) >10){
		  		BSP_LCD_FillTriangle(120,95,145,10,50,50); //phia tren + ben trai
		  		BSP_LCD_FillTriangle(5,45,45,160,135,185);
		  		HAL_Delay(250);
		  }
		  else if ((gyroYangle - tmpY) >10){
		  		BSP_LCD_FillTriangle(120,95,145,10,50,50); // phia tren + ben phai
		  		BSP_LCD_FillTriangle(235,195,195,160,135,185);
		  		HAL_Delay(250);
		  }
		  else
			  	HAL_Delay(250);
	  }
	  else if ((gyroXangle - tmpX)>10){
		  if ((tmpY - gyroYangle) >10){
		  		BSP_LCD_SetBackColor(LCD_COLOR_BLUE);
		  		BSP_LCD_FillTriangle(120,95,145,310,270,270); //phia duoi + ben trai
		  		BSP_LCD_FillTriangle(5,45,45,160,135,185);
		  		HAL_Delay(250);
		  }
		  else if ((gyroYangle - tmpY) >10){
		  		BSP_LCD_SetBackColor(LCD_COLOR_BLUE);
		  		BSP_LCD_FillTriangle(120,95,145,310,270,270); //phia duoi + ben phai
		  		BSP_LCD_FillTriangle(235,195,195,160,135,185);
		  		HAL_Delay(250);
		  	}
		  else
		  		HAL_Delay(250);
	  }
	  else
		  	 	HAL_Delay(250);

	  tmpX = gyroXangle;
	  tmpY = gyroYangle;

	  sprintf(LCD_send,"x = %f, y = %f, z = %f \n",gyroXangle,gyroYangle, gyroZangle);
	  CDC_Transmit_HS(LCD_send,strlen(LCD_send));

	 for (int i = 0;i<1500000;i++);

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI5_Init(void)
{

  /* USER CODE BEGIN SPI5_Init 0 */

  /* USER CODE END SPI5_Init 0 */

  /* USER CODE BEGIN SPI5_Init 1 */

  /* USER CODE END SPI5_Init 1 */
  /* SPI5 parameter configuration*/
  hspi5.Instance = SPI5;
  hspi5.Init.Mode = SPI_MODE_MASTER;
  hspi5.Init.Direction = SPI_DIRECTION_2LINES;
  hspi5.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi5.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi5.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi5.Init.NSS = SPI_NSS_SOFT;
  hspi5.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi5.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi5.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi5.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi5.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI5_Init 2 */

  /* USER CODE END SPI5_Init 2 */

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13|GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PG13 PG14 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

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
