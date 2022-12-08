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
#include "cmsis_os.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "stm32f429i_discovery_lcd.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define DT 0.01         // [s/loop] loop period. 20ms
#define G_GAIN 0.070     // [deg/s/LSB]
#define RAD_TO_DEG 57.29578
#define M_PI 3.14159265358979323846
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
osThreadId Task01Handle;
osThreadId Task02Handle;
osMessageQId Queue1Handle;
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


typedef struct
{
  float x_ang_rate;
  float y_ang_rate;
  float z_ang_rate;

} GYRO_DATA_T;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
void StartTask01(void const * argument);
void StartTask02(void const * argument);

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
  /* USER CODE BEGIN 2 */
  BSP_LCD_Init();//init LCD
  //set the layer buffer address into SDRAM
  BSP_LCD_LayerDefaultInit(1, SDRAM_DEVICE_ADDR);
  BSP_LCD_SelectLayer(1);//select on which layer we write
  BSP_LCD_DisplayOn();//turn on LCD
  BSP_LCD_Clear(LCD_COLOR_BLUE);//clear the LCD on blue color
  BSP_LCD_SetBackColor(LCD_COLOR_BLUE);//set text background color
  BSP_LCD_SetTextColor(LCD_COLOR_WHITE);//set text color
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of Queue1 */
  osMessageQDef(Queue1, 16, GYRO_DATA_T);
  Queue1Handle = osMessageCreate(osMessageQ(Queue1), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of Task01 */
  osThreadDef(Task01, StartTask01, osPriorityNormal, 0, 256);
  Task01Handle = osThreadCreate(osThread(Task01), NULL);

  /* definition and creation of Task02 */
  osThreadDef(Task02, StartTask02, osPriorityNormal, 0, 128);
  Task02Handle = osThreadCreate(osThread(Task02), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */
void GYRO_Task(void)
{
	BSP_GYRO_Init();
	GYRO_DATA_T gyro_data;
	osSignalWait(0x1, osWaitForever); //Wait for signal. Check cmsis_os.c for API name.
	for (;;)
	  {
		BSP_GYRO_GetXYZ(val);

		rate_gyr_x = val[0]  * G_GAIN;
		rate_gyr_y = val[1]  * G_GAIN;
		rate_gyr_z = val[2]  * G_GAIN;
		BSP_LCD_FillTriangle(120,95,145,10,50,50);
		gyro_data.x_ang_rate+=rate_gyr_x*DT;
		gyro_data.y_ang_rate+=rate_gyr_y*DT;
		gyro_data.z_ang_rate+=rate_gyr_z*DT;
	    /*Send gyroscope data to USB*/
	    /*GYRO_DATA_T *gyro_tx;
	    gyro_tx->x_ang_rate = gyro_data.x_ang_rate;
	    gyro_tx->y_ang_rate = gyro_data.y_ang_rate;
	    gyro_tx->z_ang_rate = gyro_data.z_ang_rate;*/
	    osMessagePut(Queue1Handle, (uint32_t)&gyro_data, 0); //Put data into mail queue. Check cmsis_os.c for API name

	    //LCD

	    BSP_LCD_Clear(LCD_COLOR_BLUE);
	    BSP_LCD_SetBackColor(LCD_COLOR_BLUE);
	    if ((tmpX - gyro_data.x_ang_rate)>10){
	    		  if ((tmpY - gyro_data.y_ang_rate) >10){
	    		  		BSP_LCD_FillTriangle(120,95,145,10,50,50); //phia tren + ben trai
	    		  		BSP_LCD_FillTriangle(5,45,45,160,135,185);
	    		  		HAL_Delay(250);
	    		  }
	    		  else if ((gyro_data.y_ang_rate - tmpY) >10){
	    		  		BSP_LCD_FillTriangle(120,95,145,10,50,50); // phia tren + ben phai
	    		  		BSP_LCD_FillTriangle(235,195,195,160,135,185);
	    		  		HAL_Delay(250);
	    		  }
	    		  else
	    			  	HAL_Delay(250);
	    	  }
	    	  else if ((gyro_data.x_ang_rate - tmpX)>10){
	    		  if ((tmpY - gyro_data.y_ang_rate) >10){
	    		  		BSP_LCD_FillTriangle(120,95,145,310,270,270); //phia duoi + ben trai
	    		  		BSP_LCD_FillTriangle(5,45,45,160,135,185);
	    		  		HAL_Delay(250);
	    		  }
	    		  else if ((gyro_data.y_ang_rate - tmpY) >10){
	    		  		BSP_LCD_FillTriangle(120,95,145,310,270,270); //phia duoi + ben phai
	    		  		BSP_LCD_FillTriangle(235,195,195,160,135,185);
	    		  		HAL_Delay(250);
	    		  	}
	    		  else
	    		  		HAL_Delay(250);
	    	  }
	    	  else
	    		  	 	HAL_Delay(250);
	    	  tmpX = gyro_data.x_ang_rate;
	    	  tmpY = gyro_data.y_ang_rate;
	    	  for (int i = 0;i<1500000;i++);
	  }
}

void USB_Task(void)
{
	osSignalWait(0x1, 400);                        //Wait for USB host to configure port
	//BSP_LCD_FillTriangle(120,95,145,10,50,50);
	osSignalSet(Task02Handle, 0x1);                          //Set signal in Task 02
	for (;;)
	{
		osEvent event = osMessageGet(Queue1Handle, osWaitForever); //Get queue
		GYRO_DATA_T *gyro_rx = event.value.p;
		sprintf(LCD_send,"x = %f, y = %f, z = %f \n",gyro_rx->x_ang_rate,gyro_rx->y_ang_rate,gyro_rx->z_ang_rate);
		//sprintf(LCD_send,"x = %f, y = %f, z = %f \n",0.1,0.2,0.3);
		CDC_Transmit_HS(LCD_send,strlen(LCD_send));
	}
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartTask01 */
/**
  * @brief  Function implementing the Task01 thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartTask01 */
void StartTask01(void const * argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  USB_Task();
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the Task02 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void const * argument)
{
  /* USER CODE BEGIN StartTask02 */
  /* Infinite loop */
	 GYRO_Task();
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartTask02 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
