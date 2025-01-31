/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include <stdio.h>
#include "fnd_controller.h"
#include "ds18b20.h"
#include "heaterController.h"
#include "fonts.h"
#include "ssd1306.h"
#include "test.h"
#include "bitmap.h"
#include "horse_anim.h"
#include "ojtube_log.h"
//#include "utils.h"
//#include "oledController.h"
#include "g_var.h"
//#include "buttonController.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#include <stdio.h>
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/*
int _write(int file, char *p, int len){
  HAL_UART_Transmit(&huart1, (uint8_t *)p, len, 10);
  return len;
}
*/
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
int _write(int file, char *p, int len){
  HAL_UART_Transmit(&huart1, (uint8_t *)p, len, 10);
  return len;
}



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/*
int _write(int file, char *p, int len){
  HAL_UART_Transmit(&huart1, (uint8_t *)p, len, 10);
  return len;
}
*/
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

float temper = 0.0;
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
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_I2C2_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
  SSD1306_Init();


/*
  SSD1306_GotoXY (0,0);
  SSD1306_Puts ("CROP", &Font_11x18, 1);
  SSD1306_GotoXY (10, 30);
  SSD1306_Puts ("  DRYER :)", &Font_11x18, 1);
  SSD1306_UpdateScreen(); //display
  HAL_Delay (2000);


  SSD1306_GotoXY (0,0);
  SSD1306_Puts ("CROP", &Font_11x18, 1);
  SSD1306_GotoXY (10, 30);
  SSD1306_Puts ("  DRYER :)", &Font_11x18, 1);
  SSD1306_UpdateScreen(); //display

  HAL_Delay (2000);


  SSD1306_ScrollRight(0,7);  // scroll entire screen
  HAL_Delay(2000);  // 2 sec

  SSD1306_ScrollLeft(0,7);  // scroll entire screen
  HAL_Delay(2000);  // 2 sec

  SSD1306_Stopscroll();
  SSD1306_Clear();

  SSD1306_DrawBitmap(0,0,logo, 128, 64, 1);
  SSD1306_UpdateScreen();

  HAL_Delay(2000);

  SSD1306_ScrollRight(0x00, 0x0f);    // scroll entire screen right

  HAL_Delay (2000);

  SSD1306_ScrollLeft(0x00, 0x0f);  // scroll entire screen left

  HAL_Delay (2000);

  SSD1306_Scrolldiagright(0x00, 0x0f);  // scroll entire screen diagonal right

  HAL_Delay (2000);

  SSD1306_Scrolldiagleft(0x00, 0x0f);  // scroll entire screen diagonal left

  HAL_Delay (2000);

  SSD1306_Stopscroll();   // stop scrolling. If not done, screen will keep on scrolling


  SSD1306_InvertDisplay(1);   // invert the display

  HAL_Delay(2000);

  SSD1306_InvertDisplay(0);  // normalize the display


  HAL_Delay(300);                       */


  //HAL_TIM_Base_Start_IT(&htim2);
  //  SSD1306_InvertDisplay(1);

  /*
  SSD1306_Clear();
		  SSD1306_DrawBitmap(0,0,ojtubelog1,128,64,1);
		  SSD1306_UpdateScreen();
          HAL_Delay (300);
*/

init_fnd(&hspi2);


HAL_TIM_Base_Start_IT(&htim3);

Ds18b20_Init();
Ds18b20_Init_Simple();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  //  SSD1306_InvertDisplay(1);


   while (1)
  {
/*
	   SSD1306_Clear();
	  		  SSD1306_DrawBitmap(0,0,ojtubelog1,128,64,1);
	  		  SSD1306_UpdateScreen();
	            HAL_Delay (300);        */

	            /*
	   SSD1306_GotoXY (0,0);
	   SSD1306_Puts ("CROP", &Font_11x18, 1);
	   SSD1306_GotoXY (10, 30);
	   SSD1306_Puts ("  DRYER :)", &Font_11x18, 1);
	   SSD1306_UpdateScreen(); //display
	   HAL_Delay (2000);


	   SSD1306_GotoXY (0,0);
	   SSD1306_Puts ("CROP", &Font_11x18, 1);
	   SSD1306_GotoXY (10, 30);
	   SSD1306_Puts ("  DRYER :)", &Font_11x18, 1);
	   SSD1306_UpdateScreen(); //display

	   HAL_Delay (2000);


	   SSD1306_ScrollRight(0,7);  // scroll entire screen
	   HAL_Delay(2000);  // 2 sec

	   SSD1306_ScrollLeft(0,7);  // scroll entire screen
	   HAL_Delay(2000);  // 2 sec

	   SSD1306_Stopscroll();
	   SSD1306_Clear();

	   SSD1306_DrawBitmap(0,0,logo, 128, 64, 1);
	   SSD1306_UpdateScreen();

	   HAL_Delay(2000);

	   SSD1306_ScrollRight(0x00, 0x0f);    // scroll entire screen right

	   HAL_Delay (2000);

	   SSD1306_ScrollLeft(0x00, 0x0f);  // scroll entire screen left

	   HAL_Delay (2000);

	   SSD1306_Scrolldiagright(0x00, 0x0f);  // scroll entire screen diagonal right

	   HAL_Delay (2000);

	   SSD1306_Scrolldiagleft(0x00, 0x0f);  // scroll entire screen diagonal left

	   HAL_Delay (2000);

	   SSD1306_Stopscroll();   // stop scrolling. If not done, screen will keep on scrolling


	   SSD1306_InvertDisplay(1);   // invert the display

	   HAL_Delay(2000);

	   SSD1306_InvertDisplay(0);  // normalize the display


	   HAL_Delay(300);
*/


/*
	   if(g_f_sw_up){
		   printf("push g_f_sw_up\r\n");
		   g_f_sw_up = 0;
		   HAL_GPIO_TogglePin(PB6_LED1_GPIO_Port, PB6_LED1_Pin);
	   }

	   if(g_f_sw_down){
	   		   printf("push g_f_sw_down\r\n");
	   		   g_f_sw_down = 0;
	   		 HAL_GPIO_TogglePin(PB7_LED1_GPIO_Port, PB7_LED1_Pin);
	   	}

	   	if(g_f_sw_fix){
	   			   printf("push g_f_sw_fix\r\n");
	   			   g_f_sw_fix = 0;
	   			HAL_GPIO_TogglePin(PB5_RELAY_ON_OFF_CTRL_GPIO_Port, PB5_RELAY_ON_OFF_CTRL_Pin);
	   	}
	 	if(g_f_sw_on){
		   			   printf("push g_f_sw_on\r\n");
		   			   g_f_sw_on = 0;

		 }




	 	 HAL_Delay(1000);

*/

/*
        if(HAL_GPIO_ReadPin(PB12_START_SW_PIN_GPIO_Port, PB12_START_SW_PIN_Pin)){

        	   printf("1\r\n");
        }else{

               printf("0\r\n");
        }

        */
	  // HAL_UART_Transmit(&huart1,senddata, strlen(senddata), 1000);
	//  HAL_Delay(1000);
/*
if(HAL_GPIO_ReadPin(PB0_TEMP_SET_UP_GPIO_Port, PB0_TEMP_SET_UP_Pin)){
	printf("1\r\n");
}else{
	printf("0\r\n");
}
HAL_Delay(10);
*/
	//printf("hello world!\r\n");
	//HAL_Delay (1000);

	/*
	   SSD1306_Clear();
		  SSD1306_DrawBitmap(0,0,ojtubelog1,128,64,1);
		  SSD1306_UpdateScreen();
          HAL_Delay (2000);
          */




	    if(!isConverting()){
	       StartConverting();
        }

        checkConverting();

       if(!isConverting()){
         temper = getTemper();
        }

       HAL_Delay(10);


	Ds18b20_ManualConvert();


	 if(getCurrentTemper() > 50 && getHeaterState() == t_ON){
		  heaterControll(t_OFF);
	  }else if (getCurrentTemper() < 45 && getHeaterState() == t_OFF){
		  heaterControll(t_ON);
	  }


//	 HAL_GPIO_TogglePin(PB5_RELAY_ON_OFF_CTRL_GPIO_Port, PB5_RELAY_ON_OFF_CTRL_Pin);
// HAL_Delay(2000);

	//  Ds18b20_ManualConvert();
	//  digit4_replay((int)(ds18b20[0].Temperature * 10), 5000);
	  /*
	  for(int i = 0; i<=9999; i++){

	  		  digit4_replay(i, 450); //send counter 0-9999 with delay 50 cicles and hide zero

	  	  }
               */
	 // button_state = HAL_GPIO_ReadPin(PB0_TEMP_SET_UP_GPIO_Port, PB0_TEMP_SET_UP_Pin);
	 // HAL_Delay(100);

     // printf("hello world!!!\r\n");
	 // HAL_UART_Transmit(&huart1, senddata, strlen(senddata), 1000);
	  //HAL_Delay(1000);

	/*
	  HAL_GPIO_WritePin(GPIO_LED_GPIO_Port, GPIO_LED_Pin, 1);
	  HAL_Delay(100);
	  HAL_GPIO_WritePin(GPIO_LED_GPIO_Port, GPIO_LED_Pin, 0);
	  HAL_Delay(100);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 400000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = (72- 1);
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xFFFF;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 71;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 99;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIO_LED_GPIO_Port, GPIO_LED_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, PA2_TEMP_DATA_Pin|PA3_TEMP_DATA_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, FND_RCLK_Pin|PB6_LED1_Pin|PB7_LED1_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(PB5_RELAY_ON_OFF_CTRL_GPIO_Port, PB5_RELAY_ON_OFF_CTRL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : GPIO_LED_Pin */
  GPIO_InitStruct.Pin = GPIO_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIO_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : GPIO_SW_Pin */
  GPIO_InitStruct.Pin = GPIO_SW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIO_SW_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA2_TEMP_DATA_Pin PA3_TEMP_DATA_Pin */
  GPIO_InitStruct.Pin = PA2_TEMP_DATA_Pin|PA3_TEMP_DATA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0_TEMP_SET_UP_Pin PB1_TEMP_SET_FIX_Pin PB2_TEMP_SET_DOWN_Pin PB12_START_SW_PIN_GPIO_Port_Pin */
  GPIO_InitStruct.Pin = PB0_TEMP_SET_UP_Pin|PB1_TEMP_SET_FIX_Pin|PB2_TEMP_SET_DOWN_Pin|PB12_START_SW_PIN_GPIO_Port_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : FND_RCLK_Pin PB6_LED1_Pin */
  GPIO_InitStruct.Pin = FND_RCLK_Pin|PB6_LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB5_RELAY_ON_OFF_CTRL_Pin */
  GPIO_InitStruct.Pin = PB5_RELAY_ON_OFF_CTRL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(PB5_RELAY_ON_OFF_CTRL_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB7_LED1_Pin */
  GPIO_InitStruct.Pin = PB7_LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(PB7_LED1_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 9, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 9, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 9, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 9, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
