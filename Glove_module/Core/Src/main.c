/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

#include "main.h"
#include "FreeRTOS.h"
#include "stm32f1xx.h"
#include "task.h"
#include "flex.h"
#include "USART_tx.h"
#include "ssd1306.h"
#include "ssd1306_fonts.h"

I2C_HandleTypeDef hi2c1;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);




void sendData(void* pvparameters)
{
	TickType_t  xPeriod = pdMS_TO_TICKS(5);
	TickType_t  xLastWakeTime = xTaskGetTickCount();
	while(1)
	{
		usart_tx(get_reading()) ;
		vTaskDelayUntil(&xLastWakeTime, xPeriod);
	}
}
void ADC_Read( void* pvparameters )
{
//
//	TickType_t  xPeriod = pdMS_TO_TICKS(10);
//	TickType_t  xLastWakeTime = xTaskGetTickCount();
	while(1)
	{
		ADC1->CR2 |= (1<<22) ; // Start conversion.
		while(!(ADC1->SR &(1<<1)));
		ADC1->SR &= ~(1<<1) ;// Clear the EOC Flag.
		set_fingers_v() ;

//		vTaskDelayUntil(&xLastWakeTime, xPeriod);
	}
}
void update_screen(void* pvparameters){
#ifdef SSD1306_INCLUDE_FONT_16x24
	TickType_t  xPeriod = pdMS_TO_TICKS(20);
	TickType_t  xLastWakeTime = xTaskGetTickCount();
	while(1)
	{
	char d = get_direction();
	char* direction = (d=='F')?("Forward"):(d == 'B')?("Backward"):(d == 'R')?("Right"):(d == 'L')?("Left"):("Break") ;
	char gear = get_gear()  ;
	char* speed = (gear==0)?("Neutral"):(gear==1)?("Gear 1"):(gear == 2)?("Gear 2"):("Gear 3") ;

	 ssd1306_Fill(Black);
	 ssd1306_SetCursor(0,10);
	 ssd1306_WriteString(direction,Font_11x18,White);
	 ssd1306_SetCursor(0,30);
	 ssd1306_WriteString(speed,Font_11x18,White);
	 ssd1306_UpdateScreen();
	 vTaskDelayUntil(&xLastWakeTime, xPeriod);
	}

#endif
}


int main(void)
{
  usart_init() ;
  flex_init();
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_I2C1_Init();
  ssd1306_Init() ;


//
  	xTaskCreate(sendData,"sender",128,NULL,tskIDLE_PRIORITY+3,NULL) ;
  	xTaskCreate(ADC_Read,"reader",128,NULL,tskIDLE_PRIORITY+1,NULL) ;
	xTaskCreate(update_screen,"O_led",256,NULL,tskIDLE_PRIORITY+1,NULL) ;

  	vTaskStartScheduler();

  while (1){}
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  hi2c1.Init.ClockSpeed = 400000;
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

static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

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
