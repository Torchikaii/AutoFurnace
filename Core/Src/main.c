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
#include "file.h"
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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int s_counter;
int m_counter;
unsigned int h_counter;

uint16_t raw_data;
uint16_t temperature;

typedef enum {BUTTON_PRESSED, BUTTON_RELEASED} BStatus;
BStatus Button_Status=BUTTON_RELEASED;

float read_temperature();
int ignition_mode();
void standby_mode();
void heating_mode();

int ignition_mode(){
	
	// deactivate everything just in case
	//HAL_GPIO_WritePin(V_GPIO_Port, V_Pin, GPIO_PIN_RESET);  // deactivate chimney fan
	//HAL_GPIO_WritePin(K_GPIO_Port, K_Pin, GPIO_PIN_RESET);  // deactivate ignition heater
	
	m_counter = 0;
	//HAL_GPIO_WritePin(G_GPIO_Port, G_Pin, GPIO_PIN_SET);  // send wood pellets for 20 seconds
	if (m_counter == 20000) {
	
		//HAL_GPIO_WritePin(G_GPIO_Port, G_Pin, GPIO_PIN_RESET);  // stop sending wood pellets
	}
	
	//HAL_GPIO_WritePin(V_GPIO_Port, V_Pin, GPIO_PIN_SET);  // activate chimney fan
	
	h_counter = 0;
	//HAL_GPIO_WritePin(K_GPIO_Port, K_Pin, GPIO_PIN_SET);  // activate ignition heater
	if ((h_counter == 600000) || (temperature > 85)) {
		
		// if smoke has reached 85 degress we stop ignition heater, ignition was successfull
		// we also stop ignition heater if it's working for more than 10 minutes
		//HAL_GPIO_WritePin(K_GPIO_Port, K_Pin, GPIO_PIN_RESET);
	}
	
	if ((h_counter > 600000) && (temperature < 85)) {
		// if ignition heater is running for more than 10 minutes and pellets still haven't caught on fire
		// because chimney temperature is low as there's no smoke, that means ignition failed
		
		return 1; // ignition failed
	}
	else {
		return 0; // ignition was successful
	}

}


float read_temperature() {

	uint8_t data[16];
	
	// CS enable
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
	
	for (int i =0; i <= 16; i++) {
		
		// SCK enable
		HAL_GPIO_WritePin(SCK_GPIO_Port, SCK_Pin, GPIO_PIN_SET);
		
		data[i] = HAL_GPIO_ReadPin(SO_GPIO_Port, SO_Pin);
		
		// SCK disable
		HAL_GPIO_WritePin(SCK_GPIO_Port, SCK_Pin, GPIO_PIN_RESET);
	}
	
	// CS disnable
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
	
	raw_data = data[3]*1024 + data[4]*512 + data[5]*256 + data[6]*128 + data[7]*64 + data[8]*32 + data[9]*16 + data[10]*8 + data[11]*4 + data[12]*2 + data[13];
	if (data[2] == 1) raw_data = 0 ;
	
	return raw_data;
}


// start button
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	/* Prevent unused argument(s) compilation warning */
	UNUSED(GPIO_Pin);
		if (GPIO_Pin==GPIO_PIN_5){
			if (Button_Status==BUTTON_RELEASED)
				Button_Status=BUTTON_PRESSED;
			else
				Button_Status=BUTTON_RELEASED;
		}
 /* NOTE: This function Should not be modified, when the callback is needed,
 the HAL_GPIO_EXTI_Callback could be implemented in the user file
 */
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void){

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
	HAL_GPIO_WritePin(RGB_LD4_GPIO_Port,RGB_LD4_Pin,GPIO_PIN_RESET); // test
	HAL_GPIO_WritePin(VCC_MAX6675_GPIO_Port, VCC_MAX6675_Pin, GPIO_PIN_SET); // will be turned on using 5V VCC later
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	
	
	
	
  while (1)
  {
		
		if (s_counter == 1000) m_counter = 0; // reset s_counter every second
		if (m_counter == 60000) m_counter = 0; // reset m_counter every minute
		if (h_counter == 3600000) h_counter = 0; // reset h_counter every hour
		
		// read temperature every second
		if (s_counter == 999) {

			temperature = read_temperature() / 4;			
		}
		
		// Start button was pressed
		if (Button_Status == BUTTON_PRESSED) {
			
			if (ignition_mode() == 0) {
				// ignition was succesfful entering another mode
			}
			if (ignition_mode() == 1) {
				// ignition failed
			}
			//HAL_GPIO_TogglePin(RGB_LD1_GPIO_Port,RGB_LD1_Pin); // toggle led
		}

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_5;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, VCC_MAX6675_Pin|SCK_Pin|RGB_LD1_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, CS_Pin|RGB_LD4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : VCC_MAX6675_Pin SCK_Pin RGB_LD4_Pin RGB_LD1_Pin */
  GPIO_InitStruct.Pin = VCC_MAX6675_Pin|SCK_Pin|RGB_LD4_Pin|RGB_LD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : CS_Pin */
  GPIO_InitStruct.Pin = CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SO_Pin RGB_LD3_Pin RGB_LD2_Pin */
  GPIO_InitStruct.Pin = SO_Pin|RGB_LD3_Pin|RGB_LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : Button_2_Kamami_Pin */
  GPIO_InitStruct.Pin = Button_2_Kamami_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(Button_2_Kamami_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

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
