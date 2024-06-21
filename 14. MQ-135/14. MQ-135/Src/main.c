/**
  ******************************************************************************
	*
  * File Name         	: main.c
  * Description        	: Main program body
	* Experimental 				: MQ-135 Gas Sensor                                 	*
	* Connect      				: USART 1, PA9(TX)- RX TTL, PA10(RX)-TX TTL
  * 115200 BAUD, 8 Data Bits, None Parity, 1 Stop Bit, None Flow Control
	
	* MQ-7 DOUT	- PA7
	* MQ-7 AOUT	- PA6
	
  *******************************************************************************/
	
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "adc.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"
#include <math.h>
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
	uint16_t ADC_Value[100];
	/* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();

  /* USER CODE BEGIN 2 */
	/*##-1- Start the conversion process and enable interrupt ##################*/  
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&ADC_Value, 100);

  /* USER CODE END 2 */
	double AD_value;
  /* Infinite loop */
  /* USEdouble AD_value;R CODE BEGIN WHILE */
	
	
  while (1)
  {
  /* USER CODE END WHILE */

		HAL_ADC_Start(&hadc1);
		AD_value = HAL_ADC_GetValue(&hadc1);
		if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7) == GPIO_PIN_SET)
		{
			//HAL_GPIO_WritePin(GPIOG,GPIO_PIN_13,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin,GPIO_PIN_RESET );
			printf("MQ-135 Air quality not leakage\r\n");
			
		}
		else
		{
			//HAL_GPIO_WritePin(GPIOG,GPIO_PIN_13,GPIO_PIN_SET);
			HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin,GPIO_PIN_SET );
			printf("MQ-135 Air quality leakage\r\n");	
		
    /* USER CODE BEGIN 3 */
  }
			float voltage = (AD_value / 4096.0) * 3;
			float RL = 1000; // 1000 Ohm
			float Ro = 830; // 830 ohm 
			float Rs = ( 3.00 * 1000 / voltage ) - 1000;
			float ppm = 100 / pow(Rs/Ro, 1.53);  // ppm = 100 * ((rs/ro)^-1.53);
						
			printf("MQ-135 ADC voltage = %2fV \r\n", voltage);		
			printf("MQ-135 Air Quality = %2f ppm\r\n", ppm);
			HAL_Delay(2000);   /* delay 2000ms */
			}
	
}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  __PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 210;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1
                              |RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

#ifdef USE_FULL_ASSERT

#endif
