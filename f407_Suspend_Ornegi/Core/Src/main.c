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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
	volatile uint8_t gorevSuspended;
	volatile uint32_t sonInterruptZamani;
}programDegiskenleri;

programDegiskenleri program;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define debounceSuresi  100
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* Definitions for BlinkGorev */
osThreadId_t BlinkGorevHandle;
const osThreadAttr_t BlinkGorev_attributes = {
  .name = "BlinkGorev",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for BlinkGorev2 */
osThreadId_t BlinkGorev2Handle;
const osThreadAttr_t BlinkGorev2_attributes = {
  .name = "BlinkGorev2",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
void BlinkGorevFonksiyonu(void *argument);
void BlinkGorev2Fonksiyon(void *argument);

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

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of BlinkGorev */
  BlinkGorevHandle = osThreadNew(BlinkGorevFonksiyonu, NULL, &BlinkGorev_attributes);

  /* creation of BlinkGorev2 */
  BlinkGorev2Handle = osThreadNew(BlinkGorev2Fonksiyon, NULL, &BlinkGorev2_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */

  program.gorevSuspended =0;
  /* USER CODE END RTOS_EVENTS */

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
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, Gorev1_Led1_Pin|Gorev1_Led2_Pin|Gorev2_Led1_Pin|Gorev2_Led2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Buton_Pin */
  GPIO_InitStruct.Pin = Buton_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(Buton_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Gorev1_Led1_Pin Gorev1_Led2_Pin Gorev2_Led1_Pin Gorev2_Led2_Pin */
  GPIO_InitStruct.Pin = Gorev1_Led1_Pin|Gorev1_Led2_Pin|Gorev2_Led1_Pin|Gorev2_Led2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_BlinkGorevFonksiyonu */
/**
  * @brief  Function implementing the BlinkGorev thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_BlinkGorevFonksiyonu */
void BlinkGorevFonksiyonu(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    HAL_GPIO_WritePin(Gorev1_Led1_GPIO_Port, Gorev1_Led1_Pin, 1);
    HAL_GPIO_WritePin(Gorev1_Led2_GPIO_Port, Gorev1_Led2_Pin, 1);
    vTaskDelay(1000/portTICK_PERIOD_MS);;//1000 ms
    HAL_GPIO_WritePin(Gorev1_Led1_GPIO_Port, Gorev1_Led1_Pin, 0);
    HAL_GPIO_WritePin(Gorev1_Led2_GPIO_Port, Gorev1_Led2_Pin, 0);
    vTaskDelay(1000/portTICK_PERIOD_MS);

  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_BlinkGorev2Fonksiyon */
/**
* @brief Function implementing the BlinkGorev2 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_BlinkGorev2Fonksiyon */
void BlinkGorev2Fonksiyon(void *argument)
{
  /* USER CODE BEGIN BlinkGorev2Fonksiyon */
  /* Infinite loop */
  for(;;)
  {
	  HAL_GPIO_WritePin(Gorev2_Led1_GPIO_Port, Gorev2_Led1_Pin, 1);
	  HAL_GPIO_WritePin(Gorev2_Led2_GPIO_Port, Gorev2_Led2_Pin, 1);
	  vTaskDelay(1000/portTICK_PERIOD_MS);;//1000 ms
	  HAL_GPIO_WritePin(Gorev2_Led1_GPIO_Port, Gorev2_Led1_Pin, 0);
	  HAL_GPIO_WritePin(Gorev2_Led2_GPIO_Port, Gorev2_Led2_Pin, 0);
	  vTaskDelay(1000/portTICK_PERIOD_MS);
  }
  /* USER CODE END BlinkGorev2Fonksiyon */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1)
  {
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
	  HAL_GPIO_TogglePin(Gorev1_Led1_GPIO_Port, Gorev1_Led1_Pin);
	  HAL_GPIO_TogglePin(Gorev1_Led2_GPIO_Port, Gorev1_Led2_Pin);
	  HAL_GPIO_TogglePin(Gorev2_Led1_GPIO_Port, Gorev2_Led1_Pin);
	  HAL_GPIO_TogglePin(Gorev2_Led2_GPIO_Port, Gorev2_Led2_Pin);
	  vTaskDelay(1000/portTICK_PERIOD_MS);


  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
