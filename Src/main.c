/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "SensorReport.h"
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
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_USART2_UART_Init();
#define CPU_CORE_FREQUENCY_HZ 64000000
  SWO_Init(0x1, CPU_CORE_FREQUENCY_HZ);
  SWO_PrintString("hello world with SWO\r\n", 0);

//			ITM_SendChar('a');

  /* USER CODE BEGIN 2 */
	initializeSensorReport();

	HAL_TIM_Base_Start_IT(&htim1); // will call the PeriodElapsed IT
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  doReport();
	  HAL_Delay(1000);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_TIM1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/*!
 * \brief Sends a character over the SWO channel
 * \param c Character to be sent
 * \param portNo SWO channel number, value in the range of 0 to 31
 */
void SWO_PrintChar(char c, uint8_t portNo)
{
	volatile int timeout;
 
	/* Check if Trace Control Register (ITM->TCR at 0xE0000E80) is set */
	if ((ITM->TCR&ITM_TCR_ITMENA_Msk) == 0) {
		/* check Trace Control Register if ITM trace is enabled*/
		return; /* not enabled? */
	}
	/* Check if the requested channel stimulus port (ITM->TER at 0xE0000E00) is enabled */
	if ((ITM->TER & (1ul << portNo)) == 0) {
		/* check Trace Enable Register if requested port is enabled */
		return; /* requested port not enabled? */
	}
	timeout = 5000; /* arbitrary timeout value */
	while (ITM->PORT[0].u32 == 0) {
		/* Wait until STIMx is ready, then send data */
		timeout--;
		if (timeout == 0) {
			return; /* not able to send */
		}
	}
	ITM->PORT[0].u16 = 0x08 | (c << 8);
}
/*!
 * \brief Sends a string over SWO to the host
 * \param s String to send
 * \param portNumber Port number, 0-31, use 0 for normal debug strings
 */
void SWO_PrintString(const char *s, uint8_t portNumber)
{
	while (*s != '\0') {
		SWO_PrintChar(*s++, portNumber);
	}
}
/*!
 * \brief Initialize the SWO trace port for debug message printing
 * \param portBits Port bit mask to be configured
 * \param cpuCoreFreqHz CPU core clock frequency in Hz
 */
void SWO_Init(uint32_t portBits, uint32_t cpuCoreFreqHz)
{
	uint32_t SWOSpeed = 64000; /* default 64k baud rate */
	uint32_t SWOPrescaler = (cpuCoreFreqHz / SWOSpeed) - 1; /* SWOSpeed in Hz, note that cpuCoreFreqHz is expected to be match the CPU core clock */
 
	CoreDebug->DEMCR = CoreDebug_DEMCR_TRCENA_Msk; /* enable trace in core debug */
	*((volatile unsigned *)(ITM_BASE + 0x400F0)) = 0x00000002; /* "Selected PIN Protocol Register": Select which protocol to use for trace output (2: SWO NRZ, 1: SWO Manchester encoding) */
	*((volatile unsigned *)(ITM_BASE + 0x40010)) = SWOPrescaler; /* "Async Clock Prescaler Register". Scale the baud rate of the asynchronous output */
	*((volatile unsigned *)(ITM_BASE + 0x00FB0)) = 0xC5ACCE55; /* ITM Lock Access Register, C5ACCE55 enables more write access to Control Register 0xE00 :: 0xFFC */
	ITM->TCR = ITM_TCR_TraceBusID_Msk | ITM_TCR_SWOENA_Msk | ITM_TCR_SYNCENA_Msk | ITM_TCR_ITMENA_Msk; /* ITM Trace Control Register */
	ITM->TPR = ITM_TPR_PRIVMASK_Msk; /* ITM Trace Privilege Register */
	ITM->TER = portBits; /* ITM Trace Enable Register. Enabled tracing on stimulus ports. One bit per stimulus port. */
	*((volatile unsigned *)(ITM_BASE + 0x01000)) = 0x400003FE; /* DWT_CTRL */
	*((volatile unsigned *)(ITM_BASE + 0x40304)) = 0x00000100; /* Formatter and Flush Control Register */
}

void delay_us_DWT(unsigned long  uSec)
{
	volatile uint32_t cycles = (SystemCoreClock / 1000000L)*uSec;
	volatile uint32_t start = DWT->CYCCNT;
	do {
	} while (DWT->CYCCNT - start < cycles);
}

#define SLOTS 4
static volatile uint32_t start[4];

void start_us_DWT(int slot)
{
	if (slot < SLOTS) {
		start[slot] = DWT->CYCCNT;
	}
}

unsigned int get_us_DWT(int slot)
{
	if (slot < SLOTS) {
		unsigned long long val;
		if (DWT->CYCCNT < start[slot]) {
			val = (DWT->CYCCNT - start[slot]) + (1LL << 32);
		} else {
			val = (DWT->CYCCNT - start[slot]);
		}
		return val * 1000000L / SystemCoreClock;
	}
	return 0xFFFFFFFF;
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
