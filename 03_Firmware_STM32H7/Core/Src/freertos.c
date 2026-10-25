/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
/* USER CODE BEGIN Variables */

typedef enum {
	ACC_ZONE_SAFE = 0U, /*  > 50m: green LED 	*/
	ACC_ZONE_WARNING, /*  25-50m: yellow LED  */
	ACC_ZONE_CRITICAL /*  < 25m: red LED 		*/
} AccZoneState_t;

volatile AccZoneState_t g_eCurrentAccZone = ACC_ZONE_SAFE;

/* Diagnostic variables to observe in Live Expressions (SWD) */
volatile uint32_t g_ulControlLoopCounter = 0U;
volatile uint32_t g_ulLastExecutionCycles = 0U;
volatile uint32_t g_ulMaxExecutionCycles = 0U;

volatile uint32_t g_ulRadarDistanceMeters = 100U;

/* USER CODE END Variables */
/* Definitions for ControlLoopTask */
osThreadId_t ControlLoopTaskHandle;
const osThreadAttr_t ControlLoopTask_attributes = { .name = "ControlLoopTask",
		.stack_size = 512 * 4, .priority = (osPriority_t) osPriorityNormal, };

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartControlLoopTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
 * @brief  FreeRTOS initialization
 * @param  None
 * @retval None
 */
void MX_FREERTOS_Init(void) {
	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

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
	/* creation of ControlLoopTask */
	ControlLoopTaskHandle = osThreadNew(StartControlLoopTask, NULL,
			&ControlLoopTask_attributes);

	/* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
	/* USER CODE END RTOS_THREADS */

	/* USER CODE BEGIN RTOS_EVENTS */
	/* add events, ... */
	/* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartControlLoopTask */
/**
 * @brief  Function implementing the ControlLoopTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartControlLoopTask */
void StartControlLoopTask(void *argument) {
	/* USER CODE BEGIN StartControlLoopTask */
	/* 1. Inicjalizacja sprzętowego licznika cykli rdzenia DWT (Data Watchpoint and Trace)
	 *  Pozwala mierzyć czas wykonania kodu z rozdzielczością 1 cyklu zegara (2.5 ns @ 400 MHz) */
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CYCCNT = 0U;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

	const uint32_t kPeriodTicks = 250U; /* Okres pętli: 250 ms (częstotliwość 4 Hz dla testu LED) */
	uint32_t ulWakeTick = osKernelGetTickCount();
	/* Infinite loop */
	for (;;) {
		/* Pomiar startu kroku obliczeniowego */
		uint32_t ulStartCycles = DWT->CYCCNT;

		/* --- POCZĄTEK SEKCJI KRYTYCZNEJ KROKU STEROWANIA --- */

		if (g_ulRadarDistanceMeters == 0U) {
			g_ulRadarDistanceMeters = 100U;
		} else {
			g_ulRadarDistanceMeters -= 2U;
		}

		/* Logika LED */
		if (g_ulRadarDistanceMeters > 50U) {
			g_eCurrentAccZone = ACC_ZONE_SAFE;
			HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(LED_YELLOW_GPIO_Port, LED_YELLOW_Pin,
					GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
		} else if (g_ulRadarDistanceMeters >= 25U) {
			g_eCurrentAccZone = ACC_ZONE_WARNING;
			HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin,
					GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LED_YELLOW_GPIO_Port, LED_YELLOW_Pin,
					GPIO_PIN_SET);
			HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
		} else {
			g_eCurrentAccZone = ACC_ZONE_CRITICAL;
			HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin,
					GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LED_YELLOW_GPIO_Port, LED_YELLOW_Pin,
					GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
		}

		g_ulControlLoopCounter++;

		/* --- KONIEC SEKCJI KRYTYCZNEJ KROKU STEROWANIA --- */

		/* Obliczenie liczby cykli zegara zużytych na wykonanie kroku */
		g_ulLastExecutionCycles = DWT->CYCCNT - ulStartCycles;

		if (g_ulLastExecutionCycles > g_ulMaxExecutionCycles) {
			g_ulMaxExecutionCycles = g_ulLastExecutionCycles;
		}

		/* Bezwzględne uśpienie do kolejnego wielokrotnego punktu czasu kPeriodTicks */
		ulWakeTick += kPeriodTicks;
		osDelayUntil(ulWakeTick);
	}
	/* USER CODE END StartControlLoopTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

