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
#include "fdcan.h"
#include "acc_controller.h"
#include <string.h>
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

/* Zmienne diagnostyczne czasu wykonania pętli RTOS (DWT WCET Profiling) */
volatile uint32_t g_ulControlLoopCounter = 0U;
volatile uint32_t g_ulLastExecutionCycles = 0U;
volatile uint32_t g_ulMaxExecutionCycles = 0U;

/* Bufory i liczniki diagnostyczne komunikacji CAN FD */
volatile uint8_t g_ucCanRxData[64] = { 0U };
volatile uint32_t g_ulCanRxFrameCounter = 0U;
volatile FDCAN_ProtocolStatusTypeDef g_sCanProtocolStatus = { 0 };
volatile FDCAN_ErrorCountersTypeDef g_sCanErrorCounters = { 0 };

/* Główne struktury wejść i wyjść regulatora ACC (obserwowane w Live Expressions) */
volatile AccInputs_t g_sAccInputs = { 10000U, 0, 2000U, 0U };
volatile AccOutputs_t g_sAccOutputs = { 35.0f, 0.0f, 0.0f, 0.0f, ACC_ZONE_SAFE };

/* USER CODE END Variables */
/* Definitions for ControlLoopTask */
osThreadId_t ControlLoopTaskHandle;
const osThreadAttr_t ControlLoopTask_attributes = { .name = "ControlLoopTask",
		.stack_size = 512 * 4, .priority = (osPriority_t) osPriorityNormal, };

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/**
 * @brief Funkcja warstwy sprzętowej (BSP/HAL) sterująca diodami LED na płytce Nucleo
 * @param eZone Aktualna strefa bezpieczeństwa wyznaczona przez regulator ACC
 */
static void ACC_UpdateLeds(AccZoneState_t eZone) {
	switch (eZone) {
	case ACC_ZONE_SAFE:
		HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LED_YELLOW_GPIO_Port, LED_YELLOW_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
		break;

	case ACC_ZONE_WARNING:
		HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LED_YELLOW_GPIO_Port, LED_YELLOW_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
		break;

	case ACC_ZONE_CRITICAL:
		HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LED_YELLOW_GPIO_Port, LED_YELLOW_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
		break;

	case ACC_ZONE_FAULT:
	default:
		HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LED_YELLOW_GPIO_Port, LED_YELLOW_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
		break;
	}
}

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
 * @brief  Główne zadanie czasu rzeczywistego realizujące pętlę HIL (Read -> Compute -> Write)
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartControlLoopTask */
void StartControlLoopTask(void *argument) {
	/* USER CODE BEGIN StartControlLoopTask */

	/* 1. Inicjalizacja sprzętowego licznika cykli zegara DWT (pomiar WCET) */
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CYCCNT = 0U;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

	/* 2. Konfiguracja nagłówka ramki nadawczej CAN FD (ID = 0x100, 64 bajty, BRS = ON) */
	FDCAN_TxHeaderTypeDef sTxHeader = { 0 };
	sTxHeader.Identifier = 0x100U;
	sTxHeader.IdType = FDCAN_STANDARD_ID;
	sTxHeader.TxFrameType = FDCAN_DATA_FRAME;
	sTxHeader.DataLength = FDCAN_DLC_BYTES_64;
	sTxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	sTxHeader.BitRateSwitch = FDCAN_BRS_ON;
	sTxHeader.FDFormat = FDCAN_FD_CAN;
	sTxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	sTxHeader.MessageMarker = 0U;

	/* 3. Konfiguracja filtra odbiorczego nr 0 (ID 0x000..0x7FF -> RX FIFO0) */
	FDCAN_FilterTypeDef sFilterConfig = { 0 };
	sFilterConfig.IdType = FDCAN_STANDARD_ID;
	sFilterConfig.FilterIndex = 0U;
	sFilterConfig.FilterType = FDCAN_FILTER_RANGE;
	sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	sFilterConfig.FilterID1 = 0x000U;
	sFilterConfig.FilterID2 = 0x7FFU;
	(void) HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig);

	/* 4. Uruchomienie kontrolera FDCAN1 w trybie Normal Mode */
	(void) HAL_FDCAN_Start(&hfdcan1);

	const uint32_t kPeriodTicks = ACC_CONTROL_PERIOD_MS; /* Okres pętli: 250 ms (częstotliwość 4 Hz) */
	uint32_t ulWakeTick = osKernelGetTickCount();

	/* Infinite loop */
	for (;;) {
		/* Pomiar startu kroku obliczeniowego */
		uint32_t ulStartCycles = DWT->CYCCNT;

		/* --- POCZĄTEK SEKCJI KRYTYCZNEJ KROKU STEROWANIA --- */

		/* ====================================================================
		 * KROK 1: WEJŚCIA (RX) - Nadzór Timeoutu i opróżnienie kolejki RX FIFO0
		 * ==================================================================== */

		/* Inkrementacja z nasyceniem (ochrona przed przekręceniem licznika uint32_t) */
		if (g_sAccInputs.ulRxTimeoutCounter <= ACC_MAX_RX_TIMEOUT_STEPS) {
			g_sAccInputs.ulRxTimeoutCounter++;
		}

		while (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) > 0U) {
			FDCAN_RxHeaderTypeDef sRxHeader;
			static uint8_t ucTempRx[64];

			if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &sRxHeader,
					ucTempRx) == HAL_OK) {

				/* Deserializacja sygnałów pomiarowych (Little-Endian) z ramki 0x200 */
				if (sRxHeader.Identifier == 0x200U) {
					g_sAccInputs.ulRxTimeoutCounter = 0U;

					/* Bajty 0..1: Dystans radaru uint16_t [cm] */
					g_sAccInputs.usRadarDistanceCm =
							(uint16_t) (((uint16_t) ucTempRx[1] << 8U)
									| (uint16_t) ucTempRx[0]);

					/* Bajty 2..3: Prędkość względna int16_t [cm/s] (kod U2) */
					g_sAccInputs.sRelativeVelocityCms =
							(int16_t) (((uint16_t) ucTempRx[3] << 8U)
									| (uint16_t) ucTempRx[2]);

					/* Bajty 4..5: Prędkość własna pojazdu uint16_t [cm/s] */
					g_sAccInputs.usEgoVelocityCms =
							(uint16_t) (((uint16_t) ucTempRx[5] << 8U)
									| (uint16_t) ucTempRx[4]);
				}

				/* Kopia diagnostyczna wszystkich 64 bajtów do Live Expressions */
				for (uint32_t i = 0U; i < 64U; i++) {
					g_ucCanRxData[i] = ucTempRx[i];
				}
				g_ulCanRxFrameCounter++;
			}
		}

		/* ====================================================================
		 * KROK 2: OBLICZENIA (COMPUTE) - Wywołanie modułu regulatora ACC
		 * ==================================================================== */
		ACC_ComputeStep(&g_sAccInputs, &g_sAccOutputs);

		/* ====================================================================
		 * KROK 3: WYJŚCIA (TX & GPIO) - Sygnalizacja LED i wysyłka ramki 0x100
		 * ==================================================================== */
		ACC_UpdateLeds(g_sAccOutputs.eCurrentZone);

		static uint8_t ucTxData[64] = { 0U };

		/* Bajty 0..1: Aktualny dystans uint16_t w cm (Little-Endian: LSB, MSB) */
		ucTxData[0] = (uint8_t) (g_sAccInputs.usRadarDistanceCm & 0xFFU);
		ucTxData[1] =
				(uint8_t) ((g_sAccInputs.usRadarDistanceCm >> 8U) & 0xFFU);

		/* Bajt 2: Strefa bezpieczeństwa ACC (enum: 0..3) */
		ucTxData[2] = (uint8_t) g_sAccOutputs.eCurrentZone;

		/* Bajt 3: Licznik życia (Alive Counter 0..255) */
		ucTxData[3] = (uint8_t) (g_ulControlLoopCounter & 0xFFU);

		/* Bajty 4..7: Zadane przyspieszenie po ograniczniku zrywu float [m/s^2] */
		float fAccelTemp = g_sAccOutputs.fDesiredAccelMps2;
		(void) memcpy(&ucTxData[4], &fAccelTemp, sizeof(float));

		/* Bajty 8..11: Dynamiczny dystans zadany CTH float [m] */
		float fTargetDistTemp = g_sAccOutputs.fTargetDistanceMeters;
		(void) memcpy(&ucTxData[8], &fTargetDistTemp, sizeof(float));

		/* Bajty 62..63: Znacznik kontrolny końca 64-bajtowej ramki CAN FD */
		ucTxData[62] = 0xCAU;
		ucTxData[63] = 0xFEU;

		(void) HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &sTxHeader, ucTxData);

		/* Diagnostyka statusu protokołu i liczników błędów FDCAN */
		FDCAN_ProtocolStatusTypeDef sTempStatus;
		FDCAN_ErrorCountersTypeDef sTempErrors;
		if (HAL_FDCAN_GetProtocolStatus(&hfdcan1, &sTempStatus) == HAL_OK) {
			g_sCanProtocolStatus = sTempStatus;
		}
		if (HAL_FDCAN_GetErrorCounters(&hfdcan1, &sTempErrors) == HAL_OK) {
			g_sCanErrorCounters = sTempErrors;
		}

		g_ulControlLoopCounter++;

		/* --- KONIEC SEKCJI KRYTYCZNEJ KROKU STEROWANIA --- */

		/* Obliczenie liczby cykli zegara zużytych na wykonanie kroku i detekcja WCET */
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
