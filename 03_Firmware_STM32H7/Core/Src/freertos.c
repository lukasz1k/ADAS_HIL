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
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
typedef enum {
	ACC_ZONE_SAFE = 0U, /*  > 50m: green LED 	 */
	ACC_ZONE_WARNING = 1U, /*  25-50m: yellow LED  */
	ACC_ZONE_CRITICAL = 2U, /*  < 25m: red LED 	 */
	ACC_ZONE_FAULT = 3U
} AccZoneState_t;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

volatile uint32_t g_ulControlLoopCounter = 0U;
volatile uint32_t g_ulLastExecutionCycles = 0U;
volatile uint32_t g_ulMaxExecutionCycles = 0U;

volatile uint16_t g_usRadarDistanceCm = 10000U; /* Start: 10 000 cm = 100.00 m */
volatile AccZoneState_t g_eCurrentAccZone = ACC_ZONE_SAFE;
volatile float g_fDesiredAccelMps2 = 0.0f;

volatile uint8_t g_ucCanRxData[64] = { 0U };
volatile uint32_t g_ulCanRxFrameCounter = 0U;

volatile FDCAN_ProtocolStatusTypeDef g_sCanProtocolStatus = { 0 };
volatile FDCAN_ErrorCountersTypeDef g_sCanErrorCounters = { 0 };

volatile uint32_t g_ulCanRxTimeoutCounter = 0U;

volatile float g_fDistanceErrorMeters = 0.0f;

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

	const uint32_t kPeriodTicks = 250U; /* Okres pętli: 250 ms (częstotliwość 4 Hz) */
	const uint32_t kMaxRxTimeoutSteps = 4U; /* Timeout po 4 krokach (1000 ms bez ramki 0x200) */
	uint32_t ulWakeTick = osKernelGetTickCount();

	/* Infinite loop */
	for (;;) {
		/* Pomiar startu kroku obliczeniowego */
		uint32_t ulStartCycles = DWT->CYCCNT;

		/* --- POCZĄTEK SEKCJI KRYTYCZNEJ KROKU STEROWANIA --- */

		/* ====================================================================
		 * KROK 1: WEJŚCIA (RX) - Opróżnienie kolejki RX FIFO0 (ramka 0x200 z PC)
		 * ==================================================================== */

		/* Inkrementacja z nasyceniem (ochrona przed przekręceniem licznika uint32_t) */
		if (g_ulCanRxTimeoutCounter <= kMaxRxTimeoutSteps) {
			g_ulCanRxTimeoutCounter++;
		}

		while (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) > 0U) {
			FDCAN_RxHeaderTypeDef sRxHeader;
			static uint8_t ucTempRx[64];

			if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &sRxHeader,
					ucTempRx) == HAL_OK) {

				/* Deserializacja 16-bitowego dystansu (Little-Endian) z ramki 0x200 */
				if (sRxHeader.Identifier == 0x200U) {
					g_ulCanRxTimeoutCounter = 0U;
					g_usRadarDistanceCm = (uint16_t) (((uint16_t) ucTempRx[1]
							<< 8U) | (uint16_t) ucTempRx[0]);
				}

				/* Kopia diagnostyczna wszystkich 64 bajtów do Live Expressions */
				for (uint32_t i = 0U; i < 64U; i++) {
					g_ucCanRxData[i] = ucTempRx[i];
				}
				g_ulCanRxFrameCounter++;
			}
		}

		/* ====================================================================
		 * KROK 2: OBLICZENIA (COMPUTE) - Nadzór Fail-Safe, Regulator P i LED
		 * ==================================================================== */

		/* NADRZĘDNA KONTROLA BEZPIECZEŃSTWA (ISO 26262 Fail-Safe Override) */
		if (g_ulCanRxTimeoutCounter > kMaxRxTimeoutSteps) {
			/* Stan bezpieczny: odcięcie przyspieszenia i sygnalizacja awarii */
			g_eCurrentAccZone = ACC_ZONE_FAULT;
			g_fDistanceErrorMeters = 0.0f;
			g_fDesiredAccelMps2 = 0.0f;

			HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(LED_YELLOW_GPIO_Port, LED_YELLOW_Pin,
					GPIO_PIN_SET);
			HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
		} else {
			/* --- BLOK 2A: Ciągły regulator P z nasyceniem (Clamping) --- */
			const float kTargetDistanceCm = 4000.0f;
			const float kKpGain = 0.1f;
			const float kMaxAccelMps2 = 1.5f;
			const float kMinAccelMps2 = -3.5f;

			g_fDistanceErrorMeters = ((float) g_usRadarDistanceCm
					- kTargetDistanceCm) / 100.0f;
			float fRawAccel = kKpGain * g_fDistanceErrorMeters;

			if (fRawAccel > kMaxAccelMps2) {
				g_fDesiredAccelMps2 = kMaxAccelMps2;
			} else if (fRawAccel < kMinAccelMps2) {
				g_fDesiredAccelMps2 = kMinAccelMps2;
			} else {
				g_fDesiredAccelMps2 = fRawAccel;
			} /* <-- Koniec bloku nasycenia regulatora P! */

			/* --- BLOK 2B: Maszyna stanów stref ACC i sygnalizacja LED --- */
			if (g_usRadarDistanceCm > 5000U) {
				g_eCurrentAccZone = ACC_ZONE_SAFE;
				HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin,
						GPIO_PIN_SET);
				HAL_GPIO_WritePin(LED_YELLOW_GPIO_Port, LED_YELLOW_Pin,
						GPIO_PIN_RESET);
				HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin,
						GPIO_PIN_RESET);
			} else if (g_usRadarDistanceCm >= 2500U) {
				g_eCurrentAccZone = ACC_ZONE_WARNING;
				HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin,
						GPIO_PIN_RESET);
				HAL_GPIO_WritePin(LED_YELLOW_GPIO_Port, LED_YELLOW_Pin,
						GPIO_PIN_SET);
				HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin,
						GPIO_PIN_RESET);
			} else {
				g_eCurrentAccZone = ACC_ZONE_CRITICAL;
				HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin,
						GPIO_PIN_RESET);
				HAL_GPIO_WritePin(LED_YELLOW_GPIO_Port, LED_YELLOW_Pin,
						GPIO_PIN_RESET);
				HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
			}
		}

		/* ====================================================================
		 * KROK 3: WYJŚCIA (TX) - Serializacja i wysyłka ramki 64B CAN FD (0x100)
		 * ==================================================================== */
		static uint8_t ucTxData[64] = { 0U };

		/* Bajty 0..1: Dystans uint16_t w cm (Little-Endian: LSB, MSB) */
		ucTxData[0] = (uint8_t) (g_usRadarDistanceCm & 0xFFU);
		ucTxData[1] = (uint8_t) ((g_usRadarDistanceCm >> 8U) & 0xFFU);

		/* Bajt 2: Strefa bezpieczeństwa ACC (enum) */
		ucTxData[2] = (uint8_t) g_eCurrentAccZone;

		/* Bajt 3: Licznik życia (Alive Counter 0..255) */
		ucTxData[3] = (uint8_t) (g_ulControlLoopCounter & 0xFFU);

		/* Bajty 4..7: Zadane przyspieszenie float (IEEE-754 Little-Endian) */
		float fAccelTemp = g_fDesiredAccelMps2;
		(void) memcpy(&ucTxData[4], &fAccelTemp, sizeof(float));

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

