/**
 ******************************************************************************
 * @file           : acc_controller.h
 * @brief          : Interfejs publiczny modułu regulatora ACC (CTH + PD + Jerk)
 ******************************************************************************
 */

#ifndef INC_ACC_CONTROLLER_H_
#define INC_ACC_CONTROLLER_H_

#include <stdint.h>

/* Maksymalna liczba kroków pętli bez ramki CAN przed wejściem w Fail-Safe (10 * 100 ms = 1.0 s) */
/* JEDYNE ŹRÓDŁO PRAWDY: Okres pętli sterowania ACC w milisekundach (100 ms = 10 Hz) */
#define ACC_CONTROL_PERIOD_MS     (100U)
#define ACC_MAX_RX_TIMEOUT_STEPS  (1000U / ACC_CONTROL_PERIOD_MS) /* Zawsze 1.0 s FTTI! */

/**
 * @brief Strefy bezpieczeństwa tempomatu adaptacyjnego ACC (zgodne z ISO 26262)
 */
typedef enum {
	ACC_ZONE_SAFE = 0U,     /* Strefa bezpieczna (dioda zielona) */
	ACC_ZONE_WARNING = 1U,  /* Strefa ostrzegawcza (dioda żółta) */
	ACC_ZONE_CRITICAL = 2U, /* Strefa krytyczna (dioda czerwona) */
	ACC_ZONE_FAULT = 3U     /* Awaria komunikacji CAN - stan bezpieczny Fail-Safe (3 diody) */
} AccZoneState_t;

/**
 * @brief Struktura wejść regulatora ACC (dane pomiarowe z magistrali CAN)
 */
typedef struct {
	uint16_t usRadarDistanceCm;   /* Zmierzony dystans od poprzednika [cm] */
	int16_t sRelativeVelocityCms; /* Prędkość względna: v_lead - v_ego [cm/s] (kod U2) */
	uint16_t usEgoVelocityCms;    /* Prędkość własna pojazdu v_ego [cm/s] */
	uint32_t ulRxTimeoutCounter;  /* Licznik kroków od ostatniej odebranej ramki 0x200 */
} AccInputs_t;

/**
 * @brief Struktura wyjść i stanu wewnętrznego regulatora ACC
 */
typedef struct {
	float fTargetDistanceMeters; /* Dynamiczny dystans zadany z polityki CTH [m] */
	float fDistanceErrorMeters;  /* Uchyb odległości: d_meas - d_target [m] */
	float fTargetAccelMps2;      /* Przyspieszenie docelowe z prawa PD po nasyceniu [m/s^2] */
	float fDesiredAccelMps2;     /* Przyspieszenie wyjściowe po ograniczniku zrywu (pamięć k-1) [m/s^2] */
	AccZoneState_t eCurrentZone;  /* Wyznaczona strefa bezpieczeństwa ACC */
} AccOutputs_t;

/**
 * @brief Wykonuje pojedynczy dyskretny krok obliczeniowy regulatora ACC (T_s = 250 ms)
 * @param pIn  Wskaźnik (tylko do odczytu) na strukturę wejść pomiarowych
 * @param pOut Wskaźnik na strukturę wyjść oraz pamięci stanu regulatora
 */
void ACC_ComputeStep(const volatile AccInputs_t *pIn, volatile AccOutputs_t *pOut);

#endif /* INC_ACC_CONTROLLER_H_ */
