/**
 ******************************************************************************
 * @file           : acc_controller.c
 * @brief          : Implementacja prawa regulacji ACC (ISO 15622 & ISO 26262)
 ******************************************************************************
 */

#include "acc_controller.h"

void ACC_ComputeStep(const volatile AccInputs_t *pIn,
		volatile AccOutputs_t *pOut) {
	/* ====================================================================
	 * NADRZĘDNA KONTROLA BEZPIECZEŃSTWA (ISO 26262 Fail-Safe Override)
	 * ==================================================================== */
	if (pIn->ulRxTimeoutCounter > ACC_MAX_RX_TIMEOUT_STEPS) {
		/* Stan bezpieczny: odcięcie przyspieszenia, wyzerowanie uchybu i flaga awarii */
		pOut->eCurrentZone = ACC_ZONE_FAULT;
		pOut->fDistanceErrorMeters = 0.0f;
		pOut->fTargetAccelMps2 = 0.0f;
		pOut->fDesiredAccelMps2 = 0.0f;
	} else {
		/* ================================================================
		 * BLOK 2A: Polityka CTH, ciągły regulator PD, nasycenie i Jerk Limiter
		 * ================================================================ */
		const float kStandStillDistMeters = 5.0f; /* Minimalny odstęp postoju d_min [m] */
		const float kTimeHeadwaySec = 1.5f;       /* Stały odstęp czasowy t_hw [s] */
		const float kKpDistanceGain = 0.4f;       /* Wzmocnienie członu uchybu odległości K_p */
		const float kKvVelocityGain = 0.8f;       /* Wzmocnienie członu prędkości względnej K_v */
		const float kMaxAccelMps2 = 1.5f;         /* Górne nasycenie przyspieszenia [m/s^2] */
		const float kMinAccelMps2 = -3.5f;        /* Dolne nasycenie hamowania [m/s^2] */
		const float kMaxAccelStepMps2 = 1.0f;     /* Limit zrywu: max 0.5 m/s^2 na krok 250 ms (2.0 m/s^3) */

		/* Konwersja jednostek stałoprzecinkowych z CAN (cm, cm/s) na układ SI (m, m/s) */
		float fMeasDistMeters = (float) pIn->usRadarDistanceCm / 100.0f;
		float fRelVelMps = (float) pIn->sRelativeVelocityCms / 100.0f;
		float fEgoVelMps = (float) pIn->usEgoVelocityCms / 100.0f;

		/* 1. Wyznaczenie dynamicznego dystansu zadanego według polityki CTH */
		pOut->fTargetDistanceMeters = kStandStillDistMeters
				+ (kTimeHeadwaySec * fEgoVelMps);

		/* 2. Obliczenie uchybu regulacji odległości */
		pOut->fDistanceErrorMeters = fMeasDistMeters
				- pOut->fTargetDistanceMeters;

		/* 3. Dwuwejściowe prawo regulacji PD (uchyb odległości + prędkość względna) */
		float fRawAccel = (kKpDistanceGain * pOut->fDistanceErrorMeters)
				+ (kKvVelocityGain * fRelVelMps);

		/* 4. Nasycenie sygnału sterującego (Actuator Saturation / Clamping) */
		if (fRawAccel > kMaxAccelMps2) {
			pOut->fTargetAccelMps2 = kMaxAccelMps2;
		} else if (fRawAccel < kMinAccelMps2) {
			pOut->fTargetAccelMps2 = kMinAccelMps2;
		} else {
			pOut->fTargetAccelMps2 = fRawAccel;
		}

		/* 5. Dyskretny ogranicznik zrywu (ISO 15622 Jerk / Slew-Rate Limiter z pamięcią k-1) */
		float fAccelDelta = pOut->fTargetAccelMps2 - pOut->fDesiredAccelMps2;

		if (fAccelDelta > kMaxAccelStepMps2) {
			pOut->fDesiredAccelMps2 += kMaxAccelStepMps2;
		} else if (fAccelDelta < -kMaxAccelStepMps2) {
			pOut->fDesiredAccelMps2 -= kMaxAccelStepMps2;
		} else {
			pOut->fDesiredAccelMps2 = pOut->fTargetAccelMps2;
		}

		/* ================================================================
		 * BLOK 2B: Dynamiczna maszyna stanów stref bezpieczeństwa ACC
		 * ================================================================ */
		if ((pOut->fDistanceErrorMeters <= -10.0f)
				|| (pIn->usRadarDistanceCm < 300U)) {
			pOut->eCurrentZone = ACC_ZONE_CRITICAL;
		} else if (pOut->fDistanceErrorMeters <= -2.0f) {
			pOut->eCurrentZone = ACC_ZONE_WARNING;
		} else {
			pOut->eCurrentZone = ACC_ZONE_SAFE;
		}
	}
}
