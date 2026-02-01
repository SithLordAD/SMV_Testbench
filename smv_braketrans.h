#ifndef _SMV_BRAKETRANS_H
#define _SMV_BRAKETRANS_H

#include <stdint.h>
#include "smv_board_enums.h"

/* ---- Hardware Constants ---- */
#define BRAKE_TRANS_ADC_MAX     4095U
#define BRAKE_TRANS_VREF        3.3         /* STM32 ADC reference voltage */
#define BRAKE_TRANS_DIVIDER     2.0         /* Voltage divider ratio (sensor -> ADC) */
#define BRAKE_TRANS_VMAX        4.5         /* Sensor full-scale voltage */
#define BRAKE_TRANS_VMIN        0.5         /* Sensor zero-pressure voltage */

/* ---- Calibration Constants ---- */
#define BRAKE_TRANS_SLOPE       125.0
#define BRAKE_TRANS_OFFSET      62.5

/* ---- Physical Limits ---- */
#define PSI_MAX					500.0
#define PSI_MIN					0.0

/* ---- HSMessage for SMV Can Communication ---- */
#define HSMESSAGE_PRESSURE      Pressure    /* HSMessage.Pressure from "smv_board_enums.h" */

double ADCtoPSI(double adc)
{
	double v_sensor = (adc * BRAKE_TRANS_VREF * BRAKE_TRANS_DIVIDER) / BRAKE_TRANS_ADC_MAX;
	double psi = (BRAKE_TRANS_SLOPE * v_sensor) - BRAKE_TRANS_OFFSET;
	if (psi < PSI_MIN) psi = PSI_MIN;
	if (psi > PSI_MAX) psi = PSI_MAX;

	return psi;
}

#endif /* _SMV_BRAKETRANS_H */