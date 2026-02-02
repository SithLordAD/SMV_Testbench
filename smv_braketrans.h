#ifndef _SMV_BRAKETRANS_H
#define _SMV_BRAKETRANS_H

#include <stdint.h>
#include "smv_board_enums.h"

/* ---- Calibration Constants ---- */
#define BRAKE_TRANS_SLOPE       125.0
#define BRAKE_TRANS_OFFSET      62.5

/* ---- Physical Limits ---- */
#define PSI_MAX					500.0
#define PSI_MIN					0.0

/* ---- HSMessage for SMV Can Communication ---- */
#define HSMESSAGE_PRESSURE      Pressure    /* HSMessage.Pressure from "smv_board_enums.h" */

double ADCtoPSI(double voltage)
{
	double psi = (BRAKE_TRANS_SLOPE * voltage) - BRAKE_TRANS_OFFSET;
	if (psi < PSI_MIN) psi = PSI_MIN;
	if (psi > PSI_MAX) psi = PSI_MAX;

	return psi;
}

#endif /* _SMV_BRAKETRANS_H */