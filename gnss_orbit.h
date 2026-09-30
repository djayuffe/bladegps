#ifndef BLADEGPS_GNSS_ORBIT_H
#define BLADEGPS_GNSS_ORBIT_H

#include "gnss_nav.h"

int gnss_propagate_kepler(const gnss_nav_record_t *record, double transmit_sow,
	double position[3], double velocity[3], double *clock_bias,
	double *clock_drift);
int gnss_propagate_glonass(const gnss_nav_record_t *record, double delta_seconds,
	double position[3], double velocity[3], double *clock_bias,
	double *clock_drift);

#endif
