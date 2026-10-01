#ifndef BLADEGPS_GNSS_GEOMETRY_H
#define BLADEGPS_GNSS_GEOMETRY_H

#include "gnss_nav.h"

typedef struct {
	double position[3], velocity[3];
	double geometric_range_m, pseudorange_m, range_rate_mps;
	double azimuth_rad, elevation_rad, clock_bias_s, clock_drift_sps;
	double doppler_hz, code_phase_chips, carrier_phase_rad;
} gnss_observation_t;

int gnss_observe(const gnss_nav_record_t *record, double receive_sow,
	const double receiver_position[3], const double receiver_velocity[3],
	double carrier_hz, double code_rate_hz, unsigned int code_length,
	gnss_observation_t *observation);

#endif
