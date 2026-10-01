#ifndef BLADEGPS_GNSS_GLONASS_NAV_H
#define BLADEGPS_GNSS_GLONASS_NAV_H

#include <stdint.h>

#include "gnss_nav.h"

#define GLONASS_GNAV_STRING_BITS 85U
#define GLONASS_GNAV_FRAME_STRINGS 15U

typedef struct {
	uint32_t tk_seconds;
	uint8_t tb, bn, p1, p2, p3, p4, p, ln, ft, en, slot, mode;
	uint16_t nt;
	int32_t position[3], velocity[3], acceleration[3];
	int32_t gamma, tau, delta_tau;
} glonass_gnav_immediate_t;

typedef struct {
	uint16_t na;
	uint8_t n4, ln;
	int32_t tau_c, tau_gps;
} glonass_gnav_string5_t;

typedef struct {
	uint8_t slot, satellite_type, healthy, frequency, ln;
	int32_t tau, lambda, delta_i, delta_t, delta_t_rate, omega;
	uint32_t eccentricity, ascending_time;
} glonass_gnav_almanac_t;

/* Build GNAV strings 1..3. Values are quantized in the ICD LSB units and
 * signed quantities use the GLONASS sign-magnitude representation. */
int gnss_glonass_gnav_immediate_strings(const glonass_gnav_immediate_t *fields,
	uint8_t strings[4][GLONASS_GNAV_STRING_BITS]);
int gnss_glonass_gnav_from_rinex(const gnss_nav_record_t *record,
	glonass_gnav_immediate_t *fields);
int gnss_glonass_gnav_string5(const glonass_gnav_string5_t *fields,
	uint8_t string[GLONASS_GNAV_STRING_BITS]);
int gnss_glonass_gnav_almanac_pair(const glonass_gnav_almanac_t *fields,
	unsigned int even_string_number,
	uint8_t even_string[GLONASS_GNAV_STRING_BITS],
	uint8_t odd_string[GLONASS_GNAV_STRING_BITS]);
int gnss_glonass_gnav_frame(const glonass_gnav_immediate_t *immediate,
	const glonass_gnav_string5_t *time_data,
	const glonass_gnav_almanac_t almanacs[5],
	uint8_t frame[GLONASS_GNAV_FRAME_STRINGS][GLONASS_GNAV_STRING_BITS]);

#endif
