#ifndef BLADEGPS_GNSS_GLONASS_NAV_H
#define BLADEGPS_GNSS_GLONASS_NAV_H

#include <stdint.h>

#include "gnss_nav.h"

#define GLONASS_GNAV_STRING_BITS 85U

typedef struct {
	uint32_t tk_seconds;
	uint8_t tb, bn, p1, p2, p3, p4, p, ln, ft, en, slot, mode;
	uint16_t nt;
	int32_t position[3], velocity[3], acceleration[3];
	int32_t gamma, tau, delta_tau;
} glonass_gnav_immediate_t;

/* Build GNAV strings 1..3. Values are quantized in the ICD LSB units and
 * signed quantities use the GLONASS sign-magnitude representation. */
int gnss_glonass_gnav_immediate_strings(const glonass_gnav_immediate_t *fields,
	uint8_t strings[4][GLONASS_GNAV_STRING_BITS]);
int gnss_glonass_gnav_from_rinex(const gnss_nav_record_t *record,
	glonass_gnav_immediate_t *fields);

#endif
