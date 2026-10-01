#ifndef BLADEGPS_GNSS_SCHEDULE_H
#define BLADEGPS_GNSS_SCHEDULE_H

#include <stdint.h>

#include "gnss_nav.h"

#define GALILEO_E1_CYCLE_SYMBOLS 7500U
#define BEIDOU_D1_FRAME_SYMBOLS 1500U
#define BEIDOU_D2_CYCLE_SYMBOLS 15000U
#define GLONASS_GNAV_FRAME_SYMBOLS 3000U

int gnss_schedule_galileo_e1(const gnss_nav_record_t *record,
	unsigned int gst_week, uint32_t frame_tow, int8_t symbols[GALILEO_E1_CYCLE_SYMBOLS]);
int gnss_schedule_beidou_d1(const gnss_nav_record_t *record,
	const int8_t alpha[4], const int8_t beta[4], uint32_t frame_sow,
	int8_t symbols[BEIDOU_D1_FRAME_SYMBOLS]);
int gnss_schedule_beidou_d2(const gnss_nav_record_t *record,
	const int8_t alpha[4], const int8_t beta[4], uint32_t frame_sow,
	int8_t symbols[BEIDOU_D2_CYCLE_SYMBOLS]);
int gnss_schedule_glonass(const gnss_nav_record_t *record,
	int8_t symbols[GLONASS_GNAV_FRAME_SYMBOLS]);

#endif
