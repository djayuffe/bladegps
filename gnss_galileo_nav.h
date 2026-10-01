#ifndef BLADEGPS_GNSS_GALILEO_NAV_H
#define BLADEGPS_GNSS_GALILEO_NAV_H

#include <stdint.h>

#include "gnss_nav.h"

#define GALILEO_INAV_WORD_BITS 128U
#define GALILEO_INAV_PAGE_PART_BITS 120U
#define GALILEO_INAV_CODED_SYMBOLS 240U
#define GALILEO_INAV_SYNC_SYMBOLS 10U
#define GALILEO_INAV_PAGE_PART_SYMBOLS 250U
#define GALILEO_INAV_OSNMA_BITS 40U
#define GALILEO_INAV_SAR_BITS 22U

typedef enum {
	GALILEO_INAV_SSP1 = 1,
	GALILEO_INAV_SSP2 = 2,
	GALILEO_INAV_SSP3 = 3
} galileo_inav_ssp_t;

typedef struct {
	uint16_t iodnav, toe;
	int32_t mean_anomaly;
	uint32_t eccentricity, sqrt_a;
} galileo_inav_word1_t;

typedef struct {
	uint16_t iodnav;
	int32_t omega0, inclination0, argument_of_perigee;
	int16_t inclination_rate;
} galileo_inav_word2_t;

typedef struct {
	uint16_t iodnav;
	int32_t omega_rate;
	int16_t delta_mean_motion, cuc, cus, crc, crs;
	uint8_t sisa;
} galileo_inav_word3_t;

typedef struct {
	uint16_t iodnav;
	uint8_t svid;
	int16_t cic, cis;
	uint16_t toc;
	int32_t af0, af1;
	int8_t af2;
} galileo_inav_word4_t;

typedef struct {
	uint16_t ai0;
	int16_t ai1, ai2;
	uint8_t disturbance_flags;
	int16_t bgd_e1e5a, bgd_e1e5b;
	uint8_t e5b_health, e1b_health, e5b_dvs, e1b_dvs;
	uint16_t week;
	uint32_t tow;
} galileo_inav_word5_t;

/* Values are already quantized in the ICD LSB units. Builders reject values
 * that cannot be represented instead of silently truncating them. */
int gnss_galileo_inav_word1(const galileo_inav_word1_t *fields,
	uint8_t word[GALILEO_INAV_WORD_BITS]);
int gnss_galileo_inav_word2(const galileo_inav_word2_t *fields,
	uint8_t word[GALILEO_INAV_WORD_BITS]);
int gnss_galileo_inav_word3(const galileo_inav_word3_t *fields,
	uint8_t word[GALILEO_INAV_WORD_BITS]);
int gnss_galileo_inav_word4(const galileo_inav_word4_t *fields,
	uint8_t word[GALILEO_INAV_WORD_BITS]);
int gnss_galileo_inav_word5(const galileo_inav_word5_t *fields,
	uint8_t word[GALILEO_INAV_WORD_BITS]);
int gnss_galileo_inav_ephemeris_words(const gnss_nav_record_t *record,
	uint8_t words[4][GALILEO_INAV_WORD_BITS]);

/* Build one nominal E1-B vertical page.  Inputs and outputs contain unpacked
 * bits/symbols (one 0 or 1 per byte), MSB/transmission order. */
int gnss_galileo_inav_e1b_page(const uint8_t word[GALILEO_INAV_WORD_BITS],
	const uint8_t osnma[GALILEO_INAV_OSNMA_BITS],
	const uint8_t sar[GALILEO_INAV_SAR_BITS], uint8_t spare,
	galileo_inav_ssp_t ssp,
	uint8_t even_symbols[GALILEO_INAV_PAGE_PART_SYMBOLS],
	uint8_t odd_symbols[GALILEO_INAV_PAGE_PART_SYMBOLS], uint32_t *crc);

/* The nominal E1-B word type broadcast at the given integer GST second within
 * a 30-second I/NAV subframe. Almanac slots are returned as 7..10; the caller
 * selects the satellite-specific almanac word for that slot. */
int gnss_galileo_inav_e1b_word_type(unsigned int gst_second_mod_30);
/* Return the E1-B secondary synchronization pattern for an absolute or
 * modulo-six GST second. Every unsigned input is accepted and wrapped by six. */
galileo_inav_ssp_t gnss_galileo_inav_ssp_for_second(unsigned int gst_second_mod_6);

#endif
