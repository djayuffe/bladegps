#ifndef BLADEGPS_GNSS_BEIDOU_NAV_H
#define BLADEGPS_GNSS_BEIDOU_NAV_H

#include <stdint.h>

#include "gnss_nav.h"

#define BEIDOU_NAV_SUBFRAME_BITS 300U
#define BEIDOU_NAV_INFORMATION_BITS 224U
#define BEIDOU_NAV_PAYLOAD_BITS 186U

typedef struct {
	uint32_t toe;
	int32_t delta_mean_motion, cuc, mean_anomaly, cus, crc, crs;
	uint32_t eccentricity, sqrt_a;
	int32_t inclination0, cic, omega_rate, cis, inclination_rate;
	int32_t omega0, argument_of_perigee;
} beidou_d1_ephemeris_t;

typedef struct {
	uint8_t health, aodc, urai, aode;
	uint16_t week;
	uint32_t toc;
	int16_t tgd1, tgd2;
	int8_t alpha[4], beta[4];
	int32_t af0, af1, af2;
} beidou_d1_clock_t;

typedef enum {
	BEIDOU_NAV_D1 = 1,
	BEIDOU_NAV_D2 = 2
} beidou_nav_format_t;

typedef struct {
	uint32_t sqrt_a;
	int16_t clock_rate, clock_bias;
	int32_t omega0;
	uint32_t eccentricity;
	int16_t inclination_offset;
	uint8_t toa;
	int32_t omega_rate, argument_of_perigee, mean_anomaly;
	uint8_t identifier;
} beidou_almanac_t;

/* D1 and D2 use the same 10-word BCH/interleaving structure. Inputs and output
 * are unpacked bits in MSB/transmission order. */
int gnss_beidou_nav_encode_subframe(
	const uint8_t information[BEIDOU_NAV_INFORMATION_BITS],
	uint8_t subframe[BEIDOU_NAV_SUBFRAME_BITS]);

/* Insert the ICD preamble, reserved header, FraID and BDT seconds-of-week,
 * then encode the caller-supplied constellation data payload. */
int gnss_beidou_nav_build_subframe(unsigned int fraid, uint32_t sow,
	const uint8_t payload[BEIDOU_NAV_PAYLOAD_BITS],
	uint8_t subframe[BEIDOU_NAV_SUBFRAME_BITS]);

/* Build D1 ephemeris subframes 2 and 3. frame_sow is the start of subframe 1;
 * the generated headers use frame_sow+6 and frame_sow+12 modulo one BDT week. */
int gnss_beidou_d1_ephemeris_subframes(const beidou_d1_ephemeris_t *fields,
	uint32_t frame_sow, uint8_t subframe2[BEIDOU_NAV_SUBFRAME_BITS],
	uint8_t subframe3[BEIDOU_NAV_SUBFRAME_BITS]);
/* D1 and D2 broadcast identical ephemeris/clock parameter scales; these
 * converters accept either RINEX message type. */
int gnss_beidou_d1_ephemeris_from_rinex(const gnss_nav_record_t *record,
	beidou_d1_ephemeris_t *fields);
int gnss_beidou_d1_clock_subframe(const beidou_d1_clock_t *fields,
	uint32_t frame_sow, uint8_t subframe1[BEIDOU_NAV_SUBFRAME_BITS]);
int gnss_beidou_d1_clock_from_rinex(const gnss_nav_record_t *record,
	const int8_t alpha[4], const int8_t beta[4], beidou_d1_clock_t *fields);
int gnss_beidou_ionosphere_quantize(const gnss_klobuchar_t *model,
	int8_t alpha[4], int8_t beta[4]);

/* Build an almanac page using the common ICD payload used by D1 and D2.
 * D1 permits subframe 4 pages 1-24 and subframe 5 pages 1-6/11-23.
 * D2 permits subframe 5 pages 37-60, 95-100 and 103-115. `identifier`
 * carries the page's two-bit AmEpID/AmID value required by its schedule. */
int gnss_beidou_almanac_subframe(beidou_nav_format_t format,
	unsigned int fraid, unsigned int page, uint32_t sow,
	const beidou_almanac_t *fields,
	uint8_t subframe[BEIDOU_NAV_SUBFRAME_BITS]);

/* Build the ten subframe-1 pages carrying a GEO satellite's D2 clock,
 * ionosphere and ephemeris. frame_sow is page 1's frame epoch; later pages
 * receive successive three-second frame epochs modulo one BDT week. */
int gnss_beidou_d2_basic_pages(const beidou_d1_clock_t *clock,
	const beidou_d1_ephemeris_t *ephemeris, uint32_t frame_sow,
	uint8_t pages[10][BEIDOU_NAV_SUBFRAME_BITS]);

#endif
