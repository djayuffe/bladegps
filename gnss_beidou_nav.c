#include "gnss_beidou_nav.h"

#include <stddef.h>
#include <limits.h>
#include <math.h>
#include <string.h>

#include "gnss_fec.h"

static int valid_bits(const uint8_t *bits, size_t count)
{
	size_t index;
	if (bits == NULL) return 0;
	for (index = 0; index < count; index++)
		if (bits[index] > 1U) return 0;
	return 1;
}

static void append_uint(uint8_t *bits, size_t *offset, uint32_t value,
	unsigned int width)
{
	unsigned int bit;
	for (bit = 0U; bit < width; bit++)
		bits[(*offset)++] = (uint8_t)((value >> (width - bit - 1U)) & 1U);
}

static int fits_signed(int32_t value, unsigned int width)
{
	int64_t limit = INT64_C(1) << (width - 1U);
	return width == 32U || ((int64_t)value >= -limit && (int64_t)value < limit);
}

static void append_signed(uint8_t *bits, size_t *offset, int32_t value,
	unsigned int width)
{
	uint32_t encoded = (uint32_t)value;
	if (width < 32U) encoded &= (UINT32_C(1) << width) - 1U;
	append_uint(bits, offset, encoded, width);
}

int gnss_beidou_nav_encode_subframe(const uint8_t information[224],
	uint8_t subframe[300])
{
	uint8_t first[15], second[15];
	unsigned int word;

	if (!valid_bits(information, BEIDOU_NAV_INFORMATION_BITS) || subframe == NULL)
		return -1;
	/* Word 1: 15 uncoded bits followed by one systematic BCH(15,11,1)
	 * codeword. */
	memcpy(subframe, information, 15U);
	if (gnss_beidou_bch15_11(information + 15U, subframe + 15U) != 0)
		return -1;
	/* Words 2..10: two 11-bit information blocks are BCH encoded and
	 * transmitted alternately one bit from each codeword. */
	for (word = 1U; word < 10U; word++) {
		const uint8_t *source = information + 26U + (word - 1U) * 22U;
		if (gnss_beidou_bch15_11(source, first) != 0 ||
			gnss_beidou_bch15_11(source + 11U, second) != 0 ||
			gnss_beidou_interleave_2x15(first, second, subframe + word * 30U) != 0)
			return -1;
	}
	return 0;
}

int gnss_beidou_nav_build_subframe(unsigned int fraid, uint32_t sow,
	const uint8_t payload[186], uint8_t subframe[300])
{
	uint8_t information[BEIDOU_NAV_INFORMATION_BITS];
	size_t offset = 0U;

	if (fraid < 1U || fraid > 5U || sow >= 604800U ||
		!valid_bits(payload, BEIDOU_NAV_PAYLOAD_BITS) || subframe == NULL)
		return -1;
	append_uint(information, &offset, UINT32_C(0x712), 11U);
	append_uint(information, &offset, 0U, 4U);
	append_uint(information, &offset, fraid, 3U);
	append_uint(information, &offset, sow, 20U);
	memcpy(information + offset, payload, BEIDOU_NAV_PAYLOAD_BITS);
	offset += BEIDOU_NAV_PAYLOAD_BITS;
	return offset == BEIDOU_NAV_INFORMATION_BITS ?
		gnss_beidou_nav_encode_subframe(information, subframe) : -1;
}

int gnss_beidou_d1_ephemeris_subframes(const beidou_d1_ephemeris_t *f,
	uint32_t frame_sow, uint8_t subframe2[300], uint8_t subframe3[300])
{
	uint8_t second[BEIDOU_NAV_PAYLOAD_BITS] = {0};
	uint8_t third[BEIDOU_NAV_PAYLOAD_BITS] = {0};
	size_t at2 = 0U, at3 = 0U;

	if (f == NULL || subframe2 == NULL || subframe3 == NULL ||
		frame_sow >= 604800U || f->toe >= (UINT32_C(1) << 17) ||
		!fits_signed(f->delta_mean_motion,16U) || !fits_signed(f->cuc,18U) ||
		!fits_signed(f->cus,18U) || !fits_signed(f->crc,18U) ||
		!fits_signed(f->crs,18U) || !fits_signed(f->inclination0,32U) ||
		!fits_signed(f->cic,18U) || !fits_signed(f->omega_rate,24U) ||
		!fits_signed(f->cis,18U) || !fits_signed(f->inclination_rate,14U))
		return -1;
	append_signed(second,&at2,f->delta_mean_motion,16U);
	append_signed(second,&at2,f->cuc,18U);
	append_signed(second,&at2,f->mean_anomaly,32U);
	append_uint(second,&at2,f->eccentricity,32U);
	append_signed(second,&at2,f->cus,18U);
	append_signed(second,&at2,f->crc,18U);
	append_signed(second,&at2,f->crs,18U);
	append_uint(second,&at2,f->sqrt_a,32U);
	append_uint(second,&at2,f->toe >> 15,2U);

	append_uint(third,&at3,f->toe & UINT32_C(0x7fff),15U);
	append_signed(third,&at3,f->inclination0,32U);
	append_signed(third,&at3,f->cic,18U);
	append_signed(third,&at3,f->omega_rate,24U);
	append_signed(third,&at3,f->cis,18U);
	append_signed(third,&at3,f->inclination_rate,14U);
	append_signed(third,&at3,f->omega0,32U);
	append_signed(third,&at3,f->argument_of_perigee,32U);
	append_uint(third,&at3,0U,1U);
	if (at2 != BEIDOU_NAV_PAYLOAD_BITS || at3 != BEIDOU_NAV_PAYLOAD_BITS)
		return -1;
	return gnss_beidou_nav_build_subframe(2U,(frame_sow+6U)%604800U,
		second,subframe2) == 0 &&
		gnss_beidou_nav_build_subframe(3U,(frame_sow+12U)%604800U,
		third,subframe3) == 0 ? 0 : -1;
}

static int quantize_signed(double value, int exponent, int32_t *result)
{
	double scaled;
	if (!isfinite(value) || result == NULL) return -1;
	scaled = ldexp(value, exponent);
	if (scaled < (double)INT32_MIN || scaled > (double)INT32_MAX) return -1;
	*result = (int32_t)llround(scaled);
	return 0;
}

static int quantize_unsigned(double value, int exponent, uint32_t *result)
{
	double scaled;
	if (!isfinite(value) || result == NULL) return -1;
	scaled = ldexp(value, exponent);
	if (scaled < 0.0 || scaled > (double)UINT32_MAX) return -1;
	*result = (uint32_t)llround(scaled);
	return 0;
}

int gnss_beidou_d1_ephemeris_from_rinex(const gnss_nav_record_t *r,
	beidou_d1_ephemeris_t *f)
{
	const double pi = 3.14159265358979323846;
	int32_t value;
	if (r == NULL || f == NULL || r->system != GNSS_SYSTEM_BEIDOU ||
		strcmp(r->message,"D1") != 0 || r->model != GNSS_NAV_KEPLERIAN ||
		r->orbit_count < 26U || !isfinite(r->orbit[8]) ||
		r->orbit[8] < 0.0 || r->orbit[8] >= 604800.0) return -1;
	memset(f,0,sizeof(*f));
	f->toe = (uint32_t)llround(r->orbit[8] / 8.0);
	if (fabs(r->orbit[8] - f->toe * 8.0) > 1.0e-3 ||
		quantize_signed(r->orbit[2]/pi,43,&f->delta_mean_motion) != 0 ||
		quantize_signed(r->orbit[4],31,&f->cuc) != 0 ||
		quantize_signed(r->orbit[3]/pi,31,&f->mean_anomaly) != 0 ||
		quantize_unsigned(r->orbit[5],33,&f->eccentricity) != 0 ||
		quantize_signed(r->orbit[6],31,&f->cus) != 0 ||
		quantize_signed(r->orbit[13],6,&f->crc) != 0 ||
		quantize_signed(r->orbit[1],6,&f->crs) != 0 ||
		quantize_unsigned(r->orbit[7],19,&f->sqrt_a) != 0 ||
		quantize_signed(r->orbit[12]/pi,31,&f->inclination0) != 0 ||
		quantize_signed(r->orbit[9],31,&f->cic) != 0 ||
		quantize_signed(r->orbit[15]/pi,43,&f->omega_rate) != 0 ||
		quantize_signed(r->orbit[11],31,&f->cis) != 0 ||
		quantize_signed(r->orbit[16]/pi,43,&value) != 0 ||
		!fits_signed(value,14U)) return -1;
	f->inclination_rate = value;
	if (quantize_signed(r->orbit[10]/pi,31,&f->omega0) != 0 ||
		quantize_signed(r->orbit[14]/pi,31,&f->argument_of_perigee) != 0 ||
		!fits_signed(f->delta_mean_motion,16U) || !fits_signed(f->cuc,18U) ||
		!fits_signed(f->cus,18U) || !fits_signed(f->crc,18U) ||
		!fits_signed(f->crs,18U) || !fits_signed(f->cic,18U) ||
		!fits_signed(f->omega_rate,24U) || !fits_signed(f->cis,18U)) return -1;
	return 0;
}
