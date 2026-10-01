#include "gnss_beidou_nav.h"

#include <stddef.h>
#include <limits.h>
#include <math.h>
#include <string.h>

#include "gnss_fec.h"

static int quantize_signed(double value, int exponent, int32_t *result);
static int quantize_unsigned(double value, int exponent, uint32_t *result);

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

int gnss_beidou_d1_clock_subframe(const beidou_d1_clock_t *f,
	uint32_t sow, uint8_t subframe[300])
{
	uint8_t payload[BEIDOU_NAV_PAYLOAD_BITS] = {0};
	size_t at=0U;
	unsigned int index;
	if (f==NULL || subframe==NULL || sow>=604800U || f->health>1U ||
		f->aodc>31U || f->urai>15U || f->week>8191U || f->toc>75599U ||
		!fits_signed(f->tgd1,10U) || !fits_signed(f->tgd2,10U) ||
		!fits_signed(f->af2,11U) || !fits_signed(f->af0,24U) ||
		!fits_signed(f->af1,22U) || f->aode>31U) return -1;
	append_uint(payload,&at,f->health,1U); append_uint(payload,&at,f->aodc,5U);
	append_uint(payload,&at,f->urai,4U); append_uint(payload,&at,f->week,13U);
	append_uint(payload,&at,f->toc,17U); append_signed(payload,&at,f->tgd1,10U);
	append_signed(payload,&at,f->tgd2,10U);
	for(index=0U;index<4U;index++) append_signed(payload,&at,f->alpha[index],8U);
	for(index=0U;index<4U;index++) append_signed(payload,&at,f->beta[index],8U);
	append_signed(payload,&at,f->af2,11U); append_signed(payload,&at,f->af0,24U);
	append_signed(payload,&at,f->af1,22U); append_uint(payload,&at,f->aode,5U);
	return at==BEIDOU_NAV_PAYLOAD_BITS ?
		gnss_beidou_nav_build_subframe(1U,sow,payload,subframe) : -1;
}

static int64_t civil_days(int year, unsigned int month, unsigned int day)
{
	int y=year-(month<=2U); int era=(y>=0?y:y-399)/400;
	unsigned int yoe=(unsigned int)(y-era*400), m=month>2U?month-3U:month+9U;
	unsigned int doy=(153U*m+2U)/5U+day-1U;
	return (int64_t)era*146097+(int64_t)(yoe*365U+yoe/4U-yoe/100U+doy)-719468;
}

static double calendar_sow(const gnss_calendar_time_t *t)
{
	int weekday=(int)((civil_days(t->year,(unsigned int)t->month,
		(unsigned int)t->day)+4)%7);
	if(weekday<0) weekday+=7;
	return weekday*86400.0+t->hour*3600.0+t->minute*60.0+t->second;
}

static int beidou_urai(double accuracy, uint8_t *urai)
{
	double best_error=HUGE_VAL;
	unsigned int value,best=15U;
	if (!isfinite(accuracy) || accuracy<0.0 || urai==NULL) return -1;
	if (accuracy>=6144.0) { *urai=15U; return 0; }
	for(value=0U;value<15U;value++) {
		double nominal=value<6U?pow(2.0,1.0+value/2.0):pow(2.0,(double)value-2.0);
		double error=fabs(accuracy-nominal);
		if(error<best_error){best_error=error;best=value;}
	}
	*urai=(uint8_t)best; return 0;
}

int gnss_beidou_d1_clock_from_rinex(const gnss_nav_record_t *r,
	const int8_t alpha[4], const int8_t beta[4], beidou_d1_clock_t *f)
{
	double toc;
	int32_t value;
	unsigned int index;
	if(r==NULL||alpha==NULL||beta==NULL||f==NULL||r->system!=GNSS_SYSTEM_BEIDOU||
		strcmp(r->message,"D1")!=0||r->orbit_count<26U) return -1;
	memset(f,0,sizeof(*f));
	for(index=0U;index<4U;index++){f->alpha[index]=alpha[index];f->beta[index]=beta[index];}
	if(!isfinite(r->orbit[0])||!isfinite(r->orbit[18])||!isfinite(r->orbit[21])||
		!isfinite(r->orbit[22])||!isfinite(r->orbit[23])||!isfinite(r->orbit[25])) return -1;
	f->aode=(uint8_t)llround(r->orbit[0]); f->aodc=(uint8_t)llround(r->orbit[25]);
	f->week=(uint16_t)llround(r->orbit[18]); f->health=(uint8_t)llround(r->orbit[21]);
	if(fabs(r->orbit[0]-f->aode)>1e-6||fabs(r->orbit[25]-f->aodc)>1e-6||
		fabs(r->orbit[18]-f->week)>1e-6||fabs(r->orbit[21]-f->health)>1e-6||
		beidou_urai(r->orbit[20],&f->urai)!=0) return -1;
	toc=calendar_sow(&r->toc); f->toc=(uint32_t)llround(toc/8.0);
	if(fabs(toc-f->toc*8.0)>1e-3 || fabs(r->orbit[22]*1.0e10)>32767.0 ||
		fabs(r->orbit[23]*1.0e10)>32767.0) return -1;
	/* TGD LSB is 0.1 ns = 1e-10 s. */
	value=(int32_t)llround(r->orbit[22]*1.0e10); f->tgd1=(int16_t)value;
	value=(int32_t)llround(r->orbit[23]*1.0e10); f->tgd2=(int16_t)value;
	if(quantize_signed(r->clock_bias,33,&f->af0)!=0||
		quantize_signed(r->clock_drift,50,&f->af1)!=0||
		quantize_signed(r->clock_drift_rate,66,&f->af2)!=0) return -1;
	return f->aode<=31U&&f->aodc<=31U&&f->week<=8191U&&f->health<=1U&&
		fits_signed(f->tgd1,10U)&&fits_signed(f->tgd2,10U)&&
		fits_signed(f->af0,24U)&&fits_signed(f->af1,22U)&&fits_signed(f->af2,11U)?0:-1;
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
