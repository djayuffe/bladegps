#include "gnss_galileo_nav.h"

#include <stddef.h>
#include <limits.h>
#include <math.h>
#include <string.h>

#include "gnss_fec.h"

static const uint8_t inav_sync[GALILEO_INAV_SYNC_SYMBOLS] =
	{0, 1, 0, 1, 1, 0, 0, 0, 0, 0};

static int valid_bits(const uint8_t *bits, size_t count)
{
	size_t index;

	if (bits == NULL)
		return 0;
	for (index = 0; index < count; index++)
		if (bits[index] > 1U)
			return 0;
	return 1;
}

static void append_uint(uint8_t *bits, size_t *offset, uint32_t value,
	size_t width)
{
	size_t index;

	for (index = 0; index < width; index++)
		bits[(*offset)++] = (uint8_t)((value >> (width - index - 1U)) & 1U);
}

static int fits_unsigned(uint32_t value, unsigned int width)
{
	return width == 32U || value < (UINT32_C(1) << width);
}

static int fits_signed(int32_t value, unsigned int width)
{
	int64_t limit;

	if (width == 32U)
		return 1;
	limit = INT64_C(1) << (width - 1U);
	return (int64_t)value >= -limit && (int64_t)value < limit;
}

static void append_signed(uint8_t *bits, size_t *offset, int32_t value,
	unsigned int width)
{
	uint32_t encoded = (uint32_t)value;

	if (width < 32U)
		encoded &= (UINT32_C(1) << width) - 1U;
	append_uint(bits, offset, encoded, width);
}

static int finish_word(size_t offset)
{
	return offset == GALILEO_INAV_WORD_BITS ? 0 : -1;
}

int gnss_galileo_inav_word1(const galileo_inav_word1_t *f, uint8_t word[128])
{
	size_t at = 0U;
	if (f == NULL || word == NULL || !fits_unsigned(f->iodnav, 10U) ||
		!fits_unsigned(f->toe, 14U)) return -1;
	append_uint(word,&at,1U,6U); append_uint(word,&at,f->iodnav,10U);
	append_uint(word,&at,f->toe,14U); append_signed(word,&at,f->mean_anomaly,32U);
	append_uint(word,&at,f->eccentricity,32U); append_uint(word,&at,f->sqrt_a,32U);
	append_uint(word,&at,0U,2U); return finish_word(at);
}

int gnss_galileo_inav_word2(const galileo_inav_word2_t *f, uint8_t word[128])
{
	size_t at = 0U;
	if (f == NULL || word == NULL || !fits_unsigned(f->iodnav,10U) ||
		!fits_signed(f->inclination_rate,14U)) return -1;
	append_uint(word,&at,2U,6U); append_uint(word,&at,f->iodnav,10U);
	append_signed(word,&at,f->omega0,32U); append_signed(word,&at,f->inclination0,32U);
	append_signed(word,&at,f->argument_of_perigee,32U);
	append_signed(word,&at,f->inclination_rate,14U); append_uint(word,&at,0U,2U);
	return finish_word(at);
}

int gnss_galileo_inav_word3(const galileo_inav_word3_t *f, uint8_t word[128])
{
	size_t at = 0U;
	if (f == NULL || word == NULL || !fits_unsigned(f->iodnav,10U) ||
		!fits_signed(f->omega_rate,24U)) return -1;
	append_uint(word,&at,3U,6U); append_uint(word,&at,f->iodnav,10U);
	append_signed(word,&at,f->omega_rate,24U);
	append_signed(word,&at,f->delta_mean_motion,16U);
	append_signed(word,&at,f->cuc,16U); append_signed(word,&at,f->cus,16U);
	append_signed(word,&at,f->crc,16U); append_signed(word,&at,f->crs,16U);
	append_uint(word,&at,f->sisa,8U); return finish_word(at);
}

int gnss_galileo_inav_word4(const galileo_inav_word4_t *f, uint8_t word[128])
{
	size_t at = 0U;
	if (f == NULL || word == NULL || !fits_unsigned(f->iodnav,10U) ||
		!fits_unsigned(f->svid,6U) || f->svid == 0U || !fits_unsigned(f->toc,14U) ||
		!fits_signed(f->af0,31U) || !fits_signed(f->af1,21U) ||
		!fits_signed(f->af2,6U)) return -1;
	append_uint(word,&at,4U,6U); append_uint(word,&at,f->iodnav,10U);
	append_uint(word,&at,f->svid,6U); append_signed(word,&at,f->cic,16U);
	append_signed(word,&at,f->cis,16U); append_uint(word,&at,f->toc,14U);
	append_signed(word,&at,f->af0,31U); append_signed(word,&at,f->af1,21U);
	append_signed(word,&at,f->af2,6U); append_uint(word,&at,0U,2U);
	return finish_word(at);
}

int gnss_galileo_inav_word5(const galileo_inav_word5_t *f, uint8_t word[128])
{
	size_t at = 0U;
	if (f == NULL || word == NULL || !fits_unsigned(f->ai0,11U) ||
		!fits_signed(f->ai1,11U) || !fits_signed(f->ai2,14U) ||
		!fits_unsigned(f->disturbance_flags,5U) ||
		!fits_signed(f->bgd_e1e5a,10U) || !fits_signed(f->bgd_e1e5b,10U) ||
		!fits_unsigned(f->e5b_health,2U) || !fits_unsigned(f->e1b_health,2U) ||
		!fits_unsigned(f->e5b_dvs,1U) || !fits_unsigned(f->e1b_dvs,1U) ||
		!fits_unsigned(f->week,12U) || !fits_unsigned(f->tow,20U)) return -1;
	append_uint(word,&at,5U,6U); append_uint(word,&at,f->ai0,11U);
	append_signed(word,&at,f->ai1,11U); append_signed(word,&at,f->ai2,14U);
	append_uint(word,&at,f->disturbance_flags,5U);
	append_signed(word,&at,f->bgd_e1e5a,10U); append_signed(word,&at,f->bgd_e1e5b,10U);
	append_uint(word,&at,f->e5b_health,2U); append_uint(word,&at,f->e1b_health,2U);
	append_uint(word,&at,f->e5b_dvs,1U); append_uint(word,&at,f->e1b_dvs,1U);
	append_uint(word,&at,f->week,12U); append_uint(word,&at,f->tow,20U);
	append_uint(word,&at,0U,23U); return finish_word(at);
}

static int64_t civil_days(int year, unsigned int month, unsigned int day)
{
	int adjusted_year = year - (month <= 2U);
	int era = (adjusted_year >= 0 ? adjusted_year : adjusted_year - 399) / 400;
	unsigned int yoe = (unsigned int)(adjusted_year - era * 400);
	unsigned int adjusted_month = month > 2U ? month - 3U : month + 9U;
	unsigned int doy = (153U * adjusted_month + 2U) / 5U + day - 1U;
	unsigned int doe = yoe * 365U + yoe / 4U - yoe / 100U + doy;
	return (int64_t)era * 146097 + (int64_t)doe - 719468;
}

static double galileo_sow(const gnss_calendar_time_t *time)
{
	int weekday = (int)((civil_days(time->year, (unsigned int)time->month,
		(unsigned int)time->day) + 4) % 7);
	if (weekday < 0) weekday += 7;
	return weekday * 86400.0 + time->hour * 3600.0 +
		time->minute * 60.0 + time->second;
}

static int quantize(double value, int exponent, int32_t *result)
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

static int galileo_sisa_index(double meters, uint8_t *index)
{
	double raw;
	if (index == NULL || !isfinite(meters)) return -1;
	if (meters < 0.0) { *index = 255U; return 0; }
	if (meters <= 0.49) raw = meters * 100.0;
	else if (meters <= 0.98) raw = 50.0 + (meters - 0.50) / 0.02;
	else if (meters <= 1.96) raw = 75.0 + (meters - 1.00) / 0.04;
	else if (meters <= 6.0) raw = 100.0 + (meters - 2.00) / 0.16;
	else { *index = 255U; return 0; }
	if (raw < 0.0 || raw > 125.0) return -1;
	*index = (uint8_t)llround(raw);
	return 0;
}

int gnss_galileo_inav_ephemeris_words(const gnss_nav_record_t *r,
	uint8_t words[4][GALILEO_INAV_WORD_BITS])
{
	galileo_inav_word1_t w1;
	galileo_inav_word2_t w2;
	galileo_inav_word3_t w3;
	galileo_inav_word4_t w4;
	int32_t temporary;
	double toc;
	const double pi = 3.14159265358979323846;

	if (r == NULL || words == NULL || r->system != GNSS_SYSTEM_GALILEO ||
		strcmp(r->message, "INAV") != 0 || r->model != GNSS_NAV_KEPLERIAN ||
		r->orbit_count < 23U || !isfinite(r->orbit[0]) ||
		r->orbit[0] < 0.0 || r->orbit[0] > 1023.0) return -1;
	memset(&w1,0,sizeof(w1)); memset(&w2,0,sizeof(w2));
	memset(&w3,0,sizeof(w3)); memset(&w4,0,sizeof(w4));
	w1.iodnav = w2.iodnav = w3.iodnav = w4.iodnav = (uint16_t)llround(r->orbit[0]);
	if (fabs(r->orbit[0] - w1.iodnav) > 1.0e-6 || !isfinite(r->orbit[8]) ||
		r->orbit[8] < 0.0 || r->orbit[8] >= 604800.0) return -1;
	w1.toe = (uint16_t)llround(r->orbit[8] / 60.0);
	if (fabs(r->orbit[8] - w1.toe * 60.0) > 1.0e-3 ||
		quantize(r->orbit[3] / pi,31,&w1.mean_anomaly) != 0 ||
		quantize_unsigned(r->orbit[5],33,&w1.eccentricity) != 0 ||
		quantize_unsigned(r->orbit[7],19,&w1.sqrt_a) != 0) return -1;
	if (
		quantize(r->orbit[10]/pi,31,&w2.omega0) != 0 ||
		quantize(r->orbit[12]/pi,31,&w2.inclination0) != 0 ||
		quantize(r->orbit[14]/pi,31,&w2.argument_of_perigee) != 0 ||
		quantize(r->orbit[16]/pi,43,&temporary) != 0 ||
		!fits_signed(temporary,14U)) return -1;
	w2.inclination_rate = (int16_t)temporary;
	if (quantize(r->orbit[15]/pi,43,&w3.omega_rate) != 0 ||
		!fits_signed(w3.omega_rate,24U) ||
		quantize(r->orbit[2]/pi,43,&temporary) != 0 ||
		!fits_signed(temporary,16U)) return -1;
	w3.delta_mean_motion = (int16_t)temporary;
#define Q16(field, value, exponent) do { \
	if (quantize((value),(exponent),&temporary) != 0 || \
		!fits_signed(temporary,16U)) return -1; \
	(field) = (int16_t)temporary; \
} while (0)
	Q16(w3.cuc,r->orbit[4],29); Q16(w3.cus,r->orbit[6],29);
	Q16(w3.crc,r->orbit[13],5); Q16(w3.crs,r->orbit[1],5);
	if (galileo_sisa_index(r->orbit[19],&w3.sisa) != 0) return -1;
	w4.svid = (uint8_t)r->prn;
	Q16(w4.cic,r->orbit[9],29); Q16(w4.cis,r->orbit[11],29);
#undef Q16
	toc = galileo_sow(&r->toc);
	w4.toc = (uint16_t)llround(toc / 60.0);
	if (fabs(toc - w4.toc * 60.0) > 1.0e-3 ||
		quantize(r->clock_bias,34,&w4.af0) != 0 || !fits_signed(w4.af0,31U) ||
		quantize(r->clock_drift,46,&w4.af1) != 0 || !fits_signed(w4.af1,21U) ||
		quantize(r->clock_drift_rate,59,&temporary) != 0 ||
		!fits_signed(temporary,6U)) return -1;
	w4.af2 = (int8_t)temporary;
	if (gnss_galileo_inav_word1(&w1,words[0]) != 0) return -1;
	if (gnss_galileo_inav_word2(&w2,words[1]) != 0) return -1;
	if (gnss_galileo_inav_word3(&w3,words[2]) != 0) return -1;
	if (gnss_galileo_inav_word4(&w4,words[3]) != 0) return -1;
	return 0;
}

static int encode_page_part(const uint8_t page[GALILEO_INAV_PAGE_PART_BITS],
	uint8_t symbols[GALILEO_INAV_PAGE_PART_SYMBOLS])
{
	uint8_t encoded[GALILEO_INAV_CODED_SYMBOLS];

	memcpy(symbols, inav_sync, sizeof(inav_sync));
	if (gnss_galileo_convolutional_encode(page, GALILEO_INAV_PAGE_PART_BITS,
		encoded, sizeof(encoded)) != 0)
		return -1;
	return gnss_block_interleave(encoded, 30U, 8U,
		symbols + GALILEO_INAV_SYNC_SYMBOLS, GALILEO_INAV_CODED_SYMBOLS);
}

int gnss_galileo_inav_e1b_page(const uint8_t word[GALILEO_INAV_WORD_BITS],
	const uint8_t osnma[GALILEO_INAV_OSNMA_BITS],
	const uint8_t sar[GALILEO_INAV_SAR_BITS], uint8_t spare,
	galileo_inav_ssp_t ssp,
	uint8_t even_symbols[GALILEO_INAV_PAGE_PART_SYMBOLS],
	uint8_t odd_symbols[GALILEO_INAV_PAGE_PART_SYMBOLS], uint32_t *crc)
{
	static const uint8_t ssp_bits[3][8] = {
		{0,0,0,0,0,1,0,0}, {0,0,1,0,1,0,1,1}, {0,0,1,0,1,1,1,1}
	};
	uint8_t even[GALILEO_INAV_PAGE_PART_BITS] = {0};
	uint8_t odd[GALILEO_INAV_PAGE_PART_BITS] = {0};
	uint8_t protected_bits[196];
	size_t even_offset = 0U, odd_offset = 0U, protected_offset = 0U;
	uint32_t checksum;

	if (!valid_bits(word, GALILEO_INAV_WORD_BITS) ||
		!valid_bits(osnma, GALILEO_INAV_OSNMA_BITS) ||
		!valid_bits(sar, GALILEO_INAV_SAR_BITS) || spare > 3U ||
		ssp < GALILEO_INAV_SSP1 || ssp > GALILEO_INAV_SSP3 ||
		even_symbols == NULL || odd_symbols == NULL)
		return -1;

	/* Even/odd=0, page type=0, data(1/2), six zero tail bits. */
	append_uint(even, &even_offset, 0U, 2U);
	memcpy(even + even_offset, word, 112U);
	even_offset += 112U;
	even_offset += 6U;

	/* CRC protection order is the two E1-B page headers and their protected
	 * fields in transmission order. Tail, CRC, SSP are excluded. */
	memcpy(protected_bits + protected_offset, even, 114U);
	protected_offset += 114U;

	append_uint(odd, &odd_offset, 2U, 2U); /* even/odd=1, page type=0 */
	memcpy(odd + odd_offset, word + 112U, 16U);
	odd_offset += 16U;
	memcpy(odd + odd_offset, osnma, GALILEO_INAV_OSNMA_BITS);
	odd_offset += GALILEO_INAV_OSNMA_BITS;
	memcpy(odd + odd_offset, sar, GALILEO_INAV_SAR_BITS);
	odd_offset += GALILEO_INAV_SAR_BITS;
	append_uint(odd, &odd_offset, spare, 2U);
	memcpy(protected_bits + protected_offset, odd, odd_offset);
	protected_offset += odd_offset;
	if (even_offset != GALILEO_INAV_PAGE_PART_BITS || protected_offset != 196U)
		return -1;

	checksum = gnss_crc24q_bits(protected_bits, protected_offset);
	append_uint(odd, &odd_offset, checksum, 24U);
	memcpy(odd + odd_offset, ssp_bits[(unsigned int)ssp - 1U], 8U);
	odd_offset += 8U;
	odd_offset += 6U;
	if (odd_offset != GALILEO_INAV_PAGE_PART_BITS ||
		encode_page_part(even, even_symbols) != 0 ||
		encode_page_part(odd, odd_symbols) != 0)
		return -1;
	if (crc != NULL)
		*crc = checksum;
	return 0;
}

int gnss_galileo_inav_e1b_dummy_page(const uint8_t sequence[186],
	uint8_t even_symbols[GALILEO_INAV_PAGE_PART_SYMBOLS],
	uint8_t odd_symbols[GALILEO_INAV_PAGE_PART_SYMBOLS], uint32_t *crc)
{
	uint8_t dummy[192],even[GALILEO_INAV_PAGE_PART_BITS]={0};
	uint8_t odd[GALILEO_INAV_PAGE_PART_BITS]={0},protected_bits[196];
	size_t dummy_offset=0U,even_offset=0U,odd_offset=0U,protected_offset=0U;
	uint32_t checksum;
	if(!valid_bits(sequence,186U)||even_symbols==NULL||odd_symbols==NULL)return -1;
	append_uint(dummy,&dummy_offset,63U,6U);
	memcpy(dummy+dummy_offset,sequence,186U);dummy_offset+=186U;
	if(dummy_offset!=sizeof(dummy))return -1;
	/* OS SIS ICD 2.2 Tables 56-57: one vertical E1-B dummy page.
	 * The final eight bits are spare, not a secondary sync pattern. */
	append_uint(even,&even_offset,0U,2U);
	memcpy(even+even_offset,dummy,112U);even_offset+=112U;
	memcpy(protected_bits+protected_offset,even,114U);protected_offset+=114U;
	even_offset+=6U;
	append_uint(odd,&odd_offset,2U,2U);
	memcpy(odd+odd_offset,dummy+112U,80U);odd_offset+=80U;
	memcpy(protected_bits+protected_offset,odd,82U);protected_offset+=82U;
	checksum=gnss_crc24q_bits(protected_bits,protected_offset);
	append_uint(odd,&odd_offset,checksum,24U);
	odd_offset+=8U; /* Spare. */
	odd_offset+=6U; /* Convolutional encoder tail. */
	if(even_offset!=GALILEO_INAV_PAGE_PART_BITS||
		odd_offset!=GALILEO_INAV_PAGE_PART_BITS||protected_offset!=196U||
		encode_page_part(even,even_symbols)!=0||
		encode_page_part(odd,odd_symbols)!=0)return -1;
	if(crc!=NULL)*crc=checksum;
	return 0;
}

int gnss_galileo_inav_e1b_word_type(unsigned int gst_second_mod_30)
{
	static const int words[30] = {
		16, 2, 2, 4, 4, 6, 6, 7, 7, 8,
		8, 17, 17, 19, 19, 16, 16, 0, 0, 22,
		22, 1, 1, 3, 3, 5, 5, 0, 0, 16
	};

	return gst_second_mod_30 < 30U ? words[gst_second_mod_30] : -1;
}

galileo_inav_ssp_t gnss_galileo_inav_ssp_for_second(unsigned int second)
{
	/* Odd E1-B parts occur at even-numbered GST seconds. Table 40 cycles
	 * SSP3, SSP1, SSP2 over those successive page epochs. */
	static const galileo_inav_ssp_t sequence[6] = {
		GALILEO_INAV_SSP3, GALILEO_INAV_SSP3,
		GALILEO_INAV_SSP1, GALILEO_INAV_SSP1,
		GALILEO_INAV_SSP2, GALILEO_INAV_SSP2
	};
	return sequence[second % 6U];
}
