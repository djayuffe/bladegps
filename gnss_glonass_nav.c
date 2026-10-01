#include "gnss_glonass_nav.h"

#include <limits.h>
#include <math.h>
#include <stddef.h>
#include <string.h>

#include "gnss_fec.h"

static int set_unsigned(uint8_t data[77], unsigned int first,
	unsigned int width, uint32_t value)
{
	unsigned int bit;
	if (first < 9U || width == 0U || first + width - 1U > 85U ||
		(width < 32U && value >= (UINT32_C(1) << width))) return -1;
	for (bit = 0U; bit < width; bit++)
		data[85U-(first+bit)] = (uint8_t)((value >> bit) & 1U);
	return 0;
}

static int set_sign_magnitude(uint8_t data[77], unsigned int first,
	unsigned int width, int32_t value)
{
	uint32_t magnitude;
	if (width < 2U || (value == INT32_MIN)) return -1;
	magnitude = value < 0 ? (uint32_t)-value : (uint32_t)value;
	if (magnitude >= (UINT32_C(1) << (width-1U)) ||
		set_unsigned(data,first,width-1U,magnitude) != 0 ||
		set_unsigned(data,first+width-1U,1U,value < 0 ? 1U : 0U) != 0)
		return -1;
	return 0;
}

static int finish_string(uint8_t data[77], uint8_t output[85])
{
	return gnss_glonass_hamming_85_77(data,output);
}

int gnss_glonass_gnav_immediate_strings(const glonass_gnav_immediate_t *f,
	uint8_t strings[4][85])
{
	uint8_t data[4][77];
	uint32_t tk;
	unsigned int axis;

	if (f == NULL || strings == NULL || f->tk_seconds >= 86400U ||
		(f->tk_seconds % 30U) != 0U || f->tb > 95U || f->bn > 7U ||
		f->p1 > 3U || f->p2 > 1U || f->p3 > 1U || f->p4 > 1U ||
		f->p > 3U || f->ln > 1U || f->ft > 15U || f->en > 31U ||
		f->slot > 31U || f->mode > 3U || f->nt > 1461U)
		return -1;
	memset(data,0,sizeof(data));
	tk = (f->tk_seconds/3600U)<<7 |
		((f->tk_seconds%3600U)/60U)<<1 | ((f->tk_seconds%60U)/30U);
	for (axis=0U; axis<3U; axis++)
		if (set_sign_magnitude(data[axis],9U,27U,f->position[axis]) != 0 ||
			set_sign_magnitude(data[axis],36U,5U,f->acceleration[axis]) != 0 ||
			set_sign_magnitude(data[axis],41U,24U,f->velocity[axis]) != 0 ||
			set_unsigned(data[axis],81U,4U,axis+1U) != 0) return -1;
	if (set_unsigned(data[0],65U,12U,tk) != 0 ||
		set_unsigned(data[0],77U,2U,f->p1) != 0 ||
		set_unsigned(data[1],70U,7U,f->tb) != 0 ||
		set_unsigned(data[1],77U,1U,f->p2) != 0 ||
		set_unsigned(data[1],78U,3U,f->bn) != 0 ||
		set_unsigned(data[2],65U,1U,f->ln) != 0 ||
		set_unsigned(data[2],66U,2U,f->p) != 0 ||
		set_sign_magnitude(data[2],69U,11U,f->gamma) != 0 ||
		set_unsigned(data[2],80U,1U,f->p3) != 0 ||
		set_unsigned(data[3],9U,2U,f->mode) != 0 ||
		set_unsigned(data[3],11U,5U,f->slot) != 0 ||
		set_unsigned(data[3],16U,11U,f->nt) != 0 ||
		set_unsigned(data[3],30U,4U,f->ft) != 0 ||
		set_unsigned(data[3],34U,1U,f->p4) != 0 ||
		set_unsigned(data[3],49U,5U,f->en) != 0 ||
		set_sign_magnitude(data[3],54U,5U,f->delta_tau) != 0 ||
		set_sign_magnitude(data[3],59U,22U,f->tau) != 0 ||
		set_unsigned(data[3],81U,4U,4U) != 0) return -1;
	return finish_string(data[0],strings[0]) == 0 &&
		finish_string(data[1],strings[1]) == 0 &&
		finish_string(data[2],strings[2]) == 0 &&
		finish_string(data[3],strings[3]) == 0 ? 0 : -1;
}

int gnss_glonass_gnav_string5(const glonass_gnav_string5_t *f,
	uint8_t string[85])
{
	uint8_t data[77] = {0};
	if (f == NULL || string == NULL || f->na < 1U || f->na > 1461U ||
		f->n4 < 1U || f->n4 > 31U || f->ln > 1U ||
		set_unsigned(data,9U,1U,f->ln) != 0 ||
		set_sign_magnitude(data,10U,22U,f->tau_gps) != 0 ||
		set_unsigned(data,32U,5U,f->n4) != 0 ||
		set_sign_magnitude(data,38U,32U,f->tau_c) != 0 ||
		set_unsigned(data,70U,11U,f->na) != 0 ||
		set_unsigned(data,81U,4U,5U) != 0)
		return -1;
	return finish_string(data,string);
}

int gnss_glonass_gnav_almanac_pair(const glonass_gnav_almanac_t *f,
	unsigned int number, uint8_t even[85], uint8_t odd[85])
{
	uint8_t first[77] = {0}, second[77] = {0};
	if (f == NULL || even == NULL || odd == NULL || number < 6U || number > 14U ||
		(number&1U)!=0U || f->slot < 1U || f->slot > 31U ||
		f->satellite_type > 3U || f->healthy > 1U || f->frequency > 31U ||
		f->ln > 1U || f->eccentricity >= (UINT32_C(1)<<15) ||
		f->ascending_time >= (UINT32_C(1)<<21) ||
		set_unsigned(first,9U,15U,f->eccentricity)!=0 ||
		set_sign_magnitude(first,24U,18U,f->delta_i)!=0 ||
		set_sign_magnitude(first,42U,21U,f->lambda)!=0 ||
		set_sign_magnitude(first,63U,10U,f->tau)!=0 ||
		set_unsigned(first,73U,5U,f->slot)!=0 ||
		set_unsigned(first,78U,2U,f->satellite_type)!=0 ||
		set_unsigned(first,80U,1U,f->healthy)!=0 ||
		set_unsigned(first,81U,4U,number)!=0 ||
		set_unsigned(second,9U,1U,f->ln)!=0 ||
		set_unsigned(second,10U,5U,f->frequency)!=0 ||
		set_sign_magnitude(second,15U,7U,f->delta_t_rate)!=0 ||
		set_sign_magnitude(second,22U,22U,f->delta_t)!=0 ||
		set_unsigned(second,44U,21U,f->ascending_time)!=0 ||
		set_sign_magnitude(second,65U,16U,f->omega)!=0 ||
		set_unsigned(second,81U,4U,number+1U)!=0)
		return -1;
	return finish_string(first,even)==0 && finish_string(second,odd)==0 ? 0 : -1;
}

int gnss_glonass_gnav_frame(const glonass_gnav_immediate_t *immediate,
	const glonass_gnav_string5_t *time_data,
	const glonass_gnav_almanac_t almanacs[5], uint8_t frame[15][85])
{
	unsigned int pair;
	if (immediate==NULL || time_data==NULL || almanacs==NULL || frame==NULL ||
		gnss_glonass_gnav_immediate_strings(immediate,frame)!=0 ||
		gnss_glonass_gnav_string5(time_data,frame[4])!=0) return -1;
	for(pair=0U;pair<5U;pair++)
		if(gnss_glonass_gnav_almanac_pair(&almanacs[pair],6U+pair*2U,
			frame[5U+pair*2U],frame[6U+pair*2U])!=0) return -1;
	return 0;
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

static int leap_year(int year)
{
	return year%4 == 0 && (year%100 != 0 || year%400 == 0);
}

static int day_of_year(int year, int month, int day)
{
	static const int before_month[12] =
		{0,31,59,90,120,151,181,212,243,273,304,334};
	if (month < 1 || month > 12 || day < 1 || day > 31) return -1;
	return before_month[month-1] + day + (month > 2 && leap_year(year));
}

static int four_year_day(const gnss_calendar_time_t *time)
{
	int start = time->year;
	int total = 0, year;
	while (!leap_year(start)) start--;
	for (year=start; year<time->year; year++) total += leap_year(year) ? 366 : 365;
	return total + day_of_year(time->year,time->month,time->day);
}

int gnss_glonass_gnav_from_rinex(const gnss_nav_record_t *r,
	glonass_gnav_immediate_t *f)
{
	int32_t status, health_flags;
	int nt;
	unsigned int axis;
	double frame_time;
	if (r == NULL || f == NULL || r->system != GNSS_SYSTEM_GLONASS ||
		strcmp(r->message,"FDMA") != 0 ||
		r->model != GNSS_NAV_GLONASS_STATE_VECTOR || r->orbit_count < 16U ||
		r->prn == 0U || r->prn > 31U || !isfinite(r->clock_drift_rate)) return -1;
	memset(f,0,sizeof(*f));
	frame_time = fmod(r->clock_drift_rate,604800.0);
	if (frame_time < 0.0) frame_time += 604800.0;
	frame_time = fmod(frame_time,86400.0);
	f->tk_seconds = (uint32_t)llround(fmod(frame_time+10800.0,86400.0));
	if (fabs(fmod(frame_time+10800.0,86400.0)-f->tk_seconds) > 1.0e-3 ||
		(f->tk_seconds%30U)!=0U) return -1;
	f->tb = (uint8_t)((((unsigned int)r->toc.hour*3600U+
		(unsigned int)r->toc.minute*60U+(unsigned int)llround(r->toc.second)+10800U)
		%86400U)/900U);
	if (!isfinite(r->orbit[3]) || r->orbit[3] < 0.0 || r->orbit[3] > 1.0 ||
		fabs(r->orbit[3]-llround(r->orbit[3])) > 1.0e-6) return -1;
	f->bn = (uint8_t)((uint32_t)llround(r->orbit[3]) << 2);
	f->slot = (uint8_t)r->prn;
	nt = four_year_day(&r->toc);
	if (nt < 1 || nt > 1461) return -1;
	f->nt = (uint16_t)nt;
	if (!isfinite(r->orbit[12]) || !isfinite(r->orbit[14]) || !isfinite(r->orbit[15]))
		return -1;
	status=(int32_t)llround(r->orbit[12]); health_flags=(int32_t)llround(r->orbit[15]);
	if (status<0 || status>511 || health_flags<0 || health_flags>7 ||
		fabs(r->orbit[12]-status)>1e-6 || fabs(r->orbit[14]-llround(r->orbit[14]))>1e-6)
		return -1;
	f->mode=(uint8_t)((status>>7)&3); f->p4=(uint8_t)((status>>6)&1);
	f->p3=(uint8_t)((status>>5)&1); f->p2=(uint8_t)((status>>4)&1);
	f->p1=(uint8_t)((status>>2)&3); f->p=(uint8_t)(status&3);
	f->ft=(uint8_t)llround(r->orbit[14]); f->ln=(uint8_t)((health_flags>>2)&1);
	f->en=(uint8_t)llround(r->orbit[11]);
	if (f->ft>15U || f->en>31U || quantize(-r->clock_bias,30,&f->tau)!=0 ||
		quantize(r->clock_drift,40,&f->gamma)!=0 ||
		quantize(r->orbit[13],30,&f->delta_tau)!=0) return -1;
	for (axis=0U;axis<3U;axis++) {
		size_t base=axis*4U;
		if (quantize(r->orbit[base],11,&f->position[axis])!=0 ||
			quantize(r->orbit[base+1U],20,&f->velocity[axis])!=0 ||
			quantize(r->orbit[base+2U],30,&f->acceleration[axis])!=0)
			return -1;
	}
	return 0;
}
