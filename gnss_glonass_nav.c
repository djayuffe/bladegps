#include "gnss_glonass_nav.h"

#include <limits.h>
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
