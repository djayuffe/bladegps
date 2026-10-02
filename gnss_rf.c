#include "gnss_rf.h"

#include <stdint.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "gnss_codes.h"

#define TWO_PI 6.28318530717958647693

typedef struct {
	double i;
	double q;
	double rotate_i;
	double rotate_q;
} rf_oscillator_t;

static int symbols_valid(const int8_t *symbols, size_t count)
{
	size_t index;
	if (symbols==NULL || count==0U) return 0;
	for(index=0U;index<count;index++) if(symbols[index]!=-1 && symbols[index]!=1) return 0;
	return 1;
}

int gnss_rf_validate_channel(const gnss_rf_channel_t *c, double center,
	double sample_rate)
{
	double bandwidth;
	if(c==NULL || !c->enabled ||
		(c->modulation!=GNSS_RF_BPSK && c->modulation!=GNSS_RF_GALILEO_E1) ||
		c->system<0 || c->system>=GNSS_SYSTEM_COUNT ||
		c->prn==0U || !isfinite(c->carrier_hz) || !isfinite(c->doppler_hz) ||
		!isfinite(c->amplitude) || c->amplitude<0.0 || !isfinite(c->code_rate_hz) ||
		c->code_rate_hz<=0.0 || !symbols_valid(c->data_code,c->code_length) ||
		!isfinite(c->code_phase) || c->code_phase<0.0 ||
		c->code_phase>=(double)c->code_length ||
		!symbols_valid(c->data_symbols,c->data_symbol_count) ||
		!isfinite(c->data_rate_hz) || c->data_rate_hz<=0.0 ||
		!isfinite(c->data_phase) || c->data_phase<0.0 ||
		c->data_phase>=(double)c->data_symbol_count ||
		!isfinite(c->carrier_phase) || c->carrier_phase<0.0 || c->carrier_phase>=TWO_PI)
		return -1;
	if(c->overlay_symbol_count>0U && (!symbols_valid(c->overlay_symbols,
		c->overlay_symbol_count) || !isfinite(c->overlay_rate_hz) ||
		c->overlay_rate_hz<=0.0 || !isfinite(c->overlay_phase) ||
		c->overlay_phase<0.0 || c->overlay_phase>=(double)c->overlay_symbol_count)) return -1;
	if(c->modulation==GNSS_RF_GALILEO_E1 &&
		(!symbols_valid(c->pilot_code,c->code_length) || c->system!=GNSS_SYSTEM_GALILEO))
		return -1;
	bandwidth=c->modulation==GNSS_RF_GALILEO_E1?
		GNSS_GALILEO_E1_REFERENCE_BANDWIDTH_HZ:2.2*c->code_rate_hz;
	return gnss_frequency_fits(center,sample_rate,c->carrier_hz+c->doppler_hz,bandwidth)?0:-1;
}

static void advance(double *phase, double step, double period)
{
	*phase += step;
	if(*phase>=period || *phase<0.0) *phase -= floor(*phase/period)*period;
}

static int16_t sample16(double value)
{
	long result;
	if(value>2047.0) value=2047.0;
	if(value<-2048.0) value=-2048.0;
	result=lrint(value);
	return (int16_t)result;
}

int gnss_rf_render(gnss_rf_channel_t *channels, size_t count, double center,
	double sample_rate, int16_t *iq, size_t samples)
{
	size_t channel,sample;
	double normalization=1.0,peak_bound=0.0;
	rf_oscillator_t *oscillators;
	if(channels==NULL || count==0U || iq==NULL || samples==0U ||
		!isfinite(center) || !isfinite(sample_rate) || sample_rate<=0.0 ||
		samples>SIZE_MAX/(2U*sizeof(*iq)) || count>SIZE_MAX/sizeof(*oscillators)) return -1;
	oscillators=calloc(count,sizeof(*oscillators));
	if(oscillators==NULL)return -1;
	for(channel=0U;channel<count;channel++) if(channels[channel].enabled) {
		double phase_step;
		if(gnss_rf_validate_channel(&channels[channel],center,sample_rate)!=0){free(oscillators);return -1;}
		peak_bound+=channels[channel].amplitude*
			(channels[channel].modulation==GNSS_RF_GALILEO_E1?
				(sqrt(2.0)*sqrt(10.0/11.0)):1.0);
		phase_step=TWO_PI*(channels[channel].carrier_hz+channels[channel].doppler_hz-center)/sample_rate;
		oscillators[channel].i=cos(channels[channel].carrier_phase);
		oscillators[channel].q=sin(channels[channel].carrier_phase);
		oscillators[channel].rotate_i=cos(phase_step);
		oscillators[channel].rotate_q=sin(phase_step);
	}
	/* Preserve at least 1 dB of deterministic headroom in SC16 Q11.  The
	 * scale is fixed for a channel bank, so it cannot introduce AGC pumping. */
	if(peak_bound>1800.0) normalization=1800.0/peak_bound;
	memset(iq,0,samples*2U*sizeof(*iq));
	for(sample=0U;sample<samples;sample++) {
		double i=0.0,q=0.0;
		for(channel=0U;channel<count;channel++) {
			gnss_rf_channel_t *c=&channels[channel];
			rf_oscillator_t *osc=&oscillators[channel];
			double base,data,signal,next_i,next_q;
			size_t code_index,data_index;
			if(!c->enabled) continue;
			code_index=(size_t)c->code_phase%c->code_length;
			data_index=(size_t)c->data_phase%c->data_symbol_count;
			data=c->data_symbols[data_index];
			base=c->data_code[code_index];
			if(c->modulation==GNSS_RF_GALILEO_E1) {
				unsigned int sub=(unsigned int)((c->code_phase-floor(c->code_phase))*12.0);
				if(sub>=12U) sub=11U;
				double boc11=sub<6U?1.0:-1.0, boc61=(sub&1U)?-1.0:1.0;
				double plus=sqrt(10.0/11.0)*boc11+sqrt(1.0/11.0)*boc61;
				double minus=sqrt(10.0/11.0)*boc11-sqrt(1.0/11.0)*boc61;
				double secondary=1.0;
				if(c->overlay_symbol_count>0U)
					secondary=c->overlay_symbols[(size_t)c->overlay_phase%c->overlay_symbol_count];
				/* Galileo OS SIS ICD 2.2 Eq. 12: E1-B and E1-C share
				 * power equally and the pilot component has opposite sign. */
				signal=(base*data*plus-c->pilot_code[code_index]*secondary*minus)/sqrt(2.0);
			} else {
				double overlay=1.0;
				if(c->overlay_symbol_count>0U)
					overlay=c->overlay_symbols[(size_t)c->overlay_phase%c->overlay_symbol_count];
				signal=base*data*overlay;
			}
			i += normalization*c->amplitude*signal*osc->i;
			q += normalization*c->amplitude*signal*osc->q;
			advance(&c->carrier_phase,TWO_PI*(c->carrier_hz+c->doppler_hz-center)/sample_rate,TWO_PI);
			next_i=osc->i*osc->rotate_i-osc->q*osc->rotate_q;
			next_q=osc->q*osc->rotate_i+osc->i*osc->rotate_q;
			osc->i=next_i;osc->q=next_q;
			/* Bound recurrence drift without putting trigonometry back into the
			 * per-sample hot path. c->carrier_phase remains the canonical state. */
			if((sample&4095U)==4095U){osc->i=cos(c->carrier_phase);osc->q=sin(c->carrier_phase);}
			advance(&c->code_phase,c->code_rate_hz/sample_rate,(double)c->code_length);
			advance(&c->data_phase,c->data_rate_hz/sample_rate,(double)c->data_symbol_count);
			if(c->overlay_symbol_count>0U)
				advance(&c->overlay_phase,c->overlay_rate_hz/sample_rate,(double)c->overlay_symbol_count);
		}
		iq[sample*2U]=sample16(i); iq[sample*2U+1U]=sample16(q);
	}
	free(oscillators);
	return 0;
}

int gnss_rf_allocate(const gnss_rf_candidate_t *candidates, size_t count,
	double center, double sample_rate, double mask, size_t *selected,
	size_t capacity, size_t *selected_count)
{
	size_t used=0U,index;
	if(candidates==NULL || selected==NULL || selected_count==NULL || capacity==0U ||
		!isfinite(center) || !isfinite(sample_rate) || !isfinite(mask) ||
		sample_rate<=0.0) return -1;
	for(index=0U;index<count;index++) {
		size_t insert;
		if(candidates[index].system<0 || candidates[index].system>=GNSS_SYSTEM_COUNT ||
			!candidates[index].healthy || !isfinite(candidates[index].elevation_rad) ||
			candidates[index].elevation_rad<mask || candidates[index].prn==0U ||
			!gnss_frequency_fits(center,sample_rate,candidates[index].carrier_hz,
				candidates[index].occupied_bandwidth_hz)) continue;
		if(used==capacity && candidates[index].elevation_rad<=
			candidates[selected[capacity-1U]].elevation_rad) continue;
		insert=used<capacity?used:capacity-1U;
		while(insert>0U && candidates[selected[insert-1U]].elevation_rad<
			candidates[index].elevation_rad) {
			if(insert<capacity) selected[insert]=selected[insert-1U];
			insert--;
		}
		if(insert<capacity) selected[insert]=index;
		if(used<capacity) used++;
	}
	*selected_count=used; return 0;
}

static int same_signal(const gnss_rf_channel_t *a, const gnss_rf_channel_t *b)
{
	return a->enabled && b->enabled && a->system==b->system && a->prn==b->prn &&
		a->modulation==b->modulation && a->carrier_hz==b->carrier_hz &&
		a->code_length==b->code_length &&
		a->data_symbol_count==b->data_symbol_count &&
		a->overlay_symbol_count==b->overlay_symbol_count;
}

int gnss_rf_reconcile(gnss_rf_channel_t *active, size_t capacity,
	const gnss_rf_channel_t *desired, size_t count)
{
	gnss_rf_channel_t *next;
	size_t index,old,other;
	if(active==NULL || capacity==0U || count>capacity ||
		(count>0U && desired==NULL) || desired==active) return -1;
	for(index=0U;index<count;index++) {
		if(!desired[index].enabled) return -1;
		for(other=0U;other<index;other++)
			if(desired[index].system==desired[other].system &&
				desired[index].prn==desired[other].prn &&
				desired[index].carrier_hz==desired[other].carrier_hz) return -1;
	}
	next=calloc(capacity,sizeof(*next));
	if(next==NULL) return -1;
	for(index=0U;index<count;index++) {
		next[index]=desired[index];
		for(old=0U;old<capacity;old++) if(same_signal(&active[old],&desired[index])) {
			next[index].carrier_phase=active[old].carrier_phase;
			next[index].code_phase=active[old].code_phase;
			next[index].data_phase=active[old].data_phase;
			next[index].overlay_phase=active[old].overlay_phase;
			break;
		}
	}
	memcpy(active,next,capacity*sizeof(*active));
	free(next);
	return 0;
}

int gnss_rf_bits_to_symbols(const uint8_t *bits, size_t count, int8_t *symbols)
{
	size_t index;
	if(bits==NULL || symbols==NULL || count==0U) return -1;
	for(index=0U;index<count;index++) {
		if(bits[index]>1U) return -1;
		symbols[index]=bits[index]!=0U?-1:1;
	}
	return 0;
}

int gnss_glonass_l1of_symbols(const uint8_t string[85],
	uint8_t *previous_relative_bit, int8_t symbols[GLONASS_L1OF_STRING_SYMBOLS])
{
	int8_t time_mark[GLONASS_TIME_MARK_LENGTH];
	uint8_t previous;
	size_t bit;
	if(string==NULL || previous_relative_bit==NULL || symbols==NULL ||
		*previous_relative_bit>1U || gnss_glonass_time_mark(time_mark)!=0) return -1;
	previous=*previous_relative_bit;
	for(bit=0U;bit<85U;bit++) {
		uint8_t relative;
		if(string[bit]>1U) return -1;
		relative=(uint8_t)(string[bit]^previous);
		/* Modulo-2 addition with the 100 Hz meander 0,1. */
		symbols[bit*2U]=relative!=0U?-1:1;
		symbols[bit*2U+1U]=relative!=0U?1:-1;
		previous=relative;
	}
	memcpy(symbols+170U,time_mark,sizeof(time_mark));
	*previous_relative_bit=previous;
	return 0;
}
