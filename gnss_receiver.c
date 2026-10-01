#include "gnss_receiver.h"

#include <float.h>
#include <math.h>

#define TWO_PI 6.28318530717958647693

int gnss_rx_acquire_bpsk(const int16_t *iq, size_t samples, double sample_rate,
	const int8_t *code, size_t code_length, double code_rate,
	double minimum_carrier, double maximum_carrier, double carrier_step,
	gnss_rx_acquisition_t *result)
{
	double energy=0.0,best=-DBL_MAX,frequency;
	size_t sample,phase;
	if(iq==NULL||code==NULL||result==NULL||samples==0U||code_length==0U||
		!isfinite(sample_rate)||sample_rate<=0.0||!isfinite(code_rate)||code_rate<=0.0||
		!isfinite(minimum_carrier)||!isfinite(maximum_carrier)||
		!isfinite(carrier_step)||carrier_step<=0.0||minimum_carrier>maximum_carrier)
		return -1;
	for(phase=0U;phase<code_length;phase++)if(code[phase]!=-1&&code[phase]!=1)return -1;
	for(sample=0U;sample<samples;sample++)
		energy+=hypot((double)iq[2U*sample],(double)iq[2U*sample+1U]);
	if(energy<=0.0)return -1;
	result->normalized_correlation=0.0;
	for(frequency=minimum_carrier;frequency<=maximum_carrier+0.5*carrier_step;
		frequency+=carrier_step) {
		for(phase=0U;phase<code_length;phase++) {
			double real=0.0,imaginary=0.0;
			for(sample=0U;sample<samples;sample++) {
				double angle=-TWO_PI*frequency*(double)sample/sample_rate;
				size_t chip=((size_t)floor((double)sample*code_rate/sample_rate)+phase)%code_length;
				double local=(double)code[chip],c=cos(angle),s=sin(angle);
				double in_phase=(double)iq[2U*sample],quadrature=(double)iq[2U*sample+1U];
				real+=local*(in_phase*c-quadrature*s);
				imaginary+=local*(in_phase*s+quadrature*c);
			}
			{
				double magnitude=hypot(real,imaginary);
				if(magnitude>best){best=magnitude;result->carrier_offset_hz=frequency;
					result->code_phase_chips=(unsigned int)phase;}
			}
		}
	}
	result->normalized_correlation=best/energy;
	return isfinite(result->normalized_correlation)&&result->normalized_correlation>=0.0&&
		result->normalized_correlation<=1.000001?0:-1;
}
