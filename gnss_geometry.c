#include "gnss_geometry.h"

#include <math.h>
#include <string.h>

#include "gnss_orbit.h"
#include "gnss_time.h"
#include "gpssim.h"

#define LIGHT_SPEED 299792458.0
#define TWO_PI 6.28318530717958647693

static double wrap_week(double value)
{
	while(value>302400.0) value-=604800.0;
	while(value< -302400.0) value+=604800.0;
	return value;
}

static double signal_group_delay(const gnss_nav_record_t *r)
{
	/* RINEX A8/A13/A23: Galileo BGD E5b/E1 is orbit field 22 and
	 * BeiDou TGD1 B1/B3 is field 22.  The broadcast clock polynomial is
	 * referenced to the constellation clock; subtracting the signal delay
	 * gives the clock correction applicable to the generated ranging code. */
	if(r->system==GNSS_SYSTEM_GPS && r->orbit_count>22U && isfinite(r->orbit[22]))
		return r->orbit[22];
	if(r->system==GNSS_SYSTEM_GALILEO && r->orbit_count>22U && isfinite(r->orbit[22]))
		return r->orbit[22];
	if(r->system==GNSS_SYSTEM_BEIDOU && r->orbit_count>22U && isfinite(r->orbit[22]))
		return r->orbit[22];
	return 0.0;
}

int gnss_observe(const gnss_nav_record_t *r, double receive_sow,
	const double rx[3], const double rv[3], double carrier, double code_rate,
	unsigned int code_length, gnss_observation_t *o)
{
	double transmit=receive_sow-0.075,clock_bias=0.0,clock_drift=0.0,group_delay;
	double sat[3],vel[3],delta[3],range=0.0,travel,angle,c,s;
	double llh[3],matrix[3][3],neu[3],unit[3],relative_velocity[3];
	unsigned int iteration,axis;
	if(r==NULL || rx==NULL || rv==NULL || o==NULL || !isfinite(receive_sow) ||
		!isfinite(carrier) || carrier<=0.0 || !isfinite(code_rate) || code_rate<=0.0 ||
		code_length==0U) return -1;
	for(iteration=0U;iteration<4U;iteration++) {
		int status;
		if(r->system==GNSS_SYSTEM_GLONASS) {
			gpstime_t toc_gps,toc_system;
			if(gnss_calendar_to_gps(r->system,&r->toc,&toc_gps)!=0 ||
				gnss_gps_to_system_time(r->system,&toc_gps,&toc_system)!=0) return -1;
			status=gnss_propagate_glonass(r,wrap_week(transmit-toc_system.sec),
				sat,vel,&clock_bias,&clock_drift);
		} else status=gnss_propagate_kepler(r,transmit,sat,vel,&clock_bias,&clock_drift);
		if(status!=0) return -1;
		travel=receive_sow-transmit; angle=7.2921151467e-5*travel;
		c=cos(angle); s=sin(angle);
		{
			double x=c*sat[0]+s*sat[1],y=-s*sat[0]+c*sat[1];
			double vx=c*vel[0]+s*vel[1],vy=-s*vel[0]+c*vel[1];
			sat[0]=x; sat[1]=y; vel[0]=vx; vel[1]=vy;
		}
		range=0.0;
		for(axis=0U;axis<3U;axis++){delta[axis]=sat[axis]-rx[axis];range+=delta[axis]*delta[axis];}
		range=sqrt(range); transmit=receive_sow-range/LIGHT_SPEED;
	}
	if(!isfinite(range) || range<=0.0) return -1;
	for(axis=0U;axis<3U;axis++) {
		unit[axis]=delta[axis]/range;
		relative_velocity[axis]=vel[axis]-rv[axis];
	}
	memset(o,0,sizeof(*o)); memcpy(o->position,sat,sizeof(sat)); memcpy(o->velocity,vel,sizeof(vel));
	group_delay=signal_group_delay(r);
	clock_bias-=group_delay;
	o->geometric_range_m=range; o->clock_bias_s=clock_bias; o->clock_drift_sps=clock_drift;
	o->signal_group_delay_s=group_delay;o->transmit_sow=transmit;
	o->travel_time_s=receive_sow-transmit;
	o->pseudorange_m=range-LIGHT_SPEED*clock_bias;
	o->range_rate_mps=unit[0]*relative_velocity[0]+unit[1]*relative_velocity[1]+
		unit[2]*relative_velocity[2]-LIGHT_SPEED*clock_drift;
	xyz2llh(rx,llh); ltcmat(llh,matrix); ecef2neu(delta,matrix,neu);
	o->azimuth_rad=atan2(neu[1],neu[0]); if(o->azimuth_rad<0.0)o->azimuth_rad+=TWO_PI;
	o->elevation_rad=atan2(neu[2],hypot(neu[0],neu[1]));
	o->doppler_hz=-o->range_rate_mps*carrier/LIGHT_SPEED;
	o->code_phase_chips=fmod(o->pseudorange_m*code_rate/LIGHT_SPEED,(double)code_length);
	if(o->code_phase_chips<0.0)o->code_phase_chips+=(double)code_length;
	o->carrier_phase_rad=fmod(-TWO_PI*o->pseudorange_m*carrier/LIGHT_SPEED,TWO_PI);
	if(o->carrier_phase_rad<0.0)o->carrier_phase_rad+=TWO_PI;
	return isfinite(o->doppler_hz)&&isfinite(o->elevation_rad)?0:-1;
}
