#include "bladegps.h"

#include <limits.h>
#include <math.h>

#include "getch.h"

#define RF_CHANNELS MAX_CHAN
#define MAX_GNSS_PRN 63U
#define MAX_GNSS_CANDIDATES (GNSS_SYSTEM_COUNT*MAX_GNSS_PRN)

typedef struct {
	int ready;
	size_t record_index;
	int schedule_week;
	uint32_t schedule_sow;
	int8_t data_code[GALILEO_E1_CODE_LENGTH];
	int8_t pilot_code[GALILEO_E1_CODE_LENGTH];
	int8_t overlay[GALILEO_E1C_SECONDARY_LENGTH];
	int8_t symbols[BEIDOU_D2_CYCLE_SYMBOLS];
	size_t symbol_count;
	uint8_t glonass_previous_relative_bit;
} signal_store_t;

static double record_age(const gnss_nav_record_t *record, const gpstime_t *now)
{
	gpstime_t epoch;
	if(gnss_calendar_to_gps(record->system,&record->toc,&epoch)!=0)return HUGE_VAL;
	return gnss_time_difference(now,&epoch);
}

static int generation_finished(sim_t *sim)
{
	int finished;
	pthread_mutex_lock(&sim->gps.lock);
	finished=sim->finished;
	pthread_mutex_unlock(&sim->gps.lock);
	return finished;
}

static int record_healthy(const gnss_nav_record_t *r)
{
	double value;
	long encoded;
	if(r->system==GNSS_SYSTEM_GPS)value=r->orbit[21];
	else if(r->system==GNSS_SYSTEM_GALILEO)value=r->orbit[20];
	else if(r->system==GNSS_SYSTEM_BEIDOU)value=r->orbit[21];
	else value=r->orbit[3];
	if(!isfinite(value))return 0;
	encoded=lround(value);
	if(fabs(value-(double)encoded)>1.0e-6||encoded<0)return 0;
	/* Galileo RINEX health is a packed multi-signal field.  E1-B uses only
	 * DVS bit 0 and HS bits 1-2; E5a/E5b status must not suppress E1-B. */
	if(r->system==GNSS_SYSTEM_GALILEO)return encoded<=511L&&(encoded&7L)==0L;
	return encoded==0L;
}

static int matching(const gnss_nav_record_t *r, gnss_signal_t signal)
{
	if(signal==GNSS_SIGNAL_MIXED_OPEN)return
		(r->system==GNSS_SYSTEM_GPS&&strcmp(r->message,"LNAV")==0)||
		(r->system==GNSS_SYSTEM_GALILEO&&strcmp(r->message,"INAV")==0)||
		(r->system==GNSS_SYSTEM_BEIDOU&&(strcmp(r->message,"D1")==0||strcmp(r->message,"D2")==0))||
		(r->system==GNSS_SYSTEM_GLONASS&&strcmp(r->message,"FDMA")==0);
	if(signal==GNSS_SIGNAL_GPS_L1CA)return r->system==GNSS_SYSTEM_GPS&&strcmp(r->message,"LNAV")==0;
	if(signal==GNSS_SIGNAL_GALILEO_E1) return r->system==GNSS_SYSTEM_GALILEO&&strcmp(r->message,"INAV")==0;
	if(signal==GNSS_SIGNAL_BEIDOU_B1I) return r->system==GNSS_SYSTEM_BEIDOU&&
		(strcmp(r->message,"D1")==0||strcmp(r->message,"D2")==0);
	return signal==GNSS_SIGNAL_GLONASS_L1OF&&r->system==GNSS_SYSTEM_GLONASS&&strcmp(r->message,"FDMA")==0;
}

static gnss_signal_t record_signal(const gnss_nav_record_t *r)
{
	if(r->system==GNSS_SYSTEM_GPS)return GNSS_SIGNAL_GPS_L1CA;
	if(r->system==GNSS_SYSTEM_GALILEO)return GNSS_SIGNAL_GALILEO_E1;
	if(r->system==GNSS_SYSTEM_BEIDOU)return GNSS_SIGNAL_BEIDOU_B1I;
	return GNSS_SIGNAL_GLONASS_L1OF;
}

static int gps_ephemeris_from_record(const gnss_nav_record_t *r,ephem_t *e)
{
	gpstime_t toc;
	size_t index;
	long iode,iodc,health,week;
	if(r==NULL||e==NULL||r->system!=GNSS_SYSTEM_GPS||strcmp(r->message,"LNAV")!=0||
		r->orbit_count<24U||gnss_calendar_to_gps(r->system,&r->toc,&toc)!=0)return -1;
	for(index=0U;index<=16U;index++)if(!isfinite(r->orbit[index]))return -1;
	if(!isfinite(r->orbit[18])||!isfinite(r->orbit[20])||!isfinite(r->orbit[21])||
		!isfinite(r->orbit[22])||!isfinite(r->orbit[23]))return -1;
	iode=lround(r->orbit[0]);week=lround(r->orbit[18]);health=lround(r->orbit[21]);
	iodc=lround(r->orbit[23]);
	if(fabs(r->orbit[0]-(double)iode)>1.0e-6||iode<0||iode>255||
		fabs(r->orbit[18]-(double)week)>1.0e-6||week<0||week>INT_MAX||
		fabs(r->orbit[21]-(double)health)>1.0e-6||health<0||health>63||
		fabs(r->orbit[23]-(double)iodc)>1.0e-6||iodc<0||iodc>1023)return -1;
	memset(e,0,sizeof(*e));e->vflg=1;e->toc=toc;e->toe.week=(int)week;
	e->toe.sec=r->orbit[8];e->iode=(int)iode;e->iodc=(int)iodc;
	e->crs=r->orbit[1];e->deltan=r->orbit[2];e->m0=r->orbit[3];e->cuc=r->orbit[4];
	e->ecc=r->orbit[5];e->cus=r->orbit[6];e->sqrta=r->orbit[7];e->cic=r->orbit[9];
	e->omg0=r->orbit[10];e->cis=r->orbit[11];e->inc0=r->orbit[12];e->crc=r->orbit[13];
	e->aop=r->orbit[14];e->omgdot=r->orbit[15];e->idot=r->orbit[16];
	e->af0=r->clock_bias;e->af1=r->clock_drift;e->af2=r->clock_drift_rate;
	e->sv_accuracy=r->orbit[20];e->sv_health=(int)health;e->tgd=r->orbit[22];
	e->fit_interval=r->orbit_count>25U&&isfinite(r->orbit[25])&&r->orbit[25]>0.0?
		r->orbit[25]:DEFAULT_EPHEMERIS_FIT_HOURS;
	return e->toe.week>=0&&isfinite(e->toe.sec)&&e->toe.sec>=0.0&&e->toe.sec<SECONDS_IN_WEEK&&
		isfinite(e->sqrta)&&e->sqrta>0.0&&isfinite(e->ecc)&&e->ecc>=0.0&&e->ecc<1.0?0:-1;
}

static int build_gps_store(signal_store_t *s,const gnss_nav_record_t *r,int week,uint32_t sow)
{
	ephem_t ephemeris;channel_t channel;int ca[CA_SEQ_LEN];size_t bit;
	gpstime_t frame={(int)week,(double)sow};
	if(gps_ephemeris_from_record(r,&ephemeris)!=0)return -1;
	memset(&channel,0,sizeof(channel));codegen(ca,(int)r->prn);
	for(bit=0U;bit<CA_SEQ_LEN;bit++)s->data_code[bit]=(int8_t)(ca[bit]*2-1);
	eph2sbf(ephemeris,channel.sbf);if(generateNavMsg(frame,&channel,1)!=1)return -1;
	for(bit=0U;bit<1500U;bit++) {
		size_t word=10U+bit/30U,position=bit%30U;
		s->symbols[bit]=((channel.dwrd[word]>>(29U-position))&1UL)!=0UL?1:-1;
	}
	s->symbol_count=1500U;s->ready=1;return 0;
}

static int build_store(signal_store_t *s,const gnss_nav_record_t *r,
	gnss_signal_t signal,int week,uint32_t sow,const int8_t alpha[4],
	const int8_t beta[4])
{
	if(signal==GNSS_SIGNAL_GPS_L1CA)return build_gps_store(s,r,week,sow);
	if(signal==GNSS_SIGNAL_GALILEO_E1) {
		/* The current transmit week, not the ephemeris reference week, belongs
		 * in I/NAV word 5. GST's on-air 12-bit week started at GPS week 1024. */
		if(week<1024)return -1;
		week=(week-1024)%4096;
		if(gnss_galileo_e1_primary_code(r->prn,GALILEO_E1_COMPONENT_B,s->data_code)!=0||
			gnss_galileo_e1_primary_code(r->prn,GALILEO_E1_COMPONENT_C,s->pilot_code)!=0||
			gnss_galileo_e1c_secondary_code(s->overlay)!=0||
			gnss_schedule_galileo_e1(r,(unsigned int)week,sow,s->symbols)!=0)return -1;
		s->symbol_count=GALILEO_E1_CYCLE_SYMBOLS;
	} else if(signal==GNSS_SIGNAL_BEIDOU_B1I) {
		if(gnss_beidou_b1i_code(r->prn,s->data_code)!=0||gnss_beidou_b1i_nh_code(s->overlay)!=0)return -1;
		if(strcmp(r->message,"D1")==0) {
			if(gnss_schedule_beidou_d1(r,alpha,beta,sow,s->symbols)!=0)return -1;
			s->symbol_count=BEIDOU_D1_FRAME_SYMBOLS;
		} else {
			if(gnss_schedule_beidou_d2(r,alpha,beta,sow,s->symbols)!=0)return -1;
			s->symbol_count=BEIDOU_D2_CYCLE_SYMBOLS;
		}
	} else {
		gpstime_t frame_time={week,(double)sow};
		datetime_t utc;
		gnss_calendar_time_t utc_frame;
		gps2date(&frame_time,&utc);
		utc_frame=(gnss_calendar_time_t){utc.y,utc.m,utc.d,utc.hh,utc.mm,utc.sec};
		if(r->orbit_count<16U) {
			fprintf(stderr,"ERROR: GLONASS GNAV requires the 16 extended FDMA "
				"fields supplied by RINEX 4; R%02u has %zu fields.\n",
				r->prn,r->orbit_count);
			return -1;
		}
		if(gnss_glonass_l1of_code(s->data_code)!=0) {
			fprintf(stderr,"ERROR: GLONASS ranging-code construction failed.\n");
			return -1;
		}
		if(gnss_schedule_glonass(r,&utc_frame,
			&s->glonass_previous_relative_bit,s->symbols)!=0) {
			fprintf(stderr,"ERROR: GLONASS GNAV schedule rejected UTC frame "
				"%04d-%02d-%02dT%02d:%02d:%06.3f for R%02u.\n",
				utc_frame.year,utc_frame.month,utc_frame.day,utc_frame.hour,
				utc_frame.minute,utc_frame.second,r->prn);
			return -1;
		}
		s->symbol_count=GLONASS_GNAV_FRAME_SYMBOLS;
	}
	s->ready=1; return 0;
}

static void finish_task(sim_t *sim,int error)
{
	pthread_mutex_lock(&sim->gps.lock);
	if(error)sim->gps.error=-1;
	sim->finished=true; sim->gps.ready=1;
	pthread_cond_broadcast(&sim->gps.initialization_done);
	pthread_cond_broadcast(&sim->fifo_read_ready);
	pthread_cond_broadcast(&sim->fifo_write_ready);
	pthread_mutex_unlock(&sim->gps.lock);
}

void *gnss_task(void *argument)
{
	sim_t *sim=argument;
	gnss_nav_record_t *records=NULL;
	signal_store_t *stores=NULL;
	gnss_rf_channel_t active[RF_CHANNELS]={{0}},desired[RF_CHANNELS];
	gnss_rf_candidate_t candidates[MAX_GNSS_CANDIDATES];
	gnss_observation_t observations[MAX_GNSS_CANDIDATES];
	size_t candidate_records[MAX_GNSS_CANDIDATES];
	gnss_signal_t candidate_signals[MAX_GNSS_CANDIDATES];
	const gnss_signal_profile_t *profile=gnss_signal_profile(sim->opt.signal);
	double **xyz=NULL,*xyz_data=NULL,receiver_velocity[3]={0},live_offset[3]={0};
	int16_t *iq=NULL; size_t record_count=0U,selected[RF_CHANNELS],selected_count;
	int numd,step,error=1,direction=UNDEF; double interactive_velocity=0.0;
	gpstime_t time=sim->opt.g0,scenario_start;
	int8_t iono_alpha[4]={0},iono_beta[4]={0};
	motion_controller_t controller={0};
	const char *failure="unknown GNSS producer error";
	if(profile==NULL||!profile->waveform_implemented){failure="signal profile has no implemented baseband waveform";goto done;}
	stores=calloc(GNSS_SYSTEM_COUNT*(MAX_GNSS_PRN+1U),sizeof(*stores));
	xyz=malloc(USER_MOTION_SIZE*sizeof(*xyz)); xyz_data=calloc(USER_MOTION_SIZE*3U,sizeof(*xyz_data));
	iq=calloc(sim->iq_block_samples*2U,sizeof(*iq));
	if(!stores||!xyz||!xyz_data||!iq){failure="cannot allocate GNSS producer buffers";goto done;}
	for(step=0;step<USER_MOTION_SIZE;step++)xyz[step]=xyz_data+(size_t)step*3U;
	if(gnss_load_rinex_nav(sim->opt.navfile,&records,&record_count)!=0){failure="cannot parse supported ephemerides from RINEX 2/3/4 navigation file";goto done;}
	if(sim->opt.signal==GNSS_SIGNAL_BEIDOU_B1I||sim->opt.signal==GNSS_SIGNAL_MIXED_OPEN) {
		gnss_klobuchar_t model;
		int iono_status=gnss_read_beidou_ionosphere(sim->opt.navfile,&model);
		if(iono_status<0||(iono_status==0&&
			gnss_beidou_ionosphere_quantize(&model,iono_alpha,iono_beta)!=0)) {
			failure="invalid BeiDou ionosphere model in navigation file";goto done;
		}
	}
	if(sim->opt.staticLocationMode){llh2xyz(sim->opt.llh,xyz[0]);numd=sim->opt.iduration;
		for(step=1;step<numd;step++)memcpy(xyz[step],xyz[0],3U*sizeof(double));}
	else {numd=sim->opt.nmeaGGA?readNmeaGGA(xyz,sim->opt.umfile):
		(sim->opt.geodeticMotion?readLlhMotion(xyz,sim->opt.umfile):readUserMotion(xyz,sim->opt.umfile));
		if(numd<=0){failure="motion file contains no usable receiver positions";goto done;} if(numd>sim->opt.iduration)numd=sim->opt.iduration;}
	if(sim->opt.controller_index>=0&&motion_controller_open(&controller,sim->opt.controller_index)!=0){
		failure="cannot open requested SDL game controller";goto done;}
	if(time.week<0){size_t n;for(n=0;n<record_count&&!matching(&records[n],sim->opt.signal);n++);
		if(n==record_count){failure="navigation file has no records for the selected signal";goto done;}
		if(gnss_calendar_to_gps(records[n].system,&records[n].toc,&time)!=0){failure="invalid navigation epoch";goto done;}}
	{
		size_t n;int usable=0;
		for(n=0;n<record_count;n++)if(matching(&records[n],sim->opt.signal)&&record_healthy(&records[n])){
			const gnss_signal_profile_t *record_profile=gnss_signal_profile(record_signal(&records[n]));
			if(record_profile==NULL||records[n].prn>record_profile->maximum_sv)continue;
			double age=record_age(&records[n],&time);
			if(age < -30.0 || age > (records[n].system==GNSS_SYSTEM_GLONASS?1800.0:14400.0))continue;
			usable=1;break;
		}
		if(!usable){failure="navigation file has no healthy in-age record for the selected signal and start time";goto done;}
	}
	scenario_start=time;
	for(step=0;step<numd&&!generation_finished(sim);step++) {
		size_t best[GNSS_SYSTEM_COUNT][MAX_GNSS_PRN+1U],n,candidate_count=0U;
		double best_age[GNSS_SYSTEM_COUNT][MAX_GNSS_PRN+1U];gnss_system_t system;
		for(system=GNSS_SYSTEM_GPS;system<GNSS_SYSTEM_COUNT;system++)for(n=0;n<=MAX_GNSS_PRN;n++){
			best[system][n]=SIZE_MAX;best_age[system][n]=HUGE_VAL;}
		if(stop_was_requested())break;
		/* Derive each epoch from the immutable start and integer 10 Hz tick.
		 * Repeatedly adding binary 0.1 accumulated enough error in long runs to
		 * move exact data/page boundaries by one sample. */
		time=scenario_start;time.sec+=(double)step/10.0;normalizeGpsTime(&time);
		if(step>0&&(sim->opt.interactive||controller.active)) {
			double neu_velocity[3]={0.0,0.0,0.0};
			if(sim->opt.interactive) {
			int key_direction=UNDEF;
			if(_kbhit())switch(_getch()){case NORTH_KEY:key_direction=NORTH;break;
				case SOUTH_KEY:key_direction=SOUTH;break;case EAST_KEY:key_direction=EAST;break;
				case WEST_KEY:key_direction=WEST;break;case UP_KEY:key_direction=UP;break;
				case DOWN_KEY:key_direction=DOWN;break;default:break;}
			if(motion_keyboard_update(key_direction,&direction,&interactive_velocity,
				DEL_VEL,MAX_VEL)!=0){failure="invalid keyboard motion state";goto done;}
			if(direction==NORTH)neu_velocity[0]+=interactive_velocity;
			else if(direction==SOUTH)neu_velocity[0]-=interactive_velocity;
			else if(direction==EAST)neu_velocity[1]+=interactive_velocity;
			else if(direction==WEST)neu_velocity[1]-=interactive_velocity;
			else if(direction==UP)neu_velocity[2]+=interactive_velocity;
			else if(direction==DOWN)neu_velocity[2]-=interactive_velocity;
			}
			if(controller.active) {
				double controller_velocity[3];
				if(motion_controller_poll(&controller,MAX_VEL,MAX_VEL,controller_velocity)!=0){
					failure="game controller disconnected";goto done;}
				for(n=0;n<3U;n++)neu_velocity[n]+=controller_velocity[n];
			}
			{
				double llh[3],m[3][3],current[3];
				for(n=0;n<3U;n++)current[n]=xyz[step][n]+live_offset[n];
				xyz2llh(current,llh);ltcmat(llh,m);
				for(n=0;n<3U;n++)live_offset[n]+=0.1*(m[0][n]*neu_velocity[0]+
					m[1][n]*neu_velocity[1]+m[2][n]*neu_velocity[2]);
				for(n=0;n<3U;n++)xyz[step][n]+=live_offset[n];
			}
		}
		for(n=0;n<record_count;n++)if(matching(&records[n],sim->opt.signal)&&record_healthy(&records[n])){
			const gnss_signal_profile_t *record_profile=gnss_signal_profile(record_signal(&records[n]));
			if(record_profile==NULL||records[n].prn>record_profile->maximum_sv)continue;
			double age=record_age(&records[n],&time);
			/* Prefer the newest healthy record already in force.  A small future
			 * tolerance accommodates files whose integer epoch is rounded at a
			 * message boundary without selecting an arbitrary future ephemeris. */
			if((age>=0.0&&(best[records[n].system][records[n].prn]==SIZE_MAX||
				best_age[records[n].system][records[n].prn]<0.0||
				age<best_age[records[n].system][records[n].prn]))||
				(age<0.0&&age>=-30.0&&
				(best[records[n].system][records[n].prn]==SIZE_MAX||
				(best_age[records[n].system][records[n].prn]<0.0&&
				age>best_age[records[n].system][records[n].prn])))){
				best_age[records[n].system][records[n].prn]=age;
				best[records[n].system][records[n].prn]=n;
			}
		}
		if(step>0)for(n=0;n<3U;n++)receiver_velocity[n]=(xyz[step][n]-xyz[step-1][n])*10.0;
		for(system=GNSS_SYSTEM_GPS;system<GNSS_SYSTEM_COUNT;system++)for(n=1;n<=MAX_GNSS_PRN;n++)
			if(best[system][n]!=SIZE_MAX&&best_age[system][n]>=-30.0&&best_age[system][n]<=
			(records[best[system][n]].system==GNSS_SYSTEM_GLONASS?1800.0:14400.0)){
			size_t record_index=best[system][n];gnss_signal_t signal=record_signal(&records[record_index]);
			const gnss_signal_profile_t *channel_profile=gnss_signal_profile(signal);
			double carrier=channel_profile->carrier_hz,occupied=channel_profile->occupied_bandwidth_hz;
			if(signal==GNSS_SIGNAL_GLONASS_L1OF){double raw_slot=records[record_index].orbit[7];int slot;
				long long rounded_slot;
				if(!isfinite(raw_slot))continue;
				rounded_slot=llround(raw_slot);
				if(fabs(raw_slot-(double)rounded_slot)>1.0e-6||rounded_slot<INT_MIN||rounded_slot>INT_MAX)continue;
				slot=(int)rounded_slot;if(gnss_glonass_l1of_carrier_hz(slot,&carrier)!=0)continue;}
			if(signal==GNSS_SIGNAL_GLONASS_L1OF)occupied=2.2*channel_profile->code_rate_hz;
			gpstime_t system_time;
			if(gnss_gps_to_system_time(records[record_index].system,&time,&system_time)!=0)continue;
			if(gnss_observe(&records[record_index],system_time.sec,xyz[step],receiver_velocity,carrier,
				channel_profile->code_rate_hz,channel_profile->code_length,&observations[candidate_count])!=0)continue;
			candidates[candidate_count]=(gnss_rf_candidate_t){records[record_index].system,(unsigned int)n,carrier,
				occupied,observations[candidate_count].elevation_rad,1};
			candidate_records[candidate_count]=record_index;candidate_signals[candidate_count]=signal;candidate_count++;}
		if(gnss_rf_allocate(candidates,candidate_count,sim->opt.tx_frequency,sim->opt.tx_sample_rate,
			sim->opt.elevation_mask/R2D,selected,RF_CHANNELS,&selected_count)!=0){failure="RF channel allocation failed";goto done;}
		for(n=0;n<selected_count;n++){size_t c=selected[n],ri=candidate_records[c];gnss_signal_t signal=candidate_signals[c];
			const gnss_signal_profile_t *channel_profile=gnss_signal_profile(signal);
			signal_store_t *st=&stores[(size_t)records[ri].system*(MAX_GNSS_PRN+1U)+candidates[c].prn];
			gpstime_t transmit_time;
			double transmit_sow;
			double rate_scale=1.0+observations[c].doppler_hz/candidates[c].carrier_hz;
			gpstime_t system_time;
			if(gnss_gps_to_system_time(records[ri].system,&time,&system_time)!=0){failure="time-scale conversion failed";goto done;}
			transmit_time.week=system_time.week;
			transmit_time.sec=observations[c].transmit_sow;
			normalizeGpsTime(&transmit_time);
			transmit_sow=transmit_time.sec;
			uint32_t schedule_sow=(uint32_t)(floor(transmit_sow/30.0)*30.0);
			double overlay_rate=signal==GNSS_SIGNAL_GALILEO_E1?250.0:1000.0;
			size_t overlay_count=signal==GNSS_SIGNAL_GALILEO_E1?GALILEO_E1C_SECONDARY_LENGTH:
				signal==GNSS_SIGNAL_BEIDOU_B1I&&strcmp(records[ri].message,"D1")==0?BEIDOU_B1I_NH_LENGTH:0U;
			if(schedule_sow>=604800U){failure="normalized transmit time is outside its week";goto done;}
			if((!st->ready||st->record_index!=ri||st->schedule_week!=transmit_time.week||st->schedule_sow!=schedule_sow)&&
				build_store(st,&records[ri],signal,transmit_time.week,schedule_sow,
					iono_alpha,iono_beta)!=0){failure="navigation-message cycle construction failed";goto done;}
			st->record_index=ri;
			st->schedule_week=transmit_time.week;st->schedule_sow=schedule_sow;
			desired[n]=(gnss_rf_channel_t){1,signal==GNSS_SIGNAL_GALILEO_E1?GNSS_RF_GALILEO_E1:GNSS_RF_BPSK,
				records[ri].system,candidates[c].prn,candidates[c].carrier_hz,observations[c].doppler_hz,
				signal==GNSS_SIGNAL_GALILEO_E1?90.0:120.0,
				st->data_code,signal==GNSS_SIGNAL_GALILEO_E1?st->pilot_code:NULL,channel_profile->code_length,
				channel_profile->code_rate_hz*rate_scale,observations[c].code_phase_chips,
				st->symbols,st->symbol_count,(signal==GNSS_SIGNAL_BEIDOU_B1I&&strcmp(records[ri].message,"D2")==0?500.0:
				(signal==GNSS_SIGNAL_GALILEO_E1?250.0:signal==GNSS_SIGNAL_BEIDOU_B1I?50.0:signal==GNSS_SIGNAL_GPS_L1CA?50.0:100.0))*rate_scale,
				fmod(transmit_sow*(signal==GNSS_SIGNAL_GALILEO_E1?250.0:signal==GNSS_SIGNAL_BEIDOU_B1I&&strcmp(records[ri].message,"D2")==0?500.0:signal==GNSS_SIGNAL_BEIDOU_B1I?50.0:signal==GNSS_SIGNAL_GPS_L1CA?50.0:100.0),(double)st->symbol_count),
				signal==GNSS_SIGNAL_GALILEO_E1||signal==GNSS_SIGNAL_BEIDOU_B1I?st->overlay:NULL,
				overlay_count,overlay_rate*rate_scale,overlay_count>0U?fmod(transmit_sow*overlay_rate,(double)overlay_count):0.0,
				observations[c].carrier_phase_rad};}
		if(gnss_rf_reconcile(active,RF_CHANNELS,desired,selected_count)!=0||
			gnss_rf_render(active,RF_CHANNELS,sim->opt.tx_frequency,sim->opt.tx_sample_rate,iq,sim->iq_block_samples)!=0){failure="RF channel reconciliation or rendering failed";goto done;}
		pthread_mutex_lock(&sim->gps.lock);if(!sim->gps.ready){sim->gps.ready=1;pthread_cond_signal(&sim->gps.initialization_done);}
		while(!is_fifo_write_ready(sim)&&!sim->finished)pthread_cond_wait(&sim->fifo_write_ready,&sim->gps.lock);
		if(sim->finished){pthread_mutex_unlock(&sim->gps.lock);break;}
		memcpy(&sim->fifo[sim->head*2],iq,sim->iq_block_samples*2U*sizeof(*iq));sim->head+=(long)sim->iq_block_samples;
		if((size_t)sim->head>=sim->fifo_length)sim->head-=(long)sim->fifo_length;
		pthread_cond_signal(&sim->fifo_read_ready);pthread_mutex_unlock(&sim->gps.lock);
	}
	error=0;
done:
	if(error)fprintf(stderr,"ERROR: %s.\n",failure);
	motion_controller_close(&controller);
	free(iq);free(xyz_data);free(xyz);free(stores);free(records);finish_task(sim,error);return NULL;
}
