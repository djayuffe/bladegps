#include "bladegps.h"

#include <math.h>

#include "getch.h"

#define RF_CHANNELS MAX_CHAN
#define MAX_GNSS_PRN 63U

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
} signal_store_t;

static double record_age(const gnss_nav_record_t *record, const gpstime_t *now)
{
	datetime_t date={record->toc.year,record->toc.month,record->toc.day,
		record->toc.hour,record->toc.minute,record->toc.second};
	gpstime_t epoch;
	date2gps(&date,&epoch);
	return fabs(((double)now->week-(double)epoch.week)*604800.0+
		now->sec-epoch.sec);
}

static int record_healthy(const gnss_nav_record_t *r)
{
	double value;
	long encoded;
	if(r->system==GNSS_SYSTEM_GALILEO)value=r->orbit[20];
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
	if(signal==GNSS_SIGNAL_GALILEO_E1) return r->system==GNSS_SYSTEM_GALILEO&&strcmp(r->message,"INAV")==0;
	if(signal==GNSS_SIGNAL_BEIDOU_B1I) return r->system==GNSS_SYSTEM_BEIDOU&&
		(strcmp(r->message,"D1")==0||strcmp(r->message,"D2")==0);
	return signal==GNSS_SIGNAL_GLONASS_L1OF&&r->system==GNSS_SYSTEM_GLONASS&&strcmp(r->message,"FDMA")==0;
}

static int build_store(signal_store_t *s,const gnss_nav_record_t *r,
	gnss_signal_t signal,int week,uint32_t sow,const int8_t alpha[4],
	const int8_t beta[4])
{
	if(signal==GNSS_SIGNAL_GALILEO_E1) {
		if(isfinite(r->orbit[18]))week=(int)llround(r->orbit[18]);
		/* RINEX GAL week is continuous and GPS-aligned; the on-air 12-bit
		 * GST week started at continuous GPS week 1024. */
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
		if(gnss_glonass_l1of_code(s->data_code)!=0||gnss_schedule_glonass(r,s->symbols)!=0)return -1;
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
	gnss_rf_candidate_t candidates[MAX_GNSS_PRN];
	gnss_observation_t observations[MAX_GNSS_PRN];
	size_t candidate_records[MAX_GNSS_PRN];
	const gnss_signal_profile_t *profile=gnss_signal_profile(sim->opt.signal);
	double **xyz=NULL,*xyz_data=NULL,receiver_velocity[3]={0};
	int16_t *iq=NULL; size_t record_count=0U,selected[RF_CHANNELS],selected_count;
	int numd,step,error=1,direction=UNDEF; double interactive_velocity=0.0; gpstime_t time=sim->opt.g0;
	int8_t iono_alpha[4]={0},iono_beta[4]={0};
	const char *failure="unknown non-GPS producer error";
	if(profile==NULL||sim->opt.signal==GNSS_SIGNAL_GPS_L1CA){failure="invalid non-GPS signal profile";goto done;}
	stores=calloc(MAX_GNSS_PRN+1U,sizeof(*stores));
	xyz=malloc(USER_MOTION_SIZE*sizeof(*xyz)); xyz_data=calloc(USER_MOTION_SIZE*3U,sizeof(*xyz_data));
	iq=calloc(sim->iq_block_samples*2U,sizeof(*iq));
	if(!stores||!xyz||!xyz_data||!iq){failure="cannot allocate non-GPS producer buffers";goto done;}
	for(step=0;step<USER_MOTION_SIZE;step++)xyz[step]=xyz_data+(size_t)step*3U;
	if(gnss_load_rinex_nav(sim->opt.navfile,&records,&record_count)!=0){failure="cannot parse supported ephemerides from RINEX 3/4 navigation file";goto done;}
	if(sim->opt.signal==GNSS_SIGNAL_BEIDOU_B1I) {
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
	if(time.week<0){datetime_t d={0}; size_t n; for(n=0;n<record_count&&!matching(&records[n],sim->opt.signal);n++);
		if(n==record_count){failure="navigation file has no records for the selected signal";goto done;} d.y=records[n].toc.year;d.m=records[n].toc.month;d.d=records[n].toc.day;
		d.hh=records[n].toc.hour;d.mm=records[n].toc.minute;d.sec=records[n].toc.second;date2gps(&d,&time);}
	{
		size_t n;int usable=0;
		for(n=0;n<record_count;n++)if(matching(&records[n],sim->opt.signal)&&
			records[n].prn<=MAX_GNSS_PRN&&record_healthy(&records[n])&&
			record_age(&records[n],&time)<=(records[n].system==GNSS_SYSTEM_GLONASS?1800.0:14400.0)){
			usable=1;break;
		}
		if(!usable){failure="navigation file has no healthy in-age record for the selected signal and start time";goto done;}
	}
	for(step=0;step<numd&&!sim->finished;step++) {
		size_t best[MAX_GNSS_PRN+1U],n,candidate_count=0U; double best_age[MAX_GNSS_PRN+1U];
		for(n=0;n<=MAX_GNSS_PRN;n++){best[n]=SIZE_MAX;best_age[n]=HUGE_VAL;}
		if(stop_was_requested())break;
		if(sim->opt.interactive) {
			int key_direction=UNDEF;
			if(_kbhit())switch(_getch()){case NORTH_KEY:key_direction=NORTH;break;
				case SOUTH_KEY:key_direction=SOUTH;break;case EAST_KEY:key_direction=EAST;break;
				case WEST_KEY:key_direction=WEST;break;case UP_KEY:key_direction=UP;break;
				case DOWN_KEY:key_direction=DOWN;break;default:break;}
			if(key_direction!=UNDEF&&direction==key_direction){if(interactive_velocity<MAX_VEL)interactive_velocity+=DEL_VEL;}
			else if(interactive_velocity>=0.0)interactive_velocity-=DEL_VEL;else direction=key_direction;
			if(step>0)memcpy(xyz[step],xyz[step-1],3U*sizeof(double));
			if(step>0&&direction!=UNDEF&&interactive_velocity>=0.0){double llh[3],m[3][3],neu[3]={0,0,0};
				xyz2llh(xyz[step-1],llh);ltcmat(llh,m);
				if(direction==NORTH)neu[0]=interactive_velocity*0.1;else if(direction==SOUTH)neu[0]=-interactive_velocity*0.1;
				else if(direction==EAST)neu[1]=interactive_velocity*0.1;else if(direction==WEST)neu[1]=-interactive_velocity*0.1;
				else if(direction==UP)neu[2]=interactive_velocity*0.1;else if(direction==DOWN)neu[2]=-interactive_velocity*0.1;
				for(n=0;n<3U;n++)xyz[step][n]+=m[0][n]*neu[0]+m[1][n]*neu[1]+m[2][n]*neu[2];}
		}
		for(n=0;n<record_count;n++)if(matching(&records[n],sim->opt.signal)&&records[n].prn<=MAX_GNSS_PRN){
			double age=record_age(&records[n],&time);if(age<best_age[records[n].prn]){best_age[records[n].prn]=age;best[records[n].prn]=n;}}
		if(step>0)for(n=0;n<3U;n++)receiver_velocity[n]=(xyz[step][n]-xyz[step-1][n])*10.0;
		for(n=1;n<=MAX_GNSS_PRN;n++)if(best[n]!=SIZE_MAX&&record_healthy(&records[best[n]])&&
			best_age[n]<=(records[best[n]].system==GNSS_SYSTEM_GLONASS?1800.0:14400.0)){double carrier=profile->carrier_hz;
			double occupied=profile->recommended_bandwidth_hz;
			if(sim->opt.signal==GNSS_SIGNAL_GLONASS_L1OF){double raw_slot=records[best[n]].orbit[7];int slot;
				if(!isfinite(raw_slot)||fabs(raw_slot-llround(raw_slot))>1.0e-6)continue;
				slot=(int)llround(raw_slot);if(gnss_glonass_l1of_carrier_hz(slot,&carrier)!=0)continue;}
			if(sim->opt.signal==GNSS_SIGNAL_GLONASS_L1OF)occupied=2.2*profile->code_rate_hz;
			if(gnss_observe(&records[best[n]],time.sec,xyz[step],receiver_velocity,carrier,
				profile->code_rate_hz,profile->code_length,&observations[candidate_count])!=0)continue;
			candidates[candidate_count]=(gnss_rf_candidate_t){profile->system,(unsigned int)n,carrier,
				occupied,observations[candidate_count].elevation_rad,1};
			candidate_records[candidate_count]=best[n];candidate_count++;}
		if(gnss_rf_allocate(candidates,candidate_count,sim->opt.tx_frequency,sim->opt.tx_sample_rate,
			sim->opt.elevation_mask/R2D,selected,RF_CHANNELS,&selected_count)!=0){failure="RF channel allocation failed";goto done;}
		for(n=0;n<selected_count;n++){size_t c=selected[n],ri=candidate_records[c];signal_store_t *st=&stores[candidates[c].prn];
			uint32_t schedule_sow=(uint32_t)(floor((time.sec+1.0e-7)/30.0)*30.0);
			double overlay_rate=sim->opt.signal==GNSS_SIGNAL_GALILEO_E1?250.0:1000.0;
			size_t overlay_count=sim->opt.signal==GNSS_SIGNAL_GALILEO_E1?GALILEO_E1C_SECONDARY_LENGTH:
				sim->opt.signal==GNSS_SIGNAL_BEIDOU_B1I&&strcmp(records[ri].message,"D1")==0?BEIDOU_B1I_NH_LENGTH:0U;
			if(schedule_sow>=604800U)schedule_sow=0U;
			if((!st->ready||st->record_index!=ri||st->schedule_week!=time.week||st->schedule_sow!=schedule_sow)&&
				build_store(st,&records[ri],sim->opt.signal,time.week,schedule_sow,
					iono_alpha,iono_beta)!=0){failure="navigation-message cycle construction failed";goto done;}
			st->record_index=ri;
			st->schedule_week=time.week;st->schedule_sow=schedule_sow;
			desired[n]=(gnss_rf_channel_t){1,sim->opt.signal==GNSS_SIGNAL_GALILEO_E1?GNSS_RF_GALILEO_E1:GNSS_RF_BPSK,
				profile->system,candidates[c].prn,candidates[c].carrier_hz,observations[c].doppler_hz,
				sim->opt.signal==GNSS_SIGNAL_GALILEO_E1?90.0:120.0,
				st->data_code,sim->opt.signal==GNSS_SIGNAL_GALILEO_E1?st->pilot_code:NULL,profile->code_length,
				profile->code_rate_hz*(1.0+observations[c].doppler_hz/candidates[c].carrier_hz),observations[c].code_phase_chips,
				st->symbols,st->symbol_count,sim->opt.signal==GNSS_SIGNAL_BEIDOU_B1I&&strcmp(records[ri].message,"D2")==0?500.0:
				(sim->opt.signal==GNSS_SIGNAL_GALILEO_E1?250.0:sim->opt.signal==GNSS_SIGNAL_BEIDOU_B1I?50.0:100.0),
				fmod(time.sec*(sim->opt.signal==GNSS_SIGNAL_GALILEO_E1?250.0:sim->opt.signal==GNSS_SIGNAL_BEIDOU_B1I&&strcmp(records[ri].message,"D2")==0?500.0:sim->opt.signal==GNSS_SIGNAL_BEIDOU_B1I?50.0:100.0),st->symbol_count),
				sim->opt.signal==GNSS_SIGNAL_GALILEO_E1||sim->opt.signal==GNSS_SIGNAL_BEIDOU_B1I?st->overlay:NULL,
				overlay_count,overlay_rate,overlay_count>0U?fmod(time.sec*overlay_rate,(double)overlay_count):0.0,
				observations[c].carrier_phase_rad};}
		if(gnss_rf_reconcile(active,RF_CHANNELS,desired,selected_count)!=0||
			gnss_rf_render(active,RF_CHANNELS,sim->opt.tx_frequency,sim->opt.tx_sample_rate,iq,sim->iq_block_samples)!=0){failure="RF channel reconciliation or rendering failed";goto done;}
		pthread_mutex_lock(&sim->gps.lock);if(!sim->gps.ready){sim->gps.ready=1;pthread_cond_signal(&sim->gps.initialization_done);}
		while(!is_fifo_write_ready(sim)&&!sim->finished)pthread_cond_wait(&sim->fifo_write_ready,&sim->gps.lock);
		if(sim->finished){pthread_mutex_unlock(&sim->gps.lock);break;}
		memcpy(&sim->fifo[sim->head*2],iq,sim->iq_block_samples*2U*sizeof(*iq));sim->head+=(long)sim->iq_block_samples;
		if((size_t)sim->head>=sim->fifo_length)sim->head-=(long)sim->fifo_length;
		pthread_cond_signal(&sim->fifo_read_ready);pthread_mutex_unlock(&sim->gps.lock);
		time.sec+=0.1;normalizeGpsTime(&time);
	}
	error=0;
done:
	if(error)fprintf(stderr,"ERROR: %s.\n",failure);
	free(iq);free(xyz_data);free(xyz);free(stores);free(records);finish_task(sim,error);return NULL;
}
