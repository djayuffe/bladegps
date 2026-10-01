#include "gnss_schedule.h"

#include <math.h>
#include <string.h>

#include "gnss_beidou_nav.h"
#include "gnss_galileo_nav.h"
#include "gnss_glonass_nav.h"
#include "gnss_rf.h"

static int leap_year(int year)
{
	return year%4==0 && (year%100!=0 || year%400==0);
}

static int glonass_calendar_fields(const gnss_calendar_time_t *time,
	uint16_t *day, uint8_t *cycle)
{
	static const unsigned int before_month[12]={0U,31U,59U,90U,120U,151U,
		181U,212U,243U,273U,304U,334U};
	int start,year,total;
	if(time==NULL||day==NULL||cycle==NULL||time->year<1996||
		time->month<1||time->month>12||time->day<1||time->day>31) return -1;
	start=time->year;
	while(!leap_year(start))start--;
	total=(int)before_month[time->month-1]+time->day+
		(time->month>2&&leap_year(time->year));
	for(year=start;year<time->year;year++)total+=leap_year(year)?366:365;
	if(total<1||total>1461||start<1996||(start-1996)%4!=0||
		(start-1996)/4+1>31) return -1;
	*day=(uint16_t)total;*cycle=(uint8_t)((start-1996)/4+1);return 0;
}

static void put_uint(uint8_t *bits, unsigned int value, unsigned int width)
{
	unsigned int index;
	for(index=0U;index<width;index++)
		bits[index]=(uint8_t)((value>>(width-index-1U))&1U);
}

static int galileo_word5_from_rinex(const gnss_nav_record_t *record,
	galileo_inav_word5_t *fields)
{
	long health,bgd_a,bgd_b;
	if(record==NULL||fields==NULL||record->system!=GNSS_SYSTEM_GALILEO||
		record->orbit_count<24U||!isfinite(record->orbit[20])||
		!isfinite(record->orbit[21])||!isfinite(record->orbit[22]))return -1;
	health=lround(record->orbit[20]);
	bgd_a=lround(ldexp(record->orbit[21],32));
	bgd_b=lround(ldexp(record->orbit[22],32));
	if(fabs(record->orbit[20]-(double)health)>1.0e-6||health<0||health>511||
		bgd_a<-512||bgd_a>511||bgd_b<-512||bgd_b>511)return -1;
	fields->bgd_e1e5a=(int16_t)bgd_a;fields->bgd_e1e5b=(int16_t)bgd_b;
	fields->e1b_dvs=(uint8_t)(health&1L);
	fields->e1b_health=(uint8_t)((health>>1)&3L);
	fields->e5b_dvs=(uint8_t)((health>>6)&1L);
	fields->e5b_health=(uint8_t)((health>>7)&3L);
	return 0;
}

int gnss_schedule_galileo_e1(const gnss_nav_record_t *record,
	unsigned int week, uint32_t tow, int8_t symbols[GALILEO_E1_CYCLE_SYMBOLS])
{
	uint8_t words[4][GALILEO_INAV_WORD_BITS],word[GALILEO_INAV_WORD_BITS];
	uint8_t even[GALILEO_INAV_PAGE_PART_SYMBOLS],odd[GALILEO_INAV_PAGE_PART_SYMBOLS];
	uint8_t osnma[GALILEO_INAV_OSNMA_BITS]={0},sar[GALILEO_INAV_SAR_BITS]={0};
	unsigned int second;
	if(record==NULL || symbols==NULL || week>4095U || tow>=604800U ||
		gnss_galileo_inav_ephemeris_words(record,words)!=0) return -1;
	for(second=0U;second<30U;second+=2U) {
		int type=gnss_galileo_inav_e1b_word_type(second);
		size_t offset=(size_t)second*250U;
		memset(word,0,sizeof(word));
		if(type>=1 && type<=4) memcpy(word,words[type-1],sizeof(word));
		else if(type==5) {
			galileo_inav_word5_t fields={0};
			fields.week=(uint16_t)week; fields.tow=(tow+second)%604800U;
			if(galileo_word5_from_rinex(record,&fields)!=0||
				gnss_galileo_inav_word5(&fields,word)!=0) return -1;
		} else {
			/* Optional service/almanac/FEC2 content is not present in an EPH
			 * record. ICD dummy word type 63 is the safe on-air substitute;
			 * labeling a zero body as the scheduled type creates false data. */
			put_uint(word,type==0?0U:63U,6U);
		}
		if(gnss_galileo_inav_e1b_page(word,osnma,sar,0U,
			gnss_galileo_inav_ssp_for_second(second%6U),even,odd,NULL)!=0 ||
			gnss_rf_bits_to_symbols(even,sizeof(even),symbols+offset)!=0 ||
			gnss_rf_bits_to_symbols(odd,sizeof(odd),symbols+offset+sizeof(even))!=0)
			return -1;
	}
	return 0;
}

int gnss_schedule_beidou_d1(const gnss_nav_record_t *record,
	const int8_t alpha[4], const int8_t beta[4], uint32_t sow,
	int8_t symbols[BEIDOU_D1_FRAME_SYMBOLS])
{
	beidou_d1_clock_t clock; beidou_d1_ephemeris_t ephemeris;
	uint8_t frames[5][BEIDOU_NAV_SUBFRAME_BITS],payload[BEIDOU_NAV_PAYLOAD_BITS]={0};
	unsigned int frame;
	if(record==NULL || symbols==NULL || strcmp(record->message,"D1")!=0 || sow>=604800U ||
		gnss_beidou_d1_clock_from_rinex(record,alpha,beta,&clock)!=0 ||
		gnss_beidou_d1_ephemeris_from_rinex(record,&ephemeris)!=0 ||
		gnss_beidou_d1_clock_subframe(&clock,sow,frames[0])!=0 ||
		gnss_beidou_d1_ephemeris_subframes(&ephemeris,sow,frames[1],frames[2])!=0 ||
		gnss_beidou_nav_build_subframe(4U,(sow+18U)%604800U,payload,frames[3])!=0 ||
		gnss_beidou_nav_build_subframe(5U,(sow+24U)%604800U,payload,frames[4])!=0) return -1;
	for(frame=0U;frame<5U;frame++)
		if(gnss_rf_bits_to_symbols(frames[frame],BEIDOU_NAV_SUBFRAME_BITS,
			symbols+frame*BEIDOU_NAV_SUBFRAME_BITS)!=0) return -1;
	return 0;
}

int gnss_schedule_beidou_d2(const gnss_nav_record_t *record,
	const int8_t alpha[4], const int8_t beta[4], uint32_t sow,
	int8_t symbols[BEIDOU_D2_CYCLE_SYMBOLS])
{
	beidou_d1_clock_t clock; beidou_d1_ephemeris_t ephemeris;
	uint8_t basic[10][BEIDOU_NAV_SUBFRAME_BITS],subframe[BEIDOU_NAV_SUBFRAME_BITS];
	uint8_t payload[BEIDOU_NAV_PAYLOAD_BITS]={0};
	unsigned int page,fraid;
	if(record==NULL || symbols==NULL || strcmp(record->message,"D2")!=0 || sow>=604800U ||
		gnss_beidou_d1_clock_from_rinex(record,alpha,beta,&clock)!=0 ||
		gnss_beidou_d1_ephemeris_from_rinex(record,&ephemeris)!=0 ||
		gnss_beidou_d2_basic_pages(&clock,&ephemeris,sow,basic)!=0) return -1;
	for(page=0U;page<10U;page++) {
		size_t offset=(size_t)page*1500U;
		uint32_t page_sow=(sow+page*3U)%604800U;
		if(gnss_rf_bits_to_symbols(basic[page],300U,symbols+offset)!=0) return -1;
		for(fraid=2U;fraid<=5U;fraid++) {
			if(gnss_beidou_nav_build_subframe(fraid,page_sow,payload,subframe)!=0 ||
				gnss_rf_bits_to_symbols(subframe,300U,
					symbols+offset+(fraid-1U)*300U)!=0) return -1;
		}
	}
	return 0;
}

int gnss_schedule_glonass(const gnss_nav_record_t *record,
	int8_t symbols[GLONASS_GNAV_FRAME_SYMBOLS])
{
	glonass_gnav_immediate_t immediate;
	glonass_gnav_string5_t time_data={0};
	glonass_gnav_almanac_t almanacs[5];
	uint8_t frame[15][85],previous=0U;
	unsigned int index;
	if(record==NULL || symbols==NULL ||
		gnss_glonass_gnav_from_rinex(record,&immediate)!=0) return -1;
	if(glonass_calendar_fields(&record->toc,&time_data.na,&time_data.n4)!=0)
		return -1;
	memset(almanacs,0,sizeof(almanacs));
	for(index=0U;index<5U;index++) {
		almanacs[index].slot=(uint8_t)(index+1U);
		/* An FDMA ephemeris record does not contain constellation almanacs.
		 * Cn=0 explicitly marks these structurally required filler pairs as
		 * non-operational instead of advertising fabricated usable orbits. */
		almanacs[index].healthy=0U;
	}
	if(gnss_glonass_gnav_frame(&immediate,&time_data,almanacs,frame)!=0) return -1;
	for(index=0U;index<15U;index++)
		if(gnss_glonass_l1of_symbols(frame[index],&previous,
			symbols+index*GLONASS_L1OF_STRING_SYMBOLS)!=0) return -1;
	return 0;
}
