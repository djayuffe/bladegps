#include "gnss_time.h"

#include <math.h>
#include <stddef.h>

typedef struct {
	int year, month, day;
} leap_epoch_t;

/* UTC dates at which the new GPS-UTC value became effective.  GPS time began
 * with zero offset on 1980-01-06.  This table is intentionally explicit so
 * archived navigation files are deterministic.  A future leap second requires
 * updating this table; until then the last known offset remains in force. */
static const leap_epoch_t leap_epochs[] = {
	{1981,7,1},{1982,7,1},{1983,7,1},{1985,7,1},{1988,1,1},{1990,1,1},
	{1991,1,1},{1992,7,1},{1993,7,1},{1994,7,1},{1996,1,1},{1997,7,1},
	{1999,1,1},{2006,1,1},{2009,1,1},{2012,7,1},{2015,7,1},{2017,1,1}
};

static int days_in_month(int year, int month)
{
	static const int days[12]={31,28,31,30,31,30,31,31,30,31,30,31};
	if(month<1||month>12)return 0;
	return days[month-1]+(month==2&&year%4==0&&
		(year%100!=0||year%400==0)?1:0);
}

static int leap_insertion_date(const gnss_calendar_time_t *t)
{
	size_t index;
	for(index=0U;index<sizeof(leap_epochs)/sizeof(leap_epochs[0]);index++) {
		int year=leap_epochs[index].year,month=leap_epochs[index].month;
		int day=leap_epochs[index].day-1;
		if(day==0) {
			month--;
			if(month==0){month=12;year--;}
			day=days_in_month(year,month);
		}
		if(t->year==year&&t->month==month&&t->day==day)return 1;
	}
	return 0;
}

static int calendar_valid(const gnss_calendar_time_t *t)
{
	int days;
	if(t==NULL || t->year<1980 || t->month<1 || t->month>12 || t->day<1 ||
		t->hour<0 || t->hour>23 || t->minute<0 || t->minute>59 ||
		!isfinite(t->second) || t->second<0.0 || t->second>=61.0) return 0;
	days=days_in_month(t->year,t->month);
	if(t->day>days)return 0;
	if(t->second>=60.0 && (t->hour!=23 || t->minute!=59 ||
		!leap_insertion_date(t)))return 0;
	return 1;
}

static int compare_date(const gnss_calendar_time_t *t, const leap_epoch_t *e)
{
	if(t->year!=e->year) return t->year<e->year?-1:1;
	if(t->month!=e->month) return t->month<e->month?-1:1;
	if(t->day!=e->day) return t->day<e->day?-1:1;
	return 0;
}

int gnss_gps_utc_offset(const gnss_calendar_time_t *utc, int *offset)
{
	size_t index;
	int result=0;
	if(!calendar_valid(utc) || offset==NULL) return -1;
	for(index=0U;index<sizeof(leap_epochs)/sizeof(leap_epochs[0]);index++) {
		if(compare_date(utc,&leap_epochs[index])<0) break;
		result++;
	}
	*offset=result;
	return 0;
}

static void add_seconds(gpstime_t *time, double seconds)
{
	time->sec+=seconds;
	normalizeGpsTime(time);
}

int gnss_calendar_to_gps(gnss_system_t system,
	const gnss_calendar_time_t *calendar, gpstime_t *gps)
{
	datetime_t civil;
	int offset=0;
	if(!calendar_valid(calendar) || gps==NULL || system<0 || system>=GNSS_SYSTEM_COUNT)
		return -1;
	civil.y=calendar->year;civil.m=calendar->month;civil.d=calendar->day;
	civil.hh=calendar->hour;civil.mm=calendar->minute;civil.sec=calendar->second;
	date2gps(&civil,gps);
	if(system==GNSS_SYSTEM_BEIDOU) offset=14;
	else if(system==GNSS_SYSTEM_GLONASS && gnss_gps_utc_offset(calendar,&offset)!=0)
		return -1;
	add_seconds(gps,(double)offset);
	return 0;
}

static int gps_utc_offset_from_gps(const gpstime_t *gps)
{
	int offset=0;
	size_t index;
	for(index=0U;index<sizeof(leap_epochs)/sizeof(leap_epochs[0]);index++) {
		gnss_calendar_time_t calendar={leap_epochs[index].year,leap_epochs[index].month,
			leap_epochs[index].day,0,0,0.0};
		datetime_t civil={calendar.year,calendar.month,calendar.day,0,0,0.0};
		gpstime_t threshold;
		date2gps(&civil,&threshold);
		/* At the UTC effective epoch the new offset applies. */
		add_seconds(&threshold,(double)(index+1U));
		if(gnss_time_difference(gps,&threshold)<0.0) break;
		offset=(int)index+1;
	}
	return offset;
}

int gnss_gps_to_utc_calendar(const gpstime_t *gps,
	gnss_calendar_time_t *utc)
{
	gpstime_t normalized,linear;
	datetime_t civil;
	size_t index;
	int offset;
	if(gps==NULL||utc==NULL||gps->week<0||!isfinite(gps->sec))return -1;
	normalized=*gps;normalizeGpsTime(&normalized);
	/* A leap second occupies the GPS interval immediately before the effective
	 * UTC midnight threshold.  Linear civil time has no representation for
	 * that interval, so identify it before applying the ordinary offset. */
	for(index=0U;index<sizeof(leap_epochs)/sizeof(leap_epochs[0]);index++) {
		datetime_t effective={leap_epochs[index].year,leap_epochs[index].month,
			leap_epochs[index].day,0,0,0.0};
		gpstime_t threshold,previous;
		double delta;
		date2gps(&effective,&threshold);add_seconds(&threshold,(double)(index+1U));
		delta=gnss_time_difference(&normalized,&threshold);
		if(delta>=-1.0&&delta<0.0) {
			previous=threshold;add_seconds(&previous,-(double)(index+1U)-1.0);
			gps2date(&previous,&civil);
			*utc=(gnss_calendar_time_t){civil.y,civil.m,civil.d,23,59,60.0+delta+1.0};
			return 0;
		}
	}
	offset=gps_utc_offset_from_gps(&normalized);
	linear=normalized;add_seconds(&linear,-(double)offset);gps2date(&linear,&civil);
	*utc=(gnss_calendar_time_t){civil.y,civil.m,civil.d,civil.hh,civil.mm,civil.sec};
	return calendar_valid(utc)?0:-1;
}

int gnss_gps_to_system_time(gnss_system_t system,
	const gpstime_t *gps, gpstime_t *system_time)
{
	double correction=0.0;
	if(gps==NULL || system_time==NULL || gps->week<0 || !isfinite(gps->sec) ||
		system<0 || system>=GNSS_SYSTEM_COUNT) return -1;
	*system_time=*gps;
	if(system==GNSS_SYSTEM_BEIDOU) correction=-14.0;
	else if(system==GNSS_SYSTEM_GLONASS)
		correction=-(double)gps_utc_offset_from_gps(gps);
	add_seconds(system_time,correction);
	return 0;
}

double gnss_time_difference(const gpstime_t *newer, const gpstime_t *older)
{
	if(newer==NULL || older==NULL) return NAN;
	return ((double)newer->week-(double)older->week)*SECONDS_IN_WEEK+
		newer->sec-older->sec;
}
