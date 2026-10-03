#ifndef BLADEGPS_GNSS_NAV_H
#define BLADEGPS_GNSS_NAV_H

#include <stddef.h>

#include "gnss.h"
#include "gpssim.h"

#define GNSS_NAV_MESSAGE_NAME_SIZE 8
#define GNSS_NAV_ORBIT_FIELDS 32

typedef struct {
	int year;
	int month;
	int day;
	int hour;
	int minute;
	double second;
} gnss_calendar_time_t;

typedef enum {
	GNSS_NAV_KEPLERIAN = 0,
	GNSS_NAV_GLONASS_STATE_VECTOR
} gnss_nav_model_t;

typedef struct {
	gnss_system_t system;
	unsigned int prn;
	char message[GNSS_NAV_MESSAGE_NAME_SIZE];
	gnss_calendar_time_t toc;
	gnss_nav_model_t model;
	double clock_bias;
	double clock_drift;
	double clock_drift_rate;
	/* RINEX broadcast-orbit fields in specification order; blank fields are NAN. */
	double orbit[GNSS_NAV_ORBIT_FIELDS];
	size_t orbit_count;
} gnss_nav_record_t;

typedef struct {
	double alpha[4];
	double beta[4];
} gnss_klobuchar_t;

int gnss_read_rinex_nav(const char *path, gnss_nav_record_t *records,
	size_t capacity, size_t *record_count);

/* Allocate and load every supported ephemeris record in a GPS RINEX 2 or
 * mixed-system RINEX 3/4 file. POSIX builds also accept .gz and legacy .Z
 * input through a shell-free gzip child process.
 * The caller owns *records and must release it with free(). */
int gnss_load_rinex_nav(const char *path, gnss_nav_record_t **records,
	size_t *record_count);

/* Return 0 with the last BeiDou Klobuchar model in the file, 1 when none is
 * present, or -1 when a matching record is malformed. */
int gnss_read_beidou_ionosphere(const char *path, gnss_klobuchar_t *model);

/* Return the signed age of a broadcast orbit at a continuous GPS epoch.
 * Keplerian constellations are referenced to toe in their native system week;
 * GLONASS state vectors are referenced to their UTC(SU) epoch. */
double gnss_nav_record_age(const gnss_nav_record_t *record,
	const gpstime_t *gps_time);

/* Maximum supported age after the reference epoch.  GPS honours the RINEX fit
 * interval when present; the other services use their broadcast refresh
 * windows.  A negative result denotes malformed timing metadata. */
double gnss_nav_record_max_age(const gnss_nav_record_t *record);

#endif
