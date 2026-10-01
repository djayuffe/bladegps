#ifndef BLADEGPS_GNSS_TIME_H
#define BLADEGPS_GNSS_TIME_H

#include "gnss_nav.h"
#include "gpssim.h"

/* Convert a RINEX calendar epoch expressed in the constellation's native
 * time scale to continuous GPS week/seconds.  GPS and Galileo calendar
 * fields are GPS-aligned, BDT is fourteen seconds behind GPS, and GLONASS
 * navigation epochs are UTC and therefore use the historical GPS-UTC table. */
int gnss_calendar_to_gps(gnss_system_t system,
	const gnss_calendar_time_t *calendar, gpstime_t *gps);

/* Convert continuous GPS time to the constellation-native continuous week
 * and seconds-of-week used for orbit propagation and navigation scheduling.
 * The week member remains a continuous Sunday-based week counter; callers
 * that encode an on-air truncated week apply the signal-specific epoch. */
int gnss_gps_to_system_time(gnss_system_t system,
	const gpstime_t *gps, gpstime_t *system_time);

/* GPS-UTC at a UTC calendar instant.  Leap second 23:59:60 is represented
 * using the offset that was in force during that inserted second. */
int gnss_gps_utc_offset(const gnss_calendar_time_t *utc, int *offset_seconds);

double gnss_time_difference(const gpstime_t *newer, const gpstime_t *older);

#endif
