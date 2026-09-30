#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "bladegps.h"

/* gpssim.c's real-time task references this producer-side helper. Unit tests
 * never enter gps_task(), so a local stub keeps the model tests independent
 * from the application entry point in bladegps.c. */
int is_fifo_write_ready(sim_t *sim)
{
	(void)sim;
	return 0;
}

int stop_was_requested(void)
{
	return 0;
}

static void test_time_conversions(void)
{
	datetime_t epoch = {1980, 1, 6, 0, 0, 0.0};
	datetime_t before_century = {2100, 2, 28, 0, 0, 0.0};
	datetime_t after_century = {2100, 3, 1, 0, 0, 0.0};
	gpstime_t g_epoch;
	gpstime_t g_before;
	gpstime_t g_after;
	gpstime_t rollover = {2200, SECONDS_IN_WEEK + 1.25};

	date2gps(&epoch, &g_epoch);
	assert(g_epoch.week == 0);
	assert(g_epoch.sec == 0.0);

	date2gps(&before_century, &g_before);
	date2gps(&after_century, &g_after);
	assert(subGpsTime(g_after, g_before) == SECONDS_IN_DAY);

	normalizeGpsTime(&rollover);
	assert(rollover.week == 2201);
	assert(fabs(rollover.sec - 1.25) < 1.0e-12);
}

static void test_coordinate_round_trip(void)
{
	double llh[3] = {59.3293/R2D, 18.0686/R2D, 30.0};
	double xyz[3];
	double result[3];

	llh2xyz(llh, xyz);
	xyz2llh(xyz, result);
	assert(fabs(result[0] - llh[0]) < 1.0e-10);
	assert(fabs(result[1] - llh[1]) < 1.0e-10);
	assert(fabs(result[2] - llh[2]) < 1.0e-4);
}

static void test_ca_code_balance(void)
{
	int ca[CA_SEQ_LEN];
	int ones = 0;
	int i;

	memset(ca, 0, sizeof(ca));
	codegen(ca, 1);
	for (i = 0; i < CA_SEQ_LEN; i++) {
		assert(ca[i] == 0 || ca[i] == 1);
		ones += ca[i];
	}
	assert(ones == 512);
}

static void test_ephemeris_selection(void)
{
	ephem_t source[EPHEM_ARRAY_SIZE][MAX_SAT];
	ephem_t selected[MAX_SAT];
	gpstime_t now = {2200, 100100.0};

	memset(source, 0, sizeof(source));
	source[0][0].vflg = 1;
	source[0][0].toc.week = 2200;
	source[0][0].toc.sec = 98000.0;
	source[0][0].toe = source[0][0].toc;
	source[0][0].fit_interval = DEFAULT_EPHEMERIS_FIT_HOURS;
	source[0][0].iode = 1;
	source[1][0].vflg = 1;
	source[1][0].toc.week = 2200;
	source[1][0].toc.sec = 100000.0;
	source[1][0].toe = source[1][0].toc;
	source[1][0].fit_interval = DEFAULT_EPHEMERIS_FIT_HOURS;
	source[1][0].iode = 2;

	assert(selectEphemerides(selected, source, 2, now) == 1);
	assert(selected[0].iode == 2);

	now.sec += DEFAULT_EPHEMERIS_FIT_HOURS * SECONDS_IN_HOUR / 2.0 + 1000.0;
	assert(selectEphemerides(selected, source, 2, now) == 0);
}

#ifndef _WIN32
static void test_compressed_rinex_sample(void)
{
	ephem_t ephemerides[EPHEM_ARRAY_SIZE][MAX_SAT];
	int count;
	int valid = 0;
	int set;
	int sv;

	count = readRinexNavAll(ephemerides, "brdc2940.18n.Z");
	assert(count > 0);
	for (set = 0; set < count; set++)
		for (sv = 0; sv < MAX_SAT; sv++)
			valid += ephemerides[set][sv].vflg == 1;
	assert(valid > 0);
}
#endif

int main(void)
{
	test_time_conversions();
	test_coordinate_round_trip();
	test_ca_code_balance();
	test_ephemeris_selection();
#ifndef _WIN32
	test_compressed_rinex_sample();
#endif
	puts("core model tests passed");
	return 0;
}
