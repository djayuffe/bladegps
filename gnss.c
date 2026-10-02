#include "gnss.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

static const gnss_signal_profile_t signal_profiles[GNSS_SIGNAL_COUNT] = {
	{
		GNSS_SIGNAL_GPS_L1CA, GNSS_SYSTEM_GPS, "gps-l1ca",
		"GPS L1 C/A with LNAV", 1575.42e6, 1.023e6, 1023,
		2.6e6, 2.5e6, 37, 0, 1
	},
	{
		GNSS_SIGNAL_GALILEO_E1, GNSS_SYSTEM_GALILEO, "galileo-e1",
		"Galileo E1-B/C Open Service with I/NAV", 1575.42e6, 1.023e6, 4092,
		GNSS_GALILEO_E1_SAMPLE_RATE_HZ,
		GNSS_GALILEO_E1_REFERENCE_BANDWIDTH_HZ, 36, 0, 1
	},
	{
		GNSS_SIGNAL_BEIDOU_B1I, GNSS_SYSTEM_BEIDOU, "beidou-b1i",
		"BeiDou B1I Open Service with D1/D2 NAV", 1561.098e6, 2.046e6, 2046,
		5.0e6, 4.5e6, 63, 0, 1
	},
	{
		GNSS_SIGNAL_GLONASS_L1OF, GNSS_SYSTEM_GLONASS, "glonass-l1of",
		"GLONASS L1 open FDMA service with GNAV", 1602.0e6, 0.511e6, 511,
		12.0e6, 10.0e6, 24, 1, 1
	},
	{
		GNSS_SIGNAL_MIXED_OPEN, GNSS_SYSTEM_GPS, "mixed-open",
		"Concurrent GPS L1 C/A, Galileo E1, BeiDou B1I and GLONASS L1OF",
		1582.3925e6, 1.023e6, 1023,
		48.0e6, 47.1e6, 63, 1, 1
	}
};

const gnss_signal_profile_t *gnss_signal_profile(gnss_signal_t signal)
{
	if (signal < 0 || signal >= GNSS_SIGNAL_COUNT)
		return NULL;
	return &signal_profiles[signal];
}

const gnss_signal_profile_t *gnss_signal_profile_by_name(const char *name)
{
	int signal;

	if (name == NULL)
		return NULL;
	for (signal = 0; signal < GNSS_SIGNAL_COUNT; signal++)
		if (strcmp(name, signal_profiles[signal].name) == 0)
			return &signal_profiles[signal];
	return NULL;
}

const char *gnss_system_name(gnss_system_t system)
{
	static const char *names[GNSS_SYSTEM_COUNT] = {
		"GPS", "Galileo", "BeiDou", "GLONASS"
	};

	if (system < 0 || system >= GNSS_SYSTEM_COUNT)
		return "Unknown";
	return names[system];
}

int gnss_signal_parse(const char *name, gnss_signal_t *signal)
{
	const gnss_signal_profile_t *profile = gnss_signal_profile_by_name(name);

	if (profile == NULL || signal == NULL)
		return -1;
	*signal = profile->id;
	return 0;
}

int gnss_frequency_fits(double center_hz, double sample_rate_hz,
	double carrier_hz, double occupied_bandwidth_hz)
{
	double available_half_band;
	double required_half_band;

	if (!isfinite(center_hz) || !isfinite(sample_rate_hz) ||
		!isfinite(carrier_hz) || !isfinite(occupied_bandwidth_hz) ||
		center_hz <= 0.0 || sample_rate_hz <= 0.0 || occupied_bandwidth_hz <= 0.0)
		return 0;

	available_half_band = sample_rate_hz / 2.0;
	required_half_band = fabs(carrier_hz - center_hz) + occupied_bandwidth_hz / 2.0;
	return required_half_band <= available_half_band;
}

void gnss_print_signal_profiles(void)
{
	int signal;

	puts("Signal profiles:");
	for (signal = 0; signal < GNSS_SIGNAL_COUNT; signal++) {
		const gnss_signal_profile_t *profile = &signal_profiles[signal];
		printf("  %-14s %-8s %10.3f MHz  %s\n", profile->name,
			profile->id==GNSS_SIGNAL_MIXED_OPEN?"Mixed":gnss_system_name(profile->system),
			profile->carrier_hz/1.0e6,
			profile->waveform_implemented ? "implemented" : "planned");
	}
}
