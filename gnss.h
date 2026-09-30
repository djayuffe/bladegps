#ifndef BLADEGPS_GNSS_H
#define BLADEGPS_GNSS_H

#include <stddef.h>
#include <stdint.h>

typedef enum {
	GNSS_SYSTEM_GPS = 0,
	GNSS_SYSTEM_GALILEO,
	GNSS_SYSTEM_BEIDOU,
	GNSS_SYSTEM_GLONASS,
	GNSS_SYSTEM_COUNT
} gnss_system_t;

typedef enum {
	GNSS_SIGNAL_GPS_L1CA = 0,
	GNSS_SIGNAL_GALILEO_E1,
	GNSS_SIGNAL_BEIDOU_B1I,
	GNSS_SIGNAL_GLONASS_L1OF,
	GNSS_SIGNAL_COUNT
} gnss_signal_t;

typedef struct {
	gnss_signal_t id;
	gnss_system_t system;
	const char *name;
	const char *description;
	double carrier_hz;
	double code_rate_hz;
	uint32_t code_length;
	double minimum_sample_rate_hz;
	double recommended_bandwidth_hz;
	uint32_t maximum_sv;
	int fdma;
	int waveform_implemented;
} gnss_signal_profile_t;

const gnss_signal_profile_t *gnss_signal_profile(gnss_signal_t signal);
const gnss_signal_profile_t *gnss_signal_profile_by_name(const char *name);
const char *gnss_system_name(gnss_system_t system);
int gnss_signal_parse(const char *name, gnss_signal_t *signal);
int gnss_frequency_fits(double center_hz, double sample_rate_hz,
	double carrier_hz, double occupied_bandwidth_hz);
void gnss_print_signal_profiles(void);

#endif
