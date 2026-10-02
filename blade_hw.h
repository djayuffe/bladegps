#ifndef BLADEGPS_BLADE_HW_H
#define BLADEGPS_BLADE_HW_H

#include <stddef.h>
#include <stdint.h>
#include <libbladeRF.h>

typedef struct {
	uint64_t frequency_hz;
	unsigned int sample_rate_hz;
	unsigned int bandwidth_hz;
	double signal_carrier_hz;
	double occupied_bandwidth_hz;
	int gain_db;
	int use_legacy_gain;
	int txvga1_db;
	int txvga2_db;
	int xb_board;
} blade_hw_config_t;

typedef struct {
	uint64_t frequency_hz;
	unsigned int sample_rate_hz;
	unsigned int bandwidth_hz;
	int gain_db;
	size_t tx_channel_count;
	bladerf_dev_speed device_speed;
	char board_name[32];
} blade_hw_result_t;

typedef struct {
	uint64_t complex_samples;
	uint64_t rail_components;
	unsigned int peak_abs;
} blade_sample_stats_t;

int blade_hw_range_contains(const struct bladerf_range *range, double value);
double blade_hw_required_bandwidth(double center_hz, double carrier_hz,
	double occupied_bandwidth_hz);
int blade_hw_validate_rf_plan(double center_hz, double carrier_hz,
	double occupied_bandwidth_hz, double sample_rate_hz, double bandwidth_hz);
int blade_hw_validate_stream_geometry(unsigned int num_buffers,
	unsigned int buffer_size, unsigned int num_transfers);
int blade_hw_validate_transport(bladerf_dev_speed speed,
	unsigned int sample_rate_hz);
int blade_hw_configure_tx(struct bladerf *dev, const blade_hw_config_t *config,
	blade_hw_result_t *result);
void blade_hw_measure_samples(const int16_t *iq, size_t complex_samples,
	blade_sample_stats_t *stats);

#endif
