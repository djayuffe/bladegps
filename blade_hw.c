#include "blade_hw.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

static double range_minimum(const struct bladerf_range *range)
{
	return range != NULL ? (double)range->min * (double)range->scale : 0.0;
}

static double range_maximum(const struct bladerf_range *range)
{
	return range != NULL ? (double)range->max * (double)range->scale : 0.0;
}

int blade_hw_range_contains(const struct bladerf_range *range, double value)
{
	return range != NULL && isfinite(value) && value >= range_minimum(range) &&
		value <= range_maximum(range);
}

double blade_hw_required_bandwidth(double center_hz, double carrier_hz,
	double occupied_bandwidth_hz)
{
	if (!isfinite(center_hz) || !isfinite(carrier_hz) ||
		!isfinite(occupied_bandwidth_hz) || occupied_bandwidth_hz <= 0.0)
		return NAN;
	return occupied_bandwidth_hz + 2.0 * fabs(carrier_hz - center_hz);
}

int blade_hw_validate_rf_plan(double center_hz, double carrier_hz,
	double occupied_bandwidth_hz, double sample_rate_hz, double bandwidth_hz)
{
	double required = blade_hw_required_bandwidth(center_hz, carrier_hz,
		occupied_bandwidth_hz);

	if (!isfinite(required) || !isfinite(sample_rate_hz) ||
		!isfinite(bandwidth_hz) || sample_rate_hz <= 0.0 || bandwidth_hz <= 0.0)
		return -1;
	if (required > sample_rate_hz || required > bandwidth_hz)
		return -1;
	/* libbladeRF recommends keeping the analog bandwidth no wider than the
	 * sample rate so that out-of-band energy cannot alias into the stream. */
	if (bandwidth_hz > sample_rate_hz)
		return -1;
	return 0;
}

int blade_hw_validate_stream_geometry(unsigned int num_buffers,
	unsigned int buffer_size, unsigned int num_transfers)
{
	if (num_buffers == 0U || buffer_size == 0U || num_transfers == 0U)
		return -1;
	if ((buffer_size % 1024U) != 0U || num_buffers <= num_transfers)
		return -1;
	return 0;
}

static int report_api_error(const char *operation, int status)
{
	fprintf(stderr, "ERROR: %s: %s\n", operation, bladerf_strerror(status));
	return status != 0 ? status : BLADERF_ERR_UNEXPECTED;
}

static int require_range(struct bladerf *dev, const char *label,
	int (*getter)(struct bladerf *, bladerf_channel, const struct bladerf_range **),
	double requested)
{
	const struct bladerf_range *range = NULL;
	int status = getter(dev, BLADERF_CHANNEL_TX(0), &range);

	if (status != 0)
		return report_api_error(label, status);
	if (!blade_hw_range_contains(range, requested)) {
		fprintf(stderr, "ERROR: %s %.0f is outside this device's range %.0f..%.0f.\n",
			label, requested, range_minimum(range), range_maximum(range));
		return BLADERF_ERR_RANGE;
	}
	return 0;
}

static int configure_xb(struct bladerf *dev, int xb_board)
{
	int status;

	if (xb_board == 0)
		return 0;
	if (xb_board != 200) {
		fprintf(stderr, "ERROR: Unsupported expansion board XB%d for GNSS TX.\n", xb_board);
		return BLADERF_ERR_UNSUPPORTED;
	}

	status = bladerf_expansion_attach(dev, BLADERF_XB_200);
	if (status != 0)
		return report_api_error("attach XB200", status);
	/* GNSS L-band is inside the native bladeRF tuning range. Keep the mixer
	 * bypassed and let the board select the least-loss TX filter. RX/ADC state
	 * is intentionally untouched because bladeGPS is a transmit-only tool. */
	status = bladerf_xb200_set_path(dev, BLADERF_CHANNEL_TX(0), BLADERF_XB200_BYPASS);
	if (status != 0)
		return report_api_error("set XB200 TX bypass path", status);
	status = bladerf_xb200_set_filterbank(dev, BLADERF_CHANNEL_TX(0),
		BLADERF_XB200_AUTO_1DB);
	if (status != 0)
		return report_api_error("set XB200 automatic TX filter", status);
	printf("XB200 TX: native L-band bypass, automatic 1 dB filter selection\n");
	return 0;
}

static int set_legacy_stage(struct bladerf *dev, const char *name, int gain)
{
	const struct bladerf_range *range = NULL;
	int actual = 0;
	int status = bladerf_get_gain_stage_range(dev, BLADERF_CHANNEL_TX(0), name, &range);

	if (status != 0) {
		fprintf(stderr, "ERROR: Gain stage '%s' is unavailable on this device; use -G.\n", name);
		return status;
	}
	if (!blade_hw_range_contains(range, (double)gain)) {
		fprintf(stderr, "ERROR: %s gain %d dB is outside %.0f..%.0f dB.\n",
			name, gain, range_minimum(range), range_maximum(range));
		return BLADERF_ERR_RANGE;
	}
	status = bladerf_set_gain_stage(dev, BLADERF_CHANNEL_TX(0), name, gain);
	if (status != 0)
		return report_api_error("set legacy TX gain stage", status);
	status = bladerf_get_gain_stage(dev, BLADERF_CHANNEL_TX(0), name, &actual);
	if (status != 0)
		return report_api_error("read legacy TX gain stage", status);
	printf("TX %s gain: %d dB\n", name, actual);
	return 0;
}

int blade_hw_configure_tx(struct bladerf *dev, const blade_hw_config_t *config,
	blade_hw_result_t *result)
{
	const char *board;
	uint64_t actual_frequency = 0;
	unsigned int actual_sample_rate = 0;
	unsigned int actual_bandwidth = 0;
	int actual_gain = 0;
	int fpga_status;
	int status;

	if (dev == NULL || config == NULL || result == NULL)
		return BLADERF_ERR_INVAL;
	memset(result, 0, sizeof(*result));
	board = bladerf_get_board_name(dev);
	if (board == NULL)
		board = "unknown";
	(void)snprintf(result->board_name, sizeof(result->board_name), "%s", board);
	fpga_status = bladerf_is_fpga_configured(dev);
	if (fpga_status < 0)
		return report_api_error("query FPGA state", fpga_status);
	if (fpga_status == 0) {
		fprintf(stderr, "ERROR: bladeRF FPGA is not configured.\n");
		return BLADERF_ERR_NOT_INIT;
	}
	printf("bladeRF board: %s (FPGA configured)\n", result->board_name);

	status = configure_xb(dev, config->xb_board);
	if (status != 0)
		return status;
	status = require_range(dev, "TX frequency", bladerf_get_frequency_range,
		(double)config->frequency_hz);
	if (status != 0)
		return status;
	status = bladerf_set_frequency(dev, BLADERF_CHANNEL_TX(0), config->frequency_hz);
	if (status != 0)
		return report_api_error("set TX frequency", status);
	status = bladerf_get_frequency(dev, BLADERF_CHANNEL_TX(0), &actual_frequency);
	if (status != 0)
		return report_api_error("read TX frequency", status);
	if (actual_frequency != config->frequency_hz) {
		fprintf(stderr, "ERROR: Exact TX frequency unavailable (requested %llu, actual %llu Hz).\n",
			(unsigned long long)config->frequency_hz,
			(unsigned long long)actual_frequency);
		return BLADERF_ERR_RANGE;
	}

	status = require_range(dev, "TX sample rate", bladerf_get_sample_rate_range,
		(double)config->sample_rate_hz);
	if (status != 0)
		return status;
	status = bladerf_set_sample_rate(dev, BLADERF_CHANNEL_TX(0),
		config->sample_rate_hz, &actual_sample_rate);
	if (status != 0)
		return report_api_error("set TX sample rate", status);
	if (actual_sample_rate != config->sample_rate_hz) {
		fprintf(stderr, "ERROR: Exact TX sample rate unavailable (requested %u, actual %u sps).\n",
			config->sample_rate_hz, actual_sample_rate);
		return BLADERF_ERR_RANGE;
	}

	status = require_range(dev, "TX bandwidth", bladerf_get_bandwidth_range,
		(double)config->bandwidth_hz);
	if (status != 0)
		return status;
	status = bladerf_set_bandwidth(dev, BLADERF_CHANNEL_TX(0),
		config->bandwidth_hz, &actual_bandwidth);
	if (status != 0)
		return report_api_error("set TX bandwidth", status);
	if (blade_hw_validate_rf_plan((double)actual_frequency,
		config->signal_carrier_hz, config->occupied_bandwidth_hz,
		(double)actual_sample_rate, (double)actual_bandwidth) != 0) {
		fprintf(stderr, "ERROR: Hardware-selected bandwidth %u Hz does not safely contain the signal.\n",
			actual_bandwidth);
		return BLADERF_ERR_RANGE;
	}

	if (config->use_legacy_gain) {
		status = set_legacy_stage(dev, "txvga1", config->txvga1_db);
		if (status == 0)
			status = set_legacy_stage(dev, "txvga2", config->txvga2_db);
		if (status != 0)
			return status;
		status = bladerf_get_gain(dev, BLADERF_CHANNEL_TX(0), &actual_gain);
	} else {
		status = require_range(dev, "TX gain", bladerf_get_gain_range,
			(double)config->gain_db);
		if (status != 0)
			return status;
		status = bladerf_set_gain(dev, BLADERF_CHANNEL_TX(0), config->gain_db);
		if (status == 0)
			status = bladerf_get_gain(dev, BLADERF_CHANNEL_TX(0), &actual_gain);
	}
	if (status != 0)
		return report_api_error("configure/read TX gain", status);

	result->frequency_hz = actual_frequency;
	result->sample_rate_hz = actual_sample_rate;
	result->bandwidth_hz = actual_bandwidth;
	result->gain_db = actual_gain;
	printf("TX frequency: %llu Hz\n", (unsigned long long)actual_frequency);
	printf("TX sample rate: %u sps\n", actual_sample_rate);
	printf("TX analog bandwidth: %u Hz (requested %u Hz)\n",
		actual_bandwidth, config->bandwidth_hz);
	printf("TX overall gain: %d dB (relative, not calibrated output power)\n", actual_gain);
	return 0;
}

void blade_hw_measure_samples(const int16_t *iq, size_t complex_samples,
	blade_sample_stats_t *stats)
{
	size_t i;

	if (iq == NULL || stats == NULL)
		return;
	for (i = 0; i < complex_samples * 2U; i++) {
		int value = iq[i];
		unsigned int magnitude = (unsigned int)(value < 0 ? -value : value);
		if (magnitude > stats->peak_abs)
			stats->peak_abs = magnitude;
		if (value <= -2048 || value >= 2047)
			stats->rail_components++;
	}
	stats->complex_samples += complex_samples;
}
