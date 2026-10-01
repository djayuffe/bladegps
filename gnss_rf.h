#ifndef BLADEGPS_GNSS_RF_H
#define BLADEGPS_GNSS_RF_H

#include <stddef.h>
#include <stdint.h>

#include "gnss.h"

typedef enum {
	GNSS_RF_BPSK = 0,
	GNSS_RF_GALILEO_E1
} gnss_rf_modulation_t;

typedef struct {
	int enabled;
	gnss_rf_modulation_t modulation;
	gnss_system_t system;
	unsigned int prn;
	double carrier_hz, doppler_hz, amplitude;
	const int8_t *data_code, *pilot_code;
	size_t code_length;
	double code_rate_hz, code_phase;
	const int8_t *data_symbols;
	size_t data_symbol_count;
	double data_rate_hz, data_phase;
	const int8_t *overlay_symbols;
	size_t overlay_symbol_count;
	double overlay_rate_hz, overlay_phase;
	double carrier_phase;
} gnss_rf_channel_t;

typedef struct {
	gnss_system_t system;
	unsigned int prn;
	double carrier_hz, occupied_bandwidth_hz, elevation_rad;
	int healthy;
} gnss_rf_candidate_t;

#define GLONASS_L1OF_STRING_SYMBOLS 200U

/* Convert unpacked binary navigation bits (0/1, transmission order) to NRZ
 * signal levels (+1/-1). */
int gnss_rf_bits_to_symbols(const uint8_t *bits, size_t bit_count,
	int8_t *symbols);

/* Form one complete two-second GLONASS L1OF navigation sequence. The 85-bit
 * Hamming-protected string is differentially encoded, combined with the
 * 100 Hz auxiliary meander, and followed by the 30-chip time mark. The
 * differential state is supplied and returned as a binary value so adjacent
 * strings can be generated continuously. */
int gnss_glonass_l1of_symbols(const uint8_t string[85],
	uint8_t *previous_relative_bit,
	int8_t symbols[GLONASS_L1OF_STRING_SYMBOLS]);

int gnss_rf_validate_channel(const gnss_rf_channel_t *channel,
	double center_hz, double sample_rate_hz);
int gnss_rf_render(gnss_rf_channel_t *channels, size_t channel_count,
	double center_hz, double sample_rate_hz, int16_t *interleaved_iq,
	size_t sample_count);
int gnss_rf_allocate(const gnss_rf_candidate_t *candidates, size_t candidate_count,
	double center_hz, double sample_rate_hz, double elevation_mask_rad,
	size_t *selected_indices, size_t selected_capacity, size_t *selected_count);

#endif
