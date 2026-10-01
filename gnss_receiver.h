#ifndef BLADEGPS_GNSS_RECEIVER_H
#define BLADEGPS_GNSS_RECEIVER_H

#include <stddef.h>
#include <stdint.h>

typedef struct {
	double carrier_offset_hz;
	unsigned int code_phase_chips;
	double normalized_correlation;
} gnss_rx_acquisition_t;

/* Independent coherent acquisition check for a BPSK ranging code.  The
 * receiver searches every integer code phase and a caller-selected carrier
 * grid.  It intentionally does not share renderer phase state, making it a
 * useful generated-I/Q loopback and regression validator. */
int gnss_rx_acquire_bpsk(const int16_t *interleaved_iq, size_t sample_count,
	double sample_rate_hz, const int8_t *code, size_t code_length,
	double code_rate_hz, double minimum_carrier_hz, double maximum_carrier_hz,
	double carrier_step_hz, gnss_rx_acquisition_t *result);

#endif
