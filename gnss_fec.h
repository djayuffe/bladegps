#ifndef BLADEGPS_GNSS_FEC_H
#define BLADEGPS_GNSS_FEC_H

#include <stddef.h>
#include <stdint.h>

uint32_t gnss_crc24q_bits(const uint8_t *bits, size_t bit_count);
int gnss_galileo_convolutional_encode(const uint8_t *input, size_t bit_count,
	uint8_t *output, size_t output_capacity);
int gnss_block_interleave(const uint8_t *input, size_t columns, size_t rows,
	uint8_t *output, size_t output_capacity);
int gnss_beidou_bch15_11(const uint8_t information[11], uint8_t codeword[15]);
int gnss_beidou_interleave_2x15(const uint8_t first[15],
	const uint8_t second[15], uint8_t output[30]);

#endif
