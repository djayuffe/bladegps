#include "gnss_fec.h"

static unsigned int parity32(uint32_t value)
{
	value ^= value >> 16;
	value ^= value >> 8;
	value ^= value >> 4;
	value &= 0x0fU;
	return (0x6996U >> value) & 1U;
}

uint32_t gnss_crc24q_bits(const uint8_t *bits, size_t bit_count)
{
	uint32_t remainder = 0U;
	size_t bit;

	if (bits == NULL)
		return 0U;
	for (bit = 0; bit < bit_count; bit++) {
		unsigned int feedback = ((remainder >> 23) & 1U) ^ (bits[bit] & 1U);
		remainder = (remainder << 1) & 0xffffffU;
		if (feedback != 0U)
			remainder ^= 0x864cfbU;
	}
	return remainder;
}

int gnss_galileo_convolutional_encode(const uint8_t *input, size_t bit_count,
	uint8_t *output, size_t output_capacity)
{
	uint32_t state = 0U;
	size_t bit;

	if (input == NULL || output == NULL || output_capacity < bit_count * 2U)
		return -1;
	for (bit = 0; bit < bit_count; bit++) {
		if (input[bit] > 1U)
			return -1;
		state = ((state << 1) | input[bit]) & 0x7fU;
		output[bit * 2U] = (uint8_t)parity32(state & 0171U);
		/* The ICD encoder inverts the G2=133o branch. */
		output[bit * 2U + 1U] = (uint8_t)(parity32(state & 0133U) ^ 1U);
	}
	return 0;
}

int gnss_block_interleave(const uint8_t *input, size_t columns, size_t rows,
	uint8_t *output, size_t output_capacity)
{
	size_t column, row, count;

	if (input == NULL || output == NULL || columns == 0U || rows == 0U ||
		columns > SIZE_MAX / rows)
		return -1;
	count = columns * rows;
	if (output_capacity < count)
		return -1;
	/* Write down n columns, then read across k rows. */
	for (row = 0; row < rows; row++)
		for (column = 0; column < columns; column++)
			output[row * columns + column] = input[column * rows + row];
	return 0;
}

int gnss_beidou_bch15_11(const uint8_t information[11], uint8_t codeword[15])
{
	uint32_t dividend = 0U;
	uint32_t work;
	int bit;

	if (information == NULL || codeword == NULL)
		return -1;
	for (bit = 0; bit < 11; bit++) {
		if (information[bit] > 1U)
			return -1;
		dividend = (dividend << 1) | information[bit];
		codeword[bit] = information[bit];
	}
	work = dividend << 4;
	for (bit = 14; bit >= 4; bit--)
		if ((work & (1U << bit)) != 0U)
			work ^= UINT32_C(0x13) << (bit - 4);
	for (bit = 0; bit < 4; bit++)
		codeword[11 + bit] = (uint8_t)((work >> (3 - bit)) & 1U);
	return 0;
}

int gnss_beidou_interleave_2x15(const uint8_t first[15],
	const uint8_t second[15], uint8_t output[30])
{
	size_t bit;

	if (first == NULL || second == NULL || output == NULL)
		return -1;
	for (bit = 0; bit < 15U; bit++) {
		if (first[bit] > 1U || second[bit] > 1U)
			return -1;
		output[bit * 2U] = first[bit];
		output[bit * 2U + 1U] = second[bit];
	}
	return 0;
}
