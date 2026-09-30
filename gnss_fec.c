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
		/* Figure 13 numbers the newest input bit at the MSB end of the
		 * seven-stage register.  Keeping that orientation is significant:
		 * reversing it produces a valid-looking code, but not the Galileo
		 * code specified by G1=171o and G2=133o. */
		state = (state >> 1) | ((uint32_t)input[bit] << 6);
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

static int glonass_check_member(unsigned int check, unsigned int bit)
{
	static const uint8_t c1[] = {9,10,12,13,15,17,19,20,22,24,26,28,30,
		32,34,35,37,39,41,43,45,47,49,51,53,55,57,59,61,63,65,66,68,
		70,72,74,76,78,80,82,84};
	static const uint8_t c2[] = {9,11,12,14,15,18,19,21,22,25,26,29,30,
		33,34,36,37,40,41,44,45,48,49,52,53,56,57,60,61,64,65,67,68,
		71,72,75,76,79,80,83,84};
	size_t index;
	if (check == 1U || check == 2U) {
		const uint8_t *members = check == 1U ? c1 : c2;
		size_t count = check == 1U ? sizeof(c1) : sizeof(c2);
		for (index = 0; index < count; index++)
			if (members[index] == bit) return 1;
		return 0;
	}
	if (check == 3U)
		return (bit >= 10U && bit <= 12U) || (bit >= 16U && bit <= 19U) ||
			(bit >= 23U && bit <= 26U) || (bit >= 31U && bit <= 34U) ||
			(bit >= 38U && bit <= 41U) || (bit >= 46U && bit <= 49U) ||
			(bit >= 54U && bit <= 57U) || (bit >= 62U && bit <= 65U) ||
			(bit >= 69U && bit <= 72U) || (bit >= 77U && bit <= 80U) || bit == 85U;
	if (check == 4U)
		return (bit >= 13U && bit <= 19U) || (bit >= 27U && bit <= 34U) ||
			(bit >= 42U && bit <= 49U) || (bit >= 58U && bit <= 65U) ||
			(bit >= 73U && bit <= 80U);
	if (check == 5U)
		return (bit >= 20U && bit <= 34U) || (bit >= 50U && bit <= 65U) ||
			(bit >= 81U && bit <= 85U);
	if (check == 6U)
		return bit >= 35U && bit <= 65U;
	if (check == 7U)
		return bit >= 66U && bit <= 85U;
	return 0;
}

int gnss_glonass_hamming_85_77(const uint8_t data[77], uint8_t string[85])
{
	uint8_t checks[8] = {0};
	unsigned int check, bit;

	if (data == NULL || string == NULL)
		return -1;
	for (bit = 9U; bit <= 85U; bit++) {
		uint8_t value = data[85U - bit];
		if (value > 1U)
			return -1;
		string[85U - bit] = value;
		checks[0] ^= value;
		for (check = 1U; check <= 7U; check++)
			if (glonass_check_member(check, bit))
				checks[check] ^= value;
	}
	/* Transmission order is beta8, beta7, ... beta1. beta1..beta7
	 * cancel their respective checksums; beta8 makes total parity even. */
	for (check = 1U; check <= 7U; check++) {
		string[85U - check] = checks[check];
		checks[0] ^= checks[check];
	}
	string[77U] = checks[0];
	return 0;
}
