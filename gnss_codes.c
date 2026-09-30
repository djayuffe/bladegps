#include "gnss_codes.h"

#include <math.h>
#include <string.h>

/* BDS-SIS-ICD-B1I-3.0, Table 4-1. Values are one-based G2 stages. */
static const uint8_t b1i_phase_taps[63][3] = {
	{1,3,0},{1,4,0},{1,5,0},{1,6,0},{1,8,0},{1,9,0},{1,10,0},{1,11,0},
	{2,7,0},{3,4,0},{3,5,0},{3,6,0},{3,8,0},{3,9,0},{3,10,0},{3,11,0},
	{4,5,0},{4,6,0},{4,8,0},{4,9,0},{4,10,0},{4,11,0},{5,6,0},{5,8,0},
	{5,9,0},{5,10,0},{5,11,0},{6,8,0},{6,9,0},{6,10,0},{6,11,0},{8,9,0},
	{8,10,0},{8,11,0},{9,10,0},{9,11,0},{10,11,0},{1,2,7},{1,3,4},{1,3,6},
	{1,3,8},{1,3,10},{1,3,11},{1,4,5},{1,4,9},{1,5,6},{1,5,8},{1,5,10},
	{1,5,11},{1,6,9},{1,8,9},{1,9,10},{1,9,11},{2,3,7},{2,5,7},{2,7,9},
	{3,4,5},{3,4,9},{3,5,6},{3,5,8},{3,5,10},{3,5,11},{3,6,9}
};

static int hex_value(char value)
{
	if (value >= '0' && value <= '9')
		return value - '0';
	if (value >= 'a' && value <= 'f')
		return value - 'a' + 10;
	if (value >= 'A' && value <= 'F')
		return value - 'A' + 10;
	return -1;
}

int gnss_beidou_b1i_code(unsigned int prn,
	int8_t chips[BEIDOU_B1I_CODE_LENGTH])
{
	uint8_t g1[11] = {0,1,0,1,0,1,0,1,0,1,0};
	uint8_t g2[11] = {0,1,0,1,0,1,0,1,0,1,0};
	const uint8_t *taps;
	size_t chip;
	int stage;

	if (chips == NULL || prn < 1U || prn > 63U)
		return -1;
	taps = b1i_phase_taps[prn - 1U];

	for (chip = 0; chip < BEIDOU_B1I_CODE_LENGTH; chip++) {
		uint8_t g2_output = g2[11U - taps[0]] ^ g2[11U - taps[1]];
		uint8_t feedback1;
		uint8_t feedback2;

		if (taps[2] != 0U)
			g2_output ^= g2[11U - taps[2]];
		chips[chip] = (g1[0] ^ g2_output) != 0U ? -1 : 1;

		feedback1 = g1[0] ^ g1[1] ^ g1[2] ^ g1[3] ^ g1[4] ^ g1[10];
		feedback2 = g2[0] ^ g2[2] ^ g2[3] ^ g2[6] ^ g2[7] ^ g2[8] ^ g2[9] ^ g2[10];
		for (stage = 0; stage < 10; stage++) {
			g1[stage] = g1[stage + 1];
			g2[stage] = g2[stage + 1];
		}
		g1[10] = feedback1;
		g2[10] = feedback2;
	}
	return 0;
}

int gnss_glonass_l1of_code(int8_t chips[GLONASS_L1OF_CODE_LENGTH])
{
	uint8_t state[9] = {1,1,1,1,1,1,1,1,1};
	size_t chip;
	int stage;

	if (chips == NULL)
		return -1;
	for (chip = 0; chip < GLONASS_L1OF_CODE_LENGTH; chip++) {
		uint8_t feedback = state[4] ^ state[8];
		chips[chip] = state[6] != 0U ? -1 : 1;
		for (stage = 8; stage > 0; stage--)
			state[stage] = state[stage - 1];
		state[0] = feedback;
	}
	return 0;
}

int gnss_glonass_l1of_carrier_hz(int frequency_slot, double *carrier_hz)
{
	if (carrier_hz == NULL || frequency_slot < -7 || frequency_slot > 6)
		return -1;
	*carrier_hz = 1602.0e6 + (double)frequency_slot * 0.5625e6;
	return 0;
}

int gnss_decode_hex_code(const char *hex, size_t chip_count, int8_t *chips)
{
	size_t expected_digits;
	size_t chip;

	if (hex == NULL || chips == NULL || chip_count == 0U)
		return -1;
	expected_digits = (chip_count + 3U) / 4U;
	if (strlen(hex) != expected_digits)
		return -1;

	for (chip = 0; chip < chip_count; chip++) {
		int nibble = hex_value(hex[chip / 4U]);
		unsigned int shift = 3U - (unsigned int)(chip % 4U);
		if (nibble < 0)
			return -1;
		chips[chip] = ((unsigned int)nibble & (1U << shift)) != 0U ? -1 : 1;
	}
	return 0;
}

int gnss_galileo_e1c_secondary_code(
	int8_t chips[GALILEO_E1C_SECONDARY_LENGTH])
{
	/* Galileo OS SIS ICD v2.2, Table 22, CS25_1; final three bits are padding. */
	return gnss_decode_hex_code("380AD90", GALILEO_E1C_SECONDARY_LENGTH, chips);
}

int gnss_beidou_b1i_nh_code(int8_t chips[BEIDOU_B1I_NH_LENGTH])
{
	static const char code[] = "00000100110101001110";
	size_t index;

	if (chips == NULL)
		return -1;
	for (index = 0; index < BEIDOU_B1I_NH_LENGTH; index++)
		chips[index] = code[index] == '1' ? -1 : 1;
	return 0;
}

int gnss_glonass_time_mark(int8_t chips[GLONASS_TIME_MARK_LENGTH])
{
	static const char code[] = "111110001101110101000010010110";
	size_t index;

	if (chips == NULL)
		return -1;
	for (index = 0; index < GLONASS_TIME_MARK_LENGTH; index++)
		chips[index] = code[index] == '1' ? -1 : 1;
	return 0;
}

void gnss_galileo_e1_cboc(double data[GALILEO_E1_CBOC_SUBCHIPS],
	double pilot[GALILEO_E1_CBOC_SUBCHIPS])
{
	const double alpha = sqrt(10.0 / 11.0);
	const double beta = sqrt(1.0 / 11.0);
	size_t subchip;

	for (subchip = 0; subchip < GALILEO_E1_CBOC_SUBCHIPS; subchip++) {
		double boc11 = subchip < 6U ? 1.0 : -1.0;
		double boc61 = (subchip & 1U) == 0U ? 1.0 : -1.0;
		data[subchip] = alpha * boc11 + beta * boc61;
		pilot[subchip] = alpha * boc11 - beta * boc61;
	}
}
