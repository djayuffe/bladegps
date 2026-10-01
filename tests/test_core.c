#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "bladegps.h"

/* gpssim.c's real-time task references this producer-side helper. Unit tests
 * never enter gps_task(), so a local stub keeps the model tests independent
 * from the application entry point in bladegps.c. */
int is_fifo_write_ready(sim_t *sim)
{
	(void)sim;
	return 0;
}

int stop_was_requested(void)
{
	return 0;
}

static void test_time_conversions(void)
{
	datetime_t epoch = {1980, 1, 6, 0, 0, 0.0};
	datetime_t before_century = {2100, 2, 28, 0, 0, 0.0};
	datetime_t after_century = {2100, 3, 1, 0, 0, 0.0};
	gpstime_t g_epoch;
	gpstime_t g_before;
	gpstime_t g_after;
	gpstime_t rollover = {2200, SECONDS_IN_WEEK + 1.25};

	date2gps(&epoch, &g_epoch);
	assert(g_epoch.week == 0);
	assert(g_epoch.sec == 0.0);

	date2gps(&before_century, &g_before);
	date2gps(&after_century, &g_after);
	assert(subGpsTime(g_after, g_before) == SECONDS_IN_DAY);

	normalizeGpsTime(&rollover);
	assert(rollover.week == 2201);
	assert(fabs(rollover.sec - 1.25) < 1.0e-12);
}

static void test_coordinate_round_trip(void)
{
	double llh[3] = {59.3293/R2D, 18.0686/R2D, 30.0};
	double xyz[3];
	double result[3];

	llh2xyz(llh, xyz);
	xyz2llh(xyz, result);
	assert(fabs(result[0] - llh[0]) < 1.0e-10);
	assert(fabs(result[1] - llh[1]) < 1.0e-10);
	assert(fabs(result[2] - llh[2]) < 1.0e-4);
}

static void test_ca_code_balance(void)
{
	int ca[CA_SEQ_LEN];
	int ones = 0;
	int i;

	memset(ca, 0, sizeof(ca));
	codegen(ca, 1);
	for (i = 0; i < CA_SEQ_LEN; i++) {
		assert(ca[i] == 0 || ca[i] == 1);
		ones += ca[i];
	}
	assert(ones == 512);
}

static void test_ephemeris_selection(void)
{
	ephem_t source[EPHEM_ARRAY_SIZE][MAX_SAT];
	ephem_t selected[MAX_SAT];
	gpstime_t now = {2200, 100100.0};

	memset(source, 0, sizeof(source));
	source[0][0].vflg = 1;
	source[0][0].toc.week = 2200;
	source[0][0].toc.sec = 98000.0;
	source[0][0].toe = source[0][0].toc;
	source[0][0].fit_interval = DEFAULT_EPHEMERIS_FIT_HOURS;
	source[0][0].iode = 1;
	source[1][0].vflg = 1;
	source[1][0].toc.week = 2200;
	source[1][0].toc.sec = 100000.0;
	source[1][0].toe = source[1][0].toc;
	source[1][0].fit_interval = DEFAULT_EPHEMERIS_FIT_HOURS;
	source[1][0].iode = 2;

	assert(selectEphemerides(selected, source, 2, now) == 1);
	assert(selected[0].iode == 2);

	now.sec += DEFAULT_EPHEMERIS_FIT_HOURS * SECONDS_IN_HOUR / 2.0 + 1000.0;
	assert(selectEphemerides(selected, source, 2, now) == 0);
}

static void test_signal_profiles(void)
{
	gnss_signal_t signal;
	const gnss_signal_profile_t *profile;

	assert(gnss_signal_parse("gps-l1ca", &signal) == 0);
	assert(signal == GNSS_SIGNAL_GPS_L1CA);
	profile = gnss_signal_profile(signal);
	assert(profile != NULL && profile->waveform_implemented == 1);
	assert(gnss_signal_parse("galileo-e1", &signal) == 0);
	assert(gnss_signal_profile(signal)->system == GNSS_SYSTEM_GALILEO);
	assert(gnss_signal_parse("invalid", &signal) == -1);
	assert(gnss_frequency_fits(1575.42e6, 5.0e6, 1575.42e6, 4.0e6));
	assert(!gnss_frequency_fits(1575.42e6, 5.0e6, 1561.098e6, 4.5e6));
}

static void test_rf_renderer_and_allocator(void)
{
	static const int8_t code[4] = {1,-1,1,-1};
	static const int8_t data[3] = {1,1,-1};
	static const int8_t overlay[2] = {1,-1};
	gnss_rf_channel_t whole = {
		.enabled=1,.modulation=GNSS_RF_BPSK,.system=GNSS_SYSTEM_GPS,.prn=1U,
		.carrier_hz=10000000.0,.doppler_hz=137.25,.amplitude=700.0,
		.data_code=code,.code_length=4U,.code_rate_hz=1000.0,.code_phase=1.25,
		.data_symbols=data,.data_symbol_count=3U,.data_rate_hz=50.0,.data_phase=0.75,
		.overlay_symbols=overlay,.overlay_symbol_count=2U,.overlay_rate_hz=25.0,
		.overlay_phase=0.25,.carrier_phase=0.125
	};
	gnss_rf_channel_t split=whole;
	gnss_rf_candidate_t candidates[] = {
		{GNSS_SYSTEM_GPS,1U,1575.42e6,2.5e6,0.50,1},
		{GNSS_SYSTEM_GALILEO,2U,1575.42e6,4.0e6,0.80,1},
		{GNSS_SYSTEM_BEIDOU,3U,1561.098e6,4.092e6,1.00,1},
		{GNSS_SYSTEM_GPS,4U,1575.42e6,2.5e6,0.70,0},
		{GNSS_SYSTEM_GPS,5U,1575.42e6,2.5e6,0.20,1},
		{GNSS_SYSTEM_GPS,6U,1575.42e6,2.5e6,0.60,1}
	};
	int16_t one_block[200], split_blocks[200];
	size_t selected[2], selected_count=0U;
	gnss_rf_channel_t bank[2]={{0}},desired[2];

	assert(gnss_rf_render(&whole,1U,10000000.0,1000000.0,one_block,100U)==0);
	assert(gnss_rf_render(&split,1U,10000000.0,1000000.0,split_blocks,40U)==0);
	assert(gnss_rf_render(&split,1U,10000000.0,1000000.0,split_blocks+80U,60U)==0);
	assert(memcmp(one_block,split_blocks,sizeof(one_block))==0);
	assert(whole.carrier_phase==split.carrier_phase && whole.code_phase==split.code_phase &&
		whole.data_phase==split.data_phase && whole.overlay_phase==split.overlay_phase);
	split=whole; split.data_rate_hz=NAN;
	assert(gnss_rf_validate_channel(&split,10000000.0,1000000.0)==-1);
	split=whole; split.modulation=(gnss_rf_modulation_t)99;
	assert(gnss_rf_validate_channel(&split,10000000.0,1000000.0)==-1);

	/* The 30 MHz test passband includes GPS/Galileo but excludes BeiDou. */
	assert(gnss_rf_allocate(candidates,6U,1575.42e6,30.0e6,0.30,
		selected,2U,&selected_count)==0);
	assert(selected_count==2U && selected[0]==1U && selected[1]==5U);
	desired[0]=whole; desired[1]=whole; desired[1].system=GNSS_SYSTEM_GALILEO;
	desired[1].prn=2U; desired[1].carrier_hz=10001000.0;
	assert(gnss_rf_reconcile(bank,2U,desired,2U)==0);
	bank[0].carrier_phase=1.25; bank[0].code_phase=2.5;
	bank[0].data_phase=1.5; bank[0].overlay_phase=0.5;
	desired[0]=desired[1]; desired[1]=whole;
	assert(gnss_rf_reconcile(bank,2U,desired,2U)==0);
	assert(bank[1].carrier_phase==1.25 && bank[1].code_phase==2.5 &&
		bank[1].data_phase==1.5 && bank[1].overlay_phase==0.5);
	desired[1]=desired[0];
	assert(gnss_rf_reconcile(bank,2U,desired,2U)==-1);
}

static void test_constellation_rf_sequences(void)
{
	int8_t galileo_b[GALILEO_E1_CODE_LENGTH],galileo_c[GALILEO_E1_CODE_LENGTH];
	int8_t galileo_secondary[GALILEO_E1C_SECONDARY_LENGTH];
	int8_t beidou[BEIDOU_B1I_CODE_LENGTH],nh[BEIDOU_B1I_NH_LENGTH];
	int8_t glonass[GLONASS_L1OF_CODE_LENGTH];
	int8_t glonass_symbols[GLONASS_L1OF_STRING_SYMBOLS];
	int8_t nav_symbols[300];
	uint8_t glonass_string[85]={0},previous=0U;
	uint8_t nav_bits[300];
	int16_t iq[256];
	gnss_rf_channel_t channel;
	size_t index;

	assert(gnss_galileo_e1_primary_code(1U,GALILEO_E1_COMPONENT_B,galileo_b)==0);
	assert(gnss_galileo_e1_primary_code(1U,GALILEO_E1_COMPONENT_C,galileo_c)==0);
	assert(gnss_galileo_e1c_secondary_code(galileo_secondary)==0);
	for(index=0U;index<300U;index++) nav_bits[index]=(uint8_t)(index&1U);
	assert(gnss_rf_bits_to_symbols(nav_bits,300U,nav_symbols)==0);
	memset(&channel,0,sizeof(channel));
	channel.enabled=1; channel.modulation=GNSS_RF_GALILEO_E1;
	channel.system=GNSS_SYSTEM_GALILEO; channel.prn=1U;
	channel.carrier_hz=1575.42e6; channel.amplitude=500.0;
	channel.data_code=galileo_b; channel.pilot_code=galileo_c;
	channel.code_length=GALILEO_E1_CODE_LENGTH; channel.code_rate_hz=1.023e6;
	channel.data_symbols=nav_symbols; channel.data_symbol_count=300U;
	channel.data_rate_hz=250.0; channel.overlay_symbols=galileo_secondary;
	channel.overlay_symbol_count=GALILEO_E1C_SECONDARY_LENGTH;
	channel.overlay_rate_hz=250.0;
	assert(gnss_rf_render(&channel,1U,1575.42e6,5.0e6,iq,128U)==0);
	{
		double alpha=sqrt(10.0/11.0),beta=sqrt(1.0/11.0);
		double expected=500.0*(galileo_b[0]*(alpha+beta)-
			galileo_c[0]*galileo_secondary[0]*(alpha-beta))/sqrt(2.0);
		assert(iq[0]==(int16_t)lrint(expected) && iq[1]==0);
	}

	assert(gnss_beidou_b1i_code(1U,beidou)==0);
	assert(gnss_beidou_b1i_nh_code(nh)==0);
	channel.modulation=GNSS_RF_BPSK; channel.system=GNSS_SYSTEM_BEIDOU;
	channel.carrier_hz=1561.098e6; channel.data_code=beidou; channel.pilot_code=NULL;
	channel.code_length=BEIDOU_B1I_CODE_LENGTH; channel.code_rate_hz=2.046e6;
	channel.data_rate_hz=50.0; channel.overlay_symbols=nh;
	channel.overlay_symbol_count=BEIDOU_B1I_NH_LENGTH; channel.overlay_rate_hz=1000.0;
	channel.code_phase=channel.data_phase=channel.overlay_phase=channel.carrier_phase=0.0;
	assert(gnss_rf_render(&channel,1U,1561.098e6,5.0e6,iq,128U)==0);

	for(index=0U;index<85U;index++) glonass_string[index]=(uint8_t)((index/3U)&1U);
	assert(gnss_glonass_l1of_symbols(glonass_string,&previous,glonass_symbols)==0);
	assert(previous<=1U && glonass_symbols[0]==1 && glonass_symbols[1]==-1);
	assert(gnss_glonass_l1of_code(glonass)==0);
	channel.system=GNSS_SYSTEM_GLONASS; channel.carrier_hz=1602.0e6;
	channel.data_code=glonass; channel.code_length=GLONASS_L1OF_CODE_LENGTH;
	channel.code_rate_hz=0.511e6; channel.data_symbols=glonass_symbols;
	channel.data_symbol_count=GLONASS_L1OF_STRING_SYMBOLS; channel.data_rate_hz=100.0;
	channel.overlay_symbols=NULL; channel.overlay_symbol_count=0U;
	channel.overlay_rate_hz=channel.overlay_phase=0.0;
	channel.code_phase=channel.data_phase=channel.carrier_phase=0.0;
	assert(gnss_rf_render(&channel,1U,1602.0e6,2.0e6,iq,128U)==0);
	glonass_string[2]=2U;
	assert(gnss_glonass_l1of_symbols(glonass_string,&previous,glonass_symbols)==-1);
}

static uint32_t code_bit_checksum(const int8_t *chips, size_t count)
{
	uint32_t hash = UINT32_C(2166136261);
	size_t index;

	for (index = 0; index < count; index++) {
		uint8_t bit = chips[index] < 0 ? 1U : 0U;
		hash = (hash ^ bit) * UINT32_C(16777619);
	}
	return hash;
}

static void test_multi_gnss_codes(void)
{
	int8_t b1i[BEIDOU_B1I_CODE_LENGTH];
	int8_t glonass[GLONASS_L1OF_CODE_LENGTH];
	int8_t galileo[GALILEO_E1_CODE_LENGTH];
	int8_t secondary[GLONASS_TIME_MARK_LENGTH];
	int8_t decoded[8];
	double carrier;
	double data[GALILEO_E1_CBOC_SUBCHIPS];
	double pilot[GALILEO_E1_CBOC_SUBCHIPS];
	size_t index;
	int negative_count;

	assert(gnss_beidou_b1i_code(1, b1i) == 0);
	assert(code_bit_checksum(b1i, BEIDOU_B1I_CODE_LENGTH) == UINT32_C(0x58325b3a));
	negative_count = 0;
	for (index = 0; index < BEIDOU_B1I_CODE_LENGTH; index++)
		negative_count += b1i[index] < 0;
	assert(negative_count == 1023);
	assert(gnss_beidou_b1i_code(38, b1i) == 0);
	assert(code_bit_checksum(b1i, BEIDOU_B1I_CODE_LENGTH) == UINT32_C(0x130a655e));
	assert(gnss_beidou_b1i_code(63, b1i) == 0);
	assert(code_bit_checksum(b1i, BEIDOU_B1I_CODE_LENGTH) == UINT32_C(0x29af803e));
	assert(gnss_beidou_b1i_code(0, b1i) == -1);
	assert(gnss_beidou_b1i_code(64, b1i) == -1);

	assert(gnss_glonass_l1of_code(glonass) == 0);
	assert(code_bit_checksum(glonass, GLONASS_L1OF_CODE_LENGTH) == UINT32_C(0x143e2769));
	assert(gnss_glonass_l1of_carrier_hz(-7, &carrier) == 0);
	assert(carrier == 1598062500.0);
	assert(gnss_glonass_l1of_carrier_hz(6, &carrier) == 0);
	assert(carrier == 1605375000.0);
	assert(gnss_glonass_l1of_carrier_hz(7, &carrier) == -1);

	assert(gnss_decode_hex_code("A5", 8, decoded) == 0);
	assert(decoded[0] == -1 && decoded[1] == 1 && decoded[2] == -1 && decoded[3] == 1);
	assert(decoded[4] == 1 && decoded[5] == -1 && decoded[6] == 1 && decoded[7] == -1);
	assert(gnss_decode_hex_code("G5", 8, decoded) == -1);
	assert(gnss_galileo_e1_primary_code(1, GALILEO_E1_COMPONENT_B, galileo) == 0);
	assert(code_bit_checksum(galileo, GALILEO_E1_CODE_LENGTH) == UINT32_C(0x2c3f6f93));
	assert(gnss_galileo_e1_primary_code(50, GALILEO_E1_COMPONENT_B, galileo) == 0);
	assert(code_bit_checksum(galileo, GALILEO_E1_CODE_LENGTH) == UINT32_C(0x25f2a4f9));
	assert(gnss_galileo_e1_primary_code(1, GALILEO_E1_COMPONENT_C, galileo) == 0);
	assert(code_bit_checksum(galileo, GALILEO_E1_CODE_LENGTH) == UINT32_C(0x2f729bf5));
	assert(gnss_galileo_e1_primary_code(50, GALILEO_E1_COMPONENT_C, galileo) == 0);
	assert(code_bit_checksum(galileo, GALILEO_E1_CODE_LENGTH) == UINT32_C(0xc0099bb1));
	assert(gnss_galileo_e1_primary_code(0, GALILEO_E1_COMPONENT_B, galileo) == -1);
	assert(gnss_galileo_e1_primary_code(51, GALILEO_E1_COMPONENT_C, galileo) == -1);
	assert(gnss_galileo_e1_primary_code(1, (galileo_e1_component_t)99, galileo) == -1);
	assert(gnss_galileo_e1c_secondary_code(secondary) == 0);
	assert(code_bit_checksum(secondary, GALILEO_E1C_SECONDARY_LENGTH) == UINT32_C(0x45d5cfa3));
	assert(gnss_beidou_b1i_nh_code(secondary) == 0);
	assert(code_bit_checksum(secondary, BEIDOU_B1I_NH_LENGTH) == UINT32_C(0xf478aba7));
	assert(gnss_glonass_time_mark(secondary) == 0);
	assert(code_bit_checksum(secondary, GLONASS_TIME_MARK_LENGTH) == UINT32_C(0x94c5d52b));

	gnss_galileo_e1_cboc(data, pilot);
	for (index = 0; index < GALILEO_E1_CBOC_SUBCHIPS; index++) {
		assert(isfinite(data[index]) && isfinite(pilot[index]));
		assert(fabs(data[index] - pilot[index]) > 0.0);
	}
	assert(fabs(data[0] - (sqrt(10.0/11.0) + sqrt(1.0/11.0))) < 1.0e-14);
	assert(fabs(pilot[0] - (sqrt(10.0/11.0) - sqrt(1.0/11.0))) < 1.0e-14);
}

static void test_multi_gnss_fec(void)
{
	static const uint8_t convolution_input[10] = {1,0,1,1,0,0,0,0,0,0};
	static const uint8_t convolution_expected[20] = {
		1,0,1,1,0,1,1,1,0,0,0,0,0,1,0,0,1,1,1,0
	};
	static const uint8_t bch_input[11] = {1,0,1,1,0,0,1,0,1,0,1};
	static const uint8_t bch_expected[15] = {1,0,1,1,0,0,1,0,1,0,1,0,0,1,0};
	static const char check_text[] = "123456789";
	uint8_t check_bits[72];
	uint8_t encoded[20];
	uint8_t codeword[15];
	uint8_t interleaved[30];
	uint8_t matrix_input[240];
	uint8_t matrix_output[240];
	uint8_t glonass_data[77];
	uint8_t glonass_string[85];
	uint32_t glonass_check_bits = 0U;
	size_t byte, bit;

	for (byte = 0; byte < 9U; byte++)
		for (bit = 0; bit < 8U; bit++)
			check_bits[byte*8U+bit] = (uint8_t)
				(((unsigned char)check_text[byte] >> (7U-bit)) & 1U);
	assert(gnss_crc24q_bits(check_bits, 72) == UINT32_C(0xcde703));
	assert(gnss_galileo_convolutional_encode(convolution_input, 10, encoded, 20) == 0);
	assert(memcmp(encoded, convolution_expected, sizeof(encoded)) == 0);
	assert(gnss_beidou_bch15_11(bch_input, codeword) == 0);
	assert(memcmp(codeword, bch_expected, sizeof(codeword)) == 0);
	assert(gnss_beidou_interleave_2x15(codeword, bch_expected, interleaved) == 0);
	for (bit = 0; bit < 15U; bit++) {
		assert(interleaved[bit*2U] == codeword[bit]);
		assert(interleaved[bit*2U+1U] == bch_expected[bit]);
	}
	for (bit = 0; bit < 240U; bit++)
		matrix_input[bit] = (uint8_t)(bit & 1U);
	assert(gnss_block_interleave(matrix_input, 30, 8, matrix_output, 240) == 0);
	for (bit = 0; bit < 240U; bit++)
		assert(matrix_output[(bit % 8U)*30U + bit/8U] == matrix_input[bit]);
	for (bit = 0; bit < 77U; bit++)
		glonass_data[bit] = (uint8_t)(((bit * 13U + bit / 5U) >> 1U) & 1U);
	assert(gnss_glonass_hamming_85_77(glonass_data, glonass_string) == 0);
	for (bit = 77U; bit < 85U; bit++)
		glonass_check_bits = (glonass_check_bits << 1) | glonass_string[bit];
	assert(glonass_check_bits == UINT32_C(0xa8));
	for (bit = 0; bit < 77U; bit++)
		assert(glonass_string[bit] == glonass_data[bit]);
	glonass_data[3] = 2U;
	assert(gnss_glonass_hamming_85_77(glonass_data, glonass_string) == -1);
}

static void test_galileo_inav_pages(void)
{
	static const char *ssp_plain[3] = {"00000100", "00101011", "00101111"};
	static const char *ssp_encoded[3] = {
		"1110100100100101", "0110110001001110", "1101000000111110"
	};
	static const uint8_t sync[10] = {0,1,0,1,1,0,0,0,0,0};
	uint8_t input[120] = {0};
	uint8_t encoded[240];
	uint8_t word[GALILEO_INAV_WORD_BITS];
	uint8_t osnma[GALILEO_INAV_OSNMA_BITS];
	uint8_t sar[GALILEO_INAV_SAR_BITS];
	uint8_t even[GALILEO_INAV_PAGE_PART_SYMBOLS];
	uint8_t odd[GALILEO_INAV_PAGE_PART_SYMBOLS];
	uint32_t crc;
	size_t pattern, index;

	/* Galileo OS SIS ICD 2.2 Table 85: the last 16 encoder symbols are
	 * authoritative vectors for register orientation and G2 inversion. */
	for (pattern = 0; pattern < 3U; pattern++) {
		memset(input, 0, sizeof(input));
		for (index = 0; index < 8U; index++)
			input[106U + index] = (uint8_t)(ssp_plain[pattern][index] - '0');
		assert(gnss_galileo_convolutional_encode(input, 120U, encoded,
			sizeof(encoded)) == 0);
		for (index = 0; index < 16U; index++)
			assert(encoded[224U + index] ==
				(uint8_t)(ssp_encoded[pattern][index] - '0'));
	}

	for (index = 0; index < GALILEO_INAV_WORD_BITS; index++)
		word[index] = (uint8_t)(((index * 7U) + 3U) & 1U);
	for (index = 0; index < GALILEO_INAV_OSNMA_BITS; index++)
		osnma[index] = (uint8_t)((index / 3U) & 1U);
	for (index = 0; index < GALILEO_INAV_SAR_BITS; index++)
		sar[index] = (uint8_t)((index / 2U) & 1U);
	assert(gnss_galileo_inav_e1b_page(word, osnma, sar, 2U,
		GALILEO_INAV_SSP2, even, odd, &crc) == 0);
	assert(memcmp(even, sync, sizeof(sync)) == 0);
	assert(memcmp(odd, sync, sizeof(sync)) == 0);
	assert(crc == UINT32_C(0x232d71));
	for (index = 0; index < GALILEO_INAV_PAGE_PART_SYMBOLS; index++) {
		assert(even[index] <= 1U);
		assert(odd[index] <= 1U);
	}
	word[0] = 2U;
	assert(gnss_galileo_inav_e1b_page(word, osnma, sar, 0U,
		GALILEO_INAV_SSP1, even, odd, NULL) == -1);
	assert(gnss_galileo_inav_e1b_word_type(0U) == 16);
	assert(gnss_galileo_inav_e1b_word_type(21U) == 1);
	assert(gnss_galileo_inav_e1b_word_type(29U) == 16);
	assert(gnss_galileo_inav_e1b_word_type(30U) == -1);
	assert(gnss_galileo_inav_ssp_for_second(0U) == GALILEO_INAV_SSP3);
	assert(gnss_galileo_inav_ssp_for_second(2U) == GALILEO_INAV_SSP1);
	assert(gnss_galileo_inav_ssp_for_second(4U) == GALILEO_INAV_SSP2);
}

static uint32_t unpack_bits(const uint8_t *bits, size_t offset, size_t width)
{
	uint32_t value = 0U;
	size_t index;
	for (index = 0; index < width; index++)
		value = (value << 1) | bits[offset + index];
	return value;
}

static void test_galileo_inav_words(void)
{
	galileo_inav_word1_t w1 = {0x155U, 10080U, -1234567, 0x12345678U, 0x87654321U};
	galileo_inav_word2_t w2 = {7U, INT32_MIN, INT32_MAX, -1, -8192};
	galileo_inav_word3_t w3 = {9U, -8388608, -32768, 32767, -1, 0, 1, 255U};
	galileo_inav_word4_t w4 = {1023U, 36U, -2, 3, 10079U, -1073741824, 1048575, -32};
	galileo_inav_word5_t w5 = {2047U, -1024, 8191, 0x15U, -512, 511,
		3U, 2U, 1U, 0U, 4095U, 604799U};
	uint8_t word[GALILEO_INAV_WORD_BITS];

	assert(gnss_galileo_inav_word1(&w1, word) == 0);
	assert(unpack_bits(word,0,6) == 1U && unpack_bits(word,6,10) == 0x155U);
	assert(unpack_bits(word,16,14) == 10080U);
	assert(unpack_bits(word,30,32) == (uint32_t)-1234567);
	assert(unpack_bits(word,62,32) == 0x12345678U);
	assert(unpack_bits(word,94,32) == 0x87654321U && unpack_bits(word,126,2) == 0U);
	w1.toe = 16384U;
	assert(gnss_galileo_inav_word1(&w1, word) == -1);

	assert(gnss_galileo_inav_word2(&w2, word) == 0);
	assert(unpack_bits(word,0,6) == 2U && unpack_bits(word,16,32) == UINT32_C(0x80000000));
	assert(unpack_bits(word,112,14) == 0x2000U && unpack_bits(word,126,2) == 0U);
	w2.inclination_rate = 8192;
	assert(gnss_galileo_inav_word2(&w2, word) == -1);

	assert(gnss_galileo_inav_word3(&w3, word) == 0);
	assert(unpack_bits(word,0,6) == 3U && unpack_bits(word,16,24) == 0x800000U);
	assert(unpack_bits(word,40,16) == 0x8000U && unpack_bits(word,120,8) == 255U);
	w3.omega_rate = 8388608;
	assert(gnss_galileo_inav_word3(&w3, word) == -1);

	assert(gnss_galileo_inav_word4(&w4, word) == 0);
	assert(unpack_bits(word,0,6) == 4U && unpack_bits(word,6,10) == 1023U);
	assert(unpack_bits(word,16,6) == 36U && unpack_bits(word,68,31) == 0x40000000U);
	assert(unpack_bits(word,120,6) == 0x20U && unpack_bits(word,126,2) == 0U);
	w4.svid = 0U;
	assert(gnss_galileo_inav_word4(&w4, word) == -1);

	assert(gnss_galileo_inav_word5(&w5, word) == 0);
	assert(unpack_bits(word,0,6) == 5U && unpack_bits(word,6,11) == 2047U);
	assert(unpack_bits(word,17,11) == 0x400U && unpack_bits(word,28,14) == 0x1fffU);
	assert(unpack_bits(word,42,5) == 0x15U && unpack_bits(word,47,10) == 0x200U);
	assert(unpack_bits(word,73,12) == 4095U && unpack_bits(word,85,20) == 604799U);
	assert(unpack_bits(word,105,23) == 0U);
	w5.tow = 1048576U;
	assert(gnss_galileo_inav_word5(&w5, word) == -1);
}

static void test_beidou_navigation_subframe(void)
{
	static const uint8_t preamble[11] = {1,1,1,0,0,0,1,0,0,1,0};
	uint8_t information[BEIDOU_NAV_INFORMATION_BITS];
	uint8_t payload[BEIDOU_NAV_PAYLOAD_BITS];
	uint8_t subframe[BEIDOU_NAV_SUBFRAME_BITS];
	uint8_t expected[15];
	uint32_t recovered_sow;
	size_t index;

	for (index = 0; index < BEIDOU_NAV_INFORMATION_BITS; index++)
		information[index] = (uint8_t)(((index * 5U + index / 7U) >> 1U) & 1U);
	assert(gnss_beidou_nav_encode_subframe(information, subframe) == 0);
	assert(memcmp(subframe, information, 15U) == 0);
	assert(gnss_beidou_bch15_11(information + 15U, expected) == 0);
	assert(memcmp(subframe + 15U, expected, 15U) == 0);
	for (index = 0; index < 11U; index++) {
		assert(subframe[30U + index * 2U] == information[26U + index]);
		assert(subframe[31U + index * 2U] == information[37U + index]);
	}

	for (index = 0; index < BEIDOU_NAV_PAYLOAD_BITS; index++)
		payload[index] = (uint8_t)((index / 4U) & 1U);
	assert(gnss_beidou_nav_build_subframe(3U, UINT32_C(345678), payload,
		subframe) == 0);
	assert(memcmp(subframe, preamble, sizeof(preamble)) == 0);
	assert(unpack_bits(subframe, 11U, 4U) == 0U);
	assert(unpack_bits(subframe, 15U, 3U) == 3U);
	recovered_sow = unpack_bits(subframe, 18U, 8U) << 12;
	for (index = 0; index < 11U; index++)
		recovered_sow |= (uint32_t)subframe[30U + index * 2U] << (11U - index);
	recovered_sow |= subframe[31U];
	assert(recovered_sow == UINT32_C(345678));
	assert(gnss_beidou_nav_build_subframe(0U, 0U, payload, subframe) == -1);
	assert(gnss_beidou_nav_build_subframe(6U, 0U, payload, subframe) == -1);
	assert(gnss_beidou_nav_build_subframe(1U, 604800U, payload, subframe) == -1);
	payload[9] = 2U;
	assert(gnss_beidou_nav_build_subframe(1U, 0U, payload, subframe) == -1);
}

static void recover_beidou_information(const uint8_t subframe[300],
	uint8_t information[224])
{
	unsigned int word, bit;
	memcpy(information, subframe, 15U);
	memcpy(information + 15U, subframe + 15U, 11U);
	for (word = 1U; word < 10U; word++)
		for (bit = 0U; bit < 11U; bit++) {
			information[26U+(word-1U)*22U+bit] = subframe[word*30U+bit*2U];
			information[37U+(word-1U)*22U+bit] = subframe[word*30U+bit*2U+1U];
		}
}

static void test_beidou_d1_ephemeris(void)
{
	beidou_d1_ephemeris_t fields = {
		.toe = UINT32_C(0x15555), .delta_mean_motion = -32768,
		.cuc = 131071, .mean_anomaly = INT32_MIN, .cus = -131072,
		.crc = -1, .crs = 1, .eccentricity = UINT32_C(0x89abcdef),
		.sqrt_a = UINT32_C(0xfedcba98), .inclination0 = INT32_MAX,
		.cic = -2, .omega_rate = -8388608, .cis = 2,
		.inclination_rate = 8191, .omega0 = -123456789,
		.argument_of_perigee = 123456789
	};
	uint8_t sf2[300], sf3[300], info2[224], info3[224];
	const uint8_t *p2, *p3;

	assert(gnss_beidou_d1_ephemeris_subframes(&fields,604794U,sf2,sf3) == 0);
	recover_beidou_information(sf2,info2);
	recover_beidou_information(sf3,info3);
	assert(unpack_bits(info2,15,3) == 2U && unpack_bits(info3,15,3) == 3U);
	assert(unpack_bits(info2,18,20) == 0U);
	assert(unpack_bits(info3,18,20) == 6U);
	p2 = info2 + 38U; p3 = info3 + 38U;
	assert(unpack_bits(p2,0,16) == 0x8000U);
	assert(unpack_bits(p2,16,18) == 0x1ffffU);
	assert(unpack_bits(p2,34,32) == UINT32_C(0x80000000));
	assert(unpack_bits(p2,66,32) == UINT32_C(0x89abcdef));
	assert(unpack_bits(p2,184,2) == 2U);
	assert(unpack_bits(p3,0,15) == 0x5555U);
	assert(unpack_bits(p3,15,32) == UINT32_C(0x7fffffff));
	assert(unpack_bits(p3,65,24) == UINT32_C(0x800000));
	assert(unpack_bits(p3,121,32) == (uint32_t)-123456789);
	assert(unpack_bits(p3,153,32) == UINT32_C(123456789));
	assert(p3[185] == 0U);
	fields.inclination_rate = 8192;
	assert(gnss_beidou_d1_ephemeris_subframes(&fields,0U,sf2,sf3) == -1);
}

static void test_beidou_d1_clock(void)
{
	beidou_d1_clock_t fields = {
		.health=1U,.aodc=31U,.urai=15U,.aode=30U,.week=8191U,.toc=75599U,
		.tgd1=-512,.tgd2=511,.alpha={-128,127,-1,1},
		.beta={1,-1,64,-64},.af0=-8388608,.af1=2097151,.af2=-1024
	};
	uint8_t sf[300],info[224];
	const uint8_t *p;
	assert(gnss_beidou_d1_clock_subframe(&fields,604799U,sf)==0);
	recover_beidou_information(sf,info); p=info+38U;
	assert(unpack_bits(info,15,3)==1U && unpack_bits(info,18,20)==604799U);
	assert(unpack_bits(p,0,1)==1U && unpack_bits(p,1,5)==31U);
	assert(unpack_bits(p,6,4)==15U && unpack_bits(p,10,13)==8191U);
	assert(unpack_bits(p,23,17)==75599U);
	assert(unpack_bits(p,40,10)==0x200U && unpack_bits(p,50,10)==0x1ffU);
	assert(unpack_bits(p,60,8)==0x80U && unpack_bits(p,68,8)==0x7fU);
	assert(unpack_bits(p,124,11)==0x400U);
	assert(unpack_bits(p,135,24)==0x800000U);
	assert(unpack_bits(p,159,22)==0x1fffffU && unpack_bits(p,181,5)==30U);
	fields.toc=75600U;
	assert(gnss_beidou_d1_clock_subframe(&fields,0U,sf)==-1);
}

static void test_beidou_almanac_pages(void)
{
	beidou_almanac_t fields={
		.sqrt_a=UINT32_C(0xabcdef),.clock_rate=-1024,.clock_bias=1023,
		.omega0=-8388608,.eccentricity=UINT32_C(0x1ffff),
		.inclination_offset=-32768,.toa=255U,.omega_rate=65535,
		.argument_of_perigee=8388607,.mean_anomaly=-1,.identifier=3U
	};
	uint8_t subframe[300],information[224];
	const uint8_t *payload;
	assert(gnss_beidou_almanac_subframe(BEIDOU_NAV_D1,4U,24U,12345U,
		&fields,subframe)==0);
	recover_beidou_information(subframe,information); payload=information+38U;
	assert(unpack_bits(information,15U,3U)==4U);
	assert(unpack_bits(information,18U,20U)==12345U);
	assert(unpack_bits(payload,0U,1U)==0U && unpack_bits(payload,1U,7U)==24U);
	assert(unpack_bits(payload,8U,24U)==UINT32_C(0xabcdef));
	assert(unpack_bits(payload,32U,11U)==0x400U);
	assert(unpack_bits(payload,43U,11U)==0x3ffU);
	assert(unpack_bits(payload,54U,24U)==UINT32_C(0x800000));
	assert(unpack_bits(payload,78U,17U)==UINT32_C(0x1ffff));
	assert(unpack_bits(payload,95U,16U)==UINT32_C(0x8000));
	assert(unpack_bits(payload,111U,8U)==255U);
	assert(unpack_bits(payload,119U,17U)==65535U);
	assert(unpack_bits(payload,136U,24U)==UINT32_C(0x7fffff));
	assert(unpack_bits(payload,160U,24U)==UINT32_C(0xffffff));
	assert(unpack_bits(payload,184U,2U)==3U);
	assert(gnss_beidou_almanac_subframe(BEIDOU_NAV_D2,5U,37U,0U,
		&fields,subframe)==0);
	assert(gnss_beidou_almanac_subframe(BEIDOU_NAV_D1,5U,10U,0U,
		&fields,subframe)==-1);
	assert(gnss_beidou_almanac_subframe(BEIDOU_NAV_D2,4U,37U,0U,
		&fields,subframe)==-1);
	fields.identifier=4U;
	assert(gnss_beidou_almanac_subframe(BEIDOU_NAV_D1,4U,1U,0U,
		&fields,subframe)==-1);
}

static void test_beidou_d2_basic_pages(void)
{
	beidou_d1_clock_t clock={
		.health=1U,.aodc=17U,.urai=9U,.aode=19U,.week=4097U,.toc=65535U,
		.tgd1=-511,.tgd2=510,.alpha={-128,127,-1,1},.beta={2,-2,64,-64},
		.af0=-8388608,.af1=-1,.af2=1023
	};
	beidou_d1_ephemeris_t ephemeris={
		.toe=UINT32_C(0x15555),.delta_mean_motion=-32768,.cuc=-1,
		.mean_anomaly=INT32_MIN,.cus=131071,.crc=-131072,.crs=131071,
		.eccentricity=UINT32_C(0x89abcdef),.sqrt_a=UINT32_C(0xfedcba98),
		.inclination0=INT32_MAX,.cic=-2,.omega_rate=-8388608,.cis=2,
		.inclination_rate=-8192,.omega0=-123456789,
		.argument_of_perigee=123456789
	};
	uint8_t pages[10][300],information[224];
	const uint8_t *payload;
	assert(gnss_beidou_d2_basic_pages(&clock,&ephemeris,604794U,pages)==0);
	recover_beidou_information(pages[0],information); payload=information+38U;
	assert(unpack_bits(information,18U,20U)==604794U);
	assert(unpack_bits(payload,0U,4U)==1U && unpack_bits(payload,4U,1U)==1U);
	assert(unpack_bits(payload,5U,5U)==17U && unpack_bits(payload,10U,4U)==9U);
	assert(unpack_bits(payload,14U,13U)==4097U && unpack_bits(payload,27U,17U)==65535U);
	assert(unpack_bits(payload,44U,10U)==0x201U && unpack_bits(payload,54U,10U)==510U);
	recover_beidou_information(pages[2],information); payload=information+38U;
	assert(unpack_bits(payload,0U,4U)==3U);
	assert(unpack_bits(payload,42U,24U)==UINT32_C(0x800000));
	assert(unpack_bits(payload,66U,4U)==15U);
	recover_beidou_information(pages[3],information); payload=information+38U;
	assert(unpack_bits(payload,4U,18U)==UINT32_C(0x3ffff));
	assert(unpack_bits(payload,22U,11U)==1023U && unpack_bits(payload,33U,5U)==19U);
	assert(unpack_bits(payload,38U,16U)==UINT32_C(0x8000));
	recover_beidou_information(pages[9],information); payload=information+38U;
	assert(unpack_bits(information,18U,20U)==21U);
	assert(unpack_bits(payload,0U,4U)==10U);
	assert(unpack_bits(payload,4U,5U)==(UINT32_C(123456789)&31U));
	assert(unpack_bits(payload,9U,14U)==UINT32_C(0x2000));
	clock.af2=1024;
	assert(gnss_beidou_d2_basic_pages(&clock,&ephemeris,0U,pages)==-1);
}

static uint32_t glonass_field(const uint8_t string[85], unsigned int first,
	unsigned int width)
{
	uint32_t value = 0U;
	unsigned int bit;
	for (bit=0U; bit<width; bit++)
		value |= (uint32_t)string[85U-(first+bit)] << bit;
	return value;
}

static void test_glonass_immediate_strings(void)
{
	glonass_gnav_immediate_t fields = {
		.tk_seconds=23U*3600U+59U*60U+30U, .tb=95U, .bn=5U,
		.p1=3U, .p2=1U, .p3=1U, .p4=1U, .p=2U, .ln=1U,
		.ft=12U, .en=17U, .slot=24U, .mode=2U, .nt=1461U,
		.position={-1234567,2345678,-3456789},
		.velocity={765432,-654321,543210},
		.acceleration={-15,14,-13}, .gamma=-511, .tau=-1048575,
		.delta_tau=15
	};
	uint8_t strings[4][GLONASS_GNAV_STRING_BITS];
	assert(gnss_glonass_gnav_immediate_strings(&fields,strings) == 0);
	assert(glonass_field(strings[0],81,4) == 1U);
	assert(glonass_field(strings[1],81,4) == 2U);
	assert(glonass_field(strings[2],81,4) == 3U);
	assert(glonass_field(strings[3],81,4) == 4U);
	assert(glonass_field(strings[0],65,12) == ((23U<<7)|(59U<<1)|1U));
	assert(glonass_field(strings[0],9,26) == 1234567U);
	assert(glonass_field(strings[0],35,1) == 1U);
	assert(glonass_field(strings[1],9,26) == 2345678U);
	assert(glonass_field(strings[1],35,1) == 0U);
	assert(glonass_field(strings[2],69,10) == 511U);
	assert(glonass_field(strings[2],79,1) == 1U);
	assert(glonass_field(strings[1],70,7) == 95U);
	assert(glonass_field(strings[1],78,3) == 5U);
	assert(glonass_field(strings[3],9,2) == 2U);
	assert(glonass_field(strings[3],11,5) == 24U);
	assert(glonass_field(strings[3],16,11) == 1461U);
	assert(glonass_field(strings[3],59,21) == 1048575U);
	assert(glonass_field(strings[3],80,1) == 1U);
	fields.tk_seconds = 1U;
	assert(gnss_glonass_gnav_immediate_strings(&fields,strings) == -1);
	fields.tk_seconds = 0U; fields.position[0] = (1<<26);
	assert(gnss_glonass_gnav_immediate_strings(&fields,strings) == -1);
}

static void test_glonass_string5(void)
{
	glonass_gnav_string5_t fields = {
		.na=1461U, .n4=31U, .ln=1U,
		.tau_c=-INT32_MAX, .tau_gps=1048575
	};
	uint8_t string[GLONASS_GNAV_STRING_BITS];
	assert(gnss_glonass_gnav_string5(&fields,string) == 0);
	assert(glonass_field(string,9,1) == 1U);
	assert(glonass_field(string,10,21) == 1048575U);
	assert(glonass_field(string,31,1) == 0U);
	assert(glonass_field(string,32,5) == 31U);
	assert(glonass_field(string,38,31) == UINT32_C(0x7fffffff));
	assert(glonass_field(string,69,1) == 1U);
	assert(glonass_field(string,70,11) == 1461U);
	assert(glonass_field(string,81,4) == 5U);
	fields.n4=0U;
	assert(gnss_glonass_gnav_string5(&fields,string) == -1);
}

static void test_glonass_almanac_pair(void)
{
	glonass_gnav_almanac_t fields = {
		.slot=24U,.satellite_type=1U,.healthy=1U,.frequency=31U,.ln=0U,
		.tau=-511,.lambda=-1048575,.delta_i=131071,.delta_t=-2097151,
		.delta_t_rate=63,.omega=-32767,.eccentricity=32767U,
		.ascending_time=UINT32_C(1234567)
	};
	uint8_t even[85],odd[85];
	assert(gnss_glonass_gnav_almanac_pair(&fields,6U,even,odd)==0);
	assert(glonass_field(even,9,15)==32767U);
	assert(glonass_field(even,41,1)==0U);
	assert(glonass_field(even,42,20)==1048575U && glonass_field(even,62,1)==1U);
	assert(glonass_field(even,73,5)==24U && glonass_field(even,81,4)==6U);
	assert(glonass_field(odd,10,5)==31U);
	assert(glonass_field(odd,22,21)==2097151U && glonass_field(odd,43,1)==1U);
	assert(glonass_field(odd,44,21)==UINT32_C(1234567));
	assert(glonass_field(odd,81,4)==7U);
	assert(gnss_glonass_gnav_almanac_pair(&fields,7U,even,odd)==-1);
}

static void test_glonass_frame(void)
{
	glonass_gnav_immediate_t immediate={0};
	glonass_gnav_string5_t time_data={.na=1U,.n4=1U};
	glonass_gnav_almanac_t almanacs[5];
	uint8_t frame[15][85];
	unsigned int index;
	immediate.slot=1U; immediate.nt=1U;
	memset(almanacs,0,sizeof(almanacs));
	for(index=0U;index<5U;index++) almanacs[index].slot=(uint8_t)(index+1U);
	assert(gnss_glonass_gnav_frame(&immediate,&time_data,almanacs,frame)==0);
	for(index=0U;index<15U;index++)
		assert(glonass_field(frame[index],81,4)==index+1U);
	almanacs[4].slot=0U;
	assert(gnss_glonass_gnav_frame(&immediate,&time_data,almanacs,frame)==-1);
}

static void test_llh_motion(void)
{
	double storage[2][3];
	double *rows[2] = {storage[0], storage[1]};
	double llh[3];

	assert(readLlhMotion(rows, "tests/llh_motion.csv") == 2);
	xyz2llh(rows[0], llh);
	assert(fabs(llh[0]*R2D - 59.3293) < 1.0e-7);
	assert(fabs(llh[1]*R2D - 18.0686) < 1.0e-7);
	assert(fabs(llh[2] - 30.0) < 1.0e-3);
}

static void test_rinex4_mixed_navigation(void)
{
	gnss_nav_record_t records[4];
	uint8_t galileo_words[4][GALILEO_INAV_WORD_BITS];
	beidou_d1_ephemeris_t beidou_ephemeris;
	uint8_t beidou_sf2[BEIDOU_NAV_SUBFRAME_BITS];
	uint8_t beidou_sf3[BEIDOU_NAV_SUBFRAME_BITS];
	beidou_d1_clock_t beidou_clock;
	const int8_t zero_iono[4]={0,0,0,0};
	glonass_gnav_immediate_t glonass_immediate;
	uint8_t glonass_strings[4][GLONASS_GNAV_STRING_BITS];
	int8_t galileo_cycle[GALILEO_E1_CYCLE_SYMBOLS];
	int8_t beidou_d1_cycle[BEIDOU_D1_FRAME_SYMBOLS];
	int8_t beidou_d2_cycle[BEIDOU_D2_CYCLE_SYMBOLS];
	int8_t glonass_cycle[GLONASS_GNAV_FRAME_SYMBOLS];
	size_t count = 0;
	double position[3];
	double velocity[3];
	double clock_bias;
	double clock_drift;
	double radius;
	gnss_observation_t observation;
	double receiver[3],receiver_velocity[3]={0.0,0.0,0.0};
	double receiver_llh[3]={59.3293/R2D,18.0686/R2D,30.0};
	datetime_t observation_time;
	gpstime_t observation_gps;

	assert(gnss_read_rinex_nav("tests/rinex4_mixed.nav", records, 4, &count) == 0);
	assert(count == 3);
	assert(records[0].system == GNSS_SYSTEM_GALILEO && records[0].prn == 12);
	assert(strcmp(records[0].message, "INAV") == 0);
	assert(records[0].model == GNSS_NAV_KEPLERIAN && records[0].orbit_count == 24);
	assert(fabs(records[0].orbit[7] - 5440.609727859) < 1.0e-9);
	assert(records[0].orbit[19] == 3.12 && records[0].orbit[23] == 176434.0);
	assert(gnss_galileo_inav_ephemeris_words(&records[0], galileo_words) == 0);
	assert(unpack_bits(galileo_words[0],0,6) == 1U);
	assert(unpack_bits(galileo_words[1],0,6) == 2U);
	assert(unpack_bits(galileo_words[2],0,6) == 3U);
	assert(unpack_bits(galileo_words[3],0,6) == 4U);
	assert(unpack_bits(galileo_words[0],6,10) == 36U);
	assert(unpack_bits(galileo_words[0],16,14) == 2920U);
	assert(unpack_bits(galileo_words[2],120,8) == 107U);
	assert(unpack_bits(galileo_words[3],16,6) == 12U);
	assert(unpack_bits(galileo_words[3],54,14) == 2920U);
	assert(gnss_schedule_galileo_e1(&records[0],2300U,175200U,galileo_cycle)==0);
	assert(galileo_cycle[0]==1 || galileo_cycle[0]==-1);
	llh2xyz(receiver_llh,receiver);
	assert(gnss_observe(&records[0],records[0].orbit[8],receiver,receiver_velocity,
		1575.42e6,1.023e6,GALILEO_E1_CODE_LENGTH,&observation)==0);
	assert(observation.geometric_range_m>1.0e7 && observation.geometric_range_m<5.0e7);
	assert(observation.code_phase_chips>=0.0 &&
		observation.code_phase_chips<GALILEO_E1_CODE_LENGTH);
	assert(records[1].system == GNSS_SYSTEM_BEIDOU && records[1].prn == 20);
	assert(strcmp(records[1].message, "D1") == 0);
	assert(records[1].orbit_count == 26U);
	assert(isnan(records[1].orbit[17]) && records[1].orbit[18] == 809.0 &&
		isnan(records[1].orbit[19]));
	assert(gnss_beidou_d1_ephemeris_from_rinex(&records[1],&beidou_ephemeris) == 0);
	assert(beidou_ephemeris.toe == 11700U);
	assert(gnss_beidou_d1_ephemeris_subframes(&beidou_ephemeris,93600U,
		beidou_sf2,beidou_sf3) == 0);
	assert(gnss_beidou_d1_clock_from_rinex(&records[1],zero_iono,zero_iono,
		&beidou_clock)==0);
	assert(beidou_clock.aode==1U && beidou_clock.aodc==1U);
	assert(beidou_clock.week==809U && beidou_clock.toc==11700U);
	assert(beidou_clock.health==0U && beidou_clock.urai==0U);
	assert(beidou_clock.tgd1==230 && beidou_clock.tgd2==230);
	assert(gnss_schedule_beidou_d1(&records[1],zero_iono,zero_iono,93600U,
		beidou_d1_cycle)==0);
	assert(beidou_d1_cycle[0]==-1 && beidou_d1_cycle[1]==-1);
	assert(records[2].system == GNSS_SYSTEM_GLONASS && records[2].prn == 1);
	assert(strcmp(records[2].message, "FDMA") == 0);
	assert(records[2].model == GNSS_NAV_GLONASS_STATE_VECTOR && records[2].orbit_count == 16);
	assert(fabs(records[2].orbit[0] + 13904.48925781) < 1.0e-7);
	assert(records[2].orbit[14] == 2.0);
	assert(gnss_glonass_gnav_from_rinex(&records[2],&glonass_immediate) == 0);
	assert(glonass_immediate.tk_seconds == 10320U);
	assert(glonass_immediate.tb == 11U && glonass_immediate.nt == 259U);
	assert(glonass_immediate.slot == 1U && glonass_immediate.mode == 1U);
	assert(glonass_immediate.p2 == 1U && glonass_immediate.p == 3U);
	assert(glonass_immediate.ft == 2U && glonass_immediate.bn == 0U);
	assert(gnss_glonass_gnav_immediate_strings(&glonass_immediate,
		glonass_strings) == 0);
	assert(gnss_schedule_glonass(&records[2],glonass_cycle)==0);
	assert(glonass_cycle[0]==1 || glonass_cycle[0]==-1);
	observation_time.y=records[2].toc.year; observation_time.m=records[2].toc.month;
	observation_time.d=records[2].toc.day; observation_time.hh=records[2].toc.hour;
	observation_time.mm=records[2].toc.minute; observation_time.sec=records[2].toc.second;
	date2gps(&observation_time,&observation_gps);
	assert(gnss_observe(&records[2],observation_gps.sec,receiver,receiver_velocity,
		1602.0e6,0.511e6,GLONASS_L1OF_CODE_LENGTH,&observation)==0);
	assert(isfinite(observation.doppler_hz) && observation.carrier_phase_rad>=0.0 &&
		observation.carrier_phase_rad<2.0*PI);
	assert(gnss_propagate_kepler(&records[0], records[0].orbit[8], position,
		velocity, &clock_bias, &clock_drift) == 0);
	radius = sqrt(position[0]*position[0] + position[1]*position[1] + position[2]*position[2]);
	assert(radius > 2.0e7 && radius < 4.0e7);
	assert(isfinite(clock_bias) && isfinite(clock_drift));
	assert(gnss_propagate_kepler(&records[1], records[1].orbit[8] + 60.0, position,
		velocity, &clock_bias, &clock_drift) == 0);
	radius = sqrt(position[0]*position[0] + position[1]*position[1] + position[2]*position[2]);
	assert(radius > 2.0e7 && radius < 5.0e7);
	strcpy(records[1].message, "D2");
	assert(gnss_beidou_d1_ephemeris_from_rinex(&records[1],&beidou_ephemeris)==0);
	assert(gnss_beidou_d1_clock_from_rinex(&records[1],zero_iono,zero_iono,
		&beidou_clock)==0);
	assert(gnss_schedule_beidou_d2(&records[1],zero_iono,zero_iono,93600U,
		beidou_d2_cycle)==0);
	assert(beidou_d2_cycle[0]==-1 && beidou_d2_cycle[1]==-1);
	assert(gnss_propagate_kepler(&records[1], records[1].orbit[8] + 60.0, position,
		velocity, &clock_bias, &clock_drift) == 0);
	assert(isfinite(position[0]) && isfinite(position[1]) && isfinite(position[2]));
	assert(gnss_propagate_glonass(&records[2], 0.0, position, velocity,
		&clock_bias, &clock_drift) == 0);
	assert(fabs(position[0] + 13904489.25781) < 1.0e-5);
	assert(fabs(velocity[0] - 2552.483558655) < 1.0e-9);
	assert(gnss_propagate_glonass(&records[2], 60.0, position, velocity,
		&clock_bias, &clock_drift) == 0);
	radius = sqrt(position[0]*position[0] + position[1]*position[1] + position[2]*position[2]);
	assert(radius > 2.0e7 && radius < 3.0e7);
	assert(isfinite(clock_bias) && isfinite(clock_drift));
	assert(gnss_read_rinex_nav("tests/rinex4_mixed.nav", records, 2, &count) == -1);
}

#ifndef _WIN32
static void test_compressed_rinex_sample(void)
{
	ephem_t ephemerides[EPHEM_ARRAY_SIZE][MAX_SAT];
	int count;
	int valid = 0;
	int set;
	int sv;

	count = readRinexNavAll(ephemerides, "brdc2940.18n.Z");
	assert(count > 0);
	for (set = 0; set < count; set++)
		for (sv = 0; sv < MAX_SAT; sv++)
			valid += ephemerides[set][sv].vflg == 1;
	assert(valid > 0);
}
#endif

int main(void)
{
	test_time_conversions();
	test_coordinate_round_trip();
	test_ca_code_balance();
	test_ephemeris_selection();
	test_signal_profiles();
	test_rf_renderer_and_allocator();
	test_constellation_rf_sequences();
	test_multi_gnss_codes();
	test_multi_gnss_fec();
	test_galileo_inav_pages();
	test_galileo_inav_words();
	test_beidou_navigation_subframe();
	test_beidou_d1_ephemeris();
	test_beidou_d1_clock();
	test_beidou_almanac_pages();
	test_beidou_d2_basic_pages();
	test_glonass_immediate_strings();
	test_glonass_string5();
	test_glonass_almanac_pair();
	test_glonass_frame();
	test_llh_motion();
	test_rinex4_mixed_navigation();
#ifndef _WIN32
	test_compressed_rinex_sample();
#endif
	puts("core model tests passed");
	return 0;
}
