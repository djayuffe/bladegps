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
		1,0,0,0,1,0,0,0,1,1,0,0,1,0,0,0,0,0,1,0
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
	size_t count = 0;
	double position[3];
	double velocity[3];
	double clock_bias;
	double clock_drift;
	double radius;

	assert(gnss_read_rinex_nav("tests/rinex4_mixed.nav", records, 4, &count) == 0);
	assert(count == 3);
	assert(records[0].system == GNSS_SYSTEM_GALILEO && records[0].prn == 12);
	assert(strcmp(records[0].message, "INAV") == 0);
	assert(records[0].model == GNSS_NAV_KEPLERIAN && records[0].orbit_count == 28);
	assert(fabs(records[0].orbit[7] - 5440.609727859) < 1.0e-9);
	assert(isnan(records[0].orbit[25]) && isnan(records[0].orbit[26]) && isnan(records[0].orbit[27]));
	assert(records[1].system == GNSS_SYSTEM_BEIDOU && records[1].prn == 20);
	assert(strcmp(records[1].message, "D1") == 0);
	assert(isnan(records[1].orbit[17]) && records[1].orbit[18] == 809.0);
	assert(records[2].system == GNSS_SYSTEM_GLONASS && records[2].prn == 1);
	assert(strcmp(records[2].message, "FDMA") == 0);
	assert(records[2].model == GNSS_NAV_GLONASS_STATE_VECTOR && records[2].orbit_count == 16);
	assert(fabs(records[2].orbit[0] + 13904.48925781) < 1.0e-7);
	assert(records[2].orbit[14] == 2.0);
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
	test_multi_gnss_codes();
	test_multi_gnss_fec();
	test_llh_motion();
	test_rinex4_mixed_navigation();
#ifndef _WIN32
	test_compressed_rinex_sample();
#endif
	puts("core model tests passed");
	return 0;
}
