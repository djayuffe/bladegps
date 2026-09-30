#ifndef BLADEGPS_GNSS_CODES_H
#define BLADEGPS_GNSS_CODES_H

#include <stddef.h>
#include <stdint.h>

#define BEIDOU_B1I_CODE_LENGTH 2046U
#define GLONASS_L1OF_CODE_LENGTH 511U
#define GALILEO_E1_CODE_LENGTH 4092U
#define GALILEO_E1_CBOC_SUBCHIPS 12U
#define GALILEO_E1C_SECONDARY_LENGTH 25U
#define BEIDOU_B1I_NH_LENGTH 20U
#define GLONASS_TIME_MARK_LENGTH 30U

typedef enum {
	GALILEO_E1_COMPONENT_B = 0,
	GALILEO_E1_COMPONENT_C = 1
} galileo_e1_component_t;

int gnss_beidou_b1i_code(unsigned int prn,
	int8_t chips[BEIDOU_B1I_CODE_LENGTH]);
int gnss_glonass_l1of_code(int8_t chips[GLONASS_L1OF_CODE_LENGTH]);
int gnss_glonass_l1of_carrier_hz(int frequency_slot, double *carrier_hz);
int gnss_decode_hex_code(const char *hex, size_t chip_count, int8_t *chips);
int gnss_galileo_e1_primary_code(unsigned int prn,
	galileo_e1_component_t component, int8_t chips[GALILEO_E1_CODE_LENGTH]);
int gnss_galileo_e1c_secondary_code(
	int8_t chips[GALILEO_E1C_SECONDARY_LENGTH]);
int gnss_beidou_b1i_nh_code(int8_t chips[BEIDOU_B1I_NH_LENGTH]);
int gnss_glonass_time_mark(int8_t chips[GLONASS_TIME_MARK_LENGTH]);
void gnss_galileo_e1_cboc(double data[GALILEO_E1_CBOC_SUBCHIPS],
	double pilot[GALILEO_E1_CBOC_SUBCHIPS]);

#endif
