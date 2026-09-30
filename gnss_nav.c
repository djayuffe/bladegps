#include "gnss_nav.h"

#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define NAV_LINE_SIZE 256
#define RINEX_FIELD_WIDTH 19U

static int system_from_character(char value, gnss_system_t *system)
{
	if (system == NULL)
		return -1;
	switch (value) {
	case 'G': *system = GNSS_SYSTEM_GPS; return 0;
	case 'E': *system = GNSS_SYSTEM_GALILEO; return 0;
	case 'C': *system = GNSS_SYSTEM_BEIDOU; return 0;
	case 'R': *system = GNSS_SYSTEM_GLONASS; return 0;
	default: return -1;
	}
}

static int parse_field(const char *line, size_t offset, double *value)
{
	char field[RINEX_FIELD_WIDTH + 1U];
	char *end;
	size_t length;
	size_t index;
	int has_digit = 0;

	if (line == NULL || value == NULL)
		return -1;
	length = strlen(line);
	memset(field, ' ', RINEX_FIELD_WIDTH);
	if (offset < length) {
		size_t available = length - offset;
		if (available > RINEX_FIELD_WIDTH)
			available = RINEX_FIELD_WIDTH;
		memcpy(field, line + offset, available);
	}
	field[RINEX_FIELD_WIDTH] = '\0';
	for (index = 0; index < RINEX_FIELD_WIDTH; index++) {
		if (field[index] == 'D' || field[index] == 'd')
			field[index] = 'E';
		if (isdigit((unsigned char)field[index]))
			has_digit = 1;
	}
	if (!has_digit) {
		*value = NAN;
		return 0;
	}
	*value = strtod(field, &end);
	if (end == field || !isfinite(*value))
		return -1;
	while (*end != '\0') {
		if (!isspace((unsigned char)*end))
			return -1;
		end++;
	}
	return 0;
}

static int parse_epoch_line(const char *line, gnss_nav_record_t *record)
{
	char system;
	unsigned int prn;
	int consumed;

	if (sscanf(line, "%c%2u %d %d %d %d %d %lf%n", &system, &prn,
		&record->toc.year, &record->toc.month, &record->toc.day,
		&record->toc.hour, &record->toc.minute, &record->toc.second,
		&consumed) != 8 || system_from_character(system, &record->system) != 0 ||
		prn == 0U || consumed > 24)
		return -1;
	record->prn = prn;
	if (parse_field(line, 23U, &record->clock_bias) != 0 ||
		parse_field(line, 42U, &record->clock_drift) != 0 ||
		parse_field(line, 61U, &record->clock_drift_rate) != 0)
		return -1;
	return 0;
}

static int parse_orbit_line(const char *line, gnss_nav_record_t *record,
	size_t fields)
{
	size_t field;

	if (record->orbit_count + fields > GNSS_NAV_ORBIT_FIELDS)
		return -1;
	for (field = 0; field < fields; field++) {
		if (parse_field(line, 4U + field * RINEX_FIELD_WIDTH,
			&record->orbit[record->orbit_count]) != 0)
			return -1;
		record->orbit_count++;
	}
	return 0;
}

static int valid_calendar(const gnss_calendar_time_t *time)
{
	return time->year >= 1980 && time->month >= 1 && time->month <= 12 &&
		time->day >= 1 && time->day <= 31 && time->hour >= 0 && time->hour <= 23 &&
		time->minute >= 0 && time->minute <= 59 && isfinite(time->second) &&
		time->second >= 0.0 && time->second < 61.0;
}

static int supported_ephemeris(char system, const char *message)
{
	if (message == NULL)
		return 0;
	if (system == 'G')
		return strcmp(message, "LNAV") == 0;
	if (system == 'E')
		return strcmp(message, "INAV") == 0 || strcmp(message, "FNAV") == 0;
	if (system == 'C')
		return strcmp(message, "D1") == 0 || strcmp(message, "D2") == 0;
	if (system == 'R')
		return strcmp(message, "FDMA") == 0;
	return 0;
}

static int skip_rinex4_record(FILE *stream)
{
	char line[NAV_LINE_SIZE];
	long position;

	while ((position = ftell(stream)) >= 0 && fgets(line, sizeof(line), stream) != NULL) {
		if (line[0] == '>')
			return fseek(stream, position, SEEK_SET) == 0 ? 0 : -1;
	}
	return ferror(stream) ? -1 : 1;
}

int gnss_read_rinex_nav(const char *path, gnss_nav_record_t *records,
	size_t capacity, size_t *record_count)
{
	FILE *stream;
	char line[NAV_LINE_SIZE];
	double version = 0.0;
	int header_done = 0;
	size_t count = 0;

	if (path == NULL || records == NULL || capacity == 0U || record_count == NULL)
		return -1;
	*record_count = 0U;
	stream = fopen(path, "r");
	if (stream == NULL)
		return -1;
	while (fgets(line, sizeof(line), stream) != NULL) {
		if (version == 0.0)
			version = strtod(line, NULL);
		if (strstr(line, "END OF HEADER") != NULL) {
			header_done = 1;
			break;
		}
	}
	if (!header_done || version < 3.0 || version >= 5.0)
		goto fail;

	while (fgets(line, sizeof(line), stream) != NULL) {
		gnss_nav_record_t record;
		char epoch_line[NAV_LINE_SIZE];
		size_t continuation_lines;
		size_t line_index;
		int rinex4 = line[0] == '>';

		if (line[0] == '\n' || line[0] == '\r')
			continue;
		memset(&record, 0, sizeof(record));
		strcpy(record.message, "LEGACY");
		if (rinex4) {
			char type[4] = {0};
			char satellite[4] = {0};
			char message[GNSS_NAV_MESSAGE_NAME_SIZE] = {0};
			if (sscanf(line, "> %3s %3s %7s", type, satellite, message) != 3)
				goto fail;
			if (strcmp(type, "EPH") != 0 || !supported_ephemeris(satellite[0], message)) {
				int skip_status = skip_rinex4_record(stream);
				if (skip_status < 0)
					goto fail;
				if (skip_status > 0)
					break;
				continue;
			}
			if (system_from_character(satellite[0], &record.system) != 0)
				goto fail;
			if (strlen(message) >= sizeof(record.message))
				goto fail;
			strcpy(record.message, message);
			if (fgets(epoch_line, sizeof(epoch_line), stream) == NULL)
				goto fail;
		} else {
			strcpy(epoch_line, line);
		}
		if (parse_epoch_line(epoch_line, &record) != 0 || !valid_calendar(&record.toc))
			goto fail;
		record.model = record.system == GNSS_SYSTEM_GLONASS ?
			GNSS_NAV_GLONASS_STATE_VECTOR : GNSS_NAV_KEPLERIAN;
		continuation_lines = record.system == GNSS_SYSTEM_GLONASS ?
			(rinex4 ? 4U : 3U) : 7U;
		for (line_index = 0; line_index < continuation_lines; line_index++) {
			size_t field_count = 4U;
			/* RINEX 3/4 Galileo A13 defines three fields on Broadcast
			 * Orbit-5 and one on Orbit-7.  Treating the unused print columns
			 * as data shifts SISA, health and BGD semantics by one slot. */
			if (record.system == GNSS_SYSTEM_GALILEO) {
				if (line_index == 4U)
					field_count = 3U;
				else if (line_index == 6U)
					field_count = 1U;
			}
			if (fgets(line, sizeof(line), stream) == NULL ||
				parse_orbit_line(line, &record, field_count) != 0)
				goto fail;
		}
		if (count >= capacity)
			goto fail;
		records[count++] = record;
	}
	fclose(stream);
	*record_count = count;
	return count > 0U ? 0 : -1;

fail:
	fclose(stream);
	return -1;
}
