#include "gnss_nav.h"

#include <ctype.h>
#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifndef _WIN32
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>
#endif

#define NAV_LINE_SIZE 256
#define RINEX_FIELD_WIDTH 19U

typedef struct {
	FILE *file;
#ifndef _WIN32
	pid_t decompressor;
#endif
} nav_stream_t;

static int has_suffix(const char *value, const char *suffix)
{
	size_t value_length,suffix_length;
	if(value==NULL||suffix==NULL)return 0;
	value_length=strlen(value);suffix_length=strlen(suffix);
	return value_length>=suffix_length&&
		strcmp(value+value_length-suffix_length,suffix)==0;
}

static int nav_stream_open(const char *path,nav_stream_t *source)
{
	if(path==NULL||source==NULL)return -1;
	memset(source,0,sizeof(*source));
#ifdef _WIN32
	if(has_suffix(path,".gz")||has_suffix(path,".Z"))return -1;
	source->file=fopen(path,"r");
#else
	source->decompressor=-1;
	if(has_suffix(path,".gz")||has_suffix(path,".Z")) {
		int descriptors[2];pid_t child;
		if(pipe(descriptors)!=0)return -1;
		child=fork();
		if(child<0){close(descriptors[0]);close(descriptors[1]);return -1;}
		if(child==0){close(descriptors[0]);
			if(dup2(descriptors[1],STDOUT_FILENO)<0)_exit(127);
			close(descriptors[1]);execlp("gzip","gzip","-cd","--",path,(char *)NULL);_exit(127);}
		close(descriptors[1]);source->file=fdopen(descriptors[0],"r");
		if(source->file==NULL){close(descriptors[0]);waitpid(child,NULL,0);return -1;}
		source->decompressor=child;
	} else source->file=fopen(path,"r");
#endif
	return source->file==NULL?-1:0;
}

static int nav_stream_close(nav_stream_t *source)
{
	int result,child_status=0;
	if(source==NULL||source->file==NULL)return -1;
	result=fclose(source->file);source->file=NULL;
#ifndef _WIN32
	if(source->decompressor>0) {
		while(waitpid(source->decompressor,&child_status,0)<0)
			if(errno!=EINTR)return -1;
		if(!WIFEXITED(child_status)||WEXITSTATUS(child_status)!=0)return -1;
	}
#else
	(void)child_status;
#endif
	return result==0?0:-1;
}

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

static int parse_rinex2_gps_epoch(const char *line, gnss_nav_record_t *record)
{
	unsigned int prn;
	int year, consumed;

	if (line == NULL || record == NULL ||
		sscanf(line, "%2u %2d %d %d %d %d %lf%n", &prn, &year,
			&record->toc.month, &record->toc.day, &record->toc.hour,
			&record->toc.minute, &record->toc.second, &consumed) != 7 ||
		prn == 0U || consumed > 22)
		return -1;
	record->system = GNSS_SYSTEM_GPS;
	record->prn = prn;
	record->toc.year = year >= 80 ? 1900 + year : 2000 + year;
	if (parse_field(line, 22U, &record->clock_bias) != 0 ||
		parse_field(line, 41U, &record->clock_drift) != 0 ||
		parse_field(line, 60U, &record->clock_drift_rate) != 0)
		return -1;
	return 0;
}

static int parse_orbit_line(const char *line, gnss_nav_record_t *record,
	size_t fields, size_t first_field)
{
	size_t field;

	if (record->orbit_count + fields > GNSS_NAV_ORBIT_FIELDS)
		return -1;
	for (field = 0; field < fields; field++) {
		if (parse_field(line, first_field + field * RINEX_FIELD_WIDTH,
			&record->orbit[record->orbit_count]) != 0)
			return -1;
		record->orbit_count++;
	}
	return 0;
}

static int valid_calendar(const gnss_calendar_time_t *time)
{
	static const int month_days[12]={31,28,31,30,31,30,31,31,30,31,30,31};
	int days;
	if(time->year<1980||time->month<1||time->month>12)return 0;
	days=month_days[time->month-1];
	if(time->month==2&&time->year%4==0&&
		(time->year%100!=0||time->year%400==0))days++;
	return time->day >= 1 && time->day <= days && time->hour >= 0 && time->hour <= 23 &&
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

static int infer_rinex3_message(gnss_nav_record_t *record)
{
	long sources;
	if(record==NULL)return -1;
	switch(record->system) {
	case GNSS_SYSTEM_GPS:
		strcpy(record->message,"LNAV");return 0;
	case GNSS_SYSTEM_GALILEO:
		if(record->orbit_count<19U||!isfinite(record->orbit[17]))return -1;
		sources=lround(record->orbit[17]);
		if(fabs(record->orbit[17]-(double)sources)>1.0e-6||sources<0||sources>1023)
			return -1;
		if((sources&5L)!=0L)strcpy(record->message,"INAV");
		else if((sources&2L)!=0L)strcpy(record->message,"FNAV");
		else return -1;
		return 0;
	case GNSS_SYSTEM_BEIDOU:
		strcpy(record->message,record->prn<=5U?"D2":"D1");return 0;
	case GNSS_SYSTEM_GLONASS:
		strcpy(record->message,"FDMA");return 0;
	default:return -1;
	}
}

static int skip_rinex4_record(FILE *stream, char next_record[NAV_LINE_SIZE])
{
	char line[NAV_LINE_SIZE];

	if (stream == NULL || next_record == NULL)
		return -1;
	while (fgets(line, sizeof(line), stream) != NULL) {
		if (line[0] == '>') {
			strcpy(next_record, line);
			return 0;
		}
	}
	return ferror(stream) ? -1 : 1;
}

int gnss_read_rinex_nav(const char *path, gnss_nav_record_t *records,
	size_t capacity, size_t *record_count)
{
	FILE *stream;
	nav_stream_t source;
	char line[NAV_LINE_SIZE];
	char pending_line[NAV_LINE_SIZE];
	double version = 0.0;
	int header_done = 0;
	int have_pending_line = 0;
	size_t count = 0;

	if (path == NULL || record_count == NULL ||
		((records == NULL) != (capacity == 0U)))
		return -1;
	*record_count = 0U;
	if (nav_stream_open(path,&source)!=0)
		return -1;
	stream=source.file;
	while (fgets(line, sizeof(line), stream) != NULL) {
		if (version == 0.0)
			version = strtod(line, NULL);
		if (strstr(line, "END OF HEADER") != NULL) {
			header_done = 1;
			break;
		}
	}
	if (!header_done || version < 2.0 || version >= 5.0)
		goto fail;

	for (;;) {
		gnss_nav_record_t record;
		char epoch_line[NAV_LINE_SIZE];
		size_t continuation_lines;
		size_t line_index;
		int rinex2;
		int rinex4;

		if (have_pending_line) {
			strcpy(line, pending_line);
			have_pending_line = 0;
		} else if (fgets(line, sizeof(line), stream) == NULL) {
			break;
		}
		rinex2 = version < 3.0;
		rinex4 = line[0] == '>';

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
				int skip_status = skip_rinex4_record(stream, pending_line);
				if (skip_status < 0)
					goto fail;
				if (skip_status > 0)
					break;
				have_pending_line = 1;
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
		if ((rinex2 ? parse_rinex2_gps_epoch(epoch_line, &record) :
			parse_epoch_line(epoch_line, &record)) != 0 || !valid_calendar(&record.toc))
			goto fail;
		if (rinex2)
			strcpy(record.message, "LNAV");
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
			} else if (record.system == GNSS_SYSTEM_BEIDOU && line_index == 6U) {
				/* RINEX A23 Orbit-7 contains only transmission time and AODC. */
				field_count = 2U;
			}
			if (fgets(line, sizeof(line), stream) == NULL ||
				parse_orbit_line(line, &record, field_count, rinex2 ? 3U : 4U) != 0)
				goto fail;
		}
		if(!rinex2&&!rinex4&&infer_rinex3_message(&record)!=0)
			goto fail;
		if (count == SIZE_MAX || (records != NULL && count >= capacity))
			goto fail;
		if (records != NULL)
			records[count] = record;
		count++;
	}
	if(nav_stream_close(&source)!=0)return -1;
	*record_count = count;
	return count > 0U ? 0 : -1;

fail:
	(void)nav_stream_close(&source);
	return -1;
}

int gnss_load_rinex_nav(const char *path, gnss_nav_record_t **records,
	size_t *record_count)
{
	gnss_nav_record_t *loaded;
	size_t count;

	if (path == NULL || records == NULL || record_count == NULL)
		return -1;
	*records = NULL;
	*record_count = 0U;
	if (gnss_read_rinex_nav(path, NULL, 0U, &count) != 0 ||
		count > SIZE_MAX / sizeof(*loaded))
		return -1;
	loaded = malloc(count * sizeof(*loaded));
	if (loaded == NULL)
		return -1;
	if (gnss_read_rinex_nav(path, loaded, count, record_count) != 0 ||
		*record_count != count) {
		free(loaded);
		*record_count = 0U;
		return -1;
	}
	*records = loaded;
	return 0;
}

static int parse_four_values(const char *text, double values[4])
{
	char copy[NAV_LINE_SIZE];
	char *cursor,*end;
	unsigned int index;
	if(text==NULL||values==NULL||strlen(text)>=sizeof(copy))return -1;
	strcpy(copy,text);
	for(cursor=copy;*cursor!='\0';cursor++)if(*cursor=='D'||*cursor=='d')*cursor='E';
	cursor=copy;
	for(index=0U;index<4U;index++) {
		while(isspace((unsigned char)*cursor))cursor++;
		values[index]=strtod(cursor,&end);
		if(end==cursor||!isfinite(values[index]))return -1;
		cursor=end;
	}
	return 0;
}

int gnss_read_beidou_ionosphere(const char *path, gnss_klobuchar_t *model)
{
	FILE *stream;
	nav_stream_t source;
	char line[NAV_LINE_SIZE];
	double alpha[4],beta[4];
	int have_alpha=0,have_beta=0,found=0;
	if(path==NULL||model==NULL)return -1;
	if(nav_stream_open(path,&source)!=0)return -1;
	stream=source.file;
	while(fgets(line,sizeof(line),stream)!=NULL) {
		if(strncmp(line,"BDSA",4U)==0) {
			if(parse_four_values(line+4U,alpha)!=0)goto malformed;
			have_alpha=1;
		} else if(strncmp(line,"BDSB",4U)==0) {
			if(parse_four_values(line+4U,beta)!=0)goto malformed;
			have_beta=1;
		} else if(line[0]=='>') {
			char type[4]={0},satellite[4]={0},message[8]={0};
			if(sscanf(line,"> %3s %3s %7s",type,satellite,message)==3&&
				strcmp(type,"ION")==0&&satellite[0]=='C'&&
				(strcmp(message,"D1D2")==0||strcmp(message,"D1")==0||
				 strcmp(message,"D2")==0)) {
				char second[NAV_LINE_SIZE],third[NAV_LINE_SIZE];
				if(fgets(line,sizeof(line),stream)==NULL||
					fgets(second,sizeof(second),stream)==NULL||
					fgets(third,sizeof(third),stream)==NULL||
					parse_field(line,23U,&alpha[0])!=0||
					parse_field(line,42U,&alpha[1])!=0||
					parse_field(line,61U,&alpha[2])!=0||
					parse_field(second,4U,&alpha[3])!=0||
					parse_field(second,23U,&beta[0])!=0||
					parse_field(second,42U,&beta[1])!=0||
					parse_field(second,61U,&beta[2])!=0||
					parse_field(third,4U,&beta[3])!=0)goto malformed;
				have_alpha=have_beta=1;
			}
		}
		if(have_alpha&&have_beta) {
			memcpy(model->alpha,alpha,sizeof(alpha));
			memcpy(model->beta,beta,sizeof(beta));
			found=1;have_alpha=have_beta=0;
		}
	}
	if(ferror(stream))goto malformed;
	if(nav_stream_close(&source)!=0)return -1;
	return found?0:1;
malformed:
	(void)nav_stream_close(&source);return -1;
}
