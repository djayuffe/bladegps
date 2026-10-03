#define _CRT_SECURE_NO_WARNINGS

#include "bladegps.h"
#include <errno.h>
#include <limits.h>
#include <math.h>
#include <signal.h>
#include <time.h>
#include <sys/stat.h>

// for _getch used in Windows runtime.
#ifdef _WIN32
#include <conio.h>
#include "getopt.h"
#include <windows.h>
#else
#include <unistd.h>
#endif

static volatile sig_atomic_t stop_requested = 0;

static void request_stop(int signal_number)
{
	(void)signal_number;
	stop_requested = 1;
}

int stop_was_requested(void)
{
	return stop_requested != 0;
}

static int copy_option(char *dst, size_t dst_size, const char *src, const char *name)
{
	if (src == NULL || strlen(src) >= dst_size) {
		fprintf(stderr, "ERROR: %s is too long. Maximum length is %zu characters.\n", name, dst_size - 1);
		return -1;
	}

	strcpy(dst, src);
	return 0;
}

static int parse_location(const char *arg, double llh[3])
{
	double lat, lon, hgt;
	char extra;

	if (sscanf(arg, "%lf,%lf,%lf%c", &lat, &lon, &hgt, &extra) != 3)
		return -1;

	if (!isfinite(lat) || !isfinite(lon) || !isfinite(hgt) ||
		lat < -90.0 || lat > 90.0 || lon < -180.0 || lon > 180.0)
		return -1;

	llh[0] = lat / R2D;
	llh[1] = lon / R2D;
	llh[2] = hgt;
	return 0;
}

static int parse_duration(const char *arg, int *iduration)
{
	char *end = NULL;
	double duration;

	errno = 0;
	duration = strtod(arg, &end);
	if (errno != 0 || end == arg || *end != '\0' || !isfinite(duration) ||
		duration <= 0.0 || duration > ((double)USER_MOTION_SIZE) / 10.0)
		return -1;

	*iduration = (int)(duration * 10.0 + 0.5);
	if (*iduration < 1)
		return -1;
	return 0;
}

static int parse_xb_board(const char *arg, int *xb_board)
{
	char *end = NULL;
	long value;

	errno = 0;
	value = strtol(arg, &end, 10);
	if (errno != 0 || end == arg || *end != '\0' || (value != 0 && value != 200))
		return -1;

	*xb_board = (int)value;
	return 0;
}

static int parse_uint_option(const char *arg, unsigned int minimum,
	unsigned int maximum, unsigned int *value)
{
	char *end = NULL;
	unsigned long parsed;

	errno = 0;
	parsed = strtoul(arg, &end, 10);
	if (errno != 0 || end == arg || *end != '\0' || parsed < minimum || parsed > maximum)
		return -1;
	*value = (unsigned int)parsed;
	return 0;
}

static int parse_int_option(const char *arg, int minimum, int maximum, int *value)
{
	char *end = NULL;
	long parsed;

	errno = 0;
	parsed = strtol(arg, &end, 10);
	if (errno != 0 || end == arg || *end != '\0' || parsed < minimum || parsed > maximum)
		return -1;
	*value = (int)parsed;
	return 0;
}

static int parse_elevation_mask(const char *arg, double *value)
{
	char *end = NULL;
	double parsed;

	errno = 0;
	parsed = strtod(arg, &end);
	if (errno != 0 || end == arg || *end != '\0' || !isfinite(parsed) || parsed < -90.0 || parsed > 90.0)
		return -1;
	*value = parsed;
	return 0;
}

static int file_exists(const char *path)
{
	struct stat st;

	return path != NULL && stat(path, &st) == 0 && S_ISREG(st.st_mode);
}

static int run_command(const char *cmd, const char *description)
{
	int status;

	status = system(cmd);
	if (status != 0) {
		fprintf(stderr, "ERROR: Failed to %s.\n", description);
		return -1;
	}

	return 0;
}

static int append_dependency_check(char *cmd, size_t cmd_size, const char *tool)
{
#ifdef _WIN32
	return snprintf(cmd, cmd_size, "where %s >nul 2>nul", tool) < (int)cmd_size ? 0 : -1;
#else
	return snprintf(cmd, cmd_size, "command -v %s >/dev/null 2>&1", tool) < (int)cmd_size ? 0 : -1;
#endif
}

static int is_leap_year(int year)
{
	return (year % 4 == 0 && year % 100 != 0) || year % 400 == 0;
}

static int day_of_year(const datetime_t *date)
{
	static const int month_days[] = { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };
	int doy = 0;
	int month;

	if (date->m < 1 || date->m > 12 || date->d < 1)
		return -1;

	for (month = 1; month < date->m; month++)
		doy += month_days[month - 1] + (month == 2 && is_leap_year(date->y));

	if (date->d > month_days[date->m - 1] + (date->m == 2 && is_leap_year(date->y)))
		return -1;

	return doy + date->d;
}

static int previous_utc_day(datetime_t *date)
{
	static const int month_days[]={31,28,31,30,31,30,31,31,30,31,30,31};
	int days;
	if(date==NULL||day_of_year(date)<1)return -1;
	date->d--;
	if(date->d>0)return 0;
	date->m--;
	if(date->m==0){date->m=12;date->y--;}
	days=month_days[date->m-1]+(date->m==2&&is_leap_year(date->y));
	date->d=days;return 0;
}

static int utc_today(datetime_t *date)
{
	time_t now;
	struct tm tm_utc;
#ifdef _WIN32
	struct tm *result;
#endif

	now = time(NULL);
	if (now == (time_t)-1)
		return -1;

#ifdef _WIN32
	result = gmtime(&now);
	if (result == NULL)
		return -1;
	tm_utc = *result;
#else
	if (gmtime_r(&now, &tm_utc) == NULL)
		return -1;
#endif

	date->y = tm_utc.tm_year + 1900;
	date->m = tm_utc.tm_mon + 1;
	date->d = tm_utc.tm_mday;
	date->hh = 0;
	date->mm = 0;
	date->sec = 0.0;
	return 0;
}

static int wallclock_seconds(double *seconds)
{
	if(seconds==NULL)return -1;
#ifdef _WIN32
	{
		FILETIME file_time;
		ULARGE_INTEGER ticks;
		GetSystemTimeAsFileTime(&file_time);
		ticks.LowPart=file_time.dwLowDateTime;ticks.HighPart=file_time.dwHighDateTime;
		if(ticks.QuadPart<UINT64_C(116444736000000000))return -1;
		*seconds=(double)(ticks.QuadPart-UINT64_C(116444736000000000))/1.0e7;
	}
#else
	{
		struct timespec now;
		if(clock_gettime(CLOCK_REALTIME,&now)!=0)return -1;
		*seconds=(double)now.tv_sec+(double)now.tv_nsec/1.0e9;
	}
#endif
	return isfinite(*seconds)?0:-1;
}

static int utc_now_gps(gpstime_t *gps,double *unix_seconds)
{
	double now;
	time_t whole;
	struct tm utc_tm;
	gnss_calendar_time_t utc;
#ifdef _WIN32
	struct tm *result;
#endif
	if(gps==NULL||wallclock_seconds(&now)!=0)return -1;
	whole=(time_t)floor(now);
#ifdef _WIN32
	result=gmtime(&whole);if(result==NULL)return -1;utc_tm=*result;
#else
	if(gmtime_r(&whole,&utc_tm)==NULL)return -1;
#endif
	utc=(gnss_calendar_time_t){utc_tm.tm_year+1900,utc_tm.tm_mon+1,utc_tm.tm_mday,
		utc_tm.tm_hour,utc_tm.tm_min,(double)utc_tm.tm_sec+(now-floor(now))};
	if(gnss_calendar_to_gps(GNSS_SYSTEM_GLONASS,&utc,gps)!=0)return -1;
	if(unix_seconds!=NULL)*unix_seconds=now;
	return 0;
}

static int downloaded_record_healthy(const gnss_nav_record_t *record)
{
	double value;
	long encoded;
	if(record->system==GNSS_SYSTEM_GPS)value=record->orbit[21];
	else if(record->system==GNSS_SYSTEM_GALILEO)value=record->orbit[20];
	else if(record->system==GNSS_SYSTEM_BEIDOU)value=record->orbit[21];
	else value=record->orbit[3];
	if(!isfinite(value))return 0;encoded=lround(value);
	if(fabs(value-(double)encoded)>1.0e-6||encoded<0)return 0;
	if(record->system==GNSS_SYSTEM_GALILEO)return encoded<=511L&&(encoded&7L)==0L;
	return encoded==0L;
}

static int downloaded_nav_supports_signal(const char *path,gnss_signal_t signal,
	const gpstime_t *required_time,int *legacy_glonass_only)
{
	gnss_nav_record_t *records=NULL;
	size_t count=0U,index;
	unsigned int found=0U;
	int saw_glonass=0;
	if(legacy_glonass_only!=NULL)*legacy_glonass_only=0;
	if(gnss_load_rinex_nav(path,&records,&count)!=0)return 0;
	for(index=0U;index<count;index++) {
		const gnss_nav_record_t *record=&records[index];
		gpstime_t epoch;
		double age=0.0;
		if(!downloaded_record_healthy(record))continue;
		if(required_time!=NULL) {
			if(gnss_calendar_to_gps(record->system,&record->toc,&epoch)!=0)continue;
			age=gnss_time_difference(required_time,&epoch);
			if(age< -30.0||age>(record->system==GNSS_SYSTEM_GLONASS?1800.0:14400.0))
				continue;
		}
		if(record->system==GNSS_SYSTEM_GPS&&strcmp(record->message,"LNAV")==0)
			found|=1U<<GNSS_SYSTEM_GPS;
		else if(record->system==GNSS_SYSTEM_GALILEO&&strcmp(record->message,"INAV")==0)
			found|=1U<<GNSS_SYSTEM_GALILEO;
		else if(record->system==GNSS_SYSTEM_BEIDOU&&
			(strcmp(record->message,"D1")==0||strcmp(record->message,"D2")==0))
			found|=1U<<GNSS_SYSTEM_BEIDOU;
		else if(record->system==GNSS_SYSTEM_GLONASS&&strcmp(record->message,"FDMA")==0) {
			glonass_gnav_immediate_t immediate;
			saw_glonass=1;
			if(gnss_glonass_gnav_from_rinex(record,&immediate)==0)
				found|=1U<<GNSS_SYSTEM_GLONASS;
		}
	}
	free(records);
	if(legacy_glonass_only!=NULL&&saw_glonass&&
		(found&(1U<<GNSS_SYSTEM_GLONASS))==0U)*legacy_glonass_only=1;
	if(signal==GNSS_SIGNAL_MIXED_OPEN)
		return (found&((1U<<GNSS_SYSTEM_COUNT)-1U))==
			((1U<<GNSS_SYSTEM_COUNT)-1U);
	if(signal==GNSS_SIGNAL_GPS_L1CA)return (found&(1U<<GNSS_SYSTEM_GPS))!=0U;
	if(signal==GNSS_SIGNAL_GALILEO_E1)return (found&(1U<<GNSS_SYSTEM_GALILEO))!=0U;
	if(signal==GNSS_SIGNAL_BEIDOU_B1I)return (found&(1U<<GNSS_SYSTEM_BEIDOU))!=0U;
	return (found&(1U<<GNSS_SYSTEM_GLONASS))!=0U;
}

static int download_broadcast_ephemeris(const datetime_t *date,
	gnss_signal_t signal,const gpstime_t *required_time,char *navfile,size_t navfile_size)
{
	int doy;
	int yy;
	char out_path[MAX_CHAR];
	char gz_path[MAX_CHAR];
	char tmp_gz_path[MAX_CHAR + 8];
	char tmp_out_path[MAX_CHAR + 8];
	char urls[4][256];
	char cmd[768];
	size_t source,source_count;
	int downloaded = 0;

	doy = day_of_year(date);
	if (doy < 1)
		return -1;

	yy = date->y % 100;
	if (signal == GNSS_SIGNAL_GPS_L1CA) {
		if (snprintf(out_path, sizeof(out_path), "brdc%03d0.%02dn", doy, yy) >= (int)sizeof(out_path))
			return -1;
	} else if (snprintf(out_path, sizeof(out_path),
		"BRDC00IGS_R_%04d%03d0000_01D_MN.rnx", date->y, doy) >= (int)sizeof(out_path))
		return -1;
	if (snprintf(gz_path, sizeof(gz_path), "%s.gz", out_path) >= (int)sizeof(gz_path))
		return -1;

	if (file_exists(out_path)) {
		int legacy_glonass_only=0;
		if(downloaded_nav_supports_signal(out_path,signal,required_time,&legacy_glonass_only)) {
			printf("Using validated broadcast ephemeris: %s\n", out_path);
			return copy_option(navfile, navfile_size, out_path, "downloaded ephemeris path");
		}
		fprintf(stderr,"WARNING: Cached ephemeris %s is not usable for the selected profile%s; refreshing it.\n",
			out_path,legacy_glonass_only?" (legacy GLONASS record lacks complete GNAV fields)":"");
	}
	if (snprintf(tmp_gz_path, sizeof(tmp_gz_path), "%s.tmp", gz_path) >= (int)sizeof(tmp_gz_path))
		return -1;
	if (snprintf(tmp_out_path, sizeof(tmp_out_path), "%s.tmp", out_path) >= (int)sizeof(tmp_out_path))
		return -1;

	if (signal == GNSS_SIGNAL_GPS_L1CA) {
		if (snprintf(urls[0], sizeof(urls[0]),
			"https://geodesy.noaa.gov/corsdata/rinex/%04d/%03d/brdc%03d0.%02dn.gz",
			date->y, doy, doy, yy) >= (int)sizeof(urls[0]) ||
			snprintf(urls[1], sizeof(urls[1]),
			"https://igs.bkg.bund.de/root_ftp/IGS/BRDC/%04d/%03d/brdc%03d0.%02dn.gz",
			date->y, doy, doy, yy) >= (int)sizeof(urls[1]) ||
			snprintf(urls[2],sizeof(urls[2]),
			"https://igs.bkg.bund.de/root_ftp/IGS/BRDC/%04d/%03d/BRDC00WRD_R_%04d%03d0000_01D_MN.rnx.gz",
			date->y,doy,date->y,doy)>=(int)sizeof(urls[2]) ||
			snprintf(urls[3],sizeof(urls[3]),
			"https://igs.bkg.bund.de/root_ftp/IGS/BRDC/%04d/%03d/BRDC00WRD_S_%04d%03d0000_01D_MN.rnx.gz",
			date->y,doy,date->y,doy)>=(int)sizeof(urls[3])) return -1;
		source_count=4U;
	} else {
		if (snprintf(urls[0], sizeof(urls[0]),
			"https://igs.bkg.bund.de/root_ftp/IGS/BRDC/%04d/%03d/BRDC00IGS_R_%04d%03d0000_01D_MN.rnx.gz",
			date->y,doy,date->y,doy) >= (int)sizeof(urls[0]) ||
			snprintf(urls[1], sizeof(urls[1]),
			"https://igs.bkg.bund.de/root_ftp/IGS/BRDC/%04d/%03d/BRDC00WRD_R_%04d%03d0000_01D_MN.rnx.gz",
			date->y,doy,date->y,doy) >= (int)sizeof(urls[1]) ||
			snprintf(urls[2], sizeof(urls[2]),
			"https://igs.bkg.bund.de/root_ftp/IGS/BRDC/%04d/%03d/BRDC00WRD_S_%04d%03d0000_01D_MN.rnx.gz",
			date->y,doy,date->y,doy) >= (int)sizeof(urls[2]) ||
			snprintf(urls[3], sizeof(urls[3]),
			"https://igs.bkg.bund.de/root_ftp/IGS/BRDC/%04d/%03d/BRDM00DLR_S_%04d%03d0000_01D_MN.rnx.gz",
			date->y,doy,date->y,doy) >= (int)sizeof(urls[3])) return -1;
		source_count=4U;
	}

	remove(tmp_gz_path);
	remove(tmp_out_path);

	if (append_dependency_check(cmd, sizeof(cmd), "curl") != 0)
		return -1;
	if (run_command(cmd, "find curl in PATH") != 0)
		return -1;

	if (append_dependency_check(cmd, sizeof(cmd), "gzip") != 0)
		return -1;
	if (run_command(cmd, "find gzip in PATH") != 0)
		return -1;

	for (source = 0; source < source_count; source++) {
		int legacy_glonass_only=0;
		printf("Downloading broadcast ephemeris from source %zu/%zu: %s\n",
			source + 1,source_count,urls[source]);
		remove(tmp_gz_path);
		if (snprintf(cmd, sizeof(cmd),
			"curl -fL --retry 2 --connect-timeout 15 -o \"%s\" \"%s\"",
			tmp_gz_path, urls[source]) >= (int)sizeof(cmd))
			goto fail;
		if (run_command(cmd, "download broadcast ephemeris") == 0 && file_exists(tmp_gz_path)) {
			remove(tmp_out_path);
			if (snprintf(cmd, sizeof(cmd), "gzip -cd \"%s\" > \"%s\"", tmp_gz_path, tmp_out_path) >= (int)sizeof(cmd))
				goto fail;
			if(run_command(cmd,"decompress broadcast ephemeris")==0&&file_exists(tmp_out_path)&&
				downloaded_nav_supports_signal(tmp_out_path,signal,required_time,&legacy_glonass_only)) {
				downloaded = 1;break;
			}
			fprintf(stderr,"WARNING: Ephemeris source %zu produced no usable records%s; trying the next source.\n",
				source+1U,legacy_glonass_only?" (legacy GLONASS data lacks complete GNAV fields)":"");
			continue;
		}
		fprintf(stderr, "WARNING: Ephemeris source %zu failed; trying the next source.\n",
			source + 1);
	}
	if (!downloaded)
		goto fail;

	if (!file_exists(tmp_gz_path)) {
		fprintf(stderr, "ERROR: Download did not create %s.\n", tmp_gz_path);
		goto fail;
	}

	if (!file_exists(tmp_out_path)) {
		fprintf(stderr, "ERROR: Decompression did not create %s.\n", tmp_out_path);
		goto fail;
	}

	remove(gz_path);
	if (rename(tmp_gz_path, gz_path) != 0) {
		fprintf(stderr, "ERROR: Failed to save %s.\n", gz_path);
		goto fail;
	}

	remove(out_path);
	if (rename(tmp_out_path, out_path) != 0) {
		fprintf(stderr, "ERROR: Failed to save %s.\n", out_path);
		goto fail;
	}

	printf("Saved broadcast ephemeris: %s\n", out_path);
	return copy_option(navfile, navfile_size, out_path, "downloaded ephemeris path");

fail:
	remove(tmp_gz_path);
	remove(tmp_out_path);
	return -1;
}

void init_sim(sim_t *s)
{
	s->tx.dev = NULL;
	s->tx.buffer = NULL;
	memset(&s->tx.sample_stats, 0, sizeof(s->tx.sample_stats));
	s->tx.padded_samples = 0U;
	s->tx.submitted_samples = 0U;
	s->tx.start_timestamp = 0U;
	s->tx.end_timestamp = 0U;
	s->tx.host_elapsed_seconds = 0.0;
	s->tx.realtime_target_unix_seconds = 0.0;
	s->tx.hardware_timeline_valid = 0;
	s->tx.hardware_drain_complete = 0;
	pthread_mutex_init(&(s->tx.lock), NULL);
	s->tx.error = 0;

	pthread_mutex_init(&(s->gps.lock), NULL);
	s->gps.error = 0;
	s->gps.ready = 0;
	pthread_cond_init(&(s->gps.initialization_done), NULL);

	s->status = 0;
	s->finished = false;
	s->fifo = NULL;
	s->iq_block_samples = s->opt.tx_sample_rate / 10U;
	s->fifo_length = s->iq_block_samples * 2U;
	s->head = 0;
	s->tail = 0;
	s->sample_length = 0;

	pthread_cond_init(&(s->fifo_write_ready), NULL);
	pthread_cond_init(&(s->fifo_read_ready), NULL);

	s->time = 0.0;
}

size_t get_sample_length(sim_t *s)
{
	long length;

	length = s->head - s->tail;
	if (length < 0)
		length += (long)s->fifo_length;

	return((size_t)length);
}

size_t fifo_read(int16_t *buffer, size_t samples, sim_t *s)
{
	size_t length;
	size_t samples_remaining;
	int16_t *buffer_current = buffer;

	length = get_sample_length(s);

	if (length < samples)
		samples = length;

	length = samples; // return value

	samples_remaining = s->fifo_length - (size_t)s->tail;

	if (samples > samples_remaining) {
		memcpy(buffer_current, &(s->fifo[s->tail * 2]), samples_remaining * sizeof(int16_t) * 2);
		s->tail = 0;
		buffer_current += samples_remaining * 2;
		samples -= samples_remaining;
	}

	memcpy(buffer_current, &(s->fifo[s->tail * 2]), samples * sizeof(int16_t) * 2);
	s->tail += (long)samples;
	if ((size_t)s->tail >= s->fifo_length)
		s->tail -= (long)s->fifo_length;

	return(length);
}

bool is_finished_generation(sim_t *s)
{
	return s->finished;
}

int is_fifo_write_ready(sim_t *s)
{
	int status = 0;

	s->sample_length = get_sample_length(s);
	if (s->sample_length < s->iq_block_samples)
		status = 1;

	return(status);
}

static int monotonic_seconds(double *seconds)
{
	if (seconds == NULL)
		return -1;
#ifdef _WIN32
	{
		LARGE_INTEGER counter, frequency;
		if (!QueryPerformanceFrequency(&frequency) || frequency.QuadPart <= 0 ||
			!QueryPerformanceCounter(&counter))
			return -1;
		*seconds = (double)counter.QuadPart / (double)frequency.QuadPart;
	}
#else
	{
		struct timespec now;
		if (clock_gettime(CLOCK_MONOTONIC, &now) != 0)
			return -1;
		*seconds = (double)now.tv_sec + (double)now.tv_nsec / 1.0e9;
	}
#endif
	return 0;
}

static void sleep_one_millisecond(void)
{
#ifdef _WIN32
	Sleep(1U);
#else
	struct timespec delay = {0, 1000000L};
	(void)nanosleep(&delay, NULL);
#endif
}

static int wait_for_tx_drain(sim_t *s, bladerf_timestamp target)
{
	double now, deadline;
	bladerf_timestamp initial;
	if (s == NULL || s->tx.dev == NULL || s->opt.tx_sample_rate == 0U ||
		monotonic_seconds(&now) != 0)
		return -1;
	if(bladerf_get_timestamp(s->tx.dev,BLADERF_TX,&initial)!=0)return -1;
	deadline = now + 2.0 +
		(double)(NUM_BUFFERS * SAMPLES_PER_BUFFER) / (double)s->opt.tx_sample_rate+
		(target>initial?(double)(target-initial)/(double)s->opt.tx_sample_rate:0.0);
	for (;;) {
		bladerf_timestamp current;
		int status = bladerf_get_timestamp(s->tx.dev, BLADERF_TX, &current);
		if (status != 0)
			return status;
		if (current >= target)
			return 0;
		if (monotonic_seconds(&now) != 0 || now >= deadline)
			return BLADERF_ERR_TIMEOUT;
		sleep_one_millisecond();
	}
}

void *tx_task(void *arg)
{
	sim_t *s = (sim_t *)arg;
	size_t samples_populated;
	int status;
	int burst_started = 0;
	double host_start = 0.0, host_end = 0.0;

	while (1) {
		int16_t *tx_buffer_current = s->tx.buffer;
		unsigned int buffer_samples_remaining = SAMPLES_PER_BUFFER;
		unsigned int samples_to_send;
		int final_buffer = 0;
		struct bladerf_metadata metadata;

		while (buffer_samples_remaining > 0) {
			
			pthread_mutex_lock(&(s->gps.lock));
			while (get_sample_length(s) == 0 && !is_finished_generation(s))
			{
				pthread_cond_wait(&(s->fifo_read_ready), &(s->gps.lock));
			}
			if (get_sample_length(s) == 0 && is_finished_generation(s)) {
				pthread_mutex_unlock(&(s->gps.lock));
				final_buffer = 1;
				break;
			}
//			assert(get_sample_length(s) > 0);

			samples_populated = fifo_read(tx_buffer_current,
				buffer_samples_remaining,
				s);
			pthread_mutex_unlock(&(s->gps.lock));

			pthread_cond_signal(&(s->fifo_write_ready));
#if 0
			if (is_fifo_write_ready(s)) {
				/*
				printf("\rTime = %4.1f", s->time);
				s->time += 0.1;
				fflush(stdout);
				*/
			}
			else if (is_finished_generation(s))
			{
				goto out;
			}
#endif
			// Advance the buffer pointer.
			buffer_samples_remaining -= (unsigned int)samples_populated;
			tx_buffer_current += (2 * samples_populated);
		}
		if (!final_buffer && buffer_samples_remaining == 0U) {
			pthread_mutex_lock(&(s->gps.lock));
			final_buffer = get_sample_length(s) == 0U && is_finished_generation(s);
			pthread_mutex_unlock(&(s->gps.lock));
		}
		if (buffer_samples_remaining == SAMPLES_PER_BUFFER && !burst_started)
			goto out;

		/* A timestamped burst-end flushes the final transfer. Keep full transport
		 * buffers by zero-padding the tail; padding is silence and is reported. */
		samples_to_send = SAMPLES_PER_BUFFER - buffer_samples_remaining;
		blade_hw_measure_samples(s->tx.buffer, samples_to_send, &s->tx.sample_stats);
		if (samples_to_send < SAMPLES_PER_BUFFER) {
			memset(s->tx.buffer + 2U * samples_to_send, 0,
				2U * (SAMPLES_PER_BUFFER - samples_to_send) * sizeof(*s->tx.buffer));
			s->tx.padded_samples += SAMPLES_PER_BUFFER - samples_to_send;
		}
		memset(&metadata, 0, sizeof(metadata));
		if (!burst_started) {
			bladerf_timestamp current;
			double start_delay=0.1;
			status = bladerf_get_timestamp(s->tx.dev, BLADERF_TX, &current);
			if (status != 0) {
				fprintf(stderr, "Failed to read initial TX hardware timestamp: %s\n",
					bladerf_strerror(status));
				s->tx.error = status;
				goto stop_producer;
			}
			if(s->tx.realtime_target_unix_seconds>0.0) {
				double now;
				if(wallclock_seconds(&now)!=0) {
					fprintf(stderr,"Failed to read the UTC wall clock for real-time TX.\n");
					s->tx.error=BLADERF_ERR_UNEXPECTED;goto stop_producer;
				}
				start_delay=s->tx.realtime_target_unix_seconds-now;
				if(!isfinite(start_delay)||start_delay<0.05) {
					fprintf(stderr,"Real-time TX deadline has insufficient lead (%.6f seconds remain).\n",
						start_delay);
					s->tx.error=BLADERF_ERR_TIMEOUT;goto stop_producer;
				}
			}
			if(start_delay>(double)(UINT64_MAX-current)/(double)s->opt.tx_sample_rate) {
				fprintf(stderr,"TX hardware timestamp range overflow.\n");
				s->tx.error=BLADERF_ERR_RANGE;goto stop_producer;
			}
			s->tx.start_timestamp=current+(bladerf_timestamp)llround(
				start_delay*(double)s->opt.tx_sample_rate);
			metadata.timestamp = s->tx.start_timestamp;
			metadata.flags = BLADERF_META_FLAG_TX_BURST_START;
			s->tx.hardware_timeline_valid = 1;
			(void)monotonic_seconds(&host_start);
		}
		if (final_buffer)
			metadata.flags |= BLADERF_META_FLAG_TX_BURST_END;
		status = bladerf_sync_tx(s->tx.dev, s->tx.buffer, SAMPLES_PER_BUFFER,
			&metadata,s->tx.realtime_target_unix_seconds>0.0?
			REALTIME_TIMEOUT_MS:TIMEOUT_MS);
		if (status != 0) {
			fprintf(stderr, "TX stream failed: %s\n", bladerf_strerror(status));
			s->tx.error = status;
			goto stop_producer;
		}
		burst_started = 1;
		s->tx.submitted_samples += SAMPLES_PER_BUFFER;
		if (final_buffer) {
			if (s->tx.start_timestamp > UINT64_MAX - s->tx.submitted_samples) {
				fprintf(stderr, "TX hardware timestamp range overflow.\n");
				s->tx.error = BLADERF_ERR_RANGE;
				goto stop_producer;
			}
			s->tx.end_timestamp = s->tx.start_timestamp + s->tx.submitted_samples;
			status = wait_for_tx_drain(s, s->tx.end_timestamp);
			if (status != 0) {
				fprintf(stderr, "TX hardware timeline did not drain: %s\n",
					bladerf_strerror(status));
				s->tx.error = status;
				goto stop_producer;
			}
			s->tx.hardware_drain_complete = 1;
			if (monotonic_seconds(&host_end) == 0 && host_start > 0.0)
				s->tx.host_elapsed_seconds = host_end - host_start;
			goto out;
		}
		continue;

stop_producer:
		{
			pthread_mutex_lock(&(s->gps.lock));
			s->finished = true;
			pthread_cond_broadcast(&(s->fifo_write_ready));
			pthread_cond_broadcast(&(s->fifo_read_ready));
			pthread_mutex_unlock(&(s->gps.lock));
			goto out;
		}
	}
out:
	return NULL;
}

int start_tx_task(sim_t *s)
{
	int status;

	status = pthread_create(&(s->tx.thread), NULL, tx_task, s);

	return(status);
}

int start_gnss_task(sim_t *s)
{
	int status;

	/* Every advertised signal now uses the same constellation-neutral
	 * producer.  Keeping L1 C/A on the historical gps_task() path made its
	 * buffering, channel allocation, transmit-time alignment and SC16 scaling
	 * differ from the otherwise identical L1 C/A component of mixed-open. */
	status = pthread_create(&(s->gps.thread), NULL, gnss_task, s);

	return(status);
}

void usage(void)
{
	printf("Usage: bladegps [options]\n"
		"Options:\n"
		"  -e <nav_file>    RINEX navigation file (daily broadcast data auto-downloads if omitted)\n"
		"  -u <user_motion> User motion file (dynamic mode)\n"
		"  -p <llh_motion>  Geodetic CSV motion: time,latitude,longitude,height\n"
		"  -g <nmea_gga>    NMEA GGA stream (dynamic mode)\n"
		"  -l <location>    Lat,Lon,Hgt (static mode) e.g. 35.274,137.014,100\n"
		"  -t <date,time>   Scenario start time YYYY/MM/DD,hh:mm:ss\n"
		"  -R               Use current UTC/GPS time and align sample zero to the wall clock\n"
		"  -d <duration>    Duration [sec] (max: %.0f)\n"
		"  -x <XB number>   Enable XB board, e.g. '-x 200' for XB200\n"
		"  -S <signal>      Signal profile (gps-l1ca, galileo-e1, beidou-b1i, glonass-l1of, mixed-open)\n"
		"  -L               List signal profiles and implementation status\n"
		"  -D <device>      libbladeRF device identifier\n"
		"  -f <Hz>          TX center frequency\n"
		"  -r <samples/s>   TX sample rate (must be divisible by 10)\n"
		"  -b <Hz>          TX analog bandwidth\n"
		"  -G <dB>          Portable overall TX gain (default: %d dB)\n"
		"  -a <dB>          Legacy bladeRF 1 TXVGA1 gain (requires -A)\n"
		"  -A <dB>          Legacy bladeRF 1 TXVGA2 gain (requires -a)\n"
		"  -M <degrees>     Satellite elevation mask (-90 to 90)\n"
		"  -i               Interactive mode: North='%c', South='%c', East='%c', West='%c', Up='%c', Down='%c'\n"
		"  -j <index>       Live SDL USB/Bluetooth game controller index (left stick NE, right stick up/down)\n"
		"Profile-safe center, sample-rate, and filter defaults are used unless -f/-r/-b override them.\n"
		"See USER_GUIDE.md and CLI_REFERENCE.md for hardware limits, formats, and examples.\n",
		((double)USER_MOTION_SIZE)/10.0, DEFAULT_TX_GAIN,
		NORTH_KEY, SOUTH_KEY, EAST_KEY, WEST_KEY, UP_KEY, DOWN_KEY);

	return;
}

int main(int argc, char *argv[])
{
	sim_t s;
	char *devstr;
	int xb_board=0;
	int exit_code = 1;
	int gps_started = 0;
	int tx_enabled = 0;
	blade_hw_config_t hw_config;
	blade_hw_result_t hw_result;

	int result;
	datetime_t t0;
	datetime_t navdate;
	int navdate_set = 0;
	int list_signals = 0;
	int tx_frequency_set = 0;
	int tx_sample_rate_set = 0;
	int tx_bandwidth_set = 0;
	int tx_gain_set = 0;
	int tx_vga1_set = 0;
	int tx_vga2_set = 0;
	const gnss_signal_profile_t *signal_profile;
	void (*previous_sigint)(int);
#ifdef SIGTERM
	void (*previous_sigterm)(int);
#endif

	if (argc<2)
	{
		usage();
		exit(1);
	}
	s.finished = false;

	s.opt.navfile[0] = 0;
	s.opt.umfile[0] = 0;
	s.opt.device[0] = 0;
	s.opt.signal = GNSS_SIGNAL_GPS_L1CA;
	s.opt.tx_frequency = DEFAULT_TX_FREQUENCY;
	s.opt.tx_sample_rate = DEFAULT_TX_SAMPLERATE;
	s.opt.tx_bandwidth = DEFAULT_TX_BANDWIDTH;
	s.opt.tx_vga1 = DEFAULT_TX_VGA1;
	s.opt.tx_vga2 = DEFAULT_TX_VGA2;
	s.opt.tx_gain = DEFAULT_TX_GAIN;
	s.opt.elevation_mask = 0.0;
	s.opt.g0.week = -1;
	s.opt.g0.sec = 0.0;
	s.opt.iduration = USER_MOTION_SIZE;
	s.opt.verb = TRUE;
	s.opt.nmeaGGA = FALSE;
	s.opt.geodeticMotion = FALSE;
	s.opt.staticLocationMode = TRUE; // default static location
	s.opt.llh[0] = 35.274016 / R2D;
	s.opt.llh[1] = 137.013765 / R2D;
	s.opt.llh[2] = 100.0;
	s.opt.interactive = FALSE;
	s.opt.controller_index = -1;
	s.opt.realtime_start = FALSE;

	while ((result=getopt(argc,argv,"e:u:p:g:l:t:d:x:ij:S:LRD:f:r:b:G:a:A:M:"))!=-1)
	{
		switch (result)
		{
		case 'e':
			if (copy_option(s.opt.navfile, sizeof(s.opt.navfile), optarg, "ephemeris path") != 0)
				exit(1);
			break;
		case 'u':
			if (copy_option(s.opt.umfile, sizeof(s.opt.umfile), optarg, "user motion path") != 0)
				exit(1);
			s.opt.nmeaGGA = FALSE;
			s.opt.geodeticMotion = FALSE;
			s.opt.staticLocationMode = FALSE;
			break;
		case 'p':
			if (copy_option(s.opt.umfile, sizeof(s.opt.umfile), optarg, "geodetic motion path") != 0)
				exit(1);
			s.opt.nmeaGGA = FALSE;
			s.opt.geodeticMotion = TRUE;
			s.opt.staticLocationMode = FALSE;
			break;
		case 'g':
			if (copy_option(s.opt.umfile, sizeof(s.opt.umfile), optarg, "NMEA GGA path") != 0)
				exit(1);
			s.opt.nmeaGGA = TRUE;
			s.opt.geodeticMotion = FALSE;
			s.opt.staticLocationMode = FALSE;
			break;
		case 'l':
			// Static geodetic coordinates input mode
			// Added by scateu@gmail.com
			s.opt.nmeaGGA = FALSE;
			s.opt.geodeticMotion = FALSE;
			s.opt.staticLocationMode = TRUE;
			if (parse_location(optarg, s.opt.llh) != 0) {
				printf("ERROR: Invalid static location. Expected Lat,Lon,Hgt.\n");
				exit(1);
			}
			break;
		case 't':
			if (sscanf(optarg, "%d/%d/%d,%d:%d:%lf", &t0.y, &t0.m, &t0.d, &t0.hh, &t0.mm, &t0.sec) != 6) {
				printf("ERROR: Invalid date and time.\n");
				exit(1);
			}
			if (t0.y<=1980 || day_of_year(&t0) < 1 ||
				t0.hh<0 || t0.hh>23 || t0.mm<0 || t0.mm>59 ||
				!isfinite(t0.sec) || t0.sec<0.0 || t0.sec>=60.0)
			{
				printf("ERROR: Invalid date and time.\n");
				exit(1);
			}
			t0.sec = floor(t0.sec);
			date2gps(&t0, &s.opt.g0);
			{
				gnss_calendar_time_t utc;
				if(gnss_gps_to_utc_calendar(&s.opt.g0,&utc)!=0) {
					fprintf(stderr,"ERROR: Cannot convert scenario GPS time to UTC.\n");exit(1);
				}
				navdate=(datetime_t){utc.year,utc.month,utc.day,utc.hour,utc.minute,utc.second};
			}
			navdate_set = 1;
			break;
		case 'R':
			s.opt.realtime_start=TRUE;
			break;
		case 'd':
			if (parse_duration(optarg, &s.opt.iduration) != 0) {
				printf("ERROR: Invalid duration.\n");
				exit(1);
			}
			break;
		case 'x':
			if (parse_xb_board(optarg, &xb_board) != 0) {
				printf("ERROR: Invalid XB board number.\n");
				exit(1);
			}
			break;
		case 'i':
			s.opt.interactive = TRUE;
			break;
		case 'j':
			if(parse_int_option(optarg,0,255,&s.opt.controller_index)!=0){
				fprintf(stderr,"ERROR: Invalid controller index.\n");exit(1);}
			break;
		case 'S':
			if (gnss_signal_parse(optarg, &s.opt.signal) != 0) {
				fprintf(stderr, "ERROR: Unknown signal profile '%s'. Use -L to list profiles.\n", optarg);
				exit(1);
			}
			break;
		case 'L':
			list_signals = 1;
			break;
		case 'D':
			if (copy_option(s.opt.device, sizeof(s.opt.device), optarg, "bladeRF device identifier") != 0)
				exit(1);
			break;
		case 'f':
			if (parse_uint_option(optarg, 1U, UINT_MAX, &s.opt.tx_frequency) != 0) {
				fprintf(stderr, "ERROR: Invalid TX center frequency.\n");
				exit(1);
			}
			tx_frequency_set = 1;
			break;
		case 'b':
			if (parse_uint_option(optarg, 1U, UINT_MAX, &s.opt.tx_bandwidth) != 0) {
				fprintf(stderr, "ERROR: Invalid TX bandwidth.\n");
				exit(1);
			}
			tx_bandwidth_set = 1;
			break;
		case 'r':
			if (parse_uint_option(optarg, 1000000U, 100000000U, &s.opt.tx_sample_rate) != 0 ||
				s.opt.tx_sample_rate % 10U != 0U) {
				fprintf(stderr, "ERROR: Invalid sample rate; use at least 1000000 samples/s and a multiple of 10.\n");
				exit(1);
			}
			tx_sample_rate_set = 1;
			break;
		case 'G':
			if (parse_int_option(optarg, -200, 200, &s.opt.tx_gain) != 0) {
				fprintf(stderr, "ERROR: Invalid overall TX gain.\n");
				exit(1);
			}
			tx_gain_set = 1;
			break;
		case 'a':
			if (parse_int_option(optarg, -100, 100, &s.opt.tx_vga1) != 0) {
				fprintf(stderr, "ERROR: Invalid TX VGA1 gain.\n");
				exit(1);
			}
			tx_vga1_set = 1;
			break;
		case 'A':
			if (parse_int_option(optarg, -100, 100, &s.opt.tx_vga2) != 0) {
				fprintf(stderr, "ERROR: Invalid TX VGA2 gain.\n");
				exit(1);
			}
			tx_vga2_set = 1;
			break;
		case 'M':
			if (parse_elevation_mask(optarg, &s.opt.elevation_mask) != 0) {
				fprintf(stderr, "ERROR: Invalid elevation mask.\n");
				exit(1);
			}
			break;
		case ':':
		case '?':
			usage();
			exit(1);
		default:
			break;
		}
	}

	if (list_signals) {
		gnss_print_signal_profiles();
		return 0;
	}
	if(navdate_set&&s.opt.realtime_start) {
		fprintf(stderr,"ERROR: -R cannot be combined with explicit scenario time -t.\n");
		return 1;
	}
	if (tx_gain_set && (tx_vga1_set || tx_vga2_set)) {
		fprintf(stderr, "ERROR: -G cannot be combined with legacy -a/-A gain stages.\n");
		return 1;
	}
	if (tx_vga1_set != tx_vga2_set) {
		fprintf(stderr, "ERROR: Legacy gain mode requires both -a and -A. Prefer portable -G.\n");
		return 1;
	}

	signal_profile = gnss_signal_profile(s.opt.signal);
	if (signal_profile != NULL) {
		if (!tx_frequency_set)
			s.opt.tx_frequency = (unsigned int)signal_profile->carrier_hz;
		if (!tx_sample_rate_set)
			s.opt.tx_sample_rate = (unsigned int)signal_profile->minimum_sample_rate_hz;
		if (!tx_bandwidth_set)
			s.opt.tx_bandwidth = (unsigned int)signal_profile->recommended_bandwidth_hz;
	}
	if (signal_profile == NULL || !signal_profile->waveform_implemented) {
		fprintf(stderr, "ERROR: %s is registered but its waveform backend is not implemented yet.\n",
			signal_profile != NULL ? signal_profile->name : "selected signal");
		return 1;
	}
	if (!gnss_frequency_fits((double)s.opt.tx_frequency, (double)s.opt.tx_sample_rate,
		signal_profile->carrier_hz, signal_profile->occupied_bandwidth_hz)) {
		fprintf(stderr, "ERROR: Selected sample rate/center frequency does not contain the %s signal.\n",
			signal_profile->name);
		return 1;
	}
	if (blade_hw_validate_rf_plan((double)s.opt.tx_frequency,
		signal_profile->carrier_hz, signal_profile->occupied_bandwidth_hz,
		(double)s.opt.tx_sample_rate, (double)s.opt.tx_bandwidth) != 0) {
		fprintf(stderr, "ERROR: TX analog bandwidth must contain the complete signal span and not exceed the sample rate.\n");
		return 1;
	}
	if (blade_hw_validate_stream_geometry(NUM_BUFFERS, SAMPLES_PER_BUFFER,
		NUM_TRANSFERS) != 0) {
		fprintf(stderr, "ERROR: Invalid synchronous-stream buffer geometry.\n");
		return 1;
	}
	devstr = s.opt.device[0] != 0 ? s.opt.device : NULL;

	if (s.opt.navfile[0]==0) {
		gpstime_t required_time;
		const gpstime_t *required_time_pointer=NULL;
		if(!navdate_set)s.opt.realtime_start=TRUE;
		if (!navdate_set && utc_today(&navdate) != 0) {
			printf("ERROR: Navigation file is not specified and current UTC date is unavailable.\n");
			exit(1);
		}
		if(s.opt.realtime_start) {
			if(utc_now_gps(&required_time,NULL)!=0) {
				fprintf(stderr,"ERROR: Current UTC/GPS time is unavailable for ephemeris validation.\n");exit(1);
			}
			required_time_pointer=&required_time;
		} else if(s.opt.g0.week>=0) required_time_pointer=&s.opt.g0;
		if (download_broadcast_ephemeris(&navdate,s.opt.signal,required_time_pointer,
			s.opt.navfile,sizeof(s.opt.navfile)) != 0) {
			datetime_t fallback=navdate;
			if(!s.opt.realtime_start||previous_utc_day(&fallback)!=0||
				download_broadcast_ephemeris(&fallback,s.opt.signal,required_time_pointer,
					s.opt.navfile,sizeof(s.opt.navfile))!=0) {
				printf("ERROR: Failed to auto-download usable broadcast ephemeris. Use -e <nav_file> to provide one manually.\n");
				exit(1);
			}
			printf("Using previous-day broadcast ephemeris fallback for the live start.\n");
		}
	}

	if (s.opt.umfile[0]==0 && !s.opt.staticLocationMode)
	{
		printf("ERROR: User motion file / NMEA GGA stream is not specified.\n");
		printf("You may use -l to specify the static location directly.\n");
		exit(1);
	}

	// Initialize simulator
	init_sim(&s);
	previous_sigint = signal(SIGINT, request_stop);
#ifdef SIGTERM
	previous_sigterm = signal(SIGTERM, request_stop);
#endif

	// Allocate TX buffer to hold each block of samples to transmit.
	s.tx.buffer = (int16_t *)malloc(SAMPLES_PER_BUFFER * sizeof(int16_t) * 2); // for 16-bit I and Q samples
	
	if (s.tx.buffer == NULL) {
		fprintf(stderr, "Failed to allocate TX buffer.\n");
		goto out;
	}

	// Allocate FIFOs to hold 0.1 seconds of I/Q samples each.
	s.fifo = (int16_t *)malloc(s.fifo_length * sizeof(int16_t) * 2); // complex I/Q samples

	if (s.fifo == NULL) {
		fprintf(stderr, "Failed to allocate I/Q sample buffer.\n");
		goto out;
	}

	// Initializing device.
	printf("Opening and initializing device...\n");

	s.status = bladerf_open(&s.tx.dev, devstr);
	if (s.status != 0) {
		fprintf(stderr, "Failed to open device: %s\n", bladerf_strerror(s.status));
		goto out;
	}

	memset(&hw_config, 0, sizeof(hw_config));
	hw_config.frequency_hz = s.opt.tx_frequency;
	hw_config.sample_rate_hz = s.opt.tx_sample_rate;
	hw_config.bandwidth_hz = s.opt.tx_bandwidth;
	hw_config.signal_carrier_hz = signal_profile->carrier_hz;
	hw_config.occupied_bandwidth_hz = signal_profile->occupied_bandwidth_hz;
	hw_config.gain_db = s.opt.tx_gain;
	hw_config.use_legacy_gain = tx_vga1_set && tx_vga2_set;
	hw_config.txvga1_db = s.opt.tx_vga1;
	hw_config.txvga2_db = s.opt.tx_vga2;
	hw_config.xb_board = xb_board;
	s.status = blade_hw_configure_tx(s.tx.dev, &hw_config, &hw_result);
	if (s.status != 0)
		goto out;
	if(s.opt.realtime_start) {
		double now;
		if(utc_now_gps(&s.opt.g0,&now)!=0) {
			fprintf(stderr,"Failed to read current UTC/GPS time.\n");goto out;
		}
		s.opt.g0.sec+=REALTIME_START_LEAD_SECONDS;normalizeGpsTime(&s.opt.g0);
		s.tx.realtime_target_unix_seconds=now+REALTIME_START_LEAD_SECONDS;
		printf("Real-time epoch: GPS week %d SOW %.3f; RF sample zero scheduled %.1f seconds ahead.\n",
			s.opt.g0.week,s.opt.g0.sec,REALTIME_START_LEAD_SECONDS);
	}

	// Start the selected constellation producer task.
	s.status = start_gnss_task(&s);
	if (s.status != 0) {
		fprintf(stderr, "Failed to start GNSS producer task.\n");
		goto out;
	}
	else {
		gps_started = 1;
		printf("Creating GNSS producer task...\n");
	}

	// Wait until the GNSS producer is initialized.
	pthread_mutex_lock(&(s.gps.lock));
	while (!s.gps.ready)
		pthread_cond_wait(&(s.gps.initialization_done), &(s.gps.lock));
	if (s.finished) {
		pthread_mutex_unlock(&(s.gps.lock));
		fprintf(stderr, "GNSS signal generator failed to initialize.\n");
		goto out;
	}
	pthread_mutex_unlock(&(s.gps.lock));

	// Fillfull the FIFO.
	pthread_mutex_lock(&(s.gps.lock));
	if (is_fifo_write_ready(&s))
		pthread_cond_signal(&(s.fifo_write_ready));
	pthread_mutex_unlock(&(s.gps.lock));

	// Configure the TX module for use with the synchronous interface.
	s.status = bladerf_sync_config(s.tx.dev,
			BLADERF_TX_X1,
			BLADERF_FORMAT_SC16_Q11_META,
			NUM_BUFFERS,
			SAMPLES_PER_BUFFER,
			NUM_TRANSFERS,
			s.opt.realtime_start?REALTIME_TIMEOUT_MS:TIMEOUT_MS);

	if (s.status != 0) {
		fprintf(stderr, "Failed to configure TX sync interface: %s\n", bladerf_strerror(s.status));
		goto out;
	}

	// We must always enable the modules *after* calling bladerf_sync_config().
	s.status = bladerf_enable_module(s.tx.dev, BLADERF_MODULE_TX, true);
	if (s.status != 0) {
		fprintf(stderr, "Failed to enable TX module: %s\n", bladerf_strerror(s.status));
		goto out;
	}
	tx_enabled = 1;

	// Start TX task
	s.status = start_tx_task(&s);
	if (s.status != 0) {
		fprintf(stderr, "Failed to start TX task.\n");
		goto out;
	}
	else {
		printf("Creating TX task...\n");
	}

	// Running...
	printf("Running...\n");
	printf("Press 'Ctrl+C' to abort.\n");

	// Wait for the TX task to complete.
	pthread_join(s.tx.thread, NULL);
	printf("TX digital peak: %u/2048; rail components: %llu across %llu complex samples\n",
		s.tx.sample_stats.peak_abs,
		(unsigned long long)s.tx.sample_stats.rail_components,
		(unsigned long long)s.tx.sample_stats.complex_samples);
	if (s.tx.sample_stats.rail_components != 0U)
		fprintf(stderr, "WARNING: Digital I/Q reached the SC16 Q11 rails; reduce waveform amplitude.\n");
	if (s.tx.padded_samples != 0U)
		printf("TX stream flush: %llu trailing zero samples\n",
			(unsigned long long)s.tx.padded_samples);
	if (s.tx.hardware_timeline_valid) {
		double rf_seconds = (double)s.tx.submitted_samples /
			(double)s.opt.tx_sample_rate;
		printf("TX hardware timeline: %llu samples, %.6f seconds; drain %s",
			(unsigned long long)s.tx.submitted_samples, rf_seconds,
			s.tx.hardware_drain_complete ? "complete" : "incomplete");
		if (s.tx.host_elapsed_seconds > 0.0)
			printf("; host elapsed %.6f seconds", s.tx.host_elapsed_seconds);
		printf("\n");
	}
	if (stop_was_requested())
		printf("\nStopped by user.\n");
	else if (s.tx.error == 0 && s.gps.error == 0)
		printf("\nDone!\n");
	else
		printf("\nAborted after signal-generation or TX error.\n");

	// Disable TX module and shut down underlying TX stream.
	s.status = bladerf_enable_module(s.tx.dev, BLADERF_MODULE_TX, false);
	if (s.status != 0)
		fprintf(stderr, "Failed to disable TX module: %s\n", bladerf_strerror(s.status));
	tx_enabled = 0;

	pthread_join(s.gps.thread, NULL);
	gps_started = 0;
	exit_code = stop_was_requested() ? 130 :
		((s.tx.error == 0 && s.gps.error == 0) ? 0 : 1);

out:
	if (gps_started) {
		pthread_mutex_lock(&(s.gps.lock));
		s.finished = true;
		pthread_cond_broadcast(&(s.fifo_read_ready));
		pthread_cond_broadcast(&(s.fifo_write_ready));
		pthread_cond_broadcast(&(s.gps.initialization_done));
		pthread_mutex_unlock(&(s.gps.lock));
	}

	if (gps_started)
		pthread_join(s.gps.thread, NULL);

	if (tx_enabled && s.tx.dev != NULL) {
		s.status = bladerf_enable_module(s.tx.dev, BLADERF_MODULE_TX, false);
		if (s.status != 0)
			fprintf(stderr, "Failed to disable TX module: %s\n", bladerf_strerror(s.status));
	}

	// Free up resources
	if (s.tx.buffer != NULL)
		free(s.tx.buffer);

	if (s.fifo != NULL)
		free(s.fifo);

	if (s.tx.dev != NULL) {
		printf("Closing device...\n");
		bladerf_close(s.tx.dev);
	}

	pthread_cond_destroy(&(s.fifo_read_ready));
	pthread_cond_destroy(&(s.fifo_write_ready));
	pthread_cond_destroy(&(s.gps.initialization_done));
	pthread_mutex_destroy(&(s.gps.lock));
	pthread_mutex_destroy(&(s.tx.lock));
	if (previous_sigint != SIG_ERR)
		signal(SIGINT, previous_sigint);
#ifdef SIGTERM
	if (previous_sigterm != SIG_ERR)
		signal(SIGTERM, previous_sigterm);
#endif

	return(exit_code);
}
