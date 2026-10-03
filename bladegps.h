#ifndef _BLADEGPS_H
#define _BLADEGPS_H

#include <stdlib.h>
#include <stdio.h>
#include <libbladeRF.h>
#include <string.h>
#ifdef _WIN32
// To avoid conflict between time.h and pthread.h on Windows
#define HAVE_STRUCT_TIMESPEC
#endif
#include <pthread.h>
#include "gpssim.h"
#include "gnss.h"
#include "gnss_time.h"
#include "gnss_receiver.h"
#include "gnss_codes.h"
#include "gnss_nav.h"
#include "gnss_orbit.h"
#include "gnss_fec.h"
#include "gnss_galileo_nav.h"
#include "gnss_beidou_nav.h"
#include "gnss_glonass_nav.h"
#include "gnss_rf.h"
#include "gnss_schedule.h"
#include "gnss_geometry.h"
#include "gnss_task.h"
#include "blade_hw.h"
#include "motion_controller.h"

#define DEFAULT_TX_FREQUENCY	1575420000U
#define DEFAULT_TX_SAMPLERATE	2600000U
#define DEFAULT_TX_BANDWIDTH	2500000U
#define DEFAULT_TX_VGA1			-25
#define DEFAULT_TX_VGA2			0
#define DEFAULT_TX_GAIN			27

#define NUM_BUFFERS			32
#define SAMPLES_PER_BUFFER	(32 * 1024)
#define NUM_TRANSFERS		16
#define TIMEOUT_MS			1000


// Interactive mode directions
#define UNDEF 0
#define NORTH 1
#define SOUTH 2
#define EAST  3
#define WEST  4
#define UP    5
#define DOWN  6
// Interactive keys
#define NORTH_KEY 'w'
#define SOUTH_KEY 's'
#define EAST_KEY  'd'
#define WEST_KEY  'a'
#define UP_KEY    'e'
#define DOWN_KEY  'q'
// Interactive motion
#define MAX_VEL 1.4 // 1.4 m/s = 5 km/h
#define DEL_VEL 0.1

typedef struct {
	char navfile[MAX_CHAR];
	char umfile[MAX_CHAR];
	char device[MAX_CHAR];
	gnss_signal_t signal;
	unsigned int tx_frequency;
	unsigned int tx_sample_rate;
	unsigned int tx_bandwidth;
	int tx_vga1;
	int tx_vga2;
	int tx_gain;
	double elevation_mask;
	int staticLocationMode;
	int nmeaGGA;
	int geodeticMotion;
	int iduration;
	int verb;
	gpstime_t g0;
	double llh[3];
	int interactive;
	int controller_index;
} option_t;

typedef struct {
	pthread_t thread;
	pthread_mutex_t lock;
	int error;

	struct bladerf *dev;
	int16_t *buffer;
	blade_sample_stats_t sample_stats;
	uint64_t padded_samples;
	uint64_t submitted_samples;
	bladerf_timestamp start_timestamp;
	bladerf_timestamp end_timestamp;
	double host_elapsed_seconds;
	int hardware_timeline_valid;
	int hardware_drain_complete;
} tx_t;

typedef struct {
	pthread_t thread;
	pthread_mutex_t lock;
	int error;

	int ready;
	pthread_cond_t initialization_done;
} gps_t;

typedef struct {
	option_t opt;

	tx_t tx;
	gps_t gps;

	int status;
	bool finished;
	int16_t *fifo;
	long head, tail;
	size_t iq_block_samples;
	size_t fifo_length;
	size_t sample_length;

	pthread_cond_t fifo_read_ready;
	pthread_cond_t fifo_write_ready;

	double time;
} sim_t;

extern void *gps_task(void *arg);
extern int is_fifo_write_ready(sim_t *s);
extern int stop_was_requested(void);

#endif
