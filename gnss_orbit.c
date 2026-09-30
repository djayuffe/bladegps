#include "gnss_orbit.h"

#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#define GNSS_WEEK_SECONDS 604800.0
#define GNSS_HALF_WEEK_SECONDS 302400.0
#define GLONASS_MU 3.9860044e14
#define GLONASS_AE 6378136.0
#define GLONASS_J2 1.0826257e-3
#define GLONASS_OMEGA 7.292115e-5

static double wrap_week(double value)
{
	while (value > GNSS_HALF_WEEK_SECONDS)
		value -= GNSS_WEEK_SECONDS;
	while (value < -GNSS_HALF_WEEK_SECONDS)
		value += GNSS_WEEK_SECONDS;
	return value;
}

static int64_t days_from_civil(int year, unsigned int month, unsigned int day)
{
	int adjusted_year = year - (month <= 2U);
	int era = (adjusted_year >= 0 ? adjusted_year : adjusted_year - 399) / 400;
	unsigned int year_of_era = (unsigned int)(adjusted_year - era * 400);
	unsigned int adjusted_month = month > 2U ? month - 3U : month + 9U;
	unsigned int day_of_year = (153U * adjusted_month + 2U) / 5U + day - 1U;
	unsigned int day_of_era = year_of_era * 365U + year_of_era / 4U -
		year_of_era / 100U + day_of_year;
	return (int64_t)era * 146097 + (int64_t)day_of_era - 719468;
}

static double calendar_sow(const gnss_calendar_time_t *time)
{
	int64_t days = days_from_civil(time->year, (unsigned int)time->month,
		(unsigned int)time->day);
	int weekday = (int)((days + 4) % 7);
	if (weekday < 0)
		weekday += 7;
	return (double)weekday * 86400.0 + (double)time->hour * 3600.0 +
		(double)time->minute * 60.0 + time->second;
}

static int system_constants(gnss_system_t system, double *mu, double *omega)
{
	if (mu == NULL || omega == NULL)
		return -1;
	switch (system) {
	case GNSS_SYSTEM_GPS:
		*mu = 3.986005e14;
		*omega = 7.2921151467e-5;
		return 0;
	case GNSS_SYSTEM_GALILEO:
		*mu = 3.986004418e14;
		*omega = 7.2921151467e-5;
		return 0;
	case GNSS_SYSTEM_BEIDOU:
		*mu = 3.986004418e14;
		*omega = 7.2921150e-5;
		return 0;
	default:
		return -1;
	}
}

static int kepler_position(const gnss_nav_record_t *record, double transmit_sow,
	double position[3], double *eccentric_anomaly)
{
	const double *v;
	double mu, omega_e, a, tk, mean_motion, mean_anomaly, eccentric;
	double true_anomaly, phi, du, dr, di, u, radius, inclination, node;
	double x_orbit, y_orbit;
	int iteration;

	if (record == NULL || position == NULL || record->model != GNSS_NAV_KEPLERIAN ||
		record->orbit_count < 17U || system_constants(record->system, &mu, &omega_e) != 0)
		return -1;
	v = record->orbit;
	a = v[7] * v[7];
	if (!isfinite(a) || a < 1.0e6 || !isfinite(v[5]) || v[5] < 0.0 || v[5] >= 1.0)
		return -1;
	tk = wrap_week(transmit_sow - v[8]);
	mean_motion = sqrt(mu / (a * a * a)) + v[2];
	mean_anomaly = v[3] + mean_motion * tk;
	eccentric = mean_anomaly;
	for (iteration = 0; iteration < 20; iteration++) {
		double next = mean_anomaly + v[5] * sin(eccentric);
		if (fabs(next - eccentric) < 1.0e-14) {
			eccentric = next;
			break;
		}
		eccentric = next;
	}
	true_anomaly = atan2(sqrt(1.0 - v[5] * v[5]) * sin(eccentric),
		cos(eccentric) - v[5]);
	phi = true_anomaly + v[14];
	du = v[6] * sin(2.0 * phi) + v[4] * cos(2.0 * phi);
	dr = v[1] * sin(2.0 * phi) + v[13] * cos(2.0 * phi);
	di = v[11] * sin(2.0 * phi) + v[9] * cos(2.0 * phi);
	u = phi + du;
	radius = a * (1.0 - v[5] * cos(eccentric)) + dr;
	inclination = v[12] + v[16] * tk + di;
	x_orbit = radius * cos(u);
	y_orbit = radius * sin(u);

	if (record->system == GNSS_SYSTEM_BEIDOU && strcmp(record->message, "D2") == 0) {
		double inertial[3];
		double tilted[3];
		double angle;
		const double tilt = -5.0 * (3.14159265358979323846 / 180.0);
		node = v[10] + v[15] * tk - omega_e * v[8];
		inertial[0] = x_orbit * cos(node) - y_orbit * cos(inclination) * sin(node);
		inertial[1] = x_orbit * sin(node) + y_orbit * cos(inclination) * cos(node);
		inertial[2] = y_orbit * sin(inclination);
		tilted[0] = inertial[0];
		tilted[1] = cos(tilt) * inertial[1] + sin(tilt) * inertial[2];
		tilted[2] = -sin(tilt) * inertial[1] + cos(tilt) * inertial[2];
		angle = omega_e * tk;
		position[0] = cos(angle) * tilted[0] + sin(angle) * tilted[1];
		position[1] = -sin(angle) * tilted[0] + cos(angle) * tilted[1];
		position[2] = tilted[2];
	} else {
		node = v[10] + (v[15] - omega_e) * tk - omega_e * v[8];
		position[0] = x_orbit * cos(node) - y_orbit * cos(inclination) * sin(node);
		position[1] = x_orbit * sin(node) + y_orbit * cos(inclination) * cos(node);
		position[2] = y_orbit * sin(inclination);
	}
	if (eccentric_anomaly != NULL)
		*eccentric_anomaly = eccentric;
	return 0;
}

int gnss_propagate_kepler(const gnss_nav_record_t *record, double transmit_sow,
	double position[3], double velocity[3], double *clock_bias,
	double *clock_drift)
{
	double before[3], after[3], eccentric, eccentric_before, eccentric_after;
	double dt;
	const double relativity = -4.442807309e-10;
	int axis;

	if (record == NULL || position == NULL || velocity == NULL || clock_bias == NULL ||
		clock_drift == NULL || kepler_position(record, transmit_sow, position, &eccentric) != 0 ||
		kepler_position(record, transmit_sow - 0.5, before, &eccentric_before) != 0 ||
		kepler_position(record, transmit_sow + 0.5, after, &eccentric_after) != 0)
		return -1;
	for (axis = 0; axis < 3; axis++)
		velocity[axis] = after[axis] - before[axis];
	dt = wrap_week(transmit_sow - calendar_sow(&record->toc));
	*clock_bias = record->clock_bias + record->clock_drift * dt +
		record->clock_drift_rate * dt * dt +
		relativity * record->orbit[5] * record->orbit[7] * sin(eccentric);
	*clock_drift = record->clock_drift + 2.0 * record->clock_drift_rate * dt +
		relativity * record->orbit[5] * record->orbit[7] *
		(sin(eccentric_after) - sin(eccentric_before));
	return isfinite(*clock_bias) && isfinite(*clock_drift) ? 0 : -1;
}

static void glonass_derivative(const double state[6], const double acceleration[3],
	double derivative[6])
{
	double r2 = state[0]*state[0] + state[1]*state[1] + state[2]*state[2];
	double r = sqrt(r2);
	double gravity = -GLONASS_MU / (r2 * r);
	double j2 = 1.5 * GLONASS_J2 * GLONASS_MU * GLONASS_AE * GLONASS_AE /
		(r2 * r2 * r);
	double z_ratio = 5.0 * state[2] * state[2] / r2;

	derivative[0] = state[3];
	derivative[1] = state[4];
	derivative[2] = state[5];
	derivative[3] = gravity*state[0] + j2*state[0]*(1.0-z_ratio) +
		GLONASS_OMEGA*GLONASS_OMEGA*state[0] + 2.0*GLONASS_OMEGA*state[4] + acceleration[0];
	derivative[4] = gravity*state[1] + j2*state[1]*(1.0-z_ratio) +
		GLONASS_OMEGA*GLONASS_OMEGA*state[1] - 2.0*GLONASS_OMEGA*state[3] + acceleration[1];
	derivative[5] = gravity*state[2] + j2*state[2]*(3.0-z_ratio) + acceleration[2];
}

int gnss_propagate_glonass(const gnss_nav_record_t *record, double delta_seconds,
	double position[3], double velocity[3], double *clock_bias,
	double *clock_drift)
{
	double state[6], acceleration[3];
	double remaining;
	int axis;

	if (record == NULL || position == NULL || velocity == NULL || clock_bias == NULL ||
		clock_drift == NULL || record->model != GNSS_NAV_GLONASS_STATE_VECTOR ||
		record->orbit_count < 12U || !isfinite(delta_seconds))
		return -1;
	for (axis = 0; axis < 3; axis++) {
		size_t base = (size_t)axis * 4U;
		state[axis] = record->orbit[base] * 1000.0;
		state[axis + 3] = record->orbit[base + 1U] * 1000.0;
		acceleration[axis] = record->orbit[base + 2U] * 1000.0;
		if (!isfinite(state[axis]) || !isfinite(state[axis + 3]) || !isfinite(acceleration[axis]))
			return -1;
	}
	remaining = delta_seconds;
	while (fabs(remaining) > 1.0e-12) {
		double h = remaining > 60.0 ? 60.0 : remaining < -60.0 ? -60.0 : remaining;
		double k1[6], k2[6], k3[6], k4[6], temp[6];
		int index;
		glonass_derivative(state, acceleration, k1);
		for (index=0; index<6; index++) temp[index]=state[index]+0.5*h*k1[index];
		glonass_derivative(temp, acceleration, k2);
		for (index=0; index<6; index++) temp[index]=state[index]+0.5*h*k2[index];
		glonass_derivative(temp, acceleration, k3);
		for (index=0; index<6; index++) temp[index]=state[index]+h*k3[index];
		glonass_derivative(temp, acceleration, k4);
		for (index=0; index<6; index++)
			state[index] += h*(k1[index]+2.0*k2[index]+2.0*k3[index]+k4[index])/6.0;
		remaining -= h;
	}
	for (axis = 0; axis < 3; axis++) {
		position[axis] = state[axis];
		velocity[axis] = state[axis + 3];
	}
	*clock_bias = record->clock_bias + record->clock_drift * delta_seconds;
	*clock_drift = record->clock_drift;
	return 0;
}
