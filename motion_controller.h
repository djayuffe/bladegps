#ifndef BLADEGPS_MOTION_CONTROLLER_H
#define BLADEGPS_MOTION_CONTROLLER_H

typedef struct {
	void *handle;
	int active;
} motion_controller_t;

/* Advance the keyboard velocity state by one 100 ms input update. Direction
 * zero means no new key event; positive direction identifiers are opaque to
 * this helper. A new direction starts immediately at one increment, repeated
 * events accelerate to maximum, and missing events decelerate to rest. */
int motion_keyboard_update(int requested_direction, int *active_direction,
	double *speed_mps, double increment_mps, double maximum_mps);

/* Open an SDL game controller by zero-based index.  Builds without SDL keep
 * the API but return an explicit unsupported error. */
int motion_controller_open(motion_controller_t *controller, int index);
int motion_controller_poll(motion_controller_t *controller,
	double maximum_horizontal_mps, double maximum_vertical_mps,
	double neu_velocity_mps[3]);
void motion_controller_close(motion_controller_t *controller);

#endif
