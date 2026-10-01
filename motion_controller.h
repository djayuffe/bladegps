#ifndef BLADEGPS_MOTION_CONTROLLER_H
#define BLADEGPS_MOTION_CONTROLLER_H

typedef struct {
	void *handle;
	int active;
} motion_controller_t;

/* Open an SDL game controller by zero-based index.  Builds without SDL keep
 * the API but return an explicit unsupported error. */
int motion_controller_open(motion_controller_t *controller, int index);
int motion_controller_poll(motion_controller_t *controller,
	double maximum_horizontal_mps, double maximum_vertical_mps,
	double neu_velocity_mps[3]);
void motion_controller_close(motion_controller_t *controller);

#endif
