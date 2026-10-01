#include "motion_controller.h"

#include <math.h>
#include <stddef.h>

#ifdef HAVE_SDL2
#include <SDL2/SDL.h>

static double axis_value(Sint16 raw)
{
	const double deadzone=4096.0;
	double value=(double)raw/32767.0,magnitude=fabs(value);
	if(magnitude<=deadzone/32767.0)return 0.0;
	magnitude=(magnitude-deadzone/32767.0)/(1.0-deadzone/32767.0);
	return copysign(magnitude>1.0?1.0:magnitude,value);
}
#endif

int motion_controller_open(motion_controller_t *controller, int index)
{
	if(controller==NULL||index<0)return -1;
	controller->handle=NULL;controller->active=0;
#ifdef HAVE_SDL2
	if(SDL_InitSubSystem(SDL_INIT_GAMECONTROLLER|SDL_INIT_JOYSTICK|SDL_INIT_EVENTS)!=0)
		return -1;
	if(index>=SDL_NumJoysticks()||!SDL_IsGameController(index)) {
		SDL_QuitSubSystem(SDL_INIT_GAMECONTROLLER|SDL_INIT_JOYSTICK|SDL_INIT_EVENTS);
		return -1;
	}
	controller->handle=SDL_GameControllerOpen(index);
	if(controller->handle==NULL) {
		SDL_QuitSubSystem(SDL_INIT_GAMECONTROLLER|SDL_INIT_JOYSTICK|SDL_INIT_EVENTS);
		return -1;
	}
	controller->active=1;return 0;
#else
	(void)index;return -2;
#endif
}

int motion_controller_poll(motion_controller_t *controller,
	double horizontal, double vertical, double neu[3])
{
	if(controller==NULL||!controller->active||neu==NULL||
		!isfinite(horizontal)||horizontal<0.0||!isfinite(vertical)||vertical<0.0)
		return -1;
#ifdef HAVE_SDL2
	SDL_GameController *game=(SDL_GameController *)controller->handle;
	double north,east,up,magnitude;
	SDL_GameControllerUpdate();
	if(game==NULL||!SDL_GameControllerGetAttached(game))return -1;
	east=axis_value(SDL_GameControllerGetAxis(game,SDL_CONTROLLER_AXIS_LEFTX));
	north=-axis_value(SDL_GameControllerGetAxis(game,SDL_CONTROLLER_AXIS_LEFTY));
	up=-axis_value(SDL_GameControllerGetAxis(game,SDL_CONTROLLER_AXIS_RIGHTY));
	magnitude=hypot(north,east);
	if(magnitude>1.0){north/=magnitude;east/=magnitude;}
	neu[0]=north*horizontal;neu[1]=east*horizontal;neu[2]=up*vertical;
	return 0;
#else
	(void)horizontal;(void)vertical;return -2;
#endif
}

void motion_controller_close(motion_controller_t *controller)
{
	if(controller==NULL)return;
#ifdef HAVE_SDL2
	if(controller->handle!=NULL)SDL_GameControllerClose((SDL_GameController *)controller->handle);
	if(controller->active)SDL_QuitSubSystem(SDL_INIT_GAMECONTROLLER|SDL_INIT_JOYSTICK|SDL_INIT_EVENTS);
#endif
	controller->handle=NULL;controller->active=0;
}
