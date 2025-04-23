
#ifndef _MOTION_EVENT_H_
#define _MOTION_EVENT_H_

#include <app_event_manager.h>
#include <app_event_manager_profiler_tracer.h>


struct accel_event
{
	struct app_event_header header;

	float x;
	float y;
	float z;
};

APP_EVENT_TYPE_DECLARE(accel_event);

#endif /* _MOTION_EVENT_H_ */
