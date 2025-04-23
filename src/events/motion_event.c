
#include <assert.h>
#include <stdio.h>

#include "motion_event.h"

static void log_accel_event(const struct app_event_header *aeh)
{
	const struct accel_event *event = cast_accel_event(aeh);

	APP_EVENT_MANAGER_LOG(aeh, "x: %.02f, y: %.02f, z: %.02f", (double)event->x,(double)event->y,(double)event->z);
}

APP_EVENT_TYPE_DEFINE(accel_event, log_accel_event, NULL, APP_EVENT_FLAGS_CREATE(IF_ENABLED(CONFIG_ISP_LOG_MOTION_ACCEL_EVENT, (APP_EVENT_TYPE_FLAGS_INIT_LOG_ENABLE))));
