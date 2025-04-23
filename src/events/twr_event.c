
#include <assert.h>
#include <stdio.h>

#include "twr_event.h"

static void log_range_event(const struct app_event_header *aeh)
{
	const struct range_event *event = cast_range_event(aeh);

	APP_EVENT_MANAGER_LOG(aeh, "range = %.02f m", (double)event->range);
}

APP_EVENT_TYPE_DEFINE(range_event, log_range_event, NULL, APP_EVENT_FLAGS_CREATE(IF_ENABLED(CONFIG_ISP_LOG_TWR_RANGE_EVENT, (APP_EVENT_TYPE_FLAGS_INIT_LOG_ENABLE))));

static void log_twr_error_event(const struct app_event_header *aeh)
{
	const struct twr_error_event *event = cast_twr_error_event(aeh);

	APP_EVENT_MANAGER_LOG(aeh, "code = %d", event->code);
}

APP_EVENT_TYPE_DEFINE(twr_error_event, log_twr_error_event, NULL, APP_EVENT_FLAGS_CREATE(IF_ENABLED(CONFIG_ISP_LOG_TWR_ERROR_EVENT, (APP_EVENT_TYPE_FLAGS_INIT_LOG_ENABLE))));