
#ifndef _TWR_EVENT_H_
#define _TWR_EVENT_H_

#include <app_event_manager.h>
#include <app_event_manager_profiler_tracer.h>


typedef enum {
	TWR_TX_POLL_FAILED = 0,
	TWR_TX_RESP_FAILED,
	TWR_RX_POLL_FAILED,
	TWR_RX_RESP_FAILED,
	TWR_RX_POLL_TIMEOUT,
	TWR_RX_RESP_TIMEOUT
} twr_error_t;

struct range_event
{
	struct app_event_header header;

	float range;
};

APP_EVENT_TYPE_DECLARE(range_event);

struct twr_error_event
{
	struct app_event_header header;

	twr_error_t code;
};

APP_EVENT_TYPE_DECLARE(twr_error_event);

#endif /* _TWR_EVENT_H_ */
