/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#ifndef _CONFIG_EVENT_H_
#define _CONFIG_EVENT_H_

#include <app_event_manager.h>

typedef enum
{
	TWR_ROLE_INITIATOR,
	TWR_ROLE_RESPONDER
} twr_role_t;

struct config_twr_event
{
	struct app_event_header header;

	bool enable;
	twr_role_t role;
};

APP_EVENT_TYPE_DECLARE(config_twr_event);

struct config_twr_interval_event
{
	struct app_event_header header;

	uint16_t interval;
};

APP_EVENT_TYPE_DECLARE(config_twr_interval_event);

#endif /* _CONFIG_EVENT_H_ */
