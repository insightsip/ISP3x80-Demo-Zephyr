/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */


#include <assert.h>
#include <stdio.h>

#include "config_event.h"


static void log_config_twr_event(const struct app_event_header *aeh)
{
	const struct config_twr_event *event = cast_config_twr_event(aeh);

	APP_EVENT_MANAGER_LOG(aeh, "twr role = %d m", event->role);
}

APP_EVENT_TYPE_DEFINE(config_twr_event, log_config_twr_event, NULL, APP_EVENT_FLAGS_CREATE());

static void log_config_twr_interval_event(const struct app_event_header *aeh)
{
	const struct config_twr_interval_event *event = cast_config_twr_interval_event(aeh);

	APP_EVENT_MANAGER_LOG(aeh, "interval=%d m", event->interval);
}

APP_EVENT_TYPE_DEFINE(config_twr_interval_event, log_config_twr_interval_event, NULL, APP_EVENT_FLAGS_CREATE());