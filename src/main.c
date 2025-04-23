#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#define MODULE main
#include <app_event_manager.h>
#include <caf/events/module_state_event.h>

LOG_MODULE_REGISTER(main);

int main(void)
{
	LOG_INF("ISP3x80 Ranging Demo version %s", CONFIG_ISP_VERSION);

	if (app_event_manager_init())
	{
		LOG_ERR("Application Event Manager not initialized");
	}
	else
	{
		module_set_state(MODULE_STATE_READY);
	}


	return 0;
}
