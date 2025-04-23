#include <zephyr/init.h>
#include <zephyr/settings/settings.h>

#define PRIORITY 7


int set_version()
{
#if defined(CONFIG_BT_DIS_FW_REV)
	settings_runtime_set("bt/dis/fw", CONFIG_ISP_VERSION, sizeof(CONFIG_ISP_VERSION));
#endif
	return 0;
}

SYS_INIT(set_version, APPLICATION, PRIORITY);

