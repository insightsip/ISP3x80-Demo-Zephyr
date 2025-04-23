/******************************************************************************
 * @file    ble_range_service.c
 * @author  Insight SiP
 *
 * @attention
 *	THIS SOFTWARE IS PROVIDED BY INSIGHT SIP "AS IS" AND ANY EXPRESS
 *	OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 *	OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *	DISCLAIMED. IN NO EVENT SHALL INSIGHT SIP OR CONTRIBUTORS BE
 *	LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *	CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 *	GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 *	HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *	LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 *	OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *****************************************************************************/

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#define MODULE ble_range
#include <app_event_manager.h>
#include <caf/events/module_state_event.h>
#include "ble_range_service.h"
#include "twr_event.h"

LOG_MODULE_REGISTER(MODULE);

static double range;
static uint16_t range_interval = 300; /**< Interval between ranging operations */
static uint16_t uwb_conf = 0; 
static uint8_t avg_conf = 0; 
static uint32_t threshold_conf = 0; 
#if (CONFIG_ISP_TWR_INITIATOR)
static uint8_t mode_conf = 1; 
static uint32_t controls = 1; 
static uint32_t status = 1; 
#else
static uint8_t mode_conf = 2; 
static uint32_t controls = 0; 
static uint32_t status = 0; 
#endif
static bool is_range_active;


static ssize_t range_read(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset)
{
	const char *value = attr->user_data;

	return bt_gatt_attr_read(conn, attr, buf, len, offset, value, sizeof(*value));
}

static void range_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	is_range_active = (value == BT_GATT_CCC_NOTIFY);
}

static ssize_t avg_range_read(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset)
{
	const char *value = attr->user_data;

	return bt_gatt_attr_read(conn, attr, buf, len, offset, value, sizeof(*value));
}

static ssize_t interval_conf_read(struct bt_conn *conn,	const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset)
{
	return bt_gatt_attr_read(conn, attr, buf, len, offset, &range_interval, sizeof(range_interval));
}

static ssize_t interval_conf_write(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
	if (len == sizeof(uint16_t))
	{
		range_interval = *(uint16_t*)buf;

		// Submit event
		struct range_event *range_event = new_range_event();
		range_event->range = range_interval;
		APP_EVENT_SUBMIT(range_event);
	}
	else
	{
		LOG_WRN("Invalid attribute write size, handle: %u, conn: %p", attr->handle, (void *)conn);
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

	return len;
}

static ssize_t uwb_conf_read(struct bt_conn *conn,	const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset)
{
	return bt_gatt_attr_read(conn, attr, buf, len, offset, &uwb_conf, sizeof(uwb_conf));
}

static ssize_t uwb_conf_write(struct bt_conn *conn,	const struct bt_gatt_attr *attr, const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
	//TODO
	return len;
}

static ssize_t mode_conf_read(struct bt_conn *conn,	const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset)
{
	return bt_gatt_attr_read(conn, attr, buf, len, offset, &mode_conf, sizeof(mode_conf));
}

static ssize_t mode_conf_write(struct bt_conn *conn,	const struct bt_gatt_attr *attr, const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
	//TODO
	return len;
}

static ssize_t avg_conf_read(struct bt_conn *conn,	const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset)
{
	return bt_gatt_attr_read(conn, attr, buf, len, offset, &avg_conf, sizeof(avg_conf));
}

static ssize_t avg_conf_write(struct bt_conn *conn,	const struct bt_gatt_attr *attr, const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
	//TODO
	return len;
}

static ssize_t ths_conf_read(struct bt_conn *conn,	const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset)
{
	return bt_gatt_attr_read(conn, attr, buf, len, offset, &threshold_conf, sizeof(threshold_conf));
}

static ssize_t ths_conf_write(struct bt_conn *conn,	const struct bt_gatt_attr *attr, const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
	//TODO
	return len;
}

static ssize_t status_read(struct bt_conn *conn,	const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset)
{
	return bt_gatt_attr_read(conn, attr, buf, len, offset, &status, sizeof(status));
}

static ssize_t control_read(struct bt_conn *conn,	const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset)
{
	return bt_gatt_attr_read(conn, attr, buf, len, offset, &controls, sizeof(controls));
}

static ssize_t control_write(struct bt_conn *conn,	const struct bt_gatt_attr *attr, const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
	//TODO
	return len;
}

/* Service Declaration */
BT_GATT_SERVICE_DEFINE(range_svc,
	BT_GATT_PRIMARY_SERVICE(BT_UUID_RANGE_SVC),
	BT_GATT_CHARACTERISTIC(BT_UUID_RANGE_CHRC,
						   	BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
						   	BT_GATT_PERM_READ,
						   	range_read, NULL, &range),
	BT_GATT_CCC(&range_ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
	BT_GATT_CHARACTERISTIC(BT_UUID_AVG_RANGE_CHRC,
							BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
							BT_GATT_PERM_READ,
							avg_range_read, NULL, NULL),
	BT_GATT_CCC(NULL, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
	BT_GATT_CHARACTERISTIC(BT_UUID_INTERVAL_CONF_CHRC,
							BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP,
							BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
							interval_conf_read, interval_conf_write, &range_interval),
	BT_GATT_CHARACTERISTIC(BT_UUID_UWB_CONF_CHRC,
							BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP,
							BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
							uwb_conf_read, uwb_conf_write, &uwb_conf),
	BT_GATT_CHARACTERISTIC(BT_UUID_MODE_CONF_CHRC,
							BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP,
							BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
							mode_conf_read, mode_conf_write, &mode_conf),
	BT_GATT_CHARACTERISTIC(BT_UUID_AVG_CONF_CHRC,
							BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP,
							BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
							avg_conf_read, avg_conf_write, &avg_conf),
	BT_GATT_CHARACTERISTIC(BT_UUID_THS_CONF_CHRC,
							BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP,
							BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
							ths_conf_read, ths_conf_write, &threshold_conf),
	BT_GATT_CHARACTERISTIC(BT_UUID_STATUS_CHRC,
							BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
							BT_GATT_PERM_READ,
							status_read, NULL, &status),
	BT_GATT_CCC(NULL, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),	
	BT_GATT_CHARACTERISTIC(BT_UUID_CTRL_CHRC,
							BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP,
							BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
							control_read, control_write, &controls),
);


int ble_range_service_init(void)
{
	int err = 0;


	return err;
}


static bool app_event_handler(const struct app_event_header *aeh)
{
	if (is_module_state_event(aeh)) {
		struct module_state_event *event = cast_module_state_event(aeh);

		if (check_state(event, MODULE_ID(ble_state), MODULE_STATE_READY)) {
			static bool initialized;

			__ASSERT_NO_MSG(!initialized);
			initialized = true;

			module_set_state(MODULE_STATE_READY);
			
		}
		return false;
	}

	if (is_range_event(aeh)) {
		struct range_event *event = cast_range_event(aeh);

		range = event->range;	
		if (is_range_active) {
			const struct bt_uuid *uuid = BT_UUID_RANGE_CHRC;
			const struct bt_gatt_attr *attr = &range_svc.attrs[0];

			int err = bt_gatt_notify_uuid(NULL, uuid, attr, &range, sizeof(range));
			if (err == -ENOTCONN) {
				LOG_WRN("Cannot notify. Peer disconnecting.");
			} else if (err) {
				LOG_ERR("GATT notify failed (err=%d)", err);
			}
		}

		return false;
	}
	
	// If event is unhandled, unsubscribe.
	__ASSERT_NO_MSG(false);

	return false;
}

APP_EVENT_LISTENER(MODULE, app_event_handler);
APP_EVENT_SUBSCRIBE(MODULE, range_event);
APP_EVENT_SUBSCRIBE(MODULE, module_state_event);