/*
 * Copyright (c) 2022 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/bluetooth/uuid.h>
#include <bluetooth/adv_prov.h>
#include "ble_range_service.h"

static int get_data(struct bt_data *ad, const struct bt_le_adv_prov_adv_state *state, struct bt_le_adv_prov_feedback *fb)
{
	ARG_UNUSED(fb);

	if (!state->pairing_mode) {
		return -ENOENT;
	}

	static const uint8_t data[] = {	
		BT_UUID_RANGE_SVC_VAL,
	};

	ad->type = BT_DATA_UUID128_ALL;
	ad->data_len = sizeof(data);
	ad->data = data;

	return 0;
}

BT_LE_ADV_PROV_SD_PROVIDER_REGISTER(uuid128_all, get_data);