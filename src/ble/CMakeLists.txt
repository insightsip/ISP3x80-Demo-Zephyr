#
# SPDX-FileCopyrightText: Copyright (c) 2024 Qorvo US, Inc.
# SPDX-License-Identifier: LicenseRef-QORVO-2
#
cmake_minimum_required(VERSION 3.20.0)
 
target_sources(app PRIVATE ble_adv_prov_uuid128.c)
target_sources(app PRIVATE ble_dis_prov_version.c)

target_include_directories(app PRIVATE .)

