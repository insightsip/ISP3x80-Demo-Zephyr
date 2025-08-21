#
# SPDX-FileCopyrightText: Copyright (c) 2024 Qorvo US, Inc.
# SPDX-License-Identifier: LicenseRef-QORVO-2
#
cmake_minimum_required(VERSION 3.20.0)
 
target_sources(app PRIVATE dfu_module.c)

target_include_directories(app PRIVATE .)

