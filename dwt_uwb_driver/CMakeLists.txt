#
# SPDX-FileCopyrightText: Copyright (c) 2024 Qorvo US, Inc.
# SPDX-License-Identifier: LicenseRef-QORVO-2
#
cmake_minimum_required(VERSION 3.13.1)

project(dw3xxx)

target_sources(app PRIVATE deca_compat.c)
target_sources(app PRIVATE deca_interface.c)
target_sources(app PRIVATE deca_rsl.c)
target_sources(app PRIVATE dw3000/dw3000_device.c)
target_sources(app PRIVATE dw3720/dw3720_device.c)
target_sources(app PRIVATE lib/qmath/src/qmath.c)
zephyr_library_include_directories(.)
zephyr_library_include_directories(./dw3000)
zephyr_library_include_directories(./dw3720)
zephyr_library_include_directories(./lib/qmath/include)