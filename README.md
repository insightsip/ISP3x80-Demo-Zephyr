# ISP3x80-Demo

Zephyr implementation of UWB ranging demo using Zephyr RTOS

## Overview

This project is the Zephyr implementation of Insight SiP UWB ranging demo.
This is compatible with ISP3080-UX and ISP3580-UW modules.
For more information go to:
https://www.insightsip.com/products/uwb-aoa-modules

## Getting started

Before getting started, make sure you have a proper nRF Connect SDK development environment.
Follow the official
[Getting started guide](https://developer.nordicsemi.com/nRF_Connect_SDK/doc/latest/nrf/getting_started.html).

Then initialize the workspace:

```shell
west init -m https://github.com/insightsip/ISP3x80-Demo.git --mr main my-workspace
cd my-workspace
west update
```

Build the application for the the board you are using:

```shell
west build -b isp3080_ux_tg -- -DBOARD_ROOT=. -DCONF_FILE=prj_release.conf
west build -b isp3080_ux_an -- -DBOARD_ROOT=. -DCONF_FILE=prj_release.conf
west build -b isp3580_ux_tg/nrf54l15/cpuapp -- -DBOARD_ROOT=. -DCONF_FILE=prj_release.conf
west build -b isp3580_ux_an/nrf54l15/cpuapp  -- -DBOARD_ROOT=. -DCONF_FILE=prj_release.conf
```

Flash the device:

```shell
west flash
```
