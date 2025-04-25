# ISP3x80-Demo

Zephyr implementation of UWB ranging demo using Zephyr RTOS

## Overview

This project is the Zephyr implementation of Insight SiP UWB ranging demo.
This is compatible with ISP3080-UX modules.
For more information go to:
https://www.insightsip.com/products/uwb-aoa-modules

## Build with CLI

Before getting started, make sure you have a proper nRF Connect SDK development environment.
Follow the official
[Getting started guide](https://developer.nordicsemi.com/nRF_Connect_SDK/doc/latest/nrf/getting_started.html).

Then initialize the workspace:

```shell
west init -m https://github.com/insightsip/ISP3x80-Demo-Zephyr --mr main
cd ISP3x80-Demo-Zephyr
west update
```

Build the application for the the board you are using:

```shell
west build -b isp3080_ux_tg -- -DBOARD_ROOT="." -DCONF_FILE="prj_release.conf"
west build -b isp3080_ux_an -- -DBOARD_ROOT="." -DCONF_FILE="prj_release.conf"
```

Flash the device:

```shell
west flash
```

## Build with Visual Studio Code

Before getting started, make sure you have a proper nRF Connect SDK development environment.
Follow the official
[Getting started guide](https://developer.nordicsemi.com/nRF_Connect_SDK/doc/latest/nrf/getting_started.html).

Then clone the project to you workspace:

```shell
git clone https://github.com/insightsip/ISP3x80-Demo-Zephyr.git
```

Open Visual Studio Code then go to **File** -> **Open folder** and select your workspace.  
In nRF Connect extension, **ISP3x80-Demo-Zephyr** should appear as an application.

Click **Add build configuration** and select your target.  
For example choose **isp3080_ux_tg/nrf52833** to build the application for the ISP3080 Tag.  
By default it will build n debug mode. To build as release mode click on **Add Argument** and put

```shell
-- -DCONF_FILE=prj_release.conf
```

After a successful build click on **flash** to program the device.

