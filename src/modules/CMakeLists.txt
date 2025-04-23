target_sources_ifdef(CONFIG_ISP_TWR
		     app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/twr.c)

target_sources_ifdef(CONFIG_ISP_BLE_RANGE_SERVICE
		     app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/ble_range_service.c)

target_sources_ifdef(CONFIG_ISP_BATTERY_MEAS
		     app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/battery_meas.c)

target_sources_ifdef(CONFIG_ISP_MOTION
		     app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/motion.c)

target_sources_ifdef(CONFIG_ISP_LEDS_CTRL
		     app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/leds_ctrl.c)

target_sources_ifdef(CONFIG_ISP_WATCHDOG
		     app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/watchdog.c)

target_include_directories(app PRIVATE .)