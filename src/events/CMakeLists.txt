cmake_minimum_required(VERSION 3.20.0)

target_sources(app PRIVATE
	       ${CMAKE_CURRENT_SOURCE_DIR}/battery_event.c
	       ${CMAKE_CURRENT_SOURCE_DIR}/twr_event.c
	       ${CMAKE_CURRENT_SOURCE_DIR}/config_event.c
		   ${CMAKE_CURRENT_SOURCE_DIR}/motion_event.c
)

target_include_directories(app PRIVATE .)

