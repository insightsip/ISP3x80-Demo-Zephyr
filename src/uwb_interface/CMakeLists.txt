cmake_minimum_required(VERSION 3.20.0)
  
target_sources(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/dw_sleep.c)
target_sources(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/dw_spi.c)
target_sources(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/dw_mutex.c)
target_sources(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/dw_hw.c)
target_sources(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/dw_probe_interface.c)

target_include_directories(app PRIVATE .)

