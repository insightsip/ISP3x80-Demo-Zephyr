cmake_minimum_required(VERSION 3.20.0)

# The application uses the configuration/<board> scheme for configuration files.
set(APPLICATION_CONFIG_DIR "${CMAKE_CURRENT_LIST_DIR}/configuration/\${NORMALIZED_BOARD_TARGET}")

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(ISP3x80-DEMO)

zephyr_include_directories(
  ${APPLICATION_CONFIG_DIR}
  )


#add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/dwt_uwb_driver)
#target_link_libraries(app PUBLIC uwb_driver)

target_sources(app PRIVATE src/main.c)

add_subdirectory(src/uwb_interface)
add_subdirectory(src/ble)
add_subdirectory(src/modules)
add_subdirectory(src/events)
add_subdirectory(dwt_uwb_driver)