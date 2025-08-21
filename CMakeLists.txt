cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(ISP3x80-DEMO)

# Verify that this samnple is built through sysbuild to ensure MCUboot is
# automatically included and that sample specific MCUboot configurations are
# used when building MCUboot.
test_sysbuild()


#add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/dwt_uwb_driver)
#target_link_libraries(app PUBLIC uwb_driver)

target_sources(app PRIVATE src/main.c)

add_subdirectory(src/uwb_interface)
add_subdirectory(src/ble)
add_subdirectory(src/dfu)
add_subdirectory(src/modules)
add_subdirectory(src/events)
add_subdirectory(dwt_uwb_driver)