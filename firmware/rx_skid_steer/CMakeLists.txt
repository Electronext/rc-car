cmake_minimum_required(VERSION 3.16)
set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/../../components/rc_legacy" "${CMAKE_CURRENT_LIST_DIR}/../../components/rc_protocol" "${CMAKE_CURRENT_LIST_DIR}/../../components/rc_radio")
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(rc_rx_skid_steer)
