cmake_minimum_required(VERSION 3.16)
set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/../../components/rc_legacy")
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(rc_transmitter)
