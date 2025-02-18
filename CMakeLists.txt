#
# Copyright 2019-2020 CNRS-UM LIRMM, CNRS-AIST JRL
#

cmake_minimum_required(VERSION 3.1) # Minimum required by recent mc_rtc

project(mc_udp LANGUAGES CXX VERSION 1.0.0)
set(CMAKE_CXX_STANDARD 11)

enable_testing()

include(cmake/cython/cython.cmake)
include(cmake/uninstall.cmake)
_SETUP_PROJECT_UNINSTALL()

option(BUILD_TESTING "Build unit tests" OFF)
option(BUILD_MC_RTC_CLIENT "Build mc_rtc client" ON)
option(BUILD_OPENRTM_SERVER "Build OpenRTM server component" OFF)
option(BUILD_LEGACY_OPENRTM_SERVER "Build OpenRTM server with c++0x" OFF)
option(BUILD_DUMMY_CLIENT "Build dummy client" ON)
option(BUILD_DUMMY_SERVER "Build dummy server" ON)

add_subdirectory(src)

if(PYTHON_BINDING)
  add_subdirectory(binding/python)
endif()

install(
  FILES "${CMAKE_SOURCE_DIR}/mc_udpConfig.cmake"
  DESTINATION lib/cmake/mc_udp
)
