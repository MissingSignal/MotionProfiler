# Copyright: (C) 2014 RBCS Robotics Brain and Cognitive Science
# Authors: Francesco Rea, Giorgio Metta, Omar Eldardeer
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 3.12)

project (motionProfiler)

find_package(ICUBcontrib REQUIRED)
#find_package(IPOPT)
list(APPEND CMAKE_MODULE_PATH ${ICUBCONTRIB_MODULE_PATH})

include(ICUBcontribHelpers)
include(ICUBcontribOptions)

icubcontrib_set_default_prefix() #set CMAKE_INSTALL_PREFIX to the ICUBcontrib one; print warning if user modifies it


# default prefix for applications
# set(ICUB_APPLICATIONS_PREFIX "$ENV{ICUB_ROOT}" CACHE PATH "Application path prefix")

add_subdirectory(src)
add_subdirectory(app)

icubcontrib_set_default_prefix()
icubcontrib_add_uninstall_target()        
