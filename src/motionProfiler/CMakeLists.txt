# Copyright (C) 2021 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
# Authors: Luca Garello
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

SET(KEYWORD "motionProfiler")
PROJECT(${KEYWORD})
cmake_minimum_required(VERSION 3.12)

find_package(YARP REQUIRED)
find_package(YCM REQUIRED)
find_package(ICUBcontrib REQUIRED)

list(APPEND CMAKE_MODULE_PATH ${ICUBCONTRIB_MODULE_PATH})


set(ICUB_CONTRIB_DIRS $ENV{ICUB_DIR}/include)

INCLUDE_DIRECTORIES(
    ${PROJECT_SOURCE_DIR}/include
    ${ICUB_CONTRIB_DIRS}
    ${iKin_INCLUDE_DIRS}
)

# Search for source code.
FILE(GLOB folder_source src/*.cpp src/*.cc src/*.c)
FILE(GLOB folder_header include/iCub/*.h)
SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

# Set up the main executable.
IF (folder_source)
    ADD_EXECUTABLE(${KEYWORD} 
        ${folder_source} 
        ${folder_header}
    )

    TARGET_LINK_LIBRARIES(${KEYWORD}        
      ${YARP_LIBRARIES}
      ${IPP_LIBRARIES}
      )	

    INSTALL_TARGETS(/bin ${KEYWORD})
ELSE (folder_source)
    MESSAGE(FATAL_ERROR "No source code files found. Please add something")
ENDIF (folder_source)

