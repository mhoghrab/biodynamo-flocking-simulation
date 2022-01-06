# -----------------------------------------------------------------------------
#
# Copyright (C) 2021 CERN & University of Surrey for the benefit of the
# BioDynaMo collaboration. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
#
# See the LICENSE file distributed with this work for details.
# See the NOTICE file distributed with this work for additional information
# regarding copyright ownership.
#
# -----------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.19.3)
project(flocking_simulation)

# BioDynaMo curretly uses the C++14 standard.
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_MACOSX_RPATH OFF)

# Use BioDynaMo in this project.
find_package(BioDynaMo REQUIRED)

# See UseBioDynaMo.cmake in your BioDynaMo build folder for details.
# Note that BioDynaMo provides gtest header/libraries in its include/lib dir.
include(${BDM_USE_FILE})

## THESE LINES ARE NEW
# Load CNPY
set(CNPY_PATH "/usr/local/include") # PATH to source directore, e.g. cloned git
set(CNPY_LIB "/usr/local/lib") # PATH to installed directory
if(APPLE)
  set(CNPY_LIB_NAME "libcnpy.dylib")
else()
  set(CNPY_LIB_NAME "libcnpy.so")
endif(APPLE)
include_directories(${CNPY_PATH})
add_library(cnpy SHARED IMPORTED)
set_target_properties(cnpy PROPERTIES
  IMPORTED_LOCATION "${CNPY_LIB}/${CNPY_LIB_NAME}"
  INTERFACE_INCLUDE_DIRECTORIES "${CNPY_PATH}"
)

# Consider all files in src/ for BioDynaMo simulation.
include_directories("src")
file(GLOB_RECURSE PROJECT_HEADERS src/*.h)
file(GLOB_RECURSE PROJECT_SOURCES src/*.cc)

bdm_add_executable(${CMAKE_PROJECT_NAME}
                   HEADERS ${PROJECT_HEADERS}
                   SOURCES ${PROJECT_SOURCES}
                   LIBRARIES ${BDM_REQUIRED_LIBRARIES})

target_link_libraries(${CMAKE_PROJECT_NAME} cnpy)
