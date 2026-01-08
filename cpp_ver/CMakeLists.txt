cmake_minimum_required(VERSION 3.8)
project(railway_pager_demod)

set(CMAKE_CXX_STANDARD 14)

include_directories(.)
include_directories(
        dsp
        util
        fcdlib
)

add_executable(railway_pager_demod
        railway_pager_demod.cpp
        railway_pager_demod.h
        demod.cpp
        demod.h
        dsp/firfilter.cpp
)

target_link_libraries(railway_pager_demod
        m
)

add_definitions(-DSDRBASE_API=)
