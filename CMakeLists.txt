cmake_minimum_required(VERSION 3.31)
project(accKP)

set(CMAKE_CXX_STANDARD 20)
set(EIGEN3_INCLUDE_DIR "C:/libs/eigen-master")

find_package(GTSAM REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})

# add_library(ParserLib
#        SensorParser.hpp
#        graph_types.hpp)

add_executable(accKP main.cpp
        SingleSolver.hpp
        SensorParser.hpp
        graph_types.hpp
        SafeQueue.hpp)
target_link_libraries(accKP PRIVATE gtsam)