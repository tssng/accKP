cmake_minimum_required(VERSION 3.31)
project(accKP)

set(CMAKE_CXX_STANDARD 20)

# dependencies
set(EIGEN3_INCLUDE_DIR "C:/libs/eigen-master")
find_package(GTSAM REQUIRED)
find_package(FlatBuffers REQUIRED)

# flatbuffer logic
set(SCHEMA_FILE "${CMAKE_CURRENT_SOURCE_DIR}/packet.fbs")
set(GENERATED_DIR "${CMAKE_CURRENT_BINARY_DIR}/generated")
file(MAKE_DIRECTORY ${GENERATED_DIR})

add_custom_command(
        OUTPUT "${GENERATED_DIR}/packet_generated.h"
        COMMAND flatbuffers::flatc -c -o "${GENERATED_DIR}" "${SCHEMA_FILE}"
        DEPENDS "${SCHEMA_FILE}"
        COMMENT "Generating FlatBuffers header from ${SCHEMA_FILE}"
)

# get main executable
add_executable(accKP
        main.cpp
        SingleSolver.hpp
        SensorParser.hpp
        graph_types.hpp
        SafeQueue.hpp
        "${GENERATED_DIR}/packet_generated.h" # Add this so CLion tracks it
)

# link/set include paths
target_include_directories(accKP PRIVATE
        ${EIGEN3_INCLUDE_DIR}
        ${GENERATED_DIR}
)

target_link_libraries(accKP PRIVATE
        gtsam
        flatbuffers::flatbuffers
)