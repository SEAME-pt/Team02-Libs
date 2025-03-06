cmake_minimum_required(VERSION 3.16)

project(CAN)

add_library(CAN
    src/CAN.cpp
    tests/MockFunctions.cpp
)

# target_compile_definitions(CAN PUBLIC TEST_MODE)

target_include_directories(CAN PUBLIC include)