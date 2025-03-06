cmake_minimum_required(VERSION 3.16)

project(I2C)

add_library(I2C
    src/I2C.cpp
    tests/MockFunctions.cpp
)

target_include_directories(I2C PUBLIC include)