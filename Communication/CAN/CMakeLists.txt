cmake_minimum_required(VERSION 3.16)

project(CAN)

add_library(CAN
    src/CAN.cpp
)

target_include_directories(CAN PUBLIC include)