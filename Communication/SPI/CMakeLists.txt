cmake_minimum_required(VERSION 3.16)

project(SPI)

add_library(SPI
    src/SPI.cpp
)

target_include_directories(SPI PUBLIC include)