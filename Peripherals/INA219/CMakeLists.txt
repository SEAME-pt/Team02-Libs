cmake_minimum_required(VERSION 3.16)

project(INA219)

add_library(INA219
    src/INA219.cpp
)

target_include_directories(INA219 PUBLIC include)