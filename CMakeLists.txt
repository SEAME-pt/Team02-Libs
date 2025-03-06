cmake_minimum_required(VERSION 3.16)

project(Libs VERSION 0.1)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_EXE_LINKER_FLAGS "-static")

add_subdirectory(Communication/I2C)
add_subdirectory(Peripherals/INA219)
add_subdirectory(Communication/CAN)
add_subdirectory(Peripherals/PCA9685)

file(GLOB_RECURSE SOURCES
    ./Communication/I2C/src/*.cpp
    ./Peripherals/INA219/src/*.cpp
    ./Communication/CAN/src/*.cpp
    ./Peripherals/PCA9685/src/*.cpp
)

add_library(Libs STATIC ${SOURCES})

target_include_directories(Libs PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/Communication/I2C
    ${CMAKE_CURRENT_SOURCE_DIR}/Peripherals/INA219
    ${CMAKE_CURRENT_SOURCE_DIR}/Communication/CAN
    ${CMAKE_CURRENT_SOURCE_DIR}/Peripherals/PCA9685
)

target_compile_options(Libs PRIVATE -Wall -Werror -Wextra)
