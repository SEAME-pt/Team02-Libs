cmake_minimum_required(VERSION 3.16)

project(Libs VERSION 0.1)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_EXE_LINKER_FLAGS "-static")

include_directories(include)

add_subdirectory(Communication/I2C)
add_subdirectory(Peripherals/INA219)
add_subdirectory(Communication/SPI)
add_subdirectory(Communication/CAN)
add_subdirectory(Peripherals/PCA9685)

file(GLOB_RECURSE SOURCES
    ./Communication/I2C/*.cpp
    ./Peripherals/INA219/*.cpp
    ./Communication/CAN/*.cpp
    ./Communication/SPI/*.cpp
    ./Peripherals/PCA9685/*.cpp
)

add_library(Libs STATIC ${SOURCES})

target_include_directories(Libs PUBLIC
    Communication/I2C
    Peripherals/INA219
    Communication/CAN
    Communication/SPI
    Peripherals/PCA9685
)

target_compile_options(Libs PRIVATE -Wall -Werror -Wextra)
