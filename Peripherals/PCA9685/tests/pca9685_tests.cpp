#define CATCH_CONFIG_MAIN
#include <catch2/catch.hpp>
#include "../include/PCA9685.hpp"

TEST_CASE("PCA9685 initialization", "[PCA9685]") {
    auto mockI2C = std::make_shared<I2C>();
    PCA9685 pca9685;
    REQUIRE_NOTHROW(pca9685.init(mockI2C, 0x40));
}

TEST_CASE("PCA9685 setPWMFreq", "[PCA9685]") {
    auto mockI2C = std::make_shared<I2C>();
    PCA9685 pca9685;
    pca9685.init(mockI2C, 0x40);

    REQUIRE_NOTHROW(pca9685.setPWMFreq(1000.0));
}

TEST_CASE("PCA9685 setPWM", "[PCA9685]") {
    auto mockI2C = std::make_shared<I2C>();
    PCA9685 pca9685;
    pca9685.init(mockI2C, 0x40);

    REQUIRE_NOTHROW(pca9685.setPWM(0, 0, 4095));
}

TEST_CASE("PCA9685 setDutyCicle", "[PCA9685]") {
    auto mockI2C = std::make_shared<I2C>();
    PCA9685 pca9685;
    pca9685.init(mockI2C, 0x40);

    REQUIRE_NOTHROW(pca9685.setDutyCicle(0, 2048));
}

TEST_CASE("PCA9685 setGPIO", "[PCA9685]") {
    auto mockI2C = std::make_shared<I2C>();
    PCA9685 pca9685;
    pca9685.init(mockI2C, 0x40);

    REQUIRE_NOTHROW(pca9685.setGPIO(0, 1));
}