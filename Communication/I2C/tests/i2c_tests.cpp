
#define CATCH_CONFIG_MAIN
#include <catch2/catch.hpp>
#include "../include/I2C.hpp"
#include <cstdint>
#include <stdexcept>


TEST_CASE("I2C initialization", "[i2c]") {
    I2C i2c;
    REQUIRE_NOTHROW(i2c.init("/dev/i2c-1"));
}

TEST_CASE("I2C getFd", "[i2c]") {
    I2C i2c;
    i2c.init("/dev/i2c-1");
    REQUIRE(i2c.getFd() >= 0);
}

TEST_CASE("I2C writeByte", "[i2c]") {
    I2C i2c;
    i2c.init("/dev/i2c-1");
    REQUIRE_NOTHROW(i2c.writeByte(0x50, 0x00, 0xFF));
}

TEST_CASE("I2C writeMessage", "[i2c]") {
    I2C i2c;
    i2c.init("/dev/i2c-1");
    uint8_t buffer[2] = {0x00, 0xFF};
    REQUIRE_NOTHROW(i2c.writeMessage(0x50, buffer));
}

TEST_CASE("I2C readByte", "[i2c]") {
    I2C i2c;
    i2c.init("/dev/i2c-1");
    REQUIRE_NOTHROW(i2c.readByte(0x50));
}

TEST_CASE("I2C readRegister", "[i2c]") {
    I2C i2c;
    i2c.init("/dev/i2c-1");
    uint8_t data[2];
    REQUIRE_NOTHROW(i2c.readRegister(0x50, 0x00, data));
}