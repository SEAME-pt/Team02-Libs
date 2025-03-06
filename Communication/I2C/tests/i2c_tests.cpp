
#define CATCH_CONFIG_MAIN
#include <catch2/catch.hpp>
#include "../include/I2C.hpp"
#include <cstdint>
#include <stdexcept>

extern void setMockData(const std::vector<uint8_t>& data);
extern std::vector<uint8_t> getWrittenData();

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


TEST_CASE("I2C readRegister", "[I2C]") {
    I2C i2c;
    i2c.init("/dev/i2c-1");

    uint8_t data[2];
    setMockData({0x34, 0x56});  // Set mock data to be read

    REQUIRE_NOTHROW(i2c.readRegister(0x50, 0x00, data));
    REQUIRE(data[0] == 0x34);
    REQUIRE(data[1] == 0x56);
}