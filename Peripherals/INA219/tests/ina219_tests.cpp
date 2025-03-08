#define CATCH_CONFIG_MAIN
#include <catch2/catch.hpp>
#include "../include/INA219.hpp"

extern void custom_i2c_setMockData(const std::vector<uint8_t>& data);
extern std::vector<uint8_t> custom_ina219_getWrittenData();

TEST_CASE("INA219 initialization", "[INA219]") {
    I2C mockI2C;
    INA219 ina219;
    REQUIRE_NOTHROW(ina219.init(&mockI2C, 0x40));
}

TEST_CASE("INA219 readVoltage", "[INA219]") {
    I2C mockI2C;
    INA219 ina219;
    ina219.init(&mockI2C, 0x40);

    custom_i2c_setMockData({0x01, 0x23});  // Set mock data to be read

    double voltage = 0.0;
    REQUIRE_NOTHROW(voltage = ina219.readVoltage(0x02));
    //REQUIRE(voltage == Approx(0.291).epsilon(0.001));  // Example expected voltage
}