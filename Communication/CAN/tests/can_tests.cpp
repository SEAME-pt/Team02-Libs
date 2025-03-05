
#define CATCH_CONFIG_MAIN
#include <catch2/catch.hpp>
#include "../include/CAN.hpp"
#include <cstdint>


TEST_CASE("CAN initialization", "[can]") {
    CAN can;
    REQUIRE_NOTHROW(can.init("/dev/spidev0.0"));
}

TEST_CASE("CAN setSPI", "[can]") {
    CAN can;
    REQUIRE_NOTHROW(can.setSPI());
}

TEST_CASE("CAN setBaudRate", "[can]") {
    CAN can;
    REQUIRE_NOTHROW(can.setBaudRate());
}

TEST_CASE("CAN setMasksFilters", "[can]") {
    CAN can;
    REQUIRE_NOTHROW(can.setMasksFilters());
}

TEST_CASE("CAN configureRxBuffers", "[can]") {
    CAN can;
    REQUIRE_NOTHROW(can.configureRxBuffers());
}

TEST_CASE("CAN configureTxBuffers", "[can]") {
    CAN can;
    REQUIRE_NOTHROW(can.configureTxBuffers());
}

TEST_CASE("CAN setNormalMode", "[can]") {
    CAN can;
    REQUIRE_NOTHROW(can.setNormalMode());
}

TEST_CASE("CAN reset", "[can]") {
    CAN can;
    REQUIRE_NOTHROW(can.reset());
}

TEST_CASE("CAN spiTransfer", "[can]") {
    CAN can;
    uint8_t tx_buffer[3] = {0x01, 0x02, 0x03};
    uint8_t rx_buffer[3] = {0};
    REQUIRE_NOTHROW(can.spiTransfer(tx_buffer, rx_buffer, 3));
}

TEST_CASE("CAN writeRegister", "[can]") {
    CAN can;
    REQUIRE_NOTHROW(can.writeRegister(0x01, 0x02));
}

TEST_CASE("CAN readRegister", "[can]") {
    CAN can;
    REQUIRE_NOTHROW(can.readRegister(0x01));
}

TEST_CASE("CAN readMessage", "[can]") {
    CAN can;
    uint32_t can_id = 0;
    uint8_t data[8] = {0};
    REQUIRE_NOTHROW(can.readMessage(0, can_id, data));
}

TEST_CASE("CAN writeMessage", "[can]") {
    CAN can;
    uint8_t data[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
    REQUIRE_NOTHROW(can.writeMessage(0x123, data, 8));
}

TEST_CASE("CAN checktheReceive", "[can]") {
    CAN can;
    REQUIRE_NOTHROW(can.checktheReceive());
}