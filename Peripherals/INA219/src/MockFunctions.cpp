#include <cstdint>
#include <vector>
#include <cstring>
#include <stdexcept>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>

static std::vector<uint8_t> mockData;
static std::vector<uint8_t> writtenData;
static uint8_t mockI2CAddress = 0; 

extern "C" int custom_i2c_ioctl(int fd, unsigned long request, uint8_t arg) {
    (void)fd;
    if (request == I2C_SLAVE) {
        mockI2CAddress = arg;
        return 0;
    }
    return -1;
}

void custom_ina219_setMockData(const std::vector<uint8_t>& data) {
    mockData = data;
}

std::vector<uint8_t> custom_ina219_getWrittenData() {
    return writtenData;
}