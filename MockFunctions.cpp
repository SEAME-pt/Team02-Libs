#include <cstdint>
#include <vector>
#include <cstring>
#include <stdexcept>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>

static std::vector<uint8_t> mockData;
static std::vector<uint8_t> writtenData;
static int mockFd = 1;
static uint8_t mockI2CAddress = 0; 

extern "C" int custom_i2c_open(const char* path, int flags) {
    (void)path;
    (void)flags;
    return mockFd;
}

extern "C" int custom_i2c_close(int fd) {
    (void)fd;
    return 0;
}

extern "C" int custom_i2c_ioctl(int fd, unsigned long request, uint8_t arg) {
    (void)fd;

    if (request == I2C_SLAVE) {
        mockI2CAddress = arg;
        return 0;
    }
    return -1;
}

extern "C" ssize_t custom_i2c_read(int fd, void* buf, size_t count) {
    (void)fd;

    if (mockData.size() < count) {
        throw std::runtime_error("Not enough mock data available");
    }
    std::memcpy(buf, mockData.data(), count);
    mockData.erase(mockData.begin(), mockData.begin() + count);
    return count;
}

extern "C" ssize_t custom_i2c_write(int fd, const void* buf, size_t count) {
    (void)fd;

    writtenData.insert(writtenData.end(), (uint8_t*)buf, (uint8_t*)buf + count);
    return count;
}

void custom_i2c_setMockData(const std::vector<uint8_t>& data) {
    mockData = data;
}

std::vector<uint8_t> custom_i2c_getWrittenData() {
    return writtenData;
}