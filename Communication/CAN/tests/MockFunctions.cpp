#include <cstdint>
#include <vector>
#include <cstring>
#include <stdexcept>

static std::vector<uint8_t> mockData;
static std::vector<uint8_t> writtenData;
static int mockFd = 1;

extern "C" int custom_open(const char* path, int flags) {
    (void)path;
    (void)flags;
    return mockFd;
}

extern "C" int custom_close(int fd) {
    (void)fd;
    return 0;
}

extern "C" int custom_ioctl(int fd, unsigned long request, void* arg) {
    (void)fd;
    (void)request;
    (void)arg;
    return 0;
}

extern "C" ssize_t custom_read(int fd, void* buf, size_t count) {
    (void)fd;

    if (mockData.size() < count) {
        throw std::runtime_error("Not enough mock data available");
    }
    std::memcpy(buf, mockData.data(), count);
    mockData.erase(mockData.begin(), mockData.begin() + count);
    return count;
}

extern "C" ssize_t custom_write(int fd, const void* buf, size_t count) {
    (void)fd;

    writtenData.insert(writtenData.end(), (uint8_t*)buf, (uint8_t*)buf + count);
    return count;
}

void setMockData(const std::vector<uint8_t>& data) {
    mockData = data;
}

std::vector<uint8_t> getWrittenData() {
    return writtenData;
}