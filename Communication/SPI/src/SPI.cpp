#include <iostream>
#include <cstring>
#include <fcntl.h>
#include <unistd.h>
#include <linux/spi/spidev.h>
#include <sys/ioctl.h>

#define SPI_DEVICE "/dev/spidev0.0"   // Change based on your system
#define SPI_SPEED 500000             // 500 kHz
#define SPI_BITS_PER_WORD 8          // 8 bits per word
#define SPI_MODE SPI_MODE_0          // SPI Mode 0 (CPOL = 0, CPHA = 0)

int main() {
    int fd; // File descriptor for SPI device

    // Open SPI device
    fd = open(SPI_DEVICE, O_RDWR);
    if (fd < 0) {
        std::cerr << "Failed to open SPI device: " << SPI_DEVICE << std::endl;
        return -1;
    }

    // Configure SPI settings
    uint8_t mode = SPI_MODE_0;
    uint8_t bits = SPI_BITS_PER_WORD;
    uint32_t speed = SPI_SPEED;

    if (ioctl(fd, SPI_IOC_WR_MODE, &mode) < 0 || ioctl(fd, SPI_IOC_RD_MODE, &mode) < 0) {
        std::cerr << "Failed to set SPI mode." << std::endl;
        close(fd);
        return -1;
    }

    if (ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0 || ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &bits) < 0) {
        std::cerr << "Failed to set bits per word." << std::endl;
        close(fd);
        return -1;
    }

    if (ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0 || ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed) < 0) {
        std::cerr << "Failed to set max speed." << std::endl;
        close(fd);
        return -1;
    }

    // Data to send and buffer for received data
    uint8_t tx_buffer[] = {0xAA, 0xBB, 0xCC}; // Example data to send
    uint8_t rx_buffer[sizeof(tx_buffer)] = {0}; // Buffer to store received data

    struct spi_ioc_transfer spi_transfer = {};
    spi_transfer.tx_buf = reinterpret_cast<__u64>(tx_buffer); // Pointer to data to send
    spi_transfer.rx_buf = reinterpret_cast<__u64>(rx_buffer); // Pointer to buffer for received data
    spi_transfer.len = sizeof(tx_buffer);                     // Size of data
    spi_transfer.speed_hz = speed;
    spi_transfer.bits_per_word = bits;
    spi_transfer.delay_usecs = 0; // Optional delay between transfers

    // Perform SPI transaction
    if (ioctl(fd, SPI_IOC_MESSAGE(1), &spi_transfer) < 0) {
        std::cerr << "Failed to perform SPI transaction." << std::endl;
        close(fd);
        return -1;
    }

    // Print received data
    std::cout << "Received data: ";
    for (size_t i = 0; i < sizeof(rx_buffer); ++i) {
        std::cout << std::hex << static_cast<int>(rx_buffer[i]) << " ";
    }
    std::cout << std::dec << std::endl;

    // Close SPI device
    close(fd);

    return 0;
}






#include <linux/spi/spidev.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <iostream>

int main() {
    int fd = open("/dev/spidev0.0", O_RDWR); // Open SPI device
    if (fd < 0) {
        std::cerr << "Failed to open SPI device!" << std::endl;
        return -1;
    }

    // Prepare data
    uint8_t tx_data[2] = {0xAA, 0x55};  // Data to send
    uint8_t rx_data[2] = {0};           // Buffer for received data

    // Configure transfer
    struct spi_ioc_transfer transfer = {};
    transfer.tx_buf = reinterpret_cast<__u64>(tx_data);
    transfer.rx_buf = reinterpret_cast<__u64>(rx_data);
    transfer.len = sizeof(tx_data);
    transfer.speed_hz = 500000;    // 500 kHz
    transfer.bits_per_word = 8;
    transfer.cs_change = 0;        // Keep chip select active

    // Perform the transfer
    int status = ioctl(fd, SPI_IOC_MESSAGE(1), &transfer);
    if (status < 0) {
        std::cerr << "SPI transaction failed!" << std::endl;
    } else {
        std::cout << "Received data: ";
        for (size_t i = 0; i < sizeof(rx_data); ++i) {
            std::cout << std::hex << static_cast<int>(rx_data[i]) << " ";
        }
        std::cout << std::endl;
    }

    close(fd); // Close SPI device
    return 0;
}