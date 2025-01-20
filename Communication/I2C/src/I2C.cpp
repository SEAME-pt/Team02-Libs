
/**
 * @file I2C.cpp
 * @brief Implementation file for the I2C class.
 * 
 * This file contains the implementation of methods declared in the I2C.hpp file.
 * The methods include initializing the I2C bus, writing bytes and messages to
 * specific devices, and reading data from registers.
 */

#include "../include/I2C.hpp"

/**
 * @brief Default constructor for the I2C class.
 * Initializes an I2C object with default values.
 */
I2C::I2C() {}

/**
 * @brief Destructor for the I2C class.
 * Closes the I2C file descriptor if it is open.
 */
I2C::~I2C()
{
    close(this->_i2cFd);
}

/**
 * @brief Copy constructor for the I2C class.
 * Copies the file descriptor and device name from the original object.
 *
 * @param originalAddress Reference to the original I2C object to copy.
 */
I2C::I2C(const I2C& originalAddress)
{
    this->_i2cFd     = originalAddress._i2cFd;
    this->_i2cDevice = originalAddress._i2cDevice;
}

/**
 * @brief Copy assignment operator for the I2C class.
 * Copies the file descriptor and device name from the original object.
 *
 * @param originalAddress Reference to the original I2C object to copy.
 * @return Reference to the assigned object.
 */
I2C& I2C::operator=(const I2C& originalAddress)
{
    if (this != &originalAddress)
    {
        this->_i2cFd     = originalAddress._i2cFd;
        this->_i2cDevice = originalAddress._i2cDevice;
    }
    return *this;
}

/**
 * @brief Gets the file descriptor of the I2C device.
 *
 * @return File descriptor of the I2C device.
 */
int I2C::getFd()
{
    return _i2cFd;
}

/**
 * @brief Initializes the I2C device.
 * Opens the specified I2C device and sets the file descriptor.
 *
 * @param i2cDevice Path to the I2C device (e.g., "/dev/i2c-1").
 * @throws std::runtime_error If the I2C device cannot be opened.
 */
void I2C::init(const std::string& i2cDevice)
{
    this->_i2cDevice = i2cDevice;
    this->_i2cFd     = open(i2cDevice.c_str(), O_RDWR);
    if (this->_i2cFd < 0)
    {
        throw std::runtime_error("Failed to open the I2C device");
    }
}

/**
 * @brief Writes a single byte to the specified I2C device and register.
 *
 * @param deviceAddress Address of the I2C device.
 * @param reg Register to write to.
 * @param value Value to write to the register.
 * @throws std::runtime_error If setting the device address or writing fails.
 */
void I2C::writeByte(uint8_t deviceAddress, uint8_t reg, uint8_t value)
{
    if (ioctl(this->_i2cFd, I2C_SLAVE, deviceAddress) < 0)
    {
        throw std::runtime_error("Failed to set I2C address");
    }

    uint8_t buffer[2] = {reg, value};
    if (write(this->_i2cFd, buffer, 2) != 2)
    {
        throw std::runtime_error("Failed to write to the I2C device");
    }
}

/**
 * @brief Sends a message to the I2C device.
 *
 * @param deviceAddress Address of the I2C device.
 * @param buffer Pointer to the buffer containing the message.
 * @throws std::runtime_error If setting the device address or writing fails.
 */
void I2C::writeMessage(uint8_t deviceAddress, uint8_t *buffer)
{
    if (ioctl(this->_i2cFd, I2C_SLAVE, deviceAddress) < 0)
    {
        throw std::runtime_error("Failed to set I2C address");
    }
    if (write(this->_i2cFd, buffer, sizeof(buffer)) == -1)
    {
        throw std::runtime_error("Failed to write to the I2C device");
    }
}

/**
 * @brief Reads a single byte from the specified I2C device.
 *
 * @param deviceAddress Address of the I2C device.
 * @return The byte read from the I2C device.
 * @throws std::runtime_error If setting the device address or reading fails.
 */
uint8_t I2C::readByte(uint8_t deviceAddress)
{
    if (ioctl(this->_i2cFd, I2C_SLAVE, deviceAddress) < 1)
    {
        throw std::runtime_error("Failed to set I2C address");
    }
    uint8_t value;
    if (read(this->_i2cFd, &value, 1) != 1)
    {
        throw std::runtime_error("Failed to read from the I2C device");
    }
    return value;
}

/**
 * @brief Reads data from a specific register of the I2C device.
 *
 * @param deviceAddress Address of the I2C device.
 * @param registerAddr Register address to read from.
 * @param data Pointer to store the data read from the register.
 * @throws std::runtime_error If writing the register address or reading the data fails.
 */
void I2C::readRegister(uint8_t deviceAddress, uint8_t registerAddr, uint8_t *data)
{
    if (write(this->_i2cFd, &registerAddr, sizeof(registerAddr)) == -1)
    {
        throw std::runtime_error("Failed to write to the I2C device");
    }
    if (read(this->_i2cFd, data, 2) != 2)
    {
        throw std::runtime_error("Failed to read from the I2C device");
    }
}
