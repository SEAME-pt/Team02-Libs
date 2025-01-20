/**
 * @file INA219.cpp
 * @brief Implementation of the INA219 class, used for interfacing with the INA219 current sensor.
 *
 * This file contains the implementation of the functions declared in the INA219.hpp file,
 * including initialization, setting the I2C address, and reading voltage values.
 */

#include "../include/INA219.hpp"

/**
 * @brief Default constructor for the INA219 class.
 */
INA219::INA219()
{

}

/**
 * @brief Destructor for the INA219 class.
 */
INA219::~INA219()
{

}

/**
 * @brief Copy constructor for the INA219 class.
 *
 * @param originalINA219 Reference to the original INA219 object to copy.
 */
INA219::INA219(const INA219& originalINA219)
{
    (void)originalINA219;
}

/**
 * @brief Copy assignment operator for the INA219 class.
 *
 * @param originalINA219 Reference to the original INA219 object to assign from.
 * @return Reference to the assigned INA219 object.
 */
INA219& INA219::operator=(const INA219& originalINA219)
{
    (void)originalINA219;
    return *this;
}

/**
 * @brief Initializes the INA219 sensor.
 *
 * Sets up the I2C communication object and configures the device address.
 *
 * @param m_i2c Pointer to the I2C object used for communication.
 * @param deviceAddress The I2C address of the INA219 sensor.
 * @throw std::runtime_error If setting the I2C address fails.
 */
void INA219::init(I2C* m_i2c, uint8_t deviceAddress)
{
    this->_i2c = m_i2c;
    this->_deviceAddress = deviceAddress;
    if (ioctl(this->_i2c->getFd(), I2C_SLAVE, deviceAddress) < 0)
    {
        throw std::runtime_error("Failed to set I2C address");
    }
}

/**
 * @brief Reads the bus voltage from the specified register on the INA219 sensor.
 *
 * This function reads two bytes from the specified register address and converts
 * the raw data into a voltage value in volts.
 *
 * @param registerAddr The address of the register to read from.
 * @return The bus voltage in volts as a double.
 */
double INA219::readVoltage(uint8_t registerAddr)
{
    uint8_t buf[2] = {0};
    _i2c->readRegister(_deviceAddress, registerAddr, buf);
    uint16_t rawVoltage = (buf[0] << 8) | buf[1];
    rawVoltage >>= 3;
    double busVoltage = rawVoltage * 0.004;
    std::cout << busVoltage << std::endl;

    return busVoltage;
}
