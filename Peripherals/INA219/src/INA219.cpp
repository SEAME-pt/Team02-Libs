#include "../include/INA219.hpp"

INA219::INA219()
{

}

INA219::~INA219()
{

}

INA219::INA219(const INA219& originalINA219)
{
    (void)originalINA219;
}

INA219& INA219::operator=(const INA219& originalINA219)
{
    (void)originalINA219;
    return *this;
}

void INA219::init(I2C* m_i2c, uint8_t deviceAddress)
{
    this->_i2c = m_i2c;
    this->_deviceAddress = deviceAddress;
    if (ioctl(this->_i2c->getFd(), I2C_SLAVE, deviceAddress) < 0)
    {
        throw std::runtime_error("Failed to set I2C address");
    }
}

double INA219::readVoltage(uint8_t registerAddr)
{
    
    uint8_t buf[2] = {0};
    _i2c->readRegister(_deviceAddress, registerAddr, buf);
    uint16_t rawVoltage = (buf[0] << 8) | buf[1];
    rawVoltage >>= 3;
    double busVoltage = rawVoltage * 0.004;
    // std::cout << busVoltage << std::endl;

    return busVoltage;
}