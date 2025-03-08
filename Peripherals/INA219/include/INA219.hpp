#pragma once

#include <iostream>
#include <cstdint>
#include "../../../Communication/I2C/include/I2C.hpp"

#define INA_ADDRESS 0x41

#ifdef TEST_MODE
  // Declare your custom functions
    extern "C" int custom_i2c_ioctl(int fd, unsigned long request, uint8_t arg);
#endif

class INA219 {

    private:
        int _deviceAddress;
        I2C* _i2c;

    public:
        INA219();
        ~INA219();
        INA219(const INA219& originalINA219);
        INA219& operator=(const INA219& originalINA219);

        void init(I2C* m_i2c, uint8_t deviceAddress);
        double readVoltage(uint8_t registerAddr);

};
