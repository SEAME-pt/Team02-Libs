/**
 * @file INA219.hpp
 * @brief Header file for the INA219 class, providing an interface to communicate with the INA219 current sensor.
 * 
 * The INA219 class facilitates initializing the sensor, configuring its I2C address,
 * and reading voltage values from specific registers.
 */

#pragma once

#include <iostream>
#include <cstdint>
#include "../../../Communication/I2C/include/I2C.hpp"

#define INA_ADDRESS 0x41 ///< Default I2C address for the INA219 sensor

/**
 * @class INA219
 * @brief A class for interfacing with the INA219 current sensor using I2C.
 */
class INA219 {

    private:
        int _deviceAddress; ///< I2C device address for the INA219 sensor
        I2C* _i2c;         ///< Pointer to the I2C communication object

    public:

        /**
         * @brief Default constructor for the INA219 class.
         */
        INA219();

        /**
         * @brief Destructor for the INA219 class.
         */
        ~INA219();

        /**
         * @brief Copy constructor for the INA219 class.
         * @param originalINA219 Reference to the original INA219 object to copy.
         */
        INA219(const INA219& originalINA219);

        /**
         * @brief Copy assignment operator for the INA219 class.
         * @param originalINA219 Reference to the original INA219 object to assign from.
         * @return Reference to the assigned INA219 object.
         */
        INA219& operator=(const INA219& originalINA219);

        /**
         * @brief Initializes the INA219 sensor.
         * Sets up the I2C communication and stores the device address.
         *
         * @param m_i2c Pointer to the I2C object used for communication.
         * @param deviceAddress I2C address of the INA219 sensor.
         */
        void init(I2C* m_i2c, uint8_t deviceAddress);

        /**
         * @brief Reads a voltage value from a specified register on the INA219 sensor.
         *
         * @param registerAddr Address of the register to read from.
         * @return The voltage value read from the specified register as a double.
         */
        double readVoltage(uint8_t registerAddr);

};
