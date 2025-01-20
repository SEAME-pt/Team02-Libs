/**
 * @file PCA9685.cpp
 * @brief Implementation file for the PCA9685 class, providing functions to control the PCA9685 PWM driver.
 *
 * This file contains the implementation of methods to initialize the PCA9685 device, set PWM frequency,
 * configure PWM signals for specific channels, and control GPIO states using the I2C protocol.
 */

#include "../include/PCA9685.hpp"

/**
 * @brief Default constructor for the PCA9685 class.
 */
PCA9685::PCA9685() {}

/**
 * @brief Destructor for the PCA9685 class.
 * Resets the PCA9685 module by turning off all LEDs and clearing settings.
 */
PCA9685::~PCA9685()
{
    _i2c->writeByte(_deviceAddress, MODE1, 0x00);
    _i2c->writeByte(_deviceAddress, ALL_LED_ON_L, 0x00);
    _i2c->writeByte(_deviceAddress, ALL_LED_ON_H, 0x00);
    _i2c->writeByte(_deviceAddress, ALL_LED_OFF_L, 0x00);
    _i2c->writeByte(_deviceAddress, ALL_LED_OFF_H, 0x10);
}

/**
 * @brief Copy constructor for the PCA9685 class.
 * Currently performs no operations as copying is not implemented.
 *
 * @param originalPCA9685 Reference to the original PCA9685 object to copy.
 */
PCA9685::PCA9685(const PCA9685& originalPCA9685)
{
    (void)originalPCA9685;
}

/**
 * @brief Copy assignment operator for the PCA9685 class.
 * Currently performs no operations as copying is not implemented.
 *
 * @param originalPCA9685 Reference to the original PCA9685 object to assign from.
 * @return Reference to the assigned PCA9685 object.
 */
PCA9685& PCA9685::operator=(const PCA9685& originalPCA9685)
{
    (void)originalPCA9685;
    return *this;
}

/**
 * @brief Initializes the PCA9685 device.
 * Configures the device with the I2C address and resets all channels.
 *
 * @param m_i2c Pointer to the I2C object for communication.
 * @param deviceAddress I2C address of the PCA9685 module.
 */
void PCA9685::init(I2C* m_i2c, uint8_t deviceAddress)
{
    this->_i2c           = m_i2c;
    this->_deviceAddress = deviceAddress;
    _i2c->writeByte(_deviceAddress, MODE1, 0x00);
    _i2c->writeByte(_deviceAddress, ALL_LED_ON_L, 0x00);
    _i2c->writeByte(_deviceAddress, ALL_LED_ON_H, 0x00);
    _i2c->writeByte(_deviceAddress, ALL_LED_OFF_L, 0x00);
    _i2c->writeByte(_deviceAddress, ALL_LED_OFF_H, 0x10);
}

/**
 * @brief Sets the PWM frequency for the PCA9685 module.
 * Configures the prescaler based on the desired frequency.
 *
 * @param freq_hz Desired frequency in Hz.
 */
void PCA9685::setPWMFreq(float freq_hz)
{
    uint8_t prescale =
        static_cast<uint8_t>((25000000.0 / (4096 * freq_hz)) - 1.0);
    _i2c->writeByte(_deviceAddress, MODE1, 0x10);
    _i2c->writeByte(_deviceAddress, PRE_SCALE, prescale);
    _i2c->writeByte(_deviceAddress, MODE1, 0x80);
    usleep(5000);
}

/**
 * @brief Configures PWM settings for a specific channel.
 * Sets the ON and OFF times for the PWM signal on a given channel.
 *
 * @param channel Channel number (0-15).
 * @param on Time step at which the PWM signal turns ON.
 * @param off Time step at which the PWM signal turns OFF.
 */
void PCA9685::setPWM(uint8_t channel, uint16_t on, uint16_t off)
{
    _i2c->writeByte(_deviceAddress, LED0_ON_L + 4 * channel, on & 0xFF);
    _i2c->writeByte(_deviceAddress, LED0_ON_H + 4 * channel, on >> 8);
    _i2c->writeByte(_deviceAddress, LED0_OFF_L + 4 * channel, off & 0xFF);
    _i2c->writeByte(_deviceAddress, LED0_OFF_H + 4 * channel, off >> 8);
}

/**
 * @brief Sets the duty cycle for a specific channel.
 * Calculates the PWM OFF time based on the pulse width.
 *
 * @param channel Channel number (0-15).
 * @param pulseWidth Pulse width to set the duty cycle.
 */
void PCA9685::setDutyCicle(uint8_t channel, uint16_t pulseWidth)
{
    this->setPWM(channel, 0, pulseWidth % 0x1000);
}

/**
 * @brief Sets a GPIO output state for a specific channel.
 * Configures the channel as ON or OFF.
 *
 * @param channel Channel number (0-15).
 * @param on State of the GPIO pin (1 for ON, 0 for OFF).
 */
void PCA9685::setGPIO(uint8_t channel, uint16_t on)
{
    this->setPWM(channel, 0x1000 * on, 0x1000 * !on);
}
