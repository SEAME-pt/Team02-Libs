/**
 * @file PCA9685.hpp
 * @brief Header file for the PCA9685 class, providing an interface for PWM control using the PCA9685 module.
 *
 * The PCA9685 class utilizes an I2C communication interface to control PWM signals
 * across multiple channels. It allows setting PWM frequency, duty cycle, and GPIO states
 * for connected devices.
 */

#pragma once

#include <iostream>
#include <cstdint>
#include "../../../Communication/I2C/include/I2C.hpp"

// PCA9685 Register Definitions
#define MODE1 0x00           ///< Mode register 1 address
#define PRE_SCALE 0xFE       ///< Prescale register address
#define LED0_ON_L 0x06       ///< LED0 ON low byte
#define LED0_ON_H 0x07       ///< LED0 ON high byte
#define LED0_OFF_L 0x08      ///< LED0 OFF low byte
#define LED0_OFF_H 0x09      ///< LED0 OFF high byte
#define ALL_LED_ON_L 0xFA    ///< All LEDs ON low byte
#define ALL_LED_ON_H 0xFB    ///< All LEDs ON high byte
#define ALL_LED_OFF_L 0xFC   ///< All LEDs OFF low byte
#define ALL_LED_OFF_H 0xFD   ///< All LEDs OFF high byte

/**
 * @class PCA9685
 * @brief A class for controlling the PCA9685 16-channel PWM driver via I2C.
 *
 * This class provides methods to initialize the PCA9685 module, set PWM frequency,
 * configure individual channel PWM signals, and set GPIO states.
 */
class PCA9685
{
  private:
    int _deviceAddress;  ///< I2C address of the PCA9685 module
    I2C* _i2c;           ///< Pointer to the I2C communication object

  public:
    /**
     * @brief Default constructor for the PCA9685 class.
     */
    PCA9685();

    /**
     * @brief Destructor for the PCA9685 class.
     * Releases any resources associated with the object.
     */
    ~PCA9685();

    /**
     * @brief Copy constructor for the PCA9685 class.
     * @param originalPCA9685 The original PCA9685 object to copy.
     */
    PCA9685(const PCA9685& originalPCA9685);

    /**
     * @brief Copy assignment operator for the PCA9685 class.
     * @param originalPCA9685 The original PCA9685 object to assign from.
     * @return Reference to the assigned PCA9685 object.
     */
    PCA9685& operator=(const PCA9685& originalPCA9685);

    /**
     * @brief Initializes the PCA9685 module.
     * @param m_i2c Pointer to the I2C object for communication.
     * @param deviceAddress I2C address of the PCA9685 module.
     */
    void init(I2C* m_i2c, uint8_t deviceAddress);

    /**
     * @brief Sets the PWM frequency for the PCA9685 module.
     * @param freq_hz Desired frequency in Hz.
     */
    void setPWMFreq(float freq_hz);

    /**
     * @brief Configures PWM settings for a specific channel.
     * @param channel Channel number (0-15).
     * @param on Time step at which the PWM signal turns ON.
     * @param off Time step at which the PWM signal turns OFF.
     */
    void setPWM(uint8_t channel, uint16_t on, uint16_t off);

    /**
     * @brief Sets the duty cycle for a specific channel.
     * @param channel Channel number (0-15).
     * @param pulseWidth Pulse width for the desired duty cycle.
     */
    void setDutyCicle(uint8_t channel, uint16_t pulseWidth);

    /**
     * @brief Sets a GPIO output state for a specific channel.
     * @param channel Channel number (0-15).
     * @param on State of the GPIO pin (1 for ON, 0 for OFF).
     */
    void setGPIO(uint8_t channel, uint16_t on);
};
