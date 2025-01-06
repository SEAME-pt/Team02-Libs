# JetRacer Peripherals and Libraries Repository

This repository serves as a centralized location to document and manage all peripherals, libraries, and related code integrated into the JetRacer project.

## Purpose

The goal of this repository is to:
- Keep track of all peripherals added to the JetRacer, including sensors, cameras, and other hardware components.
- Store and document libraries for communication protocols.
- Maintain code associated with each peripheral for easier management and collaboration.

---

## Current Peripherals

### 1. Speed Sensor
- **Description**: Measures the vehicle's real-time speed and transmits the data to the system.
- **Hardware**: Arduino-based speed sensor.
- **Communication Protocol with Jetson Nano**: CAN Bus.
- **Code**: The Arduino code for processing and transmitting speed data is located in the [`Pheripherals/speed-sensor`](./Pheripherals/speed-sensor) directory.
- **Setup**: 
  - Connect the speed sensor to the Arduino as per the wiring diagram (included in the `Pheripherals/speed-sensor` directory).
  - Upload the provided code to the Arduino.

---

## Libraries

### 1. I2C Library
- **Description**: A library for interfacing with devices over the I2C communication protocol.
- **Location**: [`/Communication/I2C`](./libraries/I2C).
- **Usage**: Include the library in your project to communicate with I2C-compatible peripherals.

### 2. CAN Bus Library
- **Description**: A library for communication using the CAN protocol, enabling data exchange between devices like the Jetson Nano and the arduino.
- **Location**: [`/Communication/CAN`](./libraries/CAN).
- **Usage**: Import the library to implement CAN-based communication for connected devices.

---

## Adding New Peripherals

To maintain consistency, follow the steps below when adding a new peripheral to the repository:
1. **Create a New Directory**: Create a directory for the peripheral (e.g., `/camera` for a camera module inside /Pheripherals).
2. **Add Documentation**: Include a `README.md` file in the directory with:
   - **Peripheral Description**: Explain the purpose and function of the device.
   - **Connection Details**: Provide a wiring diagram or description.
   - **Code Details**: Describe the code provided in the directory and how to use it.
3. **Update the Main README**: Add a section for the new peripheral, linking to its directory.

---
