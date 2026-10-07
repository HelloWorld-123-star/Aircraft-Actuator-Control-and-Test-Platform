# Aircraft Actuator Control and Test Platform

Embedded and industrial automation test platform integrating STM32, FreeRTOS, PLC/HMI, Modbus, CAN, RS-485, Ethernet/Wi-Fi, DSP-based condition monitoring, safety control, and automated actuator validation.

## Project Overview

This project develops a modular aircraft actuator control and test platform combining embedded control with industrial automation.

The STM32 serves as the real-time embedded controller for actuator control, sensor acquisition, and system monitoring, while the PLC provides supervisory control, operating logic, safety interlocks, and high-level command management.

The system is designed to support communication between the embedded controller, PLC, and HMI through industrial communication protocols.

## System Architecture

PC / HMI  
↓  
Modbus TCP  
↓  
PLC Supervisory Controller  
↓  
STM32 Embedded Controller  
↓  
Motor Driver / Actuator / Sensors

### STM32 Controller

The STM32 is responsible for low-level and real-time control tasks, including:

- PWM motor control
- Position measurement
- Current sensing
- Temperature and vibration monitoring
- ADC and timer-based data acquisition
- Fault detection
- Communication with the supervisory controller
- Future FreeRTOS-based task management

## PLC Supervisory Control

The PLC layer provides higher-level control and safety logic for the actuator test platform.

Current PLC development includes:

- Run/Stop control logic
- System-ready and fault interlocks
- Manual and automatic operating modes
- Actuator target validation
- Motion command generation
- Modbus TCP communication
- Holding-register read/write testing using `mbpoll`

Initial PLC logic and communication are being developed and tested in a virtual PLC environment before migration to a physical CLICK PLUS PLC.

## PLC–STM32 Integration

The PLC will send high-level commands to the STM32, such as:

- Run/Stop
- Operating mode
- Target actuator position
- Test commands

The STM32 will execute the real-time actuator control and return system information such as:

- Actual actuator position
- Motor current
- Temperature
- Vibration
- Motion status
- Fault status

## Planned Development

- Physical CLICK PLUS PLC integration
- PLC–STM32 communication
- HMI development
- FreeRTOS firmware architecture
- CAN and RS-485 communication
- Ethernet/Wi-Fi connectivity
- DSP-based condition monitoring
- Automated actuator test sequences
- Data logging and fault diagnostics