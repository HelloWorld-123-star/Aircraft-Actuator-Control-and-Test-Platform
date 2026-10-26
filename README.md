# Aircraft Actuator Control and Test Platform

## Project Overview

The Aircraft Actuator Control and Test Platform is an embedded systems project focused on controlling and testing a motor-driven actuator. It brings together motor control, position feedback, sensor measurements, PLC communication, and a desktop interface. The setup is inspired by aerospace actuator testing, using a 12 V DC geared motor as a prototype rather than an aircraft-certified actuator.

An **STM32 NUCLEO-F446RE** handles motor control, encoder feedback, sensor readings, and communication. A **CLICK PLUS PLC** exchanges data with the STM32 over **Modbus TCP**, while a **Python-based HMI** provides live monitoring and command testing.

Motor operation, encoder feedback, Ethernet communication, basic PLC logic, and HMI communication have been tested. Full closed-loop positioning and physical safety functions are still under development.

## Hardware and Software

| Component | Purpose |
|---|---|
| STM32 NUCLEO-F446RE | Main controller and embedded firmware |
| 12 V geared DC motor + driver | PWM motor control |
| Quadrature encoder | Position feedback |
| WIZ550IO / W5500 | Ethernet connection over SPI |
| CLICK PLUS C2-01CPU | PLC communication and basic Ladder Logic |
| DS18B20 | Motor temperature readings |
| ADXL345 | Vibration measurements over I²C |
| ACS724 | Current sensing (still being tested) |
| Python, PySide6, PySerial | Desktop HMI on macOS |
| STM32CubeIDE, Embedded C, STM32 HAL | Firmware development |

## System Architecture

```text
                  Mac HMI (Python / PySide6)
                            |
                       USB / UART
                            |
                    STM32 NUCLEO-F446RE
                     /       |       \
               TIM3 PWM  TIM4 Encoder  SPI1
                  |          ^          |
              Motor Driver   |       W5500 Ethernet
                  |          |          |
                12 V Motor --+       Modbus TCP
                                         |
                                  CLICK PLUS PLC

STM32 sensors:
  I²C    -> ADXL345 vibration sensor
  1-Wire -> DS18B20 temperature sensor
  ADC    -> ACS724 current sensor (not yet verified)
```

## 1. STM32 Motor Control

Motor output and encoder feedback were first tested directly through the STM32 firmware.

- Set up **TIM3 PWM at 20 kHz** to drive the motor through an external motor driver.
- Used GPIO pins for motor direction control.
- Verified physical motor rotation during a short PWM test.
- Configured **TIM4 in encoder mode** and verified that encoder counts changed during motor rotation.
- Developed an initial proportional position-control routine with position-error calculation, PWM adjustment, target tolerance, and a **5-second timeout**.

Motor rotation and encoder feedback have been verified. The position-control routine has not yet been connected to the HMI MOVE command for physical closed-loop operation.

### Sensors and STM32 Peripherals

The firmware also uses **UART** for HMI messages and debugging, **SPI** for the W5500, **I²C** for the ADXL345, **1-Wire** for the DS18B20, and **ADC** for current-sensor testing.

The ADXL345 provides vibration data for RMS calculation. DS18B20 temperature readings have occasionally been unstable, and ACS724 current measurements are not yet verified. The HMI also displays NORMAL, WARNING, and ERROR status information.

## 2. W5500 Ethernet Setup

A **WIZ550IO Ethernet module**, based on the W5500 controller, was connected to the STM32 through SPI1.

Ethernet setup was tested in stages:

1. Read the W5500 `VERSIONR` register and got the expected value, **`0x04`**.
2. Set a static IP address and read it back from the W5500 registers.
3. Checked the Ethernet link using the `PHYCFGR` register.
4. Verified network connectivity from macOS with **4 out of 4 successful ping replies**.
5. Set up a TCP server on **port 502** for Modbus TCP communication.

| Device | IP address |
|---|---|
| STM32 / W5500 | `192.168.1.50` |
| CLICK PLUS PLC | `192.168.1.60` |

During debugging, SPI register access, Ethernet link status, IP configuration, TCP connection, and Modbus responses were checked separately to identify communication issues.

## 3. Modbus TCP Between the PLC and STM32

The **CLICK PLUS C2-01CPU** communicates with the STM32 through the W5500 Ethernet module. The PLC operates as the **Modbus TCP client**, and the STM32/W5500 operates as the **server**.

The following Modbus function codes were tested:

| Function | Test performed |
|---|---|
| **FC06** – Write Single Register | Sent the value `123` from the PLC to the STM32 and checked the PLC success flag |
| **FC03** – Read Holding Registers | Read values back from the STM32, including temperature and vibration register tests |
| **FC16** – Write Multiple Registers | Sent a basic STOP command from the PLC and confirmed that the STM32 received function code `0x10` |

### Troubleshooting

**Modbus response timeouts.** The Ethernet link and TCP connection were active, but the PLC reported communication errors. Received Modbus frames, W5500 transmit buffers, socket states, and send flags were checked. Blocking sensor reads and delays in the STM32 main loop were found to interfere with timely responses. Temporarily disabling these reads improved communication; non-blocking sensor scheduling remains to be implemented.

**PLC register conflict.** During FC03 sensor testing, the Modbus exception-code destination overlapped with temperature register `DS4`. Moving the exception-code destination to `DS10` resolved the conflict and restored the expected temperature and vibration readings.

**FC16 trigger timing.** An earlier Ladder Logic version reset the send trigger before the FC16 request could reliably execute. Trigger conditions and a temporary timer were tested during troubleshooting. The final program was simplified to retain the basic communication functions.

## 4. CLICK PLC Ladder Logic

The retained PLC program contains 18 rungs and focuses on Modbus communication, heartbeat monitoring, and sending a STOP command. Earlier experiments with run/fault latches, manual/automatic modes, and a separate acknowledgment-read sequence are not part of this version.

### Heartbeat and Communication Status (Rungs 1–7)

The PLC reads a heartbeat value into `DS6` and compares it with the previous value in `DS31`. A change activates `HeartbeatChanged` (`C67`) and updates `DS31`. Once a valid heartbeat has been observed, `HeartbeatValid` (`C68`) is set. If the heartbeat stops changing for 3 seconds, timer `T1` activates `CommTimeout` (`C59`). `CommOK` (`C52`) is on when a valid heartbeat has been received and no timeout is active.

### STOP Command and Modbus Write (Rungs 8–15)

A `StopPending` request (`C111`) copies `0` into `DS20`. When communication is OK, `StopReady` (`C112`) generates a one-shot `StopPulse` (`C114`). The pulse increments the command sequence in `DS22` and sets `SendTrigger` (`C81`). A 500 ms timer (`T2`) resets that trigger. The Modbus **FC16** instruction writes three registers starting from `DS20`, with `C83` and `C84` indicating success and error respectively. The PLC program therefore prepares and transmits a STOP command; this does not by itself establish that the physical motor stops.

### Modbus Read (Rungs 16–17)

The program enables a Modbus **FC03** read when no FC16 send is active and `SendTrigger` is off. It reads four holding registers into `DS3` through `DS6`, including the heartbeat value used by the monitoring logic. The receive instruction exposes success and error indicators (`C14` and `C15`) and stores its exception code in `DS10`.

| PLC address | Purpose in the retained program |
|---|---|
| `DS3–DS6` | Four holding registers read from STM32; `DS6` is used as heartbeat |
| `DS31` | Previous heartbeat value |
| `DS20` | STOP command value (`0`) |
| `DS21` | Included in the three-register FC16 write block |
| `DS22` | Incremented command sequence |
| `C52` / `C59` | Communication OK / timeout |
| `C111` / `C112` / `C114` | STOP pending / ready / one-shot pulse |
| `C81` | FC16 send trigger |
| `C83` / `C84` | FC16 success / error |

The current ladder logic has been tested for communication and command transmission. Physical STOP performance, communication-loss behavior during powered motion, and long-term reliability still require testing.

## 5. Python HMI

A desktop HMI was developed using **Python, PySide6, and PySerial**, communicating with the STM32 over USB serial at **115200 baud**.

The interface includes live commanded and encoder positions, position error, movement time, temperature, vibration, system status, a position-history graph, target-position input, MOVE/STOP controls, connection status, diagnostics, and event logging.

### Physical Motion and Position Control

The HMI **MOVE** command has been tested with actual motor movement, not just UART message exchange. The STM32 uses TIM4 encoder feedback and a basic proportional position controller to move toward the requested target. The tested controller used `Kp = 0.01`, a **±20-count deadband**, and a **1000 ms MOVE timeout**. TIM3 PWM compare values were limited to 6–10 during the tested control sequence; these are compare values, not duty-cycle percentages.

The HMI **STOP** command was also demonstrated during powered motion. The STM32 reported STOP and a TIM3 PWM compare value of **0**. This is a **software STOP**, not a verified hardware emergency stop or motor-power disconnect.

A forced MOVE timeout test reported a position error and a TIM3 PWM compare readback of **0**, verifying the firmware's timeout shutdown path. Mechanical stopping distance was not measured.

Some HMI messages still contain legacy labels such as `TEST MODE` and `software only`; these labels need cleanup to reflect the tested functionality.

## 6. Test Results

| Test | Result |
|---|---|
| STM32 PWM motor rotation | Passed |
| TIM4 encoder feedback | Passed |
| W5500 SPI communication | Passed |
| Ethernet link and ping | Passed |
| Modbus TCP FC03, FC06, FC16 | Passed in basic tests |
| PLC heartbeat and STOP command communication | Passed (communication only) |
| HMI live telemetry and MOVE/STOP command exchange | Passed |
| HMI MOVE controlling physical motor movement | Passed |
| Encoder-based proportional position control | Passed in short tests |
| HMI software STOP during powered movement | Demonstrated; PWM compare reported 0 |
| MOVE timeout protection | Passed; PWM compare readback 0 |
| Position targets 0, 100, and 200 counts | All final errors within ±20 counts |
| Six alternating 0 ↔ 200 position commands | Passed; maximum absolute final error 14 counts |
| Current measurement and overcurrent protection | Not yet verified |
| Independent hardware emergency stop | Not yet verified |
| PLC-controlled physical motor movement | Not yet verified |
| Long-duration system reliability | Not yet tested |

### Position Test Details

| Target (counts) | Final encoder position | Signed error (target − position) |
|---:|---:|---:|
| 0 | -2 | +2 |
| 100 | 108 | -8 |
| 200 | 210 | -10 |

Six successive commands alternating between 0 and 200 counts returned final positions of **-14, 213, -14, 213, -14, and 213** counts, respectively. All were within the ±20-count deadband; the largest absolute error was **14 counts**. This indicates short-term repeatability under the tested conditions, not calibrated position accuracy or long-term reliability.

## 7. Firmware Architecture and Additional Interfaces

The current firmware uses **STM32 HAL with a main-loop structure**. The project architecture allows for future **FreeRTOS** task scheduling to separate motor control, sensor acquisition, and communication. **CAN**, **RS-485 / Modbus RTU**, and **Wi-Fi** are additional communication interfaces under consideration. Existing position-error and vibration RMS calculations also provide a basis for further **DSP** and fault-detection work.

These interfaces and RTOS features have not yet been implemented or verified.

## 8. Next Steps

- Validate the current sensor with the 12 V motor circuit and calibrate current measurements before implementing overcurrent protection.
- Improve DS18B20 wiring and sensor-read timing.
- Add and test an independent physical emergency-stop circuit and suitable electrical protection.
- Integrate the existing PLC Modbus commands with actual STM32 motor control; PLC-to-motor operation is not yet verified.
- Test communication-loss behavior, mechanical limits, and fault handling during powered motion.
- Run longer tests with the PLC, STM32, sensors, and HMI together, including different loads.
- Clean up outdated `TEST MODE` labels in the HMI.

---

**Project scope:** This is an engineering prototype, not a certified aircraft actuator controller. Software STOP commands do not replace a physical emergency-stop circuit.
