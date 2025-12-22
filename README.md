# Intelligent Fan Control System (STM32 + FreeRTOS)

An embedded FreeRTOS-based intelligent fan control system implemented on STM32F407, integrating multiple peripherals and supporting two operation modes: an automatic mode using fuzzy-logic-based control for adaptive fan speed regulation, and a Bluetooth-driven manual mode for user-defined control.

This project focuses on real-time embedded system design, task synchronization, mode management, and modular firmware architecture.


---

##  Project Overview

This system monitors **temperature**, **humidity**, and **IR proximity**, and dynamically adjusts fan speed using a **fuzzy logic controller** with **EMA signal conditioning** to reduce sensor noise and unnecessary output fluctuations.

The firmware is structured as **independent FreeRTOS tasks**, communicating via **queues, mutexes, and ISR-driven events**, reflecting a realistic embedded product architecture.

---

##  Key Features

###  FreeRTOS-Based Architecture
- Multiple dedicated tasks:
  - `FanControl_task`
  - `DHT11_task`
  - `IR_task`
  - `Bluetooth_task`
  - `OLED_task`
  - `Log_task`
- Clear separation between **control logic**, **I/O**, and **communication**
- Deterministic task scheduling and non-blocking design

###  Fuzzy Logic Fan Control
- Temperature fuzzy sets: *Cool / Warm / Hot*
- Humidity fuzzy sets: *Dry / Comfortable / Wet*
- Weighted decision logic produces smooth PWM output

###  EMA Signal Conditioning
- Exponential Moving Average applied to sensor data
- Reduces short-term noise and measurement jitter

###  Output Update Threshold
- Suppresses insignificant PWM changes
- Reduces unnecessary PWM updates

---

##  Hardware & Peripherals

| Component | Interface | Description |
|--------|----------|-------------|
| STM32F407 | Cortex-M4 | Main MCU |
| DHT11 | GPIO + Timer | Temperature & Humidity Sensor |
| IR Sensor | GPIO | Proximity Detection |
| Fan + L298N | PWM (TIM2) | Fan Speed Control |
| HC-05 Bluetooth | UART | Remote Command Input |
| SH1106 OLED | I²C | System Status Display |
| USB-TTL | UART2 | Debug Logging |

---


