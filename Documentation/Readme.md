# Preface
# Project Background — The Original Quadcopter (2013–2017)

## Introduction

The original Quadcopter project began in 2013 as a personal engineering exercise, with the objective of designing and constructing a fully functional quadrotor aircraft using an Arduino-based flight controller.

The primary motivation was not simply to build a flying aircraft, but to develop a deeper practical understanding of embedded systems, microcontroller architecture, control theory and quadrotor flight dynamics.

Three principal learning objectives were established:

1. **Embedded Systems:** Develop an understanding of the Atmel ATmega328P microcontroller, including hardware timers, interrupts, peripheral registers and low-level programming in C/C++.
2. **Control Theory:** Understand and implement Proportional–Integral–Derivative (PID) control algorithms, including feedback systems, sensor processing and controller tuning.
3. **Flight Dynamics:** Understand how a quadrotor achieves stable flight through differential motor thrust, and how pitch, roll and yaw are controlled.

Rather than using a commercially available flight controller, the intention was to develop the complete control system from first principles.

## Mechanical Design

The original airframe was constructed using readily available 20 mm PVC electrical conduit, with a four-way conduit junction forming the central structure.

This approach was selected for its low cost, simplicity and ease of construction. It also allowed the frame to be modified without specialist manufacturing equipment.

The aircraft used four Turnigy Aerodrive D2830/11 1000KV brushless motors, each controlled by a Turnigy Plush 30A Electronic Speed Controller (ESC).

Although unconventional compared with commercial quadcopter frames, the PVC construction provided a practical experimental platform.

Later development included more sophisticated CAD concepts incorporating electronics mounting platforms, motor brackets and landing gear.

## Electronics and Hardware

The flight controller was developed around the Atmel ATmega328P, initially using Arduino development boards before progressing towards an Arduino Nano-based implementation.

The principal hardware consisted of:

* **Microcontroller:** Atmel ATmega328P (Arduino Uno/Nano).
* **Motors:** 4 × Turnigy Aerodrive D2830/11 1000KV.
* **ESCs:** 4 × Turnigy Plush 30A.
* **Radio Transmitter:** OrangeRx T-SIX 2.4 GHz DSM2.
* **Radio Receiver:** OrangeRx R620 DSM2, six-channel.
* **Inertial Measurement Unit:** GY-521 (MPU6050).
* **Additional Sensor Development:** GY-80 multi-sensor IMU.
* **Communication:** HC-06 Bluetooth serial module.

The MPU6050 provided three-axis acceleration and angular velocity measurements, which formed the basis of the flight-control feedback system.

Additional investigations explored sensor fusion using complementary and Kalman filtering, along with the MPU6050's Digital Motion Processor (DMP).

## Software Architecture

A significant part of the project involved developing a modular embedded software architecture.

The software was divided into independent functional components, including:

* Sensor acquisition and processing.
* Radio receiver input.
* PID control algorithms.
* Cyclic Collective Throttle Mixer (CCTM).
* Motor PWM generation.
* System timing and task scheduling.
* Watchdog supervision.
* Digital I/O and diagnostics.

The design separated hardware-level drivers from higher-level application functions, allowing individual subsystems to be developed and tested independently.

### Task Scheduling

A custom cooperative scheduler was developed using the ATmega328P's 16-bit Timer1 peripheral.

Timer interrupts provided the timing reference for multiple software tasks, allowing different operations to execute at defined intervals.

The later design targeted a flight-control frequency of approximately 200 Hz, corresponding to a 5 ms control period.

Timer0 and Timer2 were utilised to generate hardware PWM signals for the four ESCs.

Radio receiver signals were captured using pin-change interrupts, avoiding the blocking behaviour of the Arduino `pulseIn()` function.

This work provided practical experience with hardware timers, interrupt service routines, register manipulation and real-time embedded software.

### Flight Control

The flight-control architecture was based on closed-loop PID control.

Two operating modes were investigated:

**Acro Mode (Rate Control)**

The pilot commands a desired angular velocity, which is compared against the measured gyroscope output. PID corrections are then applied to the motors to achieve the requested rotational rate.

**Stabilisation Mode (Attitude Control)**

A cascaded control architecture was proposed, consisting of an outer attitude controller and an inner angular-rate controller.

This arrangement was intended to provide self-levelling flight characteristics.

The overall design incorporated up to six PID controllers, providing attitude and rate control across the pitch, roll and yaw axes.

A custom motor mixer translated the resulting control corrections into individual motor commands.

## Development and Testing

Development progressed from individual hardware experiments to a substantially integrated flight-controller application.

Several important milestones were achieved, including:

* Control of brushless motors through ESC-generated command signals.
* Acquisition and processing of radio receiver inputs.
* Communication with the MPU6050 over I²C.
* Investigation of sensor calibration, filtering and attitude estimation.
* Development of a timer-driven cooperative scheduler.
* Implementation of PID control calculations and motor mixing.
* Integration of watchdog functionality and serial diagnostics.
* Development of monitoring and visualisation using National Instruments LabVIEW.

The software ultimately evolved into a modular Version 3 implementation, with development continuing through approximately 2016–2017.

## Project Outcome and Lessons Learned

Despite considerable progress in both hardware and software development, **the original quadcopter never achieved successful controlled flight**.

The principal difficulty arose during the integration and tuning of the flight-control system.

Achieving stable PID behaviour proved particularly challenging, and the project was eventually discontinued before the aircraft became operational.

A retrospective review of the recovered firmware in 2026 identified several implementation problems involving PID calculations, motor mixing, interrupt configuration, task scheduling and safety handling.

These defects provide plausible explanations for the difficulties encountered during testing, although the precise causes of the original flight instability cannot be established conclusively without further investigation.

In hindsight, the project highlighted the importance of validating individual subsystems before attempting full system integration.

It also demonstrated that successful flight depends on more than correctly implementing a PID equation. Sensor accuracy, sampling frequency, execution timing, control direction, motor mixing and mechanical characteristics must all work together as a coherent feedback system.

Nevertheless, the project achieved many of its original educational objectives, providing valuable practical experience in microcontroller programming, embedded software architecture, digital signal processing and feedback control.

---

# Quadcopter MKII Reloaded — A New Beginning (2026)

More than a decade after the original project began, Quadcopter MKII Reloaded represents an opportunity to revisit the original engineering challenge with additional knowledge, experience and development tools.

Rather than continuing directly from the previous firmware, the intention is to establish a completely new software project while retaining the original design philosophy and learning objectives.

The original documentation, source code and engineering drawings will be preserved as historical references.

The new development will continue to use the ATmega328P microcontroller, with an Arduino Uno serving as the primary development platform and an Arduino Nano forming the onboard flight controller.

The original PVC airframe, motors, ESCs, radio equipment and MPU6050 sensor will be retained where practical.

Software development will move to a modern toolchain using Visual Studio Code, PlatformIO and Git.

Particular emphasis will be placed on:

* Understanding the underlying ATmega328P hardware rather than unnecessarily abstracting it behind libraries.
* Developing a modular, maintainable and independently testable software architecture.
* Implementing deterministic task scheduling with measurable execution times.
* Understanding sensor acquisition, calibration and attitude estimation.
* Developing and validating PID algorithms through simulation and controlled experiments.
* Implementing robust motor control, fault detection and safety mechanisms.
* Applying a structured engineering process with defined requirements, testing and documented design decisions.

The intention is not simply to correct the shortcomings of the original project, but to use the knowledge gained from it to develop a more robust and thoroughly understood flight-control system.

## Project Objective

**To design, develop and successfully fly a quadrotor aircraft controlled by a custom ATmega328P-based flight controller, while developing a detailed understanding of embedded systems, PID control and quadrotor flight dynamics.**

The success of Quadcopter MKII Reloaded will be measured not only by achieving stable flight, but also by the ability to understand, explain, test and validate the engineering principles that make that flight possible.


# Introduction
Aims of the prorject: 
- To create an arduino based flight controller for a quadcopter
- Learn industry level programming techniques and architecture 
- Learn Atmega328p


# Getting Started

## Programming IDE
Choosing to use VS Code with the platform IO extension for programming the arduino.

![alt text](image.png)
![alt text](image-2.png)