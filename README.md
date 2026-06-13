# Self-Balancing 2WD Inverted Pendulum & Light Tracker

## Project Overview
This repository contains the implementation files and the comprehensive project report for a 2WD self-balancing inverted pendulum robot. Built around an ESP32 microcontroller and an MPU6050 motion sensor, the project demonstrates the practical application of classical control systems to achieve robust physical stabilization without the use of machine learning algorithms. 

## Control Methodologies
The repository documents the mathematical modeling, firmware implementation, and performance analysis of three distinct control architectures:

* **Standard PID Balancing:** The baseline stabilization system, relying on a finely tuned Proportional-Integral-Derivative (PID) control loop.
* **Lead-Lag Compensator:** An advanced balancing strategy utilizing a discrete Lead-Lag compensator, derived strictly from the mathematical model of the physical pendulum system to ensure robust stability.
* **MIMO Light Tracking:** The expansion of the project into a Multiple-Input Multiple-Output (MIMO) system. This mode integrates a secondary PID control loop driven by Light Dependent Resistors (LDRs), enabling the robot to maintain its vertical balance while simultaneously tracking and following a directional light source.

## Hardware Architecture
* **Microcontroller:** ESP32
* **Sensors:** MPU6050 (6-DoF IMU), Light Dependent Resistors (LDRs)
* **Actuators:** 2WD DC Motors with Encoders and standard Motor Driver

## Repository Contents
The files included in this repository consist of the C/C++ source code for the microcontroller, detailing the control loops for all three operating modes, alongside the formal project report. The report provides an in-depth breakdown of the system's dynamics, the derivation of the compensator equations, and the comparative performance of the applied control strategies.
