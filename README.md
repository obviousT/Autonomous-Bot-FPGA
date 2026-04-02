# Autonomous Maze-Solving Robot (Micromouse)

## Objective
The objective of this project is to design and develop an autonomous, self-navigating robot capable of exploring, mapping, and solving an unknown maze environment. Built entirely around a custom digital logic architecture, the robot relies on real-time sensor fusion and deterministic state machines rather than relying on a traditional microprocessor, showcasing advanced hardware-level control.

## Key Features
* **Custom Digital Control Architecture:** The core brain of the robot is entirely designed in Verilog, utilizing a robust Finite State Machine (FSM) to handle dynamic decision-making for intersections, dead-ends, and straightaways.
* **Discrete LUT-Based PID Control:** Implements a highly optimized Proportional-Derivative (PD) control loop using a custom Look-Up Table (LUT) for sensor distance buckets. This allows for aggressive, real-time wall-following and self-centering without floating-point math overhead.
* **Sensor Fusion & Odometry:** Integrates continuous data from proximity sensors (IR) with wheel encoder feedback. This ensures precise 90-degree and 180-degree tank steering, immune to surface slippage.
* **Real-Time Telemetry & Mapping:** Features an onboard coordinate tracking system that calculates absolute X/Y grid positions and compass directions. This data is streamed in real-time via an HC-05 Bluetooth module (at 115200 baud) to a remote terminal for live path mapping.
