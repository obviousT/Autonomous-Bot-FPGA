# Autonomous Maze-Solving Robot (FPGA)

A Verilog RTL design for an autonomous maze-solving robot implemented on an Intel/Altera FPGA. The robot navigates a maze using IR and ultrasonic obstacle sensing, closed-loop wheel-encoder feedback with a PID-corrected drive, tracks its own position by dead reckoning, and streams live telemetry (position, temperature, humidity, soil moisture) over UART/Bluetooth. A servo actuates at maze dead-ends.

> Target toolchain: **Intel Quartus Prime** (design uses Altera SignalTap IP under `ALL_IN_ONE/db/`). Board clock: **50 MHz**.

---

## Table of Contents
- [Features](#features)
- [System Architecture](#system-architecture)
- [Module Reference](#module-reference)
- [Clocking](#clocking)
- [Top-Level I/O](#top-level-io)
- [Finite State Machines](#finite-state-machines)
- [Clock-Domain Crossing & Handshakes](#clock-domain-crossing--handshakes)
- [UART Telemetry Protocol](#uart-telemetry-protocol)
- [Build & Program](#build--program)
- [Repository Layout](#repository-layout)


---

## Features

- **Wall-following maze navigation** driven by a 10-state motor-control FSM.
- **Dual obstacle sensing:** 3× IR sensors (left/front/right) + 3× HC-SR04 ultrasonic rangefinders.
- **Closed-loop drive:** quadrature encoder decoding per wheel, PID + smooth ramp control of PWM duty.
- **Dead-reckoning localization:** live `(x, y, direction)` estimate from encoder counts and turn events.
- **Environmental telemetry:** DHT11 temperature/humidity + ADC soil-moisture, packetized over UART.
- **Servo actuation** sequenced at dead-ends, with a request/acknowledge handshake to the drive FSM.
- **Bluetooth/UART command interface:** remote `START`/`STOP` control of the robot.

---

## System Architecture

```
                                  robot_motor_top
   ┌──────────────────────────────────────────────────────────────────────────┐
   │                                                                            │
 clk 50MHz ─┬─► clk3125 ──► clk_3125k (3.125 MHz) ──► uart_tx, msg_sender       │
            │                                                                   │
   ENA/ENB  │   ┌───────────────┐   count_A/B   ┌────────────────┐              │
   encoders─┼──►│ encoder_decoder│─────────────►│                │              │
            │   └───────────────┘               │                │  IN1..IN4    │
   IR ──────┼──►┌───────────────┐   dis_*, ir_* │ motor_controller├──► H-bridge │
   echo ────┼──►│  ir_and_us    │──────────────►│  (MAIN FSM)    │  ENA/ENB(PWM)│
            │   │ (3×IR + 3×US) │               │                │              │
            │   └───────────────┘               └───┬───┬────────┘              │
            │                                turn_done│   │servo_req/stop        │
            │   ┌───────────────┐                    ▼   ▼                       │
            │   │ bot_position  │◄──count_A/B   ┌────────────┐  SERVO_1/2        │
            │   │ (x,y,dir)     │──update_done─►│   servo    │──► PWM            │
            │   └──────┬────────┘               └────────────┘                   │
            │          │ x,y,dir                                                 │
   DHT11 ───┼─► t2a_dht ─► T/RH      ┌────────────┐  tx_start/tx_done            │
   soil ────┼─► moisture_sensor ─►   │ msg_sender ├──────────────► uart_tx ─► tx │
            │                        │ (TX FSM)   │                              │
   rx ──────┼─► uart_rx ─► uart_cmd_parser ─► bot_start                          │
            │                                                                    │
   └────────┴────────────────────────────────────────────────────────────────┘
```

**Top module:** [`robot_motor_top`](ALL_IN_ONE/robot_motor_top.v)

---

## Module Reference

| Module | File | Responsibility |
|---|---|---|
| `robot_motor_top` | [robot_motor_top.v](ALL_IN_ONE/robot_motor_top.v) | Top-level integration of all blocks |
| `motor_controller` | [motor_controller.v](ALL_IN_ONE/motor_controller.v) | **Main navigation FSM**: obstacle logic, PID, ramp, turns |
| `pwm_generator` | [pwm_generator.v](ALL_IN_ONE/pwm_generator.v) | 6-bit counter-comparator PWM (~781 kHz) for motor speed |
| `encoder_decoder` | [encoder_decoder.v](ALL_IN_ONE/encoder_decoder.v) | Quadrature decode → signed 32-bit count + direction |
| `ir_and_us` | [ir_and_us.v](ALL_IN_ONE/ir_and_us.v) | Wraps 3× `ir_sensor` + 3× `us_sensor` |
| `ir_sensor` | [ir_sensor.v](ALL_IN_ONE/ir_sensor.v) | Synchronizes IR input → obstacle flag |
| `us_sensor` | [us_sensor.v](ALL_IN_ONE/us_sensor.v) | HC-SR04 trigger/echo timing → distance FSM |
| `bot_position` | [position_finder.v](ALL_IN_ONE/position_finder.v) | Dead-reckoning `(x, y, dir)` with map boundaries |
| `servo` / `servo_pwm` | [servo.v](ALL_IN_ONE/servo.v) | Dead-end servo sequence FSM + 50 Hz servo PWM |
| `t2a_dht` | [dht_11.v](ALL_IN_ONE/dht_11.v) | DHT11 single-wire protocol FSM (bidirectional `inout`) |
| `moisture_sensor` | [soil_moisture.v](ALL_IN_ONE/soil_moisture.v) | ADC clock gen + wrapper |
| `adc_controller` | [adc_controller.v](ALL_IN_ONE/adc_controller.v) | Serial ADC read of soil-moisture channel |
| `msg_sender` | [msg_sender.v](ALL_IN_ONE/msg_sender.v) | Telemetry packetizer FSM (TX side) |
| `uart_tx` | [uart_tx.v](ALL_IN_ONE/uart_tx.v) | UART transmitter FSM (115200 baud) |
| `uart_rx` | [uart_rx.v](ALL_IN_ONE/uart_rx.v) | UART receiver FSM (115200 baud) |
| `uart_cmd_parser` | [uart_cmd_parser.v](ALL_IN_ONE/uart_cmd_parser.v) | Decodes `START`/`STOP` command bytes |
| `clk3125` | [clk3125.v](ALL_IN_ONE/clk3125.v) | 50 MHz → 3.125 MHz clock divider |
| `test_mpi` | [test_mpi.v](ALL_IN_ONE/test_mpi.v) | Standalone tick generator (not instantiated) |

---

## Clocking

| Clock | Frequency | Source | Used by |
|---|---|---|---|
| `clk` | 50 MHz | Board oscillator | Motor, sensors, encoders, DHT, servo, uart_rx, position |
| `clk_3125k` | ~3.125 MHz | `clk3125` (÷16 toggle) | `uart_tx`, `msg_sender` |
| `adc_sck` | ~3.125 MHz | `counter[3]` (÷16 gate) | `adc_controller` |

Signals crossing between the 50 MHz and 3.125 MHz domains are synchronized inside `msg_sender` (see below).

---

## Top-Level I/O

| Signal | Dir | Description |
|---|---|---|
| `clk` | in | 50 MHz system clock |
| `reset` | in | **Active-low** async reset |
| `ENA_1, ENB_1, ENA_2, ENB_2` | in | Wheel encoder A/B channels (2 motors) |
| `ir_l, ir_f, ir_r` | in | IR obstacle sensors (left/front/right) |
| `ec_l, ec_f, ec_r` | in | Ultrasonic echo inputs |
| `trig_l, trig_f, trig_r` | out | Ultrasonic trigger outputs |
| `rx` / `tx` | in / out | UART (Bluetooth) receive / transmit |
| `dout, din, adc_cs_n, adc_sck` | mixed | Soil-moisture ADC serial interface |
| `dht_inout` | inout | DHT11 single-wire data |
| `SERVO_1, SERVO_2` | out | Servo PWM outputs |
| `ENA, ENB` | out | Motor PWM (enable) outputs |
| `IN1, IN2, IN3, IN4` | out | H-bridge direction control |
| `led1..led5` | out | State/status indicators |

---

## Finite State Machines

The design contains **8 state machines** (plus 2 case-based sequential decoders). All use binary encoding with async active-low reset.

| FSM | States | Type | Notes |
|---|---|---|---|
| `motor_controller` | up to 10 (`FORWARD, LEFT, RIGHT, UTURN, SERVO, STOP, BFD, AFD, SBT, SAT`) | Mealy | **Main + most complex**; embeds PID + ramp |
| `t2a_dht` | 8 (`IDLE, LOW_19MS, REL_40US, WF_LOW80, WF_HIGH80, WF_AG_LOW, DATA, CHECK_SUM`) | Moore | Bidirectional 1-wire, timeout-protected |
| `msg_sender` | 6 (`IDLE, SEND_MPI, SEND_MM, SEND_TH, SEND_END, SEND_POS`) | Mealy | Telemetry packetizer |
| `servo` | 5 declared / 4 used (`S_IDLE, S_MOVE_S2, S_MOVE_S1, S_RET_S1, S_RET_S2`) | Moore | Timed actuation sequence |
| `us_sensor` | 4 (`s0..s3`) | Moore | Trigger → echo → compute → wait |
| `uart_rx` | 4 (`IDLE, START, DATA, STOP`) | Moore | Baud counter = 434 |
| `uart_tx` | 4 (`IDLE, START, DATA, STOP`) | Moore | 27 clocks/bit @ 3.125 MHz |
| `ir_sensor` | 2-state obstacle flag | Moore | Simplest |
| `encoder_decoder` | quadrature `{prev,curr}` | decoder | Gray-code transition table |
| `bot_position` | `case(turn_done)` navigation | datapath | Dead-reckoning |

---

## Clock-Domain Crossing & Handshakes

**Synchronizers** (all flip-flop chains):

| Location | Type | Signals |
|---|---|---|
| `ir_sensor` (×3) | 2-FF | `ir_sync1/2` |
| `us_sensor` (×3) | 2-FF | `echo_ff1/2` |
| `encoder_decoder` (×2) | 2-FF/channel | `A1/A2`, `B1/B2` |
| `t2a_dht` | 2-FF + edge detect | `s1/s2`, `sensor_prev` |
| `uart_rx` | 3-FF | `rx_ff1 → rx_ff3 → rx_ff2` |
| `msg_sender` | 2-FF + rising-edge (CDC 50 → 3.125 MHz) | `update_done`, `mpi_detected`, `End`, `mpi_count` |

**Handshakes:**

- **Motor ↔ Servo:** motor enters `SERVO` → `servo_req`; servo completes → `servo_stop` → motor proceeds to `UTURN`. Servo also pulses `send` to trigger telemetry.
- **Motor → bot_position:** `turn_done[1:0]` (00=straight, 01=left, 10=right, 11=u-turn), edge-detected to update heading once per turn.
- **msg_sender ↔ uart_tx:** classic `tx_start` / `tx_done` per-byte request/acknowledge.
- **UART cmd → system:** `bot_start` latch set/cleared by received `START`/`STOP` bytes.

---

## UART Telemetry Protocol

`msg_sender` emits ASCII packets, each terminated with `#`. `<n>` = MPI (dead-end/marker) count.

| Packet | Format | Meaning |
|---|---|---|
| MPI marker | `MPIM-<n>-#` | Marker/dead-end reached |
| Soil moisture | `MM-<n>-<M\|D>-#` | `M` = moist, `D` = dry |
| Temp/Humidity | `TH-<n>-<TT>-<HH>-#` | Temperature (°C) and humidity (%) |
| Position | `(<x>,<y>,<dir>)#` | `dir` ∈ `N/E/S/W` |
| End | `END-#` | Run finished |

UART: 115200 baud, 8-N-1.

---

## Build & Program

This is a **Quartus Prime** project.

1. Open Quartus Prime and create/open the project, adding all `.v` files under `ALL_IN_ONE/` (exclude `ALL_IN_ONE/db/` and `ALL_IN_ONE/output_files/` — those are generated).
2. Set **`robot_motor_top`** as the top-level entity.
3. Assign the `clk` pin to the board's 50 MHz oscillator and map all I/O per your board's pin-out; add a 50 MHz timing constraint:
   ```tcl
   create_clock -name sys_clk -period 20.000 [get_ports clk]
   ```
4. Compile (Analysis & Synthesis → Fitter → Assembler).
5. Program the device with the generated `.sof` via the Quartus Programmer.

**Simulation:** a testbench is provided at [output_files/tb_robot_motor_top.v](ALL_IN_ONE/output_files/tb_robot_motor_top.v). Run it in ModelSim/Questa with all design sources.

> Porting to Xilinx Vivado? Use the same clock constraint in an `.xdc` (`create_clock ... [get_ports clk]`) and add `set_false_path` on the async inputs (echo, IR, encoders, `rx`, `dht_inout`, `reset`).

---

## Repository Layout

```
ALL_IN_ONE/
├── robot_motor_top.v         # Top-level
├── motor_controller.v        # Main navigation FSM
├── pwm_generator.v           # Motor PWM
├── encoder_decoder.v         # Quadrature decoder
├── ir_and_us.v / path_controller.v  # Sensor wrappers
├── ir_sensor.v / us_sensor.v # Obstacle + ranging
├── position_finder.v         # Dead reckoning
├── servo.v                   # Servo FSM + PWM
├── dht_11.v                  # DHT11 driver
├── soil_moisture.v / adc_controller.v  # Soil ADC
├── msg_sender.v              # Telemetry FSM
├── uart_tx.v / uart_rx.v / uart_cmd_parser.v  # UART
├── clk3125.v                 # Clock divider
├── test_mpi.v                # Unused tick generator
├── output_files/             # Generated (testbench lives here)
└── db/                       # Generated Quartus/SignalTap IP (do not edit)
```

---


```
