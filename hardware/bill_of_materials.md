# Bill of Materials — Sensor Network

Hardware components used in the sensor network implementation for the quadruped robot. All PCBs were designed in KiCad with double-layer copper, 0.7 mm trace width.

---

## Computing & Communication

| # | Component | Model | Qty | Notes |
|---|-----------|-------|-----|-------|
| 1 | Single-board computer | Raspberry Pi 5 | 1 | Central ROS2 node — Ubuntu 24.04 |
| 2 | Development PC | x86_64, Ubuntu 22.04 | 1 | ROS2 Humble visualization |
| 3 | Microcontrollers | Arduino Nano | 3 | 2× transmitter nodes + 1× CAN receiver |
| 4 | CAN transceiver modules | MCP2515 + TJA1050 | 3 | SPI interface to Arduino, one per node |

---

## Sensors

| # | Component | Model | Qty | Interface | PCB |
|---|-----------|-------|-----|-----------|-----|
| 5 | 9-DOF IMU | MPU9250 | 1 | I2C | PCB1 |
| 6 | Gas sensor | MQ-7 | 1 | Analog | PCB1 |
| 7 | Capacitive tactile | TTP223 | 4 | Digital | PCB1 (one per leg) |
| 8 | Hall-effect current sensor | ACS712 | 1 | Analog | PCB2 |
| 9 | Voltage sensor | FZ0430 | 1 | Analog | PCB2 |
| 10 | GPS module | NMEA 0183 (e.g., Neo-6M) | 1 | UART | External |
| 11 | RGB USB Camera | — | 1 | USB | External |
| 12 | 2D LiDAR | RPLiDAR A1/A2 | 1 | UART/USB | External |

---

## Custom PCBs (KiCad, double-layer)

| # | Board | Dimensions | Layers | Key components |
|---|-------|-----------|--------|---------------|
| 13 | PCB 1 — Proprioceptive + Environmental | 70 × 90 mm | 2 | Arduino Nano, MCP2515, MPU9250, MQ-7, 4× TTP223 |
| 14 | PCB 2 — Power Monitoring | 122.5 × 60.428 mm | 2 | 2× Arduino Nano, 2× MCP2515, ACS712, FZ0430 |
| 15 | Distributor PCB | 141 × 121 mm | 2 | Power distribution, CAN_H/CAN_L bus lines, male-female connectors |

---

## Power System

| # | Component | Qty | Notes |
|---|-----------|-----|-------|
| 16 | Robot power supply | LiPo / DC supply | 1 | Powers PCBs via distributor board |
| 17 | USB power for RPi5 | USB-C 27W | 1 | Raspberry Pi 5 power requirement |
| 18 | CAN Bus termination resistors | 120 Ω | 2 | One at each end of the CAN Bus |

---

## Wiring & Connectors

| # | Component | Qty | Notes |
|---|-----------|-----|-------|
| 19 | CAN Bus cable | Twisted pair | — | CAN_H and CAN_L, shielded if possible |
| 20 | Dupont jumper wires | M/F assorted | — | Sensor-to-PCB connections |
| 21 | Male-female pin headers | 2.54 mm pitch | — | PCB-to-distributor connectors |
| 22 | USB-A to USB-B Mini cables | — | 3 | Arduino Nano to Raspberry Pi |

---

## Tools

| # | Item | Notes |
|---|------|-------|
| 23 | KiCad 6+ | PCB design software (free) |
| 24 | Soldering iron | For PCB assembly |
| 25 | Multimeter | Electrical verification |
| 26 | Logic analyzer / oscilloscope | Optional — CAN Bus debugging |

---

## Notes

- All three MCP2515 modules must be configured at the **same CAN bitrate** (500,000 bps) to prevent message collision and data loss.
- Terminate the CAN Bus with 120 Ω resistors at both physical ends of the bus.
- The TTP223 tactile sensors are placed one per leg to detect foot-ground contact events.
- The ACS712 model (5A, 20A, or 30A) should be selected based on the expected current range of the monitored circuit.
- PCB manufacturing: designed in double-layer to reduce board size. Compatible with most PCB fabrication services (JLCPCB, PCBWay, etc.).

---

*For wiring diagrams, see [`../docs/can_bus_diagram.png`](../docs/can_bus_diagram.png)*  
*For PCB source files, see [`../hardware/schematics/`](schematics/)*
