# 🚁 Autonomous GPS Waypoint Navigation Drone with Obstacle Avoidance

> **An autonomous drone navigation and delivery platform combining real-world GIS route planning, embedded flight control, obstacle avoidance, and autonomous Return-to-Home (RTH).**

---

## 📌 Project Overview

This project presents the **design, simulation, and embedded implementation of an autonomous GPS-based navigation system for a quadcopter drone**.

The system integrates a **Python-based ground station**, an **Arduino Uno (ATmega328P) flight controller**, real-world geographic data from **OpenStreetMap**, and a **Proteus virtual simulation environment**.

The overall architecture is designed to enable the drone to:

* 🗺️ Generate routes using real-world geographic networks
* 📍 Navigate through sequential GPS waypoints
* 🧭 Determine distance and bearing to each waypoint
* 🚧 Detect and avoid obstacles during navigation
* 📦 Hold position at the destination for payload delivery
* 🏠 Automatically return to the launch location
* 🛑 Execute emergency motor shutdown during unsafe orientation
* 💾 Operate within the strict SRAM limitations of the ATmega328P

The project combines **GIS route planning, embedded systems, sensor fusion, finite-state-machine control, and autonomous navigation** into a single end-to-end system.

---

# 🎯 Core Objectives

### Objective 1 — Autonomous GIS Route Planning

Develop an automated geospatial route-planning pipeline capable of:

1. Accepting a start and destination coordinate.
2. Downloading real-world geographic network data from **OpenStreetMap**.
3. Constructing a navigable graph using **OSMnx** and **NetworkX**.
4. Calculating a shortest-path trajectory.
5. Extracting GPS coordinates from the resulting route.
6. Downsampling waypoints to remain within the **<2 KB SRAM constraint** of the ATmega328P.
7. Exporting the optimized coordinates into structured C++ headers for embedded execution.
8. Generating an interactive HTML visualization of the planned trajectory.

### Objective 2 — Autonomous Embedded Flight Control

Develop a deterministic embedded flight controller featuring:

* GPS-based waypoint navigation
* MPU-6050 inertial measurements
* IR-based obstacle detection
* Ball-tilt emergency protection
* Differential-thrust motor control
* Non-blocking execution using `millis()`
* Finite State Machine (FSM) architecture
* Autonomous Return-to-Home navigation

---

# 🏗️ System Architecture

```text
                    ┌──────────────────────────┐
                    │      USER / OPERATOR     │
                    │ Start + Destination GPS  │
                    └────────────┬─────────────┘
                                 │
                                 ▼
                    ┌──────────────────────────┐
                    │    PYTHON GROUND STATION │
                    │                          │
                    │ OSMnx + NetworkX        │
                    │ Route Generation         │
                    │ Waypoint Optimization    │
                    │ Folium Visualization     │
                    └────────────┬─────────────┘
                                 │
                         C++ Waypoint Header
                                 │
                                 ▼
                    ┌──────────────────────────┐
                    │      ARDUINO UNO         │
                    │       ATmega328P         │
                    │                          │
                    │   8-State FSM Controller │
                    └───────┬──────┬───────────┘
                            │      │
             ┌──────────────┘      └───────────────┐
             ▼                                     ▼
    ┌─────────────────┐                   ┌─────────────────┐
    │     SENSORS     │                   │  MOTOR CONTROL  │
    │                 │                   │                 │
    │ NEO-6M GPS      │                   │ M1 ── D2        │
    │ MPU-6050 IMU    │                   │ M2 ── D3        │
    │ IR Obstacle     │                   │ M3 ── D4        │
    │ Tilt Switch     │                   │ M4 ── D5        │
    └─────────────────┘                   └─────────────────┘
```

---

# 🧠 Autonomous Control Logic

The onboard firmware is organized around a **deterministic 8-state Finite State Machine**.

The controller uses a **non-blocking 700 ms control interval**, scheduled using `millis()`, allowing sensor processing, navigation, communication, and motor control to operate without blocking delays.

## 🔄 Flight State Machine

### 1. `STATE_IDLE`

Initial and standby state.

* All motors are stopped.
* The controller monitors incoming commands.
* Available commands include:

  * `START`
  * `STOP`
  * `LIST`
  * `RTH`

---

### 2. `STATE_NAVIGATING`

The drone navigates toward the current GPS waypoint.

The controller continuously calculates:

* GPS position
* Haversine distance to the waypoint
* Great-circle bearing
* Required directional correction
* Differential motor thrust

When the waypoint is reached, the waypoint index advances to the next coordinate.

---

### 3. `STATE_AVOIDING_OBSTACLE`

When the forward-facing IR sensor detects an obstacle above the configured threshold:

```text
NAVIGATING
     │
     │ Obstacle detected
     ▼
AVOIDING_OBSTACLE
     │
     │ Path clear
     ▼
NAVIGATING
```

The drone performs an evasive **starboard maneuver** and continues the maneuver until the obstacle is no longer detected.

---

### 4. `STATE_WAITING_AT_DESTINATION`

When the final waypoint is reached within the configured **5.0 m arrival radius**:

* Navigation stops.
* Motors disengage.
* The drone enters a **5000 ms destination hold**.
* This state represents the payload-delivery phase.

After the delivery interval, the system initiates Return-to-Home.

---

### 5. `STATE_RETURNING`

The drone autonomously returns to its launch location.

Instead of generating a second route, the controller reverses the previously generated waypoint sequence:

```text
Waypoint 0 → Waypoint 1 → Waypoint 2 → ... → Waypoint N
     ▲                                      │
     └──────────── Return-to-Home ─────────┘
```

This reduces computational and memory requirements while ensuring that the return journey follows the previously planned trajectory.

---

### 6. `STATE_EMERGENCY_HALT`

A safety override designed to immediately stop the motors when an unsafe condition is detected.

The emergency state is triggered by conditions such as:

* Airframe inversion detected by the ball tilt switch
* Excessive pitch
* Excessive roll
* Other configured safety boundaries

When triggered:

```text
M1 = OFF
M2 = OFF
M3 = OFF
M4 = OFF
```

The objective is to prevent continued motor operation during a potentially dangerous orientation.

---

# 🗺️ GIS Route Planning Pipeline

The ground station uses real-world geographic data to generate the drone's navigation route.

```text
Start Coordinate
       │
       ▼
Destination Coordinate
       │
       ▼
OpenStreetMap Network
       │
       ▼
OSMnx Graph
       │
       ▼
NetworkX Shortest Path
       │
       ▼
GPS Waypoint Extraction
       │
       ▼
Waypoint Downsampling
       │
       ▼
SRAM Optimization
       │
       ▼
C++ Header Generation
       │
       ▼
Arduino Flight Controller
```

The route planner is specifically optimized for the limited memory resources of the **8-bit ATmega328P**.

The generated waypoint set is therefore sampled and compressed before being transferred to the embedded controller.

---

# 📡 Hardware Interface

## Arduino Uno Pin Mapping

| Arduino Pin | Function              | Connected Device             |
| ----------- | --------------------- | ---------------------------- |
| **D0 / RX** | Hardware UART RX      | Virtual Terminal / HC-05 TXD |
| **D1 / TX** | Hardware UART TX      | Virtual Terminal / HC-05 RXD |
| **D2**      | Motor 1 control       | Top-Left Motor               |
| **D3**      | Motor 2 control       | Top-Right Motor              |
| **D4**      | Motor 3 control       | Bottom-Left Motor            |
| **D5**      | Motor 4 control       | Bottom-Right Motor           |
| **D6**      | Tilt detection        | Ball Tilt Switch             |
| **D8**      | GPS RX                | NEO-6M TX                    |
| **D9**      | GPS TX                | NEO-6M RX                    |
| **A3**      | Analog obstacle input | IR Obstacle Sensor           |
| **A4**      | I²C SDA               | MPU-6050 SDA                 |
| **A5**      | I²C SCL               | MPU-6050 SCL                 |

---

# ⚙️ Motor Control

The quadcopter uses four independently controlled DC motors:

| Motor  | Position     | Arduino Pin |
| ------ | ------------ | ----------- |
| **M1** | Top-Left     | D2          |
| **M2** | Top-Right    | D3          |
| **M3** | Bottom-Left  | D4          |
| **M4** | Bottom-Right | D5          |

Each motor-control output drives an NPN transistor through a current-limiting resistor.

The controller uses **differential thrust** to produce directional corrections during autonomous navigation and obstacle avoidance.

> ⚠️ **Hardware Note:** The Arduino GPIO pins are used only as control signals. Motors must be powered from an appropriate external motor supply through suitable driver/transistor circuitry. The Arduino should never supply motor current directly.

---

# 🧭 Sensor System

## 📍 NEO-6M GPS

The GPS module provides:

* Latitude
* Longitude
* Position updates
* Navigation reference for waypoint tracking

Communication is implemented using `SoftwareSerial` at **9600 baud**.

```text
NEO-6M TX → Arduino D8
NEO-6M RX → Arduino D9
```

---

## 🧭 MPU-6050 IMU

The MPU-6050 provides inertial measurements through the Arduino's hardware I²C interface.

```text
MPU-6050 SDA → A4
MPU-6050 SCL → A5
```

The IMU is used for orientation and flight-integrity monitoring.

---

## 🚧 IR Obstacle Sensor

A forward-facing IR proximity sensor is connected to:

```text
IR Sensor OUT → Arduino A3
```

The analog reading is compared against a configurable obstacle threshold.

When the threshold is exceeded, the FSM transitions from:

```text
STATE_NAVIGATING
        ↓
STATE_AVOIDING_OBSTACLE
```

---

## 🛑 Ball Tilt Switch

The tilt switch provides an additional physical safety mechanism.

```text
Tilt Switch → D6
```

The pin uses:

```cpp
INPUT_PULLUP
```

Therefore:

```text
Normal orientation → HIGH
Airframe inversion → LOW
```

An active-low signal causes the controller to enter the emergency motor shutdown state.

---

# 💻 Software Stack

| Component            | Technology               |
| -------------------- | ------------------------ |
| Ground Station       | Python 3.13              |
| Route Planning       | OSMnx 2.x                |
| Graph Processing     | NetworkX                 |
| Map Visualization    | Folium                   |
| Serial Communication | PySerial                 |
| Embedded Firmware    | C++                      |
| Microcontroller      | Arduino Uno / ATmega328P |
| Embedded Compiler    | AVR-GCC                  |
| Simulation           | Proteus 8.10 / 8.16      |
| IDE                  | Arduino IDE 2.x          |

### Embedded Libraries

```cpp
#include <Wire.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
```

### Python Dependencies

```text
osmnx
networkx
folium
pyserial
shutup
```

---

# 💾 Memory Optimization

One of the major engineering constraints of the project is the limited SRAM available on the **ATmega328P**.

The Arduino Uno provides only a small amount of SRAM, making it impractical to store a large number of raw GPS coordinates.

To address this limitation, the system performs waypoint optimization on the Python side:

```text
Raw GIS Route
     ↓
Many Coordinates
     ↓
Waypoint Sampling
     ↓
Reduced Coordinate Set
     ↓
C++ Header
     ↓
ATmega328P SRAM
```

This approach moves computationally expensive geographic processing to the Python ground station while leaving the Arduino responsible for real-time embedded control.

---

# 🔄 Complete Mission Sequence

```text
             ┌─────────────┐
             │    IDLE     │
             └──────┬──────┘
                    │ START
                    ▼
          ┌──────────────────┐
          │    NAVIGATING    │◄──────────────┐
          └───────┬──────────┘               │
                  │                          │
          Obstacle detected                  │
                  │                          │
                  ▼                          │
       ┌─────────────────────┐               │
       │ AVOIDING_OBSTACLE   │───────────────┘
       └─────────────────────┘
                  │
          Final waypoint
                  │
                  ▼
       ┌─────────────────────┐
       │ WAITING_AT_DEST.    │
       └──────────┬──────────┘
                  │
             5 second hold
                  │
                  ▼
       ┌─────────────────────┐
       │    RETURNING/RTH    │
       └──────────┬──────────┘
                  │
             Home reached
                  │
                  ▼
             ┌─────────┐
             │  IDLE   │
             └─────────┘


      ┌─────────────────────────────┐
      │  SAFETY VIOLATION DETECTED  │
      └──────────────┬──────────────┘
                     │
                     ▼
          ┌─────────────────────┐
          │  EMERGENCY_HALT     │
          │                     │
          │ M1 = OFF            │
          │ M2 = OFF            │
          │ M3 = OFF            │
          │ M4 = OFF            │
          └─────────────────────┘
```

---

# 🧪 Simulation Environment

The system is designed to be tested within **Proteus Design Suite** before deployment to physical hardware.

The virtual testbed allows the following subsystems to be evaluated:

* Arduino flight-controller firmware
* GPS communication
* IMU communication
* IR obstacle detection
* Tilt-switch safety behavior
* Motor-control logic
* Serial communication
* State-machine transitions
* Waypoint navigation logic

This provides a controlled environment for validating the embedded software before physical flight testing.

---



# 📁 Project Structure

```text
Autonomous_GPS_Waypoint_Navigation_Drone_with_Obstacle_Avoidance/
│
├── 📂 Arduino/
│   ├── flight_controller.ino
│   ├── waypoints.h
│   └── ...
│
├── 📂 Python/
│   ├── route_planner.py
│   ├── waypoint_generator.py
│   └── ...
│
├── 📂 Proteus/
│   ├── simulation.pdsprj
│   └── ...
│
├── 📂 Maps/
│   └── generated_routes/
│
├── 📂 Documentation/
│   └── ...
│
├── requirements.txt
├── README.md
└── LICENSE
```

---

# 🔬 Engineering Concept

The fundamental design philosophy of this project is **division of computational responsibility**.

### Ground Station

The Python system handles computationally intensive tasks:

```text
GIS Data
   ↓
Graph Construction
   ↓
Shortest Path
   ↓
Waypoint Optimization
   ↓
C++ Header Generation
```

### Flight Controller

The Arduino handles deterministic real-time tasks:

```text
GPS
 ↓
Position
 ↓
Waypoint Navigation
 ↓
Sensor Monitoring
 ↓
Obstacle Avoidance
 ↓
Motor Control
 ↓
RTH
```

This architecture allows a resource-constrained **8-bit microcontroller** to participate in an autonomous navigation system without requiring it to perform computationally expensive GIS operations.

---

