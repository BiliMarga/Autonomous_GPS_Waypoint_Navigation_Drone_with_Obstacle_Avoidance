# Autonomous_GPS_Waypoint_Navigation_Drone_with_Obstacle_Avoidance

This project presents the design, simulation and embedded implementation of an autonomous navigation system. Modern autonomous drones require reliable spatial orientation, collision evasion and failsafe return mechanisms when deployed in urban. The system uses Python ground station with an SRAM-optimized Arduino Uno (ATmega328P) flight controller and a comprehensive Proteus virtual simulation testbed.
The Python ground station queries real-world geospatial street and transit vector networks via OpenStreetMap (OSMNx and NetworkX), generates an interactive HTML flight trajectory and compiles limited sampled latitude/longitude coordinates into microcontroller headers. Onboard, the drone executes a non-blocking Finite State Machine (FSM) reading from a NEO-6M GPS receiver, angular kinematics from an MPU-6050 6-axis IMU, proximity alerts from an infrared (IR) obstacle sensor and orientation integrity from a ball tilt switch. Differential thrust motor control guides the quadcopter along sequential waypoints, initiates evasive maneuvers upon obstacle detection, holds at the target destination and autonomously executes a Return-to-Home (RTH) reverse landing sequence.
•	Core Objective 1: Develop an automated GIS route-planning pipeline that downloads real-world geographic coordinates, calculates shortest-path trajectories, down samples waypoints to strictly adhere to 8-bit MCU SRAM constraints (<2 KB) and exports structured C++ flight headers.
•	Core Objective 2: Architect a robust embedded flight controller firmware featuring an 8-state deterministic FSM, I2C IMU telemetry, real-time IR collision evasion and failsafe tilt-inversion motor shutdown.
The system schematic integrates digital, analog and communication peripherals mapped specifically to optimize ATmega328P port registers and prevent hardware timer/interrupt conflicts. The four DC motor drivers (M1 Top-Left, M2 Top-Right, M3 Bottom-Left, M4 Bottom-Right) are interfaced to digital output pins D2, D3, D4, and D5, respectively. Each pin drives the base of a 2N1711 NPN transistor through a 100Ω current-limiting resistor, pulling the motor cathode to ground when biased HIGH.
The MPU-6050 IMU communicates over the standard I2C hardware bus using analog pins A4 (Serial Data Line, SDA) and A5 (Serial Clock Line, SCL) operating at 100 kHz with 3.3V/5V pull-ups. The NEO-6M GPS module connects via SoftwareSerial on digital pin D8 (MCU RX, receiving from GPS TX) and D9 (MCU TX to GPS RX) at 9600 baud. The forward-facing IR obstacle sensor is mapped to analog input A3, allowing analog threshold calibration. The ball tilt switch connects digital pin D6 directly to Ground; utilizing the internal ATmega328P pull-up resistor (INPUT_PULLUP), D6 reads logic HIGH during normal upright flight and drops to LOW upon airframe inversion.
Pin Mapping & Hardware Interface Table:
•	Digital Pin D0 (RXD): Hardware UART Receive (Virtual Terminal / HC-05 Bluetooth TXD)
•	Digital Pin D1 (TXD): Hardware UART Transmit (Virtual Terminal / HC-05 Bluetooth RXD)
•	Digital Pin D2: Motor 1 (Top-Left, Q3 Base via 10 kΩ)
•	Digital Pin D3: Motor 2 (Top-Right, Q1 Base via 10 kΩ)
•	Digital Pin D4: Motor 3 (Bottom-Left, Q4 Base via 10 kΩ)
•	Digital Pin D5: Motor 4 (Bottom-Right, Q2 Base via 10 kΩ)
•	Digital Pin D6: Tilt Switch Sensor Signal (Internal Pull-Up, Active-LOW to GND)
•	Digital Pin D8: GPS SoftwareSerial RX (Connected to GPS Module TXD)
•	Digital Pin D9: GPS SoftwareSerial TX (Connected to GPS Module RXD)
•	Analog Pin A3: Analog IR Proximity Sensor Output (Analog Voltage Comparator)
•	Analog Pin A4: I2C SDA (MPU-6050 Motion Sensor Data Line)
•	Analog Pin A5: I2C SCL (MPU-6050 Motion Sensor Clock Line)

 Firmware, Software & Control Logic
The software architecture is decoupled into an offline Python geospatial planner and an embedded C++ flight executive running on the Arduino Uno.
•	Software Stack / IDE: Python 3.13 (OSMNx 2.x, NetworkX, Folium, Shutup, PySerial), Arduino IDE 2.x / AVR-GCC, Proteus Design Suite 8.10/8.16 Professional (VSM Simulation).
•	Libraries: <Wire.h> (I2C communication), <TinyGPS++.h> (NMEA sentence decoding, Haversine spherical distance, and Great Circle bearing), <SoftwareSerial.h> (auxiliary GPS UART).
•	Control Logic & State Machine: The embedded executive operates on a non-blocking 700ms control interval orchestrated by millis() scheduling. The core logic executes a deterministic 8-state Finite State Machine:
1.	STATE_IDLE: Motors halted; listens for incoming ASCII commands (START, STOP, LIST, RTH).
2.	STATE_NAVIGATING: Computes Haversine distance and bearing to target waypoint. 
3.	STATE_AVOIDING_OBSTACLE: When IR sensor reading exceeds the obstacle threshold, the FSM initiates an evasive starboard maneuver, steering away from the obstacle until path clearance is confirmed.
4.	STATE_WAITING_AT_DESTINATION: Upon reaching the final mission coordinate within a 5.0m arrival radius, motors disengage for a 5000ms loiter/payload hold.
5.	STATE_RETURNING (RTH): Automatically reverses the waypoint index, navigating the quadcopter autonomously back to launch origin.
6.	STATE_EMERGENCY_HALT: Interrupt-level safety override; cuts all 4 motor pins immediately if ball tilt switch closes to GND or pitch/roll exceeds safety boundaries.


 


