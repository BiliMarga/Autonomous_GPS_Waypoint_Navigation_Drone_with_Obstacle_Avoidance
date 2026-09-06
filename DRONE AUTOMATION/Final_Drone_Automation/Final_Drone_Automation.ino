// ============================================================================
// DRONE WAYPOINT AUTONOMOUS NAVIGATION & OBSTACLE AVOIDANCE
// OPTIMIZED FOR ARDUINO UNO (SRAM-SAFE VERSION)
// ============================================================================

#include <Wire.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include "waypoints.h" // Auto-generated from main.py

TinyGPSPlus gps;

// SoftwareSerial for GPS (TXD -> Pin 8, RXD -> Pin 9)
#define GPS_RX_PIN 8
#define GPS_TX_PIN 9
SoftwareSerial gpsSerial(GPS_RX_PIN, GPS_TX_PIN);
#define GPS_BAUD 9600

#define MPU6050_ADDR 0x68
float accelX = 0, accelY = 0, accelZ = 0;
float gyroX = 0, gyroY = 0, gyroZ = 0;

// Sensor Pin Definitions
#define IR_SENSOR_PIN A3
#define TILT_SWITCH_PIN 6

// Sensor Thresholds
#define IR_OBSTACLE_THRESHOLD 400
int irSensorValue = 0;
bool isTiltDetected = false;

// Motor Driver Transistor Base Pins (Pins 2, 3, 4, 5)
const uint8_t MOTOR_1 = 2; // Top-Left (Q3)
const uint8_t MOTOR_2 = 3; // Top-Right (Q1)
const uint8_t MOTOR_3 = 4; // Bottom-Left (Q4)
const uint8_t MOTOR_4 = 5; // Bottom-Right (Q2)

// ======================================================
// WAYPOINTS & NAVIGATION (RAM OPTIMIZED)
// ======================================================
// Restricted to under 50 waypoints to maintain strict SRAM safety (~400 bytes for 25 waypoints)
const uint8_t MAX_WAYPOINTS = 25; 
#define ARRIVAL_RADIUS_METERS 5.0

Waypoint path[MAX_WAYPOINTS];
uint8_t totalWaypoints = 0;
int8_t currentTargetIdx = 0;

// Spatial State
double currentLat = 0.0;
double currentLon = 0.0;
double currentHeading = 0.0;
double targetBearing = 0.0;
double distanceToTarget = 0.0;

// System States
enum DroneState : uint8_t {
    STATE_IDLE,
    STATE_NAVIGATING,
    STATE_TURNING,
    STATE_AVOIDING_OBSTACLE,
    STATE_WAITING_AT_DESTINATION,
    STATE_RETURNING,              
    STATE_MISSION_COMPLETE
};

DroneState droneState = STATE_IDLE;
unsigned long destinationArrivalTime = 0;
const unsigned long HOLD_TIME_MS = 5000;
bool isReturningHome = false;
uint8_t avoidanceStepCount = 0;

// Memory Diagnostic Utility
int getFreeRam() {
    extern int __heap_start, *__brkval;
    int v;
    return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}

void initMotors() {
    pinMode(MOTOR_1, OUTPUT);
    pinMode(MOTOR_2, OUTPUT);
    pinMode(MOTOR_3, OUTPUT);
    pinMode(MOTOR_4, OUTPUT);
    stopMotors();
}

void stopMotors() {
    digitalWrite(MOTOR_1, LOW);
    digitalWrite(MOTOR_2, LOW);
    digitalWrite(MOTOR_3, LOW);
    digitalWrite(MOTOR_4, LOW);
}

void moveForward() {
    digitalWrite(MOTOR_1, HIGH);
    digitalWrite(MOTOR_2, HIGH);
    digitalWrite(MOTOR_3, HIGH);
    digitalWrite(MOTOR_4, HIGH);
}

void turnLeft() {
    digitalWrite(MOTOR_1, LOW);
    digitalWrite(MOTOR_2, HIGH);
    digitalWrite(MOTOR_3, LOW);
    digitalWrite(MOTOR_4, HIGH);
}

void turnRight() {
    digitalWrite(MOTOR_1, HIGH);
    digitalWrite(MOTOR_2, LOW);
    digitalWrite(MOTOR_3, HIGH);
    digitalWrite(MOTOR_4, LOW);
}

void setupMPU6050() {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x6B);
    Wire.write(0x00);
    Wire.endTransmission(true);

    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x1C);
    Wire.write(0x00);
    Wire.endTransmission(true);

    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x1B);
    Wire.write(0x00);
    Wire.endTransmission(true);
}

void readMPU6050() {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x3B);
    if (Wire.endTransmission(false) != 0) return;
    
    Wire.requestFrom(MPU6050_ADDR, (uint8_t)14, (uint8_t)true);

    if (Wire.available() >= 14) {
        int16_t ax = Wire.read() << 8 | Wire.read();
        int16_t ay = Wire.read() << 8 | Wire.read();
        int16_t az = Wire.read() << 8 | Wire.read();
        Wire.read(); Wire.read();
        int16_t gx = Wire.read() << 8 | Wire.read();
        int16_t gy = Wire.read() << 8 | Wire.read();
        int16_t gz = Wire.read() << 8 | Wire.read();

        accelX = ax / 16384.0;
        accelY = ay / 16384.0;
        accelZ = az / 16384.0;
        gyroX  = gx / 131.0;
        gyroY  = gy / 131.0;
        gyroZ  = gz / 131.0;
    }
}

int readIRSensor() {
    return analogRead(IR_SENSOR_PIN);
}

bool readTiltSwitch() {
    return digitalRead(TILT_SWITCH_PIN) == HIGH;
}

void loadDefaultWaypoints() {
    totalWaypoints = (TOTAL_WAYPOINTS <= MAX_WAYPOINTS) ? TOTAL_WAYPOINTS : MAX_WAYPOINTS;

    for (uint8_t i = 0; i < totalWaypoints; i++) {
        path[i] = pathHeader[i];
    }

    if (totalWaypoints > 0) {
        currentLat = path[0].lat;
        currentLon = path[0].lon;
        currentTargetIdx = 0;
        isReturningHome = false;

        Serial.print(F(">>> [MISSION LOADED] "));
        Serial.print(totalWaypoints);
        Serial.println(F(" waypoints imported."));
    } else {
        Serial.println(F(">>> [ERROR] No waypoints found!"));
    }
}

void displayAllWaypoints() {
    Serial.println();
    Serial.println(F("=================================================================="));
    Serial.println(F("                    MISSION FLIGHT WAYPOINTS                      "));
    Serial.println(F("=================================================================="));
    Serial.println(F("WP # |   Latitude   |  Longitude  | Leg Dist | Bearing | Cardinal  "));
    Serial.println(F("-----+-------------+-------------+----------+---------+-----------"));

    double totalDist = 0.0;

    for (uint8_t i = 0; i < totalWaypoints; i++) {
        Serial.print(F(" "));
        if (i < 10) Serial.print(F("0"));
        Serial.print(i);
        Serial.print(F("  | "));
        Serial.print(path[i].lat, 6);
        Serial.print(F(" | "));
        Serial.print(path[i].lon, 6);
        Serial.print(F(" | "));

        if (i == 0) {
            Serial.println(F("  START   |   ---   |   LAUNCH  "));
        } else {
            double legDist = TinyGPSPlus::distanceBetween(
                path[i - 1].lat, path[i - 1].lon,
                path[i].lat, path[i].lon
            );
            double legBearing = TinyGPSPlus::courseTo(
                path[i - 1].lat, path[i - 1].lon,
                path[i].lat, path[i].lon
            );
            totalDist += legDist;

            Serial.print(legDist, 1);
            Serial.print(F("m | "));
            Serial.print(legBearing, 1);
            Serial.print(F(" deg| "));
            Serial.println(TinyGPSPlus::cardinal(legBearing));
        }
    }

    Serial.println(F("=================================================================="));
}

void displayTelemetry() {
    Serial.println(F("------------------------------------------------------------------"));
    Serial.print(F("STATUS: "));
    switch (droneState) {
        case STATE_IDLE: Serial.print(F("IDLE")); break;
        case STATE_NAVIGATING: Serial.print(F("NAVIGATING")); break;
        case STATE_TURNING: Serial.print(F("TURNING")); break;
        case STATE_AVOIDING_OBSTACLE: Serial.print(F("AVOIDING OBSTACLE")); break;
        case STATE_WAITING_AT_DESTINATION: Serial.print(F("HOLDING AT DESTINATION")); break;
        case STATE_RETURNING: Serial.print(F("RETURNING HOME")); break;
        case STATE_MISSION_COMPLETE: Serial.print(F("MISSION COMPLETE")); break;
    }

    Serial.print(F(" | FREE RAM: "));
    Serial.print(getFreeRam());
    Serial.println(F(" bytes"));

    // --- TOTAL MISSION DISTANCE DISPLAY ---
    double totalMissionDist = 0.0;
    for (uint8_t i = 1; i < totalWaypoints; i++) {
        totalMissionDist += TinyGPSPlus::distanceBetween(
            path[i - 1].lat, path[i - 1].lon,
            path[i].lat, path[i].lon
        );
    }
    Serial.print(F("TOTAL MISSION DISTANCE: "));
    Serial.print(totalMissionDist, 2);
    Serial.print(F(" meters ("));
    Serial.print(totalMissionDist / 1000.0, 3);
    Serial.println(F(" km)"));

    // --- POSITION & HEADING ---
    Serial.print(F("POS: ["));
    Serial.print(currentLat, 6);
    Serial.print(F(", "));
    Serial.print(currentLon, 6);
    Serial.print(F("] | HEADING: "));
    Serial.print(currentHeading, 1);
    Serial.println(F(" deg"));

    // --- MPU6050 ACCEL & GYRO SENSOR DATA ---
    Serial.print(F("ACCEL (g): X="));
    Serial.print(accelX, 2);
    Serial.print(F(" | Y="));
    Serial.print(accelY, 2);
    Serial.print(F(" | Z="));
    Serial.println(accelZ, 2);

    Serial.print(F("GYRO (deg/s): X="));
    Serial.print(gyroX, 1);
    Serial.print(F(" | Y="));
    Serial.print(gyroY, 1);
    Serial.print(F(" | Z="));
    Serial.println(gyroZ, 1);

    if (droneState != STATE_MISSION_COMPLETE && currentTargetIdx < totalWaypoints && currentTargetIdx >= 0) {
        Serial.print(F("TARGET WP ["));
        Serial.print(currentTargetIdx);
        Serial.print(F("] | DIST: "));
        Serial.print(distanceToTarget, 1);
        Serial.print(F("m | BEARING: "));
        Serial.print(targetBearing, 1);
        Serial.println(F(" deg"));
    }

    Serial.print(F("IR VALUE: "));
    Serial.print(irSensorValue);
    if (irSensorValue >= IR_OBSTACLE_THRESHOLD) {
        Serial.println(F(" *** [OBSTACLE DETECTED!] ***"));
    } else {
        Serial.println(F(" [PATH CLEAR]"));
    }

    Serial.print(F("TILT: "));
    Serial.println(isTiltDetected ? F("TRIGGERED") : F("NORMAL"));
    Serial.println(F("------------------------------------------------------------------"));
}

void executeNavigationStep() {
    if ((!isReturningHome && currentTargetIdx >= totalWaypoints) || 
        (isReturningHome && currentTargetIdx < 0)) {
        
        if (!isReturningHome) {
            droneState = STATE_WAITING_AT_DESTINATION;
            stopMotors();
            destinationArrivalTime = millis();
            Serial.println(F(">>> DESTINATION REACHED! HOLDING... <<<"));
            return;
        } else {
            droneState = STATE_MISSION_COMPLETE;
            stopMotors();
            Serial.println(F(">>> RTH COMPLETED! DRONE LANDED. <<<"));
            currentLat = path[0].lat;
            currentLon = path[0].lon;
            currentHeading = 0.0;
            currentTargetIdx = 0;
            isReturningHome = false;
            return;
        }
    }

    Waypoint target = path[currentTargetIdx];

    distanceToTarget = TinyGPSPlus::distanceBetween(currentLat, currentLon, target.lat, target.lon);
    targetBearing = TinyGPSPlus::courseTo(currentLat, currentLon, target.lat, target.lon);

    if (distanceToTarget <= ARRIVAL_RADIUS_METERS) {
        Serial.print(F(">>> REACHED WAYPOINT #"));
        Serial.println(currentTargetIdx);

        if (isReturningHome) currentTargetIdx--;
        else currentTargetIdx++;
        return;
    }

    double headingError = targetBearing - currentHeading;
    while (headingError > 180.0) headingError -= 360.0;
    while (headingError < -180.0) headingError += 360.0;

    if (headingError > 15.0) {
        droneState = STATE_TURNING;
        turnRight();
        currentHeading += 10.0;
        if (currentHeading >= 360.0) currentHeading -= 360.0;
    } else if (headingError < -15.0) {
        droneState = STATE_TURNING;
        turnLeft();
        currentHeading -= 10.0;
        if (currentHeading < 0.0) currentHeading += 360.0;
    } else {
        droneState = (isReturningHome) ? STATE_RETURNING : STATE_NAVIGATING;
        moveForward();

        double stepRatio = 0.15; 
        currentLat += (target.lat - currentLat) * stepRatio;
        currentLon += (target.lon - currentLon) * stepRatio;
    }
}

void executeObstacleAvoidance() {
    droneState = STATE_AVOIDING_OBSTACLE;
    avoidanceStepCount++;

    Serial.println(F(">>> EVASIVE ACTION: Steering RIGHT to clear obstacle..."));

    turnRight();
    currentHeading += 20.0;
    if (currentHeading >= 360.0) currentHeading -= 360.0;
    currentLon += 0.00003; 

    delay(200);
    irSensorValue = readIRSensor();

    if (irSensorValue < IR_OBSTACLE_THRESHOLD || avoidanceStepCount > 5) {
        Serial.println(F(">>> OBSTACLE CLEARED! Recalculating path..."));
        avoidanceStepCount = 0;
        droneState = STATE_NAVIGATING;
    }
}

// C-String Command Processor (Prevents RAM fragmentation from String class)
void processSerialCommands() {
    static char cmdBuf[32];
    static uint8_t bufIdx = 0;

    while (Serial.available() > 0) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
            if (bufIdx == 0) continue;
            cmdBuf[bufIdx] = '\0';

            if (strcmp(cmdBuf, "START") == 0 || strcmp(cmdBuf, "1") == 0) {
                if (totalWaypoints >= 2) {
                    currentTargetIdx = 1;
                    currentLat = path[0].lat;
                    currentLon = path[0].lon;
                    isReturningHome = false;
                    droneState = STATE_NAVIGATING;
                    Serial.println(F(">>> MISSION LAUNCHED!"));
                }
            } else if (strcmp(cmdBuf, "LIST") == 0 || strcmp(cmdBuf, "2") == 0) {
                displayAllWaypoints();
            } else if (strcmp(cmdBuf, "STOP") == 0 || strcmp(cmdBuf, "0") == 0) {
                droneState = STATE_IDLE;
                stopMotors();
                Serial.println(F(">>> DRONE HALTED."));
            }
            bufIdx = 0;
        } else if (bufIdx < sizeof(cmdBuf) - 1) {
            cmdBuf[bufIdx++] = c;
        }
    }
}

void setup() {
    // Hardware Serial on Pins 0 (RX) & 1 (TX) mapped directly to VLM / Virtual Terminal
    Serial.begin(9600);
    Wire.begin(); // MPU6050 defaulting to A4 (SDA) and A5 (SCL)
    Wire.setWireTimeout(3000, true);

    gpsSerial.begin(GPS_BAUD);
    setupMPU6050();

    pinMode(IR_SENSOR_PIN, INPUT);
    pinMode(TILT_SWITCH_PIN, INPUT);

    initMotors();
    loadDefaultWaypoints();
    displayAllWaypoints();

    if (totalWaypoints >= 2) {
        currentTargetIdx = 1;
        currentLat = path[0].lat;
        currentLon = path[0].lon;
        isReturningHome = false;
        droneState = STATE_NAVIGATING;
        Serial.println(F(">>> AUTO-LAUNCH: Navigating to WP #1!"));
    }
}

unsigned long lastStepTime = 0;

void loop() {
    processSerialCommands();

    if (millis() - lastStepTime >= 700) {
        lastStepTime = millis();

        while (gpsSerial.available() > 0) {
            gps.encode(gpsSerial.read());
        }
        if (gps.location.isValid() && gps.location.isUpdated()) {
            currentLat = gps.location.lat();
            currentLon = gps.location.lng();
        }

        readMPU6050();
        irSensorValue = readIRSensor();
        isTiltDetected = readTiltSwitch();

        if (droneState == STATE_WAITING_AT_DESTINATION) {
            if (millis() - destinationArrivalTime >= HOLD_TIME_MS) {
                isReturningHome = true;
                currentTargetIdx = totalWaypoints - 2;
                droneState = STATE_RETURNING;
                Serial.println(F(">>> RTH INITIATED! Returning home... <<<"));
            }
        } 
        else if (droneState != STATE_IDLE && droneState != STATE_MISSION_COMPLETE) {
            if (irSensorValue >= IR_OBSTACLE_THRESHOLD) {
                executeObstacleAvoidance();
            } else {
                executeNavigationStep();
            }
        }

        displayTelemetry();
    }
}