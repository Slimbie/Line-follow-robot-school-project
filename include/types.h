#ifndef TYPES_H
#define TYPES_H

#include <stdint.h>
#include <vector>

// Hoofd states van de robot
enum RobotState {
    IDLE,
    COUNTDOWN,
    MAPPING,
    SPEEDRUN,
    FINISHED,
    DATA_DUMP
};

// Bocht types voor robuuste opslag
enum TurnType {
    STRAIGHT,
    SHARP_LEFT,
    SHARP_RIGHT,
    TURN_90_REGULAR
};

// Start/Stop en Kruispunt detectie
enum FinishState {
    NOT_FINISH,
    CHECKING_FINISH,
    FINISH_CONFIRMED,
    INTERSECTION_DETECTED
};

// Het nieuwe, geavanceerde route punt
struct EnhancedRoutePoint {
    float distance;
    TurnType turnType;
    uint8_t recommendedSpeed;
    float errorAtPoint; // Lijnpositie of error
    bool isSharpTurnEntry;
    bool isSharpTurnExit;
};

// Sensor struct (zoals je al gebruikte)
struct SensorData {
    bool lineDetected;
    uint8_t sensorMask;
    float linePosition;
    float distanceDriven;
    float distance; // Ultrasoon
    int32_t leftTicks;
    int32_t rightTicks;
    int irValues[8];
};

extern volatile SensorData currentSensors; // Globale sensor data

#endif