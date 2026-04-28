#ifndef LINE_FOLLOW_TYPES_H
#define LINE_FOLLOW_TYPES_H

#include <stdint.h>
#include <vector>

// ============================================================================
// MODE & STATUS ENUMS
// ============================================================================

enum Mode {
    READY,
    MAPPING,
    FINISHED,
    SPEEDRUN,
    DATADUMP
};

enum TurnType {
    STRAIGHT,
    SHARP_LEFT,
    SHARP_RIGHT,
    TURN_90_LEFT,
    TURN_90_RIGHT,
    JUNCTION_T,
    JUNCTION_PLUS,
    DEAD_END
};

enum JunctionAction {
    GO_STRAIGHT,
    TURN_LEFT,
    TURN_RIGHT,
    TURN_180
};

enum FinishState {
    NOT_FINISH,
    CHECKING_FINISH,
    FINISH_CONFIRMED,
    INTERSECTION_DETECTED
};

// ============================================================================
// ROUTE & MAPPING STRUCTS
// ============================================================================

struct EnhancedRoutePoint {
    float distance;
    TurnType turnType;
    uint8_t recommendedSpeed;
    float errorAtPoint;
    bool isSharpTurnEntry;
    bool isSharpTurnExit;
    bool isDeadEnd;
};

struct Node {
    float distance;
    JunctionAction action;
    TurnType turnType;
    bool leftDeadEnd;
    bool straightDeadEnd;
    bool rightDeadEnd;
    float nextJunctionDistance;
};

struct SensorData {
    bool lineDetected;
    uint8_t sensorMask;
    int irValues[8];
    float linePosition;
    float distanceDriven;
    float distance;
    int32_t leftTicks;
    int32_t rightTicks;
    uint32_t timestamp;
};

struct RobotState {
    Mode currentMode;
    bool isRunning;
    uint32_t lastStateChange;
    float totalDistance;
    int pointsRecorded;
};

extern volatile SensorData currentSensors;

#endif