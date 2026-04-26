#include "robot_logic.h"
#include "types.h"
#include "config.h"
#include "sensors.h"
#include "motors.h"
#include "StartStop.h"

static bool inSharpTurnMode = false;
static int sharpTurnDir = 0; 
static uint32_t sharpTurnEntryTime = 0;
static float smoothSpeed = BASE_SPEED_MAPPING;
static float lastSavedDist = 0;

std::vector<EnhancedRoutePoint> route;

void initLogic() {
    route.reserve(1000);
}

void detectSharpTurnEntry(uint8_t mask, float error) {
    bool leftSide = (mask & 0b11000000);
    bool rightSide = (mask & 0b00000011);
    bool midSensors = (mask & MID_SENSOR_MASK);

    if ((leftSide || rightSide) && midSensors && !inSharpTurnMode) {
        inSharpTurnMode = true;
        sharpTurnDir = leftSide ? -1 : 1;
        sharpTurnEntryTime = millis();
        route.push_back({currentSensors.distanceDriven, (sharpTurnDir == -1 ? SHARP_LEFT : SHARP_RIGHT), BASE_SPEED_TURNS, error, true, false});
    }
}

void handleSharpTurnInFlight() {
    if (millis() - sharpTurnEntryTime > 800) { inSharpTurnMode = false; return; }

    if (sharpTurnDir == -1) setMotorSpeed(-BASE_SPEED_TURNS, BASE_SPEED_TURNS);
    else setMotorSpeed(BASE_SPEED_TURNS, -BASE_SPEED_TURNS);

    if (currentSensors.sensorMask & MID_SENSOR_MASK) {
        inSharpTurnMode = false;
        route.push_back({currentSensors.distanceDriven, STRAIGHT, BASE_SPEED_TURNS, 0, false, true});
    }
}

void executePID(float targetSpeed) {
    static float lastError = 0;
    float error = currentSensors.linePosition;
    float P = KP * error;
    float D = KD * (error - lastError);
    lastError = error;
    setMotorSpeed(targetSpeed + P + D, targetSpeed - (P + D));
}

void updateLogic(RobotState &currentState) {
    if (currentSensors.distance < DISTANCE_THRESHOLD) { stopMotors(); return; }

    FinishState fStatus = checkFinishStatus();
    
    // Intersectie afhandeling (Linkerhand regel)
    if (fStatus == INTERSECTION_DETECTED) {
        sharpTurnDir = -1; // Forceer links
        inSharpTurnMode = true;
        sharpTurnEntryTime = millis();
        return;
    }

    if (fStatus == FINISH_CONFIRMED) { currentState = FINISHED; return; }

    if (inSharpTurnMode) { handleSharpTurnInFlight(); return; }

    if (currentState == MAPPING) {
        if (currentSensors.lineDetected) {
            detectSharpTurnEntry(currentSensors.sensorMask, currentSensors.linePosition);
            executePID(BASE_SPEED_MAPPING);
            if (currentSensors.distanceDriven - lastSavedDist >= SAVE_POINT_INTERVAL) {
                route.push_back({currentSensors.distanceDriven, STRAIGHT, BASE_SPEED_MAPPING, currentSensors.linePosition, false, false});
                lastSavedDist = currentSensors.distanceDriven;
            }
        } else { setMotorSpeed(-60, 60); }
    } 
    else if (currentState == SPEEDRUN) {
        // Simpele versie van lookahead voor nu
        executePID(MAX_SPEED);
    }
}