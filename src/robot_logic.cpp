#include "types.h"
#include "robot_logic.h"
#include "Config.h"
#include "Sensors.h"
#include "Motors.h"
#include "StartStop.h"
#include <Arduino.h>
#include <vector>

static bool inSharpTurnMode = false;
static int sharpTurnDir = 0;
static uint32_t sharpTurnEntryTime = 0;
static float smoothSpeed = BASE_SPEED_MAPPING;
static float lastSavedDist = 0;
static uint8_t lastMask = 0;
static float lastError = 0;
static bool probingIntersection = false;
static float probeStartDistance = 0;
static bool probeSawCenterLine = false;
static bool probeSawSideLine = false;

std::vector<EnhancedRoutePoint> route;
std::vector<Node> nodeMap;

void initLogic() {
    route.reserve(1000);
    nodeMap.reserve(100);
}

void resetLogic() {
    route.clear();
    nodeMap.clear();
    inSharpTurnMode = false;
    smoothSpeed = BASE_SPEED_MAPPING;
    lastSavedDist = 0;
    lastMask = 0;
    lastError = 0;
    probingIntersection = false;
    probeStartDistance = 0;
    probeSawCenterLine = false;
    probeSawSideLine = false;
}

inline bool allSensorsBlack(uint8_t mask) {
    return mask == 0xFF;
}

inline bool lineCentered(uint8_t mask) {
    return (mask & MID_SENSOR_MASK) != 0;
}

inline bool hasLeftSide(uint8_t mask) {
    return (mask & 0xE0) != 0;
}

inline bool hasRightSide(uint8_t mask) {
    return (mask & 0x07) != 0;
}

inline bool hasOuterLeft(uint8_t mask) {
    return (mask & 0xC0) != 0;
}

inline bool hasOuterRight(uint8_t mask) {
    return (mask & 0x03) != 0;
}

void saveIntersectionNode(JunctionAction action, TurnType turnType, bool leftDeadEnd, bool straightDeadEnd, bool rightDeadEnd) {
    Node node;
    node.distance = currentSensors.distanceDriven;
    node.action = action;
    node.turnType = turnType;
    node.leftDeadEnd = leftDeadEnd;
    node.straightDeadEnd = straightDeadEnd;
    node.rightDeadEnd = rightDeadEnd;
    node.nextJunctionDistance = 0.0f;
    nodeMap.push_back(node);
    Serial.printf("[NODE] %.1fcm action=%d type=%d Ldead=%d Sdead=%d Rdead=%d\n",
        currentSensors.distanceDriven,
        action,
        turnType,
        leftDeadEnd,
        straightDeadEnd,
        rightDeadEnd);
}

void saveRoutePoint(TurnType type, uint8_t speed, float error, bool entry, bool exit, bool deadEnd) {
    if (route.empty() || currentSensors.distanceDriven - lastSavedDist >= SAVE_POINT_INTERVAL || entry || exit || deadEnd) {
        EnhancedRoutePoint point;
        point.distance = currentSensors.distanceDriven;
        point.turnType = type;
        point.recommendedSpeed = speed;
        point.errorAtPoint = error;
        point.isSharpTurnEntry = entry;
        point.isSharpTurnExit = exit;
        point.isDeadEnd = deadEnd;
        route.push_back(point);
        lastSavedDist = currentSensors.distanceDriven;
    }
}

void prepareSpeedrun() {
    smoothSpeed = BASE_SPEED_MAPPING;
    lastError = 0;

    for (int i = 0; i < (int)nodeMap.size(); i++) {
        float currentDist = nodeMap[i].distance;
        float nextDist = 0.0f;
        for (int j = i + 1; j < (int)nodeMap.size(); j++) {
            if (nodeMap[j].action != TURN_180) {
                nextDist = nodeMap[j].distance - currentDist;
                break;
            }
        }
        nodeMap[i].nextJunctionDistance = nextDist;
    }

    Serial.printf("[PREP] Speedrun klaar met %d knooppunten\n", (int)nodeMap.size());
}

void startIntersectionProbe() {
    probingIntersection = true;
    probeStartDistance = currentSensors.distanceDriven;
    probeSawCenterLine = false;
    probeSawSideLine = false;
    setMotorSpeed(JUNCTION_YIELD_SPEED, JUNCTION_YIELD_SPEED);
    Serial.println("[PROBE] Start intersection probe");
}

void finishIntersectionProbe() {
    bool middleLast = lineCentered(lastMask);
    bool leftAvailable = hasLeftSide(lastMask);
    bool rightAvailable = hasRightSide(lastMask);
    bool isIntersect = probeSawCenterLine || probeSawSideLine;

    if (probeSawCenterLine) {
        saveIntersectionNode(TURN_LEFT, JUNCTION_T, !leftAvailable, false, !rightAvailable);
        saveRoutePoint(JUNCTION_T, JUNCTION_YIELD_SPEED, currentSensors.linePosition, false, false, false);
        Serial.println("[NODE] Kruispunt met centerline opnieuw gedetecteerd");
        setMotorSpeed(-BASE_SPEED_TURNS, BASE_SPEED_TURNS);
    } else if (!isIntersect && middleLast) {
        saveIntersectionNode(TURN_180, DEAD_END, true, true, true);
        saveRoutePoint(DEAD_END, DEAD_END_RECOVERY_SPEED, currentSensors.linePosition, false, false, true);
        Serial.println("[NODE] Doodlopend pad gedetecteerd");
        setMotorSpeed(-DEAD_END_RECOVERY_SPEED, DEAD_END_RECOVERY_SPEED);
    } else if (isIntersect) {
        saveIntersectionNode(TURN_LEFT, JUNCTION_T, !leftAvailable, false, !rightAvailable);
        saveRoutePoint(JUNCTION_T, JUNCTION_YIELD_SPEED, currentSensors.linePosition, false, false, false);
        Serial.println("[NODE] T-splitsing / kruispunt gedetecteerd");
        setMotorSpeed(-BASE_SPEED_TURNS, BASE_SPEED_TURNS);
    } else {
        saveIntersectionNode(TURN_LEFT, JUNCTION_T, false, false, true);
        saveRoutePoint(JUNCTION_T, JUNCTION_YIELD_SPEED, currentSensors.linePosition, false, false, false);
        Serial.println("[NODE] Linkerhandregel toegepast op vermoedelijke T-splitsing");
        setMotorSpeed(-BASE_SPEED_TURNS, BASE_SPEED_TURNS);
    }

    probingIntersection = false;
}

void updateIntersectionProbe() {
    if (!probingIntersection) return;

    if (currentSensors.lineDetected) {
        if (lineCentered(currentSensors.sensorMask)) {
            probeSawCenterLine = true;
        }
        if (hasLeftSide(currentSensors.sensorMask) || hasRightSide(currentSensors.sensorMask)) {
            probeSawSideLine = true;
        }
    }

    if (currentSensors.distanceDriven - probeStartDistance >= INTERSECTION_PROBE_CM) {
        finishIntersectionProbe();
    }
}

void executePID(float targetSpeed) {
    float error = currentSensors.linePosition;
    // Zorg dat KP en KD in Config.h staan!
    float P = KP * error;
    float D = KD * (error - lastError);
    lastError = error;
    
    setMotorSpeed(targetSpeed + P + D, targetSpeed - (P + D));
}

void detectSharpTurnEntry(uint8_t mask, float error) {
    bool leftSide = hasOuterLeft(mask);
    bool rightSide = hasOuterRight(mask);
    bool midSensors = lineCentered(mask);

    if ((leftSide || rightSide) && midSensors && !inSharpTurnMode) {
        inSharpTurnMode = true;
        sharpTurnDir = leftSide ? -1 : 1;
        sharpTurnEntryTime = millis();
        saveRoutePoint(leftSide ? SHARP_LEFT : SHARP_RIGHT, BASE_SPEED_TURNS, error, true, false, false);
        Serial.printf("[SHARP] Entry at %.1f cm dir=%d\n", currentSensors.distanceDriven, sharpTurnDir);
    }
}

void handleSharpTurnInFlight() {
    // Timeout beveiliging
    if (millis() - sharpTurnEntryTime > 800) { 
        inSharpTurnMode = false; 
        return; 
    }

    // Blijf draaien in de laatste bekende richting als we niks zien
    if (sharpTurnDir == -1) {
        setMotorSpeed(-BASE_SPEED_TURNS, BASE_SPEED_TURNS);
    } else {
        setMotorSpeed(BASE_SPEED_TURNS, -BASE_SPEED_TURNS);
    }

    // Exit: We zien de lijn weer met de middelste sensoren!
    if (currentSensors.sensorMask & MID_SENSOR_MASK) {
        inSharpTurnMode = false;
        saveRoutePoint(STRAIGHT, BASE_SPEED_TURNS, 0.0f, false, true, false);
    }
}

void handleMapping(RobotState &currentState) {
    if (currentSensors.distance < DISTANCE_THRESHOLD) {
        stopMotors();
        Serial.printf("[OBSTACLE] Afstand %.1f cm - stoppen\n", currentSensors.distance);
        return;
    }

    if (probingIntersection) {
        updateIntersectionProbe();
        lastMask = currentSensors.sensorMask;
        return;
    }

    if (allSensorsBlack(currentSensors.sensorMask)) {
        startIntersectionProbe();
        lastMask = currentSensors.sensorMask;
        return;
    }

    if (!currentSensors.lineDetected) {
        if (hasOuterLeft(lastMask)) {
            setMotorSpeed(-SEARCH_HARD_TURN_SPEED, SEARCH_HARD_TURN_SPEED);
        } else if (hasOuterRight(lastMask)) {
            setMotorSpeed(SEARCH_HARD_TURN_SPEED, -SEARCH_HARD_TURN_SPEED);
        } else {
            setMotorSpeed(BASE_SPEED_MAPPING / 2, -BASE_SPEED_MAPPING / 2);
        }
        lastMask = currentSensors.sensorMask;
        return;
    }

    detectSharpTurnEntry(currentSensors.sensorMask, currentSensors.linePosition);
    executePID(BASE_SPEED_MAPPING);
    saveRoutePoint(STRAIGHT, BASE_SPEED_MAPPING, currentSensors.linePosition, false, false, false);
    lastMask = currentSensors.sensorMask;
}

Node* findNextNode() {
    for (auto &node : nodeMap) {
        if (node.distance > currentSensors.distanceDriven + 1.0f) {
            return &node;
        }
    }
    return nullptr;
}

void handleSpeedrun(RobotState &currentState) {
    if (currentSensors.distance < DISTANCE_THRESHOLD) {
        stopMotors();
        Serial.printf("[OBSTACLE] Speedrun onderbroken op %.1f cm\n", currentSensors.distance);
        return;
    }

    if (probingIntersection) {
        updateIntersectionProbe();
        return;
    }

    if (inSharpTurnMode) {
        handleSharpTurnInFlight();
        return;
    }

    Node* nextNode = findNextNode();
    float targetSpeed = MAX_SPEED;

    if (nextNode != nullptr && nextNode->nextJunctionDistance > 0.0f && nextNode->nextJunctionDistance < LOOKAHEAD_DISTANCE) {
        targetSpeed = max(70, MAX_SPEED - 40);
    }

    if (nextNode != nullptr && allSensorsBlack(currentSensors.sensorMask)) {
        if (nextNode->action == TURN_LEFT) {
            setMotorSpeed(-BASE_SPEED_TURNS, BASE_SPEED_TURNS);
            return;
        } else if (nextNode->action == TURN_RIGHT) {
            setMotorSpeed(BASE_SPEED_TURNS, -BASE_SPEED_TURNS);
            return;
        } else if (nextNode->action == TURN_180) {
            setMotorSpeed(-DEAD_END_RECOVERY_SPEED, DEAD_END_RECOVERY_SPEED);
            return;
        }
    }

    if (smoothSpeed > targetSpeed) {
        smoothSpeed -= 3.5;
        if (smoothSpeed < targetSpeed) smoothSpeed = targetSpeed;
    } else if (smoothSpeed < targetSpeed) {
        smoothSpeed += 1.5;
        if (smoothSpeed > targetSpeed) smoothSpeed = targetSpeed;
    }

    if (currentSensors.lineDetected) {
        detectSharpTurnEntry(currentSensors.sensorMask, currentSensors.linePosition);
        executePID(smoothSpeed);
    } else {
        float recovery = lastError * 3.0f;
        setMotorSpeed(
            constrain((int)(smoothSpeed + recovery), 0, MAX_SPEED),
            constrain((int)(smoothSpeed - recovery), 0, MAX_SPEED)
        );
    }
}

Mode updateLogic(RobotState &currentState) {
    // Obstakel check
    if (currentSensors.distance < 15.0) { // DISTANCE_THRESHOLD
        stopMotors();
        return currentState.currentMode;
    }

    FinishState fStatus = checkFinishStatus();
    
    if (fStatus == INTERSECTION_DETECTED) {
        sharpTurnDir = -1; // Forceer links op T-splitsing
        inSharpTurnMode = true;
        sharpTurnEntryTime = millis();
        return currentState.currentMode;
    }

    if (fStatus == FINISH_CONFIRMED) {
        currentState.currentMode = FINISHED;
        stopMotors();
        return currentState.currentMode;
    }

    // Als we in een bocht zitten, neem controle over!
    if (inSharpTurnMode) {
        handleSharpTurnInFlight();
        return currentState.currentMode;
    }

    // State routing
    if (currentState.currentMode == MAPPING) {
        handleMapping(currentState);
    } else if (currentState.currentMode == SPEEDRUN) {
        handleSpeedrun(currentState);
    }
    return currentState.currentMode;
}

void dumpEnhancedRoute(unsigned long travelTime) {
    Serial.println("\n======= ROUTE DUMP =======");
    Serial.printf("Tijd: %.2f sec\n", travelTime / 1000.0);
    Serial.println("Afstand | Type | Angle/Error | Advies Snelheid | Entry/Exit/DeadEnd");
    for (const auto& pt : route) {
        Serial.printf("%.1f cm | %d | %.2f | %d | %d/%d/%d\n", 
            pt.distance,
            pt.turnType,
            pt.errorAtPoint,
            pt.recommendedSpeed,
            pt.isSharpTurnEntry,
            pt.isSharpTurnExit,
            pt.isDeadEnd);
    }

    if (!nodeMap.empty()) {
        Serial.println("\n======= NODE MAP =======");
        Serial.println("Dist | Action | Junction | DeadEnds L/S/R | NextJunctionDist");
        for (const auto& node : nodeMap) {
            Serial.printf("%.1f | %d | %d | %d/%d/%d | %.1f\n",
                node.distance,
                node.action,
                node.turnType,
                node.leftDeadEnd,
                node.straightDeadEnd,
                node.rightDeadEnd,
                node.nextJunctionDistance);
        }
    }
    Serial.println("=========================\n");
}