#include "robot_logic.h"
#include "Config.h"
#include "Sensors.h" // Voor currentSensors indien daar gedeclareerd
#include "Motors.h"
#include "StartStop.h"
#include <Arduino.h>

static bool inSharpTurnMode = false;
static int sharpTurnDir = 0; 
static uint32_t sharpTurnEntryTime = 0;
static float smoothSpeed = BASE_SPEED_MAPPING;
static float lastSavedDist = 0;

std::vector<EnhancedRoutePoint> route;

void initLogic() {
    route.reserve(1000);
}

void resetLogic() {
    route.clear();
    inSharpTurnMode = false;
    smoothSpeed = BASE_SPEED_MAPPING;
    lastSavedDist = 0;
}

void executePID(float targetSpeed) {
    static float lastError = 0;
    float error = currentSensors.linePosition;
    // Zorg dat KP en KD in Config.h staan!
    float P = KP * error;
    float D = KD * (error - lastError);
    lastError = error;
    
    setMotorSpeed(targetSpeed + P + D, targetSpeed - (P + D));
}

void detectSharpTurnEntry(uint8_t mask, float error) {
    bool leftSide = (mask & 0b11000000);
    bool rightSide = (mask & 0b00000011);
    bool midSensors = (mask & MID_SENSOR_MASK);

    // Entry: we zien flank en mid, en we zitten nog niet in de bocht
    if ((leftSide || rightSide) && midSensors && !inSharpTurnMode) {
        inSharpTurnMode = true;
        sharpTurnDir = leftSide ? -1 : 1;
        sharpTurnEntryTime = millis();
        
        // Opslaan in route
        route.push_back(EnhancedRoutePoint{
            currentSensors.distanceDriven, 
            (sharpTurnDir == -1 ? SHARP_LEFT : SHARP_RIGHT), 
            (uint8_t)BASE_SPEED_TURNS, 
            error, 
            true, 
            false
        });
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
        route.push_back(EnhancedRoutePoint{
            currentSensors.distanceDriven, 
            STRAIGHT, 
            (uint8_t)BASE_SPEED_TURNS, 
            0, 
            false, 
            true
        });
    }
}

void handleMapping(RobotState &currentState) {
    if (currentSensors.lineDetected) {
        detectSharpTurnEntry(currentSensors.sensorMask, currentSensors.linePosition);
        executePID(BASE_SPEED_MAPPING);
        
        // Sla periodiek rechte stukken op
        if (currentSensors.distanceDriven - lastSavedDist >= SAVE_POINT_INTERVAL) {
            route.push_back(EnhancedRoutePoint{
                currentSensors.distanceDriven, STRAIGHT, (uint8_t)BASE_SPEED_MAPPING, currentSensors.linePosition, false, false
            });
            lastSavedDist = currentSensors.distanceDriven;
        }
    } else {
        // Als we écht de lijn kwijt zijn (en niet in een geregistreerde scherpe bocht zitten)
        setMotorSpeed(-60, 60); // Voorzichtig zoeken
    }
}

void handleSpeedrun(RobotState &currentState) {
    // 1. Lookahead Logica (30 cm)
    float lookaheadDist = currentSensors.distanceDriven + 30.0;
    float targetSpeed = MAX_SPEED;

    for (const auto& pt : route) {
        if (pt.distance > currentSensors.distanceDriven && pt.distance <= lookaheadDist) {
            if (pt.turnType == SHARP_LEFT || pt.turnType == SHARP_RIGHT) {
                targetSpeed = pt.recommendedSpeed; // Anticipeer op de bocht!
                break; // We hebben de dichtstbijzijnde aankomende bocht gevonden
            }
        }
    }

    // 2. Smooth Speed Controle (Hard remmen, zacht optrekken)
    if (smoothSpeed > targetSpeed) {
        smoothSpeed -= 3.5; // Hard in de ankers
        if (smoothSpeed < targetSpeed) smoothSpeed = targetSpeed;
    } else if (smoothSpeed < targetSpeed) {
        smoothSpeed += 1.5; // Geleidelijk accelereren
        if (smoothSpeed > targetSpeed) smoothSpeed = targetSpeed;
    }

    // 3. Sturen
    if (currentSensors.lineDetected) {
        detectSharpTurnEntry(currentSensors.sensorMask, currentSensors.linePosition);
        executePID(smoothSpeed);
    } else {
        // Back-up als hij toch de lijn even verliest
        executePID(BASE_SPEED_TURNS); 
    }
}

void updateLogic(RobotState &currentState) {
    // Obstakel check
    if (currentSensors.distance < 15.0) { // DISTANCE_THRESHOLD
        stopMotors(); 
        return; 
    }

    FinishState fStatus = checkFinishStatus();
    
    if (fStatus == INTERSECTION_DETECTED) {
        sharpTurnDir = -1; // Forceer links op T-splitsing
        inSharpTurnMode = true;
        sharpTurnEntryTime = millis();
        return;
    }

    if (fStatus == FINISH_CONFIRMED) { 
        currentState = FINISHED; 
        stopMotors();
        return; 
    }

    // Als we in een bocht zitten, neem controle over!
    if (inSharpTurnMode) { 
        handleSharpTurnInFlight(); 
        return; 
    }

    // State routing
    if (currentState == MAPPING) {
        handleMapping(currentState);
    } else if (currentState == SPEEDRUN) {
        handleSpeedrun(currentState);
    }
}

void dumpEnhancedRoute(unsigned long travelTime) {
    Serial.println("\n======= ROUTE DUMP =======");
    Serial.printf("Tijd: %.2f sec\n", travelTime / 1000.0);
    Serial.println("Afstand | Type | Angle/Error | Advies Snelheid | Entry/Exit");
    for (const auto& pt : route) {
        Serial.printf("%.1f cm | %d | %.2f | %d | %d/%d\n", 
            pt.distance, pt.turnType, pt.errorAtPoint, pt.recommendedSpeed, pt.isSharpTurnEntry, pt.isSharpTurnExit);
    }
    Serial.println("=========================\n");
}